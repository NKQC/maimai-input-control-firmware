#include "vcam_queue.h"

#include <atomic>

// 打开失败后的重试间隔(以帧计): 30fps 下约 0.5s。
static const int kRetryFrames = 15;

void Mai2VcamQueueReader::Close() {
    if (_view != nullptr) {
        UnmapViewOfFile(_view);
        _view = nullptr;
    }
    if (_map != nullptr) {
        CloseHandle(_map);
        _map = nullptr;
    }
}

unsigned int Mai2VcamQueueReader::_Load(unsigned int offset) const {
    // ★跨进程原子 acquire 读, 不是 volatile 读★
    // 生产者(Rust 侧 src/vcam/share.rs)用同宽度原子 release 写发布 state/sequence/tick;
    // 这里必须用配对的原子 acquire 读: volatile 只约束编译器, 不提供原子性与可见性契约。
    //
    // ★不能用 Interlocked* 做"原子读"★: 本视图以 FILE_MAP_READ 打开, InterlockedCompareExchange
    // 一类实现是 `lock cmpxchg`(读改写), 会对只读页写入而触发访问违例。std::atomic_ref 的
    // load 在 x86/x64 上是单条对齐 mov, 不写内存, 与 Rust 侧 AtomicU32::load 同语义。
    // 头部按 4 字节对齐(映射视图页对齐 + 字段偏移是 4 的倍数), 满足 atomic_ref 的对齐要求。
    unsigned int* cell =
        const_cast<unsigned int*>(reinterpret_cast<const unsigned int*>(_view + offset));
    return std::atomic_ref<unsigned int>(*cell).load(std::memory_order_acquire);
}

void Mai2VcamQueueReader::_Placeholder(BYTE* destination) {
    // NV12 全黑(BT.601 full range): Y=0, UV=128。与生产者对黑帧的编码一致,
    // 消费端在"生产者不在"和"生产者输出黑屏"两种情况下看到的画面完全相同。
    const size_t luma = (size_t)MAI2VCAM_WIDTH * MAI2VCAM_HEIGHT;
    memset(destination, 0, luma);
    memset(destination + luma, 128, (size_t)MAI2VCAM_FRAME_BYTES - luma);
}

bool Mai2VcamQueueReader::_Open() {
    if (_view != nullptr) {
        return true;
    }
    if (_retryCountdown > 0) {
        _retryCountdown--;
        return false;
    }
    _retryCountdown = kRetryFrames;
    HANDLE map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME);
    if (map == nullptr) {
        return false;
    }
    const BYTE* view = (const BYTE*)MapViewOfFile(map, FILE_MAP_READ, 0, 0, MAI2VCAM_MAP_BYTES);
    if (view == nullptr) {
        CloseHandle(map);
        return false;
    }
    // 映射实际大小必须覆盖整个布局: 生产者版本不一致时宁可当作没有生产者。
    MEMORY_BASIC_INFORMATION region = {};
    if (VirtualQuery(view, &region, sizeof(region)) == 0 || region.RegionSize < MAI2VCAM_MAP_BYTES) {
        UnmapViewOfFile(view);
        CloseHandle(map);
        return false;
    }
    _map = map;
    _view = view;
    return true;
}

bool Mai2VcamQueueReader::Read(BYTE* destination) {
    if (destination == nullptr) {
        return false;
    }
    if (!_Open()) {
        _Placeholder(destination);
        if (_lastLive || !_logged) {
            Mai2VcamLog("queue: 生产者不在(未找到共享队列) → 占位帧");
            _lastLive = false;
            _logged = true;
        }
        return false;
    }

    const bool headerOk = _Load(MAI2VCAM_OFF_MAGIC) == MAI2VCAM_MAGIC &&
                          _Load(MAI2VCAM_OFF_VERSION) == MAI2VCAM_LAYOUT_VERSION &&
                          _Load(MAI2VCAM_OFF_HEADER_BYTES) == MAI2VCAM_HEADER_BYTES &&
                          _Load(MAI2VCAM_OFF_WIDTH) == (unsigned int)MAI2VCAM_WIDTH &&
                          _Load(MAI2VCAM_OFF_HEIGHT) == (unsigned int)MAI2VCAM_HEIGHT &&
                          _Load(MAI2VCAM_OFF_FOURCC) == (unsigned int)MAKEFOURCC('N', 'V', '1', '2') &&
                          _Load(MAI2VCAM_OFF_SLOT_COUNT) == MAI2VCAM_SLOT_COUNT &&
                          _Load(MAI2VCAM_OFF_SLOT_BYTES) == (unsigned int)MAI2VCAM_FRAME_BYTES;
    const unsigned int state = headerOk ? _Load(MAI2VCAM_OFF_STATE) : MAI2VCAM_STATE_INVALID;
    // 心跳: 生产者每次发布都会刷新, 停滞超时即视为掉线(进程被杀不会改 state)。
    const unsigned int tick = headerOk ? _Load(MAI2VCAM_OFF_TICK_MS) : 0;
    const unsigned int now = GetTickCount();
    const bool alive = headerOk && state == MAI2VCAM_STATE_READY &&
                       (now - tick) < MAI2VCAM_HEARTBEAT_TIMEOUT_MS;
    if (!alive) {
        _Placeholder(destination);
        if (_lastLive || !_logged) {
            Mai2VcamLog("queue: 队列不可用(header=%d state=%u 心跳滞后=%ums) → 占位帧",
                        headerOk ? 1 : 0, state, headerOk ? (now - tick) : 0u);
            _lastLive = false;
            _logged = true;
        }
        return false;
    }

    // 三缓冲读取: 最多 3 次尝试拿到一个未被追写的槽。
    // 两次 sequence 都是 acquire 原子读(见 _Load), 像素拷贝夹在两者之间;
    // 额外的 atomic_thread_fence(acquire) 明确禁止把像素读重排到第一次 sequence 读之前,
    // 也禁止把它们推到第二次读之后 —— 撕裂判定完全依赖这个顺序。
    for (int attempt = 0; attempt < 3; attempt++) {
        const unsigned int first = _Load(MAI2VCAM_OFF_SEQUENCE);
        std::atomic_thread_fence(std::memory_order_acquire);
        const BYTE* slot = _view + MAI2VCAM_HEADER_BYTES +
                           (size_t)(first % MAI2VCAM_SLOT_COUNT) * MAI2VCAM_FRAME_BYTES;
        memcpy(destination, slot, MAI2VCAM_FRAME_BYTES);
        std::atomic_thread_fence(std::memory_order_acquire);
        const unsigned int second = _Load(MAI2VCAM_OFF_SEQUENCE);
        if ((unsigned int)(second - first) < 2u) {
            if (!_lastLive || !_logged) {
                Mai2VcamLog("queue: 已连上生产者(seq=%u)", second);
                _lastLive = true;
                _logged = true;
            }
            return true;
        }
    }
    // 连续撕裂: 生产者刷新速度远高于本消费者, 给稳定占位帧而不是半帧。
    _Placeholder(destination);
    return false;
}
