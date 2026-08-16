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

void Mai2VcamQueueReader::_Placeholder(BYTE* destination, int width, int height) {
    // NV12 全黑(BT.601 full range): Y=0, UV=128。与生产者对黑帧的编码一致,
    // 消费端在"生产者不在"和"生产者输出黑屏"两种情况下看到的画面完全相同。
    const size_t luma = (size_t)width * height;
    memset(destination, 0, luma);
    memset(destination + luma, 128, (size_t)Mai2VcamFrameBytes(width, height) - luma);
}

// 只读打开一次队列, 取生产者当前分辨率。拿不到就给回落分辨率。
// 这是个独立的短生命周期读取: 过滤器构造时调一次, 不与推流用的那个 reader 共享句柄。
void Mai2VcamQueueSizeProbe(int* width, int* height) {
    int w = MAI2VCAM_DEFAULT_WIDTH;
    int h = MAI2VCAM_DEFAULT_HEIGHT;
    HANDLE map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME_GLOBAL);
    if (map == nullptr) {
        map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME_LOCAL);
    }
    if (map != nullptr) {
        const BYTE* view =
            (const BYTE*)MapViewOfFile(map, FILE_MAP_READ, 0, 0, MAI2VCAM_MAP_BYTES);
        if (view != nullptr) {
            const auto load = [view](unsigned int offset) -> unsigned int {
                unsigned int* cell = const_cast<unsigned int*>(
                    reinterpret_cast<const unsigned int*>(view + offset));
                return std::atomic_ref<unsigned int>(*cell).load(std::memory_order_acquire);
            };
            if (load(MAI2VCAM_OFF_MAGIC) == MAI2VCAM_MAGIC &&
                load(MAI2VCAM_OFF_VERSION) == MAI2VCAM_LAYOUT_VERSION) {
                const int qw = (int)load(MAI2VCAM_OFF_WIDTH);
                const int qh = (int)load(MAI2VCAM_OFF_HEIGHT);
                if (Mai2VcamSizeValid(qw, qh)) {
                    w = qw;
                    h = qh;
                }
            }
            UnmapViewOfFile(view);
        }
        CloseHandle(map);
    }
    if (width != nullptr) {
        *width = w;
    }
    if (height != nullptr) {
        *height = h;
    }
}

void Mai2VcamQueryQueueSize(int* width, int* height) { Mai2VcamQueueSizeProbe(width, height); }

bool Mai2VcamQueueReader::_Open() {
    if (_view != nullptr) {
        return true;
    }
    if (_retryCountdown > 0) {
        _retryCountdown--;
        return false;
    }
    _retryCountdown = kRetryFrames;
    // ★Global 优先, Local 兜底★ 本 DLL 可能被载入 Session 0 的 Frame Server(Windows 设置 /
    // 相机应用的取帧路径), 那里的 `Local\` 与上位机所在交互会话的 `Local\` 是两个不同的对象
    // 目录 —— 只试 Local 就等于对 Windows 自带的相机预览永久输出黑帧。顺序不能反: 生产者
    // 提权时两个名字都存在, 而 Global 那份才是跨会话都指向同一块内存的那一份。
    HANDLE map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME_GLOBAL);
    const wchar_t* opened = MAI2VCAM_MAP_NAME_GLOBAL;
    if (map == nullptr) {
        map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME_LOCAL);
        opened = MAI2VCAM_MAP_NAME_LOCAL;
    }
    if (map == nullptr) {
        return false;
    }
    const BYTE* view = (const BYTE*)MapViewOfFile(map, FILE_MAP_READ, 0, 0, MAI2VCAM_MAP_BYTES);
    if (view == nullptr) {
        CloseHandle(map);
        return false;
    }
    Mai2VcamLog("queue: 已打开共享映射 %ls", opened);
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

bool Mai2VcamQueueReader::Read(BYTE* destination, int width, int height) {
    if (destination == nullptr || !Mai2VcamSizeValid(width, height)) {
        return false;
    }
    const int frameBytes = Mai2VcamFrameBytes(width, height);
    if (!_Open()) {
        _Placeholder(destination, width, height);
        if (_lastLive || !_logged) {
            Mai2VcamLog("queue: 生产者不在(未找到共享队列) → 占位帧");
            _lastLive = false;
            _logged = true;
        }
        return false;
    }

    // ★分辨率必须与已协商的那一种逐字相等★ 生产者可以在运行中改分辨率, 但 DirectShow 的媒体
    // 类型在连接时就定死了 —— 此时只能给占位帧, 等消费端重新打开摄像头重新协商。把新尺寸的
    // 像素按旧尺寸推下去只会得到花屏, 而且会越过下游按旧尺寸申请的缓冲。
    const bool headerOk = _Load(MAI2VCAM_OFF_MAGIC) == MAI2VCAM_MAGIC &&
                          _Load(MAI2VCAM_OFF_VERSION) == MAI2VCAM_LAYOUT_VERSION &&
                          _Load(MAI2VCAM_OFF_HEADER_BYTES) == MAI2VCAM_HEADER_BYTES &&
                          _Load(MAI2VCAM_OFF_WIDTH) == (unsigned int)width &&
                          _Load(MAI2VCAM_OFF_HEIGHT) == (unsigned int)height &&
                          _Load(MAI2VCAM_OFF_FOURCC) == (unsigned int)MAKEFOURCC('N', 'V', '1', '2') &&
                          _Load(MAI2VCAM_OFF_SLOT_COUNT) == MAI2VCAM_SLOT_COUNT &&
                          _Load(MAI2VCAM_OFF_SLOT_BYTES) == (unsigned int)MAI2VCAM_SLOT_STRIDE;
    const unsigned int state = headerOk ? _Load(MAI2VCAM_OFF_STATE) : MAI2VCAM_STATE_INVALID;
    // 心跳: 生产者每次发布都会刷新, 停滞超时即视为掉线(进程被杀不会改 state)。
    const unsigned int tick = headerOk ? _Load(MAI2VCAM_OFF_TICK_MS) : 0;
    const unsigned int now = GetTickCount();
    const bool alive = headerOk && state == MAI2VCAM_STATE_READY &&
                       (now - tick) < MAI2VCAM_HEARTBEAT_TIMEOUT_MS;
    if (!alive) {
        _Placeholder(destination, width, height);
        if (_lastLive || !_logged) {
            Mai2VcamLog("queue: 队列不可用(header=%d state=%u 心跳滞后=%ums 协商=%dx%d) → 占位帧",
                        headerOk ? 1 : 0, state, headerOk ? (now - tick) : 0u, width, height);
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
        // 槽间距恒为上限帧长(与当前分辨率无关), 但只拷当前分辨率这一帧的有效字节。
        const BYTE* slot = _view + MAI2VCAM_HEADER_BYTES +
                           (size_t)(first % MAI2VCAM_SLOT_COUNT) * MAI2VCAM_SLOT_STRIDE;
        memcpy(destination, slot, frameBytes);
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
    _Placeholder(destination, width, height);
    return false;
}
