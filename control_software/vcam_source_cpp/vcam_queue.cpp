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

// 跨进程原子 acquire 读。读法本身(以及"为什么不能用 volatile / Interlocked")定义在
// vcam_queue_layout.h 的 Mai2VcamLoadU32 —— 与 MF 侧消费端共用同一份, 避免两个 DLL 各写一套。
unsigned int Mai2VcamQueueReader::_Load(unsigned int offset) const {
    return Mai2VcamLoadU32(_view, offset);
}

void Mai2VcamQueueReader::_Placeholder(BYTE* destination, int width, int height) {
    // NV12 全黑(BT.601 full range): Y=0, UV=128。与生产者对黑帧的编码一致,
    // 消费端在"生产者不在"和"生产者输出黑屏"两种情况下看到的画面完全相同。
    const size_t luma = (size_t)width * height;
    memset(destination, 0, luma);
    memset(destination + luma, 128, (size_t)Mai2VcamFrameBytes(width, height) - luma);
}

// 只读打开一次队列, 取生产者当前分辨率。拿不到就给回落分辨率。
// 这是个独立的短生命周期读取: 针脚在**未连接**时(构造、枚举格式、连接前)各调一次,
// 不与推流用的那个 reader 共享句柄 —— 分辨率透传就靠这一路把队列头搬到报出去的媒体类型上。
void Mai2VcamQueueSizeProbe(int* width, int* height) {
    int w = MAI2VCAM_DEFAULT_WIDTH;
    int h = MAI2VCAM_DEFAULT_HEIGHT;
    HANDLE map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME_GLOBAL);
    const DWORD globalError = map == nullptr ? GetLastError() : 0;
    if (map == nullptr) {
        map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME_LOCAL);
        if (map == nullptr) {
            Mai2VcamLog("queue: 分辨率探测打不开队列(Global err=%lu, Local err=%lu) → 回落 %dx%d",
                        globalError, GetLastError(), w, h);
        }
    }
    if (map != nullptr) {
        const BYTE* view =
            (const BYTE*)MapViewOfFile(map, FILE_MAP_READ, 0, 0, MAI2VCAM_MAP_BYTES);
        if (view != nullptr) {
            const auto load = [view](unsigned int offset) -> unsigned int {
                return Mai2VcamLoadU32(view, offset);
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
    const DWORD globalError = map == nullptr ? GetLastError() : 0;
    const wchar_t* opened = MAI2VCAM_MAP_NAME_GLOBAL;
    if (map == nullptr) {
        map = OpenFileMappingW(FILE_MAP_READ, FALSE, MAI2VCAM_MAP_NAME_LOCAL);
        opened = MAI2VCAM_MAP_NAME_LOCAL;
    }
    if (map == nullptr) {
        // ★必须把失败码记下来★ 2(不存在) = 生产者没在这个命名空间建队;
        // 5(拒绝访问) = 队列在, 但本进程的令牌过不了段对象的访问检查(AppContainer / 受限
        // 应用包的检查除了用户组还要求安全描述符显式授予应用包 SID) —— 两者的处置完全不同,
        // 混在一起就只能靠猜。
        const DWORD localError = GetLastError();
        // 打不开会每 kRetryFrames 帧重试一次, 因此只在失败码变化时记一条, 不让日志涨爆。
        if (_loggedOpenError != globalError || _loggedLocalError != localError) {
            Mai2VcamLog("queue: 两个命名空间都打不开(Global err=%lu, Local err=%lu)", globalError,
                        localError);
            _loggedOpenError = globalError;
            _loggedLocalError = localError;
        }
        return false;
    }
    _loggedOpenError = 0;
    _loggedLocalError = 0;
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

bool Mai2VcamQueueReader::_ReadSlots(BYTE* destination, int frameBytes, unsigned int* sequence) {
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
            *sequence = second;
            return true;
        }
    }
    return false;
}

bool Mai2VcamQueueReader::Read(BYTE* destination, int width, int height) {
    if (destination == nullptr || !Mai2VcamSizeValid(width, height)) {
        return false;
    }
    if (!_Open()) {
        _Placeholder(destination, width, height);
        if (_lastLive || !_logged) {
            Mai2VcamLog("queue: 生产者不在(未找到共享队列) → 占位帧");
            _lastLive = false;
            _logged = true;
        }
        return false;
    }

    // ★队列尺寸必须与协商尺寸完全相等, 不等即不可用★
    // 分辨率是硬透传的: 针脚在未连接时按队列头报格式, 所以连上之后两者本就应当相等。真的不等
    // 只有一种成因 —— 消费端协商完之后生产者改了分辨率。DirectShow 没有任何在运行中换尺寸的
    // 合法通路, 而缩放是对画面内容的加工(改变每模块像素边长、抹掉 QR 硬边), 与透传要求冲突,
    // 所以这里既不缩放也不按旧尺寸解释像素, 只给稳定占位帧, 由消费端重新打开摄像头解决。
    // 注: 下面判定不可用时会**关闭本次映射**, 让下一次尝试重新按 Global → Local 选一遍。
    const bool layoutOk = _Load(MAI2VCAM_OFF_MAGIC) == MAI2VCAM_MAGIC &&
                          _Load(MAI2VCAM_OFF_VERSION) == MAI2VCAM_LAYOUT_VERSION &&
                          _Load(MAI2VCAM_OFF_HEADER_BYTES) == MAI2VCAM_HEADER_BYTES &&
                          _Load(MAI2VCAM_OFF_FOURCC) == MAI2VCAM_FOURCC_NV12 &&
                          _Load(MAI2VCAM_OFF_SLOT_COUNT) == MAI2VCAM_SLOT_COUNT &&
                          _Load(MAI2VCAM_OFF_SLOT_BYTES) == (unsigned int)MAI2VCAM_SLOT_STRIDE;
    const int queueWidth = layoutOk ? (int)_Load(MAI2VCAM_OFF_WIDTH) : 0;
    const int queueHeight = layoutOk ? (int)_Load(MAI2VCAM_OFF_HEIGHT) : 0;
    const bool sizeMatch = layoutOk && queueWidth == width && queueHeight == height;
    const bool headerOk = sizeMatch && Mai2VcamSizeValid(queueWidth, queueHeight);
    const unsigned int state = headerOk ? _Load(MAI2VCAM_OFF_STATE) : MAI2VCAM_STATE_INVALID;
    // 心跳: 生产者每次发布都会刷新, 停滞超时即视为掉线(进程被杀不会改 state)。
    const unsigned int tick = headerOk ? _Load(MAI2VCAM_OFF_TICK_MS) : 0;
    const unsigned int now = GetTickCount();
    const bool alive = headerOk && state == MAI2VCAM_STATE_READY &&
                       (now - tick) < MAI2VCAM_HEARTBEAT_TIMEOUT_MS;
    if (!alive) {
        _Placeholder(destination, width, height);
        if (_lastLive || !_logged) {
            // 尺寸不等单独报: 它是唯一能靠用户操作(在消费端重开摄像头)解决的失效原因,
            // 混进通用的"队列不可用"里就只能靠读数猜。
            if (layoutOk && !sizeMatch) {
                Mai2VcamLog("queue: 队列 %dx%d 与协商 %dx%d 不等 → 占位帧(本源不缩放, 分辨率硬透传; "
                            "请在消费端重新打开摄像头)",
                            queueWidth, queueHeight, width, height);
            } else {
                Mai2VcamLog("queue: 队列不可用(header=%d state=%u 心跳滞后=%ums 队列=%dx%d 协商=%dx%d) → 占位帧",
                            headerOk ? 1 : 0, state, headerOk ? (now - tick) : 0u, queueWidth,
                            queueHeight, width, height);
            }
            _lastLive = false;
            _logged = true;
        }
        // ★关掉重选, 不要死抱着这一份★
        // 旧实现一旦 _Open 成功就永远用那一份映射(`_Open` 见 _view != nullptr 直接返回 true)。
        // 于是出现这种死局: 上位机曾以管理员跑过, 在 Global 建过队列; 某个消费端一直持有该映射的
        // 读句柄, 使它在生产者退出后仍然存在(state=Stopping 的孤儿); 之后上位机以普通权限重启,
        // 只能在 Local 建队 —— 而本读取器永远停在那份 Global 孤儿上, 明明有活着的 Local 队列却
        // 一帧都取不到。关闭后, 下一次(受 kRetryFrames 节流)会重新按 Global → Local 选一遍,
        // 自然切到活着的那一份。分辨率对不上的情形同理: 另一个命名空间里可能正有匹配的队列。
        Close();
        return false;
    }

    // 尺寸相等已由 headerOk 保证, 因此这里是逐字节直拷 —— 没有中转缓冲, 没有重采样。
    unsigned int sequence = 0;
    if (!_ReadSlots(destination, Mai2VcamFrameBytes(width, height), &sequence)) {
        // 连续撕裂: 生产者刷新速度远高于本消费者, 给稳定占位帧而不是半帧。
        _Placeholder(destination, width, height);
        return false;
    }
    if (!_lastLive || !_logged) {
        Mai2VcamLog("queue: 已连上生产者(seq=%u 尺寸透传 %dx%d)", sequence, width, height);
        _lastLive = true;
        _logged = true;
    }
    return true;
}
