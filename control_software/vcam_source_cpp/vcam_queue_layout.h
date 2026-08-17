// 共享 NV12 帧队列的**布局定义** —— 生产者与全部消费端唯一的耦合面。
//
// ★为什么单独一个头文件★ 这份布局有三个实现者:
//   · 生产者          control_software/src/vcam/share.rs        (Rust)
//   · DirectShow 消费端 vcam_source_cpp/vcam_queue.cpp            (本目录)
//   · Media Foundation 消费端 vcam_source_mf/FrameReader.cpp      (MF 媒体源)
// 后两者是两个独立的 DLL, 跑在完全不同的宿主进程里(游戏 / 帧服务器)。布局若在两处各写一份,
// 改一个常量就会留下一个"只表现为黑屏、没有任何线索"的不一致 —— 那正是 08-04 换架构时踩过的坑
// (队列改名后旧 MF 消费端变成恒黑幽灵)。故本文件**不含任何 DirectShow / MF 专有类型**,
// 只要 <windows.h> 就能包含, 让两个 DLL 都直接用同一份定义。
// ★改动本文件必须同步 share.rs★ 那边是 Rust, 无法共享头文件, 只能靠注释约束。

#pragma once

#include <windows.h>

// ── 命名空间 ────────────────────────────────────────────────────────────────────
//
// ★为什么必须有 Global\ 这一路★
// `Local\` 是**会话相对**命名空间: 在每个登录会话里解析到各自的 \Sessions\<N>\BaseNamedObjects。
// 消费端不一定在上位机所在的登录会话里 —— MF 帧服务器就跑在 Session 0, 它查的 `Local\...`
// 与上位机建的是两个不同的对象目录, OpenFileMapping 必然失败。因此按 Global → Local 顺序各试
// 一次: Global 覆盖跨会话(帧服务器 ⇒ Windows 设置的相机预览), Local 兜住生产者未提权
// (拿不到 SeCreateGlobalPrivilege)时的同会话消费端。
#define MAI2VCAM_MAP_NAME_GLOBAL L"Global\\Mai2ControlVirtualCamVideoV1"
#define MAI2VCAM_MAP_NAME_LOCAL L"Local\\Mai2ControlVirtualCamVideoV1"

// 'M2VC'
#define MAI2VCAM_MAGIC 0x4D325643u
// ★布局版本 2★ 与 1 相比有两处**不兼容**改动, 因此必须换号:
//   · slot_bytes 的含义从"当前帧字节数"变成"槽间距(上限帧长, 恒定)";
//   · 映射总大小随之按上限分辨率计算, 从约 1.4MB 变成约 8.9MB。
// 不换号的后果是: 装着旧 DLL 的消费端会在头部校验里静默失败(它拿 slot_bytes 与自己编译期的
// 帧长比), 只表现为"一直黑屏", 没有任何线索指向"DLL 该更新了"。
#define MAI2VCAM_LAYOUT_VERSION 2u
#define MAI2VCAM_HEADER_BYTES 64u
#define MAI2VCAM_SLOT_COUNT 3u

// 载荷 FourCC = 'NV12'(小端)。不用 MAKEFOURCC 是为了不牵进 mmsystem.h / amvideo.h ——
// 本头文件要能被 MF 侧直接包含。
#define MAI2VCAM_FOURCC_NV12 0x3231564Eu

// ── 分辨率 ──────────────────────────────────────────────────────────────────────
//
// 分辨率是**运行期**量: 由生产者写在共享队列头的 width/height 里, 消费端原样透传给下游,
// 全链路不缩放。
//
// ★为什么共享映射按上限开, 而不按当前分辨率开★ 映射的名字是固定的, 消费端(可能是别的进程、
// 甚至别的会话)映射时必须知道映射多大。若映射大小随分辨率变, 那么每次改分辨率都要重建映射并让
// 所有消费端重新打开 —— 而消费端的打开时机不由我们控制。按上限一次开好, 槽间距恒定, 改分辨率
// 只改头部两个字段, 布局完全不动。代价是常驻虚拟内存按上限计(1920x1080 三槽约 8.9MB, 且未写到
// 的页不会真正占物理内存)。
#define MAI2VCAM_MAX_WIDTH 1920
#define MAI2VCAM_MAX_HEIGHT 1080
// 槽间距 = 上限分辨率下的一帧字节数(NV12: Y 平面 W*H + 交错 UV 平面 W*H/2)。恒定, 与当前分辨率无关。
#define MAI2VCAM_SLOT_STRIDE ((MAI2VCAM_MAX_WIDTH * MAI2VCAM_MAX_HEIGHT * 3) / 2)
#define MAI2VCAM_MAP_BYTES (MAI2VCAM_HEADER_BYTES + MAI2VCAM_SLOT_COUNT * MAI2VCAM_SLOT_STRIDE)

// 生产者不在时的回落分辨率: 没有队列可读时也必须报得出一种格式, 否则消费端连枚举都做不了。
#define MAI2VCAM_DEFAULT_WIDTH 640
#define MAI2VCAM_DEFAULT_HEIGHT 480

// 当前分辨率下的一帧字节数。NV12 要求宽高均为偶数(色度 4:2:0 按 2x2 块取样), 生产者侧已强制。
inline int Mai2VcamFrameBytes(int width, int height) { return (width * height * 3) / 2; }

// 分辨率是否在本实现支持的范围内且满足 NV12 的偶数要求。
inline bool Mai2VcamSizeValid(int width, int height) {
    return width >= 16 && height >= 16 && width <= MAI2VCAM_MAX_WIDTH &&
           height <= MAI2VCAM_MAX_HEIGHT && (width % 2) == 0 && (height % 2) == 0;
}

// ── 节拍 ────────────────────────────────────────────────────────────────────────
//
// ★固定 10fps(100ns 单位)★ 本路画面是"静态 QR 显示若干秒"的准静态源, 30fps 只是把同一帧重复推
// 3 倍、白烧 CPU 与带宽; 10fps 对扫码识别绰绰有余, 且与生产者侧的 100ms 定频发布一一对应
// (见 src/vcam/share.rs 的 PUBLISH_INTERVAL_MS)。
#define MAI2VCAM_DEFAULT_INTERVAL 1000000LL
#define MAI2VCAM_MAX_INTERVAL 1000000LL
// 上面那个间隔换算成整数帧率, 给 MF 侧的 MF_MT_FRAME_RATE 用(10000000 / 1000000 = 10)。
#define MAI2VCAM_FPS 10u

// 生产者心跳超时: 超过则视为生产者停滞, 输出占位帧。
#define MAI2VCAM_HEARTBEAT_TIMEOUT_MS 1500u

enum Mai2VcamState : unsigned int {
    MAI2VCAM_STATE_INVALID = 0,
    MAI2VCAM_STATE_STARTING = 1,
    MAI2VCAM_STATE_READY = 2,
    MAI2VCAM_STATE_STOPPING = 3,
};

// 头部字段偏移(字节)。用显式偏移而不是结构体, 保证两侧不受编译器打包策略影响。
enum Mai2VcamHeaderOffset : unsigned int {
    MAI2VCAM_OFF_MAGIC = 0x00,
    MAI2VCAM_OFF_VERSION = 0x04,
    MAI2VCAM_OFF_HEADER_BYTES = 0x08,
    MAI2VCAM_OFF_STATE = 0x0C,
    MAI2VCAM_OFF_WIDTH = 0x10,
    MAI2VCAM_OFF_HEIGHT = 0x14,
    MAI2VCAM_OFF_FOURCC = 0x18,
    MAI2VCAM_OFF_SLOT_COUNT = 0x1C,
    MAI2VCAM_OFF_SLOT_BYTES = 0x20,
    MAI2VCAM_OFF_SEQUENCE = 0x24,
    MAI2VCAM_OFF_PRODUCER_PID = 0x28,
    MAI2VCAM_OFF_TICK_MS = 0x2C,
    MAI2VCAM_OFF_INTERVAL = 0x30,
};

// ── 诊断日志 ────────────────────────────────────────────────────────────────────
//
// 只写 %ProgramData%\mai2control\mai2vcam_dshow.log, 且默认极低频(连接 / 状态迁移 / 错误),
// 不打每帧日志 —— 这两个 DLL 都跑在别人的进程里(游戏 / MF 帧服务器), 日志膨胀是真实事故源。
//
// ★两个消费端 DLL 共用同一个文件★ 每行带 pid, 于是"DirectShow 那半边在干什么"与"帧服务器
// 那半边在干什么"能在同一条时间轴上对照。黑屏排查唯一有用的现场就是这个文件 —— MF 侧原本
// 只有 ETW(WINTRACE), 没开 trace 会话就什么都留不下, 等于没有。
// 实现在 vcam_log.cpp, 两个工程都编译它。
void Mai2VcamLog(const char* format, ...);

// 队列头里的动态字段必须按跨进程原子语义读取。生产者(Rust `AtomicU32::store(Release)`)与
// 消费端(`std::atomic_ref<unsigned int>::load(acquire)`)配对。
//
// ★不能用 volatile 冒充原子★ volatile 只约束编译器, 不提供原子性与可见性契约。
// ★不能用 Interlocked* 做"原子读"★ 消费端的视图是 FILE_MAP_READ, InterlockedCompareExchange
// 一类实现是 `lock cmpxchg`(读改写), 会对只读页写入而触发访问违例。atomic_ref 的 load 在
// x86/x64 上是单条对齐 mov, 不写内存。头部按 4 字节对齐(映射视图页对齐 + 字段偏移是 4 的倍数),
// 满足 atomic_ref 的对齐要求。
//
// 本函数放在布局头里, 是为了让两个消费端 DLL 用**同一份**读法 —— 这条规则错一次就是随机撕裂,
// 而撕裂在静态 QR 画面上几乎观察不到。
#ifdef __cplusplus
#include <atomic>
inline unsigned int Mai2VcamLoadU32(const BYTE* view, unsigned int offset) {
    unsigned int* cell =
        const_cast<unsigned int*>(reinterpret_cast<const unsigned int*>(view + offset));
    return std::atomic_ref<unsigned int>(*cell).load(std::memory_order_acquire);
}
#endif
