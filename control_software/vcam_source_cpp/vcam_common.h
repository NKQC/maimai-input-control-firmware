// mai2control 虚拟摄像头 (DirectShow 视频捕获源) — 公共定义
//
// 本模块是**自研 clean-room 实现**: 只用 Windows SDK 公开的 DirectShow / Win32 API,
// 不链接 DirectShow BaseClasses(strmbase), 不携带任何第三方虚拟摄像头代码。
// 架构与 Windows 上主流虚拟摄像头一致(这是 DirectShow 唯一可行的公开做法):
//   进程内 COM DLL 注册为 VideoInputDeviceCategory 下的捕获源过滤器 →
//   上位机进程作为唯一生产者往命名共享内存队列写 NV12 帧 →
//   过滤器在消费端(游戏/播放器)的图中按协商节拍取帧并 push 给下游。
//
// 生产者与消费者只通过共享内存耦合: 生产者退出/停滞 → 过滤器输出稳定占位帧;
// 消费者断开 → 生产者完全无感。

#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <windows.h>
#include <objbase.h>
#include <strmif.h>
#include <uuids.h>
#include <amvideo.h>
#include <vidcap.h>
#include <ks.h>
#include <ksmedia.h>
// VFW_E_* / E_PROP_* 的定义所在。
#include <vfwmsgs.h>

#include <stdio.h>

// ── 身份 ────────────────────────────────────────────────────────────────────────
// CLSID 与旧的自研 Media Foundation 媒体源彻底分家, 避免残留注册项互相冒充。
// {6E5A1C74-2F83-4C9B-9D1E-7A4B0F3C58E2}
// 这里只声明; 定义放在 vcam_common.cpp(唯一包含 initguid.h 的翻译单元), 其余 DirectShow
// GUID 全部由 strmiids.lib 提供, 避免重复符号。
EXTERN_C const GUID CLSID_Mai2VcamDshow;

#define MAI2VCAM_FRIENDLY_NAME L"mai2control Virtual Camera"
#define MAI2VCAM_PIN_NAME L"Capture"

// ── 共享队列布局(必须与 Rust 侧 src/vcam/share.rs 逐字段一致) ───────────────────
//
// ★为什么必须有 Global\ 这一路★
// Windows 设置 / 相机应用的预览并不在本用户会话里开图: 它们走 Windows Camera Frame Server
// 服务(以 LOCAL SERVICE 跑在 **Session 0**), 由该服务进程载入本 DLL 取帧。而 `Local\` 是
// **会话相对**命名空间, 在 Session 0 里解析成 \Sessions\0\BaseNamedObjects, 与上位机所在
// 交互会话(通常 Session 1)的 \Sessions\1\BaseNamedObjects 是两个不同的对象目录 ——
// 于是 OpenFileMapping 必然失败, 表现为"设备能列出、别的软件能出画、Windows 设置里恒黑"。
// 因此消费端按 Global → Local 顺序各试一次: Global 覆盖跨会话(Frame Server), Local 兜住
// 生产者未提权(拿不到 SeCreateGlobalPrivilege)时的同会话消费端。
#define MAI2VCAM_MAP_NAME_GLOBAL L"Global\\Mai2ControlVirtualCamVideoV1"
#define MAI2VCAM_MAP_NAME_LOCAL L"Local\\Mai2ControlVirtualCamVideoV1"

// 'M2VC'
#define MAI2VCAM_MAGIC 0x4D325643u
#define MAI2VCAM_LAYOUT_VERSION 1u
#define MAI2VCAM_HEADER_BYTES 64u
#define MAI2VCAM_SLOT_COUNT 3u

// ── 分辨率 ──────────────────────────────────────────────────────────────────────
//
// 分辨率是**运行期**量: 由生产者写在共享队列头的 width/height 里, 本 DLL 在过滤器构造时读一次。
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

// ★固定 10fps(100ns 单位)★ 上下限取同一个值 ⇒ 协商结果恒为 10fps, 不再随下游喜好漂移。
// 本路画面是"静态 QR 显示若干秒"的准静态源, 30fps 只是把同一帧重复推 3 倍、白烧 CPU 与带宽;
// 10fps 对扫码识别绰绰有余, 且与生产者侧的 100ms 定频发布一一对应(见 src/vcam/share.rs)。
// 上下限一致后 Mai2VcamBuildMediaType / AcceptMediaType / IntervalOf 的钳位逻辑自动收敛到该值,
// 无需在各处另写特例。
#define MAI2VCAM_DEFAULT_INTERVAL 1000000LL
#define MAI2VCAM_MAX_INTERVAL 1000000LL

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

// ── 小工具 ──────────────────────────────────────────────────────────────────────

// 诊断日志: 只写 %ProgramData%\mai2control\mai2vcam_dshow.log, 且默认极低频
// (连接/状态迁移/错误), 不打每帧日志 —— 过滤器跑在别人的进程里, 日志膨胀是真实事故源。
void Mai2VcamLog(const char* format, ...);

// AM_MEDIA_TYPE 的 CoTaskMem 深拷贝/释放。BaseClasses 里叫 CopyMediaType/FreeMediaType,
// 这里自实现以免引入 strmbase。
HRESULT Mai2VcamCopyMediaType(AM_MEDIA_TYPE* destination, const AM_MEDIA_TYPE* source);
void Mai2VcamFreeMediaTypeContents(AM_MEDIA_TYPE* type);
void Mai2VcamDeleteMediaType(AM_MEDIA_TYPE* type);

// 构造本过滤器当前支持的媒体类型(NV12, 指定分辨率与帧间隔)。
HRESULT Mai2VcamBuildMediaType(AM_MEDIA_TYPE* type, LONGLONG frameInterval, int width, int height);
// 判定给定媒体类型是否可接受(部分指定也允许: 空 major/subtype/format 视为通配)。
// 分辨率必须与本针脚当前这一种完全一致 —— 本源只报一种格式, 不做缩放。
bool Mai2VcamAcceptMediaType(const AM_MEDIA_TYPE* type, int width, int height);
// 从共享队列头读一次生产者的当前分辨率; 读不到(生产者不在/头非法)则给回落分辨率。
// 只读一次: DirectShow 的格式在连接时就定死了, 中途变不了(见 vcam_filter.cpp 的说明)。
void Mai2VcamQueryQueueSize(int* width, int* height);
// 从媒体类型取帧间隔; 非法值回落默认值。
LONGLONG Mai2VcamIntervalOf(const AM_MEDIA_TYPE* type);

// 模块级 COM 对象计数(DllCanUnloadNow 用)。
void Mai2VcamLockModule();
void Mai2VcamUnlockModule();
long Mai2VcamModuleLocks();

extern HINSTANCE g_mai2vcamModule;
