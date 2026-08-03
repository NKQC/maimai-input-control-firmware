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
#define MAI2VCAM_MAP_NAME L"Local\\Mai2ControlVirtualCamVideoV1"

// 'M2VC'
#define MAI2VCAM_MAGIC 0x4D325643u
#define MAI2VCAM_LAYOUT_VERSION 1u
#define MAI2VCAM_HEADER_BYTES 64u
#define MAI2VCAM_SLOT_COUNT 3u
#define MAI2VCAM_WIDTH 640
#define MAI2VCAM_HEIGHT 480
// NV12: Y 平面 W*H + 交错 UV 平面 W*H/2。
#define MAI2VCAM_FRAME_BYTES ((MAI2VCAM_WIDTH * MAI2VCAM_HEIGHT * 3) / 2)
#define MAI2VCAM_MAP_BYTES (MAI2VCAM_HEADER_BYTES + MAI2VCAM_SLOT_COUNT * MAI2VCAM_FRAME_BYTES)

// 默认 30fps(100ns 单位)与协商下限 5fps。
#define MAI2VCAM_DEFAULT_INTERVAL 333333LL
#define MAI2VCAM_MAX_INTERVAL 2000000LL

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

// 构造本过滤器唯一支持的媒体类型(NV12 640x480, 指定帧间隔)。
HRESULT Mai2VcamBuildMediaType(AM_MEDIA_TYPE* type, LONGLONG frameInterval);
// 判定给定媒体类型是否可接受(部分指定也允许: 空 major/subtype/format 视为通配)。
bool Mai2VcamAcceptMediaType(const AM_MEDIA_TYPE* type);
// 从媒体类型取帧间隔; 非法值回落默认值。
LONGLONG Mai2VcamIntervalOf(const AM_MEDIA_TYPE* type);

// 模块级 COM 对象计数(DllCanUnloadNow 用)。
void Mai2VcamLockModule();
void Mai2VcamUnlockModule();
long Mai2VcamModuleLocks();

extern HINSTANCE g_mai2vcamModule;
