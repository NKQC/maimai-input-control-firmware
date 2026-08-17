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

// ── 共享队列布局 ────────────────────────────────────────────────────────────────
// 布局定义已抽到 vcam_queue_layout.h: 它同时被 MF 媒体源(vcam_source_mf/)包含, 两个消费端
// DLL 共用同一份常量与同一套原子读法, 不再各写一份。改动那里必须同步 src/vcam/share.rs。
//
// ★注意: 本 DLL 与"Windows 设置里看不到"无关★ Media Foundation 不枚举 DirectShow 采集过滤器,
// 所以 Windows 设置 / 相机应用 / Teams 这类 MF 消费端从来就不会加载本 DLL —— 那条路由
// vcam_source_mf/ 那份 MF 媒体源承担(同一个共享队列, 两个门面)。
// 本 DLL 的服务对象是 DirectShow 消费端(游戏 / OBS 的"视频采集设备" / amcap)。
#include "vcam_queue_layout.h"

// ── 小工具 ──────────────────────────────────────────────────────────────────────

// 诊断日志 Mai2VcamLog 的声明在 vcam_queue_layout.h(上面已包含), 实现在 vcam_log.cpp。

// AM_MEDIA_TYPE 的 CoTaskMem 深拷贝/释放。BaseClasses 里叫 CopyMediaType/FreeMediaType,
// 这里自实现以免引入 strmbase。
HRESULT Mai2VcamCopyMediaType(AM_MEDIA_TYPE* destination, const AM_MEDIA_TYPE* source);
void Mai2VcamFreeMediaTypeContents(AM_MEDIA_TYPE* type);
void Mai2VcamDeleteMediaType(AM_MEDIA_TYPE* type);

// 构造本过滤器当前支持的媒体类型(NV12, 指定分辨率与帧间隔)。
HRESULT Mai2VcamBuildMediaType(AM_MEDIA_TYPE* type, LONGLONG frameInterval, int width, int height);
// 判定给定媒体类型是否可接受(部分指定也允许: 空 major/subtype/format 视为通配)。
// 本源只**报**一种格式(= 生产者当前分辨率), 分辨率必须与之完全一致才接受: 不缩放、不裁剪,
// 取帧时队列尺寸与已协商尺寸不等一律给占位帧(见 vcam_queue.h)。
bool Mai2VcamAcceptMediaType(const AM_MEDIA_TYPE* type, int width, int height);
// 从共享队列头读一次生产者的当前分辨率; 读不到(生产者不在/头非法)则给回落分辨率。
// 针脚在**未连接**的每个格式入口都调它一次(见 vcam_filter.cpp 的 _RefreshSize): 分辨率靠这一路
// 原样透传给下游; 连上之后格式与下游定死, 不再重取。
void Mai2VcamQueryQueueSize(int* width, int* height);
// 从媒体类型取帧间隔; 非法值回落默认值。
LONGLONG Mai2VcamIntervalOf(const AM_MEDIA_TYPE* type);

// 模块级 COM 对象计数(DllCanUnloadNow 用)。
void Mai2VcamLockModule();
void Mai2VcamUnlockModule();
long Mai2VcamModuleLocks();

extern HINSTANCE g_mai2vcamModule;
