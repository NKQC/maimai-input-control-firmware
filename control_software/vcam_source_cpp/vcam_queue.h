// 共享 NV12 帧队列的**消费端**。生产者(上位机 Rust 进程)实现见 src/vcam/share.rs。
//
// 设计要点:
//  - 只读打开命名映射; 打不开/头非法/状态非 READY/心跳停滞 → 输出稳定占位帧(纯黑 NV12),
//    绝不把上一帧残留或未初始化内存推给消费端。
//  - 3 槽 + 单调 sequence 构成无锁三缓冲: 读前后各取一次 sequence, 差值 >=2 说明生产者可能
//    已经追上正在读的槽 → 重试(最多 3 次)。不需要任何互斥体, 生产者永不被消费者阻塞。
//  - 打开失败按节流重试, 避免在每帧回调里反复 OpenFileMapping。

#pragma once

#include "vcam_common.h"

class Mai2VcamQueueReader {
public:
    Mai2VcamQueueReader() = default;
    ~Mai2VcamQueueReader() { Close(); }

    // 取一帧 NV12 到 destination(容量必须 >= Mai2VcamFrameBytes(width, height))。
    // width/height 是本针脚**已协商**的分辨率。
    //
    // ★分辨率全程透传, 本源永不缩放★
    // 生产者写进队列头的尺寸就是消费端拿到的尺寸: 针脚在**未连接**时按队列头报格式(见
    // vcam_filter.cpp 的 _RefreshSize), 连上之后队列尺寸与协商尺寸必然相等, 于是这里只做
    // 逐字节 memcpy。缩放会改变每模块的像素边长并抹掉 QR 的硬边, 属于对画面内容的加工,
    // 与"设置的分辨率原样出画"这个硬要求直接冲突, 因此一行都不做。
    //
    // 唯一会出现尺寸不等的窗口是"消费端已协商完、生产者随后改了分辨率"。此时没有任何合法
    // 途径把新尺寸通知下游(DirectShow 的媒体类型在 Connect 时定死), 所以只能给稳定占位帧
    // 并要求在消费端重新打开摄像头 —— 上位机改分辨率时会强制卸载重建摄像头(见
    // src/ui_callbacks/virtual_camera.rs), 把这个窗口压到一次重连之内。
    // 返回 true = 来自生产者的真实帧; false = 已填占位帧。
    bool Read(BYTE* destination, int width, int height);

    void Close();

private:
    bool _Open();
    unsigned int _Load(unsigned int offset) const;
    // 三缓冲读取(撕裂重试在内)。frameBytes 恒为**协商尺寸**下的帧长 —— 尺寸不等时根本走不到这里。
    bool _ReadSlots(BYTE* destination, int frameBytes, unsigned int* sequence);
    static void _Placeholder(BYTE* destination, int width, int height);

    HANDLE _map = nullptr;
    const BYTE* _view = nullptr;
    // 打开失败节流: 以帧为单位倒数, 避免高频 OpenFileMapping。
    int _retryCountdown = 0;
    // 只在状态发生变化时写日志。
    bool _lastLive = false;
    bool _logged = false;
    // 已记录过的打开失败码(Global / Local), 只在变化时再记一条。
    DWORD _loggedOpenError = 0;
    DWORD _loggedLocalError = 0;
};
