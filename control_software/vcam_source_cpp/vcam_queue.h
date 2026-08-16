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
    // width/height 是本针脚**已协商**的分辨率: 队列头报的分辨率与它不一致时一律给占位帧,
    // 绝不把另一种尺寸的像素塞进按这个尺寸协商好的缓冲(那是花屏或越界)。
    // 返回 true = 来自生产者的真实帧; false = 已填占位帧。
    bool Read(BYTE* destination, int width, int height);

    void Close();

private:
    bool _Open();
    unsigned int _Load(unsigned int offset) const;
    static void _Placeholder(BYTE* destination, int width, int height);

    HANDLE _map = nullptr;
    const BYTE* _view = nullptr;
    // 打开失败节流: 以帧为单位倒数, 避免高频 OpenFileMapping。
    int _retryCountdown = 0;
    // 只在状态发生变化时写日志。
    bool _lastLive = false;
    bool _logged = false;
};
