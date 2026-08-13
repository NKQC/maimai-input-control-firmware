#pragma once
#include <cstdint>
// 触控输出流水线各段耗时滚动最大值(us)。每次 telemetry 帧上报后清零，故反映最近一帧窗口内峰值。
// 常测(成本仅几次 time_us_32)，UI 开关只控显示。
extern volatile uint16_t g_lat_spi_us;   // PSoC SPI 触控读事务耗时
extern volatile uint16_t g_lat_proc_us;  // RP2040 处理(mask→区映射→mai2帧构建)
extern volatile uint16_t g_lat_usb_us;   // serial/CDC 写耗时
// 内联:以 v 更新滚动最大(饱和到 u16)
static inline void latency_note(volatile uint16_t* slot, uint32_t v) {
    uint16_t cur = (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
    if (cur > *slot) *slot = cur;
}

// ---------------------------------------------------------------------------
// 触控延迟线的**补偿偏差**: (实际发出时刻 − 该掩码的采样时刻) − comm.touch_delay_100us。
// ★这是"设定的触控延迟到底兑现了没有"的唯一直接读数★ 链路耗时只是延迟线内部被扣掉的一项,
// 它离目标多远说明不了任何事; 而这里的偏差正负值直接就是答案: 正=发晚了, 负=发早了。
// 只有固件算得出它 —— 采样时刻取自延迟线**实际读出的那一片**(DelayLine::last_read_slice_us),
// 主机手上没有这个量, 拿链路耗时反推只会得出一个与目标不同量纲的差值。
// 语义与上面三段一致: 遥测窗口内的滚动极值, 发帧后清零。★不取平均★ 一正一负会互相抵消,
// 把"两边各偏了多少"抹成 0。
// ---------------------------------------------------------------------------
// ★报区间(min/max)而不是单个极值★ 偏差里天生含延迟线 100us 时间片的截断量(0..99, 恒为正),
// 只报"|偏差|最大的那次"会把读数系统性推向正侧、负侧永远看不见 —— 而用户要的正是正负两边。
// 一对 min/max 才如实回答"这一窗里最早发了多少、最晚发了多少"。
extern volatile int16_t g_delay_dev_min_us;  // 本窗口内偏差的最小值(最"早"的那次)
extern volatile int16_t g_delay_dev_max_us;  // 本窗口内偏差的最大值(最"晚"的那次)
extern volatile uint8_t g_delay_dev_flags;   // bit0=本窗口真的发出过帧(有观测); bit1=延迟线被钳制

static constexpr uint8_t DELAY_DEV_FLAG_VALID   = 0x01u;
static constexpr uint8_t DELAY_DEV_FLAG_CLAMPED = 0x02u;

static inline void delay_dev_note(int32_t dev_us, bool clamped) {
    int32_t v = dev_us;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    if ((g_delay_dev_flags & DELAY_DEV_FLAG_VALID) == 0u) {
        g_delay_dev_min_us = (int16_t)v;
        g_delay_dev_max_us = (int16_t)v;
    } else {
        if (v < g_delay_dev_min_us) g_delay_dev_min_us = (int16_t)v;
        if (v > g_delay_dev_max_us) g_delay_dev_max_us = (int16_t)v;
    }
    g_delay_dev_flags = (uint8_t)(g_delay_dev_flags | DELAY_DEV_FLAG_VALID |
                                  (clamped ? DELAY_DEV_FLAG_CLAMPED : 0u));
}
