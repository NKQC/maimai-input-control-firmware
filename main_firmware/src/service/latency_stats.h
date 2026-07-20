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
