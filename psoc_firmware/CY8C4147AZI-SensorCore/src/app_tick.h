/*******************************************************************************
 * app_tick.h —— 全工程唯一的 1ms 时基声明
 *
 * ★定义在 main.c★(SysTick 回调是启动序列的一部分, 见 systick_ms_callback), 而链路、算法、
 * 长操作各层都要读它。C 里无法让上层的定义被下层的 static inline 看见, 故单独一个只含声明
 * 的头: 谁都可以读, 谁都不许写(唯一写者是 SysTick 回调)。
 ******************************************************************************/
#ifndef APP_TICK_H
#define APP_TICK_H

#include <stdint.h>

/* 毫秒计时：SysTick 每 1ms 回调递增。真实 ms 源，见 systick_ms_callback()。 */
extern volatile uint32_t g_ms_tick;

#endif /* APP_TICK_H */
