#pragma once

// ======================================================================
// core1 生命周期（传感器核）
//
// 边界: 本文件只负责"把 core1 拉起来并交出 SPI 所有权"这一件事。
//   1) 注册 lockout victim —— 这是 core0 落 flash 的前置硬要求;
//   2) 把本核永久交给 Psoc::core1_run()(恒定周期的触控快路 + 快照慢路 + 命令环消费)。
// core1 的每周期实际内容属于 Psoc 的职责, 实现在 Psoc::_spi_service();
// 那里才是"传感器核状态机"的本体, 不在 app 层复制一份编排。
// core0 自此不再触碰 SPI, 只经 seqlock 读共享态、经命令队列投递指令。
// ======================================================================

// config.h 先于任何 Arduino 变体头: 见 core0_loop.h 同处说明。
#include "../config.h"

#include <stdint.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include "../protocol/psoc/psoc.h"

namespace app {

// ★双核 flash 竞态根治★: core0 跑 Arduino setup/loop(USB + 触控快路 + flash 落地),
// core1 必须已 multicore_lockout_victim_init(), 否则 config_manager 的
// save_and_disable_interrupts() + multicore_lockout_start_blocking() 会永久死锁。
constexpr uint32_t CORE1_STACK_SIZE_BYTES = 0x2000u;
inline uint32_t __attribute__((aligned(8)))
    core1_stack[CORE1_STACK_SIZE_BYTES / sizeof(uint32_t)];

inline void core1_entry() {
    // 注册本核为 lockout 受害者：core0 flash 写时经 SIO IRQ 暂停本核(IRQ 抢占, 与下面永不返回无关)。
    multicore_lockout_victim_init();
    // ★core1 接管 PSoC SPI★：固定周期独占传感器循环, 保证传感器/键盘总延迟每周期一致。
    Psoc::getInstance()->core1_run();   // 永不返回
}

// 必须在任何 flash 写(loop 的落盘窗口)之前调用, 使 lockout 有已注册的 victim 可暂停。
inline void core1_launch() {
    multicore_launch_core1_with_stack(core1_entry, core1_stack, sizeof(core1_stack));
}

}  // namespace app
