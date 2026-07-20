#pragma once

// ============================================================================
// Flash 写引用计数守卫（看门狗自持恢复用）
// ----------------------------------------------------------------------------
// 思想（借鉴 Linux 引用计数）：进入 flash 写临界区 +1，退出 -1。计数存于
// watchdog scratch[7]（看门狗/软复位保留，仅上电清零）。
//
// 启动时（main setup 最早）判据：
//   * scratch[7] != 0  → 上次在 flash 写临界区内被复位 = flash 写死锁 → 进 BOOTSEL 自持恢复。
//   * scratch[7] == 0  → 首次上电 / 普通"跑飞"看门狗复位 / 主动重启 → 正常重启即可。
//
// 用 RAII 守卫确保 +1/-1 配对（任何 return 路径都自动 -1）。flash 写不嵌套，
// 但计数天然支持嵌套，稳健。
// ============================================================================
#ifdef PICO_PLATFORM
#include <hardware/structs/watchdog.h>

struct FlashWriteGuard {
    FlashWriteGuard() { watchdog_hw->scratch[7] += 1u; }
    ~FlashWriteGuard() { watchdog_hw->scratch[7] -= 1u; }

    FlashWriteGuard(const FlashWriteGuard&) = delete;
    FlashWriteGuard& operator=(const FlashWriteGuard&) = delete;
};
#else
struct FlashWriteGuard {
    FlashWriteGuard() {}
    ~FlashWriteGuard() {}
    FlashWriteGuard(const FlashWriteGuard&) = delete;
    FlashWriteGuard& operator=(const FlashWriteGuard&) = delete;
};
#endif
