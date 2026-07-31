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
#include "service/usb_debug.h"

#ifdef PICO_PLATFORM
#include <hardware/structs/watchdog.h>
#include <hardware/watchdog.h>

// ★flash 写窗口必须放宽看门狗★
// 实测(scratch[0]/[1] 死前遗言): last_boot_was_wd=1 且 last_crash_stage=1(CFG_FLASH) ——
// 保存配置时确确实实是被 5s 看门狗咬死的。整个 config_save 跑在 save_and_disable_interrupts()
// 之后, 而 LittleFS 的 close/sync 是一次性长操作(写元数据 + 磨损搬移), 内部没有可插喂狗的点,
// 靠"分片写 + 每片喂狗"覆盖不到它。故在临界区期间把看门狗周期整体放宽, 退出时恢复 ——
// 这样不依赖能否往库内部插喂狗点。
// 放宽期间若真跑飞, 恢复延迟到 FLASH_WD_MS; 保存是用户主动操作且有 UI 反馈, 这个代价可接受。
// ★8300 而不是更大★: RP2040 看门狗是 24 位计数器 + 2us tick, 硬件上限约 8388ms。
// 先前写 30000 会被截断/失效, 等于没放宽 —— 实测放宽后仍被咬死就是这个原因。
// 必须与 main.cpp 的 WATCHDOG_TIMEOUT_MS 一致; 改那边记得同步这里。
static constexpr uint32_t NORMAL_WD_MS = 5000u;
// NvStore 一次写约 100ms(擦 2 扇区 + 编程), 5s 看门狗绰绰有余 ⇒ **不再放宽**。
// 放宽的代价是真跑飞时晚几秒才恢复; 既然不需要就不留。
static constexpr uint32_t FLASH_WD_MS  = NORMAL_WD_MS;

struct FlashWriteGuard {
    FlashWriteGuard() {
        // LittleFS 擦写会使 XIP 暂停，不能安全分片；先阻止调度器产生遥测/进度 IN 帧，
        // 再进入临界区，避免 64B vendor FIFO 在 USB 无法服务时被继续填满。
        g_usb_flash_busy += 1u;
        watchdog_hw->scratch[7] += 1u;
        watchdog_enable(FLASH_WD_MS, true);
        watchdog_update();
    }
    ~FlashWriteGuard() {
        watchdog_update();
        watchdog_enable(NORMAL_WD_MS, true);
        watchdog_hw->scratch[7] -= 1u;
        g_usb_flash_busy -= 1u;
    }

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
