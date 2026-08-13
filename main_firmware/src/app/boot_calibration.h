#pragma once

// ======================================================================
// 启动可用性状态机（core0 私有）
//
// 职责边界: 每个 provisioning 代次只走一遍, 判定"这颗 PSoC 是否真的在扫描"。
// 所有重操作都只负责入队, core0 每轮仅轮询一次状态, 绝不在此阻塞。
// 从 main.cpp 独立出来的理由: 它是一台完整的状态机, 与主循环的阶段编排是两件事;
// 混在 main 的匿名 namespace 里会让"每轮做什么"与"启动怎么收敛"互相淹没。
// ======================================================================

// config.h 先于 <Arduino.h>: 见 core0_loop.h 同处说明(PIN_SPI1_* constexpr 与变体头宏同名)。
#include "../config.h"

#include <stdint.h>
#include <Arduino.h>

#include "../protocol/psoc/psoc.h"
#include "../service/csd_config/csd_config.h"
#include "../service/config_manager/config_manager.h"
#include "../service/self_heal/self_heal.h"

namespace app {

enum class BootCalibrationStage : uint8_t {
    WAIT_TRUST,
    IDAC_START,
    IDAC_WAIT,
    CHANNEL_START,
    CHANNEL_WAIT,
    BASELINE_START,
    BASELINE_WAIT,
    VERIFY_WAIT,
    DONE,
};

struct BootCalibrationState {
    BootCalibrationStage stage = BootCalibrationStage::WAIT_TRUST;
    uint32_t autotune_req = 0u;
    uint32_t verify_generation = 0u;
    uint32_t verify_started_ms = 0u;
    uint32_t health_started_ms = 0u;
    uint32_t health_generation = 0u;
    // 重操作在途计时起点: heavy_busy() 由 core1 清除, 万一 core1 卡死也必须能收敛到 DONE,
    // 否则整条流水线连同它后面的验收会永久停在 *_WAIT。
    uint32_t heavy_started_ms = 0u;
    // 本档"启动"的首次尝试时刻(0=还没真正尝试过)。见 BOOT_CAL_START_RETRY_MS。
    uint32_t start_try_ms = 0u;
    // 各档结局位图, 经 UsbDebugCounters::boot_cal_fail_mask 上报(见该字段说明)。
    uint8_t fail_mask = 0u;
    uint8_t baseline_retry = 0u;
    uint8_t health_primed = 0u;

    void clear() {
        stage = BootCalibrationStage::WAIT_TRUST;
        autotune_req = 0u;
        verify_generation = 0u;
        verify_started_ms = 0u;
        health_started_ms = 0u;
        health_generation = 0u;
        heavy_started_ms = 0u;
        start_try_ms = 0u;
        fail_mask = 0u;
        baseline_retry = 0u;
        health_primed = 0u;
    }
};

// 各档结局位(与 UsbDebugCounters::boot_cal_fail_mask 同一编码)。
constexpr uint8_t BOOT_CAL_SKIP_IDAC      = 0x01;  // 该档 KV 关闭, 按用户意图跳过
constexpr uint8_t BOOT_CAL_FAIL_IDAC      = 0x02;  // 该档启动失败(重试窗口耗尽)
constexpr uint8_t BOOT_CAL_SKIP_CHANNEL   = 0x04;
constexpr uint8_t BOOT_CAL_FAIL_CHANNEL   = 0x08;
constexpr uint8_t BOOT_CAL_SKIP_BASELINE  = 0x10;
constexpr uint8_t BOOT_CAL_FAIL_BASELINE  = 0x20;
constexpr uint8_t BOOT_CAL_FAIL_VERIFY    = 0x40;  // 三档跑完但末尾快照/基线验收未通过
constexpr uint8_t BOOT_CAL_RAN_SOMETHING  = 0x80;  // 至少有一档真的把重操作交给了 PSoC

// 单条重操作的在途上限。PSoC 全通道校准实测 12s+、基线复位远快于此; 取 90s 只为"core1 真卡死时
// 也能收敛", 不作为正常路径的判据。
constexpr uint32_t BOOT_CAL_HEAVY_TIMEOUT_MS = 90000u;

// 每档"启动"允许的重试窗口。
// ★为什么必须重试★ provisioning 刚落地那一瞬, core1 的命令环里还压着 396 条 SET_PARAM 与随后的
// APPLY, `_submit` 的入队自旋只有 100ms 预算, 拿不到空位就返回 false —— 这是**可预期的暂时性
// 拥塞**, 不是"这档做不了"。实测(诊断字节 stage 追踪)整条流水线在启动后 4s 内空转到 VERIFY_WAIT,
// IDAC 与频率自适应两档都是这样被一次入队失败判死的, 只有排在最后的基线复位赶上了空环。
// 窗口内 heavy_busy 期间不计时(见调用处), 所以 8s 全部用在真正的入队尝试上。
constexpr uint32_t BOOT_CAL_START_RETRY_MS = 8000u;

// 三档开机校准是否有任何一项被启用。全关 ⇒ 整条流水线不该被走一遍(连末尾的快照验收也不做),
// 直接 DONE, 行为与"开机不校准"完全一致。
inline bool boot_calibration_any_enabled() {
    return ConfigManager::get_bool("calib.boot_idac") ||
           ConfigManager::get_bool("calib.boot_channel") ||
           ConfigManager::get_bool("calib.boot_baseline");
}

constexpr uint32_t BOOT_CAL_FAILURE_FLAG = 0x80000000u;

inline void boot_calibration_fail(uint8_t stage) {
    // 复用既有 SH_REPROVISIONED 事件；bit31 区分普通重下发 detail(0/1)，不新增事件码。
    SelfHeal::getInstance()->note(SH_REPROVISIONED, BOOT_CAL_FAILURE_FLAG | stage);
}

inline void boot_calibration_tick(Psoc* psoc, CsdConfig* csd, BootCalibrationState& state) {
    if (psoc == nullptr || csd == nullptr || !psoc->link_alive()) return;

    switch (state.stage) {
        case BootCalibrationStage::WAIT_TRUST: {
            // readiness-first：连接建立后先被动观察真实扫描推进，不无条件发全通道校准。
            // 常态健康设备因此零重操作、全速进入推流/Focus/JIT；只有连续 3s 无扫描吞吐或
            // 快照不可信才进入异步恢复链。观测期间所有健康检测仍运行，并未用“跳过校准”冒充健康。
            if (psoc->heavy_busy()) return;
            const uint32_t now_ms = millis();
            if (state.health_primed == 0u) {
                state.health_primed = 1u;
                state.health_started_ms = now_ms;
                state.health_generation = psoc->snapshot_generation();
                return;
            }
            const bool generation_advanced = psoc->snapshot_valid() &&
                psoc->snapshot_generation() != state.health_generation;
            const bool scan_advanced = psoc->samples_per_sec() != 0u;
            // Link/generation/scan advancement are the startup availability contract.
            // Baseline quality may legitimately be untrusted while electrodes are idle
            // (all-zero raw on this board); it is recorded by CsdConfig but must not
            // re-enter a global calibration chain and delay telemetry/Focus.
            if (generation_advanced && scan_advanced) {
                if (!csd->sampling_trustworthy(psoc)) csd->note_baseline_untrusted(true);
                // 健康设备照旧零重操作直达 DONE —— 除非用户显式勾了开机校准。三档由各自阶段
                // 再逐项门控, 这里只决定"要不要进流水线", 避免全关时白跑一遍末尾验收。
                state.stage = boot_calibration_any_enabled()
                    ? BootCalibrationStage::IDAC_START
                    : BootCalibrationStage::DONE;
                return;
            }
            // Health monitoring remains mandatory, but normal startup must never
            // manufacture a full-panel calibration merely because the first
            // passive window was quiet.  The PSoC release APPLY owns its actual
            // lifecycle through busy; a failed readiness observation is recorded
            // for diagnostics and the existing reset/reprovision path remains the
            // recovery authority.
            if ((uint32_t)(now_ms - state.health_started_ms) < 3000u) return;
            boot_calibration_fail(1u);
            csd->note_baseline_untrusted(true);
            // 观察窗内看不到扫描推进时同样进流水线(用户勾了才做): 校准/基线复位正是这种情况下
            // 唯一能自动尝试的恢复手段。仍然只走一遍、失败不重试, 不构成启动循环。
            state.stage = boot_calibration_any_enabled()
                ? BootCalibrationStage::IDAC_START
                : BootCalibrationStage::DONE;
            return;
        }

        // 全通道 IDAC 重校准。等同"触控全局调整 → 通道操作 → 全通道校准"的那一次动作。
        case BootCalibrationStage::IDAC_START:
            if (!ConfigManager::get_bool("calib.boot_idac")) {
                state.fail_mask |= BOOT_CAL_SKIP_IDAC;
                state.stage = BootCalibrationStage::CHANNEL_START;
                return;
            }
            // heavy 在途不算"尝试过": 把计时起点清掉, 重试窗口只用于真正的入队失败。
            if (psoc->heavy_busy()) { state.start_try_ms = 0u; return; }
            if (state.start_try_ms == 0u) state.start_try_ms = millis();
            if (!psoc->start_boot_calibrate(0xFFu)) {
                if ((uint32_t)(millis() - state.start_try_ms) < BOOT_CAL_START_RETRY_MS) return;
                boot_calibration_fail(2u);
                state.fail_mask |= BOOT_CAL_FAIL_IDAC;
                state.start_try_ms = 0u;
                state.stage = BootCalibrationStage::CHANNEL_START;
                return;
            }
            state.fail_mask |= BOOT_CAL_RAN_SOMETHING;
            state.start_try_ms = 0u;
            state.heavy_started_ms = millis();
            state.stage = BootCalibrationStage::IDAC_WAIT;
            return;

        case BootCalibrationStage::IDAC_WAIT:
            // 非阻塞入队拿不到回执: 完成判据就是 heavy_busy() 由真转假(CALIBRATE 在 heavy 名单内)。
            if (psoc->heavy_busy()) {
                if ((uint32_t)(millis() - state.heavy_started_ms) < BOOT_CAL_HEAVY_TIMEOUT_MS) return;
                boot_calibration_fail(2u);
            }
            state.stage = BootCalibrationStage::CHANNEL_START;
            return;

        case BootCalibrationStage::CHANNEL_START:
            if (!ConfigManager::get_bool("calib.boot_channel")) {
                state.fail_mask |= BOOT_CAL_SKIP_CHANNEL;
                state.stage = BootCalibrationStage::BASELINE_START;
                return;
            }
            if (psoc->heavy_busy()) { state.start_try_ms = 0u; return; }
            if (state.start_try_ms == 0u) state.start_try_ms = millis();
            if (!psoc->auto_tune_start(0xFFu, ConfigManager::get_uint8("calib.pref"), 0x3Eu)) {
                if ((uint32_t)(millis() - state.start_try_ms) < BOOT_CAL_START_RETRY_MS) return;
                boot_calibration_fail(3u);
                state.fail_mask |= BOOT_CAL_FAIL_CHANNEL;
                state.start_try_ms = 0u;
                state.stage = BootCalibrationStage::BASELINE_START;
                return;
            }
            state.fail_mask |= BOOT_CAL_RAN_SOMETHING;
            state.start_try_ms = 0u;
            state.autotune_req = psoc->autotune_req();
            state.stage = BootCalibrationStage::CHANNEL_WAIT;
            return;

        case BootCalibrationStage::CHANNEL_WAIT: {
            // 请求代次被别的调用覆盖时，本轮已失去归属；记失败并继续，绝不重新启动形成循环。
            if (psoc->autotune_req() != state.autotune_req) {
                boot_calibration_fail(3u);
                state.stage = BootCalibrationStage::BASELINE_START;
                return;
            }
            const psoc::AutoTuneProgress status = psoc->autotune_status();
            if (status.req != state.autotune_req || status.state != 2u) return;
            if (status.result != 1u) boot_calibration_fail(3u);
            state.stage = BootCalibrationStage::BASELINE_START;
            return;
        }

        // 全通道基线复位。等同"触控全局调整 → 通道操作 → 全通道基线复位"。
        case BootCalibrationStage::BASELINE_START:
            if (!ConfigManager::get_bool("calib.boot_baseline")) {
                // 未启用基线复位时不做末尾验收: 验收判据里就含 sampling_trustworthy, 而它在电极
                // 空闲时本就可能不可信, 拿它给"用户没要求的动作"判失败只会制造假警报。
                state.fail_mask |= BOOT_CAL_SKIP_BASELINE;
                state.stage = BootCalibrationStage::DONE;
                return;
            }
            if (psoc->heavy_busy()) { state.start_try_ms = 0u; return; }
            if (state.start_try_ms == 0u) state.start_try_ms = millis();
            if (!psoc->start_boot_baseline_reset(0xFFu)) {
                if ((uint32_t)(millis() - state.start_try_ms) < BOOT_CAL_START_RETRY_MS) return;
                boot_calibration_fail(4u);
                state.fail_mask |= BOOT_CAL_FAIL_BASELINE;
                state.start_try_ms = 0u;
                state.stage = BootCalibrationStage::DONE;
                return;
            }
            state.fail_mask |= BOOT_CAL_RAN_SOMETHING;
            state.start_try_ms = 0u;
            state.heavy_started_ms = millis();
            state.stage = BootCalibrationStage::BASELINE_WAIT;
            return;

        case BootCalibrationStage::BASELINE_WAIT:
            if (psoc->heavy_busy()) {
                if ((uint32_t)(millis() - state.heavy_started_ms) < BOOT_CAL_HEAVY_TIMEOUT_MS) return;
                boot_calibration_fail(4u);
                state.stage = BootCalibrationStage::DONE;
                return;
            }
            // 基线复位真的做完了才进验收: 记下当前代次, 要求之后能看到**新的**一份快照。
            state.verify_generation = psoc->snapshot_generation();
            state.verify_started_ms = millis();
            state.stage = BootCalibrationStage::VERIFY_WAIT;
            return;

        case BootCalibrationStage::VERIFY_WAIT:
            if (psoc->heavy_busy()) return;
            if (psoc->snapshot_valid() &&
                psoc->snapshot_generation() != state.verify_generation &&
                csd->sampling_trustworthy(psoc)) {
                state.stage = BootCalibrationStage::DONE;
                return;
            }
            if ((uint32_t)(millis() - state.verify_started_ms) < 3000u) return;
            // 基线是用户手动操作能恢复的最小动作；验收失败时只自动补做一次，禁止形成启动循环。
            if (state.baseline_retry == 0u &&
                ConfigManager::get_bool("calib.boot_baseline")) {
                state.baseline_retry = 1u;
                state.stage = BootCalibrationStage::BASELINE_START;
                return;
            }
            boot_calibration_fail(5u);   // 三项已跑完但快照/基线验收仍失败
            state.fail_mask |= BOOT_CAL_FAIL_VERIFY;
            csd->note_baseline_untrusted(true);
            state.stage = BootCalibrationStage::DONE;
            return;

        case BootCalibrationStage::DONE:
            return;
    }
}

}  // namespace app
