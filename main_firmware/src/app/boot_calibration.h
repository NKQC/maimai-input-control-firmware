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
    uint8_t baseline_retry = 0u;
    uint8_t health_primed = 0u;

    void clear() {
        stage = BootCalibrationStage::WAIT_TRUST;
        autotune_req = 0u;
        verify_generation = 0u;
        verify_started_ms = 0u;
        health_started_ms = 0u;
        health_generation = 0u;
        baseline_retry = 0u;
        health_primed = 0u;
    }
};

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
                state.stage = BootCalibrationStage::DONE;
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
            state.stage = BootCalibrationStage::DONE;
            return;
        }

        case BootCalibrationStage::IDAC_START:
            state.stage = BootCalibrationStage::CHANNEL_START;
            return;

        case BootCalibrationStage::IDAC_WAIT:
            state.stage = BootCalibrationStage::CHANNEL_START;
            return;

        case BootCalibrationStage::CHANNEL_START:
            if (!ConfigManager::get_bool("calib.boot_channel")) {
                state.stage = BootCalibrationStage::BASELINE_START;
                return;
            }
            if (psoc->heavy_busy()) return;
            if (!psoc->auto_tune_start(0xFFu, ConfigManager::get_uint8("calib.pref"), 0x3Eu)) {
                boot_calibration_fail(3u);
                state.stage = BootCalibrationStage::BASELINE_START;
                return;
            }
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

        case BootCalibrationStage::BASELINE_START:
            state.stage = BootCalibrationStage::DONE;
            return;

        case BootCalibrationStage::BASELINE_WAIT:
            state.stage = BootCalibrationStage::DONE;
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
            csd->note_baseline_untrusted(true);
            state.stage = BootCalibrationStage::DONE;
            return;

        case BootCalibrationStage::DONE:
            return;
    }
}

}  // namespace app
