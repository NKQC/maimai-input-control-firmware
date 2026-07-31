#pragma once

#include <stdint.h>

// RP_FIRMWARE_VERSION = 编译时间戳(十进制 YYMMDDHHMM, 本地时间), 每次 pio run 由
// pre:gen_build_stamp.py 重新生成; 线上仍是 u32, 诊断结构与帧格式不变。
#include "rp_build_stamp.h"

class Psoc;

static constexpr uint32_t RP_BUILD_ID = 0x4D324401u;  // "M2D" diagnostic image v1
static constexpr uint8_t PSOC_BRINGUP_REPORT_VERSION = 1u;

enum class PsocBringupStage : uint8_t {
    NONE = 0,
    SWD_READY = 1,
    INDICATOR_APP = 2,
    ACQUIRE = 3,
    SILICON_ID = 4,
    INDICATOR_SWD = 5,
    PROTECTION = 6,
    IMO = 7,
    ERASE = 8,
    PROGRAM = 9,
    CHECKSUM = 10,
    VERIFY_ACQUIRE = 11,
    VERIFY = 12,
    RUN = 13,
    COMPLETE = 14,
};

struct PsocBringupReport {
    uint32_t rp_firmware_version = RP_FIRMWARE_VERSION;
    uint32_t rp_build_id = RP_BUILD_ID;
    uint32_t embedded_psoc_version = 0;
    PsocBringupStage last_stage = PsocBringupStage::NONE;
    PsocBringupStage failure_stage = PsocBringupStage::NONE;

    bool indicator_app_ok = false;
    bool indicator_swd_ok = false;
    bool acquired = false;
    bool silicon_id_ok = false;
    bool protection_ok = false;
    bool imo_ok = false;
    bool erase_ok = false;
    bool program_ok = false;
    bool checksum_ok = false;
    bool verify_acquired = false;
    bool verify_ok = false;
    bool link_ok = false;
    bool snapshot_valid = false;
    bool clock_config_ok = false;
    bool erase_scan_complete = false;
    bool skipped_flash = false;   // 本次启动 PSoC flash 内容已与镜像一致, 跳过擦写(省寿命)

    uint32_t idcode = 0;
    uint32_t actual_silicon_id = 0;
    uint8_t chip_protection = 0xFF;
    uint16_t psoc_generation = 0;
    uint16_t fail_row = 0xFFFF;
    uint32_t fail_addr = 0xFFFFFFFFu;
    uint32_t last_srom_status = 0;
    uint32_t acquire_status = 0;
    uint32_t acquire_sysreq = 0;
    uint32_t acquire_delay = 0xFFFFFFFFu;
    uint32_t erase_status = 0;
    uint32_t program_status = 0;
    uint32_t checksum_srom = 0;
    uint32_t checksum_value = 0;
    uint32_t verify_read = 0;
    uint32_t verify_expect = 0;
    uint32_t row_protection = 0;
    uint32_t clock_select = 0;
    uint32_t clock_imo_select = 0;
    uint32_t clock_trim1 = 0;
    uint32_t clock_trim2 = 0;
    uint32_t clock_trim3 = 0;
    uint32_t erase_flash_sum = 0;
    uint32_t erase_flash_or = 0;
    uint32_t erase_first_nonzero_addr = 0xFFFFFFFFu;
    uint32_t erase_first_nonzero_value = 0;
    uint32_t erase_words_read = 0;

    uint16_t flags() const;
    bool flash_ok() const { return acquired && silicon_id_ok && erase_ok && program_ok && verify_ok; }
};

// ★救砖(PSOC_RESCUE)状态★: 主机命令受理后由主循环执行强制重刷 + 重新应用, 经推送流上报。
// state: 0=空闲 1=进行中 2=完成; stage 复用 PsocBringupStage(擦除/写入/校验/运行…),
// REAPPLY 借用 COMPLETE 前的语义由 phase 字段区分(见 rescue_phase)。
enum class PsocRescuePhase : uint8_t {
    IDLE = 0,
    FLASHING = 1,    // SWD 强制重刷中(acquire/erase/program/verify)
    REAPPLY = 2,     // 已复位运行, 等主循环 provisioning 重新下发算法 + CSD
    DONE = 3,
    FAILED = 4,
};

class PsocUpdater {
public:
    static PsocUpdater* getInstance();

    bool init();
    // force_flash=true: 跳过"内容一致则免烧"的省寿命短路, 无条件擦写+校验(救砖用)。
    bool run(Psoc* psoc, bool force_flash = false);
    void update();
    const PsocBringupReport& report() const { return _report; }

    // ---------- 救砖(强制重刷 + 重新应用) ----------
    void rescue_request();                       // host handler 调用: 置位后立即返回(不阻塞 core0)
    bool rescue_active() const {                 // 进行中(重刷或等重新应用): 期间禁止兜底 XRES
        return _rescue_pending || _rescue_phase == PsocRescuePhase::FLASHING ||
               _rescue_phase == PsocRescuePhase::REAPPLY;
    }
    // 主循环每轮调用: 有待处理请求则执行强制重刷(内部保活喂狗/泵 USB/推进度)。
    // 返回 true = 本轮刚完成重刷, 调用方需清 provisioned 使算法/CSD 重新下发。
    bool rescue_step(Psoc* psoc);
    void rescue_note_reapplied();                // provisioning 重新下发完成 → 终态
    uint8_t rescue_state() const;                // 0=空闲 1=进行中 2=完成
    uint8_t rescue_phase() const { return static_cast<uint8_t>(_rescue_phase); }
    uint8_t rescue_result() const { return _rescue_result; }   // 0=进行中 1=成功 2=失败

private:
    PsocUpdater();
    PsocUpdater(const PsocUpdater&) = delete;
    PsocUpdater& operator=(const PsocUpdater&) = delete;

    bool _fail(Psoc* psoc, PsocBringupStage stage);
    void _capture_acquire(Psoc* psoc);
    static void _keepalive();                    // 长擦写期间: 喂狗 + 泵 USB + 推送进度

    PsocBringupReport _report;
    bool _rescue_pending = false;
    PsocRescuePhase _rescue_phase = PsocRescuePhase::IDLE;
    uint8_t _rescue_result = 0;
    // 重新应用阶段的兜底截止时刻: 链路始终不回来时必须退出 REAPPLY, 否则 rescue_active() 恒真会
    // 永久屏蔽 needs_reset 兜底复位(失效兜底被关掉比救砖失败更危险)。
    uint32_t _rescue_deadline_ms = 0;
    static PsocUpdater* _instance;
};
