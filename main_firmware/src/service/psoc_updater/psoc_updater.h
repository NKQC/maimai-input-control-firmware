#pragma once

#include <stdint.h>

class Psoc;

static constexpr uint32_t RP_FIRMWARE_VERSION = 0x00000401u;
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

class PsocUpdater {
public:
    static PsocUpdater* getInstance();

    bool init();
    bool run(Psoc* psoc);
    void update();
    const PsocBringupReport& report() const { return _report; }

private:
    PsocUpdater();
    PsocUpdater(const PsocUpdater&) = delete;
    PsocUpdater& operator=(const PsocUpdater&) = delete;

    bool _fail(Psoc* psoc, PsocBringupStage stage);
    void _capture_acquire(Psoc* psoc);

    PsocBringupReport _report;
    static PsocUpdater* _instance;
};
