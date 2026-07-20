#include "psoc_updater.h"

#include <pico/stdlib.h>

#include "../../protocol/psoc/psoc.h"
#include "../../protocol/psoc/psoc_fw_image.h"

namespace {
constexpr uint16_t FLAG_INDICATOR_APP = 1u << 0;
constexpr uint16_t FLAG_INDICATOR_SWD = 1u << 1;
constexpr uint16_t FLAG_ACQUIRED = 1u << 2;
constexpr uint16_t FLAG_SILICON_ID = 1u << 3;
constexpr uint16_t FLAG_PROTECTION = 1u << 4;
constexpr uint16_t FLAG_IMO = 1u << 5;
constexpr uint16_t FLAG_ERASE = 1u << 6;
constexpr uint16_t FLAG_PROGRAM = 1u << 7;
constexpr uint16_t FLAG_CHECKSUM = 1u << 8;
constexpr uint16_t FLAG_VERIFY_ACQUIRE = 1u << 9;
constexpr uint16_t FLAG_VERIFY = 1u << 10;
constexpr uint16_t FLAG_LINK = 1u << 11;
constexpr uint16_t FLAG_SNAPSHOT = 1u << 12;
constexpr uint16_t FLAG_CLOCK_CONFIG = 1u << 13;
constexpr uint16_t FLAG_ERASE_SCAN_COMPLETE = 1u << 14;
}

uint16_t PsocBringupReport::flags() const {
    return (indicator_app_ok ? FLAG_INDICATOR_APP : 0) |
           (indicator_swd_ok ? FLAG_INDICATOR_SWD : 0) |
           (acquired ? FLAG_ACQUIRED : 0) |
           (silicon_id_ok ? FLAG_SILICON_ID : 0) |
           (protection_ok ? FLAG_PROTECTION : 0) |
           (imo_ok ? FLAG_IMO : 0) |
           (erase_ok ? FLAG_ERASE : 0) |
           (program_ok ? FLAG_PROGRAM : 0) |
           (checksum_ok ? FLAG_CHECKSUM : 0) |
           (verify_acquired ? FLAG_VERIFY_ACQUIRE : 0) |
           (verify_ok ? FLAG_VERIFY : 0) |
           (link_ok ? FLAG_LINK : 0) |
           (snapshot_valid ? FLAG_SNAPSHOT : 0) |
           (clock_config_ok ? FLAG_CLOCK_CONFIG : 0) |
           (erase_scan_complete ? FLAG_ERASE_SCAN_COMPLETE : 0);
}

PsocUpdater* PsocUpdater::_instance = nullptr;

PsocUpdater::PsocUpdater() {
    init();
}

PsocUpdater* PsocUpdater::getInstance() {
    if (!_instance) {
        _instance = new PsocUpdater();
    }
    return _instance;
}

bool PsocUpdater::init() {
    _report = PsocBringupReport{};
    _report.embedded_psoc_version = PSOC_FW_VERSION;
    return true;
}

void PsocUpdater::_capture_acquire(Psoc* psoc) {
    _report.acquired = psoc->acquire();
    _report.idcode = psoc->idcode();
    _report.acquire_status = psoc->last_acquire_status();
    _report.acquire_sysreq = psoc->last_acquire_sysreq();
    _report.acquire_delay = psoc->last_acquire_delay();
    _report.last_srom_status = psoc->last_srom_status();
}

bool PsocUpdater::_fail(Psoc* psoc, PsocBringupStage stage) {
    _report.failure_stage = stage;
    _report.last_stage = stage;
    if (psoc && psoc->swd_ready()) {
        psoc->reset_run();
        psoc->release_swd();
    }
    return false;
}

bool PsocUpdater::run(Psoc* psoc) {
    init();
    _report.last_stage = PsocBringupStage::SWD_READY;
    if (!psoc || !psoc->swd_ready()) {
        return _fail(psoc, PsocBringupStage::SWD_READY);
    }

    _report.last_stage = PsocBringupStage::INDICATOR_APP;
    _report.indicator_app_ok = psoc->prepare_flash_indicator();

    _report.last_stage = PsocBringupStage::ACQUIRE;
    _capture_acquire(psoc);
    if (!_report.acquired) {
        return _fail(psoc, PsocBringupStage::ACQUIRE);
    }

    _report.last_stage = PsocBringupStage::SILICON_ID;
    const bool silicon_read = psoc->read_silicon_id(&_report.actual_silicon_id);
    _report.last_srom_status = psoc->last_srom_status();
    _report.silicon_id_ok = silicon_read &&
        psoc::is_cy8c4147azi_s455(_report.actual_silicon_id);
    if (!_report.silicon_id_ok) {
        return _fail(psoc, PsocBringupStage::SILICON_ID);
    }

    _report.last_stage = PsocBringupStage::INDICATOR_SWD;
    _report.indicator_swd_ok = psoc->set_program_indicator();

    _report.last_stage = PsocBringupStage::PROTECTION;
    _report.protection_ok = psoc->read_chip_protection(&_report.chip_protection);
    if (!_report.protection_ok) {
        return _fail(psoc, PsocBringupStage::PROTECTION);
    }
    _report.row_protection = psoc->read_row_protection();

    _report.last_stage = PsocBringupStage::IMO;
    _report.imo_ok = psoc->set_imo();
    _report.last_srom_status = psoc->last_srom_status();

    _report.last_stage = PsocBringupStage::ERASE;
    _report.erase_ok = psoc->erase();
    _report.clock_config_ok = psoc->clock_config_ok();
    _report.clock_select = psoc->clock_select();
    _report.clock_imo_select = psoc->clock_imo_select();
    _report.clock_trim1 = psoc->clock_trim1();
    _report.clock_trim2 = psoc->clock_trim2();
    _report.clock_trim3 = psoc->clock_trim3();
    _report.erase_scan_complete = psoc->erase_scan_complete();
    _report.erase_flash_sum = psoc->erase_flash_sum();
    _report.erase_flash_or = psoc->erase_flash_or();
    _report.erase_first_nonzero_addr = psoc->erase_first_nonzero_addr();
    _report.erase_first_nonzero_value = psoc->erase_first_nonzero_value();
    _report.erase_words_read = psoc->erase_words_read();
    _report.erase_status = psoc->last_srom_status();
    _report.last_srom_status = _report.erase_status;
    if (!_report.erase_ok) {
        return _fail(psoc, PsocBringupStage::ERASE);
    }

    _report.last_stage = PsocBringupStage::PROGRAM;
    _report.program_ok = psoc->program_rows(PSOC_FW_IMAGE, PSOC_FW_IMAGE_LEN);
    _report.fail_row = psoc->last_fail_row();
    _report.program_status = psoc->last_srom_status();
    _report.last_srom_status = _report.program_status;
    if (!_report.program_ok) {
        return _fail(psoc, PsocBringupStage::PROGRAM);
    }

    _report.last_stage = PsocBringupStage::CHECKSUM;
    _report.checksum_ok = psoc->checksum(&_report.checksum_value);
    _report.checksum_srom = psoc->last_srom_status();
    _report.last_srom_status = _report.checksum_srom;

    // ★in-session verify★：在 program 的同一 acquire 会话内直接 AHB 读回比对，
    // 不做 reset_run + 重新 acquire，用于定位 fail@0x80 是 program 未写全，还是 reset/re-acquire 读取干扰。
    _report.verify_acquired = true;   // 复用当前会话
    _report.last_stage = PsocBringupStage::VERIFY;
    _report.verify_ok = psoc->verify(PSOC_FW_IMAGE, PSOC_FW_IMAGE_LEN);
    _report.fail_addr = psoc->last_fail_addr();
    _report.verify_read = psoc->last_verify_read();
    _report.verify_expect = psoc->last_verify_expect();
    if (!_report.verify_ok) {
        return _fail(psoc, PsocBringupStage::VERIFY);
    }

    psoc->reset_run();
    psoc->release_swd();
    sleep_ms(50);
    _report.last_stage = PsocBringupStage::RUN;
    return true;
}

void PsocUpdater::update() {
    Psoc* psoc = Psoc::getInstance();
    _report.link_ok = psoc->link_ok();
    _report.snapshot_valid = psoc->snapshot_valid();
    _report.psoc_generation = psoc->snapshot_generation();
    if (_report.flash_ok() && _report.link_ok && _report.snapshot_valid &&
        _report.psoc_generation != 0) {
        _report.last_stage = PsocBringupStage::COMPLETE;
    }
}
