#include "psoc_updater.h"

#include <Arduino.h>
#include <pico/stdlib.h>
#include <hardware/watchdog.h>

#include "../../protocol/psoc/psoc.h"
#include "../../protocol/psoc/psoc_fw_image.h"
#include "../../hal/usb/hal_usb.h"
#include "../tx_scheduler/tx_scheduler.h"
#include "../usb_comm/usb_comm.h"

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
constexpr uint16_t FLAG_SKIPPED_FLASH = 1u << 15;
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
           (erase_scan_complete ? FLAG_ERASE_SCAN_COMPLETE : 0) |
           (skipped_flash ? FLAG_SKIPPED_FLASH : 0);
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
    SwdProgrammer::set_keepalive(nullptr);
    return false;
}

bool PsocUpdater::run(Psoc* psoc, bool force_flash) {
    // setup() may call this before loop() exists. Keep TinyUSB responsive while
    // SWD verify/program operations poll so enumeration is not delayed by PSoC bring-up.
    SwdProgrammer::set_keepalive(&PsocUpdater::_keepalive);
    init();
    _report.last_stage = PsocBringupStage::SWD_READY;
    if (!psoc || !psoc->swd_ready()) {
        return _fail(psoc, PsocBringupStage::SWD_READY);
    }

    _report.last_stage = PsocBringupStage::INDICATOR_APP;
    // ★运行期救砖不走 prepare_flash_indicator★: 它内部要发一条 SPI(indicator_on), 而运行期 PSoC SPI
    // 由 core1 独占(PIO1), core0 再发就会撞车。白灯准备纯属指示, 故 force 路径只做 XRES 复位到已知态。
    if (force_flash) {
        psoc->reset_run();
        sleep_ms(75);
        _report.indicator_app_ok = false;
    } else {
        _report.indicator_app_ok = psoc->prepare_flash_indicator();
    }

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

    // ★烧录版本/内容检查(省 PSoC flash 寿命)★：每次上电无条件擦写会快速耗尽 PSoC flash 擦写次数。
    // erase 前先只读 AHB 逐字比对当前 flash 与嵌入镜像; 一致则跳过 erase/program/verify, 直接运行。
    // 只读比对不消耗寿命; 内容不同(升级/损坏)时才擦写, 天然等价"版本检查"。
    // ★救砖(force_flash)不走短路★: 目标是"无条件重刷"(内容一致但 RAM 态/扫描引擎已崩时也要恢复)。
    _report.last_stage = PsocBringupStage::VERIFY;
    if (!force_flash && psoc->verify(PSOC_FW_IMAGE, PSOC_FW_IMAGE_LEN)) {
        _report.skipped_flash = true;
        _report.erase_ok = true;
        _report.program_ok = true;
        _report.checksum_ok = true;
        _report.verify_ok = true;
        _report.verify_acquired = true;
        psoc->reset_run();
        psoc->release_swd();
        sleep_ms(50);
        _report.last_stage = PsocBringupStage::RUN;
        SwdProgrammer::set_keepalive(nullptr);
        return true;
    }
    // 内容不一致 → 需要烧录, 记录首个不符地址供诊断。
    _report.fail_addr = psoc->last_fail_addr();

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
    SwdProgrammer::set_keepalive(nullptr);
    return true;
}

// ---------------- 救砖: 强制重刷 + 重新应用 ----------------

void PsocUpdater::rescue_request() {
    _rescue_pending = true;
    _rescue_phase = PsocRescuePhase::FLASHING;
    _rescue_result = 0;
}

uint8_t PsocUpdater::rescue_state() const {
    switch (_rescue_phase) {
        case PsocRescuePhase::IDLE:    return 0u;
        case PsocRescuePhase::DONE:
        case PsocRescuePhase::FAILED:  return 2u;
        default:                       return 1u;
    }
}

// SWD 擦写/校验内层循环的保活: 喂狗 + 泵完整 HostCmd 收发 + 推送阶段进度。
// setup() 尚未进入 loop() 时，只有 UsbComm::update() 会解码 HELLO 并生成 DEVICE_INFO；
// 单独调用 TinyUSB task 只能完成枚举，不能服务 vendor 命令。
void PsocUpdater::_keepalive() {
    watchdog_update();
    HAL_USB_Device::getInstance()->task();
    UsbComm::getInstance()->update();
    TxScheduler::getInstance()->tick();
    HAL_USB_Device::getInstance()->task();
}

bool PsocUpdater::rescue_step(Psoc* psoc) {
    if (!_rescue_pending || psoc == nullptr) return false;
    _rescue_pending = false;

    // ACK 已由 UsbComm 写入 TX FIFO: 先泵几轮把它送出, 再进入数秒的 SWD 擦写窗口。
    for (uint8_t i = 0; i < 8u; i++) {
        HAL_USB_Device::getInstance()->task();
        TxScheduler::getInstance()->tick();
        sleep_ms(1);
    }

    SwdProgrammer::set_keepalive(&PsocUpdater::_keepalive);
    const bool ok = run(psoc, /*force_flash=*/true);   // 复用启动期完整流程(acquire→擦→写→校验→运行)
    SwdProgrammer::set_keepalive(nullptr);             // 启动期与常规路径不再保活(避免意外重入)

    if (!ok) {
        _rescue_phase = PsocRescuePhase::FAILED;
        _rescue_result = 2u;
        return false;
    }
    // 重刷后 PSoC RAM 内的算法/CSD 参数全部丢失 → 交回主循环的 provisioning 重新下发(唯一入口)。
    _rescue_phase = PsocRescuePhase::REAPPLY;
    _rescue_deadline_ms = millis() + 8000u;   // 链路+下发正常只需数百 ms; 超时判失败并交回兜底
    return true;
}

void PsocUpdater::rescue_note_reapplied() {
    if (_rescue_phase != PsocRescuePhase::REAPPLY) return;
    _rescue_phase = PsocRescuePhase::DONE;
    _rescue_result = 1u;
}

void PsocUpdater::update() {
    // 救砖"重新应用"超时兜底: 退出 REAPPLY 使 rescue_active() 转假, 失效兜底(XRES)重新生效。
    if (_rescue_phase == PsocRescuePhase::REAPPLY &&
        (int32_t)(millis() - _rescue_deadline_ms) >= 0) {
        _rescue_phase = PsocRescuePhase::FAILED;
        _rescue_result = 2u;
    }
    Psoc* psoc = Psoc::getInstance();
    _report.link_ok = psoc->link_ok();
    _report.snapshot_valid = psoc->snapshot_valid();
    _report.psoc_generation = psoc->snapshot_generation();
    if (_report.flash_ok() && _report.link_ok && _report.snapshot_valid &&
        _report.psoc_generation != 0) {
        _report.last_stage = PsocBringupStage::COMPLETE;
    }
}
