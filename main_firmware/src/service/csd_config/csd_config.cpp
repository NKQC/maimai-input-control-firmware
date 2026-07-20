#include "csd_config.h"
#include "../../protocol/psoc/psoc.h"
#include "../config_manager/config_crc.h"
#include "../../flash_guard.h"
#include <cstring>
#include "../usb_debug.h"

#ifdef PICO_PLATFORM
#include "LittleFS.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#endif

namespace {
constexpr uint32_t CSD_BLOB_MAGIC = 0x31445343u;  // "CSD1"
constexpr const char* CSD_BLOB_PATH = "/csd.bin";

#pragma pack(push, 1)
struct CsdBlob {
    uint32_t magic;
    uint8_t  mode;
    uint8_t  valid;
    uint16_t reserved;
    uint16_t param[CSD_CHANNELS][CSD_PARAM_COUNT];
    uint32_t crc32;   // 覆盖前面全部字节
};
#pragma pack(pop)
}  // namespace

CsdConfig* CsdConfig::_instance = nullptr;

CsdConfig::CsdConfig() : _mode(CSD_MODE_SEMI), _valid(false) {
    std::memset(_param, 0, sizeof(_param));
}

CsdConfig* CsdConfig::getInstance() {
    if (_instance == nullptr) _instance = new CsdConfig();
    return _instance;
}

void CsdConfig::note_param(uint8_t ch, uint8_t param_id, uint32_t value) {
    if (ch >= CSD_CHANNELS || !_param_id_ok(param_id)) return;
    _param[ch][_param_index(param_id)] = (uint16_t)value;
}

void CsdConfig::init() {
#ifdef PICO_PLATFORM
    File f = LittleFS.open(CSD_BLOB_PATH, "r");
    if (!f) return;
    CsdBlob blob;
    size_t n = f.read((uint8_t*)&blob, sizeof(blob));
    f.close();
    if (n != sizeof(blob) || blob.magic != CSD_BLOB_MAGIC) return;
    uint32_t crc = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));
    if (crc != blob.crc32) return;
    _mode = (blob.mode != 0u) ? CSD_MODE_SEMI : CSD_MODE_AUTO;
    _valid = (blob.valid != 0u);
    std::memcpy(_param, blob.param, sizeof(_param));
#endif
}

bool CsdConfig::save() {
    _save_pending = false;   // 落地即清信号
#ifdef PICO_PLATFORM
    CsdBlob blob;
    std::memset(&blob, 0, sizeof(blob));
    blob.magic = CSD_BLOB_MAGIC;
    blob.mode = _mode;
    blob.valid = _valid ? 1u : 0u;
    std::memcpy(blob.param, _param, sizeof(_param));
    blob.crc32 = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));

    // ★双核安全 flash 写(同 config_manager::save_config_task)★：禁中断 + 暂停 core1，
    // 保护 XIP 擦写窗口，两核都不访问总线→写后 USB 自动恢复，不破坏 vendor 通信。
    uint32_t _irq = save_and_disable_interrupts();
    multicore_lockout_start_blocking();
    File f = LittleFS.open(CSD_BLOB_PATH, "w");
    size_t n = 0;
    if (f) {
        n = f.write((const uint8_t*)&blob, sizeof(blob));
        f.close();
    }
    multicore_lockout_end_blocking();
    restore_interrupts(_irq);
    g_usb_dbg.flash_write_count++;
    g_usb_dbg.loop_at_last_flash = g_usb_dbg.loop_count;
    return n == sizeof(blob);
#else
    return false;
#endif
}

void CsdConfig::download_to_psoc(Psoc* psoc) {
    if (psoc == nullptr || !psoc->link_ok()) return;
    psoc->set_mode(_mode);
    // 半自动模式才需要下发手动参数(全自动由 PSoC SmartSense 每周期自整定)。
    if (_mode == CSD_MODE_SEMI && _valid) {
        for (uint8_t ch = 0; ch < CSD_CHANNELS; ch++) {
            for (uint8_t i = 0; i < CSD_PARAM_COUNT; i++) {
                psoc->set_param(ch, (uint8_t)(CSD_PARAM_ID_MIN + i), _param[ch][i]);
            }
        }
        psoc->apply_params();   // 应用硬件参数(半自动重初始化,不覆盖手动阈值)
    }
}

void CsdConfig::capture_from_psoc(Psoc* psoc) {
    if (psoc == nullptr || !psoc->link_ok()) return;
    for (uint8_t ch = 0; ch < CSD_CHANNELS; ch++) {
        for (uint8_t i = 0; i < CSD_PARAM_COUNT; i++) {
            uint32_t v = 0;
            if (psoc->get_param(ch, (uint8_t)(CSD_PARAM_ID_MIN + i), &v)) {
                _param[ch][i] = (uint16_t)v;
            }
        }
    }
    _valid = true;
}
