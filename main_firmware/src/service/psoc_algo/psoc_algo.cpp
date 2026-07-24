#include "psoc_algo.h"
#include "psoc_algo_default.h"
#include "../../protocol/psoc/psoc.h"
#include "../../protocol/host_cmd/host_cmd.h"   // HostCmdCrc16::crc16 (CCITT-FALSE)
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
constexpr uint32_t ALGO_BLOB_MAGIC = 0x31474C41u;  // "ALG1"
constexpr const char* ALGO_BLOB_PATH = "/algo.bin";
constexpr uint32_t ALGO_SRC_MAGIC = 0x31435241u;   // "ARC1" (algo source / mapping table)
constexpr const char* ALGO_SRC_PATH = "/algo_src.bin";

#pragma pack(push, 1)
struct AlgoBlob {
    uint32_t magic;
    uint16_t len;
    uint16_t crc16;               // CCITT-FALSE over data[0,len)
    uint8_t  is_default;
    uint8_t  reserved[3];
    uint8_t  data[PSOC_ALGO_MAX_LEN];
    uint16_t rom[PSOC_ALGO_CHANNELS];   // 每通道 16 位只读 ROM
    uint8_t  cfg[8];               // 共享算法可设置变量(ABI cfg[8])
    uint32_t crc32;               // 覆盖前面全部字节
};
#pragma pack(pop)
}  // namespace

PsocAlgo* PsocAlgo::_instance = nullptr;

PsocAlgo::PsocAlgo() : _len(0), _crc16(0), _is_default(true) {
    std::memset(_rom, 0, sizeof(_rom));   // 每通道 ROM 默认 0(算法未下发校准前)
    std::memset(_cfg, 0, sizeof(_cfg));   // 共享可设置变量默认 0
    std::memset(_src, 0, sizeof(_src));
    _src_len = 0;
    _load_default();
}

void PsocAlgo::set_rom(uint8_t ch, uint16_t val) {
    if (ch >= PSOC_ALGO_CHANNELS) return;
    _rom[ch] = val;
    request_save();
}

void PsocAlgo::set_cfg(uint8_t idx, uint8_t val) {
    if (idx >= 8u) return;
    _cfg[idx] = val;
    request_save();
}

void PsocAlgo::set_src(const uint8_t* s, uint16_t n) {
    if (s == nullptr) { _src_len = 0; }
    else {
        if (n > PSOC_ALGO_SRC_MAX) n = PSOC_ALGO_SRC_MAX;
        std::memcpy(_src, s, n);
        _src_len = n;
    }
    request_save();   // 与算法 blob 共用主循环安全窗口落盘
}

PsocAlgo* PsocAlgo::getInstance() {
    if (_instance == nullptr) _instance = new PsocAlgo();
    return _instance;
}

void PsocAlgo::_load_default() {
    uint16_t n = (uint16_t)PSOC_ALGO_DEFAULT_LEN;
    if (n > PSOC_ALGO_MAX_LEN) n = PSOC_ALGO_MAX_LEN;
    std::memcpy(_blob, PSOC_ALGO_DEFAULT, n);
    _len = n;
    _crc16 = PSOC_ALGO_DEFAULT_CRC16;
    _is_default = true;
}

void PsocAlgo::init() {
#ifdef PICO_PLATFORM
    File f = LittleFS.open(ALGO_BLOB_PATH, "r");
    if (!f) return;   // 无存储 → 保留构造时载入的默认
    AlgoBlob blob;
    size_t rd = f.read((uint8_t*)&blob, sizeof(blob));
    f.close();
    if (rd != sizeof(blob) || blob.magic != ALGO_BLOB_MAGIC) return;
    uint32_t crc = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));
    if (crc != blob.crc32) return;
    // ROM 表随算法一起持久化, 与 is_default 无关(每通道校准数据独立于代码 blob 是否默认)。
    std::memcpy(_rom, blob.rom, sizeof(_rom));
    std::memcpy(_cfg, blob.cfg, sizeof(_cfg));   // 共享可设置变量同样与 is_default 无关
    if (blob.is_default != 0u || blob.len == 0u || blob.len > PSOC_ALGO_MAX_LEN) return;  // 默认/非法 → 用内嵌默认(但 ROM 已载入)
    // 二次校验 blob 内容 crc16, 防存储位翻转导致下发坏算法。
    uint16_t c16 = HostCmdCrc16::crc16(blob.data, blob.len);
    if (c16 != blob.crc16) return;
    std::memcpy(_blob, blob.data, blob.len);
    _len = blob.len;
    _crc16 = blob.crc16;
    // 内容等于内嵌默认则视为默认(纠正历史上被误存为 custom 的默认算法)。
    _is_default = (blob.len == (uint16_t)PSOC_ALGO_DEFAULT_LEN && blob.crc16 == PSOC_ALGO_DEFAULT_CRC16);
#endif
    _load_src();   // 算法 C 源(映射表)独立文件, 与 blob 是否默认无关
}

void PsocAlgo::_load_src() {
#ifdef PICO_PLATFORM
    File f = LittleFS.open(ALGO_SRC_PATH, "r");
    if (!f) return;
    uint8_t hdr[10];
    size_t rd = f.read(hdr, sizeof(hdr));
    if (rd != sizeof(hdr)) { f.close(); return; }
    uint32_t magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
    uint16_t len   = (uint16_t)hdr[4] | ((uint16_t)hdr[5] << 8);
    uint32_t crc32 = (uint32_t)hdr[6] | ((uint32_t)hdr[7] << 8) | ((uint32_t)hdr[8] << 16) | ((uint32_t)hdr[9] << 24);
    if (magic != ALGO_SRC_MAGIC || len > PSOC_ALGO_SRC_MAX) { f.close(); return; }
    uint16_t got = (uint16_t)f.read(_src, len);
    f.close();
    if (got != len) { _src_len = 0; return; }
    if (ConfigCRC::calculate_crc32(_src, len) != crc32) { _src_len = 0; return; }
    _src_len = len;
#endif
}

bool PsocAlgo::set_algo(const uint8_t* src, uint16_t src_len, uint16_t src_crc16) {
    if (src == nullptr || src_len == 0u || src_len > PSOC_ALGO_MAX_LEN) return false;
    // 校验 host 提供的 crc16 与内容一致(CCITT-FALSE), 与 PSoC/默认 blob 同一算法。
    if (HostCmdCrc16::crc16(src, src_len) != src_crc16) return false;
    std::memcpy(_blob, src, src_len);
    _len = src_len;
    _crc16 = src_crc16;
    // 内容等于内嵌默认(v3.1 HDR)则标记为默认: 使上位机"读取信息"能正确识别并载入默认 C 源,
    // 而非误判为无源可还原的自定义算法。
    _is_default = (src_len == (uint16_t)PSOC_ALGO_DEFAULT_LEN && src_crc16 == PSOC_ALGO_DEFAULT_CRC16);
    request_save();
    return true;
}

void PsocAlgo::reset_default() {
    _load_default();
    _src_len = 0;   // 恢复默认: 清源, host 端读回为空时会载入内嵌默认模板
    request_save();
}

bool PsocAlgo::download_to_psoc(Psoc* psoc) {
    if (psoc == nullptr || !psoc->link_ok() || _len == 0u) return false;
    if (!psoc->upload_algo(_blob, _len, _crc16)) return false;
    // 代码下发成功后推送每通道 ROM(算法所需的 per-channel 常量)。非零才推, 省事务; 失败不阻断。
    for (uint8_t ch = 0; ch < PSOC_ALGO_CHANNELS; ++ch) {
        if (_rom[ch] != 0u) { (void)psoc->set_algo_rom(ch, _rom[ch]); }
    }
    // 同时推送共享可设置变量(cfg[8]), 使复位/重刷代码后仍恢复上次设置。全量推(含0), 与 ROM
    // 的"非零才推"不同: cfg[idx]=0 是合法且常见的默认设定值, 不能用非零判定省略。
    for (uint8_t idx = 0; idx < 8u; ++idx) {
        (void)psoc->algo_set_cfg(idx, _cfg[idx]);
    }
    return true;
}

bool PsocAlgo::save() {
    _save_pending = false;
#ifdef PICO_PLATFORM
    AlgoBlob blob;
    std::memset(&blob, 0, sizeof(blob));
    blob.magic = ALGO_BLOB_MAGIC;
    blob.len = _len;
    blob.crc16 = _crc16;
    blob.is_default = _is_default ? 1u : 0u;
    std::memcpy(blob.data, _blob, (_len <= PSOC_ALGO_MAX_LEN) ? _len : PSOC_ALGO_MAX_LEN);
    std::memcpy(blob.rom, _rom, sizeof(blob.rom));
    std::memcpy(blob.cfg, _cfg, sizeof(blob.cfg));
    blob.crc32 = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));

    // 双核安全 flash 写(同 csd_config/config_manager): 禁中断 + 暂停 core1, 保护 XIP 擦写窗口。
    uint32_t _irq = save_and_disable_interrupts();
    multicore_lockout_start_blocking();
    File f = LittleFS.open(ALGO_BLOB_PATH, "w");
    size_t n = 0;
    if (f) {
        n = f.write((const uint8_t*)&blob, sizeof(blob));
        f.close();
    }
    // 同一安全窗口内顺带落盘算法 C 源(映射表), 复用已暂停的 core1 lockout。
    {
        uint8_t hdr[10];
        uint32_t src_crc = ConfigCRC::calculate_crc32(_src, _src_len);
        hdr[0] = (uint8_t)ALGO_SRC_MAGIC;       hdr[1] = (uint8_t)(ALGO_SRC_MAGIC >> 8);
        hdr[2] = (uint8_t)(ALGO_SRC_MAGIC >> 16); hdr[3] = (uint8_t)(ALGO_SRC_MAGIC >> 24);
        hdr[4] = (uint8_t)_src_len;             hdr[5] = (uint8_t)(_src_len >> 8);
        hdr[6] = (uint8_t)src_crc;              hdr[7] = (uint8_t)(src_crc >> 8);
        hdr[8] = (uint8_t)(src_crc >> 16);      hdr[9] = (uint8_t)(src_crc >> 24);
        File sf = LittleFS.open(ALGO_SRC_PATH, "w");
        if (sf) {
            sf.write(hdr, sizeof(hdr));
            if (_src_len > 0u) { sf.write(_src, _src_len); }
            sf.close();
        }
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
