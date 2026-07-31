#include "psoc_algo.h"
#include "psoc_algo_default.h"
#include "../../protocol/psoc/psoc.h"
#include "../../protocol/host_cmd/host_cmd.h"   // HostCmdCrc16::crc16 (CCITT-FALSE)
#include "../config_manager/config_crc.h"
#include "../../flash_guard.h"
#include "../../hal/usb/hal_usb.h"
#include <cstring>
#include "../usb_debug.h"

#ifdef PICO_PLATFORM
#include "LittleFS.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "../nv_store/nv_store.h"   // 自管理 flash blob(替代 LittleFS 文件)
#endif

namespace {
constexpr uint32_t ALGO_BLOB_MAGIC = 0x31474C41u;  // "ALG1"
constexpr const char* ALGO_BLOB_PATH = "/algo.bin";
constexpr uint32_t ALGO_SRC_MAGIC = 0x31435241u;   // "ARC1" (algo source / mapping table)
constexpr const char* ALGO_SRC_PATH = "/algo_src.bin";

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
    _sync_bin_storage();
    request_save();
}

void PsocAlgo::set_cfg(uint8_t idx, uint8_t val) {
    if (idx >= 8u) return;
    _cfg[idx] = val;
    _sync_bin_storage();
    request_save();
}

bool PsocAlgo::set_src_chunk(uint32_t offset, uint32_t total, const uint8_t* s, uint32_t n) {
    if (total > PSOC_ALGO_SRC_MAX) return false;
    if (offset > total || n > total - offset) return false;
    if (total != 0u && n == 0u) return false;
    if (offset == 0u) _src_wr = 0u;        // offset=0 即"新一轮传输", 允许中途重来
    if (offset != _src_wr) return false;   // 严格连续: 缺片/乱序会拼出半份源, 宁可失败
    if (n > 0u) {
        if (s == nullptr) return false;
        std::memcpy(_src + offset, s, n);
    }
    _src_wr = offset + n;
    if (_src_wr < total) return true;       // 未收完: 不动生效长度, 不置脏
    _src_len = total;                      // 最后一片才让新源生效(total=0 → 清空)
    _src_wr = 0u;                          // 已提交；下一轮传输必须重新从 offset 0 开始
    _sync_src_storage();
    request_save();                        // 保留 SAVE_CONFIG 兜底，不参与实际落盘时机
    return true;
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
    // ALGO_BIN 校验只决定是否采用自定义二进制；ALGO_SRC 必须始终独立加载，不能因默认/损坏
    // 的二进制镜像提前 return 而丢弃已校验的源码区。
    Mirror& blob = _mirror;
    if (_mirror_len == sizeof(blob) && blob.magic == ALGO_BLOB_MAGIC) {
        const uint32_t crc = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));
        if (crc == blob.crc32) {
            std::memcpy(_rom, blob.rom, sizeof(_rom));
            std::memcpy(_cfg, blob.cfg, sizeof(_cfg));
            if (blob.is_default == 0u && blob.len > 0u && blob.len <= PSOC_ALGO_MAX_LEN &&
                HostCmdCrc16::crc16(blob.data, blob.len) == blob.crc16) {
                std::memcpy(_blob, blob.data, blob.len);
                _len = blob.len;
                _crc16 = blob.crc16;
                _is_default = (blob.len == (uint16_t)PSOC_ALGO_DEFAULT_LEN &&
                               blob.crc16 == PSOC_ALGO_DEFAULT_CRC16);
            }
        }
    }
#endif
    _load_src();
}

void PsocAlgo::_load_src() {
#ifdef PICO_PLATFORM
    // ★"C 源只读回一半"的老出处★: 原实现自己拼 10 字节头 + 两次 LittleFS read, 文件系统一不一致
    // 就读出截断内容。现在 _src 由 NvStore 直接注册, 区头自带 len + CRC32, 一次 memcpy 摊回,
    // 长度不符或 CRC 不过整区丢弃 —— 不可能出现半份源码。
    if (_src_store_len > PSOC_ALGO_SRC_MAX) { _src_len = 0; _src_store_len = 0; return; }
    _src_len = _src_store_len;
#endif
    _src_wr = 0u;   // 已加载源只影响有效长度；新上传始终要求从 offset 0 开始
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
    _sync_bin_storage();
    request_save();
    return true;
}

void PsocAlgo::reset_default() {
    _load_default();
    _src_len = 0;
    _src_wr = 0;
    _sync_bin_storage();
    _sync_src_storage();
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

void PsocAlgo::_sync_bin_storage() {
#ifdef PICO_PLATFORM
    Mirror& blob = _mirror;
    std::memset(&blob, 0, sizeof(blob));
    blob.magic = ALGO_BLOB_MAGIC;
    blob.len = _len;
    blob.crc16 = _crc16;
    blob.is_default = _is_default ? 1u : 0u;
    std::memcpy(blob.data, _blob, (_len <= PSOC_ALGO_MAX_LEN) ? _len : PSOC_ALGO_MAX_LEN);
    std::memcpy(blob.rom, _rom, sizeof(blob.rom));
    std::memcpy(blob.cfg, _cfg, sizeof(blob.cfg));
    blob.crc32 = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));
    _mirror_len = sizeof(_mirror);
    NvStore::getInstance()->mark_dirty(NvStore::Region::ALGO_BIN);
#endif
}

void PsocAlgo::_sync_src_storage() {
#ifdef PICO_PLATFORM
    _src_store_len = _src_len;
    NvStore::getInstance()->mark_dirty(NvStore::Region::ALGO_SRC);
#endif
}

bool PsocAlgo::save() {
    _save_pending = false;
    _sync_bin_storage();
    _sync_src_storage();
#ifdef PICO_PLATFORM
    return true;
#else
    return false;
#endif
}

// ★落盘由 NvStore 单点负责★
// 本类不再自己碰 flash: ALGO_BIN 注册 _mirror, ALGO_SRC 直接注册 _src(它本身就是连续缓冲, 不需要
// 再复制一份镜像 —— 少一份就少一处不一致的可能)。必须在 NvStore::load() 之前调用。
void PsocAlgo::register_storage() {
#ifdef PICO_PLATFORM
    // 编译期核对两个区的容量: 区变小/结构变大都在编译时就炸, 而不是等落盘静默截断。
    static_assert(sizeof(Mirror) <= NvStore::payload_capacity(NvStore::Region::ALGO_BIN),
                  "ALGO_BIN 镜像超出该区一份的负载容量");
    static_assert(PSOC_ALGO_SRC_MAX <= NvStore::payload_capacity(NvStore::Region::ALGO_SRC),
                  "ALGO_SRC 上限超出该区一份的负载容量");
    NvStore* nv = NvStore::getInstance();
    _mirror_len = sizeof(_mirror);
    nv->register_blob(NvStore::Region::ALGO_BIN, (uint8_t*)&_mirror, &_mirror_len, sizeof(_mirror));
    _src_store_len = 0;
    nv->register_blob(NvStore::Region::ALGO_SRC, _src, &_src_store_len, PSOC_ALGO_SRC_MAX);
#endif
}
