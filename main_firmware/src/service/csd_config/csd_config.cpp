#include "csd_config.h"
#include "../../protocol/psoc/psoc.h"
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
constexpr uint32_t CSD_BLOB_MAGIC = 0x31445343u;  // "CSD1"
constexpr const char* CSD_BLOB_PATH = "/csd.bin";

}  // namespace

CsdConfig* CsdConfig::_instance = nullptr;

CsdConfig::CsdConfig() : _mode(CSD_MODE_SEMI), _valid(false) {
    std::memset(_param, 0, sizeof(_param));
    std::memset(_global, 0, sizeof(_global));
}

void CsdConfig::note_mode(uint8_t mode) {
    _mode = (mode != 0u) ? CSD_MODE_SEMI : CSD_MODE_AUTO;
    _sync_storage();
}

void CsdConfig::note_global(uint8_t gparam_id, uint32_t value) {
    if (!_global_id_ok(gparam_id)) return;
    _global[gparam_id - CSD_GLOBAL_ID_MIN] = (uint16_t)value;
    _global_valid = true;
    _sync_storage();
}

CsdConfig* CsdConfig::getInstance() {
    if (_instance == nullptr) _instance = new CsdConfig();
    return _instance;
}

void CsdConfig::note_param(uint8_t ch, uint8_t param_id, uint32_t value) {
    if (ch >= CSD_CHANNELS || !_param_id_ok(param_id)) return;
    _param[ch][_param_index(param_id)] = (uint16_t)value;
    _valid = true;
    _sync_storage();
}

void CsdConfig::clear() {
    std::memset(_param, 0, sizeof(_param));
    std::memset(_global, 0, sizeof(_global));
    _valid = false;
    _global_valid = false;
    _mode = CSD_MODE_AUTO;
    _sync_storage();
    request_save();
}

// ★落盘由 NvStore 单点负责★
// 本类不再自己碰 flash: 只把 _mirror(与 flash 布局等价的镜像)注册给 NvStore, 由它 load/commit。
// 必须在 NvStore::load() 之前调用。
void CsdConfig::register_storage() {
#ifdef PICO_PLATFORM
    static_assert(sizeof(Mirror) <= NvStore::payload_capacity(NvStore::Region::CSD),
                  "CSD 镜像超出该区一份的负载容量");
    _mirror_len = sizeof(_mirror);
    NvStore::getInstance()->register_blob(
        NvStore::Region::CSD, (uint8_t*)&_mirror, &_mirror_len, sizeof(_mirror));
#endif
}

void CsdConfig::init() {
#ifdef PICO_PLATFORM
    // NvStore::load() 已把 flash 内容摊进 _mirror(带 len+CRC 校验), 这里只做本类自己的语义校验。
    // 旧实现在这里直接读文件, 是三条互不知情的落盘路径之一 —— 现在读路径也收敛到 NvStore。
    if (_mirror_len != sizeof(_mirror) || _mirror.magic != CSD_BLOB_MAGIC) return;
    uint32_t crc = ConfigCRC::calculate_crc32((const uint8_t*)&_mirror, sizeof(_mirror) - sizeof(uint32_t));
    if (crc != _mirror.crc32) return;
    _mode = (_mirror.mode != 0u) ? CSD_MODE_SEMI : CSD_MODE_AUTO;
    _valid = (_mirror.valid != 0u);
    _global_valid = (_mirror.global_valid != 0u);
    std::memcpy(_param, _mirror.param, sizeof(_param));
    std::memcpy(_global, _mirror.global, sizeof(_global));
#endif
}

void CsdConfig::_sync_storage() {
#ifdef PICO_PLATFORM
    std::memset(&_mirror, 0, sizeof(_mirror));
    _mirror.magic = CSD_BLOB_MAGIC;
    _mirror.mode = _mode;
    _mirror.valid = _valid ? 1u : 0u;
    _mirror.global_valid = _global_valid ? 1u : 0u;
    std::memcpy(_mirror.param, _param, sizeof(_param));
    std::memcpy(_mirror.global, _global, sizeof(_global));
    _mirror.crc32 = ConfigCRC::calculate_crc32((const uint8_t*)&_mirror, sizeof(_mirror) - sizeof(uint32_t));
    _mirror_len = sizeof(_mirror);
    NvStore::getInstance()->mark_dirty(NvStore::Region::CSD);
#endif
}

bool CsdConfig::save() {
    _save_pending = false;
    _sync_storage();
#ifdef PICO_PLATFORM
    return true;
#else
    return false;
#endif
}

void CsdConfig::download_to_psoc(Psoc* psoc) {
    if (psoc == nullptr || !psoc->link_ok()) return;
    psoc->set_mode(_mode);
    bool need_apply = false;

    // 全局 CSD 配置(未激活传感器连接/IDAC/MFS 等)与扫描模式无关, 有则先全部下发到 RAM 影子,
    // 再 global_commit() 触发【一次】完整重初始化(合并, 防每项重初始化的反复重校准漂移)。
    if (_global_valid) {
        for (uint8_t i = 0; i < CSD_GLOBAL_COUNT; i++) {
            psoc->set_global((uint8_t)(CSD_GLOBAL_ID_MIN + i), _global[i]);
        }
        psoc->global_commit();
    }

    // 半自动手动模式才需要下发手动参数；自动校准模式由 PSoC 运行标准完整处理链。
    if (_mode == CSD_MODE_SEMI && _valid) {
        for (uint8_t ch = 0; ch < CSD_CHANNELS; ch++) {
            for (uint8_t i = 0; i < CSD_PARAM_COUNT; i++) {
                const uint8_t param_id = (uint8_t)(CSD_PARAM_ID_MIN + i);
                // ★0 不是合法工作值的项一律跳过★: 分辨率/分频/模态IDAC/IDAC增益 为 0 表示
                // "store 里没有有效值"(旧 blob、被污染、或从未校准过), 硬把 0 下发会直接毁掉
                // PSoC 自动校准算出的结果 —— 实测 IDAC=0 下发后全通道 raw 卡满量程 4095 不可用。
                // 阈值类为 0 仍照发: 0 阈值虽不推荐但语义明确(用户可能真想关掉某项判定)。
                if (_param[ch][i] == 0u && _param_zero_invalid(param_id)) {
                    continue;
                }
                psoc->set_param(ch, param_id, _param[ch][i]);
            }
        }
        need_apply = true;
    }

    // 全局配置(改 RAM 影子)与硬件参数均需 APPLY 重初始化生效。
    if (need_apply) {
        psoc->apply_params();
    }
}

bool CsdConfig::sampling_trustworthy(Psoc* psoc) {
    if (psoc == nullptr || !psoc->link_ok()) return false;
    // 抽样通道覆盖 Cp 两极(实测 ch8=22pF 最低, ch35=138pF 最高)与中段, 6 个通道足以暴露 railed/停滞,
    // 又不会在 core0 阻塞太久(每条 get_raw 约 ms 级)。
    static const uint8_t k_probe_ch[] = { 0u, 7u, 8u, 17u, 34u, 35u };
    constexpr uint8_t PROBE_COUNT = (uint8_t)(sizeof(k_probe_ch) / sizeof(k_probe_ch[0]));
    constexpr uint16_t RAW_RAILED = 4090u;   // 12 位满量程 4095: 校准发散/过充的典型表现
    uint16_t first[PROBE_COUNT] = {0};
    uint8_t moved = 0;

    for (uint8_t i = 0; i < PROBE_COUNT; i++) {
        if (!psoc->get_raw(k_probe_ch[i], &first[i])) return false;   // 读不到 = 不可信
        if (first[i] >= RAW_RAILED) return false;                     // railed = 不可信
    }
    // 第二遍读取与第一遍天然相隔数 ms(> 一个扫描周期), 正常采样必有 LSB 级抖动。
    for (uint8_t i = 0; i < PROBE_COUNT; i++) {
        uint16_t again = 0;
        if (!psoc->get_raw(k_probe_ch[i], &again)) return false;
        if (again >= RAW_RAILED) return false;
        if (again != first[i]) moved++;
    }
    return moved > 0;   // 全部抽样通道两次完全一致 = 扫描停滞
}

bool CsdConfig::capture_from_psoc(Psoc* psoc) {
    // ★这里不再按模式设卡★: 用户显式下发 CSD_CAPTURE 的语义就是"把设备当前这套值收作我的手动基线",
    // 在 AUTO 下捕获正是建立半自动基线的正常做法(让 CapSense 先算好, 再收成种子)。
    // 需要防的是**自动路径**的静默固化(掉线恢复/恢复默认后自行回读), 那个卡在调用方 main.cpp 的
    // recapture 分支里 —— 静默固化会把用户手动阈值/snsClk 无声覆盖, 而显式捕获是用户自己要的。
    if (psoc == nullptr || !psoc->link_ok()) return false;
    // ★不固化坏采样★: 显式捕获的语义是"把设备当前这套值收作我的基线", 前提是这套值【真的在工作】。
    // PSoC 处于 railed(raw 卡满量程 4095)/扫描停滞时, 读回来的是一套自毁配置, 一旦固化就会在每次
    // 开机 provision 时被重新下发, 把面板永久钉死在不可用状态 —— 实测正是这条路把 snsClk=8 +
    // IDAC_MOD=127 反复写回 store(恢复默认清掉后, 一跑显式捕获又被写回来), 即"DIV 异常固化"。
    // 这不是按模式设卡(用户想在 AUTO 下收种子仍然允许), 而是数据有效性门禁: 明知是坏值就不该存。
    if (!sampling_trustworthy(psoc)) return false;
    for (uint8_t ch = 0; ch < CSD_CHANNELS; ch++) {
        for (uint8_t i = 0; i < CSD_PARAM_COUNT; i++) {
            uint32_t v = 0;
            if (psoc->get_param(ch, (uint8_t)(CSD_PARAM_ID_MIN + i), &v)) {
                _param[ch][i] = (uint16_t)v;
            }
        }
    }
    // 同时捕获全局 CSD 配置(未激活传感器连接/IDAC/MFS)作为真相源种子。
    for (uint8_t i = 0; i < CSD_GLOBAL_COUNT; i++) {
        uint32_t v = 0;
        if (psoc->get_global((uint8_t)(CSD_GLOBAL_ID_MIN + i), &v)) {
            _global[i] = (uint16_t)v;
        }
    }
    _valid = true;
    _global_valid = true;
    _sync_storage();
    return true;
}
