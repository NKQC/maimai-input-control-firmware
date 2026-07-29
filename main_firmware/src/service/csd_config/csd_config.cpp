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
#endif

namespace {
constexpr uint32_t CSD_BLOB_MAGIC = 0x31445343u;  // "CSD1"
constexpr const char* CSD_BLOB_PATH = "/csd.bin";

#pragma pack(push, 1)
struct CsdBlob {
    uint32_t magic;
    uint8_t  mode;
    uint8_t  valid;
    uint8_t  global_valid;
    uint8_t  reserved;
    uint16_t param[CSD_CHANNELS][CSD_PARAM_COUNT];
    uint16_t global[CSD_GLOBAL_COUNT];
    uint32_t crc32;   // 覆盖前面全部字节
};
#pragma pack(pop)
}  // namespace

CsdConfig* CsdConfig::_instance = nullptr;

CsdConfig::CsdConfig() : _mode(CSD_MODE_SEMI), _valid(false) {
    std::memset(_param, 0, sizeof(_param));
    std::memset(_global, 0, sizeof(_global));
}

void CsdConfig::note_global(uint8_t gparam_id, uint32_t value) {
    if (!_global_id_ok(gparam_id)) return;
    _global[gparam_id - CSD_GLOBAL_ID_MIN] = (uint16_t)value;
    _global_valid = true;
}

CsdConfig* CsdConfig::getInstance() {
    if (_instance == nullptr) _instance = new CsdConfig();
    return _instance;
}

void CsdConfig::note_param(uint8_t ch, uint8_t param_id, uint32_t value) {
    if (ch >= CSD_CHANNELS || !_param_id_ok(param_id)) return;
    _param[ch][_param_index(param_id)] = (uint16_t)value;
    // 置有效: 否则 SAVE_CONFIG 落盘的 blob 里 _valid=false, 重启 download_to_psoc 会跳过逐通道
    // 参数下发 → 分辨率/snsClk/idac 等每通道参数保存后重启仍丢失(半自动模式下不恢复)。
    _valid = true;
}

void CsdConfig::clear() {
    std::memset(_param, 0, sizeof(_param));
    std::memset(_global, 0, sizeof(_global));
    _valid = false;
    _global_valid = false;
    _mode = CSD_MODE_AUTO;   // 回到自动校准, PSoC 用生成默认跑标准处理链
    request_save();
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
    _global_valid = (blob.global_valid != 0u);
    std::memcpy(_param, blob.param, sizeof(_param));
    std::memcpy(_global, blob.global, sizeof(_global));
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
    blob.global_valid = _global_valid ? 1u : 0u;
    std::memcpy(blob.param, _param, sizeof(_param));
    std::memcpy(blob.global, _global, sizeof(_global));
    blob.crc32 = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));

    // LittleFS 擦写期间 XIP 不可执行，当前 API 无安全分片点；忙标记先阻止推送，再在临界区两端泵 USB，
    // 避免 telemetry/进度帧把共享的 64B vendor IN FIFO 塞满。阈值效果由 host soak 的调试计数器实测。
    size_t n = 0;
    {
        FlashWriteGuard flash_guard;
        HAL_USB_Device::getInstance()->task();
        uint32_t _irq = save_and_disable_interrupts();
        multicore_lockout_start_blocking();
        File f = LittleFS.open(CSD_BLOB_PATH, "w");
        if (f) {
            n = f.write((const uint8_t*)&blob, sizeof(blob));
            f.close();
        }
        multicore_lockout_end_blocking();
        restore_interrupts(_irq);
    }
    // 忙标记已撤销后立即 pump，先恢复 vendor 端点再允许下一轮调度器产生推送。
    HAL_USB_Device::getInstance()->task();
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
    return true;
}
