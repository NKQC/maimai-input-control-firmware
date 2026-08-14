#include "csd_config.h"
#include "../../protocol/psoc/psoc.h"
#include "../config_manager/config_crc.h"
#include "../../flash_guard.h"
#include "../../hal/usb/hal_usb.h"
#include <cstring>
#include "../usb_debug.h"
#include "../self_heal/self_heal.h"   // tick_trust_recheck 撤销建议时记 SH_BASELINE_TRUST_RESTORED

#ifdef PICO_PLATFORM
#include "LittleFS.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "../nv_store/nv_store.h"   // 自管理 flash blob(替代 LittleFS 文件)
#endif

namespace {
constexpr uint32_t CSD_BLOB_MAGIC = 0x31445343u;  // "CSD1"
constexpr const char* CSD_BLOB_PATH = "/csd.bin";
constexpr uint8_t RECAPTURE_PROBE_CH[] = {0u, 7u, 8u, 17u, 34u, 35u};
constexpr uint8_t RECAPTURE_PROBE_COUNT =
    static_cast<uint8_t>(sizeof(RECAPTURE_PROBE_CH) / sizeof(RECAPTURE_PROBE_CH[0]));
constexpr uint16_t RECAPTURE_RAW_RAILED = 4090u;

}  // namespace

CsdConfig* CsdConfig::_instance = nullptr;

CsdConfig::CsdConfig() : _mode(CSD_MODE_SEMI), _valid(false) {
    _reset_params();
    std::memset(_global, 0, sizeof(_global));
}

// 参数表复位到"无值"。★enabled 列例外, 必须复位成 1(启用)★
// 其余参数取 0 的含义是"store 里没有有效值"(下发时按 _param_zero_invalid 跳过), 而 enabled 的 0
// 是一个**有效且危险**的取值(= 关掉这个通道的硬件)。若跟着一起清零, 任何一次"旧 blob 尺寸失配 /
// 恢复默认"都会把 36 个通道全部关掉 —— 面板整体失灵且用户完全不知道发生了什么。
void CsdConfig::_reset_params() {
    std::memset(_param, 0, sizeof(_param));
    for (uint8_t ch = 0; ch < CSD_CHANNELS; ch++) {
        _param[ch][_param_index(CSD_PARAM_ENABLED)] = 1u;
    }
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
    // "恢复默认"的语义是回到出厂可用状态 ⇒ 36 通道全部启用(见 _reset_params)。
    _reset_params();
    std::memset(_global, 0, sizeof(_global));
    _valid = false;
    _global_valid = false;
    _mode = CSD_MODE_AUTO;
    _recapture_stage = RecaptureStage::IDLE;
    _recapture_index = 0u;
    _recapture_moved = false;
    _sync_storage();
    request_save();
}

void CsdConfig::request_recapture() {
    _recapture_pending = true;
    _baseline_untrusted = false;
    _recapture_stage = RecaptureStage::SAMPLE_FIRST;
    _recapture_index = 0u;
    _recapture_moved = false;
    _recapture_has_enabled = false;
    _recapture_started_ms = 0u;
    std::memset(_recapture_first, 0, sizeof(_recapture_first));
}

void CsdConfig::clear_recapture() {
    _recapture_pending = false;
    _recapture_stage = RecaptureStage::IDLE;
    _recapture_index = 0u;
    _recapture_moved = false;
    _recapture_has_enabled = false;
    _recapture_started_ms = 0u;
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
    // ★校验不过就保持构造函数给的默认(36 通道全启用)★
    // Mirror 里新增了 enabled 列 ⇒ 布局变长 ⇒ 旧固件写下的 blob 必然在这里被长度判定挡掉。
    // 那正是需要的行为: 旧 blob 里没有 enabled 这一列, 强行按新布局解读会把随机字节当成启用位。
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

bool CsdConfig::download_to_psoc(Psoc* psoc) {
    if (psoc == nullptr || !psoc->link_alive() || provisioning_active()) return false;
    _provision_stage = ProvisionStage::MODE;
    _provision_ch = 0u;
    _provision_index = 0u;
    _provision_need_global_commit = _global_valid;
    return true;
}

void CsdConfig::abort_provisioning() {
    _provision_stage = ProvisionStage::IDLE;
    _provision_ch = 0u;
    _provision_index = 0u;
    _provision_need_global_commit = false;
}

bool CsdConfig::provisioning_active() const {
    return _provision_stage != ProvisionStage::IDLE && _provision_stage != ProvisionStage::DONE;
}

bool CsdConfig::provisioning_complete() const {
    return _provision_stage == ProvisionStage::DONE;
}

void CsdConfig::tick_provisioning(Psoc* psoc) {
    if (!provisioning_active() || psoc == nullptr || !psoc->core1_idle() || psoc->heavy_busy()) return;

    switch (_provision_stage) {
        case ProvisionStage::MODE:
            // Restore shadows while the PSoC startup gate still holds all widgets
            // disabled.  Hardware fields must not be written into a live scan;
            // the single release APPLY below is the barrier that makes the whole
            // prepared image active.
            if (psoc->set_mode(_mode)) {
                _provision_ch = 0u;
                _provision_index = 0u;
                _provision_stage = _global_valid ? ProvisionStage::GLOBALS : ProvisionStage::ENABLED;
            }
            return;

        case ProvisionStage::GLOBALS:
            if (_provision_index >= CSD_GLOBAL_COUNT) {
                _provision_ch = 0u;
                _provision_index = 0u;
                // ★全局项设完必须 COMMIT★ 否则 inactive_sns/IDAC/MFS 只落在 PSoC 的 RAM 影子里,
                // 要等下一次无关的重初始化才生效 —— store 与设备实际行为不一致的又一条暗路。
                _provision_stage = _provision_need_global_commit
                    ? ProvisionStage::GLOBAL_COMMIT : ProvisionStage::ENABLED;
                return;
            }
            if (psoc->set_global(static_cast<uint8_t>(CSD_GLOBAL_ID_MIN + _provision_index),
                                 _global[_provision_index])) {
                _provision_index++;
            }
            return;

        case ProvisionStage::GLOBAL_COMMIT:
            // COMMIT 内含 Cy_CapSense_Init(会把全部 widget 的 ENABLE 位重新置起), 故必须排在
            // ENABLED 之前; PSoC 侧 Init 之后自己会重放位图, 两侧都不依赖对方的时序巧合。
            if (psoc->global_commit()) {
                _provision_need_global_commit = false;
                _provision_stage = ProvisionStage::ENABLED;
            }
            return;

        case ProvisionStage::ENABLED:
            // ★启用位图必须无条件下发, 且必须排在 PARAMS 之前★
            // 1) 无条件: 它是硬件开关而非调参项, AUTO 模式与 !_valid 的空 store 同样有明确的启用集
            //    (_reset_params 把该列复位成"全启用")。历史实现里 PARAMS 分支直接跳到 DONE, 于是
            //    正常的 SEMI+valid 路径**永远不下发这 36 条**, PSoC 的启动 gate 只能 3s 超时回退成
            //    "全 36 通道启用" —— 每次 PSoC 复位后用户关掉的通道都会偷偷复活并继续扫描, 而
            //    UI 读的是 PSoC(说启用)、校准门禁读的是 store(说禁用), 双真相源就是从这里长出来的。
            // 2) 先于 PARAMS: PSoC 的 gate 只按"位图静默"计时, 36 条能在窗口内送完; 若排在 ~396 条
            //    PARAMS 之后, 窗口早已耗尽。
            if (_provision_ch >= CSD_CHANNELS) {
                _provision_ch = 0u;
                _provision_index = 0u;
                _provision_stage = (_mode == CSD_MODE_SEMI && _valid)
                    ? ProvisionStage::PARAMS : ProvisionStage::RELEASE_APPLY;
                return;
            }
            if (psoc->set_param(_provision_ch, CSD_PARAM_ENABLED,
                                _param[_provision_ch][_param_index(CSD_PARAM_ENABLED)] != 0u ? 1u : 0u)) {
                _provision_ch++;
            }
            return;

        case ProvisionStage::RELEASE_APPLY:
            // This single APPLY is the PSoC's documented startup release barrier,
            // not a shadow replay commit.  It is queued asynchronously and the
            // next stage observes busy until the PSoC has really resumed scanning.
            if (psoc->apply_params()) _provision_stage = ProvisionStage::WAIT_RELEASE_APPLY;
            return;

        case ProvisionStage::WAIT_RELEASE_APPLY:
            if (psoc->heavy_busy()) return;
            _provision_stage = ProvisionStage::DONE;
            return;

        case ProvisionStage::PARAMS:
            while (_provision_ch < CSD_CHANNELS) {
                while (_provision_index < CSD_PARAM_COUNT) {
                    const uint8_t param_id = static_cast<uint8_t>(CSD_PARAM_ID_MIN + _provision_index);
                    const uint16_t value = _param[_provision_ch][_param_index(param_id)];
                    if (param_id == CSD_PARAM_ENABLED || (value == 0u && _param_zero_invalid(param_id))) {
                        _provision_index++;
                        continue;
                    }
                    if (!psoc->set_param(_provision_ch, param_id, value)) return;
                    _provision_index++;
                    return;
                }
                _provision_ch++;
                _provision_index = 0u;
            }
            // 参数写完必须走放行屏障, 不能直接 DONE: RELEASE_APPLY 才是"这套镜像整体生效"的那一步
            // (PSoC 侧据它清 g_provision_pending 并开始扫描)。
            _provision_stage = ProvisionStage::RELEASE_APPLY;
            return;

        case ProvisionStage::IDLE:
        case ProvisionStage::DONE:
        default:
            return;
    }
}

int8_t CsdConfig::tick_recapture(Psoc* psoc) {
    if (!_recapture_pending) return -1;
    if (psoc == nullptr || !psoc->link_ok()) return 0;
    if (!psoc->core1_idle() || psoc->heavy_busy()) return 0;

    switch (_recapture_stage) {
        case RecaptureStage::SAMPLE_FIRST:
            while (_recapture_index < RECAPTURE_PROBE_COUNT &&
                   !ch_enabled(RECAPTURE_PROBE_CH[_recapture_index])) {
                _recapture_index++;
            }
            if (_recapture_index >= RECAPTURE_PROBE_COUNT) {
                _recapture_index = 0u;
                _recapture_started_ms = millis();
                _recapture_stage = RecaptureStage::SAMPLE_SECOND;
                return 0;
            }
            _recapture_has_enabled = true;
            // ★采样质量只作建议, 不再当门禁★ 读不到/railed 以前会 return -1, 而调用方对 -1 的处理是
            // clear() 把整个 store 连同用户全部调参与启用集一起清空 —— 高阶用户与预配置面板正常会
            // 出现"电极空闲 raw 不抖""个别通道读数偏高"这类形态, 却被当成故障把配置抹掉, 代价远大于
            // 它想防的问题。现在一律记标志继续走完捕获, 由上位机把它显示成建议。
            if (!psoc->get_raw(RECAPTURE_PROBE_CH[_recapture_index],
                               &_recapture_first[_recapture_index]) ||
                _recapture_first[_recapture_index] >= RECAPTURE_RAW_RAILED) {
                _baseline_untrusted = true;
                _recapture_first[_recapture_index] = 0u;
            }
            _recapture_index++;
            return 0;

        case RecaptureStage::SAMPLE_SECOND: {
            if (static_cast<uint32_t>(millis() - _recapture_started_ms) < 10u) return 0;
            while (_recapture_index < RECAPTURE_PROBE_COUNT &&
                   !ch_enabled(RECAPTURE_PROBE_CH[_recapture_index])) {
                _recapture_index++;
            }
            if (_recapture_index >= RECAPTURE_PROBE_COUNT) {
                // 两次读数完全不抖 ⇒ 只记建议标志, 照旧固化。原先这里 return -1 会让调用方清空 store。
                if (_recapture_has_enabled && !_recapture_moved) _baseline_untrusted = true;
                _recapture_index = 0u;
                _recapture_stage = RecaptureStage::PARAMS;
                return 0;
            }
            uint16_t value = 0u;
            if (!psoc->get_raw(RECAPTURE_PROBE_CH[_recapture_index], &value) ||
                value >= RECAPTURE_RAW_RAILED) {
                _baseline_untrusted = true;
                _recapture_index++;
                return 0;
            }
            if (value != _recapture_first[_recapture_index]) _recapture_moved = true;
            _recapture_index++;
            return 0;
        }

        case RecaptureStage::PARAMS: {
            constexpr uint16_t PARAM_TOTAL = CSD_CHANNELS * CSD_PARAM_COUNT;
            if (_recapture_index >= PARAM_TOTAL) {
                _recapture_index = 0u;
                _recapture_stage = RecaptureStage::GLOBALS;
                return 0;
            }
            const uint8_t ch = static_cast<uint8_t>(_recapture_index / CSD_PARAM_COUNT);
            const uint8_t index = static_cast<uint8_t>(_recapture_index % CSD_PARAM_COUNT);
            // ★启用列绝不回读★ 见 capture_from_psoc 同处说明: 它是 store 单向下发的硬件开关,
            // 从 PSoC 读回就等于让执行者反过来定义真相。
            if (index == _param_index(CSD_PARAM_ENABLED)) {
                _recapture_index++;
                return 0;
            }
            uint32_t value = 0u;
            if (!psoc->get_param(ch, static_cast<uint8_t>(CSD_PARAM_ID_MIN + index), &value)) {
                return -1;
            }
            _param[ch][index] = static_cast<uint16_t>(value);
            _recapture_index++;
            return 0;
        }

        case RecaptureStage::GLOBALS: {
            if (_recapture_index >= CSD_GLOBAL_COUNT) {
                _valid = true;
                _global_valid = true;
                _mode = CSD_MODE_SEMI;
                _sync_storage();
                clear_recapture();
                return 1;
            }
            uint32_t value = 0u;
            if (!psoc->get_global(static_cast<uint8_t>(CSD_GLOBAL_ID_MIN + _recapture_index),
                                  &value)) {
                return -1;
            }
            _global[_recapture_index] = static_cast<uint16_t>(value);
            _recapture_index++;
            return 0;
        }

        case RecaptureStage::IDLE:
        default:
            return -1;
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
    uint8_t probed = 0;
    uint8_t moved = 0;

    // ★禁用通道必须排除在抽检之外★ 它已退出扫描, PSoC 对它的 raw 恒回 0(见 cmd_get_raw):
    // 既不 railed 也永远不抖动 —— 照旧参与判定就会被当成"扫描停滞", 于是"关掉几个通道"这个正常
    // 操作会让恢复默认路径拒绝固化基线, 甚至把整个 store 清空。
    for (uint8_t i = 0; i < PROBE_COUNT; i++) {
        if (!ch_enabled(k_probe_ch[i])) continue;
        if (!psoc->get_raw(k_probe_ch[i], &first[i])) return false;   // 读不到 = 不可信
        if (first[i] >= RAW_RAILED) return false;                     // railed = 不可信
        probed++;
    }
    // 抽样集合被用户全部关掉时无从判定 ⇒ 按"没有异常证据"处理。
    // 报"不可信"会触发 clear() 把用户全部调参连同启用集一起清掉, 那比不判定坏得多。
    if (probed == 0u) return true;
    // 第二遍读取与第一遍天然相隔数 ms(> 一个扫描周期), 正常采样必有 LSB 级抖动。
    for (uint8_t i = 0; i < PROBE_COUNT; i++) {
        uint16_t again = 0;
        if (!ch_enabled(k_probe_ch[i])) continue;
        if (!psoc->get_raw(k_probe_ch[i], &again)) return false;
        if (again >= RAW_RAILED) return false;
        if (again != first[i]) moved++;
    }
    return moved > 0;   // 全部抽样通道两次完全一致 = 扫描停滞
}

// 采样质量建议的自愈重评估。见 csd_config.h 处说明。
// 节流 10s: sampling_trustworthy() 会做若干次同步 get_raw(读类阻塞), 不能每轮都付这个成本;
// 而它只在标志已置位时才被调用, 所以健康设备上这个函数是一条比较后立即返回的空路径。
void CsdConfig::tick_trust_recheck(Psoc* psoc) {
    if (!_baseline_untrusted) return;                 // 常态: 零 SPI 事务
    if (psoc == nullptr || !psoc->link_alive()) return;
    // 重操作在途时抽检读到的 raw 不代表稳态(校准/基线复位期间 PSoC 主循环不在正常扫描)。
    if (psoc->heavy_busy()) return;
    const uint32_t now_ms = millis();
    if (_trust_recheck_ms != 0u && (uint32_t)(now_ms - _trust_recheck_ms) < 10000u) return;
    _trust_recheck_ms = now_ms;
    if (!sampling_trustworthy(psoc)) return;          // 仍然存疑: 保留建议, 下个窗口再看
    _baseline_untrusted = false;
    SelfHeal::getInstance()->note(SH_BASELINE_TRUST_RESTORED, 0u);
}

bool CsdConfig::capture_from_psoc(Psoc* psoc) {
    // ★这里不再按模式设卡★: 用户显式下发 CSD_CAPTURE 的语义就是"把设备当前这套值收作我的手动基线",
    // 在 AUTO 下捕获正是建立半自动基线的正常做法(让 CapSense 先算好, 再收成种子)。
    // 需要防的是**自动路径**的静默固化(掉线恢复/恢复默认后自行回读), 那个卡在调用方 main.cpp 的
    // recapture 分支里 —— 静默固化会把用户手动阈值/snsClk 无声覆盖, 而显式捕获是用户自己要的。
    if (psoc == nullptr || !psoc->link_ok()) return false;
    // ★采样质量只作建议, 不再拦截显式捕获★ 这里原先 `return false` 直接否掉用户下发的 CSD_CAPTURE。
    // 但"raw 不抖/偏高"在高阶用户与预配置面板上是正常形态(电极空闲、手调过 IDAC/snsClk), 拿它当
    // 门禁就把"我要把当前这套值收作基线"这个明确意图给驳回了, 而用户往往看不出被驳回的原因。
    // 现在照旧捕获, 只把结论记成标志经 DEVICE_INFO 上报, 由上位机显示成建议。
    // 直接赋值而不是只置位: 这次捕获的实测结论比之前任何一次都新, 采样已恢复正常时必须把建议撤掉。
    _baseline_untrusted = !sampling_trustworthy(psoc);
    for (uint8_t ch = 0; ch < CSD_CHANNELS; ch++) {
        for (uint8_t i = 0; i < CSD_PARAM_COUNT; i++) {
            // ★启用列(0x0C)绝不从 PSoC 回读★
            // 其余参数问 PSoC 是对的: 校准/频率自适应会**合法地**改 idac/snsClk, 设备值才是最新。
            // 而启用与否只由用户经 store 决定, PSoC 只是执行者 —— 一旦把它读回来, PSoC 任何一次
            // 位图漂移(启动 gate 超时回退、丢一条 SET_PARAM)都会被固化成"用户的配置", 真相源反转。
            if (i == _param_index(CSD_PARAM_ENABLED)) continue;
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
