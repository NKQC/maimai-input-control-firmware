#include "sensor_link.h"
#include "../../hal/usb/hal_usb.h"
#include "../../protocol/psoc/psoc.h"
#include "../csd_config/csd_config.h"
#include "../psoc_algo/psoc_algo.h"
#include "../psoc_algo/psoc_algo_default.h"
#include "../psoc_updater/psoc_updater.h"
#include "../tx_scheduler/tx_scheduler.h"
#include "../latency_stats.h"
#include <pico/stdlib.h>
#include <hardware/watchdog.h>
#include <cstdio>
#include <cstring>

// 遥测租约(ms): 上位机需在此时限内经任意命令帧续期(UsbComm renew_all), 否则任务自动停。
static constexpr uint32_t TELEM_LEASE_MS = 3000;
// 频率自适应进度推送: 5Hz 足够看清阶段变化又不占带宽; 租约须覆盖最坏总耗时+余量,
// 因为长自适应期间上位机可能一条命令都不发(无 renew_all), 靠初始租约撑到完成。
static constexpr uint32_t AUTOTUNE_INTERVAL_US = 200000;
static constexpr uint32_t AUTOTUNE_LEASE_MS = 60000;
// 自续租上限(帧数): 长自适应期间上位机可能一条命令都不发, 而任意主机帧都会 renew_all(3s) 覆盖本任务
// 租约 → 靠本任务自续租撑过全程; 上限须【大于】RP2040 的自适应窗口(45s), 否则全通道逐通道自适应
// (36 × 单通道 ≈ 11-23s, 最坏更长)会在终态帧发出前先把推送任务饿死 → 上位机只能等超时。
// 350 帧 × 200ms = 70s > 45s, 异常情况下推送仍必然自灭。
static constexpr uint16_t AUTOTUNE_MAX_TICKS = 350;
// PSoC 救砖进度推送: 全片擦写+校验+重新下发数秒~十几秒, 同自适应用 5Hz + 长租约 + 帧数硬上限自灭。
static constexpr uint32_t RESCUE_INTERVAL_US = 200000;
static constexpr uint32_t RESCUE_LEASE_MS = 30000;
static constexpr uint16_t RESCUE_MAX_TICKS = 300;   // 60s > 最坏总耗时, 异常时推送必然自灭
static constexpr uint32_t FOCUS_SCAN_RENEW_INTERVAL_US = 1000000;

// ---------------- SweepSession(0x36-0x38) ----------------
// 步进周期 2ms: 一格 = runtime apply + 回读 + settle/sample 快照；恢复只写回原 gain/div，
// 全程非阻塞单步推进，每周期最多做一件事，绝不在 handler 或 tick 里等设备。
static constexpr uint32_t SWEEP_INTERVAL_US = 2000;
// 扫描期租约: 主机 keepalive 正常时持续有效；USB 背压期间也不能让结果发送窗口在 8s 内过期。
// 扫描状态机在背压时暂停推进，恢复链仍使用无租约任务。
static constexpr uint32_t SWEEP_LEASE_MS = 30000;
// 阶段安全兜底(周期数 × 2ms): 所有正常推进均以 PSoC 真实完成、队列排空或快照代次推进为准；
// 仅在连续 120s 未出现可采信进展时放弃当前格/恢复步骤，避免短暂繁忙被误判为失败。
static constexpr uint32_t SWEEP_WAIT_TICKS = 60000u;
static constexpr uint32_t SWEEP_RUNTIME_APPLY_TICKS = 60000u;
static constexpr uint16_t SWEEP_TERMINAL_TICKS = 15000;
static constexpr uint8_t  SWEEP_FRAMES_PER_TICK = 2;    // 结果/补发帧的发送预算(背压即停, 下拍续发)
static constexpr uint8_t  SWEEP_SETTLE_MAX = 64;
static constexpr uint8_t  SWEEP_SAMPLE_MAX = 64;
// ★12 位口径★ CapSense raw 是 12 位(满量程 4095), 与 csd_config 的 railed 判据同源。
// 原先写成 0xFFF0 对 12 位样本永远不可达 ⇒ RAILED 永不触发, 饱和格被错分到别的失败位上。
static constexpr uint16_t SWEEP_RAILED_RAW = 4090u;     // 接近满量程 ⇒ IDAC 补偿不足, 该格无效
static constexpr uint16_t SWEEP_INDEX_NONE = 0xFFFFu;   // 非结果帧(RESTORING/终态)的格号占位
static constexpr uint8_t  SWEEP_FRAME_LEN = 23;
// ★单格失败不再毙掉整个会话，但连续失败必须收敛★
// 一格的 runtime apply / 回读失败只说明"这个 gain/div 组合 PSoC 吃不下"(极端组合下本就会发生),
// 下一格会重新写入并回读, 状态每格都自行重建, 所以记成空洞继续扫是安全且必要的 ——
// 原先第一格失败就 _begin_sweep_restore(FAILED), 于是整张表在设备健康时也只能拿到零星几格。
// 但若 PSoC 真的卡死, 连续失败会一直空转到 448 格; 连续这么多格都失败即判为真故障, 立即回滚。
static constexpr uint16_t SWEEP_ABORT_FAIL_RUN = 12;
// 结果位: 低 4 位属该格采样, 高 4 位属恢复阶段, 复用同一 flags 字节回显给上位机。
static constexpr uint8_t  SWEEP_FLAG_RAILED = 0x01;
// 扫描停滞 = 该格在阶段兜底期内始终等不到新的快照代次(PSoC 没有产出新的扫描结果)。
// ★不能用"样本不抖动"当停滞判据★: 本扫描的目的就是找噪声最低的 gain/div, pp=0 是最好的结果,
// 拿它判无效会让整张表在设备完全健康时全军覆没(实测 448 格 valid=0)。
static constexpr uint8_t  SWEEP_FLAG_STALLED = 0x02;
static constexpr uint8_t  SWEEP_FLAG_CAL_FAIL = 0x04;
static constexpr uint8_t  SWEEP_FLAG_MISMATCH = 0x08;     // 回读值 != 期望值(PSoC 侧钳位/拒绝)
static constexpr uint8_t  SWEEP_RESTORE_FLAG_PARAM = 0x10;
static constexpr uint8_t  SWEEP_FLAG_RETRANSMIT = 0x80;   // 该帧为 SWEEP_CTRL 补发
static constexpr uint8_t  SWEEP_PARAM_ID_DIV = 0x08u;     // SNS_CLK_DIV
static constexpr uint8_t  SWEEP_PARAM_ID_GAIN = 0x0Bu;    // IDAC_GAIN(0..6)

volatile uint16_t g_lat_spi_us = 0;
volatile uint16_t g_lat_proc_us = 0;
volatile uint16_t g_lat_usb_us = 0;
volatile int16_t  g_delay_dev_min_us = 0;
volatile int16_t  g_delay_dev_max_us = 0;
volatile uint8_t  g_delay_dev_flags = 0;

SensorLink* SensorLink::_instance = nullptr;

namespace {
// Phase A 可运行时读写的 CSD 参数 id（与 PSoC cmd_get/set_param 及上位机 proto 对齐）
constexpr uint8_t kParamIds[] = {
    0x01,  // FINGER_TH
    0x02,  // NOISE_TH
    0x03,  // NEG_NOISE_TH
    0x04,  // HYSTERESIS
    0x05,  // ON_DEBOUNCE
    0x06,  // LOW_BSLN_RST
    0x07,  // RESOLUTION
    0x08,  // SNS_CLK_DIV
    0x09,  // IDAC_MOD
    0x0A,  // SNS_CLK_SOURCE
    0x0B,  // IDAC_GAIN
    0x0C,  // ENABLED(通道启用开关: 0=禁用/电极高阻 1=启用)
};
constexpr uint8_t kParamCount = sizeof(kParamIds) / sizeof(kParamIds[0]);
// Cp 哨兵(与 PSoC/上位机一致): 测量失败 / 读取失败都用 0xFFFFFF —— 它就是 PSoC 在 BIST 失败
// (短接/电容过大)时留下的值。
constexpr uint32_t CP_UNMEASURED_FF = 0x00FFFFFFu;
// ★禁用通道 = 未测量, 必须与"测量失败"分开★ PSoC 每次 MEASURE_CP 先把 36 个 cp_value 统一预置成
// 0xFFFFFF, 再逐通道测量并**跳过禁用通道**(main.c 的 `if (!_ch_is_enabled(w)) continue;`), 于是
// 禁用通道恒停在与真失败完全相同的哨兵上。上位机据此把"这轮没测它"报成"测量失败: 可能短接或电容
// 过大", 每次测电容后当次禁用的那组通道必然被点名(实测两次分别是 CH0/3/17/35 与 CH8/25/26)。
// 0xFFFFFE 不能用(PSoC 拿它表示"超上限已钳位"的真实读数), 故取 0xFFFFFD。
constexpr uint32_t CP_DISABLED_FF = 0x00FFFFFDu;
// PARAM_GET_ALL 的"全通道单参数"变体标记(payload = 0xFF + param_id)。
constexpr uint8_t kAllChannels = 0xFFu;

// ★长周期 PSoC 指令的反堆叠闸门★
// 已有一条在途时新请求一律回 DEVICE_BUSY, **绝不排队**。排队会让若干条秒级操作背靠背堆在 core1 上,
// 而 core1 忙 = core0 的落盘窗口关闭 + 命令环满时 core0 卡在入队自旋, 主循环被整体拖长 ——
// 用户实测"一次保存后紧接一次 CH8 自适应就掉线"正落在这条上。
// DEVICE_BUSY 是可重试语义(上位机既有重试/冷却路径认它), 比排队后延迟数十秒才生效诚实得多。
// 返回 true = 已写好 NAK, 调用方直接 return。
inline bool heavy_gate_reject(const char* what, const HostFrame& frame,
                              uint8_t* response, uint16_t* response_length) {
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->heavy_busy()) return false;
    psoc->note_heavy_reject();
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
                                                what, response, HOST_CMD_RESP_BUF_MAX);
    return true;
}

// Synchronous SPI reads must never wait behind startup provisioning or a long
// PSoC operation. A bounded DEVICE_BUSY terminal leaves UsbComm free to serve
// telemetry and lets the host retry the same request after the current step.
inline bool sync_read_gate_reject(const char* what, const HostFrame& frame,
                                  uint8_t* response, uint16_t* response_length) {
    Psoc* psoc = Psoc::getInstance();
    if (psoc->core1_idle() && !psoc->heavy_busy()) return false;
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
                                                what, response, HOST_CMD_RESP_BUF_MAX);
    return true;
}

// ch_mask(u64 LE) → PSoC 的通道字节: 恰好一位 ⇒ 该通道号(0..35); 其余(空/多位/全 36 位) ⇒ 0xFF 全通道。
// ★为什么"恰好一位"才算单通道★ 上位机的单通道入口本来就编码成 `1<<ch`, 而批量入口是 host 侧
// 逐通道串行队列(每条仍是 `1<<ch`)。真正需要 0xFF 的只有"全 36 位"这种兼容/兜底调用。
// 多位但不满 36 的掩码没有对应的固件语义(帧里只有一个通道字节), 退化为全通道比只做第一位诚实。
inline uint8_t _mask_to_single_ch(const HostFrame& frame) {
    if (frame.len < 8) return 0xFFu;
    uint64_t mask = 0;
    for (uint8_t i = 0; i < 8; i++) mask |= (uint64_t)frame.payload[i] << (8u * i);
    mask &= 0xFFFFFFFFFULL;   // 低 36 位有效
    if (mask == 0u || (mask & (mask - 1u)) != 0u) return 0xFFu;   // 空 / 多位 → 全通道
    uint8_t ch = 0;
    while ((mask >> ch) != 1u) ch++;
    return ch;
}

// 单通道重操作落在【已禁用】通道上 ⇒ 明确 NAK, 而不是让它静默空转。
// ★为什么要在 RP 这一层拦★ PSoC 侧本来就会跳过禁用通道(它的电极必须保持高阻, 不能为了校准去连),
// 但那是"什么都没发生"—— 上位机收到 ACK 却看不到任何变化, 只能当成"设备坏了"。在这里如实回绝,
// 上位机的既有 NAK 日志路径就会把原因写清楚。0xFF(全通道)不拦: 固件会逐通道跳过禁用项。
inline bool disabled_ch_reject(const char* what, uint8_t ch, const HostFrame& frame,
                               uint8_t* response, uint16_t* response_length) {
    if (ch >= SENSOR_LINK_CHANNELS) return false;
    if (CsdConfig::getInstance()->ch_enabled(ch)) return false;
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                                                what, response, HOST_CMD_RESP_BUF_MAX);
    return true;
}

// ★参数回读的唯一入口: 0x0C(通道启用)只认 store, 其余问 PSoC★
// 分野的依据是"谁有权改这一项":
//   - snsClk/idacMod/idacGain/阈值: 校准与频率自适应会**合法地**改写它们, 设备上的值才是最新的
//     真相, 必须问 PSoC(否则手动校准的结果在 UI 上永远看不到)。
//   - 0x0C: 只有用户经 store 能改, PSoC 只是执行者。此前这一项也问 PSoC, 于是 UI 复选框显示的是
//     "设备当前恰好在扫哪些通道", 而校准/基线复位/频率自适应的门禁读的是 store —— 两者一旦漂移,
//     用户看到勾选却点不动那三个按钮, 且完全看不出原因。现在回读与门禁同源。
inline bool param_read(uint8_t ch, uint8_t param_id, uint32_t* value) {
    if (param_id == 0x0Cu) {
        if (ch >= SENSOR_LINK_CHANNELS) return false;
        *value = CsdConfig::getInstance()->ch_enabled(ch) ? 1u : 0u;
        return true;
    }
    return Psoc::getInstance()->get_param(ch, param_id, value);
}

constexpr uint8_t PARAM_READ_TRIES = 3u;

// param_read 的有界重试包装。失败几乎总是 core1 此刻正忙这种瞬时原因，而不是参数不存在；
// 静默跳过会把一次链路抖动伪装成参数消失，冷读又没有旧值可兜。次数必须有界：12 项无限重试
// 会卡住 USB 命令处理器。两次尝试之间沿用 Psoc::_submit() 的 USB 泵送、喂狗和 tight loop 让出，
// 再以仓内已有的 sleep_us(200) 给 core1 留出最小 SPI 空窗，不能忙等。
inline bool param_read_retry(uint8_t ch, uint8_t param_id, uint32_t* value) {
    for (uint8_t attempt = 0; attempt < PARAM_READ_TRIES; attempt++) {
        if (param_read(ch, param_id, value)) return true;
        if (attempt + 1u == PARAM_READ_TRIES) break;
        watchdog_update();
        HAL_USB_Device::getInstance()->task();
        tight_loop_contents();
        sleep_us(200);
    }
    return false;
}
}  // namespace

SensorLink::SensorLink()
    : _lease_ms(TELEM_LEASE_MS),
      _mode(0),
      _rate_hz(30),
      _fields(TELEM_FIELD_RAW | TELEM_FIELD_BASELINE | TELEM_FIELD_DIFF | TELEM_FIELD_STATUS),
      _ch_mask(0),
      _last_emit_us(0),
      _stream_seq(0),
      _at_req_ch(0xFF),
      _at_ticks(0) {
    std::memset(_tx_buf, 0, sizeof(_tx_buf));
}

SensorLink* SensorLink::getInstance() {
    if (_instance == nullptr) _instance = new SensorLink();
    return _instance;
}

void SensorLink::init() {
    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::TELEM_START, _handle_telem_start);
    dispatcher->register_handler(HostCmd::TELEM_STOP, _handle_telem_stop);
    dispatcher->register_handler(HostCmd::FOCUS_START, _handle_focus_start);
    dispatcher->register_handler(HostCmd::FOCUS_STOP, _handle_focus_stop);
    dispatcher->register_handler(HostCmd::SWEEP_START, _handle_sweep_start);
    dispatcher->register_handler(HostCmd::SWEEP_CTRL, _handle_sweep_ctrl);
    dispatcher->register_handler(HostCmd::PARAM_GET, _handle_param_get);
    dispatcher->register_handler(HostCmd::PARAM_SET, _handle_param_set);
    dispatcher->register_handler(HostCmd::PARAM_GET_ALL, _handle_param_get_all);
    dispatcher->register_handler(HostCmd::CALIBRATE, _handle_calibrate);
    dispatcher->register_handler(HostCmd::BASELINE_RESET, _handle_baseline_reset);
    dispatcher->register_handler(HostCmd::MODE_SET, _handle_mode_set);
    dispatcher->register_handler(HostCmd::CSD_CAPTURE, _handle_csd_capture);
    dispatcher->register_handler(HostCmd::CP_MEASURE, _handle_cp_measure);
    dispatcher->register_handler(HostCmd::CP_GET, _handle_cp_get);
    dispatcher->register_handler(HostCmd::GLOBAL_GET, _handle_global_get);
    dispatcher->register_handler(HostCmd::GLOBAL_SET, _handle_global_set);
    dispatcher->register_handler(HostCmd::GLOBAL_GET_ALL, _handle_global_get_all);
    dispatcher->register_handler(HostCmd::GLOBAL_COMMIT, _handle_global_commit);
    dispatcher->register_handler(HostCmd::AUTO_TUNE, _handle_auto_tune);
    dispatcher->register_handler(HostCmd::PSOC_RESCUE, _handle_psoc_rescue);
    dispatcher->register_handler(HostCmd::ALGO_GET_INFO, _handle_algo_get_info);
    dispatcher->register_handler(HostCmd::ALGO_UPLOAD, _handle_algo_upload);
    dispatcher->register_handler(HostCmd::ALGO_APPLY, _handle_algo_apply);
    dispatcher->register_handler(HostCmd::ALGO_RESET_DEFAULT, _handle_algo_reset_default);
    dispatcher->register_handler(HostCmd::ALGO_SET_ROM, _handle_algo_set_rom);
    dispatcher->register_handler(HostCmd::ALGO_GET_ROM, _handle_algo_get_rom);

    dispatcher->register_handler(HostCmd::ALGO_SET_CFG, _handle_algo_set_cfg);
    dispatcher->register_handler(HostCmd::ALGO_GET_CFG, _handle_algo_get_cfg);
    dispatcher->register_handler(HostCmd::ALGO_SET_CFG_CH, _handle_algo_set_cfg_ch);
    dispatcher->register_handler(HostCmd::ALGO_GET_CFG_CH, _handle_algo_get_cfg_ch);
    dispatcher->register_handler(HostCmd::ALGO_GET_SRC, _handle_algo_get_src);
    dispatcher->register_handler(HostCmd::ALGO_SET_SRC, _handle_algo_set_src);
    dispatcher->register_handler(HostCmd::ALGO_GET_CODE, _handle_algo_get_code);
}

void SensorLink::_poll_host_write() {
    if (!_host_write.active || _host_write.complete) return;
    bool ok = false;
    if (!Psoc::getInstance()->take_host_write_result(&ok)) return;
    if (!ok) {
        _host_write.ok = false;
        _host_write.complete = true;
        return;
    }
    // 批量类(ROM / CFG_CH): 每条 PSoC 终态回来就落一条 RP 存储, 再投下一条; 全部完成才 ACK。
    // 落存储放在"PSoC 这条成功之后"是刻意的: RP 存储是下次启动重新下发的真相源, 不能把 PSoC
    // 根本没收下的值记成既成事实。
    if (_host_write.kind == HostWriteState::Kind::ALGO_ROM ||
        _host_write.kind == HostWriteState::Kind::ALGO_CFG_CH) {
        if (_host_write.kind == HostWriteState::Kind::ALGO_ROM) {
            PsocAlgo::getInstance()->set_rom(_host_write.a, static_cast<uint16_t>(_host_write.value));
        } else {
            PsocAlgo::getInstance()->set_cfg_ch(_host_write.a, _host_write.b,
                                                static_cast<uint8_t>(_host_write.value));
        }
        _host_write.batch_index++;
        if (_host_write.batch_index < _host_write.batch_count) {
            if (_start_next_host_batch()) return;
            _host_write.ok = false;
            _host_write.complete = true;
            return;
        }
        _host_write.ok = true;
        _host_write.complete = true;
        return;
    }
    _host_write.ok = true;
    switch (_host_write.kind) {
        case HostWriteState::Kind::PARAM:
            CsdConfig::getInstance()->note_param(_host_write.a, _host_write.b, _host_write.value);
            break;
        case HostWriteState::Kind::MODE:
            CsdConfig::getInstance()->note_mode(_host_write.a);
            break;
        case HostWriteState::Kind::GLOBAL:
            CsdConfig::getInstance()->note_global(_host_write.a, _host_write.value);
            break;
        case HostWriteState::Kind::ALGO_CFG:
            PsocAlgo::getInstance()->set_cfg(_host_write.a, _host_write.b);
            break;
        default:
            break;
    }
    _host_write.complete = true;
}

bool SensorLink::_start_next_host_batch() {
    if (_host_write.batch_index >= _host_write.batch_count) return false;
    const HostWriteState::BatchEntry& e = _host_write.batch[_host_write.batch_index];
    _host_write.a = e.a;
    _host_write.b = e.b;
    _host_write.value = e.value;
    Psoc* psoc = Psoc::getInstance();
    if (_host_write.kind == HostWriteState::Kind::ALGO_CFG_CH) {
        return psoc->start_host_algo_set_cfg_ch(e.a, e.b, static_cast<uint8_t>(e.value));
    }
    return psoc->start_host_algo_set_rom(e.a, e.value);
}

bool SensorLink::_start_host_write(HostWriteState::Kind kind, const HostFrame& frame,
                                   uint8_t a, uint8_t b, uint32_t value) {
    if (_host_write.active) return false;
    _host_write.clear();
    _host_write.active = true;
    _host_write.kind = kind;
    _host_write.cmd = frame.cmd;
    _host_write.seq = frame.seq;
    _host_write.a = a;
    _host_write.b = b;
    _host_write.value = value;
    Psoc* psoc = Psoc::getInstance();
    bool accepted = false;
    switch (kind) {
        case HostWriteState::Kind::PARAM:
            accepted = psoc->start_host_param_set(a, b, value);
            break;
        case HostWriteState::Kind::MODE:
            accepted = psoc->start_host_mode_set(a);
            break;
        case HostWriteState::Kind::GLOBAL:
            accepted = psoc->start_host_global_set(a, value);
            break;
        case HostWriteState::Kind::GLOBAL_COMMIT:
            accepted = psoc->start_host_global_commit();
            break;
        case HostWriteState::Kind::ALGO_CFG:
            accepted = psoc->start_host_algo_set_cfg(a, b);
            break;
        case HostWriteState::Kind::ALGO_ROM:
            accepted = psoc->start_host_algo_set_rom(a, static_cast<uint16_t>(value));
            break;
        case HostWriteState::Kind::ALGO_CFG_CH:
            accepted = psoc->start_host_algo_set_cfg_ch(a, b, static_cast<uint8_t>(value));
            break;
        case HostWriteState::Kind::CALIBRATE:
            accepted = psoc->start_host_calibrate(a);
            break;
        case HostWriteState::Kind::BASELINE_RESET:
            accepted = psoc->start_host_baseline_reset(a);
            break;
        case HostWriteState::Kind::CP_MEASURE:
            accepted = psoc->start_host_measure_cp();
            break;
        default:
            return false;
    }
    if (!accepted) {
        _host_write.clear();
        return false;
    }
    return true;
}

bool SensorLink::take_host_write_terminal(uint8_t* cmd, uint8_t* seq, bool* ok) {
    _poll_host_write();
    if (!_host_write.active || !_host_write.complete) return false;
    if (cmd) *cmd = _host_write.cmd;
    if (seq) *seq = _host_write.seq;
    if (ok) *ok = _host_write.ok;
    _host_write_last_cmd = _host_write.cmd;
    _host_write_last_seq = _host_write.seq;
    _host_write_last_ok = _host_write.ok;
    _host_write_last_ms = to_ms_since_boot(get_absolute_time());
    _host_write.clear();
    return true;
}

bool SensorLink::replay_host_write(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (_host_write.active && frame.cmd == _host_write.cmd && frame.seq == _host_write.seq) {
        *response_length = 0u;
        return true;
    }
    if (_host_write_last_ms != 0u && frame.cmd == _host_write_last_cmd && frame.seq == _host_write_last_seq &&
        (uint32_t)(to_ms_since_boot(get_absolute_time()) - _host_write_last_ms) <= 10000u) {
        *response_length = _host_write_last_ok
            ? HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX)
            : HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR, "PSoC command failed",
                                       response, HOST_CMD_RESP_BUF_MAX);
        return true;
    }
    return false;
}

void SensorLink::_release_focus_scan() {
    // Focus is a host-side snapshot/output optimization.  The PSoC must keep its
    // proven continuous all-channel scan loop: device-side FOCUS_SCAN can ACK yet
    // leave scan_count stalled on the deployed firmware.  Disabling the RP fast
    // snapshot selector is sufficient to restore broad snapshot paging.
    Psoc::getInstance()->set_focus_channel(0xFFu);
}

void SensorLink::stop() {
    _stream.clear();
    _focus.clear();
    _release_focus_scan();
    if (_sweep.active()) {
        _sweep.output_restore = SweepOutputRestore::STOPPED;
        _begin_sweep_restore(SweepDataState::CANCELLED);
        return;
    }
    _saved_stream.clear();
    Psoc::getInstance()->set_telemetry_active(false);
    TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
    TxScheduler::getInstance()->cancel(TX_TASK_FOCUS);
}

void SensorLink::prepare_host_session() {
    // 命令响应由 UsbComm 独占 vendor TX；这里仅处理遗留的异步输出状态。
    // 扫描期的 telemetry/focus 是恢复与采样链的一部分，不能因 HELLO 被取消或抢走快照快路。
    if (_sweep.active()) {
        _stream.clear();
        _focus.clear();
        _saved_stream.clear();
        _sweep.output_restore = SweepOutputRestore::QUIET;
        TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
        TxScheduler::getInstance()->cancel(TX_TASK_FOCUS);
        return;
    }
    stop();
}

void SensorLink::suspend() {
    if (_sweep.active()) return;
    if (_focus.active) {
        if (_focus.suspended) return;
        _focus.suspended = true;
        _release_focus_scan();
        TxScheduler::getInstance()->cancel(TX_TASK_FOCUS);
    } else {
        if (!_stream.active || _stream.suspended) return;
        _stream.suspended = true;
        TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
    }
    Psoc::getInstance()->set_telemetry_active(false);   // 快照慢路暂停, 触控快路不受影响
}

void SensorLink::resume() {
    if (_sweep.active()) return;
    if (_focus.active) {
        if (!_focus.suspended && TxScheduler::getInstance()->active(TX_TASK_FOCUS)) return;
        _focus.suspended = false;
        // Keep the PSoC scan engine in continuous broad-scan mode. Focus narrows
        // only the RP snapshot/output path, which is sufficient for exclusive
        // delivery without risking a device-side scan stall.
        _focus.last_focus_scan_ack_us = time_us_32();
        Psoc::getInstance()->set_telemetry_active(true);
        _focus.last_generation = Psoc::getInstance()->snapshot_generation();
        _focus.generation_fence = 1u;
        Psoc::getInstance()->set_focus_channel(_focus.channel);
        const uint32_t interval_us = 1000000UL / ((_focus.rate_hz != 0u) ? _focus.rate_hz : 1u);
        TxScheduler::getInstance()->schedule(TX_TASK_FOCUS, interval_us, _focus.lease_ms,
                                             &SensorLink::emit_focus_task,
                                             &SensorLink::focus_lease_expired_task);
        return;
    }
    if (!_stream.active || !_stream.suspended) return;
    _stream.suspended = false;
    Psoc::getInstance()->set_telemetry_active(true);
    const uint32_t interval_us = 1000000UL / ((_rate_hz != 0u) ? _rate_hz : 1u);
    TxScheduler::getInstance()->schedule(TX_TASK_TELEM, interval_us, _lease_ms,
                                         &SensorLink::emit_telem_task);
}

void SensorLink::_pause_broad_for_focus() {
    _saved_stream.clear();
    if (!_stream.active) return;

    _saved_stream.valid = true;
    _saved_stream.state = _stream;
    _saved_stream.lease_ms = _lease_ms;
    _saved_stream.mode = _mode;
    _saved_stream.rate_hz = _rate_hz;
    _saved_stream.fields = _fields;
    _saved_stream.ch_mask = _ch_mask;
    _stream.suspended = true;
    TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
}

void SensorLink::_restore_broad_after_focus() {
    if (!_saved_stream.valid) {
        Psoc::getInstance()->set_telemetry_active(false);
        return;
    }

    _stream = _saved_stream.state;
    _lease_ms = _saved_stream.lease_ms;
    _mode = _saved_stream.mode;
    _rate_hz = _saved_stream.rate_hz;
    _fields = _saved_stream.fields;
    _ch_mask = _saved_stream.ch_mask;
    _saved_stream.clear();
    if (!_stream.emitting()) {
        Psoc::getInstance()->set_telemetry_active(false);
        return;
    }

    Psoc::getInstance()->set_telemetry_active(true);
    const uint32_t interval_us = 1000000UL / ((_rate_hz != 0u) ? _rate_hz : 1u);
    TxScheduler::getInstance()->schedule(TX_TASK_TELEM, interval_us, _lease_ms,
                                         &SensorLink::emit_telem_task);
}

void SensorLink::emit_telem_task() {
    getInstance()->tick();
}

void SensorLink::tick() {
    // 周期由 TxScheduler 定时任务驱动; 本函数只负责"发送一帧"(不再自门控频率)。
    if (!_stream.emitting()) return;

    const uint32_t now_us = time_us_32();

    const psoc::SensorSnapshot& snapshot = Psoc::getInstance()->snapshot();
    uint16_t length = 0;
    uint8_t* payload = _telem_frame.payload;

    payload[length++] = static_cast<uint8_t>(now_us);
    payload[length++] = static_cast<uint8_t>(now_us >> 8);
    payload[length++] = static_cast<uint8_t>(now_us >> 16);
    payload[length++] = static_cast<uint8_t>(now_us >> 24);
    const uint16_t channel_count_position = length++;
    payload[length++] = _fields;

    if ((_fields & TELEM_FIELD_STATS) != 0) {
        const uint32_t sps = Psoc::getInstance()->samples_per_sec();
        const uint32_t spu = Psoc::getInstance()->scan_period_us();
        payload[length++] = (uint8_t)sps; payload[length++] = (uint8_t)(sps >> 8);
        payload[length++] = (uint8_t)(sps >> 16); payload[length++] = (uint8_t)(sps >> 24);
        payload[length++] = (uint8_t)spu; payload[length++] = (uint8_t)(spu >> 8);
        payload[length++] = (uint8_t)(spu >> 16); payload[length++] = (uint8_t)(spu >> 24);
    }

    if ((_fields & TELEM_FIELD_LATENCY) != 0) {
        const uint16_t ls = g_lat_spi_us, lp = g_lat_proc_us, lu = g_lat_usb_us;
        payload[length++] = (uint8_t)ls; payload[length++] = (uint8_t)(ls >> 8);
        payload[length++] = (uint8_t)lp; payload[length++] = (uint8_t)(lp >> 8);
        payload[length++] = (uint8_t)lu; payload[length++] = (uint8_t)(lu >> 8);
        g_lat_spi_us = 0; g_lat_proc_us = 0; g_lat_usb_us = 0;  // 清零开新窗口
    }

    if ((_fields & TELEM_FIELD_DELAY_DEV) != 0) {
        const int16_t lo = g_delay_dev_min_us, hi = g_delay_dev_max_us;
        const uint8_t flags = g_delay_dev_flags;
        payload[length++] = (uint8_t)lo; payload[length++] = (uint8_t)((uint16_t)lo >> 8);
        payload[length++] = (uint8_t)hi; payload[length++] = (uint8_t)((uint16_t)hi >> 8);
        payload[length++] = flags;
        // 清零开新窗口。flags 的 VALID 位即"本窗口有观测", 故 min/max 归零不会被误读成真实观测。
        g_delay_dev_min_us = 0; g_delay_dev_max_us = 0; g_delay_dev_flags = 0;
    }

    // ★触发判定必须是 PSoC 的最终判定★
    // 快照里的 sample.status 是 CapSense widget 的原始状态, **不经 JIT 算法**。装了改判算法后
    // 它与设备实际触发不一致(实测界面"触发判定"恒不动)。PSoC 的最终判定唯一真相源是它的 TOUCH
    // 帧(update_touch_frame 已含算法 out_active), RP 侧即 touch_mask()。
    // 无算法时该位就是 PSoC 内部基线差判定, 语义同样正确。
    const uint64_t final_touch = Psoc::getInstance()->touch_mask();

    uint8_t channel_count = 0;
    for (uint8_t channel = 0; channel < SENSOR_LINK_CHANNELS; channel++) {
        if (!_channel_selected(_ch_mask, channel)) continue;
        if (length + 8 > HOST_CMD_PAYLOAD_MAX) break;

        uint16_t raw = 0;
        uint16_t baseline = 0;
        int16_t diff = 0;
        uint8_t status = 0;
        if (snapshot.valid) {
            const psoc::SensorSample& sample = snapshot.channels[channel];
            raw = sample.raw;
            baseline = sample.baseline;
            diff = sample.diff;
            // bit0 取最终判定, 其余位保留 CapSense 原始状态位供诊断。
            status = static_cast<uint8_t>((sample.status & 0xFEu) |
                (((final_touch >> channel) & 1ull) != 0ull ? 1u : 0u));
        }

        payload[length++] = channel;
        if ((_fields & TELEM_FIELD_RAW) != 0) {
            payload[length++] = static_cast<uint8_t>(raw);
            payload[length++] = static_cast<uint8_t>(raw >> 8);
        }
        if ((_fields & TELEM_FIELD_BASELINE) != 0) {
            payload[length++] = static_cast<uint8_t>(baseline);
            payload[length++] = static_cast<uint8_t>(baseline >> 8);
        }
        if ((_fields & TELEM_FIELD_DIFF) != 0) {
            payload[length++] = static_cast<uint8_t>(diff);
            payload[length++] = static_cast<uint8_t>(static_cast<uint16_t>(diff) >> 8);
        }
        if ((_fields & TELEM_FIELD_STATUS) != 0) payload[length++] = status;
        channel_count++;
    }
    payload[channel_count_position] = channel_count;

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::TELEM_DATA);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;

    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length > 0) {
        HAL_USB_Device* usb = HAL_USB_Device::getInstance();
        // config_write() 会自行按当前 FIFO 空间分段并在有限预算内推进 USB。
        // 不能要求 FIFO 一次容纳整帧：全局 36 通道帧通常大于瞬时可用空间，
        // 原先的 `available >= frame_length` 会让广谱帧永久被跳过；FOCUS 小帧却能通过，
        // 于是表现为“单通道正常、全局遥测过期”。响应正在发送时 available 也会由 HAL 置零，
        // 本轮跳过即可，下一轮继续尝试，避免覆盖命令响应缓冲。
        if (usb->config_write_available() > 0) {
            usb->config_write(_tx_buf, frame_length);
        }
    }
    if (_mode == 1) {
        stop();   // 单次模式: 一帧即止(与主机显式 TELEM_STOP 同语义, 不自动恢复)
    }
}

void SensorLink::emit_focus_task() {
    getInstance()->focus_tick();
}

// 独占流租约到期(上位机丢失/长时间无主机帧)。★必须显式还原快照快路★ 租约到期只会把推送任务
// 停掉, 若不在这里复位独占目标, 设备会继续只刷新那一个通道 —— 没有任何消费者, 而广谱流恢复后
// 其余 35 个通道全是陈旧值(表现为"重连后只有一个通道在动")。
void SensorLink::focus_lease_expired_task() {
    SensorLink* self = getInstance();
    self->_focus.clear();
    self->_release_focus_scan();
    self->_restore_broad_after_focus();
}

void SensorLink::focus_tick() {
    if (!_focus.emitting()) return;

    Psoc* psoc = Psoc::getInstance();
    const uint32_t now_us = time_us_32();

    const psoc::SensorSnapshot& snapshot = psoc->snapshot();
    if (!snapshot.valid || snapshot.generation == _focus.last_generation) return;

    _focus.last_generation = snapshot.generation;
    if (_focus.generation_fence != 0u) {
        _focus.generation_fence--;
        return;
    }
    const uint16_t sample_seq = _focus.sample_seq++;
    const psoc::SensorSample& sample = snapshot.channels[_focus.channel];
    uint16_t length = 0;
    uint8_t* payload = _telem_frame.payload;

    payload[length++] = static_cast<uint8_t>(_focus.id);
    payload[length++] = static_cast<uint8_t>(_focus.id >> 8);
    payload[length++] = static_cast<uint8_t>(sample_seq);
    payload[length++] = static_cast<uint8_t>(sample_seq >> 8);
    payload[length++] = static_cast<uint8_t>(snapshot.generation);
    payload[length++] = static_cast<uint8_t>(snapshot.generation >> 8);
    payload[length++] = _focus.channel;
    payload[length++] = _focus.fields;
    payload[length++] = static_cast<uint8_t>(now_us);
    payload[length++] = static_cast<uint8_t>(now_us >> 8);
    payload[length++] = static_cast<uint8_t>(now_us >> 16);
    payload[length++] = static_cast<uint8_t>(now_us >> 24);
    if ((_focus.fields & TELEM_FIELD_RAW) != 0) {
        payload[length++] = static_cast<uint8_t>(sample.raw);
        payload[length++] = static_cast<uint8_t>(sample.raw >> 8);
    }
    if ((_focus.fields & TELEM_FIELD_BASELINE) != 0) {
        payload[length++] = static_cast<uint8_t>(sample.baseline);
        payload[length++] = static_cast<uint8_t>(sample.baseline >> 8);
    }
    if ((_focus.fields & TELEM_FIELD_DIFF) != 0) {
        payload[length++] = static_cast<uint8_t>(sample.diff);
        payload[length++] = static_cast<uint8_t>(static_cast<uint16_t>(sample.diff) >> 8);
    }
    if ((_focus.fields & TELEM_FIELD_STATUS) != 0) {
        // 同全通道路径: bit0 用 PSoC 最终触发判定(含算法改判), 高位保留 CapSense 原始状态。
        const uint64_t final_touch = psoc->touch_mask();
        payload[length++] = static_cast<uint8_t>((sample.status & 0xFEu) |
            (((final_touch >> _focus.channel) & 1ull) != 0ull ? 1u : 0u));
    }
    // ★算法运行值随本帧一起走★ 顺序固定在 STATUS 之后、STATS 之前, 与上位机解码一一对应。
    // 只有该组值确实属于本帧的通道时才算有效, 否则填 0 并把 active 置 0(上位机据此显示无值)。
    if ((_focus.fields & TELEM_FIELD_ALGO) != 0) {
        const bool algo_ok = snapshot.algo_channel == _focus.channel;
        payload[length++] = algo_ok ? snapshot.algo_active : 0u;
        for (size_t i = 0; i < psoc::ALGO_REPORT_SLOTS; ++i) {
            const uint16_t value = algo_ok ? snapshot.algo_report[i] : 0u;
            payload[length++] = static_cast<uint8_t>(value);
            payload[length++] = static_cast<uint8_t>(value >> 8);
        }
    }
    if ((_focus.fields & TELEM_FIELD_STATS) != 0) {
        const uint32_t sps = Psoc::getInstance()->samples_per_sec();
        const uint32_t spu = Psoc::getInstance()->scan_period_us();
        payload[length++] = static_cast<uint8_t>(sps); payload[length++] = static_cast<uint8_t>(sps >> 8);
        payload[length++] = static_cast<uint8_t>(sps >> 16); payload[length++] = static_cast<uint8_t>(sps >> 24);
        payload[length++] = static_cast<uint8_t>(spu); payload[length++] = static_cast<uint8_t>(spu >> 8);
        payload[length++] = static_cast<uint8_t>(spu >> 16); payload[length++] = static_cast<uint8_t>(spu >> 24);
    }
    if ((_focus.fields & TELEM_FIELD_LATENCY) != 0) {
        const uint16_t ls = g_lat_spi_us, lp = g_lat_proc_us, lu = g_lat_usb_us;
        payload[length++] = static_cast<uint8_t>(ls); payload[length++] = static_cast<uint8_t>(ls >> 8);
        payload[length++] = static_cast<uint8_t>(lp); payload[length++] = static_cast<uint8_t>(lp >> 8);
        payload[length++] = static_cast<uint8_t>(lu); payload[length++] = static_cast<uint8_t>(lu >> 8);
        g_lat_spi_us = 0; g_lat_proc_us = 0; g_lat_usb_us = 0;
    }
    if ((_focus.fields & TELEM_FIELD_DELAY_DEV) != 0) {
        const int16_t lo = g_delay_dev_min_us, hi = g_delay_dev_max_us;
        const uint8_t flags = g_delay_dev_flags;
        payload[length++] = static_cast<uint8_t>(lo);
        payload[length++] = static_cast<uint8_t>(static_cast<uint16_t>(lo) >> 8);
        payload[length++] = static_cast<uint8_t>(hi);
        payload[length++] = static_cast<uint8_t>(static_cast<uint16_t>(hi) >> 8);
        payload[length++] = flags;
        g_delay_dev_min_us = 0; g_delay_dev_max_us = 0; g_delay_dev_flags = 0;
    }

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::FOCUS_DATA);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;
    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    HAL_USB_Device* usb = HAL_USB_Device::getInstance();
    if (frame_length > 0 && usb->config_write_available() > 0) {
        usb->config_write(_tx_buf, frame_length);
    }
}

void SensorLink::emit_sweep_task() {
    getInstance()->sweep_tick();
}

// ★扫描会话独占目标通道的 gain/div 修改权★
// 期间放行调参/校准/自适应会与逐格 runtime apply 或原值恢复竞争，导致状态无法证明；
// DEVICE_BUSY 是上位机既有的可重试语义, 比让两边同时改同一个通道诚实。
bool SensorLink::_sweep_busy_reject(const char* what, const HostFrame& frame,
                                    uint8_t* response, uint16_t* response_length) {
    if (!getInstance()->_sweep.active()) return false;
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
                                               what, response, HOST_CMD_RESP_BUF_MAX);
    return true;
}

// 租约到期(上位机丢失/长时间无主机帧)。★不能只是把任务停掉★: 此刻 PSoC 的 gain/div 仍停在扫描
// 中间值上, 直接停等于把设备永久留在错参数下。故一律转入恢复, 并把任务换成【无租约】继续跑完;
// 已在恢复中时只重新武装(防 renew_all 给无租约任务重新塞进一个租约后到期把它悄悄停掉)。
void SensorLink::sweep_lease_expired_task() {
    SensorLink* self = getInstance();
    if (!self->_sweep.active()) return;
    if (self->_sweep_phase_is_restore(self->_sweep.phase)) {
        TxScheduler::getInstance()->schedule(TX_TASK_SWEEP, SWEEP_INTERVAL_US, 0u,
                                             &SensorLink::emit_sweep_task,
                                             &SensorLink::sweep_lease_expired_task);
        return;
    }
    self->_begin_sweep_restore(SweepDataState::CANCELLED);
}

// 扫描期间 PSoC 的 gain/div 被逐格改写, raw/touch 全是无意义值。触控消费方(键盘映射 / mai2 上报 /
// 指触绑定)据此把输出按"全松开"处理, 而不是把扫描噪声当成真实触摸打出去。
// 生命周期 = 会话存在期间(含恢复与终态重试), 由 _finish_sweep_session() 单点解除。
bool SensorLink::output_suppressed() const {
    return _sweep.active();
}

uint16_t SensorLink::_sqrt_u32(uint32_t value) {
    // 逐位试根(无浮点、无除法): 结果为 floor(sqrt(value))。
    uint32_t root = 0;
    uint32_t bit = 1u << 30;
    while (bit > value) bit >>= 2;
    while (bit != 0u) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<uint16_t>(root);
}

// 单格终结: 真实采样点标记为 sampled/resolved；连续前缀中的未采样空洞随后按 index 发送空 CELL。
void SensorLink::_finish_sweep_cell() {
    const uint16_t index = _sweep.cell;
    SweepResult& result = _sweep_results[index];
    const SweepStats& stats = _sweep.stats;
    result.samples = static_cast<uint16_t>(stats.count);
    if (stats.count != 0u) {
        const uint32_t mean = static_cast<uint32_t>(stats.sum / stats.count);
        result.mean = static_cast<uint16_t>(mean);
        result.pp = static_cast<uint16_t>((stats.max >= stats.min) ? (stats.max - stats.min) : 0u);
        const uint32_t mean_sq = mean * mean;
        const uint32_t e_sq = static_cast<uint32_t>(stats.sum_sq / stats.count);
        const uint32_t variance = (e_sq > mean_sq) ? (e_sq - mean_sq) : 0u;
        const uint32_t root = _sqrt_u32(variance);
        const uint32_t frac = ((variance - root * root) << 8) / (2u * root + 1u);
        const uint32_t std_q8 = (root << 8) + frac;
        result.std_q8 = (std_q8 > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(std_q8);
        // 出界(饱和或恒零)才是该格不可用; 低噪声(pp=0)是有效且理想的结果, 不再据此判失败。
        if (stats.railed != 0u) result.flags |= SWEEP_FLAG_RAILED;
    }

    const uint8_t invalid_flags = SWEEP_FLAG_CAL_FAIL | SWEEP_FLAG_MISMATCH |
                                  SWEEP_FLAG_STALLED | SWEEP_FLAG_RAILED;
    const uint8_t trigger_flags = result.flags & invalid_flags;
    const bool valid = result.samples != 0u && trigger_flags == 0u;
    if (!valid) {
        const bool expected_invalid = result.samples != 0u &&
            (trigger_flags & static_cast<uint8_t>(~(SWEEP_FLAG_RAILED | SWEEP_FLAG_STALLED))) == 0u;
        if (!expected_invalid) {
            const uint8_t fail_phase = (result.fail_phase != 0u)
                ? result.fail_phase : static_cast<uint8_t>(_sweep.phase);
            result.fail_phase = fail_phase;
            _sweep_note_fail_phase();
        }
        _sweep.failed_cells++;
    }

    if (! _sweep.sampled[index]) {
        _sweep.sampled[index] = true;
    }
    _sweep.resolved[index] = true;
    while (_sweep.produced < SWEEP_TOTAL && _sweep.resolved[_sweep.produced]) _sweep.produced++;
    // 采到样本就算这一格闭合了(railed 也是有效结论, 用户要看到它); 只有压根没采到才算失败连击。
    if (result.samples != 0u) _sweep.fail_run = 0u;
    _advance_sweep_cell();
}

// 会话收尾(扫完 / 取消 / 租约到期 / 真故障)时把剩余未决格判空。
// 顺序全覆盖下, 只有**被中断**才会留下未测格; 它们以 samples=0 如实发出, 不冒充测过。
void SensorLink::_resolve_sweep_tail() {
    for (uint16_t i = _sweep.produced; i < SWEEP_TOTAL; ++i) {
        if (!_sweep.sampled[i]) _sweep_results[i].clear();
        _sweep.resolved[i] = true;
    }
    _sweep.produced = SWEEP_TOTAL;
}

// 顺序推进到下一格, 直到 448 格全部实测完。
//
// ★为什么不再用"探针 + 两翼扩散"★(那是本轮修掉的根因)
// 旧策略把 cell 当成一维区间: PROBE 每 10 格试探一次, 试到有效格才向左右扩散, 一侧遇到首个无效
// 格就封口、并从该点 +10 处重新试探。它有两个致命前提, 而两个都不成立:
//   ① 有效区是**连续区间** —— 但 cell = gain*64 + (div-1) 是把 7×64 的二维网格压成一维,
//      gain 行之间的接缝处根本不连续, 一个 gain 行的有效区与下一行毫无关系;
//   ② 跳过的格子可以不测 —— 而封口/试探跳过的 [边界+1, 边界+10) 会被 `_resolve_sweep_prefix`
//      直接标成 resolved 且 samples=0, produced 照样推到 448。于是上位机看到"produced=448"
//      却只有增益 0 附近那一段真的有数据(用户报的"只扫增益0就结束 400+ 点"就是这个)。
// 这张热图的用途是让用户在整个 gain×div 空间里挑噪声最低的点, 少测一格就是少一个候选;
// 而"哪些格无效"本身也是要看的结论(极端组合会 railed), 不该由设备提前替用户裁掉。
// 所以改为全覆盖顺序扫: 唯一的代价是时长, 而这本来就是一次显式发起的长任务。
void SensorLink::_advance_sweep_cell() {
    const uint16_t next = static_cast<uint16_t>(_sweep.cell + 1u);
    if (next >= SWEEP_TOTAL) {
        _resolve_sweep_tail();
        _begin_sweep_restore(SweepDataState::DONE);
        return;
    }
    _sweep.cell = next;
    _sweep_enter(SweepPhase::SET_CELL);
}

void SensorLink::_sweep_note_fail_phase() {
    if (_sweep.fail_phase == 0u) _sweep.fail_phase = static_cast<uint8_t>(_sweep.phase);
}

// 当前格无法闭合: 记成协议可见的空洞(samples=0, fail_phase 保留卡住的阶段)并继续下一格。
//
// ★单格失败不再毙掉整个会话★(本轮修掉的第二个根因)
// 原实现在这里直接 _begin_sweep_restore(FAILED): 极端 gain/div 组合下 PSoC 本来就可能吃不下
// (runtime apply 确认不了 / 回读被钳位), 于是整轮扫描常常在头几格就终止, 热图几乎全空。
// 每一格都会重新写入 gain/div 并回读确认, 硬件状态每格自行重建 —— 继续下一格并不建立在
// "未知状态"上。真故障(PSoC 卡死)表现为**连续**失败, 由 SWEEP_ABORT_FAIL_RUN 收敛。
void SensorLink::_abort_sweep_cell() {
    const uint16_t index = _sweep.cell;
    SweepResult& result = _sweep_results[index];
    const uint8_t gain = static_cast<uint8_t>(index / SWEEP_DIV_COUNT);
    const uint8_t div = static_cast<uint8_t>((index % SWEEP_DIV_COUNT) + 1u);
    // ★诊断位必须保住★ 原实现先 clear 再走人, 把 READBACK 刚置上的 MISMATCH、超时刚置上的
    // STALLED 一起擦掉 —— 上位机于是只看到"samples=0 且 flags=0", 分不清是钳位、停滞还是没下发。
    const uint8_t kept_flags = result.flags;
    result.clear();
    result.flags = kept_flags;
    result.gain = gain;
    result.div = div;
    result.fail_phase = static_cast<uint8_t>(_sweep.phase);
    _sweep_note_fail_phase();
    _sweep.failed_cells++;
    _sweep.stats.clear();
    _sweep.sampled[index] = true;
    _sweep.resolved[index] = true;
    while (_sweep.produced < SWEEP_TOTAL && _sweep.resolved[_sweep.produced]) _sweep.produced++;
    if (++_sweep.fail_run >= SWEEP_ABORT_FAIL_RUN) {
        _begin_sweep_restore(SweepDataState::FAILED);
        return;
    }
    _advance_sweep_cell();
}

// 扫描期(非恢复链)阶段的有界超时处置: 当前格记空洞后继续下一格(见 _abort_sweep_cell)。
// SETTLE/SAMPLE 等不到新快照代次属"扫描停滞", 这是该格的真实结论, 必须让上位机看到。
void SensorLink::_sweep_cell_timeout(uint32_t limit) {
    if (_sweep.phase_ticks <= limit) return;
    _sweep_note_fail_phase();
    if (_sweep.phase == SweepPhase::SETTLE || _sweep.phase == SweepPhase::SAMPLE) {
        _sweep_results[_sweep.cell].flags |= SWEEP_FLAG_STALLED;
    }
    _abort_sweep_cell();
}

// 扫描不再只发送实际采样点：produced 是从 0 开始连续已判定前缀长度。
// 已判定但未采样的空洞同样发送 CELL(samples=0, flags=0)，已采样点保留真实结果。
// SWEEP_DATA(0x38) 定长 23 字节:
//   [session u16][state u8][ch u8][index u16][total u16][produced u16][flags u8]
//   [gain u8][div u8][samples u16][mean u16][std_q8 u16][pp u16][fail_phase u8][phase u8]
// 非结果帧(RESTORING/DONE/CANCELLED/FAILED) index=0xFFFF 且格字段为 0, flags 回显恢复阶段异常位。
// ★尾部追加而非改布局★ 前 21 字节与旧固件逐字节相同(与 KBD_GET_STATE / AUTO_TUNE_PROGRESS 同一
// 先例), 只读前 21 字节的旧上位机不受影响; 新增两项让每个阶段都可枚举:
//   fail_phase = 结果帧→该格卡住的阶段, 非结果帧→本会话首个失败阶段(0=无故障);
//   phase      = 发帧当刻的会话阶段(SweepPhase), 用于追踪"卡在哪一步"这类未知故障。
// 返回是否真的发出: 背压时保持游标；补发允许 resolved 前缀中的空洞。
bool SensorLink::_emit_sweep_frame(uint16_t index, SweepDataState state, bool retransmit) {
    const bool has_cell = (state == SweepDataState::CELL) && (index < SWEEP_TOTAL);
    const SweepResult& result = _sweep_results[has_cell ? index : 0u];
    uint8_t* payload = _telem_frame.payload;
    uint16_t length = 0;
    payload[length++] = static_cast<uint8_t>(_sweep.id);
    payload[length++] = static_cast<uint8_t>(_sweep.id >> 8);
    payload[length++] = static_cast<uint8_t>(state);
    payload[length++] = _sweep.channel;
    payload[length++] = static_cast<uint8_t>(index);
    payload[length++] = static_cast<uint8_t>(index >> 8);
    payload[length++] = static_cast<uint8_t>(SWEEP_TOTAL);
    payload[length++] = static_cast<uint8_t>(SWEEP_TOTAL >> 8);
    payload[length++] = static_cast<uint8_t>(_sweep.produced);
    payload[length++] = static_cast<uint8_t>(_sweep.produced >> 8);
    uint8_t flags = has_cell ? result.flags : _sweep.restore_flags;
    if (retransmit) flags |= SWEEP_FLAG_RETRANSMIT;
    payload[length++] = flags;
    payload[length++] = has_cell ? result.gain : 0u;
    payload[length++] = has_cell ? result.div : 0u;
    const uint16_t samples = has_cell ? result.samples : 0u;
    const uint16_t mean = has_cell ? result.mean : 0u;
    const uint16_t std_q8 = has_cell ? result.std_q8 : 0u;
    const uint16_t pp = has_cell ? result.pp : 0u;
    payload[length++] = static_cast<uint8_t>(samples);
    payload[length++] = static_cast<uint8_t>(samples >> 8);
    payload[length++] = static_cast<uint8_t>(mean);
    payload[length++] = static_cast<uint8_t>(mean >> 8);
    payload[length++] = static_cast<uint8_t>(std_q8);
    payload[length++] = static_cast<uint8_t>(std_q8 >> 8);
    payload[length++] = static_cast<uint8_t>(pp);
    payload[length++] = static_cast<uint8_t>(pp >> 8);
    payload[length++] = has_cell ? result.fail_phase : _sweep.fail_phase;
    payload[length++] = static_cast<uint8_t>(_sweep.phase);

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::SWEEP_DATA);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;
    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length == 0) return false;
    HAL_USB_Device* usb = HAL_USB_Device::getInstance();
    if (usb->config_write_available() < frame_length) return false;
    usb->config_write(_tx_buf, frame_length);
    return true;
}

// 取消 / 租约到期 / 扫描结束的唯一去处：仅写回原 gain/div，等待 runtime apply 结果并回读确认。
void SensorLink::_begin_sweep_restore(SweepDataState terminal) {
    if (!_sweep.active()) return;
    if (_sweep_phase_is_restore(_sweep.phase) || _sweep.restoring) return;
    // 恢复链前的结果前缀必须完整判定；扫描结束时把剩余未决尾部判空。
    _resolve_sweep_tail();
    // ★恢复期先交还快照快路★ 采样已结束, 恢复只需写回并回读 gain/div。继续武装单通道快路会让
    // core1 每周期都在做完整锁存(BEGIN+INFO+页), 命令环消费变慢 ⇒ 阻塞读类的 get_param 长期
    // 超时, 恢复链只能一路等到 120s 阶段兜底(实测会话停在 100% 迟迟不出终态)。
    _release_focus_scan();
    // 恢复链接管时显式复位启动闸门，确保只发起一次原 gain/div 的 runtime apply。
    _sweep.restoring = true;
    _sweep.restore_started = false;
    _sweep.terminal = terminal;
    _sweep.restore_announced = false;
    _sweep_enter(SweepPhase::RESTORE_WRITE);
    // 恢复期改为【无租约】任务: 触发点本身可能就是"上位机丢失", 再挂租约会让恢复半途被停掉。
    TxScheduler::getInstance()->schedule(TX_TASK_SWEEP, SWEEP_INTERVAL_US, 0u,
                                         &SensorLink::emit_sweep_task,
                                         &SensorLink::sweep_lease_expired_task);
}

void SensorLink::_finish_sweep_session() {
    const SweepOutputRestore restore = _sweep.output_restore;
    _sweep.clear();
    TxScheduler::getInstance()->cancel(TX_TASK_SWEEP);
    _release_focus_scan();
    if (restore != SweepOutputRestore::BROAD) {
        // STOPPED = 显式 TELEM_STOP；QUIET = 扫描期间收到 HELLO。两者都必须等原参数
        // 恢复链结束后再关闭快照慢路，不能在恢复中途抢走 PSoC 的扫描所有权。
        _saved_stream.clear();
        Psoc::getInstance()->set_telemetry_active(false);
        TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
        TxScheduler::getInstance()->cancel(TX_TASK_FOCUS);
        return;
    }
    if (_focus.active) {
        _focus.suspended = false;
        resume();                        // Focus 仍在: 复原单通道流(广谱保存态继续由 _saved_stream 持有)
        return;
    }
    _restore_broad_after_focus();         // 无 Focus: 复原(或如实关闭)广谱流
}

// SweepSession 单步推进。每周期只做一件事, 任何等待都靠阶段超时收敛, 不阻塞主循环。
void SensorLink::sweep_tick() {
    if (!_sweep.active()) return;
    Psoc* psoc = Psoc::getInstance();
    const uint8_t channel = _sweep.channel;
    _sweep.phase_ticks++;

    // 进入恢复后如实告知上位机(它据此把界面从"扫描中"切到"恢复中")。★发不出去也绝不挡恢复★:
    // 这只是一帧告知, 而恢复关系到设备是否停在错参数上, 两者优先级不可倒置。
    if (_sweep_phase_is_restore(_sweep.phase) && !_sweep.restore_announced) {
        _sweep.restore_announced = _emit_sweep_frame(SWEEP_INDEX_NONE, SweepDataState::RESTORING, false);
    }

    // 流式结果与补发先于状态推进: 终态帧必须排在全部格结果之后, 否则上位机会在收全数据前收到 DONE。
    bool tx_blocked = false;
    uint8_t budget = SWEEP_FRAMES_PER_TICK;
    while (!tx_blocked && budget != 0u && _sweep.resend_cursor < _sweep.resend_end) {
        const uint16_t index = _sweep.resend_cursor;
        if (_emit_sweep_frame(index, SweepDataState::CELL, true)) {
            _sweep.resend_cursor++;
            budget--;
        } else {
            tx_blocked = true;
        }
    }
    while (!tx_blocked && budget != 0u && _sweep.stream_cursor < _sweep.produced) {
        const uint16_t index = _sweep.stream_cursor;
        if (_emit_sweep_frame(index, SweepDataState::CELL, false)) {
            _sweep.stream_cursor++;
            budget--;
        } else {
            tx_blocked = true;
        }
    }
    // USB 背压时不能继续改 PSoC 参数：否则结果队列积压、IN FIFO 饱和，主机连 keepalive/补发也会收到
    // Windows ERROR_INVALID_FUNCTION(os error 22)，最终又因租约到期把仍在扫描的会话误取消。
    if (tx_blocked && !_sweep_phase_is_restore(_sweep.phase)) return;

    switch (_sweep.phase) {
        case SweepPhase::SET_CELL: {
            const uint8_t gain = static_cast<uint8_t>(_sweep.cell / SWEEP_DIV_COUNT);
            const uint8_t div = static_cast<uint8_t>((_sweep.cell % SWEEP_DIV_COUNT) + 1u);
            SweepResult& result = _sweep_results[_sweep.cell];
            result.gain = gain;
            result.div = div;
            if (!psoc->start_runtime_param_apply(channel, gain, div)) {
                _sweep_cell_timeout(SWEEP_WAIT_TICKS);
                return;
            }
            _sweep_enter(SweepPhase::WAIT_PARAMS);
            return;
        }

        case SweepPhase::WAIT_PARAMS: {
            bool ok = false;
            if (!psoc->take_runtime_param_apply_result(&ok)) {
                _sweep_cell_timeout(SWEEP_RUNTIME_APPLY_TICKS);
                return;
            }
            if (!ok) {
                SweepResult& result = _sweep_results[_sweep.cell];
                result.fail_phase = static_cast<uint8_t>(SweepPhase::WAIT_PARAMS);
                _sweep_note_fail_phase();
                _abort_sweep_cell();
                return;
            }
            _sweep_enter(SweepPhase::READBACK);
            return;
        }


        case SweepPhase::READBACK: {
            uint32_t gain = 0u;
            uint32_t div = 0u;
            // ★读不到 ≠ PSoC 拒绝★ get_param 是阻塞读类命令; Sweep 期间 core1 每周期都在推进
            // 单通道快照锁存, 命令环消费变慢时读类会本地超时。一次超时就把该格判死会让整张表在
            // 设备完全健康时全部无效(实测首格即 fail_phase=READBACK)。交给阶段兜底重试, 只有
            // 真的读回了却与期望不符才是 PSoC 侧钳位/拒绝。
            if (!psoc->get_param(channel, SWEEP_PARAM_ID_GAIN, &gain) ||
                !psoc->get_param(channel, SWEEP_PARAM_ID_DIV, &div)) {
                _sweep_cell_timeout(SWEEP_WAIT_TICKS);
                return;
            }
            const uint8_t expected_gain = static_cast<uint8_t>(_sweep.cell / SWEEP_DIV_COUNT);
            const uint8_t expected_div = static_cast<uint8_t>((_sweep.cell % SWEEP_DIV_COUNT) + 1u);
            if (gain != expected_gain || div != expected_div) {
                SweepResult& result = _sweep_results[_sweep.cell];
                result.flags |= SWEEP_FLAG_MISMATCH;
                result.fail_phase = static_cast<uint8_t>(SweepPhase::READBACK);
                _sweep_note_fail_phase();
                _abort_sweep_cell();
                return;
            }
            _sweep.stats.clear();
            _sweep.settle_seen = 0u;
            _sweep.last_generation = psoc->snapshot_generation();
            _sweep_enter(SweepPhase::SETTLE);
            return;
        }

        case SweepPhase::SETTLE: {
            // 丢弃运行时参数生效后的前若干帧，按快照代次计数而不按时间猜。
            if (!psoc->snapshot_valid() || psoc->snapshot_generation() == _sweep.last_generation) {
                _sweep_cell_timeout(SWEEP_WAIT_TICKS);
                return;
            }
            _sweep.last_generation = psoc->snapshot_generation();
            _sweep.phase_ticks = 0;
            if (++_sweep.settle_seen >= _sweep.settle_samples) _sweep_enter(SweepPhase::SAMPLE);
            return;
        }

        case SweepPhase::SAMPLE: {
            const psoc::SensorSnapshot& snapshot = psoc->snapshot();
            if (!snapshot.valid || snapshot.generation == _sweep.last_generation) {
                _sweep_cell_timeout(SWEEP_WAIT_TICKS);
                return;
            }
            _sweep.last_generation = snapshot.generation;
            _sweep.phase_ticks = 0;
            const uint16_t raw = snapshot.channels[channel].raw;
            SweepStats& stats = _sweep.stats;
            stats.count++;
            stats.sum += raw;
            stats.sum_sq += static_cast<uint64_t>(raw) * raw;
            if (raw < stats.min) stats.min = raw;
            if (raw > stats.max) stats.max = raw;
            // 饱和与恒零同属"该 gain/div 下电极不在可用工作区", 两侧出界都记 railed。
            if ((raw >= SWEEP_RAILED_RAW || raw == 0u) && stats.railed < 0xFFu) stats.railed++;
            // ★每一格用同一个样本数★ 原先每 8 格的段首只采 8 个样本当"快锚点"(为探针策略服务)。
            // 全覆盖之后那只会让 1/8 的格子拿 8 样本的标准差、其余拿 32 样本的标准差 —— 而这张图
            // 的全部用处就是横向比较各格的标准差, 样本数不一致等于把估计误差混进结论里。
            if (stats.count >= _sweep.sample_count) _finish_sweep_cell();
            return;
        }

        case SweepPhase::RESTORE_WRITE: {
            // 恢复仅执行一次原 gain/div 的 runtime apply，随后在本阶段内等待并回读确认。
            if (!_sweep.restore_started) {
                if (psoc->runtime_param_apply_in_progress()) return;
                bool discarded_result = false;
                (void)psoc->take_runtime_param_apply_result(&discarded_result);
                if (!psoc->start_runtime_param_apply(channel, _sweep.original_gain, _sweep.original_div)) {
                    if (_sweep.phase_ticks > SWEEP_WAIT_TICKS) {
                        _sweep_note_fail_phase();
                        _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
                        _sweep.terminal = SweepDataState::FAILED;
                        _sweep_enter(SweepPhase::TERMINAL);
                    }
                    return;
                }
                _sweep.restore_started = true;
                _sweep.phase_ticks = 0u;
                return;
            }

            bool ok = false;
            if (!psoc->take_runtime_param_apply_result(&ok)) {
                if (_sweep.phase_ticks <= SWEEP_RUNTIME_APPLY_TICKS) return;
                _sweep_note_fail_phase();
                _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
                _sweep.terminal = SweepDataState::FAILED;
                _sweep_enter(SweepPhase::TERMINAL);
                return;
            }
            uint32_t gain = 0u;
            uint32_t div = 0u;
            // 同 READBACK: 回读本身超时只说明链路正忙, 不能据此宣布"原参数没恢复"。留在本阶段
            // 重试到兜底期限, 期限内读回不符才是真的恢复失败(设备停在扫描参数上, 必须如实上报)。
            if (ok && (!psoc->get_param(channel, SWEEP_PARAM_ID_GAIN, &gain) ||
                       !psoc->get_param(channel, SWEEP_PARAM_ID_DIV, &div))) {
                if (_sweep.phase_ticks <= SWEEP_WAIT_TICKS) return;
                _sweep_note_fail_phase();
                _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
                _sweep.terminal = SweepDataState::FAILED;
                _sweep_enter(SweepPhase::TERMINAL);
                return;
            }
            if (!ok || gain != _sweep.original_gain || div != _sweep.original_div) {
                _sweep_note_fail_phase();
                _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
                _sweep.terminal = SweepDataState::FAILED;
            } else {
                CsdConfig* csd = CsdConfig::getInstance();
                csd->note_param(channel, SWEEP_PARAM_ID_GAIN, gain);
                csd->note_param(channel, SWEEP_PARAM_ID_DIV, div);
            }
            _sweep_enter(SweepPhase::TERMINAL);
            return;
        }

        case SweepPhase::TERMINAL:
            if (_sweep.restore_flags != 0u) _sweep.terminal = SweepDataState::FAILED;
            if (_sweep.phase_ticks <= SWEEP_TERMINAL_TICKS) {
                if (_sweep.stream_cursor < _sweep.produced ||
                    _sweep.resend_cursor < _sweep.resend_end) return;
                if (!_emit_sweep_frame(SWEEP_INDEX_NONE, _sweep.terminal, false)) return;
            }
            _finish_sweep_session();
            return;

        case SweepPhase::IDLE:
        default:
            return;
    }
}

void SensorLink::emit_autotune_task() {
    getInstance()->autotune_tick();
}

// 频率自适应阶段进度推送: core1 在阻塞完成自适应的同时发布 phase/step/试探分频,
// 本任务把它按 5Hz 组帧推给上位机, 使 20-25s 的长操作全程可见(而非上位机干等一个最终响应)。
void SensorLink::autotune_tick() {
    // 自续租: 本任务的存活由"设备侧操作未完成"决定, 不依赖上位机发命令(renew_all 会把租约压到 3s)。
    // 有硬上限, 异常时必然自灭。
    if (_at_ticks < AUTOTUNE_MAX_TICKS) {
        _at_ticks++;
        TxScheduler::getInstance()->renew(TX_TASK_AUTOTUNE, AUTOTUNE_LEASE_MS);
    }
    Psoc* psoc = Psoc::getInstance();
    psoc::AutoTuneProgress st = psoc->autotune_status();
    // core1 尚未取到本轮命令(队列排队中): 发布态仍是上一轮的结果 → 一律按"已受理/排队中"上报,
    // 否则上一轮的 done 会让本轮进度流刚开始就自取消。
    if (st.req != psoc->autotune_req()) {
        st.clear();
        st.state = 1;
        st.ch = _at_req_ch;
    }
    const bool done = (st.state == 2u);
    // 全通道终态必须回显请求哨兵 0xFF；进度帧才回显当前通道，不能让最后一次进度污染完成分支。
    const uint8_t report_ch = (done && _at_req_ch == kAllChannels) ? kAllChannels : st.ch;

    uint8_t* payload = _telem_frame.payload;
    uint16_t length = 0;
    payload[length++] = st.state;
    payload[length++] = st.phase;
    payload[length++] = st.step;
    payload[length++] = static_cast<uint8_t>(st.cur_div);
    payload[length++] = static_cast<uint8_t>(st.cur_div >> 8);
    payload[length++] = report_ch;
    payload[length++] = st.result;
    payload[length++] = static_cast<uint8_t>(st.div);
    payload[length++] = static_cast<uint8_t>(st.div >> 8);
    // ★尾部追加: 发起本轮的上位机请求 seq★ 前 9 字节布局一字未动(旧上位机按 >=9 解析, 兼容),
    // 新上位机据此把终态严格归属到自己发起的那一次请求。
    payload[length++] = _at_req_seq;

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::AUTO_TUNE_PROGRESS);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;

    bool sent = false;
    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length > 0) {
        HAL_USB_Device* usb = HAL_USB_Device::getInstance();
        if (usb->config_write_available() > 0) {   // 背压保护: 满则跳过本帧, 不阻塞不排队
            usb->config_write(_tx_buf, frame_length);
            sent = true;
        }
    }

    // 终态帧若被背压丢弃则不收尾, 下个周期重发: 否则上位机永远等不到完成帧(要靠 28s 超时兜)。
    if (!done || !sent) return;
    // 终态: 成功且找到分频 → 写穿 RP2040 真相源(供持久化与回读一致)。原同步 handler 里的这段
    // 必须搬到此处, 因为受理时刻还没有结果。
    if (st.result == 1u) {
        CsdConfig* cfg = CsdConfig::getInstance();
        if (_at_req_ch < SENSOR_LINK_CHANNELS && st.div != 0u) {
            cfg->note_param(_at_req_ch, 0x08u, st.div);   // 0x08 = PARAM_SNS_CLK_DIV
        } else if (_at_req_ch >= SENSOR_LINK_CHANNELS) {
            // ★逐通道自适应★: 各通道分频互不相同(终态 st.div 已复用为"成功通道数"), 故逐通道
            // 从 PSoC 回读真实 snsClk 写穿真相源, 否则重启后 download_to_psoc 会用旧值覆盖调好的结果。
            for (uint8_t ch = 0; ch < SENSOR_LINK_CHANNELS; ch++) {
                uint32_t v = 0;
                if (Psoc::getInstance()->get_param(ch, 0x08u, &v) && v != 0u) {
                    cfg->note_param(ch, 0x08u, v);
                }
            }
        }
        // 写穿后请求持久化: 否则调好的 snsClk 只活在 RAM, 重启后 download_to_psoc 用旧 blob 覆盖。
        // 实际 flash 写由主循环安全窗口(main.cpp: has_pending_save)执行, 本函数不阻塞。
        // 生效无需再 APPLY: PSoC 自适应内部已写 widgetContext 并 InitializeAllBaselines, 真相源写穿
        // 只为持久化/回读一致; 多余 APPLY 会触发整片重初始化+重校准(额外重扫, 白掉一次基线)。
        if (!_at_defer_save) {
            cfg->request_save();
        }
    }
    TxScheduler::getInstance()->cancel(TX_TASK_AUTOTUNE);   // 最终帧已发出 → 自取消
}

void SensorLink::emit_rescue_task() {
    getInstance()->rescue_tick();
}

// PSoC 救砖阶段进度推送: 与自适应同结构(自续租 + 终态自取消)。重刷期间主循环被 SWD 阻塞,
// 本函数由 SwdProgrammer 的保活钩子经 TxScheduler::tick 调用, 故擦写全程仍有帧发出。
void SensorLink::rescue_tick() {
    if (_rescue_ticks < RESCUE_MAX_TICKS) {
        _rescue_ticks++;
        TxScheduler::getInstance()->renew(TX_TASK_RESCUE, RESCUE_LEASE_MS);
    }
    PsocUpdater* updater = PsocUpdater::getInstance();
    const PsocBringupReport& rep = updater->report();
    const uint8_t state = updater->rescue_state();

    uint8_t* payload = _telem_frame.payload;
    uint16_t length = 0;
    payload[length++] = state;
    payload[length++] = updater->rescue_phase();
    payload[length++] = updater->rescue_result();
    payload[length++] = static_cast<uint8_t>(rep.last_stage);
    payload[length++] = static_cast<uint8_t>(rep.failure_stage);

    _telem_frame.cmd = static_cast<uint8_t>(HostCmd::PSOC_RESCUE_PROGRESS);
    _telem_frame.flags = HOST_CMD_FLAG_STREAM;
    _telem_frame.seq = _stream_seq++;
    _telem_frame.len = length;

    bool sent = false;
    const uint16_t frame_length = HostCmdCodec::encode_frame(_telem_frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length > 0) {
        HAL_USB_Device* usb = HAL_USB_Device::getInstance();
        if (usb->config_write_available() > 0) {   // 背压保护: 满则跳过本帧
            usb->config_write(_tx_buf, frame_length);
            sent = true;
        }
    }
    // 终态帧被背压丢弃则下周期重发, 否则上位机等不到完成帧。
    if (state == 2u && sent) TxScheduler::getInstance()->cancel(TX_TASK_RESCUE);
}

void SensorLink::_handle_psoc_rescue(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // PSOC_RESCUE(0x08): 空 payload。★立即回 ACK("已受理")★ —— 全片擦写数秒, 在 handler 里同步做
    // 会饿死 USB 导致掉线。仅置位请求, 由主循环 rescue_step 执行(其内部保活喂狗+泵 USB+推进度)。
    PsocUpdater* updater = PsocUpdater::getInstance();
    if (updater->rescue_active()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "psoc rescue already running", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    updater->rescue_request();
    // ★救援 = 显式把被隔离的算法重新放行★
    // 隔离(连续 3 次致命)之后 PSoC 跑的是原生 CapSense, 用户算法仍完好躺在 flash 里。用户点
    // "救援"的意思就是"我知道它之前搞死过, 再给它一次机会"(通常他刚在外部改过 ROM/cfg 或就是
    // 想复现)。clear_quarantine 同时会重新武装一次下发, 使解禁立刻生效而不是等下次复位。
    // 这也是"任何门禁都不许把上传通道永久锁死"的最后一道人工出口。
    PsocAlgo::getInstance()->clear_quarantine();
    SensorLink* self = getInstance();
    self->_rescue_ticks = 0;
    TxScheduler::getInstance()->schedule(TX_TASK_RESCUE, RESCUE_INTERVAL_US, RESCUE_LEASE_MS,
                                        &SensorLink::emit_rescue_task);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_telem_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 扫描期间广谱流被挂起并由会话负责复原; 放行 TELEM_START 会让它把保存态覆盖掉(结束后回不去)。
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(遥测流)", frame, response, response_length)) return;
    if (frame.len < 12) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "telem_start payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    SensorLink* self = getInstance();
    uint16_t position = 0;
    self->_mode = (frame.payload[position++] == 1) ? 1 : 0;
    self->_rate_hz = static_cast<uint16_t>(frame.payload[position]) |
                     (static_cast<uint16_t>(frame.payload[position + 1]) << 8);
    position += 2;
    self->_fields = frame.payload[position++];
    self->_ch_mask = 0;
    for (uint8_t byte = 0; byte < 8; byte++) {
        self->_ch_mask |= static_cast<uint64_t>(frame.payload[position + byte]) << (8u * byte);
    }

    if (self->_rate_hz < 1) self->_rate_hz = 1;
    if (self->_rate_hz > 1000) self->_rate_hz = 1000;
    self->_last_emit_us = 0;
    self->_stream.active = true;
    self->_stream.suspended = false;

    // 注册/续期遥测定时任务(续期制): 周期=1e6/rate。可选 payload[12..13]=lease_ms(u16), 缺省 3s。
    // 之后任意主机命令帧都会经 UsbComm renew_all 续租; 续期超时(上位机丢失)则任务自动停。
    uint32_t lease_ms = TELEM_LEASE_MS;
    if (frame.len >= 14) {
        const uint16_t l = static_cast<uint16_t>(frame.payload[12]) |
                           (static_cast<uint16_t>(frame.payload[13]) << 8);
        if (l != 0) lease_ms = l;
    }
    self->_lease_ms = lease_ms;   // 记住协商值: 租约超时挂起后自动恢复要复用同一租约
    if (self->_focus.active) {
        self->_pause_broad_for_focus();
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    Psoc::getInstance()->set_telemetry_active(true);   // Phase C：开启全通道 raw 快照慢路
    const uint32_t interval_us = 1000000UL / self->_rate_hz;
    TxScheduler::getInstance()->schedule(TX_TASK_TELEM, interval_us, lease_ms,
                                         &SensorLink::emit_telem_task);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_telem_stop(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    getInstance()->stop();   // 显式停流同时清理 FocusSession 与已保存的广谱状态，之后不自动恢复
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_focus_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (getInstance()->_sweep.active()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "sweep session is active", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (frame.len != 6u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "focus_start requires ch, fields, rate and lease", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    const uint8_t channel = frame.payload[0];
    if (channel >= SENSOR_LINK_CHANNELS) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "focus_start channel out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    uint16_t rate_hz = static_cast<uint16_t>(frame.payload[2]) |
                       (static_cast<uint16_t>(frame.payload[3]) << 8);
    uint16_t lease_ms = static_cast<uint16_t>(frame.payload[4]) |
                        (static_cast<uint16_t>(frame.payload[5]) << 8);
    if (rate_hz < 1u) rate_hz = 1u;
    if (rate_hz > 1000u) rate_hz = 1000u;
    if (lease_ms == 0u) lease_ms = static_cast<uint16_t>(TELEM_LEASE_MS);

    SensorLink* self = getInstance();
    const bool replacing = self->_focus.active;
    // Do not invoke device-side FOCUS_SCAN here.  On the deployed PSoC image it
    // can accept the command while pausing scan_count indefinitely; focus is
    // implemented by the RP snapshot fast path below instead.
    TxScheduler::getInstance()->cancel(TX_TASK_FOCUS);
    if (!replacing) self->_pause_broad_for_focus();

    uint16_t session = static_cast<uint16_t>(self->_focus_session_counter + 1u);
    if (session == 0u) session = 1u;
    self->_focus_session_counter = session;
    self->_focus.clear();
    self->_focus.active = true;
    self->_focus.id = session;
    self->_focus.rate_hz = rate_hz;
    self->_focus.lease_ms = lease_ms;
    self->_focus.channel = channel;
    self->_focus.fields = frame.payload[1];
    self->_focus.last_generation = Psoc::getInstance()->snapshot_generation();
    self->_focus.last_focus_scan_ack_us = time_us_32();
    self->_focus.generation_fence = 1u;
    Psoc::getInstance()->set_telemetry_active(true);
    // Focus keeps the PSoC continuous scan loop and narrows only the RP snapshot/output path.
    Psoc::getInstance()->set_focus_channel(channel);
    TxScheduler::getInstance()->schedule(TX_TASK_FOCUS, 1000000UL / rate_hz, lease_ms,
                                         &SensorLink::emit_focus_task,
                                         &SensorLink::focus_lease_expired_task);

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::FOCUS_START);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = static_cast<uint8_t>(session);
    resp.payload[1] = static_cast<uint8_t>(session >> 8);
    resp.payload[2] = static_cast<uint8_t>(rate_hz);
    resp.payload[3] = static_cast<uint8_t>(rate_hz >> 8);
    resp.len = 4;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_focus_stop(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "focus_stop requires session", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    const uint16_t session = static_cast<uint16_t>(frame.payload[0]) |
                             (static_cast<uint16_t>(frame.payload[1]) << 8);
    SensorLink* self = getInstance();
    if (!self->_focus.active || session != self->_focus.id) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "focus_stop session is not current", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    TxScheduler::getInstance()->cancel(TX_TASK_FOCUS);
    self->_focus.clear();
    self->_release_focus_scan();
    self->_restore_broad_after_focus();
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

// SWEEP_START(0x36): payload = [ch, settle_samples, sample_count] → 响应 [session u16, total u16]。
// 受理即回, 448 格由 TX_TASK_SWEEP 单步推进(见 sweep_tick)。
void SensorLink::_handle_sweep_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    SensorLink* self = getInstance();
    if (self->_sweep.active()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "sweep session already running", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (frame.len != 3u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "sweep_start requires ch, settle_samples and sample_count", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t channel = frame.payload[0];
    if (channel >= SENSOR_LINK_CHANNELS) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "sweep_start channel out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法扫描; 请先启用该通道",
                           channel, frame, response, response_length)) return;
    // 扫描会话独占目标通道的 gain/div；已有普通 heavy 操作在途时如实回绝而不排队。
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(增益/分频扫描)",
                          frame, response, response_length)) return;
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->link_alive()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC link is down", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // ★先拿到原值再动手★: 取不到就绝不开扫 —— 否则会话结束时无从写回, 该通道永久停在扫描参数上。
    // ★但单次读失败不等于读不到★: get_param 走"core0 入队 → core1 独占 SPI 执行"的信箱, 在遥测/
    // Focus 正在推流时命令环与 SPI 都更忙, 偶发失败是**已知现象** —— 逐格 READBACK 那里早就按
    // "留在本阶段兜底重试"处理了(见本文件 READBACK 分支注释: 设备完全健康时首格也会读失败)。
    // 这里却一直是单次尝试、失败即 NAK, 于是无头路径能开扫、UI 推流中点"频谱扫描"却被拒
    // (实测报错 `PSoC param readback failed (sweep needs original gain/div)`)。同一课要学完。
    // 重试上限取 3: 每次失败最多已消耗一个 100ms 信箱预算, 再多就该如实告诉用户链路真的不通。
    uint32_t original_gain = 0, original_div = 0;
    bool gain_ok = false, div_ok = false;
    for (uint8_t attempt = 0; attempt < 3u && !(gain_ok && div_ok); attempt++) {
        if (!gain_ok) gain_ok = psoc->get_param(channel, SWEEP_PARAM_ID_GAIN, &original_gain);
        if (!div_ok)  div_ok  = psoc->get_param(channel, SWEEP_PARAM_ID_DIV, &original_div);
    }
    if (!(gain_ok && div_ok)) {
        // 分别报出哪一项没读到: "两个都没读到"与"只有分频没读到"指向的排查方向不同。
        const char* detail = gain_ok ? "PSoC param readback failed after 3 tries (sns_clk_div)"
                           : (div_ok ? "PSoC param readback failed after 3 tries (idac_gain)"
                                     : "PSoC param readback failed after 3 tries (idac_gain + sns_clk_div)");
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            detail, response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    uint8_t settle_samples = frame.payload[1];
    uint8_t sample_count = frame.payload[2];
    if (settle_samples > SWEEP_SETTLE_MAX) settle_samples = SWEEP_SETTLE_MAX;
    if (sample_count == 0u) sample_count = 1u;
    if (sample_count > SWEEP_SAMPLE_MAX) sample_count = SWEEP_SAMPLE_MAX;

    // 与 Focus 互斥: Focus 在跑就挂起它(广谱保存态由 _saved_stream 继续持有, 不重复保存);
    // 否则按 Focus 那条同一路径挂起广谱流。两种情形都在会话收尾时经 _finish_sweep_session 复原。
    // Sweep must never inherit a PSoC Focus lease, even when no FocusSession is locally active.
    self->_release_focus_scan();
    if (self->_focus.active) {
        self->_focus.suspended = true;
        TxScheduler::getInstance()->cancel(TX_TASK_FOCUS);
    } else {
        self->_pause_broad_for_focus();
    }

    uint16_t session = static_cast<uint16_t>(self->_sweep_session_counter + 1u);
    if (session == 0u) session = 1u;
    self->_sweep_session_counter = session;
    self->_sweep.clear();
    for (uint16_t i = 0; i < SWEEP_TOTAL; ++i) {
        self->_sweep_results[i].clear();
    }
    self->_sweep.id = session;
    self->_sweep.channel = channel;
    self->_sweep.settle_samples = settle_samples;
    self->_sweep.sample_count = sample_count;
    self->_sweep.original_gain = static_cast<uint8_t>(original_gain);
    self->_sweep.original_div = static_cast<uint8_t>(original_div);
    self->_sweep.output_restore = SweepOutputRestore::BROAD;
    self->_sweep_enter(SweepPhase::SET_CELL);
    psoc->set_telemetry_active(true);   // 采样取自快照慢路, 会话期间必须开着
    psoc->set_focus_channel(channel);   // Sweep 本质是单通道 ⇒ 武装快照快路, 获得 ~3× 采样率
    TxScheduler::getInstance()->schedule(TX_TASK_SWEEP, SWEEP_INTERVAL_US, SWEEP_LEASE_MS,
                                         &SensorLink::emit_sweep_task,
                                         &SensorLink::sweep_lease_expired_task);

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::SWEEP_START);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = static_cast<uint8_t>(session);
    resp.payload[1] = static_cast<uint8_t>(session >> 8);
    resp.payload[2] = static_cast<uint8_t>(SWEEP_TOTAL);
    resp.payload[3] = static_cast<uint8_t>(SWEEP_TOTAL >> 8);
    resp.len = 4;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

// SWEEP_CTRL(0x37): payload = [op(0=取消 / 1=补发 / 2=续租), session u16, first u16, count u8]。
    // 取消只是“请求恢复”(仍要走完原 gain/div runtime apply + 回读才报终态)；补发覆盖已判定前缀，
    // 其中未采样空洞也会以 samples=0 如实发送。
void SensorLink::_handle_sweep_ctrl(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len < 3u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "sweep_ctrl requires op and session", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    SensorLink* self = getInstance();
    const uint8_t op = frame.payload[0];
    const uint16_t session = static_cast<uint16_t>(frame.payload[1]) |
                             (static_cast<uint16_t>(frame.payload[2]) << 8);
    if (!self->_sweep.active() || session != self->_sweep.id) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "sweep_ctrl session is not current", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    if (op == 0u) {
        self->_begin_sweep_restore(SweepDataState::CANCELLED);   // 已在恢复中则幂等
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (op == 2u) {
        // 显式续租仅刷新当前会话的 Sweep task，不改变阶段/结果游标，也不把“收到任意命令”误当续租。
        TxScheduler::getInstance()->renew(TX_TASK_SWEEP, SWEEP_LEASE_MS);
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (op != 1u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "sweep_ctrl op must be 0(cancel), 1(resend) or 2(keepalive)", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (frame.len < 6u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "sweep_ctrl resend requires first and count", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint16_t first = static_cast<uint16_t>(frame.payload[3]) |
                           (static_cast<uint16_t>(frame.payload[4]) << 8);
    const uint8_t count = frame.payload[5];
        if (count == 0u || first >= SWEEP_TOTAL || first >= self->_sweep.produced) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "sweep_ctrl resend range not cached yet", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    uint32_t end = static_cast<uint32_t>(first) + count;
    if (end > self->_sweep.produced) end = self->_sweep.produced;
    self->_sweep.resend_cursor = first;
    self->_sweep.resend_end = static_cast<uint16_t>(end);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_param_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) + param_id(u8) + value(u32 LE)
    if (frame.len < 6) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_set payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t param_id = frame.payload[1];
    const uint32_t value = static_cast<uint32_t>(frame.payload[2]) |
                           (static_cast<uint32_t>(frame.payload[3]) << 8) |
                           (static_cast<uint32_t>(frame.payload[4]) << 16) |
                           (static_cast<uint32_t>(frame.payload[5]) << 24);

    // ★三处同源★ 本围栏与以下两处必须逐位等价, 任何一处改动必须三处同改:
    //   - PSoC:   psoc_firmware/CY8C4147AZI-SensorCore/main.c::_param_value_legal
    //   - 上位机: control_software/src/proto/telemetry.rs::param_fence(上位机侧唯一权威声明表,
    //             UI 输入范围 / 下发前拒绝 / 回读防污染都只读它)
    // 合法性防护(与 PSoC 端一致): 拒绝会 railed/时钟异常/校准发散的非法值, 不下发也不写真相源。
    // RESOLUTION 6..16; SNS_CLK_DIV 1..255; IDAC_MOD 0..127; IDAC_GAIN 0..6; SNS_CLK_SOURCE 低7位 0..6;
    // ENABLED(0x0C) 0..1。
    bool legal = true;
    switch (param_id) {
        case 0x07: legal = (value >= 6u)  && (value <= 16u);  break;   // RESOLUTION
        case 0x08: legal = (value >= 1u)  && (value <= 255u); break;   // SNS_CLK_DIV
        case 0x09: legal = (value <= 127u);                   break;   // IDAC_MOD
        case 0x0B: legal = (value <= 6u);                     break;   // IDAC_GAIN(0..6, 表7项索引7越界崩溃)
        case 0x0A: legal = ((value & 0x7Fu) <= 6u);           break;   // SNS_CLK_SOURCE
        case 0x0C: legal = (value <= 1u);                     break;   // ENABLED(硬件开关, 只 0/1)
        default: break;
    }
    if (!legal) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param value out of legal range (guarded)", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(调参)", frame, response, response_length)) return;

    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::PARAM, frame, ch, param_id, value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // Delayed terminal: ACK only after core1/PSoC has executed the command and CsdConfig was written through.
    *response_length = 0u;
}

void SensorLink::_handle_param_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) + param_id(u8) → 响应 channel + param_id + value(u32 LE)
    if (frame.len < 2) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_get payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t param_id = frame.payload[1];
    if (sync_read_gate_reject("PSoC command queue busy", frame, response, response_length)) return;
    uint32_t value = 0;
    if (!param_read(ch, param_id, &value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC param_get failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::PARAM_GET);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t position = 0;
    resp.payload[position++] = ch;
    resp.payload[position++] = param_id;
    resp.payload[position++] = static_cast<uint8_t>(value);
    resp.payload[position++] = static_cast<uint8_t>(value >> 8);
    resp.payload[position++] = static_cast<uint8_t>(value >> 16);
    resp.payload[position++] = static_cast<uint8_t>(value >> 24);
    resp.len = position;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_param_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) → 响应 channel + count + [param_id + value(u32 LE)]×count
    //           + missing_count + missing_ids；尾随项只表示此刻取不到，旧上位机按 count 截止仍兼容。
    // payload = 0xFF + param_id(u8) → "全通道单参数"变体: 响应 0xFF + param_id + count
    //           + [ch + value(u32 LE)]×count + missing_count + missing_chs。
    //   36 通道单参数一次取回替代 36 条单发 PARAM_GET；最大 3+36×5+1+36=220B，远小于 4096B payload。
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_get_all payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t ch = frame.payload[0];
    if (sync_read_gate_reject("PSoC command queue busy", frame, response, response_length)) return;
    if (ch == kAllChannels) {
        if (frame.len < 2) {
            *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "param_get_all(all channels) needs param_id", response, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        _emit_param_all_channels(frame, response, response_length);
        return;
    }

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::PARAM_GET_ALL);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t position = 0;
    resp.payload[position++] = ch;
    const uint16_t count_position = position++;

    uint8_t count = 0;
    uint8_t missing_ids[kParamCount] = {};
    uint8_t missing_count = 0;
    for (uint8_t i = 0; i < kParamCount; i++) {
        uint32_t value = 0;
        if (!param_read_retry(ch, kParamIds[i], &value)) {
            missing_ids[missing_count++] = kParamIds[i];
            continue;
        }
        resp.payload[position++] = kParamIds[i];
        resp.payload[position++] = static_cast<uint8_t>(value);
        resp.payload[position++] = static_cast<uint8_t>(value >> 8);
        resp.payload[position++] = static_cast<uint8_t>(value >> 16);
        resp.payload[position++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[count_position] = count;
    // 尾随项如实说明“此刻取不到”，不是“设备没有”：上位机可保留旧值，不能显示为 — 或 0。
    resp.payload[position++] = missing_count;
    for (uint8_t i = 0; i < missing_count; i++) resp.payload[position++] = missing_ids[i];
    if (missing_count != 0u) {
        char log[96];
        int log_length = std::snprintf(log, sizeof(log),
            "PARAM_GET_ALL missing: ch=%u param_ids=", static_cast<unsigned>(ch));
        for (uint8_t i = 0; i < missing_count && log_length > 0 &&
                            static_cast<size_t>(log_length) < sizeof(log); i++) {
            log_length += std::snprintf(log + log_length, sizeof(log) - static_cast<size_t>(log_length),
                                        "%s0x%02X", i == 0u ? "" : ",", missing_ids[i]);
        }
        std::printf("%s\n", log);  // 每次响应最多一行，避免 12 项失败时刷屏。
    }
    resp.len = position;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

// "全通道单参数"批量回读: 响应 0xFF + param_id + count + [ch + value(u32 LE)]×count
// + missing_count + missing_chs。读不到是“此刻取不到”，不是设备没有；旧上位机仍只按 count 解析。
void SensorLink::_emit_param_all_channels(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    const uint8_t param_id = frame.payload[1];
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::PARAM_GET_ALL);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t position = 0;
    resp.payload[position++] = kAllChannels;
    resp.payload[position++] = param_id;
    const uint16_t count_position = position++;

    uint8_t count = 0;
    uint8_t missing_chs[SENSOR_LINK_CHANNELS] = {};
    uint8_t missing_count = 0;
    for (uint8_t ch = 0; ch < SENSOR_LINK_CHANNELS; ch++) {
        uint32_t value = 0;
        if (!param_read_retry(ch, param_id, &value)) {
            missing_chs[missing_count++] = ch;
            continue;
        }
        resp.payload[position++] = ch;
        resp.payload[position++] = static_cast<uint8_t>(value);
        resp.payload[position++] = static_cast<uint8_t>(value >> 8);
        resp.payload[position++] = static_cast<uint8_t>(value >> 16);
        resp.payload[position++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[count_position] = count;
    // 尾随项如实说明“此刻取不到”，不是“设备没有”：上位机可保留旧值，不能显示为 — 或 0。
    resp.payload[position++] = missing_count;
    for (uint8_t i = 0; i < missing_count; i++) resp.payload[position++] = missing_chs[i];
    if (missing_count != 0u) {
        char log[192];
        int log_length = std::snprintf(log, sizeof(log),
            "PARAM_GET_ALL missing: param_id=0x%02X chs=", param_id);
        for (uint8_t i = 0; i < missing_count && log_length > 0 &&
                            static_cast<size_t>(log_length) < sizeof(log); i++) {
            log_length += std::snprintf(log + log_length, sizeof(log) - static_cast<size_t>(log_length),
                                        "%s%u", i == 0u ? "" : ",",
                                        static_cast<unsigned>(missing_chs[i]));
        }
        std::printf("%s\n", log);  // 每次响应最多一行，避免 36 通道失败时刷屏。
    }
    resp.len = position;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_calibrate(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = ch_mask(u64 LE)。★单通道语义端到端透传★: 掩码恰好只有一位 ⇒ 把该通道号透传给
    // PSoC(它只校准该 widget 并只初始化该 widget 的基线); 多位/空/全 36 位 ⇒ 0xFF 全通道兼容语义。
    // 上位机的"全通道校准"已改为 host 侧可取消串行队列逐通道下发, 不再依赖固件内部 36 通道循环。
    const uint8_t cal_ch = _mask_to_single_ch(frame);
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法校准; 请先启用该通道",
                           cal_ch, frame, response, response_length)) return;
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(校准)", frame, response, response_length)) return;
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(校准)", frame, response, response_length)) return;
    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::CALIBRATE, frame, cal_ch, 0u, 0u)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;
}

void SensorLink::_handle_baseline_reset(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = ch_mask(u64 LE); 单位掩码 ⇒ 只复位该通道基线(见 _mask_to_single_ch)。
    const uint8_t bsln_ch = _mask_to_single_ch(frame);
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法复位基线; 请先启用该通道",
                           bsln_ch, frame, response, response_length)) return;
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(基线复位)", frame, response, response_length)) return;
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(基线复位)", frame, response, response_length)) return;
    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::BASELINE_RESET, frame, bsln_ch, 0u, 0u)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;
}

void SensorLink::_handle_auto_tune(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), pref(u8, 可选)]: ch 0..35=仅该通道下探, 0xFF=全 36 通道(旧行为);
    // 缺省(空 payload)=全通道; pref=灵敏度档位 1..7(缺省/越界→4), 越高=落档时往低频多让分频。
    // ★异步化★: 自适应最坏 20-25s, 端到端阻塞会让上位机干等且无从判断进度/异常。改为
    // 入队即回 ACK("已受理"), 阶段进度与最终结果(含 result/div)经 AUTO_TUNE_PROGRESS(0x2E)
    // 推送流上报, 完成帧发出后任务自取消。真相源写穿(note_param)随之搬到完成时刻。
    const uint8_t req_ch = (frame.len >= 1) ? frame.payload[0] : 0xFFu;
    uint8_t req_pref = (frame.len >= 2) ? frame.payload[1] : 4u;
    const bool defer_save = (frame.len >= 3) && (frame.payload[2] != 0u);
    if (req_pref < 1u || req_pref > 7u) req_pref = 4u;
    if (req_ch >= 36u && req_ch != 0xFFu) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "auto_tune channel out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法做频率自适应; 请先启用该通道",
                           req_ch, frame, response, response_length)) return;
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(频率自适应)", frame, response, response_length)) return;
    // ★首要嫌疑就是这条的堆叠★: 自适应单次 20s+, 期间再来一条只会背靠背排队, 把主循环整体拖长。
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(频率自适应)", frame, response, response_length)) return;
    SensorLink* self = getInstance();
    self->_at_req_ch = req_ch;
    // 归属键: 本轮进度/终态帧一律回显这个 seq(上位机据此丢弃上一轮的残留帧), 并折成 6 bit 标签
    // 随命令下到 PSoC, 使 RP↔PSoC 这一段也能认出陈旧结果。
    self->_at_req_seq = frame.seq;
    self->_at_defer_save = defer_save;
    self->_at_ticks = 0;
    if (!Psoc::getInstance()->auto_tune_start(req_ch, req_pref, frame.seq)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC auto_tune enqueue failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    TxScheduler::getInstance()->schedule(TX_TASK_AUTOTUNE, AUTOTUNE_INTERVAL_US, AUTOTUNE_LEASE_MS,
                                         &SensorLink::emit_autotune_task);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_mode_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = mode(u8)：0=自动校准/标准完整处理，非0=半自动手动
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "mode_set payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::MODE, frame, frame.payload[0], 0u, 0u)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;
}

void SensorLink::_handle_csd_capture(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // AUTO 的实时参数由 CapSense 自动计算，回读固化会覆盖用户保存的半自动手动参数。
    CsdConfig* csd = CsdConfig::getInstance();
    if (sync_read_gate_reject("PSoC command queue busy", frame, response, response_length)) return;
    if (csd->mode() != CSD_MODE_SEMI) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::CONFIG_ERROR,
            "自动模式由 PSoC 接管，禁止捕获以保护手动参数", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (!csd->capture_from_psoc(Psoc::getInstance())) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC 参数读取失败", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_cp_measure(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 0) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "cp_measure payload must be empty", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(Cp 测量)", frame, response, response_length)) return;
    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::CP_MEASURE, frame, 0u, 0u, 0u)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;
}

void SensorLink::_handle_cp_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 1 || frame.payload[0] >= SENSOR_LINK_CHANNELS) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "cp_get requires channel 0..35", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    const uint8_t ch = frame.payload[0];
    if (sync_read_gate_reject("PSoC command queue busy", frame, response, response_length)) return;
    uint32_t cp_ff = 0;
    // ★不再 NAK★: PSoC 未测量/测量失败本就以哨兵 0xFFFFFF 表达; 读取(SPI 忙/超时)失败时也回同一
    // 哨兵的正常响应, 使上位机显示"未测量/测量失败"而不是刷 NAK 日志(实测 NAK 每秒 8~12 条刷屏)。
    // NAK 只保留给非法通道(上面已处理)。
    if (!Psoc::getInstance()->get_cp(ch, &cp_ff)) {
        cp_ff = CP_UNMEASURED_FF;
    }
    // 禁用通道读回哨兵 ⇒ 改报"未测量"。只在读回值**就是**哨兵时改写: 通道若曾在启用状态下被测过、
    // 之后才被禁用, PSoC 那份 cp_value 会保留上次的真实读数, 那是真实数据, 不该抹成"未测量"。
    if (cp_ff == CP_UNMEASURED_FF && !CsdConfig::getInstance()->ch_enabled(ch)) {
        cp_ff = CP_DISABLED_FF;
    }

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::CP_GET);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = ch;
    resp.payload[1] = static_cast<uint8_t>(cp_ff);
    resp.payload[2] = static_cast<uint8_t>(cp_ff >> 8);
    resp.payload[3] = static_cast<uint8_t>(cp_ff >> 16);
    resp.payload[4] = static_cast<uint8_t>(cp_ff >> 24);
    resp.len = 5;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_unsupported(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::NOT_IMPLEMENTED,
        "PSoC mutation is not implemented", response, HOST_CMD_RESP_BUF_MAX);
}

// ---- 全局 CSD 配置命令 ----
namespace {
constexpr uint8_t kGlobalIds[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };  // ...IDAC_SENSE_CONFIG/AUTO_CALIBRATE_EN
constexpr uint8_t kGlobalCount = sizeof(kGlobalIds) / sizeof(kGlobalIds[0]);
}  // namespace

void SensorLink::_handle_global_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = gparam_id(u8) → 响应 [gparam_id, value(u32 LE)]
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global_get payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t gid = frame.payload[0];
    if (sync_read_gate_reject("PSoC command queue busy", frame, response, response_length)) return;
    uint32_t value = 0;
    if (!Psoc::getInstance()->get_global(gid, &value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC global_get failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::GLOBAL_GET);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    resp.payload[p++] = gid;
    resp.payload[p++] = static_cast<uint8_t>(value);
    resp.payload[p++] = static_cast<uint8_t>(value >> 8);
    resp.payload[p++] = static_cast<uint8_t>(value >> 16);
    resp.payload[p++] = static_cast<uint8_t>(value >> 24);
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_global_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = gparam_id(u8) + value(u32 LE)。写 PSoC RAM 影子 + 写穿 CsdConfig + APPLY 重初始化生效。
    if (frame.len < 5) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global_set payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t gid = frame.payload[0];
    const uint32_t value = static_cast<uint32_t>(frame.payload[1]) |
                           (static_cast<uint32_t>(frame.payload[2]) << 8) |
                           (static_cast<uint32_t>(frame.payload[3]) << 16) |
                           (static_cast<uint32_t>(frame.payload[4]) << 24);
    // 合法性防护(与 PSoC 端一致): INACTIVE_SNS∈{1,2,4}; IDAC_GAIN_INIT 0..6; IDAC_MIN 0..127;
    // RAW_TARGET 1..99(0/≥100 会让自动校准发散→railed); MFS 分频 0..255(PSoC 侧字段是 uint8_t)。
    bool glegal = true;
    switch (gid) {
        case 0x01: glegal = (value == 1u) || (value == 2u) || (value == 4u); break; // INACTIVE_SNS
        case 0x02: glegal = (value <= 6u);                                   break; // IDAC_GAIN_INIT(0..6, 索引7越界崩溃)
        case 0x03: glegal = (value <= 127u);                                 break; // IDAC_MIN
        case 0x04: glegal = (value >= 1u) && (value <= 99u);                 break; // RAW_TARGET
        // ★补齐原先缺失的围栏★: MFS 偏移落在 PSoC 的 uint8_t 字段, 之前两端都不查, 写 300 会被
        // 静默截断成 44 且回显送回请求值, 上位机毫无察觉。现在与其它项同口径, 超范围直接 NAK。
        case 0x05:
        case 0x06: glegal = (value <= 255u);                                 break; // MFS_DIV_F1/F2
        case 0x07: glegal = (value <= 1u);                                   break; // IDAC_SENSE_CONFIG(0=sourcing,1=sinking)
        case 0x08: glegal = (value <= 1u);                                   break; // AUTO_CALIBRATE_EN(0/1)
        default: break;
    }
    if (!glegal) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global value out of legal range (guarded)", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::GLOBAL, frame, gid, 0u, value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // GLOBAL_SET only mutates the PSoC shadow. The following GLOBAL_COMMIT carries the one-time reinitialization.
    *response_length = 0u;
}

// 批量全局项下发完毕后, 单次触发 PSoC 完整重初始化(合并, 防反复重校准漂移/风暴)。
void SensorLink::_handle_global_commit(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 全局提交会整片重初始化 + 重校准, 与逐格扫描直接冲突(会把已扫的参数与基线一并推翻)。
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(全局提交)", frame, response, response_length)) return;
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(全局提交)", frame, response, response_length)) return;
    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::GLOBAL_COMMIT, frame, 0u, 0u, 0u)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC global_commit busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;
}

void SensorLink::_handle_global_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空 → [count(u8), (gparam_id, value u32 LE)×count]
    // This is a synchronous multi-read.  Refuse while the core1 FIFO is owned
    // by provisioning, a delayed host write, or a heavy operation instead of
    // holding UsbComm inside eight blocking reads and turning a busy PSoC into
    // a silent six-second host timeout.
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->core1_idle() || psoc->heavy_busy()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC command queue busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::GLOBAL_GET_ALL);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 1;   // count 占位
    uint8_t count = 0;
    for (uint8_t i = 0; i < kGlobalCount; i++) {
        uint32_t value = 0;
        if (!psoc->get_global(kGlobalIds[i], &value)) continue;
        resp.payload[p++] = kGlobalIds[i];
        resp.payload[p++] = static_cast<uint8_t>(value);
        resp.payload[p++] = static_cast<uint8_t>(value >> 8);
        resp.payload[p++] = static_cast<uint8_t>(value >> 16);
        resp.payload[p++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[0] = count;
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

// ---- JIT 算法引擎命令 ----
void SensorLink::_handle_algo_get_info(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → 响应布局(前 6 字节的语义与偏移与旧固件**逐字节相同**, 只在尾部追加):
    //   [0]      is_default
    //   [1]      psoc_valid
    //   [2..3]   store_len    (u16 LE) RP 存储的算法长度
    //   [4..5]   store_crc16  (u16 LE) RP 存储的算法 CRC16
    //   [6..7]   psoc_len     (u16 LE) ★PSoC 槽内实际长度
    //   [8..9]   psoc_crc16   (u16 LE) ★PSoC 槽内实际内容 CRC16(PSoC commit 时算出)
    //   [10..11] heap_used    (u16 LE) ★算法共享堆峰值占用
    //   [12..13] heap_size    (u16 LE) ★算法共享堆容量
    //   [14]     flags: bit0=quarantined bit1=download_pending bit2=uploading bit3=psoc_cache_unavailable
    //   [15..16] slot_capacity (u16 LE): PSoC 自报可执行槽容量；读不到回 RP 上限作兜底
    //   [17..18] upload_limit  (u16 LE): 单帧算法代码上限 = HOST_CMD_PAYLOAD_MAX - 4
    //   [19..22] src_capacity  (u32 LE): C 源存储容量
    //   [23..24] src_chunk     (u16 LE): C 源分片粒度
    //   [25]     caps_flags: bit0=slot_capacity 来自 PSoC；bit1=PSoC 与 RP 槽容量不一致
    //   [26..27] abort_page   (u16 LE): 最近一次下发中止在第几页(0xFFFF=BEGIN 阶段)
    //   [28]     abort_reason (u8): 1=BEGIN状态泄漏 2=BEGIN未受理 3=页回显不符 4=END失败
    //                               5=commit超时 6=内容CRC不符 7=链路失败 (见 PsocLink::AlgoAbort)
    //   [29..30] abort_count  (u16 LE): 累计中止次数
    // ★上位机判"上传成功"只能用 psoc_len + psoc_crc16★:
    //   · store_* 只证明"RP 把字节存下了", 与 PSoC 槽里装的是什么毫无关系;
    //   · psoc_valid 也不够 —— 它分不出"新算法装上了"和"旧算法还在、长度恰好相同"(同一份源改
    //     一个常量重编译, 长度几乎必然不变), 这正是"上传成功但行为没变"这类报告的来源。
    //   判据: psoc_len == store_len && psoc_crc16 == store_crc16 && !uploading。
    PsocAlgo* store = PsocAlgo::getInstance();
    // REST_DEFAULT 的 ACK 仅表示异步请求已受理；先用 RP 侧权威默认元数据回包，避免同一 USB
    // 批次中紧随其后的 GET_INFO 抢在 core1 完成默认 blob 下发前读到旧 PSoC cache。
    // 非 pending 的普通查询仍回报 PSoC cache，保持设备真值可见。
    const bool reset_pending = store->reset_default_pending();
    bool psoc_valid = false;
    uint16_t psoc_len = 0u;
    uint16_t psoc_crc16 = 0u;
    uint16_t heap_used = 0u;
    uint16_t heap_size = 0u;
    bool uploading = false;
    bool cache_ok = false;
    const uint16_t reported_len = reset_pending
        ? static_cast<uint16_t>(PSOC_ALGO_DEFAULT_LEN) : store->len();
    const uint16_t reported_crc16 = reset_pending
        ? static_cast<uint16_t>(PSOC_ALGO_DEFAULT_CRC16) : store->crc16();
    // RESET_DEFAULT ACK 表示恢复已被异步所有者受理；同一 USB 批次里的即时查询据此返回
    // 目标默认元数据。pending 只会在 core1 cache 已确认 valid+540 后清除，普通查询始终报告真值。
    // ★缓存的 psoc_* 与 flags 无条件取一次★: 即使处于 reset_pending, 上位机也需要看到 PSoC 侧
    // 真值(否则"恢复默认还没落地"这段时间里界面上完全看不出 PSoC 里现在装的是什么)。
    cache_ok = Psoc::getInstance()->get_algo_info_cached(&psoc_valid, &psoc_len, &psoc_crc16,
                                                        &heap_used, &heap_size, &uploading);
    // 容量独立于 INFO 新鲜度：首次由 PSoC 自报成功后即可复用，读不到才回退 RP 编译期上限。
    uint16_t slot_capacity = static_cast<uint16_t>(PSOC_ALGO_MAX_LEN);
    const bool slot_capacity_from_psoc =
        Psoc::getInstance()->get_algo_caps_cached(&slot_capacity, nullptr);
    if (reset_pending) {
        // 恢复默认已被异步所有者受理: psoc_valid 先按目标态报, 避免同一 USB 批次里的即时查询
        // 读到旧自定义信息(下发仍由 tick() 异步完成)。psoc_len/psoc_crc16 保持缓存真值不篡改。
        psoc_valid = true;
    }

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_INFO);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    resp.payload[p++] = reset_pending || store->is_default() ? 1u : 0u;
    resp.payload[p++] = psoc_valid ? 1u : 0u;
    resp.payload[p++] = static_cast<uint8_t>(reported_len);
    resp.payload[p++] = static_cast<uint8_t>(reported_len >> 8);
    resp.payload[p++] = static_cast<uint8_t>(reported_crc16);
    resp.payload[p++] = static_cast<uint8_t>(reported_crc16 >> 8);
    resp.payload[p++] = static_cast<uint8_t>(psoc_len);
    resp.payload[p++] = static_cast<uint8_t>(psoc_len >> 8);
    resp.payload[p++] = static_cast<uint8_t>(psoc_crc16);
    resp.payload[p++] = static_cast<uint8_t>(psoc_crc16 >> 8);
    resp.payload[p++] = static_cast<uint8_t>(heap_used);
    resp.payload[p++] = static_cast<uint8_t>(heap_used >> 8);
    resp.payload[p++] = static_cast<uint8_t>(heap_size);
    resp.payload[p++] = static_cast<uint8_t>(heap_size >> 8);
    // flags: 隔离/推迟是 RP 侧的策略态, uploading/cache 不可用是 PSoC 侧的采集态。
    // 没有这一字节, "算法存着但设备就是不跑它"在界面上完全无法解释(只能看成设备坏了)。
    uint8_t flags = 0u;
    if (store->quarantined()) flags |= 0x01u;
    if (store->download_pending()) flags |= 0x02u;
    if (uploading) flags |= 0x04u;
    if (!cache_ok) flags |= 0x08u;   // psoc_len/psoc_crc16/heap_* 本次不可信(链路抖动或缓存过期)
    resp.payload[p++] = flags;
    // ★槽上限与单帧上限必须分开上报★ 槽可装 4096B，但 ALGO_UPLOAD 的 len/crc 头占 4B，
    // 混成同一个数会让 4093..4096B 的算法在上传门口被拒却无从解释。
    resp.payload[p++] = static_cast<uint8_t>(slot_capacity);
    resp.payload[p++] = static_cast<uint8_t>(slot_capacity >> 8);
    const uint16_t upload_limit = static_cast<uint16_t>(HOST_CMD_PAYLOAD_MAX - 4u);
    resp.payload[p++] = static_cast<uint8_t>(upload_limit);
    resp.payload[p++] = static_cast<uint8_t>(upload_limit >> 8);
    const uint32_t src_capacity = PSOC_ALGO_SRC_MAX;
    resp.payload[p++] = static_cast<uint8_t>(src_capacity);
    resp.payload[p++] = static_cast<uint8_t>(src_capacity >> 8);
    resp.payload[p++] = static_cast<uint8_t>(src_capacity >> 16);
    resp.payload[p++] = static_cast<uint8_t>(src_capacity >> 24);
    const uint16_t src_chunk = static_cast<uint16_t>(HOST_CMD_ALGO_SRC_CHUNK);
    resp.payload[p++] = static_cast<uint8_t>(src_chunk);
    resp.payload[p++] = static_cast<uint8_t>(src_chunk >> 8);
    uint8_t caps_flags = slot_capacity_from_psoc ? 0x01u : 0u;
    if (slot_capacity_from_psoc && slot_capacity != PSOC_ALGO_MAX_LEN) caps_flags |= 0x02u;
    resp.payload[p++] = caps_flags;
    // ★上传中止现场★ 没有这几个字节, "上传没成功"在主机侧只是一句 len=0, 五种完全不同的成因
    // (BEGIN 未受理 / 状态泄漏 / 页回显不符 / END 失败 / 内容 CRC 不符)无法区分, 只能靠反复推断。
    uint16_t abort_page = 0u;
    uint8_t abort_reason = 0u;
    uint16_t abort_count = 0u;
    Psoc::getInstance()->algo_abort_info(&abort_page, &abort_reason, &abort_count);
    resp.payload[p++] = static_cast<uint8_t>(abort_page);
    resp.payload[p++] = static_cast<uint8_t>(abort_page >> 8);
    resp.payload[p++] = abort_reason;
    resp.payload[p++] = static_cast<uint8_t>(abort_count);
    resp.payload[p++] = static_cast<uint8_t>(abort_count >> 8);
    // ★LINK v2 起这两格换成链路级计数(宽度/偏移不变, 上位机无需改动)★
    // 前者 = 收到的非法帧数(SOF/CRC 不合法, 含从机 TX FIFO 被读空的全 0 帧);
    // 后者 = 事务超时重发次数。两者都是"链路抖动已被就地吸收"的证据: 若它们长期为 0 而上传仍失败,
    // 说明成因不在链路层, 不要再往这个方向修。
    const uint16_t resync_count = Psoc::getInstance()->algo_resync_count();
    resp.payload[p++] = static_cast<uint8_t>(resync_count);
    resp.payload[p++] = static_cast<uint8_t>(resync_count >> 8);
    const uint16_t restart_count = Psoc::getInstance()->algo_restart_count();
    resp.payload[p++] = static_cast<uint8_t>(restart_count);
    resp.payload[p++] = static_cast<uint8_t>(restart_count >> 8);
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_upload(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = len(u16 LE) + crc16(u16 LE) + data[len]
    if (frame.len < 4) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_upload header too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint16_t len = static_cast<uint16_t>(frame.payload[0]) |
                         (static_cast<uint16_t>(frame.payload[1]) << 8);
    const uint16_t crc16 = static_cast<uint16_t>(frame.payload[2]) |
                           (static_cast<uint16_t>(frame.payload[3]) << 8);
    if (len == 0u || len > PSOC_ALGO_MAX_LEN || (uint32_t)frame.len < 4u + (uint32_t)len) {
        // ★单帧承载上限比槽小 4 字节★ payload 固定 4096 而本命令要占 4 字节头(len+crc16),
        // 故一帧最多带 4092 字节代码; 槽本身是 4096。真要用满 4096 必须先给 ALGO_UPLOAD 加分片
        // (照 ALGO_SET_SRC 的 offset/total 协议), 那要上位机同步改, 不在本次改动范围。
        // 这里把上限如实写进错误文案, 免得用户对着"len invalid"猜半天。
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_upload len invalid(单帧最多 4092 字节代码)", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // ★必须先挡住"上一次下发还没做完"★: blob 缓冲被 core1 持有, 此刻 set_algo 改写它会让 PSoC
    // 收到半新半旧的代码(=必然跑飞的算法)。下发已异步化, 故这里明确回 DEVICE_BUSY 让上位机重试。
    if (Psoc::getInstance()->algo_download_busy()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "algo download still in progress", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    if (!store->set_algo(&frame.payload[4], len, crc16)) {   // 校验 crc16 一致才接受+持久化
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo crc16 mismatch", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // ★存储优先, 下发尽力★
    // 这里【刻意删掉了原来的 !link_alive() ⇒ NAK】。原逻辑的后果: PSoC 正被坏算法搞死(链路自然
    // 不可用)时, 用户想传一份修好的算法进来会被一律拒收 —— 而"传新算法"恰恰是唯一的自救手段,
    // 于是设备进入一个用户无法脱出的死锁(只能重烧 PSoC)。
    // 现在: blob 已经 set_algo 存进 RP 并落 flash(上面那步), 即使这一刻发不出去也不会丢:
    //   · set_algo 内部已 clear_quarantine() + 置 _download_pending ⇒ 新算法必然获得一次运行机会;
    //   · PsocAlgo::tick() 会在链路恢复/core1 空闲后自动把它推下去(500ms 节流重试);
    //   · 即便本次上电一直发不出去, 下次 PSoC 启动的 provisioning 也会下发它(R1)。
    // request_download() 现在恒返回"已受理"(链路不可用/入队失败只是推迟, 见其注释), 故它不再
    // 产生任何 NAK 出口。ACK 的语义是"已存下并受理下发", 真实生效由 ALGO_GET_INFO 的
    // psoc_len/psoc_crc16 回读对账(失败则主循环上报 SELF_HEAL_EVENT(SH_ALGO_FALLBACK))。
    (void)store->request_download(Psoc::getInstance());
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_apply(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (Psoc::getInstance()->algo_download_busy()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "algo download still in progress", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // 同 ALGO_UPLOAD: 只入队, 不同步推 ROM/cfg。
    if (PsocAlgo::getInstance()->request_download(Psoc::getInstance())) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "algo apply failed", response, HOST_CMD_RESP_BUF_MAX);
    }
}

void SensorLink::_handle_algo_reset_default(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    PsocAlgo* store = PsocAlgo::getInstance();
    Psoc* psoc = Psoc::getInstance();
    // 恢复请求与上传共享一个异步所有者。若此刻仍在下发，交由 PsocAlgo::tick() 等自然终态后
    // 自动执行，避免丢弃主机请求或复用已被 core1 持有的 blob 缓冲。
    if (!store->request_reset_default()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "default algo reset already pending", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // 空闲时立即切换 RP 侧默认元数据：上位机的有序写队列可能与直接 GET_INFO 轮询并行，
    // 先发布目标状态可避免旧自定义信息覆盖已受理的恢复请求。实际 PSoC 分页下发仍由 tick()
    // 异步启动；在途上传或命令环非空时绝不改写 core1 可能持有的 blob。
    if (!psoc->algo_download_busy() && psoc->core1_idle()) {
        store->reset_default();
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_set_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // Keep the wire format batched, but execute it through the one host-write owner in FIFO order.
    // ACK is emitted only after every entry completed, so no partial payload can be reported as success.
    if (frame.len == 0u || (frame.len % 3u) != 0u || frame.len > PSOC_ALGO_CHANNELS * 3u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_rom payload must contain 1..36 entries", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    SensorLink* self = getInstance();
    if (self->_host_write.active) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t count = static_cast<uint8_t>(frame.len / 3u);
    for (uint8_t i = 0; i < count; ++i) {
        const uint16_t p = static_cast<uint16_t>(i) * 3u;
        if (frame.payload[p] >= PSOC_ALGO_CHANNELS) {
            *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "algo_set_rom channel out of range", response, HOST_CMD_RESP_BUF_MAX);
            return;
        }
    }
    self->_host_write.clear();
    self->_host_write.active = true;
    self->_host_write.kind = HostWriteState::Kind::ALGO_ROM;
    self->_host_write.cmd = frame.cmd;
    self->_host_write.seq = frame.seq;
    self->_host_write.batch_count = count;
    for (uint8_t i = 0; i < count; ++i) {
        const uint16_t p = static_cast<uint16_t>(i) * 3u;
        self->_host_write.batch[i].a = frame.payload[p];
        self->_host_write.batch[i].b = 0u;
        self->_host_write.batch[i].value = static_cast<uint16_t>(frame.payload[p + 1]) |
                                           (static_cast<uint16_t>(frame.payload[p + 2]) << 8);
    }
    if (!self->_start_next_host_batch()) {
        self->_host_write.clear();
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC algo_set_rom enqueue failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;
}

void SensorLink::_handle_algo_get_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → 响应 36×u16 LE (RP 存储的每通道 ROM 表, 真相源)
    PsocAlgo* store = PsocAlgo::getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_ROM);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    for (uint8_t ch = 0; ch < PSOC_ALGO_CHANNELS; ++ch) {
        const uint16_t v = store->rom(ch);
        resp.payload[p++] = static_cast<uint8_t>(v);
        resp.payload[p++] = static_cast<uint8_t>(v >> 8);
    }
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_set_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [idx(u8), val(u8)]。写 PSoC 共享 cfg[idx] + PsocAlgo 持久化(随算法下发恢复)。
    if (frame.len < 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_cfg payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t idx = frame.payload[0];
    const uint8_t val = frame.payload[1];
    if (idx >= 8u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_cfg idx out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    SensorLink* self = getInstance();
    if (!self->_start_host_write(HostWriteState::Kind::ALGO_CFG, frame, idx, val, 0u)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;
}

void SensorLink::_handle_algo_get_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [idx(u8)] → 响应 [idx, cfg(u8)]
    if (frame.len < 1u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_cfg payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t idx = frame.payload[0];
    if (idx >= 8u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_cfg idx out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_CFG);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = idx;
    resp.payload[1] = PsocAlgo::getInstance()->cfg(idx);   // 直读 RP 存储的真相源(不下发 PSoC 查询)
    resp.len = 2;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_set_cfg_ch(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), idx(u8), val(u8)] × N。逐通道 cfg_ch[ch][idx](ABI v2)。
    // 与 ALGO_SET_ROM 完全同构: 整帧先全量校验, 再走**唯一的** host-write 所有者逐条下发,
    // 全部条目完成才 ACK —— 半张帧被当成成功是最难查的一类问题(设备与界面从此各说各话)。
    if (frame.len == 0u || (frame.len % 3u) != 0u ||
        frame.len > PSOC_ALGO_CHANNELS * 8u * 3u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_cfg_ch payload must be 1..288 [ch,idx,val] entries", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    SensorLink* self = getInstance();
    if (self->_host_write.active) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC host write slot busy", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint16_t count = static_cast<uint16_t>(frame.len / 3u);
    for (uint16_t i = 0; i < count; ++i) {
        const uint16_t p = static_cast<uint16_t>(i * 3u);
        if (frame.payload[p] >= PSOC_ALGO_CHANNELS || frame.payload[p + 1] >= 8u) {
            *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "algo_set_cfg_ch ch/idx out of range", response, HOST_CMD_RESP_BUF_MAX);
            return;
        }
    }
    self->_host_write.clear();
    self->_host_write.active = true;
    self->_host_write.kind = HostWriteState::Kind::ALGO_CFG_CH;
    self->_host_write.cmd = frame.cmd;
    self->_host_write.seq = frame.seq;
    self->_host_write.batch_count = count;
    for (uint16_t i = 0; i < count; ++i) {
        const uint16_t p = static_cast<uint16_t>(i * 3u);
        self->_host_write.batch[i].a = frame.payload[p];
        self->_host_write.batch[i].b = frame.payload[p + 1];
        self->_host_write.batch[i].value = frame.payload[p + 2];
    }
    if (!self->_start_next_host_batch()) {
        self->_host_write.clear();
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC algo_set_cfg_ch enqueue failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = 0u;   // 终态由 take_host_write_terminal 延迟 ACK
}

void SensorLink::_handle_algo_get_cfg_ch(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → 288 字节(ch 主序, 每 ch 连续 8 字节)。回的是 **RP 存储的真相源**:
    // 它才是"下次 PSoC 启动会被下发什么"的依据; 去查 PSoC 反而会在算法无效/正在上传时读到 0。
    PsocAlgo* store = PsocAlgo::getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_CFG_CH);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    for (uint8_t ch = 0; ch < PSOC_ALGO_CHANNELS; ++ch) {
        for (uint8_t idx = 0; idx < 8u; ++idx) resp.payload[p++] = store->cfg_ch(ch, idx);
    }
    resp.len = p;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_get_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 请求 [offset(u16 LE)](空 payload = offset 0) → 响应 [total(u16 LE), offset(u16 LE), chunk]。
    // 源最大 32KB, 单帧 payload 只有 4096, 故按 HOST_CMD_ALGO_SRC_CHUNK 分片, 由上位机按响应续请求。
    if (frame.len != 0u && frame.len != 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_src payload must be offset", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    const uint32_t total = store->src_len();
    uint32_t offset = 0u;
    if (frame.len >= 2u) {
        offset = static_cast<uint32_t>(frame.payload[0]) |
                 (static_cast<uint32_t>(frame.payload[1]) << 8);
    }
    if (offset > total) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_src offset past end", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    uint32_t n = total - offset;
    if (n > (uint32_t)HOST_CMD_ALGO_SRC_CHUNK) n = (uint32_t)HOST_CMD_ALGO_SRC_CHUNK;
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_SRC);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = static_cast<uint8_t>(total);
    resp.payload[1] = static_cast<uint8_t>(total >> 8);
    resp.payload[2] = static_cast<uint8_t>(offset);
    resp.payload[3] = static_cast<uint8_t>(offset >> 8);
    if (n > 0u) { memcpy(&resp.payload[4], store->src() + offset, n); }
    resp.len = static_cast<uint16_t>(4u + n);
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_set_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [offset(u16 LE), total(u16 LE), chunk]; 存算法 C 源(已由上位机滤注释)+持久化。
    // 严格连续分片: 只有最后一片才让新源生效, 中途断掉不会把半份源固化进 flash。
    if (frame.len < 4u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src header too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint32_t offset = static_cast<uint32_t>(frame.payload[0]) |
                            (static_cast<uint32_t>(frame.payload[1]) << 8);
    const uint32_t total  = static_cast<uint32_t>(frame.payload[2]) |
                            (static_cast<uint32_t>(frame.payload[3]) << 8);
    const uint32_t n = static_cast<uint32_t>(frame.len) - 4u;
    if (total > PSOC_ALGO_SRC_MAX || offset > total || n > total - offset) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src chunk range invalid", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    if (!PsocAlgo::getInstance()->set_src_chunk(offset, total, &frame.payload[4], n)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src chunk not contiguous", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_get_code(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → [len(u16 LE), asm bytes] RP2040 存的算法 ASM 机器码回读(供反汇编查看)。
    PsocAlgo* store = PsocAlgo::getInstance();
    const uint16_t n = store->len();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_CODE);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = static_cast<uint8_t>(n);
    resp.payload[1] = static_cast<uint8_t>(n >> 8);
    if (n > 0u) { memcpy(&resp.payload[2], store->data(), n); }
    resp.len = static_cast<uint16_t>(2u + n);
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}
