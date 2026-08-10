#include "sensor_link.h"
#include "../../hal/usb/hal_usb.h"
#include "../../protocol/psoc/psoc.h"
#include "../csd_config/csd_config.h"
#include "../psoc_algo/psoc_algo.h"
#include "../psoc_updater/psoc_updater.h"
#include "../tx_scheduler/tx_scheduler.h"
#include "../latency_stats.h"
#include <pico/stdlib.h>
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
// 步进周期 2ms: 一格 = 写参数 + 回读 + settle/sample 快照；扫描期不逐格校准，结束后统一恢复校准，
// 全程非阻塞单步推进，每周期最多做一件事，绝不在 handler 或 tick 里等设备。
static constexpr uint32_t SWEEP_INTERVAL_US = 2000;
// 扫描期租约: 任何主机帧都会 renew_all(3s) 覆盖它, 故上位机在 = 永活; 上位机丢失 ⇒ 到期转入恢复。
static constexpr uint32_t SWEEP_LEASE_MS = 8000;
// 阶段硬超时(周期数 × 2ms): 普通等待 5s(参数落地/快照推进), 重操作 40s(单通道 IDAC 校准/基线复位),
// 终态帧重试 3s(USB 长时间背压/掉线也必须收尾, 否则任务与被抑制的输出永久悬空)。
static constexpr uint16_t SWEEP_WAIT_TICKS = 2500;
static constexpr uint16_t SWEEP_HEAVY_TICKS = 20000;
static constexpr uint16_t SWEEP_TERMINAL_TICKS = 1500;
static constexpr uint8_t  SWEEP_FRAMES_PER_TICK = 2;    // 结果/补发帧的发送预算(背压即停, 下拍续发)
static constexpr uint8_t  SWEEP_SETTLE_MAX = 64;
static constexpr uint8_t  SWEEP_SAMPLE_MAX = 64;
static constexpr uint16_t SWEEP_RAILED_RAW = 0xFFF0u;   // 接近满量程 ⇒ IDAC 补偿不足, 该格无效
static constexpr uint16_t SWEEP_INDEX_NONE = 0xFFFFu;   // 非结果帧(RESTORING/终态)的格号占位
static constexpr uint8_t  SWEEP_FRAME_LEN = 23;
static constexpr uint8_t  SWEEP_SEGMENT_CELLS = 8;
// 结果位: 低 4 位属该格采样, 高 4 位属恢复阶段, 复用同一 flags 字节回显给上位机。
static constexpr uint8_t  SWEEP_FLAG_RAILED = 0x01;
static constexpr uint8_t  SWEEP_FLAG_STALLED = 0x02;      // 全部样本完全不抖动 = 扫描停滞
static constexpr uint8_t  SWEEP_FLAG_CAL_FAIL = 0x04;
static constexpr uint8_t  SWEEP_FLAG_MISMATCH = 0x08;     // 回读值 != 期望值(PSoC 侧钳位/拒绝)
static constexpr uint8_t  SWEEP_RESTORE_FLAG_PARAM = 0x10;
static constexpr uint8_t  SWEEP_RESTORE_FLAG_CAL = 0x20;
static constexpr uint8_t  SWEEP_RESTORE_FLAG_BSLN = 0x40;
static constexpr uint8_t  SWEEP_FLAG_RETRANSMIT = 0x80;   // 该帧为 SWEEP_CTRL 补发
static constexpr uint8_t  SWEEP_PARAM_ID_DIV = 0x08u;     // SNS_CLK_DIV
static constexpr uint8_t  SWEEP_PARAM_ID_GAIN = 0x0Bu;    // IDAC_GAIN(0..6)

volatile uint16_t g_lat_spi_us = 0;
volatile uint16_t g_lat_proc_us = 0;
volatile uint16_t g_lat_usb_us = 0;

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
// Cp 哨兵(与 PSoC/上位机一致): 未测量 / 测量失败 / 读取失败。
constexpr uint32_t CP_UNMEASURED_FF = 0x00FFFFFFu;
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
    dispatcher->register_handler(HostCmd::ALGO_GET_TRACE, _handle_algo_get_trace);
    dispatcher->register_handler(HostCmd::ALGO_SET_CFG, _handle_algo_set_cfg);
    dispatcher->register_handler(HostCmd::ALGO_GET_CFG, _handle_algo_get_cfg);
    dispatcher->register_handler(HostCmd::ALGO_GET_SRC, _handle_algo_get_src);
    dispatcher->register_handler(HostCmd::ALGO_SET_SRC, _handle_algo_set_src);
    dispatcher->register_handler(HostCmd::ALGO_GET_CODE, _handle_algo_get_code);
}

void SensorLink::_release_focus_scan() {
    Psoc* psoc = Psoc::getInstance();
    psoc->set_focus_channel(0xFFu);
    (void)psoc->set_focus_scan(0xFFu);
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
        Psoc* psoc = Psoc::getInstance();
        if (!psoc->set_focus_scan(_focus.channel)) {
            _focus.suspended = true;
            _release_focus_scan();
            return;
        }
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
            status = sample.status;
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
    if (static_cast<uint32_t>(now_us - _focus.last_focus_scan_ack_us) >= FOCUS_SCAN_RENEW_INTERVAL_US) {
        if (!psoc->set_focus_scan(_focus.channel)) {
            _focus.clear();
            _release_focus_scan();
            _restore_broad_after_focus();
            return;
        }
        _focus.last_focus_scan_ack_us = now_us;
    }

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
    if ((_focus.fields & TELEM_FIELD_STATUS) != 0) payload[length++] = sample.status;
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

// ★扫描会话独占该通道的 gain/div 与 PSoC 重操作槽★
// 期间放行调参/校准/自适应会有两个后果: ① 抢走单一在途重操作槽, 让扫描步进反复失败直至超时;
// ② 恢复阶段写回的"原值"与上位机中途改出来的值打架, 会话结束后没人知道设备到底是什么状态。
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

// 单格终结: 把聚合量落进 448 格缓存(留待流式发送与补发); 无论该格有效与否，都只推进当前格。
void SensorLink::_finish_sweep_cell() {
    SweepResult& result = _sweep_results[_sweep.cell];
    const SweepStats& stats = _sweep.stats;
    result.samples = static_cast<uint16_t>(stats.count);
    if (stats.count != 0u) {
        const uint32_t mean = static_cast<uint32_t>(stats.sum / stats.count);
        result.mean = static_cast<uint16_t>(mean);
        result.pp = static_cast<uint16_t>((stats.max >= stats.min) ? (stats.max - stats.min) : 0u);
        // 方差 = E[x²] - E[x]²(u16 输入下 sum_sq/count 与 mean² 都 < 2^32, 不会溢出);
        // 负值只可能来自整除截断, 钳到 0。q8 定点: 整数部分 + 由余数推出的小数部分, 只用整型开方。
        const uint32_t mean_sq = mean * mean;
        const uint32_t e_sq = static_cast<uint32_t>(stats.sum_sq / stats.count);
        const uint32_t variance = (e_sq > mean_sq) ? (e_sq - mean_sq) : 0u;
        const uint32_t root = _sqrt_u32(variance);
        const uint32_t frac = ((variance - root * root) << 8) / (2u * root + 1u);
        const uint32_t std_q8 = (root << 8) + frac;
        result.std_q8 = (std_q8 > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(std_q8);
        if (stats.railed != 0u) result.flags |= SWEEP_FLAG_RAILED;
        if (stats.max == stats.min) result.flags |= SWEEP_FLAG_STALLED;
    }

    const uint8_t invalid_flags = SWEEP_FLAG_CAL_FAIL | SWEEP_FLAG_MISMATCH |
                                  SWEEP_FLAG_STALLED | SWEEP_FLAG_RAILED;
    const uint8_t trigger_flags = result.flags & invalid_flags;
    if (result.samples == 0u || trigger_flags != 0u) {
        // RAILED/STALLED 有真实样本，是扫描发现的预期无效点而非状态机故障：不污染会话/单格阶段码。
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
    _sweep.cell++;
    _sweep.produced = _sweep.cell;
    if (_sweep.cell >= SWEEP_TOTAL) {
        _begin_sweep_restore(SweepDataState::DONE);
        return;
    }
    _sweep_enter(SweepPhase::SET_CELL);
}

// 本会话首个失败阶段只记一次(后续故障不覆盖): 第一个才是根因, 后面的多半是它的连带结果。
void SensorLink::_sweep_note_fail_phase() {
    if (_sweep.fail_phase == 0u) _sweep.fail_phase = static_cast<uint8_t>(_sweep.phase);
}

// 扫描期(非恢复链)阶段的有界超时处置。
// 当前格超时后按其实际回读值保留结果与阶段码；只将当前格标为无效，扫描仍逐格完成全部 448 格后统一走 DONE 恢复链。
void SensorLink::_sweep_cell_timeout(uint16_t limit) {
    if (_sweep.phase_ticks <= limit) return;
    _sweep_note_fail_phase();
    SweepResult& result = _sweep_results[_sweep.cell];
    if (static_cast<uint8_t>(_sweep.phase) <= static_cast<uint8_t>(SweepPhase::READBACK)) {
        result.clear();
        result.gain = static_cast<uint8_t>(_sweep.cell / SWEEP_DIV_COUNT);
        result.div = static_cast<uint8_t>((_sweep.cell % SWEEP_DIV_COUNT) + 1u);
    }
    result.flags |= SWEEP_FLAG_CAL_FAIL;
    result.fail_phase = static_cast<uint8_t>(_sweep.phase);
    // 该格没有可信样本: 清空聚合量, 由 _finish_sweep_cell() 以 samples=0 落盘后只推进当前格。
    _sweep.stats.clear();
    _finish_sweep_cell();
}

// SWEEP_DATA(0x38) 定长 23 字节:
//   [session u16][state u8][ch u8][index u16][total u16][produced u16][flags u8]
//   [gain u8][div u8][samples u16][mean u16][std_q8 u16][pp u16][fail_phase u8][phase u8]
// 非结果帧(RESTORING/DONE/CANCELLED/FAILED) index=0xFFFF 且格字段为 0, flags 回显恢复阶段异常位。
// ★尾部追加而非改布局★ 前 21 字节与旧固件逐字节相同(与 KBD_GET_STATE / AUTO_TUNE_PROGRESS 同一
// 先例), 只读前 21 字节的旧上位机不受影响; 新增两项让每个阶段都可枚举:
//   fail_phase = 结果帧→该格卡住的阶段, 非结果帧→本会话首个失败阶段(0=无故障);
//   phase      = 发帧当刻的会话阶段(SweepPhase), 用于追踪"卡在哪一步"这类未知故障。
// 返回是否真的发出: 背压丢弃时调用方保持游标不动, 下一周期重发(结果与终态都不允许静默丢失)。
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

// 取消 / 租约到期 / 任一阶段失败的唯一去处: 先把原 gain/div 写回并重新校准 + 复位基线, 再报终态。
// ★不允许直接结束会话★: 扫描把该通道的 IDAC 增益与 snsClk 分频改成了中间值, 不写回 + 不校准的话
// 触控在会话结束后依然是坏的(上位机看不出来, 只会以为"扫描完就坏了")。
void SensorLink::_begin_sweep_restore(SweepDataState terminal) {
    if (!_sweep.active()) return;
    if (_sweep_phase_is_restore(_sweep.phase)) return;   // 已在恢复链上: 保持首个终因, 不覆盖
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
    if (restore == SweepOutputRestore::STOPPED) {
        // 会话是被 stop()(TELEM_STOP / 新主机 HELLO)掀掉的: 流状态已在 stop() 里清空, 这里只收尾硬件。
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
        if (_emit_sweep_frame(_sweep.resend_cursor, SweepDataState::CELL, true)) {
            _sweep.resend_cursor++;
            budget--;
        } else {
            tx_blocked = true;
        }
    }
    while (!tx_blocked && budget != 0u && _sweep.stream_cursor < _sweep.produced) {
        if (_emit_sweep_frame(_sweep.stream_cursor, SweepDataState::CELL, false)) {
            _sweep.stream_cursor++;
            budget--;
        } else {
            tx_blocked = true;
        }
    }
    // ★Bug 3 修复★: 扫描期背压只暂停"发送已产出的结果帧"（stream_cursor/resend_cursor 不前进），
    // 但允许状态机继续推进（SET_CELL/READBACK/SAMPLE 等阶段产出新结果缓存在 _sweep_results[]）。
    // 恢复期背压仍照常推进（写回/校准/基线不依赖 USB）。
    // 删除原有的 "if (tx_blocked && !_sweep_phase_is_restore(_sweep.phase)) return;"，
    // 让状态机在扫描期背压时继续推进游标产出结果（缓存在数组里），下一 tick USB 空闲后再继续发送。
    // 这样遥测帧占用 USB 带宽时，扫描状态机不会卡住，只是结果帧暂时积压在缓冲区。

    switch (_sweep.phase) {
        case SweepPhase::SET_CELL:
            // gain/div 必须随 QUICK_APPLY 原子下发；扫描 ISR 中先 SET_PARAM 会改当前扫描的
            // widgetContext，导致这一轮 CapSense 扫描卡死，主循环永远到不了 pending。
            _sweep_enter(SweepPhase::START_CALIBRATE);
            return;

        case SweepPhase::WAIT_PARAMS:
            // 写类指令异步入队 ⇒ 必须等 core1 把它们真正执行完再回读, 否则读到的是上一格的值。
            if (!psoc->core1_idle()) {
                _sweep_cell_timeout(SWEEP_WAIT_TICKS);
                return;
            }
            _sweep_enter(SweepPhase::READBACK);
            return;

        case SweepPhase::READBACK: {
            uint32_t gain = 0, div = 0;
            if (!psoc->get_param(channel, SWEEP_PARAM_ID_GAIN, &gain) ||
                !psoc->get_param(channel, SWEEP_PARAM_ID_DIV, &div)) {
                _sweep_cell_timeout(SWEEP_WAIT_TICKS);
                return;
            }
            // ★该格记录的是设备实际生效值★: PSoC 可能钳位/拒绝, 记期望值会让整张图对不上真相。
            SweepResult& result = _sweep_results[_sweep.cell];
            result.clear();
            result.gain = static_cast<uint8_t>(gain);
            result.div = static_cast<uint8_t>(div);
            if (gain != (_sweep.cell / SWEEP_DIV_COUNT) ||
                div != ((_sweep.cell % SWEEP_DIV_COUNT) + 1u)) {
                result.flags |= SWEEP_FLAG_MISMATCH;
                result.fail_phase = static_cast<uint8_t>(SweepPhase::READBACK);
                _sweep_note_fail_phase();
                _sweep.stats.clear();
                _finish_sweep_cell();
                return;
            }
            // readback 已确认 QUICK_APPLY 的设备真值；从新代次开始 settle，绝不采改参前快照。
            _sweep.stats.clear();
            _sweep.settle_seen = 0;
            _sweep.last_generation = psoc->snapshot_generation();
            _sweep_enter((_sweep.settle_samples != 0u) ? SweepPhase::SETTLE : SweepPhase::SAMPLE);
            return;
        }

        case SweepPhase::START_CALIBRATE: {
            // 保留旧阶段码供协议兼容：该槽发起携带 gain/div 的 QUICK_APPLY，而非逐格校准。
            const uint8_t gain = static_cast<uint8_t>(_sweep.cell / SWEEP_DIV_COUNT);
            const uint8_t div = static_cast<uint8_t>((_sweep.cell % SWEEP_DIV_COUNT) + 1u);
            if (!psoc->start_sweep_apply(channel, gain, div)) {
                _sweep_cell_timeout(SWEEP_WAIT_TICKS);
                return;
            }
            _sweep_enter(SweepPhase::WAIT_CALIBRATE);
            return;
        }

        case SweepPhase::WAIT_CALIBRATE: {
            bool ok = false;
            if (!psoc->take_sweep_result(&ok)) {
                _sweep_cell_timeout(SWEEP_HEAVY_TICKS);
                return;
            }
            // 参数应用拒绝只标记当前格无效；后续格仍须各自完成 QUICK_APPLY、READBACK、SETTLE 与 SAMPLE。
            if (!ok) {
                SweepResult& result = _sweep_results[_sweep.cell];
                result.flags |= SWEEP_FLAG_CAL_FAIL;
                result.fail_phase = static_cast<uint8_t>(SweepPhase::WAIT_CALIBRATE);
                _sweep_note_fail_phase();
                _sweep.stats.clear();
                _finish_sweep_cell();
                return;
            }
            // QUICK_APPLY 已完成，先读回确认 PSoC 的真值；只有一致才开始 settle/sample。
            _sweep_enter(SweepPhase::READBACK);
            return;
        }

        case SweepPhase::SETTLE: {
            // 丢弃校准后前若干帧(基线/滤波仍在收敛)。按快照代次计数, 不按时间猜。
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
            if (raw >= SWEEP_RAILED_RAW && stats.railed < 0xFFu) stats.railed++;
            // 每 8 格段首是快速锚点，最多采 8 个样本；锚点有效后，段内其余格仍用请求的完整样本数精扫。
            const uint8_t sample_target = ((_sweep.cell % SWEEP_SEGMENT_CELLS) == 0u &&
                                           _sweep.sample_count > SWEEP_SEGMENT_CELLS)
                ? SWEEP_SEGMENT_CELLS : _sweep.sample_count;
            if (stats.count >= sample_target) _finish_sweep_cell();
            return;
        }

        case SweepPhase::RESTORE_WRITE:
            // 恢复同样禁止先 SET_PARAM：原 gain/div 与 QUICK_APPLY 同帧下发，随后 readback 确认。
            _sweep_enter(SweepPhase::RESTORE_APPLY);
            return;

        case SweepPhase::RESTORE_WAIT_PARAMS:
            // 旧状态码仅为协议诊断兼容保留；新恢复路径不会到达。
            _sweep_enter(SweepPhase::RESTORE_APPLY);
            return;

        case SweepPhase::RESTORE_READBACK: {
            uint32_t gain = 0, div = 0;
            if (!psoc->get_param(channel, SWEEP_PARAM_ID_GAIN, &gain) ||
                !psoc->get_param(channel, SWEEP_PARAM_ID_DIV, &div)) {
                if (_sweep.phase_ticks > SWEEP_WAIT_TICKS) {
                    _sweep_note_fail_phase();
                    _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
                    _sweep_enter(SweepPhase::RESTORE_CALIBRATE);
                }
                return;
            }
            if (gain != _sweep.original_gain || div != _sweep.original_div) {
                _sweep_note_fail_phase();
                _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
            }
            // QUICK_APPLY 已完成；写穿设备真值后只做一次 CALIBRATE + BASELINE 收口。
            CsdConfig* csd = CsdConfig::getInstance();
            csd->note_param(channel, SWEEP_PARAM_ID_GAIN, gain);
            csd->note_param(channel, SWEEP_PARAM_ID_DIV, div);
            _sweep_enter(SweepPhase::RESTORE_CALIBRATE);
            return;
        }

        case SweepPhase::RESTORE_APPLY:
            // 直接携带会话起始时的原 gain/div；PSoC 在 NOT_BUSY 窗口原子写入后 Initialize。
            if (!psoc->start_sweep_apply(channel, _sweep.original_gain, _sweep.original_div)) {
                if (_sweep.phase_ticks > SWEEP_WAIT_TICKS) {
                    _sweep_note_fail_phase();
                    _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
                    _sweep_enter(SweepPhase::RESTORE_CALIBRATE);
                }
                return;
            }
            _sweep_enter(SweepPhase::RESTORE_WAIT_APPLY);
            return;

        case SweepPhase::RESTORE_WAIT_APPLY: {
            bool ok = false;
            if (!psoc->take_sweep_result(&ok)) {
                if (_sweep.phase_ticks > SWEEP_HEAVY_TICKS) {
                    _sweep_note_fail_phase();
                    _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM;
                    _sweep_enter(SweepPhase::RESTORE_CALIBRATE);
                }
                return;
            }
            if (!ok) { _sweep_note_fail_phase(); _sweep.restore_flags |= SWEEP_RESTORE_FLAG_PARAM; }
            _sweep_enter(SweepPhase::RESTORE_READBACK);
            return;
        }

        case SweepPhase::RESTORE_CALIBRATE:
            if (!psoc->start_sweep_calibrate(channel)) {
                if (_sweep.phase_ticks > SWEEP_WAIT_TICKS) {
                    _sweep_note_fail_phase();
                    _sweep.restore_flags |= SWEEP_RESTORE_FLAG_CAL;
                    _sweep_enter(SweepPhase::RESTORE_BASELINE);
                }
                return;
            }
            _sweep_enter(SweepPhase::RESTORE_WAIT_CALIBRATE);
            return;

        case SweepPhase::RESTORE_WAIT_CALIBRATE: {
            bool ok = false;
            if (!psoc->take_sweep_result(&ok)) {
                if (_sweep.phase_ticks > SWEEP_HEAVY_TICKS) {
                    _sweep_note_fail_phase();
                    _sweep.restore_flags |= SWEEP_RESTORE_FLAG_CAL;
                    _sweep_enter(SweepPhase::RESTORE_BASELINE);
                }
                return;
            }
            if (!ok) { _sweep_note_fail_phase(); _sweep.restore_flags |= SWEEP_RESTORE_FLAG_CAL; }
            _sweep_enter(SweepPhase::RESTORE_BASELINE);
            return;
        }

        case SweepPhase::RESTORE_BASELINE:
            // 校准后再显式复位一次基线: 扫描期间的中间值已把该通道基线污染, 不复位会留下持续误触。
            if (!psoc->start_sweep_baseline_reset(channel)) {
                if (_sweep.phase_ticks > SWEEP_WAIT_TICKS) {
                    _sweep_note_fail_phase();
                    _sweep.restore_flags |= SWEEP_RESTORE_FLAG_BSLN;
                    _sweep_enter(SweepPhase::TERMINAL);
                }
                return;
            }
            _sweep_enter(SweepPhase::RESTORE_WAIT_BASELINE);
            return;

        case SweepPhase::RESTORE_WAIT_BASELINE: {
            bool ok = false;
            if (!psoc->take_sweep_result(&ok)) {
                if (_sweep.phase_ticks > SWEEP_HEAVY_TICKS) {
                    _sweep_note_fail_phase();
                    _sweep.restore_flags |= SWEEP_RESTORE_FLAG_BSLN;
                    _sweep_enter(SweepPhase::TERMINAL);
                }
                return;
            }
            if (!ok) { _sweep_note_fail_phase(); _sweep.restore_flags |= SWEEP_RESTORE_FLAG_BSLN; }
            _sweep_enter(SweepPhase::TERMINAL);
            return;
        }

        case SweepPhase::TERMINAL:
            // 全部格结果与补发都送出后才发终态帧, 背压则下拍重发。★重试窗到点无条件收尾★:
            // 此刻硬件已经恢复完毕, 若还因为"上位机不收"而不清会话, 输出抑制与任务就永久悬着
            // (表现为扫描完成后触摸再也不输出), 那比丢一帧终态严重得多。
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
    if (!Psoc::getInstance()->set_focus_scan(channel)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC focus_scan was not accepted", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
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
    // 快照转入单通道快路: 独占期只读该通道 ⇒ 每个 core1 拍都能出一份新代数(不再是 16 拍一份),
    // 精调曲线才拿得到设备的全速采样。FOCUS_STOP / 会话清理时必须还原为 0xFF。
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
    // 扫描全程独占 PSoC 的重操作槽(每格一次校准), 已有长周期指令在途时如实回绝而不排队。
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(增益/分频扫描)",
                          frame, response, response_length)) return;
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->link_alive()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC link is down", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // ★先拿到原值再动手★: 取不到就绝不开扫 —— 否则会话结束时无从写回, 该通道永久停在扫描参数上。
    uint32_t original_gain = 0, original_div = 0;
    if (!psoc->get_param(channel, SWEEP_PARAM_ID_GAIN, &original_gain) ||
        !psoc->get_param(channel, SWEEP_PARAM_ID_DIV, &original_div)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC param readback failed (sweep needs original gain/div)", response, HOST_CMD_RESP_BUF_MAX);
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
// 取消只是“请求恢复”(仍要走完写回+校准+基线才报终态)；补发只允许取已缓存的格(index < produced)。
// 续租必须显式存在：448 格会话远长于 8s，不能假设其它主机命令刚好替它续期。
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
    if (count == 0u || first >= self->_sweep.produced) {
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

    if (Psoc::getInstance()->set_param(ch, param_id, value)) {
        CsdConfig::getInstance()->note_param(ch, param_id, value);  // 写穿 RP2040 真相源
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC param_set failed", response, HOST_CMD_RESP_BUF_MAX);
    }
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
    uint32_t value = 0;
    if (!Psoc::getInstance()->get_param(ch, param_id, &value)) {
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
    // payload = 0xFF + param_id(u8) → "全通道单参数"变体: 响应 0xFF + param_id + count + [ch + value(u32 LE)]×count
    //   (36 通道单参数一次取回, 替代 36 条单发 PARAM_GET; 36×5+3=183B 仍在单帧内)
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_get_all payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t ch = frame.payload[0];
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
    for (uint8_t i = 0; i < kParamCount; i++) {
        uint32_t value = 0;
        if (!Psoc::getInstance()->get_param(ch, kParamIds[i], &value)) continue;
        resp.payload[position++] = kParamIds[i];
        resp.payload[position++] = static_cast<uint8_t>(value);
        resp.payload[position++] = static_cast<uint8_t>(value >> 8);
        resp.payload[position++] = static_cast<uint8_t>(value >> 16);
        resp.payload[position++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[count_position] = count;
    resp.len = position;
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

// "全通道单参数"批量回读: 响应 0xFF + param_id + count + [ch + value(u32 LE)]×count。
// 读不到的通道直接跳过(不占 pair), 上位机按 count 解析。
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
    for (uint8_t ch = 0; ch < SENSOR_LINK_CHANNELS; ch++) {
        uint32_t value = 0;
        if (!Psoc::getInstance()->get_param(ch, param_id, &value)) continue;
        resp.payload[position++] = ch;
        resp.payload[position++] = static_cast<uint8_t>(value);
        resp.payload[position++] = static_cast<uint8_t>(value >> 8);
        resp.payload[position++] = static_cast<uint8_t>(value >> 16);
        resp.payload[position++] = static_cast<uint8_t>(value >> 24);
        count++;
    }
    resp.payload[count_position] = count;
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
    if (Psoc::getInstance()->calibrate(cal_ch)) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC calibrate failed", response, HOST_CMD_RESP_BUF_MAX);
    }
}

void SensorLink::_handle_baseline_reset(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = ch_mask(u64 LE); 单位掩码 ⇒ 只复位该通道基线(见 _mask_to_single_ch)。
    const uint8_t bsln_ch = _mask_to_single_ch(frame);
    if (disabled_ch_reject("该通道已禁用(电极保持高阻), 无法复位基线; 请先启用该通道",
                           bsln_ch, frame, response, response_length)) return;
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(基线复位)", frame, response, response_length)) return;
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(基线复位)", frame, response, response_length)) return;
    if (Psoc::getInstance()->baseline_reset(bsln_ch)) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC baseline reset failed", response, HOST_CMD_RESP_BUF_MAX);
    }
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
    if (Psoc::getInstance()->set_mode(frame.payload[0])) {
        CsdConfig::getInstance()->note_mode(frame.payload[0]);  // 写穿 RP2040 真相源
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC mode_set failed", response, HOST_CMD_RESP_BUF_MAX);
    }
}

void SensorLink::_handle_csd_capture(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // AUTO 的实时参数由 CapSense 自动计算，回读固化会覆盖用户保存的半自动手动参数。
    CsdConfig* csd = CsdConfig::getInstance();
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
    if (Psoc::getInstance()->measure_cp()) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC cp_measure failed", response, HOST_CMD_RESP_BUF_MAX);
    }
}

void SensorLink::_handle_cp_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 1 || frame.payload[0] >= SENSOR_LINK_CHANNELS) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "cp_get requires channel 0..35", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    const uint8_t ch = frame.payload[0];
    uint32_t cp_ff = 0;
    // ★不再 NAK★: PSoC 未测量/测量失败本就以哨兵 0xFFFFFF 表达; 读取(SPI 忙/超时)失败时也回同一
    // 哨兵的正常响应, 使上位机显示"未测量/测量失败"而不是刷 NAK 日志(实测 NAK 每秒 8~12 条刷屏)。
    // NAK 只保留给非法通道(上面已处理)。
    if (!Psoc::getInstance()->get_cp(ch, &cp_ff)) {
        cp_ff = CP_UNMEASURED_FF;
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
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->set_global(gid, value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC global_set failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    CsdConfig::getInstance()->note_global(gid, value);   // 写穿 RP2040 真相源
    // ★不再逐项 commit★: 只写影子。由上位机在批量下发全部全局项后发一次 GLOBAL_COMMIT 统一重初始化,
    // 避免"每项各触发一次完整 Init+Enable 重校准"的风暴(实测会拖垮 core0/USB → 掉线, 见 log.log)。
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

// 批量全局项下发完毕后, 单次触发 PSoC 完整重初始化(合并, 防反复重校准漂移/风暴)。
void SensorLink::_handle_global_commit(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 全局提交会整片重初始化 + 重校准, 与逐格扫描直接冲突(会把已扫的参数与基线一并推翻)。
    if (_sweep_busy_reject("扫描会话进行中, 请先取消或等待完成(全局提交)", frame, response, response_length)) return;
    if (heavy_gate_reject("PSoC 正在执行长周期指令, 请稍后重试(全局提交)", frame, response, response_length)) return;
    Psoc::getInstance()->global_commit();
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_global_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空 → [count(u8), (gparam_id, value u32 LE)×count]
    Psoc* psoc = Psoc::getInstance();
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
    // 空请求 → [is_default(u8), psoc_valid(u8), len(u16 LE), crc16(u16 LE)]
    PsocAlgo* store = PsocAlgo::getInstance();
    bool psoc_valid = false;
    uint16_t psoc_len = 0;
    Psoc::getInstance()->get_algo_info_cached(&psoc_valid, &psoc_len);

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_INFO);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    uint16_t p = 0;
    resp.payload[p++] = store->is_default() ? 1u : 0u;
    resp.payload[p++] = psoc_valid ? 1u : 0u;
    resp.payload[p++] = static_cast<uint8_t>(store->len());
    resp.payload[p++] = static_cast<uint8_t>(store->len() >> 8);
    resp.payload[p++] = static_cast<uint8_t>(store->crc16());
    resp.payload[p++] = static_cast<uint8_t>(store->crc16() >> 8);
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
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_upload len invalid", response, HOST_CMD_RESP_BUF_MAX);
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
    // 下发已受理(异步, 重活在 core1)。真实结果经 ALGO_GET_INFO 的 psoc_valid/len 回读对账,
    // 失败则由主循环上报 SELF_HEAL_EVENT(SH_ALGO_FALLBACK, detail=1)。
    // ★把失败原因分开, 不要都报成同一句 SENSOR_ERROR★
    // download_to_psoc 只有三个失败出口: 链路不可用 / 存储为空 / 入队失败。原先一律回
    // "algo download enqueue failed", 于是链路瞬断也被说成入队失败, 排查时完全指错方向(实测踩过)。
    // 链路类是【可重试】的, 必须回 DEVICE_BUSY 让上位机自动重试, 而不是 SENSOR_ERROR 让用户以为算法坏了。
    // 用去抖后的 link_alive(): 遥测流式期间瞬时 link_ok 频繁为 false, 会把上传全部拒掉。
    if (!Psoc::getInstance()->link_alive()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "PSoC 链路暂时不可用(可能正在重初始化), 请稍后重试", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // 只入队代码下发即返回, ROM/cfg 由 PsocAlgo::tick() 补推(同步推会把 ACK 拖到几秒后)。
    if (!store->request_download(Psoc::getInstance())) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "算法下发入队失败(core1 正忙于重校准/重初始化), 请稍后重试", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
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
    // 同 ALGO_UPLOAD: reset_default 会改写 blob, 下发在途时必须先挡住。
    if (Psoc::getInstance()->algo_download_busy()) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "algo download still in progress", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    store->reset_default();                          // 回退内嵌默认 + 请求持久化
    store->request_download(Psoc::getInstance());    // 只入队(失败也回 ACK, 启动会重推)
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_set_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), rom_lo, rom_hi] × count (count = len/3)
    if (frame.len == 0u || (frame.len % 3u) != 0u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_rom payload must be 3*N bytes", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    Psoc* psoc = Psoc::getInstance();
    const uint16_t entries = (uint16_t)(frame.len / 3u);
    for (uint16_t i = 0; i < entries; ++i) {
        const uint16_t p = (uint16_t)(i * 3u);
        const uint8_t ch = frame.payload[p];
        const uint16_t rom = (uint16_t)frame.payload[p + 1] | ((uint16_t)frame.payload[p + 2] << 8);
        if (ch >= PSOC_ALGO_CHANNELS) {
            *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "algo_set_rom channel out of range", response, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        store->set_rom(ch, rom);           // 更新 RP 存储 + 请求持久化(真相源)
        psoc->set_algo_rom(ch, rom);       // 立即下发 PSoC
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
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

void SensorLink::_handle_algo_get_trace(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), idx(u8)] → 响应 [ch, idx, out_active(u8), report(u16 LE)]
    if (frame.len < 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_trace payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t idx = frame.payload[1];
    if (ch >= PSOC_ALGO_CHANNELS || idx >= 4u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_trace ch/idx out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    uint8_t active = 0;
    uint16_t report = 0;
    if (!Psoc::getInstance()->algo_get_trace(ch, idx, &active, &report)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC algo_get_trace failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_TRACE);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = ch;
    resp.payload[1] = idx;
    resp.payload[2] = active;
    resp.payload[3] = static_cast<uint8_t>(report);
    resp.payload[4] = static_cast<uint8_t>(report >> 8);
    resp.len = 5;
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
    if (!Psoc::getInstance()->algo_set_cfg(idx, val)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC algo_set_cfg failed", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    PsocAlgo::getInstance()->set_cfg(idx, val);   // 更新 RP 存储 + 请求持久化(真相源)
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
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
