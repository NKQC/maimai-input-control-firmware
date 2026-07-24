#include "sensor_link.h"
#include "../../hal/usb/hal_usb.h"
#include "../../protocol/psoc/psoc.h"
#include "../csd_config/csd_config.h"
#include "../psoc_algo/psoc_algo.h"
#include "../tx_scheduler/tx_scheduler.h"
#include "../latency_stats.h"
#include <pico/stdlib.h>
#include <cstring>

// 遥测租约(ms): 上位机需在此时限内经任意命令帧续期(UsbComm renew_all), 否则任务自动停。
static constexpr uint32_t TELEM_LEASE_MS = 3000;

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
};
constexpr uint8_t kParamCount = sizeof(kParamIds) / sizeof(kParamIds[0]);
}  // namespace

SensorLink::SensorLink()
    : _streaming(false),
      _mode(0),
      _rate_hz(30),
      _fields(TELEM_FIELD_RAW | TELEM_FIELD_BASELINE | TELEM_FIELD_DIFF | TELEM_FIELD_STATUS),
      _ch_mask(0),
      _last_emit_us(0),
      _stream_seq(0) {
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
    dispatcher->register_handler(HostCmd::AUTO_TUNE, _handle_auto_tune);
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

void SensorLink::stop() {
    _streaming = false;
    Psoc::getInstance()->set_telemetry_active(false);
    TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
}

void SensorLink::emit_telem_task() {
    getInstance()->tick();
}

void SensorLink::tick() {
    // 周期由 TxScheduler 定时任务驱动; 本函数只负责"发送一帧"(不再自门控频率)。
    if (!_streaming) return;

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
        // FIFO 尚有空间才发本帧(全满则跳过, 让积压/命令响应先泵出); config_write 分段入队非阻塞。
        // 过载(host 消费慢)自然跳帧, 不堆积、不阻塞。
        if (usb->config_write_available() > 0) {
            usb->config_write(_tx_buf, frame_length);
        }
    }
    if (_mode == 1) {
        _streaming = false;
        Psoc::getInstance()->set_telemetry_active(false);
        TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
    }
}

void SensorLink::_handle_telem_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len < 12) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "telem_start payload too short", response, 512);
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
    self->_streaming = true;
    Psoc::getInstance()->set_telemetry_active(true);   // Phase C：开启全通道 raw 快照慢路

    // 注册/续期遥测定时任务(续期制): 周期=1e6/rate。可选 payload[12..13]=lease_ms(u16), 缺省 3s。
    // 之后任意主机命令帧都会经 UsbComm renew_all 续租; 续期超时(上位机丢失)则任务自动停。
    uint32_t lease_ms = TELEM_LEASE_MS;
    if (frame.len >= 14) {
        const uint16_t l = static_cast<uint16_t>(frame.payload[12]) |
                           (static_cast<uint16_t>(frame.payload[13]) << 8);
        if (l != 0) lease_ms = l;
    }
    const uint32_t interval_us = 1000000UL / self->_rate_hz;
    TxScheduler::getInstance()->schedule(TX_TASK_TELEM, interval_us, lease_ms,
                                         &SensorLink::emit_telem_task);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_telem_stop(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    getInstance()->_streaming = false;
    Psoc::getInstance()->set_telemetry_active(false);  // Phase C：关闭快照慢路，回落触控快路
    TxScheduler::getInstance()->cancel(TX_TASK_TELEM);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_param_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) + param_id(u8) + value(u32 LE)
    if (frame.len < 6) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_set payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t param_id = frame.payload[1];
    const uint32_t value = static_cast<uint32_t>(frame.payload[2]) |
                           (static_cast<uint32_t>(frame.payload[3]) << 8) |
                           (static_cast<uint32_t>(frame.payload[4]) << 16) |
                           (static_cast<uint32_t>(frame.payload[5]) << 24);

    // 合法性防护(与 PSoC 端一致): 拒绝会 railed/时钟异常/校准发散的非法值, 不下发也不写真相源。
    // RESOLUTION 6..16; SNS_CLK_DIV 1..255; IDAC_MOD 0..127; IDAC_GAIN 0..7; SNS_CLK_SOURCE 低7位 0..6。
    bool legal = true;
    switch (param_id) {
        case 0x07: legal = (value >= 6u)  && (value <= 16u);  break;   // RESOLUTION
        case 0x08: legal = (value >= 1u)  && (value <= 255u); break;   // SNS_CLK_DIV
        case 0x09: legal = (value <= 127u);                   break;   // IDAC_MOD
        case 0x0B: legal = (value <= 6u);                     break;   // IDAC_GAIN(0..6, 表7项索引7越界崩溃)
        case 0x0A: legal = ((value & 0x7Fu) <= 6u);           break;   // SNS_CLK_SOURCE
        default: break;
    }
    if (!legal) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param value out of legal range (guarded)", response, 512);
        return;
    }

    if (Psoc::getInstance()->set_param(ch, param_id, value)) {
        CsdConfig::getInstance()->note_param(ch, param_id, value);  // 写穿 RP2040 真相源
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC param_set failed", response, 512);
    }
}

void SensorLink::_handle_param_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) + param_id(u8) → 响应 channel + param_id + value(u32 LE)
    if (frame.len < 2) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_get payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t param_id = frame.payload[1];
    uint32_t value = 0;
    if (!Psoc::getInstance()->get_param(ch, param_id, &value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC param_get failed", response, 512);
        return;
    }

    HostFrame resp;
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
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_param_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = channel(u8) → 响应 channel + count + [param_id + value(u32 LE)]×count
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "param_get_all payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];

    HostFrame resp;
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
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_calibrate(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = ch_mask(u64 LE)；当前 PSoC APPLY 对全部 widget 重校准，不细分通道。
    // 真正的 IDAC 重校准(把 railed 的 raw 拉回目标)+ 基线复位; 与 APPLY(仅重配)区分。
    if (Psoc::getInstance()->calibrate()) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC calibrate failed", response, 512);
    }
}

void SensorLink::_handle_baseline_reset(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = ch_mask(u64 LE)(当前 PSoC 对全部通道统一复位基线, 不细分通道)。
    if (Psoc::getInstance()->baseline_reset()) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC baseline reset failed", response, 512);
    }
}

void SensorLink::_handle_auto_tune(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → 频率自适应下探(阻塞至完成)。响应 [result(u8), div(u16 LE)]。
    // result: 1=成功(div 为找到的统一 snsClk 分频, 已写回 PSoC widgetContext), 2=失败(超硬件能力)。
    uint8_t result = 0; uint16_t div = 0;
    if (!Psoc::getInstance()->auto_tune(&result, &div)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC auto_tune failed/timeout", response, 512);
        return;
    }
    // 成功且找到分频 → 写穿 RP2040 真相源(全 36 通道统一 snsClk), 供持久化与回读一致。
    if (result == 1u && div != 0u) {
        for (uint8_t ch = 0; ch < 36u; ch++) {
            CsdConfig::getInstance()->note_param(ch, 0x08u, div);   // 0x08 = PARAM_SNS_CLK_DIV
        }
    }
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::AUTO_TUNE);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = result;
    resp.payload[1] = static_cast<uint8_t>(div & 0xFF);
    resp.payload[2] = static_cast<uint8_t>((div >> 8) & 0xFF);
    resp.len = 3;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_mode_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = mode(u8)：0=自动校准/标准完整处理，非0=半自动手动
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "mode_set payload too short", response, 512);
        return;
    }
    if (Psoc::getInstance()->set_mode(frame.payload[0])) {
        CsdConfig::getInstance()->note_mode(frame.payload[0]);  // 写穿 RP2040 真相源
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC mode_set failed", response, 512);
    }
}

void SensorLink::_handle_csd_capture(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 从 PSoC 读取当前全部参数入 RP2040 store，作为半自动手动模式的起点/种子。
    CsdConfig::getInstance()->capture_from_psoc(Psoc::getInstance());
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_cp_measure(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 0) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "cp_measure payload must be empty", response, 512);
        return;
    }
    if (Psoc::getInstance()->measure_cp()) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC cp_measure failed", response, 512);
    }
}

void SensorLink::_handle_cp_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len != 1 || frame.payload[0] >= SENSOR_LINK_CHANNELS) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "cp_get requires channel 0..35", response, 512);
        return;
    }

    const uint8_t ch = frame.payload[0];
    uint32_t cp_ff = 0;
    if (!Psoc::getInstance()->get_cp(ch, &cp_ff)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC cp_get failed", response, 512);
        return;
    }

    HostFrame resp;
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
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_unsupported(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::NOT_IMPLEMENTED,
        "PSoC mutation is not implemented", response, 512);
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
            "global_get payload too short", response, 512);
        return;
    }
    const uint8_t gid = frame.payload[0];
    uint32_t value = 0;
    if (!Psoc::getInstance()->get_global(gid, &value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC global_get failed", response, 512);
        return;
    }
    HostFrame resp;
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
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_global_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = gparam_id(u8) + value(u32 LE)。写 PSoC RAM 影子 + 写穿 CsdConfig + APPLY 重初始化生效。
    if (frame.len < 5) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global_set payload too short", response, 512);
        return;
    }
    const uint8_t gid = frame.payload[0];
    const uint32_t value = static_cast<uint32_t>(frame.payload[1]) |
                           (static_cast<uint32_t>(frame.payload[2]) << 8) |
                           (static_cast<uint32_t>(frame.payload[3]) << 16) |
                           (static_cast<uint32_t>(frame.payload[4]) << 24);
    // 合法性防护(与 PSoC 端一致): INACTIVE_SNS∈{1,2,4}; IDAC_GAIN_INIT 0..7; IDAC_MIN 0..127;
    // RAW_TARGET 1..99(0/≥100 会让自动校准发散→railed)。MFS 分频不额外限制。
    bool glegal = true;
    switch (gid) {
        case 0x01: glegal = (value == 1u) || (value == 2u) || (value == 4u); break; // INACTIVE_SNS
        case 0x02: glegal = (value <= 6u);                                   break; // IDAC_GAIN_INIT(0..6, 索引7越界崩溃)
        case 0x03: glegal = (value <= 127u);                                 break; // IDAC_MIN
        case 0x04: glegal = (value >= 1u) && (value <= 99u);                 break; // RAW_TARGET
        case 0x07: glegal = (value <= 1u);                                   break; // IDAC_SENSE_CONFIG(0=sourcing,1=sinking)
        case 0x08: glegal = (value <= 1u);                                   break; // AUTO_CALIBRATE_EN(0/1)
        default: break;
    }
    if (!glegal) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "global value out of legal range (guarded)", response, 512);
        return;
    }
    Psoc* psoc = Psoc::getInstance();
    if (!psoc->set_global(gid, value)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC global_set failed", response, 512);
        return;
    }
    CsdConfig::getInstance()->note_global(gid, value);   // 写穿 RP2040 真相源
    psoc->global_commit();   // 一次完整重初始化生效(合并, 防反复重校准漂移)
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_global_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空 → [count(u8), (gparam_id, value u32 LE)×count]
    Psoc* psoc = Psoc::getInstance();
    HostFrame resp;
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
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

// ---- JIT 算法引擎命令 ----
void SensorLink::_handle_algo_get_info(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → [is_default(u8), psoc_valid(u8), len(u16 LE), crc16(u16 LE)]
    PsocAlgo* store = PsocAlgo::getInstance();
    bool psoc_valid = false;
    uint16_t psoc_len = 0;
    Psoc::getInstance()->get_algo_info(&psoc_valid, &psoc_len);   // 失败则 valid=false

    HostFrame resp;
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
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_upload(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = len(u16 LE) + crc16(u16 LE) + data[len]
    if (frame.len < 4) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_upload header too short", response, 512);
        return;
    }
    const uint16_t len = static_cast<uint16_t>(frame.payload[0]) |
                         (static_cast<uint16_t>(frame.payload[1]) << 8);
    const uint16_t crc16 = static_cast<uint16_t>(frame.payload[2]) |
                           (static_cast<uint16_t>(frame.payload[3]) << 8);
    if (len == 0u || len > PSOC_ALGO_MAX_LEN || (uint32_t)frame.len < 4u + (uint32_t)len) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_upload len invalid", response, 512);
        return;
    }
    PsocAlgo* store = PsocAlgo::getInstance();
    if (!store->set_algo(&frame.payload[4], len, crc16)) {   // 校验 crc16 一致才接受+持久化
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo crc16 mismatch", response, 512);
        return;
    }
    if (!store->download_to_psoc(Psoc::getInstance())) {     // 立即下发 PSoC + commit 校验
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "algo download to PSoC failed", response, 512);
        return;
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_apply(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (PsocAlgo::getInstance()->download_to_psoc(Psoc::getInstance())) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "algo apply failed", response, 512);
    }
}

void SensorLink::_handle_algo_reset_default(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    PsocAlgo* store = PsocAlgo::getInstance();
    store->reset_default();                          // 回退内嵌默认 + 请求持久化
    store->download_to_psoc(Psoc::getInstance());    // 立即下发默认(失败也回 ACK, 启动会重推)
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_set_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), rom_lo, rom_hi] × count (count = len/3)
    if (frame.len == 0u || (frame.len % 3u) != 0u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_rom payload must be 3*N bytes", response, 512);
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
                "algo_set_rom channel out of range", response, 512);
            return;
        }
        store->set_rom(ch, rom);           // 更新 RP 存储 + 请求持久化(真相源)
        psoc->set_algo_rom(ch, rom);       // 立即下发 PSoC
    }
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_get_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → 响应 36×u16 LE (RP 存储的每通道 ROM 表, 真相源)
    PsocAlgo* store = PsocAlgo::getInstance();
    HostFrame resp;
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
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_get_trace(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [ch(u8), idx(u8)] → 响应 [ch, out_active(u8), report(u16 LE)]
    if (frame.len < 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_trace payload too short", response, 512);
        return;
    }
    const uint8_t ch = frame.payload[0];
    const uint8_t idx = frame.payload[1];
    if (ch >= PSOC_ALGO_CHANNELS || idx >= 4u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_trace ch/idx out of range", response, 512);
        return;
    }
    uint8_t active = 0;
    uint16_t report = 0;
    if (!Psoc::getInstance()->algo_get_trace(ch, idx, &active, &report)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC algo_get_trace failed", response, 512);
        return;
    }
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_TRACE);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = ch;
    resp.payload[1] = active;
    resp.payload[2] = static_cast<uint8_t>(report);
    resp.payload[3] = static_cast<uint8_t>(report >> 8);
    resp.len = 4;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_set_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [idx(u8), val(u8)]。写 PSoC 共享 cfg[idx] + PsocAlgo 持久化(随算法下发恢复)。
    if (frame.len < 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_cfg payload too short", response, 512);
        return;
    }
    const uint8_t idx = frame.payload[0];
    const uint8_t val = frame.payload[1];
    if (idx >= 8u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_cfg idx out of range", response, 512);
        return;
    }
    if (!Psoc::getInstance()->algo_set_cfg(idx, val)) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC algo_set_cfg failed", response, 512);
        return;
    }
    PsocAlgo::getInstance()->set_cfg(idx, val);   // 更新 RP 存储 + 请求持久化(真相源)
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_get_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [idx(u8)] → 响应 [idx, cfg(u8)]
    if (frame.len < 1u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_cfg payload too short", response, 512);
        return;
    }
    const uint8_t idx = frame.payload[0];
    if (idx >= 8u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_get_cfg idx out of range", response, 512);
        return;
    }
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_CFG);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = idx;
    resp.payload[1] = PsocAlgo::getInstance()->cfg(idx);   // 直读 RP 存储的真相源(不下发 PSoC 查询)
    resp.len = 2;
    *response_length = HostCmdCodec::encode_frame(resp, response, 512);
}

void SensorLink::_handle_algo_get_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → [len(u16 LE), src bytes] RP2040 存的算法 C 源(映射表), 供回读还原可编辑 C。
    PsocAlgo* store = PsocAlgo::getInstance();
    const uint16_t n = store->src_len();
    HostFrame resp;
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::ALGO_GET_SRC);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.payload[0] = static_cast<uint8_t>(n);
    resp.payload[1] = static_cast<uint8_t>(n >> 8);
    if (n > 0u) { memcpy(&resp.payload[2], store->src(), n); }
    resp.len = static_cast<uint16_t>(2u + n);
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void SensorLink::_handle_algo_set_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = [len(u16 LE), src bytes]; 存算法 C 源(已由上位机滤注释)+持久化。
    if (frame.len < 2u) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src header too short", response, 512);
        return;
    }
    const uint16_t n = static_cast<uint16_t>(frame.payload[0]) |
                       (static_cast<uint16_t>(frame.payload[1]) << 8);
    if (n > PSOC_ALGO_SRC_MAX || (uint32_t)frame.len < 2u + (uint32_t)n) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "algo_set_src len invalid", response, 512);
        return;
    }
    PsocAlgo::getInstance()->set_src(&frame.payload[2], n);
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_algo_get_code(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // 空请求 → [len(u16 LE), asm bytes] RP2040 存的算法 ASM 机器码回读(供反汇编查看)。
    PsocAlgo* store = PsocAlgo::getInstance();
    const uint16_t n = store->len();
    HostFrame resp;
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
