#include "sensor_link.h"
#include "../../hal/usb/hal_usb.h"
#include "../../protocol/psoc/psoc.h"
#include "../csd_config/csd_config.h"
#include "../latency_stats.h"
#include <pico/stdlib.h>
#include <cstring>

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
    dispatcher->register_handler(HostCmd::BASELINE_RESET, _handle_unsupported);
    dispatcher->register_handler(HostCmd::MODE_SET, _handle_mode_set);
    dispatcher->register_handler(HostCmd::CSD_CAPTURE, _handle_csd_capture);
    dispatcher->register_handler(HostCmd::CP_MEASURE, _handle_cp_measure);
    dispatcher->register_handler(HostCmd::CP_GET, _handle_cp_get);
}

void SensorLink::stop() {
    _streaming = false;
    Psoc::getInstance()->set_telemetry_active(false);
}

void SensorLink::tick() {
    if (!_streaming || _rate_hz == 0) return;

    const uint32_t now_us = time_us_32();
    const uint32_t interval_us = 1000000UL / _rate_hz;
    if (_last_emit_us != 0 && (now_us - _last_emit_us) < interval_us) return;
    _last_emit_us = now_us;

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
        HAL_USB_Device::getInstance()->config_write(_tx_buf, frame_length);
    }
    if (_mode == 1) _streaming = false;
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
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
}

void SensorLink::_handle_telem_stop(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    getInstance()->_streaming = false;
    Psoc::getInstance()->set_telemetry_active(false);  // Phase C：关闭快照慢路，回落触控快路
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
    if (Psoc::getInstance()->apply_params()) {
        *response_length = HostCmdCodec::encode_ack(frame.seq, response, 512);
    } else {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::SENSOR_ERROR,
            "PSoC calibrate/apply failed", response, 512);
    }
}

void SensorLink::_handle_mode_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    // payload = mode(u8)：0=全自动 SmartSense, 非0=半自动/手动
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
    // 从 PSoC 读取当前(自整定)全部参数入 RP2040 store，作为半自动模式的手动起点/种子。
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
