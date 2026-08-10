#include "binding_service.h"
#include "../../hal/usb/hal_usb.h"
#include "../config_manager/config_manager.h"
#include <cstdio>
#include <cstring>

BindingService* BindingService::_instance = nullptr;

BindingService::BindingService()
    : _state(State::IDLE), _active_zone(0), _bind_event_pending(false),
      _bind_event_zone(0), _bind_event_channel(0xFF), _bind_event_status(0) {
    for (uint8_t z = 0; z < ZONE_COUNT; z++) _bind_ch[z] = 0xFF;
}

BindingService* BindingService::getInstance() {
    if (_instance == nullptr) _instance = new BindingService();
    return _instance;
}

void BindingService::init() {
    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::BIND_START, _handle_bind_start);
    dispatcher->register_handler(HostCmd::BIND_ABORT, _handle_bind_abort);
    dispatcher->register_handler(HostCmd::BIND_GET_MAP, _handle_bind_get_map);
    dispatcher->register_handler(HostCmd::BIND_SET_MAP, _handle_bind_set_map);
    reload_binding();
}

void BindingService::reload_binding() {
    char key_buf[32];
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        snprintf(key_buf, sizeof(key_buf), "bind.map%02u", z);
        const uint32_t value = ConfigManager::get_uint32(key_buf);
        if (value == 0xFFFFFFFFu || value > 35u) {
            _bind_ch[z] = 0xFF;
        } else {
            _bind_ch[z] = static_cast<uint8_t>(value & 0xFF);
        }
    }
}

uint64_t BindingService::map_to_areas(uint64_t touch_mask) const {
    uint64_t area_bits = 0;
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        const uint8_t ch = _bind_ch[z];
        if (ch < 36 && ((touch_mask >> ch) & 1ULL) != 0) {
            area_bits |= (uint64_t{1} << z);
        }
    }
    return area_bits;
}

void BindingService::tick(uint64_t touch_mask, bool link_ok) {
    // 先重试之前因命令响应占用 vendor TX 而暂存的主动事件。
    _try_emit_bind_event();
    if (_state != State::WAIT_TOUCH) return;
    if (!link_ok || touch_mask == 0) return;

    const uint8_t ch = static_cast<uint8_t>(__builtin_ctzll(touch_mask));
    _complete_bind(_active_zone, ch);
}

void BindingService::_complete_bind(uint8_t zone, uint8_t channel) {
    char key_buf[32];
    snprintf(key_buf, sizeof(key_buf), "bind.map%02u", zone);
    ConfigManager::set_uint32(key_buf, channel);
    reload_binding();
    _emit_bind_event(zone, channel, 1);
    _state = State::IDLE;
}

void BindingService::_emit_bind_event(uint8_t zone, uint8_t channel, uint8_t status) {
    // BIND_EVENT 由 tick() 主动发送，可能发生在 dispatch() 之外，不能与响应共享帧重叠。
    // 若当前响应占用 vendor TX，先保留事件，下一轮 tick 重试，不能静默丢回传。
    if (_bind_event_pending) return;
    _bind_event_pending = true;
    _bind_event_zone = zone;
    _bind_event_channel = channel;
    _bind_event_status = status;
    _try_emit_bind_event();
}

void BindingService::_try_emit_bind_event() {
    if (!_bind_event_pending) return;
    HAL_USB_Device* usb = HAL_USB_Device::getInstance();
    if (!usb->is_ready() || usb->config_write_available() == 0) return;

    static HostFrame frame;
    frame.clear();
    frame.cmd = static_cast<uint8_t>(HostCmd::BIND_EVENT);
    frame.flags = 0;
    frame.seq = 0;
    frame.payload[0] = _bind_event_zone;
    frame.payload[1] = _bind_event_channel;
    frame.payload[2] = _bind_event_status;
    frame.len = 3;

    uint8_t tx_buf[32];
    const uint16_t frame_length = HostCmdCodec::encode_frame(frame, tx_buf, sizeof(tx_buf));
    if (frame_length > 0 && usb->config_write(tx_buf, frame_length)) {
        _bind_event_pending = false;
    }
}

void BindingService::_handle_bind_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len < 1) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "bind_start payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t zone = frame.payload[0];
    if (zone >= ZONE_COUNT) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "bind_start zone out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    BindingService* self = getInstance();
    self->_active_zone = zone;
    self->_state = State::WAIT_TOUCH;
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void BindingService::_handle_bind_abort(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    getInstance()->_state = State::IDLE;
    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}

void BindingService::_handle_bind_get_map(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    BindingService* self = getInstance();

    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.clear();
    resp.cmd = static_cast<uint8_t>(HostCmd::BIND_GET_MAP);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.len = ZONE_COUNT;
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        resp.payload[z] = self->_bind_ch[z];
    }
    *response_length = HostCmdCodec::encode_frame(resp, response, HOST_CMD_RESP_BUF_MAX);
}

void BindingService::_handle_bind_set_map(const HostFrame& frame, uint8_t* response, uint16_t* response_length) {
    if (frame.len < 2) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "bind_set_map payload too short", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t zone = frame.payload[0];
    const uint8_t channel = frame.payload[1];
    if (zone >= ZONE_COUNT) {
        *response_length = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "bind_set_map zone out of range", response, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    char key_buf[32];
    snprintf(key_buf, sizeof(key_buf), "bind.map%02u", zone);
    const uint32_t value = (channel >= 36) ? 0xFFFFFFFFu : static_cast<uint32_t>(channel);
    ConfigManager::set_uint32(key_buf, value);
    getInstance()->reload_binding();

    *response_length = HostCmdCodec::encode_ack(frame.seq, response, HOST_CMD_RESP_BUF_MAX);
}
