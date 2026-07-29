#include "keyboard.h"
#include "../config_manager/config_manager.h"
#include "../binding_service/binding_service.h"
#include "../../protocol/hid/hid.h"
#include "../../protocol/psoc/psoc.h"
#include "../../hal/usb/hal_usb.h"
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <cstdio>

KeyboardService* KeyboardService::_instance = nullptr;

KeyboardService::KeyboardService()
    : _kbd_map_en(false), _phys_state(0), _phys_out(0), _raw_last(0), _raw_stable_since_us(0),
      _touch_active(0), _gpio_ready(false) {
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        _keycode[i] = 0; _keymod[i] = 0;
        _hold[i].clear(); _hold_st[i].clear();
    }
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        _zone_keycode[z] = 0; _zone_mod[z] = 0;
        _zone_hold[z].clear(); _zone_hold_st[z].clear();
    }
}

void KeyboardService::_apply_mods(uint8_t mod, bool pressed) {
    HID* hid = HID::getInstance();
    for (uint8_t b = 0; b < 4; b++) {
        if ((mod >> b) & 1u) {
            const HID_KeyCode k = static_cast<HID_KeyCode>(0xE0u + b);  // LCtrl/LShift/LAlt/LGui
            if (pressed) hid->press_key(k); else hid->release_key(k);
        }
    }
}

KeyboardService* KeyboardService::getInstance() {
    if (_instance == nullptr) _instance = new KeyboardService();
    return _instance;
}

void KeyboardService::init() {
    // GPIO1-12: 输入 + 上拉(硬件已有 1K, 内部上拉冗余但无害), active-low。
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        const uint8_t pin = GPIO_BASE + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }
    _gpio_ready = true;

    reload_map();

    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::KBD_GET_STATE, _handle_get_state);
    dispatcher->register_handler(HostCmd::KBD_GET_MAP, _handle_get_map);
    dispatcher->register_handler(HostCmd::KBD_SET_MAP, _handle_set_map);
    dispatcher->register_handler(HostCmd::KBD_GET_TOUCHMAP, _handle_get_touchmap);
    dispatcher->register_handler(HostCmd::KBD_SET_TOUCHMAP, _handle_set_touchmap);
    dispatcher->register_handler(HostCmd::KBD_GET_HOLD, _handle_get_hold);
    dispatcher->register_handler(HostCmd::KBD_SET_HOLD, _handle_set_hold);
}

void KeyboardService::reload_map() {
    char key_buf[16];
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        snprintf(key_buf, sizeof(key_buf), "kbd.key%02u", i);
        _keycode[i] = ConfigManager::get_uint8(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.km%02u", i);
        _keymod[i] = ConfigManager::get_uint8(key_buf);
        // 长按参数: ConfigManager 存 uint16, 语义上限 65535ms。
        snprintf(key_buf, sizeof(key_buf), "kbd.hd%02u", i);
        _hold[i].delay_ms = ConfigManager::get_uint16(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.mh%02u", i);
        _hold[i].max_hold_ms = ConfigManager::get_uint16(key_buf);
    }
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        snprintf(key_buf, sizeof(key_buf), "kbd.zone%02u", z);
        _zone_keycode[z] = ConfigManager::get_uint8(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.zm%02u", z);
        _zone_mod[z] = ConfigManager::get_uint8(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.zhd%02u", z);
        _zone_hold[z].delay_ms = ConfigManager::get_uint16(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.zmh%02u", z);
        _zone_hold[z].max_hold_ms = ConfigManager::get_uint16(key_buf);
    }
    _kbd_map_en = ConfigManager::get_bool("comm.keyboard_map_en");
}

uint16_t KeyboardService::_read_raw() const {
    uint16_t raw = 0;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        // 上拉 active-low: 低电平=按下。
        if (gpio_get(GPIO_BASE + i) == 0) raw |= (uint16_t)(1u << i);
    }
    return raw;
}

void KeyboardService::_apply_phys(uint8_t idx, bool pressed) {
    const uint8_t code = _keycode[idx];
    const uint8_t mod = _keymod[idx];
    if (code == 0 && mod == 0) return;
    HID* hid = HID::getInstance();
    if (pressed) {
        _apply_mods(mod, true);
        if (code != 0) hid->press_key(static_cast<HID_KeyCode>(code));
    } else {
        if (code != 0) hid->release_key(static_cast<HID_KeyCode>(code));
        _apply_mods(mod, false);
    }
}

void KeyboardService::_apply_touch(uint64_t area_mask) {
    const uint64_t changed = area_mask ^ _touch_active;
    if (changed == 0) return;
    HID* hid = HID::getInstance();
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        if (((changed >> z) & 1ULL) == 0) continue;
        const uint8_t code = _zone_keycode[z];
        const uint8_t mod = _zone_mod[z];
        if (code == 0 && mod == 0) continue;
        const bool pressed = ((area_mask >> z) & 1ULL) != 0;
        if (pressed) {
            _apply_mods(mod, true);
            if (code != 0) hid->press_key(static_cast<HID_KeyCode>(code));
        } else {
            if (code != 0) hid->release_key(static_cast<HID_KeyCode>(code));
            _apply_mods(mod, false);
        }
    }
    _touch_active = area_mask;
}

void KeyboardService::task() {
    if (!_gpio_ready) return;

    // 物理键整字去抖: 原始采样稳定 DEBOUNCE_US 后一次性提交变化位。
    const uint32_t now = time_us_32();
    const uint16_t raw = _read_raw();
    if (raw != _raw_last) {
        _raw_last = raw;
        _raw_stable_since_us = now;
    }
    if (raw != _phys_state && (now - _raw_stable_since_us) >= DEBOUNCE_US) {
        _phys_state = raw;
    }

    // 去抖后的按下态再过一遍每键长按状态机(延迟触发 / 最长按住自动抬起), 差分驱动 HID。
    uint16_t phys_out = 0;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (_hold_eval(_hold_st[i], _hold[i], ((_phys_state >> i) & 1u) != 0, now)) {
            phys_out |= (uint16_t)(1u << i);
        }
    }
    if (phys_out != _phys_out) {
        const uint16_t changed = phys_out ^ _phys_out;
        for (uint8_t i = 0; i < KEY_COUNT; i++) {
            if (((changed >> i) & 1u) != 0) {
                _apply_phys(i, ((phys_out >> i) & 1u) != 0);
            }
        }
        _phys_out = phys_out;
    }

    // 触控→键盘: 开启时按当前分区触摸驱动(同样过长按状态机); 关闭时按"全松开"走同一路径确保释放。
    uint64_t area_raw = 0;
    if (_kbd_map_en) {
        Psoc* psoc = Psoc::getInstance();
        if (psoc->link_ok()) {
            area_raw = BindingService::getInstance()->map_to_areas(psoc->touch_mask());
        }
    }
    uint64_t area_out = 0;
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        if (_hold_eval(_zone_hold_st[z], _zone_hold[z], ((area_raw >> z) & 1ULL) != 0, now)) {
            area_out |= (1ULL << z);
        }
    }
    _apply_touch(area_out);

    HID::getInstance()->task();
}

void KeyboardService::_handle_get_state(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    HostFrame r;
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_STATE);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    r.len = 2;
    r.payload[0] = (uint8_t)(self->_phys_state & 0xFF);
    r.payload[1] = (uint8_t)((self->_phys_state >> 8) & 0xFF);
    *resp_len = HostCmdCodec::encode_frame(r, resp, 512);
}

void KeyboardService::_handle_get_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    HostFrame r;
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_MAP);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // 每键 2 字节: [keycode, modifier]。
    r.payload[0] = KEY_COUNT;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        r.payload[1 + i * 2] = self->_keycode[i];
        r.payload[2 + i * 2] = self->_keymod[i];
    }
    r.len = 1 + KEY_COUNT * 2;
    *resp_len = HostCmdCodec::encode_frame(r, resp, 512);
}

void KeyboardService::_handle_set_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // 每项 3 字节: [idx, keycode, modifier]。
    if (frame.len < 3 || (frame.len % 3) != 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_map payload must be [idx,keycode,mod] triples", resp, 512);
        return;
    }
    char key_buf[16];
    for (uint16_t off = 0; off + 2 < frame.len; off += 3) {
        const uint8_t idx = frame.payload[off];
        const uint8_t code = frame.payload[off + 1];
        const uint8_t mod = frame.payload[off + 2];
        if (idx >= KEY_COUNT) continue;
        snprintf(key_buf, sizeof(key_buf), "kbd.key%02u", idx);
        ConfigManager::set_uint8(key_buf, code);
        snprintf(key_buf, sizeof(key_buf), "kbd.km%02u", idx);
        ConfigManager::set_uint8(key_buf, mod);
    }
    // 只写 RAM 影子并即时下发生效; flash 落地统一由 SAVE_CONFIG 触发(保护 flash 寿命)。
    getInstance()->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, 512);
}

void KeyboardService::_handle_get_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    HostFrame r;
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_HOLD);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // [phys_count, zone_count] + 12×(delay u16 LE, maxhold u16 LE) + 34×(同上)。
    r.payload[0] = KEY_COUNT;
    r.payload[1] = ZONE_COUNT;
    uint16_t off = 2;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        r.payload[off++] = (uint8_t)(self->_hold[i].delay_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_hold[i].delay_ms >> 8) & 0xFF);
        r.payload[off++] = (uint8_t)(self->_hold[i].max_hold_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_hold[i].max_hold_ms >> 8) & 0xFF);
    }
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        r.payload[off++] = (uint8_t)(self->_zone_hold[z].delay_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_zone_hold[z].delay_ms >> 8) & 0xFF);
        r.payload[off++] = (uint8_t)(self->_zone_hold[z].max_hold_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_zone_hold[z].max_hold_ms >> 8) & 0xFF);
    }
    r.len = off;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_set_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // 每项 6 字节: [kind(0=物理键/1=分区), idx, delay_ms(u16 LE), maxhold_ms(u16 LE)]。
    if (frame.len < 6 || (frame.len % 6) != 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_hold payload must be [kind,idx,delay16,maxhold16] sextets", resp, 512);
        return;
    }
    char key_buf[16];
    for (uint16_t off = 0; off + 5 < frame.len; off += 6) {
        const uint8_t kind = frame.payload[off];
        const uint8_t idx = frame.payload[off + 1];
        const uint16_t delay_ms = (uint16_t)(frame.payload[off + 2] | (frame.payload[off + 3] << 8));
        const uint16_t hold_ms = (uint16_t)(frame.payload[off + 4] | (frame.payload[off + 5] << 8));
        if (kind == 0) {
            if (idx >= KEY_COUNT) continue;
            snprintf(key_buf, sizeof(key_buf), "kbd.hd%02u", idx);
            ConfigManager::set_uint16(key_buf, delay_ms);
            snprintf(key_buf, sizeof(key_buf), "kbd.mh%02u", idx);
            ConfigManager::set_uint16(key_buf, hold_ms);
        } else {
            if (idx >= ZONE_COUNT) continue;
            snprintf(key_buf, sizeof(key_buf), "kbd.zhd%02u", idx);
            ConfigManager::set_uint16(key_buf, delay_ms);
            snprintf(key_buf, sizeof(key_buf), "kbd.zmh%02u", idx);
            ConfigManager::set_uint16(key_buf, hold_ms);
        }
    }
    // 只写 RAM 影子并即时下发生效; flash 落地统一由 SAVE_CONFIG 触发(保护 flash 寿命)。
    getInstance()->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, 512);
}

void KeyboardService::_handle_get_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    HostFrame r;
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_TOUCHMAP);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // 每分区 2 字节: [keycode, modifier]。
    r.payload[0] = self->_kbd_map_en ? 1 : 0;
    r.payload[1] = ZONE_COUNT;
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        r.payload[2 + z * 2] = self->_zone_keycode[z];
        r.payload[3 + z * 2] = self->_zone_mod[z];
    }
    r.len = 2 + ZONE_COUNT * 2;
    *resp_len = HostCmdCodec::encode_frame(r, resp, 512);
}

void KeyboardService::_handle_set_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // 每项 3 字节: [zone, keycode, modifier]。
    if (frame.len < 3 || (frame.len % 3) != 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_touchmap payload must be [zone,keycode,mod] triples", resp, 512);
        return;
    }
    char key_buf[16];
    for (uint16_t off = 0; off + 2 < frame.len; off += 3) {
        const uint8_t zone = frame.payload[off];
        const uint8_t code = frame.payload[off + 1];
        const uint8_t mod = frame.payload[off + 2];
        if (zone >= ZONE_COUNT) continue;
        snprintf(key_buf, sizeof(key_buf), "kbd.zone%02u", zone);
        ConfigManager::set_uint8(key_buf, code);
        snprintf(key_buf, sizeof(key_buf), "kbd.zm%02u", zone);
        ConfigManager::set_uint8(key_buf, mod);
    }
    // 只写 RAM 影子并即时下发生效; flash 落地统一由 SAVE_CONFIG 触发(保护 flash 寿命)。
    getInstance()->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, 512);
}
