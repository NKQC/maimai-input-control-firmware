#include "hid.h"
#include "src/hal/usb/hal_usb.h"
#include <cstring>
#include <pico/time.h>

// 单例实例
HID* HID::instance_ = nullptr;

// 私有构造函数
HID::HID() 
    : initialized_(false), hal_usb_(nullptr), report_count_(0), last_report_time_(0), cached_report_rate_(0) {
    // 按键状态现在由HAL层直接管理
}

// 析构函数
HID::~HID() {
    deinit();
}

// 获取单例实例
HID* HID::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HID();
    }
    return instance_;
}

// 初始化
bool HID::init(HAL_USB* hal_usb) {
    if (initialized_ || !hal_usb) {
        return false;
    }
    
    hal_usb_ = hal_usb;
    report_count_ = 0;
    last_report_time_ = to_ms_since_boot(get_absolute_time());
    cached_report_rate_ = 0;
    
    // 按键状态现在由HAL层直接管理
    
    initialized_ = true;
    return true;
}

// 反初始化
void HID::deinit() {
    if (!initialized_) {
        return;
    }
    
    initialized_ = false;
    hal_usb_ = nullptr;
    report_count_ = 0;
    last_report_time_ = 0;
    cached_report_rate_ = 0;
}

// 检查是否已初始化
bool HID::is_initialized() const {
    return initialized_;
}

// 按下按键
bool HID::press_key(HID_KeyCode key) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    return keyboard_state.add(key);
}

// 释放按键
bool HID::release_key(HID_KeyCode key) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    return keyboard_state.remove(key);
}

// 获取实际回报速率
uint8_t HID::get_report_rate() const {
    return cached_report_rate_;
}

// 触摸报告发送 (空实现，保持向后兼容)
bool HID::send_touch_report(const HID_TouchPoint& report) {
    return report.press ? touch_state.press(report) : touch_state.release(report.id);
}

void HID::report_keyboard() {
    static HID_KeyCode keyboard_report[8] = {HID_KeyCode::KEY_NONE, HID_KeyCode::KEY_NONE};
    static uint8_t keyboard_report_num = 0;
    static uint8_t keyboard_enum = 0;
    keyboard_enum = 0;
    keyboard_report_num = 2;
    for (uint8_t i = 0; keyboard_state.modifier && i < KEYBOARD_SIMUL_PRESS; i++) {
        if (keyboard_state.key[i] != HID_KeyCode::KEY_NONE) {
            keyboard_report[keyboard_report_num++] = keyboard_state.key[i];
            keyboard_state.modifier--;
        }
        if (!keyboard_state.modifier || keyboard_report_num == 8) {
            for (uint8_t c = keyboard_report_num; c < 8; c++) {
                keyboard_report[c] = HID_KeyCode::KEY_NONE;
            }
            hal_usb_->send_hid_report(keyboard_id[keyboard_enum], (uint8_t*)keyboard_report, 8);
            keyboard_report_num = 0;
            keyboard_enum++;
        }
    }
}

void HID::report_touch(uint32_t _now) {
    static uint8_t TouchData[9];
    static HID_TouchPoint report;
    // 先处理按下的
    for (uint8_t i = 0; i < touch_state.press_modifier; i++) {
        report = touch_state.touch_press[i];
        TouchData[0] = touch_state.press_modifier;   // Press
        TouchData[1] = report.id;                    // Touch_id
        TouchData[2] = report.x & 0xFF;              // X lsb
        TouchData[3] = report.x >> 8 & 0xFF;         // X msb
        TouchData[4] = report.y & 0xFF;              // Y lsb
        TouchData[5] = report.y >> 8 & 0xFF;         // Y msb
        TouchData[6] = (_now * 10) & 0xFF;        // Scan time lsb
        TouchData[7] = ((_now * 10) >> 8) & 0xFF; // Scan time msb
        TouchData[8] = touch_state.press_modifier > 1;
        hal_usb_->send_hid_report(HID_ReportID::REPORT_ID_TOUCHSCREEN, (uint8_t*)TouchData, 9);
    }
    
    // 处理松开的 同时销毁记录
    for (uint8_t i = 0; i < touch_state.release_modifier; i++) {
        HID_TouchPoint report = touch_state.touch_release[i];
        TouchData[0] = 0;                   // Press
        TouchData[1] = report.id;           // Touch_id
        TouchData[2] = 0;                   // X lsb
        TouchData[3] = 0;                   // X msb
        TouchData[4] = 0;                   // Y lsb
        TouchData[5] = 0;                   // Y msb
        TouchData[6] = 0;                   // Scan time lsb
        TouchData[7] = 0;                   // Scan time msb
        TouchData[8] = touch_state.release_modifier > 1;
        hal_usb_->send_hid_report(HID_ReportID::REPORT_ID_TOUCHSCREEN, (uint8_t*)TouchData, 9);
        touch_state.remove(report.id);
    }
}

void HID::task() {
    static uint32_t _now = 0;
    _now = us_to_ms(time_us_64());
    report_keyboard();
    report_touch(_now);
    report_count_++;
    if (_now - last_report_time_ >= 1000000) {
        cached_report_rate_ = report_count_;
        report_count_ = 0;
        last_report_time_ = _now;
    }
}
