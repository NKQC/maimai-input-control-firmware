#include "hid.h"
#include "src/hal/usb/hal_usb.h"
#include <cstring>
#include <pico/time.h>

// 单例实例
HID* HID::instance_ = nullptr;

// 私有构造函数
HID::HID() 
    : initialized_(false), hal_usb_(nullptr), report_count_(0), last_report_time_(0), cached_report_rate_(0),
      keyboard_needs_send_(false), touch_needs_send_(false), last_keyboard_send_(0), last_touch_send_(0) {
    keyboard_state.clear();
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
    bool result = keyboard_state.add(key);
    if (result && keyboard_state.has_state_changed()) {
        keyboard_needs_send_ = true;
    }
    return result;
}

// 释放按键
bool HID::release_key(HID_KeyCode key) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    bool result = keyboard_state.remove(key);
    if (result && keyboard_state.has_state_changed()) {
        keyboard_needs_send_ = true;
    }
    return result;
}

// 清空键盘状态并发送空报文
void HID::clear_keyboard_state() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    keyboard_state.clear();
    keyboard_needs_send_ = true;
    
    // 立即发送空报文确保主机知道所有按键都已释放
    force_send_keyboard_report();
}

// 强制发送键盘报文
void HID::force_send_keyboard_report() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    report_keyboard();
    keyboard_needs_send_ = false;
    keyboard_state.update_last_state();
    last_keyboard_send_ = us_to_ms(time_us_64());
}

// 获取实际回报速率
uint8_t HID::get_report_rate() const {
    return cached_report_rate_;
}

// 触摸报告发送
bool HID::send_touch_report(const HID_TouchPoint& report) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    
    bool result = false;
    if (report.press) {
        // 按下触摸点
        result = touch_state.press(report);
    } else {
        // 松开触摸点
        result = touch_state.release(report.id);
    }
    
    if (result) {
        touch_needs_send_ = true;
    }
    
    return result;
}

// 强制发送触摸报文
void HID::force_send_touch_report() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    uint32_t _now = us_to_ms(time_us_64());
    report_touch(_now);
    touch_needs_send_ = false;
    last_touch_send_ = _now;
}

void HID::report_keyboard() {
    // HID键盘报文格式: [modifier][reserved][key1][key2][key3][key4][key5][key6]
    // 总共8字节，符合标准HID键盘报文格式
    static uint8_t keyboard_report[8];
    
    uint8_t keyboard_enum = 0;
    uint8_t keys_to_send = keyboard_state.key_count;
    uint8_t key_index = 0;
    
    // 如果没有按键按下且没有修饰键，发送空报文
    if (keyboard_state.key_count == 0 && keyboard_state.modifier_keys == 0) {
        memset(keyboard_report, 0, 8);
        hal_usb_->send_hid_report(keyboard_id[0], keyboard_report, 8);
        return;
    }
    
    // 分批发送按键，每个报文最多6个按键
    do {
        // 清空报文缓冲区
        memset(keyboard_report, 0, 8);
        
        // 第0字节：修饰键状态
        keyboard_report[0] = keyboard_state.modifier_keys;
        // 第1字节：保留字节
        keyboard_report[1] = 0;
        
        // 第2-7字节：最多6个按键码，避免发送0x00按键
        uint8_t report_key_count = 0;
        for (uint8_t i = 0; i < 6 && key_index < keyboard_state.key_count; i++) {
            if (keyboard_state.key[key_index] != HID_KeyCode::KEY_NONE && 
                static_cast<uint8_t>(keyboard_state.key[key_index]) != 0x00) {
                keyboard_report[2 + i] = static_cast<uint8_t>(keyboard_state.key[key_index]);
                report_key_count++;
            }
            key_index++;
        }
        
        // 发送报文
        hal_usb_->send_hid_report(keyboard_id[keyboard_enum], keyboard_report, 8);
        
        keyboard_enum++;
        keys_to_send -= report_key_count;
        
    } while (keys_to_send > 0 && keyboard_enum < KEYBOARD_NUM);
    
    // 如果只有修饰键没有普通按键，确保至少发送一次报文
    if (keyboard_state.key_count == 0 && keyboard_state.modifier_keys != 0 && keyboard_enum == 0) {
        memset(keyboard_report, 0, 8);
        keyboard_report[0] = keyboard_state.modifier_keys;
        hal_usb_->send_hid_report(keyboard_id[0], keyboard_report, 8);
    }
}

void HID::report_touch(uint32_t _now) {
    static uint8_t TouchData[9];
    bool has_sent_report = false;
    
    // 处理按下的触摸点
    for (uint8_t i = 0; i < touch_state.press_modifier; i++) {
        const HID_TouchPoint& report = touch_state.touch_press[i];
        
        // 构建触摸报文
        TouchData[0] = 1;                           // Press状态 (1=按下)
        TouchData[1] = report.id;                   // 触摸点ID
        TouchData[2] = report.x & 0xFF;             // X坐标低字节
        TouchData[3] = (report.x >> 8) & 0xFF;      // X坐标高字节
        TouchData[4] = report.y & 0xFF;             // Y坐标低字节
        TouchData[5] = (report.y >> 8) & 0xFF;      // Y坐标高字节
        TouchData[6] = (_now * 10) & 0xFF;          // 扫描时间低字节
        TouchData[7] = ((_now * 10) >> 8) & 0xFF;   // 扫描时间高字节
        TouchData[8] = (touch_state.press_modifier > 1) ? 1 : 0; // 多点触摸标志
        
        hal_usb_->send_hid_report(HID_ReportID::REPORT_ID_TOUCHSCREEN, TouchData, 9);
        has_sent_report = true;
    }
    
    // 处理松开的触摸点
    for (uint8_t i = 0; i < touch_state.release_modifier; i++) {
        const HID_TouchPoint& report = touch_state.touch_release[i];
        
        // 构建触摸松开报文
        TouchData[0] = 0;                           // Press状态 (0=松开)
        TouchData[1] = report.id;                   // 触摸点ID
        TouchData[2] = 0;                           // X坐标清零
        TouchData[3] = 0;                           // X坐标清零
        TouchData[4] = 0;                           // Y坐标清零
        TouchData[5] = 0;                           // Y坐标清零
        TouchData[6] = 0;                           // 扫描时间清零
        TouchData[7] = 0;                           // 扫描时间清零
        TouchData[8] = 0;                           // 多点触摸标志清零
        
        hal_usb_->send_hid_report(HID_ReportID::REPORT_ID_TOUCHSCREEN, TouchData, 9);
        has_sent_report = true;
    }
    
    // 清理已处理的松开触摸点
    if (touch_state.release_modifier > 0) {
        // 将release数组中的数据移动到press数组末尾，然后清空release数组
        for (uint8_t i = 0; i < touch_state.release_modifier; i++) {
            touch_state.remove(touch_state.touch_release[i].id);
        }
        touch_state.release_modifier = 0;
    }
    
    // 清理已处理的按下触摸点状态（如果需要）
    if (has_sent_report) {
        // 重置press_modifier，因为所有按下的触摸点都已发送
        touch_state.press_modifier = 0;
    }
}

void HID::task() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    static uint32_t _now = 0;
    _now = us_to_ms(time_us_64());
    
    // 触发式发送键盘报文 - 只在状态变化时发送
    if ((keyboard_needs_send_ || keyboard_state.has_state_changed())) {
        report_keyboard();
        keyboard_needs_send_ = false;
        keyboard_state.update_last_state();
        last_keyboard_send_ = _now;
        report_count_++;
    }
    
    // 触发式发送触摸报文 - 只在有触摸事件时发送
    if ((touch_needs_send_ || 
         touch_state.press_modifier > 0 || 
         touch_state.release_modifier > 0)) {
        report_touch(_now);
        touch_needs_send_ = false;
        last_touch_send_ = _now;
        report_count_++;
    }
    
    // 统计报文发送速率
    if (_now - last_report_time_ >= 1000) {  // 每秒更新一次
        cached_report_rate_ = report_count_;
        report_count_ = 0;
        last_report_time_ = _now;
    }
}
