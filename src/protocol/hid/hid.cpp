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

// 按下按键 - 立即发送更新
bool HID::press_key(HID_KeyCode key, uint8_t modifier) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    
    // 直接调用HAL接口立即发送
    return hal_usb_->hid_keyboard_single_key(key, true, modifier);
}

// 释放按键 - 立即发送更新
bool HID::release_key(HID_KeyCode key) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    
    // 直接调用HAL接口立即发送
    return hal_usb_->hid_keyboard_single_key(key, false, 0);
}

// 获取实际回报速率
uint8_t HID::get_report_rate() const {
    return cached_report_rate_;
}

// 触摸报告发送 (空实现，保持向后兼容)
bool HID::send_touch_report(const HID_TouchReport& report) {
    // 触摸功能保留方法
    // 直接返回true，不执行任何操作
    return true;
}
