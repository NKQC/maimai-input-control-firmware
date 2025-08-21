#include "hid.h"
#include "pico/time.h"
#include <cstring>
#include <algorithm>
#include <cctype>

// HID描述符常量
static const uint8_t HID_KEYBOARD_DESCRIPTOR[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0x65,        //   Usage Maximum (0x65)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};  

// 游戏手柄HID描述符
static const uint8_t HID_GAMEPAD_DESCRIPTOR[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x32,        //     Usage (Z)
    0x09, 0x35,        //     Usage (Rz)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x04,        //     Report Count (4)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x10,        //   Usage Maximum (0x10)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x10,        //   Report Count (16)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x00,        //   Logical Maximum (0)
    0x35, 0x00,        //   Physical Minimum (0)
    0x45, 0x00,        //   Physical Maximum (0)
    0x65, 0x00,        //   Unit (None)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};

// 触摸屏HID描述符
static const uint8_t HID_TOUCH_DESCRIPTOR[] = {
    0x05, 0x0D,        // Usage Page (Digitizer)
    0x09, 0x04,        // Usage (Touch Screen)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x09, 0x22,        //   Usage (Finger)
    0xA1, 0x02,        //   Collection (Logical)
    0x09, 0x42,        //     Usage (Tip Switch)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x32,        //     Usage (In Range)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x47,        //     Usage (Confidence)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x05,        //     Report Count (5)
    0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x08,        //     Report Size (8)
    0x09, 0x51,        //     Usage (Contact Identifier)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x26, 0xFF, 0x0F,  //     Logical Maximum (4095)
    0x75, 0x10,        //     Report Size (16)
    0x55, 0x0E,        //     Unit Exponent (-2)
    0x65, 0x33,        //     Unit (System: English Linear, Length: Inch)
    0x09, 0x30,        //     Usage (X)
    0x35, 0x00,        //     Physical Minimum (0)
    0x46, 0xB5, 0x04,  //     Physical Maximum (1205)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x46, 0x8A, 0x03,  //     Physical Maximum (906)
    0x09, 0x31,        //     Usage (Y)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x0D,        //     Usage Page (Digitizer)
    0x09, 0x48,        //     Usage (Width)
    0x09, 0x49,        //     Usage (Height)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x30,        //     Usage (Tip Pressure)
    0x26, 0xFF, 0x00,  //     Logical Maximum (255)
    0x75, 0x08,        //     Report Size (8)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x05, 0x0D,        //   Usage Page (Digitizer)
    0x09, 0x54,        //   Usage (Contact Count)
    0x25, 0x7F,        //   Logical Maximum (127)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, 0x03,        //   Report ID (3)
    0x09, 0x55,        //   Usage (Contact Count Maximum)
    0x25, 0x0A,        //   Logical Maximum (10)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection
};

static const uint8_t HID_MOUSE_DESCRIPTOR[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x05,        //     Usage Maximum (0x05)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0xC0,              // End Collection
};

// 构造函数
HID::HID(HAL_USB* usb_hal)
    : usb_hal_(usb_hal)
    , initialized_(false)
    , report_count_(0)
    , error_count_(0)
    , last_report_time_(0) {
}

// 析构函数
HID::~HID() {
    deinit();
}

// 初始化
bool HID::init() {
    if (!usb_hal_ || initialized_) {
        return false;
    }
    
    // 初始化USB HAL
    if (!usb_hal_->init()) {
        handle_error("Failed to initialize USB HID");
        return false;
    }
    
    initialized_ = true;
    
    // 清除所有报告状态
    clear_keyboard_report();
    clear_mouse_report();
    clear_gamepad_report();
    clear_touch_report();
    
    return true;
}

// 释放资源
void HID::deinit() {
    if (initialized_) {
        // 释放所有按键
        release_all_keys();
        release_all_touch_points();
        
        usb_hal_->deinit();
        initialized_ = false;
        
        // 清理回调
        report_callback_ = nullptr;
        connect_callback_ = nullptr;
        error_callback_ = nullptr;
    }
}

// 检查是否就绪
bool HID::is_ready() const {
    return initialized_ && usb_hal_->is_connected();
}

// 设置配置
bool HID::set_config(const HID_Config& config) {
    config_ = config;
    
    if (initialized_) {
        // 重新初始化以应用新配置
        deinit();
        return init();
    }
    
    return true;
}

// 获取配置
bool HID::get_config(HID_Config& config) {
    config = config_;
    return true;
}

// 发送键盘报告
bool HID::send_keyboard_report(const HID_KeyboardReport& report) {
    if (!is_ready()) {
        return false;
    }
    
    current_keyboard_report_ = report;
    return send_report(HID_ReportType::INPUT, 1, 
                      reinterpret_cast<const uint8_t*>(&report), 
                      sizeof(report));
}

// 按下按键
bool HID::press_key(HID_KeyCode key, uint8_t modifier) {
    if (!is_ready()) {
        return false;
    }
    
    // 设置修饰键
    current_keyboard_report_.modifier |= modifier;
    
    // 添加按键到报告
    if (!add_key_to_report(key)) {
        return false;
    }
    
    return send_keyboard_report(current_keyboard_report_);
}

// 释放按键
bool HID::release_key(HID_KeyCode key) {
    if (!is_ready()) {
        return false;
    }
    
    remove_key_from_report(key);
    return send_keyboard_report(current_keyboard_report_);
}

// 释放所有按键
bool HID::release_all_keys() {
    if (!is_ready()) {
        return false;
    }
    
    clear_keyboard_report();
    return send_keyboard_report(current_keyboard_report_);
}

// 输入字符串
bool HID::type_string(const std::string& text) {
    if (!is_ready()) {
        return false;
    }
    
    for (char c : text) {
        HID_KeyCode key = char_to_keycode(c);
        uint8_t modifier = char_to_modifier(c);
        
        if (key != HID_KeyCode::KEY_NONE) {
            // 按下按键
            if (!press_key(key, modifier)) {
                return false;
            }
            
            // 短暂延迟
            sleep_ms(10);
            
            // 释放按键
            if (!release_key(key)) {
                return false;
            }
            
            // 清除修饰键
            current_keyboard_report_.modifier = 0;
            
            sleep_ms(10);
        }
    }
    
    return true;
}

// 发送鼠标报告
bool HID::send_mouse_report(const HID_MouseReport& report) {
    if (!is_ready()) {
        return false;
    }
    
    current_mouse_report_ = report;
    return send_report(HID_ReportType::INPUT, 2, 
                      reinterpret_cast<const uint8_t*>(&report), 
                      sizeof(report));
}

// 移动鼠标
bool HID::move_mouse(int8_t x, int8_t y) {
    if (!is_ready()) {
        return false;
    }
    
    current_mouse_report_.x = x;
    current_mouse_report_.y = y;
    
    bool result = send_mouse_report(current_mouse_report_);
    
    // 清除移动量
    current_mouse_report_.x = 0;
    current_mouse_report_.y = 0;
    
    return result;
}

// 点击鼠标
bool HID::click_mouse(HID_MouseButton button) {
    if (!press_mouse_button(button)) {
        return false;
    }
    
    sleep_ms(50);
    
    return release_mouse_button(button);
}

// 按下鼠标按键
bool HID::press_mouse_button(HID_MouseButton button) {
    if (!is_ready()) {
        return false;
    }
    
    current_mouse_report_.buttons |= static_cast<uint8_t>(button);
    return send_mouse_report(current_mouse_report_);
}

// 释放鼠标按键
bool HID::release_mouse_button(HID_MouseButton button) {
    if (!is_ready()) {
        return false;
    }
    
    current_mouse_report_.buttons &= ~static_cast<uint8_t>(button);
    return send_mouse_report(current_mouse_report_);
}

// 滚动滚轮
bool HID::scroll_wheel(int8_t wheel, int8_t pan) {
    if (!is_ready()) {
        return false;
    }
    
    current_mouse_report_.wheel = wheel;
    current_mouse_report_.pan = pan;
    
    bool result = send_mouse_report(current_mouse_report_);
    
    // 清除滚轮值
    current_mouse_report_.wheel = 0;
    current_mouse_report_.pan = 0;
    
    return result;
}

// 发送游戏手柄报告
bool HID::send_gamepad_report(const HID_GamepadReport& report) {
    if (!is_ready()) {
        return false;
    }
    
    current_gamepad_report_ = report;
    return send_report(HID_ReportType::INPUT, 3, 
                      reinterpret_cast<const uint8_t*>(&report), 
                      sizeof(report));
}

// 设置游戏手柄按键
bool HID::set_gamepad_button(HID_GamepadButton button, bool pressed) {
    if (!is_ready()) {
        return false;
    }
    
    if (pressed) {
        current_gamepad_report_.buttons |= static_cast<uint16_t>(button);
    } else {
        current_gamepad_report_.buttons &= ~static_cast<uint16_t>(button);
    }
    
    return send_gamepad_report(current_gamepad_report_);
}

// 设置游戏手柄轴
bool HID::set_gamepad_axis(uint8_t axis, int8_t value) {
    if (!is_ready()) {
        return false;
    }
    
    switch (axis) {
        case 0: current_gamepad_report_.left_x = value; break;
        case 1: current_gamepad_report_.left_y = value; break;
        case 2: current_gamepad_report_.right_x = value; break;
        case 3: current_gamepad_report_.right_y = value; break;
        default: return false;
    }
    
    return send_gamepad_report(current_gamepad_report_);
}

// 设置游戏手柄扳机
bool HID::set_gamepad_trigger(uint8_t trigger, uint8_t value) {
    if (!is_ready()) {
        return false;
    }
    
    switch (trigger) {
        case 0: current_gamepad_report_.left_trigger = value; break;
        case 1: current_gamepad_report_.right_trigger = value; break;
        default: return false;
    }
    
    return send_gamepad_report(current_gamepad_report_);
}

// 设置游戏手柄方向键
bool HID::set_gamepad_dpad(uint8_t direction) {
    if (!is_ready()) {
        return false;
    }
    
    current_gamepad_report_.dpad = direction;
    return send_gamepad_report(current_gamepad_report_);
}

// 发送触摸报告
bool HID::send_touch_report(const HID_TouchReport& report) {
    if (!is_ready()) {
        return false;
    }
    
    current_touch_report_ = report;
    return send_report(HID_ReportType::INPUT, 4, 
                      reinterpret_cast<const uint8_t*>(&report), 
                      sizeof(report));
}

// 设置触摸点
bool HID::set_touch_point(uint8_t contact_id, uint16_t x, uint16_t y, uint8_t pressure) {
    if (!is_ready()) {
        return false;
    }
    
    HID_TouchPoint* point = find_touch_point(contact_id);
    if (!point) {
        // 查找空闲触点
        for (int i = 0; i < 10; i++) {
            if (!current_touch_report_.contacts[i].in_contact) {
                point = &current_touch_report_.contacts[i];
                current_touch_report_.contact_count++;
                break;
            }
        }
    }
    
    if (!point) {
        return false; // 没有可用触点
    }
    
    point->contact_id = contact_id;
    point->x = x;
    point->y = y;
    point->pressure = pressure;
    point->in_contact = true;
    point->tip_switch = true;
    
    return send_touch_report(current_touch_report_);
}

// 释放触摸点
bool HID::release_touch_point(uint8_t contact_id) {
    if (!is_ready()) {
        return false;
    }
    
    HID_TouchPoint* point = find_touch_point(contact_id);
    if (point) {
        point->in_contact = false;
        point->tip_switch = false;
        current_touch_report_.contact_count--;
    }
    
    return send_touch_report(current_touch_report_);
}

// 释放所有触摸点
bool HID::release_all_touch_points() {
    if (!is_ready()) {
        return false;
    }
    
    clear_touch_report();
    return send_touch_report(current_touch_report_);
}

// 发送自定义报告
bool HID::send_custom_report(const HID_CustomReport& report) {
    return send_raw_report(report.report_id, report.data, report.length);
}

// 发送原始报告
bool HID::send_raw_report(uint8_t report_id, const uint8_t* data, uint8_t length) {
    if (!is_ready() || !data) {
        return false;
    }
    
    return send_report(HID_ReportType::INPUT, report_id, data, length);
}

// 检查连接状态
bool HID::is_connected() const {
    return usb_hal_ && usb_hal_->is_connected();
}

// 获取报告计数
uint32_t HID::get_report_count() const {
    return report_count_;
}

// 获取错误计数
uint32_t HID::get_error_count() const {
    return error_count_;
}

// 设置回调
void HID::set_report_callback(HID_ReportCallback callback) {
    report_callback_ = callback;
}

void HID::set_connect_callback(HID_ConnectCallback callback) {
    connect_callback_ = callback;
}

void HID::set_error_callback(HID_ErrorCallback callback) {
    error_callback_ = callback;
}

// 任务处理
void HID::task() {
    if (!initialized_) {
        return;
    }
    
    // 处理USB任务
    usb_hal_->task();
    
    // 检查连接状态变化
    static bool last_connected = false;
    bool current_connected = is_connected();
    if (current_connected != last_connected) {
        handle_connection_change(current_connected);
        last_connected = current_connected;
    }
}

// 静态辅助方法
HID_KeyCode HID::char_to_keycode(char c) {
    if (c >= 'a' && c <= 'z') {
        return static_cast<HID_KeyCode>(static_cast<uint8_t>(HID_KeyCode::KEY_A) + (c - 'a'));
    } else if (c >= 'A' && c <= 'Z') {
        return static_cast<HID_KeyCode>(static_cast<uint8_t>(HID_KeyCode::KEY_A) + (c - 'A'));
    } else if (c >= '1' && c <= '9') {
        return static_cast<HID_KeyCode>(static_cast<uint8_t>(HID_KeyCode::KEY_1) + (c - '1'));
    } else if (c == '0') {
        return HID_KeyCode::KEY_0;
    } else if (c == ' ') {
        return HID_KeyCode::KEY_SPACE;
    } else if (c == '\n' || c == '\r') {
        return HID_KeyCode::KEY_ENTER;
    } else if (c == '\t') {
        return HID_KeyCode::KEY_TAB;
    } else if (c == '\b') {
        return HID_KeyCode::KEY_BACKSPACE;
    }
    
    return HID_KeyCode::KEY_NONE;
}

uint8_t HID::char_to_modifier(char c) {
    if (c >= 'A' && c <= 'Z') {
        return 0x02; // Left Shift
    }
    
    // 其他特殊字符的修饰键可以在这里添加
    switch (c) {
        case '!': case '@': case '#': case '$': case '%':
        case '^': case '&': case '*': case '(': case ')':
        case '_': case '+': case '{': case '}': case '|':
        case ':': case '"': case '<': case '>': case '?':
            return 0x02; // Left Shift
        default:
            return 0x00;
    }
}

std::vector<uint8_t> HID::generate_hid_descriptor(HID_DeviceType device_type) {
    switch (device_type) {
        case HID_DeviceType::KEYBOARD:
            return std::vector<uint8_t>(HID_KEYBOARD_DESCRIPTOR, 
                                       HID_KEYBOARD_DESCRIPTOR + sizeof(HID_KEYBOARD_DESCRIPTOR));
        case HID_DeviceType::MOUSE:
            return std::vector<uint8_t>(HID_MOUSE_DESCRIPTOR, 
                                       HID_MOUSE_DESCRIPTOR + sizeof(HID_MOUSE_DESCRIPTOR));
        case HID_DeviceType::GAMEPAD:
        case HID_DeviceType::JOYSTICK:
            return std::vector<uint8_t>(HID_GAMEPAD_DESCRIPTOR, 
                                       HID_GAMEPAD_DESCRIPTOR + sizeof(HID_GAMEPAD_DESCRIPTOR));
        case HID_DeviceType::TOUCH:
            return std::vector<uint8_t>(HID_TOUCH_DESCRIPTOR, 
                                       HID_TOUCH_DESCRIPTOR + sizeof(HID_TOUCH_DESCRIPTOR));
        case HID_DeviceType::CUSTOM:
        default:
            // 返回空描述符，需要用户自定义
            return std::vector<uint8_t>();
    }
}

// 私有方法实现
bool HID::send_report(HID_ReportType type, uint8_t report_id, const uint8_t* data, uint8_t length) {
    if (!is_ready() || !data) {
        return false;
    }
    
    // 检查报告间隔
    uint32_t current_time = time_us_32() / 1000;
    if (current_time - last_report_time_ < config_.report_interval_ms) {
        return false;
    }
    
    bool result = false;
     if (type == HID_ReportType::INPUT) {
         if (report_id == REPORTID_TOUCHPAD) {
             result = usb_hal_->send_touch_report(reinterpret_cast<const TouchPoint*>(data), length / sizeof(TouchPoint));
         } else if (report_id == REPORTID_KEYBOARD_1 || report_id == REPORTID_KEYBOARD_2) {
             result = usb_hal_->send_keyboard_report(report_id, *reinterpret_cast<const KeyboardReport*>(data));
         }
     }
    
    if (result) {
        report_count_++;
        last_report_time_ = current_time;
        
        // 通知回调
        if (report_callback_) {
            report_callback_(type, data, length);
        }
        
        return true;
    } else {
        error_count_++;
        handle_error("Failed to send HID report");
        return false;
    }
}

void HID::handle_received_report(HID_ReportType type, const uint8_t* data, uint8_t length) {
    if (report_callback_) {
        report_callback_(type, data, length);
    }
}

void HID::handle_connection_change(bool connected) {
    if (connect_callback_) {
        connect_callback_(connected);
    }
}

void HID::handle_error(const std::string& error) {
    if (error_callback_) {
        error_callback_(error);
    }
}

// 键盘辅助方法
bool HID::add_key_to_report(HID_KeyCode key) {
    uint8_t keycode = static_cast<uint8_t>(key);
    
    // 检查是否已经在报告中
    for (int i = 0; i < 6; i++) {
        if (current_keyboard_report_.keys[i] == keycode) {
            return true; // 已存在
        }
    }
    
    // 查找空位
    for (int i = 0; i < 6; i++) {
        if (current_keyboard_report_.keys[i] == 0) {
            current_keyboard_report_.keys[i] = keycode;
            return true;
        }
    }
    
    return false; // 报告已满
}

bool HID::remove_key_from_report(HID_KeyCode key) {
    uint8_t keycode = static_cast<uint8_t>(key);
    
    for (int i = 0; i < 6; i++) {
        if (current_keyboard_report_.keys[i] == keycode) {
            current_keyboard_report_.keys[i] = 0;
            return true;
        }
    }
    
    return false;
}

void HID::clear_keyboard_report() {
    current_keyboard_report_.modifier = 0;
    current_keyboard_report_.reserved = 0;
    memset(current_keyboard_report_.keys, 0, sizeof(current_keyboard_report_.keys));
}

void HID::clear_mouse_report() {
    current_mouse_report_.buttons = 0;
    current_mouse_report_.x = 0;
    current_mouse_report_.y = 0;
    current_mouse_report_.wheel = 0;
    current_mouse_report_.pan = 0;
}

void HID::clear_gamepad_report() {
    current_gamepad_report_.buttons = 0;
    current_gamepad_report_.left_x = 0;
    current_gamepad_report_.left_y = 0;
    current_gamepad_report_.right_x = 0;
    current_gamepad_report_.right_y = 0;
    current_gamepad_report_.left_trigger = 0;
    current_gamepad_report_.right_trigger = 0;
    current_gamepad_report_.dpad = 0;
}

HID_TouchPoint* HID::find_touch_point(uint8_t contact_id) {
    for (int i = 0; i < 10; i++) {
        if (current_touch_report_.contacts[i].contact_id == contact_id && 
            current_touch_report_.contacts[i].in_contact) {
            return &current_touch_report_.contacts[i];
        }
    }
    return nullptr;
}

void HID::clear_touch_report() {
    current_touch_report_.contact_count = 0;
    for (int i = 0; i < 10; i++) {
        current_touch_report_.contacts[i] = HID_TouchPoint();
    }
}