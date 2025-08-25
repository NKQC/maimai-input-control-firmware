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
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, 0x03,        //   Report ID (3)
    0x09, 0x55,        //   Usage (Contact Count Maximum)
    0x25, 0x0A,        //   Logical Maximum (10)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection
};

// 单例实例
HID* HID::instance_ = nullptr;

// 私有构造函数
HID::HID()
    : usb_hal_(nullptr)
    , initialized_(false)
    , report_count_(0)
    , error_count_(0)
    , last_report_time_(0) {
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
bool HID::init(HAL_USB* usb_hal) {
    if (initialized_) {
        return true;
    }
    
    if (!usb_hal) {
        handle_error("USB HAL is null");
        return false;
    }
    
    usb_hal_ = usb_hal;
    
    // 清空报告状态
    clear_keyboard_report();
    clear_touch_report();
    
    initialized_ = true;
    return true;
}

// 反初始化
void HID::deinit() {
    if (!initialized_) {
        return;
    }
    
    // 清空所有报告
    release_all_keys();
    release_all_touch_points();
    
    if (usb_hal_) {
        usb_hal_->deinit();
        usb_hal_ = nullptr;
    }
    
    initialized_ = false;
}

// 检查是否就绪
bool HID::is_ready() const {
    return initialized_ && usb_hal_ && usb_hal_->is_connected();
}

// 设置配置
bool HID::set_config(const HID_Config& config) {
    config_ = config;
    
    if (initialized_ && usb_hal_) {
        // 重新配置USB设备
        return usb_hal_->configure_device(config_.vendor_id, config_.product_id, 
                                         config_.manufacturer, config_.product, config_.serial_number);
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
    
    // 发送报告 (移除了reserved字段，所以大小是7字节)
    return send_report(HID_ReportType::INPUT, 1, 
                      reinterpret_cast<const uint8_t*>(&report), 7);
}

// 高效发送键盘bitmap数据
bool HID::send_keyboard_data(const KeyboardBitmap& bitmap) {
    if (!is_ready()) {
        return false;
    }
    
    HID_KeyboardReport report;
    uint8_t key_count = 0;
    
    // 处理修饰键
    if (bitmap.getKey(HID_KeyCode::KEY_LEFT_CTRL)) report.modifier |= 0x01;
    if (bitmap.getKey(HID_KeyCode::KEY_LEFT_SHIFT)) report.modifier |= 0x02;
    if (bitmap.getKey(HID_KeyCode::KEY_LEFT_ALT)) report.modifier |= 0x04;
    if (bitmap.getKey(HID_KeyCode::KEY_LEFT_GUI)) report.modifier |= 0x08;
    if (bitmap.getKey(HID_KeyCode::KEY_RIGHT_CTRL)) report.modifier |= 0x10;
    if (bitmap.getKey(HID_KeyCode::KEY_RIGHT_SHIFT)) report.modifier |= 0x20;
    if (bitmap.getKey(HID_KeyCode::KEY_RIGHT_ALT)) report.modifier |= 0x40;
    if (bitmap.getKey(HID_KeyCode::KEY_RIGHT_GUI)) report.modifier |= 0x80;
    
    // 遍历所有支持的按键（除修饰键外）
    for (const HID_KeyCode& key : supported_keys) {
        if (key >= HID_KeyCode::KEY_LEFT_CTRL) continue; // 跳过修饰键
        
        if (bitmap.getKey(key) && key_count < 6) {
            report.keys[key_count++] = static_cast<uint8_t>(key);
        }
    }
    
    return send_keyboard_report(report);
}

// 按下按键
bool HID::press_key(HID_KeyCode key, uint8_t modifier) {
    if (!is_ready()) {
        return false;
    }
    
    current_keyboard_report_.modifier |= modifier;
    
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
            
            sleep_ms(10);
        }
    }
    
    return true;
}

// 发送触摸报告
bool HID::send_touch_report(const HID_TouchReport& report) {
    if (!is_ready()) {
        return false;
    }
    
    current_touch_report_ = report;
    
    return send_report(HID_ReportType::INPUT, 2, 
                      reinterpret_cast<const uint8_t*>(&report), sizeof(report));
}

// 设置触摸点
bool HID::set_touch_point(uint8_t contact_id, uint16_t x, uint16_t y, uint8_t pressure) {
    if (!is_ready()) {
        return false;
    }
    
    HID_TouchPoint* point = find_touch_point(contact_id);
    if (!point) {
        // 查找空闲触摸点
        for (int i = 0; i < 10; i++) {
            if (!current_touch_report_.contacts[i].in_contact) {
                point = &current_touch_report_.contacts[i];
                current_touch_report_.contact_count++;
                break;
            }
        }
    }
    
    if (!point) {
        return false; // 没有可用的触摸点
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
    if (point && point->in_contact) {
        point->in_contact = false;
        point->tip_switch = false;
        current_touch_report_.contact_count--;
        
        return send_touch_report(current_touch_report_);
    }
    
    return false;
}

// 释放所有触摸点
bool HID::release_all_touch_points() {
    if (!is_ready()) {
        return false;
    }
    
    clear_touch_report();
    return send_touch_report(current_touch_report_);
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

// 设置报告回调
void HID::set_report_callback(HID_ReportCallback callback) {
    report_callback_ = callback;
}

// 设置连接回调
void HID::set_connect_callback(HID_ConnectCallback callback) {
    connect_callback_ = callback;
}

// 设置错误回调
void HID::set_error_callback(HID_ErrorCallback callback) {
    error_callback_ = callback;
}

// 任务循环
void HID::task() {
    if (!initialized_ || !usb_hal_) {
        return;
    }
    
    // 处理USB任务
    usb_hal_->task();
    
    // 检查连接状态变化
    static bool last_connected = false;
    bool connected = is_connected();
    if (connected != last_connected) {
        handle_connection_change(connected);
        last_connected = connected;
    }
    
    // 定期发送心跳报告
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (connected && (current_time - last_report_time_) > config_.report_interval_ms) {
        last_report_time_ = current_time;
    }
}

// 字符转按键码
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
    } else if (c == '\n') {
        return HID_KeyCode::KEY_ENTER;
    } else if (c == '\t') {
        return HID_KeyCode::KEY_TAB;
    }
    
    return HID_KeyCode::KEY_NONE;
}

// 字符转修饰键
uint8_t HID::char_to_modifier(char c) {
    if (c >= 'A' && c <= 'Z') {
        return static_cast<uint8_t>(HID_KeyCode::KEY_LEFT_SHIFT);
    }
    return 0;
}

// 生成HID描述符
std::vector<uint8_t> HID::generate_hid_descriptor(HID_DeviceType device_type) {
    switch (device_type) {
        case HID_DeviceType::KEYBOARD:
            return std::vector<uint8_t>(HID_KEYBOARD_DESCRIPTOR, 
                                       HID_KEYBOARD_DESCRIPTOR + sizeof(HID_KEYBOARD_DESCRIPTOR));
        case HID_DeviceType::TOUCH:
            return std::vector<uint8_t>(HID_TOUCH_DESCRIPTOR, 
                                       HID_TOUCH_DESCRIPTOR + sizeof(HID_TOUCH_DESCRIPTOR));
        default:
            return std::vector<uint8_t>();
    }
}

// 发送报告
bool HID::send_report(HID_ReportType type, uint8_t report_id, const uint8_t* data, uint8_t length) {
    if (!is_ready()) {
        return false;
    }
    
    bool success = false;
    
    switch (type) {
        case HID_ReportType::INPUT:
            success = usb_hal_->send_hid_report(report_id, data, length);
            break;
        case HID_ReportType::OUTPUT:
        case HID_ReportType::FEATURE:
            // 暂不支持输出和特性报告
            break;
    }
    
    if (success) {
        report_count_++;
        last_report_time_ = to_ms_since_boot(get_absolute_time());
    } else {
        error_count_++;
        handle_error("Failed to send HID report");
    }
    
    return success;
}

// 处理接收到的报告
void HID::handle_received_report(HID_ReportType type, const uint8_t* data, uint8_t length) {
    if (report_callback_) {
        report_callback_(type, data, length);
    }
}

// 处理连接状态变化
void HID::handle_connection_change(bool connected) {
    if (connect_callback_) {
        connect_callback_(connected);
    }
}

// 处理错误
void HID::handle_error(const std::string& error) {
    if (error_callback_) {
        error_callback_(error);
    }
}

// 生成键盘描述符
std::vector<uint8_t> HID::generate_keyboard_descriptor() {
    return std::vector<uint8_t>(HID_KEYBOARD_DESCRIPTOR, 
                               HID_KEYBOARD_DESCRIPTOR + sizeof(HID_KEYBOARD_DESCRIPTOR));
}

// 生成触摸描述符
std::vector<uint8_t> HID::generate_touch_descriptor() {
    return std::vector<uint8_t>(HID_TOUCH_DESCRIPTOR, 
                               HID_TOUCH_DESCRIPTOR + sizeof(HID_TOUCH_DESCRIPTOR));
}

// 添加按键到报告
bool HID::add_key_to_report(HID_KeyCode key) {
    if (key == HID_KeyCode::KEY_NONE) {
        return false;
    }
    
    // 检查是否已经存在
    for (int i = 0; i < 6; i++) {
        if (current_keyboard_report_.keys[i] == static_cast<uint8_t>(key)) {
            return true; // 已存在
        }
    }
    
    // 查找空位
    for (int i = 0; i < 6; i++) {
        if (current_keyboard_report_.keys[i] == 0) {
            current_keyboard_report_.keys[i] = static_cast<uint8_t>(key);
            return true;
        }
    }
    
    return false; // 没有空位
}

// 从报告中移除按键
bool HID::remove_key_from_report(HID_KeyCode key) {
    for (int i = 0; i < 6; i++) {
        if (current_keyboard_report_.keys[i] == static_cast<uint8_t>(key)) {
            current_keyboard_report_.keys[i] = 0;
            return true;
        }
    }
    return false;
}

// 清空键盘报告
void HID::clear_keyboard_report() {
    current_keyboard_report_.modifier = 0;
    memset(current_keyboard_report_.keys, 0, sizeof(current_keyboard_report_.keys));
}

// 查找触摸点
HID_TouchPoint* HID::find_touch_point(uint8_t contact_id) {
    for (int i = 0; i < 10; i++) {
        if (current_touch_report_.contacts[i].contact_id == contact_id && 
            current_touch_report_.contacts[i].in_contact) {
            return &current_touch_report_.contacts[i];
        }
    }
    return nullptr;
}

// 清空触摸报告
void HID::clear_touch_report() {
    current_touch_report_.contact_count = 0;
    for (int i = 0; i < 10; i++) {
        current_touch_report_.contacts[i] = HID_TouchPoint();
    }
}