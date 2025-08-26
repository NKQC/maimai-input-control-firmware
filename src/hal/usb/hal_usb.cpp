#include "hal_usb.h"
#include <cstring>

// TinyUSB HID报告描述符
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

// 单例实例
HAL_USB* HAL_USB::instance_ = nullptr;

// 私有构造函数
HAL_USB::HAL_USB() 
    : initialized_(false) {
}

// 析构函数
HAL_USB::~HAL_USB() {
    deinit();
}

// 获取单例实例
HAL_USB* HAL_USB::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_USB();
    }
    return instance_;
}

// 初始化USB
bool HAL_USB::init() {
    if (initialized_) {
        return true;
    }

    usb_cdc.begin(CDC_BAUD);
    
    // 初始化HID
    usb_hid.setPollInterval(0);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.setStringDescriptor(USB_DEVICE_NAME);
    usb_hid.begin();
    
    initialized_ = true;
    return true;
}

// 清理USB
void HAL_USB::deinit() {
    if (!initialized_) {
        return;
    }
    
    // TinyUSB会自动处理清理
    initialized_ = false;
}

// 检查是否已初始化
bool HAL_USB::is_initialized() const {
    return initialized_;
}

// 检查是否已连接
bool HAL_USB::is_connected() const {
    return initialized_ && TinyUSBDevice.mounted();
}

// CDC写入数据
bool HAL_USB::cdc_write(const uint8_t* data, size_t length) {
    if (!is_connected() || !data || length == 0) {
        return false;
    }
    size_t written = usb_cdc.write(data, length);
    return written == length;
}

// CDC写入字符串
bool HAL_USB::cdc_write_string(const std::string& str) {
    return cdc_write(reinterpret_cast<const uint8_t*>(str.c_str()), str.length());
}

// CDC读取数据
size_t HAL_USB::cdc_read(uint8_t* buffer, size_t max_length) {
    if (!is_connected() || !buffer || max_length == 0) {
        return 0;
    }
    
    return usb_cdc.read(buffer, max_length);
}

// 检查CDC是否有数据可读
bool HAL_USB::cdc_available() {
    return is_connected() && usb_cdc.available();
}

// 刷新CDC缓冲区
void HAL_USB::cdc_flush() {
    if (is_connected()) {
        usb_cdc.flush();
    }
}

// 发送NKRO键盘报告（位图模式）
bool HAL_USB::hid_keyboard_nkro_report(const NKROKeyboardReport* report) {
    if (!is_connected() || !usb_hid.ready() || !report) {
        return false;
    }
    
    // 将位图转换为按键数组（最多6个）
    uint8_t keys[6] = {0};
    uint8_t key_index = 0;
    
    // 按键码映射表：位图索引 -> USB HID按键码
    static const uint8_t keycode_map[] = {
        0x00, // bit 0: KEY_NONE
        0x04, // bit 1: KEY_A
        0x05, // bit 2: KEY_B  
        0x06, // bit 3: KEY_C
        0x07, // bit 4: KEY_D
        0x08, // bit 5: KEY_E
        0x09, // bit 6: KEY_F
        0x0A, // bit 7: KEY_G
        0x0B, // bit 8: KEY_H
        0x0C, // bit 9: KEY_I
        0x0D, // bit 10: KEY_J
        0x0E, // bit 11: KEY_K
        0x0F, // bit 12: KEY_L
        0x10, // bit 13: KEY_M
        0x11, // bit 14: KEY_N
        0x12, // bit 15: KEY_O
        0x13, // bit 16: KEY_P
        0x14, // bit 17: KEY_Q
        0x15, // bit 18: KEY_R
        0x16, // bit 19: KEY_S
        0x17, // bit 20: KEY_T
        0x18, // bit 21: KEY_U
        0x19, // bit 22: KEY_V
        0x1A, // bit 23: KEY_W
        0x1B, // bit 24: KEY_X
        0x1C, // bit 25: KEY_Y
        0x1D, // bit 26: KEY_Z
        0x1E, // bit 27: KEY_1
        0x1F, // bit 28: KEY_2
        0x20, // bit 29: KEY_3
        0x21, // bit 30: KEY_4
        0x22, // bit 31: KEY_5
        0x23, // bit 32: KEY_6
        0x24, // bit 33: KEY_7
        0x25, // bit 34: KEY_8
        0x26, // bit 35: KEY_9
        0x27, // bit 36: KEY_0
        0x28, // bit 37: KEY_ENTER
        0x29, // bit 38: KEY_ESCAPE
        0x2A, // bit 39: KEY_BACKSPACE
        0x2B, // bit 40: KEY_TAB
        0x2C, // bit 41: KEY_SPACE
        0x3A, // bit 42: KEY_F1
        0x3B, // bit 43: KEY_F2
        0x3C, // bit 44: KEY_F3
        0x3D, // bit 45: KEY_F4
        0x3E, // bit 46: KEY_F5
        0x3F, // bit 47: KEY_F6
        0x40, // bit 48: KEY_F7
        0x41, // bit 49: KEY_F8
        0x42, // bit 50: KEY_F9
        0x43, // bit 51: KEY_F10
        0x44, // bit 52: KEY_F11
        0x45, // bit 53: KEY_F12
        0xE0, // bit 54: KEY_LEFT_CTRL
        0xE1, // bit 55: KEY_LEFT_SHIFT
        0xE2, // bit 56: KEY_LEFT_ALT
        0xE3, // bit 57: KEY_LEFT_GUI
        0xE4, // bit 58: KEY_RIGHT_CTRL
        0xE5, // bit 59: KEY_RIGHT_SHIFT
        0xE6, // bit 60: KEY_RIGHT_ALT
        0xE7, // bit 61: KEY_RIGHT_GUI
        0x00, // bit 62: KEY_JOYSTICK_A (自定义，映射为无效)
        0x00, // bit 63: KEY_JOYSTICK_B (自定义，映射为无效)
        0x00  // bit 64: KEY_JOYSTICK_CONFIRM (自定义，映射为无效)
    };
    
    for (uint8_t byte_idx = 0; byte_idx < 13 && key_index < 6; byte_idx++) {
        uint8_t byte_val = report->keys[byte_idx];
        for (uint8_t bit_idx = 0; bit_idx < 8 && key_index < 6; bit_idx++) {
            if (byte_val & (1 << bit_idx)) {
                uint8_t bit_position = byte_idx * 8 + bit_idx;
                if (bit_position < sizeof(keycode_map)) {
                    uint8_t keycode = keycode_map[bit_position];
                    if (keycode != 0x00) { // 跳过无效按键码
                        keys[key_index++] = keycode;
                    }
                }
            }
        }
    }
    
    // 直接发送，无延迟
    return usb_hid.keyboardReport(0, report->modifier, keys);
}



// 直接发送单个按键状态更新 - 立即发送
bool HAL_USB::hid_keyboard_single_key(HID_KeyCode key, bool pressed, uint8_t modifier) {
    if (!is_connected() || !usb_hid.ready()) {
        return false;
    }
    
    // 维护当前按键状态（最多6个按键）
    static uint8_t current_keys[6] = {0};
    static uint8_t current_modifier = 0;
    
    // 更新修饰键
    current_modifier = modifier;
    
    // HID_KeyCode的枚举值本身就是标准的USB HID keycode
    uint8_t hid_keycode = static_cast<uint8_t>(key);
    
    // 跳过自定义摇杆按键（它们不是标准HID按键）
    if (key == HID_KeyCode::KEY_JOYSTICK_A || 
        key == HID_KeyCode::KEY_JOYSTICK_B || 
        key == HID_KeyCode::KEY_JOYSTICK_CONFIRM) {
        return true; // 摇杆按键不发送HID报告
    }
    
    if (pressed) {
        // 添加按键（如果不存在且有空位）
        bool found = false;
        for (int i = 0; i < 6; i++) {
            if (current_keys[i] == hid_keycode) {
                found = true;
                break;
            }
        }
        if (!found) {
            for (int i = 0; i < 6; i++) {
                if (current_keys[i] == 0) {
                    current_keys[i] = hid_keycode;
                    break;
                }
            }
        }
    } else {
        // 移除按键
        for (int i = 0; i < 6; i++) {
            if (current_keys[i] == hid_keycode) {
                current_keys[i] = 0;
                // 压缩数组，将0移到末尾
                for (int j = i; j < 5; j++) {
                    current_keys[j] = current_keys[j + 1];
                }
                current_keys[5] = 0;
                break;
            }
        }
    }
    
    // 立即发送
    return usb_hid.keyboardReport(0, current_modifier, current_keys);
}

// 发送触摸报告（空实现）
bool HAL_USB::hid_touch_report(const TouchPoint* points, uint8_t count) {
    // 空实现，保持兼容性
    (void)points;
    (void)count;
    return true;
}
