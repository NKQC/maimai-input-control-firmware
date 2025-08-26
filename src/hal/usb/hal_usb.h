#pragma once

#include "Adafruit_TinyUSB.h"
#include <stdint.h>
#include <stdbool.h>
#include <string>

/**
 * USB HAL 模块 - 基于TinyUSB的简化实现
 * 提供CDC串口和HID键盘的基础功能
 */

// USB设备配置
#ifndef USB_VID
#define USB_VID                 0x2E8A  // Raspberry Pi Foundation
#endif

#ifndef USB_PID
#define USB_PID                 0x000A  // Raspberry Pi Pico
#endif

#define CDC_BAUD                115200

#define USB_DEVICE_NAME         "maimai_input_control"

// HID报告ID定义
#define HID_REPORTID_KEYBOARD   0x01
#define HID_REPORTID_TOUCH      0x02

// 键盘按键代码 (USB HID) - 前置定义
enum class HID_KeyCode : uint8_t {
    KEY_NONE = 0x00,
    KEY_A = 0x04,
    KEY_B,
    KEY_C,
    KEY_D,
    KEY_E,
    KEY_F,
    KEY_G,
    KEY_H,
    KEY_I,
    KEY_J,
    KEY_K,
    KEY_L,
    KEY_M,
    KEY_N,
    KEY_O,
    KEY_P,
    KEY_Q,
    KEY_R,
    KEY_S,
    KEY_T,
    KEY_U,
    KEY_V,
    KEY_W,
    KEY_X,
    KEY_Y,
    KEY_Z,
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_6,
    KEY_7,
    KEY_8,
    KEY_9,
    KEY_0,
    KEY_ENTER,
    KEY_ESCAPE,
    KEY_BACKSPACE,
    KEY_TAB,
    KEY_SPACE,

    KEY_F1 = 0x3A,
    KEY_F2,
    KEY_F3,
    KEY_F4,
    KEY_F5,
    KEY_F6,
    KEY_F7,
    KEY_F8,
    KEY_F9,
    KEY_F10,
    KEY_F11,
    KEY_F12,
    // HID控制按键
    KEY_LEFT_CTRL = 0xE0,
    KEY_LEFT_SHIFT,
    KEY_LEFT_ALT,
    KEY_LEFT_GUI,
    KEY_RIGHT_CTRL,
    KEY_RIGHT_SHIFT,
    KEY_RIGHT_ALT,
    KEY_RIGHT_GUI,
    // 摇杆专用按键
    KEY_JOYSTICK_A,      // 摇杆A按钮
    KEY_JOYSTICK_B,      // 摇杆B按钮
    KEY_JOYSTICK_CONFIRM, // 摇杆确认按钮
};

// 标准6键键盘报告结构（兼容性）
struct KeyboardReport {
    uint8_t modifier;    // 修饰键
    uint8_t keys[6];     // 按键数组
};

// 全键无冲（NKRO）键盘报告结构 - 支持104个按键
struct NKROKeyboardReport {
    uint8_t modifier;    // 修饰键 (8位)
    uint8_t keys[13];    // 按键位图 (104位 = 13字节)
};

// 扩展键盘报告结构 - 支持最多26个同时按下的按键
struct ExtendedKeyboardReport {
    uint8_t modifier;    // 修饰键
    uint8_t key_count;   // 当前按下的按键数量
    uint8_t keys[26];    // 按键数组，支持26个按键同时按下
};

// 触摸点结构
struct TouchPoint {
    uint16_t x;
    uint16_t y;
    uint8_t id;
    bool pressed;
};

// HAL_USB类声明
class HAL_USB {
public:
    // 单例模式
    static HAL_USB* getInstance();
    
    // 禁用拷贝构造和赋值
    HAL_USB(const HAL_USB&) = delete;
    HAL_USB& operator=(const HAL_USB&) = delete;
    
    // 初始化和清理
    bool init();
    void deinit();
    bool is_initialized() const;
    bool is_connected() const;
    
    // CDC串口功能
    bool cdc_write(const uint8_t* data, size_t length);
    bool cdc_write_string(const std::string& str);
    size_t cdc_read(uint8_t* buffer, size_t max_length);
    bool cdc_available();
    void cdc_flush();
    
    // HID键盘功能 - 全键无冲接口（零拷贝，最低延迟）
    bool hid_keyboard_nkro_report(const NKROKeyboardReport* report);
    
    // 直接发送单个按键状态更新 - 立即发送
    bool hid_keyboard_single_key(HID_KeyCode key, bool pressed, uint8_t modifier = 0);
    
    // 触摸功能（空实现，保持兼容性）
    bool hid_touch_report(const TouchPoint* points, uint8_t count);
    
public:
    ~HAL_USB();
    
private:
    HAL_USB();
    Adafruit_USBD_HID usb_hid;
    Adafruit_USBD_CDC usb_cdc;

    static HAL_USB* instance_;
    bool initialized_;
    
    // 内部状态
    KeyboardReport current_keyboard_report_;
    bool keyboard_report_pending_;
    
};
