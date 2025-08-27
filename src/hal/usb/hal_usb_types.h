#pragma once

#include <stdint.h>
#include <string>
#include <tusb.h>

enum HID_ReportID : uint8_t {
    REPORT_ID_TOUCHSCREEN = 0x1,
    REPORT_ID_KEYBOARD1,   // 6 * 3
    REPORT_ID_KEYBOARD2,
    REPORT_ID_KEYBOARD3,
};

// HID Keyboard
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
    // 箭头键
    KEY_UP_ARROW = 0x52,
    KEY_DOWN_ARROW = 0x51,
    KEY_LEFT_ARROW = 0x50,
    KEY_RIGHT_ARROW = 0x4F,
    // 控制键
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

// 遍历所有支持的HID键码
#define SUPPORTED_KEYS_COUNT 61
static const HID_KeyCode supported_keys[SUPPORTED_KEYS_COUNT] = {
    HID_KeyCode::KEY_A, HID_KeyCode::KEY_B, HID_KeyCode::KEY_C, HID_KeyCode::KEY_D,
    HID_KeyCode::KEY_E, HID_KeyCode::KEY_F, HID_KeyCode::KEY_G, HID_KeyCode::KEY_H,
    HID_KeyCode::KEY_I, HID_KeyCode::KEY_J, HID_KeyCode::KEY_K, HID_KeyCode::KEY_L,
    HID_KeyCode::KEY_M, HID_KeyCode::KEY_N, HID_KeyCode::KEY_O, HID_KeyCode::KEY_P,
    HID_KeyCode::KEY_Q, HID_KeyCode::KEY_R, HID_KeyCode::KEY_S, HID_KeyCode::KEY_T,
    HID_KeyCode::KEY_U, HID_KeyCode::KEY_V, HID_KeyCode::KEY_W, HID_KeyCode::KEY_X,
    HID_KeyCode::KEY_Y, HID_KeyCode::KEY_Z, HID_KeyCode::KEY_1, HID_KeyCode::KEY_2,
    HID_KeyCode::KEY_3, HID_KeyCode::KEY_4, HID_KeyCode::KEY_5, HID_KeyCode::KEY_6,
    HID_KeyCode::KEY_7, HID_KeyCode::KEY_8, HID_KeyCode::KEY_9, HID_KeyCode::KEY_0,
    HID_KeyCode::KEY_ENTER, HID_KeyCode::KEY_ESCAPE, HID_KeyCode::KEY_BACKSPACE,
    HID_KeyCode::KEY_TAB, HID_KeyCode::KEY_SPACE, HID_KeyCode::KEY_F1, HID_KeyCode::KEY_F2,
    HID_KeyCode::KEY_F3, HID_KeyCode::KEY_F4, HID_KeyCode::KEY_F5, HID_KeyCode::KEY_F6,
    HID_KeyCode::KEY_F7, HID_KeyCode::KEY_F8, HID_KeyCode::KEY_F9, HID_KeyCode::KEY_F10,
    HID_KeyCode::KEY_F11, HID_KeyCode::KEY_F12, HID_KeyCode::KEY_LEFT_CTRL,
    HID_KeyCode::KEY_LEFT_SHIFT, HID_KeyCode::KEY_LEFT_ALT, HID_KeyCode::KEY_LEFT_GUI,
    HID_KeyCode::KEY_RIGHT_CTRL, HID_KeyCode::KEY_RIGHT_SHIFT, HID_KeyCode::KEY_RIGHT_ALT,
    HID_KeyCode::KEY_RIGHT_GUI
};

// 触摸点数据结构
struct HID_TouchPoint {
    bool press = false;
    uint8_t id = 0;         // 触摸点ID
    uint16_t x = 0;         // X坐标
    uint16_t y = 0;         // Y坐标
    void clear() {
        press = false;
        id = 0;
        x = 0;
        y = 0;
    }
};
