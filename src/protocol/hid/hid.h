#ifndef HID_H
#define HID_H

#include "../../hal/usb/hal_usb.h"
#include <cstdint>
#include <functional>
#include <vector>
#include <array>
#include <cstring>

// HID设备类型
enum class HID_DeviceType : uint8_t {
    KEYBOARD = 0,
    MOUSE = 1,
    GAMEPAD = 2,
    JOYSTICK = 3,
    TOUCH = 4,
    CUSTOM = 5
};

// HID报告类型
enum class HID_ReportType : uint8_t {
    INPUT = 1,
    OUTPUT = 2,
    FEATURE = 3
};

// 键盘按键代码 (USB HID)
enum class HID_KeyCode : uint8_t {
    KEY_NONE = 0x00,
    KEY_A = 0x04,
    KEY_B = 0x05,
    KEY_C = 0x06,
    KEY_D = 0x07,
    KEY_E = 0x08,
    KEY_F = 0x09,
    KEY_G = 0x0A,
    KEY_H = 0x0B,
    KEY_I = 0x0C,
    KEY_J = 0x0D,
    KEY_K = 0x0E,
    KEY_L = 0x0F,
    KEY_M = 0x10,
    KEY_N = 0x11,
    KEY_O = 0x12,
    KEY_P = 0x13,
    KEY_Q = 0x14,
    KEY_R = 0x15,
    KEY_S = 0x16,
    KEY_T = 0x17,
    KEY_U = 0x18,
    KEY_V = 0x19,
    KEY_W = 0x1A,
    KEY_X = 0x1B,
    KEY_Y = 0x1C,
    KEY_Z = 0x1D,
    KEY_1 = 0x1E,
    KEY_2 = 0x1F,
    KEY_3 = 0x20,
    KEY_4 = 0x21,
    KEY_5 = 0x22,
    KEY_6 = 0x23,
    KEY_7 = 0x24,
    KEY_8 = 0x25,
    KEY_9 = 0x26,
    KEY_0 = 0x27,
    KEY_ENTER = 0x28,
    KEY_ESCAPE = 0x29,
    KEY_BACKSPACE = 0x2A,
    KEY_TAB = 0x2B,
    KEY_SPACE = 0x2C,
    KEY_F1 = 0x3A,
    KEY_F2 = 0x3B,
    KEY_F3 = 0x3C,
    KEY_F4 = 0x3D,
    KEY_F5 = 0x3E,
    KEY_F6 = 0x3F,
    KEY_F7 = 0x40,
    KEY_F8 = 0x41,
    KEY_F9 = 0x42,
    KEY_F10 = 0x43,
    KEY_F11 = 0x44,
    KEY_F12 = 0x45,
    KEY_LEFT_CTRL = 0xE0,
    KEY_LEFT_SHIFT = 0xE1,
    KEY_LEFT_ALT = 0xE2,
    KEY_LEFT_GUI = 0xE3,
    KEY_RIGHT_CTRL = 0xE4,
    KEY_RIGHT_SHIFT = 0xE5,
    KEY_RIGHT_ALT = 0xE6,
    KEY_RIGHT_GUI = 0xE7
};

// 鼠标按键
enum class HID_MouseButton : uint8_t {
    LEFT = 0x01,
    RIGHT = 0x02,
    MIDDLE = 0x04,
    BUTTON4 = 0x08,
    BUTTON5 = 0x10
};

// 游戏手柄按键
enum class HID_GamepadButton : uint16_t {
    BUTTON_1 = 0x0001,
    BUTTON_2 = 0x0002,
    BUTTON_3 = 0x0004,
    BUTTON_4 = 0x0008,
    BUTTON_5 = 0x0010,
    BUTTON_6 = 0x0020,
    BUTTON_7 = 0x0040,
    BUTTON_8 = 0x0080,
    BUTTON_9 = 0x0100,
    BUTTON_10 = 0x0200,
    BUTTON_11 = 0x0400,
    BUTTON_12 = 0x0800,
    BUTTON_13 = 0x1000,
    BUTTON_14 = 0x2000,
    BUTTON_15 = 0x4000,
    BUTTON_16 = 0x8000
};

// 键盘报告结构
struct HID_KeyboardReport {
    uint8_t modifier;           // 修饰键 (Ctrl, Shift, Alt, GUI)
    uint8_t reserved;           // 保留字节
    uint8_t keys[6];           // 同时按下的按键 (最多6个)
    
    HID_KeyboardReport() : modifier(0), reserved(0) {
        memset(keys, 0, sizeof(keys));
    }
};

// 鼠标报告结构
struct HID_MouseReport {
    uint8_t buttons;            // 鼠标按键状态
    int8_t x;                   // X轴相对移动
    int8_t y;                   // Y轴相对移动
    int8_t wheel;               // 滚轮
    int8_t pan;                 // 水平滚轮
    
    HID_MouseReport() : buttons(0), x(0), y(0), wheel(0), pan(0) {}
};

// 游戏手柄报告结构
struct HID_GamepadReport {
    uint16_t buttons;           // 按键状态
    int8_t left_x;              // 左摇杆X轴
    int8_t left_y;              // 左摇杆Y轴
    int8_t right_x;             // 右摇杆X轴
    int8_t right_y;             // 右摇杆Y轴
    uint8_t left_trigger;       // 左扳机
    uint8_t right_trigger;      // 右扳机
    uint8_t dpad;               // 方向键
    
    HID_GamepadReport() : buttons(0), left_x(0), left_y(0), right_x(0), 
                         right_y(0), left_trigger(0), right_trigger(0), dpad(0) {}
};

// 触摸报告结构
struct HID_TouchPoint {
    uint16_t x;                 // X坐标
    uint16_t y;                 // Y坐标
    uint8_t pressure;           // 压力
    uint8_t contact_id;         // 触点ID
    bool in_contact;            // 是否接触
    bool tip_switch;            // 触笔开关
    
    HID_TouchPoint() : x(0), y(0), pressure(0), contact_id(0), 
                      in_contact(false), tip_switch(false) {}
};

struct HID_TouchReport {
    uint8_t contact_count;      // 触点数量
    HID_TouchPoint contacts[10]; // 最多10个触点
    
    HID_TouchReport() : contact_count(0) {}
};

// 自定义报告结构
struct HID_CustomReport {
    uint8_t report_id;          // 报告ID
    uint8_t data[64];           // 数据
    uint8_t length;             // 数据长度
    
    HID_CustomReport() : report_id(0), length(0) {
        memset(data, 0, sizeof(data));
    }
};

// HID配置结构
struct HID_Config {
    HID_DeviceType device_type; // 设备类型
    uint16_t vendor_id;         // 厂商ID
    uint16_t product_id;        // 产品ID
    std::string manufacturer;   // 制造商
    std::string product;        // 产品名称
    std::string serial_number;  // 序列号
    uint8_t report_interval_ms; // 报告间隔
    bool enable_boot_protocol;  // 启用引导协议
    
    HID_Config() 
        : device_type(HID_DeviceType::CUSTOM)
        , vendor_id(0x2E8A)
        , product_id(0x000A)
        , manufacturer("MaiMai Controller")
        , product("MaiMai Input Device")
        , serial_number("123456789")
        , report_interval_ms(1)
        , enable_boot_protocol(false) {}
};

// 回调函数类型
using HID_ReportCallback = std::function<void(HID_ReportType type, const uint8_t* data, uint8_t length)>;
using HID_ConnectCallback = std::function<void(bool connected)>;
using HID_ErrorCallback = std::function<void(const std::string& error)>;

// HID类
class HID {
public:
    explicit HID(HAL_USB* usb_hal);
    ~HID();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 配置管理
    bool set_config(const HID_Config& config);
    bool get_config(HID_Config& config);
    
    // 键盘功能
    bool send_keyboard_report(const HID_KeyboardReport& report);
    bool press_key(HID_KeyCode key, uint8_t modifier = 0);
    bool release_key(HID_KeyCode key);
    bool release_all_keys();
    bool type_string(const std::string& text);
    
    // 鼠标功能
    bool send_mouse_report(const HID_MouseReport& report);
    bool move_mouse(int8_t x, int8_t y);
    bool click_mouse(HID_MouseButton button);
    bool press_mouse_button(HID_MouseButton button);
    bool release_mouse_button(HID_MouseButton button);
    bool scroll_wheel(int8_t wheel, int8_t pan = 0);
    
    // 游戏手柄功能
    bool send_gamepad_report(const HID_GamepadReport& report);
    bool set_gamepad_button(HID_GamepadButton button, bool pressed);
    bool set_gamepad_axis(uint8_t axis, int8_t value);
    bool set_gamepad_trigger(uint8_t trigger, uint8_t value);
    bool set_gamepad_dpad(uint8_t direction);
    
    // 触摸功能
    bool send_touch_report(const HID_TouchReport& report);
    bool set_touch_point(uint8_t contact_id, uint16_t x, uint16_t y, uint8_t pressure = 255);
    bool release_touch_point(uint8_t contact_id);
    bool release_all_touch_points();
    
    // 自定义报告
    bool send_custom_report(const HID_CustomReport& report);
    bool send_raw_report(uint8_t report_id, const uint8_t* data, uint8_t length);
    
    // 状态查询
    bool is_connected() const;
    uint32_t get_report_count() const;
    uint32_t get_error_count() const;
    
    // 回调设置
    void set_report_callback(HID_ReportCallback callback);
    void set_connect_callback(HID_ConnectCallback callback);
    void set_error_callback(HID_ErrorCallback callback);
    
    // 任务处理
    void task();
    
    // 静态辅助方法
    static HID_KeyCode char_to_keycode(char c);
    static uint8_t char_to_modifier(char c);
    static std::vector<uint8_t> generate_hid_descriptor(HID_DeviceType device_type);
    
private:
    HAL_USB* usb_hal_;
    bool initialized_;
    HID_Config config_;
    
    // 当前状态
    HID_KeyboardReport current_keyboard_report_;
    HID_MouseReport current_mouse_report_;
    HID_GamepadReport current_gamepad_report_;
    HID_TouchReport current_touch_report_;
    
    // 统计信息
    uint32_t report_count_;
    uint32_t error_count_;
    uint32_t last_report_time_;
    
    // 回调
    HID_ReportCallback report_callback_;
    HID_ConnectCallback connect_callback_;
    HID_ErrorCallback error_callback_;
    
    // 内部方法
    bool send_report(HID_ReportType type, uint8_t report_id, const uint8_t* data, uint8_t length);
    void handle_received_report(HID_ReportType type, const uint8_t* data, uint8_t length);
    void handle_connection_change(bool connected);
    void handle_error(const std::string& error);
    
    // HID描述符生成
    std::vector<uint8_t> generate_keyboard_descriptor();
    std::vector<uint8_t> generate_mouse_descriptor();
    std::vector<uint8_t> generate_gamepad_descriptor();
    std::vector<uint8_t> generate_touch_descriptor();
    std::vector<uint8_t> generate_custom_descriptor();
    
    // 键盘辅助方法
    bool add_key_to_report(HID_KeyCode key);
    bool remove_key_from_report(HID_KeyCode key);
    void clear_keyboard_report();
    
    // 鼠标辅助方法
    void clear_mouse_report();
    
    // 游戏手柄辅助方法
    void clear_gamepad_report();
    
    // 触摸辅助方法
    HID_TouchPoint* find_touch_point(uint8_t contact_id);
    void clear_touch_report();
};

// 便利宏定义
#define HID_PRESS_KEY(hid, key) (hid)->press_key(HID_KeyCode::key)
#define HID_RELEASE_KEY(hid, key) (hid)->release_key(HID_KeyCode::key)
#define HID_CLICK_MOUSE(hid, button) (hid)->click_mouse(HID_MouseButton::button)
#define HID_PRESS_GAMEPAD(hid, button) (hid)->set_gamepad_button(HID_GamepadButton::button, true)
#define HID_RELEASE_GAMEPAD(hid, button) (hid)->set_gamepad_button(HID_GamepadButton::button, false)

#endif // HID_H