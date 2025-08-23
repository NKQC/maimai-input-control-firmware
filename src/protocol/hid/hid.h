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
    TOUCH = 4
};

// HID报告类型
enum class HID_ReportType : uint8_t {
    INPUT = 1,
    OUTPUT = 2,
    FEATURE = 3
};

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

// 遍历所有支持的HID键码
#define SUPPORTED_KEYS_COUNT 64
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
    HID_KeyCode::KEY_F11, HID_KeyCode::KEY_F12, HID_KeyCode::KEY_JOYSTICK_A,
    HID_KeyCode::KEY_JOYSTICK_B, HID_KeyCode::KEY_JOYSTICK_CONFIRM, HID_KeyCode::KEY_LEFT_CTRL,
    HID_KeyCode::KEY_LEFT_SHIFT, HID_KeyCode::KEY_LEFT_ALT, HID_KeyCode::KEY_LEFT_GUI,
    HID_KeyCode::KEY_RIGHT_CTRL, HID_KeyCode::KEY_RIGHT_SHIFT, HID_KeyCode::KEY_RIGHT_ALT,
    HID_KeyCode::KEY_RIGHT_GUI
};

// 键盘bitmap结构体 - 使用union 128位高效编码键盘状态
union KeyboardBitmap {
    struct {
        uint64_t bitmap_low;   // 低64位bitmap
        uint64_t bitmap_high;  // 高64位bitmap
    };
    uint64_t bitmap[2];  // 兼容性访问，支持128个按键
    struct {
        uint8_t KEY_NONE : 1;     // bit 0
        uint8_t KEY_A : 1;        // bit 1
        uint8_t KEY_B : 1;        // bit 2
        uint8_t KEY_C : 1;        // bit 3
        uint8_t KEY_D : 1;        // bit 4
        uint8_t KEY_E : 1;        // bit 5
        uint8_t KEY_F : 1;        // bit 6
        uint8_t KEY_G : 1;        // bit 7
        uint8_t KEY_H : 1;        // bit 8
        uint8_t KEY_I : 1;        // bit 9
        uint8_t KEY_J : 1;        // bit 10
        uint8_t KEY_K : 1;        // bit 11
        uint8_t KEY_L : 1;        // bit 12
        uint8_t KEY_M : 1;        // bit 13
        uint8_t KEY_N : 1;        // bit 14
        uint8_t KEY_O : 1;        // bit 15
        uint8_t KEY_P : 1;        // bit 16
        uint8_t KEY_Q : 1;        // bit 17
        uint8_t KEY_R : 1;        // bit 18
        uint8_t KEY_S : 1;        // bit 19
        uint8_t KEY_T : 1;        // bit 20
        uint8_t KEY_U : 1;        // bit 21
        uint8_t KEY_V : 1;        // bit 22
        uint8_t KEY_W : 1;        // bit 23
        uint8_t KEY_X : 1;        // bit 24
        uint8_t KEY_Y : 1;        // bit 25
        uint8_t KEY_Z : 1;        // bit 26
        uint8_t KEY_1 : 1;        // bit 27
        uint8_t KEY_2 : 1;        // bit 28
        uint8_t KEY_3 : 1;        // bit 29
        uint8_t KEY_4 : 1;        // bit 30
        uint8_t KEY_5 : 1;        // bit 31
        uint8_t KEY_6 : 1;        // bit 32
        uint8_t KEY_7 : 1;        // bit 33
        uint8_t KEY_8 : 1;        // bit 34
        uint8_t KEY_9 : 1;        // bit 35
        uint8_t KEY_0 : 1;        // bit 36
        uint8_t KEY_ENTER : 1;    // bit 37
        uint8_t KEY_ESCAPE : 1;   // bit 38
        uint8_t KEY_BACKSPACE : 1; // bit 39
        uint8_t KEY_TAB : 1;      // bit 40
        uint8_t KEY_SPACE : 1;    // bit 41
        uint8_t KEY_F1 : 1;       // bit 42
        uint8_t KEY_F2 : 1;       // bit 43
        uint8_t KEY_F3 : 1;       // bit 44
        uint8_t KEY_F4 : 1;       // bit 45
        uint8_t KEY_F5 : 1;       // bit 46
        uint8_t KEY_F6 : 1;       // bit 47
        uint8_t KEY_F7 : 1;       // bit 48
        uint8_t KEY_F8 : 1;       // bit 49
        uint8_t KEY_F9 : 1;       // bit 50
        uint8_t KEY_F10 : 1;      // bit 51
        uint8_t KEY_F11 : 1;      // bit 52
        uint8_t KEY_F12 : 1;      // bit 53
        uint8_t KEY_LEFT_CTRL : 1; // bit 54
        uint8_t KEY_LEFT_SHIFT : 1; // bit 55
        uint8_t KEY_LEFT_ALT : 1;  // bit 56
        uint8_t KEY_LEFT_GUI : 1;  // bit 57
        uint8_t KEY_RIGHT_CTRL : 1; // bit 58
        uint8_t KEY_RIGHT_SHIFT : 1; // bit 59
        uint8_t KEY_RIGHT_ALT : 1;  // bit 60
        uint8_t KEY_RIGHT_GUI : 1;  // bit 61
        uint8_t KEY_JOYSTICK_A : 1; // bit 62
        uint8_t KEY_JOYSTICK_B : 1; // bit 63
        uint8_t KEY_JOYSTICK_CONFIRM : 1; // bit 64 (高64位的第0位)
        uint8_t reserved : 7;      // bit 65-71 保留
    } keys;
    
    KeyboardBitmap() : bitmap_low(0), bitmap_high(0) {}
    
    KeyboardBitmap& operator|=(const KeyboardBitmap& other) {
        bitmap_low |= other.bitmap_low;
        bitmap_high |= other.bitmap_high;
        return *this;
    }

    // 设置按键状态
    void setKey(HID_KeyCode key, bool pressed) {
        uint8_t bit_index = getBitIndex(key);
        if (bit_index < 64) {
            if (pressed) {
                bitmap_low |= (1ULL << bit_index);
            } else {
                bitmap_low &= ~(1ULL << bit_index);
            }
        } else if (bit_index < 128) {
            uint8_t high_bit = bit_index - 64;
            if (pressed) {
                bitmap_high |= (1ULL << high_bit);
            } else {
                bitmap_high &= ~(1ULL << high_bit);
            }
        }
    }
    
    // volatile版本的setKey方法
    void setKey(HID_KeyCode key, bool pressed) volatile {
        uint8_t bit_index = getBitIndex(key);
        if (bit_index < 64) {
            if (pressed) {
                bitmap_low |= (1ULL << bit_index);
            } else {
                bitmap_low &= ~(1ULL << bit_index);
            }
        } else if (bit_index < 128) {
            uint8_t high_bit = bit_index - 64;
            if (pressed) {
                bitmap_high |= (1ULL << high_bit);
            } else {
                bitmap_high &= ~(1ULL << high_bit);
            }
        }
    }
    
    // 获取按键状态
    bool getKey(HID_KeyCode key) const {
        uint8_t bit_index = getBitIndex(key);
        if (bit_index < 64) {
            return (bitmap_low & (1ULL << bit_index)) != 0;
        } else if (bit_index < 128) {
            uint8_t high_bit = bit_index - 64;
            return (bitmap_high & (1ULL << high_bit)) != 0;
        }
        return false;
    }
    
    // 清空所有按键
    void clear() {
        bitmap_low = 0;
        bitmap_high = 0;
    }
    
    // 清空所有按键（volatile版本）
    void clear() volatile {
        bitmap_low = 0;
        bitmap_high = 0;
    }
    
    // 获取HID_KeyCode对应的位索引
    uint8_t getBitIndex(HID_KeyCode key) const volatile {
        if ((uint8_t)key < 3) return 0;
        if ((uint8_t)key < 44) return (uint8_t)key - 3;
        if ((uint8_t)key < 70) return (uint8_t)key - 16;
        return (uint8_t)key - 54;
    }
};



// 键盘报告结构 (移除reserved字段)
struct HID_KeyboardReport {
    uint8_t modifier;           // 修饰键 (Ctrl, Shift, Alt, GUI)
    uint8_t keys[6];           // 同时按下的按键 (最多6个)
    
    HID_KeyboardReport() : modifier(0) {
        memset(keys, 0, sizeof(keys));
    }
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
        : device_type(HID_DeviceType::KEYBOARD)
        , vendor_id(0x2E8A)
        , product_id(0x000A)
        , manufacturer("MaiMai Controller")
        , product("MaiMai Input Device")
        , serial_number("123456789")
        , report_interval_ms(1)
        , enable_boot_protocol(false) {}
};

// 回调函数类型定义
using HID_ReportCallback = std::function<void(HID_ReportType type, const uint8_t* data, uint8_t length)>;
using HID_ConnectCallback = std::function<void(bool connected)>;
using HID_ErrorCallback = std::function<void(const std::string& error)>;

// HID类 - 单例模式
class HID {
public:
    // 获取单例实例
    static HID* getInstance();
    
    // 禁用拷贝构造和赋值操作
    HID(const HID&) = delete;
    HID& operator=(const HID&) = delete;
    
    ~HID();
    
    // 初始化和反初始化
    bool init(HAL_USB* usb_hal);
    void deinit();
    bool is_ready() const;
    
    // 配置管理
    bool set_config(const HID_Config& config);
    bool get_config(HID_Config& config);
    
    // 键盘功能
    bool send_keyboard_report(const HID_KeyboardReport& report);
    bool send_keyboard_data(const KeyboardBitmap& bitmap);  // 高效发送键盘bitmap数据
    bool press_key(HID_KeyCode key, uint8_t modifier = 0);
    bool release_key(HID_KeyCode key);
    bool release_all_keys();
    bool type_string(const std::string& text);
    
    // 触摸功能
    bool send_touch_report(const HID_TouchReport& report);
    bool set_touch_point(uint8_t contact_id, uint16_t x, uint16_t y, uint8_t pressure = 255);
    bool release_touch_point(uint8_t contact_id);
    bool release_all_touch_points();
    
    // 状态查询
    bool is_connected() const;
    uint32_t get_report_count() const;
    uint32_t get_error_count() const;
    
    // 回调设置
    void set_report_callback(HID_ReportCallback callback);
    void set_connect_callback(HID_ConnectCallback callback);
    void set_error_callback(HID_ErrorCallback callback);
    
    // 任务循环
    void task();
    
    // 静态工具函数
    static HID_KeyCode char_to_keycode(char c);
    static uint8_t char_to_modifier(char c);
    static std::vector<uint8_t> generate_hid_descriptor(HID_DeviceType device_type);
    
private:
    // 私有构造函数
    HID();
    
    // 单例实例
    static HID* instance_;
    
    HAL_USB* usb_hal_;
    bool initialized_;
    HID_Config config_;
    
    // 当前报告状态
    HID_KeyboardReport current_keyboard_report_;
    HID_TouchReport current_touch_report_;
    
    // 统计信息
    uint32_t report_count_;
    uint32_t error_count_;
    uint32_t last_report_time_;
    
    // 回调函数
    HID_ReportCallback report_callback_;
    HID_ConnectCallback connect_callback_;
    HID_ErrorCallback error_callback_;
    
    // 内部方法
    bool send_report(HID_ReportType type, uint8_t report_id, const uint8_t* data, uint8_t length);
    void handle_received_report(HID_ReportType type, const uint8_t* data, uint8_t length);
    void handle_connection_change(bool connected);
    void handle_error(const std::string& error);
    
    // 描述符生成
    std::vector<uint8_t> generate_keyboard_descriptor();
    std::vector<uint8_t> generate_touch_descriptor();
    
    // 键盘报告处理
    bool add_key_to_report(HID_KeyCode key);
    bool remove_key_from_report(HID_KeyCode key);
    void clear_keyboard_report();
    
    // 触摸报告处理
    HID_TouchPoint* find_touch_point(uint8_t contact_id);
    void clear_touch_report();
};

// 便利宏定义
#define HID_PRESS_KEY(key) HID::getInstance()->press_key(HID_KeyCode::key)
#define HID_RELEASE_KEY(key) HID::getInstance()->release_key(HID_KeyCode::key)

#endif // HID_H