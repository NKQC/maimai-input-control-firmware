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

// 遍历所有支持的HID键码
static const HID_KeyCode supported_keys[] = {
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

// 键盘bitmap结构体 - 使用union uint64_t高效编码键盘状态
union KeyboardBitmap {
    uint64_t bitmap;  // 64位bitmap，支持64个按键
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
        uint8_t reserved : 2;      // bit 62-63 保留
    } keys;
    
    KeyboardBitmap() : bitmap(0) {}
    
    // 设置按键状态
    void setKey(HID_KeyCode key, bool pressed) {
        uint8_t bit_index = getBitIndex(key);
        if (pressed) {
            bitmap |= (1ULL << bit_index);
        } else {
            bitmap &= ~(1ULL << bit_index);
        }
    }
    
    // volatile版本的setKey方法
    void setKey(HID_KeyCode key, bool pressed) volatile {
        uint8_t bit_index = getBitIndex(key);
        if (bit_index < 64) {
            if (pressed) {
                bitmap |= (1ULL << bit_index);
            } else {
                bitmap &= ~(1ULL << bit_index);
            }
        }
    }
    
    // 获取按键状态
    bool getKey(HID_KeyCode key) const {
        uint8_t bit_index = getBitIndex(key);
        if (bit_index < 64) {
            return (bitmap & (1ULL << bit_index)) != 0;
        }
        return false;
    }
    
    // 清空所有按键
    void clear() {
        bitmap = 0;
    }
    
    // 清空所有按键（volatile版本）
    void clear() volatile {
        bitmap = 0;
    }
    
private:
    // 获取HID_KeyCode对应的位索引
    uint8_t getBitIndex(HID_KeyCode key) const volatile {
        switch (key) {
            case HID_KeyCode::KEY_NONE: return 0;
            case HID_KeyCode::KEY_A: return 1;
            case HID_KeyCode::KEY_B: return 2;
            case HID_KeyCode::KEY_C: return 3;
            case HID_KeyCode::KEY_D: return 4;
            case HID_KeyCode::KEY_E: return 5;
            case HID_KeyCode::KEY_F: return 6;
            case HID_KeyCode::KEY_G: return 7;
            case HID_KeyCode::KEY_H: return 8;
            case HID_KeyCode::KEY_I: return 9;
            case HID_KeyCode::KEY_J: return 10;
            case HID_KeyCode::KEY_K: return 11;
            case HID_KeyCode::KEY_L: return 12;
            case HID_KeyCode::KEY_M: return 13;
            case HID_KeyCode::KEY_N: return 14;
            case HID_KeyCode::KEY_O: return 15;
            case HID_KeyCode::KEY_P: return 16;
            case HID_KeyCode::KEY_Q: return 17;
            case HID_KeyCode::KEY_R: return 18;
            case HID_KeyCode::KEY_S: return 19;
            case HID_KeyCode::KEY_T: return 20;
            case HID_KeyCode::KEY_U: return 21;
            case HID_KeyCode::KEY_V: return 22;
            case HID_KeyCode::KEY_W: return 23;
            case HID_KeyCode::KEY_X: return 24;
            case HID_KeyCode::KEY_Y: return 25;
            case HID_KeyCode::KEY_Z: return 26;
            case HID_KeyCode::KEY_1: return 27;
            case HID_KeyCode::KEY_2: return 28;
            case HID_KeyCode::KEY_3: return 29;
            case HID_KeyCode::KEY_4: return 30;
            case HID_KeyCode::KEY_5: return 31;
            case HID_KeyCode::KEY_6: return 32;
            case HID_KeyCode::KEY_7: return 33;
            case HID_KeyCode::KEY_8: return 34;
            case HID_KeyCode::KEY_9: return 35;
            case HID_KeyCode::KEY_0: return 36;
            case HID_KeyCode::KEY_ENTER: return 37;
            case HID_KeyCode::KEY_ESCAPE: return 38;
            case HID_KeyCode::KEY_BACKSPACE: return 39;
            case HID_KeyCode::KEY_TAB: return 40;
            case HID_KeyCode::KEY_SPACE: return 41;
            case HID_KeyCode::KEY_F1: return 42;
            case HID_KeyCode::KEY_F2: return 43;
            case HID_KeyCode::KEY_F3: return 44;
            case HID_KeyCode::KEY_F4: return 45;
            case HID_KeyCode::KEY_F5: return 46;
            case HID_KeyCode::KEY_F6: return 47;
            case HID_KeyCode::KEY_F7: return 48;
            case HID_KeyCode::KEY_F8: return 49;
            case HID_KeyCode::KEY_F9: return 50;
            case HID_KeyCode::KEY_F10: return 51;
            case HID_KeyCode::KEY_F11: return 52;
            case HID_KeyCode::KEY_F12: return 53;
            case HID_KeyCode::KEY_LEFT_CTRL: return 54;
            case HID_KeyCode::KEY_LEFT_SHIFT: return 55;
            case HID_KeyCode::KEY_LEFT_ALT: return 56;
            case HID_KeyCode::KEY_LEFT_GUI: return 57;
            case HID_KeyCode::KEY_RIGHT_CTRL: return 58;
            case HID_KeyCode::KEY_RIGHT_SHIFT: return 59;
            case HID_KeyCode::KEY_RIGHT_ALT: return 60;
            case HID_KeyCode::KEY_RIGHT_GUI: return 61;
            default: return 255; // 无效索引
        }
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