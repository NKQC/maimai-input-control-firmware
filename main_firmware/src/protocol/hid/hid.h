#ifndef HID_H
#define HID_H

#include "../../hal/usb/hal_usb.h"
#include <functional>
#include <cstdint>
#include <vector>
#include <array>
#include <cstring>


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
    inline void setKey(HID_KeyCode key, bool pressed) {
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
    inline void setKey(HID_KeyCode key, bool pressed) volatile {
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
    inline bool getKey(HID_KeyCode key) const {
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
    
    // 获取内部位图数据指针，用于位运算优化
    inline const uint8_t* getData() const {
        return reinterpret_cast<const uint8_t*>(bitmap);
    }
    
    // 获取HID_KeyCode对应的位索引 - 基于supported_keys数组的索引
    inline uint8_t getBitIndex(HID_KeyCode key) const volatile {
        // 遍历supported_keys数组找到对应的索引
        for (uint8_t i = 0; i < SUPPORTED_KEYS_COUNT; i++) {
            if (supported_keys[i] == key) {
                return i + 1;
            }
        }
        return 0; // 如果找不到，返回0（KEY_NONE的位置）
    }
};

// HID类 - 单例模式
class HID {
public:
    // 获取单例实例
    static HID* getInstance();
    
    // 禁用拷贝构造和赋值操作
    HID(const HID&) = delete;
    HID& operator=(const HID&) = delete;
    
    ~HID();
    
    // 初始化和清理
    bool init(HAL_USB* hal_usb);
    void deinit();
    bool is_initialized() const;
    
    // 键盘相关
    bool press_key(HID_KeyCode key);
    bool release_key(HID_KeyCode key);
    void clear_keyboard_state();  // 清空键盘状态并发送空报文
    void force_send_keyboard_report(); // 强制发送键盘报文
    
    // 触摸操作
    bool send_touch_report(const HID_TouchPoint& report);
    void force_send_touch_report(); // 强制发送触摸报文
    
    // 状态查询
    uint32_t get_report_rate() const;

    void task();
    
    // 静态工具函数
    static HID_KeyCode char_to_keycode(char c);
    static uint8_t char_to_modifier(char c);

private:
    // 构造函数
    HID();
    
    // 键盘状态
    struct HID_Keyboard_state_t {
        uint8_t key_count;                    // 当前按下的按键数量
        uint8_t modifier_keys;                // 修饰键状态位图 (Ctrl, Shift, Alt, GUI)
        HID_KeyCode key[KEYBOARD_SIMUL_PRESS]; // 按键数组
        
        // 触发式发送相关
        bool state_changed;                   // 状态是否发生变化
        uint8_t last_key_count;              // 上次的按键数量
        uint8_t last_modifier_keys;          // 上次的修饰键状态
        HID_KeyCode last_key[KEYBOARD_SIMUL_PRESS]; // 上次的按键数组

        void clear() {
            key_count = 0;
            modifier_keys = 0;
            state_changed = true;  // 清空状态时标记为已变化
            for (int32_t i = 0; i < KEYBOARD_SIMUL_PRESS; i++) {
                key[i] = HID_KeyCode::KEY_NONE;
            }
            update_last_state();
        }
        
        bool add(HID_KeyCode _key) {
            // 避免添加无效按键
            if (_key == HID_KeyCode::KEY_NONE) {
                return false;
            }
            
            // 检查是否为修饰键
            if (isModifierKey(_key)) {
                uint8_t old_modifier = modifier_keys;
                setModifierKey(_key, true);
                if (modifier_keys != old_modifier) {
                    state_changed = true;
                }
                return true;
            }
            
            // 检查是否已存在该按键
            for (int32_t i = 0; i < key_count; i++) {
                if (key[i] == _key) {
                    return true; // 按键已存在，不重复添加
                }
            }
            
            // 普通按键处理
            if (key_count < KEYBOARD_SIMUL_PRESS) {
                key[key_count] = _key;
                key_count++;
                state_changed = true;
                return true;
            }
            return false;
        }
        
        bool remove(HID_KeyCode _key) {
            // 避免移除无效按键
            if (_key == HID_KeyCode::KEY_NONE) {
                return false;
            }
            
            // 检查是否为修饰键
            if (isModifierKey(_key)) {
                uint8_t old_modifier = modifier_keys;
                setModifierKey(_key, false);
                if (modifier_keys != old_modifier) {
                    state_changed = true;
                }
                return true;
            }
            
            // 普通按键处理
            for (int32_t i = 0; i < key_count; i++) {
                if (key[i] == _key) {
                    for (int32_t j = i; j < key_count - 1; j++) {
                        key[j] = key[j + 1];
                    }
                    key[key_count - 1] = HID_KeyCode::KEY_NONE;
                    key_count--;
                    state_changed = true;
                    return true;
                }
            }
            return false;
        }

        // 检查状态是否发生变化
        bool has_state_changed() const {
            return state_changed;
        }

        // 更新上次状态
        void update_last_state() {
            last_key_count = key_count;
            last_modifier_keys = modifier_keys;
            for (int32_t i = 0; i < KEYBOARD_SIMUL_PRESS; i++) {
                last_key[i] = key[i];
            }
            state_changed = false;
        }
        
    private:
        bool isModifierKey(HID_KeyCode _key) const {
            return (_key >= HID_KeyCode::KEY_LEFT_CTRL && _key <= HID_KeyCode::KEY_RIGHT_GUI);
        }
        
        void setModifierKey(HID_KeyCode _key, bool pressed) {
            uint8_t bit = 0;
            switch (_key) {
                case HID_KeyCode::KEY_LEFT_CTRL:  bit = 0; break;
                case HID_KeyCode::KEY_LEFT_SHIFT: bit = 1; break;
                case HID_KeyCode::KEY_LEFT_ALT:   bit = 2; break;
                case HID_KeyCode::KEY_LEFT_GUI:   bit = 3; break;
                case HID_KeyCode::KEY_RIGHT_CTRL: bit = 4; break;
                case HID_KeyCode::KEY_RIGHT_SHIFT:bit = 5; break;
                case HID_KeyCode::KEY_RIGHT_ALT:  bit = 6; break;
                case HID_KeyCode::KEY_RIGHT_GUI:  bit = 7; break;
                default: return;
            }
            
            if (pressed) {
                modifier_keys |= (1 << bit);
            } else {
                modifier_keys &= ~(1 << bit);
            }
        }
        
        // 检查是否有按键按下
        bool has_keys_pressed() const {
            return key_count > 0 || modifier_keys != 0;
        }
    };

    struct HID_Touch_state_t {
        uint8_t press_modifier = 0, release_modifier = 0;
        // 位移结构要求最后一位保留空位
        HID_TouchPoint touch_press[TOUCH_LOCAL_NUM + 1] = {};
        HID_TouchPoint touch_release[TOUCH_LOCAL_NUM + 1] = {};
        // 数据流: press: press -> touch_press
        // release: touch_press -> touch_release
        // remove: touch_release -> X
        bool press(HID_TouchPoint _touch) {
            if (press_modifier < TOUCH_LOCAL_NUM) {
                touch_press[press_modifier] = _touch;
                press_modifier++;
                return true;
            }
            return false;
        }
        bool release(uint8_t _id) {
            for (int32_t i = 0; i < press_modifier; i++) {
                if (touch_press[i].id == _id) {
                    // 将触摸点移动到release数组
                    if (release_modifier < TOUCH_LOCAL_NUM) {
                        touch_release[release_modifier] = touch_press[i];
                        release_modifier++;
                    }
                    
                    // 从press数组中移除
                    for (int32_t j = i; j < press_modifier - 1; j++) {
                        touch_press[j] = touch_press[j + 1];
                    }
                    press_modifier--;
                    
                    // 清空最后一个位置
                    touch_press[press_modifier].clear();
                    return true;
                }
            }
            return false;
        }
        void remove(uint8_t _id) {
            for (int32_t i = 0; i < release_modifier; i++) {
                if (touch_release[i].id == _id) {
                    touch_release[i] = {0, 0, 0, 0};
                    for (int32_t j = i; j < release_modifier - 1; j++) {
                        touch_release[j] = touch_release[j + 1];
                    }
                    release_modifier--;
                    return;
                }
            }
        }
    };

    // 静态实例
    static HID* instance_;
    
    bool initialized_;
    HAL_USB* hal_usb_;

    const HID_ReportID keyboard_id[KEYBOARD_NUM] = {
        HID_ReportID::REPORT_ID_KEYBOARD1,
        HID_ReportID::REPORT_ID_KEYBOARD2,
        HID_ReportID::REPORT_ID_KEYBOARD3,
    };

    HID_Keyboard_state_t keyboard_state;
    HID_Touch_state_t touch_state;

    // 回报速率统计
    uint32_t report_count_;
    uint32_t last_report_time_;
    uint32_t cached_report_rate_;

    inline void report_keyboard();
    inline void report_touch(uint32_t _now);
    
    // 触发式发送相关
    bool keyboard_needs_send_;     // 键盘是否需要发送报文
    bool touch_needs_send_;        // 触摸是否需要发送报文
    uint32_t last_keyboard_send_;  // 上次键盘报文发送时间
    uint32_t last_touch_send_;     // 上次触摸报文发送时间

};

#endif // HID_H