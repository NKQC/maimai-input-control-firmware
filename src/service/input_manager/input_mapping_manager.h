#pragma once

#include "../../hal/usb/hal_usb_types.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include <vector>
#include <map>
#include <functional>

/**
 * 输入映射管理器 - 专门负责输入映射的管理和处理
 * 将映射逻辑从InputManager中分离出来，提供更清晰的接口
 */

// 映射类型枚举
enum class MappingType {
    SERIAL,     // Mai2串口映射
    HID,        // HID坐标映射
    KEYBOARD    // 键盘映射
};

// 串口映射结构体
struct SerialMapping {
    uint16_t device_addr;
    uint8_t channel;
    Mai2_TouchArea area;
    
    SerialMapping() : device_addr(0), channel(0), area(MAI2_NO_USED) {}
    SerialMapping(uint16_t addr, uint8_t ch, Mai2_TouchArea a) 
        : device_addr(addr), channel(ch), area(a) {}
};

// HID映射结构体
struct HIDMapping {
    uint16_t device_addr;
    uint8_t channel;
    float x;
    float y;
    
    HIDMapping() : device_addr(0), channel(0), x(0.0f), y(0.0f) {}
    HIDMapping(uint16_t addr, uint8_t ch, float x_pos, float y_pos) 
        : device_addr(addr), channel(ch), x(x_pos), y(y_pos) {}
};

// 键盘映射结构体
struct KeyboardMapping {
    uint16_t device_addr;
    uint8_t channel;
    HID_KeyCode key;
    
    KeyboardMapping() : device_addr(0), channel(0), key(static_cast<HID_KeyCode>(HID_KEY_NONE)) {}
    KeyboardMapping(uint16_t addr, uint8_t ch, HID_KeyCode k) 
        : device_addr(addr), channel(ch), key(k) {}
};

// 映射键结构体（用于快速查找）
struct MappingKey {
    uint16_t device_addr;
    uint8_t channel;
    
    MappingKey(uint16_t addr, uint8_t ch) : device_addr(addr), channel(ch) {}
    
    bool operator<(const MappingKey& other) const {
        if (device_addr != other.device_addr) {
            return device_addr < other.device_addr;
        }
        return channel < other.channel;
    }
    
    bool operator==(const MappingKey& other) const {
        return device_addr == other.device_addr && channel == other.channel;
    }
};

// 映射事件回调类型
using SerialMappingCallback = std::function<void(Mai2_TouchArea area, bool pressed)>;
using HIDMappingCallback = std::function<void(float x, float y, bool pressed)>;
using KeyboardMappingCallback = std::function<void(HID_KeyCode key, bool pressed)>;

class InputMappingManager {
public:
    InputMappingManager();
    ~InputMappingManager();
    
    // 串口映射管理
    bool addSerialMapping(uint16_t device_addr, uint8_t channel, Mai2_TouchArea area);
    bool removeSerialMapping(uint16_t device_addr, uint8_t channel);
    Mai2_TouchArea getSerialMapping(uint16_t device_addr, uint8_t channel) const;
    std::vector<SerialMapping> getAllSerialMappings() const;
    
    // HID映射管理
    bool addHIDMapping(uint16_t device_addr, uint8_t channel, float x, float y);
    bool removeHIDMapping(uint16_t device_addr, uint8_t channel);
    HIDMapping getHIDMapping(uint16_t device_addr, uint8_t channel) const;
    std::vector<HIDMapping> getAllHIDMappings() const;
    
    // 键盘映射管理
    bool addKeyboardMapping(uint16_t device_addr, uint8_t channel, HID_KeyCode key);
    bool removeKeyboardMapping(uint16_t device_addr, uint8_t channel);
    HID_KeyCode getKeyboardMapping(uint16_t device_addr, uint8_t channel) const;
    std::vector<KeyboardMapping> getAllKeyboardMappings() const;
    
    // 批量操作
    void clearAllMappings();
    void clearDeviceMappings(uint16_t device_addr);
    void clearMappingsByType(MappingType type);
    
    // 映射查询
    bool hasMapping(uint16_t device_addr, uint8_t channel, MappingType type) const;
    MappingType getMappingType(uint16_t device_addr, uint8_t channel) const;
    size_t getMappingCount(MappingType type) const;
    
    // 事件处理
    void processTouch(uint16_t device_addr, uint8_t channel, bool pressed);
    void setSerialMappingCallback(SerialMappingCallback callback);
    void setHIDMappingCallback(HIDMappingCallback callback);
    void setKeyboardMappingCallback(KeyboardMappingCallback callback);
    
    // 配置导入导出
    struct MappingConfig {
        std::vector<SerialMapping> serial_mappings;
        std::vector<HIDMapping> hid_mappings;
        std::vector<KeyboardMapping> keyboard_mappings;
    };
    
    MappingConfig exportConfig() const;
    bool importConfig(const MappingConfig& config);
    
private:
    // 映射存储
    std::map<MappingKey, SerialMapping> serial_mappings_;
    std::map<MappingKey, HIDMapping> hid_mappings_;
    std::map<MappingKey, KeyboardMapping> keyboard_mappings_;
    
    // 事件回调
    SerialMappingCallback serial_callback_;
    HIDMappingCallback hid_callback_;
    KeyboardMappingCallback keyboard_callback_;
    
    // 内部辅助函数
    bool isValidMapping(uint16_t device_addr, uint8_t channel) const;
    void notifySerialMapping(Mai2_TouchArea area, bool pressed);
    void notifyHIDMapping(float x, float y, bool pressed);
    void notifyKeyboardMapping(HID_KeyCode key, bool pressed);
};