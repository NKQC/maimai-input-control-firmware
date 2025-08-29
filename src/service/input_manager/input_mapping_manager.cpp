#include "input_mapping_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include <algorithm>

InputMappingManager::InputMappingManager() 
    : serial_callback_(nullptr)
    , hid_callback_(nullptr)
    , keyboard_callback_(nullptr) {
}

InputMappingManager::~InputMappingManager() {
    clearAllMappings();
}

// 添加串口映射
bool InputMappingManager::addSerialMapping(uint16_t device_addr, uint8_t channel, Mai2_TouchArea area) {
    if (!isValidMapping(device_addr, channel) || area == MAI2_NO_USED) {
        return false;
    }
    
    MappingKey key(device_addr, channel);
    
    // 检查是否已存在其他类型的映射
    if (hid_mappings_.find(key) != hid_mappings_.end() || 
        keyboard_mappings_.find(key) != keyboard_mappings_.end()) {
        USB_LOG_TAG_WARNING("InputMappingManager", 
                           "Channel 0x%04X:%d already has different mapping type", 
                           device_addr, channel);
        return false;
    }
    
    serial_mappings_[key] = SerialMapping(device_addr, channel, area);
    
    USB_LOG_TAG_INFO("InputMappingManager", 
                     "Added serial mapping: 0x%04X:%d -> Area %d", 
                     device_addr, channel, static_cast<int>(area));
    
    return true;
}

// 移除串口映射
bool InputMappingManager::removeSerialMapping(uint16_t device_addr, uint8_t channel) {
    MappingKey key(device_addr, channel);
    auto it = serial_mappings_.find(key);
    
    if (it == serial_mappings_.end()) {
        return false;
    }
    
    serial_mappings_.erase(it);
    
    USB_LOG_TAG_INFO("InputMappingManager", 
                     "Removed serial mapping: 0x%04X:%d", 
                     device_addr, channel);
    
    return true;
}

// 获取串口映射
Mai2_TouchArea InputMappingManager::getSerialMapping(uint16_t device_addr, uint8_t channel) const {
    MappingKey key(device_addr, channel);
    auto it = serial_mappings_.find(key);
    
    return (it != serial_mappings_.end()) ? it->second.area : MAI2_NO_USED;
}

// 获取所有串口映射
std::vector<SerialMapping> InputMappingManager::getAllSerialMappings() const {
    std::vector<SerialMapping> mappings;
    mappings.reserve(serial_mappings_.size());
    
    for (const auto& pair : serial_mappings_) {
        mappings.push_back(pair.second);
    }
    
    return mappings;
}

// 添加HID映射
bool InputMappingManager::addHIDMapping(uint16_t device_addr, uint8_t channel, float x, float y) {
    if (!isValidMapping(device_addr, channel) || x < 0.0f || x > 1.0f || y < 0.0f || y > 1.0f) {
        return false;
    }
    
    MappingKey key(device_addr, channel);
    
    // 检查是否已存在其他类型的映射
    if (serial_mappings_.find(key) != serial_mappings_.end() || 
        keyboard_mappings_.find(key) != keyboard_mappings_.end()) {
        USB_LOG_TAG_WARNING("InputMappingManager", 
                           "Channel 0x%04X:%d already has different mapping type", 
                           device_addr, channel);
        return false;
    }
    
    hid_mappings_[key] = HIDMapping(device_addr, channel, x, y);
    
    USB_LOG_TAG_INFO("InputMappingManager", 
                     "Added HID mapping: 0x%04X:%d -> (%.3f, %.3f)", 
                     device_addr, channel, x, y);
    
    return true;
}

// 移除HID映射
bool InputMappingManager::removeHIDMapping(uint16_t device_addr, uint8_t channel) {
    MappingKey key(device_addr, channel);
    auto it = hid_mappings_.find(key);
    
    if (it == hid_mappings_.end()) {
        return false;
    }
    
    hid_mappings_.erase(it);
    
    USB_LOG_TAG_INFO("InputMappingManager", 
                     "Removed HID mapping: 0x%04X:%d", 
                     device_addr, channel);
    
    return true;
}

// 获取HID映射
HIDMapping InputMappingManager::getHIDMapping(uint16_t device_addr, uint8_t channel) const {
    MappingKey key(device_addr, channel);
    auto it = hid_mappings_.find(key);
    
    return (it != hid_mappings_.end()) ? it->second : HIDMapping();
}

// 获取所有HID映射
std::vector<HIDMapping> InputMappingManager::getAllHIDMappings() const {
    std::vector<HIDMapping> mappings;
    mappings.reserve(hid_mappings_.size());
    
    for (const auto& pair : hid_mappings_) {
        mappings.push_back(pair.second);
    }
    
    return mappings;
}

// 添加键盘映射
bool InputMappingManager::addKeyboardMapping(uint16_t device_addr, uint8_t channel, HID_KeyCode key) {
    if (!isValidMapping(device_addr, channel) || key == HID_KeyCode::KEY_NONE) {
        return false;
    }
    
    MappingKey mapping_key(device_addr, channel);
    
    // 检查是否已存在其他类型的映射
    if (serial_mappings_.find(mapping_key) != serial_mappings_.end() || 
        hid_mappings_.find(mapping_key) != hid_mappings_.end()) {
        USB_LOG_TAG_WARNING("InputMappingManager", 
                           "Channel 0x%04X:%d already has different mapping type", 
                           device_addr, channel);
        return false;
    }
    
    keyboard_mappings_[mapping_key] = KeyboardMapping(device_addr, channel, key);
    
    USB_LOG_TAG_INFO("InputMappingManager", 
                     "Added keyboard mapping: 0x%04X:%d -> Key %d", 
                     device_addr, channel, static_cast<int>(key));
    
    return true;
}

// 移除键盘映射
bool InputMappingManager::removeKeyboardMapping(uint16_t device_addr, uint8_t channel) {
    MappingKey key(device_addr, channel);
    auto it = keyboard_mappings_.find(key);
    
    if (it == keyboard_mappings_.end()) {
        return false;
    }
    
    keyboard_mappings_.erase(it);
    
    USB_LOG_TAG_INFO("InputMappingManager", 
                     "Removed keyboard mapping: 0x%04X:%d", 
                     device_addr, channel);
    
    return true;
}

// 获取键盘映射
HID_KeyCode InputMappingManager::getKeyboardMapping(uint16_t device_addr, uint8_t channel) const {
    MappingKey key(device_addr, channel);
    auto it = keyboard_mappings_.find(key);
    
    return (it != keyboard_mappings_.end()) ? it->second.key : HID_KeyCode::KEY_NONE;
}

// 获取所有键盘映射
std::vector<KeyboardMapping> InputMappingManager::getAllKeyboardMappings() const {
    std::vector<KeyboardMapping> mappings;
    mappings.reserve(keyboard_mappings_.size());
    
    for (const auto& pair : keyboard_mappings_) {
        mappings.push_back(pair.second);
    }
    
    return mappings;
}

// 清除所有映射
void InputMappingManager::clearAllMappings() {
    serial_mappings_.clear();
    hid_mappings_.clear();
    keyboard_mappings_.clear();
    
    USB_LOG_TAG_INFO("InputMappingManager", "All mappings cleared");
}

// 清除设备映射
void InputMappingManager::clearDeviceMappings(uint16_t device_addr) {
    // 清除串口映射
    auto serial_it = serial_mappings_.begin();
    while (serial_it != serial_mappings_.end()) {
        if (serial_it->first.device_addr == device_addr) {
            serial_it = serial_mappings_.erase(serial_it);
        } else {
            ++serial_it;
        }
    }
    
    // 清除HID映射
    auto hid_it = hid_mappings_.begin();
    while (hid_it != hid_mappings_.end()) {
        if (hid_it->first.device_addr == device_addr) {
            hid_it = hid_mappings_.erase(hid_it);
        } else {
            ++hid_it;
        }
    }
    
    // 清除键盘映射
    auto keyboard_it = keyboard_mappings_.begin();
    while (keyboard_it != keyboard_mappings_.end()) {
        if (keyboard_it->first.device_addr == device_addr) {
            keyboard_it = keyboard_mappings_.erase(keyboard_it);
        } else {
            ++keyboard_it;
        }
    }
    
    USB_LOG_TAG_INFO("InputMappingManager", "Cleared mappings for device 0x%04X", device_addr);
}

// 按类型清除映射
void InputMappingManager::clearMappingsByType(MappingType type) {
    switch (type) {
        case MappingType::SERIAL:
            serial_mappings_.clear();
            USB_LOG_TAG_INFO("InputMappingManager", "Serial mappings cleared");
            break;
        case MappingType::HID:
            hid_mappings_.clear();
            USB_LOG_TAG_INFO("InputMappingManager", "HID mappings cleared");
            break;
        case MappingType::KEYBOARD:
            keyboard_mappings_.clear();
            USB_LOG_TAG_INFO("InputMappingManager", "Keyboard mappings cleared");
            break;
    }
}

// 检查是否有映射
bool InputMappingManager::hasMapping(uint16_t device_addr, uint8_t channel, MappingType type) const {
    MappingKey key(device_addr, channel);
    
    switch (type) {
        case MappingType::SERIAL:
            return serial_mappings_.find(key) != serial_mappings_.end();
        case MappingType::HID:
            return hid_mappings_.find(key) != hid_mappings_.end();
        case MappingType::KEYBOARD:
            return keyboard_mappings_.find(key) != keyboard_mappings_.end();
    }
    
    return false;
}

// 获取映射类型
MappingType InputMappingManager::getMappingType(uint16_t device_addr, uint8_t channel) const {
    MappingKey key(device_addr, channel);
    
    if (serial_mappings_.find(key) != serial_mappings_.end()) {
        return MappingType::SERIAL;
    }
    if (hid_mappings_.find(key) != hid_mappings_.end()) {
        return MappingType::HID;
    }
    if (keyboard_mappings_.find(key) != keyboard_mappings_.end()) {
        return MappingType::KEYBOARD;
    }
    
    // 没有找到映射，返回默认值
    return MappingType::SERIAL;
}

// 获取映射数量
size_t InputMappingManager::getMappingCount(MappingType type) const {
    switch (type) {
        case MappingType::SERIAL:
            return serial_mappings_.size();
        case MappingType::HID:
            return hid_mappings_.size();
        case MappingType::KEYBOARD:
            return keyboard_mappings_.size();
    }
    
    return 0;
}

// 处理触摸事件
void InputMappingManager::processTouch(uint16_t device_addr, uint8_t channel, bool pressed) {
    MappingKey key(device_addr, channel);
    
    // 检查串口映射
    auto serial_it = serial_mappings_.find(key);
    if (serial_it != serial_mappings_.end()) {
        notifySerialMapping(serial_it->second.area, pressed);
        return;
    }
    
    // 检查HID映射
    auto hid_it = hid_mappings_.find(key);
    if (hid_it != hid_mappings_.end()) {
        notifyHIDMapping(hid_it->second.x, hid_it->second.y, pressed);
        return;
    }
    
    // 检查键盘映射
    auto keyboard_it = keyboard_mappings_.find(key);
    if (keyboard_it != keyboard_mappings_.end()) {
        notifyKeyboardMapping(keyboard_it->second.key, pressed);
        return;
    }
}

// 设置串口映射回调
void InputMappingManager::setSerialMappingCallback(SerialMappingCallback callback) {
    serial_callback_ = callback;
}

// 设置HID映射回调
void InputMappingManager::setHIDMappingCallback(HIDMappingCallback callback) {
    hid_callback_ = callback;
}

// 设置键盘映射回调
void InputMappingManager::setKeyboardMappingCallback(KeyboardMappingCallback callback) {
    keyboard_callback_ = callback;
}

// 导出配置
InputMappingManager::MappingConfig InputMappingManager::exportConfig() const {
    MappingConfig config;
    config.serial_mappings = getAllSerialMappings();
    config.hid_mappings = getAllHIDMappings();
    config.keyboard_mappings = getAllKeyboardMappings();
    return config;
}

// 导入配置
bool InputMappingManager::importConfig(const MappingConfig& config) {
    clearAllMappings();
    
    // 导入串口映射
    for (const auto& mapping : config.serial_mappings) {
        addSerialMapping(mapping.device_addr, mapping.channel, mapping.area);
    }
    
    // 导入HID映射
    for (const auto& mapping : config.hid_mappings) {
        addHIDMapping(mapping.device_addr, mapping.channel, mapping.x, mapping.y);
    }
    
    // 导入键盘映射
    for (const auto& mapping : config.keyboard_mappings) {
        addKeyboardMapping(mapping.device_addr, mapping.channel, mapping.key);
    }
    
    USB_LOG_TAG_INFO("InputMappingManager", "Configuration imported successfully");
    return true;
}

// 验证映射有效性
bool InputMappingManager::isValidMapping(uint16_t device_addr, uint8_t channel) const {
    return device_addr != 0 && channel < 32; // 假设最大32个通道
}

// 通知串口映射事件
void InputMappingManager::notifySerialMapping(Mai2_TouchArea area, bool pressed) {
    if (serial_callback_) {
        serial_callback_(area, pressed);
    }
}

// 通知HID映射事件
void InputMappingManager::notifyHIDMapping(float x, float y, bool pressed) {
    if (hid_callback_) {
        hid_callback_(x, y, pressed);
    }
}

// 通知键盘映射事件
void InputMappingManager::notifyKeyboardMapping(HID_KeyCode key, bool pressed) {
    if (keyboard_callback_) {
        keyboard_callback_(key, pressed);
    }
}