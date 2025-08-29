#include "input_device_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include <pico/time.h>
#include <algorithm>

InputDeviceManager::InputDeviceManager() 
    : device_event_callback_(nullptr)
    , device_status_callback_(nullptr) {
}

InputDeviceManager::~InputDeviceManager() {
    deinitializeAllDevices();
    devices_.clear();
}

// 注册触摸传感器设备
bool InputDeviceManager::registerTouchSensor(std::shared_ptr<TouchSensor> device, uint16_t device_addr) {
    if (!device) {
        USB_LOG_TAG_ERROR("InputDeviceManager", "Cannot register null device");
        return false;
    }
    
    // 检查设备是否已注册
    if (isDeviceRegistered(device_addr)) {
        USB_LOG_TAG_WARNING("InputDeviceManager", "Device 0x%04X already registered", device_addr);
        return false;
    }
    
    // 添加设备
    devices_.emplace_back(device_addr, device);
    
    USB_LOG_TAG_INFO("InputDeviceManager", "Registered device: %s (0x%04X)", 
                     device->getDeviceName().c_str(), device_addr);
    
    // 通知设备状态变化
    notifyDeviceStatus(device_addr, true);
    
    return true;
}

// 注销触摸传感器设备
bool InputDeviceManager::unregisterTouchSensor(uint16_t device_addr) {
    auto it = std::find_if(devices_.begin(), devices_.end(),
                          [device_addr](const DeviceEntry& entry) {
                              return entry.device_addr == device_addr;
                          });
    
    if (it == devices_.end()) {
        return false;
    }
    
    // 先去初始化设备
    if (it->status.is_initialized) {
        deinitializeDevice(device_addr);
    }
    
    USB_LOG_TAG_INFO("InputDeviceManager", "Unregistered device: %s (0x%04X)", 
                     it->device->getDeviceName().c_str(), device_addr);
    
    // 通知设备状态变化
    notifyDeviceStatus(device_addr, false);
    
    // 移除设备
    devices_.erase(it);
    
    return true;
}

// 注销所有设备
void InputDeviceManager::unregisterAllDevices() {
    deinitializeAllDevices();
    
    for (const auto& entry : devices_) {
        notifyDeviceStatus(entry.device_addr, false);
    }
    
    devices_.clear();
    USB_LOG_TAG_INFO("InputDeviceManager", "All devices unregistered");
}

// 获取设备
std::shared_ptr<TouchSensor> InputDeviceManager::getDevice(uint16_t device_addr) {
    DeviceEntry* entry = findDevice(device_addr);
    return entry ? entry->device : nullptr;
}

// 获取已注册设备地址列表
std::vector<uint16_t> InputDeviceManager::getRegisteredDeviceAddresses() const {
    std::vector<uint16_t> addresses;
    addresses.reserve(devices_.size());
    
    for (const auto& entry : devices_) {
        addresses.push_back(entry.device_addr);
    }
    
    return addresses;
}

// 检查设备是否已注册
bool InputDeviceManager::isDeviceRegistered(uint16_t device_addr) const {
    return findDevice(device_addr) != nullptr;
}

// 初始化设备
bool InputDeviceManager::initializeDevice(uint16_t device_addr) {
    DeviceEntry* entry = findDevice(device_addr);
    if (!entry || !entry->device) {
        return false;
    }
    
    if (entry->status.is_initialized) {
        return true; // 已经初始化
    }
    
    bool success = entry->device->init();
    if (success) {
        entry->status.is_initialized = true;
        entry->status.enabled_channels = entry->device->getEnabledModuleMask();
        entry->status.supported_channels = entry->device->getSupportedChannelCount();
        
        USB_LOG_TAG_INFO("InputDeviceManager", "Device initialized: %s (0x%04X)", 
                         entry->device->getDeviceName().c_str(), device_addr);
    } else {
        USB_LOG_TAG_ERROR("InputDeviceManager", "Failed to initialize device: %s (0x%04X)", 
                          entry->device->getDeviceName().c_str(), device_addr);
    }
    
    return success;
}

// 去初始化设备
bool InputDeviceManager::deinitializeDevice(uint16_t device_addr) {
    DeviceEntry* entry = findDevice(device_addr);
    if (!entry || !entry->device) {
        return false;
    }
    
    if (!entry->status.is_initialized) {
        return true; // 已经去初始化
    }
    
    entry->device->deinit();
    entry->status.is_initialized = false;
    entry->status.enabled_channels = 0;
    entry->status.current_touch_state = 0;
    entry->last_touch_state = 0;
    
    USB_LOG_TAG_INFO("InputDeviceManager", "Device deinitialized: %s (0x%04X)", 
                     entry->device->getDeviceName().c_str(), device_addr);
    
    return true;
}

// 初始化所有设备
void InputDeviceManager::initializeAllDevices() {
    for (auto& entry : devices_) {
        initializeDevice(entry.device_addr);
    }
}

// 去初始化所有设备
void InputDeviceManager::deinitializeAllDevices() {
    for (auto& entry : devices_) {
        deinitializeDevice(entry.device_addr);
    }
}

// 获取设备状态
TouchDeviceStatus InputDeviceManager::getDeviceStatus(uint16_t device_addr) const {
    const DeviceEntry* entry = findDevice(device_addr);
    if (entry) {
        return entry->status;
    }
    
    // 返回空状态
    TouchDeviceStatus empty_status = {};
    empty_status.device_addr = device_addr;
    return empty_status;
}

// 获取所有设备状态
std::vector<TouchDeviceStatus> InputDeviceManager::getAllDeviceStatus() const {
    std::vector<TouchDeviceStatus> statuses;
    statuses.reserve(devices_.size());
    
    for (const auto& entry : devices_) {
        statuses.push_back(entry.status);
    }
    
    return statuses;
}

// 获取设备触摸状态
uint32_t InputDeviceManager::getDeviceTouchState(uint16_t device_addr) const {
    const DeviceEntry* entry = findDevice(device_addr);
    return entry ? entry->status.current_touch_state : 0;
}

// 设置通道使能状态
bool InputDeviceManager::setChannelEnabled(uint16_t device_addr, uint8_t channel, bool enabled) {
    DeviceEntry* entry = findDevice(device_addr);
    if (!entry || !entry->device || !entry->status.is_initialized) {
        return false;
    }
    
    // 更新本地状态
    if (enabled) {
        entry->status.enabled_channels |= (1UL << channel);
    } else {
        entry->status.enabled_channels &= ~(1UL << channel);
    }
    
    return true;
}

// 检查通道是否启用
bool InputDeviceManager::isChannelEnabled(uint16_t device_addr, uint8_t channel) const {
    const DeviceEntry* entry = findDevice(device_addr);
    if (!entry) {
        return false;
    }
    
    return (entry->status.enabled_channels >> channel) & 0x01;
}

// 获取启用通道掩码
uint32_t InputDeviceManager::getEnabledChannelsMask(uint16_t device_addr) const {
    const DeviceEntry* entry = findDevice(device_addr);
    return entry ? entry->status.enabled_channels : 0;
}

// 设置设备事件回调
void InputDeviceManager::setDeviceEventCallback(DeviceEventCallback callback) {
    device_event_callback_ = callback;
}

// 设置设备状态回调
void InputDeviceManager::setDeviceStatusCallback(DeviceStatusCallback callback) {
    device_status_callback_ = callback;
}

// 更新设备状态
void InputDeviceManager::updateDeviceStates() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    for (auto& entry : devices_) {
        if (!entry.status.is_initialized || !entry.device) {
            continue;
        }
        
        // 获取当前触摸状态
        uint32_t current_state = entry.device->getCurrentTouchState();
        
        // 检查状态是否发生变化
        if (current_state != entry.last_touch_state) {
            entry.status.current_touch_state = current_state;
            entry.status.timestamp = current_time;
            entry.last_touch_state = current_state;
            
            // 通知状态变化
            notifyDeviceEvent(entry.device_addr, current_state);
        }
    }
}

// 查找设备（非const版本）
InputDeviceManager::DeviceEntry* InputDeviceManager::findDevice(uint16_t device_addr) {
    auto it = std::find_if(devices_.begin(), devices_.end(),
                          [device_addr](const DeviceEntry& entry) {
                              return entry.device_addr == device_addr;
                          });
    return (it != devices_.end()) ? &(*it) : nullptr;
}

// 查找设备（const版本）
const InputDeviceManager::DeviceEntry* InputDeviceManager::findDevice(uint16_t device_addr) const {
    auto it = std::find_if(devices_.begin(), devices_.end(),
                          [device_addr](const DeviceEntry& entry) {
                              return entry.device_addr == device_addr;
                          });
    return (it != devices_.end()) ? &(*it) : nullptr;
}

// 通知设备事件
void InputDeviceManager::notifyDeviceEvent(uint16_t device_addr, uint32_t touch_state) {
    if (device_event_callback_) {
        device_event_callback_(device_addr, touch_state);
    }
}

// 通知设备状态变化
void InputDeviceManager::notifyDeviceStatus(uint16_t device_addr, bool connected) {
    if (device_status_callback_) {
        device_status_callback_(device_addr, connected);
    }
}