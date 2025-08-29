#pragma once

#include "../../protocol/touch_sensor/touch_sensor.h"
#include "../../protocol/touch_sensor/gtx312l/gtx312l.h"
#include "../../protocol/touch_sensor/ad7147/ad7147.h"
#include <vector>
#include <memory>
#include <functional>

/**
 * 输入设备管理器 - 专门负责设备的注册、管理和状态查询
 * 将设备管理从InputManager中分离出来，提供更清晰的接口
 */

// 设备状态结构体
struct TouchDeviceStatus {
    uint16_t device_addr;           // 设备地址
    std::string device_name;        // 设备名称
    bool is_initialized;            // 是否已初始化
    uint32_t enabled_channels;      // 启用的通道掩码
    uint32_t current_touch_state;   // 当前触摸状态
    uint32_t supported_channels;    // 支持的通道数量
    uint32_t timestamp;             // 最后更新时间戳
};

// 设备事件回调类型
using DeviceEventCallback = std::function<void(uint16_t device_addr, uint32_t touch_state)>;
using DeviceStatusCallback = std::function<void(uint16_t device_addr, bool connected)>;

class InputDeviceManager {
public:
    InputDeviceManager();
    ~InputDeviceManager();
    
    // 设备注册和管理
    bool registerTouchSensor(std::shared_ptr<TouchSensor> device, uint16_t device_addr);
    bool unregisterTouchSensor(uint16_t device_addr);
    void unregisterAllDevices();
    
    // 设备查询
    std::shared_ptr<TouchSensor> getDevice(uint16_t device_addr);
    std::vector<uint16_t> getRegisteredDeviceAddresses() const;
    bool isDeviceRegistered(uint16_t device_addr) const;
    
    // 设备状态管理
    bool initializeDevice(uint16_t device_addr);
    bool deinitializeDevice(uint16_t device_addr);
    void initializeAllDevices();
    void deinitializeAllDevices();
    
    // 状态查询
    TouchDeviceStatus getDeviceStatus(uint16_t device_addr) const;
    std::vector<TouchDeviceStatus> getAllDeviceStatus() const;
    uint32_t getDeviceTouchState(uint16_t device_addr) const;
    
    // 通道管理
    bool setChannelEnabled(uint16_t device_addr, uint8_t channel, bool enabled);
    bool isChannelEnabled(uint16_t device_addr, uint8_t channel) const;
    uint32_t getEnabledChannelsMask(uint16_t device_addr) const;
    
    // 事件回调
    void setDeviceEventCallback(DeviceEventCallback callback);
    void setDeviceStatusCallback(DeviceStatusCallback callback);
    
    // 更新循环
    void updateDeviceStates();
    
private:
    struct DeviceEntry {
        uint16_t device_addr;
        std::shared_ptr<TouchSensor> device;
        TouchDeviceStatus status;
        uint32_t last_touch_state;
        
        DeviceEntry(uint16_t addr, std::shared_ptr<TouchSensor> dev)
            : device_addr(addr), device(dev), last_touch_state(0) {
            status.device_addr = addr;
            status.device_name = dev ? dev->getDeviceName() : "Unknown";
            status.is_initialized = false;
            status.enabled_channels = 0;
            status.current_touch_state = 0;
            status.supported_channels = dev ? dev->getSupportedChannelCount() : 0;
            status.timestamp = 0;
        }
    };
    
    std::vector<DeviceEntry> devices_;
    DeviceEventCallback device_event_callback_;
    DeviceStatusCallback device_status_callback_;
    
    // 内部辅助函数
    DeviceEntry* findDevice(uint16_t device_addr);
    const DeviceEntry* findDevice(uint16_t device_addr) const;
    void notifyDeviceEvent(uint16_t device_addr, uint32_t touch_state);
    void notifyDeviceStatus(uint16_t device_addr, bool connected);
};