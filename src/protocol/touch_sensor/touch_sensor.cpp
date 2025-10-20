#include "touch_sensor.h"
#include "gtx312l/gtx312l.h"
#include "ad7147/ad7147.h"
#include "psoc/psoc.h"
#include "../../hal/i2c/hal_i2c.h"
#include "../usb_serial_logs/usb_serial_logs.h"

// TouchSensor基类实现文件

// 地址识别规则表 类型：匹配方式（Range/Exact/Mask）、起始地址、结束地址/精确地址/掩码
static constexpr TouchSensorAddressRule kTouchAddressRules[] = {
    { TouchSensorType::PSOC,    TouchSensorAddressRule::Match::Range, 0x08, 0x0E },
    { TouchSensorType::GTX312L, TouchSensorAddressRule::Match::Range, 0xB0, 0xB6 },
    { TouchSensorType::AD7147,  TouchSensorAddressRule::Match::Range, 0x2C, 0x2F },
};

/**
 * 获取当前模块支持的通道数量
 * @return 支持的最大通道数（1-24）
 */
uint32_t TouchSensor::getSupportedChannelCount() const {
    return supported_channel_count_;
}

// 静态方法实现

/**
 * 识别IC类型
 * @param i2c_address I2C地址
 * @return IC类型
 */
TouchSensorType TouchSensor::identifyICType(uint8_t i2c_address) {
    for (const auto& rule : kTouchAddressRules) {
        switch (rule.match) {
            case TouchSensorAddressRule::Match::Range:
                if (i2c_address >= rule.a && i2c_address <= rule.b) return rule.type;
                break;
            case TouchSensorAddressRule::Match::Exact:
                if (i2c_address == rule.a) return rule.type;
                break;
            case TouchSensorAddressRule::Match::Mask:
                if ((i2c_address & rule.a) == rule.b) return rule.type;
                break;
        }
    }
    return TouchSensorType::UNKNOWN;
}

/**
 * 扫描I2C总线上的触摸传感器设备
 * @param i2c_hal I2C HAL接口指针
 * @param i2c_bus I2C总线类型
 * @param max_devices 最大设备数量限制（默认8）
 * @return 扫描结果向量
 */
std::vector<TouchSensorScanResult> TouchSensor::scanDevices(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t max_devices) {
    std::vector<TouchSensorScanResult> results;
    
    if (!i2c_hal || max_devices == 0) {
        return results;
    }
    // 扫描I2C总线获取所有设备地址
    std::vector<uint8_t> addresses = i2c_hal->scan_devices();

    for (uint8_t addr : addresses) {
        if (results.size() >= max_devices) {
            break;
        }
        
        // 识别IC类型
        TouchSensorType type = identifyICType(addr);
        if (type == TouchSensorType::UNKNOWN) {
            USB_LOG_WARNING("Unknow TouchSensor BUS:%d ID:0x%02X", static_cast<int>(i2c_bus), addr);
            continue;
        }
        
        // 尝试创建传感器实例
        auto sensor = createSensor(type, i2c_hal, i2c_bus, addr);
        if (sensor) {
            TouchSensorScanResult result;
            result.type = type;
            result.i2c_address = addr;
            result.i2c_bus = i2c_bus;
            result.sensor = std::move(sensor);
            results.push_back(std::move(result));
        }
    }
    
    return results;
}

/**
 * 创建指定类型的触摸传感器实例
 * @param type IC类型
 * @param i2c_hal I2C HAL接口指针
 * @param i2c_bus I2C总线类型
 * @param i2c_address I2C地址
 * @return 触摸传感器实例指针（失败返回nullptr）
 */
std::unique_ptr<TouchSensor> TouchSensor::createSensor(TouchSensorType type, HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t i2c_address) {
    if (!i2c_hal) {
        return nullptr;
    }
    
    std::unique_ptr<TouchSensor> sensor = nullptr;
    
    switch (type) {
        case TouchSensorType::GTX312L:
            sensor = std::make_unique<GTX312L>(i2c_hal, i2c_bus, i2c_address);
            break;
            
        case TouchSensorType::AD7147:
            sensor = std::make_unique<AD7147>(i2c_hal, i2c_bus, i2c_address);
            break;

        case TouchSensorType::PSOC:
            sensor = std::make_unique<PSoC>(i2c_hal, i2c_bus, i2c_address);
            break;
            
        default:
            return nullptr;
    }
    
    // 直接创建实例并init，以init是否成功为准
    if (sensor && sensor->init()) {
        USB_LOG_DEBUG("InitSensorType:%d I2C_Bus:%d I2C_Address:0x%02X", static_cast<int>(type), static_cast<int>(i2c_bus), i2c_address);
        return sensor;
    }
    USB_LOG_DEBUG("Failed init SensorType:%d I2C_Bus:%d I2C_Address:0x%02X", static_cast<int>(type), static_cast<int>(i2c_bus), i2c_address);
    
    return nullptr;
}

// TouchSensorManager实现

/**
 * 扫描并注册所有I2C总线上的触摸传感器
 * @param i2c0_hal I2C0 HAL接口指针
 * @param i2c1_hal I2C1 HAL接口指针
 * @param max_devices 最大设备数量限制
 * @return 成功注册的设备数量
 */
uint8_t TouchSensorManager::scanAndRegisterAll(HAL_I2C* i2c0_hal, HAL_I2C* i2c1_hal, uint8_t max_devices) {
    clear();
    
    uint8_t registered_count = 0;
    // 扫描I2C0总线
    if (i2c0_hal && registered_count < max_devices) {
        auto i2c0_results = TouchSensor::scanDevices(i2c0_hal, I2C_Bus::I2C0, max_devices - registered_count);
        for (auto& result : i2c0_results) {
            if (result.sensor) {
                registered_sensors_.push_back(std::move(result.sensor));
                registered_count++;
            }
        }
    }
    
    // 扫描I2C1总线
    if (i2c1_hal && registered_count < max_devices) {
        auto i2c1_results = TouchSensor::scanDevices(i2c1_hal, I2C_Bus::I2C1, max_devices - registered_count);
        for (auto& result : i2c1_results) {
            if (result.sensor) {
                registered_sensors_.push_back(std::move(result.sensor));
                registered_count++;
            }
        }
    }
    
    return registered_count;
}