#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <cstdlib>
#include <functional>

// 前向声明
class HAL_I2C;
enum class I2C_Bus : uint8_t;
class TouchSensor;

/**
 * IC类型枚举 - 支持的触摸传感器IC类型
 */
enum class TouchSensorType : uint8_t {
    UNKNOWN = 0,
    GTX312L = 1,    // GTX312L触摸控制器
    AD7147 = 2      // AD7147触摸控制器
};

/**
 * IC反掩码定义 - 用于自动识别不同IC类型
 * 当(IC_ADDRESS & REVERSE_MASK) == 0时判定为匹配
 */
enum class TouchSensorReverseMask : uint8_t {
    GTX312L_MASK = 0x4F,  // GTX312L使用0xB*地址模式的反掩码
    AD7147_MASK = 0xD2    // AD7147使用0x2*地址模式的反掩码
};

/**
 * 采样模式枚举 - 触摸传感器采样模式
 */
enum class TouchSensorSampleMode : uint8_t {
    CONTINUOUS = 0,  // 连续采样模式
    SINGLE_SHOT = 1  // 单次采样模式
};

typedef struct {
    union {
        uint32_t touch_mask;     // 完整的32位触摸掩码
        struct {
            uint32_t channel_mask : 24;  // 低24位：通道bitmap (0-23)
            uint8_t module_mask : 8;      // 高8位：模块掩码
        };
    };
    uint32_t timestamp_us;   // 微秒级时间戳 返回0时代表采样失败
} TouchSampleResult;

// 异步采样结果回调函数类型定义
using async_touchsampleresult = std::function<void(const TouchSampleResult&)>;

/**
 * IC扫描结果结构体
 */
struct TouchSensorScanResult {
    TouchSensorType type;
    uint8_t i2c_address;
    I2C_Bus i2c_bus;
    std::unique_ptr<TouchSensor> sensor;
    
    TouchSensorScanResult() : type(TouchSensorType::UNKNOWN), i2c_address(0), sensor(nullptr) {}
};

/**
 * TouchSensor基类 - 触摸传感器统一接口
 * 提供触摸传感器模块的统一抽象接口
 * 支持最大24个触摸通道，使用24位掩码存储状态
 * 模块掩码为8位：bit7=I2C总线编号，bit6-0=I2C 7位地址
 */
class TouchSensor {
public:
    // 构造函数和析构函数
    TouchSensor(uint8_t max_channels) : max_channels_(max_channels), 
                                        module_mask_(0),
                                        supported_channel_count_(0) {}
    virtual ~TouchSensor() = default;

    // 纯虚函数接口 - 所有派生类必须实现
    
    /**
     * 异步采样接口：通过回调函数返回TouchSampleResult结构体
     * @param callback 异步采样完成后的回调函数
     */
    virtual void sample(async_touchsampleresult callback) = 0;
    
    /**
     * 获取当前模块支持的通道数量
     * @return 支持的最大通道数（1-24）
     */
    virtual uint32_t getSupportedChannelCount() const = 0;
    
    /**
     * 获取当前模块掩码
     * @return 8位模块掩码 (bit7=I2C总线编号, bit6-0=I2C 7位地址)
     */
    uint8_t getModuleMask() const {
        return module_mask_;
    };

    // 通用接口
    
    /**
     * 初始化触摸传感器
     * @return true=成功，false=失败
     */
    virtual bool init() = 0;
    
    /**
     * 反初始化触摸传感器
     */
    virtual void deinit() = 0;
    
    /**
     * 检查设备是否已初始化
     * @return true=已初始化，false=未初始化
     */
    virtual bool isInitialized() const = 0;
    
    // 通道控制接口 - 子类可选实现
    virtual bool setChannelEnabled(uint8_t channel, bool enabled) { return false; }  // 设置单个通道使能
    virtual bool getChannelEnabled(uint8_t channel) const { return false; }     // 获取单个通道使能状态
    virtual uint32_t getEnabledChannelMask() const { return (1UL << max_channels_) - 1; }  // 获取启用通道掩码，基于max_channels_自动生成（最大24通道）
    
    // 灵敏度控制接口 - 子类可选实现 (统一使用0-99范围)
    virtual bool setChannelSensitivity(uint8_t channel, uint8_t sensitivity) { return false; }  // 设置通道灵敏度 (0-99)
    virtual uint8_t getChannelSensitivity(uint8_t channel) const { return 50; }  // 获取通道灵敏度 (0-99，默认50)
    
    // 配置管理接口 - 子类可选实现
    virtual bool loadConfig(const std::string& config_data) { return false; }  // 从字符串加载配置
    virtual std::string saveConfig() const { return ""; }  // 保存配置到字符串
    
    // 校准相关接口 - 子类可选实现
    virtual bool calibrateSensor() { return false; }  // 校准传感器
    virtual bool calibrateSensor(uint8_t sensitivity_target) { return calibrateSensor(); }  // 校准传感器(带灵敏度目标)
    virtual bool setChannelCalibrationTarget(uint8_t channel, uint8_t sensitivity_target) { return false; }  // 设置指定通道的校准目标灵敏度 (不自动发起校准)
    virtual bool startCalibration() { return false; }  // 启动校准过程 (使用之前设置的通道目标)
    virtual uint8_t getCalibrationProgress() const { return 0; }  // 获取校准进度 (0-255)
    virtual bool setLEDEnabled(bool enabled) { return false; }  // 设置LED状态
    
    // 自动校准控制接口 - 子类可选实现
    virtual void setAutoCalibration(bool enable) { }  // 控制自动校准启停
    
    // 异常通道检测接口 - 子类可选实现
    virtual uint32_t getAbnormalChannelMask() const { return 0; }  // 获取异常通道bitmap (格式同sample返回的channel_mask)
    
    // 校准支持标志
    bool supports_calibration_ = false;  // 该芯片是否支持校准功能

protected:
    uint8_t max_channels_;  // 该IC支持的最大通道数（最大24）
    
    // 模块掩码相关成员变量
    uint8_t module_mask_;              // 8位模块掩码 (bit7=I2C总线编号, bit6-0=I2C 7位地址)
    uint32_t supported_channel_count_; // 支持的通道数量
    
    /**
     * 配置保存器结构体 - 优化的顺序存储版本
     * 使用顺序存储，大幅减少存储开销，无键名存储
     */
    struct SaveConfig {
        std::string config_data;
        mutable size_t read_pos = 0;  // 读取位置指针
        
        // 顺序写入函数 - 按调用顺序写入值
        void writeValue(uint32_t value) {
            if (!config_data.empty()) config_data += ",";
            config_data += std::to_string(value);
        }
        
        void writeValue(int32_t value) {
            if (!config_data.empty()) config_data += ",";
            config_data += std::to_string(value);
        }
        
        void writeValue(uint16_t value) {
            if (!config_data.empty()) config_data += ",";
            config_data += std::to_string(value);
        }
        
        void writeValue(int16_t value) {
            if (!config_data.empty()) config_data += ",";
            config_data += std::to_string(value);
        }
        
        void writeValue(uint8_t value) {
            if (!config_data.empty()) config_data += ",";
            config_data += std::to_string(static_cast<unsigned int>(value));
        }
        
        void writeValue(int8_t value) {
            if (!config_data.empty()) config_data += ",";
            config_data += std::to_string(static_cast<int>(value));
        }
        
        void writeValue(float value) {
            if (!config_data.empty()) config_data += ",";
            config_data += std::to_string(value);
        }
        
        void writeValue(bool value) {
            if (!config_data.empty()) config_data += ",";
            config_data += (value ? "1" : "0");
        }
        
        void writeValue(const std::string& value) {
            if (!config_data.empty()) config_data += ",";
            config_data += value;
        }
        
        // 顺序读取函数 - 按调用顺序读取值
        uint32_t readValue(uint32_t default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            char* end;
            unsigned long result = std::strtoul(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<uint32_t>(result);
        }
        
        int32_t readValue(int32_t default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            char* end;
            long result = std::strtol(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<int32_t>(result);
        }
        
        uint16_t readValue(uint16_t default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            char* end;
            unsigned long result = std::strtoul(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<uint16_t>(result);
        }
        
        int16_t readValue(int16_t default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            char* end;
            long result = std::strtol(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<int16_t>(result);
        }
        
        uint8_t readValue(uint8_t default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            char* end;
            unsigned long result = std::strtoul(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<uint8_t>(result);
        }
        
        int8_t readValue(int8_t default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            char* end;
            long result = std::strtol(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<int8_t>(result);
        }
        
        float readValue(float default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            char* end;
            float result = std::strtof(value.c_str(), &end);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return result;
        }
        
        bool readValue(bool default_value) const {
            std::string value = getNextValue();
            if (value.empty()) return default_value;
            return (value == "1" || value == "true");
        }
        
        std::string readValue(const std::string& default_value) const {
            std::string value = getNextValue();
            return value.empty() ? default_value : value;
        }
        
        // 序列化为字符串
        std::string toString() const {
            return config_data;
        }
        
        // 从字符串反序列化
        bool fromString(const std::string& data) {
            config_data = data;
            read_pos = 0;  // 重置读取位置
            return true;
        }
        
        // 清空配置
        void clear() {
            config_data.clear();
            read_pos = 0;
        }
        
        // 重置读取位置
        void resetReadPosition() const {
            read_pos = 0;
        }
        
    private:
        // 获取下一个值
        std::string getNextValue() const {
            if (read_pos >= config_data.length()) {
                return "";
            }
            
            size_t comma_pos = config_data.find(',', read_pos);
            std::string value;
            
            if (comma_pos != std::string::npos) {
                value = config_data.substr(read_pos, comma_pos - read_pos);
                read_pos = comma_pos + 1;
            } else {
                value = config_data.substr(read_pos);
                read_pos = config_data.length();
            }
            
            return value;
        }
    };

    /**
     * 生成模块掩码
     * @param i2c_bus I2C总线编号 (0或1)
     * @param i2c_address 7位I2C地址
     * @return 8位模块掩码
     */
    static uint8_t generateModuleMask(uint8_t i2c_bus, uint8_t i2c_address) {
        return ((i2c_bus & 0x01) << 7) | (i2c_address & 0x7F);
    }

    std::string module_name;
    
public:
    // 公共静态接口 - IC自动扫描和识别
    
    /**
     * 识别IC类型
     * @param i2c_address I2C地址
     * @return IC类型
     */
    static TouchSensorType identifyICType(uint8_t i2c_address);
    
    /**
     * 扫描I2C总线上的触摸传感器设备
     * @param i2c_hal I2C HAL接口指针
     * @param i2c_bus I2C总线类型
     * @param max_devices 最大设备数量限制（默认8）
     * @return 扫描结果向量
     */
    static std::vector<TouchSensorScanResult> scanDevices(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t max_devices = 8);
    
    /**
     * 创建指定类型的触摸传感器实例
     * @param type IC类型
     * @param i2c_hal I2C HAL接口指针
     * @param i2c_bus I2C总线类型
     * @param i2c_address I2C地址
     * @return 触摸传感器实例指针（失败返回nullptr）
     */
    static std::unique_ptr<TouchSensor> createSensor(TouchSensorType type, HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t i2c_address);

    std::string getDeviceName() const { 
        char hex[3];
        snprintf(hex, sizeof(hex), "%02x", module_mask_);
        return module_name + "_" + hex;
    }

    /**
     * 从模块掩码中提取I2C总线编号
     * @param module_mask 8位模块掩码
     * @return I2C总线编号 (0或1)
     */
    static uint8_t extractI2CBusFromMask(uint8_t module_mask) {
        return (module_mask >> 7) & 0x01;
    }
    
    /**
     * 从模块掩码中提取I2C地址
     * @param module_mask 8位模块掩码
     * @return 7位I2C地址
     */
    static uint8_t extractI2CAddressFromMask(uint8_t module_mask) {
        return module_mask & 0x7F;
    }
};

/**
 * 触摸传感器管理器 - 统一管理多个触摸传感器
 */
class TouchSensorManager {
public:
    TouchSensorManager() = default;
    ~TouchSensorManager() = default;
    
    /**
     * 扫描并注册所有I2C总线上的触摸传感器
     * @param i2c0_hal I2C0 HAL接口指针
     * @param i2c1_hal I2C1 HAL接口指针
     * @param max_devices 最大设备数量限制
     * @return 成功注册的设备数量
     */
    uint8_t scanAndRegisterAll(HAL_I2C* i2c0_hal, HAL_I2C* i2c1_hal, uint8_t max_devices = 8);
    
    /**
     * 获取已注册的传感器数量
     */
    uint8_t getRegisteredCount() const { return registered_sensors_.size(); }
    
    /**
     * 获取指定索引的传感器
     */
    TouchSensor* getSensor(uint8_t index) const {
        return (index < registered_sensors_.size()) ? registered_sensors_[index].get() : nullptr;
    }
    
    /**
     * 清空所有已注册的传感器
     */
    void clear() { registered_sensors_.clear(); }
    
private:
    std::vector<std::unique_ptr<TouchSensor>> registered_sensors_;
};