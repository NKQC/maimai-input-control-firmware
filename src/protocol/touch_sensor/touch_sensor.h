#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <cstdlib>

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
    GTX312L_MASK = 0x4F,  // GTX312L使用0xB*地址模式的反掩码 (B=1011, 反掩码=0100)
    AD7147_MASK = 0xEF    // AD7147使用0x2*地址模式的反掩码 (2=0010, 反掩码=1110)
};

typedef struct {
    union {
        uint32_t touch_mask;     // 完整的32位触摸掩码
        struct {
            uint32_t channel_mask : 24;  // 低24位：通道bitmap (0-23)
            uint8_t module_mask : 8;      // 高8位：模块掩码
        };
    };
    uint32_t timestamp_us;   // 微秒级时间戳
} TouchSampleResult;

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
     * 统一采样接口：返回TouchSampleResult结构体
     * @return TouchSampleResult 包含通道掩码、模块掩码和微秒时间戳
     */
    virtual TouchSampleResult sample() = 0;
    
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

protected:
    uint8_t max_channels_;  // 该IC支持的最大通道数（最大24）
    
    // 模块掩码相关成员变量
    uint8_t module_mask_;              // 8位模块掩码 (bit7=I2C总线编号, bit6-0=I2C 7位地址)
    uint32_t supported_channel_count_; // 支持的通道数量
    
    /**
     * 配置管理器结构体 - 提供通用的配置存储和转换功能
     * 使用字符串存储，支持多种数据类型的重载读写函数
     */
    struct ConfigManager {
        std::string config_data;
        
        // Write functions for different types
        void setConfig(const std::string& key, int32_t value) {
            setConfigInternal(key, std::to_string(value));
        }
        
        void setConfig(const std::string& key, uint32_t value) {
            setConfigInternal(key, std::to_string(value));
        }
        
        void setConfig(const std::string& key, int16_t value) {
            setConfigInternal(key, std::to_string(value));
        }
        
        void setConfig(const std::string& key, uint16_t value) {
            setConfigInternal(key, std::to_string(value));
        }
        
        void setConfig(const std::string& key, int8_t value) {
            setConfigInternal(key, std::to_string(static_cast<int>(value)));
        }
        
        void setConfig(const std::string& key, uint8_t value) {
            setConfigInternal(key, std::to_string(static_cast<unsigned int>(value)));
        }
        
        void setConfig(const std::string& key, float value) {
            setConfigInternal(key, std::to_string(value));
        }
        
        void setConfig(const std::string& key, bool value) {
             setConfigInternal(key, value ? "1" : "0");
         }
        
        void setConfig(const std::string& key, const std::string& value) {
            setConfigInternal(key, value);
        }
        
        // Read functions for different types
        int32_t getConfig(const std::string& key, int32_t default_value) const {
            std::string value = getConfigInternal(key);
            if (value.empty()) return default_value;
            char* end;
            long result = std::strtol(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<int32_t>(result);
        }
        
        uint32_t getConfig(const std::string& key, uint32_t default_value) const {
            std::string value = getConfigInternal(key);
            if (value.empty()) return default_value;
            char* end;
            unsigned long result = std::strtoul(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<uint32_t>(result);
        }
        
        int16_t getConfig(const std::string& key, int16_t default_value) const {
            std::string value = getConfigInternal(key);
            if (value.empty()) return default_value;
            char* end;
            long result = std::strtol(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<int16_t>(result);
        }
        
        uint16_t getConfig(const std::string& key, uint16_t default_value) const {
            std::string value = getConfigInternal(key);
            if (value.empty()) return default_value;
            char* end;
            unsigned long result = std::strtoul(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<uint16_t>(result);
        }
        
        int8_t getConfig(const std::string& key, int8_t default_value) const {
            std::string value = getConfigInternal(key);
            if (value.empty()) return default_value;
            char* end;
            long result = std::strtol(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<int8_t>(result);
        }
        
        uint8_t getConfig(const std::string& key, uint8_t default_value) const {
            std::string value = getConfigInternal(key);
            if (value.empty()) return default_value;
            char* end;
            unsigned long result = std::strtoul(value.c_str(), &end, 10);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return static_cast<uint8_t>(result);
        }
        
        float getConfig(const std::string& key, float default_value) const {
            std::string value = getConfigInternal(key);
            if (value.empty()) return default_value;
            char* end;
            float result = std::strtof(value.c_str(), &end);
            if (end == value.c_str() || *end != '\0') {
                return default_value;
            }
            return result;
        }
        
        bool getConfig(const std::string& key, bool default_value) const {
             std::string value = getConfigInternal(key);
             if (value.empty()) return default_value;
             return (value == "1" || value == "true");
         }
        
        std::string getConfig(const std::string& key, const std::string& default_value) const {
            std::string value = getConfigInternal(key);
            return value.empty() ? default_value : value;
        }
        
        // 序列化为字符串
        std::string toString() const {
            return config_data;
        }
        
        // 从字符串反序列化
        bool fromString(const std::string& data) {
            config_data = data;
            return true;
        }
        
        // 清空配置
        void clear() {
            config_data.clear();
        }
        
        // 检查是否包含指定配置
        bool hasConfig(const std::string& key) const {
            return !getConfigInternal(key).empty();
        }
        
    private:
        void setConfigInternal(const std::string& key, const std::string& value) {
            // Remove existing key if present
            std::string search_key = key + "=";
            size_t start_pos = config_data.find(search_key);
            if (start_pos != std::string::npos) {
                size_t end_pos = config_data.find('\n', start_pos);
                if (end_pos != std::string::npos) {
                    config_data.erase(start_pos, end_pos - start_pos + 1);
                } else {
                    config_data.erase(start_pos);
                }
            }
            
            // Add new key-value pair
            if (!config_data.empty() && config_data.back() != '\n') {
                config_data += "\n";
            }
            config_data += key + "=" + value + "\n";
        }
        
        std::string getConfigInternal(const std::string& key) const {
            std::string search_key = key + "=";
            size_t start_pos = config_data.find(search_key);
            if (start_pos == std::string::npos) {
                return "";
            }
            
            start_pos += search_key.length();
            size_t end_pos = config_data.find('\n', start_pos);
            if (end_pos == std::string::npos) {
                return config_data.substr(start_pos);
            }
            
            return config_data.substr(start_pos, end_pos - start_pos);
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