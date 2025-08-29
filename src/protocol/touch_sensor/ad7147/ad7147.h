#pragma once

#include "../../../hal/i2c/hal_i2c.h"
#include "../touch_sensor.h"
#include <stdint.h>
#include <string>

/**
 * 协议层 - AD7147触摸控制器
 * 基于I2C通信的电容式触摸控制器
 * 支持最多13个触摸通道
 * 工作电压：2.6V-5.5V，支持I2C接口
 */

// AD7147寄存器定义
#define AD7147_I2C_ADDR_DEFAULT     0x2C    // 默认I2C地址
#define AD7147_MAX_CHANNELS         13      // 最大触摸通道数

// 主要寄存器地址
#define AD7147_REG_PWR_CONTROL      0x00    // 电源控制寄存器
#define AD7147_REG_STAGE_CAL_EN     0x01    // 阶段校准使能寄存器
#define AD7147_REG_AMB_COMP_CTRL0   0x02    // 环境补偿控制寄存器0
#define AD7147_REG_AMB_COMP_CTRL1   0x03    // 环境补偿控制寄存器1
#define AD7147_REG_AMB_COMP_CTRL2   0x04    // 环境补偿控制寄存器2
#define AD7147_REG_STAGE_LOW_INT_EN 0x05    // 阶段低中断使能寄存器
#define AD7147_REG_STAGE_HIGH_INT_EN 0x06   // 阶段高中断使能寄存器
#define AD7147_REG_STAGE_COMPLETE_INT_EN 0x07 // 阶段完成中断使能寄存器
#define AD7147_REG_STAGE_LOW_LIMIT_INT 0x08   // 阶段低限制中断寄存器
#define AD7147_REG_STAGE_HIGH_LIMIT_INT 0x09  // 阶段高限制中断寄存器
#define AD7147_REG_STAGE_COMPLETE_LIMIT_INT 0x0A // 阶段完成限制中断寄存器

// 触摸数据结构体
struct AD7147_TouchData {
    uint16_t touch_status;      // 13位触摸状态（位0-12对应通道0-12）
    uint32_t timestamp;         // 时间戳
    bool valid;                 // 数据是否有效
    
    AD7147_TouchData() : touch_status(0), timestamp(0), valid(false) {}
    
    bool is_channel_touched(uint8_t channel) const {
        return (channel < AD7147_MAX_CHANNELS) && ((touch_status >> channel) & 0x01);
    }
    
    uint8_t get_touched_count() const {
        uint8_t count = 0;
        for (uint8_t i = 0; i < AD7147_MAX_CHANNELS; i++) {
            if ((touch_status >> i) & 0x01) {
                count++;
            }
        }
        return count;
    }
};

// 设备信息结构体
struct AD7147_DeviceInfo {
    uint8_t i2c_address;
    bool is_valid;
    
    AD7147_DeviceInfo() : i2c_address(0), is_valid(false) {}
};

class AD7147 : public TouchSensor {
public:
    AD7147(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr);
    ~AD7147() override;
    
    // TouchSensor接口实现
    uint32_t getEnabledModuleMask() const override;
    uint32_t getCurrentTouchState() const override;
    uint32_t getSupportedChannelCount() const override;
    uint32_t getModuleIdMask() const override;
    bool init() override;
    void deinit() override;
    std::string getDeviceName() const override;
    bool isInitialized() const override;
    
    // AD7147特有接口
    bool read_device_info(AD7147_DeviceInfo& info);
    AD7147_TouchData sample_touch_data();
    bool set_channel_enable(uint8_t channel, bool enabled);
    bool set_sensitivity(uint8_t channel, uint8_t sensitivity);
    
private:
    // 硬件接口
    HAL_I2C* i2c_hal_;
    I2C_Bus i2c_bus_;
    uint8_t device_addr_;                    // AD7147设备地址
    uint8_t i2c_device_address_;             // 实际I2C设备地址
    
    // 设备状态
    bool initialized_;
    uint8_t module_id_;                      // 模块ID（0-15）
    uint32_t enabled_channels_mask_;         // 启用的通道掩码
    mutable uint32_t last_touch_state_;      // 最后一次触摸状态缓存
    
    // 内部辅助函数
    bool write_register(uint8_t reg, uint16_t value);
    bool read_register(uint8_t reg, uint16_t& value);
    bool write_registers(uint8_t reg, const uint16_t* data, size_t length);
    bool read_registers(uint8_t reg, uint16_t* data, size_t length);
};