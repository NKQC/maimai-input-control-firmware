#pragma once

#include <stdint.h>
#include <string>
#include <functional>
#include "../../../hal/i2c/hal_i2c.h"
#include "../touch_sensor.h"

/**
 * 协议层 - PSoC TouchSensor I2C从机
 * 参考 I2C_Registers_README.md 中的寄存器映射
 * 地址范围：0x08 - 0x0B（由硬件引脚组合决定）
 */

// 寄存器地址（1字节）
#define PSOC_REG_SCAN_RATE       0x00  // R: 16-bit，当前每秒扫描次数
#define PSOC_REG_TOUCH_STATUS    0x01  // R: 16-bit，bit[0..11] 对应 CAP0..CAPB
#define PSOC_REG_CONTROL         0x02  // R/W: 16-bit，bit0=复位, bit1=LED, bit4=绝对模式
#define PSOC_REG_CAP0_THRESHOLD  0x03  // R/W: 16-bit 阈值/触摸电容设置寄存器（相对/绝对模式）
#define PSOC_REG_CAP1_THRESHOLD  0x04
#define PSOC_REG_CAP2_THRESHOLD  0x05
#define PSOC_REG_CAP3_THRESHOLD  0x06
#define PSOC_REG_CAP4_THRESHOLD  0x07
#define PSOC_REG_CAP5_THRESHOLD  0x08
#define PSOC_REG_CAP6_THRESHOLD  0x09
#define PSOC_REG_CAP7_THRESHOLD  0x0A
#define PSOC_REG_CAP8_THRESHOLD  0x0B
#define PSOC_REG_CAP9_THRESHOLD  0x0C
#define PSOC_REG_CAPA_THRESHOLD  0x0D
#define PSOC_REG_CAPB_THRESHOLD  0x0E

// 总触摸电容只读寄存器（单位：步进 0.01 pF）
#define PSOC_REG_CAP0_TOTAL_CAP  0x0F  // R: 16-bit
#define PSOC_REG_CAP1_TOTAL_CAP  0x10
#define PSOC_REG_CAP2_TOTAL_CAP  0x11
#define PSOC_REG_CAP3_TOTAL_CAP  0x12
#define PSOC_REG_CAP4_TOTAL_CAP  0x13
#define PSOC_REG_CAP5_TOTAL_CAP  0x14
#define PSOC_REG_CAP6_TOTAL_CAP  0x15
#define PSOC_REG_CAP7_TOTAL_CAP  0x16
#define PSOC_REG_CAP8_TOTAL_CAP  0x17
#define PSOC_REG_CAP9_TOTAL_CAP  0x18
#define PSOC_REG_CAPA_TOTAL_CAP  0x19
#define PSOC_REG_CAPB_TOTAL_CAP  0x1A

#define PSOC_MAX_CHANNELS        12

class PSoC : public TouchSensor {
public:
    PSoC(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr);
    ~PSoC() override;

    // TouchSensor接口实现
    void sample(async_touchsampleresult callback) override;  // 异步采样接口
    uint32_t getSupportedChannelCount() const override;
    bool init() override;
    void deinit() override;
    bool isInitialized() const override;

    bool setChannelEnabled(uint8_t channel, bool enabled) override;    // 仅维护本地启用掩码
    bool getChannelEnabled(uint8_t channel) const override;
    uint32_t getEnabledChannelMask() const override;

    bool setChannelSensitivity(uint8_t channel, int8_t sensitivity) override; // -127..127 映射为阈值写入（相对模式）
    uint8_t getChannelSensitivity(uint8_t channel) const override;              // 返回UI侧0..99
    bool setLEDEnabled(bool enabled) override;  // 写 CONTROL bit1

    // 配置持久化接口实现
    bool loadConfig(const std::string& config_data) override;
    std::string saveConfig() const override;

private:
    // 简化的寄存器访问封装（1字节寄存器地址，读/写16位值）
    bool read_reg16(uint8_t reg, uint16_t& value);
    bool write_reg16(uint8_t reg, uint16_t value);

    // 控制位辅助
    bool setAbsoluteMode(bool enabled);
    bool readTotalCap(uint8_t channel, uint16_t& steps);

    HAL_I2C* i2c_hal_;
    I2C_Bus i2c_bus_;
    uint8_t i2c_device_address_;

    bool initialized_;
    uint32_t enabled_channels_mask_;
    uint16_t control_reg_;  // 缓存PSOC_REG_CONTROL寄存器的值
    
    // 总电容步进存储（绝对模式下每通道的总电容设置）
    uint16_t channel_total_cap_steps_[PSOC_MAX_CHANNELS];

    // 异步读取缓冲
    static uint8_t _async_read_buffer[2];
};