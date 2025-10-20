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
#define PSOC_REG_LED_CONTROL     0x02  // R/W: 16-bit，bit0 控制LED
#define PSOC_REG_CAP0_THRESHOLD  0x03  // R/W: 16-bit 阈值寄存器
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

    bool setChannelSensitivity(uint8_t channel, uint8_t sensitivity) override; // 0..99 映射为阈值写入
    bool setLEDEnabled(bool enabled) override;  // 写 LED_CONTROL bit0

    // 配置持久化接口实现
    bool loadConfig(const std::string& config_data) override;
    std::string saveConfig() const override;

private:
    // 简化的寄存器访问封装（1字节寄存器地址，读/写16位值）
    bool read_reg16(uint8_t reg, uint16_t& value);
    bool write_reg16(uint8_t reg, uint16_t value);

    HAL_I2C* i2c_hal_;
    I2C_Bus i2c_bus_;
    uint8_t i2c_device_address_;

    bool initialized_;
    uint32_t enabled_channels_mask_;
    
    // 灵敏度设置存储（每通道阈值）
    uint16_t channel_thresholds_[PSOC_MAX_CHANNELS];

    // 异步读取缓冲
    static uint8_t _async_read_buffer[2];
};