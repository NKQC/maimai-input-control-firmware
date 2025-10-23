#include "psoc.h"
#include <pico/time.h>
#include <pico/stdlib.h>
#include "../../usb_serial_logs/usb_serial_logs.h"
#include "../../../service/input_manager/input_manager.h"
#include <cstring>
#include <algorithm>

uint8_t PSoC::_async_read_buffer[2];

PSoC::PSoC(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr)
    : TouchSensor(PSOC_MAX_CHANNELS), i2c_hal_(i2c_hal), i2c_bus_(i2c_bus),
      i2c_device_address_(device_addr), initialized_(false), enabled_channels_mask_(0), control_reg_(0) {
    module_name = "PSoC";
    module_mask_ = TouchSensor::generateModuleMask(static_cast<uint8_t>(i2c_bus), device_addr);
    
    // 设置PSoC的功能标志：支持一般灵敏度设置(位0) + 相对设置模式(位1)
    sensor_flag_.supports_general_sensitivity = true;
    sensor_flag_.sensitivity_relative_mode = true;
    sensor_flag_.sensitivity_private_mode = false;
    sensor_flag_.supports_calibration = false;
    
    // 初始化默认总电容步进值
    for (int i = 0; i < PSOC_MAX_CHANNELS; i++) {
        channel_total_cap_steps_[i] = 0;   // 未知，加载/写入后读取
    }
}

PSoC::~PSoC() {
    deinit();
}

bool PSoC::init() {
    if (initialized_ || !i2c_hal_) return false;

    if (!write_reg16(PSOC_REG_CONTROL, 0x01)) {
        USB_LOG_TAG_WARNING("PSoC", "Control reset failed at addr 0x%02X", i2c_device_address_);
        return false;
    }
    control_reg_ = 0x01;  // 同步缓存状态

    sleep_ms(500);

    // 读取SCAN_RATE寄存器 启动时应不为0
    uint16_t scan_rate = 0;
    if (!read_reg16(PSOC_REG_SCAN_RATE, scan_rate)) {
        USB_LOG_TAG_WARNING("PSoC", "Detect failed at addr 0x%02X", i2c_device_address_);
        return false;
    }

    if (!write_reg16(PSOC_REG_CONTROL, 0x20)) {
        USB_LOG_TAG_WARNING("PSoC", "Control settings failed at addr 0x%02X", i2c_device_address_);
        return false;
    }
    control_reg_ = 0x20;  // 同步缓存状态

    // 默认启用所有通道
    enabled_channels_mask_ = (PSOC_MAX_CHANNELS >= 32) ? 0xFFFFFFFFu : ((1u << PSOC_MAX_CHANNELS) - 1u);
    initialized_ = true;
    
    USB_LOG_TAG_INFO("PSoC", "Init ok, scan_rate=%u (LED off)", (unsigned)scan_rate);
    return true;
}

void PSoC::deinit() {
    initialized_ = false;
    control_reg_ = 0;  // 重置缓存状态
}

bool PSoC::isInitialized() const { return initialized_; }

uint32_t PSoC::getSupportedChannelCount() const {
    return static_cast<uint32_t>(max_channels_);
}

#ifndef PSOC_SAMPLE_USE_SYNC
#define PSOC_SAMPLE_USE_SYNC 1  // 仅对本文件生效：1=同步采样，0=异步采样
#endif
void PSoC::sample(async_touchsampleresult callback) {
    if (!callback) return;
#if PSOC_SAMPLE_USE_SYNC
    // 同步采样路径：用于验证稳定性
    if (!initialized_) {
        TouchSampleResult result{};
        result.timestamp_us = 0; // 未初始化视为失败
        callback(result);
        return;
    }
    uint16_t touch_status = 0;
    if (!read_reg16(PSOC_REG_TOUCH_STATUS, touch_status)) {
        TouchSampleResult result{};
        result.timestamp_us = 0; // 读取失败视为失败
        callback(result);
        return;
    }
    TouchSampleResult result{0, 0};
    uint32_t mask12 = static_cast<uint32_t>(touch_status & 0x0FFFu);
    result.channel_mask = mask12 & enabled_channels_mask_;
    result.module_mask = module_mask_;
    result.timestamp_us = time_us_32();
    // USB_LOG_DEBUG("Sync read, data={0x%02X, 0x%02X}", (touch_status >> 8) & 0xFF, touch_status & 0xFF);
    callback(result);
#else
    // 异步采样路径：DMA+中断完成后回调（HAL已在STOP中断等待RX DMA完成）
    if (!initialized_) {
        TouchSampleResult result{};
        result.timestamp_us = 0; // 未初始化视为失败
        callback(result);
        return;
    }
    i2c_hal_->read_register_async(
        i2c_device_address_,
        PSOC_REG_TOUCH_STATUS,
        _async_read_buffer,
        2,
        [this, callback](bool success) {
            TouchSampleResult result{0, 0};
            if (success) {
                uint16_t v = (static_cast<uint16_t>(_async_read_buffer[0]) << 8) |
                              static_cast<uint16_t>(_async_read_buffer[1]);
                uint32_t mask12 = static_cast<uint32_t>(v & 0x0FFFu);
                result.channel_mask = mask12 & enabled_channels_mask_;
                result.module_mask = module_mask_;
                result.timestamp_us = time_us_32();
                USB_LOG_DEBUG("Async read, data={0x%02X, 0x%02X}", (v >> 8) & 0xFF, v & 0xFF);
            } else {
                result.timestamp_us = 0; // 失败采样，InputManager将忽略
            }
            callback(result);
        }
    );
#endif
}

bool PSoC::setChannelEnabled(uint8_t channel, bool enabled) {
    if (channel >= PSOC_MAX_CHANNELS) return false;
    if (enabled) enabled_channels_mask_ |= (1u << channel);
    else enabled_channels_mask_ &= ~(1u << channel);
    return true;
}

bool PSoC::getChannelEnabled(uint8_t channel) const {
    if (channel >= PSOC_MAX_CHANNELS) return false;
    return (enabled_channels_mask_ & (1u << channel)) != 0;
}

uint32_t PSoC::getEnabledChannelMask() const {
    return enabled_channels_mask_;
}

uint8_t PSoC::getChannelSensitivity(uint8_t channel) const {
    if (channel >= PSOC_MAX_CHANNELS) return 0;
    // 相对灵敏度传感器始终返回0，不存在绝对灵敏度概念
    return 0;
}

// 相对灵敏度调整：临时调整阈值，不存储相对值
// 0为基线，正值增加灵敏度（减少阈值），负值降低灵敏度（增加阈值）
// 每个单位对应10步进，限制在0..8191范围
bool PSoC::setChannelSensitivity(uint8_t channel, int8_t sensitivity) {
    if (channel >= PSOC_MAX_CHANNELS || !initialized_) return false;

    // 相对模式（CONTROL.bit4=0）
    (void)setAbsoluteMode(false);

    // 以4095为基线，直接使用sensitivity进行相对调整
    int32_t raw = 4095 - static_cast<int32_t>(sensitivity) * 20; // 单灵敏度变化0.2pf
    raw = (raw < 0 ? 0 : (raw > 8191 ? 8191 : raw));
    uint16_t raw16 = static_cast<uint16_t>(raw);
    
    uint8_t reg = PSOC_REG_CAP0_THRESHOLD + channel; // 连续映射
    if (!write_reg16(reg, raw16)) {
        return false;
    }

    // 写入后读取该通道总电容（单位步进0.01pF），用于保存
    uint16_t steps = 0;
    if (readTotalCap(channel, steps)) {
        channel_total_cap_steps_[channel] = steps;
    }
    USB_LOG_DEBUG("[PSoC]CurrentSensitivity: [%d], ch=%d, raw=%d, Cap=%d", module_mask_, channel, raw, steps);
    return true;
}

bool PSoC::setLEDEnabled(bool enabled) {
    if (!initialized_) return false;
    uint16_t ctrl = control_reg_;  // 使用缓存的值
    if (enabled) ctrl |= 0x0002; else ctrl &= ~0x0002; // bit1
    if (!write_reg16(PSOC_REG_CONTROL, ctrl)) return false;
    control_reg_ = ctrl;  // 更新缓存
    return true;
}

bool PSoC::read_reg16(uint8_t reg, uint16_t& value) {
    uint16_t tmp = 0;
    int32_t r = i2c_hal_->read_register(i2c_device_address_, reg, (uint8_t*)&tmp, 2);
    if (r != 2) return false;
    __asm__ volatile (
            "rev16 %0, %0\n"
            : "+r" (tmp)
            :: "cc"
    );
    value = tmp;
    return true;
}

bool PSoC::write_reg16(uint8_t reg, uint16_t value) {
    uint16_t tmp = value;
    __asm__ volatile (
            "rev16 %0, %0\n"
            : "+r" (tmp)
            :: "cc"
    );
    int32_t w = i2c_hal_->write_register(i2c_device_address_, reg, (uint8_t*)&tmp, 2);
    return w == 2;
}

bool PSoC::setAbsoluteMode(bool enabled) {
    uint16_t ctrl = control_reg_;  // 使用缓存的值
    if (enabled) ctrl |= 0x0010; else ctrl &= ~0x0010; // bit4
    if (!write_reg16(PSOC_REG_CONTROL, ctrl)) return false;
    control_reg_ = ctrl;  // 更新缓存
    return true;
}

bool PSoC::readTotalCap(uint8_t channel, uint16_t& steps) {
    if (channel >= PSOC_MAX_CHANNELS) return false;
    uint8_t reg = PSOC_REG_CAP0_TOTAL_CAP + channel;
    return read_reg16(reg, steps);
}

bool PSoC::loadConfig(const std::string& config_data) {
    if (!initialized_) {
        return false;
    }
    
    SaveConfig cfg;
    if (!cfg.fromString(config_data)) {
        return false;
    }

    // 加载总电容步进值
    for (int ch = 0; ch < PSOC_MAX_CHANNELS; ch++) {
        channel_total_cap_steps_[ch] = cfg.readValue(channel_total_cap_steps_[ch]);
    }

    // 加载时使用绝对模式：写入保存的总电容
    (void)setAbsoluteMode(true);
    for (int ch = 0; ch < PSOC_MAX_CHANNELS; ch++) {
        if (channel_total_cap_steps_[ch] > 0) {
            uint16_t steps = std::min<uint16_t>(channel_total_cap_steps_[ch], 2200);
            uint8_t threshold_reg = PSOC_REG_CAP0_THRESHOLD + ch;
            (void)write_reg16(threshold_reg, steps);
        }
    }

    return true;
}

std::string PSoC::saveConfig() const {
    SaveConfig cfg;
    
    // 保存总电容步进值
    for (int ch = 0; ch < PSOC_MAX_CHANNELS; ch++) {
        cfg.writeValue(static_cast<uint32_t>(channel_total_cap_steps_[ch]));
    }
    
    return cfg.toString();
}