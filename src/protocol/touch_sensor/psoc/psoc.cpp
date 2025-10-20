#include "psoc.h"
#include <pico/time.h>
#include <pico/stdlib.h>
#include "../../usb_serial_logs/usb_serial_logs.h"
#include <cstring>

uint8_t PSoC::_async_read_buffer[2];

PSoC::PSoC(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr)
    : TouchSensor(PSOC_MAX_CHANNELS), i2c_hal_(i2c_hal), i2c_bus_(i2c_bus),
      i2c_device_address_(device_addr), initialized_(false), enabled_channels_mask_(0) {
    module_name = "PSoC";
    module_mask_ = TouchSensor::generateModuleMask(static_cast<uint8_t>(i2c_bus), device_addr);
    
    // 初始化默认阈值（对应灵敏度50）
    for (int i = 0; i < PSOC_MAX_CHANNELS; i++) {
        channel_thresholds_[i] = 425; // 50 + 50 * (800-50) / 99 = 425
    }
}

PSoC::~PSoC() {
    deinit();
}

bool PSoC::init() {
    if (initialized_ || !i2c_hal_) return false;

    // 简单探测：读取SCAN_RATE寄存器（16位），成功即可
    uint16_t scan_rate = 0;
    if (!read_reg16(PSOC_REG_SCAN_RATE, scan_rate)) {
        USB_LOG_TAG_WARNING("PSoC", "Init failed at addr 0x%02X", i2c_device_address_);
        return false;
    }

    // 初始化时关闭LED
    (void)write_reg16(PSOC_REG_LED_CONTROL, 0x0000);
    // 读回确认写入是否生效
    uint16_t led_rb = 0xFFFF;
    if (!read_reg16(PSOC_REG_LED_CONTROL, led_rb)) {
        USB_LOG_TAG_WARNING("PSoC", "LED readback failed");
    } else {
        USB_LOG_TAG_INFO("PSoC", "LED reg=0x%04X after off", (unsigned)led_rb);
    }

    // 默认启用所有通道
    enabled_channels_mask_ = (PSOC_MAX_CHANNELS >= 32) ? 0xFFFFFFFFu : ((1u << PSOC_MAX_CHANNELS) - 1u);
    initialized_ = true;
    USB_LOG_TAG_INFO("PSoC", "Init ok, scan_rate=%u (LED off)", (unsigned)scan_rate);
    return true;
}

void PSoC::deinit() {
    initialized_ = false;
}

bool PSoC::isInitialized() const { return initialized_; }

uint32_t PSoC::getSupportedChannelCount() const {
    return static_cast<uint32_t>(max_channels_);
}

void PSoC::sample(async_touchsampleresult callback) {
    if (!callback) return;
    if (!initialized_) {
        TouchSampleResult result{};
        result.timestamp_us = time_us_32();
        callback(result);
        return;
    }

    // 标准异步实现：发起寄存器异步读取（写子地址+读2字节），回调解析
    i2c_hal_->read_register_async(
        i2c_device_address_,
        PSOC_REG_TOUCH_STATUS,
        _async_read_buffer,
        2,
        [this, callback](bool success) {
            TouchSampleResult result{0, 0};
            result.timestamp_us = time_us_32();
            if (success) {
                uint16_t v = (static_cast<uint16_t>(_async_read_buffer[0]) << 8) |
                              static_cast<uint16_t>(_async_read_buffer[1]);
                uint32_t mask12 = static_cast<uint32_t>(v & 0x0FFFu);
                result.channel_mask = mask12 & enabled_channels_mask_;
                result.module_mask = module_mask_;
            }
            callback(result);
        }
    );
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

// 简单将0..99映射到推荐阈值区间（约50..800 counts）并写入对应通道阈值寄存器
bool PSoC::setChannelSensitivity(uint8_t channel, uint8_t sensitivity) {
    if (channel >= PSOC_MAX_CHANNELS || sensitivity > 99 || !initialized_) return false;

    // 线性映射：阈值 = 50 + sensitivity * (800-50) / 99
    uint16_t threshold = static_cast<uint16_t>(50 + (static_cast<uint32_t>(sensitivity) * (800 - 50)) / 99);
    
    // 保存到本地存储
    channel_thresholds_[channel] = threshold;
    
    uint8_t reg = PSOC_REG_CAP0_THRESHOLD + channel; // 连续映射
    return write_reg16(reg, threshold);
}

bool PSoC::setLEDEnabled(bool enabled) {
    if (!initialized_) return false;
    uint16_t val = enabled ? 0x0001 : 0x0000;
    return write_reg16(PSOC_REG_LED_CONTROL, val);
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

// 配置持久化实现
bool PSoC::loadConfig(const std::string& config_data) {
    if (!initialized_) {
        return false;
    }
    
    SaveConfig config_manager;
    if (!config_manager.fromString(config_data)) {
        return false;
    }
    
    // 按固定顺序从配置中加载通道阈值
    for (int channel = 0; channel < PSOC_MAX_CHANNELS; channel++) {
        channel_thresholds_[channel] = config_manager.readValue(channel_thresholds_[channel]);
        
        // 应用到硬件寄存器
        uint8_t reg = PSOC_REG_CAP0_THRESHOLD + channel;
        if (!write_reg16(reg, channel_thresholds_[channel])) {
            return false; // 如果任何通道写入失败，返回失败
        }
    }
    
    return true;
}

std::string PSoC::saveConfig() const {
    SaveConfig config_manager;
    
    // 按固定顺序保存通道阈值到配置
    for (int channel = 0; channel < PSOC_MAX_CHANNELS; channel++) {
        config_manager.writeValue(static_cast<uint32_t>(channel_thresholds_[channel]));
    }
    
    return config_manager.toString();
}