#include "psoc.h"
#include <pico/time.h>
#include <pico/stdlib.h>
#include "../../usb_serial_logs/usb_serial_logs.h"
#include <cstring>
#include <algorithm>

uint8_t PSoC::_async_read_buffer[2];

PSoC::PSoC(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr)
    : TouchSensor(PSOC_MAX_CHANNELS), i2c_hal_(i2c_hal), i2c_bus_(i2c_bus),
      i2c_device_address_(device_addr), initialized_(false), enabled_channels_mask_(0) {
    module_name = "PSoC";
    module_mask_ = TouchSensor::generateModuleMask(static_cast<uint8_t>(i2c_bus), device_addr);
    
    // 初始化默认寄存器原始值（对应灵敏度49为零偏，4095）
    for (int i = 0; i < PSOC_MAX_CHANNELS; i++) {
        channel_thresholds_[i] = 4095;
        channel_sensitivity_ui_[i] = 49;   // UI基准
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

    sleep_ms(500);

    if (!write_reg16(PSOC_REG_CONTROL, 0x04)) {
        USB_LOG_TAG_WARNING("PSoC", "Control write failed at addr 0x%02X", i2c_device_address_);
        return false;
    }

    // 读取SCAN_RATE寄存器 启动时应不为0
    uint16_t scan_rate = 0;
    if (!read_reg16(PSOC_REG_SCAN_RATE, scan_rate)) {
        USB_LOG_TAG_WARNING("PSoC", "Detect failed at addr 0x%02X", i2c_device_address_);
        return false;
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

uint8_t PSoC::getChannelSensitivity(uint8_t channel) const {
    if (channel >= PSOC_MAX_CHANNELS) return 49;
    return channel_sensitivity_ui_[channel];
}

// 将0..99的灵敏度映射为原始寄存器值（反向步进）：raw = 4095 - (sensitivity - 49) * 10
// 其中4095为零偏基准，单步对应10“步进”，并限制在0..8191范围
// 增加灵敏度 -> 实际减少原始值；降低灵敏度 -> 实际增加原始值
bool PSoC::setChannelSensitivity(uint8_t channel, uint8_t sensitivity) {
    if (channel >= PSOC_MAX_CHANNELS || sensitivity > 99 || !initialized_) return false;

    // 相对模式（CONTROL.bit4=0）
    (void)setAbsoluteMode(false);

    int32_t raw = 4095 - (static_cast<int32_t>(sensitivity) - 49) * 10;
    raw = (raw < 0 ? 0 : (raw > 8191 ? 8191 : raw));
    uint16_t raw16 = static_cast<uint16_t>(raw);
    
    // 保存到本地存储（UI与原始编码）
    channel_sensitivity_ui_[channel] = sensitivity;
    channel_thresholds_[channel] = raw16;
    
    uint8_t reg = PSOC_REG_CAP0_THRESHOLD + channel; // 连续映射
    if (!write_reg16(reg, raw16)) {
        return false;
    }

    // 写入后读取该通道总电容（单位步进0.01pF），用于保存观感
    uint16_t steps = 0;
    if (readTotalCap(channel, steps)) {
        channel_total_cap_steps_[channel] = steps;
    }

    return true;
}

bool PSoC::setLEDEnabled(bool enabled) {
    if (!initialized_) return false;
    uint16_t ctrl = 0;
    if (!read_reg16(PSOC_REG_CONTROL, ctrl)) return false;
    if (enabled) ctrl |= 0x0002; else ctrl &= ~0x0002; // bit1
    return write_reg16(PSOC_REG_CONTROL, ctrl);
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
    uint16_t ctrl = 0;
    if (!read_reg16(PSOC_REG_CONTROL, ctrl)) return false;
    if (enabled) ctrl |= 0x0010; else ctrl &= ~0x0010; // bit4
    return write_reg16(PSOC_REG_CONTROL, ctrl);
}

bool PSoC::readTotalCap(uint8_t channel, uint16_t& steps) {
    if (channel >= PSOC_MAX_CHANNELS) return false;
    uint8_t reg = PSOC_REG_CAP0_TOTAL_CAP + channel;
    return read_reg16(reg, steps);
}

// 配置持久化实现（支持旧格式兼容）：
// 旧格式：每通道1个值（raw阈值，共12项）
// 新格式：每通道3个值（UI敏感度, raw阈值, 总电容步进，共36项）
bool PSoC::loadConfig(const std::string& config_data) {
    if (!initialized_) {
        return false;
    }
    
    // 统计逗号数以判定项数
    size_t tokens = 0;
    for (char c : config_data) { if (c == ',') tokens++; }
    tokens += (config_data.empty() ? 0 : 1);

    SaveConfig cfg;
    if (!cfg.fromString(config_data)) {
        return false;
    }

    if (tokens >= PSOC_MAX_CHANNELS * 3) {
        // 新格式：UI, raw, total
        for (int ch = 0; ch < PSOC_MAX_CHANNELS; ch++) {
            channel_sensitivity_ui_[ch] = static_cast<uint8_t>(cfg.readValue(static_cast<uint8_t>(channel_sensitivity_ui_[ch])));
            channel_thresholds_[ch]      = cfg.readValue(channel_thresholds_[ch]);
            channel_total_cap_steps_[ch] = cfg.readValue(channel_total_cap_steps_[ch]);
        }
    } else if (tokens == PSOC_MAX_CHANNELS) {
        // 旧格式：仅raw阈值
        for (int ch = 0; ch < PSOC_MAX_CHANNELS; ch++) {
            channel_thresholds_[ch] = cfg.readValue(channel_thresholds_[ch]);
            // 计算UI灵敏度（反推）：sens = 49 - (raw - 4095)/10
            int32_t delta = static_cast<int32_t>(channel_thresholds_[ch]) - 4095;
            int32_t sens = 49 - (delta / 10);
            if (sens < 0) sens = 0;
            if (sens > 99) sens = 99;
            channel_sensitivity_ui_[ch] = static_cast<uint8_t>(sens);
            channel_total_cap_steps_[ch] = 0; // 无法从旧格式恢复
        }
    } else {
        // 无法识别，保持默认
        return false;
    }

    // 加载时使用绝对模式：先写入保存的总电容（有则写，无则跳过）
    (void)setAbsoluteMode(true);
    for (int ch = 0; ch < PSOC_MAX_CHANNELS; ch++) {
        if (channel_total_cap_steps_[ch] > 0) {
            uint16_t steps = std::min<uint16_t>(channel_total_cap_steps_[ch], 2200);
            uint8_t reg = PSOC_REG_CAP0_THRESHOLD + ch;
            if (!write_reg16(reg, steps)) {
                return false;
            }
        }
    }

    // 同时加载0-99的UI灵敏度（用于后续相对调整的基准与观感）
    // 不直接写raw以免覆盖绝对模式设置；仅更新本地缓存即可。
    return true;
}

std::string PSoC::saveConfig() const {
    SaveConfig cfg;
    
    // 新格式：每通道写入 UI敏感度, raw阈值, 总电容步进
    for (int ch = 0; ch < PSOC_MAX_CHANNELS; ch++) {
        cfg.writeValue(static_cast<uint8_t>(channel_sensitivity_ui_[ch]));
        cfg.writeValue(static_cast<uint32_t>(channel_thresholds_[ch]));
        cfg.writeValue(static_cast<uint32_t>(channel_total_cap_steps_[ch]));
    }
    
    return cfg.toString();
}