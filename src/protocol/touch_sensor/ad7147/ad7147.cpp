#include "ad7147.h"
#include <cstring>
#include <pico/time.h>
#include <pico/stdlib.h>
#include "../../../protocol/usb_serial_logs/usb_serial_logs.h"

// AD7147构造函数
AD7147::AD7147(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr)
    : TouchSensor(AD7147_MAX_CHANNELS), i2c_hal_(i2c_hal), i2c_bus_(i2c_bus),
      device_addr_(device_addr), i2c_device_address_(device_addr),
      initialized_(false), i2c_bus_enum_(i2c_bus), enabled_channels_mask_(0), last_touch_state_(0) {
    module_name = "AD7147";
    module_mask_ = TouchSensor::generateModuleMask(static_cast<uint8_t>(i2c_bus), device_addr);
    supported_channel_count_ = AD7147_MAX_CHANNELS;
}

// AD7147析构函数
AD7147::~AD7147() {
    deinit();
}

// 初始化AD7147
bool AD7147::init() {
    if (initialized_) {
        return true;
    }

    if (!i2c_hal_) {
        return false;
    }

    // 基本的初始化检查
    uint16_t test_value = 0;
    if (!read_register(AD7147_REG_PWR_CONTROL, test_value)) {
        return false;
    }
    uint8_t ret = 0;
    // 设置基本配置 
    ret |= !write_register(AD7147_REG_PWR_CONTROL, 0x03B0);

    // 启用所有通道的校准（芯片上电后的初始值），随后根据enabled_channels_mask_实时下发
    ret |= write_register(AD7147_REG_STAGE_CAL_EN, 0x1FFF); // 13位通道

    ret |= write_register(AD7147_REG_AMB_COMP_CTRL0, 0x0000);

    // 默认启用全部通道，与芯片默认一致，并将各阶段的中断/校准开关同步到寄存器，保证设置可实时生效
    enabled_channels_mask_ = 0x1FFFu; // 13位
    applyEnabledChannelsToHardware();

    initialized_ = !ret;
    return ret;
}

// 反初始化AD7147
void AD7147::deinit() {
    initialized_ = false;
}

// 读取设备信息
bool AD7147::read_device_info(AD7147_DeviceInfo& info) {
    info.i2c_address = device_addr_;
    info.is_valid = initialized_;
    return initialized_;
}

// 设置通道灵敏度（统一接口，0-99）
bool AD7147::setChannelSensitivity(uint8_t channel, uint8_t sensitivity) {
    if (!initialized_ || channel >= AD7147_MAX_CHANNELS || sensitivity > 99) {
        return false;
    }
    
    // 计算对应stage的灵敏度寄存器地址
    uint16_t stage_base_addr = AD7147_REG_STAGE0_CONNECTION + (channel * AD7147_REG_STAGE_SIZE);
    uint16_t sensitivity_reg_addr = stage_base_addr + AD7147_STAGE_SENSITIVITY_OFFSET;
    
    // 根据AD7147寄存器规格映射灵敏度值
    // 寄存器格式：[15:12]正峰值检测，[11:8]正阈值灵敏度，[6:4]负峰值检测，[3:0]负阈值灵敏度
    // 正负半部分灵敏度相同，峰值检测和灵敏度阈值组合作为整体灵敏度转换
    
    uint8_t threshold_sensitivity;
    uint8_t peak_detect;
    
    // 反转灵敏度映射：99为最高灵敏度（最低阈值），0为最低灵敏度（最高阈值）
    uint8_t inverted_sensitivity = 99 - sensitivity;
    
    if (inverted_sensitivity == 0) {
        // 最高灵敏度（sensitivity=99）：最低阈值25%，40%峰值检测
        threshold_sensitivity = 0x0;  // 25%
        peak_detect = 0x0;           // 40% level
    } else if (inverted_sensitivity <= 20) {
        // 高灵敏度范围：25%-43.79%阈值，40%-50%峰值检测
        threshold_sensitivity = (inverted_sensitivity * 4) / 20;  // 0-4 映射到 25%-43.79%
        peak_detect = (inverted_sensitivity > 10) ? 0x1 : 0x0;    // >10时使用50%，否则40%
    } else if (inverted_sensitivity <= 50) {
        // 中等灵敏度范围：43.79%-67.22%阈值，50%-70%峰值检测
        threshold_sensitivity = 0x4 + ((inverted_sensitivity - 20) * 5) / 30;  // 4-9
        peak_detect = 0x1 + ((inverted_sensitivity - 20) * 2) / 30;            // 1-3
    } else if (inverted_sensitivity <= 80) {
        // 低灵敏度范围：67.22%-90.64%阈值，70%-90%峰值检测
        threshold_sensitivity = 0x9 + ((inverted_sensitivity - 50) * 5) / 30;  // 9-14
        peak_detect = 0x3 + ((inverted_sensitivity - 50) * 2) / 30;            // 3-5
    } else {
        // 最低灵敏度（sensitivity=0）：最高阈值95.32%，90%峰值检测
        threshold_sensitivity = 0xF;  // 95.32%
        peak_detect = 0x5;           // 90% level
    }
    
    // 构建寄存器值：正负半部分使用相同的灵敏度和峰值检测设置
    uint16_t ad7147_sensitivity = 0;
    ad7147_sensitivity |= (threshold_sensitivity & 0xF);        // [3:0] 负阈值灵敏度
    ad7147_sensitivity |= ((peak_detect & 0x7) << 4);          // [6:4] 负峰值检测
    ad7147_sensitivity |= ((threshold_sensitivity & 0xF) << 8); // [11:8] 正阈值灵敏度
    ad7147_sensitivity |= ((peak_detect & 0x7) << 12);         // [14:12] 正峰值检测
    // [7] 和 [15] 保持为0（未使用位）
    
    // 写入灵敏度寄存器
    return write_register(sensitivity_reg_addr, ad7147_sensitivity);
}

// TouchSensor接口实现
uint32_t AD7147::getSupportedChannelCount() const {
    return static_cast<uint32_t>(max_channels_);
}

bool AD7147::isInitialized() const {
    return initialized_;
}

TouchSampleResult AD7147::sample() {
    TouchSampleResult result = {};
    
    if (!initialized_) {
        result.timestamp_us = time_us_32();
        return result;
    }

    // 读取触摸状态
    uint16_t high_status = 0;
    uint16_t low_status = 0;
    read_register(AD7147_REG_STAGE_HIGH_INT_STATUS, high_status);
    read_register(AD7147_REG_STAGE_LOW_INT_STATUS, low_status);
    
    // 合并触摸状态并限制在24位范围内
    result.channel_mask = static_cast<uint32_t>(high_status | low_status) & 0xFFFFFF;

    // 仅报告当前启用的通道，保证禁用通道不产生事件
    result.channel_mask &= enabled_channels_mask_;

    result.module_mask = module_mask_;
    result.timestamp_us = time_us_32();
    
    return result;
}

bool AD7147::setChannelEnabled(uint8_t channel, bool enabled) {
    if (!initialized_ || channel >= AD7147_MAX_CHANNELS) {
        return false;
    }
    
    if (enabled) {
        enabled_channels_mask_ |= (1UL << channel);
    } else {
        enabled_channels_mask_ &= ~(1UL << channel);
    }

    // 将开关通道设置实时下发到芯片，关闭未用阶段以提升扫描/采样速率
    return applyEnabledChannelsToHardware();
}

bool AD7147::getChannelEnabled(uint8_t channel) const {
    if (channel >= AD7147_MAX_CHANNELS) {
        return false;
    }
    return (enabled_channels_mask_ & (1UL << channel)) != 0;
}

uint32_t AD7147::getEnabledChannelMask() const {
    return enabled_channels_mask_;
}

// 内部辅助函数
bool AD7147::write_register(uint16_t reg, uint16_t value) {
    uint8_t data[4] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
                       static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    return i2c_hal_->write(device_addr_, data, 4);
}

bool AD7147::read_register(uint16_t reg, uint16_t& value) {
    uint8_t reg_data[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};
    uint8_t read_data[2] = {0};
    
    if (!i2c_hal_->write(device_addr_, reg_data, 2)) {
        return false;
    }
    
    if (!i2c_hal_->read(device_addr_, read_data, 2)) {
        return false;
    }
    
    value = (static_cast<uint16_t>(read_data[0]) << 8) | read_data[1];
    return true;
}

bool AD7147::write_registers(uint16_t start_reg, const uint16_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (!write_register(start_reg + i, data[i])) {
            return false;
        }
    }
    return true;
}

bool AD7147::read_registers(uint16_t start_reg, uint16_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (!read_register(start_reg + i, data[i])) {
            return false;
        }
    }
    return true;
}

// 将启用的通道掩码实时应用到硬件，按位启用/关闭各个阶段（Stage）
bool AD7147::applyEnabledChannelsToHardware() {
    if (!initialized_) return false;

    uint16_t mask13 = static_cast<uint16_t>(enabled_channels_mask_ & 0x1FFFu);

    // 关闭未启用阶段的校准和比较中断，减少扫描负担，提高有效采样速率
    bool ok = true;
    ok = ok && write_register(AD7147_REG_STAGE_CAL_EN, mask13);

    // 低中断使能位为[11:0]（避免覆盖[13:12]）
    uint16_t new_low_en = static_cast<uint16_t>((0x3u << 12) | (mask13 & 0x0FFFu));
    ok = ok && write_register(AD7147_REG_STAGE_LOW_INT_EN, new_low_en);

    ok = ok && write_register(AD7147_REG_STAGE_HIGH_INT_EN, mask13);
    ok = ok && write_register(AD7147_REG_STAGE_COMPLETE_INT_EN, mask13);

    return ok;
}