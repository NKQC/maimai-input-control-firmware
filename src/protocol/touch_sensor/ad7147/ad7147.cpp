#include "ad7147.h"
#include <cstring>
#include <pico/time.h>
#include <pico/stdlib.h>
#include "../../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "src/protocol/usb_serial_logs/usb_serial_logs.h"

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
    uint8_t ret = 1;
    uint16_t _device_id = 0;
    ret &= read_register(AD7147_REG_DEVICE_ID, _device_id);
    USB_LOG_DEBUG("AD7147 device ID: %0x", _device_id);
    ret &= _device_id != 0;
    // 默认启用全部通道，与芯片默认一致，并将各阶段的中断/校准开关同步到寄存器，保证设置可实时生效
    enabled_channels_mask_ = 0x1FFFu; // 13位
    
    // 可选：配置所有阶段（如果需要自定义配置）
    uint16_t power_control_val = 0xC3B0;
    ret &= configureStages(power_control_val, nullptr);

    initialized_ = ret;
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
    uint8_t ad7147_sensitivity[2] = {0, 0};
    ad7147_sensitivity[0] = (threshold_sensitivity & 0xF);        // [3:0] 负阈值灵敏度
    ad7147_sensitivity[0] |= ((peak_detect & 0x7) << 4);          // [6:4] 负峰值检测
    ad7147_sensitivity[1] = ((threshold_sensitivity & 0xF) << 8); // [11:8] 正阈值灵敏度
    ad7147_sensitivity[1] |= ((peak_detect & 0x7) << 12);         // [14:12] 正峰值检测
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
    static TouchSampleResult result = {};
    static uint16_t status_regs[2] = {0};
    if (!initialized_) {
        result.timestamp_us = time_us_32();
        return result;
    }

    read_register(AD7147_REG_STAGE_LOW_INT_STATUS, status_regs[0]);
    read_register(AD7147_REG_STAGE_HIGH_INT_STATUS, status_regs[1]);
    
    // 合并触摸状态并限制在24位范围内
    result.channel_mask = static_cast<uint32_t>(status_regs[0] | status_regs[1]);
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
bool AD7147::write_register(uint16_t reg, uint8_t* value, uint16_t size) {
    int cb = 0;
    cb = i2c_hal_->write_register(device_addr_, reg | 0x8000, value, size);
    USB_LOG_DEBUG("AD7147 write_register reg:0x%0x, size:%d, cb:%d", reg, size, cb);
    return cb == size;
}

bool AD7147::read_register(uint16_t reg, uint16_t& value) {
    bool success = i2c_hal_->read_register(device_addr_, reg | 0x8000, (uint8_t*)&value, 2) == 2;
    __asm__ volatile (
            "rev16 %0, %0\n"
            : "+r" (value)
            :: "cc"
    );
    return success;
}

// 将启用的通道掩码实时应用到硬件，按位启用/关闭各个阶段（Stage）
bool AD7147::applyEnabledChannelsToHardware() {
    uint8_t mask13[2] = {(uint8_t)(enabled_channels_mask_ >> 8), (uint8_t)(enabled_channels_mask_ & 0x1FFFu)};

    // 关闭未启用阶段的校准和比较中断，减少扫描负担，提高有效采样速率
    bool ok = true;
    ok &= write_register(AD7147_REG_STAGE_CAL_EN, mask13);
    ok &= write_register(AD7147_REG_STAGE_LOW_INT_EN, mask13);
    ok &= write_register(AD7147_REG_STAGE_HIGH_INT_EN, mask13);

    return ok;
}

// 配置所有阶段，参考sample.c中的ConfigAD7147函数
bool AD7147::configureStages(uint16_t power_control_val, const uint16_t* connection_values) {
    bool ret = true;
    uint8_t config_buffer[16];
    
    // 阶段连接配置值数组，对应CIN0-CIN11的单端配置
    const uint16_t stage_connection_configs[12][2] = {
        {0x0003, 0x0000}, // Stage 0 - CIN0
        {0x000C, 0x0000}, // Stage 1 - CIN1  
        {0x0030, 0x0000}, // Stage 2 - CIN2
        {0x00C0, 0x0000}, // Stage 3 - CIN3
        {0x0300, 0x0000}, // Stage 4 - CIN4
        {0x0C00, 0x0000}, // Stage 5 - CIN5
        {0x3000, 0x0000}, // Stage 6 - CIN6
        {0x0000, 0x0003}, // Stage 7 - CIN7
        {0x0000, 0x000C}, // Stage 8 - CIN8
        {0x0000, 0x0030}, // Stage 9 - CIN9
        {0x0000, 0x00C0}, // Stage 10 - CIN10
        {0x0000, 0x0300}  // Stage 11 - CIN11
    };
    USB_LOG_DEBUG("AD7147 ConfigureStages Stage");
    // 配置每个阶段（Stage 0-11）
    for (uint8_t stage = 0; stage < 12; stage++) {
        config_buffer[0] = (uint8_t)(stage_connection_configs[stage][0] >> 8);
        config_buffer[1] = (uint8_t)(stage_connection_configs[stage][0] & 0xFF);

        config_buffer[2] = (uint8_t)(stage_connection_configs[stage][1] >> 8);
        config_buffer[3] = (uint8_t)(stage_connection_configs[stage][1] & 0xFF);

        config_buffer[4] = (uint8_t)(connection_values ? connection_values[0] : AD7147_DEFAULT_AFE_OFFSET) >> 8;
        config_buffer[5] = (uint8_t)(connection_values ? connection_values[0] : AD7147_DEFAULT_AFE_OFFSET) & 0xFF;

        config_buffer[6] = AD7147_SENSITIVITY_DEFAULT >> 8;
        config_buffer[7] = (uint8_t)(AD7147_SENSITIVITY_DEFAULT & 0xFF);

        config_buffer[8] = (uint8_t)AD7147_DEFAULT_OFFSET_LOW >> 8;
        config_buffer[9] = (uint8_t)AD7147_DEFAULT_OFFSET_LOW & 0xFF;

        config_buffer[10] = (uint8_t)(AD7147_DEFAULT_OFFSET_HIGH >> 8);
        config_buffer[11] = (uint8_t)AD7147_DEFAULT_OFFSET_HIGH & 0xFF;
        
        config_buffer[12] = (uint8_t)AD7147_DEFAULT_OFFSET_HIGH_CLAMP >> 8;
        config_buffer[13] = (uint8_t)AD7147_DEFAULT_OFFSET_HIGH_CLAMP & 0xFF;

        config_buffer[14] = (uint8_t)AD7147_DEFAULT_OFFSET_LOW_CLAMP >> 8;
        config_buffer[15] = (uint8_t)AD7147_DEFAULT_OFFSET_LOW_CLAMP & 0xFF;
        
        for (uint8_t i = 0; i < 8; i++) {

            ret &= write_register(AD7147_REG_STAGE0_CONNECTION + (stage * AD7147_REG_STAGE_SIZE), &config_buffer[i * 2], 2);
        }
    }
    USB_LOG_DEBUG("AD7147 ConfigureStages Register");
    // 配置第一寄存器组
    uint8_t reg_config[16];
    reg_config[0] = (uint8_t)(power_control_val >> 8);
    reg_config[1] = (uint8_t)(power_control_val & 0xFF);  // PWR_CONTROL 0x0
    reg_config[2] = 0x00;
    reg_config[3] = 0x00;            // STAGE_CAL_EN  此阶段必须为0 为保持序号与地址一致 保留 0x1
    reg_config[4] = 0x00;
    reg_config[5] = 0x00;            // AMB_COMP_CTRL0 0x2
    reg_config[6] = 0x01;
    reg_config[7] = 0x40;            // AMB_COMP_CTRL1 0x3
    reg_config[8] = 0x08;
    reg_config[9] = 0x32;            // AMB_COMP_CTRL2 0x4
    reg_config[10] = 0x3F;
    reg_config[11] = 0xFF;            // STAGE_LOW_INT_ENABLE 0x5
    reg_config[12] = 0x0F;
    reg_config[13] = 0xFF;            // STAGE_HIGH_INT_ENABLE 0x6
    reg_config[14] = 0x00;
    reg_config[15] = 0x00;            // STAGE_COMPLETE_INT_ENABLE 0x7

    // 写入配置寄存器组
    bool cb = true;
    for (int i = 0; i < 8; i++) {
        cb &= write_register(AD7147_REG_PWR_CONTROL + i, &reg_config[i * 2], 2);
        if (i == 1) cb = true;  // 允许ADDR 1失败
        ret &= cb;
    }
    reg_config[0] = 0x0F;
    reg_config[1] = 0xFF;
    USB_LOG_DEBUG("AD7147 ConfigureStages EN");
    // 启用所有阶段校准
    ret &= write_register(AD7147_REG_STAGE_CAL_EN, reg_config, 2);
    
    return ret;
}
