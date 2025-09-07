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
      initialized_(false), i2c_bus_enum_(i2c_bus), enabled_channels_mask_(0),
      cdc_read_request_(false), cdc_read_stage_(0), cdc_read_value_(0),
      pending_config_count_(0) {
    module_name = "AD7147";
    module_mask_ = TouchSensor::generateModuleMask(static_cast<uint8_t>(i2c_bus), device_addr);
    supported_channel_count_ = AD7147_MAX_CHANNELS;
    calibration_tools_.pthis = this;
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
    uint16_t power_control_val = 0x02B0;
    ret &= configureStages(power_control_val, nullptr);

    initialized_ = ret;
    return ret;
}

// 配置管理接口实现
bool AD7147::loadConfig(const std::string& config_data) {
    if (!initialized_) {
        return false;
    }
    
    ConfigManager config_manager;
    if (!config_manager.fromString(config_data)) {
        return false;
    }
    
    // 从配置中加载stage设置
    for (int stage = 0; stage < 12; stage++) {
        std::string stage_prefix = "stage" + std::to_string(stage) + "_";
        
        stage_settings_.stages[stage].connection_6_0 = config_manager.getConfig(stage_prefix + "connection_6_0", stage_settings_.stages[stage].connection_6_0);
        stage_settings_.stages[stage].connection_12_7 = config_manager.getConfig(stage_prefix + "connection_12_7", stage_settings_.stages[stage].connection_12_7);
        stage_settings_.stages[stage].afe_offset = AFEOffsetRegister(config_manager.getConfig(stage_prefix + "afe_offset", static_cast<uint32_t>(stage_settings_.stages[stage].afe_offset.raw)));
        stage_settings_.stages[stage].sensitivity = SensitivityRegister(config_manager.getConfig(stage_prefix + "sensitivity", static_cast<uint32_t>(stage_settings_.stages[stage].sensitivity.raw)));
        stage_settings_.stages[stage].offset_low = config_manager.getConfig(stage_prefix + "offset_low", stage_settings_.stages[stage].offset_low);
        stage_settings_.stages[stage].offset_high = config_manager.getConfig(stage_prefix + "offset_high", stage_settings_.stages[stage].offset_high);
        stage_settings_.stages[stage].offset_high_clamp = config_manager.getConfig(stage_prefix + "offset_high_clamp", stage_settings_.stages[stage].offset_high_clamp);
        stage_settings_.stages[stage].offset_low_clamp = config_manager.getConfig(stage_prefix + "offset_low_clamp", stage_settings_.stages[stage].offset_low_clamp);
    }
    
    // 应用配置到硬件
    return apply_stage_settings();
}

std::string AD7147::saveConfig() const {
    ConfigManager config_manager;
    
    // 保存stage设置到配置
    for (int stage = 0; stage < 12; stage++) {
        std::string stage_prefix = "stage" + std::to_string(stage) + "_";
        
        config_manager.setConfig(stage_prefix + "connection_6_0", static_cast<uint32_t>(stage_settings_.stages[stage].connection_6_0));
        config_manager.setConfig(stage_prefix + "connection_12_7", static_cast<uint32_t>(stage_settings_.stages[stage].connection_12_7));
        config_manager.setConfig(stage_prefix + "afe_offset", static_cast<uint32_t>(stage_settings_.stages[stage].afe_offset.raw));
        config_manager.setConfig(stage_prefix + "sensitivity", static_cast<uint32_t>(stage_settings_.stages[stage].sensitivity.raw));
        config_manager.setConfig(stage_prefix + "offset_low", static_cast<uint32_t>(stage_settings_.stages[stage].offset_low));
        config_manager.setConfig(stage_prefix + "offset_high", static_cast<uint32_t>(stage_settings_.stages[stage].offset_high));
        config_manager.setConfig(stage_prefix + "offset_high_clamp", static_cast<uint32_t>(stage_settings_.stages[stage].offset_high_clamp));
        config_manager.setConfig(stage_prefix + "offset_low_clamp", static_cast<uint32_t>(stage_settings_.stages[stage].offset_low_clamp));
    }
    
    return config_manager.toString();
}

bool AD7147::setCustomSensitivitySettings(const std::string& settings_data) {
    if (!initialized_) {
        return false;
    }
    
    ConfigManager config_manager;
    if (!config_manager.fromString(settings_data)) {
        return false;
    }
    
    // 更新stage设置中的灵敏度配置
    for (int stage = 0; stage < 12; stage++) {
        std::string stage_key = "stage" + std::to_string(stage) + "_sensitivity";
        if (config_manager.hasConfig(stage_key)) {
            uint32_t sensitivity_value = config_manager.getConfig(stage_key, static_cast<uint32_t>(stage_settings_.stages[stage].sensitivity.raw));
            stage_settings_.stages[stage].sensitivity = SensitivityRegister(static_cast<uint16_t>(sensitivity_value));
        }
    }
    
    // 应用更新后的设置到硬件
    return apply_stage_settings();
}

bool AD7147::setStageConfig(uint8_t stage, const StageConfig& config) {
    if (!initialized_ || stage >= 12) {
        return false;
    }
    
    // 更新内存中的配置
    stage_settings_.stages[stage] = config;
    
    // 应用单个阶段配置到硬件
    uint8_t config_buffer[16];
    
    config_buffer[0] = (uint8_t)(config.connection_6_0 >> 8);
    config_buffer[1] = (uint8_t)(config.connection_6_0 & 0xFF);
    config_buffer[2] = (uint8_t)(config.connection_12_7 >> 8);
    config_buffer[3] = (uint8_t)(config.connection_12_7 & 0xFF);
    config_buffer[4] = (uint8_t)(config.afe_offset.raw >> 8);
    config_buffer[5] = (uint8_t)(config.afe_offset.raw & 0xFF);
    config_buffer[6] = (uint8_t)(config.sensitivity.raw >> 8);
    config_buffer[7] = (uint8_t)(config.sensitivity.raw & 0xFF);
    config_buffer[8] = (uint8_t)(config.offset_low >> 8);
    config_buffer[9] = (uint8_t)(config.offset_low & 0xFF);
    config_buffer[10] = (uint8_t)(config.offset_high >> 8);
    config_buffer[11] = (uint8_t)(config.offset_high & 0xFF);
    config_buffer[12] = (uint8_t)(config.offset_high_clamp >> 8);
    config_buffer[13] = (uint8_t)(config.offset_high_clamp & 0xFF);
    config_buffer[14] = (uint8_t)(config.offset_low_clamp >> 8);
    config_buffer[15] = (uint8_t)(config.offset_low_clamp & 0xFF);
    
    bool ret = true;
    for (uint8_t i = 0; i < 8; i++) {
        ret &= write_register(AD7147_REG_STAGE0_CONNECTION + (stage * AD7147_REG_STAGE_SIZE) + i, &config_buffer[i * 2], 2);
    }
    
    return ret;
}

StageConfig AD7147::getStageConfig(uint8_t stage) const {
    if (stage >= 12) {
        return StageConfig(); // 返回默认配置
    }
    
    return stage_settings_.stages[stage];
}

bool AD7147::readStageCDC(uint8_t stage, uint16_t& cdc_value) {
    if (!initialized_ || stage >= 12) {
        return false;
    }
    
    // 设置CDC读取请求
     cdc_read_stage_ = stage;
     cdc_read_request_ = true;
     
     // 等待读取完成（超时保护）
     uint32_t start_time = time_us_32();
     while (cdc_read_request_ && (time_us_32() - start_time) < 10000) {
         // 等待sample()处理CDC读取请求
     }
     
     if (!cdc_read_request_) {
         cdc_value = cdc_read_value_;
         return true;
     }
     
     return false;
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
    // 直接返回true，不进行实际的寄存器操作
    return true;
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
    static uint16_t status_regs;
    if (!initialized_) {
        result.timestamp_us = time_us_32();
        return result;
    }

    // 处理待应用的异步配置
    if (pending_config_count_) {
        setStageConfig(pending_configs_.stage, pending_configs_.config);
        pending_config_count_--;
    }

    // 处理CDC读取请求
    if (cdc_read_request_) {
        // 读取指定阶段的CDC数据
        uint16_t cdc_reg_addr = AD7147_REG_CDC_DATA + cdc_read_stage_;
        if (read_register(cdc_reg_addr, cdc_read_value_)) cdc_read_request_ = false;
    }
    
    read_register(AD7147_REG_STAGE_HIGH_INT_STATUS, status_regs);
    
    // 合并触摸状态并限制在24位范围内
    result.channel_mask = status_regs;

    // 留给校准模块



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
    return i2c_hal_->write_register(device_addr_, reg | 0x8000, value, size) == size;
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

// 应用stage设置到硬件的内联函数
inline bool AD7147::apply_stage_settings() {
    bool ret = true;
    uint8_t config_buffer[16];
    
    USB_LOG_DEBUG("AD7147 apply_stage_settings");
    // 配置每个阶段（Stage 0-11）
    for (uint8_t stage = 0; stage < 12; stage++) {
        const StageConfig& stage_config = stage_settings_.stages[stage];
        
        config_buffer[0] = (uint8_t)(stage_config.connection_6_0 >> 8);
        config_buffer[1] = (uint8_t)(stage_config.connection_6_0 & 0xFF);

        config_buffer[2] = (uint8_t)(stage_config.connection_12_7 >> 8);
        config_buffer[3] = (uint8_t)(stage_config.connection_12_7 & 0xFF);

        config_buffer[4] = (uint8_t)(stage_config.afe_offset.raw >> 8);
        config_buffer[5] = (uint8_t)(stage_config.afe_offset.raw & 0xFF);

        config_buffer[6] = (uint8_t)(stage_config.sensitivity.raw >> 8);
        config_buffer[7] = (uint8_t)(stage_config.sensitivity.raw & 0xFF);

        config_buffer[8] = (uint8_t)(stage_config.offset_low >> 8);
        config_buffer[9] = (uint8_t)(stage_config.offset_low & 0xFF);

        config_buffer[10] = (uint8_t)(stage_config.offset_high >> 8);
        config_buffer[11] = (uint8_t)(stage_config.offset_high & 0xFF);
        
        config_buffer[12] = (uint8_t)(stage_config.offset_high_clamp >> 8);
        config_buffer[13] = (uint8_t)(stage_config.offset_high_clamp & 0xFF);

        config_buffer[14] = (uint8_t)(stage_config.offset_low_clamp >> 8);
        config_buffer[15] = (uint8_t)(stage_config.offset_low_clamp & 0xFF);
        
        for (uint8_t i = 0; i < 8; i++) {
            ret &= write_register(AD7147_REG_STAGE0_CONNECTION + (stage * AD7147_REG_STAGE_SIZE) + i, &config_buffer[i * 2], 2);
        }
    }
    return ret;
}

bool AD7147::configureStages(uint16_t power_control_val, const uint16_t* connection_values) {
    bool ret = true;
    
    // 如果提供了connection_values，更新stage_settings_中的AFE偏移
    if (connection_values) {
        for (int i = 0; i < 12; i++) {
            stage_settings_.stages[i].afe_offset = connection_values[0];
        }
    }
    
    // 应用stage设置
    ret &= apply_stage_settings();
    USB_LOG_DEBUG("AD7147 ConfigureStages Register");
    // 使用结构体配置寄存器
    register_config_.pwr_control.raw = power_control_val;
    
    // 准备寄存器数据数组
    uint8_t reg_data[16];
    reg_data[0] = (uint8_t)(register_config_.pwr_control.raw >> 8);
    reg_data[1] = (uint8_t)(register_config_.pwr_control.raw & 0xFF);  // PWR_CONTROL 0x0
    reg_data[2] = (uint8_t)(register_config_.stage_cal_en.raw >> 8);
    reg_data[3] = (uint8_t)(register_config_.stage_cal_en.raw & 0xFF);  // STAGE_CAL_EN 0x1
    reg_data[4] = (uint8_t)(register_config_.amb_comp_ctrl0.raw >> 8);
    reg_data[5] = (uint8_t)(register_config_.amb_comp_ctrl0.raw & 0xFF);  // AMB_COMP_CTRL0 0x2
    reg_data[6] = (uint8_t)(register_config_.amb_comp_ctrl1.raw >> 8);
    reg_data[7] = (uint8_t)(register_config_.amb_comp_ctrl1.raw & 0xFF);  // AMB_COMP_CTRL1 0x3
    reg_data[8] = (uint8_t)(register_config_.amb_comp_ctrl2.raw >> 8);
    reg_data[9] = (uint8_t)(register_config_.amb_comp_ctrl2.raw & 0xFF);  // AMB_COMP_CTRL2 0x4
    reg_data[10] = (uint8_t)(register_config_.stage_low_int_enable >> 8);
    reg_data[11] = (uint8_t)(register_config_.stage_low_int_enable & 0xFF);  // STAGE_LOW_INT_ENABLE 0x5
    reg_data[12] = (uint8_t)(register_config_.stage_high_int_enable >> 8);
    reg_data[13] = (uint8_t)(register_config_.stage_high_int_enable & 0xFF);  // STAGE_HIGH_INT_ENABLE 0x6
    reg_data[14] = (uint8_t)(register_config_.stage_complete_int_enable >> 8);
    reg_data[15] = (uint8_t)(register_config_.stage_complete_int_enable & 0xFF);  // STAGE_COMPLETE_INT_ENABLE 0x7

    // 写入配置寄存器组
    bool cb = true;
    for (int i = 0; i < 8; i++) {
        cb &= write_register(AD7147_REG_PWR_CONTROL + i, &reg_data[i * 2], 2);
        if (i == 1) cb = true;  // 允许ADDR 1失败
        ret &= cb;
    }
    
    // 启用所有阶段校准
    register_config_.stage_cal_en.raw = 0x0FFF;
    uint8_t cal_en_data[2] = {
        (uint8_t)(register_config_.stage_cal_en.raw >> 8),
        (uint8_t)(register_config_.stage_cal_en.raw & 0xFF)
    };
    ret &= write_register(AD7147_REG_STAGE_CAL_EN, cal_en_data, 2);

    // 更新AMB_COMP_CTRL0配置
    register_config_.amb_comp_ctrl0.raw = 0x72F0;
    uint8_t amb_ctrl0_data[2] = {
        (uint8_t)(register_config_.amb_comp_ctrl0.raw >> 8),
        (uint8_t)(register_config_.amb_comp_ctrl0.raw & 0xFF)
    };
    ret &= write_register(AD7147_REG_AMB_COMP_CTRL0, amb_ctrl0_data, 2);
    
    return ret;
}

// 异步设置Stage配置
bool AD7147::setStageConfigAsync(uint8_t stage, const StageConfig& config) {
    if (stage >= 12) {
        return false;
    }
    if (!pending_config_count_) {
        pending_configs_.stage = stage;
        pending_configs_.config = config;
        pending_config_count_++;
    }
    
    // 没有空闲槽位
    return false;
}


