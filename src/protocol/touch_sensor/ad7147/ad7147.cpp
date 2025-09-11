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
      initialized_(false), i2c_bus_enum_(i2c_bus), enabled_stage(MIN(AD7147_MAX_CHANNELS, 12)), enabled_channels_mask_(0),
      cdc_read_request_(false), cdc_read_stage_(0), cdc_read_value_(0),
      sample_result_{.touch_mask = uint32_t(0)}, status_regs_(0),
      reconstructed_mask_(0), stage_status_(0), stage_index_(0), temp_mask_(0), channel_pos_(0),
      pending_config_count_(0), abnormal_channels_bitmap_(0) {
    module_name = "AD7147";
    module_mask_ = TouchSensor::generateModuleMask(static_cast<uint8_t>(i2c_bus), device_addr);
    supported_channel_count_ = AD7147_MAX_CHANNELS;
    supports_calibration_ = true;  // AD7147支持校准功能
    calibration_tools_.pthis = this;
    // 初始化sample_result_的touch_mask
    sample_result_.touch_mask = uint32_t(module_mask_ << 24);
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
    enabled_channels_mask_ = ((1 << AD7147_MAX_CHANNELS) - 1); // 13位
    
    // 可选：配置所有阶段（如果需要自定义配置）
    register_config_.pwr_control.bits.power_mode = 0;
    register_config_.pwr_control.bits.lp_conv_delay = 0;
    register_config_.pwr_control.bits.sequence_stage_num = 0xB;
    register_config_.pwr_control.bits.decimation = 3;
    register_config_.pwr_control.bits.sw_reset = 0;
    register_config_.pwr_control.bits.int_pol = 0;
    register_config_.pwr_control.bits.ext_source = 0;
    register_config_.pwr_control.bits.cdc_bias = 3;
    ret &= configureStages(nullptr);

    initialized_ = ret;
    return ret;
}

// 配置管理接口实现
bool AD7147::loadConfig(const std::string& config_data) {
    if (!initialized_) {
        return false;
    }
    
    SaveConfig config_manager;
    if (!config_manager.fromString(config_data)) {
        return false;
    }
    
    // 按固定顺序从配置中加载stage设置
    for (int32_t stage = 0; stage < 12; stage++) {
        stage_settings_.stages[stage].connection_6_0 = config_manager.readValue(stage_settings_.stages[stage].connection_6_0);
        stage_settings_.stages[stage].connection_12_7 = config_manager.readValue(stage_settings_.stages[stage].connection_12_7);
        stage_settings_.stages[stage].afe_offset = AFEOffsetRegister(config_manager.readValue(static_cast<uint32_t>(stage_settings_.stages[stage].afe_offset.raw)));
        stage_settings_.stages[stage].sensitivity = SensitivityRegister(config_manager.readValue(static_cast<uint32_t>(stage_settings_.stages[stage].sensitivity.raw)));
        stage_settings_.stages[stage].offset_low = config_manager.readValue(stage_settings_.stages[stage].offset_low);
        stage_settings_.stages[stage].offset_high = config_manager.readValue(stage_settings_.stages[stage].offset_high);
        stage_settings_.stages[stage].offset_high_clamp = config_manager.readValue(stage_settings_.stages[stage].offset_high_clamp);
        stage_settings_.stages[stage].offset_low_clamp = config_manager.readValue(stage_settings_.stages[stage].offset_low_clamp);
    }
    
    // 应用配置到硬件
    return apply_stage_settings();
}

std::string AD7147::saveConfig() const {
    SaveConfig config_manager;
    
    // 按固定顺序保存stage设置到配置
    for (int32_t stage = 0; stage < 12; stage++) {
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].connection_6_0));
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].connection_12_7));
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].afe_offset.raw));
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].sensitivity.raw));
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].offset_low));
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].offset_high));
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].offset_high_clamp));
        config_manager.writeValue(static_cast<uint32_t>(stage_settings_.stages[stage].offset_low_clamp));
    }
    
    return config_manager.toString();
}

bool AD7147::setCustomSensitivitySettings(const std::string& settings_data) {
    if (!initialized_) {
        return false;
    }
    
    SaveConfig config_manager;
    if (!config_manager.fromString(settings_data)) {
        return false;
    }
    
    // 按固定顺序解析配置并应用到stage设置
    for (int32_t stage = 0; stage < 12; stage++) {
        // 跳过连接设置，只读取敏感度相关的设置
        config_manager.readValue(stage_settings_.stages[stage].connection_6_0); // 跳过
        config_manager.readValue(stage_settings_.stages[stage].connection_12_7); // 跳过
        
        // 读取并应用敏感度相关设置
        uint32_t afe_offset_raw = config_manager.readValue(static_cast<uint32_t>(stage_settings_.stages[stage].afe_offset.raw));
        stage_settings_.stages[stage].afe_offset = AFEOffsetRegister(afe_offset_raw);
        
        uint32_t sensitivity_raw = config_manager.readValue(static_cast<uint32_t>(stage_settings_.stages[stage].sensitivity.raw));
        stage_settings_.stages[stage].sensitivity = SensitivityRegister(sensitivity_raw);
        
        // 跳过其他设置
        config_manager.readValue(stage_settings_.stages[stage].offset_low); // 跳过
        config_manager.readValue(stage_settings_.stages[stage].offset_high); // 跳过
        config_manager.readValue(stage_settings_.stages[stage].offset_high_clamp); // 跳过
        config_manager.readValue(stage_settings_.stages[stage].offset_low_clamp); // 跳过
    }
    
    // 应用更新后的设置到硬件
    return apply_stage_settings();
}

bool AD7147::setStageConfig(uint8_t stage, const PortConfig& config) {
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

PortConfig AD7147::getStageConfig(uint8_t stage) const {
    if (stage >= 12) {
        return PortConfig(); // 返回默认配置
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

bool AD7147::readStageCDC_direct(uint8_t stage, uint16_t& cdc_value) {
    if (!initialized_ || stage >= 12) return false;
    // 读取指定阶段的CDC数据，直接访问CDC寄存器
    uint16_t cdc_reg_addr = AD7147_REG_CDC_DATA + stage;
    return read_register(cdc_reg_addr, cdc_value);
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
    i2c_hal_->read_register(device_addr_, (AD7147_REG_STAGE_HIGH_INT_STATUS | 0x8000), (uint8_t*)&status_regs_, 2);
    __asm__ volatile (
            "rev16 %0, %0\n"
            : "+r" (status_regs_)
            :: "cc"
    );
    
    // 重建通道映射：将stage反馈映射回正确的通道位置
    reconstructed_mask_ = 0;
    stage_status_ = ~status_regs_; // 反转状态位（触摸时为1）
    
    stage_index_ = 0;
    temp_mask_ = enabled_channels_mask_;
    
    // 使用位运算快速找到每个启用通道的位置并映射stage状态
    while (temp_mask_ && stage_index_ < enabled_stage) {
        // 找到下一个启用通道的位置（从低位开始）
        channel_pos_ = __builtin_ctz(temp_mask_); // 计算尾随零的个数
        
        // 如果当前stage有触摸状态，设置对应通道位
        if (stage_status_ & (1U << stage_index_)) {
            reconstructed_mask_ |= (1UL << channel_pos_);
        }
        
        // 清除已处理的通道位，继续下一个
        temp_mask_ &= temp_mask_ - 1; // 清除最低位的1
        stage_index_++;
    }
    
    sample_result_.channel_mask = reconstructed_mask_;
    
    // 留给校准模块
    if (calibration_tools_.calibration_state_)
        calibration_tools_.CalibrationLoop(sample_result_.channel_mask);
    sample_result_.timestamp_us = time_us_32();
    
    return sample_result_;
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
    // 更新启用的阶段数量
    enabled_stage = 0;
    for (uint8_t stage = 0; stage < AD7147_MAX_CHANNELS; stage++) {
        if (enabled_channels_mask_ & (1UL << stage)) {
            enabled_stage++;
        }
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
    // 通道12始终禁用以简化实现（仅支持12个stage，无法实现13通道的多点触控）
    uint32_t working_mask = enabled_channels_mask_ & 0x0FFF; // 只处理通道0-11
    
    // 获取启用通道的数量和索引
    uint8_t enabled_channels[12];
    uint8_t enabled_count = 0;
    
    for (uint8_t ch = 0; ch < 12; ch++) {
        if (working_mask & (1UL << ch)) {
            enabled_channels[enabled_count] = ch;
            enabled_count++;
        }
    }

    bool ok = true;
    uint8_t config_buffer[16];
    
    // 配置启用的通道到对应的stage
    for (uint8_t stage = 0; stage < 12; stage++) {
        if (stage < enabled_count) {
            // 使用启用通道的配置
            uint8_t channel = enabled_channels[stage];
            stage_settings_.stages[stage].connection_6_0 = channel_connections[channel][0];
            stage_settings_.stages[stage].connection_12_7 = channel_connections[channel][1];
        } else {
            // 未使用的stage设置为0（禁用）
            stage_settings_.stages[stage].connection_6_0 = 0x0000;
            stage_settings_.stages[stage].connection_12_7 = 0x0000;
        }
        
        // 写入stage连接配置
        const PortConfig& stage_config = stage_settings_.stages[stage];
        config_buffer[0] = (uint8_t)(stage_config.connection_6_0 >> 8);
        config_buffer[1] = (uint8_t)(stage_config.connection_6_0 & 0xFF);
        config_buffer[2] = (uint8_t)(stage_config.connection_12_7 >> 8);
        config_buffer[3] = (uint8_t)(stage_config.connection_12_7 & 0xFF);
        
        uint16_t stage_base_addr = AD7147_REG_STAGE0_CONNECTION + (stage * AD7147_REG_STAGE_SIZE);
        ok &= write_register(stage_base_addr + AD7147_STAGE_CONNECTION_OFFSET, config_buffer, 4);
    }
    
    // 设置校准使能：只启用实际使用的stage
    uint16_t cal_enable_mask = 0;
    if (enabled_count > 0) {
        cal_enable_mask = (1 << enabled_count) - 1; // 启用stage 0到enabled_count-1
    }
    
    uint8_t cal_en_data[2] = {
        (uint8_t)(cal_enable_mask >> 8),
        (uint8_t)(cal_enable_mask & 0xFF)
    };
    ok &= write_register(AD7147_REG_STAGE_CAL_EN, cal_en_data, 2);
    
    // 设置高中断使能
    uint8_t high_int_data[2] = {
        (uint8_t)(cal_enable_mask >> 8),
        (uint8_t)(cal_enable_mask & 0xFF)
    };
    ok &= write_register(AD7147_REG_STAGE_HIGH_INT_EN, high_int_data, 2);
    
    // 设置sequence_stage_num为启用通道数量减1
    if (enabled_count > 0) {
        register_config_.pwr_control.bits.sequence_stage_num = enabled_count - 1;
    } else {
        register_config_.pwr_control.bits.sequence_stage_num = 0;
    }
    
    // 写入电源控制寄存器
    uint8_t pwr_ctrl_data[2] = {
        (uint8_t)(register_config_.pwr_control.raw >> 8),
        (uint8_t)(register_config_.pwr_control.raw & 0xFF)
    };
    ok &= write_register(AD7147_REG_PWR_CONTROL, pwr_ctrl_data, 2);
    
    return ok;
}

// 应用stage设置到硬件的内联函数
inline bool AD7147::apply_stage_settings() {
    bool ret = true;
    uint8_t config_buffer[16];
    
    USB_LOG_DEBUG("AD7147 apply_stage_settings");
    // 配置每个阶段（Stage 0-11）
    for (uint8_t stage = 0; stage < 12; stage++) {
        const PortConfig& stage_config = stage_settings_.stages[stage];
        
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

bool AD7147::configureStages(const uint16_t* connection_values) {
    bool ret = true;
    
    // 如果提供了connection_values，更新stage_settings_中的AFE偏移
    if (connection_values) {
        for (int32_t i = 0; i < 12; i++) {
            stage_settings_.stages[i].afe_offset = connection_values[0];
        }
    }
    
    // 应用stage设置
    ret &= apply_stage_settings();
    USB_LOG_DEBUG("AD7147 ConfigureStages Register");
    
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
    for (int32_t i = 0; i < 8; i++) {
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
    register_config_.amb_comp_ctrl0.raw = 0x32FF; 
    register_config_.amb_comp_ctrl0.bits.forced_cal = false;
    uint8_t amb_ctrl0_data[2] = {
        (uint8_t)(register_config_.amb_comp_ctrl0.raw >> 8),
        (uint8_t)(register_config_.amb_comp_ctrl0.raw & 0xFF)
    };
    ret &= write_register(AD7147_REG_AMB_COMP_CTRL0, amb_ctrl0_data, 2);
    
    return ret;
}

// 异步设置Stage配置
bool AD7147::setStageConfigAsync(uint8_t stage, const PortConfig& config) {
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

bool AD7147::startAutoOffsetCalibration() {
    if (!initialized_) return false;
    calibration_tools_.pthis = this;
    // 启动内部校准工具
    calibration_tools_.calibration_state_ = AD7147::CalibrationTools::PROCESS;
    return true;
}

bool AD7147::isAutoOffsetCalibrationActive() const {
    return initialized_ && (calibration_tools_.calibration_state_ != AD7147::CalibrationTools::IDLE);
}

uint8_t AD7147::getAutoOffsetCalibrationTotalProgress() const {
    // 运行中时，直接返回CalibrationLoop中计算的平均进度
    return calibration_tools_.stage_process;
}

// TouchSensor基类虚函数实现
bool AD7147::calibrateSensor() {
    // 启动自动偏移校准
    return startAutoOffsetCalibration();
}

uint8_t AD7147::getCalibrationProgress() const {
    // 返回自动偏移校准的总进度
    return getAutoOffsetCalibrationTotalProgress();
}

bool AD7147::setLEDEnabled(bool enabled) {
    // AD7147 LED control through register 0x005 bits [13:12]
    // 00 = disable GPIO pin
    // 01 = configure GPIO as an input  
    // 10 = configure GPIO as an active low output
    // 11 = configure GPIO as an active high output
    
    uint16_t reg_value;
    if (!read_register(0x005, reg_value)) {
        return false;
    }
    
    // Clear bits [13:12]
    reg_value &= ~(0x3 << 12);
    
    // Set bits [13:12] = 00 (disable GPIO pin)
    reg_value |= (enabled ? 0x3 : 0x2) << 12;

    uint8_t reg_data[2] = {
        (uint8_t)(reg_value >> 8),
        (uint8_t)(reg_value & 0xFF)
    };
    
    return write_register(0x005, reg_data, 2);
}

uint32_t AD7147::getAbnormalChannelMask() const {
    // 返回异常通道位图
    return abnormal_channels_bitmap_;
}


