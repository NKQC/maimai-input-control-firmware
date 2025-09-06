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
    uint16_t power_control_val = 0xC2B0;
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
    
    // 处理自动偏移校准
    if (auto_offset_calibration_.is_active) {
        processAutoOffsetCalibration();
    }

    read_register(AD7147_REG_STAGE_HIGH_INT_STATUS, status_regs);
    
    // 合并触摸状态并限制在24位范围内
    result.channel_mask = status_regs;
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
    // 配置第一寄存器组
    uint8_t reg_config[16];
    reg_config[0] = (uint8_t)(power_control_val >> 8);
    reg_config[1] = (uint8_t)(power_control_val & 0xFF);  // PWR_CONTROL 0x0
    reg_config[2] = 0x00;
    reg_config[3] = 0x00;            // STAGE_CAL_EN  此阶段必须为0 为保持序号与地址一致 保留 0x1
    reg_config[4] = 0x32;
    reg_config[5] = 0x30;            // AMB_COMP_CTRL0 0x2
    reg_config[6] = 0xC4;
    reg_config[7] = 0x1F;            // AMB_COMP_CTRL1 0x3
    reg_config[8] = 0x08;
    reg_config[9] = 0x32;            // AMB_COMP_CTRL2 0x4
    reg_config[10] = 0x00;
    reg_config[11] = 0x00;            // STAGE_LOW_INT_ENABLE 0x5
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
    // 启用所有阶段校准
    ret &= write_register(AD7147_REG_STAGE_CAL_EN, reg_config, 2);

    reg_config[0] = 0x72;
    reg_config[1] = 0x30;
    // 启用所有阶段校准
    ret &= write_register(AD7147_REG_AMB_COMP_CTRL0, reg_config, 2);
    
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

// ==================== 自动偏移校准实现 ====================

// 启动自动偏移校准（所有启用通道）
bool AD7147::startAutoOffsetCalibration() {
    if (!initialized_ || auto_offset_calibration_.is_active) {
        return false;
    }
    
    // 初始化全局校准状态
    auto_offset_calibration_.reset_status();
    auto_offset_calibration_.is_active = true;
    auto_offset_calibration_.current_channel_index = 0;
    auto_offset_calibration_.completed_channels = 0;
    auto_offset_calibration_.start_time_ms = to_ms_since_boot(get_absolute_time());
    
    // 统计启用的通道数并初始化各通道校准状态
    uint8_t channel_count = 0;
    for (uint8_t stage = 0; stage < 12; stage++) {
        if (enabled_channels_mask_ & (1 << stage)) {
            resetChannelCalibration(auto_offset_calibration_.channels[channel_count], stage);
            channel_count++;
        }
    }
    auto_offset_calibration_.total_channels = channel_count;
    
    if (channel_count == 0) {
        auto_offset_calibration_.is_active = false;
        return false;
    }
    cdc_read_request_ = true; // 预采样
    USB_LOG_DEBUG("AD7147: Started auto offset calibration for %d channels", channel_count);
    return true;
}

// 启动指定阶段的自动偏移校准
bool AD7147::startAutoOffsetCalibrationForStage(uint8_t stage) {
    if (!initialized_ || stage >= 12 || auto_offset_calibration_.is_active) {
        return false;
    }
    
    // 初始化单通道校准
    auto_offset_calibration_.is_active = true;
    auto_offset_calibration_.current_channel_index = 0;
    auto_offset_calibration_.completed_channels = 0;
    auto_offset_calibration_.total_channels = 1;
    auto_offset_calibration_.start_time_ms = to_ms_since_boot(get_absolute_time());
    
    resetChannelCalibration(auto_offset_calibration_.channels[0], stage);
    
    USB_LOG_DEBUG("AD7147: Started auto offset calibration for stage %d", stage);
    return true;
}

// 检查是否正在进行自动偏移校准
bool AD7147::isAutoOffsetCalibrationActive() const {
    return auto_offset_calibration_.is_active;
}

// 获取指定阶段的校准结果
AutoOffsetResult AD7147::getAutoOffsetCalibrationResult(uint8_t stage) const {
    for (uint8_t i = 0; i < auto_offset_calibration_.total_channels; i++) {
        if (auto_offset_calibration_.channels[i].stage == stage) {
            // 基于state返回结果，移除result字段依赖
            switch (auto_offset_calibration_.channels[i].state) {
                case AutoOffsetState::COMPLETED:
                    return AutoOffsetResult::SUCCESS;
                case AutoOffsetState::FAILED:
                case AutoOffsetState::CALIBRATION_ERROR:
                    return AutoOffsetResult::FAILED;
                case AutoOffsetState::IDLE:
                    return AutoOffsetResult::TIMEOUT;  // 使用TIMEOUT表示未开始
                default:
                    return AutoOffsetResult::OUT_OF_RANGE;  // 使用OUT_OF_RANGE表示进行中
            }
        }
    }
    return AutoOffsetResult::HARDWARE_ERROR;
}

// 获取校准进度（0-100）
uint8_t AD7147::getAutoOffsetCalibrationProgress() const {
    if (!auto_offset_calibration_.is_active || auto_offset_calibration_.total_channels == 0) {
        return 255;
    }
    return (auto_offset_calibration_.completed_channels * 255) / auto_offset_calibration_.total_channels;
}

// 停止自动偏移校准
void AD7147::stopAutoOffsetCalibration() {
    auto_offset_calibration_.is_active = false;
    USB_LOG_DEBUG("AD7147: Auto offset calibration stopped");
}

// 在sample()中调用，处理自动偏移校准状态机
void AD7147::processAutoOffsetCalibration() {
    
    // 检查超时（120秒）
    uint32_t current_time = us_to_ms(time_us_32());
    if (current_time - auto_offset_calibration_.start_time_ms > 120000) {
        USB_LOG_DEBUG("AD7147: Auto offset calibration timeout");
        stopAutoOffsetCalibration();
        return;
    }
    
    // 处理当前通道
    if (auto_offset_calibration_.current_channel_index < auto_offset_calibration_.total_channels) {
        ChannelOffsetCalibration& current_channel = auto_offset_calibration_.channels[auto_offset_calibration_.current_channel_index];
        
        // 如果需要读取CDC且没有正在进行的读取请求，则发起读取
        if (current_channel.state != AutoOffsetState::IDLE && 
            current_channel.state != AutoOffsetState::COMPLETED && 
            !cdc_read_request_) {
            cdc_read_stage_ = current_channel.stage;
            cdc_read_request_ = true;
            current_channel.current_cdc = cdc_read_value_;
        }
        
        if (calibrateSingleChannel(current_channel)) {
            // 当前通道校准完成，移动到下一个通道
            auto_offset_calibration_.completed_channels++;
            auto_offset_calibration_.current_channel_index++;
            auto_offset_calibration_.start_time_ms = current_time; // 重置超时
            
            USB_LOG_DEBUG("AD7147: Stage %d calibration completed with state %d", 
                         current_channel.stage, static_cast<int>(current_channel.state));
        }
    } else {
        // 所有通道校准完成
        USB_LOG_DEBUG("AD7147: All channels calibration completed");
        stopAutoOffsetCalibration();
     }
 }

// 校准单个通道 - 线性状态机
bool AD7147::calibrateSingleChannel(ChannelOffsetCalibration& channel_cal) {
    switch (channel_cal.state) {
        case AutoOffsetState::IDLE:
            // 开始校准，初始化状态
            channel_cal.pos_afe_reverse_enabled = false;
            channel_cal.neg_afe_reverse_enabled = false;
            channel_cal.adjustment_direction = 0;
            channel_cal.continuous_non_trigger_count = 0;
            channel_cal.continuous_non_extreme_count = 0;
            channel_cal.state = AutoOffsetState::ADJUSTING_POS_AFE;
            USB_LOG_DEBUG("AD7147: Stage %d starting calibration", channel_cal.stage);
            return false;
            
        case AutoOffsetState::ADJUSTING_POS_AFE: {
            // 步骤1: 如果CDC是极值，尝试向正极AFE拉偏移
            USB_LOG_DEBUG("AD7147: Stage %d ADJUSTING_POS_AFE - CDC: %d", channel_cal.stage, channel_cal.current_cdc);
            
            // 检查是否连续5次不是极值
            bool is_extreme = (channel_cal.current_cdc <= AD7147_CDC_EXTREME_LOW_THRESHOLD || 
                              channel_cal.current_cdc >= AD7147_CDC_EXTREME_HIGH_THRESHOLD);
            
            if (!is_extreme) {
                channel_cal.continuous_non_extreme_count++;
                if (channel_cal.continuous_non_extreme_count >= 5) {
                    // 连续5次不是极值，进入基线调整
                    channel_cal.adjustment_direction = 0; // 正向调整
                    channel_cal.state = AutoOffsetState::FINE_TUNING_TO_BASELINE;
                    USB_LOG_DEBUG("AD7147: Stage %d CDC stable, moving to baseline tuning", channel_cal.stage);
                    return false;
                }
            } else {
                channel_cal.continuous_non_extreme_count = 0;
            }
            
            StageConfig config = getStageConfig(channel_cal.stage);
            if (config.afe_offset.bits.pos_afe_offset < AD7147_AFE_OFFSET_MAX) {
                config.afe_offset.bits.pos_afe_offset++;
                if (setStageConfigAsync(channel_cal.stage, config)) {
                    USB_LOG_DEBUG("AD7147: Stage %d pos AFE: %d", channel_cal.stage, config.afe_offset.bits.pos_afe_offset);
                }
            } else {
                // 正向AFE已达到极值，进入下一步
                channel_cal.state = AutoOffsetState::TESTING_NEG_AFE_REVERSE;
                USB_LOG_DEBUG("AD7147: Stage %d pos AFE max, testing neg AFE reverse", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::TESTING_NEG_AFE_REVERSE: {
            // 步骤2: 把负向AFE设置反向
            USB_LOG_DEBUG("AD7147: Stage %d TESTING_NEG_AFE_REVERSE - CDC: %d", channel_cal.stage, channel_cal.current_cdc);
            
            StageConfig config = getStageConfig(channel_cal.stage);
            config.afe_offset.bits.neg_afe_offset_swap = 1;
            channel_cal.neg_afe_reverse_enabled = true;
            
            if (setStageConfigAsync(channel_cal.stage, config)) {
                channel_cal.state = AutoOffsetState::ADJUSTING_NEG_AFE;
                USB_LOG_DEBUG("AD7147: Stage %d enabling neg AFE reverse", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::ADJUSTING_NEG_AFE: {
            // 步骤3: 把负AFE拉到极值
            USB_LOG_DEBUG("AD7147: Stage %d ADJUSTING_NEG_AFE - CDC: %d", channel_cal.stage, channel_cal.current_cdc);
            
            // 检查是否连续5次不是极值
            bool is_extreme = (channel_cal.current_cdc <= AD7147_CDC_EXTREME_LOW_THRESHOLD || 
                              channel_cal.current_cdc >= AD7147_CDC_EXTREME_HIGH_THRESHOLD);
            
            if (!is_extreme) {
                channel_cal.continuous_non_extreme_count++;
                if (channel_cal.continuous_non_extreme_count >= 5) {
                    // 连续5次不是极值，进入基线调整
                    channel_cal.adjustment_direction = 1; // 负向调整
                    channel_cal.state = AutoOffsetState::FINE_TUNING_TO_BASELINE;
                    USB_LOG_DEBUG("AD7147: Stage %d CDC stable, moving to baseline tuning (neg direction)", channel_cal.stage);
                    return false;
                }
            } else {
                channel_cal.continuous_non_extreme_count = 0;
            }
            
            StageConfig config = getStageConfig(channel_cal.stage);
            if (config.afe_offset.bits.neg_afe_offset < AD7147_AFE_OFFSET_MAX) {
                config.afe_offset.bits.neg_afe_offset++;
                if (setStageConfigAsync(channel_cal.stage, config)) {
                    USB_LOG_DEBUG("AD7147: Stage %d adjusting neg AFE to %d", 
                                 channel_cal.stage, config.afe_offset.bits.neg_afe_offset);
                }
            } else {
                // 负向AFE已达到极值，进入下一步
                channel_cal.state = AutoOffsetState::RESET_NEG_AFE_REVERSE;
                USB_LOG_DEBUG("AD7147: Stage %d neg AFE reached max, resetting neg AFE reverse", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::RESET_NEG_AFE_REVERSE: {
            // 步骤4: 如果仍然极值，取消负向AFE设置反向
            USB_LOG_DEBUG("AD7147: Stage %d RESET_NEG_AFE_REVERSE - CDC: %d", channel_cal.stage, channel_cal.current_cdc);
            
            StageConfig config = getStageConfig(channel_cal.stage);
            config.afe_offset.bits.neg_afe_offset_swap = 0;
            config.afe_offset.bits.neg_afe_offset = 0;
            channel_cal.neg_afe_reverse_enabled = false;
            
            if (setStageConfigAsync(channel_cal.stage, config)) {
                channel_cal.state = AutoOffsetState::RESET_POS_AFE_ZERO;
                USB_LOG_DEBUG("AD7147: Stage %d resetting neg AFE reverse", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::RESET_POS_AFE_ZERO: {
            // 步骤5: 把正向AFE设置为0
            USB_LOG_DEBUG("AD7147: Stage %d RESET_POS_AFE_ZERO - CDC: %d", channel_cal.stage, channel_cal.current_cdc);
            
            // 检查是否连续5次不是极值
            bool is_extreme = (channel_cal.current_cdc <= AD7147_CDC_EXTREME_LOW_THRESHOLD || 
                              channel_cal.current_cdc >= AD7147_CDC_EXTREME_HIGH_THRESHOLD);
            
            if (!is_extreme) {
                channel_cal.continuous_non_extreme_count++;
                if (channel_cal.continuous_non_extreme_count >= 5) {
                    // 连续5次不是极值，进入基线调整
                    channel_cal.adjustment_direction = 0; // 正向调整
                    channel_cal.state = AutoOffsetState::FINE_TUNING_TO_BASELINE;
                    USB_LOG_DEBUG("AD7147: Stage %d CDC stable, moving to baseline tuning", channel_cal.stage);
                    return false;
                }
            } else {
                channel_cal.continuous_non_extreme_count = 0;
            }
            
            StageConfig config = getStageConfig(channel_cal.stage);
            config.afe_offset.bits.pos_afe_offset = 0;
            
            if (setStageConfigAsync(channel_cal.stage, config)) {
                channel_cal.state = AutoOffsetState::TESTING_POS_AFE_REVERSE;
                USB_LOG_DEBUG("AD7147: Stage %d resetting pos AFE to 0", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::TESTING_POS_AFE_REVERSE: {
            // 步骤6: 如果仍然为极值，把正向AFE设置为反向
            USB_LOG_DEBUG("AD7147: Stage %d TESTING_POS_AFE_REVERSE - CDC: %d", channel_cal.stage, channel_cal.current_cdc);
            
            StageConfig config = getStageConfig(channel_cal.stage);
            config.afe_offset.bits.pos_afe_offset_swap = 1;
            channel_cal.pos_afe_reverse_enabled = true;
            
            if (setStageConfigAsync(channel_cal.stage, config)) {
                channel_cal.state = AutoOffsetState::ADJUSTING_POS_AFE_REVERSE;
                USB_LOG_DEBUG("AD7147: Stage %d enabling pos AFE reverse", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::ADJUSTING_POS_AFE_REVERSE: {
            // 步骤7: 把正向AFE设置为极值
            USB_LOG_DEBUG("AD7147: Stage %d ADJUSTING_POS_AFE_REVERSE - CDC: %d", channel_cal.stage, channel_cal.current_cdc);
            
            // 检查是否连续5次不是极值
            bool is_extreme = (channel_cal.current_cdc <= AD7147_CDC_EXTREME_LOW_THRESHOLD || 
                              channel_cal.current_cdc >= AD7147_CDC_EXTREME_HIGH_THRESHOLD);
            
            if (!is_extreme) {
                channel_cal.continuous_non_extreme_count++;
                if (channel_cal.continuous_non_extreme_count >= 5) {
                    // 连续5次不是极值，进入基线调整
                    channel_cal.adjustment_direction = 0; // 正向调整（反向模式）
                    channel_cal.state = AutoOffsetState::FINE_TUNING_TO_BASELINE;
                    USB_LOG_DEBUG("AD7147: Stage %d CDC stable, moving to baseline tuning (pos reverse)", channel_cal.stage);
                    return false;
                }
            } else {
                channel_cal.continuous_non_extreme_count = 0;
            }
            
            StageConfig config = getStageConfig(channel_cal.stage);
            if (config.afe_offset.bits.pos_afe_offset < AD7147_AFE_OFFSET_MAX) {
                config.afe_offset.bits.pos_afe_offset++;
                if (setStageConfigAsync(channel_cal.stage, config)) {
                    USB_LOG_DEBUG("AD7147: Stage %d adjusting pos AFE reverse to %d", 
                                 channel_cal.stage, config.afe_offset.bits.pos_afe_offset);
                }
            } else {
                // 如果还是极值，放弃这个通道
                channel_cal.state = AutoOffsetState::FAILED;
                USB_LOG_DEBUG("AD7147: Stage %d calibration failed - still extreme after all attempts", channel_cal.stage);
                return true;
            }
            return false;
        }
        
        case AutoOffsetState::FINE_TUNING_TO_BASELINE: {
            // 精细调整CDC到基线附近
            int32_t cdc_diff = (int32_t)channel_cal.current_cdc - (int32_t)AD7147_CDC_BASELINE;
            USB_LOG_DEBUG("AD7147: Stage %d FINE_TUNING_TO_BASELINE - CDC: %d, diff: %ld", 
                         channel_cal.stage, channel_cal.current_cdc, cdc_diff);
            
            if (abs(cdc_diff) <= AD7147_AUTO_OFFSET_TOLERANCE) {
                // 已接近基线，检查触发状态
                channel_cal.state = AutoOffsetState::CHECKING_TRIGGER_STATUS;
                USB_LOG_DEBUG("AD7147: Stage %d reached baseline tolerance, checking trigger status", channel_cal.stage);
                return false;
            }
            
            StageConfig config = getStageConfig(channel_cal.stage);
            bool config_changed = false;
            
            // 根据调整方向和CDC差值进行微调
            if (channel_cal.adjustment_direction == 0) { // 正向调整
                if (cdc_diff > 0 && config.afe_offset.bits.pos_afe_offset > 0) {
                    config.afe_offset.bits.pos_afe_offset--;
                    config_changed = true;
                } else if (cdc_diff < 0 && config.afe_offset.bits.pos_afe_offset < AD7147_AFE_OFFSET_MAX) {
                    config.afe_offset.bits.pos_afe_offset++;
                    config_changed = true;
                }
            } else { // 负向调整
                if (cdc_diff > 0 && config.afe_offset.bits.neg_afe_offset > 0) {
                    config.afe_offset.bits.neg_afe_offset--;
                    config_changed = true;
                } else if (cdc_diff < 0 && config.afe_offset.bits.neg_afe_offset < AD7147_AFE_OFFSET_MAX) {
                    config.afe_offset.bits.neg_afe_offset++;
                    config_changed = true;
                }
            }
            
            if (config_changed && setStageConfigAsync(channel_cal.stage, config)) {
                USB_LOG_DEBUG("AD7147: Stage %d fine tuning adjustment made", channel_cal.stage);
            } else {
                // 无法进一步调整，检查触发状态
                channel_cal.state = AutoOffsetState::CHECKING_TRIGGER_STATUS;
                USB_LOG_DEBUG("AD7147: Stage %d no further adjustment possible, checking trigger status", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::CHECKING_TRIGGER_STATUS: {
            // 检查通道触发状态
            bool is_triggered = checkChannelTriggered(channel_cal.stage);
            USB_LOG_DEBUG("AD7147: Stage %d CHECKING_TRIGGER_STATUS - CDC: %d, triggered: %s, count: %d", 
                         channel_cal.stage, channel_cal.current_cdc, is_triggered ? "yes" : "no", 
                         channel_cal.continuous_non_trigger_count);
            
            if (!is_triggered) {
                channel_cal.continuous_non_trigger_count++;
                if (channel_cal.continuous_non_trigger_count >= AD7147_CONTINUOUS_NON_TRIGGER_THRESHOLD) {
                    // 连续10次未触发，校准成功
                    channel_cal.state = AutoOffsetState::COMPLETED;
                    USB_LOG_DEBUG("AD7147: Stage %d calibration completed successfully", channel_cal.stage);
                    return true;
                }
            } else {
                // 通道被触发，需要调整灵敏度
                channel_cal.continuous_non_trigger_count = 0;
                channel_cal.state = AutoOffsetState::ADJUSTING_SENSITIVITY;
                return false;
            }
            
            // 继续检查触发状态
            USB_LOG_DEBUG("AD7147: Stage %d continuing trigger status check", channel_cal.stage);
            return false;
        }
        
        case AutoOffsetState::ADJUSTING_SENSITIVITY: {
            // 使用新的基于极值范围的灵敏度调整方法
            return adjustSensitivity(channel_cal.stage, channel_cal);
        }
        
        case AutoOffsetState::ADJUSTING_OFFSET_RANGE: {
            // 提升offset_low和offset_high
            StageConfig config = getStageConfig(channel_cal.stage);
            bool config_changed = false;
            USB_LOG_DEBUG("AD7147: Stage %d ADJUSTING_OFFSET_RANGE - CDC: %d, offset_low: %d, offset_high: %d", 
                         channel_cal.stage, channel_cal.current_cdc, config.offset_low, config.offset_high);
            
            if (config.offset_low < (0xFFFF - AD7147_OFFSET_ADJUSTMENT_STEP) && 
                config.offset_high < (0xFFFF - AD7147_OFFSET_ADJUSTMENT_STEP)) {
                config.offset_low += AD7147_OFFSET_ADJUSTMENT_STEP;
                config.offset_high += AD7147_OFFSET_ADJUSTMENT_STEP;
                config_changed = true;
                USB_LOG_DEBUG("AD7147: Stage %d adjusting offset range to low=%d, high=%d", 
                             channel_cal.stage, config.offset_low, config.offset_high);
            } else {
                // offset已达到极值，调整峰值检测
                channel_cal.state = AutoOffsetState::ADJUSTING_PEAK_DETECT;
                USB_LOG_DEBUG("AD7147: Stage %d offset range at maximum, adjusting peak detect", channel_cal.stage);
                return false;
            }
            
            if (config_changed && setStageConfigAsync(channel_cal.stage, config)) {
                channel_cal.state = AutoOffsetState::CHECKING_TRIGGER_STATUS;
                USB_LOG_DEBUG("AD7147: Stage %d offset range adjusted, checking trigger status", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::ADJUSTING_PEAK_DETECT: {
            // 提升pos_peak_detect和neg_peak_detect
            StageConfig config = getStageConfig(channel_cal.stage);
            bool config_changed = false;
            USB_LOG_DEBUG("AD7147: Stage %d ADJUSTING_PEAK_DETECT - CDC: %d, pos_peak: %d, neg_peak: %d", 
                         channel_cal.stage, channel_cal.current_cdc, 
                         config.sensitivity.bits.pos_peak_detect,
                         config.sensitivity.bits.neg_peak_detect);
            
            if (config.sensitivity.bits.pos_peak_detect < 7 && 
                config.sensitivity.bits.neg_peak_detect < 7) {
                config.sensitivity.bits.pos_peak_detect++;
                config.sensitivity.bits.neg_peak_detect++;
                config_changed = true;
                USB_LOG_DEBUG("AD7147: Stage %d adjusting peak detect to pos=%d, neg=%d", 
                             channel_cal.stage, config.sensitivity.bits.pos_peak_detect,
                             config.sensitivity.bits.neg_peak_detect);
            } else {
                // 峰值检测已达到极值，仍然触发，宣告失败
                channel_cal.state = AutoOffsetState::FAILED;
                USB_LOG_DEBUG("AD7147: Stage %d calibration failed - still triggered after all adjustments", channel_cal.stage);
                return true;
            }
            
            if (config_changed && setStageConfigAsync(channel_cal.stage, config)) {
                channel_cal.state = AutoOffsetState::CHECKING_TRIGGER_STATUS;
                USB_LOG_DEBUG("AD7147: Stage %d peak detect adjusted, checking trigger status", channel_cal.stage);
            }
            return false;
        }
        
        case AutoOffsetState::COMPLETED:
        case AutoOffsetState::FAILED:
        case AutoOffsetState::CALIBRATION_ERROR:
            return true;
    }
    return false;
}

// 调整AFE偏移（二分法）
bool AD7147::adjustAFEOffset(uint8_t stage, ChannelOffsetCalibration& channel_cal) {
    StageConfig current_config = getStageConfig(stage);
    int32_t cdc_diff = static_cast<int32_t>(channel_cal.current_cdc) - static_cast<int32_t>(channel_cal.target_cdc);
    
    USB_LOG_DEBUG("AD7147: Stage %d adjusting AFE, CDC=%d, target=%d, diff=%d, strategy=%d", 
                 stage, channel_cal.current_cdc, channel_cal.target_cdc, cdc_diff, (int)channel_cal.current_strategy);
    
    // 检测是否卡住（连续多次调整但CDC值无明显变化）
    if (channel_cal.cdc_trend == 0) {
        channel_cal.stuck_count++;
        if (channel_cal.stuck_count >= 3) {
            USB_LOG_DEBUG("AD7147: Stage %d stuck detected, switching to reverse search", stage);
            channel_cal.current_strategy = CalibrationStrategy::REVERSE_SEARCH;
            channel_cal.stuck_count = 0;
            channel_cal.reverse_direction = !channel_cal.reverse_direction;
        }
    } else {
        channel_cal.stuck_count = 0;
    }
    
    bool adjustment_made = false;
    
    // 根据策略选择调整方法
    switch (channel_cal.current_strategy) {
        case CalibrationStrategy::REVERSE_SEARCH:
            // 反向搜索：如果正常方向不行，尝试反向
            if (channel_cal.reverse_direction) {
                cdc_diff = -cdc_diff;  // 反转调整方向
                USB_LOG_DEBUG("AD7147: Stage %d using reverse direction, inverted diff=%d", stage, cdc_diff);
            }
            // 继续使用二分法逻辑
            [[fallthrough]];
            
        case CalibrationStrategy::BINARY_SEARCH:
        default:
            // 二分法调整
            if (cdc_diff > AD7147_AUTO_OFFSET_TOLERANCE) {
                // CDC值过高，需要增加正AFE偏移或减少负AFE偏移
                if (channel_cal.pos_afe_min < channel_cal.pos_afe_max) {
                    uint8_t new_pos_afe = (channel_cal.pos_afe_min + channel_cal.pos_afe_max) / 2;
                    current_config.afe_offset.bits.pos_afe_offset = new_pos_afe;
                    
                    if (cdc_diff > 0) {
                        channel_cal.pos_afe_min = new_pos_afe + 1;
                    } else {
                        channel_cal.pos_afe_max = new_pos_afe - 1;
                    }
                    adjustment_made = true;
                    USB_LOG_DEBUG("AD7147: Stage %d pos AFE=%d, range=[%d,%d]", 
                                 stage, new_pos_afe, channel_cal.pos_afe_min, channel_cal.pos_afe_max);
                } else if (channel_cal.neg_afe_min < channel_cal.neg_afe_max) {
                    uint8_t new_neg_afe = (channel_cal.neg_afe_min + channel_cal.neg_afe_max) / 2;
                    current_config.afe_offset.bits.neg_afe_offset = new_neg_afe;
                    
                    if (cdc_diff > 0) {
                        channel_cal.neg_afe_max = new_neg_afe - 1;
                    } else {
                        channel_cal.neg_afe_min = new_neg_afe + 1;
                    }
                    adjustment_made = true;
                    USB_LOG_DEBUG("AD7147: Stage %d neg AFE=%d, range=[%d,%d]", 
                                 stage, new_neg_afe, channel_cal.neg_afe_min, channel_cal.neg_afe_max);
                }
            } else if (cdc_diff < -AD7147_AUTO_OFFSET_TOLERANCE) {
                // CDC值过低，需要减少正AFE偏移或增加负AFE偏移
                if (channel_cal.pos_afe_min < channel_cal.pos_afe_max) {
                    uint8_t new_pos_afe = (channel_cal.pos_afe_min + channel_cal.pos_afe_max) / 2;
                    current_config.afe_offset.bits.pos_afe_offset = new_pos_afe;
                    
                    if (cdc_diff < 0) {
                        channel_cal.pos_afe_max = new_pos_afe - 1;
                    } else {
                        channel_cal.pos_afe_min = new_pos_afe + 1;
                    }
                    adjustment_made = true;
                    USB_LOG_DEBUG("AD7147: Stage %d pos AFE=%d, range=[%d,%d]", 
                                 stage, new_pos_afe, channel_cal.pos_afe_min, channel_cal.pos_afe_max);
                } else if (channel_cal.neg_afe_min < channel_cal.neg_afe_max) {
                    uint8_t new_neg_afe = (channel_cal.neg_afe_min + channel_cal.neg_afe_max) / 2;
                    current_config.afe_offset.bits.neg_afe_offset = new_neg_afe;
                    
                    if (cdc_diff < 0) {
                        channel_cal.neg_afe_min = new_neg_afe + 1;
                    } else {
                        channel_cal.neg_afe_max = new_neg_afe - 1;
                    }
                    adjustment_made = true;
                    USB_LOG_DEBUG("AD7147: Stage %d neg AFE=%d, range=[%d,%d]", 
                                 stage, new_neg_afe, channel_cal.neg_afe_min, channel_cal.neg_afe_max);
                }
            }
            break;
    }
    
    if (!adjustment_made) {
        USB_LOG_DEBUG("AD7147: Stage %d AFE adjustment failed, no more range", stage);
        return false;
    }
    
    return setStageConfigAsync(stage, current_config);
}

// 尝试交换AFE偏移
bool AD7147::trySwapAFEOffset(uint8_t stage, ChannelOffsetCalibration& channel_cal) {
    StageConfig current_config = getStageConfig(stage);
    bool changed = false;
    
    // 尝试交换正AFE偏移
    if (!channel_cal.pos_afe_swap_tried) {
        current_config.afe_offset.bits.pos_afe_offset_swap = !current_config.afe_offset.bits.pos_afe_offset_swap;
        channel_cal.pos_afe_swap_tried = true;
        changed = true;
        
        // 重置搜索范围
        channel_cal.pos_afe_min = 0;
        channel_cal.pos_afe_max = AD7147_AFE_OFFSET_MAX;
        
        USB_LOG_DEBUG("AD7147: Trying pos AFE swap for stage %d", stage);
    }
    // 尝试交换负AFE偏移
    else if (!channel_cal.neg_afe_swap_tried) {
        current_config.afe_offset.bits.neg_afe_offset_swap = !current_config.afe_offset.bits.neg_afe_offset_swap;
        channel_cal.neg_afe_swap_tried = true;
        changed = true;
        
        // 重置搜索范围
        channel_cal.neg_afe_min = 0;
        channel_cal.neg_afe_max = AD7147_AFE_OFFSET_MAX;
        
        USB_LOG_DEBUG("AD7147: Trying neg AFE swap for stage %d", stage);
    }
    
    if (changed) {
        // 先尝试极限偏移看是否能回到量程
        current_config.afe_offset.bits.pos_afe_offset = AD7147_AFE_OFFSET_MAX;
        current_config.afe_offset.bits.neg_afe_offset = AD7147_AFE_OFFSET_MAX;
        
        return setStageConfigAsync(stage, current_config);
    }
    
    return false;
}

// 重置通道校准状态
void AD7147::resetChannelCalibration(ChannelOffsetCalibration& channel_cal, uint8_t stage) {
    channel_cal.stage = stage;
    channel_cal.state = AutoOffsetState::IDLE;
    channel_cal.target_cdc = AD7147_CDC_BASELINE;
    channel_cal.current_cdc = 0;
    channel_cal.pos_afe_min = 0;
    channel_cal.pos_afe_max = AD7147_AFE_OFFSET_MAX;
    channel_cal.neg_afe_min = 0;
    channel_cal.neg_afe_max = AD7147_AFE_OFFSET_MAX;
    channel_cal.iteration_count = 0;
    channel_cal.pos_afe_swap_tried = false;
    channel_cal.neg_afe_swap_tried = false;
}

// 调整采样偏移
bool AD7147::adjustSampleOffset(uint8_t stage, ChannelOffsetCalibration& channel_cal) {
    StageConfig current_config = getStageConfig(stage);
    
    // 检查当前通道是否被触发
    if (checkChannelTriggered(stage)) {
        channel_cal.is_triggered = true;
        
        // 使用二分法调整低采样偏移
        if (channel_cal.sample_offset_low_min < channel_cal.sample_offset_low_max) {
            uint16_t mid = (channel_cal.sample_offset_low_min + channel_cal.sample_offset_low_max) / 2;
            current_config.offset_low = mid;
            
            // 更新搜索范围 - 增加偏移以降低灵敏度
            channel_cal.sample_offset_low_min = mid + 1;
            
            USB_LOG_DEBUG("AD7147: Adjusting sample offset low for stage %d to %d", stage, mid);
            return setStageConfigAsync(stage, current_config);
        }
        
        // 如果低偏移已达到最大值，尝试调整高偏移
        if (channel_cal.sample_offset_high_min < channel_cal.sample_offset_high_max) {
            uint16_t mid = (channel_cal.sample_offset_high_min + channel_cal.sample_offset_high_max) / 2;
            current_config.offset_high = mid;
            
            // 更新搜索范围 - 增加偏移以降低灵敏度
            channel_cal.sample_offset_high_min = mid + 1;
            
            USB_LOG_DEBUG("AD7147: Adjusting sample offset high for stage %d to %d", stage, mid);
            return setStageConfigAsync(stage, current_config);
        }
        
        // 采样偏移调整完成但仍被触发，进入灵敏度调整阶段
        channel_cal.state = AutoOffsetState::ADJUSTING_SENSITIVITY;
        return true;
    } else {
        // 通道未被触发，采样偏移调整成功
        channel_cal.is_triggered = false;
        channel_cal.state = AutoOffsetState::COMPLETED;
        USB_LOG_DEBUG("AD7147: Sample offset calibration completed successfully for stage %d", stage);
        return true;
    }
}

// 调整灵敏度 - 使用50周期极值范围采样
bool AD7147::adjustSensitivity(uint8_t stage, ChannelOffsetCalibration& channel_cal) {
    StageConfig current_config = getStageConfig(stage);
    
    // 读取当前CDC值
    uint16_t current_cdc;
    if (!readStageCDC(stage, current_cdc)) {
        return false;
    }
    
    // 初始化采样或继续采样
    if (channel_cal.sample_count == 0) {
        // 开始新的采样周期
        channel_cal.sample_max_cdc = current_cdc;
        channel_cal.sample_min_cdc = current_cdc;
        channel_cal.sample_count = 1;
        return true;
    } else if (channel_cal.sample_count < 50) {
        // 继续采样，更新极值
        if (current_cdc > channel_cal.sample_max_cdc) {
            channel_cal.sample_max_cdc = current_cdc;
        }
        if (current_cdc < channel_cal.sample_min_cdc) {
            channel_cal.sample_min_cdc = current_cdc;
        }
        channel_cal.sample_count++;
        return true;
    }
    
    // 完成50个周期的采样，计算范围
    channel_cal.sample_range = channel_cal.sample_max_cdc - channel_cal.sample_min_cdc;
    
    // 基于极值范围判断调整方向
    if (channel_cal.sample_range > AD7147_AUTO_OFFSET_TOLERANCE) {
        // 范围过大，需要降低灵敏度
        if (channel_cal.sensitivity_min < channel_cal.sensitivity_max) {
            uint16_t mid = (channel_cal.sensitivity_min + channel_cal.sensitivity_max) / 2;
            current_config.sensitivity = mid;
            channel_cal.sensitivity_min = mid + 1;
            
            // 重置采样计数器开始新的采样周期
            channel_cal.sample_count = 0;
            return setStageConfigAsync(stage, current_config);
        } else {
            // 灵敏度已调整到最低，校准失败
            channel_cal.state = AutoOffsetState::CALIBRATION_ERROR;
            return false;
        }
    } else {
        // 范围合适，灵敏度调整成功
        channel_cal.state = AutoOffsetState::COMPLETED;
        return true;
    }
}

// 检查通道是否被触发
bool AD7147::checkChannelTriggered(uint8_t stage) {
    // 使用sample固定采样的触摸状态，避免重复采样占用带宽
    // 从最近的sample结果中获取触摸状态
    TouchSampleResult sample_result = sample();
    
    // 检查指定阶段是否被触发
    return (sample_result.channel_mask & (1 << stage)) != 0;
}
