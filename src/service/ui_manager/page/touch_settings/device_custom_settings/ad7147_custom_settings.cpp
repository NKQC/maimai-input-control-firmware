#include "ad7147_custom_settings.h"
#include "src/service/ui_manager/ui_manager.h"
#include "src/service/ui_manager/engine/page_construction/page_macros.h"
#include "src/service/ui_manager/engine/page_construction/page_template.h"
#include "src/service/input_manager/input_manager.h"
#include <cstdio>

namespace ui {

// 静态成员变量定义
std::string AD7147CustomSettings::device_name_;
int32_t AD7147CustomSettings::current_stage_ = 0;
PortConfig AD7147CustomSettings::current_config_;
BitfieldHelper AD7147CustomSettings::bitfield_helper_;
uint16_t AD7147CustomSettings::current_cdc_value_ = 0;
bool AD7147CustomSettings::config_loaded_ = false;
bool AD7147CustomSettings::channel_triggered_ = false;

// 一键拉偏移功能相关
bool AD7147CustomSettings::auto_offset_active_ = false;
uint8_t AD7147CustomSettings::auto_offset_total_progress_ = 0;

// BitfieldHelper方法实现
void BitfieldHelper::loadFromPortConfig(const PortConfig& config) {
    // 加载AFE偏移位域
    afe_offset.neg_afe_offset = config.afe_offset.bits.neg_afe_offset;
    afe_offset.neg_afe_swap = config.afe_offset.bits.neg_afe_offset_swap;
    afe_offset.pos_afe_offset = config.afe_offset.bits.pos_afe_offset;
    afe_offset.pos_afe_swap = config.afe_offset.bits.pos_afe_offset_swap;
    
    // 加载灵敏度位域
    sensitivity.neg_threshold_sensitivity = config.sensitivity.bits.neg_threshold_sensitivity;
    sensitivity.neg_peak_detect = config.sensitivity.bits.neg_peak_detect;
    sensitivity.pos_threshold_sensitivity = config.sensitivity.bits.pos_threshold_sensitivity;
    sensitivity.pos_peak_detect = config.sensitivity.bits.pos_peak_detect;
}

void BitfieldHelper::writeToPortConfig(PortConfig& config) const {
    // 写回AFE偏移位域
    config.afe_offset.bits.neg_afe_offset = afe_offset.neg_afe_offset;
    config.afe_offset.bits.neg_afe_offset_swap = afe_offset.neg_afe_swap;
    config.afe_offset.bits.pos_afe_offset = afe_offset.pos_afe_offset;
    config.afe_offset.bits.pos_afe_offset_swap = afe_offset.pos_afe_swap;
    
    // 写回灵敏度位域
    config.sensitivity.bits.neg_threshold_sensitivity = sensitivity.neg_threshold_sensitivity;
    config.sensitivity.bits.neg_peak_detect = sensitivity.neg_peak_detect;
    config.sensitivity.bits.pos_threshold_sensitivity = sensitivity.pos_threshold_sensitivity;
    config.sensitivity.bits.pos_peak_detect = sensitivity.pos_peak_detect;
}

AD7147CustomSettings::AD7147CustomSettings() {
    // 构造函数无需特殊初始化
}

void AD7147CustomSettings::render(PageTemplate& page_template) {
    // 获取AD7147设备实例
    AD7147* ad7147 = getAD7147Device();
    if (!ad7147) {
        PAGE_START()
        SET_TITLE("AD7147 " + device_name_ + " 灵敏度设置", COLOR_WHITE)
        ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
        ADD_TEXT("设备未找到或未初始化", COLOR_RED, LineAlign::CENTER)
        PAGE_END()
        return;
    }
    
    // 加载当前阶段配置和状态数据
    loadStageDataAndStatus();
    
    // 构建标题，包含CDC值和触发状态
    // CDC显示逻辑：使用AD7147_CDC_BASELINE为0值基准，低于此值显示负值，高于此值显示正值
    static char text[32];
    int32_t cdc_display_value = static_cast<int32_t>(current_cdc_value_) - AD7147_CDC_BASELINE;
    snprintf(text, sizeof(text), "CDC:%ld [%s]", cdc_display_value, channel_triggered_ ? "1" : "0");
    
    PAGE_START()
    SET_TITLE(text, COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    // 一键拉偏移按钮
    updateAutoOffsetStatus();
    if (auto_offset_active_) {
        ADD_PROGRESS(&auto_offset_total_progress_, COLOR_TEXT_WHITE)
    } else {
        ADD_BUTTON("一键调整", onAutoOffsetButtonClick, COLOR_TEXT_YELLOW, LineAlign::CENTER)
    }
    
    // 阶段选择
    static char stage_text[32];
    snprintf(stage_text, sizeof(stage_text), "阶段选择: %ld", current_stage_);
    ADD_SIMPLE_SELECTOR(stage_text, [](JoystickState state) {
        if (state == JoystickState::UP && current_stage_ < 11) {
            current_stage_++;
            config_loaded_ = false;
            onStageChange();
        } else if (state == JoystickState::DOWN && current_stage_ > 0) {
            current_stage_--;
            config_loaded_ = false;
            onStageChange();
        }
    }, COLOR_TEXT_YELLOW)

    // AFE偏移位域配置
    static char neg_afe_text[32];
    snprintf(neg_afe_text, sizeof(neg_afe_text), "负AFE偏移: %ld", bitfield_helper_.afe_offset.neg_afe_offset);
    ADD_SIMPLE_SELECTOR(neg_afe_text, [](JoystickState state) {
        if (state == JoystickState::UP && bitfield_helper_.afe_offset.neg_afe_offset < 63) {
            bitfield_helper_.afe_offset.neg_afe_offset++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && bitfield_helper_.afe_offset.neg_afe_offset > 0) {
            bitfield_helper_.afe_offset.neg_afe_offset--;
            onConfigComplete();
        }
    }, COLOR_TEXT_YELLOW)
    
    // 负AFE交换按钮
    std::string neg_afe_swap_text = "负AFE交换: " + std::string(bitfield_helper_.afe_offset.neg_afe_swap ? "启用" : "禁用");
    ADD_BUTTON(neg_afe_swap_text, []() {
        bitfield_helper_.afe_offset.neg_afe_swap = !bitfield_helper_.afe_offset.neg_afe_swap;
        onConfigComplete();
    }, bitfield_helper_.afe_offset.neg_afe_swap ? COLOR_TEXT_GREEN : COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    static char pos_afe_text[32];
    snprintf(pos_afe_text, sizeof(pos_afe_text), "正AFE偏移: %ld", bitfield_helper_.afe_offset.pos_afe_offset);
    ADD_SIMPLE_SELECTOR(pos_afe_text, [](JoystickState state) {
        if (state == JoystickState::UP && bitfield_helper_.afe_offset.pos_afe_offset < 63) {
            bitfield_helper_.afe_offset.pos_afe_offset++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && bitfield_helper_.afe_offset.pos_afe_offset > 0) {
            bitfield_helper_.afe_offset.pos_afe_offset--;
            onConfigComplete();
        }
    }, COLOR_TEXT_YELLOW)
    
    // 正AFE交换按钮
    std::string pos_afe_swap_text = "正AFE交换: " + std::string(bitfield_helper_.afe_offset.pos_afe_swap ? "启用" : "禁用");
    ADD_BUTTON(pos_afe_swap_text, []() {
        bitfield_helper_.afe_offset.pos_afe_swap = !bitfield_helper_.afe_offset.pos_afe_swap;
        onConfigComplete();
    }, bitfield_helper_.afe_offset.pos_afe_swap ? COLOR_TEXT_GREEN : COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // 灵敏度位域配置
    static char neg_thresh_text[32];
    snprintf(neg_thresh_text, sizeof(neg_thresh_text), "负灵敏度: %ld", bitfield_helper_.sensitivity.neg_threshold_sensitivity);
    ADD_SIMPLE_SELECTOR(neg_thresh_text, [](JoystickState state) {
        if (state == JoystickState::UP && bitfield_helper_.sensitivity.neg_threshold_sensitivity < 15) {
            bitfield_helper_.sensitivity.neg_threshold_sensitivity++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && bitfield_helper_.sensitivity.neg_threshold_sensitivity > 0) {
            bitfield_helper_.sensitivity.neg_threshold_sensitivity--;
            onConfigComplete();
        }
    }, COLOR_TEXT_YELLOW)
    
    static char neg_peak_text[32];
    snprintf(neg_peak_text, sizeof(neg_peak_text), "负峰值: %ld", bitfield_helper_.sensitivity.neg_peak_detect);
    ADD_SIMPLE_SELECTOR(neg_peak_text, [](JoystickState state) {
        if (state == JoystickState::UP && bitfield_helper_.sensitivity.neg_peak_detect < 5) {
            bitfield_helper_.sensitivity.neg_peak_detect++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && bitfield_helper_.sensitivity.neg_peak_detect > 0) {
            bitfield_helper_.sensitivity.neg_peak_detect--;
            onConfigComplete();
        }
    }, COLOR_TEXT_YELLOW)
    
    static char pos_thresh_text[32];
    snprintf(pos_thresh_text, sizeof(pos_thresh_text), "正灵敏度: %ld", bitfield_helper_.sensitivity.pos_threshold_sensitivity);
    ADD_SIMPLE_SELECTOR(pos_thresh_text, [](JoystickState state) {
        if (state == JoystickState::UP && bitfield_helper_.sensitivity.pos_threshold_sensitivity < 15) {
            bitfield_helper_.sensitivity.pos_threshold_sensitivity++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && bitfield_helper_.sensitivity.pos_threshold_sensitivity > 0) {
            bitfield_helper_.sensitivity.pos_threshold_sensitivity--;
            onConfigComplete();
        }
    }, COLOR_TEXT_YELLOW)
    
    static char pos_peak_text[32];
    snprintf(pos_peak_text, sizeof(pos_peak_text), "正峰值: %ld", bitfield_helper_.sensitivity.pos_peak_detect);
    ADD_SIMPLE_SELECTOR(pos_peak_text, [](JoystickState state) {
        if (state == JoystickState::UP && bitfield_helper_.sensitivity.pos_peak_detect < 5) {
            bitfield_helper_.sensitivity.pos_peak_detect++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && bitfield_helper_.sensitivity.pos_peak_detect > 0) {
            bitfield_helper_.sensitivity.pos_peak_detect--;
            onConfigComplete();
        }
    }, COLOR_TEXT_YELLOW)
    
    // 偏移设置
    static char offset_low_text[32];
    snprintf(offset_low_text, sizeof(offset_low_text), "低偏移: %u", current_config_.offset_low);
    ADD_SIMPLE_SELECTOR(offset_low_text, [](JoystickState state) {
        if (state == JoystickState::UP && current_config_.offset_low < 65535) {
            current_config_.offset_low++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && current_config_.offset_low > 0) {
            current_config_.offset_low--;
            onConfigComplete();
        }
    }, COLOR_TEXT_WHITE)
    
    static char offset_high_text[32];
    snprintf(offset_high_text, sizeof(offset_high_text), "高偏移: %d", current_config_.offset_high);
    ADD_SIMPLE_SELECTOR(offset_high_text, [](JoystickState state) {
        if (state == JoystickState::UP && current_config_.offset_high < 65535) {
            current_config_.offset_high++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && current_config_.offset_high > 0) {
            current_config_.offset_high--;
            onConfigComplete();
        }
    }, COLOR_TEXT_WHITE)
    
    // 钳位设置
    static char offset_high_clamp_text[32];
    snprintf(offset_high_clamp_text, sizeof(offset_high_clamp_text), "高偏移钳位: %u", current_config_.offset_high_clamp);
    ADD_SIMPLE_SELECTOR(offset_high_clamp_text, [](JoystickState state) {
        if (state == JoystickState::UP && current_config_.offset_high_clamp < 65535) {
            current_config_.offset_high_clamp++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && current_config_.offset_high_clamp > 0) {
            current_config_.offset_high_clamp--;
            onConfigComplete();
        }
    }, COLOR_TEXT_WHITE)
    
    static char offset_low_clamp_text[32];
    snprintf(offset_low_clamp_text, sizeof(offset_low_clamp_text), "低偏移钳位: %u", current_config_.offset_low_clamp);
    ADD_SIMPLE_SELECTOR(offset_low_clamp_text, [](JoystickState state) {
        if (state == JoystickState::UP && current_config_.offset_low_clamp < 65535) {
            current_config_.offset_low_clamp++;
            onConfigComplete();
        } else if (state == JoystickState::DOWN && current_config_.offset_low_clamp > 0) {
            current_config_.offset_low_clamp--;
            onConfigComplete();
        }
    }, COLOR_TEXT_WHITE)
    
    // 连接配置
    // snprintf(text, sizeof(text), "连接6-0: 0x%04X", current_config_.connection_6_0);
    // ADD_TEXT(text, COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // snprintf(text, sizeof(text), "连接12-7: 0x%04X", current_config_.connection_12_7);
    // ADD_TEXT(text, COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // 容易误触 暂时关掉 
    /* 
     * TODO: 添加确认页面
     */
    // // 重置当前阶段配置按钮
    // ADD_BUTTON("重置当前阶段", []() {
    //     resetCurrentStageFromDevice();
    // }, COLOR_TEXT_YELLOW, LineAlign::CENTER)
    
    // // 重置设置按钮
    // ADD_BUTTON("重置设置", []() {
    //     resetToDefault();
    // }, COLOR_ERROR, LineAlign::CENTER)

    PAGE_END()
}

void AD7147CustomSettings::jump_str(const std::string& str) {
    device_name_ = str;
    current_stage_ = 0;
    config_loaded_ = false;
    current_cdc_value_ = 0;
    channel_triggered_ = false;
    // 重置位域辅助结构体
    bitfield_helper_ = BitfieldHelper();
}

AD7147* AD7147CustomSettings::getAD7147Device() {
    // 检查设备名称是否有效
    if (device_name_.empty()) {
        return nullptr;
    }
    
    // 获取InputManager实例
    InputManager* input_mgr = InputManager::getInstance();
    if (!input_mgr) {
        return nullptr;
    }
    // 通过设备名称获取TouchSensor实例
    TouchSensor* touch_sensor = input_mgr->getTouchSensorByDeviceName(device_name_);
    if (!touch_sensor) {
        return nullptr;
    }
    
    // 将TouchSensor转换为AD7147实例
    AD7147* ad7147 = static_cast<AD7147*>(touch_sensor);
    
    // 验证转换是否成功（检查设备是否确实是AD7147）
    if (ad7147 && ad7147->isInitialized()) {
        return ad7147;
    }
    
    return nullptr;
}

void AD7147CustomSettings::loadStageDataAndStatus() {
    // 检查stage范围是否有效（AD7147支持0-11个stage）
    if (current_stage_ < 0 || current_stage_ > 11) {
        current_cdc_value_ = 0;
        channel_triggered_ = false;
        return;
    }
    
    AD7147* ad7147 = getAD7147Device();
    if (!ad7147) {
        current_cdc_value_ = 0;
        channel_triggered_ = false;
        return;
    }
    
    // 加载配置数据（仅在需要时）
    if (!config_loaded_) {
        current_config_ = ad7147->getStageConfig(current_stage_);
        bitfield_helper_.loadFromPortConfig(current_config_);
        config_loaded_ = true;
    }
    
    // 读取CDC值
    uint16_t cdc_value;
    if (ad7147->readStageCDC(current_stage_, cdc_value)) {
        current_cdc_value_ = cdc_value;
    } else {
        current_cdc_value_ = 0;
    }
    
    // 获取触摸状态（一次性获取，避免重复调用）
    InputManager* input_manager = InputManager::getInstance();
    channel_triggered_ = false;
    if (input_manager) {
        int device_count = input_manager->get_device_count();
        if (device_count > 0 && device_count <= 32) { // 限制最大设备数量，防止栈溢出
            InputManager::TouchDeviceStatus device_status[32]; // 使用固定大小数组
            input_manager->get_all_device_status(device_status);
            
            uint8_t ad7147_mask = ad7147->getModuleMask();
            for (int i = 0; i < device_count; i++) {
                if (device_status[i].touch_device.device_id_mask == ad7147_mask && 
                    device_status[i].is_connected) {
                    // 确保stage位移不会超出32位范围
                    if (current_stage_ < 32) {
                        channel_triggered_ = (device_status[i].touch_states_32bit & (1u << current_stage_)) != 0;
                    }
                    break;
                }
            }
        }
    }
}

void AD7147CustomSettings::applyConfig() {
    // 检查stage范围是否有效
    if (current_stage_ < 0 || current_stage_ > 11) {
        return;
    }
    
    AD7147* ad7147 = getAD7147Device();
    if (ad7147) {
        // 将位域配置写回到PortConfig
        bitfield_helper_.writeToPortConfig(current_config_);
        // 应用配置到硬件
        ad7147->setStageConfigAsync(current_stage_, current_config_);
    }
}

// 静态回调函数实现
void AD7147CustomSettings::onStageChange() {
    config_loaded_ = false; // 强制重新加载配置
    loadStageDataAndStatus(); // 立即加载新阶段的数据
}

void AD7147CustomSettings::onConfigComplete() {
    applyConfig();
}

void AD7147CustomSettings::resetToDefault() {
    // 检查stage范围是否有效
    if (current_stage_ < 0 || current_stage_ > 11) {
        return;
    }
    
    AD7147* ad7147 = getAD7147Device();
    if (ad7147) {
        // 创建默认的StageSettings以获取默认连接配置
        StageSettings default_settings;
        
        // 从默认设置中获取当前stage的配置
        PortConfig default_config = default_settings.stages[current_stage_];
        
        // 应用默认配置到当前配置
        current_config_ = default_config;
        
        // 从默认配置加载位域辅助结构
        bitfield_helper_.loadFromPortConfig(current_config_);
        
        // 应用配置到硬件
        applyConfig();
        
        // 标记配置已加载
        config_loaded_ = true;
    }
}

void AD7147CustomSettings::resetCurrentStageFromDevice() {
    // 检查stage范围是否有效
    if (current_stage_ < 0 || current_stage_ > 11) {
        return;
    }
    
    AD7147* ad7147 = getAD7147Device();
    if (ad7147) {
        // 从AD7147设备读取当前阶段的配置
        current_config_ = ad7147->getStageConfig(current_stage_);
        
        // 从当前配置加载位域辅助结构
        bitfield_helper_.loadFromPortConfig(current_config_);
        
        // 标记配置已加载
        config_loaded_ = true;
        
        // 重新应用配置确保同步
        applyConfig();
    }
}

void AD7147CustomSettings::startAutoOffsetCalibration() {
    AD7147* ad7147 = getAD7147Device();
    if (!ad7147) {
        return;
    }
    
    // 启动自动偏移校准
    if (ad7147->startAutoOffsetCalibration()) {
        auto_offset_active_ = true;
        auto_offset_total_progress_ = 0;
    }
}

void AD7147CustomSettings::updateAutoOffsetStatus() {
    AD7147* ad7147 = getAD7147Device();
    if (!ad7147) {
        auto_offset_active_ = false;
        auto_offset_total_progress_ = 0;
        return;
    }
    // 检查校准是否仍在进行
    auto_offset_active_ = ad7147->isAutoOffsetCalibrationActive();

    if (auto_offset_active_) {
        auto_offset_total_progress_ = ad7147->getAutoOffsetCalibrationTotalProgress();
    } else {
        // 重新加载配置以反映校准结果
        config_loaded_ = false;
    }
}

void AD7147CustomSettings::onAutoOffsetButtonClick() {
    startAutoOffsetCalibration();
}

} // namespace ui