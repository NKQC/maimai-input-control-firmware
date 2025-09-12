#include "communication_settings.h"
#include "../../ui_manager.h"
#include "../../../input_manager/input_manager.h"
#include "../../../config_manager/config_manager.h"
#include "../../../light_manager/light_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/graphics_rendering/graphics_engine.h"

namespace ui {

// 使用HAL层提供的波特率预设

// 静态成员变量定义
uint32_t CommunicationSettings::current_mai2serial_baud_ = 115200;
uint32_t CommunicationSettings::current_lightmanager_baud_ = 115200;
uint8_t CommunicationSettings::current_serial_delay_ = 0;
bool CommunicationSettings::current_keyboard_mapping_enabled_ = false;
size_t CommunicationSettings::mai2serial_baud_index_ = 1; // 默认115200（HAL预设索引1）
size_t CommunicationSettings::lightmanager_baud_index_ = 1; // 默认115200（HAL预设索引1）

CommunicationSettings::CommunicationSettings() {
}

void CommunicationSettings::render(PageTemplate& page_template) {
    PAGE_WITH_TITLE("通信设置", COLOR_TEXT_WHITE)
    loadCurrentSettings();
    // 返回项
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    static char _text[64];
    // Mai2Serial波特率设置
    snprintf(_text, sizeof(_text), "Serial波特率: %s", 
             formatBaudRateText(current_mai2serial_baud_).c_str());
    ADD_SIMPLE_SELECTOR(_text, onMai2SerialBaudRateChange, COLOR_TEXT_YELLOW)
    
    // LightManager波特率设置
    snprintf(_text, sizeof(_text), "Light波特率: %s", 
             formatBaudRateText(current_lightmanager_baud_).c_str());
    ADD_SIMPLE_SELECTOR(_text, onLightManagerBaudRateChange, COLOR_TEXT_YELLOW)
    
    // 串口模式延迟设置
    snprintf(_text, sizeof(_text), "串口延迟: %s", 
             formatDelayText(current_serial_delay_).c_str());
    ADD_SIMPLE_SELECTOR(_text, onSerialDelayChange, COLOR_TEXT_WHITE)
    
    // 串口模式通道映射键盘开关
    snprintf(_text, sizeof(_text), "映射键盘: %s", 
             formatKeyboardMappingText(current_keyboard_mapping_enabled_).c_str());
    ADD_BUTTON(_text, onKeyboardMappingToggle, COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    PAGE_END()
}

void CommunicationSettings::onMai2SerialBaudRateChange(JoystickState state) {
    if (state == JoystickState::UP) {
        if (mai2serial_baud_index_ < get_supported_baud_rates_count() - 1) {
            mai2serial_baud_index_++;
            current_mai2serial_baud_ = get_supported_baud_rates()[mai2serial_baud_index_];
        }
    } else if (state == JoystickState::DOWN) {
        if (mai2serial_baud_index_ > 0) {
            mai2serial_baud_index_--;
            current_mai2serial_baud_ = get_supported_baud_rates()[mai2serial_baud_index_];
        }
    }
    ApplySettings();
}

void CommunicationSettings::onLightManagerBaudRateChange(JoystickState state) {
    if (state == JoystickState::UP) {
        if (lightmanager_baud_index_ < get_supported_baud_rates_count() - 1) {
            lightmanager_baud_index_++;
            current_lightmanager_baud_ = get_supported_baud_rates()[lightmanager_baud_index_];
        }
    } else if (state == JoystickState::DOWN) {
        if (lightmanager_baud_index_ > 0) {
            lightmanager_baud_index_--;
            current_lightmanager_baud_ = get_supported_baud_rates()[lightmanager_baud_index_];
        }
    }
    ApplySettings();
}

void CommunicationSettings::onSerialDelayChange(JoystickState state) {
    if (state == JoystickState::UP) {
        if (current_serial_delay_ < 100) {
            current_serial_delay_++;
        }
    } else if (state == JoystickState::DOWN) {
        if (current_serial_delay_ > 0) {
            current_serial_delay_--;
        }
    }
    ApplySettings();
}

void CommunicationSettings::onKeyboardMappingToggle() {
    current_keyboard_mapping_enabled_ = !current_keyboard_mapping_enabled_;
    ApplySettings();
}

void CommunicationSettings::loadCurrentSettings() {
    // 从InputManager获取当前设置
    InputManager* input_mgr = InputManager::getInstance();
    if (input_mgr) {
        current_serial_delay_ = input_mgr->getTouchResponseDelay();
        current_keyboard_mapping_enabled_ = input_mgr->getTouchKeyboardEnabled();
    }
    
    // 从InputManager获取Mai2Serial真实波特率
    if (input_mgr) {
        Mai2Serial_Config mai2serial_config = input_mgr->getMai2SerialConfig();
        current_mai2serial_baud_ = mai2serial_config.baud_rate;
    }
    
    // 从LightManager获取波特率设置（只读接口）
    LightManager_PrivateConfig light_config = lightmanager_get_config_copy();
    current_lightmanager_baud_ = light_config.baud_rate;
    
    // 更新波特率索引
    mai2serial_baud_index_ = findBaudRateIndex(current_mai2serial_baud_);
    lightmanager_baud_index_ = findBaudRateIndex(current_lightmanager_baud_);
}

void CommunicationSettings::ApplySettings() {
    // 保存到InputManager
    InputManager* input_mgr = InputManager::getInstance();
    if (input_mgr) {
        input_mgr->setTouchResponseDelay(current_serial_delay_);
        input_mgr->setTouchKeyboardEnabled(current_keyboard_mapping_enabled_);
    }
    
    LightManager* light_mgr = LightManager::getInstance();
    if (light_mgr) {
        Mai2Light_Config light_config;
        light_config.baud_rate = current_lightmanager_baud_;
        light_mgr->update_mai2light_config(light_config);
    }
    
    // 应用Mai2Serial波特率设置
    if (input_mgr) {
        Mai2Serial_Config mai2serial_config = input_mgr->getMai2SerialConfig();
        mai2serial_config.baud_rate = current_mai2serial_baud_;
        input_mgr->setMai2SerialConfig(mai2serial_config);
    }
}

size_t CommunicationSettings::findBaudRateIndex(uint32_t baud_rate) {
    const uint32_t* supported_rates = get_supported_baud_rates();
    size_t count = get_supported_baud_rates_count();
    for (size_t i = 0; i < count; i++) {
        if (supported_rates[i] == baud_rate) {
            return i;
        }
    }
    return 1; // 默认返回115200的索引（在HAL预设中是索引1）
}

std::string CommunicationSettings::formatBaudRateText(uint32_t baud_rate) {
    char buffer[16];
    
    if (baud_rate >= 1000000) {
        // 处理兆位级波特率，使用整数运算避免浮点
        uint32_t mbaud_int = baud_rate / 1000000;
        uint32_t mbaud_frac = (baud_rate % 1000000) / 100000; // 取一位小数
        
        if (baud_rate % 1000000 == 0) {
            // 整数兆位，如1M, 2M, 3M等
            snprintf(buffer, sizeof(buffer), "%luM", (unsigned long)mbaud_int);
        } else {
            // 小数兆位，如1.5M, 2.5M等
            snprintf(buffer, sizeof(buffer), "%lu.%luM", (unsigned long)mbaud_int, (unsigned long)mbaud_frac);
        }
    } else if (baud_rate >= 1000) {
        // 处理千位级波特率，使用整数运算
        uint32_t kbaud_int = baud_rate / 1000;
        uint32_t kbaud_frac = (baud_rate % 1000) / 100; // 取一位小数
        
        if (baud_rate % 1000 == 0) {
            snprintf(buffer, sizeof(buffer), "%luK", (unsigned long)kbaud_int);
        } else {
            snprintf(buffer, sizeof(buffer), "%lu.%luK", (unsigned long)kbaud_int, (unsigned long)kbaud_frac);
        }
    } else {
        snprintf(buffer, sizeof(buffer), "%lu", (unsigned long)baud_rate);
    }
    
    return std::string(buffer);
}

std::string CommunicationSettings::formatDelayText(uint8_t delay_ms) {
    return std::to_string(delay_ms) + "ms";
}

std::string CommunicationSettings::formatKeyboardMappingText(bool enabled) {
    return enabled ? "开启" : "关闭";
}

} // namespace ui