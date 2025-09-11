#include "communication_settings.h"
#include "../../ui_manager.h"
#include "../../../input_manager/input_manager.h"
#include "../../../config_manager/config_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/graphics_rendering/graphics_engine.h"

namespace ui {

// 波特率预设值数组 - 使用统一标准
const uint32_t CommunicationSettings::BAUD_RATES[] = {
    9600, 115200, 250000, 500000, 1000000, 1500000,
    2000000, 2500000, 3000000, 4000000, 5000000, 6000000
};
const size_t CommunicationSettings::BAUD_RATES_COUNT = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

// 静态成员变量定义
uint32_t CommunicationSettings::current_mai2serial_baud_ = 115200;
uint32_t CommunicationSettings::current_lightmanager_baud_ = 115200;
uint8_t CommunicationSettings::current_serial_delay_ = 0;
bool CommunicationSettings::current_keyboard_mapping_enabled_ = false;
size_t CommunicationSettings::mai2serial_baud_index_ = 4; // 默认115200
size_t CommunicationSettings::lightmanager_baud_index_ = 4; // 默认115200

CommunicationSettings::CommunicationSettings() {
    loadCurrentSettings();
}

void CommunicationSettings::render(PageTemplate& page_template) {
    PAGE_WITH_TITLE("通信设置", COLOR_TEXT_WHITE)
    
    // 返回项
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)

    // Mai2Serial波特率设置
    static char mai2serial_baud_text[64];
    snprintf(mai2serial_baud_text, sizeof(mai2serial_baud_text), "Serial波特率: %s", 
             formatBaudRateText(current_mai2serial_baud_).c_str());
    ADD_SIMPLE_SELECTOR(mai2serial_baud_text, onMai2SerialBaudRateChange, COLOR_TEXT_YELLOW)
    
    // LightManager波特率设置
    static char lightmanager_baud_text[64];
    snprintf(lightmanager_baud_text, sizeof(lightmanager_baud_text), "Light波特率: %s", 
             formatBaudRateText(current_lightmanager_baud_).c_str());
    ADD_SIMPLE_SELECTOR(lightmanager_baud_text, onLightManagerBaudRateChange, COLOR_TEXT_YELLOW)
    
    // 串口模式延迟设置
    static char serial_delay_text[64];
    snprintf(serial_delay_text, sizeof(serial_delay_text), "串口延迟: %s", 
             formatDelayText(current_serial_delay_).c_str());
    ADD_SIMPLE_SELECTOR(serial_delay_text, onSerialDelayChange, COLOR_TEXT_WHITE)
    
    // 串口模式通道映射键盘开关
    static char keyboard_mapping_text[64];
    snprintf(keyboard_mapping_text, sizeof(keyboard_mapping_text), "映射键盘: %s", 
             formatKeyboardMappingText(current_keyboard_mapping_enabled_).c_str());
    ADD_SIMPLE_SELECTOR(keyboard_mapping_text, onKeyboardMappingToggle, COLOR_TEXT_WHITE)
    
    // 键盘设置子菜单
    ADD_MENU("键盘设置", "keyboard_settings", COLOR_TEXT_GRAY)
    
    PAGE_END()
}

void CommunicationSettings::onMai2SerialBaudRateChange(JoystickState state) {
    if (state == JoystickState::UP) {
        if (mai2serial_baud_index_ < BAUD_RATES_COUNT - 1) {
            mai2serial_baud_index_++;
            current_mai2serial_baud_ = BAUD_RATES[mai2serial_baud_index_];
            saveSettings();
        }
    } else if (state == JoystickState::DOWN) {
        if (mai2serial_baud_index_ > 0) {
            mai2serial_baud_index_--;
            current_mai2serial_baud_ = BAUD_RATES[mai2serial_baud_index_];
            saveSettings();
        }
    }
}

void CommunicationSettings::onLightManagerBaudRateChange(JoystickState state) {
    if (state == JoystickState::UP) {
        if (lightmanager_baud_index_ < BAUD_RATES_COUNT - 1) {
            lightmanager_baud_index_++;
            current_lightmanager_baud_ = BAUD_RATES[lightmanager_baud_index_];
            saveSettings();
        }
    } else if (state == JoystickState::DOWN) {
        if (lightmanager_baud_index_ > 0) {
            lightmanager_baud_index_--;
            current_lightmanager_baud_ = BAUD_RATES[lightmanager_baud_index_];
            saveSettings();
        }
    }
}

void CommunicationSettings::onSerialDelayChange(JoystickState state) {
    if (state == JoystickState::UP) {
        if (current_serial_delay_ < 100) {
            current_serial_delay_++;
            saveSettings();
        }
    } else if (state == JoystickState::DOWN) {
        if (current_serial_delay_ > 0) {
            current_serial_delay_--;
            saveSettings();
        }
    }
}

void CommunicationSettings::onKeyboardMappingToggle(JoystickState state) {
    if (state == JoystickState::UP || state == JoystickState::DOWN) {
        current_keyboard_mapping_enabled_ = !current_keyboard_mapping_enabled_;
        saveSettings();
    }
}

void CommunicationSettings::onKeyboardSettingsMenu() {
    // 跳转到键盘设置页面
    UIManager* ui_mgr = UIManager::getInstance();
    if (ui_mgr) {
        ui_mgr->switch_to_page("keyboard_settings");
    }
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
    
    // 从ConfigManager获取LightManager波特率设置
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (config_mgr) {
        // LightManager波特率
        current_lightmanager_baud_ = config_mgr->get_uint32(LIGHTMANAGER_BAUD_RATE);
    }
    
    // 更新波特率索引
    mai2serial_baud_index_ = findBaudRateIndex(current_mai2serial_baud_);
    lightmanager_baud_index_ = findBaudRateIndex(current_lightmanager_baud_);
}

void CommunicationSettings::saveSettings() {
    // 保存到InputManager
    InputManager* input_mgr = InputManager::getInstance();
    if (input_mgr) {
        input_mgr->setTouchResponseDelay(current_serial_delay_);
        input_mgr->setTouchKeyboardEnabled(current_keyboard_mapping_enabled_);
    }
    
    // 保存到ConfigManager
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (config_mgr) {
        // 保存LightManager波特率
        config_mgr->set_uint32(LIGHTMANAGER_BAUD_RATE, current_lightmanager_baud_);
        
        // 应用LightManager波特率设置
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
}

size_t CommunicationSettings::findBaudRateIndex(uint32_t baud_rate) {
    for (size_t i = 0; i < BAUD_RATES_COUNT; i++) {
        if (BAUD_RATES[i] == baud_rate) {
            return i;
        }
    }
    return 4; // 默认返回115200的索引
}

std::string CommunicationSettings::formatBaudRateText(uint32_t baud_rate) {
    if (baud_rate >= 1000000) {
        return std::to_string(baud_rate / 1000000) + "M";
    } else if (baud_rate >= 1000) {
        return std::to_string(baud_rate / 1000) + "K";
    } else {
        return std::to_string(baud_rate);
    }
}

std::string CommunicationSettings::formatDelayText(uint8_t delay_ms) {
    return std::to_string(delay_ms) + "ms";
}

std::string CommunicationSettings::formatKeyboardMappingText(bool enabled) {
    return enabled ? "开启" : "关闭";
}

} // namespace ui