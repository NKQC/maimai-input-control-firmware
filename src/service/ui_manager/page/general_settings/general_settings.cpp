#include "general_settings.h"
#include "../../ui_manager.h"
#include "../../engine/page_construction/page_macros.h"
#include "../../engine/page_construction/page_template.h"
#include "../../../config_manager/config_manager.h"
#include "../../../../protocol/usb_serial_logs/usb_serial_logs.h"
#include <cstdio>

namespace ui {

// 静态成员变量定义
int32_t GeneralSettings::screen_timeout_seconds_ = 300;
int32_t GeneralSettings::brightness_value_ = 128;
uint8_t GeneralSettings::brightness_progress_data_ = 128;

GeneralSettings::GeneralSettings() {
    // 从配置管理器加载当前设置值
    ConfigManager* config_manager = ConfigManager::getInstance();
    if (config_manager) {
        // 加载息屏超时设置（从毫秒转换为秒）
        uint16_t timeout_ms = config_manager->get_uint16("UIMANAGER_SCREEN_TIMEOUT");
        screen_timeout_seconds_ = static_cast<int32_t>(timeout_ms / 1000);
        
        // 加载亮度设置
        brightness_value_ = static_cast<int32_t>(config_manager->get_uint8("UIMANAGER_BRIGHTNESS"));
        
        // 更新进度条数据
        update_brightness_progress();
    }
}

void GeneralSettings::render(PageTemplate& page_template) {
    // 通用设置页面 - 使用页面构造宏
    PAGE_START()
    SET_TITLE("通用设置", COLOR_WHITE)
    
    // 返回上级页面
    ADD_BACK_ITEM("返回", COLOR_TEXT_WHITE)
    
    // 息屏超时设置
    ADD_INT_SETTING(&screen_timeout_seconds_, 30, 3600, "秒", "息屏超时", 
                    on_screen_timeout_changed, on_screen_timeout_complete, COLOR_TEXT_WHITE)
    
    // 亮度设置标题
    ADD_TEXT("亮度设置:", COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // 亮度进度条显示
    ADD_PROGRESS(&brightness_progress_data_, COLOR_BLUE)
    
    // 亮度数值调整
    ADD_INT_SETTING(&brightness_value_, 0, 255, "亮度:", "亮度值", 
                    on_brightness_changed, on_brightness_complete, COLOR_TEXT_WHITE)
    
    PAGE_END()
}

void GeneralSettings::update_brightness_progress() {
    // 将亮度值（0-255）转换为进度条数据（0-255）
    brightness_progress_data_ = static_cast<uint8_t>(brightness_value_);
}

void GeneralSettings::on_screen_timeout_changed(int new_value) {
    screen_timeout_seconds_ = new_value;
    
    // 通过UIManager接口设置息屏超时，不保存配置
    UIManager* ui_manager = UIManager::getInstance();
    if (ui_manager) {
        ui_manager->set_screen_timeout(static_cast<uint16_t>(new_value));
    }
}

void GeneralSettings::on_screen_timeout_complete() {
     // 在完成时保存配置到ConfigManager
     ConfigManager* config_manager = ConfigManager::getInstance();
     if (config_manager) {
         // 将秒转换为毫秒保存
         config_manager->set_uint16("UIMANAGER_SCREEN_TIMEOUT", static_cast<uint16_t>(screen_timeout_seconds_ * 1000));
         config_manager->save_config();
     }
 }

void GeneralSettings::on_brightness_changed(int new_value) {
    brightness_value_ = new_value;
    
    // 通过UIManager接口设置亮度，不保存配置
    UIManager* ui_manager = UIManager::getInstance();
    if (ui_manager) {
        ui_manager->set_brightness(static_cast<uint8_t>(new_value));
    }
    
    // 更新进度条显示
    brightness_progress_data_ = static_cast<uint8_t>(new_value);
}

void GeneralSettings::on_brightness_complete() {
    // 在完成时保存配置到ConfigManager
    ConfigManager::log_debug("Brightness setting complete, value: " + std::to_string(brightness_value_));
    ConfigManager* config_manager = ConfigManager::getInstance();
    if (config_manager) {
        ConfigManager::log_debug("Setting UIMANAGER_BRIGHTNESS to: " + std::to_string(static_cast<uint8_t>(brightness_value_)));
        config_manager->set_uint8("UIMANAGER_BRIGHTNESS", static_cast<uint8_t>(brightness_value_));
        
        // 验证设置是否成功
        uint8_t saved_value = config_manager->get_uint8("UIMANAGER_BRIGHTNESS");
        ConfigManager::log_debug("Verified saved brightness value: " + std::to_string(saved_value));
        
        config_manager->save_config();
    } else {
        ConfigManager::log_error("ConfigManager instance is null!");
    }
}

} // namespace ui