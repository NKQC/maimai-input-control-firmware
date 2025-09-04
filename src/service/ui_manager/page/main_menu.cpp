#include "main_menu.h"
#include "../ui_manager.h"
#include "src/service/config_manager/config_manager.h"
#include "src/service/input_manager/input_manager.h"
#include "src/service/light_manager/light_manager.h"
#include "../engine/page_construction/page_macros.h"
#include "../engine/page_construction/page_template.h"
#include <cstdio>

namespace ui {

MainMenu::MainMenu() 
    : progress_(0), progress_data_(0) {
    // 初始化progress_data_与progress_同步
    progress_data_ = (uint8_t)((progress_ * 255) / 100);
}

void MainMenu::render(PageTemplate& page_template) {
    // 主菜单页面 - 使用页面构造宏
    PAGE_START()

    ADD_BACK_ITEM("返回", COLOR_WHITE)

    SET_TITLE("主菜单", COLOR_WHITE)
    
    ADD_MENU("触摸设置", "touch_settings_main", COLOR_TEXT_WHITE)

    ADD_MENU("绑定设置", "binding_settings", COLOR_TEXT_WHITE)
    
    ADD_MENU("通用设置", "general_settings", COLOR_TEXT_WHITE)
    
    ADD_BUTTON("保存设置", [this]() {this->save_config();}, COLOR_WHITE, LineAlign::LEFT)
    
    PAGE_END()
}

void MainMenu::set_progress(int progress) {
    // 限制进度值在0-100范围内
    if (progress < 0) {
        progress_ = 0;
    } else if (progress > 100) {
        progress_ = 100;
    } else {
        progress_ = progress;
    }
    
    // 同步更新progress_data_成员变量
    progress_data_ = (uint8_t)((progress_ * 255) / 100);
    
    // 将进度存储到共享数据中，供宏控制使用
    char progress_str[16];
    snprintf(progress_str, sizeof(progress_str), "%d", progress_);
    set_shared_data("main_menu_progress", progress_str);
}

int MainMenu::get_progress() const {
    return progress_;
}

void MainMenu::save_config() {
    // 获取各服务的当前配置并写入到ConfigManager
    ConfigManager* config_mgr = ConfigManager::getInstance();
    if (config_mgr) {
        // 调用各服务的配置写入函数
        InputManager_PrivateConfig input_config = inputmanager_get_config_copy();
        inputmanager_write_config_to_manager(input_config);
        
        UIManager_PrivateConfig ui_config = ui_manager_get_config_copy();
        ui_manager_write_config_to_manager(config_mgr, ui_config);
        
        LightManager_PrivateConfig light_config = lightmanager_get_config_copy();
        lightmanager_write_config_to_manager(light_config);
        
        // 统一保存所有配置到存储
        ConfigManager::save_config();
    }
}


} // namespace ui