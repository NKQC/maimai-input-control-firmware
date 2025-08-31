#include "main_menu.h"
#include "../ui_manager.h"
#include "src/service/config_manager/config_manager.h"
#include "../engine/page_construction/page_macros.h"
#include "../engine/page_construction/page_template.h"
#include <cstdio>

namespace ui {

MainMenu::MainMenu() 
    : progress_(0), progress_data_(0) {
}

void MainMenu::render(PageTemplate& page_template) {
    // 主菜单页面 - 使用页面构造宏
    PAGE_START()
    SET_TITLE("主菜单", COLOR_WHITE)
    
    // 第一行：系统状态菜单项
    ADD_MENU("系统状态", "status", COLOR_TEXT_WHITE)
    
    // 第二行：设置菜单项
    ADD_MENU("设置", "settings", COLOR_TEXT_WHITE)
    
    // 第三行：校准菜单项
    ADD_MENU("校准", "calibration", COLOR_TEXT_WHITE)
    
    // 第四行：诊断菜单项
    ADD_MENU("诊断", "diagnostics", COLOR_TEXT_WHITE)
    
    // 第五行：选择器测试菜单项
    ADD_MENU("选择器测试", "selector_test", COLOR_TEXT_WHITE)
    
    // 第六行：进度条测试 - 使用ADD_PROGRESS宏
    // 将progress_转换为uint8_t供进度条使用
    progress_data_ = (uint8_t)((progress_ * 255) / 100);  // 转换0-100到0-255
    ADD_PROGRESS(&progress_data_, COLOR_TEXT_WHITE)
    
    // 第七行：进度控制设置 - 使用ADD_INT_SETTING宏
    static int32_t progress_int = progress_;
    progress_int = progress_;
    ADD_INT_SETTING(&progress_int, 0, 100, "进度控制", "设置进度值", 
                   [this](int32_t new_value) { 
                       // 值变更回调
                       this->set_progress((int)new_value);
                   },
                   []() { 
                       // 完成回调
                       // 可以在这里添加保存配置等操作
                   }, 
                   COLOR_TEXT_WHITE)
    
    // 第八行：返回项
    ADD_BACK_ITEM("返回主界面", COLOR_TEXT_WHITE)
    
    PAGE_END()
    
    // 绘制页面
    page_template.draw();
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

} // namespace ui