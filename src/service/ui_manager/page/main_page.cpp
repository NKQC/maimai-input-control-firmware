#include "main_page.h"
#include "src/service/config_manager/config_manager.h"
#include "../engine/page_construction/page_macros.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "../../input_manager/input_manager.h"
#include <cstdio>

namespace ui {

MainPage::MainPage() 
    : uptime_ms_(0)
    , system_status_("系统正常")
    , connection_status_(false)
    , current_page_name_("main") {
}

void MainPage::render(PageTemplate& page_template) {
    // 每帧更新运行时长
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    update_uptime(current_time_ms);

    // 获取InputManager实例来获取回报率信息
    InputManager* input_manager = InputManager::getInstance();
    
    // 主界面页面 - 使用页面构造宏简化代码
    PAGE_START()
    SET_TITLE("主界面", COLOR_WHITE)
    
    // 第一行：触摸轮询回报率
    std::string touch_rate_text = "触摸轮询: ";
    if (input_manager) {
        // 获取实际的采样率（不依赖特定设备ID）
        touch_rate_text += std::to_string(input_manager->getTouchSampleRate()) + "Hz";
    } else {
        touch_rate_text += "N/A";
    }
    ADD_TEXT(touch_rate_text, COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // 第二行：键盘回报率
    std::string keyboard_rate_text = "键盘回报: ";
    if (input_manager) {
        keyboard_rate_text += std::to_string(input_manager->getHIDReportRate()) + "Hz";
    } else {
        keyboard_rate_text += "N/A";
    }
    ADD_TEXT(keyboard_rate_text, COLOR_TEXT_WHITE, LineAlign::LEFT)

    // 第五行：主菜单跳转
    ADD_MENU(">> 主菜单", "main_menu", COLOR_TEXT_WHITE)
    
    PAGE_END()
}

void MainPage::update_uptime(uint32_t uptime_ms) {
    uptime_ms_ = uptime_ms;
    
    // 将运行时长存储到共享数据中，供其他页面使用
    char uptime_str[32];
    snprintf(uptime_str, sizeof(uptime_str), "%lu", (unsigned long)uptime_ms);
    set_shared_data("system_uptime_ms", uptime_str);
    set_shared_data("system_uptime_formatted", format_uptime(uptime_ms));
}

void MainPage::set_system_status(const std::string& status) {
    system_status_ = status;
    set_shared_data("system_status", status);
}

void MainPage::set_connection_status(bool connected) {
    connection_status_ = connected;
    set_shared_data("connection_status", connected ? "true" : "false");
}

std::string MainPage::format_uptime(uint32_t uptime_ms) const {
    uint32_t total_seconds = uptime_ms / 1000;
    uint32_t hours = total_seconds / 3600;
    uint32_t minutes = (total_seconds % 3600) / 60;
    uint32_t seconds = total_seconds % 60;
    
    char buffer[32];
    if (hours > 0) {
        snprintf(buffer, sizeof(buffer), "%luh%lum%lus", 
                (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
    } else if (minutes > 0) {
        snprintf(buffer, sizeof(buffer), "%lum%lus", 
                (unsigned long)minutes, (unsigned long)seconds);
    } else {
        snprintf(buffer, sizeof(buffer), "%lus", (unsigned long)seconds);
    }
    
    return std::string(buffer);
}

std::string MainPage::get_connection_text() const {
    return connection_status_ ? "已连接" : "未连接";
}

void MainPage::set_current_page_name(const std::string& page_name) {
    current_page_name_ = page_name;
}

} // namespace ui