#include "main_page.h"
#include "src/service/config_manager/config_manager.h"
#include "../engine/page_construction/page_macros.h"
#include "pico/time.h"
#include "pico/stdlib.h"
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

    // 主界面页面 - 使用页面构造宏简化代码
    PAGE_START()
    SET_TITLE("主界面", COLOR_WHITE)
    
    // 第一行：运行时长
    std::string uptime_text = "运行时长: " + format_uptime(uptime_ms_);
    ADD_TEXT(uptime_text, COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // 第二行：系统状态
    std::string status_text = "状态: " + system_status_;
    Color status_color = (system_status_ == "系统正常") ? COLOR_SUCCESS : COLOR_ERROR;
    ADD_TEXT(status_text, status_color, LineAlign::LEFT)
    
    // 第三行：连接状态
    std::string connection_text = "连接: " + get_connection_text();
    Color connection_color = connection_status_ ? COLOR_SUCCESS : COLOR_WARNING;
    ADD_TEXT(connection_text, connection_color, LineAlign::LEFT)
    
    // 第四行：隐藏的菜单跳转项，用于按钮A跳转到主菜单
    ADD_MENU("A:菜单 B:设置", "main_menu", COLOR_TEXT_WHITE)
    
    // 第五行：额外信息行（用于测试滚动条）
    ADD_TEXT("内存使用: 45%", COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // 第六行：额外信息行
    ADD_TEXT("CPU温度: 42°C", COLOR_TEXT_WHITE, LineAlign::LEFT)
    
    // 第七行：额外菜单项
    ADD_MENU("调试信息", "debug", COLOR_TEXT_WHITE)
    
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