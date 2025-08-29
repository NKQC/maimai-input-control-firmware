#include "settings_page.h"
#include "../engine/graphics_engine.h"
#include "../engine/font_system.h"

namespace ui {

SettingsPage::SettingsPage(GraphicsEngine* graphics_engine) : PageTemplate(graphics_engine) {
    // 构造函数实现
}

SettingsPage::~SettingsPage() {
    // 析构函数实现
}

bool SettingsPage::init() {
    // 初始化页面
    return true;
}

void SettingsPage::deinit() {
    // 清理页面资源
}

void SettingsPage::draw(GraphicsEngine* graphics) {
    if (!graphics) return;
    
    // 绘制设置页面
    graphics->clear();
    graphics->draw_text("设置", 10, 10, 0xFFFF);
    
    // 绘制设置项
    const char* setting_items[] = {
        "亮度设置",
        "背光设置", 
        "摇杆设置",
        "触摸映射",
        "按键映射",
        "灯光映射",
        "UART设置",
        "返回主菜单"
    };
    
    int y_pos = 40;
    for (int i = 0; i < 8; i++) {
        uint16_t color = (selected_index_ == i) ? 0xF800 : 0xFFFF; // 红色高亮
        graphics->draw_text(setting_items[i], 20, y_pos, color);
        y_pos += 18;
    }
}

void SettingsPage::update() {
    // 更新页面状态
}


bool SettingsPage::handle_setting_selection() {
    // 根据选中的设置项执行相应操作
    switch (selected_index_) {
        case 0: // 亮度设置
            // 进入亮度设置
            break;
        case 1: // 背光设置
            // 进入背光设置
            break;
        case 2: // 摇杆设置
            // 进入摇杆设置
            break;
        case 3: // 触摸映射
            // 切换到触摸映射页面
            break;
        case 4: // 按键映射
            // 切换到按键映射页面
            break;
        case 5: // 灯光映射
            // 切换到灯光映射页面
            break;
        case 6: // UART设置
            // 切换到UART设置页面
            break;
        case 7: // 返回主菜单
            // 返回主菜单
            break;
        default:
            return false;
    }
    return true;
}

void SettingsPage::set_selected_index(int index) {
    if (index >= 0 && index < 8) {
        selected_index_ = index;
    }
}

int SettingsPage::get_selected_index() const {
    return selected_index_;
}

} // namespace ui