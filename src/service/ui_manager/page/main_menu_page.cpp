#include "main_menu_page.h"
#include "../engine/graphics_engine.h"
#include "../engine/font_system.h"

namespace ui {

MainMenuPage::MainMenuPage(GraphicsEngine* graphics_engine) : PageTemplate(graphics_engine) {
    // 构造函数实现
}

MainMenuPage::~MainMenuPage() {
    // 析构函数实现
}

bool MainMenuPage::init() {
    // 初始化页面
    return true;
}

void MainMenuPage::deinit() {
    // 清理页面资源
}

void MainMenuPage::draw(GraphicsEngine* graphics) {
    if (!graphics) return;
    
    // 绘制主菜单页面
    graphics->clear();
    graphics->draw_text("主菜单", 10, 10, 0xFFFF);
    
    // 绘制菜单项
    const char* menu_items[] = {
        "状态",
        "设置", 
        "校准",
        "诊断",
        "关于"
    };
    
    int y_pos = 40;
    for (int i = 0; i < 5; i++) {
        uint16_t color = (selected_index_ == i) ? 0xF800 : 0xFFFF; // 红色高亮
        graphics->draw_text(menu_items[i], 20, y_pos, color);
        y_pos += 20;
    }
}

void MainMenuPage::update() {
    // 更新页面状态
}

bool MainMenuPage::handle_menu_selection() {
    // 根据选中的菜单项执行相应操作
    switch (selected_index_) {
        case 0: // 状态
            // 切换到状态页面
            break;
        case 1: // 设置
            // 切换到设置页面
            break;
        case 2: // 校准
            // 切换到校准页面
            break;
        case 3: // 诊断
            // 切换到诊断页面
            break;
        case 4: // 关于
            // 切换到关于页面
            break;
        default:
            return false;
    }
    return true;
}

void MainMenuPage::set_selected_index(int index) {
    if (index >= 0 && index < 5) {
        selected_index_ = index;
    }
}

int MainMenuPage::get_selected_index() const {
    return selected_index_;
}

} // namespace ui