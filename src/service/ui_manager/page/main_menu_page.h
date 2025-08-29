#ifndef MAIN_MENU_PAGE_H
#define MAIN_MENU_PAGE_H

#include "page_template.h"
#include "page_types.h"

class GraphicsEngine;
class UIManager;

namespace ui {

class MainMenuPage : public PageTemplate {
public:
    MainMenuPage(GraphicsEngine* graphics_engine);
    virtual ~MainMenuPage();
    
    // PageTemplate接口实现
    bool init();
    void deinit();
    void draw(GraphicsEngine* graphics);
    void update();
    
    // 主菜单特有方法
    void set_selected_index(int index);
    int get_selected_index() const;
    
private:
    int selected_index_ = 0;  // 当前选中的菜单项索引
    
    bool handle_menu_selection();
};

} // namespace ui

#endif // MAIN_MENU_PAGE_H