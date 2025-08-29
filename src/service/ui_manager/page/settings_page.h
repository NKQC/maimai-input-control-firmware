#ifndef SETTINGS_PAGE_H
#define SETTINGS_PAGE_H

#include "page_template.h"
#include "page_types.h"

class GraphicsEngine;
class UIManager;

namespace ui {

class SettingsPage : public PageTemplate {
public:
    SettingsPage(GraphicsEngine* graphics_engine);
    virtual ~SettingsPage();
    
    // PageTemplate接口实现
    bool init();
    void deinit();
    void draw(GraphicsEngine* graphics);
    void update();
    
    // 设置页面特有方法
    void set_selected_index(int index);
    int get_selected_index() const;
    
private:
    int selected_index_ = 0;  // 当前选中的设置项索引
    
    bool handle_setting_selection();
};

} // namespace ui

#endif // SETTINGS_PAGE_H