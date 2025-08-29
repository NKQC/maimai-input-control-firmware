#pragma once

#include "page_types.h"
#include "../engine/ui_constructs.h"
#include "page_template.h"
#include "main_menu_page.h"
#include "settings_page.h"
#include "status_page.h"
#include <memory>
#include <unordered_map>
#include <string>

class GraphicsEngine;

namespace ui {

/**
 * 页面管理器 - 负责管理所有页面的创建、注册和获取
 */
class PageManager {
public:
    PageManager();
    ~PageManager();

    // 初始化所有页面
    bool init(GraphicsEngine* graphics_engine);
    
    // 清理所有页面
    void deinit();
    
    // 获取指定页面
    std::shared_ptr<PageTemplate> get_page(const std::string& page_id) const;
    
    // 注册页面
    void register_page(const std::string& page_id, std::shared_ptr<PageTemplate> page);
    
    // 注销页面
    void unregister_page(const std::string& page_id);
    
    // 检查页面是否存在
    bool has_page(const std::string& page_id) const;

private:
    // 页面存储
    std::unordered_map<std::string, std::shared_ptr<PageTemplate>> pages_;
    
    // GraphicsEngine实例
    GraphicsEngine* graphics_engine_;
    
    // 页面创建方法
    std::shared_ptr<PageTemplate> create_main_menu_page();
    std::shared_ptr<PageTemplate> create_settings_page();
    std::shared_ptr<PageTemplate> create_status_page();
    std::shared_ptr<PageTemplate> create_calibration_page();
    std::shared_ptr<PageTemplate> create_diagnostics_page();
};

} // namespace ui