#ifndef UI_CONSTRUCTS_H
#define UI_CONSTRUCTS_H

#include "page_template.h"
#include <functional>
#include <string>
#include <vector>
#include <memory>
#include <map>

// 前向声明
class UIManager;

// 页面状态结构
struct PageState {
    std::string page_name;
    int cursor_position;
    int scroll_position;
    
    PageState(const std::string& name = "", int cursor = 0, int scroll = 0)
        : page_name(name), cursor_position(cursor), scroll_position(scroll) {}
};

// 页面回退管理器
class PageNavigationManager {
public:
    static PageNavigationManager& getInstance();
    
    // 简化的页面状态管理（只保存上一个页面）
    void push_page(const std::string& page, int cursor_pos = 0, int scroll_pos = 0);
    PageState pop_page();
    std::string get_current_page() const;
    std::string get_previous_page() const;
    bool can_go_back() const;
    
    // 获取上一页状态
    PageState get_previous_page_state() const;
    
    // 返回导航处理
    std::string handle_back_navigation(bool has_interactive_content);
    
    // 设置主页面
    void set_main_page(const std::string& main_page) { main_page_ = main_page; }
    std::string get_main_page() const { return main_page_; }
    
    // 清空页面状态
    void clear_stack();
    
private:
    PageNavigationManager() = default;
    PageState previous_page_state_;  // 只保存上一个页面状态
    bool has_previous_page_ = false; // 是否有上一个页面
    std::string main_page_ = "main";
};
#endif // UI_CONSTRUCTS_H