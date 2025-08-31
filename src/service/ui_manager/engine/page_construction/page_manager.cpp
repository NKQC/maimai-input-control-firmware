#include "page_manager.h"
#include "../../ui_manager.h"
#include <algorithm>
#include <cmath>

// ============================================================================
// PageNavigationManager 实现
// ============================================================================

PageNavigationManager& PageNavigationManager::getInstance() {
    static PageNavigationManager instance;
    return instance;
}

void PageNavigationManager::push_page(const std::string& page, int cursor_pos, int scroll_pos) {
    previous_page_state_ = PageState(page, cursor_pos, scroll_pos);
    has_previous_page_ = true;
}

PageState PageNavigationManager::pop_page() {
    if (has_previous_page_) {
        PageState state = previous_page_state_;
        has_previous_page_ = false;
        previous_page_state_ = PageState(main_page_, 0, 0);
        return state;
    }
    return PageState(main_page_, 0, 0);
}

std::string PageNavigationManager::get_current_page() const {
    return has_previous_page_ ? previous_page_state_.page_name : main_page_;
}

std::string PageNavigationManager::get_previous_page() const {
    return has_previous_page_ ? previous_page_state_.page_name : main_page_;
}

PageState PageNavigationManager::get_previous_page_state() const {
    if (has_previous_page_) {
        return previous_page_state_;
    }
    return PageState(main_page_, 0, 0);
}

bool PageNavigationManager::can_go_back() const {
    return has_previous_page_;
}

std::string PageNavigationManager::handle_back_navigation(bool has_interactive_content) {
    std::string current = get_current_page();
    
    if (current == main_page_) {
        // 主页面无交互内容时跳转至预设主菜单
        if (!has_interactive_content) {
            return main_page_; // 可以设置为特定的主菜单页面
        }
        return current; // 主页面有交互内容时保持不变
    } else {
        // 非主页面无交互内容时自动返回上一级
        if (!has_interactive_content) {
            PageState prev_state = pop_page();
            return prev_state.page_name;
        }
        return current; // 有交互内容时保持不变
    }
}

void PageNavigationManager::clear_stack() {
    has_previous_page_ = false;
    previous_page_state_ = PageState(main_page_, 0, 0);
}
