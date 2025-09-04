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
    PageState new_page(page, cursor_pos, scroll_pos);
    
    // 检查是否存在跳跃式回环 (A->B->C->D->B)
    for (auto it = page_history_.begin(); it != page_history_.end(); ++it) {
        if (it->page_name == page) {
            // 找到重复页面，清空该页面后续的所有路径
            page_history_.erase(it + 1, page_history_.end());
            return; // 不添加重复页面，直接返回
        }
    }
    
    // 正常添加新页面到历史记录
    page_history_.push_back(new_page);
}

PageState PageNavigationManager::pop_page() {
    if (!page_history_.empty()) {
        PageState state = page_history_.back();
        page_history_.pop_back();
        return state;
    }
    return PageState(main_page_, 0, 0);
}

std::string PageNavigationManager::get_current_page() const {
    return !page_history_.empty() ? page_history_.back().page_name : main_page_;
}

std::string PageNavigationManager::get_previous_page() const {
    if (page_history_.size() >= 2) {
        return page_history_[page_history_.size() - 2].page_name;
    }
    return main_page_;
}

PageState PageNavigationManager::get_previous_page_state() const {
    if (page_history_.size() >= 2) {
        return page_history_[page_history_.size() - 2];
    }
    return PageState(main_page_, 0, 0);
}

bool PageNavigationManager::can_go_back() const {
    return !page_history_.empty();
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
    page_history_.clear();
}
