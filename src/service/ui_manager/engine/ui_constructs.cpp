#include "ui_constructs.h"
#include "../ui_manager.h"
#include <algorithm>
#include <cmath>

// ============================================================================
// PageJumpConstruct 实现
// ============================================================================

PageJumpConstruct::PageJumpConstruct(const std::string& text, ui::UIPage target_page)
    : static_text_(text), target_page_(target_page), use_dynamic_text_(false) {
}

PageJumpConstruct::PageJumpConstruct(DynamicStringFunc text_func, ui::UIPage target_page)
    : dynamic_text_func_(text_func), target_page_(target_page), use_dynamic_text_(true) {
}

std::string PageJumpConstruct::get_display_text() const {
    if (use_dynamic_text_ && dynamic_text_func_) {
        return dynamic_text_func_();
    }
    return static_text_;
}

bool PageJumpConstruct::handle_interaction() {
    UIManager* ui_manager = UIManager::getInstance();
    if (ui_manager) {
        return ui_manager->set_current_page(target_page_);
    }
    return false;
}

void PageJumpConstruct::update() {
    // 动态文本每帧更新
    if (use_dynamic_text_ && dynamic_text_func_) {
        // 文本内容在get_display_text()中动态获取，这里无需额外操作
    }
}

// ============================================================================
// ButtonConstruct 实现
// ============================================================================

ButtonConstruct::ButtonConstruct(const std::string& text, ButtonCallback callback)
    : static_text_(text), callback_(callback), use_dynamic_text_(false) {
}

ButtonConstruct::ButtonConstruct(DynamicStringFunc text_func, ButtonCallback callback)
    : dynamic_text_func_(text_func), callback_(callback), use_dynamic_text_(true) {
}

std::string ButtonConstruct::get_display_text() const {
    if (use_dynamic_text_ && dynamic_text_func_) {
        return dynamic_text_func_();
    }
    return static_text_;
}

bool ButtonConstruct::handle_interaction() {
    if (callback_) {
        callback_();
        return true;
    }
    return false;
}

void ButtonConstruct::update() {
    // 动态文本每帧更新
    if (use_dynamic_text_ && dynamic_text_func_) {
        // 文本内容在get_display_text()中动态获取，这里无需额外操作
    }
}

// ============================================================================
// SettingsConstruct 实现
// ============================================================================

SettingsConstruct::SettingsConstruct(const std::string& text, int* target_variable,
                                   int min_value, int max_value, SettingsCallback callback)
    : static_text_(text), target_variable_(target_variable), min_value_(min_value),
      max_value_(max_value), callback_(callback), use_dynamic_text_(false) {
    if (target_variable_) {
        clamp_value();
    }
}

SettingsConstruct::SettingsConstruct(DynamicStringFunc text_func, int* target_variable,
                                   int min_value, int max_value, SettingsCallback callback)
    : dynamic_text_func_(text_func), target_variable_(target_variable), min_value_(min_value),
      max_value_(max_value), callback_(callback), use_dynamic_text_(true) {
    if (target_variable_) {
        clamp_value();
    }
}

std::string SettingsConstruct::get_display_text() const {
    std::string base_text;
    if (use_dynamic_text_ && dynamic_text_func_) {
        base_text = dynamic_text_func_();
    } else {
        base_text = static_text_;
    }
    
    // 添加当前值显示
    if (target_variable_) {
        base_text += ": " + std::to_string(*target_variable_);
    }
    
    return base_text;
}

bool SettingsConstruct::handle_interaction() {
    // 设置构造体的交互通过摇杆左右调整值，确认键触发回调
    if (callback_) {
        callback_(get_current_value());
    }
    return true;
}

void SettingsConstruct::update() {
    // 动态文本每帧更新
    if (use_dynamic_text_ && dynamic_text_func_) {
        // 文本内容在get_display_text()中动态获取，这里无需额外操作
    }
}

void SettingsConstruct::adjust_value(int delta) {
    if (target_variable_) {
        *target_variable_ += delta;
        clamp_value();
    }
}

int SettingsConstruct::get_current_value() const {
    return target_variable_ ? *target_variable_ : 0;
}

void SettingsConstruct::set_value(int value) {
    if (target_variable_) {
        *target_variable_ = value;
        clamp_value();
    }
}

void SettingsConstruct::clamp_value() {
    if (target_variable_) {
        *target_variable_ = std::max(min_value_, std::min(max_value_, *target_variable_));
    }
}

// ============================================================================
// TextConstruct 实现
// ============================================================================

TextConstruct::TextConstruct(const std::string& text)
    : static_text_(text), use_dynamic_text_(false) {
}

TextConstruct::TextConstruct(DynamicStringFunc text_func)
    : dynamic_text_func_(text_func), use_dynamic_text_(true) {
}

std::string TextConstruct::get_display_text() const {
    if (use_dynamic_text_ && dynamic_text_func_) {
        return dynamic_text_func_();
    }
    return static_text_;
}

void TextConstruct::update() {
    // 动态文本每帧更新
    if (use_dynamic_text_ && dynamic_text_func_) {
        // 文本内容在get_display_text()中动态获取，这里无需额外操作
    }
}

// ============================================================================
// ConstructPage 实现
// ============================================================================

ConstructPage::ConstructPage(const std::string& title)
    : title_(title), selected_index_(-1) {
}

ConstructPage::~ConstructPage() {
    clear_constructs();
}

void ConstructPage::add_construct(std::shared_ptr<UIConstruct> construct) {
    if (construct) {
        constructs_.push_back(construct);
        
        // 如果是第一个交互元素，自动选中
        if (selected_index_ == -1 && construct->is_interactive()) {
            selected_index_ = constructs_.size() - 1;
            update_selection();
        }
    }
}

void ConstructPage::remove_construct(size_t index) {
    if (index < constructs_.size()) {
        constructs_.erase(constructs_.begin() + index);
        
        // 调整选中索引
        if (selected_index_ >= static_cast<int>(constructs_.size())) {
            selected_index_ = constructs_.size() - 1;
        }
        
        // 确保选中的是交互元素
        if (selected_index_ >= 0 && !constructs_[selected_index_]->is_interactive()) {
            selected_index_ = find_next_interactive(selected_index_, true);
        }
        
        update_selection();
    }
}

void ConstructPage::clear_constructs() {
    constructs_.clear();
    selected_index_ = -1;
}

std::shared_ptr<UIConstruct> ConstructPage::get_construct(size_t index) const {
    if (index < constructs_.size()) {
        return constructs_[index];
    }
    return nullptr;
}

size_t ConstructPage::get_construct_count() const {
    return constructs_.size();
}

bool ConstructPage::has_interactive_elements() const {
    for (const auto& construct : constructs_) {
        if (construct && construct->is_interactive()) {
            return true;
        }
    }
    return false;
}

std::vector<size_t> ConstructPage::get_interactive_indices() const {
    std::vector<size_t> indices;
    for (size_t i = 0; i < constructs_.size(); ++i) {
        if (constructs_[i] && constructs_[i]->is_interactive()) {
            indices.push_back(i);
        }
    }
    return indices;
}

void ConstructPage::set_selected_index(int index) {
    if (index >= 0 && index < static_cast<int>(constructs_.size()) && 
        constructs_[index]->is_interactive()) {
        selected_index_ = index;
        update_selection();
    }
}

bool ConstructPage::navigate_up() {
    if (constructs_.empty()) return false;
    
    int new_index = find_next_interactive(selected_index_ - 1, false);
    if (new_index != selected_index_) {
        selected_index_ = new_index;
        update_selection();
        return true;
    }
    return false;
}

bool ConstructPage::navigate_down() {
    if (constructs_.empty()) return false;
    
    int new_index = find_next_interactive(selected_index_ + 1, true);
    if (new_index != selected_index_) {
        selected_index_ = new_index;
        update_selection();
        return true;
    }
    return false;
}

bool ConstructPage::navigate_left() {
    // Z字型导航：向左移动
    if (selected_index_ > 0) {
        int new_index = find_next_interactive(selected_index_ - 1, false);
        if (new_index != selected_index_) {
            selected_index_ = new_index;
            update_selection();
            return true;
        }
    }
    return false;
}

bool ConstructPage::navigate_right() {
    // Z字型导航：向右移动
    if (selected_index_ < static_cast<int>(constructs_.size()) - 1) {
        int new_index = find_next_interactive(selected_index_ + 1, true);
        if (new_index != selected_index_) {
            selected_index_ = new_index;
            update_selection();
            return true;
        }
    }
    return false;
}

bool ConstructPage::handle_confirm() {
    if (selected_index_ >= 0 && selected_index_ < static_cast<int>(constructs_.size())) {
        auto construct = constructs_[selected_index_];
        if (construct && construct->is_interactive()) {
            // 特殊处理Settings构造体的左右调整
            if (construct->get_type() == "Settings") {
                return construct->handle_interaction();
            } else {
                return construct->handle_interaction();
            }
        }
    }
    return false;
}

void ConstructPage::update_all() {
    for (auto& construct : constructs_) {
        if (construct) {
            construct->update();
        }
    }
}

void ConstructPage::render_to_page_template(PageTemplate& page_template) {
    page_template.clear();
    
    // 设置标题
    if (!title_.empty()) {
        page_template.set_title(title_);
    }
    
    // 渲染构造体到行
    std::vector<LineConfig> lines;
    for (size_t i = 0; i < constructs_.size() && i < 4; ++i) {
        auto construct = constructs_[i];
        if (construct) {
            LineType line_type = construct->is_interactive() ? LineType::MENU_ITEM : LineType::CONTENT;
            bool is_selected = (static_cast<int>(i) == selected_index_);
            
            LineConfig config(line_type, construct->get_display_text(), 
                            COLOR_TEXT_WHITE, FontSize::MEDIUM, LineAlign::LEFT, is_selected);
            lines.push_back(config);
        }
    }
    
    page_template.set_lines(lines);
}

void ConstructPage::update_selection() {
    // 更新所有构造体的选中状态
    for (size_t i = 0; i < constructs_.size(); ++i) {
        if (constructs_[i]) {
            constructs_[i]->set_selected(static_cast<int>(i) == selected_index_);
        }
    }
}

int ConstructPage::find_next_interactive(int start_index, bool forward) const {
    if (constructs_.empty()) return -1;
    
    int size = static_cast<int>(constructs_.size());
    int step = forward ? 1 : -1;
    
    // 循环查找下一个交互元素
    for (int i = 0; i < size; ++i) {
        int index = (start_index + i * step + size) % size;
        if (constructs_[index] && constructs_[index]->is_interactive()) {
            return index;
        }
    }
    
    return selected_index_; // 没找到则保持当前选择
}

// ============================================================================
// PageNavigationManager 实现
// ============================================================================

PageNavigationManager& PageNavigationManager::getInstance() {
    static PageNavigationManager instance;
    return instance;
}

void PageNavigationManager::push_page(ui::UIPage page) {
    page_stack_.push_back(page);
}

ui::UIPage PageNavigationManager::pop_page() {
    if (!page_stack_.empty()) {
        ui::UIPage page = page_stack_.back();
        page_stack_.pop_back();
        return page;
    }
    return main_page_;
}

ui::UIPage PageNavigationManager::get_current_page() const {
    return page_stack_.empty() ? main_page_ : page_stack_.back();
}

ui::UIPage PageNavigationManager::get_previous_page() const {
    if (page_stack_.size() > 1) {
        return page_stack_[page_stack_.size() - 2];
    }
    return main_page_;
}

bool PageNavigationManager::can_go_back() const {
    return !page_stack_.empty();
}

ui::UIPage PageNavigationManager::handle_back_navigation(bool has_interactive_content) {
    ui::UIPage current = get_current_page();
    
    if (current == main_page_) {
        // 主页面无交互内容时跳转至预设主菜单
        if (!has_interactive_content) {
            return main_page_; // 可以设置为特定的主菜单页面
        }
        return current; // 主页面有交互内容时保持不变
    } else {
        // 非主页面无交互内容时自动返回上一级
        if (!has_interactive_content) {
            return pop_page();
        }
        return current; // 有交互内容时保持不变
    }
}

void PageNavigationManager::clear_stack() {
    page_stack_.clear();
}

// ============================================================================
// MenuInteractionSystem 实现
// ============================================================================

MenuInteractionSystem& MenuInteractionSystem::getInstance() {
    static MenuInteractionSystem instance;
    return instance;
}

void MenuInteractionSystem::register_page(ui::UIPage page_id, std::shared_ptr<PageTemplate> page) {
    if (page) {
        pages_[page_id] = page;
    }
}

void MenuInteractionSystem::unregister_page(ui::UIPage page_id) {
    pages_.erase(page_id);
}

std::shared_ptr<PageTemplate> MenuInteractionSystem::get_current_page() const {
    return get_page(current_page_id_);
}

std::shared_ptr<PageTemplate> MenuInteractionSystem::get_page(ui::UIPage page_id) const {
    auto it = pages_.find(page_id);
    return (it != pages_.end()) ? it->second : nullptr;
}

bool MenuInteractionSystem::switch_to_page(ui::UIPage page_id) {
    auto page = get_page(page_id);
    if (page) {
        PageNavigationManager::getInstance().push_page(current_page_id_);
        current_page_id_ = page_id;
        return true;
    }
    return false;
}

bool MenuInteractionSystem::handle_joystick_up() {
    auto page = get_current_page();
    if (page) {
        int current_index = page->get_selected_index();
        int item_count = page->get_menu_item_count();
        if (item_count > 0) {
            int new_index = (current_index - 1 + item_count) % item_count;
            page->set_selected_index(new_index);
            return true;
        }
    }
    return false;
}

bool MenuInteractionSystem::handle_joystick_down() {
    auto page = get_current_page();
    if (page) {
        int current_index = page->get_selected_index();
        int item_count = page->get_menu_item_count();
        if (item_count > 0) {
            int new_index = (current_index + 1) % item_count;
            page->set_selected_index(new_index);
            return true;
        }
    }
    return false;
}

bool MenuInteractionSystem::handle_joystick_left() {
    // PageTemplate系统中左右导航暂不支持
    // 可以在此处添加特定的左导航逻辑
    return false;
}

bool MenuInteractionSystem::handle_joystick_right() {
    // PageTemplate系统中左右导航暂不支持
    // 可以在此处添加特定的右导航逻辑
    return false;
}

bool MenuInteractionSystem::handle_joystick_confirm() {
    // PageTemplate系统中确认操作需要通过回调处理
    // 可以在此处添加菜单项确认逻辑
    return false;
}

bool MenuInteractionSystem::handle_back_button() {
    auto page = get_current_page();
    bool has_interactive = page ? (page->get_menu_item_count() > 0) : false;
    
    ui::UIPage target_page = PageNavigationManager::getInstance().handle_back_navigation(has_interactive);
    
    if (target_page != current_page_id_) {
        current_page_id_ = target_page;
        return true;
    }
    return false;
}

void MenuInteractionSystem::update() {
    // PageTemplate系统中更新逻辑由外部管理
    // 可以在此处添加特定的更新逻辑
}

void MenuInteractionSystem::render_current_page(PageTemplate& page_template) {
    auto page = get_current_page();
    if (page) {
        // 直接复制页面内容到目标模板
        page_template = *page;
    }
}