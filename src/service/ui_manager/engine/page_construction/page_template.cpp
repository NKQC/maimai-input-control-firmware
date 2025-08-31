#include "page_template.h"
#include "../graphics_rendering/graphics_engine.h"
#include "src/service/ui_manager/ui_manager.h"
#include <algorithm>
#include <cstdio>

// LineConfig拷贝构造函数实现
LineConfig::LineConfig(const LineConfig& other)
    : type(other.type)
    , text(other.text)
    , color(other.color)
    , align(other.align)
    , selected(other.selected)
    , setting_title(other.setting_title)
    , target_page_name(other.target_page_name)
    , callback_type(other.callback_type) {
    // 复制类型特定数据（union）
    data = other.data;

    // 根据callback_type复制union中的回调数据
    switch (callback_type) {
        case CallbackType::VALUE_CHANGE:
            callback_data.value_change_callback = other.callback_data.value_change_callback;
            break;
        case CallbackType::COMPLETE:
            callback_data.complete_callback = other.callback_data.complete_callback;
            break;
        case CallbackType::CLICK:
            callback_data.click_callback = other.callback_data.click_callback;
            break;
        case CallbackType::SELECTOR:
            callback_data.selector_callback = other.callback_data.selector_callback;
            break;
        case CallbackType::NONE:
        default:
            // 对于NONE类型，不需要复制任何数据
            break;
    }
    
    // 复制独立的lock_callback
    lock_callback = other.lock_callback;
}

// LineConfig赋值操作符实现
LineConfig& LineConfig::operator=(const LineConfig& other) {
    if (this != &other) {
        text = other.text;
        type = other.type;
        color = other.color;
        align = other.align;
        selected = other.selected;
        setting_title = other.setting_title;
        target_page_name = other.target_page_name;
        callback_type = other.callback_type;

        // 复制类型特定数据（union）
        data = other.data;
        
        // 根据callback_type复制union中的回调数据
        switch (callback_type) {
            case CallbackType::VALUE_CHANGE:
                callback_data.value_change_callback = other.callback_data.value_change_callback;
                break;
            case CallbackType::COMPLETE:
                callback_data.complete_callback = other.callback_data.complete_callback;
                break;
            case CallbackType::CLICK:
                callback_data.click_callback = other.callback_data.click_callback;
                break;
            case CallbackType::SELECTOR:
                callback_data.selector_callback = other.callback_data.selector_callback;
                break;
            case CallbackType::NONE:
            default:
                // 对于NONE类型，不需要复制任何数据
                break;
        }
        
        // 复制独立的lock_callback
        lock_callback = other.lock_callback;
    }
    return *this;
}

// LineConfig移动构造函数实现
LineConfig::LineConfig(LineConfig&& other) noexcept
    : type(other.type)
    , text(std::move(other.text))
    , color(other.color)
    , align(other.align)
    , selected(other.selected)
    , setting_title(std::move(other.setting_title))
    , target_page_name(std::move(other.target_page_name))
    , callback_type(other.callback_type) {
    // 移动类型特定数据（union）
    data = other.data;
    
    // 根据callback_type移动union中的回调数据
    switch (callback_type) {
        case CallbackType::VALUE_CHANGE:
            callback_data.value_change_callback = std::move(other.callback_data.value_change_callback);
            break;
        case CallbackType::COMPLETE:
            callback_data.complete_callback = std::move(other.callback_data.complete_callback);
            break;
        case CallbackType::CLICK:
            callback_data.click_callback = std::move(other.callback_data.click_callback);
            break;
        case CallbackType::SELECTOR:
            callback_data.selector_callback = std::move(other.callback_data.selector_callback);
            break;
        case CallbackType::NONE:
        default:
            break;
    }
    
    // 移动独立的lock_callback
    lock_callback = std::move(other.lock_callback);
    
    // 重置源对象
    other.callback_type = CallbackType::NONE;
}

// LineConfig析构函数实现
LineConfig::~LineConfig() {
    // 由于使用std::function，析构函数会自动处理回调函数的清理
    // 不需要显式清理
}

// LineConfig移动赋值操作符实现
LineConfig& LineConfig::operator=(LineConfig&& other) noexcept {
    if (this != &other) {
        text = std::move(other.text);
        type = other.type;
        color = other.color;
        align = other.align;
        selected = other.selected;
        setting_title = std::move(other.setting_title);
        target_page_name = std::move(other.target_page_name);
        callback_type = other.callback_type;
        
        // 移动类型特定数据（union）
        data = other.data;
        
        // 根据callback_type移动union中的回调数据
        switch (callback_type) {
            case CallbackType::VALUE_CHANGE:
                callback_data.value_change_callback = std::move(other.callback_data.value_change_callback);
                break;
            case CallbackType::COMPLETE:
                callback_data.complete_callback = std::move(other.callback_data.complete_callback);
                break;
            case CallbackType::CLICK:
                callback_data.click_callback = std::move(other.callback_data.click_callback);
                break;
            case CallbackType::SELECTOR:
                callback_data.selector_callback = std::move(other.callback_data.selector_callback);
                break;
            case CallbackType::NONE:
            default:
                break;
        }
        
        // 移动独立的lock_callback
        lock_callback = std::move(other.lock_callback);
        
        // 重置源对象
        other.callback_type = CallbackType::NONE;
    }
    return *this;
}

// 静态成员
bool PageTemplate::has_title_ = false;
bool PageTemplate::has_split_screen_ = false;

// 页面布局常量
static const int16_t TITLE_Y = 2;
static const int16_t TITLE_HEIGHT = 16;
static const int16_t LINE_WEIGHT = ST7735S_WIDTH - 5;
static const int16_t LINE_HEIGHT = 12;
static const int16_t LINE_SPACING = 2;
static const int16_t CONTENT_START_Y = TITLE_Y + TITLE_HEIGHT + LINE_SPACING;
static const int16_t SELECTION_INDICATOR_WIDTH = 8;
static const int16_t SPLIT_SCREEN_DIVIDER = 128 / 2;

PageTemplate::PageTemplate(GraphicsEngine* graphics_engine)
    : graphics_engine_(graphics_engine)
    , title_("")
    , title_color_(COLOR_WHITE)
    , lines_()  // 动态初始化行内容
    , all_lines_()  // 所有行内容（用于滚动）
    , visible_lines_count_(5)
    , selected_menu_index_(0)
    , scroll_bar_()  // 使用默认构造函数
    , scroll_enabled_(false)
    , split_screen_enabled_(false)
    , left_header_("")
    , right_header_("")
    , split_borders_enabled_(true)
    , split_ratio_(0.5f) {
    
    // 初始化lines_容器大小
    lines_.resize(visible_lines_count_);
    
    // 配置滚动条位置和样式 - 右侧纵向圆弧条
    ui::ScrollBar::Config scroll_config;
    scroll_config.x = 158;  // 屏幕最右侧边缘位置（160-2=158）
    scroll_config.y = CONTENT_START_Y;  // 与内容对齐
    scroll_config.width = 2;  // 更窄的滚动条
    scroll_config.height = visible_lines_count_ * (LINE_HEIGHT + LINE_SPACING) - LINE_SPACING;  // 根据可见行数计算高度
    scroll_config.bg_color = COLOR_DARK_GRAY;
    scroll_config.bar_color = COLOR_CYAN;  // 使用青色更明显
    scroll_config.border_color = COLOR_LIGHT_GRAY;
    scroll_config.show_border = false;  // 不显示边框，使用圆弧样式
    scroll_bar_.set_config(scroll_config);
}

void PageTemplate::flush() {
    // 只清空行缓存，保留滚动状态和选中索引
    lines_.clear();
    
    // 重置状态跟踪变量
    has_title_ = false;
    has_split_screen_ = false;
    
    // 清空标题但不重置滚动状态
    title_.clear();
    title_color_ = COLOR_WHITE;
    
    // 清空分屏相关但不重置滚动
    split_screen_enabled_ = false;
    left_lines_.clear();
    right_lines_.clear();
    left_header_.clear();
    right_header_.clear();
}

void PageTemplate::set_title(const std::string& title, Color color) {
    title_ = title;
    title_color_ = color;
    has_title_ = !title.empty();
    // 更新可见行数缓存
    visible_lines_count_ = has_title_ ? 4 : 5;
    // 重新调整lines_容器大小
    lines_.resize(visible_lines_count_);
}

void PageTemplate::set_line(int line_index, const LineConfig& config) {
    // 如果当前是滚动模式，操作all_lines_
    if (scroll_enabled_ && !all_lines_.empty()) {
        if (line_index >= 0 && line_index < (int)all_lines_.size()) {
            all_lines_[line_index] = config;
            update_scroll_display(); // 更新显示
        }
    } else {
        // 非滚动模式，操作lines_
        if (line_index >= 0 && line_index < 4) {
            lines_[line_index] = config;
        }
    }
}

void PageTemplate::set_lines(const std::vector<LineConfig>& lines) {
    // 直接设置可见行，避免递归调用set_all_lines
    lines_.clear();
    lines_.resize(visible_lines_count_);
    
    // 复制传入的行到可见行数组
    for (size_t i = 0; i < lines.size() && i < visible_lines_count_; ++i) {
        lines_[i] = lines[i];
    }
}

void PageTemplate::set_all_lines(const std::vector<LineConfig>& lines) {
    // 根据是否有标题动态设置可见行数
    visible_lines_count_ = has_title_ ? 4 : 5;
    lines_.clear();
    lines_.resize(visible_lines_count_);
    
    // 保存当前选中项的文本标识（而非索引）
    std::string selected_item_text = "";
    if (scroll_enabled_ && selected_menu_index_ >= 0 && selected_menu_index_ < (int)all_lines_.size()) {
        selected_item_text = all_lines_[selected_menu_index_].text;
    }
    
    // 更新内容
    all_lines_ = lines;
    
    if (lines.size() > visible_lines_count_) {
        // 设置滚动条参数
        scroll_bar_.setup_page_scroll(lines, visible_lines_count_);
        
        scroll_enabled_ = true;
        // 显示当前滚动位置的行
        update_scroll_display();
    } else {
        scroll_enabled_ = false;
        // 直接设置行内容，避免递归调用
        lines_.clear();
        lines_.resize(visible_lines_count_);
        for (int i = 0; i < visible_lines_count_ && i < (int)lines.size(); ++i) {
            lines_[i] = lines[i];
        }
        // 重置选中索引
        selected_menu_index_ = 0;
    }
}

void PageTemplate::clear() {
    title_.clear();
    title_color_ = COLOR_WHITE;
    for (auto& line : lines_) {
        line.text.clear();
        line.type = LineType::TEXT_ITEM;
        line.color = COLOR_TEXT_WHITE;
        line.selected = false;
        // Union数据会通过构造函数自动初始化为0
    }
    selected_menu_index_ = 0;
}

void PageTemplate::clear_line(int line_index) {
    if (line_index >= 0 && line_index < 4) {
        lines_[line_index].text.clear();
        lines_[line_index].type = LineType::TEXT_ITEM;
        lines_[line_index].selected = false;
        // Union数据会通过重新赋值自动重置
    }
}

void PageTemplate::draw() {
    if (!graphics_engine_) return;
    
    // 绘制背景 - 使用黑色背景而不是深色主题背景
    draw_background(COLOR_BLACK);
    
    if (split_screen_enabled_) {
        draw_split_screen();
    } else {
        // 绘制标题
        draw_title();
        
        // 绘制内容行
        for (int i = 0; i < visible_lines_count_; i++) {
            if (!lines_[i].text.empty()) {
                draw_line(i, lines_[i]);
            }
        }
        
        // 绘制滚动条（如果启用）
        if (scroll_enabled_) {
            scroll_bar_.render(*graphics_engine_);
        }
    }
}

void PageTemplate::draw_background(Color bg_color) {
    if (graphics_engine_) {
        graphics_engine_->clear(bg_color);
    }
}

void PageTemplate::set_selected_index(int index) {
    // 在不可滚动模式下，只处理可交互菜单项
    if (!scroll_enabled_) {
        int menu_count = get_menu_item_count();
        if (index >= 0 && index < menu_count) {
            // 统一使用all_lines_处理，在非滚动模式下all_lines_和lines_内容相同
            const auto& target_lines = all_lines_.empty() ? lines_ : all_lines_;
            auto& mutable_target_lines = all_lines_.empty() ? lines_ : all_lines_;
            
            // 找到第index个可交互菜单项的实际行索引
            int actual_line_index = -1;
            int menu_item_counter = 0;
            for (int i = 0; i < (int)target_lines.size(); ++i) {
                if (!target_lines[i].text.empty() && 
                    (target_lines[i].type == LineType::MENU_JUMP || 
                     target_lines[i].type == LineType::INT_SETTING || 
                     target_lines[i].type == LineType::BUTTON_ITEM || 
                     target_lines[i].type == LineType::BACK_ITEM || 
                     target_lines[i].type == LineType::SELECTOR_ITEM)) {
                    if (menu_item_counter == index) {
                        actual_line_index = i;
                        break;
                    }
                    menu_item_counter++;
                }
            }
            
            if (actual_line_index == -1) return; // 未找到有效的菜单项
            
            // 清除旧选中状态
            if (selected_menu_index_ >= 0 && selected_menu_index_ < (int)target_lines.size()) {
                mutable_target_lines[selected_menu_index_].selected = false;
                if (mutable_target_lines[selected_menu_index_].type == LineType::MENU_JUMP) {
                    mutable_target_lines[selected_menu_index_].color = COLOR_TEXT_WHITE;
                }
            }
            
            // 设置新选中状态
            selected_menu_index_ = actual_line_index;
            
            if (actual_line_index < (int)target_lines.size()) {
                mutable_target_lines[actual_line_index].selected = true;
                mutable_target_lines[actual_line_index].color = COLOR_PRIMARY;
            }
        }
    }
    // 在可滚动模式下，UIManager直接管理current_menu_index_，不需要在这里处理
}

bool PageTemplate::scroll_up() {
    if (!scroll_enabled_) {
        return false;
    }
    
    bool scrolled = scroll_bar_.scroll_up();
    if (scrolled) {
        update_scroll_display();
        // 添加调试日志
        UIManager::log_debug_static("ScrollBar: UP scrolled, start_index=" + std::to_string(scroll_bar_.get_display_start_index()));
    }
    return scrolled;
}

bool PageTemplate::scroll_down() {
    if (!scroll_enabled_) {
        return false;
    }
    
    bool scrolled = scroll_bar_.scroll_down();
    if (scrolled) {
        update_scroll_display();
        // 添加调试日志
        UIManager::log_debug_static("ScrollBar: DOWN scrolled, start_index=" + std::to_string(scroll_bar_.get_display_start_index()));
    }
    return scrolled;
}

void PageTemplate::update_scroll_display() {
    if (!scroll_enabled_) {
        return;
    }
    
    std::vector<LineConfig> visible_lines;
    scroll_bar_.get_visible_lines(visible_lines);
    
    // // 添加调试日志
    // UIManager::log_debug_static("update_scroll_display: start_index=" + std::to_string(scroll_bar_.get_display_start_index()) + 
    //                            ", visible_lines_size=" + std::to_string(visible_lines.size()) + 
    //                            ", visible_lines_count=" + std::to_string(visible_lines_count_));
    
    set_lines(visible_lines);
}

void PageTemplate::set_visible_end_line(int target_line_index) {
    if (!scroll_enabled_ || all_lines_.empty()) {
        return;
    }
    
    // 确保目标行索引在有效范围内
    if (target_line_index < 0 || target_line_index >= (int)all_lines_.size()) {
        return;
    }
    
    // 计算新的滚动起始位置
    int new_start_index;
    int max_start = std::max(0, (int)all_lines_.size() - visible_lines_count_);
    
    if (target_line_index < visible_lines_count_) {
        // 如果目标行在前几行，直接从第0行开始显示
        new_start_index = 0;
    } else {
        // 计算使目标行成为可见区域最后一行的起始位置
        new_start_index = target_line_index - visible_lines_count_ + 1;
        // 确保不会超出范围
        if (new_start_index > max_start) {
            new_start_index = max_start;
        }
    }
    
    // 额外的边界检查，确保new_start_index不会为负数
    new_start_index = std::max(0, std::min(new_start_index, max_start));
    
    // 设置新的滚动位置
    scroll_bar_.set_display_start_index(new_start_index);
    update_scroll_display();
    
    // 添加调试日志
    UIManager::log_debug_static("set_visible_end_line: target=" + std::to_string(target_line_index) + 
                               ", new_start=" + std::to_string(new_start_index) + " - " + std::to_string(scroll_bar_.get_display_start_index()) + 
                               ", visible_count=" + std::to_string(visible_lines_count_));
}

void PageTemplate::set_progress(int line_index, float progress, const std::string& text) {
    if (line_index >= 0 && line_index < 4) {
        lines_[line_index].type = LineType::PROGRESS_BAR;
        // Note: progress value should be handled via progress_ptr in LineConfig
        lines_[line_index].text = text;
        lines_[line_index].color = COLOR_SUCCESS;
    }
}

void PageTemplate::show_status_indicator(int line_index, Color color, bool filled) {
    if (line_index >= 0 && line_index < 4 && graphics_engine_) {
        Rect line_rect = get_line_rect(line_index);
        int16_t indicator_size = 6;
        int16_t x = line_rect.x + line_rect.width - indicator_size - 4;
        int16_t y = line_rect.y + (line_rect.height - indicator_size) / 2;
        
        graphics_engine_->draw_status_indicator(x, y, indicator_size, color, filled);
    }
}

void PageTemplate::set_left_content(const std::vector<LineConfig>& left_lines) {
    left_lines_ = left_lines;
    if (left_lines_.size() > 4) {
        left_lines_.resize(4);
    }
}

void PageTemplate::set_right_content(const std::vector<LineConfig>& right_lines) {
    right_lines_ = right_lines;
    if (right_lines_.size() > 4) {
        right_lines_.resize(4);
    }
}

void PageTemplate::set_split_screen_content(const std::string& title,
                                           const std::vector<LineConfig>& left_lines,
                                           const std::vector<LineConfig>& right_lines,
                                           const std::string& left_header,
                                           const std::string& right_header) {
    set_title(title);
    set_left_content(left_lines);
    set_right_content(right_lines);
    set_split_headers(left_header, right_header);
    enable_split_screen(true);
    has_split_screen_ = true;
}

void PageTemplate::set_split_headers(const std::string& left_header, const std::string& right_header) {
    left_header_ = left_header;
    right_header_ = right_header;
}

void PageTemplate::set_split_ratio(float ratio) {
    if (ratio >= 0.2f && ratio <= 0.8f) {
        split_ratio_ = ratio;
    }
}

int16_t PageTemplate::get_line_y_position(int line_index) {
    if (line_index < 0 || line_index >= 5) return 0;
    return (has_title_ ? CONTENT_START_Y : LINE_SPACING) + line_index * (LINE_HEIGHT + LINE_SPACING);
}

Rect PageTemplate::get_line_rect(int line_index) {
    int16_t y = get_line_y_position(line_index);
    return Rect(0, y, LINE_WEIGHT, LINE_HEIGHT);
}

Rect PageTemplate::get_split_left_rect(int line_index) {
    int16_t y = get_line_y_position(line_index);
    int16_t divider_x = (int16_t)(128 * split_ratio_);
    return Rect(0, y, divider_x - 1, LINE_HEIGHT);
}

Rect PageTemplate::get_split_right_rect(int line_index) {
    int16_t y = get_line_y_position(line_index);
    int16_t divider_x = (int16_t)(128 * split_ratio_);
    return Rect(divider_x + 1, y, 128 - divider_x - 1, LINE_HEIGHT);
}

void PageTemplate::draw_title() {
    if (!graphics_engine_ || title_.empty()) return;
    
    Rect title_rect(0, TITLE_Y, LINE_WEIGHT, TITLE_HEIGHT);
    graphics_engine_->draw_chinese_text_aligned(title_.c_str(), title_rect, title_color_, 
                                               TextAlign::CENTER);
}

void PageTemplate::draw_line(int line_index, const LineConfig& config) {
    if (!graphics_engine_) return;
    
    switch (config.type) {
        case LineType::TEXT_ITEM:
            draw_text_item(line_index, config);
            break;
        case LineType::STATUS_LINE:
            draw_status_line(line_index, config);
            break;
        case LineType::MENU_JUMP:
            draw_menu_jump(line_index, config);
            break;
        case LineType::PROGRESS_BAR:
            draw_progress_bar(line_index, config);
            break;
        case LineType::INT_SETTING:
            draw_int_setting(line_index, config);
            break;
        case LineType::BUTTON_ITEM:
            draw_button_item(line_index, config);
            break;
        case LineType::BACK_ITEM:
            draw_back_item(line_index, config);
            break;
        case LineType::SELECTOR_ITEM:
            draw_selector_item(line_index, config);
            break;
    }
}

// 文本项渲染
void PageTemplate::draw_text_item(int line_index, const LineConfig& config) {
    if (!graphics_engine_ || config.text.empty()) return;
    
    Rect line_rect = get_line_rect(line_index);
    int16_t x = get_text_x_position(config.text, config.align, line_rect);
    int16_t y = line_rect.y + (line_rect.height - 14) / 2; // 固定14px字体高度
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), x, y, config.color);
}

// 状态行渲染
void PageTemplate::draw_status_line(int line_index, const LineConfig& config) {
    if (!graphics_engine_ || config.text.empty()) return;
    
    Rect line_rect = get_line_rect(line_index);
    int16_t x = get_text_x_position(config.text, config.align, line_rect);
    int16_t y = line_rect.y + (line_rect.height - 14) / 2;
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), x, y, config.color);
}

// 菜单跳转项渲染
void PageTemplate::draw_menu_jump(int line_index, const LineConfig& config) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
        draw_selection_indicator(line_index);
    }
    
    // 绘制文本
    int16_t text_x = line_rect.x + (config.selected ? SELECTION_INDICATOR_WIDTH + 4 : 8);
    int16_t text_y = line_rect.y + (line_rect.height - 14) / 2;
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), text_x, text_y, config.color);
}

// 进度条项渲染
void PageTemplate::draw_progress_bar(int line_index, const LineConfig& config) {
    if (!graphics_engine_ || !config.data.progress.progress_ptr) return;
    
    Rect line_rect = get_line_rect(line_index);
    uint8_t progress_value = *config.data.progress.progress_ptr;
    float progress = progress_value / 255.0f; // 转换0-255到0.0-1.0
    
    // 确保progress值在有效范围内
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    
    // 计算百分比文本
    char percent_str[8];
    int progress_percent = (int)(progress * 100.0f); // 转换为整数避免浮点数问题
    snprintf(percent_str, sizeof(percent_str), "%d%%", progress_percent);
    int16_t percent_width = graphics_engine_->get_text_width(percent_str);
    
    // 为百分比文本预留空间，进度条宽度减去百分比文本宽度和间距
    int16_t text_margin = 6; // 百分比文本与进度条的间距
    int16_t progress_width = line_rect.width - percent_width - text_margin - 8; // 8是左右边距
    
    // 绘制进度条（带外边框，高度增加）
    Rect progress_rect(line_rect.x + 4, line_rect.y + 1, progress_width, 8);
    graphics_engine_->draw_progress_bar(progress_rect, progress, COLOR_BG_CARD, config.color);
    
    // 绘制百分比文本，始终在右侧
    int16_t percent_x = line_rect.x + line_rect.width - percent_width - 4;
    int16_t percent_y = line_rect.y + 6; // 垂直居中

    graphics_engine_->draw_text(percent_str, percent_x, percent_y, COLOR_TEXT_GRAY);
    
}

// INT设置项渲染
void PageTemplate::draw_int_setting(int line_index, const LineConfig& config) {
    if (!graphics_engine_ || !config.data.int_setting.int_value_ptr) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
    }
    
    // 第二行：标题居中显示
    if (line_index == 1 && !config.setting_title.empty()) {
        int16_t title_x = get_text_x_position(config.setting_title, LineAlign::CENTER, line_rect);
        int16_t title_y = line_rect.y + (line_rect.height - 14) / 2;
        graphics_engine_->draw_chinese_text(config.setting_title.c_str(), title_x, title_y, config.color);
    }
    
    // 第四行：当前值动态显示
    if (line_index == 3) {
        char value_str[16];
        snprintf(value_str, sizeof(value_str), "%ld", (long)*config.data.int_setting.int_value_ptr);
        int16_t value_x = get_text_x_position(value_str, LineAlign::CENTER, line_rect);
        int16_t value_y = line_rect.y + (line_rect.height - 14) / 2;
        graphics_engine_->draw_text(value_str, value_x, value_y, config.color);
    }
    
    // 第五行："{min} - {max}"居中显示
    if (line_index == 4) {
        char range_str[32];
        snprintf(range_str, sizeof(range_str), "%ld - %ld", (long)config.data.int_setting.min_value, (long)config.data.int_setting.max_value);
        int16_t range_x = get_text_x_position(range_str, LineAlign::CENTER, line_rect);
        int16_t range_y = line_rect.y + (line_rect.height - 14) / 2;
        graphics_engine_->draw_text(range_str, range_x, range_y, COLOR_TEXT_GRAY);
    }
}

// 按钮项渲染
void PageTemplate::draw_button_item(int line_index, const LineConfig& config) {
    if (!graphics_engine_ || config.text.empty()) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
        // 绘制按钮边框
        graphics_engine_->draw_rect(line_rect, config.color);
    }
    
    // 绘制按钮文本
    int16_t text_x = get_text_x_position(config.text, config.align, line_rect);
    int16_t text_y = line_rect.y + (line_rect.height - 14) / 2;
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), text_x, text_y, config.color);
}

// 返回项渲染
void PageTemplate::draw_back_item(int line_index, const LineConfig& config) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
    }
    
    // 绘制返回箭头符号
    int16_t arrow_x = line_rect.x + 2;
    int16_t arrow_y = line_rect.y + (line_rect.height - 8) / 2;
    graphics_engine_->draw_chinese_text("←", arrow_x, arrow_y, config.color);
    
    // 绘制文本（在箭头右侧）
    int16_t text_x = arrow_x + 12; // 箭头宽度 + 间距
    int16_t text_y = line_rect.y + (line_rect.height - 14) / 2;
    graphics_engine_->draw_chinese_text(config.text.c_str(), text_x, text_y, config.color);
    
    // 绘制选中指示器
    if (config.selected) {
        draw_selection_indicator(line_index);
    }
}

// 选择器项渲染
void PageTemplate::draw_selector_item(int line_index, const LineConfig& config) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
        draw_selection_indicator(line_index);
    }
    
    // 绘制锁定状态指示器
    if (config.data.selector.is_locked) {
        // 绘制锁定图标或符号
        int16_t lock_x = line_rect.x + line_rect.width - 16;
        int16_t lock_y = line_rect.y + (line_rect.height - 8) / 2;
        graphics_engine_->draw_chinese_text("🔒", lock_x, lock_y, COLOR_PRIMARY);
    }
    
    // 绘制选择器文本
    int16_t text_x = line_rect.x + (config.selected ? SELECTION_INDICATOR_WIDTH + 4 : 8);
    int16_t text_y = line_rect.y + (line_rect.height - 14) / 2;
    
    // 根据锁定状态调整文本颜色
    Color text_color = config.data.selector.is_locked ? COLOR_PRIMARY : config.color;
    graphics_engine_->draw_chinese_text(config.text.c_str(), text_x, text_y, text_color);
}

void PageTemplate::draw_split_screen() {
    if (!graphics_engine_) return;
    
    // 绘制标题
    draw_title();
    
    int16_t divider_x = (int16_t)(SCREEN_WIDTH * split_ratio_);
    int16_t content_height = 4 * (LINE_HEIGHT + LINE_SPACING);
    
    // 绘制边框（如果启用）
    if (split_borders_enabled_) {
        // 绘制外边框
        graphics_engine_->draw_rect(Rect(0, CONTENT_START_Y, 128, content_height), COLOR_BORDER);
        
        // 绘制分割线
        graphics_engine_->draw_vline(divider_x, CONTENT_START_Y, content_height, COLOR_BORDER);
    } else {
        // 只绘制分割线
        graphics_engine_->draw_vline(divider_x, CONTENT_START_Y, content_height, COLOR_BORDER);
    }
    
    // 绘制左侧标题（如果有）
    if (!left_header_.empty()) {
        Rect left_header_rect(2, CONTENT_START_Y - 12, divider_x - 4, 10);
        graphics_engine_->draw_chinese_text_aligned(left_header_.c_str(), left_header_rect, 
                                                   COLOR_TEXT_GRAY, TextAlign::CENTER);
    }
    
    // 绘制右侧标题（如果有）
    if (!right_header_.empty()) {
        Rect right_header_rect(divider_x + 2, CONTENT_START_Y - 12, 128 - divider_x - 4, 10);
        graphics_engine_->draw_chinese_text_aligned(right_header_.c_str(), right_header_rect, 
                                                   COLOR_TEXT_GRAY, TextAlign::CENTER);
    }
    
    // 绘制左侧内容
    for (size_t i = 0; i < left_lines_.size() && i < 4; i++) {
        if (!left_lines_[i].text.empty()) {
            Rect left_rect = get_split_left_rect(i);
            // 添加内边距
            left_rect.x += 2;
            left_rect.width -= 4;
            
            // 使用graphics_engine的对齐功能
            TextAlign text_align = TextAlign::LEFT;
            switch (left_lines_[i].align) {
                case LineAlign::CENTER: text_align = TextAlign::CENTER; break;
                case LineAlign::RIGHT: text_align = TextAlign::RIGHT; break;
                case LineAlign::LEFT: default: text_align = TextAlign::LEFT; break;
            }
            
            graphics_engine_->draw_chinese_text_aligned(left_lines_[i].text.c_str(), left_rect, 
                                                       left_lines_[i].color, text_align);
        }
    }
    
    // 绘制右侧内容
    for (size_t i = 0; i < right_lines_.size() && i < 4; i++) {
        if (!right_lines_[i].text.empty()) {
            Rect right_rect = get_split_right_rect(i);
            // 添加内边距
            right_rect.x += 2;
            right_rect.width -= 4;
            
            // 使用graphics_engine的对齐功能
            TextAlign text_align = TextAlign::LEFT;
            switch (right_lines_[i].align) {
                case LineAlign::CENTER: text_align = TextAlign::CENTER; break;
                case LineAlign::RIGHT: text_align = TextAlign::RIGHT; break;
                case LineAlign::LEFT: default: text_align = TextAlign::LEFT; break;
            }
            
            graphics_engine_->draw_chinese_text_aligned(right_lines_[i].text.c_str(), right_rect, 
                                                       right_lines_[i].color, text_align);
        }
    }
}

int16_t PageTemplate::get_text_x_position(const std::string& text, 
                                         LineAlign align, const Rect& rect) {
    if (!graphics_engine_) return rect.x;
    
    int16_t text_width = graphics_engine_->get_chinese_text_width(text.c_str());
    
    switch (align) {
        case LineAlign::CENTER:
            return rect.x + (rect.width - text_width) / 2;
        case LineAlign::RIGHT:
            return rect.x + rect.width - text_width - 4;
        case LineAlign::LEFT:
        default:
            return rect.x + 4;
    }
}

void PageTemplate::draw_selection_indicator(int line_index) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    int16_t indicator_x = line_rect.x + 2;
    int16_t indicator_y = line_rect.y + line_rect.height / 2;
    
    graphics_engine_->draw_icon_arrow_right(indicator_x, indicator_y - 3, 6, COLOR_PRIMARY);
}

// 预定义模板实现
namespace PageTemplates {


void setup_error_page(PageTemplate& page, const std::string& error_message, const std::string& action_hint) {
    page.clear();
    page.set_title("错误", COLOR_ERROR);
    
    // 错误信息
    LineConfig error_config(error_message, COLOR_ERROR, LineAlign::CENTER);
    page.set_line(0, error_config);
    
    // 操作提示
    LineConfig hint_config = LineConfig::create_status_line(action_hint, COLOR_TEXT_WHITE, LineAlign::CENTER);
    page.set_line(2, hint_config);
}

// 保留错误页面创建函数，其他页面应在page目录中实现
void create_error_page(PageTemplate& page) {
    setup_error_page(page, "系统错误", "按任意键返回");
}

// INT设置页面模板
void setup_int_setting_page(PageTemplate& page, const std::string& title, 
                           int32_t* value_ptr, int32_t min_val, int32_t max_val,
                           std::function<void(int32_t)> change_cb,
                           std::function<void()> complete_cb) {
    page.clear();
    page.set_title(title, COLOR_WHITE);
    
    // 第一行：空行
    LineConfig empty_line("", COLOR_TEXT_WHITE, LineAlign::CENTER);
    page.set_line(0, empty_line);
    
    // 第二行：设置标题
    LineConfig title_line = LineConfig::create_int_setting(value_ptr, min_val, max_val, 
                                                          "", title, change_cb, complete_cb, COLOR_TEXT_WHITE);
    page.set_line(1, title_line);
    
    // 第三行：操作提示
    LineConfig hint_line("← → 调整值", COLOR_TEXT_GRAY, LineAlign::CENTER);
    page.set_line(2, hint_line);
    
    // 第四行：当前值显示
    LineConfig value_line = LineConfig::create_int_setting(value_ptr, min_val, max_val, 
                                                          "", title, change_cb, complete_cb, COLOR_TEXT_WHITE);
    value_line.selected = true;  // 标记为选中状态
    page.set_line(3, value_line);
    
    // 第五行：范围显示
    LineConfig range_line = LineConfig::create_int_setting(value_ptr, min_val, max_val, 
                                                          "", title, change_cb, complete_cb, COLOR_TEXT_GRAY);
    page.set_line(4, range_line);
    
    // 设置选中索引为值显示行
    page.set_selected_index(3);
}

} // namespace PageTemplates