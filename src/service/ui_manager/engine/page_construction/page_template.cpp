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
    , jump_str(other.jump_str)
    , callback_type(other.callback_type) {
    // 复制类型特定数据（union）
    data = other.data;

    // 根据callback_type复制union中的回调数据
    switch (callback_type) {
        case CallbackType::VALUE_CHANGE:
            callback_data.value_change_callback = other.callback_data.value_change_callback;
            break;
        case CallbackType::CLICK:
            callback_data.click_callback = other.callback_data.click_callback;
            break;
        case CallbackType::SELECTOR:
            callback_data.selector_callback = other.callback_data.selector_callback;
            break;
        case CallbackType::COMPLETE:
        case CallbackType::NONE:
        default:
            // 对于NONE类型，不需要复制任何数据
            break;
    }
    
    // 复制独立的回调函数
    lock_callback = other.lock_callback;
    int_complete_callback = other.int_complete_callback;
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
        jump_str = other.jump_str;
        callback_type = other.callback_type;

        // 复制类型特定数据（union）
        data = other.data;
        
        // 根据callback_type复制union中的回调数据
        switch (callback_type) {
            case CallbackType::VALUE_CHANGE:
                callback_data.value_change_callback = other.callback_data.value_change_callback;
                break;
            case CallbackType::CLICK:
                callback_data.click_callback = other.callback_data.click_callback;
                break;
            case CallbackType::SELECTOR:
                callback_data.selector_callback = other.callback_data.selector_callback;
                break;
            case CallbackType::COMPLETE:
            case CallbackType::NONE:
            default:
                // 对于NONE类型，不需要复制任何数据
                break;
        }
        
        // 复制独立的回调函数
        lock_callback = other.lock_callback;
        int_complete_callback = other.int_complete_callback;
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
    , jump_str(std::move(other.jump_str))
    , callback_type(other.callback_type) {
    // 移动类型特定数据（union）
    data = other.data;
    
    // 根据callback_type移动union中的回调数据
    switch (callback_type) {
        case CallbackType::VALUE_CHANGE:
            callback_data.value_change_callback = std::move(other.callback_data.value_change_callback);
            break;
        case CallbackType::CLICK:
            callback_data.click_callback = std::move(other.callback_data.click_callback);
            break;
        case CallbackType::SELECTOR:
            callback_data.selector_callback = std::move(other.callback_data.selector_callback);
            break;
        case CallbackType::COMPLETE:
        case CallbackType::NONE:
        default:
            break;
    }
    
    // 移动独立的回调函数
    lock_callback = std::move(other.lock_callback);
    int_complete_callback = std::move(other.int_complete_callback);
    
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
        jump_str = std::move(other.jump_str);
        callback_type = other.callback_type;
        
        // 移动类型特定数据（union）
        data = other.data;
        
        // 根据callback_type移动union中的回调数据
        switch (callback_type) {
            case CallbackType::VALUE_CHANGE:
                callback_data.value_change_callback = std::move(other.callback_data.value_change_callback);
                break;
            case CallbackType::CLICK:
                callback_data.click_callback = std::move(other.callback_data.click_callback);
                break;
            case CallbackType::SELECTOR:
                callback_data.selector_callback = std::move(other.callback_data.selector_callback);
                break;
            case CallbackType::COMPLETE:
            case CallbackType::NONE:
            default:
                break;
        }
        
        // 移动独立的回调函数
        lock_callback = std::move(other.lock_callback);
        int_complete_callback = std::move(other.int_complete_callback);
        
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
    , all_lines_()  // 所有行内容（统一管理）
    , visible_lines_count_(5)
    , selected_menu_index_(0)
    , scroll_bar_()  // 使用默认构造函数
    , scroll_enabled_(false)
    , split_screen_enabled_(false)
    , left_header_("")
    , right_header_("")
    , split_borders_enabled_(true)
    , split_ratio_(0.5f) {
    
    // 初始化all_lines_容器大小
    all_lines_.resize(visible_lines_count_);
    
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
    // 重置状态跟踪变量
    has_title_ = false;
    has_split_screen_ = false;
    
    // 使用单个静态容量变量依次处理所有容器
    static size_t capacity_buffer;
    
    // 清空标题但保留内存容量
    capacity_buffer = title_.capacity();
    title_.clear();
    title_.reserve(capacity_buffer);
    title_color_ = COLOR_WHITE;
    
    // 清空分屏相关但保留内存容量
    split_screen_enabled_ = false;
    
    // 依次处理vector容器，复用capacity_buffer
    capacity_buffer = left_lines_.capacity();
    left_lines_.clear();
    left_lines_.reserve(capacity_buffer);
    
    capacity_buffer = right_lines_.capacity();
    right_lines_.clear();
    right_lines_.reserve(capacity_buffer);
    
    capacity_buffer = left_header_.capacity();
    left_header_.clear();
    left_header_.reserve(capacity_buffer);
    
    capacity_buffer = right_header_.capacity();
    right_header_.clear();
    right_header_.reserve(capacity_buffer);
    
    // 对于all_lines_，由于在构造函数中已经resize过，这里只需要重置内容
    // 不清空all_lines_以保持已分配的内存，只重置每个元素为默认状态
    for (auto& line : all_lines_) {
        // 依次保存和恢复字符串容量，复用capacity_buffer
        capacity_buffer = line.text.capacity();
        line = LineConfig();  // 重置为默认状态
        line.text.reserve(capacity_buffer);
        
        capacity_buffer = line.setting_title.capacity();
        line.setting_title.reserve(capacity_buffer);
        
        capacity_buffer = line.target_page_name.capacity();
        line.target_page_name.reserve(capacity_buffer);
    }
}

void PageTemplate::set_title(const std::string& title, Color color) {
    title_ = title;
    title_color_ = color;
    has_title_ = !title.empty();
    // 更新可见行数缓存
    visible_lines_count_ = has_title_ ? 4 : 5;
}

void PageTemplate::set_line(int32_t line_index, const LineConfig& config) {
    // 统一使用all_lines_管理所有行内容
    if (line_index >= 0 && line_index < (int)all_lines_.size()) {
        all_lines_[line_index] = config;
    }
}

void PageTemplate::set_lines(const std::vector<LineConfig>& lines) {
    // 直接设置all_lines_，避免递归调用set_all_lines
    all_lines_.clear();
    all_lines_.resize(visible_lines_count_);
    
    // 复制传入的行到all_lines_数组
    for (size_t i = 0; i < lines.size() && i < visible_lines_count_; ++i) {
        all_lines_[i] = lines[i];
    }
}

void PageTemplate::set_all_lines(const std::vector<LineConfig>& lines) {
    // 根据是否有标题动态设置可见行数
    visible_lines_count_ = has_title_ ? 4 : 5;
    
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
    } else {
        scroll_enabled_ = false;
        // 确保all_lines_大小符合可见行数
        if (all_lines_.size() < visible_lines_count_) {
            all_lines_.resize(visible_lines_count_);
        }
        // 重置选中索引
        selected_menu_index_ = 0;
    }
}

void PageTemplate::clear() {
    title_.clear();
    title_color_ = COLOR_WHITE;
    for (auto& line : all_lines_) {
        line.text.clear();
        line.type = LineType::TEXT_ITEM;
        line.color = COLOR_TEXT_WHITE;
        line.selected = false;
        // Union数据会通过构造函数自动初始化为0
    }
    selected_menu_index_ = 0;
}

void PageTemplate::clear_line(int32_t line_index) {
    if (line_index >= 0 && line_index < (int)all_lines_.size()) {
        all_lines_[line_index].text.clear();
        all_lines_[line_index].type = LineType::TEXT_ITEM;
        all_lines_[line_index].selected = false;
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
        for (int32_t i = 0; i < visible_lines_count_; i++) {
            int32_t actual_index = scroll_enabled_ ? scroll_bar_.get_display_start_index() + i : i;
            if (actual_index < (int)all_lines_.size()) {
                // 对于进度条，即使text为空也要绘制
                if (all_lines_[actual_index].type == LineType::PROGRESS_BAR || !all_lines_[actual_index].text.empty()) {
                    draw_line(i, all_lines_[actual_index]);
                }
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

void PageTemplate::set_selected_index(int32_t index) {
    // 清除所有选中状态
    for (auto& line : all_lines_) {
        line.selected = false;
    }
    
    // 设置新的选中状态
    if (index >= 0 && index < (int)all_lines_.size()) {
        selected_menu_index_ = index;
        all_lines_[index].selected = true;
    }
}

bool PageTemplate::scroll_up() {
    if (!scroll_enabled_) {
        return false;
    }
    
    bool scrolled = scroll_bar_.scroll_up();
    if (scrolled) {
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
        // 添加调试日志
        UIManager::log_debug_static("ScrollBar: DOWN scrolled, start_index=" + std::to_string(scroll_bar_.get_display_start_index()));
    }
    return scrolled;
}

void PageTemplate::set_visible_end_line(int32_t target_line_index) {
    if (!scroll_enabled_ || all_lines_.empty()) {
        return;
    }
    
    // 确保目标行索引在有效范围内
    if (target_line_index < 0 || target_line_index >= (int)all_lines_.size()) {
        return;
    }
    
    // 计算新的滚动起始位置
    int32_t new_start_index;
    int32_t max_start = MAX(0, all_lines_.size() - visible_lines_count_);
    
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
    new_start_index = MAX(0, MIN(new_start_index, max_start));
    
    // 设置新的滚动位置
    scroll_bar_.set_display_start_index(new_start_index);

    // 添加调试日志
    UIManager::log_debug_static("set_visible_end_line: target=" + std::to_string(target_line_index) + 
                               ", new_start=" + std::to_string(new_start_index) + " - " + std::to_string(scroll_bar_.get_display_start_index()) + 
                               ", visible_count=" + std::to_string(visible_lines_count_));
}

void PageTemplate::set_progress(int32_t line_index, float progress, const std::string& text) {
    if (line_index >= 0 && line_index < (int)all_lines_.size()) {
        all_lines_[line_index].type = LineType::PROGRESS_BAR;
        // Note: progress value should be handled via progress_ptr in LineConfig
        all_lines_[line_index].text = text;
        all_lines_[line_index].color = COLOR_SUCCESS;
    }
}

void PageTemplate::show_status_indicator(int32_t line_index, Color color, bool filled) {
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

int16_t PageTemplate::get_line_y_position(int32_t line_index) {
    if (line_index < 0 || line_index >= 5) return 0;
    return (has_title_ ? CONTENT_START_Y : LINE_SPACING) + line_index * (LINE_HEIGHT + LINE_SPACING);
}

Rect PageTemplate::get_line_rect(int32_t line_index) {
    int16_t y = get_line_y_position(line_index);
    return Rect(0, y, LINE_WEIGHT, LINE_HEIGHT);
}

Rect PageTemplate::get_split_left_rect(int32_t line_index) {
    int16_t y = get_line_y_position(line_index);
    int16_t divider_x = (int16_t)(128 * split_ratio_);
    return Rect(0, y, divider_x - 1, LINE_HEIGHT);
}

Rect PageTemplate::get_split_right_rect(int32_t line_index) {
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

void PageTemplate::draw_line(int32_t line_index, const LineConfig& config) {
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
void PageTemplate::draw_text_item(int32_t line_index, const LineConfig& config) {
    if (!graphics_engine_ || config.text.empty()) return;
    
    Rect line_rect = get_line_rect(line_index);
    int16_t x = get_text_x_position(config.text, config.align, line_rect);
    int16_t y = line_rect.y + (line_rect.height - 14) / 2; // 固定14px字体高度
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), x, y, config.color);
}

// 状态行渲染
void PageTemplate::draw_status_line(int32_t line_index, const LineConfig& config) {
    if (!graphics_engine_ || config.text.empty()) return;
    
    Rect line_rect = get_line_rect(line_index);
    int16_t x = get_text_x_position(config.text, config.align, line_rect);
    int16_t y = line_rect.y + (line_rect.height - 14) / 2;
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), x, y, config.color);
}

// 菜单跳转项渲染
void PageTemplate::draw_menu_jump(int32_t line_index, const LineConfig& config) {
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
void PageTemplate::draw_progress_bar(int32_t line_index, const LineConfig& config) {
    if (!graphics_engine_) {
        return;
    }
    
    Rect line_rect = get_line_rect(line_index);
    
    // 获取进度值，检查指针是否有效
    uint8_t progress_value = 0;
    bool has_valid_data = false;
    if (config.data.progress.progress_ptr) {
        progress_value = *config.data.progress.progress_ptr;
        has_valid_data = true;
    }
    
    // 计算百分比文本或错误信息
    char display_str[8];
    if (has_valid_data) {
        // 使用整数运算计算百分比: percentage = progress_value * 100 / 255
        int32_t progress_percent = (progress_value * 100) / 255;
        snprintf(display_str, sizeof(display_str), "%ld%%", progress_percent);
    } else {
        snprintf(display_str, sizeof(display_str), "ERR");
    }
    int16_t text_width = graphics_engine_->get_text_width(display_str);
    
    // 为文本预留空间，进度条宽度减去文本宽度和间距
    int16_t text_margin = 6; // 文本与进度条的间距
    int16_t progress_width = line_rect.width - text_width - text_margin - 8; // 8是左右边距
    
    // 确保进度条宽度不会太小
    if (progress_width < 20) {
        progress_width = 20;
    }
    
    uint8_t progress_for_engine = has_valid_data ? progress_value : 0;
    
    // 绘制进度条（高度14，在行内垂直居中）- 始终绘制边框和背景
    // 行高12，进度条高度14，所以Y偏移 = (12-14)/2 = -1，使进度条在行内居中
    Rect progress_rect(line_rect.x + 4, line_rect.y - 1, progress_width, 14);
    
    graphics_engine_->draw_progress_bar(progress_rect, progress_for_engine, COLOR_BG_CARD, config.color);
    
    // 绘制文本，始终在右侧显示
    int16_t text_x = line_rect.x + line_rect.width - text_width - 4;
    // 计算文字Y坐标，使其与进度条中线对齐
    // 进度条: y=line_rect.y - 1, height=14, 中线=line_rect.y - 1 + 7 = line_rect.y + 6
    // 文字高度14，所以文字Y坐标应该是中线位置
    int16_t text_y = line_rect.y - 2;

    graphics_engine_->draw_text(display_str, text_x, text_y, COLOR_WHITE);
}

// INT设置项渲染
void PageTemplate::draw_int_setting(int32_t line_index, const LineConfig& config) {
    if (!graphics_engine_ || !config.data.int_setting.int_value_ptr) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
    }
    
    // 构建显示文本："标题: 当前值 (min-max)"
    char display_str[64];
    if (!config.setting_title.empty()) {
        snprintf(display_str, sizeof(display_str), "%s: %ld (%ld-%ld)", 
                config.setting_title.c_str(),
                (long)*config.data.int_setting.int_value_ptr,
                (long)config.data.int_setting.min_value,
                (long)config.data.int_setting.max_value);
    } else {
        snprintf(display_str, sizeof(display_str), "%ld (%ld-%ld)", 
                (long)*config.data.int_setting.int_value_ptr,
                (long)config.data.int_setting.min_value,
                (long)config.data.int_setting.max_value);
    }
    
    // 绘制文本
    int16_t text_x = get_text_x_position(display_str, config.align, line_rect);
    int16_t text_y = line_rect.y + (line_rect.height - 14) / 2;
    graphics_engine_->draw_text(display_str, text_x, text_y, config.color);
}

// 按钮项渲染
void PageTemplate::draw_button_item(int32_t line_index, const LineConfig& config) {
    if (!graphics_engine_ || config.text.empty()) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
        // 绘制按钮边框
        graphics_engine_->draw_rect(line_rect, config.color);
        // 绘制选中指示器
        draw_selection_indicator(line_index);
    }
    
    // 绘制按钮文本 - 与菜单项保持一致的对齐方式
    int16_t text_x = line_rect.x + (config.selected ? SELECTION_INDICATOR_WIDTH + 4 : 8);
    int16_t text_y = line_rect.y + (line_rect.height - 14) / 2;
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), text_x, text_y, config.color);
}

// 返回项渲染
void PageTemplate::draw_back_item(int32_t line_index, const LineConfig& config) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
    }
    
    // 绘制返回箭头符号
    int16_t arrow_x = line_rect.x + 2;
    int16_t arrow_y = line_rect.y + (line_rect.height - 8) / 2;
    graphics_engine_->draw_chinese_text("<<", arrow_x, arrow_y, config.color);
    
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
void PageTemplate::draw_selector_item(int32_t line_index, const LineConfig& config) {
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

void PageTemplate::draw_selection_indicator(int32_t line_index) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    int16_t indicator_x = line_rect.x + 2;
    int16_t indicator_y = line_rect.y + line_rect.height / 2;
    
    graphics_engine_->draw_icon_arrow_right(indicator_x, indicator_y - 3, 6, COLOR_PRIMARY);
}
