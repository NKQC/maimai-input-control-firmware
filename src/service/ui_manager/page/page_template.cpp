#include "page_template.h"
#include <algorithm>
#include <cstdio>

// 页面布局常量
static const int16_t TITLE_Y = 2;
static const int16_t TITLE_HEIGHT = 16;
static const int16_t LINE_HEIGHT = 12;
static const int16_t LINE_SPACING = 2;
static const int16_t CONTENT_START_Y = TITLE_Y + TITLE_HEIGHT + LINE_SPACING;
static const int16_t SELECTION_INDICATOR_WIDTH = 8;
static const int16_t SPLIT_SCREEN_DIVIDER = SCREEN_WIDTH / 2;

PageTemplate::PageTemplate(GraphicsEngine* graphics_engine)
    : graphics_engine_(graphics_engine)
    , title_("")
    , title_color_(COLOR_WHITE)
    , lines_(4)  // 初始化4行内容
    , selected_menu_index_(0)
    , split_screen_enabled_(false)
    , left_header_("")
    , right_header_("")
    , split_borders_enabled_(true)
    , split_ratio_(0.5f) {
}

void PageTemplate::set_title(const std::string& title, Color color) {
    title_ = title;
    title_color_ = color;
}

void PageTemplate::set_line(int line_index, const LineConfig& config) {
    if (line_index >= 0 && line_index < 4) {
        lines_[line_index] = config;
    }
}

void PageTemplate::set_lines(const std::vector<LineConfig>& lines) {
    for (size_t i = 0; i < std::min(lines.size(), (size_t)4); i++) {
        lines_[i] = lines[i];
    }
}

void PageTemplate::clear() {
    title_.clear();
    title_color_ = COLOR_WHITE;
    for (auto& line : lines_) {
        line.text.clear();
        line.type = LineType::CONTENT;
        line.color = COLOR_TEXT_WHITE;
        line.selected = false;
        line.progress = 0.0f;
    }
    menu_items_.clear();
    selected_menu_index_ = 0;
}

void PageTemplate::clear_line(int line_index) {
    if (line_index >= 0 && line_index < 4) {
        lines_[line_index].text.clear();
        lines_[line_index].selected = false;
        lines_[line_index].progress = 0.0f;
    }
}

void PageTemplate::draw() {
    if (!graphics_engine_) return;
    
    // 绘制背景
    draw_background();
    
    if (split_screen_enabled_) {
        draw_split_screen();
    } else {
        // 绘制标题
        draw_title();
        
        // 绘制内容行
        for (int i = 0; i < 4; i++) {
            if (!lines_[i].text.empty()) {
                draw_line(i, lines_[i]);
            }
        }
    }
}

void PageTemplate::draw_background(Color bg_color) {
    if (graphics_engine_) {
        graphics_engine_->clear(bg_color);
    }
}

void PageTemplate::set_menu_items(const std::vector<std::string>& items, int selected_index) {
    menu_items_ = items;
    selected_menu_index_ = std::max(0, std::min(selected_index, (int)items.size() - 1));
    
    // 更新行配置为菜单项
    for (size_t i = 0; i < std::min(items.size(), (size_t)4); i++) {
        lines_[i].type = LineType::MENU_ITEM;
        lines_[i].text = items[i];
        lines_[i].selected = (i == selected_menu_index_);
        lines_[i].color = lines_[i].selected ? COLOR_PRIMARY : COLOR_TEXT_WHITE;
        lines_[i].font_size = FontSize::MEDIUM;
        lines_[i].align = LineAlign::LEFT;
    }
}

void PageTemplate::set_selected_index(int index) {
    if (index >= 0 && index < (int)menu_items_.size()) {
        // 清除旧选中状态
        if (selected_menu_index_ < 4) {
            lines_[selected_menu_index_].selected = false;
            lines_[selected_menu_index_].color = COLOR_TEXT_WHITE;
        }
        
        // 设置新选中状态
        selected_menu_index_ = index;
        if (index < 4) {
            lines_[index].selected = true;
            lines_[index].color = COLOR_PRIMARY;
        }
    }
}

void PageTemplate::set_progress(int line_index, float progress, const std::string& text) {
    if (line_index >= 0 && line_index < 4) {
        lines_[line_index].type = LineType::PROGRESS;
        lines_[line_index].progress = std::max(0.0f, std::min(1.0f, progress));
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
    if (line_index < 0 || line_index >= 4) return 0;
    return CONTENT_START_Y + line_index * (LINE_HEIGHT + LINE_SPACING);
}

Rect PageTemplate::get_line_rect(int line_index) {
    int16_t y = get_line_y_position(line_index);
    return Rect(0, y, SCREEN_WIDTH, LINE_HEIGHT);
}

Rect PageTemplate::get_split_left_rect(int line_index) {
    int16_t y = get_line_y_position(line_index);
    int16_t divider_x = (int16_t)(SCREEN_WIDTH * split_ratio_);
    return Rect(0, y, divider_x - 1, LINE_HEIGHT);
}

Rect PageTemplate::get_split_right_rect(int line_index) {
    int16_t y = get_line_y_position(line_index);
    int16_t divider_x = (int16_t)(SCREEN_WIDTH * split_ratio_);
    return Rect(divider_x + 1, y, SCREEN_WIDTH - divider_x - 1, LINE_HEIGHT);
}

void PageTemplate::draw_title() {
    if (!graphics_engine_ || title_.empty()) return;
    
    Rect title_rect(0, TITLE_Y, SCREEN_WIDTH, TITLE_HEIGHT);
    graphics_engine_->draw_text_aligned(title_.c_str(), title_rect, title_color_, 
                                       TextAlign::CENTER, FontSize::LARGE);
}

void PageTemplate::draw_line(int line_index, const LineConfig& config) {
    if (!graphics_engine_ || config.text.empty()) return;
    
    switch (config.type) {
        case LineType::MENU_ITEM:
            draw_menu_line(line_index, config);
            break;
        case LineType::PROGRESS:
            draw_progress_line(line_index, config);
            break;
        default:
            {
                Rect line_rect = get_line_rect(line_index);
                int16_t x = get_text_x_position(config.text, config.font_size, config.align, line_rect);
                int16_t y = line_rect.y + (line_rect.height - graphics_engine_->get_font_height(config.font_size)) / 2;
                
                graphics_engine_->draw_chinese_text(config.text.c_str(), x, y, config.color, config.font_size);
            }
            break;
    }
}

void PageTemplate::draw_menu_line(int line_index, const LineConfig& config) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制选中背景
    if (config.selected) {
        graphics_engine_->fill_rect(line_rect, COLOR_BG_CARD);
        draw_selection_indicator(line_index);
    }
    
    // 绘制文本
    int16_t text_x = line_rect.x + (config.selected ? SELECTION_INDICATOR_WIDTH + 4 : 8);
    int16_t text_y = line_rect.y + (line_rect.height - graphics_engine_->get_font_height(config.font_size)) / 2;
    
    graphics_engine_->draw_chinese_text(config.text.c_str(), text_x, text_y, config.color, config.font_size);
}

void PageTemplate::draw_progress_line(int line_index, const LineConfig& config) {
    if (!graphics_engine_) return;
    
    Rect line_rect = get_line_rect(line_index);
    
    // 绘制进度条
    Rect progress_rect(line_rect.x + 4, line_rect.y + 2, line_rect.width - 8, 6);
    graphics_engine_->draw_progress_bar(progress_rect, config.progress, COLOR_BG_CARD, config.color);
    
    // 绘制进度文本
    if (!config.text.empty()) {
        int16_t text_y = line_rect.y + 8;
        graphics_engine_->draw_chinese_text(config.text.c_str(), line_rect.x + 4, text_y, 
                                           COLOR_TEXT_WHITE, FontSize::SMALL);
    }
    
    // 绘制百分比
    char percent_str[8];
    snprintf(percent_str, sizeof(percent_str), "%.0f%%", config.progress * 100);
    int16_t percent_width = graphics_engine_->get_text_width(percent_str, FontSize::SMALL);
    int16_t percent_x = line_rect.x + line_rect.width - percent_width - 4;
    int16_t percent_y = line_rect.y + 8;
    graphics_engine_->draw_text(percent_str, percent_x, percent_y, COLOR_TEXT_GRAY, FontSize::SMALL);
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
        graphics_engine_->draw_rect(Rect(0, CONTENT_START_Y, SCREEN_WIDTH, content_height), COLOR_BORDER);
        
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
                                                   COLOR_TEXT_GRAY, TextAlign::CENTER, FontSize::SMALL);
    }
    
    // 绘制右侧标题（如果有）
    if (!right_header_.empty()) {
        Rect right_header_rect(divider_x + 2, CONTENT_START_Y - 12, SCREEN_WIDTH - divider_x - 4, 10);
        graphics_engine_->draw_chinese_text_aligned(right_header_.c_str(), right_header_rect, 
                                                   COLOR_TEXT_GRAY, TextAlign::CENTER, FontSize::SMALL);
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
                                                       left_lines_[i].color, text_align, left_lines_[i].font_size);
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
                                                       right_lines_[i].color, text_align, right_lines_[i].font_size);
        }
    }
}

int16_t PageTemplate::get_text_x_position(const std::string& text, FontSize font_size, 
                                         LineAlign align, const Rect& rect) {
    if (!graphics_engine_) return rect.x;
    
    int16_t text_width = graphics_engine_->get_chinese_text_width(text.c_str(), font_size);
    
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
    
void setup_main_menu(PageTemplate& page, const std::vector<std::string>& menu_items, int selected_index) {
    page.clear();
    page.set_title("主菜单", COLOR_WHITE);
    page.set_menu_items(menu_items, selected_index);
}

void setup_status_page(PageTemplate& page, const std::string& title, const std::vector<std::string>& status_items) {
    page.clear();
    page.set_title(title, COLOR_WHITE);
    
    for (size_t i = 0; i < std::min(status_items.size(), (size_t)4); i++) {
        LineConfig config(LineType::CONTENT, status_items[i], COLOR_SUCCESS, FontSize::SMALL, LineAlign::LEFT);
        page.set_line(i, config);
    }
}

void setup_settings_page(PageTemplate& page, const std::vector<std::string>& settings, int selected_index) {
    page.clear();
    page.set_title("设置", COLOR_WHITE);
    page.set_menu_items(settings, selected_index);
}

void setup_progress_page(PageTemplate& page, const std::string& title, float progress, const std::string& status) {
    page.clear();
    page.set_title(title, COLOR_WHITE);
    
    // 进度条在第一行
    page.set_progress(0, progress, status);
    
    // 状态信息在第二行
    if (!status.empty()) {
        LineConfig status_config(LineType::STATUS, status, COLOR_TEXT_WHITE, FontSize::SMALL, LineAlign::CENTER);
        page.set_line(1, status_config);
    }
}

void setup_error_page(PageTemplate& page, const std::string& error_message, const std::string& action_hint) {
    page.clear();
    page.set_title("错误", COLOR_ERROR);
    
    // 错误信息
    LineConfig error_config(LineType::CONTENT, error_message, COLOR_ERROR, FontSize::MEDIUM, LineAlign::CENTER);
    page.set_line(0, error_config);
    
    // 操作提示
    LineConfig hint_config(LineType::STATUS, action_hint, COLOR_TEXT_WHITE, FontSize::SMALL, LineAlign::CENTER);
    page.set_line(2, hint_config);
}

void setup_info_page(PageTemplate& page, const std::string& title, const std::vector<std::string>& info_items) {
    page.clear();
    page.set_title(title, COLOR_WHITE);
    
    for (size_t i = 0; i < std::min(info_items.size(), (size_t)4); i++) {
        LineConfig config(LineType::CONTENT, info_items[i], COLOR_TEXT_WHITE, FontSize::SMALL, LineAlign::LEFT);
        page.set_line(i, config);
    }
}

void setup_split_comparison(PageTemplate& page, const std::string& title,
                           const std::vector<std::string>& left_items,
                           const std::vector<std::string>& right_items) {
    page.clear();
    page.set_title(title, COLOR_WHITE);
    page.enable_split_screen(true);
    
    // 设置左侧内容
    std::vector<LineConfig> left_configs;
    for (const auto& item : left_items) {
        left_configs.emplace_back(LineType::CONTENT, item, COLOR_TEXT_WHITE, FontSize::SMALL, LineAlign::LEFT);
    }
    page.set_left_content(left_configs);
    
    // 设置右侧内容
    std::vector<LineConfig> right_configs;
    for (const auto& item : right_items) {
        right_configs.emplace_back(LineType::CONTENT, item, COLOR_TEXT_WHITE, FontSize::SMALL, LineAlign::LEFT);
    }
    page.set_right_content(right_configs);
}

// 生产级页面创建接口 - 替代示例文件功能
void create_main_menu_page(PageTemplate& page) {
    std::vector<std::string> menu_items = {
        "状态监控",
        "触摸设置",
        "按键映射",
        "系统设置"
    };
    setup_main_menu(page, menu_items, 0);
}

void create_status_page(PageTemplate& page) {
    std::vector<std::string> status_items = {
        "系统状态: 正常",
        "触摸设备: 已连接",
        "按键状态: 正常",
        "灯光状态: 正常"
    };
    setup_status_page(page, "系统状态", status_items);
}

void create_settings_page(PageTemplate& page) {
    std::vector<std::string> settings = {
        "触摸灵敏度",
        "按键映射",
        "串口设置",
        "系统信息"
    };
    setup_settings_page(page, settings, 0);
}

void create_progress_page(PageTemplate& page, float progress) {
    std::string status = "进度: " + std::to_string((int)(progress)) + "%";
    setup_progress_page(page, "处理中", progress / 100.0f, status);
}

void create_dynamic_menu_page(PageTemplate& page, int selected_index) {
    std::vector<std::string> menu_items = {
        "选项 1",
        "选项 2",
        "选项 3",
        "返回"
    };
    setup_main_menu(page, menu_items, selected_index);
}

void create_error_page(PageTemplate& page) {
    setup_error_page(page, "系统错误", "按任意键返回");
}

void create_info_page(PageTemplate& page) {
    std::vector<std::string> info_items = {
        "版本: V3.0",
        "作者: MaiMai Team",
        "构建: 2024-01-20",
        "许可: MIT"
    };
    setup_info_page(page, "关于", info_items);
}

} // namespace PageTemplates