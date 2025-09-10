#include "scroll_bar.h"
#include "graphics_engine.h"
#include "../page_construction/page_template.h"
#include <algorithm>

namespace ui {

ScrollBar::ScrollBar() 
    : config_set_(false) {
    // 默认配置
    config_.x = 0;
    config_.y = 0;
    config_.width = 4;
    config_.height = 100;
    config_.bg_color = COLOR_DARK_GRAY;
    config_.bar_color = COLOR_LIGHT_GRAY;
    config_.border_color = COLOR_WHITE;
    config_.show_border = true;
    
    // 默认滚动信息
    scroll_info_.total_items = 0;
    scroll_info_.visible_items = 0;
    scroll_info_.current_offset = 0;
    scroll_info_.max_items = 15;
    
    // 初始化页面滚动状态
    page_scroll_state_.display_start_index = 0;
    page_scroll_state_.visible_line_count = 4;  // PageTemplate有标题时4行内容，无标题时5行
    page_scroll_state_.scroll_enabled = false;
}

ScrollBar::~ScrollBar() {
    // 无需特殊清理
}

void ScrollBar::set_config(const Config& config) {
    config_ = config;
    config_set_ = true;
}

void ScrollBar::update_scroll_info(const ScrollInfo& info) {
    scroll_info_ = info;
    
    // 确保偏移量在有效范围内
    int max_offset = std::max(0, scroll_info_.total_items - scroll_info_.visible_items);
    scroll_info_.current_offset = std::max(0, std::min(scroll_info_.current_offset, max_offset));
}

void ScrollBar::setup_page_scroll(const std::vector<LineConfig>& lines, int visible_lines) {
    // 限制最多48行
    size_t max_lines = std::min(lines.size(), (size_t)48);
    page_scroll_state_.all_lines.clear();
    page_scroll_state_.all_lines.reserve(max_lines);
    
    for (size_t i = 0; i < max_lines; i++) {
        page_scroll_state_.all_lines.push_back(lines[i]);
    }
    
    // 更新可见行数
    page_scroll_state_.visible_line_count = visible_lines;
    
    // 判断是否需要启用滚动（超过可见行数）
    page_scroll_state_.scroll_enabled = (page_scroll_state_.all_lines.size() > visible_lines);
    
    // 更新滚动信息
    update_scroll_info_from_page_state();
}

bool ScrollBar::scroll_up() {
    if (!page_scroll_state_.scroll_enabled || page_scroll_state_.display_start_index <= 0) {
        return false;
    }
    
    page_scroll_state_.display_start_index--;
    clamp_display_index();
    update_scroll_info_from_page_state();
    return true;
}

bool ScrollBar::scroll_down() {
    if (!page_scroll_state_.scroll_enabled) {
        return false;
    }
    
    int max_start_index = (int)page_scroll_state_.all_lines.size() - page_scroll_state_.visible_line_count;
    if (page_scroll_state_.display_start_index >= max_start_index) {
        return false;
    }
    
    page_scroll_state_.display_start_index++;
    clamp_display_index();
    update_scroll_info_from_page_state();
    return true;
}

void ScrollBar::get_visible_lines(std::vector<LineConfig>& visible_lines) const {
    visible_lines.clear();
    
    int start_index = page_scroll_state_.display_start_index;
    int end_index = std::min(start_index + page_scroll_state_.visible_line_count, 
                            (int)page_scroll_state_.all_lines.size());
    
    for (int i = start_index; i < end_index; i++) {
        visible_lines.push_back(page_scroll_state_.all_lines[i]);
    }
    
    // 如果不足可见行数，用空行填充
    while (visible_lines.size() < page_scroll_state_.visible_line_count) {
        LineConfig empty_line;
        empty_line.type = LineType::TEXT_ITEM;
        empty_line.text = "";
        empty_line.color = COLOR_TEXT_WHITE;
        empty_line.align = LineAlign::LEFT;
        empty_line.selected = false;
        visible_lines.push_back(empty_line);
    }
}

void ScrollBar::update_scroll_info_from_page_state() {
    scroll_info_.total_items = (int)page_scroll_state_.all_lines.size();
    scroll_info_.visible_items = page_scroll_state_.visible_line_count;
    scroll_info_.current_offset = page_scroll_state_.display_start_index;
    scroll_info_.max_items = 15;
}

void ScrollBar::clamp_display_index() {
    int max_start_index = std::max(0, (int)page_scroll_state_.all_lines.size() - page_scroll_state_.visible_line_count);
    page_scroll_state_.display_start_index = std::max(0, std::min(page_scroll_state_.display_start_index, max_start_index));
}

void ScrollBar::set_display_start_index(int index) {

    page_scroll_state_.display_start_index = index;
    clamp_display_index();
    update_scroll_info_from_page_state();
}

bool ScrollBar::should_show() const {
    return scroll_info_.total_items > scroll_info_.visible_items;
}

void ScrollBar::calculate_bar_geometry(int16_t& bar_y, int16_t& bar_height) const {
    if (!should_show()) {
        bar_y = config_.y;
        bar_height = config_.height;
        return;
    }
    
    // 计算滚动条高度比例
    float visible_ratio = static_cast<float>(scroll_info_.visible_items) / scroll_info_.total_items;
    bar_height = static_cast<int16_t>(config_.height * visible_ratio);
    
    // 确保滚动条有最小高度
    bar_height = std::max(bar_height, static_cast<int16_t>(8));
    
    // 计算滚动条位置
    int scrollable_items = scroll_info_.total_items - scroll_info_.visible_items;
    if (scrollable_items > 0) {
        float scroll_ratio = static_cast<float>(scroll_info_.current_offset) / scrollable_items;
        int available_space = config_.height - bar_height;
        bar_y = config_.y + static_cast<int16_t>(available_space * scroll_ratio);
    } else {
        bar_y = config_.y;
    }
}

void ScrollBar::render(GraphicsEngine& graphics) {
    if (!config_set_ || !should_show()) {
        return;
    }
    
    // 绘制背景轨道（圆弧样式）
    Rect bg_rect(config_.x, config_.y, config_.width, config_.height);
    graphics.fill_rect(bg_rect, config_.bg_color);
    
    // 绘制圆弧端点
    int16_t radius = config_.width / 2;
    if (radius > 0) {
        // 上端圆弧
        graphics.fill_circle(config_.x + radius, config_.y + radius, radius, config_.bg_color);
        // 下端圆弧
        graphics.fill_circle(config_.x + radius, config_.y + config_.height - radius, radius, config_.bg_color);
    }
    
    // 计算并绘制滚动条
    int16_t bar_y, bar_height;
    calculate_bar_geometry(bar_y, bar_height);
    
    // 绘制滚动条主体（圆弧样式）
    int16_t bar_x = config_.x;
    int16_t bar_width = config_.width;
    
    // 绘制滚动条矩形主体
    Rect bar_rect(bar_x, bar_y + radius, bar_width, bar_height - 2 * radius);
    if (bar_height > 2 * radius) {
        graphics.fill_rect(bar_rect, config_.bar_color);
    }
    
    // 绘制滚动条圆弧端点
    if (radius > 0 && bar_height >= 2 * radius) {
        // 上端圆弧
        graphics.fill_circle(bar_x + radius, bar_y + radius, radius, config_.bar_color);
        // 下端圆弧
        graphics.fill_circle(bar_x + radius, bar_y + bar_height - radius, radius, config_.bar_color);
    }
}

} // namespace ui