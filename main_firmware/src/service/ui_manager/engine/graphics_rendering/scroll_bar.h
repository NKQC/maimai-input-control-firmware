#pragma once

#include "graphics_engine.h"
#include <cstdint>
#include <vector>

// 前向声明
class PageTemplate;
struct LineConfig;

namespace ui {

/**
 * 滚动条组件
 * 用于显示内容的滚动位置和比例
 * 支持与PageTemplate配合工作，最多支持30行列表
 */
class ScrollBar {
public:
    struct Config {
        int16_t x;              // 滚动条X位置
        int16_t y;              // 滚动条Y位置
        int16_t width;          // 滚动条宽度
        int16_t height;         // 滚动条高度
        uint16_t bg_color;      // 背景颜色
        uint16_t bar_color;     // 滚动条颜色
        uint16_t border_color;  // 边框颜色
        bool show_border;       // 是否显示边框
    };
    
    struct ScrollInfo {
        int total_items;        // 总项目数
        int visible_items;      // 可见项目数
        int current_offset;     // 当前偏移量
        int max_items;          // 最大支持项目数（默认30）
    };
    
    // 页面滚动状态
    struct PageScrollState {
        std::vector<LineConfig> all_lines;  // 所有行配置（最多30行）
        int display_start_index;            // 当前显示起始索引
        int visible_line_count;             // 可见行数（固定4行）
        bool scroll_enabled;                // 是否启用滚动
    };
    
    ScrollBar();
    ~ScrollBar();
    
    // 配置滚动条
    void set_config(const Config& config);
    const Config& get_config() const { return config_; }
    
    // 更新滚动信息
    void update_scroll_info(const ScrollInfo& info);
    const ScrollInfo& get_scroll_info() const { return scroll_info_; }
    
    // 渲染滚动条
    void render(GraphicsEngine& graphics);
    
    // 检查是否需要显示滚动条
    bool should_show() const;
    
    // 计算滚动条位置和大小
    void calculate_bar_geometry(int16_t& bar_y, int16_t& bar_height) const;
    
    // PageTemplate集成接口
    void setup_page_scroll(const std::vector<LineConfig>& lines, int visible_lines = 4);
    bool scroll_up();                    // 向上滚动，返回是否成功
    bool scroll_down();                  // 向下滚动，返回是否成功
    void get_visible_lines(std::vector<LineConfig>& visible_lines) const;
    inline bool is_scroll_enabled() const { return page_scroll_state_.scroll_enabled; }
    inline int32_t get_total_lines() const { return page_scroll_state_.all_lines.size(); }
    inline int32_t get_display_start_index() const { return page_scroll_state_.display_start_index; }
    void set_display_start_index(int index);  // 设置滚动起始位置
    
private:
    Config config_;
    ScrollInfo scroll_info_;
    bool config_set_;
    PageScrollState page_scroll_state_;  // 页面滚动状态
    
    // 内部辅助函数
    void update_scroll_info_from_page_state();
    void clamp_display_index();
};

} // namespace ui