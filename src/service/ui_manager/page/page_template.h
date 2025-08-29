#ifndef PAGE_TEMPLATE_H
#define PAGE_TEMPLATE_H

#include "../engine/graphics_engine.h"
#include <string>
#include <vector>
#include <functional>

/**
 * 5行字符显示架构的标准页面模板
 * 屏幕尺寸: 160x80像素
 * 标准布局:
 * - 第1行: 标题行 (y=2-18, 16px高度)
 * - 第2行: 内容行1 (y=20-32, 12px高度)
 * - 第3行: 内容行2 (y=34-46, 12px高度) 
 * - 第4行: 内容行3 (y=48-60, 12px高度)
 * - 第5行: 状态/操作行 (y=62-74, 12px高度)
 */

// 页面行类型枚举
enum class LineType {
    TITLE,      // 标题行 - 大字体，居中显示
    CONTENT,    // 内容行 - 中字体，左对齐
    STATUS,     // 状态行 - 小字体，可配置对齐
    MENU_ITEM,  // 菜单项 - 中字体，支持选中状态
    PROGRESS    // 进度行 - 包含进度条和文本
};

// 文本对齐方式
enum class LineAlign {
    LEFT,
    CENTER,
    RIGHT
};

// 行配置结构
struct LineConfig {
    LineType type;
    std::string text;
    Color color;
    FontSize font_size;
    LineAlign align;
    bool selected;      // 用于菜单项选中状态
    float progress;     // 用于进度条 (0.0-1.0)
    
    LineConfig(LineType t = LineType::CONTENT, 
               const std::string& txt = "", 
               Color c = COLOR_TEXT_WHITE,
               FontSize fs = FontSize::MEDIUM,
               LineAlign a = LineAlign::LEFT,
               bool sel = false,
               float prog = 0.0f)
        : type(t), text(txt), color(c), font_size(fs), align(a), selected(sel), progress(prog) {}
};

// 页面模板类
class PageTemplate {
public:
    // 构造函数
    PageTemplate(GraphicsEngine* graphics_engine);
    
    // 设置页面内容
    void set_title(const std::string& title, Color color = COLOR_WHITE);
    void set_line(int line_index, const LineConfig& config);  // line_index: 0-3 (对应第2-5行)
    void set_lines(const std::vector<LineConfig>& lines);
    
    // 清空页面内容
    void clear();
    void clear_line(int line_index);
    
    // 绘制页面
    void draw();
    void draw_background(Color bg_color = COLOR_BG_DARK);
    
    // 菜单相关功能
    void set_menu_items(const std::vector<std::string>& items, int selected_index = 0);
    void set_selected_index(int index);
    int get_selected_index() const { return selected_menu_index_; }
    int get_menu_item_count() const { return menu_items_.size(); }
    
    // 进度条功能
    void set_progress(int line_index, float progress, const std::string& text = "");
    
    // 状态指示器
    void show_status_indicator(int line_index, Color color, bool filled = true);
    
    // 分屏显示支持
    void enable_split_screen(bool enable) { split_screen_enabled_ = enable; }
    void set_left_content(const std::vector<LineConfig>& left_lines);
    void set_right_content(const std::vector<LineConfig>& right_lines);
    
    // 增强的分屏功能
    void set_split_screen_content(const std::string& title,
                                 const std::vector<LineConfig>& left_lines,
                                 const std::vector<LineConfig>& right_lines,
                                 const std::string& left_header = "",
                                 const std::string& right_header = "");
    void set_split_headers(const std::string& left_header, const std::string& right_header);
    void enable_split_borders(bool enable) { split_borders_enabled_ = enable; }
    void set_split_ratio(float ratio); // 设置左右分屏比例 (0.1-0.9)
    
    // 工具函数
    static int16_t get_line_y_position(int line_index);  // 获取行的Y坐标
    static Rect get_line_rect(int line_index);           // 获取行的矩形区域
    Rect get_split_left_rect(int line_index);            // 获取左半屏行区域
    Rect get_split_right_rect(int line_index);           // 获取右半屏行区域
    
private:
    GraphicsEngine* graphics_engine_;
    
    // 页面内容
    std::string title_;
    Color title_color_;
    std::vector<LineConfig> lines_;  // 4行内容 (第2-5行)
    
    // 菜单状态
    std::vector<std::string> menu_items_;
    int selected_menu_index_;
    
    // 分屏模式
    bool split_screen_enabled_;
    std::vector<LineConfig> left_lines_;
    std::vector<LineConfig> right_lines_;
    std::string left_header_;
    std::string right_header_;
    bool split_borders_enabled_;
    float split_ratio_;  // 左侧占比 (0.1-0.9)
    
    // 内部绘制函数
    void draw_title();
    void draw_line(int line_index, const LineConfig& config);
    void draw_menu_line(int line_index, const LineConfig& config);
    void draw_progress_line(int line_index, const LineConfig& config);
    void draw_split_screen();
    
    // 工具函数
    int16_t get_text_x_position(const std::string& text, FontSize font_size, LineAlign align, const Rect& rect);
    void draw_selection_indicator(int line_index);
};

// 预定义的页面模板
namespace PageTemplates {
    // 预定义页面模板函数
    void setup_main_menu(PageTemplate& page, const std::vector<std::string>& menu_items, int selected_index = 0);
    
    // 状态页面模板
    void setup_status_page(PageTemplate& page, const std::string& title, const std::vector<std::string>& status_items);
    
    // 设置页面模板
    void setup_settings_page(PageTemplate& page, const std::vector<std::string>& settings, int selected_index = 0);
    
    // 进度页面模板
    void setup_progress_page(PageTemplate& page, const std::string& title, float progress, const std::string& status = "");
    
    // 错误页面模板
    void setup_error_page(PageTemplate& page, const std::string& error_message, const std::string& action_hint = "按任意键返回");
    
    // 信息页面模板
    void setup_info_page(PageTemplate& page, const std::string& title, const std::vector<std::string>& info_items);
    
    // 分屏对比页面模板
    void setup_split_comparison(PageTemplate& page, const std::string& title, 
                               const std::vector<std::string>& left_items,
                               const std::vector<std::string>& right_items);
    
    // 生产级页面创建接口 - 替代示例文件功能
    void create_main_menu_page(PageTemplate& page);
    void create_status_page(PageTemplate& page);
    void create_settings_page(PageTemplate& page);
    void create_progress_page(PageTemplate& page, float progress);
    void create_dynamic_menu_page(PageTemplate& page, int selected_index);
    void create_error_page(PageTemplate& page);
    void create_info_page(PageTemplate& page);
}

#endif // PAGE_TEMPLATE_H