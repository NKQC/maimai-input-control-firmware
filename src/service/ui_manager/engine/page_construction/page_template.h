#ifndef PAGE_TEMPLATE_H
#define PAGE_TEMPLATE_H

#include "../graphics_rendering/graphics_engine.h"
#include "../graphics_rendering/scroll_bar.h"
#include <string>
#include <vector>
#include <functional>
#include <cstring>  // for memset

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
    TEXT_ITEM,      // 文本项 - 支持标题行与内容行合并构造
    STATUS_LINE,    // 状态行 - 支持动态文本构造
    MENU_JUMP,      // 菜单跳转项 - 支持指定目标页面名称和文本构造
    PROGRESS_BAR,   // 进度条项 - 接收uint8_t指针(0-255对应0%-100%)
    INT_SETTING,    // INT设置项 - 包含值变更回调和完成回调
    BUTTON_ITEM,    // 按钮项 - 支持文本构造和点击回调
    BACK_ITEM       // 返回项 - 返回上一页并恢复状态
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
    LineAlign align;
    bool selected;      // 用于菜单项选中状态
    
    // 使用union节省内存 - 不同类型的数据不会同时使用
    union TypeSpecificData {
        // 进度条相关
        struct {
            uint8_t* progress_ptr;  // 进度条数据指针 (0-255对应0%-100%)
        } progress;
        
        // INT设置项相关
        struct {
            int32_t* int_value_ptr; // INT值指针
            int32_t min_value;      // 最小值
            int32_t max_value;      // 最大值
        } int_setting;
        
        // 默认构造函数
        TypeSpecificData() { memset(this, 0, sizeof(TypeSpecificData)); }
    } data;
    
    // 字符串数据 - 需要单独存储因为union不能包含非POD类型
    std::string setting_title;     // 设置项标题
    std::string target_page_name;  // 目标页面名称
    
    // 回调函数 - 需要单独存储
    std::function<void(int32_t)> value_change_callback; // 值变更回调
    std::function<void()> complete_callback;            // 完成回调
    std::function<void()> click_callback;               // 点击回调
    
    // 基础构造函数 - 文本项
    LineConfig(const std::string& txt, 
               Color c = COLOR_TEXT_WHITE,
               LineAlign a = LineAlign::LEFT)
        : type(LineType::TEXT_ITEM), text(txt), color(c), align(a), selected(false), data() {}
    
    // 标题行与内容行合并构造 - 文本项
    LineConfig(const std::string& title, const std::string& content,
               Color c = COLOR_TEXT_WHITE, LineAlign a = LineAlign::LEFT)
        : type(LineType::TEXT_ITEM), text(title + ": " + content), color(c), align(a), selected(false), data() {}
    
    // 状态行构造
    static LineConfig create_status_line(const std::string& txt, 
                                        Color c = COLOR_TEXT_WHITE,
                                        LineAlign a = LineAlign::CENTER) {
        LineConfig config;
        config.type = LineType::STATUS_LINE;
        config.text = txt;
        config.color = c;
        config.align = a;
        config.selected = false;
        return config;
    }
    
    // 菜单跳转项构造
    static LineConfig create_menu_jump(const std::string& txt, const std::string& target_page,
                                     Color c = COLOR_TEXT_WHITE) {
        LineConfig config;
        config.type = LineType::MENU_JUMP;
        config.text = txt;
        config.target_page_name = target_page;
        config.color = c;
        config.align = LineAlign::LEFT;
        config.selected = false;
        // union数据会通过构造函数自动初始化为0
        return config;
    }
    
    // 进度条项构造
    static LineConfig create_progress_bar(uint8_t* progress_data_ptr, const std::string& txt = "",
                                         Color c = COLOR_TEXT_WHITE) {
        LineConfig config;
        config.type = LineType::PROGRESS_BAR;
        config.text = txt;
        config.data.progress.progress_ptr = progress_data_ptr;
        config.color = c;
        config.align = LineAlign::LEFT;
        config.selected = false;
        return config;
    }
    
    // INT设置项构造
    static LineConfig create_int_setting(int32_t* value_ptr, int32_t min_val, int32_t max_val,
                                        const std::string& display_text, const std::string& title,
                                        std::function<void(int32_t)> change_cb = nullptr,
                                        std::function<void()> complete_cb = nullptr,
                                        Color c = COLOR_TEXT_WHITE) {
        LineConfig config;
        config.type = LineType::INT_SETTING;
        config.text = display_text;
        config.setting_title = title;
        config.data.int_setting.int_value_ptr = value_ptr;
        config.data.int_setting.min_value = min_val;
        config.data.int_setting.max_value = max_val;
        config.value_change_callback = change_cb;
        config.complete_callback = complete_cb;
        config.color = c;
        config.align = LineAlign::CENTER;
        config.selected = false;
        return config;
    }
    
    // 按钮项构造
    static LineConfig create_button(const std::string& txt, std::function<void()> callback,
                                   Color c = COLOR_TEXT_WHITE, LineAlign a = LineAlign::CENTER) {
        LineConfig config;
        config.type = LineType::BUTTON_ITEM;
        config.text = txt;
        config.click_callback = callback;
        config.color = c;
        config.align = a;
        config.selected = false;
        // union数据会通过构造函数自动初始化为0
        return config;
    }
    
    // 返回项构造
    static LineConfig create_back_item(const std::string& txt = "返回",
                                      Color c = COLOR_TEXT_WHITE) {
        LineConfig config;
        config.type = LineType::BACK_ITEM;
        config.text = txt;
        config.color = c;
        config.align = LineAlign::LEFT;
        config.selected = false;
        // union数据会通过构造函数自动初始化为0
        return config;
    }
    
    // 默认构造函数
    LineConfig() : type(LineType::TEXT_ITEM), color(COLOR_TEXT_WHITE), align(LineAlign::LEFT),
                   selected(false), data() {}
};

// 页面模板类
class PageTemplate {
public:
    // 构造函数
    PageTemplate(GraphicsEngine* graphics_engine);
    
    // 基础方法
    void set_title(const std::string& title, Color color = COLOR_WHITE);
    void set_line(int line_index, const LineConfig& config);  // line_index: 0-3 (对应第2-5行)
    void set_lines(const std::vector<LineConfig>& lines);
    void set_all_lines(const std::vector<LineConfig>& lines);  // 设置所有行（支持滚动）

    // 清理方法
    void flush();     // 清空行缓存但保留滚动状态，用于页面刷新
    void clear();     // 完全清空所有状态，仅在换页面时使用
    void clear_line(int line_index);
    
    // 绘制页面
    void draw();
    void draw_background(Color bg_color = COLOR_BG_DARK);
    
    // 菜单相关功能
    void set_selected_index(int index);
    
    // 滚动功能
    bool scroll_up();     // 向上滚动
    bool scroll_down();   // 向下滚动
    bool is_scroll_enabled() const { return scroll_enabled_; }
    void update_scroll_display();  // 更新滚动显示
    int get_selected_index() const { return selected_menu_index_; }
    
    // 页面状态保存和恢复
    int get_scroll_position() const { return scroll_bar_.get_display_start_index(); }
    void set_scroll_position(int position) { 
        if (scroll_enabled_) {
            scroll_bar_.set_display_start_index(position);
            update_scroll_display();
        }
    }
    
    // 设置可见区域的最后一行（让滚动跟随光标）
    void set_visible_end_line(int target_line_index);
    
    // 数据访问方法
    const std::vector<LineConfig>& get_all_lines() const { return all_lines_; }
    const std::vector<LineConfig>& get_lines() const { return lines_; }
    int get_scroll_start_index() const { return scroll_bar_.get_display_start_index(); }
    int get_visible_lines_count() const { return visible_lines_count_; }
    
    int get_menu_item_count() const {
        // 统一使用all_lines_，在非滚动模式下all_lines_和lines_内容相同
        const auto& target_lines = all_lines_.empty() ? lines_ : all_lines_;
        int count = 0;
        for (const auto& line : target_lines) {
            // 只计算可交互的菜单项，不包括普通文本
            if (!line.text.empty() && 
                (line.type == LineType::MENU_JUMP || 
                 line.type == LineType::INT_SETTING || 
                 line.type == LineType::BUTTON_ITEM || 
                 line.type == LineType::BACK_ITEM)) {
                count++;
            }
        }
        return count;
    }
    
    // 获取行配置
    const LineConfig& get_line_config(int line_index) const {
        if (!all_lines_.empty()) {
            if (scroll_enabled_) {
                // 滚动模式下，需要根据滚动位置计算实际索引
                int actual_index = scroll_bar_.get_display_start_index() + line_index;
                if (actual_index >= 0 && actual_index < (int)all_lines_.size()) {
                    return all_lines_[actual_index];
                }
            } else {
                // 非滚动模式，直接从all_lines_获取
                if (line_index >= 0 && line_index < (int)all_lines_.size()) {
                    return all_lines_[line_index];
                }
            }
        } else {
            // 回退到lines_（兼容性）
            if (line_index >= 0 && line_index < (int)lines_.size()) {
                return lines_[line_index];
            }
        }
        static LineConfig empty_config;
        return empty_config;
    }
    
    // 进度条功能
    void set_progress(int line_index, float progress, const std::string& text = "");
    
    // 状态指示器
    void show_status_indicator(int line_index, Color color, bool filled = true);
    
    // 分屏显示支持
    void enable_split_screen(bool enable) { split_screen_enabled_ = enable; }
    bool is_split_screen_enabled() const { return split_screen_enabled_; }
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
    
    // 获取分屏内容
    const std::vector<LineConfig>& get_left_lines() const { return left_lines_; }
    const std::vector<LineConfig>& get_right_lines() const { return right_lines_; }
    
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
    std::vector<LineConfig> all_lines_;  // 所有行内容（用于滚动）

    // 状态跟踪变量
    bool has_title_;          // 是否设置了标题
    bool has_split_screen_;   // 是否启用分屏模式
    int visible_lines_count_; // 当前可见行数缓存

    // 选中状态
    int selected_menu_index_;
    
    // 滚动条支持
    ui::ScrollBar scroll_bar_;
    bool scroll_enabled_;
    
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
    void draw_split_screen();
    
    // 新的渲染方法
    void draw_text_item(int line_index, const LineConfig& config);
    void draw_status_line(int line_index, const LineConfig& config);
    void draw_menu_jump(int line_index, const LineConfig& config);
    void draw_progress_bar(int line_index, const LineConfig& config);
    void draw_int_setting(int line_index, const LineConfig& config);
    void draw_button_item(int line_index, const LineConfig& config);
    void draw_back_item(int line_index, const LineConfig& config);

    // 辅助方法
    int16_t get_text_x_position(const std::string& text, LineAlign align, const Rect& rect);
    void draw_selection_indicator(int line_index);
};

// 预定义的页面模板
namespace PageTemplates {
    // 保留错误页面创建函数，其他页面应在page目录中实现
    void create_error_page(PageTemplate& page);
    
    // INT设置页面模板
    void setup_int_setting_page(PageTemplate& page, const std::string& title, 
                               int32_t* value_ptr, int32_t min_val, int32_t max_val,
                               std::function<void(int32_t)> change_cb = nullptr,
                               std::function<void()> complete_cb = nullptr);
}

#endif // PAGE_TEMPLATE_H