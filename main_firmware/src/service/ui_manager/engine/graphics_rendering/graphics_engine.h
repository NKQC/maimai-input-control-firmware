#ifndef GRAPHICS_ENGINE_H
#define GRAPHICS_ENGINE_H

#include <stdint.h>
#include <cstring>

// 屏幕尺寸定义
#define SCREEN_WIDTH  160
#define SCREEN_HEIGHT 80
#define SCREEN_BUFFER_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT)

// RGB565颜色定义
typedef uint16_t Color;

// 基础颜色定义 (BGR565格式 - 适配ST7735S BGR模式)
#define COLOR_BLACK       0x0000  // 黑色
#define COLOR_WHITE       0xFFFF  // 白色
#define COLOR_RED         0x001F  // 红色 (BGR565: B=31, G=0, R=0)
#define COLOR_GREEN       0x07E0  // 绿色 (BGR565: B=0, G=63, R=0)
#define COLOR_BLUE        0xF800  // 蓝色 (BGR565: B=0, G=0, R=31)
#define COLOR_YELLOW      0x07FF  // 黄色 (BGR565: B=31, G=63, R=0)
#define COLOR_CYAN        0xFFE0  // 青色 (BGR565: B=31, G=63, R=0)
#define COLOR_MAGENTA     0xF81F  // 洋红 (BGR565: B=31, G=0, R=31)
#define COLOR_GRAY        0x8410  // 灰色
#define COLOR_DARK_GRAY   0x4208  // 深灰色dcxz
#define COLOR_LIGHT_GRAY  0xC618  // 浅灰色

// 现代化UI颜色主题 (RBG565格式)
#define COLOR_BG_DARK     0x18C3  // 深色背景 #1a1a1a
#define COLOR_BG_CARD     0x2945  // 卡片背景 #2a2a2a
#define COLOR_BORDER      0x4208  // 边框颜色 #404040
#define COLOR_PRIMARY     0x801F  // 主色调 #0080ff (RBG565格式)
#define COLOR_TEXT_WHITE  0xFFFF  // 白色文字
#define COLOR_TEXT_GRAY   0x8410  // 灰色文字
#define COLOR_TEXT_YELLOW 0xFFE0  // 黄色文字
#define COLOR_TEXT_GREEN  0xE007  // 绿色文字
#define COLOR_SUCCESS     0xE007  // 成功绿色
#define COLOR_WARNING     0xFFE0  // 警告黄色 (RBG565格式)
#define COLOR_ERROR       0xF001  // 错误红色 (RBG565格式)
#define COLOR_INFO        0x801F  // 信息蓝色 (RBG565格式)

// 点结构
struct Point {
    int16_t x, y;
    Point(int16_t x = 0, int16_t y = 0) : x(x), y(y) {}
};

// 矩形结构
struct Rect {
    int16_t x, y, width, height;
    Rect(int16_t x = 0, int16_t y = 0, int16_t w = 0, int16_t h = 0) 
        : x(x), y(y), width(w), height(h) {}
};

// 统一字体大小定义
#define DEFAULT_FONT_HEIGHT 14  // 统一使用14px高度字体

// 文本对齐方式
enum class TextAlign {
    LEFT,
    CENTER,
    RIGHT
};

// 绘制引擎类
class GraphicsEngine {
public:
    // 构造函数，传入framebuffer指针
    GraphicsEngine(uint16_t* framebuffer);
    
    // 基本绘制函数
    void clear(Color color = COLOR_BLACK);
    void set_pixel(int16_t x, int16_t y, Color color);
    Color get_pixel(int16_t x, int16_t y) const;
    
    // 线条绘制
    void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, Color color);
    void draw_hline(int16_t x, int16_t y, int16_t width, Color color);
    void draw_vline(int16_t x, int16_t y, int16_t height, Color color);
    
    // 矩形绘制
    void draw_rect(const Rect& rect, Color color);
    void fill_rect(const Rect& rect, Color color);
    void draw_rounded_rect(const Rect& rect, int16_t radius, Color color);
    void fill_rounded_rect(const Rect& rect, int16_t radius, Color color);
    
    // 圆形绘制
    void draw_circle(int16_t x, int16_t y, int16_t radius, Color color);
    void fill_circle(int16_t x, int16_t y, int16_t radius, Color color);
    
    // 文本绘制
    void draw_text(const char* text, int16_t x, int16_t y, Color color);
    void draw_text_aligned(const char* text, const Rect& rect, Color color, 
                          TextAlign align = TextAlign::LEFT);

    // 中文文本绘制
    void draw_chinese_text(const char* utf8_text, int16_t x, int16_t y, Color color);
    void draw_chinese_text_aligned(const char* utf8_text, const Rect& rect, Color color,
                                  TextAlign align = TextAlign::LEFT);
    
    // 图标绘制（简单的几何图标）
    void draw_icon_arrow_up(int16_t x, int16_t y, int16_t size, Color color);
    void draw_icon_arrow_down(int16_t x, int16_t y, int16_t size, Color color);
    void draw_icon_arrow_left(int16_t x, int16_t y, int16_t size, Color color);
    void draw_icon_arrow_right(int16_t x, int16_t y, int16_t size, Color color);
    void draw_icon_check(int16_t x, int16_t y, int16_t size, Color color);
    void draw_icon_cross(int16_t x, int16_t y, int16_t size, Color color);
    void draw_icon_settings(int16_t x, int16_t y, int16_t size, Color color);
    
    // UI组件绘制
    void draw_button(const Rect& rect, const char* text, Color bg_color, Color text_color, bool pressed = false);
    void draw_progress_bar(const Rect& rect, uint8_t progress, Color bg_color, Color fill_color);
    void draw_slider(const Rect& rect, float value, Color bg_color, Color handle_color);
    void draw_checkbox(int16_t x, int16_t y, int16_t size, bool checked, Color color);
    
    // 现代化UI组件
    void draw_card(const Rect& rect, Color bg_color = COLOR_BG_CARD, int16_t radius = 6);
    void draw_list_item(const Rect& rect, const char* text, const char* icon, bool selected = false);
    void draw_status_indicator(int16_t x, int16_t y, int16_t size, Color color, bool filled = true);
    
    // 文本测量
    int16_t get_text_width(const char* text) const;
    int16_t get_chinese_text_width(const char* utf8_text) const;
    int16_t get_font_height() const;
    
    // RGB转换函数
    static Color rgb_to_color(uint8_t r, uint8_t g, uint8_t b);
    static void color_to_rgb(Color color, uint8_t& r, uint8_t& g, uint8_t& b);
    
    // 边界检查
    bool is_valid_coord(int16_t x, int16_t y) const;
    
private:
    uint16_t* framebuffer_;
    
    // 字符绘制相关函数
    void draw_character(const char* utf8_char, int16_t x, int16_t y, Color color);
    int16_t get_character_width(const char* utf8_char) const;
    void draw_char_bitmap(const uint8_t* bitmap_data, uint8_t width, uint8_t height, 
                         int16_t x, int16_t y, Color color);
    
    // 辅助函数
    void draw_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t corner, Color color);
    void fill_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t corner, int16_t delta, Color color);
    void swap_int16(int16_t& a, int16_t& b);
};

#endif // GRAPHICS_ENGINE_H