#include "graphics_engine.h"
#include "font_system.h"
#include <algorithm>
#include <cmath>

GraphicsEngine::GraphicsEngine(uint16_t* framebuffer) : framebuffer_(framebuffer) {
}

void GraphicsEngine::clear(Color color) {
    if (!framebuffer_) return;
    
    std::fill(framebuffer_, framebuffer_ + SCREEN_BUFFER_SIZE, color);
}

void GraphicsEngine::set_pixel(int16_t x, int16_t y, Color color) {
    if (!framebuffer_ || !is_valid_coord(x, y)) return;
    
    framebuffer_[y * SCREEN_WIDTH + x] = color;
}

Color GraphicsEngine::get_pixel(int16_t x, int16_t y) const {
    if (!framebuffer_ || !is_valid_coord(x, y)) return COLOR_BLACK;
    
    return framebuffer_[y * SCREEN_WIDTH + x];
}

void GraphicsEngine::draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, Color color) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (true) {
        set_pixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void GraphicsEngine::draw_hline(int16_t x, int16_t y, int16_t width, Color color) {
    if (!is_valid_coord(x, y) || width <= 0) return;
    
    int16_t end_x = std::min((int16_t)(x + width), (int16_t)SCREEN_WIDTH);
    for (int16_t i = std::max((int16_t)0, x); i < end_x; i++) {
        set_pixel(i, y, color);
    }
}

void GraphicsEngine::draw_vline(int16_t x, int16_t y, int16_t height, Color color) {
    if (!is_valid_coord(x, y) || height <= 0) return;
    
    int16_t end_y = std::min((int16_t)(y + height), (int16_t)SCREEN_HEIGHT);
    for (int16_t i = std::max((int16_t)0, y); i < end_y; i++) {
        set_pixel(x, i, color);
    }
}

void GraphicsEngine::draw_rect(const Rect& rect, Color color) {
    draw_hline(rect.x, rect.y, rect.width, color);
    draw_hline(rect.x, rect.y + rect.height - 1, rect.width, color);
    draw_vline(rect.x, rect.y, rect.height, color);
    draw_vline(rect.x + rect.width - 1, rect.y, rect.height, color);
}

void GraphicsEngine::fill_rect(const Rect& rect, Color color) {
    int16_t start_x = std::max((int16_t)0, rect.x);
    int16_t start_y = std::max((int16_t)0, rect.y);
    int16_t end_x = std::min((int16_t)SCREEN_WIDTH, (int16_t)(rect.x + rect.width));
    int16_t end_y = std::min((int16_t)SCREEN_HEIGHT, (int16_t)(rect.y + rect.height));
    
    for (int16_t y = start_y; y < end_y; y++) {
        for (int16_t x = start_x; x < end_x; x++) {
            set_pixel(x, y, color);
        }
    }
}

void GraphicsEngine::draw_rounded_rect(const Rect& rect, int16_t radius, Color color) {
    // 绘制四条边（减去圆角部分）
    draw_hline(rect.x + radius, rect.y, rect.width - 2 * radius, color);
    draw_hline(rect.x + radius, rect.y + rect.height - 1, rect.width - 2 * radius, color);
    draw_vline(rect.x, rect.y + radius, rect.height - 2 * radius, color);
    draw_vline(rect.x + rect.width - 1, rect.y + radius, rect.height - 2 * radius, color);
    
    // 绘制四个圆角
    draw_circle_helper(rect.x + radius, rect.y + radius, radius, 1, color);
    draw_circle_helper(rect.x + rect.width - radius - 1, rect.y + radius, radius, 2, color);
    draw_circle_helper(rect.x + rect.width - radius - 1, rect.y + rect.height - radius - 1, radius, 4, color);
    draw_circle_helper(rect.x + radius, rect.y + rect.height - radius - 1, radius, 8, color);
}

void GraphicsEngine::fill_rounded_rect(const Rect& rect, int16_t radius, Color color) {
    // 填充中间矩形
    fill_rect(Rect(rect.x + radius, rect.y, rect.width - 2 * radius, rect.height), color);
    
    // 填充左右两侧矩形
    fill_rect(Rect(rect.x, rect.y + radius, radius, rect.height - 2 * radius), color);
    fill_rect(Rect(rect.x + rect.width - radius, rect.y + radius, radius, rect.height - 2 * radius), color);
    
    // 填充四个圆角
    fill_circle_helper(rect.x + radius, rect.y + radius, radius, 1, 0, color);
    fill_circle_helper(rect.x + rect.width - radius - 1, rect.y + radius, radius, 2, 0, color);
    fill_circle_helper(rect.x + rect.width - radius - 1, rect.y + rect.height - radius - 1, radius, 4, 0, color);
    fill_circle_helper(rect.x + radius, rect.y + rect.height - radius - 1, radius, 8, 0, color);
}

void GraphicsEngine::draw_circle(int16_t x, int16_t y, int16_t radius, Color color) {
    int16_t f = 1 - radius;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * radius;
    int16_t px = 0;
    int16_t py = radius;
    
    set_pixel(x, y + radius, color);
    set_pixel(x, y - radius, color);
    set_pixel(x + radius, y, color);
    set_pixel(x - radius, y, color);
    
    while (px < py) {
        if (f >= 0) {
            py--;
            ddF_y += 2;
            f += ddF_y;
        }
        px++;
        ddF_x += 2;
        f += ddF_x;
        
        set_pixel(x + px, y + py, color);
        set_pixel(x - px, y + py, color);
        set_pixel(x + px, y - py, color);
        set_pixel(x - px, y - py, color);
        set_pixel(x + py, y + px, color);
        set_pixel(x - py, y + px, color);
        set_pixel(x + py, y - px, color);
        set_pixel(x - py, y - px, color);
    }
}

void GraphicsEngine::fill_circle(int16_t x, int16_t y, int16_t radius, Color color) {
    draw_vline(x, y - radius, 2 * radius + 1, color);
    fill_circle_helper(x, y, radius, 3, 0, color);
}

void GraphicsEngine::draw_text(const char* text, int16_t x, int16_t y, Color color, FontSize size) {
    if (!text) return;
    
    int16_t cursor_x = x;
    int16_t font_height = get_font_height(size);
    
    while (*text) {
        if (*text == '\n') {
            cursor_x = x;
            y += font_height + 2;
            text++;
            continue;
        }
        
        // 绘制ASCII字符
        if (*text >= 32 && *text <= 126) {
            FontSystem::draw_ascii_char(*text, cursor_x, y, color, size, this);
            cursor_x += FontSystem::get_ascii_char_width(*text, size);
        }
        text++;
    }
}

void GraphicsEngine::draw_text_aligned(const char* text, const Rect& rect, Color color, TextAlign align, FontSize size) {
    if (!text) return;
    
    int16_t text_width = get_text_width(text, size);
    int16_t text_height = get_font_height(size);
    
    int16_t x = rect.x;
    int16_t y = rect.y + (rect.height - text_height) / 2;
    
    switch (align) {
        case TextAlign::CENTER:
            x = rect.x + (rect.width - text_width) / 2;
            break;
        case TextAlign::RIGHT:
            x = rect.x + rect.width - text_width;
            break;
        case TextAlign::LEFT:
        default:
            break;
    }
    
    draw_text(text, x, y, color, size);
}

void GraphicsEngine::draw_chinese_text(const char* utf8_text, int16_t x, int16_t y, Color color, FontSize size) {
    if (!utf8_text) return;
    
    int16_t cursor_x = x;
    int16_t font_height = get_font_height(size);
    
    const char* p = utf8_text;
    while (*p) {
        if (*p == '\n') {
            cursor_x = x;
            y += font_height + 2;
            p++;
            continue;
        }
        
        // 检查是否为UTF-8中文字符（3字节）
        if ((*p & 0xE0) == 0xE0 && (*(p+1) & 0x80) == 0x80 && (*(p+2) & 0x80) == 0x80) {
            // 绘制中文字符
            uint32_t unicode = FontSystem::utf8_to_unicode(p);
            FontSystem::draw_chinese_char(unicode, cursor_x, y, color, size, this);
            cursor_x += FontSystem::get_chinese_char_width(unicode, size);
            p += 3;
        } else if (*p >= 32 && *p <= 126) {
            // 绘制ASCII字符
            FontSystem::draw_ascii_char(*p, cursor_x, y, color, size, this);
            cursor_x += FontSystem::get_ascii_char_width(*p, size);
            p++;
        } else {
            p++;
        }
    }
}

void GraphicsEngine::draw_chinese_text_aligned(const char* utf8_text, const Rect& rect, Color color, TextAlign align, FontSize size) {
    if (!utf8_text) return;
    
    int16_t text_width = get_chinese_text_width(utf8_text, size);
    int16_t text_height = get_font_height(size);
    
    int16_t x = rect.x;
    int16_t y = rect.y + (rect.height - text_height) / 2;
    
    switch (align) {
        case TextAlign::CENTER:
            x = rect.x + (rect.width - text_width) / 2;
            break;
        case TextAlign::RIGHT:
            x = rect.x + rect.width - text_width;
            break;
        case TextAlign::LEFT:
        default:
            break;
    }
    
    draw_chinese_text(utf8_text, x, y, color, size);
}

// 图标绘制实现
void GraphicsEngine::draw_icon_arrow_up(int16_t x, int16_t y, int16_t size, Color color) {
    int16_t half = size / 2;
    for (int16_t i = 0; i < half; i++) {
        draw_hline(x + half - i, y + i, 2 * i + 1, color);
    }
}

void GraphicsEngine::draw_icon_arrow_down(int16_t x, int16_t y, int16_t size, Color color) {
    int16_t half = size / 2;
    for (int16_t i = 0; i < half; i++) {
        draw_hline(x + i, y + i, size - 2 * i, color);
    }
}

void GraphicsEngine::draw_icon_arrow_left(int16_t x, int16_t y, int16_t size, Color color) {
    int16_t half = size / 2;
    for (int16_t i = 0; i < half; i++) {
        draw_vline(x + i, y + half - i, 2 * i + 1, color);
    }
}

void GraphicsEngine::draw_icon_arrow_right(int16_t x, int16_t y, int16_t size, Color color) {
    int16_t half = size / 2;
    for (int16_t i = 0; i < half; i++) {
        draw_vline(x + half - i, y + i, size - 2 * i, color);
    }
}

void GraphicsEngine::draw_icon_check(int16_t x, int16_t y, int16_t size, Color color) {
    int16_t third = size / 3;
    draw_line(x + third, y + size - third, x + 2 * third, y + size, color);
    draw_line(x + 2 * third, y + size, x + size, y + third, color);
}

void GraphicsEngine::draw_icon_cross(int16_t x, int16_t y, int16_t size, Color color) {
    draw_line(x, y, x + size, y + size, color);
    draw_line(x + size, y, x, y + size, color);
}

void GraphicsEngine::draw_icon_settings(int16_t x, int16_t y, int16_t size, Color color) {
    int16_t center = size / 2;
    int16_t inner_radius = size / 4;
    int16_t outer_radius = size / 2;
    
    // 绘制齿轮的简化版本
    draw_circle(x + center, y + center, outer_radius, color);
    fill_circle(x + center, y + center, inner_radius, COLOR_BLACK);
    
    // 绘制齿轮齿
    for (int i = 0; i < 8; i++) {
        float angle = i * 3.14159f / 4;
        int16_t dx = (int16_t)(cos(angle) * outer_radius);
        int16_t dy = (int16_t)(sin(angle) * outer_radius);
        draw_line(x + center, y + center, x + center + dx, y + center + dy, color);
    }
}

// UI组件绘制实现
void GraphicsEngine::draw_button(const Rect& rect, const char* text, Color bg_color, Color text_color, bool pressed) {
    Color actual_bg = pressed ? COLOR_BORDER : bg_color;
    fill_rounded_rect(rect, 4, actual_bg);
    draw_rounded_rect(rect, 4, COLOR_BORDER);
    
    if (text) {
        draw_text_aligned(text, rect, text_color, TextAlign::CENTER, FontSize::MEDIUM);
    }
}

void GraphicsEngine::draw_progress_bar(const Rect& rect, float progress, Color bg_color, Color fill_color) {
    fill_rounded_rect(rect, 2, bg_color);
    
    if (progress > 0.0f) {
        int16_t fill_width = (int16_t)(rect.width * std::min(1.0f, std::max(0.0f, progress)));
        Rect fill_rect(rect.x, rect.y, fill_width, rect.height);
        fill_rounded_rect(fill_rect, 2, fill_color);
    }
}

void GraphicsEngine::draw_slider(const Rect& rect, float value, Color bg_color, Color handle_color) {
    // 绘制滑轨
    int16_t track_height = 4;
    int16_t track_y = rect.y + (rect.height - track_height) / 2;
    fill_rounded_rect(Rect(rect.x, track_y, rect.width, track_height), 2, bg_color);
    
    // 绘制滑块
    int16_t handle_size = rect.height;
    int16_t handle_x = rect.x + (int16_t)((rect.width - handle_size) * std::min(1.0f, std::max(0.0f, value)));
    fill_circle(handle_x + handle_size / 2, rect.y + rect.height / 2, handle_size / 2, handle_color);
}

void GraphicsEngine::draw_checkbox(int16_t x, int16_t y, int16_t size, bool checked, Color color) {
    draw_rect(Rect(x, y, size, size), color);
    
    if (checked) {
        draw_icon_check(x + 2, y + 2, size - 4, color);
    }
}

void GraphicsEngine::draw_card(const Rect& rect, Color bg_color, int16_t radius) {
    fill_rounded_rect(rect, radius, bg_color);
    draw_rounded_rect(rect, radius, COLOR_BORDER);
}

void GraphicsEngine::draw_list_item(const Rect& rect, const char* text, const char* icon, bool selected) {
    Color bg_color = selected ? COLOR_PRIMARY : COLOR_BG_CARD;
    Color text_color = selected ? COLOR_BLACK : COLOR_TEXT_WHITE;
    
    fill_rect(rect, bg_color);
    
    int16_t x_offset = rect.x + 8;
    
    // 绘制图标（如果有）
    if (icon && *icon) {
        draw_text(icon, x_offset, rect.y + (rect.height - get_font_height(FontSize::MEDIUM)) / 2, text_color, FontSize::MEDIUM);
        x_offset += 20;
    }
    
    // 绘制文本
    if (text) {
        draw_chinese_text(text, x_offset, rect.y + (rect.height - get_font_height(FontSize::MEDIUM)) / 2, text_color, FontSize::MEDIUM);
    }
}

void GraphicsEngine::draw_status_indicator(int16_t x, int16_t y, int16_t size, Color color, bool filled) {
    if (filled) {
        fill_circle(x + size / 2, y + size / 2, size / 2, color);
    } else {
        draw_circle(x + size / 2, y + size / 2, size / 2, color);
    }
}

// 工具函数实现
int16_t GraphicsEngine::get_text_width(const char* text, FontSize size) const {
    if (!text) return 0;
    
    int16_t width = 0;
    while (*text) {
        if (*text >= 32 && *text <= 126) {
            width += FontSystem::get_ascii_char_width(*text, size);
        }
        text++;
    }
    return width;
}

int16_t GraphicsEngine::get_chinese_text_width(const char* utf8_text, FontSize size) const {
    if (!utf8_text) return 0;
    
    int16_t width = 0;
    const char* p = utf8_text;
    
    while (*p) {
        if ((*p & 0xE0) == 0xE0 && (*(p+1) & 0x80) == 0x80 && (*(p+2) & 0x80) == 0x80) {
            uint32_t unicode = FontSystem::utf8_to_unicode(p);
            width += FontSystem::get_chinese_char_width(unicode, size);
            p += 3;
        } else if (*p >= 32 && *p <= 126) {
            width += FontSystem::get_ascii_char_width(*p, size);
            p++;
        } else {
            p++;
        }
    }
    return width;
}

int16_t GraphicsEngine::get_font_height(FontSize size) const {
    return (int16_t)size;
}

Color GraphicsEngine::rgb_to_color(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void GraphicsEngine::color_to_rgb(Color color, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = (color >> 8) & 0xF8;
    g = (color >> 3) & 0xFC;
    b = (color << 3) & 0xF8;
}

bool GraphicsEngine::is_valid_coord(int16_t x, int16_t y) const {
    return x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT;
}

// 内部辅助函数实现
void GraphicsEngine::draw_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t corner, Color color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        if (corner & 0x4) {
            set_pixel(x0 + x, y0 + y, color);
            set_pixel(x0 + y, y0 + x, color);
        }
        if (corner & 0x2) {
            set_pixel(x0 + x, y0 - y, color);
            set_pixel(x0 + y, y0 - x, color);
        }
        if (corner & 0x8) {
            set_pixel(x0 - y, y0 + x, color);
            set_pixel(x0 - x, y0 + y, color);
        }
        if (corner & 0x1) {
            set_pixel(x0 - y, y0 - x, color);
            set_pixel(x0 - x, y0 - y, color);
        }
    }
}

void GraphicsEngine::fill_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t corner, int16_t delta, Color color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        if (corner & 0x1) {
            draw_vline(x0 + x, y0 - y, 2 * y + 1 + delta, color);
            draw_vline(x0 + y, y0 - x, 2 * x + 1 + delta, color);
        }
        if (corner & 0x2) {
            draw_vline(x0 - x, y0 - y, 2 * y + 1 + delta, color);
            draw_vline(x0 - y, y0 - x, 2 * x + 1 + delta, color);
        }
    }
}

void GraphicsEngine::swap_int16(int16_t& a, int16_t& b) {
    int16_t temp = a;
    a = b;
    b = temp;
}