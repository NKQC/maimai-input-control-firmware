#ifndef FONT_SYSTEM_H
#define FONT_SYSTEM_H

#include <stdint.h>
#include "graphics_engine.h"
#include "fonts/font_data.h"

// FontSize枚举已在graphics_engine.h中定义

// 中文字符映射结构
struct ChineseCharMap {
    uint32_t unicode;
    const CharBitmap* bitmap;
};

// 字体系统类
class FontSystem {
public:
    // ASCII字符相关
    static void draw_ascii_char(char c, int16_t x, int16_t y, Color color, FontSize size, GraphicsEngine* engine);
    static int16_t get_ascii_char_width(char c, FontSize size);
    static const CharBitmap* get_ascii_char_bitmap(char c, FontSize size);
    
    // 中文字符相关
    static void draw_chinese_char(uint32_t unicode, int16_t x, int16_t y, Color color, FontSize size, GraphicsEngine* engine);
    static int16_t get_chinese_char_width(uint32_t unicode, FontSize size);
    static const CharBitmap* get_chinese_char_bitmap(uint32_t unicode, FontSize size);
    
    // UTF-8转换
    static uint32_t utf8_to_unicode(const char* utf8);
    static int utf8_char_length(const char* utf8);
    
    // 字体数据获取
    static const CharBitmap* get_font_data(FontSize size);
    
private:
    // 绘制字符点阵
    static void draw_char_bitmap(const CharBitmap* bitmap, int16_t x, int16_t y, Color color, GraphicsEngine* engine);
    
    // 查找中文字符
    static const CharBitmap* find_chinese_char(uint32_t unicode, FontSize size);
};

// 字体数据现在使用namespace格式存储在fonts/目录中
// 不再需要外部声明，通过namespace访问

#endif // FONT_SYSTEM_H