#ifndef FONT_SYSTEM_H
#define FONT_SYSTEM_H

#include <stdint.h>
#include "graphics_engine.h"

// FontSize枚举已在graphics_engine.h中定义

// 字符点阵数据结构
struct CharBitmap {
    uint8_t width;
    uint8_t height;
    const uint8_t* data;
};

// ASCII字符范围
#define ASCII_START 32
#define ASCII_END 126
#define ASCII_COUNT (ASCII_END - ASCII_START + 1)

// 常用中文字符数量（可根据需要调整）
#define CHINESE_CHAR_COUNT 500

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

// 字体数据声明（在font_data.cpp中定义）
extern const CharBitmap ascii_font_small[ASCII_COUNT];
extern const CharBitmap ascii_font_medium[ASCII_COUNT];
extern const CharBitmap ascii_font_large[ASCII_COUNT];

extern const ChineseCharMap chinese_font_small[CHINESE_CHAR_COUNT];
extern const ChineseCharMap chinese_font_medium[CHINESE_CHAR_COUNT];
extern const ChineseCharMap chinese_font_large[CHINESE_CHAR_COUNT];

// 默认字符点阵（用于未找到的字符）
extern const CharBitmap default_char_small;
extern const CharBitmap default_char_medium;
extern const CharBitmap default_char_large;

#endif // FONT_SYSTEM_H