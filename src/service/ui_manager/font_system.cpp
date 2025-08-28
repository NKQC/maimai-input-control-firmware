#include "font_system.h"
#include "graphics_engine.h"
#include <cstring>

void FontSystem::draw_ascii_char(char c, int16_t x, int16_t y, Color color, FontSize size, GraphicsEngine* engine) {
    if (!engine || c < ASCII_START || c > ASCII_END) return;
    
    const CharBitmap* bitmap = get_ascii_char_bitmap(c, size);
    if (bitmap) {
        draw_char_bitmap(bitmap, x, y, color, engine);
    }
}

int16_t FontSystem::get_ascii_char_width(char c, FontSize size) {
    if (c < ASCII_START || c > ASCII_END) return 0;
    
    const CharBitmap* bitmap = get_ascii_char_bitmap(c, size);
    return bitmap ? bitmap->width : 0;
}

const CharBitmap* FontSystem::get_ascii_char_bitmap(char c, FontSize size) {
    if (c < ASCII_START || c > ASCII_END) return nullptr;
    
    int index = c - ASCII_START;
    
    switch (size) {
        case FontSize::SMALL:
            return &ascii_font_small[index];
        case FontSize::MEDIUM:
            return &ascii_font_medium[index];
        case FontSize::LARGE:
            return &ascii_font_large[index];
        default:
            return &ascii_font_medium[index];
    }
}

void FontSystem::draw_chinese_char(uint32_t unicode, int16_t x, int16_t y, Color color, FontSize size, GraphicsEngine* engine) {
    if (!engine) return;
    
    const CharBitmap* bitmap = get_chinese_char_bitmap(unicode, size);
    if (bitmap) {
        draw_char_bitmap(bitmap, x, y, color, engine);
    } else {
        // 使用默认字符
        const CharBitmap* default_bitmap = nullptr;
        switch (size) {
            case FontSize::SMALL:
                default_bitmap = &default_char_small;
                break;
            case FontSize::MEDIUM:
                default_bitmap = &default_char_medium;
                break;
            case FontSize::LARGE:
                default_bitmap = &default_char_large;
                break;
        }
        if (default_bitmap) {
            draw_char_bitmap(default_bitmap, x, y, color, engine);
        }
    }
}

int16_t FontSystem::get_chinese_char_width(uint32_t unicode, FontSize size) {
    const CharBitmap* bitmap = get_chinese_char_bitmap(unicode, size);
    if (bitmap) {
        return bitmap->width;
    }
    
    // 返回默认中文字符宽度
    switch (size) {
        case FontSize::SMALL:
            return 8;
        case FontSize::MEDIUM:
            return 12;
        case FontSize::LARGE:
            return 16;
        default:
            return 12;
    }
}

const CharBitmap* FontSystem::get_chinese_char_bitmap(uint32_t unicode, FontSize size) {
    return find_chinese_char(unicode, size);
}

uint32_t FontSystem::utf8_to_unicode(const char* utf8) {
    if (!utf8) return 0;
    
    uint8_t c1 = utf8[0];
    
    // 单字节ASCII
    if ((c1 & 0x80) == 0) {
        return c1;
    }
    
    // 三字节UTF-8（中文字符）
    if ((c1 & 0xE0) == 0xE0) {
        uint8_t c2 = utf8[1];
        uint8_t c3 = utf8[2];
        
        if ((c2 & 0x80) == 0x80 && (c3 & 0x80) == 0x80) {
            return ((c1 & 0x0F) << 12) | ((c2 & 0x3F) << 6) | (c3 & 0x3F);
        }
    }
    
    // 二字节UTF-8
    if ((c1 & 0xE0) == 0xC0) {
        uint8_t c2 = utf8[1];
        if ((c2 & 0x80) == 0x80) {
            return ((c1 & 0x1F) << 6) | (c2 & 0x3F);
        }
    }
    
    return 0;
}

int FontSystem::utf8_char_length(const char* utf8) {
    if (!utf8) return 0;
    
    uint8_t c = utf8[0];
    
    if ((c & 0x80) == 0) return 1;      // 0xxxxxxx
    if ((c & 0xE0) == 0xC0) return 2;   // 110xxxxx
    if ((c & 0xF0) == 0xE0) return 3;   // 1110xxxx
    if ((c & 0xF8) == 0xF0) return 4;   // 11110xxx
    
    return 1; // 错误情况，返回1
}

const CharBitmap* FontSystem::get_font_data(FontSize size) {
    switch (size) {
        case FontSize::SMALL:
            return ascii_font_small;
        case FontSize::MEDIUM:
            return ascii_font_medium;
        case FontSize::LARGE:
            return ascii_font_large;
        default:
            return ascii_font_medium;
    }
}

void FontSystem::draw_char_bitmap(const CharBitmap* bitmap, int16_t x, int16_t y, Color color, GraphicsEngine* engine) {
    if (!bitmap || !bitmap->data || !engine) return;
    
    for (uint8_t row = 0; row < bitmap->height; row++) {
        for (uint8_t col = 0; col < bitmap->width; col++) {
            // 计算字节索引和位索引
            uint16_t bit_index = row * bitmap->width + col;
            uint16_t byte_index = bit_index / 8;
            uint8_t bit_offset = bit_index % 8;
            
            // 检查该位是否设置
            if (bitmap->data[byte_index] & (0x80 >> bit_offset)) {
                engine->set_pixel(x + col, y + row, color);
            }
        }
    }
}

const CharBitmap* FontSystem::find_chinese_char(uint32_t unicode, FontSize size) {
    const ChineseCharMap* char_map = nullptr;
    int count = CHINESE_CHAR_COUNT;
    
    switch (size) {
        case FontSize::SMALL:
            char_map = chinese_font_small;
            break;
        case FontSize::MEDIUM:
            char_map = chinese_font_medium;
            break;
        case FontSize::LARGE:
            char_map = chinese_font_large;
            break;
        default:
            char_map = chinese_font_medium;
            break;
    }
    
    if (!char_map) return nullptr;
    
    // 线性搜索（可以优化为二分搜索如果字符按Unicode排序）
    for (int i = 0; i < count; i++) {
        if (char_map[i].unicode == unicode) {
            return char_map[i].bitmap;
        }
        // 如果遇到0，说明到了数组末尾
        if (char_map[i].unicode == 0) {
            break;
        }
    }
    
    return nullptr;
}