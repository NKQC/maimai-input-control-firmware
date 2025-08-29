#include "font_system.h"
#include "graphics_engine.h"
#include "fonts/font_data.h"
#include <cstring>
#include <avr/pgmspace.h>

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
    
    // 使用新的namespace格式字库
    return FontData::ASCII::get_char_data(c);
}

void FontSystem::draw_chinese_char(uint32_t unicode, int16_t x, int16_t y, Color color, FontSize size, GraphicsEngine* engine) {
    if (!engine) return;
    
    const CharBitmap* bitmap = get_chinese_char_bitmap(unicode, size);
    if (bitmap) {
        draw_char_bitmap(bitmap, x, y, color, engine);
    } else {
        // 使用默认字符 - 使用空格字符作为默认
        const CharBitmap* default_bitmap = FontData::ASCII::get_char_data(' ');
        if (!default_bitmap) {
            // 如果找不到空格字符，创建一个简单的默认字符
            static const uint8_t default_data[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
            static const CharBitmap default_char = {8, 14, default_data};
            default_bitmap = &default_char;
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
    
    // 返回默认中文字符宽度 - 统一使用12px宽度
    // 由于FontSize枚举已统一为14px高度，这里也统一宽度
    (void)size; // 避免未使用参数警告
    return 12; // 统一使用12px宽度，与中文字体文件中的FONT_WIDTH保持一致
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
    // 返回默认的ASCII字符数据 - 这个函数可能需要重新设计
    return FontData::ASCII::get_char_data('A'); // 临时返回字符A作为示例
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
    // 使用新的namespace格式中文字库
    const ChineseChar* chinese_char = FontData::Chinese::find_chinese_char(unicode);
    
    if (chinese_char) {
        // 创建临时CharBitmap结构
        static CharBitmap temp_bitmap;
        temp_bitmap.width = chinese_char->width;
        temp_bitmap.height = chinese_char->height;
        temp_bitmap.data = chinese_char->data;
        return &temp_bitmap;
    }
    
    // 如果找不到字符，返回默认字符
    const ChineseChar* default_char = FontData::Chinese::get_default_chinese_char();
    if (default_char) {
        static CharBitmap default_bitmap;
        default_bitmap.width = default_char->width;
        default_bitmap.height = default_char->height;
        default_bitmap.data = default_char->data;
        return &default_bitmap;
    }
    
    return nullptr;
}