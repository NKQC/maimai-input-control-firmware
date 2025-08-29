#ifndef FONT_DATA_H
#define FONT_DATA_H

#include <stdint.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
// For non-AVR platforms (like RP2040), define PROGMEM as empty
#define PROGMEM
#endif

// 字符点阵数据结构
struct CharBitmap {
    uint8_t width;
    uint8_t height;
    const uint8_t* data;
};

// ASCII字符范围定义
#define ASCII_START 32
#define ASCII_END 126
#define ASCII_COUNT (ASCII_END - ASCII_START + 1)

// 中文字符数量定义
#define CHINESE_CHAR_COUNT 575

// 中文字符映射结构
struct ChineseChar {
    uint16_t unicode;     // Unicode编码
    uint8_t width;        // 字符宽度
    uint8_t height;       // 字符高度
    const uint8_t* data;  // 点阵数据指针
};

// 字体数据命名空间
namespace FontData {
    // ASCII字体数据命名空间
    namespace ASCII {
        // 字体属性
        extern const uint8_t FONT_WIDTH;
        extern const uint8_t FONT_HEIGHT;
        extern const uint8_t FONT_BPP;
        extern const uint8_t FONT_PITCH;
        extern const uint8_t FONT_CHAR_COUNT;
        extern const bool FONT_MONOSPACE;
        
        // 字符点阵数据数组
        extern const uint8_t char_bits[ASCII_COUNT][14];
        
        // 获取ASCII字符数据
        const CharBitmap* get_char_data(char c);
        
        // 获取字符索引
        int get_char_index(char c);
    }
    
    // 中文字体数据命名空间
    namespace Chinese {
        // 字体属性
        extern const uint8_t FONT_WIDTH;
        extern const uint8_t FONT_HEIGHT;
        extern const uint8_t FONT_BPP;
        extern const uint8_t FONT_PITCH;
        extern const uint16_t FONT_CHAR_COUNT;
        extern const bool FONT_MONOSPACE;
        
        // 字符点阵数据数组
        extern const uint8_t char_bits[CHINESE_CHAR_COUNT][28];
        
        // 中文字符映射数组
        extern const ChineseChar chinese_chars[CHINESE_CHAR_COUNT];
        
        // 查找中文字符
        const ChineseChar* find_chinese_char(uint16_t unicode);
        
        // 获取默认中文字符
        const ChineseChar* get_default_chinese_char();
        
        // 获取字符索引
        int get_char_index(uint16_t unicode);
    }
}

#endif // FONT_DATA_H