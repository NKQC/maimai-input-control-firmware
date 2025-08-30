#include "font_data.h"
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_byte(addr) (*(const uint8_t*)(addr))
#define pgm_read_word(addr) (*(const uint16_t*)(addr))
#define strcmp_P strcmp
#endif

namespace FontData {
    
    // UTF-8字符解码函数
    static int utf8_char_length(const char* utf8_char) {
        unsigned char c = (unsigned char)utf8_char[0];
        if (c < 0x80) return 1;        // ASCII
        if ((c & 0xE0) == 0xC0) return 2;  // 2字节UTF-8
        if ((c & 0xF0) == 0xE0) return 3;  // 3字节UTF-8
        if ((c & 0xF8) == 0xF0) return 4;  // 4字节UTF-8
        return 1; // 错误情况，当作ASCII处理
    }
    
    // ASCII字符二分查找
    static int binary_search_ascii(char target) {
        const char* index_str = ASCII::ascii_index_string;
        int left = 0;
        int right = ASCII::FONT_CHAR_COUNT - 1;
        
        while (left <= right) {
            int mid = (left + right) / 2;
            char mid_char = pgm_read_byte(&index_str[mid]);
            
            if (mid_char == target) {
                return mid;
            } else if (mid_char < target) {
                left = mid + 1;
            } else {
                right = mid - 1;
            }
        }
        return -1; // 未找到
    }
    
    // 中文字符线性查找
    static int binary_search_chinese(const char* utf8_char) {
        const char* index_str = Chinese::chinese_index_string;
        int char_len = utf8_char_length(utf8_char);
        
        if (char_len < 2) return -1; // 中文字符至少2字节
        
        // 简化搜索：直接遍历查找匹配的字符
        for (int i = 0; i < Chinese::FONT_CHAR_COUNT; i++) {
            int pos = i * 3; // 每个中文字符3字节UTF-8
            
            // 比较UTF-8字符的前3个字节
            bool match = true;
            for (int j = 0; j < 3 && j < char_len; j++) {
                char index_byte = pgm_read_byte(&index_str[pos + j]);
                if (utf8_char[j] != index_byte) {
                    match = false;
                    break;
                }
            }
            
            if (match) {
                 return i; // 找到匹配的字符
             }
         }
         
         return -1; // 未找到
     }
    
    // 统一字库搜索接口 - 特殊性优先搜索
    FontSearchResult find_character(const char* utf8_char) {
        FontSearchResult result = {nullptr, 0, 0, false};
        
        if (!utf8_char || utf8_char[0] == '\0') {
            return result;
        }
        
        unsigned char first_byte = (unsigned char)utf8_char[0];
        
        // 优先判断是否为UTF-8多字节字符（有UTF-8头部）
        if (first_byte >= 0x80) {
            // 这是UTF-8多字节字符，优先在中文字库中搜索
            int index = binary_search_chinese(utf8_char);
            if (index >= 0) {
                result.bitmap_data = Chinese::char_bits[index];
                result.width = Chinese::FONT_WIDTH;
                result.height = Chinese::FONT_HEIGHT;
                result.found = true;
                return result;
            }
        } else {
            // 这是ASCII字符（0x00-0x7F），在ASCII字库中搜索
            int index = binary_search_ascii(utf8_char[0]);
            if (index >= 0) {
                result.bitmap_data = ASCII::char_bits[index];
                result.width = ASCII::FONT_WIDTH;
                result.height = ASCII::FONT_HEIGHT;
                result.found = true;
                return result;
            }
        }
        
        return result; // 未找到
    }
    
    // 创建默认方框字符(14x14)
    void create_default_box_char(uint8_t* buffer) {
        if (!buffer) return;
        
        // 清空缓冲区
        memset(buffer, 0, 28); // 14x14位图需要28字节
        
        // 绘制方框边框
        // 顶部和底部边框
        buffer[0] = 0xFF; buffer[1] = 0xE0;  // 第1行
        buffer[26] = 0xFF; buffer[27] = 0xE0; // 第14行
        
        // 左右边框
        for (int row = 1; row < 13; row++) {
            int offset = row * 2;
            buffer[offset] = 0x80;     // 左边框
            buffer[offset + 1] = 0x20; // 右边框
        }
    }
}