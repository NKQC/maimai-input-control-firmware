#pragma once

#include <cstdint>
#include <string>

// CRC32计算类
class ConfigCRC {
public:
    // 计算字符串的CRC32
    static uint32_t calculate_crc32(const std::string& data);
    
    // 计算字节数组的CRC32
    static uint32_t calculate_crc32(const uint8_t* data, size_t length);
    
private:
    // CRC32查找表
    static const uint32_t crc32_table[256];
    
    // 初始化CRC32查找表
    static void init_crc32_table();
    
    // 表是否已初始化
    static bool table_initialized;
};
