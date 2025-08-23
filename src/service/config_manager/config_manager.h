#pragma once

#include <string>
#include <map>
#include <mutex>
#ifdef PICO_PLATFORM
#include "pico/mutex.h"
#endif
#include <functional>
#include "config_types.h"
#include "config_crc.h"
#include <mutex>

// 配置键常量
#define CONFIG_KEY_CRC "__crc32__"

// 配置初始化函数类型
using ConfigInitFunction = std::function<void(config_map_t&)>;

/**
 * ConfigManager - 配置文件管理类
 * 严格按照main.cpp中的架构要求实现
 * 使用双MAP架构：默认配置MAP + 运行时配置MAP
 * 支持LittleFS存储，适配RP2040平台
 */
class ConfigManager {
public:
    // 单例模式
    static ConfigManager* getInstance();
    
    // 析构函数
    ~ConfigManager();
    
    // 初始化配置模块
    static bool initialize();
    
    // 反初始化配置模块
    static void deinit();
    
    // 保存配置到文件
    static bool save_config();
    
    // 重置到默认配置
    static bool reset_to_defaults();
    
    // 获取配置状态
    static bool is_config_valid() { return _config_valid; }
    static uint32_t get_error_count() { return _error_count; }
    
    // 核心配置接口
    static bool has_key(const std::string& key);
    static ConfigValue get(const std::string& key);
    static void set(const std::string& key, const ConfigValue& value);
    
    // 便捷类型获取接口
    static bool get_bool(const std::string& key);
    static int8_t get_int8(const std::string& key);
    static uint8_t get_uint8(const std::string& key);
    static uint16_t get_uint16(const std::string& key);
    static uint32_t get_uint32(const std::string& key);
    static float get_float(const std::string& key);
    static std::string get_string(const std::string& key);
    static const char* get_cstring(const std::string& key);
    
    // 便捷类型设置接口
    static void set_bool(const std::string& key, bool value);
    static void set_int8(const std::string& key, int8_t value);
    static void set_uint8(const std::string& key, uint8_t value);
    static void set_uint16(const std::string& key, uint16_t value);
    static void set_uint32(const std::string& key, uint32_t value);
    static void set_float(const std::string& key, float value);
    static void set_string(const std::string& key, const std::string& value);
    
    // 批量操作接口
    static std::map<std::string, ConfigValue> get_all();
    static void set_batch(const std::map<std::string, ConfigValue>& values);
    
    // 配置分组接口
    static std::map<std::string, ConfigValue> get_group(const std::string& prefix);
    static void set_group(const std::string& prefix, const std::map<std::string, ConfigValue>& values);
    
    // 配置初始化函数注册
    static bool register_init_function(ConfigInitFunction func);
    
    // 调试接口
    static void debug_print_all_configs();

private:
    // 私有构造函数
    ConfigManager();
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    
    // 常量定义
    static const char* CONFIG_FILE_PATH;
    
    // 静态成员变量 - 双map架构
    static bool _initialized;
    static bool _config_valid;
    static uint32_t _error_count;
    static config_map_t _default_map;      // 默认配置map（只读）
    static config_map_t _runtime_map;      // 运行时配置map（可读写）
    static std::map<std::string, std::string> _string_cache;
    // static std::mutex _default_map_mutex;  // 默认map的读写锁
    static std::vector<ConfigInitFunction> _init_functions; // 初始化函数列表
    
    // 单例实例
    static ConfigManager* _instance;
    
    // 私有文件操作接口（不对外访问）
    static bool config_save(const config_map_t* config_map, const std::string& file_path);
    static void config_read(config_map_t* config_map, const std::string& file_path);
    
    // 辅助函数
    static void initialize_defaults();
    static void handle_config_exception();
    static uint32_t calculate_crc32(const config_map_t& config_map);
    static bool is_valid_string(const std::string& str);
    
    // LittleFS操作
    static bool littlefs_init();
    static bool littlefs_file_exists(const std::string& path);
    static bool littlefs_read_file(const std::string& path, std::string& content);
    static bool littlefs_write_file(const std::string& path, const std::string& content);
    
    // 预定义配置注册函数

};
