#pragma once

#include <string>
#include <map>
#include <functional>
#include "config_types.h"
#include "config_crc.h"

#ifdef PICO_PLATFORM
#include "LittleFS.h"
#endif

// 配置键常量
#define CONFIG_KEY_CRC "__crc32__"

// 配置初始化函数类型
using ConfigInitFunction = std::function<void(config_map_t&)>;

// 流式JSON解析器状态
enum class JsonParseState {
    EXPECT_OBJECT_START,    // 期待对象开始 '{'
    EXPECT_KEY,             // 期待键
    EXPECT_COLON,           // 期待冒号 ':'
    EXPECT_VALUE,           // 期待值
    EXPECT_COMMA_OR_END,    // 期待逗号或结束
    PARSING_OBJECT_VALUE,   // 解析对象值
    COMPLETE                // 解析完成
};

// 流式JSON解析器
class StreamingJsonParser {
public:
    StreamingJsonParser();
    void reset();
    bool parse_chunk(const std::string& chunk);
    bool is_complete() const { return state_ == JsonParseState::COMPLETE; }
    std::map<std::string, std::string> get_parsed_data() const { return parsed_data_; }
    
private:
    JsonParseState state_;
    std::string buffer_;
    std::string current_key_;
    std::string current_value_;
    int brace_depth_;
    std::map<std::string, std::string> parsed_data_;
    
    void process_buffer();
    bool extract_key();
    bool extract_value();
    void skip_whitespace(size_t& pos);
};

// 流式JSON序列化器
class StreamingJsonSerializer {
public:
    StreamingJsonSerializer();
    ~StreamingJsonSerializer();
    bool start_object();
    bool add_key_value(const char* key, const char* value);
    bool end_object();
    bool is_complete() const { return complete_; }
    
private:
    std::string buffer_;
    bool first_item_;
    bool complete_;
    bool file_opened_;
    static const size_t BUFFER_SIZE = 256;
    
#ifdef PICO_PLATFORM
    File file_handle_;
#endif
    
    // 静态分配的uint8_t缓冲区
    static uint8_t write_buffer_[BUFFER_SIZE];
    
    bool open_file();
    bool close_file();
    bool flush_buffer();
    bool write_to_file(const char* data, size_t length);
};

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
    static bool save_config_task();
    static void save_config();  // 置位保存信号
    
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
    
    // 动态设置接口 - 允许设置未注册的键（字符串限定）
    static void set_string_dynamic(const std::string& key, const std::string& value);
    static bool has_string_dynamic(const std::string& key);
    static std::string get_string_dynamic(const std::string& key, const std::string& default_value = "");
    
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
    static void enable_debug_output(bool enable);
    
    // 内部日志接口
    static void log_debug(const std::string& message);
    static void log_info(const std::string& message);
    static void log_error(const std::string& message);
    
    // LittleFS状态访问
    static bool is_littlefs_ready();

    // 文件操作封装方法（供StreamingJsonSerializer使用）
    static bool open_file_for_read(File& file);
    static bool open_file_for_write(File& file);
    static void close_file(File& file);

private:
    // 私有构造函数
    ConfigManager();
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    // LittleFS操作
    static bool littlefs_init();
    static bool littlefs_file_exists();
    
    // 常量定义
    static const char CONFIG_FILE_PATH[];
    
    // 静态成员变量 - 双map架构
    static bool _initialized;
    static bool _config_valid;
    static uint32_t _error_count;
    static config_map_t _default_map;      // 默认配置map（只读）
    static config_map_t _runtime_map;      // 运行时配置map（可读写）
    static std::map<std::string, std::string> _string_cache;
    static std::vector<ConfigInitFunction> _init_functions; // 初始化函数列表
    
    // 核心私有成员
    static bool _littlefs_ready;            // LittleFS就绪状态
    static volatile bool _save_requested;    // 保存请求信号
    
    // 单例实例
    static ConfigManager* _instance;

    // DEBUG
    static bool _debug_output_enabled;
    
    // 私有文件操作接口（不对外访问）
    static bool config_save(const config_map_t* config_map);
    static bool config_read(config_map_t* config_map);
    
    // 辅助函数
    static void initialize_defaults();
    static uint32_t calculate_crc32(const config_map_t& config_map);
    static bool is_valid_string(const std::string& str);
    
    // 流式读写操作
    static bool config_read_streaming(config_map_t* config_map);
    static bool config_save_streaming(const config_map_t* config_map);
    static void process_config_item(config_map_t* config_map, const std::string& key, const std::string& obj_str);
    static std::string serialize_config_value(const ConfigValue& value);
    
};
