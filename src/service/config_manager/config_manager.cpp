#include <mutex>
#include "config_manager.h"
#include "../input_manager/input_manager.h"
#include "../light_manager/light_manager.h"
#include "../ui_manager/ui_manager.h"
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>

// RP2040 LittleFS支持
#ifdef PICO_PLATFORM
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "LittleFS.h"
#endif

// 常量定义
const char* ConfigManager::CONFIG_FILE_PATH = "/config.json";

// 静态成员变量定义
bool ConfigManager::_initialized = false;
bool ConfigManager::_config_valid = false;
uint32_t ConfigManager::_error_count = 0;
config_map_t ConfigManager::_default_map;
config_map_t ConfigManager::_runtime_map;
std::map<std::string, std::string> ConfigManager::_string_cache;
std::vector<ConfigInitFunction> ConfigManager::_init_functions;
ConfigManager* ConfigManager::_instance = nullptr;

// 单例获取
ConfigManager* ConfigManager::getInstance() {
    if (!_instance) {
        _instance = new ConfigManager();
    }
    return _instance;
}

// 构造函数
ConfigManager::ConfigManager() {
}

// 析构函数
ConfigManager::~ConfigManager() {
    if (_initialized) {
        save_config();
    }
}

// 初始化默认配置并加锁只读
void ConfigManager::initialize_defaults() {
    
    _default_map.clear();
    _string_cache.clear();
    
    // 调用所有注册的初始化函数
    for (const auto& init_func : _init_functions) {
        init_func(_default_map);
    }
    
    // 调用各服务的默认配置注册函数
    inputmanager_register_default_configs(_default_map);
    lightmanager_register_default_configs(_default_map);
    uimanager_register_default_configs(_default_map);
}

// 计算config_map的CRC32校验码
uint32_t ConfigManager::calculate_crc32(const config_map_t& config_map) {
    std::string json_data;
    
    // 简化的JSON序列化用于CRC计算
    json_data += "{";
    bool first = true;
    for (const auto& pair : config_map) {
        if (!first) json_data += ",";
        first = false;
        
        const std::string& key = pair.first;
        const ConfigValue& value = pair.second;
        
        json_data += "\"" + key + "\":{";
        json_data += "\"type\":" + std::to_string(static_cast<int>(value.type)) + ",";
        json_data += "\"has_range\":" + std::string(value.has_range ? "true" : "false") + ",";
        
        switch (value.type) {
            case ConfigValueType::BOOL:
                json_data += "\"value\":" + std::string(value.bool_val ? "true" : "false");
                break;
            case ConfigValueType::INT8:
                json_data += "\"value\":" + std::to_string(value.int8_val);
                if (value.has_range) {
                    json_data += ",\"min\":" + std::to_string(value.min_val.int8_min);
                    json_data += ",\"max\":" + std::to_string(value.max_val.int8_max);
                }
                break;
            case ConfigValueType::UINT8:
                json_data += "\"value\":" + std::to_string(value.uint8_val);
                if (value.has_range) {
                    json_data += ",\"min\":" + std::to_string(value.min_val.uint8_min);
                    json_data += ",\"max\":" + std::to_string(value.max_val.uint8_max);
                }
                break;
            case ConfigValueType::UINT16:
                json_data += "\"value\":" + std::to_string(value.uint16_val);
                if (value.has_range) {
                    json_data += ",\"min\":" + std::to_string(value.min_val.uint16_min);
                    json_data += ",\"max\":" + std::to_string(value.max_val.uint16_max);
                }
                break;
            case ConfigValueType::UINT32:
                json_data += "\"value\":" + std::to_string(value.uint32_val);
                if (value.has_range) {
                    json_data += ",\"min\":" + std::to_string(value.min_val.uint32_min);
                    json_data += ",\"max\":" + std::to_string(value.max_val.uint32_max);
                }
                break;
            case ConfigValueType::FLOAT:
                json_data += "\"value\":" + std::to_string(value.float_val);
                if (value.has_range) {
                    json_data += ",\"min\":" + std::to_string(value.min_val.float_min);
                    json_data += ",\"max\":" + std::to_string(value.max_val.float_max);
                }
                break;
            case ConfigValueType::STRING:
                json_data += "\"value\":\"" + value.string_val + "\"";
                break;
        }
        json_data += "}";
    }
    json_data += "}";
    
    return ConfigCRC::calculate_crc32(json_data);
}

// LittleFS初始化
bool ConfigManager::littlefs_init() {
#ifdef PICO_PLATFORM
    if (!LittleFS.begin()) {
        return false;
    }
    return true;
#else
    return false;
#endif
}

// 检查文件是否存在
bool ConfigManager::littlefs_file_exists(const std::string& path) {
#ifdef PICO_PLATFORM
    return LittleFS.exists(path.c_str());
#else
    return false;
#endif
}

// 读取文件
bool ConfigManager::littlefs_read_file(const std::string& path, std::string& content) {
#ifdef PICO_PLATFORM
    if (!LittleFS.exists(path.c_str())) {
        return false;
    }
    
    File file = LittleFS.open(path.c_str(), "r");
    if (!file) {
        return false;
    }
    
    content.clear();
    while (file.available()) {
        content += (char)file.read();
    }
    file.close();
    return true;
#else
    return false;
#endif
}

// 写入文件
bool ConfigManager::littlefs_write_file(const std::string& path, const std::string& content) {
#ifdef PICO_PLATFORM
    File file = LittleFS.open(path.c_str(), "w");
    if (!file) {
        return false;
    }
    
    size_t written = file.write(reinterpret_cast<const uint8_t*>(content.c_str()), content.length());
    file.close();
    return written == content.length();
#else
    return false;
#endif
}

// 私有接口：保存config_map到文件
bool ConfigManager::config_save(const config_map_t* config_map, const std::string& file_path) {
    if (!config_map) {
        return false;
    }
        
        // 构建JSON字符串
        std::string json_data = "{";
        bool first = true;
        
        // 序列化配置数据
        for (const auto& pair : *config_map) {
            if (!first) json_data += ",";
            first = false;
            
            const std::string& key = pair.first;
            const ConfigValue& value = pair.second;
            
            json_data += "\"" + key + "\":{";
            json_data += "\"type\":" + std::to_string(static_cast<int>(value.type));
            
            switch (value.type) {
                case ConfigValueType::BOOL:
                    json_data += ",\"value\":" + std::string(value.bool_val ? "true" : "false");
                    break;
                case ConfigValueType::INT8:
                    json_data += ",\"value\":" + std::to_string(value.int8_val);
                    if (value.has_range) {
                        json_data += ",\"min\":" + std::to_string(value.min_val.int8_min);
                        json_data += ",\"max\":" + std::to_string(value.max_val.int8_max);
                    }
                    break;
                case ConfigValueType::UINT8:
                    json_data += ",\"value\":" + std::to_string(value.uint8_val);
                    if (value.has_range) {
                        json_data += ",\"min\":" + std::to_string(value.min_val.uint8_min);
                        json_data += ",\"max\":" + std::to_string(value.max_val.uint8_max);
                    }
                    break;
                case ConfigValueType::UINT16:
                    json_data += ",\"value\":" + std::to_string(value.uint16_val);
                    if (value.has_range) {
                        json_data += ",\"min\":" + std::to_string(value.min_val.uint16_min);
                        json_data += ",\"max\":" + std::to_string(value.max_val.uint16_max);
                    }
                    break;
                case ConfigValueType::UINT32:
                    json_data += ",\"value\":" + std::to_string(value.uint32_val);
                    if (value.has_range) {
                        json_data += ",\"min\":" + std::to_string(value.min_val.uint32_min);
                        json_data += ",\"max\":" + std::to_string(value.max_val.uint32_max);
                    }
                    break;
                case ConfigValueType::FLOAT:
                    json_data += ",\"value\":" + std::to_string(value.float_val);
                    if (value.has_range) {
                        json_data += ",\"min\":" + std::to_string(value.min_val.float_min);
                        json_data += ",\"max\":" + std::to_string(value.max_val.float_max);
                    }
                    break;
                case ConfigValueType::STRING:
                    json_data += ",\"value\":\"" + value.string_val + "\"";
                    break;
            }
            json_data += "}";
        }
        
        // 计算并添加CRC
        uint32_t crc = calculate_crc32(*config_map);
        json_data += ",\"" + std::string(CONFIG_KEY_CRC) + "\":" + std::to_string(crc);
        json_data += "}";
        
        // 写入文件
        return littlefs_write_file(file_path, json_data);
}

// 简化的JSON解析器
static std::map<std::string, std::string> parse_simple_json(const std::string& json) {
    std::map<std::string, std::string> result;
    
    size_t pos = 0;
    while (pos < json.length()) {
        // 查找键
        size_t key_start = json.find('"', pos);
        if (key_start == std::string::npos) break;
        key_start++;
        
        size_t key_end = json.find('"', key_start);
        if (key_end == std::string::npos) break;
        
        std::string key = json.substr(key_start, key_end - key_start);
        
        // 查找值
        size_t colon = json.find(':', key_end);
        if (colon == std::string::npos) break;
        
        size_t value_start = colon + 1;
        while (value_start < json.length() && (json[value_start] == ' ' || json[value_start] == '\t')) {
            value_start++;
        }
        
        size_t value_end;
        if (json[value_start] == '{') {
            // 对象值
            int brace_count = 1;
            value_end = value_start + 1;
            while (value_end < json.length() && brace_count > 0) {
                if (json[value_end] == '{') brace_count++;
                else if (json[value_end] == '}') brace_count--;
                value_end++;
            }
        } else {
            // 简单值
            value_end = json.find(',', value_start);
            if (value_end == std::string::npos) {
                value_end = json.find('}', value_start);
            }
        }
        
        if (value_end == std::string::npos) break;
        
        std::string value = json.substr(value_start, value_end - value_start);
        result[key] = value;
        
        pos = value_end + 1;
    }
    
    return result;
}

// 从对象字符串中提取值
static std::string extract_value_from_object(const std::string& obj, const std::string& field) {
    size_t field_pos = obj.find('"' + field + '"');
    if (field_pos == std::string::npos) return "";
    
    size_t colon = obj.find(':', field_pos);
    if (colon == std::string::npos) return "";
    
    size_t value_start = colon + 1;
    while (value_start < obj.length() && (obj[value_start] == ' ' || obj[value_start] == '\t')) {
        value_start++;
    }
    
    size_t value_end;
    if (obj[value_start] == '"') {
        value_start++;
        value_end = obj.find('"', value_start);
    } else {
        value_end = obj.find(',', value_start);
        if (value_end == std::string::npos) {
            value_end = obj.find('}', value_start);
        }
    }
    
    if (value_end == std::string::npos) return "";
    
    return obj.substr(value_start, value_end - value_start);
}

// 私有接口：从文件读取config_map
void ConfigManager::config_read(config_map_t* config_map, const std::string& file_path) {
    if (!config_map) {
        return;
    }
    
    std::string json_content;
    if (!littlefs_read_file(file_path, json_content)) {
        return;
    }
    
    if (json_content.empty()) {
        return;
    }
        
        // 解析JSON
        auto json_map = parse_simple_json(json_content);
        
        config_map->clear();
        uint32_t stored_crc = 0;
        
        // 反序列化配置数据
        for (const auto& kv : json_map) {
            const std::string& key = kv.first;
            const std::string& obj_str = kv.second;
            
            // 处理CRC键
            if (key == CONFIG_KEY_CRC) {
                stored_crc = std::stoul(obj_str);
                continue;
            }
            
            // 解析配置对象
            std::string type_str = extract_value_from_object(obj_str, "type");
            std::string value_str = extract_value_from_object(obj_str, "value");
            
            if (type_str.empty() || value_str.empty()) {
                continue;
            }
            
            ConfigValueType type = static_cast<ConfigValueType>(std::stoi(type_str));
            ConfigValue new_value;
            
            switch (type) {
                case ConfigValueType::BOOL:
                    new_value = ConfigValue(value_str == "true");
                    break;
                case ConfigValueType::INT8: {
                    int8_t val = static_cast<int8_t>(std::stoi(value_str));
                    std::string min_str = extract_value_from_object(obj_str, "min");
                    std::string max_str = extract_value_from_object(obj_str, "max");
                    if (!min_str.empty() && !max_str.empty()) {
                        new_value = ConfigValue(val, static_cast<int8_t>(std::stoi(min_str)), static_cast<int8_t>(std::stoi(max_str)));
                    } else {
                        new_value = ConfigValue(val);
                    }
                    break;
                }
                case ConfigValueType::UINT8: {
                    uint8_t val = static_cast<uint8_t>(std::stoul(value_str));
                    std::string min_str = extract_value_from_object(obj_str, "min");
                    std::string max_str = extract_value_from_object(obj_str, "max");
                    if (!min_str.empty() && !max_str.empty()) {
                        new_value = ConfigValue(val, static_cast<uint8_t>(std::stoul(min_str)), static_cast<uint8_t>(std::stoul(max_str)));
                    } else {
                        new_value = ConfigValue(val);
                    }
                    break;
                }
                case ConfigValueType::UINT16: {
                    uint16_t val = static_cast<uint16_t>(std::stoul(value_str));
                    std::string min_str = extract_value_from_object(obj_str, "min");
                    std::string max_str = extract_value_from_object(obj_str, "max");
                    if (!min_str.empty() && !max_str.empty()) {
                        new_value = ConfigValue(val, static_cast<uint16_t>(std::stoul(min_str)), static_cast<uint16_t>(std::stoul(max_str)));
                    } else {
                        new_value = ConfigValue(val);
                    }
                    break;
                }
                case ConfigValueType::UINT32: {
                    uint32_t val = static_cast<uint32_t>(std::stoul(value_str));
                    std::string min_str = extract_value_from_object(obj_str, "min");
                    std::string max_str = extract_value_from_object(obj_str, "max");
                    if (!min_str.empty() && !max_str.empty()) {
                        new_value = ConfigValue(val, static_cast<uint32_t>(std::stoul(min_str)), static_cast<uint32_t>(std::stoul(max_str)));
                    } else {
                        new_value = ConfigValue(val);
                    }
                    break;
                }
                case ConfigValueType::FLOAT: {
                    float val = std::stof(value_str);
                    std::string min_str = extract_value_from_object(obj_str, "min");
                    std::string max_str = extract_value_from_object(obj_str, "max");
                    if (!min_str.empty() && !max_str.empty()) {
                        new_value = ConfigValue(val, std::stof(min_str), std::stof(max_str));
                    } else {
                        new_value = ConfigValue(val);
                    }
                    break;
                }
                case ConfigValueType::STRING:
                    new_value = ConfigValue(value_str);
                    break;
                default:
                    continue;
            }
            
            (*config_map)[key] = new_value;
        }
        
        // CRC校验
        uint32_t calculated_crc = calculate_crc32(*config_map);
        if (calculated_crc != stored_crc) {
            config_map->clear();
            return;
        }
}

// 异常处理/重置流程
void ConfigManager::handle_config_exception() {
    _error_count++;
    
    // 使用默认map保存配置
    if (config_save(&_default_map, CONFIG_FILE_PATH)) {
        // 复制默认map到运行时map
        _runtime_map = _default_map;
        _config_valid = true;
    } else {
        _config_valid = false;
    }
}

// 初始化配置模块
bool ConfigManager::initialize() {
    if (_initialized) {
        return true;
    }
    
    // 初始化LittleFS
    if (!littlefs_init()) {
        return false;
    }
    
    _initialized = true;
    _config_valid = false;
    
    // 初始化默认配置
    initialize_defaults();
    
    // 尝试读取配置文件
    config_read(&_runtime_map, CONFIG_FILE_PATH);
    
    // 合并缺失的默认配置键
    for (const auto& default_pair : _default_map) {
        if (_runtime_map.find(default_pair.first) == _runtime_map.end()) {
            _runtime_map[default_pair.first] = default_pair.second;
        }
    }
    
    _config_valid = true;
    
    return true;
}

// 反初始化配置模块
void ConfigManager::deinit() {
    _initialized = false;
    _config_valid = false;
    _runtime_map.clear();
    _default_map.clear();
    _string_cache.clear();
    _init_functions.clear();
    _error_count = 0;
}

// 检查配置键是否存在
bool ConfigManager::has_key(const std::string& key) {
    // 先检查运行时map
    if (_runtime_map.find(key) != _runtime_map.end()) {
        return true;
    }
    
    // 再检查默认map
    return _default_map.find(key) != _default_map.end();
}

// 读取配置值
ConfigValue ConfigManager::get(const std::string& key) {
    // 先从运行时map中查找
    auto runtime_it = _runtime_map.find(key);
    if (runtime_it != _runtime_map.end()) {
        return runtime_it->second;
    }
    
    // 从默认map中查找
    auto default_it = _default_map.find(key);
    if (default_it != _default_map.end()) {
        // 添加到运行时map
        _runtime_map[key] = default_it->second;
        return default_it->second;
    }
    
    // 键未注册，返回默认的bool值
    return ConfigValue(false);
}

// 设置配置值
void ConfigManager::set(const std::string& key, const ConfigValue& value) {
    // 先检查运行时map
    auto runtime_it = _runtime_map.find(key);
    if (runtime_it != _runtime_map.end()) {
        // 复制范围限制信息
        ConfigValue new_value = value;
        if (runtime_it->second.has_range && new_value.type == runtime_it->second.type) {
            new_value.copy_range_from(runtime_it->second);
            new_value.clamp_value();
        }
        _runtime_map[key] = new_value;
        
        // 清除字符串缓存
        auto cache_it = _string_cache.find(key);
        if (cache_it != _string_cache.end()) {
            _string_cache.erase(cache_it);
        }
        return;
    }
    
    // 检查默认map
    auto default_it = _default_map.find(key);
    if (default_it != _default_map.end()) {
        // 从默认map添加到运行时map
        ConfigValue new_value = value;
        if (default_it->second.has_range && new_value.type == default_it->second.type) {
            new_value.copy_range_from(default_it->second);
            new_value.clamp_value();
        }
        _runtime_map[key] = new_value;
        return;
    }
    return;
}

// 便捷类型获取接口实现
bool ConfigManager::get_bool(const std::string& key) {
    ConfigValue val = get(key);
    if (val.type != ConfigValueType::BOOL) {
        return false;
    }
    return val.bool_val;
}

int8_t ConfigManager::get_int8(const std::string& key) {
    ConfigValue val = get(key);
    if (val.type != ConfigValueType::INT8) {
        return 0;
    }
    return val.int8_val;
}

uint8_t ConfigManager::get_uint8(const std::string& key) {
    ConfigValue val = get(key);
    if (val.type != ConfigValueType::UINT8) {
        return 0;
    }
    return val.uint8_val;
}

uint16_t ConfigManager::get_uint16(const std::string& key) {
    ConfigValue val = get(key);
    if (val.type != ConfigValueType::UINT16) {
        return 0;
    }
    return val.uint16_val;
}

uint32_t ConfigManager::get_uint32(const std::string& key) {
    ConfigValue val = get(key);
    if (val.type != ConfigValueType::UINT32) {
        return 0;
    }
    return val.uint32_val;
}

float ConfigManager::get_float(const std::string& key) {
    ConfigValue val = get(key);
    if (val.type != ConfigValueType::FLOAT) {
        return 0.0f;
    }
    return val.float_val;
}

std::string ConfigManager::get_string(const std::string& key) {
    ConfigValue val = get(key);
    if (val.type != ConfigValueType::STRING) {
        return "";
    }
    return val.string_val;
}

const char* ConfigManager::get_cstring(const std::string& key) {
    // 使用缓存避免返回临时对象的指针
    auto cache_it = _string_cache.find(key);
    if (cache_it != _string_cache.end()) {
        return cache_it->second.c_str();
    }
    
    std::string str = get_string(key);
    _string_cache[key] = str;
    return _string_cache[key].c_str();
}

// 便捷类型设置接口实现
void ConfigManager::set_bool(const std::string& key, bool value) {
    set(key, ConfigValue(value));
}

void ConfigManager::set_int8(const std::string& key, int8_t value) {
    set(key, ConfigValue(value));
}

void ConfigManager::set_uint8(const std::string& key, uint8_t value) {
    set(key, ConfigValue(value));
}

void ConfigManager::set_uint16(const std::string& key, uint16_t value) {
    set(key, ConfigValue(value));
}

void ConfigManager::set_uint32(const std::string& key, uint32_t value) {
    set(key, ConfigValue(value));
}

void ConfigManager::set_float(const std::string& key, float value) {
    set(key, ConfigValue(value));
}

void ConfigManager::set_string(const std::string& key, const std::string& value) {
    set(key, ConfigValue(value));
}

// 批量操作接口
std::map<std::string, ConfigValue> ConfigManager::get_all() {
    return _runtime_map;
}

void ConfigManager::set_batch(const std::map<std::string, ConfigValue>& values) {
    for (const auto& pair : values) {
        set(pair.first, pair.second);
    }
}

// 配置分组接口
std::map<std::string, ConfigValue> ConfigManager::get_group(const std::string& prefix) {
    std::map<std::string, ConfigValue> result;
    for (const auto& pair : _runtime_map) {
        if (pair.first.substr(0, prefix.length()) == prefix) {
            result[pair.first] = pair.second;
        }
    }
    return result;
}

void ConfigManager::set_group(const std::string& prefix, const std::map<std::string, ConfigValue>& values) {
    for (const auto& pair : values) {
        if (pair.first.substr(0, prefix.length()) == prefix) {
            set(pair.first, pair.second);
        }
    }
}

// 配置初始化函数注册
bool ConfigManager::register_init_function(ConfigInitFunction func) {
    _init_functions.push_back(func);
    return true;
}

// 保存配置到文件
bool ConfigManager::save_config() {
    bool result = config_save(&_runtime_map, CONFIG_FILE_PATH);
    if (!result) {
        _error_count++;
    }
    return result;
}

// 重置到默认配置
bool ConfigManager::reset_to_defaults() {
    _runtime_map = _default_map;
    return save_config();
}

// 调试接口
void ConfigManager::debug_print_all_configs() {
    // 在RP2040上可以通过串口输出调试信息
    // 这里简化实现
}

// 字符串验证
bool ConfigManager::is_valid_string(const std::string& str) {
    // 简单的字符串验证
    return !str.empty() && str.length() < 256;
}
