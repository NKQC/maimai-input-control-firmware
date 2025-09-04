#include "config_manager.h"
#include "../input_manager/input_manager.h"
#include "../light_manager/light_manager.h"
#include "../ui_manager/ui_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
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
#endif

// 常量定义
const char ConfigManager::CONFIG_FILE_PATH[] = "/config.bin";

// 静态成员变量定义
bool ConfigManager::_initialized = false;
bool ConfigManager::_config_valid = false;
uint32_t ConfigManager::_error_count = 0;
config_map_t ConfigManager::_default_map;
config_map_t ConfigManager::_runtime_map;
std::map<std::string, std::string> ConfigManager::_string_cache;
std::vector<ConfigInitFunction> ConfigManager::_init_functions;
bool ConfigManager::_littlefs_ready = false;
volatile bool ConfigManager::_save_requested = false;
ConfigManager* ConfigManager::_instance = nullptr;

// StreamingJsonSerializer静态缓冲区定义
uint8_t StreamingJsonSerializer::write_buffer_[StreamingJsonSerializer::BUFFER_SIZE];

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
        save_config_task();
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
    std::ostringstream oss;
    
    log_debug("=== CRC计算开始 ===");
    log_debug("配置项总数: " + std::to_string(config_map.size()));
    
    // 按键名排序以确保一致性，但排除CRC字段
    std::vector<std::pair<std::string, ConfigValue>> sorted_items;
    for (const auto& kv : config_map) {
        // 排除CRC字段，避免循环依赖
        if (kv.first != CONFIG_KEY_CRC) {
            sorted_items.push_back(kv);
            log_debug("包含配置项: " + kv.first);
        } else {
            log_debug("排除CRC字段: " + kv.first);
        }
    }
    
    log_debug("用于CRC计算的配置项数量: " + std::to_string(sorted_items.size()));
    
    std::sort(sorted_items.begin(), sorted_items.end(), 
              [](const std::pair<std::string, ConfigValue>& a, const std::pair<std::string, ConfigValue>& b) {
                  return a.first < b.first;
              });
    
    oss << "{";
    bool first = true;
    for (const auto& kv : sorted_items) {
        const std::string& key = kv.first;
        const ConfigValue& value = kv.second;
        
        if (!first) {
            oss << ",";
        }
        first = false;
        
        std::string serialized_value = serialize_config_value(value);
        
        oss << "\"" << key << "\":";
        oss << serialized_value;
    }
    oss << "}";
    
    std::string data = oss.str();
    log_debug("数据长度: " + std::to_string(data.length()) + " 字节");
    
    // 格式化输出CRC计算数据，每行不超过200字符
    log_debug("=== CRC计算数据开始 ===");
    size_t pos = 0;
    const size_t max_line_length = 160; // 留一些余量给前缀
    while (pos < data.length()) {
        size_t end_pos = std::min(pos + max_line_length, data.length());
        std::string line = data.substr(pos, end_pos - pos);
        log_debug("CRC_DATA[" + std::to_string(pos) + "-" + std::to_string(end_pos-1) + "]: " + line);
        pos = end_pos;
    }
    log_debug("=== CRC计算数据结束 ===");
    
    uint32_t crc_result = ConfigCRC::calculate_crc32(data);
    log_debug("计算得到的CRC32值: " + std::to_string(crc_result));
    log_debug("=== CRC计算结束 ===");
    
    return crc_result;
}

// LittleFS初始化
bool ConfigManager::littlefs_init() {
#ifdef PICO_PLATFORM
    if (_littlefs_ready) {
        log_debug("LittleFS already initialized");
        return true;
    }
    
    
    bool ok = LittleFS.begin();
    if (!ok) {
        log_error("LittleFS begin failed, try format then begin");
        if (LittleFS.format()) {
            ok = LittleFS.begin();
        }
    }
    
    
    if (ok) {
        log_info("LittleFS begin successful");
        _littlefs_ready = true;
        return true;
    } else {
        log_error("LittleFS initialization failed after format");
        _littlefs_ready = false;
        return false;
    }
#else
    return false;
#endif
}

// 检查文件是否存在
bool ConfigManager::littlefs_file_exists() {
#ifdef PICO_PLATFORM
    if (!_littlefs_ready) {
        log_error("LittleFS not ready for exists check");
        return false;
    }
    
    bool exists = LittleFS.exists(CONFIG_FILE_PATH);
    
    return exists;
#else
    return false;
#endif
}

// 文件操作封装方法（供StreamingJsonSerializer使用）
bool ConfigManager::open_file_for_read(File& file) {
#ifdef PICO_PLATFORM
    if (!_littlefs_ready) {
        log_error("LittleFS not ready for file read");
        return false;
    }
    // 如果配置文件不存在，直接返回 false
    
    bool exists = LittleFS.exists(CONFIG_FILE_PATH);
    
    if (!exists) {
        log_debug("Config file not found for reading: " + std::string(CONFIG_FILE_PATH));
        return false;
    }
    
    file = LittleFS.open(CONFIG_FILE_PATH, "r");
    
    if (!file) {
        log_debug("Failed to open config file for reading");
        return false;
    }
    
    return true;
#else
    return false;
#endif
}

bool ConfigManager::open_file_for_write(File& file) {
#ifdef PICO_PLATFORM
    if (!_littlefs_ready) {
        log_error("LittleFS not ready for file write");
        return false;
    }
    
    file = LittleFS.open(CONFIG_FILE_PATH, "w");
    
    if (!file) {
        log_error("Failed to open config file for writing");
        return false;
    }
    
    return true;
#else
    return false;
#endif
}

void ConfigManager::close_file(File& file) {
#ifdef PICO_PLATFORM
    if (file) {
        
        file.close();
        
    }
#endif
}

// 私有接口：保存config_map到文件
bool ConfigManager::config_save(const config_map_t* config_map) {
    // 使用新的流式保存机制
    bool result = config_save_streaming(config_map);
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

// ========== 流式JSON解析器实现 ==========

StreamingJsonParser::StreamingJsonParser() {
    reset();
}

void StreamingJsonParser::reset() {
    state_ = JsonParseState::EXPECT_OBJECT_START;
    buffer_.clear();
    current_key_.clear();
    current_value_.clear();
    brace_depth_ = 0;
    parsed_data_.clear();
}

bool StreamingJsonParser::parse_chunk(const std::string& chunk) {
    buffer_ += chunk;
    process_buffer();
    return true;
}

void StreamingJsonParser::process_buffer() {
    while (!buffer_.empty() && state_ != JsonParseState::COMPLETE) {
        size_t pos = 0;
        skip_whitespace(pos);
        if (pos >= buffer_.length()) {
            buffer_.clear();
            break;
        }
        
        switch (state_) {
            case JsonParseState::EXPECT_OBJECT_START:
                if (buffer_[pos] == '{') {
                    state_ = JsonParseState::EXPECT_KEY;
                    buffer_ = buffer_.substr(pos + 1);
                } else {
                    buffer_ = buffer_.substr(pos + 1);
                }
                break;
                
            case JsonParseState::EXPECT_KEY:
                if (buffer_[pos] == '}') {
                    state_ = JsonParseState::COMPLETE;
                    buffer_ = buffer_.substr(pos + 1);
                } else if (extract_key()) {
                    state_ = JsonParseState::EXPECT_COLON;
                    // extract_key已经更新了buffer_
                } else {
                    // 数据不足，等待更多数据
                    return;
                }
                break;
                
            case JsonParseState::EXPECT_COLON:
                if (pos < buffer_.length() && buffer_[pos] == ':') {
                    state_ = JsonParseState::EXPECT_VALUE;
                    buffer_ = buffer_.substr(pos + 1);
                } else {
                    // 数据不足或格式错误
                    return;
                }
                break;
                
            case JsonParseState::EXPECT_VALUE:
                if (extract_value()) {
                    parsed_data_[current_key_] = current_value_;
                    current_key_.clear();
                    current_value_.clear();
                    state_ = JsonParseState::EXPECT_COMMA_OR_END;
                    // extract_value已经更新了buffer_
                } else {
                    // 数据不足，等待更多数据
                    return;
                }
                break;
                
            case JsonParseState::EXPECT_COMMA_OR_END:
                if (pos < buffer_.length() && buffer_[pos] == ',') {
                    state_ = JsonParseState::EXPECT_KEY;
                    buffer_ = buffer_.substr(pos + 1);
                } else if (pos < buffer_.length() && buffer_[pos] == '}') {
                    state_ = JsonParseState::COMPLETE;
                    buffer_ = buffer_.substr(pos + 1);
                } else {
                    // 数据不足或格式错误
                    return;
                }
                break;
                
            default:
                return;
        }
    }
}

bool StreamingJsonParser::extract_key() {
    size_t start = buffer_.find('"');
    if (start == std::string::npos) return false;
    
    size_t end = buffer_.find('"', start + 1);
    if (end == std::string::npos) return false;
    
    current_key_ = buffer_.substr(start + 1, end - start - 1);
    buffer_ = buffer_.substr(end + 1);
    return true;
}

bool StreamingJsonParser::extract_value() {
    size_t pos = 0;
    skip_whitespace(pos);
    
    if (pos >= buffer_.length()) return false;
    
    if (buffer_[pos] == '{') {
        // 对象值
        brace_depth_ = 1;
        size_t start = pos;
        pos++;
        
        while (pos < buffer_.length() && brace_depth_ > 0) {
            if (buffer_[pos] == '{') brace_depth_++;
            else if (buffer_[pos] == '}') brace_depth_--;
            pos++;
        }
        
        if (brace_depth_ == 0) {
            current_value_ = buffer_.substr(start, pos - start);
            buffer_ = buffer_.substr(pos);
            return true;
        }
        return false;
    } else {
        // 简单值
        size_t end = buffer_.find(',', pos);
        if (end == std::string::npos) {
            end = buffer_.find('}', pos);
        }
        
        if (end != std::string::npos) {
            std::string raw_value = buffer_.substr(pos, end - pos);
            // 去除值末尾的空白字符
            size_t value_end = raw_value.find_last_not_of(" \t\n\r");
            if (value_end != std::string::npos) {
                current_value_ = raw_value.substr(0, value_end + 1);
            } else {
                current_value_ = raw_value;
            }
            buffer_ = buffer_.substr(end);
            return true;
        }
        return false;
    }
}

void StreamingJsonParser::skip_whitespace(size_t& pos) {
    while (pos < buffer_.length() && (buffer_[pos] == ' ' || buffer_[pos] == '\t' || buffer_[pos] == '\n' || buffer_[pos] == '\r')) {
        pos++;
    }
}

// ========== 流式JSON序列化器实现 ==========

StreamingJsonSerializer::StreamingJsonSerializer() 
    : first_item_(true), complete_(false), file_opened_(false) {
    buffer_.reserve(BUFFER_SIZE);
}

StreamingJsonSerializer::~StreamingJsonSerializer() {
    if (!complete_) {
        end_object();
    }
    if (file_opened_) {
        close_file();
    }
}

bool StreamingJsonSerializer::start_object() {
    if (!open_file()) {
        return false;
    }
    
    buffer_ = "{";
    first_item_ = true;
    complete_ = false;
    return true;
}

bool StreamingJsonSerializer::add_key_value(const char* key, const char* value) {
    if (complete_) return false;
    
    std::string entry;
    if (!first_item_) {
        entry += ",";
    }
    entry += "\"" + std::string(key) + "\":" + std::string(value);

    // 检查是否需要刷新缓冲区
    if (buffer_.length() + entry.length() >= BUFFER_SIZE) {
        if (!flush_buffer()) {
            return false;
        }
    }
    
    buffer_ += entry;
    first_item_ = false;
    
    return true;
}

bool StreamingJsonSerializer::end_object() {
    if (complete_) return true;
    
    buffer_ += "}";
    bool result = flush_buffer();
    
    if (file_opened_) {
        close_file();
    }
    
    complete_ = true;
    return result;
}

bool StreamingJsonSerializer::flush_buffer() {
    if (buffer_.empty()) return true;
    
    bool result = write_to_file(buffer_.c_str(), buffer_.length());
    buffer_.clear();
    return result;
}

bool StreamingJsonSerializer::open_file() {
#ifdef PICO_PLATFORM
    if (file_opened_) {
        return true; // 文件已经打开
    }
    
    if (!ConfigManager::is_littlefs_ready()) {
        ConfigManager::log_error("LittleFS not ready for streaming write");
        return false;
    }
    
    if (!ConfigManager::open_file_for_write(file_handle_)) {
        ConfigManager::log_error("Failed to open config file for streaming write");
        return false;
    }
    
    file_opened_ = true;
    return true;
#else
    // 非PICO平台的实现（用于测试）
    file_opened_ = true;
    return true;
#endif
}

bool StreamingJsonSerializer::close_file() {
#ifdef PICO_PLATFORM
    if (!file_opened_) {
        return true; // 文件已经关闭
    }
    
    ConfigManager::close_file(file_handle_);
    file_opened_ = false;
    return true;
#else
    // 非PICO平台的实现（用于测试）
    file_opened_ = false;
    return true;
#endif
}

bool StreamingJsonSerializer::write_to_file(const char* data, size_t length) {
#ifdef PICO_PLATFORM
    if (!file_opened_) {
        ConfigManager::log_error("File not opened for streaming write");
        return false;
    }
    
    if (length > BUFFER_SIZE) {
        ConfigManager::log_error("Data size exceeds buffer size: " + std::to_string(length) + " > " + std::to_string(BUFFER_SIZE));
        return false;
    }
    
    // 直接复制到静态uint8_t缓冲区
    std::memcpy(write_buffer_, data, length);
    
    size_t written = file_handle_.write(write_buffer_, length);
    file_handle_.flush();
    
    
    if (written != length) {
        ConfigManager::log_error("Streaming write size mismatch: expected " + std::to_string(length) + ", written " + std::to_string(written));
        return false;
    }
    
    return true;
#else
    // 非PICO平台的实现（用于测试）
    return true;
#endif
}

// ========== 流式读写函数实现 ==========

bool ConfigManager::config_read(config_map_t* config_map) {
    // 使用新的流式读取机制
    bool ok = config_read_streaming(config_map);
    if (!ok) {
        log_error("Config read failed, falling back to defaults and re-saving");
    }
    return ok;
}

bool ConfigManager::config_read_streaming(config_map_t* config_map) {
    if (!config_map) {
        log_error("config_map is null");
        return false;
    }
    
    if (!_littlefs_ready) {
        log_error("LittleFS not ready for streaming read");
        return false;
    }
    
#ifdef PICO_PLATFORM
    File file;
    if (!open_file_for_read(file)) {
        log_error("Failed to open config file for streaming read");
        return false;
    }
    
    StreamingJsonParser parser;
    char chunk_buffer[256];
    
    log_debug("Starting streaming read of config file");
    
    while (file.available() && !parser.is_complete()) {
        
        size_t bytes_read = file.read(reinterpret_cast<uint8_t*>(chunk_buffer), sizeof(chunk_buffer));
        
        if (bytes_read == 0) break;
        std::string chunk(chunk_buffer, bytes_read);
        
        // 格式化输出读取的JSON块，每行不超过200字符
        log_debug("=== 读取JSON块开始 (" + std::to_string(bytes_read) + " 字节) ===");
        size_t pos = 0;
        const size_t max_line_length = 160; // 留一些余量给前缀
        while (pos < chunk.length()) {
            size_t end_pos = std::min(pos + max_line_length, chunk.length());
            std::string line = chunk.substr(pos, end_pos - pos);
            log_debug("CHUNK[" + std::to_string(pos) + "-" + std::to_string(end_pos-1) + "]: " + line);
            pos = end_pos;
        }
        log_debug("=== 读取JSON块结束 ===");
        if (!parser.parse_chunk(chunk)) {
            log_error("Failed to parse JSON chunk");
            close_file(file);
            return false;
        }
        
        // 不在这里处理配置项，等到解析完成后统一处理
        
        // 喂狗操作 - 防止长时间读取时看门狗复位
#ifdef PICO_PLATFORM
        watchdog_update();
#endif
    }
    
    close_file(file);
    
    if (!parser.is_complete()) {
        log_error("JSON parsing incomplete");
        return false;
    }
    
    // 解析完成后统一处理所有配置项
    auto final_data = parser.get_parsed_data();
    uint32_t stored_crc = 0;
    bool has_crc = false;
    
    // 先处理所有非CRC配置项
    for (const auto& kv : final_data) {
        const std::string& key = kv.first;
        const std::string& obj_str = kv.second;
        
        if (key == CONFIG_KEY_CRC) {
            // CRC值直接存储为字符串，不是JSON对象
            stored_crc = std::stoul(obj_str);
            has_crc = true;
            continue;
        }
        
        // 处理配置项
        process_config_item(config_map, key, obj_str);
        log_debug("Processed config item: " + key);
    }
    
    // 处理CRC验证
    if (has_crc) {
        log_debug("=== 读取时CRC验证开始 ===");
        log_debug("从文件读取的CRC值: " + std::to_string(stored_crc));
        log_debug("开始重新计算CRC...");
        
        uint32_t calculated_crc = calculate_crc32(*config_map);
        
        log_debug("读取时重新计算的CRC值: " + std::to_string(calculated_crc));
        log_debug("CRC对比: 存储=" + std::to_string(stored_crc) + ", 计算=" + std::to_string(calculated_crc));
        
        if (stored_crc != calculated_crc) {
            log_error("=== CRC校验失败 ===");
            log_error("CRC mismatch in streaming read: stored=" + std::to_string(stored_crc) + ", calculated=" + std::to_string(calculated_crc));
            log_error("差值: " + std::to_string(static_cast<int64_t>(calculated_crc) - static_cast<int64_t>(stored_crc)));
            log_error("=== CRC校验失败结束 ===");
            return false;
        }
        
        log_debug("CRC verification passed in streaming read");
        log_debug("=== 读取时CRC验证结束 ===");
    } else {
        log_debug("配置文件中未找到CRC字段");
    }
    
    log_debug("Streaming read completed successfully, loaded " + std::to_string(config_map->size()) + " config items");
    return true;
    
#else
    // 非PICO平台回退到常规读取
    return false;
#endif
}

// 辅助函数：处理单个配置项
void ConfigManager::process_config_item(config_map_t* config_map, const std::string& key, const std::string& obj_str) {
    log_debug("Processing config item: " + key);
    log_debug("Object string: " + obj_str);
    
    std::string type_str = extract_value_from_object(obj_str, "type");
    std::string value_str = extract_value_from_object(obj_str, "value");
    
    log_debug("Extracted type: '" + type_str + "', value: '" + value_str + "'");
    
    if (type_str.empty()) {
        log_error("Skipping config item '" + key + "' due to empty type. Type: '" + type_str + "', Value: '" + value_str + "'");
        return;
    }
    
    // 注意：value_str可以为空字符串，这对于STRING类型是有效的
    
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
            uint32_t val = std::stoul(value_str);
            std::string min_str = extract_value_from_object(obj_str, "min");
            std::string max_str = extract_value_from_object(obj_str, "max");
            if (!min_str.empty() && !max_str.empty()) {
                new_value = ConfigValue(val, std::stoul(min_str), std::stoul(max_str));
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
            log_error("Unknown config value type: " + type_str);
            return;
    }
    
    (*config_map)[key] = new_value;
    log_debug("Processed config item: " + key);
}

bool ConfigManager::config_save_streaming(const config_map_t* config_map) {
    if (!config_map) {
        log_error("config_map is null");
        return false;
    }
    
    if (!_littlefs_ready) {
        log_error("LittleFS not ready for streaming write");
        return false;
    }
    
    log_debug("Starting streaming save of config file: " + std::string(CONFIG_FILE_PATH));
    
    // 先生成完整的JSON字符串（不含CRC）
    std::ostringstream json_stream;
    json_stream << "{";
    
    // 按键名排序以确保与CRC计算一致
    std::vector<std::pair<std::string, ConfigValue>> sorted_items(config_map->begin(), config_map->end());
    std::sort(sorted_items.begin(), sorted_items.end(), 
              [](const std::pair<std::string, ConfigValue>& a, const std::pair<std::string, ConfigValue>& b) {
                  return a.first < b.first;
              });
    
    bool first_item = true;
    for (const auto& kv : sorted_items) {
        const std::string& key = kv.first;
        const ConfigValue& value = kv.second;
        
        if (!first_item) {
            json_stream << ",";
        }
        first_item = false;
        
        std::string json_value = serialize_config_value(value);
        json_stream << "\"" << key << "\":" << json_value;
        
        log_debug("Added config item to JSON: " + key);
        
        // 喂狗操作 - 防止长时间处理时看门狗复位
#ifdef PICO_PLATFORM
        watchdog_update();
#endif
    }
    
    // 先计算CRC（基于不含CRC的JSON数据）
    log_debug("=== 保存时CRC计算开始 ===");
    uint32_t crc = calculate_crc32(*config_map);
    log_debug("保存时计算的CRC值: " + std::to_string(crc));
    
    // 添加CRC到JSON
    if (!first_item) {
        json_stream << ",";
    }
    json_stream << "\"" << CONFIG_KEY_CRC << "\":" << crc;
    json_stream << "}";
    
    std::string complete_json = json_stream.str();
    log_debug("完整JSON长度: " + std::to_string(complete_json.length()) + " 字节");
    
    // 格式化输出JSON内容，每行不超过200字符
    log_debug("=== 生成的完整JSON开始 ===");
    size_t pos = 0;
    const size_t max_line_length = 180; // 留一些余量给前缀
    while (pos < complete_json.length()) {
        size_t end_pos = std::min(pos + max_line_length, complete_json.length());
        std::string line = complete_json.substr(pos, end_pos - pos);
        log_debug("JSON[" + std::to_string(pos) + "-" + std::to_string(end_pos-1) + "]: " + line);
        pos = end_pos;
    }
    log_debug("=== 生成的完整JSON结束 ===");
    log_debug("=== 保存时CRC计算结束 ===");
    
    // 一次性写入完整的JSON
    File file;
    if (!open_file_for_write(file)) {
        log_error("Failed to open config file for write");
        return false;
    }
    
    size_t bytes_written = file.write(reinterpret_cast<const uint8_t*>(complete_json.c_str()), complete_json.length());
    close_file(file);
    
    if (bytes_written != complete_json.length()) {
        log_error("Failed to write complete JSON to file");
        return false;
    }
    
    log_debug("Streaming save completed successfully, saved " + std::to_string(config_map->size()) + " config items");
    return true;
}

// 辅助函数：序列化单个配置值
std::string ConfigManager::serialize_config_value(const ConfigValue& value) {
    std::ostringstream oss;
    oss << "{";
    oss << "\"type\":" << static_cast<int>(value.type) << ",";
    
    switch (value.type) {
        case ConfigValueType::BOOL:
            oss << "\"value\":\"" << (value.bool_val ? "true" : "false") << "\"";
            break;
        case ConfigValueType::INT8:
            oss << "\"value\":\"" << static_cast<int>(value.int8_val) << "\"";
            if (value.has_range) {
                oss << ",\"min\":\"" << static_cast<int>(value.min_val.int8_min) << "\"";
                oss << ",\"max\":\"" << static_cast<int>(value.max_val.int8_max) << "\"";
            }
            break;
        case ConfigValueType::UINT8:
            oss << "\"value\":\"" << static_cast<unsigned int>(value.uint8_val) << "\"";
            if (value.has_range) {
                oss << ",\"min\":\"" << static_cast<unsigned int>(value.min_val.uint8_min) << "\"";
                oss << ",\"max\":\"" << static_cast<unsigned int>(value.max_val.uint8_max) << "\"";
            }
            break;
        case ConfigValueType::UINT16:
            oss << "\"value\":\"" << value.uint16_val << "\"";
            if (value.has_range) {
                oss << ",\"min\":\"" << value.min_val.uint16_min << "\"";
                oss << ",\"max\":\"" << value.max_val.uint16_max << "\"";
            }
            break;
        case ConfigValueType::UINT32:
            oss << "\"value\":\"" << value.uint32_val << "\"";
            if (value.has_range) {
                oss << ",\"min\":\"" << value.min_val.uint32_min << "\"";
                oss << ",\"max\":\"" << value.max_val.uint32_max << "\"";
            }
            break;
        case ConfigValueType::FLOAT: {
            // 使用与读取时相同的格式，避免精度问题
            std::ostringstream float_oss;
            float_oss << value.float_val;
            oss << "\"value\":\"" << float_oss.str() << "\"";
            if (value.has_range) {
                std::ostringstream min_oss, max_oss;
                min_oss << value.min_val.float_min;
                max_oss << value.max_val.float_max;
                oss << ",\"min\":\"" << min_oss.str() << "\"";
                oss << ",\"max\":\"" << max_oss.str() << "\"";
            }
            break;
        }
        case ConfigValueType::STRING:
            oss << "\"value\":\"" << value.string_val << "\"";
            break;
    }
    
    oss << "}";
    return oss.str();
}

// 初始化配置模块
bool ConfigManager::initialize() {
    if (_initialized) {
        log_debug("Already initialized");
        return true;
    }
    
    log_info("Starting initialization...");

    // 初始化LittleFS
    if (!littlefs_init()) {
        log_error("LittleFS initialization failed");
        return false;
    }

    _initialized = true;
    _config_valid = false;
    
    // 初始化默认配置
    initialize_defaults();
    log_debug("Default configs initialized, count: " + std::to_string(_default_map.size()));
    
    // 检查配置文件是否存在
    bool config_exists = littlefs_file_exists();
    log_debug("Config file exists: " + std::string(config_exists ? "true" : "false"));
    if (config_exists && config_read(&_runtime_map)) {
        log_debug("Config file read, runtime map size: " + std::to_string(_runtime_map.size()));
    }else {
        
        bool result = LittleFS.format();
        
        if (result && config_save(&_default_map)) {
            log_debug("Save default config successful");
        }else {
            log_error("Format and save default config failed");
        }
        _runtime_map = _default_map;
    }

    _config_valid = true;
    log_info("Initialization completed successfully");
    
    return true;
}

// 反初始化配置模块
void ConfigManager::deinit() {
    _initialized = false;
    _config_valid = false;
    _littlefs_ready = false;
    
    _runtime_map.clear();
    _default_map.clear();
    _string_cache.clear();
    _init_functions.clear();
    _error_count = 0;
    
    log_info("ConfigManager deinitialized");
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

// 置位保存信号
void ConfigManager::save_config() {
    _save_requested = true;
}

// 保存配置到文件（task版本，检查信号）
bool ConfigManager::save_config_task() {
    if (!_save_requested) {
        return true;  // 没有保存请求，直接返回成功
    }
    
    _save_requested = false;  // 清除保存请求信号
    
    log_debug("Starting config save process...");
    log_debug("Runtime map size: " + std::to_string(_runtime_map.size()));
    multicore_lockout_start_blocking();
    bool result = config_save(&_runtime_map);
    if (!result) {
        _error_count++;
        log_error("Config save failed! Error count: " + std::to_string(_error_count));
    } else {
        log_info("Config save successful");
    }
    multicore_lockout_end_blocking();
    return result;
}

// 重置到默认配置
bool ConfigManager::reset_to_defaults() {
    _runtime_map = _default_map;
    return save_config_task();
}

// 调试接口
void ConfigManager::debug_print_all_configs() {
    log_debug("===== DEBUG: All Configurations =====");
    log_debug("Initialized: " + std::string(_initialized ? "true" : "false"));
    log_debug("Config Valid: " + std::string(_config_valid ? "true" : "false"));
    log_debug("Error Count: " + std::to_string(_error_count));
    log_debug("Runtime Map Size: " + std::to_string(_runtime_map.size()));
    
    // 输出所有运行时配置
    for (const auto& pair : _runtime_map) {
        const std::string& key = pair.first;
        const ConfigValue& value = pair.second;
        
        log_debug("Key: " + key + ", Type: " + std::to_string(static_cast<int>(value.type)));
        
        switch (value.type) {
            case ConfigValueType::BOOL:
                log_debug("Value: " + std::string(value.bool_val ? "true" : "false"));
                break;
            case ConfigValueType::UINT8:
                log_debug("Value: " + std::to_string(value.uint8_val));
                break;
            case ConfigValueType::UINT16:
                log_debug("Value: " + std::to_string(value.uint16_val));
                break;
            case ConfigValueType::UINT32:
                log_debug("Value: " + std::to_string(value.uint32_val));
                break;
            case ConfigValueType::STRING:
                log_debug("Value: " + value.string_val);
                break;
            default:
                log_debug("Value: (unknown type)");
                break;
        }
    }
    log_debug("===== END DEBUG =====");
}

// 字符串验证
bool ConfigManager::is_valid_string(const std::string& str) {
    // 简单的字符串验证
    return !str.empty() && str.length() < 256;
}

// 内部日志接口实现
void ConfigManager::log_debug(const std::string& message) {
    auto* logger = USB_SerialLogs::get_global_instance();
    if (logger) {
        logger->debug(message, "ConfigManager");
    }
}

void ConfigManager::log_info(const std::string& message) {
    auto* logger = USB_SerialLogs::get_global_instance();
    if (logger) {
        logger->info(message, "ConfigManager");
    }
}

void ConfigManager::log_error(const std::string& message) {
    auto* logger = USB_SerialLogs::get_global_instance();
    if (logger) {
        logger->error(message, "ConfigManager");
    }
}

bool ConfigManager::is_littlefs_ready() {
    return _littlefs_ready;
}
