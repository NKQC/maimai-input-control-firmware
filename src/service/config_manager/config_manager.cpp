#include "config_manager.h"
#include "../../service/input_manager/input_manager.h"
#include "../../service/light_manager/light_manager.h"
#include "../../service/ui_manager/ui_manager.h"
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <avr/eeprom.h>
#include <hardware/flash.h>
#include <pico/stdlib.h>

// 静态实例变量定义
ConfigManager* ConfigManager::instance_ = nullptr;

// 静态配置变量
static ConfigManager_PrivateConfig static_config_;

// 纯公开函数实现
ConfigManager_PrivateConfig* config_manager_get_config_holder() {
    return &static_config_;
}

bool config_manager_load_config_from_manager(ConfigManager* config_manager) {
    if (!config_manager) {
        return false;
    }
    
    // 从ConfigManager加载配置项到静态配置
    bool success = true;
    
    // 加载各种配置项
    success &= config_manager->get_bool("enable_auto_save", static_config_.enable_auto_save);
    success &= config_manager->get_uint16("auto_save_interval", static_config_.auto_save_interval);
    success &= config_manager->get_bool("enable_backup", static_config_.enable_backup);
    success &= config_manager->get_uint8("max_backups", static_config_.max_backups);
    success &= config_manager->get_bool("enable_validation", static_config_.enable_validation);
    success &= config_manager->get_bool("enable_encryption", static_config_.enable_encryption);
    success &= config_manager->get_string("encryption_key", static_config_.encryption_key);
    success &= config_manager->get_uint16("eeprom_start_addr", static_config_.eeprom_start_addr);
    success &= config_manager->get_uint16("eeprom_size", static_config_.eeprom_size);
    success &= config_manager->get_uint32("flash_start_addr", static_config_.flash_start_addr);
    success &= config_manager->get_uint32("flash_size", static_config_.flash_size);
    
    return success;
}

ConfigManager_PrivateConfig config_manager_get_config_copy() {
    return static_config_;
}

bool config_manager_write_config_to_manager(ConfigManager* config_manager, const ConfigManager_PrivateConfig& config) {
    if (!config_manager) {
        return false;
    }
    
    // 将配置写入ConfigManager
    bool success = true;
    
    success &= config_manager->set_bool("enable_auto_save", config.enable_auto_save);
    success &= config_manager->set_uint16("auto_save_interval", config.auto_save_interval);
    success &= config_manager->set_bool("enable_backup", config.enable_backup);
    success &= config_manager->set_uint8("max_backups", config.max_backups);
    success &= config_manager->set_bool("enable_validation", config.enable_validation);
    success &= config_manager->set_bool("enable_encryption", config.enable_encryption);
    success &= config_manager->set_string("encryption_key", config.encryption_key);
    success &= config_manager->set_uint16("eeprom_start_addr", config.eeprom_start_addr);
    success &= config_manager->set_uint16("eeprom_size", config.eeprom_size);
    success &= config_manager->set_uint32("flash_start_addr", config.flash_start_addr);
    success &= config_manager->set_uint32("flash_size", config.flash_size);
    
    // 更新静态配置
    static_config_ = config;
    
    return success;
}

// 单例模式实现
ConfigManager* ConfigManager::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new ConfigManager();
    }
    return instance_;
}

// 构造函数和析构函数
ConfigManager::ConfigManager()
    : initialized_(false)
    , input_manager_(nullptr)
    , light_manager_(nullptr)
    , ui_manager_(nullptr)
    , last_auto_save_time_(0)
    , debug_enabled_(false)
    , statistics_() {
}

ConfigManager::~ConfigManager() {
    deinit();
}

// 初始化和释放
bool ConfigManager::init() {
    if (initialized_) {
        return true;
    }
    
    log_debug("Initializing ConfigManager...");
    
    // 注册系统配置
    if (!register_system_configs()) {
        log_error("Failed to register system configs");
        return false;
    }
    
    // 加载所有配置
    if (!load_all_configs()) {
        log_debug("Failed to load configs, using defaults");
        // 使用默认值，不算错误
    }
    
    // 重置统计信息
    reset_statistics();
    
    initialized_ = true;
    log_debug("ConfigManager initialized successfully");
    return true;
}

void ConfigManager::deinit() {
    if (!initialized_) {
        return;
    }
    
    log_debug("Deinitializing ConfigManager...");
    
    // 保存所有脏配置
    save_all_configs();
    
    // 清理数据
    configs_.clear();
    groups_.clear();
    templates_.clear();
    backups_.clear();
    
    // 清理回调
    change_callback_ = nullptr;
    save_callback_ = nullptr;
    load_callback_ = nullptr;
    validation_callback_ = nullptr;
    
    initialized_ = false;
    log_debug("ConfigManager deinitialized");
}

bool ConfigManager::is_ready() const {
    return initialized_;
}

// 服务依赖
bool ConfigManager::set_input_manager(InputManager* input_manager) {
    input_manager_ = input_manager;
    
    // 注册输入相关配置
    if (initialized_ && input_manager_) {
        register_input_configs();
    }
    
    return true;
}

bool ConfigManager::set_light_manager(LightManager* light_manager) {
    light_manager_ = light_manager;
    
    // 注册灯光相关配置
    if (initialized_ && light_manager_) {
        register_light_configs();
    }
    
    return true;
}

bool ConfigManager::set_ui_manager(UIManager* ui_manager) {
    ui_manager_ = ui_manager;
    
    // 注册UI相关配置
    if (initialized_ && ui_manager_) {
        register_ui_configs();
    }
    
    return true;
}

// 配置管理已移至纯公开函数

// 配置项管理
bool ConfigManager::register_config(const std::string& key, const ConfigItem& item) {
    if (!validate_key(key)) {
        log_error("Invalid config key: " + key);
        return false;
    }
    
    if (configs_.find(key) != configs_.end()) {
        log_debug("Config key already exists, updating: " + key);
    }
    
    configs_[key] = item;
    configs_[key].key = key;
    
    log_debug("Registered config: " + key);
    return true;
}

bool ConfigManager::unregister_config(const std::string& key) {
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        return false;
    }
    
    configs_.erase(it);
    log_debug("Unregistered config: " + key);
    return true;
}

bool ConfigManager::has_config(const std::string& key) const {
    return configs_.find(key) != configs_.end();
}

bool ConfigManager::get_config_info(const std::string& key, ConfigItem& item) {
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        return false;
    }
    
    item = it->second;
    return true;
}

std::vector<std::string> ConfigManager::get_config_keys() const {
    std::vector<std::string> keys;
    for (const auto& pair : configs_) {
        keys.push_back(pair.first);
    }
    return keys;
}

// 基本读写操作
bool ConfigManager::set_bool(const std::string& key, bool value) {
    return set_value(key, to_bytes(value));
}

bool ConfigManager::get_bool(const std::string& key, bool& value) {
    std::vector<uint8_t> data;
    if (!get_value(key, data)) {
        return false;
    }
    return from_bytes(data, value);
}

bool ConfigManager::set_uint8(const std::string& key, uint8_t value) {
    return set_value(key, to_bytes(value));
}

bool ConfigManager::get_uint8(const std::string& key, uint8_t& value) {
    std::vector<uint8_t> data;
    if (!get_value(key, data)) {
        return false;
    }
    return from_bytes(data, value);
}

bool ConfigManager::set_uint16(const std::string& key, uint16_t value) {
    return set_value(key, to_bytes(value));
}

bool ConfigManager::get_uint16(const std::string& key, uint16_t& value) {
    std::vector<uint8_t> data;
    if (!get_value(key, data)) {
        return false;
    }
    return from_bytes(data, value);
}

bool ConfigManager::set_uint32(const std::string& key, uint32_t value) {
    return set_value(key, to_bytes(value));
}

bool ConfigManager::get_uint32(const std::string& key, uint32_t& value) {
    std::vector<uint8_t> data;
    if (!get_value(key, data)) {
        return false;
    }
    return from_bytes(data, value);
}

bool ConfigManager::set_float(const std::string& key, float value) {
    return set_value(key, to_bytes(value));
}

bool ConfigManager::get_float(const std::string& key, float& value) {
    std::vector<uint8_t> data;
    if (!get_value(key, data)) {
        return false;
    }
    return from_bytes(data, value);
}

bool ConfigManager::set_string(const std::string& key, const std::string& value) {
    return set_value(key, to_bytes(value));
}

bool ConfigManager::get_string(const std::string& key, std::string& value) {
    std::vector<uint8_t> data;
    if (!get_value(key, data)) {
        return false;
    }
    return from_bytes(data, value);
}

bool ConfigManager::set_binary(const std::string& key, const std::vector<uint8_t>& value) {
    return set_value(key, value);
}

bool ConfigManager::get_binary(const std::string& key, std::vector<uint8_t>& value) {
    return get_value(key, value);
}

// 通用读写操作
bool ConfigManager::set_value(const std::string& key, const std::vector<uint8_t>& data) {
    if (!initialized_) {
        return false;
    }
    
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        log_error("Config not found: " + key);
        return false;
    }
    
    if (!validate_access(key, true)) {
        log_error("Write access denied: " + key);
        return false;
    }
    
    if (!validate_type(it->second, data) || !validate_range(it->second, data)) {
        log_error("Validation failed: " + key);
        statistics_.validation_errors++;
        return false;
    }
    
    // 更新数据
    it->second.data = data;
    it->second.dirty = true;
    it->second.last_modified = statistics_.uptime_seconds;
    
    // 通知变更
    notify_change(key, it->second);
    
    statistics_.total_writes++;
    log_debug("Set config: " + key);
    return true;
}

bool ConfigManager::get_value(const std::string& key, std::vector<uint8_t>& data) {
    if (!initialized_) {
        return false;
    }
    
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        return false;
    }
    
    if (!validate_access(key, false)) {
        return false;
    }
    
    data = it->second.data;
    statistics_.total_reads++;
    return true;
}

// 默认值操作
bool ConfigManager::reset_to_default(const std::string& key) {
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        return false;
    }
    
    it->second.data = it->second.default_data;
    it->second.dirty = true;
    it->second.last_modified = statistics_.uptime_seconds;
    
    notify_change(key, it->second);
    log_debug("Reset to default: " + key);
    return true;
}

bool ConfigManager::reset_all_to_default() {
    for (auto& pair : configs_) {
        pair.second.data = pair.second.default_data;
        pair.second.dirty = true;
        pair.second.last_modified = statistics_.uptime_seconds;
        notify_change(pair.first, pair.second);
    }
    
    log_debug("Reset all configs to default");
    return true;
}

// 持久化操作
bool ConfigManager::save_config(const std::string& key) {
    if (key.empty()) {
        return save_all_configs();
    }
    
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        return false;
    }
    
    bool success = false;
    switch (it->second.storage) {
        case ConfigStorage::EEPROM:
            success = save_to_eeprom(key, it->second.data);
            break;
        case ConfigStorage::FLASH:
            success = save_to_flash(key, it->second.data);
            break;
        case ConfigStorage::RAM:
        case ConfigStorage::EXTERNAL:
        default:
            success = true; // RAM不需要保存
            break;
    }
    
    if (success) {
        it->second.dirty = false;
        statistics_.total_saves++;
        log_debug("Saved config: " + key);
    } else {
        log_error("Failed to save config: " + key);
    }
    
    return success;
}

bool ConfigManager::load_config(const std::string& key) {
    if (key.empty()) {
        return load_all_configs();
    }
    
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        return false;
    }
    
    std::vector<uint8_t> data;
    bool success = false;
    
    switch (it->second.storage) {
        case ConfigStorage::EEPROM:
            success = load_from_eeprom(key, data);
            break;
        case ConfigStorage::FLASH:
            success = load_from_flash(key, data);
            break;
        case ConfigStorage::RAM:
        case ConfigStorage::EXTERNAL:
        default:
            success = true; // RAM使用当前值
            data = it->second.data;
            break;
    }
    
    if (success && !data.empty()) {
        if (validate_type(it->second, data) && validate_range(it->second, data)) {
            it->second.data = data;
            it->second.dirty = false;
            statistics_.total_loads++;
            log_debug("Loaded config: " + key);
        } else {
            log_error("Validation failed for loaded config: " + key);
            statistics_.validation_errors++;
            success = false;
        }
    } else {
        log_debug("Failed to load config, using default: " + key);
        it->second.data = it->second.default_data;
        it->second.dirty = true;
    }
    
    return success;
}

bool ConfigManager::save_all_configs() {
    bool all_success = true;
    
    for (auto& pair : configs_) {
        if (pair.second.dirty && pair.second.storage != ConfigStorage::RAM) {
            if (!save_config(pair.first)) {
                all_success = false;
            }
        }
    }
    
    log_debug("Save all configs completed");
    return all_success;
}

bool ConfigManager::load_all_configs() {
    bool all_success = true;
    
    for (auto& pair : configs_) {
        if (pair.second.storage != ConfigStorage::RAM) {
            if (!load_config(pair.first)) {
                all_success = false;
            }
        }
    }
    
    log_debug("Load all configs completed");
    return all_success;
}

// 统计信息
bool ConfigManager::get_statistics(ConfigStatistics& stats) {
    stats = statistics_;
    return true;
}

void ConfigManager::reset_statistics() {
    statistics_ = ConfigStatistics();
    statistics_.last_reset_time = 0; // 应该使用实际时间
}

// 回调设置
void ConfigManager::set_change_callback(ConfigChangeCallback callback) {
    change_callback_ = callback;
}

void ConfigManager::set_save_callback(ConfigSaveCallback callback) {
    save_callback_ = callback;
}

void ConfigManager::set_load_callback(ConfigLoadCallback callback) {
    load_callback_ = callback;
}

void ConfigManager::set_validation_callback(ConfigValidationCallback callback) {
    validation_callback_ = callback;
}

// 任务处理
void ConfigManager::task() {
    if (!initialized_) {
        return;
    }
    
    statistics_.uptime_seconds++;
    
    // 处理自动保存
    handle_auto_save();
}

// 调试功能
void ConfigManager::enable_debug_output(bool enabled) {
    debug_enabled_ = enabled;
}

std::string ConfigManager::get_debug_info() {
    std::stringstream ss;
    ss << "ConfigManager Debug Info:\n";
    ss << "Initialized: " << (initialized_ ? "Yes" : "No") << "\n";
    ss << "Total configs: " << configs_.size() << "\n";
    ss << "Total groups: " << groups_.size() << "\n";
    ss << "Total templates: " << templates_.size() << "\n";
    ss << "Total backups: " << backups_.size() << "\n";
    ConfigManager_PrivateConfig config = config_manager_get_config_copy();
    ss << "Auto save enabled: " << (config.enable_auto_save ? "Yes" : "No") << "\n";
    ss << "Debug enabled: " << (debug_enabled_ ? "Yes" : "No") << "\n";
    return ss.str();
}

// 预定义配置注册
bool ConfigManager::register_system_configs() {
    // 系统基本配置
    ConfigItem item;
    
    // 设备名称
    item = ConfigItem();
    item.description = "Device name";
    item.type = ConfigType::STRING;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.max_length = 32;
    item.default_data = to_bytes(std::string("MaiMai Controller"));
    item.data = item.default_data;
    register_config("system.device_name", item);
    
    // 固件版本
    item = ConfigItem();
    item.description = "Firmware version";
    item.type = ConfigType::STRING;
    item.access = ConfigAccess::READ_ONLY;
    item.storage = ConfigStorage::RAM;
    item.max_length = 16;
    item.default_data = to_bytes(std::string("3.0.0"));
    item.data = item.default_data;
    register_config("system.firmware_version", item);
    
    // 硬件版本
    item = ConfigItem();
    item.description = "Hardware version";
    item.type = ConfigType::STRING;
    item.access = ConfigAccess::READ_ONLY;
    item.storage = ConfigStorage::RAM;
    item.max_length = 16;
    item.default_data = to_bytes(std::string("1.0.0"));
    item.data = item.default_data;
    register_config("system.hardware_version", item);
    
    // 调试模式
    item = ConfigItem();
    item.description = "Debug mode enabled";
    item.type = ConfigType::BOOL;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.default_data = to_bytes(false);
    item.data = item.default_data;
    register_config("system.debug_enabled", item);
    
    log_debug("System configs registered");
    return true;
}

bool ConfigManager::register_input_configs() {
    if (!input_manager_) {
        return false;
    }
    
    ConfigItem item;
    
    // 扫描间隔
    item = ConfigItem();
    item.description = "Input scan interval (ms)";
    item.type = ConfigType::UINT16;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.min_value = 1;
    item.max_value = 100;
    item.default_data = to_bytes(static_cast<uint16_t>(10));
    item.data = item.default_data;
    register_config("input.scan_interval", item);
    
    // 防抖时间
    item = ConfigItem();
    item.description = "Input debounce time (ms)";
    item.type = ConfigType::UINT16;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.min_value = 0;
    item.max_value = 100;
    item.default_data = to_bytes(static_cast<uint16_t>(5));
    item.data = item.default_data;
    register_config("input.debounce_time", item);
    
    // 长按阈值
    item = ConfigItem();
    item.description = "Hold threshold (ms)";
    item.type = ConfigType::UINT16;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.min_value = 100;
    item.max_value = 5000;
    item.default_data = to_bytes(static_cast<uint16_t>(500));
    item.data = item.default_data;
    register_config("input.hold_threshold", item);
    
    log_debug("Input configs registered");
    return true;
}

bool ConfigManager::register_light_configs() {
    if (!light_manager_) {
        return false;
    }
    
    ConfigItem item;
    
    // 全局亮度
    item = ConfigItem();
    item.description = "Global brightness (0-255)";
    item.type = ConfigType::UINT8;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.min_value = 0;
    item.max_value = 255;
    item.default_data = to_bytes(static_cast<uint8_t>(128));
    item.data = item.default_data;
    register_config("light.global_brightness", item);
    
    // 更新间隔
    item = ConfigItem();
    item.description = "Light update interval (ms)";
    item.type = ConfigType::UINT16;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.min_value = 10;
    item.max_value = 1000;
    item.default_data = to_bytes(static_cast<uint16_t>(50));
    item.data = item.default_data;
    register_config("light.update_interval", item);
    
    log_debug("Light configs registered");
    return true;
}

bool ConfigManager::register_ui_configs() {
    if (!ui_manager_) {
        return false;
    }
    
    ConfigItem item;
    
    // 屏幕亮度
    item = ConfigItem();
    item.description = "Screen brightness (0-255)";
    item.type = ConfigType::UINT8;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.min_value = 0;
    item.max_value = 255;
    item.default_data = to_bytes(static_cast<uint8_t>(200));
    item.data = item.default_data;
    register_config("ui.screen_brightness", item);
    
    // 屏保时间
    item = ConfigItem();
    item.description = "Screensaver timeout (s)";
    item.type = ConfigType::UINT16;
    item.access = ConfigAccess::READ_WRITE;
    item.storage = ConfigStorage::EEPROM;
    item.min_value = 0;
    item.max_value = 3600;
    item.default_data = to_bytes(static_cast<uint16_t>(300));
    item.data = item.default_data;
    register_config("ui.screensaver_timeout", item);
    
    log_debug("UI configs registered");
    return true;
}

// 私有方法实现
bool ConfigManager::validate_key(const std::string& key) const {
    if (key.empty() || key.length() > 64) {
        return false;
    }
    
    // 检查字符是否有效
    for (char c : key) {
        if (!std::isalnum(c) && c != '.' && c != '_' && c != '-') {
            return false;
        }
    }
    
    return true;
}

bool ConfigManager::validate_access(const std::string& key, bool write_access) const {
    auto it = configs_.find(key);
    if (it == configs_.end()) {
        return false;
    }
    
    const ConfigItem& item = it->second;
    
    if (write_access) {
        return item.access == ConfigAccess::READ_WRITE || item.access == ConfigAccess::WRITE_ONCE;
    } else {
        return item.access != ConfigAccess::ADMIN_ONLY; // 简化的权限检查
    }
}

bool ConfigManager::validate_type(const ConfigItem& item, const std::vector<uint8_t>& data) const {
    switch (item.type) {
        case ConfigType::BOOL:
            return data.size() == 1;
        case ConfigType::INT8:
        case ConfigType::UINT8:
            return data.size() == 1;
        case ConfigType::INT16:
        case ConfigType::UINT16:
            return data.size() == 2;
        case ConfigType::INT32:
        case ConfigType::UINT32:
        case ConfigType::FLOAT:
            return data.size() == 4;
        case ConfigType::STRING:
        case ConfigType::BINARY:
            return data.size() <= item.max_length;
        default:
            return false;
    }
}

bool ConfigManager::validate_range(const ConfigItem& item, const std::vector<uint8_t>& data) const {
    // 简化的范围检查
    if (item.type == ConfigType::UINT8 && data.size() == 1) {
        uint8_t value = data[0];
        return value >= item.min_value && value <= item.max_value;
    }
    
    return true; // 其他类型暂时跳过范围检查
}

bool ConfigManager::save_to_eeprom(const std::string& key, const std::vector<uint8_t>& data) {
    ConfigManager_PrivateConfig config = config_manager_get_config_copy();
    uint16_t addr = config.eeprom_start_addr;
    
    // 计算键的哈希作为地址偏移
    uint32_t hash = 0;
    for (char c : key) hash = hash * 31 + c;
    addr += (hash % (config.eeprom_size - data.size() - 4));
    
    // 写入数据长度
    uint16_t len = data.size();
    eeprom_write_block(&len, (void*)addr, 2);
    addr += 2;
    
    // 写入数据
    if (!data.empty()) {
        eeprom_write_block(data.data(), (void*)addr, len);
    }
    
    statistics_.save_count++;
    return true;
}

bool ConfigManager::load_from_eeprom(const std::string& key, std::vector<uint8_t>& data) {
    ConfigManager_PrivateConfig config = config_manager_get_config_copy();
    uint16_t addr = config.eeprom_start_addr;
    
    // 计算键的哈希作为地址偏移
    uint32_t hash = 0;
    for (char c : key) hash = hash * 31 + c;
    addr += (hash % (config.eeprom_size - 4));
    
    // 读取数据长度
    uint16_t len;
    eeprom_read_block(&len, (void*)addr, 2);
    
    if (len == 0xFFFF || len > config.eeprom_size) {
        return false; // 未初始化或损坏
    }
    
    addr += 2;
    data.resize(len);
    
    if (len > 0) {
        eeprom_read_block(data.data(), (void*)addr, len);
    }
    
    statistics_.load_count++;
    return true;
}

bool ConfigManager::save_to_flash(const std::string& key, const std::vector<uint8_t>& data) {
    ConfigManager_PrivateConfig config = config_manager_get_config_copy();
    uint32_t addr = config.flash_start_addr;
    
    // 计算键的哈希作为地址偏移
    uint32_t hash = 0;
    for (char c : key) hash = hash * 31 + c;
    addr += (hash % (config.flash_size - data.size() - 4)) & ~0xFF; // 页对齐
    
    // 擦除扇区
    flash_range_erase(addr - XIP_BASE, FLASH_SECTOR_SIZE);
    
    // 准备写入数据
    std::vector<uint8_t> write_data;
    uint32_t len = data.size();
    write_data.insert(write_data.end(), (uint8_t*)&len, (uint8_t*)&len + 4);
    write_data.insert(write_data.end(), data.begin(), data.end());
    
    // 页对齐
    while (write_data.size() % FLASH_PAGE_SIZE) {
        write_data.push_back(0xFF);
    }
    
    // 写入Flash
    flash_range_program(addr - XIP_BASE, write_data.data(), write_data.size());
    
    statistics_.save_count++;
    return true;
}

bool ConfigManager::load_from_flash(const std::string& key, std::vector<uint8_t>& data) {
    ConfigManager_PrivateConfig config = config_manager_get_config_copy();
    uint32_t addr = config.flash_start_addr;
    
    // 计算键的哈希作为地址偏移
    uint32_t hash = 0;
    for (char c : key) hash = hash * 31 + c;
    addr += (hash % (config.flash_size - 4)) & ~0xFF; // 页对齐
    
    // 读取数据长度
    uint32_t len = *(uint32_t*)addr;
    
    if (len == 0xFFFFFFFF || len > config.flash_size) {
        return false; // 未初始化或损坏
    }
    
    addr += 4;
    data.resize(len);
    
    if (len > 0) {
        memcpy(data.data(), (void*)addr, len);
    }
    
    statistics_.load_count++;
    return true;
}

void ConfigManager::handle_auto_save() {
    ConfigManager_PrivateConfig config = config_manager_get_config_copy();
    if (!config.enable_auto_save) {
        return;
    }
    
    uint32_t current_time = statistics_.uptime_seconds;
    if (current_time - last_auto_save_time_ >= config.auto_save_interval) {
        save_all_configs();
        last_auto_save_time_ = current_time;
    }
}

void ConfigManager::notify_change(const std::string& key, const ConfigItem& item) {
    if (change_callback_) {
        change_callback_(key, item);
    }
}

void ConfigManager::log_debug(const std::string& message) {
    if (debug_enabled_) {
        printf("[CONFIG_DEBUG] %s\n", message.c_str());
    }
}

void ConfigManager::log_error(const std::string& message) {
    printf("[CONFIG_ERROR] %s\n", message.c_str());
}

// 类型转换辅助实现
std::vector<uint8_t> ConfigManager::to_bytes(bool value) {
    return {static_cast<uint8_t>(value ? 1 : 0)};
}

std::vector<uint8_t> ConfigManager::to_bytes(uint8_t value) {
    return {value};
}

std::vector<uint8_t> ConfigManager::to_bytes(uint16_t value) {
    return {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
}

std::vector<uint8_t> ConfigManager::to_bytes(uint32_t value) {
    return {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF)
    };
}

std::vector<uint8_t> ConfigManager::to_bytes(float value) {
    union { float f; uint32_t i; } converter;
    converter.f = value;
    return to_bytes(converter.i);
}

std::vector<uint8_t> ConfigManager::to_bytes(const std::string& value) {
    std::vector<uint8_t> result(value.begin(), value.end());
    return result;
}

bool ConfigManager::from_bytes(const std::vector<uint8_t>& data, bool& value) {
    if (data.size() != 1) return false;
    value = data[0] != 0;
    return true;
}

bool ConfigManager::from_bytes(const std::vector<uint8_t>& data, uint8_t& value) {
    if (data.size() != 1) return false;
    value = data[0];
    return true;
}

bool ConfigManager::from_bytes(const std::vector<uint8_t>& data, uint16_t& value) {
    if (data.size() != 2) return false;
    value = data[0] | (static_cast<uint16_t>(data[1]) << 8);
    return true;
}

bool ConfigManager::from_bytes(const std::vector<uint8_t>& data, uint32_t& value) {
    if (data.size() != 4) return false;
    value = data[0] | (static_cast<uint32_t>(data[1]) << 8) |
            (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
    return true;
}

bool ConfigManager::from_bytes(const std::vector<uint8_t>& data, float& value) {
    if (data.size() != 4) return false;
    uint32_t int_value;
    if (!from_bytes(data, int_value)) return false;
    union { float f; uint32_t i; } converter;
    converter.i = int_value;
    value = converter.f;
    return true;
}

bool ConfigManager::from_bytes(const std::vector<uint8_t>& data, std::string& value) {
    value = std::string(data.begin(), data.end());
    return true;
}