#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <map>
#include <vector>
#include <functional>
#include <memory>
#include <cstdint>

// 前向声明
class InputManager;
class LightManager;
class UIManager;

// 配置数据类型
enum class ConfigType {
    BOOL = 0,
    INT8,
    UINT8,
    INT16,
    UINT16,
    INT32,
    UINT32,
    FLOAT,
    STRING,
    BINARY
};

// 配置访问权限
enum class ConfigAccess {
    READ_ONLY = 0,      // 只读
    READ_WRITE,         // 读写
    WRITE_ONCE,         // 只能写一次
    ADMIN_ONLY          // 仅管理员
};

// 配置存储位置
enum class ConfigStorage {
    RAM = 0,            // 内存（临时）
    EEPROM,             // EEPROM（持久）
    FLASH,              // Flash（持久）
    EXTERNAL            // 外部存储
};

// 配置项定义
struct ConfigItem {
    std::string key;            // 配置键
    std::string description;    // 描述
    ConfigType type;            // 数据类型
    ConfigAccess access;        // 访问权限
    ConfigStorage storage;      // 存储位置
    std::vector<uint8_t> data;  // 数据
    std::vector<uint8_t> default_data; // 默认值
    uint32_t min_value;         // 最小值（数值类型）
    uint32_t max_value;         // 最大值（数值类型）
    uint16_t max_length;        // 最大长度（字符串/二进制）
    bool dirty;                 // 是否需要保存
    uint32_t last_modified;     // 最后修改时间
    
    ConfigItem()
        : type(ConfigType::UINT8)
        , access(ConfigAccess::READ_WRITE)
        , storage(ConfigStorage::EEPROM)
        , min_value(0)
        , max_value(255)
        , max_length(0)
        , dirty(false)
        , last_modified(0) {}
};

// 配置组
struct ConfigGroup {
    std::string name;           // 组名
    std::string description;    // 描述
    std::vector<std::string> keys; // 包含的配置键
    bool enabled;               // 是否启用
    
    ConfigGroup() : enabled(true) {}
};

// 配置模板
struct ConfigTemplate {
    std::string name;           // 模板名
    std::string description;    // 描述
    std::map<std::string, ConfigItem> items; // 配置项
    
    ConfigTemplate() {}
};

// 配置备份信息
struct ConfigBackup {
    std::string name;           // 备份名
    uint32_t timestamp;         // 时间戳
    std::string description;    // 描述
    std::map<std::string, ConfigItem> data; // 备份数据
    uint32_t checksum;          // 校验和
    
    ConfigBackup() : timestamp(0), checksum(0) {}
};

// 配置管理器私有配置
struct ConfigManager_PrivateConfig {
    bool enable_auto_save;      // 启用自动保存
    uint16_t auto_save_interval; // 自动保存间隔 (s)
    bool enable_backup;         // 启用备份
    uint8_t max_backups;        // 最大备份数量
    bool enable_validation;     // 启用验证
    bool enable_encryption;     // 启用加密
    std::string encryption_key; // 加密密钥
    uint16_t eeprom_start_addr; // EEPROM起始地址
    uint16_t eeprom_size;       // EEPROM大小
    uint32_t flash_start_addr;  // Flash起始地址
    uint32_t flash_size;        // Flash大小
    
    ConfigManager_PrivateConfig()
        : enable_auto_save(true)
        , auto_save_interval(60)
        , enable_backup(true)
        , max_backups(3)
        , enable_validation(true)
        , enable_encryption(false)
        , eeprom_start_addr(0)
        , eeprom_size(1024)
        , flash_start_addr(0x10000000)
        , flash_size(4096) {}
};

// 配置管理器服务配置（仅包含服务指针）
struct ConfigManager_ServiceConfig {
    InputManager* input_manager;
    LightManager* light_manager;
    UIManager* ui_manager;
    
    ConfigManager_ServiceConfig()
        : input_manager(nullptr)
        , light_manager(nullptr)
        , ui_manager(nullptr) {}
};

// 配置统计信息
struct ConfigStatistics {
    uint32_t total_reads;       // 总读取次数
    uint32_t total_writes;      // 总写入次数
    uint32_t total_saves;       // 总保存次数
    uint32_t total_loads;       // 总加载次数
    uint32_t validation_errors; // 验证错误次数
    uint32_t backup_count;      // 备份次数
    uint32_t last_reset_time;   // 上次重置时间
    uint32_t uptime_seconds;    // 运行时间
    
    ConfigStatistics()
        : total_reads(0)
        , total_writes(0)
        , total_saves(0)
        , total_loads(0)
        , validation_errors(0)
        , backup_count(0)
        , last_reset_time(0)
        , uptime_seconds(0) {}
};

// 回调函数类型
using ConfigChangeCallback = std::function<void(const std::string& key, const ConfigItem& item)>;
using ConfigSaveCallback = std::function<void(bool success, const std::string& error)>;
using ConfigLoadCallback = std::function<void(bool success, const std::string& error)>;
using ConfigValidationCallback = std::function<bool(const std::string& key, const ConfigItem& item)>;

// 前向声明
class ConfigManager;

// ConfigManager配置管理纯公开函数
ConfigManager_PrivateConfig* config_manager_get_config_holder();
bool config_manager_load_config_from_manager(ConfigManager* config_manager);
ConfigManager_PrivateConfig config_manager_get_config_copy();
bool config_manager_write_config_to_manager(ConfigManager* config_manager, const ConfigManager_PrivateConfig& config);

// 配置管理器类
class ConfigManager {
public:
    // 单例模式
    static ConfigManager* getInstance();
    
    // 析构函数
    ~ConfigManager();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 服务依赖
    bool set_input_manager(InputManager* input_manager);
    bool set_light_manager(LightManager* light_manager);
    bool set_ui_manager(UIManager* ui_manager);
    
    // 配置管理已移至纯公开函数
    
    // 配置项管理
    bool register_config(const std::string& key, const ConfigItem& item);
    bool unregister_config(const std::string& key);
    bool has_config(const std::string& key) const;
    bool get_config_info(const std::string& key, ConfigItem& item);
    std::vector<std::string> get_config_keys() const;
    
    // 基本读写操作
    bool set_bool(const std::string& key, bool value);
    bool get_bool(const std::string& key, bool& value);
    bool set_int8(const std::string& key, int8_t value);
    bool get_int8(const std::string& key, int8_t& value);
    bool set_uint8(const std::string& key, uint8_t value);
    bool get_uint8(const std::string& key, uint8_t& value);
    bool set_int16(const std::string& key, int16_t value);
    bool get_int16(const std::string& key, int16_t& value);
    bool set_uint16(const std::string& key, uint16_t value);
    bool get_uint16(const std::string& key, uint16_t& value);
    bool set_int32(const std::string& key, int32_t value);
    bool get_int32(const std::string& key, int32_t& value);
    bool set_uint32(const std::string& key, uint32_t value);
    bool get_uint32(const std::string& key, uint32_t& value);
    bool set_float(const std::string& key, float value);
    bool get_float(const std::string& key, float& value);
    bool set_string(const std::string& key, const std::string& value);
    bool get_string(const std::string& key, std::string& value);
    bool set_binary(const std::string& key, const std::vector<uint8_t>& value);
    bool get_binary(const std::string& key, std::vector<uint8_t>& value);
    
    // 通用读写操作
    bool set_value(const std::string& key, const std::vector<uint8_t>& data);
    bool get_value(const std::string& key, std::vector<uint8_t>& data);
    
    // 默认值操作
    bool reset_to_default(const std::string& key);
    bool reset_all_to_default();
    bool set_default_value(const std::string& key, const std::vector<uint8_t>& data);
    
    // 组管理
    bool create_group(const std::string& name, const std::string& description = "");
    bool delete_group(const std::string& name);
    bool add_to_group(const std::string& group_name, const std::string& key);
    bool remove_from_group(const std::string& group_name, const std::string& key);
    bool enable_group(const std::string& name, bool enabled);
    std::vector<std::string> get_group_names() const;
    std::vector<std::string> get_group_keys(const std::string& group_name) const;
    
    // 模板管理
    bool save_template(const std::string& name, const std::vector<std::string>& keys, const std::string& description = "");
    bool load_template(const std::string& name);
    bool delete_template(const std::string& name);
    std::vector<std::string> get_template_names() const;
    
    // 持久化操作
    bool save_config(const std::string& key = "");
    bool load_config(const std::string& key = "");
    bool save_all_configs();
    bool load_all_configs();
    
    // 备份和恢复
    bool create_backup(const std::string& name, const std::string& description = "");
    bool restore_backup(const std::string& name);
    bool delete_backup(const std::string& name);
    std::vector<std::string> get_backup_names() const;
    bool get_backup_info(const std::string& name, ConfigBackup& backup);
    
    // 导入导出
    bool export_config(const std::string& filename, const std::vector<std::string>& keys = {});
    bool import_config(const std::string& filename, bool overwrite = false);
    bool export_to_string(std::string& output, const std::vector<std::string>& keys = {});
    bool import_from_string(const std::string& input, bool overwrite = false);
    
    // 验证和校验
    bool validate_config(const std::string& key = "");
    bool validate_all_configs();
    uint32_t calculate_checksum(const std::string& key = "");
    bool verify_checksum(const std::string& key, uint32_t expected_checksum);
    
    // 加密和解密
    bool encrypt_config(const std::string& key);
    bool decrypt_config(const std::string& key);
    bool set_encryption_key(const std::string& key);
    
    // 监控和通知
    bool is_dirty(const std::string& key = "") const;
    bool mark_clean(const std::string& key = "");
    uint32_t get_last_modified(const std::string& key) const;
    
    // 统计信息
    bool get_statistics(ConfigStatistics& stats);
    void reset_statistics();
    
    // 回调设置
    void set_change_callback(ConfigChangeCallback callback);
    void set_save_callback(ConfigSaveCallback callback);
    void set_load_callback(ConfigLoadCallback callback);
    void set_validation_callback(ConfigValidationCallback callback);
    
    // 任务处理
    void task();
    
    // 调试功能
    void enable_debug_output(bool enabled);
    std::string get_debug_info();
    bool test_storage();
    void dump_all_configs();
    
    // 预定义配置注册
    bool register_system_configs();
    bool register_input_configs();
    bool register_light_configs();
    bool register_ui_configs();
    
private:
    // 私有构造函数（单例模式）
    ConfigManager();
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    
    // 静态实例
    static ConfigManager* instance_;
    
    // 私有成员
    bool initialized_;
    InputManager* input_manager_;
    LightManager* light_manager_;
    UIManager* ui_manager_;
    
    // 配置数据
    std::map<std::string, ConfigItem> configs_;
    std::map<std::string, ConfigGroup> groups_;
    std::map<std::string, ConfigTemplate> templates_;
    std::map<std::string, ConfigBackup> backups_;
    
    // 状态变量
    uint32_t last_auto_save_time_;
    bool debug_enabled_;
    
    // 统计信息
    ConfigStatistics statistics_;
    
    // 回调函数
    ConfigChangeCallback change_callback_;
    ConfigSaveCallback save_callback_;
    ConfigLoadCallback load_callback_;
    ConfigValidationCallback validation_callback_;
    
    // 私有方法
    bool validate_key(const std::string& key) const;
    bool validate_access(const std::string& key, bool write_access) const;
    bool validate_type(const ConfigItem& item, const std::vector<uint8_t>& data) const;
    bool validate_range(const ConfigItem& item, const std::vector<uint8_t>& data) const;
    
    // 存储操作
    bool save_to_eeprom(const std::string& key, const std::vector<uint8_t>& data);
    bool load_from_eeprom(const std::string& key, std::vector<uint8_t>& data);
    bool save_to_flash(const std::string& key, const std::vector<uint8_t>& data);
    bool load_from_flash(const std::string& key, std::vector<uint8_t>& data);
    
    // 序列化和反序列化
    bool serialize_config(const ConfigItem& item, std::vector<uint8_t>& data);
    bool deserialize_config(const std::vector<uint8_t>& data, ConfigItem& item);
    
    // 加密解密
    bool encrypt_data(const std::vector<uint8_t>& input, std::vector<uint8_t>& output);
    bool decrypt_data(const std::vector<uint8_t>& input, std::vector<uint8_t>& output);
    
    // 校验和计算
    uint32_t calculate_crc32(const std::vector<uint8_t>& data);
    
    // 自动保存
    void handle_auto_save();
    
    // 工具方法
    std::string get_storage_path(const std::string& key) const;
    bool is_config_valid(const ConfigItem& item) const;
    void notify_change(const std::string& key, const ConfigItem& item);
    void log_debug(const std::string& message);
    void log_error(const std::string& message);
    
    // 类型转换辅助
    template<typename T>
    bool set_numeric_value(const std::string& key, T value);
    
    template<typename T>
    bool get_numeric_value(const std::string& key, T& value);
    
    std::vector<uint8_t> to_bytes(bool value);
    std::vector<uint8_t> to_bytes(int8_t value);
    std::vector<uint8_t> to_bytes(uint8_t value);
    std::vector<uint8_t> to_bytes(int16_t value);
    std::vector<uint8_t> to_bytes(uint16_t value);
    std::vector<uint8_t> to_bytes(int32_t value);
    std::vector<uint8_t> to_bytes(uint32_t value);
    std::vector<uint8_t> to_bytes(float value);
    std::vector<uint8_t> to_bytes(const std::string& value);
    
    bool from_bytes(const std::vector<uint8_t>& data, bool& value);
    bool from_bytes(const std::vector<uint8_t>& data, int8_t& value);
    bool from_bytes(const std::vector<uint8_t>& data, uint8_t& value);
    bool from_bytes(const std::vector<uint8_t>& data, int16_t& value);
    bool from_bytes(const std::vector<uint8_t>& data, uint16_t& value);
    bool from_bytes(const std::vector<uint8_t>& data, int32_t& value);
    bool from_bytes(const std::vector<uint8_t>& data, uint32_t& value);
    bool from_bytes(const std::vector<uint8_t>& data, float& value);
    bool from_bytes(const std::vector<uint8_t>& data, std::string& value);
};

#endif // CONFIG_MANAGER_H