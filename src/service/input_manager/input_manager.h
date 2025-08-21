#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include "../../protocol/gtx312l/gtx312l.h"
#include "../../protocol/hid/hid.h"
#include "../../protocol/mcp23s17/mcp23s17.h"
#include <vector>
#include <array>
#include <functional>
#include <string>
#include <map>
#include <cstdint>


// 物理点位配置
struct PhysicalPoint {
    std::string device_name;    // 设备名称 (I2C通道+地址)
    uint8_t channel;            // 通道号 (0-11)
    uint8_t threshold;          // 触发阈值
    std::string name;           // 点位名称
    bool enabled;               // 是否启用
    
    PhysicalPoint() 
        : device_name(""), channel(0), threshold(50), name(""), enabled(false) {}
};

// 逻辑区域配置 (Serial模式)
struct LogicalArea {
    std::string name;           // 逻辑区域名称 (如 "A1", "B2")
    std::vector<uint8_t> physical_points; // 映射的物理点位索引
    bool enabled;               // 是否启用
    
    LogicalArea() : name(""), enabled(false) {}
};

// 逻辑点位配置 (HID模式)
struct LogicalPoint {
    uint16_t x_coord;           // X坐标
    uint16_t y_coord;           // Y坐标
    std::vector<uint8_t> physical_points; // 映射的物理点位索引
    bool enabled;               // 是否启用
    
    LogicalPoint() : x_coord(0), y_coord(0), enabled(false) {}
};

// 键盘按键配置
struct KeyboardKey {
    uint8_t gpio_pin;           // GPIO引脚号
    bool is_mcp_pin;            // 是否为MCP23S17引脚
    std::string name;           // 按键名称
    bool enabled;               // 是否启用
    std::vector<uint32_t> hid_keys; // 映射的HID按键 (最多3个组合键)
    
    KeyboardKey() : gpio_pin(0), is_mcp_pin(false), name(""), enabled(false) {}
};

// 触摸映射配置 (Serial模式: 物理点位->逻辑区域)
struct TouchMappingSerial {
    std::map<uint8_t, std::vector<std::string>> physical_to_logical; // 物理点位->逻辑区域名称列表
    std::map<std::string, std::vector<uint8_t>> logical_to_physical; // 逻辑区域->物理点位列表
};

// 触摸映射配置 (HID模式: 物理点位->逻辑点位->XY坐标)
struct TouchMappingHID {
    std::map<uint8_t, std::vector<uint8_t>> physical_to_logical; // 物理点位->逻辑点位索引列表
    std::vector<LogicalPoint> logical_points; // 逻辑点位配置
};

// 键盘映射配置 (物理按键->键盘逻辑区->HID键盘映射)
struct KeyboardMapping {
    std::map<uint8_t, std::vector<uint32_t>> key_to_hid; // 按键索引->HID按键列表
    std::vector<KeyboardKey> keyboard_keys; // 键盘按键配置
};

// 通道使用位图 (每个GTX312L模块一个)
struct ChannelBitmap {
    std::string device_name;    // 设备名称
    uint16_t enabled_channels;  // 启用的通道位图 (bit0-bit11)
    uint16_t mapped_channels;   // 已映射的通道位图
};

// 自定义映射处理函数类型
using CustomMappingHandler = std::function<void(uint8_t point_index, bool pressed, uint8_t pressure)>;

// 触摸事件
struct TouchEvent {
    enum class Type {
        PRESS,                  // 按下
        RELEASE,                // 释放
        MOVE,                   // 移动
        HOLD                    // 保持
    };
    
    Type type;                  // 事件类型
    uint8_t point_index;        // 点位索引
    uint16_t x, y;              // 坐标
    uint8_t pressure;           // 压力值
    uint32_t timestamp;         // 时间戳
    uint32_t duration;          // 持续时间 (仅用于HOLD)
    
    TouchEvent() 
        : type(Type::PRESS), point_index(0), x(0), y(0)
        , pressure(0), timestamp(0), duration(0) {}
};

// 输入统计信息
struct InputStatistics {
    uint32_t total_touches;     // 总触摸次数
    uint32_t valid_touches;     // 有效触摸次数
    uint32_t false_positives;   // 误触次数
    uint32_t missed_touches;    // 漏检次数
    uint32_t multi_touches;     // 多点触摸次数
    std::array<uint32_t, 34> point_counts; // 每个点位的触摸次数
    uint32_t last_reset_time;   // 上次重置时间
    
    InputStatistics() 
        : total_touches(0), valid_touches(0), false_positives(0)
        , missed_touches(0), multi_touches(0), last_reset_time(0) {
        point_counts.fill(0);
    }
};

// 映射执行统计信息
struct MappingStatistics {
    uint32_t execution_count;   // 执行次数
    uint32_t last_execution_time; // 上次执行时间
    uint32_t total_execution_time; // 总执行时间
    uint32_t max_execution_time;   // 最大执行时间
    
    MappingStatistics() 
        : execution_count(0), last_execution_time(0)
        , total_execution_time(0), max_execution_time(0) {}
};

// 工作模式枚举
enum class InputMode {
    SERIAL,     // Serial模式 - 使用Mai2Serial协议
    HID         // HID模式 - 使用HID协议
};

// 输入管理器私有配置
struct InputManager_PrivateConfig {
    InputMode mode;             // 工作模式
    uint8_t scan_interval_ms;   // 扫描间隔
    bool enable_multi_touch;    // 启用多点触控
    bool enable_palm_rejection; // 启用防误触
    bool enable_auto_calibration; // 启用自动校准
    uint16_t calibration_interval_s; // 校准间隔(秒)
    
    InputManager_PrivateConfig() 
        : mode(InputMode::SERIAL), scan_interval_ms(1)
        , enable_multi_touch(true), enable_palm_rejection(true)
        , enable_auto_calibration(false), calibration_interval_s(300) {}
};

// 输入管理器配置（仅包含服务指针）
struct InputManager_Config {
    // 外部设备指针
    class ConfigManager* config_manager;
    class Mai2Serial* mai2_serial;
    class HID* hid;
    MCP23S17* mcp23s17;
    GTX312L* gtx312l_devices[8];
    uint8_t gtx312l_count;
    
    InputManager_Config() 
        : config_manager(nullptr), mai2_serial(nullptr), hid(nullptr)
        , mcp23s17(nullptr), gtx312l_count(0) {
        for (uint8_t i = 0; i < 8; i++) {
            gtx312l_devices[i] = nullptr;
        }
    }
};

// 回调函数类型
using TouchEventCallback = std::function<void(const TouchEvent&)>;
using InputMappingCallback = std::function<void(uint8_t point_index, bool pressed)>;
using DeviceStatusCallback = std::function<void(const std::string& device_name, bool connected)>;
using CalibrationCallback = std::function<void(const std::string& device_name, bool success)>;

// 前向声明
class InputManager;

// InputManager配置管理纯公开函数
InputManager_PrivateConfig* input_manager_get_config_holder();
bool input_manager_load_config_from_manager(InputManager* manager);
InputManager_PrivateConfig input_manager_get_config_copy();
bool input_manager_write_config_to_manager(InputManager* manager, const InputManager_PrivateConfig& config);

// 输入管理器类
class InputManager {
public:
    // 单例模式
    static InputManager* getInstance();
    
    // 析构函数
    ~InputManager();
    
    // 初始化和释放
    bool init(const InputManager_Config& config);
    void deinit();
    bool is_ready() const;
    
    // 设备管理
    bool add_device(GTX312L* device, const std::string& device_name = "");
    bool remove_device(const std::string& device_name);
    bool get_device_count() const;
    bool get_device_info(const std::string& device_name, bool& connected);
    std::vector<std::string> get_device_names() const;
    
    // 工作模式管理
    bool set_input_mode(InputMode mode);
    InputMode get_input_mode() const;
    bool set_mai2serial_device(class Mai2Serial* mai2_serial);
    class Mai2Serial* get_mai2serial_device() const;
    
    // HID设备设置
    bool set_hid_device(HID* hid_device);
    HID* get_hid_device() const;
    
    // 配置管理已移至纯公开函数
    
    // 物理点位配置
    bool set_physical_point_config(uint8_t point_index, const PhysicalPoint& point);
    bool get_physical_point_config(uint8_t point_index, PhysicalPoint& point);
    bool enable_physical_point(uint8_t point_index, bool enabled);
    bool get_physical_point_enabled(uint8_t point_index) const;
    bool set_physical_point_sensitivity(uint8_t point_index, uint8_t sensitivity);
    bool set_physical_point_threshold(uint8_t point_index, uint8_t threshold);
    
    // 触摸映射管理 (Serial模式)
    bool add_logical_area(const LogicalArea& area);
    bool remove_logical_area(const std::string& name);
    bool map_physical_to_logical_area(uint8_t physical_point, const std::string& logical_area);
    bool unmap_physical_from_logical_area(uint8_t physical_point, const std::string& logical_area);
    std::vector<std::string> get_logical_areas_for_physical(uint8_t physical_point);
    
    // 触摸映射管理 (HID模式)
    bool add_logical_point(const LogicalPoint& point);
    bool remove_logical_point(uint8_t logical_index);
    bool map_physical_to_logical_point(uint8_t physical_point, uint8_t logical_point);
    bool unmap_physical_from_logical_point(uint8_t physical_point, uint8_t logical_point);
    bool set_logical_point_coordinates(uint8_t logical_index, uint16_t x, uint16_t y);
    
    // 键盘映射管理
    bool add_keyboard_key(const KeyboardKey& key);
    bool remove_keyboard_key(uint8_t key_index);
    bool map_key_to_hid(uint8_t key_index, const std::vector<uint32_t>& hid_keys);
    bool get_key_mapping(uint8_t key_index, std::vector<uint32_t>& hid_keys);
    
    // 通道管理
    bool update_channel_bitmap(const std::string& device_name);
    bool get_channel_bitmap(const std::string& device_name, ChannelBitmap& bitmap);
    void enable_all_channels_for_binding();
    void restore_channel_state_after_binding();
    
    // 全局灵敏度控制
    bool set_global_sensitivity(uint8_t sensitivity);
    uint8_t get_global_sensitivity() const;
    bool adjust_global_sensitivity(int8_t delta); // 相对调整
    
    // 灵敏度管理 - 通道级别 (基于设备名称)
    bool set_channel_sensitivity_by_name(const std::string& device_name, uint8_t channel, uint8_t sensitivity);
    bool get_channel_sensitivity_by_name(const std::string& device_name, uint8_t channel, uint8_t& sensitivity);
    
    // 触摸状态获取 (基于设备名称)
    bool get_channel_touch_state_by_name(const std::string& device_name, uint8_t channel, bool& pressed, uint8_t& pressure);
    
    // 自动触摸灵敏度调整
    bool start_auto_sensitivity_adjustment(uint8_t physical_point);
    bool get_auto_sensitivity_result(uint8_t& optimal_sensitivity);
    bool is_auto_sensitivity_running() const;
    
    // 手动触发映射接口
    bool trigger_logical_area_manual(const std::string& logical_area);
    bool trigger_logical_point_manual(uint8_t logical_index);
    bool trigger_hid_coordinate_manual(uint16_t x, uint16_t y);
    
    // 通道管理接口
    void update_channel_bitmaps();                           // 根据映射更新通道位图
    bool enable_device_channel(uint8_t device_id, uint8_t channel, bool enabled);
    bool is_device_channel_enabled(uint8_t device_id, uint8_t channel) const;
    uint16_t get_device_channel_bitmap(uint8_t device_id) const;
    void apply_channel_bitmaps_to_devices();                 // 将位图应用到实际设备
    
    // 校准管理
    bool start_calibration(const std::string& device_name = ""); // 空字符串表示所有设备
    bool stop_calibration();
    bool is_calibrating() const;
    bool get_calibration_progress(uint8_t& progress);
    
    // 触摸检测
    bool get_touch_state(uint8_t point_index, bool& pressed, uint8_t& pressure);
    bool get_all_touch_states(std::array<bool, 34>& states);
    uint8_t get_active_touch_count();
    
    // 坐标转换
    bool point_to_coordinates(uint8_t point_index, uint16_t& x, uint16_t& y);
    bool coordinates_to_point(uint16_t x, uint16_t y, uint8_t& point_index);
    
    // 统计信息
    bool get_statistics(InputStatistics& stats);
    void reset_statistics();
    
    // 回调设置
    void set_touch_event_callback(TouchEventCallback callback);
    void set_input_mapping_callback(InputMappingCallback callback);
    void set_device_status_callback(DeviceStatusCallback callback);
    void set_calibration_callback(CalibrationCallback callback);
    
    // 任务处理 - 根据架构设计要求提供双核心处理接口
    // CPU0任务处理 - 读取原始分区触发bitmap -> 映射到逻辑区 -> [Serial模式](传递给Mai2Light -> 调用Mai2Light的Loop 处理UART消息) / [HID模式](将触摸映射Bitmap发送到FIFO跨核心传输)
    void loop0();
    
    // CPU1任务处理 - 读取MCP23S17数据 -> 映射到键盘逻辑区 -> 映射到HID_KEYBOARD / [如果使用HID模式](接收触摸Bitmap 映射到预先设置好的点位坐标)
    void loop1();
    
    // 调试功能
    void enable_debug_output(bool enabled);
    std::string get_debug_info();
    bool test_point(uint8_t point_index);
    
private:
    // 私有构造函数（单例模式）
    InputManager();
    InputManager(const InputManager&) = delete;
    InputManager& operator=(const InputManager&) = delete;
    
    // 静态实例
    static InputManager* instance_;
    
    // 成员变量
    bool initialized_;
    InputMode current_mode_;        // 当前工作模式
    MCP23S17* mcp23s17_;
    class Mai2Serial* mai2_serial_; // Mai2Serial设备指针
    
    // 设备信息结构
    struct DeviceInfo {
        GTX312L* device;
        std::string name;
        bool connected;
        uint32_t last_scan_time;
        GTX312L_TouchData last_data;
        
        DeviceInfo() : device(nullptr), connected(false), last_scan_time(0) {}
    };
    
    std::map<std::string, DeviceInfo> devices_; // 使用设备名称作为键
    HID* hid_device_;
    
    // 配置数据
    std::array<PhysicalPoint, 72> physical_points_;  // 最多72个物理点位 (6个GTX312L * 12通道)
    TouchMappingSerial serial_mapping_;               // Serial模式映射
    TouchMappingHID hid_mapping_;                     // HID模式映射
    KeyboardMapping keyboard_mapping_;                // 键盘映射
    std::vector<ChannelBitmap> channel_bitmaps_;      // 通道使用位图
    
    // 状态跟踪
    std::array<bool, 72> current_touch_states_;       // 当前触摸状态
    std::array<bool, 16> current_key_states_;         // 当前按键状态 (MCP23S17 16个GPIO)
    
    // 自动灵敏度调整状态
    bool auto_sensitivity_running_;
    uint8_t auto_sensitivity_point_;
    uint8_t auto_sensitivity_result_;
    uint32_t auto_sensitivity_start_time_;
    
    // 校准状态
    bool calibrating_;
    std::string calibration_device_name_;
    uint8_t calibration_progress_;
    uint32_t calibration_start_time_;
    
    // 统计信息
    InputStatistics statistics_;
    std::array<MappingStatistics, 34> mapping_statistics_; // 映射统计信息
    
    // 回调函数
    TouchEventCallback touch_event_callback_;
    InputMappingCallback input_mapping_callback_;
    DeviceStatusCallback device_status_callback_;
    CalibrationCallback calibration_callback_;
    
    // 调试
    bool debug_enabled_;
    
    // 内部方法
    void scan_devices();
    void process_device_data(const std::string& device_name, const GTX312L_TouchData& data);
    void process_touch_event(uint8_t point_index, bool pressed, uint8_t pressure);
    void handle_touch_press(uint8_t point_index, uint8_t pressure);
    void handle_touch_release(uint8_t point_index);
    void handle_touch_hold(uint8_t point_index);
    
    void execute_mapping(uint8_t point_index, bool pressed);
    void send_keyboard_input(const InputMapping& mapping, bool pressed);
    void send_mouse_input(const InputMapping& mapping, bool pressed);
    void send_gamepad_input(const InputMapping& mapping, bool pressed);
    void send_touch_input(const InputMapping& mapping, bool pressed);
    
    void process_mcp23s17_input(const MCP23S17_GPIO_State& gpio_state);
    void process_hid_keyboard_mapping();
    void process_hid_touch_mapping();
    void process_key_input(uint8_t key_index, bool pressed);
    
    // 私有方法声明
    void process_serial_mode_mapping(uint32_t touch_bitmap);
    void process_hid_mode_mapping(uint32_t touch_bitmap);
    void process_auto_sensitivity_adjustment();
    void process_hid_touch_output(uint32_t touch_bitmap);
    void process_keyboard_mapping(uint16_t key_state, uint16_t changed);
    
    // 通道管理辅助方法
    void calculate_required_channels();
    uint8_t get_device_index_by_id(uint8_t device_id) const;
    uint8_t physical_point_to_device_channel(uint8_t physical_point, uint8_t& device_id) const;
    
    bool is_point_valid(uint8_t point_index) const;
    uint8_t device_channel_to_point_index(const std::string& device_name, uint8_t channel) const;
    void point_index_to_device_channel(uint8_t point_index, std::string& device_name, uint8_t& channel) const;
    
    // 设备名称生成工具函数
    std::string generate_device_name(uint8_t i2c_channel, uint8_t i2c_address) const;
    GTX312L* find_device_by_name(const std::string& device_name) const;
    
    void update_statistics(uint8_t point_index, bool pressed);
    void perform_auto_calibration();
    void check_device_connections();
    
    void log_debug(const std::string& message);
};

#endif // INPUT_MANAGER_H