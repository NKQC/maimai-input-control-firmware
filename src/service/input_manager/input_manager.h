#pragma once

#include <stdint.h>
#include "../config_manager/config_types.h"

// 时间类型定义
typedef uint32_t millis_t;

// InputManager配置键定义 - 移除重复定义，使用下方统一定义
#include <functional>
#include <array>
#include <vector>
#include <map>
#include "../../hal/i2c/hal_i2c.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "../../protocol/hid/hid.h"
// 统一使用TouchSensor接口
#include "../../protocol/touch_sensor/touch_sensor.h"
#include "../../protocol/mcp23s17/mcp23s17.h"
#include "../ui_manager/ui_manager.h"

// 触摸键盘映射预处理定义
#define TOUCH_KEYBOARD_KEY HID_KeyCode::KEY_SPACE  // 默认触摸映射按键

// 灵敏度设置定义
#define DEFAULT_TOUCH_SENSITIVITY 45  // 默认触摸灵敏度 (0-99范围)
#define MAX_TOUCH_DEVICE 16           // 最大触摸模块数量

// 触摸坐标结构体 - 前向声明，供TouchDeviceMapping使用
struct TouchAxis {
    float x;
    float y;
    
    TouchAxis() : x(0.0f), y(0.0f) {}
    TouchAxis(float x_val, float y_val) : x(x_val), y(y_val) {}
};

// GPIO枚举定义 - 使用6-7位区分类型，0-5位存储引脚编号
// MCU GPIO枚举 (6-7位 = 00)
enum class MCU_GPIO : uint8_t {
    GPIO0 = 0,   GPIO1 = 1,   GPIO2 = 2,   GPIO3 = 3,
    GPIO4 = 4,   GPIO5 = 5,   GPIO6 = 6,   GPIO7 = 7,
    GPIO8 = 8,   GPIO9 = 9,   GPIO10 = 10, GPIO11 = 11,
    GPIO12 = 12, GPIO13 = 13, GPIO14 = 14, GPIO15 = 15,
    GPIO16 = 16, GPIO17 = 17, GPIO18 = 18, GPIO19 = 19,
    GPIO20 = 20, GPIO21 = 21, GPIO22 = 22, GPIO23 = 23,
    GPIO24 = 24, GPIO25 = 25, GPIO26 = 26, GPIO27 = 27,
    GPIO28 = 28, GPIO29 = 29, GPIO_NONE = 63  // 无效GPIO
};

// GPIO类型判断辅助函数
inline bool is_mcu_gpio(uint8_t gpio_val) {
    return (gpio_val & 0xC0) == 0x00;
}

inline bool is_mcp_gpio(uint8_t gpio_val) {
    return (gpio_val & 0xC0) == 0xC0;
}

inline uint8_t get_gpio_pin_number(uint8_t gpio_val) {
    return gpio_val & 0x3F;
}

// 物理键盘映射结构体
struct PhysicalKeyboardMapping {
    union {
        MCU_GPIO mcu_gpio;
        MCP_GPIO mcp_gpio;
        uint8_t gpio;  // 原始GPIO值
    };
    HID_KeyCode default_key;
    
    PhysicalKeyboardMapping() : gpio(static_cast<uint8_t>(MCU_GPIO::GPIO_NONE)), default_key(HID_KeyCode::KEY_NONE) {}
    PhysicalKeyboardMapping(MCU_GPIO mcu, HID_KeyCode key) : mcu_gpio(mcu), default_key(key) {}
    PhysicalKeyboardMapping(MCP_GPIO mcp, HID_KeyCode key) : mcp_gpio(mcp), default_key(key) {}
};

// 单次触发时的状态枚举
enum TouchKeyboard_TriggleStage {
    TOUCH_KEYBOARD_TRIGGLE_STAGE_NONE,
    TOUCH_KEYBOARD_TRIGGLE_STAGE_PRESS,
    TOUCH_KEYBOARD_TRIGGLE_STAGE_RELEASE,
};

// 触摸键盘映射结构体
struct TouchKeyboardMapping {
    uint64_t area_mask;          // 触摸区域掩码，使用Mai2Serial_TouchState的64位格式
    uint32_t hold_time_ms;       // 长按生效时间（毫秒）
    HID_KeyCode key;             // 触发的按键
    uint32_t press_timestamp;    // 按下时间戳（毫秒）
    bool key_pressed;            // 当前按键是否处于按下状态
    bool trigger_once;           // 是否只触发一次（启用时触发只触发一次，关闭时保持现状）
    TouchKeyboard_TriggleStage has_triggered;          // 是否已经触发过（用于trigger_once模式）
    
    TouchKeyboardMapping() : area_mask(0), hold_time_ms(0), key(HID_KeyCode::KEY_NONE), press_timestamp(0), key_pressed(false), trigger_once(false), has_triggered(TOUCH_KEYBOARD_TRIGGLE_STAGE_NONE) {}
    TouchKeyboardMapping(uint64_t mask, uint32_t hold_time, HID_KeyCode trigger_key, bool once = false) 
        : area_mask(mask), hold_time_ms(hold_time), key(trigger_key), press_timestamp(0), key_pressed(false), trigger_once(once), has_triggered(TOUCH_KEYBOARD_TRIGGLE_STAGE_NONE) {}
};

// 逻辑按键映射结构体 - 支持每个GPIO绑定最多3个HID键
struct LogicalKeyMapping {
    uint8_t gpio_id;  // GPIO编号
    HID_KeyCode keys[3];  // 最多3个同时触发的按键
    uint8_t key_count;    // 实际按键数量
    
    LogicalKeyMapping() : gpio_id(static_cast<uint8_t>(MCU_GPIO::GPIO_NONE)), key_count(0) {
        for (int i = 0; i < 3; i++) {
            keys[i] = HID_KeyCode::KEY_NONE;
        }
    }
};

// 独立的区域通道映射配置结构体 - 与设备解耦的通用映射配置
struct AreaChannelMappingConfig {
    // 基础映射结构
    struct AreaChannelMapping {
        uint32_t channel;                           // 32位物理通道地址：高8位设备掩码+低24位bitmap
        
        AreaChannelMapping() : channel(0xFFFFFFFF) {}    // 0xFFFFFFFF表示未映射
    };
    
    // Serial模式：Mai2区域 -> 通道映射（反向映射）
    AreaChannelMapping serial_mappings[34];         // Mai2区域1-34的映射（索引0-33对应区域1-34）
    
    // HID模式：坐标区域 -> 通道映射（支持多个HID触摸点）
    struct HIDAreaMapping {
        uint32_t channel;                           // 32位物理通道地址：高8位设备掩码+低24位bitmap
        TouchAxis coordinates;                      // HID坐标
        
        HIDAreaMapping() : channel(0xFFFFFFFF), coordinates({0.0f, 0.0f}) {}  // 0xFFFFFFFF表示未映射
    };
    HIDAreaMapping hid_mappings[10];                // 支持最多10个HID触摸区域
    
    // 键盘映射：按键 -> 通道
    struct KeyboardMapping {
        uint32_t channel;                           // 32位物理通道地址：高8位设备掩码+低24位bitmap
        
        KeyboardMapping() : channel(0xFFFFFFFF) {}       // 0xFFFFFFFF表示未映射
    };
    std::map<HID_KeyCode, KeyboardMapping> keyboard_mappings; // 按键到通道的映射
    
    AreaChannelMappingConfig() {
        // 初始化所有映射为未映射状态
        for (int i = 0; i < 34; i++) {
            serial_mappings[i] = AreaChannelMapping();
        }
        for (int i = 0; i < 10; i++) {
            hid_mappings[i] = HIDAreaMapping();
        }
    }
};

// 触摸设备映射结构体 - 8位设备掩码版本（剥离映射配置后）
struct TouchDeviceMapping {
    uint8_t device_id_mask;                         // 8位设备ID掩码
    uint8_t max_channels;                           // 设备支持的最大通道数
    uint8_t sensitivity[24];                        // 每个物理通道的灵敏度设置（以设备为单位统一管理）
    uint32_t enabled_channels_mask;                  // 启用的通道掩码（位图，仅低24位有效）
    bool is_connected;                              // 设备连接状态标志
    
    TouchDeviceMapping() : device_id_mask(0), max_channels(0), enabled_channels_mask(0), is_connected(false) {
        // 初始化物理通道灵敏度为默认值
        for (int i = 0; i < 24; i++) {
            sensitivity[i] = DEFAULT_TOUCH_SENSITIVITY;
        }
    }
    
    // 根据通道编号获取灵敏度
    uint8_t getChannelSensitivity(uint8_t channel) const {
        if (channel < 24) {
            return sensitivity[channel];
        }
        return DEFAULT_TOUCH_SENSITIVITY;  // 默认值
    }
    
    // 设置物理通道的灵敏度
    void setChannelSensitivity(uint8_t channel, uint8_t sens) {
        if (channel < 24) {
            sensitivity[channel] = sens;
        }
    }
};

// 触摸键盘映射模式
enum class TouchKeyboardMode : uint8_t {
    KEY_ONLY = 0,      // 仅按键触发
    TOUCH_ONLY = 1,    // 仅触摸触发
    BOTH = 2           // 同时触发
};

// 配置键预处理定义
#define INPUTMANAGER_WORK_MODE "input_manager_work_mode"
#define INPUTMANAGER_TOUCH_DEVICES "input_manager_touch_devices"
#define INPUTMANAGER_TOUCH_KEYBOARD_ENABLED "input_manager_touch_keyboard_enabled"
#define INPUTMANAGER_TOUCH_KEYBOARD_MODE "input_manager_touch_keyboard_mode"
#define INPUTMANAGER_PHYSICAL_KEYBOARDS "input_manager_physical_keyboards"
#define INPUTMANAGER_TOUCH_RESPONSE_DELAY "input_manager_touch_response_delay"
#define INPUTMANAGER_AREA_CHANNEL_MAPPINGS "input_manager_area_channel_mappings"
#define INPUTMANAGER_MAI2SERIAL_BAUD_RATE "input_manager_mai2serial_baud_rate"


// 工作模式枚举
enum class InputWorkMode : uint8_t {
    SERIAL_MODE = 0,  // Mai2Serial模式
    HID_MODE = 1      // HID模式
};

// 绑定状态枚举
enum class BindingState : uint8_t {
    IDLE = 0,        // 空闲状态
    PREPARE,         // 准备绑定
    WAIT_TOUCH,      // 等待触摸输入
    PROCESSING       // 处理绑定
};

// 统一使用TouchDeviceMapping

// 私有配置结构体
struct InputManager_PrivateConfig {
    InputWorkMode work_mode;
    TouchDeviceMapping touch_device_mappings[MAX_TOUCH_DEVICE];    // 触摸设备映射
    uint8_t device_count;
    
    // 独立的区域通道映射配置
    AreaChannelMappingConfig area_channel_mappings;
    
    // 物理键盘映射配置
    std::vector<PhysicalKeyboardMapping> physical_keyboard_mappings;
    
    // 触摸键盘映射配置
    std::vector<TouchKeyboardMapping> touch_keyboard_mappings;
    TouchKeyboardMode touch_keyboard_mode;
    bool touch_keyboard_enabled;
    
    // 触摸响应延迟配置 (0-100ms)
    uint8_t touch_response_delay_ms;
    
    // Mai2Serial配置 - 内部管理
    Mai2Serial_Config mai2serial_config;
    
    InputManager_PrivateConfig() 
        : work_mode(InputWorkMode::SERIAL_MODE)
        , device_count(0)
        , touch_keyboard_mode(TouchKeyboardMode::BOTH)
        , touch_keyboard_enabled(false)
        , touch_response_delay_ms(0)
        , mai2serial_config() {
    }
};

// 配置管理函数声明 - 符合架构要求的配置注册模式
void inputmanager_register_default_configs(config_map_t& default_map);  // [默认配置注册函数]
InputManager_PrivateConfig* inputmanager_get_config_holder();  // [配置保管函数]
bool inputmanager_load_config_from_manager();  // [配置加载函数]
InputManager_PrivateConfig inputmanager_get_config_copy();  // [配置读取函数]
bool inputmanager_write_config_to_manager(const InputManager_PrivateConfig& config);  // [配置写入函数]

// 交互式绑定回调函数类型
using InteractiveBindingCallback = std::function<void(bool success, const char* message)>;

class InputManager {
public:
    // 单例模式
    static InputManager* getInstance();
    ~InputManager();
    
    // 初始化结构体
    struct InitConfig {
        Mai2Serial* mai2_serial;
        HID* hid;
        MCP23S17* mcp23s17;
        UIManager* ui_manager;
        
        InitConfig() : mai2_serial(nullptr), hid(nullptr), mcp23s17(nullptr), ui_manager(nullptr) {}
    };
    
    // 初始化和去初始化
    bool init(const InitConfig& config);
    void deinit();
    
    // 设备注册 - TouchSensor统一接口
    bool registerTouchSensor(TouchSensor* device);
    void unregisterTouchSensor(TouchSensor* device);
    void load_touch_device_config(TouchSensor* device);
    
    // 工作模式设置
    inline bool setWorkMode(InputWorkMode mode);
    inline InputWorkMode getWorkMode() const {
        return config_->work_mode;
    }
    
    // 核心循环函数
    void task0();  // CPU0：TouchSensor触摸采样和Serial/HID处理
    void task1();  // CPU1：键盘处理和HID发送
    
    // 交互式绑定
    void startSerialBinding(InteractiveBindingCallback callback);
    bool startAutoSerialBinding(); // 引导式自动绑区(仅Serial模式)
    void cancelBinding();
    
    // 自动绑区辅助方法
    bool isAutoSerialBindingComplete() const;  // 检查自动绑区是否完成
    void confirmAutoSerialBinding();           // 确认自动绑区结果
    
    // 绑定状态查询
    BindingState getBindingState() const;      // 获取当前绑定状态
    uint8_t getCurrentBindingIndex() const;  // 获取当前绑定区域索引
    void requestCancelBinding();             // 请求取消绑定（UI层调用）
    
    // 灵敏度管理
    uint8_t autoAdjustSensitivity(uint8_t device_id_mask, uint8_t channel); // 指定通道的灵敏度调整
    void setSensitivity(uint8_t device_id_mask, uint8_t channel, uint8_t sensitivity);
    uint8_t getSensitivity(uint8_t device_id_mask, uint8_t channel);
    bool setSensitivityByDeviceName(const std::string& device_name, uint8_t channel, uint8_t sensitivity);
    
    // 校准管理
    void calibrateAllSensors();                        // 校准所有支持校准的传感器
    void calibrateAllSensorsWithTarget(uint8_t sensitivity_target); // 校准所有传感器并指定灵敏度目标
    void calibrateSelectedChannels();                 // 启动特殊校准（仅校准被setChannelCalibrationTarget设置过的通道）
    void setCalibrationTargetByBitmap(uint32_t channel_bitmap, uint8_t target_sensitivity); // 按bitmap设置校准目标灵敏度
    uint8_t getCalibrationProgress();                  // 获取校准进度 (0-255范围)
    inline bool isCalibrationInProgress() const {
        return calibration_in_progress_;
    }
    
    // 根据设备ID掩码获取设备名称 - UI显示时调用
    std::string getDeviceNameByMask(uint32_t device_and_channel_mask) const;
    
    // 根据设备ID掩码获取设备类型 - UI显示时调用
    TouchSensorType getDeviceTypeByMask(uint32_t device_and_channel_mask) const;
    
    // UI专用接口 - 获取所有设备状态
    struct TouchDeviceStatus {
        TouchDeviceMapping touch_device;           // 32位设备映射
        uint32_t touch_states_32bit;              // 32位触摸状态
        std::string device_name;
        TouchSensorType device_type = TouchSensorType::UNKNOWN;
        bool is_connected;
        
        TouchDeviceStatus() : touch_states_32bit(0), is_connected(false) {}
    };
    inline uint8_t get_device_count() { return config_->device_count; }

    void get_all_device_status(TouchDeviceStatus *data);
    
    // 通过设备名称获取TouchSensor实例
    TouchSensor* getTouchSensorByDeviceName(const std::string& device_name);
    
    // 设置Serial映射
    void setSerialMapping(uint8_t device_id_mask, uint8_t channel, Mai2_TouchArea area);
    
    // 设备通道灵敏度管理
    void setDeviceChannelSensitivity(uint8_t device_id_mask, uint8_t channel, uint8_t sensitivity);
    uint8_t getDeviceChannelSensitivity(uint8_t device_id_mask, uint8_t channel);
    
    // 通过逻辑区域映射设置物理通道灵敏度
    void setSerialAreaSensitivity(Mai2_TouchArea area, uint8_t sensitivity);
    void setHIDAreaSensitivity(uint8_t hid_area_index, uint8_t sensitivity);
    void setKeyboardSensitivity(HID_KeyCode key, uint8_t sensitivity);
    // 设置映射 TODO: TouchHID暂未实现UI
    void setHIDMapping(uint8_t device_id_mask, uint8_t channel, float x, float y);
    void setTouchKeyboardMapping(uint8_t device_id_mask, uint8_t channel, HID_KeyCode key);
    Mai2_TouchArea getSerialMapping(uint8_t device_id_mask, uint8_t channel);
    TouchAxis getHIDMapping(uint8_t device_id_mask, uint8_t channel);
    HID_KeyCode getTouchKeyboardMapping(uint8_t device_id_mask, uint8_t channel);
    
    // 检查映射是否存在的接口
    bool hasAvailableSerialMapping() const;
    
    // 物理键盘GPIO映射方法
    bool addPhysicalKeyboard(MCU_GPIO gpio, HID_KeyCode default_key = HID_KeyCode::KEY_NONE);
    bool addPhysicalKeyboard(MCP_GPIO gpio, HID_KeyCode default_key = HID_KeyCode::KEY_NONE);
    bool removePhysicalKeyboard(uint8_t gpio_id);
    void clearPhysicalKeyboards();
    const std::vector<PhysicalKeyboardMapping>& getPhysicalKeyboards() const;
    
    // 触摸键盘映射方法
    void setTouchKeyboardEnabled(bool enabled);
    bool getTouchKeyboardEnabled() const;
    inline void setTouchKeyboardMode(TouchKeyboardMode mode);
    inline TouchKeyboardMode getTouchKeyboardMode() const;
    
    // 触摸键盘映射管理方法
    bool addTouchKeyboardMapping(uint64_t area_mask, uint32_t hold_time_ms, HID_KeyCode key, bool trigger_once = false);
    bool removeTouchKeyboardMapping(uint64_t area_mask, HID_KeyCode key);
    const std::vector<TouchKeyboardMapping>& getTouchKeyboardMappings() const;
    
    // 触摸键盘转换内联接口 - serial模式专用
    inline void checkTouchKeyboardTrigger();  // 检查触发条件并处理按键状态
    
    // 通道控制接口
    void enableAllChannels();   // 启用所有通道(绑定时使用)
    void enableMappedChannels(); // 仅启用已映射的通道
    void clearSerialMappings();  // 清空当前绑区的串口映射
    void updateChannelStatesAfterBinding();  // 绑定后更新通道状态
    
    // 性能监控
    uint32_t getTouchSampleRate();  // 获取触摸IC采样速率
    uint32_t getHIDReportRate();                           // 获取HID键盘回报速率
    
    // 触摸响应延迟管理
    void setTouchResponseDelay(uint8_t delay_ms);  // 设置触摸响应延迟 (0-100ms)
    uint8_t getTouchResponseDelay() const;         // 获取当前延迟设置
    
    // 获取配置副本
    InputManager_PrivateConfig getConfig() const;
    
    // 获取Mai2Serial配置
    Mai2Serial_Config getMai2SerialConfig() const;
    
    // 设置Mai2Serial配置
    bool setMai2SerialConfig(const Mai2Serial_Config& config);
    
    // 获取触摸设备列表
    const std::vector<TouchSensor*>& getTouchSensorDevices() const { return touch_sensor_devices_; }  // 获取当前配置副本
    
    // 采样计数器管理
    inline void incrementSampleCounter();
    void resetSampleCounter();

    // LOG
    // 静态日志函数
    static void log_debug(const std::string& msg);
    static void log_info(const std::string& msg);
    static void log_warning(const std::string& msg);
    static void log_error(const std::string& msg);
    
    // Debug开关控制
    static void set_debug_enabled(bool enabled);
    static bool is_debug_enabled();
    
private:
    // 私有构造函数
    InputManager();
    InputManager(const InputManager&) = delete;
    InputManager& operator=(const InputManager&) = delete;

    // 静态实例
    static InputManager* instance_;
    
    // Debug开关静态变量
    static bool debug_enabled_;
    
    // 设备管理
    std::vector<TouchSensor*> touch_sensor_devices_;           // 注册的TouchSensor设备列表

    // 32位触摸状态管理
    struct TouchDeviceState {
        union {
            uint32_t current_touch_mask;    // 完整的32位触摸状态掩码
            struct {
                uint32_t channel_mask : 24;    // 低24位：通道掩码
                uint8_t device_mask : 8;      // 高8位：设备掩码
            } parts;
        };
        uint32_t previous_touch_mask;   // 上一次触摸状态掩码
        uint32_t timestamp_us;          // 微秒级时间戳
        
        TouchDeviceState() : current_touch_mask(0), 
                           previous_touch_mask(0), timestamp_us(0) {}
    };
    TouchDeviceState touch_device_states_[MAX_TOUCH_DEVICE];          // 每个设备的触摸状态
    
    // 统一使用TouchDeviceState

    // 延迟缓冲区管理 - 优化为只存储Serial数据
    static constexpr uint16_t DELAY_BUFFER_SIZE = 512;  // 扩充缓冲区大小，支持更长延迟和更多数据
    struct DelayedSerialState {
        Mai2Serial_TouchState serial_touch_state;  // 存储64位Serial触摸状态（支持34分区）
        uint32_t timestamp_us;                     // 微秒级时间戳
    };
    DelayedSerialState delay_buffer_[DELAY_BUFFER_SIZE]; // 延迟缓冲区
    uint16_t delay_buffer_head_;                        // 缓冲区头指针
    uint16_t delay_buffer_count_;                       // 缓冲区中的有效数据数量
    
    // GPIO状态管理
    uint32_t mcu_gpio_states_;               // MCU GPIO状态位图
    uint32_t mcu_gpio_previous_states_;      // MCU GPIO上一次状态
    
    // Serial状态桥梁变量 - 用于触摸键盘转换
    Mai2Serial_TouchState serial_state_;    // 当前Serial触摸状态，作为task0和task1之间的桥梁
    
    // 触摸键盘性能优化缓存变量（避免函数调用和局部变量）
    mutable uint32_t touch_keyboard_current_time_cache_; // 当前时间缓存
    mutable bool touch_keyboard_areas_matched_cache_;    // 区域匹配结果缓存
    mutable bool touch_keyboard_hold_satisfied_cache_;   // 长按时间满足缓存
    
    // 采样频率测量相关
    uint32_t sample_counter_;           // 采样计数器
    uint32_t last_reset_time_;          // 上次重置时间
    uint32_t current_sample_rate_;      // 当前采样频率
    
    // 绑定相关私有成员变量
    bool binding_active_;
    InteractiveBindingCallback binding_callback_;
    BindingState binding_state_;
    uint8_t current_binding_index_;          // 当前绑定的区域索引
    uint32_t binding_start_time_;            // 绑定开始时间
    uint32_t binding_timeout_ms_;            // 绑定超时时间
    bool binding_hardware_ops_pending_;      // 绑定硬件操作待执行标志
    bool binding_cancel_pending_;            // 绑定取消操作待执行标志
    bool original_channels_backup_[8][12];   // 原始通道启用状态备份

    // 校准类别枚举
    enum class CalibrationRequestType : uint8_t {
        IDLE = 0,           // 空闲状态
        REQUEST_NORMAL = 1, // 普通校准请求
        REQUEST_SUPER = 2   // 特殊校准请求（按分区）
    };
    
    // 校准管理相关变量
    CalibrationRequestType calibration_request_pending_; // 校准请求类型
    uint8_t calibration_sensitivity_target_;             // 校准灵敏度目标 (1=高敏, 2=默认, 3=低敏)
    bool calibration_in_progress_;                       // 校准正在运行
    
    // 自动灵敏度调整状态机
    enum class AutoAdjustState {
        IDLE,                    // 空闲状态
        FIND_TOUCH_START,       // 开始寻找触摸阈值
        FIND_TOUCH_WAIT,        // 等待触摸检测稳定
        FIND_RELEASE_START,     // 开始寻找释放阈值
        FIND_RELEASE_WAIT,      // 等待释放检测稳定
        VERIFY_THRESHOLD,       // 验证阈值
        COMPLETE                // 完成
    };
    
    struct AutoAdjustContext {
        uint8_t device_id_mask;
        uint8_t channel;
        uint8_t original_sensitivity;
        uint8_t current_sensitivity;
        uint8_t touch_found_sensitivity;
        uint8_t touch_lost_sensitivity;
        AutoAdjustState state;
        millis_t state_start_time;
        millis_t stabilize_duration;
        bool active;
        
        AutoAdjustContext() : device_id_mask(0), channel(0), original_sensitivity(0), 
                             current_sensitivity(0), touch_found_sensitivity(0), 
                             touch_lost_sensitivity(0), state(AutoAdjustState::IDLE), 
                             state_start_time(0), stabilize_duration(100), active(false) {}
    } auto_adjust_context_;
    
    // 协议模块引用
    Mai2Serial* mai2_serial_;                // Mai2Serial实例引用
    HID* hid_;                               // HID实例引用
    MCP23S17* mcp23s17_;                     // MCP23S17实例引用
    InputManager_PrivateConfig* config_;    // 配置指针缓存
    bool mcp23s17_available_;                // MCP23S17是否可用的缓存状态

    // 服务层调用
    UIManager* ui_manager_;
    
    KeyboardBitmap gpio_keyboard_bitmap_;  // GPIO键盘bitmap（loop1使用，避免跨核竞态）
    KeyboardBitmap touch_bitmap_cache_;    // 触摸键盘bitmap缓存（从loop0共享内存读取）
    MCP23S17_GPIO_State mcp_gpio_states_;    // MCP GPIO状态
    MCP23S17_GPIO_State mcp_gpio_previous_states_; // MCP GPIO上一次状态
    
    // GPIO处理缓存变量（避免高频函数中的内存分配）
    mutable uint32_t gpio_mcu_changed_;      // MCU GPIO变化位图缓存
    mutable uint32_t gpio_mcu_inverted_;     // MCU GPIO反转位图缓存
    mutable uint16_t gpio_mcp_changed_a_;    // MCP GPIO Port A变化位图缓存
    mutable uint16_t gpio_mcp_changed_b_;    // MCP GPIO Port B变化位图缓存
    mutable uint16_t gpio_mcp_inverted_a_;   // MCP GPIO Port A反转位图缓存
    mutable uint16_t gpio_mcp_inverted_b_;   // MCP GPIO Port B反转位图缓存
    mutable const PhysicalKeyboardMapping* gpio_mappings_cache_;     // 物理键盘映射指针缓存
    mutable size_t gpio_mapping_count_cache_;                        // 物理键盘映射数量缓存
    mutable const LogicalKeyMapping* gpio_logical_mappings_cache_;   // 逻辑键盘映射指针缓存
    mutable size_t gpio_logical_count_cache_;                        // 逻辑键盘映射数量缓存
    mutable uint8_t gpio_pin_cache_;         // GPIO引脚号缓存
    mutable uint8_t gpio_pin_num_cache_;     // GPIO引脚编号缓存
    mutable uint8_t gpio_bit_pos_cache_;     // GPIO位位置缓存
    mutable bool gpio_current_state_cache_;  // GPIO当前状态缓存
    mutable const PhysicalKeyboardMapping* gpio_mapping_ptr_cache_;  // 当前映射指针缓存
    mutable const HID_KeyCode* gpio_keys_ptr_cache_;                 // 当前按键指针缓存
    
    // 内部处理函数
    inline void updateTouchStates();
    inline void updateAutoCalibrationControl();  // 处理自动校准控制
    inline void sendHIDTouchData();
    void processCalibrationRequest();               // 处理校准请求（在task0中调用）
    
    // 触摸响应延迟管理私有方法
    inline void storeDelayedSerialState();                     // 存储当前Serial状态到延迟缓冲区

    inline void processSerialModeWithDelay();          // 带延迟的Serial模式处理
    
    // 通道mask辅助函数
    static uint32_t generateChannelMask(uint8_t ic_id, uint8_t channel) {
        return ((uint32_t)ic_id << 24) | (1UL << channel);
    }
    
    static uint8_t getICFromChannelMask(uint32_t channel_mask) {
        return (uint8_t)(channel_mask >> 24);
    }
    
    static uint8_t getChannelFromChannelMask(uint32_t channel_mask) {
        uint32_t channel_bitmap = channel_mask & 0x00FFFFFF;
        for (uint8_t i = 0; i < 24; i++) {
            if (channel_bitmap & (1UL << i)) {
                return i;
            }
        }
        return 0xFF; // 无效通道
    }
    
    static bool isChannelMaskValid(uint32_t channel_mask) {
        uint32_t channel_bitmap = channel_mask & 0x00FFFFFF;
        return (channel_mask != 0) && (__builtin_popcount(channel_bitmap) == 1);
    }
    
    // 统一键盘处理函数
    void processAutoAdjustSensitivity();
    
    // GPIO键盘处理函数
    inline void updateGPIOStates();          // 更新GPIO状态
    inline void processGPIOKeyboard();       // 处理GPIO键盘输入
    inline void processTouchKeyboard();      // 处理触摸键盘映射

    // 32位物理通道地址处理辅助函数
    static inline uint32_t encodePhysicalChannelAddress(uint8_t device_mask, uint32_t channel_mask) {
        return (device_mask << 24) | channel_mask;
    }
    
    static inline uint8_t decodeDeviceMask(uint32_t physical_address) {
        return (physical_address >> 24);
    }
    
    static inline uint8_t decodeChannelNumber(uint32_t physical_address) {
        uint32_t bitmap = physical_address & 0x00FFFFFF;
        if (bitmap == 0) return 0xFF;  // 无效
        return static_cast<uint8_t>(__builtin_ctz(bitmap));  // 找到第一个置位的位置
    }
    
    static inline bool isValidPhysicalAddress(uint32_t physical_address) {
        uint32_t bitmap = physical_address & 0x00FFFFFF;
        return bitmap != 0 && __builtin_popcount(bitmap) == 1;  // 确保只有一个位被设置
    }
    
    // 地址处理辅助方法
    int32_t findTouchDeviceIndex(uint8_t device_id_mask);
    TouchDeviceMapping* findTouchDeviceMapping(uint8_t device_id_mask);
    TouchSensor* findTouchSensorByIdMask(uint8_t device_id_mask);
    
    // 绑定处理函数
    void processBinding();
    void processSerialBinding();
    void backupChannelStates();
    void restoreChannelStates();
    const char* getMai2AreaName(Mai2_TouchArea area);
};
