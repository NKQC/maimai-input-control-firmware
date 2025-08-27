#pragma once

#include <stdint.h>
#include "../config_manager/config_types.h"

// InputManager配置键定义 - 移除重复定义，使用下方统一定义
#include <functional>
#include <array>
#include <vector>
#include "../../hal/i2c/hal_i2c.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "../../protocol/hid/hid.h"
#include "../../protocol/gtx312l/gtx312l.h"
#include "../../protocol/mcp23s17/mcp23s17.h"
#include "../ui_manager/ui_manager.h"

// 触摸键盘映射预处理定义
#define TOUCH_KEYBOARD_KEY HID_KeyCode::KEY_SPACE  // 默认触摸映射按键

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

// 触摸键盘映射模式
enum class TouchKeyboardMode : uint8_t {
    KEY_ONLY = 0,      // 仅按键触发
    TOUCH_ONLY = 1,    // 仅触摸触发
    BOTH = 2           // 同时触发
};

// 配置键预处理定义
#define INPUTMANAGER_WORK_MODE "input_manager_work_mode"
#define INPUTMANAGER_GTX312L_DEVICES "input_manager_gtx312l_devices"
#define INPUTMANAGER_TOUCH_KEYBOARD_ENABLED "input_manager_touch_keyboard_enabled"
#define INPUTMANAGER_TOUCH_KEYBOARD_MODE "input_manager_touch_keyboard_mode"
#define INPUTMANAGER_DEVICE_COUNT "input_manager_device_count"
#define INPUTMANAGER_PHYSICAL_KEYBOARDS "input_manager_physical_keyboards"
#define INPUTMANAGER_LOGICAL_MAPPINGS "input_manager_logical_mappings"

// 工作模式枚举
enum class InputWorkMode : uint8_t {
    SERIAL_MODE = 0,  // Mai2Serial模式
    HID_MODE = 1      // HID模式
};

// 触摸坐标结构体
struct TouchAxis {
    float x;
    float y;
    
    TouchAxis() : x(0.0f), y(0.0f) {}
    TouchAxis(float x_val, float y_val) : x(x_val), y(y_val) {}
};

// 绑定状态枚举
enum class BindingState : uint8_t {
    IDLE = 0,                    // 空闲状态
    SERIAL_BINDING_INIT,         // Serial绑定初始化
    SERIAL_BINDING_WAIT_TOUCH,   // 等待触摸输入
    SERIAL_BINDING_PROCESSING,   // 处理绑定
    SERIAL_BINDING_COMPLETE,     // 绑定完成
    HID_BINDING_INIT,           // HID绑定初始化
    HID_BINDING_WAIT_TOUCH,     // 等待触摸输入
    HID_BINDING_SET_COORDS,     // 设置坐标
    HID_BINDING_COMPLETE,       // HID绑定完成
    AUTO_SERIAL_BINDING_INIT,   // 自动Serial绑定初始化
    AUTO_SERIAL_BINDING_SCAN,   // 扫描触摸输入
    AUTO_SERIAL_BINDING_WAIT,   // 等待用户确认
    AUTO_SERIAL_BINDING_COMPLETE // 自动绑定完成
};

// GTX312L设备映射结构体 - 基于模块地址的统一结构
struct GTX312L_DeviceMapping {
    uint16_t device_addr;                           // GTX312L_PhysicalAddr的get_device_mask()返回值
    uint8_t sensitivity[12];                        // 每个通道的灵敏度设置
    GTX312L_PortEnableBitmap CH_available;          // 通道是否启用bitmap 启用为1
    Mai2_TouchArea serial_area[12];                 // 12个通道对应的Mai2区域映射
    TouchAxis hid_area[12];                         // 12个通道对应的HID坐标映射
    HID_KeyCode keyboard_keys[12];                  // 12个通道对应的HID键盘按键映射
    
    GTX312L_DeviceMapping() : device_addr(0) {
        // 初始化serial_area为未使用状态
        for (int i = 0; i < 12; i++) {
            sensitivity[i] = 15;                        // 默认灵敏度
            serial_area[i] = MAI2_NO_USED;
            hid_area[i] = TouchAxis(0.0f, 0.0f);
            keyboard_keys[i] = HID_KeyCode::KEY_NONE;   // 默认无按键映射
        }
        CH_available.value = 0x0FFF;  // 启用所有12个通道
    }
};

// 私有配置结构体
struct InputManager_PrivateConfig {
    InputWorkMode work_mode;
    GTX312L_DeviceMapping device_mappings[8];
    uint8_t device_count;
    
    // 物理键盘映射配置
    std::vector<PhysicalKeyboardMapping> physical_keyboard_mappings;
    
    // 逻辑按键映射配置
    std::vector<LogicalKeyMapping> logical_key_mappings;
    
    // 触摸键盘映射配置
    TouchKeyboardMode touch_keyboard_mode;
    bool touch_keyboard_enabled;
    
    InputManager_PrivateConfig() 
        : work_mode(InputWorkMode::SERIAL_MODE)
        , device_count(0)
        , touch_keyboard_mode(TouchKeyboardMode::BOTH)
        , touch_keyboard_enabled(false) {
        // 初始化设备映射
        for (int i = 0; i < 8; i++) {
            device_mappings[i] = GTX312L_DeviceMapping();
        }
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
        UIManager* ui_manager;
        MCP23S17* mcp23s17;
        
        InitConfig() : mai2_serial(nullptr), hid(nullptr), ui_manager(nullptr), mcp23s17(nullptr) {}
    };
    
    // 初始化和去初始化
    bool init(const InitConfig& config);
    void deinit();
    
    // 设备管理
    bool registerGTX312L(GTX312L* device);
    void unregisterGTX312L(GTX312L* device);
    
    // 工作模式设置
    inline bool setWorkMode(InputWorkMode mode);
    inline InputWorkMode getWorkMode() const {
        return config_->work_mode;
    }
    
    // 核心循环函数
    void loop0();  // CPU0：GTX312L触摸采样和Serial/HID处理
    void loop1();  // CPU1：键盘处理和HID发送
    
    // 交互式绑定
    void startSerialBinding(InteractiveBindingCallback callback);
    void startHIDBinding(InteractiveBindingCallback callback);
    bool startAutoSerialBinding(); // 引导式自动绑区(仅Serial模式)
    void cancelBinding();
    
    // 自动绑区辅助方法
    bool isAutoSerialBindingComplete() const;  // 检查自动绑区是否完成
    void confirmAutoSerialBinding();           // 确认自动绑区结果
    
    // HID绑定辅助方法
    void setHIDCoordinates(float x, float y);  // 设置HID绑定坐标
    void confirmHIDBinding();                  // 确认HID绑定
    
    // 灵敏度调整接口
    uint8_t autoAdjustSensitivity(uint16_t device_addr, uint8_t channel); // 指定通道的灵敏度调整
    void setSensitivity(uint16_t device_addr, uint8_t channel, uint8_t sensitivity);
    uint8_t getSensitivity(uint16_t device_addr, uint8_t channel);
    bool set_channel_sensitivity_by_name(const std::string& device_name, uint8_t channel, uint8_t sensitivity);
    
    // UI专用接口 - 获取所有设备状态
    struct TouchDeviceStatus {
        GTX312L_DeviceMapping device;
        uint16_t touch_states;
        std::string device_name;
        bool is_connected;
        
        TouchDeviceStatus() : touch_states(0), is_connected(false) {}
    };
    
    void get_all_device_status(TouchDeviceStatus data[8], int& device_count);
    
    // 映射设置方法
    void setSerialMapping(uint16_t device_addr, uint8_t channel, Mai2_TouchArea area);
    void setHIDMapping(uint16_t device_addr, uint8_t channel, float x, float y);
    void setTouchKeyboardMapping(uint16_t device_addr, uint8_t channel, HID_KeyCode key); // 设置键盘映射
    Mai2_TouchArea getSerialMapping(uint16_t device_addr, uint8_t channel);

    HID_KeyCode getTouchKeyboardMapping(uint16_t device_addr, uint8_t channel);          // 获取键盘映射
    
    // 物理键盘GPIO映射方法
    bool addPhysicalKeyboard(MCU_GPIO gpio, HID_KeyCode default_key = HID_KeyCode::KEY_NONE);
    bool addPhysicalKeyboard(MCP_GPIO gpio, HID_KeyCode default_key = HID_KeyCode::KEY_NONE);
    bool removePhysicalKeyboard(uint8_t gpio_id);
    void clearPhysicalKeyboards();
    const std::vector<PhysicalKeyboardMapping>& getPhysicalKeyboards() const;
    
    // 逻辑按键映射方法
    bool addLogicalKeyMapping(uint8_t gpio_id, HID_KeyCode key);
    bool removeLogicalKeyMapping(uint8_t gpio_id, HID_KeyCode key);
    bool clearLogicalKeyMapping(uint8_t gpio_id);
    void clearAllLogicalKeyMappings();
    const std::vector<LogicalKeyMapping>& getLogicalKeyMappings() const;
    
    // 触摸键盘映射方法
    void setTouchKeyboardEnabled(bool enabled);
    inline bool getTouchKeyboardEnabled() const;
    inline void setTouchKeyboardMode(TouchKeyboardMode mode);
    inline TouchKeyboardMode getTouchKeyboardMode() const;
    
    // 通道控制接口
    void enableAllChannels();   // 启用所有通道(绑定时使用)
    void enableMappedChannels(); // 仅启用已映射的通道
    void updateChannelStatesAfterBinding();  // 绑定后更新通道状态
    
    // 性能监控接口
    uint8_t getTouchSampleRate(uint16_t device_addr);  // 获取触摸IC采样速率
    uint8_t getHIDReportRate();                        // 获取HID键盘回报速率
    
    // 采样计数器（用于实际测量采样频率）
    void incrementSampleCounter();
    void resetSampleCounter();
    
private:
    // 单例模式
    InputManager();
    InputManager(const InputManager&) = delete;
    InputManager& operator=(const InputManager&) = delete;
    
    // 私有成员变量
    static InputManager* instance_;
    
    // 设备管理
    std::vector<GTX312L*> gtx312l_devices_;                    // 注册的GTX312L设备列表

    
    // 触摸状态管理
    GTX312L_SampleResult current_touch_states_[8];     // 每个设备的当前触摸状态(包含时间戳)
    GTX312L_SampleResult previous_touch_states_[8];    // 每个设备的上一次触摸状态
    
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
    bool original_channels_backup_[8][12];   // 原始通道启用状态备份
    
    // HID绑定相关变量
    uint16_t hid_binding_device_addr_;       // HID绑定的设备地址
    uint8_t hid_binding_channel_;            // HID绑定的通道
    float hid_binding_x_, hid_binding_y_;    // HID绑定的坐标
    
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
        uint16_t device_addr;
        uint8_t channel;
        uint8_t original_sensitivity;
        uint8_t current_sensitivity;
        uint8_t touch_found_sensitivity;
        uint8_t touch_lost_sensitivity;
        AutoAdjustState state;
        millis_t state_start_time;
        millis_t stabilize_duration;
        bool active;
        
        AutoAdjustContext() : device_addr(0), channel(0), original_sensitivity(0), 
                             current_sensitivity(0), touch_found_sensitivity(0), 
                             touch_lost_sensitivity(0), state(AutoAdjustState::IDLE), 
                             state_start_time(0), stabilize_duration(100), active(false) {}
    } auto_adjust_context_;
    
    // 协议模块引用
    Mai2Serial* mai2_serial_;                // Mai2Serial实例引用
    HID* hid_;                               // HID实例引用
    UIManager* ui_manager_;                  // UIManager实例引用
    MCP23S17* mcp23s17_;                     // MCP23S17实例引用
    InputManager_PrivateConfig* config_;    // 配置指针缓存
    bool mcp23s17_available_;                // MCP23S17是否可用的缓存状态
    
    KeyboardBitmap gpio_keyboard_bitmap_;  // GPIO键盘bitmap（loop1使用，避免跨核竞态）
    KeyboardBitmap touch_bitmap_cache_;    // 触摸键盘bitmap缓存（从loop0共享内存读取）
    
    // GPIO状态管理
    uint32_t mcu_gpio_states_;               // MCU GPIO状态位图
    uint32_t mcu_gpio_previous_states_;      // MCU GPIO上一次状态
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
    inline void processSerialMode();
    inline void sendHIDTouchData();
    
    // 统一键盘处理函数
    void processAutoAdjustSensitivity();
    
    // GPIO键盘处理函数
    inline void updateGPIOStates();          // 更新GPIO状态
    inline void processGPIOKeyboard();       // 处理GPIO键盘输入
    inline void processTouchKeyboard();      // 处理触摸键盘映射

    // 设备查找
    int findDeviceIndex(uint16_t device_addr);
    GTX312L_DeviceMapping* findDeviceMapping(uint16_t device_addr);
    
    // 绑定处理函数
    void processBinding();
    void processSerialBinding();
    void processHIDBinding();
    void processAutoSerialBinding();
    void backupChannelStates();
    void restoreChannelStates();
    void sendMai2TouchMessage(Mai2_TouchArea area);
    const char* getMai2AreaName(Mai2_TouchArea area);
};
