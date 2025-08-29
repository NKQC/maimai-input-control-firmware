#pragma once

#include "input_device_manager.h"
#include "input_mapping_manager.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "../../protocol/hid/hid.h"
#include "../../protocol/mcp23s17/mcp23s17.h"
#include "../ui_manager/ui_manager.h"
#include <memory>
#include <functional>

/**
 * 重构后的输入管理器 V2
 * 使用组合模式，将职责分离到专门的管理器中
 * 提供更清晰、更易维护的接口
 */

// 工作模式枚举
enum class InputWorkMode {
    SERIAL,     // Mai2串口模式
    HID,        // HID模式
    KEYBOARD    // 键盘模式
};

// 绑定状态枚举
enum class BindingState {
    IDLE,           // 空闲状态
    SERIAL_MANUAL,  // 手动串口绑定
    SERIAL_AUTO,    // 自动串口绑定
    HID_MANUAL,     // 手动HID绑定
    COMPLETED       // 绑定完成
};

// 初始化配置结构体
struct InputManagerV2_InitConfig {
    Mai2Serial* mai2_serial;
    HID* hid;
    MCP23S17* mcp23s17;
    UIManager* ui_manager;
    
    InputManagerV2_InitConfig() 
        : mai2_serial(nullptr), hid(nullptr), mcp23s17(nullptr), ui_manager(nullptr) {}
};

// 绑定回调类型
using BindingProgressCallback = std::function<void(uint8_t current_index, uint8_t total_count, const std::string& area_name)>;
using BindingCompleteCallback = std::function<void(bool success, const std::string& message)>;

class InputManagerV2 {
public:
    // 单例模式
    static InputManagerV2* getInstance();
    
    // 初始化和清理
    bool init(const InputManagerV2_InitConfig& config);
    void deinit();
    
    // 设备管理
    bool registerTouchSensor(std::shared_ptr<TouchSensor> device, uint16_t device_addr);
    bool unregisterTouchSensor(uint16_t device_addr);
    std::vector<TouchDeviceStatus> getAllDeviceStatus() const;
    
    // 工作模式管理
    void setWorkMode(InputWorkMode mode);
    InputWorkMode getWorkMode() const;
    
    // 映射管理
    bool addSerialMapping(uint16_t device_addr, uint8_t channel, Mai2_TouchArea area);
    bool addHIDMapping(uint16_t device_addr, uint8_t channel, float x, float y);
    bool addKeyboardMapping(uint16_t device_addr, uint8_t channel, HID_KeyCode key);
    
    bool removeMapping(uint16_t device_addr, uint8_t channel);
    void clearAllMappings();
    void clearDeviceMappings(uint16_t device_addr);
    
    // 映射查询
    Mai2_TouchArea getSerialMapping(uint16_t device_addr, uint8_t channel) const;
    HIDMapping getHIDMapping(uint16_t device_addr, uint8_t channel) const;
    HID_KeyCode getKeyboardMapping(uint16_t device_addr, uint8_t channel) const;
    
    // 绑定功能
    bool startSerialBinding(BindingProgressCallback progress_cb, BindingCompleteCallback complete_cb);
    bool startHIDBinding(BindingProgressCallback progress_cb, BindingCompleteCallback complete_cb);
    bool startAutoSerialBinding();
    void cancelBinding();
    
    // HID绑定辅助
    void setHIDCoordinates(float x, float y);
    void confirmHIDBinding();
    
    // 绑定状态查询
    BindingState getBindingState() const;
    bool isBindingActive() const;
    uint8_t getCurrentBindingIndex() const;
    uint8_t getTotalBindingCount() const;
    
    // 敏感度管理
    bool setSensitivity(uint16_t device_addr, uint8_t channel, uint8_t sensitivity);
    uint8_t getSensitivity(uint16_t device_addr, uint8_t channel) const;
    uint8_t autoAdjustSensitivity(uint16_t device_addr, uint8_t channel);
    
    // 通道管理
    bool setChannelEnabled(uint16_t device_addr, uint8_t channel, bool enabled);
    bool isChannelEnabled(uint16_t device_addr, uint8_t channel) const;
    void enableAllChannels();
    void enableMappedChannels();
    
    // 主循环
    void loop0(); // 核心0循环
    void loop1(); // 核心1循环
    
    // 统计信息
    uint32_t getSampleRate() const;
    uint32_t getHIDReportRate() const;
    void resetStatistics();
    
    // 配置管理
    bool saveConfiguration();
    bool loadConfiguration();
    
private:
    // 私有构造函数（单例模式）
    InputManagerV2();
    ~InputManagerV2();
    
    // 禁用拷贝构造和赋值
    InputManagerV2(const InputManagerV2&) = delete;
    InputManagerV2& operator=(const InputManagerV2&) = delete;
    
    // 单例实例
    static InputManagerV2* instance_;
    
    // 组件管理器
    std::unique_ptr<InputDeviceManager> device_manager_;
    std::unique_ptr<InputMappingManager> mapping_manager_;
    
    // 外部依赖
    Mai2Serial* mai2_serial_;
    HID* hid_;
    MCP23S17* mcp23s17_;
    UIManager* ui_manager_;
    
    // 工作状态
    InputWorkMode work_mode_;
    bool initialized_;
    
    // 绑定状态
    BindingState binding_state_;
    uint8_t current_binding_index_;
    uint8_t total_binding_count_;
    uint32_t binding_start_time_;
    uint32_t binding_timeout_ms_;
    float hid_binding_x_;
    float hid_binding_y_;
    
    // 绑定回调
    BindingProgressCallback binding_progress_callback_;
    BindingCompleteCallback binding_complete_callback_;
    
    // 统计信息
    uint32_t sample_counter_;
    uint32_t last_sample_time_;
    uint32_t current_sample_rate_;
    uint32_t hid_report_counter_;
    uint32_t last_hid_report_time_;
    uint32_t current_hid_report_rate_;
    
    // 内部处理函数
    void processDeviceEvents();
    void processBindingLogic();
    void processSerialBinding();
    void processHIDBinding();
    void processAutoSerialBinding();
    
    // 映射事件处理
    void onSerialMapping(Mai2_TouchArea area, bool pressed);
    void onHIDMapping(float x, float y, bool pressed);
    void onKeyboardMapping(HID_KeyCode key, bool pressed);
    
    // 设备事件处理
    void onDeviceEvent(uint16_t device_addr, uint32_t touch_state);
    void onDeviceStatus(uint16_t device_addr, bool connected);
    
    // 统计更新
    void updateStatistics();
    
    // 绑定辅助函数
    Mai2_TouchArea getBindingArea(uint8_t index) const;
    std::string getBindingAreaName(uint8_t index) const;
    void completeBinding(bool success, const std::string& message);
    
    // 配置辅助函数
    bool validateConfiguration() const;
};