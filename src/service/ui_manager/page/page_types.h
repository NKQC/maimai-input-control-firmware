#pragma once

#include <string>
#include <vector>
#include <functional>
#include <cstdint>

namespace ui {

// UI页面类型枚举
enum class UIPage {
    MAIN = 0,           // 主页面
    STATUS,             // 状态页面
    SETTINGS,           // 设置页面
    CALIBRATION,        // 校准页面
    DIAGNOSTICS,        // 诊断页面
    SENSITIVITY,        // 灵敏度调整页面
    TOUCH_MAPPING,      // 触摸区域映射页面
    KEY_MAPPING,        // 按键映射页面
    GUIDED_BINDING,     // 引导式绑区页面
    LIGHT_MAPPING,      // 灯光区域映射页面
    UART_SETTINGS,      // UART波特率设置页面
    ERROR,              // 故障页面
    ABOUT               // 关于页面
};

// UI事件类型
enum class UIEvent {
    NONE = 0,
    BUTTON_PRESS,       // 按钮按下
    SLIDER_CHANGE,      // 滑块变化
    PAGE_ENTER,         // 页面进入
    PAGE_EXIT,          // 页面退出
    PAGE_CHANGED,       // 页面切换
    JOYSTICK_UP,        // 摇杆上
    JOYSTICK_DOWN,      // 摇杆下
    JOYSTICK_CONFIRM,   // 摇杆确认
    CUSTOM              // 自定义事件
};

// 摇杆按钮类型
enum class JoystickButton {
    BUTTON_A = 0,       // A按钮(上方向)
    BUTTON_B,           // B按钮(下方向)
    BUTTON_CONFIRM      // 确认按钮
};

// 故障类型
enum class ErrorType {
    NONE = 0,           // 无故障
    HARDWARE_INIT,      // 硬件初始化失败
    DISPLAY_ERROR,      // 显示设备故障
    INPUT_ERROR,        // 输入设备故障
    LIGHT_ERROR,        // 灯光设备故障
    CONFIG_ERROR,       // 配置错误
    COMMUNICATION_ERROR,// 通信错误
    MEMORY_ERROR,       // 内存错误
    SENSOR_ERROR,       // 传感器错误
    CALIBRATION_ERROR,  // 校准错误
    UNKNOWN_ERROR       // 未知错误
};

// 故障信息结构体
struct ErrorInfo {
    ErrorType type;             // 故障类型
    std::string module_name;    // 故障模块名称
    std::string description;    // 故障描述
    uint32_t error_code;        // 错误代码
    uint32_t timestamp;         // 故障时间戳
    bool is_critical;           // 是否为严重故障
    
    ErrorInfo() 
        : type(ErrorType::NONE), error_code(0), timestamp(0), is_critical(false) {}
    
    ErrorInfo(ErrorType t, const std::string& module, const std::string& desc, 
              uint32_t code = 0, bool critical = false)
        : type(t), module_name(module), description(desc), error_code(code), 
          timestamp(0), is_critical(critical) {}
};

// 页面数据结构体
struct PageData {
    std::string title;                    // 页面标题
    std::vector<std::string> menu_items;  // 菜单项
    std::vector<std::string> status_items; // 状态项
    std::string content;                  // 页面内容
    int progress_value;                   // 进度值
    std::vector<bool> button_states;      // 按钮状态
    size_t selected_index;                // 选中索引
    
    PageData() : progress_value(0), selected_index(0) {}
};

// UI统计信息结构体
struct UIStatistics {
    uint32_t total_refreshes;       // 总刷新次数
    uint32_t page_changes;          // 页面切换次数
    uint32_t joystick_events;       // 摇杆事件次数
    uint32_t button_presses;        // 按钮按下次数
    uint32_t last_reset_time;       // 上次重置时间
    uint32_t uptime_seconds;        // 运行时间
    uint8_t calibration_progress;   // 校准进度 (0-100)
    uint32_t diagnostic_errors;     // 诊断错误计数
    bool hardware_status;           // 硬件状态

    UIStatistics()
        : total_refreshes(0), page_changes(0), joystick_events(0)
        , button_presses(0), last_reset_time(0), uptime_seconds(0)
        , calibration_progress(0), diagnostic_errors(0), hardware_status(true) {}
};

// 回调函数类型定义
using UIEventCallback = std::function<void(UIEvent event, const std::string& element_id, int32_t value)>;
using UIPageCallback = std::function<void(UIPage page)>;

// 页面ID转字符串函数
std::string page_id_to_string(UIPage page_id);

// 错误类型转字符串函数
std::string error_type_to_string(ErrorType type);

} // namespace ui