#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "../../protocol/st7735s/st7735s.h"
#include "../../protocol/hid/hid.h"
#include "../config_manager/config_types.h"
#include "graphics_engine.h"
#include "font_system.h"
#include <string>
#include <functional>
#include <map>
#include <vector>

// 前向声明
class InputManager;
class LightManager;
class ConfigManager;

// UIManager配置键定义
#define UIMANAGER_REFRESH_RATE "UIMANAGER_REFRESH_RATE"
#define UIMANAGER_BRIGHTNESS "UIMANAGER_BRIGHTNESS"
#define UIMANAGER_ENABLE_BACKLIGHT "UIMANAGER_ENABLE_BACKLIGHT"
#define UIMANAGER_BACKLIGHT_TIMEOUT "UIMANAGER_BACKLIGHT_TIMEOUT"
#define UIMANAGER_SCREEN_TIMEOUT "UIMANAGER_SCREEN_TIMEOUT"
#define UIMANAGER_ENABLE_JOYSTICK "UIMANAGER_ENABLE_JOYSTICK"
#define UIMANAGER_JOYSTICK_SENSITIVITY "UIMANAGER_JOYSTICK_SENSITIVITY"

// UI页面类型
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
    JOYSTICK_UP,        // 摇杆上
    JOYSTICK_DOWN,      // 摇杆下
    JOYSTICK_CONFIRM,   // 摇杆确认
    SENSITIVITY_SELECT, // 灵敏度选择
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

// 故障信息结构
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

// UI配置结构
struct UIManager_PrivateConfig {
    uint16_t refresh_rate_ms;       // 刷新率
    uint8_t brightness;             // 屏幕亮度
    bool enable_backlight;          // 启用背光
    uint16_t backlight_timeout;     // 背光超时 (s)
    uint16_t screen_timeout;        // 息屏超时 (s)
    bool enable_joystick;           // 启用摇杆输入
    uint8_t joystick_sensitivity;   // 摇杆灵敏度

    UIManager_PrivateConfig()
        : refresh_rate_ms(50)
        , brightness(255)
        , enable_backlight(true)
        , backlight_timeout(60)
        , screen_timeout(300)
        , enable_joystick(true)
        , joystick_sensitivity(5) {}
};

// UI配置结构
struct UIManager_Config {
    ConfigManager* config_manager;
    LightManager* light_manager;
    ST7735S* st7735s;
    
    uint8_t joystick_a_pin;
    uint8_t joystick_b_pin;
    uint8_t joystick_confirm_pin;

    UIManager_Config()
        : config_manager(nullptr)
        , light_manager(nullptr)
        , st7735s(nullptr) {}
};

// UI统计信息
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

// 页面数据结构
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

// 回调函数类型
using UIEventCallback = std::function<void(UIEvent event, const std::string& element_id, int32_t value)>;
using UIPageCallback = std::function<void(UIPage page)>;

// 配置管理函数声明
void uimanager_register_default_configs(config_map_t& default_map);  // 注册默认配置到ConfigManager
UIManager_PrivateConfig* ui_manager_get_config_holder();  // 配置保管函数
bool ui_manager_load_config_from_manager(ConfigManager* config_manager);  // 配置加载函数
UIManager_PrivateConfig ui_manager_get_config_copy();  // 配置读取函数
bool ui_manager_write_config_to_manager(ConfigManager* config_manager, const UIManager_PrivateConfig& config);  // 配置写入函数

// UIManager类
class UIManager {
public:
    // 单例模式
    static UIManager* getInstance();
    
    // 析构函数
    ~UIManager();
    
    // 初始化和清理
    bool init(const UIManager_Config& config);
    void deinit();
    bool is_ready() const;
    
    // 显示设备管理
    bool set_display_device(ST7735S* display);
    ST7735S* get_display_device() const;
    
    // 依赖管理
    bool set_input_manager(InputManager* input_manager);
    bool set_light_manager(LightManager* light_manager);
    bool set_config_manager(ConfigManager* config_manager);
    
    // 页面管理
    bool set_current_page(UIPage page);
    UIPage get_current_page() const;
    std::vector<UIPage> get_available_pages();
    
    // 显示控制
    bool set_brightness(uint8_t brightness);
    uint8_t get_brightness() const;
    bool set_backlight(bool enabled);
    bool get_backlight() const;
    bool clear_screen();
    bool refresh_screen();
    bool force_refresh();
    
    // 输入处理
    bool navigate_menu(bool up);
    
    // 事件处理
    bool trigger_event(UIEvent event, const std::string& element_id = "", int32_t value = 0);
    
    // 息屏管理
    bool set_screen_timeout(uint16_t timeout_seconds);
    bool is_screen_off() const;
    bool wake_screen();
    
    // 页面显示功能
    bool show_status_info();
    bool show_joystick_status();
    bool show_light_status();
    bool show_system_info();
    
    // 校准页面
    bool show_calibration_page();
    bool update_calibration_progress(uint8_t progress);
    
    // 诊断页面
    bool show_diagnostics_page();
    bool update_diagnostics_data();
    
    // 灵敏度调整页面
    bool show_sensitivity_page();
    bool auto_select_sensitivity_point(); // 自动选择需要修改灵敏度的位点

    // 触摸区域映射页面
    bool show_touch_mapping_page();
    bool start_touch_mapping_mode(); // 开始触摸映射模式
    bool cancel_touch_mapping();
    bool show_mapping_selection_ui(); // 显示映射选择界面
    bool update_mapping_display(); // 更新映射显示
    bool handle_touch_mapping_selection(int selection); // 处理触摸映射选择
    
    // 按键映射页面
    bool show_key_mapping_page();
    bool start_key_mapping_mode(); // 开始按键映射模式
    bool show_hid_key_selection(); // 显示HID按键选择
    bool update_key_mapping_display(); // 更新键盘映射显示
    const char* getKeyName(HID_KeyCode key); // 获取按键名称
    HID_KeyCode getKeyCodeFromName(const char* name); // 从名称获取按键代码
    bool handle_hid_key_selection(); // 处理HID按键选择
    bool handle_logical_key_selection(int key_index, const char* key_name); // 处理逻辑按键选择
    bool clear_all_key_mappings(); // 清除所有键盘映射
    bool clear_all_logical_key_mappings(); // 清除所有逻辑按键映射
    
    // 引导式绑区页面
    bool show_guided_binding_page();
    bool start_guided_binding(); // 开始引导式绑区
    bool update_guided_binding_progress(uint8_t step, const std::string& current_area); // 更新绑区进度
    
    // 灯光映射相关
    bool show_light_mapping_page();
    bool start_light_mapping_mode(); // 开始灯光映射模式
    bool show_light_region_selection(); // 显示灯光区域选择
    bool show_neopixel_grid(); // 显示Neopixel网格选择
    bool update_light_mapping_display(); // 更新灯光映射显示
    bool handle_light_region_selection(int region_index); // 处理灯光区域选择
    bool handle_neopixel_selection(int index); // 处理Neopixel选择
    bool save_light_mapping(); // 保存灯光映射
    bool clear_light_mapping(); // 清除灯光映射
    
    // UART设置页面
    bool show_uart_settings_page();
    void update_uart_settings_display(); // 更新UART设置显示
    void handle_mai2serial_baudrate_change(); // 处理mai2serial波特率变化
    void handle_mai2light_baudrate_change(); // 处理mai2light波特率变化
    void save_uart_settings(); // 保存UART设置
    void reset_uart_settings(); // 重置UART设置
    
    // 故障界面管理
    bool show_error_page();
    bool report_error(const ErrorInfo& error);
    bool clear_error();
    bool has_error() const;
    ErrorInfo get_current_error() const;
    std::vector<ErrorInfo> get_error_history() const;
    bool restart_system();
    
    // 绑定状态显示
    bool show_binding_status(const std::string& message, bool is_success = false);
    bool update_binding_progress(uint8_t progress, const std::string& current_step = "");
    bool clear_binding_status();
    
    // 统计信息
    bool get_statistics(UIStatistics& stats);
    void reset_statistics();
    
    // 全局故障接口 - 静态方法，可在任何地方调用
    static bool global_report_error(ErrorType type, const std::string& module_name, 
                                   const std::string& description, uint32_t error_code = 0, 
                                   bool is_critical = false);
    static bool global_has_error();
    static void global_clear_error();
    
    // 回调设置
    void set_event_callback(UIEventCallback callback);
    void set_page_callback(UIPageCallback callback);
    
    // 主循环
    void task();
    
    // 调试功能
    void enable_debug_output(bool enabled);
    std::string get_debug_info();
    bool test_display();
    bool test_joystick();

private:
    // 私有构造函数（单例模式）
    UIManager();
    UIManager(const UIManager&) = delete;
    UIManager& operator=(const UIManager&) = delete;
    
    // 静态实例
    static UIManager* instance_;
    
    // 成员变量 - 按构造函数初始化顺序排列
    bool initialized_;
    ST7735S* display_device_;
    LightManager* light_manager_;
    ConfigManager* config_manager_;
    InputManager* input_manager_;
    GraphicsEngine* graphics_engine_;
    bool page_needs_redraw_;
    int16_t current_menu_index_;
    bool buttons_active_low_;
    bool framebuffer_dirty_;
    UIPage current_page_;
    UIPage previous_page_;
    bool backlight_enabled_;
    bool screen_off_;
    uint32_t last_activity_time_;
    uint32_t last_refresh_time_;
    bool needs_full_refresh_;
    bool debug_enabled_;
    uint32_t last_navigation_time_;
    
    // GPIO配置
    uint8_t joystick_a_pin_;
    uint8_t joystick_b_pin_;
    uint8_t joystick_confirm_pin_;
    
    // 静态framebuffer缓冲区
    static uint16_t framebuffer_[ST7735S_WIDTH * ST7735S_HEIGHT];
    static uint16_t static_framebuffer_[ST7735S_WIDTH * ST7735S_HEIGHT];
    
    // 输入状态
    bool joystick_buttons_[3];      // A, B, CONFIRM按钮状态
    uint32_t button_press_times_[3]; // 按钮按下时间
    
    // 灵敏度调整相关
    int selected_device_index_;        // 当前选中的设备索引
    uint8_t selected_channel_;         // 当前选中的通道
    bool auto_adjust_active_;          // 自动调整是否激活
    
    // 触摸映射相关
    bool touch_mapping_active_;        // 触摸映射是否激活
    int mapping_step_;                 // 映射步骤
    uint16_t mapping_device_addr_;     // 映射的设备地址
    uint8_t mapping_channel_;          // 映射的通道

    
    // 按键映射相关
    bool key_mapping_active_;          // 按键映射是否激活
    int selected_key_index_;           // 选中的按键索引

    
    // 逻辑按键映射相关
    int selected_gpio_;               // 当前选中的GPIO
    
    // 引导式绑区相关
    bool guided_binding_active_;       // 引导式绑区是否激活
    uint8_t binding_step_;             // 绑区步骤

    
    // 灯光映射相关
    bool light_mapping_active_;        // 灯光映射是否激活
    std::string selected_light_region_; // 当前选中的灯光区域
    std::vector<uint8_t> selected_neopixels_; // 选中的Neopixel编号列表

    
    // UART设置相关

    uint32_t current_mai2serial_baudrate_;   // 当前mai2serial波特率
    uint32_t current_mai2light_baudrate_;    // 当前mai2light波特率
    
    // 状态页面相关
    
    // 页面数据
    PageData page_data_;
    
    // 故障管理相关
    ErrorInfo current_error_;          // 当前故障信息
    std::vector<ErrorInfo> error_history_; // 故障历史记录
    bool has_error_;                   // 是否有故障
    static ErrorInfo global_error_;    // 全局故障信息
    static bool global_has_error_;     // 全局故障标志
    
    // 统计信息
    UIStatistics statistics_;
    
    // 回调函数
    UIEventCallback event_callback_;
    UIPageCallback page_callback_;
    
    // 显示刷新相关
    bool init_display();
    void deinit_display();
    void refresh_display();
    
    // 页面绘制函数
    void draw_current_page();
    void draw_main_page();
    void draw_status_page();
    void draw_settings_page();
    void draw_sensitivity_page();
    void draw_touch_mapping_page();
    void draw_key_mapping_page();
    void draw_guided_binding_page();
    void draw_uart_settings_page();
    void draw_calibration_page();
    void draw_diagnostics_page();
    void draw_light_mapping_page();
    void draw_about_page();
    void draw_error_page();
    
    // 页面数据管理
    void reset_page_data();
    
    // 30fps刷新任务
    void display_refresh_task();
    void refresh_task_30fps();
    static void display_refresh_timer_callback(void* arg);
    
    // 页面创建和管理
    void destroy_current_page();     // 销毁当前页面
    
    // 页面更新
    void update_main_page();
    void update_status_page();
    void update_settings_page();
    void update_calibration_page();
    void update_diagnostics_page();
    void update_sensitivity_page();
    void update_touch_mapping_page();
    void update_key_mapping_page();
    void update_guided_binding_page();
    void update_light_mapping_page();
    void update_uart_settings_page();
    void update_error_page();
    
    // 事件回调函数
    
    // 输入处理
    void handle_navigation_input(bool up);
    void handle_confirm_input();
    
    // 屏保和背光管理
    inline void handle_backlight();
    inline void handle_screen_timeout();
    
    // 工具函数
    bool is_page_valid(UIPage page) const;
    void mark_page_dirty(UIPage page);
    void log_debug(const std::string& message);
    void log_error(const std::string& message);
    
    // GPIO初始化
    bool init_gpio();
    
    // GPIO输入处理
    void handle_input();
    
    // 故障处理相关
    void handle_error_detection();
    std::string error_type_to_string(ErrorType type) const;
    void add_error_to_history(const ErrorInfo& error);
    
    // 灵敏度调整辅助函数
    int get_available_device_count();
    bool detect_touched_channel(int& device_index, uint8_t& channel);
    void update_sensitivity_display();
};

#endif // UI_MANAGER_H