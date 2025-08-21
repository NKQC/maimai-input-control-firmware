#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "lvgl.h"
#include "../../protocol/st7735s/st7735s.h"
#include <string>
#include <functional>
#include <map>
#include <vector>

// 前向声明
class InputManager;
class LightManager;
class ConfigManager;

// UI页面类型
enum class UIPage {
    MAIN = 0,           // 主页面
    STATUS,             // 状态页面
    SETTINGS,           // 设置页面
    CALIBRATION,        // 校准页面
    DIAGNOSTICS,        // 诊断页面
    SENSITIVITY,        // 灵敏度调整页面
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

// UI配置结构
struct UIManager_PrivateConfig {
    uint16_t refresh_rate_ms;       // 刷新率
    uint8_t brightness;             // 屏幕亮度
    bool enable_backlight;          // 启用背光
    uint16_t backlight_timeout;     // 背光超时 (s)
    bool enable_screensaver;        // 启用屏保
    uint16_t screensaver_timeout;   // 屏保超时 (s)
    bool enable_joystick;           // 启用摇杆输入
    uint8_t joystick_sensitivity;   // 摇杆灵敏度

    UIManager_PrivateConfig()
        : refresh_rate_ms(50)
        , brightness(255)
        , enable_backlight(true)
        , backlight_timeout(60)
        , enable_screensaver(false)
        , screensaver_timeout(300)
        , enable_joystick(true)
        , joystick_sensitivity(5) {}
};

// UI配置结构
struct UIManager_Config {
    ConfigManager* config_manager;
    InputManager* input_manager;
    LightManager* light_manager;
    ST7735S* st7735s;

    UIManager_Config()
        : config_manager(nullptr)
        , input_manager(nullptr)
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

// 回调函数类型
using UIEventCallback = std::function<void(UIEvent event, const std::string& element_id, int32_t value)>;
using UIPageCallback = std::function<void(UIPage page)>;
using UIJoystickCallback = std::function<void(JoystickButton button, bool pressed)>;

// 配置管理函数
UIManager_PrivateConfig* ui_manager_get_config_holder();
bool ui_manager_load_config_from_manager(ConfigManager* config_manager);
UIManager_PrivateConfig ui_manager_get_config_copy();
bool ui_manager_write_config_to_manager(ConfigManager* config_manager, const UIManager_PrivateConfig& config);

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
    bool handle_joystick_input(JoystickButton button, bool pressed);
    bool navigate_menu(bool up);
    
    // 事件处理
    bool trigger_event(UIEvent event, const std::string& element_id = "", int32_t value = 0);
    
    // 屏保管理
    bool enable_screensaver(bool enabled);
    bool is_screensaver_active() const;
    bool set_screensaver_timeout(uint16_t timeout_seconds);
    
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
    bool set_sensitivity_for_device(const std::string& device_name, uint8_t channel, uint8_t sensitivity);
    
    // 统计信息
    bool get_statistics(UIStatistics& stats);
    void reset_statistics();
    
    // 回调设置
    void set_event_callback(UIEventCallback callback);
    void set_page_callback(UIPageCallback callback);
    void set_joystick_callback(UIJoystickCallback callback);
    
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
    
    // 成员变量
    bool initialized_;
    ST7735S* display_device_;
    InputManager* input_manager_;
    LightManager* light_manager_;
    ConfigManager* config_manager_;
    
    // LVGL相关
    lv_disp_t* lv_display_;
    lv_indev_t* lv_input_device_;
    lv_disp_draw_buf_t lv_draw_buf_;
    lv_color_t* lv_buf1_;
    lv_color_t* lv_buf2_;
    
    // 页面管理
    std::map<UIPage, lv_obj_t*> pages_;
    UIPage current_page_;
    UIPage previous_page_;
    
    // 状态管理
    bool backlight_enabled_;
    bool screensaver_active_;
    uint32_t last_activity_time_;
    uint32_t last_refresh_time_;
    bool needs_full_refresh_;
    bool debug_enabled_;
    
    // 输入状态
    bool joystick_buttons_[3];      // A, B, CONFIRM按钮状态
    uint32_t button_press_times_[3]; // 按钮按下时间
    uint32_t last_navigation_time_; // 上次导航时间
    int16_t current_menu_index_;    // 当前菜单索引
    bool in_menu_mode_;             // 是否在菜单模式
    
    // 灵敏度调整相关
    std::string selected_device_name_; // 当前选中的设备名称
    uint8_t selected_channel_;         // 当前选中的通道
    lv_obj_t* sensitivity_slider_;     // 灵敏度滑块
    lv_obj_t* device_label_;          // 设备标签
    
    // 统计信息
    UIStatistics statistics_;
    
    // 回调函数
    UIEventCallback event_callback_;
    UIPageCallback page_callback_;
    UIJoystickCallback joystick_callback_;
    
    // LVGL初始化和清理
    bool init_lvgl();
    void deinit_lvgl();
    
    // 显示驱动回调
    static void display_flush_cb(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p);
    static bool input_read_cb(lv_indev_drv_t* indev_drv, lv_indev_data_t* data);
    
    // 页面创建和管理
    void create_main_page();
    void create_status_page();
    void create_settings_page();
    void create_calibration_page();
    void create_diagnostics_page();
    void create_sensitivity_page();
    void create_about_page();
    
    // 页面更新
    void update_main_page();
    void update_status_page();
    void update_settings_page();
    void update_calibration_page();
    void update_diagnostics_page();
    void update_sensitivity_page();
    
    // 事件处理
    static void button_event_cb(lv_event_t* e);
    static void slider_event_cb(lv_event_t* e);
    static void page_event_cb(lv_event_t* e);
    
    // 输入处理
    void process_joystick_event(JoystickButton button, bool pressed);
    void handle_navigation_input(bool up);
    void handle_confirm_input();
    
    // 屏保和背光管理
    void handle_backlight();
    void handle_screensaver();
    
    // 工具函数
    bool is_page_valid(UIPage page) const;
    void mark_page_dirty(UIPage page);
    void log_debug(const std::string& message);
    void log_error(const std::string& message);
    
    // 灵敏度调整辅助函数
    std::vector<std::string> get_available_devices();
    bool detect_touched_channel(std::string& device_name, uint8_t& channel);
    void update_sensitivity_display();
};

#endif // UI_MANAGER_H