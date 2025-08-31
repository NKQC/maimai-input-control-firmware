#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "../../protocol/st7735s/st7735s.h"
#include "../../protocol/hid/hid.h"
#include "../config_manager/config_types.h"
#include "engine/graphics_rendering/graphics_engine.h"
#include "engine/fonts/font_data.h"
#include "engine/page_construction/page_template.h"
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

// 简化的UI类型定义
struct ErrorInfo {
    std::string message;
    uint32_t timestamp = 0;
    
    ErrorInfo() = default;
    explicit ErrorInfo(const std::string& msg) : message(msg) {}
};

struct PageData {
    std::string title;
    std::string content;
    std::vector<std::string> menu_items;
    int32_t selected_index = 0;
    bool has_progress = false;
    float progress_value = 0.0f;
    
    void clear() {
        title.clear();
        content.clear();
        menu_items.clear();
        selected_index = 0;
        has_progress = false;
        progress_value = 0.0f;
    }
};

struct UIStatistics {
    uint32_t page_switches = 0;
    uint32_t input_events = 0;
    uint32_t render_frames = 0;
    uint32_t error_count = 0;
    uint32_t uptime_seconds = 0;
    
    void reset() {
        page_switches = 0;
        input_events = 0;
        render_frames = 0;
        error_count = 0;
        uptime_seconds = 0;
    }
};

// 页面类型枚举 - 用于特殊页面处理
enum class PageType {
    NORMAL,         // 普通页面，使用标准导航
    INT_SETTING,    // INT设置页面，上下调整数值
    SELECTOR        // 选择器页面，摇杆触发回调
};

// 摇杆输入状态枚举
enum class JoystickState {
    UP,
    DOWN,
    CONFIRM
};

// 选择器回调函数类型
using SelectorCallback = std::function<void(JoystickState state)>;

// 简化的回调函数类型
using InputCallback = std::function<void(const std::string& action)>;
using PageCallback = std::function<void(const std::string& from_page, const std::string& to_page)>;


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
    
    // 页面切换和管理
    std::string get_current_page() const;
    std::vector<std::string> get_available_pages();
    
    // 新页面引擎接口
    inline bool switch_to_page(const std::string& page_name);
    const std::string& get_current_page_name() const;
    // Removed PageBase and PageConstructor - using simplified approach
    bool register_main_page();
    bool register_main_menu_page();
    
    // 显示控制
    bool set_brightness(uint8_t brightness);
    uint8_t get_brightness() const;
    bool set_backlight(bool enabled);
    bool get_backlight() const;
    bool clear_screen();
    bool refresh_screen();
    bool force_refresh();
    
    // 页面模板控制
    PageTemplate* get_page_template() const;

    // 页面名称处理函数
    std::string get_page_display_name(const std::string& page_name) const;

    // 简化的输入处理接口（直接函数调用）
    // 简化的摇杆输入处理（仅A和B方向）
    void handle_joystick_a();
    void handle_joystick_b();
    void handle_confirm_input();
    bool handle_joystick_input(int button);
    bool handle_back_navigation();
    
    // 动态光标渲染
    void render_cursor_indicator();
    int get_current_menu_index() const;
    
    // 息屏管理
    bool set_screen_timeout(uint16_t timeout_seconds);
    bool is_screen_off() const;
    bool wake_screen();

    // 主循环
    void task();
    
    // 调试功能
    void enable_debug_output(bool enabled);

    // 给其他子模块提供的静态日志方法
    static void log_debug_static(const std::string& message);
    static void log_error_static(const std::string& message);

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
    PageTemplate* page_template_;       // 页面模板实例
    int16_t current_menu_index_;
    bool buttons_active_low_;

    bool backlight_enabled_;
    bool screen_off_;
    uint32_t last_activity_time_;
    uint32_t last_refresh_time_;
    bool debug_enabled_;
    uint32_t last_navigation_time_;
    
    // 光标闪烁相关
    uint32_t cursor_blink_timer_;
    bool cursor_visible_;
    
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
    
    // 页面数据
    PageData page_data_;

    // 新页面引擎
    static std::string current_page_name_;                // 当前页面名称
    
    // 故障处理相关
    ErrorInfo current_error_;          // 当前故障信息
    std::vector<ErrorInfo> error_history_; // 故障历史记录
    bool has_error_;                   // 是否有故障
    static ErrorInfo global_error_;    // 全局故障信息
    static bool global_has_error_;     // 全局故障标志
    
    // 绑定进度相关
    int binding_step_;                 // 绑定步骤
    
    // 统计信息
    UIStatistics statistics_;
    
    // 回调函数（简化版）
    InputCallback input_callback_;
    PageCallback page_callback_;
    
    // 显示刷新相关
    bool init_display();
    void deinit_display();
    void refresh_display();

    // 页面模板系统
    bool init_page_template();
    void deinit_page_template();
    void draw_page_with_template();
    void update_page_template_content();
    
    // 页面数据管理
    void reset_page_data();
    
    // 30fps刷新任务
    void refresh_task_30fps();
    static void display_refresh_timer_callback(void* arg);
    
    // 处理输入相关
    void handle_navigation_input(bool up);
    
    // 页面特殊处理接口
    PageType get_current_page_type() const;
    bool needs_special_joystick_handling() const;
    bool handle_special_joystick_input(JoystickState state);
    
    // 私有成员函数
    bool is_in_int_setting_page() const;
    bool adjust_int_setting_value(bool increase);
    bool handle_selector_input(JoystickState state);
    
    // handle_confirm_input的辅助函数
    inline void handle_menu_jump(const LineConfig* line_config);
    inline void handle_int_setting(const LineConfig* line_config);
    inline void handle_selector_item(const LineConfig* line_config, LineConfig* mutable_line);
    inline void handle_back_item();
    
    // 背光和屏幕超时处理
    inline void handle_backlight();
    inline void handle_screen_timeout();
    
    // 工具函数
    void log_debug(const std::string& message);
    void log_error(const std::string& message);
    
    // GPIO初始化
    bool init_gpio();
    
    // GPIO输入处理
    void handle_input();
    
    // 异常处理相关（简化版）
    void show_error(const std::string& error_message);
    void clear_error();
    void add_error_to_history(const ErrorInfo& error);
};

#endif // UI_MANAGER_H