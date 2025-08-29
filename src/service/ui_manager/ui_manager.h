#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "../../protocol/st7735s/st7735s.h"
#include "../../protocol/hid/hid.h"
#include "../config_manager/config_types.h"
#include "engine/graphics_engine.h"
#include "engine/font_system.h"
#include "page/page_template.h"
#include "engine/ui_constructs.h"
#include "page/page_manager.h"
#include "page/page_types.h"
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
// 使用page模块中的类型定义
using UIPage = ui::UIPage;
using UIEvent = ui::UIEvent;
using JoystickButton = ui::JoystickButton;
using ErrorType = ui::ErrorType;
using ErrorInfo = ui::ErrorInfo;
using PageData = ui::PageData;
using UIStatistics = ui::UIStatistics;
using UIEventCallback = ui::UIEventCallback;
using UIPageCallback = ui::UIPageCallback;


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
    
    // 页面模板控制
    bool enable_page_template_system(bool enable);
    bool is_page_template_enabled() const;
    PageTemplate* get_page_template() const;
    
    // 页面管理器访问
    ui::PageManager* get_page_manager() const;

    // 页面ID转换辅助函数
    std::string page_id_to_string(ui::UIPage page_id) const;

    // 统一的摇杆和菜单交互接口
    bool handle_joystick_input(ui::JoystickButton button);
    bool navigate_menu(bool up);
    bool navigate_menu_horizontal(bool right);
    bool confirm_selection();
    bool handle_back_navigation();
    
    // 动态光标渲染
    void render_cursor_indicator();
    int get_current_menu_index() const;
    int get_menu_item_count() const;
    
    // 息屏管理
    bool set_screen_timeout(uint16_t timeout_seconds);
    bool is_screen_off() const;
    bool wake_screen();

    // 主循环
    void task();
    
    // 调试功能
    void enable_debug_output(bool enabled);
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
    bool use_page_template_;            // 是否使用页面模板系统
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
    
    // 页面数据
    PageData page_data_;
    
    // 构造体系统 - 引擎核心组件，始终启用
    MenuInteractionSystem* menu_system_;     // 菜单交互系统
    PageNavigationManager* nav_manager_;     // 页面导航管理器
    ui::PageManager* page_manager_;          // 页面管理器
    
    // 故障管理
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

    // 页面模板系统
    bool init_page_template();
    void deinit_page_template();
    void draw_page_with_template();
    void update_page_template_content();
    
    // 页面数据管理
    void reset_page_data();
    
    // 30fps刷新任务
    void display_refresh_task();
    void refresh_task_30fps();
    static void display_refresh_timer_callback(void* arg);
    
    // 页面创建和管理
    void destroy_current_page();     // 销毁当前页面
    
    // 输入处理
    void handle_navigation_input(bool up);
    void handle_confirm_input();
    
    // 屏保和背光管理
    inline void handle_backlight();
    inline void handle_screen_timeout();
    
    // 工具函数
    bool is_page_valid(UIPage page) const;
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
};

#endif // UI_MANAGER_H