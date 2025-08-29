#include "ui_manager.h"
#include "page/page_types.h"
#include "../../service/input_manager/input_manager.h"
#include "../../service/light_manager/light_manager.h"
#include "../../service/config_manager/config_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "../../protocol/st7735s/st7735s.h"
#include "engine/font_system.h"
#include "page/page_template.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

// 静态实例变量定义
UIManager* UIManager::instance_ = nullptr;

// 静态配置实例
static UIManager_PrivateConfig static_config_;

// 全局错误状态
ErrorInfo UIManager::global_error_;
bool UIManager::global_has_error_ = false;

// 静态framebuffer定义
uint16_t UIManager::framebuffer_[ST7735S_WIDTH * ST7735S_HEIGHT];
uint16_t UIManager::static_framebuffer_[ST7735S_WIDTH * ST7735S_HEIGHT];

// 配置管理函数实现
UIManager_PrivateConfig* ui_manager_get_config_holder() {
    return &static_config_;
}

// 默认配置注册函数
void uimanager_register_default_configs(config_map_t& default_map) {
    // 注册UIManager默认配置
    default_map[UIMANAGER_REFRESH_RATE] = ConfigValue((uint16_t)50);  // 默认刷新率50ms
    default_map[UIMANAGER_BRIGHTNESS] = ConfigValue((uint8_t)128);  // 默认亮度50%
    default_map[UIMANAGER_ENABLE_BACKLIGHT] = ConfigValue(true);  // 默认启用背光
    default_map[UIMANAGER_BACKLIGHT_TIMEOUT] = ConfigValue((uint16_t)30000);  // 默认背光超时30秒
    default_map[UIMANAGER_SCREEN_TIMEOUT] = ConfigValue((uint16_t)60000);  // 默认屏幕超时60秒
    default_map[UIMANAGER_ENABLE_JOYSTICK] = ConfigValue(true);  // 默认启用摇杆
    default_map[UIMANAGER_JOYSTICK_SENSITIVITY] = ConfigValue((uint8_t)128);  // 默认摇杆灵敏度50%
}

// 配置加载函数 - 从ConfigManager加载配置到静态配置变量
bool ui_manager_load_config_from_manager(ConfigManager* config_manager) {
    if (config_manager == nullptr) {
        return false;
    }
    
    // 从ConfigManager加载各项配置
    static_config_.refresh_rate_ms = config_manager->get_uint16("UIMANAGER_REFRESH_RATE");
    static_config_.brightness = config_manager->get_uint8("UIMANAGER_BRIGHTNESS");
    static_config_.enable_backlight = config_manager->get_bool("UIMANAGER_ENABLE_BACKLIGHT");
    static_config_.backlight_timeout = config_manager->get_uint16("UIMANAGER_BACKLIGHT_TIMEOUT");
    static_config_.screen_timeout = config_manager->get_uint16("UIMANAGER_SCREEN_TIMEOUT");
    static_config_.enable_joystick = config_manager->get_bool("UIMANAGER_ENABLE_JOYSTICK");
    static_config_.joystick_sensitivity = config_manager->get_uint8("UIMANAGER_JOYSTICK_SENSITIVITY");
    
    return true;
}

// PageManager访问方法
ui::PageManager* UIManager::get_page_manager() const {
    return page_manager_;
}

// 辅助函数：将UIPage枚举转换为字符串
std::string UIManager::page_id_to_string(ui::UIPage page_id) const {
    return ui::page_id_to_string(page_id);
}

// 配置读取函数 - 返回当前配置的副本
UIManager_PrivateConfig ui_manager_get_config_copy() {
    return static_config_;
}

// 配置写入函数 - 将配置写入ConfigManager并保存
bool ui_manager_write_config_to_manager(ConfigManager* config_manager, const UIManager_PrivateConfig& config) {
    if (config_manager == nullptr) {
        return false;
    }
    
    // 更新静态配置
    static_config_ = config;
    
    // 将配置写入ConfigManager
    config_manager->set_uint16("UIMANAGER_REFRESH_RATE", config.refresh_rate_ms);
    config_manager->set_uint8("UIMANAGER_BRIGHTNESS", config.brightness);
    config_manager->set_bool("UIMANAGER_ENABLE_BACKLIGHT", config.enable_backlight);
    config_manager->set_uint16("UIMANAGER_BACKLIGHT_TIMEOUT", config.backlight_timeout);
    config_manager->set_uint16("UIMANAGER_SCREEN_TIMEOUT", config.screen_timeout);
    config_manager->set_bool("UIMANAGER_ENABLE_JOYSTICK", config.enable_joystick);
    config_manager->set_uint8("UIMANAGER_JOYSTICK_SENSITIVITY", config.joystick_sensitivity);
    
    // 保存所有配置到存储
    config_manager->save_config();
    
    return true;
}

// 单例模式实现
UIManager* UIManager::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new UIManager();
    }
    return instance_;
}

// 构造函数
UIManager::UIManager() 
    : initialized_(false)
    , display_device_(nullptr)
    , light_manager_(nullptr)
    , config_manager_(nullptr)
    , input_manager_(nullptr)
    , graphics_engine_(nullptr)
    , page_template_(nullptr)
    , use_page_template_(true)
    , page_needs_redraw_(true)
    , current_menu_index_(0)
    , buttons_active_low_(true)
    , framebuffer_dirty_(true)
    , current_page_(UIPage::STATUS)
    , previous_page_(UIPage::STATUS)
    , backlight_enabled_(true)
    , screen_off_(false)
    , last_activity_time_(0)
    , last_refresh_time_(0)
    , needs_full_refresh_(true)
    , debug_enabled_(false)
    , last_navigation_time_(0)
    , menu_system_(nullptr)
    , nav_manager_(nullptr)
    , page_manager_(nullptr)
    , has_error_(false)
    {
    
    // 初始化按钮状态
    for (int i = 0; i < 3; i++) {
        joystick_buttons_[i] = false;
        button_press_times_[i] = 0;
    }
}

// 析构函数
UIManager::~UIManager() {
    deinit();
}

// 初始化
bool UIManager::init(const UIManager_Config& config) {
    if (initialized_) {
        return true;
    }
    
    // 保存配置
    display_device_ = config.st7735s;
    light_manager_ = config.light_manager;
    config_manager_ = config.config_manager;
    
    // 保存GPIO配置
    joystick_a_pin_ = config.joystick_a_pin;
    joystick_b_pin_ = config.joystick_b_pin;
    joystick_confirm_pin_ = config.joystick_confirm_pin;
    
    if (!display_device_) {
        log_error("Display device is null");
        return false;
    }
    
    // 初始化GPIO
    if (!init_gpio()) {
        log_error("Failed to initialize GPIO");
        return false;
    }
    
    // 初始化显示系统
    if (!init_display()) {
        log_error("Failed to initialize display system");
        return false;
    }
    
    // 初始化页面模板系统
    if (!init_page_template()) {
        log_error("Failed to initialize page template system");
        return false;
    }
    
    // 初始化构造体系统
    menu_system_ = &MenuInteractionSystem::getInstance();
    nav_manager_ = &PageNavigationManager::getInstance();
    
    // 创建并初始化PageManager
    page_manager_ = new ui::PageManager();
    if (!page_manager_->init(graphics_engine_)) {
        log_error("Failed to initialize page manager");
        return false;
    }
    
    log_debug("Construct system initialized with predefined pages");
    
    // 设置初始页面为状态页面
    set_current_page(UIPage::STATUS);
    
    initialized_ = true;
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    
    log_debug("UIManager initialized successfully");
    return true;
}

// GPIO初始化
bool UIManager::init_gpio() {
    // 初始化摇杆A按钮
    gpio_init(joystick_a_pin_);
    gpio_set_dir(joystick_a_pin_, GPIO_IN);
    gpio_pull_up(joystick_a_pin_);
    
    // 初始化摇杆B按钮
    gpio_init(joystick_b_pin_);
    gpio_set_dir(joystick_b_pin_, GPIO_IN);
    gpio_pull_up(joystick_b_pin_);
    
    // 初始化确认按钮
    gpio_init(joystick_confirm_pin_);
    gpio_set_dir(joystick_confirm_pin_, GPIO_IN);
    gpio_pull_up(joystick_confirm_pin_);
    
    log_debug("GPIO initialized successfully");
    return true;
}

// 优化的GPIO输入处理
void UIManager::handle_input() {
    // 批量读取所有按钮状态
    bool button_states[3] = {
        gpio_get(joystick_a_pin_),
        gpio_get(joystick_b_pin_),
        gpio_get(joystick_confirm_pin_)
    };
    
    // 应用active_low逻辑
    if (buttons_active_low_) {
        for (int i = 0; i < 3; i++) {
            button_states[i] = !button_states[i];
        }
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 处理所有按钮状态变化
    for (int i = 0; i < 3; i++) {
        if (button_states[i] != joystick_buttons_[i]) {
            joystick_buttons_[i] = button_states[i];
            
            if (button_states[i]) { // 按钮按下
                button_press_times_[i] = current_time;
                statistics_.joystick_events++;
                last_activity_time_ = current_time;
                
                // 如果息屏，先唤醒屏幕
                if (screen_off_) {
                    wake_screen();
                } else {
                    // 使用统一的摇杆输入处理接口
                    ui::JoystickButton button;
                    switch (i) {
                        case 0: // A按钮 - 向上导航
                            button = ui::JoystickButton::BUTTON_A;
                            break;
                        case 1: // B按钮 - 向下导航
                            button = ui::JoystickButton::BUTTON_B;
                            break;
                        case 2: // 确认按钮
                            button = ui::JoystickButton::BUTTON_CONFIRM;
                            break;
                        default:
                            continue;
                    }
                    handle_joystick_input(button);
                }
            }
        }
    }
}

// 清理
void UIManager::deinit() {
    if (!initialized_) {
        return;
    }
    
    // 清理显示系统
    deinit_display();
    
    // 清理页面模板系统
    deinit_page_template();
    
    // 清理构造体系统
    // 构造体页面已由page_manager管理，无需手动清理
    menu_system_ = nullptr;
    nav_manager_ = nullptr;
    // 构造体系统作为引擎核心组件，无需禁用
    
    display_device_ = nullptr;
    input_manager_ = nullptr;
    light_manager_ = nullptr;
    config_manager_ = nullptr;
    
    initialized_ = false;
    log_debug("UIManager deinitialized");
}

// 检查是否就绪
bool UIManager::is_ready() const {
    return initialized_ && display_device_ != nullptr;
}

// 初始化LVGL
// 初始化显示系统
bool UIManager::init_display() {
    if (!display_device_) {
        log_error("Display device not available for initialization");
        return false;
    }
    
    // 清空静态framebuffer
    memset(framebuffer_, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
    
    // 初始化graphics engine，使用静态framebuffer
    graphics_engine_ = new GraphicsEngine(framebuffer_);
    if (!graphics_engine_) {
        log_error("Failed to create graphics engine");
        return false;
    }
    
    // 清空framebuffer
    graphics_engine_->clear(COLOR_BLACK);
    framebuffer_dirty_ = true;
    
    // 初始化页面模板系统
    if (!init_page_template()) {
        log_error("Failed to initialize page template system");
        delete graphics_engine_;
        graphics_engine_ = nullptr;
        return false;
    }
    
    log_debug("Display system initialized");
    return true;
}

// 清理显示系统
void UIManager::deinit_display() {
    if (graphics_engine_) {
        delete graphics_engine_;
        graphics_engine_ = nullptr;
    }
    // 静态framebuffer无需释放，只需清空
    memset(framebuffer_, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
    
    log_debug("Display system deinitialized");
}

// 显示刷新函数
void UIManager::refresh_display() {
    if (!framebuffer_dirty_) return;

    bool result = display_device_->write_buffer(framebuffer_, SCREEN_BUFFER_SIZE);
    if (result) {
        framebuffer_dirty_ = false;
    }
}

// 30fps刷新任务
void UIManager::refresh_task_30fps() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    // 30fps = 33.33ms间隔
    if (current_time - last_refresh_time_ >= 33) {
        // 检查是否需要重绘页面
        if (page_needs_redraw_) {
            page_needs_redraw_ = false;
            framebuffer_dirty_ = true;
        }
        
        // 刷新显示
        refresh_display();
        
        last_refresh_time_ = current_time;
    }
}

// 设置当前页面
bool UIManager::set_current_page(UIPage page) {
    if (!is_page_valid(page)) {
        return false;
    }
    
    if (current_page_ == page) {
        return true;
    }
    
    // 更新页面状态
    previous_page_ = current_page_;
    current_page_ = page;
    
    // 重置光标位置到第一项
    current_menu_index_ = 0;
    
    // 更新菜单系统的当前页面
    if (menu_system_) {
        menu_system_->switch_to_page(page);
    }
    
    // 更新页面模板的选中索引
    if (use_page_template_ && page_template_) {
        page_template_->set_selected_index(current_menu_index_);
    }
    
    // 标记需要重绘
    page_needs_redraw_ = true;
    
    // 重置页面数据
    reset_page_data();
    
    // 记录活动时间
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    
    log_debug("Page switched from " + std::to_string(static_cast<int>(previous_page_)) + 
              " to " + std::to_string(static_cast<int>(current_page_)));
    
    return true;
}

// 重置页面数据
void UIManager::reset_page_data() {
    page_data_.title = "";
    page_data_.menu_items.clear();
    page_data_.status_items.clear();
    page_data_.content = "";
    page_data_.progress_value = 0;
    page_data_.button_states.clear();
    page_data_.selected_index = current_menu_index_; // 与当前菜单索引保持同步
    
    // 根据页面类型设置默认数据
    switch (current_page_) {
        case UIPage::MAIN:
            page_data_.title = "Main Menu";
            page_data_.menu_items = {"Status", "Settings", "Calibration", "Diagnostics"};
            break;
        case UIPage::STATUS:
            page_data_.title = "System Status";
            page_data_.status_items = {
                "Input Mgr: OK",
                "Light Mgr: OK", 
                "Display: OK",
                "Config: OK",
                "Touch Poll: 0/s",
                "Key Poll: 0/s",
                "Touch Active: 0",
                "Key Active: 0",
                "Uptime: 0s",
                "Memory: OK"
            };
            break;
        case UIPage::SETTINGS:
            page_data_.title = "Settings";
            page_data_.menu_items = {"Sensitivity", "Touch Map", "Key Map", "Guide Bind", "UART Config", "Light Map", "About"};
            break;
        case UIPage::SENSITIVITY:
            page_data_.title = "Sensitivity Settings";
            page_data_.content = "Adjust touch sensitivity";
            // 灵敏度页面不使用菜单项，而是直接调整数值
            break;
        case UIPage::TOUCH_MAPPING:
            page_data_.title = "Touch Mapping";
            page_data_.content = "Configure touch mappings";
            break;
        case UIPage::KEY_MAPPING:
            page_data_.title = "Key Mapping";
            page_data_.content = "Configure key mappings";
            break;
        case UIPage::GUIDED_BINDING:
            page_data_.title = "Guided Binding";
            page_data_.content = "Follow the binding guide";
            break;
        case UIPage::LIGHT_MAPPING:
            page_data_.title = "Light Mapping";
            page_data_.content = "Configure light mappings";
            break;
        case UIPage::UART_SETTINGS:
            page_data_.title = "UART Settings";
            page_data_.menu_items = {"Mai2Serial Baud", "Mai2Light Baud", "Save Settings", "Reset Settings"};
            break;
        case UIPage::CALIBRATION:
            page_data_.title = "Calibration";
            page_data_.content = "Touch calibration in progress";
            break;
        case UIPage::DIAGNOSTICS:
            page_data_.title = "Diagnostics";
            page_data_.content = "System diagnostics";
            break;
        case UIPage::ABOUT:
            page_data_.title = "About";
            page_data_.content = "Maimai Input Control Firmware v3.0";
            break;
        case UIPage::ERROR:
            page_data_.title = "Error";
            page_data_.content = "System error occurred";
            break;
        default:
            page_data_.title = "Unknown Page";
            break;
    }
    
    // 确保选中索引在有效范围内
    int menu_count = get_menu_item_count();
    if (menu_count > 0 && current_menu_index_ >= menu_count) {
        current_menu_index_ = 0;
        page_data_.selected_index = 0;
    }
}

// 销毁当前页面（已不需要，保留空函数以兼容）
void UIManager::destroy_current_page() {
    // 静态framebuffer模式下不需要销毁页面对象
    // 页面数据已通过reset_page_data()重置
}

// 获取当前页面
UIPage UIManager::get_current_page() const {
    return current_page_;
}

// 处理导航输入
void UIManager::handle_navigation_input(bool up) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 防抖处理
    if (current_time - last_navigation_time_ < 150) { // 减少防抖时间以提高响应性
        return;
    }
    last_navigation_time_ = current_time;
    
    // 记录活动时间
    last_activity_time_ = current_time;
    
    // 如果屏幕关闭，唤醒屏幕
    if (screen_off_) {
        wake_screen();
        return; // 唤醒后不处理导航，避免意外操作
    }
    
    // 通用菜单导航逻辑
    int menu_count = get_menu_item_count();
    if (menu_count <= 0) {
        return; // 当前页面没有菜单项
    }
    
    // 确保当前索引在有效范围内
    if (current_menu_index_ < 0 || current_menu_index_ >= menu_count) {
        current_menu_index_ = 0;
    }
    
    // 执行导航
    if (up) {
        current_menu_index_--;
        if (current_menu_index_ < 0) {
            current_menu_index_ = menu_count - 1; // 循环到最后一项
        }
    } else {
        current_menu_index_++;
        if (current_menu_index_ >= menu_count) {
            current_menu_index_ = 0; // 循环到第一项
        }
    }
    
    // 更新页面模板的选中索引
    if (use_page_template_ && page_template_) {
        page_template_->set_selected_index(current_menu_index_);
    }
    
    // 标记页面需要重绘
    page_needs_redraw_ = true;
    
    // 触发导航事件
    if (event_callback_) {
        UIEvent event = up ? UIEvent::JOYSTICK_UP : UIEvent::JOYSTICK_DOWN;
        event_callback_(event, "menu_navigation", current_menu_index_);
    }
    
    log_debug("Navigation: " + std::string(up ? "UP" : "DOWN") + 
              ", Index: " + std::to_string(current_menu_index_) + 
              "/" + std::to_string(menu_count));
}

// 处理确认输入
void UIManager::handle_confirm_input() {
    switch (current_page_) {
        case UIPage::MAIN:
            // 根据菜单索引切换页面
            switch (current_menu_index_) {
                case 0: set_current_page(UIPage::STATUS); break;
                case 1: set_current_page(UIPage::SETTINGS); break;
                case 2: set_current_page(UIPage::CALIBRATION); break;
                case 3: set_current_page(UIPage::DIAGNOSTICS); break;
                case 4: set_current_page(UIPage::SENSITIVITY); break;
                case 5: set_current_page(UIPage::LIGHT_MAPPING); break;
                case 6: set_current_page(UIPage::ABOUT); break;
                case 7: /* 退出或其他操作 */ break;
            }
            break;
            
        case UIPage::SETTINGS:
            // 根据菜单索引切换到设置子页面
            switch (current_menu_index_) {
                case 0: set_current_page(UIPage::UART_SETTINGS); break;
                // 可以在这里添加更多设置子页面
                default: break;
            }
            break;
            
        default:
            // 其他页面返回主页面
            set_current_page(UIPage::MAIN);
            break;
    }
}

// 页面模板系统实现
bool UIManager::init_page_template() {
    if (page_template_) {
        return true; // 已经初始化
    }
    
    page_template_ = new PageTemplate(graphics_engine_);
    if (!page_template_) {
        log_error("Failed to create page template");
        return false;
    }
    
    log_debug("Page template system initialized");
    return true;
}

void UIManager::deinit_page_template() {
    if (page_template_) {
        delete page_template_;
        page_template_ = nullptr;
        log_debug("Page template system deinitialized");
    }
}

void UIManager::draw_page_with_template() {
    if (!page_template_ || !use_page_template_) {
        return;
    }
    
    update_page_template_content();
    page_template_->draw();
}

void UIManager::update_page_template_content() {
    if (!page_template_) {
        return;
    }
    
    // 根据当前页面更新模板内容
    switch (current_page_) {
        case UIPage::MAIN:
            PageTemplates::create_main_menu_page(*page_template_);
            break;
        case UIPage::STATUS:
            PageTemplates::create_status_page(*page_template_);
            break;
        case UIPage::SETTINGS:
            PageTemplates::create_settings_page(*page_template_);
            break;
        case UIPage::SENSITIVITY:
            PageTemplates::create_progress_page(*page_template_, 50.0f);
            break;
        case UIPage::TOUCH_MAPPING:
            PageTemplates::create_dynamic_menu_page(*page_template_, 0);
            break;
        case UIPage::KEY_MAPPING:
            PageTemplates::create_dynamic_menu_page(*page_template_, 0);
            break;
        case UIPage::GUIDED_BINDING:
            PageTemplates::create_progress_page(*page_template_, binding_step_ * 10.0f);
            break;
        case UIPage::ERROR:
            if (has_error_) {
                PageTemplates::create_error_page(*page_template_);
            }
            break;
        default:
            PageTemplates::create_info_page(*page_template_);
            break;
    }
}

bool UIManager::enable_page_template_system(bool enable) {
    use_page_template_ = enable;
    if (enable && !page_template_) {
        return init_page_template();
    }
    return true;
}

bool UIManager::is_page_template_enabled() const {
    return use_page_template_;
}

PageTemplate* UIManager::get_page_template() const {
    return page_template_;
}

// 主循环
void UIManager::task() {
    if (!initialized_) {
        return;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 处理背光和息屏
    handle_backlight();
    handle_screen_timeout();
    
    // 处理故障检测 - 即使息屏也要检测故障
    handle_error_detection();
    
    // 更新统计信息
    statistics_.uptime_seconds = current_time / 1000;
    
    // 处理输入
    handle_input();
    
    // 息屏状态下的节能处理
    if (screen_off_) {
        // 息屏时只进行必要的检测，不进行渲染
        // 检查是否有需要唤醒屏幕的条件
        
        // 检查摇杆输入
        if (static_config_.enable_joystick) {
            // 简单检查摇杆状态，如有输入则唤醒
            for (int i = 0; i < 3; i++) {
                if (joystick_buttons_[i]) {
                    wake_screen();
                    break;
                }
            }
        }

        // 检查是否有故障需要显示
        if (has_error_ || global_has_error_) {
            wake_screen();
        }
        
        // 息屏状态下不进行页面更新
        return;
    }
    
    // 屏幕开启状态下的正常渲染
    // 使用30fps刷新任务进行页面渲染
    refresh_task_30fps();
    
    // 使用构造引擎统一处理页面更新
    if (current_time - last_refresh_time_ >= static_config_.refresh_rate_ms) {
        // 构造引擎会自动处理所有页面的更新和渲染
        if (menu_system_) {
            menu_system_->update();
        }
        last_refresh_time_ = current_time;
    }
}

// 背光管理
void UIManager::handle_backlight() {
    if (!static_config_.enable_backlight) {
        return;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = static_config_.backlight_timeout * 1000;
    
    if (current_time - last_activity_time_ > timeout_ms) {
        if (backlight_enabled_) {
            set_backlight(false);
        }
    } else {
        if (!backlight_enabled_) {
            set_backlight(true);
        }
    }
}

// 息屏管理
void UIManager::handle_screen_timeout() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = static_config_.screen_timeout * 1000;
    if (!screen_off_) {
        if (current_time - last_activity_time_ > timeout_ms) {
            screen_off_ = true;
            // 息屏：关闭背光但保持显示内容
            if (display_device_) {
                display_device_->set_backlight(false);
            }
        }
    }
}

// 设置背光
bool UIManager::set_backlight(bool enabled) {
    backlight_enabled_ = enabled;
    if (display_device_) {
        return display_device_->set_backlight(enabled ? 999 : 0);
    }
    return false;
}

// 获取背光状态
bool UIManager::get_backlight() const {
    return backlight_enabled_;
}

// 设置亮度
bool UIManager::set_brightness(uint8_t brightness) {
    static_config_.brightness = brightness;
    if (display_device_) {
        return display_device_->set_backlight(brightness);
    }
    return false;
}

// 获取亮度
uint8_t UIManager::get_brightness() const {
    return static_config_.brightness;
}

// 清屏
bool UIManager::clear_screen() {
    if (display_device_) {
        // 创建黑色填充缓冲区并写入屏幕
        const size_t screen_pixels = ST7735S_WIDTH * ST7735S_HEIGHT;
        uint16_t* black_buffer = new uint16_t[screen_pixels];
        for (size_t i = 0; i < screen_pixels; i++) {
            black_buffer[i] = 0x0000; // 黑色RGB565
        }
        bool result = display_device_->write_buffer(black_buffer, screen_pixels);
        delete[] black_buffer;
        return result;
    }
    return false;
}

// 刷新屏幕
bool UIManager::refresh_screen() {
    if (!initialized_) {
        return false;
    }

    const size_t buffer_size = ST7735S_WIDTH * ST7735S_HEIGHT;
    display_device_->write_buffer(framebuffer_, buffer_size);

    statistics_.total_refreshes++;
    return true;
}

// 强制刷新
bool UIManager::force_refresh() {
    needs_full_refresh_ = true;
    return refresh_screen();
}

// 摇杆回调已移除，简化输入处理 工具函数
bool UIManager::is_page_valid(UIPage page) const {
    return static_cast<int>(page) >= 0 && static_cast<int>(page) <= 7;
}

void UIManager::log_debug(const std::string& message) {
    if (debug_enabled_) {
        auto* logger = USB_SerialLogs::get_global_instance();
        if (logger) {
            logger->debug(message, "UIManager");
        }
    }
}

void UIManager::log_error(const std::string& message) {
    auto* logger = USB_SerialLogs::get_global_instance();
    if (logger) {
        logger->error(message, "UIManager");
    }
}

// 其他必要的函数实现...
bool UIManager::set_display_device(ST7735S* display) {
    display_device_ = display;
    return true;
}

ST7735S* UIManager::get_display_device() const {
    return display_device_;
}

bool UIManager::set_input_manager(InputManager* input_manager) {
    input_manager_ = input_manager;
    return true;
}

bool UIManager::set_light_manager(LightManager* light_manager) {
    light_manager_ = light_manager;
    return true;
}

bool UIManager::set_config_manager(ConfigManager* config_manager) {
    config_manager_ = config_manager;
    return true;
}

std::vector<UIPage> UIManager::get_available_pages() {
    return {UIPage::MAIN, UIPage::STATUS, UIPage::SETTINGS, 
            UIPage::CALIBRATION, UIPage::DIAGNOSTICS, 
            UIPage::SENSITIVITY, UIPage::ABOUT};
}

// 统一的摇杆输入处理接口
bool UIManager::handle_joystick_input(ui::JoystickButton button) {
    // 更新活动时间
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    
    // 优先使用MenuInteractionSystem处理
    if (menu_system_) {
        switch (button) {
            case ui::JoystickButton::BUTTON_A:
                return menu_system_->handle_joystick_up();
            case ui::JoystickButton::BUTTON_B:
                return menu_system_->handle_joystick_down();
            case ui::JoystickButton::BUTTON_CONFIRM:
                return menu_system_->handle_joystick_confirm();
        }
    }
    
    // 回退到传统处理方式
    switch (button) {
        case ui::JoystickButton::BUTTON_A:
            return navigate_menu(true);
        case ui::JoystickButton::BUTTON_B:
            return navigate_menu(false);
        case ui::JoystickButton::BUTTON_CONFIRM:
            return confirm_selection();
    }
    
    return false;
}

bool UIManager::navigate_menu(bool up) {
    handle_navigation_input(up);
    page_needs_redraw_ = true;
    return true;
}

bool UIManager::navigate_menu_horizontal(bool right) {
    // 水平导航逻辑（用于某些页面的左右选择）
    // 目前大多数页面不需要水平导航，可以根据需要扩展
    page_needs_redraw_ = true;
    return true;
}

bool UIManager::confirm_selection() {
    handle_confirm_input();
    page_needs_redraw_ = true;
    return true;
}

bool UIManager::handle_back_navigation() {
    // 统一的返回导航处理
    if (nav_manager_ && nav_manager_->can_go_back()) {
        ui::UIPage target_page = nav_manager_->pop_page();
        return set_current_page(target_page);
    } else {
        // 默认返回主页面
        return set_current_page(UIPage::MAIN);
    }
}

// 动态光标渲染
void UIManager::render_cursor_indicator() {
    if (!graphics_engine_ || !page_template_) {
        return;
    }
    
    // 根据当前菜单索引渲染光标指示
    int menu_count = get_menu_item_count();
    if (menu_count > 0 && current_menu_index_ >= 0 && current_menu_index_ < menu_count) {
        // 使用PageTemplate的静态方法计算光标位置
        int16_t cursor_y = PageTemplate::get_line_y_position(current_menu_index_);
        
        // 绘制光标指示符 - 使用更醒目的样式
        // 方案1: 左侧箭头指示符
        graphics_engine_->draw_text(">", 2, cursor_y + 1, COLOR_PRIMARY, FontSize::MEDIUM);
        
        // 方案2: 可选的高亮背景框（注释掉，可根据需要启用）
        // graphics_engine_->draw_rect(line_rect.x, line_rect.y, line_rect.width, line_rect.height, COLOR_SELECTION_BG, false);
        
        // 方案3: 右侧状态指示器（注释掉，可根据需要启用）
        // graphics_engine_->draw_status_indicator(line_rect.x + line_rect.width - 8, cursor_y + 2, 4, COLOR_PRIMARY, true);
    }
}

int UIManager::get_current_menu_index() const {
    return current_menu_index_;
}

int UIManager::get_menu_item_count() const {
    // 如果使用页面模板系统，从模板获取菜单项数量
    if (use_page_template_ && page_template_) {
        return page_template_->get_menu_item_count();
    }
    
    // 回退到基于页面类型的静态菜单项数量
    switch (current_page_) {
        case UIPage::MAIN:
            return 7; // 主菜单: 状态、设置、灵敏度、触摸映射、按键映射、校准、关于
        case UIPage::SETTINGS:
            return 6; // 设置菜单: UART设置、灯光映射、引导绑定、诊断、重置、返回
        case UIPage::SENSITIVITY:
            return 4; // 灵敏度页面: 自动调节、手动调节、保存设置、返回
        case UIPage::TOUCH_MAPPING:
            return 3; // 触摸映射: 开始映射、清除映射、返回
        case UIPage::KEY_MAPPING:
            return 4; // 按键映射: 开始映射、清除所有映射、清除逻辑映射、返回
        case UIPage::UART_SETTINGS:
            return 4; // UART设置: Mai2Serial波特率、Mai2Light波特率、保存、返回
        case UIPage::GUIDED_BINDING:
            return 2; // 引导绑定: 开始绑定、返回
        case UIPage::LIGHT_MAPPING:
            return 4; // 灯光映射: 选择区域、保存映射、清除映射、返回
        case UIPage::CALIBRATION:
            return 2; // 校准页面: 开始校准、返回
        case UIPage::DIAGNOSTICS:
            return 3; // 诊断页面: 刷新数据、测试显示、返回
        case UIPage::ABOUT:
            return 1; // 关于页面: 返回
        case UIPage::ERROR:
            return 2; // 错误页面: 重试、返回
        case UIPage::STATUS:
        default:
            return 0; // 状态页面和其他页面通常没有菜单项
    }
}

bool UIManager::set_screen_timeout(uint16_t timeout_seconds) {
    static_config_.screen_timeout = timeout_seconds;
    return true;
}

bool UIManager::is_screen_off() const {
    return screen_off_;
}

bool UIManager::wake_screen() {
    if (screen_off_) {
        screen_off_ = false;
        last_activity_time_ = to_ms_since_boot(get_absolute_time());
        // 唤醒屏幕：恢复背光
        if (display_device_ && backlight_enabled_) {
            display_device_->set_backlight(true);
        }
        return true;
    }
    return false;
}

void UIManager::enable_debug_output(bool enabled) {
    debug_enabled_ = enabled;
}

void UIManager::handle_error_detection() {
    // 检查全局故障状态
    if (global_has_error_ && !has_error_) {
        has_error_ = true;
        current_error_ = global_error_;
        add_error_to_history(current_error_);
        set_current_page(UIPage::ERROR);
    }
}
 
std::string UIManager::error_type_to_string(ErrorType type) const {
    switch (type) {
        case ErrorType::NONE: return "None";
        case ErrorType::HARDWARE_INIT: return "Hardware Init";
        case ErrorType::DISPLAY_ERROR: return "Display Error";
        case ErrorType::INPUT_ERROR: return "Input Error";
        case ErrorType::LIGHT_ERROR: return "Light Error";
        case ErrorType::CONFIG_ERROR: return "Config Error";
        case ErrorType::COMMUNICATION_ERROR: return "Communication Error";
        case ErrorType::MEMORY_ERROR: return "Memory Error";
        case ErrorType::SENSOR_ERROR: return "Sensor Error";
        case ErrorType::CALIBRATION_ERROR: return "Calibration Error";
        case ErrorType::UNKNOWN_ERROR: return "Unknown Error";
        default: return "Unknown";
    }
}

void UIManager::add_error_to_history(const ErrorInfo& error) {
    error_history_.push_back(error);
    
    // 限制历史记录数量，避免内存溢出
    const size_t MAX_ERROR_HISTORY = 50;
    if (error_history_.size() > MAX_ERROR_HISTORY) {
        error_history_.erase(error_history_.begin());
    }
}
