#include "ui_manager.h"
#include "../../service/input_manager/input_manager.h"
#include "../../service/light_manager/light_manager.h"
#include "../../service/config_manager/config_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "../../protocol/st7735s/st7735s.h"
#include "font_system.h"
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
    , selected_device_index_(-1)
    , selected_channel_(0)
    , auto_adjust_active_(false)
    , touch_mapping_active_(false)
    , mapping_step_(0)
    , mapping_device_addr_(0)
    , mapping_channel_(0)
    , key_mapping_active_(false)
    , selected_key_index_(-1)
    , selected_gpio_(-1)
    , guided_binding_active_(false)
    , binding_step_(0)
    , light_mapping_active_(false)
    , selected_light_region_("")
    , current_mai2serial_baudrate_(115200)
    , current_mai2light_baudrate_(115200)
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
    
    // 初始化时不预创建页面，而是在需要时按需创建
    // 已移除LVGL依赖，不再需要页面对象
    
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
                    // 根据按钮类型处理输入
                    switch (i) {
                        case 0: // A按钮 - 向上导航
                            handle_navigation_input(true);
                            break;
                        case 1: // B按钮 - 向下导航
                            handle_navigation_input(false);
                            break;
                        case 2: // 确认按钮
                            handle_confirm_input();
                            break;
                    }
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
    uint64_t current_time = time_us_64();
    // 30fps = 33.33ms间隔
    if (current_time - last_refresh_time_ >= 33333) {
        // 检查是否需要重绘页面
        if (page_needs_redraw_) {
            draw_current_page();
            page_needs_redraw_ = false;
            framebuffer_dirty_ = true;
        }
        
        // 刷新显示
        refresh_display();
        
        last_refresh_time_ = current_time;
    }
}

// 绘制当前页面
void UIManager::draw_current_page() {
    if (!graphics_engine_) {
        return;
    }
    
    // 清空屏幕
    graphics_engine_->clear(COLOR_BLACK);
    
    // 根据当前页面绘制内容
    switch (current_page_) {
        case UIPage::MAIN:
            draw_main_page();
            break;
        case UIPage::STATUS:
            draw_status_page();
            break;
        case UIPage::SENSITIVITY:
            draw_sensitivity_page();
            break;
        case UIPage::TOUCH_MAPPING:
            draw_touch_mapping_page();
            break;
        case UIPage::KEY_MAPPING:
            draw_key_mapping_page();
            break;
        case UIPage::GUIDED_BINDING:
            draw_guided_binding_page();
            break;
        case UIPage::SETTINGS:
            draw_settings_page();
            break;
        case UIPage::UART_SETTINGS:
            draw_uart_settings_page();
            break;
        case UIPage::CALIBRATION:
            draw_calibration_page();
            break;
        case UIPage::DIAGNOSTICS:
            draw_diagnostics_page();
            break;
        case UIPage::LIGHT_MAPPING:
            draw_light_mapping_page();
            break;
        case UIPage::ABOUT:
            draw_about_page();
            break;
        case UIPage::ERROR:
            draw_error_page();
            break;
        default:
            break;
    }
// 输入处理已移至handle_input()函数
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
    
    // 标记需要重绘
    page_needs_redraw_ = true;
    
    // 重置页面数据
    reset_page_data();
    
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
    page_data_.selected_index = 0;
    
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
        default:
            break;
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

// 页面绘制函数实现
void UIManager::draw_main_page() {
    if (!graphics_engine_) return;
    
    // 绘制标题 - 使用16x16大字体
    graphics_engine_->draw_text(page_data_.title.c_str(), 10, 2, COLOR_WHITE, FontSize::LARGE);
    
    // 绘制菜单项 - 使用12x12中字体以提高清晰度
    for (size_t i = 0; i < page_data_.menu_items.size(); i++) {
        uint16_t y = 22 + i * 8;
        if (y > 70) break; // 防止超出屏幕范围
        uint16_t color = (i == page_data_.selected_index) ? COLOR_PRIMARY : COLOR_TEXT_WHITE;
        graphics_engine_->draw_text(page_data_.menu_items[i].c_str(), 12, y, color, FontSize::MEDIUM);
        
        // 绘制选中指示器 - 使用更大的字体
        if (i == page_data_.selected_index) {
            graphics_engine_->draw_text(">", 2, y, COLOR_PRIMARY, FontSize::MEDIUM);
        }
    }
}

void UIManager::draw_status_page() {
    if (!graphics_engine_) return;
    
    // 绘制标题 - 使用16x16大字体
    graphics_engine_->draw_text(page_data_.title.c_str(), 10, 2, COLOR_WHITE, FontSize::LARGE);
    
    // 绘制状态信息 - 使用12x12中字体以提高清晰度
    uint16_t y = 22;
    for (const auto& status : page_data_.status_items) {
        if (y > 70) break; // 防止超出屏幕范围
        graphics_engine_->draw_text(status.c_str(), 2, y, COLOR_SUCCESS, FontSize::MEDIUM);
        y += 8;
    }
}

void UIManager::draw_settings_page() {
    if (!graphics_engine_) return;
    
    // 绘制标题
    graphics_engine_->draw_text(page_data_.title.c_str(), 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    
    // 绘制设置菜单
    for (size_t i = 0; i < page_data_.menu_items.size(); i++) {
        uint16_t y = 25 + i * 10;
        uint16_t color = (i == page_data_.selected_index) ? COLOR_PRIMARY : COLOR_TEXT_WHITE;
        graphics_engine_->draw_text(page_data_.menu_items[i].c_str(), 10, y, color, FontSize::SMALL);
        
        if (i == page_data_.selected_index) {
            graphics_engine_->draw_text(">", 5, y, COLOR_PRIMARY, FontSize::SMALL);
        }
    }
}

void UIManager::draw_sensitivity_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("灵敏度设置", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("设备: 1/8", 10, 25, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("通道: 0", 10, 35, COLOR_TEXT_WHITE, FontSize::SMALL);
    
    // 绘制滑块
    graphics_engine_->draw_slider(Rect{10, 45, 140, 8}, page_data_.progress_value, COLOR_PRIMARY, COLOR_BG_CARD);
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "值: %d", page_data_.progress_value);
    graphics_engine_->draw_text(value_str, 10, 58, COLOR_TEXT_WHITE, FontSize::SMALL);
}

void UIManager::draw_touch_mapping_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("触摸映射", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("请触摸要映射的区域", 80, 30, COLOR_TEXT_WHITE, FontSize::SMALL);
    
    if (!page_data_.content.empty()) {
        graphics_engine_->draw_text(page_data_.content.c_str(), 80, 45, COLOR_PRIMARY, FontSize::SMALL);
    }
}

void UIManager::draw_key_mapping_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("按键映射", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("GPIO: 未选择", 10, 25, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("HID键: 未设置", 10, 35, COLOR_TEXT_WHITE, FontSize::SMALL);
}

void UIManager::draw_guided_binding_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("引导绑定", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    
    // 绘制进度条
    graphics_engine_->draw_progress_bar(Rect{10, 25, 140, 8}, page_data_.progress_value, COLOR_SUCCESS, COLOR_BG_CARD);
    
    char progress_str[32];
    snprintf(progress_str, sizeof(progress_str), "进度: %d%%", page_data_.progress_value);
    graphics_engine_->draw_text(progress_str, 80, 40, COLOR_TEXT_WHITE, FontSize::SMALL);
    
    if (!page_data_.content.empty()) {
        graphics_engine_->draw_text(page_data_.content.c_str(), 80, 55, COLOR_PRIMARY, FontSize::SMALL);
    }
}

void UIManager::draw_uart_settings_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("UART设置", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("Mai2Serial: 115200", 10, 25, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("Mai2Light: 115200", 10, 35, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("状态: 正常", 10, 45, COLOR_SUCCESS, FontSize::SMALL);
}

void UIManager::draw_calibration_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("校准", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("请按照提示进行校准", 80, 30, COLOR_TEXT_WHITE, FontSize::SMALL);
    
    // 绘制校准进度
    graphics_engine_->draw_progress_bar(Rect{10, 45, 140, 8}, page_data_.progress_value, COLOR_WARNING, COLOR_BG_CARD);
}

void UIManager::draw_diagnostics_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("诊断", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("系统状态: 正常", 10, 25, COLOR_SUCCESS, FontSize::SMALL);
    graphics_engine_->draw_text("内存使用: 45%", 10, 35, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("CPU使用: 23%", 10, 45, COLOR_TEXT_WHITE, FontSize::SMALL);
}

void UIManager::draw_light_mapping_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("灯光映射", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("区域: 未选择", 10, 25, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("LED: 0个", 10, 35, COLOR_TEXT_WHITE, FontSize::SMALL);
}

void UIManager::draw_about_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("关于", 80, 10, COLOR_WHITE, FontSize::MEDIUM);
    graphics_engine_->draw_text("MaiMai控制器 V3.0", 80, 25, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("固件版本: 1.0.0", 80, 35, COLOR_TEXT_WHITE, FontSize::SMALL);
    graphics_engine_->draw_text("构建日期: 2024-01-20", 80, 45, COLOR_TEXT_WHITE, FontSize::SMALL);
}

void UIManager::draw_error_page() {
    if (!graphics_engine_) return;
    
    graphics_engine_->draw_text("错误", 80, 10, COLOR_ERROR, FontSize::MEDIUM);
    
    if (!page_data_.content.empty()) {
        graphics_engine_->draw_text(page_data_.content.c_str(), 80, 30, COLOR_ERROR, FontSize::SMALL);
    } else {
        graphics_engine_->draw_text("发生未知错误", 80, 30, COLOR_ERROR, FontSize::SMALL);
    }
    
    graphics_engine_->draw_text("按任意键返回", 80, 50, COLOR_TEXT_WHITE, FontSize::SMALL);
}

// 处理导航输入
void UIManager::handle_navigation_input(bool up) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 防抖处理
    if (current_time - last_navigation_time_ < 200) {
        return;
    }
    last_navigation_time_ = current_time;
    
    // 根据当前页面处理导航
    switch (current_page_) {
        case UIPage::MAIN:
            // 主页面导航
            if (up) {
                current_menu_index_ = (current_menu_index_ > 0) ? current_menu_index_ - 1 : 6;
            } else {
                current_menu_index_ = (current_menu_index_ < 6) ? current_menu_index_ + 1 : 0;
            }
            break;
            
        case UIPage::SENSITIVITY:
            // 灵敏度页面导航
            {
                int32_t current_value = page_data_.progress_value;
                int32_t new_value = up ? current_value + 1 : current_value - 1;
                new_value = (new_value < 0) ? 0 : (new_value > 63) ? 63 : new_value;
                
                page_data_.progress_value = new_value;
                
                // 应用灵敏度变化到输入管理器
                if (input_manager_ && selected_device_index_ >= 0) {
                    input_manager_->setSensitivity(selected_device_index_, selected_channel_, new_value);
                }
                
                page_needs_redraw_ = true;
            }
            break;
            
        case UIPage::STATUS:
        case UIPage::SETTINGS:
        case UIPage::CALIBRATION:
        case UIPage::DIAGNOSTICS:
        case UIPage::TOUCH_MAPPING:
        case UIPage::KEY_MAPPING:
        case UIPage::GUIDED_BINDING:
        case UIPage::LIGHT_MAPPING:
        case UIPage::UART_SETTINGS:
        case UIPage::ERROR:
        case UIPage::ABOUT:
        default:
            // 其他页面暂时不处理导航，或者添加通用导航逻辑
            break;
    }
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
            
        case UIPage::SENSITIVITY:
            // 自动选择灵敏度点
            auto_select_sensitivity_point();
            break;
            
        default:
            // 其他页面返回主页面
            set_current_page(UIPage::MAIN);
            break;
    }
}

// 自动选择需要修改灵敏度的位点
bool UIManager::auto_select_sensitivity_point() {
    if (!input_manager_) {
        log_error("InputManager not available");
        return false;
    }
    
    int device_index;
    uint8_t channel;
    
    // 检测当前被触摸的通道
    if (detect_touched_channel(device_index, channel)) {
        selected_device_index_ = device_index;
        selected_channel_ = channel;
        
        // 更新显示
        update_sensitivity_display();
        
        // 获取设备状态用于日志
        InputManager::TouchDeviceStatus device_status[8];
        int device_count;
        input_manager_->get_all_device_status(device_status, device_count);
        
        if (device_index < device_count) {
            log_debug("Auto-selected device: " + device_status[device_index].device_name + ", channel: " + std::to_string(channel));
        }
        return true;
    }
    
    log_debug("No touched channel detected");
    return false;
}

// 检测被触摸的通道
bool UIManager::detect_touched_channel(int& device_index, uint8_t& channel) {
    if (!input_manager_) {
        return false;
    }
    
    // 获取所有设备状态
    InputManager::TouchDeviceStatus device_status[8];
    int device_count;
    input_manager_->get_all_device_status(device_status, device_count);
    
    for (int dev_idx = 0; dev_idx < device_count; dev_idx++) {
        uint16_t touch_states = device_status[dev_idx].touch_states;
        
        for (uint8_t ch = 0; ch < 12; ch++) {
            // 检查通道是否被触摸
            if (touch_states & (1 << ch)) {
                device_index = dev_idx;
                channel = ch;
                return true;
            }
        }
    }
    
    return false;
}

// 获取可用设备数量
int UIManager::get_available_device_count() {
    if (!input_manager_) {
        return 0;
    }
    
    InputManager::TouchDeviceStatus device_status[8];
    int device_count;
    input_manager_->get_all_device_status(device_status, device_count);
    
    return device_count;
}

// 触摸映射页面功能实现
bool UIManager::show_touch_mapping_page() {
    return set_current_page(UIPage::TOUCH_MAPPING);
}

bool UIManager::start_touch_mapping_mode() {
    if (!input_manager_) {
        log_error("InputManager not available");
        return false;
    }
    
    touch_mapping_active_ = true;
    mapping_step_ = 0;
    mapping_device_addr_ = 0;
    mapping_channel_ = 0;
    
    log_debug("Touch mapping mode started");
    return true;
}

bool UIManager::cancel_touch_mapping() {
    touch_mapping_active_ = false;
    mapping_step_ = 0;
    
    // TODO: 更新静态framebuffer显示映射取消状态
    page_needs_redraw_ = true;
    
    log_debug("Touch mapping cancelled");
    return true;
}

bool UIManager::show_mapping_selection_ui() {
    // TODO: 使用静态framebuffer绘制映射选择界面
    page_needs_redraw_ = true;
    return true;
}

bool UIManager::update_mapping_display() {
    // TODO: 使用静态framebuffer更新映射显示
    page_needs_redraw_ = true;
    return true;
}

// 按键映射页面功能实现
bool UIManager::show_key_mapping_page() {
    return set_current_page(UIPage::KEY_MAPPING);
}

bool UIManager::start_key_mapping_mode() {
    if (!input_manager_) {
        log_error("InputManager not available");
        return false;
    }
    
    key_mapping_active_ = true;
    selected_key_index_ = -1;
    
    // 显示当前键盘映射状态
    update_key_mapping_display();
    
    log_debug("Key mapping mode started");
    return true;
}

bool UIManager::show_hid_key_selection() {
    // TODO: 移除LVGL依赖，改用静态framebuffer渲染
    // TODO: 使用静态framebuffer显示HID按键选项列表
    // 常用HID按键选项: A, S, D, F, J, K, L, Space, Enter, F1-F12, Clear
    
    page_needs_redraw_ = true;
    return true;
}

bool UIManager::update_key_mapping_display() {
    // TODO: 使用静态framebuffer更新按键映射显示
    page_needs_redraw_ = true;
    return true;
}

const char* UIManager::getKeyName(HID_KeyCode key) {
    switch (key) {
        case HID_KeyCode::KEY_A: return "A";
        case HID_KeyCode::KEY_S: return "S";
        case HID_KeyCode::KEY_D: return "D";
        case HID_KeyCode::KEY_F: return "F";
        case HID_KeyCode::KEY_J: return "J";
        case HID_KeyCode::KEY_K: return "K";
        case HID_KeyCode::KEY_L: return "L";
        case HID_KeyCode::KEY_SPACE: return "Space";
        case HID_KeyCode::KEY_ENTER: return "Enter";
        case HID_KeyCode::KEY_F1: return "F1";
        case HID_KeyCode::KEY_F2: return "F2";
        case HID_KeyCode::KEY_F3: return "F3";
        case HID_KeyCode::KEY_F4: return "F4";
        case HID_KeyCode::KEY_F5: return "F5";
        case HID_KeyCode::KEY_F6: return "F6";
        case HID_KeyCode::KEY_F7: return "F7";
        case HID_KeyCode::KEY_F8: return "F8";
        case HID_KeyCode::KEY_F9: return "F9";
        case HID_KeyCode::KEY_F10: return "F10";
        case HID_KeyCode::KEY_F11: return "F11";
        case HID_KeyCode::KEY_F12: return "F12";
        default: return "Unknown";
    }
}

HID_KeyCode UIManager::getKeyCodeFromName(const char* name) {
    if (!name || strlen(name) == 0) {
        return HID_KeyCode::KEY_NONE;
    }
    
    // 单字符处理：A-Z (大写字母)
    if (strlen(name) == 1) {
        char c = name[0];
        
        // A-Z (ASCII 65-90)
        if (c >= 'A' && c <= 'Z') {
            return static_cast<HID_KeyCode>(static_cast<int>(HID_KeyCode::KEY_A) + (c - 'A'));
        }
        
        // a-z (ASCII 97-122) -> 转换为大写
        if (c >= 'a' && c <= 'z') {
            return static_cast<HID_KeyCode>(static_cast<int>(HID_KeyCode::KEY_A) + (c - 'a'));
        }
        
        // 0-9 (ASCII 48-57)
        if (c >= '0' && c <= '9') {
            return static_cast<HID_KeyCode>(static_cast<int>(HID_KeyCode::KEY_0) + (c - '0'));
        }
    }
    
    // 特殊按键处理
    switch (name[0]) {
        case 'S':
            if (strcmp(name, "Space") == 0) return HID_KeyCode::KEY_SPACE;
            if (strcmp(name, "Shift") == 0) return HID_KeyCode::KEY_LEFT_SHIFT;
            break;
            
        case 'E':
            if (strcmp(name, "Enter") == 0) return HID_KeyCode::KEY_ENTER;
            if (strcmp(name, "Escape") == 0) return HID_KeyCode::KEY_ESCAPE;
            break;
            
        case 'T':
            if (strcmp(name, "Tab") == 0) return HID_KeyCode::KEY_TAB;
            break;
            
        case 'B':
            if (strcmp(name, "Backspace") == 0) return HID_KeyCode::KEY_BACKSPACE;
            break;
            
        case 'C':
            if (strcmp(name, "Clear") == 0) return HID_KeyCode::KEY_NONE;
            if (strcmp(name, "Ctrl") == 0) return HID_KeyCode::KEY_LEFT_CTRL;
            break;
            
        case 'A':
            if (strcmp(name, "Alt") == 0) return HID_KeyCode::KEY_LEFT_ALT;
            break;
            
        case 'F':
            // 功能键 F1-F12
            if (strlen(name) >= 2 && name[1] >= '1' && name[1] <= '9') {
                if (strcmp(name, "F1") == 0) return HID_KeyCode::KEY_F1;
                if (strcmp(name, "F2") == 0) return HID_KeyCode::KEY_F2;
                if (strcmp(name, "F3") == 0) return HID_KeyCode::KEY_F3;
                if (strcmp(name, "F4") == 0) return HID_KeyCode::KEY_F4;
                if (strcmp(name, "F5") == 0) return HID_KeyCode::KEY_F5;
                if (strcmp(name, "F6") == 0) return HID_KeyCode::KEY_F6;
                if (strcmp(name, "F7") == 0) return HID_KeyCode::KEY_F7;
                if (strcmp(name, "F8") == 0) return HID_KeyCode::KEY_F8;
                if (strcmp(name, "F9") == 0) return HID_KeyCode::KEY_F9;
            }
            if (strcmp(name, "F10") == 0) return HID_KeyCode::KEY_F10;
            if (strcmp(name, "F11") == 0) return HID_KeyCode::KEY_F11;
            if (strcmp(name, "F12") == 0) return HID_KeyCode::KEY_F12;
            break;
            
        case 'U':
            if (strcmp(name, "Up") == 0) return HID_KeyCode::KEY_UP_ARROW;
            break;
            
        case 'D':
            if (strcmp(name, "Down") == 0) return HID_KeyCode::KEY_DOWN_ARROW;
            break;
            
        case 'L':
            if (strcmp(name, "Left") == 0) return HID_KeyCode::KEY_LEFT_ARROW;
            break;
            
        case 'R':
            if (strcmp(name, "Right") == 0) return HID_KeyCode::KEY_RIGHT_ARROW;
            break;
            
        default:
            break;
    }
    
    return HID_KeyCode::KEY_NONE;
}

bool UIManager::handle_hid_key_selection() {
    if (!input_manager_ || !key_mapping_active_) {
        return false;
    }
    
    // TODO: 切换到静态framebuffer渲染，移除LVGL依赖
    // 需要实现基于静态framebuffer的按键选择逻辑
    
    // 获取选中的按键名称 - 需要重新实现
    const char* key_name = nullptr; // TODO: 从静态UI状态获取选中的按键名称
    if (!key_name) {
        return false;
    }
    
    // 检测当前触摸的通道
    int device_index = -1;
    uint8_t channel = 0;
    if (!detect_touched_channel(device_index, channel)) {
        log_debug("No channel touched for key mapping");
        return false;
    }
    
    // 获取设备地址
    InputManager::TouchDeviceStatus devices[8];
    int device_count = 0;
    input_manager_->get_all_device_status(devices, device_count);
    
    if (device_index >= device_count) {
        return false;
    }
    
    uint16_t device_addr = devices[device_index].device.device_addr;
    
    // 转换按键名称为按键代码
    HID_KeyCode key_code = getKeyCodeFromName(key_name);
    
    // 设置触摸键盘映射
    input_manager_->setTouchKeyboardMapping(device_addr, channel, key_code);
    log_debug("Key mapping set: Dev" + std::to_string(device_addr) + " Ch" + std::to_string(channel) + " -> " + key_name);
    
    // 更新显示
    update_key_mapping_display();
    return true;
}

bool UIManager::clear_all_key_mappings() {
    if (!input_manager_) {
        return false;
    }
    
    // 获取所有设备状态
    InputManager::TouchDeviceStatus devices[8];
    int device_count = 0;
    input_manager_->get_all_device_status(devices, device_count);
    
    // 清除所有设备的键盘映射
    for (int i = 0; i < device_count; i++) {
        for (uint8_t ch = 0; ch < 12; ch++) {
            input_manager_->setTouchKeyboardMapping(devices[i].device.device_addr, ch, HID_KeyCode::KEY_NONE);
        }
    }
    
    // 更新显示
    update_key_mapping_display();
    
    log_debug("All key mappings cleared");
    return true;
}

bool UIManager::clear_all_logical_key_mappings() {
    if (!input_manager_) {
        return false;
    }
    
    // 清除所有逻辑按键映射
    input_manager_->clearAllLogicalKeyMappings();
    
    // TODO: 切换到静态framebuffer渲染，移除LVGL依赖
    // 重置下拉栏选择和GPIO标签显示需要重新实现
    
    page_needs_redraw_ = true;
    
    log_debug("All logical key mappings cleared");
    return true;
}

// 处理触摸映射选择
bool UIManager::handle_touch_mapping_selection(int selection_index) {
    if (!input_manager_) {
        return false;
    }
    
    // TODO: 切换到静态framebuffer渲染，移除LVGL依赖
    // 触摸映射选择逻辑需要重新实现，使用按键输入而非LVGL事件
    
    // 检查是否选择了HID映射选项
    if (selection_index == 0) { // Map to HID Touch
        // 启动HID触摸映射模式
        if (input_manager_->getWorkMode() == InputWorkMode::SERIAL_MODE && 
            input_manager_->isAutoSerialBindingComplete()) {
            
            touch_mapping_active_ = true;
            mapping_step_ = 1; // 设置为HID映射步骤
            page_needs_redraw_ = true;
            
            return true;
        }
    } else {
        // 处理Serial区域映射
        // 这里可以添加Serial区域映射的处理逻辑
        page_needs_redraw_ = true;
    }
    
    return true;
}

// 引导式绑区页面功能实现
bool UIManager::show_guided_binding_page() {
    return set_current_page(UIPage::GUIDED_BINDING);
}

bool UIManager::start_guided_binding() {
    if (!input_manager_) {
        log_error("InputManager not available");
        return false;
    }
    
    // 检查是否为Serial模式
    if (input_manager_->getWorkMode() != InputWorkMode::SERIAL_MODE) {
        log_error("Guided binding only available in Serial mode");
        return false;
    }
    
    guided_binding_active_ = true;
    binding_step_ = 0;
    
    // 开始自动绑区
    if (!input_manager_->startAutoSerialBinding()) {
        log_error("Failed to start auto serial binding");
        guided_binding_active_ = false;
        return false;
    }
    
    log_debug("Auto guided binding started");
    return true;
}

bool UIManager::update_guided_binding_progress(uint8_t step, const std::string& current_area) {
    binding_step_ = step;
    
    // TODO: 切换到静态framebuffer渲染，移除LVGL依赖
    // 绑定进度显示需要重新实现
    
    page_needs_redraw_ = true;
    
    return true;
}

// 更新灵敏度显示
void UIManager::update_sensitivity_display() {
    if (selected_device_index_ < 0) {
        return;
    }
    
    // 获取设备状态
    InputManager::TouchDeviceStatus device_status[8];
    int device_count;
    input_manager_->get_all_device_status(device_status, device_count);
    
    if (selected_device_index_ >= device_count) {
        return;
    }
    
    // TODO: 切换到静态framebuffer渲染，移除LVGL依赖
    // 灵敏度显示需要重新实现
    
    page_needs_redraw_ = true;
}

// All LVGL create_xxx_page functions and event callbacks removed
// Using static framebuffer rendering instead
// TODO: 移除LVGL事件回调函数，改用静态framebuffer渲染和按键输入处理

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
        
        // 息屏状态下不进行页面更新和LVGL渲染
        return;
    }
    
    // 屏幕开启状态下的正常渲染
    // 使用30fps刷新任务进行页面渲染
    refresh_task_30fps();
    // 限制页面更新频率，根据配置的刷新率
    // if (current_time - last_refresh_time_ >= static_config_.refresh_rate_ms) {
    //     // 更新当前页面
    //     switch (current_page_) {
    //         case UIPage::STATUS:
    //             update_status_page();
    //             break;
    //         case UIPage::SENSITIVITY:
    //             update_sensitivity_page();
    //             break;
    //         case UIPage::TOUCH_MAPPING:
    //             update_touch_mapping_page();
    //             break;
    //         case UIPage::KEY_MAPPING:
    //             update_key_mapping_page();
    //             break;
    //         case UIPage::GUIDED_BINDING:
    //             update_guided_binding_page();
    //             break;
    //         case UIPage::LIGHT_MAPPING:
    //             update_light_mapping_page();
    //             break;
    //         default:
    //             break;
    //     }
    //     last_refresh_time_ = current_time;
    // }
}

// 更新页面
void UIManager::update_status_page() {
    // 获取当前时间用于计算运行时间
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t uptime_seconds = current_time / 1000;
    
    // 获取输入管理器的实时数据
    uint32_t touch_poll_rate = 0;
    uint32_t key_poll_rate = 0;
    uint32_t active_touches = 0;
    uint32_t active_keys = 0;
    
    if (input_manager_) {
        // 获取实际的轮询频率
        InputManager::TouchDeviceStatus devices[8];
        int device_count = 0;
        
        input_manager_->get_all_device_status(devices, device_count);
        
        if (device_count > 0) {
            // 获取第一个设备的触摸采样率
            touch_poll_rate = input_manager_->getTouchSampleRate(devices[0].device.device_addr);
        }
        
        // 获取HID键盘回报速率
        key_poll_rate = input_manager_->getHIDReportRate();
        
        // 计算当前活跃的触摸点数量
        for (int i = 0; i < device_count; i++) {
            if (devices[i].is_connected) {
                uint16_t touch_states = devices[i].touch_states;
                for (int ch = 0; ch < 12; ch++) {
                    if (touch_states & (1 << ch)) {
                        active_touches++;
                    }
                }
            }
        }
        
        // 获取物理键盘映射数量作为活跃按键的参考
        const auto& physical_keyboards = input_manager_->getPhysicalKeyboards();
        active_keys = physical_keyboards.size(); // 简化显示已配置的按键数量
    }
    
    // 更新状态项数据
    char buffer[32];
    
    // 更新轮询率
    snprintf(buffer, sizeof(buffer), "Touch Poll: %luHz", touch_poll_rate);
    page_data_.status_items[4] = buffer;
    
    snprintf(buffer, sizeof(buffer), "Key Poll: %luHz", key_poll_rate);
    page_data_.status_items[5] = buffer;
    
    // 更新活跃数量
    snprintf(buffer, sizeof(buffer), "Touch Active: %lu", active_touches);
    page_data_.status_items[6] = buffer;
    
    snprintf(buffer, sizeof(buffer), "Key Config: %lu", active_keys);
    page_data_.status_items[7] = buffer;
    
    // 更新运行时间
    if (uptime_seconds < 60) {
        snprintf(buffer, sizeof(buffer), "Uptime: %lus", uptime_seconds);
    } else if (uptime_seconds < 3600) {
        snprintf(buffer, sizeof(buffer), "Uptime: %lum%lus", uptime_seconds / 60, uptime_seconds % 60);
    } else {
        uint32_t hours = uptime_seconds / 3600;
        uint32_t minutes = (uptime_seconds % 3600) / 60;
        snprintf(buffer, sizeof(buffer), "Uptime: %luh%lum", hours, minutes);
    }
    page_data_.status_items[8] = buffer;
    
    // 标记页面需要重绘
    page_needs_redraw_ = true;
}

void UIManager::update_sensitivity_page() {
    if (!input_manager_) return;
    
    // 检查自动调整状态
    if (auto_adjust_active_) {
        // 自动调整期间不处理其他输入，需要手动停止
        return;
    }
    
    // 检测触摸输入以自动选择通道
    int device_index;
    uint8_t channel;
    if (detect_touched_channel(device_index, channel)) {
        if (selected_device_index_ != device_index || selected_channel_ != channel) {
            selected_device_index_ = device_index;
            selected_channel_ = channel;
            update_sensitivity_display();
        }
    }
    
    // TODO: 移除LVGL依赖，改用静态framebuffer渲染
    // 更新当前选中通道的灵敏度显示
    if (selected_device_index_ >= 0) {
        InputManager::TouchDeviceStatus device_status[8];
        int device_count;
        input_manager_->get_all_device_status(device_status, device_count);
        
        if (selected_device_index_ < device_count) {
            // TODO: 使用静态framebuffer更新灵敏度显示
            page_needs_redraw_ = true;
        }
    }
}

void UIManager::update_touch_mapping_page() {
    // TODO: 移除LVGL依赖，改用静态framebuffer渲染
    
    if (touch_mapping_active_) {
        switch (mapping_step_) {
            case 0:
                // TODO: 使用静态framebuffer显示"Touch a point to map"
                
                // 检测触摸输入
                int device_index;
                uint8_t channel;
                if (detect_touched_channel(device_index, channel)) {
                    mapping_device_addr_ = device_index;
                    mapping_channel_ = channel;
                    mapping_step_ = 1;
                    
                    // 显示映射选择UI
                    show_mapping_selection_ui();
                }
                break;
            case 1:
                // TODO: 使用静态framebuffer显示"Select target area"
                break;
            case 2:
                // TODO: 使用静态framebuffer显示"Mapping complete"
                break;
        }
        
        // TODO: 处理HID映射模式的触摸检测，移除LVGL依赖
        if (mapping_step_ == 0 && input_manager_) {
            // TODO: 实现基于硬件的触摸检测，不依赖LVGL
            // 检查当前是否有设备和通道被选中进行HID映射
            if (mapping_device_addr_ >= 0 && mapping_channel_ >= 0) {
                // TODO: 获取实际触摸坐标，不使用LVGL
                // 设置HID坐标映射
                // input_manager_->setHIDMapping(mapping_device_addr_, mapping_channel_, touch_x, touch_y);
                mapping_step_ = 2;
            }
        }
        
        page_needs_redraw_ = true;
    }
}

void UIManager::update_key_mapping_page() {
    if (!key_mapping_active_) {
        return;
    }
    
    // 更新HID按键选择列表
    show_hid_key_selection();
    
    // 更新当前映射显示
    update_key_mapping_display();
}

void UIManager::update_guided_binding_page() {
    // TODO: 移除LVGL依赖，改用静态framebuffer渲染
    
    if (guided_binding_active_ && input_manager_) {
        // 检查自动绑区是否完成
        if (input_manager_->isAutoSerialBindingComplete()) {
            // TODO: 使用静态framebuffer显示完成状态
            
            // 可以添加确认按钮或等待用户输入
            // 这里简化处理，自动确认
            static uint32_t complete_time = 0;
            if (complete_time == 0) {
                complete_time = to_ms_since_boot(get_absolute_time());
            }
            
            // 3秒后自动确认
            if (to_ms_since_boot(get_absolute_time()) - complete_time > 3000) {
                guided_binding_active_ = false;
                complete_time = 0;
                set_current_page(UIPage::MAIN);
            }
        } else {
            // TODO: 使用静态framebuffer更新进度显示
            std::string step_text = "Auto binding in progress... " + std::to_string(binding_step_) + "/34";
            // TODO: 使用静态framebuffer显示进度文本
        }
        
        page_needs_redraw_ = true;
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

// 获取统计信息
bool UIManager::get_statistics(UIStatistics& stats) {
    stats = statistics_;
    return true;
}

// 重置统计信息
void UIManager::reset_statistics() {
    statistics_ = UIStatistics();
    statistics_.last_reset_time = to_ms_since_boot(get_absolute_time());
}

// 设置回调函数
void UIManager::set_event_callback(UIEventCallback callback) {
    event_callback_ = callback;
}

void UIManager::set_page_callback(UIPageCallback callback) {
    page_callback_ = callback;
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

bool UIManager::navigate_menu(bool up) {
    handle_navigation_input(up);
    return true;
}

bool UIManager::trigger_event(UIEvent event, const std::string& element_id, int32_t value) {
    if (event_callback_) {
        event_callback_(event, element_id, value);
    }
    return true;
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

// 页面显示功能的简化实现
bool UIManager::show_status_info() {
    return set_current_page(UIPage::STATUS);
}

bool UIManager::show_joystick_status() {
    return set_current_page(UIPage::STATUS);
}

bool UIManager::show_light_status() {
    return set_current_page(UIPage::STATUS);
}

bool UIManager::show_system_info() {
    return set_current_page(UIPage::STATUS);
}

bool UIManager::show_calibration_page() {
    return set_current_page(UIPage::CALIBRATION);
}

bool UIManager::update_calibration_progress(uint8_t progress) {
    statistics_.calibration_progress = progress;
    return true;
}

bool UIManager::show_diagnostics_page() {
    return set_current_page(UIPage::DIAGNOSTICS);
}

bool UIManager::update_diagnostics_data() {
    return true;
}

bool UIManager::show_sensitivity_page() {
    return set_current_page(UIPage::SENSITIVITY);
}



// 显示绑定状态
bool UIManager::show_binding_status(const std::string& message, bool is_success) {
    if (!initialized_) {
        return false;
    }
    
    // TODO: 实现静态framebuffer绑定状态显示
    // 移除LVGL依赖，转为静态framebuffer渲染
    
    // 记录活动时间
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    page_needs_redraw_ = true;
    
    return true;
}

// 更新绑定进度
bool UIManager::update_binding_progress(uint8_t progress, const std::string& current_step) {
    if (!initialized_) {
        return false;
    }
    
    // TODO: 实现静态framebuffer绑定进度显示
    // 移除LVGL依赖，转为静态framebuffer渲染
    
    // 记录活动时间
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    page_needs_redraw_ = true;
    
    return true;
}

// 清除绑定状态显示
bool UIManager::clear_binding_status() {
    if (!initialized_) {
        return false;
    }
    
    // TODO: 实现静态framebuffer绑定状态清除
    // 移除LVGL依赖，转为静态framebuffer渲染
    
    // 触发页面重绘
    page_needs_redraw_ = true;
    
    return true;
}

void UIManager::enable_debug_output(bool enabled) {
    debug_enabled_ = enabled;
}

std::string UIManager::get_debug_info() {
    return "UIManager Debug Info";
}

bool UIManager::test_display() {
    return clear_screen();
}

bool UIManager::test_joystick() {
    return true;
}

void UIManager::mark_page_dirty(UIPage page) {
    // 标记页面需要刷新
}

void UIManager::update_main_page() {
    // 更新主页面
}

void UIManager::update_settings_page() {
    // 更新设置页面
}

void UIManager::update_uart_settings_page() {
    // UART设置页面更新逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
}

void UIManager::handle_mai2serial_baudrate_change() {
    // Mai2Serial波特率变更逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
}

void UIManager::handle_mai2light_baudrate_change() {
    // Mai2Light波特率变更逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
}

void UIManager::save_uart_settings() {
    // UART设置保存逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
}

void UIManager::reset_uart_settings() {
    // UART设置重置逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
    current_mai2serial_baudrate_ = 115200;
    current_mai2light_baudrate_ = 115200;
}

void UIManager::update_calibration_page() {
    // 更新校准页面
}

void UIManager::update_diagnostics_page() {
    // 更新诊断页面
}

// 故障相关方法实现
// create_error_page函数已移除LVGL依赖
// 现在使用静态framebuffer渲染
 
 void UIManager::update_error_page() {
     if (!has_error_) {
         return;
     }
     
     // 只有当前页面是错误页面时才更新
     if (current_page_ != UIPage::ERROR) {
         return;
     }
     
     // 更新错误信息显示
     // 这里可以添加更详细的错误信息显示逻辑
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
 
 bool UIManager::show_error_page() {
     return set_current_page(UIPage::ERROR);
 }
 
 bool UIManager::report_error(const ErrorInfo& error) {
     current_error_ = error;
     current_error_.timestamp = to_ms_since_boot(get_absolute_time());
     has_error_ = true;
     
     add_error_to_history(current_error_);
     
     // 如果是严重故障，立即切换到错误页面
     if (error.is_critical) {
         set_current_page(UIPage::ERROR);
     }
     
     log_error("Error reported: " + error.description);
     return true;
 }
 
 bool UIManager::clear_error() {
     has_error_ = false;
     current_error_ = ErrorInfo();
     
     log_debug("Error cleared");
     return true;
 }
 
 bool UIManager::has_error() const {
     return has_error_;
 }
 
 ErrorInfo UIManager::get_current_error() const {
     return current_error_;
 }
 
 std::vector<ErrorInfo> UIManager::get_error_history() const {
     return error_history_;
 }
 
 bool UIManager::restart_system() {
     log_debug("System restart requested");
     // 这里应该调用系统重启功能
     // 在实际实现中，可能需要调用硬件相关的重启函数
     return true;
 }
 
 // 全局故障接口实现
 bool UIManager::global_report_error(ErrorType type, const std::string& module_name, 
                                    const std::string& description, uint32_t error_code, 
                                    bool is_critical) {
     global_error_ = ErrorInfo(type, module_name, description, error_code, is_critical);
     global_error_.timestamp = to_ms_since_boot(get_absolute_time());
     global_has_error_ = true;
     
     // 如果UIManager实例存在，立即报告错误
     if (instance_ && instance_->initialized_) {
         instance_->report_error(global_error_);
     }
     
     return true;
 }
 
 bool UIManager::global_has_error() {
     return global_has_error_;
 }
 
 void UIManager::global_clear_error() {
     global_has_error_ = false;
     global_error_ = ErrorInfo();
     
     if (instance_ && instance_->initialized_) {
         instance_->clear_error();
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

// 灯光映射相关方法实现
void UIManager::update_light_mapping_page() {
    // 灯光映射页面更新逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
}

bool UIManager::handle_light_region_selection(int region_index) {
    // 灯光区域选择逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
    selected_light_region_ = "Region " + std::to_string(region_index);
    selected_neopixels_.clear();
    return true;
}

bool UIManager::handle_neopixel_selection(int index) {
    // Neopixel选择逻辑已移除LVGL依赖
    // 现在使用静态framebuffer渲染
    if (index < 0 || index >= 32) return false;
    
    // 切换选中状态
    auto it = std::find(selected_neopixels_.begin(), selected_neopixels_.end(), index);
    if (it == selected_neopixels_.end()) {
        selected_neopixels_.push_back(index);
    } else {
        selected_neopixels_.erase(it);
    }
    
    return true;
}

bool UIManager::save_light_mapping() {
    if (selected_light_region_.empty() || !light_manager_) {
        return false;
    }
    
    // 将选中的neopixel转换为bitmap
    bitmap16_t neopixel_bitmap = 0;
    for (uint8_t led_index : selected_neopixels_) {
        if (led_index < 16) {  // 确保不超出bitmap范围
            neopixel_bitmap |= (1U << led_index);
        }
    }
    
    // 解析区域名称获取区域ID (假设格式为"Region X")
    uint8_t region_id = 1;  // 默认区域1
    if (selected_light_region_.find("Region ") == 0) {
        std::string num_str = selected_light_region_.substr(7);
        region_id = std::stoi(num_str);
    }
    
    // 使用新的bitmap接口设置区域映射
    light_manager_->set_region_bitmap(region_id, neopixel_bitmap);
    
    // 保存到配置
    light_manager_->save_region_mappings();
    
    // 移除LVGL依赖 - 状态显示将通过静态framebuffer实现
    // TODO: 实现静态framebuffer状态显示
    page_needs_redraw_ = true;
    return true;
}

bool UIManager::clear_light_mapping() {
    if (selected_light_region_.empty() || !light_manager_) {
        return false;
    }
    
    // 解析区域名称获取区域ID (假设格式为"Region X")
    uint8_t region_id = 1;  // 默认区域1
    if (selected_light_region_.find("Region ") == 0) {
        std::string num_str = selected_light_region_.substr(7);
        region_id = std::stoi(num_str);
    }
    
    // 清除该区域的映射 (设置为0)
    light_manager_->set_region_bitmap(region_id, 0);
    
    // 保存配置
    light_manager_->save_region_mappings();
    
    // 清除UI中的选中状态
    selected_neopixels_.clear();
    
    // 移除LVGL依赖 - 按钮状态和状态显示将通过静态framebuffer实现
    // TODO: 实现静态framebuffer按钮状态管理和状态显示
    page_needs_redraw_ = true;
    
    return true;
}

bool UIManager::show_light_mapping_page() {
    return set_current_page(UIPage::LIGHT_MAPPING);
}

bool UIManager::handle_logical_key_selection(int key_index, const char* key_name) {
    if (!input_manager_ || !key_name) {
        return false;
    }
    
    // 如果选择了"None"，则清除该GPIO的逻辑按键映射
    if (strcmp(key_name, "None") == 0) {
        input_manager_->clearLogicalKeyMapping(static_cast<uint8_t>(key_index));
        log_debug("Cleared logical key mapping for GPIO " + std::to_string(key_index));
        page_needs_redraw_ = true;
        return true;
    }
    
    // 将选项文本转换为HID按键代码
    HID_KeyCode key_code = getKeyCodeFromName(key_name);
    if (key_code == HID_KeyCode::KEY_NONE) {
        log_error("Invalid key name: " + std::string(key_name));
        return false;
    }
    
    // 添加逻辑按键映射
    if (input_manager_->addLogicalKeyMapping(static_cast<uint8_t>(key_index), key_code)) {
        log_debug("Added logical key mapping: GPIO " + std::to_string(key_index) + " -> " + std::string(key_name));
        page_needs_redraw_ = true;
        return true;
    } else {
        log_error("Failed to add logical key mapping for GPIO " + std::to_string(key_index));
        return false;
    }
    
    return false;
}
