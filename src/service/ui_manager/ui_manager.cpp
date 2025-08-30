#include "ui_manager.h"
#include "page_registry.h"
#include "page/main_page.h"

#include "../../service/input_manager/input_manager.h"
#include "../../service/light_manager/light_manager.h"
#include "../../service/config_manager/config_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "../../protocol/st7735s/st7735s.h"
#include "engine/fonts/font_data.h"
#include "engine/page_construction/page_template.h"
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
// 新页面引擎相关函数
bool UIManager::switch_to_page(const std::string& page_name) {
    // 简化的页面切换，直接更新页面名称
    current_page_name_ = page_name;
    current_menu_index_ = 0;
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    log_debug("Switched to page: " + page_name);
    return true;
}

const std::string& UIManager::get_current_page_name() const {
    return current_page_name_;
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
    , current_menu_index_(0)
    , buttons_active_low_(true)
    , backlight_enabled_(true)
    , screen_off_(false)
    , last_activity_time_(0)
    , last_refresh_time_(0)
    , needs_full_refresh_(true)
    , debug_enabled_(false)
    , last_navigation_time_(0)
    , cursor_blink_timer_(0)
    , cursor_visible_(true)
    , current_page_name_("status")
    , has_error_(false)
    , binding_step_(0)
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
    
    // 初始化页面注册系统
    ui::PageRegistry::get_instance().register_default_pages();
    log_debug("Page registry initialized with default pages");
    
    // 切换到主界面
    switch_to_page("main");
    
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
            bool was_pressed = joystick_buttons_[i];
            joystick_buttons_[i] = button_states[i];
            
            if (button_states[i]) { // 按钮按下
                button_press_times_[i] = current_time;
                last_activity_time_ = current_time;
                
                // 如果息屏，先唤醒屏幕
                if (screen_off_) {
                    wake_screen();
                }
                // 按下时不触发操作，等待释放
            } else if (was_pressed) { // 按钮释放（从按下状态变为释放状态）
                last_activity_time_ = current_time;
                
                // 只有在屏幕开启时才处理按钮释放事件
                if (!screen_off_) {
                    // 计算按下持续时间
                    uint32_t press_duration = current_time - button_press_times_[i];
                    
                    // 防抖处理：只有按下时间超过最小阈值才处理
                    if (press_duration >= 30) { // 减少防抖时间提高响应性
                        // 调用统一的摇杆输入处理接口
                        bool handled = handle_joystick_input(i);
                        if (handled) {
                            log_debug("Joystick input handled: button " + std::to_string(i));
                        }
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
    
    // 页面注册器已移除，无需清理
    
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
    display_device_->write_buffer(framebuffer_, SCREEN_BUFFER_SIZE);
}

// 30fps刷新任务
void UIManager::refresh_task_30fps() {
    if (!initialized_ || !display_device_) {
        return;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    // 30fps = 33.33ms间隔
    if (current_time - last_refresh_time_ >= 33) {
        // 屏幕亮起时始终刷新，对于主页面也需要重绘以更新运行时长

        if (page_template_) {
            draw_page_with_template();
            // 渲染光标指示器
            render_cursor_indicator();
        }
        
        // 刷新显示
        refresh_display();
        
        last_refresh_time_ = current_time;
    }
}

// 设置当前页面
// set_current_page函数已删除，页面切换已改为面向对象方式

// 重置页面数据
void UIManager::reset_page_data() {
    // 清空页面数据，让页面组件自己管理内容
    page_data_.title = "";
    page_data_.menu_items.clear();
    page_data_.content = "";
    page_data_.progress_value = 0;
    page_data_.selected_index = current_menu_index_; // 与当前菜单索引保持同步
    
    // 重置光标位置：根据用户需求设置初始光标位置
    // 滚动时光标在第一行(索引0)，无滚动时光标在第一个可选内容
    current_menu_index_ = 0;
    page_data_.selected_index = 0;
    
    // 确保选中索引在有效范围内
    int menu_count = page_template_ ? page_template_->get_menu_item_count() : 0;
    if (menu_count > 0 && current_menu_index_ >= menu_count) {
        current_menu_index_ = 0;
        page_data_.selected_index = 0;
    }
}

// 处理导航输入
void UIManager::handle_navigation_input(bool up) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 防抖处理
    if (current_time - last_navigation_time_ < 150) {
        return;
    }
    last_navigation_time_ = current_time;
    
    // 记录活动时间
    last_activity_time_ = current_time;
    
    // 如果屏幕关闭，唤醒屏幕
    if (screen_off_) {
        wake_screen();
        return;
    }
    
    if (!page_template_) {
        return;
    }
    
    // 获取目标行数据
    const auto& target_lines = page_template_->is_scroll_enabled() ? 
                              page_template_->get_all_lines() : page_template_->get_lines();
    
    if (target_lines.empty()) {
        return;
    }
    
    bool is_scrollable = page_template_->is_scroll_enabled();
    int max_index;
    
    if (is_scrollable) {
        // 可滚动模式：Z字型选择所有区域
        max_index = target_lines.size() - 1;
        
        // 确保当前索引在有效范围内
        if (current_menu_index_ < 0 || current_menu_index_ > max_index) {
            current_menu_index_ = 0;
        }
        
        // 执行导航
        if (up) {
            current_menu_index_--;
            if (current_menu_index_ < 0) {
                current_menu_index_ = max_index; // 循环到最后一项
            }
        } else {
            current_menu_index_++;
            if (current_menu_index_ > max_index) {
                current_menu_index_ = 0; // 循环到第一项
            }
        }
        
        // 让滚动跟随光标：使用新的 set_visible_end_line 方法
        // 这样光标始终可见，滚动会自动调整以跟随光标位置
        page_template_->set_visible_end_line(current_menu_index_);
        page_template_->set_selected_index(current_menu_index_);
        
    } else {
        // 不可滚动模式：只选择可触发区域
        int menu_count = page_template_->get_menu_item_count();
        if (menu_count <= 0) {
            return;
        }
        
        // 确保当前索引在有效范围内
        if (current_menu_index_ < 0 || current_menu_index_ >= menu_count) {
            current_menu_index_ = 0;
        }
        
        // 执行导航
        if (up) {
            current_menu_index_--;
            if (current_menu_index_ < 0) {
                current_menu_index_ = menu_count - 1;
            }
        } else {
            current_menu_index_++;
            if (current_menu_index_ >= menu_count) {
                current_menu_index_ = 0;
            }
        }
        
        // 更新页面模板的选中索引
        page_template_->set_selected_index(current_menu_index_);
    }
    
    log_debug("Navigation: " + std::string(up ? "UP" : "DOWN") + 
              ", Index: " + std::to_string(current_menu_index_) + 
              ", Scrollable: " + (is_scrollable ? "true" : "false"));
}

// 处理确认输入
void UIManager::handle_confirm_input() {
    // 使用PageTemplate的LineConfig跳转机制
    if (page_template_) {
        bool is_scrollable = page_template_->is_scroll_enabled();
        
        if (is_scrollable) {
            // 可滚动模式：需要从所有行中找到当前选中行的配置
            const auto& all_lines = page_template_->get_all_lines();
            if (current_menu_index_ >= 0 && current_menu_index_ < (int)all_lines.size()) {
                LineConfig line_config = all_lines[current_menu_index_];
                
                // 检查是否是菜单跳转类型
                if (line_config.type == LineType::MENU_JUMP && !line_config.target_page_name.empty()) {
                    // 检查目标页面是否存在
                    std::vector<std::string> available_pages = get_available_pages();
                    bool page_exists = std::find(available_pages.begin(), available_pages.end(), line_config.target_page_name) != available_pages.end();
                    
                    if (!page_exists) {
                        log_error("Target page does not exist: " + line_config.target_page_name + ", switching to main page");
                        switch_to_page("main");
                        return;
                    }
                    
                    // 保存当前页面状态到导航管理器
                    if (nav_manager_) {
                        int scroll_pos = page_template_->get_scroll_position();
                        nav_manager_->push_page(current_page_name_, current_menu_index_, scroll_pos);
                    }
                    // 使用LineConfig的target_page_name进行跳转
                    switch_to_page(line_config.target_page_name);
                    return;
                }
                // 检查是否是返回项类型
                else if (line_config.type == LineType::BACK_ITEM) {
                    // 处理返回操作，恢复上一页状态
                    if (nav_manager_ && nav_manager_->can_go_back()) {
                        PageState prev_state = nav_manager_->pop_page();
                        current_page_name_ = prev_state.page_name;
                        current_menu_index_ = prev_state.cursor_position;
                        
                        // 重置页面数据并恢复状态
                        reset_page_data();
                        
                        // 恢复光标和滚动位置
                        if (page_template_) {
                            page_template_->set_selected_index(current_menu_index_);
                            page_template_->set_scroll_position(prev_state.scroll_position);
                        }
                        
                        last_activity_time_ = to_ms_since_boot(get_absolute_time());
                        log_debug("Returned to page: " + current_page_name_ + 
                                 ", cursor: " + std::to_string(current_menu_index_) + 
                                 ", scroll: " + std::to_string(prev_state.scroll_position));
                    }
                    return;
                }
            }
        } else {
            // 不可滚动模式：使用原有逻辑
            int menu_count = page_template_->get_menu_item_count();
            if (menu_count > 0 && current_menu_index_ >= 0 && current_menu_index_ < menu_count) {
                // 获取当前选中的菜单项配置
                LineConfig line_config = page_template_->get_line_config(current_menu_index_);
            
                // 检查是否是菜单跳转类型
                if (line_config.type == LineType::MENU_JUMP && !line_config.target_page_name.empty()) {
                // 保存当前页面状态到导航管理器
                if (nav_manager_) {
                    int scroll_pos = page_template_->get_scroll_position();
                    nav_manager_->push_page(current_page_name_, current_menu_index_, scroll_pos);
                }
                // 使用LineConfig的target_page_name进行跳转
                switch_to_page(line_config.target_page_name);
                return;
            }
            // 检查是否是返回项类型
            else if (line_config.type == LineType::BACK_ITEM) {
                // 处理返回操作，恢复上一页状态
                if (nav_manager_ && nav_manager_->can_go_back()) {
                    PageState prev_state = nav_manager_->pop_page();
                    current_page_name_ = prev_state.page_name;
                    current_menu_index_ = prev_state.cursor_position;
                    
                    // 重置页面数据并恢复状态
                    reset_page_data();
                    
                    // 恢复光标和滚动位置
                    if (page_template_) {
                        page_template_->set_selected_index(current_menu_index_);
                        page_template_->set_scroll_position(prev_state.scroll_position);
                    }
                    
                    last_activity_time_ = to_ms_since_boot(get_absolute_time());
                    log_debug("Returned to page: " + current_page_name_ + 
                             ", cursor: " + std::to_string(current_menu_index_) + 
                             ", scroll: " + std::to_string(prev_state.scroll_position));
                }
                return;
            }
            }
        }
    }
    
    // 如果没有找到跳转目标或不是跳转类型，使用默认行为
    if (current_page_name_ != "main") {
        // 非主页面默认返回主页面
        switch_to_page("main");
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
    if (!page_template_) {
        return;
    }
    
    update_page_template_content();
    page_template_->draw();
}

void UIManager::update_page_template_content() {
    if (!page_template_) {
        return;
    }
    
    // 使用PageRegistry获取页面构造器并渲染
    auto& registry = ui::PageRegistry::get_instance();
    
    // 安全检查：确保页面存在
    if (!registry.has_page(current_page_name_)) {
        log_error("Page not found: " + current_page_name_ + ", switching to main page");
        current_page_name_ = "main";
        current_menu_index_ = 0;
        return;
    }
    
    auto page_constructor = registry.get_page(current_page_name_);
    
    if (page_constructor) {
        // 使用面向对象的页面构造器渲染页面
        page_constructor->render(*page_template_);
    } else if (current_page_name_ == "error" && has_error_) {
        // 错误页面的后备处理
        PageTemplates::create_error_page(*page_template_);
    } else if (current_page_name_ != "error") {
        // 如果页面构造器为空且不是错误页面，切换到主页面
        log_error("Page constructor is null for: " + current_page_name_);
        current_page_name_ = "main";
        current_menu_index_ = 0;
    }
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
    
    // 故障检测已简化，不再需要专门的处理函数
    
    // 更新统计信息
    statistics_.uptime_seconds = current_time / 1000;
    
    // 处理输入
    handle_input();
    
    // 息屏状态下的节能处理
    if (screen_off_) {
        // 息屏时只进行必要的检测，不进行渲染
        // 检查是否有需要唤醒屏幕的条件
        
        // 检查摇杆输入
        // 简单检查摇杆状态，如有输入则唤醒
        for (int i = 0; i < 3; i++) {
            if (joystick_buttons_[i]) {
                wake_screen();
                break;
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

    // 统计信息已简化，不再记录刷新次数
    return true;
}

// 强制刷新
bool UIManager::force_refresh() {
    needs_full_refresh_ = true;
    return refresh_screen();
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

// 静态日志方法实现
void UIManager::log_debug_static(const std::string& message) {
    auto* logger = USB_SerialLogs::get_global_instance();
    if (logger) {
        logger->debug(message, "UIManager");
    }
}

void UIManager::log_error_static(const std::string& message) {
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

std::vector<std::string> UIManager::get_available_pages() {
    // 从PageRegistry获取动态注册的页面列表
    auto& registry = ui::PageRegistry::get_instance();
    std::vector<std::string> registered_pages = registry.get_all_page_names();
    
    if (!registered_pages.empty()) {
        return registered_pages;
    }
    
    // 如果注册表为空，返回默认页面列表（包含main_menu）
    return {"main", "main_menu", "status", "settings", 
            "calibration", "diagnostics", 
            "sensitivity", "about"};
}

// 统一的摇杆输入处理接口
bool UIManager::handle_joystick_input(int button) {
    // 更新活动时间
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    switch (button) {
        case 0: // 数组索引0 = joystick_a_pin_ = GPIO 2 (下方向)
            log_debug("Joystick DOWN pressed (GPIO 2)");
            return navigate_menu(false);
        case 1: // 数组索引1 = joystick_b_pin_ = GPIO 3 (上方向)
            log_debug("Joystick UP pressed (GPIO 3)");
            return navigate_menu(true);
        case 2: // 数组索引2 = joystick_confirm_pin_ = GPIO 1 (确认)
            log_debug("Joystick CONFIRM pressed (GPIO 1)");
            return confirm_selection();
        default:
            log_debug("Unknown joystick button index: " + std::to_string(button));
            return false;
    }
}

bool UIManager::navigate_menu(bool up) {
    handle_navigation_input(up);
    return true;
}

bool UIManager::navigate_menu_horizontal(bool right) {
    // 水平导航逻辑（用于某些页面的左右选择）
    // 目前大多数页面不需要水平导航，可以根据需要扩展
    return true;
}

bool UIManager::confirm_selection() {
    handle_confirm_input();
    return true;
}

bool UIManager::handle_back_navigation() {
    // 统一的返回导航处理
    if (nav_manager_ && nav_manager_->can_go_back()) {
        PageState prev_state = nav_manager_->pop_page();
        return switch_to_page(prev_state.page_name);
    } else {
        // 默认返回主页面
        return switch_to_page("main");
    }
}

// 动态光标渲染
void UIManager::render_cursor_indicator() {
    if (!graphics_engine_ || !page_template_) {
        return;
    }
    
    // 更新光标闪烁状态 (500ms闪烁，更快响应)
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - cursor_blink_timer_ >= 500) {
        cursor_visible_ = !cursor_visible_;
        cursor_blink_timer_ = current_time;
    }
    
    if (!cursor_visible_) {
        return;
    }
    
    // 获取目标行数据
    const auto& target_lines = page_template_->is_scroll_enabled() ? 
                              page_template_->get_all_lines() : page_template_->get_lines();
    
    if (target_lines.empty()) {
        return;
    }
    
    // 实现两种光标模式
    bool is_scrollable = page_template_->is_scroll_enabled();
    int actual_line_index = -1;
    
    if (is_scrollable) {
        // 可滚动模式：Z字型选择所有区域（包括文本）
        if (current_menu_index_ >= 0 && current_menu_index_ < (int)target_lines.size()) {
            actual_line_index = current_menu_index_;
        }
    } else {
        // 不可滚动模式：只选择可触发区域
        int menu_item_counter = 0;
        for (int i = 0; i < (int)target_lines.size(); ++i) {
            if (!target_lines[i].text.empty() && 
                (target_lines[i].type == LineType::MENU_JUMP || 
                 target_lines[i].type == LineType::INT_SETTING || 
                 target_lines[i].type == LineType::BUTTON_ITEM || 
                 target_lines[i].type == LineType::BACK_ITEM)) {
                if (menu_item_counter == current_menu_index_) {
                    actual_line_index = i;
                    break;
                }
                menu_item_counter++;
            }
        }
    }
    
    // 只有找到有效行时才显示光标
    if (actual_line_index >= 0 && actual_line_index < (int)target_lines.size()) {
        // 在滚动模式下，需要转换为可见行索引
        int visible_line_index = actual_line_index;
        if (page_template_->is_scroll_enabled()) {
            int display_start = page_template_->get_scroll_start_index();
            visible_line_index = actual_line_index - display_start;
            
            // 光标应该始终渲染，因为滚动会跟随光标确保其可见
            // 如果计算出的可见行索引超出范围，则进行边界限制
            if (visible_line_index < 0) {
                visible_line_index = 0;
            } else if (visible_line_index >= page_template_->get_visible_lines_count()) {
                visible_line_index = page_template_->get_visible_lines_count() - 1;
            }
        }
        
        // 获取行矩形区域
        Rect line_rect;
        if (page_template_->is_split_screen_enabled()) {
            // 分屏模式下需要正确判断光标在左侧还是右侧
            // 需要根据菜单项索引来确定是在左侧还是右侧内容中
            
            // 计算左侧可交互菜单项数量
            const auto& left_lines = page_template_->get_left_lines();
            int left_menu_count = 0;
            for (const auto& line : left_lines) {
                if (!line.text.empty() && 
                    (line.type == LineType::MENU_JUMP || 
                     line.type == LineType::INT_SETTING || 
                     line.type == LineType::BUTTON_ITEM || 
                     line.type == LineType::BACK_ITEM)) {
                    left_menu_count++;
                }
            }
            
            bool is_left_side = (current_menu_index_ < left_menu_count);
            int local_line_index;
            
            if (is_left_side) {
                // 在左侧内容中，找到对应的行索引
                local_line_index = 0;
                int menu_counter = 0;
                for (int i = 0; i < (int)left_lines.size() && i < 4; i++) {
                    if (!left_lines[i].text.empty() && 
                        (left_lines[i].type == LineType::MENU_JUMP || 
                         left_lines[i].type == LineType::INT_SETTING || 
                         left_lines[i].type == LineType::BUTTON_ITEM || 
                         left_lines[i].type == LineType::BACK_ITEM)) {
                        if (menu_counter == current_menu_index_) {
                            local_line_index = i;
                            break;
                        }
                        menu_counter++;
                    }
                }
                line_rect = page_template_->get_split_left_rect(local_line_index);
            } else {
                // 在右侧内容中，找到对应的行索引
                const auto& right_lines = page_template_->get_right_lines();
                local_line_index = 0;
                int menu_counter = 0;
                int right_menu_index = current_menu_index_ - left_menu_count;
                for (int i = 0; i < (int)right_lines.size() && i < 4; i++) {
                    if (!right_lines[i].text.empty() && 
                        (right_lines[i].type == LineType::MENU_JUMP || 
                         right_lines[i].type == LineType::INT_SETTING || 
                         right_lines[i].type == LineType::BUTTON_ITEM || 
                         right_lines[i].type == LineType::BACK_ITEM)) {
                        if (menu_counter == right_menu_index) {
                            local_line_index = i;
                            break;
                        }
                        menu_counter++;
                    }
                }
                line_rect = page_template_->get_split_right_rect(local_line_index);
            }
        } else {
            line_rect = PageTemplate::get_line_rect(visible_line_index);
        }
        
        // 扩展光标区域，上下各扩展2像素，避免遮挡字体
        line_rect.y -= 2;
        line_rect.height += 4;
        
        // 绘制包围框光标
        graphics_engine_->draw_rect(line_rect, COLOR_WHITE);
    }
}

int UIManager::get_current_menu_index() const {
    return current_menu_index_;
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

void UIManager::add_error_to_history(const ErrorInfo& error) {
    error_history_.push_back(error);
    
    // 限制历史记录数量，避免内存溢出
    const size_t MAX_ERROR_HISTORY = 50;
    if (error_history_.size() > MAX_ERROR_HISTORY) {
        error_history_.erase(error_history_.begin());
    }
}
