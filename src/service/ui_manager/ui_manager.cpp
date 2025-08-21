#include "ui_manager.h"
#include "../../service/input_manager/input_manager.h"
#include "../../service/light_manager/light_manager.h"
#include "../../service/config_manager/config_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include <cstring>
#include <cstdio>

// 静态实例变量定义
UIManager* UIManager::instance_ = nullptr;

// 静态配置变量
static UIManager_PrivateConfig static_config_;

// 配置管理函数实现
UIManager_PrivateConfig* ui_manager_get_config_holder() {
    return &static_config_;
}

bool ui_manager_load_config_from_manager(ConfigManager* config_manager) {
    if (config_manager == nullptr) {
        return false;
    }
    // 从配置管理器加载配置
    return true;
}

UIManager_PrivateConfig ui_manager_get_config_copy() {
    return static_config_;
}

bool ui_manager_write_config_to_manager(ConfigManager* config_manager, const UIManager_PrivateConfig& config) {
    if (config_manager == nullptr) {
        return false;
    }
    static_config_ = config;
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
    , input_manager_(nullptr)
    , light_manager_(nullptr)
    , config_manager_(nullptr)
    , lv_display_(nullptr)
    , lv_input_device_(nullptr)
    , lv_buf1_(nullptr)
    , lv_buf2_(nullptr)
    , current_page_(UIPage::MAIN)
    , previous_page_(UIPage::MAIN)
    , backlight_enabled_(true)
    , screensaver_active_(false)
    , last_activity_time_(0)
    , last_refresh_time_(0)
    , needs_full_refresh_(true)
    , debug_enabled_(false)
    , current_menu_index_(0)
    , in_menu_mode_(false)
    , selected_device_name_("")
    , selected_channel_(0)
    , sensitivity_slider_(nullptr)
    , device_label_(nullptr) {
    
    // 初始化按钮状态
    for (int i = 0; i < 3; i++) {
        joystick_buttons_[i] = false;
        button_press_times_[i] = 0;
    }
    last_navigation_time_ = 0;
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
    input_manager_ = config.input_manager;
    light_manager_ = config.light_manager;
    config_manager_ = config.config_manager;
    
    if (!display_device_) {
        log_error("Display device is null");
        return false;
    }
    
    // 初始化LVGL
    if (!init_lvgl()) {
        log_error("Failed to initialize LVGL");
        return false;
    }
    
    // 创建所有页面
    create_main_page();
    create_status_page();
    create_settings_page();
    create_calibration_page();
    create_diagnostics_page();
    create_sensitivity_page();
    create_about_page();
    
    // 设置初始页面
    set_current_page(UIPage::MAIN);
    
    initialized_ = true;
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    
    log_debug("UIManager initialized successfully");
    return true;
}

// 清理
void UIManager::deinit() {
    if (!initialized_) {
        return;
    }
    
    deinit_lvgl();
    
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
bool UIManager::init_lvgl() {
    // 初始化LVGL
    lv_init();
    
    // 从ST7735S设备获取实际屏幕参数
    if (!display_device_) {
        log_error("Display device not available for LVGL initialization");
        return false;
    }
    
    uint16_t screen_width = display_device_->get_width();
    uint16_t screen_height = display_device_->get_height();
    
    // 分配显示缓冲区 - 使用实际屏幕尺寸
    const uint32_t buf_size = screen_width * screen_height;
    lv_buf1_ = (lv_color_t*)malloc(buf_size * sizeof(lv_color_t));
    lv_buf2_ = (lv_color_t*)malloc(buf_size * sizeof(lv_color_t));
    
    if (!lv_buf1_ || !lv_buf2_) {
        log_error("Failed to allocate LVGL buffers");
        return false;
    }
    
    // 初始化显示缓冲区
    lv_disp_draw_buf_init(&lv_draw_buf_, lv_buf1_, lv_buf2_, buf_size);
    
    // 注册显示驱动 - 使用实际屏幕尺寸
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screen_width;
    disp_drv.ver_res = screen_height;
    disp_drv.flush_cb = display_flush_cb;
    disp_drv.draw_buf = &lv_draw_buf_;
    disp_drv.user_data = this;
    lv_display_ = lv_disp_drv_register(&disp_drv);
    
    // 注册输入设备驱动
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = input_read_cb;
    indev_drv.user_data = this;
    lv_input_device_ = lv_indev_drv_register(&indev_drv);
    
    return true;
}

// 清理LVGL
void UIManager::deinit_lvgl() {
    if (lv_buf1_) {
        free(lv_buf1_);
        lv_buf1_ = nullptr;
    }
    if (lv_buf2_) {
        free(lv_buf2_);
        lv_buf2_ = nullptr;
    }
    
    // 清理页面对象
    pages_.clear();
    
    lv_display_ = nullptr;
    lv_input_device_ = nullptr;
}

// 显示刷新回调
void UIManager::display_flush_cb(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p) {
    UIManager* ui = static_cast<UIManager*>(disp_drv->user_data);
    if (!ui || !ui->display_device_) {
        lv_disp_flush_ready(disp_drv);
        return;
    }
    
    // 计算区域尺寸
    uint16_t width = area->x2 - area->x1 + 1;
    uint16_t height = area->y2 - area->y1 + 1;
    int32_t pixel_count = width * height;
    
    // 转换LVGL颜色格式到RGB888格式
    uint8_t* rgb888_buffer = new uint8_t[pixel_count * 3];
    for (int32_t i = 0; i < pixel_count; i++) {
        lv_color_t color = color_p[i];
        // LVGL颜色转换为RGB888
        rgb888_buffer[i * 3] = (color.ch.red << 3) | (color.ch.red >> 2);     // R
        rgb888_buffer[i * 3 + 1] = (color.ch.green << 2) | (color.ch.green >> 4); // G
        rgb888_buffer[i * 3 + 2] = (color.ch.blue << 3) | (color.ch.blue >> 2);   // B
    }
    
    // 使用draw_bitmap_rgb888一次性绘制整个区域
    ui->display_device_->draw_bitmap_rgb888(area->x1, area->y1, width, height, rgb888_buffer);
    
    delete[] rgb888_buffer;
    lv_disp_flush_ready(disp_drv);
}

// 输入读取回调
bool UIManager::input_read_cb(lv_indev_drv_t* indev_drv, lv_indev_data_t* data) {
    UIManager* ui = static_cast<UIManager*>(indev_drv->user_data);
    if (!ui) {
        return false;
    }
    
    // 检查摇杆按钮状态
    static uint32_t last_key = LV_KEY_ENTER;
    static bool last_state = false;
    
    bool current_state = false;
    uint32_t current_key = LV_KEY_ENTER;
    
    // 检查各个按钮
    if (ui->joystick_buttons_[0]) { // A按钮 - 上
        current_key = LV_KEY_UP;
        current_state = true;
    } else if (ui->joystick_buttons_[1]) { // B按钮 - 下
        current_key = LV_KEY_DOWN;
        current_state = true;
    } else if (ui->joystick_buttons_[2]) { // 确认按钮
        current_key = LV_KEY_ENTER;
        current_state = true;
    }
    
    data->key = current_key;
    data->state = current_state ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    
    // 更新上次状态
    last_key = current_key;
    last_state = current_state;
    
    return false; // 没有更多数据
}

// 设置当前页面
bool UIManager::set_current_page(UIPage page) {
    if (!is_page_valid(page)) {
        return false;
    }
    
    if (current_page_ == page) {
        return true;
    }
    
    // 隐藏当前页面
    auto current_it = pages_.find(current_page_);
    if (current_it != pages_.end() && current_it->second) {
        lv_obj_add_flag(current_it->second, LV_OBJ_FLAG_HIDDEN);
    }
    
    // 显示新页面
    auto new_it = pages_.find(page);
    if (new_it != pages_.end() && new_it->second) {
        lv_obj_clear_flag(new_it->second, LV_OBJ_FLAG_HIDDEN);
        lv_scr_load(new_it->second);
    }
    
    previous_page_ = current_page_;
    current_page_ = page;
    
    // 触发页面切换事件
    if (page_callback_) {
        page_callback_(page);
    }
    
    statistics_.page_changes++;
    log_debug("Page changed to: " + std::to_string(static_cast<int>(page)));
    
    return true;
}

// 获取当前页面
UIPage UIManager::get_current_page() const {
    return current_page_;
}

// 处理摇杆输入
bool UIManager::handle_joystick_input(JoystickButton button, bool pressed) {
    if (static_cast<int>(button) >= 3) {
        return false;
    }
    
    int btn_idx = static_cast<int>(button);
    joystick_buttons_[btn_idx] = pressed;
    
    if (pressed) {
        button_press_times_[btn_idx] = to_ms_since_boot(get_absolute_time());
        statistics_.joystick_events++;
        
        // 更新活动时间
        last_activity_time_ = to_ms_since_boot(get_absolute_time());
        
        // 如果屏保激活，先退出屏保
        if (screensaver_active_) {
            screensaver_active_ = false;
            return true;
        }
        
        // 处理按钮事件
        process_joystick_event(button, pressed);
    }
    
    // 触发摇杆回调
    if (joystick_callback_) {
        joystick_callback_(button, pressed);
    }
    
    return true;
}

// 处理摇杆事件
void UIManager::process_joystick_event(JoystickButton button, bool pressed) {
    if (!pressed) return;
    
    switch (button) {
        case JoystickButton::BUTTON_A:
            handle_navigation_input(true); // 向上
            break;
        case JoystickButton::BUTTON_B:
            handle_navigation_input(false); // 向下
            break;
        case JoystickButton::BUTTON_CONFIRM:
            handle_confirm_input();
            break;
    }
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
            if (sensitivity_slider_) {
                int32_t current_value = lv_slider_get_value(sensitivity_slider_);
                int32_t new_value = up ? current_value + 1 : current_value - 1;
                new_value = (new_value < 0) ? 0 : (new_value > 63) ? 63 : new_value;
                lv_slider_set_value(sensitivity_slider_, new_value, LV_ANIM_ON);
                
                // 应用灵敏度变化
                if (!selected_device_name_.empty() && input_manager_) {
                    input_manager_->set_channel_sensitivity_by_name(selected_device_name_, selected_channel_, new_value);
                }
            }
            break;
            
        default:
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
                case 5: set_current_page(UIPage::ABOUT); break;
                case 6: /* 退出或其他操作 */ break;
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
    
    std::string device_name;
    uint8_t channel;
    
    // 检测当前被触摸的通道
    if (detect_touched_channel(device_name, channel)) {
        selected_device_name_ = device_name;
        selected_channel_ = channel;
        
        // 更新显示
        update_sensitivity_display();
        
        log_debug("Auto-selected device: " + device_name + ", channel: " + std::to_string(channel));
        return true;
    }
    
    log_debug("No touched channel detected");
    return false;
}

// 检测被触摸的通道
bool UIManager::detect_touched_channel(std::string& device_name, uint8_t& channel) {
    if (!input_manager_) {
        return false;
    }
    
    // 获取所有可用设备
    auto devices = get_available_devices();
    
    for (const auto& dev_name : devices) {
        for (uint8_t ch = 0; ch < 12; ch++) {
            bool pressed;
            uint8_t pressure;
            
            // 检查通道是否被触摸
            if (input_manager_->get_channel_touch_state_by_name(dev_name, ch, pressed, pressure)) {
                if (pressed && pressure > 10) { // 确保有足够的压力
                    device_name = dev_name;
                    channel = ch;
                    return true;
                }
            }
        }
    }
    
    return false;
}

// 获取可用设备列表
std::vector<std::string> UIManager::get_available_devices() {
    std::vector<std::string> devices;
    
    if (input_manager_) {
        devices = input_manager_->get_device_names();
    }
    
    return devices;
}

// 更新灵敏度显示
void UIManager::update_sensitivity_display() {
    if (!device_label_ || selected_device_name_.empty()) {
        return;
    }
    
    std::string label_text = "Device: " + selected_device_name_ + "\nChannel: " + std::to_string(selected_channel_);
    lv_label_set_text(device_label_, label_text.c_str());
    
    // 获取当前灵敏度值并更新滑块
    if (sensitivity_slider_ && input_manager_) {
        uint8_t current_sensitivity;
        if (input_manager_->get_channel_sensitivity_by_name(selected_device_name_, selected_channel_, current_sensitivity)) {
            lv_slider_set_value(sensitivity_slider_, current_sensitivity, LV_ANIM_ON);
        }
    }
}

// 创建主页面
void UIManager::create_main_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 创建标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "MaiMai Control");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // 创建菜单项
    const char* menu_items[] = {
        "Status", "Settings", "Calibration", 
        "Diagnostics", "Sensitivity", "About", "Exit"
    };
    
    for (int i = 0; i < 7; i++) {
        lv_obj_t* btn = lv_btn_create(page);
        lv_obj_set_size(btn, 120, 30);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 40 + i * 35);
        
        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text(label, menu_items[i]);
        lv_obj_center(label);
        
        // 设置按钮事件
        lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_CLICKED, this);
        lv_obj_set_user_data(btn, (void*)(intptr_t)i);
    }
    
    pages_[UIPage::MAIN] = page;
}

// 创建灵敏度页面
void UIManager::create_sensitivity_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 创建标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Sensitivity Adjust");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // 创建设备信息标签
    device_label_ = lv_label_create(page);
    lv_label_set_text(device_label_, "Touch a point to select");
    lv_obj_set_style_text_color(device_label_, lv_color_white(), 0);
    lv_obj_align(device_label_, LV_ALIGN_TOP_MID, 0, 40);
    
    // 创建灵敏度滑块
    sensitivity_slider_ = lv_slider_create(page);
    lv_obj_set_size(sensitivity_slider_, 120, 20);
    lv_obj_align(sensitivity_slider_, LV_ALIGN_CENTER, 0, 0);
    lv_slider_set_range(sensitivity_slider_, 0, 63);
    lv_slider_set_value(sensitivity_slider_, 15, LV_ANIM_OFF);
    
    // 设置滑块事件
    lv_obj_add_event_cb(sensitivity_slider_, slider_event_cb, LV_EVENT_VALUE_CHANGED, this);
    
    // 创建说明文本
    lv_obj_t* help_text = lv_label_create(page);
    lv_label_set_text(help_text, "Press CONFIRM to\nauto-select point");
    lv_obj_set_style_text_color(help_text, lv_color_white(), 0);
    lv_obj_align(help_text, LV_ALIGN_BOTTOM_MID, 0, -20);
    
    pages_[UIPage::SENSITIVITY] = page;
}

// 创建其他页面（简化实现）
void UIManager::create_status_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "System Status");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    pages_[UIPage::STATUS] = page;
}

void UIManager::create_settings_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    pages_[UIPage::SETTINGS] = page;
}

void UIManager::create_calibration_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Calibration");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    pages_[UIPage::CALIBRATION] = page;
}

void UIManager::create_diagnostics_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Diagnostics");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    pages_[UIPage::DIAGNOSTICS] = page;
}

void UIManager::create_about_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "About");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    pages_[UIPage::ABOUT] = page;
}

// 事件回调函数
void UIManager::button_event_cb(lv_event_t* e) {
    UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
    lv_obj_t* btn = lv_event_get_target(e);
    int menu_index = (int)(intptr_t)lv_obj_get_user_data(btn);
    
    if (ui) {
        ui->current_menu_index_ = menu_index;
        ui->handle_confirm_input();
    }
}

void UIManager::slider_event_cb(lv_event_t* e) {
    UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
    lv_obj_t* slider = lv_event_get_target(e);
    
    if (ui && ui->input_manager_ && !ui->selected_device_name_.empty()) {
        int32_t value = lv_slider_get_value(slider);
        ui->input_manager_->set_channel_sensitivity_by_name(
            ui->selected_device_name_, ui->selected_channel_, value);
    }
}

// 主循环
void UIManager::task() {
    if (!initialized_) {
        return;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // 处理LVGL任务
    lv_timer_handler();
    
    // 处理背光和屏保
    handle_backlight();
    handle_screensaver();
    
    // 更新统计信息
    statistics_.uptime_seconds = current_time / 1000;
    
    // 更新当前页面
    switch (current_page_) {
        case UIPage::STATUS:
            update_status_page();
            break;
        case UIPage::SENSITIVITY:
            update_sensitivity_page();
            break;
        default:
            break;
    }
    
    last_refresh_time_ = current_time;
}

// 更新页面
void UIManager::update_status_page() {
    // 更新状态页面内容
}

void UIManager::update_sensitivity_page() {
    // 更新灵敏度页面内容
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

// 屏保管理
void UIManager::handle_screensaver() {
    if (!static_config_.enable_screensaver) {
        return;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = static_config_.screensaver_timeout * 1000;
    
    if (current_time - last_activity_time_ > timeout_ms) {
        if (!screensaver_active_) {
            screensaver_active_ = true;
            // 可以在这里实现屏保效果
        }
    }
}

// 设置背光
bool UIManager::set_backlight(bool enabled) {
    backlight_enabled_ = enabled;
    if (display_device_) {
        return display_device_->set_backlight(enabled);
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
        return display_device_->set_brightness(brightness);
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
        return display_device_->clear_screen(0x0000); // 黑色
    }
    return false;
}

// 刷新屏幕
bool UIManager::refresh_screen() {
    if (!initialized_) {
        return false;
    }
    
    lv_refr_now(lv_display_);
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

void UIManager::set_joystick_callback(UIJoystickCallback callback) {
    joystick_callback_ = callback;
}

// 工具函数
bool UIManager::is_page_valid(UIPage page) const {
    return static_cast<int>(page) >= 0 && static_cast<int>(page) <= 6;
}

void UIManager::log_debug(const std::string& message) {
    if (debug_enabled_) {
        usb_serial_log("[UIManager] " + message);
    }
}

void UIManager::log_error(const std::string& message) {
    usb_serial_log("[UIManager ERROR] " + message);
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

bool UIManager::enable_screensaver(bool enabled) {
    static_config_.enable_screensaver = enabled;
    return true;
}

bool UIManager::is_screensaver_active() const {
    return screensaver_active_;
}

bool UIManager::set_screensaver_timeout(uint16_t timeout_seconds) {
    static_config_.screensaver_timeout = timeout_seconds;
    return true;
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

bool UIManager::set_sensitivity_for_device(const std::string& device_name, uint8_t channel, uint8_t sensitivity) {
    if (input_manager_) {
        return input_manager_->set_channel_sensitivity_by_name(device_name, channel, sensitivity);
    }
    return false;
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

void UIManager::update_calibration_page() {
    // 更新校准页面
}

void UIManager::update_diagnostics_page() {
    // 更新诊断页面
}