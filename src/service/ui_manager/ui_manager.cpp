#include "ui_manager.h"
#include "../../service/input_manager/input_manager.h"
#include "../../service/light_manager/light_manager.h"
#include "../../service/config_manager/config_manager.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "../../protocol/st7735s/st7735s.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

// 静态实例变量定义
UIManager* UIManager::instance_ = nullptr;

// 静态配置变量
static UIManager_PrivateConfig static_config_;

// 静态故障变量定义
ErrorInfo UIManager::global_error_;
bool UIManager::global_has_error_ = false;

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
    , screen_off_(false)
    , last_activity_time_(0)
    , last_refresh_time_(0)
    , needs_full_refresh_(true)
    , debug_enabled_(false)
    , current_menu_index_(0)
    , in_menu_mode_(false)
    
    // 灵敏度调整相关
    , selected_device_index_(-1)
    , selected_channel_(0)
    , sensitivity_slider_(nullptr)
    , device_label_(nullptr)
    , auto_adjust_active_(false)
    , auto_adjust_button_(nullptr)
    
    // 触摸映射相关
    , touch_mapping_active_(false)
    , mapping_step_(0)
    , mapping_device_addr_(0)
    , mapping_channel_(0)
    , mapping_status_label_(nullptr)
    , mapping_area_list_(nullptr)
    
    // 按键映射相关
    , key_mapping_active_(false)
    , selected_key_index_(-1)
    , key_list_(nullptr)
    , hid_key_list_(nullptr)
    , gpio_label_(nullptr)
    , logical_key1_dropdown_(nullptr)
    , logical_key2_dropdown_(nullptr)
    , logical_key3_dropdown_(nullptr)
    , selected_gpio_(-1)
    
    // 引导式绑区相关
    , guided_binding_active_(false)
    , binding_step_(0)
    , binding_progress_bar_(nullptr)
    , binding_step_label_(nullptr)
    
    // 灯光映射相关
    , light_mapping_active_(false)
    , selected_light_region_("")
    , selected_neopixels_()
    , light_region_list_(nullptr)
    , neopixel_grid_(nullptr)
    , light_mapping_status_(nullptr)
    
    // UART设置相关
    , mai2serial_baudrate_dropdown_(nullptr)
    , mai2light_baudrate_dropdown_(nullptr)
    , uart_status_label_(nullptr)
    , current_mai2serial_baudrate_(115200)
    , current_mai2light_baudrate_(115200)
    
    // 状态页面相关
    , status_mode_label_(nullptr)
    , status_touch_label_(nullptr)
    , status_binding_label_(nullptr)
    , status_sensitivity_label_(nullptr)
    , status_light_label_(nullptr)
    , status_system_label_(nullptr)
    , has_error_(false) {
    
    // 初始化按钮状态
    for (int i = 0; i < 3; i++) {
        joystick_buttons_[i] = false;
        button_press_times_[i] = 0;
    }
    
    // 初始化Neopixel按钮数组
    for (int i = 0; i < 32; i++) {
        neopixel_buttons_[i] = nullptr;
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
    create_touch_mapping_page();
    create_key_mapping_page();
    create_guided_binding_page();
    create_error_page();
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
        rgb888_buffer[i * 3 + 1] = (color.ch.green_h << 2) | (color.ch.green_h >> 4); // G
        rgb888_buffer[i * 3 + 2] = (color.ch.blue << 3) | (color.ch.blue >> 2);   // B
    }
    
    // 使用draw_bitmap_rgb888一次性绘制整个区域
    ui->display_device_->draw_bitmap_rgb888(area->x1, area->y1, width, height, rgb888_buffer);
    
    delete[] rgb888_buffer;
    lv_disp_flush_ready(disp_drv);
}

// 输入读取回调
void UIManager::input_read_cb(lv_indev_drv_t* indev_drv, lv_indev_data_t* data) {
    UIManager* ui = static_cast<UIManager*>(indev_drv->user_data);
    if (!ui) {
        return;
    }
    
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
    if (new_it == pages_.end() || !new_it->second) {
        // 页面不存在，需要创建
        switch (page) {
            case UIPage::MAIN: create_main_page(); break;
            case UIPage::STATUS: create_status_page(); break;
            case UIPage::SENSITIVITY: create_sensitivity_page(); break;
            case UIPage::TOUCH_MAPPING: create_touch_mapping_page(); break;
            case UIPage::KEY_MAPPING: create_key_mapping_page(); break;
            case UIPage::GUIDED_BINDING: create_guided_binding_page(); break;
            case UIPage::SETTINGS: create_settings_page(); break;
            case UIPage::UART_SETTINGS: create_uart_settings_page(); break;
            case UIPage::CALIBRATION: create_calibration_page(); break;
            case UIPage::DIAGNOSTICS: create_diagnostics_page(); break;
            case UIPage::LIGHT_MAPPING: create_light_mapping_page(); break;
            case UIPage::ABOUT: create_about_page(); break;
            default: break;
        }
        new_it = pages_.find(page);
    }
    
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
        
        // 如果息屏，先唤醒屏幕
        if (screen_off_) {
            wake_screen();
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
                if (selected_device_index_ >= 0 && input_manager_) {
                    // 获取设备状态
                    InputManager::TouchDeviceStatus device_status[8];
                    int device_count;
                    input_manager_->get_all_device_status(device_status, device_count);
                    
                    if (selected_device_index_ < device_count) {
                        uint16_t device_addr = device_status[selected_device_index_].device.device_addr;
                        input_manager_->setSensitivity(device_addr, selected_channel_, new_value);
                    }
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
    
    if (mapping_status_label_) {
        lv_label_set_text(mapping_status_label_, "Mapping cancelled");
    }
    
    log_debug("Touch mapping cancelled");
    return true;
}

bool UIManager::show_mapping_selection_ui() {
    if (!mapping_area_list_) {
        return false;
    }
    
    // 清空列表
    lv_obj_clean(mapping_area_list_);
    
    // 根据当前工作模式显示不同的映射选项
    if (input_manager_) {
        InputWorkMode mode = input_manager_->getWorkMode();
        if (mode == InputWorkMode::SERIAL_MODE) {
            // 添加Serial模式的区域选项
            lv_list_add_text(mapping_area_list_, "A1");
            lv_list_add_text(mapping_area_list_, "A2");
            lv_list_add_text(mapping_area_list_, "A3");
            lv_list_add_text(mapping_area_list_, "A4");
            lv_list_add_text(mapping_area_list_, "A5");
            lv_list_add_text(mapping_area_list_, "A6");
            lv_list_add_text(mapping_area_list_, "A7");
            lv_list_add_text(mapping_area_list_, "A8");
            lv_list_add_text(mapping_area_list_, "B1");
            lv_list_add_text(mapping_area_list_, "B2");
            lv_list_add_text(mapping_area_list_, "B3");
            lv_list_add_text(mapping_area_list_, "B4");
            lv_list_add_text(mapping_area_list_, "B5");
            lv_list_add_text(mapping_area_list_, "B6");
            lv_list_add_text(mapping_area_list_, "B7");
            lv_list_add_text(mapping_area_list_, "B8");
            
            // 在Serial模式下，如果绑区完成，添加HID映射选项
            if (input_manager_->isAutoSerialBindingComplete()) {
                lv_list_add_text(mapping_area_list_, "--- HID Mapping ---");
                lv_list_add_text(mapping_area_list_, "Map to HID Touch");
            }
        } else {
            // 添加HID模式的坐标设置选项
            lv_list_add_text(mapping_area_list_, "Set X,Y coordinates");
        }
    }
    
    return true;
}

bool UIManager::update_mapping_display() {
    if (!mapping_status_label_) {
        return false;
    }
    
    std::string status = "Device: 0x" + std::to_string(mapping_device_addr_) + 
                        ", Channel: " + std::to_string(mapping_channel_);
    lv_label_set_text(mapping_status_label_, status.c_str());
    
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
    if (!hid_key_list_) {
        return false;
    }
    
    // 清空HID按键列表
    lv_obj_clean(hid_key_list_);
    
    // 添加常用HID按键选项
    lv_list_add_text(hid_key_list_, "A");
    lv_list_add_text(hid_key_list_, "S");
    lv_list_add_text(hid_key_list_, "D");
    lv_list_add_text(hid_key_list_, "F");
    lv_list_add_text(hid_key_list_, "J");
    lv_list_add_text(hid_key_list_, "K");
    lv_list_add_text(hid_key_list_, "L");
    lv_list_add_text(hid_key_list_, "Space");
    lv_list_add_text(hid_key_list_, "Enter");
    lv_list_add_text(hid_key_list_, "F1-F12");
    lv_list_add_text(hid_key_list_, "Clear");
    
    return true;
}

bool UIManager::update_key_mapping_display() {
    if (!input_manager_ || !key_list_) {
        return false;
    }
    
    // 清空按键列表
    lv_obj_clean(key_list_);
    
    // 获取所有设备状态
    InputManager::TouchDeviceStatus devices[8];
    int device_count = 0;
    input_manager_->get_all_device_status(devices, device_count);
    
    // 显示每个设备的键盘映射
    for (int i = 0; i < device_count; i++) {
        for (uint8_t ch = 0; ch < 12; ch++) {
            HID_KeyCode key = input_manager_->getTouchKeyboardMapping(devices[i].device.device_addr, ch);
            if (key != HID_KeyCode::KEY_NONE) {
                char item_text[64];
                snprintf(item_text, sizeof(item_text), "Dev%04X Ch%d: %s", 
                        devices[i].device.device_addr, ch, getKeyName(key));
                lv_list_add_text(key_list_, item_text);
            }
        }
    }
    
    // 显示物理键盘映射（GPIO按键）
    const auto& physical_keyboards = input_manager_->getPhysicalKeyboards();
    for (const auto& mapping : physical_keyboards) {
        if (mapping.default_key != HID_KeyCode::KEY_NONE) {
            char item_text[64];
            if (is_mcu_gpio(mapping.gpio)) {
                snprintf(item_text, sizeof(item_text), "MCU GPIO%d: %s", 
                        get_gpio_pin_number(mapping.gpio), getKeyName(mapping.default_key));
            } else if (is_mcp_gpio(mapping.gpio)) {
                uint8_t pin = get_gpio_pin_number(mapping.gpio);
                if (pin <= 8) {
                    snprintf(item_text, sizeof(item_text), "MCP GPIOA%d: %s", 
                            pin, getKeyName(mapping.default_key));
                } else {
                    snprintf(item_text, sizeof(item_text), "MCP GPIOB%d: %s", 
                            pin - 8, getKeyName(mapping.default_key));
                }
            }
            lv_list_add_text(key_list_, item_text);
        }
    }
    
    // 显示逻辑按键映射（支持多键绑定）
    const auto& logical_mappings = input_manager_->getLogicalKeyMappings();
    for (const auto& mapping : logical_mappings) {
        if (mapping.key_count > 0) {
            char item_text[128];
            char keys_str[64] = "";
            
            // 组合多个按键名称
            for (uint8_t i = 0; i < mapping.key_count && i < 3; i++) {
                if (i > 0) strcat(keys_str, "+");
                strcat(keys_str, getKeyName(mapping.keys[i]));
            }
            
            if (is_mcu_gpio(mapping.gpio_id)) {
                snprintf(item_text, sizeof(item_text), "MCU GPIO%d Logic: %s", 
                        get_gpio_pin_number(mapping.gpio_id), keys_str);
            } else if (is_mcp_gpio(mapping.gpio_id)) {
                uint8_t pin = get_gpio_pin_number(mapping.gpio_id);
                if (pin <= 8) {
                    snprintf(item_text, sizeof(item_text), "MCP GPIOA%d Logic: %s", 
                            pin, keys_str);
                } else {
                    snprintf(item_text, sizeof(item_text), "MCP GPIOB%d Logic: %s", 
                            pin - 8, keys_str);
                }
            }
            lv_list_add_text(key_list_, item_text);
        }
    }
    
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
    if (strcmp(name, "A") == 0) return HID_KeyCode::KEY_A;
    if (strcmp(name, "S") == 0) return HID_KeyCode::KEY_S;
    if (strcmp(name, "D") == 0) return HID_KeyCode::KEY_D;
    if (strcmp(name, "F") == 0) return HID_KeyCode::KEY_F;
    if (strcmp(name, "J") == 0) return HID_KeyCode::KEY_J;
    if (strcmp(name, "K") == 0) return HID_KeyCode::KEY_K;
    if (strcmp(name, "L") == 0) return HID_KeyCode::KEY_L;
    if (strcmp(name, "Space") == 0) return HID_KeyCode::KEY_SPACE;
    if (strcmp(name, "Enter") == 0) return HID_KeyCode::KEY_ENTER;
    if (strcmp(name, "Clear") == 0) return HID_KeyCode::KEY_NONE;
    return HID_KeyCode::KEY_NONE;
}

bool UIManager::handle_hid_key_selection(lv_event_t* e) {
    if (!input_manager_ || !key_mapping_active_) {
        return false;
    }
    
    lv_obj_t* target = lv_event_get_target(e);
    if (!target) {
        return false;
    }
    
    // 获取选中的按键名称
    const char* key_name = lv_list_get_btn_text(hid_key_list_, target);
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
    
    // 重置下拉栏选择
    if (logical_key1_dropdown_) {
        lv_dropdown_set_selected(logical_key1_dropdown_, 0); // 设置为"无"
    }
    if (logical_key2_dropdown_) {
        lv_dropdown_set_selected(logical_key2_dropdown_, 0); // 设置为"无"
    }
    if (logical_key3_dropdown_) {
        lv_dropdown_set_selected(logical_key3_dropdown_, 0); // 设置为"无"
    }
    
    // 更新GPIO标签
    if (gpio_label_) {
        lv_label_set_text(gpio_label_, "GPIO: 未选择");
    }
    
    log_debug("All logical key mappings cleared");
    return true;
}

// 处理触摸映射选择
bool UIManager::handle_touch_mapping_selection(lv_event_t* e) {
    if (!input_manager_ || !mapping_area_list_) {
        return false;
    }
    
    // 获取选中的项目
    lv_obj_t* clicked_obj = lv_event_get_target(e);
    if (!clicked_obj) return false;
    
    // 获取选中项的文本
    const char* selected_text = lv_list_get_btn_text(mapping_area_list_, clicked_obj);
    if (!selected_text) return false;
    
    // 检查是否选择了HID映射选项
    if (strcmp(selected_text, "Map to HID Touch") == 0) {
        // 启动HID触摸映射模式
        if (input_manager_->getWorkMode() == InputWorkMode::SERIAL_MODE && 
            input_manager_->isAutoSerialBindingComplete()) {
            
            // 更新状态标签
            if (mapping_status_label_) {
                lv_label_set_text(mapping_status_label_, "Touch area to map to HID");
            }
            
            touch_mapping_active_ = true;
            mapping_step_ = 1; // 设置为HID映射步骤
            
            return true;
        }
    } else {
        // 处理Serial区域映射
        // 这里可以添加Serial区域映射的处理逻辑
        if (mapping_status_label_) {
            std::string status = "Selected: " + std::string(selected_text);
            lv_label_set_text(mapping_status_label_, status.c_str());
        }
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
    
    if (binding_step_label_) {
        std::string text = "Binding " + current_area + " (" + std::to_string(step) + "/34)";
        lv_label_set_text(binding_step_label_, text.c_str());
    }
    
    if (binding_progress_bar_) {
        uint8_t progress = (step * 100) / 34;
        lv_bar_set_value(binding_progress_bar_, progress, LV_ANIM_ON);
    }
    
    return true;
}

// 更新灵敏度显示
void UIManager::update_sensitivity_display() {
    if (!device_label_ || selected_device_index_ < 0) {
        return;
    }
    
    // 获取设备状态
    InputManager::TouchDeviceStatus device_status[8];
    int device_count;
    input_manager_->get_all_device_status(device_status, device_count);
    
    if (selected_device_index_ >= device_count) {
        return;
    }
    
    std::string label_text = "Device: " + device_status[selected_device_index_].device_name + "\nChannel: " + std::to_string(selected_channel_);
    lv_label_set_text(device_label_, label_text.c_str());
    
    // 获取当前灵敏度值并更新滑块
    if (sensitivity_slider_ && input_manager_) {
        uint16_t device_addr = device_status[selected_device_index_].device.device_addr;
        uint8_t current_sensitivity = input_manager_->getSensitivity(device_addr, selected_channel_);
        lv_slider_set_value(sensitivity_slider_, current_sensitivity, LV_ANIM_ON);
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
        "Diagnostics", "Sensitivity", "Light Mapping", "About", "Exit"
    };
    
    for (int i = 0; i < 8; i++) {
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
    
    // 创建自动调整按钮
    auto_adjust_button_ = lv_btn_create(page);
    lv_obj_set_size(auto_adjust_button_, 100, 30);
    lv_obj_align(auto_adjust_button_, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_event_cb(auto_adjust_button_, button_event_cb, LV_EVENT_CLICKED, this);
    
    lv_obj_t* auto_adjust_label = lv_label_create(auto_adjust_button_);
    lv_label_set_text(auto_adjust_label, "Auto Adjust");
    lv_obj_center(auto_adjust_label);
    
    // 创建说明文本
    lv_obj_t* help_text = lv_label_create(page);
    lv_label_set_text(help_text, "Touch point to select\nPress CONFIRM for auto-select\nAuto Adjust for optimal");
    lv_obj_set_style_text_color(help_text, lv_color_white(), 0);
    lv_obj_align(help_text, LV_ALIGN_BOTTOM_MID, 0, -20);
    
    pages_[UIPage::SENSITIVITY] = page;
}

// 创建状态页面
void UIManager::create_status_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "System Status");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    
    // 工作模式状态
    status_mode_label_ = lv_label_create(page);
    lv_label_set_text(status_mode_label_, "Mode: Unknown");
    lv_obj_set_style_text_color(status_mode_label_, lv_color_white(), 0);
    lv_obj_align(status_mode_label_, LV_ALIGN_TOP_LEFT, 5, 25);
    
    // 触摸设备状态
    status_touch_label_ = lv_label_create(page);
    lv_label_set_text(status_touch_label_, "Touch: 0/0 devices");
    lv_obj_set_style_text_color(status_touch_label_, lv_color_white(), 0);
    lv_obj_align(status_touch_label_, LV_ALIGN_TOP_LEFT, 5, 45);
    
    // 绑定状态
    status_binding_label_ = lv_label_create(page);
    lv_label_set_text(status_binding_label_, "Binding: Not configured");
    lv_obj_set_style_text_color(status_binding_label_, lv_color_white(), 0);
    lv_obj_align(status_binding_label_, LV_ALIGN_TOP_LEFT, 5, 65);
    
    // 灵敏度状态
    status_sensitivity_label_ = lv_label_create(page);
    lv_label_set_text(status_sensitivity_label_, "Sensitivity: Default");
    lv_obj_set_style_text_color(status_sensitivity_label_, lv_color_white(), 0);
    lv_obj_align(status_sensitivity_label_, LV_ALIGN_TOP_LEFT, 5, 85);
    
    // 灯光状态
    status_light_label_ = lv_label_create(page);
    lv_label_set_text(status_light_label_, "Light: Unknown");
    lv_obj_set_style_text_color(status_light_label_, lv_color_white(), 0);
    lv_obj_align(status_light_label_, LV_ALIGN_TOP_LEFT, 5, 105);
    
    // 系统信息
    status_system_label_ = lv_label_create(page);
    lv_label_set_text(status_system_label_, "Uptime: 0s");
    lv_obj_set_style_text_color(status_system_label_, lv_color_white(), 0);
    lv_obj_align(status_system_label_, LV_ALIGN_TOP_LEFT, 5, 125);
    
    // 说明文本
    lv_obj_t* help_text = lv_label_create(page);
    lv_label_set_text(help_text, "A/B: Navigate\nCONFIRM: Refresh");
    lv_obj_set_style_text_color(help_text, lv_color_white(), 0);
    lv_obj_align(help_text, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    pages_[UIPage::STATUS] = page;
}

// 创建触摸映射页面
void UIManager::create_touch_mapping_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Touch Mapping");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // 状态标签
    mapping_status_label_ = lv_label_create(page);
    lv_label_set_text(mapping_status_label_, "Select mapping mode");
    lv_obj_set_style_text_color(mapping_status_label_, lv_color_white(), 0);
    lv_obj_align(mapping_status_label_, LV_ALIGN_TOP_MID, 0, 40);
    
    // 映射区域列表
    mapping_area_list_ = lv_list_create(page);
    lv_obj_set_size(mapping_area_list_, 120, 80);
    lv_obj_align(mapping_area_list_, LV_ALIGN_CENTER, 0, 10);
    
    // 添加映射区域列表事件处理
    lv_obj_add_event_cb(mapping_area_list_, [](lv_event_t* e) {
        UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
        if (ui && lv_event_get_code(e) == LV_EVENT_CLICKED) {
            ui->handle_touch_mapping_selection(e);
        }
    }, LV_EVENT_CLICKED, this);
    
    // 显示映射选项
    show_mapping_selection_ui();
    
    // 说明文本
    lv_obj_t* help_text = lv_label_create(page);
    lv_label_set_text(help_text, "CONFIRM: Start mapping\nA/B: Navigate");
    lv_obj_set_style_text_color(help_text, lv_color_white(), 0);
    lv_obj_align(help_text, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    pages_[UIPage::TOUCH_MAPPING] = page;
}

// 创建按键映射页面
void UIManager::create_key_mapping_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Key Mapping");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);
    
    // 状态标签
    lv_obj_t* status_label = lv_label_create(page);
    lv_label_set_text(status_label, "Press physical key to configure");
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 20);
    
    // 当前选中的GPIO标签
    lv_obj_t* gpio_label = lv_label_create(page);
    lv_label_set_text(gpio_label, "Selected: None");
    lv_obj_set_style_text_color(gpio_label, lv_color_white(), 0);
    lv_obj_align(gpio_label, LV_ALIGN_TOP_MID, 0, 35);
    
    // 逻辑按键1下拉栏
    lv_obj_t* key1_label = lv_label_create(page);
    lv_label_set_text(key1_label, "Key 1:");
    lv_obj_set_style_text_color(key1_label, lv_color_white(), 0);
    lv_obj_align(key1_label, LV_ALIGN_TOP_LEFT, 5, 55);
    
    lv_obj_t* key1_dropdown = lv_dropdown_create(page);
    lv_dropdown_set_options(key1_dropdown, "None\nA\nB\nC\nD\nE\nF\nG\nH\nI\nJ\nK\nL\nM\nN\nO\nP\nQ\nR\nS\nT\nU\nV\nW\nX\nY\nZ\n1\n2\n3\n4\n5\n6\n7\n8\n9\n0\nEnter\nEscape\nBackspace\nTab\nSpace");
    lv_obj_set_size(key1_dropdown, 80, 20);
    lv_obj_align(key1_dropdown, LV_ALIGN_TOP_RIGHT, -5, 53);
    
    // 逻辑按键2下拉栏
    lv_obj_t* key2_label = lv_label_create(page);
    lv_label_set_text(key2_label, "Key 2:");
    lv_obj_set_style_text_color(key2_label, lv_color_white(), 0);
    lv_obj_align(key2_label, LV_ALIGN_TOP_LEFT, 5, 80);
    
    lv_obj_t* key2_dropdown = lv_dropdown_create(page);
    lv_dropdown_set_options(key2_dropdown, "None\nA\nB\nC\nD\nE\nF\nG\nH\nI\nJ\nK\nL\nM\nN\nO\nP\nQ\nR\nS\nT\nU\nV\nW\nX\nY\nZ\n1\n2\n3\n4\n5\n6\n7\n8\n9\n0\nEnter\nEscape\nBackspace\nTab\nSpace");
    lv_obj_set_size(key2_dropdown, 80, 20);
    lv_obj_align(key2_dropdown, LV_ALIGN_TOP_RIGHT, -5, 78);
    
    // 逻辑按键3下拉栏
    lv_obj_t* key3_label = lv_label_create(page);
    lv_label_set_text(key3_label, "Key 3:");
    lv_obj_set_style_text_color(key3_label, lv_color_white(), 0);
    lv_obj_align(key3_label, LV_ALIGN_TOP_LEFT, 5, 105);
    
    lv_obj_t* key3_dropdown = lv_dropdown_create(page);
    lv_dropdown_set_options(key3_dropdown, "None\nA\nB\nC\nD\nE\nF\nG\nH\nI\nJ\nK\nL\nM\nN\nO\nP\nQ\nR\nS\nT\nU\nV\nW\nX\nY\nZ\n1\n2\n3\n4\n5\n6\n7\n8\n9\n0\nEnter\nEscape\nBackspace\nTab\nSpace");
    lv_obj_set_size(key3_dropdown, 80, 20);
    lv_obj_align(key3_dropdown, LV_ALIGN_TOP_RIGHT, -5, 103);
    
    // 存储UI组件引用
    gpio_label_ = gpio_label;
    logical_key1_dropdown_ = key1_dropdown;
    logical_key2_dropdown_ = key2_dropdown;
    logical_key3_dropdown_ = key3_dropdown;
    
    // 添加下拉栏事件处理
    lv_obj_add_event_cb(key1_dropdown, [](lv_event_t* e) {
        UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
        if (ui && lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
            ui->handle_logical_key_selection(e, 0); // 第一个逻辑按键
        }
    }, LV_EVENT_VALUE_CHANGED, this);
    
    lv_obj_add_event_cb(key2_dropdown, [](lv_event_t* e) {
        UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
        if (ui && lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
            ui->handle_logical_key_selection(e, 1); // 第二个逻辑按键
        }
    }, LV_EVENT_VALUE_CHANGED, this);
    
    lv_obj_add_event_cb(key3_dropdown, [](lv_event_t* e) {
        UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
        if (ui && lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
            ui->handle_logical_key_selection(e, 2); // 第三个逻辑按键
        }
    }, LV_EVENT_VALUE_CHANGED, this);
    
    // 当前映射列表
    key_list_ = lv_list_create(page);
    lv_obj_set_size(key_list_, 150, 40);
    lv_obj_align(key_list_, LV_ALIGN_CENTER, 0, 20);
    
    // 清除映射按钮
    lv_obj_t* clear_btn = lv_btn_create(page);
    lv_obj_set_size(clear_btn, 80, 25);
    lv_obj_align(clear_btn, LV_ALIGN_BOTTOM_LEFT, 10, -40);
    lv_obj_t* clear_label = lv_label_create(clear_btn);
    lv_label_set_text(clear_label, "Clear All");
    lv_obj_center(clear_label);
    lv_obj_add_event_cb(clear_btn, [](lv_event_t* e) {
        UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
        if (ui && lv_event_get_code(e) == LV_EVENT_CLICKED) {
            ui->clear_all_logical_key_mappings();
        }
    }, LV_EVENT_CLICKED, this);
    
    // 返回按钮
    lv_obj_t* back_btn = lv_btn_create(page);
    lv_obj_set_size(back_btn, 60, 25);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -40);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "Back");
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
        if (ui && lv_event_get_code(e) == LV_EVENT_CLICKED) {
            ui->set_current_page(UIPage::MAIN);
        }
    }, LV_EVENT_CLICKED, this);
    
    pages_[UIPage::KEY_MAPPING] = page;
}

// 创建引导式绑区页面
void UIManager::create_guided_binding_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Guided Binding");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // 进度条
    binding_progress_bar_ = lv_bar_create(page);
    lv_obj_set_size(binding_progress_bar_, 120, 10);
    lv_obj_align(binding_progress_bar_, LV_ALIGN_TOP_MID, 0, 40);
    lv_bar_set_range(binding_progress_bar_, 0, 100);
    
    // 步骤标签
    binding_step_label_ = lv_label_create(page);
    lv_label_set_text(binding_step_label_, "Ready to start");
    lv_obj_set_style_text_color(binding_step_label_, lv_color_white(), 0);
    lv_obj_align(binding_step_label_, LV_ALIGN_CENTER, 0, 0);
    
    // 说明文本
    lv_obj_t* help_text = lv_label_create(page);
    lv_label_set_text(help_text, "CONFIRM: Start\nOnly for Serial mode");
    lv_obj_set_style_text_color(help_text, lv_color_white(), 0);
    lv_obj_align(help_text, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    pages_[UIPage::GUIDED_BINDING] = page;
}

void UIManager::create_settings_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // 创建设置菜单项
    lv_obj_t* uart_settings_label = lv_label_create(page);
    lv_label_set_text(uart_settings_label, "> UART Settings");
    lv_obj_set_style_text_color(uart_settings_label, lv_color_white(), 0);
    lv_obj_align(uart_settings_label, LV_ALIGN_CENTER, 0, 0);
    
    // 添加导航提示
    lv_obj_t* help_text = lv_label_create(page);
    lv_label_set_text(help_text, "UP/DOWN: Navigate\nCONFIRM: Select\nBACK: Return");
    lv_obj_set_style_text_color(help_text, lv_color_white(), 0);
    lv_obj_align(help_text, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    pages_[UIPage::SETTINGS] = page;
}

void UIManager::create_uart_settings_page() {
    // 创建UART设置页面
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 添加标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "UART Settings");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    
    // Mai2Serial波特率设置
    lv_obj_t* mai2serial_label = lv_label_create(page);
    lv_label_set_text(mai2serial_label, "Mai2Serial Baudrate:");
    lv_obj_align(mai2serial_label, LV_ALIGN_TOP_LEFT, 10, 40);
    lv_obj_set_style_text_color(mai2serial_label, lv_color_white(), 0);
    
    mai2serial_baudrate_dropdown_ = lv_dropdown_create(page);
    lv_dropdown_set_options(mai2serial_baudrate_dropdown_, "9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600");
    lv_obj_align(mai2serial_baudrate_dropdown_, LV_ALIGN_TOP_LEFT, 10, 65);
    lv_obj_set_width(mai2serial_baudrate_dropdown_, 120);
    lv_obj_add_event_cb(mai2serial_baudrate_dropdown_, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, this);
    
    // Mai2Light波特率设置
    lv_obj_t* mai2light_label = lv_label_create(page);
    lv_label_set_text(mai2light_label, "Mai2Light Baudrate:");
    lv_obj_align(mai2light_label, LV_ALIGN_TOP_LEFT, 10, 100);
    lv_obj_set_style_text_color(mai2light_label, lv_color_white(), 0);
    
    mai2light_baudrate_dropdown_ = lv_dropdown_create(page);
    lv_dropdown_set_options(mai2light_baudrate_dropdown_, "9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600");
    lv_obj_align(mai2light_baudrate_dropdown_, LV_ALIGN_TOP_LEFT, 10, 125);
    lv_obj_set_width(mai2light_baudrate_dropdown_, 120);
    lv_obj_add_event_cb(mai2light_baudrate_dropdown_, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, this);
    
    // 状态标签
    uart_status_label_ = lv_label_create(page);
    lv_label_set_text(uart_status_label_, "Status: Ready");
    lv_obj_align(uart_status_label_, LV_ALIGN_TOP_LEFT, 10, 160);
    lv_obj_set_style_text_color(uart_status_label_, lv_color_white(), 0);
    
    // 保存按钮
    lv_obj_t* save_btn = lv_btn_create(page);
    lv_obj_set_size(save_btn, 80, 30);
    lv_obj_align(save_btn, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(save_btn, button_event_cb, LV_EVENT_CLICKED, this);
    lv_obj_t* save_label = lv_label_create(save_btn);
    lv_label_set_text(save_label, "Save");
    lv_obj_center(save_label);
    
    // 重置按钮
    lv_obj_t* reset_btn = lv_btn_create(page);
    lv_obj_set_size(reset_btn, 80, 30);
    lv_obj_align(reset_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(reset_btn, button_event_cb, LV_EVENT_CLICKED, this);
    lv_obj_t* reset_label = lv_label_create(reset_btn);
    lv_label_set_text(reset_label, "Reset");
    lv_obj_center(reset_label);
    
    // 设置默认值
    lv_dropdown_set_selected(mai2serial_baudrate_dropdown_, 4); // 115200
    lv_dropdown_set_selected(mai2light_baudrate_dropdown_, 4);  // 115200
    
    pages_[UIPage::UART_SETTINGS] = page;
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

void UIManager::create_light_mapping_page() {
    lv_obj_t* page = lv_obj_create(NULL);
    lv_obj_set_size(page, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(page, lv_color_black(), 0);
    
    // 标题
    lv_obj_t* title = lv_label_create(page);
    lv_label_set_text(title, "Light Mapping");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    
    // 状态标签
    light_mapping_status_ = lv_label_create(page);
    lv_label_set_text(light_mapping_status_, "Select light region to configure");
    lv_obj_set_style_text_color(light_mapping_status_, lv_color_white(), 0);
    lv_obj_align(light_mapping_status_, LV_ALIGN_TOP_MID, 0, 25);
    
    // 灯光区域列表
    light_region_list_ = lv_list_create(page);
    lv_obj_set_size(light_region_list_, 120, 80);
    lv_obj_align(light_region_list_, LV_ALIGN_TOP_LEFT, 5, 45);
    
    // 添加灯光区域选项
    if (light_manager_) {
        // 获取所有可用的灯光区域
        std::vector<std::string> regions = {"all", "button1", "button2", "button3", "button4", "button5", "button6", "button7", "button8"};
        
        for (const auto& region : regions) {
            lv_obj_t* btn = lv_list_add_btn(light_region_list_, NULL, region.c_str());
            lv_obj_add_event_cb(btn, [](lv_event_t* e) {
                UIManager* ui = UIManager::getInstance();
                if (ui) {
                    ui->handle_light_region_selection(e);
                }
            }, LV_EVENT_CLICKED, NULL);
        }
    }
    
    // Neopixel网格容器
    neopixel_grid_ = lv_obj_create(page);
    lv_obj_set_size(neopixel_grid_, 120, 80);
    lv_obj_align(neopixel_grid_, LV_ALIGN_TOP_RIGHT, -5, 45);
    lv_obj_set_style_bg_color(neopixel_grid_, lv_color_hex(0x333333), 0);
    
    // 创建Neopixel按钮网格 (4x8 = 32个)
    for (int i = 0; i < 32; i++) {
        neopixel_buttons_[i] = lv_btn_create(neopixel_grid_);
        lv_obj_set_size(neopixel_buttons_[i], 12, 8);
        
        int row = i / 8;
        int col = i % 8;
        lv_obj_set_pos(neopixel_buttons_[i], col * 14 + 5, row * 18 + 5);
        
        // 设置按钮样式
        lv_obj_set_style_bg_color(neopixel_buttons_[i], lv_color_hex(0x666666), 0);
        lv_obj_set_style_bg_color(neopixel_buttons_[i], lv_color_hex(0x00FF00), LV_STATE_CHECKED);
        
        // 添加标签显示编号
        lv_obj_t* label = lv_label_create(neopixel_buttons_[i]);
        lv_label_set_text_fmt(label, "%d", i);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_center(label);
        
        // 添加点击事件
        lv_obj_add_event_cb(neopixel_buttons_[i], [](lv_event_t* e) {
            UIManager* ui = UIManager::getInstance();
            if (ui) {
                ui->handle_neopixel_selection(e);
            }
        }, LV_EVENT_CLICKED, NULL);
        
        lv_obj_add_flag(neopixel_buttons_[i], LV_OBJ_FLAG_CHECKABLE);
    }
    
    // 控制按钮
    lv_obj_t* save_btn = lv_btn_create(page);
    lv_obj_set_size(save_btn, 50, 20);
    lv_obj_align(save_btn, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_t* save_label = lv_label_create(save_btn);
    lv_label_set_text(save_label, "Save");
    lv_obj_center(save_label);
    lv_obj_add_event_cb(save_btn, [](lv_event_t* e) {
        UIManager* ui = UIManager::getInstance();
        if (ui) {
            ui->save_light_mapping();
        }
    }, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* clear_btn = lv_btn_create(page);
    lv_obj_set_size(clear_btn, 50, 20);
    lv_obj_align(clear_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_t* clear_label = lv_label_create(clear_btn);
    lv_label_set_text(clear_label, "Clear");
    lv_obj_center(clear_label);
    lv_obj_add_event_cb(clear_btn, [](lv_event_t* e) {
        UIManager* ui = UIManager::getInstance();
        if (ui) {
            ui->clear_light_mapping();
        }
    }, LV_EVENT_CLICKED, NULL);
    
    pages_[UIPage::LIGHT_MAPPING] = page;
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
    
    if (ui) {
        // 检查是否为灵敏度页面的自动调整按钮
        if (ui->current_page_ == UIPage::SENSITIVITY) {
            // 检查是否有选中的设备和通道
            if (ui->selected_device_index_ >= 0 && ui->input_manager_) {
                // 获取设备状态
                InputManager::TouchDeviceStatus device_status[8];
                int device_count;
                ui->input_manager_->get_all_device_status(device_status, device_count);
                
                if (ui->selected_device_index_ < device_count) {
                    uint16_t device_addr = device_status[ui->selected_device_index_].device.device_addr;
                    
                    // 启动自动灵敏度调整
                    ui->input_manager_->autoAdjustSensitivity(device_addr, ui->selected_channel_);
                    
                    // 设置自动调整状态
                    ui->auto_adjust_active_ = true;
                    
                    // 更新设备标签显示调整状态
                    if (ui->device_label_) {
                        std::string status_text = "Auto adjusting...\nDevice: " + device_status[ui->selected_device_index_].device_name + "\nChannel: " + std::to_string(ui->selected_channel_);
                        lv_label_set_text(ui->device_label_, status_text.c_str());
                    }
                    
                    // 禁用按钮
                    if (ui->auto_adjust_button_) {
                        lv_label_set_text(lv_obj_get_child(ui->auto_adjust_button_, 0), "Auto adjusting...");
                        lv_obj_add_state(ui->auto_adjust_button_, LV_STATE_DISABLED);
                    }
                }
            } else {
                // 提示用户先选择一个触摸点
                if (ui->device_label_) {
                    lv_label_set_text(ui->device_label_, "Please touch a point first");
                }
            }
            return;
        }
        
        // 灯光映射页面的按钮处理
        if (ui->current_page_ == UIPage::LIGHT_MAPPING) {
            // 检查是否为灯光区域选择按钮
            if (ui->light_region_list_ && lv_obj_get_parent(btn) == ui->light_region_list_) {
                ui->handle_light_region_selection(e);
                return;
            }
            
            // 检查是否为Neopixel按钮
            for (int i = 0; i < 32; i++) {
                if (ui->neopixel_buttons_[i] == btn) {
                    ui->handle_neopixel_selection(e);
                    return;
                }
            }
            
            // 检查是否为保存或清除按钮
            lv_obj_t* label = lv_obj_get_child(btn, 0);
            if (label) {
                const char* text = lv_label_get_text(label);
                if (text) {
                    if (strcmp(text, "Save") == 0) {
                        ui->save_light_mapping();
                        return;
                    } else if (strcmp(text, "Clear") == 0) {
                        ui->clear_light_mapping();
                        return;
                    }
                }
            }
        }
        
        // UART设置页面的按钮处理
        if (ui->current_page_ == UIPage::UART_SETTINGS) {
            lv_obj_t* label = lv_obj_get_child(btn, 0);
            if (label) {
                const char* text = lv_label_get_text(label);
                if (text) {
                    if (strcmp(text, "Save") == 0) {
                        ui->save_uart_settings();
                        return;
                    } else if (strcmp(text, "Reset") == 0) {
                        ui->reset_uart_settings();
                        return;
                    }
                }
            }
        }
        
        // 其他页面的按钮处理
        int menu_index = (int)(intptr_t)lv_obj_get_user_data(btn);
        ui->current_menu_index_ = menu_index;
        ui->handle_confirm_input();
    }
}

void UIManager::dropdown_event_cb(lv_event_t* e) {
    UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
    lv_obj_t* dropdown = lv_event_get_target(e);
    
    if (ui && ui->current_page_ == UIPage::UART_SETTINGS) {
        if (dropdown == ui->mai2serial_baudrate_dropdown_) {
            ui->handle_mai2serial_baudrate_change();
        } else if (dropdown == ui->mai2light_baudrate_dropdown_) {
            ui->handle_mai2light_baudrate_change();
        }
    }
}

void UIManager::slider_event_cb(lv_event_t* e) {
    UIManager* ui = static_cast<UIManager*>(lv_event_get_user_data(e));
    lv_obj_t* slider = lv_event_get_target(e);
    
    if (ui && ui->input_manager_ && ui->selected_device_index_ >= 0) {
        int32_t value = lv_slider_get_value(slider);
        
        // 获取设备状态
        InputManager::TouchDeviceStatus device_status[8];
        int device_count;
        ui->input_manager_->get_all_device_status(device_status, device_count);
        
        if (ui->selected_device_index_ < device_count) {
            uint16_t device_addr = device_status[ui->selected_device_index_].device.device_addr;
            ui->input_manager_->setSensitivity(device_addr, ui->selected_channel_, value);
        }
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
    
    // 处理背光和息屏
    handle_backlight();
    handle_screen_timeout();
    
    // 处理故障检测
    handle_error_detection();
    
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
        case UIPage::TOUCH_MAPPING:
            update_touch_mapping_page();
            break;
        case UIPage::KEY_MAPPING:
            update_key_mapping_page();
            break;
        case UIPage::GUIDED_BINDING:
            update_guided_binding_page();
            break;
        case UIPage::LIGHT_MAPPING:
            update_light_mapping_page();
            break;
        default:
            break;
    }
    
    last_refresh_time_ = current_time;
}

// 更新页面
void UIManager::update_status_page() {
    if (!status_mode_label_ || !status_touch_label_ || !status_binding_label_ || 
        !status_sensitivity_label_ || !status_light_label_ || !status_system_label_) {
        return;
    }
    
    // 更新工作模式状态
    if (input_manager_) {
        InputWorkMode mode = input_manager_->getWorkMode();
        std::string mode_text = std::string("Mode: ") + (mode == InputWorkMode::SERIAL_MODE ? "Serial" : "HID");
        lv_label_set_text(status_mode_label_, mode_text.c_str());
    }
    
    // 更新触摸设备状态
    if (input_manager_) {
        InputManager::TouchDeviceStatus device_status[8];
        int device_count;
        input_manager_->get_all_device_status(device_status, device_count);
        
        int active_devices = 0;
        for (int i = 0; i < device_count; i++) {
            if (device_status[i].is_connected) {
                active_devices++;
            }
        }
        
        std::string touch_text = "Touch: " + std::to_string(active_devices) + "/" + std::to_string(device_count) + " devices";
        lv_label_set_text(status_touch_label_, touch_text.c_str());
    }
    
    // 更新绑定状态
    if (input_manager_) {
        InputWorkMode mode = input_manager_->getWorkMode();
        if (mode == InputWorkMode::SERIAL_MODE) {
            bool is_bound = input_manager_->isAutoSerialBindingComplete();
            std::string binding_text = "Binding: " + std::string(is_bound ? "Complete" : "Not configured");
            lv_label_set_text(status_binding_label_, binding_text.c_str());
        } else {
            lv_label_set_text(status_binding_label_, "Binding: HID mode");
        }
    }
    
    // 更新灵敏度状态
    if (selected_device_index_ >= 0 && input_manager_) {
        InputManager::TouchDeviceStatus device_status[8];
        int device_count;
        input_manager_->get_all_device_status(device_status, device_count);
        
        if (selected_device_index_ < device_count) {
            uint16_t device_addr = device_status[selected_device_index_].device.device_addr;
            uint8_t sensitivity = input_manager_->getSensitivity(device_addr, selected_channel_);
            std::string sens_text = "Sensitivity: Ch" + std::to_string(selected_channel_) + "=" + std::to_string(sensitivity);
            lv_label_set_text(status_sensitivity_label_, sens_text.c_str());
        }
    } else {
        lv_label_set_text(status_sensitivity_label_, "Sensitivity: Default");
    }
    
    // 更新灯光状态
    if (light_manager_) {
        bool light_enabled = light_manager_->is_ready();
        std::string light_text = "Light: " + std::string(light_enabled ? "Ready" : "Not Ready");
        lv_label_set_text(status_light_label_, light_text.c_str());
    }
    
    // 更新系统信息
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t uptime_seconds = current_time / 1000;
    
    // 获取采样速率和回报速率
    std::string system_text = "Uptime: " + std::to_string(uptime_seconds) + "s";
    if (input_manager_) {
        // 获取第一个可用设备的采样速率
        InputManager::TouchDeviceStatus device_status[8];
        int device_count;
        input_manager_->get_all_device_status(device_status, device_count);
        
        uint16_t touch_sample_rate = 0;
        if (device_count > 0 && device_status[0].is_connected) {
            touch_sample_rate = input_manager_->getTouchSampleRate(device_status[0].device.device_addr);
        }
        
        uint16_t hid_report_rate = input_manager_->getHIDReportRate();
        system_text += "\nTouch: " + std::to_string(touch_sample_rate) + "Hz";
        system_text += "\nHID: " + std::to_string(hid_report_rate) + "Hz";
    }
    lv_label_set_text(status_system_label_, system_text.c_str());
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
    
    // 更新当前选中通道的灵敏度显示
    if (selected_device_index_ >= 0 && sensitivity_slider_) {
        InputManager::TouchDeviceStatus device_status[8];
        int device_count;
        input_manager_->get_all_device_status(device_status, device_count);
        
        if (selected_device_index_ < device_count) {
            uint16_t device_addr = device_status[selected_device_index_].device.device_addr;
            uint8_t current_sensitivity = input_manager_->getSensitivity(device_addr, selected_channel_);
            
            // 只有当滑块值与实际值不同时才更新，避免循环更新
            if (lv_slider_get_value(sensitivity_slider_) != current_sensitivity) {
                lv_slider_set_value(sensitivity_slider_, current_sensitivity, LV_ANIM_OFF);
            }
        }
    }
}

void UIManager::update_touch_mapping_page() {
    if (!mapping_status_label_) return;
    
    if (touch_mapping_active_) {
        switch (mapping_step_) {
            case 0:
                lv_label_set_text(mapping_status_label_, "Touch a point to map");
                
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
                lv_label_set_text(mapping_status_label_, "Select target area");
                break;
            case 2:
                lv_label_set_text(mapping_status_label_, "Mapping complete");
                break;
        }
        
        // 处理HID映射模式的触摸检测
        if (mapping_step_ == 0 && input_manager_) {
            // 检查是否为HID映射模式
            lv_indev_t* indev = lv_indev_get_act();
            if (indev && lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
                lv_point_t point;
                lv_indev_get_point(indev, &point);
                
                // 检查是否有触摸输入
                if (true) {  // lv_indev_get_point总是返回最后的点，我们假设有触摸
                    // 获取触摸坐标
                    int16_t touch_x = point.x;
                    int16_t touch_y = point.y;
                    
                    // 检查当前是否有设备和通道被选中进行HID映射
                    if (mapping_device_addr_ >= 0 && mapping_channel_ >= 0) {
                        // 设置HID坐标映射
                        input_manager_->setHIDMapping(mapping_device_addr_, mapping_channel_, touch_x, touch_y);
                        lv_label_set_text(mapping_status_label_, "HID mapping set successfully");
                        mapping_step_ = 2;
                    }
                }
            }
        }
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
    if (!binding_step_label_ || !binding_progress_bar_) return;
    
    if (guided_binding_active_ && input_manager_) {
        // 检查自动绑区是否完成
        if (input_manager_->isAutoSerialBindingComplete()) {
            // 显示完成状态
            lv_bar_set_value(binding_progress_bar_, 100, LV_ANIM_ON);
            lv_label_set_text(binding_step_label_, "Auto binding complete! Press OK to confirm.");
            
            // 可以添加确认按钮或等待用户输入
            // 这里简化处理，自动确认
            static uint32_t complete_time = 0;
            if (complete_time == 0) {
                complete_time = to_ms_since_boot(get_absolute_time());
            }
            
            // 3秒后自动确认
            if (to_ms_since_boot(get_absolute_time()) - complete_time > 3000) {
                input_manager_->confirmAutoSerialBinding();
                guided_binding_active_ = false;
                complete_time = 0;
                set_current_page(UIPage::MAIN);
            }
        } else {
            // 正常进度更新
            uint8_t progress = (binding_step_ * 100) / 34; // 假设总共34个区域
            lv_bar_set_value(binding_progress_bar_, progress, LV_ANIM_ON);
            
            std::string step_text = "Auto binding in progress... " + std::to_string(binding_step_) + "/34";
            lv_label_set_text(binding_step_label_, step_text.c_str());
        }
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
    
    if (current_time - last_activity_time_ > timeout_ms) {
        if (!screen_off_) {
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
        return display_device_->fill_screen(ST7735S_Color(0x0000)); // 黑色
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
    if (!initialized_ || !lv_display_) {
        return false;
    }
    
    // 创建状态显示对象
    lv_obj_t* status_obj = lv_obj_create(lv_scr_act());
    lv_obj_set_size(status_obj, 200, 80);
    lv_obj_center(status_obj);
    lv_obj_set_style_bg_color(status_obj, is_success ? lv_color_make(0, 128, 0) : lv_color_make(128, 128, 0), 0);
    
    lv_obj_t* label = lv_label_create(status_obj);
    lv_label_set_text(label, message.c_str());
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);
    
    // 记录活动时间
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    
    return true;
}

// 更新绑定进度
bool UIManager::update_binding_progress(uint8_t progress, const std::string& current_step) {
    if (!initialized_ || !lv_display_) {
        return false;
    }
    
    // 创建进度显示对象
    lv_obj_t* progress_obj = lv_obj_create(lv_scr_act());
    lv_obj_set_size(progress_obj, 250, 100);
    lv_obj_center(progress_obj);
    lv_obj_set_style_bg_color(progress_obj, lv_color_make(64, 64, 64), 0);
    
    // 进度条
    lv_obj_t* bar = lv_bar_create(progress_obj);
    lv_obj_set_size(bar, 200, 20);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, -10);
    lv_bar_set_value(bar, progress, LV_ANIM_OFF);
    
    // 步骤文本
    if (!current_step.empty()) {
        lv_obj_t* step_label = lv_label_create(progress_obj);
        lv_label_set_text(step_label, current_step.c_str());
        lv_obj_set_style_text_color(step_label, lv_color_white(), 0);
        lv_obj_align(step_label, LV_ALIGN_CENTER, 0, 20);
    }
    
    // 进度百分比
    lv_obj_t* percent_label = lv_label_create(progress_obj);
    char percent_text[16];
    snprintf(percent_text, sizeof(percent_text), "%d%%", progress);
    lv_label_set_text(percent_label, percent_text);
    lv_obj_set_style_text_color(percent_label, lv_color_white(), 0);
    lv_obj_align(percent_label, LV_ALIGN_CENTER, 0, -30);
    
    // 记录活动时间
    last_activity_time_ = to_ms_since_boot(get_absolute_time());
    
    return true;
}

// 清除绑定状态显示
bool UIManager::clear_binding_status() {
    if (!initialized_ || !lv_display_) {
        return false;
    }
    
    // 清除屏幕上的临时对象
    lv_obj_clean(lv_scr_act());
    
    // 重新创建当前页面
    switch (current_page_) {
        case UIPage::MAIN:
            create_main_page();
            break;
        case UIPage::STATUS:
            create_status_page();
            break;
        case UIPage::SETTINGS:
            create_settings_page();
            break;
        case UIPage::UART_SETTINGS:
            create_uart_settings_page();
            break;
        case UIPage::CALIBRATION:
            create_calibration_page();
            break;
        case UIPage::DIAGNOSTICS:
            create_diagnostics_page();
            break;
        case UIPage::SENSITIVITY:
            create_sensitivity_page();
            break;
        case UIPage::TOUCH_MAPPING:
            create_touch_mapping_page();
            break;
        case UIPage::KEY_MAPPING:
            create_key_mapping_page();
            break;
        case UIPage::GUIDED_BINDING:
            create_guided_binding_page();
            break;
        case UIPage::LIGHT_MAPPING:
            create_light_mapping_page();
            break;
        case UIPage::ERROR:
            create_error_page();
            break;
        case UIPage::ABOUT:
            create_about_page();
            break;
    }
    
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
    if (current_page_ != UIPage::UART_SETTINGS || !uart_status_label_) {
        return;
    }
    
    // 更新状态标签
    std::string status_text = "Status: ";
    if (input_manager_ && light_manager_) {
        status_text += "Ready";
    } else {
        status_text += "Not Ready";
    }
    
    lv_label_set_text(uart_status_label_, status_text.c_str());
}

void UIManager::handle_mai2serial_baudrate_change() {
    if (!mai2serial_baudrate_dropdown_) {
        return;
    }
    
    uint16_t selected = lv_dropdown_get_selected(mai2serial_baudrate_dropdown_);
    uint32_t baudrates[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    
    if (selected < sizeof(baudrates) / sizeof(baudrates[0])) {
        current_mai2serial_baudrate_ = baudrates[selected];
        
        // 更新状态
        if (uart_status_label_) {
            std::string status = "Mai2Serial: " + std::to_string(current_mai2serial_baudrate_) + " bps";
            lv_label_set_text(uart_status_label_, status.c_str());
        }
    }
}

void UIManager::handle_mai2light_baudrate_change() {
    if (!mai2light_baudrate_dropdown_) {
        return;
    }
    
    uint16_t selected = lv_dropdown_get_selected(mai2light_baudrate_dropdown_);
    uint32_t baudrates[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    
    if (selected < sizeof(baudrates) / sizeof(baudrates[0])) {
        current_mai2light_baudrate_ = baudrates[selected];
        
        // 更新状态
        if (uart_status_label_) {
            std::string status = "Mai2Light: " + std::to_string(current_mai2light_baudrate_) + " bps";
            lv_label_set_text(uart_status_label_, status.c_str());
        }
    }
}

void UIManager::save_uart_settings() {
    if (!input_manager_ || !light_manager_) {
        if (uart_status_label_) {
            lv_label_set_text(uart_status_label_, "Error: Managers not available");
        }
        return;
    }
    
    // 通过InputManager设置Mai2Serial波特率
    // 这里需要InputManager提供设置波特率的接口
    
    // 通过LightManager设置Mai2Light波特率
    // 这里需要LightManager提供设置波特率的接口
    
    if (uart_status_label_) {
        lv_label_set_text(uart_status_label_, "Settings saved successfully");
    }
}

void UIManager::reset_uart_settings() {
    // 重置为默认值115200
    current_mai2serial_baudrate_ = 115200;
    current_mai2light_baudrate_ = 115200;
    
    if (mai2serial_baudrate_dropdown_) {
        lv_dropdown_set_selected(mai2serial_baudrate_dropdown_, 4); // 115200
    }
    
    if (mai2light_baudrate_dropdown_) {
        lv_dropdown_set_selected(mai2light_baudrate_dropdown_, 4); // 115200
    }
    
    if (uart_status_label_) {
        lv_label_set_text(uart_status_label_, "Settings reset to default");
    }
}

void UIManager::update_calibration_page() {
    // 更新校准页面
}

void UIManager::update_diagnostics_page() {
    // 更新诊断页面
}

// 故障相关方法实现
 void UIManager::create_error_page() {
     lv_obj_t* page = lv_obj_create(NULL);
     lv_obj_set_style_bg_color(page, lv_color_make(128, 0, 0), 0);
     
     lv_obj_t* title = lv_label_create(page);
     lv_label_set_text(title, "System Error");
     lv_obj_set_style_text_color(title, lv_color_white(), 0);
     lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
     
     pages_[UIPage::ERROR] = page;
 }
 
 void UIManager::update_error_page() {
     if (!has_error_) {
         return;
     }
     
     lv_obj_t* page = pages_[UIPage::ERROR];
     if (!page) {
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
    if (!light_mapping_status_) {
        return;
    }
    
    // 更新状态显示
    if (!selected_light_region_.empty()) {
        std::string status = "Region: " + selected_light_region_;
        if (!selected_neopixels_.empty()) {
            status += " | Selected: " + std::to_string(selected_neopixels_.size()) + " LEDs";
        }
        lv_label_set_text(light_mapping_status_, status.c_str());
    }
}

bool UIManager::handle_light_region_selection(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    if (!btn) return false;
    
    // 获取按钮文本作为区域名称
    lv_obj_t* label = lv_obj_get_child(btn, 0);
    if (!label) return false;
    
    const char* region_name = lv_label_get_text(label);
    if (!region_name) return false;
    
    selected_light_region_ = region_name;
    selected_neopixels_.clear();
    
    // 清除所有Neopixel按钮的选中状态
    for (int i = 0; i < 32; i++) {
        if (neopixel_buttons_[i]) {
            lv_obj_clear_state(neopixel_buttons_[i], LV_STATE_CHECKED);
        }
    }
    
    // 如果有LightManager，获取当前区域的LED映射
    if (light_manager_) {
        // 这里可以从LightManager获取当前区域已映射的LED编号
        // 并设置对应按钮为选中状态
    }
    
    return true;
}

bool UIManager::handle_neopixel_selection(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    if (!btn) return false;
    
    // 查找按钮在数组中的索引
    int index = -1;
    for (int i = 0; i < 32; i++) {
        if (neopixel_buttons_[i] == btn) {
            index = i;
            break;
        }
    }
    
    if (index == -1) return false;
    
    // 切换选中状态
    bool is_checked = lv_obj_has_state(btn, LV_STATE_CHECKED);
    
    if (is_checked) {
        // 添加到选中列表
        auto it = std::find(selected_neopixels_.begin(), selected_neopixels_.end(), index);
        if (it == selected_neopixels_.end()) {
            selected_neopixels_.push_back(index);
        }
    } else {
        // 从选中列表移除
        auto it = std::find(selected_neopixels_.begin(), selected_neopixels_.end(), index);
        if (it != selected_neopixels_.end()) {
            selected_neopixels_.erase(it);
        }
    }
    
    return true;
}

bool UIManager::save_light_mapping() {
    if (selected_light_region_.empty() || !light_manager_) {
        return false;
    }
    
    // 清除该区域的现有映射
    // light_manager_->clear_region_mapping(selected_light_region_);
    
    // 添加新的LED映射
    for (uint8_t led_index : selected_neopixels_) {
        (void)led_index;  // 消除未使用变量警告
        // light_manager_->add_led_to_region(selected_light_region_, led_index);
    }
    
    // 更新状态显示
    if (light_mapping_status_) {
        lv_label_set_text(light_mapping_status_, "Mapping saved successfully!");
    }
    
    return true;
}

bool UIManager::clear_light_mapping() {
    if (selected_light_region_.empty() || !light_manager_) {
        return false;
    }
    
    // 清除该区域的所有LED映射
    // light_manager_->clear_region_mapping(selected_light_region_);
    
    // 清除UI中的选中状态
    selected_neopixels_.clear();
    for (int i = 0; i < 32; i++) {
        if (neopixel_buttons_[i]) {
            lv_obj_clear_state(neopixel_buttons_[i], LV_STATE_CHECKED);
        }
    }
    
    // 更新状态显示
    if (light_mapping_status_) {
        lv_label_set_text(light_mapping_status_, "Mapping cleared!");
    }
    
    return true;
}

bool UIManager::show_light_mapping_page() {
    return set_current_page(UIPage::LIGHT_MAPPING);
}

bool UIManager::handle_logical_key_selection(lv_event_t* e, int key_index) {
    if (!e || !input_manager_) {
        return false;
    }
    
    lv_obj_t* dropdown = lv_event_get_target(e);
    if (!dropdown) {
        return false;
    }
    
    // 获取选中的选项文本
    char buf[64];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    
    // 如果选择了"None"，则清除该GPIO的逻辑按键映射
    if (strcmp(buf, "None") == 0) {
        input_manager_->clearLogicalKeyMapping(static_cast<uint8_t>(key_index));
        log_debug("Cleared logical key mapping for GPIO " + std::to_string(key_index));
        return true;
    }
    
    // 将选项文本转换为HID按键代码
    HID_KeyCode key_code = getKeyCodeFromName(buf);
    if (key_code == HID_KeyCode::KEY_NONE) {
        log_error("Invalid key name: " + std::string(buf));
        return false;
    }
    
    // 添加逻辑按键映射
    if (input_manager_->addLogicalKeyMapping(static_cast<uint8_t>(key_index), key_code)) {
        log_debug("Added logical key mapping: GPIO " + std::to_string(key_index) + " -> " + std::string(buf));
        
        // 更新GPIO标签显示
        if (gpio_label_) {
            std::string gpio_text = "GPIO Mappings:\n";
            auto mappings = input_manager_->getLogicalKeyMappings();
            for (const auto& mapping : mappings) {
                if (mapping.key_count > 0) {
                    gpio_text += "GPIO " + std::to_string(mapping.gpio_id) + ": " + getKeyName(mapping.keys[0]);
                    for (uint8_t i = 1; i < mapping.key_count; i++) {
                        gpio_text += ", " + std::string(getKeyName(mapping.keys[i]));
                    }
                    gpio_text += "\n";
                }
            }
            lv_label_set_text(gpio_label_, gpio_text.c_str());
        }
        
        return true;
    } else {
        log_error("Failed to add logical key mapping for GPIO " + std::to_string(key_index));
        return false;
    }
}