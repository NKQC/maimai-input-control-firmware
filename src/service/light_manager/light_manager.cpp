#include "light_manager.h"
#include "pico/time.h"
#include "../../protocol/usb_serial_logs/usb_serial_logs.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cmath>

// 静态实例变量定义
LightManager* LightManager::instance_ = nullptr;

// 静态配置变量
static LightManager_PrivateConfig light_manager_private_config;

// 纯公开函数实现
LightManager_Config* light_manager_get_config_holder() {
    static LightManager_Config config_holder;
    return &config_holder;
}

bool light_manager_load_config_from_manager(LightManager_Config* config) {
    if (!config) return false;
    
    // 从LightManager实例加载配置到私有配置
    LightManager* manager = LightManager::getInstance();
    if (!manager) return false;
    
    // 这里应该从manager加载配置到light_manager_private_config
    // 目前先返回true，实际实现需要根据具体需求
    return true;
}

LightManager_PrivateConfig* light_manager_get_config_copy() {
    return &light_manager_private_config;
}

bool light_manager_write_config_to_manager(const LightManager_Config* config) {
    if (!config) return false;
    
    // 将配置写入到LightManager实例
    LightManager* manager = LightManager::getInstance();
    if (!manager) return false;
    
    // 这里应该将配置写入到manager
    // 目前先返回true，实际实现需要根据具体需求
    return true;
}

// 单例模式实现
LightManager* LightManager::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new LightManager();
    }
    return instance_;
}

// 构造函数
LightManager::LightManager()
    : initialized_(false)
    , total_led_count_(0)
    , power_enabled_(true)
    , power_saving_mode_(false)
    , last_activity_time_(0)
    , last_update_time_(0)
    , debug_enabled_(false) {
    
    // 初始化LED状态
    for (auto& state : led_states_) {
        state = LedState();
    }
    
    // 初始化触摸反馈
    for (auto& feedback : touch_feedbacks_) {
        feedback = TouchFeedback();
    }
}

// 析构函数
LightManager::~LightManager() {
    deinit();
}

// 初始化
bool LightManager::init() {
    if (initialized_) {
        return true;
    }
    
    // 重置统计信息
    statistics_ = LightStatistics();
    statistics_.last_reset_time = time_us_32() / 1000;
    
    // 设置默认区域
    LightRegion all_region;
    all_region.name = "all";
    all_region.enabled = true;
    all_region.priority = 255;
    regions_.push_back(all_region);
    
    last_activity_time_ = time_us_32() / 1000;
    last_update_time_ = time_us_32() / 1000;
    
    initialized_ = true;
    
    log_debug("LightManager initialized");
    
    return true;
}

// 释放资源
void LightManager::deinit() {
    if (initialized_) {
        // 停止所有效果
        stop_all_effects();
        
        // 关闭所有LED
        set_all_leds_color(0, 0, 0);
        update_device_leds();
        
        // 清理设备
        devices_.clear();
        regions_.clear();
        active_effects_.clear();
        
        // 清理回调
        effect_callback_ = nullptr;
        status_callback_ = nullptr;
        error_callback_ = nullptr;
        
        initialized_ = false;
        
        log_debug("LightManager deinitialized");
    }
}

// 检查是否就绪
bool LightManager::is_ready() const {
    return initialized_ && !devices_.empty();
}

// 添加Mai2Light设备
bool LightManager::add_mai2light_device(Mai2Light* device, const std::string& device_name) {
    if (!device) {
        return false;
    }
    
    std::string name = device_name.empty() ? ("Mai2Light_" + std::to_string(devices_.size())) : device_name;
    
    // 检查名称是否已存在
    if (devices_.find(name) != devices_.end()) {
        return false;
    }
    
    DeviceInfo info;
    info.type = DeviceInfo::MAI2LIGHT;
    info.name = name;
    info.device.mai2light = device;
    info.connected = device->is_ready();
    info.led_count = 34; // Mai2Light支持34个LED
    info.led_offset = total_led_count_;
    
    devices_[name] = info;
    total_led_count_ += info.led_count;
    
    // 更新"all"区域
    auto& all_region = regions_[0];
    for (uint8_t i = 0; i < info.led_count; i++) {
        all_region.leds.push_back(info.led_offset + i);
    }
    
    log_debug("Added Mai2Light device: " + name + " (" + std::to_string(info.led_count) + " LEDs)");
    
    return true;
}

// 添加NeoPixel设备
bool LightManager::add_neopixel_device(NeoPixel* device, const std::string& device_name) {
    if (!device) {
        return false;
    }
    
    std::string name = device_name.empty() ? ("NeoPixel_" + std::to_string(devices_.size())) : device_name;
    
    // 检查名称是否已存在
    if (devices_.find(name) != devices_.end()) {
        return false;
    }
    
    DeviceInfo info;
    info.type = DeviceInfo::NEOPIXEL;
    info.name = name;
    info.device.neopixel = device;
    info.connected = device->is_ready();
    info.led_count = device->get_num_leds();
    info.led_offset = total_led_count_;
    
    devices_[name] = info;
    total_led_count_ += info.led_count;
    
    // 更新"all"区域
    auto& all_region = regions_[0];
    for (uint8_t i = 0; i < info.led_count; i++) {
        all_region.leds.push_back(info.led_offset + i);
    }
    
    log_debug("Added NeoPixel device: " + name + " (" + std::to_string(info.led_count) + " LEDs)");
    
    return true;
}

// 移除设备
bool LightManager::remove_device(const std::string& device_name) {
    auto it = devices_.find(device_name);
    if (it == devices_.end()) {
        return false;
    }
    
    // 停止该设备相关的效果
    // 遍历所有区域，停止包含该设备LED的区域效果
    for (const auto& region : regions_) {
        // 检查区域是否包含该设备的LED
        bool contains_device_leds = false;
        for (uint8_t led_index : region.leds) {
            if (led_index >= it->second.led_offset && 
                led_index < it->second.led_offset + it->second.led_count) {
                contains_device_leds = true;
                break;
            }
        }
        
        // 如果区域包含该设备的LED，停止该区域的效果
        if (contains_device_leds) {
            auto effect_it = active_effects_.find(region.name);
            if (effect_it != active_effects_.end()) {
                active_effects_.erase(effect_it);
                log_debug("Stopped effect for region: " + region.name + " (device removed)");
                
                // 触发效果回调
                if (effect_callback_) {
                    effect_callback_(region.name, LightEffect::NONE);
                }
            }
        }
    }
    
    // 清除该设备LED的状态
    for (uint8_t i = 0; i < it->second.led_count; i++) {
        uint8_t led_index = it->second.led_offset + i;
        led_states_[led_index].current_color = {0, 0, 0};
        led_states_[led_index].target_color = {0, 0, 0};
        led_states_[led_index].dirty = true;
    }
    
    total_led_count_ -= it->second.led_count;
    devices_.erase(it);
    
    log_debug("Removed device: " + device_name);
    
    return true;
}

// 获取设备数量
uint8_t LightManager::get_device_count() const {
    return devices_.size();
}

// 获取设备信息
bool LightManager::get_device_info(const std::string& device_name, std::string& type, bool& connected) {
    auto it = devices_.find(device_name);
    if (it == devices_.end()) {
        return false;
    }
    
    type = (it->second.type == DeviceInfo::MAI2LIGHT) ? "Mai2Light" : "NeoPixel";
    connected = it->second.connected;
    return true;
}

// 配置管理已移至纯公开函数

// 添加区域
bool LightManager::add_region(const LightRegion& region) {
    // 检查区域名称是否已存在
    for (const auto& existing : regions_) {
        if (existing.name == region.name) {
            return false;
        }
    }
    
    regions_.push_back(region);
    log_debug("Added region: " + region.name + " (" + std::to_string(region.leds.size()) + " LEDs)");
    return true;
}

// 移除区域
bool LightManager::remove_region(const std::string& name) {
    if (name == "all") {
        return false; // 不能删除"all"区域
    }
    
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&name](const LightRegion& region) {
                              return region.name == name;
                          });
    
    if (it != regions_.end()) {
        // 停止该区域的效果
        stop_effect(name);
        
        regions_.erase(it);
        log_debug("Removed region: " + name);
        return true;
    }
    
    return false;
}

// 获取区域
bool LightManager::get_region(const std::string& name, LightRegion& region) {
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&name](const LightRegion& r) {
                              return r.name == name;
                          });
    
    if (it != regions_.end()) {
        region = *it;
        return true;
    }
    
    return false;
}

// 获取区域名称列表
std::vector<std::string> LightManager::get_region_names() {
    std::vector<std::string> names;
    for (const auto& region : regions_) {
        names.push_back(region.name);
    }
    return names;
}

// 启用/禁用区域
bool LightManager::enable_region(const std::string& name, bool enabled) {
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&name](LightRegion& region) {
                              return region.name == name;
                          });
    
    if (it != regions_.end()) {
        it->enabled = enabled;
        log_debug("Region " + name + " " + (enabled ? "enabled" : "disabled"));
        return true;
    }
    
    return false;
}

// 设置区域优先级
bool LightManager::set_region_priority(const std::string& name, uint8_t priority) {
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&name](LightRegion& region) {
                              return region.name == name;
                          });
    
    if (it != regions_.end()) {
        it->priority = priority;
        log_debug("Region " + name + " priority set to " + std::to_string(priority));
        return true;
    }
    
    return false;
}

// 设置LED颜色
bool LightManager::set_led_color(uint8_t led_index, const Mai2Light_RGB& color) {
    if (!is_led_valid(led_index)) {
        return false;
    }
    
    led_states_[led_index].target_color = color;
    led_states_[led_index].dirty = true;
    
    last_activity_time_ = time_us_32() / 1000;
    
    return true;
}

bool LightManager::set_led_color(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    Mai2Light_RGB color = {r, g, b};
    return set_led_color(led_index, color);
}

// 设置区域颜色
bool LightManager::set_region_color(const std::string& region_name, const Mai2Light_RGB& color) {
    if (!is_region_valid(region_name)) {
        return false;
    }
    
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&region_name](const LightRegion& region) {
                              return region.name == region_name;
                          });
    
    if (it != regions_.end() && it->enabled) {
        for (uint8_t led_index : it->leds) {
            if (is_led_valid(led_index)) {
                led_states_[led_index].target_color = color;
                led_states_[led_index].dirty = true;
            }
        }
        
        last_activity_time_ = time_us_32() / 1000;
        return true;
    }
    
    return false;
}

bool LightManager::set_region_color(const std::string& region_name, uint8_t r, uint8_t g, uint8_t b) {
    Mai2Light_RGB color = {r, g, b};
    return set_region_color(region_name, color);
}

// 设置所有LED颜色
bool LightManager::set_all_leds_color(const Mai2Light_RGB& color) {
    return set_region_color("all", color);
}

bool LightManager::set_all_leds_color(uint8_t r, uint8_t g, uint8_t b) {
    Mai2Light_RGB color = {r, g, b};
    return set_all_leds_color(color);
}

// 设置全局亮度
bool LightManager::set_global_brightness(uint8_t brightness) {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    config->global_brightness = std::min(brightness, config->max_brightness);
    
    // 标记所有LED需要更新
    for (uint8_t i = 0; i < total_led_count_; i++) {
        led_states_[i].dirty = true;
    }
    
    last_activity_time_ = time_us_32() / 1000;
    
    log_debug("Global brightness set to " + std::to_string(config->global_brightness));
    
    return true;
}

// 设置LED亮度
bool LightManager::set_led_brightness(uint8_t led_index, uint8_t brightness) {
    if (!is_led_valid(led_index)) {
        return false;
    }
    
    led_states_[led_index].brightness = brightness;
    led_states_[led_index].dirty = true;
    
    last_activity_time_ = time_us_32() / 1000;
    
    return true;
}

// 设置区域亮度
bool LightManager::set_region_brightness(const std::string& region_name, uint8_t brightness) {
    if (!is_region_valid(region_name)) {
        return false;
    }
    
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&region_name](const LightRegion& region) {
                              return region.name == region_name;
                          });
    
    if (it != regions_.end() && it->enabled) {
        for (uint8_t led_index : it->leds) {
            if (is_led_valid(led_index)) {
                led_states_[led_index].brightness = brightness;
                led_states_[led_index].dirty = true;
            }
        }
        
        last_activity_time_ = time_us_32() / 1000;
        return true;
    }
    
    return false;
}

// 获取全局亮度
uint8_t LightManager::get_global_brightness() const {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    return config->global_brightness;
}

// 设置效果
bool LightManager::set_effect(const std::string& region_name, const EffectParams& params) {
    if (!is_region_valid(region_name)) {
        return false;
    }
    
    active_effects_[region_name] = params;
    
    statistics_.effect_changes++;
    last_activity_time_ = time_us_32() / 1000;
    
    log_debug("Effect set for region " + region_name + ": " + std::to_string(static_cast<int>(params.effect)));
    
    // 通知效果回调
    if (effect_callback_) {
        effect_callback_(region_name, params.effect);
    }
    
    return true;
}

// 停止效果
bool LightManager::stop_effect(const std::string& region_name) {
    auto it = active_effects_.find(region_name);
    if (it != active_effects_.end()) {
        active_effects_.erase(it);
        log_debug("Stopped effect for region " + region_name);
        return true;
    }
    
    return false;
}

// 停止所有效果
bool LightManager::stop_all_effects() {
    active_effects_.clear();
    log_debug("Stopped all effects");
    return true;
}

// 获取效果
bool LightManager::get_effect(const std::string& region_name, EffectParams& params) {
    auto it = active_effects_.find(region_name);
    if (it != active_effects_.end()) {
        params = it->second;
        return true;
    }
    
    return false;
}

// 预设效果实现
bool LightManager::apply_static_effect(const std::string& region_name, const Mai2Light_RGB& color) {
    EffectParams params;
    params.effect = LightEffect::STATIC;
    params.color1 = color;
    return set_effect(region_name, params);
}

bool LightManager::apply_breathing_effect(const std::string& region_name, const Mai2Light_RGB& color, uint16_t speed) {
    EffectParams params;
    params.effect = LightEffect::BREATHING;
    params.color1 = color;
    params.speed = speed;
    return set_effect(region_name, params);
}

bool LightManager::apply_rainbow_effect(const std::string& region_name, uint16_t speed) {
    EffectParams params;
    params.effect = LightEffect::RAINBOW;
    params.speed = speed;
    return set_effect(region_name, params);
}

bool LightManager::apply_wave_effect(const std::string& region_name, const Mai2Light_RGB& color, uint16_t speed) {
    EffectParams params;
    params.effect = LightEffect::WAVE;
    params.color1 = color;
    params.speed = speed;
    return set_effect(region_name, params);
}

bool LightManager::apply_flash_effect(const std::string& region_name, const Mai2Light_RGB& color, uint16_t speed) {
    EffectParams params;
    params.effect = LightEffect::FLASH;
    params.color1 = color;
    params.speed = speed;
    return set_effect(region_name, params);
}

// 触摸反馈
bool LightManager::set_touch_feedback(uint8_t point_index, const TouchFeedback& feedback) {
    if (point_index >= 34) {
        return false;
    }
    
    touch_feedbacks_[point_index] = feedback;
    log_debug("Touch feedback set for point " + std::to_string(point_index));
    return true;
}

bool LightManager::get_touch_feedback(uint8_t point_index, TouchFeedback& feedback) {
    if (point_index >= 34) {
        return false;
    }
    
    feedback = touch_feedbacks_[point_index];
    return true;
}

bool LightManager::trigger_touch_feedback(uint8_t point_index) {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    if (point_index >= 34 || !config->enable_touch_feedback) {
        return false;
    }
    
    const TouchFeedback& feedback = touch_feedbacks_[point_index];
    if (!feedback.enabled) {
        return false;
    }
    
    // 创建临时效果
    EffectParams params;
    params.effect = feedback.effect;
    params.color1 = feedback.color;
    params.speed = feedback.duration;
    params.intensity = feedback.intensity;
    params.fade_time = feedback.fade_time;
    params.loop = false;
    params.duration = feedback.duration;
    
    // 应用到对应的LED（假设点位直接对应LED索引）
    if (point_index < total_led_count_) {
        set_led_color(point_index, feedback.color);
    }
    
    statistics_.touch_feedbacks++;
    last_activity_time_ = time_us_32() / 1000;
    
    log_debug("Triggered touch feedback for point " + std::to_string(point_index));
    
    return true;
}

bool LightManager::enable_touch_feedback(bool enabled) {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    config->enable_touch_feedback = enabled;
    log_debug("Touch feedback " + std::string(enabled ? "enabled" : "disabled"));
    return true;
}

// 电源管理
bool LightManager::set_power_state(bool enabled) {
    if (power_enabled_ != enabled) {
        power_enabled_ = enabled;
        
        if (!enabled) {
            // 关闭所有LED
            set_all_leds_color(0, 0, 0);
            update_device_leds();
        }
        
        statistics_.power_cycles++;
        
        log_debug("Power " + std::string(enabled ? "enabled" : "disabled"));
        
        // 通知状态回调
        if (status_callback_) {
            LightManager_PrivateConfig* config = light_manager_get_config_copy();
            status_callback_(enabled, config->global_brightness);
        }
    }
    
    return true;
}

bool LightManager::get_power_state() const {
    return power_enabled_;
}

bool LightManager::enter_power_saving_mode() {
    if (!power_saving_mode_) {
        power_saving_mode_ = true;
        
        // 降低亮度
        LightManager_PrivateConfig* config = light_manager_get_config_copy();
        set_global_brightness(config->idle_brightness);
        
        log_debug("Entered power saving mode");
    }
    
    return true;
}

bool LightManager::exit_power_saving_mode() {
    if (power_saving_mode_) {
        power_saving_mode_ = false;
        
        // 恢复亮度
        set_global_brightness(255); // 或者恢复之前保存的亮度
        
        log_debug("Exited power saving mode");
    }
    
    return true;
}

bool LightManager::is_power_saving_mode() const {
    return power_saving_mode_;
}

// 自动调光
bool LightManager::enable_auto_dim(bool enabled) {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    config->enable_auto_dim = enabled;
    log_debug("Auto dim " + std::string(enabled ? "enabled" : "disabled"));
    return true;
}

bool LightManager::set_auto_dim_timeout(uint16_t timeout_seconds) {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    config->auto_dim_timeout = timeout_seconds;
    log_debug("Auto dim timeout set to " + std::to_string(timeout_seconds) + " seconds");
    return true;
}

bool LightManager::set_idle_brightness(uint8_t brightness) {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    config->idle_brightness = brightness;
    log_debug("Idle brightness set to " + std::to_string(brightness));
    return true;
}

// 状态查询
bool LightManager::get_led_color(uint8_t led_index, Mai2Light_RGB& color) {
    if (!is_led_valid(led_index)) {
        return false;
    }
    
    color = led_states_[led_index].current_color;
    return true;
}

bool LightManager::get_led_brightness(uint8_t led_index, uint8_t& brightness) {
    if (!is_led_valid(led_index)) {
        return false;
    }
    
    brightness = led_states_[led_index].brightness;
    return true;
}

bool LightManager::is_effect_active(const std::string& region_name) {
    return active_effects_.find(region_name) != active_effects_.end();
}

// 统计信息
bool LightManager::get_statistics(LightStatistics& stats) {
    stats = statistics_;
    stats.uptime_seconds = ((time_us_32() / 1000) - stats.last_reset_time) / 1000;
    return true;
}

void LightManager::reset_statistics() {
    statistics_ = LightStatistics();
    statistics_.last_reset_time = time_us_32() / 1000;
    log_debug("Statistics reset");
}

// 设置回调
void LightManager::set_effect_callback(LightEffectCallback callback) {
    effect_callback_ = callback;
}

void LightManager::set_status_callback(LightStatusCallback callback) {
    status_callback_ = callback;
}

void LightManager::set_error_callback(LightErrorCallback callback) {
    error_callback_ = callback;
}

// 任务处理
void LightManager::task() {
    if (!initialized_ || !power_enabled_) {
        return;
    }
    
    uint32_t current_time = time_us_32() / 1000;
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    
    // 检查更新间隔
    if (current_time - last_update_time_ < config->update_interval_ms) {
        return;
    }
    
    // 更新效果
    update_effects();
    
    // 更新LED状态
    update_led_states();
    
    // 应用亮度限制
    apply_brightness_limits();
    
    // 更新设备LED
    update_device_leds();
    
    // 处理自动调光
    if (config->enable_auto_dim) {
        handle_auto_dim();
    }
    
    // 处理节能模式
    if (config->enable_power_saving) {
        handle_power_saving();
    }
    
    statistics_.total_updates++;
    last_update_time_ = current_time;
}

// 调试功能
void LightManager::enable_debug_output(bool enabled) {
    debug_enabled_ = enabled;
    log_debug("Debug output " + std::string(enabled ? "enabled" : "disabled"));
}

std::string LightManager::get_debug_info() {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    std::string info = "LightManager Debug Info:\n";
    info += "Devices: " + std::to_string(devices_.size()) + "\n";
    info += "Total LEDs: " + std::to_string(total_led_count_) + "\n";
    info += "Active effects: " + std::to_string(active_effects_.size()) + "\n";
    info += "Power enabled: " + std::string(power_enabled_ ? "Yes" : "No") + "\n";
    info += "Power saving: " + std::string(power_saving_mode_ ? "Yes" : "No") + "\n";
    info += "Global brightness: " + std::to_string(config->global_brightness) + "\n";
    info += "Total updates: " + std::to_string(statistics_.total_updates) + "\n";
    
    return info;
}

bool LightManager::test_led(uint8_t led_index) {
    if (!is_led_valid(led_index)) {
        return false;
    }
    
    // 闪烁测试
    set_led_color(led_index, 255, 255, 255);
    update_device_leds();
    sleep_ms(200);
    set_led_color(led_index, 0, 0, 0);
    update_device_leds();
    
    log_debug("Tested LED " + std::to_string(led_index));
    return true;
}

bool LightManager::test_region(const std::string& region_name) {
    if (!is_region_valid(region_name)) {
        return false;
    }
    
    // 闪烁测试
    set_region_color(region_name, 255, 255, 255);
    update_device_leds();
    sleep_ms(500);
    set_region_color(region_name, 0, 0, 0);
    update_device_leds();
    
    log_debug("Tested region " + region_name);
    return true;
}

// 私有方法实现
void LightManager::update_effects() {
    for (auto& pair : active_effects_) {
        const std::string& region_name = pair.first;
        const EffectParams& params = pair.second;
        
        switch (params.effect) {
            case LightEffect::STATIC:
                process_static_effect(region_name, params);
                break;
            case LightEffect::BREATHING:
                process_breathing_effect(region_name, params);
                break;
            case LightEffect::RAINBOW:
                process_rainbow_effect(region_name, params);
                break;
            case LightEffect::WAVE:
                process_wave_effect(region_name, params);
                break;
            case LightEffect::RIPPLE:
                process_ripple_effect(region_name, params);
                break;
            case LightEffect::FLASH:
                process_flash_effect(region_name, params);
                break;
            case LightEffect::FADE:
                process_fade_effect(region_name, params);
                break;
            case LightEffect::CHASE:
                process_chase_effect(region_name, params);
                break;
            case LightEffect::SPARKLE:
                process_sparkle_effect(region_name, params);
                break;
            case LightEffect::FIRE:
                process_fire_effect(region_name, params);
                break;
            default:
                break;
        }
    }
}

void LightManager::update_led_states() {
    for (uint8_t i = 0; i < total_led_count_; i++) {
        LedState& state = led_states_[i];
        
        // 简单的颜色插值（可以改进为更平滑的过渡）
        if (state.current_color.r != state.target_color.r ||
            state.current_color.g != state.target_color.g ||
            state.current_color.b != state.target_color.b) {
            
            state.current_color = state.target_color;
            state.dirty = true;
        }
        
        state.last_update = time_us_32() / 1000;
    }
}

void LightManager::apply_brightness_limits() {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    for (uint8_t i = 0; i < total_led_count_; i++) {
        LedState& state = led_states_[i];
        
        if (state.dirty) {
            // 应用全局亮度和个别亮度
            uint16_t brightness_calc = static_cast<uint16_t>(config->global_brightness * state.brightness) / 255;
            uint16_t max_brightness = static_cast<uint16_t>(config->max_brightness);
            uint8_t effective_brightness = static_cast<uint8_t>(std::min(brightness_calc, max_brightness));
            
            state.current_color = apply_brightness(state.current_color, effective_brightness);
        }
    }
}

void LightManager::handle_auto_dim() {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    uint32_t current_time = time_us_32() / 1000;
    uint32_t idle_time = current_time - last_activity_time_;
    
    if (idle_time >= config->auto_dim_timeout * 1000) {
        if (!power_saving_mode_) {
            enter_power_saving_mode();
        }
    } else {
        if (power_saving_mode_) {
            exit_power_saving_mode();
        }
    }
}

void LightManager::handle_power_saving() {
    LightManager_PrivateConfig* config = light_manager_get_config_copy();
    uint32_t current_time = time_us_32() / 1000;
    uint32_t idle_time = current_time - last_activity_time_;
    
    if (idle_time >= config->power_saving_timeout * 1000) {
        if (power_enabled_) {
            set_power_state(false);
        }
    }
}

// 效果处理方法（简化实现）
void LightManager::process_static_effect(const std::string& region_name, const EffectParams& params) {
    set_region_color(region_name, params.color1);
}

void LightManager::process_breathing_effect(const std::string& region_name, const EffectParams& params) {
    uint32_t time = time_us_32() / 1000;
    float phase = (time % params.speed) / static_cast<float>(params.speed);
    float brightness = (sin(phase * 2 * 3.14159f) + 1.0f) / 2.0f;
    
    Mai2Light_RGB color = apply_brightness(params.color1, static_cast<uint8_t>(brightness * params.intensity));
    set_region_color(region_name, color);
}

void LightManager::process_rainbow_effect(const std::string& region_name, const EffectParams& params) {
    uint32_t time = time_us_32() / 1000;
    uint16_t hue = (time / (params.speed / 360)) % 360;
    
    Mai2Light_RGB color = hsv_to_rgb(hue, 255, params.intensity);
    set_region_color(region_name, color);
}

void LightManager::process_wave_effect(const std::string& region_name, const EffectParams& params) {
    // 简化的波浪效果
    process_breathing_effect(region_name, params);
}

void LightManager::process_ripple_effect(const std::string& region_name, const EffectParams& params) {
    // 简化的涟漪效果
    process_breathing_effect(region_name, params);
}

void LightManager::process_flash_effect(const std::string& region_name, const EffectParams& params) {
    uint32_t time = time_us_32() / 1000;
    bool on = (time % params.speed) < (params.speed / 2);
    
    Mai2Light_RGB color = on ? params.color1 : Mai2Light_RGB{0, 0, 0};
    set_region_color(region_name, color);
}

void LightManager::process_fade_effect(const std::string& region_name, const EffectParams& params) {
    uint32_t time = time_us_32() / 1000;
    float factor = (time % params.speed) / static_cast<float>(params.speed);
    
    Mai2Light_RGB color = interpolate_color(params.color1, params.color2, factor);
    set_region_color(region_name, color);
}

void LightManager::process_chase_effect(const std::string& region_name, const EffectParams& params) {
    // 简化的追逐效果
    process_wave_effect(region_name, params);
}

void LightManager::process_sparkle_effect(const std::string& region_name, const EffectParams& params) {
    // 简化的闪烁星光效果
    if (rand() % 100 < 10) { // 10%概率闪烁
        set_region_color(region_name, params.color1);
    } else {
        set_region_color(region_name, Mai2Light_RGB{0, 0, 0});
    }
}

void LightManager::process_fire_effect(const std::string& region_name, const EffectParams& params) {
    // 简化的火焰效果
    uint8_t red = params.color1.r;
    uint8_t green = params.color1.g / 2 + (rand() % (params.color1.g / 2));
    uint8_t blue = 0;
    
    set_region_color(region_name, Mai2Light_RGB{red, green, blue});
}

// 颜色处理方法
Mai2Light_RGB LightManager::interpolate_color(const Mai2Light_RGB& color1, const Mai2Light_RGB& color2, float factor) {
    factor = std::max(0.0f, std::min(1.0f, factor));
    
    Mai2Light_RGB result;
    result.r = static_cast<uint8_t>(color1.r + (color2.r - color1.r) * factor);
    result.g = static_cast<uint8_t>(color1.g + (color2.g - color1.g) * factor);
    result.b = static_cast<uint8_t>(color1.b + (color2.b - color1.b) * factor);
    
    return result;
}

Mai2Light_RGB LightManager::apply_brightness(const Mai2Light_RGB& color, uint8_t brightness) {
    Mai2Light_RGB result;
    result.r = (color.r * brightness) / 255;
    result.g = (color.g * brightness) / 255;
    result.b = (color.b * brightness) / 255;
    
    return result;
}

Mai2Light_RGB LightManager::hsv_to_rgb(uint16_t hue, uint8_t saturation, uint8_t value) {
    hue = hue % 360;
    float h = hue / 60.0f;
    float s = saturation / 255.0f;
    float v = value / 255.0f;
    
    int i = static_cast<int>(h);
    float f = h - i;
    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));
    
    float r, g, b;
    
    switch (i) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    
    Mai2Light_RGB result;
    result.r = static_cast<uint8_t>(r * 255);
    result.g = static_cast<uint8_t>(g * 255);
    result.b = static_cast<uint8_t>(b * 255);
    
    return result;
}

// 工具方法
bool LightManager::is_led_valid(uint8_t led_index) const {
    return led_index < total_led_count_;
}

bool LightManager::is_region_valid(const std::string& region_name) const {
    return std::find_if(regions_.begin(), regions_.end(),
                       [&region_name](const LightRegion& region) {
                           return region.name == region_name;
                       }) != regions_.end();
}

uint8_t LightManager::get_region_led_count(const std::string& region_name) const {
    auto it = std::find_if(regions_.begin(), regions_.end(),
                          [&region_name](const LightRegion& region) {
                              return region.name == region_name;
                          });
    
    return (it != regions_.end()) ? it->leds.size() : 0;
}

void LightManager::mark_leds_dirty(const std::vector<uint8_t>& led_indices) {
    for (uint8_t index : led_indices) {
        if (is_led_valid(index)) {
            led_states_[index].dirty = true;
        }
    }
}

void LightManager::update_device_leds() {
    for (auto& pair : devices_) {
        DeviceInfo& info = pair.second;
        
        if (!info.connected) continue;
        
        // 检查是否有LED需要更新
        bool needs_update = false;
        for (uint8_t i = 0; i < info.led_count; i++) {
            if (led_states_[info.led_offset + i].dirty) {
                needs_update = true;
                break;
            }
        }
        
        if (!needs_update) continue;
        
        // 更新设备LED
        if (info.type == DeviceInfo::MAI2LIGHT) {
            for (uint8_t i = 0; i < info.led_count; i++) {
                uint8_t led_index = info.led_offset + i;
                if (led_states_[led_index].dirty) {
                    info.device.mai2light->set_led_color(i, led_states_[led_index].current_color);
                    led_states_[led_index].dirty = false;
                }
            }
        } else if (info.type == DeviceInfo::NEOPIXEL) {
            for (uint8_t i = 0; i < info.led_count; i++) {
                uint8_t led_index = info.led_offset + i;
                if (led_states_[led_index].dirty) {
                    const Mai2Light_RGB& color = led_states_[led_index].current_color;
                    info.device.neopixel->set_pixel(i, color.r, color.g, color.b);
                    led_states_[led_index].dirty = false;
                }
            }
            info.device.neopixel->show();
        }
    }
}

void LightManager::log_debug(const std::string& message) {
    if (debug_enabled_) {
        USB_LOG_TAG_DEBUG("LIGHT_MGR", "%s", message.c_str());
    }
}

void LightManager::log_error(const std::string& message) {
    USB_LOG_TAG_ERROR("LIGHT_MGR", "%s", message.c_str());
    
    if (error_callback_) {
        error_callback_(message);
    }
}