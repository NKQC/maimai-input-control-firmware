#ifndef LIGHT_MANAGER_H
#define LIGHT_MANAGER_H

#include "../../protocol/mai2light/mai2light.h"
#include "../../protocol/neopixel/neopixel.h"
#include <array>
#include <vector>
#include <string>
#include <functional>
#include <map>
#include <memory>

// 前向声明
class HID;

// LED效果类型
enum class LightEffect {
    NONE = 0,
    STATIC,          // 静态颜色
    BREATHING,       // 呼吸效果
    RAINBOW,         // 彩虹效果
    WAVE,           // 波浪效果
    RIPPLE,         // 涟漪效果
    FLASH,          // 闪烁效果
    FADE,           // 渐变效果
    CHASE,          // 追逐效果
    SPARKLE,        // 闪烁星光
    FIRE,           // 火焰效果
    CUSTOM          // 自定义效果
};

// LED区域定义
struct LightRegion {
    std::string name;           // 区域名称
    std::vector<uint8_t> leds;  // LED索引列表
    bool enabled;               // 是否启用
    uint8_t priority;           // 优先级 (0-255)
    
    LightRegion() : enabled(true), priority(128) {}
};

// LED效果参数
struct EffectParams {
    LightEffect effect;         // 效果类型
    Mai2Light_RGB color1;       // 主颜色
    Mai2Light_RGB color2;       // 辅助颜色
    uint16_t speed;             // 速度 (ms)
    uint8_t intensity;          // 强度 (0-255)
    uint8_t fade_time;          // 渐变时间
    bool reverse;               // 反向
    bool loop;                  // 循环
    uint32_t duration;          // 持续时间 (ms, 0=无限)
    
    EffectParams() 
        : effect(LightEffect::NONE)
        , color1{255, 255, 255}
        , color2{0, 0, 0}
        , speed(1000)
        , intensity(255)
        , fade_time(100)
        , reverse(false)
        , loop(true)
        , duration(0) {}
};

// LED状态
struct LedState {
    Mai2Light_RGB current_color;    // 当前颜色
    Mai2Light_RGB target_color;     // 目标颜色
    uint8_t brightness;             // 亮度
    uint32_t last_update;           // 上次更新时间
    bool dirty;                     // 需要更新
    
    LedState() : brightness(255), last_update(0), dirty(false) {
        current_color = {0, 0, 0};
        target_color = {0, 0, 0};
    }
};

// 触摸反馈配置
struct TouchFeedback {
    bool enabled;               // 是否启用
    LightEffect effect;         // 反馈效果
    Mai2Light_RGB color;        // 反馈颜色
    uint16_t duration;          // 持续时间 (ms)
    uint8_t intensity;          // 强度
    uint8_t fade_time;          // 渐变时间
    
    TouchFeedback() 
        : enabled(true)
        , effect(LightEffect::FLASH)
        , color{255, 255, 255}
        , duration(200)
        , intensity(255)
        , fade_time(50) {}
};

// 灯光管理器私有配置
struct LightManager_PrivateConfig {
    uint16_t update_interval_ms;    // 更新间隔
    uint8_t global_brightness;      // 全局亮度
    uint8_t max_brightness;         // 最大亮度限制
    bool enable_touch_feedback;     // 启用触摸反馈
    bool enable_auto_dim;           // 启用自动调光
    uint16_t auto_dim_timeout;      // 自动调光超时 (s)
    uint8_t idle_brightness;        // 空闲亮度
    bool enable_power_saving;       // 启用节能模式
    uint16_t power_saving_timeout;  // 节能模式超时 (s)
    
    LightManager_PrivateConfig()
        : update_interval_ms(20)
        , global_brightness(255)
        , max_brightness(255)
        , enable_touch_feedback(true)
        , enable_auto_dim(false)
        , auto_dim_timeout(300)
        , idle_brightness(64)
        , enable_power_saving(false)
        , power_saving_timeout(600) {}
};

// 灯光管理器配置（仅包含服务指针）
struct LightManager_Config {
    // 外部服务指针
    class ConfigManager* config_manager;
    class InputManager* input_manager;
    class UIManager* ui_manager;
    
    LightManager_Config()
        : config_manager(nullptr)
        , input_manager(nullptr)
        , ui_manager(nullptr) {}
};

// 灯光统计信息
struct LightStatistics {
    uint32_t total_updates;         // 总更新次数
    uint32_t effect_changes;        // 效果变更次数
    uint32_t touch_feedbacks;       // 触摸反馈次数
    uint32_t power_cycles;          // 电源循环次数
    uint32_t last_reset_time;       // 上次重置时间
    uint32_t uptime_seconds;        // 运行时间
    
    LightStatistics() 
        : total_updates(0)
        , effect_changes(0)
        , touch_feedbacks(0)
        , power_cycles(0)
        , last_reset_time(0)
        , uptime_seconds(0) {}
};

// 回调函数类型
using LightEffectCallback = std::function<void(const std::string& region, LightEffect effect)>;
using LightStatusCallback = std::function<void(bool enabled, uint8_t brightness)>;
using LightErrorCallback = std::function<void(const std::string& error)>;

// LightManager配置管理纯公开函数
LightManager_Config* light_manager_get_config_holder();
bool light_manager_load_config_from_manager(LightManager_Config* config);
LightManager_PrivateConfig* light_manager_get_config_copy();
bool light_manager_write_config_to_manager(const LightManager_Config* config);

// 灯光管理器类
class LightManager {
public:
    // 单例模式
    static LightManager* getInstance();
    
    // 析构函数
    ~LightManager();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 设备管理
    bool add_mai2light_device(Mai2Light* device, const std::string& device_name = "");
    bool add_neopixel_device(NeoPixel* device, const std::string& device_name = "");
    bool remove_device(const std::string& device_name);
    uint8_t get_device_count() const;
    bool get_device_info(const std::string& device_name, std::string& type, bool& connected);
    
    // 配置管理已移至纯公开函数
    
    // 区域管理
    bool add_region(const LightRegion& region);
    bool remove_region(const std::string& name);
    bool get_region(const std::string& name, LightRegion& region);
    std::vector<std::string> get_region_names();
    bool enable_region(const std::string& name, bool enabled);
    bool set_region_priority(const std::string& name, uint8_t priority);
    
    // LED控制
    bool set_led_color(uint8_t led_index, const Mai2Light_RGB& color);
    bool set_led_color(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);
    bool set_region_color(const std::string& region_name, const Mai2Light_RGB& color);
    bool set_region_color(const std::string& region_name, uint8_t r, uint8_t g, uint8_t b);
    bool set_all_leds_color(const Mai2Light_RGB& color);
    bool set_all_leds_color(uint8_t r, uint8_t g, uint8_t b);
    
    // 亮度控制
    bool set_global_brightness(uint8_t brightness);
    bool set_led_brightness(uint8_t led_index, uint8_t brightness);
    bool set_region_brightness(const std::string& region_name, uint8_t brightness);
    uint8_t get_global_brightness() const;
    
    // 效果控制
    bool set_effect(const std::string& region_name, const EffectParams& params);
    bool stop_effect(const std::string& region_name);
    bool stop_all_effects();
    bool get_effect(const std::string& region_name, EffectParams& params);
    
    // 预设效果
    bool apply_static_effect(const std::string& region_name, const Mai2Light_RGB& color);
    bool apply_breathing_effect(const std::string& region_name, const Mai2Light_RGB& color, uint16_t speed = 2000);
    bool apply_rainbow_effect(const std::string& region_name, uint16_t speed = 1000);
    bool apply_wave_effect(const std::string& region_name, const Mai2Light_RGB& color, uint16_t speed = 500);
    bool apply_flash_effect(const std::string& region_name, const Mai2Light_RGB& color, uint16_t speed = 200);
    
    // 触摸反馈
    bool set_touch_feedback(uint8_t point_index, const TouchFeedback& feedback);
    bool get_touch_feedback(uint8_t point_index, TouchFeedback& feedback);
    bool trigger_touch_feedback(uint8_t point_index);
    bool enable_touch_feedback(bool enabled);
    
    // 电源管理
    bool set_power_state(bool enabled);
    bool get_power_state() const;
    bool enter_power_saving_mode();
    bool exit_power_saving_mode();
    bool is_power_saving_mode() const;
    
    // 自动调光
    bool enable_auto_dim(bool enabled);
    bool set_auto_dim_timeout(uint16_t timeout_seconds);
    bool set_idle_brightness(uint8_t brightness);
    
    // 状态查询
    bool get_led_color(uint8_t led_index, Mai2Light_RGB& color);
    bool get_led_brightness(uint8_t led_index, uint8_t& brightness);
    bool is_effect_active(const std::string& region_name);
    
    // 统计信息
    bool get_statistics(LightStatistics& stats);
    void reset_statistics();
    
    // 回调设置
    void set_effect_callback(LightEffectCallback callback);
    void set_status_callback(LightStatusCallback callback);
    void set_error_callback(LightErrorCallback callback);
    
    // 任务处理
    void task();
    
    // 调试功能
    void enable_debug_output(bool enabled);
    std::string get_debug_info();
    bool test_led(uint8_t led_index);
    bool test_region(const std::string& region_name);
    
private:
    // 私有构造函数（单例模式）
    LightManager();
    LightManager(const LightManager&) = delete;
    LightManager& operator=(const LightManager&) = delete;
    
    // 静态实例
    static LightManager* instance_;
    
    // 设备信息结构体
    struct DeviceInfo {
        enum Type { MAI2LIGHT, NEOPIXEL } type;
        std::string name;
        union {
            Mai2Light* mai2light;
            NeoPixel* neopixel;
        } device;
        bool connected;
        uint8_t led_count;
        uint8_t led_offset;  // 在全局LED数组中的偏移
    };
    
    // 成员变量
    bool initialized_;
    std::map<std::string, DeviceInfo> devices_;
    std::vector<LightRegion> regions_;
    std::map<std::string, EffectParams> active_effects_;
    std::array<LedState, 256> led_states_;  // 最多支持256个LED
    std::array<TouchFeedback, 34> touch_feedbacks_;  // 34个触摸点
    LightStatistics statistics_;
    
    // 状态变量
    uint8_t total_led_count_;
    bool power_enabled_;
    bool power_saving_mode_;
    uint32_t last_activity_time_;
    uint32_t last_update_time_;
    bool debug_enabled_;
    
    // 回调函数
    LightEffectCallback effect_callback_;
    LightStatusCallback status_callback_;
    LightErrorCallback error_callback_;
    
    // 私有方法
    void update_effects();
    void update_led_states();
    void apply_brightness_limits();
    void handle_auto_dim();
    void handle_power_saving();
    
    // 效果处理
    void process_static_effect(const std::string& region_name, const EffectParams& params);
    void process_breathing_effect(const std::string& region_name, const EffectParams& params);
    void process_rainbow_effect(const std::string& region_name, const EffectParams& params);
    void process_wave_effect(const std::string& region_name, const EffectParams& params);
    void process_ripple_effect(const std::string& region_name, const EffectParams& params);
    void process_flash_effect(const std::string& region_name, const EffectParams& params);
    void process_fade_effect(const std::string& region_name, const EffectParams& params);
    void process_chase_effect(const std::string& region_name, const EffectParams& params);
    void process_sparkle_effect(const std::string& region_name, const EffectParams& params);
    void process_fire_effect(const std::string& region_name, const EffectParams& params);
    
    // 颜色处理
    Mai2Light_RGB interpolate_color(const Mai2Light_RGB& color1, const Mai2Light_RGB& color2, float factor);
    Mai2Light_RGB apply_brightness(const Mai2Light_RGB& color, uint8_t brightness);
    Mai2Light_RGB hsv_to_rgb(uint16_t hue, uint8_t saturation, uint8_t value);
    
    // 工具方法
    bool is_led_valid(uint8_t led_index) const;
    bool is_region_valid(const std::string& region_name) const;
    uint8_t get_region_led_count(const std::string& region_name) const;
    void mark_leds_dirty(const std::vector<uint8_t>& led_indices);
    void update_device_leds();
    void log_debug(const std::string& message);
    void log_error(const std::string& message);
};

#endif // LIGHT_MANAGER_H