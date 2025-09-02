#pragma once

#include "../../hal/pio/hal_pio.h"
#include <stdint.h>
#include <vector>

/**
 * 协议层 - NeoPixel (WS2812B) LED控制
 * 基于PIO实现的高精度时序控制
 * 支持RGB和RGBW LED
 */

// NeoPixel时序参数（基于800kHz）
#define NEOPIXEL_T0H_NS     350   // 0码高电平时间 (ns)
#define NEOPIXEL_T0L_NS     800   // 0码低电平时间 (ns)
#define NEOPIXEL_T1H_NS     700   // 1码高电平时间 (ns)
#define NEOPIXEL_T1L_NS     600   // 1码低电平时间 (ns)
#define NEOPIXEL_RESET_US   50    // 复位时间 (us)
#define NEOPIXEL_WAIT_TIMEOUT_US 1000   // 等待超时时间 (us)

// LED类型定义
enum NeoPixel_Type {
    NEOPIXEL_RGB = 0,   // RGB (24位)
    NEOPIXEL_RGBW = 1   // RGBW (32位)
};

// 颜色结构
struct NeoPixel_Color {
    uint8_t r;  // 红色 (0-255)
    uint8_t g;  // 绿色 (0-255)
    uint8_t b;  // 蓝色 (0-255)
    uint8_t w;  // 白色 (0-255, 仅RGBW)
    
    NeoPixel_Color(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0, uint8_t white = 0)
        : r(red), g(green), b(blue), w(white) {}
    
    // 从32位颜色值创建
    static NeoPixel_Color from_rgb(uint32_t rgb) {
        return NeoPixel_Color((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
    }
    
    static NeoPixel_Color from_rgbw(uint32_t rgbw) {
        return NeoPixel_Color((rgbw >> 24) & 0xFF, (rgbw >> 16) & 0xFF, 
                             (rgbw >> 8) & 0xFF, rgbw & 0xFF);
    }
    
    // 转换为32位颜色值
    uint32_t to_rgb() const {
        return (r << 16) | (g << 8) | b;
    }
    
    uint32_t to_rgbw() const {
        return (r << 24) | (g << 16) | (b << 8) | w;
    }
};

// 动画效果类型
enum NeoPixel_Effect {
    NEOPIXEL_EFFECT_NONE = 0,
    NEOPIXEL_EFFECT_FADE,
    NEOPIXEL_EFFECT_RAINBOW,
    NEOPIXEL_EFFECT_CHASE,
    NEOPIXEL_EFFECT_BREATHE,
    NEOPIXEL_EFFECT_TWINKLE
};

// 动画参数结构
struct NeoPixel_Animation {
    NeoPixel_Effect effect;
    uint32_t duration_ms;       // 动画持续时间
    uint32_t speed_ms;          // 动画速度（每步时间）
    NeoPixel_Color color1;      // 主颜色
    NeoPixel_Color color2;      // 辅助颜色
    bool loop;                  // 是否循环
    uint8_t brightness;         // 亮度 (0-255)
};

class NeoPixel {
public:
    NeoPixel(HAL_PIO* pio_hal, uint16_t num_leds, NeoPixel_Type type = NEOPIXEL_RGB);
    ~NeoPixel();
    
    // 初始化
    bool init();
    
    // 释放资源
    void deinit();
    
    // 检查是否就绪
    bool is_ready() const;
    
    // 基本LED控制
    bool set_pixel(uint16_t index, const NeoPixel_Color& color);
    bool set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0);
    bool set_all_pixels(const NeoPixel_Color& color);
    bool clear_all();
    
    // 批量设置
    bool set_pixels(uint16_t start_index, const std::vector<NeoPixel_Color>& colors);
    bool set_range(uint16_t start_index, uint16_t count, const NeoPixel_Color& color);
    
    // 获取像素颜色
    NeoPixel_Color get_pixel(uint16_t index) const;
    
    // 显示更新
    bool show();
    
    // 亮度控制
    void set_brightness(uint8_t brightness);  // 0-255
    uint8_t get_brightness() const;
    
    // 动画控制
    bool start_animation(const NeoPixel_Animation& animation);
    bool stop_animation();
    bool is_animation_running() const;
    
    // 预定义效果
    bool fade_to_color(const NeoPixel_Color& target_color, uint32_t duration_ms);
    bool rainbow_cycle(uint32_t speed_ms, bool loop = true);
    bool color_chase(const NeoPixel_Color& color, uint32_t speed_ms, bool loop = true);
    bool breathe_effect(const NeoPixel_Color& color, uint32_t period_ms, bool loop = true);
    bool twinkle_effect(const NeoPixel_Color& color, uint32_t speed_ms, bool loop = true);
    
    // 任务处理（需要在主循环中调用）
    void task();
    
    // 获取LED数量和类型
    uint16_t get_num_leds() const { return num_leds_; }
    NeoPixel_Type get_type() const { return type_; }
    
    // 颜色工具函数
    static NeoPixel_Color hsv_to_rgb(uint16_t hue, uint8_t saturation, uint8_t value);
    static NeoPixel_Color wheel_color(uint8_t pos);  // 0-255 色轮
    static NeoPixel_Color blend_colors(const NeoPixel_Color& color1, const NeoPixel_Color& color2, uint8_t blend);
    
private:
    HAL_PIO* pio_hal_;
    uint16_t num_leds_;
    NeoPixel_Type type_;
    bool initialized_;
    
    // PIO程序相关
    uint8_t pio_sm_;           // 状态机编号
    uint8_t pio_offset_;       // 程序偏移
    
    // LED数据缓冲区
    std::vector<NeoPixel_Color> pixels_;
    std::vector<uint32_t> pixel_data_;  // 实际发送的数据
    
    // 亮度控制
    uint8_t brightness_;
    
    // 动画状态
    bool animation_running_;
    NeoPixel_Animation current_animation_;
    uint32_t animation_start_time_;
    uint32_t animation_last_update_;
    uint32_t animation_step_;
    std::vector<NeoPixel_Color> animation_start_colors_;
    
    // 内部方法
    bool load_pio_program();
    void unload_pio_program();
    bool configure_pio();
    void prepare_pixel_data();
    void apply_brightness(NeoPixel_Color& color) const;
    uint32_t color_to_data(const NeoPixel_Color& color) const;
    
    // 动画更新函数
    void update_fade_animation();
    void update_rainbow_animation();
    void update_chase_animation();
    void update_breathe_animation();
    void update_twinkle_animation();
    
    // PIO程序（静态）
    static const struct pio_program neopixel_program_;
};