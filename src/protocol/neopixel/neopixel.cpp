#include "neopixel.h"
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <cmath>
#include <algorithm>

// NeoPixel PIO程序
static const uint16_t neopixel_program_instructions[] = {
    //     .wrap_target
    0x6221, //  0: out    x, 1            side 0 [2] 
    0x1123, //  1: jmp    !x, 3           side 1 [1] 
    0x1400, //  2: jmp    0               side 1 [4] 
    0xa442, //  3: nop                    side 0 [4] 
    //     .wrap
};

static const struct pio_program neopixel_program = {
    .instructions = neopixel_program_instructions,
    .length = 4,
    .origin = -1,
};

const struct pio_program NeoPixel::neopixel_program_ = neopixel_program;

NeoPixel::NeoPixel(HAL_PIO* pio_hal, uint16_t num_leds, NeoPixel_Type type)
    : pio_hal_(pio_hal), num_leds_(num_leds), type_(type),
      initialized_(false), pio_sm_(0), pio_offset_(0), brightness_(255),
      animation_running_(false), animation_step_(0) {
    
    pixels_.resize(num_leds_);
    pixel_data_.resize(num_leds_);
    animation_start_colors_.resize(num_leds_);
}

NeoPixel::~NeoPixel() {
    deinit();
}

bool NeoPixel::init() {
    if (initialized_) {
        return true;
    }
    
    if (!pio_hal_ || !pio_hal_->is_ready()) {
        return false;
    }
    
    // 加载PIO程序
    if (!load_pio_program()) {
        return false;
    }
    
    // 获取状态机
    if (!pio_hal_->claim_sm(&pio_sm_)) {
        unload_pio_program();
        return false;
    }
    
    // 配置PIO
    if (!configure_pio()) {
        pio_hal_->unclaim_sm(pio_sm_);
        unload_pio_program();
        return false;
    }
    
    initialized_ = true;
    return true;
}

void NeoPixel::deinit() {
    if (initialized_) {
        // 停止动画
        stop_animation();
        
        // 关闭所有LED
        clear_all();
        show();
        
        // 停止状态机
        pio_hal_->sm_set_enabled(pio_sm_, false);
        
        // 释放资源
        pio_hal_->unclaim_sm(pio_sm_);
        unload_pio_program();
        
        initialized_ = false;
    }
}

bool NeoPixel::is_ready() const {
    return initialized_;
}

bool NeoPixel::set_pixel(uint16_t index, const NeoPixel_Color& color) {
    if (!is_ready() || index >= num_leds_) {
        return false;
    }
    
    pixels_[index] = color;
    return true;
}

bool NeoPixel::set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    return set_pixel(index, NeoPixel_Color(r, g, b, w));
}

bool NeoPixel::set_all_pixels(const NeoPixel_Color& color) {
    std::fill(pixels_.begin(), pixels_.end(), color);
    return true;
}

bool NeoPixel::clear_all() {
    return set_all_pixels(NeoPixel_Color(0, 0, 0, 0));
}

bool NeoPixel::set_pixels(uint16_t start_index, const std::vector<NeoPixel_Color>& colors) {
    if (!is_ready() || start_index >= num_leds_) {
        return false;
    }
    
    for (size_t i = 0; i < colors.size() && (start_index + i) < num_leds_; i++) {
        pixels_[start_index + i] = colors[i];
    }
    
    return true;
}

bool NeoPixel::set_range(uint16_t start_index, uint16_t count, const NeoPixel_Color& color) {
    if (!is_ready() || start_index >= num_leds_) {
        return false;
    }
    
    uint16_t end_index = std::min((uint16_t)(start_index + count), num_leds_);
    for (uint16_t i = start_index; i < end_index; i++) {
        pixels_[i] = color;
    }
    
    return true;
}

NeoPixel_Color NeoPixel::get_pixel(uint16_t index) const {
    if (index >= num_leds_) {
        return NeoPixel_Color();
    }
    
    return pixels_[index];
}

bool NeoPixel::show() {
    // 准备像素数据
    prepare_pixel_data();
    
    // 使用完全非阻塞的方式发送数据
    for (uint16_t i = 0; i < num_leds_; i++) {
        uint32_t wait_start = time_us_32();
        while (pio_hal_->sm_is_tx_fifo_full(pio_sm_)) {
            // 检查是否超时
            if (time_us_32() - wait_start > NEOPIXEL_WAIT_TIMEOUT_US) return false;  // 等待超时，返回失败
            // 短暂让步CPU
            tight_loop_contents();
        }
        pio_hal_->sm_put_nonblocking(pio_sm_, pixel_data_[i]);
    }
    
    return true;  // 所有数据发送成功
}

void NeoPixel::set_brightness(uint8_t brightness) {
    brightness_ = brightness;
}

uint8_t NeoPixel::get_brightness() const {
    return brightness_;
}

bool NeoPixel::start_animation(const NeoPixel_Animation& animation) {
    if (!is_ready()) {
        return false;
    }
    
    // 保存当前颜色作为动画起始点
    animation_start_colors_ = pixels_;
    
    current_animation_ = animation;
    animation_running_ = true;
    animation_start_time_ = time_us_32();
    animation_last_update_ = animation_start_time_;
    animation_step_ = 0;
    
    // 设置亮度
    set_brightness(animation.brightness);
    
    return true;
}

bool NeoPixel::stop_animation() {
    animation_running_ = false;
    return true;
}

bool NeoPixel::is_animation_running() const {
    return animation_running_;
}

bool NeoPixel::fade_to_color(const NeoPixel_Color& target_color, uint32_t duration_ms) {
    NeoPixel_Animation animation;
    animation.effect = NEOPIXEL_EFFECT_FADE;
    animation.duration_ms = duration_ms;
    animation.speed_ms = 20;  // 50fps
    animation.color1 = target_color;
    animation.loop = false;
    animation.brightness = brightness_;
    
    return start_animation(animation);
}

bool NeoPixel::rainbow_cycle(uint32_t speed_ms, bool loop) {
    NeoPixel_Animation animation;
    animation.effect = NEOPIXEL_EFFECT_RAINBOW;
    animation.duration_ms = loop ? 0 : 5000;  // 5秒一个周期
    animation.speed_ms = speed_ms;
    animation.loop = loop;
    animation.brightness = brightness_;
    
    return start_animation(animation);
}

bool NeoPixel::color_chase(const NeoPixel_Color& color, uint32_t speed_ms, bool loop) {
    NeoPixel_Animation animation;
    animation.effect = NEOPIXEL_EFFECT_CHASE;
    animation.duration_ms = loop ? 0 : num_leds_ * speed_ms;
    animation.speed_ms = speed_ms;
    animation.color1 = color;
    animation.loop = loop;
    animation.brightness = brightness_;
    
    return start_animation(animation);
}

bool NeoPixel::breathe_effect(const NeoPixel_Color& color, uint32_t period_ms, bool loop) {
    NeoPixel_Animation animation;
    animation.effect = NEOPIXEL_EFFECT_BREATHE;
    animation.duration_ms = loop ? 0 : period_ms;
    animation.speed_ms = 20;  // 50fps
    animation.color1 = color;
    animation.loop = loop;
    animation.brightness = brightness_;
    
    return start_animation(animation);
}

bool NeoPixel::twinkle_effect(const NeoPixel_Color& color, uint32_t speed_ms, bool loop) {
    NeoPixel_Animation animation;
    animation.effect = NEOPIXEL_EFFECT_TWINKLE;
    animation.duration_ms = loop ? 0 : 10000;  // 10秒
    animation.speed_ms = speed_ms;
    animation.color1 = color;
    animation.loop = loop;
    animation.brightness = brightness_;
    
    return start_animation(animation);
}

void NeoPixel::task() {
    if (!is_ready() || !animation_running_) {
        return;
    }
    
    uint32_t current_time = time_us_32();
    uint32_t elapsed_time = current_time - animation_start_time_;
    uint32_t update_interval = current_animation_.speed_ms * 1000;  // 转换为微秒
    
    // 检查是否需要更新
    if (current_time - animation_last_update_ < update_interval) {
        return;
    }
    
    animation_last_update_ = current_time;
    
    // 检查动画是否结束
    if (!current_animation_.loop && current_animation_.duration_ms > 0) {
        if (elapsed_time >= current_animation_.duration_ms * 1000) {
            stop_animation();
            return;
        }
    }
    
    // 根据动画类型更新
    switch (current_animation_.effect) {
        case NEOPIXEL_EFFECT_FADE:
            update_fade_animation();
            break;
        case NEOPIXEL_EFFECT_RAINBOW:
            update_rainbow_animation();
            break;
        case NEOPIXEL_EFFECT_CHASE:
            update_chase_animation();
            break;
        case NEOPIXEL_EFFECT_BREATHE:
            update_breathe_animation();
            break;
        case NEOPIXEL_EFFECT_TWINKLE:
            update_twinkle_animation();
            break;
        default:
            break;
    }
    
    // 显示更新
    show();
    
    animation_step_++;
}

// 静态工具函数
NeoPixel_Color NeoPixel::hsv_to_rgb(uint16_t hue, uint8_t saturation, uint8_t value) {
    uint8_t region, remainder, p, q, t;
    
    if (saturation == 0) {
        return NeoPixel_Color(value, value, value);
    }
    
    region = hue / 43;
    remainder = (hue - (region * 43)) * 6;
    
    p = (value * (255 - saturation)) >> 8;
    q = (value * (255 - ((saturation * remainder) >> 8))) >> 8;
    t = (value * (255 - ((saturation * (255 - remainder)) >> 8))) >> 8;
    
    switch (region) {
        case 0:
            return NeoPixel_Color(value, t, p);
        case 1:
            return NeoPixel_Color(q, value, p);
        case 2:
            return NeoPixel_Color(p, value, t);
        case 3:
            return NeoPixel_Color(p, q, value);
        case 4:
            return NeoPixel_Color(t, p, value);
        default:
            return NeoPixel_Color(value, p, q);
    }
}

NeoPixel_Color NeoPixel::wheel_color(uint8_t pos) {
    pos = 255 - pos;
    if (pos < 85) {
        return NeoPixel_Color(255 - pos * 3, 0, pos * 3);
    }
    if (pos < 170) {
        pos -= 85;
        return NeoPixel_Color(0, pos * 3, 255 - pos * 3);
    }
    pos -= 170;
    return NeoPixel_Color(pos * 3, 255 - pos * 3, 0);
}

NeoPixel_Color NeoPixel::blend_colors(const NeoPixel_Color& color1, const NeoPixel_Color& color2, uint8_t blend) {
    uint8_t inv_blend = 255 - blend;
    return NeoPixel_Color(
        (color1.r * inv_blend + color2.r * blend) >> 8,
        (color1.g * inv_blend + color2.g * blend) >> 8,
        (color1.b * inv_blend + color2.b * blend) >> 8,
        (color1.w * inv_blend + color2.w * blend) >> 8
    );
}

// 私有方法实现
bool NeoPixel::load_pio_program() {
    return pio_hal_->load_program(&neopixel_program_, &pio_offset_);
}

void NeoPixel::unload_pio_program() {
    pio_hal_->unload_program(&neopixel_program_, pio_offset_);
}

bool NeoPixel::configure_pio() {
    // WS2812时序要求：
    // T0H = 0.4us, T0L = 0.85us (总周期 1.25us)
    // T1H = 0.8us, T1L = 0.45us (总周期 1.25us)
    // 每个bit周期 = 1.25us = 800kHz
    // PIO程序中每个指令的延迟周期数：
    // side 0 [2]: 3个周期 (0.375us @ 8MHz)
    // side 1 [1]: 2个周期 (0.25us @ 8MHz) 
    // side 1 [4]: 5个周期 (0.625us @ 8MHz)
    // side 0 [4]: 5个周期 (0.625us @ 8MHz)
    
    float clock_freq = clock_get_hz(clk_sys);
    // 目标频率8MHz，使PIO指令周期为125ns
    // 这样[2]延迟=375ns, [4]延迟=625ns, [1]延迟=250ns
    float target_freq = 8000000.0f;  // 8MHz
    float clkdiv = clock_freq / target_freq;
    
    // 创建统一配置
    PIOStateMachineConfig config;
    config.out_base = 0;  // GPIO引脚由PIO HAL管理
    config.out_count = 1;
    config.sideset_base = 0;  // GPIO引脚由PIO HAL管理
    config.sideset_bit_count = 1;
    config.sideset_optional = false;
    config.sideset_pindirs = false;
    config.clkdiv = clkdiv;
    config.wrap_target = pio_offset_;
    config.wrap = pio_offset_ + neopixel_program_.length - 1;
    config.program_offset = pio_offset_;
    config.enabled = true;  // 直接启用状态机
    
    // 使用统一接口配置状态机
    return pio_hal_->sm_configure(pio_sm_, config);
}

void NeoPixel::prepare_pixel_data() {
    for (uint16_t i = 0; i < num_leds_; i++) {
        NeoPixel_Color color = pixels_[i];
        apply_brightness(color);
        pixel_data_[i] = color_to_data(color);
    }
}

void NeoPixel::apply_brightness(NeoPixel_Color& color) const {
    if (brightness_ == 255) {
        return;
    }
    
    color.r = (color.r * brightness_) >> 8;
    color.g = (color.g * brightness_) >> 8;
    color.b = (color.b * brightness_) >> 8;
    if (type_ == NEOPIXEL_RGBW) {
        color.w = (color.w * brightness_) >> 8;
    }
}

uint32_t NeoPixel::color_to_data(const NeoPixel_Color& color) const {
    if (type_ == NEOPIXEL_RGBW) {
        // RGBW: 32位数据
        return (color.g << 24) | (color.r << 16) | (color.b << 8) | color.w;
    } else {
        // RGB: 24位数据，左移8位对齐到32位
        return ((color.g << 16) | (color.r << 8) | color.b) << 8;
    }
}

void NeoPixel::update_fade_animation() {
    uint32_t elapsed_time = (time_us_32() - animation_start_time_) / 1000;
    uint8_t progress = std::min(255UL, static_cast<uint32_t>((elapsed_time * 255) / current_animation_.duration_ms));
    
    for (uint16_t i = 0; i < num_leds_; i++) {
        pixels_[i] = blend_colors(animation_start_colors_[i], current_animation_.color1, progress);
    }
    
    if (progress >= 255) {
        stop_animation();
    }
}

void NeoPixel::update_rainbow_animation() {
    uint8_t hue_offset = (animation_step_ * 5) % 256;  // 每步增加5度
    
    for (uint16_t i = 0; i < num_leds_; i++) {
        uint8_t hue = (hue_offset + (i * 256 / num_leds_)) % 256;
        pixels_[i] = hsv_to_rgb(hue, 255, 255);
    }
}

void NeoPixel::update_chase_animation() {
    clear_all();
    
    uint16_t pos = animation_step_ % num_leds_;
    set_pixel(pos, current_animation_.color1);
    
    // 添加尾迹效果
    for (int32_t i = 1; i <= 3; i++) {
        uint16_t tail_pos = (pos - i + num_leds_) % num_leds_;
        NeoPixel_Color tail_color = current_animation_.color1;
        tail_color.r >>= i;
        tail_color.g >>= i;
        tail_color.b >>= i;
        set_pixel(tail_pos, tail_color);
    }
}

void NeoPixel::update_breathe_animation() {
    uint32_t elapsed_time = (time_us_32() - animation_start_time_) / 1000;
    float phase = (elapsed_time % current_animation_.duration_ms) * 2.0f * M_PI / current_animation_.duration_ms;
    uint8_t brightness = (sin(phase) + 1.0f) * 127.5f;
    
    NeoPixel_Color color = current_animation_.color1;
    color.r = (color.r * brightness) >> 8;
    color.g = (color.g * brightness) >> 8;
    color.b = (color.b * brightness) >> 8;
    
    set_all_pixels(color);
}

void NeoPixel::update_twinkle_animation() {
    // 随机闪烁效果
    for (uint16_t i = 0; i < num_leds_; i++) {
        if (rand() % 100 < 5) {  // 5%概率闪烁
            if (pixels_[i].r == 0 && pixels_[i].g == 0 && pixels_[i].b == 0) {
                pixels_[i] = current_animation_.color1;
            } else {
                pixels_[i] = NeoPixel_Color(0, 0, 0);
            }
        }
    }
}