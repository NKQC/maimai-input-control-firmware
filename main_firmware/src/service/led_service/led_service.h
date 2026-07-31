#pragma once

#include <stdint.h>

/**
 * LedService - RGB 状态灯服务（单例）
 * 驱动 GPIO PWM 三色状态灯，颜色掩码位为 R=1/G=2/B=4。
 */
class LedService {
public:
    static LedService* getInstance();

    // 初始化 R/G/B PWM 并熄灭状态灯
    bool init();

    void set_color(uint8_t color_mask, uint8_t brightness);
    void set_rgb(uint8_t r, uint8_t g, uint8_t b);
    void get_rgb(uint8_t& r, uint8_t& g, uint8_t& b) const;
    void push_state(uint8_t flags = 0u);

    // 兼容既有布尔状态灯接口
    void set_r(bool on);
    void set_g(bool on);
    void set_b(bool on);
    void set_rgb(bool r, bool g, bool b);
    void toggle_g();

private:
    LedService();
    LedService(const LedService&) = delete;
    LedService& operator=(const LedService&) = delete;

    void _set_duty(uint8_t pin, uint8_t duty);
    void _set_rgb_duty(uint8_t r, uint8_t g, uint8_t b);
    static void _handle_bus_led_set(uint8_t msg_id, uint16_t counter, const uint8_t* data, uint16_t len, void* ctx);

    bool _initialized;
    uint8_t _rgb[3];

    static LedService* _instance;
};
