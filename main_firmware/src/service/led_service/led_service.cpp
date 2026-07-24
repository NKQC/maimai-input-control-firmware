#include "led_service.h"
#include "../../config.h"
#include <hardware/gpio.h>
#include <hardware/pwm.h>

namespace {
constexpr uint16_t PWM_WRAP = 255;
constexpr uint8_t COLOR_R = 1u;
constexpr uint8_t COLOR_G = 2u;
constexpr uint8_t COLOR_B = 4u;
}

LedService* LedService::_instance = nullptr;

LedService::LedService() : _initialized(false), _g_state(false) {}

LedService* LedService::getInstance() {
    if (!_instance) {
        _instance = new LedService();
    }
    return _instance;
}

void LedService::_set_duty(uint8_t pin, uint8_t duty) {
    const uint16_t level = LED_ACTIVE_HIGH ? duty : (PWM_WRAP - duty);
    pwm_set_gpio_level(pin, level);
}

void LedService::_set_rgb_duty(uint8_t r, uint8_t g, uint8_t b) {
    _set_duty(PIN_LED_R, r);
    _set_duty(PIN_LED_G, g);
    _set_duty(PIN_LED_B, b);
    _g_state = g != 0u;
}

bool LedService::init() {
    if (_initialized) {
        return true;
    }

    const uint slice_rg = pwm_gpio_to_slice_num(PIN_LED_R);
    const uint slice_b = pwm_gpio_to_slice_num(PIN_LED_B);

    gpio_set_function(PIN_LED_R, GPIO_FUNC_PWM);
    gpio_set_function(PIN_LED_G, GPIO_FUNC_PWM);
    gpio_set_function(PIN_LED_B, GPIO_FUNC_PWM);

    // GPIO18/19 共享一个 slice 的两个 channel，必须先统一设置 wrap 再启用。
    pwm_set_wrap(slice_rg, PWM_WRAP);
    if (slice_b != slice_rg) {
        pwm_set_wrap(slice_b, PWM_WRAP);
    }
    _set_rgb_duty(0u, 0u, 0u);

    pwm_set_enabled(slice_rg, true);
    if (slice_b != slice_rg) {
        pwm_set_enabled(slice_b, true);
    }

    _initialized = true;
    return true;
}

void LedService::set_color(uint8_t color_mask, uint8_t brightness) {
    _set_rgb_duty(
        (color_mask & COLOR_R) ? brightness : 0u,
        (color_mask & COLOR_G) ? brightness : 0u,
        (color_mask & COLOR_B) ? brightness : 0u);
}

void LedService::set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    _set_rgb_duty(r, g, b);
}

void LedService::set_r(bool on) {
    _set_duty(PIN_LED_R, on ? PWM_WRAP : 0u);
}

void LedService::set_g(bool on) {
    _set_duty(PIN_LED_G, on ? PWM_WRAP : 0u);
    _g_state = on;
}

void LedService::set_b(bool on) {
    _set_duty(PIN_LED_B, on ? PWM_WRAP : 0u);
}

void LedService::set_rgb(bool r, bool g, bool b) {
    _set_rgb_duty(
        r ? PWM_WRAP : 0u,
        g ? PWM_WRAP : 0u,
        b ? PWM_WRAP : 0u);
}

void LedService::toggle_g() {
    set_g(!_g_state);
}
