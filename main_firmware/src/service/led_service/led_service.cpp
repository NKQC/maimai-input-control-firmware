#include "led_service.h"
#include "../../config.h"
#include <hardware/gpio.h>

LedService* LedService::_instance = nullptr;

LedService::LedService() : _initialized(false), _g_state(false) {}

LedService* LedService::getInstance() {
    if (!_instance) {
        _instance = new LedService();
    }
    return _instance;
}

void LedService::_write_pin(uint8_t pin, bool on) {
    // LED_ACTIVE_HIGH == true 时，on 对应高电平点亮；否则取反
    bool level = LED_ACTIVE_HIGH ? on : !on;
    gpio_put(pin, level);
}

bool LedService::init() {
    gpio_init(PIN_LED_R);
    gpio_set_dir(PIN_LED_R, GPIO_OUT);
    gpio_init(PIN_LED_G);
    gpio_set_dir(PIN_LED_G, GPIO_OUT);
    gpio_init(PIN_LED_B);
    gpio_set_dir(PIN_LED_B, GPIO_OUT);

    // 初始状态熄灭
    _write_pin(PIN_LED_R, false);
    _write_pin(PIN_LED_G, false);
    _write_pin(PIN_LED_B, false);
    _g_state = false;

    _initialized = true;
    return true;
}

void LedService::set_r(bool on) {
    _write_pin(PIN_LED_R, on);
}

void LedService::set_g(bool on) {
    _write_pin(PIN_LED_G, on);
    _g_state = on;
}

void LedService::set_b(bool on) {
    _write_pin(PIN_LED_B, on);
}

void LedService::set_rgb(bool r, bool g, bool b) {
    _write_pin(PIN_LED_R, r);
    _write_pin(PIN_LED_G, g);
    _write_pin(PIN_LED_B, b);
    _g_state = g;
}

void LedService::toggle_g() {
    _g_state = !_g_state;
    _write_pin(PIN_LED_G, _g_state);
}
