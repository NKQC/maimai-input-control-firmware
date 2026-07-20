#pragma once

#include "../../hal/usb/hal_usb_cdc_uart.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "../../protocol/mai2light/mai2light.h"
#include "../delay_line/delay_line.h"

/** Serial-mode game protocol composition and light-state consumer cache. */
class GameIoService {
public:
    static GameIoService* getInstance();

    bool init(UsbWorkMode mode);
    void deinit();
    void task();
    bool is_ready() const { return _initialized; }

    const Mai2Light_LEDStatus* light_state() const { return _light_state; }
    uint32_t light_generation() const { return _light_generation; }

private:
    GameIoService();
    GameIoService(const GameIoService&) = delete;
    GameIoService& operator=(const GameIoService&) = delete;

    void _consume_light_state();

    static GameIoService* _instance;
    HAL_USB_CDC_UART _serial_uart;
    HAL_USB_CDC_UART _light_uart;
    Mai2Serial _serial;
    Mai2Light _light;
    Mai2Light_LEDStatus _light_state[MAI2LIGHT_NUM_LEDS];
    uint32_t _light_generation;
    bool _initialized;

    // 触控串口延迟线(100us 片, 0..100ms), 单拷贝环形; 延迟值 50ms 缓存刷新一次避免频繁查表。
    DelayLine<uint64_t> _touch_delay;
    uint16_t _touch_delay_units;
    uint32_t _delay_refresh_us;
};
