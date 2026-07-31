#pragma once

#include "../../hal/usb/hal_usb_cdc_uart.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "../../protocol/mai2light/mai2light.h"
#include "../../protocol/host_cmd/host_cmd.h"
#include "../delay_line/delay_line.h"
#include "../led_map/led_map.h"

/** Serial-mode game protocol composition and light-state consumer cache. */
class GameIoService {
public:
    static GameIoService* getInstance();

    bool init(UsbWorkMode mode);
    void deinit();
    void task();

    // 注册 mai2 串口状态 host_cmd(MAI2_GET_STATE / MAI2_SET_SEND_EN)。与 init 分离:
    // HID 模式不跑 game_io 也要能被上位机查询, 此时如实回 STOPPED。
    void register_host_cmds();
    bool is_ready() const { return _initialized; }
    // 触控数据是否真的在发的唯一定义；调用方不得自行重新组合 is_ready()/get_serial_ok()，以免判定分叉。
    bool mai2_touch_sending() const;

    const Mai2Light_LEDStatus* light_state() const { return _light_state; }
    uint32_t light_generation() const { return _light_generation; }

private:
    GameIoService();
    GameIoService(const GameIoService&) = delete;
    GameIoService& operator=(const GameIoService&) = delete;

    void _consume_light_state();
    void _process_serial_reset();

    static void _handle_mai2_command(Mai2Serial_Command command, const uint8_t* params, uint8_t param_len);
    static void _handle_mai2_get_state(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_mai2_set_send_en(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_led_get(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_led_set_region(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_led_preview(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);

    static GameIoService* _instance;
    HAL_USB_CDC_UART _serial_uart;
    HAL_USB_CDC_UART _light_uart;
    Mai2Serial _serial;
    Mai2Light _light;
    Mai2Light_LEDStatus _light_state[MAI2LIGHT_NUM_LEDS];
    // 交给 LedMapService 的 flat RGB(11×3): 映射服务不依赖协议头, 只吃裸颜色。
    uint8_t _light_rgb[MAI2LIGHT_NUM_LEDS * 3];
    uint32_t _light_generation;
    bool _led_map_ready;
    bool _initialized;

    // 触控串口延迟线(100us 片, 0..100ms), 单拷贝环形; 延迟值 50ms 缓存刷新一次避免频繁查表。
    DelayLine<uint64_t> _touch_delay;
    uint16_t _touch_delay_units;
    uint32_t _delay_refresh_us;
    uint8_t _serial_reset_requests;
};
