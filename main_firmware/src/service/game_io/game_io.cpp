#include "game_io.h"
#include "../config_manager/config_manager.h"
#include "../binding_service/binding_service.h"
#include "../latency_stats.h"
#include "../../protocol/psoc/psoc.h"
#include <pico/stdlib.h>

GameIoService* GameIoService::_instance = nullptr;

GameIoService::GameIoService()
    : _serial_uart(HAL_USB_Device::getInstance(), UsbCdcPort::CDC_SERIAL),
      _light_uart(HAL_USB_Device::getInstance(), UsbCdcPort::CDC_LIGHT),
      _serial(&_serial_uart),
      _light(&_light_uart, ConfigManager::get_uint8("led.node_id")),
      _light_generation(0),
      _initialized(false),
      _touch_delay_units(0),
      _delay_refresh_us(0) {}

GameIoService* GameIoService::getInstance() {
    if (_instance == nullptr) _instance = new GameIoService();
    return _instance;
}

bool GameIoService::init(UsbWorkMode mode) {
    if (_initialized) return true;
    if (mode != UsbWorkMode::WORK_SERIAL) return false;

    const uint32_t serial_baud = ConfigManager::get_uint32("comm.serial_baud");
    const uint32_t light_baud = ConfigManager::get_uint32("comm.light_baud");
    if (!_serial_uart.init(255, 255, serial_baud) || !_light_uart.init(255, 255, light_baud)) {
        deinit();
        return false;
    }

    Mai2Serial_Config serial_config;
    serial_config.baud_rate = serial_baud;
    _serial.set_config(serial_config);

    Mai2Light_Config light_config;
    light_config.baud_rate = light_baud;
    light_config.node_id = ConfigManager::get_uint8("led.node_id");
    _light.set_config(light_config);

    if (!_serial.init() || !_light.init()) {
        deinit();
        return false;
    }

    _consume_light_state();

    // 触控延迟线预热: 全环清 0(无触摸), 读入初始延迟片数。
    _touch_delay.reset(0);
    _touch_delay_units = ConfigManager::get_uint16("comm.touch_delay_100us");
    _delay_refresh_us = 0;

    _initialized = true;
    return true;
}

void GameIoService::deinit() {
    _serial.deinit();
    _light.deinit();
    _serial_uart.deinit();
    _light_uart.deinit();
    _initialized = false;
}

void GameIoService::_consume_light_state() {
    const Mai2Light_LEDStatus* source = _light.get_led_status_array();
    bool changed = false;
    for (uint8_t index = 0; index < MAI2LIGHT_NUM_LEDS; index++) {
        const Mai2Light_LEDStatus& next = source[index];
        Mai2Light_LEDStatus& current = _light_state[index];
        if (current.color.r != next.color.r || current.color.g != next.color.g ||
            current.color.b != next.color.b || current.brightness != next.brightness ||
            current.enabled != next.enabled) {
            current = next;
            changed = true;
        }
    }
    if (changed) _light_generation++;
}

void GameIoService::task() {
    if (!_initialized) return;

    _serial.task();
    _light.task();

    // 实时触控快路：链路正常时取 36 位 on/off 掩码，经 BindingService 真实绑定表转 34 区；
    // 链路异常时上报全 0（无触摸），保持诚实。
    Psoc* psoc = Psoc::getInstance();
    const uint32_t _lt0 = time_us_32();
    const uint64_t channel_mask = psoc->link_ok() ? psoc->touch_mask() : 0;
    const uint64_t area_now = BindingService::getInstance()->map_to_areas(channel_mask);
    // 触控延迟线(100us 片, 0..100ms, UI 可配): 单拷贝环形, O(1)。延迟值 50ms 缓存刷新。
    if (_lt0 - _delay_refresh_us >= 50000u) {
        _touch_delay_units = ConfigManager::get_uint16("comm.touch_delay_100us");
        _delay_refresh_us = _lt0;
    }
    Mai2Serial_TouchState touch(_touch_delay.tick(_lt0, _touch_delay_units, area_now));
    const uint32_t _lt1 = time_us_32();
    _serial.send_touch_data(touch);
    const uint32_t _lt2 = time_us_32();
    latency_note(&g_lat_proc_us, _lt1 - _lt0);
    latency_note(&g_lat_usb_us, _lt2 - _lt1);
    _consume_light_state();
}
