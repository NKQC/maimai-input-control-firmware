#include "sensor_link.h"
#include "../../config.h"
#include "../../hal/spi/hal_spi.h"
#include <Arduino.h>

namespace {
constexpr uint32_t SENSOR_LINK_UPDATE_INTERVAL_MS = 20;
}

SensorLink* SensorLink::_instance = nullptr;

SensorLink::SensorLink()
    : _spi(nullptr), _initialized(false), _link_ok(false), _last_update_ms(0), _seq(0) {}

SensorLink* SensorLink::getInstance() {
    if (!_instance) {
        _instance = new SensorLink();
    }
    return _instance;
}

bool SensorLink::init(HAL_SPI* spi) {
    if (!spi) {
        return false;
    }
    _spi = spi;

    if (!_spi->init(PIN_SPI1_SCK, PIN_SPI1_MOSI, PIN_SPI1_MISO, SPI1_FREQ_HZ)) {
        return false;
    }
    _spi->set_format(8, 0, 0, 0);
    _spi->set_cs_pin(PIN_SPI1_CS, true);

    _initialized = true;
    return true;
}

void SensorLink::update() {
    if (!_initialized) {
        return;
    }

    uint32_t now = millis();
    if (now - _last_update_ms < SENSOR_LINK_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_ms = now;

    SensorFrame tx;
    SensorFrame rx;
    tx.clear();
    tx.cmd = static_cast<uint8_t>(SensorCmd::PING);
    tx.seq = _seq;

    _spi->cs_select();
    _spi->transfer(reinterpret_cast<const uint8_t*>(&tx), reinterpret_cast<uint8_t*>(&rx), sizeof(SensorFrame));
    _spi->cs_deselect();

    // 只更新链路状态，不直接驱动任何同层服务；由上层读取 link_ok() 决定指示方式
    _link_ok = (rx.magic == SENSOR_FRAME_MAGIC) && (rx.cmd == static_cast<uint8_t>(SensorCmd::PONG));

    _seq++;
}
