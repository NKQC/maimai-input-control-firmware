#pragma once

#include <stdint.h>
#include <stddef.h>

class HAL_SPI;

/**
 * SensorLink - RP2040 <-> PSoC 传感器 SPI 通信服务（单例）
 * 本里程碑只实现定长 ping/pong 帧的骨架，DATA 帧留给后续里程碑。
 *
 * 解耦原则：本服务只依赖下层 HAL_SPI（由上层以指针注入），不耦合任何同层服务。
 * 链路状态通过 link_ok() 对外暴露，由上层（main/app）读取后自行驱动 LED 等。
 */

// 帧魔数，用于校验帧头合法性
static constexpr uint8_t SENSOR_FRAME_MAGIC = 0xA5;

enum class SensorCmd : uint8_t {
    PING = 0x01,
    PONG = 0x02,
    DATA = 0x10, // 预留 DATA 帧用于后续里程碑
};

static constexpr size_t SENSOR_FRAME_PAYLOAD_SIZE = 4;

// 定长协议帧，禁止动态内存
struct SensorFrame {
    uint8_t magic = SENSOR_FRAME_MAGIC;
    uint8_t cmd = 0;
    uint8_t seq = 0;
    uint8_t payload[SENSOR_FRAME_PAYLOAD_SIZE] = {};

    void clear() {
        magic = SENSOR_FRAME_MAGIC;
        cmd = 0;
        seq = 0;
        for (auto& b : payload) b = 0;
    }
};

class SensorLink {
public:
    static SensorLink* getInstance();

    // 依赖注入：只接收下层 HAL_SPI，由上层传入
    bool init(HAL_SPI* spi);

    // 20ms 节流，内部发送 PING 并判定 PONG，刷新链路状态
    void update();

    // 对外暴露链路状态，供上层决定如何指示（如驱动 LED）
    bool link_ok() const { return _link_ok; }

private:
    SensorLink();
    SensorLink(const SensorLink&) = delete;
    SensorLink& operator=(const SensorLink&) = delete;

    HAL_SPI* _spi;
    bool _initialized;
    bool _link_ok;

    uint32_t _last_update_ms;
    uint8_t _seq;

    static SensorLink* _instance;
};
