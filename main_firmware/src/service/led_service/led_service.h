#pragma once

#include <stdint.h>

/**
 * LedService - RGB 状态灯服务（单例）
 * 驱动普通 GPIO 三色状态灯（非 WS2812），引脚定义见 config.h
 */
class LedService {
public:
    static LedService* getInstance();

    // 初始化 R/G/B 三个引脚为输出并熄灭
    bool init();

    void set_r(bool on);
    void set_g(bool on);
    void set_b(bool on);
    void set_rgb(bool r, bool g, bool b);
    void toggle_g();

private:
    LedService();
    LedService(const LedService&) = delete;
    LedService& operator=(const LedService&) = delete;

    // 内部工具函数：根据 LED_ACTIVE_HIGH 换算实际写入电平，供上层接口复用
    void _write_pin(uint8_t pin, bool on);

    bool _initialized;
    bool _g_state;

    static LedService* _instance;
};
