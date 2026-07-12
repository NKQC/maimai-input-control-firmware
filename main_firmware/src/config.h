#pragma once

#include <stdint.h>

// ============================================================
// 全局硬件引脚与系统常量定义（权威来源：hardware.txt）
// 全部使用 constexpr，禁止使用宏
// 注意：本文件须在 <Arduino.h> 之前包含，否则 arduino-pico 变体头里的
//       PIN_SPI1_MOSI/MISO/SCK 等宏会与下面的 constexpr 同名冲突。
// ============================================================

// SPI1 - RP2040 主机连 PSoC 传感器
constexpr uint8_t PIN_SPI1_SCK = 26;
constexpr uint8_t PIN_SPI1_MOSI = 27;
constexpr uint8_t PIN_SPI1_MISO = 28;
constexpr uint8_t PIN_SPI1_CS = 29;
constexpr uint32_t SPI1_FREQ_HZ = 4000000;

// RGB 状态灯（普通 GPIO，非 WS2812）
// 实测映射（顺序蓝红绿）：GPIO20=蓝 / GPIO19=红 / GPIO18=绿（hardware.txt 标注有误，以实测为准）
constexpr uint8_t PIN_LED_G = 18;
constexpr uint8_t PIN_LED_R = 19;
constexpr uint8_t PIN_LED_B = 20;
// LED 为共阳（common anode）：GPIO 拉低点亮 → 低电平有效
constexpr bool LED_ACTIVE_HIGH = false;

// SWD（给 PSoC 编程，本里程碑只留骨架）
constexpr uint8_t PIN_SWD_IO = 16;
constexpr uint8_t PIN_SWD_CLK = 17;
constexpr uint8_t PIN_SWD_RST = 21;

// WS2812 / NeoPixel（v4 硬件改为 13/14，v3 遗留代码用的是 11，不要复用）
constexpr uint8_t PIN_WS2812_0 = 13;
constexpr uint8_t PIN_WS2812_1 = 14;

// 系统版本
constexpr const char* SYSTEM_VERSION = "4.0.0";
