#pragma once

#include <stdint.h>

// ============================================================
// 全局硬件引脚与系统常量定义（权威来源：hardware.txt）
// 全部使用 constexpr，禁止使用宏
// 注意：本文件须在 <Arduino.h> 之前包含，否则 arduino-pico 变体头里的
//       PIN_SPI1_MOSI/MISO/SCK 等宏会与下面的 constexpr 同名冲突。
// ============================================================

// SPI1 - RP2040 主机连 PSoC 传感器（保留原始物理编号）
constexpr uint8_t PIN_SPI1_SCK = 26;
constexpr uint8_t PIN_SPI1_MOSI = 27;
constexpr uint8_t PIN_SPI1_MISO = 28;
constexpr uint8_t PIN_SPI1_CS = 29;
constexpr uint32_t SPI1_FREQ_HZ = 4000000;

// PSoC PIO-SPI 主机引脚
// ⚠ 本板物理连线的 MOSI/MISO 与 RP2040 硬件 SPI1 固定引脚(TX=27/RX=28)相反，
//   硬件 SPI 外设无法交换 TX/RX，故用 PIO 自定义引脚实现主机：
//     数据输出 MOSI → GPIO28 → PSoC P1.0(spi_mosi/从机输入)
//     数据输入 MISO ← GPIO27 ← PSoC P1.1(spi_miso/从机输出)
//     SCK → GPIO26 → P1.2；CS → GPIO29 → P1.3
constexpr uint8_t PIN_PSOC_SPI_SCK  = 26;
constexpr uint8_t PIN_PSOC_SPI_MOSI = 28;  // RP2040 数据输出
constexpr uint8_t PIN_PSOC_SPI_MISO = 27;  // RP2040 数据输入
constexpr uint8_t PIN_PSOC_SPI_CS   = 29;
constexpr uint32_t PSOC_SPI_SCK_HZ  = 3000000;  // SCK 频率（延后采样补偿往返延迟；扫频找硬件上限）
// Phase C 全通道 raw 快照慢路：分块流水线读取，每次 update 只读少量页，与 1kHz 触控快路交织，
// 保证触控不被 ~10ms 全快照读阻塞。每页间延时给 PSoC ISR 装载下一页的确定性窗口。
// 看门狗自持恢复(宽松策略先保可迭代)：进入运行态后置 watchdog scratch[7]=此值。
// 启动时若读到它=上次运行中被复位(任何看门狗超时:死锁/跑飞/flash异步冲突)→一律进 BOOTSEL 自动重烧。
// 读后立即清零，恢复重烧后正常启动，打破循环。首次上电 scratch=0→正常；主动重启前清 0→回 app。
constexpr uint32_t WD_RUNNING_MAGIC = 0xB007C0DEu;
constexpr uint32_t PSOC_SNAPSHOT_PAGE_DELAY_US = 60;   // 每页请求-应答间隔（< 旧 150us，仍留 ISR 余量）
constexpr uint8_t  PSOC_SNAPSHOT_PAGES_PER_PUMP = 4;   // 每次 update 读取的页数（4×~80us≈320us < 触控预算，与触控快路交织）

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

// 隔离测试开关：让出 SWD 总线给外部编程器（DAP-LINK）
//   true  = 上电即把 SWDIO(16)/SWDCLK(17) 设为高阻输入(无上下拉)、XRES(21) 保持 HIGH，
//           且不初始化 PSoC 的 SWD/SPI 通道，由外部 DAP-LINK 独占 SWD 连接/烧录 PSoC。
//           用于隔离"芯片/硬件问题"还是"RP2040 侧问题"。
//   false = 正常模式（RP2040 经 SWD 烧录 + PIO-SPI 联调）。
constexpr bool SWD_RELEASE_TO_EXTERNAL = false;

// WS2812 / NeoPixel（v4 硬件改为 13/14，v3 遗留代码用的是 11，不要复用）
constexpr uint8_t PIN_WS2812_0 = 13;
constexpr uint8_t PIN_WS2812_1 = 14;

// 系统版本
constexpr const char* SYSTEM_VERSION = "4.0.0";
