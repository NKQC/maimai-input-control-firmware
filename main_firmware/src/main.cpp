#include "config.h"   // 必须在 Arduino.h 之前，避免 PIN_SPI1_* 宏冲突

#include <Arduino.h>
#include <hardware/watchdog.h>
#include <cstdio>

#include "hal/usb/hal_usb.h"
#include "hal/spi/hal_spi.h"

extern "C" {
#include "hal/global_irq.h"
}

#include "service/led_service/led_service.h"
#include "service/sensor_link/sensor_link.h"
#include "driver/swd/swd.h"

// Watchdog 超时时间
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 5000;

// 里程碑2 bring-up 开关：为 true 时启动阶段用 PIO SWD 获取 PSoC 并读 IDCODE，
// 结果经 RGB 状态灯 + USB CDC 输出，用于实机验证 SWD 传输层。验证完成后置 false。
static constexpr bool SWD_BRINGUP_TEST = true;

static SwdProgrammer s_swd(PIN_SWD_IO, PIN_SWD_CLK, PIN_SWD_RST);
static bool s_swd_init_ok = false;
static bool s_swd_idcode_ok = false;
static bool s_swd_acquired = false;
static bool s_swd_silicon_ok = false;
static uint32_t s_swd_idcode = 0;
static uint32_t s_swd_silicon = 0;

static void swd_bringup() {
    s_swd_init_ok = s_swd.init();
    if (!s_swd_init_ok) return;

    // acquire() 内部会复位芯片并在启动窗口内读取 IDCODE，成功后暴露实际 IDCODE。
    s_swd_acquired = s_swd.acquire();
    s_swd_idcode = s_swd.last_idcode();
    s_swd_idcode_ok = (s_swd_idcode == 0x0BC11477u);
    if (s_swd_acquired) {
        s_swd_silicon_ok = s_swd.read_silicon_id(&s_swd_silicon);
    }
}

void setup() {
    global_irq_init();
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    HAL_USB_Device::getInstance()->init();
    LedService::getInstance()->init();

    HAL_SPI1* spi1 = HAL_SPI1::getInstance();
    SensorLink::getInstance()->init(spi1);

    if (SWD_BRINGUP_TEST) {
        swd_bringup();
    }
}

void loop() {
    if (SWD_BRINGUP_TEST) {
        // 每 1.2s 单独点亮一个通道用于识别 RGB↔GPIO 映射，并经 CDC 打印 SWD 状态 + 当前通道。
        LedService* led = LedService::getInstance();
        static uint32_t last = 0;
        static uint8_t phase = 0;
        if (millis() - last > 1200) {
            last = millis();
            led->set_rgb(phase == 0, phase == 1, phase == 2);

            static const char* const names[3] = {"R", "G", "B"};
            static const uint8_t pins[3] = {PIN_LED_R, PIN_LED_G, PIN_LED_B};
            char buf[144];
            int n = snprintf(buf, sizeof(buf),
                "SWD init=%d idcode=0x%08lX ok=%d acquired=%d silicon=0x%08lX(%d) | RGB: %s -> GPIO%u ON\r\n",
                s_swd_init_ok ? 1 : 0, (unsigned long)s_swd_idcode,
                s_swd_idcode_ok ? 1 : 0, s_swd_acquired ? 1 : 0,
                (unsigned long)s_swd_silicon, s_swd_silicon_ok ? 1 : 0,
                names[phase], (unsigned)pins[phase]);
            if (n > 0) {
                HAL_USB_Device::getInstance()->cdc_write((const uint8_t*)buf, (size_t)n);
            }
            phase = (uint8_t)((phase + 1) % 3);
        }

        watchdog_update();
        return;
    }

    // 生产路径（里程碑1）：main 作为组装层，读取通信服务状态点亮状态灯。
    SensorLink* sensor = SensorLink::getInstance();
    sensor->update();

    bool ok = sensor->link_ok();
    LedService::getInstance()->set_rgb(!ok, ok, false);

    watchdog_update();
}
