#include "config.h"   // 必须在 Arduino.h 之前，避免 PIN_SPI1_* 宏冲突

#include <Arduino.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>
#include <pico/bootrom.h>
#include "config.h"

#include "hal/usb/hal_usb.h"
#include "service/usb_comm/usb_comm.h"
#include "service/config_manager/config_manager.h"
#include "service/app_config/app_config.h"
#include "service/sensor_link/sensor_link.h"
#include "service/binding_service/binding_service.h"
#include "service/game_io/game_io.h"
#include "service/led_service/led_service.h"
#include "service/psoc_updater/psoc_updater.h"
#include "service/csd_config/csd_config.h"
#include "service/usb_debug.h"
#include "protocol/psoc/psoc.h"

extern "C" {
#include "hal/global_irq.h"
}

static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 5000;

// ★主机租约超时★：上位机每~1s PING 续期(刷新 g_last_host_cmd_ms)。超过本时长未收到任何
// 主机指令 = 上位机已丢失(关闭/崩溃)→ 固件自停遥测流,避免遗留流淹没下次连接的 DEVICE_INFO。
static constexpr uint32_t HOST_LEASE_TIMEOUT_MS = 3000;

// ★双核 flash 竞态根治(照抄 v3.1)★：core0 跑 Arduino setup/loop(USB+触控快路+flash 落地)，
// core1 仅注册为 lockout victim 并空转。config_manager 的 flash 写用
// save_and_disable_interrupts()+multicore_lockout_start_blocking() 暂停 core1，
// 使 XIP 擦写期间两核都不取指/访问总线→flash 安全完成、USB 快速恢复。
// 前提是 core1 必须已 multicore_lockout_victim_init()，否则 lockout 永久死锁。
#define CORE1_STACK_SIZE_BYTES 0x2000u
static uint32_t __attribute__((aligned(8))) core1_stack[CORE1_STACK_SIZE_BYTES / sizeof(uint32_t)];

static void core1_entry() {
    // 注册本核为 lockout 受害者：core0 flash 写时经 SIO IRQ 暂停本核(IRQ 抢占, 与下面死循环无关)。
    multicore_lockout_victim_init();
    // ★core1 接管 PSoC SPI★：固定 1ms 周期独占传感器循环(触控快路 + 快照慢路 + 命令队列消费),
    // 保证传感器/键盘总延迟每周期一致。core0 自此不再触碰 SPI, 只经 seqlock 读共享态、经队列投递指令。
    Psoc::getInstance()->core1_run();   // 永不返回
}

// 隔离测试：SWDIO/SWDCLK 高阻，XRES 上拉，让外部 DAP-LINK 独占目标。
static void release_swd_to_external() {
    gpio_init(PIN_SWD_IO);
    gpio_set_dir(PIN_SWD_IO, GPIO_IN);
    gpio_disable_pulls(PIN_SWD_IO);

    gpio_init(PIN_SWD_CLK);
    gpio_set_dir(PIN_SWD_CLK, GPIO_IN);
    gpio_disable_pulls(PIN_SWD_CLK);

    gpio_init(PIN_SWD_RST);
    gpio_set_dir(PIN_SWD_RST, GPIO_IN);
    gpio_pull_up(PIN_SWD_RST);
}

void setup() {
    // ★看门狗自持恢复(宽松策略，先确保可迭代)★：scratch[7] 在进入运行态后被置 WD_RUNNING_MAGIC。
    // 若本次启动读到它=上次运行中被复位(任何看门狗超时:死锁/跑飞/flash异步冲突)，一律进 BOOTSEL
    // 便于自动重烧恢复。读后立即清零→恢复重烧后正常启动，打破死循环。
    const uint32_t boot_flag = watchdog_hw->scratch[7];
    watchdog_hw->scratch[7] = 0u;
    if (boot_flag == WD_RUNNING_MAGIC) {
        reset_usb_boot(0, 0);
    }

    global_irq_init();
    LedService::getInstance()->init();

    if (SWD_RELEASE_TO_EXTERNAL) {
        release_swd_to_external();
        HAL_USB_Device::getInstance()->init();
        watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
        return;
    }

    app_config_register_schema();
    ConfigManager::initialize();

    Psoc* psoc = Psoc::getInstance();
    psoc->init();

    // USB 枚举前完成阻塞式 SWD bring-up；结果由 PsocUpdater 统一保存并经 HostCmd 上报。
    PsocUpdater::getInstance()->run(psoc);

    // 枚举前取得首份真实快照，使 DEVICE_INFO 的运行态字段立即可用。
    psoc->update();
    PsocUpdater::getInstance()->update();

    // ★无状态 PSoC★：RP2040 为唯一真相源。从 flash 载入 CSD 配置；实际下发延到 loop()
    // 首次 SPI 链路稳定(psoc->link_ok())后一次性执行——setup 单次 update 时链路可能尚未就绪。
    CsdConfig::getInstance()->init();

    HAL_USB_Device::getInstance()->init();
    UsbComm::getInstance()->init();
    SensorLink::getInstance()->init();
    BindingService::getInstance()->init();

    const UsbWorkMode work_mode = (ConfigManager::get_uint8("mode.work") ==
        static_cast<uint8_t>(UsbWorkMode::WORK_HID))
        ? UsbWorkMode::WORK_HID : UsbWorkMode::WORK_SERIAL;
    if (work_mode == UsbWorkMode::WORK_SERIAL) {
        GameIoService::getInstance()->init(work_mode);
    }

    // ★启动 core1★：必须在任何 flash 写(loop 的 save_config_task)之前完成，
    // 使 multicore_lockout_start_blocking() 有已注册的 victim 可暂停，而非永久死锁。
    multicore_launch_core1_with_stack(core1_entry, core1_stack, sizeof(core1_stack));

    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    // 进入运行态标记：此后任何看门狗超时复位，启动时即判为"运行中死锁/异常"→进 BOOTSEL 自恢复。
    watchdog_hw->scratch[7] = WD_RUNNING_MAGIC;
}

void loop() {
    g_usb_dbg.loop_count++;
    HAL_USB_Device::getInstance()->task();

    // ★控制通道 BOOTSEL★：EP0 请求 0x52 置位后，泵几轮 task 让 ACK 发出，再进烧录。
    // 因 EP0 在 bulk vendor 死后仍存活，此路径免去 vendor 卡死时的物理 BOOTSEL。
    if (g_bootsel_request) {
        for (uint8_t i = 0; i < 32; i++) { HAL_USB_Device::getInstance()->task(); sleep_ms(2); }
        watchdog_hw->scratch[7] = 0u;
        reset_usb_boot(0, 0);
    }

    if (SWD_RELEASE_TO_EXTERNAL) {
        UsbComm::getInstance()->update();
        SensorLink::getInstance()->tick();
        static uint32_t last = 0;
        static bool on = false;
        if (millis() - last > 500) {
            last = millis();
            on = !on;
            LedService::getInstance()->set_rgb(false, false, on);
        }
        watchdog_update();
        return;
    }

    Psoc* psoc = Psoc::getInstance();
    // ★不再在 core0 调 psoc->update()★：SPI 已由 core1 (core1_run) 固定周期独占;
    // core0 只经 seqlock 读 touch_mask()/snapshot()、经命令队列投递 CSD 指令。
    PsocUpdater* updater = PsocUpdater::getInstance();
    updater->update();

    // ★无状态 PSoC 启动下发★：SPI 链路首次就绪后一次性把 RP2040 持有的 CSD 配置下发 PSoC
    //（SET_MODE + 半自动全部参数 + APPLY）。此时 PSoC 已完整启动，链路稳定。
    static bool csd_downloaded = false;
    if (!csd_downloaded && psoc->link_ok()) {
        CsdConfig::getInstance()->download_to_psoc(psoc);
        csd_downloaded = true;
    }

    UsbComm::getInstance()->update();

    // ★主机租约★：ping 续期。超时未续期即认定上位机丢失,主动停遥测(自洽:绿灯亦据此判连接)。
    if ((millis() - g_last_host_cmd_ms) > HOST_LEASE_TIMEOUT_MS) {
        SensorLink::getInstance()->stop();
    }

    SensorLink::getInstance()->tick();
    BindingService::getInstance()->tick(psoc->link_ok() ? psoc->touch_mask() : 0, psoc->link_ok());
    GameIoService::getInstance()->task();

    // ★flash 落地安全窗口★：命令 handler 只置保存信号，实际 flash 写在此(命令已处理完、ACK 已发)执行。
    // flash 写内部已用 disable_interrupts()+multicore_lockout(暂停 core1)保护 XIP 擦写窗口，
    // 两核都不访问总线→写后 USB 自动恢复，无需重新枚举(照抄 v3.1，不再 reconnect)。
    {
        if (ConfigManager::has_pending_save()) { ConfigManager::save_config_task(); }
        CsdConfig* csd_cfg = CsdConfig::getInstance();
        if (csd_cfg->has_pending_save()) { csd_cfg->save(); }
    }

    // ★vendor OUT 自愈★：每轮检查 config vendor OUT 是否仍处 arm 态，若因 flash 扰动等
    // 掉出则重新武装。放在 flash 落地之后，确保刚写完 flash 即可立即恢复 host→device 接收。
    HAL_USB_Device::getInstance()->vendor_service();

    // ★主循环心跳★：LED 每 150ms 亮灭翻转 = loop 在跑；若卡住则 LED 停在某态(不再闪)。
    // 颜色表状态：红=flash 未就绪 / 蓝=SPI link 断 / 绿=正常。用于目视判断"主循环死"vs"仅 USB 死"。
    const PsocBringupReport& report = updater->report();
    LedService* led = LedService::getInstance();
    // ★主机连接指示★：近 2s 内收到过上位机 host_cmd 帧 = 已连接 → 绿灯常亮;
    // 否则按运行态心跳闪烁(红=flash未就绪/蓝=SPI链路断/绿=正常)。
    const bool host_connected = (millis() - g_last_host_cmd_ms) < 2000u;
    static uint32_t hb_last = 0;
    static bool hb_on = false;
    if (host_connected) {
        led->set_rgb(false, true, false);   // 已连接: 绿灯常亮
        hb_on = false;
        hb_last = millis();
    } else if (millis() - hb_last >= 150) {
        hb_last = millis();
        hb_on = !hb_on;
        const bool red = !report.flash_ok();
        const bool blue = report.flash_ok() && !report.link_ok;
        const bool green = report.flash_ok() && report.link_ok;
        led->set_rgb(red && hb_on, green && hb_on, blue && hb_on);
    }

    watchdog_update();
}
