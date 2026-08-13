
// ======================================================================
// 固件入口
//
// 本文件只负责三件事:
//   1) 启动取证(死前遗言 / 崩溃恢复策略);
//   2) setup() 的初始化顺序 —— 存储注册→load→各服务 init→SWD bring-up→拉起 core1;
//   3) 每轮编排的调用点。
// 两个核的每轮内容各自单列:
//   core0(协议核)  → app/core0_loop.h   : 各阶段 inline 函数 + core0_loop_step()
//   core1(传感器核) → app/core1_loop.h  : 拉起 + 交出 SPI 所有权给 Psoc::core1_run()
//   启动可用性状态机 → app/boot_calibration.h
// ======================================================================

// ★config.h 必须在 <Arduino.h> 之前★: 它用 constexpr 定义 PIN_SPI1_* 等,
// 而 arduino-pico 的变体头把同名标识符宏化, 顺序颠倒会让 constexpr 名字被替换成数字常量。
#include "config.h"

#include <Arduino.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>
#include <pico/bootrom.h>

#include "app/core0_loop.h"
#include "app/core1_loop.h"

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
#include "service/psoc_algo/psoc_algo.h"
#include "service/self_heal/self_heal.h"
#include "service/keyboard/keyboard.h"
#include "service/hid_touch_mapper/hid_touch_mapper.h"
#include "service/bus/bus_core.h"
#include "service/bus/bus_usb_link.h"
#include "service/usb_debug.h"
#include "protocol/psoc/psoc.h"
#include "protocol/hid/hid.h"

#include "service/nv_store/nv_store.h"   // NvStore::enable_lockout()

extern "C" {
#include "hal/global_irq.h"
}

static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 5000;

// core0 的跨轮状态。阶段实现全在 app/core0_loop.h。
static app::Core0State g_core0;

// ★hardfault 兜底 + 取证★
// pico-sdk 的默认 isr_hardfault 落进 while(1), 于是"跑飞"最终也表现为看门狗超时复位 ——
// 与"主循环真的跑太慢"完全同形, 无从区分(实测一次复位报 stage=0 就卡在这里)。
// 覆盖它: 先在 scratch[2] 留标记(跨复位保留), 再立刻软复位。scratch[4..7] 归 SDK 的
// watchdog_reboot 自用, 故标记只能放 0..3(见 usb_debug.h 的 scratch 分配注释)。
extern "C" void isr_hardfault(void) {
    // ★连"哪个核跑飞"一起记★ 两个核共用本处理程序; 只记一个 magic 时无从区分, 而 core0 的
    // 阶段码(scratch[1])又会被 core1 的正常运行覆盖 ⇒ 上一轮据此把 core0 的崩溃误判成 core1。
    // 低字节存 get_core_num(), 启动时取出上报(见 UsbDebugCounters::last_boot_fault_core)。
    watchdog_hw->scratch[CRASH_SCRATCH_FAULT] = crash_fault_word(get_core_num());
    watchdog_reboot(0u, 0u, 0u);
    while (true) { tight_loop_contents(); }
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
    // ★崩溃恢复策略★: 默认"看门狗复位→正常重启"(不进 BOOTSEL), 避免改设置偶发死机把设备卡进烧录。
    // 仅当【运行时指令 DEBUG_CRASH_BOOTSEL 已武装】(scratch[6]=magic, 跨复位存活)时, 运行中崩溃才进
    // BOOTSEL——供自持 debug: 上位机连上发指令武装后, 一旦崩溃自动进烧录便于 dev.ps1 自动重烧恢复。
    // scratch[6] 由指令置/清, 掉电清零; scratch[7]=运行中标记, 每次启动清零。
    const uint32_t boot_flag = watchdog_hw->scratch[7];
    const bool crash_bootsel_armed = (watchdog_hw->scratch[6] == DEBUG_BOOTSEL_MAGIC);
    // ★死前遗言★: scratch[5] 记录"复位前 core0 正处于哪个阶段"(见 CrashStage)。scratch 跨复位保留,
    // 于是崩溃重启后能精确知道卡在哪一步, 不必再靠时间差反推 —— 掉线定位已经在退避/环容量/串行队列/
    // 分片写/lockout 门控上猜错过多次, 这里改用实测。
    // ★只有"运行中标记有效"时 scratch[5] 才可信★: 上电/BOOTSEL 复位不保证清 scratch, 里面是随机值。
    // 上一轮就被一个随机的 1 误导成"死在 CFG_FLASH"。boot_flag 无效时一律报 0xFF=不可信。
    // 判据只认 scratch[0..3](用户区)。scratch[7] 的旧 boot_flag 会被 SDK 的 watchdog_reboot 覆盖,
    // 不能用来判断"上次是否运行中崩溃"。
    const bool ran_before = (watchdog_hw->scratch[CRASH_SCRATCH_RUN] == CRASH_RUN_MAGIC);
    g_usb_dbg.last_crash_stage = ran_before
        ? (uint8_t)(watchdog_hw->scratch[CRASH_SCRATCH_STAGE] & 0xFFu)
        : 0xFFu;
    g_usb_dbg.last_boot_was_wd = ran_before ? 1u : 0u;
    // 死前遗言增强: 上次是否 hardfault + 上次运行期整轮耗时峰值 + 硬件复位原因。
    // 三者合起来才能定性: fault=1 → 跑飞; fault=0 且 loop_max 接近 5s → 主循环真被拖死;
    // fault=0 且 loop_max 很小 → 既没跑飞也没拖慢, 那就是外部原因(掉电/XRES)。
    const uint32_t fault_word = watchdog_hw->scratch[CRASH_SCRATCH_FAULT];
    // 只比高 24 位: 低字节是出错核编号(见 crash_fault_word)。旧 magic 0x46554C54 的高 24 位
    // 同样是 0x46554C, 故刷入新固件前留下的标记仍能被正确识别为 fault。
    g_usb_dbg.last_boot_was_fault =
        ((fault_word & CRASH_FAULT_MASK) == CRASH_FAULT_TAG) ? 1u : 0u;
    g_usb_dbg.last_boot_fault_core =
        g_usb_dbg.last_boot_was_fault ? (uint8_t)(fault_word & 0xFFu) : 0xFFu;
    const uint32_t pm_word = watchdog_hw->scratch[CRASH_SCRATCH_PM];
    g_usb_dbg.last_boot_stage_at_ms = ran_before ? (uint16_t)(pm_word >> 16) : 0u;
    g_usb_dbg.last_boot_peak_ms = ran_before ? (uint16_t)(pm_word & 0xFFFFu) : 0u;
    g_usb_dbg.last_boot_loop_max_us = (uint32_t)g_usb_dbg.last_boot_peak_ms * 1000u;
    g_usb_dbg.last_reset_reason = (uint8_t)(watchdog_hw->reason & 0x3u);
    watchdog_hw->scratch[CRASH_SCRATCH_FAULT] = 0u;
    watchdog_hw->scratch[CRASH_SCRATCH_PM] = 0u;
    watchdog_hw->scratch[CRASH_SCRATCH_RUN] = 0u;
    watchdog_hw->scratch[CRASH_SCRATCH_STAGE] = 0u;
    watchdog_hw->scratch[7] = 0u;
    // ★仅当"上次确实运行中崩溃"且"已武装"才进 BOOTSEL★: ran_before 读 scratch[0]=CRASH_RUN_MAGIC(主循环每轮刷),
    // boot_flag 读 scratch[7]=WD_RUNNING_MAGIC(只在 setup 末写一次)。主动重启会清 scratch[7] 但保留 scratch[0],
    // 所以判据必须用 ran_before, 否则任何主动重启+武装窗口重合都会误进 BOOTSEL。
    if (ran_before && crash_bootsel_armed) {
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

    // ★存储初始化顺序是硬要求★
    // 1) 各持有者先把自己的镜像缓冲注册给 NvStore(此时不读 flash);
    // 2) NvStore::load() 一次性把 flash 里所有区摊进这些镜像 —— 这是**整个固件唯一一次读 flash**;
    // 3) 各服务从镜像解出运行值。
    // 顺序错了(比如先 init 再 load)就会拿到空镜像, 正是先前"保存看着成功、重启全丢"的成因之一。
    app_config_register_schema();
    CsdConfig::getInstance()->register_storage();
    PsocAlgo::getInstance()->register_storage();
    NvStore::getInstance()->load();
    // ★单份存储: 某区无效必须上报★
    // 无效区会被持有者按默认值重建, 用户看到的只是"设置又丢了", 不上报就无从分辨是"这一区坏了"
    // 还是"固件有 bug"。四个区都注册在 load() 之前, 故此刻的 valid_mask 是完整判据(全好 = 0x0F)。
    {
        const uint8_t nv_valid = NvStore::getInstance()->valid_mask();
        if (nv_valid != 0x0Fu) SelfHeal::getInstance()->note(SH_NV_REGION_INVALID, nv_valid);
    }
    ConfigManager::initialize();

    Psoc* psoc = Psoc::getInstance();
    psoc->init();

    // USB 与最小 HostCmd 服务先完成注册，使 SWD bring-up 期间仍能处理 HELLO/DEVICE_INFO。
    HAL_USB_Device::getInstance()->init();
    CsdConfig::getInstance()->init();
    PsocAlgo::getInstance()->init();
    UsbComm::getInstance()->init();
    Mai2Bus::getInstance()->init();
    BusUsbLink::getInstance()->init();
    SensorLink::getInstance()->init();
    BindingService::getInstance()->init();

    // ★HID 键盘★：两种 USB 模式都枚举 HID(serial 也带键盘 report)。
#if MAI2_ENABLE_SERIAL_HID
    HID::getInstance()->init(HAL_USB_Device::getInstance());
    KeyboardService::getInstance()->init();
    HidTouchMapper::getInstance()->init(HID::getInstance());
#endif

    const UsbWorkMode work_mode = (ConfigManager::get_uint8("mode.work") ==
        static_cast<uint8_t>(UsbWorkMode::WORK_HID))
        ? UsbWorkMode::WORK_HID : UsbWorkMode::WORK_SERIAL;
    if (work_mode == UsbWorkMode::WORK_SERIAL) {
        GameIoService::getInstance()->init(work_mode);
    }
    GameIoService::getInstance()->register_host_cmds();

    // 启动期 SWD bring-up；此时 USB/HostCmd 已可服务 HELLO 与 DEVICE_INFO。
    PsocUpdater::getInstance()->run(psoc);

    // 取得首份真实快照，使 DEVICE_INFO 的运行态字段立即可用。
    psoc->update();
    PsocUpdater::getInstance()->update();

    // ★启动 core1★：必须在任何 flash 写(loop 的落盘窗口)之前完成，
    // 使 multicore_lockout_start_blocking() 有已注册的 victim 可暂停，而非永久死锁。
    app::core1_launch();
    // core1 已启动并会在 core1_entry 首行注册 lockout victim ⇒ 此后 NvStore 的 flash 写才允许
    // lockout。在此之前(ConfigManager::initialize 首次上电写默认配置)必须不 lockout, 否则永久死锁。
    NvStore::enable_lockout();

    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    // 进入运行态标记：此后任何看门狗超时复位，启动时即判为"运行中死锁/异常"→进 BOOTSEL 自恢复。
    watchdog_hw->scratch[7] = WD_RUNNING_MAGIC;
}

// 每轮内容见 app/core0_loop.h::core0_loop_step —— 本文件不再夹带任何阶段实现。
void loop() {
    app::core0_loop_step(g_core0);
}
