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
#include "service/psoc_algo/psoc_algo.h"
#include "service/self_heal/self_heal.h"
#include "service/tx_scheduler/tx_scheduler.h"
#include "service/keyboard/keyboard.h"
#include "service/tx_scheduler/tx_scheduler.h"
#include "service/usb_debug.h"
#include "protocol/psoc/psoc.h"
#include "protocol/hid/hid.h"

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
    // ★崩溃恢复策略★: 默认"看门狗复位→正常重启"(不进 BOOTSEL), 避免改设置偶发死机把设备卡进烧录。
    // 仅当【运行时指令 DEBUG_CRASH_BOOTSEL 已武装】(scratch[6]=magic, 跨复位存活)时, 运行中崩溃才进
    // BOOTSEL——供自持 debug: 上位机连上发指令武装后, 一旦崩溃自动进烧录便于 dev.ps1 自动重烧恢复。
    // scratch[6] 由指令置/清, 掉电清零; scratch[7]=运行中标记, 每次启动清零。
    const uint32_t boot_flag = watchdog_hw->scratch[7];
    const bool crash_bootsel_armed = (watchdog_hw->scratch[6] == DEBUG_BOOTSEL_MAGIC);
    watchdog_hw->scratch[7] = 0u;
    if ((boot_flag == WD_RUNNING_MAGIC) && crash_bootsel_armed) {
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
    // JIT 触控算法 store：从 flash 载入自定义算法(若有)，否则用内嵌默认(v3.1 HDR)；同样延到 loop() 下发。
    PsocAlgo::getInstance()->init();

    HAL_USB_Device::getInstance()->init();
    UsbComm::getInstance()->init();
    SensorLink::getInstance()->init();
    BindingService::getInstance()->init();

    // ★HID 键盘★：两种 USB 模式都枚举 HID(serial 也带键盘 report), 使 触控→键盘映射
    // 与物理键盘 GPIO1-12 在游戏(serial)模式下也能输出 HID 键。诊断开关可整体禁用做二分。
#if MAI2_ENABLE_SERIAL_HID
    HID::getInstance()->init(HAL_USB_Device::getInstance());
    KeyboardService::getInstance()->init();
#endif

    const UsbWorkMode work_mode = (ConfigManager::get_uint8("mode.work") ==
        static_cast<uint8_t>(UsbWorkMode::WORK_HID))
        ? UsbWorkMode::WORK_HID : UsbWorkMode::WORK_SERIAL;
    if (work_mode == UsbWorkMode::WORK_SERIAL) {
        GameIoService::getInstance()->init(work_mode);
    }
    // mai2 串口状态命令与工作模式无关(HID 模式如实回 STOPPED), 故无条件注册。
    GameIoService::getInstance()->register_host_cmds();

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

    // provisioning 状态(算法 + CSD 是否已下发到当前这颗运行中的 PSoC)。救砖与失效兜底都要清它，
    // 故在此提前声明(其语义与去抖逻辑见下方 provisioning 段)。
    static bool provisioned = false;
    static uint32_t link_down_since_ms = 0;
    // 主机主动重启后的事件抑制窗(见下方 g_psoc_reboot_request 分支): 窗内不把链路丢失型复位
    // 上报为"固件自行救自己", 否则上位机会看到自己发起的重启被描述成设备异常。
    static uint32_t host_reboot_quiet_until_ms = 0u;

    // ★PSoC 救砖(PSOC_RESCUE)★：命令已回 ACK，重活在此(主循环安全窗口)执行——经 SWD 强制全片
    // 擦写内嵌镜像+校验+复位运行。擦写数秒期间由 SwdProgrammer 的保活钩子喂狗/泵 USB/推送进度。
    // 完成后清 provisioned，交由下方既有 provisioning 重新下发算法 + CSD(即"重新应用")。
    if (updater->rescue_step(psoc)) {
        provisioned = false;
        psoc->clear_reset_request();   // 重刷期间 core1 必然判过链路丢失，清掉避免刚恢复就被 XRES
        SelfHeal::getInstance()->note(SH_PSOC_RESCUED, 0u);
    }

    // ★失效兜底★：core1 检测到 PSoC 崩溃/掉线(reason=1)或主循环卡死(reason=2, 疑似坏算法)后，
    // core0 在此脉冲 XRES 硬复位 PSoC。算法/CSD 参数在 PSoC RAM，复位即丢失，故 provisioned 归零
    // 令链路恢复后重新下发。reason=2(卡死)且当前为自定义算法 → 判定该算法致命，回退内嵌默认防复位环。
    // 救砖进行中不介入: 此时 PSoC 被 halt 在 SWD 会话里, XRES 会打断擦写。
    const uint8_t reset_reason = updater->rescue_active() ? 0u : psoc->needs_reset();
    if (reset_reason != 0u) {
        if (reset_reason == 2u && !PsocAlgo::getInstance()->is_default()) {
            PsocAlgo::getInstance()->reset_default();
            // 用户算法被判定致命并回退 —— 这是最容易让人误判"我的算法还在跑"的一步, 必须上报。
            SelfHeal::getInstance()->note(SH_ALGO_FALLBACK, 0u);
        }
        psoc->reset_run();
        psoc->clear_reset_request();
        // 卡死型(reason=2)永远上报; 链路丢失型(reason=1)在主机主动重启的抑制窗内跳过, 免得把
        // "上位机自己发起的重启"报成"固件自行复位"。
        if (reset_reason == 2u) {
            SelfHeal::getInstance()->note(SH_PSOC_RESET_HANG, reset_reason);
        } else if (millis() >= host_reboot_quiet_until_ms) {
            SelfHeal::getInstance()->note(SH_PSOC_RESET_LINK, reset_reason);
        }
    }

    // ★主机请求重启 PSoC★(REBOOT_PSOC=0x06): 脉冲 XRES 复位 PSoC 进运行态, 使"需重启生效"的
    // 改动(如全局 CSD 重初始化)真正生效。复位后链路短暂丢失→下方 provisioned 逻辑自动重新下发算法/CSD。
    // ★主机主动重启不算"固件自行救自己"★: 它必然带来一次链路丢失, core1 随后会把它当成 PSoC 掉线
    // 再报一次复位事件, 上位机就会看到"固件已自行 XRES 复位"这种误导文案。故在此开一个短抑制窗,
    // 窗内的链路丢失型复位事件不上报(重启本身由上位机发起, 它自己知道)。
    if (g_psoc_reboot_request) {
        g_psoc_reboot_request = 0u;
        psoc->reset_run();
        host_reboot_quiet_until_ms = millis() + 3000u;
    }

    // ★无状态 PSoC 启动/复位后下发★：SPI 链路(重新)就绪后一次性把 RP2040 持有的算法与 CSD 配置
    // 下发 PSoC——先下发 JIT 触控算法(ALGO_*)到其 1KB 可执行槽，再下发 CSD 参数(SET_MODE+参数+APPLY)。
    // provisioned 在失效复位后被清零，链路恢复即自动重新下发(算法在 PSoC RAM，复位丢失必须重推)。
    // ★链路抖动去抖★：download_to_psoc 内含 Init+Enable 等重初始化, 期间 PSoC SPI 会短暂无响应,
    // 使 link_ok 瞬时 false。若一有 false 就清 provisioned 会触发"重下发→重初始化→又瞬断"的
    // provision 死循环(表现为状态灯白/绿反复闪 + 周期性重扫扰动 Cp/时序)。仅当链路【持续】断开
    // 超过阈值(真正 PSoC 复位)才判为未就绪重新下发, 忽略重初始化期间的瞬时抖动。
    const bool link_now = psoc->link_ok();
    if (link_now) {
        link_down_since_ms = 0;
    } else if (link_down_since_ms == 0u) {
        link_down_since_ms = millis();
    }
    if (!provisioned && link_now) {
        PsocAlgo::getInstance()->download_to_psoc(psoc);
        CsdConfig* csd = CsdConfig::getInstance();
        csd->download_to_psoc(psoc);
        provisioned = true;
        // ★恢复默认→良好半自动基线★: RESET_DEFAULTS 后 store 已清空, PSoC 此刻带出厂强制好全局
        // (增益4/目标85%) 且 Enable 已自动校准好 IDAC(raw≈目标, 未 railed)。回读这些校准好的值作默认,
        // 切 SEMI 快速模式并立即重下发生效 → 得到"正常半自动基线"作为默认; RP2040 持有, PSoC 无状态。
        // ★可信度校验(修"恢复默认救不回来")★: 若 PSoC 此刻本身就异常(raw 满量程 railed / 完全不抖动
        // = 扫描停滞), 无条件回读就会把坏状态固化成新默认, 越点越回不去(实机已复现)。故先抽检采样,
        // 不可信则【不固化】: 保持 store 空(AUTO + invalid)让 PSoC 跑自己的出厂默认链, 并置标志
        // 经 DEVICE_INFO 上报, 由上位机提示改用"PSoC 救砖"。
        if (csd->has_pending_recapture()) {
            if (csd->mode() != CSD_MODE_SEMI) {
                // AUTO 的实时参数由 CapSense 自动计算；即使采样可信也绝不能回读固化，
                // 否则会覆盖用户原先保存的手动阈值/snsClk，切回 SEMI 时无法恢复。
            } else if (csd->sampling_trustworthy(psoc) && csd->capture_from_psoc(psoc)) {
                csd->download_to_psoc(psoc);       // set_mode(SEMI)+参数+APPLY: 手动参数即时生效
                csd->request_save();               // 持久化, 成为下次开机默认
            } else {
                csd->clear();                      // 空 store(含 request_save): 真正回到出厂默认链
                csd->download_to_psoc(psoc);       // set_mode(AUTO): PSoC 走标准完整处理 + 自动校准
                csd->note_baseline_untrusted(true);
                // 整个 CSD store 被清空并落盘 —— 用户所有逐通道调参就此消失, 不上报等于骗人。
                SelfHeal::getInstance()->note(SH_STORE_CLEARED, 0u);
            }
            csd->clear_recapture();
        }
        // 救砖的"重新应用"以此为完成点: 算法 + CSD 已重新下发到刚刷好的 PSoC。
        updater->rescue_note_reapplied();
        // 该标志只记录当前运行态的不可信采样；PSoC 重启/救砖重新应用后必须再次实测，
        // 已恢复才清除，避免无条件隐藏仍存在的硬件或扫描异常。
        if (csd->baseline_untrusted() && csd->sampling_trustworthy(psoc)) {
            csd->note_baseline_untrusted(false);
            SelfHeal::getInstance()->note(SH_BASELINE_TRUST_RESTORED, 0u);
        }
        // 重新下发完成 → 上位机据此重新回读设备真值(PSoC 的 CSD 配置活在 RAM, 复位后必然换了一套)。
        SelfHeal::getInstance()->note(SH_REPROVISIONED, csd->mode());
        // PSoC 启动时若强制改写过生成配置(IDAC 增益档抬到下限 / 非法校准目标% 回退 85), 一并上报,
        // 使"设备实际值 != 下发值"永远有据可查, 不再是静默不同步。
        {
            // PSoC 侧 GPARAM_BOOT_OVERRIDE(0x09): 只读位掩码, 见 psoc_firmware main.c。
            constexpr uint8_t GPARAM_ID_BOOT_OVERRIDE = 0x09u;
            uint32_t override_bits = 0u;
            if (psoc->get_global(GPARAM_ID_BOOT_OVERRIDE, &override_bits) && override_bits != 0u) {
                SelfHeal::getInstance()->note(SH_PSOC_BOOT_OVERRIDE, override_bits);
            }
        }
    }
    // 持续断开 >400ms 视为真复位 → 清 provisioned, 链路恢复后重下发(算法/CSD 在 PSoC RAM, 复位丢失)。
    if (!link_now && link_down_since_ms != 0u && (millis() - link_down_since_ms) > 400u) {
        provisioned = false;
    }

    // ★自持恢复事件必达兜底★: note() 只在事件发生那一刻拉起推送任务, 若当时上位机没连(或租约过期
    // 自取消), 队列里的事件就再也没人推 → 用户永远看不到"设备被固件改过"。故只要队列非空且任务不在,
    // 就重新拉起(有上位机时 10Hz 排空, 无上位机时靠租约自灭, 不常驻)。
    if (!SelfHeal::getInstance()->empty() && !TxScheduler::getInstance()->active(TX_TASK_SELFHEAL)) {
        SelfHeal::getInstance()->note_rearm();
    }

    UsbComm::getInstance()->update();

    // ★主机租约★：ping 续期。超时未续期即认定上位机丢失,暂停遥测(自洽:绿灯亦据此判连接)。
    // ★只挂起、不永久停★: core0 可能只是被长设备操作(JIT 算法下发/校准/flash 落地)按在
    // UsbComm::update() 之外几秒 —— 上位机其实一直在, 但租约照样过期。旧实现在这里调 stop(),
    // 把 _streaming 清掉且没有任何恢复路径 → 上位机不会再发 TELEM_START → 全通道 raw/baseline/diff
    // 永久冻结, 只能复位设备。改为挂起, 主机命令一到即用原参数自动续推。
    if ((millis() - g_last_host_cmd_ms) > HOST_LEASE_TIMEOUT_MS) {
        SensorLink::getInstance()->suspend();
    } else {
        SensorLink::getInstance()->resume();
    }

    // ★算法下发结果上报★: 下发已异步化(core1 执行), 失败只有 core1 知道。在此取走标志上报,
    // 使"算法没真正装上"永远有据可查, 而不是让用户以为自定义算法正在跑。
    if (psoc->algo_download_take_failure()) {
        SelfHeal::getInstance()->note(SH_ALGO_FALLBACK, 1u);
    }

    // ★大吞吐统一走定时任务队列★: 遥测等周期发送由 TxScheduler 按各自频率+租约驱动(续期制),
    // 帧经非阻塞 config_write 入 vendor TX FIFO, 由 HAL_USB task() 泵出。不再在此直接 tick 遥测。
    TxScheduler::getInstance()->tick();
    BindingService::getInstance()->tick(psoc->link_ok() ? psoc->touch_mask() : 0, psoc->link_ok());
    GameIoService::getInstance()->task();
    // 物理键盘(GPIO1-12) + 触控→键盘映射 → HID(内部 task HID 发报文)。
#if MAI2_ENABLE_SERIAL_HID
    KeyboardService::getInstance()->task();
#endif

    // ★flash 落地安全窗口★：命令 handler 只置保存信号，实际 flash 写在此(命令已处理完、ACK 已发)执行。
    // flash 写内部已用 disable_interrupts()+multicore_lockout(暂停 core1)保护 XIP 擦写窗口，
    // 两核都不访问总线→写后 USB 自动恢复，无需重新枚举(照抄 v3.1，不再 reconnect)。
    // ★每轮最多落一份★: 单份 LittleFS 写要禁中断 + 停 XIP + lockout core1 数十~上百 ms, 期间
    // TinyUSB 的 USB 中断完全得不到服务。一次"保存到设备"会同时置起 config/csd/algo 三个信号,
    // 三份背靠背写 = 数百 ms 连续 USB 黑洞, 主机侧待处理的 OUT 传输会被 Windows 直接 abort
    // (实测 kind=ConnectionAborted → 拆端点 → 判断开)。分轮落地, 每份之间必有一次完整 USB 服务轮。
    {
        if (ConfigManager::has_pending_save()) {
            ConfigManager::save_config_task();
        } else if (CsdConfig::getInstance()->has_pending_save()) {
            CsdConfig::getInstance()->save();
        } else if (PsocAlgo::getInstance()->has_pending_save()) {
            PsocAlgo::getInstance()->save();
        }
    }

    // ★vendor OUT 自愈★：每轮检查 config vendor OUT 是否仍处 arm 态，若因 flash 扰动等
    // 掉出则重新武装。放在 flash 落地之后，确保刚写完 flash 即可立即恢复 host→device 接收。
    HAL_USB_Device::getInstance()->vendor_service();

    // ★主循环心跳★：LED 每 150ms 亮灭翻转 = loop 在跑；若卡住则 LED 停在某态(不再闪)。
    // 状态优先级保持既有语义：主机连接常亮优先，其余依次为 flash 未就绪、SPI 链路断、健康。
    const PsocBringupReport& report = updater->report();
    LedService* led = LedService::getInstance();
    const uint8_t status_brightness = std::min<uint8_t>(
        ConfigManager::get_uint8("led.status_brightness"), 255u);
    const uint8_t connected_color = std::min<uint8_t>(
        ConfigManager::get_uint8("led.color_connected"), 7u);
    const uint8_t flash_error_color = std::min<uint8_t>(
        ConfigManager::get_uint8("led.color_flash_error"), 7u);
    const uint8_t link_error_color = std::min<uint8_t>(
        ConfigManager::get_uint8("led.color_link_error"), 7u);
    const uint8_t healthy_color = std::min<uint8_t>(
        ConfigManager::get_uint8("led.color_healthy"), 7u);
    const bool led_enabled = ConfigManager::get_bool("led.enable");
    // 近 2s 内收到过 host_cmd 帧 = 已连接；沿用现有租约语义。
    const bool host_connected = (millis() - g_last_host_cmd_ms) < 2000u;
    static uint32_t hb_last = 0;
    static bool hb_on = false;
    if (!led_enabled) {
        led->set_color(0u, 0u);
        hb_on = false;
        hb_last = millis();
    } else if (!provisioned) {
        // ★PSoC 启动/重启加载指示★：链路(重新)建立、CSD/算法尚未下发完成前状态灯常亮(白),
        // provisioned 置真(进入正常工作)即转入下方常规状态逻辑, 便于肉眼清晰分辨 PSoC 重启窗口。
        led->set_color(7u /* 白 */, status_brightness);
        hb_on = false;
        hb_last = millis();
    } else if (host_connected) {
        led->set_color(connected_color, status_brightness);
        hb_on = false;
        hb_last = millis();
    } else if (millis() - hb_last >= 150) {
        hb_last = millis();
        hb_on = !hb_on;
        const uint8_t status_color = !report.flash_ok()
            ? flash_error_color
            : (!report.link_ok ? link_error_color : healthy_color);
        led->set_color(status_color, hb_on ? status_brightness : 0u);
    }

    watchdog_update();
}
