#pragma once

// ======================================================================
// core0 主循环（协议核）
//
// 边界: 每个关注点一个 inline 阶段函数, core0_loop_step() 只做顺序编排。
// main.cpp 因此只剩"启动"与"每轮按什么顺序跑", 不再夹带任何阶段实现。
// 跨阶段状态全部收进 Core0State 一个结构体 —— 原先散落在 loop() 里的
// static 局部变量既看不出生命周期, 也无法被复位路径统一清理。
//
// 所有 loop_seg_begin / loop_prof_mark / crash_stage_set 调用的位置语义保持不变:
// 它们是与 5s 看门狗竞争的实测剖面与崩溃取证依据, 移位就失去可比性。
// ======================================================================

// ★config.h 必须在 <Arduino.h> 之前★: 它的 PIN_SPI1_* constexpr 与 arduino-pico
// 变体头的同名宏冲突, 顺序颠倒会把 constexpr 名字替换成数字常量。
#include "../config.h"

#include <stdint.h>
#include <algorithm>
#include <Arduino.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>

#include "boot_calibration.h"

#include "../hal/usb/hal_usb.h"
#include "../service/usb_comm/usb_comm.h"
#include "../service/config_manager/config_manager.h"
#include "../service/sensor_link/sensor_link.h"
#include "../service/binding_service/binding_service.h"
#include "../service/game_io/game_io.h"
#include "../service/led_service/led_service.h"
#include "../service/psoc_updater/psoc_updater.h"
#include "../service/csd_config/csd_config.h"
#include "../service/psoc_algo/psoc_algo.h"
#include "../service/self_heal/self_heal.h"
#include "../service/tx_scheduler/tx_scheduler.h"
#include "../service/keyboard/keyboard.h"
#include "../service/hid_touch_mapper/hid_touch_mapper.h"
#include "../service/bus/bus_core.h"
#include "../service/nv_store/nv_store.h"
#include "../service/persistence_txn/persistence_txn.h"
#include "../service/usb_debug.h"
#include "../protocol/psoc/psoc.h"
#include "../protocol/hid/hid.h"

// PSoC 重启请求(定义于 hal_usb.cpp): 置 1 → 主循环脉冲 XRES 重启 PSoC。
extern volatile uint8_t g_psoc_reboot_request;

namespace app {

// ★主机租约超时★：上位机每 ~1s PING 续期(刷新 g_last_host_cmd_ms)。超过本时长未收到任何
// 主机指令 = 上位机已丢失(关闭/崩溃) → 固件自停遥测流, 避免遗留流淹没下次连接的 DEVICE_INFO。
constexpr uint32_t HOST_LEASE_TIMEOUT_MS = 3000;
// ★命令信封关闭后才允许擦 flash★: 200ms 足以吸收同一批 host 命令的正常帧间抖动,
// 同时避免命令洪流期间反复进入 flash 黑洞; 脏标记保持粘性, 门未开时只延后本轮。
constexpr uint32_t NV_COMMIT_QUIET_MS = 200u;

// core0 的全部跨阶段状态。原先是 loop() 内的 static 局部变量, 集中后
// "PSoC 换了一代次要清哪些东西"变成一次 clear_provisioning() 而不是散落五处。
struct Core0State {
    // provisioning = 算法 + CSD 是否已下发到当前这颗运行中的 PSoC。
    bool provisioned = false;
    bool provisioning_started = false;
    bool csd_provisioning_started = false;
    uint32_t link_down_since_ms = 0u;
    // 主机主动重启后的事件抑制窗: 窗内不把链路丢失型复位上报为"固件自行救自己",
    // 否则上位机会看到自己发起的重启被描述成设备异常。
    uint32_t host_reboot_quiet_until_ms = 0u;
    BootCalibrationState boot_calibration {};
    // USB 枚举沿检测与状态灯心跳(原 loop() 内的 static)。
    bool usb_was_up = false;
    uint32_t hb_last_ms = 0u;
    bool hb_on = false;

    // 确知 PSoC 代次失效时的统一清理: 算法/CSD 都活在 PSoC RAM, 复位即丢, 必须重推。
    void clear_provisioning() {
        provisioned = false;
        provisioning_started = false;
        csd_provisioning_started = false;
        PsocAlgo::getInstance()->abort_provisioning();
        CsdConfig::getInstance()->abort_provisioning();
        boot_calibration.clear();
    }
};

inline void core0_usb_task() {
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_USB_TASK);
    HAL_USB_Device::getInstance()->task();
    Mai2Bus::getInstance()->task();
    loop_prof_mark(LOOP_SEG_USB_TASK, seg_t);
}

// HostCmd 在 provisioning 之前取得本轮优先权。否则 provisioning 每轮先排一条 PSoC 命令，
// 后面的 PARAM_SET/只读探针永远只看到 core1 非空并持续 DEVICE_BUSY。
inline void core0_host_cmd() {
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_HOST_CMD);
    crash_stage_set(CRASH_STAGE_USB_UPDATE);
    UsbComm::getInstance()->update();
    crash_stage_set(CRASH_STAGE_NONE);
    loop_prof_mark(LOOP_SEG_HOST_CMD, seg_t);
}

// ★控制通道 BOOTSEL★：EP0 请求 0x52 置位后，泵几轮 task 让 ACK 发出，再进烧录。
// 因 EP0 在 bulk vendor 死后仍存活，此路径免去 vendor 卡死时的物理 BOOTSEL。
inline void core0_bootsel_if_requested() {
    if (!g_bootsel_request) return;
    for (uint8_t i = 0; i < 32; i++) { HAL_USB_Device::getInstance()->task(); sleep_ms(2); }
    watchdog_hw->scratch[7] = 0u;
    reset_usb_boot(0, 0);
}

// 隔离测试模式: SWD 已交给外部 DAP, 本轮只维持 USB/遥测骨架与心跳灯。
// 返回 true = 本轮到此为止(调用方直接 return)。
inline bool core0_swd_released_step() {
    if (!SWD_RELEASE_TO_EXTERNAL) return false;
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
    return true;
}

// PSoC 代次生命周期: 救砖 / 失效兜底 XRES / 主机请求重启 / 算法与 CSD 下发 / 启动可用性观察。
// ★不在 core0 调 psoc->update()★：SPI 已由 core1 (core1_run) 固定周期独占;
// core0 只经 seqlock 读 touch_mask()/snapshot()、经命令队列投递 CSD 指令。
inline void core0_psoc_lifecycle(Core0State& st) {
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_PSOC);
    Psoc* psoc = Psoc::getInstance();
    PsocUpdater* updater = PsocUpdater::getInstance();
    updater->update();

    // ★PSoC 救砖(PSOC_RESCUE)★：命令已回 ACK，重活在此(主循环安全窗口)执行——经 SWD 强制全片
    // 擦写内嵌镜像+校验+复位运行。擦写数秒期间由 SwdProgrammer 的保活钩子喂狗/泵 USB/推送进度。
    // 完成后清 provisioned，交由下方既有 provisioning 重新下发算法 + CSD(即"重新应用")。
    if (updater->rescue_step(psoc)) {
        st.clear_provisioning();
        psoc->clear_reset_request();   // 重刷期间 core1 必然判过链路丢失，清掉避免刚恢复就被 XRES
        SelfHeal::getInstance()->note(SH_PSOC_RESCUED, 0u);
    }

    // 失效兜底：core1 检测到 PSoC 崩溃/掉线或运行期主循环卡死后，core0 在此脉冲 XRES 硬复位。
    // 算法/CSD 参数在 PSoC RAM，复位即丢失，故 provisioned 归零令链路恢复后重新下发。
    // 救砖进行中不介入：此时 PSoC 被 halt 在 SWD 会话里，XRES 会打断擦写。
    const uint8_t reset_reason = updater->rescue_active() ? 0u : psoc->needs_reset();
    if (reset_reason != 0u) {
        if (reset_reason == 2u && !PsocAlgo::getInstance()->is_default()) {
            PsocAlgo::getInstance()->reset_default();
            // 用户算法被判定致命并回退 —— 这是最容易让人误判"我的算法还在跑"的一步, 必须上报。
            SelfHeal::getInstance()->note(SH_ALGO_FALLBACK, 0u);
        }
        psoc->reset_run();
        psoc->clear_reset_request();
        // ★确知复位 ⇒ 显式清 provisioned★, 不再依赖"链路断开>400ms"去反推。
        // 反推那条路现在有重操作宽限窗守着(APPLY 12s 期间链路必然抖动, 不能判成掉线),
        // 若仍只靠它, 我们自己发起的复位就可能因宽限而不触发重新下发 → 算法/CSD 永久丢失。
        st.clear_provisioning();
        // 仅运行期卡死属于异常自愈事件；链路故障按主机重启抑制窗决定是否上报。
        if (reset_reason == 2u) {
            SelfHeal::getInstance()->note(SH_PSOC_RESET_HANG, reset_reason);
        } else if (reset_reason == 1u && millis() >= st.host_reboot_quiet_until_ms) {
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
        // 同上: 主机请求的重启是"确知复位", 显式清 provisioned 保证 RESET_DEFAULTS 后
        // 清空的 store 一定会被重新下发(否则 PSoC 仍留着复位前推下去的旧参数, 恢复默认等于没生效)。
        st.clear_provisioning();
        st.host_reboot_quiet_until_ms = millis() + 3000u;
    }

    // ★无状态 PSoC 启动/复位后下发★：SPI 链路(重新)就绪后一次性把 RP2040 持有的算法与 CSD 配置
    // 下发 PSoC——先下发 JIT 触控算法(ALGO_*)到其 1KB 可执行槽，再下发 CSD 参数(SET_MODE+参数+APPLY)。
    // ★链路抖动去抖★：download_to_psoc 内含 Init+Enable 等重初始化, 期间 PSoC SPI 会短暂无响应,
    // 使 link_ok 瞬时 false。若一有 false 就清 provisioned 会触发"重下发→重初始化→又瞬断"的
    // provision 死循环(表现为状态灯白/绿反复闪 + 周期性重扫扰动 Cp/时序)。仅当链路【持续】断开
    // 超过阈值(真正 PSoC 复位)才判为未就绪重新下发, 忽略重初始化期间的瞬时抖动。
    const bool link_now = psoc->link_ok();
    if (link_now) {
        st.link_down_since_ms = 0;
    } else if (st.link_down_since_ms == 0u) {
        st.link_down_since_ms = millis();
    }
    CsdConfig* csd = CsdConfig::getInstance();
    PsocAlgo* algo = PsocAlgo::getInstance();
    if (!st.provisioned && link_now) {
        // Start each provisioning generation once, then advance only bounded work.
        // Algorithm code, ROM/cfg, mode, global values, enabled flags and CSD
        // parameters are serialized as individual queue steps instead of filling
        // the PSoC ring during a single USB service window.
        if (!st.provisioning_started) {
            st.provisioning_started = algo->download_to_psoc(psoc);
        }
        algo->tick(psoc);
        if (st.provisioning_started && !algo->provisioning_active() && !st.csd_provisioning_started) {
            st.csd_provisioning_started = csd->download_to_psoc(psoc);
        }
        csd->tick_provisioning(psoc);
        if (st.provisioning_started && st.csd_provisioning_started && csd->provisioning_complete()) {
            if (csd->has_pending_recapture()) {
                const int8_t recapture = csd->tick_recapture(psoc);
                if (recapture == 0) {
                    // 每轮最多读取一个 PSoC 值；完整捕获结束前不进入本代 provisioning 终态。
                } else if (recapture > 0) {
                    csd->abort_provisioning();
                    st.csd_provisioning_started = csd->download_to_psoc(psoc);
                    csd->request_save();
                } else {
                    csd->clear_recapture();
                    csd->note_baseline_untrusted(true);
                    SelfHeal::getInstance()->note(SH_STORE_CLEARED, 0u);
                    st.provisioned = true;
                    updater->rescue_note_reapplied();
                    SelfHeal::getInstance()->note(SH_REPROVISIONED, csd->mode());
                }
            } else {
                st.provisioned = true;
                updater->rescue_note_reapplied();
                SelfHeal::getInstance()->note(SH_REPROVISIONED, csd->mode());
            }
        }
    }

    // provisioning 成功后启动一次；DONE 后本代次不再进入，只有明确的新 PSoC 代次会 clear()。
    // 启动基础架构只做被动健康观察，不调用扫描专属或运行时参数应用 API。
    // 五个判据摊到一个诊断字节(见 usb_debug.h)。★无条件每轮更新★: 若只在下面那个 if 内部填,
    // 恰好"因为 if 不成立所以流水线不动"这种情形就永远读不出来 —— 那正是要查的那一种。
    {
        const bool suppressed = SensorLink::getInstance()->output_suppressed();
        uint8_t diag = (uint8_t)st.boot_calibration.stage & 0x0Fu;
        if (st.provisioned)      diag |= 0x10u;
        if (suppressed)          diag |= 0x20u;
        if (psoc->heavy_busy())  diag |= 0x40u;
        if (psoc->link_alive())  diag |= 0x80u;
        g_usb_dbg.boot_cal_diag = diag;
        g_usb_dbg.boot_cal_fail_mask = st.boot_calibration.fail_mask;
    }
    if (st.provisioned && !SensorLink::getInstance()->output_suppressed()) {
        boot_calibration_tick(psoc, csd, st.boot_calibration);
        // "采样质量抽检存疑"是运行态**建议**, 必须能在问题消失后自己撤销 —— 否则它就退化成
        // 永久的历史判定(实测: 全通道启用且状态良好时面板仍在报存疑)。内部自带 10s 节流,
        // 且只在标志已置位时才真的去抽检, 健康设备上这一行是一次布尔比较。
        csd->tick_trust_recheck(psoc);
    }

    // 持续断开 >400ms 视为真复位 → 清 provisioned, 链路恢复后重下发(算法/CSD 在 PSoC RAM, 复位丢失)。
    // ★但"正在执行重操作"不算掉线★: APPLY/CALIBRATE/GLOBAL_COMMIT/AUTO_TUNE 由 PSoC 主循环同步跑,
    // provision 后的 APPLY 实测 12.4s(36 通道逐个重校准), 期间 CapSense 内部临界区推迟 SPI DMA 中断,
    // read_touch 成批失败、链路"断开"远超 400ms。旧逻辑据此清 provisioned → 链路一恢复就重新下发
    // 396 条 SET_PARAM(把 36 通道全部重新置脏) + APPLY → 又一次 12.4s 重校准 → 永久 provision 风暴。
    // 实测(带外 SWD 读 PSoC 计数): setparam 约 428/s、apply 约 1.1/s、apply_dirty 恒 36、
    // scan_count 每 14s 才 +1 ⇒ 扫描被彻底压住, raw 全 0、scan_period_us=0。
    // 宽限窗由 Psoc 在派发重操作时开启(见 psoc.cpp 的 HEAVY_OP_GRACE_MS)。
    if (!link_now && st.link_down_since_ms != 0u &&
        (millis() - st.link_down_since_ms) > 400u && !psoc->busy_grace_active()) {
        st.clear_provisioning();
    }
    // ★运行期掉枚举: 如实宣判失效, 不做救援★
    // 主机一旦拆掉接口, 设备这边任何"重新武装/重开"都改变不了主机的判断, 只会把主循环搅乱。
    // 事件是粘性的(队列 + note_rearm 兜底), 故即使掉枚举期间无人可发, 重新连上后仍会送达 ——
    // 于是"上次运行期间掉过枚举"永远有据可查, 而不是只剩用户一句"它自己断了"。
    {
        const bool usb_up = HAL_USB_Device::getInstance()->is_ready();
        if (st.usb_was_up && !usb_up) {
            // detail: bit0=serial 协议在跑, bit1=灯板协议已就绪 —— 便于分辨掉的是哪一侧的负载。
            uint32_t detail = 0u;
            GameIoService* gio = GameIoService::getInstance();
            if (gio->mai2_touch_sending()) detail |= 0x1u;
            if (gio->is_ready()) detail |= 0x2u;
            SelfHeal::getInstance()->note(SH_CDC_LOST, detail);
        }
        st.usb_was_up = usb_up;
    }

    // ★自持恢复事件必达兜底★: note() 只在事件发生那一刻拉起推送任务, 若当时上位机没连(或租约过期
    // 自取消), 队列里的事件就再也没人推 → 用户永远看不到"设备被固件改过"。故只要队列非空且任务不在,
    // 就重新拉起(有上位机时 10Hz 排空, 无上位机时靠租约自灭, 不常驻)。
    if (!SelfHeal::getInstance()->empty() && !TxScheduler::getInstance()->active(TX_TASK_SELFHEAL)) {
        SelfHeal::getInstance()->note_rearm();
    }
    loop_prof_mark(LOOP_SEG_PSOC, seg_t);
}

// ★每轮重写运行标记★: 判定"上次是否运行中崩溃"依赖 scratch[7]==WD_RUNNING_MAGIC。setup 里只
// 设一次的话, 任何把它冲掉的路径都会让判定失真。每轮一条 store, 代价可忽略。
// 这样下次崩溃后若仍读到 last_boot_was_wd=0, 就**确证**复位清掉了 scratch —— 看门狗复位与
// hardfault 都会保留 scratch, 只有上电复位(POR)会清。即: 供电跌落, 而非软件问题。
inline void core0_run_marks() {
    watchdog_hw->scratch[7] = WD_RUNNING_MAGIC;
    crash_run_mark();
}

// ★主机租约★：ping 续期。超时未续期即认定上位机丢失, 暂停遥测(自洽: 绿灯亦据此判连接)。
// ★只挂起、不永久停★: core0 可能只是被长设备操作(JIT 算法下发/校准/flash 落地)按在
// UsbComm::update() 之外几秒 —— 上位机其实一直在, 但租约照样过期。旧实现在这里调 stop(),
// 把 _streaming 清掉且没有任何恢复路径 → 上位机不会再发 TELEM_START → 全通道 raw/baseline/diff
// 永久冻结, 只能复位设备。改为挂起, 主机命令一到即用原参数自动续推。
inline void core0_host_lease() {
    if ((millis() - g_last_host_cmd_ms) > HOST_LEASE_TIMEOUT_MS) {
        SensorLink::getInstance()->suspend();
    } else {
        SensorLink::getInstance()->resume();
    }
}

// ★算法下发结果上报★: 下发已异步化(core1 执行), 失败只有 core1 知道。在此取走标志上报,
// 使"算法没真正装上"永远有据可查, 而不是让用户以为自定义算法正在跑。
inline void core0_algo_followup(const Core0State& st) {
    Psoc* psoc = Psoc::getInstance();
    if (psoc->algo_download_take_failure()) {
        SelfHeal::getInstance()->note(SH_ALGO_FALLBACK, 1u);
    }
    // 代码下发完成后补推每通道 ROM 与 cfg[8](不可放进 USB 命令处理器, 会拖住 ACK)。
    // 常规运行依赖 provisioned；但用户已受理的默认恢复必须独立推进，不能因链路代次
    // 尚未被标记为 provisioned 而永久挂起在旧自定义 blob 上。
    PsocAlgo* algo = PsocAlgo::getInstance();
    if (st.provisioned || algo->reset_default_pending()) algo->tick(psoc);
}

// ★大吞吐统一走定时任务队列★: 遥测等周期发送由 TxScheduler 按各自频率+租约驱动(续期制),
// 帧经非阻塞 config_write 入 vendor TX FIFO, 由 HAL_USB task() 泵出。不在主循环直接 tick 遥测。
inline void core0_tx_scheduler() {
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_TX_SCHED);
    TxScheduler::getInstance()->tick();
    loop_prof_mark(LOOP_SEG_TX_SCHED, seg_t);
}

inline void core0_game_io() {
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_GAME_IO);
    Psoc* psoc = Psoc::getInstance();
    {
        const uint32_t gio_t = gio_seg_begin(GIO_SEG_BINDING);
        // 掩码与"是否可信"都取 Psoc 的统一裁决(见 psoc.h touch_mask/touch_hold_ok):
        // 瞬时 link_ok 在遥测分页期间频繁为假, 用它清零会让指触绑定在按着的时候突然丢采样。
        // 扫描会话期间掩码无意义(见 SensorLink::output_suppressed): 按"不可信"喂给绑定捕获,
        // 免得把逐格改参数产生的噪声采成用户的指触样本。
        const bool touch_trusted = psoc->touch_hold_ok() &&
                                   !SensorLink::getInstance()->output_suppressed();
        const uint64_t touch_now = psoc->touch_mask();
        BindingService::getInstance()->tick(touch_now, touch_trusted);
        // ★HID 触摸屏点位: 复用同一对 (掩码, 可信) 裁决★
        // 不自行重算 touch_hold_ok/output_suppressed —— 两处各算一遍必然漂移出"绑定看得见触摸、
        // 触摸屏却不动"这类分叉。HID 模式外本调用内部直接返回, serial 模式零开销、零副作用。
#if MAI2_ENABLE_SERIAL_HID
        HidTouchMapper::getInstance()->tick(touch_now, touch_trusted);
#endif
        gio_seg_mark(GIO_SEG_BINDING, gio_t);
    }
    GameIoService::getInstance()->task();
    loop_prof_mark(LOOP_SEG_GAME_IO, seg_t);
}

// 物理键盘(GPIO1-12) + 触控→键盘映射 → HID(内部 task HID 发报文)。
inline void core0_keyboard() {
#if MAI2_ENABLE_SERIAL_HID
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_KEYBOARD);
    KeyboardService::getInstance()->task();
    loop_prof_mark(LOOP_SEG_KEYBOARD, seg_t);
#endif
}

// ★flash 落地安全窗口★：命令 handler 只置保存信号，实际 flash 写在此(命令已处理完、ACK 已发)执行。
// flash 写内部已用 disable_interrupts()+multicore_lockout(暂停 core1)保护 XIP 擦写窗口，
// 两核都不访问总线→写后 USB 自动恢复，无需重新枚举。
// ★每轮最多落一份★: 单份写要禁中断 + 停 XIP + lockout core1 数十~上百 ms, 期间 TinyUSB 的
// USB 中断完全得不到服务。一次"保存到设备"会同时置起 config/csd/algo 三个信号, 三份背靠背写 =
// 数百 ms 连续 USB 黑洞, 主机侧待处理的 OUT 传输会被 Windows 直接 abort
// (实测 kind=ConnectionAborted → 拆端点 → 判断开)。分轮落地, 每份之间必有一次完整 USB 服务轮。
inline void core0_nv_commit() {
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_NV_COMMIT);
    Psoc* psoc = Psoc::getInstance();
    // ★必须等 core1 空闲才落盘★: flash 写内部 multicore_lockout_start_blocking(core1), 而
    // core1 若正在执行重操作(PSoC 重初始化 / 全通道校准, _wait_op_done 轮询数秒), 期间它不进
    // wfe 也就响应不了 lockout ⇒ core0 死等、不喂狗 ⇒ 5s 看门狗复位整机。实测正是"保存后约
    // 8.5 秒掉线 + 设备重新枚举 + LED 重启"。落盘信号是粘性的, 推迟一轮没有任何副作用。
    // ★本层绝对不要再套 lockout / 关中断★
    // save_config_task / CsdConfig::save / PsocAlgo::save **内部已经**有完整的
    // FlashWriteGuard + save_and_disable_interrupts() + multicore_lockout_start_blocking()。
    // 在这里再包一层 = 嵌套 lockout: core1 已被内层锁住, 响应不了外层请求, core0 永久死等。
    // 本层只负责"什么时候允许落盘"这个门控。
    // ★安静窗口只管"起片", 不管"续片"★: 落盘已按扇区分片跨轮进行(见 NvStore 的分片注释)。
    // 若续片也要等安静窗口, 命令洪流下一个区会长时间停在半成品状态 —— 单份存储下那正是最该
    // 缩短的窗口。core1_idle 仍是硬条件(它关系到 lockout 死锁, 不能让)。
    // A save transaction is the sole owner of persistence preparation.
    // Ordinary runtime mutations stay dirty in RAM until an explicit
    // SAVE_CONFIG request claims them here.
    if (PersistenceTxn::getInstance()->take_prepare_request()) {
        ConfigManager::save_config();
        CsdConfig::getInstance()->request_save();
        PsocAlgo::getInstance()->request_save();
    }
    const bool commit_window_open = psoc->core1_idle() &&
        (NvStore::getInstance()->commit_in_progress() ||
         (static_cast<uint32_t>(millis() - g_last_host_cmd_ms) >= NV_COMMIT_QUIET_MS &&
          !UsbComm::getInstance()->has_pending_response()));
    if (commit_window_open) {
        // ★先把各服务的"待保存"信号收进 NvStore 镜像(纯内存), 再由 commit_step 落一个区★
        // 这三步都不擦写 flash, 只更新镜像 + 置脏; 真正的擦写只有下面 commit_step 一处。
        bool prepared = true;
        if (ConfigManager::has_pending_save()) {
            prepared = ConfigManager::save_config_task() && prepared;
        }
        if (CsdConfig::getInstance()->has_pending_save()) {
            prepared = CsdConfig::getInstance()->save() && prepared;
        }
        if (PsocAlgo::getInstance()->has_pending_save()) {
            prepared = PsocAlgo::getInstance()->save() && prepared;
        }
        if (PersistenceTxn::getInstance()->active()) {
            PersistenceTxn::getInstance()->note_prepare_result(prepared);
        }
        // 每轮最多落一个脏区: 一次"保存到设备"通常脏了 KV + 若干 blob, 背靠背写会连续几百 ms
        // 停 XIP/关中断, 主机在途传输被 abort。摊到多轮, 每轮之间 USB 正常服务。
        if (NvStore::getInstance()->dirty()) {
            crash_stage_set(CRASH_STAGE_CFG_FLASH);
            if (NvStore::getInstance()->commit_step()) {
                g_usb_dbg.flash_write_count++;
                g_usb_dbg.loop_at_last_flash = g_usb_dbg.loop_count;
            }
            crash_stage_set(CRASH_STAGE_NONE);
        }
    }
    loop_prof_mark(LOOP_SEG_NV_COMMIT, seg_t);
    // The transaction service only observes existing NvStore state; it never
    // extends the flash window. UsbComm sends its terminal reply next loop.
    PersistenceTxn::getInstance()->poll();
}

inline void core0_debug_mirror() {
    NvStore* nv = NvStore::getInstance();
    Psoc* psoc = Psoc::getInstance();
    g_usb_dbg.nv_dirty_mask = nv->dirty_mask();
    g_usb_dbg.nv_commit_ok = nv->commit_ok_count();
    g_usb_dbg.nv_commit_fail = nv->commit_fail_count();
    g_usb_dbg.nv_algo_src_len = nv->algo_src_len();
    g_usb_dbg.nv_valid_mask = nv->valid_mask();
    g_usb_dbg.psoc_heavy_rejects = psoc->heavy_reject_count();
    g_usb_dbg.psoc_heavy_busy = psoc->heavy_busy() ? 1u : 0u;
    // 新代数通知线实证(core1 写, core0 只读搬运)。armed=1 即 core1 已停止空转轮询。
    g_usb_dbg.core1_int1_edges = psoc->int1_edges();
    g_usb_dbg.core1_int1_timeouts = psoc->int1_timeouts();
    g_usb_dbg.core1_int1_armed = psoc->int1_armed() ? 1u : 0u;
    // core1 阶段码镜像进上报结构(core1 写 g_core1_stage, core0 只读搬运)。
    g_usb_dbg.core1_stage = g_core1_stage;
}

// ★主循环心跳★：LED 每 150ms 亮灭翻转 = loop 在跑；若卡住则 LED 停在某态(不再闪)。
// 状态优先级保持既有语义：主机连接常亮优先，其余依次为 flash 未就绪、SPI 链路断、健康。
inline void core0_status_led(Core0State& st) {
    const uint32_t seg_t = loop_seg_begin(LOOP_SEG_LED);
    const PsocBringupReport& report = PsocUpdater::getInstance()->report();
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
    if (!led_enabled) {
        led->set_color(0u, 0u);
        st.hb_on = false;
        st.hb_last_ms = millis();
    } else if (!st.provisioned) {
        // ★PSoC 启动/重启加载指示★：链路(重新)建立、CSD/算法尚未下发完成前状态灯常亮(白),
        // provisioned 置真(进入正常工作)即转入下方常规状态逻辑, 便于肉眼清晰分辨 PSoC 重启窗口。
        led->set_color(7u /* 白 */, status_brightness);
        st.hb_on = false;
        st.hb_last_ms = millis();
    } else if (host_connected) {
        led->set_color(connected_color, status_brightness);
        st.hb_on = false;
        st.hb_last_ms = millis();
    } else if (millis() - st.hb_last_ms >= 150) {
        st.hb_last_ms = millis();
        st.hb_on = !st.hb_on;
        const uint8_t status_color = !report.flash_ok()
            ? flash_error_color
            : (!report.link_ok ? link_error_color : healthy_color);
        led->set_color(status_color, st.hb_on ? status_brightness : 0u);
    }
    loop_prof_mark(LOOP_SEG_LED, seg_t);
}

// core0 每轮编排。顺序即契约, 下列相邻关系都是实测得来的, 不可随意调换:
//   USB/总线服务 → HostCmd(先于 provisioning 取得优先权) → PSoC 代次生命周期
//   → 运行标记/租约/算法补推 → 定时发送 → 触控快路 → 键盘 → flash 落地窗口
//   → 调试镜像 → vendor OUT 自愈(必须在落盘之后) → 状态灯。
// 整轮耗时由 loop_prof_begin/total 量测, 它是与 5s 看门狗竞争的真实值。
inline void core0_loop_step(Core0State& st) {
    g_usb_dbg.loop_count++;
    // ★阻塞剖面★: loop_t0 量整轮, seg_t 量各段。纯测量, 不改控制流。
    const uint32_t loop_t0 = loop_prof_begin();

    core0_usb_task();
    core0_host_cmd();
    core0_bootsel_if_requested();
    if (core0_swd_released_step()) return;

    core0_psoc_lifecycle(st);
    core0_run_marks();
    core0_host_lease();
    core0_algo_followup(st);
    core0_tx_scheduler();
    core0_game_io();
    core0_keyboard();
    core0_nv_commit();
    core0_debug_mirror();
    // ★vendor OUT 自愈★：每轮检查 config vendor OUT 是否仍处 arm 态, 若因 flash 扰动等掉出则
    // 重新武装。放在 flash 落地之后, 确保刚写完 flash 即可立即恢复 host→device 接收。
    HAL_USB_Device::getInstance()->vendor_service();
    core0_status_led(st);

    loop_prof_total(loop_t0);
    watchdog_update();
}

}  // namespace app
