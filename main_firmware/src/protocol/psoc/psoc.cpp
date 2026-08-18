#include "psoc.h"
#include "../../config.h"
#include "../../service/psoc_algo/psoc_algo.h"   // PSOC_ALGO_MAX_LEN(= PSoC ALGO_SLOT_SIZE), 只取常量
#include "../../service/self_heal/self_heal.h"  // 容量常量漂移必须推送给上位机留痕
#include "../../service/latency_stats.h"
#include "../../hal/usb/hal_usb.h"
#include <Arduino.h>
#include <pico/stdlib.h>
#include <hardware/sync.h>   // __dmb() / __sev() 跨核内存屏障
#include <hardware/watchdog.h>   // watchdog_update(): 重操作阻塞等待期间喂狗
#include "../../service/usb_debug.h"   // crash_stage_set(): 阻塞等待期间记录阶段码

namespace {
// ★core1 改为动态响应式★
// 原实现每轮把时间补齐到固定 1ms。那让"响应延迟"恒等于一个完整周期: 触控帧最坏要等 1ms 才被
// 取走, 命令环里的读类命令同样按 1ms 的粒度排队。而 PSoC 侧与 RP 侧都带时间戳、时序由 PSoC 自己
// 的扫描周期决定 —— RP 这边并不需要靠"恒定轮询节拍"来维持时序一致性, 各子任务(快照按扫描周期、
// GET_STATS 按 250ms)本就各自节流。
// 现在: 做完即进入下一轮, 只保留一个最小间隔下限。下限的作用是给 PSoC 的 CapSense 留出 SPI 空窗
// (它的 DMA ISR 会推迟中间件的临界区), 而不是为了对齐节拍。
// 效果: 触控与命令响应的最坏等待从 1ms 降到 CORE1_MIN_GAP_US。
constexpr uint32_t CORE1_MIN_GAP_US = 150;        // 相邻两轮之间给 PSoC 留的最小 SPI 空窗
// ★再进一步: 由 PSoC 主动通知取代"按最小间隔不停试"★
// 最小间隔只是把空转变慢, 并没有消除空转: PSoC 每 scan_period(实测约 6.1ms)才产出一份新代数,
// 而 core1 每 150us 就去读一次触控 —— 约 40 次里只有 1 次拿到的是新数据, 其余 39 次纯粹是白发
// SPI 事务, 每一次都让 PSoC 的 DMA ISR 抢走一小段 CapSense 中间件的临界区时间。
// PSoC 现在每发布一份新快照就翻转 P1.4(SENSOR-INT1 → GPIO23), core1 等这个翻转再干活。
// 门限/兜底(缺一不可, 否则通知线一有问题整机就哑了):
//   · 先自由跑, 累计观测到 INT1_ARM_EDGES 次翻转才认定通知线可信(armed);
//   · armed 后等待上限取实测扫描周期的两倍 + 1ms 余量, 迟到即算这一轮通知丢失;
//   · 连续 INT1_DISARM_TIMEOUTS 次超时就解除 armed 退回自由跑(等价于改造前的行为),
//     之后一旦重新观测到翻转会自动再 arm —— 旧 PSoC 固件、通知线断线、PSoC 复位期间都能自愈。
constexpr uint32_t INT1_ARM_EDGES        = 4;      // 认定通知线可信所需的翻转次数
constexpr uint32_t INT1_DISARM_TIMEOUTS  = 3;      // 连续超时到此即退回自由跑
constexpr uint32_t INT1_WAIT_FALLBACK_US = 3000;   // 扫描周期尚未测出时的等待上限
constexpr uint32_t INT1_WAIT_CAP_US      = 20000;  // 等待上限硬顶(保证 250ms 级周期任务不被拖死)
constexpr uint32_t SPI_CMD_TIMEOUT_US = 100000;  // core0 命令信箱等待上限 100ms
// 失效兜底阈值:
//   链路丢失: 连续 ~200 个周期取不到新鲜的合法状态帧 = PSoC 崩溃/掉线。
//   主循环卡死: scan_count 连续 8 个统计间隔(每间隔 ~500ms ⇒ ≈4s)不推进 = 主循环卡死(疑似坏算法)。
//     阈值刻意高于最长合法主循环停顿(MEASURE_CP 逐电极测量 ~1.5s), 避免测量期间误复位。
constexpr uint32_t LINK_FAIL_RESET_CYCLES = 200;
constexpr uint32_t HANG_STATS_INTERVALS   = 8;
constexpr uint32_t RUNTIME_PARAM_APPLY_TIMEOUT_MS = 3000u;
// ★XRES 复位后 PSoC 启动宽限★: XRES 复位(手动 REBOOT_PSOC 或失效兜底)后 PSoC 需重跑
// initialize_capsense(含 Cy_CapSense_Enable 全通道自动校准)才恢复 SPI 应答, 耗时可达数百 ms,
// 远超 LINK_FAIL_RESET_CYCLES(200ms)。若不设宽限, 兜底会在 PSoC 启动完成前又发 XRES →
// 永久复位死循环 → 链路永不恢复(表现: 重启后 link 掉、需重烧)。宽限期内不累计失败/不再触发复位。
constexpr uint32_t RESET_BOOT_GRACE_MS    = 1500;
// ★重操作执行宽限★: APPLY/CALIBRATE/GLOBAL_COMMIT/BASELINE_RESET 由 PSoC 主循环同步执行,
// 实测 provision 后的 APPLY(36 通道逐个重校准)达 12.4s。期间 CapSense 内部临界区推迟 SPI DMA 中断,
// read_touch 会成批失败 —— 那不是掉线, 不得据此 XRES 或重新下发配置。给足余量到 30s。
// AUTO_TUNE 更长(逐通道最坏数十秒), 与其 SPI 层 45s 超时对齐并留余量。
// 全通道 IDAC 校准现在按真实最坏值允许 60s；宽限必须比它更长，否则操作刚完成就会继承
// 期间累积的链路失败并被误 XRES。普通单通道操作也共用此窗，代价只是延后兜底，不改变完成判据。
constexpr uint32_t HEAVY_OP_GRACE_MS      = 70000;
constexpr uint32_t AUTOTUNE_GRACE_MS      = 60000;
}

Psoc* Psoc::_instance = nullptr;

Psoc::Psoc()
    : _swd(PIN_SWD_IO, PIN_SWD_CLK, PIN_SWD_RST),
      _link(PIN_PSOC_SPI_SCK, PIN_PSOC_SPI_MOSI, PIN_PSOC_SPI_MISO, PIN_PSOC_SPI_CS),
      _spi_ready(false), _swd_ready(false), _link_ok(false), _last_update_ms(0) {}

Psoc* Psoc::getInstance() {
    if (!_instance) {
        _instance = new Psoc();
    }
    return _instance;
}

bool Psoc::init() {
    // SPI(PIO1) 是链路通道；SWD(PIO0) 是编程通道。二者互不干扰。
    _spi_ready = _link.init();
    _swd_ready = _swd.init();
    return _spi_ready;
}

// ======================================================================
// 双核 SPI 服务
//   core1: core1_run() 固定 1ms 周期独占调用 _spi_service()。
//   core0: setup 阶段(core1 未启动)经 update() 直调一次; 运行期不再触碰 SPI。
//   共享态经 seqlock 发布(u64/快照防撕裂), 低频指令经命令信箱投递。
// ======================================================================

// 长周期指令判定(反堆叠闸门与"设备忙"上报的唯一名单)。判据 = 该操作由 PSoC 主循环同步执行、
// 耗时可达秒级(逐通道校准实测 12s+, 自适应 20s+)。写在一处, 避免各调用点各列一份名单而漂移。
bool Psoc::_op_is_heavy(SpiOp op) {
    switch (op) {
        case SpiOp::APPLY:
        case SpiOp::CALIBRATE:
        case SpiOp::BASELINE_RESET:
        case SpiOp::GLOBAL_COMMIT:
        case SpiOp::AUTO_TUNE:
        case SpiOp::MEASURE_CP:
        case SpiOp::UPLOAD_ALGO:
            return true;
        default:
            return false;
    }
}

// 一帧同时取回 scan_count 与设备忙位。★busy 不再是单独一条命令★: LINK v2 的每份响应都带 st,
// 故这里的 busy 直接取本次响应发布的 st.OP_BUSY —— v1 里 GET_STATS 被高频轮询、反过来抢占
// 从机主循环空窗的那条路径就此消失。
bool Psoc::_link_stats(uint32_t* out_scan_count, uint8_t* out_busy) {
    uint8_t payload[LNK_PAYLOAD_BYTES] = {0};
    if (!_link.request(LNK_CMD_STATS, nullptr, payload)) return false;
    if (out_scan_count) *out_scan_count = lnk_rd32(payload);   // ms_tick 在 payload[4..7], 暂无消费者
    if (out_busy) *out_busy = _link.operation_busy() ? 1u : 0u;
    return true;
}

bool Psoc::_link_param_get(uint8_t ch, uint8_t param_id, uint32_t* out_value) {
    uint8_t args[LNK_ARG_BYTES] = {0};
    args[0] = ch;
    args[1] = param_id;
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};
    if (!_link.request(LNK_CMD_PARAM_GET, args, p)) return false;
    if (p[0] != ch || p[1] != param_id) return false;
    if (out_value) *out_value = lnk_rd32(&p[2]);
    return true;
}

bool Psoc::_wait_op_done(uint32_t timeout_ms, bool with_progress) {
    // 阶段1: 等 busy=1(操作极快或已完成时等不到, 直接进阶段2)
    absolute_time_t d1 = make_timeout_time_ms(40);
    for (;;) {
        uint8_t busy = 0u;
        if (_link_stats(nullptr, &busy) && busy != 0u) break;
        if (time_reached(d1)) break;
        sleep_ms(2);
    }
    // 阶段2: 等 busy=0 —— 从机主循环真正完成重操作的唯一判据
    absolute_time_t d2 = make_timeout_time_ms(timeout_ms);
    absolute_time_t next_progress = make_timeout_time_ms(PROGRESS_POLL_MS);
    for (;;) {
        uint8_t busy = 1u;
        if (_link_stats(nullptr, &busy) && busy == 0u) return true;
        if (time_reached(d2)) return false;
        // ★阶段性进度★ 长操作(自适应最坏 ~20s)期间降频读一次进度并发布, 使上位机能持续看到
        // "到哪一步了"; busy 判定与超时窗完全不受影响。
        if (with_progress && time_reached(next_progress)) {
            next_progress = make_timeout_time_ms(PROGRESS_POLL_MS);
            uint8_t p[LNK_PAYLOAD_BYTES] = {0};
            if (_link.request(LNK_CMD_AUTO_TUNE_GET, nullptr, p)) {
                _at_work.state = 1;
                _at_work.result = p[0];
                _at_work.ch = p[1];        // 全通道模式下从机回显"当前正在处理的通道"
                _at_work.cur_div = lnk_rd16(&p[2]);
                _at_work.phase = p[4];
                _at_work.step = p[5];
                _publish_autotune();
            }
        }
        sleep_ms(3);
    }
}

bool Psoc::_poll_heavy_async() {
    if (_heavy_async.active == 0u) return false;
    const uint32_t now_ms = millis();
    // 状态探针与触控保活各 20ms，错开半周期。PSoC 只有在无 SPI 中断窗口才会消费 pending，
    // 因此不能在这里高频自旋 GET_STATS，否则会把“等待完成”本身变成完成不了的原因。
    if ((uint32_t)(now_ms - _heavy_async.last_poll_ms) < 20u) return true;
    _heavy_async.last_poll_ms = now_ms;
    uint32_t scan_count = 0u;
    uint8_t busy = 0u;
    const bool response_ok = _link_stats(&scan_count, &busy);
    if (!response_ok) {
        if (_heavy_async.response_fail_run < 0xFFu) _heavy_async.response_fail_run++;
    } else {
        _heavy_async.response_fail_run = 0u;
        const bool scan_advanced = _heavy_async.scan_seen != 0u &&
                                   scan_count != _heavy_async.last_scan_count;
        _heavy_async.last_scan_count = scan_count;
        _heavy_async.scan_seen = 1u;
        if (busy != 0u) {
            _heavy_async.seen_busy = 1u;
            if (scan_advanced) {
                if (_heavy_async.scan_progress_polls < 2u) _heavy_async.scan_progress_polls++;
                // 部署版 PSoC 可残留 busy=1；APPLY 后参数/算法已由 provisioning 队列写完，
                // 连续 scan_count 推进才是主循环重新健康的终态。仅一次推进可能是受理前扫描收尾。
                if (_heavy_async.scan_progress_polls >= 2u) {
                    _complete_heavy_async(true);
                    return false;
                }
            }
        } else if (_heavy_async.seen_busy != 0u || scan_advanced) {
            _complete_heavy_async(true);
            return false;
        }
    }
    // 生命周期只以 PSoC busy 真值闭合；120s 是失联/固件卡死的长兜底，不是正常完成时间。
    if ((uint32_t)(now_ms - _heavy_async.started_ms) >= 120000u) {
        _complete_heavy_async(false);
        return false;
    }
    return true;
}

void Psoc::_complete_heavy_async(bool ok) {
    _heavy_async.clear();
    _heavy_done = _heavy_done + 1u;
    if (!ok && _pub_reset_reason == 0u) _pub_reset_reason = 2u;
}

void Psoc::_complete_runtime_param_apply(bool ok) {
    _runtime_apply.ok = ok ? 1u : 0u;
    _runtime_apply.active = 0u;
    _runtime_apply.pending = 0u;
    _runtime_apply.complete = 1u;
    // Runtime parameter apply is deliberately outside the heavy lifecycle.
    // Failure is reported to SensorLink, which records a hole and restores RAM state.
}

bool Psoc::_poll_runtime_param_apply() {
    if (_runtime_apply.active == 0u) return false;
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - _runtime_apply.last_poll_ms) < 20u) return true;
    _runtime_apply.last_poll_ms = now_ms;

    uint32_t scan_count = 0u;
    if (!_link_stats(&scan_count, nullptr)) {
        if (_runtime_apply.response_fail_run < 0xFFu) _runtime_apply.response_fail_run++;
        if (_runtime_apply.response_fail_run < 25u &&
            (uint32_t)(now_ms - _runtime_apply.started_ms) < RUNTIME_PARAM_APPLY_TIMEOUT_MS) return true;
        _complete_runtime_param_apply(false);
        return false;
    }
    _runtime_apply.response_fail_run = 0u;

    if (_runtime_apply.scan_seen != 0u && scan_count != _runtime_apply.last_scan_count &&
        _runtime_apply.params_confirmed != 0u) {
        _runtime_apply.scan_progress_polls = 1u;
    }
    _runtime_apply.last_scan_count = scan_count;
    _runtime_apply.scan_seen = 1u;

    if (_runtime_apply.params_confirmed == 0u &&
        (uint32_t)(now_ms - _runtime_apply.last_param_poll_ms) >= 100u) {
        _runtime_apply.last_param_poll_ms = now_ms;
        uint32_t gain = 0u;
        uint32_t div = 0u;
        if (_link_param_get(_runtime_apply.target_ch, 0x0Bu, &gain) &&
            _link_param_get(_runtime_apply.target_ch, 0x08u, &div) &&
            gain == _runtime_apply.target_gain && div == _runtime_apply.target_div) {
            _runtime_apply.params_confirmed = 1u;
            _runtime_apply.last_scan_count = scan_count;
            _runtime_apply.scan_progress_polls = 0u;
        }
    }

    if (_runtime_apply.params_confirmed != 0u && _runtime_apply.scan_progress_polls != 0u) {
        _complete_runtime_param_apply(true);
        return false;
    }
    if ((uint32_t)(now_ms - _runtime_apply.started_ms) >= RUNTIME_PARAM_APPLY_TIMEOUT_MS) {
        _complete_runtime_param_apply(false);
        return false;
    }
    return true;
}

bool Psoc::_poll_algo_upload_async() {
    if (_algo_upload_async.active == 0u) return false;
    const uint32_t now_ms = millis();
    // 页阶段由 core1 的 1ms 周期自然推进；commit 阶段的 INFO 节流由 SPI 状态机自身保证。
    // 不在这里再限频，否则默认 135 页会被人为拉长到数秒，错过主机的即时回读窗口。
    _algo_upload_async.last_poll_ms = now_ms;
    bool complete = false;
    bool ok = false;
    bool valid = false;
    uint16_t len = 0u;
    const bool response_ok = _link.poll_upload_algo(&complete, &ok, &valid, &len);
    if (!response_ok) {
        if (_algo_upload_async.response_fail_run < 0xFFFFu) _algo_upload_async.response_fail_run++;
        // ★与链路层同一个上限★(原来两边各写一个 120000u, 改一处必漏一处)。见
        // PsocLink::ALGO_UPLOAD_TIMEOUT_MS 注释: 上限长 = 上传通道被 _algo_dl.busy 锁得久。
        if (!complete && _algo_upload_async.response_fail_run < 200u &&
            (uint32_t)(now_ms - _algo_upload_async.started_ms) <
                PsocLink::ALGO_UPLOAD_TIMEOUT_MS) return true;
        _algo_upload_async.clear();
        _algo_dl.failed = 1u;
        _algo_dl.busy = 0u;
        _heavy_done = _heavy_done + 1u;
        _invalidate_algo_info_cache();
        return false;
    }
    _algo_upload_async.response_fail_run = 0u;
    if (!complete) {
        if (valid || len != 0u) _publish_algo_info_cache(valid, len);
        if ((uint32_t)(now_ms - _algo_upload_async.started_ms) <
                PsocLink::ALGO_UPLOAD_TIMEOUT_MS) return true;
        _algo_upload_async.clear();
        _algo_dl.failed = 1u;
        _algo_dl.busy = 0u;
        _heavy_done = _heavy_done + 1u;
        _invalidate_algo_info_cache();
        return false;
    }
    if (!ok) {
        _algo_upload_async.clear();
        _algo_dl.failed = 1u;
        _algo_dl.busy = 0u;
        _heavy_done = _heavy_done + 1u;
        _invalidate_algo_info_cache();
        return false;
    }
    _publish_algo_info_cache(true, len);
    _algo_upload_async.clear();
    _algo_dl.busy = 0u;
    _heavy_done = _heavy_done + 1u;
    return true;
}

// core1 每周期执行体: 命令信箱 → 触控快路 → 快照慢路 → 采样率统计。
void Psoc::_spi_service() {
    if (!_spi_ready) return;

    const bool heavy_poll_cycle = _heavy_async.active != 0u;
    const bool heavy_still_active = _poll_heavy_async();
    const bool runtime_apply_poll_cycle = _runtime_apply.active != 0u;
    const bool algo_poll_cycle = _algo_upload_async.active != 0u;
    _poll_runtime_param_apply();
    _poll_algo_upload_async();
    // 1) 命令队列: 每周期最多消费 CMD_DRAIN_PER_CYCLE 条(限制周期抖动), FIFO 顺序执行。
    for (uint32_t n = 0; n < CMD_DRAIN_PER_CYCLE; ++n) {
        if (heavy_poll_cycle || heavy_still_active || _heavy_async.active != 0u ||
            runtime_apply_poll_cycle || _runtime_apply.active != 0u || algo_poll_cycle || _algo_upload_async.active != 0u) break;
        if (_cmd_tail == _cmd_head) break;          // 队空
        SpiCmd& c = _cmd_ring[_cmd_tail];
        uint32_t r = 0;
        // ★in_cmd 必须包住整条执行★: core0 侧的 flash 落地要 multicore_lockout core1, 而重操作
        // (PSoC 重初始化 / CalibrateAllWidgets)在 _wait_op_done 里轮询数秒, 期间 core1 不进 wfe
        // 也就响应不了 lockout ⇒ core0 死等且不喂狗 ⇒ 5s 看门狗复位整机(实测: 保存后约 8.5s 掉线,
        // 设备重新枚举、LED 重启)。core0 据此标志避开这个窗口落盘。
        _core1_in_cmd = 1u;
        // ★写 core1 自有阶段字段, 不写 core0 的 scratch[1]★
        // 原先这里用 crash_stage_set(CRASH_STAGE_CORE1_CMD) 且执行完从不复位, 于是 core0 崩溃后
        // core1 会把 7 盖到 core0 的真实阶段上 —— "死前遗言"恒报 CORE1_CMD, 排查被引到错误的核。
        core1_stage_set(CRASH_STAGE_CORE1_CMD);
        __dmb();
        const bool runtime_apply_cmd = c.async_owner == AsyncOwner::RUNTIME_PARAM &&
                                       c.async_token != 0u && c.async_token == _runtime_apply.token;
        if (runtime_apply_cmd) {
            bool ok = false;
            if (c.op == SpiOp::RUNTIME_PARAM_APPLY) {
                // 与 _exec_cmd 的同名分支同一笔事务(QUICK_APPLY); 这里单独走一遍只是为了在受理成功
                // 时立刻挂起 RP 侧那条独立的完成生命周期, 且不写普通 heavy 的忙态。
                ok = _exec_cmd(c.op, c.ch, c.pid, c.val, nullptr);
            }
            c.result = 0u;
            c.ok = ok;
            __dmb();
            c.done = true;
            if (ok) {
                // Runtime apply is not a heavy operation and must not extend the heavy grace window.
                _reset_grace_until_ms = millis();
                _hang_intervals = 0u;
                _runtime_apply.active = 1u;
                _runtime_apply.started_ms = millis();
                _runtime_apply.last_poll_ms = _runtime_apply.started_ms;
                _runtime_apply.last_touch_ms = _runtime_apply.started_ms - 10u;
                _runtime_apply.response_fail_run = 0u;
                _runtime_apply.scan_progress_polls = 0u;
                _runtime_apply.params_confirmed = 0u;
                _runtime_apply.last_param_poll_ms = 0u;
                _runtime_apply.last_scan_count = 0u;
                _runtime_apply.scan_seen = 0u;
                __dmb();
            } else {
                _complete_runtime_param_apply(false);
            }
        } else {
            const bool exec_ok = _exec_cmd(c.op, c.ch, c.pid, c.val, &r, c.data);
            c.result = r;
            c.ok = exec_ok;
            __dmb();
            c.done = true;
            if (c.async_owner == AsyncOwner::HOST_WRITE &&
                c.async_token != 0u && c.async_token == _host_write.token) {
                _host_write.ok = exec_ok ? 1u : 0u;
                __dmb();
                _host_write.pending = 0u;
                _host_write.complete = 1u;
            }
            if (_op_is_heavy(c.op)) {
                if (c.op == SpiOp::APPLY && exec_ok) {
                    // APPLY 只确认受理，之后由低频 GET_STATS 观察 PSoC busy 自然归还。
                    _heavy_async.active = 1u;
                    _heavy_async.seen_busy = 0u;
                    _heavy_async.started_ms = millis();
                    _heavy_async.last_poll_ms = _heavy_async.started_ms - 20u;
                    _heavy_async.last_touch_ms = _heavy_async.started_ms - 10u;
                    _heavy_async.response_fail_run = 0u;
                    _heavy_async.scan_seen = 0u;
                    _heavy_async.scan_progress_polls = 0u;
                    _heavy_async.last_scan_count = 0u;
                    _reset_grace_until_ms = _heavy_async.started_ms + 130000u;
                } else if (c.op == SpiOp::UPLOAD_ALGO && exec_ok) {
                    // 代码上传由独立状态机推进，直到 commit 的双次 INFO 确认后才归还 heavy 槽。
                } else {
                    _heavy_done = _heavy_done + 1u;
                }
            }
        }
        __dmb();
        _cmd_tail = (_cmd_tail + 1u) % CMD_RING_SIZE;
        __dmb();
        _core1_in_cmd = 0u;
        core1_stage_set(CRASH_STAGE_NONE);   // 本条执行完即复位, 阶段码不再"粘住"
    }

    // 2) 触控快路：常态 1kHz；普通 heavy 与 runtime apply 异步生命周期中降为 50Hz。
    // PSoC 只在 CapSense NOT_BUSY 时消费 pending；若仍每 1ms 触发 SPI ISR，它可能永远抢不到
    // 主循环空窗。降频期间跳过的周期不是链路失败，不更新失败连击；真实保活帧仍按既有逻辑发布。
    const uint32_t touch_now_ms = millis();
    const bool async_active = _heavy_async.active != 0u || _runtime_apply.active != 0u ||
                              _algo_upload_async.active != 0u;
    const uint32_t async_last_touch_ms = (_algo_upload_async.active != 0u)
        ? _algo_upload_async.last_touch_ms
        : ((_runtime_apply.active != 0u) ? _runtime_apply.last_touch_ms : _heavy_async.last_touch_ms);
    const bool touch_due = !async_active ||
        (uint32_t)(touch_now_ms - async_last_touch_ms) >= 20u;
    uint64_t mask = 0;
    uint32_t tr = 0u;
    bool ok = _pub_link_ok;
    if (touch_due) {
        if (_algo_upload_async.active != 0u) _algo_upload_async.last_touch_ms = touch_now_ms;
        if (_runtime_apply.active != 0u) _runtime_apply.last_touch_ms = touch_now_ms;
        if (_heavy_async.active != 0u) _heavy_async.last_touch_ms = touch_now_ms;
        // ★LINK v2: 触控不再有专门的 read_touch 事务★
        // 掩码由从机的主动状态帧(以及链路内部按需投的 STATUS 事务)持续发布, 这里只负责推进链路:
        // pump 会优先发出待发事务, 无事可发时发一帧 tag=0 轮询。
        const uint32_t t0 = time_us_32();
        const uint32_t frames = _link.pump(PSOC_SNAPSHOT_PAGES_PER_PUMP);
        const uint32_t t1 = time_us_32();
        tr = t1 - t0;
        // 本轮无事可发时 pump 一帧不发(计数器未超时), 那不是一次"耗时 0 的链路事务", 计进去会把
        // 延迟统计洗成 0。
        if (frames != 0u) latency_note(&g_lat_spi_us, tr);
        // ★存活判据 = 计数器新鲜度, 不是"本轮发了几帧"★
        // 链路改成"任何成功收帧都重置计数器, 超时才发探测帧": 于是空闲期大多数 core1 周期本来就
        // 一帧都不发(计数器还新鲜), 若把 frames!=0 当存活条件, 健康的空闲链路会被判成掉线。
        // 又: 掩码只搭在主动状态帧/STATUS 响应上, 密集命令流(快照 36 帧、算法 410 帧)期间它会被
        // 正常挤后, 所以"存活"与"掩码新鲜"必须分成两个判据。
        ok = _link.frame_fresh(LINK_STATUS_FRESH_US);
        const bool mask_fresh = _link.status_fresh(TOUCH_HOLD_US);
        mask = _link.touch_mask();

        // ★触发式更新 + 有限保留 + 到期优雅释放★(掩码即真相, 见 psoc.h touch_mask 注释)
        _pub_seq++;
        __dmb();
        // ★保留/释放改成按时间判★ 以前是"连续 N 个 core1 周期取不到合法帧就释放", 那建立在
        // "每个周期都必然发一笔事务"之上。现在链路是事件驱动的, 空闲周期一帧都不发、周期本身也快
        // 得多 —— 再按周期数计, 60 个周期可能只过去几百微秒, 会把正常的空闲判成假抬起。
        if (mask_fresh) {
            _pub_touch_mask = mask;
            _pub_touch_hold = true;
        } else {
            _pub_touch_bad = _pub_touch_bad + 1u;
            if (_pub_touch_hold) {
                // 掩码已陈旧超过 TOUCH_HOLD_US: 优雅释放为全 0, 而不是把按下状态永久保持住。
                _pub_touch_mask = 0;
                _pub_touch_hold = false;
                _pub_touch_releases = _pub_touch_releases + 1u;
            }
        }
        _pub_link_ok = ok;
        if (ok) _pub_link_ok_ms = touch_now_ms;
        _pub_touch_read_us = tr;
        // 掩码到手的时刻。core0 的延迟补偿以此为基准计算"采样 → 实际发出"的真实间隔;
        // 失败周期保留上一次的时间戳，因为此时对外发布的仍是上一份掩码。
        // 只有掩码真的换新才更新采样时刻: 它是延迟补偿的时间基准, 用"链路活着"当条件会把
        // 一份旧掩码标成刚采到的, 补偿量就被少算了。
        if (ok && mask_fresh) _pub_touch_sample_us = t1;
        __dmb();
        _pub_seq++;

        // 掉线升级同样改成按时间判(理由同上: 周期数在事件驱动下已不再代表时间)。
        if (ok) {
            _link_established = true;
            _link_fail_since_ms = 0u;
        } else if (_link_established) {
            _invalidate_algo_info_cache();
            if ((int32_t)(_reset_grace_until_ms - touch_now_ms) > 0) {
                _link_fail_since_ms = 0u;
            } else if (_link_fail_since_ms == 0u) {
                _link_fail_since_ms = (touch_now_ms != 0u) ? touch_now_ms : 1u;
            } else if ((uint32_t)(touch_now_ms - _link_fail_since_ms) >= LINK_FAIL_RESET_MS &&
                       _pub_reset_reason == 0) {
                _pub_reset_reason = 1;
            }
        }
    }

    // 算法信息在本周期真实完成触控事务、无长周期操作时按 100ms 节流刷新。
    // ★不再要求命令环排空★ 那是 v1 阻塞式 SPI 留下的"防嵌套"约束: 当年一条命令 = 发帧 + 等 + 再发帧,
    // 中途插一笔就会把别人的应答吃掉。LINK v2 里 core1 内的 _link.request 是单线程可重入的(配对靠
    // tag, 与时序无关), 本处又在命令消费循环之外, 不存在嵌套。
    // 而这个条件的代价是实测级的: 上位机在 settle 期间高频轮询 ALGO_GET_INFO ⇒ 命令环几乎不空 ⇒
    // 缓存长期得不到刷新, heap_used 冻在换代 commit 把 algo_heap_used_peak 清 0 的那一刻,
    // 于是 --algo-swap 随机报"heap_used=0"。v1 没暴露是因为它每次都另发一条 ALGO_GET_HEAP;
    // v2 把 heap 并进了 ALGO_CAPS 一帧, 刷新被卡住就一起卡住。
    // 容量(slot/heap 上限)仍是"首次读到即永久缓存", 但 heap_used 是动态量, 必须每轮更新。
    const uint32_t algo_now_ms = millis();
    if (touch_due && !heavy_busy() &&
        (uint32_t)(algo_now_ms - _algo_info_last_poll_ms) >= 100u) {
        _algo_info_last_poll_ms = algo_now_ms;
        if (_link_established && ok) {
            _refresh_algo_info_cache();
            // Algorithm cache is refreshed only; runtime parameter failures are reported to SensorLink.
        } else {
            _invalidate_algo_info_cache();
        }
    }

    // 链路存活指示(遥测未激活时): 仅在真实触控保活成功时 bump generation/valid。
    if (touch_due && ok && !_telem_active) {
        _snapshot.valid = true;
        uint16_t g = _snapshot.generation + 1;
        if (g == 0) g = 1;
        _snapshot.generation = g;
    }

    // 3a) 快照快路(独占单通道): 只读目标通道的 2-3 页, 每拍都能发布一份新代数。
    // ★为什么独占时必须换路★ 全通道分块泵每份要 16 个 1ms tick(63页/4页), 上限 62 份/s;
    // 而独占精调只关心这一个通道, 上位机根本不消费其余通道。只读它 ⇒ 单份约 0.5ms, 每拍一份,
    // 实测可达数百份/s, 精调曲线才能不丢细节。全通道流仍走 3b 的分块慢路(30Hz 足够)。
    // 任意异步操作期间均暂停快照搬运：启动 provisioning 的 APPLY 与运行时参数应用一样，
    // 都只能在 PSoC 的 NOT_BUSY 窗口消费 pending；只按单一异步状态门控会让启动 APPLY 仍被
    // snapshot page 流饿死，heavy 计数长期不归还，最终连扫描入口也一直 DEVICE_BUSY。
    // 触控快路和重操作自身的 GET_STATS 仍继续运行，用真实 busy 终态恢复快照。
    // Full snapshots move 63 PSoC pages, so keep a wide SPI-free window for
    // CapSense between broad-stream chunks.  The exclusive RP-only focus path
    // fetches just the selected channel (2-3 pages) and can safely use the
    // 3ms cadence required for >300Hz delivery without invoking FOCUS_SCAN.
    const uint32_t snapshot_now_ms = millis();
    // Snapshot paging itself is a bounded core1 state machine.  Throttling every
    // chunk to the host stream period multiplies 63 pages into ~3Hz; run one
    // bounded chunk per core1 cycle and let TxScheduler independently choose the
    // USB emission rate.  Focus BEGIN/read split then yields one sample per ~3ms.
    // ★Focus 必须按 PSoC 的扫描节奏取, 不能每拍都取★
    // 单通道快路一次调用要走完 BEGIN + 取 INFO + 主循环锁存窗 + 2~3 页, 约 0.8ms; 而 core1 周期
    // 只有 1ms ⇒ 每拍都做就等于把整个核占满, 命令环几乎没有消费窗口。后果是所有**阻塞读类**
    // 命令(ALGO_GET_TRACE / GET_PARAM / GET_CFG)严重迟滞: 上位机 5s 超时释放后设备响应才迟到,
    // 于是算法变量永远"暂无运行值"(日志里是成片的 ALGO_GET_TRACE 超时释放)。
    // 而 PSoC 每 scan_period_us(实测约 5.8ms)才产出一份新代数, 更快地取只是把同一份重复搬运。
    // 按扫描周期的一半取: 既不丢任何一份新代数(奈奎斯特留一倍余量), 又把 core1 占用降到约 1/3,
    // 命令环因此恢复正常消费。scan_period 尚未测出时退回 1ms(保持原行为, 不影响建链)。
    const uint32_t focus_active = (_focus_ch < psoc::SENSOR_CHANNEL_COUNT) ? 1u : 0u;
    const uint32_t scan_period_ms = _scan_period_us / 1000u;
    const uint32_t focus_interval_ms = (focus_active != 0u && scan_period_ms >= 2u)
        ? (scan_period_ms / 2u) : 1u;
    const uint32_t snapshot_interval_ms = focus_interval_ms;
    const bool snapshot_due = (uint32_t)(snapshot_now_ms - _snapshot_last_ms) >= snapshot_interval_ms;
    // Runtime parameter application has to receive an SPI-free PSoC main-loop window,
    // otherwise its pending command can never be consumed.  Snapshot paging resumes as
    // soon as the RP-owned apply lifecycle observes parameter readback and scan progress.
    const bool snapshot_allowed = _telem_active && ok && !heavy_busy() &&
                                  _runtime_apply.active == 0u && snapshot_due;
    if (snapshot_allowed) {
        _snapshot_last_ms = snapshot_now_ms;
    }
    if (snapshot_allowed && _focus_ch < psoc::SENSOR_CHANNEL_COUNT) {
        if (_link.snapshot_pump_channel(_focus_ch, &_snap_work)) {
            // ★算法运行值在这里顺带取回★
            // PSoC 没把 report[] 并进 252B 快照, 只能用 ALGO_GET_TRACE 逐项读。每份快照只读**一个**
            // 槽并轮转: 单份仅多一次约 0.2ms 的 SPI 事务, 4 份凑齐 4 个槽; 独占流上百帧/s 下每个槽
            // 仍有数十 Hz 刷新, 足够"当前值"显示, 又不会像每份读 4 项那样把 core1 占满。
            // 只有算法真的有效时才读; 无效时保持 0xFF 通道标记, 上位机据此显示"无上报变量"。
            if (_algo_info_valid != 0u) {
                uint8_t args[LNK_ARG_BYTES] = {0};
                args[0] = _focus_ch;
                uint8_t p[LNK_PAYLOAD_BYTES] = {0};
                // ★一帧取回全部 4 个 report 槽★(v1 一次只回一个槽, 只能逐份轮转)。
                // out_active 不再单独传: 它就是状态帧那份触控掩码的对应位, 再传一遍纯属冗余。
                if (_link.request(LNK_CMD_ALGO_TRACE, args, p)) {
                    for (size_t i = 0; i < psoc::ALGO_REPORT_SLOTS; ++i) {
                        _algo_trace_report[i] = lnk_rd16(&p[i * 2u]);
                    }
                    _algo_trace_active =
                        (((_link.touch_mask() >> _focus_ch) & 1u) != 0u) ? 1u : 0u;
                    _algo_trace_channel = _focus_ch;
                }
            } else {
                _algo_trace_channel = 0xFFu;
            }
            _snap_work.algo_channel = _algo_trace_channel;
            _snap_work.algo_active = _algo_trace_active;
            for (size_t i = 0; i < psoc::ALGO_REPORT_SLOTS; ++i) {
                _snap_work.algo_report[i] = _algo_trace_report[i];
            }
            _snap_seq++;              // 进入写临界区(奇)
            __dmb();
            _snapshot = _snap_work;
            __dmb();
            _snap_seq++;              // 离开写临界区(偶)
        }
    } else if (snapshot_allowed) {
    // 3b) 快照慢路: 遥测激活时分块流水读全通道 → 工作缓冲, 读满一份经 seqlock 发布到 _snapshot。
        if (_link.snapshot_pump(PSOC_SNAPSHOT_PAGES_PER_PUMP, &_snap_work)) {
            _snap_seq++;              // 进入写临界区(奇)
            __dmb();
            _snapshot = _snap_work;   // 结构体整体拷贝(含 channels 数组)
            __dmb();
            _snap_seq++;              // 离开写临界区(偶)
        }
    }

    // 4) 采样率统计: 每 ~250ms 读 PSoC scan_count 折算采样率/刷新周期。
    // 两个连续窗口即可在遥测启动后约 500ms 内建立真实速率，满足首帧/建链的 1s 目标；
    // GET_STATS 仍为低频探针，不与快照流水高频争抢 PSoC 空窗。
    const uint32_t stats_now_us = time_us_32();
    if (stats_now_us - _stats_last_us >= 250000u) {
        // A heavy-operation grace window prevents reset escalation, but it must
        // never suppress GET_STATS itself: telemetry readiness and Focus use the
        // fresh scan_count rate as their health contract.  Resetting primed on
        // every grace tick previously forced samples_per_sec to remain zero for
        // the entire 70s recovery allowance even while the PSoC was scanning.
        const bool stats_grace_active = (int32_t)(_reset_grace_until_ms - millis()) > 0;
        uint32_t sc = 0;
        uint8_t psoc_busy = 0;
        if (ok && _link_stats(&sc, &psoc_busy)) {
            if (_stats_primed) {
                const uint32_t dt = stats_now_us - _stats_last_us;
                const uint32_t dcount = sc - _stats_last_scan;
                if (dcount != 0u) {
                    const uint32_t rate = (uint32_t)(((uint64_t)dcount * 1000000ULL) / dt);
                    if (rate != 0u) {
                        _samples_per_sec = rate;
                        _scan_period_us = 1000000u / rate;
                    }
                }
                // 一个短统计窗读到相同 scan_count 只说明该窗未观察到推进：保留上一份
                // 已验证速率，健康检测仍使用 dcount/busy 的独立真值，不能把临时空窗伪装成
                // 扫描速率为零并让刚建立的遥测会话错误失败。
                // 失效兜底(主循环卡死): scan_count 长时间不推进 = PSoC 主循环卡死(疑似坏算法死循环)。
                // 注意状态帧仍由 PSoC SPI ISR 装出, 故 link_ok 不掉, 只能靠 scan_count 检测。
                // ★"忙"不等于"卡死": PSoC 报告 busy 时不得累计卡死计数★
                // APPLY/CALIBRATE/GLOBAL_COMMIT/AUTO_TUNE 由 PSoC 主循环同步执行, 期间 scan_count
                // 天然不推进(逐通道 CalibrateWidget 在 36 通道上可远超 4s, 尤其 provision 期 SPI
                // 中断负载很高时)。此前把它误判为"主循环卡死"→ XRES 复位 → 复位又触发 provision
                // 重发 APPLY → 永久复位循环(实测: 每轮 boot→APPLY→约 3s 后被复位, scan_count 恒 1,
                // raw 全 0, SelfHeal 累计上百次)。真正的卡死(坏算法死循环)不会置 busy, 仍能被抓到;
                // 重操作本身另有 _wait_op_done 的超时兜底, 不依赖这里。
                if (_link_established) {
                    if (stats_grace_active) {
                        _hang_intervals = 0;
                    } else if (dcount == 0u && psoc_busy == 0u) {
                        if (++_hang_intervals >= HANG_STATS_INTERVALS && _pub_reset_reason == 0) {
                            _pub_reset_reason = 2;
                        }
                    } else {
                        _hang_intervals = 0;
                    }
                }
            }
            _stats_last_scan = sc;
            _stats_last_us = stats_now_us;
            _stats_primed = true;
        }
    }
}

// 从未成功过(_pub_link_ok_ms == 0)一律为假: "还没建链"不能算活着。
bool Psoc::link_alive(uint32_t within_ms) const {
    if (_pub_link_ok) return true;
    const uint32_t last = _pub_link_ok_ms;
    if (last == 0u) return false;
    return (uint32_t)(millis() - last) <= within_ms;
}

// core1 入口: 置运行标志后进入"做完即下一轮 + 等 PSoC 通知"的循环, 永不返回。
// 注意: 调用方(core1_entry)必须已先调 multicore_lockout_victim_init(), 否则 flash 写入会死锁。
void Psoc::core1_run() {
    _core1_running = true;
    _int1_init();
    __dmb();
    while (true) {
        const uint32_t cycle_start = time_us_32();
        _spi_service();
        // 最小 SPI 空窗: 无论走通知还是自由跑都必须给 PSoC 留出来(它的 DMA ISR 会推迟 CapSense
        // 中间件的临界区)。忙(有快照分页/命令要跑)时这段几乎不等待。
        while ((time_us_32() - cycle_start) < CORE1_MIN_GAP_US) {
            tight_loop_contents();
        }
        // 之后要么立刻返回(还有活干 / 通知线不可信 / 已经有新代数), 要么在此等下一份新代数。
        _int1_wait_generation();
    }
}

// GPIO23 = PSoC P1.4 的新代数通知线。只读, 不装中断: core1 本来就是个独占循环, 在它的等待段里
// 直接比较电平即可, 比 GPIO IRQ + 清标志少一整套状态。
// ★不加内部上下拉★: 这条线经板上电平移位器中继(见 doc/hardware.txt), 自动方向型移位器明确
// 不允许挂比 50k 更强的上下拉, 而 RP2040 内部上下拉正好落在这个量级。悬空/PSoC 复位期间可能
// 读到乱翻转, 但那只会让 core1 提前醒 —— 等价于改造前的自由跑, 不产生错误行为。
void Psoc::_int1_init() {
    gpio_init(PIN_SENSOR_INT1);
    gpio_set_dir(PIN_SENSOR_INT1, GPIO_IN);
    gpio_disable_pulls(PIN_SENSOR_INT1);
    _int1_level = gpio_get(PIN_SENSOR_INT1);
}

// core1 本轮还有确定要干的活 ⇒ 绝不能停下来等通知。
// 这几项都是"等下去就会掉指标"的:
//   · 命令环非空: core0 正卡在 _submit 上等结果(上限 100ms), 等通知会把命令延迟抬到扫描周期;
//   · 三条异步生命周期: 它们靠周期性轮询推进(GET_STATS / 参数回读 / 上传确认);
//   · 全通道快照正在分页: 一份 63 页要 16 次调用, 中途停下等通知会把广谱流帧率压到 1/16;
//   · 链路还没建起来: 此时 PSoC 可能还没跑到会翻转 P1.4 的地方, 必须自由跑把链路先拉起来。
bool Psoc::_int1_free_run_needed() const {
    // I3 电平低时自由跑只会反复触发被 pump 拒绝的事务；此处返回 false 后仍由
    // _int1_wait_generation 的有限 budget 唤回，不能等 INT1 边沿，否则 INT2 抬高而未发布新代时会睡死。
    if (_link.tx_window_blocked()) return false;
    return _cmd_tail != _cmd_head ||
           _heavy_async.active != 0u ||
           _runtime_apply.active != 0u ||
           _algo_upload_async.active != 0u ||
           !_link_established ||
           _link.snapshot_pump_busy();
}

// 采样并(必要时)等待 PSoC 的"新代数已发布"翻转。
// 自由跑期间也照样采样计数 —— 否则永远攒不够 arm 门限, 通知机制无法自行启用。
void Psoc::_int1_wait_generation() {
    const bool level_now = gpio_get(PIN_SENSOR_INT1);
    if (level_now != _int1_level) {
        _int1_level = level_now;
        // ★这一句就是"事件驱动取数"的全部★ 翻转 = PSoC 已发布新一代(掩码与快照都已是新的)。
        // 预约一帧, 下一次 _spi_service 的 pump 把它取回来 —— 不再有任何固定节拍的轮询。
        _link.arm_probe();
        _pub_int1_edges = _pub_int1_edges + 1u;
        _int1_timeout_run = 0u;
        if (!_pub_int1_armed && _pub_int1_edges >= INT1_ARM_EDGES) _pub_int1_armed = true;
        return;                       // 已有新代数, 立刻去取, 不必等
    }
    if (!_pub_int1_armed || _int1_free_run_needed()) return;

    // INT2 是电平而非边沿事件：低电平时它可能在没有 INT1 翻转的情况下抬高，故等待必须受此上限约束，
    // 到期返回让下一轮重读 INT2，而不是把 core1 绑死在 INT1 边沿上。
    uint32_t budget = (_scan_period_us != 0u) ? (_scan_period_us * 2u + 1000u)
                                              : INT1_WAIT_FALLBACK_US;
    if (budget > INT1_WAIT_CAP_US) budget = INT1_WAIT_CAP_US;
    const uint32_t wait_start = time_us_32();
    while (true) {
        const bool level = gpio_get(PIN_SENSOR_INT1);
        if (level != _int1_level) {
            _int1_level = level;
            _link.arm_probe();
            _pub_int1_edges = _pub_int1_edges + 1u;
            _int1_timeout_run = 0u;
            return;
        }
        // 等待期间 core0 可能投进命令、也可能有异步生命周期被启动: 立刻回去干活, 不把它们压在
        // 一个扫描周期后面。等待段是 tight loop 且开中断, multicore lockout(flash 落盘)照常生效。
        if (_int1_free_run_needed()) return;
        if ((uint32_t)(time_us_32() - wait_start) >= budget) {
            _pub_int1_timeouts = _pub_int1_timeouts + 1u;
            if (++_int1_timeout_run >= INT1_DISARM_TIMEOUTS) {
                _pub_int1_armed = false;   // 通知线不可信 → 退回自由跑; 再见到翻转会自动重新 arm
                _int1_timeout_run = 0u;
            }
            return;
        }
        tight_loop_contents();
    }
}

// 兼容入口: 仅 setup 阶段(core1 未启动)由 core0 直调一次, 取首帧触控/快照供 DEVICE_INFO。
void Psoc::update() {
    _spi_service();
}

// 实际 SPI 指令执行体: core1 处理信箱时调用, 或 setup 阶段(_core1_running=false)由 _submit 直调。
bool Psoc::_exec_cmd(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out, const uint8_t* data) {
    // ★派发重操作前先开宽限窗★: 这些操作让 PSoC 主循环忙数秒到数十秒, 期间链路必然抖动。
    // 不开窗就会被失效兜底判成"PSoC 掉线"→ XRES + 清 provisioned → 重新下发又触发一次重操作,
    // 形成永久 provision 风暴。宽限只影响"是否判定 PSoC 已死", 不影响任何真实完成判据。
    switch (op) {
        case SpiOp::APPLY:
        case SpiOp::CALIBRATE:
        case SpiOp::BASELINE_RESET:
        case SpiOp::GLOBAL_COMMIT:
        case SpiOp::MEASURE_CP:
            _reset_grace_until_ms = millis() + HEAVY_OP_GRACE_MS;
            break;
        case SpiOp::AUTO_TUNE:
            _reset_grace_until_ms = millis() + AUTOTUNE_GRACE_MS;
            break;
        default:
            break;
    }

    // ★每个分支都是"一笔事务 + 按 tag 认领"★ 参数/载荷布局逐条对照 psoc_link_abi.h 的注释,
    // 不在这里出现任何裸命令码或字节偏移。回显校验只用来挡"从机拒收/参数被夹紧", 不再兼作
    // "这份响应是不是我的"——后者已由 tag 保证。
    uint8_t args[LNK_ARG_BYTES] = {0};
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};
    switch (op) {
        case SpiOp::SET_PARAM:
            args[0] = ch; args[1] = pid; lnk_wr32(&args[2], val);
            if (!_link.request(LNK_CMD_PARAM_SET, args, p)) return false;
            return p[0] == ch && p[1] == pid;
        case SpiOp::GET_PARAM: {
            uint32_t v = 0;
            const bool ok = _link_param_get(ch, pid, &v);
            if (out) *out = v;
            return ok;
        }
        case SpiOp::GET_RAW:
            args[0] = ch;
            if (!_link.request(LNK_CMD_RAW_GET, args, p) || p[0] != ch) return false;
            if (out) *out = lnk_rd16(&p[1]);
            return true;
        case SpiOp::SET_MODE:
            // mode 复用 ch 字段。从机可能把非法档位夹紧后回显实际生效值, 故不比值(与 v1 一致)。
            args[0] = ch;
            return _link.request(LNK_CMD_MODE_SET, args, p);
        case SpiOp::APPLY:
            // 只确认已受理; 完成由 _poll_heavy_async 观察 st.OP_BUSY 归还, 不在 core1 内干等。
            return _link.request(LNK_CMD_APPLY, args, p);
        case SpiOp::RUNTIME_PARAM_APPLY:
            // 不校准/不复位基线的快速应用。accepted=p[3]; 完成由 RP 侧以参数回读 + scan 推进独立判定。
            args[0] = ch; args[1] = pid; args[2] = (uint8_t)val;
            if (!_link.request(LNK_CMD_QUICK_APPLY, args, p)) return false;
            return p[0] == ch && p[3] != 0u;
        case SpiOp::CALIBRATE:
            // ch 复用: 0..35=单通道 / 0xFF=全通道。单通道用时约为全通道的 1/36, 故超时分档给。
            args[0] = ch;
            if (!_link.request(LNK_CMD_CALIBRATE, args, p) || p[0] != ch) return false;
            return _wait_op_done((ch < psoc::SENSOR_CHANNEL_COUNT) ? 30000u : 60000u);
        case SpiOp::BASELINE_RESET:
            // 基线初始化本身很快; 5s 允许最慢扫描收尾, 避免把"仍在收尾"误报成恢复失败。
            args[0] = ch;
            if (!_link.request(LNK_CMD_BASELINE_RESET, args, p) || p[0] != ch) return false;
            return _wait_op_done(5000u);
        case SpiOp::MEASURE_CP:
            // BIST 是一次完整 CSD 模式切换: 必须等固件恢复正常扫描并让 OP_BUSY 落下才算完成。
            if (!_link.request(LNK_CMD_MEASURE_CP, args, p)) return false;
            return _wait_op_done(10000u);
        case SpiOp::GET_CP:
            args[0] = ch;
            if (!_link.request(LNK_CMD_CP_GET, args, p) || p[0] != ch) return false;
            if (out) *out = lnk_rd32(&p[1]);   // 测量中=0, 成功=fF, 失败/未测量=0xFFFFFF
            return true;
        case SpiOp::UPLOAD_ALGO: {
            // 上传由 core1 每拍推进一笔 SPI 事务；这里仅启动状态机，不等待 PSoC commit。
            const uint16_t len = (uint16_t)(val & 0xFFFFu);
            const uint16_t crc = (uint16_t)((val >> 16) & 0xFFFFu);
            const bool ok = _link.begin_upload_algo(data, len, crc);
            if (ok) {
                _algo_upload_async.active = 1u;
                _algo_upload_async.started_ms = millis();
                _algo_upload_async.last_poll_ms = _algo_upload_async.started_ms - 1u;
                // 与首个分页事务错开约 10ms，随后上传期间仅按 20ms 保活触控。
                _algo_upload_async.last_touch_ms = _algo_upload_async.started_ms - 10u;
                _algo_upload_async.response_fail_run = 0u;
                // ★宽限窗必须跟着上传上限一起收★ 宽限期内 core1 一律不判"PSoC 挂死", 原来给 130s
                // 意味着刚装上的坏算法把 PSoC 跑死之后, 要等两分多钟才会被发现 —— 那期间 note_fatal
                // 不会累计、隔离也就永远不触发, 救援机制形同虚设。上限 30s + 10s 余量覆盖
                // LNK_ALGO_SLOT_SIZE / 10B 的 410 页分页(可并发在途)与 commit。
                _reset_grace_until_ms = _algo_upload_async.started_ms +
                                        (PsocLink::ALGO_UPLOAD_TIMEOUT_MS + 10000u);
            }
            return ok;
        }
        case SpiOp::GET_ALGO_INFO:
            if (!_link.request(LNK_CMD_ALGO_INFO, args, p)) return false;
            if (out) *out = ((uint32_t)(p[0] != 0u ? 1u : 0u) << 16) | lnk_rd16(&p[2]);
            return true;
        case SpiOp::SET_ALGO_ROM:
            args[0] = ch; lnk_wr16(&args[2], (uint16_t)val);
            if (!_link.request(LNK_CMD_ALGO_ROM_SET, args, p)) return false;
            return p[0] == ch;
        case SpiOp::GET_ALGO_ROM:
            args[0] = ch;
            if (!_link.request(LNK_CMD_ALGO_ROM_GET, args, p) || p[0] != ch) return false;
            if (out) *out = lnk_rd16(&p[2]);
            return true;
        case SpiOp::ALGO_SET_CFG:
            args[0] = ch; args[1] = (uint8_t)val;   // ch 字段复用为 cfg idx
            if (!_link.request(LNK_CMD_ALGO_CFG_SET, args, p)) return false;
            return p[0] == ch;
        case SpiOp::ALGO_GET_CFG:
            args[0] = ch;                            // ch 字段复用为 cfg idx
            if (!_link.request(LNK_CMD_ALGO_CFG_GET, args, p) || p[0] != ch) return false;
            if (out) *out = p[1];
            return true;
        // 逐通道 cfg_ch: ch 字段=通道, pid 字段=idx, val 字段=值(沿用既有复用手法, 不给信箱
        // 新增字段 —— 加字段会让 SpiCmd 变大 ×64 环深, 白吃 RAM)。
        case SpiOp::ALGO_SET_CFG_CH:
            args[0] = ch; args[1] = pid; args[2] = (uint8_t)val;
            if (!_link.request(LNK_CMD_ALGO_CFGCH_SET, args, p)) return false;
            return p[0] == ch && p[1] == pid;
        case SpiOp::ALGO_GET_CFG_CH:
            args[0] = ch; args[1] = pid;
            if (!_link.request(LNK_CMD_ALGO_CFGCH_GET, args, p) ||
                p[0] != ch || p[1] != pid) return false;
            if (out) *out = p[2];
            return true;
        case SpiOp::ALGO_GET_HEAP:
            // v2 把堆容量/峰值并入 CAPS 一帧(v1 是单独的 0x4C)。
            if (!_link.request(LNK_CMD_ALGO_CAPS, args, p)) return false;
            if (out) *out = ((uint32_t)lnk_rd16(&p[4]) << 16) | lnk_rd16(&p[2]);   // used<<16 | size
            return true;
        case SpiOp::ALGO_GET_CRC:
            // v2 把槽内内容 CRC 并入 INFO 一帧(v1 是单独的 0x4D), 于是 valid/len/crc 天然同批一致。
            if (!_link.request(LNK_CMD_ALGO_INFO, args, p)) return false;
            if (out) *out = ((uint32_t)(p[0] != 0u ? 1u : 0u) << 16) | lnk_rd16(&p[4]);
            return true;
        case SpiOp::ALGO_GET_CAPS:
            if (!_link.request(LNK_CMD_ALGO_CAPS, args, p)) return false;
            if (out) *out = ((uint32_t)lnk_rd16(&p[2]) << 16) | lnk_rd16(p);   // heap<<16 | slot
            return true;
        case SpiOp::SET_GLOBAL:
            args[0] = ch; lnk_wr32(&args[2], val);   // ch 字段复用为 gparam_id
            if (!_link.request(LNK_CMD_GLOBAL_SET, args, p)) return false;
            return p[0] == ch;
        case SpiOp::GET_GLOBAL:
            args[0] = ch;
            if (!_link.request(LNK_CMD_GLOBAL_GET, args, p) || p[0] != ch) return false;
            if (out) *out = lnk_rd32(&p[2]);
            return true;
        case SpiOp::GLOBAL_COMMIT:
            // 只在从机 ISR 置 pending; 必须等主循环跑完 Init/Initialize 才允许后续 PARAM_SET 排上去,
            // 否则重初始化会覆盖已排队的逐通道值。命令环是 FIFO 且由 core1 顺序执行, 屏障放在这里。
            if (!_link.request(LNK_CMD_GLOBAL_COMMIT, args, p)) return false;
            return _wait_op_done(800u);
        case SpiOp::AUTO_TUNE: {
            // ch 字段复用为目标通道(0xFF=全通道); pid 字段复用为灵敏度档位 pref(1..7);
            // val 字段复用为 host 侧进度标签(★不下到线上★, 见 psoc_types.h 的说明)。
            // ★仍在 core1 内一次跑完★: 拆成跨周期状态机的话, _spi_service 的 scan_count 卡死兜底
            // 会在从机长校准期间误判并对其硬复位。
            const uint8_t tag = (uint8_t)(val & psoc::AUTOTUNE_TAG_MASK);
            _at_work.clear();
            _at_work.req = _at_req;
            _at_work.state = 1;
            _at_work.ch = ch;
            _at_work.tag = tag;
            _publish_autotune();
            args[0] = ch; args[1] = pid;
            bool ok = _link.request(LNK_CMD_AUTO_TUNE, args, p) && p[0] == ch;
            uint8_t result = 0; uint16_t div = 0;
            // 单通道三步算法约 300-650ms; 全通道逐通道各自校准 36 × 单通道 ≈ 11-23s(最坏更长) →
            // 窗口 45s, 且必须小于上位机卡死阈值(csd_diag_tick 3200 tick ≈ 51s)。
            if (ok) ok = _wait_op_done(45000u, true);
            if (ok && _link.request(LNK_CMD_AUTO_TUNE_GET, nullptr, p)) {
                result = p[0];
                div = lnk_rd16(&p[2]);
            } else {
                ok = false;
            }
            _at_work.state = 2;
            _at_work.phase = 4;
            _at_work.step = 0;
            // 超时/链路失败按"失败"上报, 避免上位机永远等不到终态。
            _at_work.result = (ok && result == 1u) ? 1u : 2u;
            _at_work.div = (_at_work.result == 1u) ? div : 0u;
            _at_work.cur_div = _at_work.div;
            _publish_autotune();
            if (out) *out = (uint32_t)result | ((uint32_t)div << 8);
            return ok;
        }
        default:
            return false;
    }
}

// core0 侧: 入队一条 SPI 指令。写类(out==null)异步立即返回; 读类阻塞等本条结果。
// core1 未接管(setup 阶段)时本核直执行, 避免死等无人消费的队列。
bool Psoc::_submit(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out, const uint8_t* data,
                   uint32_t timeout_us, AsyncOwner async_owner, uint8_t async_token) {
    if (!_spi_ready) return false;
    if (!_core1_running) {
        return _exec_cmd(op, ch, pid, val, out, data);
    }

    const bool wait = (out != nullptr);   // 读类需返回值
    // ★入队自旋也要用调用方给的预算★
    // 原先这里硬编码 SPI_CMD_TIMEOUT_US(100ms), 但 core1 执行重操作时(APPLY 实测 12s、CALIBRATE、
    // GLOBAL_COMMIT)整段时间不消费命令环, 环一满 100ms 就放弃入队 —— 于是"刚保存完配置就点算法上传"
    // 必然失败, 上位机看到的是 `algo download enqueue failed`(实测复现)。调用方明知自己是长操作时
    // 给更长 timeout_us, 这里就该照办; 自旋期间已经在泵 USB + 喂狗, 等下去是安全的。
    const uint32_t eff_timeout = (timeout_us != 0u) ? timeout_us : SPI_CMD_TIMEOUT_US;

    // 等待队列有空位(仅当被 core1 拖慢时短暂自旋; 长期满=core1 卡死则超时放弃)。
    const uint32_t enq_start = time_us_32();
    while (((_cmd_head + 1u) % CMD_RING_SIZE) == _cmd_tail) {
        if (time_us_32() - enq_start > eff_timeout) return false;
        // ★保活 USB★: core1 执行重操作(如 CALIBRATE _wait_op_done ~1.5s)期间不消费命令环,
        // 高频连发(如探针单轮 38 条 > 环深 32)会让 core0 卡在此入队自旋。与下方"等结果"环一致,
        // 必须在此泵 tud_task 否则 TinyUSB 得不到服务 → 主机写超时拆端点掉线。
        crash_stage_set(CRASH_STAGE_PSOC_ENQUEUE);
        watchdog_update();
        HAL_USB_Device::getInstance()->task();
        tight_loop_contents();
    }
    crash_stage_set(CRASH_STAGE_NONE);

    const uint32_t slot = _cmd_head;
    SpiCmd& c = _cmd_ring[slot];
    c.op = op;
    c.ch = ch;
    c.pid = pid;
    c.val = val;
    c.data = data;
    c.result = 0;
    c.ok = false;
    c.done = false;
    c.async_owner = async_owner;
    c.async_token = async_token;
    // ★必须在推进 head 之前置★: 否则 core1 可能先执行完并递增 _heavy_done, 之后 core0 再递增
    // _heavy_enq, heavy_busy() 就会在指令早已完成后仍报忙(且永不复位)。
    if (_op_is_heavy(op)) _heavy_enq = _heavy_enq + 1u;
    __dmb();
    _cmd_head = (slot + 1u) % CMD_RING_SIZE;   // 发布: 推进 head, core1 下周期可见
    __sev();

    if (!wait) return true;   // 写类: 异步, 立即返回

    // 读类/需等真实完成类: 单生产者, 入队后本条槽位在读到 done 前不会被复用, 阻塞等结果。
    // 重操作(校准/apply/基线复位)在 core1 内轮询 PSoC busy 至真实完成, 耗时可达 ~1.5s,
    // 故允许调用方给更长 timeout_us(默认 SPI_CMD_TIMEOUT_US); eff_timeout 已在入队前算好。
    const uint32_t start = time_us_32();
    while (!c.done) {
        if (time_us_32() - start > eff_timeout) return false;   // 超时
        crash_stage_set(CRASH_STAGE_PSOC_SUBMIT);
        watchdog_update();   // 重操作阻塞等待可达~1.5s, 期间喂狗防 5s 看门狗误复位
        HAL_USB_Device::getInstance()->task();
        tight_loop_contents();
    }
    crash_stage_set(CRASH_STAGE_NONE);
    __dmb();
    if (out) *out = c.result;
    return c.ok;
}

void Psoc::_invalidate_algo_info_cache() {
    _algo_info_seq++;
    __dmb();
    _algo_info_available = 0u;
    _algo_info_valid = 0u;
    _algo_info_len = 0u;
    _algo_info_crc = 0u;
    _algo_info_uploading = 0u;
    _algo_heap_used = 0u;
    _algo_heap_size = 0u;
    _algo_info_refresh_ms = 0u;
    _algo_info_cache_epoch = _algo_info_epoch;
    __dmb();
    _algo_info_seq++;
}

bool Psoc::_refresh_algo_info_cache() {
    // ★一帧拿齐 valid/uploading/len/slot_crc★ v1 要 INFO + GET_CRC 两条命令, 于是 crc 与 len
    // 天然可能来自不同时刻(commit 正好插在中间就会对不上)。v2 的 INFO 载荷把四项放在同一帧里。
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};
    if (!_link.request(LNK_CMD_ALGO_INFO, nullptr, p)) {
        _invalidate_algo_info_cache();
        return false;
    }
    const bool valid = p[0] != 0u;
    const bool uploading = p[1] != 0u;
    const uint16_t len = lnk_rd16(&p[2]);
    const uint16_t crc = valid ? lnk_rd16(&p[4]) : 0u;
    // ★追加读取, 失败不作废★ 容量/堆读不到时一律降级为 0 而不是把整份缓存判无效 ——
    // 否则一次偶发失败会让上位机连"算法有没有装上"都读不到。
    uint16_t heap_size = 0u;
    uint16_t heap_used = 0u;
    uint16_t slot_cap = 0u;
    uint16_t heap_cap = 0u;
    bool caps_read = false;
    uint8_t c[LNK_PAYLOAD_BYTES] = {0};
    // 槽/堆容量是 PSoC 编译期常量，首次成功读到后永久复用；PSoC XRES 不会改变同一固件容量。
    // heap_used(峰值)不是常量, 故只要缓存过容量就仍每轮取一次(同一帧顺带回来, 不多花事务)。
    if (_link.request(LNK_CMD_ALGO_CAPS, nullptr, c)) {
        slot_cap = lnk_rd16(c);
        heap_cap = lnk_rd16(&c[2]);
        heap_size = heap_cap;
        heap_used = lnk_rd16(&c[4]);
    }
    if (_algo_slot_cap == 0u && slot_cap != 0u) {
        caps_read = true;
        if (slot_cap != PSOC_ALGO_MAX_LEN && !_algo_caps_mismatch_reported) {
            // 三层任一容量常量漏改都会"上传成功却跑旧代码"；主动推事件使漂移当场可见。
            SelfHeal::getInstance()->note(SH_ALGO_CAPACITY_MISMATCH,
                                          ((uint32_t)PSOC_ALGO_MAX_LEN << 16) | slot_cap);
            _algo_caps_mismatch_reported = true;
        }
    }
    _publish_algo_info_cache(valid, len, crc, uploading, heap_used, heap_size,
                             caps_read, slot_cap, heap_cap);
    return true;
}

void Psoc::_publish_algo_info_cache(bool valid, uint16_t len, uint16_t crc, bool uploading,
                                    uint16_t heap_used, uint16_t heap_size, bool caps_read,
                                    uint16_t slot_cap, uint16_t heap_cap) {
    _algo_info_seq++;
    __dmb();
    _algo_info_valid = valid ? 1u : 0u;
    _algo_info_len = len;
    _algo_info_crc = crc;
    _algo_info_uploading = uploading ? 1u : 0u;
    _algo_heap_used = heap_used;
    _algo_heap_size = heap_size;
    if (caps_read) {
        _algo_slot_cap = slot_cap;
        _algo_heap_cap = heap_cap;
    }
    _algo_info_refresh_ms = millis();
    _algo_info_cache_epoch = _algo_info_epoch;
    _algo_info_available = 1u;
    __dmb();
    _algo_info_seq++;
}

bool Psoc::get_algo_info_cached(bool* out_valid, uint16_t* out_len, uint16_t* out_crc,
                                uint16_t* out_heap_used, uint16_t* out_heap_size,
                                bool* out_uploading) const {
    if (out_valid) *out_valid = false;
    if (out_len) *out_len = 0u;
    if (out_crc) *out_crc = 0u;
    if (out_heap_used) *out_heap_used = 0u;
    if (out_heap_size) *out_heap_size = 0u;
    if (out_uploading) *out_uploading = false;
    uint32_t s1;
    uint32_t s2;
    uint8_t available;
    uint8_t valid;
    uint16_t len;
    uint16_t crc;
    uint8_t uploading;
    uint16_t heap_used;
    uint16_t heap_size;
    uint32_t refreshed;
    uint32_t epoch;
    do {
        s1 = _algo_info_seq;
        __dmb();
        available = _algo_info_available;
        valid = _algo_info_valid;
        len = _algo_info_len;
        crc = _algo_info_crc;
        uploading = _algo_info_uploading;
        heap_used = _algo_heap_used;
        heap_size = _algo_heap_size;
        refreshed = _algo_info_refresh_ms;
        epoch = _algo_info_cache_epoch;
        __dmb();
        s2 = _algo_info_seq;
    } while ((s1 & 1u) || s1 != s2);
    if (available == 0u || epoch != _algo_info_epoch || refreshed == 0u ||
        (uint32_t)(millis() - refreshed) > ALGO_INFO_CACHE_MAX_AGE_MS) {
        return false;
    }
    if (out_valid) *out_valid = valid != 0u;
    if (out_len) *out_len = len;
    if (out_crc) *out_crc = crc;
    if (out_uploading) *out_uploading = uploading != 0u;
    if (out_heap_used) *out_heap_used = heap_used;
    if (out_heap_size) *out_heap_size = heap_size;
    return true;
}

bool Psoc::get_algo_caps_cached(uint16_t* out_slot, uint16_t* out_heap) const {
    if (out_slot) *out_slot = 0u;
    if (out_heap) *out_heap = 0u;
    uint32_t s1;
    uint32_t s2;
    uint16_t slot;
    uint16_t heap;
    do {
        s1 = _algo_info_seq;
        __dmb();
        slot = _algo_slot_cap;
        heap = _algo_heap_cap;
        __dmb();
        s2 = _algo_info_seq;
    } while ((s1 & 1u) || s1 != s2);
    if (slot == 0u) return false;
    if (out_slot) *out_slot = slot;
    if (out_heap) *out_heap = heap;
    return true;
}

// ---------- seqlock 读访问器(core0 侧) ----------
uint64_t Psoc::touch_mask() const {
    uint32_t s1, s2;
    uint64_t m;
    do {
        s1 = _pub_seq;
        __dmb();
        m = _pub_touch_mask;
        __dmb();
        s2 = _pub_seq;
    } while ((s1 & 1u) || (s1 != s2));   // 写入中或期间被改写则重读
    return m;
}

uint32_t Psoc::touch_sample_us() const {
    uint32_t s1, s2, t;
    do {
        s1 = _pub_seq;
        __dmb();
        t = _pub_touch_sample_us;
        __dmb();
        s2 = _pub_seq;
    } while ((s1 & 1u) || (s1 != s2));
    return t;
}

const psoc::SensorSnapshot& Psoc::snapshot() const {
    uint32_t s1, s2;
    do {
        s1 = _snap_seq;
        __dmb();
        _snapshot_ro = _snapshot;   // 结构体整体拷贝到 core0 私有副本
        __dmb();
        s2 = _snap_seq;
    } while ((s1 & 1u) || (s1 != s2));
    return _snapshot_ro;
}

// ---------- CSD 运行时指令(core0→信箱→core1) ----------
bool Psoc::set_param(uint8_t ch, uint8_t param_id, uint32_t value) {
    return _submit(SpiOp::SET_PARAM, ch, param_id, value, nullptr);
}
bool Psoc::get_param(uint8_t ch, uint8_t param_id, uint32_t* out) {
    return _submit(SpiOp::GET_PARAM, ch, param_id, 0, out);
}
bool Psoc::get_raw(uint8_t ch, uint16_t* out) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::GET_RAW, ch, 0, 0, &r);
    if (out) *out = (uint16_t)r;
    return ok;
}
// ★core0 非阻塞(修 USB 掉线)★: 保存流里会连发 参数×N + CALIBRATE, 若 core0 阻塞等真实完成
// (~1.5s)会饿死 USB 服务 → 主机写超时 → "Missing config bulk OUT endpoint" 掉线。故这三类改为
// 入队即返回(ACK 表示"已受理"); 重活仍由 core1 在 _exec_cmd 内 _wait_op_done 完成(只占用 core1,
// 期间 touch 暂停但 core0/USB 正常)。完成后 raw 经遥测自然刷新, UI 无需阻塞等待。
bool Psoc::apply_params() {
    return _submit(SpiOp::APPLY, 0, 0, 0, nullptr);
}
// 开机校准流水线专用的非阻塞入队(见 psoc.h 处说明)。out=nullptr ⇒ _submit 走写类分支立即返回;
// 入队时 _heavy_enq 已递增, 故调用方在下一轮就能看到 heavy_busy()==true, 直到 core1 真正执行完。
// 入队自旋预算给 100ms 默认值即可: 本流水线只在 heavy_busy()==false 时才发起, 环必然有空位。
bool Psoc::start_boot_calibrate(uint8_t ch) {
    return _submit(SpiOp::CALIBRATE, ch, 0, 0, nullptr);
}
bool Psoc::start_boot_baseline_reset(uint8_t ch) {
    return _submit(SpiOp::BASELINE_RESET, ch, 0, 0, nullptr);
}

bool Psoc::_start_runtime_param_apply(uint8_t ch, uint8_t gain, uint8_t div) {
    if (_runtime_apply.active != 0u || _runtime_apply.pending != 0u ||
        _runtime_apply.complete != 0u || heavy_busy()) return false;
    uint8_t token = static_cast<uint8_t>(_runtime_apply.token + 1u);
    if (token == 0u) token = 1u;
    _runtime_apply.token = token;
    _runtime_apply.ok = 0u;
    _runtime_apply.complete = 0u;
    _runtime_apply.response_fail_run = 0u;
    _runtime_apply.params_confirmed = 0u;
    _runtime_apply.scan_seen = 0u;
    _runtime_apply.scan_progress_polls = 0u;
    _runtime_apply.target_ch = ch;
    _runtime_apply.target_gain = gain;
    _runtime_apply.target_div = div;
    _runtime_apply.started_ms = 0u;
    _runtime_apply.last_param_poll_ms = 0u;
    _runtime_apply.last_poll_ms = 0u;
    _runtime_apply.last_touch_ms = 0u;
    _runtime_apply.last_scan_count = 0u;
    _runtime_apply.pending = 1u;
    __dmb();
    if (_submit(SpiOp::RUNTIME_PARAM_APPLY, ch, gain, div, nullptr, nullptr, 0u,
                AsyncOwner::RUNTIME_PARAM, token)) return true;
    _runtime_apply.pending = 0u;
    return false;
}

bool Psoc::_start_host_write(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val) {
    // Host delayed ACK has a bounded ownership window.  Do not place it behind
    // provisioning or a heavy PSoC operation: UsbComm deliberately holds RX
    // until this terminal is sent, so queueing here would make every following
    // host command appear to disappear.  A caller receives DEVICE_BUSY and can
    // retry after the competing operation reaches a real terminal state.
    if (!host_write_ready()) return false;
    uint8_t token = static_cast<uint8_t>(_host_write.token + 1u);
    if (token == 0u) token = 1u;
    _host_write.token = token;
    _host_write.ok = 0u;
    _host_write.complete = 0u;
    _host_write.pending = 1u;
    __dmb();
    if (_submit(op, ch, pid, val, nullptr, nullptr, 0u, AsyncOwner::HOST_WRITE, token)) return true;
    _host_write.pending = 0u;
    return false;
}

bool Psoc::take_host_write_result(bool* out_ok) {
    if (_host_write.complete == 0u) return false;
    __dmb();
    if (out_ok) *out_ok = _host_write.ok != 0u;
    _host_write.complete = 0u;
    return true;
}

bool Psoc::start_host_param_set(uint8_t ch, uint8_t param_id, uint32_t value) {
    return _start_host_write(SpiOp::SET_PARAM, ch, param_id, value);
}

bool Psoc::start_host_mode_set(uint8_t mode) {
    return _start_host_write(SpiOp::SET_MODE, mode);
}

bool Psoc::start_host_global_set(uint8_t gparam_id, uint32_t value) {
    return _start_host_write(SpiOp::SET_GLOBAL, gparam_id, 0u, value);
}

bool Psoc::start_host_global_commit() {
    return _start_host_write(SpiOp::GLOBAL_COMMIT, 0u);
}

bool Psoc::start_host_algo_set_cfg(uint8_t idx, uint8_t val) {
    return _start_host_write(SpiOp::ALGO_SET_CFG, idx, 0u, val);
}

bool Psoc::start_host_algo_set_cfg_ch(uint8_t ch, uint8_t idx, uint8_t val) {
    // 与 start_host_algo_set_cfg 逐字对照: ch 走 ch 字段、idx 走 pid 字段、值走 val 字段。
    return _start_host_write(SpiOp::ALGO_SET_CFG_CH, ch, idx, val);
}

bool Psoc::start_host_algo_set_rom(uint8_t ch, uint16_t rom) {
    return _start_host_write(SpiOp::SET_ALGO_ROM, ch, 0u, rom);
}

bool Psoc::start_host_calibrate(uint8_t ch) {
    return _start_host_write(SpiOp::CALIBRATE, ch);
}

bool Psoc::start_host_baseline_reset(uint8_t ch) {
    return _start_host_write(SpiOp::BASELINE_RESET, ch);
}

bool Psoc::start_host_measure_cp() {
    return _start_host_write(SpiOp::MEASURE_CP, 0u);
}

bool Psoc::start_runtime_param_apply(uint8_t ch, uint8_t gain, uint8_t div) {
    return _start_runtime_param_apply(ch, gain, div);
}

bool Psoc::take_runtime_param_apply_result(bool* out_ok) {
    if (_runtime_apply.complete == 0u) return false;
    __dmb();
    if (out_ok) *out_ok = _runtime_apply.ok != 0u;
    _runtime_apply.complete = 0u;
    return true;
}

// ★异步启动★: 入队即返回(同 calibrate/baseline_reset 的写类语义), core0 不再为 20-25s 长自适应干等。
// 先自增 _at_req 再入队: core1 取到本条命令时回显该代号, core0 据此区分本轮进度与上一轮残留结果。
bool Psoc::auto_tune_start(uint8_t ch, uint8_t pref, uint8_t host_seq) {
    _at_req++;
    __dmb();
    // host_seq = 上位机 AUTO_TUNE 请求帧的 seq: 折成 6 bit 标签随命令下到 PSoC 并由其回显,
    // 于是"这一轮的结果"在 RP↔PSoC 这一段也有归属键, 陈旧结果不会冒充本轮成功。
    return _submit(SpiOp::AUTO_TUNE, ch, pref, psoc::autotune_tag_of(host_seq), nullptr);
}

// core1: 把工作副本发布为一致快照(多字段防撕裂, 同 _snapshot 的 seqlock 模式)。
void Psoc::_publish_autotune() {
    _at_seq++;            // 进入写临界区(奇)
    __dmb();
    _at_pub = _at_work;
    __dmb();
    _at_seq++;            // 离开写临界区(偶)
}

psoc::AutoTuneProgress Psoc::autotune_status() const {
    uint32_t s1, s2;
    do {
        s1 = _at_seq;
        __dmb();
        _at_ro = _at_pub;
        __dmb();
        s2 = _at_seq;
    } while ((s1 & 1u) || (s1 != s2));
    return _at_ro;
}
bool Psoc::set_mode(uint8_t mode) {
    return _submit(SpiOp::SET_MODE, mode, 0, 0, nullptr);
}
bool Psoc::get_cp(uint8_t ch, uint32_t* out) {
    return _submit(SpiOp::GET_CP, ch, 0, 0, out);          // 读类, 阻塞等结果
}

bool Psoc::upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16) {
    // 上限取 store 的同一个常量(= PSoC 的 ALGO_SLOT_SIZE)。原先这里写死 1024, 槽扩到 4096 后
    // 会把合法的大算法在门口就拒掉, 且错误信息与真实原因完全无关。
    if (data == nullptr || len == 0 || len > PSOC_ALGO_MAX_LEN) return false;
    // 上一次下发未完成时拒绝: blob 缓冲被 core1 持有, 此刻改写会让它下发出半新半旧的代码。
    if (_algo_dl.busy != 0u) return false;
    _algo_dl.busy = 1u;
    __dmb();
    // val 打包 crc16<<16 | len; 写类入队(out 为空)即返回, 重活全在 core1, core0 继续服务 USB。
    const uint32_t packed = ((uint32_t)crc16 << 16) | (uint32_t)len;
    // ★入队预算给足 15s★: 算法上传是用户显式动作, 且常紧跟在"保存到设备"之后 —— 那时 core1 可能正在
    // 跑 GLOBAL_COMMIT/APPLY(逐通道重校准实测 12s), 整段不消费命令环。默认 100ms 会让上传必然失败,
    // 表现为 `algo download enqueue failed`。等下去是安全的: 自旋内已泵 USB + 喂狗。
    if (!_submit(SpiOp::UPLOAD_ALGO, 0, 0, packed, nullptr, data, 15000000u)) {
        _algo_dl.busy = 0u;   // 入队都没成功: 立刻释放, 否则永久锁死上传通道
        return false;
    }
    return true;
}

bool Psoc::algo_download_take_failure() {
    if (_algo_dl.failed == 0u) return false;
    _algo_dl.failed = 0u;
    return true;
}

bool Psoc::get_algo_info(bool* out_valid, uint16_t* out_len) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::GET_ALGO_INFO, 0, 0, 0, &r);
    if (ok) {
        if (out_valid) *out_valid = ((r >> 16) & 1u) != 0u;
        if (out_len) *out_len = (uint16_t)(r & 0xFFFFu);
    }
    return ok;
}

bool Psoc::set_algo_rom(uint8_t ch, uint16_t rom) {
    // ★写类异步★: 一次算法下发要推 36 条 ROM, 逐条阻塞等 core1 回显会把 core0 按在 handler 里
    // 数百 ms(且 core1 正忙于上一条 UPLOAD_ALGO 时每条都会撞上 100ms 超时假失败)。
    // 入队即返回, core1 按 ring 顺序执行; 真值由 ALGO_GET_ROM 回读对账。
    return _submit(SpiOp::SET_ALGO_ROM, ch, 0, rom, nullptr);
}

bool Psoc::get_algo_rom(uint8_t ch, uint16_t* out_rom) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::GET_ALGO_ROM, ch, 0, 0, &r);
    if (ok && out_rom) *out_rom = (uint16_t)(r & 0xFFFFu);
    return ok;
}

bool Psoc::algo_set_cfg(uint8_t idx, uint8_t val) {
    // 写类异步(同 set_algo_rom / set_global 的既有语义): idx 走 ch 字段, core1 按序执行。
    return _submit(SpiOp::ALGO_SET_CFG, idx, 0, val, nullptr);
}

bool Psoc::algo_get_cfg(uint8_t idx, uint8_t* out_val) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::ALGO_GET_CFG, idx, 0, 0, &r);
    if (ok && out_val) *out_val = (uint8_t)(r & 0xFFu);
    return ok;
}

bool Psoc::set_algo_cfg_ch(uint8_t ch, uint8_t idx, uint8_t val) {
    // 写类异步(同 algo_set_cfg / set_algo_rom): 一次补推最多 288 条, 逐条阻塞等回显会把 core0
    // 按在 handler 里数百 ms。真值由 RP 存储持有(ALGO_GET_CFG_CH 回读的是 RP 侧真相源)。
    return _submit(SpiOp::ALGO_SET_CFG_CH, ch, idx, val, nullptr);
}

bool Psoc::algo_get_cfg_ch(uint8_t ch, uint8_t idx, uint8_t* out_val) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::ALGO_GET_CFG_CH, ch, idx, 0, &r);
    if (ok && out_val) *out_val = (uint8_t)(r & 0xFFu);
    return ok;
}

bool Psoc::algo_get_crc(bool* out_valid, uint16_t* out_crc) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::ALGO_GET_CRC, 0, 0, 0, &r);
    if (ok) {
        if (out_valid) *out_valid = ((r >> 16) & 1u) != 0u;
        if (out_crc) *out_crc = (uint16_t)(r & 0xFFFFu);
    }
    return ok;
}

bool Psoc::algo_get_heap(uint16_t* out_size, uint16_t* out_used) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::ALGO_GET_HEAP, 0, 0, 0, &r);
    if (ok) {
        if (out_size) *out_size = (uint16_t)(r & 0xFFFFu);
        if (out_used) *out_used = (uint16_t)((r >> 16) & 0xFFFFu);
    }
    return ok;
}

bool Psoc::set_global(uint8_t gparam_id, uint32_t value) {
    // ★core0 非阻塞(修 USB 掉线)★: 保存改全局项时 _handle_global_set 会连续 set_global+global_commit,
    // 若阻塞 core0(各~100ms)会饿死 USB → 主机写 SAVE_CONFIG 超时 → "Missing config bulk OUT endpoint"
    // 掉线。改为入队即返回; 写影子(set_global)与完整重初始化(global_commit)由 core1 按 ring 顺序执行。
    return _submit(SpiOp::SET_GLOBAL, gparam_id, 0, value, nullptr);   // gparam_id 走 ch 字段
}

bool Psoc::get_global(uint8_t gparam_id, uint32_t* out_value) {
    return _submit(SpiOp::GET_GLOBAL, gparam_id, 0, 0, out_value);   // 读类, 阻塞(按需, 不在保存热路径)
}

bool Psoc::global_commit() {
    // ★写类异步入队, 完成屏障放在 core1 内★
    // GLOBAL_COMMIT 会让 PSoC 主循环跑 Init/Initialize。要防的是"Init 与随后的 PARAM_SET 交错"
    // (实测表现: CH0/3/17/35 的 0x08/0x0A/0x0B 在重启 provision 后被 Init 重置回生成配置)。
    // 该屏障由 _exec_cmd 的 GLOBAL_COMMIT 分支内的 _wait_op_done 提供 —— 命令环是 FIFO 且由 core1 顺序
    // 执行, 后续 PARAM_SET 必然排在它之后, 顺序本身已足够。
    // ★不要让 core0 阻塞等它★: 实测 core0 在启动 provision 里干等(读类 _submit, 最长 1.5s)会与
    // core1 正在跑的 Init 叠加, 把触控/GET_STATS 的响应流水线搅乱 —— link_ok 抖动、scan_count 恒 0,
    // smoke 因 bring-up 缺 LINK 位与 link_valid=false 持续 FAIL。二分已确认这是唯一诱因。
    return _submit(SpiOp::GLOBAL_COMMIT, 0, 0, 0, nullptr);
}

bool Psoc::busy_grace_active() const {
    return (int32_t)(_reset_grace_until_ms - millis()) > 0;
}

void Psoc::reset_run() {
    _swd.reset_target_run();   // 脉冲 XRES 复位 PSoC 进运行态
    // Runtime apply 结果与恢复闸门由 core1/SensorLink 消费；XRES 不能在 core0 侧清掉，
    // 否则当前格结果与 heavy 所有权失配，且下一格会越过 provisioning 提前下发。
    // 复位后旧算法信息不能跨 PSoC 实例复用；递增 epoch 令 core0 读缓存时立即失效。
    _algo_info_epoch++;
    if (_algo_info_epoch == 0u) _algo_info_epoch = 1u;
    __dmb();
    // 设启动宽限: XRES 后 PSoC 需数百 ms 跑完 initialize_capsense 才恢复 SPI, 期间失效兜底
    // 不得触发新的复位, 否则形成永久复位死循环(链路永不恢复)。core1 的失效检测读此截止时间。
    _reset_grace_until_ms = millis() + RESET_BOOT_GRACE_MS;
}

bool Psoc::prepare_flash_indicator() {
    if (!_swd_ready) return false;

    // 旧 0.4.0 的 PING 每四次翻转，运行态灯值不可推断；XRES 复位会同时清零
    // 其静态计数，并由 generated pin config 在启动时把 P1.6 强驱动为高。
    _swd.reset_target_run();
    sleep_ms(75);
    return _spi_ready && _link.indicator_on();
}

bool Psoc::acquire() {
    if (!_swd_ready) {
        return false;
    }
    return _swd.acquire();
}

bool Psoc::psoc_debug_counters(uint32_t out[SwdProgrammer::DEBUG_COUNTER_WORDS]) {
    return _swd.debug_read_spi_counters(out);
}

bool Psoc::psoc_debug_read_words(const uint32_t* addresses, uint32_t* out_words, uint8_t word_count) {
    return _swd.debug_read_words(addresses, out_words, word_count);
}

uint32_t Psoc::psoc_debug_status() const {
    return _swd.debug_last_status();
}

uint32_t Psoc::psoc_debug_block_addr() const {
    return _swd.debug_block_addr();
}

bool Psoc::program(const uint8_t* data, uint32_t len) {
    if (!_swd_ready || data == nullptr) {
        return false;
    }
    if (!_swd.erase_all()) {
        return false;
    }
    return _swd.program_flash(data, len);
}
