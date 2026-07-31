#include "psoc.h"
#include "../../config.h"
#include "../../service/latency_stats.h"
#include "../../hal/usb/hal_usb.h"
#include <Arduino.h>
#include <pico/stdlib.h>
#include <hardware/sync.h>   // __dmb() / __sev() 跨核内存屏障
#include <hardware/watchdog.h>   // watchdog_update(): 重操作阻塞等待期间喂狗
#include "../../service/usb_debug.h"   // crash_stage_set(): 阻塞等待期间记录阶段码

namespace {
constexpr uint32_t CORE1_CYCLE_US = 1000;        // core1 固定周期 1ms(1kHz), 保证传感器时序恒定
constexpr uint32_t SPI_CMD_TIMEOUT_US = 100000;  // core0 命令信箱等待上限 100ms
// 失效兜底阈值:
//   链路丢失: 连续 ~200 个 1ms 周期(≈200ms)read_touch 失败 = PSoC 崩溃/掉线。
//   主循环卡死: scan_count 连续 8 个统计间隔(每间隔 ~500ms ⇒ ≈4s)不推进 = 主循环卡死(疑似坏算法)。
//     阈值刻意高于最长合法主循环停顿(MEASURE_CP 逐电极测量 ~1.5s), 避免测量期间误复位。
constexpr uint32_t LINK_FAIL_RESET_CYCLES = 200;
constexpr uint32_t HANG_STATS_INTERVALS   = 8;
// ★XRES 复位后 PSoC 启动宽限★: XRES 复位(手动 REBOOT_PSOC 或失效兜底)后 PSoC 需重跑
// initialize_capsense(含 Cy_CapSense_Enable 全通道自动校准)才恢复 SPI 应答, 耗时可达数百 ms,
// 远超 LINK_FAIL_RESET_CYCLES(200ms)。若不设宽限, 兜底会在 PSoC 启动完成前又发 XRES →
// 永久复位死循环 → 链路永不恢复(表现: 重启后 link 掉、需重烧)。宽限期内不累计失败/不再触发复位。
constexpr uint32_t RESET_BOOT_GRACE_MS    = 1500;
// ★重操作执行宽限★: APPLY/CALIBRATE/GLOBAL_COMMIT/BASELINE_RESET 由 PSoC 主循环同步执行,
// 实测 provision 后的 APPLY(36 通道逐个重校准)达 12.4s。期间 CapSense 内部临界区推迟 SPI DMA 中断,
// read_touch 会成批失败 —— 那不是掉线, 不得据此 XRES 或重新下发配置。给足余量到 30s。
// AUTO_TUNE 更长(逐通道最坏数十秒), 与其 SPI 层 45s 超时对齐并留余量。
constexpr uint32_t HEAVY_OP_GRACE_MS      = 30000;
constexpr uint32_t AUTOTUNE_GRACE_MS      = 60000;
}

Psoc* Psoc::_instance = nullptr;

Psoc::Psoc()
    : _swd(PIN_SWD_IO, PIN_SWD_CLK, PIN_SWD_RST),
      _spi(PIN_PSOC_SPI_SCK, PIN_PSOC_SPI_MOSI, PIN_PSOC_SPI_MISO, PIN_PSOC_SPI_CS),
      _spi_ready(false), _swd_ready(false), _link_ok(false), _last_update_ms(0) {}

Psoc* Psoc::getInstance() {
    if (!_instance) {
        _instance = new Psoc();
    }
    return _instance;
}

bool Psoc::init() {
    // SPI(PIO1) 是链路通道；SWD(PIO0) 是编程通道。二者互不干扰。
    _spi_ready = _spi.init();
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

// core1 每周期执行体: 命令信箱 → 触控快路 → 快照慢路 → 采样率统计。
void Psoc::_spi_service() {
    if (!_spi_ready) return;

    // 1) 命令队列: 每周期最多消费 CMD_DRAIN_PER_CYCLE 条(限制周期抖动), FIFO 顺序执行。
    for (uint32_t n = 0; n < CMD_DRAIN_PER_CYCLE; ++n) {
        if (_cmd_tail == _cmd_head) break;          // 队空
        SpiCmd& c = _cmd_ring[_cmd_tail];
        uint32_t r = 0;
        // ★in_cmd 必须包住整条执行★: core0 侧的 flash 落地要 multicore_lockout core1, 而重操作
        // (PSoC 重初始化 / CalibrateAllWidgets)在 _wait_op_done 里轮询数秒, 期间 core1 不进 wfe
        // 也就响应不了 lockout ⇒ core0 死等且不喂狗 ⇒ 5s 看门狗复位整机(实测: 保存后约 8.5s 掉线,
        // 设备重新枚举、LED 重启)。core0 据此标志避开这个窗口落盘。
        _core1_in_cmd = 1u;
        crash_stage_set(CRASH_STAGE_CORE1_CMD);
        __dmb();
        const bool ok = _exec_cmd(c.op, c.ch, c.pid, c.val, &r, c.data);
        c.result = r;
        c.ok = ok;
        __dmb();
        c.done = true;                              // 发布结果(读类 core0 在等)
        __dmb();
        _cmd_tail = (_cmd_tail + 1) % CMD_RING_SIZE; // 消费者推进 tail, 释放槽位
        // 长周期指令出清: core1 是本计数器的唯一写者, 与 core0 的 _heavy_enq 配对(见 heavy_busy)。
        if (_op_is_heavy(c.op)) _heavy_done = _heavy_done + 1u;
        __dmb();
        _core1_in_cmd = 0u;
    }

    // 2) 触控快路: 单次 7 字节事务读 36 区位图, 实测耗时。seqlock 发布防 u64 撕裂。
    uint64_t mask = 0;
    const uint32_t t0 = time_us_32();
    const bool ok = _spi.read_touch(&mask);
    const uint32_t tr = time_us_32() - t0;
    latency_note(&g_lat_spi_us, tr);

    _pub_seq++;                       // 进入写临界区(奇)
    __dmb();
    if (ok) _pub_touch_mask = mask;   // 读失败保留旧掩码, 避免误抬起
    _pub_link_ok = ok;
    _pub_touch_read_us = tr;
    __dmb();
    _pub_seq++;                       // 离开写临界区(偶)

    // 失效兜底(链路丢失): 链路曾就绪后连续多周期失败 = PSoC 崩溃/掉线 → 请求 XRES 复位。
    // 复位窗口内 PSoC 短暂无响应(~数周期)远低于阈值, 不会误触发。
    if (ok) {
        _link_established = true;
        _link_fail_run = 0;
    } else if (_link_established) {
        // XRES 复位后的启动宽限期内: 不累计失败、不触发复位, 让 PSoC 有时间跑完 initialize_capsense
        // 并恢复 SPI 应答, 避免"启动未完成→又复位"的死循环。宽限过后仍失败才判为真崩溃。
        if ((int32_t)(_reset_grace_until_ms - millis()) > 0) {
            _link_fail_run = 0;
        } else if (++_link_fail_run >= LINK_FAIL_RESET_CYCLES && _pub_reset_reason == 0) {
            _pub_reset_reason = 1;
        }
    }

    // 链路存活指示(遥测未激活时): 仅 bump generation/valid 单字段(原子, 无需 seqlock)。
    if (ok && !_telem_active) {
        _snapshot.valid = true;
        uint16_t g = _snapshot.generation + 1;
        if (g == 0) g = 1;
        _snapshot.generation = g;
    }

    // 3) 快照慢路: 遥测激活时分块流水读全通道 → 工作缓冲, 读满一份经 seqlock 发布到 _snapshot。
    if (_telem_active && ok) {
        if (_spi.snapshot_pump(PSOC_SNAPSHOT_PAGES_PER_PUMP, &_snap_work)) {
            _snap_seq++;              // 进入写临界区(奇)
            __dmb();
            _snapshot = _snap_work;   // 结构体整体拷贝(含 channels 数组)
            __dmb();
            _snap_seq++;              // 离开写临界区(偶)
        }
    }

    // 4) 采样率统计: 每 ~500ms 读 PSoC scan_count 折算采样率/刷新周期。
    const uint32_t stats_now_us = time_us_32();
    if (stats_now_us - _stats_last_us >= 500000u) {
        // ★XRES 启动宽限同样适用于"卡死"检测★: 复位后 PSoC 要重跑 initialize_capsense(数百 ms),
        // 期间 scan_count 天然不推进。若此时不清零, 上一轮已攒到阈值的 _hang_intervals 会让兜底在
        // PSoC 尚未启动完成时立刻再发一次 XRES → 反复复位 → 采样永远起不来(只能整机复位才恢复)。
        // 同时清 primed: 跨复位的 scan_count 从 0 重新开始, 与旧值相减毫无意义。
        if ((int32_t)(_reset_grace_until_ms - millis()) > 0) {
            _hang_intervals = 0;
            _stats_primed = false;
            _stats_last_us = stats_now_us;
            return;
        }
        uint32_t sc = 0;
        uint8_t psoc_busy = 0;
        if (ok && _spi.get_stats(&sc, &psoc_busy)) {
            if (_stats_primed) {
                const uint32_t dt = stats_now_us - _stats_last_us;
                const uint32_t dcount = sc - _stats_last_scan;
                _samples_per_sec = (uint32_t)(((uint64_t)dcount * 1000000ULL) / dt);
                _scan_period_us = (_samples_per_sec > 0u) ? (1000000u / _samples_per_sec) : 0u;
                // 失效兜底(主循环卡死): scan_count 长时间不推进 = PSoC 主循环卡死(疑似坏算法死循环)。
                // 注意 read_touch 仍由 PSoC SPI ISR 应答, 故 link_ok 不掉, 只能靠 scan_count 检测。
                // ★"忙"不等于"卡死": PSoC 报告 busy 时不得累计卡死计数★
                // APPLY/CALIBRATE/GLOBAL_COMMIT/AUTO_TUNE 由 PSoC 主循环同步执行, 期间 scan_count
                // 天然不推进(逐通道 CalibrateWidget 在 36 通道上可远超 4s, 尤其 provision 期 SPI
                // 中断负载很高时)。此前把它误判为"主循环卡死"→ XRES 复位 → 复位又触发 provision
                // 重发 APPLY → 永久复位循环(实测: 每轮 boot→APPLY→约 3s 后被复位, scan_count 恒 1,
                // raw 全 0, SelfHeal 累计上百次)。真正的卡死(坏算法死循环)不会置 busy, 仍能被抓到;
                // 重操作本身另有 _wait_op_done 的超时兜底, 不依赖这里。
                if (_link_established) {
                    if (dcount == 0u && psoc_busy == 0u) {
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

// core1 入口: 置运行标志后进入固定 1ms 周期循环, 永不返回。
// 注意: 调用方(core1_entry)必须已先调 multicore_lockout_victim_init(), 否则 flash 写入会死锁。
void Psoc::core1_run() {
    _core1_running = true;
    __dmb();
    while (true) {
        const uint32_t cycle_start = time_us_32();
        _spi_service();
        // 恒定周期: 补足到 CORE1_CYCLE_US, 保证传感器时序每周期一致。
        while ((time_us_32() - cycle_start) < CORE1_CYCLE_US) {
            tight_loop_contents();
        }
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
            _reset_grace_until_ms = millis() + HEAVY_OP_GRACE_MS;
            break;
        case SpiOp::AUTO_TUNE:
            _reset_grace_until_ms = millis() + AUTOTUNE_GRACE_MS;
            break;
        default:
            break;
    }

    switch (op) {
        case SpiOp::SET_PARAM:
            return _spi.set_param(ch, pid, val);
        case SpiOp::GET_PARAM: {
            uint32_t v = 0;
            const bool ok = _spi.get_param(ch, pid, &v);
            if (out) *out = v;
            return ok;
        }
        case SpiOp::GET_RAW: {
            uint16_t v = 0;
            const bool ok = _spi.get_raw(ch, &v);
            if (out) *out = v;
            return ok;
        }
        case SpiOp::SET_MODE:
            return _spi.set_mode(ch);   // mode 复用 ch 字段
        case SpiOp::APPLY:
            return _spi.apply();
        case SpiOp::CALIBRATE:
            return _spi.calibrate();
        case SpiOp::BASELINE_RESET:
            return _spi.baseline_reset();
        case SpiOp::MEASURE_CP:
            return _spi.measure_cp();
        case SpiOp::GET_CP: {
            uint32_t v = 0;
            const bool ok = _spi.get_cp(ch, &v);
            if (out) *out = v;
            return ok;
        }
        case SpiOp::UPLOAD_ALGO: {
            // val 打包 crc16<<16 | len; data 为 blob 指针(调用方持久缓冲)
            const uint16_t len = (uint16_t)(val & 0xFFFFu);
            const uint16_t crc = (uint16_t)((val >> 16) & 0xFFFFu);
            const bool ok = _spi.upload_algo(data, len, crc);
            // 异步下发的真实结果只能在这里得知: 失败置标志由 core0 取走上报(SH_ALGO_FALLBACK detail=1),
            // 忙标志同时释放, 允许下一次上传改写 blob。
            if (!ok) _algo_dl.failed = 1u;
            __dmb();
            _algo_dl.busy = 0u;
            return ok;
        }
        case SpiOp::GET_ALGO_INFO: {
            bool valid = false;
            uint16_t l = 0;
            const bool ok = _spi.algo_info(&valid, &l);
            if (out) *out = ((uint32_t)(valid ? 1u : 0u) << 16) | l;
            return ok;
        }
        case SpiOp::SET_ALGO_ROM:
            return _spi.set_algo_rom(ch, (uint16_t)val);
        case SpiOp::GET_ALGO_ROM: {
            uint16_t v = 0;
            const bool ok = _spi.get_algo_rom(ch, &v);
            if (out) *out = v;
            return ok;
        }
        case SpiOp::ALGO_GET_TRACE: {
            // ch=通道, pid=report idx(复用); out 打包 active<<16 | report
            uint8_t active = 0;
            uint16_t report = 0;
            const bool ok = _spi.algo_get_trace(ch, pid, &active, &report);
            if (out) *out = ((uint32_t)active << 16) | report;
            return ok;
        }
        case SpiOp::ALGO_SET_CFG:
            return _spi.algo_set_cfg(ch, (uint8_t)val);   // ch 字段复用为 cfg idx
        case SpiOp::ALGO_GET_CFG: {
            uint8_t v = 0;
            const bool ok = _spi.algo_get_cfg(ch, &v);   // ch 字段复用为 cfg idx
            if (out) *out = v;
            return ok;
        }
        case SpiOp::SET_GLOBAL:
            return _spi.set_global(ch, val);   // ch 字段复用为 gparam_id
        case SpiOp::GET_GLOBAL: {
            uint32_t v = 0;
            const bool ok = _spi.get_global(ch, &v);
            if (out) *out = v;
            return ok;
        }
        case SpiOp::GLOBAL_COMMIT:
            return _spi.global_commit();
        case SpiOp::AUTO_TUNE: {
            uint8_t result = 0; uint16_t div = 0;
            // ch 字段复用为目标通道(0xFF=全通道); pid 字段复用为灵敏度档位 pref(1..7)
            // 阶段进度: 本核在 _spi.auto_tune 的阻塞等待中被回调, 经 seqlock 发布给 core0(推送上位机)。
            _at_work.clear();
            _at_work.req = _at_req;
            _at_work.state = 1;
            _at_work.ch = ch;
            _publish_autotune();
            const bool ok = _spi.auto_tune(ch, pid, &result, &div, &Psoc::_on_autotune_progress, this);
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
bool Psoc::_submit(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out, const uint8_t* data, uint32_t timeout_us) {
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
// 入队即返回(ACK 表示"已受理"); 重活仍由 core1 的 _spi.* 内 _wait_op_done 完成(只占用 core1,
// 期间 touch 暂停但 core0/USB 正常)。完成后 raw 经遥测自然刷新, UI 无需阻塞等待。
bool Psoc::apply_params() {
    return _submit(SpiOp::APPLY, 0, 0, 0, nullptr);
}
bool Psoc::calibrate() {
    return _submit(SpiOp::CALIBRATE, 0, 0, 0, nullptr);
}
bool Psoc::baseline_reset() {
    return _submit(SpiOp::BASELINE_RESET, 0, 0, 0, nullptr);
}
// 频率自适应: 阻塞至 PSoC 重校准完成。out 打包 result(低8位) | div<<8。
// ch: 0..35=仅该通道, 0xFF=全通道逐通道各自校准(36 × 单通道 ≈ 11-23s) → 窗口 50s(> SPI 层 45s)。
bool Psoc::auto_tune(uint8_t ch, uint8_t pref, uint8_t* out_result, uint16_t* out_div) {
    uint32_t packed = 0;
    const bool ok = _submit(SpiOp::AUTO_TUNE, ch, pref, 0, &packed, nullptr, 50000000u);
    if (out_result) *out_result = (uint8_t)(packed & 0xFFu);
    if (out_div) *out_div = (uint16_t)((packed >> 8) & 0xFFFFu);
    return ok;
}
// ★异步启动★: 入队即返回(同 calibrate/baseline_reset 的写类语义), core0 不再为 20-25s 长自适应干等。
// 先自增 _at_req 再入队: core1 取到本条命令时回显该代号, core0 据此区分本轮进度与上一轮残留结果。
bool Psoc::auto_tune_start(uint8_t ch, uint8_t pref) {
    _at_req++;
    __dmb();
    return _submit(SpiOp::AUTO_TUNE, ch, pref, 0, nullptr);
}

// core1: 把工作副本发布为一致快照(多字段防撕裂, 同 _snapshot 的 seqlock 模式)。
void Psoc::_publish_autotune() {
    _at_seq++;            // 进入写临界区(奇)
    __dmb();
    _at_pub = _at_work;
    __dmb();
    _at_seq++;            // 离开写临界区(偶)
}

void Psoc::_on_autotune_progress(void* ctx, const psoc::AutoTuneProgress& p) {
    Psoc* self = static_cast<Psoc*>(ctx);
    if (self == nullptr) return;
    self->_at_work.state = 1;
    self->_at_work.phase = p.phase;
    self->_at_work.step = p.step;
    self->_at_work.cur_div = p.cur_div;
    // 全通道模式下 PSoC 回显"当前正在处理的通道"→ 必须透传, 上位机才能显示 CHn/36 的逐通道进度。
    self->_at_work.ch = p.ch;
    self->_publish_autotune();
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
bool Psoc::measure_cp() {
    uint32_t acknowledged = 0;
    // MEASURE_CP 必须等待 core1 完成实际 SPI 事务并收到 PSoC ACK，Host 才能回复 ACK。
    return _submit(SpiOp::MEASURE_CP, 0, 0, 0, &acknowledged);
}
bool Psoc::get_cp(uint8_t ch, uint32_t* out) {
    return _submit(SpiOp::GET_CP, ch, 0, 0, out);          // 读类, 阻塞等结果
}

bool Psoc::upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16) {
    if (data == nullptr || len == 0 || len > 1024) return false;
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

bool Psoc::algo_get_trace(uint8_t ch, uint8_t idx, uint8_t* out_active, uint16_t* out_report) {
    uint32_t r = 0;
    const bool ok = _submit(SpiOp::ALGO_GET_TRACE, ch, idx, 0, &r);
    if (ok) {
        if (out_active) *out_active = (uint8_t)((r >> 16) & 0xFFu);
        if (out_report) *out_report = (uint16_t)(r & 0xFFFFu);
    }
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
    // 该屏障由 PsocSpi::global_commit 内的 _wait_op_done 提供 —— 命令环是 FIFO 且由 core1 顺序
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
    return _spi_ready && _spi.indicator_on();
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
