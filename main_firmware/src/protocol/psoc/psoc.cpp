#include "psoc.h"
#include "../../config.h"
#include "../../service/latency_stats.h"
#include <Arduino.h>
#include <pico/stdlib.h>
#include <hardware/sync.h>   // __dmb() / __sev() 跨核内存屏障

namespace {
constexpr uint32_t CORE1_CYCLE_US = 1000;        // core1 固定周期 1ms(1kHz), 保证传感器时序恒定
constexpr uint32_t SPI_CMD_TIMEOUT_US = 100000;  // core0 命令信箱等待上限 100ms
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

// core1 每周期执行体: 命令信箱 → 触控快路 → 快照慢路 → 采样率统计。
void Psoc::_spi_service() {
    if (!_spi_ready) return;

    // 1) 命令队列: 每周期最多消费 CMD_DRAIN_PER_CYCLE 条(限制周期抖动), FIFO 顺序执行。
    for (uint32_t n = 0; n < CMD_DRAIN_PER_CYCLE; ++n) {
        if (_cmd_tail == _cmd_head) break;          // 队空
        SpiCmd& c = _cmd_ring[_cmd_tail];
        uint32_t r = 0;
        const bool ok = _exec_cmd(c.op, c.ch, c.pid, c.val, &r);
        c.result = r;
        c.ok = ok;
        __dmb();
        c.done = true;                              // 发布结果(读类 core0 在等)
        __dmb();
        _cmd_tail = (_cmd_tail + 1) % CMD_RING_SIZE; // 消费者推进 tail, 释放槽位
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
        uint32_t sc = 0;
        if (ok && _spi.get_stats(&sc)) {
            if (_stats_primed) {
                const uint32_t dt = stats_now_us - _stats_last_us;
                const uint32_t dcount = sc - _stats_last_scan;
                _samples_per_sec = (uint32_t)(((uint64_t)dcount * 1000000ULL) / dt);
                _scan_period_us = (_samples_per_sec > 0u) ? (1000000u / _samples_per_sec) : 0u;
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
bool Psoc::_exec_cmd(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out) {
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
        case SpiOp::MEASURE_CP:
            return _spi.measure_cp();
        case SpiOp::GET_CP: {
            uint32_t v = 0;
            const bool ok = _spi.get_cp(ch, &v);
            if (out) *out = v;
            return ok;
        }
        default:
            return false;
    }
}

// core0 侧: 入队一条 SPI 指令。写类(out==null)异步立即返回; 读类阻塞等本条结果。
// core1 未接管(setup 阶段)时本核直执行, 避免死等无人消费的队列。
bool Psoc::_submit(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out) {
    if (!_spi_ready) return false;
    if (!_core1_running) {
        return _exec_cmd(op, ch, pid, val, out);
    }

    const bool wait = (out != nullptr);   // 读类需返回值

    // 等待队列有空位(仅当被 core1 拖慢时短暂自旋; 长期满=core1 卡死则超时放弃)。
    const uint32_t enq_start = time_us_32();
    while (((_cmd_head + 1u) % CMD_RING_SIZE) == _cmd_tail) {
        if (time_us_32() - enq_start > SPI_CMD_TIMEOUT_US) return false;
        tight_loop_contents();
    }

    const uint32_t slot = _cmd_head;
    SpiCmd& c = _cmd_ring[slot];
    c.op = op;
    c.ch = ch;
    c.pid = pid;
    c.val = val;
    c.result = 0;
    c.ok = false;
    c.done = false;
    __dmb();
    _cmd_head = (slot + 1u) % CMD_RING_SIZE;   // 发布: 推进 head, core1 下周期可见
    __sev();

    if (!wait) return true;   // 写类: 异步, 立即返回

    // 读类: 单生产者, 入队后本条槽位在读到 done 前不会被复用, 阻塞等结果。
    const uint32_t start = time_us_32();
    while (!c.done) {
        if (time_us_32() - start > SPI_CMD_TIMEOUT_US) return false;   // 超时
        tight_loop_contents();
    }
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
bool Psoc::apply_params() {
    return _submit(SpiOp::APPLY, 0, 0, 0, nullptr);
}
bool Psoc::set_mode(uint8_t mode) {
    return _submit(SpiOp::SET_MODE, mode, 0, 0, nullptr);
}
bool Psoc::measure_cp() {
    return _submit(SpiOp::MEASURE_CP, 0, 0, 0, nullptr);   // 异步(写类), 立即返回
}
bool Psoc::get_cp(uint8_t ch, uint32_t* out) {
    return _submit(SpiOp::GET_CP, ch, 0, 0, out);          // 读类, 阻塞等结果
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

bool Psoc::program(const uint8_t* data, uint32_t len) {
    if (!_swd_ready || data == nullptr) {
        return false;
    }
    if (!_swd.erase_all()) {
        return false;
    }
    return _swd.program_flash(data, len);
}
