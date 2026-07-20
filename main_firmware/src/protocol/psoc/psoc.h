#pragma once

#include <stdint.h>
#include <stddef.h>
#include "psoc_types.h"
#include "psoc_swd.h"
#include "psoc_spi.h"

/**
 * Psoc - PSoC 协议门面（单例）
 *
 * 封装底层两条通道，对服务层出统一抽象：
 *   - SPI 通信（PIO1）：周期 ping/pong 维护链路状态；后续 CSD 数据传输。
 *   - SWD 编程（PIO0）：acquire / 读 silicon id / erase+program+verify flash。
 * 服务层只依赖本门面，不直接接触 PIO/SWD/SPI 细节。
 */
class Psoc {
public:
    static Psoc* getInstance();

    // 初始化 SPI(PIO1) 与 SWD(PIO0) 两条传输通道
    bool init();

    // ---------- SPI 链路 ----------
    void update();                    // 兼容: 执行一次 SPI 服务(仅 setup 阶段 core1 未启动时直调)
    void core1_run();                 // ★core1 入口★: 固定 1ms 周期独占 PSoC SPI 传感器循环, 永不返回
    bool link_ok() const { return _pub_link_ok; }   // core1 发布(bool 原子)

    // Phase C：遥测慢路开关。激活后 update() 分块流水读全通道 raw/baseline/diff 填充 snapshot()。
    void set_telemetry_active(bool active) { _telem_active = active; }
    bool telemetry_active() const { return _telem_active; }

    // ---------- 实时触控快路（core1 发布, core0 经 seqlock 读）----------
    uint64_t touch_mask() const;                                // 36 区 on/off 位图(seqlock 防撕裂)
    uint32_t touch_read_us() const { return _pub_touch_read_us; }  // 最近触控读取耗时(us,原子)

    // ---------- Phase A：CSD 运行时指令（core0 调用→命令信箱→core1 独占 SPI 执行；签名不变）----------
    bool set_param(uint8_t ch, uint8_t param_id, uint32_t value);
    bool get_param(uint8_t ch, uint8_t param_id, uint32_t* out);
    bool get_raw(uint8_t ch, uint16_t* out);
    bool measure_cp();                          // 触发逐电极寄生电容测量(异步,经命令队列投递 core1)
    bool get_cp(uint8_t ch, uint32_t* out);     // 读指定通道最近 Cp 值(fF)

    // ---------- 采样率统计（每 ~500ms 由 update() 用 PSoC 自由递增 scan_count 折算）----------
    uint32_t samples_per_sec() const { return _samples_per_sec; }
    uint32_t scan_period_us() const { return _scan_period_us; }
    bool apply_params();
    bool set_mode(uint8_t mode);   // 0=全自动 SmartSense, 1=半自动/手动
    const psoc::SensorSnapshot& snapshot() const;   // seqlock 拷贝到 _snapshot_ro 后返回引用
    bool snapshot_valid() const { return _snapshot.valid; }        // bool 原子, 直读
    uint16_t snapshot_generation() const { return _snapshot.generation; }  // u16 原子, 直读

    // 烧录前白灯准备：先复位旧 PSoC，使 0.4.0/0.4.1 generated 启动态确定为 P1.6 高，
    // 再发向后兼容命令确认 SPI。返回失败只影响指示诊断，不阻止烧录。
    bool prepare_flash_indicator();
    bool set_program_indicator() { return _swd.set_program_indicator(); }

    // ---------- SWD 编程 ----------
    bool acquire();
    uint32_t idcode() const { return _swd.last_idcode(); }
    bool read_silicon_id(uint32_t* out_id) { return _swd.read_silicon_id(out_id); }
    bool program(const uint8_t* data, uint32_t len);   // erase_all + program_flash
    bool verify(const uint8_t* data, uint32_t len) { return _swd.verify_flash(data, len); }
    bool checksum(uint32_t* out) { return _swd.checksum_all(out); }
    void reset_run() { _swd.reset_target_run(); }   // 烧录后复位 PSoC 进运行态
    void release_swd() { _swd.release_swd(); }
    bool swd_ready() const { return _swd_ready; }

    // ---------- 细粒度 flash 步骤 + 诊断（bring-up 用，Phase B 收敛） ----------
    bool set_imo() { return _swd.set_imo_48mhz(); }
    bool read_clk_trim_snapshot() { return _swd.read_clk_trim_snapshot(); }
    bool erase() { return _swd.erase_all(); }
    bool program_rows(const uint8_t* data, uint32_t len) { return _swd.program_flash(data, len); }

    uint32_t last_srom_status() const { return _swd.last_srom_status(); }
    uint32_t clk_trim1() const { return _swd.clk_trim1(); }
    uint32_t clk_trim3() const { return _swd.clk_trim3(); }
    uint32_t sfl_trim_word() const { return _swd.sfl_trim_word(); }
    uint32_t sfl_tctrim_word() const { return _swd.sfl_tctrim_word(); }
    bool clock_config_ok() const { return _swd.clock_config_ok(); }
    uint32_t clock_select() const { return _swd.clock_select(); }
    uint32_t clock_imo_select() const { return _swd.clock_imo_select(); }
    uint32_t clock_trim1() const { return _swd.clock_trim1(); }
    uint32_t clock_trim2() const { return _swd.clock_trim2(); }
    uint32_t clock_trim3() const { return _swd.clock_trim3(); }
    bool erase_scan_complete() const { return _swd.erase_scan_complete(); }
    uint32_t erase_flash_sum() const { return _swd.erase_flash_sum(); }
    uint32_t erase_flash_or() const { return _swd.erase_flash_or(); }
    uint32_t erase_first_nonzero_addr() const { return _swd.erase_first_nonzero_addr(); }
    uint32_t erase_first_nonzero_value() const { return _swd.erase_first_nonzero_value(); }
    uint32_t erase_words_read() const { return _swd.erase_words_read(); }
    uint32_t last_acquire_status() const { return _swd.last_acquire_status(); }
    uint32_t last_acquire_sysreq() const { return _swd.last_acquire_sysreq(); }
    uint32_t last_acquire_delay() const { return _swd.last_acquire_delay(); }
    uint16_t last_fail_row() const { return _swd.last_fail_row(); }
    uint32_t last_fail_addr() const { return _swd.last_fail_addr(); }
    uint32_t last_verify_read() const { return _swd.last_verify_read(); }
    uint32_t last_verify_expect() const { return _swd.last_verify_expect(); }
    bool read_chip_protection(uint8_t* out_prot) { return _swd.read_chip_protection(out_prot); }
    uint32_t read_row_protection() { return _swd.read_row_protection(); }
    uint32_t last_rowprot0() const { return _swd.last_rowprot0(); }
    uint32_t last_rowprot1() const { return _swd.last_rowprot1(); }
    uint8_t last_chip_prot() const { return _swd.last_chip_prot(); }

private:
    Psoc();
    Psoc(const Psoc&) = delete;
    Psoc& operator=(const Psoc&) = delete;

    SwdProgrammer _swd;
    PsocSpi _spi;

    bool _spi_ready;
    bool _swd_ready;
    bool _link_ok;
    bool _telem_active = false;       // 遥测慢路是否激活（TELEM_START/STOP 控制）
    uint32_t _last_update_ms;
    psoc::SensorSnapshot _snapshot;
    uint64_t _touch_mask = 0;
    uint32_t _touch_read_us = 0;

    // 采样率统计（每 ~500ms 读 PSoC scan_count 折算）
    uint32_t _samples_per_sec = 0;
    uint32_t _scan_period_us = 0;
    uint32_t _stats_last_scan = 0;
    uint32_t _stats_last_us = 0;
    bool _stats_primed = false;

    // ---------- 双核: core1 独占 SPI, seqlock 发布共享态 + 命令信箱投递低频指令 ----------
    // RP2040 无 cache, 跨核共享用 volatile + __dmb() 内存屏障即可保证可见性与顺序。
    enum class SpiOp : uint8_t { NONE, SET_PARAM, GET_PARAM, GET_RAW, SET_MODE, APPLY, MEASURE_CP, GET_CP };
    volatile bool _core1_running = false;   // core1_run() 已接管 SPI 后置真

    // 发布态: core1 唯一写者, core0 经 seqlock 读(u64 触控掩码防撕裂)
    volatile uint32_t _pub_seq = 0;         // 触控发布序列(奇=写入中)
    volatile uint64_t _pub_touch_mask = 0;
    volatile bool     _pub_link_ok = false;
    volatile uint32_t _pub_touch_read_us = 0;
    volatile uint32_t _snap_seq = 0;        // 快照发布序列(奇=写入中)
    psoc::SensorSnapshot _snap_work;             // core1 快照流水工作缓冲
    mutable psoc::SensorSnapshot _snapshot_ro;   // core0 seqlock 读出的一致副本

    // 命令 SPSC 环形队列(core0 生产, core1 消费, 单拷贝, 无锁):
    //   写类指令(set_param/set_mode/apply) 入队即返回(异步), core0 不再每条阻塞 ~1ms;
    //   读类指令(get_param/get_raw) 入队后按 FIFO 阻塞等本条 done, 顺序与前序写一致。
    static constexpr uint32_t CMD_RING_SIZE = 32;        // 队列深度
    static constexpr uint32_t CMD_DRAIN_PER_CYCLE = 4;   // core1 每周期最多消费条数(限制周期抖动)
    struct SpiCmd {
        SpiOp         op = SpiOp::NONE;
        uint8_t       ch = 0;
        uint8_t       pid = 0;
        uint32_t      val = 0;
        uint32_t      result = 0;
        bool          ok = false;
        volatile bool done = false;   // core1 执行完置真(读类 core0 等此位)
    };
    SpiCmd            _cmd_ring[CMD_RING_SIZE];
    volatile uint32_t _cmd_head = 0;   // core0 生产位置(生产者独占推进)
    volatile uint32_t _cmd_tail = 0;   // core1 消费位置(消费者独占推进)

    void _spi_service();   // core1 每周期: 命令队列 + 触控快路 + 快照慢路 + 采样率统计
    bool _submit(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out);  // core0 投递(读类等结果)
    bool _exec_cmd(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out); // 实际执行(core1 或 setup 直调)

    static Psoc* _instance;
};
