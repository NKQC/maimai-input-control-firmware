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
    // link_alive() 的默认判定窗口(ms): 足以跨过快照分页与重操作期间的连续失败, 远小于掉线判据。
    static constexpr uint32_t LINK_ALIVE_WINDOW_MS = 750u;
    // ★触摸保留窗口(core1 周期数, 1 周期 = 1ms)★: 连续这么多周期取不到合法触控帧才把掩码优雅
    // 释放为全 0。取 60ms —— 长于遥测分页/命令流水造成的成批错帧(实测数个到十几个周期), 短于
    // LINK_FAIL_RESET_CYCLES(200ms)的掉线判据, 也短于人手最短一次点击, 故既不制造假抬起,
    // 也不会在真故障时把按下状态永久保持住。
    static constexpr uint32_t TOUCH_RELEASE_FAIL_CYCLES = 60u;

    static Psoc* getInstance();

    // 初始化 SPI(PIO1) 与 SWD(PIO0) 两条传输通道
    bool init();

    // ---------- SPI 链路 ----------
    void update();                    // 兼容: 执行一次 SPI 服务(仅 setup 阶段 core1 未启动时直调)
    void core1_run();                 // ★core1 入口★: 固定 1ms 周期独占 PSoC SPI 传感器循环, 永不返回
    // core1 发布(bool 原子): 最近一拍 read_touch 的**瞬时**结果。实时消费者(触控/键盘/灯效)用它。
    bool link_ok() const { return _pub_link_ok; }
    // 去抖判据: 最近一次 read_touch 成功在窗口内即为真。供上传/下发这类非实时动作做门禁。
    bool link_alive(uint32_t within_ms = LINK_ALIVE_WINDOW_MS) const;

    // Phase C：遥测慢路开关。激活后 update() 分块流水读全通道 raw/baseline/diff 填充 snapshot()。
    void set_telemetry_active(bool active) { _telem_active = active; }
    bool telemetry_active() const { return _telem_active; }
    // 独占单通道目标: <36 = 快照只读该通道(每拍一份, 高速精调); 0xFF = 关闭快路走全通道分块慢路。
    // core0 写 / core1 读的单字节, 天然原子; volatile 保证 core1 每拍重新读取而非缓存在寄存器里。
    void set_focus_channel(uint8_t ch) { _focus_ch = ch; }
    uint8_t focus_channel() const { return _focus_ch; }
    // PSoC 实际扫描切换。仅收到 PSoC 对目标/状态的明确回显才成功；0xFF 关闭单通道模式。
    bool set_focus_scan(uint8_t ch);

    // ---------- 实时触控快路（core1 发布, core0 经 seqlock 读）----------
    // ★touch_mask() 自身就是触摸真相★: 合法新帧才更新; 短暂错帧保留最后一份合法值;
    // 连续失败超过 TOUCH_RELEASE_FAIL_CYCLES 由 core1 主动释放为 0。
    // 故消费者【不得】再用瞬时 link_ok() 给它套一层清零 —— 那正是假抬起/假按下的来源。
    uint64_t touch_mask() const;                                // 36 区 on/off 位图(seqlock 防撕裂)
    // 掩码是否仍在可信窗口内(最近有合法帧, 或仍处于保留窗口)。需要"有没有触摸源"这一门禁的
    // 消费者(绑定捕获/键盘映射)用它, 不要自行用 link_ok() 重新组合判据。
    bool touch_hold_ok() const { return _pub_touch_hold; }
    uint32_t touch_read_us() const { return _pub_touch_read_us; }  // 最近触控读取耗时(us,原子)
    // 低成本诊断计数(core1 单写者, 只增不减; 不打日志, 由上位机/压测按差值判断链路质量):
    //   touch_bad_frames() = read_touch 未取到合法帧的周期数(含重试用尽);
    //   touch_releases()   = 因持续失败而优雅释放掩码的次数(每次故障只记一次)。
    uint32_t touch_bad_frames() const { return _pub_touch_bad; }
    uint32_t touch_releases() const { return _pub_touch_releases; }

    // ---------- 失效兜底(core1 检测, core0 执行 XRES 复位) ----------
    // 返回复位原因: 0=无, 1=SPI 链路持续丢失(PSoC 崩溃/掉线), 2=主循环卡死(scan_count 长时间不推进, 疑似坏算法)。
    uint8_t needs_reset() const { return _pub_reset_reason; }
    void clear_reset_request() { _pub_reset_reason = 0; }
    // 宽限期内(XRES 启动中 / 正在执行重操作)链路失败不代表 PSoC 真的掉线或被复位。
    // 调用方据此避免"把正忙的 PSoC 判成已复位"而重新下发配置(见 _reset_grace_until_ms 注释)。
    bool busy_grace_active() const;

    // ---------- Phase A：CSD 运行时指令（core0 调用→命令信箱→core1 独占 SPI 执行；签名不变）----------
    bool set_param(uint8_t ch, uint8_t param_id, uint32_t value);
    bool get_param(uint8_t ch, uint8_t param_id, uint32_t* out);
    bool get_raw(uint8_t ch, uint16_t* out);
    bool measure_cp();                          // 触发逐电极寄生电容测量；返回 true 表示 BIST 与固件 CSD 恢复已完成
    bool get_cp(uint8_t ch, uint32_t* out);     // 读指定通道 Cp：测量中=0，成功=fF，失败/未测量=0xFFFFFF

    // ---------- JIT 算法引擎：下发/查询(core0→信箱→core1 独占 SPI) ----------
    // data 必须持续有效直到下发完成(调用方持久缓冲, 见 PsocAlgo::_blob)。
    // ★异步入队(修 USB 掉线 + 遥测永久冻结)★: core1 单次下发 = 256 页 SPI 事务 + PSoC commit 轮询
    // (最坏 ~700ms), 远超命令信箱默认 100ms 等待窗。原先按"读类"阻塞 core0 会 (1) 必然超时误报
    // "download failed", (2) 让 core0 在 host_cmd handler 里滞留近 1s 不跑 UsbComm::update() →
    // 主机租约与 TxScheduler 租约一并过期 → 遥测被停且无恢复路径。改为入队即返回。
    bool upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16);
    bool algo_download_busy() const { return _algo_dl.busy != 0u; }

    // ---------- 长周期指令在途判据(反堆叠闸门的唯一真相源) ----------
    /// 是否有长周期 PSoC 指令在途(已入队但未执行完)。长周期 = PSoC 主循环同步执行数秒~数十秒的那类:
    /// APPLY / CALIBRATE / BASELINE_RESET / GLOBAL_COMMIT / AUTO_TUNE / MEASURE_CP / UPLOAD_ALGO。
    /// ★调用方不得自行用 core1_idle() 或 busy_grace_active() 重新组合出"忙"★:
    /// 前者把普通读写命令也算忙(过严, 正常轮询就会被拒), 后者是 30s 宽限窗(过宽, 会把设备锁死半分钟)。
    /// 判据 = 两个单写者计数器之差 —— core0 只写 _heavy_enq, core1 只写 _heavy_done, 故无跨核 RMW 竞态
    /// (与 _cmd_head/_cmd_tail 同一手法); 用单个 bool 会在"内部 APPLY 与主机指令先后入队"时被提前清掉。
    bool heavy_busy() const { return _heavy_enq != _heavy_done; }
    /// 因"忙"被拒的长周期指令累计次数。压测据此**确证**堆叠真实发生过, 而不是靠推断。
    uint32_t heavy_reject_count() const { return _heavy_rejects; }
    void note_heavy_reject() { _heavy_rejects++; }

    /// core1 是否空闲(命令环已排空且当前没有在执行的命令)。
    /// core0 落 flash 前必须确认为 true —— flash 写会 multicore_lockout core1, 而 core1 正在跑的
    /// 重操作(重初始化/全通道校准)会在 _wait_op_done 里轮询数秒, 那期间它响应不了 lockout,
    /// core0 就会死等到看门狗复位。宁可把落盘推迟到下一轮, 也不能在这个窗口里进 lockout。
    bool core1_idle() const {
        return (_cmd_head == _cmd_tail) && (_core1_in_cmd == 0u);
    }
    // core0: 取走并清除"最近一次下发未通过 PSoC commit 校验"标志(用于一次性上报, 不重复刷屏)。
    bool algo_download_take_failure();
    bool get_algo_info(bool* out_valid, uint16_t* out_len);   // 读 PSoC 端算法 valid/len
    // 只读 core1 周期刷新结果；不会投递或同步等待 SPI 命令。返回 true 表示缓存可用。
    bool get_algo_info_cached(bool* out_valid, uint16_t* out_len) const;
    bool set_algo_rom(uint8_t ch, uint16_t rom);              // 设每通道 16 位只读 ROM
    bool get_algo_rom(uint8_t ch, uint16_t* out_rom);         // 读每通道 16 位 ROM
    // 算法运行时追踪(report[]/out_active)与可调变量(cfg[8], 见 psoc_algo_abi.h)
    bool algo_get_trace(uint8_t ch, uint8_t idx, uint8_t* out_active, uint16_t* out_report);
    bool algo_set_cfg(uint8_t idx, uint8_t val);
    bool algo_get_cfg(uint8_t idx, uint8_t* out_val);
    bool set_global(uint8_t gparam_id, uint32_t value);       // 写全局 CSD 配置(仅影子, 不重初始化)
    bool get_global(uint8_t gparam_id, uint32_t* out_value);  // 读全局 CSD 配置
    bool global_commit();                                     // 全局项设完后触发一次完整重初始化

    // ---------- 采样率统计（每 ~500ms 由 update() 用 PSoC 自由递增 scan_count 折算）----------
    uint32_t samples_per_sec() const { return _samples_per_sec; }
    uint32_t scan_period_us() const { return _scan_period_us; }
    bool apply_params();
    // 真正的 IDAC 重校准 + 基线复位。ch: 0..35=仅该通道(PSoC 只校准该 widget 并只初始化该 widget
    // 基线), 0xFF=全 36 通道。★通道走 _submit 既有的 ch 字段★, 不新增命令码/不扩帧。
    bool calibrate(uint8_t ch = 0xFFu);
    // 基线复位。ch: 0..35=仅该通道, 0xFF=全通道。
    bool baseline_reset(uint8_t ch = 0xFFu);
    // SweepSession 专用的非阻塞重操作: 仅允许单个在途操作；完成结果由 take_sweep_result() 取走。
    // ★start_sweep_apply 走 QUICK_APPLY(0x3F) 而非 APPLY★: 逐格把 gain/div 随帧原子下发，绝不先
    // SET_PARAM 改 widgetContext；PSoC 仅在 NOT_BUSY 窗口写入并 Initialize。
    bool start_sweep_apply(uint8_t ch, uint8_t gain, uint8_t div);
    bool start_sweep_calibrate(uint8_t ch);
    bool start_sweep_baseline_reset(uint8_t ch);
    bool take_sweep_result(bool* out_ok);
    // 频率自适应下探(阻塞至完成, 最多~10s): ch 0..35=单通道 / 0xFF=全通道;
    // pref 灵敏度档位 1..7(越高越灵敏, 落档时往低频多让分频);
    // out_result 0进行中/1成功/2失败, out_div 最终写入的分频;
    // host_seq = 发起本轮的上位机请求 seq(折成 6 bit 标签下到 PSoC 并回显, 用于识别陈旧结果)。
    bool auto_tune(uint8_t ch, uint8_t pref, uint8_t host_seq, uint8_t* out_result, uint16_t* out_div);
    // ★异步启动(推荐)★: 入队即返回, core0 不阻塞; 阶段进度经 autotune_status() 读, 由服务层推送上位机。
    // core1 仍在 _exec_cmd 内一次跑完整个自适应(不拆成跨周期状态机), 否则 _spi_service 的 scan_count
    // 卡死兜底会在 PSoC 长校准期间误判并对其硬复位。
    // host_seq = 发起本轮的上位机请求 seq(见 auto_tune 说明), 服务层把它一路带到进度推送里。
    bool auto_tune_start(uint8_t ch, uint8_t pref, uint8_t host_seq);
    // 本轮请求代号(core0 侧自增): 与 autotune_status().req 不等 ⇒ core1 尚未开始本轮(结果字段仍属上一轮)。
    uint32_t autotune_req() const { return _at_req; }
    psoc::AutoTuneProgress autotune_status() const;   // seqlock 一致读(core1 发布)
    bool set_mode(uint8_t mode);   // 0=自动校准/标准完整处理，1=半自动手动
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
    void reset_run();   // 复位 PSoC 进运行态(脉冲 XRES) + 设启动宽限期防兜底复位死循环
    void release_swd() { _swd.release_swd(); }
    bool swd_ready() const { return _swd_ready; }

    // 运行态带外 SWD 诊断（仅 core0 host 命令路径调用，不经 core1 SPI）。
    bool psoc_debug_counters(uint32_t out[SwdProgrammer::DEBUG_COUNTER_WORDS]);
    bool psoc_debug_read_words(const uint32_t* addresses, uint32_t* out_words, uint8_t word_count);
    uint32_t psoc_debug_status() const;
    uint32_t psoc_debug_block_addr() const;

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
    volatile uint8_t _focus_ch = 0xFF;   // 独占单通道目标(0xFF = 无, 走全通道分块慢路)
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
    enum class SpiOp : uint8_t { NONE, SET_PARAM, GET_PARAM, GET_RAW, SET_MODE, APPLY, QUICK_APPLY, FOCUS_SCAN, CALIBRATE, BASELINE_RESET, MEASURE_CP, GET_CP,
                                 UPLOAD_ALGO, GET_ALGO_INFO, SET_ALGO_ROM, GET_ALGO_ROM,
                                 ALGO_GET_TRACE, ALGO_SET_CFG, ALGO_GET_CFG,
                                 SET_GLOBAL, GET_GLOBAL, GLOBAL_COMMIT, AUTO_TUNE };
    volatile bool _core1_running = false;   // core1_run() 已接管 SPI 后置真

    // 发布态: core1 唯一写者, core0 经 seqlock 读(u64 触控掩码防撕裂)
    volatile uint32_t _pub_seq = 0;         // 触控发布序列(奇=写入中)
    volatile uint64_t _pub_touch_mask = 0;
    volatile bool     _pub_link_ok = false;
    volatile bool     _pub_touch_hold = false;   // 掩码可信(见 touch_hold_ok)
    volatile uint32_t _pub_link_ok_ms = 0;   // 最近一次 read_touch 成功时刻(ms), 0=从未成功
    volatile uint32_t _pub_touch_read_us = 0;
    volatile uint32_t _pub_touch_bad = 0;        // 累计无合法触控帧的周期数(诊断)
    volatile uint32_t _pub_touch_releases = 0;   // 累计优雅释放次数(诊断)
    uint32_t _touch_fail_run = 0;                // 连续无合法帧周期数(core1 独占, 与掉线判据分开计)

    // 失效兜底检测态(core1 唯一写者, core0 只读 _pub_reset_reason / 清零)
    volatile uint8_t  _pub_reset_reason = 0;   // 0/1/2, core1 置位, core0 处理后清零
    bool     _link_established = false;         // 链路曾就绪(避免启动期误判)
    uint32_t _link_fail_run = 0;                // 连续 read_touch 失败周期数
    // 宽限截止时刻: 期内一律【不把 PSoC 判成死了】(不累计链路失败/不累计卡死/不清 provisioned)。
    // 两个来源: ① XRES 复位后的启动宽限; ② 派发重操作(APPLY/CALIBRATE/GLOBAL_COMMIT/AUTO_TUNE)后的
    // 执行宽限 —— 这些操作由 PSoC 主循环同步执行(逐通道校准可达 12s+), 期间 CapSense 内部临界区会
    // 推迟 SPI DMA 中断, 响应装不上 → read_touch 成批失败。若照旧判为"PSoC 掉线/被复位", 就会
    // XRES 它并清 provisioned 重新下发, 而重新下发又把 36 通道全部置脏、再触发一次 12s 重校准 ——
    // 形成永久 provision 风暴(实测 setparam 约 428/s、apply 约 1.1/s、scan_count 几乎不动)。
    // core0 写, core1 读。
    volatile uint32_t _reset_grace_until_ms = 0;
    uint32_t _hang_intervals = 0;               // 连续 scan_count 不推进的统计间隔数
    // 算法信息缓存：仅 core1 刷新/失效，core0 通过 seqlock 只读；epoch 由 core0 在复位时递增。
    static constexpr uint32_t ALGO_INFO_CACHE_MAX_AGE_MS = 500u;
    volatile uint32_t _algo_info_seq = 0;
    volatile uint8_t _algo_info_available = 0;
    volatile uint8_t _algo_info_valid = 0;
    volatile uint16_t _algo_info_len = 0;
    volatile uint32_t _algo_info_refresh_ms = 0;
    volatile uint32_t _algo_info_cache_epoch = 0;
    volatile uint32_t _algo_info_epoch = 1;
    uint32_t _algo_info_last_poll_ms = 0;       // core1 独占

    // 频率自适应阶段进度: core1 唯一写者(经 _at_seq seqlock 发布多字段一致副本), core0 只读。
    // _at_req 反向: core0 唯一写者(启动时自增), core1 只读回显, 使 core0 能区分"上一轮的 done"。
    volatile uint32_t _at_req = 0;
    volatile uint32_t _at_seq = 0;                        // 自适应发布序列(奇=写入中)
    psoc::AutoTuneProgress _at_pub;                       // 发布副本(core1 写, core0 seqlock 读)
    psoc::AutoTuneProgress _at_work;                      // core1 工作副本
    mutable psoc::AutoTuneProgress _at_ro;                // core0 读出的一致副本
    void _publish_autotune();                             // core1: _at_work → _at_pub(seqlock)
    static void _on_autotune_progress(void* ctx, const psoc::AutoTuneProgress& p);   // SPI 层回调
    volatile uint32_t _snap_seq = 0;        // 快照发布序列(奇=写入中)
    psoc::SensorSnapshot _snap_work;             // core1 快照流水工作缓冲
    mutable psoc::SensorSnapshot _snapshot_ro;   // core0 seqlock 读出的一致副本

    // 算法下发状态(core0 发起, core1 执行)。"忙"与"失败"必须成对判定, 故合为一个结构体而非散装 bool。
    struct AlgoDownloadState {
        volatile uint8_t busy;      // 1 = core1 仍在执行 UPLOAD_ALGO(期间拒绝新的上传, 防 blob 被改写)
        volatile uint8_t failed;    // 1 = 最近一次下发未通过 PSoC commit 校验, 待 core0 上报后清零
        void clear() { busy = 0u; failed = 0u; }
    };
    AlgoDownloadState _algo_dl { 0u, 0u };

    struct SweepAsyncState {
        volatile uint8_t pending = 0;
        volatile uint8_t complete = 0;
        volatile uint8_t ok = 0;
        volatile uint8_t token = 0;

        void clear() { pending = 0; complete = 0; ok = 0; token = 0; }
    };
    SweepAsyncState _sweep_async;

    // 命令 SPSC 环形队列(core0 生产, core1 消费, 单拷贝, 无锁):
    //   写类指令(set_param/set_mode/apply) 入队即返回(异步), core0 不再每条阻塞 ~1ms;
    //   读类指令(get_param/get_raw) 入队后按 FIFO 阻塞等本条 done, 顺序与前序写一致。
    // 深度 64: 一次完整算法下发 = 1×UPLOAD_ALGO + 36×SET_ALGO_ROM + 8×SET_CFG = 45 条,
    // 必须能一次性全部入队, 否则 core0 仍会卡在入队自旋里(等于没异步)。
    static constexpr uint32_t CMD_RING_SIZE = 64;        // 队列深度
    static constexpr uint32_t CMD_DRAIN_PER_CYCLE = 4;   // core1 每周期最多消费条数(限制周期抖动)
    struct SpiCmd {
        SpiOp          op = SpiOp::NONE;
        uint8_t        ch = 0;
        uint8_t        pid = 0;
        uint32_t       val = 0;
        const uint8_t* data = nullptr;   // 仅 UPLOAD_ALGO: blob 指针(调用方持久缓冲)
        uint32_t       result = 0;
        bool           ok = false;
        volatile bool  done = false;   // core1 执行完置真(读类 core0 等此位)
        uint8_t        async_token = 0; // Sweep 非阻塞重操作的完成归属(0=普通命令)
    };
    SpiCmd            _cmd_ring[CMD_RING_SIZE];
    volatile uint32_t _cmd_head = 0;   // core0 生产位置(生产者独占推进)
    volatile uint32_t _cmd_tail = 0;   // core1 消费位置(消费者独占推进)
    volatile uint8_t  _core1_in_cmd = 0;  // core1 正在执行一条 SPI 命令(见 core1_idle 注释)
    // 长周期指令在途计数(见 heavy_busy)。core0 只写 _heavy_enq, core1 只写 _heavy_done。
    volatile uint32_t _heavy_enq = 0;
    volatile uint32_t _heavy_done = 0;
    uint32_t          _heavy_rejects = 0;   // core0 独占
    static bool _op_is_heavy(SpiOp op);

    void _spi_service();   // core1 每周期: 命令队列 + 触控快路 + 快照慢路 + 采样率统计
    bool _refresh_algo_info_cache();       // core1 安全位置直接读取 PSoC
    void _invalidate_algo_info_cache();   // core1 写者：链路失效时清空
    bool _start_sweep_op(SpiOp op, uint8_t ch, uint8_t pid = 0u, uint32_t val = 0u);
    bool _submit(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out, const uint8_t* data = nullptr,
                 uint32_t timeout_us = 0u, uint8_t async_token = 0u);
    bool _exec_cmd(SpiOp op, uint8_t ch, uint8_t pid, uint32_t val, uint32_t* out, const uint8_t* data = nullptr); // 实际执行(core1 或 setup 直调)

    static Psoc* _instance;
};
