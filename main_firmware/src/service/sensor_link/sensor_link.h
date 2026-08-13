#pragma once

#include <cstdint>
#include "../../protocol/host_cmd/host_cmd.h"

#define SENSOR_LINK_CHANNELS 36
#define TELEM_FIELD_RAW      0x01
#define TELEM_FIELD_BASELINE 0x02
#define TELEM_FIELD_DIFF     0x04
#define TELEM_FIELD_STATUS   0x08
#define TELEM_FIELD_STATS    0x10
static constexpr uint8_t TELEM_FIELD_LATENCY = 0x20;
// ★算法运行值随帧上报★ 逐通道追加 out_active(u8) + report[4](u16 LE ×4) = 9B。
// 为什么不再用 ALGO_GET_TRACE 轮询: 那是"core0 阻塞等 core1 执行"的读类命令, 而设备的 vendor
// IN 是单响应槽; 独占流以上百帧/s 推送时它几乎抢不到响应窗口, 实测请求成片超时(既无响应也无
// NAK), 算法变量永远显示"暂无运行值"。改为与 raw/diff 同帧同源: 天然对齐、零额外往返。
static constexpr uint8_t TELEM_FIELD_ALGO = 0x40;
// ★触控延迟线的补偿偏差★ int16 dev_min(LE) + int16 dev_max(LE) + u8 flags = 5B。见 latency_stats.h:
// 语义是"(实际发出 − 采样) − comm.touch_delay_100us", 正=发晚、负=发早, 只有固件算得出。
// ★为什么另开字段位而不是把它塞进 LATENCY 块★ 帧里字段块的长度由 fields 位决定, 而 fields 是
// **主机在 TELEM_START 里指定的** ⇒ 新开一位天然向后兼容(旧主机不请求, 固件就不发);
// 而把 6B 的 LATENCY 块扩成 9B 会让任何不知情的解码方把后面的逐通道数据整片读偏。
// ★本位用掉了 fields 的最后一个 bit★: 再要新字段必须先扩宽 fields 宽度, 不得复用已退役位。
static constexpr uint8_t TELEM_FIELD_DELAY_DEV = 0x80;

/**
 * SensorLink exposes immutable PSoC CapSense snapshots on HostCmd telemetry.
 * Unsupported PSoC mutation commands explicitly return NOT_IMPLEMENTED.
 */
class SensorLink {
public:
    static SensorLink* getInstance();

    void init();
    // PSoC 设置类命令的单槽延迟终态，由 UsbComm 在响应缓冲空闲时取走并编码 ACK/NAK。
    bool host_write_active() const { return _host_write.active; }
    bool take_host_write_terminal(uint8_t* cmd, uint8_t* seq, bool* ok);
    // Returns true when `frame` is an active duplicate or a bounded terminal replay.
    bool replay_host_write(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    // 发送一帧遥测(仅当流处于 emitting 态)。周期由 TxScheduler 定时任务驱动, 本函数不再自门控频率。
    void tick();

    // TxScheduler 定时任务入口(无参函数指针): 转发到 getInstance()->tick()。
    static void emit_telem_task();

    // FocusSession 定时任务入口: 仅在出现新的快照 generation 时发送当前单通道帧。
    void focus_tick();
    static void emit_focus_task();
    static void focus_lease_expired_task();

    // SweepSession 定时任务入口: 单步推进参数切换、快照统计、原参数恢复与流式结果发送。
    void sweep_tick();
    static void emit_sweep_task();
    static void sweep_lease_expired_task();
    bool output_suppressed() const;

    // 发送一帧频率自适应阶段进度(AUTO_TUNE_PROGRESS)。周期由 TxScheduler 驱动;
    // 检测到设备侧终态即发最终帧、写穿真相源并自取消任务。
    void autotune_tick();
    static void emit_autotune_task();

    // 发送一帧 PSoC 救砖阶段进度(PSOC_RESCUE_PROGRESS)。与自适应同机制(TxScheduler 驱动 + 终态自取消);
    // 长擦写窗口内由 SWD 保活钩子调 TxScheduler::tick 顺带泵出, 故全程可见。
    void rescue_tick();
    static void emit_rescue_task();

    // 显式 TELEM_STOP 的永久停止语义：若扫描活跃则取消扫描并完成安全恢复，之后不会自动恢复。
    void stop();
    // HELLO 的新主机会话语义：清除遗留输出以确保 DEVICE_INFO 可优先送达；扫描活跃时保留其
    // PSoC 参数恢复链，仅在终态后静默输出，绝不把仍在进行的扫描取消在中间参数上。
    void prepare_host_session();

    // ★主机租约超时用★: 只挂起, 保留 rate/fields/ch_mask/lease 参数。
    // 停流的理由是"上位机疑似丢失", 但 core0 也可能只是被长设备操作(JIT 下发/校准/flash 落地)
    // 按在 UsbComm::update() 之外几秒 —— 那种情况下上位机其实一直在, 用 stop() 会把遥测永久停掉
    // 且没有任何恢复路径(实测: 算法更新后 raw/baseline 永久冻结, 只能复位设备)。
    void suspend();
    // 主机命令重新到达后由主循环调用: 若处于挂起态则用原参数续推(无挂起则空转)。
    void resume();

private:
    SensorLink();
    SensorLink(const SensorLink&) = delete;
    SensorLink& operator=(const SensorLink&) = delete;

    static SensorLink* _instance;

    // 遥测流状态。active 与 suspended 必须成对判定(挂起态要保留参数并禁止发送), 故合为一个
    // 结构体而非两个散装 bool, 避免任一处只更新其中一个导致"半挂起"。
    struct StreamState {
        bool active;      // 已被主机 TELEM_START 启用
        bool suspended;   // 因主机租约超时暂时停发, 主机回来即自动续推
        void clear() { active = false; suspended = false; }
        bool emitting() const { return active && !suspended; }
    };
    struct FocusSession {
        bool active = false;
        bool suspended = false;
        uint16_t id = 0;
        uint16_t sample_seq = 0;
        uint16_t last_generation = 0;
        uint32_t last_focus_scan_ack_us = 0;
        uint8_t generation_fence = 0;
        uint16_t rate_hz = 0;
        uint16_t lease_ms = 0;
        uint8_t channel = 0;
        uint8_t fields = 0;

        void clear() {
            active = false;
            suspended = false;
            id = 0;
            sample_seq = 0;
            last_generation = 0;
            last_focus_scan_ack_us = 0;
            generation_fence = 0;
            rate_hz = 0;
            lease_ms = 0;
            channel = 0;
            fields = 0;
        }
        bool emitting() const { return active && !suspended; }
    };

    struct SavedStreamState {
        bool valid = false;
        StreamState state { false, false };
        uint32_t lease_ms = 0;
        uint8_t mode = 0;
        uint16_t rate_hz = 0;
        uint8_t fields = 0;
        uint64_t ch_mask = 0;

        void clear() {
            valid = false;
            state.clear();
            lease_ms = 0;
            mode = 0;
            rate_hz = 0;
            fields = 0;
            ch_mask = 0;
        }
    };

    static constexpr uint16_t SWEEP_TOTAL = 448;
    static constexpr uint8_t SWEEP_GAIN_COUNT = 7;
    static constexpr uint8_t SWEEP_DIV_COUNT = 64;
    enum class SweepPhase : uint8_t {
        IDLE, SET_CELL, WAIT_PARAMS, READBACK, SETTLE, SAMPLE,
        RESTORE_WRITE, TERMINAL,
    };
    enum class SweepDataState : uint8_t { CELL = 1, RESTORING = 2, DONE = 3, CANCELLED = 4, FAILED = 5 };
    enum class SweepOutputRestore : uint8_t { BROAD, STOPPED, QUIET };
    struct SweepResult {
        uint16_t samples = 0;
        uint16_t mean = 0;
        uint16_t std_q8 = 0;
        uint16_t pp = 0;
        uint8_t gain = 0;
        uint8_t div = 1;
        uint8_t flags = 0;
        // 该格未取得有效结果时**卡在哪个阶段**(SweepPhase 枚举值; 0=无故障)。
        // flags 只能说明该格不可用，阶段码用于区分参数应用、回读或快照推进失败。
        uint8_t fail_phase = 0;

        void clear() {
            samples = 0; mean = 0; std_q8 = 0; pp = 0; gain = 0; div = 1; flags = 0; fail_phase = 0;
        }
    };
    // 单格聚合。★用 sum/sum_sq 而非 Welford★: raw 是 u16 且单格样本数 ≤ SWEEP_SAMPLE_MAX,
    // sum ≤ 2^22、sum_sq ≤ 2^38 都在 u64 内精确无溢出, 定点 Welford 反而要引入除法与截断误差。
    struct SweepStats {
        uint32_t count = 0;
        uint64_t sum = 0;
        uint64_t sum_sq = 0;
        uint16_t min = 0xFFFFu;
        uint16_t max = 0;
        uint8_t railed = 0;

        void clear() { count = 0; sum = 0; sum_sq = 0; min = 0xFFFFu; max = 0; railed = 0; }
    };
    struct SweepSession {
        SweepPhase phase = SweepPhase::IDLE;
        SweepDataState terminal = SweepDataState::DONE;
        SweepOutputRestore output_restore = SweepOutputRestore::BROAD;
        uint16_t id = 0;
        uint16_t cell = 0;
        uint16_t produced = 0;
        // 连续失败格数。单格失败记空洞继续扫, 连续失败到阈值才判真故障回滚(见 SWEEP_ABORT_FAIL_RUN)。
        uint16_t fail_run = 0;
        bool resolved[SWEEP_TOTAL] = {};
        bool sampled[SWEEP_TOTAL] = {};
        uint16_t stream_cursor = 0;
        uint16_t resend_cursor = 0;
        uint16_t resend_end = 0;
        uint16_t last_generation = 0;
        uint16_t settle_seen = 0;
        uint8_t channel = 0;
        uint8_t settle_samples = 0;
        uint8_t sample_count = 0;
        uint32_t phase_ticks = 0;   // 当前阶段已耗周期数
        uint8_t original_gain = 0;
        uint8_t original_div = 1;
        uint8_t restore_flags = 0;  // 原 gain/div 恢复失败位，回显在终态帧 flags。
        uint8_t fail_phase = 0;     // 本会话**首个**失败阶段(SweepPhase 值; 0=至今无故障), 回显在非结果帧
        uint16_t failed_cells = 0;  // 累计无效格数(仅诊断/终态文案用, 不影响流程)
        bool restore_announced = false;
        bool restoring = false;
        bool restore_started = false;
        SweepStats stats {};

        void clear() {
            phase = SweepPhase::IDLE;
            terminal = SweepDataState::DONE;
            output_restore = SweepOutputRestore::BROAD;
            id = 0; cell = 0; produced = 0; fail_run = 0;
            for (uint16_t i = 0; i < SWEEP_TOTAL; ++i) {
                resolved[i] = false;
                sampled[i] = false;
            }
            stream_cursor = 0; resend_cursor = 0; resend_end = 0;
            last_generation = 0; settle_seen = 0; channel = 0; settle_samples = 0; sample_count = 0;
            phase_ticks = 0; original_gain = 0; original_div = 1; restore_flags = 0;
            fail_phase = 0; failed_cells = 0;
            restore_announced = false; restoring = false; restore_started = false; stats.clear();
        }
        bool active() const { return phase != SweepPhase::IDLE; }
    };

    StreamState _stream { false, false };
    struct HostWriteState {
        enum class Kind : uint8_t { NONE, PARAM, MODE, GLOBAL, GLOBAL_COMMIT, ALGO_CFG, ALGO_ROM,
                                   CALIBRATE, BASELINE_RESET, CP_MEASURE };
        bool active = false;
        bool complete = false;
        bool ok = false;
        Kind kind = Kind::NONE;
        uint8_t cmd = 0;
        uint8_t seq = 0;
        uint8_t a = 0;
        uint8_t b = 0;
        uint32_t value = 0;
        uint8_t rom_count = 0;
        uint8_t rom_index = 0;
        uint8_t rom_ch[SENSOR_LINK_CHANNELS] = {};
        uint16_t rom_value[SENSOR_LINK_CHANNELS] = {};

        void clear() {
            active = false; complete = false; ok = false; kind = Kind::NONE;
            cmd = 0; seq = 0; a = 0; b = 0; value = 0; rom_count = 0; rom_index = 0;
        }
    };
    HostWriteState _host_write;
    uint8_t _host_write_last_cmd = 0;
    uint8_t _host_write_last_seq = 0;
    bool _host_write_last_ok = false;
    uint32_t _host_write_last_ms = 0;
    FocusSession _focus;
    SweepSession _sweep;
    SweepResult _sweep_results[SWEEP_TOTAL] = {};
    // 已判定格按真实 index 连续发布；空洞也进入 resolved 前缀并发送 samples=0 的空 CELL。
    SavedStreamState _saved_stream;
    uint16_t _focus_session_counter = 0;
    uint16_t _sweep_session_counter = 0;
    uint32_t _lease_ms;   // TELEM_START 协商的租约(自动恢复时复用同一值)
    uint8_t _mode;
    uint16_t _rate_hz;
    uint8_t _fields;
    uint64_t _ch_mask;
    uint32_t _last_emit_us;
    uint8_t _stream_seq;
    uint8_t _tx_buf[512];
    // 组帧缓冲: 遥测与自适应进度共用(两者都只在 core0 的 TxScheduler::tick 里顺序发送, 不会重入)。
    HostFrame _telem_frame;
    uint8_t _at_req_ch;      // 本轮自适应目标通道(0..35 / 0xFF), 供进度帧回显与写穿真相源
    // 本轮自适应的**上位机请求 seq**(AUTO_TUNE 请求帧的 seq): 逐帧回显在 AUTO_TUNE_PROGRESS 尾部,
    // 使上位机能把终态严格归属到它发起的那一次请求 —— 推送流的帧头 seq 是设备流序号, 与请求无关,
    // 没有这个回显时"上一轮的终态"会被算到下一个通道头上。
    uint8_t _at_req_seq = 0;
    bool _at_defer_save = false; // host 批量模式：逐通道终态不写 flash，批次结束统一保存
    uint16_t _at_ticks;      // 本轮已发进度帧数: 用于自续租的硬上限(防设备侧异常导致推送永不停)
    uint16_t _rescue_ticks = 0;   // 救砖进度帧数(同上, 自续租硬上限)

    static inline bool _channel_selected(uint64_t channel_mask, uint8_t channel) {
        return ((channel_mask >> channel) & 1ULL) != 0;
    }

    static void _handle_telem_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_telem_stop(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_focus_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_focus_stop(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_sweep_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_sweep_ctrl(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_unsupported(const HostFrame& frame, uint8_t* response, uint16_t* response_length);

    // 扫描会话进行中的互斥闸门: 会改 CSD 状态或抢 PSoC 重操作槽的命令一律回 DEVICE_BUSY。
    // 返回 true = 已写好 NAK, 调用方直接 return。
    static bool _sweep_busy_reject(const char* what, const HostFrame& frame,
                                   uint8_t* response, uint16_t* response_length);
    bool _start_host_write(HostWriteState::Kind kind, const HostFrame& frame,
                           uint8_t a, uint8_t b, uint32_t value);
    bool _start_next_host_rom();
    void _poll_host_write();
    void _pause_broad_for_focus();
    void _restore_broad_after_focus();
    void _release_focus_scan();
    void _begin_sweep_restore(SweepDataState terminal);
    bool _emit_sweep_frame(uint16_t index, SweepDataState state, bool retransmit);
    void _finish_sweep_cell();
    void _resolve_sweep_tail();
    // 当前格完成后推进到下一格(顺序全覆盖 0..SWEEP_TOTAL-1)。
    void _advance_sweep_cell();
    // 扫描阶段的有界超时: 当前格记空洞后继续下一格。
    void _sweep_cell_timeout(uint32_t limit);
    // 当前格判为空洞并推进; 连续失败达阈值才回滚整个会话。
    void _abort_sweep_cell();
    // 恢复链的有界超时: 记录首个失败阶段并置对应异常位。
    void _sweep_note_fail_phase();
    // 恢复完毕(或终态帧已送达/放弃重试)后的唯一收尾出口: 清会话 + 按 output_restore 复原输出。
    void _finish_sweep_session();
    void _sweep_enter(SweepPhase phase) { _sweep.phase = phase; _sweep.phase_ticks = 0; }
    static bool _sweep_phase_is_restore(SweepPhase phase) {
        return phase == SweepPhase::RESTORE_WRITE || phase == SweepPhase::TERMINAL;
    }
    static uint16_t _sqrt_u32(uint32_t value);

    // Phase A：CSD 运行时调参（转发 PSoC SPI 指令通道）
    static void _handle_param_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_param_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_param_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    // PARAM_GET_ALL 的"全通道单参数"变体(payload = 0xFF + param_id): 一帧回全 36 通道该参数值。
    static void _emit_param_all_channels(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_calibrate(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_baseline_reset(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_mode_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_csd_capture(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_cp_measure(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_cp_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length);

    // 全局 CSD 配置(未激活传感器连接/IDAC/MFS): 转发 PSoC SET/GET_GLOBAL + 写穿 CsdConfig
    static void _handle_global_get(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_global_set(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_global_get_all(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_global_commit(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_auto_tune(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_psoc_rescue(const HostFrame& frame, uint8_t* response, uint16_t* response_length);

    // JIT 算法引擎：查询/上传/应用/恢复默认(转发 PsocAlgo store + PSoC SPI ALGO_* 下发)
    static void _handle_algo_get_info(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_upload(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_apply(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_reset_default(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_set_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_get_rom(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    // 算法可调变量(cfg[8])。运行值(report[]/out_active)无主机命令: 随遥测帧的 TELEM_FIELD_ALGO 走。
    static void _handle_algo_set_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_get_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    // 算法 C 源(映射表)存取 + ASM 机器码回读
    static void _handle_algo_get_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_set_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_get_code(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
};
