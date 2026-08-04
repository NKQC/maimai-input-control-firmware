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

/**
 * SensorLink exposes immutable PSoC CapSense snapshots on HostCmd telemetry.
 * Unsupported PSoC mutation commands explicitly return NOT_IMPLEMENTED.
 */
class SensorLink {
public:
    static SensorLink* getInstance();

    void init();
    // 发送一帧遥测(仅当流处于 emitting 态)。周期由 TxScheduler 定时任务驱动, 本函数不再自门控频率。
    void tick();

    // TxScheduler 定时任务入口(无参函数指针): 转发到 getInstance()->tick()。
    static void emit_telem_task();

    // 发送一帧频率自适应阶段进度(AUTO_TUNE_PROGRESS)。周期由 TxScheduler 驱动;
    // 检测到设备侧终态即发最终帧、写穿真相源并自取消任务。
    void autotune_tick();
    static void emit_autotune_task();

    // 发送一帧 PSoC 救砖阶段进度(PSOC_RESCUE_PROGRESS)。与自适应同机制(TxScheduler 驱动 + 终态自取消);
    // 长擦写窗口内由 SWD 保活钩子调 TxScheduler::tick 顺带泵出, 故全程可见。
    void rescue_tick();
    static void emit_rescue_task();

    // 停止遥测流(清流状态 + 关快照慢路)。新主机会话(HELLO)与 TELEM_STOP 时调用,
    // 使遗留遥测流不再淹没 vendor 端点、DEVICE_INFO 可正常送达。之后不会自动恢复。
    void stop();

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
    StreamState _stream { false, false };
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
    static void _handle_unsupported(const HostFrame& frame, uint8_t* response, uint16_t* response_length);

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
    // 算法运行时追踪(report[]/out_active)与可调变量(cfg[8])
    static void _handle_algo_get_trace(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_set_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_get_cfg(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    // 算法 C 源(映射表)存取 + ASM 机器码回读
    static void _handle_algo_get_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_set_src(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_algo_get_code(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
};
