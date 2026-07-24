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
    // 发送一帧遥测(若 _streaming)。周期由 TxScheduler 定时任务驱动, 本函数不再自门控频率。
    void tick();

    // TxScheduler 定时任务入口(无参函数指针): 转发到 getInstance()->tick()。
    static void emit_telem_task();

    // 停止遥测流(清 _streaming + 关快照慢路)。新主机会话(HELLO)时调用,
    // 使遗留遥测流不再淹没 vendor 端点、DEVICE_INFO 可正常送达。
    void stop();

private:
    SensorLink();
    SensorLink(const SensorLink&) = delete;
    SensorLink& operator=(const SensorLink&) = delete;

    static SensorLink* _instance;

    bool _streaming;
    uint8_t _mode;
    uint16_t _rate_hz;
    uint8_t _fields;
    uint64_t _ch_mask;
    uint32_t _last_emit_us;
    uint8_t _stream_seq;
    uint8_t _tx_buf[512];
    HostFrame _telem_frame;

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
    static void _handle_auto_tune(const HostFrame& frame, uint8_t* response, uint16_t* response_length);

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
