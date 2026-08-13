#pragma once

#include "../../hal/usb/hal_usb_cdc_uart.h"
#include "../../protocol/mai2serial/mai2serial.h"
#include "../../protocol/mai2light/mai2light.h"
#include "../../protocol/host_cmd/host_cmd.h"
#include "../delay_line/delay_line.h"
#include "../led_map/led_map.h"

/** Serial-mode game protocol composition and light-state consumer cache. */
class GameIoService {
public:
    static GameIoService* getInstance();

    bool init(UsbWorkMode mode);
    void deinit();
    void task();

    // 注册 mai2 串口状态 host_cmd(MAI2_GET_STATE / MAI2_SET_SEND_EN)。与 init 分离:
    // HID 模式不跑 game_io 也要能被上位机查询, 此时如实回 STOPPED。
    void register_host_cmds();
    bool is_ready() const { return _initialized; }
    // 触控数据是否真的在发的唯一定义；调用方不得自行重新组合 is_ready()/get_serial_ok()，以免判定分叉。
    bool mai2_touch_sending() const;

    const Mai2Light_LEDStatus* light_state() const { return _light_state; }
    uint32_t light_generation() const { return _light_generation; }

private:
    struct SerialPublishSettings {
        uint16_t aggregation_delay_ms;
        uint16_t rate_limit_hz;
        uint8_t extra_send;
        union {
            struct {
                uint8_t rate_limit_enabled : 1;
                uint8_t send_only_on_change : 1;
                uint8_t reserved : 6;
            } bits;
            uint8_t flags;
        };

        void clear() {
            aggregation_delay_ms = 0u;
            rate_limit_hz = 10u;
            extra_send = 0u;
            flags = 0u;
        }
    };

    class SerialPublishState {
    public:
        void reset();
        void record(uint32_t now_us, const Mai2Serial_TouchState& sample);
        bool rate_limited(uint32_t now_us, const SerialPublishSettings& settings) const;
        Mai2Serial_TouchState value(uint32_t now_us, uint16_t window_ms);
        bool should_send(const Mai2Serial_TouchState& state,
                         const SerialPublishSettings& settings) const;
        void note_send_success(const Mai2Serial_TouchState& state, uint32_t now_us,
                               const SerialPublishSettings& settings);

    private:
        static constexpr uint16_t kAggregateSlots = 101u;
        struct Sample {
            uint32_t time_ms;
            uint64_t state;
        };
        union {
            struct {
                uint8_t has_last_sent : 1;
                uint8_t has_last_aggregate : 1;
                uint8_t reserved : 6;
            } bits;
            uint8_t flags;
        } _flags;
        Sample _samples[kAggregateSlots];
        Mai2Serial_TouchState _last_sent;
        Mai2Serial_TouchState _last_aggregate;
        Mai2Serial_TouchState _last_recorded;
        uint32_t _last_rate_send_us;
        uint8_t _remaining_extra_sends;
    };

    GameIoService();
    GameIoService(const GameIoService&) = delete;
    GameIoService& operator=(const GameIoService&) = delete;

    void _consume_light_state();
    void _process_serial_reset();
    void _refresh_serial_publish_settings();

    static void _handle_mai2_command(Mai2Serial_Command command, const uint8_t* params, uint8_t param_len);
    static void _handle_mai2_get_state(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_mai2_set_send_en(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_led_get(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_led_set_region(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_led_preview(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);

    static GameIoService* _instance;
    HAL_USB_CDC_UART _serial_uart;
    HAL_USB_CDC_UART _light_uart;
    Mai2Serial _serial;
    Mai2Light _light;
    Mai2Light_LEDStatus _light_state[MAI2LIGHT_NUM_LEDS];
    // 交给 LedMapService 的 flat RGB(11×3): 映射服务不依赖协议头, 只吃裸颜色。
    uint8_t _light_rgb[MAI2LIGHT_NUM_LEDS * 3];
    uint32_t _light_generation;
    bool _led_map_ready;
    bool _initialized;

    // 触控延迟线(100us 片, 0..100ms), 单拷贝环形; 协议设置 50ms 刷新一次。
    DelayLine<uint64_t> _touch_delay;
    SerialPublishSettings _serial_publish_settings;
    SerialPublishState _serial_publish;
    uint16_t _touch_delay_units;
    // 延迟线的补偿量: "本段起点 → 帧真正写出 CDC" 的实测耗时(us), 单指数平滑。
    // ★不得复用 g_lat_proc_us / g_lat_usb_us★ 那两个是**滚动峰值**, 且清零权在遥测发射器手里
    // (SensorLink::tick 组完 LATENCY 块才清)。没有上位机连着时它们永不清零, 会一路 latch 住历史
    // 最坏值(实测可达 1.5ms + 1.2ms), 于是 emit_us 被高估数毫秒 ⇒ 延迟线读到比应读更新的片
    // ⇒ 实际端到端延迟比设定值短几毫秒, 且随运行时间单调变短、不会自己恢复。
    // 平滑值也比峰值更贴近"这一拍要花多久"这个待预测量: 用峰值等于系统性过补偿。
    uint32_t _emit_cost_us;
    uint32_t _delay_refresh_us;
    uint8_t _serial_reset_requests;
};
