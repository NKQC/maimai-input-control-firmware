#pragma once

#include "../../hal/uart/hal_uart.h"
#include <stdint.h>

/**
 * 协议层 - Mai2Light: 模拟官方 maimai DX LED 板 837-15070-04 (BD15070_4)
 *
 * 帧(请求): E0 dst src length command payload... sum
 * 帧(应答): E0 dst src length status command report payload... sum
 *   - 请求 length = command 起到 sum 之前的字节数; 应答 length = 3 + payload 字节数(status/command/report 各 1)
 *   - sum = 转义前, dst 起到 sum 之前逐字节相加取低 8 位
 *   - 转义: 原始字节为 0xE0/0xD0 时发 0xD0 + (原值-1); 收端遇 0xD0 则下一字节 +1 还原
 *
 * 虚拟 LED 单元共 11 个: 0..7 = 按键灯(写缓冲, 需 0x3C 提交); 8/9/10 = Body/Ext/Side 白灯(0x39 立即生效)。
 * 本类只维护"灯板应该输出什么颜色", 实际推到 WS2812 由 LedMapService 完成(职责分离)。
 */

#define MAI2LIGHT_NUM_LEDS          11      // 虚拟 LED 单元数
#define MAI2LIGHT_BUTTON_LEDS       8       // 其中按键灯数量(0..7)
#define MAI2LIGHT_SYNC_BYTE         0xE0
#define MAI2LIGHT_ESCAPE_BYTE       0xD0
#define MAI2LIGHT_MAX_PACKET_SIZE   48      // 去转义后的请求体上限(官方最长命令 payload 远小于此)
#define MAI2LIGHT_MAX_ACK_SIZE      24      // 应答体上限(最长为 GetBoardInfo: 6 + 10)
#define MAI2LIGHT_DEFAULT_BAUD_RATE 115200
#define MAI2LIGHT_EEPROM_SIZE       8       // 官方板只有 8 字节模拟 EEPROM

// 命令码(与官方板一致, 不可自行发明)
enum class Mai2Light_Command : uint8_t {
    SET_LED_GS_8BIT             = 0x31,
    SET_LED_GS_8BIT_MULTI       = 0x32,
    SET_LED_GS_8BIT_MULTI_FADE  = 0x33,
    SET_LED_FET                 = 0x39,
    SET_LED_GS_UPDATE           = 0x3C,
    SET_EEPROM                  = 0x7B,
    GET_EEPROM                  = 0x7C,
    SET_ENABLE_RESPONSE         = 0x7D,
    SET_DISABLE_RESPONSE        = 0x7E,
    GET_BOARD_INFO              = 0xF0,
    GET_BOARD_STATUS            = 0xF1,
    GET_FIRM_SUM                = 0xF2,
    GET_PROTOCOL_VERSION        = 0xF3,
};

// 应答 status / report(官方定义, 正常一律 0x01)
#define MAI2LIGHT_ACK_STATUS_OK         0x01
#define MAI2LIGHT_ACK_STATUS_SUM_ERROR  0x02
#define MAI2LIGHT_ACK_REPORT_OK         0x01
#define MAI2LIGHT_ACK_REPORT_NONE       0x00

struct Mai2Light_RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    Mai2Light_RGB() : r(0), g(0), b(0) {}
    Mai2Light_RGB(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
    inline void clear() { r = 0; g = 0; b = 0; }
};

// 单元状态(brightness/enabled 保留给上层缓存比较, 协议本身不区分)
struct Mai2Light_LEDStatus {
    Mai2Light_RGB color;
    uint8_t brightness;
    bool enabled;

    Mai2Light_LEDStatus() : brightness(255), enabled(true) {}
};

struct Mai2Light_Config {
    uint32_t baud_rate;
    uint8_t node_id;

    Mai2Light_Config() : baud_rate(MAI2LIGHT_DEFAULT_BAUD_RATE), node_id(0) {}
};

// 链路统计(上位机据此判断"游戏是否真在刷灯"以及线路质量)
struct Mai2Light_Stats {
    uint32_t rx_frames;     // 校验通过的帧数
    uint32_t sum_errors;    // 校验和错误数
    uint32_t unknown_cmds;  // 未知命令数

    inline void clear() { rx_frames = 0; sum_errors = 0; unknown_cmds = 0; }
};

class Mai2Light {
public:
    // 与 Mai2Serial 同一状态机范式: init→READY, 首个合法帧→RUNNING, 长时间无帧→READY, deinit→STOPPED
    enum class Status : uint8_t {
        STOPPED = 0,
        READY = 1,
        RUNNING = 2
    };

    Mai2Light(HAL_UART* uart_hal, uint8_t node_id = 0);
    ~Mai2Light();

    bool init();
    void deinit();
    inline bool is_ready() const { return _status != Status::STOPPED; }

    bool set_config(const Mai2Light_Config& config);
    bool get_config(Mai2Light_Config& config) const;

    // 非阻塞: 收帧 + 应答 + 渐变按时间片推进
    void task();

    const Mai2Light_LEDStatus* get_led_status_array() const { return _led; }
    Status get_status() const { return _status; }
    bool response_enabled() const { return _flags.resp_enabled; }
    const Mai2Light_Stats& get_stats() const { return _stats; }

private:
    Mai2Light(const Mai2Light&) = delete;
    Mai2Light& operator=(const Mai2Light&) = delete;

    // 成套开关归并, 避免散装 bool
    struct Flags {
        bool resp_enabled;  // 0x7D/0x7E 切换的"是否回应答"
        bool rx_active;     // 已见 sync, 正在收帧
        bool rx_escape;     // 上一字节是 0xD0
        inline void clear() { resp_enabled = true; rx_active = false; rx_escape = false; }
    };

    // 渐变(0x33 设参数, 0x3C 触发开始, task 内按时间推进)
    struct Fade {
        uint32_t start_ms;
        uint32_t end_ms;
        Mai2Light_RGB from;
        Mai2Light_RGB to;
        uint8_t first;
        uint8_t last;
        bool armed;    // 收到 0x33, 等 0x3C
        bool running;
        inline void clear() {
            start_ms = 0; end_ms = 0; from.clear(); to.clear();
            first = 0; last = 0; armed = false; running = false;
        }
    };

    inline void _feed(uint8_t byte);
    void _dispatch();
    void _cmd_set_single();
    void _cmd_set_multi(bool fade);
    void _cmd_set_fet();
    void _cmd_commit();
    void _fade_step(uint32_t now_ms);

    // payload 首字节地址与长度(请求体 = dst,src,len,cmd,payload...)
    inline const uint8_t* _req_payload() const { return &_rx[4]; }
    inline uint8_t _req_payload_len() const { return (_rx[2] > 0u) ? (uint8_t)(_rx[2] - 1u) : 0u; }
    inline uint8_t* _ack_payload() { return &_ack[6]; }
    inline void _stage_unit(uint8_t index, const Mai2Light_RGB& color);
    inline void _set_unit(uint8_t index, const Mai2Light_RGB& color);
    void _ack_send(uint8_t payload_len, uint8_t status = MAI2LIGHT_ACK_STATUS_OK,
                   uint8_t report = MAI2LIGHT_ACK_REPORT_OK);
    inline void _write_escaped(uint8_t value, uint8_t* out, uint8_t* out_len) const;

    HAL_UART* _uart;
    Status _status;
    Mai2Light_Config _config;
    Flags _flags;
    Mai2Light_Stats _stats;

    Mai2Light_LEDStatus _led[MAI2LIGHT_NUM_LEDS];   // 已提交(对外输出)色
    Mai2Light_RGB _stage[MAI2LIGHT_NUM_LEDS];       // 缓冲色, 0x3C 提交
    uint16_t _stage_mask;                           // 缓冲脏位, 避免提交时覆盖 0x39 的即时白灯
    Mai2Light_RGB _multi_color;                     // 上一次 0x32 设定色 = 渐变起始色
    Fade _fade;

    uint8_t _eeprom[MAI2LIGHT_EEPROM_SIZE];

    uint8_t _rx[MAI2LIGHT_MAX_PACKET_SIZE];  // 去转义后的请求体(不含 sync / sum)
    uint8_t _rx_len;
    uint8_t _rx_sum;                         // 增量累加校验和
    uint8_t _ack[MAI2LIGHT_MAX_ACK_SIZE];    // 应答体(不含 sync / sum)
    uint32_t _last_frame_ms;                 // 最近一次合法帧时刻, 用于 RUNNING→READY 回落
};
