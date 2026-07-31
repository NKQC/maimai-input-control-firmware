#include "mai2light.h"
#include "../../service/usb_debug.h"
#include <pico/time.h>
#include <cstring>

namespace {
// RUNNING→READY 回落阈值: 官方板 GetBoardStatus 的 timeoutSec 为 1s, 但游戏在结算/加载间隙
// 可能短暂不刷灯; 阈值取 3s 余量, 避免把正常间隙误判成"灯板链路断开"。
constexpr uint32_t IDLE_FALLBACK_MS = 3000u;
// 一次 task 最多读 4 个 64B 块: 灯板流量远小于此, 上限只为防止异常涌入时占死主循环。
constexpr uint8_t RX_CHUNKS_PER_TASK = 4u;
constexpr uint8_t RX_CHUNK_SIZE = 64u;
// SetLedGs8BitMulti 的 end==0x20 表示"到全部灯末尾"(官方约定)。
constexpr uint8_t MULTI_END_ALL = 0x20u;
// 渐变总时长(ms) = 4095 / speed * 8(官方近似式, 整数除法与官方板一致)。
constexpr uint32_t FADE_BASE = 4095u;
constexpr uint32_t FADE_SCALE = 8u;

inline uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

inline uint8_t blend_channel(uint8_t from, uint8_t to, uint8_t progress) {
    return (uint8_t)(((uint16_t)from * (uint16_t)(255u - progress) + (uint16_t)to * (uint16_t)progress) / 255u);
}
}  // namespace

Mai2Light::Mai2Light(HAL_UART* uart_hal, uint8_t node_id)
    : _uart(uart_hal), _status(Status::STOPPED), _stage_mask(0u),
      _rx_len(0u), _rx_sum(0u), _last_frame_ms(0u) {
    _config.node_id = node_id;
    _flags.clear();
    _stats.clear();
    _fade.clear();
    memset(_eeprom, 0, sizeof(_eeprom));
    memset(_ack, 0, sizeof(_ack));
    memset(_rx, 0, sizeof(_rx));
}

Mai2Light::~Mai2Light() {
    deinit();
}

bool Mai2Light::init() {
    if (_status != Status::STOPPED) return true;
    if (_uart == nullptr) return false;

    for (uint8_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
        _led[i].color.clear();
        _led[i].brightness = 255u;
        _led[i].enabled = true;
        _stage[i].clear();
    }
    _stage_mask = 0u;
    _multi_color.clear();
    _fade.clear();
    _flags.clear();
    _rx_len = 0u;
    _rx_sum = 0u;
    _last_frame_ms = now_ms();
    _status = Status::READY;
    return true;
}

void Mai2Light::deinit() {
    _status = Status::STOPPED;
    _flags.rx_active = false;
    _flags.rx_escape = false;
    _rx_len = 0u;
    _fade.clear();
}

bool Mai2Light::set_config(const Mai2Light_Config& config) {
    _config = config;
    return true;
}

bool Mai2Light::get_config(Mai2Light_Config& config) const {
    config = _config;
    return true;
}

void Mai2Light::task() {
    if (_status == Status::STOPPED) return;

    uint8_t chunk[RX_CHUNK_SIZE];
    for (uint8_t round = 0; round < RX_CHUNKS_PER_TASK; round++) {
        pm_stage(PM_STAGE_LIGHT_RX_READ);
        const size_t got = _uart->read_from_rx_buffer(chunk, sizeof(chunk));
        pm_stage(PM_STAGE_LIGHT_FEED);
        for (size_t i = 0; i < got; i++) _feed(chunk[i]);
        if (got < sizeof(chunk)) break;
    }

    pm_stage(PM_STAGE_LIGHT_FADE);
    const uint32_t now = now_ms();
    _fade_step(now);
    // 长时间无合法帧 = 游戏侧已停止刷灯 → 回落 READY(链路仍在, 只是没人驱动)。
    if (_status == Status::RUNNING && (now - _last_frame_ms) > IDLE_FALLBACK_MS) {
        _status = Status::READY;
    }
}

// ---------------- 收帧 ----------------

inline void Mai2Light::_feed(uint8_t byte) {
    if (byte == MAI2LIGHT_SYNC_BYTE) {
        _rx_len = 0u;
        _rx_sum = 0u;
        _flags.rx_active = true;
        _flags.rx_escape = false;
        return;
    }
    if (!_flags.rx_active) return;

    if (byte == MAI2LIGHT_ESCAPE_BYTE) {
        _flags.rx_escape = true;
        return;
    }
    if (_flags.rx_escape) {
        byte = (uint8_t)(byte + 1u);
        _flags.rx_escape = false;
    }

    // 体长 = dst,src,len(3) + len 字节; 收满后本字节即 sum。
    if (_rx_len >= 4u && (uint16_t)_rx_len == (uint16_t)_rx[2] + 3u) {
        _flags.rx_active = false;   // 本帧结束, 等下一个 sync
        if (_rx_sum == byte) {
            _stats.rx_frames++;
            _last_frame_ms = now_ms();
            _status = Status::RUNNING;   // 首个校验通过的帧即进入 RUNNING
            _dispatch();
        } else {
            _stats.sum_errors++;
            _ack_send(0u, MAI2LIGHT_ACK_STATUS_SUM_ERROR, MAI2LIGHT_ACK_REPORT_NONE);
        }
        return;
    }
    if (_rx_len >= MAI2LIGHT_MAX_PACKET_SIZE) {
        _flags.rx_active = false;   // 超长(必为噪声/错位), 丢弃等重同步
        return;
    }
    _rx[_rx_len++] = byte;
    _rx_sum = (uint8_t)(_rx_sum + byte);
}

void Mai2Light::_dispatch() {
    const uint8_t* payload = _req_payload();
    const uint8_t payload_len = _req_payload_len();

    switch ((Mai2Light_Command)_rx[3]) {
        case Mai2Light_Command::SET_LED_GS_8BIT:
            _cmd_set_single();
            break;
        case Mai2Light_Command::SET_LED_GS_8BIT_MULTI:
            _cmd_set_multi(false);
            break;
        case Mai2Light_Command::SET_LED_GS_8BIT_MULTI_FADE:
            _cmd_set_multi(true);
            break;
        case Mai2Light_Command::SET_LED_FET:
            _cmd_set_fet();
            break;
        case Mai2Light_Command::SET_LED_GS_UPDATE:
            _cmd_commit();
            break;
        case Mai2Light_Command::SET_EEPROM:
            if (payload_len >= 2u && payload[0] < MAI2LIGHT_EEPROM_SIZE) {
                _eeprom[payload[0]] = payload[1];
            }
            _ack_send(0u);
            break;
        case Mai2Light_Command::GET_EEPROM:
            _ack_payload()[0] = (payload_len >= 1u && payload[0] < MAI2LIGHT_EEPROM_SIZE)
                ? _eeprom[payload[0]] : 0u;
            _ack_send(1u);
            break;
        case Mai2Light_Command::SET_ENABLE_RESPONSE:
            _flags.resp_enabled = true;   // 本命令不回应答, 只切换后续是否回
            break;
        case Mai2Light_Command::SET_DISABLE_RESPONSE:
            _flags.resp_enabled = false;
            break;
        case Mai2Light_Command::GET_BOARD_INFO: {
            // boardNo[9] = "15070-04" + 0xFF 终止, 随后 firmRevision=144。
            uint8_t* out = _ack_payload();
            memcpy(out, "15070-04", 8);
            out[8] = 0xFFu;
            out[9] = 144u;
            _ack_send(10u);
            break;
        }
        case Mai2Light_Command::GET_BOARD_STATUS: {
            uint8_t* out = _ack_payload();
            out[0] = 0u;   // timeoutStat
            out[1] = 1u;   // timeoutSec
            out[2] = 0u;   // pwmIo
            out[3] = 0u;   // fetTimeout
            _ack_send(4u);
            break;
        }
        case Mai2Light_Command::GET_FIRM_SUM:
            _ack_payload()[0] = 0u;   // 正常运行不会被调用, 回 0 即可
            _ack_payload()[1] = 0u;
            _ack_send(2u);
            break;
        case Mai2Light_Command::GET_PROTOCOL_VERSION: {
            uint8_t* out = _ack_payload();
            out[0] = 1u;   // appliMode: 绝不能为 0, 否则主机会进固件升级流程
            out[1] = 1u;   // major
            out[2] = 1u;   // minor
            _ack_send(3u);
            break;
        }
        default:
            _stats.unknown_cmds++;
            _ack_send(0u);   // 官方板对未知命令同样回空应答
            break;
    }
}

// ---------------- 命令实现 ----------------

inline void Mai2Light::_stage_unit(uint8_t index, const Mai2Light_RGB& color) {
    if (index >= MAI2LIGHT_NUM_LEDS) return;
    _stage[index] = color;
    _stage_mask |= (uint16_t)(1u << index);
}

inline void Mai2Light::_set_unit(uint8_t index, const Mai2Light_RGB& color) {
    if (index >= MAI2LIGHT_NUM_LEDS) return;
    _led[index].color = color;
}

void Mai2Light::_cmd_set_single() {
    const uint8_t* p = _req_payload();
    if (_req_payload_len() < 4u) {
        _ack_send(0u);
        return;
    }
    _stage_unit(p[0], Mai2Light_RGB(p[1], p[2], p[3]));
    _ack_send(0u);
}

void Mai2Light::_cmd_set_multi(bool fade) {
    const uint8_t* p = _req_payload();
    const uint8_t payload_len = _req_payload_len();
    if (payload_len < 6u) {
        _ack_send(0u);
        return;
    }

    uint8_t end = p[1];
    if (end == MULTI_END_ALL) end = MAI2LIGHT_NUM_LEDS;   // 到全部灯末尾
    const uint8_t first = (p[0] < MAI2LIGHT_NUM_LEDS) ? p[0] : (uint8_t)(MAI2LIGHT_NUM_LEDS - 1u);
    uint8_t last = (end > first) ? (uint8_t)(end - 1u) : first;
    if (last >= MAI2LIGHT_NUM_LEDS) last = MAI2LIGHT_NUM_LEDS - 1u;

    const Mai2Light_RGB color(p[3], p[4], p[5]);
    if (!fade) {
        // skip / speed 在本命令不生效(官方约定); 写缓冲并记为后续渐变的起始色。
        _multi_color = color;
        for (uint8_t i = first; i <= last; i++) _stage_unit(i, color);
        _fade.clear();
        _ack_send(0u);
        return;
    }

    const uint8_t speed = (payload_len >= 7u) ? p[6] : 0u;
    _fade.from = _multi_color;
    _fade.to = color;
    _fade.first = first;
    _fade.last = last;
    _fade.running = false;
    _fade.armed = true;      // 等 0x3C 触发开始
    _fade.start_ms = 0u;
    // speed=0 视为立即到位(避免除零)。
    _fade.end_ms = (speed != 0u) ? (FADE_BASE / (uint32_t)speed) * FADE_SCALE : 0u;
    _ack_send(0u);
}

void Mai2Light::_cmd_set_fet() {
    const uint8_t* p = _req_payload();
    if (_req_payload_len() < 3u) {
        _ack_send(0u);
        return;
    }
    // 框体灯只有白色, 值即亮度; 官方会连续多帧实现渐变, 故立即生效(不等 0x3C)。
    for (uint8_t i = 0; i < 3u; i++) {
        _set_unit((uint8_t)(MAI2LIGHT_BUTTON_LEDS + i), Mai2Light_RGB(p[i], p[i], p[i]));
    }
    _ack_send(0u);
}

void Mai2Light::_cmd_commit() {
    for (uint8_t i = 0; i < MAI2LIGHT_NUM_LEDS; i++) {
        if ((_stage_mask & (uint16_t)(1u << i)) != 0u) _led[i].color = _stage[i];
    }
    _stage_mask = 0u;

    if (_fade.armed) {
        const uint32_t now = now_ms();
        _fade.armed = false;
        _fade.running = true;
        _fade.start_ms = now;
        _fade.end_ms = now + _fade.end_ms;   // 0x33 时暂存的是时长, 此处转为绝对结束时刻
        _fade_step(now);
    }
    _ack_send(0u);
}

void Mai2Light::_fade_step(uint32_t now) {
    if (!_fade.running) return;

    uint8_t progress = 255u;
    if (_fade.end_ms > _fade.start_ms && now < _fade.end_ms) {
        progress = (uint8_t)(((uint64_t)(now - _fade.start_ms) * 255u) / (_fade.end_ms - _fade.start_ms));
    } else {
        _fade.running = false;
    }

    const Mai2Light_RGB blended(
        blend_channel(_fade.from.r, _fade.to.r, progress),
        blend_channel(_fade.from.g, _fade.to.g, progress),
        blend_channel(_fade.from.b, _fade.to.b, progress));
    for (uint8_t i = _fade.first; i <= _fade.last; i++) _set_unit(i, blended);
    if (!_fade.running) _multi_color = _fade.to;   // 渐变落点成为下一次渐变的起点
}

// ---------------- 应答 ----------------

inline void Mai2Light::_write_escaped(uint8_t value, uint8_t* out, uint8_t* out_len) const {
    if (value == MAI2LIGHT_SYNC_BYTE || value == MAI2LIGHT_ESCAPE_BYTE) {
        out[(*out_len)++] = MAI2LIGHT_ESCAPE_BYTE;
        out[(*out_len)++] = (uint8_t)(value - 1u);
    } else {
        out[(*out_len)++] = value;
    }
}

void Mai2Light::_ack_send(uint8_t payload_len, uint8_t status, uint8_t report) {
    if (!_flags.resp_enabled || _uart == nullptr) return;
    const uint8_t body_len = (uint8_t)(6u + payload_len);
    if (body_len > MAI2LIGHT_MAX_ACK_SIZE) return;

    _ack[0] = _rx[1];                      // dst = 请求方
    _ack[1] = _rx[0];                      // src = 本板
    _ack[2] = (uint8_t)(3u + payload_len); // status + command + report + payload
    _ack[3] = status;
    _ack[4] = _rx[3];                      // 回同一命令码
    _ack[5] = report;

    // sync + 转义体(最坏 2 倍) + sum
    uint8_t out[2u * MAI2LIGHT_MAX_ACK_SIZE + 2u];
    uint8_t out_len = 0u;
    uint8_t sum = 0u;
    out[out_len++] = MAI2LIGHT_SYNC_BYTE;
    for (uint8_t i = 0; i < body_len; i++) {
        sum = (uint8_t)(sum + _ack[i]);
        _write_escaped(_ack[i], out, &out_len);
    }
    out[out_len++] = sum;   // 校验和本身不转义(与官方板一致)

    pm_stage(PM_STAGE_LIGHT_ACK_FREE);
    if (_uart->get_tx_buffer_free_space() < out_len) return;   // 写不下就丢, 不阻塞
    pm_stage(PM_STAGE_LIGHT_ACK_WRITE);
    _uart->write_to_tx_buffer(out, out_len);
}
