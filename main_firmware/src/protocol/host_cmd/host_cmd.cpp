#include "host_cmd.h"
#include "../../service/config_manager/config_types.h"
#include "../../service/config_manager/config_manager.h"
#include "../../service/psoc_updater/psoc_updater.h"
#include "../../service/csd_config/csd_config.h"
#include "../../service/self_heal/self_heal.h"
#include "../psoc/psoc.h"
#include "../../service/sensor_link/sensor_link.h"
#include "../../service/hid_touch_mapper/hid_touch_mapper.h"
#include "../../service/led_map/led_map.h"
#include "../../service/persistence_txn/persistence_txn.h"
#include <cstring>
#include <cstdio>
#include <map>
#include <string>
#include <pico/stdlib.h>   // time_us_32(): 出向帧尾部时间戳的唯一时基(同 bus_core.cpp)
#ifdef PICO_PLATFORM
#include <hardware/watchdog.h>
#endif

// Forward declarations of handler functions
static void _handle_cfg_get(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_cfg_set(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);

// ★CFG_SET 不通知任何服务, 故按键前缀就地转发★
// ConfigManager 没有变更通知机制(见 keyboard.cpp:364 为此不得不 200ms 轮询两个开关)。
// hid.* 有 108 项, 轮询它们等于每秒上千次字符串查表, 纯浪费; 而"改了配置要生效"的时机
// 恰好只有 CFG_SET / CFG_SET_BATCH / RESET_DEFAULTS 三处, 在这里按前缀调一次 reload 即精确覆盖。
//
// ★键前缀 → 需要通知的服务, 是本文件唯一一张表★ 三处写入路径都只经 _notify_key_written /
// _notify_batch_written, 不各自列一份前缀名单 —— 那种写法一定会漂移出"CFG_SET 生效、批量保存不生效"
// 这类只在其中一条路上出现的 bug(led.ws_brightness 此前正是如此: 只有 500ms 轮询, 批量保存后
// 界面已显示新值而灯亮度要等下一个轮询窗才动)。
static inline void _notify_key_written(const std::string& key) {
    // rfind(prefix, 0)==0 是 C++17 下的 starts_with 等价写法(compare(0,n,..) 在越短的串上
    // 需要 pos<=size() 的前置条件, 这里用 rfind 免掉那层推理)。
    if (key.rfind("hid.", 0) == 0) {
        HidTouchMapper::getInstance()->reload();
        return;
    }
    // led.ws_brightness 是 LedMapService 唯一靠轮询取的 KV(led_map.cpp BRIGHTNESS_REFRESH_MS)。
    // 只认这一个键: led.ws_count* 改的是灯链长度(要重建 PIO 灯链), led.map* 走 LED_SET_MAP 的
    // apply_map() 本就即时生效, 都不该被这条"刷亮度"的通知捎带。
    if (key == "led.ws_brightness") {
        LedMapService::getInstance()->reload_brightness();
    }
}

// 批量写入的整批通知: 每类服务最多刷一次。逐键调 reload 会把全量刷新重复 N 遍(hid.* 有 108 项),
// 而 reload 本身就是全量的。
static inline void _notify_batch_written(const std::map<std::string, ConfigValue>& batch) {
    bool hid_dirty = false;
    bool led_brightness_dirty = false;
    for (const auto& kv : batch) {
        if (kv.first.rfind("hid.", 0) == 0) {
            hid_dirty = true;
        } else if (kv.first == "led.ws_brightness") {
            led_brightness_dirty = true;
        }
        if (hid_dirty && led_brightness_dirty) break;
    }
    if (hid_dirty) HidTouchMapper::getInstance()->reload();
    if (led_brightness_dirty) LedMapService::getInstance()->reload_brightness();
}
static void _handle_cfg_get_group(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_cfg_get_all(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_cfg_set_batch(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_save_config(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_reset_defaults(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);

namespace {
// ★流式遍历, 不再持有整表副本★
// 原实现在 CFG_GET_ALL 入口做 `entries = ConfigManager::get_all()` —— 按值返回即整表深拷贝:
// 约 319 项、每项含 std::string 键 + 含 std::string 的 ConfigValue, 折合约 41KB 堆与约 640 次
// malloc/free, 每来一条命令抖一次。core0 的栈区下界恰好就是堆顶(__StackLimit==__HeapLimit),
// 一旦别处有栈越界写坏了 arena, 这里就是最先踩到坏链表并 hardfault 的地方。
// 改为直接遍历 ConfigManager 的运行表, 用"上次发出的键"作游标, 续帧时 lower_bound 定位:
//   · 零拷贝、零堆抖动;
//   · 游标是值(std::string)而非迭代器, 不会因表结构变化而悬空 —— 迭代器方案在跨帧存活期间
//     若有人写配置就是未定义行为;
//   · 每片一次 O(log n) 定位, 相对一次 USB 往返可忽略。
struct CfgGetAllStreamState {
    uint8_t seq = 0;
    uint16_t cursor = 0;
    uint16_t total = 0;
    bool active = false;
    bool started = false;      // 是否已发出过至少一项(false ⇒ 从表头开始)
    std::string last_key;      // 已发出的最后一个键(下一片从它之后继续)

    void clear() {
        seq = 0;
        cursor = 0;
        total = 0;
        active = false;
        started = false;
        last_key.clear();
    }
};

CfgGetAllStreamState _cfg_get_all_stream;

uint16_t _encode_cfg_get_all_stream_frame(uint8_t* resp_buf) {
    if (!resp_buf || !_cfg_get_all_stream.active) return 0;

    // HostFrame 是 4102B, 而 core0 的栈区只有 8192B 且下界就是堆顶 ⇒ 绝不能放栈上。
    // 借 host_cmd 的共享响应工作帧(core0 单线程分发, 一帧处理完才取下一帧)。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.cmd = static_cast<uint8_t>(HostCmd::CFG_GET_ALL);
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = _cfg_get_all_stream.seq;
    resp.len = 0;

    const uint16_t header_len = _cfg_get_all_stream.cursor == 0 ? 2 : 0;
    if (header_len != 0) {
        resp.payload[resp.len++] = static_cast<uint8_t>(_cfg_get_all_stream.total);
        resp.payload[resp.len++] = static_cast<uint8_t>(_cfg_get_all_stream.total >> 8);
    }

    // 本片 payload 上限: 取 1KB 与协议上限的较小值(见 CFG_GET_ALL_FRAME_PAYLOAD_MAX)。
    constexpr uint16_t kFrameCap =
        (CFG_GET_ALL_FRAME_PAYLOAD_MAX < HOST_CMD_PAYLOAD_MAX)
            ? (uint16_t)CFG_GET_ALL_FRAME_PAYLOAD_MAX : (uint16_t)HOST_CMD_PAYLOAD_MAX;

    // 零拷贝遍历运行表; 游标 = 上次发出的键, lower_bound 定位其后继。
    const config_map_t& all = ConfigManager::runtime_map();
    config_map_t::const_iterator it =
        _cfg_get_all_stream.started ? all.upper_bound(_cfg_get_all_stream.last_key) : all.cbegin();

    const uint16_t cursor_before = _cfg_get_all_stream.cursor;
    while (it != all.cend()) {
        // ★空间硬闸★ 先算剩余可写字节, 绝不把负数/回绕值当 max_len 传下去。
        if (resp.len >= kFrameCap) break;              // 本片已满, 余下留给下一片
        const uint16_t room = (uint16_t)(kFrameCap - resp.len);

        const uint16_t entry_len = HostCmdCodec::encode_entry(
            it->second, it->first.c_str(), &resp.payload[resp.len], room);
        if (entry_len == 0) {
            // 装不下: 本片已有内容就收尾, 让下一片从这一项重新开始。
            if (resp.len > header_len) break;

            // 空片都装不下这一项 ⇒ 真的编码失败(单项超过 1KB 上限或类型异常), 终止流。
            const uint8_t seq = _cfg_get_all_stream.seq;
            _cfg_get_all_stream.clear();
            return HostCmdCodec::encode_nak(
                seq, HostCmdError::CONFIG_ERROR, "entry encode failed", resp_buf, HOST_CMD_RESP_BUF_MAX);
        }
        // encode_entry 已按 room 自限; 这里再断言一次, 越界即视为编码失败而不是继续累加。
        if ((uint32_t)resp.len + entry_len > kFrameCap) break;

        resp.len += entry_len;
        ++_cfg_get_all_stream.cursor;
        _cfg_get_all_stream.last_key = it->first;
        _cfg_get_all_stream.started = true;
        ++it;
    }

    // ★完成判据用"游标已走到表尾", 不用 cursor==total★
    // total 是开流那一刻的项数快照; 若流进行中有人写了配置(增/删项), cursor 可能永远追不上
    // total, 流就永远收不了尾。以迭代器是否耗尽为准则, 表怎么变都能正常结束。
    const bool exhausted = (it == all.cend());

    if (!exhausted && _cfg_get_all_stream.cursor == cursor_before) {
        // 一项都没进展且还没走完 ⇒ 无法推进(单项装不进空片等), 终止流而不是无限空转。
        const uint8_t seq = _cfg_get_all_stream.seq;
        _cfg_get_all_stream.clear();
        return HostCmdCodec::encode_nak(
            seq, HostCmdError::CONFIG_ERROR, "stream made no progress", resp_buf, HOST_CMD_RESP_BUF_MAX);
    }

    if (!exhausted) {
        resp.flags |= HOST_CMD_FLAG_STREAM;
    } else {
        _cfg_get_all_stream.clear();
    }

    return HostCmdCodec::encode_frame(resp, resp_buf, HOST_CMD_RESP_BUF_MAX);
}
}  // namespace

// ============ HostCmdCodec ============

void HostCmdCodec::init() {
    reset();
}

void HostCmdCodec::reset() {
    _state = RxState::FIND_SOF0;
    _header_pos = 0;
    _payload_len = 0;
    _payload_pos = 0;
    _crc_pos = 0;
    memset(_header, 0, sizeof(_header));
    memset(_payload, 0, sizeof(_payload));
    memset(_crc_bytes, 0, sizeof(_crc_bytes));
}

bool HostCmdCodec::feed_byte(uint8_t byte, HostFrame* out_frame) {
    if (!out_frame) return false;
    
    switch (_state) {
        case RxState::FIND_SOF0:
            if (byte == HOST_CMD_SOF0) {
                _state = RxState::FIND_SOF1;
            }
            return false;
        
        case RxState::FIND_SOF1:
            if (byte == HOST_CMD_SOF1) {
                _state = RxState::READ_HEADER;
                _header_pos = 0;
            } else {
                _state = RxState::FIND_SOF0;  // 重新查找
            }
            return false;
        
        case RxState::READ_HEADER:
            _header[_header_pos++] = byte;
            if (_header_pos >= 5) {
                // 读完头: cmd(0) flags(1) seq(2) len_lo(3) len_hi(4)
                _payload_len = (_header[4] << 8) | _header[3];
                
                // 校验 payload 长度
                if (_payload_len > HOST_CMD_PAYLOAD_MAX) {
                    reset();
                    return false;
                }
                
                _payload_pos = 0;
                _crc_pos = 0;
                
                if (_payload_len == 0) {
                    // 无 payload，直接读 CRC
                    _state = RxState::READ_CRC;
                } else {
                    _state = RxState::READ_PAYLOAD;
                }
            }
            return false;
        
        case RxState::READ_PAYLOAD:
            _payload[_payload_pos++] = byte;
            if (_payload_pos >= _payload_len) {
                _state = RxState::READ_CRC;
                _crc_pos = 0;
            }
            return false;
        
        case RxState::READ_CRC:
            _crc_bytes[_crc_pos++] = byte;
            if (_crc_pos >= 2) {
                return _check_frame_complete(out_frame);
            }
            return false;
        
        default:
            reset();
            return false;
    }
}

bool HostCmdCodec::_check_frame_complete(HostFrame* out_frame) {
    // 计算 CRC: cmd flags seq len_lo len_hi payload
    // 使用增量 CRC 来避免 4KB 栈缓冲
    uint16_t crc = HostCmdCrc16::crc16(_header, 5);  // 先算头
    if (_payload_len > 0) {
        crc = HostCmdCrc16::crc16(_payload, _payload_len, crc);  // 再算 payload
    }
    
    uint16_t crc_received = _crc_bytes[0] | (((uint16_t)_crc_bytes[1]) << 8);
    
    if (crc != crc_received) {
        // CRC 错误，重新查找
        reset();
        return false;
    }
    
    // CRC 通过，填充 HostFrame
    out_frame->cmd = _header[0];
    out_frame->flags = _header[1];
    out_frame->seq = _header[2];
    out_frame->len = _payload_len;
    memcpy(out_frame->payload, _payload, _payload_len);
    
    reset();
    return true;
}

uint16_t HostCmdCodec::_encode_into(uint8_t cmd, uint8_t flags, uint8_t seq,
                                    const uint8_t* payload, uint16_t len,
                                    uint8_t* out_buffer, uint16_t max_len) {
    if (!out_buffer || max_len < HOST_CMD_HEADER_SIZE + 2 + len) {
        return 0;
    }
    if (len > 0 && !payload) {
        return 0;
    }

    // ★设备时间戳唯一追加点★(设计与退化条件见 host_cmd.h 的 HOST_CMD_FLAG_TS 注释)。
    // 本函数是固件唯一的出向组帧实现 ⇒ 所有设备→主机的帧自动带戳, 新增命令无需任何改动。
    const bool with_ts =
        ((uint32_t)len + HOST_CMD_TS_SIZE <= HOST_CMD_PAYLOAD_MAX) &&
        ((uint32_t)max_len >= (uint32_t)HOST_CMD_HEADER_SIZE + 2u + len + HOST_CMD_TS_SIZE);
    const uint16_t body_len =
        with_ts ? (uint16_t)(len + HOST_CMD_TS_SIZE) : len;

    uint16_t pos = 0;
    
    // SOF
    out_buffer[pos++] = HOST_CMD_SOF0;
    out_buffer[pos++] = HOST_CMD_SOF1;
    
    // 头部: cmd flags seq len_lo len_hi
    out_buffer[pos++] = cmd;
    out_buffer[pos++] = with_ts ? (uint8_t)(flags | HOST_CMD_FLAG_TS) : flags;
    out_buffer[pos++] = seq;
    out_buffer[pos++] = body_len & 0xFF;
    out_buffer[pos++] = (body_len >> 8) & 0xFF;
    
    // payload
    if (len > 0) {
        memcpy(&out_buffer[pos], payload, len);
        pos += len;
    }
    // 时间戳尾巴(u32 LE), 紧跟原 payload 之后。
    if (with_ts) {
        const uint32_t t_us = time_us_32();
        out_buffer[pos++] = (uint8_t)(t_us & 0xFF);
        out_buffer[pos++] = (uint8_t)((t_us >> 8) & 0xFF);
        out_buffer[pos++] = (uint8_t)((t_us >> 16) & 0xFF);
        out_buffer[pos++] = (uint8_t)((t_us >> 24) & 0xFF);
    }
    
    // CRC: 计算 cmd flags seq len_lo len_hi payload(含时间戳尾巴)
    // 使用增量 CRC 避免 4KB 栈缓冲; body 直接读 out_buffer, 不再单独走 frame.payload,
    // 否则时间戳尾巴会漏出校验范围。
    uint16_t crc = HostCmdCrc16::crc16((const uint8_t*)(&out_buffer[2]), 5);  // header starts at offset 2
    if (body_len > 0) {
        crc = HostCmdCrc16::crc16(&out_buffer[HOST_CMD_HEADER_SIZE], body_len, crc);
    }
    
    out_buffer[pos++] = crc & 0xFF;
    out_buffer[pos++] = (crc >> 8) & 0xFF;
    
    return pos;
}

uint16_t HostCmdCodec::encode_frame(const HostFrame& frame, uint8_t* out_buffer, uint16_t max_len) {
    // ★payload 长度硬钳制★ frame.len 由各 handler 自增累加得出; 一旦某处算错就会让
    // memcpy 读越 payload 尾、并按超长 body 写穿 out_buffer。这里做最后一道闸: 协议上
    // len 本就不可能 > HOST_CMD_PAYLOAD_MAX, 故越界即视为编码失败而不是"尽力发出去"。
    if (frame.len > HOST_CMD_PAYLOAD_MAX) return 0;
    return _encode_into(frame.cmd, frame.flags, frame.seq, frame.payload, frame.len,
                        out_buffer, max_len);
}

HostFrame& HostCmdCodec::resp_frame() {
    // 函数内静态: 4102B 落 .bss 而非 core0 的 8KB 栈(见 host_cmd.h 的声明注释)。
    static HostFrame frame;
    return frame;
}

uint16_t HostCmdCodec::encode_ack(uint8_t seq, uint8_t* out_buffer, uint16_t max_len) {
    // 空 payload, 直接组帧: 不再借道 HostFrame(那会在调用者的帧之上再压 4102B)。
    return _encode_into((uint8_t)HostCmd::ACK, HOST_CMD_FLAG_RESPONSE, seq,
                        nullptr, 0, out_buffer, max_len);
}

uint16_t HostCmdCodec::encode_nak(uint8_t seq, HostCmdError err_code, const char* msg,
                                   uint8_t* out_buffer, uint16_t max_len) {
    // payload: err_code(1B) + msg(可选)
    // ★局部缓冲只 256B★ NAK 消息全是固定短字面量或配置键名(最长的键也远短于 64), 旧上限
    // HOST_CMD_PAYLOAD_MAX-1(4095) 从未被用到, 却逼着本函数持有一个 4102B 的 HostFrame ——
    // 而它恰恰被"已经持有一个栈上 HostFrame 的 handler"到处调用, 两帧相加 8204B > 8192B 栈。
    // 消息超长按 255 截断: NAK 只是诊断文本, 截断不影响错误码语义。
    uint8_t buf[256];
    buf[0] = (uint8_t)err_code;
    uint16_t len = 1;

    if (msg) {
        size_t msg_len = strlen(msg);
        if (msg_len > sizeof(buf) - 1) {
            msg_len = sizeof(buf) - 1;
        }
        memcpy(&buf[1], msg, msg_len);
        len = (uint16_t)(1u + msg_len);
    }

    return _encode_into((uint8_t)HostCmd::NAK,
                        HOST_CMD_FLAG_RESPONSE | HOST_CMD_FLAG_NAK_ERR, seq,
                        buf, len, out_buffer, max_len);
}

// ============ HostCmdDispatcher ============

HostCmdDispatcher* HostCmdDispatcher::_instance = nullptr;

HostCmdDispatcher::HostCmdDispatcher() {
    memset(_handler_registered, 0, sizeof(_handler_registered));
    
    // 注册内置处理器
    register_handler(HostCmd::HELLO, _handle_hello);
    register_handler(HostCmd::PING, _handle_ping);
    
    // 注册配置命令处理器（#5b）
    register_handler(HostCmd::CFG_GET, _handle_cfg_get);
    register_handler(HostCmd::CFG_SET, _handle_cfg_set);
    register_handler(HostCmd::CFG_GET_GROUP, _handle_cfg_get_group);
    register_handler(HostCmd::CFG_GET_ALL, _handle_cfg_get_all);
    register_handler(HostCmd::CFG_SET_BATCH, _handle_cfg_set_batch);
    register_handler(HostCmd::SAVE_CONFIG, _handle_save_config);
    register_handler(HostCmd::RESET_DEFAULTS, _handle_reset_defaults);
}

HostCmdDispatcher* HostCmdDispatcher::getInstance() {
    if (!_instance) {
        _instance = new HostCmdDispatcher();
    }
    return _instance;
}

void HostCmdDispatcher::register_handler(HostCmd cmd, CmdHandler handler) {
    uint8_t cmd_code = (uint8_t)cmd;
    if (handler) {
        _handlers[cmd_code] = handler;
        _handler_registered[cmd_code] = true;
    }
}

void HostCmdDispatcher::dispatch(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    if (!resp_buf || !resp_len) return;
    
    *resp_len = 0;

    // SWD bring-up owns the PSoC until run() releases the session. _keepalive()
    // must still service HELLO/DEVICE_INFO so the host can establish USB state,
    // but every other command can touch runtime services or schedule work that
    // conflicts with the active SWD transaction. Return a retryable terminal
    // instead of silently dropping it, so the host never converts startup work
    // into a command timeout.
    const PsocBringupStage stage = PsocUpdater::getInstance()->report().last_stage;
    const bool swd_session_active =
        stage >= PsocBringupStage::SWD_READY && stage < PsocBringupStage::RUN;
    if (swd_session_active && frame.cmd != static_cast<uint8_t>(HostCmd::HELLO) &&
        frame.cmd != static_cast<uint8_t>(HostCmd::PING)) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
                                              "psoc bring-up active", resp_buf,
                                              HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    uint8_t cmd_code = frame.cmd;
    
    // 查找对应的处理器
    if (_handler_registered[cmd_code]) {
        _handlers[cmd_code](frame, resp_buf, resp_len);
    } else {
        // 未实现的命令，回 NAK
        _handle_not_implemented(frame, resp_buf, resp_len);
    }
}

bool HostCmdDispatcher::has_pending_stream() const {
    return _cfg_get_all_stream.active;
}

bool HostCmdDispatcher::next_stream_frame(uint8_t* resp_buf, uint16_t* resp_len) {
    if (!resp_len) return false;

    *resp_len = _encode_cfg_get_all_stream_frame(resp_buf);
    return *resp_len > 0;
}

void HostCmdDispatcher::clear_pending_stream() {
    _cfg_get_all_stream.clear();
}

void HostCmdDispatcher::_handle_hello(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // ★新会话只静默遗留输出，不得中断扫描恢复★：扫描已独占 PSoC 的 gain/div 与快照快路；
    // 若在 HELLO 里走 stop()，会取消会话并把 PSoC 留在恢复长操作的竞争路径上。命令响应本身
    // 已由 UsbComm::begin_command_response() 抑制异步流，扫描中不需要停 PSoC 即可优先发送 DEVICE_INFO。
    SensorLink::getInstance()->prepare_host_session();

    // Bytes 0..14 preserve the legacy DEVICE_INFO layout. A versioned report follows.
    // 借共享工作帧: 4102B 绝不能放 core0 的 8KB 栈(见 HostCmdCodec::resp_frame 注释)。
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.cmd = (uint8_t)HostCmd::DEVICE_INFO;
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.len = 0;

    const PsocBringupReport& report = PsocUpdater::getInstance()->report();
    auto append_u16 = [&resp](uint16_t value) {
        resp.payload[resp.len++] = static_cast<uint8_t>(value);
        resp.payload[resp.len++] = static_cast<uint8_t>(value >> 8);
    };
    auto append_u32 = [&resp](uint32_t value) {
        resp.payload[resp.len++] = static_cast<uint8_t>(value);
        resp.payload[resp.len++] = static_cast<uint8_t>(value >> 8);
        resp.payload[resp.len++] = static_cast<uint8_t>(value >> 16);
        resp.payload[resp.len++] = static_cast<uint8_t>(value >> 24);
    };

    append_u16(0x0001);                         // protocol_version
    append_u32(report.rp_firmware_version);     // RP2040 firmware version
    resp.payload[resp.len++] = 36;              // capsense_channels
    append_u32(0x00000007);                     // telemetry/config/binding capabilities
    append_u16(report.psoc_generation);
    resp.payload[resp.len++] = report.link_ok ? 1 : 0;
    resp.payload[resp.len++] = report.snapshot_valid ? 1 : 0;

    const uint16_t report_start = resp.len;
    resp.payload[resp.len++] = PSOC_BRINGUP_REPORT_VERSION;
    resp.payload[resp.len++] = 0;  // report length, filled after serialization
    append_u32(report.rp_build_id);
    append_u32(report.embedded_psoc_version);
    resp.payload[resp.len++] = static_cast<uint8_t>(report.last_stage);
    resp.payload[resp.len++] = static_cast<uint8_t>(report.failure_stage);
    append_u16(report.flags());
    append_u32(report.actual_silicon_id);
    append_u32(report.idcode);
    resp.payload[resp.len++] = report.chip_protection;
    append_u32(report.last_srom_status);
    append_u32(report.acquire_status);
    append_u32(report.acquire_sysreq);
    append_u32(report.acquire_delay);
    append_u16(report.fail_row);
    append_u32(report.fail_addr);
    append_u32(report.verify_read);
    append_u32(report.verify_expect);
    append_u32(report.erase_status);
    append_u32(report.program_status);
    append_u32(report.checksum_srom);
    append_u32(report.checksum_value);
    append_u32(report.clock_select);
    append_u32(report.clock_imo_select);
    append_u32(report.clock_trim1);
    append_u32(report.clock_trim2);
    append_u32(report.clock_trim3);
    append_u32(report.erase_flash_sum);
    append_u32(report.erase_flash_or);
    append_u32(report.erase_first_nonzero_addr);
    append_u32(report.erase_first_nonzero_value);
    append_u32(report.erase_words_read);
    // ★CSD 运行态标志(尾部追加, 向后兼容: 旧上位机按 report_length 解析会自然忽略)★
    // bit0 = 上次"恢复默认"因 PSoC 采样异常(railed/停滞)拒绝固化基线 → 提示改用 PSoC 救砖。
    resp.payload[resp.len++] = CsdConfig::getInstance()->baseline_untrusted() ? 0x01u : 0x00u;
    // ★自持恢复事件计数(尾部追加, 旧上位机按 report_length 自然忽略)★: STREAM 推送是发射后不管的,
    // 单靠它无法证明"一条都没漏"。这里给出累计/待发/被丢弃三个计数, 上位机可与自己收到的条数核对,
    // 发现漏帧就明确提示, 而不是默认"没收到就等于没发生"。
    {
        SelfHeal* sh = SelfHeal::getInstance();
        const uint16_t sh_total = sh->total();
        const uint16_t sh_dropped = sh->dropped();
        resp.payload[resp.len++] = static_cast<uint8_t>(sh_total);
        resp.payload[resp.len++] = static_cast<uint8_t>(sh_total >> 8);
        resp.payload[resp.len++] = static_cast<uint8_t>(sh_dropped);
        resp.payload[resp.len++] = static_cast<uint8_t>(sh_dropped >> 8);
        resp.payload[resp.len++] = sh->empty() ? 0x00u : 0x01u;
    }
    // CSD 真正运行模式由 RP2040 store 持有；追加在诊断尾部，旧上位机按 report_length 自然忽略。
    resp.payload[resp.len++] = CsdConfig::getInstance()->mode();
    // PSoC 运行态 SWD 调试块：bring-up 期间 SWD 会话正由 updater 独占，HELLO 只能返回零快照；
    // run()/release_swd() 完成后再按既有路径读取，避免 DEVICE_INFO 反向扰动正在进行的校验/烧录。
    uint32_t spi_dbg_counters[SwdProgrammer::DEBUG_COUNTER_WORDS] = {};
    Psoc* psoc = Psoc::getInstance();
    const PsocBringupStage stage = report.last_stage;
    const bool swd_session_active =
        stage >= PsocBringupStage::SWD_READY && stage < PsocBringupStage::RUN;
    const uint8_t debug_status = swd_session_active
        ? 0u : static_cast<uint8_t>(psoc->psoc_debug_status());
    const uint32_t debug_block_addr = swd_session_active ? 0u : psoc->psoc_debug_block_addr();
    if (!swd_session_active) (void)psoc->psoc_debug_counters(spi_dbg_counters);
    resp.payload[resp.len++] = debug_status;
    append_u32(debug_block_addr);
    for (uint8_t i = 0; i < SwdProgrammer::DEBUG_COUNTER_WORDS; ++i) append_u32(spi_dbg_counters[i]);
    resp.payload[report_start + 1] = static_cast<uint8_t>(resp.len - report_start);

    *resp_len = HostCmdCodec::encode_frame(resp, resp_buf, HOST_CMD_RESP_BUF_MAX);
}

void HostCmdDispatcher::_handle_ping(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // PING → ACK
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
}

void HostCmdDispatcher::_handle_not_implemented(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // 未实现 → NAK(NOT_IMPLEMENTED)
    *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::NOT_IMPLEMENTED, 
                                          "TODO: #5b/#5c", resp_buf, HOST_CMD_RESP_BUF_MAX);
}

// ============ Entry Encode/Decode ============

uint16_t HostCmdCodec::encode_entry(const ConfigValue& val, const char* key, uint8_t* out_buf, uint16_t max_len) {
    if (!out_buf || !key || max_len < 4) {
        return 0;
    }
    
    uint16_t pos = 0;
    
    // type(u8)
    out_buf[pos++] = (uint8_t)val.type;
    
    // has_range(u8)
    out_buf[pos++] = val.has_range ? 1 : 0;
    
    // key_len(u8) + key
    size_t key_len = strlen(key);
    if (key_len > 255 || pos + 1 + key_len > max_len) {
        return 0;
    }
    out_buf[pos++] = (uint8_t)key_len;
    memcpy(&out_buf[pos], key, key_len);
    pos += key_len;
    
    // value (按类型编码，LE)
    switch (val.type) {
        case ConfigValueType::BOOL: {
            if (pos + 1 > max_len) return 0;
            out_buf[pos++] = val.bool_val ? 1 : 0;
            break;
        }
        case ConfigValueType::INT8: {
            if (pos + 1 > max_len) return 0;
            out_buf[pos++] = (uint8_t)val.int8_val;
            break;
        }
        case ConfigValueType::UINT8: {
            if (pos + 1 > max_len) return 0;
            out_buf[pos++] = val.uint8_val;
            break;
        }
        case ConfigValueType::UINT16: {
            if (pos + 2 > max_len) return 0;
            out_buf[pos++] = val.uint16_val & 0xFF;
            out_buf[pos++] = (val.uint16_val >> 8) & 0xFF;
            break;
        }
        case ConfigValueType::UINT32: {
            if (pos + 4 > max_len) return 0;
            out_buf[pos++] = val.uint32_val & 0xFF;
            out_buf[pos++] = (val.uint32_val >> 8) & 0xFF;
            out_buf[pos++] = (val.uint32_val >> 16) & 0xFF;
            out_buf[pos++] = (val.uint32_val >> 24) & 0xFF;
            break;
        }
        case ConfigValueType::FLOAT: {
            if (pos + 4 > max_len) return 0;
            uint32_t* fptr = (uint32_t*)&val.float_val;
            out_buf[pos++] = *fptr & 0xFF;
            out_buf[pos++] = (*fptr >> 8) & 0xFF;
            out_buf[pos++] = (*fptr >> 16) & 0xFF;
            out_buf[pos++] = (*fptr >> 24) & 0xFF;
            break;
        }
        case ConfigValueType::STRING: {
            size_t slen = val.string_val.length();
            if (slen > 65535 || pos + 2 + slen > max_len) return 0;
            out_buf[pos++] = slen & 0xFF;
            out_buf[pos++] = (slen >> 8) & 0xFF;
            memcpy(&out_buf[pos], val.string_val.c_str(), slen);
            pos += slen;
            break;
        }
        default:
            return 0;
    }
    
    // min/max if has_range
    if (val.has_range) {
        switch (val.type) {
            case ConfigValueType::INT8: {
                if (pos + 1 > max_len) return 0;
                out_buf[pos++] = (uint8_t)val.min_val.int8_min;
                if (pos + 1 > max_len) return 0;
                out_buf[pos++] = (uint8_t)val.max_val.int8_max;
                break;
            }
            case ConfigValueType::UINT8: {
                if (pos + 2 > max_len) return 0;
                out_buf[pos++] = val.min_val.uint8_min;
                out_buf[pos++] = val.max_val.uint8_max;
                break;
            }
            case ConfigValueType::UINT16: {
                if (pos + 4 > max_len) return 0;
                out_buf[pos++] = val.min_val.uint16_min & 0xFF;
                out_buf[pos++] = (val.min_val.uint16_min >> 8) & 0xFF;
                out_buf[pos++] = val.max_val.uint16_max & 0xFF;
                out_buf[pos++] = (val.max_val.uint16_max >> 8) & 0xFF;
                break;
            }
            case ConfigValueType::UINT32: {
                if (pos + 8 > max_len) return 0;
                out_buf[pos++] = val.min_val.uint32_min & 0xFF;
                out_buf[pos++] = (val.min_val.uint32_min >> 8) & 0xFF;
                out_buf[pos++] = (val.min_val.uint32_min >> 16) & 0xFF;
                out_buf[pos++] = (val.min_val.uint32_min >> 24) & 0xFF;
                out_buf[pos++] = val.max_val.uint32_max & 0xFF;
                out_buf[pos++] = (val.max_val.uint32_max >> 8) & 0xFF;
                out_buf[pos++] = (val.max_val.uint32_max >> 16) & 0xFF;
                out_buf[pos++] = (val.max_val.uint32_max >> 24) & 0xFF;
                break;
            }
            case ConfigValueType::FLOAT: {
                if (pos + 8 > max_len) return 0;
                uint32_t *min_ptr = (uint32_t*)&val.min_val.float_min;
                uint32_t *max_ptr = (uint32_t*)&val.max_val.float_max;
                out_buf[pos++] = *min_ptr & 0xFF;
                out_buf[pos++] = (*min_ptr >> 8) & 0xFF;
                out_buf[pos++] = (*min_ptr >> 16) & 0xFF;
                out_buf[pos++] = (*min_ptr >> 24) & 0xFF;
                out_buf[pos++] = *max_ptr & 0xFF;
                out_buf[pos++] = (*max_ptr >> 8) & 0xFF;
                out_buf[pos++] = (*max_ptr >> 16) & 0xFF;
                out_buf[pos++] = (*max_ptr >> 24) & 0xFF;
                break;
            }
            default:
                break;
        }
    }
    
    return pos;
}

uint16_t HostCmdCodec::decode_entry(const uint8_t* in_buf, uint16_t in_len, std::string* out_key, ConfigValue* out_val) {
    if (!in_buf || !out_key || !out_val || in_len < 4) {
        return 0;
    }
    
    uint16_t pos = 0;
    
    // type(u8)
    if (pos + 1 > in_len) return 0;
    ConfigValueType type = (ConfigValueType)in_buf[pos++];
    
    // has_range(u8)
    if (pos + 1 > in_len) return 0;
    bool has_range = (in_buf[pos++] != 0);
    
    // key_len(u8) + key
    if (pos + 1 > in_len) return 0;
    uint8_t key_len = in_buf[pos++];
    if (pos + key_len > in_len) return 0;
    out_key->assign((const char*)&in_buf[pos], key_len);
    pos += key_len;
    
    // value (按类型解码，LE)
    switch (type) {
        case ConfigValueType::BOOL: {
            if (pos + 1 > in_len) return 0;
            *out_val = ConfigValue(in_buf[pos++] != 0);
            break;
        }
        case ConfigValueType::INT8: {
            if (pos + 1 > in_len) return 0;
            int8_t val = (int8_t)in_buf[pos++];
            *out_val = ConfigValue(val);
            break;
        }
        case ConfigValueType::UINT8: {
            if (pos + 1 > in_len) return 0;
            uint8_t val = in_buf[pos++];
            *out_val = ConfigValue(val);
            break;
        }
        case ConfigValueType::UINT16: {
            if (pos + 2 > in_len) return 0;
            uint16_t val = in_buf[pos] | (((uint16_t)in_buf[pos+1]) << 8);
            pos += 2;
            *out_val = ConfigValue(val);
            break;
        }
        case ConfigValueType::UINT32: {
            if (pos + 4 > in_len) return 0;
            uint32_t val = in_buf[pos] | (((uint32_t)in_buf[pos+1]) << 8) | 
                          (((uint32_t)in_buf[pos+2]) << 16) | (((uint32_t)in_buf[pos+3]) << 24);
            pos += 4;
            *out_val = ConfigValue(val);
            break;
        }
        case ConfigValueType::FLOAT: {
            if (pos + 4 > in_len) return 0;
            uint32_t ival = in_buf[pos] | (((uint32_t)in_buf[pos+1]) << 8) | 
                           (((uint32_t)in_buf[pos+2]) << 16) | (((uint32_t)in_buf[pos+3]) << 24);
            pos += 4;
            float fval;
            memcpy(&fval, &ival, sizeof(float));
            *out_val = ConfigValue(fval);
            break;
        }
        case ConfigValueType::STRING: {
            if (pos + 2 > in_len) return 0;
            uint16_t slen = in_buf[pos] | (((uint16_t)in_buf[pos+1]) << 8);
            pos += 2;
            if (pos + slen > in_len) return 0;
            out_val->type = ConfigValueType::STRING;
            out_val->string_val.assign((const char*)&in_buf[pos], slen);
            pos += slen;
            break;
        }
        default:
            return 0;
    }
    
    // min/max if has_range
    if (has_range) {
        out_val->has_range = true;
        switch (type) {
            case ConfigValueType::INT8: {
                if (pos + 2 > in_len) return 0;
                out_val->min_val.int8_min = (int8_t)in_buf[pos++];
                out_val->max_val.int8_max = (int8_t)in_buf[pos++];
                break;
            }
            case ConfigValueType::UINT8: {
                if (pos + 2 > in_len) return 0;
                out_val->min_val.uint8_min = in_buf[pos++];
                out_val->max_val.uint8_max = in_buf[pos++];
                break;
            }
            case ConfigValueType::UINT16: {
                if (pos + 4 > in_len) return 0;
                out_val->min_val.uint16_min = in_buf[pos] | (((uint16_t)in_buf[pos+1]) << 8);
                pos += 2;
                out_val->max_val.uint16_max = in_buf[pos] | (((uint16_t)in_buf[pos+1]) << 8);
                pos += 2;
                break;
            }
            case ConfigValueType::UINT32: {
                if (pos + 8 > in_len) return 0;
                out_val->min_val.uint32_min = in_buf[pos] | (((uint32_t)in_buf[pos+1]) << 8) | 
                                             (((uint32_t)in_buf[pos+2]) << 16) | (((uint32_t)in_buf[pos+3]) << 24);
                pos += 4;
                out_val->max_val.uint32_max = in_buf[pos] | (((uint32_t)in_buf[pos+1]) << 8) | 
                                             (((uint32_t)in_buf[pos+2]) << 16) | (((uint32_t)in_buf[pos+3]) << 24);
                pos += 4;
                break;
            }
            case ConfigValueType::FLOAT: {
                if (pos + 8 > in_len) return 0;
                uint32_t min_i = in_buf[pos] | (((uint32_t)in_buf[pos+1]) << 8) | 
                                (((uint32_t)in_buf[pos+2]) << 16) | (((uint32_t)in_buf[pos+3]) << 24);
                pos += 4;
                float min_f;
                memcpy(&min_f, &min_i, sizeof(float));
                out_val->min_val.float_min = min_f;
                
                uint32_t max_i = in_buf[pos] | (((uint32_t)in_buf[pos+1]) << 8) | 
                                (((uint32_t)in_buf[pos+2]) << 16) | (((uint32_t)in_buf[pos+3]) << 24);
                pos += 4;
                float max_f;
                memcpy(&max_f, &max_i, sizeof(float));
                out_val->max_val.float_max = max_f;
                break;
            }
            default:
                break;
        }
    }
    
    return pos;
}

// ============ CFG_* Command Handlers ============

static void _handle_cfg_get(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_GET(0x10): payload = key 原始字节
    // 响应: Entry(1 个)
    
    if (frame.len == 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "key required", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    std::string key((const char*)frame.payload, frame.len);
    if (!ConfigManager::has_key(key)) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "key not found", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    ConfigValue val = ConfigManager::get(key);
    
    // 构建响应帧(共享工作帧, 不上栈; 见 HostCmdCodec::resp_frame 注释)
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.cmd = (uint8_t)HostCmd::CFG_GET;
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.len = HostCmdCodec::encode_entry(val, key.c_str(), resp.payload, HOST_CMD_PAYLOAD_MAX);
    
    if (resp.len == 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::CONFIG_ERROR, "entry encode failed", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    *resp_len = HostCmdCodec::encode_frame(resp, resp_buf, HOST_CMD_RESP_BUF_MAX);
}

static void _handle_cfg_set(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_SET(0x11): payload = Entry
    // 响应: ACK 或 NAK
    
    std::string key;
    ConfigValue val;
    uint16_t consumed = HostCmdCodec::decode_entry(frame.payload, frame.len, &key, &val);
    
    if (consumed == 0 || key.empty()) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "entry decode failed", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    if (!ConfigManager::has_key(key)) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "key not found", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    // 类型必须一致
    ConfigValue existing = ConfigManager::get(key);
    if (existing.type != val.type) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "type mismatch", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    // set 会自动 clamp
    ConfigManager::set(key, val);
    // 写 RAM 影子后即时让持有者刷新(flash 落地仍统一由 SAVE_CONFIG 触发, 与 kbd 各 SET 同口径)。
    _notify_key_written(key);
    
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
}

static void _handle_cfg_get_group(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_GET_GROUP(0x12): payload = prefix
    // 响应: count(u16 LE) + Entry×count
    
    std::string prefix((const char*)frame.payload, frame.len);
    auto group = ConfigManager::get_group(prefix);
    
    // 共享工作帧, 不上栈(见 HostCmdCodec::resp_frame 注释)
    HostFrame& resp = HostCmdCodec::resp_frame();
    resp.cmd = (uint8_t)HostCmd::CFG_GET_GROUP;
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.len = 0;
    
    // count
    uint16_t count = group.size();
    resp.payload[resp.len++] = count & 0xFF;
    resp.payload[resp.len++] = (count >> 8) & 0xFF;
    
    // entries
    for (auto& kv : group) {
        uint16_t entry_len = HostCmdCodec::encode_entry(kv.second, kv.first.c_str(), 
                                                         &resp.payload[resp.len], 
                                                         HOST_CMD_PAYLOAD_MAX - resp.len);
        if (entry_len == 0) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::CONFIG_ERROR, 
                                                 "entry encode failed", resp_buf, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        resp.len += entry_len;
    }
    
    *resp_len = HostCmdCodec::encode_frame(resp, resp_buf, HOST_CMD_RESP_BUF_MAX);
}

static void _handle_cfg_get_all(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_GET_ALL(0x13): 空 payload
    // 响应流: 首帧为 count(u16 LE) + Entry×N，后续帧为 Entry×N。
    // 每帧只在 Entry 边界切分；中间帧带 RESPONSE|STREAM，末帧仅 RESPONSE。
    // ★不再整表拷贝★ total 只取一次项数用于首片头部; 遍历直接走运行表(见 CfgGetAllStreamState 注释)。
    _cfg_get_all_stream.clear();
    _cfg_get_all_stream.seq = frame.seq;
    _cfg_get_all_stream.total = static_cast<uint16_t>(ConfigManager::runtime_map().size());
    _cfg_get_all_stream.active = true;
    *resp_len = _encode_cfg_get_all_stream_frame(resp_buf);
}

static void _handle_cfg_set_batch(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_SET_BATCH(0x14): payload = count(u16) + Entry×count
    // 响应: ACK(全成功) 或 NAK(首个失败 key)
    
    if (frame.len < 2) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "count required", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    
    uint16_t count = frame.payload[0] | (((uint16_t)frame.payload[1]) << 8);
    uint16_t pos = 2;
    
    std::map<std::string, ConfigValue> batch;
    
    // 解码所有 entry
    for (uint16_t i = 0; i < count; i++) {
        if (pos >= frame.len) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "incomplete entry", resp_buf, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        
        std::string key;
        ConfigValue val;
        uint16_t entry_len = HostCmdCodec::decode_entry(&frame.payload[pos], frame.len - pos, &key, &val);
        
        if (entry_len == 0 || key.empty()) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "entry decode failed", resp_buf, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        
        if (!ConfigManager::has_key(key)) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, key.c_str(), resp_buf, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        
        // 类型检查
        ConfigValue existing = ConfigManager::get(key);
        if (existing.type != val.type) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, key.c_str(), resp_buf, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        
        batch[key] = val;
        pos += entry_len;
    }
    
    // 全部有效，批量设置
    ConfigManager::set_batch(batch);
    // 写 RAM 影子后即时让持有者刷新(与 CFG_SET 同一张前缀表; flash 落地仍由 SAVE_CONFIG 触发)。
    _notify_batch_written(batch);
    
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
}

static void _handle_save_config(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // SAVE_CONFIG is a deferred completion barrier. The response is produced by
    // UsbComm only after NvStore has verified every dirty region by readback.
    PersistenceTxn* txn = PersistenceTxn::getInstance();
    const PersistenceTxn::Result accepted = txn->begin(frame.seq);
    if (accepted == PersistenceTxn::Result::BUSY) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
            "another save transaction is active", resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // The main-loop transaction owner consumes this one-time request before it
    // copies service snapshots into NvStore. Replaying the same sequence only
    // observes the original terminal and cannot start a second write.
    *resp_len = 0u;
}

// PSoC 重启请求(定义于 hal_usb.cpp): 置 1 → 主循环脉冲 XRES 重启 PSoC。
extern volatile uint8_t g_psoc_reboot_request;

static void _handle_reset_defaults(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // RESET_DEFAULTS(0x0F): 恢复默认。除普通 config 外, 一并清空 CSD 参数 store 并重启 PSoC:
    // 清 store 后 PSoC 启动 provisioning 跳过参数下发 → 用其生成的出厂默认(已验证 180Hz 正常),
    // 从而把被调崩(railed/降速)的 CSD 恢复到可用基线。
    // 清空整个 CSD store 是"用户所有逐通道调参就此消失"的破坏性动作, 本来就该留痕上报;
    // 同时它也是判定"RESET_DEFAULTS 到底有没有真的执行"的唯一可观测证据 —— 实测出现过
    // "上位机报 PASS, 但设备端 store 模式仍是 SEMI、PSoC 也没重启"的情况, 缺了这个计数就只能靠猜。
    SelfHeal::getInstance()->note(SH_STORE_CLEARED, 1u);
    ConfigManager::reset_to_defaults();
    CsdConfig* csd = CsdConfig::getInstance();
    csd->clear();              // 清 store → PSoC 重启后以出厂强制好全局(增益4/目标85)自动校准, 不再被坏全局污染
    csd->request_recapture();  // 就绪后回读校准好的默认 → 切 SEMI 快速基线 → 持久化(正常半自动基线=默认)
    g_psoc_reboot_request = 1u;          // 重启 PSoC, 使其以出厂默认重新初始化并自动校准 CSD
    // hid.* 已随 reset_to_defaults 回到"全不启用 + 网格默认坐标", 无条件刷新点位表:
    // 不刷的话映射器还按旧锚点输出, 与设备真值不一致(且用户看到的是已恢复默认的界面)。
    HidTouchMapper::getInstance()->reload();
    // led.ws_brightness 同样已回默认值: 不刷新则灯链亮度还停在恢复前的值, 最长要等 500ms 轮询窗
    // (而 XRES 之后的 PSoC 重初始化期间主循环很忙, 实际可见延迟更长)。
    LedMapService::getInstance()->reload_brightness();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
}
