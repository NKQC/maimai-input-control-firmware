#include "host_cmd.h"
#include "../../service/config_manager/config_types.h"
#include "../../service/config_manager/config_manager.h"
#include "../../service/psoc_updater/psoc_updater.h"
#include "../../service/csd_config/csd_config.h"
#include "../psoc/psoc.h"
#include "../../service/sensor_link/sensor_link.h"
#include <cstring>
#include <cstdio>
#include <string>

// Forward declarations of handler functions
static void _handle_cfg_get(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_cfg_set(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_cfg_get_group(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_cfg_get_all(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_cfg_set_batch(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_save_config(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
static void _handle_reset_defaults(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);

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

uint16_t HostCmdCodec::encode_frame(const HostFrame& frame, uint8_t* out_buffer, uint16_t max_len) {
    if (!out_buffer || max_len < HOST_CMD_HEADER_SIZE + 2 + frame.len) {
        return 0;
    }
    
    uint16_t pos = 0;
    
    // SOF
    out_buffer[pos++] = HOST_CMD_SOF0;
    out_buffer[pos++] = HOST_CMD_SOF1;
    
    // 头部: cmd flags seq len_lo len_hi
    out_buffer[pos++] = frame.cmd;
    out_buffer[pos++] = frame.flags;
    out_buffer[pos++] = frame.seq;
    out_buffer[pos++] = frame.len & 0xFF;
    out_buffer[pos++] = (frame.len >> 8) & 0xFF;
    
    // payload
    if (frame.len > 0) {
        memcpy(&out_buffer[pos], frame.payload, frame.len);
        pos += frame.len;
    }
    
    // CRC: 计算 cmd flags seq len_lo len_hi payload
    // 使用增量 CRC 避免 4KB 栈缓冲
    uint16_t crc = HostCmdCrc16::crc16((const uint8_t*)(&out_buffer[2]), 5);  // header starts at offset 2
    if (frame.len > 0) {
        crc = HostCmdCrc16::crc16(frame.payload, frame.len, crc);
    }
    
    out_buffer[pos++] = crc & 0xFF;
    out_buffer[pos++] = (crc >> 8) & 0xFF;
    
    return pos;
}

uint16_t HostCmdCodec::encode_ack(uint8_t seq, uint8_t* out_buffer, uint16_t max_len) {
    HostFrame frame;
    frame.cmd = (uint8_t)HostCmd::ACK;
    frame.flags = HOST_CMD_FLAG_RESPONSE;
    frame.seq = seq;
    frame.len = 0;
    
    return encode_frame(frame, out_buffer, max_len);
}

uint16_t HostCmdCodec::encode_nak(uint8_t seq, HostCmdError err_code, const char* msg,
                                   uint8_t* out_buffer, uint16_t max_len) {
    HostFrame frame;
    frame.cmd = (uint8_t)HostCmd::NAK;
    frame.flags = HOST_CMD_FLAG_RESPONSE | HOST_CMD_FLAG_NAK_ERR;
    frame.seq = seq;
    
    // payload: err_code(1B) + msg(可选)
    frame.len = 1;
    frame.payload[0] = (uint8_t)err_code;
    
    if (msg) {
        size_t msg_len = strlen(msg);
        if (msg_len > HOST_CMD_PAYLOAD_MAX - 1) {
            msg_len = HOST_CMD_PAYLOAD_MAX - 1;
        }
        memcpy(&frame.payload[1], msg, msg_len);
        frame.len += msg_len;
    }
    
    return encode_frame(frame, out_buffer, max_len);
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
    
    uint8_t cmd_code = frame.cmd;
    
    // 查找对应的处理器
    if (_handler_registered[cmd_code]) {
        _handlers[cmd_code](frame, resp_buf, resp_len);
    } else {
        // 未实现的命令，回 NAK
        _handle_not_implemented(frame, resp_buf, resp_len);
    }
}

void HostCmdDispatcher::_handle_hello(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // ★新会话先停遗留遥测流★：上一会话若未 TELEM_STOP(如 UI 崩溃/直接关闭),设备会持续
    // 狂发 TELEM_DATA 淹没 vendor 端点,导致本次 HELLO 的 DEVICE_INFO 响应挤不出去→UI 卡在
    // "等待设备信息"。收到 HELLO=新主机连接,先停流使端点安静,DEVICE_INFO 得以送达。
    SensorLink::getInstance()->stop();

    // Bytes 0..14 preserve the legacy DEVICE_INFO layout. A versioned report follows.
    HostFrame resp;
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
    resp.payload[report_start + 1] = static_cast<uint8_t>(resp.len - report_start);

    *resp_len = HostCmdCodec::encode_frame(resp, resp_buf, 512);
}

void HostCmdDispatcher::_handle_ping(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // PING → ACK
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
}

void HostCmdDispatcher::_handle_not_implemented(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // 未实现 → NAK(NOT_IMPLEMENTED)
    *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::NOT_IMPLEMENTED, 
                                          "TODO: #5b/#5c", resp_buf, 512);
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
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "key required", resp_buf, 512);
        return;
    }
    
    std::string key((const char*)frame.payload, frame.len);
    if (!ConfigManager::has_key(key)) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "key not found", resp_buf, 512);
        return;
    }
    
    ConfigValue val = ConfigManager::get(key);
    
    // 构建响应帧
    HostFrame resp;
    resp.cmd = (uint8_t)HostCmd::CFG_GET;
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.len = HostCmdCodec::encode_entry(val, key.c_str(), resp.payload, HOST_CMD_PAYLOAD_MAX);
    
    if (resp.len == 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::CONFIG_ERROR, "entry encode failed", resp_buf, 512);
        return;
    }
    
    *resp_len = HostCmdCodec::encode_frame(resp, resp_buf, 512);
}

static void _handle_cfg_set(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_SET(0x11): payload = Entry
    // 响应: ACK 或 NAK
    
    std::string key;
    ConfigValue val;
    uint16_t consumed = HostCmdCodec::decode_entry(frame.payload, frame.len, &key, &val);
    
    if (consumed == 0 || key.empty()) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "entry decode failed", resp_buf, 512);
        return;
    }
    
    if (!ConfigManager::has_key(key)) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "key not found", resp_buf, 512);
        return;
    }
    
    // 类型必须一致
    ConfigValue existing = ConfigManager::get(key);
    if (existing.type != val.type) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "type mismatch", resp_buf, 512);
        return;
    }
    
    // set 会自动 clamp
    ConfigManager::set(key, val);
    
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
}

static void _handle_cfg_get_group(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_GET_GROUP(0x12): payload = prefix
    // 响应: count(u16 LE) + Entry×count
    
    std::string prefix((const char*)frame.payload, frame.len);
    auto group = ConfigManager::get_group(prefix);
    
    HostFrame resp;
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
                                                 "entry encode failed", resp_buf, 512);
            return;
        }
        resp.len += entry_len;
    }
    
    *resp_len = HostCmdCodec::encode_frame(resp, resp_buf, 512);
}

static void _handle_cfg_get_all(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_GET_ALL(0x13): 空 payload
    // 响应: count(u16 LE) + Entry×count; 超 4096B 则分帧
    
    auto all = ConfigManager::get_all();
    
    HostFrame resp;
    resp.cmd = (uint8_t)HostCmd::CFG_GET_ALL;
    resp.flags = HOST_CMD_FLAG_RESPONSE;
    resp.seq = frame.seq;
    resp.len = 0;
    
    // count
    uint16_t count = all.size();
    resp.payload[resp.len++] = count & 0xFF;
    resp.payload[resp.len++] = (count >> 8) & 0xFF;
    
    // entries - 简化：一次全发，不做分帧（假设总量 < 4096B）
    for (auto& kv : all) {
        uint16_t entry_len = HostCmdCodec::encode_entry(kv.second, kv.first.c_str(), 
                                                         &resp.payload[resp.len], 
                                                         HOST_CMD_PAYLOAD_MAX - resp.len);
        if (entry_len == 0) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::CONFIG_ERROR, 
                                                 "entry encode failed", resp_buf, 512);
            return;
        }
        resp.len += entry_len;
    }
    
    // ⚠ 全量配置响应可达 ~900B，需用大 max_len（_resp_buf 已扩到 2048）；旧硬编码 512 会截断→上位机收0项。
    *resp_len = HostCmdCodec::encode_frame(resp, resp_buf, 2048);
}

static void _handle_cfg_set_batch(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // CFG_SET_BATCH(0x14): payload = count(u16) + Entry×count
    // 响应: ACK(全成功) 或 NAK(首个失败 key)
    
    if (frame.len < 2) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "count required", resp_buf, 512);
        return;
    }
    
    uint16_t count = frame.payload[0] | (((uint16_t)frame.payload[1]) << 8);
    uint16_t pos = 2;
    
    std::map<std::string, ConfigValue> batch;
    
    // 解码所有 entry
    for (uint16_t i = 0; i < count; i++) {
        if (pos >= frame.len) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "incomplete entry", resp_buf, 512);
            return;
        }
        
        std::string key;
        ConfigValue val;
        uint16_t entry_len = HostCmdCodec::decode_entry(&frame.payload[pos], frame.len - pos, &key, &val);
        
        if (entry_len == 0 || key.empty()) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, "entry decode failed", resp_buf, 512);
            return;
        }
        
        if (!ConfigManager::has_key(key)) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, key.c_str(), resp_buf, 512);
            return;
        }
        
        // 类型检查
        ConfigValue existing = ConfigManager::get(key);
        if (existing.type != val.type) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM, key.c_str(), resp_buf, 512);
            return;
        }
        
        batch[key] = val;
        pos += entry_len;
    }
    
    // 全部有效，批量设置
    ConfigManager::set_batch(batch);
    
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
}

static void _handle_save_config(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // SAVE_CONFIG(0x0E): 仅置保存信号，实际 flash 落地延迟到主循环安全窗口(写后重新枚举恢复 USB)。
    // 避免在 handler 上下文(正处理命令/待发 ACK)flash 写禁中断数十 ms 打断 USB 事务→vendor 失步。
    ConfigManager::save_config();               // 置 config 保存信号
    CsdConfig::getInstance()->request_save();   // 置 CSD store 保存信号
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
}

static void _handle_reset_defaults(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    // RESET_DEFAULTS(0x0F): 空 → reset_to_defaults()
    ConfigManager::reset_to_defaults();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
}
