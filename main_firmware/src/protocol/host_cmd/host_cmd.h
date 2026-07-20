#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

/**
 * Host Command 协议 - USB CDC 二进制帧编解码 + 命令分发
 * 
 * 帧格式(双向通用):
 *   SOF0(0xAA) SOF1(0x55) cmd flags seq len(u16 LE) payload[len] crc16(u16 LE)
 * 
 * - CRC-16/CCITT-FALSE(poly 0x1021, init 0xFFFF), 覆盖 cmd..payload
 * - 接收状态机: 增量喂字节，找 SOF→读头→按 len 收 payload→校验 CRC
 * - 编码: 给定 cmd/flags/seq/payload 组帧到输出缓冲(含 CRC)
 */

// 帧格式常量
#define HOST_CMD_SOF0           0xAA
#define HOST_CMD_SOF1           0x55
#define HOST_CMD_PAYLOAD_MAX    4096  // payload 上限，超限丢弃保护
#define HOST_CMD_HEADER_SIZE    7     // SOF0 SOF1 cmd flags seq len(2B)

// flags 定义
#define HOST_CMD_FLAG_RESPONSE  0x01  // bit0=1 表示响应
#define HOST_CMD_FLAG_STREAM    0x02  // bit1=1 流数据帧
#define HOST_CMD_FLAG_NAK_ERR   0x04  // bit2=1 NAK(错误)

// 命令码 enum class
enum class HostCmd : uint8_t {
    // 系统域 0x01-0x0F
    HELLO           = 0x01,
    DEVICE_INFO     = 0x02,
    PING            = 0x03,
    REBOOT          = 0x04,
    REBOOT_BOOTLOADER = 0x05,
    SAVE_CONFIG     = 0x0E,
    RESET_DEFAULTS  = 0x0F,
    
    // 配置KV域 0x10-0x1F
    CFG_GET         = 0x10,
    CFG_SET         = 0x11,
    CFG_GET_GROUP   = 0x12,
    CFG_GET_ALL     = 0x13,
    CFG_SET_BATCH   = 0x14,
    
    // CapSense调参域 0x20-0x2F
    PARAM_GET       = 0x20,
    PARAM_SET       = 0x21,
    PARAM_GET_ALL   = 0x22,
    CALIBRATE       = 0x23,
    BASELINE_RESET  = 0x24,
    MODE_SET        = 0x25,  // payload=mode(u8): 0=自动校准, 1=半自动/手动
    CSD_CAPTURE     = 0x26,  // 空: 从 PSoC 读当前参数入 RP2040 store(半自动种子)
    CP_MEASURE      = 0x27,  // 空: 异步触发全部电极的寄生电容测量
    CP_GET          = 0x28,  // payload=channel(u8): 读取最近一次 Cp(fF)
    
    // 遥测流域 0x30-0x3F
    TELEM_START     = 0x30,
    TELEM_STOP      = 0x31,
    TELEM_DATA      = 0x32,
    
    // 绑区域 0x40-0x4F
    BIND_START      = 0x40,
    BIND_ABORT      = 0x41,
    BIND_CONFIRM    = 0x42,
    BIND_GET_MAP    = 0x43,
    BIND_SET_MAP    = 0x44,
    BIND_EVENT      = 0x45,
    
    // 灯效域 0x50-0x5F
    LED_GET         = 0x50,
    LED_SET_REGION  = 0x51,
    LED_PREVIEW     = 0x52,
    
    // 应答 0x7E-0x7F
    ACK             = 0x7E,
    NAK             = 0x7F,
};

// NAK 错误码
enum class HostCmdError : uint8_t {
    NOT_IMPLEMENTED = 0x01,
    INVALID_PARAM   = 0x02,
    DEVICE_BUSY     = 0x03,
    CONFIG_ERROR    = 0x04,
    SENSOR_ERROR    = 0x05,
};

// 解析后的帧
struct HostFrame {
    uint8_t cmd;
    uint8_t flags;
    uint8_t seq;
    uint16_t len;
    uint8_t payload[HOST_CMD_PAYLOAD_MAX];
    
    void clear() {
        cmd = 0;
        flags = 0;
        seq = 0;
        len = 0;
        memset(payload, 0, sizeof(payload));
    }
};

// CRC-16/CCITT-FALSE 工具(内联)
class HostCmdCrc16 {
public:
    static inline uint16_t crc16(const uint8_t* data, uint16_t len, uint16_t init = 0xFFFF) {
        uint16_t crc = init;
        for (uint16_t i = 0; i < len; i++) {
            crc ^= ((uint16_t)data[i] << 8);
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
};

// Forward declare ConfigValue
struct ConfigValue;

// 帧编解码器
class HostCmdCodec {
public:
    // 初始化
    void init();
    
    // 增量喂字节，返回是否解析出完整帧
    bool feed_byte(uint8_t byte, HostFrame* out_frame);
    
    // 编码帧到输出缓冲，返回帧字节数
    static uint16_t encode_frame(const HostFrame& frame, uint8_t* out_buffer, uint16_t max_len);
    
    // 快捷编码应答(ACK/NAK)
    static uint16_t encode_ack(uint8_t seq, uint8_t* out_buffer, uint16_t max_len);
    static uint16_t encode_nak(uint8_t seq, HostCmdError err_code, const char* msg, 
                               uint8_t* out_buffer, uint16_t max_len);
    
    // Entry 编码/解码(配置 KV 统一格式)
    // 编码一个条目到缓冲: type(u8) + has_range(u8) + key_len(u8) + key + value(+min+max if has_range)
    static uint16_t encode_entry(const ConfigValue& val, const char* key, uint8_t* out_buf, uint16_t max_len);
    // 解码一个条目从缓冲，返回消耗字节数；失败返回 0
    static uint16_t decode_entry(const uint8_t* in_buf, uint16_t in_len, std::string* out_key, ConfigValue* out_val);
    
    // 复位状态机
    void reset();
    
private:
    enum class RxState {
        FIND_SOF0,
        FIND_SOF1,
        READ_HEADER,
        READ_PAYLOAD,
        READ_CRC,
    };
    
    RxState _state;
    uint8_t _header[5];  // cmd flags seq len_lo len_hi
    uint8_t _header_pos;
    uint16_t _payload_len;
    uint16_t _payload_pos;
    uint8_t _payload[HOST_CMD_PAYLOAD_MAX];
    uint8_t _crc_bytes[2];
    uint8_t _crc_pos;
    
    bool _check_frame_complete(HostFrame* out_frame);
};

// 命令分发器
class HostCmdDispatcher {
public:
    using CmdHandler = std::function<void(const HostFrame&, uint8_t* resp_buf, uint16_t* resp_len)>;
    
    static HostCmdDispatcher* getInstance();
    
    // 注册命令处理器
    void register_handler(HostCmd cmd, CmdHandler handler);
    
    // 分发命令
    void dispatch(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
    
private:
    HostCmdDispatcher();
    HostCmdDispatcher(const HostCmdDispatcher&) = delete;
    HostCmdDispatcher& operator=(const HostCmdDispatcher&) = delete;
    
    static HostCmdDispatcher* _instance;
    
    // 处理器映射(sparse，不是全 256 项)
    CmdHandler _handlers[256];
    bool _handler_registered[256];
    
    // 内置处理器
    static void _handle_hello(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
    static void _handle_ping(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
    static void _handle_not_implemented(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
};

