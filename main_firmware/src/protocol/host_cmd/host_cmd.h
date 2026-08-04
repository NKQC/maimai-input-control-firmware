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
 * - 设备→主机方向的 payload 末尾统一追加 4 字节设备时间戳并置 HOST_CMD_FLAG_TS,
 *   详见该宏的注释(加法式, 对既有命令零影响)。
 */

// 帧格式常量
#define HOST_CMD_SOF0           0xAA
#define HOST_CMD_SOF1           0x55
#define HOST_CMD_PAYLOAD_MAX    4096  // payload 上限，超限丢弃保护
#define HOST_CMD_HEADER_SIZE    7     // SOF0 SOF1 cmd flags seq len(2B)
// 完整编码帧上限 = 头(7) + 满载 payload(4096) + CRC(2), 余量取整。响应缓冲与
// encode_frame max_len 统一用它: 避免全量 CFG_GET_ALL(~140 项 ~2.1KB)超旧 2048/512
// 上限时 encode_frame 返回 0 → 主机收 0 项 → 设置页空白。
#define HOST_CMD_RESP_BUF_MAX   (HOST_CMD_PAYLOAD_MAX + 16)
// 算法 C 源(最大 32KB)分片传输的单片字节数。payload 固定 4096 且 HostFrame 常作栈对象, 绝不放大,
// 故源必须分片; 留出 4 字节片头(offset/total 或 total/offset)后仍有充裕余量。
#define HOST_CMD_ALGO_SRC_CHUNK 2048

// flags 定义
#define HOST_CMD_FLAG_RESPONSE  0x01  // bit0=1 表示响应
#define HOST_CMD_FLAG_STREAM    0x02  // bit1=1 流数据帧
#define HOST_CMD_FLAG_NAK_ERR   0x04  // bit2=1 NAK(错误)
// bit3=1: payload 末 4 字节是设备端时间戳 t_us(u32 LE) = time_us_32()。
//
// ★加法式设计, 不是破坏式改版★ 60+ 条既有命令的 payload 布局一律不动, 时间戳只**追加**在
// 末尾并由本 flag 位标识 —— 与 KBD_GET_STATE(0x70) "尾部追加 raw/out, 旧上位机只读前 2 字节
// 仍然正确"是同一条先例。于是:
//   · 所有按固定偏移读前部字段的处理器/解析器完全不受影响, 一行都不用改;
//   · 忽略本 flag 的旧上位机只会多看到 4 字节尾巴, 按 len 收帧、按偏移解析都仍正确;
//   · 新增命令不必单独处理时间戳 —— 唯一填充点在 HostCmdCodec::encode_frame。
// ★方向★ 只有设备→主机的帧带戳(主机时钟对设备无意义), 主机→设备的请求帧一律不带。
// ★退化条件(两个, 均为"不追加"而非"报错")★
//   · frame.len + 4 > HOST_CMD_PAYLOAD_MAX: 追加会让 len 超协议上限, 对端会当超长帧丢弃;
//   · max_len 放不下多出的 4 字节: 保证任何原本能编码成功的帧都仍然成功(零回归)。
//   这两种情形只出现在满载的批量传输帧(CFG_GET_ALL/ALGO 源分片)上, 那些帧的时刻本无意义。
// ★时基★ time_us_32(), u32 微秒, 约 71.6 分钟 32 位回绕; 与 bus 信封头 t_us、
//   KBD_GET_EDGES 的 t_us、TELEM_DATA 的 ts_us 同源同口径 ⇒ 可直接横向对位。
#define HOST_CMD_FLAG_TS        0x08
// 尾部时间戳字节数(u32 LE)。
#define HOST_CMD_TS_SIZE        4

// 命令码 enum class
enum class HostCmd : uint8_t {
    // 系统域 0x01-0x0F
    HELLO           = 0x01,
    DEVICE_INFO     = 0x02,
    PING            = 0x03,
    REBOOT          = 0x04,
    REBOOT_BOOTLOADER = 0x05,
    REBOOT_PSOC     = 0x06,  // 脉冲 XRES 重启 PSoC 使"需重启生效"的改动生效
    DEBUG_CRASH_BOOTSEL = 0x07,  // 运行时武装/解除"崩溃→进BOOTSEL"(自持debug); payload[0]: 1=武装 0=解除
    // ★PSoC 救砖★: 空 payload → 立即 ACK("已受理"), 主循环随后经 SWD 强制全片擦写内嵌镜像 + 校验 +
    // 复位运行, 再由 provisioning 重新下发算法 + CSD。阶段进度经 PSOC_RESCUE_PROGRESS(0x09) 推送。
    PSOC_RESCUE     = 0x08,
    // ★设备主动推送(flags=STREAM)★: 救砖受理后 5Hz 上报, 完成帧发出即自取消任务。
    // payload = [state(u8: 0空闲/1进行中/2完成), phase(u8: 0空闲/1重刷中/2重新应用/3完成/4失败),
    //            result(u8: 0进行中/1成功/2失败), stage(u8 PsocBringupStage), fail_stage(u8)]
    PSOC_RESCUE_PROGRESS = 0x09,
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
    MODE_SET        = 0x25,  // payload=mode(u8): 0=自动校准/标准完整处理, 1=半自动手动
    CSD_CAPTURE     = 0x26,  // 空: 从 PSoC 读当前参数入 RP2040 store(半自动种子)
    CP_MEASURE      = 0x27,  // 空: 异步触发全部电极的寄生电容测量
    CP_GET          = 0x28,  // payload=channel(u8): 读取最近一次 Cp(fF)
    GLOBAL_GET      = 0x29,  // payload=gparam_id(u8) → 响应 [gparam_id, value(u32 LE)]
    GLOBAL_SET      = 0x2A,  // payload=[gparam_id(u8), value(u32 LE)] 写全局CSD配置+APPLY → ACK
    GLOBAL_GET_ALL  = 0x2B,  // 空 → 响应 [count(u8), (gparam_id, value u32 LE)×count]
    AUTO_TUNE       = 0x2C,  // [ch(u8): 0..35 单通道 / 0xFF 全通道, pref(u8 可选): 灵敏度档位 1..7(缺省4)]
                             // → 异步触发频率自适应下探; 立即回 ACK(仅表示"已受理", 非完成),
                             // 阶段进度与最终结果经 AUTO_TUNE_PROGRESS(0x2E) 推送流上报。
    GLOBAL_COMMIT   = 0x2D,  // 空 → 批量下发全局项后【单次】触发 PSoC 完整重初始化(替代逐项 commit, 防重初始化风暴)
    // ★设备主动推送(flags=STREAM)★: AUTO_TUNE 受理后 5Hz 上报阶段进度, 完成帧发出即自取消任务。
    // payload = [state(u8: 0空闲/1进行中/2完成), phase(u8: 0受理/1粗定位/2细搜临界/3落档回退/4完成),
    //            step(u8 阶段内步序), cur_div(u16 LE 当前试探分频), ch(u8),
    //            result(u8: 0进行中/1成功/2失败), final_div(u16 LE), origin_seq(u8)]
    // ★origin_seq = 发起本轮的 AUTO_TUNE 请求帧 seq★ 本流是 STREAM 推送, 帧头 seq 是设备流序号、与
    // 请求无关, 故必须回显请求 seq, 上位机才能把终态严格归属到自己发起的那一次(逐通道批量下, 迟到的
    // 上一轮终态否则会被算到下一个通道头上)。追加在尾部 ⇒ 前 9 字节与旧固件逐字节相同, 旧上位机不受影响。
    AUTO_TUNE_PROGRESS = 0x2E,
    // ★设备主动推送(flags=STREAM)★: 固件"自己救自己"的动作(复位 PSoC/回退算法/清空 CSD store/
    // 重新下发/PSoC 启动强制改写配置)会让设备实际状态偏离上位机以为的状态, 必须上报, 否则界面是幻觉。
    // payload = [code(u8 见 SelfHealCode), detail(u32 LE), seq(u16 LE), total(u16 LE)]
    SELF_HEAL_EVENT = 0x2F,

    
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

    // JIT 触控算法域 0x60-0x6F
    ALGO_GET_INFO      = 0x60,  // 空 → 响应 [is_default(u8),psoc_valid(u8),len(u16 LE),crc16(u16 LE)]
    ALGO_UPLOAD        = 0x61,  // payload=[len(u16 LE),crc16(u16 LE),data[len]] 存储+校验+下发 → ACK/NAK
    ALGO_APPLY         = 0x62,  // 空 → 把当前存储算法下发 PSoC → ACK/NAK
    ALGO_RESET_DEFAULT = 0x63,  // 空 → 回退内嵌默认(v3.1 HDR)+下发 → ACK
    ALGO_SET_ROM       = 0x64,  // payload=[ch(u8),rom(u16 LE)]×count 设每通道 ROM+下发 → ACK
    ALGO_GET_ROM       = 0x65,  // 空 → 响应 36×u16 LE (每通道 ROM 表)
    // ★C 源(≤32KB)分片协议★ 单帧 payload 只有 4096, 所以按 HOST_CMD_ALGO_SRC_CHUNK 分片。
    // GET: 请求 [offset(u16 LE)](空 payload 视作 offset=0) → 响应 [total(u16 LE), offset(u16 LE), chunk]
    ALGO_GET_SRC       = 0x66,
    // SET: [offset(u16 LE), total(u16 LE), chunk]; offset 从 0 开始严格连续, 收到最后一片
    //      (offset+chunk==total)才更新有效长度并持久化。total=0 = 清空源。→ ACK / NAK
    ALGO_SET_SRC       = 0x67,
    ALGO_GET_CODE      = 0x68,  // 空 → 响应 [len(u16 LE), asm bytes] RP 存的算法 ASM 机器码回读
    ALGO_GET_TRACE     = 0x69,  // payload=[ch(u8),idx(u8)] → 响应 [ch,idx,out_active(u8),report(u16 LE)]
    ALGO_SET_CFG       = 0x6A,  // payload=[idx(u8),val(u8)] 设共享 cfg[idx]+持久化+下发 → ACK
    ALGO_GET_CFG       = 0x6B,  // payload=[idx(u8)] → 响应 [idx,cfg(u8)]

    // 物理键盘 / 触控键盘映射域 + mai2 串口状态 0x70-0x7D
    // 空 → [phys_state(u16 LE), raw(u16 LE), out(u16 LE)]: 去抖后 / 去抖前 / 实际输出 HID 三态。
    // 后两个字段为后续追加, 旧上位机只读前 2 字节仍然正确。
    KBD_GET_STATE    = 0x70,
    KBD_GET_MAP      = 0x71,  // 空 → [count(u8)=12, keycode(u8)×12] 物理键 HID 键码表
    KBD_SET_MAP      = 0x72,  // [idx(u8),keycode(u8)]×n 设物理键 HID 键码 → ACK
    KBD_GET_TOUCHMAP = 0x73,  // 空 → [en(u8), count(u8)=34, keycode(u8)×34] 触控→键盘映射表
    KBD_SET_TOUCHMAP = 0x74,  // [zone(u8),keycode(u8)]×n 设触控→键盘键码 → ACK
    // 每键长按参数: delay_ms=按住够久才真正输出(0=立即); maxhold_ms=输出后最长保持即自动抬起(0=不抬)。
    KBD_GET_HOLD     = 0x75,  // 空 → [phys_count(u8)=12, zone_count(u8)=34,
                              //       12×(delay u16 LE, maxhold u16 LE), 34×(delay u16 LE, maxhold u16 LE)]
    KBD_SET_HOLD     = 0x76,  // [kind(u8: 0=物理键/1=分区), idx(u8), delay(u16 LE), maxhold(u16 LE)]×n → ACK
    // ★逻辑分析仪★ 物理键 12 位掩码每次变化即带 time_us_32() 时间戳入固件定长环形缓冲(192 条),
    // 主机主动拉取(无推流)。请求 [max(u8) 可选, 0=默认上限 96] →
    //   [cap(u16 LE), overflow(u32 LE 累计丢弃数), remaining(u16 LE 取完后剩余), count(u8),
    //    count×(t_us(u32 LE), raw(u16 LE 去抖前), out(u16 LE 实际输出 HID))]
    // 返回即消费(FIFO 最旧优先)。overflow 只增不清零, 主机取差值判断"有事件丢失"。
    KBD_GET_EDGES    = 0x77,
    // 每键触发极性 + 独立防抖窗。pol: 0=低电平触发 / 1=高电平触发 / 2=AUTO(默认);
    // AUTO = 只看启动时电平并当作"抬起"电平(启动高→低触发, 启动低→高触发), 启动后不再重采样。
    // GET 回显的是**配置态**(含 2), 尾部另追加解析后的生效掩码; debounce_us: 0..10000(0=不去抖),
    // 越界 NAK 不夹取。
    KBD_GET_KEYCFG   = 0x7C,  // 空 → [count(u8)=12, 12×(pol(u8), debounce_us(u16 LE)), resolved_pol_high_mask(u16 LE)]
    KBD_SET_KEYCFG   = 0x7D,  // [idx(u8), pol(u8), debounce_us(u16 LE)]×n → ACK/NAK(整批校验后才落值)
    // 组合映射(N 个分区同时按下 → M 个键同时输出)。单条 16B:
    //   zone_mask(u64 LE) | key0..key3(u8×4) | mod(u8) | delay(u16 LE) | maxhold(u16 LE)
    KBD_GET_COMBO    = 0x7A,  // 空 → [combo_count(u8)=16, key_count(u8)=4, 16×16B]
    KBD_SET_COMBO    = 0x7B,  // [count(u8)] + count×16B → ACK (整表替换; 空掩码条目丢弃, 同掩码去重)

    // mai2 触控串口状态域
    MAI2_GET_STATE   = 0x78,  // 空 → [send_active(u8: RUNNING&&serial_ok), status(u8: 0=STOPPED/1=READY/2=RUNNING), baud(u32 LE)]
    MAI2_SET_SEND_EN = 0x79,  // [en(u8)] 覆盖"是否发送触控帧"(游戏 {A}/{L} 之外的手动开关) → ACK

    // 应答 0x7E-0x7F
    ACK             = 0x7E,
    NAK             = 0x7F,

    // 0x80..0xFF 保留给总线。
    BUS_XFER        = 0x80,
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

    // 是否处于空闲(未在解析半帧)。供上层做"陈旧半帧超时复位"判断:
    // 截断/损坏帧声明了大 len 时会卡在 READ_PAYLOAD, 吞掉后续新命令(如重连后的 HELLO)。
    bool is_idle() const { return _state == RxState::FIND_SOF0; }

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

    // CFG_GET_ALL 分帧响应：UsbComm 在当前帧完全写入后调用以取得下一帧。
    bool has_pending_stream() const;
    bool next_stream_frame(uint8_t* resp_buf, uint16_t* resp_len);
    void clear_pending_stream();
    
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

