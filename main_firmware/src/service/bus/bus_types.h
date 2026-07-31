#pragma once

#include <cstdint>
#include <cstring>

// CRC-16/CCITT-FALSE(poly 0x1021, init 0xFFFF) 复用既有实现, 全仓唯一一份:
//   HostCmdCrc16::crc16(data, len, init)  @ protocol/host_cmd/host_cmd.h
// 本文件不新写第二份 CRC。
#include "../../protocol/host_cmd/host_cmd.h"

/**
 * Mai2Bus 线上信封 + RAM 信封槽的类型定义(纯数据, 无行为依赖)。
 *
 * ── 线上信封头: 固定 16 字节, 全字段小端(LE), 字段序写死 ──────────────────
 *  off  size  field       说明
 *   0    1    magic       固定 BUS_MAGIC(0xB5), 帧同步用
 *   1    1    msg_id      业务消息 id(总线不解释其含义)
 *   2    2    seq         LE; 一次 publish/stream 的传输序号, 同一传输的所有分片共享
 *   4    2    total_len   LE; 整条消息字节数; 流式未知长度用 BUS_LEN_UNKNOWN(0xFFFF)
 *   6    2    frag_off    LE; 本分片在整条消息中的字节偏移(流式=流内累计偏移, 允许回绕)
 *   8    1    frag_len    本分片 payload 字节数, ≤ BUS_INLINE_MAX(64)
 *   9    1    flags       见 BUS_FLAG_* 位常量(位常量, 不用位域)
 *  10    4    t_us        LE; ★发送方生成本信封那一刻的 time_us_32()★(见下)
 *  14    2    crc         LE; CRC-16/CCITT-FALSE
 *
 *  CRC 覆盖范围 = off[1..13](msg_id..t_us, 共 13 字节) + payload[0..frag_len-1]。
 *  即: 不含 magic(off 0), 不含 crc 自身(off 14..15)。
 *
 *  ── t_us: 全协议统一时间基准 ─────────────────────────────────────────────
 *  取值 = 设备端 time_us_32()(u32 微秒, 约 71.6 分钟 32 位回绕), 与 host_cmd 尾戳
 *  (HOST_CMD_FLAG_TS)、KBD_GET_EDGES 的 t_us、TELEM_DATA 的 ts_us 同源同口径 ⇒
 *  跨协议的事件可直接横向对位。回绕由接收方用"相邻差值 wrapping_sub"吸收, 本层不处理。
 *  出向填充点唯一: Mai2Bus::_tx_enqueue / Mai2Bus::push(ACK/NAK 也经 _tx_enqueue)。
 *
 *  注: BusHdr 的 C++ 成员序 **不等于** 线上字段序 —— 成员按对齐紧排以保证
 *  sizeof(BusHdr)==16 无内部填充(否则 uint32_t t_us 插在 flags 后会撑出 20 字节,
 *  连带 BusEnv/Mai2Bus 超预算)。线上字节序**只**由 bus_hdr_encode/bus_hdr_decode
 *  逐字节定义, 任何一方都不得依赖结构体布局。
 */

// ── 线上常量 ──────────────────────────────────────────────────────────────
#define BUS_MAGIC          0xB5u
#define BUS_HDR_SIZE       16u      // 线上头字节数(固定)
#define BUS_CRC_SPAN       13u      // 头内被 CRC 覆盖的字节数(off 1..13)
#define BUS_INLINE_MAX     64u      // 单信封内联 payload 上限 = 分片 MTU
#define BUS_LEN_UNKNOWN    0xFFFFu  // total_len 哨兵: 流式, 总长未知

// ── flags 位常量 ─────────────────────────────────────────────────────────
#define BUS_FLAG_FIRST     0x01u    // bit0 本分片是整条消息的首片
#define BUS_FLAG_LAST      0x02u    // bit1 本分片是整条消息的末片
#define BUS_FLAG_STREAM    0x04u    // bit2 流式帧: 不整包缓存, 逐片直接派发
#define BUS_FLAG_ACK_REQ   0x08u    // bit3 要求对端回 ACK(进入重传窗口)
#define BUS_FLAG_IS_ACK    0x10u    // bit4 本帧是 ACK
#define BUS_FLAG_NAK       0x20u    // bit5 本帧是 NAK(显式失败)

// 传输结果码: 每个带回调的传输恰好以其中一个终结一次。
enum class BusResult : uint8_t {
    OK = 0,       // 成功(非 ACK 传输 = 已交付传输层; ACK 传输 = 已收到对端 ACK)
    TIMEOUT,      // 超时(未送出 / 重传耗尽仍无 ACK)
    PEER_NAK,     // 对端显式 NAK
    CRC_FAIL,     // 校验失败
    QUEUE_FULL,   // 队列背压, 未能受理
    ABORTED,      // 主动中止(如 init/clear 清场)
};

// 信封槽生命周期状态(per-transfer, 只存在于 BusEnv 内, 总线本身不持业务状态)。
enum class BusState : uint8_t {
    FREE = 0,     // 槽空闲
    PENDING,      // 已排队, 等待传输层取走(或等待重发)
    INFLIGHT,     // 已交给传输层, 等待 ACK; 到期触发重传
    DONE,         // 已终结(回调已触发或无回调), 可回收
};

// 传输完成回调: 只在 Mai2Bus::task() 上下文触发, 绝不在中断/另一核触发。
using BusCb = void (*)(uint16_t seq, BusResult result, void* ctx);

// 订阅接收回调: 交付完整消息(或流式的一片)及该 msg_id 通道的循环计数。
// 跳号判定由订阅方自行维护状态；同样只在 task() 上下文触发。
using BusRxCb = void (*)(uint8_t msg_id, uint16_t counter, const uint8_t* data, uint16_t len, void* ctx);

// ── 线上信封头 ───────────────────────────────────────────────────────────
// ★成员序按对齐紧排, 与线上字段序无关★(线上序见文件头表 + encode/decode)。
// 4 字节成员在前、2 字节居中、1 字节收尾 ⇒ 16 字节零填充。
struct BusHdr {
    uint32_t t_us;        // 线上 off 10..13
    uint16_t seq;         // 线上 off 2..3
    uint16_t total_len;   // 线上 off 4..5
    uint16_t frag_off;    // 线上 off 6..7
    uint16_t crc;         // 线上 off 14..15
    uint8_t  magic;       // 线上 off 0
    uint8_t  msg_id;      // 线上 off 1
    uint8_t  frag_len;    // 线上 off 8
    uint8_t  flags;       // 线上 off 9

    void clear() {
        t_us = 0u;
        seq = 0u;
        total_len = 0u;
        frag_off = 0u;
        crc = 0u;
        magic = BUS_MAGIC;
        msg_id = 0u;
        frag_len = 0u;
        flags = 0u;
    }
};
static_assert(sizeof(BusHdr) == BUS_HDR_SIZE, "BusHdr 必须是 16 字节且无内部填充");

// ── RAM 信封槽 ───────────────────────────────────────────────────────────
// 布局(实测 sizeof(BusEnv) == 96 字节, 见文件末 static_assert, 目标上限 96):
//   off  0  hdr(16) | off 16 payload(64) | off 80 deadline_us(4)
//   off 84 cb(4)    | off 88 ctx(4)      | off 92 retries(1) 93 state(1) 94 result(1) +1 pad
// 无 std::function / 无 new / 无 malloc。
// 头由 12→16(加 t_us)使本结构 92→96, 已把 Mai2Bus 顶到 12KB 上限附近 ⇒ 子环池由 4 降为 3
// (见 bus_core.h BUS_SUBRING_SLOTS 注释)。此处 96 是硬预算, 再加字段必须先腾容量。
struct BusEnv {
    BusHdr   hdr;
    uint8_t  payload[BUS_INLINE_MAX];

    // ── per-transfer 状态(仅此处, 不外溢到总线成员) ──
    uint32_t deadline_us;   // time_us_32() 基准的到期时刻
    BusCb    cb;            // 完成回调; 触发后立即置 nullptr → 结构性保证只触发一次
    void*    ctx;
    uint8_t  retries;       // 已重传次数
    BusState state;
    BusResult result;       // state==DONE 时的终结原因; 回调只在 task() 内按此值触发

    void clear() {
        hdr.clear();
        memset(payload, 0, sizeof(payload));
        deadline_us = 0u;
        cb = nullptr;
        ctx = nullptr;
        retries = 0u;
        state = BusState::FREE;
        result = BusResult::OK;
    }
};
static_assert(sizeof(BusEnv) == 96u, "BusEnv 实测应为 96 字节");
static_assert(sizeof(BusEnv) <= 96u, "BusEnv 超出 96 字节预算");

// 入向拥塞时的 NAK 请求(跨核 SPSC 传递, 由 task() 转成真正的 NAK 信封)。
struct BusNakReq {
    uint16_t seq;
    uint8_t  msg_id;
    uint8_t  _rsv;

    void clear() { seq = 0u; msg_id = 0u; _rsv = 0u; }
};

// ── 编解码 ───────────────────────────────────────────────────────────────
// 头 → 16 字节线上缓冲(小端)。out 必须 ≥ BUS_HDR_SIZE。
static inline void bus_hdr_encode(const BusHdr& h, uint8_t* out) {
    out[0]  = BUS_MAGIC;
    out[1]  = h.msg_id;
    out[2]  = (uint8_t)(h.seq & 0xFFu);
    out[3]  = (uint8_t)((h.seq >> 8) & 0xFFu);
    out[4]  = (uint8_t)(h.total_len & 0xFFu);
    out[5]  = (uint8_t)((h.total_len >> 8) & 0xFFu);
    out[6]  = (uint8_t)(h.frag_off & 0xFFu);
    out[7]  = (uint8_t)((h.frag_off >> 8) & 0xFFu);
    out[8]  = h.frag_len;
    out[9]  = h.flags;
    out[10] = (uint8_t)(h.t_us & 0xFFu);
    out[11] = (uint8_t)((h.t_us >> 8) & 0xFFu);
    out[12] = (uint8_t)((h.t_us >> 16) & 0xFFu);
    out[13] = (uint8_t)((h.t_us >> 24) & 0xFFu);
    out[14] = (uint8_t)(h.crc & 0xFFu);
    out[15] = (uint8_t)((h.crc >> 8) & 0xFFu);
}

// 16 字节线上缓冲 → 头。magic 不符返回 false。
static inline bool bus_hdr_decode(const uint8_t* in, BusHdr& h) {
    if (in[0] != BUS_MAGIC) {
        return false;
    }
    h.magic     = in[0];
    h.msg_id    = in[1];
    h.seq       = (uint16_t)(in[2] | ((uint16_t)in[3] << 8));
    h.total_len = (uint16_t)(in[4] | ((uint16_t)in[5] << 8));
    h.frag_off  = (uint16_t)(in[6] | ((uint16_t)in[7] << 8));
    h.frag_len  = in[8];
    h.flags     = in[9];
    h.t_us      = (uint32_t)in[10] | ((uint32_t)in[11] << 8) |
                  ((uint32_t)in[12] << 16) | ((uint32_t)in[13] << 24);
    h.crc       = (uint16_t)(in[14] | ((uint16_t)in[15] << 8));
    return true;
}

// 按 CRC 覆盖范围(msg_id..t_us + payload)计算校验值; 复用 HostCmdCrc16。
static inline uint16_t bus_crc(const BusHdr& h, const uint8_t* payload) {
    uint8_t span[BUS_CRC_SPAN];
    span[0]  = h.msg_id;
    span[1]  = (uint8_t)(h.seq & 0xFFu);
    span[2]  = (uint8_t)((h.seq >> 8) & 0xFFu);
    span[3]  = (uint8_t)(h.total_len & 0xFFu);
    span[4]  = (uint8_t)((h.total_len >> 8) & 0xFFu);
    span[5]  = (uint8_t)(h.frag_off & 0xFFu);
    span[6]  = (uint8_t)((h.frag_off >> 8) & 0xFFu);
    span[7]  = h.frag_len;
    span[8]  = h.flags;
    span[9]  = (uint8_t)(h.t_us & 0xFFu);
    span[10] = (uint8_t)((h.t_us >> 8) & 0xFFu);
    span[11] = (uint8_t)((h.t_us >> 16) & 0xFFu);
    span[12] = (uint8_t)((h.t_us >> 24) & 0xFFu);

    uint16_t crc = HostCmdCrc16::crc16(span, BUS_CRC_SPAN);
    if ((h.frag_len > 0u) && (payload != nullptr)) {
        crc = HostCmdCrc16::crc16(payload, h.frag_len, crc);
    }
    return crc;
}
