#pragma once

#include <cstdint>
#include "bus_types.h"
#include "bus_container.h"

/**
 * Mai2Bus - 设备内统一消息总线内核(单例)。
 *
 * ── 结构性约束(设计即保证, 不靠调用方自律) ──────────────────────────────
 *  1) 总线不持任何业务/协议状态: 成员里只有信封槽容器、重组暂存、订阅表和传输光标;
 *     msg_id 一律当不透明字节处理, 不解释、不分支。
 *  2) per-transfer 状态(deadline/retries/state/cb/ctx)只存在于 BusEnv 内, 不外溢成员。
 *  3) 出向满 → publish/stream_write 返回 false(背压排队, 调用方自己重试), 绝不静默丢。
 *     入向满 → 记 BusNakReq, 由 task() 生成 NAK 信封显式失败, 绝不静默丢。
 *  4) 回调只在 task() 上下文触发, 且每个带 cb 的信封恰好触发一次:
 *     触发点唯一(_tx_reap), 触发前先把 BusEnv::cb 置 nullptr → 结构上无法二次触发。
 *  5) 全部接口非阻塞: 无 delay / 无忙等 / 无自旋; 重传由 task() 比对 time_us_32()
 *     与 BusEnv::deadline_us 驱动。
 *  6) 循环嵌套 ≤ 3 层。
 *  7) 注册态(订阅表 / 子环借出 / 各 msg_id 循环计数)由构造函数拥有；init() 只清
 *     per-transfer 态，服务订阅与 bus init 的先后顺序无关，重 init 也不使计数倒退。
 *
 * ── 容量与占用(sizeof(Mai2Bus) 实测 11568 B, 上限由 bus_core.cpp 末 static_assert 卡死) ──
 * 注: 按本仓单例约定(懒 new), 这些字节落在堆上, 故链接器 RAM 报告只增 4 B(静态指针);
 *     它是本模块一次性、固定不再增长的内存占用, 已在 12 KB 硬上限内。
 *   容器                               容量            字节数        用途
 *   _usb_in     BusQueue<BusEnv,16>   16×96+8 = 1544   线上→总线(跨核 SPSC 安全)
 *   _usb_out    BusQueue<BusEnv,16>   16×96+8 = 1544   总线→线上(含在飞重传窗口)
 *   _push_out   BusRing <BusEnv,8>    8×96+8  = 776    实时推送(满则覆盖最旧)
 *   _svc_ring   BusRing <BusEnv,16>   16×96+8 = 1544   已重组待派发(可丢旧)
 *   _subring    3×BusRing<BusEnv,8>   3×776   = 2328   子环池(订阅方私有缓冲)
 *   _cmd_ring   BusRing <BusEnv,8>    776             ★预留吸收位, 本阶段不接线
 *   _asm        2×BusAsmSlot          2×1032  = 2064   分包重组暂存 2×1024B
 *   _subs       32×BusSub             32×12   = 384    订阅表(扁平, 线性扫描)
 *   _nak_req    BusQueue<BusNakReq,8> 8×4+8   = 40     入向拥塞/坏帧 → NAK 请求
 *   _streams    2×BusStreamSlot       16              出向流光标
 *   _stat/_counter/_subring_used/_initialized  552     诊断计数、循环计数与光标
 *   合计 11568 B < 12 KB 硬上限(实测值由编译期 static_assert 兜底, 表格仅供人读)。
 *
 * ── 最坏延迟(单次 task() 内, 全部有界, 无阻塞) ──────────────────────────
 *   入向: 16 帧 × (CRC 12..76B + 重组 memcpy ≤64B)
 *   派发: 16 条 × 32 项线性扫描订阅表
 *   出向: 16 槽扫描(到期判定/重传/回调/回收)
 *   NAK : 8 条
 *   → 每次 task() 的工作量固定上界, 不随消息大小无界增长(单片 ≤64B)。
 */

// 重组暂存单槽容量(单条消息上限)。
#define BUS_ASM_MAX        1024u
#define BUS_ASM_SLOTS      2u
// ★取舍说明★ 子环池原为 4 槽。线上头加 t_us(12→16B)使 BusEnv 92→96, 全部 96 个信封槽
// 合计 +384 B, 把 sizeof(Mai2Bus) 顶到 12344 B 超出 12 KB 硬上限 56 B。硬上限不放宽,
// 于是这里 4→3(省 776 B)。★为什么牺牲子环而不是别的★: _usb_in/_usb_out 决定线上吞吐与
// 重传窗口深度, _svc_ring 决定派发不丢旧, 都直接影响协议正确性; 子环只是"订阅方要私有缓冲
// 时借一个"的可选便利, 借不到时 subring_acquire() 显式返回 BUS_STREAM_NONE(调用方本就必须
// 处理该失败), 且本阶段实际借用方为 0。
#define BUS_SUBRING_SLOTS  3u
#define BUS_SUB_MAX        32u
#define BUS_TX_RETRY_MAX   3u      // ACK_REQ 传输的最大重传次数
#define BUS_STREAM_SLOTS   2u
#define BUS_STREAM_NONE    (-1)

// 订阅表项(扁平, 线性扫描; 32 项定长)。
struct BusSub {
    BusRxCb  cb;
    void*    ctx;
    uint8_t  msg_id;
    uint8_t  _used;

    void clear() { cb = nullptr; ctx = nullptr; msg_id = 0u; _used = 0u; }
};

// 分包重组槽: 只做"同 (msg_id,seq) 顺序分片拼接", 不理解内容。
struct BusAsmSlot {
    uint8_t  buf[BUS_ASM_MAX];
    uint16_t seq;
    uint16_t total_len;   // BUS_LEN_UNKNOWN 表示未知(流式不入本槽, 故此处必为已知)
    uint16_t recv_len;    // 已拼接字节数, 同时是下一片期望的 frag_off
    uint8_t  msg_id;
    uint8_t  _used;

    void clear() {
        seq = 0u; total_len = 0u; recv_len = 0u; msg_id = 0u; _used = 0u;
    }
};

// 出向流光标(传输层游标, 非业务状态): 流式发送不整包缓存, 只记 seq 与流内偏移。
struct BusStreamSlot {
    uint16_t seq;
    uint16_t off;
    uint8_t  msg_id;
    uint8_t  _used;
    uint8_t  _first_sent;
    uint8_t  _rsv;

    void clear() {
        seq = 0u; off = 0u; msg_id = 0u; _used = 0u; _first_sent = 0u; _rsv = 0u;
    }
};

// 运行计数(诊断用, 不参与控制流)。
struct BusStat {
    uint32_t tx_frag;
    uint32_t rx_frag;
    uint32_t rx_drop_crc;
    uint32_t rx_drop_full;
    uint32_t retrans;
    uint32_t timeout;
    uint32_t deliver;
    uint32_t svc_overwrite;
    uint32_t push_overwrite;

    void clear() {
        tx_frag = 0u; rx_frag = 0u; rx_drop_crc = 0u; rx_drop_full = 0u;
        retrans = 0u; timeout = 0u; deliver = 0u; svc_overwrite = 0u;
        push_overwrite = 0u;
    }
};

class Mai2Bus {
public:
    static Mai2Bus* getInstance();

    // 清空全部容器与光标, 已排队且带 cb 的信封以 ABORTED 终结(在本调用内触发)。
    bool init();

    // 唯一驱动点: 在 core0 loop() 内周期调用。非阻塞, 工作量有界。
    // 顺序: 入向解析 → 重组/派发 → NAK 生成 → 出向到期/重传/回调回收。
    void task();

    // ── 业务侧接口 ───────────────────────────────────────────────────────
    // 发一条完整消息(内部按 64B 分片)。
    //   timeout_us > 0 → 末片置 ACK_REQ, 进重传窗口; 到期未收 ACK 则重传, 耗尽回 TIMEOUT。
    //   timeout_us = 0 → 不要求 ACK, 交给传输层即视为 OK。
    // 返回 false = 未受理(队列不足/参数非法), 此时回调**不会**触发。
    // 受理成功后 cb 恰好触发一次(OK/TIMEOUT/PEER_NAK/ABORTED), 且只在 task() 内。
    bool publish(uint8_t msg_id, const uint8_t* data, uint16_t len,
                 BusCb cb = nullptr, void* ctx = nullptr, uint32_t timeout_us = 0u);

    // 推送一条实时数据。通道仅由 msg_id 标识, 无需 open/close, 同 id 反复调用即续期。
    // 每条带该 id 独立循环计数(到 max 归零续继), 接收方靠计数跳号发现丢包。
    // 实时语义: _push_out 满则覆盖最旧(计数照常递增, 故跳号可见), 不静默丢。
    // len 必须 <= BUS_INLINE_MAX(64), 超了返回 false(显式拒绝, 不截断)。
    bool push(uint8_t msg_id, const uint8_t* data, uint16_t len);

    // 注册接收回调(同一 msg_id 可多个订阅者)。满 32 项返回 false。
    bool subscribe(uint8_t msg_id, BusRxCb cb, void* ctx = nullptr);
    bool unsubscribe(uint8_t msg_id, BusRxCb cb);

    // ── 流式接口(不整包缓存, 逐片直发) ───────────────────────────────────
    int8_t stream_open(uint8_t msg_id);                                   // 失败返回 BUS_STREAM_NONE
    bool   stream_write(int8_t handle, const uint8_t* data, uint16_t len); // 满则 false(背压)
    bool   stream_close(int8_t handle);

    // ── 子环池: 订阅方需要自己的缓冲时借一个, 用完还回 ───────────────────
    int8_t                subring_acquire();
    BusRing<BusEnv, 8>*   subring(int8_t handle);
    bool                  subring_release(int8_t handle);

    // ── 传输层接线口(阶段2 由 USB 侧调用; 本阶段无调用方) ────────────────
    // 喂入一整帧线上信封。可在任意上下文调用(含中断/另一核): 只做解码+CRC+入队,
    // 不触发任何回调。返回 false = 显式失败(坏帧或入向满, 已记 NAK 请求)。
    bool wire_rx(const uint8_t* frame, uint16_t len);
    // 取一帧待发线上信封(消费者上下文 = 与 task() 同核)。无待发返回 false。
    bool wire_tx_pop(uint8_t* out, uint16_t cap, uint16_t& out_len);
    // 出向是否有待发帧(供传输层免拷贝轮询)。
    bool wire_tx_pending() const;

    // ★预留吸收位★: 未来把 host_cmd 命令流并入总线时, 由 protocol 侧改走 _cmd_ring。
    // 本阶段仅占位, 不接线、不碰 protocol/ 与 psoc/ 任何文件。
    bool cmd_ring_push(const BusEnv& env);   // 阶段1: 空实现, 恒 false
    bool cmd_ring_pop(BusEnv& out);          // 阶段1: 空实现, 恒 false

    const BusStat& stat() const { return _stat; }

private:
    Mai2Bus();
    Mai2Bus(const Mai2Bus&) = delete;
    Mai2Bus& operator=(const Mai2Bus&) = delete;

    // ── 出向 ──
    bool _tx_space(uint16_t frags) const;
    bool _tx_enqueue(uint8_t msg_id, uint16_t seq, uint16_t total_len, uint16_t frag_off,
                     const uint8_t* data, uint8_t frag_len, uint8_t flags,
                     BusCb cb, void* ctx, uint32_t timeout_us);
    void _tx_reap();                     // 到期/重传/回调/回收, 回调唯一触发点
    void _tx_finish(BusEnv& e, BusResult r);
    BusEnv* _tx_find_inflight(uint8_t msg_id, uint16_t counter);

    // ── 入向 ──
    void _rx_pump();                     // _usb_in → 重组/流式/ACK 处理
    void _rx_handle(BusEnv& e);
    void _nak_pump();                    // BusNakReq → NAK 信封
    void _nak_mark(uint8_t msg_id, uint16_t seq);

    // ── 重组 ──
    BusAsmSlot* _asm_find(uint8_t msg_id, uint16_t seq);
    BusAsmSlot* _asm_alloc(uint8_t msg_id, uint16_t seq, uint16_t total_len);

    // ── 派发 ──
    void _svc_pump();                    // _svc_ring → 订阅表
    void _dispatch(uint8_t msg_id, uint16_t counter, const uint8_t* data, uint16_t len);

    uint16_t _next_counter(uint8_t msg_id);

    // ── 容器(容量见文件头表) ──
    BusQueue<BusEnv, 16>    _usb_in;
    BusQueue<BusEnv, 16>    _usb_out;
    BusRing<BusEnv, 8>      _push_out;
    BusRing<BusEnv, 16>     _svc_ring;
    BusRing<BusEnv, 8>      _subring[BUS_SUBRING_SLOTS];
    BusRing<BusEnv, 8>      _cmd_ring;   // ★预留, 本阶段不接线
    BusQueue<BusNakReq, 8>  _nak_req;
    BusAsmSlot              _asm[BUS_ASM_SLOTS];
    BusSub                  _subs[BUS_SUB_MAX];
    BusStreamSlot           _streams[BUS_STREAM_SLOTS];
    BusStat                 _stat;

    uint16_t _counter[256];
    uint8_t  _subring_used;              // 位掩码, bit i = _subring[i] 已借出
    bool     _initialized;

    static Mai2Bus* _instance;
};
