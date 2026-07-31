#include "bus_core.h"
#include <pico/stdlib.h>   // time_us_32()
#include <cstring>

namespace {
// 出向"未被传输层取走"的滞留上限: 防止无人接线时槽位被永久占住(仍是非阻塞, 只是到期终结)。
constexpr uint32_t BUS_TX_LINGER_US = 200000u;   // 200 ms
// 单次 task() 的每阶段处理上限 = 对应容器容量, 保证工作量有界。
constexpr uint32_t BUS_RX_BUDGET = 16u;
constexpr uint32_t BUS_SVC_BUDGET = 16u;
constexpr uint32_t BUS_NAK_BUDGET = 8u;

// 有符号时差判定, 免受 time_us_32() 32 位回绕影响。
inline bool bus_expired(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}
}   // namespace

Mai2Bus* Mai2Bus::_instance = nullptr;

Mai2Bus::Mai2Bus() : _subring_used(0u), _initialized(false) {
    _push_out.clear();
    for (uint32_t i = 0u; i < BUS_SUB_MAX; i++) {
        _subs[i].clear();
    }
    for (uint32_t i = 0u; i < 256u; i++) {
        _counter[i] = 0u;
    }
    for (uint32_t i = 0u; i < BUS_ASM_SLOTS; i++) {
        _asm[i].clear();
    }
    for (uint32_t i = 0u; i < BUS_STREAM_SLOTS; i++) {
        _streams[i].clear();
    }
    _stat.clear();
}

Mai2Bus* Mai2Bus::getInstance() {
    if (!_instance) {
        _instance = new Mai2Bus();
    }
    return _instance;
}

bool Mai2Bus::init() {
    // 清场前先把已排队且带 cb 的信封以 ABORTED 终结, 保证"受理即恰好回调一次"。
    for (uint32_t i = 0u; i < _usb_out.size(); i++) {
        BusEnv* e = _usb_out.at(i);
        if ((e != nullptr) && (e->cb != nullptr)) {
            BusCb cb = e->cb;
            e->cb = nullptr;
            cb(e->hdr.seq, BusResult::ABORTED, e->ctx);
        }
    }

    _usb_in.clear();
    _usb_out.clear();
    _push_out.clear();
    _svc_ring.clear();
    _cmd_ring.clear();
    _nak_req.clear();
    for (uint32_t i = 0u; i < BUS_SUBRING_SLOTS; i++) {
        _subring[i].clear();
    }
    for (uint32_t i = 0u; i < BUS_ASM_SLOTS; i++) {
        _asm[i].clear();
    }
    for (uint32_t i = 0u; i < BUS_STREAM_SLOTS; i++) {
        _streams[i].clear();
    }
    _stat.clear();
    _initialized = true;
    return true;
}

// ── 唯一驱动点 ───────────────────────────────────────────────────────────
void Mai2Bus::task() {
    if (!_initialized) {
        return;
    }
    _rx_pump();     // 线上入向 → ACK/NAK 匹配、流式直发、分片重组
    _svc_pump();    // 已就绪信封 → 订阅表
    _nak_pump();    // 入向拥塞/坏帧 → NAK 信封(显式失败)
    _tx_reap();     // 出向到期/重传/回调/回收 ★回调唯一触发点★
}

uint16_t Mai2Bus::_next_counter(uint8_t msg_id) {
    const uint16_t counter = _counter[msg_id];
    // 到 max 自动归零续继。
    _counter[msg_id] = (uint16_t)((_counter[msg_id] + 1u) & 0xFFFFu);
    return counter;
}

// ── 出向 ─────────────────────────────────────────────────────────────────
bool Mai2Bus::_tx_space(uint16_t frags) const {
    const uint32_t used = _usb_out.size();
    return (used + (uint32_t)frags) <= _usb_out.capacity();
}

bool Mai2Bus::_tx_enqueue(uint8_t msg_id, uint16_t seq, uint16_t total_len, uint16_t frag_off,
                          const uint8_t* data, uint8_t frag_len, uint8_t flags,
                          BusCb cb, void* ctx, uint32_t timeout_us) {
    BusEnv e;
    e.clear();
    e.hdr.magic = BUS_MAGIC;
    e.hdr.msg_id = msg_id;
    e.hdr.seq = seq;
    e.hdr.total_len = total_len;
    e.hdr.frag_off = frag_off;
    e.hdr.frag_len = frag_len;
    e.hdr.flags = flags;
    // ★出向时间戳唯一填充点之一★: 生成这一刻的 time_us_32()。publish/stream_write/ACK/NAK
    // 全部经本函数 ⇒ 无需在各调用点重复填。重传不重填(t_us 表示"信封生成时刻", 非"上线时刻"),
    // 这样对端拿到的始终是事件发生时间, 与 host_cmd 尾戳/KBD_GET_EDGES 同口径。
    e.hdr.t_us = time_us_32();
    if ((frag_len > 0u) && (data != nullptr)) {
        memcpy(e.payload, data, frag_len);
    }
    e.hdr.crc = bus_crc(e.hdr, e.payload);

    e.cb = cb;
    e.ctx = ctx;
    e.state = BusState::PENDING;
    const uint32_t linger = ((flags & BUS_FLAG_ACK_REQ) != 0u) ? timeout_us : BUS_TX_LINGER_US;
    e.deadline_us = time_us_32() + linger;
    return _usb_out.push(e);
}

bool Mai2Bus::publish(uint8_t msg_id, const uint8_t* data, uint16_t len,
                      BusCb cb, void* ctx, uint32_t timeout_us) {
    if (!_initialized || (len > BUS_ASM_MAX) || ((len > 0u) && (data == nullptr))) {
        return false;
    }

    const uint16_t frags = (len == 0u) ? 1u : (uint16_t)((len + BUS_INLINE_MAX - 1u) / BUS_INLINE_MAX);
    // 全有或全无: 空间不足直接背压, 绝不半条入队。
    if (!_tx_space(frags)) {
        return false;
    }

    const uint16_t seq = _next_counter(msg_id);
    const bool want_ack = (timeout_us > 0u);
    uint16_t off = 0u;
    for (uint16_t i = 0u; i < frags; i++) {
        const uint16_t remain = (uint16_t)(len - off);
        const uint8_t flen = (remain > BUS_INLINE_MAX) ? (uint8_t)BUS_INLINE_MAX : (uint8_t)remain;
        const bool last = (i + 1u) == frags;

        uint8_t flags = 0u;
        if (i == 0u) { flags |= BUS_FLAG_FIRST; }
        if (last) {
            flags |= BUS_FLAG_LAST;
            if (want_ack) { flags |= BUS_FLAG_ACK_REQ; }
        }
        // 回调只挂在末片 → 一次 publish 恰好一次回调。
        (void)_tx_enqueue(msg_id, seq, len, off, data + off, flen, flags,
                          last ? cb : nullptr, last ? ctx : nullptr, timeout_us);
        _stat.tx_frag++;
        off = (uint16_t)(off + flen);
    }
    return true;
}

bool Mai2Bus::push(uint8_t msg_id, const uint8_t* data, uint16_t len) {
    if (!_initialized || (len > BUS_INLINE_MAX) || ((len > 0u) && (data == nullptr))) {
        return false;
    }

    BusEnv e;
    e.clear();
    e.hdr.msg_id = msg_id;
    e.hdr.seq = _next_counter(msg_id);
    e.hdr.total_len = BUS_LEN_UNKNOWN;
    e.hdr.frag_off = 0u;
    e.hdr.frag_len = (uint8_t)len;
    e.hdr.flags = BUS_FLAG_STREAM;
    // ★出向时间戳唯一填充点之二★(push 不经 _tx_enqueue, 直入 _push_out)。
    e.hdr.t_us = time_us_32();
    if (len > 0u) {
        memcpy(e.payload, data, len);
    }
    e.hdr.crc = bus_crc(e.hdr, e.payload);
    e.state = BusState::PENDING;
    if (!_push_out.push(e)) {
        _stat.push_overwrite++;
    }
    _stat.tx_frag++;
    return true;
}

BusEnv* Mai2Bus::_tx_find_inflight(uint8_t msg_id, uint16_t counter) {
    for (uint32_t i = 0u; i < _usb_out.size(); i++) {
        BusEnv* e = _usb_out.at(i);
        if ((e != nullptr) && (e->state == BusState::INFLIGHT) &&
            (e->hdr.msg_id == msg_id) && (e->hdr.seq == counter)) {
            return e;
        }
    }
    return nullptr;
}

void Mai2Bus::_tx_finish(BusEnv& e, BusResult r) {
    e.result = r;
    e.state = BusState::DONE;
}

void Mai2Bus::_tx_reap() {
    const uint32_t now = time_us_32();

    for (uint32_t i = 0u; i < _usb_out.size(); i++) {
        BusEnv* e = _usb_out.at(i);
        if (e == nullptr) {
            break;
        }
        if (e->state == BusState::PENDING) {
            // 传输层长期未取 → 到期以 TIMEOUT 终结, 释放槽位(非阻塞, 无自旋)。
            if (bus_expired(now, e->deadline_us)) {
                _stat.timeout++;
                _tx_finish(*e, BusResult::TIMEOUT);
            }
        } else if (e->state == BusState::INFLIGHT) {
            if (bus_expired(now, e->deadline_us)) {
                if (e->retries < BUS_TX_RETRY_MAX) {
                    e->retries++;
                    e->state = BusState::PENDING;   // 重回待发 → 由 wire_tx_pop 重发
                    e->deadline_us = now + BUS_TX_LINGER_US;
                    _stat.retrans++;
                } else {
                    _stat.timeout++;
                    _tx_finish(*e, BusResult::TIMEOUT);
                }
            }
        }

        // ★回调唯一触发点★: 先摘 cb 再调用 → 结构上不可能触发第二次。
        if ((e->state == BusState::DONE) && (e->cb != nullptr)) {
            BusCb cb = e->cb;
            e->cb = nullptr;
            cb(e->hdr.seq, e->result, e->ctx);
        }
    }

    // 队首已终结的槽位回收(FIFO 语义; 未终结的槽必有 deadline → 终会推进, 不会永久卡住)。
    while (true) {
        BusEnv* head = _usb_out.peek();
        if ((head == nullptr) || (head->state != BusState::DONE)) {
            break;
        }
        (void)_usb_out.drop();
    }
}

// ── 流式出向 ─────────────────────────────────────────────────────────────
int8_t Mai2Bus::stream_open(uint8_t msg_id) {
    if (!_initialized) {
        return BUS_STREAM_NONE;
    }
    for (uint8_t i = 0u; i < BUS_STREAM_SLOTS; i++) {
        if (_streams[i]._used == 0u) {
            _streams[i].clear();
            _streams[i]._used = 1u;
            _streams[i].msg_id = msg_id;
            _streams[i].seq = _next_counter(msg_id);
            return (int8_t)i;
        }
    }
    return BUS_STREAM_NONE;
}

bool Mai2Bus::stream_write(int8_t handle, const uint8_t* data, uint16_t len) {
    if ((handle < 0) || ((uint8_t)handle >= BUS_STREAM_SLOTS) || (data == nullptr) || (len == 0u)) {
        return false;
    }
    BusStreamSlot& s = _streams[(uint8_t)handle];
    if (s._used == 0u) {
        return false;
    }

    const uint16_t frags = (uint16_t)((len + BUS_INLINE_MAX - 1u) / BUS_INLINE_MAX);
    if (!_tx_space(frags)) {
        return false;   // 背压: 一片都不入队, 调用方自己重试
    }

    uint16_t off = 0u;
    for (uint16_t i = 0u; i < frags; i++) {
        const uint16_t remain = (uint16_t)(len - off);
        const uint8_t flen = (remain > BUS_INLINE_MAX) ? (uint8_t)BUS_INLINE_MAX : (uint8_t)remain;
        uint8_t flags = BUS_FLAG_STREAM;
        if (s._first_sent == 0u) {
            flags |= BUS_FLAG_FIRST;
            s._first_sent = 1u;
        }
        // 流式不整包缓存: total_len 未知, frag_off 为流内累计偏移(允许回绕)。
        (void)_tx_enqueue(s.msg_id, s.seq, BUS_LEN_UNKNOWN, s.off, data + off, flen, flags,
                          nullptr, nullptr, 0u);
        _stat.tx_frag++;
        s.off = (uint16_t)(s.off + flen);
        off = (uint16_t)(off + flen);
    }
    return true;
}

bool Mai2Bus::stream_close(int8_t handle) {
    if ((handle < 0) || ((uint8_t)handle >= BUS_STREAM_SLOTS)) {
        return false;
    }
    BusStreamSlot& s = _streams[(uint8_t)handle];
    if (s._used == 0u) {
        return false;
    }
    if (!_tx_space(1u)) {
        return false;   // 收尾帧也走背压, 不静默丢
    }
    (void)_tx_enqueue(s.msg_id, s.seq, BUS_LEN_UNKNOWN, s.off, nullptr, 0u,
                      (uint8_t)(BUS_FLAG_STREAM | BUS_FLAG_LAST), nullptr, nullptr, 0u);
    _stat.tx_frag++;
    s.clear();
    return true;
}

// ── 订阅 ─────────────────────────────────────────────────────────────────
bool Mai2Bus::subscribe(uint8_t msg_id, BusRxCb cb, void* ctx) {
    if (cb == nullptr) {
        return false;
    }
    for (uint32_t i = 0u; i < BUS_SUB_MAX; i++) {
        if (_subs[i]._used == 0u) {
            _subs[i].cb = cb;
            _subs[i].ctx = ctx;
            _subs[i].msg_id = msg_id;
            _subs[i]._used = 1u;
            return true;
        }
    }
    return false;   // 满 32 项: 显式失败
}

bool Mai2Bus::unsubscribe(uint8_t msg_id, BusRxCb cb) {
    for (uint32_t i = 0u; i < BUS_SUB_MAX; i++) {
        if ((_subs[i]._used != 0u) && (_subs[i].msg_id == msg_id) && (_subs[i].cb == cb)) {
            _subs[i].clear();
            return true;
        }
    }
    return false;
}

void Mai2Bus::_dispatch(uint8_t msg_id, uint16_t counter, const uint8_t* data, uint16_t len) {
    for (uint32_t i = 0u; i < BUS_SUB_MAX; i++) {
        if ((_subs[i]._used != 0u) && (_subs[i].msg_id == msg_id) && (_subs[i].cb != nullptr)) {
            _subs[i].cb(msg_id, counter, data, len, _subs[i].ctx);
            _stat.deliver++;
        }
    }
}

void Mai2Bus::_svc_pump() {
    BusEnv e;
    for (uint32_t n = 0u; n < BUS_SVC_BUDGET; n++) {
        if (!_svc_ring.pop(e)) {
            break;
        }
        _dispatch(e.hdr.msg_id, e.hdr.seq, e.payload, e.hdr.frag_len);
    }
}

// ── 重组 ─────────────────────────────────────────────────────────────────
BusAsmSlot* Mai2Bus::_asm_find(uint8_t msg_id, uint16_t seq) {
    for (uint32_t i = 0u; i < BUS_ASM_SLOTS; i++) {
        if ((_asm[i]._used != 0u) && (_asm[i].msg_id == msg_id) && (_asm[i].seq == seq)) {
            return &_asm[i];
        }
    }
    return nullptr;
}

BusAsmSlot* Mai2Bus::_asm_alloc(uint8_t msg_id, uint16_t seq, uint16_t total_len) {
    for (uint32_t i = 0u; i < BUS_ASM_SLOTS; i++) {
        if (_asm[i]._used == 0u) {
            _asm[i].clear();
            _asm[i]._used = 1u;
            _asm[i].msg_id = msg_id;
            _asm[i].seq = seq;
            _asm[i].total_len = total_len;
            return &_asm[i];
        }
    }
    return nullptr;   // 暂存全忙 → 调用方回 NAK, 不静默丢
}

// ── 入向 ─────────────────────────────────────────────────────────────────
bool Mai2Bus::wire_rx(const uint8_t* frame, uint16_t len) {
    // 任意上下文(含中断/另一核)可调: 只解码+校验+入队, 不触发任何回调。
    if (!_initialized || (frame == nullptr) || (len < BUS_HDR_SIZE)) {
        return false;
    }

    BusEnv e;
    e.clear();
    if (!bus_hdr_decode(frame, e.hdr)) {
        _stat.rx_drop_crc++;
        return false;
    }
    if ((e.hdr.frag_len > BUS_INLINE_MAX) ||
        ((uint32_t)BUS_HDR_SIZE + e.hdr.frag_len > len)) {
        _stat.rx_drop_crc++;
        _nak_mark(e.hdr.msg_id, e.hdr.seq);
        return false;
    }
    if (e.hdr.frag_len > 0u) {
        memcpy(e.payload, frame + BUS_HDR_SIZE, e.hdr.frag_len);
    }
    if (bus_crc(e.hdr, e.payload) != e.hdr.crc) {
        _stat.rx_drop_crc++;
        _nak_mark(e.hdr.msg_id, e.hdr.seq);
        return false;
    }

    e.state = BusState::PENDING;
    if (!_usb_in.push(e)) {
        // 入向满: 显式失败 —— 记 NAK 请求由 task() 回帧, 绝不静默丢。
        _stat.rx_drop_full++;
        _nak_mark(e.hdr.msg_id, e.hdr.seq);
        return false;
    }
    _stat.rx_frag++;
    return true;
}

void Mai2Bus::_rx_pump() {
    BusEnv e;
    for (uint32_t n = 0u; n < BUS_RX_BUDGET; n++) {
        if (!_usb_in.pop(e)) {
            break;
        }
        _rx_handle(e);
    }
}

void Mai2Bus::_rx_handle(BusEnv& e) {
    const uint8_t flags = e.hdr.flags;

    // 1) ACK/NAK: 只影响出向重传窗口, 不派发给订阅方。
    if ((flags & BUS_FLAG_IS_ACK) != 0u) {
        BusEnv* tx = _tx_find_inflight(e.hdr.msg_id, e.hdr.seq);
        if (tx != nullptr) {
            _tx_finish(*tx, BusResult::OK);
        }
        return;
    }
    if ((flags & BUS_FLAG_NAK) != 0u) {
        BusEnv* tx = _tx_find_inflight(e.hdr.msg_id, e.hdr.seq);
        if (tx != nullptr) {
            _tx_finish(*tx, BusResult::PEER_NAK);
        }
        return;
    }

    // 2) 对端要求 ACK → 立刻排一帧 ACK(排不下则不回, 由对端重传兜底)。
    if ((flags & BUS_FLAG_ACK_REQ) != 0u) {
        if (_tx_space(1u)) {
            (void)_tx_enqueue(e.hdr.msg_id, e.hdr.seq, 0u, 0u, nullptr, 0u,
                              BUS_FLAG_IS_ACK, nullptr, nullptr, 0u);
        }
    }

    // 3) 流式: 不整包缓存, 逐片进 _svc_ring 直发(满则覆盖最旧 = 实时可丢旧语义)。
    if ((flags & BUS_FLAG_STREAM) != 0u) {
        if (!_svc_ring.push(e)) {
            _stat.svc_overwrite++;
        }
        return;
    }

    // 4) 单片整条消息: 直接进 _svc_ring, 不占重组槽。
    if (((flags & BUS_FLAG_FIRST) != 0u) && ((flags & BUS_FLAG_LAST) != 0u)) {
        if (!_svc_ring.push(e)) {
            _stat.svc_overwrite++;
        }
        return;
    }

    // 5) 多片: 顺序拼接进重组槽; 完成时就地派发(>64B 无法塞回 64B 信封)。
    BusAsmSlot* slot = ((flags & BUS_FLAG_FIRST) != 0u)
        ? _asm_alloc(e.hdr.msg_id, e.hdr.seq, e.hdr.total_len)
        : _asm_find(e.hdr.msg_id, e.hdr.seq);
    if (slot == nullptr) {
        _stat.rx_drop_full++;
        _nak_mark(e.hdr.msg_id, e.hdr.seq);   // 无槽/失序: 显式 NAK
        return;
    }
    // 偏移必须严格接续, 且总长受暂存容量约束; 违反即弃槽并 NAK。
    if ((e.hdr.frag_off != slot->recv_len) ||
        ((uint32_t)slot->recv_len + e.hdr.frag_len > BUS_ASM_MAX) ||
        (slot->total_len > BUS_ASM_MAX)) {
        slot->clear();
        _stat.rx_drop_crc++;
        _nak_mark(e.hdr.msg_id, e.hdr.seq);
        return;
    }
    if (e.hdr.frag_len > 0u) {
        memcpy(slot->buf + slot->recv_len, e.payload, e.hdr.frag_len);
        slot->recv_len = (uint16_t)(slot->recv_len + e.hdr.frag_len);
    }
    if ((flags & BUS_FLAG_LAST) != 0u) {
        if (slot->recv_len == slot->total_len) {
            _dispatch(slot->msg_id, slot->seq, slot->buf, slot->recv_len);
        } else {
            _stat.rx_drop_crc++;
            _nak_mark(e.hdr.msg_id, e.hdr.seq);
        }
        slot->clear();
    }
}

// ── NAK ──────────────────────────────────────────────────────────────────
void Mai2Bus::_nak_mark(uint8_t msg_id, uint16_t seq) {
    BusNakReq r;
    r.clear();
    r.msg_id = msg_id;
    r.seq = seq;
    (void)_nak_req.push(r);   // 已积压 8 条时不再叠加(计数已记, 非静默数据丢弃)
}

void Mai2Bus::_nak_pump() {
    for (uint32_t n = 0u; n < BUS_NAK_BUDGET; n++) {
        BusNakReq* r = _nak_req.peek();
        if (r == nullptr) {
            break;
        }
        // 出向满 → 保留请求, 下轮 task() 再发(背压, 不丢)。
        if (!_tx_enqueue(r->msg_id, r->seq, 0u, 0u, nullptr, 0u,
                         BUS_FLAG_NAK, nullptr, nullptr, 0u)) {
            break;
        }
        (void)_nak_req.drop();
    }
}

// ── 传输层接线口 ─────────────────────────────────────────────────────────
bool Mai2Bus::wire_tx_pending() const {
    return !_usb_out.empty() || !_push_out.empty();
}

bool Mai2Bus::wire_tx_pop(uint8_t* out, uint16_t cap, uint16_t& out_len) {
    out_len = 0u;
    if ((out == nullptr) || (cap < BUS_HDR_SIZE)) {
        return false;
    }

    // 可靠流量优先。可靠流量令推送饥饿时，接收方看到计数跳号是设计预期而非缺陷。
    for (uint32_t i = 0u; i < _usb_out.size(); i++) {
        BusEnv* e = _usb_out.at(i);
        if (e == nullptr) {
            break;
        }
        if (e->state != BusState::PENDING) {
            continue;
        }
        const uint16_t need = (uint16_t)(BUS_HDR_SIZE + e->hdr.frag_len);
        if (cap < need) {
            return false;
        }
        bus_hdr_encode(e->hdr, out);
        if (e->hdr.frag_len > 0u) {
            memcpy(out + BUS_HDR_SIZE, e->payload, e->hdr.frag_len);
        }
        out_len = need;

        if ((e->hdr.flags & BUS_FLAG_ACK_REQ) != 0u) {
            e->state = BusState::INFLIGHT;   // 等 ACK; 到期由 _tx_reap 重传
            e->deadline_us = time_us_32() + BUS_TX_LINGER_US;
        } else {
            // 不要求 ACK: 交给传输层即终结; 回调仍留给 task() 触发。
            _tx_finish(*e, BusResult::OK);
        }
        return true;
    }

    BusEnv* e = _push_out.peek();
    if (e == nullptr) {
        return false;
    }
    const uint16_t need = (uint16_t)(BUS_HDR_SIZE + e->hdr.frag_len);
    if (cap < need) {
        return false;
    }
    bus_hdr_encode(e->hdr, out);
    if (e->hdr.frag_len > 0u) {
        memcpy(out + BUS_HDR_SIZE, e->payload, e->hdr.frag_len);
    }
    out_len = need;
    (void)_push_out.drop();
    return true;
}

// ── 子环池 ───────────────────────────────────────────────────────────────
int8_t Mai2Bus::subring_acquire() {
    for (uint8_t i = 0u; i < BUS_SUBRING_SLOTS; i++) {
        const uint8_t bit = (uint8_t)(1u << i);
        if ((_subring_used & bit) == 0u) {
            _subring_used = (uint8_t)(_subring_used | bit);
            _subring[i].clear();
            return (int8_t)i;
        }
    }
    return BUS_STREAM_NONE;
}

BusRing<BusEnv, 8>* Mai2Bus::subring(int8_t handle) {
    if ((handle < 0) || ((uint8_t)handle >= BUS_SUBRING_SLOTS)) {
        return nullptr;
    }
    if ((_subring_used & (uint8_t)(1u << (uint8_t)handle)) == 0u) {
        return nullptr;
    }
    return &_subring[(uint8_t)handle];
}

bool Mai2Bus::subring_release(int8_t handle) {
    if ((handle < 0) || ((uint8_t)handle >= BUS_SUBRING_SLOTS)) {
        return false;
    }
    const uint8_t bit = (uint8_t)(1u << (uint8_t)handle);
    if ((_subring_used & bit) == 0u) {
        return false;
    }
    _subring[(uint8_t)handle].clear();
    _subring_used = (uint8_t)(_subring_used & (uint8_t)~bit);
    return true;
}

// ── ★预留吸收位★(阶段1 空实现; 阶段2 才把 host_cmd 命令流并进来) ────────
bool Mai2Bus::cmd_ring_push(const BusEnv& env) {
    (void)env;
    return false;
}

bool Mai2Bus::cmd_ring_pop(BusEnv& out) {
    (void)out;
    return false;
}

// 静态占用硬上限: 超 12KB 立即编译失败(不放宽)。
static_assert(sizeof(Mai2Bus) <= 12u * 1024u, "Mai2Bus 静态占用超过 12KB 预算");
// 实测值锁定(与 bus_core.h 头部容量表同源): 头 12→16B 后 BusEnv 96B、子环 3 槽 ⇒ 11568 B。
// 改动任何容器容量都会撞这条, 届时按新值同步更新上表 —— 让"表格与真实占用一致"由编译器保证。
static_assert(sizeof(Mai2Bus) == 11568u, "Mai2Bus 实测应为 11568 字节(容量表已过期?)");
