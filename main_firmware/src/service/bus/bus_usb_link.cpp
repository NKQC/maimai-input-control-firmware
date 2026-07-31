#include "bus_usb_link.h"

#include "bus_core.h"
#include "../../protocol/host_cmd/host_cmd.h"

namespace {
constexpr uint16_t BUS_XFER_RESPONSE_HEADER = 4u;
constexpr uint8_t BUS_XFER_FLAG_TX_PENDING = 0x01u;

// ── 自测钩子: 仅当 BUS_XFER 请求 payload 长度**恰好 1 字节**时生效 ──────────────
// 判据(与正常数据路径天然不冲突, 不靠约定自律):
//   0 字节   = 纯轮询(只取出向帧, 不喂入);
//   ≥16 字节 = 一条或多条线上信封流(单帧最小长度 = BUS_HDR_SIZE = 16);
//   恰好 1 字节在正常路径上不可能出现 —— 它既不是空轮询, 也放不下一个信封头,
//   走原来的循环只会立刻记 rejected 并 break, 没有任何正常语义被占用。
// 三个钩子都只读诊断计数 / 只调总线自身已有的对外接口(stream_*/subring_*),
// **不改变正常数据路径的任何行为**: 不喂入信封、不排空出向队列、不动订阅表。
constexpr uint8_t BUS_XFER_PROBE_STAT = 0x01u;     // 响应体 = BusStat 9 个 u32(LE), 36B
constexpr uint8_t BUS_XFER_PROBE_STREAM = 0x02u;   // 固件自发一条流: open → write×3 → close
constexpr uint8_t BUS_XFER_PROBE_SUBRING = 0x03u;  // 子环池借满/越界/全还/再借
constexpr uint8_t BUS_XFER_PROBE_STREAM_MSG = 0x11u;
constexpr uint8_t BUS_XFER_PROBE_STREAM_CHUNKS = 3u;
constexpr uint8_t BUS_XFER_PROBE_CHUNK_LEN = 8u;
constexpr uint32_t BUS_XFER_STAT_FIELDS = 9u;

inline void _put_le32(uint8_t* out, uint32_t v) {
    out[0] = (uint8_t)(v & 0xFFu);
    out[1] = (uint8_t)((v >> 8) & 0xFFu);
    out[2] = (uint8_t)((v >> 16) & 0xFFu);
    out[3] = (uint8_t)((v >> 24) & 0xFFu);
}

// HostFrame 含 payload[4096], 放栈上会在 USB 分发路径上吃掉 ~4.1KB 栈
// (同样的坑 usb_comm.h:35 已踩过并注释在案)。改为文件级静态复用:
// BUS_XFER 只在 core0 命令分发里被调用, 单线程独占, 无重入。
// 同时省掉每次调用 clear() 对 4096B 的 memset —— encode_frame 只读 len 字节,
// 而 payload[0..len) 每次都被完整写满, 无残留可读。
HostFrame _resp_frame;
}

BusUsbLink* BusUsbLink::_instance = nullptr;

BusUsbLink::BusUsbLink() : _initialized(false) {}

BusUsbLink* BusUsbLink::getInstance() {
    if (_instance == nullptr) {
        _instance = new BusUsbLink();
    }
    return _instance;
}

bool BusUsbLink::init() {
    if (_initialized) {
        return true;
    }
    HostCmdDispatcher::getInstance()->register_handler(HostCmd::BUS_XFER, _handle_xfer);
    _initialized = true;
    return true;
}

void BusUsbLink::_increment(uint8_t& value) {
    if (value != 0xFFu) {
        value++;
    }
}

uint16_t BusUsbLink::_probe(uint8_t code, uint8_t* body) {
    Mai2Bus* bus = Mai2Bus::getInstance();

    if (code == BUS_XFER_PROBE_STAT) {
        const BusStat& s = bus->stat();
        // 字段序写死, 与主机侧解析同源: tx_frag, rx_frag, rx_drop_crc, rx_drop_full,
        // retrans, timeout, deliver, svc_overwrite, push_overwrite。
        const uint32_t fields[BUS_XFER_STAT_FIELDS] = {
            s.tx_frag, s.rx_frag, s.rx_drop_crc, s.rx_drop_full,
            s.retrans, s.timeout, s.deliver, s.svc_overwrite, s.push_overwrite
        };
        for (uint32_t i = 0u; i < BUS_XFER_STAT_FIELDS; i++) {
            _put_le32(body + (i * 4u), fields[i]);
        }
        return (uint16_t)(BUS_XFER_STAT_FIELDS * 4u);
    }

    if (code == BUS_XFER_PROBE_STREAM) {
        // 响应体 = [open_ok, w0, w1, w2, close_ok]; 帧本身随后由普通轮询排空。
        const int8_t handle = bus->stream_open(BUS_XFER_PROBE_STREAM_MSG);
        const bool opened = (handle != BUS_STREAM_NONE);
        body[0] = opened ? 1u : 0u;
        for (uint8_t i = 0u; i < BUS_XFER_PROBE_STREAM_CHUNKS; i++) {
            uint8_t chunk[BUS_XFER_PROBE_CHUNK_LEN];
            memset(chunk, (uint8_t)(0xA0u + i), sizeof(chunk));
            body[1u + i] = (opened && bus->stream_write(handle, chunk, sizeof(chunk))) ? 1u : 0u;
        }
        body[1u + BUS_XFER_PROBE_STREAM_CHUNKS] =
            (opened && bus->stream_close(handle)) ? 1u : 0u;
        return (uint16_t)(2u + BUS_XFER_PROBE_STREAM_CHUNKS);
    }

    if (code == BUS_XFER_PROBE_SUBRING) {
        // 响应体 = [acq0..acq(N-1), overflow_rejected, all_released, reacquire_ok]
        // 共 BUS_SUBRING_SLOTS+3 字节(N = BUS_SUBRING_SLOTS, 现为 3)。
        int8_t handles[BUS_SUBRING_SLOTS];
        for (uint8_t i = 0u; i < BUS_SUBRING_SLOTS; i++) {
            handles[i] = bus->subring_acquire();
            body[i] = (handles[i] != BUS_STREAM_NONE) ? 1u : 0u;
        }
        // ★容量硬上限判据★: 借满之后再借一次必须失败, 不允许静默复用已借出的槽。
        const int8_t overflow = bus->subring_acquire();
        body[BUS_SUBRING_SLOTS] = (overflow == BUS_STREAM_NONE) ? 1u : 0u;
        if (overflow != BUS_STREAM_NONE) {
            (void)bus->subring_release(overflow);
        }
        uint8_t released = 0u;
        for (uint8_t i = 0u; i < BUS_SUBRING_SLOTS; i++) {
            if (bus->subring_release(handles[i])) {
                released++;
            }
        }
        body[BUS_SUBRING_SLOTS + 1u] = (released == BUS_SUBRING_SLOTS) ? 1u : 0u;
        const int8_t again = bus->subring_acquire();
        body[BUS_SUBRING_SLOTS + 2u] = (again != BUS_STREAM_NONE) ? 1u : 0u;
        (void)bus->subring_release(again);
        return (uint16_t)(BUS_SUBRING_SLOTS + 3u);
    }

    return 0u;   // 未知请求码: 空体, 主机据此判定钩子不被支持
}

void BusUsbLink::_handle_xfer(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
    if ((resp_buf == nullptr) || (resp_len == nullptr)) {
        return;
    }

    // ── 自测钩子分流(见文件头注释): 长度恰好 1 → 不进信封解析、不排空出向 ──
    if (frame.len == 1u) {
        HostFrame& probe = _resp_frame;
        probe.cmd = static_cast<uint8_t>(HostCmd::BUS_XFER);
        probe.flags = HOST_CMD_FLAG_RESPONSE;
        probe.seq = frame.seq;
        const uint16_t body = _probe(frame.payload[0], probe.payload + BUS_XFER_RESPONSE_HEADER);
        probe.payload[0] = 0u;   // accepted: 钩子不喂入信封
        probe.payload[1] = 0u;   // rejected: 同上
        probe.payload[2] = 0u;   // frames_out: 钩子不排空出向队列
        probe.payload[3] = Mai2Bus::getInstance()->wire_tx_pending() ? BUS_XFER_FLAG_TX_PENDING : 0u;
        probe.len = (uint16_t)(BUS_XFER_RESPONSE_HEADER + body);
        *resp_len = HostCmdCodec::encode_frame(probe, resp_buf, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    Mai2Bus* bus = Mai2Bus::getInstance();
    uint8_t accepted = 0u;
    uint8_t rejected = 0u;
    uint16_t in_pos = 0u;
    while (in_pos < frame.len) {
        const uint16_t remaining = (uint16_t)(frame.len - in_pos);
        if (remaining < BUS_HDR_SIZE) {
            _increment(rejected);
            break;
        }

        const uint8_t frag_len = frame.payload[in_pos + 8u];
        const uint16_t env_len = (uint16_t)(BUS_HDR_SIZE + frag_len);
        if ((frag_len > BUS_INLINE_MAX) || (env_len > remaining)) {
            _increment(rejected);
            break;
        }

        if (bus->wire_rx(frame.payload + in_pos, env_len)) {
            _increment(accepted);
        } else {
            _increment(rejected);
        }
        in_pos = (uint16_t)(in_pos + env_len);
    }

    HostFrame& response = _resp_frame;
    response.cmd = static_cast<uint8_t>(HostCmd::BUS_XFER);
    response.flags = HOST_CMD_FLAG_RESPONSE;
    response.seq = frame.seq;

    uint16_t out_pos = BUS_XFER_RESPONSE_HEADER;
    uint8_t frames_out = 0u;
    while (out_pos < HOST_CMD_PAYLOAD_MAX) {
        uint16_t env_len = 0u;
        const uint16_t capacity = (uint16_t)(HOST_CMD_PAYLOAD_MAX - out_pos);
        if (!bus->wire_tx_pop(response.payload + out_pos, capacity, env_len)) {
            break;
        }
        out_pos = (uint16_t)(out_pos + env_len);
        _increment(frames_out);
    }

    response.payload[0] = accepted;
    response.payload[1] = rejected;
    response.payload[2] = frames_out;
    response.payload[3] = bus->wire_tx_pending() ? BUS_XFER_FLAG_TX_PENDING : 0u;
    response.len = out_pos;
    *resp_len = HostCmdCodec::encode_frame(response, resp_buf, HOST_CMD_RESP_BUF_MAX);
}
