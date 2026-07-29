#include "self_heal.h"

#include <Arduino.h>   // millis()

#include "../../hal/usb/hal_usb.h"
#include "../../service/usb_debug.h"   // g_last_host_cmd_ms(主机在线判定)
#include "../tx_scheduler/tx_scheduler.h"

// 事件推送: 10Hz 足够(事件是稀疏的)。租约必须够长: 自持恢复往往正好发生在上位机没连的时候
// (比如插上电、算法把 PSoC 搞死), 事件要能等到上位机连上来再送出去, 否则"UI 知情"就是空话。
static constexpr uint32_t SELFHEAL_INTERVAL_US = 100000;
static constexpr uint32_t SELFHEAL_LEASE_MS = 600000;   // 10 分钟: 覆盖"先出事、后开上位机"
// 主机在线判定窗: 上位机每 ~1s 会发命令刷新 g_last_host_cmd_ms。
static constexpr uint32_t HOST_ONLINE_WINDOW_MS = 3000;

SelfHeal* SelfHeal::_instance = nullptr;

SelfHeal* SelfHeal::getInstance() {
    if (_instance == nullptr) _instance = new SelfHeal();
    return _instance;
}

void SelfHeal::note(uint8_t code, uint32_t detail) {
    if (code == SH_NONE) return;
    if (_count >= QUEUE_SIZE) {
        // 队列满: 丢最旧的一条(保证最新事件必然可达), 计数供上位机知道"有事件没看到"。
        _head = static_cast<uint8_t>((_head + 1u) % QUEUE_SIZE);
        _count--;
        _dropped++;
    }
    const uint8_t tail = static_cast<uint8_t>((_head + _count) % QUEUE_SIZE);
    _queue[tail].code = code;
    _queue[tail].detail = detail;
    _queue[tail].seq = ++_seq;
    _queue[tail].sends = 0u;
    _queue[tail].last_send_ms = 0u;
    _count++;
    _total++;
    // 有事件即拉起推送任务(已存在则续期, 不打断节奏)。
    TxScheduler::getInstance()->schedule(TX_TASK_SELFHEAL, SELFHEAL_INTERVAL_US,
                                        SELFHEAL_LEASE_MS, &SelfHeal::emit_task);
}

void SelfHeal::note_rearm() {
    if (_count == 0u) return;
    TxScheduler::getInstance()->schedule(TX_TASK_SELFHEAL, SELFHEAL_INTERVAL_US,
                                        SELFHEAL_LEASE_MS, &SelfHeal::emit_task);
}

bool SelfHeal::pop(uint8_t* code, uint32_t* detail, uint16_t* seq) {
    if (_count == 0u) return false;
    const Entry& e = _queue[_head];
    if (code != nullptr)   *code = e.code;
    if (detail != nullptr) *detail = e.detail;
    if (seq != nullptr)    *seq = e.seq;
    _head = static_cast<uint8_t>((_head + 1u) % QUEUE_SIZE);
    _count--;
    return true;
}

void SelfHeal::emit_task() {
    getInstance()->_tick();
}

void SelfHeal::_tick() {
    if (_count == 0u) {
        TxScheduler::getInstance()->cancel(TX_TASK_SELFHEAL);
        return;
    }
    // ★主机不在线就不发★: STREAM 帧是发射后不管的, 没人听时发出去等于把事件丢进空气,
    // 而 pop() 会把它从队列里消掉 —— 事件就永久丢失了。故先确认主机在线再发。
    // 初值 0 = 开机至今从未收到过任何主机命令。不能只看时间差: 开机头 3 秒内 (millis()-0) 仍小于
    // 窗口, 会把"没人连"误判成"在线", 于是启动阶段的事件(首次 provisioning 等)被发进空气再被 pop 掉。
    const uint32_t host_seen_ms = g_last_host_cmd_ms;
    if (host_seen_ms == 0u) return;
    if ((millis() - host_seen_ms) >= HOST_ONLINE_WINDOW_MS) return;
    HAL_USB_Device* usb = HAL_USB_Device::getInstance();
    if (usb->config_write_available() == 0u) return;   // 背压: 不弹队列, 下轮重发

    // 只看不弹: 组帧成功且真正写出去了才计一次发送, 发满 SEND_TIMES 次才出队。
    // HostFrame 含 4KB payload, 绝不能放栈上(RP2040 栈 8KB) → 用类成员帧缓冲。
    Entry& e = _queue[_head];
    const uint32_t now_ms = millis();
    if (e.sends > 0u && (now_ms - e.last_send_ms) < RESEND_GAP_MS) return;   // 重发间隔未到
    HostFrame& frame = _frame;
    uint16_t length = 0u;
    frame.payload[length++] = e.code;
    frame.payload[length++] = static_cast<uint8_t>(e.detail);
    frame.payload[length++] = static_cast<uint8_t>(e.detail >> 8);
    frame.payload[length++] = static_cast<uint8_t>(e.detail >> 16);
    frame.payload[length++] = static_cast<uint8_t>(e.detail >> 24);
    frame.payload[length++] = static_cast<uint8_t>(e.seq);
    frame.payload[length++] = static_cast<uint8_t>(e.seq >> 8);
    frame.payload[length++] = static_cast<uint8_t>(_total);
    frame.payload[length++] = static_cast<uint8_t>(_total >> 8);
    frame.cmd = static_cast<uint8_t>(HostCmd::SELF_HEAL_EVENT);
    frame.flags = HOST_CMD_FLAG_STREAM;
    frame.seq = static_cast<uint8_t>(e.seq);
    frame.len = length;

    const uint16_t frame_length = HostCmdCodec::encode_frame(frame, _tx_buf, sizeof(_tx_buf));
    if (frame_length == 0u) {
        pop(nullptr, nullptr, nullptr);   // 编码不出来的坏事件直接丢, 不卡住队列
        return;
    }
    usb->config_write(_tx_buf, frame_length);
    e.sends++;
    e.last_send_ms = now_ms;
    if (e.sends >= SEND_TIMES) {
        pop(nullptr, nullptr, nullptr);
    }
}
