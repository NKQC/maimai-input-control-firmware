#include "usb_comm.h"
#include "../../hal/usb/hal_usb.h"
#include "../../config.h"
#include "../tx_scheduler/tx_scheduler.h"
#include "../persistence_txn/persistence_txn.h"
#include "../sensor_link/sensor_link.h"
#include "../usb_debug.h"
#include <Arduino.h>
#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>
#include <pico/bootrom.h>

namespace {
constexpr uint32_t ACK_DRAIN_MS = 200;
constexpr uint32_t USB_DETACH_MS = 400;
constexpr size_t RX_BYTES_PER_UPDATE = 1024;  // PSoC 段最坏阻塞 156ms；256B/轮跟不上主机命令速率，会灌满 RX 环丢帧。
}

UsbComm* UsbComm::_instance = nullptr;

UsbComm::UsbComm() = default;

UsbComm* UsbComm::getInstance() {
    if (_instance == nullptr) {
        static UsbComm instance;
        _instance = &instance;
    }
    return _instance;
}

bool UsbComm::init() {
    _codec.init();

    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::REBOOT,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
            UsbComm* comm = UsbComm::getInstance();
            if (comm->_reboot.stage == RebootState::Stage::IDLE) {
                comm->_reboot.stage = RebootState::Stage::ACK_DRAIN;
                comm->_reboot.mode = UsbComm::RebootState::Mode::APP;
                comm->_reboot.deadline_ms = 0;
            }
        });

    dispatcher->register_handler(HostCmd::REBOOT_BOOTLOADER,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
            UsbComm* comm = UsbComm::getInstance();
            if (comm->_reboot.stage == RebootState::Stage::IDLE) {
                comm->_reboot.stage = RebootState::Stage::ACK_DRAIN;
                comm->_reboot.mode = UsbComm::RebootState::Mode::BOOTLOADER;
                comm->_reboot.deadline_ms = 0;
            }
        });

    // REBOOT_PSOC(0x06): 置位请求, 由 loop() 脉冲 XRES 复位 PSoC(使需重启生效的改动生效)。
    dispatcher->register_handler(HostCmd::REBOOT_PSOC,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            g_psoc_reboot_request = 1u;
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
        });

    // DEBUG_CRASH_BOOTSEL(0x07): 运行时武装/解除"运行中崩溃→进 BOOTSEL"。payload[0]:1=武装 0=解除。
    // 武装标志写 watchdog scratch[6](跨复位存活、掉电清零), 由 setup() 启动决策读取。
    dispatcher->register_handler(HostCmd::DEBUG_CRASH_BOOTSEL,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            if (frame.len != 1u || (frame.payload[0] != 0u && frame.payload[0] != 1u)) {
                *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                                                     "arm payload must be 0 or 1", resp_buf,
                                                     HOST_CMD_RESP_BUF_MAX);
                return;
            }
            watchdog_hw->scratch[6] = frame.payload[0] != 0u ? DEBUG_BOOTSEL_MAGIC : 0u;
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
        });

    // DEBUG_TRIGGER_CRASH(0x0A): 仅允许已武装、设备空闲、空 payload。ACK 完成后复用主动重启
    // 的 detach/drain 状态机，保留 scratch[6]/运行标记，让 setup() 进入既有崩溃 BOOTSEL 路径。
    dispatcher->register_handler(HostCmd::DEBUG_TRIGGER_CRASH,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            UsbComm* comm = UsbComm::getInstance();
            if (frame.len != 0u) {
                *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                                                     "trigger payload must be empty", resp_buf,
                                                     HOST_CMD_RESP_BUF_MAX);
                return;
            }
            if (watchdog_hw->scratch[6] != DEBUG_BOOTSEL_MAGIC) {
                *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                                                     "crash BOOTSEL is not armed", resp_buf,
                                                     HOST_CMD_RESP_BUF_MAX);
                return;
            }
            if (comm->_reboot.stage != RebootState::Stage::IDLE) {
                *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::DEVICE_BUSY,
                                                     "reboot already in progress", resp_buf,
                                                     HOST_CMD_RESP_BUF_MAX);
                return;
            }
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, HOST_CMD_RESP_BUF_MAX);
            comm->_reboot.stage = RebootState::Stage::ACK_DRAIN;
            comm->_reboot.mode = UsbComm::RebootState::Mode::DEBUG_TRIGGER;
            comm->_reboot.deadline_ms = 0;
        });

    return true;
}

void UsbComm::_clear_pending_tx(HAL_USB_Device* usb) {
    _pending_resp_len = 0;
    _pending_resp_off = 0;
    usb->end_command_response();
}

bool UsbComm::has_pending_response() const {
    return _pending_resp_off < _pending_resp_len ||
        HostCmdDispatcher::getInstance()->has_pending_stream();
}

void UsbComm::_pump_persistence_terminal(HAL_USB_Device* usb) {
    if (has_pending_response()) return;
    uint8_t seq = 0u;
    PersistenceTxn::Result result = PersistenceTxn::Result::NONE;
    if (!PersistenceTxn::getInstance()->take_terminal(&seq, &result)) return;
    uint16_t response_len = 0u;
    if (result == PersistenceTxn::Result::OK) {
        response_len = HostCmdCodec::encode_ack(seq, _resp_buf, HOST_CMD_RESP_BUF_MAX);
    } else {
        const char* detail = result == PersistenceTxn::Result::BUSY
            ? "save transaction busy" : "persistent commit failed";
        response_len = HostCmdCodec::encode_nak(seq, HostCmdError::CONFIG_ERROR,
                                                detail, _resp_buf, HOST_CMD_RESP_BUF_MAX);
    }
    if (response_len == 0u) return;
    _pending_resp_len = response_len;
    _pending_resp_off = 0u;
    usb->begin_command_response();
}

void UsbComm::_pump_sensor_terminal(HAL_USB_Device* usb) {
    if (has_pending_response()) return;
    uint8_t cmd = 0u;
    uint8_t seq = 0u;
    bool ok = false;
    if (!SensorLink::getInstance()->take_host_write_terminal(&cmd, &seq, &ok)) return;
    uint16_t response_len = ok
        ? HostCmdCodec::encode_ack(seq, _resp_buf, HOST_CMD_RESP_BUF_MAX)
        : HostCmdCodec::encode_nak(seq, HostCmdError::SENSOR_ERROR, "PSoC command failed",
                                   _resp_buf, HOST_CMD_RESP_BUF_MAX);
    if (response_len == 0u) return;
    _pending_resp_len = response_len;
    _pending_resp_off = 0u;
    usb->begin_command_response();
}

void UsbComm::_dispatch_frame(HAL_USB_Device* usb, const HostFrame& frame) {
    uint16_t resp_len = 0;
    g_usb_dbg.host_dispatch_count++;
    if (frame.cmd == static_cast<uint8_t>(HostCmd::ALGO_GET_INFO)) {
        g_usb_dbg.host_algo_info_dispatch_count++;
    }
    g_usb_dbg.host_last_dispatch_cmd = frame.cmd;
    g_usb_dbg.host_last_dispatch_seq = frame.seq;
    if (SensorLink::getInstance()->replay_host_write(frame, _resp_buf, &resp_len)) {
        g_usb_dbg.host_last_dispatch_resp_len = resp_len;
        if (resp_len > 0u) {
            _pending_resp_len = resp_len;
            _pending_resp_off = 0u;
            usb->begin_command_response();
        }
        return;
    }
    HostCmdDispatcher::getInstance()->dispatch(frame, _resp_buf, &resp_len);
    g_usb_dbg.host_last_dispatch_resp_len = resp_len;
    if (resp_len > 0) {
        _pending_resp_len = resp_len;
        _pending_resp_off = 0;
        usb->begin_command_response();
        return;
    }
    // SAVE_CONFIG deliberately has no immediate response: its terminal ACK/NAK
    // is generated only after the verified persistence barrier completes.
    if (frame.cmd == static_cast<uint8_t>(HostCmd::SAVE_CONFIG) &&
        (PersistenceTxn::getInstance()->awaiting_response(frame.seq) ||
         PersistenceTxn::getInstance()->completed(frame.seq))) {
        return;
    }
    // Delayed terminal commands intentionally return no immediate payload; UsbComm owns their final ACK/NAK.
    if (SensorLink::getInstance()->host_write_active()) {
        return;
    }
    // ★resp_len==0 只有一个语义: 组帧被拒★ 每个已注册 handler 都必然写 resp_len, 未注册命令走
    // _handle_not_implemented 回 NAK, 而遥测/自持恢复等推送根本不经 dispatch(各自 _tx_buf + config_write)。
    // 故这里不是"本命令无需响应", 而是 HostCmdCodec::encode_* 因 max_len 装不下返回了 0 —— 主机会
    // 永远等不到该命令的响应(回读页面空白), 设备侧此前完全静默。留证以便直接指认是哪条命令。
    _note_resp_encode_fail(frame);
}

void UsbComm::_note_resp_encode_fail(const HostFrame& frame) {
    // 封顶不回绕: 首个现场比"最新现场"更有价值, 计数溢出回 0 会把已发生过失败这件事抹掉。
    if (g_usb_dbg.resp_encode_fail < 0xFFFFFFFFu) g_usb_dbg.resp_encode_fail++;
    g_usb_dbg.resp_fail_cmd = frame.cmd;
    g_usb_dbg.resp_fail_req_len = frame.len;
}

bool UsbComm::_pump_pending_tx(HAL_USB_Device* usb) {
    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    // 只有上一帧已经完全写出后，才在下一轮主循环生成下一片。不能在同一轮
    // tud_vendor_write_flush() 后立即重写 _resp_buf：TinyUSB 的 64B TX stream
    // 可能仍引用上一片，CFG_GET_ALL 的下一片会覆盖它并令 WinUSB IN stall。
    if (_pending_resp_off >= _pending_resp_len) {
        if (dispatcher->has_pending_stream()) {
            uint16_t next_len = 0;
            if (dispatcher->next_stream_frame(_resp_buf, &next_len) && next_len > 0) {
                _pending_resp_len = next_len;
                _pending_resp_off = 0;
                usb->begin_command_response();
            } else {
                dispatcher->clear_pending_stream();
                _clear_pending_tx(usb);
                return true;
            }
        } else {
            return true;
        }
    }

    const uint16_t remaining = _pending_resp_len - _pending_resp_off;
    // 按当前 FIFO 可用空间批量递交；config_write_some 不递归 tud_task()，下一轮再续传。
    // FIFO 扩大后 2052B 响应通常只需 1~2 轮主循环即可发完。
    const size_t packet = std::min<size_t>(remaining, usb->config_response_write_available());
    const size_t written = usb->config_write_some(_resp_buf + _pending_resp_off, packet);
    if (written == 0u) return false;
    _pending_resp_off += static_cast<uint16_t>(written);
    if (_pending_resp_off < _pending_resp_len) return false;

    if (dispatcher->has_pending_stream()) {
        // 保持 response-active，下一轮才生成/写入下一片。
        return false;
    }
    _clear_pending_tx(usb);
    return true;
}

void UsbComm::update() {
#ifdef HOST_CMD_BINARY_MODE
    HAL_USB_Device* usb = HAL_USB_Device::getInstance();
    if (!usb->is_ready()) {
        _clear_pending_tx(usb);
        HostCmdDispatcher::getInstance()->clear_pending_stream();
        _codec.reset();
        _frame_open_since = 0u;
        if (_reboot.stage != RebootState::Stage::DETACHED) {
            _reboot.clear();
            return;
        }
    }
    // Complete a deferred SAVE_CONFIG terminal before accepting another frame.
    // Normal commands are explicitly flow-controlled by leaving unread bytes in
    // the USB FIFO while a response is in flight; this keeps RAM bounded without
    // cloning 4 KiB HostFrames and never silently overwrites a request.
    _pump_persistence_terminal(usb);
    _pump_sensor_terminal(usb);

    // 再泵 TX：可能是刚上面派发出的响应，也可能是续传的大响应分片。
    _pump_pending_tx(usb);
    // A response buffer is single-owner. Stop before decoding another complete
    // request so RX remains the bounded backpressure queue supplied by TinyUSB.
    if (has_pending_response()) return;
    // SAVE_CONFIG owns the transport until its verified terminal response has
    // been queued. This is intentional endpoint backpressure, not a drop: the
    // host keeps later bytes in its FIFO and retries the same sequence safely.
    if (PersistenceTxn::getInstance()->active() || SensorLink::getInstance()->host_write_active()) return;

    const size_t available = usb->config_available();
    uint8_t read_byte;

    // ★陈旧半帧超时复位★：上一会话截断/损坏的帧可能声明了大 len, 卡在 READ_PAYLOAD,
    // 把重连后新到的 HELLO 字节当作其 payload 吞掉 → 永远解析不出 HELLO → 无 DEVICE_INFO 响应
    // (实测: 设备 rx 计数增长但 tx 计数不变)。正常帧即便满载也在数十 ms 内收完; 半帧开启超过
    // 250ms 判定陈旧, 复位解析器释放通道。
    if (available > 0 && !_codec.is_idle()) {
        const uint32_t now_ms = millis();
        if (_frame_open_since == 0u) {
            _frame_open_since = now_ms;
        } else if ((now_ms - _frame_open_since) > 250u) {
            _codec.reset();
            _frame_open_since = 0u;
        }
    }

    for (size_t rx_count = 0;
         rx_count < RX_BYTES_PER_UPDATE && usb->config_available() > 0;
         ++rx_count) {
        if (usb->config_read(&read_byte, 1) == 0) break;

        if (_codec.feed_byte(read_byte, &_frame)) {
            _frame_open_since = 0u;          // 帧完成, 清半帧计时
            g_last_host_cmd_ms = millis();   // 主机连接活跃指示(供 loop 绿灯常亮判定)
            // 续期制: 任意主机命令帧都续租全部定时任务(遥测等)。上位机停止发命令(丢失/关闭)
            // → 租约到期 → 大吞吐任务自动取消。上位机的 ~1s PING 天然维持续租。
            TxScheduler::getInstance()->renew_all(3000);

            if (!has_pending_response()) {
                _dispatch_frame(usb, _frame);
                // 响应缓冲为单所有者：当前帧一旦生成响应，必须立刻停止读取 RX。
                // 若继续解码到下一完整帧才检查 has_pending_response，该帧的字节已经从
                // TinyUSB FIFO 被取走，却既不能分发也没有软件队列承接，结果就是静默丢命令。
                // 主机允许把直接查询与有序写队列帧连续提交，因此这里必须在帧边界施加背压。
                if (has_pending_response() || PersistenceTxn::getInstance()->active() ||
                    SensorLink::getInstance()->host_write_active()) break;
            } else {
                // 正常入口不会在已有响应时读取 RX；保留防御分支，避免未来调用顺序变化时吞帧。
                break;
            }
        }
    }

    const uint32_t now = millis();
    switch (_reboot.stage) {
        case RebootState::Stage::IDLE:
            break;

        case RebootState::Stage::ACK_DRAIN:
            if (_pending_resp_off < _pending_resp_len) break;
            // Keep TinyUSB alive until the ACK IN transaction has had a stable completion window.
            usb->config_flush();
            usb->task();
            if (_reboot.deadline_ms == 0) {
                _reboot.deadline_ms = now + ACK_DRAIN_MS;
                break;
            }
            if (static_cast<int32_t>(now - _reboot.deadline_ms) >= 0) {
                usb->soft_disconnect();
                _reboot.stage = RebootState::Stage::DETACHED;
                _reboot.deadline_ms = millis() + USB_DETACH_MS;
            }
            break;

        case RebootState::Stage::DETACHED:
            // D+ remains physically detached while the hub debounces/removes the child device.
            usb->task();
            if (static_cast<int32_t>(now - _reboot.deadline_ms) < 0) break;

            if (_reboot.mode == RebootState::Mode::APP) {
                // 主动重启回 app：清运行态标记，避免启动时被判为死锁而进 BOOTSEL。
                watchdog_hw->scratch[7] = 0u;
                // ★同时清 scratch[0]/[1]★: setup() 用 scratch[0]==CRASH_RUN_MAGIC 判"上次是否运行中
                // 崩溃", 而 watchdog_reboot 会保留它 ⇒ 主机主动 REBOOT 也被报成 last_boot_was_wd=1
                // 且带一个陈旧 stage(实测 nv-soak 重启后恒报 USB_UPDATE 崩溃)。这会把"设备是否真卡住"
                // 的判据污染成永远为真, 故主动重启必须一并清掉。
                watchdog_hw->scratch[CRASH_SCRATCH_RUN] = 0u;
                watchdog_hw->scratch[CRASH_SCRATCH_STAGE] = 0u;
                watchdog_reboot(0, 0, 10);
                while (true) tight_loop_contents();
            }
            if (_reboot.mode == RebootState::Mode::DEBUG_TRIGGER) {
                // 不清 scratch[6] 或 scratch[7]/运行标记: watchdog_reboot 后 setup() 的既有判据
                // 会把这次安全触发当作运行中崩溃并转入 BOOTSEL。
                watchdog_reboot(0, 0, 10);
                while (true) tight_loop_contents();
            }
            reset_usb_boot(0, 0);
            break;
    }
#endif
}
