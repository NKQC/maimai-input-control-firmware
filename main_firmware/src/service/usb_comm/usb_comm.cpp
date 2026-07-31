#include "usb_comm.h"
#include "../../hal/usb/hal_usb.h"
#include "../../config.h"
#include "../tx_scheduler/tx_scheduler.h"
#include "../usb_debug.h"
#include <Arduino.h>
#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>
#include <pico/bootrom.h>

namespace {
constexpr uint32_t ACK_DRAIN_MS = 200;
constexpr uint32_t USB_DETACH_MS = 400;
constexpr size_t RX_BYTES_PER_UPDATE = 256;
constexpr uint8_t TX_PUMP_ATTEMPTS = 24;
}

UsbComm* UsbComm::_instance = nullptr;

UsbComm::UsbComm() = default;

UsbComm* UsbComm::getInstance() {
    if (!_instance) {
        _instance = new UsbComm();
    }
    return _instance;
}

bool UsbComm::init() {
    _codec.init();

    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::REBOOT,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
            UsbComm* comm = UsbComm::getInstance();
            if (comm->_reboot.stage == RebootState::Stage::IDLE) {
                comm->_reboot.stage = RebootState::Stage::ACK_DRAIN;
                comm->_reboot.mode = 0;
                comm->_reboot.deadline_ms = 0;
            }
        });

    dispatcher->register_handler(HostCmd::REBOOT_BOOTLOADER,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
            UsbComm* comm = UsbComm::getInstance();
            if (comm->_reboot.stage == RebootState::Stage::IDLE) {
                comm->_reboot.stage = RebootState::Stage::ACK_DRAIN;
                comm->_reboot.mode = 1;
                comm->_reboot.deadline_ms = 0;
            }
        });

    // REBOOT_PSOC(0x06): 置位请求, 由 loop() 脉冲 XRES 复位 PSoC(使需重启生效的改动生效)。
    dispatcher->register_handler(HostCmd::REBOOT_PSOC,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            g_psoc_reboot_request = 1u;
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
        });

    // DEBUG_CRASH_BOOTSEL(0x07): 运行时武装/解除"运行中崩溃→进 BOOTSEL"。payload[0]:1=武装 0=解除。
    // 武装标志写 watchdog scratch[6](跨复位存活、掉电清零), 由 setup() 启动决策读取。默认解除:
    // 崩溃只正常重启; 自持 debug 连上后武装, 崩溃即进烧录便于 dev.ps1 自动重烧。
    dispatcher->register_handler(HostCmd::DEBUG_CRASH_BOOTSEL,
        [](const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len) {
            const bool arm = (frame.len >= 1) && (frame.payload[0] != 0u);
            watchdog_hw->scratch[6] = arm ? DEBUG_BOOTSEL_MAGIC : 0u;
            *resp_len = HostCmdCodec::encode_ack(frame.seq, resp_buf, 512);
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

bool UsbComm::_pump_pending_tx(HAL_USB_Device* usb) {
    if (_pending_resp_off >= _pending_resp_len) return true;

    const uint16_t remaining = _pending_resp_len - _pending_resp_off;
    size_t sent = 0;
    for (uint8_t attempt = 0; attempt < TX_PUMP_ATTEMPTS && sent < remaining; ++attempt) {
        const size_t written = usb->config_write_some(
            _resp_buf + _pending_resp_off + sent, remaining - sent);
        if (written > remaining - sent) {
            HostCmdDispatcher::getInstance()->clear_pending_stream();
            _clear_pending_tx(usb);
            return true;
        }
        if (written != 0) {
            sent += written;
            continue;
        }

        usb->task();
        sleep_us(50);
    }

    if (sent == 0) return false;

    _pending_resp_off += static_cast<uint16_t>(sent);
    if (_pending_resp_off < _pending_resp_len) return false;

    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    if (dispatcher->has_pending_stream()) {
        uint16_t next_len = 0;
        if (dispatcher->next_stream_frame(_resp_buf, &next_len) && next_len > 0) {
            _pending_resp_len = next_len;
            _pending_resp_off = 0;
            return false;
        }
        dispatcher->clear_pending_stream();
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
    if (!_pump_pending_tx(usb)) return;

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
            uint16_t resp_len = 0;
            HostCmdDispatcher::getInstance()->dispatch(_frame, _resp_buf, &resp_len);
            if (resp_len > 0) {
                _pending_resp_len = resp_len;
                _pending_resp_off = 0;
                usb->begin_command_response();
                _pump_pending_tx(usb);
                return;
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

            if (_reboot.mode == 0) {
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
            reset_usb_boot(0, 0);
            break;
    }
#endif
}
