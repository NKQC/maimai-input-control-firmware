#include "usb_comm.h"
#include "../../hal/usb/hal_usb.h"
#include "../../config.h"
#include "../usb_debug.h"
#include <Arduino.h>
#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>
#include <pico/bootrom.h>

namespace {
constexpr uint32_t ACK_DRAIN_MS = 200;
constexpr uint32_t USB_DETACH_MS = 400;
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

    return true;
}

void UsbComm::update() {
#ifdef HOST_CMD_BINARY_MODE
    HAL_USB_Device* usb = HAL_USB_Device::getInstance();
    size_t available = usb->config_available();
    uint8_t read_buf[64];

    while (available > 0) {
        const size_t to_read = (available > sizeof(read_buf)) ? sizeof(read_buf) : available;
        const size_t read_len = usb->config_read(read_buf, to_read);
        if (read_len == 0) break;

        for (size_t index = 0; index < read_len; index++) {
            if (_codec.feed_byte(read_buf[index], &_frame)) {
                g_last_host_cmd_ms = millis();   // 主机连接活跃指示(供 loop 绿灯常亮判定)
                uint16_t resp_len = 0;
                HostCmdDispatcher::getInstance()->dispatch(_frame, _resp_buf, &resp_len);
                if (resp_len > 0) {
                    usb->config_write(_resp_buf, resp_len);
                }
            }
        }
        available = usb->config_available();
    }

    const uint32_t now = millis();
    switch (_reboot.stage) {
        case RebootState::Stage::IDLE:
            break;

        case RebootState::Stage::ACK_DRAIN:
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
                watchdog_reboot(0, 0, 10);
                while (true) tight_loop_contents();
            }
            reset_usb_boot(0, 0);
            break;
    }
#endif
}
