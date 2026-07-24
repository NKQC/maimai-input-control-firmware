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

void UsbComm::update() {
#ifdef HOST_CMD_BINARY_MODE
    HAL_USB_Device* usb = HAL_USB_Device::getInstance();
    size_t available = usb->config_available();
    uint8_t read_buf[64];

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

    while (available > 0) {
        const size_t to_read = (available > sizeof(read_buf)) ? sizeof(read_buf) : available;
        const size_t read_len = usb->config_read(read_buf, to_read);
        if (read_len == 0) break;

        for (size_t index = 0; index < read_len; index++) {
            if (_codec.feed_byte(read_buf[index], &_frame)) {
                _frame_open_since = 0u;          // 帧完成, 清半帧计时
                g_last_host_cmd_ms = millis();   // 主机连接活跃指示(供 loop 绿灯常亮判定)
                // 续期制: 任意主机命令帧都续租全部定时任务(遥测等)。上位机停止发命令(丢失/关闭)
                // → 租约到期 → 大吞吐任务自动取消。上位机的 ~1s PING 天然维持续租。
                TxScheduler::getInstance()->renew_all(3000);
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
