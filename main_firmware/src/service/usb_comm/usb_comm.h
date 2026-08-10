#pragma once

#include "protocol/host_cmd/host_cmd.h"

class HAL_USB_Device;

/**
 * UsbComm - USB 通信服务
 * 单例模式，负责:
 * - init(): 初始化编解码器
 * - update(): 从 CDC 读入字节喂给接收状态机，解析帧后分发，响应写回 CDC
 */
class UsbComm {
public:
    static UsbComm* getInstance();

    bool init();
    void update();
    bool has_pending_response() const;

private:
    UsbComm();
    UsbComm(const UsbComm&) = delete;
    UsbComm& operator=(const UsbComm&) = delete;

    static UsbComm* _instance;

    bool _pump_pending_tx(HAL_USB_Device* usb);
    void _clear_pending_tx(HAL_USB_Device* usb);
    void _dispatch_frame(HAL_USB_Device* usb, const HostFrame& frame);
    // dispatch 后 resp_len==0 的取证(= encode_* 拒绝组帧, 见 usb_debug.h 的 resp_encode_fail)。
    void _note_resp_encode_fail(const HostFrame& frame);
    HostCmdCodec _codec;
    uint8_t _resp_buf[HOST_CMD_RESP_BUF_MAX];  // 全量 CFG_GET_ALL(~140 项 ~2.1KB)整帧, 见 host_cmd.h
    uint16_t _pending_resp_len = 0;
    uint16_t _pending_resp_off = 0;
    HostFrame _frame;  // Reuse codec output across loop iterations to avoid stack bloat
    // ★单槽暂存 + 槽满即停止解析★(已实测稳定: 0 告警 / 25 分钟无复位)
    // 上一响应占用 _resp_buf 期间仍须把 RX 环里已完整的帧取走, 否则环会溢出丢字节。
    // 槽满后立刻 break 停止解析 —— 于是绝不会覆盖已暂存的帧, 剩余字节留在环里下一轮再取。
    // (曾改成 4 槽队列试图让 ALGO_GET_SRC 续片不被挤掉, 实测引发设备反复重枚举 + os error 22,
    //  已回退。那条路要走必须先弄清 dispatch 与 TinyUSB IN stream 的确切时序, 不能靠加槽位猜。)
    HostFrame _deferred_frame;
    bool _deferred_frame_pending = false;
    uint32_t _frame_open_since = 0;  // 当前半帧起始 millis(0=空闲), 供陈旧半帧超时复位
    
    struct RebootState {
        enum class Stage : uint8_t {
            IDLE,
            ACK_DRAIN,
            DETACHED,
        };

        Stage stage = Stage::IDLE;
        uint8_t mode = 0;
        uint32_t deadline_ms = 0;

        void clear() {
            stage = Stage::IDLE;
            mode = 0;
            deadline_ms = 0;
        }
    };

    RebootState _reboot;
};

// 编译宏：是否启用二进制帧模式（禁用时文本诊断保留，但不污染二进制流）
#define HOST_CMD_BINARY_MODE 1
