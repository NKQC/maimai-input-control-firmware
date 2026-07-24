#pragma once

#include "protocol/host_cmd/host_cmd.h"

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

private:
    UsbComm();
    UsbComm(const UsbComm&) = delete;
    UsbComm& operator=(const UsbComm&) = delete;

    static UsbComm* _instance;
    
    HostCmdCodec _codec;
    uint8_t _resp_buf[HOST_CMD_RESP_BUF_MAX];  // 全量 CFG_GET_ALL(~140 项 ~2.1KB)整帧, 见 host_cmd.h
    HostFrame _frame;  // Reuse frame buffer across loop iterations to avoid stack bloat
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
