#pragma once
#include <cstdint>

// 调试计数器：经 vendor 控制请求(bRequest=0x50, EP0)读取。EP0 在 flash 写破坏 bulk 后仍存活，
// 可诊断"主循环是否存活/vendor OUT 是否 arm/rx 回调是否触发"等 bulk 死后的设备侧真相。
#pragma pack(push, 1)
struct UsbDebugCounters {
    uint16_t magic;              // 0xDB01
    uint16_t struct_len;         // sizeof(UsbDebugCounters)
    uint32_t loop_count;         // 主循环迭代次数(证明 loop 存活)
    uint32_t tud_task_count;     // tud_task 调用次数
    uint32_t vendor_rx_cb_count; // tud_vendor_rx_cb 触发次数(host->device OUT 完成)
    uint32_t vendor_rx_bytes;    // 已从 vendor 内部 fifo 搬出的总字节
    uint32_t vendor_tx_calls;    // config_write 调用次数
    uint32_t vendor_tx_bytes;    // config_write 写出的总字节
    uint32_t flash_write_count;  // flash 落地次数(config + csd)
    uint32_t loop_at_last_flash; // 最近一次 flash 落地时的 loop_count
    uint32_t rearm_count;        // vendor_service 实际重新武装 OUT 次数
    uint8_t  out_busy;           // 最近采样: OUT(0x01) usbd_edpt_busy
    uint8_t  out_stalled;        // 最近采样: OUT stalled
    uint8_t  mounted;            // tud_mounted
    uint8_t  debug_enabled;      // SET_DEBUG(0x51) 开关(默认0)
};
#pragma pack(pop)

extern volatile UsbDebugCounters g_usb_dbg;

// 经 EP0 控制请求(bRequest=0x52)置位的 BOOTSEL 请求：因 EP0 在 bulk vendor 死后仍存活，
// 可在 vendor 卡死时仍软件触发进烧录模式，免去物理 BOOTSEL。loop() 检测到后 reset_usb_boot。
extern volatile uint8_t g_bootsel_request;

// 最近一次收到主机(上位机)host_cmd 帧的 millis 时间戳。UsbComm 每次分发帧时更新。
// loop() 据此判定"主机已连接"(近 2s 内有帧)→ 绿灯常亮，否则心跳闪烁。
extern volatile uint32_t g_last_host_cmd_ms;
