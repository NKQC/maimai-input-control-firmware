#pragma once

#include <cstdint>

struct HostFrame;

class BusUsbLink {
public:
    static BusUsbLink* getInstance();

    bool init();

private:
    BusUsbLink();
    BusUsbLink(const BusUsbLink&) = delete;
    BusUsbLink& operator=(const BusUsbLink&) = delete;

    static void _handle_xfer(const HostFrame& frame, uint8_t* resp_buf, uint16_t* resp_len);
    static void _increment(uint8_t& value);
    // 自测钩子(仅 payload 恰好 1 字节时触发, 见 bus_usb_link.cpp 注释)。
    // 返回写入 body 的字节数; 未知请求码返回 0。
    static uint16_t _probe(uint8_t code, uint8_t* body);

    bool _initialized;

    static BusUsbLink* _instance;
};
