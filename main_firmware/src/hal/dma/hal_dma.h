#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * HAL_DMA_Duplex - 外设 FIFO ↔ 内存 的 8 位全双工 DMA 通道对
 *
 * 用途：让 PIO(或其它带 DREQ 的外设)把数据直接搬进/搬出内存，收发只做内存操作。
 * 调用方全程不触碰 FIFO：配置地址与计数 → start() → wait() 轮询完成标志，
 * 不再有逐字节阻塞，也不需要用固定 sleep 估算传输时间。
 *
 * 两条通道各自由 DREQ 节流：TX(mem→外设 TX FIFO)、RX(外设 RX FIFO→mem)。
 * 字节道地址由外设侧提供(RP2040 窄写会把字节复制到全部 4 个字节道，读则取指定道)。
 * 命名规范：类内部成员/函数以 _ 开头，对外接口不加 _。
 */

// 一对通道的静态端口描述（地址与 DREQ 在 init 时固定，之后每次传输只换内存侧地址与计数）
struct DmaDuplexPorts {
    volatile void* tx_fifo = nullptr;   // 目标：外设 TX FIFO 的字节道地址
    volatile void* rx_fifo = nullptr;   // 源：  外设 RX FIFO 的字节道地址
    uint8_t tx_dreq = 0;                // 外设 TX 的 DREQ 编号
    uint8_t rx_dreq = 0;                // 外设 RX 的 DREQ 编号
};

class HAL_DMA_Duplex {
public:
    HAL_DMA_Duplex() = default;
    ~HAL_DMA_Duplex();

    // 申请 2 条 DMA 通道并记下端口；失败(无空闲通道/端口非法)返回 false
    bool init(const DmaDuplexPorts& ports);
    void deinit();
    bool ready() const { return _ready; }

    // 启动一次 len 字节的 8 位全双工传输（立即返回，不等完成）。
    // tx == nullptr：发固定 0x00；rx == nullptr：收到的字节丢进内部黑洞(仍必须搬走，否则外设 RX FIFO 会堵)。
    void start(const uint8_t* tx, uint8_t* rx, size_t len);

    // 传输是否仍在进行（收/发任一通道忙）
    bool busy() const;

    // 轮询完成标志直到两条通道都空闲；超时则 abort 两条通道并返回 false
    bool wait(uint32_t timeout_us);

private:
    DmaDuplexPorts _ports;
    int8_t _tx_ch = -1;
    int8_t _rx_ch = -1;
    bool _ready = false;
    uint8_t _zero = 0;   // tx == nullptr 时的固定源（read_increment=false）
    uint8_t _sink = 0;   // rx == nullptr 时的固定目标（write_increment=false）
};
