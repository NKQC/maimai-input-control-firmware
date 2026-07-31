#include "hal_dma.h"
#include <hardware/dma.h>
#include <pico/stdlib.h>

HAL_DMA_Duplex::~HAL_DMA_Duplex() {
    deinit();
}

bool HAL_DMA_Duplex::init(const DmaDuplexPorts& ports) {
    if (_ready) return true;
    if (ports.tx_fifo == nullptr || ports.rx_fifo == nullptr) return false;

    const int tx = dma_claim_unused_channel(false);   // false: 无空闲通道时返回 -1 而非 panic
    if (tx < 0) return false;
    const int rx = dma_claim_unused_channel(false);
    if (rx < 0) {
        dma_channel_unclaim(tx);
        return false;
    }

    _ports = ports;
    _tx_ch = (int8_t)tx;
    _rx_ch = (int8_t)rx;
    _ready = true;
    return true;
}

void HAL_DMA_Duplex::deinit() {
    if (!_ready) return;
    dma_channel_abort(_tx_ch);
    dma_channel_abort(_rx_ch);
    dma_channel_unclaim(_tx_ch);
    dma_channel_unclaim(_rx_ch);
    _tx_ch = -1;
    _rx_ch = -1;
    _ready = false;
}

void HAL_DMA_Duplex::start(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (!_ready || len == 0) return;

    // RX 必须与 TX 同时武装：外设 RX FIFO 只有 4 级，若晚于 TX 启动会溢出丢字节。
    dma_channel_config rc = dma_channel_get_default_config(_rx_ch);
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_8);
    channel_config_set_read_increment(&rc, false);            // 源固定：外设 RX FIFO
    channel_config_set_write_increment(&rc, rx != nullptr);
    channel_config_set_dreq(&rc, _ports.rx_dreq);
    dma_channel_configure(_rx_ch, &rc,
                          (rx != nullptr) ? (void*)rx : (void*)&_sink,
                          (const void*)_ports.rx_fifo, (uint32_t)len, false);

    dma_channel_config tc = dma_channel_get_default_config(_tx_ch);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_8);
    channel_config_set_read_increment(&tc, tx != nullptr);
    channel_config_set_write_increment(&tc, false);           // 目标固定：外设 TX FIFO
    channel_config_set_dreq(&tc, _ports.tx_dreq);
    dma_channel_configure(_tx_ch, &tc,
                          (void*)_ports.tx_fifo,
                          (tx != nullptr) ? (const void*)tx : (const void*)&_zero,
                          (uint32_t)len, false);

    dma_start_channel_mask((1u << (uint32_t)_tx_ch) | (1u << (uint32_t)_rx_ch));
}

bool HAL_DMA_Duplex::busy() const {
    if (!_ready) return false;
    return dma_channel_is_busy(_tx_ch) || dma_channel_is_busy(_rx_ch);
}

bool HAL_DMA_Duplex::wait(uint32_t timeout_us) {
    if (!_ready) return false;
    const uint32_t t0 = time_us_32();
    while (busy()) {
        if ((time_us_32() - t0) > timeout_us) {
            // 超时：外设侧时钟/FIFO 异常。丢掉本次传输，让调用方复位外设，绝不带着半条传输继续。
            dma_channel_abort(_tx_ch);
            dma_channel_abort(_rx_ch);
            return false;
        }
        tight_loop_contents();
    }
    return true;
}
