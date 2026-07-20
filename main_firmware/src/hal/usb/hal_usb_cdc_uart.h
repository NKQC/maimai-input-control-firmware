#pragma once

#include "hal_usb.h"
#include "../uart/hal_uart.h"

/** HAL_UART adapter backed by one independent TinyUSB CDC role. */
class HAL_USB_CDC_UART final : public HAL_UART {
public:
    HAL_USB_CDC_UART(HAL_USB_Device* usb, UsbCdcPort port);

    bool init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate = 115200,
              bool flow_control = false, uint8_t cts_pin = 255, uint8_t rts_pin = 255) override;
    void deinit() override;
    size_t write_to_tx_buffer(const uint8_t* data, size_t length) override;
    size_t read_from_rx_buffer(uint8_t* buffer, size_t length) override;
    size_t get_tx_buffer_free_space() const override;
    size_t get_rx_buffer_data_count() const override;
    bool is_busy() const override;
    size_t available() override;
    void flush_rx() override;
    void flush_tx() override;
    bool set_baudrate(uint32_t baudrate) override;
    void set_rx_callback(std::function<void(uint8_t)> callback) override;
    std::string get_name() const override;
    bool is_ready() const override;

private:
    HAL_USB_Device* _usb;
    UsbCdcPort _port;
    bool _initialized;
    uint32_t _baudrate;
    std::function<void(uint8_t)> _rx_callback;
};
