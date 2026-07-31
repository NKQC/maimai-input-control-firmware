#include "hal_usb_cdc_uart.h"

HAL_USB_CDC_UART::HAL_USB_CDC_UART(HAL_USB_Device* usb, UsbCdcPort port)
    : _usb(usb), _port(port), _initialized(false), _baudrate(115200) {}

bool HAL_USB_CDC_UART::init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate,
                            bool flow_control, uint8_t cts_pin, uint8_t rts_pin) {
    (void)tx_pin;
    (void)rx_pin;
    (void)flow_control;
    (void)cts_pin;
    (void)rts_pin;
    if (_usb == nullptr || baudrate == 0) return false;
    _baudrate = baudrate;
    _initialized = true;
    flush_rx();
    return true;
}

void HAL_USB_CDC_UART::deinit() {
    if (!_initialized) return;
    flush_rx();
    flush_tx();
    _initialized = false;
    _rx_callback = nullptr;
}

size_t HAL_USB_CDC_UART::write_to_tx_buffer(const uint8_t* data, size_t length) {
    if (!_initialized || data == nullptr || length == 0) return 0;
    return _usb->cdc_write(_port, data, length) ? length : 0;
}

size_t HAL_USB_CDC_UART::read_from_rx_buffer(uint8_t* buffer, size_t length) {
    if (!_initialized || buffer == nullptr || length == 0) return 0;
    const size_t read = _usb->cdc_read(_port, buffer, length);
    if (_rx_callback) {
        for (size_t index = 0; index < read; index++) _rx_callback(buffer[index]);
    }
    return read;
}

size_t HAL_USB_CDC_UART::get_tx_buffer_free_space() const {
    // ★必须是真值★: 原先恒返回 64, 于是 Mai2Light 的"写不下就丢, 不阻塞"guard 永远不触发,
    // 每条应答都进到写路径里去等空间。调用方拿这个数决定丢不丢帧, 谎报等于取消了它的背压。
    return (_initialized && _usb != nullptr) ? _usb->cdc_write_available(_port) : 0;
}

size_t HAL_USB_CDC_UART::get_rx_buffer_data_count() const {
    return _initialized ? _usb->cdc_available(_port) : 0;
}

bool HAL_USB_CDC_UART::is_busy() const {
    return false;
}

size_t HAL_USB_CDC_UART::available() {
    return get_rx_buffer_data_count();
}

void HAL_USB_CDC_UART::flush_rx() {
    if (!_usb) return;
    uint8_t discard[64];
    while (_usb->cdc_available(_port) > 0) {
        if (_usb->cdc_read(_port, discard, sizeof(discard)) == 0) break;
    }
}

void HAL_USB_CDC_UART::flush_tx() {
    if (_usb) _usb->cdc_flush(_port);
}

bool HAL_USB_CDC_UART::set_baudrate(uint32_t baudrate) {
    if (baudrate == 0) return false;
    _baudrate = baudrate;
    return true;
}

void HAL_USB_CDC_UART::set_rx_callback(std::function<void(uint8_t)> callback) {
    _rx_callback = callback;
}

std::string HAL_USB_CDC_UART::get_name() const {
    return (_port == UsbCdcPort::CDC_SERIAL) ? "USB CDC serial" : "USB CDC light";
}

bool HAL_USB_CDC_UART::is_ready() const {
    return _initialized && _usb != nullptr;
}
