#include "hal_uart.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/dma.h>
#include <pico/stdlib.h>

// HAL_UART0 静态成员初始化
HAL_UART0* HAL_UART0::instance_ = nullptr;

// HAL_UART0 实现
HAL_UART0* HAL_UART0::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_UART0();
    }
    return instance_;
}

HAL_UART0::HAL_UART0() 
    : initialized_(false), tx_pin_(0), rx_pin_(0), baudrate_(115200),
      dma_busy_(false), dma_tx_channel_(-1), dma_rx_channel_(-1),
      rx_head_(0), rx_tail_(0), tx_head_(0), tx_tail_(0), tx_dma_active_(false) {
}

HAL_UART0::~HAL_UART0() {
    deinit();
    instance_ = nullptr;
}

bool HAL_UART0::init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate, bool flow_control, uint8_t cts_pin, uint8_t rts_pin) {
    if (initialized_) {
        deinit();
    }
    
    tx_pin_ = tx_pin;
    rx_pin_ = rx_pin;
    baudrate_ = baudrate;
    
    // 初始化UART0
    uart_init(uart0, baudrate);
    
    // 设置GPIO功能
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // 配置硬件流控引脚
    if (flow_control && cts_pin != 255 && rts_pin != 255) {
        gpio_set_function(cts_pin, GPIO_FUNC_UART); // CTS
        gpio_set_function(rts_pin, GPIO_FUNC_UART); // RTS
    }
    
    // 配置UART参数
    uart_set_hw_flow(uart0, flow_control, flow_control);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    
    // 启用FIFO
    uart_set_fifo_enabled(uart0, true);
    
    // 清空缓冲区
    rx_head_ = 0;
    rx_tail_ = 0;
    
    // 设置中断
    irq_set_exclusive_handler(UART0_IRQ, irq_handler);
    irq_set_enabled(UART0_IRQ, true);
    
    // 启用接收中断
    uart_set_irq_enables(uart0, true, false);
    
    // 分配DMA通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_rx_channel_ = dma_claim_unused_channel(true);
    
    initialized_ = true;
    return true;
}

void HAL_UART0::deinit() {
    if (initialized_) {
        // 禁用中断
        uart_set_irq_enables(uart0, false, false);
        irq_set_enabled(UART0_IRQ, false);
        
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_rx_channel_ >= 0) {
            dma_channel_unclaim(dma_rx_channel_);
            dma_rx_channel_ = -1;
        }
        
        // 反初始化UART
        uart_deinit(uart0);
        
        initialized_ = false;
    }
}

// 内联函数：写入数据到TX环形缓冲区
inline size_t HAL_UART0::write_to_tx_buffer(const uint8_t* data, size_t length) {
    if (!initialized_ || !data) {
        return 0;
    }
    
    size_t written = 0;
    for (size_t i = 0; i < length && written < length; i++) {
        size_t next_head = (tx_head_ + 1) % TX_BUFFER_SIZE;
        if (next_head == tx_tail_) {
            break; // 缓冲区满
        }
        tx_buffer_[tx_head_] = data[i];
        tx_head_ = next_head;
        written++;
    }
    
    return written;
}

// 内联函数：从RX环形缓冲区读取数据
inline size_t HAL_UART0::read_from_rx_buffer(uint8_t* buffer, size_t length) {
    if (!initialized_ || !buffer) {
        return 0;
    }
    
    size_t read_count = 0;
    while (read_count < length && rx_head_ != rx_tail_) {
        buffer[read_count] = rx_buffer_[rx_tail_];
        rx_tail_ = (rx_tail_ + 1) % RX_BUFFER_SIZE;
        read_count++;
    }
    
    return read_count;
}

size_t HAL_UART0::available() {
    if (!initialized_) return 0;
    
    if (rx_head_ >= rx_tail_) {
        return rx_head_ - rx_tail_;
    } else {
        return RX_BUFFER_SIZE - rx_tail_ + rx_head_;
    }
}

void HAL_UART0::flush_rx() {
    rx_head_ = 0;
    rx_tail_ = 0;
}

void HAL_UART0::flush_tx() {
    if (initialized_) {
        // 等待发送完成
        while (!uart_is_writable(uart0)) {
            tight_loop_contents();
        }
    }
}

void HAL_UART0::set_rx_callback(std::function<void(uint8_t)> callback) {
    rx_callback_ = callback;
}

void HAL_UART0::irq_handler() {
    if (instance_) {
        instance_->handle_rx_irq();
    }
}

void HAL_UART0::handle_rx_irq() {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        
        // 存储到环形缓冲区
        size_t next_head = (rx_head_ + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail_) {
            rx_buffer_[rx_head_] = ch;
            rx_head_ = next_head;
        }
        
        // 调用回调函数
        if (rx_callback_) {
            rx_callback_(ch);
        }
    }
}

// DMA中断处理函数
void dma_uart1_tx_complete() {
    HAL_UART1* instance = HAL_UART1::getInstance();
    if (instance) {
        // 更新tx_tail_指针
        size_t transmitted_length = dma_channel_hw_addr(instance->dma_tx_channel_)->transfer_count;
        instance->tx_tail_ = (instance->tx_tail_ + transmitted_length) % instance->TX_BUFFER_SIZE;
        instance->tx_dma_active_ = false;
        
        // 如果还有数据需要传输，继续DMA
        if (instance->tx_head_ != instance->tx_tail_) {
            instance->trigger_tx_dma();
        }
        
        instance->dma_busy_ = false;
        if (instance->dma_callback_) {
            instance->dma_callback_(true);
        }
    }
}

void dma_uart1_rx_complete() {
    HAL_UART1* instance = HAL_UART1::getInstance();
    instance->dma_busy_ = false;
    if (instance->dma_callback_) {
        instance->dma_callback_(true);
    }
}

// HAL_UART1 DMA异步方法
// 内联函数：触发TX DMA传输
inline void HAL_UART1::trigger_tx_dma() {
    if (!initialized_ || tx_dma_active_ || tx_head_ == tx_tail_) {
        return;
    }
    
    // 计算可传输的数据长度
    size_t data_length;
    if (tx_head_ > tx_tail_) {
        data_length = tx_head_ - tx_tail_;
    } else {
        data_length = TX_BUFFER_SIZE - tx_tail_;
    }
    
    if (data_length == 0) {
        return;
    }
    
    tx_dma_active_ = true;
    
    // 配置DMA通道
    dma_channel_config c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, true));
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_tx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_uart1_tx_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &uart_get_hw(uart1)->dr,
        &tx_buffer_[tx_tail_],
        data_length,
        true
    );
}

// 内联函数：获取TX缓冲区空闲空间
inline size_t HAL_UART1::get_tx_buffer_free_space() const {
    if (tx_head_ >= tx_tail_) {
        return TX_BUFFER_SIZE - (tx_head_ - tx_tail_) - 1;
    } else {
        return tx_tail_ - tx_head_ - 1;
    }
}

// 内联函数：获取RX缓冲区数据数量
inline size_t HAL_UART1::get_rx_buffer_data_count() const {
    if (rx_head_ >= rx_tail_) {
        return rx_head_ - rx_tail_;
    } else {
        return RX_BUFFER_SIZE - (rx_tail_ - rx_head_);
    }
}

bool HAL_UART1::write_dma(const uint8_t* data, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_ || dma_tx_channel_ < 0) {
        return false;
    }
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 配置DMA通道
    dma_channel_config c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, true)); // TX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_tx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_uart1_tx_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &uart_get_hw(uart1)->dr, // 写入地址
        data,                    // 读取地址
        length,                  // 传输长度
        true                     // 立即启动
    );
    
    return true;
}

bool HAL_UART1::read_dma(uint8_t* buffer, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_ || dma_rx_channel_ < 0) {
        return false;
    }
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 配置DMA通道
    dma_channel_config c = dma_channel_get_default_config(dma_rx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, false)); // RX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_rx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_uart1_rx_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_rx_channel_,
        &c,
        buffer,                   // 写入地址
        &uart_get_hw(uart1)->dr, // 读取地址
        length,                   // 传输长度
        true                      // 立即启动
    );
    
    return true;
}

bool HAL_UART1::is_busy() const {
    return dma_busy_;
}

// get_name() 和 is_ready() 已在头文件中内联定义

// DMA中断处理函数
void dma_uart0_tx_complete() {
    HAL_UART0* instance = HAL_UART0::getInstance();
    if (instance) {
        // 更新tx_tail_指针
        size_t transmitted_length = dma_channel_hw_addr(instance->dma_tx_channel_)->transfer_count;
        instance->tx_tail_ = (instance->tx_tail_ + transmitted_length) % instance->TX_BUFFER_SIZE;
        instance->tx_dma_active_ = false;
        
        // 如果还有数据需要传输，继续DMA
        if (instance->tx_head_ != instance->tx_tail_) {
            instance->trigger_tx_dma();
        }
        
        instance->dma_busy_ = false;
        if (instance->dma_callback_) {
            instance->dma_callback_(true);
        }
    }
}

void dma_uart0_rx_complete() {
    HAL_UART0* instance = HAL_UART0::getInstance();
    instance->dma_busy_ = false;
    if (instance->dma_callback_) {
        instance->dma_callback_(true);
    }
}

// HAL_UART0 DMA异步方法
// 内联函数：触发TX DMA传输
inline void HAL_UART0::trigger_tx_dma() {
    if (!initialized_ || tx_dma_active_ || tx_head_ == tx_tail_) {
        return;
    }
    
    // 计算可传输的数据长度
    size_t data_length;
    if (tx_head_ > tx_tail_) {
        data_length = tx_head_ - tx_tail_;
    } else {
        data_length = TX_BUFFER_SIZE - tx_tail_;
    }
    
    if (data_length == 0) {
        return;
    }
    
    tx_dma_active_ = true;
    
    // 配置DMA通道
    dma_channel_config c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, true));
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_tx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_uart0_tx_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &uart_get_hw(uart0)->dr,
        &tx_buffer_[tx_tail_],
        data_length,
        true
    );
}

// 内联函数：获取TX缓冲区空闲空间
inline size_t HAL_UART0::get_tx_buffer_free_space() const {
    if (tx_head_ >= tx_tail_) {
        return TX_BUFFER_SIZE - (tx_head_ - tx_tail_) - 1;
    } else {
        return tx_tail_ - tx_head_ - 1;
    }
}

// 内联函数：获取RX缓冲区数据数量
inline size_t HAL_UART0::get_rx_buffer_data_count() const {
    if (rx_head_ >= rx_tail_) {
        return rx_head_ - rx_tail_;
    } else {
        return RX_BUFFER_SIZE - (rx_tail_ - rx_head_);
    }
}

bool HAL_UART0::write_dma(const uint8_t* data, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_ || dma_tx_channel_ < 0) {
        return false;
    }
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 配置DMA通道
    dma_channel_config c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, true)); // TX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_tx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_uart0_tx_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &uart_get_hw(uart0)->dr, // 写入地址
        data,                    // 读取地址
        length,                  // 传输长度
        true                     // 立即启动
    );
    
    return true;
}

bool HAL_UART0::read_dma(uint8_t* buffer, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_ || dma_rx_channel_ < 0) {
        return false;
    }
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 配置DMA通道
    dma_channel_config c = dma_channel_get_default_config(dma_rx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, false)); // RX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_rx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_uart0_rx_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_rx_channel_,
        &c,
        buffer,                   // 写入地址
        &uart_get_hw(uart0)->dr, // 读取地址
        length,                   // 传输长度
        true                      // 立即启动
    );
    
    return true;
}

bool HAL_UART0::is_busy() const {
    return dma_busy_;
}

// get_name() is already defined inline in header

// is_ready() is already defined inline in header

// HAL_UART1 静态成员初始化
HAL_UART1* HAL_UART1::instance_ = nullptr;

// HAL_UART1 实现
HAL_UART1* HAL_UART1::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_UART1();
    }
    return instance_;
}

HAL_UART1::HAL_UART1() 
    : initialized_(false), tx_pin_(0), rx_pin_(0), baudrate_(115200),
      dma_busy_(false), dma_tx_channel_(-1), dma_rx_channel_(-1),
      rx_head_(0), rx_tail_(0), tx_head_(0), tx_tail_(0), tx_dma_active_(false) {
}

HAL_UART1::~HAL_UART1() {
    deinit();
    instance_ = nullptr;
}

bool HAL_UART1::init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate, bool flow_control, uint8_t cts_pin, uint8_t rts_pin) {
    if (initialized_) {
        deinit();
    }
    
    tx_pin_ = tx_pin;
    rx_pin_ = rx_pin;
    baudrate_ = baudrate;
    
    // 初始化UART1
    uart_init(uart1, baudrate);
    
    // 设置GPIO功能
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // 配置硬件流控引脚
    if (flow_control && cts_pin != 255 && rts_pin != 255) {
        gpio_set_function(cts_pin, GPIO_FUNC_UART); // CTS
        gpio_set_function(rts_pin, GPIO_FUNC_UART); // RTS
    }
    
    // 配置UART参数
    uart_set_hw_flow(uart1, flow_control, flow_control);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    
    // 启用FIFO
    uart_set_fifo_enabled(uart1, true);
    
    // 清空缓冲区
    rx_head_ = 0;
    rx_tail_ = 0;
    
    // 设置中断
    irq_set_exclusive_handler(UART1_IRQ, irq_handler);
    irq_set_enabled(UART1_IRQ, true);
    
    // 启用接收中断
    uart_set_irq_enables(uart1, true, false);
    
    // 分配DMA通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_rx_channel_ = dma_claim_unused_channel(true);
    
    initialized_ = true;
    return true;
}

void HAL_UART1::deinit() {
    if (initialized_) {
        // 禁用中断
        uart_set_irq_enables(uart1, false, false);
        irq_set_enabled(UART1_IRQ, false);
        
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_rx_channel_ >= 0) {
            dma_channel_unclaim(dma_rx_channel_);
            dma_rx_channel_ = -1;
        }
        
        // 反初始化UART
        uart_deinit(uart1);
        
        initialized_ = false;
    }
}

// 内联函数：写入数据到TX环形缓冲区
inline size_t HAL_UART1::write_to_tx_buffer(const uint8_t* data, size_t length) {
    if (!initialized_ || !data) {
        return 0;
    }
    
    size_t written = 0;
    for (size_t i = 0; i < length && written < length; i++) {
        size_t next_head = (tx_head_ + 1) % TX_BUFFER_SIZE;
        if (next_head == tx_tail_) {
            break; // 缓冲区满
        }
        tx_buffer_[tx_head_] = data[i];
        tx_head_ = next_head;
        written++;
    }
    
    return written;
}

// 内联函数：从RX环形缓冲区读取数据
inline size_t HAL_UART1::read_from_rx_buffer(uint8_t* buffer, size_t length) {
    if (!initialized_ || !buffer) {
        return 0;
    }
    
    size_t read_count = 0;
    while (read_count < length && rx_head_ != rx_tail_) {
        buffer[read_count] = rx_buffer_[rx_tail_];
        rx_tail_ = (rx_tail_ + 1) % RX_BUFFER_SIZE;
        read_count++;
    }
    
    return read_count;
}

size_t HAL_UART1::available() {
    if (!initialized_) return 0;
    
    if (rx_head_ >= rx_tail_) {
        return rx_head_ - rx_tail_;
    } else {
        return RX_BUFFER_SIZE - rx_tail_ + rx_head_;
    }
}

void HAL_UART1::flush_rx() {
    rx_head_ = 0;
    rx_tail_ = 0;
}

void HAL_UART1::flush_tx() {
    if (initialized_) {
        // 等待发送完成
        while (!uart_is_writable(uart1)) {
            tight_loop_contents();
        }
    }
}

void HAL_UART1::set_rx_callback(std::function<void(uint8_t)> callback) {
    rx_callback_ = callback;
}

void HAL_UART1::irq_handler() {
    if (instance_) {
        instance_->handle_rx_irq();
    }
}

void HAL_UART1::handle_rx_irq() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        
        // 存储到环形缓冲区
        size_t next_head = (rx_head_ + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail_) {
            rx_buffer_[rx_head_] = ch;
            rx_head_ = next_head;
        }
        
        // 调用回调函数
        if (rx_callback_) {
            rx_callback_(ch);
        }
    }
}