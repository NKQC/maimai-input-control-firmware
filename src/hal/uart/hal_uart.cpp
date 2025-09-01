#include "hal_uart.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/dma.h>
#include <pico/stdlib.h>
#include <cstring>

// 包含全局中断管理
#include "../global_irq.h"

// 仅保留TX DMA回调函数声明
void uart0_tx_dma_callback(bool success);
void uart1_tx_dma_callback(bool success);
// HAL_UART0 静态成员初始化
HAL_UART0* HAL_UART0::instance_ = nullptr;

// UART0
uint8_t HAL_UART0::RxBuffer::buffer[HAL_UART0::RxBuffer::BUFFER_SIZE];
uint8_t* HAL_UART0::RxBuffer::write_ptr = HAL_UART0::RxBuffer::buffer;
uint8_t* HAL_UART0::RxBuffer::read_ptr = HAL_UART0::RxBuffer::buffer;
size_t HAL_UART0::RxBuffer::data_count = 0;
char HAL_UART0::TxBuffer::data_buffer[HAL_UART0::TxBuffer::BUFFER_SIZE];
HAL_UART0::DmaControlBlock HAL_UART0::TxBuffer::control_buffer[HAL_UART0::TxBuffer::BUFFER_SIZE + 1];

// UART1
uint8_t HAL_UART1::RxBuffer::buffer[HAL_UART1::RxBuffer::BUFFER_SIZE];
uint8_t* HAL_UART1::RxBuffer::write_ptr = HAL_UART1::RxBuffer::buffer;
uint8_t* HAL_UART1::RxBuffer::read_ptr = HAL_UART1::RxBuffer::buffer;
size_t HAL_UART1::RxBuffer::data_count = 0;
char HAL_UART1::TxBuffer::data_buffer[HAL_UART1::TxBuffer::BUFFER_SIZE];
HAL_UART1::DmaControlBlock HAL_UART1::TxBuffer::control_buffer[HAL_UART1::TxBuffer::BUFFER_SIZE + 1];

// HAL_UART0 实现
HAL_UART0* HAL_UART0::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_UART0();
    }
    return instance_;
}

HAL_UART0::HAL_UART0() 
    : initialized_(false), tx_pin_(0), rx_pin_(0), baudrate_(115200),
      dma_busy_(false), dma_tx_channel_(-1), dma_ctrl_channel_(-1) {
    // 缓冲区结构体会自动初始化
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
    
    // 重置缓冲区状态
    rx_buffer_.write_ptr = rx_buffer_.buffer;
    rx_buffer_.read_ptr = rx_buffer_.buffer;
    rx_buffer_.data_count = 0;

    // 设置中断（仅用于接收）
    irq_set_exclusive_handler(UART0_IRQ, irq_handler);
    irq_set_enabled(UART0_IRQ, true);
    
    // 启用接收中断
    uart_set_irq_enables(uart0, true, false);
    
    // 分配DMA通道 - 数据通道和控制通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_ctrl_channel_ = dma_claim_unused_channel(true);

    // 仅为TX通道注册DMA回调到全局中断管理系统
    bool tx_registered = global_irq_register_dma_callback(dma_tx_channel_, uart0_tx_dma_callback);
    
    initialized_ = tx_registered && dma_tx_channel_ >= 0 && dma_ctrl_channel_ >= 0;
    
    return initialized_;
}

void HAL_UART0::deinit() {
    if (initialized_) {
        // 禁用中断
        uart_set_irq_enables(uart0, false, false);
        irq_set_enabled(UART0_IRQ, false);
        
        // 注销DMA回调（仅TX）
        if (dma_tx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_tx_channel_);
        }
        
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_ctrl_channel_ >= 0) {
            dma_channel_unclaim(dma_ctrl_channel_);
            dma_ctrl_channel_ = -1;
        }
        
        // 反初始化UART
        uart_deinit(uart0);
        
        initialized_ = false;
    }
}

// 内联函数：写入数据到TX单缓冲区并直接发起DMA传输
inline size_t HAL_UART0::write_to_tx_buffer(const uint8_t* data, size_t length) {
    if (!initialized_ || !data || length == 0) {
        return 0;
    }
    
    // 如果DMA忙，直接返回0，不缓存数据
    if (dma_busy_) {
        return 0;
    }
    
    // 限制传输长度不超过缓冲区大小
    size_t to_write = (length > TxBuffer::BUFFER_SIZE) ? TxBuffer::BUFFER_SIZE : length;
    
    // 复制数据到缓冲区
    for (size_t i = 0; i < to_write; i++) {
        tx_buffer_.data_buffer[i] = data[i];
        tx_buffer_.control_buffer[i].len = 1;
        tx_buffer_.control_buffer[i].data = &tx_buffer_.data_buffer[i];
    }
    
    // 直接发起DMA传输
    trigger_tx_dma(to_write);
    
    return to_write;
}

// 内联函数：从RX静态缓冲区读取数据
inline size_t HAL_UART0::read_from_rx_buffer(uint8_t* buffer, size_t length) {
    if (!initialized_ || !buffer) {
        return 0;
    }
    
    size_t to_read = (length > rx_buffer_.data_count) ? rx_buffer_.data_count : length;
    
    if (to_read == 0) {
        return 0; // 没有数据可读
    }
    
    // 计算从读指针到缓冲区末尾的数据长度
    size_t end_length = rx_buffer_.buffer + RxBuffer::BUFFER_SIZE - rx_buffer_.read_ptr;
    
    if (to_read <= end_length) {
        // 数据不跨越缓冲区边界
        memcpy(buffer, (void*)rx_buffer_.read_ptr, to_read);
        rx_buffer_.read_ptr += to_read;
    } else {
        // 数据跨越缓冲区边界
        memcpy(buffer, (void*)rx_buffer_.read_ptr, end_length);
        memcpy(buffer + end_length, rx_buffer_.buffer, to_read - end_length);
        rx_buffer_.read_ptr = rx_buffer_.buffer + (to_read - end_length);
    }
    
    // 处理读指针回绕
    if (rx_buffer_.read_ptr >= rx_buffer_.buffer + RxBuffer::BUFFER_SIZE) {
        rx_buffer_.read_ptr = rx_buffer_.buffer;
    }
    
    rx_buffer_.data_count -= to_read;
    
    return to_read;
}

size_t HAL_UART0::available() {
    if (!initialized_) return 0;
    
    return rx_buffer_.data_count;
}

void HAL_UART0::flush_rx() {
    rx_buffer_.write_ptr = rx_buffer_.buffer;
    rx_buffer_.read_ptr = rx_buffer_.buffer;
    rx_buffer_.data_count = 0;
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
        
        // 存储到静态缓冲区（如果有空间）
        if (rx_buffer_.data_count < RxBuffer::BUFFER_SIZE) {
            *rx_buffer_.write_ptr = ch;
            rx_buffer_.write_ptr++;
            
            // 处理写指针回绕
            if (rx_buffer_.write_ptr >= rx_buffer_.buffer + RxBuffer::BUFFER_SIZE) {
                rx_buffer_.write_ptr = rx_buffer_.buffer;
            }
            
            rx_buffer_.data_count++;
        }
        
        // 调用回调函数
        if (rx_callback_) {
            rx_callback_(ch);
        }
    }
}

// DMA中断处理函数
// C风格的UART1 DMA回调函数实现
void uart1_tx_dma_callback(bool success) {
    HAL_UART1* instance = HAL_UART1::getInstance();
    if (!instance || instance->dma_tx_channel_ < 0) {
        return;
    }

    // 清除DMA忙标志
    instance->dma_busy_ = false;
    
    // 调用用户回调
    if (instance->dma_callback_) {
        instance->dma_callback_(success);
    }
}

// HAL_UART1 DMA异步方法
// 内联函数：触发TX DMA传输（单缓冲区模式）
// 触发TX DMA传输（双通道控制模式）
inline void HAL_UART1::trigger_tx_dma(size_t length) {
    if (!initialized_ || dma_busy_ || length == 0) {
        return;
    }
    
    dma_busy_ = true;

    tx_buffer_.control_buffer[length].len = 0;
    tx_buffer_.control_buffer[length].data = NULL;

    // 控制通道配置：32位、读写自增，写指针按8字节环绕，写入data通道别名3的TRANS_COUNT和READ_ADDR
    dma_channel_config c_ctrl = dma_channel_get_default_config(dma_ctrl_channel_);
    channel_config_set_transfer_data_size(&c_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl, true);
    channel_config_set_write_increment(&c_ctrl, true);
    channel_config_set_ring(&c_ctrl, true, 3); // 1<<3 = 8字节边界

    dma_channel_configure(
        dma_ctrl_channel_,
        &c_ctrl,
        &dma_hw->ch[dma_tx_channel_].al3_transfer_count,
        &tx_buffer_.control_buffer[0],
        2,      // 每次写两个32位词：len -> TRANS_COUNT, data -> READ_ADDR
        false   // 暂不启动
    );

    // 数据通道配置：8位、读自增、写不增、按UART TX DREQ节流；完成后链回控制通道；quiet以便在结束块触发IRQ
    dma_channel_config c_data = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c_data, DMA_SIZE_8);
    channel_config_set_dreq(&c_data, uart_get_dreq(uart1, true));
    channel_config_set_chain_to(&c_data, dma_ctrl_channel_);
    channel_config_set_irq_quiet(&c_data, true);

    dma_channel_configure(
        dma_tx_channel_,
        &c_data,
        &uart_get_hw(uart1)->dr,
        NULL,   // READ_ADDR和TRANS_COUNT由控制通道装载
        0,
        false
    );

    // 启动控制通道装载首个控制块
    dma_start_channel_mask(1u << dma_ctrl_channel_);
}

// 内联函数：获取TX缓冲区剩余空间
inline size_t HAL_UART1::get_tx_buffer_free_space() const {
    return dma_busy_ ? 0 : TxBuffer::BUFFER_SIZE;
}

// 内联函数：获取RX缓冲区数据数量
inline size_t HAL_UART1::get_rx_buffer_data_count() const {
    return rx_buffer_.data_count;
}

bool HAL_UART1::is_busy() const {
    return dma_busy_;
}

// C风格的DMA回调函数实现
void uart0_tx_dma_callback(bool success) {
    HAL_UART0* instance = HAL_UART0::getInstance();
    if (!instance || instance->dma_tx_channel_ < 0) {
        return;
    }
    
    // 清除DMA忙标志
    instance->dma_busy_ = false;
    
    // 调用用户回调
    if (instance->dma_callback_) {
        instance->dma_callback_(success);
    }
}

// 触发TX DMA传输（双通道控制模式）
inline void HAL_UART0::trigger_tx_dma(size_t length) {
    if (!initialized_ || dma_busy_ || length == 0) {
        return;
    }
    
    // 设置DMA忙标志
    dma_busy_ = true;

    tx_buffer_.control_buffer[length].len = 0;
    tx_buffer_.control_buffer[length].data = NULL;

    // 控制通道配置：32位、读写自增，写指针按8字节环绕，写入data通道别名3的TRANS_COUNT和READ_ADDR
    dma_channel_config c_ctrl = dma_channel_get_default_config(dma_ctrl_channel_);
    channel_config_set_transfer_data_size(&c_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl, true);
    channel_config_set_write_increment(&c_ctrl, true);
    channel_config_set_ring(&c_ctrl, true, 3); // 1<<3 = 8字节边界

    dma_channel_configure(
        dma_ctrl_channel_,
        &c_ctrl,
        &dma_hw->ch[dma_tx_channel_].al3_transfer_count,
        &tx_buffer_.control_buffer[0],
        2,      // 每次写两个32位词：len -> TRANS_COUNT, data -> READ_ADDR
        false   // 暂不启动
    );
    
    // 数据通道配置：8位、读自增、写不增、按UART TX DREQ节流；完成后链回控制通道；quiet以便在结束块触发IRQ
    dma_channel_config c_data = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c_data, DMA_SIZE_8);
    channel_config_set_dreq(&c_data, uart_get_dreq(uart0, true));
    channel_config_set_chain_to(&c_data, dma_ctrl_channel_);
    channel_config_set_irq_quiet(&c_data, true);

    dma_channel_configure(
        dma_tx_channel_,
        &c_data,
        &uart_get_hw(uart0)->dr,
        NULL,   // READ_ADDR和TRANS_COUNT由控制通道装载
        0,
        false
    );
 
    // 启动控制通道装载首个控制块
    dma_start_channel_mask(1u << dma_ctrl_channel_);
}

// 内联函数：获取TX缓冲区剩余空间
inline size_t HAL_UART0::get_tx_buffer_free_space() const {
    return dma_busy_ ? 0 : TxBuffer::BUFFER_SIZE;
}

// 内联函数：获取RX缓冲区数据数量
inline size_t HAL_UART0::get_rx_buffer_data_count() const {
    return rx_buffer_.data_count;
}

bool HAL_UART0::is_busy() const {
    return dma_busy_;
}

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
      dma_busy_(false), dma_tx_channel_(-1), dma_ctrl_channel_(-1) {
    // 缓冲区结构体会自动初始化
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
    
    // 重置缓冲区状态
    rx_buffer_.write_ptr = rx_buffer_.buffer;
    rx_buffer_.read_ptr = rx_buffer_.buffer;
    rx_buffer_.data_count = 0;

    // 设置中断（仅用于接收）
    irq_set_exclusive_handler(UART1_IRQ, irq_handler);
    irq_set_enabled(UART1_IRQ, true);
    
    // 启用接收中断
    uart_set_irq_enables(uart1, true, false);
    
    // 分配DMA通道 - 数据通道和控制通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_ctrl_channel_ = dma_claim_unused_channel(true);
    
    // 仅为TX通道注册DMA回调
    bool tx_registered = global_irq_register_dma_callback(dma_tx_channel_, uart1_tx_dma_callback);
    
    initialized_ = tx_registered && dma_tx_channel_ >= 0 && dma_ctrl_channel_ >= 0;
    
    return initialized_;
}

void HAL_UART1::deinit() {
    if (initialized_) {
        // 禁用中断
        uart_set_irq_enables(uart1, false, false);
        irq_set_enabled(UART1_IRQ, false);
        
        // 注销DMA回调（仅TX）
        if (dma_tx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_tx_channel_);
        }
        
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_ctrl_channel_ >= 0) {
            dma_channel_unclaim(dma_ctrl_channel_);
            dma_ctrl_channel_ = -1;
        }
        
        // 反初始化UART
        uart_deinit(uart1);
        
        initialized_ = false;
    }
}

// 内联函数：写入数据到TX单缓冲区并直接发起DMA传输
inline size_t HAL_UART1::write_to_tx_buffer(const uint8_t* data, size_t length) {
    if (!initialized_ || !data || length == 0) {
        return 0;
    }
    
    // 如果DMA忙，直接返回0，不缓存数据
    if (dma_busy_) {
        return 0;
    }
    
    // 限制传输长度不超过缓冲区大小
    size_t to_write = (length > TxBuffer::BUFFER_SIZE) ? TxBuffer::BUFFER_SIZE : length;
    
    // 复制数据到缓冲区
    for (size_t i = 0; i < to_write; i++) {
        tx_buffer_.data_buffer[i] = data[i];
        tx_buffer_.control_buffer[i].len = 1;
        tx_buffer_.control_buffer[i].data = &tx_buffer_.data_buffer[i];
    }
    
    // 直接发起DMA传输
    trigger_tx_dma(to_write);
    
    return to_write;
}

// 内联函数：从RX静态缓冲区读取数据
inline size_t HAL_UART1::read_from_rx_buffer(uint8_t* buffer, size_t length) {
    if (!initialized_ || !buffer) {
        return 0;
    }
    
    size_t to_read = (length > rx_buffer_.data_count) ? rx_buffer_.data_count : length;
    if (to_read == 0) {
        return 0;
    }
    
    size_t read_count = 0;
    while (read_count < to_read) {
        buffer[read_count] = *rx_buffer_.read_ptr;
        read_count++;
        
        // 更新读指针，处理环绕
        rx_buffer_.read_ptr++;
        if (rx_buffer_.read_ptr >= rx_buffer_.buffer + RxBuffer::BUFFER_SIZE) {
            rx_buffer_.read_ptr = rx_buffer_.buffer;
        }
    }
    
    // 更新数据计数
    rx_buffer_.data_count -= read_count;
    
    return read_count;
}

size_t HAL_UART1::available() {
    if (!initialized_) return 0;
    
    return rx_buffer_.data_count;
}

void HAL_UART1::flush_rx() {
    rx_buffer_.write_ptr = rx_buffer_.buffer;
    rx_buffer_.read_ptr = rx_buffer_.buffer;
    rx_buffer_.data_count = 0;
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
        
        // 存储到静态缓冲区（如果有空间）
        if (rx_buffer_.data_count < RxBuffer::BUFFER_SIZE) {
            *rx_buffer_.write_ptr = ch;
            rx_buffer_.data_count++;
            
            // 更新写指针，处理环绕
            rx_buffer_.write_ptr++;
            if (rx_buffer_.write_ptr >= rx_buffer_.buffer + RxBuffer::BUFFER_SIZE) {
                rx_buffer_.write_ptr = rx_buffer_.buffer;
            }
        }
        
        // 调用回调函数
        if (rx_callback_) {
            rx_callback_(ch);
        }
    }
}