#pragma once

#include <stdint.h>
#include <string>
#include <functional>

// 包含Pico SDK头文件
extern "C" {
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "../global_irq.h"
}

/**
 * HAL层 - UART接口抽象类
 * 提供底层UART接口，支持UART0和UART1两个实例
 * 使用DMA实现高效的数据传输和环形缓冲区
 */

class HAL_UART {
public:
    using dma_callback_t = std::function<void(bool success)>;
    
    virtual ~HAL_UART() = default;
    
    // 初始化UART接口
    virtual bool init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate = 115200, bool flow_control = false, uint8_t cts_pin = 255, uint8_t rts_pin = 255) = 0;
    
    // 释放UART资源
    virtual void deinit() = 0;
    
    // 缓冲区操作接口 - 自动处理DMA传输
    virtual inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) = 0;
    virtual inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) = 0;
    virtual inline size_t get_tx_buffer_free_space() const = 0;
    virtual inline size_t get_rx_buffer_data_count() const = 0;
    
    // 检查DMA传输状态
    virtual bool is_busy() const = 0;
    
    // 检查可读数据数量
    virtual size_t available() = 0;
    
    // 清空接收缓冲区
    virtual void flush_rx() = 0;
    
    // 清空发送缓冲区
    virtual void flush_tx() = 0;
    
    // 设置波特率（即时生效）
    virtual bool set_baudrate(uint32_t baudrate) = 0;
    
    // 设置接收回调函数
    virtual void set_rx_callback(std::function<void(uint8_t)> callback) = 0;
    
    // 获取实例名称
    virtual std::string get_name() const = 0;
    
    // 检查UART是否就绪
    virtual bool is_ready() const = 0;

    struct DmaControlBlock {
        uint32_t len = 1;
        char* data;
    };
};

// UART0实例
class HAL_UART0 : public HAL_UART {
public:
    static HAL_UART0* getInstance();
    ~HAL_UART0();
    
    bool init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate = 115200, bool flow_control = false, uint8_t cts_pin = 255, uint8_t rts_pin = 255) override;
    void deinit() override;

    inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) override;
    inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) override;
    inline size_t get_tx_buffer_free_space() const override;
    inline size_t get_rx_buffer_data_count() const override;
    bool is_busy() const override;

    size_t available() override;
    void flush_rx() override;
    void flush_tx() override;
    void set_rx_callback(std::function<void(uint8_t)> callback) override;
    bool set_baudrate(uint32_t baudrate) override;
    std::string get_name() const override { return "UART0"; }
    bool is_ready() const override { return initialized_; }
    
private:
    static void irq_handler();
    void handle_rx_irq();
    inline void trigger_tx_dma(size_t length); // 内部私有方法，自动处理DMA传输
    
    bool initialized_;
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint32_t baudrate_;
    std::function<void(uint8_t)> rx_callback_;
    bool dma_busy_;
    dma_callback_t dma_callback_;
    int dma_tx_channel_;
    int dma_ctrl_channel_;  // DMA控制通道
    
    // 友元函数声明，仅保留TX回调
    friend void uart0_tx_dma_callback(bool success);
    
    // RX静态缓冲区结构体
    struct RxBuffer {
        static constexpr size_t BUFFER_SIZE = 256;
        static uint8_t buffer[BUFFER_SIZE];
        static uint8_t* write_ptr;
        static uint8_t* read_ptr;
        static size_t data_count;
    } rx_buffer_;
    
    // TX DMA缓冲区结构体 这是罕见级DMA的必要操作 必须构造一个管道去操作第二个管道循环运行
    struct TxBuffer {
        static constexpr size_t BUFFER_SIZE = 256;
        static char data_buffer[BUFFER_SIZE];
        static DmaControlBlock control_buffer[BUFFER_SIZE + 1];
    } tx_buffer_;
    
    static HAL_UART0* instance_;
    
    // 私有构造函数（单例模式）
    HAL_UART0();
    HAL_UART0(const HAL_UART0&) = delete;
    HAL_UART0& operator=(const HAL_UART0&) = delete;
};

// UART1实例
class HAL_UART1 : public HAL_UART {
public:
    static HAL_UART1* getInstance();
    ~HAL_UART1();
    
    bool init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate = 115200, bool flow_control = false, uint8_t cts_pin = 255, uint8_t rts_pin = 255) override;
    void deinit() override;

    inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) override;
    inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) override;
    inline size_t get_tx_buffer_free_space() const override;
    inline size_t get_rx_buffer_data_count() const override;
    bool is_busy() const override;
    size_t available() override;
    void flush_rx() override;
    void flush_tx() override;
    void set_rx_callback(std::function<void(uint8_t)> callback) override;
    bool set_baudrate(uint32_t baudrate) override;
    std::string get_name() const override { return "UART1"; }
    bool is_ready() const override { return initialized_; }
    
private:
    static void irq_handler();
    void handle_rx_irq();
    inline void trigger_tx_dma(size_t length); // 内部私有方法，自动处理DMA传输
    
    bool initialized_;
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint32_t baudrate_;
    std::function<void(uint8_t)> rx_callback_;
    bool dma_busy_;
    dma_callback_t dma_callback_;
    int dma_tx_channel_;
    int dma_ctrl_channel_;  // DMA控制通道

    // 友元函数声明，仅保留TX回调
    friend void uart1_tx_dma_callback(bool success);
    
    // RX静态缓冲区结构体
    struct RxBuffer {
        static constexpr size_t BUFFER_SIZE = 256;
        static uint8_t buffer[BUFFER_SIZE];
        static uint8_t* write_ptr;
        static uint8_t* read_ptr;
        static size_t data_count;
    } rx_buffer_;
    
    // TX DMA缓冲区结构体
    struct TxBuffer {
        static constexpr size_t BUFFER_SIZE = 256;
        static char data_buffer[BUFFER_SIZE];
        static DmaControlBlock control_buffer[BUFFER_SIZE + 1];
    } tx_buffer_;
    
    static HAL_UART1* instance_;
    
    // 私有构造函数（单例模式）
    HAL_UART1();
    HAL_UART1(const HAL_UART1&) = delete;
    HAL_UART1& operator=(const HAL_UART1&) = delete;
};