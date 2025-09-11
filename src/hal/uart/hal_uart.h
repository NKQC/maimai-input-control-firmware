#pragma once

#include <stdint.h>
#include <string>
#include <functional>

// 统一的串口波特率枚举定义
enum class UartBaudRate : uint32_t {
    BAUD_9600 = 9600,
    BAUD_115200 = 115200,
    BAUD_250000 = 250000,
    BAUD_500000 = 500000,
    BAUD_1000000 = 1000000,
    BAUD_1500000 = 1500000,
    BAUD_2000000 = 2000000,
    BAUD_2500000 = 2500000,
    BAUD_3000000 = 3000000,
    BAUD_4000000 = 4000000,
    BAUD_5000000 = 5000000,
    BAUD_6000000 = 6000000
};

// 波特率枚举到实际值的转换函数
inline uint32_t uart_baud_rate_to_value(UartBaudRate baud_rate) {
    return static_cast<uint32_t>(baud_rate);
}

// 实际值到波特率枚举的转换函数
inline UartBaudRate uart_value_to_baud_rate(uint32_t value) {
    switch (value) {
        case 9600: return UartBaudRate::BAUD_9600;
        case 115200: return UartBaudRate::BAUD_115200;
        case 250000: return UartBaudRate::BAUD_250000;
        case 500000: return UartBaudRate::BAUD_500000;
        case 1000000: return UartBaudRate::BAUD_1000000;
        case 1500000: return UartBaudRate::BAUD_1500000;
        case 2000000: return UartBaudRate::BAUD_2000000;
        case 2500000: return UartBaudRate::BAUD_2500000;
        case 3000000: return UartBaudRate::BAUD_3000000;
        case 4000000: return UartBaudRate::BAUD_4000000;
        case 5000000: return UartBaudRate::BAUD_5000000;
        case 6000000: return UartBaudRate::BAUD_6000000;
        default: return UartBaudRate::BAUD_115200; // 默认值
    }
}

// 获取所有支持的波特率数组
inline const uint32_t* get_supported_baud_rates() {
    static const uint32_t supported_rates[] = {
        9600, 115200, 250000, 500000, 1000000, 1500000,
        2000000, 2500000, 3000000, 4000000, 5000000, 6000000
    };
    return supported_rates;
}

// 获取支持的波特率数量
inline size_t get_supported_baud_rates_count() {
    return 12;
}

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
    int32_t dma_tx_channel_;
    int32_t dma_ctrl_channel_;  // DMA控制通道
    
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
    int32_t dma_tx_channel_;
    int32_t dma_ctrl_channel_;  // DMA控制通道

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