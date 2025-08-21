#pragma once

#include <stdint.h>
#include <string>
#include <functional>

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
    virtual bool init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate = 115200, bool flow_control = false) = 0;
    
    // 释放UART资源
    virtual void deinit() = 0;
    
    // DMA操作 - 完全DMA化
    virtual bool write_dma(const uint8_t* data, size_t length, dma_callback_t callback = nullptr) = 0;
    virtual bool read_dma(uint8_t* buffer, size_t length, dma_callback_t callback = nullptr) = 0;
    
    // 内联环形缓冲区操作
    virtual inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) = 0;
    virtual inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) = 0;
    virtual inline void trigger_tx_dma() = 0;
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
    
    // 设置接收回调函数
    virtual void set_rx_callback(std::function<void(uint8_t)> callback) = 0;
    
    // 获取实例名称
    virtual std::string get_name() const = 0;
    
    // 检查UART是否就绪
    virtual bool is_ready() const = 0;
};

// UART0实例
class HAL_UART0 : public HAL_UART {
public:
    static HAL_UART0* getInstance();
    ~HAL_UART0();
    
    bool init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate = 115200, bool flow_control = false) override;
    void deinit() override;
    bool write_dma(const uint8_t* data, size_t length, dma_callback_t callback = nullptr) override;
    bool read_dma(uint8_t* buffer, size_t length, dma_callback_t callback = nullptr) override;
    inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) override;
    inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) override;
    inline void trigger_tx_dma() override;
    inline size_t get_tx_buffer_free_space() const override;
    inline size_t get_rx_buffer_data_count() const override;
    bool is_busy() const override;
    size_t available() override;
    void flush_rx() override;
    void flush_tx() override;
    void set_rx_callback(std::function<void(uint8_t)> callback) override;
    std::string get_name() const override { return "UART0"; }
    bool is_ready() const override { return initialized_; }
    
private:
    static void irq_handler();
    void handle_rx_irq();
    
    bool initialized_;
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint32_t baudrate_;
    std::function<void(uint8_t)> rx_callback_;
    bool dma_busy_;
    dma_callback_t dma_callback_;
    int dma_tx_channel_;
    int dma_rx_channel_;
    
    // RX环形缓冲区
    static constexpr size_t RX_BUFFER_SIZE = 256;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    volatile size_t rx_head_;
    volatile size_t rx_tail_;
    
    // TX环形缓冲区
    static constexpr size_t TX_BUFFER_SIZE = 256;
    uint8_t tx_buffer_[TX_BUFFER_SIZE];
    volatile size_t tx_head_;
    volatile size_t tx_tail_;
    volatile bool tx_dma_active_;
    
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
    
    bool init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate = 115200, bool flow_control = false) override;
    void deinit() override;
    bool write_dma(const uint8_t* data, size_t length, dma_callback_t callback = nullptr) override;
    bool read_dma(uint8_t* buffer, size_t length, dma_callback_t callback = nullptr) override;
    inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) override;
    inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) override;
    inline void trigger_tx_dma() override;
    inline size_t get_tx_buffer_free_space() const override;
    inline size_t get_rx_buffer_data_count() const override;
    bool is_busy() const override;
    size_t available() override;
    void flush_rx() override;
    void flush_tx() override;
    void set_rx_callback(std::function<void(uint8_t)> callback) override;
    std::string get_name() const override { return "UART1"; }
    bool is_ready() const override { return initialized_; }
    
private:
    static void irq_handler();
    void handle_rx_irq();
    
    bool initialized_;
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint32_t baudrate_;
    std::function<void(uint8_t)> rx_callback_;
    bool dma_busy_;
    dma_callback_t dma_callback_;
    int dma_tx_channel_;
    int dma_rx_channel_;
    
    // RX环形缓冲区
    static constexpr size_t RX_BUFFER_SIZE = 256;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    volatile size_t rx_head_;
    volatile size_t rx_tail_;
    
    // TX环形缓冲区
    static constexpr size_t TX_BUFFER_SIZE = 256;
    uint8_t tx_buffer_[TX_BUFFER_SIZE];
    volatile size_t tx_head_;
    volatile size_t tx_tail_;
    volatile bool tx_dma_active_;
    
    static HAL_UART1* instance_;
    
    // 私有构造函数（单例模式）
    HAL_UART1();
    HAL_UART1(const HAL_UART1&) = delete;
    HAL_UART1& operator=(const HAL_UART1&) = delete;
};