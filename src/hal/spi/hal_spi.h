#pragma once

#include <stdint.h>
#include <string>
#include <functional>

/**
 * HAL层 - SPI接口抽象类
 * 提供底层SPI接口，支持SPI0和SPI1两个实例
 * 使用DMA实现高效的数据传输
 */

class HAL_SPI {
public:
    using dma_callback_t = std::function<void(bool success)>;
    
    virtual ~HAL_SPI() = default;
    
    // 初始化SPI接口
    virtual bool init(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t frequency = 1000000) = 0;
    
    // 释放SPI资源
    virtual void deinit() = 0;
    
    // DMA操作 - 完全DMA化
    virtual bool write_dma(const uint8_t* data, size_t length, dma_callback_t callback = nullptr) = 0;
    virtual bool read_dma(uint8_t* buffer, size_t length, dma_callback_t callback = nullptr) = 0;
    virtual bool transfer_dma(const uint8_t* tx_data, uint8_t* rx_data, size_t length, dma_callback_t callback = nullptr) = 0;
    
    // 内联环形缓冲区操作
    virtual inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) = 0;
    virtual inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) = 0;
    virtual inline void trigger_tx_dma() = 0;
    virtual inline size_t get_tx_buffer_free_space() const = 0;
    virtual inline size_t get_rx_buffer_data_count() const = 0;
    
    // 检查DMA传输状态
    virtual bool is_busy() const = 0;
    
    // 设置CS引脚
    virtual void set_cs_pin(uint8_t cs_pin, bool active_low = true) = 0;
    
    // CS控制
    virtual void cs_select() = 0;
    virtual void cs_deselect() = 0;
    
    // 设置SPI模式和频率
    virtual void set_format(uint8_t data_bits, uint8_t cpol, uint8_t cpha) = 0;
    virtual void set_frequency(uint32_t frequency) = 0;
    
    // 获取实例名称
    virtual std::string get_name() const = 0;
    
    // 检查SPI是否就绪
    virtual bool is_ready() const = 0;
};

// SPI0实例
class HAL_SPI0 : public HAL_SPI {
public:
    static HAL_SPI0* getInstance();
    ~HAL_SPI0();
    
    bool init(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t frequency = 1000000) override;
    void deinit() override;
    bool write_dma(const uint8_t* data, size_t length, dma_callback_t callback = nullptr) override;
    bool read_dma(uint8_t* buffer, size_t length, dma_callback_t callback = nullptr) override;
    bool transfer_dma(const uint8_t* tx_data, uint8_t* rx_data, size_t length, dma_callback_t callback = nullptr) override;
    inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) override;
    inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) override;
    inline void trigger_tx_dma() override;
    inline size_t get_tx_buffer_free_space() const override;
    inline size_t get_rx_buffer_data_count() const override;
    bool is_busy() const override;
    void set_cs_pin(uint8_t cs_pin, bool active_low = true) override;
    void cs_select() override;
    void cs_deselect() override;
    void set_format(uint8_t data_bits, uint8_t cpol, uint8_t cpha) override;
    void set_frequency(uint32_t frequency) override;
    std::string get_name() const override { return "SPI0"; }
    bool is_ready() const override { return initialized_; }

    bool dma_busy_;
    dma_callback_t dma_callback_;
    
private:
    bool initialized_;
    uint8_t sck_pin_;
    uint8_t mosi_pin_;
    uint8_t miso_pin_;
    uint8_t cs_pin_;
    bool cs_active_low_;
    uint32_t frequency_;
    int dma_tx_channel_;
    int dma_rx_channel_;
    
    // TX环形缓冲区
    static constexpr size_t TX_BUFFER_SIZE = 256;
    uint8_t tx_buffer_[TX_BUFFER_SIZE];
    volatile size_t tx_head_;
    volatile size_t tx_tail_;
    volatile bool tx_dma_active_;
    
    // RX环形缓冲区
    static constexpr size_t RX_BUFFER_SIZE = 256;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    volatile size_t rx_head_;
    volatile size_t rx_tail_;
    
    static HAL_SPI0* instance_;
    
    // 私有构造函数（单例模式）
    HAL_SPI0();
    HAL_SPI0(const HAL_SPI0&) = delete;
    HAL_SPI0& operator=(const HAL_SPI0&) = delete;
};

// SPI1实例
class HAL_SPI1 : public HAL_SPI {
public:
    static HAL_SPI1* getInstance();
    ~HAL_SPI1();
    
    bool init(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t frequency = 1000000) override;
    void deinit() override;
    bool write_dma(const uint8_t* data, size_t length, dma_callback_t callback = nullptr) override;
    bool read_dma(uint8_t* buffer, size_t length, dma_callback_t callback = nullptr) override;
    bool transfer_dma(const uint8_t* tx_data, uint8_t* rx_data, size_t length, dma_callback_t callback = nullptr) override;
    inline size_t write_to_tx_buffer(const uint8_t* data, size_t length) override;
    inline size_t read_from_rx_buffer(uint8_t* buffer, size_t length) override;
    inline void trigger_tx_dma() override;
    inline size_t get_tx_buffer_free_space() const override;
    inline size_t get_rx_buffer_data_count() const override;
    bool is_busy() const override;
    void set_cs_pin(uint8_t cs_pin, bool active_low = true) override;
    void cs_select() override;
    void cs_deselect() override;
    void set_format(uint8_t data_bits, uint8_t cpol, uint8_t cpha) override;
    void set_frequency(uint32_t frequency) override;
    std::string get_name() const override;
    bool is_ready() const override;

private:
    bool initialized_;
    uint8_t sck_pin_;
    uint8_t mosi_pin_;
    uint8_t miso_pin_;
    uint8_t cs_pin_;
    bool cs_active_low_;
    uint32_t frequency_;
    bool dma_busy_;
    dma_callback_t dma_callback_;
    int dma_tx_channel_;
    int dma_rx_channel_;
    
    static HAL_SPI1* instance_;
    
    // 私有构造函数（单例模式）
    HAL_SPI1();
    HAL_SPI1(const HAL_SPI1&) = delete;
    HAL_SPI1& operator=(const HAL_SPI1&) = delete;
};