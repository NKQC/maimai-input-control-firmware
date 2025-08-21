#include "hal_spi.h"
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>

// HAL_SPI0 静态成员初始化
HAL_SPI0* HAL_SPI0::instance_ = nullptr;

// HAL_SPI0 实现
HAL_SPI0* HAL_SPI0::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_SPI0();
    }
    return instance_;
}

HAL_SPI0::HAL_SPI0() 
    : initialized_(false), sck_pin_(0), mosi_pin_(0), miso_pin_(0), 
      cs_pin_(0), cs_active_low_(true), frequency_(1000000), dma_busy_(false),
      dma_tx_channel_(-1), dma_rx_channel_(-1) {}

HAL_SPI0::~HAL_SPI0() {
    deinit();
}

bool HAL_SPI0::init(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t frequency) {
    if (initialized_) {
        deinit();
    }
    
    sck_pin_ = sck_pin;
    mosi_pin_ = mosi_pin;
    miso_pin_ = miso_pin;
    frequency_ = frequency;
    
    // 初始化SPI0
    spi_init(spi0, frequency);
    
    // 设置GPIO功能
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    
    // 设置默认SPI格式 (8位, CPOL=0, CPHA=0)
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    // 分配DMA通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_rx_channel_ = dma_claim_unused_channel(true);
    
    initialized_ = true;
    return true;
}

void HAL_SPI0::deinit() {
    if (initialized_) {
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_rx_channel_ >= 0) {
            dma_channel_unclaim(dma_rx_channel_);
            dma_rx_channel_ = -1;
        }
        
        spi_deinit(spi0);
        initialized_ = false;
    }
}

size_t HAL_SPI0::write(const uint8_t* data, size_t length) {
    if (!initialized_) return 0;
    
    return spi_write_blocking(spi0, data, length);
}

size_t HAL_SPI0::read(uint8_t* buffer, size_t length) {
    if (!initialized_) return 0;
    
    // SPI读取需要发送dummy数据
    uint8_t dummy = 0xFF;
    return spi_read_blocking(spi0, dummy, buffer, length);
}

size_t HAL_SPI0::transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) {
    if (!initialized_) return 0;
    
    return spi_write_read_blocking(spi0, tx_data, rx_data, length);
}

void HAL_SPI0::set_cs_pin(uint8_t cs_pin, bool active_low) {
    cs_pin_ = cs_pin;
    cs_active_low_ = active_low;
    
    // 初始化CS引脚
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    
    // 设置为非激活状态
    cs_deselect();
}

void HAL_SPI0::cs_select() {
    if (cs_pin_ != 0) {
        gpio_put(cs_pin_, cs_active_low_ ? 0 : 1);
    }
}

void HAL_SPI0::cs_deselect() {
    if (cs_pin_ != 0) {
        gpio_put(cs_pin_, cs_active_low_ ? 1 : 0);
    }
}

void HAL_SPI0::set_format(uint8_t data_bits, uint8_t cpol, uint8_t cpha) {
    if (initialized_) {
        spi_cpol_t spi_cpol = (cpol == 0) ? SPI_CPOL_0 : SPI_CPOL_1;
        spi_cpha_t spi_cpha = (cpha == 0) ? SPI_CPHA_0 : SPI_CPHA_1;
        spi_set_format(spi0, data_bits, spi_cpol, spi_cpha, SPI_MSB_FIRST);
    }
}

void HAL_SPI0::set_frequency(uint32_t frequency) {
    if (initialized_) {
        frequency_ = frequency;
        spi_set_baudrate(spi0, frequency);
    }
}

bool HAL_SPI0::write_async(const uint8_t* data, size_t length, dma_callback_t callback) {
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
    channel_config_set_dreq(&c, spi_get_dreq(spi0, true)); // TX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_tx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_spi0_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &spi_get_hw(spi0)->dr, // 写入地址
        data,                  // 读取地址
        length,                // 传输长度
        true                   // 立即启动
    );
    
    return true;
}

bool HAL_SPI0::read_async(uint8_t* buffer, size_t length, dma_callback_t callback) {
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
    channel_config_set_dreq(&c, spi_get_dreq(spi0, false)); // RX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_rx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_spi0_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_rx_channel_,
        &c,
        buffer,                // 写入地址
        &spi_get_hw(spi0)->dr, // 读取地址
        length,                // 传输长度
        true                   // 立即启动
    );
    
    return true;
}

bool HAL_SPI0::transfer_async(const uint8_t* tx_data, uint8_t* rx_data, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_) return false;
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 对于简单实现，先使用阻塞方式，后续可优化为真正的DMA
    size_t result = transfer(tx_data, rx_data, length);
    
    dma_busy_ = false;
    if (dma_callback_) {
        dma_callback_(result == length);
    }
    
    return result == length;
}

// DMA中断处理函数
static void dma_spi0_complete() {
    HAL_SPI0* instance = HAL_SPI0::getInstance();
    instance->dma_busy_ = false;
    if (instance->dma_callback_) {
        instance->dma_callback_(true);
    }
}

bool HAL_SPI0::is_busy() const {
    return dma_busy_;
}

std::string HAL_SPI0::get_name() const {
    return "SPI0";
}

bool HAL_SPI0::is_ready() const {
    return initialized_;
}

// HAL_SPI1 静态成员初始化
HAL_SPI1* HAL_SPI1::instance_ = nullptr;

// HAL_SPI1 实现
HAL_SPI1* HAL_SPI1::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_SPI1();
    }
    return instance_;
}

HAL_SPI1::HAL_SPI1() 
    : initialized_(false), sck_pin_(0), mosi_pin_(0), miso_pin_(0), 
      cs_pin_(0), cs_active_low_(true), frequency_(1000000), dma_busy_(false),
      dma_tx_channel_(-1), dma_rx_channel_(-1) {}

HAL_SPI1::~HAL_SPI1() {
    deinit();
}

bool HAL_SPI1::init(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint32_t frequency) {
    if (initialized_) {
        deinit();
    }
    
    sck_pin_ = sck_pin;
    mosi_pin_ = mosi_pin;
    miso_pin_ = miso_pin;
    frequency_ = frequency;
    
    // 初始化SPI1
    spi_init(spi1, frequency);
    
    // 设置GPIO功能
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    
    // 设置默认SPI格式 (8位, CPOL=0, CPHA=0)
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    // 分配DMA通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_rx_channel_ = dma_claim_unused_channel(true);
    
    initialized_ = true;
    return true;
}

void HAL_SPI1::deinit() {
    if (initialized_) {
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_rx_channel_ >= 0) {
            dma_channel_unclaim(dma_rx_channel_);
            dma_rx_channel_ = -1;
        }
        
        spi_deinit(spi1);
        initialized_ = false;
    }
}

size_t HAL_SPI1::write(const uint8_t* data, size_t length) {
    if (!initialized_) return 0;
    
    return spi_write_blocking(spi1, data, length);
}

size_t HAL_SPI1::read(uint8_t* buffer, size_t length) {
    if (!initialized_) return 0;
    
    // SPI读取需要发送dummy数据
    uint8_t dummy = 0xFF;
    return spi_read_blocking(spi1, dummy, buffer, length);
}

size_t HAL_SPI1::transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) {
    if (!initialized_) return 0;
    
    return spi_write_read_blocking(spi1, tx_data, rx_data, length);
}

void HAL_SPI1::set_cs_pin(uint8_t cs_pin, bool active_low) {
    cs_pin_ = cs_pin;
    cs_active_low_ = active_low;
    
    // 初始化CS引脚
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    
    // 设置为非激活状态
    cs_deselect();
}

void HAL_SPI1::cs_select() {
    if (cs_pin_ != 0) {
        gpio_put(cs_pin_, cs_active_low_ ? 0 : 1);
    }
}

void HAL_SPI1::cs_deselect() {
    if (cs_pin_ != 0) {
        gpio_put(cs_pin_, cs_active_low_ ? 1 : 0);
    }
}

void HAL_SPI1::set_format(uint8_t data_bits, uint8_t cpol, uint8_t cpha) {
    if (initialized_) {
        spi_cpol_t spi_cpol = (cpol == 0) ? SPI_CPOL_0 : SPI_CPOL_1;
        spi_cpha_t spi_cpha = (cpha == 0) ? SPI_CPHA_0 : SPI_CPHA_1;
        spi_set_format(spi1, data_bits, spi_cpol, spi_cpha, SPI_MSB_FIRST);
    }
}

void HAL_SPI1::set_frequency(uint32_t frequency) {
    if (initialized_) {
        frequency_ = frequency;
        spi_set_baudrate(spi1, frequency);
    }
}

// DMA中断处理函数
static void dma_spi1_complete() {
    HAL_SPI1* instance = HAL_SPI1::getInstance();
    instance->dma_busy_ = false;
    if (instance->dma_callback_) {
        instance->dma_callback_(true);
    }
}

bool HAL_SPI1::write_async(const uint8_t* data, size_t length, dma_callback_t callback) {
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
    channel_config_set_dreq(&c, spi_get_dreq(spi1, true)); // TX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_tx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_spi1_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &spi_get_hw(spi1)->dr, // 写入地址
        data,                  // 读取地址
        length,                // 传输长度
        true                   // 立即启动
    );
    
    return true;
}

bool HAL_SPI1::read_async(uint8_t* buffer, size_t length, dma_callback_t callback) {
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
    channel_config_set_dreq(&c, spi_get_dreq(spi1, false)); // RX DREQ
    
    // 设置中断
    dma_channel_set_irq0_enabled(dma_rx_channel_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_spi1_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 启动DMA传输
    dma_channel_configure(
        dma_rx_channel_,
        &c,
        buffer,                // 写入地址
        &spi_get_hw(spi1)->dr, // 读取地址
        length,                // 传输长度
        true                   // 立即启动
    );
    
    return true;
}

bool HAL_SPI1::transfer_async(const uint8_t* tx_data, uint8_t* rx_data, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_) return false;
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 对于简单实现，先使用阻塞方式，后续可优化为真正的DMA
    size_t result = transfer(tx_data, rx_data, length);
    
    dma_busy_ = false;
    if (dma_callback_) {
        dma_callback_(result == length);
    }
    
    return result == length;
}

bool HAL_SPI1::is_busy() const {
    return dma_busy_;
}

std::string HAL_SPI1::get_name() const {
    return "SPI1";
}

bool HAL_SPI1::is_ready() const {
    return initialized_;
}