#include "hal_i2c.h"
#include <Wire.h>
#include <hardware/i2c.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <vector>

extern "C" {
#include "../global_irq.h"
}

// C风格DMA回调函数声明
// I2C DMA回调函数声明
void i2c0_tx_dma_callback(bool success);
void i2c0_rx_dma_callback(bool success);
void i2c1_tx_dma_callback(bool success);
void i2c1_rx_dma_callback(bool success);

// HAL_I2C0 静态成员初始化
HAL_I2C0* HAL_I2C0::instance_ = nullptr;

// HAL_I2C0 实现
HAL_I2C0* HAL_I2C0::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_I2C0();
    }
    return instance_;
}

HAL_I2C0::HAL_I2C0() : initialized_(false), sda_pin_(0), scl_pin_(0), dma_busy_(false), dma_tx_channel_(-1), dma_rx_channel_(-1) {}

HAL_I2C0::~HAL_I2C0() {
    deinit();
}

bool HAL_I2C0::init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency) {
    if (initialized_) {
        deinit();
    }
    
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
    
    // 初始化I2C0
    i2c_init(i2c0, frequency);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // 分配DMA通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_rx_channel_ = dma_claim_unused_channel(true);
    
    // 注册DMA回调到global_irq
    global_irq_register_dma_callback(dma_tx_channel_, i2c0_tx_dma_callback);
    global_irq_register_dma_callback(dma_rx_channel_, i2c0_rx_dma_callback);
    
    initialized_ = true;
    return true;
}

void HAL_I2C0::deinit() {
    if (initialized_) {
        // 注销DMA回调
        if (dma_tx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_tx_channel_);
        }
        if (dma_rx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_rx_channel_);
        }
        
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_rx_channel_ >= 0) {
            dma_channel_unclaim(dma_rx_channel_);
            dma_rx_channel_ = -1;
        }
        
        i2c_deinit(i2c0);
        initialized_ = false;
    }
}

bool HAL_I2C0::write(uint8_t address, const uint8_t* data, size_t length) {
    if (!initialized_) return false;
    
    int result = i2c_write_blocking(i2c0, address, data, length, false);
    return result == length;
}

bool HAL_I2C0::read(uint8_t address, uint8_t* buffer, size_t length) {
    if (!initialized_) return false;
    
    int result = i2c_read_blocking(i2c0, address, buffer, length, false);
    return result == length;
}

bool HAL_I2C0::write_register(uint8_t address, uint8_t reg, uint8_t value) {
    if (!initialized_) return false;
    
    uint8_t data[2] = {reg, value};
    return write(address, data, 2);
}

bool HAL_I2C0::read_register(uint8_t address, uint8_t reg, uint8_t* value) {
    if (!initialized_) return false;
    
    // 先写寄存器地址
    if (!write(address, &reg, 1)) {
        return false;
    }
    
    // 再读取数据
    return read(address, value, 1);
}

bool HAL_I2C0::device_exists(uint8_t address) {
    if (!initialized_) return false;
    
    uint8_t dummy;
    int result = i2c_read_blocking(i2c0, address, &dummy, 1, false);
    return result >= 0;
}

std::vector<uint8_t> HAL_I2C0::scan_devices() {
    std::vector<uint8_t> found_devices;
    
    if (!initialized_) return found_devices;
    
    // 扫描I2C地址范围 0x08-0x77
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (device_exists(addr)) {
            found_devices.push_back(addr);
        }
    }
    
    return found_devices;
}

std::string HAL_I2C1::get_name() const {
    return "I2C1";
}

// I2C DMA回调函数实现
void i2c0_tx_dma_callback(bool success) {
        HAL_I2C0* instance = HAL_I2C0::getInstance();
        if (instance != nullptr) {
            instance->dma_busy_ = false;
            if (instance->dma_callback_ != nullptr) {
                instance->dma_callback_(success);
            }
        }
    }
    
void i2c0_rx_dma_callback(bool success) {
        HAL_I2C0* instance = HAL_I2C0::getInstance();
        if (instance != nullptr) {
            instance->dma_busy_ = false;
            if (instance->dma_callback_ != nullptr) {
                instance->dma_callback_(success);
            }
        }
    }
    
void i2c1_tx_dma_callback(bool success) {
        HAL_I2C1* instance = HAL_I2C1::getInstance();
        if (instance != nullptr) {
            instance->dma_busy_ = false;
            if (instance->dma_callback_ != nullptr) {
                instance->dma_callback_(success);
            }
        }
    }
    
void i2c1_rx_dma_callback(bool success) {
        HAL_I2C1* instance = HAL_I2C1::getInstance();
        if (instance != nullptr) {
            instance->dma_busy_ = false;
            if (instance->dma_callback_ != nullptr) {
                instance->dma_callback_(success);
            }
        }
    }

bool HAL_I2C1::read_async(uint8_t address, uint8_t* buffer, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_) return false;
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 对于简单实现，先使用阻塞方式，后续可优化为真正的DMA
    bool result = read(address, buffer, length);
    
    dma_busy_ = false;
    if (dma_callback_) {
        dma_callback_(result);
    }
    
    return result;
}

bool HAL_I2C1::write_async(uint8_t address, const uint8_t* data, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_) return false;
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 对于简单实现，先使用阻塞方式，后续可优化为真正的DMA
    bool result = write(address, data, length);
    
    dma_busy_ = false;
    if (dma_callback_) {
        dma_callback_(result);
    }
    
    return result;
}

bool HAL_I2C1::is_busy() const {
    return dma_busy_;
}

bool HAL_I2C0::read_async(uint8_t address, uint8_t* buffer, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_) return false;
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 对于简单实现，先使用阻塞方式，后续可优化为真正的DMA
    bool result = read(address, buffer, length);
    
    dma_busy_ = false;
    if (dma_callback_) {
        dma_callback_(result);
    }
    
    return result;
}

bool HAL_I2C0::write_async(uint8_t address, const uint8_t* data, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_busy_) return false;
    
    dma_busy_ = true;
    dma_callback_ = callback;
    
    // 对于简单实现，先使用阻塞方式，后续可优化为真正的DMA
    bool result = write(address, data, length);
    
    dma_busy_ = false;
    if (dma_callback_) {
        dma_callback_(result);
    }
    
    return result;
}

bool HAL_I2C0::is_busy() const {
    return dma_busy_;
}

// HAL_I2C1 静态成员初始化
HAL_I2C1* HAL_I2C1::instance_ = nullptr;

// HAL_I2C1 实现
HAL_I2C1* HAL_I2C1::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_I2C1();
    }
    return instance_;
}

HAL_I2C1::HAL_I2C1() : initialized_(false), sda_pin_(0), scl_pin_(0), dma_busy_(false), dma_tx_channel_(-1), dma_rx_channel_(-1) {}

HAL_I2C1::~HAL_I2C1() {
    deinit();
}

bool HAL_I2C1::init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency) {
    if (initialized_) {
        deinit();
    }
    
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
    
    // 初始化I2C1
    i2c_init(i2c1, frequency);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // 分配DMA通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_rx_channel_ = dma_claim_unused_channel(true);
    
    // 注册DMA回调到global_irq
    global_irq_register_dma_callback(dma_tx_channel_, i2c1_tx_dma_callback);
    global_irq_register_dma_callback(dma_rx_channel_, i2c1_rx_dma_callback);
    
    initialized_ = true;
    return true;
}

void HAL_I2C1::deinit() {
    if (initialized_) {
        // 注销DMA回调
        if (dma_tx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_tx_channel_);
        }
        if (dma_rx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_rx_channel_);
        }
        
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_rx_channel_ >= 0) {
            dma_channel_unclaim(dma_rx_channel_);
            dma_rx_channel_ = -1;
        }
        
        i2c_deinit(i2c1);
        initialized_ = false;
    }
}

bool HAL_I2C1::write(uint8_t address, const uint8_t* data, size_t length) {
    if (!initialized_) return false;
    
    int result = i2c_write_blocking(i2c1, address, data, length, false);
    return result == length;
}

bool HAL_I2C1::read(uint8_t address, uint8_t* buffer, size_t length) {
    if (!initialized_) return false;
    
    int result = i2c_read_blocking(i2c1, address, buffer, length, false);
    return result == length;
}

bool HAL_I2C1::write_register(uint8_t address, uint8_t reg, uint8_t value) {
    if (!initialized_) return false;
    
    uint8_t data[2] = {reg, value};
    return write(address, data, 2);
}

bool HAL_I2C1::read_register(uint8_t address, uint8_t reg, uint8_t* value) {
    if (!initialized_) return false;
    
    // 先写寄存器地址
    if (!write(address, &reg, 1)) {
        return false;
    }
    
    // 再读取数据
    return read(address, value, 1);
}

bool HAL_I2C1::device_exists(uint8_t address) {
    if (!initialized_) return false;
    
    uint8_t dummy;
    int result = i2c_read_blocking(i2c1, address, &dummy, 1, false);
    return result >= 0;
}

std::vector<uint8_t> HAL_I2C1::scan_devices() {
    std::vector<uint8_t> found_devices;
    
    if (!initialized_) return found_devices;
    
    // 扫描I2C地址范围 0x08-0x77
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (device_exists(addr)) {
            found_devices.push_back(addr);
        }
    }
    
    return found_devices;
}