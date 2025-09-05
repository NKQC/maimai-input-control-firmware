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
void i2c0_tx_dma_callback(bool success);
void i2c0_rx_dma_callback(bool success);
void i2c1_tx_dma_callback(bool success);
void i2c1_rx_dma_callback(bool success);

// HAL_I2C 基类实现
HAL_I2C::HAL_I2C(i2c_inst_t* i2c_instance, void (*tx_callback)(bool), void (*rx_callback)(bool))
    : i2c_instance_(i2c_instance), initialized_(false), sda_pin_(0), scl_pin_(0), 
      dma_busy_(false), dma_tx_channel_(-1), dma_rx_channel_(-1),
      tx_dma_callback_(tx_callback), rx_dma_callback_(rx_callback) {
}

bool HAL_I2C::init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency) {
    if (initialized_) {
        deinit();
    }
    
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
    
    // 初始化I2C
    i2c_init(i2c_instance_, frequency);
    
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // 分配DMA通道
    dma_tx_channel_ = dma_claim_unused_channel(true);
    dma_rx_channel_ = dma_claim_unused_channel(true);
    
    // 注册DMA回调到global_irq
    global_irq_register_dma_callback(dma_tx_channel_, tx_dma_callback_);
    global_irq_register_dma_callback(dma_rx_channel_, rx_dma_callback_);
    
    initialized_ = true;
    return true;
}

void HAL_I2C::deinit() {
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
        
        i2c_deinit(i2c_instance_);
        initialized_ = false;
    }
}

bool HAL_I2C::write(uint8_t address, const uint8_t* data, size_t length) {
    if (!initialized_) return false;
    
    int result = i2c_write_blocking(i2c_instance_, address, data, length, false);
    return result == (int)length;
}

bool HAL_I2C::read(uint8_t address, uint8_t* buffer, size_t length) {
    if (!initialized_) return false;
    
    int result = i2c_read_blocking(i2c_instance_, address, buffer, length, false);
    return result == (int)length;
}

int HAL_I2C::write_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length) {
    if (!initialized_) return false;
    uint8_t reg_size = reg & 0xFF00 ? 2 : reg & 0x8000 ? 2 : 1;
    uint8_t data[length + reg_size];
    data[0] = reg & 0xFF;
    if (reg_size == 2){
        data[0] = (reg >> 8) & 0x7F;
        data[1] = reg & 0xFF;
    }
    memcpy(data + reg_size, value, length);
    return i2c_write_blocking(i2c_instance_, address, data, length + reg_size, false) - reg_size;
}

int HAL_I2C::read_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length) {
    if (!initialized_) return false;
    uint8_t reg_size = reg & 0xFF00 ? 2 : reg & 0x8000 ? 2 : 1;
    uint8_t data[2];
    data[0] = (uint8_t)(reg & 0xFF);
    if (reg_size == 2){
        data[0] = (reg >> 8) & 0x7F;
        data[1] = reg & 0xFF;
    }
    i2c_write_blocking(i2c_instance_, address, data, reg_size, true);
    return i2c_read_blocking(i2c_instance_, address, value, length, false);
}

bool HAL_I2C::device_exists(uint8_t address) {
    if (!initialized_) return false;
    
    uint8_t dummy;
    int result = i2c_read_blocking(i2c_instance_, address, &dummy, 1, false);
    return result >= 0;
}

std::vector<uint8_t> HAL_I2C::scan_devices() {
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

bool HAL_I2C::read_async(uint8_t address, uint8_t* buffer, size_t length, dma_callback_t callback) {
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

bool HAL_I2C::write_async(uint8_t address, const uint8_t* data, size_t length, dma_callback_t callback) {
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

bool HAL_I2C::is_busy() const {
    return dma_busy_;
}

// HAL_I2C0 静态成员初始化
HAL_I2C0* HAL_I2C0::instance_ = nullptr;

// HAL_I2C0 实现
HAL_I2C0* HAL_I2C0::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_I2C0();
    }
    return instance_;
}

HAL_I2C0::HAL_I2C0() : HAL_I2C(i2c0, i2c0_tx_dma_callback, i2c0_rx_dma_callback) {}

HAL_I2C0::~HAL_I2C0() {
    deinit();
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

// HAL_I2C1 静态成员初始化
HAL_I2C1* HAL_I2C1::instance_ = nullptr;

// HAL_I2C1 实现
HAL_I2C1* HAL_I2C1::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_I2C1();
    }
    return instance_;
}

HAL_I2C1::HAL_I2C1() : HAL_I2C(i2c1, i2c1_tx_dma_callback, i2c1_rx_dma_callback) {}

HAL_I2C1::~HAL_I2C1() {
    deinit();
}