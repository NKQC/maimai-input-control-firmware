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
      dma_status_(DMA_Status::IDLE), dma_tx_channel_(-1), dma_rx_channel_(-1),
      read_cmd_(I2C_IC_DATA_CMD_CMD_BITS),
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
    
    if (dma_tx_channel_ < 0 || dma_rx_channel_ < 0) {
        return false;
    }
    
    // 注册DMA回调到global_irq
    global_irq_register_dma_callback(dma_tx_channel_, tx_dma_callback_);
    global_irq_register_dma_callback(dma_rx_channel_, rx_dma_callback_);
    
    // 启用I2C中断
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    hw->intr_mask = I2C_IC_INTR_MASK_M_STOP_DET_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS;
    
    // 设置I2C中断处理器
    uint irq_num = (i2c_instance_ == i2c0) ? I2C0_IRQ : I2C1_IRQ;
    if (i2c_instance_ == i2c0) {
        irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);
    } else {
        irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    }
    irq_set_enabled(irq_num, true);
    
    dma_status_ = DMA_Status::IDLE;
    initialized_ = true;
    
    return true;
}

void HAL_I2C::deinit() {
    if (initialized_) {
        // 停止任何正在进行的DMA传输
        if (dma_status_ != DMA_Status::IDLE) {
            if (dma_tx_channel_ >= 0) {
                dma_channel_abort(dma_tx_channel_);
            }
            if (dma_rx_channel_ >= 0) {
                dma_channel_abort(dma_rx_channel_);
            }
            dma_status_ = DMA_Status::IDLE;
        }
        
        // 禁用I2C中断
        uint irq_num = (i2c_instance_ == i2c0) ? I2C0_IRQ : I2C1_IRQ;
        irq_set_enabled(irq_num, false);
        
        // 释放DMA通道
        if (dma_tx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_tx_channel_);
            dma_channel_unclaim(dma_tx_channel_);
            dma_tx_channel_ = -1;
        }
        if (dma_rx_channel_ >= 0) {
            global_irq_unregister_dma_callback(dma_rx_channel_);
            dma_channel_unclaim(dma_rx_channel_);
            dma_rx_channel_ = -1;
        }
        
        // 反初始化I2C
        i2c_deinit(i2c_instance_);
        
        initialized_ = false;
    }
}

bool HAL_I2C::write(uint8_t address, const uint8_t* data, size_t length) {
    if (!initialized_) return false;
    
    int32_t result = i2c_write_blocking(i2c_instance_, address, data, length, false);
    return result == (int)length;
}

bool HAL_I2C::read(uint8_t address, uint8_t* buffer, size_t length) {
    if (!initialized_) return false;
    
    int32_t result = i2c_read_blocking(i2c_instance_, address, buffer, length, false);
    return result == (int)length;
}

int32_t HAL_I2C::write_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length) {
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

int32_t HAL_I2C::read_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length) {
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
    int32_t result = i2c_read_blocking(i2c_instance_, address, &dummy, 1, false);
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
    if (!initialized_ || dma_status_ != DMA_Status::IDLE || !buffer || length == 0) {
        return false;
    }
    
    // 设置DMA上下文
    dma_context_.device_addr = address;
    dma_context_.buffer = buffer;
    dma_context_.length = length;
    dma_context_.is_write = false;
    dma_context_.callback = callback;
    
    return _setup_dma_read(address, buffer, length);
}

bool HAL_I2C::write_async(uint8_t address, const uint8_t* data, size_t length, dma_callback_t callback) {
    if (!initialized_ || dma_status_ != DMA_Status::IDLE || !data || length == 0) {
        return false;
    }
    
    // 设置DMA上下文
    dma_context_.device_addr = address;
    dma_context_.buffer = const_cast<uint8_t*>(data);
    dma_context_.length = length;
    dma_context_.is_write = true;
    dma_context_.callback = callback;
    
    return _setup_dma_write(address, data, length);
}

bool HAL_I2C::is_busy() const {
    return dma_status_ != DMA_Status::IDLE;
}

// DMA写入设置
bool HAL_I2C::_setup_dma_write(uint8_t address, const uint8_t* data, size_t length) {
    dma_status_ = DMA_Status::TX_BUSY;
    
    // 配置DMA通道
    dma_channel_config c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, i2c_get_dreq(i2c_instance_, true)); // TX DREQ
    
    // 设置I2C地址
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    hw->tar = address;
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &hw->data_cmd,
        data,
        length,
        true // 立即启动
    );
    
    return true;
}

// DMA读取设置  
bool HAL_I2C::_setup_dma_read(uint8_t address, uint8_t* buffer, size_t length) {
    dma_status_ = DMA_Status::RX_BUSY;
    
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    
    // 配置RX DMA通道
    dma_channel_config rx_c = dma_channel_get_default_config(dma_rx_channel_);
    channel_config_set_transfer_data_size(&rx_c, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_c, false);
    channel_config_set_write_increment(&rx_c, true);
    channel_config_set_dreq(&rx_c, i2c_get_dreq(i2c_instance_, false)); // RX DREQ
    
    // 配置TX DMA通道用于发送读命令
    dma_channel_config tx_c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&tx_c, DMA_SIZE_16);
    channel_config_set_read_increment(&tx_c, false);
    channel_config_set_write_increment(&tx_c, false);
    channel_config_set_dreq(&tx_c, i2c_get_dreq(i2c_instance_, true)); // TX DREQ
    
    // 设置I2C地址
    hw->tar = address;
    
    // 启动RX DMA
    dma_channel_configure(
        dma_rx_channel_,
        &rx_c,
        buffer,
        &hw->data_cmd,
        length,
        false // 先不启动
    );
    
    // 准备读命令 - 使用类成员变量而非静态变量
    // read_cmd_ 在构造函数中已初始化为 I2C_IC_DATA_CMD_CMD_BITS
    
    // 启动TX DMA发送读命令
    dma_channel_configure(
        dma_tx_channel_,
        &tx_c,
        &hw->data_cmd,
        &read_cmd_,
        length,
        false // 先不启动
    );
    
    // 先启动RX，再启动TX
    dma_channel_start(dma_rx_channel_);
    dma_channel_start(dma_tx_channel_);
    
    return true;
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

// DMA传输完成处理
void HAL_I2C::_dma_tx_complete(bool success) {
    if (dma_context_.is_write) {
        // 写操作完成
        dma_status_ = success ? DMA_Status::IDLE : DMA_Status::ERROR;
        
        if (dma_context_.callback) {
            dma_context_.callback(success);
        }
    } else {
        // 读操作的TX完成，等待RX完成
        // TX完成不需要特殊处理，等待RX完成
    }
}

void HAL_I2C::_dma_rx_complete(bool success) {
    // 读操作完成
    dma_status_ = success ? DMA_Status::IDLE : DMA_Status::ERROR;
    
    if (dma_context_.callback) {
        dma_context_.callback(success);
    }
}

// I2C中断处理
void HAL_I2C::_handle_i2c_irq() {
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    uint32_t intr_stat = hw->intr_stat;
    
    // 检查传输中止
    if (intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        // 清除中断
        hw->clr_tx_abrt;
        
        // 停止DMA传输
        if (dma_status_ == DMA_Status::TX_BUSY) {
            dma_channel_abort(dma_tx_channel_);
            _dma_tx_complete(false);
        } else if (dma_status_ == DMA_Status::RX_BUSY) {
            dma_channel_abort(dma_rx_channel_);
            dma_channel_abort(dma_tx_channel_);
            _dma_rx_complete(false);
        }
    }
    
    // 检查停止检测
    if (intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        // 清除中断
        hw->clr_stop_det;
        
        // 传输正常完成
        if (dma_status_ == DMA_Status::TX_BUSY && dma_context_.is_write) {
            if (!dma_channel_is_busy(dma_tx_channel_)) {
                _dma_tx_complete(true);
            }
        } else if (dma_status_ == DMA_Status::RX_BUSY && !dma_context_.is_write) {
            if (!dma_channel_is_busy(dma_rx_channel_)) {
                _dma_rx_complete(true);
            }
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

// C风格DMA回调函数实现
void i2c0_tx_dma_callback(bool success) {
    HAL_I2C0* instance = HAL_I2C0::getInstance();
    if (instance) {
        instance->_dma_tx_complete(success);
    }
}

void i2c0_rx_dma_callback(bool success) {
    HAL_I2C0* instance = HAL_I2C0::getInstance();
    if (instance) {
        instance->_dma_rx_complete(success);
    }
}

void i2c1_tx_dma_callback(bool success) {
    HAL_I2C1* instance = HAL_I2C1::getInstance();
    if (instance) {
        instance->_dma_tx_complete(success);
    }
}

void i2c1_rx_dma_callback(bool success) {
    HAL_I2C1* instance = HAL_I2C1::getInstance();
    if (instance) {
        instance->_dma_rx_complete(success);
    }
}

// I2C中断处理器实现
void i2c0_irq_handler() {
    HAL_I2C0* instance = HAL_I2C0::getInstance();
    if (instance) {
        instance->_handle_i2c_irq();
    }
}

void i2c1_irq_handler() {
    HAL_I2C1* instance = HAL_I2C1::getInstance();
    if (instance) {
        instance->_handle_i2c_irq();
    }
}