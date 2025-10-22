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

// HAL_I2C 基类实现
HAL_I2C::HAL_I2C(i2c_inst_t* i2c_instance)
    : i2c_instance_(i2c_instance), initialized_(false), sda_pin_(0), scl_pin_(0), 
      dma_status_(DMA_Status::IDLE), dma_tx_channel_(-1), dma_rx_channel_(-1),
      interrupts_enabled_(false), read_cmd_(I2C_IC_DATA_CMD_CMD_BITS) {
    // 初始化DMA上下文
    dma_context_ = DMA_Context();
    // 初始化命令缓冲区
    memset(data_cmds_, 0, sizeof(data_cmds_));
}

bool HAL_I2C::init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency) {
    if (initialized_) {
        return true;
    }
    
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
    
    // 在初始化I2C之前尝试解除总线锁定
    HAL_I2C::unlock_bus(sda_pin_, scl_pin_);
    
    // 初始化I2C
    i2c_init(i2c_instance_, frequency);
    
    // 设置GPIO功能
    gpio_set_function(sda_pin_, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin_, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin_);
    gpio_pull_up(scl_pin_);
    
    // 申请DMA通道但不启动中断
    dma_tx_channel_ = dma_claim_unused_channel(false);
    dma_rx_channel_ = dma_claim_unused_channel(false);
    if (dma_tx_channel_ < 0 || dma_rx_channel_ < 0) {
        return false;
    }
    
    // 设置I2C中断处理器但不启用中断
    if (i2c_instance_ == i2c0) {
        irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);
    } else {
        irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    }
    
    // 初始化状态变量
    dma_status_ = DMA_Status::IDLE;
    interrupts_enabled_ = false;  // 初始化时中断未启用
    initialized_ = true;
    
    return true;
}

void HAL_I2C::deinit() {
    if (!initialized_) return;
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
        dma_channel_unclaim(dma_tx_channel_);
        dma_tx_channel_ = -1;
    }
    if (dma_rx_channel_ >= 0) {
        dma_channel_unclaim(dma_rx_channel_);
        dma_rx_channel_ = -1;
    }
    
    // 反初始化I2C
    i2c_deinit(i2c_instance_);
    
    initialized_ = false;
}

bool HAL_I2C::write(uint8_t address, const uint8_t* data, size_t length) {
    if (!initialized_) return false;
    
    // 等待总线空闲，最长10ms超时
    if (!_wait_for_bus_idle(10)) {
        return false;  // 总线忙或超时
    }
    
    // 确保中断已关闭（惰性管理）
    if (interrupts_enabled_) {
        _disable_i2c_interrupts();
    }
    
    int32_t result = i2c_write_blocking(i2c_instance_, address, data, length, false);
    return result == (int)length;
}

bool HAL_I2C::read(uint8_t address, uint8_t* buffer, size_t length) {
    if (!initialized_) return false;
    
    // 等待总线空闲，最长10ms超时
    if (!_wait_for_bus_idle(10)) {
        return false;  // 总线忙或超时
    }
    
    // 确保中断已关闭（惰性管理）
    if (interrupts_enabled_) {
        _disable_i2c_interrupts();
    }
    
    int32_t result = i2c_read_blocking(i2c_instance_, address, buffer, length, false);
    return result == (int)length;
}

int32_t HAL_I2C::write_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length) {
    if (!initialized_) return -1;
    
    // 等待总线空闲，最长10ms超时
    if (!_wait_for_bus_idle(10)) {
        return -1;  // 总线忙或超时
    }
    
    // 确保中断已关闭（惰性管理）
    if (interrupts_enabled_) {
        _disable_i2c_interrupts();
    }
    
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
    if (!initialized_) return -1;
    
    // 等待总线空闲，最长10ms超时
    if (!_wait_for_bus_idle(10)) {
        return -1;  // 总线忙或超时
    }
    
    // 确保中断已关闭（惰性管理）
    if (interrupts_enabled_) {
        _disable_i2c_interrupts();
    }
    
    uint8_t reg_size = reg & 0xFF00 ? 2 : reg & 0x8000 ? 2 : 1;
    uint8_t data[2];
    data[0] = (uint8_t)(reg & 0xFF);
    if (reg_size == 2){
        data[0] = (reg >> 8) & 0x7F;
        data[1] = reg & 0xFF;
    }
    // 子地址写入后发送 STOP，确保 EZI2C 指针锁定
    i2c_write_blocking(i2c_instance_, address, data, reg_size, false);
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

// 新的易用异步接口实现
bool HAL_I2C::read_register_async(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length, dma_callback_t callback) {
    if (!initialized_ || dma_status_ != DMA_Status::IDLE || !value || length == 0 || !callback) {
        return false;
    }
    
    // 计算寄存器地址大小
    uint8_t reg_size = (reg & 0xFF00) ? 2 : (reg & 0x8000) ? 2 : 1;
    
    // 设置DMA上下文
    dma_context_.device_addr = address;
    dma_context_.buffer = value;
    dma_context_.length = length;
    dma_context_.is_write = false;
    dma_context_.callback = callback;
    dma_context_.is_register_op = true;
    dma_context_.reg_addr = reg;
    dma_context_.reg_size = reg_size;
    dma_context_.reg_buffer = reg_write_buffer_;
    
    // 准备寄存器地址数据
    if (reg_size == 2) {
        reg_write_buffer_[0] = (reg >> 8) & 0x7F;
        reg_write_buffer_[1] = reg & 0xFF;
    } else {
        reg_write_buffer_[0] = reg & 0xFF;
    }
    
    // 使用写后读操作
    return _setup_dma_write_read(address, reg_write_buffer_, reg_size, value, length);
}

bool HAL_I2C::write_register_async(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length, dma_callback_t callback) {
    if (!initialized_ || dma_status_ != DMA_Status::IDLE || !value || length == 0 || !callback) {
        return false;
    }
    
    // 计算寄存器地址大小
    uint8_t reg_size = (reg & 0xFF00) ? 2 : (reg & 0x8000) ? 2 : 1;
    
    if (reg_size + length > sizeof(reg_write_buffer_)) {
        return false;  // 数据太大
    }
    
    // 设置DMA上下文
    dma_context_.device_addr = address;
    dma_context_.buffer = value;
    dma_context_.length = length;
    dma_context_.is_write = true;
    dma_context_.callback = callback;
    dma_context_.is_register_op = true;
    dma_context_.reg_addr = reg;
    dma_context_.reg_size = reg_size;
    dma_context_.reg_buffer = reg_write_buffer_;
    
    // 准备寄存器地址+数据
    if (reg_size == 2) {
        reg_write_buffer_[0] = (reg >> 8) & 0x7F;
        reg_write_buffer_[1] = reg & 0xFF;
    } else {
        reg_write_buffer_[0] = reg & 0xFF;
    }
    memcpy(reg_write_buffer_ + reg_size, value, length);
    
    // 写入寄存器地址+数据
    return _setup_dma_write(address, reg_write_buffer_, reg_size + length);
}

bool HAL_I2C::is_busy() const {
    return dma_status_ != DMA_Status::IDLE;
}

// DMA写入设置
bool HAL_I2C::_setup_dma_write(uint8_t address, const uint8_t* data, size_t length) {
    if (dma_status_ != DMA_Status::IDLE) {
        return false;
    }
    
    // 启用I2C中断用于异步操作
    _enable_i2c_interrupts();
    
    if (length == 0 || length > 256) {
        return false;
    }
    
    dma_status_ = DMA_Status::TX_BUSY;
    
    // 准备I2C命令 - 参考i2c_dma.c的data_cmds设置
    for (size_t i = 0; i < length; ++i) {
        data_cmds_[i] = data[i];
    }
    
    // 第一个字节需要RESTART位
    data_cmds_[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
    // 最后一个字节需要STOP位
    data_cmds_[length - 1] |= I2C_IC_DATA_CMD_STOP_BITS;
    
    // 设置I2C目标地址
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    hw->enable = 0;
    hw->tar = address;
    hw->enable = 1;
    
    // 配置DMA通道 - 使用16位传输以支持命令位
    dma_channel_config c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, i2c_get_dreq(i2c_instance_, true));
    
    // 启动DMA传输
    dma_channel_configure(
        dma_tx_channel_,
        &c,
        &hw->data_cmd,
        data_cmds_,
        length,
        true
    );
    
    return true;
}

// DMA读取设置
bool HAL_I2C::_setup_dma_read(uint8_t address, uint8_t* buffer, size_t length) {
    if (dma_status_ != DMA_Status::IDLE) {
        return false;
    }
    
    if (length == 0 || length > 256) {
        return false;
    }
    
    dma_status_ = DMA_Status::RX_BUSY;
    
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    
    // 准备读命令 - 参考i2c_dma.c的实现
    for (size_t i = 0; i < length; ++i) {
        data_cmds_[i] = I2C_IC_DATA_CMD_CMD_BITS;
    }
    
    // 第一个读命令需要RESTART位
    data_cmds_[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
    // 最后一个读命令需要STOP位
    data_cmds_[length - 1] |= I2C_IC_DATA_CMD_STOP_BITS;
    
    // 设置I2C目标地址
    hw->enable = 0;
    hw->tar = address;
    hw->enable = 1;
    
    // 配置RX DMA通道
    dma_channel_config rx_c = dma_channel_get_default_config(dma_rx_channel_);
    channel_config_set_transfer_data_size(&rx_c, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_c, false);
    channel_config_set_write_increment(&rx_c, true);
    channel_config_set_dreq(&rx_c, i2c_get_dreq(i2c_instance_, false));
    
    // 配置TX DMA通道用于发送读命令
    dma_channel_config tx_c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&tx_c, DMA_SIZE_16);
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_write_increment(&tx_c, false);
    channel_config_set_dreq(&tx_c, i2c_get_dreq(i2c_instance_, true));
    
    // 配置DMA通道
    dma_channel_configure(
        dma_rx_channel_,
        &rx_c,
        buffer,
        &hw->data_cmd,
        length,
        false
    );
    
    dma_channel_configure(
        dma_tx_channel_,
        &tx_c,
        &hw->data_cmd,
        data_cmds_,
        length,
        false
    );

    // 启用I2C中断用于异步操作
    _enable_i2c_interrupts();
    
    // 先启动RX，再启动TX
    dma_channel_start(dma_rx_channel_);
    dma_channel_start(dma_tx_channel_);
    
    return true;
}

// DMA写后读设置 - 参考i2c_dma.c的write_read实现
bool HAL_I2C::_setup_dma_write_read(uint8_t address, const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen) {
    if (wlen == 0 || rlen == 0 || (wlen + rlen) > 256) {
        return false;
    }
    
    dma_status_ = DMA_Status::RX_BUSY;  // 最终状态是读取
    
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    
    // 准备写命令
    for (size_t i = 0; i < wlen; ++i) {
        data_cmds_[i] = wbuf[i];
    }
    
    // 准备读命令
    for (size_t i = 0; i < rlen; ++i) {
        data_cmds_[wlen + i] = I2C_IC_DATA_CMD_CMD_BITS;
    }
    
    // 第一个写字节需要RESTART位
    data_cmds_[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
    // 第一个读字节需要RESTART位
    data_cmds_[wlen] |= I2C_IC_DATA_CMD_RESTART_BITS;
    // 最后一个读字节需要STOP位
    data_cmds_[wlen + rlen - 1] |= I2C_IC_DATA_CMD_STOP_BITS;
    
    // 设置I2C目标地址
    hw->enable = 0;
    hw->tar = address;
    hw->enable = 1;
    
    // 配置RX DMA通道
    dma_channel_config rx_c = dma_channel_get_default_config(dma_rx_channel_);
    channel_config_set_transfer_data_size(&rx_c, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_c, false);
    channel_config_set_write_increment(&rx_c, true);
    channel_config_set_dreq(&rx_c, i2c_get_dreq(i2c_instance_, false));
    
    // 配置TX DMA通道
    dma_channel_config tx_c = dma_channel_get_default_config(dma_tx_channel_);
    channel_config_set_transfer_data_size(&tx_c, DMA_SIZE_16);
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_write_increment(&tx_c, false);
    channel_config_set_dreq(&tx_c, i2c_get_dreq(i2c_instance_, true));
    
    // 配置DMA通道
    dma_channel_configure(
        dma_rx_channel_,
        &rx_c,
        rbuf,
        &hw->data_cmd,
        rlen,
        false
    );
    
    dma_channel_configure(
        dma_tx_channel_,
        &tx_c,
        &hw->data_cmd,
        data_cmds_,
        wlen + rlen,
        false
    );

    // 启用I2C中断用于异步操作
    _enable_i2c_interrupts();
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

void HAL_I2C::_handle_i2c_irq() {
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    uint32_t intr_stat = hw->intr_stat;
    
    // 处理TX_ABRT中断
    if (intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        // 清除TX_ABRT中断
        (void)hw->clr_tx_abrt;
        
        // 停止DMA传输
        if (dma_tx_channel_ >= 0) {
            dma_channel_abort(dma_tx_channel_);
        }
        if (dma_rx_channel_ >= 0) {
            dma_channel_abort(dma_rx_channel_);
        }
        
        dma_status_ = DMA_Status::ERROR;
        
        // 调用回调函数
        if (dma_context_.callback) {
            dma_context_.callback(false);
        }
        
        dma_status_ = DMA_Status::IDLE;
        return;
    }
    
    // 处理STOP_DET中断
    if (intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        // 清除STOP_DET中断
        (void)hw->clr_stop_det;

        dma_channel_wait_for_finish_blocking(dma_rx_channel_);
        
        // 调用回调函数
        if (dma_context_.callback) {
            dma_context_.callback(true);
        }
        dma_status_ = DMA_Status::IDLE;
        return;
    }
}

HAL_I2C0::HAL_I2C0() : HAL_I2C(i2c0) {}

HAL_I2C0::~HAL_I2C0() {
    deinit();
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

HAL_I2C1::HAL_I2C1() : HAL_I2C(i2c1) {}

HAL_I2C1::~HAL_I2C1() {
    deinit();
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

// 动态启用I2C中断（仅在异步操作时使用）
void HAL_I2C::_enable_i2c_interrupts() {
    if (interrupts_enabled_) {
        return;  // 已经启用，避免重复操作
    }
    
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    hw->intr_mask = I2C_IC_INTR_MASK_M_STOP_DET_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS;
    
    uint irq_num = (i2c_instance_ == i2c0) ? I2C0_IRQ : I2C1_IRQ;
    irq_set_enabled(irq_num, true);
    
    interrupts_enabled_ = true;
}

// 动态禁用I2C中断（异步操作完成后恢复同步操作）
void HAL_I2C::_disable_i2c_interrupts() {
    if (!interrupts_enabled_) {
        return;  // 已经禁用，避免重复操作
    }
    
    uint irq_num = (i2c_instance_ == i2c0) ? I2C0_IRQ : I2C1_IRQ;
    irq_set_enabled(irq_num, false);
    
    i2c_hw_t *hw = i2c_get_hw(i2c_instance_);
    hw->intr_mask = 0; // 清除所有中断掩码
    
    interrupts_enabled_ = false;
}

// 等待总线空闲的内联函数，带有超时设置
inline bool HAL_I2C::_wait_for_bus_idle(uint32_t timeout_ms) {
    if (!initialized_) return false;
    
    uint32_t start_time = time_us_32();
    
    // 等待异步操作完成
    while (dma_status_ != DMA_Status::IDLE) {
        if (time_us_32() - start_time >= timeout_ms * 1000) {
            return false; // 超时
        }
        sleep_us(10); // 短暂等待，避免忙等待
    }
    
    return true;
}

// 解除I2C总线锁定：通过手动SCL脉冲与STOP条件释放SDA
void HAL_I2C::unlock_bus(uint8_t sda_pin, uint8_t scl_pin, uint32_t pulse_delay_us) {
    // 切换为GPIO功能，并保持上拉
    gpio_set_function(sda_pin, GPIO_FUNC_SIO);
    gpio_set_function(scl_pin, GPIO_FUNC_SIO);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // 释放（输入态模拟开漏高电平）
    gpio_set_dir(sda_pin, GPIO_IN);
    gpio_set_dir(scl_pin, GPIO_IN);

    // 若SDA为低，尝试通过SCL脉冲使从设备释放SDA
    if (!gpio_get(sda_pin)) {
        for (int i = 0; i < 18 && !gpio_get(sda_pin); ++i) {
            // Drive SCL low
            gpio_set_dir(scl_pin, GPIO_OUT);
            gpio_put(scl_pin, 0);
            sleep_us(pulse_delay_us);
            // Release SCL high
            gpio_set_dir(scl_pin, GPIO_IN);
            sleep_us(pulse_delay_us);
        }
    }

    // 发送STOP条件：SDA低 -> SCL高 -> SDA高
    gpio_set_dir(sda_pin, GPIO_OUT);
    gpio_put(sda_pin, 0);
    sleep_us(pulse_delay_us);

    gpio_set_dir(scl_pin, GPIO_IN); // 释放SCL为高
    sleep_us(pulse_delay_us);

    gpio_set_dir(sda_pin, GPIO_IN); // 释放SDA为高
    sleep_us(pulse_delay_us);
}