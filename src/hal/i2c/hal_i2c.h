#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <functional>
#include <hardware/i2c.h>

extern "C" {
#include "../global_irq.h"
}

/**
 * HAL层 - I2C接口抽象类
 * 提供底层I2C接口，支持I2C0和I2C1两个实例
 * 使用DMA实现高效的数据传输
 * TODO: DMA尚未跑通 参考
 * https://github.com/fivdi/pico-i2c-dma/blob/master/src/i2c_dma.c
 */

// I2C总线枚举 - HAL层只提供通道信息
enum class I2C_Bus : uint8_t {
    I2C0 = 0,
    I2C1 = 1
};

class HAL_I2C {
public:
    using dma_callback_t = std::function<void(bool success)>;
    
    virtual ~HAL_I2C() = default;
    
    // 初始化I2C接口
    bool init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency = 100000);
    
    // 释放I2C资源
    void deinit();
    
    // 写入数据
    bool write(uint8_t address, const uint8_t* data, size_t length);
    
    // 读取数据
    bool read(uint8_t address, uint8_t* buffer, size_t length);

    // 异步DMA读写操作
    bool read_async(uint8_t address, uint8_t* buffer, size_t length, dma_callback_t callback = nullptr);
    bool write_async(uint8_t address, const uint8_t* data, size_t length, dma_callback_t callback = nullptr);
    
    // 检查DMA传输状态
    bool is_busy() const;
    
    // 写入寄存器 REG & 0x8000 时 锁定16位发送 否则根据是否满9位地址判断发送8或16位
    int write_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length);
    
    // 读取寄存器 REG & 0x8000 时 锁定16位发送 否则根据是否满9位地址判断发送8或16位
    int read_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length);

    // 检查设备是否存在
    bool device_exists(uint8_t address);
    
    // 扫描I2C总线上的设备
    std::vector<uint8_t> scan_devices();
    
    // 获取实例名称
    virtual std::string get_name() const = 0;

protected:
    // 构造函数，由子类调用
    HAL_I2C(i2c_inst_t* i2c_instance, void (*tx_callback)(bool), void (*rx_callback)(bool));
    
    // I2C实例和状态变量
    i2c_inst_t* i2c_instance_;
    bool initialized_;
    uint8_t sda_pin_;
    uint8_t scl_pin_;
    bool dma_busy_;
    dma_callback_t dma_callback_;
    int dma_tx_channel_;
    int dma_rx_channel_;
    
    // DMA回调函数指针
    void (*tx_dma_callback_)(bool);
    void (*rx_dma_callback_)(bool);
};

// I2C0实例
class HAL_I2C0 : public HAL_I2C {
public:
    static HAL_I2C0* getInstance();
    ~HAL_I2C0();
    
    std::string get_name() const override { return "I2C0"; }
    
    // 友元函数声明，允许DMA回调函数访问私有成员
    friend void i2c0_tx_dma_callback(bool success);
    friend void i2c0_rx_dma_callback(bool success);

private:
    static HAL_I2C0* instance_;
    
    // 私有构造函数（单例模式）
    HAL_I2C0();
    HAL_I2C0(const HAL_I2C0&) = delete;
    HAL_I2C0& operator=(const HAL_I2C0&) = delete;
};

// I2C1实例
class HAL_I2C1 : public HAL_I2C {
public:
    static HAL_I2C1* getInstance();
    ~HAL_I2C1();
    
    std::string get_name() const override { return "I2C1"; }
    
    // 友元函数声明，允许DMA回调函数访问私有成员
    friend void i2c1_tx_dma_callback(bool success);
    friend void i2c1_rx_dma_callback(bool success);

private:
    static HAL_I2C1* instance_;
    
    // 私有构造函数（单例模式）
    HAL_I2C1();
    HAL_I2C1(const HAL_I2C1&) = delete;
    HAL_I2C1& operator=(const HAL_I2C1&) = delete;
};