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
 * 参考: https://github.com/fivdi/pico-i2c-dma/blob/master/src/i2c_dma.c
 */

// I2C总线枚举 - HAL层只提供通道信息
enum class I2C_Bus : uint8_t {
    I2C0 = 0,
    I2C1 = 1
};

enum I2C_Error : int32_t {
    OK = 0,
    TIMEOUT = -1,
    STATE_ERROR = -2,
    NACK = -3,
    WRITE_SIZE_ERROR = -4,
    READ_SIZE_ERROR = -5,
    OTHER = -6,
};

// DMA传输状态
enum class DMA_Status : uint8_t {
    IDLE = 0,
    TX_BUSY,
    RX_BUSY,
    ERROR
};

// DMA传输上下文结构体
struct DMA_Context {
    uint8_t device_addr;
    uint8_t* buffer;
    size_t length;
    bool is_write;
    std::function<void(bool)> callback;
    
    // 新增寄存器操作相关字段
    uint16_t reg_addr;
    uint8_t reg_size;
    bool is_register_op;
    uint8_t* reg_buffer;  // 用于存储寄存器地址+数据的缓冲区
    
    // 添加DMA传输控制字段
    uint16_t* data_cmds;  // I2C命令缓冲区
    size_t cmd_count;     // 命令数量
    
    DMA_Context() : device_addr(0), buffer(nullptr), length(0), is_write(false), 
                   reg_addr(0), reg_size(0), is_register_op(false), reg_buffer(nullptr),
                   data_cmds(nullptr), cmd_count(0) {}
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
    int32_t write(uint8_t address, const uint8_t* data, size_t length);
    
    // 读取数据
    int32_t read(uint8_t address, uint8_t* buffer, size_t length);

    int32_t read_register_async(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length, dma_callback_t callback);
    int32_t write_register_async(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length, dma_callback_t callback);

    int32_t read_async(uint8_t address, uint8_t* buffer, size_t length, dma_callback_t callback = nullptr);
    int32_t write_async(uint8_t address, const uint8_t* data, size_t length, dma_callback_t callback = nullptr);
    
    // 检查DMA传输状态
    bool is_busy() const;
    
    // 写入寄存器 REG & 0x8000 时 锁定16位发送 否则根据是否满9位地址判断发送8或16位
    int32_t write_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length);
    
    // 读取寄存器 REG & 0x8000 时 锁定16位发送 否则根据是否满9位地址判断发送8或16位
    int32_t read_register(uint8_t address, uint16_t reg, uint8_t* value, uint8_t length);

    // 检查设备是否存在
    bool device_exists(uint8_t address);
    
    // 扫描I2C总线上的设备
    std::vector<uint8_t> scan_devices();
    
    // 获取实例名称
    virtual std::string get_name() const = 0;

protected:
    // 构造函数 - 子类调用
    HAL_I2C(i2c_inst_t* i2c_instance);
    
    // I2C实例和基本配置
    i2c_inst_t* i2c_instance_;
    bool initialized_;
    uint8_t sda_pin_;
    uint8_t scl_pin_;
    
    // DMA相关成员
    volatile DMA_Status dma_status_;
    DMA_Context dma_context_;
    int32_t dma_tx_channel_;
    int32_t dma_rx_channel_;
    
    // 中断状态跟踪（惰性管理）
    volatile bool interrupts_enabled_;
    
    // I2C读命令字
    uint16_t read_cmd_;
    
    // 寄存器操作缓冲区
    uint8_t reg_write_buffer_[258];  // 最大2字节寄存器地址 + 256字节数据
    
    // DMA命令缓冲区
    uint16_t data_cmds_[260];  // 最大传输大小的命令缓冲区
    
    // 内部DMA设置函数
    inline int32_t _setup_dma_write(uint8_t address, const uint8_t* data, size_t length);
    inline int32_t _setup_dma_read(uint8_t address, uint8_t* buffer, size_t length);
    inline int32_t _setup_dma_write_read(uint8_t address, const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen);
    
    // 等待总线空闲的内联函数，带有超时设置
    inline bool _wait_for_bus_idle(uint32_t timeout_ms = 10);
    
    // I2C中断处理
    void _handle_i2c_irq();
    
    // 动态中断管理（用于同步/异步操作共存）
    inline void _enable_i2c_interrupts();
    inline void _disable_i2c_interrupts();
};

// I2C0实例
class HAL_I2C0 : public HAL_I2C {
public:
    static HAL_I2C0* getInstance();
    ~HAL_I2C0();
    
    std::string get_name() const override { return "I2C0"; }
    
    // I2C中断处理友元
    friend void i2c0_irq_handler();

private:
    static HAL_I2C0* instance_;
    
    // 私有构造函数 - 单例模式
    HAL_I2C0();
    HAL_I2C0(const HAL_I2C0&) = delete;
    HAL_I2C0& operator=(const HAL_I2C0&) = delete;
};

// HAL_I2C1类 - I2C1实例
class HAL_I2C1 : public HAL_I2C {
public:
    static HAL_I2C1* getInstance();
    ~HAL_I2C1();
    
    std::string get_name() const override { return "I2C1"; }
    
    // I2C中断处理友元
    friend void i2c1_irq_handler();

private:
    static HAL_I2C1* instance_;
    
    // 私有构造函数 - 单例模式
    HAL_I2C1();
    HAL_I2C1(const HAL_I2C1&) = delete;
    HAL_I2C1& operator=(const HAL_I2C1&) = delete;
};

// I2C中断处理器声明
void i2c0_irq_handler();
void i2c1_irq_handler();