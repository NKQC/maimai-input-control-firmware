#pragma once

#include "../../hal/spi/hal_spi.h"
#include <stdint.h>
#include <functional>

/**
 * 协议层 - MCP23S17 GPIO扩展器
 * 基于SPI通信的16位GPIO扩展器
 * 支持中断检测和GPIO状态读写
 */

// MCP23S17 SPI配置
#define MCP23S17_SPI_SPEED      10000000  // 10MHz
#define MCP23S17_OPCODE_WRITE   0x40
#define MCP23S17_OPCODE_READ    0x41

// MCP23S17寄存器地址（IOCON.BANK = 0模式）
#define MCP23S17_REG_IODIRA     0x00  // GPIO方向寄存器A
#define MCP23S17_REG_IODIRB     0x01  // GPIO方向寄存器B
#define MCP23S17_REG_IPOLA      0x02  // 输入极性寄存器A
#define MCP23S17_REG_IPOLB      0x03  // 输入极性寄存器B
#define MCP23S17_REG_GPINTENA   0x04  // 中断使能寄存器A
#define MCP23S17_REG_GPINTENB   0x05  // 中断使能寄存器B
#define MCP23S17_REG_DEFVALA    0x06  // 默认值寄存器A
#define MCP23S17_REG_DEFVALB    0x07  // 默认值寄存器B
#define MCP23S17_REG_INTCONA    0x08  // 中断控制寄存器A
#define MCP23S17_REG_INTCONB    0x09  // 中断控制寄存器B
#define MCP23S17_REG_IOCON      0x0A  // 配置寄存器
#define MCP23S17_REG_GPPUA      0x0C  // 上拉电阻寄存器A
#define MCP23S17_REG_GPPUB      0x0D  // 上拉电阻寄存器B
#define MCP23S17_REG_INTFA      0x0E  // 中断标志寄存器A
#define MCP23S17_REG_INTFB      0x0F  // 中断标志寄存器B
#define MCP23S17_REG_INTCAPA    0x10  // 中断捕获寄存器A
#define MCP23S17_REG_INTCAPB    0x11  // 中断捕获寄存器B
#define MCP23S17_REG_GPIOA      0x12  // GPIO寄存器A
#define MCP23S17_REG_GPIOB      0x13  // GPIO寄存器B
#define MCP23S17_REG_OLATA      0x14  // 输出锁存寄存器A
#define MCP23S17_REG_OLATB      0x15  // 输出锁存寄存器B

// GPIO端口定义
enum MCP23S17_Port {
    MCP23S17_PORT_A = 0,
    MCP23S17_PORT_B = 1
};

// GPIO方向定义
enum MCP23S17_Direction {
    MCP23S17_OUTPUT = 0,
    MCP23S17_INPUT = 1
};

// 中断类型定义
enum MCP23S17_IntType {
    MCP23S17_INT_CHANGE = 0,    // 状态变化中断
    MCP23S17_INT_COMPARE = 1    // 与默认值比较中断
};

// GPIO状态结构
struct MCP23S17_GPIO_State {
    uint8_t port_a;     // 端口A状态
    uint8_t port_b;     // 端口B状态
    uint32_t timestamp; // 时间戳
};

class MCP23S17 {
public:
    MCP23S17(HAL_SPI* spi_hal, uint8_t cs_pin, uint8_t device_addr = 0);
    ~MCP23S17();
    
    // 初始化设备
    bool init();
    
    // 释放资源
    void deinit();
    
    // 检查设备是否就绪
    bool is_ready() const;
    
    // GPIO方向配置
    bool set_pin_direction(MCP23S17_Port port, uint8_t pin, MCP23S17_Direction dir);
    bool set_port_direction(MCP23S17_Port port, uint8_t direction_mask);
    
    // GPIO读写
    bool write_pin(MCP23S17_Port port, uint8_t pin, bool value);
    bool write_port(MCP23S17_Port port, uint8_t value);
    bool read_pin(MCP23S17_Port port, uint8_t pin, bool& value);
    bool read_port(MCP23S17_Port port, uint8_t& value);
    
    // 读取所有GPIO状态
    bool read_all_gpio(MCP23S17_GPIO_State& state);
    
    // 上拉电阻配置
    bool set_pin_pullup(MCP23S17_Port port, uint8_t pin, bool enable);
    bool set_port_pullup(MCP23S17_Port port, uint8_t pullup_mask);
    
    // 输入极性配置
    bool set_pin_polarity(MCP23S17_Port port, uint8_t pin, bool inverted);
    bool set_port_polarity(MCP23S17_Port port, uint8_t polarity_mask);
    
    // 中断配置
    bool enable_pin_interrupt(MCP23S17_Port port, uint8_t pin, MCP23S17_IntType type, uint8_t compare_value = 0);
    bool disable_pin_interrupt(MCP23S17_Port port, uint8_t pin);
    bool enable_port_interrupt(MCP23S17_Port port, uint8_t interrupt_mask, MCP23S17_IntType type, uint8_t compare_value = 0);
    bool disable_port_interrupt(MCP23S17_Port port);
    
    // 中断状态读取
    bool read_interrupt_flags(uint8_t& intf_a, uint8_t& intf_b);
    bool read_interrupt_capture(uint8_t& intcap_a, uint8_t& intcap_b);
    bool clear_interrupts();
    
    // 设置中断回调
    void set_interrupt_callback(std::function<void(const MCP23S17_GPIO_State&, uint8_t, uint8_t)> callback);
    
    // 中断处理（需要在中断服务程序中调用）
    void handle_interrupt();
    
    // 任务处理（需要在主循环中调用）
    void task();
    
    // 配置寄存器操作
    bool configure_iocon(uint8_t config);
    
private:
    HAL_SPI* spi_hal_;
    uint8_t cs_pin_;
    uint8_t device_addr_;  // 设备地址（A2,A1,A0）
    bool initialized_;
    
    // 中断回调
    std::function<void(const MCP23S17_GPIO_State&, uint8_t, uint8_t)> interrupt_callback_;
    
    // 状态缓存
    MCP23S17_GPIO_State last_state_;
    bool state_changed_;
    
    // 内部方法
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool write_register_pair(uint8_t reg_a, uint8_t value_a, uint8_t reg_b, uint8_t value_b);
    bool read_register_pair(uint8_t reg_a, uint8_t& value_a, uint8_t reg_b, uint8_t& value_b);
    
    // SPI传输
    bool spi_transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length);
    
    // 设备配置
    bool configure_device();
    bool test_device_communication();
};