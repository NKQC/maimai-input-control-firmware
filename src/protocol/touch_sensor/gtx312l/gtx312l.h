#pragma once

#include "../../../hal/i2c/hal_i2c.h"
#include "../touch_sensor.h"
#include <stdint.h>
#include <vector>
#include <functional>
#include <cstring>

/**
 * 协议层 - GTX312L触摸控制器
 * 基于I2C通信的12通道电容式触摸控制器
 * 支持触摸按键检测，无坐标输出
 * 工作电压：1.8V-5.5V，支持I2C接口
 * 负责物理地址组装和掩码管理
 */
typedef uint32_t millis_t;

// https://www.cpbay.com/Uploads/20210128/601279b9b90ec.pdf

// GTX312L I2C地址范围（可配置）
#define GTX312L_I2C_ADDR_MIN        0xB6    // 最小I2C地址
#define GTX312L_I2C_ADDR_MAX        0xB0    // 最大I2C地址
#define GTX312L_I2C_ADDR_DEFAULT    0xB2    // 默认I2C地址

// GTX312L寄存器定义（基于实际datasheet）
#define GTX312L_REG_CHIPADDR_VER    0x01    // 芯片地址寄存器
#define GTX312L_REG_TOUCH_STATUS_L  0x02    // 触摸状态寄存器低字节（通道1-8）
#define GTX312L_REG_TOUCH_STATUS_H  0x03    // 触摸状态寄存器高字节（通道9-12）
#define GTX312L_REG_CH_ENABLE_L     0x04    // 通道使能寄存器低字节（通道1-8）
#define GTX312L_REG_CH_ENABLE_H     0x05    // 通道使能寄存器高字节（通道9-12）
#define GTX312L_REG_MON_RST         0x0A    // 监控复位寄存器
#define GTX312L_REG_SLEEP           0x0B    // 睡眠寄存器
#define GTX312L_REG_I2C_PU_DIS      0x0C    // I2C上拉禁用寄存器
#define GTX312L_REG_WRITE_LOCK      0x0F    // 寄存器写保护锁
#define GTX312L_REG_INT_TOUCH_MODE  0x10    // 中断/触触摸模式配置寄存器 0:(0单点模式/1多点模式) 5:(0脉冲模式/1电平模式) 
#define GTX312L_REG_EXP_CONFIG      0x11    // 触摸过期时间寄存器 0:过期模式(0不同触摸不更新时间/1不同触摸更新时间) 1:模式开关(0关闭/1开启) 4-6: 超时时间
#define GTX312L_REG_CAL_TIME        0x13    // 校准时间配置寄存器  0-3
#define GTX312L_REG_SEN_IDLE_TIME   0x14    // 感应空闲时间寄存器  0-3
#define GTX312L_REG_SEN_IDLE_SUFFIX 0x15    // 感应空闲时间后缀寄存器 0-3
#define GTX312L_REG_BUSY_TO_IDLE    0x17    // 忙碌到空闲时间寄存器 0-2
#define GTX312L_REG_I2B_MODE        0x18    // I2B(空闲到忙碌)模式寄存器 0:(0自动模式/1手动模式)
#define GTX312L_REG_SLIDE_MODE      0x19    // 滑动模式寄存器 0:(0关闭/1启用)
#define GTX312L_REG_SENSITIVITY_1   0x20    // 通道1灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_2   0x21    // 通道2灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_3   0x22    // 通道3灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_4   0x23    // 通道4灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_5   0x24    // 通道5灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_6   0x25    // 通道6灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_7   0x26    // 通道7灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_8   0x27    // 通道8灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_9   0x28    // 通道9灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_10  0x29    // 通道10灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_11  0x2A    // 通道11灵敏度寄存器
#define GTX312L_REG_SENSITIVITY_12  0x2B    // 通道12灵敏度寄存器

// 触摸通道相关常量
#define GTX312L_MAX_CHANNELS        12      // 最大触摸通道数

// 寄存器值定义
#define GTX312L_WRITE_LOCK_VALUE    0x5A    // 写保护解锁值
#define GTX312L_SOFT_RST_VALUE      0x01    // 软件复位值
#define GTX312L_MON_RST_VALUE       0x01    // 监控复位值
#define GTX312L_CH_ENABLE_ALL_L     0xFF    // 通道1-8全部使能
#define GTX312L_CH_ENABLE_ALL_H     0x3F    // 通道9-12全部使能（bit3-0有效）

// 灵敏度相关常量
#define GTX312L_SENSITIVITY_MIN     0x00    // 最小灵敏度
#define GTX312L_SENSITIVITY_MAX     0x3F    // 最大灵敏度（6位有效）
#define GTX312L_SENSITIVITY_DEFAULT 0x0F    // 默认灵敏度

// 中断模式位定义
#define GTX312L_INT_MODE_ENABLE     0x08    // 中断模式使能位
#define GTX312L_MULTI_MODE_ENABLE   0x01    // 多点触摸模式使能位

// 扩展配置位定义
#define GTX312L_EXP_EN              0x02    // 扩展功能使能位
#define GTX312L_EXP_MODE            0x01    // 扩展模式位



// 设备信息结构
struct GTX312L_DeviceInfo {
    uint8_t i2c_address;
    bool is_valid;
    
    GTX312L_DeviceInfo() : i2c_address(0), is_valid(false) {}
};

typedef union {
    struct {
        uint8_t h;
        uint8_t l;
    };
    uint16_t value;
} GTX312L_SampleData;



class GTX312L : public TouchSensor {
public:
    GTX312L(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr);
    ~GTX312L() override;
    
    // 初始化和清理
    // TouchSensor接口实现
    void sample(async_touchsampleresult callback) override; // 异步采样接口
    uint32_t getSupportedChannelCount() const override;
    bool init() override;
    void deinit() override;
    bool isInitialized() const override;
    
    // GTX312L特有接口
    bool read_device_info(GTX312L_DeviceInfo& info);
    
    // TouchSensor新接口实现
    bool setChannelEnabled(uint8_t channel, bool enabled) override;    // 设置单个通道使能
    bool getChannelEnabled(uint8_t channel) const override;            // 获取单个通道使能状态
    uint32_t getEnabledChannelMask() const override;                   // 获取启用通道掩码
    bool setChannelSensitivity(uint8_t channel, uint8_t sensitivity) override;  // 设置通道灵敏度 (0-99)
    uint8_t getChannelSensitivity(uint8_t channel) const override;     // 获取通道灵敏度 (0-99)
    
private:
    // I2C通信相关
    HAL_I2C* i2c_hal_;
    I2C_Bus i2c_bus_;
    uint8_t device_addr_;                    // GTX312L设备地址 (0-3)
    uint8_t i2c_device_address_;             // 实际I2C设备地址 (0x14 + device_addr_)
    
    // 设备状态
    bool initialized_;
    I2C_Bus i2c_bus_enum_;                   // I2C总线枚举
    uint32_t enabled_channels_mask_;         // 启用的通道掩码

    // 异步I2C操作缓冲区（避免热点函数反复创建变量）
    static uint8_t _async_read_buffer[2]; // 异步读取数据缓冲区

    // 私有方法
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool write_registers(uint8_t reg, const uint8_t* data, size_t length);
    bool read_registers(uint8_t reg, uint8_t* data, size_t length);
};