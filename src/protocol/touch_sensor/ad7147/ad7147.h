#pragma once

#include "../../../hal/i2c/hal_i2c.h"
#include "../touch_sensor.h"
#include <stdint.h>
#include <string>

/**
 * 协议层 - AD7147触摸控制器
 * 基于I2C通信的电容式触摸控制器
 * 支持最多13个触摸通道
 * 工作电压：2.6V-5.5V，支持I2C接口
 * https://www.analog.com/media/en/technical-documentation/data-sheets/ad7147.pdf
 */

// AD7147寄存器定义
#define AD7147_I2C_ADDR_DEFAULT     0x2C    // 默认I2C地址
#define AD7147_MAX_CHANNELS         13      // 最大触摸通道数

// 主要寄存器地址
#define AD7147_REG_PWR_CONTROL      0x0000  // 电源控制寄存器
#define AD7147_REG_STAGE_CAL_EN     0x0001  // 阶段校准使能寄存器
#define AD7147_REG_AMB_COMP_CTRL0   0x0002  // 环境补偿控制寄存器0
#define AD7147_REG_AMB_COMP_CTRL1   0x0003  // 环境补偿控制寄存器1
#define AD7147_REG_AMB_COMP_CTRL2   0x0004  // 环境补偿控制寄存器2
#define AD7147_REG_STAGE_LOW_INT_EN 0x0005  // 阶段低中断使能寄存器
#define AD7147_REG_STAGE_HIGH_INT_EN 0x0006 // 阶段高中断使能寄存器
#define AD7147_REG_STAGE_COMPLETE_INT_EN 0x0007 // 阶段完成中断使能寄存器

// 状态寄存器（读取将清除已置位的状态位）
#define AD7147_REG_STAGE_LOW_INT_STATUS      0x0008    // 阶段低中断状态寄存器
#define AD7147_REG_STAGE_HIGH_INT_STATUS     0x0009    // 阶段高中断状态寄存器
#define AD7147_REG_STAGE_COMPLETE_INT_STATUS 0x000A    // 阶段完成中断状态寄存器

// CDC数据
#define AD7147_REG_CDC_DATA                 0x000B    // CDC数据寄存器

// Stage配置寄存器基地址（每个stage占用8个16位寄存器）
#define AD7147_REG_STAGE0_CONNECTION         0x0080   // Stage 0连接寄存器
#define AD7147_REG_STAGE_SIZE                8        // 每个stage占用的寄存器数量

#define AD7147_REG_DEVICE_ID                 0x0017   // 设备ID寄存器

// Stage寄存器偏移（相对于STAGEx_CONNECTION基地址）
#define AD7147_STAGE_CONNECTION_OFFSET       0        // 连接配置寄存器偏移
#define AD7147_STAGE_AFE_OFFSET_OFFSET       2        // AFE偏移寄存器偏移
#define AD7147_STAGE_SENSITIVITY_OFFSET      3        // 灵敏度寄存器偏移
#define AD7147_STAGE_OFFSET_LOW_OFFSET       4        // 低偏移寄存器偏移
#define AD7147_STAGE_OFFSET_HIGH_OFFSET      5        // 高偏移寄存器偏移
#define AD7147_STAGE_OFFSET_HIGH_CLAMP_OFFSET 6       // 高偏移钳位寄存器偏移
#define AD7147_STAGE_OFFSET_LOW_CLAMP_OFFSET 7        // 低偏移钳位寄存器偏移

// 灵敏度寄存器默认值
#define AD7147_SENSITIVITY_DEFAULT           0x4A4A   // 默认灵敏度值
#define AD7147_DEFAULT_AFE_OFFSET            0x0      // AFE偏移默认值

// 阶段配置相关常量
#define AD7147_STAGE1_CONNECTION             0x0088   // Stage 1连接寄存器
#define AD7147_STAGE2_CONNECTION             0x0090   // Stage 2连接寄存器
#define AD7147_STAGE3_CONNECTION             0x0098   // Stage 3连接寄存器
#define AD7147_STAGE4_CONNECTION             0x00A0   // Stage 4连接寄存器
#define AD7147_STAGE5_CONNECTION             0x00A8   // Stage 5连接寄存器
#define AD7147_STAGE6_CONNECTION             0x00B0   // Stage 6连接寄存器
#define AD7147_STAGE7_CONNECTION             0x00B8   // Stage 7连接寄存器
#define AD7147_STAGE8_CONNECTION             0x00C0   // Stage 8连接寄存器
#define AD7147_STAGE9_CONNECTION             0x00C8   // Stage 9连接寄存器
#define AD7147_STAGE10_CONNECTION            0x00D0   // Stage 10连接寄存器
#define AD7147_STAGE11_CONNECTION            0x00D8   // Stage 11连接寄存器

// 阶段配置默认值
#define AD7147_DEFAULT_OFFSET_LOW            50       // 默认低偏移值
#define AD7147_DEFAULT_OFFSET_HIGH           50       // 默认高偏移值
#define AD7147_DEFAULT_OFFSET_HIGH_CLAMP     100      // 默认高偏移钳位值
#define AD7147_DEFAULT_OFFSET_LOW_CLAMP      100      // 默认低偏移钳位值

// 设备信息结构体
struct AD7147_DeviceInfo {
    uint8_t i2c_address;
    bool is_valid;
    
    AD7147_DeviceInfo() : i2c_address(0), is_valid(false) {}
};

class AD7147 : public TouchSensor {
public:
    AD7147(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr);
    ~AD7147() override;
    
    // TouchSensor接口实现
    uint32_t getSupportedChannelCount() const override;
    bool init() override;
    void deinit() override;
    bool isInitialized() const override;
    bool setChannelSensitivity(uint8_t channel, uint8_t sensitivity) override;  // 设置通道灵敏度 (0-99)
    TouchSampleResult sample() override; // 统一采样接口
    bool setChannelEnabled(uint8_t channel, bool enabled) override;    // 设置单个通道使能
    bool getChannelEnabled(uint8_t channel) const override;            // 获取单个通道使能状态
    uint32_t getEnabledChannelMask() const override;                   // 获取启用通道掩码
    
    // AD7147特有接口
    bool read_device_info(AD7147_DeviceInfo& info);
    
private:
    // 硬件接口
    HAL_I2C* i2c_hal_;
    I2C_Bus i2c_bus_;
    uint8_t device_addr_;                    // AD7147设备地址
    uint8_t i2c_device_address_;             // 实际I2C设备地址
    
    // 设备状态
    bool initialized_;
    I2C_Bus i2c_bus_enum_;                   // I2C总线枚举
    uint32_t enabled_channels_mask_;         // 启用的通道掩码
    mutable uint32_t last_touch_state_;      // 最后一次触摸状态缓存

    // 将启用通道掩码实时下发到硬件，使对应Stage的校准/中断启用或关闭
    bool applyEnabledChannelsToHardware();
    bool configureStages(uint16_t power_control_val, const uint16_t* connection_values);

    // 内部辅助函数（16位寄存器地址 + 16位数据）
    bool write_register(uint16_t reg, uint8_t* value, uint16_t size = 2);
    bool read_register(uint16_t reg, uint16_t& value);
};