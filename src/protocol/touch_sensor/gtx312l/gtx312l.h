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

// 16位物理地址掩码和反馈结构体 - 使用union和位域
union GTX312L_PhysicalAddr {
    struct {
        uint16_t CH0: 1;     // 通道0状态
        uint16_t CH1: 1;     // 通道1状态
        uint16_t CH2: 1;     // 通道2状态
        uint16_t CH3: 1;     // 通道3状态
        uint16_t CH4: 1;     // 通道4状态
        uint16_t CH5: 1;     // 通道5状态
        uint16_t CH6: 1;     // 通道6状态
        uint16_t CH7: 1;     // 通道7状态
        uint16_t CH8: 1;     // 通道8状态
        uint16_t CH9: 1;     // 通道9状态
        uint16_t CH10: 1;    // 通道10状态
        uint16_t CH11: 1;    // 通道11状态
        uint16_t addr: 2;    // GTX312L设备地址 (13-12位)
        uint16_t i2c_port: 2; // I2C端口 (15-14位)
    };
    uint16_t mask;           // 完整的16位掩码
    
    // 构造函数
    GTX312L_PhysicalAddr(uint8_t i2c_addr = 0, uint8_t gtx_addr = 0, uint16_t channel_bitmap = 0) {
        mask = ((uint16_t)(i2c_addr & 0x03) << 14) | 
               ((uint16_t)(gtx_addr & 0x03) << 12) | 
               (channel_bitmap & 0x0FFF);
    }
    
    // 获取设备掩码 (通道bitmap为0时的地址)
    uint16_t get_device_mask() const {
        return mask & 0xF000;
    }
};

union GTX312L_PortEnableBitmap {
    struct {
        uint16_t CH0: 1;     // 通道0状态
        uint16_t CH1: 1;     // 通道1状态
        uint16_t CH2: 1;     // 通道2状态
        uint16_t CH3: 1;     // 通道3状态
        uint16_t CH4: 1;     // 通道4状态
        uint16_t CH5: 1;     // 通道5状态
        uint16_t CH6: 1;     // 通道6状态
        uint16_t CH7: 1;     // 通道7状态
        uint16_t CH8: 1;     // 通道8状态
        uint16_t CH9: 1;     // 通道9状态
        uint16_t CH10: 1;    // 通道10状态
        uint16_t CH11: 1;    // 通道11状态
    } port_enable;
    uint16_t value;
};

// GTX312L采样结果结构体
struct GTX312L_SampleResult {
    GTX312L_PhysicalAddr physical_addr;  // 包含设备地址和触摸bitmap的完整16位数据
    millis_t timestamp;                  // 时间戳
    
    // 默认构造函数
    GTX312L_SampleResult() : physical_addr(0, 0, 0), timestamp(0) {}
    
    GTX312L_SampleResult(uint16_t device_mask, uint16_t touch_bitmap) 
        : physical_addr((device_mask | (touch_bitmap & 0x0FFF))), timestamp(0) {}
};

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

// 触摸数据结构（12个通道的状态）
struct GTX312L_TouchData {
    uint16_t touch_status;                      // 12位触摸状态（位0-11对应通道0-11）
    uint32_t timestamp;                         // 时间戳
    bool valid;                                 // 数据是否有效
    
    GTX312L_TouchData() : touch_status(0), timestamp(0), valid(false) {
    }
    
    // 检查指定通道是否被触摸
    bool is_channel_touched(uint8_t channel) const {
        if (channel >= GTX312L_MAX_CHANNELS) return false;
        return (touch_status & (1 << channel)) != 0;
    }
    
    // 获取被触摸的通道数量
    uint8_t get_touched_count() const {
        uint8_t count = 0;
        for (int i = 0; i < GTX312L_MAX_CHANNELS; i++) {
            if (touch_status & (1 << i)) count++;
        }
        return count;
    }
};

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

// 触摸回调函数类型
typedef std::function<void(uint8_t device_index, const GTX312L_TouchData& touch_data)> GTX312L_TouchCallback;

class GTX312L : public TouchSensor {
public:
    GTX312L(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr);
    ~GTX312L() override;
    
    // 初始化和清理
    // TouchSensor接口实现
    uint32_t getEnabledModuleMask() const override;
    uint32_t getCurrentTouchState() const override;
    uint32_t getSupportedChannelCount() const override;
    uint32_t getModuleIdMask() const override;
    bool init() override;
    void deinit() override;
    std::string getDeviceName() const override;
    bool isInitialized() const override;
    
    // 物理地址相关
    GTX312L_PhysicalAddr get_physical_device_address() const;  // 获取16位物理设备地址（通道bitmap为0）
    
    // 触摸数据读取 - 返回设备地址+采样bitmap
    GTX312L_SampleResult sample_touch_data();  // 高效采样接口
    
    // 设备信息
    bool read_device_info(GTX312L_DeviceInfo& info);
    // GTX312L特有接口
    std::string get_device_name() const;  // 保持向后兼容
    
    // 核心功能接口
    bool set_channel_enable(uint8_t channel, bool enabled);            // 设置单个通道使能状态
    bool set_sensitivity(uint8_t channel, uint8_t sensitivity);        // 设置通道灵敏度
    
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
    GTX312L_PhysicalAddr physical_device_address_;  // 16位物理设备地址
    
    // 设备状态
    bool initialized_;
    uint8_t module_id_;                      // 模块ID（0-15）
    uint32_t enabled_channels_mask_;         // 启用的通道掩码
    mutable uint32_t last_touch_state_;      // 最后一次触摸状态缓存
    
    // 内部辅助函数
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool write_registers(uint8_t reg, const uint8_t* data, size_t length);
    bool read_registers(uint8_t reg, uint8_t* data, size_t length);
};