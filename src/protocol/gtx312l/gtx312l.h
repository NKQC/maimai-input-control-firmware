#pragma once

#include "../../hal/i2c/hal_i2c.h"
#include <stdint.h>
#include <vector>
#include <functional>
#include <cstring>

/**
 * 协议层 - GTX312L触摸控制器
 * 基于I2C通信的12通道电容式触摸控制器
 * 支持触摸按键检测，无坐标输出
 * 工作电压：1.8V-5.5V，支持I2C接口
 */

// GTX312L I2C地址范围（可配置）
#define GTX312L_I2C_ADDR_MIN        0x28    // 最小I2C地址
#define GTX312L_I2C_ADDR_MAX        0x2F    // 最大I2C地址
#define GTX312L_I2C_ADDR_DEFAULT    0x28    // 默认I2C地址

// GTX312L寄存器定义（基于官方datasheet）
#define GTX312L_REG_CHIP_ID         0x00    // 芯片ID寄存器
#define GTX312L_REG_FIRMWARE_VER    0x01    // 固件版本寄存器
#define GTX312L_REG_TOUCH_STATUS_L  0x02    // 触摸状态寄存器低字节（通道1-8）
#define GTX312L_REG_TOUCH_STATUS_H  0x03    // 触摸状态寄存器高字节（通道9-12）
#define GTX312L_REG_CH_ENABLE_L     0x04    // 通道使能寄存器低字节（通道1-8）
#define GTX312L_REG_CH_ENABLE_H     0x05    // 通道使能寄存器高字节（通道9-12）
#define GTX312L_REG_MON_RST         0x0A    // 监控复位寄存器
#define GTX312L_REG_SOFT_RST        0x0B    // 软件复位寄存器
#define GTX312L_REG_I2C_PU_DIS      0x0C    // I2C上拉禁用寄存器
#define GTX312L_REG_WRITE_LOCK      0x0F    // 寄存器写保护锁
#define GTX312L_REG_INT_MODE        0x10    // 中断模式配置寄存器
#define GTX312L_REG_EXP_CONFIG      0x11    // 扩展配置寄存器
#define GTX312L_REG_CAL_TIME        0x13    // 校准时间配置寄存器
#define GTX312L_REG_SEN_IDLE_TIME   0x14    // 感应空闲时间寄存器
#define GTX312L_REG_SEN_IDLE_SUFFIX 0x15    // 感应空闲时间后缀寄存器
#define GTX312L_REG_BUSY_TO_IDLE    0x17    // 忙碌到空闲时间寄存器
#define GTX312L_REG_I2B_MODE        0x18    // I2B模式寄存器
#define GTX312L_REG_SLIDE_MODE      0x19    // 滑动模式寄存器
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
#define GTX312L_REG_GLOBAL_CONFIG_1 0x2C    // 全局配置寄存器1
#define GTX312L_REG_GLOBAL_CONFIG_2 0x2D    // 全局配置寄存器2

// 触摸通道相关常量
#define GTX312L_MAX_CHANNELS        12      // 最大触摸通道数
#define GTX312L_CHIP_ID_VALUE       0xB6B2  // 预期的芯片ID值（根据datasheet）

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

// 配置结构
struct GTX312L_Config {
    uint8_t sensitivity[GTX312L_MAX_CHANNELS];     // 每个通道的灵敏度 (0-63)
    uint8_t channel_enable_mask_l;                 // 通道1-8使能掩码
    uint8_t channel_enable_mask_h;                 // 通道9-12使能掩码
    uint8_t cal_time;                              // 校准时间配置 (0-15)
    uint8_t sen_idle_time;                         // 感应空闲时间 (0-15)
    uint8_t sen_idle_time_suffix;                  // 感应空闲时间后缀 (0-15)
    uint8_t busy_to_idle_time;                     // 忙碌到空闲时间 (0-7)
    bool interrupt_enable;                         // 中断使能
    bool multi_touch_enable;                       // 多点触摸使能
    bool exp_enable;                               // 扩展功能使能
    bool i2c_pullup_disable;                       // I2C上拉禁用
    
    GTX312L_Config() {
        // 初始化所有通道灵敏度为默认值
        for (int i = 0; i < GTX312L_MAX_CHANNELS; i++) {
            sensitivity[i] = GTX312L_SENSITIVITY_DEFAULT;
        }
        
        channel_enable_mask_l = GTX312L_CH_ENABLE_ALL_L;  // 默认启用所有通道
        channel_enable_mask_h = GTX312L_CH_ENABLE_ALL_H;
        cal_time = 0x0A;                    // 默认校准时间
        sen_idle_time = 0x00;               // 默认感应空闲时间
        sen_idle_time_suffix = 0x01;        // 默认感应空闲时间后缀
        busy_to_idle_time = 0x03;           // 默认忙碌到空闲时间
        interrupt_enable = false;           // 默认关闭中断
        multi_touch_enable = true;          // 默认启用多点触摸
        exp_enable = false;                 // 默认关闭扩展功能
        i2c_pullup_disable = false;         // 默认启用I2C上拉
    }
};

// 设备信息结构
struct GTX312L_DeviceInfo {
    uint16_t chip_id;
    uint8_t firmware_version;
    uint8_t i2c_address;
    bool is_valid;
    
    GTX312L_DeviceInfo() : chip_id(0), firmware_version(0), i2c_address(0), is_valid(false) {}
};

// 触摸回调函数类型
typedef std::function<void(uint8_t device_index, const GTX312L_TouchData& touch_data)> GTX312L_TouchCallback;

class GTX312L {
public:
    GTX312L(HAL_I2C* i2c_hal, uint8_t device_address, const std::string& device_name = "");
    ~GTX312L();
    
    // 初始化和释放
    bool init();
    void deinit();
    bool is_ready() const;
    
    // 设备信息
    bool read_device_info(GTX312L_DeviceInfo& info);
    std::string get_device_name() const;
    uint8_t get_device_address() const;
    
    // 触摸数据读取
    bool read_touch_data(GTX312L_TouchData& touch_data);
    
    // 配置管理
    bool set_config(const GTX312L_Config& config);
    bool get_config(GTX312L_Config& config);
    
    // 单独设置
    bool set_global_sensitivity(uint8_t sensitivity);                   // 设置全局灵敏度（应用到所有通道）
    bool set_channel_sensitivity(uint8_t channel, uint8_t sensitivity); // 设置单个通道灵敏度
    bool set_channel_enable(uint8_t channel, bool enabled);            // 设置单个通道使能状态
    bool set_all_channels_enable(bool enabled);                        // 设置所有通道使能状态
    bool set_multi_touch_mode(bool enabled);                           // 设置多点触摸模式
    bool set_interrupt_mode(bool enabled);                             // 设置中断模式
    
    // 单独获取 - 实时从设备读取
    uint8_t get_global_sensitivity() const;                            // 获取全局灵敏度（从第一个通道读取）
    uint8_t get_channel_sensitivity(uint8_t channel) const;            // 获取单个通道灵敏度
    bool get_channel_enable(uint8_t channel) const;                    // 获取单个通道使能状态
    
    // 校准和控制
    bool calibrate();                   // 手动校准
    bool reset();                       // 复位设备
    bool enter_sleep();                 // 进入睡眠模式
    bool wakeup();                      // 唤醒设备
    
    // 中断处理（默认关闭）
    void set_touch_callback(GTX312L_TouchCallback callback, uint8_t device_index);
    void handle_interrupt();
    
    // 任务处理（轮询模式）
    void task();
    
    // 静态方法：I2C总线扫描和设备发现
    static std::vector<uint8_t> scan_i2c_bus(HAL_I2C* i2c_hal);
    static std::vector<GTX312L*> discover_devices(HAL_I2C* i2c_hal, const std::string& name_prefix = "GTX312L");
    static void cleanup_devices(std::vector<GTX312L*>& devices);
    
private:
    HAL_I2C* i2c_hal_;
    uint8_t device_address_;
    std::string device_name_;
    bool initialized_;
    
    // 配置数据
    GTX312L_Config config_;
    
    // 回调函数
    GTX312L_TouchCallback touch_callback_;
    uint8_t device_index_;
    
    // 内部方法
    bool write_register(uint8_t reg_addr, uint8_t data);
    bool read_register(uint8_t reg_addr, uint8_t& data);
    bool read_registers(uint8_t reg_addr, uint8_t* data, uint8_t length);

    // 数据解析
    void parse_touch_data(const uint8_t* raw_data, GTX312L_TouchData& touch_data);
    
    // 设备检测
    static bool is_gtx312l_device(HAL_I2C* i2c_hal, uint8_t address);
};