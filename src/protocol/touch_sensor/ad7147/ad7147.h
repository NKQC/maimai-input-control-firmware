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
#define AD7147_I2C_ADDR_DEFAULT 0x2C // 默认I2C地址
#define AD7147_MAX_CHANNELS 12       // 最大触摸通道数 这里只考虑支持最多同时触发的点数 13点那个不考虑

// 主要寄存器地址
#define AD7147_REG_PWR_CONTROL 0x0000           // 电源控制寄存器
#define AD7147_REG_STAGE_CAL_EN 0x0001          // 阶段校准使能寄存器
#define AD7147_REG_AMB_COMP_CTRL0 0x0002        // 环境补偿控制寄存器0
#define AD7147_REG_AMB_COMP_CTRL1 0x0003        // 环境补偿控制寄存器1
#define AD7147_REG_AMB_COMP_CTRL2 0x0004        // 环境补偿控制寄存器2
#define AD7147_REG_STAGE_LOW_INT_EN 0x0005      // 阶段低中断使能寄存器
#define AD7147_REG_STAGE_HIGH_INT_EN 0x0006     // 阶段高中断使能寄存器
#define AD7147_REG_STAGE_COMPLETE_INT_EN 0x0007 // 阶段完成中断使能寄存器

// 状态寄存器（读取将清除已置位的状态位）
#define AD7147_REG_STAGE_LOW_INT_STATUS 0x0008      // 阶段低中断状态寄存器
#define AD7147_REG_STAGE_HIGH_INT_STATUS 0x0009     // 阶段高中断状态寄存器
#define AD7147_REG_STAGE_COMPLETE_INT_STATUS 0x000A // 阶段完成中断状态寄存器

// CDC数据
#define AD7147_REG_CDC_DATA 0x000B // CDC数据寄存器

// 阈值寄存器
#define STAGE1_HIGH_THRESHOLD 0x11E
#define STAGE1_LOW_THRESHOLD 0x125
#define STAGE2_HIGH_THRESHOLD 0x142
#define STAGE2_LOW_THRESHOLD 0x149

// 阈值寄存器偏移 不同阶段的同名寄存器偏移
#define STAGE_THRESHOLD_OFFSET 0x24

// Stage配置寄存器基地址（每个stage占用8个16位寄存器）
#define AD7147_REG_STAGE0_CONNECTION 0x0080 // Stage 0连接寄存器
#define AD7147_REG_STAGE_SIZE 8             // 每个stage占用的寄存器数量

#define AD7147_REG_DEVICE_ID 0x0017 // 设备ID寄存器

// Stage寄存器偏移（相对于STAGEx_CONNECTION基地址）
#define AD7147_STAGE_CONNECTION_OFFSET 0        // 连接配置寄存器偏移
#define AD7147_STAGE_AFE_OFFSET_OFFSET 2        // AFE偏移寄存器偏移
#define AD7147_STAGE_SENSITIVITY_OFFSET 3       // 灵敏度寄存器偏移
#define AD7147_STAGE_OFFSET_LOW_OFFSET 4        // 低偏移寄存器偏移
#define AD7147_STAGE_OFFSET_HIGH_OFFSET 5       // 高偏移寄存器偏移
#define AD7147_STAGE_OFFSET_HIGH_CLAMP_OFFSET 6 // 高偏移钳位寄存器偏移
#define AD7147_STAGE_OFFSET_LOW_CLAMP_OFFSET 7  // 低偏移钳位寄存器偏移

// 灵敏度寄存器默认值
#define AD7147_SENSITIVITY_DEFAULT 0x0F0F // 默认灵敏度值
#define AD7147_DEFAULT_AFE_OFFSET 0x0000  // AFE偏移默认值

// 校准设置
#define AD7147_STAGE_CAL_EN 0x0FFF // 阶段校准使能寄存器 默认0x0000 0x0FFF启动校准

// 阶段配置相关常量
#define AD7147_STAGE1_CONNECTION 0x0088  // Stage 1连接寄存器
#define AD7147_STAGE2_CONNECTION 0x0090  // Stage 2连接寄存器
#define AD7147_STAGE3_CONNECTION 0x0098  // Stage 3连接寄存器
#define AD7147_STAGE4_CONNECTION 0x00A0  // Stage 4连接寄存器
#define AD7147_STAGE5_CONNECTION 0x00A8  // Stage 5连接寄存器
#define AD7147_STAGE6_CONNECTION 0x00B0  // Stage 6连接寄存器
#define AD7147_STAGE7_CONNECTION 0x00B8  // Stage 7连接寄存器
#define AD7147_STAGE8_CONNECTION 0x00C0  // Stage 8连接寄存器
#define AD7147_STAGE9_CONNECTION 0x00C8  // Stage 9连接寄存器
#define AD7147_STAGE10_CONNECTION 0x00D0 // Stage 10连接寄存器
#define AD7147_STAGE11_CONNECTION 0x00D8 // Stage 11连接寄存器

// 阶段配置默认值
#define AD7147_DEFAULT_OFFSET_LOW 0x3000        // 默认低偏移值
#define AD7147_DEFAULT_OFFSET_HIGH 0x3000       // 默认高偏移值
#define AD7147_DEFAULT_OFFSET_LOW_CLAMP 0x3100  // 默认低偏移钳位值
#define AD7147_DEFAULT_OFFSET_HIGH_CLAMP 0x3100 // 默认高偏移钳位值
#define AD7147_CDC_BASELINE 0x8000              // CDC基准值，用于显示计算

#define CALIBRATION_STAGE1_SCAN_RANGEA -5 // A -> B
#define CALIBRATION_STAGE1_SCAN_RANGEB -127
#define CALIBRATION_SCAN_SAMPLE_COUNT 300 // 自动校准单轮采样次数
#define CALIBRATION_MEASURE_SAMPLE_COUNT 3000
#define CALIBRATION_AEF_SAVE_AREA -1 // AEF完成时额外偏置保留区域 预留缓冲空间防止意外触发

// 反向指数算法参数宏定义
#define FLUCTUATION_MIN_THRESHOLD 800   // 波动最小阈值
#define FLUCTUATION_MAX_THRESHOLD 6000 // 波动最大阈值
#define FLUCTUATION_MAX_FACTOR (1 / 2)        // 最大调整系数
#define FLUCTUATION_MIN_FACTOR 5        // 最小调整系数（除数）
#define TAYLOR_SCALE_FACTOR 1024        // 泰勒级数缩放因子
#define TAYLOR_K_DIVISOR 2              // K因子除数
#define TAYLOR_NORMALIZATION_RANGE 4950 // 归一化范围

#define AD7147_CALIBRATION_TARGET_VALUE (AD7147_CDC_BASELINE - AD7147_DEFAULT_OFFSET_HIGH_CLAMP - FLUCTUATION_MIN_THRESHOLD)

// 设备信息结构体
struct AD7147_DeviceInfo
{
    uint8_t i2c_address;
    bool is_valid;

    AD7147_DeviceInfo() : i2c_address(0), is_valid(false) {}
};

// 通道连接配置定义（对应CIN0-CIN11）
const uint16_t channel_connections[12][2] = {
    // POSTIVE
    // BIAS版 显然不适合mai2情况 天线效应会干碎BIAS
    // {0xFFFE, 0x1FFF}, // Stage 0 - CIN0
    // {0xFFFB, 0x1FFF}, // Stage 1 - CIN1
    // {0xFFEF, 0x1FFF}, // Stage 2 - CIN2
    // {0xFFBF, 0x1FFF}, // Stage 3 - CIN3
    // {0xFEFF, 0x1FFF}, // Stage 4 - CIN4
    // {0xFBFF, 0x1FFF}, // Stage 5 - CIN5
    // {0xEFFF, 0x1FFF}, // Stage 6 - CIN6
    // {0xFFFF, 0x1FFE}, // Stage 7 - CIN7
    // {0xEFFF, 0x1FFB}, // Stage 8 - CIN8
    // {0xEFFF, 0x1FEF}, // Stage 9 - CIN9
    // {0xEFFF, 0x1FBF}, // Stage 10 - CIN10
    // {0xEFFF, 0x1EFF}  // Stage 11 - CIN11
    // 高阻版
    // {0x0002, 0x1000}, // Stage 0 - CIN0
    // {0x0008, 0x1000}, // Stage 1 - CIN1
    // {0x0020, 0x1000}, // Stage 2 - CIN2
    // {0x0080, 0x1000}, // Stage 3 - CIN3
    // {0x0200, 0x1000}, // Stage 4 - CIN4
    // {0x0800, 0x1000}, // Stage 5 - CIN5
    // {0x2000, 0x1000}, // Stage 6 - CIN6
    // {0x0000, 0x1002}, // Stage 7 - CIN7
    // {0x0000, 0x1008}, // Stage 8 - CIN8
    // {0x0000, 0x1020}, // Stage 9 - CIN9
    // {0x0000, 0x1080}, // Stage 10 - CIN10
    // {0x0000, 0x1200}  // Stage 11 - CIN11
    // NEGTIVE
    {0x0001, 0x1000}, // Stage 0 - CIN0
    {0x0004, 0x1000}, // Stage 1 - CIN1
    {0x0010, 0x1000}, // Stage 2 - CIN2
    {0x0040, 0x1000}, // Stage 3 - CIN3
    {0x0100, 0x1000}, // Stage 4 - CIN4
    {0x0400, 0x1000}, // Stage 5 - CIN5
    {0x1000, 0x1000}, // Stage 6 - CIN6
    {0x0000, 0x1001}, // Stage 7 - CIN7
    {0x0000, 0x1004}, // Stage 8 - CIN8
    {0x0000, 0x1010}, // Stage 9 - CIN9
    {0x0000, 0x1040}, // Stage 10 - CIN10
    {0x0000, 0x1100}  // Stage 11 - CIN11
};

// AFE偏移寄存器位域结构
union AFEOffsetRegister
{
    uint16_t raw; // 原始16位值
    struct
    {
        uint16_t neg_afe_offset : 6;      // [5:0] 负AFE偏移设置 (20 pF范围, 1 LSB = 0.32 pF)
        uint16_t unused1 : 1;             // [6] 未使用，设为0
        uint16_t neg_afe_offset_swap : 1; // [7] 负AFE偏移交换控制
        uint16_t pos_afe_offset : 6;      // [13:8] 正AFE偏移设置 (20 pF范围, 1 LSB = 0.32 pF)
        uint16_t unused2 : 1;             // [14] 未使用，设为0
        uint16_t pos_afe_offset_swap : 1; // [15] 正AFE偏移交换控制
    } bits;

    AFEOffsetRegister() : raw(AD7147_DEFAULT_AFE_OFFSET) {}
    AFEOffsetRegister(uint16_t value) : raw(value) {}
};

// 灵敏度寄存器位域结构
union SensitivityRegister
{
    uint16_t raw; // 原始16位值
    struct
    {
        uint16_t neg_threshold_sensitivity : 4; // [3:0] 负阈值灵敏度控制
        uint16_t neg_peak_detect : 3;           // [6:4] 负峰值检测设置
        uint16_t unused1 : 1;                   // [7] 未使用，设为0
        uint16_t pos_threshold_sensitivity : 4; // [11:8] 正阈值灵敏度控制
        uint16_t pos_peak_detect : 3;           // [14:12] 正峰值检测设置
        uint16_t unused2 : 1;                   // [15] 未使用，设为0
    } bits;

    SensitivityRegister() : raw(AD7147_SENSITIVITY_DEFAULT) {}
    SensitivityRegister(uint16_t value) : raw(value) {}
};

// Stage配置结构体 - 包含单个stage的所有配置参数
struct PortConfig
{
    uint16_t connection_6_0;         // 连接配置寄存器6-0
    uint16_t connection_12_7;        // 连接配置寄存器12-7
    AFEOffsetRegister afe_offset;    // AFE偏移值（位域结构）
    SensitivityRegister sensitivity; // 灵敏度值（位域结构）
    uint16_t offset_low;             // 低偏移值
    uint16_t offset_high;            // 高偏移值
    uint16_t offset_high_clamp;      // 高偏移钳位值
    uint16_t offset_low_clamp;       // 低偏移钳位值

    PortConfig() : connection_6_0(0), connection_12_7(0), afe_offset(AD7147_DEFAULT_AFE_OFFSET),
                   sensitivity(AD7147_SENSITIVITY_DEFAULT), offset_low(AD7147_DEFAULT_OFFSET_LOW),
                   offset_high(AD7147_DEFAULT_OFFSET_HIGH), offset_high_clamp(AD7147_DEFAULT_OFFSET_HIGH_CLAMP),
                   offset_low_clamp(AD7147_DEFAULT_OFFSET_LOW_CLAMP) {}
};

// 全阶段设置结构体 - 包含所有12个stage的配置
struct StageSettings
{
    PortConfig stages[12]; // 12个stage的配置

    StageSettings()
    {
        for (int32_t i = 0; i < 12; i++)
        {
            stages[i].connection_6_0 = channel_connections[i][0];
            stages[i].connection_12_7 = channel_connections[i][1];
        }
    }
};

// 寄存器结构体定义
// PWR_CONTROL寄存器 (0x000)
union PWRControlRegister
{
    uint16_t raw;
    struct
    {
        uint16_t power_mode : 2;         // [1:0] 工作模式
        uint16_t lp_conv_delay : 2;      // [3:2] 低功耗模式转换延迟
        uint16_t sequence_stage_num : 4; // [7:4] 序列中的阶段数量 (N+1)
        uint16_t decimation : 2;         // [9:8] ADC抽取因子
        uint16_t sw_reset : 1;           // [10] 软件复位控制
        uint16_t int_pol : 1;            // [11] 中断极性控制
        uint16_t ext_source : 1;         // [12] 激励源控制
        uint16_t unused : 1;             // [13] 未使用，设为0
        uint16_t cdc_bias : 2;           // [15:14] CDC偏置电流控制
    } bits;

    PWRControlRegister() : raw(0x12F0) {} // 默认值
};

// STAGE_CAL_EN寄存器 (0x001)
union StageCalEnRegister
{
    uint16_t raw;
    struct
    {
        uint16_t stage0_cal_en : 1;  // [0] STAGE0校准使能
        uint16_t stage1_cal_en : 1;  // [1] STAGE1校准使能
        uint16_t stage2_cal_en : 1;  // [2] STAGE2校准使能
        uint16_t stage3_cal_en : 1;  // [3] STAGE3校准使能
        uint16_t stage4_cal_en : 1;  // [4] STAGE4校准使能
        uint16_t stage5_cal_en : 1;  // [5] STAGE5校准使能
        uint16_t stage6_cal_en : 1;  // [6] STAGE6校准使能
        uint16_t stage7_cal_en : 1;  // [7] STAGE7校准使能
        uint16_t stage8_cal_en : 1;  // [8] STAGE8校准使能
        uint16_t stage9_cal_en : 1;  // [9] STAGE9校准使能
        uint16_t stage10_cal_en : 1; // [10] STAGE10校准使能
        uint16_t stage11_cal_en : 1; // [11] STAGE11校准使能
        uint16_t avg_fp_skip : 2;    // [13:12] 全功率模式跳过控制
        uint16_t avg_lp_skip : 2;    // [15:14] 低功率模式跳过控制
    } bits;

    StageCalEnRegister() : raw(0x0000) {} // 默认值
};

// AMB_COMP_CTRL0寄存器 (0x002)
union AmbCompCtrl0Register
{
    uint16_t raw;
    struct
    {
        uint16_t ff_skip_cnt : 4;      // [3:0] 快速滤波器跳过控制
        uint16_t fp_proximity_cnt : 4; // [7:4] 全功率模式接近计数
        uint16_t lp_proximity_cnt : 4; // [11:8] 低功率模式接近计数
        uint16_t pwr_down_timeout : 2; // [13:12] 全功率到低功率模式超时控制
        uint16_t forced_cal : 1;       // [14] 强制校准控制
        uint16_t conv_reset : 1;       // [15] 转换复位控制
    } bits;

    AmbCompCtrl0Register() : raw(0x00FF) {} // 默认值
};

// AMB_COMP_CTRL1寄存器 (0x003)
union AmbCompCtrl1Register
{
    uint16_t raw;
    struct
    {
        uint16_t proximity_recal_lvl : 8;      // [7:0] 接近重新校准级别
        uint16_t proximity_detection_rate : 6; // [13:8] 接近检测速率
        uint16_t slow_filter_update_lvl : 2;   // [15:14] 慢滤波器更新级别
    } bits;

    AmbCompCtrl1Register() : raw(0x0040) {} // 默认值 (64)
};

// AMB_COMP_CTRL2寄存器 (0x004)
union AmbCompCtrl2Register
{
    uint16_t raw;
    struct
    {
        uint16_t fp_proximity_recal : 10; // [9:0] 全功率模式接近重新校准时间控制
        uint16_t lp_proximity_recal : 6;  // [15:10] 低功率模式接近重新校准时间控制
    } bits;

    AmbCompCtrl2Register() : raw(0xFFFF) {} // 默认值 (0x3FF | (0x3F << 10))
};

// 寄存器配置结构体
struct AD7147RegisterConfig
{
    PWRControlRegister pwr_control;      // 0x000 电源控制
    StageCalEnRegister stage_cal_en;     // 0x001 阶段校准使能
    AmbCompCtrl0Register amb_comp_ctrl0; // 0x002 环境补偿控制0
    AmbCompCtrl1Register amb_comp_ctrl1; // 0x003 环境补偿控制1
    AmbCompCtrl2Register amb_comp_ctrl2; // 0x004 环境补偿控制2
    uint16_t stage_low_int_enable;       // 0x005 阶段低中断使能
    uint16_t stage_high_int_enable;      // 0x006 阶段高中断使能
    uint16_t stage_complete_int_enable;  // 0x007 阶段完成中断使能

    AD7147RegisterConfig() : stage_low_int_enable(0x0FFF),     // 默认值
                             stage_high_int_enable(0x0000),    // 默认值
                             stage_complete_int_enable(0x0000) // 默认值
    {
    }
};

class AD7147 : public TouchSensor
{
public:
    AD7147(HAL_I2C *i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr);
    ~AD7147() noexcept override;

    // TouchSensor接口实现
    uint32_t getSupportedChannelCount() const override;
    bool init() override;
    void deinit() override;
    bool isInitialized() const override;
    bool setChannelSensitivity(uint8_t channel, uint8_t sensitivity) override; // 设置通道灵敏度 (0-99)
    TouchSampleResult sample() override;                                       // 统一采样接口
    bool setChannelEnabled(uint8_t channel, bool enabled) override;            // 设置单个通道使能
    bool getChannelEnabled(uint8_t channel) const override;                    // 获取单个通道使能状态
    uint32_t getEnabledChannelMask() const override;                           // 获取启用通道掩码

    // 配置管理接口实现
    bool loadConfig(const std::string &config_data) override;            // 从字符串加载配置
    std::string saveConfig() const override;                             // 保存配置到字符串
    bool setCustomSensitivitySettings(const std::string &settings_data); // 设置自定义灵敏度配置
    bool setStageConfig(uint8_t stage, const PortConfig &config);        // 设置指定阶段配置 只能在同核心调用 否则秒崩
    bool setStageConfigAsync(uint8_t stage, const PortConfig &config);   // 异步设置指定阶段配置
    PortConfig getStageConfig(uint8_t stage) const;                      // 获取指定阶段配置副本
    bool readStageCDC(uint8_t stage, uint16_t &cdc_value);               // 读取指定阶段CDC值

    // 校准相关接口实现
    bool calibrateSensor() override;                           // 校准传感器(接入startAutoOffsetCalibration)
    bool calibrateSensor(uint8_t sensitivity_target) override; // 校准传感器(带灵敏度目标)
    bool setChannelCalibrationTarget(uint8_t channel, uint8_t sensitivity_target) override; // 设置指定通道的校准目标灵敏度
    bool startCalibration() override;                          // 启动校准过程
    uint8_t getCalibrationProgress() const override;           // 获取校准进度(接入getAutoOffsetCalibrationTotalProgress)
    bool setLEDEnabled(bool enabled) override;                 // 设置LED状态(修改stage_low_int_enable的第12-13位)

    // 异常通道检测接口实现
    uint32_t getAbnormalChannelMask() const override; // 获取异常通道bitmap (返回abnormal_channels_bitmap_)

    // 自动校准控制接口重写
    void setAutoCalibration(bool enable) override; // 控制自动校准启停

    // 自动偏移校准：供UI调用
    bool startAutoOffsetCalibration();
    bool isAutoOffsetCalibrationActive() const;
    uint8_t getAutoOffsetCalibrationTotalProgress() const; // 新增：全局总进度

    // 设备信息读取
    bool read_device_info(AD7147_DeviceInfo &info);

private:
    // 硬件相关成员变量
    HAL_I2C *i2c_hal_;
    I2C_Bus i2c_bus_;
    uint8_t device_addr_;        // AD7147设备地址
    uint8_t i2c_device_address_; // 实际I2C设备地址

    // 状态相关成员变量
    bool initialized_;
    I2C_Bus i2c_bus_enum_; // I2C总线枚举
    uint8_t enabled_stage;
    uint32_t enabled_channels_mask_;               // 启用的通道掩码
    uint32_t calirate_save_enabled_channels_mask_; // 校准时保存的启用的通道掩码

    // 配置相关成员变量
    StageSettings stage_settings_;         // Stage配置设置
    AD7147RegisterConfig register_config_; // 寄存器配置

    // 实例级状态变量（原来的静态变量）
    volatile bool cdc_read_request_;  // CDC读取请求标志
    volatile uint8_t cdc_read_stage_; // 请求读取的阶段
    uint16_t cdc_read_value_;         // 读取到的CDC值

    // sample()函数的实例级变量（原来的静态变量）
    TouchSampleResult sample_result_; // 采样结果
    uint16_t status_regs_;            // 状态寄存器值

    // sample()函数中的映射重建临时变量（优化热点函数性能）
    uint32_t reconstructed_mask_; // 重建的通道掩码
    uint16_t stage_status_;       // 反转后的stage状态
    uint8_t stage_index_;         // stage索引计数器
    uint32_t temp_mask_;          // 临时掩码用于位运算
    uint8_t channel_pos_;         // 通道位置

    // 异步配置相关
    struct PendingPortConfig
    {
        uint8_t stage;
        PortConfig config;
    };
    PendingPortConfig pending_configs_;
    uint8_t pending_config_count_; // 待处理配置计数器

    // 校准异常通道记录
    uint16_t abnormal_channels_bitmap_; // 校准时异常通道的bitmap (bit0-11对应channel0-11)

    // 自动校准控制
    volatile int32_t auto_calibration_control_; // 自动校准控制变量 (最高位=执行标志, 低24位=寄存器值)

    // 允许内部校准工具访问私有成员与方法
    friend class CalibrationTools;

    // 私有方法
    bool applyEnabledChannelsToHardware();
    bool configureStages(const uint16_t *connection_values);
    inline bool apply_stage_settings(); // 应用stage设置到硬件
    // 内部快速读取当前stage配置（不做边界检查）
    inline PortConfig getStageConfigInternal(uint8_t stage) const { return stage_settings_.stages[stage]; }

    // 校准相关辅助方法
    void processAutoOffsetCalibration(); // 在sample()中调用，处理自动偏移校准状态机

    // I2C通信方法
    bool write_register(uint16_t reg, uint8_t *value, uint16_t size = 2);
    bool read_register(uint16_t reg, uint16_t &value);
    // 直接读取指定stage的CDC寄存器（内部使用）
    bool readStageCDC_direct(uint8_t stage, uint16_t &cdc_value);

    class CalibrationTools
    {
    public:
        // 校准阶段
        enum CalibrationState
        {
            IDLE, // 初始化/完成时设置回IDLE
            PROCESS,
        };

        enum Direction
        {
            Pos,
            Neg
        };

        struct CDCSample_result
        {
            uint16_t average = 0;
            uint16_t max = 0;
            uint16_t min = 0;
            uint16_t sample_count = 0;
            CDCSample_result() {}
            void clear()
            {
                average = 0;
                max = 0;
                min = 0;
                sample_count = 0;
            }
        };

        struct TriggleSample
        {
            uint32_t triggle_num = 0;
            uint32_t not_triggle_num = 0;
            uint32_t sample_count = 0;
            TriggleSample() {}
            void clear()
            {
                triggle_num = 0;
                not_triggle_num = 0;
                sample_count = 0;
            }
        };

        // 校准状态结构体 - 包含所有校准相关的数据
        // 单个通道的校准数据结构
        struct ChannelCalibrationData
        {
            uint8_t sensitivity_target = 2;        // 灵敏度目标 (1=高敏, 2=默认, 3=低敏)
            bool s1_inited_ = false;               // 初始化状态
            int16_t s1_aef_ = 0;                   // 当前扫描AEF -127..127
            int16_t s1_best_aef_ = 0;              // 阶段1找到的最佳AEF
            CDCSample_result cdc_samples_;         // CDC采样结果
            uint16_t max_fluctuation_ = 0;         // 最大波动差
            TriggleSample trigger_samples_;        // 触发采样结果
        };

        struct CalibrationData
        {
            uint8_t stage_process = 0;
            bool inited_ = false;
            bool global_initialized_ = false;                     // 全局初始化标志，用于一次性初始化所有通道
            ChannelCalibrationData channels[AD7147_MAX_CHANNELS]; // 每个通道的校准数据
        };

        AD7147 *pthis = nullptr;

        CalibrationState calibration_state_ = IDLE;
        CalibrationData calibration_data_; // 校准数据结构体

        bool start_calibration()
        {
            if (calibration_state_ != IDLE)
                return false;
            calibration_state_ = PROCESS;
            return true;
        }

        // 主循环方法
        void CalibrationLoop(uint32_t sample);

        // 工具方法
        // 清空设置到校准所需值
        void Clear_and_prepare_stage_settings();
        // 完成时恢复校准
        void Complete_and_restore_calibration();
        // 直接设置AEF偏移 已内置正负翻转 0-127
        void Set_AEF_Offset(uint8_t stage, int16_t offset);

        // [执行一次采样一次 到目标周期返回True] 读取CDC值 计算平均值 最大值和最小值
        bool Read_CDC_Sample(uint8_t stage, CDCSample_result &result, bool measure);
        // [执行一次采样一次 直接解析sample中的采样数据 sample应当通过外部直接传入循环中的采样结果原始值 到目标周期/验证时为触发 返回True] 读取触发值 计算触发和未触发次数
        bool Read_Triggle_Sample(uint8_t stage, uint32_t sample, TriggleSample &result, bool measure);
    };

    CalibrationTools calibration_tools_;
};