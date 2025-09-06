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
#define AD7147_DEFAULT_AFE_OFFSET            0x00BE      // AFE偏移默认值

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
#define AD7147_DEFAULT_OFFSET_LOW            0x0      // 默认低偏移值
#define AD7147_DEFAULT_OFFSET_HIGH           0x3000      // 默认高偏移值
#define AD7147_DEFAULT_OFFSET_HIGH_CLAMP     0xFFFF     // 默认高偏移钳位值
#define AD7147_DEFAULT_OFFSET_LOW_CLAMP      0x4000     // 默认低偏移钳位值
#define AD7147_CDC_BASELINE                  32767    // CDC基准值，用于显示计算

// 自动偏移校准相关常量
#define AD7147_AUTO_OFFSET_MAX_ITERATIONS    100     // 最大迭代次数
#define AD7147_AUTO_OFFSET_TOLERANCE         100      // CDC值容差范围
#define AD7147_CDC_EXTREME_LOW_THRESHOLD     100      // CDC极值低阈值
#define AD7147_CDC_EXTREME_HIGH_THRESHOLD    65435    // CDC极值高阈值
#define AD7147_CONTINUOUS_NON_TRIGGER_THRESHOLD 10   // 连续未触发阈值
#define AD7147_OFFSET_ADJUSTMENT_STEP        100      // 偏移调整步进
#define AD7147_SENSITIVITY_ADJUSTMENT_STEP   1        // 灵敏度调整步进
#define AD7147_AFE_OFFSET_MAX                63       // AFE偏移最大值
#define AD7147_AFE_OFFSET_MIN                0        // AFE偏移最小值

// 自动偏移校准状态枚举
enum class AutoOffsetState {
    IDLE,                    // 空闲状态
    ADJUSTING_POS_AFE,      // 调整正向AFE偏移
    TESTING_NEG_AFE_REVERSE, // 测试负向AFE反向设置
    ADJUSTING_NEG_AFE,      // 调整负向AFE偏移
    RESET_NEG_AFE_REVERSE,  // 重置负向AFE反向设置
    RESET_POS_AFE_ZERO,     // 重置正向AFE为0
    TESTING_POS_AFE_REVERSE, // 测试正向AFE反向设置
    ADJUSTING_POS_AFE_REVERSE, // 调整正向AFE反向偏移
    FINE_TUNING_TO_BASELINE, // 精细调整到基线
    CHECKING_TRIGGER_STATUS, // 检查触发状态
    ADJUSTING_SENSITIVITY,   // 调整灵敏度
    ADJUSTING_OFFSET_RANGE,  // 调整偏移范围
    ADJUSTING_PEAK_DETECT,   // 调整峰值检测
    COMPLETED,              // 校准完成
    FAILED,                 // 校准失败
    CALIBRATION_ERROR       // 校准错误
};

// 校准策略枚举
enum class CalibrationStrategy {
    EXTREME_TEST,           // 极值测试策略
    BINARY_SEARCH,          // 二分搜索策略
    REVERSE_SEARCH,         // 反向搜索策略
    FINE_TUNE              // 精细调整策略
};

// 自动偏移校准结果枚举
enum class AutoOffsetResult {
    SUCCESS,                // 校准成功
    OUT_OF_RANGE,          // 超出调整范围
    TIMEOUT,               // 超时
    HARDWARE_ERROR,        // 硬件错误
    FAILED                 // 校准失败
};

// 单个通道的偏移校准状态
struct ChannelOffsetCalibration {
    uint8_t stage;                      // 阶段编号
    AutoOffsetState state;              // 当前状态
    uint16_t target_cdc;               // 目标CDC值
    uint16_t current_cdc;              // 当前CDC值
    
    // AFE偏移校准相关
    uint8_t pos_afe_min, pos_afe_max;  // 正AFE偏移搜索范围
    uint8_t neg_afe_min, neg_afe_max;  // 负AFE偏移搜索范围
    bool pos_afe_swap_tried;           // 是否尝试过正AFE交换
    bool neg_afe_swap_tried;           // 是否尝试过负AFE交换
    
    // 采样偏移校准相关
    uint16_t sample_offset_low_min, sample_offset_low_max;   // 低采样偏移搜索范围
    uint16_t sample_offset_high_min, sample_offset_high_max; // 高采样偏移搜索范围
    
    // 灵敏度校准相关
    uint16_t sensitivity_min, sensitivity_max;               // 灵敏度搜索范围
    uint16_t pos_sensitivity_min, pos_sensitivity_max;       // 正灵敏度搜索范围
    uint16_t neg_sensitivity_min, neg_sensitivity_max;       // 负灵敏度搜索范围
    uint8_t pos_peak_detect_min, pos_peak_detect_max;       // 正峰值检测搜索范围
    uint8_t neg_peak_detect_min, neg_peak_detect_max;       // 负峰值检测搜索范围
    
    uint8_t iteration_count;           // 迭代计数
    bool is_triggered;                 // 当前通道是否被触发
    
    // 其他状态跟踪
    CalibrationStrategy current_strategy;  // 当前校准策略
    uint16_t extreme_pos_cdc;             // 正极值CDC测试结果
    uint16_t extreme_neg_cdc;             // 负极值CDC测试结果
    bool extreme_pos_tested;              // 是否已测试正极值
    bool extreme_neg_tested;              // 是否已测试负极值
    bool reverse_direction;               // 是否需要反向调整
    uint16_t last_cdc_value;              // 上次CDC值
    int16_t cdc_trend;                    // CDC变化趋势 (+1上升, -1下降, 0无变化)
    uint8_t stuck_count;                  // 卡住计数器
    
    // 新增字段用于新的校准逻辑
    bool is_cdc_extreme;                  // 当前CDC是否为极值
    uint8_t calibration_step;             // 当前校准步骤
    bool pos_afe_reverse_enabled;         // 正向AFE是否启用反向
    bool neg_afe_reverse_enabled;         // 负向AFE是否启用反向
    uint8_t adjustment_direction;         // 调整方向 (0=正向, 1=负向)
    uint8_t continuous_non_trigger_count; // 连续未触发计数
    uint8_t continuous_non_extreme_count; // 连续非极值计数
    uint8_t baseline_adjustment_step;     // 基线调整步进
    
    // 50周期极值范围采样字段
    uint8_t sample_count;                 // 当前采样计数
    uint16_t sample_max_cdc;              // 采样期间最大CDC值
    uint16_t sample_min_cdc;              // 采样期间最小CDC值
    uint16_t sample_range;                // CDC值范围 (max - min)
    
    ChannelOffsetCalibration() : stage(0), state(AutoOffsetState::IDLE), 
                                target_cdc(AD7147_CDC_BASELINE),
                                current_cdc(0), pos_afe_min(0), pos_afe_max(AD7147_AFE_OFFSET_MAX),
                                neg_afe_min(0), neg_afe_max(AD7147_AFE_OFFSET_MAX), 
                                pos_afe_swap_tried(false), neg_afe_swap_tried(false),
                                sample_offset_low_min(0), sample_offset_low_max(0xFFFF),
                                sample_offset_high_min(0), sample_offset_high_max(0xFFFF),
                                sensitivity_min(0), sensitivity_max(0xFFFF),
                                 pos_sensitivity_min(0), pos_sensitivity_max(0xFFFF),
                                 neg_sensitivity_min(0), neg_sensitivity_max(0xFFFF),
                                pos_peak_detect_min(0), pos_peak_detect_max(7),
                                neg_peak_detect_min(0), neg_peak_detect_max(7),
                                iteration_count(0), is_triggered(false),
                                current_strategy(CalibrationStrategy::EXTREME_TEST),
                                extreme_pos_cdc(0), extreme_neg_cdc(0),
                                extreme_pos_tested(false), extreme_neg_tested(false),
                                reverse_direction(false), last_cdc_value(0),
                                cdc_trend(0), stuck_count(0), is_cdc_extreme(false),
                                calibration_step(0), pos_afe_reverse_enabled(false),
                                neg_afe_reverse_enabled(false), adjustment_direction(0),
                                continuous_non_trigger_count(0), continuous_non_extreme_count(0),
                                baseline_adjustment_step(1), sample_count(0), sample_max_cdc(0),
                                sample_min_cdc(0xFFFF), sample_range(0) {}
};

// 全局偏移校准状态
struct GlobalOffsetCalibration {
    bool is_active;                     // 是否正在进行校准
    uint8_t total_channels;             // 总通道数
    uint8_t current_channel_index;      // 当前校准通道索引
    uint8_t completed_channels;         // 已完成通道数
    uint32_t start_time_ms;            // 开始时间
    ChannelOffsetCalibration channels[AD7147_MAX_CHANNELS]; // 各通道校准状态
    
    void reset_status() {
        for (uint8_t i = 0; i < AD7147_MAX_CHANNELS; i++) {
            channels[i].state = AutoOffsetState::IDLE;
        }
    }

    GlobalOffsetCalibration() : is_active(false), total_channels(0), 
                               current_channel_index(0), completed_channels(0), start_time_ms(0) {}
};

// 设备信息结构体
struct AD7147_DeviceInfo {
    uint8_t i2c_address;
    bool is_valid;
    
    AD7147_DeviceInfo() : i2c_address(0), is_valid(false) {}
};

// AFE偏移寄存器位域结构
union AFEOffsetRegister {
    uint16_t raw;                   // 原始16位值
    struct {
        uint16_t neg_afe_offset : 6;        // [5:0] 负AFE偏移设置 (20 pF范围, 1 LSB = 0.32 pF)
        uint16_t unused1 : 1;               // [6] 未使用，设为0
        uint16_t neg_afe_offset_swap : 1;   // [7] 负AFE偏移交换控制
        uint16_t pos_afe_offset : 6;        // [13:8] 正AFE偏移设置 (20 pF范围, 1 LSB = 0.32 pF)
        uint16_t unused2 : 1;               // [14] 未使用，设为0
        uint16_t pos_afe_offset_swap : 1;   // [15] 正AFE偏移交换控制
    } bits;
    
    AFEOffsetRegister() : raw(AD7147_DEFAULT_AFE_OFFSET) {}
    AFEOffsetRegister(uint16_t value) : raw(value) {}
};

// 灵敏度寄存器位域结构
union SensitivityRegister {
    uint16_t raw;                   // 原始16位值
    struct {
        uint16_t neg_threshold_sensitivity : 4;  // [3:0] 负阈值灵敏度控制
        uint16_t neg_peak_detect : 3;            // [6:4] 负峰值检测设置
        uint16_t unused1 : 1;                   // [7] 未使用，设为0
        uint16_t pos_threshold_sensitivity : 4;  // [11:8] 正阈值灵敏度控制
        uint16_t pos_peak_detect : 3;            // [14:12] 正峰值检测设置
        uint16_t unused2 : 1;                   // [15] 未使用，设为0
    } bits;
    
    SensitivityRegister() : raw(AD7147_SENSITIVITY_DEFAULT) {}
    SensitivityRegister(uint16_t value) : raw(value) {}
};

// Stage配置结构体 - 包含单个stage的所有配置参数
struct StageConfig {
    uint16_t connection_6_0;        // 连接配置寄存器6-0
    uint16_t connection_12_7;       // 连接配置寄存器12-7
    AFEOffsetRegister afe_offset;   // AFE偏移值（位域结构）
    SensitivityRegister sensitivity; // 灵敏度值（位域结构）
    uint16_t offset_low;            // 低偏移值
    uint16_t offset_high;           // 高偏移值
    uint16_t offset_high_clamp;     // 高偏移钳位值
    uint16_t offset_low_clamp;      // 低偏移钳位值
    
    StageConfig() : connection_6_0(0), connection_12_7(0), afe_offset(AD7147_DEFAULT_AFE_OFFSET),
                   sensitivity(AD7147_SENSITIVITY_DEFAULT), offset_low(AD7147_DEFAULT_OFFSET_LOW),
                   offset_high(AD7147_DEFAULT_OFFSET_HIGH), offset_high_clamp(AD7147_DEFAULT_OFFSET_HIGH_CLAMP),
                   offset_low_clamp(AD7147_DEFAULT_OFFSET_LOW_CLAMP) {}
};

// 全阶段设置结构体 - 包含所有12个stage的配置
 struct StageSettings {
     StageConfig stages[12];         // 12个stage的配置
     
    StageSettings() {
        // 初始化阶段连接配置，对应CIN0-CIN11的单端配置
        const uint16_t default_connections[12][2] = {
            {0xFFFE, 0x1FFF}, // Stage 0 - CIN0
            {0xFFFB, 0x1FFF}, // Stage 1 - CIN1  
            {0xFFEF, 0x1FFF}, // Stage 2 - CIN2
            {0xFFBF, 0x1FFF}, // Stage 3 - CIN3
            {0xFEFF, 0x1FFF}, // Stage 4 - CIN4
            {0xFBFF, 0x1FFF}, // Stage 5 - CIN5
            {0xEFFF, 0x1FFF}, // Stage 6 - CIN6
            {0xFFFF, 0x1FFE}, // Stage 7 - CIN7
            {0xEFFF, 0x1FFB}, // Stage 8 - CIN8
            {0xEFFF, 0x1FEF}, // Stage 9 - CIN9
            {0xEFFF, 0x1FBF}, // Stage 10 - CIN10
            {0xEFFF, 0x1EFF}  // Stage 11 - CIN11
        };
        
        for (int i = 0; i < 12; i++) {
            stages[i].connection_6_0 = default_connections[i][0];
            stages[i].connection_12_7 = default_connections[i][1];
        }
    }
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
    
    // 配置管理接口实现
    bool loadConfig(const std::string& config_data) override;          // 从字符串加载配置
    std::string saveConfig() const override;                           // 保存配置到字符串
    bool setCustomSensitivitySettings(const std::string& settings_data); // 设置自定义灵敏度配置
    bool setStageConfig(uint8_t stage, const StageConfig& config);        // 设置指定阶段配置 只能在同核心调用 否则秒崩
    bool setStageConfigAsync(uint8_t stage, const StageConfig& config);   // 异步设置指定阶段配置
    StageConfig getStageConfig(uint8_t stage) const;                     // 获取指定阶段配置副本
    bool readStageCDC(uint8_t stage, uint16_t& cdc_value);              // 读取指定阶段CDC值

    // 自动偏移校准接口
    bool startAutoOffsetCalibration();                                  // 启动自动偏移校准（所有启用通道）
    bool startAutoOffsetCalibrationForStage(uint8_t stage);            // 启动指定阶段的自动偏移校准
    bool isAutoOffsetCalibrationActive() const;                        // 检查是否正在进行自动偏移校准
    AutoOffsetResult getAutoOffsetCalibrationResult(uint8_t stage) const; // 获取指定阶段的校准结果
    uint8_t getAutoOffsetCalibrationProgress() const;                  // 获取校准进度（0-100）
    void stopAutoOffsetCalibration();                                  // 停止自动偏移校准
    
    // 设备信息读取
    bool read_device_info(AD7147_DeviceInfo& info);
    
private:
    // 硬件相关成员变量
    HAL_I2C* i2c_hal_;
    I2C_Bus i2c_bus_;
    uint8_t device_addr_;                    // AD7147设备地址
    uint8_t i2c_device_address_;             // 实际I2C设备地址
    
    // 状态相关成员变量
    bool initialized_;
    I2C_Bus i2c_bus_enum_;                   // I2C总线枚举
    uint32_t enabled_channels_mask_;         // 启用的通道掩码

    // 配置相关成员变量
    StageSettings stage_settings_;           // Stage配置设置
    
    // CDC读取相关
    volatile bool cdc_read_request_;         // CDC读取请求标志
    volatile uint8_t cdc_read_stage_;        // 请求读取的阶段
    uint16_t cdc_read_value_;                // 读取到的CDC值
    
    // 异步配置相关
    struct PendingStageConfig {
        uint8_t stage;
        StageConfig config;
    };
    PendingStageConfig pending_configs_;
    uint8_t pending_config_count_;          // 待处理配置计数器
    
    // 自动偏移校准相关成员变量
    GlobalOffsetCalibration auto_offset_calibration_;  // 全局偏移校准状态

    // 私有方法
    bool applyEnabledChannelsToHardware();
    bool configureStages(uint16_t power_control_val, const uint16_t* connection_values);
    inline bool apply_stage_settings();      // 应用stage设置到硬件
    
    // 自动偏移校准辅助方法
    void processAutoOffsetCalibration();     // 在sample()中调用，处理自动偏移校准状态机
    bool calibrateSingleChannel(ChannelOffsetCalibration& channel_cal); // 校准单个通道
    bool adjustAFEOffset(uint8_t stage, ChannelOffsetCalibration& channel_cal); // 调整AFE偏移
    bool trySwapAFEOffset(uint8_t stage, ChannelOffsetCalibration& channel_cal); // 尝试交换AFE偏移
    bool adjustSampleOffset(uint8_t stage, ChannelOffsetCalibration& channel_cal); // 调整采样偏移
    bool adjustSensitivity(uint8_t stage, ChannelOffsetCalibration& channel_cal); // 调整灵敏度
    bool checkChannelTriggered(uint8_t stage); // 检查通道是否被触发
    void resetChannelCalibration(ChannelOffsetCalibration& channel_cal, uint8_t stage); // 重置通道校准状态

    // I2C通信方法
    bool write_register(uint16_t reg, uint8_t* value, uint16_t size = 2);
    bool read_register(uint16_t reg, uint16_t& value);
};