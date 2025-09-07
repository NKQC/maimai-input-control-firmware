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
#define AD7147_SENSITIVITY_DEFAULT           0x2929   // 默认灵敏度值
#define AD7147_DEFAULT_AFE_OFFSET            0x0000      // AFE偏移默认值

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
#define AD7147_DEFAULT_OFFSET_LOW            0x0000      // 默认低偏移值
#define AD7147_DEFAULT_OFFSET_HIGH           0x1000      // 默认高偏移值
#define AD7147_DEFAULT_OFFSET_HIGH_CLAMP     0x0000     // 默认高偏移钳位值
#define AD7147_DEFAULT_OFFSET_LOW_CLAMP      0x0000     // 默认低偏移钳位值
#define AD7147_CDC_BASELINE                  32767    // CDC基准值，用于显示计算


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
            {0x0002, 0x1000}, // Stage 0 - CIN0
            {0x0008, 0x1000}, // Stage 1 - CIN1  
            {0x0020, 0x1000}, // Stage 2 - CIN2
            {0x0080, 0x1000}, // Stage 3 - CIN3
            {0x0200, 0x1000}, // Stage 4 - CIN4
            {0x0800, 0x1000}, // Stage 5 - CIN5
            {0x2000, 0x1000}, // Stage 6 - CIN6
            {0x0000, 0x1002}, // Stage 7 - CIN7
            {0x0000, 0x1008}, // Stage 8 - CIN8
            {0x0000, 0x1020}, // Stage 9 - CIN9
            {0x0000, 0x1080}, // Stage 10 - CIN10
            {0x0000, 0x1200}  // Stage 11 - CIN11
        };
        
        for (int i = 0; i < 12; i++) {
            stages[i].connection_6_0 = default_connections[i][0];
            stages[i].connection_12_7 = default_connections[i][1];
        }
    }
 };

// 寄存器结构体定义
// PWR_CONTROL寄存器 (0x000)
union PWRControlRegister {
    uint16_t raw;
    struct {
        uint16_t power_mode : 2;           // [1:0] 工作模式
        uint16_t lp_conv_delay : 2;        // [3:2] 低功耗模式转换延迟
        uint16_t sequence_stage_num : 4;   // [7:4] 序列中的阶段数量 (N+1)
        uint16_t decimation : 2;           // [9:8] ADC抽取因子
        uint16_t sw_reset : 1;             // [10] 软件复位控制
        uint16_t int_pol : 1;              // [11] 中断极性控制
        uint16_t ext_source : 1;           // [12] 激励源控制
        uint16_t unused : 1;               // [13] 未使用，设为0
        uint16_t cdc_bias : 2;             // [15:14] CDC偏置电流控制
    } bits;
    
    PWRControlRegister() : raw(0x12F0) {}  // 默认值
};

// STAGE_CAL_EN寄存器 (0x001)
union StageCalEnRegister {
    uint16_t raw;
    struct {
        uint16_t stage0_cal_en : 1;        // [0] STAGE0校准使能
        uint16_t stage1_cal_en : 1;        // [1] STAGE1校准使能
        uint16_t stage2_cal_en : 1;        // [2] STAGE2校准使能
        uint16_t stage3_cal_en : 1;        // [3] STAGE3校准使能
        uint16_t stage4_cal_en : 1;        // [4] STAGE4校准使能
        uint16_t stage5_cal_en : 1;        // [5] STAGE5校准使能
        uint16_t stage6_cal_en : 1;        // [6] STAGE6校准使能
        uint16_t stage7_cal_en : 1;        // [7] STAGE7校准使能
        uint16_t stage8_cal_en : 1;        // [8] STAGE8校准使能
        uint16_t stage9_cal_en : 1;        // [9] STAGE9校准使能
        uint16_t stage10_cal_en : 1;       // [10] STAGE10校准使能
        uint16_t stage11_cal_en : 1;       // [11] STAGE11校准使能
        uint16_t avg_fp_skip : 2;          // [13:12] 全功率模式跳过控制
        uint16_t avg_lp_skip : 2;          // [15:14] 低功率模式跳过控制
    } bits;
    
    StageCalEnRegister() : raw(0x0000) {}  // 默认值

};

// AMB_COMP_CTRL0寄存器 (0x002)
union AmbCompCtrl0Register {
    uint16_t raw;
    struct {
        uint16_t ff_skip_cnt : 4;          // [3:0] 快速滤波器跳过控制
        uint16_t fp_proximity_cnt : 4;     // [7:4] 全功率模式接近计数
        uint16_t lp_proximity_cnt : 4;     // [11:8] 低功率模式接近计数
        uint16_t pwr_down_timeout : 2;     // [13:12] 全功率到低功率模式超时控制
        uint16_t forced_cal : 1;           // [14] 强制校准控制
        uint16_t conv_reset : 1;           // [15] 转换复位控制
    } bits;
    
    AmbCompCtrl0Register() : raw(0xC0FF) {}  // 默认值
};

// AMB_COMP_CTRL1寄存器 (0x003)
union AmbCompCtrl1Register {
    uint16_t raw;
    struct {
        uint16_t proximity_recal_lvl : 8;  // [7:0] 接近重新校准级别
        uint16_t proximity_detection_rate : 6; // [13:8] 接近检测速率
        uint16_t slow_filter_update_lvl : 2;   // [15:14] 慢滤波器更新级别
    } bits;
    
    AmbCompCtrl1Register() : raw(0x0040) {}  // 默认值 (64)
};

// AMB_COMP_CTRL2寄存器 (0x004)
union AmbCompCtrl2Register {
    uint16_t raw;
    struct {
        uint16_t fp_proximity_recal : 10;  // [9:0] 全功率模式接近重新校准时间控制
        uint16_t lp_proximity_recal : 6;   // [15:10] 低功率模式接近重新校准时间控制
    } bits;
    
    AmbCompCtrl2Register() : raw(0xFFFF) {}  // 默认值 (0x3FF | (0x3F << 10))
};

// 寄存器配置结构体
struct AD7147RegisterConfig {
    PWRControlRegister pwr_control;        // 0x000 电源控制
    StageCalEnRegister stage_cal_en;       // 0x001 阶段校准使能
    AmbCompCtrl0Register amb_comp_ctrl0;   // 0x002 环境补偿控制0
    AmbCompCtrl1Register amb_comp_ctrl1;   // 0x003 环境补偿控制1
    AmbCompCtrl2Register amb_comp_ctrl2;   // 0x004 环境补偿控制2
    uint16_t stage_low_int_enable;         // 0x005 阶段低中断使能
    uint16_t stage_high_int_enable;        // 0x006 阶段高中断使能
    uint16_t stage_complete_int_enable;    // 0x007 阶段完成中断使能
    
    AD7147RegisterConfig() :
        stage_low_int_enable(0x0FFF),      // 默认值
        stage_high_int_enable(0x0000),     // 默认值
        stage_complete_int_enable(0x0000)  // 默认值
    {}
};

class AD7147 : public TouchSensor {
public:
    AD7147(HAL_I2C* i2c_hal, I2C_Bus i2c_bus, uint8_t device_addr);
    ~AD7147() noexcept override;
    
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
    AD7147RegisterConfig register_config_;   // 寄存器配置
    
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

    // 私有方法
    bool applyEnabledChannelsToHardware();
    bool configureStages(uint16_t power_control_val, const uint16_t* connection_values);
    inline bool apply_stage_settings();      // 应用stage设置到硬件
    
    // 校准相关辅助方法
    void processAutoOffsetCalibration();     // 在sample()中调用，处理自动偏移校准状态机
 
    // I2C通信方法
    bool write_register(uint16_t reg, uint8_t* value, uint16_t size = 2);
    bool read_register(uint16_t reg, uint16_t& value);

    class CalibrationTools {
        public:
        /**
         * 采样: CDC采样连续采样50次 记录平均值 最大值和最小值
         * 每个通道都要来一遍下面的流程 校准即一次顺序校准全部阶段
         * 
         * 触发连续采样50次 记录触发和未触发次数 换算为比例作为稳定值
         * 重置设置均为0 
         * 灵敏度设置为默认值
         * 截断值设置最大0xFFFF 最小0x0000
         * 
         * 1.启动检查CDC值是否在5000 - 60000 区间
         * 2.如果超过60000 则尝试每次10步拉AEF负值 如果低于5000 则尝试每次10步拉AEF正值 如果卡在极限值震荡 则优先向正方向拉线 尝试均极值后负方向拉线
         * 3.当值显著向低回偏 则尝试拉到基准线 过基准线则减半步数 反方向拉 当再次过线继续减半步数 反方向拉 直到step为1时在基线附近
         * 4.当AEF基线在对应值到极值时仍未拉到基线 则启动对方方向值的反向模式 随后继续以当前步数拉值
         * 5.当任意时刻成功调回基线 则宣告阶段1完成 如果双边均测试到极值无法继续 则宣告当前通道电容值过大 失败
         * 6.在调回基线后 检查通道是否存在触发情况 如果存在触发 则每步50提高高偏移值 直到整个采样中不存在触发 宣告阶段2完成
         * 7.如果阶段2不存在触发/已调整完毕不存在触发 则每步50降低偏移值 一旦检测到触发 则减半步数 提高步进偏移值 直到没有触发 则继续减半步数 步进降低偏移值 直到再次触发 再次减半步数...
         * 该迭代最多迭代5次步进值降低 确保最后一步是抬高阈值 最终采样不应该出现触发
         * 当完成后 继续监测1000个采样 确认无触发 宣告完成 如果仍有触发 则以5步进抬高偏移值 继续监测1000个采样 确认无触发 宣告完成 如果仍有触发 继续抬高 循环监测 直到确认无触发
         * 8.当阶段2完成后 执行阶段3 根据空载低值 设置低极限值为阶段2验证采样最低值的80% 设置最高值为阶段2验证采样最高值的200%(若超限则直接CDC极限值) 即结束
         */
        // 校准阶段
        enum CalibrationState {
            IDLE,  // 初始化/完成时设置回IDLE
            Stage1_Pos_baseline,
            Stage1_Neg_baseline,

            Stage2_Offset_calibration,
            Stage2_Measure_offset,

            Stage3_verify_limit
        };
        
        enum Direction {
            Pos,
            Neg
        };

        struct CDCSample_result {
            uint16_t average = 0;
            uint16_t max = 0;
            uint16_t min = 0;
            uint16_t sample_count = 0;
            CDCSample_result () {}
            void clear() {
                average = 0;
                max = 0;
                min = 0;
                sample_count = 0;
            }
        };

        struct TriggleSample {
            uint16_t triggle_num = 0;
            uint16_t not_triggle_num = 0;
            uint16_t sample_count = 0;
            TriggleSample() {}
            void clear() {
                triggle_num = 0;
                not_triggle_num = 0;
                sample_count = 0;
            }
        };

        AD7147* pthis = nullptr;

        CalibrationState calibration_state_ = IDLE;

        bool start_calibration() {
            if (calibration_state_ != IDLE) return false;
            calibration_state_ = Stage1_Pos_baseline;
            return true;
        }

        // 主循环方法
        void CalibrationLoop();
        
        // 工具方法
        // 清空设置到校准所需值
        void Clear_and_prepare_stage_settings(uint8_t stage);
        // 直接设置AEF偏移 已内置正负翻转 0-127 
        void Set_AEF_Offset(uint8_t stage, Direction direction, uint8_t offset);
        // 直接设置执行方向的偏移值 0-65535
        void Set_Offset(uint8_t stage, Direction direction, uint16_t offset);
        // 直接设置指定方向的偏移阈值 0-65535
        void Set_Clamp(uint8_t stage, Direction direction, uint16_t clamp);

        // [执行一次采样一次 到目标周期返回True] 读取CDC值 50次采样 计算平均值 最大值和最小值
        bool Read_CDC_50Sample(uint8_t stage, CDCSample_result& result);
        // [执行一次采样一次 直接解析sample中的采样数据 sample应当通过外部直接传入循环中的采样结果原始值 到目标周期返回True] 读取触发值 50次采样 计算触发和未触发次数
        bool Read_Triggle_50Sample(uint8_t stage, uint32_t sample, TriggleSample& result);
    };

    CalibrationTools calibration_tools_;
};