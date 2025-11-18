#ifndef CAPSENSE_MODULE_H
#define CAPSENSE_MODULE_H

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_capsense.h"
#include "cycfg_capsense_defines.h"

#define CAPSENSE_INTR_PRIORITY    (3u)
#define CAPSENSE_WIDGET_COUNT     (12u)

// TouchSensitivity相关定义（增量设置）：单位 0.01 pF
#define TOUCH_SENSITIVITY_STEP_PF           (0.01f)     // 每步进对应0.01pF
// 非FULL模式最低值可为0；FULL模式维持历史最小0.1pF（10步）
#if (CY_CAPSENSE_SMARTSENSE_FULL_EN)
    #define TOUCH_INCREMENT_MIN_STEPS       (10u)
    // FULL模式：最大值=1.00pF（100步进）
    #define TOUCH_SENSITIVITY_MAX_STEPS     (100u)
#else
    #define TOUCH_INCREMENT_MIN_STEPS       (0u)
    // 非FULL模式：最大值=20.00pF（2000步进），满足部分SmartSense场景
    #define TOUCH_SENSITIVITY_MAX_STEPS     (2000u)
#endif
// 默认增量为1.00pF（100步进）
#define TOUCH_SENSITIVITY_DEFAULT_STEPS     (100u)
// 总触摸电容上限（Cp+增量，不超过22pF）
#define TOUCH_CAP_TOTAL_MAX_STEPS           (2200u)
// 新增：触摸灵敏度寄存器原始值编码（有符号偏移），零点为4095
#define TOUCH_SENSITIVITY_ZERO_BIAS         (4095u)
#define TOUCH_SENSITIVITY_RAW_MIN           (0u)
#define TOUCH_SENSITIVITY_RAW_MAX           (8191u)

// 新增：触摸阈值相关定义（范围1-65535）
#define TOUCH_THRESHOLD_MIN                     (1u)
#define TOUCH_THRESHOLD_MAX                     (65535u)
#define TOUCH_THRESHOLD_DEFAULT                 (110u)

// 全局异步状态位域（统一管理）：
// bit0: calibrate_req（校准请求）
// bit1: calibrating（校准进行中）
// bit2: calibration_done（校准完成）
// bit3: baseline_frozen（基线冻结）
// 其余保留
typedef union {
    uint32_t raw;
    struct {
        uint32_t calibrate_req     : 1;
        uint32_t calibrating       : 1;
        uint32_t calibration_done  : 1;
        uint32_t baseline_frozen   : 1;
        uint32_t reserved          : 28;
    } bits;
} capsense_async_flags_t;

extern volatile capsense_async_flags_t g_capsense_async;

static inline void capsense_request_calibration(void)
{
    g_capsense_async.bits.calibrate_req = 1u;
}

// 统一的异步更新结构体：将数值和更新mask绑定在一起
typedef struct {
    // 电容值数组：统一的fingercap值（单位：0.01pF步进）
    uint16_t fingercap_steps[CAPSENSE_WIDGET_COUNT];
    // 触摸阈值数组
    uint16_t touch_thresholds[CAPSENSE_WIDGET_COUNT];
    // 更新掩码：标记哪些通道需要更新
    volatile uint16_t fingercap_update_mask;
    volatile uint16_t threshold_update_mask;
} capsense_unified_update_t;

extern capsense_unified_update_t g_capsense_update;

// 内联限位函数：避免反复构造
static inline uint16_t clamp_fingercap_steps(uint16_t steps)
{
    if (steps > TOUCH_CAP_TOTAL_MAX_STEPS) {
        return TOUCH_CAP_TOTAL_MAX_STEPS;
    }
    return steps;
}

static inline uint16_t clamp_threshold(uint16_t threshold)
{
    if (threshold < TOUCH_THRESHOLD_MIN) {
        return TOUCH_THRESHOLD_MIN;
    } else if (threshold > TOUCH_THRESHOLD_MAX) {
        return TOUCH_THRESHOLD_MAX;
    }
    return threshold;
}

static inline int16_t clamp_sensitivity_steps(int16_t steps)
{
    if (steps > (int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
        steps = (int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
    } else if (steps < -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
        steps = -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
    }
    if (steps > 0 && steps < (int16_t)TOUCH_INCREMENT_MIN_STEPS) {
        steps = (int16_t)TOUCH_INCREMENT_MIN_STEPS;
    }
    return steps;
}

// 标记fingercap更新
static inline void capsense_mark_fingercap_update(uint8_t idx)
{
    if (idx < CAPSENSE_WIDGET_COUNT) {
        __disable_irq();
        g_capsense_update.fingercap_update_mask |= (uint16_t)(1u << idx);
        __enable_irq();
    }
}

// 标记触摸阈值更新
static inline void capsense_mark_threshold_update(uint8_t idx)
{
    if (idx < CAPSENSE_WIDGET_COUNT) {
        __disable_irq();
        g_capsense_update.threshold_update_mask |= (uint16_t)(1u << idx);
        __enable_irq();
    }
}

// 中断安全的快照结构体：包含掩码和对应的数据
typedef struct {
    uint16_t fingercap_mask;
    uint16_t threshold_mask;
    uint16_t fingercap_steps[CAPSENSE_WIDGET_COUNT];
    uint16_t touch_thresholds[CAPSENSE_WIDGET_COUNT];
} capsense_update_snapshot_t;

// 中断安全的快照函数：原子性地获取掩码和对应数据
static inline void capsense_consume_updates_snapshot(capsense_update_snapshot_t* snapshot)
{
    __disable_irq();
    // 获取掩码
    snapshot->fingercap_mask = g_capsense_update.fingercap_update_mask;
    snapshot->threshold_mask = g_capsense_update.threshold_update_mask;
    
    // 只有在有更新时才复制对应的数据
    if (snapshot->fingercap_mask != 0u) {
        for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
            if (snapshot->fingercap_mask & (1u << i)) {
                snapshot->fingercap_steps[i] = g_capsense_update.fingercap_steps[i];
            }
        }
        // 清除已消费的掩码
        g_capsense_update.fingercap_update_mask = 0u;
    }
    
    if (snapshot->threshold_mask != 0u) {
        for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
            if (snapshot->threshold_mask & (1u << i)) {
                snapshot->touch_thresholds[i] = g_capsense_update.touch_thresholds[i];
            }
        }
        // 清除已消费的掩码
        g_capsense_update.threshold_update_mask = 0u;
    }
    __enable_irq();
}

// 保留旧的函数以维持兼容性（但标记为已弃用）
static inline uint16_t capsense_consume_fingercap_updates(void)
{
    uint16_t pending;
    __disable_irq();
    pending = g_capsense_update.fingercap_update_mask;
    g_capsense_update.fingercap_update_mask = 0u;
    __enable_irq();
    return pending;
}

static inline uint16_t capsense_consume_threshold_updates(void)
{
    uint16_t pending;
    __disable_irq();
    pending = g_capsense_update.threshold_update_mask;
    g_capsense_update.threshold_update_mask = 0u;
    __enable_irq();
    return pending;
}

// 噪声只读寄存器内联访问（直接从cy_capsense_tuner读取当前阈值）
static inline uint16_t capsense_get_noise_th(uint8_t idx)
{
    return cy_capsense_context.ptrWdContext[idx].noiseTh;
}

static inline uint16_t capsense_get_nnoise_th(uint8_t idx)
{
    return cy_capsense_context.ptrWdContext[idx].nNoiseTh;
}

uint16_t capsense_get_touch_status_bitmap(void);

// 触摸阈值API（fingerTh参数，范围1-65535）
uint16_t capsense_get_touch_threshold(uint8_t idx);
void     capsense_set_touch_threshold(uint8_t idx, uint16_t threshold);

// 触摸灵敏度API（fingerCap参数，以0.01pF为单位的步进值）
uint16_t capsense_sensitivity_to_raw_count(int16_t steps);
int16_t  capsense_raw_count_to_sensitivity(uint16_t raw);

// 统一的fingercap电容值API
uint16_t capsense_get_fingercap_steps(uint8_t idx);
void     capsense_set_fingercap_steps(uint8_t idx, uint16_t steps);

// 唯一的读写函数：直接操作cy_capsense_context
uint16_t capsense_read_fingercap_from_context(uint8_t channel);
uint16_t capsense_read_threshold_from_context(uint8_t channel);
void capsense_write_fingercap_to_context(uint8_t channel, uint16_t fingercap_steps);
void capsense_write_threshold_to_context(uint8_t channel, uint16_t threshold);

uint16_t capsense_get_cp_base_steps(uint8_t idx);

void capsense_init(void);
void capsense_process_widgets(void);
void capsense_update_touch_status(void);
void capsense_apply_threshold_changes(void);
void capsense_start_scan(void);
bool capsense_is_busy(void);
void capsense_handle_async_ops(void);

// 自动调谐阈值
void capsense_auto_tune_thresholds(uint8_t passes);

#if CY_CAPSENSE_BIST_EN
void capsense_measure_sensor_cp(void);
#endif

// 新增：读取通道滤波后的原始计数（raw）
uint16_t capsense_get_raw_filtered(uint8_t idx);
// 新增：读取通道当前基线（bsln）
uint16_t capsense_get_baseline(uint8_t idx);

#endif // CAPSENSE_MODULE_H