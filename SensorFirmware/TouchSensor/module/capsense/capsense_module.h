#ifndef CAPSENSE_MODULE_H
#define CAPSENSE_MODULE_H

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_capsense.h"
#include "cycfg_capsense_defines.h"

#define CAPSENSE_INTR_PRIORITY    (3u)
#define CAPSENSE_WIDGET_COUNT     (12u)

#define CAPSENSOR_RATE (1.2f)

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

// 独立的常驻更新掩码（每Widget一位）：用于阈值/参数更新的挂起标记
extern volatile uint16_t g_capsense_update_mask;

static inline void capsense_mark_update(uint8_t idx)
{
    if (idx < CAPSENSE_WIDGET_COUNT) {
        // 在ISR上下文中不切换全局中断，仅进行原子位设置
        g_capsense_update_mask |= (uint16_t)(1u << idx);
    }
}

static inline uint16_t capsense_consume_updates(void)
{
    __disable_irq();
    uint16_t pending = g_capsense_update_mask;
    g_capsense_update_mask = 0u;
    __enable_irq();
    return pending;
}

// 噪声只读寄存器内联访问（直接从cy_capsense_tuner读取当前阈值）
static inline uint16_t capsense_get_noise_th(uint8_t idx)
{
    return (idx < CAPSENSE_WIDGET_COUNT) ? cy_capsense_tuner.widgetContext[idx].noiseTh : 0u;
}

static inline uint16_t capsense_get_nnoise_th(uint8_t idx)
{
    return (idx < CAPSENSE_WIDGET_COUNT) ? cy_capsense_tuner.widgetContext[idx].nNoiseTh : 0u;
}

uint16_t capsense_get_touch_status_bitmap(void);
int16_t capsense_get_threshold(uint8_t idx);
void    capsense_set_threshold(uint8_t idx, int16_t value);

// TouchSensitivity相关函数（I2C按步进值读写，表示增量设置，以0.01pF为单位）
int16_t  capsense_get_touch_sensitivity(uint8_t idx);
void     capsense_set_touch_sensitivity(uint8_t idx, int16_t sensitivity_steps);
uint16_t capsense_sensitivity_to_raw_count(int16_t steps);
int16_t  capsense_raw_count_to_sensitivity(uint16_t raw);
// 读取总触摸电容（Cp基数 + 增量设置），单位步进（0.01 pF）
uint16_t capsense_get_total_touch_cap(uint8_t idx);
// 新增：读取Cp基数（单位步进，0.01 pF），供I2C绝对模式换算
uint16_t capsense_get_cp_base_steps(uint8_t idx);

void capsense_init(void);
void capsense_process_widgets(void);
void capsense_update_touch_status(void);
void capsense_apply_threshold_changes(void);
void capsense_start_scan(void);
bool capsense_is_busy(void);
void capsense_handle_async_ops(void);

// 自动噪声测量并写入阈值（fingerTh/hysteresis/noiseTh/nNoiseTh等）
void capsense_auto_tune_thresholds(uint8_t passes);

#if CY_CAPSENSE_BIST_EN
void capsense_measure_sensor_cp(void);
#endif

#endif /* CAPSENSE_MODULE_H */