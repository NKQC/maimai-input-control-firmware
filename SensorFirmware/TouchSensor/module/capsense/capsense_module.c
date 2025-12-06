#include "capsense_module.h"
#include "../i2c/i2c_module.h"
#include <string.h>
#include "cy_capsense_processing.h"
#include "cy_capsense_filter.h"
#include "cy_capsense_selftest.h"
#include "fast_trigger.h"
#include "led_module.h"

static uint16_t g_touch_status_bitmap = 0;

volatile capsense_async_flags_t g_capsense_async = { .raw = 0u };

static void _capsense_preset_before_measurement(void);
#define CAPSENSE_CP_BALANCE_PCT 25
#define CAPSENSE_CP_RETRY_MAX 5

// 统一的异步更新结构体实例
capsense_unified_update_t g_capsense_update = {
    .fingercap_steps = {
        TOUCH_SENSITIVITY_DEFAULT_STEPS, TOUCH_SENSITIVITY_DEFAULT_STEPS,
        TOUCH_SENSITIVITY_DEFAULT_STEPS, TOUCH_SENSITIVITY_DEFAULT_STEPS,
        TOUCH_SENSITIVITY_DEFAULT_STEPS, TOUCH_SENSITIVITY_DEFAULT_STEPS,
        TOUCH_SENSITIVITY_DEFAULT_STEPS, TOUCH_SENSITIVITY_DEFAULT_STEPS,
        TOUCH_SENSITIVITY_DEFAULT_STEPS, TOUCH_SENSITIVITY_DEFAULT_STEPS,
        TOUCH_SENSITIVITY_DEFAULT_STEPS, TOUCH_SENSITIVITY_DEFAULT_STEPS,
    },
    .touch_thresholds = {
        TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT,
        TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT,
        TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT, TOUCH_THRESHOLD_DEFAULT,
    },
    .fingercap_update_mask = 0u,
    .threshold_update_mask = 0u,
};
typedef enum {
    CAPSENSE_COMP_LINEAR = 0u,
    CAPSENSE_COMP_LOG2 = 1u,
} capsense_comp_mode_t;

typedef struct {
    uint32_t start;
    uint32_t end;
    capsense_comp_mode_t mode;
    uint16_t den;
    uint16_t k;
} capsense_comp_seg_t;

static inline uint32_t _ilog2_u32(uint32_t x)
{
    uint32_t r = 0u;
    while (x >>= 1u) {
        ++r;
    }
    return r;
}

static const capsense_comp_seg_t g_capsense_comp_curve[] = {
    { 0u, 1000u, CAPSENSE_COMP_LINEAR, 1u, 0u },
    { 1000u, TOUCH_CAP_TOTAL_MAX_STEPS, CAPSENSE_COMP_LOG2, 0u, 1000u },
};

static inline uint32_t _median3_u32(uint32_t a, uint32_t b, uint32_t c)
{
    if (a > b) { uint32_t t = a; a = b; b = t; }
    if (b > c) { uint32_t t = b; b = c; c = t; }
    if (a > b) { uint32_t t = a; a = b; b = t; }
    return b;
}

static inline uint32_t _absdiff_u32(uint32_t x, uint32_t y)
{
    return (x > y) ? (x - y) : (y - x);
}

static inline bool _balanced3_u32(uint32_t a, uint32_t b, uint32_t c, uint32_t pct)
{
    uint32_t m = _median3_u32(a, b, c);
    uint32_t th = (m * pct) / 100u;
    return (_absdiff_u32(a, m) <= th) && (_absdiff_u32(b, m) <= th) && (_absdiff_u32(c, m) <= th);
}

static inline cy_en_capsense_bist_status_t _capsense_measure_cap_avg(uint32_t widget_id, uint32_t *out_cp_ff)
{
    uint8_t retries = CAPSENSE_CP_RETRY_MAX;
    for (;;) {
        uint32_t v1 = 0u, v2 = 0u, v3 = 0u;
        cy_en_capsense_bist_status_t s1 = Cy_CapSense_MeasureCapacitanceSensor(widget_id, 0u, &v1, &cy_capsense_context);
        if (s1 != CY_CAPSENSE_BIST_SUCCESS_E) return s1;
        cy_en_capsense_bist_status_t s2 = Cy_CapSense_MeasureCapacitanceSensor(widget_id, 0u, &v2, &cy_capsense_context);
        if (s2 != CY_CAPSENSE_BIST_SUCCESS_E) return s2;
        cy_en_capsense_bist_status_t s3 = Cy_CapSense_MeasureCapacitanceSensor(widget_id, 0u, &v3, &cy_capsense_context);
        if (s3 != CY_CAPSENSE_BIST_SUCCESS_E) return s3;

        if (_balanced3_u32(v1, v2, v3, CAPSENSE_CP_BALANCE_PCT)) {
            *out_cp_ff = (v1 + v2 + v3) / 3u;
            return CY_CAPSENSE_BIST_SUCCESS_E;
        }

        if (retries == 0u) {
            *out_cp_ff = _median3_u32(v1, v2, v3);
            return CY_CAPSENSE_BIST_SUCCESS_E;
        }
        --retries;
    }
}

static inline uint16_t _capsense_normalize_fingercap_steps(uint32_t cp_ff)
{
    uint32_t s = (uint32_t)(cp_ff / 10u);
    uint32_t y = s;
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_capsense_comp_curve)/sizeof(g_capsense_comp_curve[0])); ++i) {
        const capsense_comp_seg_t *seg = &g_capsense_comp_curve[i];
        if (s <= seg->end && s >= seg->start) {
            if (seg->mode == CAPSENSE_COMP_LINEAR) {
                y = seg->start + (uint32_t)((s - seg->start) / (seg->den == 0u ? 1u : seg->den));
            } else {
                uint32_t d = (s / (seg->start == 0u ? 1u : seg->start));
                if (d == 0u) d = 1u;
                y = seg->start + (uint32_t)(_ilog2_u32(d) * seg->k);
            }
            break;
        }
    }
    if (y > TOUCH_CAP_TOTAL_MAX_STEPS) {
        y = TOUCH_CAP_TOTAL_MAX_STEPS;
    }
    return (uint16_t)y;
}

static const uint32_t widget_ids[CAPSENSE_WIDGET_COUNT] = {
    CY_CAPSENSE_CAP0_WDGT_ID,
    CY_CAPSENSE_CAP1_WDGT_ID,
    CY_CAPSENSE_CAP2_WDGT_ID,
    CY_CAPSENSE_CAP3_WDGT_ID,
    CY_CAPSENSE_CAP4_WDGT_ID,
    CY_CAPSENSE_CAP5_WDGT_ID,
    CY_CAPSENSE_CAP6_WDGT_ID,
    CY_CAPSENSE_CAP7_WDGT_ID,
    CY_CAPSENSE_CAP8_WDGT_ID,
    CY_CAPSENSE_CAP9_WDGT_ID,
    CY_CAPSENSE_CAPA_WDGT_ID,
    CY_CAPSENSE_CAPB_WDGT_ID,
};

uint16_t capsense_get_touch_status_bitmap(void)
{
    return g_touch_status_bitmap;
}

// 触摸阈值API（fingerTh参数，范围1-65535）
uint16_t capsense_get_touch_threshold(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return TOUCH_THRESHOLD_DEFAULT;
    }
    return g_capsense_update.touch_thresholds[idx];
}

void capsense_set_touch_threshold(uint8_t idx, uint16_t threshold)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    threshold = clamp_threshold(threshold);
    g_capsense_update.touch_thresholds[idx] = threshold;
    capsense_mark_threshold_update(idx);
}

// 统一的fingercap电容值API
uint16_t capsense_get_fingercap_steps(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    return g_capsense_update.fingercap_steps[idx];
}

void capsense_set_fingercap_steps(uint8_t idx, uint16_t steps)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    steps = clamp_fingercap_steps(steps);
    g_capsense_update.fingercap_steps[idx] = steps;
    capsense_mark_fingercap_update(idx);
}

// 读取Cp基数（单位步进，0.001 pF），供I2C绝对模式换算

void capsense_init(void)
{
    NVIC_SetPriority(csd_interrupt_IRQn, CAPSENSE_INTR_PRIORITY);
    NVIC_EnableIRQ(csd_interrupt_IRQn);

    Cy_CapSense_Init(&cy_capsense_context);

    Cy_CapSense_Enable(&cy_capsense_context);

    _capsense_preset_before_measurement();

    g_capsense_async.bits.calibrate_req = 1u;
    g_capsense_async.bits.calibration_done = 1u;
    g_capsense_async.bits.baseline_frozen = 0u;
}

void capsense_process_widgets(void)
{
    // 校准完成后冻结基线，仅处理滤波、噪声与阈值，不更新基线
    if (g_capsense_async.bits.baseline_frozen) {
        for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
            Cy_CapSense_ProcessWidgetExt(
                widget_ids[i],
                (uint32_t)(
                    // CY_CAPSENSE_PROCESS_FILTER |
                    // CY_CAPSENSE_PROCESS_THRESHOLDS |
                    // CY_CAPSENSE_PROCESS_BASELINE |
                    // CY_CAPSENSE_PROCESS_CALC_NOISE |
                    CY_CAPSENSE_PROCESS_STATUS ),
                &cy_capsense_context);
        }
    } else {
        // 启动/校准阶段：保持默认的全部处理（含基线更新）
        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
    }
}

void capsense_update_touch_status(void)
{
    static uint16_t bitmap = 0;
    bitmap = 0;
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        bitmap |= (Cy_CapSense_IsWidgetActive(widget_ids[i], &cy_capsense_context) ? (1u << i) : 0u);
    }
    __disable_irq();
    g_touch_status_bitmap = bitmap;
    __enable_irq();
}

void capsense_apply_threshold_changes(void)
{
    // 使用中断安全的快照函数获取完整的更新数据
    capsense_update_snapshot_t snapshot;
    capsense_consume_updates_snapshot(&snapshot);

    // 如果没有任何更新，直接返回
    if (snapshot.fingercap_mask == 0u && snapshot.threshold_mask == 0u) {
        return;
    }

    bool fingercap_changed = false;
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        if (snapshot.fingercap_mask & (1u << i)) {
            fingercap_changed = true;
        }
    }

    if (fingercap_changed) {
        for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
            if (snapshot.fingercap_mask & (1u << i)) {
                capsense_write_fingercap_to_context(i, snapshot.fingercap_steps[i]);
            }
        }
    }
    capsense_apply_config_changes();
}

// 基于噪声包络运行与扩展处理，计算并写入阈值（fingerTh/hysteresis/noiseTh/nNoiseTh等）
void capsense_auto_tune_thresholds(uint8_t passes)
{
    if (passes == 0u) {
        return;
    }

    Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);

    for (uint8_t k = 0; k < passes; ++k) {
        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        while (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) {
            // 等待扫描完成
        }
        for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
            Cy_CapSense_ProcessWidgetExt(
                widget_ids[i],
                (uint32_t)(
                    CY_CAPSENSE_PROCESS_ALL),
                &cy_capsense_context);
        }
    }

    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
        Cy_CapSense_UpdateCrcWidget(widget_ids[i], &cy_capsense_context);
#endif
    }
}

void capsense_start_scan(void)
{
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
}

bool capsense_is_busy(void)
{
    return CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context);
}

// 在中断到来时调用库的处理函数，确保扫描能完成并清除 busy 状态
void csd_interrupt_IRQHandler(void)
{
    Cy_CapSense_InterruptHandler(CSD0, &cy_capsense_context);
}

static void _capsense_preset_before_measurement(void)
{
#if CY_CAPSENSE_BIST_EN
    Cy_CapSense_BistInitialize(&cy_capsense_context);
    Cy_CapSense_BistDsInitialize(&cy_capsense_context);
    if (cy_capsense_context.ptrBistContext != NULL) {
        cy_capsense_context.ptrBistContext->hwConfig = CY_CAPSENSE_BIST_HW_ELTD_CAP_E;
        cy_capsense_context.ptrBistContext->currentISC = CY_CAPSENSE_BIST_IO_HIGHZA_E;
        cy_capsense_context.ptrBistContext->eltdCapCsdISC = CY_CAPSENSE_BIST_IO_HIGHZA_E;
        cy_capsense_context.ptrBistContext->intrEltdCapCsdISC = CY_CAPSENSE_BIST_IO_HIGHZA_E;
    }
    while (Cy_CapSense_IsBusy(&cy_capsense_context) == CY_CAPSENSE_BUSY) {}
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        uint32_t cp_ff = 0u;
        cy_en_capsense_bist_status_t s = _capsense_measure_cap_avg(widget_ids[i], &cp_ff);
        if (s == CY_CAPSENSE_BIST_SUCCESS_E) {
            uint32_t cp_steps = (uint32_t)_capsense_normalize_fingercap_steps(cp_ff);
            cy_capsense_tuner.bistData.cy_capsense_eltdCap[i] = cp_ff;
            capsense_write_fingercap_to_context(i, cp_steps);
        } else {
            // led_on();
            capsense_write_fingercap_to_context(i, TOUCH_CAP_TOTAL_MAX_STEPS - 1 - (uint32_t)s);
        }
    }
    Cy_CapSense_BistDisableMode(&cy_capsense_context);
    capsense_apply_config_changes();
#endif
}

void capsense_handle_async_ops(void)
{
    // 校准请求：仅在CapSense空闲时执行
    if (g_capsense_async.bits.calibrate_req)
    {
        g_capsense_async.bits.calibrating = 1u;
        g_capsense_async.bits.calibration_done = 0u;

        // 噪声测量/阈值计算
        capsense_auto_tune_thresholds(8u);
        fast_trigger_init();

        g_capsense_async.bits.calibrating = 0u;
        g_capsense_async.bits.calibrate_req = 0u;
        g_capsense_async.bits.calibration_done = 1u;
    }
}

#if CY_CAPSENSE_BIST_EN
void capsense_measure_sensor_cp(void)
{
    Cy_CapSense_BistInitialize(&cy_capsense_context);
    Cy_CapSense_BistDsInitialize(&cy_capsense_context);
    if (cy_capsense_context.ptrBistContext != NULL) {
        cy_capsense_context.ptrBistContext->eltdCapModClk = 2u;
        cy_capsense_context.ptrBistContext->eltdCapSnsClk = 1024u;
        cy_capsense_context.ptrBistContext->eltdCapResolution = 12u;
        cy_capsense_context.ptrBistContext->eltdCapVrefMv = 1200u;
    }
    while (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) {}
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        uint32_t cp_ff = 0u;
        (void)Cy_CapSense_MeasureCapacitanceSensor(widget_ids[i], 0u, &cp_ff, &cy_capsense_context);
    }
    Cy_CapSense_BistDisableMode(&cy_capsense_context);
}
#endif

// 唯一的电容读取函数：直接从cy_capsense_context获取fingercap值（单位：0.001pF）
uint16_t capsense_read_fingercap_from_context(uint8_t channel)
{
    if (channel >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    return (uint16_t)(cy_capsense_context.ptrWdContext[channel].fingerCap);
}

// 唯一的阈值读取函数：直接从cy_capsense_context获取threshold值
uint16_t capsense_read_threshold_from_context(uint8_t channel)
{
    if (channel >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    // 从cy_capsense_context直接读取触摸阈值
    return cy_capsense_context.ptrWdContext[channel].fingerTh;
}

// 唯一的电容写入函数：接收通道和参数，写入cy_capsense_context并生效
void capsense_write_fingercap_to_context(uint8_t channel, uint16_t fingercap)
{
    if (channel >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    
    if (fingercap > TOUCH_CAP_TOTAL_MAX_STEPS) {
        fingercap = TOUCH_CAP_TOTAL_MAX_STEPS;
    }

    cy_capsense_tuner.widgetContext[channel].fingerCap = fingercap;
    cy_capsense_context.ptrWdContext[channel].fingerCap = fingercap;
}

// 唯一的阈值写入函数：接收通道和参数，写入cy_capsense_context并生效
void capsense_write_threshold_to_context(uint8_t channel, uint16_t threshold)
{
    if (channel >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    
    // 边界检查
    if (threshold > TOUCH_THRESHOLD_MAX) {
        threshold = TOUCH_THRESHOLD_MAX;
    }
    
    // 直接写入cy_capsense_context
    cy_capsense_tuner.widgetContext[channel].fingerTh = threshold;
}

void capsense_apply_config_changes(void)
{
    while (CY_CAPSENSE_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context)) {}
    Cy_CapSense_DeInit(&cy_capsense_context);
    NVIC_DisableIRQ(csd_interrupt_IRQn);
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        Cy_CapSense_UpdateCrcWidget(widget_ids[i], &cy_capsense_context);
    }
#endif
    NVIC_SetPriority(csd_interrupt_IRQn, CAPSENSE_INTR_PRIORITY);
    NVIC_EnableIRQ(csd_interrupt_IRQn);
    Cy_CapSense_Init(&cy_capsense_context);
    Cy_CapSense_Enable(&cy_capsense_context);
}

uint16_t capsense_get_diff(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0u;
    }
    return (uint16_t)cy_capsense_tuner.sensorContext[idx].diff;
}

uint16_t capsense_get_baseline(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0u;
    }
    return (uint16_t)cy_capsense_tuner.sensorContext[idx].bsln;
}

uint8_t capsense_get_resolution(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0u;
    }
    return (uint8_t)cy_capsense_context.ptrWdContext[idx].resolution;
}

uint16_t capsense_get_max_raw_count(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0u;
    }
    return (uint16_t)cy_capsense_context.ptrWdContext[idx].maxRawCount;
}