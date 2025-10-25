#include "capsense_module.h"
#include "../i2c/i2c_module.h"
#include <string.h>
#include "cy_capsense_processing.h"
#include "cy_capsense_filter.h"
#include "cy_capsense_selftest.h"

static uint16_t g_touch_status_bitmap = 0;

volatile capsense_async_flags_t g_capsense_async = { .raw = 0u };

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

// Cp基数（单位步进，0.01 pF），由自电容测量更新
static uint16_t g_cp_steps[CAPSENSE_WIDGET_COUNT] = {0};

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

// 触摸灵敏度API（兼容性接口，内部转换为fingercap操作）
int16_t capsense_get_touch_sensitivity(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    // 在相对模式下，返回相对于基础电容的偏移
    if (i2c_get_absolute_mode()) {
        return (int16_t)g_capsense_update.fingercap_steps[idx];
    } else {
        int32_t offset = (int32_t)g_capsense_update.fingercap_steps[idx] - (int32_t)g_cp_steps[idx];
        return clamp_sensitivity_steps((int16_t)offset);
    }
}

void capsense_set_touch_sensitivity(uint8_t idx, int16_t sensitivity_steps)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    sensitivity_steps = clamp_sensitivity_steps(sensitivity_steps);
    
    uint16_t new_fingercap;
    if (i2c_get_absolute_mode()) {
        // 绝对模式：直接设置fingercap值
        new_fingercap = (sensitivity_steps >= 0) ? (uint16_t)sensitivity_steps : 0u;
    } else {
        // 相对模式：基于基础电容计算新的fingercap值
        int32_t total_steps = (int32_t)g_cp_steps[idx] + (int32_t)sensitivity_steps;
        new_fingercap = (total_steps >= 0) ? (uint16_t)total_steps : 0u;
    }
    
    capsense_set_fingercap_steps(idx, new_fingercap);
}

uint16_t capsense_sensitivity_to_raw_count(int16_t steps)
{
    steps = clamp_sensitivity_steps(steps);
    int32_t raw = (int32_t)TOUCH_SENSITIVITY_ZERO_BIAS + (int32_t)steps;
    if (raw < (int32_t)TOUCH_SENSITIVITY_RAW_MIN) raw = (int32_t)TOUCH_SENSITIVITY_RAW_MIN;
    if (raw > (int32_t)TOUCH_SENSITIVITY_RAW_MAX) raw = (int32_t)TOUCH_SENSITIVITY_RAW_MAX;
    return (uint16_t)raw;
}

int16_t capsense_raw_count_to_sensitivity(uint16_t raw)
{
    int32_t steps = (int32_t)raw - (int32_t)TOUCH_SENSITIVITY_ZERO_BIAS;
    return clamp_sensitivity_steps((int16_t)steps);
}

// 读取Cp基数（单位步进，0.01 pF），供I2C绝对模式换算
uint16_t capsense_get_cp_base_steps(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    return g_cp_steps[idx];
}

void capsense_init(void)
{
    Cy_CapSense_Init(&cy_capsense_context);
    NVIC_SetPriority((IRQn_Type)10, CAPSENSE_INTR_PRIORITY);
    NVIC_EnableIRQ((IRQn_Type)10);
    Cy_CapSense_Enable(&cy_capsense_context);
    // 开机时触发一次噪声测量/校准，主循环在空闲时执行
    g_capsense_async.bits.calibrate_req = 1u;
    g_capsense_async.bits.calibration_done = 0u;
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
                    CY_CAPSENSE_PROCESS_FILTER |
                    CY_CAPSENSE_PROCESS_DIFFCOUNTS |
                    CY_CAPSENSE_PROCESS_THRESHOLDS |
                    CY_CAPSENSE_PROCESS_BASELINE |
                    CY_CAPSENSE_PROCESS_CALC_NOISE |
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

    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        // 处理fingercap更新
        if (snapshot.fingercap_mask & (1u << i)) {
            uint16_t fingercap_steps = snapshot.fingercap_steps[i];
            
#if (CY_CAPSENSE_SMARTSENSE_FULL_EN)
            // FULL模式：不对 fingerCap 做增量更新（由库/SmartSense管理）
#else
            // 确保基础电容是最新的：如果g_cp_steps为0，则实时测量
            if (g_cp_steps[i] == 0u) {
#if CY_CAPSENSE_BIST_EN
                uint32_t cp_ff = 0u;
                if (Cy_CapSense_MeasureCapacitanceSensor(widget_ids[i], 0u, &cp_ff, &cy_capsense_context) == CY_CAPSENSE_STATUS_SUCCESS) {
                    // 以0.01pF步进记录基数
                    uint32_t cp_steps = (cp_ff + 5u) / 10u;
                    if (cp_steps > TOUCH_CAP_TOTAL_MAX_STEPS) cp_steps = TOUCH_CAP_TOTAL_MAX_STEPS;
                    g_cp_steps[i] = (uint16_t)cp_steps;
                }
#endif
            }
            
            // 直接设置fingercap值（已经是最终的电容值）
            cy_capsense_context.ptrWdContext[i].fingerCap = (uint16_t)(fingercap_steps * 10u);
            
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
            Cy_CapSense_UpdateCrcWidget(widget_ids[i], &cy_capsense_context);
#endif
#endif
        }

        // 处理触摸阈值更新
        if (snapshot.threshold_mask & (1u << i)) {
            uint16_t threshold = snapshot.touch_thresholds[i];
            // 应用触摸阈值到CapSense上下文
            cy_capsense_context.ptrWdContext[i].fingerTh = threshold;
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
            Cy_CapSense_UpdateCrcWidget(widget_ids[i], &cy_capsense_context);
#endif
        }
    }
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
    Cy_CapSense_InterruptHandler(NULL, &cy_capsense_context);
}

static void _capsense_preset_before_measurement(void)
{
    // 若启用了BIST，可进行自电容测量并计算Cp步进基数
#if CY_CAPSENSE_BIST_EN
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        uint32_t cp_ff = 0u;
        (void)Cy_CapSense_MeasureCapacitanceSensor(widget_ids[i], 0u, &cp_ff, &cy_capsense_context);
        // 以0.01pF步进记录基数
        uint32_t cp_steps = (cp_ff + 5u) / 10u;
        if (cp_steps > TOUCH_CAP_TOTAL_MAX_STEPS) cp_steps = TOUCH_CAP_TOTAL_MAX_STEPS;
        g_cp_steps[i] = (uint16_t)cp_steps;
    }

    // 按当前设置应用fingercap值
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        uint16_t fingercap_steps = g_capsense_update.fingercap_steps[i];
        cy_capsense_context.ptrWdContext[i].fingerCap = (uint16_t)(fingercap_steps * 10u);
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
        Cy_CapSense_UpdateCrcWidget(widget_ids[i], &cy_capsense_context);
#endif
    }
#endif

    // 初始化基线，确保后续噪声测量稳定
    Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
}

void capsense_handle_async_ops(void)
{
    // 校准请求：仅在CapSense空闲时执行
    if (g_capsense_async.bits.calibrate_req)
    {
        g_capsense_async.bits.calibrating = 1u;
        g_capsense_async.bits.calibration_done = 0u;

        // 启动前自测与参数预设
        _capsense_preset_before_measurement();

        // 噪声测量/阈值计算
        capsense_auto_tune_thresholds(8u);

        g_capsense_async.bits.calibrating = 0u;
        g_capsense_async.bits.calibrate_req = 0u;
        g_capsense_async.bits.calibration_done = 1u;
    }
}

#if CY_CAPSENSE_BIST_EN
void capsense_measure_sensor_cp(void)
{
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        uint32_t cp_ff = 0u;
        (void)Cy_CapSense_MeasureCapacitanceSensor(widget_ids[i], 0u, &cp_ff, &cy_capsense_context);
    }
}
#endif