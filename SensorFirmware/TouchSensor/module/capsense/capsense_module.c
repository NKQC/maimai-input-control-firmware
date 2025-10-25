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

// 读取Cp基数（单位步进，0.01 pF），供I2C绝对模式换算

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
                    // CY_CAPSENSE_PROCESS_DIFFCOUNTS |
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

    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        if (snapshot.fingercap_mask & (1u << i)) {
            uint16_t fingercap_steps = snapshot.fingercap_steps[i];
            // 调用唯一的写入函数，直接写入cy_capsense_context并生效
            capsense_write_fingercap_to_context(i, fingercap_steps);
        }

        // 处理触摸阈值更新：仅一对一更新
        if (snapshot.threshold_mask & (1u << i)) {
            uint16_t threshold = snapshot.touch_thresholds[i];
            // 调用唯一的写入函数，直接写入cy_capsense_context并生效
            capsense_write_threshold_to_context(i, threshold);
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
    // 若启用了BIST，可进行自电容测量并直接更新fingercap_steps
#if CY_CAPSENSE_BIST_EN
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        uint32_t cp_ff = 0u;
        if (Cy_CapSense_MeasureCapacitanceSensor(widget_ids[i], 0u, &cp_ff, &cy_capsense_context) == CY_CAPSENSE_STATUS_SUCCESS) {
            // 以0.01pF步进记录基数，直接写入cy_capsense_context
            uint32_t cp_steps = (cp_ff + 5u) / 10u;
            if (cp_steps > TOUCH_CAP_TOTAL_MAX_STEPS) cp_steps = TOUCH_CAP_TOTAL_MAX_STEPS;
            // 使用唯一的写入函数更新对应通道
            capsense_write_fingercap_to_context(i, (uint16_t)cp_steps);
        }
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

// 唯一的电容读取函数：直接从cy_capsense_context获取fingercap值（单位：0.01pF）
uint16_t capsense_read_fingercap_from_context(uint8_t channel)
{
    if (channel >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    // 从cy_capsense_context直接读取，转换为0.01pF步进
    return (uint16_t)(cy_capsense_context.ptrWdContext[channel].fingerCap / 10u);
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
void capsense_write_fingercap_to_context(uint8_t channel, uint16_t fingercap_steps)
{
    if (channel >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    
    // 边界检查
    if (fingercap_steps > TOUCH_CAP_TOTAL_MAX_STEPS) {
        fingercap_steps = TOUCH_CAP_TOTAL_MAX_STEPS;
    }
    
    // 直接写入cy_capsense_context（转换为0.1pF单位）
    cy_capsense_context.ptrWdContext[channel].fingerCap = (uint16_t)(fingercap_steps * 10u);
    
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
    // 更新CRC使配置生效
    Cy_CapSense_UpdateCrcWidget(widget_ids[channel], &cy_capsense_context);
#endif
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
    cy_capsense_context.ptrWdContext[channel].fingerTh = threshold;
    
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
    // 更新CRC使配置生效
    Cy_CapSense_UpdateCrcWidget(widget_ids[channel], &cy_capsense_context);
#endif
}