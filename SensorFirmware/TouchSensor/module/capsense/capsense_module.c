#include "capsense_module.h"
#include <string.h>
#include "cy_capsense_processing.h"
#include "cy_capsense_filter.h"
#include "cy_capsense_selftest.h"

static uint16_t g_touch_status_bitmap = 0;

// 统一的全局异步位域变量
volatile capsense_async_flags_t g_capsense_async = { .raw = 0u };
// 独立的常驻更新掩码（每Widget一位）
volatile uint16_t g_capsense_update_mask = 0u;

// 异步更新结构：保存期望值（步进，表示增量设置）
typedef struct {
    int16_t sensitivity_steps[CAPSENSE_WIDGET_COUNT];
} capsense_update_req_t;

static capsense_update_req_t g_update = {
    .sensitivity_steps = {
        (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS, (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS,
        (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS, (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS,
        (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS, (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS,
        (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS, (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS,
        (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS, (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS,
        (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS, (int16_t)TOUCH_SENSITIVITY_DEFAULT_STEPS,
    },
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

int16_t capsense_get_threshold(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    return g_update.sensitivity_steps[idx];
}

void capsense_set_threshold(uint8_t idx, int16_t value)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    g_update.sensitivity_steps[idx] = value;
    capsense_mark_update(idx);
}

int16_t capsense_get_touch_sensitivity(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    return g_update.sensitivity_steps[idx];
}

void capsense_set_touch_sensitivity(uint8_t idx, int16_t sensitivity_steps)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return;
    }
    // clamp幅度：[-MAX, +MAX]；正偏移在FULL模式下维持最小步进规则
    if (sensitivity_steps > (int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
        sensitivity_steps = (int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
    } else if (sensitivity_steps < -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
        sensitivity_steps = -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
    }
    if (sensitivity_steps > 0 && sensitivity_steps < (int16_t)TOUCH_INCREMENT_MIN_STEPS) {
        sensitivity_steps = (int16_t)TOUCH_INCREMENT_MIN_STEPS;
    }
    g_update.sensitivity_steps[idx] = sensitivity_steps;
    // 异步：仅登记待更新位，在主循环应用
    capsense_mark_update(idx);
}

uint16_t capsense_sensitivity_to_raw_count(int16_t steps)
{
    // 同步clamp：保持与set函数一致
    if (steps > (int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
        steps = (int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
    } else if (steps < -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
        steps = -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
    }
    if (steps > 0 && steps < (int16_t)TOUCH_INCREMENT_MIN_STEPS) {
        steps = (int16_t)TOUCH_INCREMENT_MIN_STEPS;
    }
    int32_t raw = (int32_t)TOUCH_SENSITIVITY_ZERO_BIAS + (int32_t)steps;
    if (raw < (int32_t)TOUCH_SENSITIVITY_RAW_MIN) raw = (int32_t)TOUCH_SENSITIVITY_RAW_MIN;
    if (raw > (int32_t)TOUCH_SENSITIVITY_RAW_MAX) raw = (int32_t)TOUCH_SENSITIVITY_RAW_MAX;
    return (uint16_t)raw;
}

int16_t capsense_raw_count_to_sensitivity(uint16_t raw)
{
    int32_t steps = (int32_t)raw - (int32_t)TOUCH_SENSITIVITY_ZERO_BIAS;
    if (steps > (int32_t)TOUCH_SENSITIVITY_MAX_STEPS) steps = (int32_t)TOUCH_SENSITIVITY_MAX_STEPS;
    if (steps < -(int32_t)TOUCH_SENSITIVITY_MAX_STEPS) steps = -(int32_t)TOUCH_SENSITIVITY_MAX_STEPS;
    if (steps > 0 && steps < (int32_t)TOUCH_INCREMENT_MIN_STEPS) steps = (int32_t)TOUCH_INCREMENT_MIN_STEPS;
    return (int16_t)steps;
}

// 读取总触摸电容（Cp基数 + 增量设置），单位步进（0.01 pF），并按22pF上限夹取
uint16_t capsense_get_total_touch_cap(uint8_t idx)
{
    if (idx >= CAPSENSE_WIDGET_COUNT) {
        return 0;
    }
    uint16_t cp = g_cp_steps[idx];
    int16_t add = g_update.sensitivity_steps[idx];
#if (CY_CAPSENSE_SMARTSENSE_FULL_EN)
    // FULL模式：不叠加增量，仅返回Cp
    int32_t total = (int32_t)cp;
#else
    int32_t total = (int32_t)cp + (int32_t)add;
#endif
    if (total < 0) {
        total = 0;
    } else if (total > (int32_t)TOUCH_CAP_TOTAL_MAX_STEPS) {
        total = (int32_t)TOUCH_CAP_TOTAL_MAX_STEPS;
    }
    return (uint16_t)total;
}

// 新增：读取Cp基数（单位步进，0.01 pF），供I2C绝对模式换算
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
                    CY_CAPSENSE_PROCESS_CALC_NOISE |
                    CY_CAPSENSE_PROCESS_DIFFCOUNTS |
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
    uint16_t bitmap = 0;
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        bitmap |= (Cy_CapSense_IsWidgetActive(widget_ids[i], &cy_capsense_context) ? (1u << i) : 0u);
    }
    g_touch_status_bitmap = bitmap;
}

void capsense_apply_threshold_changes(void)
{
    // 仅在有待更新位时应用；避免在扫描过程中修改
    uint16_t pending = capsense_consume_updates();

    if (pending == 0u) {
        return;
    }

    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        if (pending & (1u << i)) {
            int16_t add_steps = g_update.sensitivity_steps[i];
            // clamp幅度
            if (add_steps > (int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
                add_steps = (int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
            } else if (add_steps < -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS) {
                add_steps = -(int16_t)TOUCH_SENSITIVITY_MAX_STEPS;
            }
            if (add_steps > 0 && add_steps < (int16_t)TOUCH_INCREMENT_MIN_STEPS) {
                add_steps = (int16_t)TOUCH_INCREMENT_MIN_STEPS;
            }
#if (CY_CAPSENSE_SMARTSENSE_FULL_EN)
            // FULL模式：不对 fingerCap 做增量更新（由库/SmartSense管理）
#else
            int32_t total_steps = (int32_t)g_cp_steps[i] + (int32_t)add_steps;
            if (total_steps < 0) {
                total_steps = 0;
            } else if (total_steps > (int32_t)TOUCH_CAP_TOTAL_MAX_STEPS) {
                total_steps = (int32_t)TOUCH_CAP_TOTAL_MAX_STEPS;
            }
            cy_capsense_context.ptrWdContext[i].fingerCap = (uint16_t)(total_steps * 10u);
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
            Cy_CapSense_UpdateCrcWidget(widget_ids[i], &cy_capsense_context);
#endif
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

    // 按当前设置增量应用总触摸电容上限，并回写到fingerCap
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        int32_t total_steps = (int32_t)g_cp_steps[i] + (int32_t)g_update.sensitivity_steps[i];
        if (total_steps < 0) {
            total_steps = 0;
        } else if (total_steps > (int32_t)TOUCH_CAP_TOTAL_MAX_STEPS) {
            total_steps = (int32_t)TOUCH_CAP_TOTAL_MAX_STEPS;
        }
        cy_capsense_context.ptrWdContext[i].fingerCap = (uint16_t)(total_steps * 10u);
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
        // 可根据cp_ff进行记录或调试
    }
}
#endif