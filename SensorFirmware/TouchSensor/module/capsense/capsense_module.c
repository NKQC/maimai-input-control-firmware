#include "capsense_module.h"

static uint16_t g_touch_status_bitmap = 0;
static uint16_t g_cap_thresholds[12] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
static bool g_threshold_changed = false;

#if CY_CAPSENSE_BIST_EN
static uint32_t sensor_cp = 0;
static cy_en_capsense_bist_status_t status;
#endif /* CY_CAPSENSE_BIST_EN */

static void _capsense_isr(void);

// 初始化CapSense模块
void capsense_init(void)
{
    static cy_capsense_status_t status_local; 
    status_local = CY_CAPSENSE_STATUS_SUCCESS;

    static const cy_stc_sysint_t capsense_intr_config =
    {
        .intrSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    status_local = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status_local)
    {
        Cy_SysInt_Init(&capsense_intr_config, _capsense_isr);
        NVIC_ClearPendingIRQ(capsense_intr_config.intrSrc);
        NVIC_EnableIRQ(capsense_intr_config.intrSrc);

        status_local = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status_local != CY_CAPSENSE_STATUS_SUCCESS)
    {
    }
}

// CapSense中断处理
static void _capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CSD0, &cy_capsense_context);
}

// 处理CapSense控件
inline void capsense_process_widgets(void)
{
    Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
}

// 更新触摸状态位图
inline void capsense_update_touch_status(void)
{
    g_touch_status_bitmap = 0;

    static uint8_t i;
    for (i = 0; i < 12; i++)
    {
        if (Cy_CapSense_IsWidgetActive(i, &cy_capsense_context))
        {
            g_touch_status_bitmap |= (1 << i);
        }
    }
}

// 获取触摸状态位图
inline uint16_t capsense_get_touch_status_bitmap(void)
{
    return g_touch_status_bitmap;
}

// 应用阈值更改
void capsense_apply_threshold_changes(void)
{
    if (g_threshold_changed)
    {
        static const uint32_t widget_finger_th_param_ids[CAPSENSE_WIDGET_COUNT] = {
            CY_CAPSENSE_CAP0_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP1_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP2_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP3_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP4_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP5_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP6_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP7_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP8_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAP9_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAPA_FINGER_TH_PARAM_ID,
            CY_CAPSENSE_CAPB_FINGER_TH_PARAM_ID
        };

        static uint8_t i;
        for (i = 0; i < CAPSENSE_WIDGET_COUNT; i++)
        {
            Cy_CapSense_SetParam(widget_finger_th_param_ids[i], g_cap_thresholds[i], &cy_capsense_tuner, &cy_capsense_context);
        }
        g_threshold_changed = false;
    }
}

// 开始CapSense扫描
void capsense_start_scan(void)
{
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
}

// 检查CapSense是否忙碌
bool capsense_is_busy(void)
{
    return (CY_CAPSENSE_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context));
}

// 阈值访问器
uint16_t capsense_get_threshold(uint8_t idx)
{
    return g_cap_thresholds[idx];
}

void capsense_set_threshold(uint8_t idx, uint16_t value)
{
    g_cap_thresholds[idx] = value;
    g_threshold_changed = true;
}

#if CY_CAPSENSE_BIST_EN
// 测量传感器电极自电容
void capsense_measure_sensor_cp(void)
{
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_CAP0_WDGT_ID,
                                                  CY_CAPSENSE_CAP0_SNS0_ID,
                                             &sensor_cp, &cy_capsense_context);
}
#endif /* CY_CAPSENSE_BIST_EN */