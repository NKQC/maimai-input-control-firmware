/******************************************************************************
* File Name: capsense_module.h
*
* Description: CapSense模块头文件
*
*******************************************************************************/

#ifndef CAPSENSE_MODULE_H
#define CAPSENSE_MODULE_H

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_capsense.h"

/*******************************************************************************
* 宏定义
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY    (3u)
#define CAPSENSE_WIDGET_COUNT     (12u)

/*******************************************************************************
* 全局变量声明
*******************************************************************************/
extern uint16_t g_touch_status_bitmap;
extern uint16_t g_cap_thresholds[12];
extern bool g_threshold_changed;

/*******************************************************************************
* 函数声明
*******************************************************************************/
void capsense_init(void);
void capsense_process_widgets(void);
void capsense_update_touch_status(void);
void capsense_apply_threshold_changes(void);
void capsense_start_scan(void);
bool capsense_is_busy(void);

#if CY_CAPSENSE_BIST_EN
void capsense_measure_sensor_cp(void);
#endif /* CY_CAPSENSE_BIST_EN */

#endif /* CAPSENSE_MODULE_H */