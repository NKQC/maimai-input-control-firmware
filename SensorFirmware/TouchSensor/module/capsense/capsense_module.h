#ifndef CAPSENSE_MODULE_H
#define CAPSENSE_MODULE_H

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_capsense.h"

#define CAPSENSE_INTR_PRIORITY    (3u)
#define CAPSENSE_WIDGET_COUNT     (12u)

extern uint16_t g_touch_status_bitmap;
extern uint16_t g_cap_thresholds[12];
extern bool g_threshold_changed;

void capsense_init(void);
void capsense_process_widgets(void);
void capsense_update_touch_status(void);
void capsense_apply_threshold_changes(void);
void capsense_start_scan(void);
bool capsense_is_busy(void);

#if CY_CAPSENSE_BIST_EN
void capsense_measure_sensor_cp(void);
#endif

#endif /* CAPSENSE_MODULE_H */