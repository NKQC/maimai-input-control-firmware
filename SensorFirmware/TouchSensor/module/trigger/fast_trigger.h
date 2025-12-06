#ifndef FAST_TRIGGER_H
#define FAST_TRIGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "../capsense/capsense_module.h"

// 基于下一个期望状态的触发算法
#define FAST_TRIG_BASE_NEXT_STATE 1

#ifndef FAST_TRIG_BASE_NEXT_STATE
// 快速触发状态机触发窗口（毫秒） 仅在基础状态下生效
#define FAST_TRIG_WINDOW_MS           (200u)

#endif

// 触发改变阈值千分比（基于基线的千分比），抬起与下降分开定义
#define FAST_TRIG_RISE_PERMILLE_DEFAULT  (10u)
#define FAST_TRIG_DROP_PERMILLE_DEFAULT  (30u)

#define FAST_TRIG_INVALID_HIGH        (0)
#define FAST_TRIG_INVALID_LOW         (65535)

void     fast_trigger_init(void);
uint16_t fast_trigger_process(uint64_t now_ms, uint16_t base_status);

void fast_trigger_set_enable_mask(uint16_t mask);
void fast_trigger_set_rise_permille(uint16_t p);
void fast_trigger_set_drop_permille(uint16_t p);

uint16_t fast_trigger_get_enable_mask(void);
uint16_t fast_trigger_get_rise_permille(void);
uint16_t fast_trigger_get_drop_permille(void);

#endif