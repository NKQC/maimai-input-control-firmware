#ifndef FAST_TRIGGER_H
#define FAST_TRIGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "../capsense/capsense_module.h"

// 基于下一个期望状态的触发算法
#define FAST_TRIG_BASE_NEXT_STATE 1

#ifndef FAST_TRIG_BASE_NEXT_STATE
// 快速触发状态机触发窗口（毫秒） 仅在基础状态下生效
#define FAST_TRIG_WINDOW_MS           (5u)

#endif

// 触发改变阈值千分比 （基于基线的千分比）
#define FAST_TRIG_X_PERMILLE_DEFAULT  (70u)             // 阈值千分比

#define FAST_TRIG_INVALID_HIGH        (0)
#define FAST_TRIG_INVALID_LOW         (16383)

void     fast_trigger_init(void);
uint16_t fast_trigger_process(uint64_t now_ms, uint16_t base_status);

void fast_trigger_set_x_permille(uint16_t permille);
void fast_trigger_set_enable_mask(uint16_t mask);

uint16_t fast_trigger_get_x_permille(void);
uint16_t fast_trigger_get_enable_mask(void);

#endif