/*******************************************************************************
 * csd_autotune.h —— 频率自适应(AUTO_TUNE)状态机与进度回报
 *
 * ★状态归属★ _at_task 与 auto_tune_* 上报量归本模块。长操作编排(csd_ops)只经 at_active()/
 *   at_step() 使用它, 本模块不反向依赖 csd_ops(空闲相位码用同值的 AT_PH_IDLE)。
 ******************************************************************************/
#ifndef CSD_AUTOTUNE_H
#define CSD_AUTOTUNE_H

#include "psoc_link_abi.h"
#include <stdint.h>
#include <stdbool.h>

#define AUTO_TUNE_PHASE_IDLE             (0u)
#define AUTO_TUNE_PHASE_COARSE           (1u)
#define AUTO_TUNE_PHASE_FINE             (2u)
#define AUTO_TUNE_PHASE_SETTLE           (3u)
#define AUTO_TUNE_PHASE_DONE             (4u)
#define AUTO_TUNE_STEP_MAX_REPORT        (31u)

extern volatile bool auto_tune_pending;
extern volatile uint8_t auto_tune_result;
extern volatile uint16_t auto_tune_div;
extern volatile uint8_t auto_tune_ch;
extern volatile uint8_t auto_tune_pref;
extern volatile uint8_t auto_tune_phase;
extern volatile uint8_t auto_tune_step;

/* 本状态机是否在跑(长操作编排的判据之一)。 */
bool at_active(void);
void at_step(void);
/* 主循环阶段: 受理 AUTO_TUNE(返回 true = 本轮 continue)。 */
bool at_begin(void);
void csd_autotune_reset(void);

#endif /* CSD_AUTOTUNE_H */
