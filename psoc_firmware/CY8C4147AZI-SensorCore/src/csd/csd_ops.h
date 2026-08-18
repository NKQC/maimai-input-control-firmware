/*******************************************************************************
 * csd_ops.h —— 长操作串行状态机(APPLY / CALIBRATE / BASELINE_RESET / GLOBAL_COMMIT /
 *              MEASURE_CP)与它们的主循环编排
 *
 * ★状态归属★ 逐通道任务结构(_apply_task/_cal_task/_cp_task)与每通道 Cp 结果归本模块;
 *   "主机请求了什么"归 csd_params(见那里的说明)。依赖方向: csd_ops -> {csd_params,
 *   csd_autotune, lnk_wire, diag}, 反向无。
 ******************************************************************************/
#ifndef CSD_OPS_H
#define CSD_OPS_H

#include "psoc_link_abi.h"
#include <stdint.h>
#include <stdbool.h>


/* ============ 长操作逐通道串行状态机(psoc-link-v3.md 的 I4: 主循环延迟有界) ============
 * v2 的 APPLY / CALIBRATE / AUTO_TUNE / MEASURE_CP 都是"一次调用内跑完 36 通道"的阻塞实现
 * (APPLY 最坏 12~13s, AUTO_TUNE 全通道分钟级)。那期间主循环整段出局 ⇒ SPI 请求既不被消费也不被
 * 应答, 主机只能靠超时兜底 —— v2 每隔十几秒就在 AUTO_TUNE 期间掉链路正是这个场景。
 * v3 把它们摊平成"每轮主循环只推进一步"的显式状态机, 单步上界 = 一次 calibrate_widget_locked
 * (约 200~270ms, 中间件内部不可再分, 是本工程能给出的最小原子步)。
 * ★对主机的语义必须与 v2 逐位一致★
 *   · st.OP_BUSY 在整个操作跨越的所有轮次里保持 1, 只在最后一步做完才清 —— 主机的完成判据是
 *     OP_BUSY 1→0, 绝不能变成"每通道抖一次";
 *   · 通道处理顺序、每步参数、最终硬件效果一律不变, 只是把嵌套循环摊平;
 *   · 同一时刻只允许一个长操作在跑(见 op_serial_active), 其余 pending 排在它后面 ⇒ v2 里
 *     "同一轮先 APPLY 再 CALIBRATE"那种天然先后不会退化成逐通道交错;
 *   · 长操作期间不扫描、不发布快照、不落实通道启用改动 —— 硬件视角与 v2 那段"整段出局"一致。 */
#define OP_PHASE_IDLE                    (0u)
#define OP_PHASE_PRE                     (1u)   /* 一次性前置(重初始化 / 清结果表) */
#define OP_PHASE_CH                      (2u)   /* 逐通道推进, 每轮一个 */
#define OP_PHASE_FIN                     (3u)   /* 一次性收尾(锁定档回写 / 立基线 / 恢复扫描) */

/* 每通道最近一次寄生电容测量值(fF), 0xFFFFFF=失败/未测量。CP_GET 只读。 */
extern volatile uint32_t cp_value[LNK_CHANNEL_COUNT];

/* 重配之后必须先在新配置下真扫一遍再立基线(否则基线被钉在陈旧 raw 上)。 */
void scan_then_initialize_baselines(void);

/* 同一时刻只有一个长操作在跑; active 期间主循环只推进一步, 不处理/不发布/不启动扫描。 */
bool op_serial_active(void);
void op_serial_step(void);

/* 主循环阶段(返回 true = 本轮必须 continue, 与拆分前的控制流逐条一致)。 */
bool op_begin_measure_cp(void);
void op_global_apply_step(void);
bool op_begin_apply(void);
void op_quick_apply_step(void);
bool op_begin_calibrate(void);
void op_baseline_reset_step(void);
/* 本轮已把入队的重操作全部做完 => 清 st.OP_BUSY。 */
void op_busy_release_step(void);
void csd_ops_reset(void);

#endif /* CSD_OPS_H */
