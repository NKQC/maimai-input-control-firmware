/*******************************************************************************
 * algo_engine.h —— JIT 可加载触控算法引擎(ABI v1, psoc_algo_abi.h)
 *
 * ★依赖方向★ 本模块只依赖线级(lnk_st)、诊断与时基, 不认识任何 csd_* 模块; 反过来
 *   csd_scan(执行算法)与 csd_params(禁用通道时清算法现场)包含本头 —— 单向。
 * 4KB 可执行槽与 256B 共享堆是本模块私有(algo_slot / algo_heap 不外泄)。
 ******************************************************************************/
#ifndef ALGO_ENGINE_H
#define ALGO_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
/* psoc_algo_abi.h 的布局静态断言用 size_t(该头是两端共用契约, 不许改) => 先给出 stddef。
 * 原先它在 main.c 里排在 cy_pdl.h 之后才被间接满足。 */
#include <stddef.h>
#include "psoc_algo_abi.h"
#include "psoc_link_abi.h"

extern volatile bool algo_valid;
extern volatile bool algo_upload_active;
extern volatile uint16_t algo_len;
extern volatile uint16_t algo_expected_len;
extern volatile uint16_t algo_slot_crc;
extern volatile uint16_t algo_reject_count;
extern volatile uint16_t algo_heap_used_peak;
/* 逐通道持久 IO 记录 / 上一周期 base_active / 每通道 ROM / 共享 cfg / 逐通道 cfg。 */
extern algo_io_t g_algo_io[LNK_CHANNEL_COUNT];
extern uint8_t algo_prev_active[LNK_CHANNEL_COUNT];
extern volatile uint16_t g_algo_rom[LNK_CHANNEL_COUNT];
extern volatile uint8_t g_algo_cfg[8];
extern uint8_t g_algo_cfg_ch[LNK_CHANNEL_COUNT][8];

uint32_t algo_engine_run_channel(uint32_t ch, uint32_t base_active);
uint16_t cmd_algo_begin(uint16_t len);
bool cmd_algo_page(uint32_t page, const uint8_t * data, uint32_t * out_offset);
void cmd_algo_end(uint16_t crc);
/* 主循环阶段: 上传断流自愈 与 ALGO_END 的全量 CRC16 commit。 */
void algo_upload_watchdog(void);
void algo_commit_step(void);
void algo_reset(void);

#endif /* ALGO_ENGINE_H */
