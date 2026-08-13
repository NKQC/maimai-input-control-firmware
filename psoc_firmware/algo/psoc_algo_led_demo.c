/******************************************************************************
 * File Name: psoc_algo_led_demo.c
 *
 * Description: 纯"白灯演示"触控算法(ABI v1)。用途: 直观展示 JIT 下发算法对 PSoC
 *              硬件(白色 LED)的【绝对可控性】——白灯完全由本算法写出的 out_active
 *              驱动: 某通道被触摸即令其 out_active=1(白灯亮), 松开即 0(灭)。
 *              换成别的算法后白灯行为随之改变, 证明"触摸点亮白灯"是算法(而非固件
 *              写死)的一部分。
 *
 * 约束同 psoc_algo_default.c(裸跑于 1KB RAM 槽): 无 '/'、'%'、64 位运算、外部符号,
 * algo() 必须是 .text 首符号。本算法极简, 天然满足。
 *
 * 构建/校验只经 dev.ps1 build-blob 工具链, 勿手工调用编译器。
 ******************************************************************************/

#include <stddef.h>
#include "psoc_algo_abi.h"

/* 上位机 schema(宏展开为空, 被上位机 grep 出以建折线/可调项)。 */
/* 别名/简介一律不进固件, 由上位机按英文 name 本地化(见 psoc_algo_default.c 同处说明)。 */
ALGO_REPORT_META(0, "diff", "u16", 0, 65535, "", "")
ALGO_REPORT_META(1, "active", "bool", 0, 1, "", "")
ALGO_REPORT_META(2, "led", "bool", 0, 1, "", "")

/* ABI 入口, 必须是 .text 首符号(本文件仅此一个函数)。 */
void algo(algo_io_t *io)
{
    /* 演示逻辑极简: 直接把中间件基础激活判定(base_active, 即"是否被触摸")作为最终触发。 */
    io->out_active = (io->base_active != 0u) ? 1u : 0u;

    /* ★点灯必须显式请求★: 固件已不再从 out_active 推导灯态(那是写死的触控反馈, 已删除),
     * 白灯只在算法写 out_led 时亮。本演示把它接到触发判定上, 于是"摸即亮、松即灭"这一行为
     * 完全属于本算法; 换成不写 out_led 的算法(如默认 v3.1 HDR), 白灯就不再随触摸闪。 */
    io->out_led = (uint16_t)io->out_active;

    /* 上报供 UI 可视化: diff 信号 + 触发判定 + 灯请求。 */
    io->report[0] = io->diff;
    io->report[1] = (uint16_t)io->out_active;
    io->report[2] = io->out_led;
}

/* [] END OF FILE */
