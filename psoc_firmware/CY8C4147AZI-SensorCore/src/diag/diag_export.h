/*******************************************************************************
 * diag_export.h —— 带外 SWD 导出块 + 链路诊断计数 + 指示灯引脚
 *
 * 本模块是最底层: 不依赖任何其它模块。spi_dbg 的字段布局是 RP2040 经 SWD 按偏移读取的
 * 契约(见下面注释), 故 struct 与 magic 一律不许动。
 ******************************************************************************/
#ifndef DIAG_EXPORT_H
#define DIAG_EXPORT_H

#include "cy_pdl.h"
#include "cybsp.h"
#include <stdint.h>


/* SPI 链路诊断计数(只读, 经 GET_GLOBAL 的 GPARAM_DBG_* 上报)。同一功能组归拢成 struct,
 * 避免散装全局量; 需要整体归零时用 spi_dbg_clear()。 */
/* ★带外 SWD 取数用的定位 magic★: 带内 GET_GLOBAL 在链路故障态读不出来(实测全 None), 故本块必须
 * 能被 RP2040 经 SWD 直接读。为免改 BSP 链接脚本(生成文件, 改了脆), 不固定地址, 而是在块首放一对
 * magic 字, 由 RP2040 扫描 SRAM(0x20000000..0x20004000, 16KB) 一次定位后缓存地址。 */
#define SPI_DBG_MAGIC0                   (0x53504442u)   /* "SPDB" */
#define SPI_DBG_MAGIC1                   (0x4C4E4B31u)   /* "LNK1" */

/* 主循环阶段码: 每进入一个可能长耗时的段就写一次, 挂死时停在肇事段上(带外 SWD 读槽3)。
 * 20+ 是 APPLY 分支内部的细分阶段, 用来把"APPLY 耗时 13s"落到具体哪一步。 */
/* 1、2、20 已退役, 号一律不回填 —— 历史 SWD 日志里的这些值应当仍指旧固件的那个阶段, 复用会让
 * 旧记录被误读。1 = 原 MLOOP_STAGE_TOP(从来没被赋值过); 2 = 原 MLOOP_STAGE_LATCH(随"钉住缓冲"
 * 取代"拷一份"一起消失); 20 = 原 APPLY_ENABLE(逐通道串行化后 APPLY 不再有独立的启用阶段)。 */
#define MLOOP_STAGE_WAIT_SCAN            (3u)   /* 等 Cy_CapSense_IsBusy 变 NOT_BUSY */
#define MLOOP_STAGE_PROCESS              (4u)
#define MLOOP_STAGE_TOUCH                (5u)
#define MLOOP_STAGE_PUBLISH              (6u)
#define MLOOP_STAGE_MEASURE_CP           (7u)
#define MLOOP_STAGE_ALGO_COMMIT          (8u)
#define MLOOP_STAGE_GLOBAL_APPLY         (9u)
#define MLOOP_STAGE_APPLY                (10u)
#define MLOOP_STAGE_CALIBRATE            (11u)
#define MLOOP_STAGE_BASELINE_RESET       (12u)
#define MLOOP_STAGE_AUTO_TUNE            (13u)
#define MLOOP_STAGE_SCAN_START           (14u)
#define MLOOP_STAGE_QUICK_APPLY          (15u)
/* APPLY 分支细分 */
#define MLOOP_STAGE_APPLY_RECAL          (21u)  /* APPLY 的逐通道校准阶段(_apply_step OP_PHASE_CH) */
#define MLOOP_STAGE_APPLY_INIT           (22u)  /* Cy_CapSense_Initialize */
#define MLOOP_STAGE_APPLY_BASELINE       (23u)  /* Cy_CapSense_InitializeAllBaselines */

/* ★前 8 个 volatile 字段(block+8 .. block+36)是 RP2040 经 SWD 读走的"导出槽"★
 * psoc_swd.cpp:823 按 `_dbg_block_addr + 8u + i*4` 连读 DEBUG_COUNTER_WORDS(=8) 个字,
 * 顺序即上报顺序 ⇒ 这 8 个字段的类型与次序不许动, 新字段只能追加在它们之后。
 * ★槽位内容已换代★: 帧对齐问题已由实测证伪(bad_magic 连续两轮为 0, 帧边界完全正确), 故把导出槽
 * 让给当前真正待查的问题 —— PSoC 主循环是否活着、是否在扫描、是否在发布快照。
 * v1 的 rx_bad_magic / cs_resync / rx_leftover 已退役: 它们在导出范围之外(RP 读不到), 现由
 * lnk_diag 承载并经 LNK_CMD_DIAG / GLOBAL_GET 带内上报。 */
typedef struct
{
    uint32_t magic0;
    uint32_t magic1;
    /* ★槽0/4/5/6 已换代为"钉死 snsClk 被谁改回 8"的四件套★(rx_frames/apply_cmd/apply_last_ms/
     * apply_dirty 的使命已完成: 链路与 APPLY 风暴都已修复且有 scan/ms/stage/setparam 可继续监视)。
     * 判据:
     *   clk_boot==32 且 clk_now==8 且 clk_set_cnt>0  ⇒ 是 SET_PARAM 推下来的(store 侧问题)
     *   clk_boot==32 且 clk_now==8 且 clk_set_cnt==0 ⇒ 是中间件内部路径改的(Init/Initialize/Enable)
     *   clk_boot!=32                                 ⇒ 启动归一根本没生效 */
    volatile uint32_t clk_boot;       /* 槽0: 启动 normalize 之后立刻采样的 widgetContext[0].snsClk */
    volatile uint32_t scan_count_m;   /* 槽1: scan_count 镜像(主循环每完成一次全通道扫描 +1) */
    volatile uint32_t ms_tick_m;      /* 槽2: g_ms_tick 镜像(SysTick 毫秒, 0 说明主循环/时基死了) */
    /* 槽3: 主循环阶段码(见 MLOOP_STAGE_*)。主循环挂死时它就停在肇事阶段上, 带外 SWD 一读即知。 */
    volatile uint32_t stage;
    /* ★钉死 APPLY 问题的四个关键量★
     * 槽4/槽7 回答"帧到底有没有反复到达"(ISR 计数, 与主循环无关):
     *   apply_cmd  持续增长 ⇒ RP2040 真的在反复发 APPLY, 去 RP2040 侧抓发送方;
     *   apply_cmd  恒为 1   ⇒ 没人重发, 那 stage=10 只能是 apply_pending 被别的途径置起。
     *   setparam_cmd 同步增长 ⇒ provision 整体在重复(不只是 APPLY)。
     * 槽5/槽6 回答"13s 花在哪": 上次 APPLY 实测耗时, 以及进入时的脏通道数(逐通道重校准的工作量)。 */
    volatile uint32_t clk_set_cnt;    /* 槽4: cmd_set_param 写 PARAM_SNS_CLK_DIV 的次数 */
    volatile uint32_t clk_set_last;   /* 槽5: 最后一次被写入的 snsClk 值(低16位) | 通道<<16 */
    volatile uint32_t clk_now;        /* 槽6: 每轮主循环采样的 widgetContext[0].snsClk(当前生效值) */
    volatile uint32_t setparam_cmd;   /* 槽7: ISR 收到 LNK_CMD_PARAM_SET 的次数 */
    volatile uint32_t snap_pub;       /* 导出范围之外, 仅供扩展读取范围时使用 */
} spi_dbg_t;
/* ★对象本身即对外接口★ spi_dbg 的每个字段都在多个模块的热路径上被直接赋值(阶段打点、
 * 计数镜像), 换成访问器会改写全部调用点并多出一层调用 —— 与"拆分后行为逐字节等价、体积
 * 持平"的判据冲突。故此处按对外接口暴露该对象(名字不带 `_`), 所有权仍唯一属本模块。 */
extern spi_dbg_t spi_dbg;


/* 链路诊断计数。u16 足够(秒级量级远小于 65535, 且 LNK_CMD_DIAG 一帧就是 4 个 u16)。
 * ★不放进 spi_dbg★ spi_dbg 的前 8 个 word 是 RP2040 经 SWD 按偏移读的导出槽, 语义已固定;
 * 链路计数带内可读(DIAG / GLOBAL_GET), 不需要占用带外槽位。 */
typedef struct
{
    uint16_t rx_ok;
    /* ★v3 语义★ 字节流解析器每滑过一个非帧首字节就 +1(不是"整帧被拒"的次数)。锁定一次最多滑
     * LNK_FRAME_SIZE-1 字节, 故它的量级是"发生过几次错位", 而不是"丢了多少帧"。 */
    uint16_t rx_reject;
    uint16_t respq_drop;
    /* ★语义已换代★ 原 tx_arm(TX 描述符重挂次数)随"软件退出发送路径"一起消失; 现在这个槽记的是
     * "可写窗口已用尽, 待发响应留在队里"的次数 —— 它是链路是否被主机抽干的直接指标。 */
    uint16_t tx_stale;
    /* ★cs_resync / rx_leftover 两个字段已删★ 它们记的是 CS 上升沿 ISR 的动作, 而那个 ISR 在 v3
     * 里整个不存在(见 I1) ⇒ 这两件事结构上不可能发生。GPARAM_DBG_CS_RESYNC /
     * GPARAM_DBG_RX_LEFTOVER 与 LNK_CMD_DIAG 的对应位置一律固定回 0(id 保留, 不动上位机列表)。 */
} lnk_diag_t;
extern lnk_diag_t lnk_diag;

/* 启动时固件强制改写过哪些生成配置项(位掩码, 经 GPARAM_BOOT_OVERRIDE 只读上报)。
 * 有值即说明"设备实际配置 != 用户/生成配置给的值", 上位机必须据此告警, 不许静默不同步。 */
extern uint8_t g_boot_override;

/* 白 LED(P1.6): 启动指示 + 算法显式点灯请求, 兼作 INDICATOR_ON 命令的落点。 */
#define STATUS_LED_PORT                  (CYBSP_LED_SLD3_PORT)
#define STATUS_LED_NUM                   (CYBSP_LED_SLD3_NUM)
#define STATUS_LED_ON_STATE              (1u)
#define STATUS_LED_OFF_STATE             (0u)

void spi_dbg_clear(void);
void lnk_diag_clear(void);

#endif /* DIAG_EXPORT_H */
