/******************************************************************************
 * File Name:   main.c
 *
 * Description: PSoC 4 SPI Slave with CAPSENSE and Status LED
 * - SPI Slave: SCB0 (SLAVE, MODE0, 8-bit, MSB-first, CS ActiveLow)
 * - CAPSENSE: 36 buttons, immutable double-buffered snapshots
 * - Status LED: P1.6 (CYBSP_LED_SLD3)
 * - LINK v2 (psoc_link_abi.h): 16 字节自证身份帧(SOF+tag+st/cmd+定长体+CRC8) + DMA 自己走的
 *   TX 整帧环(软件退出发送路径) + 非幂等命令的 tag 去重。
 *   帧格式/命令码/状态位/容量常量一律取自该头文件, 本文件不再自定义任何协议数字。
 * - FW_VERSION: 编译时间戳 YYMMDDHHMM(十进制, 本地时间), 由 Makefile PREBUILD 生成
 *   fw_build_stamp.h 提供; 上位机补 "20" 前缀还原 YYYYMMDDHHMM
 * - JIT algo engine: 4KB executable RAM slot (ABI v1, psoc_algo_abi.h),
 *   uploaded directly into the slot via ALGO_BEGIN/PAGE/END/INFO SPI commands;
 *   16KB PSoC RAM cannot afford a second staging slot, so CRC16-CCITT-FALSE
 *   is verified in the main loop before accepting the direct-write contents.
 *   Falls back to Cy_CapSense_IsWidgetActive when no valid algo is loaded.
 ******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_capsense.h"
/* ★拆分后的分层(src/)★ main.c 只保留 main()、启动序列、1ms 时基与主循环的阶段编排:
 *   link/lnk_wire      线级 SCB+DMAC 环、字节流取帧、TX 环、INT1/INT2(不含任何业务语义)
 *   link/lnk_dispatch  信封 -> 业务: 命令 switch、tag 去重、响应入队
 *   csd/csd_params     逐通道/全局 CSD 参数影子、启用位图、长操作请求位
 *   csd/csd_scan       扫描发起、处理链、触控帧、快照发布与锁存
 *   csd/csd_ops        APPLY/CALIBRATE/BASELINE/GLOBAL_COMMIT/MEASURE_CP 串行状态机
 *   csd/csd_autotune   频率自适应状态机与进度回报
 *   algo/algo_engine   JIT 算法槽、分页上传、CRC16 commit、共享堆、cfg/rom/trace
 *   diag/diag_export   spi_dbg 带外 SWD 导出块、lnk_diag、g_boot_override、指示灯引脚
 * 阶段顺序、进入条件与 spi_dbg.stage 打点值与拆分前逐条一致。 */
#include "app_tick.h"
#include "diag_export.h"
#include "lnk_wire.h"
#include "lnk_dispatch.h"
#include "csd_params.h"
#include "csd_scan.h"
#include "csd_ops.h"
#include "csd_autotune.h"
#include "algo_engine.h"
#include <stdint.h>

#define CY_ASSERT_FAILED                 (0u)

/* 唯一写者是 systick_ms_callback; 各层只读, 声明见 app_tick.h。 */
volatile uint32_t g_ms_tick = 0u;


/* SysTick 1ms 回调：唯一的 g_ms_tick 写者，运行时真实毫秒时间戳来源。 */
static void systick_ms_callback(void)
{
    g_ms_tick++;
}

static void initialize_ms_tick(void)
{
    /* SysTick 用 CPU 时钟，1ms 周期：reload = SystemCoreClock/1000 - 1。 */
    uint32_t reload = (SystemCoreClock / 1000u);
    if (reload > 0u) { reload -= 1u; }
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, reload);
    (void)Cy_SysTick_SetCallback(0u, systick_ms_callback);
}

int main(void)
{
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* ★启动复位改为逐模块 .clear()★ 每个模块清自己拥有的状态, 语句逐字照搬(全部发生在
     * __enable_irq() 之前, 互不相干的独立写入, 顺序无关)。 */
    csd_scan_reset();
    spi_dbg_clear();
    lnk_diag_clear();
    lnk_wire_reset();
    lnk_dispatch_reset();
    csd_ops_reset();
    csd_params_reset();
    csd_autotune_reset();
    algo_reset();

    __enable_irq();
    initialize_ms_tick();
    initialize_common_cfg_shadow();   /* 必须在 initialize_capsense 前: 重定向 ptrCommonConfig 到 RAM 影子 */
    initialize_capsense();
    spi_slave_init();
    /* 新代数通知线: 显式定成强推挽输出并从低电平起步。BSP 生成代码已经初始化过 P1.4,
     * 但那是"顺带"的(它把这脚当 LED), 显式一次才不会随 BSP 重新生成漂掉。
     * 起始电平不重要 —— RP2040 只看变化, 它自己会记住第一次采到的电平。 */
    Cy_GPIO_Pin_FastInit(SENSOR_INT1_PORT, SENSOR_INT1_NUM, CY_GPIO_DM_STRONG_IN_OFF, 0u,
                         HSIOM_SEL_GPIO);
    /* ★流控电平线(I3)★ 与 INT1 同构的强推挽输出, 但★初值必须是高★: 刚复位时 RX 环是空的、空余
     * 最大, 给低会让主机一直不敢发帧, 链路根本起不来。此后由 lnk_rx_drain 每轮按空余帧数刷。 */
    Cy_GPIO_Pin_FastInit(SENSOR_INT2_PORT, SENSOR_INT2_NUM, CY_GPIO_DM_STRONG_IN_OFF,
                         SENSOR_INT2_INIT_STATE, HSIOM_SEL_GPIO);
    // hardware.txt: P1.6 -> LED -> R -> GND，因此高电平明确为点亮。
    // 白灯启动即点亮作为 bring-up 指示；800ms 后仅由已加载 JIT 算法的判定结果驱动，
    // 原始 CapSense 触控不会触发白灯，避免把非 JIT 模式误呈现为算法已触发。
    Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM, STATUS_LED_ON_STATE);
    /* ★可见启动指示(修"重启白灯不亮")★: 白灯在启动后保持点亮 800ms 再交给 g_any_active 驱动,
     * 使每次上电/XRES 重启都有肉眼可见的白灯闪亮(否则仅亮几微秒无法察觉, 用户误判"重启无效")。 */
    uint32_t led_boot_until_ms = g_ms_tick + 800u;

    for (;;)
    {
        /* ★链路泵: 整个 v3 的命令入口就这一处★ 放在 for(;;) 顶部而不是 NOT_BUSY 分支里 —— 扫描
         * 进行中主循环在这一层空转, 迭代频率远高于"每完成一次扫描"的约 170Hz, 于是响应外发速率
         * 由 TX 环可写窗口(= 线速)决定, 而不是由扫描周期决定。
         * 它同时负责: 消费请求 / 执行命令 / 把响应写进 TX 环 / 按空余帧数刷 INT2 流控电平。 */
        (void)lnk_rx_drain();
        algo_upload_watchdog();
        spi_dbg.stage = MLOOP_STAGE_WAIT_SCAN;
        spi_dbg.ms_tick_m = g_ms_tick;   /* 无条件刷新: 卡在等扫描时也能看出时间在走 */
        spi_dbg.clk_now = cy_capsense_tuner.widgetContext[0].snsClk;   /* 当前生效分频 */
        provision_timeout_fallback();
        /* SPI frames are moved by DMAC; 命令分发在 lnk_rx_drain(主循环), ISR 只推进 RX 写位置。 */
        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* ★长操作进行中: 本轮只推进一步, 不处理/不发布/不启动新扫描★
             * 不启动扫描是硬性要求, 不只是省事: MEASURE_CP 的 BIST 会整块接管 CSD 硬件, 中间插一次
             * ScanAllWidgets 既会拿到无意义的 raw 也可能把 BIST 的轮询搅坏; 校准类操作同理 —— v2
             * 那段是整体原子执行的, 中间不存在扫描, 保持一致才能满足"不改变最终硬件效果"。
             * 步前步后各泵一次链路: 单步上界约 200~270ms, 这样窗口至少以每步两次的粒度推进。 */
            if (op_serial_active())
            {
                (void)lnk_rx_drain();
                op_serial_step();
                (void)lnk_rx_drain();
                continue;
            }
            spi_dbg.stage = MLOOP_STAGE_PROCESS;
            csd_process_widgets();
            spi_dbg.stage = MLOOP_STAGE_TOUCH;
            update_touch_frame();       /* 重算触控掩码并重封主动状态帧 */
            /* 白 LED: 启动 800ms 内常亮(可见启动指示)；之后只有算法显式写 out_led 才亮。 */
            Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM,
                          (g_any_active || (g_ms_tick < led_boot_until_ms))
                              ? STATUS_LED_ON_STATE : STATUS_LED_OFF_STATE);
            spi_dbg.stage = MLOOP_STAGE_PUBLISH;
            publish_capsense_snapshot();/* 刷新完整 raw/baseline/diff（调参慢路数据源） */
            /* ★状态帧保鲜与响应外发都由 lnk_rx_drain 内的 lnk_tx_feed 负责★ 这里原先要额外补一次
             * lnk_tx_feed 并套临界区(TX 环曾被 RX 完成 ISR 并发改写); v3 里发送路径只有主循环一个
             * 写者, 且泵在 for(;;) 顶部每轮都跑, 故这两件事都不再需要。 */

            if (op_begin_measure_cp()) { continue; }

            algo_commit_step();

            op_global_apply_step();

            if (op_begin_apply()) { continue; }

            op_quick_apply_step();

            if (op_begin_calibrate()) { continue; }

            op_baseline_reset_step();

            if (at_begin()) { continue; }

            op_busy_release_step();

            /* ★通道启用/禁用在此落实★ 放在所有重操作之后、启动下一轮扫描之前:
             * 此刻 NOT_BUSY 成立(Cy_CapSense_SetWidgetStatus 内部的 SwitchSensingMode 要求),
             * 且新的启用集会立刻对下面这一次 ScanAllWidgets 生效(禁用的 widget 从此不再被 setup,
             * 电极停在 High-Z; 新启用的 widget 已完成校准+基线)。
             * ★长操作期间到不了这里★(上面 active 分支直接 continue) —— 必须如此: 它内部会
             * SetWidgetStatus + 校准 + 立基线, 插在某个长操作的逐通道推进中间就会改变那个操作
             * 覆盖的通道集与硬件状态, 违背"拆分不改变最终硬件效果"。 */
            ch_enable_apply();

            csd_scan_start_step();
        }
    }
}

/* [] END OF FILE */
