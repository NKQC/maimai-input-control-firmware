/*******************************************************************************
 * lnk_wire.h —— 线级链路(SCB SPI 从机 + 两条 DMAC 整帧环)
 *
 * ★本模块不含任何业务语义★ 它只做: 收字节 -> 认帧 -> 交上层执行 -> 把响应/状态帧填进 TX 环。
 * ★I1 的边界就在这里★ Cy_SCB_SPI_ClearRx/TxFifo、Cy_DMAC_Descriptor_SetState、
 *   Cy_DMAC_Channel_SetCurrentDescriptor、Cy_DMAC_Channel_Enable 只允许出现在 spi_dma_init/
 *   spi_slave_init 里(见 psoc_link_abi.h 的 I1), 任何其它模块都不许触碰这几个 API。
 * ★上层回调以"声明"给出, 本模块不包含任何上层头文件★ 依赖方向因此严格单向:
 *   lnk_deliver()       由 link/lnk_dispatch 实现(拿到一整帧后执行并入队)
 *   update_touch_frame() 由 csd/csd_scan 实现(首帧之前状态帧就必须合法)
 ******************************************************************************/
#ifndef LNK_WIRE_H
#define LNK_WIRE_H

#include "cy_pdl.h"
#include "cybsp.h"
#include "psoc_link_abi.h"
#include <stdint.h>
#include <stdbool.h>


/* ★新代数就绪通知线★ P1.4 → RP2040 GPIO23 (SENSOR-INT1, 见 doc/hardware.txt)。
 * 每发布一份新快照就把电平**翻转**一次, 一次翻转 = 一份新代数。
 * 为什么是翻转而不是脉冲: RP2040 侧只需比较"电平与上次采到的是否不同"即可判定有新代数 ——
 * 不依赖脉宽(窄脉冲会被采样错过)、不需要任何清中断/握手、也不会因为漏采一次就永久失步。
 * 有了它, RP2040 的 core1 不必再以固定间隔空转轮询 SPI: 没有新代数时它什么都不做,
 * PSoC 的 CapSense 中间件因此拿回被 SPI DMA ISR 抢走的临界区时间。
 * P1.4 由 BSP 生成为强推挽输出(CYBSP_LED_SLD2, 生成代码只初始化、从不引用), 这里仍显式
 * FastInit 一次, 免得 BSP 重新生成时把驱动模式漂掉。 */
#define SENSOR_INT1_PORT                 (GPIO_PRT1)
#define SENSOR_INT1_NUM                  (4u)

/* ★INT1 只表示"已发布新一代快照"★ 它不兼任流控 —— 流控整个由 INT2 电平承担(I3), 故这里没有
 * 第二个来源, 唯一翻转点是 publish_capsense_snapshot。
 * RP2040 只比较"电平与上次采到的是否不同", 故翻转即事件, 不依赖脉宽、不需要握手。 */
static inline void lnk_int1_notify(void)
{
    Cy_GPIO_Inv(SENSOR_INT1_PORT, SENSOR_INT1_NUM);
}

/* ★流控电平线(I3)★ P1.5 → RP2040 GPIO22 (SENSOR-INT2, 见 doc/hardware.txt)。
 * 语义: RX 环空余 >= LNK_RX_SPACE_MIN 帧 ⇒ 高; 否则低。主机每次组批先读这条线, 高则本批最多
 * LNK_BURST_MAX 帧, 低则一个字节都不发。
 * ★它是电平, 不是脉冲★ 主机只看**当刻值**, 不看边沿 ⇒ 从机只许 Cy_GPIO_Write, 绝不许 Cy_GPIO_Inv:
 * 一次多余的翻转就会把"满"读成"空", 灌进一批溢出的帧。
 * ★为什么必须带外★ SPI 上主机是唯一时钟源, 任何"把余量搭在响应帧里回报"的方案在窗口满时都拿不到
 * 时钟 ⇒ 永远等不到窗口重开(信息学死锁, 见 psoc_link_abi.h 的 I3 推演)。这条线不需要时钟。
 * P1.5 在 BSP 里只是一个无名使能脚(ioss_0_port_1_pin_5), 没有现成的命名宏, 故与 INT1 同构自建。 */
#define SENSOR_INT2_PORT                 (GPIO_PRT1)
#define SENSOR_INT2_NUM                  (5u)
/* 复位初值必须是**高**: 刚复位时 RX 环是空的(空余最大), 给低会让主机一直不敢发、链路起不来。 */
#define SENSOR_INT2_INIT_STATE           (1u)

/* tag=0 主动状态帧: 队里没有待发响应时写它。载荷 = 36 位触控掩码(5B) + gen u16 + 补 0。 */
extern uint8_t lnk_status_frame[LNK_FRAME_SIZE];
/* 每份响应都带的状态字节(LNK_ST_*)。主机因此无需为"设备忙不忙"单独发命令 —— v1 里 GET_STATS
 * 被高频轮询、反过来抢占主循环空窗正是那样来的。各位由其真相源在变更点维护。 */
extern volatile uint8_t lnk_st;

/* st 的读改写在 ISR 与主循环都会发生 ⇒ 统一走临界区。CM0+ 上开销约十几个周期, 可忽略。 */
static inline void lnk_st_upd(uint8_t set_bits, uint8_t clr_bits)
{
    uint32_t s = Cy_SysLib_EnterCriticalSection();
    lnk_st = (uint8_t)((uint8_t)(lnk_st & (uint8_t)~clr_bits) | set_bits);
    Cy_SysLib_ExitCriticalSection(s);
}

/* 线级对外接口。lnk_rx_drain() 是 v3 唯一的命令入口(主循环泵)。 */
void spi_slave_init(void);
bool lnk_rx_drain(void);
void lnk_resp_push(uint8_t tag, const uint8_t payload[LNK_PAYLOAD_BYTES]);
/* 启动时的链路态复位(待发队列/环/游标/lnk_st)。 */
void lnk_wire_reset(void);

/* ---- 上层实现的回调(本模块只声明、不包含其头文件) ---- */
void lnk_deliver(const uint8_t * rx);
void update_touch_frame(void);

#endif /* LNK_WIRE_H */
