/******************************************************************************
 * File Name:   main.c
 *
 * Description: PSoC 4 SPI Slave with CAPSENSE and Status LED
 * - SPI Slave: SCB0 (SLAVE, MODE0, 8-bit, MSB-first, CS ActiveLow)
 * - CAPSENSE: 36 buttons, immutable double-buffered snapshots
 * - Status LED: P1.6 (CYBSP_LED_SLD3)
 * - FW_VERSION: 0x00000401 (0.4.1)
 ******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include <string.h>

#define FW_VERSION_MAJOR                 (0u)
#define FW_VERSION_MINOR                 (4u)
#define FW_VERSION_PATCH                 (2u)
#define FW_VERSION                       (((uint32_t)FW_VERSION_MAJOR << 16u) | \
                                          ((uint32_t)FW_VERSION_MINOR << 8u) | \
                                          FW_VERSION_PATCH)

#define SENSOR_FRAME_MAGIC               (0xA5u)
#define SENSOR_CMD_PING                  (0x01u)
#define SENSOR_CMD_PONG                  (0x02u)
#define SENSOR_CMD_TOUCH                 (0x03u)  /* 实时触控态：7字节帧 [magic,TOUCH,mask0..mask4] */
#define SENSOR_CMD_SNAPSHOT_BEGIN        (0x10u)
#define SENSOR_CMD_SNAPSHOT_INFO         (0x11u)
#define SENSOR_CMD_SNAPSHOT_PAGE         (0x12u)
#define SENSOR_CMD_SNAPSHOT_DATA         (0x13u)
#define SENSOR_CMD_INDICATOR_ON          (0x20u)
// ---- Phase A：CSD 运行时指令通道（7字节帧 [magic,cmd,ch,param_id,val_lo,val_mid,val_hi]，value 24位）----
#define SENSOR_CMD_SET_PARAM             (0x30u)  // 写 widgetContext[ch].<param_id> = value
#define SENSOR_CMD_GET_PARAM             (0x31u)  // 读回；响应 [magic,GET_PARAM,ch,param_id,val24]
#define SENSOR_CMD_SET_MODE              (0x32u)  // ch 位置=mode(0=auto/SmartSense, 1=semi/manual)
#define SENSOR_CMD_APPLY                 (0x33u)  // 应用参数：重扫/重校准使硬件参数生效
#define SENSOR_CMD_GET_RAW               (0x34u)  // 读指定通道实时 raw 计数；响应 [magic,GET_RAW,ch,raw_lo,raw_hi]
#define SENSOR_CMD_GET_STATS              (0x35u)  // 读全局扫描计数(每秒采样率由 RP2040 用其时钟算)
#define SENSOR_CMD_MEASURE_CP            (0x36u)  // 触发逐电极寄生电容(Cp)测量(耗时,主循环执行);ack 后用 GET_CP 读回
#define SENSOR_CMD_GET_CP                (0x37u)  // 读指定通道最近一次 Cp 测量值(fF);响应 [magic,GET_CP,ch,val24]
// CapSense 参数 id（与上位机 proto PARAM_* 对齐）
#define PARAM_FINGER_TH                  (0x01u)
#define PARAM_NOISE_TH                   (0x02u)
#define PARAM_NEG_NOISE_TH               (0x03u)
#define PARAM_HYSTERESIS                 (0x04u)
#define PARAM_ON_DEBOUNCE                (0x05u)
#define PARAM_LOW_BSLN_RST               (0x06u)
#define PARAM_RESOLUTION                 (0x07u)
#define PARAM_SNS_CLK_DIV                (0x08u)
#define PARAM_IDAC_MOD                   (0x09u)
#define PARAM_SNS_CLK_SOURCE             (0x0Au)  // 时钟源(bit7=auto, 低位=PRS/direct 选择)
#define PARAM_IDAC_GAIN                  (0x0Bu)  // IDAC 增益档索引(增幅), 半自动手动调优用
// CSD 处理模式
#define SCAN_MODE_AUTO                   (0u)  // 全自动 SmartSense：阈值/硬件参数每周期自整定
#define SCAN_MODE_SEMI                   (1u)  // 半自动/手动：跳过噪声包络+阈值自整定，SET_PARAM 阈值持久
#define SENSOR_FRAME_SIZE                (7u)
#define SENSOR_FRAME_PAYLOAD_SIZE        (4u)
#define SENSOR_CHANNEL_COUNT             (36u)
#define SENSOR_BYTES_PER_CHANNEL         (7u)
#define SENSOR_SNAPSHOT_SIZE             (SENSOR_CHANNEL_COUNT * SENSOR_BYTES_PER_CHANNEL)
#define SENSOR_SNAPSHOT_PAGE_SIZE        (SENSOR_FRAME_PAYLOAD_SIZE)
#define SENSOR_SNAPSHOT_PAGE_COUNT       (SENSOR_SNAPSHOT_SIZE / SENSOR_SNAPSHOT_PAGE_SIZE)

#define CAPSENSE_INTR_PRIORITY           (3u)
#define CY_ASSERT_FAILED                 (0u)
#define STATUS_LED_PORT                  (CYBSP_LED_SLD3_PORT)
#define STATUS_LED_NUM                   (CYBSP_LED_SLD3_NUM)
#define STATUS_LED_ON_STATE              (1u)
#define STATUS_LED_OFF_STATE             (0u)

#if ((SENSOR_SNAPSHOT_SIZE % SENSOR_SNAPSHOT_PAGE_SIZE) != 0u)
#error "Snapshot size must be an exact number of SPI pages"
#endif

static cy_stc_scb_spi_context_t spi_context;
static uint8_t spi_tx_frame[SENSOR_FRAME_SIZE];
/* 实时触控帧：主循环每个 capsense 周期更新，SPI ISR 作为默认响应装入 TX FIFO（流水线）。 */
static volatile uint8_t touch_frame[SENSOR_FRAME_SIZE];
/* APPLY 指令置位，主循环执行重校准（不能在 ISR 里做耗时重扫描）。 */
static volatile bool apply_pending = false;
/* MEASURE_CP 指令置位，主循环执行逐电极 BIST 电容测量(不能在 ISR 里做耗时测量)。 */
static volatile bool measure_cp_pending = false;
/* 每通道最近一次寄生电容测量值(fF)，0xFFFFFF=测量失败。 */
static volatile uint32_t cp_value[SENSOR_CHANNEL_COUNT];
/* CSD 处理模式：SCAN_MODE_AUTO(全自动 SmartSense) / SCAN_MODE_SEMI(半自动手动阈值)。 */
static volatile uint8_t scan_mode = SCAN_MODE_AUTO;
static volatile uint32_t scan_count = 0u;  // 每完成一次全通道扫描 +1，自由递增(无触摸也增长)
static uint8_t snapshot_buffers[2u][SENSOR_SNAPSHOT_SIZE];
static uint16_t snapshot_generations[2u];
static volatile uint8_t published_snapshot_index;
static volatile bool published_snapshot_valid;
static uint8_t transfer_snapshot[SENSOR_SNAPSHOT_SIZE];
static uint16_t transfer_generation;
static bool transfer_valid;

static void spi_slave_task(void);
static void spi_isr(void);

static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

static void initialize_capsense(void)
{
    cy_stc_sysint_t capsense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    if (CY_CAPSENSE_STATUS_SUCCESS == Cy_CapSense_Init(&cy_capsense_context))
    {
        Cy_SysInt_Init(&capsense_interrupt_config, capsense_isr);
        NVIC_ClearPendingIRQ(capsense_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_interrupt_config.intrSrc);
        Cy_CapSense_Enable(&cy_capsense_context);
    }
}

static void snapshot_write_u16(uint8_t * destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFu);
    destination[1] = (uint8_t)((value >> 8u) & 0xFFu);
}

static void publish_capsense_snapshot(void)
{
    uint8_t current_index = published_snapshot_index;
    uint8_t next_index = (uint8_t)(current_index ^ 1u);
    uint8_t * destination = snapshot_buffers[next_index];
    uint32_t channel;

    for (channel = 0u; channel < SENSOR_CHANNEL_COUNT; channel++)
    {
        const cy_stc_capsense_sensor_context_t * sensor =
            &cy_capsense_context.ptrWdConfig[channel].ptrSnsContext[0u];
        uint32_t offset = channel * SENSOR_BYTES_PER_CHANNEL;

        snapshot_write_u16(&destination[offset], sensor->raw);
        snapshot_write_u16(&destination[offset + 2u], sensor->bsln);
        snapshot_write_u16(&destination[offset + 4u], sensor->diff);
        destination[offset + 6u] = sensor->status;
    }

    snapshot_generations[next_index] = (uint16_t)(snapshot_generations[current_index] + 1u);
    if (snapshot_generations[next_index] == 0u)
    {
        snapshot_generations[next_index] = 1u;
    }

    {
        uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
        published_snapshot_index = next_index;
        published_snapshot_valid = true;
        Cy_SysLib_ExitCriticalSection(interrupt_state);
    }
}

/* 从 capsense 上下文计算 36 区 on/off 位图，更新触控帧（临界区保护，防 SPI ISR 读到半更新）。 */
static void update_touch_frame(void)
{
    uint8_t mask[5] = {0u, 0u, 0u, 0u, 0u};
    uint32_t ch;
    uint32_t st;

    for (ch = 0u; ch < SENSOR_CHANNEL_COUNT; ch++)
    {
        if (0u != Cy_CapSense_IsWidgetActive((uint32_t)ch, &cy_capsense_context))
        {
            mask[ch >> 3u] |= (uint8_t)(1u << (ch & 7u));
        }
    }

    st = Cy_SysLib_EnterCriticalSection();
    touch_frame[0] = SENSOR_FRAME_MAGIC;
    touch_frame[1] = SENSOR_CMD_TOUCH;
    touch_frame[2] = mask[0];
    touch_frame[3] = mask[1];
    touch_frame[4] = mask[2];
    touch_frame[5] = mask[3];
    touch_frame[6] = mask[4];
    Cy_SysLib_ExitCriticalSection(st);
}

/* 把当前触控帧装入 TX FIFO（作为默认/流水线响应）。 */
static void spi_load_touch(void)
{
    uint8_t frame[SENSOR_FRAME_SIZE];
    uint32_t st = Cy_SysLib_EnterCriticalSection();
    frame[0] = touch_frame[0]; frame[1] = touch_frame[1]; frame[2] = touch_frame[2];
    frame[3] = touch_frame[3]; frame[4] = touch_frame[4]; frame[5] = touch_frame[5];
    frame[6] = touch_frame[6];
    Cy_SysLib_ExitCriticalSection(st);
    Cy_SCB_SPI_WriteArray(scb_0_HW, frame, SENSOR_FRAME_SIZE);
}

// ---- Phase A：运行时 CSD 参数读写（直写 cy_capsense_tuner.widgetContext RAM）----
// 阈值类(fingerTh/noiseTh/hysteresis/onDebounce/lowBslnRst)在下次 ProcessAllWidgets 自动生效；
// 硬件类(resolution/snsClk/idacMod)需 APPLY(重扫/重校准)。单字段 16/8 位写在 CM0+ 上原子。
static void cmd_set_param(uint8_t ch, uint8_t param_id, uint32_t value)
{
    if (ch >= SENSOR_CHANNEL_COUNT) return;
    cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[ch];
    switch (param_id)
    {
        case PARAM_FINGER_TH:    wc->fingerTh   = (uint16_t)value; break;
        case PARAM_NOISE_TH:     wc->noiseTh    = (uint16_t)value; break;
        case PARAM_NEG_NOISE_TH: wc->nNoiseTh   = (uint16_t)value; break;
        case PARAM_HYSTERESIS:   wc->hysteresis = (uint16_t)value; break;
        case PARAM_ON_DEBOUNCE:  wc->onDebounce = (uint8_t)value;  break;
        case PARAM_LOW_BSLN_RST: wc->lowBslnRst = (uint16_t)value; break;
        case PARAM_RESOLUTION:    wc->resolution   = (uint16_t)value; break;
        case PARAM_SNS_CLK_DIV:   wc->snsClk       = (uint16_t)value; break;
        case PARAM_IDAC_MOD:      wc->idacMod[0]   = (uint8_t)value;  break;
        case PARAM_SNS_CLK_SOURCE:wc->snsClkSource = (uint8_t)value;  break;
        case PARAM_IDAC_GAIN:     wc->idacGainIndex= (uint8_t)value;  break;
        default: break;
    }
}

static uint32_t cmd_get_param(uint8_t ch, uint8_t param_id)
{
    if (ch >= SENSOR_CHANNEL_COUNT) return 0u;
    const cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[ch];
    switch (param_id)
    {
        case PARAM_FINGER_TH:    return wc->fingerTh;
        case PARAM_NOISE_TH:     return wc->noiseTh;
        case PARAM_NEG_NOISE_TH: return wc->nNoiseTh;
        case PARAM_HYSTERESIS:   return wc->hysteresis;
        case PARAM_ON_DEBOUNCE:  return wc->onDebounce;
        case PARAM_LOW_BSLN_RST: return wc->lowBslnRst;
        case PARAM_RESOLUTION:    return wc->resolution;
        case PARAM_SNS_CLK_DIV:   return wc->snsClk;
        case PARAM_IDAC_MOD:      return wc->idacMod[0];
        case PARAM_SNS_CLK_SOURCE:return wc->snsClkSource;
        case PARAM_IDAC_GAIN:     return wc->idacGainIndex;
        default: return 0u;
    }
}

// 装载指令响应帧（直接填 TX FIFO；ISR 上下文，与 spi_load_touch 同）。
static void spi_load_cmd_response(uint8_t command, uint8_t b2, uint8_t b3, uint32_t val24)
{
    spi_tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_tx_frame[1] = command;
    spi_tx_frame[2] = b2;
    spi_tx_frame[3] = b3;
    spi_tx_frame[4] = (uint8_t)(val24 & 0xFFu);
    spi_tx_frame[5] = (uint8_t)((val24 >> 8u) & 0xFFu);
    spi_tx_frame[6] = (uint8_t)((val24 >> 16u) & 0xFFu);
    Cy_SCB_SPI_WriteArray(scb_0_HW, spi_tx_frame, SENSOR_FRAME_SIZE);
}

static uint16_t cmd_get_raw(uint8_t ch)
{
    if (ch >= SENSOR_CHANNEL_COUNT) return 0u;
    return cy_capsense_context.ptrWdConfig[ch].ptrSnsContext[0].raw;
}

// GET_STATS 响应: [magic, GET_STATS, 0, scan_count u32 LE(字节3..6)]
static void spi_load_stats(void)
{
    uint32_t sc = scan_count;   // CM0+ 上 32 位对齐读原子
    spi_tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_tx_frame[1] = SENSOR_CMD_GET_STATS;
    spi_tx_frame[2] = 0u;
    spi_tx_frame[3] = (uint8_t)(sc & 0xFFu);
    spi_tx_frame[4] = (uint8_t)((sc >> 8u) & 0xFFu);
    spi_tx_frame[5] = (uint8_t)((sc >> 16u) & 0xFFu);
    spi_tx_frame[6] = (uint8_t)((sc >> 24u) & 0xFFu);
    Cy_SCB_SPI_WriteArray(scb_0_HW, spi_tx_frame, SENSOR_FRAME_SIZE);
}

static void spi_load_frame(uint8_t command, uint8_t sequence, const uint8_t payload[SENSOR_FRAME_PAYLOAD_SIZE])
{
    uint32_t index;

    spi_tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_tx_frame[1] = command;
    spi_tx_frame[2] = sequence;
    for (index = 0u; index < SENSOR_FRAME_PAYLOAD_SIZE; index++)
    {
        spi_tx_frame[3u + index] = (payload != NULL) ? payload[index] : 0u;
    }
    Cy_SCB_SPI_WriteArray(scb_0_HW, spi_tx_frame, SENSOR_FRAME_SIZE);
}

static void spi_load_pong(uint8_t sequence)
{
    uint8_t payload[SENSOR_FRAME_PAYLOAD_SIZE];
    payload[0] = (uint8_t)(FW_VERSION & 0xFFu);
    payload[1] = (uint8_t)((FW_VERSION >> 8u) & 0xFFu);
    payload[2] = (uint8_t)((FW_VERSION >> 16u) & 0xFFu);
    payload[3] = (uint8_t)((FW_VERSION >> 24u) & 0xFFu);
    spi_load_frame(SENSOR_CMD_PONG, sequence, payload);
}

static void spi_latch_snapshot(uint8_t sequence)
{
    uint8_t payload[SENSOR_FRAME_PAYLOAD_SIZE];
    uint8_t source_index = published_snapshot_index;

    transfer_valid = published_snapshot_valid;
    transfer_generation = snapshot_generations[source_index];
    if (transfer_valid)
    {
        memcpy(transfer_snapshot, snapshot_buffers[source_index], SENSOR_SNAPSHOT_SIZE);
    }
    else
    {
        memset(transfer_snapshot, 0, SENSOR_SNAPSHOT_SIZE);
    }

    payload[0] = (uint8_t)(transfer_generation & 0xFFu);
    payload[1] = (uint8_t)((transfer_generation >> 8u) & 0xFFu);
    payload[2] = transfer_valid ? 1u : 0u;
    payload[3] = SENSOR_CHANNEL_COUNT;
    spi_load_frame(SENSOR_CMD_SNAPSHOT_INFO, sequence, payload);
}

static void spi_load_snapshot_page(uint8_t sequence, uint8_t page)
{
    uint8_t payload[SENSOR_FRAME_PAYLOAD_SIZE] = {0u};

    if (page < SENSOR_SNAPSHOT_PAGE_COUNT)
    {
        uint32_t offset = (uint32_t)page * SENSOR_SNAPSHOT_PAGE_SIZE;
        memcpy(payload, &transfer_snapshot[offset], SENSOR_SNAPSHOT_PAGE_SIZE);
        spi_load_frame(SENSOR_CMD_SNAPSHOT_DATA, sequence, payload);
    }
    else
    {
        spi_load_pong(sequence);
    }
}

static void spi_slave_init(void)
{
    static const cy_stc_sysint_t spi_int_cfg =
    {
        .intrSrc = scb_0_IRQ,
        .intrPriority = 2u,   /* 高于 capsense(3)，保证 SPI 及时响应，不被扫描阻塞 */
    };

    Cy_SCB_SPI_Init(scb_0_HW, &scb_0_config, &spi_context);
    Cy_SCB_SPI_Enable(scb_0_HW);
    Cy_SCB_SPI_ClearRxFifo(scb_0_HW);
    Cy_SCB_SPI_ClearTxFifo(scb_0_HW);
    update_touch_frame();
    spi_load_touch();   /* 默认响应=实时触控帧（流水线快路） */

    /* 中断驱动：RX FIFO 达到整帧(7字节)即中断，ISR 立即装载响应。
       RX LEVEL 中断在 FIFO 计数 > level 时触发，故 level=帧长-1=6 → 满 7 字节触发。 */
    Cy_SysInt_Init(&spi_int_cfg, spi_isr);
    NVIC_ClearPendingIRQ(scb_0_IRQ);
    NVIC_EnableIRQ(scb_0_IRQ);
    Cy_SCB_SetRxFifoLevel(scb_0_HW, SENSOR_FRAME_SIZE - 1u);
    Cy_SCB_SetRxInterruptMask(scb_0_HW, CY_SCB_RX_INTR_LEVEL);
}

static void spi_isr(void)
{
    if (Cy_SCB_SPI_GetNumInRxFifo(scb_0_HW) >= SENSOR_FRAME_SIZE)
    {
        spi_slave_task();
    }
    Cy_SCB_ClearRxInterrupt(scb_0_HW, CY_SCB_RX_INTR_LEVEL);
}

static void spi_slave_task(void)
{
    uint8_t rx[SENSOR_FRAME_SIZE];

    if (Cy_SCB_SPI_GetNumInRxFifo(scb_0_HW) < SENSOR_FRAME_SIZE)
    {
        return;
    }

    Cy_SCB_SPI_ReadArray(scb_0_HW, rx, SENSOR_FRAME_SIZE);
    Cy_SCB_SPI_ClearTxFifo(scb_0_HW);

    if (rx[0] != SENSOR_FRAME_MAGIC)
    {
        spi_load_touch();   /* 默认回落触控帧 */
        return;
    }

    // 白灯仅启动时点亮、之后熄灭(见 main())；此处不再随帧翻转链路态。
    switch (rx[1])
    {
        case SENSOR_CMD_PING:
            spi_load_pong(rx[2]);
            break;

        case SENSOR_CMD_SNAPSHOT_BEGIN:
            spi_latch_snapshot(rx[2]);
            break;

        case SENSOR_CMD_SNAPSHOT_PAGE:
            spi_load_snapshot_page(rx[2], rx[3]);
            break;

        case SENSOR_CMD_INDICATOR_ON:
            Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM, STATUS_LED_ON_STATE);
            spi_load_pong(rx[2]);
            break;

        case SENSOR_CMD_SET_PARAM: {
            uint32_t val = (uint32_t)rx[4] | ((uint32_t)rx[5] << 8u) | ((uint32_t)rx[6] << 16u);
            cmd_set_param(rx[2], rx[3], val);
            // 回显供 RP2040 校验；阈值类已即时生效，硬件类待 APPLY。
            spi_load_cmd_response(SENSOR_CMD_SET_PARAM, rx[2], rx[3], val);
            break;
        }

        case SENSOR_CMD_GET_PARAM:
            spi_load_cmd_response(SENSOR_CMD_GET_PARAM, rx[2], rx[3], cmd_get_param(rx[2], rx[3]));
            break;

        case SENSOR_CMD_GET_RAW:
            spi_load_cmd_response(SENSOR_CMD_GET_RAW, rx[2], 0u, cmd_get_raw(rx[2]));
            break;

        case SENSOR_CMD_GET_STATS:
            spi_load_stats();
            break;

        case SENSOR_CMD_MEASURE_CP:
            // 耗时的逐电极 BIST 测量不能在 ISR 做，仅置标志，主循环执行；立即 ack。
            measure_cp_pending = true;
            spi_load_cmd_response(SENSOR_CMD_MEASURE_CP, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_GET_CP:
            spi_load_cmd_response(SENSOR_CMD_GET_CP, rx[2], 0u,
                                  (rx[2] < SENSOR_CHANNEL_COUNT) ? cp_value[rx[2]] : 0u);
            break;

        case SENSOR_CMD_APPLY:
            // ★不能在 ISR 里做重校准(耗时数ms/需扫描完成)★：仅置标志，主循环执行。
            apply_pending = true;
            spi_load_cmd_response(SENSOR_CMD_APPLY, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_SET_MODE:
            // rx[2]: 0=全自动 SmartSense, 非0=半自动/手动。主循环据此选处理链。
            scan_mode = (rx[2] != 0u) ? SCAN_MODE_SEMI : SCAN_MODE_AUTO;
            spi_load_cmd_response(SENSOR_CMD_SET_MODE, scan_mode, 0u, 0u);
            break;

        case SENSOR_CMD_TOUCH:
        default:
            /* 触控快路（流水线）：装入最新触控帧，供下一次读取立即返回。 */
            spi_load_touch();
            break;
    }
}

int main(void)
{
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    memset(snapshot_buffers, 0, sizeof(snapshot_buffers));
    memset(snapshot_generations, 0, sizeof(snapshot_generations));
    memset(transfer_snapshot, 0, sizeof(transfer_snapshot));
    published_snapshot_index = 0u;
    published_snapshot_valid = false;
    transfer_generation = 0u;
    transfer_valid = false;

    __enable_irq();
    initialize_capsense();
    spi_slave_init();
    // hardware.txt: P1.6 -> LED -> R -> GND，因此高电平明确为点亮。
    // 白灯仅启动时点亮 ~300ms 作为启动指示，随后熄灭并保持(不再作链路态指示)。
    Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM, STATUS_LED_ON_STATE);
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
    Cy_SysLib_Delay(300u);
    Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM, STATUS_LED_OFF_STATE);

    for (;;)
    {
        /* SPI 从机改为中断驱动（spi_isr），主循环只做 capsense 扫描/发布快照。
           SPI 响应不再被 ProcessAllWidgets 的耗时阻塞。 */
        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            if (scan_mode == SCAN_MODE_AUTO)
            {
                /* 全自动：中间件跑完整处理链(含噪声包络+阈值自整定)。 */
                (void)Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
            }
            else
            {
                /* 半自动/手动：跳过 CALC_NOISE+THRESHOLDS，手动 SET_PARAM 阈值不被覆盖，
                 * 仍跑滤波/基线/差值/状态，触控检测正常（用手动 fingerTh 判定）。 */
                const uint32_t manual_mask = CY_CAPSENSE_PROCESS_FILTER |
                                             CY_CAPSENSE_PROCESS_BASELINE |
                                             CY_CAPSENSE_PROCESS_DIFFCOUNTS |
                                             CY_CAPSENSE_PROCESS_STATUS;
                for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                {
                    (void)Cy_CapSense_ProcessWidgetExt(w, manual_mask, &cy_capsense_context);
                }
            }
            update_touch_frame();       /* 刷新实时触控帧（快路数据源） */
            publish_capsense_snapshot();/* 刷新完整 raw/baseline/diff（调参慢路数据源） */

            /* MEASURE_CP：逐电极 BIST 寄生电容测量(fF)。测量会重配 CSD HW，完成后恢复扫描配置。 */
            if (measure_cp_pending)
            {
                measure_cp_pending = false;
#if (defined(CY_CAPSENSE_TST_SNS_CAP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN))
                /* BIST 逐电极测量(仅当 self-test 启用时编译;与 SmartSense 互斥,见半自动构建)。 */
                for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                {
                    uint32_t v = 0u;
                    if (CY_CAPSENSE_BIST_SUCCESS_E ==
                        Cy_CapSense_MeasureCapacitanceSensor(w, 0u, &v, &cy_capsense_context))
                    {
                        cp_value[w] = v & 0xFFFFFFu;
                    }
                    else
                    {
                        cp_value[w] = 0xFFFFFFu;   // 失败标记
                    }
                }
                /* 恢复正常扫描(BIST 改了 CSD HW 配置)：全自动重整定 / 半自动仅重配硬件+基线。 */
                if (scan_mode == SCAN_MODE_AUTO)
                {
                    (void)Cy_CapSense_Enable(&cy_capsense_context);
                }
                else
                {
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
                }
#endif
            }

            /* APPLY 指令：在主循环(非 ISR)重新初始化扫描硬件使硬件参数(分辨率/时钟/IDAC)生效。 */
            if (apply_pending)
            {
                apply_pending = false;
                if (scan_mode == SCAN_MODE_AUTO)
                {
                    /* 全自动：完整重初始化 + SmartSense 重整定(重校准硬件参数与阈值)。 */
                    (void)Cy_CapSense_Enable(&cy_capsense_context);
                }
                else
                {
                    /* 半自动：仅从手动 widgetContext 重配硬件(snsClk/resolution/idac)并重置基线，
                     * 不跑 SmartSense，保留手动阈值与手动硬件参数。 */
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
                }
            }

            scan_count++;
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        }
    }
}

/* [] END OF FILE */
