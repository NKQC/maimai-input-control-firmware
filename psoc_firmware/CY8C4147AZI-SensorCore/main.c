/******************************************************************************
 * File Name:   main.c
 *
 * Description: PSoC 4 SPI Slave with CAPSENSE and Status LED
 * - SPI Slave: SCB0 (SLAVE, MODE0, 8-bit, MSB-first, CS ActiveLow)
 * - CAPSENSE: 36 buttons, immutable double-buffered snapshots
 * - Status LED: P1.6 (CYBSP_LED_SLD3)
 * - FW_VERSION: 编译时间戳 YYMMDDHHMM(十进制, 本地时间), 由 Makefile PREBUILD 生成
 *   fw_build_stamp.h 提供; 上位机补 "20" 前缀还原 YYYYMMDDHHMM
 * - JIT algo engine: 1KB executable RAM slot (ABI v1, psoc_algo_abi.h),
 *   uploaded via ALGO_BEGIN/PAGE/END/INFO SPI commands, CRC16-CCITT-FALSE
 *   verified commit in main loop, falls back to Cy_CapSense_IsWidgetActive
 *   when no valid algo is loaded.
 ******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "psoc_algo_abi.h"
#include "fw_build_stamp.h"
#include <stdint.h>
#include <string.h>

/* 版本号 = 编译时间戳(十进制 YYMMDDHHMM, 本地时间), 每次 make build 由 PREBUILD 重新生成。
 * 线上格式不变: 仍是 u32, PING 响应照旧拆 4 字节上报。
 * 已知上限: YY <= 42 才放得进 u32, 2043 年起下面的断言会让构建直接失败。 */
#define FW_VERSION                       (FW_BUILD_STAMP)
_Static_assert(FW_VERSION <= 0xFFFFFFFFu, "FW_BUILD_STAMP overflows uint32 (YY > 42?)");

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
#define SENSOR_CMD_SET_MODE              (0x32u)  // ch 位置=mode(0=自动校准/标准完整处理, 1=半自动手动)
#define SENSOR_CMD_APPLY                 (0x33u)  // 应用参数：重扫/重校准使硬件参数生效
#define SENSOR_CMD_GET_RAW               (0x34u)  // 读指定通道实时 raw 计数；响应 [magic,GET_RAW,ch,raw_lo,raw_hi]
#define SENSOR_CMD_GET_STATS              (0x35u)  // 读全局扫描计数(每秒采样率由 RP2040 用其时钟算)
#define SENSOR_CMD_MEASURE_CP            (0x36u)  // 触发逐电极 Cp 测量；SPI ACK 后由主循环异步执行
#define SENSOR_CMD_GET_CP                (0x37u)  // pending/active=0，成功=fF，失败/未测量=0xFFFFFF
// ---- 全局 CSD 配置(RAM 影子 common config，改后需 APPLY 重初始化生效)----
// 帧 [magic,cmd,gparam_id,0,val_lo,val_mid,val_hi]，值 24 位。
#define SENSOR_CMD_SET_GLOBAL            (0x38u)  // 写 g_common_cfg_ram.<gparam>(仅改影子,不重初始化)
#define SENSOR_CMD_GET_GLOBAL            (0x39u)  // 读回；响应 [magic,GET_GLOBAL,gparam_id,0,val24]
#define SENSOR_CMD_GLOBAL_COMMIT         (0x3Au)  // 全部全局项设完后触发【一次】完整重初始化(合并,防反复重校准漂移)
#define SENSOR_CMD_CALIBRATE             (0x3Bu)  // 真正的 IDAC 重校准(CalibrateAllWidgets)+基线复位, 主循环执行
#define SENSOR_CMD_BASELINE_RESET        (0x3Cu)  // 仅重置全部通道基线(InitializeAllBaselines), 主循环执行
// 频率自适应(三步): ①粗表升分频定位首个可校准档 ②从该档 -1 起逐 1 升频重校准找"临界分频"
// (最后一个仍能校准成功的最小分频=最高频率=最不灵敏) ③按偏好档位往低频让 2*(pref-1) 个分频落档
// (分频越大=频率越低=充电越充分→过充产生近场探测效应→越灵敏), 失败则朝临界方向逐 1 回退。
// 帧字节2=通道号: 0..35=仅该通道自适应(只改该 widget 的 snsClk), 0xFF=全 36 通道【逐通道各自校准】
// (外层遍历 36 个通道, 每通道独立跑完整三步算法得到各自的分频; 面板 Cp 22~138pF 无法共用统一分频)。
// 帧字节3=偏好档位 pref(1..7, 非法值退化为 4): 1=临界最高频(最不灵敏), 7=最低频(最灵敏)。
#define SENSOR_CMD_AUTO_TUNE             (0x3Du)
// 读自适应结果/进度; 响应 [magic,GET_AUTO_TUNE,result(0进行中/1成功/2失败),ch,div_lo,div_hi,progress]。
// ★resp[6]=progress 字节(原 val24 最高字节恒 0, 现复用为阶段进度, 旧 RP2040 只读 resp[4..5] 故向后兼容)★:
//   bit0-2 = phase: 0=空闲/已受理 1=粗定位 2=细搜临界 3=落档/回退 4=完成
//   bit3-7 = step : 当前阶段内步序(从 1 起递增, 上限饱和 31)
// ★resp[4..5]=div 的语义随 result 变化★:
//   result==0(进行中) → 当前正在试探的 snsClk 分频(供上位机显示"正在试 ÷N");
//   result!=0(已完成) → 最终写入 widgetContext 的分频(成功)或 0(失败)。
#define SENSOR_CMD_GET_AUTO_TUNE         (0x3Eu)
/* ★Sweep 专用轻量应用(帧字节2=目标通道 0..35, 0xFF=全通道)★
 * 只做 Cy_CapSense_Initialize + CSD 模式准备, 使 widgetContext 的 gain(idacGainIndex)/div(snsClk)
 * 真正落到硬件; 【绝不】重校准 IDAC、【绝不】初始化基线。
 * ★为什么必须与 APPLY 分开★: APPLY 在 g_auto_calibrate(默认开)下必然重校准(AUTO 模式全启用通道、
 * 半自动模式脏通道), 实测每格约 800ms 已超出 Sweep 逐格预算; 而 448 格扫描只需要"参数生效后读 raw",
 * 校准与基线由会话结束时的一次显式 CALIBRATE + BASELINE_RESET 统一收口。
 * 本命令只服务 Sweep, 不改变 APPLY 的任何语义。 */
#define SENSOR_CMD_QUICK_APPLY           (0x3Fu)
/* Focus 扫描控制: 帧字节2=0..35 时只扫描该已启用 widget 并续租；0xFF 时立即恢复全通道。
 * 响应 b2=实际模式目标(0xFF=全通道), b3=1 接受/0 拒绝。 */
#define SENSOR_CMD_FOCUS_SCAN            (0x49u)
/* ★通道号哨兵(全通道)★ 单一取值, 供 CALIBRATE / BASELINE_RESET / AUTO_TUNE 共用 ——
 * 不为每条命令另造一个"全通道"常量, 免得三处各写一份必然漂移。 */
#define SENSOR_CH_ALL                    (0xFFu)
#define FOCUS_SCAN_LEASE_MS              (3000u)
#define AUTO_TUNE_CH_ALL                 SENSOR_CH_ALL  // AUTO_TUNE 通道号哨兵: 全通道(同 SENSOR_CH_ALL)
// 全局参数 id
#define GPARAM_INACTIVE_SNS              (0x01u)  // 未激活传感器连接: 1=GND 2=High-Z 4=Shield
#define GPARAM_IDAC_GAIN_INIT            (0x02u)  // csdIdacGainInitIndex(IDAC 增益档索引)
#define GPARAM_IDAC_MIN                  (0x03u)  // csdIdacMin(CSD 校准最小 IDAC)
#define GPARAM_RAW_TARGET                (0x04u)  // csdRawTarget(校准目标 raw 百分比)
#define GPARAM_MFS_DIV_F1                (0x05u)  // csdMfsDividerOffsetF1(多频通道1分频偏移)
#define GPARAM_MFS_DIV_F2                (0x06u)  // csdMfsDividerOffsetF2(多频通道2分频偏移)
#define GPARAM_IDAC_SENSE_CONFIG         (0x07u)  // csdChargeTransfer: 0=IDAC sourcing, 1=IDAC sinking (运行时可设)
#define GPARAM_AUTO_CALIBRATE_EN         (0x08u)  // 运行时是否自动校准: 0=固定IDAC(不自动校准), 1=Init/Apply 自动校准
// ---- SPI DMA 链路只读诊断计数(复用 GET_GLOBAL 通道, 不新增命令码) ----
// 用于在"RP2040 读回全 0"时区分故障域: 帧到底有没有进来、magic 对不对、CS 重同步有没有在发生。
#define GPARAM_DBG_RX_FRAMES             (0x80u)  // RX DMA 完成中断次数(收到的完整 7 字节帧数)
#define GPARAM_DBG_RX_BAD_MAGIC          (0x81u)  // 收到但 magic != 0xA5 的帧数(=帧错位的直接证据)
#define GPARAM_DBG_CS_RESYNC             (0x82u)  // CS 抬起时发现半帧而做重同步的次数
#define GPARAM_DBG_TX_ARM                (0x83u)  // TX 描述符重挂次数
#define GPARAM_DBG_RX_LEFTOVER           (0x84u)  // CS 抬起时 RX FIFO 仍有残字节的次数
// 只读: 启动时固件对生成配置做过哪些强制改写(位掩码)。上位机据此把"设备被固件改过"如实告知用户,
// 杜绝"UI 显示用户设定值、设备实际另一个值"的静默不同步。
//   bit0 = IDAC 增益档被抬到下限(生成配置默认 0 会全片 railed, 必须抬)
//   bit1 = 校准目标% 非法(0 或 >=100)被改成 85
#define GPARAM_BOOT_OVERRIDE             (0x09u)
// 注: IDAC 自动校准/补偿IDAC/自动增益 在 CapSense v5 是编译期宏(非 common_config 运行时字段),
//     无法运行时切换; 若需固定IDAC(不自动校准)换稳定灵敏度, 运行时路径=手动设 IDAC_MOD 且不触发校准。
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
/* ★通道启用开关(0=禁用/电极高阻, 1=启用)★
 * 这是硬件开关而非调参项: 禁用 = 该 widget 永久不参与扫描, 其电极保持模拟高阻(见 g_ch_enabled)。
 * 复用 SET_PARAM/GET_PARAM 通道, 不新增命令码。 */
#define PARAM_ENABLED                    (0x0Cu)
// CSD 处理模式
#define SCAN_MODE_AUTO                   (0u)  // 自动校准：运行中间件标准完整处理链
#define SCAN_MODE_SEMI                   (1u)  // 半自动手动：跳过噪声/阈值处理，保留 SET_PARAM 手动值
#define SENSOR_FRAME_SIZE                (7u)
#define SENSOR_FRAME_PAYLOAD_SIZE        (4u)
#define SENSOR_CHANNEL_COUNT             (36u)
#define SENSOR_BYTES_PER_CHANNEL         (7u)
#define SENSOR_SNAPSHOT_SIZE             (SENSOR_CHANNEL_COUNT * SENSOR_BYTES_PER_CHANNEL)
#define SENSOR_SNAPSHOT_PAGE_SIZE        (SENSOR_FRAME_PAYLOAD_SIZE)
#define SENSOR_SNAPSHOT_PAGE_COUNT       (SENSOR_SNAPSHOT_SIZE / SENSOR_SNAPSHOT_PAGE_SIZE)

/* SPI CS = P1.3 (SCB0 SPI SELECT0, 见 cycfg_routing.h)。HSIOM 交给 SCB 后端口输入仍可产生
 * GPIO 中断, 故可用它的上升沿(取消选择)作帧边界做 RX 重同步。 */
#define SPI_CS_PORT                      (GPIO_PRT1)
#define SPI_CS_NUM                       (3u)
#define SPI_CS_IRQ                       (ioss_interrupts_gpio_1_IRQn)

/* ★新代数就绪通知线★ P1.4 → RP2040 GPIO23 (SENSOR-INT1, 见 doc/hardware.txt)。
 * 每发布一份新快照就把电平**翻转**一次, 一次翻转 = 一份新代数。
 * 为什么是翻转而不是脉冲: RP2040 侧只需比较"电平与上次采到的是否不同"即可判定有新代数 ——
 * 不依赖脉宽(窄脉冲会被采样错过)、不需要任何清中断/握手、也不会因为漏采一次就永久失步。
 * 有了它, RP2040 的 core1 不必再以固定间隔空转轮询 SPI: 没有新代数时它什么都不做,
 * PSoC 的 CapSense 中间件因此拿回被 SPI DMA ISR 抢走的临界区时间。
 * P1.4 由 BSP 生成为强推挽输出(CYBSP_LED_SLD2, 生成代码只初始化、从不引用), 这里仍显式
 * FastInit 一次, 免得 BSP 重新生成时把驱动模式漂掉。P1.5(INT2) 两端都还没有约定事件语义,
 * 故不在此声明 —— 不留没人驱动的常量。 */
#define SENSOR_INT1_PORT                 (GPIO_PRT1)
#define SENSOR_INT1_NUM                  (4u)

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

typedef struct
{
    /* ★PING/PONG 双落帧缓冲(照搬官方 CE 的 SCB+DMAC 范式)★: 两个 RX 描述符【始终有效】、
     * flipping 交替, 故 SCB 的电平请求永远落在一个有效描述符上, 不存在"描述符刚完成/被软件
     * 重置时正好来触发"的窗口。此前单描述符 + ISR 内 SetState(true) 正是踩了这个窗口:
     * SetState 会清 CURR_DATA_NR, 与仍在推进的字节流相撞 → RX 通道搬完一帧后即失效,
     * 之后再不产生完成中断 ⇒ TX 永不重挂 ⇒ RP2040 恒读回全 0(与 TX 填充方式无关)。 */
    volatile uint8_t rx_frame[2u][SENSOR_FRAME_SIZE];
    uint8_t tx_frame[SENSOR_FRAME_SIZE];
    volatile bool frame_received;
    volatile bool snapshot_copy_pending;
    volatile uint8_t snapshot_source_index;
    volatile uint8_t snapshot_sequence;
} spi_dma_state_t;

static spi_dma_state_t spi_dma;

/* SPI 链路诊断计数(只读, 经 GET_GLOBAL 的 GPARAM_DBG_* 上报)。同一功能组归拢成 struct,
 * 避免散装全局量; 需要整体归零时用 _spi_dbg_clear()。 */
/* ★带外 SWD 取数用的定位 magic★: 带内 GET_GLOBAL 在链路故障态读不出来(实测全 None), 故本块必须
 * 能被 RP2040 经 SWD 直接读。为免改 BSP 链接脚本(生成文件, 改了脆), 不固定地址, 而是在块首放一对
 * magic 字, 由 RP2040 扫描 SRAM(0x20000000..0x20004000, 16KB) 一次定位后缓存地址。 */
#define SPI_DBG_MAGIC0                   (0x53504442u)   /* "SPDB" */
#define SPI_DBG_MAGIC1                   (0x4C4E4B31u)   /* "LNK1" */

/* 主循环阶段码: 每进入一个可能长耗时的段就写一次, 挂死时停在肇事段上(带外 SWD 读槽3)。
 * 20+ 是 APPLY 分支内部的细分阶段, 用来把"APPLY 耗时 13s"落到具体哪一步。 */
#define MLOOP_STAGE_TOP                  (1u)
#define MLOOP_STAGE_LATCH                (2u)
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
#define MLOOP_STAGE_APPLY_ENABLE         (20u)  /* reserved */
#define MLOOP_STAGE_APPLY_RECAL          (21u)  /* 逐通道 _recalibrate_dirty_channels */
#define MLOOP_STAGE_APPLY_INIT           (22u)  /* Cy_CapSense_Initialize */
#define MLOOP_STAGE_APPLY_BASELINE       (23u)  /* Cy_CapSense_InitializeAllBaselines */

/* 前 5 个 volatile 字段(block+8 .. block+24)是 RP2040 经 SWD 读走的"导出槽", 顺序即上报顺序;
 * 其后的字段仍在内存里累计, 需要时再扩展读取范围即可。
 * ★槽位内容已换代★: 帧对齐问题已由实测证伪(bad_magic 连续两轮为 0, 帧边界完全正确), 故把导出槽
 * 让给当前真正待查的问题 —— PSoC 主循环是否活着、是否在扫描、是否在发布快照。 */
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
    volatile uint32_t setparam_cmd;   /* 槽7: ISR 收到 SENSOR_CMD_SET_PARAM 的次数 */
    volatile uint32_t snap_pub;
    volatile uint32_t rx_bad_magic;
    volatile uint32_t cs_resync;
    volatile uint32_t rx_leftover;
} spi_dbg_t;
/* 带 magic 常量初值 ⇒ 落在 .data(而非 .bss), 内容确定, SWD 扫描必然能命中。 */
static spi_dbg_t spi_dbg = { SPI_DBG_MAGIC0, SPI_DBG_MAGIC1,
                             0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };

static inline void _spi_dbg_clear(void)
{
    spi_dbg.magic0 = SPI_DBG_MAGIC0;
    spi_dbg.magic1 = SPI_DBG_MAGIC1;
    spi_dbg.clk_boot = 0u;
    spi_dbg.scan_count_m = 0u;
    spi_dbg.ms_tick_m = 0u;
    spi_dbg.stage = 0u;
    spi_dbg.clk_set_cnt = 0u;
    spi_dbg.clk_set_last = 0u;
    spi_dbg.clk_now = 0u;
    spi_dbg.setparam_cmd = 0u;
    spi_dbg.snap_pub = 0u;
    spi_dbg.rx_bad_magic = 0u;
    spi_dbg.cs_resync = 0u;
    spi_dbg.rx_leftover = 0u;
}

/* 统计 64 位掩码里的置位数(APPLY 的脏通道工作量)。 */
static inline uint32_t _popcount64(uint64_t v)
{
    uint32_t n = 0u;
    while (v != 0u) { v &= (v - 1u); n++; }
    return n;
}
/* 实时触控帧：主循环每个 capsense 周期更新，DMA 完成 ISR 作为默认响应装入 TX FIFO（流水线）。 */
static volatile uint8_t touch_frame[SENSOR_FRAME_SIZE];
/* APPLY 指令置位，主循环执行重校准（不能在 ISR 里做耗时重扫描）。 */
static volatile bool apply_pending = false;
/* QUICK_APPLY(Sweep 专用)指令置位: 主循环只让 gain/div 落硬件, 不重校准/不重基线(见命令码注释)。
 * quick_apply_ch: 0..35=本次扫描的目标通道(消掉它的脏位), SENSOR_CH_ALL=全通道(兼容入口)。 */
static volatile bool quick_apply_pending = false;
static volatile uint8_t quick_apply_ch = SENSOR_CH_ALL;
static volatile uint8_t quick_apply_gain = 0u;
static volatile uint8_t quick_apply_div = 1u;
/* ★Focus 单通道扫描态(FOCUS_SCAN 唯一真相源)★
 * focus_scan_ch: 0..35=主循环只扫描/只处理该 widget; SENSOR_CH_ALL=正常全通道。
 * focus_scan_until_ms: 租约截止(g_ms_tick 口径)。RP2040 需周期性续租; 断链/宕机/忘记 STOP 时
 * 租约到期后主循环自行回到全通道 —— 这是"永不遗留单通道模式"的兜底, 不依赖上位机。
 * ISR 只写这两个字节, 真正的扫描目标由主循环的 _focus_scan_target() 复核(启用态+租约)。 */
static volatile uint8_t focus_scan_ch = SENSOR_CH_ALL;
static volatile uint32_t focus_scan_until_ms = 0u;
/* 硬件参数改变后仅记录对应通道，避免 APPLY 为未修改通道重校准造成状态漂移。 */
static volatile uint64_t idac_dirty_mask = 0u;
/* MEASURE_CP 指令置位，主循环执行逐电极 BIST 电容测量(不能在 ISR 里做耗时测量)。 */
static volatile bool measure_cp_pending = false;
/* SET_GLOBAL 置位：全局 CSD 配置(inactive_sns/IDAC/MFS)改动需完整 Init+Initialize 重初始化，
 * 重算内部预计算(csdInactiveSnsDm/HSIOM 见 cy_capsense_sensing.c)。轻量 APPLY 不够，会坏扫描。
 * 主循环执行 Cy_CapSense_Init+Initialize（已验证可用）。 */
static volatile bool global_apply_pending = false;
/* 主循环测量期间保持 true；SPI ISR 据 pending/active 返回 0，避免读到半更新数组。 */
static volatile bool measure_cp_active = false;
/* CALIBRATE 指令置位：主循环执行真正的 IDAC 重校准(不能在 ISR 里做)。 */
static volatile bool calibrate_pending = false;
/* BASELINE_RESET 指令置位：主循环执行基线复位。 */
static volatile bool baseline_reset_pending = false;
/* ★校准/基线的目标通道(单通道语义端到端透传)★ 0..35=只处理该通道, SENSOR_CH_ALL=全 36 通道。
 * 与 auto_tune_ch 完全同构(同一哨兵、同一 rx[2] 位置), 不新增命令码、不造平行协议。
 * ★为什么必须透传★: 上位机点"本通道校准"时原先固件把它扩成 36 通道重校准 —— 既慢(逐通道
 * CalibrateWidget × 36)又会把其它通道刚调好的 IDAC/基线一起冲掉, 与"只调这一个通道"的语义相反。
 * UI 的批量(全通道)现在由 host 侧可取消串行队列逐通道发起, 不再依赖固件内部循环。 */
static volatile uint8_t calibrate_ch = SENSOR_CH_ALL;
static volatile uint8_t baseline_ch = SENSOR_CH_ALL;
/* AUTO_TUNE 指令置位：主循环执行频率自适应下探(逐档升 snsClk 分频重校准)。 */
static volatile bool auto_tune_pending = false;
/* 自适应结果: 0=未执行/进行中, 1=成功(auto_tune_div 为找到的分频), 2=失败(超硬件上限仍压不到目标)。 */
static volatile uint8_t auto_tune_result = 0u;
/* 自适应成功时找到的 snsClk 分频(已写回目标 widgetContext)。
 * ★全通道(0xFF)模式的完成态语义★: 各通道分频互不相同, 单一分频无意义, 故完成时本字段 = 成功通道数
 * (0..36); 真正的逐通道分频由 RP2040 完成后经 GET_PARAM(0x08) 逐通道回读写穿真相源。 */
static volatile uint16_t auto_tune_div = 0u;
/* 本次自适应的目标通道: 0..35=单通道, AUTO_TUNE_CH_ALL=全通道。GET_AUTO_TUNE 回显该字节。 */
static volatile uint8_t auto_tune_ch = AUTO_TUNE_CH_ALL;
/* 本次自适应的灵敏度偏好档位(1..7, 默认 4=居中): 落档时在临界分频上加 2*(pref-1)(往低频=更灵敏)。 */
static volatile uint8_t auto_tune_pref = 4u;
/* ★阶段性进度★: 长自适应(最坏 ~20s)期间由主循环逐步更新, 经 GET_AUTO_TUNE 的 progress 字节上报,
 * 使 RP2040/上位机在过程中就能看到"到哪一步了", 而非只能干等最终结果。
 * phase: 0=空闲/已受理 1=粗定位 2=细搜临界 3=落档/回退 4=完成; step: 该阶段内步序(1 起, 上报饱和 31)。
 * SPI 是独立中断且优先级(2)高于 CapSense(3), 故校准阻塞期间 ISR 仍能应答这两个量。 */
static volatile uint8_t auto_tune_phase = 0u;
static volatile uint8_t auto_tune_step = 0u;
/* ★本轮请求标签(6 bit)★: AUTO_TUNE 帧 rx[4] 带下来的"这条命令属于上位机哪一次请求"的标记,
 * 由 GET_AUTO_TUNE 的 result 字节高 6 位原样回显(result 只用 0/1/2, 高 6 位本来空着 ——
 * 这是扩展既有响应字段, 不新增命令码, 不造平行协议)。
 * ★为什么需要它★: RP2040 侧"发命令→轮询 busy→读结果"的流水线里, 若本条 AUTO_TUNE 其实没被
 * 本 ISR 收到(SPI 丢帧, 而 busy 恰好因上一条重操作为 1), 读回来的就是**上一轮**的 result/div,
 * 却会被当成本轮成功。标签不匹配即可当场认出这种陈旧结果。0 = 未标记(旧 RP 固件)。 */
static volatile uint8_t auto_tune_tag = 0u;
#define AUTO_TUNE_TAG_MASK               (0x3Fu)
#define AUTO_TUNE_RESULT_MASK            (0x03u)
#define AUTO_TUNE_PHASE_IDLE             (0u)
#define AUTO_TUNE_PHASE_COARSE           (1u)
#define AUTO_TUNE_PHASE_FINE             (2u)
#define AUTO_TUNE_PHASE_SETTLE           (3u)
#define AUTO_TUNE_PHASE_DONE             (4u)
#define AUTO_TUNE_STEP_MAX_REPORT        (31u)
/* 细搜(1 步进上探临界)最大步数: 粗表最大相邻间隔为 48→64 的 16, 取 15 步即可覆盖整个区间;
 * 同时限制全通道模式下额外校准次数(单次全通道校准 ~200-270ms), 保证总耗时留在 RP2040 的 10s 窗内。 */
#define AUTO_TUNE_FINE_STEPS_MAX         (15u)
/* ★处理中锁定/真实完成反馈★：ISR 收到 APPLY/CALIBRATE/BASELINE_RESET/GLOBAL_COMMIT 即置 1,
 * 主循环把对应重操作真正做完后清 0。经 GET_STATS 响应 byte[2] 上报, 供 RP2040 轮询至真实完成
 * (替代原固定 sleep 盲等), 上位机据此显示"处理中"并在真正完成后解锁/刷新, 而非命令一到就误判完成。 */
static volatile uint8_t g_op_busy = 0u;
/* 任一通道的 JIT 算法【显式请求点灯】(io->out_led != 0)。白 LED 的唯一运行时来源。
 * 固件不再从触控判定(out_active)推导灯态 —— 没有算法、或算法不写 out_led 时白灯恒灭。 */
static volatile bool g_any_active = false;
/* 全局"是否自动校准"运行时开关(GPARAM_AUTO_CALIBRATE_EN, 默认开)。开=Init/Apply 走 Enable 自动校准 IDAC;
 * 关=只 Initialize+基线复位, 用固定 IDAC(配置/手动值)换取稳定灵敏度范围(修某些通道自动校准发散 railed)。
 * "校准"命令(SENSOR_CMD_CALIBRATE)始终执行一次显式校准, 不受本开关影响(满足"点校准=单次校准"需求)。 */
static volatile bool g_auto_calibrate = true;
/* 每通道最近一次寄生电容测量值(fF)，0xFFFFFF=失败/未测量。 */
static volatile uint32_t cp_value[SENSOR_CHANNEL_COUNT];
/* BIST 的 Cp 上限钳位值(CY_CAPSENSE_BIST_CP_MAX_VALUE, cy_capsense_selftest_v2.c:43)。读到它说明
 * 结果已溢出量程 —— 这才是"短路/异常大电容"的真故障判据, 见 MEASURE_CP 处说明。 */
#define CP_BIST_OVERRANGE_FF             (400000u)

/* CSD 处理模式：SCAN_MODE_AUTO(自动校准/标准完整处理) / SCAN_MODE_SEMI(半自动手动)。 */
static volatile uint8_t scan_mode = SCAN_MODE_AUTO;
/* ★通道启用位图(bit ch = 1 启用)★ 默认全 36 通道启用。
 * ★为什么不用 Cy_CapSense_SetPinState(HIGHZ) 去"关"一个仍在被扫描的 widget★
 * 那只能撑到下一轮扫描: ScanAllWidgets → CSDSetupWidget → CSDConnectSns 会把电极重新挂回
 * AMUXBUS(cy_capsense_csd_v2.c), 于是"关掉的通道"每轮都被重新连一次 —— 那是"UI 显示关闭但硬件
 * 仍在扫"的伪实现。
 * ★真正的关★ 用中间件自带的 widget-enable(CY_CAPSENSE_WD_ENABLE_MASK):
 *   Cy_CapSense_SetupWidget(sensing_v2.c:180) 先查 Cy_CapSense_IsWidgetEnabled, 不启用即返回
 *   BAD_PARAM; Cy_CapSense_ScanAllWidgets_V2 只 setup 第一个成功的 widget, 而链式推进的
 *   Cy_CapSense_SsPostAllWidgetsScan(sensing_v2.c:1604) 对失败的 widget **直接 skip**。
 *   ⇒ 禁用的 widget 永久不进入扫描序列, 从不被 CSDConnectSns 连接。
 * ★电气契约★ INACTIVE_SNS 是 Host 可选的 GND / High-Z / Shield，而非禁用通道的 High-Z 保证。
 *   Cy_CapSense_SsInitialize 会先将全部 IO 设为 STRONG_IN_OFF，CSDInitialize 又会按当前全局
 *   inactive 配置 blanket 全部电极；故每次 CSD 模式准备完成后都由
 *   _disabled_widgets_force_highz() 显式复位禁用 widget 的全部电极。
 * ★注意★ 只有 Cy_CapSense_Init(cy_capsense_control.c:147) 会把全部 widget 的 ENABLE|WORKING
 *   重新置起; Initialize/Enable 只清 ACTIVE 位(processing.c:126)不动 ENABLE。故需在 Init 之后
 *   重放本位图(见 _ch_enable_restore)。 */
#define CH_ENABLED_ALL                   (((uint64_t)1u << SENSOR_CHANNEL_COUNT) - 1u)
#define PROVISION_TIMEOUT_MS             (3000u)
/* PSoC 启动时先禁用全部 widget。RP 下发完整 enabled 位图后再用既有 APPLY 作为完成屏障；
 * 旧 RP 未发送该位图时，超时回退为全启用。禁用电极的 High-Z 由显式 helper 保证，
 * 不依赖 Host 可配置的 CSD inactive 状态。 */
static volatile uint64_t g_ch_enabled = 0u;
static volatile uint64_t g_provision_enable_seen = 0u;
static volatile bool g_provision_pending = true;
static volatile bool g_provision_apply_release = false;
/* 启用态刚被 SPI 改动、尚未由主循环落到中间件的通道位图(ISR 只置位, 重活在主循环)。 */
static volatile uint64_t ch_enable_dirty = 0u;

static inline bool _ch_is_enabled(uint32_t ch)
{
    return (ch < SENSOR_CHANNEL_COUNT) &&
           ((g_ch_enabled & ((uint64_t)1u << ch)) != 0u);
}

static inline bool _any_ch_enabled(void)
{
    return (g_ch_enabled & CH_ENABLED_ALL) != 0u;
}
static volatile uint32_t scan_count = 0u;  // 每完成一次全通道扫描 +1，自由递增(无触摸也增长)
static uint8_t snapshot_buffers[2u][SENSOR_SNAPSHOT_SIZE];
static uint16_t snapshot_generations[2u];
static volatile uint8_t published_snapshot_index;
static volatile bool published_snapshot_valid;
static uint8_t transfer_snapshot[SENSOR_SNAPSHOT_SIZE];
static uint16_t transfer_generation;
static bool transfer_valid;

/* ---- JIT 可加载触控算法引擎（ABI v1，见 psoc_algo_abi.h）----
 * algo_slot：可执行 1KB RAM 槽，4 字节对齐（Cortex-M0+ 从 SRAM 取指，thumb 入口 |1）。
 * algo_staging：ALGO_PAGE 写入的暂存区；ALGO_END 只置 pending，由主循环校验 CRC16 后 commit。 */
static uint8_t algo_slot[ALGO_SLOT_SIZE] __attribute__((aligned(4)));
static uint8_t algo_staging[ALGO_SLOT_SIZE];
static volatile bool algo_valid = false;
static volatile uint16_t algo_len = 0u;
/* ALGO_BEGIN 声明的期望长度；ALGO_PAGE 越界保护用。 */
static volatile uint16_t algo_expected_len = 0u;
/* 主循环 commit 状态机：ISR 只置位，真正的 CRC 校验 + memcpy 由主循环执行。 */
static volatile bool algo_commit_pending = false;
static volatile uint16_t algo_commit_crc = 0u;
/* 逐通道持久 IO 记录：状态字段跨周期原地保留，仅输入字段每周期刷新。 */
static algo_io_t g_algo_io[SENSOR_CHANNEL_COUNT];
/* 记录上一周期各通道 base_active，用于检测非激活→激活边沿以清零 state[]。 */
static uint8_t algo_prev_active[SENSOR_CHANNEL_COUNT];
/* 每通道 16 位 ROM：上位机随算法下发的只读常量(如 per-channel fingerCap/Cp/阈值)，
 * 每周期填入 io->rom 供算法读取。算法不写。ALGO_SET_ROM/GET_ROM 读写。 */
static volatile uint16_t g_algo_rom[SENSOR_CHANNEL_COUNT];
/* 共享算法可设置变量(ABI cfg[8]): 上位机经 ALGO_SET_CFG 下发, 每次算法执行前拷入 io->cfg。
 * 供算法运行时可调参数(如基线偏移、阈值系数等), 上位机按源码声明的名字+默认值展示为可调项。 */
static volatile uint8_t g_algo_cfg[8] = {0u};
/* 毫秒计时：SysTick 每 1ms 回调递增。真实 ms 源，见 systick_ms_callback()。 */
static volatile uint32_t g_ms_tick = 0u;

/* 全局 CSD 配置 RAM 影子：生成的 cy_capsense_commonConfig 是 const(flash)不可运行时改，
 * 故 init 前把它拷到 RAM 并把 cy_capsense_context.ptrCommonConfig 指向此副本；
 * 之后 SET_GLOBAL 改此副本、APPLY 重初始化时中间件从 ptrCommonConfig 重算内部预计算而生效。 */
static cy_stc_capsense_common_config_t g_common_cfg_ram;
/* 启动时固件强制改写过哪些生成配置项(位掩码, 经 GPARAM_BOOT_OVERRIDE 只读上报)。
 * 有值即说明"设备实际配置 != 用户/生成配置给的值", 上位机必须据此告警, 不许静默不同步。 */
static uint8_t g_boot_override = 0u;

static void spi_slave_task(uint8_t rx_index);
static void spi_dma_isr(void);
static void spi_cs_isr(void);
static void spi_dma_init(void);
static void spi_dma_arm_tx(void);
static void spi_snapshot_latch_task(void);
/* 启用位图 → 中间件 widget 状态的重放。定义在参数区(见 _ch_enable_restore 处的说明), 但
 * initialize_capsense 里 Cy_CapSense_Init 之后就要用它, 故在此前置声明。 */
static inline void _ch_enable_restore(void);
static inline void _disabled_widgets_force_highz(void);
static inline void _prepare_csd_mode(void);

static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

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

/* CRC16-CCITT-FALSE：poly=0x1021, init=0xFFFF，覆盖 data[0,len)。用于算法 blob 上传校验。 */
static uint16_t algo_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0u; i < len; i++)
    {
        uint8_t byte = data[i];
        uint8_t bit;
        crc ^= (uint16_t)((uint16_t)byte << 8u);
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (uint16_t)((crc & 0x8000u) ? ((crc << 1u) ^ 0x1021u) : (crc << 1u));
        }
    }
    return crc;
}

/* ★用户手动 IDAC 增益档锁定★: 中间件的校准(Cy_CapSense_CalibrateAllWidgets/CalibrateWidget,
 * 见 cy_capsense_csd_v2.c 校准入口)与 Cy_CapSense_Enable 一进来就把 widgetContext[ch].idacGainIndex
 * 无条件拉回全局起点档 csdIdacGainInitIndex, 于是用户经 PARAM_IDAC_GAIN 设的增幅被静默改回 ——
 * 上位机显示与设备实际不符。故记录"用户显式设过的通道 + 其值", 在所有会重置增益档的动作之后恢复。
 * 语义: 显式 PARAM_IDAC_GAIN 即锁定该通道; 修改全局 GPARAM_IDAC_GAIN_INIT 视为用户改了起点档 →
 * 清空全部锁定(以全局值为准)。GET_PARAM 恒读 widgetContext 实际生效值。 */
typedef struct
{
    uint64_t mask;                        /* bit ch = 该通道增益档被用户锁定 */
    uint8_t  gain[SENSOR_CHANNEL_COUNT];  /* 锁定通道的用户增益档(0..6) */
} idac_gain_lock_t;
static volatile idac_gain_lock_t g_idac_lock;

static inline void _idac_lock_clear(void)
{
    uint32_t i;
    g_idac_lock.mask = 0u;
    for (i = 0u; i < SENSOR_CHANNEL_COUNT; i++) { g_idac_lock.gain[i] = 0u; }
}

/* 把锁定通道的增益档写回 widgetContext。target 为 0..35 时只恢复该 widget，
 * SENSOR_CH_ALL 时恢复全部锁定 widget。CSDv2 会在下一次 ScanWidget 内部装载该 widget 的
 * idacGainIndex，故这里绝不调用全局 Initialize/SsInitialize 破坏其它 widget 的运行状态。 */
static inline bool _idac_lock_restore(uint8_t target)
{
    const uint32_t first = (target < SENSOR_CHANNEL_COUNT) ? target : 0u;
    const uint32_t last = (target < SENSOR_CHANNEL_COUNT) ? ((uint32_t)target + 1u) : SENSOR_CHANNEL_COUNT;
    uint32_t w;
    bool changed = false;
    if (g_idac_lock.mask == 0u) { return false; }
    for (w = first; w < last; w++)
    {
        if ((g_idac_lock.mask & ((uint64_t)1u << w)) != 0u)
        {
            cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[w];
            if (wc->idacGainIndex != g_idac_lock.gain[w])
            {
                wc->idacGainIndex = g_idac_lock.gain[w];
                changed = true;
            }
        }
    }
    return changed;
}

/* ★逐通道校准必须在"该通道自己的增益档"下进行★
 * cy_capsense_csd_v2.c:1713 的 CalibrateWidget 一进来就 idacGainIndex = csdIdacGainInitIndex(全局
 * 起点档), auto-gain 之后只会往低档走 —— 于是"用户给该通道设的 PARAM_IDAC_GAIN"在校准过程中被
 * 全局档顶替: 解出的 idacMod 属于全局档, 事后 _idac_lock_restore 又把档改回用户值, 二者不自洽
 * (频率自适应尤其明显: 每一档试探的通过/失败判定都用的全局档, 找到的分频对不上该通道的档)。
 * 故 per-channel 校准统一走本助手: 锁定通道把全局起点档临时替换为该通道的锁定档, 校准后还原,
 * 使中间件的强制复位正好落在该通道自己的档上(语义 = 该通道的起点档)。未锁定通道零行为变化。 */
static inline bool _calibrate_widget_locked(uint32_t ch)
{
    bool ok;
    uint8_t saved_init = g_common_cfg_ram.csdIdacGainInitIndex;
    const bool locked = (ch < SENSOR_CHANNEL_COUNT) &&
                        ((g_idac_lock.mask & ((uint64_t)1u << ch)) != 0u);
    if (locked) { g_common_cfg_ram.csdIdacGainInitIndex = g_idac_lock.gain[ch]; }
    ok = (CY_CAPSENSE_STATUS_SUCCESS == Cy_CapSense_CalibrateWidget(ch, &cy_capsense_context));
    /* 只在字段仍是我们写进去的临时值时还原: 校准期间 SPI ISR 若受理了 GPARAM_IDAC_GAIN_INIT,
     * 盲目还原会把用户刚设的新全局起点档冲掉。 */
    if (locked && (g_common_cfg_ram.csdIdacGainInitIndex == g_idac_lock.gain[ch]))
    {
        g_common_cfg_ram.csdIdacGainInitIndex = saved_init;
    }
    return ok;
}

/* 自动校准/Enable 之后恢复全部锁定档。每个 widget 的下一次扫描会从它自己的
 * widgetContext 装载 IDAC，不能为此触发全局 Initialize 或重置其它通道基线。 */
static inline void _idac_lock_reapply(void)
{
    (void)_idac_lock_restore(SENSOR_CH_ALL);
}

/* 建立全局配置 RAM 影子并重定向 ptrCommonConfig。必须在 Cy_CapSense_Init 前调用。 */
static void initialize_common_cfg_shadow(void)
{
    memcpy(&g_common_cfg_ram, cy_capsense_context.ptrCommonConfig, sizeof(g_common_cfg_ram));

    /* ★编译时 IDAC 增强(修复 raw 满量程 railed)★：生成配置默认 IDAC 增益档=0, 补偿电流过小,
     * 启动 Cy_CapSense_Enable 的自动校准无法把 raw 拉离满量程 → 全通道 raw=maxRawCount/diff=0(不可用)。
     * 实测增益档 4 时自动校准收敛到 ~RAW_TARGET(85%)。此处在【首次 Init 前】写 RAM 影子, 使启动
     * Enable 首扫即用增益 4 干净校准(无需运行时 DeInit 重配, 避开运行时改时钟致 15Hz 的降速 bug)。
     * 运行时仍可经 GLOBAL_SET(IDAC_GAIN_INIT) / PARAM_SET(IDAC_GAIN) 覆盖。 */
    if (g_common_cfg_ram.csdIdacGainInitIndex < 4u) {
        g_common_cfg_ram.csdIdacGainInitIndex = 4u;
        g_boot_override |= 0x01u;
    }
    /* ★只拦真正非法值★: 运行时 cmd_set_global 放行 1..99, 这里原先却夹到 [60,90], 于是用户设的
     * 50 每次启动被静默改成 85 —— 上位机显示 50、设备实际 85, 长期不同步。现在与运行时同口径,
     * 只有 0 / >=100(会让自动校准发散→railed)才回退默认。被改写时置位经 GPARAM_BOOT_OVERRIDE 上报。 */
    if ((g_common_cfg_ram.csdRawTarget == 0u) || (g_common_cfg_ram.csdRawTarget >= 100u)) {
        g_common_cfg_ram.csdRawTarget = 85u;
        g_boot_override |= 0x02u;
    }
    /* ★出厂默认(ROM)的 GND 在本 36 通道面板上不可用, 故只改"默认", 不改"用户选择"★
     * GND 让 35 个非激活电极接地, 每通道多约 4.3ms 固定建立开销(实测 172.5µs → 4386µs/通道),
     * 叠加 MFS 三频后实测整轮扫描约 833ms(1.2Hz), UI 上表现为"实测探测周期 1000ms / 期望 5.8ms,
     * 倍率 ×172"且 raw 全通道同值不抖 —— 这就是采样异常的直接成因。
     * ★与之前"不得静默替用户决定"的结论并不矛盾★: 那条针对的是【用户已选 GND】被启动改写;
     * 而这里改的是【生成配置的出厂默认】—— store 为空时用户根本还没做过选择, 落在不可用的默认上
     * 只会让"恢复默认"救不回来(空 store → GND → 采样不可信 → 拒绝固化基线 → 永远卡住)。
     * 用户显式选择仍由 store 的 GLOBAL_SET(INACTIVE_SNS) 在 provision 时覆盖本默认, 优先级更高。
     * 被改写时置位, 经 GPARAM_BOOT_OVERRIDE 上报, UI 可见, 不是静默行为。 */
    if (g_common_cfg_ram.csdInactiveSnsConnection == (uint8_t)CY_CAPSENSE_SNS_CONNECTION_GROUND) {
        g_common_cfg_ram.csdInactiveSnsConnection = (uint8_t)CY_CAPSENSE_SNS_CONNECTION_HIGHZ;
        g_boot_override |= 0x04u;
    }

    cy_capsense_context.ptrCommonConfig = &g_common_cfg_ram;
}

/* 写全局 CSD 配置到 RAM 影子(运行时可配项)。改后需 APPLY 重初始化生效。 */
static void cmd_set_global(uint8_t gparam_id, uint32_t value)
{
    switch (gparam_id)
    {
        case GPARAM_INACTIVE_SNS:
            /* GND 会把未激活电极接地，显著增加被测电极对地寄生电容，转换建立时间随之拉长；
             * 36 段面板的扫描周期可从约 6ms 暴涨到 142–200ms，且更易 raw railed。仍允许用户显式选择，
             * 但面板默认应使用 High-Z(2)。 */
            if ((value == 1u) || (value == 2u) || (value == 4u))
            {
                g_common_cfg_ram.csdInactiveSnsConnection = (uint8_t)value;
            }
            break;
        /* 防护: IDAC 增益档 0..7; IDAC min 0..127(7位); 校准目标 1..99%(0/≥100 会让自动校准发散→railed)。 */
        /* IDAC 增益档合法索引 0..6(idacGainTable 共 CY_CAPSENSE_IDAC_GAIN_NUMBER=7 项), 索引7越界会读
         * 到表外 gainReg → 写非法 IDAC → 扫描挂死/看门狗复位(崩溃)。夹到 <=6。 */
        /* 改起点档 = 用户重新给定全局增益基准 → 清空全部手动锁定, 以全局值为准。 */
        case GPARAM_IDAC_GAIN_INIT: if (value <= 6u)   { g_common_cfg_ram.csdIdacGainInitIndex = (uint8_t)value; _idac_lock_clear(); } break;
        case GPARAM_IDAC_MIN:       if (value <= 127u) { g_common_cfg_ram.csdIdacMin           = (uint8_t)value; } break;
        case GPARAM_RAW_TARGET:     if ((value >= 1u) && (value <= 99u)) { g_common_cfg_ram.csdRawTarget = (uint8_t)value; } break;
        /* ★补齐缺失的围栏★: 这两项原先无任何检查, 直接 (uint8_t)value 截断 —— 上位机写 300
         * 会被静默变成 44, 而 SET_GLOBAL 的回显以前送回的是"请求值"而非"存储值", 于是上位机
         * 完全看不出失败(唯一一条静默限制路径, 其余非法值都由 RP2040 侧 NAK 拦下)。
         * 现在与其它项同口径: 超范围直接不写, 保持原值, 由回显的存储值让上位机据实回读。 */
        case GPARAM_MFS_DIV_F1:     if (value <= 255u) { g_common_cfg_ram.csdMfsDividerOffsetF1 = (uint8_t)value; } break;
        case GPARAM_MFS_DIV_F2:     if (value <= 255u) { g_common_cfg_ram.csdMfsDividerOffsetF2 = (uint8_t)value; } break;
        /* IDAC 感应配置(sourcing/sinking): 运行时可设的充电方向, 影响灵敏度极性/范围。 */
        case GPARAM_IDAC_SENSE_CONFIG: g_common_cfg_ram.csdChargeTransfer = (value != 0u) ? (uint8_t)CY_CAPSENSE_IDAC_SINKING : (uint8_t)CY_CAPSENSE_IDAC_SOURCING; break;
        /* ★由关转开必须立刻做一次真实校准★: 原生 CapSense 的"IDAC 自动校准"语义是
         * Enable/Init 时按 csdRawTarget 解出各通道 IDAC。此前本项只改标志, 要等下一次 APPLY 才
         * 生效, 且半自动手动模式下只补校"脏通道" —— 用户勾选后没有任何参数变更时 dirty 为空,
         * 表现为"勾了不拉基线、IDAC 根本没运行"。故 0→1 沿置 calibrate_pending, 由主循环执行
         * CalibrateAllWidgets + InitializeAllBaselines(等价于原生 Enable 的自动校准结果)。
         * 只在上升沿触发: 重复写 1 不该反复打断扫描。 */
        case GPARAM_AUTO_CALIBRATE_EN:
            if ((value != 0u) && !g_auto_calibrate) { calibrate_pending = true; }
            g_auto_calibrate = (value != 0u);
            break;
        default: break;
    }
    /* 仅改影子, 不在此重初始化。全部全局项设完后由 GLOBAL_COMMIT 触发一次完整重初始化,
     * 避免每项都重初始化导致 SmartSense 反复重校准的漂移/降速(实测会致 8Hz/raw 饱和)。 */
}

static uint32_t cmd_get_global(uint8_t gparam_id)
{
    switch (gparam_id)
    {
        case GPARAM_INACTIVE_SNS:   return g_common_cfg_ram.csdInactiveSnsConnection;
        case GPARAM_IDAC_GAIN_INIT: return g_common_cfg_ram.csdIdacGainInitIndex;
        case GPARAM_IDAC_MIN:       return g_common_cfg_ram.csdIdacMin;
        case GPARAM_RAW_TARGET:     return g_common_cfg_ram.csdRawTarget;
        case GPARAM_MFS_DIV_F1:     return g_common_cfg_ram.csdMfsDividerOffsetF1;
        case GPARAM_MFS_DIV_F2:     return g_common_cfg_ram.csdMfsDividerOffsetF2;
        case GPARAM_IDAC_SENSE_CONFIG: return (g_common_cfg_ram.csdChargeTransfer == (uint8_t)CY_CAPSENSE_IDAC_SINKING) ? 1u : 0u;
        case GPARAM_AUTO_CALIBRATE_EN: return g_auto_calibrate ? 1u : 0u;
        case GPARAM_BOOT_OVERRIDE:  return g_boot_override;
        /* SPI 链路只读诊断(24 位截断足够: 这些计数在秒级量级内远小于 16M)。 */
        case GPARAM_DBG_RX_FRAMES:    return spi_dbg.clk_now      & 0xFFFFFFu;
        case GPARAM_DBG_RX_BAD_MAGIC: return spi_dbg.rx_bad_magic & 0xFFFFFFu;
        case GPARAM_DBG_CS_RESYNC:    return spi_dbg.cs_resync     & 0xFFFFFFu;
        case GPARAM_DBG_TX_ARM:       return spi_dbg.clk_set_cnt   & 0xFFFFFFu;
        case GPARAM_DBG_RX_LEFTOVER:  return spi_dbg.rx_leftover   & 0xFFFFFFu;
        default: return 0u;
    }
}

/* ★通道参数归一(修"不同通道分辨率口径不一样")★: 生成配置源自 buttons+slider 示例,
 * widget 0-2 为 resolution=10/snsClk=8, widget 3-35 为 resolution=12/snsClk=4(不均匀) →
 * 各通道 raw 满量程口径不同(1023 vs 4095)、灵敏度/时钟不一致, 校准与显示都错乱。
 * 本工程面板是 36 段均匀触控, 必须统一。此处把全部 widget 归一到统一分辨率(12)+时钟分频(8),
 * 使全通道口径一致、首次 Enable 自动校准(auto-gain)在统一基准上收敛。运行时仍可经 SET_PARAM 覆盖。
 * ★snsClk=32★: 本工程面板 Cp 高(~100pF)。实测 div=8 时传感器在高频下来不及建立→raw 饱和在高位、
 * 无 headroom(全通道 frozen 无抖动)且校准压不到低目标%; div=32 时全 36 通道校准准确跟踪 25%~85%
 * 目标且均有抖动(--calib-track 实测)。故默认降频到 32。运行时可经频率自适应(AUTO_TUNE)进一步下探。 */
static void normalize_widget_params(void)
{
    uint32_t w;
    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[w];
        wc->resolution = 12u;
        wc->snsClk     = 32u;
    }
}

/* ★保全逐通道硬件口径, 抵消 Init 的 ROM 重铺★
 * Cy_CapSense_Init()(内部 Restore)会把整个 widgetContext 从生成配置重铺, 于是运行时的
 * resolution/snsClk 被打回生成值(widget0-2: res=10/clk=8, widget3-35: res=12/clk=4)。
 * 后果就是"DIV 异常固化": 每次全局应用后分频又变回 8, 高频下传感器建立不足 → raw 逼近满量程、
 * 且各通道满量程口径不一致。这里在 Init 前后成对调用即可保全【当前生效值】——
 * 既保住启动归一的 32, 也保住用户 SET_PARAM / AUTO_TUNE 的逐通道分频(不能像归一那样一律冲成 32)。
 * 与 _idac_lock_save/restore 是同一模式, 只是管的字段不同。 */
typedef struct
{
    uint16_t resolution;
    uint16_t sns_clk;
} widget_hw_t;

static widget_hw_t g_widget_hw[SENSOR_CHANNEL_COUNT];

static inline void _widget_hw_save(void)
{
    uint32_t w;
    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        g_widget_hw[w].resolution = cy_capsense_tuner.widgetContext[w].resolution;
        g_widget_hw[w].sns_clk    = cy_capsense_tuner.widgetContext[w].snsClk;
    }
}

static inline void _widget_hw_restore(void)
{
    uint32_t w;
    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        cy_capsense_tuner.widgetContext[w].resolution = g_widget_hw[w].resolution;
        cy_capsense_tuner.widgetContext[w].snsClk     = g_widget_hw[w].sns_clk;
    }
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
        normalize_widget_params();   /* 首次校准(Enable)前把全部通道分辨率/时钟归一, 消除口径不一致 */
        /* Init 刚把全部 widget 置成 ENABLE|WORKING；启动位图仍是全禁用，先重放到中间件。
         * Enable 内部仍会把所有 IO 强推到默认态；返回后必须显式恢复禁用电极的 High-Z。 */
        _ch_enable_restore();
        Cy_CapSense_Enable(&cy_capsense_context);
        _prepare_csd_mode();
        /* 归一 + Enable 之后立刻取样: 这就是"启动结束时真正生效的分频", 供带外 SWD 判定
         * 32 是否连启动都没活下来(见 spi_dbg.clk_boot 注释的判据表)。 */
        spi_dbg.clk_boot = cy_capsense_tuner.widgetContext[0].snsClk;
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

        /* ★禁用通道一律发布全 0★ 它已不参与扫描, sensor->raw/bsln 仍是关闭前的陈旧值 ——
         * 原样上报会让上位机把"关掉的通道"显示成一条恒定不动的假读数(还会被停滞检测判成异常)。
         * 全 0 是明确的"无数据"约定(status=0 ⇒ 未触摸), 与上位机的灰显一致。 */
        if (!_ch_is_enabled(channel))
        {
            snapshot_write_u16(&destination[offset], 0u);
            snapshot_write_u16(&destination[offset + 2u], 0u);
            snapshot_write_u16(&destination[offset + 4u], 0u);
            destination[offset + 6u] = 0u;
            continue;
        }
        snapshot_write_u16(&destination[offset], sensor->raw);
        snapshot_write_u16(&destination[offset + 2u], sensor->bsln);
        snapshot_write_u16(&destination[offset + 4u], sensor->diff);
        destination[offset + 6u] = sensor->status;
    }

    spi_dbg.snap_pub++;
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

    /* ★通知必须在发布之后★ 翻转即"这一代已经可读"。本函数只在 CapSense NOT_BUSY 分支里、
     * 且在 update_touch_frame() 之后被调用, 所以翻转时触控快帧与完整快照都已是新的一份 ——
     * RP2040 看到翻转就可以直接取, 不会取到半新半旧。 */
    Cy_GPIO_Inv(SENSOR_INT1_PORT, SENSOR_INT1_NUM);
}

/* 用可加载算法 blob 逐通道计算激活位。返回 0/1。仅在 algo_valid 时被调用；
 * 坏 blob 死循环由 RP2040 侧 PONG 心跳兜底硬复位处理(见设计文档 §1)，PSoC 端不需看门狗。 */
static uint32_t algo_engine_run_channel(uint32_t ch, uint32_t base_active)
{
    const cy_stc_capsense_sensor_context_t * sensor =
        &cy_capsense_context.ptrWdConfig[ch].ptrSnsContext[0u];
    const cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[ch];
    algo_io_t * io = &g_algo_io[ch];
    algo_fn_t algo_fn = (algo_fn_t)(void *)((uintptr_t)algo_slot | 1u);

    /* 非激活→激活边沿：清零持久 state，供算法重新累积包络/滤波历史。 */
    if ((base_active != 0u) && (algo_prev_active[ch] == 0u))
    {
        memset(io->state, 0, sizeof(io->state));
    }
    algo_prev_active[ch] = (uint8_t)base_active;

    io->baseline     = sensor->bsln;
    io->diff         = sensor->diff;
    io->raw          = sensor->raw;
    io->noise_th     = wc->noiseTh;
    io->nnoise_th    = wc->nNoiseTh;
    io->max_raw      = (wc->maxRawCount != 0u) ? wc->maxRawCount : 1024u;
    io->finger_th    = wc->fingerTh;
    io->base_active  = (uint16_t)base_active;
    io->now_ms       = g_ms_tick;
    io->ch           = ch;
    io->rom          = g_algo_rom[ch];   /* 每通道只读 ROM(上位机下发) */
    /* cfg[8] 全通道共享的可设置变量(上位机 ALGO_SET_CFG 下发)。 */
    for (uint32_t k = 0u; k < 8u; k++) { io->cfg[k] = g_algo_cfg[k]; }

    /* 每轮先清点灯请求, 由算法重新声明: 否则换成不写 out_led 的算法后, 上一个算法(如 LED 演示)
     * 留下的 1 会让白灯永久亮着 —— 那又变成了"用户改不掉的灯"。 */
    io->out_led = 0u;

    algo_fn(io);
    return io->out_active;
}

/* 从 capsense 上下文计算 36 区 on/off 位图，更新触控帧（临界区保护，防 SPI ISR 读到半更新）。 */
static void update_touch_frame(void)
{
    uint8_t mask[5] = {0u, 0u, 0u, 0u, 0u};
    uint32_t ch;
    uint32_t st;
    bool use_algo = algo_valid;
    /* 点灯请求只来自算法显式写入的 out_led; 与触控判定(out_active)彻底解耦。 */
    bool algo_led = false;

    for (ch = 0u; ch < SENSOR_CHANNEL_COUNT; ch++)
    {
        uint32_t base_active;
        uint32_t active;

        /* ★禁用通道恒不触发★ 它不参与扫描也不参与处理, 既不该出现在触控掩码里, 也不该让
         * 上一次的算法状态继续跑(那会让关闭的通道仍能点灯/仍能报按下)。 */
        if (!_ch_is_enabled(ch))
        {
            g_algo_io[ch].out_active = 0u;
            g_algo_io[ch].out_led = 0u;
            algo_prev_active[ch] = 0u;
            continue;
        }
        base_active = Cy_CapSense_IsWidgetActive((uint32_t)ch, &cy_capsense_context);

        if (use_algo)
        {
            active = algo_engine_run_channel(ch, base_active);
            /* 只认算法自己写的 out_led; 算法不写 → 恒 0 → 不点灯。 */
            if (g_algo_io[ch].out_led != 0u) { algo_led = true; }
        }
        else
        {
            active = base_active;
            /* 未加载 JIT 时 trace 仍需反映实测触控态，不能留下上次算法的陈旧结果。 */
            g_algo_io[ch].out_active = (uint16_t)base_active;
            /* 无算法即无点灯请求: 原生 CapSense 触控不得点灯(否则又变成固件写死的触控反馈)。 */
            g_algo_io[ch].out_led = 0u;
        }

        if (active != 0u)
        {
            mask[ch >> 3u] |= (uint8_t)(1u << (ch & 7u));
        }
    }

    /* ★白 LED 只由算法显式请求(out_led)驱动★: 此前这里写的是 `use_algo && algo_active`,
     * 即把触控判定结果当成点灯信号写死 —— 任何算法(含默认 v3.1 HDR)只要判定触摸就必然亮灯,
     * 用户换算法也改不掉。现在改为汇总算法写出的 out_led: 不写该字段的算法一律不点灯。
     * 触控帧输出不受影响, 协议语义不变。 */
    g_any_active = use_algo && algo_led;

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
    uint32_t st = Cy_SysLib_EnterCriticalSection();
    spi_dma.tx_frame[0] = touch_frame[0]; spi_dma.tx_frame[1] = touch_frame[1];
    spi_dma.tx_frame[2] = touch_frame[2]; spi_dma.tx_frame[3] = touch_frame[3];
    spi_dma.tx_frame[4] = touch_frame[4]; spi_dma.tx_frame[5] = touch_frame[5];
    spi_dma.tx_frame[6] = touch_frame[6];
    Cy_SysLib_ExitCriticalSection(st);
    spi_dma_arm_tx();
}

// ---- Phase A：运行时 CSD 参数读写（直写 cy_capsense_tuner.widgetContext RAM）----
// 阈值类(fingerTh/noiseTh/hysteresis/onDebounce/lowBslnRst)在下次 ProcessAllWidgets 自动生效；
// 硬件类(resolution/snsClk/idacMod)需 APPLY(重扫/重校准)。单字段 16/8 位写在 CM0+ 上原子。
/* CSD 参数合法性防护: 非法值会让转换railed(满量程)/时钟异常/校准发散, 故在应用点拒绝越界值,
 * 保留原值不变。范围依据 CSDv2(cy_capsense_structure.h):
 *   RESOLUTION 6..16 位; SNS_CLK_DIV 1..255(0会除零); IDAC_MOD 0..127(7位);
 *   IDAC_GAIN 增益档 0..6(表7项, 索引7越界崩溃); SNS_CLK_SOURCE 低7位(去 AUTO 0x80)取值 0..6。
 * ★三处同源★ 与 main_firmware/src/service/sensor_link/sensor_link.cpp::_handle_param_set 及
 * control_software/src/proto/telemetry.rs::param_fence 必须逐位等价, 任何一处改动三处同改。 */
static bool _param_value_legal(uint8_t param_id, uint32_t value)
{
    switch (param_id)
    {
        case PARAM_RESOLUTION:     return (value >= 6u)  && (value <= 16u);
        case PARAM_SNS_CLK_DIV:    return (value >= 1u)  && (value <= 255u);
        case PARAM_IDAC_MOD:       return (value <= 127u);
        case PARAM_IDAC_GAIN:      return (value <= 6u);   /* 增益档 0..6(表7项,索引7越界崩溃) */
        case PARAM_SNS_CLK_SOURCE: return ((value & 0x7Fu) <= 6u);
        case PARAM_ENABLED:        return (value <= 1u);   /* 硬件开关: 只有 0/1 有意义 */
        /* 阈值/迟滞/消抖类为 16/8 位任意值, 无硬件危险, 不额外限制。 */
        default: return true;
    }
}

/* 返回是否被接受(合法); 非法直接拒绝, 供 SPI 层回显真实(未改)值让上位机据实回读。 */
static bool cmd_set_param(uint8_t ch, uint8_t param_id, uint32_t value)
{
    if (ch >= SENSOR_CHANNEL_COUNT) return false;
    if (!_param_value_legal(param_id, value)) return false;   // 防护: 拒绝非法值
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
        case PARAM_SNS_CLK_DIV:
            wc->snsClk = (uint16_t)value;
            /* 记录"谁把分频改了": 只有经 SET_PARAM 这条路才会累加。若 clk_now 变成 8 而本计数为 0,
             * 说明是中间件内部路径(Init/Initialize/Enable)改的, 不是上位机/store 推的。 */
            spi_dbg.clk_set_cnt++;
            spi_dbg.clk_set_last = (value & 0xFFFFu) | ((uint32_t)ch << 16u);
            break;
        case PARAM_IDAC_MOD:      wc->idacMod[0]   = (uint8_t)value;  break;
        case PARAM_SNS_CLK_SOURCE:wc->snsClkSource = (uint8_t)value;  break;
        /* 用户显式设增幅 → 锁定该通道, 后续任何校准/Enable 冲回后都会被恢复成此值。 */
        case PARAM_IDAC_GAIN:     wc->idacGainIndex= (uint8_t)value;
                                  g_idac_lock.gain[ch] = (uint8_t)value;
                                  g_idac_lock.mask |= ((uint64_t)1u << ch);
                                  break;
        /* ★启用开关只在 ISR 里改位图, 真正的生效(改 widget 状态 / 重校准 / 基线)在主循环★
         * Cy_CapSense_SetWidgetStatus 内部会走 SwitchSensingMode(重配 CSD HW), 那是不能在 ISR 里
         * 做的事; 且必须在 NOT_BUSY 窗口做, 否则会打断正在进行的转换。 */
        case PARAM_ENABLED: {
            const uint64_t bit = ((uint64_t)1u << ch);
            const bool want = (value != 0u);
            /* 启动 provision 需看到每个通道的明确值，0 也是有效配置，不能靠位图变化推断。 */
            if (g_provision_pending) { g_provision_enable_seen |= bit; }
            if (want == ((g_ch_enabled & bit) != 0u)) break;   /* 无变化: 不惊动扫描 */
            if (want) { g_ch_enabled |= bit; } else { g_ch_enabled &= ~bit; }
            ch_enable_dirty |= bit;
            break;
        }
        default: return false;
    }
    if ((param_id == PARAM_SNS_CLK_DIV) || (param_id == PARAM_RESOLUTION) ||
        (param_id == PARAM_IDAC_GAIN))
    {
        idac_dirty_mask |= ((uint64_t)1u << ch);
    }
    return true;
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
        /* 读位图而不是读 widgetContext.status: 位图是本固件的意图真相源, 而 status 会被
         * Cy_CapSense_Init 重置(重放前的那一瞬会读出不一致值)。 */
        case PARAM_ENABLED:       return _ch_is_enabled(ch) ? 1u : 0u;
        default: return 0u;
    }
}

/* 把 g_ch_enabled 位图重放到中间件的 widget 状态位。
 * ★何时必须调★ 只有 Cy_CapSense_Init 会把全部 widget 的 ENABLE|WORKING 重新置起
 * (cy_capsense_control.c:147), 所以启动 Init 之后与 GLOBAL_COMMIT 的 Init 之后各调一次即可 ——
 * 与 _widget_hw_restore / _idac_lock_restore 完全同一模式(都是"抵消 Init 的 ROM 重铺")。
 * 这里直写 status 位而不调 Cy_CapSense_SetWidgetStatus: 后者附带 SwitchSensingMode(UNDEFINED),
 * 在"Init 之后紧接 Initialize"的序列里会白白多一次 CSD 模式来回。 */
static inline void _ch_enable_restore(void)
{
    uint32_t w;
    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[w];
        if (_ch_is_enabled(w))
        {
            wc->status |= (uint8_t)(CY_CAPSENSE_WD_ENABLE_MASK | CY_CAPSENSE_WD_WORKING_MASK);
        }
        else
        {
            wc->status &= (uint8_t)~(uint8_t)(CY_CAPSENSE_WD_ENABLE_MASK | CY_CAPSENSE_WD_ACTIVE_MASK);
        }
    }
}

/* INACTIVE_SNS is a host-selected operating policy, not the electrical state of disabled
 * channels. SetPinState() follows each CSD widget's actual sensor/electrode layout and
 * applies High-Z to every pin of a ganged electrode. Call only while the middleware is idle. */
static inline void _disabled_widgets_force_highz(void)
{
    uint32_t widget;
    for (widget = 0u; widget < cy_capsense_context.ptrCommonConfig->numWd; widget++)
    {
        const cy_stc_capsense_widget_config_t * cfg = &cy_capsense_context.ptrWdConfig[widget];
        uint32_t sensor;
        if (_ch_is_enabled(widget) || (cfg->senseMethod != CY_CAPSENSE_CSD_GROUP)) { continue; }
        for (sensor = 0u; sensor < cfg->numSns; sensor++)
        {
            (void)Cy_CapSense_SetPinState(widget, sensor, CY_CAPSENSE_HIGHZ, &cy_capsense_context);
        }
    }
}

/* CSDInitialize applies the configurable inactive state to every electrode. Preparing CSD
 * before a calibration or ScanAllWidgets keeps disabled electrodes High-Z without ISR work. */
static inline void _prepare_csd_mode(void)
{
    if (cy_capsense_context.ptrActiveScanSns->currentSenseMethod != CY_CAPSENSE_CSD_GROUP)
    {
        (void)Cy_CapSense_SwitchSensingMode(CY_CAPSENSE_CSD_GROUP, &cy_capsense_context);
    }
    _disabled_widgets_force_highz();
}

/* Focus 扫描目标(主循环专用)：仅当 ISR 存的 ch 当前启用且租约未过期时返回该 ch；否则回退全通道。
 * ★为什么需要复核★ ISR 只记录命令，主循环必须按当前时间与启用位图确认"租约是否失效、通道是否
 * 已被禁用"，返回全通道哨兵(SENSOR_CH_ALL)让扫描/处理回到正常路径，绝不遗留坏状态。 */
static inline uint8_t _focus_scan_target(void)
{
    uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
    const uint8_t req = focus_scan_ch;
    const uint32_t until = focus_scan_until_ms;
    Cy_SysLib_ExitCriticalSection(interrupt_state);
    if (req >= SENSOR_CHANNEL_COUNT) return SENSOR_CH_ALL;
    if (!_ch_is_enabled(req)) return SENSOR_CH_ALL;
    if (g_ms_tick >= until) {
        /* 租约到期：主循环强制回退，ISR 里的状态一并清掉(ISR 下次收到续租会重建)。 */
        interrupt_state = Cy_SysLib_EnterCriticalSection();
        focus_scan_ch = SENSOR_CH_ALL;
        focus_scan_until_ms = 0u;
        Cy_SysLib_ExitCriticalSection(interrupt_state);
        return SENSOR_CH_ALL;
    }
    return req;
}

/* 只初始化【启用】通道的基线。替代 Cy_CapSense_InitializeAllBaselines ——
 * 后者(cy_capsense_filter.c:407)不查 enable, 会把禁用通道的基线设成它那份陈旧 raw,
 * 白做 36 次无意义写入, 也让"禁用通道不参与任何处理"这条约束出现例外。 */
static inline void _initialize_enabled_baselines(void)
{
    uint32_t w;
    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        if (_ch_is_enabled(w))
        {
            Cy_CapSense_InitializeWidgetBaseline(w, &cy_capsense_context);
        }
    }
}

/* 一次完整扫描的等待预算(ms)。36 通道 res=12/div=32 实测约 40ms, MFS 三频约 120ms; 500ms 是
 * 安全上界, 只用于"卡死时别把主循环永远堵住", 正常路径远早于此返回。 */
#define SCAN_SETTLE_BUDGET_MS            (500u)

/* ★基线只能取自"当前配置下真实扫出来的 raw"★
 * Cy_CapSense_InitializeWidgetBaseline 做的是 bsln = 该通道当前的 raw(cy_capsense_filter.c),
 * 它不会自己去扫一遍。于是"重配硬件 → 立刻初始化基线"这个序列拿到的是**上一套配置**留下的
 * 陈旧 raw:
 *   - MEASURE_CP: BIST 全程接管 CSD 硬件, 结束时 sensor context 里的 raw 与正常扫描口径无关;
 *   - GLOBAL_APPLY / 非校准 APPLY: inactive_sns/IDAC/MFS/分频刚变, 旧 raw 与新配置差一个量级。
 * 一旦基线被钉在这种陈旧值上, 下一次真实扫描的 diff 立刻远超 fingerTh → 判为按下 → 而 CapSense
 * 在通道处于 active 时**冻结基线更新** → 该通道永久 status=1、diff 恒定, 自己再也走不出来
 * (实测 CH10/CH28: bsln=344 恒定, raw≈2500, diff≈2200, 两个通道同一个错值)。
 * 故所有重配之后统一走本函数: 先在新配置下完成一次全通道扫描, 再拿这份 raw 做基线。 */
static inline void _scan_then_initialize_baselines(void)
{
    uint32_t t0;
    if (!_any_ch_enabled()) { return; }   /* 无启用通道: 无扫描可做, 也无基线可立 */
    t0 = g_ms_tick;
    while ((CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) &&
           ((uint32_t)(g_ms_tick - t0) < SCAN_SETTLE_BUDGET_MS)) { }
    if (CY_CAPSENSE_STATUS_SUCCESS == Cy_CapSense_ScanAllWidgets(&cy_capsense_context))
    {
        t0 = g_ms_tick;
        while ((CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) &&
               ((uint32_t)(g_ms_tick - t0) < SCAN_SETTLE_BUDGET_MS)) { }
    }
    _initialize_enabled_baselines();
}

/* 主循环(NOT_BUSY 窗口)落实 SPI 收到的启用/禁用请求。
 * 禁用: widget-enable 将其排除出扫描；SetWidgetStatus 会退出 CSD 并按全局 inactive 配置重置
 *       所有电极，故每次状态变更后立即切回 CSD 并显式恢复禁用电极 High-Z。
 * 启用: 恢复 enable 位后只给该 widget 重建 IDAC 与基线，绝不扰动其它通道。 */
static inline void _ch_enable_apply(void)
{
    uint64_t pending;
    uint32_t w;
    if (g_provision_pending) { return; }
    uint32_t st = Cy_SysLib_EnterCriticalSection();
    pending = ch_enable_dirty;
    ch_enable_dirty = 0u;
    Cy_SysLib_ExitCriticalSection(st);
    if (pending == 0u) { return; }

    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        if ((pending & ((uint64_t)1u << w)) == 0u) { continue; }
        const bool enable = _ch_is_enabled(w);
        (void)Cy_CapSense_SetWidgetStatus(w, CY_CAPSENSE_WD_ENABLE_MASK,
                                          enable ? CY_CAPSENSE_WD_ENABLE_MASK : 0u,
                                          &cy_capsense_context);
        _prepare_csd_mode();
        if (enable)
        {
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
            if (g_auto_calibrate) { (void)_calibrate_widget_locked(w); }
#endif
            (void)_idac_lock_restore((uint8_t)w);
            Cy_CapSense_InitializeWidgetBaseline(w, &cy_capsense_context);
        }
        else
        {
            /* 触控/算法残留清零: 关闭的通道必须立刻表现为"没被按下", 而不是冻在最后一帧。 */
            cy_capsense_tuner.widgetContext[w].status &= (uint8_t)~(uint8_t)CY_CAPSENSE_WD_ACTIVE_MASK;
            memset(&g_algo_io[w], 0, sizeof(g_algo_io[w]));
            algo_prev_active[w] = 0u;
            /* 关闭期间它不再被校准/自适应触碰, 脏位留着只会在下次 APPLY 时白跑一次校准。 */
            idac_dirty_mask &= ~((uint64_t)1u << w);
        }
    }
}

static inline void _recalibrate_dirty_channels(void)
{
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
    uint32_t w;

    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        /* 禁用通道不参与校准: 校准会连接电极(CSDCalibrateWidget → 扫描该 widget), 与"电极保持
         * 高阻"直接冲突; 且它的 IDAC 结果毫无用处。 */
        if (!_ch_is_enabled(w)) { continue; }
        if ((idac_dirty_mask & ((uint64_t)1u << w)) != 0u)
        {
            (void)_calibrate_widget_locked(w);   /* 锁定通道按其自身增益档校准 */
        }
    }
    /* 校准把增益档拉回全局起点档, 此处把用户锁定值写回; 调用方随后的 Initialize 使其下到硬件。 */
    (void)_idac_lock_restore(SENSOR_CH_ALL);
#endif
    idac_dirty_mask = 0u;
}

/* Enable() would initialize and immediately start its first scan before callers can repair
 * disabled pins. This replacement keeps the same all-enabled-widget calibration scope, but
 * prepares CSD and forces disabled electrodes High-Z before any calibration sample starts. */
static inline void _calibrate_enabled_channels(void)
{
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
    uint32_t w;
    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
    {
        if (_ch_is_enabled(w)) { (void)_calibrate_widget_locked(w); }
    }
#endif
    idac_dirty_mask = 0u;
    _idac_lock_reapply();
}

// 装载指令响应帧（直接填 TX FIFO；ISR 上下文，与 spi_load_touch 同）。
static void spi_load_cmd_response(uint8_t command, uint8_t b2, uint8_t b3, uint32_t val24)
{
    spi_dma.tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_dma.tx_frame[1] = command;
    spi_dma.tx_frame[2] = b2;
    spi_dma.tx_frame[3] = b3;
    spi_dma.tx_frame[4] = (uint8_t)(val24 & 0xFFu);
    spi_dma.tx_frame[5] = (uint8_t)((val24 >> 8u) & 0xFFu);
    spi_dma.tx_frame[6] = (uint8_t)((val24 >> 16u) & 0xFFu);
    spi_dma_arm_tx();
}

static uint16_t cmd_get_raw(uint8_t ch)
{
    if (ch >= SENSOR_CHANNEL_COUNT) return 0u;
    /* 与快照同口径: 禁用通道回 0(无数据), 不回关闭前的陈旧 raw。 */
    if (!_ch_is_enabled(ch)) return 0u;
    return cy_capsense_context.ptrWdConfig[ch].ptrSnsContext[0].raw;
}

// GET_STATS 响应: [magic, GET_STATS, 0, scan_count u32 LE(字节3..6)]
static void spi_load_stats(void)
{
    uint32_t sc = scan_count;   // CM0+ 上 32 位对齐读原子
    spi_dma.tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_dma.tx_frame[1] = SENSOR_CMD_GET_STATS;
    spi_dma.tx_frame[2] = g_op_busy;   // 处理中标志(1=主循环正在做重操作), 供 RP2040 轮询至真实完成
    spi_dma.tx_frame[3] = (uint8_t)(sc & 0xFFu);
    spi_dma.tx_frame[4] = (uint8_t)((sc >> 8u) & 0xFFu);
    spi_dma.tx_frame[5] = (uint8_t)((sc >> 16u) & 0xFFu);
    spi_dma.tx_frame[6] = (uint8_t)((sc >> 24u) & 0xFFu);
    spi_dma_arm_tx();
}

static void spi_load_frame(uint8_t command, uint8_t sequence, const uint8_t payload[SENSOR_FRAME_PAYLOAD_SIZE])
{
    uint32_t index;

    spi_dma.tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_dma.tx_frame[1] = command;
    spi_dma.tx_frame[2] = sequence;
    for (index = 0u; index < SENSOR_FRAME_PAYLOAD_SIZE; index++)
    {
        spi_dma.tx_frame[3u + index] = (payload != NULL) ? payload[index] : 0u;
    }
    spi_dma_arm_tx();
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
    uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
    uint8_t source_index = published_snapshot_index;

    /* INFO must identify the published generation immediately; copying its 252-byte
     * payload is deliberately deferred to the main loop so the DMA ISR stays bounded. */
    transfer_valid = published_snapshot_valid;
    transfer_generation = snapshot_generations[source_index];
    spi_dma.snapshot_source_index = source_index;
    spi_dma.snapshot_sequence = sequence;
    spi_dma.snapshot_copy_pending = true;
    Cy_SysLib_ExitCriticalSection(interrupt_state);

    payload[0] = (uint8_t)(transfer_generation & 0xFFu);
    payload[1] = (uint8_t)((transfer_generation >> 8u) & 0xFFu);
    payload[2] = transfer_valid ? 1u : 0u;
    payload[3] = SENSOR_CHANNEL_COUNT;
    spi_load_frame(SENSOR_CMD_SNAPSHOT_INFO, sequence, payload);
}

/* Complete the immutable snapshot latch outside the DMA ISR. Global IRQ masking keeps a
 * new BEGIN from replacing transfer metadata while its corresponding buffer is copied. */
static void spi_snapshot_latch_task(void)
{
    uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();

    if (spi_dma.snapshot_copy_pending)
    {
        uint8_t source_index = spi_dma.snapshot_source_index;
        if (transfer_valid)
        {
            memcpy(transfer_snapshot, snapshot_buffers[source_index], SENSOR_SNAPSHOT_SIZE);
        }
        else
        {
            memset(transfer_snapshot, 0, SENSOR_SNAPSHOT_SIZE);
        }
        spi_dma.snapshot_copy_pending = false;
    }
    Cy_SysLib_ExitCriticalSection(interrupt_state);
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

/* ---- JIT 算法引擎 SPI 命令处理（ISR 上下文，仅做快速缓冲写入/标志置位）---- */
static void spi_load_algo_response(uint8_t command, uint8_t b2, uint8_t b3, uint16_t len)
{
    spi_dma.tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_dma.tx_frame[1] = command;
    spi_dma.tx_frame[2] = b2;
    spi_dma.tx_frame[3] = b3;
    spi_dma.tx_frame[4] = (uint8_t)(len & 0xFFu);
    spi_dma.tx_frame[5] = (uint8_t)((len >> 8u) & 0xFFu);
    spi_dma.tx_frame[6] = 0u;
    spi_dma_arm_tx();
}

static void spi_load_algo_trace_response(uint8_t ch, uint8_t active, uint16_t report, uint8_t idx)
{
    spi_dma.tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_dma.tx_frame[1] = ALGO_GET_TRACE;
    spi_dma.tx_frame[2] = ch;
    spi_dma.tx_frame[3] = active;
    spi_dma.tx_frame[4] = (uint8_t)(report & 0xFFu);
    spi_dma.tx_frame[5] = (uint8_t)((report >> 8u) & 0xFFu);
    spi_dma.tx_frame[6] = idx;
    spi_dma_arm_tx();
}

static void cmd_algo_begin(uint8_t len_lo, uint8_t len_hi)
{
    uint16_t len = (uint16_t)len_lo | ((uint16_t)len_hi << 8u);
    if (len > ALGO_SLOT_SIZE) { len = ALGO_SLOT_SIZE; }
    algo_expected_len = len;
    algo_commit_pending = false;   /* 新一轮上传，废弃上一次未 commit 的请求 */
    spi_load_algo_response(ALGO_BEGIN, len_lo, len_hi, len);
}

static void cmd_algo_page(uint8_t page, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint32_t offset = (uint32_t)page * 4u;
    if ((offset + 4u) <= ALGO_SLOT_SIZE)
    {
        algo_staging[offset]      = d0;
        algo_staging[offset + 1u] = d1;
        algo_staging[offset + 2u] = d2;
        algo_staging[offset + 3u] = d3;
    }
    spi_load_algo_response(ALGO_PAGE, page, 0u, 0u);
}

static void cmd_algo_end(uint8_t crc_lo, uint8_t crc_hi)
{
    /* ISR 只置位 pending + 记录 CRC/len；真正的 CRC 校验与 memcpy 由主循环执行(见主循环)。
     * ok=1 表示"已接收，将校验"；实际校验结果由后续 ALGO_INFO 反映(§设计文档 4)。 */
    algo_commit_crc = (uint16_t)crc_lo | ((uint16_t)crc_hi << 8u);
    algo_commit_pending = true;
    spi_load_algo_response(ALGO_END, 1u, 0u, algo_expected_len);
}

static void cmd_algo_info(void)
{
    spi_load_algo_response(ALGO_INFO, algo_valid ? 1u : 0u, 0u, algo_len);
}

/* 设置每通道 16 位 ROM：帧 [magic,SET_ROM,ch,rom_lo,rom_hi,0,0]。回显 [.. ,ch,0,rom] 供校验。 */
static void cmd_algo_set_rom(uint8_t ch, uint8_t rom_lo, uint8_t rom_hi)
{
    uint16_t rom = (uint16_t)rom_lo | ((uint16_t)rom_hi << 8u);
    if (ch < SENSOR_CHANNEL_COUNT) { g_algo_rom[ch] = rom; }
    spi_load_algo_response(ALGO_SET_ROM, ch, 0u, rom);
}

/* 读每通道 16 位 ROM：帧 [magic,GET_ROM,ch,..]。响应 [.. ,ch,0,rom_lo,rom_hi,0]。 */
static void cmd_algo_get_rom(uint8_t ch)
{
    uint16_t rom = (ch < SENSOR_CHANNEL_COUNT) ? g_algo_rom[ch] : 0u;
    spi_load_algo_response(ALGO_GET_ROM, ch, 0u, rom);
}

/* 读某通道算法追踪: 帧 [magic,GET_TRACE,ch,idx,..]。响应 [.. ,ch,out_active,report[idx]_lo,report[idx]_hi,idx]。
 * 供上位机在单通道调整页可视化算法上报变量(report[])与触发判定(out_active)。 */
static void cmd_algo_get_trace(uint8_t ch, uint8_t idx)
{
    uint16_t rep = 0u;
    uint8_t  act = 0u;
    if ((ch < SENSOR_CHANNEL_COUNT) && (idx < 4u))
    {
        rep = g_algo_io[ch].report[idx];
        act = (g_algo_io[ch].out_active != 0u) ? 1u : 0u;
    }
    spi_load_algo_trace_response(ch, act, rep, idx);
}

/* 设共享算法可设置变量: 帧 [magic,SET_CFG,idx,val,..]。cfg[idx]=val, 下次算法执行拷入 io->cfg。 */
static void cmd_algo_set_cfg(uint8_t idx, uint8_t val)
{
    if (idx < 8u) { g_algo_cfg[idx] = val; }
    spi_load_algo_response(ALGO_SET_CFG, idx, 0u, (idx < 8u) ? g_algo_cfg[idx] : 0u);
}

/* 读共享算法可设置变量: 帧 [magic,GET_CFG,idx,..]。响应 [.. ,idx,0,cfg[idx],0,0]。 */
static void cmd_algo_get_cfg(uint8_t idx)
{
    uint16_t val = (idx < 8u) ? (uint16_t)g_algo_cfg[idx] : 0u;
    spi_load_algo_response(ALGO_GET_CFG, idx, 0u, val);
}

/* 装载本次响应帧到 TX DMA 并重挂通道（照搬官方 CE 的 TX 重挂序: 设源 → 设长 → 指定当前描述符
 * → 置有效 → 使能通道）。TX 描述符配 cpltState=true(完成即自失效), 故每帧必须显式重挂一次;
 * 这同时消除了"描述符完成后仍有效 → TX FIFO 被主机抽空使电平请求重新有效 → 同一帧被反复重发"
 * 的错位隐患。 */
static void spi_dma_arm_tx(void)
{
    /* 先清掉上一次响应在 FIFO 里的残留(流水线换帧), 再重挂描述符。 */
    Cy_SCB_SPI_ClearTxFifo(scb_0_HW);
    Cy_DMAC_Descriptor_SetSrcAddress(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PING, spi_dma.tx_frame);
    Cy_DMAC_Descriptor_SetDataCount(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                    CY_DMAC_DESCRIPTOR_PING, SENSOR_FRAME_SIZE);
    __DMB();
    Cy_DMAC_Channel_SetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING);
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PING, true);
    Cy_DMAC_Channel_Enable(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL);
}

static void spi_dma_init(void)
{
    static const cy_stc_sysint_t dma_int_cfg =
    {
        .intrSrc = cpuss_interrupt_dma_IRQn,
        .intrPriority = 2u,
    };
    const cy_stc_dmac_channel_config_t rx_channel_config =
    {
        .priority = 3u, .enable = false, .descriptor = CY_DMAC_DESCRIPTOR_PING,
    };
    const cy_stc_dmac_channel_config_t tx_channel_config =
    {
        .priority = 3u, .enable = false, .descriptor = CY_DMAC_DESCRIPTOR_PING,
    };
    const cy_stc_dmac_descriptor_config_t rx_descriptor_config =
    {
        .srcAddress = &scb_0_HW->RX_FIFO_RD, .dstAddress = spi_dma.rx_frame[0],
        .dataCount = SENSOR_FRAME_SIZE, .dataSize = CY_DMAC_BYTE,
        /* ★外设 FIFO 侧必须按字(32bit)访问★: RX_FIFO_RD 是外设寄存器, 只支持字访问;
         * 用 TRANSFER_SIZE_DATA(=dataSize=BYTE)去读它, DMAC 搬不出数据(实测 SPI 从机完全
         * 不应答: link_valid=false、generation=0)。故源(FIFO)= WORD、目的(内存)= DATA(字节)。
         * 这也是 dataSize=BYTE 与 transferSize 分开两个字段的用途所在。 */
        .srcTransferSize = CY_DMAC_TRANSFER_SIZE_WORD, .srcAddrIncrement = false,
        .dstTransferSize = CY_DMAC_TRANSFER_SIZE_DATA, .dstAddrIncrement = true,
        /* ★16CYC★: SCB 的 tr_rx_req/tr_tx_req 是 FIFO **电平**请求(非脉冲)。
         * cy_scb_spi.h 的 "DMA Trigger" 一节明确要求 DMA 描述符配置为 16 Clk_Slow 周期后重触发,
         * 以正确处理 SCB 电平请求的置起/撤销。触发连线本身由 cycfg_routing.c 的
         * Cy_TrigMux_Connect(TRIG0_IN_SCB0_TR_RX_REQ → TRIG0_OUT_CPUSS_DMAC_TR_IN0) 已生成。
         * ★retrigger/cpltState/flipping 一律照搬官方 CE(mtb-example-psoc4-uart-transmit-receive-dma
         * 的 RxDma_ping/pong_config): 4CYC 重触发、完成【不】失效、flipping=true 交替 PING/PONG。
         * cpltState=false + 双描述符 = 任何时刻都有有效描述符接住电平请求, 软件永不需要在 ISR 里
         * 重置正在服务的描述符(那正是此前 RX 只搬一帧就死的原因)。 */
        .retrigger = CY_DMAC_RETRIG_4CYC, .cpltState = false, .interrupt = true,
        .preemptable = true, .flipping = true, .triggerType = CY_DMAC_SINGLE_ELEMENT,
    };
    const cy_stc_dmac_descriptor_config_t tx_descriptor_config =
    {
        .srcAddress = spi_dma.tx_frame, .dstAddress = &scb_0_HW->TX_FIFO_WR,
        .dataCount = SENSOR_FRAME_SIZE, .dataSize = CY_DMAC_BYTE,
        /* 同 RX 反向: 源(内存)按字节, 目的(TX_FIFO_WR 外设寄存器)必须按字访问。 */
        .srcTransferSize = CY_DMAC_TRANSFER_SIZE_DATA, .srcAddrIncrement = true,
        .dstTransferSize = CY_DMAC_TRANSFER_SIZE_WORD, .dstAddrIncrement = false,
        /* ★照搬官方 CE 的 TxDma_ping_config★: 4CYC 重触发 + cpltState=true(完成即自失效) +
         * flipping=false。自失效是关键: 7 字节装完后描述符立即无效, 主机把 FIFO 抽空使 TX 电平
         * 请求重新有效时不会再次搬运同一帧(否则响应会被重复写入而错位)。每帧由 spi_dma_arm_tx
         * 显式重挂。 */
        .retrigger = CY_DMAC_RETRIG_4CYC, .cpltState = true, .interrupt = false,
        .preemptable = true, .flipping = false, .triggerType = CY_DMAC_SINGLE_ELEMENT,
    };

    /* RX 用 PING/PONG 两个描述符交替落帧: 结构相同, 只有目的缓冲不同。 */
    (void)Cy_DMAC_Descriptor_Init(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                  CY_DMAC_DESCRIPTOR_PING, &rx_descriptor_config);
    (void)Cy_DMAC_Descriptor_Init(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                  CY_DMAC_DESCRIPTOR_PONG, &rx_descriptor_config);
    Cy_DMAC_Descriptor_SetDstAddress(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PONG, spi_dma.rx_frame[1]);
    (void)Cy_DMAC_Descriptor_Init(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                  CY_DMAC_DESCRIPTOR_PING, &tx_descriptor_config);
    (void)Cy_DMAC_Channel_Init(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL, &rx_channel_config);
    (void)Cy_DMAC_Channel_Init(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL, &tx_channel_config);
    /* 两个 RX 描述符同时置有效(官方范式), 并把 PING 设为当前描述符。 */
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PING, true);
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PONG, true);
    Cy_DMAC_Channel_SetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING);
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PING, false);
    Cy_DMAC_Channel_SetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING);
    Cy_DMAC_ClearInterrupt(DMAC, CY_DMAC_INTR_CHAN_0 | CY_DMAC_INTR_CHAN_1);
    Cy_DMAC_SetInterruptMask(DMAC, CY_DMAC_INTR_CHAN_0);
    Cy_SysInt_Init(&dma_int_cfg, spi_dma_isr);
    NVIC_ClearPendingIRQ(cpuss_interrupt_dma_IRQn);
    NVIC_EnableIRQ(cpuss_interrupt_dma_IRQn);
    Cy_SCB_SetRxFifoLevel(scb_0_HW, 0u);
    Cy_SCB_SetTxFifoLevel(scb_0_HW, SENSOR_FRAME_SIZE);

    /* CS 上升沿(取消选择)中断: 帧边界重同步。优先级必须【低于】DMA 完成中断(2), 否则 CS 沿可能
     * 抢在 RX 完成 ISR 之前把刚收满的帧判成半帧丢掉。 */
    {
        static const cy_stc_sysint_t cs_int_cfg =
        {
            .intrSrc = SPI_CS_IRQ,
            .intrPriority = 3u,
        };
        /* cat2 无逐引脚中断屏蔽 API: 配置边沿(INTR_CFG.EDGE_SEL)即使能该引脚中断。 */
        Cy_GPIO_SetInterruptEdge(SPI_CS_PORT, SPI_CS_NUM, CY_GPIO_INTR_RISING);
        Cy_GPIO_ClearInterrupt(SPI_CS_PORT, SPI_CS_NUM);
        Cy_SysInt_Init(&cs_int_cfg, spi_cs_isr);
        NVIC_ClearPendingIRQ(SPI_CS_IRQ);
        NVIC_EnableIRQ(SPI_CS_IRQ);
    }

    Cy_DMAC_Enable(DMAC);
    Cy_DMAC_Channel_Enable(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL);
    /* TX 通道不在此使能: 描述符尚未装载(SetState false), 由首次 spi_load_touch → spi_dma_arm_tx
     * 装帧后使能(与官方 CE 一致: configure_tx_dma 只配不使能)。 */
}

static void spi_slave_init(void)
{
    Cy_SCB_SPI_Init(scb_0_HW, &scb_0_config, &spi_context);
    Cy_SCB_SPI_Enable(scb_0_HW);
    Cy_SCB_SPI_ClearRxFifo(scb_0_HW);
    Cy_SCB_SPI_ClearTxFifo(scb_0_HW);
    update_touch_frame();
    spi_dma_init();
    spi_load_touch();   /* 默认响应=实时触控帧（DMA 流水线快路） */
}

static void spi_dma_isr(void)
{
    uint32_t interrupt = Cy_DMAC_GetInterruptStatusMasked(DMAC);

    if ((interrupt & CY_DMAC_INTR_CHAN_0) != 0u)
    {
        /* flipping=true: 完成时通道的当前描述符已翻到【下一个】, 故刚落帧的是它的反面
         * (照搬官方 CE 的 Isr_DMA 判定)。两个描述符都保持有效, 此处【绝不】调 SetState —— 那会
         * 清掉正在服务中的描述符的传输索引。 */
        cy_en_dmac_descriptor_t current =
            Cy_DMAC_Channel_GetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL);
        uint8_t completed = (current == CY_DMAC_DESCRIPTOR_PING) ? 1u : 0u;

        Cy_DMAC_ClearInterrupt(DMAC, CY_DMAC_INTR_CHAN_0);
        spi_dma.frame_received = true;
        /* (rx_frames 计数已退役: 链路已修好, 存活由 scan_count_m/ms_tick_m 监视) */
        spi_slave_task(completed);
    }
}

/* ★CS 抬起帧重同步★: SCB 从机没有"被取消选择"的事件(只有 SLAVE_ERR=错时机取消), 而 RX DMA 是
 * 按字节流连续搬运的 —— 一旦某次事务的字节数不等于 7(PSoC 复位期间主机仍在发、RP2040 侧
 * _recover() 清 FIFO、任何一次时序抖动), 7 字节帧边界就会永久错位, 之后每帧 magic 都不在 rx[0],
 * 表现为 RP2040 恒读回非法帧。故用 CS(P1.3) 上升沿(取消选择)做唯一权威的帧边界: 事务结束时若
 * 描述符里留着半帧(或 RX FIFO 有残字节), 就丢弃这些残余并把描述符索引复位, 使下一次事务必然从
 * 帧首开始。完整帧(索引已归零)不受影响, 零副作用。 */
static void spi_cs_isr(void)
{
    const uint32_t intr = Cy_GPIO_GetInterruptStatus(SPI_CS_PORT, SPI_CS_NUM);

    Cy_GPIO_ClearInterrupt(SPI_CS_PORT, SPI_CS_NUM);
    if (intr == 0u) { return; }

    /* RX FIFO 残字节: 上一事务未凑满一个 DMA 元素窗口的剩余, 必须丢掉。 */
    if (Cy_SCB_GetNumInRxFifo(scb_0_HW) != 0u)
    {
        spi_dbg.rx_leftover++;
        Cy_SCB_SPI_ClearRxFifo(scb_0_HW);
    }

    /* 描述符里留着半帧 ⇒ 帧边界已错位, 复位两个描述符的传输索引重新对齐帧首。
     * SetState(true) 会同时清 CURR_DATA_NR 与 RESPONSE(见 cy_dmac.h), 正是所需语义。 */
    if ((Cy_DMAC_Descriptor_GetCurrentIndex(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                            CY_DMAC_DESCRIPTOR_PING) != 0u) ||
        (Cy_DMAC_Descriptor_GetCurrentIndex(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                            CY_DMAC_DESCRIPTOR_PONG) != 0u))
    {
        spi_dbg.cs_resync++;
        Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                    CY_DMAC_DESCRIPTOR_PING, true);
        Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                    CY_DMAC_DESCRIPTOR_PONG, true);
        Cy_DMAC_Channel_SetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                             CY_DMAC_DESCRIPTOR_PING);
        Cy_DMAC_Channel_Enable(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL);
    }
}

static void spi_slave_task(uint8_t rx_index)
{
    const volatile uint8_t *rx = spi_dma.rx_frame[rx_index & 1u];

    if (rx[0] != SENSOR_FRAME_MAGIC)
    {
        spi_dbg.rx_bad_magic++;
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
            spi_dbg.setparam_cmd++;
            uint32_t val = (uint32_t)rx[4] | ((uint32_t)rx[5] << 8u) | ((uint32_t)rx[6] << 16u);
            (void)cmd_set_param(rx[2], rx[3], val);
            // 回显【实际当前值】(非法被拒时=旧值), 供上位机据实回读, 天然拦截非法调参。
            spi_load_cmd_response(SENSOR_CMD_SET_PARAM, rx[2], rx[3], cmd_get_param(rx[2], rx[3]));
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

        case SENSOR_CMD_SET_GLOBAL: {
            uint32_t val = (uint32_t)rx[4] | ((uint32_t)rx[5] << 8u) | ((uint32_t)rx[6] << 16u);
            cmd_set_global(rx[2], val);
            /* ★回显存储值, 不回显请求值★: cmd_set_global 对非法值是"不写、保持原值", 若回显请求值,
             * 上位机拿到的就是自己刚发的数, 无法分辨"写进去了"还是"被拒了"(实测 MFS 被静默截断时
             * 界面显示正常)。回读存储值后, 上位机的回读对账就能自己发现不一致。 */
            spi_load_cmd_response(SENSOR_CMD_SET_GLOBAL, rx[2], 0u, cmd_get_global(rx[2]));
            break;
        }

        case SENSOR_CMD_GET_GLOBAL:
            spi_load_cmd_response(SENSOR_CMD_GET_GLOBAL, rx[2], 0u, cmd_get_global(rx[2]));
            break;

        case SENSOR_CMD_GLOBAL_COMMIT:
            // 全部全局项已设入影子 → 主循环执行【一次】完整重初始化生效。
            global_apply_pending = true;
            g_op_busy = 1u;   // 处理中锁定: 主循环完成完整重初始化后清 0
            spi_load_cmd_response(SENSOR_CMD_GLOBAL_COMMIT, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_MEASURE_CP:
            // 耗时测量由主循环执行；ACK 仅表示命令已由 SPI ISR 实际接收并置为 pending。
            // busy 覆盖 BIST 与其后的正常 CSD 恢复，RP 只会在恢复完成后继续任何重操作。
            measure_cp_pending = true;
            g_op_busy = 1u;
            spi_load_cmd_response(SENSOR_CMD_MEASURE_CP, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_GET_CP:
            // pending/active 时统一返回 0，防止 ISR 观察到主循环逐通道更新的半成品。
            spi_load_cmd_response(SENSOR_CMD_GET_CP, rx[2], 0u,
                                  (rx[2] >= SENSOR_CHANNEL_COUNT) ? 0xFFFFFFu :
                                  ((measure_cp_pending || measure_cp_active) ? 0u : cp_value[rx[2]]));
            break;

        case SENSOR_CMD_APPLY:
            // ★不能在 ISR 里做重校准(耗时数ms/需扫描完成)★：仅置标志，主循环执行。
            // 启动 gate 只承认已经收到全 36 个 enabled 值之后的 APPLY；该 APPLY 完成前不扫描。
            if (g_provision_pending && (g_provision_enable_seen == CH_ENABLED_ALL))
            {
                g_provision_apply_release = true;
            }
            apply_pending = true;
            g_op_busy = 1u;   // 处理中锁定
            spi_load_cmd_response(SENSOR_CMD_APPLY, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_QUICK_APPLY: {
            /* Sweep 帧 [magic, QUICK_APPLY, ch, gain, div, 0, 0]：ISR 只接收完整合法参数并置 pending，
             * 绝不触碰 widgetContext（当前 CapSense 扫描期间写它会卡死该轮，主循环到不了 pending）。
             * 非法帧不置 pending/busy；ACK 的 b3=0xFF、value=0 明确回显拒绝，RP 最终仍以 readback
             * 为设备真值。 */
            const bool valid = (rx[2] < SENSOR_CHANNEL_COUNT) && (rx[3] <= 6u) &&
                               (rx[4] >= 1u) && (rx[4] <= 64u);
            if (valid)
            {
                quick_apply_ch = rx[2];
                quick_apply_gain = rx[3];
                quick_apply_div = rx[4];
                quick_apply_pending = true;
                g_op_busy = 1u;
            }
            spi_load_cmd_response(SENSOR_CMD_QUICK_APPLY, rx[2],
                                  valid ? rx[3] : 0xFFu, valid ? rx[4] : 0u);
            break;
        }

        case SENSOR_CMD_FOCUS_SCAN: {
            /* Focus 帧 [magic, FOCUS_SCAN, ch, 0, 0, 0, 0]：ch=0..35 启用/续租单通道, 0xFF 立即恢复全通道。
             * 响应 [magic, FOCUS_SCAN, 实际目标 ch, 接受状态(1=OK / 0=拒绝), 0, 0, 0]。
             * 拒绝原因：ch 非法、ch 已禁用(电极必须 High-Z, 不可扫)。主循环据租约/启用态自行回退。 */
            const uint8_t req = rx[2];
            const bool is_exit = (req == SENSOR_CH_ALL);
            const bool valid = is_exit || (req < SENSOR_CHANNEL_COUNT && _ch_is_enabled(req));
            if (valid) {
                uint32_t st = Cy_SysLib_EnterCriticalSection();
                focus_scan_ch = req;
                focus_scan_until_ms = is_exit ? 0u : (g_ms_tick + FOCUS_SCAN_LEASE_MS);
                Cy_SysLib_ExitCriticalSection(st);
            }
            spi_load_cmd_response(SENSOR_CMD_FOCUS_SCAN, valid ? req : SENSOR_CH_ALL,
                                  valid ? 1u : 0u, 0u);
            break;
        }

        case SENSOR_CMD_CALIBRATE:
            // 真正的 IDAC 重校准 + 基线复位; 耗时, 仅置标志由主循环执行。
            // rx[2]=目标通道(0..35 单通道 / 0xFF 全通道), 与 AUTO_TUNE 同构; 非法值退化为全通道。
            // ★已 pending 时以最后一条为准★: RP2040 侧 heavy_gate 已拦住堆叠, host 侧又是串行队列,
            //   故不会出现"两条不同通道的请求同时在队"; 真出现也只是最后一条生效, 不会做出错通道。
            calibrate_ch = (rx[2] < SENSOR_CHANNEL_COUNT) ? rx[2] : SENSOR_CH_ALL;
            calibrate_pending = true;
            g_op_busy = 1u;   // 处理中锁定
            spi_load_cmd_response(SENSOR_CMD_CALIBRATE, calibrate_ch, 0u, 0u);
            break;

        case SENSOR_CMD_BASELINE_RESET:
            // 基线复位; rx[2]=目标通道(0..35 单通道 / 0xFF 全通道)。单通道时主循环只初始化该 widget。
            baseline_ch = (rx[2] < SENSOR_CHANNEL_COUNT) ? rx[2] : SENSOR_CH_ALL;
            baseline_reset_pending = true;
            g_op_busy = 1u;   // 处理中锁定
            spi_load_cmd_response(SENSOR_CMD_BASELINE_RESET, baseline_ch, 0u, 0u);
            break;

        case SENSOR_CMD_AUTO_TUNE:
            // 频率自适应下探(耗时: 粗定位+细搜+落档重校准); 仅置标志由主循环执行, 结果经 GET_AUTO_TUNE 读。
            // rx[2]=目标通道(0..35 单通道 / 0xFF 全通道); 非法值退化为全通道。
            // rx[3]=灵敏度偏好档位(1..7); 非法/缺省(0)退化为 4(居中)。
            // rx[4]=本轮请求标签(6 bit, 0=未标记): 原样存下并由 GET_AUTO_TUNE 回显, 供 RP2040 认出
            //       "读到的是上一轮的残留结果"(见 auto_tune_tag 注释)。
            auto_tune_ch      = (rx[2] < SENSOR_CHANNEL_COUNT) ? rx[2] : AUTO_TUNE_CH_ALL;
            auto_tune_pref    = ((rx[3] >= 1u) && (rx[3] <= 7u)) ? rx[3] : 4u;
            auto_tune_tag     = (uint8_t)(rx[4] & AUTO_TUNE_TAG_MASK);
            auto_tune_pending = true;
            auto_tune_result  = 0u;   // 进行中
            auto_tune_phase   = AUTO_TUNE_PHASE_IDLE;   // 已受理, 主循环下一轮开始推进阶段
            auto_tune_step    = 0u;
            auto_tune_div     = 0u;
            g_op_busy = 1u;           // 处理中锁定(host 轮询 busy 至真实完成)
            // 受理回显里也带上标签(byte3): RP2040 的 _send_heavy 收割到本帧即可确认"这一条被收下了"。
            spi_load_cmd_response(SENSOR_CMD_AUTO_TUNE, auto_tune_ch, auto_tune_tag, 0u);
            break;

        case SENSOR_CMD_GET_AUTO_TUNE:
            // 读结果/进度: [result|tag<<2, ch, div_lo, div_hi, progress]; progress = phase | (step << 3)。
            // result 只取 0/1/2 ⇒ 高 6 位承载本轮请求标签(旧 RP 固件不带标签时回显 0, 行为不变)。
            spi_load_cmd_response(SENSOR_CMD_GET_AUTO_TUNE,
                                  (uint8_t)((auto_tune_result & AUTO_TUNE_RESULT_MASK) |
                                            ((auto_tune_tag & AUTO_TUNE_TAG_MASK) << 2u)),
                                  auto_tune_ch,
                                  (uint32_t)auto_tune_div |
                                  (((uint32_t)(auto_tune_phase & 0x07u) |
                                    ((uint32_t)(auto_tune_step & 0x1Fu) << 3u)) << 16u));
            break;

        case SENSOR_CMD_SET_MODE:
            // rx[2]: 0=自动校准/标准完整处理，非0=半自动手动。主循环据此选处理链。
            scan_mode = (rx[2] != 0u) ? SCAN_MODE_SEMI : SCAN_MODE_AUTO;
            spi_load_cmd_response(SENSOR_CMD_SET_MODE, scan_mode, 0u, 0u);
            break;

        case ALGO_BEGIN:
            cmd_algo_begin(rx[2], rx[3]);
            break;

        case ALGO_PAGE:
            cmd_algo_page(rx[2], rx[3], rx[4], rx[5], rx[6]);
            break;

        case ALGO_END:
            cmd_algo_end(rx[2], rx[3]);
            break;

        case ALGO_INFO:
            cmd_algo_info();
            break;

        case ALGO_SET_ROM:
            cmd_algo_set_rom(rx[2], rx[3], rx[4]);
            break;

        case ALGO_GET_ROM:
            cmd_algo_get_rom(rx[2]);
            break;

        case ALGO_GET_TRACE:
            cmd_algo_get_trace(rx[2], rx[3]);
            break;

        case ALGO_SET_CFG:
            cmd_algo_set_cfg(rx[2], rx[3]);
            break;

        case ALGO_GET_CFG:
            cmd_algo_get_cfg(rx[2]);
            break;

        case SENSOR_CMD_TOUCH:
        default:
            /* 触控快路（流水线）：装入最新触控帧，供下一次读取立即返回。 */
            spi_load_touch();
            break;
    }
}

/* 自适应原子动作：把 snsClk 分频写进目标 widget 并就地校准一次, 返回该分频下 IDAC 能否把 raw
 * 校到目标容差内。粗定位/细搜/落档/回退四处复用, 避免复制粘贴。
 * ★恒为单通道判定★: 全通道模式也由外层逐通道调用本函数(不再有"写全部 widget 同一分频"的路径)。
 * ★IDAC 档口径★: 经 _calibrate_widget_locked 校准, 故"该分频能否压到目标%"是在该通道自己的
 * 增益档(用户设过 PARAM_IDAC_GAIN 时)下判定的, 而不是全局起点档。 */
static inline bool auto_tune_try_div(uint8_t target_ch, uint16_t div)
{
    if (target_ch >= SENSOR_CHANNEL_COUNT) return false;
    cy_capsense_tuner.widgetContext[target_ch].snsClk = div;
    return _calibrate_widget_locked((uint32_t)target_ch);
}

/* 自适应"打点 + 试探"：先把阶段/步序/当前试探分频发布(SPI ISR 经 GET_AUTO_TUNE 上报, 长过程可见),
 * 再执行一次试探校准。粗定位/细搜/落档/回退四处共用, 避免每处复制打点代码。 */
static inline bool auto_tune_probe(uint8_t target_ch, uint16_t div, uint8_t phase, uint32_t step)
{
    auto_tune_phase = phase;
    auto_tune_step  = (step > AUTO_TUNE_STEP_MAX_REPORT) ? (uint8_t)AUTO_TUNE_STEP_MAX_REPORT : (uint8_t)step;
    auto_tune_div   = div;   /* 进行中语义: div 字段 = 当前试探值(完成时被最终分频覆盖) */
    return auto_tune_try_div(target_ch, div);
}

/* 单通道完整三步自适应: ①粗表定位 ②1 步进上探临界 ③按 pref 落档(失败逐 1 回退)。
 * 成功时 *out_div = 最终写入该 widget 的分频。全通道模式由外层逐通道复用本函数, 使每个通道各自
 * 落在其自身临界频率上(面板 Cp 22~138pF, 单一统一分频必然被最高 Cp 的通道拖垮)。 */
static bool auto_tune_run_ch(uint8_t target_ch, uint8_t pref, uint16_t* out_div)
{
    static const uint16_t k_autotune_divs[] = { 8u, 12u, 16u, 20u, 24u, 32u, 40u, 48u, 64u };
    const uint32_t div_count = sizeof(k_autotune_divs) / sizeof(k_autotune_divs[0]);
    uint16_t saved_div = cy_capsense_tuner.widgetContext[target_ch].snsClk;
    uint16_t div_edge = 0u;   /* 临界分频(最高频率, 校准刚好通过) */
    uint16_t div_target;
    uint32_t di;
    uint32_t step;

    /* ① 粗定位: 粗表升序(升分频=降频)扫描, 首个校准成功的档位必然 >= 真实临界值。 */
    for (di = 0u; di < div_count; di++)
    {
        if (auto_tune_probe(target_ch, k_autotune_divs[di], AUTO_TUNE_PHASE_COARSE, di + 1u))
        {
            div_edge = k_autotune_divs[di];
            break;
        }
    }
    if (div_edge == 0u)
    {
        /* 粗定位全失败时必须恢复原分频并尝试校准，否则最后试探值会残留为坏状态。 */
        cy_capsense_tuner.widgetContext[target_ch].snsClk = saved_div;
        (void)_calibrate_widget_locked(target_ch);
        return false;
    }

    /* ② 1 步进上探临界: 从 div_edge-1 起逐 1 降分频(升频)重校准, 成功即继续;
     *    首次失败即停, 最后一个成功值就是临界分频。 */
    for (step = 0u; step < AUTO_TUNE_FINE_STEPS_MAX; step++)
    {
        if (div_edge <= 1u) break;
        if (!auto_tune_probe(target_ch, (uint16_t)(div_edge - 1u),
                             AUTO_TUNE_PHASE_FINE, step + 1u)) break;
        div_edge = (uint16_t)(div_edge - 1u);
    }

    /* ③ 按偏好落档: 每档往低频让 2 个分频(充电更充分→过充近场效应→更灵敏)。 */
    div_target = (uint16_t)(div_edge + 2u * (uint16_t)(pref - 1u));
    if (div_target > 255u) div_target = 255u;   /* PSoC snsClk 合法范围 1..255 */
    step = 1u;
    while (!auto_tune_probe(target_ch, div_target, AUTO_TUNE_PHASE_SETTLE, step))
    {
        step++;
        /* 落档失败(该低频下 IDAC 反而压不住): 朝临界方向逐 1 回退重试;
         * 退到临界仍失败则在临界档重校准一次(已知可用)并采用它。 */
        if (div_target <= div_edge)
        {
            (void)auto_tune_probe(target_ch, div_edge, AUTO_TUNE_PHASE_SETTLE, step);
            div_target = div_edge;
            break;
        }
        div_target = (uint16_t)(div_target - 1u);
    }
    if (out_div != NULL) *out_div = div_target;
    return true;
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
    memset(&spi_dma, 0, sizeof(spi_dma));
    _spi_dbg_clear();
    for (uint32_t channel = 0u; channel < SENSOR_CHANNEL_COUNT; channel++)
    {
        cp_value[channel] = 0xFFFFFFu;
    }
    measure_cp_pending = false;
    measure_cp_active = false;
    global_apply_pending = false;
    quick_apply_pending = false;
    quick_apply_ch = SENSOR_CH_ALL;
    quick_apply_gain = 0u;
    quick_apply_div = 1u;
    calibrate_pending = false;
    baseline_reset_pending = false;
    calibrate_ch = SENSOR_CH_ALL;
    baseline_ch = SENSOR_CH_ALL;
    auto_tune_pending = false;
    auto_tune_result = 0u;
    auto_tune_div = 0u;
    auto_tune_ch = AUTO_TUNE_CH_ALL;
    auto_tune_phase = AUTO_TUNE_PHASE_IDLE;
    auto_tune_step = 0u;
    auto_tune_tag = 0u;   /* 复位后没有任何在途请求: 标签清零 = "未标记", 不冒充上一轮 */
    /* PSoC 无状态: 上电先禁用全部 widget。新 RP 会完整下发 enabled 位图并以 APPLY 放行；
     * 未升级 RP 在 3 秒后走兼容全启用兜底。 */
    g_ch_enabled = 0u;
    g_provision_enable_seen = 0u;
    g_provision_pending = true;
    g_provision_apply_release = false;
    ch_enable_dirty = 0u;
    g_any_active = false;
    published_snapshot_index = 0u;
    published_snapshot_valid = false;
    transfer_generation = 0u;
    transfer_valid = false;
    memset(g_algo_io, 0, sizeof(g_algo_io));
    memset(algo_prev_active, 0, sizeof(algo_prev_active));
    for (uint32_t channel = 0u; channel < SENSOR_CHANNEL_COUNT; channel++) { g_algo_rom[channel] = 0u; }
    for (uint32_t k = 0u; k < 8u; k++) { g_algo_cfg[k] = 0u; }
    algo_valid = false;
    algo_len = 0u;
    algo_expected_len = 0u;
    algo_commit_pending = false;

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
    // hardware.txt: P1.6 -> LED -> R -> GND，因此高电平明确为点亮。
    // 白灯启动即点亮作为 bring-up 指示；800ms 后仅由已加载 JIT 算法的判定结果驱动，
    // 原始 CapSense 触控不会触发白灯，避免把非 JIT 模式误呈现为算法已触发。
    Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM, STATUS_LED_ON_STATE);
    /* ★可见启动指示(修"重启白灯不亮")★: 白灯在启动后保持点亮 800ms 再交给 g_any_active 驱动,
     * 使每次上电/XRES 重启都有肉眼可见的白灯闪亮(否则仅亮几微秒无法察觉, 用户误判"重启无效")。 */
    uint32_t led_boot_until_ms = g_ms_tick + 800u;

    for (;;)
    {
        spi_dbg.stage = MLOOP_STAGE_LATCH;
        /* Snapshot payload copies are intentionally outside the DMA completion ISR. */
        spi_snapshot_latch_task();
        spi_dbg.stage = MLOOP_STAGE_WAIT_SCAN;
        spi_dbg.ms_tick_m = g_ms_tick;   /* 无条件刷新: 卡在等扫描时也能看出时间在走 */
        spi_dbg.clk_now = cy_capsense_tuner.widgetContext[0].snsClk;   /* 当前生效分频 */
        /* ★兼容回退只针对"从头到尾没送过位图"的旧 RP★
         * 原判据只看时间: 只要 3 秒内没走完 provisioning 就把 36 通道全部打开 —— 而新 RP 送完
         * 36 条 PARAM_ENABLED 之后还要送约 396 条 PARAMS 才发放行 APPLY, 那段时间必然超过 3 秒,
         * 于是刚刚送到的启用位图被这条回退整片冲成"全启用", 用户关掉的通道每次复位后偷偷复活。
         * 判据改为"一条都没收到": 只要 RP 已经开始送位图, 它就是真相源, 等它送完; RP 侧 ENABLED
         * 阶段只在成功时推进、失败无限重试, 链路真断则 RP 换代次后从 MODE 重新下发, 不会永久悬空。 */
        if (g_provision_pending && (g_ms_tick >= PROVISION_TIMEOUT_MS) &&
            (g_provision_enable_seen == 0u))
        {
            g_ch_enabled = CH_ENABLED_ALL;
            ch_enable_dirty = CH_ENABLED_ALL;
            g_provision_pending = false;
        }
        /* SPI frames are moved by DMAC; the completion ISR only prepares the next response. */
        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            spi_dbg.stage = MLOOP_STAGE_PROCESS;
            /* ★Focus 单通道处理★ 仅复核通过(启用且租约未过期)时只处理该 widget；
             * 全通道哨兵回落正常全通道路径。租约到期由 _focus_scan_target() 强制回退。 */
            const uint8_t focus_target = _focus_scan_target();
            if (focus_target < SENSOR_CHANNEL_COUNT)
            {
                /* Focus 单通道：只处理该 widget。scan_mode AUTO/SEMI 处理链已覆盖全部字段(滤波/基线/
                 * 差值/状态/噪声/阈值)；无需为 Focus 另造分支——直接复用既有完整链即可。 */
                if (scan_mode == SCAN_MODE_AUTO)
                {
                    (void)Cy_CapSense_ProcessWidget(focus_target, &cy_capsense_context);
                }
                else
                {
                    const uint32_t manual_mask = CY_CAPSENSE_PROCESS_FILTER |
                                                 CY_CAPSENSE_PROCESS_BASELINE |
                                                 CY_CAPSENSE_PROCESS_DIFFCOUNTS |
                                                 CY_CAPSENSE_PROCESS_STATUS;
                    (void)Cy_CapSense_ProcessWidgetExt(focus_target, manual_mask, &cy_capsense_context);
                }
            }
            else
            {
                /* 全通道处理(正常路径 / Focus 租约到期回退 / 全通道禁用) */
                if (scan_mode == SCAN_MODE_AUTO)
                {
                    /* 自动校准：运行中间件标准完整处理链。 */
                    (void)Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
                }
                else
                {
                    /* 半自动手动：跳过 CALC_NOISE+THRESHOLDS，手动 SET_PARAM 阈值不被覆盖，
                     * 仍跑滤波/基线/差值/状态，触控检测正常（用手动 fingerTh 判定）。 */
                    const uint32_t manual_mask = CY_CAPSENSE_PROCESS_FILTER |
                                                 CY_CAPSENSE_PROCESS_BASELINE |
                                                 CY_CAPSENSE_PROCESS_DIFFCOUNTS |
                                                 CY_CAPSENSE_PROCESS_STATUS;
                    /* ★必须自己跳过禁用通道★ Cy_CapSense_ProcessWidgetExt 按文档明确"忽略 widget 的
                     * disable/non-working 状态"(cy_capsense_structure.c:850 附近的说明), 与
                     * ProcessAllWidgets(control.c:589 会查 IsWidgetEnabled)不同 —— 不自己跳的话,
                     * 半自动模式下禁用通道仍会跑滤波/基线/状态判定, 甚至靠陈旧 raw 判出"按下"。 */
                    for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                    {
                        if (!_ch_is_enabled(w)) { continue; }
                        (void)Cy_CapSense_ProcessWidgetExt(w, manual_mask, &cy_capsense_context);
                    }
                }
            }
            spi_dbg.stage = MLOOP_STAGE_TOUCH;
            update_touch_frame();       /* 刷新实时触控帧（快路数据源） */
            /* 白 LED: 启动 800ms 内常亮(可见启动指示)；之后只有算法显式写 out_led 才亮。
             * (DMA 攻坚期这里曾并入 spi_dma.frame_received 作带外探针, 链路已修复, 探针已回退。) */
            Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM,
                          (g_any_active || (g_ms_tick < led_boot_until_ms))
                              ? STATUS_LED_ON_STATE : STATUS_LED_OFF_STATE);
            spi_dbg.stage = MLOOP_STAGE_PUBLISH;
            publish_capsense_snapshot();/* 刷新完整 raw/baseline/diff（调参慢路数据源） */

            /* MEASURE_CP：逐电极 BIST 寄生电容测量(fF)。测量会重配 CSD HW，完成后恢复扫描配置。 */
            {
                bool run_measure = false;
                uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
                if (measure_cp_pending)
                {
                    measure_cp_pending = false;
                    measure_cp_active = true;
                    run_measure = true;
                }
                Cy_SysLib_ExitCriticalSection(interrupt_state);

                if (run_measure)
                {
                    spi_dbg.stage = MLOOP_STAGE_MEASURE_CP;
                    /* 先统一标为失败；BIST 可用的 Cp 结果随后覆盖。active 期间 SPI GET_CP 始终返回 0。 */
                    for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                    {
                        cp_value[w] = 0xFFFFFFu;
                    }
#if (defined(CY_CAPSENSE_TST_SNS_CAP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN))
                    /* ★整段 BIST 期间必须屏蔽 CSD 中断★
                     * BIST 是**轮询式**的: Cy_CapSense_BistWaitEndOfScan(cy_capsense_selftest_v2.c:4489)
                     * 死等 ptrCsdBase->INTR 的 SAMPLE 位。而本工程把 CSD 中断挂到了 NVIC
                     * (capsense_isr → Cy_CapSense_InterruptHandler), 转换一结束 ISR 先跑, 顺手把 INTR
                     * 清掉 —— 轮询循环就永远看不到那一位, 耗尽 watchdog 计数后返回 TIMEOUT。
                     * 这是一场竞态: ISR 抢先与否取决于中断延迟(SPI DMA 中断正忙时更容易抢先), 所以
                     * 表现为"每次测电容都有几个通道随机失败, 失败集合每次都不一样"(实测三轮分别是
                     * {15,20,26,31} / {7,14,15,26,34} / {1,3,5,11,26,28,30}) —— 与电极本身无关,
                     * 重测也救不回来(重测只是再赌一次同一场竞态)。
                     * 顺带还有第二重危害: ISR 里跑的是**正常扫描**的后处理, 它会把 BIST 的转换结果
                     * 当成 raw 写进 sns context, 污染基线与差值。
                     * 故这里整段屏蔽; 恢复扫描【之前】必须重新打开(下面的重扫要靠它推进扫描链)。 */
                    NVIC_DisableIRQ(CYBSP_CSD_IRQ);
                    for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                    {
                        uint32_t v = 0u;
                        cy_en_capsense_bist_status_t status;
                        /* BIST 测量会把该电极接到测量回路上 —— 对已禁用(要求恒高阻)的通道不能做,
                         * 其 cp_value 保持 0xFFFFFF"未测量"。 */
                        if (!_ch_is_enabled(w)) { continue; }
                        status =
                            Cy_CapSense_MeasureCapacitanceSensor(w, 0u, &v, &cy_capsense_context);

                        /* ★不能把 LOW_LIMIT / HIGH_LIMIT 当成故障★
                         * 中间件的这两个"故障"判据是**它自己那把尺子的量程边界**, 不是电极的物理
                         * 结论(cy_capsense_selftest_v2.c:2629 BistMeasureCapacitanceSensor):
                         *   - 合法 raw 窗口只有满量程的 7.5%~45%(MIN/MAX_RAW_PROMILLE = 75/450);
                         *   - 而它只有 4 个测点(gain/code = 300k×20, 300k×80, 2400k×40, 4800k×80),
                         *     相邻测点之间是 4 倍跳变, 单个测点能覆盖的 Cp 只有 6 倍。
                         *   ⇒ 4 倍步长 > 6 倍窗口的余量很薄, 落在两个测点缝隙附近的 Cp 无论走哪一档
                         *     都贴着窗口边, 于是同一块板连测多轮, "失败"的通道集合每轮都不一样, 而它
                         *     们的 Cp 与相邻通道毫无区别(实测 CH3=69pF 报失败、CH17=143pF 却成功)。
                         * 关键事实: LOW_LIMIT / HIGH_LIMIT 这两条路上 **Cp 已经算出来并写回**了
                         * (同文件 2722 行: 只有 TIMEOUT 才不写值), 只是 raw 不在它偏爱的窗口内、精度
                         * 略差。把它判成"测量失败"是拿量程当故障, 属于假故障。
                         * ★真正的故障判据★ 只保留物理上说不通的两种:
                         *   - 没有值写回(TIMEOUT / HW_BUSY / BAD_PARAM) ⇒ v 保持 0;
                         *   - 值被钳在 CY_CAPSENSE_BIST_CP_MAX_VALUE(400pF) ⇒ 溢出/短路。
                         * 0xFFFFFF 仍保留给协议的失败标记, 故撞上该值的真实读数饱和到 0xFFFFFE。 */
                        if ((v != 0u) &&
                            ((CY_CAPSENSE_BIST_SUCCESS_E == status) ||
                             (CY_CAPSENSE_BIST_LOW_LIMIT_E == status) ||
                             (CY_CAPSENSE_BIST_HIGH_LIMIT_E == status)) &&
                            (v < CP_BIST_OVERRANGE_FF))
                        {
                            cp_value[w] = (v >= 0xFFFFFFu) ? 0xFFFFFEu : v;
                        }
                    }
                    /* BIST selects its own inactive state for every measurement; restore the disabled-channel
                     * electrical contract before continuing or before the next BIST target. */
                    _disabled_widgets_force_highz();
                    /* BIST leaves the CSD block in its private configuration. Reinitialize the middleware,
                     * then prepare CSD so disabled electrodes are High-Z before regular scanning resumes. */
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    _prepare_csd_mode();
                    /* 重扫要靠 CSD 中断推进扫描链, 故必须在这之前恢复中断; BIST 残留的 pending 一并清掉。 */
                    NVIC_ClearPendingIRQ(CYBSP_CSD_IRQ);
                    NVIC_EnableIRQ(CYBSP_CSD_IRQ);
                    /* BIST 期间 raw 与正常扫描口径无关 —— 必须先真扫一遍再立基线, 否则整块面板的
                     * 基线被钉在 BIST 残留值上并因"判为按下"永久冻结(见函数处说明)。 */
                    _scan_then_initialize_baselines();
#endif
                    interrupt_state = Cy_SysLib_EnterCriticalSection();
                    measure_cp_active = false;
                    Cy_SysLib_ExitCriticalSection(interrupt_state);
                }
            }

            /* ALGO_END commit：主循环校验 CRC16 + 拷贝 1KB，避免在 ISR 里做耗时 memcpy。
             * 校验失败拒绝、保留旧算法(algo_valid 不变)，符合设计文档 §3 ALGO_END 语义。 */
            if (algo_commit_pending)
            {
                spi_dbg.stage = MLOOP_STAGE_ALGO_COMMIT;
                uint16_t len;
                uint16_t crc_expect;
                uint16_t crc_calc;

                algo_commit_pending = false;
                len = algo_expected_len;
                crc_expect = algo_commit_crc;
                crc_calc = algo_crc16(algo_staging, len);

                if (crc_calc == crc_expect)
                {
                    memcpy(algo_slot, algo_staging, ALGO_SLOT_SIZE);
                    algo_len = len;
                    algo_valid = true;
                }
                /* CRC 不一致：拒绝，algo_valid 保持旧值不变。 */
            }

            /* 全局 CSD 配置改动：完整 Init + Initialize 重初始化，重算 inactive_sns/IDAC/MFS
             * 的内部预计算；随后显式恢复禁用电极 High-Z。轻量 APPLY 不重算 → 会坏扫描。 */
            if (global_apply_pending)
            {
                spi_dbg.stage = MLOOP_STAGE_GLOBAL_APPLY;
                global_apply_pending = false;
                /* 全局配置变更应用: Init 从 ptrCommonConfig(RAM 影子)重算内部预计算
                 * (含 csdInactiveSnsDm/HSIOM)，随后 Initialize 写回硬件状态。
                 * ★不再调 Cy_CapSense_DeInit★: 实测运行时 DeInit→Init→Enable 会把扫描速率从 ~180Hz
                 * 掉到 ~15Hz(疑似 DeInit 未复位时钟分频, 再 Init 残留慢时钟); 仅 Init→Initialize
                 * 同样重算全局预计算且保持满速。 */
                /* Init 会从 ROM 生成配置重铺 widgetContext, 打掉当前生效的 resolution/snsClk
                 * (启动归一的 32, 或用户 SET_PARAM / AUTO_TUNE 的逐通道值) → 见 _widget_hw_save 注释。 */
                _widget_hw_save();
                (void)Cy_CapSense_Init(&cy_capsense_context);
                _widget_hw_restore();
                /* Cy_CapSense_Init 会把全部 widget 的 ENABLE|WORKING 重新置起
                 * (cy_capsense_control.c:147) —— 不重放位图, 用户关掉的通道会在每次"全局应用"
                 * 后偷偷复活并重新参与扫描(电极离开 High-Z)。 */
                _ch_enable_restore();
                /* Init 会按 ptrCommonConfig 重铺 widgetContext 增益档 → 先把用户锁定值写回,
                 * 再由紧随其后的 Initialize 一并下到硬件(不额外触发校准)。 */
                (void)_idac_lock_restore(SENSOR_CH_ALL);
                /* ★配置更新只重算内部预计算, 不再自动校准/频率下探(转一圈)★:
                 * 恒走轻量 Initialize+基线路径, 沿用现有(上次手动校准的)IDAC。
                 * Enable 的自动校准(SmartSense 可含频率自适应)耗时长会阻塞→掉 USB, 且用户要求
                 * "频率自适应探测只能手动触发"。故校准/频率下探仅由显式 CALIBRATE / AUTO_TUNE 命令触发,
                 * 配置改动(inactive_sns/IDAC/MFS)本身即时生效但不重扫校准。 */
                (void)Cy_CapSense_Initialize(&cy_capsense_context);
                _prepare_csd_mode();
                /* inactive_sns/IDAC/MFS 刚变 ⇒ 旧 raw 与新配置差一个量级, 不能直接当基线。 */
                _scan_then_initialize_baselines();
            }

            /* APPLY 指令：在主循环(非 ISR)重新初始化扫描硬件使硬件参数(分辨率/时钟/IDAC)生效。 */
            if (apply_pending)
            {
                const uint32_t apply_t0 = g_ms_tick;
                spi_dbg.stage = MLOOP_STAGE_APPLY;

                apply_pending = false;
                /* ★放行屏障: 启用位图在本次重配【之前】落到中间件★
                 * provisioning 期间 _ch_enable_apply 是空转的(它见 g_provision_pending 直接返回),
                 * 于是位图只存在于 g_ch_enabled 里、中间件的 widget ENABLE 位还全是关。若不在此处
                 * 重放, 下面的校准/基线跑完仍然一个通道都不会被扫, 要等下一轮 _ch_enable_apply 逐
                 * 通道 SetWidgetStatus + 再校准一次 —— 那就是"provision 后 APPLY 12.4s"的来源
                 * (同一批通道被校准两遍)。这里重放并吃掉脏位, 本次 APPLY 的校准+基线即为终态。 */
                if (g_provision_apply_release)
                {
                    uint32_t rel_st = Cy_SysLib_EnterCriticalSection();
                    ch_enable_dirty = 0u;
                    Cy_SysLib_ExitCriticalSection(rel_st);
                    _ch_enable_restore();
                    _prepare_csd_mode();
                }
                if (scan_mode == SCAN_MODE_AUTO && g_auto_calibrate)
                {
                    /* Do not call Enable here: it performs Initialize then starts ScanAllWidgets internally,
                     * leaving no application point to restore disabled electrodes before the first sample. */
                    spi_dbg.stage = MLOOP_STAGE_APPLY_INIT;
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    _prepare_csd_mode();
                    spi_dbg.stage = MLOOP_STAGE_APPLY_RECAL;
                    _calibrate_enabled_channels();
                    spi_dbg.stage = MLOOP_STAGE_APPLY_BASELINE;
                    _scan_then_initialize_baselines();
                }
                else
                {
                    /* Fixed-IDAC apply still resets every IO through Initialize; restore the disabled
                     * electrical contract before resuming scan scheduling. */
                    if (g_auto_calibrate)
                    {
                        spi_dbg.stage = MLOOP_STAGE_APPLY_RECAL;
                        _recalibrate_dirty_channels();
                    }
                    spi_dbg.stage = MLOOP_STAGE_APPLY_INIT;
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    _prepare_csd_mode();
                    spi_dbg.stage = MLOOP_STAGE_APPLY_BASELINE;
                    _scan_then_initialize_baselines();
                }
                (void)apply_t0;
                /* APPLY 是 RP provisioning 的 FIFO 完成屏障：只有主循环已执行完重配后才开闸。 */
                if (g_provision_apply_release)
                {
                    g_provision_apply_release = false;
                    g_provision_pending = false;
                }
            }

            /* QUICK_APPLY(Sweep 专用)：ISR 已保存本格的 gain/div；只在 NOT_BUSY 窗口原子写入
             * widgetContext，再 Initialize 让参数下到硬件。绝不校准/基线，禁用电极由
             * _prepare_csd_mode() 继续强制 High-Z。 */
            if (quick_apply_pending)
            {
                uint8_t qa_target;
                uint8_t qa_gain;
                uint8_t qa_div;
                uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
                qa_target = quick_apply_ch;
                qa_gain = quick_apply_gain;
                qa_div = quick_apply_div;
                quick_apply_pending = false;
                quick_apply_ch = SENSOR_CH_ALL;
                quick_apply_gain = 0u;
                quick_apply_div = 1u;
                Cy_SysLib_ExitCriticalSection(interrupt_state);

                spi_dbg.stage = MLOOP_STAGE_QUICK_APPLY;
                if (qa_target < SENSOR_CHANNEL_COUNT)
                {
                    cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[qa_target];
                    const uint64_t bit = ((uint64_t)1u << qa_target);
                    wc->idacGainIndex = qa_gain;
                    wc->snsClk = qa_div;
                    spi_dbg.clk_set_cnt++;
                    spi_dbg.clk_set_last = (uint32_t)qa_div | ((uint32_t)qa_target << 16u);
                    g_idac_lock.gain[qa_target] = qa_gain;
                    g_idac_lock.mask |= bit;
                    idac_dirty_mask |= bit;
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    _prepare_csd_mode();
                    /* 本次已经让 gain/div 生效；普通 APPLY 不得再为该格补跑校准。 */
                    idac_dirty_mask &= ~bit;
                }
            }

            /* CALIBRATE：真正的 IDAC 重校准(把 raw 拉回目标, 修 railed), 再复位基线。
             * 与 APPLY 区分——APPLY 只重配/re-init, 不重算 IDAC; 半自动手动下"校准"必须走这里
             * 才有效(否则 raw 一直卡满量程 diff=0)。CalibrateAllWidgets 需校准使能。 */
            if (calibrate_pending)
            {
                /* ★单通道语义端到端透传★: cal_target < 36 ⇒ 只校准该 widget 并只初始化该 widget
                 * 的基线(禁止 InitializeAllBaselines 把其它 35 个通道的基线一并冲掉)。
                 * UI 的"全通道校准"改由 host 串行队列逐通道下发, 故 0xFF 分支只保留兼容入口
                 * (恢复默认 / AUTO_CALIBRATE_EN 上升沿等固件内部触发仍需要它)。 */
                const uint8_t cal_target = calibrate_ch;
                const bool cal_one = (cal_target < SENSOR_CHANNEL_COUNT);
                spi_dbg.stage = MLOOP_STAGE_CALIBRATE;
                calibrate_pending = false;
                calibrate_ch = SENSOR_CH_ALL;   /* 消费即复位: 内部触发(无 rx[2])一律全通道语义 */
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
                /* ★逐通道校准, 不用 CalibrateAllWidgets★
                 * CalibrateAllWidgets 对全部 widget 一律用【全局起点增益档】csdIdacGainInitIndex,
                 * 而本面板 36 段的 Cp 跨度极大(实测 ch8≈22pF、ch35≈138pF), 各通道的 IDAC 增益档
                 * 本就不该相同 —— 用户逐通道设过的档位记在 g_idac_lock 里, 被它统一按全局档校准后
                 * 等于全部作废(高 Cp 通道压不下来仍 railed, 低 Cp 通道又过冲)。
                 * 故与 _recalibrate_dirty_channels() 同口径: 逐通道调用 _calibrate_widget_locked(),
                 * 它会在校准该通道时临时把全局起点档替换为该通道自己的锁定档, 校准完还原。
                 * 未锁定的通道行为不变(仍用全局档)。 */
                /* ★禁用通道一律跳过★ 校准要连接电极并扫描该 widget, 与"关闭 ⇒ 电极恒高阻"直接
                 * 冲突; 单通道请求落在禁用通道上就是一次无意义(且违背语义)的操作, 直接不做。 */
                if (cal_one)
                {
                    if (_ch_is_enabled((uint32_t)cal_target))
                    {
                        (void)_calibrate_widget_locked((uint32_t)cal_target);
                    }
                }
                else
                {
                    for (uint32_t cal_ch = 0u; cal_ch < SENSOR_CHANNEL_COUNT; cal_ch++)
                    {
                        if (!_ch_is_enabled(cal_ch)) { continue; }
                        (void)_calibrate_widget_locked(cal_ch);
                    }
                }
#else
                (void)Cy_CapSense_Initialize(&cy_capsense_context);
                _prepare_csd_mode();
#endif
                /* 校准会把增益档拉回起点档；只恢复本次目标范围。CSDv2 在后续扫描时按
                 * widgetContext 装载 IDAC，所以不调用全局 Initialize，也不重置其它 widget 状态。 */
                (void)_idac_lock_restore(cal_target);
                if (cal_one)
                {
                    if (_ch_is_enabled((uint32_t)cal_target))
                    {
                        Cy_CapSense_InitializeWidgetBaseline((uint32_t)cal_target, &cy_capsense_context);
                    }
                }
                else
                {
                    _initialize_enabled_baselines();
                }
            }

            /* BASELINE_RESET：把基线重置到当前 raw(消除历史漂移), 不动 IDAC/参数。
             * baseline_ch < 36 ⇒ 只初始化该 widget 的基线(单通道"基线"不该动其它通道)。 */
            if (baseline_reset_pending)
            {
                const uint8_t bsln_target = baseline_ch;
                spi_dbg.stage = MLOOP_STAGE_BASELINE_RESET;
                baseline_reset_pending = false;
                baseline_ch = SENSOR_CH_ALL;   /* 消费即复位, 同 calibrate_ch */
                if (bsln_target < SENSOR_CHANNEL_COUNT)
                {
                    /* 禁用通道没有"当前 raw"可言(发布值恒 0), 复位它的基线毫无意义 → 跳过。 */
                    if (_ch_is_enabled((uint32_t)bsln_target))
                    {
                        Cy_CapSense_InitializeWidgetBaseline((uint32_t)bsln_target, &cy_capsense_context);
                    }
                }
                else
                {
                    _initialize_enabled_baselines();
                }
            }

            /* AUTO_TUNE：频率自适应下探。高 Cp 电极在高频(小分频)下传感器来不及建立→IDAC 无论如何
             * 都压不到目标%→CalibrateWidget 返回非 SUCCESS。此处从高频起逐档升分频(降频),
             * 每档重校准, 用中间件校准返回状态判定该频率下 IDAC 能否满足目标; 首个成功的分频即为
             * "满足当前 IDAC 设置的最高频率(最快扫描)", 写回该 widgetContext 并记录; 全部档位都失败
             * = 超该通道硬件能力 → 该通道失败。运行时可调, 结果经 GET_AUTO_TUNE 上报。 */
            /* 通道 Cp 跨度极大(22pF~138pF), 单一统一分频无法兼顾 → 两种模式都按【单通道】判定:
             *   auto_tune_ch = 0..35: 只改该 widget 的 snsClk(其余通道不动), CalibrateWidget 判定;
             *   auto_tune_ch = 0xFF : 逐通道各自校准——外层遍历 36 通道, 每通道独立跑完整三步算法,
             *                         各得其自身分频; 不再要求"全通道共用同一分频且全部通过"。 */
            if (auto_tune_pending)
            {
                spi_dbg.stage = MLOOP_STAGE_AUTO_TUNE;
                const uint8_t target_ch = auto_tune_ch;
                const uint8_t pref = auto_tune_pref;
                uint16_t final_div = 0u;
                bool tuned = false;
                auto_tune_pending = false;

                if (target_ch < SENSOR_CHANNEL_COUNT)
                {
                    /* 禁用通道不做频率自适应: 它要逐档连接电极重校准, 与"恒高阻"冲突。
                     * 如实回失败(result=2)而不是假装成功, 上位机据此提示"该通道已关闭"。 */
                    tuned = _ch_is_enabled((uint32_t)target_ch) &&
                            auto_tune_run_ch(target_ch, pref, &final_div);
                }
                else
                {
                    /* 逐通道各自校准: 单通道失败不拖垮全场, 只影响该通道(其分频保持上一次的值)。
                     * 进度期间 auto_tune_ch 回显"当前正在处理的通道", 供上位机显示 CHn/36。 */
                    uint32_t ok_count = 0u;
                    uint32_t w;
                    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                    {
                        uint16_t ch_div = 0u;
                        if (!_ch_is_enabled(w)) { continue; }
                        auto_tune_ch = (uint8_t)w;
                        if (auto_tune_run_ch((uint8_t)w, pref, &ch_div)) ok_count++;
                    }
                    auto_tune_ch = AUTO_TUNE_CH_ALL;      /* 终态回显请求语义(全通道) */
                    final_div = (uint16_t)ok_count;       /* 完成态: div 字段 = 成功通道数 */
                    tuned = (ok_count > 0u);
                }

                /* 逐档校准会改回起点档；只恢复本次目标范围。扫描会自然装载该 widget 的
                 * 新 gain，不调用全局 Initialize，避免扰动其它 widget 的状态/基线。 */
                (void)_idac_lock_restore(target_ch);
                /* 最终分频只影响本次目标，单通道仅重置该 widget 基线。 */
                if (target_ch < SENSOR_CHANNEL_COUNT)
                {
                    if (_ch_is_enabled((uint32_t)target_ch))
                    {
                        Cy_CapSense_InitializeWidgetBaseline((uint32_t)target_ch, &cy_capsense_context);
                    }
                }
                else
                {
                    _initialize_enabled_baselines();
                }
                auto_tune_div = tuned ? final_div : 0u;   /* 失败: div 无效(否则残留最后一次试探值) */
                auto_tune_phase = AUTO_TUNE_PHASE_DONE;
                auto_tune_step  = 0u;
                auto_tune_result = tuned ? 1u : 2u;       /* 1=成功(至少一个通道) 2=全部失败 */
            }

            /* ★处理中锁定解除★：本轮已把入队的重操作全部做完(且未被 ISR 追加新的)→ 清 busy。
             * RP2040 轮询 GET_STATS 的 busy 字节由 1→0 即判定该重操作真实完成(替代盲等 sleep)。 */
            if (!apply_pending && !quick_apply_pending && !calibrate_pending &&
                !baseline_reset_pending &&
                !global_apply_pending && !auto_tune_pending &&
                !measure_cp_pending && !measure_cp_active)
            {
                g_op_busy = 0u;
            }

            /* ★通道启用/禁用在此落实★ 放在所有重操作之后、启动下一轮扫描之前:
             * 此刻 NOT_BUSY 成立(Cy_CapSense_SetWidgetStatus 内部的 SwitchSensingMode 要求),
             * 且新的启用集会立刻对下面这一次 ScanAllWidgets 生效(禁用的 widget 从此不再被 setup,
             * 电极停在 High-Z; 新启用的 widget 已完成校准+基线)。 */
            _ch_enable_apply();

            scan_count++;
            /* 带外 SWD 可读的存活证据: 扫描计数。 */
            spi_dbg.scan_count_m = scan_count;
            spi_dbg.stage = MLOOP_STAGE_SCAN_START;
            /* ★全部通道都被禁用时不得启动扫描★ Cy_CapSense_ScanAllWidgets_V2 会因找不到任何可用
             * widget 而返回 BAD_PARAM 且不置忙标志; 照旧调用只是每轮白跑一次 36 次 SetupWidget 失败。
             * 主循环继续空转(scan_count 照增), 故 RP2040 的"主循环卡死"兜底不会误判。 */
            if (!g_provision_pending && _any_ch_enabled())
            {
                /* All normal reset paths prepare CSD explicitly. This guard only repairs a
                 * subsequent mode change, avoiding a per-scan disabled-widget traversal. */
                if (cy_capsense_context.ptrActiveScanSns->currentSenseMethod != CY_CAPSENSE_CSD_GROUP)
                {
                    _prepare_csd_mode();
                }
                const uint8_t scan_target = _focus_scan_target();
                if (scan_target < SENSOR_CHANNEL_COUNT)
                {
                    Cy_CapSense_ScanWidget(scan_target, &cy_capsense_context);
                }
                else
                {
                    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
                }
            }
        }
    }
}

/* [] END OF FILE */
