/******************************************************************************
 * File Name:   main.c
 *
 * Description: PSoC 4 SPI Slave with CAPSENSE and Status LED
 * - SPI Slave: SCB0 (SLAVE, MODE0, 8-bit, MSB-first, CS ActiveLow)
 * - CAPSENSE: 36 buttons, immutable double-buffered snapshots
 * - Status LED: P1.6 (CYBSP_LED_SLD3)
 * - FW_VERSION: 0x0000040C (0.4.12)
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
#include <stdint.h>
#include <string.h>

#define FW_VERSION_MAJOR                 (0u)
#define FW_VERSION_MINOR                 (4u)
#define FW_VERSION_PATCH                 (18u)
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
#define SENSOR_CMD_AUTO_TUNE             (0x3Du)  // 频率自适应: 从高频起逐档升 snsClk 分频, 直到校准能压到目标%(或超上限失败), 主循环执行
#define SENSOR_CMD_GET_AUTO_TUNE         (0x3Eu)  // 读自适应结果; 响应 [magic,GET_AUTO_TUNE,result(0进行中/1成功/2失败),0,div24]
// 全局参数 id
#define GPARAM_INACTIVE_SNS              (0x01u)  // 未激活传感器连接: 1=GND 2=High-Z 4=Shield
#define GPARAM_IDAC_GAIN_INIT            (0x02u)  // csdIdacGainInitIndex(IDAC 增益档索引)
#define GPARAM_IDAC_MIN                  (0x03u)  // csdIdacMin(CSD 校准最小 IDAC)
#define GPARAM_RAW_TARGET                (0x04u)  // csdRawTarget(校准目标 raw 百分比)
#define GPARAM_MFS_DIV_F1                (0x05u)  // csdMfsDividerOffsetF1(多频通道1分频偏移)
#define GPARAM_MFS_DIV_F2                (0x06u)  // csdMfsDividerOffsetF2(多频通道2分频偏移)
#define GPARAM_IDAC_SENSE_CONFIG         (0x07u)  // csdChargeTransfer: 0=IDAC sourcing, 1=IDAC sinking (运行时可设)
#define GPARAM_AUTO_CALIBRATE_EN         (0x08u)  // 运行时是否自动校准: 0=固定IDAC(不自动校准), 1=Init/Apply 自动校准
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
/* SET_GLOBAL 置位：全局 CSD 配置(inactive_sns/IDAC/MFS)改动需完整 Init+Enable 重初始化才能
 * 重算内部预计算(csdInactiveSnsDm/HSIOM 见 cy_capsense_sensing.c)。轻量 APPLY 不够，会坏扫描。
 * 主循环执行与启动同序的 Cy_CapSense_Init+Enable(已验证可用)。 */
static volatile bool global_apply_pending = false;
/* 主循环测量期间保持 true；SPI ISR 据 pending/active 返回 0，避免读到半更新数组。 */
static volatile bool measure_cp_active = false;
/* CALIBRATE 指令置位：主循环执行真正的 IDAC 重校准 CalibrateAllWidgets(不能在 ISR 里做)。 */
static volatile bool calibrate_pending = false;
/* BASELINE_RESET 指令置位：主循环执行 InitializeAllBaselines 重置全部通道基线。 */
static volatile bool baseline_reset_pending = false;
/* AUTO_TUNE 指令置位：主循环执行频率自适应下探(逐档升 snsClk 分频重校准)。 */
static volatile bool auto_tune_pending = false;
/* 自适应结果: 0=未执行/进行中, 1=成功(auto_tune_div 为找到的分频), 2=失败(超硬件上限仍压不到目标)。 */
static volatile uint8_t auto_tune_result = 0u;
/* 自适应成功时找到的统一 snsClk 分频(已写回全部 widgetContext)。 */
static volatile uint16_t auto_tune_div = 0u;
/* ★处理中锁定/真实完成反馈★：ISR 收到 APPLY/CALIBRATE/BASELINE_RESET/GLOBAL_COMMIT 即置 1,
 * 主循环把对应重操作真正做完后清 0。经 GET_STATS 响应 byte[2] 上报, 供 RP2040 轮询至真实完成
 * (替代原固定 sleep 盲等), 上位机据此显示"处理中"并在真正完成后解锁/刷新, 而非命令一到就误判完成。 */
static volatile uint8_t g_op_busy = 0u;
/* 任一通道当前激活(算法 out_active / base_active 聚合)。驱动白 LED: 运行时激活亮、空闲灭。
 * 使新人写的算法只要令某通道判定为触发, 白灯即亮, 直观可视化算法作用(需求: 算法点亮白LED示例)。 */
static volatile bool g_any_active = false;
/* 全局"是否自动校准"运行时开关(GPARAM_AUTO_CALIBRATE_EN, 默认开)。开=Init/Apply 走 Enable 自动校准 IDAC;
 * 关=只 Initialize+基线复位, 用固定 IDAC(配置/手动值)换取稳定灵敏度范围(修某些通道自动校准发散 railed)。
 * "校准"命令(SENSOR_CMD_CALIBRATE)始终执行一次显式校准, 不受本开关影响(满足"点校准=单次校准"需求)。 */
static volatile bool g_auto_calibrate = true;
/* 每通道最近一次寄生电容测量值(fF)，0xFFFFFF=失败/未测量。 */
static volatile uint32_t cp_value[SENSOR_CHANNEL_COUNT];
/* CSD 处理模式：SCAN_MODE_AUTO(自动校准/标准完整处理) / SCAN_MODE_SEMI(半自动手动)。 */
static volatile uint8_t scan_mode = SCAN_MODE_AUTO;
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

static void spi_slave_task(void);
static void spi_isr(void);

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
    }
    if (g_common_cfg_ram.csdRawTarget < 60u || g_common_cfg_ram.csdRawTarget > 90u) {
        g_common_cfg_ram.csdRawTarget = 85u;   /* 校准目标 85%: 留触摸下摆空间且不易饱和 */
    }

    cy_capsense_context.ptrCommonConfig = &g_common_cfg_ram;
}

/* 写全局 CSD 配置到 RAM 影子(运行时可配项)。改后需 APPLY 重初始化生效。 */
static void cmd_set_global(uint8_t gparam_id, uint32_t value)
{
    switch (gparam_id)
    {
        case GPARAM_INACTIVE_SNS:
            /* 仅接受合法连接模式: 1=GND 2=High-Z 4=Shield，防非法值致扫描异常。 */
            if ((value == 1u) || (value == 2u) || (value == 4u))
            {
                g_common_cfg_ram.csdInactiveSnsConnection = (uint8_t)value;
            }
            break;
        /* 防护: IDAC 增益档 0..7; IDAC min 0..127(7位); 校准目标 1..99%(0/≥100 会让自动校准发散→railed)。 */
        /* IDAC 增益档合法索引 0..6(idacGainTable 共 CY_CAPSENSE_IDAC_GAIN_NUMBER=7 项), 索引7越界会读
         * 到表外 gainReg → 写非法 IDAC → 扫描挂死/看门狗复位(崩溃)。夹到 <=6。 */
        case GPARAM_IDAC_GAIN_INIT: if (value <= 6u)   { g_common_cfg_ram.csdIdacGainInitIndex = (uint8_t)value; } break;
        case GPARAM_IDAC_MIN:       if (value <= 127u) { g_common_cfg_ram.csdIdacMin           = (uint8_t)value; } break;
        case GPARAM_RAW_TARGET:     if ((value >= 1u) && (value <= 99u)) { g_common_cfg_ram.csdRawTarget = (uint8_t)value; } break;
        case GPARAM_MFS_DIV_F1:     g_common_cfg_ram.csdMfsDividerOffsetF1   = (uint8_t)value; break;
        case GPARAM_MFS_DIV_F2:     g_common_cfg_ram.csdMfsDividerOffsetF2   = (uint8_t)value; break;
        /* IDAC 感应配置(sourcing/sinking): 运行时可设的充电方向, 影响灵敏度极性/范围。 */
        case GPARAM_IDAC_SENSE_CONFIG: g_common_cfg_ram.csdChargeTransfer = (value != 0u) ? (uint8_t)CY_CAPSENSE_IDAC_SINKING : (uint8_t)CY_CAPSENSE_IDAC_SOURCING; break;
        case GPARAM_AUTO_CALIBRATE_EN: g_auto_calibrate = (value != 0u); break;
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

    for (ch = 0u; ch < SENSOR_CHANNEL_COUNT; ch++)
    {
        uint32_t base_active = Cy_CapSense_IsWidgetActive((uint32_t)ch, &cy_capsense_context);
        uint32_t active = use_algo ? algo_engine_run_channel(ch, base_active) : base_active;

        if (0u != active)
        {
            mask[ch >> 3u] |= (uint8_t)(1u << (ch & 7u));
        }
    }

    /* 聚合"任一通道激活"供白 LED 运行时指示(算法可视化)。 */
    g_any_active = (bool)((mask[0] | mask[1] | mask[2] | mask[3] | mask[4]) != 0u);

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
/* CSD 参数合法性防护: 非法值会让转换railed(满量程)/时钟异常/校准发散, 故在应用点拒绝越界值,
 * 保留原值不变。范围依据 CSDv2(cy_capsense_structure.h):
 *   RESOLUTION 6..16 位; SNS_CLK_DIV 1..255(0会除零); IDAC_MOD 0..127(7位);
 *   IDAC_GAIN 增益档 0..7; SNS_CLK_SOURCE 低7位(去 AUTO 0x80)取值 0..6。 */
static bool _param_value_legal(uint8_t param_id, uint32_t value)
{
    switch (param_id)
    {
        case PARAM_RESOLUTION:     return (value >= 6u)  && (value <= 16u);
        case PARAM_SNS_CLK_DIV:    return (value >= 1u)  && (value <= 255u);
        case PARAM_IDAC_MOD:       return (value <= 127u);
        case PARAM_IDAC_GAIN:      return (value <= 6u);   /* 增益档 0..6(表7项,索引7越界崩溃) */
        case PARAM_SNS_CLK_SOURCE: return ((value & 0x7Fu) <= 6u);
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
        case PARAM_SNS_CLK_DIV:   wc->snsClk       = (uint16_t)value; break;
        case PARAM_IDAC_MOD:      wc->idacMod[0]   = (uint8_t)value;  break;
        case PARAM_SNS_CLK_SOURCE:wc->snsClkSource = (uint8_t)value;  break;
        case PARAM_IDAC_GAIN:     wc->idacGainIndex= (uint8_t)value;  break;
        default: return false;
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
    spi_tx_frame[2] = g_op_busy;   // 处理中标志(1=主循环正在做重操作), 供 RP2040 轮询至真实完成
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

/* ---- JIT 算法引擎 SPI 命令处理（ISR 上下文，仅做快速缓冲写入/标志置位）---- */
static void spi_load_algo_response(uint8_t command, uint8_t b2, uint8_t b3, uint16_t len)
{
    spi_tx_frame[0] = SENSOR_FRAME_MAGIC;
    spi_tx_frame[1] = command;
    spi_tx_frame[2] = b2;
    spi_tx_frame[3] = b3;
    spi_tx_frame[4] = (uint8_t)(len & 0xFFu);
    spi_tx_frame[5] = (uint8_t)((len >> 8u) & 0xFFu);
    spi_tx_frame[6] = 0u;
    Cy_SCB_SPI_WriteArray(scb_0_HW, spi_tx_frame, SENSOR_FRAME_SIZE);
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

/* 读某通道算法追踪: 帧 [magic,GET_TRACE,ch,idx,..]。响应 [.. ,ch,out_active,report[idx]_lo,report[idx]_hi,0]。
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
    spi_load_algo_response(ALGO_GET_TRACE, ch, act, rep);
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
            spi_load_cmd_response(SENSOR_CMD_SET_GLOBAL, rx[2], 0u, val);
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
            measure_cp_pending = true;
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
            apply_pending = true;
            g_op_busy = 1u;   // 处理中锁定
            spi_load_cmd_response(SENSOR_CMD_APPLY, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_CALIBRATE:
            // 真正的 IDAC 重校准(CalibrateAllWidgets)+基线复位; 耗时, 仅置标志由主循环执行。
            calibrate_pending = true;
            g_op_busy = 1u;   // 处理中锁定
            spi_load_cmd_response(SENSOR_CMD_CALIBRATE, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_BASELINE_RESET:
            // 仅重置全部通道基线; 主循环执行 InitializeAllBaselines。
            baseline_reset_pending = true;
            g_op_busy = 1u;   // 处理中锁定
            spi_load_cmd_response(SENSOR_CMD_BASELINE_RESET, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_AUTO_TUNE:
            // 频率自适应下探(耗时: 逐档升分频重校准); 仅置标志由主循环执行, 结果经 GET_AUTO_TUNE 读。
            auto_tune_pending = true;
            auto_tune_result  = 0u;   // 进行中
            g_op_busy = 1u;           // 处理中锁定(host 轮询 busy 至真实完成)
            spi_load_cmd_response(SENSOR_CMD_AUTO_TUNE, 0u, 0u, 0u);
            break;

        case SENSOR_CMD_GET_AUTO_TUNE:
            // 读自适应结果: [result(0进行中/1成功/2失败), 0, div24]。
            spi_load_cmd_response(SENSOR_CMD_GET_AUTO_TUNE, auto_tune_result, 0u, auto_tune_div);
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
    for (uint32_t channel = 0u; channel < SENSOR_CHANNEL_COUNT; channel++)
    {
        cp_value[channel] = 0xFFFFFFu;
    }
    measure_cp_pending = false;
    measure_cp_active = false;
    global_apply_pending = false;
    calibrate_pending = false;
    baseline_reset_pending = false;
    auto_tune_pending = false;
    auto_tune_result = 0u;
    auto_tune_div = 0u;
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
    // hardware.txt: P1.6 -> LED -> R -> GND，因此高电平明确为点亮。
    // 白灯启动即点亮作为 bring-up 指示; 进入主循环后交由 g_any_active 驱动:
    // 运行空闲熄灭、任一通道激活(算法 out_active/base_active)点亮 → 兼作算法可视化。
    Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM, STATUS_LED_ON_STATE);
    /* ★可见启动指示(修"重启白灯不亮")★: 白灯在启动后保持点亮 800ms 再交给 g_any_active 驱动,
     * 使每次上电/XRES 重启都有肉眼可见的白灯闪亮(否则仅亮几微秒无法察觉, 用户误判"重启无效")。 */
    uint32_t led_boot_until_ms = g_ms_tick + 800u;
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for (;;)
    {
        /* SPI 从机改为中断驱动（spi_isr），主循环只做 capsense 扫描/发布快照。
           SPI 响应不再被 ProcessAllWidgets 的耗时阻塞。 */
        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
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
                for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                {
                    (void)Cy_CapSense_ProcessWidgetExt(w, manual_mask, &cy_capsense_context);
                }
            }
            update_touch_frame();       /* 刷新实时触控帧（快路数据源） */
            /* 白 LED: 启动 800ms 内常亮(可见重启指示), 之后运行时任一通道激活则亮、空闲灭。 */
            Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM,
                          (g_any_active || (g_ms_tick < led_boot_until_ms))
                              ? STATUS_LED_ON_STATE : STATUS_LED_OFF_STATE);
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
                    /* 先统一标为失败；BIST 可用的 Cp 结果随后覆盖。active 期间 SPI GET_CP 始终返回 0。 */
                    for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                    {
                        cp_value[w] = 0xFFFFFFu;
                    }
#if (defined(CY_CAPSENSE_TST_SNS_CAP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN))
                    for (uint32_t w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                    {
                        uint32_t v = 0u;
                        cy_en_capsense_bist_status_t status =
                            Cy_CapSense_MeasureCapacitanceSensor(w, 0u, &v, &cy_capsense_context);

                        /* HIGH_LIMIT is the 400pF BIST saturation/overrange outcome and
                         * indicates a short-circuit fault; LOW_LIMIT indicates abnormally
                         * small capacitance. Both are faults, like BAD_PARAM/HW_BUSY/TIMEOUT/ERROR
                         * and a zero result, so the prefilled 0xFFFFFF failure marker remains.
                         * Only a nonzero SUCCESS result is valid. 0xFFFFFF remains reserved for
                         * the protocol failure marker, so a colliding valid value saturates at
                         * 0xFFFFFE. */
                        if ((CY_CAPSENSE_BIST_SUCCESS_E == status) && (v != 0u))
                        {
                            cp_value[w] = (v >= 0xFFFFFFu) ? 0xFFFFFEu : v;
                        }
                    }
                    /* BIST 改写 CSD HW，按当前模式恢复自动校准/标准完整处理或半自动手动配置。 */
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
                    interrupt_state = Cy_SysLib_EnterCriticalSection();
                    measure_cp_active = false;
                    Cy_SysLib_ExitCriticalSection(interrupt_state);
                }
            }

            /* ALGO_END commit：主循环校验 CRC16 + 拷贝 1KB，避免在 ISR 里做耗时 memcpy。
             * 校验失败拒绝、保留旧算法(algo_valid 不变)，符合设计文档 §3 ALGO_END 语义。 */
            if (algo_commit_pending)
            {
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

            /* 全局 CSD 配置改动：完整 Init+Enable 重初始化(与启动同序),重算 inactive_sns/IDAC/MFS
             * 的内部预计算。轻量 APPLY 不重算 → 会坏扫描,故全局改动必须走此完整路径。 */
            if (global_apply_pending)
            {
                global_apply_pending = false;
                /* 全局配置变更应用: Init 从 ptrCommonConfig(RAM 影子)重算内部预计算
                 * (含 csdInactiveSnsDm/HSIOM), Enable 校准+基线+首扫。
                 * ★不再调 Cy_CapSense_DeInit★: 实测运行时 DeInit→Init→Enable 会把扫描速率从 ~180Hz
                 * 掉到 ~15Hz(疑似 DeInit 未复位时钟分频, 再 Init 残留慢时钟); 仅 Init→Enable 同样重算
                 * 全局预计算且保持满速。 */
                (void)Cy_CapSense_Init(&cy_capsense_context);
                /* 自动校准开→Enable(重算 IDAC); 关→仅 Initialize+基线(固定 IDAC, 稳定灵敏度范围)。 */
                if (g_auto_calibrate) {
                    (void)Cy_CapSense_Enable(&cy_capsense_context);
                } else {
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
                }
            }

            /* APPLY 指令：在主循环(非 ISR)重新初始化扫描硬件使硬件参数(分辨率/时钟/IDAC)生效。 */
            if (apply_pending)
            {
                apply_pending = false;
                if (scan_mode == SCAN_MODE_AUTO && g_auto_calibrate)
                {
                    /* 自动校准开：重新启用 CapSense(含 IDAC 自动校准)，后续继续标准完整处理。 */
                    (void)Cy_CapSense_Enable(&cy_capsense_context);
                }
                else
                {
                    /* 半自动手动 或 关闭自动校准：从 widgetContext 重配硬件(snsClk/resolution/idac)
                     * 并重置基线，用固定 IDAC(不重算校准)，保留手动阈值与手动硬件参数。 */
                    (void)Cy_CapSense_Initialize(&cy_capsense_context);
                    Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
                }
            }

            /* CALIBRATE：真正的 IDAC 重校准(把 raw 拉回目标, 修 railed), 再复位基线。
             * 与 APPLY 区分——APPLY 只重配/re-init, 不重算 IDAC; 半自动手动下"校准"必须走这里
             * 才有效(否则 raw 一直卡满量程 diff=0)。CalibrateAllWidgets 需校准使能。 */
            if (calibrate_pending)
            {
                calibrate_pending = false;
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
                (void)Cy_CapSense_CalibrateAllWidgets(&cy_capsense_context);
#else
                (void)Cy_CapSense_Enable(&cy_capsense_context);
#endif
                Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
            }

            /* BASELINE_RESET：仅把全部通道基线重置到当前 raw(消除历史漂移), 不动 IDAC/参数。 */
            if (baseline_reset_pending)
            {
                baseline_reset_pending = false;
                Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
            }

            /* AUTO_TUNE：频率自适应下探。高 Cp 面板在高频(小分频)下传感器来不及建立→IDAC 无论如何
             * 都压不到目标%→CalibrateAllWidgets 返回非 SUCCESS。此处从高频起逐档升分频(降频),
             * 每档重校准, 用中间件校准返回状态判定该频率下 IDAC 能否满足目标; 首个成功的分频即为
             * "满足当前 IDAC 设置的最高频率(最快扫描)", 写回全部 widgetContext 并记录; 全部档位都失败
             * = 超硬件能力 → 返回失败。运行时可调, 结果经 GET_AUTO_TUNE 上报。 */
            if (auto_tune_pending)
            {
                static const uint16_t k_autotune_divs[] = { 8u, 12u, 16u, 20u, 24u, 32u, 40u, 48u, 64u };
                const uint32_t div_count = sizeof(k_autotune_divs) / sizeof(k_autotune_divs[0]);
                uint32_t di;
                bool tuned = false;
                auto_tune_pending = false;
                for (di = 0u; di < div_count; di++)
                {
                    uint16_t d = k_autotune_divs[di];
                    uint32_t w;
                    for (w = 0u; w < SENSOR_CHANNEL_COUNT; w++)
                    {
                        cy_capsense_tuner.widgetContext[w].snsClk = d;
                    }
                    /* 该分频下重校准; SUCCESS = 全部 widget 的 IDAC 均能把 raw 校到目标容差内。 */
                    if (CY_CAPSENSE_STATUS_SUCCESS ==
                        Cy_CapSense_CalibrateAllWidgets(&cy_capsense_context))
                    {
                        auto_tune_div = d;   /* 已写回 widgetContext, 供上位机回读/持久化 */
                        tuned = true;
                        break;
                    }
                }
                Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
                auto_tune_result = tuned ? 1u : 2u;   /* 1=成功 2=失败(超硬件能力) */
            }

            /* ★处理中锁定解除★：本轮已把入队的重操作全部做完(且未被 ISR 追加新的)→ 清 busy。
             * RP2040 轮询 GET_STATS 的 busy 字节由 1→0 即判定该重操作真实完成(替代盲等 sleep)。 */
            if (!apply_pending && !calibrate_pending && !baseline_reset_pending &&
                !global_apply_pending && !auto_tune_pending)
            {
                g_op_busy = 0u;
            }

            scan_count++;
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        }
    }
}

/* [] END OF FILE */
