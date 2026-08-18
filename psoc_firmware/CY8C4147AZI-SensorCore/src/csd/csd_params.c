/* csd_params.c —— 参数影子/启用位图/长操作请求位, 见 csd_params.h。逐字自 main.c 搬入。 */
#include "csd_params.h"
#include "algo_engine.h"
#include "lnk_wire.h"
#include "diag_export.h"
#include "app_tick.h"
#include "cybsp.h"
#include <string.h>

/* APPLY 指令置位，主循环执行重校准（不能在 ISR 里做耗时重扫描）。 */
volatile bool apply_pending = false;
/* QUICK_APPLY(Sweep 专用)指令置位: 主循环只让 gain/div 落硬件, 不重校准/不重基线(见命令码注释)。
 * quick_apply_ch: 0..35=本次扫描的目标通道(消掉它的脏位), LNK_CH_ALL=全通道(兼容入口)。 */
volatile bool quick_apply_pending = false;
volatile uint8_t quick_apply_ch = LNK_CH_ALL;
volatile uint8_t quick_apply_gain = 0u;
volatile uint8_t quick_apply_div = 1u;
/* 硬件参数改变后仅记录对应通道，避免 APPLY 为未修改通道重校准造成状态漂移。 */
volatile uint64_t idac_dirty_mask = 0u;
/* MEASURE_CP 指令置位，主循环执行逐电极 BIST 电容测量(不能在 ISR 里做耗时测量)。 */
volatile bool measure_cp_pending = false;
/* SET_GLOBAL 置位：全局 CSD 配置(inactive_sns/IDAC/MFS)改动需完整 Init+Initialize 重初始化，
 * 重算内部预计算(csdInactiveSnsDm/HSIOM 见 cy_capsense_sensing.c)。轻量 APPLY 不够，会坏扫描。
 * 主循环执行 Cy_CapSense_Init+Initialize（已验证可用）。 */
volatile bool global_apply_pending = false;
/* 主循环测量期间保持 true；SPI ISR 据 pending/active 返回 0，避免读到半更新数组。 */
volatile bool measure_cp_active = false;
/* CALIBRATE 指令置位：主循环执行真正的 IDAC 重校准(不能在 ISR 里做)。 */
volatile bool calibrate_pending = false;
/* BASELINE_RESET 指令置位：主循环执行基线复位。 */
volatile bool baseline_reset_pending = false;
/* ★校准/基线的目标通道(单通道语义端到端透传)★ 0..35=只处理该通道, LNK_CH_ALL=全 36 通道。
 * 与 auto_tune_ch 完全同构(同一哨兵、同一 rx[2] 位置), 不新增命令码、不造平行协议。
 * ★为什么必须透传★: 上位机点"本通道校准"时原先固件把它扩成 36 通道重校准 —— 既慢(逐通道
 * CalibrateWidget × 36)又会把其它通道刚调好的 IDAC/基线一起冲掉, 与"只调这一个通道"的语义相反。
 * UI 的批量(全通道)现在由 host 侧可取消串行队列逐通道发起, 不再依赖固件内部循环。 */
volatile uint8_t calibrate_ch = LNK_CH_ALL;
volatile uint8_t baseline_ch = LNK_CH_ALL;
/* 全局"是否自动校准"运行时开关(GPARAM_AUTO_CALIBRATE_EN, 默认开)。开=Init/Apply 走 Enable 自动校准 IDAC;
 * 关=只 Initialize+基线复位, 用固定 IDAC(配置/手动值)换取稳定灵敏度范围(修某些通道自动校准发散 railed)。
 * "校准"命令(SENSOR_CMD_CALIBRATE)始终执行一次显式校准, 不受本开关影响(满足"点校准=单次校准"需求)。 */
volatile bool g_auto_calibrate = true;

/* CSD 处理模式：SCAN_MODE_AUTO(自动校准/标准完整处理) / SCAN_MODE_SEMI(半自动手动)。 */
volatile uint8_t scan_mode = SCAN_MODE_AUTO;
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
 *   disabled_widgets_force_highz() 显式复位禁用 widget 的全部电极。
 * ★注意★ 只有 Cy_CapSense_Init(cy_capsense_control.c:147) 会把全部 widget 的 ENABLE|WORKING
 *   重新置起; Initialize/Enable 只清 ACTIVE 位(processing.c:126)不动 ENABLE。故需在 Init 之后
 *   重放本位图(见 ch_enable_restore)。 */
#define PROVISION_TIMEOUT_MS             (3000u)
/* PSoC 启动时先禁用全部 widget。RP 下发完整 enabled 位图后再用既有 APPLY 作为完成屏障；
 * 旧 RP 未发送该位图时，超时回退为全启用。禁用电极的 High-Z 由显式 helper 保证，
 * 不依赖 Host 可配置的 CSD inactive 状态。 */
volatile uint64_t g_ch_enabled = 0u;
volatile uint64_t g_provision_enable_seen = 0u;
volatile bool g_provision_pending = true;
volatile bool g_provision_apply_release = false;
/* 启用态刚被 SPI 改动、尚未由主循环落到中间件的通道位图(ISR 只置位, 重活在主循环)。 */
volatile uint64_t ch_enable_dirty = 0u;

/* 全局 CSD 配置 RAM 影子：生成的 cy_capsense_commonConfig 是 const(flash)不可运行时改，
 * 故 init 前把它拷到 RAM 并把 cy_capsense_context.ptrCommonConfig 指向此副本；
 * 之后 SET_GLOBAL 改此副本、APPLY 重初始化时中间件从 ptrCommonConfig 重算内部预计算而生效。 */
cy_stc_capsense_common_config_t g_common_cfg_ram;

volatile idac_gain_lock_t g_idac_lock;


void idac_lock_clear(void)
{
    uint32_t i;
    g_idac_lock.mask = 0u;
    for (i = 0u; i < LNK_CHANNEL_COUNT; i++) { g_idac_lock.gain[i] = 0u; }
}

/* 把锁定通道的增益档写回 widgetContext。target 为 0..35 时只恢复该 widget，
 * LNK_CH_ALL 时恢复全部锁定 widget。CSDv2 会在下一次 ScanWidget 内部装载该 widget 的
 * idacGainIndex，故这里绝不调用全局 Initialize/SsInitialize 破坏其它 widget 的运行状态。 */
bool idac_lock_restore(uint8_t target)
{
    const uint32_t first = (target < LNK_CHANNEL_COUNT) ? target : 0u;
    const uint32_t last = (target < LNK_CHANNEL_COUNT) ? ((uint32_t)target + 1u) : LNK_CHANNEL_COUNT;
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
 * 全局档顶替: 解出的 idacMod 属于全局档, 事后 idac_lock_restore 又把档改回用户值, 二者不自洽
 * (频率自适应尤其明显: 每一档试探的通过/失败判定都用的全局档, 找到的分频对不上该通道的档)。
 * 故 per-channel 校准统一走本助手: 锁定通道把全局起点档临时替换为该通道的锁定档, 校准后还原,
 * 使中间件的强制复位正好落在该通道自己的档上(语义 = 该通道的起点档)。未锁定通道零行为变化。 */
bool calibrate_widget_locked(uint32_t ch)
{
    bool ok;
    uint8_t saved_init = g_common_cfg_ram.csdIdacGainInitIndex;
    const bool locked = (ch < LNK_CHANNEL_COUNT) &&
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
void idac_lock_reapply(void)
{
    (void)idac_lock_restore(LNK_CH_ALL);
}

/* 建立全局配置 RAM 影子并重定向 ptrCommonConfig。必须在 Cy_CapSense_Init 前调用。 */
void initialize_common_cfg_shadow(void)
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
void cmd_set_global(uint8_t gparam_id, uint32_t value)
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
        case GPARAM_IDAC_GAIN_INIT: if (value <= 6u)   { g_common_cfg_ram.csdIdacGainInitIndex = (uint8_t)value; idac_lock_clear(); } break;
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

uint32_t cmd_get_global(uint8_t gparam_id)
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
        /* ★这 5 个 id 的语义已换代为 LINK v2 的链路计数★ id 保留是为了不动上位机的全局项列表;
         * 内容改为如实返回 lnk_diag(此前它们指向的是几个与名字早已不符的调试槽)。 */
        case GPARAM_DBG_RX_FRAMES:    return lnk_diag.rx_ok;
        case GPARAM_DBG_RX_BAD_MAGIC: return lnk_diag.rx_reject;
        /* 这两项在 v3 里结构上不可能发生(CS ISR 与 TX 重对齐已删, 见 I1), 恒 0。 */
        case GPARAM_DBG_CS_RESYNC:    return 0u;
        case GPARAM_DBG_TX_ARM:       return lnk_diag.tx_stale;
        case GPARAM_DBG_RX_LEFTOVER:  return 0u;
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
void normalize_widget_params(void)
{
    uint32_t w;
    for (w = 0u; w < LNK_CHANNEL_COUNT; w++)
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

static widget_hw_t g_widget_hw[LNK_CHANNEL_COUNT];

void widget_hw_save(void)
{
    uint32_t w;
    for (w = 0u; w < LNK_CHANNEL_COUNT; w++)
    {
        g_widget_hw[w].resolution = cy_capsense_tuner.widgetContext[w].resolution;
        g_widget_hw[w].sns_clk    = cy_capsense_tuner.widgetContext[w].snsClk;
    }
}

void widget_hw_restore(void)
{
    uint32_t w;
    for (w = 0u; w < LNK_CHANNEL_COUNT; w++)
    {
        cy_capsense_tuner.widgetContext[w].resolution = g_widget_hw[w].resolution;
        cy_capsense_tuner.widgetContext[w].snsClk     = g_widget_hw[w].sns_clk;
    }
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
bool cmd_set_param(uint8_t ch, uint8_t param_id, uint32_t value)
{
    if (ch >= LNK_CHANNEL_COUNT) return false;
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

uint32_t cmd_get_param(uint8_t ch, uint8_t param_id)
{
    if (ch >= LNK_CHANNEL_COUNT) return 0u;
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
        case PARAM_ENABLED:       return ch_is_enabled(ch) ? 1u : 0u;
        default: return 0u;
    }
}

/* 把 g_ch_enabled 位图重放到中间件的 widget 状态位。
 * ★何时必须调★ 只有 Cy_CapSense_Init 会把全部 widget 的 ENABLE|WORKING 重新置起
 * (cy_capsense_control.c:147), 所以启动 Init 之后与 GLOBAL_COMMIT 的 Init 之后各调一次即可 ——
 * 与 widget_hw_restore / idac_lock_restore 完全同一模式(都是"抵消 Init 的 ROM 重铺")。
 * 这里直写 status 位而不调 Cy_CapSense_SetWidgetStatus: 后者附带 SwitchSensingMode(UNDEFINED),
 * 在"Init 之后紧接 Initialize"的序列里会白白多一次 CSD 模式来回。 */
void ch_enable_restore(void)
{
    uint32_t w;
    for (w = 0u; w < LNK_CHANNEL_COUNT; w++)
    {
        cy_stc_capsense_widget_context_t * wc = &cy_capsense_tuner.widgetContext[w];
        if (ch_is_enabled(w))
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
void disabled_widgets_force_highz(void)
{
    uint32_t widget;
    for (widget = 0u; widget < cy_capsense_context.ptrCommonConfig->numWd; widget++)
    {
        const cy_stc_capsense_widget_config_t * cfg = &cy_capsense_context.ptrWdConfig[widget];
        uint32_t sensor;
        if (ch_is_enabled(widget) || (cfg->senseMethod != CY_CAPSENSE_CSD_GROUP)) { continue; }
        for (sensor = 0u; sensor < cfg->numSns; sensor++)
        {
            (void)Cy_CapSense_SetPinState(widget, sensor, CY_CAPSENSE_HIGHZ, &cy_capsense_context);
        }
    }
}

/* CSDInitialize applies the configurable inactive state to every electrode. Preparing CSD
 * before a calibration or ScanAllWidgets keeps disabled electrodes High-Z without ISR work. */
void prepare_csd_mode(void)
{
    if (cy_capsense_context.ptrActiveScanSns->currentSenseMethod != CY_CAPSENSE_CSD_GROUP)
    {
        (void)Cy_CapSense_SwitchSensingMode(CY_CAPSENSE_CSD_GROUP, &cy_capsense_context);
    }
    disabled_widgets_force_highz();
}

/* 只初始化【启用】通道的基线。替代 Cy_CapSense_InitializeAllBaselines ——
 * 后者(cy_capsense_filter.c:407)不查 enable, 会把禁用通道的基线设成它那份陈旧 raw,
 * 白做 36 次无意义写入, 也让"禁用通道不参与任何处理"这条约束出现例外。 */
void initialize_enabled_baselines(void)
{
    uint32_t w;
    for (w = 0u; w < LNK_CHANNEL_COUNT; w++)
    {
        if (ch_is_enabled(w))
        {
            Cy_CapSense_InitializeWidgetBaseline(w, &cy_capsense_context);
        }
    }
}

/* 主循环(NOT_BUSY 窗口)落实 SPI 收到的启用/禁用请求。
 * 禁用: widget-enable 将其排除出扫描；SetWidgetStatus 会退出 CSD 并按全局 inactive 配置重置
 *       所有电极，故每次状态变更后立即切回 CSD 并显式恢复禁用电极 High-Z。
 * 启用: 恢复 enable 位后只给该 widget 重建 IDAC 与基线，绝不扰动其它通道。 */
void ch_enable_apply(void)
{
    uint64_t pending;
    uint32_t w;
    if (g_provision_pending) { return; }
    uint32_t st = Cy_SysLib_EnterCriticalSection();
    pending = ch_enable_dirty;
    ch_enable_dirty = 0u;
    Cy_SysLib_ExitCriticalSection(st);
    if (pending == 0u) { return; }

    for (w = 0u; w < LNK_CHANNEL_COUNT; w++)
    {
        if ((pending & ((uint64_t)1u << w)) == 0u) { continue; }
        /* 每个通道一次: 兼容回退会把整片 36 通道一起打进 pending, 那就是 36 次校准(十几秒)。
         * 本函数没有被拆成状态机(它不是主机可见的重操作), 故至少保证链路每通道被泵一次。 */
        (void)lnk_rx_drain();
        const bool enable = ch_is_enabled(w);
        (void)Cy_CapSense_SetWidgetStatus(w, CY_CAPSENSE_WD_ENABLE_MASK,
                                          enable ? CY_CAPSENSE_WD_ENABLE_MASK : 0u,
                                          &cy_capsense_context);
        prepare_csd_mode();
        if (enable)
        {
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
            if (g_auto_calibrate) { (void)calibrate_widget_locked(w); }
#endif
            (void)idac_lock_restore((uint8_t)w);
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

/* 主循环阶段: 兼容回退(注释见下, 自 main() 逐字搬来)。 */
void provision_timeout_fallback(void)
{
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
        lnk_st_upd(0u, LNK_ST_PROVISION);
    }
}

/* 启动复位: 长操作请求位 + 启用位图/provisioning 闸门(原 main() 启动序列的对应段)。 */
void csd_params_reset(void)
{
    measure_cp_pending = false;
    measure_cp_active = false;
    global_apply_pending = false;
    quick_apply_pending = false;
    quick_apply_ch = LNK_CH_ALL;
    quick_apply_gain = 0u;
    quick_apply_div = 1u;
    calibrate_pending = false;
    baseline_reset_pending = false;
    calibrate_ch = LNK_CH_ALL;
    baseline_ch = LNK_CH_ALL;
    /* PSoC 无状态: 上电先禁用全部 widget。新 RP 会完整下发 enabled 位图并以 APPLY 放行；
     * 未升级 RP 在 3 秒后走兼容全启用兜底。 */
    g_ch_enabled = 0u;
    g_provision_enable_seen = 0u;
    g_provision_pending = true;
    g_provision_apply_release = false;
    ch_enable_dirty = 0u;
}
