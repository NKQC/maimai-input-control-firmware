/* csd_ops.c —— 长操作串行状态机, 见 csd_ops.h。逐字自 main.c 搬入。 */
#include "csd_ops.h"
#include "csd_params.h"
#include "csd_autotune.h"
#include "lnk_wire.h"
#include "diag_export.h"
#include "app_tick.h"
#include "cybsp.h"
#include "cycfg_capsense.h"


typedef struct
{
    bool     active;      /* 跨轮保持: OP_BUSY 的真相源之一 */
    uint8_t  phase;
    uint8_t  ch;          /* 下一个待处理通道 */
    bool     auto_cal;    /* AUTO 且开自动校准: 先 Initialize 再校准全部启用通道 */
    bool     recal;       /* 非 AUTO 分支是否补校准(= 进入时的 g_auto_calibrate) */
    uint64_t dirty;       /* 非 AUTO 分支的脏通道快照(v2 是整段原子执行, 这里取快照等价) */
} apply_task_t;
static apply_task_t _apply_task;

typedef struct
{
    bool    active;
    uint8_t phase;
    uint8_t ch;           /* 下一个待处理通道 */
    uint8_t last;         /* 终止通道(不含) */
    uint8_t target;       /* 请求语义: 0..35 或 LNK_CH_ALL(收尾时决定锁定档/基线的范围) */
} cal_task_t;
static cal_task_t _cal_task;

typedef struct
{
    bool    active;
    uint8_t phase;
    uint8_t ch;
} cp_task_t;
static cp_task_t _cp_task;

/* 同功能组统一用 .clear() 复位, 不散着逐个赋值。 */
static inline void _apply_task_clear(void)
{
    _apply_task.active = false;
    _apply_task.phase = OP_PHASE_IDLE;
    _apply_task.ch = 0u;
    _apply_task.auto_cal = false;
    _apply_task.recal = false;
    _apply_task.dirty = 0u;
}
static inline void _cal_task_clear(void)
{
    _cal_task.active = false;
    _cal_task.phase = OP_PHASE_IDLE;
    _cal_task.ch = 0u;
    _cal_task.last = 0u;
    _cal_task.target = LNK_CH_ALL;
}
static inline void _cp_task_clear(void)
{
    _cp_task.active = false;
    _cp_task.phase = OP_PHASE_IDLE;
    _cp_task.ch = 0u;
}

bool op_serial_active(void)
{
    return _apply_task.active || _cal_task.active || _cp_task.active || at_active();
}
/* ★"处理中"已并入 lnk_st 的 LNK_ST_OP_BUSY★ ISR 受理重操作即置位, 主循环做完清位。
 * 因为**每一份响应都带 st**, 主机不必再为"设备忙不忙"单独发命令 —— v1 靠高频轮询 GET_STATS
 * 取 busy, 反过来抢占了 PSoC 主循环的空窗。独立的 g_op_busy 变量随之退役。 */
/* 每通道最近一次寄生电容测量值(fF)，0xFFFFFF=失败/未测量。 */
volatile uint32_t cp_value[LNK_CHANNEL_COUNT];
/* BIST 的 Cp 上限钳位值(CY_CAPSENSE_BIST_CP_MAX_VALUE, cy_capsense_selftest_v2.c:43)。读到它说明
 * 结果已溢出量程 —— 这才是"短路/异常大电容"的真故障判据, 见 MEASURE_CP 处说明。 */
#define CP_BIST_OVERRANGE_FF             (400000u)

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
void scan_then_initialize_baselines(void)
{
    uint32_t t0;
    if (!any_ch_enabled()) { return; }   /* 无启用通道: 无扫描可做, 也无基线可立 */
    /* ★两处忙等里必须泵链路★ 各最坏 500ms, 合起来就是一整秒的"主循环出局"。它们是 GLOBAL_COMMIT /
     * APPLY / MEASURE_CP 收尾共用的路径, 不泵的话每次重配都要让主机干等一秒。 */
    t0 = g_ms_tick;
    while ((CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) &&
           ((uint32_t)(g_ms_tick - t0) < SCAN_SETTLE_BUDGET_MS)) { (void)lnk_rx_drain(); }
    if (CY_CAPSENSE_STATUS_SUCCESS == Cy_CapSense_ScanAllWidgets(&cy_capsense_context))
    {
        t0 = g_ms_tick;
        while ((CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) &&
               ((uint32_t)(g_ms_tick - t0) < SCAN_SETTLE_BUDGET_MS)) { (void)lnk_rx_drain(); }
    }
    initialize_enabled_baselines();
}

/* ★_recalibrate_dirty_channels() / _calibrate_enabled_channels() 已删除★ 它们各是一个"一次调用
 * 内跑完 36 通道校准"的循环(最坏十几秒), 现已摊平进 APPLY 的逐通道状态机(_apply_step 的
 * OP_PHASE_CH), 通道集合与收尾动作逐位照搬:
 *   · AUTO 且开自动校准 ⇒ 全部启用通道, 收尾 idac_dirty_mask=0 + idac_lock_reapply();
 *   · 否则(开自动校准时) ⇒ 只补脏且启用的通道, 收尾 idac_lock_restore(ALL) + idac_dirty_mask=0。
 * 禁用通道一律跳过的理由不变: 校准会连接电极并扫描该 widget, 与"关闭 ⇒ 电极恒高阻"直接冲突,
 * 且它的 IDAC 结果毫无用处。 */


/* ================== 四个长操作的逐通道串行推进(每次调用最多做一个原子步) ==================
 * 原子步 = 一次 calibrate_widget_locked / 一个电极的 BIST 测量。纯 O(1) 的状态迁移不占一步,
 * 直接落到下一轮(一轮空转只花一次主循环, 相对 200ms 级的原子步可忽略)。 */

/* ---- APPLY ----
 * v2 的分支结构逐位照搬(见被删除的 _calibrate_enabled_channels / _recalibrate_dirty_channels):
 *   auto_cal  : Initialize+prepare → 全部启用通道逐个校准 → dirty=0 + 锁定档回写 → 立基线
 *   非 auto_cal: (开自动校准时)只补脏通道 → 锁定档回写 + dirty=0 → Initialize+prepare → 立基线 */
static void _apply_step(void)
{
    switch (_apply_task.phase)
    {
        case OP_PHASE_PRE:
            spi_dbg.stage = MLOOP_STAGE_APPLY;
            /* ★放行屏障: 启用位图在本次重配【之前】落到中间件★
             * provisioning 期间 ch_enable_apply 是空转的(它见 g_provision_pending 直接返回),
             * 于是位图只存在于 g_ch_enabled 里、中间件的 widget ENABLE 位还全是关。若不在此处
             * 重放, 下面的校准/基线跑完仍然一个通道都不会被扫, 要等下一轮 ch_enable_apply 逐
             * 通道 SetWidgetStatus + 再校准一次 —— 那就是"provision 后 APPLY 12.4s"的来源
             * (同一批通道被校准两遍)。这里重放并吃掉脏位, 本次 APPLY 的校准+基线即为终态。 */
            if (g_provision_apply_release)
            {
                ch_enable_dirty = 0u;
                ch_enable_restore();
                prepare_csd_mode();
            }
            if (_apply_task.auto_cal)
            {
                /* Do not call Enable here: it performs Initialize then starts ScanAllWidgets
                 * internally, leaving no application point to restore disabled electrodes. */
                spi_dbg.stage = MLOOP_STAGE_APPLY_INIT;
                (void)Cy_CapSense_Initialize(&cy_capsense_context);
                prepare_csd_mode();
            }
            _apply_task.ch = 0u;
            _apply_task.phase = OP_PHASE_CH;
            break;

        case OP_PHASE_CH: {
            uint32_t w = _apply_task.ch;
            spi_dbg.stage = MLOOP_STAGE_APPLY_RECAL;
            /* 不需要校准的通道是 O(1) 判定, 一轮里连着跳完; 真正的校准每轮只做一个。 */
            while (w < LNK_CHANNEL_COUNT)
            {
                const bool want = _apply_task.auto_cal
                                    ? ch_is_enabled(w)
                                    : (_apply_task.recal && ch_is_enabled(w) &&
                                       ((_apply_task.dirty & ((uint64_t)1u << w)) != 0u));
                if (want) { break; }
                w++;
            }
            if (w >= LNK_CHANNEL_COUNT)
            {
                _apply_task.ch = (uint8_t)LNK_CHANNEL_COUNT;
                _apply_task.phase = OP_PHASE_FIN;
                break;
            }
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
            (void)calibrate_widget_locked(w);   /* 锁定通道按其自身增益档校准 */
#endif
            _apply_task.ch = (uint8_t)(w + 1u);
            break;
        }

        case OP_PHASE_FIN:
        default:
            if (_apply_task.auto_cal)
            {
                idac_dirty_mask = 0u;
                idac_lock_reapply();
            }
            else
            {
                if (_apply_task.recal)
                {
                    /* 校准把增益档拉回全局起点档, 这里写回用户锁定值; 紧随的 Initialize 下到硬件。 */
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
                    (void)idac_lock_restore(LNK_CH_ALL);
#endif
                    idac_dirty_mask = 0u;
                }
                /* Fixed-IDAC apply still resets every IO through Initialize; restore the disabled
                 * electrical contract before resuming scan scheduling. */
                spi_dbg.stage = MLOOP_STAGE_APPLY_INIT;
                (void)Cy_CapSense_Initialize(&cy_capsense_context);
                prepare_csd_mode();
            }
            spi_dbg.stage = MLOOP_STAGE_APPLY_BASELINE;
            scan_then_initialize_baselines();
            /* APPLY 是 RP provisioning 的 FIFO 完成屏障: 只有重配真的执行完才开闸。 */
            if (g_provision_apply_release)
            {
                g_provision_apply_release = false;
                g_provision_pending = false;
                lnk_st_upd(0u, LNK_ST_PROVISION);
            }
            _apply_task_clear();
            break;
    }
}

/* ---- CALIBRATE ----
 * ★逐通道校准, 不用 CalibrateAllWidgets★ 后者对全部 widget 一律用全局起点增益档
 * csdIdacGainInitIndex, 而本面板 36 段的 Cp 跨度极大(实测 ch8≈22pF、ch35≈138pF), 各通道的增益档
 * 本就不该相同 —— 用户逐通道设过的档位记在 g_idac_lock 里, 被统一按全局档校准后等于全部作废。
 * 故走 calibrate_widget_locked(): 校准该通道时临时把全局起点档替换为它自己的锁定档, 校准完还原。 */
static void _cal_step(void)
{
    switch (_cal_task.phase)
    {
        case OP_PHASE_PRE:
            spi_dbg.stage = MLOOP_STAGE_CALIBRATE;
#if !(defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
            (void)Cy_CapSense_Initialize(&cy_capsense_context);
            prepare_csd_mode();
#endif
            _cal_task.phase = OP_PHASE_CH;
            break;

        case OP_PHASE_CH: {
            uint32_t w = _cal_task.ch;
            spi_dbg.stage = MLOOP_STAGE_CALIBRATE;
            /* ★禁用通道一律跳过★ 校准要连接电极并扫描该 widget, 与"关闭 ⇒ 电极恒高阻"冲突;
             * 单通道请求落在禁用通道上就是一次违背语义的操作, 直接不做。 */
            while ((w < _cal_task.last) && !ch_is_enabled(w)) { w++; }
            if (w >= _cal_task.last)
            {
                _cal_task.ch = _cal_task.last;
                _cal_task.phase = OP_PHASE_FIN;
                break;
            }
#if (defined(CY_CAPSENSE_CSD_CALIBRATION_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
            (void)calibrate_widget_locked(w);
#endif
            _cal_task.ch = (uint8_t)(w + 1u);
            break;
        }

        case OP_PHASE_FIN:
        default:
            /* 校准会把增益档拉回起点档; 只恢复本次目标范围。CSDv2 在后续扫描时按 widgetContext
             * 装载 IDAC, 故不调用全局 Initialize, 也不重置其它 widget 状态。 */
            (void)idac_lock_restore(_cal_task.target);
            if (_cal_task.target < LNK_CHANNEL_COUNT)
            {
                if (ch_is_enabled((uint32_t)_cal_task.target))
                {
                    Cy_CapSense_InitializeWidgetBaseline((uint32_t)_cal_task.target,
                                                         &cy_capsense_context);
                }
            }
            else
            {
                initialize_enabled_baselines();
            }
            _cal_task_clear();
            break;
    }
}

/* ---- MEASURE_CP ---- */
static void _cp_step(void)
{
    switch (_cp_task.phase)
    {
        case OP_PHASE_PRE: {
            uint32_t w;
            spi_dbg.stage = MLOOP_STAGE_MEASURE_CP;
            /* 先统一标为失败; BIST 可用的 Cp 结果随后覆盖。active 期间 CP_GET 始终返回 0。 */
            for (w = 0u; w < LNK_CHANNEL_COUNT; w++) { cp_value[w] = 0xFFFFFFu; }
            _cp_task.ch = 0u;
            _cp_task.phase = OP_PHASE_CH;
            break;
        }

        case OP_PHASE_CH: {
            uint32_t w = _cp_task.ch;
            spi_dbg.stage = MLOOP_STAGE_MEASURE_CP;
            /* BIST 测量会把该电极接到测量回路上 —— 对已禁用(要求恒高阻)的通道不能做,
             * 其 cp_value 保持 0xFFFFFF"未测量"。 */
            while ((w < LNK_CHANNEL_COUNT) && !ch_is_enabled(w)) { w++; }
            if (w >= LNK_CHANNEL_COUNT)
            {
                _cp_task.ch = (uint8_t)LNK_CHANNEL_COUNT;
                _cp_task.phase = OP_PHASE_FIN;
                break;
            }
#if (defined(CY_CAPSENSE_TST_SNS_CAP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN))
            {
                uint32_t v = 0u;
                cy_en_capsense_bist_status_t status;
                /* ★单个电极测量期间必须屏蔽 CSD 中断, 也只在这一步屏蔽★
                 * BIST 是**轮询式**的: Cy_CapSense_BistWaitEndOfScan(cy_capsense_selftest_v2.c:4489)
                 * 死等 ptrCsdBase->INTR 的 SAMPLE 位。而本工程把 CSD 中断挂到了 NVIC
                 * (capsense_isr → Cy_CapSense_InterruptHandler), 转换一结束 ISR 先跑, 顺手把 INTR
                 * 清掉 —— 轮询循环就永远看不到那一位, 耗尽 watchdog 计数后返回 TIMEOUT。这是一场
                 * 竞态: 表现为"每次测电容都有几个通道随机失败, 失败集合每次都不一样"(实测三轮分别是
                 * {15,20,26,31} / {7,14,15,26,34} / {1,3,5,11,26,28,30}), 与电极本身无关, 重测也
                 * 救不回来。第二重危害: ISR 里跑的是正常扫描的后处理, 会把 BIST 的转换结果当成 raw
                 * 写进 sns context, 污染基线与差值。
                 * ★v3 把屏蔽窗口收窄到一个电极★ v2 是整段 36 通道一直关着(最坏几百毫秒跨全过程),
                 * 而这里每轮只关一次转换: 关闭窗口内不会有正常扫描发生(长操作期间主循环不启动扫描),
                 * 所以逐电极开关与整段关掉的效果等价, 但不再有"长时间屏蔽"这件事。 */
                NVIC_DisableIRQ(CYBSP_CSD_IRQ);
                status = Cy_CapSense_MeasureCapacitanceSensor(w, 0u, &v, &cy_capsense_context);
                NVIC_ClearPendingIRQ(CYBSP_CSD_IRQ);
                NVIC_EnableIRQ(CYBSP_CSD_IRQ);
                /* ★不能把 LOW_LIMIT / HIGH_LIMIT 当成故障★
                 * 中间件的这两个"故障"判据是**它自己那把尺子的量程边界**, 不是电极的物理结论
                 * (cy_capsense_selftest_v2.c:2629 BistMeasureCapacitanceSensor):
                 *   - 合法 raw 窗口只有满量程的 7.5%~45%(MIN/MAX_RAW_PROMILLE = 75/450);
                 *   - 而它只有 4 个测点, 相邻测点之间是 4 倍跳变, 单个测点能覆盖的 Cp 只有 6 倍。
                 *   ⇒ 4 倍步长 > 6 倍窗口的余量很薄, 落在缝隙附近的 Cp 无论走哪一档都贴着窗口边,
                 *     于是同一块板连测多轮, "失败"的通道集合每轮都不一样, 而它们的 Cp 与相邻通道
                 *     毫无区别(实测 CH3=69pF 报失败、CH17=143pF 却成功)。
                 * 关键事实: 这两条路上 **Cp 已经算出来并写回**了(同文件 2722 行: 只有 TIMEOUT 才不
                 * 写值), 只是 raw 不在它偏爱的窗口内、精度略差。判成"测量失败"是拿量程当故障。
                 * ★真正的故障判据★ 只保留物理上说不通的两种: 没有值写回(v 保持 0), 或值被钳在
                 * CY_CAPSENSE_BIST_CP_MAX_VALUE(400pF, 溢出/短路)。0xFFFFFF 留给协议的失败标记,
                 * 故撞上该值的真实读数饱和到 0xFFFFFE。 */
                if ((v != 0u) &&
                    ((CY_CAPSENSE_BIST_SUCCESS_E == status) ||
                     (CY_CAPSENSE_BIST_LOW_LIMIT_E == status) ||
                     (CY_CAPSENSE_BIST_HIGH_LIMIT_E == status)) &&
                    (v < CP_BIST_OVERRANGE_FF))
                {
                    cp_value[w] = (v >= 0xFFFFFFu) ? 0xFFFFFEu : v;
                }
            }
#endif
            _cp_task.ch = (uint8_t)(w + 1u);
            break;
        }

        case OP_PHASE_FIN:
        default:
            spi_dbg.stage = MLOOP_STAGE_MEASURE_CP;
#if (defined(CY_CAPSENSE_TST_SNS_CAP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN))
            /* BIST selects its own inactive state for every measurement; restore the disabled-channel
             * electrical contract before continuing. */
            disabled_widgets_force_highz();
            /* BIST leaves the CSD block in its private configuration. Reinitialize the middleware,
             * then prepare CSD so disabled electrodes are High-Z before regular scanning resumes. */
            (void)Cy_CapSense_Initialize(&cy_capsense_context);
            prepare_csd_mode();
            /* 重扫要靠 CSD 中断推进扫描链; 逐电极那步已经成对开关过, 这里只清掉 BIST 残留 pending。 */
            NVIC_ClearPendingIRQ(CYBSP_CSD_IRQ);
            NVIC_EnableIRQ(CYBSP_CSD_IRQ);
            /* BIST 期间 raw 与正常扫描口径无关 —— 必须先真扫一遍再立基线, 否则整块面板的基线被
             * 钉在 BIST 残留值上并因"判为按下"永久冻结(见 scan_then_initialize_baselines)。 */
            scan_then_initialize_baselines();
#endif
            measure_cp_active = false;
            _cp_task_clear();
            break;
    }
}

/* 推进当前那一个长操作一步。同一时刻只有一个 active(受理点按 v2 的先后次序挑选)。 */
void op_serial_step(void)
{
    if (_cp_task.active)         { _cp_step(); }
    else if (_apply_task.active) { _apply_step(); }
    else if (_cal_task.active)   { _cal_step(); }
    else if (at_active())        { at_step(); }
}

/* 主循环阶段(以下各段自 main() 逐字搬来, 原地的 continue 变成 return true)。 */
bool op_begin_measure_cp(void)
{
    /* MEASURE_CP：逐电极 BIST 寄生电容测量(fF)。测量会重配 CSD HW，完成后恢复扫描配置。
     * ★受理即转交状态机★ 逐电极推进(见 _cp_step), 本轮不再往下走: 后面那些 pending 必须排在
     * 它完成之后, 否则就成了"两个重操作逐通道交错", 与 v2 的先后语义不符。 */
    if (measure_cp_pending)
    {
        measure_cp_pending = false;
        measure_cp_active = true;   /* 覆盖 BIST 与其后的 CSD 恢复: 期间 CP_GET 恒回 0 */
        _cp_task.active = true;
        _cp_task.phase = OP_PHASE_PRE;
        _cp_task.ch = 0u;
        return true;
    }
    return false;
}

void op_global_apply_step(void)
{

    /* 全局 CSD 配置改动：完整 Init + Initialize 重初始化，重算 inactive_sns/IDAC/MFS
     * 的内部预计算；随后显式恢复禁用电极 High-Z。轻量 APPLY 不重算 → 会坏扫描。 */
    if (global_apply_pending)
    {
        spi_dbg.stage = MLOOP_STAGE_GLOBAL_APPLY;
        global_apply_pending = false;
        /* ★阻塞阶段前后各泵一次★ 本段没有逐通道循环可拆(Init/Initialize 是中间件的单次调用),
         * 但内部 scan_then_initialize_baselines 的两处忙等已各自泵链路。 */
        (void)lnk_rx_drain();
        /* 全局配置变更应用: Init 从 ptrCommonConfig(RAM 影子)重算内部预计算
         * (含 csdInactiveSnsDm/HSIOM)，随后 Initialize 写回硬件状态。
         * ★不再调 Cy_CapSense_DeInit★: 实测运行时 DeInit→Init→Enable 会把扫描速率从 ~180Hz
         * 掉到 ~15Hz(疑似 DeInit 未复位时钟分频, 再 Init 残留慢时钟); 仅 Init→Initialize
         * 同样重算全局预计算且保持满速。 */
        /* Init 会从 ROM 生成配置重铺 widgetContext, 打掉当前生效的 resolution/snsClk
         * (启动归一的 32, 或用户 SET_PARAM / AUTO_TUNE 的逐通道值) → 见 widget_hw_save 注释。 */
        widget_hw_save();
        (void)Cy_CapSense_Init(&cy_capsense_context);
        widget_hw_restore();
        /* Cy_CapSense_Init 会把全部 widget 的 ENABLE|WORKING 重新置起
         * (cy_capsense_control.c:147) —— 不重放位图, 用户关掉的通道会在每次"全局应用"
         * 后偷偷复活并重新参与扫描(电极离开 High-Z)。 */
        ch_enable_restore();
        /* Init 会按 ptrCommonConfig 重铺 widgetContext 增益档 → 先把用户锁定值写回,
         * 再由紧随其后的 Initialize 一并下到硬件(不额外触发校准)。 */
        (void)idac_lock_restore(LNK_CH_ALL);
        /* ★配置更新只重算内部预计算, 不再自动校准/频率下探(转一圈)★:
         * 恒走轻量 Initialize+基线路径, 沿用现有(上次手动校准的)IDAC。
         * Enable 的自动校准(SmartSense 可含频率自适应)耗时长会阻塞→掉 USB, 且用户要求
         * "频率自适应探测只能手动触发"。故校准/频率下探仅由显式 CALIBRATE / AUTO_TUNE 命令触发,
         * 配置改动(inactive_sns/IDAC/MFS)本身即时生效但不重扫校准。 */
        (void)Cy_CapSense_Initialize(&cy_capsense_context);
        prepare_csd_mode();
        /* inactive_sns/IDAC/MFS 刚变 ⇒ 旧 raw 与新配置差一个量级, 不能直接当基线。 */
        scan_then_initialize_baselines();
        (void)lnk_rx_drain();
    }
}

bool op_begin_apply(void)
{

    /* APPLY 指令：重新初始化扫描硬件使硬件参数(分辨率/时钟/IDAC)生效, 并按分支重校准。
     * ★受理即转交状态机★ 逐通道推进(见 _apply_step): v2 这一整段最坏 12~13s, 主循环全程出局。 */
    if (apply_pending)
    {
        apply_pending = false;
        _apply_task.active = true;
        _apply_task.phase = OP_PHASE_PRE;
        _apply_task.ch = 0u;
        /* 分支与通道集合在受理这一刻定下来: v2 是整段原子执行的, 中途不会看到 scan_mode /
         * g_auto_calibrate / idac_dirty_mask 的新值, 取快照才等价。 */
        _apply_task.auto_cal = ((scan_mode == SCAN_MODE_AUTO) && g_auto_calibrate);
        _apply_task.recal    = g_auto_calibrate;
        _apply_task.dirty    = idac_dirty_mask;
        return true;
    }
    return false;
}

void op_quick_apply_step(void)
{

    /* QUICK_APPLY(Sweep 专用)：ISR 已保存本格的 gain/div；只在 NOT_BUSY 窗口原子写入
     * widgetContext，再 Initialize 让参数下到硬件。绝不校准/基线，禁用电极由
     * prepare_csd_mode() 继续强制 High-Z。 */
    if (quick_apply_pending)
    {
        /* ★不再需要临界区★ 参数的唯一另一个写者是命令分发, 而 v3 起它也跑在主循环
         * (lnk_rx_drain)上, 与这里不可能交错。 */
        const uint8_t qa_target = quick_apply_ch;
        const uint8_t qa_gain = quick_apply_gain;
        const uint8_t qa_div = quick_apply_div;
        quick_apply_pending = false;
        quick_apply_ch = LNK_CH_ALL;
        quick_apply_gain = 0u;
        quick_apply_div = 1u;

        spi_dbg.stage = MLOOP_STAGE_QUICK_APPLY;
        if (qa_target < LNK_CHANNEL_COUNT)
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
            prepare_csd_mode();
            /* 本次已经让 gain/div 生效；普通 APPLY 不得再为该格补跑校准。 */
            idac_dirty_mask &= ~bit;
        }
    }
}

bool op_begin_calibrate(void)
{

    /* CALIBRATE：真正的 IDAC 重校准(把 raw 拉回目标, 修 railed), 再复位基线。
     * 与 APPLY 区分——APPLY 只重配/re-init, 不重算 IDAC; 半自动手动下"校准"必须走这里
     * 才有效(否则 raw 一直卡满量程 diff=0)。CalibrateAllWidgets 需校准使能。 */
    if (calibrate_pending)
    {
        /* ★单通道语义端到端透传★: target < 36 ⇒ 只校准该 widget 并只初始化该 widget 的基线
         * (禁止 InitializeAllBaselines 把其它 35 个通道的基线一并冲掉)。
         * UI 的"全通道校准"改由 host 串行队列逐通道下发, 故 0xFF 分支只保留兼容入口
         * (恢复默认 / AUTO_CALIBRATE_EN 上升沿等固件内部触发仍需要它)。
         * ★受理即转交状态机★ 全通道分支是 36 次 calibrate_widget_locked(约 8~10s)。 */
        const uint8_t cal_target = calibrate_ch;
        calibrate_pending = false;
        calibrate_ch = LNK_CH_ALL;   /* 消费即复位: 内部触发(无 args)一律全通道语义 */
        _cal_task.active = true;
        _cal_task.phase = OP_PHASE_PRE;
        _cal_task.target = cal_target;
        _cal_task.ch   = (cal_target < LNK_CHANNEL_COUNT) ? cal_target : 0u;
        _cal_task.last = (cal_target < LNK_CHANNEL_COUNT)
                            ? (uint8_t)(cal_target + 1u) : (uint8_t)LNK_CHANNEL_COUNT;
        return true;
    }
    return false;
}

void op_baseline_reset_step(void)
{

    /* BASELINE_RESET：把基线重置到当前 raw(消除历史漂移), 不动 IDAC/参数。
     * baseline_ch < 36 ⇒ 只初始化该 widget 的基线(单通道"基线"不该动其它通道)。 */
    if (baseline_reset_pending)
    {
        const uint8_t bsln_target = baseline_ch;
        spi_dbg.stage = MLOOP_STAGE_BASELINE_RESET;
        baseline_reset_pending = false;
        baseline_ch = LNK_CH_ALL;   /* 消费即复位, 同 calibrate_ch */
        if (bsln_target < LNK_CHANNEL_COUNT)
        {
            /* 禁用通道没有"当前 raw"可言(发布值恒 0), 复位它的基线毫无意义 → 跳过。 */
            if (ch_is_enabled((uint32_t)bsln_target))
            {
                Cy_CapSense_InitializeWidgetBaseline((uint32_t)bsln_target, &cy_capsense_context);
            }
        }
        else
        {
            initialize_enabled_baselines();
        }
    }
}

void op_busy_release_step(void)
{

    /* ★处理中锁定解除★：本轮已把入队的重操作全部做完(且未被 ISR 追加新的)→ 清 OP_BUSY。
     * 主机看每一份响应自带的 st.OP_BUSY 由 1→0 即判定该重操作真实完成(替代盲等 sleep),
     * 不必再为此专门发命令轮询。 */
    /* ★串行状态机 active 也算忙★ 否则"每通道做完一轮"就会把 OP_BUSY 抖一次, 主机的完成
     * 判据(1→0)当场失真, 会把中途当成做完。走到这里其实已经保证不 active(active 时上面
     * 直接 continue 了), 条件仍显式写上 —— 这是主机语义的硬要求, 不该依赖控制流的巧合。 */
    if (!apply_pending && !quick_apply_pending && !calibrate_pending &&
        !baseline_reset_pending &&
        !global_apply_pending && !auto_tune_pending &&
        !measure_cp_pending && !measure_cp_active &&
        !op_serial_active())
    {
        lnk_st_upd(0u, LNK_ST_OP_BUSY);
    }
}

/* 启动复位: 三个任务结构 + Cp 结果表(原 main() 启动序列的对应段)。 */
void csd_ops_reset(void)
{
    _apply_task_clear();
    _cal_task_clear();
    _cp_task_clear();
    for (uint32_t channel = 0u; channel < LNK_CHANNEL_COUNT; channel++)
    {
        cp_value[channel] = 0xFFFFFFu;
    }
}
