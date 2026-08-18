/* csd_scan.c —— 扫描/处理/快照, 见 csd_scan.h。逐字自 main.c 搬入。 */
#include "csd_scan.h"
#include "csd_params.h"
#include "algo_engine.h"
#include "lnk_wire.h"
#include "diag_export.h"
#include "app_tick.h"
#include "cybsp.h"
#include <string.h>


#define CAPSENSE_INTR_PRIORITY           (3u)
/* 实时 36 位触控掩码: 主循环每个 capsense 周期重算, 由 _lnk_status_seal() 封进主动状态帧。 */
static uint8_t lnk_touch_mask[5];
/* 任一通道的 JIT 算法【显式请求点灯】(io->out_led != 0)。白 LED 的唯一运行时来源。
 * 固件不再从触控判定(out_active)推导灯态 —— 没有算法、或算法不写 out_led 时白灯恒灭。 */
volatile bool g_any_active = false;
volatile uint32_t scan_count = 0u;  // 每完成一次全通道扫描 +1，自由递增(无触摸也增长)
uint8_t snapshot_buffers[2u][SNAP_STORE_SIZE];
uint16_t snapshot_generations[2u];
volatile uint8_t published_snapshot_index;
volatile bool published_snapshot_valid;
/* ★"钉一格"取代"拷一份"★ SNAP_LATCH 在 ISR 内把当刻 published_snapshot_index 记进 lnk_snap_pin
 * 并立刻应答; 随后的 SNAP_CH 直接从 snapshot_buffers[lnk_snap_pin] 取数, 而 publish 在租约有效期内
 * 不往那一格写 ⇒ 主机看到的 36 帧必然同代, 且不需要第三份 252 字节缓冲。 */
volatile uint8_t lnk_snap_pin;
volatile bool lnk_snap_pinned;
volatile uint32_t lnk_snap_pin_until_ms;

static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

void initialize_capsense(void)
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
        ch_enable_restore();
        Cy_CapSense_Enable(&cy_capsense_context);
        prepare_csd_mode();
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

/* 封好 tag=0 主动状态帧: 载荷 = 36 位触控掩码(5B) + 已发布代数 gen(u16) + 1 spare。
 * ★为什么调用点是两处★ 掩码在 update_touch_frame 里算出, 但 gen 只有在 publish 之后才是最终值;
 * 若只在前者封帧, 状态帧的 gen 会恒落后一代(主机据此比对代数就会永远差一格)。两次封帧各约
 * 30 个周期, 换来的是"状态帧里的 gen 与刚发布的那一代严格一致"。
 * st 字节在这里只是占位: 写进 TX 环时会被 lnk_tx_feed 覆写为当刻真值。
 * ★体的尾部一律补 0★ 轻量信封没有长度字段, 补齐字节必须是确定值, 否则 CRC 无从复算。 */
static void _lnk_status_seal(void)
{
    uint32_t s = Cy_SysLib_EnterCriticalSection();
    uint8_t * body = &lnk_status_frame[LNK_OFF_BODY];
    uint32_t i;
    lnk_status_frame[LNK_OFF_SOF] = LNK_SOF_RSP;
    lnk_status_frame[LNK_OFF_TAG] = LNK_TAG_NONE;
    lnk_status_frame[LNK_OFF_ST]  = lnk_st;
    body[0] = lnk_touch_mask[0];
    body[1] = lnk_touch_mask[1];
    body[2] = lnk_touch_mask[2];
    body[3] = lnk_touch_mask[3];
    body[4] = lnk_touch_mask[4];
    lnk_wr16(&body[5], published_snapshot_valid ?
                       snapshot_generations[published_snapshot_index] : 0u);
    /* 体尾一律补 0 到 LNK_PAYLOAD_BYTES(= 12, 即 frame[3..14])。★帧内不留流控字段★ 流控走 INT2
     * 带外电平(I3), 故 [14] 仍归本帧体所有, 不许为任何"回报"让位。 */
    for (i = 7u; i < LNK_PAYLOAD_BYTES; i++) { body[i] = 0u; }
    lnk_seal(lnk_status_frame);
    Cy_SysLib_ExitCriticalSection(s);
}

/* 目标缓冲被钉住且租约未过期 ⇒ 本轮整个跳过发布。租约到期即自动解钉(不需要主机显式释放, 也就
 * 不存在"主机掉线把发布永久冻住"这种状态)。下一次 SNAP_LATCH 会重新钉并续租。 */
static inline bool _lnk_snap_pin_blocks(uint8_t index)
{
    bool blocked = false;
    uint32_t s = Cy_SysLib_EnterCriticalSection();
    if (lnk_snap_pinned)
    {
        /* 有符号差比较: g_ms_tick 回绕时也判得对(租约只有 200ms, 差值永远在 int32 量程内)。 */
        if ((int32_t)(lnk_snap_pin_until_ms - g_ms_tick) > 0)
        {
            blocked = (lnk_snap_pin == index);
        }
        else
        {
            lnk_snap_pinned = false;
        }
    }
    Cy_SysLib_ExitCriticalSection(s);
    return blocked;
}

void publish_capsense_snapshot(void)
{
    uint8_t current_index = published_snapshot_index;
    uint8_t next_index = (uint8_t)(current_index ^ 1u);
    uint8_t * destination = snapshot_buffers[next_index];
    uint32_t channel;

    /* 不写、不 bump gen、不翻 INT1 —— 对主机而言这一轮就"没发生过", 语义与"扫描还没出新数据"
     * 完全一致, 不需要额外约定。 */
    if (_lnk_snap_pin_blocks(next_index)) { return; }

    for (channel = 0u; channel < LNK_CHANNEL_COUNT; channel++)
    {
        const cy_stc_capsense_sensor_context_t * sensor =
            &cy_capsense_context.ptrWdConfig[channel].ptrSnsContext[0u];
        uint32_t offset = channel * SNAP_BYTES_PER_CH;

        /* ★禁用通道一律发布全 0★ 它已不参与扫描, sensor->raw/bsln 仍是关闭前的陈旧值 ——
         * 原样上报会让上位机把"关掉的通道"显示成一条恒定不动的假读数(还会被停滞检测判成异常)。
         * 全 0 是明确的"无数据"约定(status=0 ⇒ 未触摸), 与上位机的灰显一致。 */
        if (!ch_is_enabled(channel))
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

    /* 状态帧的 gen 必须是刚发布的这一代(见 _lnk_status_seal 说明), 故在通知线翻转之前重封一次。 */
    _lnk_status_seal();

    /* ★通知必须在发布之后★ 翻转即"这一代已经可读"。本函数只在 CapSense NOT_BUSY 分支里、
     * 且在 update_touch_frame() 之后被调用, 所以翻转时触控掩码与完整快照都已是新的一份 ——
     * RP2040 看到翻转就可以直接取, 不会取到半新半旧。 */
    lnk_int1_notify();
}

/* 从 capsense 上下文计算 36 区 on/off 位图并重封主动状态帧（临界区保护，防 SPI ISR 读到半更新）。 */
void update_touch_frame(void)
{
    uint8_t mask[5] = {0u, 0u, 0u, 0u, 0u};
    uint32_t ch;
    uint32_t st;
    bool use_algo = algo_valid && !algo_upload_active;
    /* 点灯请求只来自算法显式写入的 out_led; 与触控判定(out_active)彻底解耦。 */
    bool algo_led = false;

    for (ch = 0u; ch < LNK_CHANNEL_COUNT; ch++)
    {
        uint32_t base_active;
        uint32_t active;

        /* ★禁用通道恒不触发★ 它不参与扫描也不参与处理, 既不该出现在触控掩码里, 也不该让
         * 上一次的算法状态继续跑(那会让关闭的通道仍能点灯/仍能报按下)。 */
        if (!ch_is_enabled(ch))
        {
            g_algo_io[ch].out_active = 0u;
            g_algo_io[ch].out_led = 0u;
            algo_prev_active[ch] = 0u;
            continue;
        }
        base_active = Cy_CapSense_IsWidgetActive((uint32_t)ch, &cy_capsense_context);

        /* 页写入发生在 SPI ISR，而 algo_fn 在主循环；BEGIN 与首页之间隔一个完整 SPI
         * 事务和 RP2040 轮询间隔（至少 100us），单次 algo_fn 仅数 us。故逐通道复查
         * upload_active 即可关死 "边执行边覆盖" 的窗口：上传一开始立即回退原生判定。 */
        if (use_algo && !algo_upload_active)
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
    lnk_touch_mask[0] = mask[0];
    lnk_touch_mask[1] = mask[1];
    lnk_touch_mask[2] = mask[2];
    lnk_touch_mask[3] = mask[3];
    lnk_touch_mask[4] = mask[4];
    Cy_SysLib_ExitCriticalSection(st);
    _lnk_status_seal();
}

uint16_t cmd_get_raw(uint8_t ch)
{
    if (ch >= LNK_CHANNEL_COUNT) return 0u;
    /* 与快照同口径: 禁用通道回 0(无数据), 不回关闭前的陈旧 raw。 */
    if (!ch_is_enabled(ch)) return 0u;
    return cy_capsense_context.ptrWdConfig[ch].ptrSnsContext[0].raw;
}

/* 主循环阶段: 处理链。自 main() 逐字搬来(含 SEMI 模式必须自己跳过禁用通道的原因)。 */
void csd_process_widgets(void)
{
    /* ★恒为全通道处理★ FOCUS_SCAN(单通道扫描租约)整条链路已删除: RP 侧确证无调用者
     * (sensor_link.cpp:1473 明确注释不再调用设备侧 FOCUS_SCAN)。 */
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
        for (uint32_t w = 0u; w < LNK_CHANNEL_COUNT; w++)
        {
            if (!ch_is_enabled(w)) { continue; }
            (void)Cy_CapSense_ProcessWidgetExt(w, manual_mask, &cy_capsense_context);
        }
    }
}

/* 主循环阶段: 计数 + 带外镜像 + 启动下一轮全通道扫描。 */
void csd_scan_start_step(void)
{

    scan_count++;
    /* 带外 SWD 可读的存活证据: 扫描计数。 */
    spi_dbg.scan_count_m = scan_count;
    spi_dbg.stage = MLOOP_STAGE_SCAN_START;
    /* ★全部通道都被禁用时不得启动扫描★ Cy_CapSense_ScanAllWidgets_V2 会因找不到任何可用
     * widget 而返回 BAD_PARAM 且不置忙标志; 照旧调用只是每轮白跑一次 36 次 SetupWidget 失败。
     * 主循环继续空转(scan_count 照增), 故 RP2040 的"主循环卡死"兜底不会误判。 */
    if (!g_provision_pending && any_ch_enabled())
    {
        /* All normal reset paths prepare CSD explicitly. This guard only repairs a
         * subsequent mode change, avoiding a per-scan disabled-widget traversal. */
        if (cy_capsense_context.ptrActiveScanSns->currentSenseMethod != CY_CAPSENSE_CSD_GROUP)
        {
            prepare_csd_mode();
        }
        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
    }
}

/* 启动复位: 快照双缓冲/代数/触控掩码/发布下标/钉格租约(原 main() 启动序列的对应段)。 */
void csd_scan_reset(void)
{
    memset(snapshot_buffers, 0, sizeof(snapshot_buffers));
    memset(snapshot_generations, 0, sizeof(snapshot_generations));
    memset(lnk_touch_mask, 0, sizeof(lnk_touch_mask));
    g_any_active = false;
    published_snapshot_index = 0u;
    published_snapshot_valid = false;
    lnk_snap_pin = 0u;
    lnk_snap_pinned = false;
    lnk_snap_pin_until_ms = 0u;
}
