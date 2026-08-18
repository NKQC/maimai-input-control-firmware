/*******************************************************************************
 * csd_params.h —— 逐通道/全局 CSD 参数影子 + 通道启用位图 + 长操作请求位
 *
 * ★状态归属★ 参数影子(widgetContext 直写口、全局 RAM 影子、IDAC 档锁定)、启用位图与
 *   provisioning 闸门、以及"主机请求了哪个长操作"的那些标志位都归本模块。长操作的执行态
 *   (逐通道任务结构)属 csd_ops —— 请求与执行分开, 依赖方向才能单向(csd_ops -> csd_params)。
 * ★为什么请求位在这里★ cmd_set_global(GPARAM_AUTO_CALIBRATE_EN 上升沿)要置 calibrate_pending;
 *   若请求位归 csd_ops, 就会出现 csd_params <-> csd_ops 双向依赖。
 ******************************************************************************/
#ifndef CSD_PARAMS_H
#define CSD_PARAMS_H

#include "cy_pdl.h"
#include "cycfg_capsense.h"
#include "psoc_link_abi.h"
#include <stdint.h>
#include <stdbool.h>

// 全局参数 id
#define GPARAM_INACTIVE_SNS              (0x01u)  // 未激活传感器连接: 1=GND 2=High-Z 4=Shield
#define GPARAM_IDAC_GAIN_INIT            (0x02u)  // csdIdacGainInitIndex(IDAC 增益档索引)
#define GPARAM_IDAC_MIN                  (0x03u)  // csdIdacMin(CSD 校准最小 IDAC)
#define GPARAM_RAW_TARGET                (0x04u)  // csdRawTarget(校准目标 raw 百分比)
#define GPARAM_MFS_DIV_F1                (0x05u)  // csdMfsDividerOffsetF1(多频通道1分频偏移)
#define GPARAM_MFS_DIV_F2                (0x06u)  // csdMfsDividerOffsetF2(多频通道2分频偏移)
#define GPARAM_IDAC_SENSE_CONFIG         (0x07u)  // csdChargeTransfer: 0=IDAC sourcing, 1=IDAC sinking (运行时可设)
#define GPARAM_AUTO_CALIBRATE_EN         (0x08u)  // 运行时是否自动校准: 0=固定IDAC(不自动校准), 1=Init/Apply 自动校准
// ---- SPI 链路只读诊断计数(复用 GLOBAL_GET 通道, id 保留以免动上位机的全局项列表) ----
// 语义已换代为 LINK v2 的计数(LNK_CMD_DIAG 一帧回四项, 这里是逐项回读的等价入口)。
#define GPARAM_DBG_RX_FRAMES             (0x80u)  // 通过 SOF+CRC 的完整请求帧数(lnk_diag.rx_ok)
#define GPARAM_DBG_RX_BAD_MAGIC          (0x81u)  // v3: 字节流解析器滑过的非帧首字节数(lnk_diag.rx_reject)
// ★这两个 id 在 v3 里恒回 0★ 它们记的是 CS 上升沿 ISR 的两个动作(清 RX FIFO 残字节 / 重对齐
// RX+TX 描述符), 而那个 ISR 整个已删除(见 I1) ⇒ 结构上不可能发生。id 保留只为不动上位机的全局项列表。
#define GPARAM_DBG_CS_RESYNC             (0x82u)  // v3: 恒 0
#define GPARAM_DBG_TX_ARM                (0x83u)  // TX 可写窗口用尽而放弃写入的次数(lnk_diag.tx_stale)
#define GPARAM_DBG_RX_LEFTOVER           (0x84u)  // v3: 恒 0
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

/* ---- 全局 CSD 配置 RAM 影子 / IDAC 增益档锁定 ---- */
extern cy_stc_capsense_common_config_t g_common_cfg_ram;


/* ★用户手动 IDAC 增益档锁定★: 中间件的校准(Cy_CapSense_CalibrateAllWidgets/CalibrateWidget,
 * 见 cy_capsense_csd_v2.c 校准入口)与 Cy_CapSense_Enable 一进来就把 widgetContext[ch].idacGainIndex
 * 无条件拉回全局起点档 csdIdacGainInitIndex, 于是用户经 PARAM_IDAC_GAIN 设的增幅被静默改回 ——
 * 上位机显示与设备实际不符。故记录"用户显式设过的通道 + 其值", 在所有会重置增益档的动作之后恢复。
 * 语义: 显式 PARAM_IDAC_GAIN 即锁定该通道; 修改全局 GPARAM_IDAC_GAIN_INIT 视为用户改了起点档 →
 * 清空全部锁定(以全局值为准)。GET_PARAM 恒读 widgetContext 实际生效值。 */
typedef struct
{
    uint64_t mask;                        /* bit ch = 该通道增益档被用户锁定 */
    uint8_t  gain[LNK_CHANNEL_COUNT];  /* 锁定通道的用户增益档(0..6) */
} idac_gain_lock_t;
extern volatile idac_gain_lock_t g_idac_lock;

/* ---- 运行时开关 ---- */
extern volatile uint8_t scan_mode;         /* SCAN_MODE_AUTO / SCAN_MODE_SEMI */
extern volatile bool g_auto_calibrate;     /* GPARAM_AUTO_CALIBRATE_EN */
extern volatile uint64_t idac_dirty_mask;  /* 硬件参数被改过、等 APPLY 补校准的通道 */

/* ---- 通道启用位图与 provisioning 闸门 ---- */
#define CH_ENABLED_ALL                   (((uint64_t)1u << LNK_CHANNEL_COUNT) - 1u)
extern volatile uint64_t g_ch_enabled;
extern volatile uint64_t g_provision_enable_seen;
extern volatile bool g_provision_pending;
extern volatile bool g_provision_apply_release;
extern volatile uint64_t ch_enable_dirty;


static inline bool ch_is_enabled(uint32_t ch)
{
    return (ch < LNK_CHANNEL_COUNT) &&
           ((g_ch_enabled & ((uint64_t)1u << ch)) != 0u);
}

static inline bool any_ch_enabled(void)
{
    return (g_ch_enabled & CH_ENABLED_ALL) != 0u;
}

/* ---- 长操作请求位(分发点置位, 主循环消费) ---- */
extern volatile bool apply_pending;
extern volatile bool quick_apply_pending;
extern volatile uint8_t quick_apply_ch;
extern volatile uint8_t quick_apply_gain;
extern volatile uint8_t quick_apply_div;
extern volatile bool calibrate_pending;
extern volatile uint8_t calibrate_ch;
extern volatile bool baseline_reset_pending;
extern volatile uint8_t baseline_ch;
extern volatile bool global_apply_pending;
extern volatile bool measure_cp_pending;
extern volatile bool measure_cp_active;

/* ---- 对外接口 ---- */
void initialize_common_cfg_shadow(void);
void normalize_widget_params(void);
void widget_hw_save(void);
void widget_hw_restore(void);
void idac_lock_clear(void);
bool idac_lock_restore(uint8_t target);
void idac_lock_reapply(void);
bool calibrate_widget_locked(uint32_t ch);
bool cmd_set_param(uint8_t ch, uint8_t param_id, uint32_t value);
uint32_t cmd_get_param(uint8_t ch, uint8_t param_id);
void cmd_set_global(uint8_t gparam_id, uint32_t value);
uint32_t cmd_get_global(uint8_t gparam_id);
void ch_enable_restore(void);
void disabled_widgets_force_highz(void);
void prepare_csd_mode(void);
void initialize_enabled_baselines(void);
void ch_enable_apply(void);
/* 主循环阶段: 旧 RP(从未下发过启用位图)的兼容全启用兜底。 */
void provision_timeout_fallback(void);
void csd_params_reset(void);

#endif /* CSD_PARAMS_H */
