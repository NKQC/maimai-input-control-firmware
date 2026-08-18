/*******************************************************************************
 * csd_scan.h —— 扫描发起 / 处理链 / 触控帧 / 快照发布与锁存
 *
 * ★状态归属★ 双缓冲快照、代数、钉格租约、实时触控掩码、scan_count、g_any_active 归本模块。
 *   主动状态帧的缓冲(lnk_status_frame)归线级(它是 TX 环的内容), 由本模块的 _lnk_status_seal()
 *   写入 —— 反过来线级不认识触控掩码, 依赖方向仍是 csd_scan -> lnk_wire 单向。
 ******************************************************************************/
#ifndef CSD_SCAN_H
#define CSD_SCAN_H

#include "cy_pdl.h"
#include "cycfg_capsense.h"
#include "psoc_link_abi.h"
#include <stdint.h>
#include <stdbool.h>


/* 内部快照存储: 每通道 7 字节(raw u16, bsln u16, diff i16, status u8) —— 不含通道号。
 * 线上帧的一通道载荷是 LNK_SNAP_BYTES_PER_CH(=8) = ch 自述字节 + 这 7 字节, 由 SNAP_CH 现拼。
 * 存储侧不存 ch(它就是下标), 省 36 字节。 */
#define SNAP_BYTES_PER_CH                (7u)
#define SNAP_STORE_SIZE                  (LNK_CHANNEL_COUNT * SNAP_BYTES_PER_CH)
_Static_assert(LNK_SNAP_BYTES_PER_CH == (SNAP_BYTES_PER_CH + 1u),
               "frame payload must be ch byte + stored per-channel record");

/* ★快照"钉一格"的租约★ SNAP_LATCH 不再拷一份 252 字节副本(那份副本要额外 252B RAM, 而 PSoC 的
 * .bss 已经把栈顶逼到 CapSense 校准调用链踩穿的边上), 改为把当前已发布的那个双缓冲下标钉住:
 * 主机读完 36 帧 SNAP_CH 之前, publish 不许往那一格写。
 * ★租约是必需的兜底★ 主机中途掉线(读了几帧就没了)不能把发布永久冻住 —— 那会让 gen 不再推进、
 * INT1 不再翻转, 表现成"设备挂了"。200ms 远大于一次全通道取数(36 帧, 约 2.3ms), 又远小于人眼
 * 可感的停顿, 且最多只让 publish 跳过一轮(扫描周期约 6ms)。 */
#define LNK_SNAP_PIN_LEASE_MS            (200u)

extern volatile uint32_t scan_count;
extern uint8_t snapshot_buffers[2u][SNAP_STORE_SIZE];
extern uint16_t snapshot_generations[2u];
extern volatile uint8_t published_snapshot_index;
extern volatile bool published_snapshot_valid;
/* SNAP_LATCH 钉住的那一格与其租约(分发点写, 发布点读)。 */
extern volatile uint8_t lnk_snap_pin;
extern volatile bool lnk_snap_pinned;
extern volatile uint32_t lnk_snap_pin_until_ms;
/* 任一通道的 JIT 算法显式点灯请求(白 LED 的唯一运行时来源)。 */
extern volatile bool g_any_active;

void initialize_capsense(void);
void update_touch_frame(void);
void publish_capsense_snapshot(void);
uint16_t cmd_get_raw(uint8_t ch);
/* 主循环阶段: 处理链(AUTO 完整链 / SEMI 手动掩码)与启动下一轮扫描。 */
void csd_process_widgets(void);
void csd_scan_start_step(void);
void csd_scan_reset(void);

#endif /* CSD_SCAN_H */
