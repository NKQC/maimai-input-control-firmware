/* lnk_dispatch.c —— 命令分发, 见 lnk_dispatch.h。switch 整段自 main.c 逐字搬入。 */
#include "lnk_dispatch.h"
#include "lnk_wire.h"
#include "csd_params.h"
#include "csd_scan.h"
#include "csd_ops.h"
#include "csd_autotune.h"
#include "algo_engine.h"
#include "diag_export.h"
#include "app_tick.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "psoc_link_abi.h"
#include "fw_build_stamp.h"
#include <stdint.h>
#include <stdbool.h>


/* 版本号 = 编译时间戳(十进制 YYMMDDHHMM, 本地时间), 每次 make build 由 PREBUILD 重新生成。
 * 线上格式不变: 仍是 u32, PING 响应照旧拆 4 字节上报。
 * 已知上限: YY <= 42 才放得进 u32, 2043 年起下面的断言会让构建直接失败。 */
#define FW_VERSION                       (FW_BUILD_STAMP)
_Static_assert(FW_VERSION <= 0xFFFFFFFFu, "FW_BUILD_STAMP overflows uint32 (YY > 42?)");

/* ★协议数字一律来自 psoc_link_abi.h★ 帧长/SOF/偏移/命令码/状态位/tag 语义/容量常量都在那里
 * 定义一次。此前它们以 SENSOR_CMD_* / SENSOR_FRAME_* 的形式散在本文件里, 与 RP 侧的一份副本
 * 长期靠人工同步 —— 这一整块已随 LINK v1 一起退役, 不再保留任何兼容别名。 */


/* ★去重缓存只保护非幂等命令★ 读类 / PARAM_SET / GLOBAL_SET / MODE_SET / ALGO_PAGE(绝对寻址) /
 * SNAP_* / ALGO_ROM* / ALGO_CFG* 重执行的结果与第一次逐位相同(写的是同一地址同一值, 读的是同一份
 * 现状), 重复执行没有任何副作用 ⇒ 一律不查不记, 省 RAM 也省 ISR 周期。真正怕重复的只有重操作
 * (重跑一次 APPLY/CALIBRATE 是十几秒重校准)与 ALGO_BEGIN/END(重跑 BEGIN 会把刚传完的算法作废)。
 * 这类命令同一时刻最多一两条在途, 故 LNK_DEDUP_SLOTS 格足够。 */
typedef struct
{
    uint8_t tag;                          /* 0 = 空槽 */
    uint8_t payload[LNK_PAYLOAD_BYTES];   /* 首次执行产出的载荷, 重发时原样重放 */
} lnk_dedup_t;
static lnk_dedup_t lnk_dedup[LNK_DEDUP_SLOTS];
static volatile uint8_t lnk_dedup_head;   /* 环形覆盖最旧 */

/* ---- 去重缓存(只对非幂等命令生效, 见 lnk_dedup 声明处的理由) ---- */

/* 哪些命令重执行会产生真实副作用。其余命令(读类 / PARAM_SET / GLOBAL_SET / MODE_SET /
 * ALGO_PAGE 绝对寻址 / SNAP_* / ALGO_ROM* / ALGO_CFG*)重执行逐位等价 ⇒ 不查不记。 */
static inline bool _lnk_cmd_is_replay_unsafe(uint8_t cmd)
{
    switch (cmd)
    {
        case LNK_CMD_APPLY:
        case LNK_CMD_QUICK_APPLY:
        case LNK_CMD_CALIBRATE:
        case LNK_CMD_BASELINE_RESET:
        case LNK_CMD_GLOBAL_COMMIT:
        case LNK_CMD_MEASURE_CP:
        case LNK_CMD_AUTO_TUNE:
        case LNK_CMD_ALGO_BEGIN:
        case LNK_CMD_ALGO_END:
            return true;
        default:
            return false;
    }
}

/* tag 0 永不入缓存, 故 tag==0 表示空槽。主机的 tag 循环长度(127)远大于 LNK_DEDUP_SLOTS,
 * 一个 tag 被复用时它的旧记录必然早已被挤出 ⇒ 不会把新请求误判成重发。 */
static lnk_dedup_t * _lnk_dedup_find(uint8_t tag)
{
    uint32_t i;
    for (i = 0u; i < LNK_DEDUP_SLOTS; i++)
    {
        if (lnk_dedup[i].tag == tag) { return &lnk_dedup[i]; }
    }
    return NULL;
}

static void _lnk_dedup_store(uint8_t tag, const uint8_t payload[LNK_PAYLOAD_BYTES])
{
    lnk_dedup_t * e = &lnk_dedup[lnk_dedup_head];
    uint32_t i;
    for (i = 0u; i < LNK_PAYLOAD_BYTES; i++) { e->payload[i] = payload[i]; }
    e->tag = tag;
    lnk_dedup_head = (uint8_t)((lnk_dedup_head + 1u) % LNK_DEDUP_SLOTS);
}
/* ★命令执行★ 只在"tag 非 0 且该 tag 尚未执行过"时被调用(去重由调用方保证)。
 * ★v3 起跑在主循环上下文★(由 lnk_rx_drain 调用, 不再是 RX 完成 ISR)。因此这里的 lnk_st /
 * pending 标志读改写不需要临界区 —— 唯一还会并发改写链路状态的 ISR 只剩 spi_dma_isr, 而它只碰
 * lnk_rx_half。"耗时动作只置 pending"这条仍然保留: 它现在的作用是让重操作落到主循环那套
 * 逐通道串行状态机上去, 而不是在这里一口气跑十几秒。
 * 全程 O(1): 响应表示"已受理"; 真实完成由 st.OP_BUSY 由 1→0
 * 判定 —— tag 保证这份 ACK 属于本次请求, 不会像 v1 那样"命令在 SPI 上丢了 + busy 恰好因上一条
 * 为 1"就把上一轮结果冒充成本轮成功。
 * ★恒产出一份响应★ 原先 SNAP_LATCH 走"延迟应答"(返回 false, 由主循环拷完 252B 再入环), 那条机制
 * 随"钉一格"一起删除 —— 现在没有任何命令需要主循环参与产出响应, 故本函数无返回值。 */
static void _lnk_exec(const uint8_t * rx, uint8_t tag)
{
    const uint8_t cmd = rx[LNK_OFF_CMD];
    const uint8_t * a = &rx[LNK_OFF_BODY];
    uint8_t p[LNK_PAYLOAD_BYTES];
    uint32_t i;

    for (i = 0u; i < LNK_PAYLOAD_BYTES; i++) { p[i] = 0u; }

    switch (cmd)
    {
        /* ---------------------------------------------------------- 0x0x 链路 ---- */
        case LNK_CMD_STATUS:
            /* 与主动状态帧同内容(掩码 + gen), 供主机在需要"带 tag 的确定性快照"时显式索取。 */
            for (i = 0u; i < LNK_PAYLOAD_BYTES; i++) { p[i] = lnk_status_frame[LNK_OFF_BODY + i]; }
            break;

        case LNK_CMD_PING:
            lnk_wr32(&p[0], (uint32_t)FW_VERSION);
            p[4] = (uint8_t)LNK_ABI_VERSION;
            p[5] = (uint8_t)LNK_FRAME_SIZE;
            p[6] = (uint8_t)LNK_CHANNEL_COUNT;
            break;

        case LNK_CMD_INDICATOR_ON:
            Cy_GPIO_Write(STATUS_LED_PORT, STATUS_LED_NUM, STATUS_LED_ON_STATE);
            break;

        case LNK_CMD_DIAG:
            lnk_wr16(&p[0], lnk_diag.rx_reject);
            lnk_wr16(&p[2], lnk_diag.respq_drop);
            /* cs_resync / rx_left 恒 0: CS ISR 与 TX 重对齐在 v3 里不存在, 这两件事结构上不可能
             * 发生(见 I1)。字段位置保留, 免得动上位机的解包布局。 */
            lnk_wr16(&p[4], 0u);
            lnk_wr16(&p[6], 0u);
            break;

        /* ---------------------------------------------------------- 0x1x 快照 ---- */
        case LNK_CMD_SNAP_LATCH: {
            /* ★钉一格, 立即应答★ 不再拷副本, 故 O(1), 整件事一次做完 —— 受理与就绪
             * 不再分离, 主机拿到本响应即可按任意顺序取 36 帧, 全部属于同一代。
             * 重复 LATCH 只是改钉哪一格并续租, 天然幂等(所以它不在去重名单里)。 */
            const uint8_t pin = published_snapshot_index;
            lnk_snap_pin = pin;
            lnk_snap_pinned = true;
            lnk_snap_pin_until_ms = g_ms_tick + LNK_SNAP_PIN_LEASE_MS;
            lnk_wr16(&p[0], snapshot_generations[pin]);
            p[2] = published_snapshot_valid ? 1u : 0u;
            p[3] = (uint8_t)LNK_CHANNEL_COUNT;
            lnk_st |= LNK_ST_SNAP_READY;
            break;
        }

        case LNK_CMD_SNAP_CH: {
            /* 一帧一通道: ch 自述 + 该通道的 7 字节记录(raw/bsln/diff/status)。
             * 越界通道回全 0(ch 字节仍回显请求值, 主机据此确认这份属于哪个通道)。
             * ★没有有效 pin 时退回当前已发布那一格★ 于是"不先 LATCH 直接读"也总能拿到一份真实
             * 数据(只是不保证与其他通道同代), 比回全 0 更有用 —— 全 0 与"通道被禁用"撞语义。 */
            const uint8_t ch = a[0];
            p[0] = ch;
            if (ch < LNK_CHANNEL_COUNT)
            {
                const uint8_t src = lnk_snap_pinned ? lnk_snap_pin : published_snapshot_index;
                const uint8_t * rec = &snapshot_buffers[src][(uint32_t)ch * SNAP_BYTES_PER_CH];
                for (i = 0u; i < SNAP_BYTES_PER_CH; i++) { p[1u + i] = rec[i]; }
            }
            break;
        }

        /* ----------------------------------------------------- 0x2x 参数与状态 ---- */
        case LNK_CMD_PARAM_SET:
            spi_dbg.setparam_cmd++;
            (void)cmd_set_param(a[0], a[1], lnk_rd32(&a[2]));
            /* 回显【实际当前值】(非法被拒时=旧值), 供上位机据实回读, 天然拦截非法调参。 */
            p[0] = a[0];
            p[1] = a[1];
            lnk_wr32(&p[2], cmd_get_param(a[0], a[1]));
            break;

        case LNK_CMD_PARAM_GET:
            p[0] = a[0];
            p[1] = a[1];
            lnk_wr32(&p[2], cmd_get_param(a[0], a[1]));
            break;

        case LNK_CMD_GLOBAL_SET:
            cmd_set_global(a[0], lnk_rd32(&a[2]));
            /* ★回显存储值, 不回显请求值★: cmd_set_global 对非法值是"不写、保持原值"; 回显请求值
             * 会让上位机拿到自己刚发的数, 无法分辨"写进去了"还是"被拒了"。 */
            p[0] = a[0];
            lnk_wr32(&p[2], cmd_get_global(a[0]));
            break;

        case LNK_CMD_GLOBAL_GET:
            p[0] = a[0];
            lnk_wr32(&p[2], cmd_get_global(a[0]));
            break;

        case LNK_CMD_RAW_GET:
            p[0] = a[0];
            lnk_wr16(&p[1], cmd_get_raw(a[0]));
            break;

        case LNK_CMD_CP_GET:
            /* pending/active 时统一返回 0, 防止读到主循环逐通道更新的半成品。 */
            p[0] = a[0];
            lnk_wr32(&p[1], (a[0] >= LNK_CHANNEL_COUNT) ? 0xFFFFFFu :
                            ((measure_cp_pending || measure_cp_active) ? 0u : cp_value[a[0]]));
            break;

        case LNK_CMD_STATS:
            /* busy 不再单独上报: 它是每一帧都带的 st.OP_BUSY。 */
            lnk_wr32(&p[0], scan_count);
            lnk_wr32(&p[4], g_ms_tick);
            break;

        case LNK_CMD_MODE_SET:
            scan_mode = (a[0] != 0u) ? SCAN_MODE_SEMI : SCAN_MODE_AUTO;
            p[0] = scan_mode;
            break;

        /* ---------------------------------------------------------- 0x3x 重操作 ---- */
        case LNK_CMD_APPLY:
            /* 启动 gate 只承认已经收到全 36 个 enabled 值之后的 APPLY; 该 APPLY 完成前不扫描。 */
            if (g_provision_pending && (g_provision_enable_seen == CH_ENABLED_ALL))
            {
                g_provision_apply_release = true;
            }
            apply_pending = true;
            lnk_st |= LNK_ST_OP_BUSY;
            break;

        case LNK_CMD_QUICK_APPLY: {
            /* Sweep 专用: 受理点只收下合法参数并置 pending, 绝不在这里触碰 widgetContext ——
             * 分发点可能落在一轮 CapSense 扫描中间(泵在 for(;;) 顶部), 那时写 widgetContext 会卡死
             * 该轮扫描; 真正的写入放在主循环的 NOT_BUSY 窗口。
             * 非法帧不置 pending/busy, accepted=0 明确回显拒绝。 */
            const bool valid = (a[0] < LNK_CHANNEL_COUNT) && (a[1] <= 6u) &&
                               (a[2] >= 1u) && (a[2] <= 64u);
            if (valid)
            {
                quick_apply_ch = a[0];
                quick_apply_gain = a[1];
                quick_apply_div = a[2];
                quick_apply_pending = true;
                lnk_st |= LNK_ST_OP_BUSY;
            }
            p[0] = a[0];
            p[1] = a[1];
            p[2] = a[2];
            p[3] = valid ? 1u : 0u;
            break;
        }

        case LNK_CMD_CALIBRATE:
            /* a[0]=目标通道(0..35 单通道 / 0xFF 全通道), 非法值退化为全通道。 */
            calibrate_ch = (a[0] < LNK_CHANNEL_COUNT) ? a[0] : LNK_CH_ALL;
            calibrate_pending = true;
            lnk_st |= LNK_ST_OP_BUSY;
            p[0] = calibrate_ch;
            break;

        case LNK_CMD_BASELINE_RESET:
            baseline_ch = (a[0] < LNK_CHANNEL_COUNT) ? a[0] : LNK_CH_ALL;
            baseline_reset_pending = true;
            lnk_st |= LNK_ST_OP_BUSY;
            p[0] = baseline_ch;
            break;

        case LNK_CMD_GLOBAL_COMMIT:
            /* 全部全局项已设入影子 → 主循环执行【一次】完整重初始化生效(合并, 防反复重校准漂移)。 */
            global_apply_pending = true;
            lnk_st |= LNK_ST_OP_BUSY;
            break;

        case LNK_CMD_MEASURE_CP:
            /* busy 覆盖 BIST 与其后的正常 CSD 恢复, 主机只会在恢复完成后继续任何重操作。 */
            measure_cp_pending = true;
            lnk_st |= LNK_ST_OP_BUSY;
            break;

        case LNK_CMD_AUTO_TUNE:
            /* a[0]=目标通道(0..35 / 0xFF 全通道), a[1]=灵敏度偏好档位(1..7, 非法退化为 4)。
             * ★v1 的 auto_tune_tag 已删★ "读到的是不是本轮结果"由链路 tag 判定, 不再需要
             * 在 result 字节高位手搓一份私有标签。 */
            auto_tune_ch      = (a[0] < LNK_CHANNEL_COUNT) ? a[0] : LNK_CH_ALL;
            auto_tune_pref    = ((a[1] >= 1u) && (a[1] <= 7u)) ? a[1] : 4u;
            auto_tune_pending = true;
            auto_tune_result  = 0u;   /* 进行中 */
            auto_tune_phase   = AUTO_TUNE_PHASE_IDLE;
            auto_tune_step    = 0u;
            auto_tune_div     = 0u;
            lnk_st |= LNK_ST_OP_BUSY;
            p[0] = auto_tune_ch;
            p[1] = auto_tune_pref;
            break;

        case LNK_CMD_AUTO_TUNE_GET:
            /* div 语义随 result 变化: 进行中=当前试探分频; 完成=最终分频(单通道)或成功通道数(全通道)。 */
            p[0] = auto_tune_result;
            p[1] = auto_tune_ch;
            lnk_wr16(&p[2], auto_tune_div);
            p[4] = auto_tune_phase;
            p[5] = auto_tune_step;
            break;

        /* ------------------------------------------------------- 0x4x JIT 算法 ---- */
        case LNK_CMD_ALGO_BEGIN:
            lnk_wr16(&p[0], cmd_algo_begin(lnk_rd16(&a[0])));
            break;

        case LNK_CMD_ALGO_PAGE: {
            const uint32_t page = (uint32_t)lnk_rd16(&a[0]);
            uint32_t offset = 0u;
            if (cmd_algo_page(page, &a[2], &offset))
            {
                lnk_wr16(&p[0], (uint16_t)page);
                lnk_wr16(&p[2], (uint16_t)offset);
            }
            else
            {
                lnk_wr16(&p[0], 0xFFFFu);   /* 未受理(越界 / 无在途上传) */
            }
            break;
        }

        case LNK_CMD_ALGO_END:
            cmd_algo_end(lnk_rd16(&a[0]));
            p[0] = 1u;   /* 已受理, 将由主循环全量 CRC16 校验 */
            lnk_wr16(&p[2], algo_expected_len);
            break;

        case LNK_CMD_ALGO_INFO:
            /* v1 需要 INFO + GET_CRC 两次事务才能判"是否真的换代了", 现在一帧回齐。 */
            p[0] = algo_valid ? 1u : 0u;
            p[1] = algo_upload_active ? 1u : 0u;
            lnk_wr16(&p[2], algo_len);
            lnk_wr16(&p[4], algo_slot_crc);
            lnk_wr16(&p[6], algo_reject_count);
            break;

        case LNK_CMD_ALGO_CAPS:
            /* 容量必须由设备自报(v1 的 CAPS + HEAP 合成一帧): 三层常量漏改时当场可见。 */
            lnk_wr16(&p[0], (uint16_t)LNK_ALGO_SLOT_SIZE);
            lnk_wr16(&p[2], (uint16_t)LNK_ALGO_HEAP_SIZE);
            lnk_wr16(&p[4], algo_heap_used_peak);
            p[6] = (uint8_t)LNK_ALGO_PAGE_BYTES;
            p[7] = (uint8_t)LNK_ABI_VERSION;
            break;

        case LNK_CMD_ALGO_ROM_SET: {
            const uint16_t rom = lnk_rd16(&a[2]);
            if (a[0] < LNK_CHANNEL_COUNT) { g_algo_rom[a[0]] = rom; }
            p[0] = a[0];
            lnk_wr16(&p[2], (a[0] < LNK_CHANNEL_COUNT) ? g_algo_rom[a[0]] : 0u);
            break;
        }

        case LNK_CMD_ALGO_ROM_GET:
            p[0] = a[0];
            lnk_wr16(&p[2], (a[0] < LNK_CHANNEL_COUNT) ? g_algo_rom[a[0]] : 0u);
            break;

        case LNK_CMD_ALGO_CFG_SET:
            if (a[0] < 8u) { g_algo_cfg[a[0]] = a[1]; }
            p[0] = a[0];
            p[1] = (a[0] < 8u) ? g_algo_cfg[a[0]] : 0u;
            break;

        case LNK_CMD_ALGO_CFG_GET:
            p[0] = a[0];
            p[1] = (a[0] < 8u) ? g_algo_cfg[a[0]] : 0u;
            break;

        case LNK_CMD_ALGO_CFGCH_SET:
            if ((a[0] < LNK_CHANNEL_COUNT) && (a[1] < 8u)) { g_algo_cfg_ch[a[0]][a[1]] = a[2]; }
            p[0] = a[0];
            p[1] = a[1];
            p[2] = ((a[0] < LNK_CHANNEL_COUNT) && (a[1] < 8u)) ? g_algo_cfg_ch[a[0]][a[1]] : 0u;
            break;

        case LNK_CMD_ALGO_CFGCH_GET:
            p[0] = a[0];
            p[1] = a[1];
            p[2] = ((a[0] < LNK_CHANNEL_COUNT) && (a[1] < 8u)) ? g_algo_cfg_ch[a[0]][a[1]] : 0u;
            break;

        case LNK_CMD_ALGO_TRACE:
            /* 8 字节载荷刚好装满 report[0..3](v1 要 4 次事务轮转 idx 才能读齐)。
             * out_active 不在这里: 它逐通道的值就是状态帧那份触控掩码的对应位。 */
            if (a[0] < LNK_CHANNEL_COUNT)
            {
                for (i = 0u; i < 4u; i++) { lnk_wr16(&p[i * 2u], g_algo_io[a[0]].report[i]); }
            }
            break;

        default:
            /* 未知命令: 仍回一份带本 tag 的空载荷响应。主机据此确认"这条被收到了但设备不认",
             * 而不是等超时 —— 也保证该 tag 进了去重缓存, 重发不会被反复解析。 */
            break;
    }
    /* 非幂等命令的载荷存一份, 供重发时原样重放而【不再执行一次】。幂等命令不记 —— 重执行逐位
     * 等价, 记了只是白占槽位并把真正需要保护的那几条挤出去。 */
    if (_lnk_cmd_is_replay_unsafe(cmd)) { _lnk_dedup_store(tag, p); }
    lnk_resp_push(tag, p);
}

/* 一帧到手后的投递: 去重 → 执行 → 入队。
 *   a. tag == 0 = 纯轮询: 不携带命令, 只为给从机一次喂环的机会 ⇒ 不产出响应;
 *   b. tag != 0 且是非幂等命令且 tag 命中去重缓存 ⇒ 重发, 把存下的载荷重新入队, **不执行**;
 *   c. 其余 tag != 0 ⇒ 执行并入队(重操作只置 pending 并入"已受理"响应)。 */
void lnk_deliver(const uint8_t * rx)
{
    const uint8_t tag = rx[LNK_OFF_TAG];
    if (tag == LNK_TAG_NONE) { return; }
    {
        const uint8_t cmd = rx[LNK_OFF_CMD];
        lnk_dedup_t * dup = _lnk_cmd_is_replay_unsafe(cmd) ? _lnk_dedup_find(tag) : NULL;
        if (dup != NULL)
        {
            /* ★去重★ 非幂等命令的同一 tag 只执行一次: 重发只重放已存载荷。这是主机敢在任何时刻
             * 随意补发 APPLY/CALIBRATE/ALGO_BEGIN 的根据。 */
            lnk_resp_push(tag, dup->payload);
        }
        else
        {
            _lnk_exec(rx, tag);
        }
    }
}

/* 去重缓存复位: 原 main() 启动序列的对应两行。 */
void lnk_dispatch_reset(void)
{
    memset(lnk_dedup, 0, sizeof(lnk_dedup));
    lnk_dedup_head = 0u;
}
