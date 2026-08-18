/* algo_engine.c —— JIT 算法引擎, 见 algo_engine.h。逐字自 main.c 搬入。 */
#include "algo_engine.h"
#include "lnk_wire.h"
#include "diag_export.h"
#include "app_tick.h"
#include "cycfg_capsense.h"
#include <string.h>


/* ---- JIT 可加载触控算法引擎（ABI v1，见 psoc_algo_abi.h）----
 * algo_slot：可执行 4KB RAM 槽，4 字节对齐（Cortex-M0+ 从 SRAM 取指，thumb 入口 |1）。
 * PSoC 仅余约 3.5KB RAM，不能再留等大的暂存区；ALGO_PAGE 直写槽，由主循环 CRC16 决定是否接纳。 */
static uint8_t algo_slot[ALGO_SLOT_SIZE] __attribute__((aligned(4)));
static uint8_t algo_heap[ALGO_HEAP_SIZE] __attribute__((aligned(4)));
/* 槽/堆容量在算法 ABI(给 blob 看)与链路 ABI(给主机看)里各声明了一次, 用途不同不能合并;
 * 但它们必须永远相等 —— 不等就是"设备自报的容量与实际槽大小不符", 那正是要根除的一类漂移。 */
_Static_assert(ALGO_SLOT_SIZE == LNK_ALGO_SLOT_SIZE, "algo slot size differs between ABIs");
_Static_assert(ALGO_HEAP_SIZE == LNK_ALGO_HEAP_SIZE, "algo heap size differs between ABIs");
volatile bool algo_valid = false;
volatile uint16_t algo_len = 0u;
/* ALGO_BEGIN 声明的期望长度；ALGO_PAGE 越界保护用。 */
volatile uint16_t algo_expected_len = 0u;
/* 主循环 commit 状态机：ISR 只置位，真正的 CRC 校验在主循环执行。 */
static volatile bool algo_commit_pending = false;
static volatile uint16_t algo_commit_crc = 0u;
/* 已提交槽内容的 CRC16；上传失败时清零，避免上位机把坏码误当成功。 */
volatile uint16_t algo_slot_crc = 0u;
/* 本轮上传已写到的最高字节位置(offset+3 的最大值)，仅用于 commit 前的完整性抽查。
 * ★为什么不是"顺序游标"★ RP2040 的 _cmd_txn 在回显不匹配时会**重发同一条命令**(最多 3 次,
 * 见 psoc_spi.cpp 的 "Retry idempotent command transactions" 注释) —— 它整套读写原语都建立在
 * "命令幂等"这个前提上。顺序游标会让一次重发把游标推进两格(或直接判失序而中止), 且页数越多
 * 越容易撞上; 实测 616B(154 页)能过、1056B 必失败就是这个概率差。
 * 故 ALGO_PAGE 必须回到**绝对寻址**: 同一页重复写入结果完全相同, 天然幂等。 */
static volatile uint16_t algo_write_max = 0u;
volatile bool algo_upload_active = false;
static volatile uint32_t algo_upload_last_ms = 0u;
/* CRC 失败次数仅递增（饱和），供带外诊断确认坏码没有被静默接纳。 */
volatile uint16_t algo_reject_count = 0u;
volatile uint16_t algo_heap_used_peak = 0u;
/* 每通道 8 字节运行时配置；与全局 cfg[8] 并存，供同一算法按电极差异调整。 */
uint8_t g_algo_cfg_ch[LNK_CHANNEL_COUNT][8];
/* 逐通道持久 IO 记录：状态字段跨周期原地保留，仅输入字段每周期刷新。 */
algo_io_t g_algo_io[LNK_CHANNEL_COUNT];
/* 记录上一周期各通道 base_active，用于检测非激活→激活边沿以清零 state[]。 */
uint8_t algo_prev_active[LNK_CHANNEL_COUNT];
/* 每通道 16 位 ROM：上位机随算法下发的只读常量(如 per-channel fingerCap/Cp/阈值)，
 * 每周期填入 io->rom 供算法读取。算法不写。ALGO_SET_ROM/GET_ROM 读写。 */
volatile uint16_t g_algo_rom[LNK_CHANNEL_COUNT];
/* 共享算法可设置变量(ABI cfg[8]): 上位机经 ALGO_SET_CFG 下发, 每次算法执行前拷入 io->cfg。
 * 供算法运行时可调参数(如基线偏移、阈值系数等), 上位机按源码声明的名字+默认值展示为可调项。 */
volatile uint8_t g_algo_cfg[8] = {0u};

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

/* 用可加载算法 blob 逐通道计算激活位。返回 0/1。仅在 algo_valid 时被调用；
 * 坏 blob 死循环由 RP2040 侧 PONG 心跳兜底硬复位处理(见设计文档 §1)，PSoC 端不需看门狗。 */
uint32_t algo_engine_run_channel(uint32_t ch, uint32_t base_active)
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
    for (uint32_t k = 0u; k < 8u; k++)
    {
        io->cfg[k] = g_algo_cfg[k];
        io->cfg_ch[k] = g_algo_cfg_ch[ch][k];
    }
    /* 堆由所有通道共享；算法每轮重新声明实际占用，固件只接受 ABI 约定的 256 字节范围。 */
    io->heap = algo_heap;
    io->heap_size = (uint16_t)ALGO_HEAP_SIZE;
    io->heap_used = 0u;

    /* 每轮先清点灯请求, 由算法重新声明: 否则换成不写 out_led 的算法后, 上一个算法(如 LED 演示)
     * 留下的 1 会让白灯永久亮着 —— 那又变成了"用户改不掉的灯"。 */
    io->out_led = 0u;

    algo_fn(io);
    if (io->heap_used > ALGO_HEAP_SIZE) { io->heap_used = (uint16_t)ALGO_HEAP_SIZE; }
    if (io->heap_used > algo_heap_used_peak) { algo_heap_used_peak = io->heap_used; }
    return io->out_active;
}

/* ---- JIT 算法引擎命令(分发点上下文, 只做快速缓冲写入/标志置位; 响应由调用方入环) ---- */
uint16_t cmd_algo_begin(uint16_t len)
{
    if (len > ALGO_SLOT_SIZE) { len = ALGO_SLOT_SIZE; }
    algo_expected_len = len;
    algo_commit_pending = false;
    /* 必须在 BEGIN 立刻作废旧算法：此前 valid 粘滞会让上传失败时上位机仍看见成功。 */
    algo_valid = false;
    algo_len = 0u;
    algo_slot_crc = 0u;
    algo_write_max = 0u;
    algo_upload_active = true;
    algo_upload_last_ms = g_ms_tick;
    /* 上传期间 valid/len 属于上一份, 不可采信 —— 用 st 如实告知主机(主循环上下文, 直接改)。 */
    lnk_st = (uint8_t)((uint8_t)(lnk_st & (uint8_t)~(uint8_t)LNK_ST_ALGO_VALID) | LNK_ST_ALGO_UPLOAD);
    return len;
}

/* 写一页算法字节: args = 页号 u16 + LNK_ALGO_PAGE_BYTES 字节码, 绝对寻址 offset = page*PAGE_BYTES。
 * ★页宽一律取自宏, 本函数不许出现字面量★ 页宽 = LNK_ARG_BYTES - 2(页号), 随信封宽度变动过一轮
 * (I3 曾在帧内挖走一字节, 改成 INT2 电平后又还回来); 写死数字会让上传静默错位(每页少写一字节,
 * CRC16 才发现, 且无从定位)。
 * ★绝对寻址在 v2 里的意义变了★ v1 需要它是因为主机会重发同一条命令(靠幂等兜底); v2 的 tag
 * 去重已经保证任何重发都不会重复执行, 绝对寻址留下来是为了让主机能**单独补发某一页**而不必
 * 整轮重传 —— 这是 v1 唯一没有的能力。
 * ★末页必须按 LNK_ALGO_SLOT_SIZE 截断★ 4096 不是 10 的整数倍: 共 LNK_ALGO_PAGE_COUNT=410 页
 * (0..409), 末页 offset=4090 只能写 6 字节, 否则越界 4 字节。
 * ★越界判据用 offset 而非页号★ offset >= LNK_ALGO_SLOT_SIZE 一条就等价于 page >= 410, 页宽再变
 * 也不需要改这里。
 * ★未受理一律回 page=0xFFFF★ 明确表示"这一页没写进去", 取代 v1 那种"故意取反页号"的隐式约定
 * (取反后的页号本身是个合法页号, 主机得靠约定才知道它是错误信号)。
 * ★不做失序检查★ 绝对寻址下"失序"不存在; 完整性由 ALGO_END 的 CRC16 全量校验兜底。 */
bool cmd_algo_page(uint32_t page, const uint8_t * data, uint32_t * out_offset)
{
    const uint32_t offset = page * LNK_ALGO_PAGE_BYTES;
    uint32_t n = LNK_ALGO_PAGE_BYTES;
    uint32_t i;

    if (!algo_upload_active || (offset >= LNK_ALGO_SLOT_SIZE)) { return false; }
    if ((offset + n) > LNK_ALGO_SLOT_SIZE) { n = LNK_ALGO_SLOT_SIZE - offset; }
    for (i = 0u; i < n; i++) { algo_slot[offset + i] = data[i]; }
    if ((uint16_t)(offset + n) > algo_write_max) { algo_write_max = (uint16_t)(offset + n); }
    algo_upload_last_ms = g_ms_tick;
    *out_offset = offset;
    return true;
}

/* 只置位 pending + 记录 CRC/len; 4KB 的全量 CRC16 太慢, 不能放在分发点(它可能落在一轮 CapSense
 * 扫描中间), 故交给主循环 NOT_BUSY 窗口。响应的 accepted=1 只表示"已受理, 将校验",
 * 校验结果由随后的 ALGO_INFO 反映。 */
void cmd_algo_end(uint16_t crc)
{
    algo_commit_crc = crc;
    algo_commit_pending = true;
    algo_upload_last_ms = g_ms_tick;
}

/* 主循环阶段: 上传断流自愈(自 main() 逐字搬来)。 */
void algo_upload_watchdog(void)
{
    /* 上传断流必须自愈：否则 BEGIN 后没有 END 会永久禁用 JIT，回退原生判定。 */
    if (algo_upload_active && ((g_ms_tick - algo_upload_last_ms) > 3000u))
    {
        algo_upload_active = false;
        lnk_st_upd(0u, LNK_ST_ALGO_UPLOAD);
    }
}

/* 主循环阶段: ALGO_END commit(全量 CRC16 校验后如实接纳或回退)。 */
void algo_commit_step(void)
{

    /* ALGO_END commit：槽内容已经被 PAGE 直接覆盖，只能 CRC16 校验后如实接纳或回退。
     * CRC 失败不能保留旧算法：其字节早已被新上传覆盖，必须 valid=0 走原生 CapSense。 */
    if (algo_commit_pending)
    {
        spi_dbg.stage = MLOOP_STAGE_ALGO_COMMIT;
        uint16_t len;
        uint16_t crc_expect;
        uint16_t crc_calc;
        uint32_t i;

        algo_commit_pending = false;
        len = algo_expected_len;
        crc_expect = algo_commit_crc;
        crc_calc = algo_crc16(algo_slot, len);

        if ((crc_calc == crc_expect) && (len != 0u) && (algo_write_max >= len))
        {
            algo_len = len;
            algo_slot_crc = crc_calc;
            algo_valid = true;
            /* 换代必须清空逐通道现场与共享堆，否则新算法会读到旧版状态，表现仍像旧算法。 */
            for (i = 0u; i < LNK_CHANNEL_COUNT; i++)
            {
                memset(g_algo_io[i].state, 0, sizeof(g_algo_io[i].state));
                memset(g_algo_io[i].report, 0, sizeof(g_algo_io[i].report));
                g_algo_io[i].out_active = 0u;
                g_algo_io[i].out_led = 0u;
                g_algo_io[i].heap_used = 0u;
                algo_prev_active[i] = 0u;
            }
            memset(algo_heap, 0, sizeof(algo_heap));
            algo_heap_used_peak = 0u;
            lnk_st_upd(LNK_ST_ALGO_VALID, 0u);
        }
        else
        {
            algo_valid = false;
            algo_len = 0u;
            algo_slot_crc = 0u;
            if (algo_reject_count < 0xFFFFu) { algo_reject_count++; }
            lnk_st_upd(0u, LNK_ST_ALGO_VALID);
        }
        algo_upload_active = false;
        lnk_st_upd(0u, LNK_ST_ALGO_UPLOAD);
    }
}

/* 启动复位: 算法现场/ROM/cfg/堆与全部上传标志(原 main() 启动序列的对应段)。 */
void algo_reset(void)
{
    memset(g_algo_io, 0, sizeof(g_algo_io));
    memset(algo_prev_active, 0, sizeof(algo_prev_active));
    for (uint32_t channel = 0u; channel < LNK_CHANNEL_COUNT; channel++) { g_algo_rom[channel] = 0u; }
    for (uint32_t k = 0u; k < 8u; k++) { g_algo_cfg[k] = 0u; }
    memset(g_algo_cfg_ch, 0, sizeof(g_algo_cfg_ch));
    memset(algo_heap, 0, sizeof(algo_heap));
    algo_valid = false;
    algo_len = 0u;
    algo_expected_len = 0u;
    algo_commit_pending = false;
    algo_commit_crc = 0u;
    algo_slot_crc = 0u;
    algo_write_max = 0u;
    algo_upload_active = false;
    algo_upload_last_ms = 0u;
    algo_reject_count = 0u;
    algo_heap_used_peak = 0u;
}
