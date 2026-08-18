/* lnk_wire.c —— 线级链路实现, 见 lnk_wire.h。整段自 main.c 逐字搬入, 未改语义。 */
#include "lnk_wire.h"
#include "diag_export.h"
#include "cycfg.h"
#include <string.h>

static void spi_dma_isr(void);


/* ★CS(P1.3)上升沿中断整套已删除, 不留常量★ 见 psoc_link_abi.h 的 I1: 那个 ISR 假定自己一定在
 * 两次事务之间运行, 但优先级 0 的 DMA ISR 一批要跑多次、而主机是背靠背 pump ⇒ 它实际清掉的是
 * 【新帧的头几个字节】, 自己就是丢字节的来源; 它随后调的 TX 重对齐又因为 SCB 的 TX 移位/预取
 * 寄存器没有清除 API, 必然留下残留字节把相位永久偏掉。v3 不再有"帧边界靠时序"这件事。 */

static cy_stc_scb_spi_context_t spi_context;

/* ★RX 环(v3)★ DMA 自己绕着走的整帧环, 与 TX 环完全同构(PING/PONG 各覆盖半环、两描述符恒有效、
 * cpltState=false、flipping 交替)。LNK_RXRING × 16 = 256 字节。
 * v2 这里只有 rx_frame[2] 双缓冲, 而主机一批能连发 LNK_BURST_MAX 帧 ⇒ ISR 被拖过两帧时间就被
 * DMA 套圈丢帧。环的深度同时就是 I3 的流控上限: LNK_RXRING = LNK_RX_SPACE_MIN + LNK_BURST_MAX,
 * 即"电平陈旧时主机最多再灌一整批"也恰好用满、一格不溢出。
 * ★按字节流看, 不按格看★ 从机不假设主机的 CS 窗与自己的格边界重合(I2), 故解析器只认
 * "绝对字节位置", 环下标一律由 & (LNK_RXRING_BYTES-1) 得到。 */
#define LNK_RXRING_BYTES                 (LNK_RXRING * LNK_FRAME_SIZE)
#define LNK_RXRING_HALF_BYTES            (LNK_RXRING_BYTES / 2u)
_Static_assert((LNK_RXRING_BYTES & (LNK_RXRING_BYTES - 1u)) == 0u,
               "rx ring byte size must be a power of two");
static uint8_t lnk_rxring[LNK_RXRING][LNK_FRAME_SIZE];
/* 已完成的半环数(只由 spi_dma_isr 递增)。★它不是"收了几格"的唯一依据★: ISR 迟到会漏计, 故
 * _lnk_rx_written() 还要拿当前描述符的奇偶去纠正它 —— 反过来也不能只信硬件位置, 因为环恰好
 * 装满时"写位置 == 读位置"与"环空"在环内坐标上无从区分, 那会把 INT2 电平永久压低(主机不再发帧)。 */
static volatile uint32_t lnk_rx_half;
/* 绝对已消费字节数(主循环独占)。与 _lnk_rx_written() 的差值 = 可解析字节数, 两者同为 mod 2^32
 * 的绝对量 ⇒ 回绕天然抵消。 */
static uint32_t lnk_rx_rd;
/* ★已落字节数的单调闩存★ _lnk_rx_written() 在"读的过程中正好翻环"时会给出一个保守偏小的估计;
 * 若让这个偏小值直接传给解析器, (wr - rd) 会绕成一个巨大的正数、触发"被套圈"分支, 把读位置往回
 * 拖整整一个环 —— 那会把一环的旧字节重新解析一遍(旧帧被重执行、流控电平当场失真)。
 * 故这里只允许它单调不减: 保守的那一拍等下一轮自然补上。 */
static uint32_t lnk_rx_wr;

/* ================================ LINK v2 从机引擎 ================================
 * 三条与时序无关的不变式取代 v1 的"位置约定"(详见 psoc_link_abi.h):
 *   ① 每帧自证身份(SOF + tag + CRC8); ② 主机按 tag 匹配并退役, 迟到/重复/非请求帧一律丢弃;
 *   ③ 从机对【非幂等】命令按 tag 去重(见 lnk_dedup)。
 *
 * ★TX 侧是"DMA 自己走的整帧环", 软件彻底退出发送路径★
 * 上一版是"每帧由 RX 完成 ISR 重挂 TX 描述符"。那条路本身就是病灶: 响应能不能被取走取决于 ISR
 * 能否在主机下一帧到来之前跑完, 而 CapSense 中间件会整段关中断 —— 那期间 RX 完成 ISR 根本进不来
 * ⇒ TX FIFO 空 ⇒ 主机成片读到非法帧 ⇒ 只能重发, 时间全烧在重试上(实测满容量算法上传会随机某页
 * 把重试预算耗尽而失败, 与页号、长度都无关)。
 * 现在 DMAC ch1 用 PING/PONG 各覆盖半个环、两描述符恒有效(cpltState=false)、flipping 交替
 * ⇒ 硬件不停地把环喂进 TX FIFO, FIFO 永不空。由此:
 *   · 主机每一帧都能读到一个【格式合法】的帧, 不存在"读空"这种失败模式;
 *   · ISR 迟到只意味着环里那一格还是旧内容 —— 旧内容要么是主动状态帧(幂等, 重复无害), 要么是
 *     tag 已退役的旧响应(主机直接丢弃) ⇒ ★迟到不再产生任何需要重发的后果, 所以不用重试★;
 *   · 软件只做一件事(lnk_tx_feed): 把"DMA 读指针 + LNK_TX_LEAD"起的那些格填成待发响应或状态帧。
 *
 * ★v3: 软件在整条收发路径上都不再触碰硬件★ 除 spi_dma_init 那一次配置之外, 任何代码路径都不
 * 再出现 Cy_SCB_SPI_ClearRxFifo / ClearTxFifo / Cy_DMAC_Descriptor_SetState /
 * Cy_DMAC_Channel_SetCurrentDescriptor / Cy_DMAC_Channel_Enable —— 这就是 I1"流永不中断":
 * 字节既不会被丢也不会被插入, 相位在物理上不可能被破坏, 于是根本不需要任何"重同步"动作。
 * 命令分发也从 ISR 搬到了主循环(lnk_rx_drain), ISR 只推进"已落了多少字节"这一个量。 */

/* 待发响应队列: 只缓冲"已产出但还没轮到写进环"的响应。★不再兼作去重缓存★ —— 写进环的内容会被
 * 后续状态帧覆盖, 存不住一份可重放的副本; 去重另设 lnk_dedup。 */
typedef struct
{
    uint8_t buf[LNK_FRAME_SIZE];  /* 已封好的响应帧(st 与 CRC 在写进环时按当刻重写) */
    uint8_t tag;                  /* 0 = 空槽(tag 0 永不入队, 故可当空标记用) */
    uint8_t sent;                 /* 0 = 待发; 1 = 已写进环(该槽可被覆盖) */
} lnk_resp_t;

static lnk_resp_t lnk_respq[LNK_RESPQ];
/* ★容量不变式, 不是调优参数★ 取帧循环已没有"无槽就停"的分支(那是第二类永久死锁), 因此本条一旦
 * 被改小, 从机就会开始覆盖未发出的响应。让它在编译期失败, 而不是在链路上表现为偶发丢响应。 */
_Static_assert(LNK_RESPQ >= LNK_RXRING, "LNK_RESPQ must cover the full flow-control window (I3)");
static volatile uint8_t lnk_q_head;   /* 下一个写入位置 = 队里最旧的条目 */
static volatile uint8_t lnk_q_send;   /* 取待发条目的起点(环形推进) */

/* ★TX 环★ DMA 自己走; 软件只写"领先 LNK_TX_LEAD 格"的那一格。LNK_TXRING × 16 = 128 字节。
 * 每一格恒是一个封好口的合法帧(初始化时全部填当刻状态帧), 故任意时刻被 DMA 搬走都无害。 */
static uint8_t lnk_txring[LNK_TXRING][LNK_FRAME_SIZE];
/* 上一次写入的格号: 写入只许从它的下一格继续往前推, 绝不回头 —— 回头写会覆掉一份还没被 DMA
 * 搬走的响应。初值取 LNK_TX_LEAD-1, 使第一次写入正好落在 d(=0) + LNK_TX_LEAD 那一格。
 * ★不再有 0xFF 哨兵★ 它原本只为 _lnk_tx_realign 复位用, 而重对齐整套已随 I1 删除。 */
static uint8_t lnk_tx_last_w = (uint8_t)(LNK_TX_LEAD - 1u);

uint8_t lnk_status_frame[LNK_FRAME_SIZE];

volatile uint8_t lnk_st;

/* ★"没有空响应槽就停止取帧"这道背压已整体删除★ 它是第二类永久死锁: 响应只能靠主机时钟搬走,
 * 而主机在 INT2 为低时一个字节都不发 ⇒ 从机若因无槽而停取, 响应槽就永不释放、电平永不抬高。
 * 契约改为容量不变式 LNK_RESPQ >= LNK_RXRING(见 psoc_link_abi.h I3): 主机遵守电平时在途请求最多
 * LNK_RXRING 条 ⇒ 响应槽永远够 ⇒ 取帧循环没有任何理由停下。
 * "竟然真的没槽"这件事仍有一个只增不减的诊断: 无槽时下面的 lnk_resp_push 必然覆盖一条未发出的
 * 条目 ⇒ lnk_diag.respq_drop++ 且置 LNK_ST_RESPQ_DROP。正确实现下它恒为 0, 且不参与任何控制决策。 */

/* 把一份响应压入环。覆盖最旧条目; 若被覆盖的那条还没发出去 ⇒ 计 respq_drop 并置粘滞状态位,
 * 主机看到该位就知道"有响应被挤掉了, 需要按 tag 重发对应请求"(重发是安全的, 见去重不变式)。
 * ★v3 起只在主循环上下文被调用★(命令分发已整体移出 ISR), 故不需要临界区。 */
void lnk_resp_push(uint8_t tag, const uint8_t payload[LNK_PAYLOAD_BYTES])
{
    lnk_resp_t * e = &lnk_respq[lnk_q_head];
    uint32_t i;
    if ((e->tag != LNK_TAG_NONE) && (e->sent == 0u))
    {
        lnk_diag.respq_drop++;
        lnk_st |= LNK_ST_RESPQ_DROP;
    }
    e->buf[LNK_OFF_SOF] = LNK_SOF_RSP;
    e->buf[LNK_OFF_TAG] = tag;
    e->buf[LNK_OFF_ST]  = 0u;   /* 装载时按当刻 lnk_st 覆写 */
    for (i = 0u; i < LNK_PAYLOAD_BYTES; i++) { e->buf[LNK_OFF_BODY + i] = payload[i]; }
    e->buf[LNK_OFF_CRC] = 0u;   /* 装载时封口 */
    e->tag  = tag;
    e->sent = 0u;
    lnk_q_head = (uint8_t)((lnk_q_head + 1u) % LNK_RESPQ);
}

/* ---- TX 环 ---- */

/* DMA 当前正在读的格号(0..LNK_TXRING-1)。PING 覆盖低半环、PONG 覆盖高半环, 故"当前描述符"给出
 * 半区基址, 该描述符的传输索引(CURR_DATA_NR, 单位=元素=字节)除以帧长给出半区内的格号。
 * 索引在描述符刚完成的那一瞬可能已等于满量程(= 半环格数), 钳到半区末格即可 —— 那一格确实是
 * DMA 最后碰过的格, 用它算出的写入位置仍然安全。 */
static inline uint8_t _lnk_dma_slot(void)
{
    const cy_en_dmac_descriptor_t d =
        Cy_DMAC_Channel_GetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL);
    const uint8_t base = (d == CY_DMAC_DESCRIPTOR_PING) ? 0u : (uint8_t)(LNK_TXRING / 2u);
    uint32_t n = Cy_DMAC_Descriptor_GetCurrentIndex(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL, d)
                 / LNK_FRAME_SIZE;
    if (n >= (LNK_TXRING / 2u)) { n = (LNK_TXRING / 2u) - 1u; }
    return (uint8_t)(base + (uint8_t)n);
}

/* 环形扫描待发队列, 返回槽下标(LNK_RESPQ = 没有待发)。★不在这里推进 lnk_q_send★: 只有真的写进
 * TX 环了才推进游标, 否则一次"窗口用尽"就会把那份响应甩到游标后面, 白白改变发出顺序。
 * 不只看 lnk_q_send 一格: 条目可能因可写窗口用尽而积压在游标之后。 */
static inline uint8_t _lnk_resp_pending(void)
{
    uint8_t idx = lnk_q_send;
    uint32_t n;
    for (n = 0u; n < LNK_RESPQ; n++)
    {
        const lnk_resp_t * c = &lnk_respq[idx];
        if ((c->tag != LNK_TAG_NONE) && (c->sent == 0u)) { return idx; }
        idx = (uint8_t)((idx + 1u) % LNK_RESPQ);
    }
    return (uint8_t)LNK_RESPQ;
}

/* 把一帧内容装进 TX 环的第 w 格并封口。
 * ★st 与 CRC 在这里才最终确定★ 响应可能是几十毫秒前产生的, 但主机拿到的 st 必须是"进入发送环
 * 这一刻"的真值。★帧内不再有任何流控字段★ 余量走 INT2 带外电平(I3), 与发送路径彻底无关。 */
static inline void _lnk_tx_write(uint8_t w, const uint8_t * src)
{
    uint8_t * dst = lnk_txring[w];
    memcpy(dst, src, LNK_FRAME_SIZE);
    dst[LNK_OFF_ST] = lnk_st;
    lnk_seal(dst);
}

/* 把待发响应(没有就用主动状态帧)填进 TX 环里"DMA 还不会立刻读到"的格子。
 * ★这是软件在发送路径上做的全部事情★ 不碰通道、不碰描述符、不碰 TX FIFO(I1)。
 * ★可写窗口★ 距当前读格 d 的距离 < LNK_TX_LEAD 的格子不许写(d 正在被搬、d+1 已被 FIFO 预取,
 * 覆写会把帧撕成两半)。其余 LNK_TXRING - LNK_TX_LEAD 格都可写, 且必须【只往前推、绝不回头】——
 * 回头会覆掉一份还没被搬走的响应。
 * ★为什么允许一次填多格★ v3 把命令分发搬到主循环后, 一轮里可能连着解出好几帧, 每帧一份响应;
 * 若仍像 v2 那样一次只填一格, 剩下的会积在 LNK_RESPQ 里被后来的挤掉。
 * ★队列空时只补一格★ 而且只在读指针已经追过上次写入位置时才补: 若把整个环预填满状态帧,
 * lnk_tx_last_w 就被顶到 d+7, 随后产生的响应要等 7 帧才轮到被搬走 —— 白白多出一批往返延迟。
 * ★本函数迟到无害★ 那只意味着环里那一格还是旧内容: 幂等状态帧, 或 tag 已被主机退役的旧响应。 */
static void lnk_tx_feed(void)
{
    const uint8_t mask = (uint8_t)(LNK_TXRING - 1u);
    const uint8_t d = _lnk_dma_slot();
    uint8_t idx;
    uint8_t w;

    while ((idx = _lnk_resp_pending()) < (uint8_t)LNK_RESPQ)
    {
        w = (uint8_t)((lnk_tx_last_w + 1u) & mask);
        if ((uint8_t)((w - d) & mask) < LNK_TX_LEAD)
        {
            /* 可写窗口用尽: 待发响应留在队里等 DMA 推进。这是链路被主机抽干的直接指标。 */
            lnk_diag.tx_stale++;
            return;
        }
        lnk_tx_last_w = w;
        lnk_respq[idx].sent = 1u;
        lnk_q_send = (uint8_t)((idx + 1u) % LNK_RESPQ);
        _lnk_tx_write(w, lnk_respq[idx].buf);
    }

    /* 空闲期保鲜: 主机不发帧时环里的状态帧会停在旧掩码/旧 gen 上。 */
    if ((uint8_t)((lnk_tx_last_w - d) & mask) < LNK_TX_LEAD)
    {
        w = (uint8_t)((d + LNK_TX_LEAD) & mask);
        lnk_tx_last_w = w;
        _lnk_tx_write(w, lnk_status_frame);
    }
}

/* ★_lnk_tx_realign() 已整体删除★ 它做的是"清 TX FIFO + 两描述符 SetState + 指回 PING", 目的是把
 * 环的格边界与线上的帧边界重新对齐。实测证明这条路在硬件上不可能做对: SCB 的 TX 移位/预取寄存器
 * 没有清除 API, ClearTxFifo 之后线上必然先吐出那个残留字节 ⇒ "重对齐"自己就是永久错位的来源
 * (见 .kiro/context/psoc-link-v3.md §0 的根因链第 3 步)。v3 按 I1 让流永不中断, 相位物理上不可破坏,
 * 按 I2 让接收方从字节流里认帧首 ⇒ 相位根本不进入契约, 也就没有"需要重对齐"这回事。 */

static void spi_dma_init(void)
{
    static const cy_stc_sysint_t dma_int_cfg =
    {
        .intrSrc = cpuss_interrupt_dma_IRQn,
        /* ★优先级 0(最高)★ v3 的 ISR 只推进"已完成的半环数"(见 spi_dma_isr), 它必须在下一个半环
         * 走完之前跑到, 否则 _lnk_rx_written() 只能靠描述符奇偶纠正一拍。CapSense 保持 3。 */
        .intrPriority = 0u,
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
        /* PING 覆盖低半环, PONG(下面改 dstAddress)覆盖高半环, 各 (LNK_RXRING/2) 个整帧。
         * ★与 TX 环完全同构★ 于是 RX 也变成"DMA 自己绕着走的环": 软件永不重挂、永不清 FIFO,
         * 一批 LNK_BURST_MAX 帧连发也不会把双缓冲套圈(v2 的 rx_frame[2] 只能撑两帧)。 */
        .srcAddress = &scb_0_HW->RX_FIFO_RD, .dstAddress = lnk_rxring[0],
        .dataCount = LNK_RXRING_HALF_BYTES, .dataSize = CY_DMAC_BYTE,
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
        /* PING 覆盖低半环, PONG(下面改 srcAddress)覆盖高半环, 各 (LNK_TXRING/2) 个整帧。 */
        .srcAddress = lnk_txring[0], .dstAddress = &scb_0_HW->TX_FIFO_WR,
        .dataCount = (LNK_TXRING / 2u) * LNK_FRAME_SIZE, .dataSize = CY_DMAC_BYTE,
        /* 同 RX 反向: 源(内存)按字节, 目的(TX_FIFO_WR 外设寄存器)必须按字访问。 */
        .srcTransferSize = CY_DMAC_TRANSFER_SIZE_DATA, .srcAddrIncrement = true,
        .dstTransferSize = CY_DMAC_TRANSFER_SIZE_WORD, .dstAddrIncrement = false,
        /* ★与 RX 同构: cpltState=false + flipping=true★ 两个描述符恒有效、完成即翻到另一半环,
         * 于是 DMA 自己绕着整个环不停地喂 TX FIFO —— 软件永不需要重挂, TX FIFO 也就永不会空。
         * 这正是上一版"每帧由 ISR 重挂 + cpltState=true 自失效"的反面: 那种配置下 ISR 一旦被
         * CapSense 的关中断段挡住, FIFO 就是空的, 主机整批读到非法帧只能重发。 */
        .retrigger = CY_DMAC_RETRIG_4CYC, .cpltState = false, .interrupt = false,
        .preemptable = true, .flipping = true, .triggerType = CY_DMAC_SINGLE_ELEMENT,
    };

    /* RX 用 PING/PONG 两个描述符各覆盖半个环: 结构相同, 只有目的(半环基址)不同。 */
    (void)Cy_DMAC_Descriptor_Init(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                  CY_DMAC_DESCRIPTOR_PING, &rx_descriptor_config);
    (void)Cy_DMAC_Descriptor_Init(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                  CY_DMAC_DESCRIPTOR_PONG, &rx_descriptor_config);
    Cy_DMAC_Descriptor_SetDstAddress(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PONG, lnk_rxring[LNK_RXRING / 2u]);
    /* TX 也用 PING/PONG 两个描述符, 结构相同, 只有源(半环基址)不同。 */
    (void)Cy_DMAC_Descriptor_Init(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                  CY_DMAC_DESCRIPTOR_PING, &tx_descriptor_config);
    (void)Cy_DMAC_Descriptor_Init(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                  CY_DMAC_DESCRIPTOR_PONG, &tx_descriptor_config);
    Cy_DMAC_Descriptor_SetSrcAddress(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PONG, lnk_txring[LNK_TXRING / 2u]);
    (void)Cy_DMAC_Channel_Init(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL, &rx_channel_config);
    (void)Cy_DMAC_Channel_Init(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL, &tx_channel_config);
    /* 两个 RX 描述符同时置有效(官方范式), 并把 PING 设为当前描述符。 */
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PING, true);
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PONG, true);
    Cy_DMAC_Channel_SetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING);
    /* 两个 TX 描述符也同时置有效并从 PING 起步(与 RX 同构)。此后 ISR 永不再碰 TX 通道。 */
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PING, true);
    Cy_DMAC_Descriptor_SetState(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PONG, true);
    Cy_DMAC_Channel_SetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING);
    Cy_DMAC_ClearInterrupt(DMAC, CY_DMAC_INTR_CHAN_0 | CY_DMAC_INTR_CHAN_1);
    Cy_DMAC_SetInterruptMask(DMAC, CY_DMAC_INTR_CHAN_0);
    Cy_SysInt_Init(&dma_int_cfg, spi_dma_isr);
    NVIC_ClearPendingIRQ(cpuss_interrupt_dma_IRQn);
    NVIC_EnableIRQ(cpuss_interrupt_dma_IRQn);
    Cy_SCB_SetRxFifoLevel(scb_0_HW, 0u);
    /* ★必须是 FIFO 深度 - 1, 不能等于帧长★ PDL 的合法区间是 level < Cy_SCB_GetFifoSize()(=16),
     * 而 CY_ASSERT_L2 在 Debug 构建里是**真会死循环**的(Cy_SysLib_AssertFailed) —— 传 16 的后果
     * 不是"级别被截断", 而是启动直接卡死在这一行(实测: clk_boot 已写、主循环一次都没进)。
     * 语义上也不需要 16: TX 侧是 DMA 绕环连续喂的字节流, 帧边界由"总共推了多少字节"决定, 与
     * FIFO 当下装了 15 还是 16 个字节无关; 15 只是让补给请求早一个字节抬起, 补给更积极。 */
    Cy_SCB_SetTxFifoLevel(scb_0_HW, LNK_FRAME_SIZE - 1u);

    /* ★这里原有的 CS 上升沿中断配置已整体删除★ 见 I1 与本文件顶部的说明: 那个 ISR 是丢字节的
     * 来源, 而它调用的 TX 重对齐在硬件上做不对。v3 里 P1.3 只作 SCB 的 SELECT0, 不产生中断。 */

    Cy_DMAC_Enable(DMAC);
    Cy_DMAC_Channel_Enable(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL);
    /* ★TX 通道在此使能一次, 此后永不再动★ 环里每一格都已是合法帧(见 spi_slave_init), 所以从这
     * 一刻起 TX FIFO 就恒有内容可供主机读取 —— 哪怕 CPU 随后被 CapSense 关中断段占住整段时间。 */
    Cy_DMAC_Channel_Enable(DMAC, cpuss_0_dmac_0_chan_1_CHANNEL);
}

void spi_slave_init(void)
{
    Cy_SCB_SPI_Init(scb_0_HW, &scb_0_config, &spi_context);
    Cy_SCB_SPI_Enable(scb_0_HW);
    /* ★不再清 FIFO★ Init 已经把 SCB 整块复位过一次, 而"清 FIFO"这个动作本身在 v3 里是被 I1 明令
     * 禁止的(它是 v2 丢字节的唯一来源)。启动瞬间若真有残字节被 DMA 搬进环, 也只会让字节流解析器
     * 多滑几个字节就重新锁定(I2), 不需要、也不许用清 FIFO 去"摆正"。 */
    update_touch_frame();   /* 内含 _lnk_status_seal(): 首帧之前状态帧就必须是合法的 */
    /* ★环必须在使能 TX 通道之前就整个填满合法帧★ DMA 一使能就开始搬运, 它不会等软件; 留下任何
     * 一格全 0 就等于给主机送一个必然 CRC 失败的帧(CRC8 init=0xFF 使全 0 帧一定不合法)。 */
    {
        uint32_t i;
        for (i = 0u; i < LNK_TXRING; i++) { memcpy(lnk_txring[i], lnk_status_frame, LNK_FRAME_SIZE); }
    }
    spi_dma_init();
}

/* ★v3 的 RX ISR 只做两件事: 清中断标志 + 推进"已落了多少个半环"★ 不解析、不分发、不喂 TX 环。
 * 命令分发整体搬到主循环(lnk_rx_drain) —— v2 把整个 switch 跑在这里, 于是任何一条命令的执行时间
 * 都直接变成 SPI 中断延迟, 而重操作的受理还得靠 pending 标志绕一圈。
 * ★两个描述符都保持有效, 此处绝不调 SetState★ 那会清掉正在服务中的描述符的传输索引(I1)。 */
static void spi_dma_isr(void)
{
    uint32_t interrupt = Cy_DMAC_GetInterruptStatusMasked(DMAC);

    if ((interrupt & CY_DMAC_INTR_CHAN_0) != 0u)
    {
        Cy_DMAC_ClearInterrupt(DMAC, CY_DMAC_INTR_CHAN_0);
        lnk_rx_half++;
    }
}

/* ★spi_cs_isr() 已整体删除★ 它是 v2 链路死锁的根因(见 .kiro/context/psoc-link-v3.md §0):
 *   · 它假定自己一定在两次事务之间运行, 但优先级 0 的 DMA ISR 一批要跑多次、主机的阻塞式
 *     request() 又是背靠背 pump ⇒ 它实际清掉的是【下一帧的头几个字节】;
 *   · 字节被丢 ⇒ RX 描述符索引非零 ⇒ 它调 _lnk_tx_realign 去"修" ⇒ SCB 的 TX 移位寄存器清不掉
 *     ⇒ TX 相位永久偏 1 字节 ⇒ 此后主机 100% CRC 失败且没有任何自愈路径。
 * v3 按 I1/I2 从结构上取消了它存在的理由: 谁都不清 FIFO/不动描述符(相位不可破坏), 接收方按
 * SOF+CRC8 从字节流里认帧首(相位不进入契约)。 */


/* DMA 已经落进 RX 环的绝对字节数(mod 2^32)。
 * ★为什么要 ISR 计数与硬件位置交叉验证★
 *   · 只看硬件位置(当前描述符 + CURR_DATA_NR): 环内坐标是 mod 256 的, 环恰好装满时"写位置 ==
 *     读位置"与"环空"完全同形 —— 那会让主循环判定无帧可取, 于是既不消费也不抬 INT2 电平,
 *     而主机在低电平下一个字节都不发(I3) ⇒ 死锁;
 *   · 只看完成中断计数: ISR 迟到就漏计一个半环, 空余帧数被凭空吃掉 8 格 ⇒ 电平白白压低。
 * 故用 ISR 的半环计数给出高位, 用硬件索引给出低位, 再用"当前描述符与半环计数的奇偶是否吻合"
 * 把迟到的那一拍补回来。描述符读两次是防"读的过程中正好翻环"的竞态: 不一致时只认半环边界
 * (宁少不多 —— 少算只是晚一轮取帧, 多算会去解析还没写进来的字节)。 */
static uint32_t _lnk_rx_written(void)
{
    uint32_t halves = lnk_rx_half;
    cy_en_dmac_descriptor_t d1 =
        Cy_DMAC_Channel_GetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL);
    uint32_t idx =
        Cy_DMAC_Descriptor_GetCurrentIndex(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL, d1);
    cy_en_dmac_descriptor_t d2 =
        Cy_DMAC_Channel_GetCurrentDescriptor(DMAC, cpuss_0_dmac_0_chan_0_CHANNEL);

    if (d1 != d2) { d1 = d2; idx = 0u; }
    if (idx > LNK_RXRING_HALF_BYTES) { idx = LNK_RXRING_HALF_BYTES; }
    /* 偶数个半环已完成 ⇒ 当前应当在 PING(低半环)上; 不吻合就是完成中断还没跑到。
     * ★只会补、不会多补★ halves 是在读硬件之前采的, 故"ISR 在这两次采样之间跑了"表现为 halves 偏
     * 小 + 描述符已翻 = 恰好这一条补正; 反过来不存在(硬件不会比 ISR 计数还落后)。 */
    if (((halves & 1u) == 0u) != (d1 == CY_DMAC_DESCRIPTOR_PING)) { halves++; }
    {
        const uint32_t w = (halves * LNK_RXRING_HALF_BYTES) + idx;
        if ((int32_t)(w - lnk_rx_wr) > 0) { lnk_rx_wr = w; }   /* 单调: 见声明处 */
    }
    return lnk_rx_wr;
}

/* 从环里按绝对位置取一整帧(跨环尾自动回卷拼接)。 */
static inline void _lnk_rx_pick(uint8_t * dst, uint32_t abs)
{
    const uint8_t * ring = (const uint8_t *)lnk_rxring;
    const uint32_t off = abs & (LNK_RXRING_BYTES - 1u);
    const uint32_t head = LNK_RXRING_BYTES - off;
    if (head >= LNK_FRAME_SIZE)
    {
        memcpy(dst, &ring[off], LNK_FRAME_SIZE);
    }
    else
    {
        memcpy(dst, &ring[off], head);
        memcpy(&dst[head], ring, LNK_FRAME_SIZE - head);
    }
}

/* ★I3 的流控输出★ 按当刻 RX 环空余帧数刷 INT2 电平: 空余 >= LNK_RX_SPACE_MIN 帧 ⇒ 高, 否则低。
 * ★已占用帧数向上取整★ 环里可能残着不足一帧的字节(主机上一批被 CS 截断), 那一格已经被占了一部分,
 * 必须算成整格 —— 宁可算多不算少: 算多只是早一点压低电平(少发一批), 算少会让主机灌到溢出。
 * ★是电平不是脉冲★ 主机只看当刻值、不看边沿, 故这里必须 Cy_GPIO_Write; 一次多余的 Cy_GPIO_Inv
 * 就会把"满"读成"空"。写前不读回比较: 重复写同一电平在 GPIO 上没有任何副作用, 而多一次读反而更贵。 */
static inline void _lnk_rx_space_publish(void)
{
    const uint32_t used = (uint32_t)(_lnk_rx_written() - lnk_rx_rd);
    const uint32_t used_frames = (used + (LNK_FRAME_SIZE - 1u)) / LNK_FRAME_SIZE;
    const uint32_t free_frames = (used_frames >= LNK_RXRING) ? 0u : (LNK_RXRING - used_frames);
    Cy_GPIO_Write(SENSOR_INT2_PORT, SENSOR_INT2_NUM,
                  (free_frames >= LNK_RX_SPACE_MIN) ? 1u : 0u);
}

/* ★I2 的字节流解析器★ 主循环调用, 是 v3 唯一的命令入口。
 * 取 LNK_FRAME_SIZE 字节做 lnk_frame_ok: 通过就执行并把读位置前进一整帧, 不通过就【只前进一个
 * 字节】。⇒ 从机不假设主机的 CS 窗与自己的格边界重合: 主机哪次事务被截断, 至多一帧之内就能重新
 * 锁定帧首, 而且不需要任何"重同步"动作(那正是 v2 的病灶)。
 * ★帧内没有任何流控字段可维护★ 余量只在末尾的 _lnk_rx_space_publish 里以电平形式给出(I3)。
 * ★没有任何背压★ 契约保证 LNK_RESPQ >= LNK_RXRING, 响应槽永远够, 取帧循环不许停(否则第二类死锁)。
 * 返回本轮是否取走过帧(纯诊断用: INT1 已回退为"只表示新代数", 不再由这里翻转)。 */
bool lnk_rx_drain(void)
{
    const uint32_t wr = _lnk_rx_written();
    uint8_t frame[LNK_FRAME_SIZE];
    bool got = false;

    /* DMA 已经把最老的未读字节盖掉了。I3 保证这不会发生(电平阈值留了一整批, 最坏也恰好用满环);
     * 真发生了只能丢到"最老仍完整"的位置, 继续按撕裂内容解析毫无意义。 */
    if ((uint32_t)(wr - lnk_rx_rd) > LNK_RXRING_BYTES)
    {
        lnk_rx_rd = wr - LNK_RXRING_BYTES;
    }

    while ((uint32_t)(wr - lnk_rx_rd) >= LNK_FRAME_SIZE)
    {
        _lnk_rx_pick(frame, lnk_rx_rd);
        if (lnk_frame_ok(frame, LNK_SOF_REQ))
        {
            lnk_rx_rd += LNK_FRAME_SIZE;
            lnk_diag.rx_ok++;
            got = true;
            /* ★带外镜像★ 链路计数平时只经带内 DIAG/GLOBAL_GET 上报, 而"链路根本没建起来"时带内
             * 读不出来 —— 那正是最需要它们的时刻。故镜像到 SWD 导出槽(这两槽原本记 PARAM_SET
             * 相关, 而 PARAM_SET 只在 provisioning 期间才有量, 与建链诊断互不干扰)。 */
            spi_dbg.clk_set_cnt = lnk_diag.rx_ok;
            lnk_deliver(frame);
        }
        else
        {
            lnk_rx_rd += 1u;
            lnk_diag.rx_reject++;
            lnk_st |= LNK_ST_RX_REJECT;
            spi_dbg.clk_set_last = lnk_diag.rx_reject;
        }
    }

    lnk_tx_feed();
    /* ★无条件刷★ 本轮一帧都没取到时电平同样要刷: 上一轮压低之后, 空余是靠"这一轮什么都没进来"
     * 才恢复的, 不刷就永远抬不回高电平 —— 而主机在低电平下一个字节都不发(死锁)。 */
    _lnk_rx_space_publish();
    return got;
}

/* 链路态复位: 原 main() 启动序列里的那一段, 逐字搬来(TX 环内容仍由 spi_slave_init 填)。 */
void lnk_wire_reset(void)
{
    /* 链路态复位: 待发队列与去重缓存全空(tag=0)、游标归零、诊断归零。PROVISION 从一开始就为真
     * —— 主机在第一帧就能看出"还没开始扫描", 不必靠猜。
     * TX 环不在此填内容: 它必须填"当刻状态帧", 而状态帧要等 update_touch_frame 才成形,
     * 故由 spi_slave_init 在使能 TX 通道之前统一填满(见那里的说明)。 */
    memset(lnk_respq, 0, sizeof(lnk_respq));
    memset(lnk_txring, 0, sizeof(lnk_txring));
    memset(lnk_rxring, 0, sizeof(lnk_rxring));
    memset(lnk_status_frame, 0, sizeof(lnk_status_frame));
    lnk_q_head = 0u;
    lnk_q_send = 0u;
    /* 第一次 feed 正好落在 d(=0) + LNK_TX_LEAD 那一格。 */
    lnk_tx_last_w = (uint8_t)(LNK_TX_LEAD - 1u);
    /* RX 环游标: 半环计数与读位置都从 0 起, 与 DMA 从 PING 起步一致。全 0 的环内容会被字节流
     * 解析器当成非法帧逐字节滑过(CRC8 init=0xFF 使全 0 帧必然不合法), 不会误执行任何命令。 */
    lnk_rx_half = 0u;
    lnk_rx_wr = 0u;
    lnk_rx_rd = 0u;
    /* ★这里不碰 INT2★ 电平的初值由 main() 的 Cy_GPIO_Pin_FastInit(高)给出, 而本函数在 GPIO 初始化
     * 之前就跑完了; 复位后环是空的 ⇒ 高电平本身就是正确的当刻值, 第一次 lnk_rx_drain 再刷一遍。 */
    lnk_st = LNK_ST_PROVISION;
}
