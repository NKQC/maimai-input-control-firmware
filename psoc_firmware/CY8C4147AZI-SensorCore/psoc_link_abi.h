/******************************************************************************
 * psoc_link_abi.h — RP2040 <-> PSoC SPI 链路契约 (LINK v2)
 *
 * ★唯一真相源★ PSoC 固件(main.c)与 RP2040 固件(protocol/psoc 目录)都直接包含本文件。
 * 帧格式、命令码、状态位、容量常量只在这里定义一次 —— 此前它们散在 PSoC 的 #define、
 * RP 的 psoc_types.h 与 psoc_algo.h 三处, 漏改任何一处都表现为"上传成功却跑旧代码"。
 *
 * ============================ 设计要点 ============================
 * LINK v2 把"请求/应答配对"从**约定**变成**协议保证**, 取代 v1 的单槽应答流水。
 *
 * v1 的根因: 从机每收一帧就把一份响应压进 TX FIFO, 主机每笔事务取走一份, 配对完全靠
 * 位置。任何一次错位(帧丢失 / ISR 迟到 / CS 重同步)都会让此后所有响应对不上号, 而且
 * 错位有"多一份"和"少一份"两个相反方向, 修法相反 —— 无法在主机侧判定方向。
 *
 * v2 的三条不变式(全部与时序无关, 这是"响应稳定 + 延迟稳定"的来源):
 *   ① 每帧自证身份: SOF + tag + CRC8。位置不再承载任何信息。
 *   ② 主机为每条在途请求分配唯一 tag, 收到匹配 tag 的响应即完成并退役该 tag。
 *      迟到帧、重复帧、非请求帧一律丢弃 —— 陈旧数据无法冒充本轮结果。
 *   ③ 从机对已执行过的 tag 做**去重**(响应环兼作去重缓存): 重发同一 tag 只会重发
 *      已保存的响应, 绝不重复执行。⇒ 任何命令的重传都是安全的, 包括非幂等命令。
 *
 * 由此**不存在任何**帧间等待要求: 主机可以背靠背连发请求、乱序收取响应、把不同用途的
 * 事务任意掺杂。v1 的 PSOC_ISR_FRAME_US=150us 等待、逐页交替重试、整轮重传、
 * "一个分块必须在同一次调用内连续完成"等全部随之退役。
 *
 * ★v3 补充★ 下面这段 v2 的论述在"TX 必须是 DMA 自走的环"这一点上仍然成立, 但它最后那句
 * "帧边界的唯一权威是 CS / CS 上升沿 ISR 同时重排 RX 与 TX 的对齐"已被实测推翻并删除 ——
 * 见文件顶部 I1/I2: CS ISR 本身就是丢字节的来源, 而 TX 重对齐在硬件上做不对。
 *
 * ============================ 双端 DMA 环形缓冲 ============================
 * ★为什么从机的 TX 必须是"DMA 自己走的环", 而不是"每帧由 ISR 重挂"★
 * 每帧重挂的版本里, 响应能不能被取走取决于"ISR 有没有在主机下一帧到来之前跑完"。而 CapSense
 * 中间件会整段关中断, 那期间 ISR 根本进不来 ⇒ TX FIFO 空 ⇒ 主机读到非法帧 ⇒ 只能重发。
 * 实测这条路上重发会成片发生, 把时间全烧在重试上(满容量上传随机某页重试预算耗尽而失败)。
 * v2 的做法: TX 侧是 LNK_TXRING 个整帧组成的环, DMAC 用 PING/PONG 各覆盖半环、两个描述符
 * 恒有效、完成不失效(cpltState=false)、flipping 交替 ⇒ **DMA 自己不停地把环喂进 TX FIFO,
 * 软件永不参与发送路径**。于是:
 *   · FIFO 永远不空 ⇒ 主机每一帧都能读到一个**格式合法**的帧, 不存在"读空"这种失败;
 *   · ISR 迟到只意味着环里那一格还是旧内容 —— 旧内容要么是主动状态帧(幂等, 重复无害),
 *     要么是 tag 已退役的旧响应(主机直接丢弃) ⇒ **迟到不再产生任何需要重发的后果**;
 *   · ISR 每收一帧只做一件事: 把"下一格"(DMA 读指针 + LNK_TX_LEAD)填成待发响应或状态帧。
 * 主机侧对称: 一次 CS 内用一条 DMA 连发/连收 K 个整帧, 事后再按 tag **异步对账、乱序取数**,
 * 不为任何单帧同步等待。
 *
 * ★帧边界不由 CS 决定, 由内容决定(v3 修正 v2 的这一条)★
 * v2 曾规定"帧边界的唯一权威是 CS", 并在 CS 上升沿 ISR 里重排 RX/TX 对齐。实测证明这条路
 * 走不通: 那个 ISR 会在下一次事务已经开始之后才执行(优先级 0 的 DMA ISR 一批要跑 8 次, 而
 * 主机的阻塞式 request() 是背靠背 pump), 于是它清掉的是新帧的前几个字节 —— 它自己就是丢
 * 字节的来源; 而它随后调用的 TX 重对齐, 因为清不掉 SCB 的 TX 移位/预取寄存器, 必然留下一个
 * 残留字节, 把 TX 相位永久偏掉。v3 的做法是 I1 + I2: 谁都不许清 FIFO/动描述符(于是字节流
 * 天然连续、相位不可破坏), 接收方按 SOF + CRC8 从字节流里认帧首(于是相位根本不进入契约)。
 *
 * ============================ 轻量信封 ============================
 * 所有命令共用**同一个**定长信封: 3 字节头(SOF / tag / cmd 或 st) + 定长体 + 1 字节 CRC8。
 * 体一律**向上补齐**到 LNK_ARG_BYTES / LNK_PAYLOAD_BYTES, 不为省几个字节做变长或分帧 ——
 * 帧大点带来的补发/往返次数下降, 收益远大于那点带宽。信封里没有长度字段、没有转义、没有
 * 分片: 收帧就是"取 LNK_FRAME_SIZE 字节, 查 SOF 与 CRC8, 看 tag", 两端各一份实现。
 ******************************************************************************/

#if !defined(PSOC_LINK_ABI_H)
#define PSOC_LINK_ABI_H

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

/* 链路版本: 两端启动握手(LNK_CMD_PING)回显, 不一致即拒绝建链而不是带着错帧长跑。 */
#define LNK_ABI_VERSION      (3u)

/* ============================ LINK v3 的三条硬不变式 ============================
 * v2 实测故障(见 .kiro/context/psoc-link-v3.md §0)证明: 把"帧边界"寄托在时序约定上
 * ——"主机每次 CS 恰好整数帧" + "从机 TX 字节流的相位必须与 CS 窗重合"——是不成立的,
 * 而且它唯一的补救手段在硬件上做不对: SCB 的 TX 移位/预取寄存器没有清除 API, 所以
 * `ClearTxFifo` 之后线上必然先吐出那个残留字节, "重对齐"自己就是永久错位的来源。
 * v3 因此改成三条与时序无关的不变式, 任何一条都不允许用重试/复位/吸收去兜:
 *
 * ★I1 流永不中断★ 两侧收发 DMA 一经使能就永不停、永不清 FIFO、永不改描述符。
 *   ⇒ 字节既不会被丢也不会被插入 ⇒ 相位在物理上就不可能被破坏。
 *   直接后果: 从机的 CS 上升沿 ISR 与 TX 重对齐**整个删除**(它们是 v2 唯一的丢字节来源)。
 *
 * ★I2 帧边界由内容决定★ 接收方是字节流解析器: 跨事务保留残字节, 按 SOF + CRC8 找帧首,
 *   连续两帧都通过才算锁定。它不假设任何相位 —— 这是**框架定义**, 不是错位后的补救。
 *
 * ★I3 流控是一条硬件电平线(INT2), 不是任何形式的计数窗口★
 *   从机在 P1.5(SENSOR-INT2 → RP2040 GPIO22)上输出**电平**: RX 环空余 ≥ LNK_RX_SPACE_MIN 帧时为高,
 *   否则为低。主机每次组批**先读这条线**: 高则本批最多 LNK_BURST_MAX 帧, 低则一个字节都不发。
 *
 *   ★为什么不能用"从机回报余量/已消费计数"这类窗口★(这条路已经推演到死, 留证以免重走)
 *   SPI 上主机是时钟源: 主机**不时钟就收不到任何回报**。于是任何"回报制"窗口都有一个信息学死锁 ——
 *   窗口一满, 主机按规则停止时钟; 而更新后的余量只能搭在响应帧里回来, 响应帧只能被时钟搬出来
 *   ⇒ 主机永远等不到窗口重开, 从机也永远收不到新帧。即便让主机"满了也发一帧去探", 那一帧本身又
 *   占一格, 探测无法重复; 即便靠边沿通知唤醒, 唤醒后仍然得时钟才能读到新余量 —— 死锁只是被推迟。
 *   序号历元问题(两侧各记一份计数、任一侧复位就错位)是同一条路上的第二个坑, 一并作废。
 *   ★电平线没有这个问题★: 它是**带外**的, 主机读它不需要时钟任何字节, 也没有陈旧值。
 *
 *   ★阈值留一整批, 使"电平陈旧"也不会溢出★ 从机只在主循环里刷这条线, 所以它可能"该低了还是高"。
 *   把阈值取成"空余 ≥ 一整批"就把这点陈旧封死了: 最坏情况是主机按陈旧的高电平又灌进一整批,
 *   占用量至多 LNK_RX_SPACE_MIN + LNK_BURST_MAX = LNK_RXRING ⇒ 恰好用满, 一格都不会溢出。
 *   这是容量不变式(LNK_RXRING = LNK_RX_SPACE_MIN + LNK_BURST_MAX), 不是调优参数。
 *
 *   ★从机的取帧循环不许因为"没有空响应槽"而停下★ 那是另一个死锁: 响应只能靠主机时钟搬走,
 *   而主机在电平低时不时钟 ⇒ 响应槽永不释放 ⇒ 从机永不取帧 ⇒ 电平永不抬高。故契约要求
 *   LNK_RESPQ >= LNK_RXRING: 在途请求上限就是环深, 响应槽必然够, 取帧循环没有任何理由停下。
 *
 *   ⇒ 溢出、丢帧、错位、死锁全部在结构上不可能发生, 不需要任何重试/复位/超时来兜底。
 *
 * ★以下 I3 旧方案(序号回显窗口)已作废, 保留说明只为记录"为什么不能那样做"★
 *   主机给每一个时钟出去的请求帧盖一个序号 `seq`(帧内 LNK_OFF_SEQ, 0..LNK_SEQ_MAX 循环);
 *   从机在每一份响应的 LNK_OFF_RXSEQ 里**回显它最近一次从 RX 环取走的那一帧的 seq**。
 *   在途未消费帧数 = (uint8)(主机最后发出的 seq - 回显值), 主机只在它 < LNK_RXRING 时才发下一帧。
 *
 *   ★为什么必须是"回显主机的序号", 不能是"从机自己的已消费计数"★
 *   两侧各记一份计数时, 任何一侧单独复位(PSoC 被 XRES / RP 重启)都会让两个计数落在不同历元,
 *   差值随即变成 0..255 里的任意值 —— 有 15/256 的概率恰好落在"窗口已满"上, 而窗口一满主机就
 *   不再时钟任何字节, 从机也就永远不会再取帧、永远不会再更新计数 ⇒ **永久死锁**。这正是本项目
 *   要根除的那一类"一次扰动就永久卡死"。序号空间只由主机拥有、从机纯回显, 历元问题在定义上
 *   就不存在: 回显值永远是主机自己发过的某个数, 从机复位只会让它退回 LNK_SEQ_NONE。
 *   它同时顺手解决了"某次事务被截断"的账目问题: 从机回显的是**它真正消费到的那一帧**,
 *   所以主机永远不需要猜"上过线的字节从机到底吃进去几帧"。
 *
 *   ★从机复位后的第一拍★ 回显 LNK_SEQ_NONE 表示"本代次还没消费过任何帧"。主机见到它就把在途
 *   记为 0(刚复位的从机 RX 环必然是空的), 于是窗口自然重开, 不需要任何握手或超时。
 *
 *   ★不许出现"窗口满了就少发一点试试"或定时空转轮询★ 窗口耗尽时主机一个字节都不发, 改为等
 *   INT1 翻转 —— 从机每取走 ≥1 帧就翻转它。⇒ 溢出与丢帧在结构上不可能发生, 不需要任何重发机制。
 *
 *   ★不许让从机因为"没有空响应槽"而停止取帧★ 那会形成第二个死锁: 响应只能靠主机时钟才能被搬走,
 *   而主机在窗口满时不时钟 ⇒ 响应槽永不释放 ⇒ 从机永不取帧 ⇒ 窗口永不重开。因此契约规定
 *   LNK_RESPQ >= LNK_RXRING: 主机遵守窗口时在途请求最多 LNK_RXRING 条, 响应槽必然够, 从机的
 *   取帧循环于是**没有任何理由停下来**。这是一条容量不变式, 不是调优参数, 改小任何一边都会复活死锁。
 * ============================================================================ */

/* ---------------------------------------------------------------- 帧 ---- */
/* ★16 字节★ = SCB FIFO 深度(SCB_EZ_DATA_NR/2, 8 位模式满深度), 也是 2 的幂 ⇒ 环下标用移位。
 * 取大不取小: 体一律补齐, 用帧长换往返次数。
 *   · 请求参数 12 字节 ⇒ ALGO_PAGE 一帧带 页号 u16 + 10 字节算法码, 4KB 只需 410 帧
 *     (v1 是每帧 3 字节 × 2 次事务 × 150us 等待, 1366 帧);
 *   · 响应载荷 12 字节 ⇒ 一帧一通道快照(用 8 字节, 余 4 补 0), 全通道 36 帧;
 *     ALGO_TRACE 一帧回齐 report[0..3](v1 要 4 次事务轮转)。
 * 补齐带来的那点空转字节, 换来的是"再也不用为了塞进小帧而多跑几百次往返"。 */
/* ★帧内不含任何流控字段★ 流控走带外电平线(见 I3), 所以收发的 12 字节体一格都不用让出来。 */
#define LNK_FRAME_SIZE       (16u)
#define LNK_ARG_BYTES        (12u)   /* 请求 [3..14], 不足一律补 0 */
#define LNK_PAYLOAD_BYTES    (12u)   /* 响应 [3..14], 不足一律补 0 */

/* 收发用**不同** SOF: 一个错位/半帧永远不可能被当成合法的反向帧。 */
#define LNK_SOF_REQ          (0xA5u)
#define LNK_SOF_RSP          (0x5Au)

/* 帧内字节偏移(两个方向共用前 3 字节的位置, 只是语义不同)。 */
#define LNK_OFF_SOF          (0u)
#define LNK_OFF_TAG          (1u)
#define LNK_OFF_CMD          (2u)    /* 请求: 命令码 */
#define LNK_OFF_ST           (2u)    /* 响应: 从机状态字节(见 LNK_ST_*), 每帧都带 */
#define LNK_OFF_BODY         (3u)    /* 请求参数 / 响应载荷 */
/* ★流控不占帧内字节★ 见 I3: 余量由 INT2 电平带外给出, 主机读它不需要时钟任何字节。
 * 阈值 = 一整批。取这个值使"从机只在主循环刷电平、因此电平可能陈旧"也不会溢出:
 * 最坏是主机按陈旧的高电平又灌一整批, 占用量至多 LNK_RX_SPACE_MIN + LNK_BURST_MAX = LNK_RXRING。 */
#define LNK_RX_SPACE_MIN     (8u)
#define LNK_OFF_CRC          (15u)   /* crc8 覆盖 [0..14] */

/* ---------------------------------------------------------------- tag ---- */
/* tag 0 保留: 请求侧 = 纯轮询(不携带命令, 只为给从机一次吐出响应的机会);
 *             响应侧 = 主动状态帧(携带实时触控掩码 + 代数), 不属于任何请求。
 * 1..LNK_TAG_MAX 由主机单调循环分配。循环长度(127)远大于从机去重缓存深度(LNK_RESPQ),
 * 故一个 tag 被复用时它的旧响应必然早已被挤出缓存 —— 不会误判为重复请求。 */
#define LNK_TAG_NONE         (0u)
#define LNK_TAG_MIN          (1u)
#define LNK_TAG_MAX          (127u)

/* --------------------------------------------------------- 状态字节 ---- */
/* 每一份响应(含状态帧)都带 st。主机因此无需为"设备忙不忙"单独发命令 —— 这是 v1 里
 * GET_STATS 被高频轮询、反过来抢占 PSoC 主循环空窗的根源。 */
#define LNK_ST_OP_BUSY       (1u << 0)  /* 主循环正在执行重操作(APPLY/CALIBRATE/...) */
#define LNK_ST_ALGO_VALID    (1u << 1)  /* JIT 槽内有已通过 CRC 的算法 */
#define LNK_ST_ALGO_UPLOAD   (1u << 2)  /* 正在上传(此时 valid/len 属于上一份, 不可采信) */
#define LNK_ST_PROVISION     (1u << 3)  /* 启动 provisioning 未完成(尚未开始扫描) */
#define LNK_ST_RESPQ_DROP    (1u << 4)  /* 曾有未发出的响应被挤掉 ⇒ 主机应重发对应请求 */
#define LNK_ST_RX_REJECT     (1u << 5)  /* 曾收到 SOF/CRC 不合法的请求帧 */
#define LNK_ST_SNAP_READY    (1u << 6)  /* 锁存快照已就绪, LNK_CMD_SNAP_CH 可读 */

/* ------------------------------------------------------------ 命令码 ---- */
/* 分组: 0x0x 链路 / 0x1x 快照 / 0x2x 参数与状态 / 0x3x 重操作 / 0x4x JIT 算法。
 * ★命令码不再复用/回填空洞★ v1 的 FOCUS_SCAN(0x49) 插在算法域中间, 逼得 ABI v2 从
 * 0x4A 起排 —— 这种历史包袱在 v2 里一次清掉(该功能整条链路本就无调用者, 已删除)。 */
#define LNK_CMD_STATUS       (0x00u) /* → mask[5], gen u16, spare        (= 主动状态帧内容) */
#define LNK_CMD_PING         (0x01u) /* → fw u32, abi u8, frame u8, ch_count u8, spare */
#define LNK_CMD_INDICATOR_ON (0x02u) /* 点亮白灯(烧录前指示), 无载荷 */
#define LNK_CMD_DIAG         (0x03u) /* → rx_reject u16, respq_drop u16, cs_resync u16, rx_left u16 */

#define LNK_CMD_SNAP_LATCH   (0x10u) /* ★延迟应答★ 主循环完成 252B 锁存后才回:
                                      *   → gen u16, valid u8, ch_count u8, spare*4
                                      * 拿到本响应即保证随后任意顺序的 SNAP_CH 属于同一代。 */
#define LNK_CMD_SNAP_CH      (0x11u) /* args: ch → ch, raw u16, bsln u16, diff i16, status u8 */

#define LNK_CMD_PARAM_SET    (0x20u) /* args: ch, id, val u32 → ch, id, actual u32 */
#define LNK_CMD_PARAM_GET    (0x21u) /* args: ch, id         → ch, id, val u32 */
#define LNK_CMD_GLOBAL_SET   (0x22u) /* args: id, 0, val u32 → id, 0, actual u32 */
#define LNK_CMD_GLOBAL_GET   (0x23u) /* args: id             → id, 0, val u32 */
#define LNK_CMD_RAW_GET      (0x24u) /* args: ch             → ch, raw u16 */
#define LNK_CMD_CP_GET       (0x25u) /* args: ch             → ch, cp u32 */
#define LNK_CMD_STATS        (0x26u) /* → scan_count u32, ms_tick u32 (busy 见 st.OP_BUSY) */
#define LNK_CMD_MODE_SET     (0x27u) /* args: mode           → mode */

/* 重操作: 响应只表示"已受理并置 pending"(ISR 内不可能做完), 真实完成由 st.OP_BUSY 由 1→0
 * 判定。★受理与完成分离在 v2 里是可靠的★: tag 保证这份 ACK 属于本次请求, 不会像 v1 那样
 * "命令在 SPI 上丢了 + busy 恰好因上一条为 1"就把上一轮结果冒充成本轮成功。 */
#define LNK_CMD_APPLY          (0x30u) /* args: - */
#define LNK_CMD_QUICK_APPLY    (0x31u) /* args: ch, gain, div → ch, gain, div, accepted */
#define LNK_CMD_CALIBRATE      (0x32u) /* args: ch (0xFF=全通道) → ch */
#define LNK_CMD_BASELINE_RESET (0x33u) /* args: ch (0xFF=全通道) → ch */
#define LNK_CMD_GLOBAL_COMMIT  (0x34u) /* args: - */
#define LNK_CMD_MEASURE_CP     (0x35u) /* args: - */
#define LNK_CMD_AUTO_TUNE      (0x36u) /* args: ch, pref(1..7) → ch, pref */
#define LNK_CMD_AUTO_TUNE_GET  (0x37u) /* → result u8, ch u8, div u16, phase u8, step u8 */

#define LNK_CMD_ALGO_BEGIN     (0x40u) /* args: len u16 → len u16 */
/* args: page u16 + 10 字节码; 绝对寻址 offset = page * LNK_ALGO_PAGE_BYTES(末页按槽容量截断)。
 * 绝对寻址在 v2 里不再是为了"重发幂等"(去重已保证), 而是为了让主机能**单独补发某一页**
 * 而不必整轮重传 —— 这是 v1 唯一没有的能力。 */
#define LNK_CMD_ALGO_PAGE      (0x41u) /* → page u16, offset u16 */
#define LNK_CMD_ALGO_END       (0x42u) /* args: crc16 u16 → accepted u8, 0, len u16 */
#define LNK_CMD_ALGO_INFO      (0x43u) /* → valid u8, uploading u8, len u16, slot_crc u16, reject u16 */
#define LNK_CMD_ALGO_CAPS      (0x44u) /* → slot u16, heap u16, heap_used u16, page_bytes u8, abi u8 */
#define LNK_CMD_ALGO_ROM_SET   (0x45u) /* args: ch, 0, rom u16 → ch, 0, rom u16 */
#define LNK_CMD_ALGO_ROM_GET   (0x46u) /* args: ch → ch, 0, rom u16 */
#define LNK_CMD_ALGO_CFG_SET   (0x47u) /* args: idx, val → idx, val */
#define LNK_CMD_ALGO_CFG_GET   (0x48u) /* args: idx → idx, val */
#define LNK_CMD_ALGO_CFGCH_SET (0x49u) /* args: ch, idx, val → ch, idx, val */
#define LNK_CMD_ALGO_CFGCH_GET (0x4Au) /* args: ch, idx → ch, idx, val */
/* → report[0..3] u16×4(占载荷前 8 字节, 余 4 补 0)。★out_active 不在这里★: 它逐通道的值已经
 * 就是状态帧里那份触控掩码的对应位(掩码本身由算法输出合成), 再传一遍纯属冗余。 */
#define LNK_CMD_ALGO_TRACE     (0x4Bu) /* args: ch → report u16 × 4 */

/* ------------------------------------------------------------ 容量常量 ---- */
/* 这些以前在 PSoC / RP / 上位机三处各写一份。现在 PSoC 与 RP 共用本文件;
 * 上位机仍需自己声明, 但它已改为**从设备读** LNK_CMD_ALGO_CAPS, 不再硬编码。 */
#define LNK_CHANNEL_COUNT      (36u)
#define LNK_CH_ALL             (0xFFu) /* CALIBRATE / BASELINE_RESET / AUTO_TUNE 的全通道哨兵 */
#define LNK_ALGO_SLOT_SIZE     (4096u) /* 可执行 RAM 槽 */
#define LNK_ALGO_HEAP_SIZE     (256u)  /* 算法共享暂存堆 */
#define LNK_ALGO_PAGE_BYTES    (10u)   /* 每帧携带的算法字节数(= LNK_ARG_BYTES - 页号 2B) */
#define LNK_ALGO_PAGE_COUNT    ((LNK_ALGO_SLOT_SIZE + LNK_ALGO_PAGE_BYTES - 1u) / LNK_ALGO_PAGE_BYTES)

/* 快照打包: 一帧一通道, 载荷用 8 字节(ch + raw/bsln/diff/status), 余 4 字节补 0。 */
#define LNK_SNAP_BYTES_PER_CH  (8u)

/* ★从机 TX 环★ DMA 自己走的整帧环, 软件不参与发送。必须是 2 的幂且偶数半区(PING/PONG 各半)。
 * 8 × 16B = 128B。深度只需覆盖"ISR 停摆期间主机能连着抽走多少帧": 停摆时读到的是环里旧内容
 * (幂等状态帧 / 已退役 tag 的旧响应), 都无害, 所以环深不影响正确性, 只影响新鲜度。 */
#define LNK_TXRING             (8u)
/* ★从机 RX 环★ 同样是 DMA 自己绕着走的整帧环(PING/PONG 各覆盖半环, 恒有效、完成不失效)。
 * v2 这里只有 rx_frame[2] 双缓冲, 而主机一批能连发 8 帧 ⇒ ISR 被拖过 128us 就会被 DMA 套圈
 * 丢帧。v3 把它扩成环并把命令分发移出 ISR: ISR 只推进写索引, 主循环按信封投递业务。
 * ★环深由流控不变式定死★ LNK_RXRING = LNK_RX_SPACE_MIN + LNK_BURST_MAX = 16 帧 = 256B。
 * 这不是调优参数: 阈值那一半是"电平陈旧时的安全余量", 批量那一半是"一次最多灌进来多少"。
 * 改小任一半都会让陈旧的高电平灌出溢出; 改大只是白占 PSoC 那点 RAM 余量(且逼着 LNK_RESPQ 一起大)。
 * 必须是 2 的幂(环下标用掩码)。 */
#define LNK_RXRING             (LNK_RX_SPACE_MIN + LNK_BURST_MAX)
/* ISR 写入位置相对 DMA 读指针的领先格数。必须 ≥2: DMA 会比线上进度提前最多一整帧(FIFO 预取),
 * 且不能覆写它正在搬的那一格(会把帧撕成两半)。 */
#define LNK_TX_LEAD            (2u)
/* 从机待发响应队列深度(仅缓冲"已产出但还没轮到写进环"的响应, 不再兼作去重缓存)。
 * ★必须 >= LNK_RXRING, 这是容量不变式不是调优参数★
 * 从机的取帧循环绝不允许因为"没有空响应槽"而停下来 —— 响应只能靠主机时钟搬走, 而主机在窗口满时
 * 一个字节都不发, 于是"停止取帧"会让响应槽永不释放、窗口永不重开, 形成第二类永久死锁。
 * 主机遵守 I3 时在途请求最多 LNK_RXRING 条, 因此只要槽数不少于环深, 就永远不会出现无槽可用,
 * 取帧循环也就没有任何理由停下。改小任一边都会复活那个死锁。 */
#define LNK_RESPQ              (LNK_RXRING)
/* ★去重缓存只保护非幂等命令★ 读类/写参数/算法分页(绝对寻址)重执行结果完全相同, 不需要去重;
 * 真正怕重复执行的只有重操作与 ALGO_BEGIN/END(重跑一次 APPLY 是十几秒的重校准)。这些命令同一
 * 时刻最多一两条在途, 故 4 格足够, 省下的 RAM 留给 TX 环。 */
#define LNK_DEDUP_SLOTS        (4u)
/* 主机事务表深度(在途请求上限)。 */
#define LNK_TXN_SLOTS          (16u)
/* 主机一次 CS 内连发/连收的最大帧数(一条 DMA 搞定, 事后按 tag 异步对账)。
 * 与 LNK_TXRING 同量级: 再大也只是让从机的响应更晚出现在同一批里, 不会更快。 */
#define LNK_BURST_MAX          (8u)

/* --------------------------------------------------------------- CRC8 ---- */
/* CRC-8/ATM(poly 0x07, init 0xFF), 半字节查表 ⇒ 11 字节约 22 次查表, ISR 里可忽略。
 * ★为什么要校验而不只靠 SOF★ 整条 SPI 走双向自适应电平位移器, 采样点是靠延后 4 拍
 * 硬调出来的。SOF 只能挡住整字节错位, 挡不住单比特翻转; 而单比特翻转打在页号或参数上
 * 就是一次静默写歪 —— 那正是必须根除的一类故障。init 用 0xFF 而非 0x00, 使全 0 帧
 * (TX FIFO 空被主机读空时的典型形态)必然校验失败。 */
static inline uint8_t lnk_crc8(const uint8_t *data, uint32_t len)
{
    static const uint8_t tbl[16] = {
        0x00u, 0x07u, 0x0Eu, 0x09u, 0x1Cu, 0x1Bu, 0x12u, 0x15u,
        0x38u, 0x3Fu, 0x36u, 0x31u, 0x24u, 0x23u, 0x2Au, 0x2Du
    };
    uint8_t crc = 0xFFu;
    uint32_t i;
    for (i = 0u; i < len; i++)
    {
        crc = (uint8_t)(crc ^ data[i]);
        crc = (uint8_t)((uint8_t)(crc << 4u) ^ tbl[crc >> 4u]);
        crc = (uint8_t)((uint8_t)(crc << 4u) ^ tbl[crc >> 4u]);
    }
    return crc;
}

/* 帧封口 / 校验。两端只用这两个函数, 不许任何地方手写 CRC 位置或 SOF 比较。 */
static inline void lnk_seal(uint8_t *frame)
{
    frame[LNK_OFF_CRC] = lnk_crc8(frame, LNK_OFF_CRC);
}

static inline int lnk_frame_ok(const uint8_t *frame, uint8_t expect_sof)
{
    return (frame[LNK_OFF_SOF] == expect_sof) &&
           (frame[LNK_OFF_CRC] == lnk_crc8(frame, LNK_OFF_CRC));
}

/* 小端取值助手(载荷/参数里所有多字节量一律小端)。 */
static inline uint16_t lnk_rd16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8u));
}
static inline uint32_t lnk_rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) |
           ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}
static inline void lnk_wr16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)(v >> 8u);
}
static inline void lnk_wr32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);         p[1] = (uint8_t)((v >> 8u) & 0xFFu);
    p[2] = (uint8_t)((v >> 16u) & 0xFFu); p[3] = (uint8_t)((v >> 24u) & 0xFFu);
}

#if defined(__cplusplus)
}
#endif

#endif /* PSOC_LINK_ABI_H */
