#pragma once

#include <stdint.h>
#include <stddef.h>
#include "psoc_types.h"
#include "psoc_link_abi.h"
#include "../../hal/dma/hal_dma.h"

class HAL_PIO;

/**
 * PsocLink - RP2040 ↔ PSoC 链路主机(LINK v2, 非单例, 由 Psoc 门面持有)
 *
 * ★唯一线级原语★ = 一次 CS 内用一条 DMA 连发/连收 K 个整帧(K ≤ LNK_BURST_MAX), 中间不拆、不等待,
 * 事后再按 tag **异步对账、乱序取数**。帧格式/命令码/状态位全部取自 psoc_link_abi.h(两端共用),
 * 本类不复制任何协议数字。
 *
 * v1 的病根: 一条命令 = 发命令帧 + busy_wait(150us) + 再发一帧把响应打出来, 请求与应答靠**位置**
 * 配对, 而全仓有 8 处各自手写这套时序。一次错位就永久失步, 且错位有"多一份/少一份"两个相反方向,
 * 主机无法判定方向 —— 于是长出了逐页交替重试、整轮重传、页阶段独占总线等一堆补丁。
 *
 * v2 用事务表把配对从**约定**变成**协议保证**:
 *   · 每笔请求带唯一 tag, 主机维护 LNK_TXN_SLOTS 深的事务表, 响应按 tag 认领;
 *   · SOF/CRC 不合法、tag 已退役、tag=0(主动状态帧)三类各自处理, **任何情况都不改动其它事务**
 *     ⇒ 不存在"失步"这种状态, 也就不需要任何重同步手段;
 *   · 从机对 tag 去重(响应环兼作去重缓存) ⇒ 超时重发对所有命令都安全, 含 CALIBRATE/AUTO_TUNE
 *     这类非幂等命令。
 * 由此快照与算法分页都变成"乱序事务异步掺杂", 不再有"一个分块必须在同一次调用内连续完成"。
 *
 * ★为什么必须是"一次 CS、一条 DMA、K 个整帧"★
 * 逐帧一次 CS 的版本里, 每帧都要 CS 建立/保持 + 一次 DMA 启停 + CPU 组帧, 固定开销与线上时间同量级;
 * 更要命的是它把从机的响应产出节奏和主机的取帧节奏死锁在一起 —— 从机 ISR 一旦被 CapSense 的全局
 * 临界区推迟, 主机就只能读到非法帧并靠重发补, 时间全烧在重试上(实测满容量上传随机某页预算耗尽)。
 * 从机侧改成"DMA 自己走的 TX 整帧环"后 TX FIFO 永不空, 主机每一帧都能读到**格式合法**的帧:
 * 非法帧不再是"从机停摆"的信号, 于是这里既不为单帧同步等待, 也不因非法帧退避或中断本批 ——
 * 只计数丢弃, 在途事务一律不动, 由 tag 保证正确性。
 * 命名规范: 类内部成员/函数以 _ 开头, 对外接口不加 _。
 */
class PsocLink {
public:
    enum class TxnState : uint8_t { FREE, QUEUED, INFLIGHT, DONE, FAILED };

    // ★对外语义与 v1 逐值相同(上位机 ALGO_GET_INFO 的 abort_reason 直接透传这几个值)★
    // 只有 3 号的含义从"页回显不符"收窄为"页事务未被受理", 因为回显错位在 v2 里不可能发生。
    enum class AlgoAbort : uint8_t {
        NONE = 0,
        BEGIN_BUSY = 1,      // 上一轮 phase 没归零就又来一次(状态泄漏)
        BEGIN_REJECTED = 2,  // ALGO_BEGIN 未被受理(长度回显不符或事务失败)
        PAGE_REJECTED = 3,   // 某页事务反复失败(重投次数用尽)
        END_FAILED = 4,      // ALGO_END 未受理或 len 不匹配
        COMMIT_TIMEOUT = 5,  // 等 PSoC commit 超时
        CONTENT_CRC = 6,     // PSoC 槽内内容 CRC 与上传件不符
        LINK = 7,            // 传输层失败
    };

    // ★一次下发的硬上限★ 4096B / 10B = 410 页, 每页一帧、可并发在途, 满容量约百毫秒级。
    // 上限必须有界且短: 这段时间里上传通道被门禁锁住, 用户想传一份修好的算法进来只会吃 DEVICE_BUSY。
    static constexpr uint32_t ALGO_UPLOAD_TIMEOUT_MS = 30000u;

    PsocLink(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin);

    bool init();
    bool ready() const { return _ready; }

    // ---- 事务表 ----
    // 受理一笔请求, 返回 tag(0 = 事务表满)。只入队, 真正上线由 pump() 完成。
    uint8_t submit(uint8_t cmd, const uint8_t args[LNK_ARG_BYTES]);
    TxnState txn_state(uint8_t tag) const;
    // DONE 时取走载荷并释放 tag; 其它状态返回 false 且不改变任何状态。
    bool take(uint8_t tag, uint8_t payload[LNK_PAYLOAD_BYTES]);
    void abandon(uint8_t tag);
    // 推进链路: 一次 CS 内连发 K = min(max_frames, LNK_BURST_MAX) 个整帧, 返回本批收到的**合法**帧数。
    // 无待发事务、无在途事务、未 arm_probe、看门狗也没到期 ⇒ 一帧都不发(返回 0), 这是空闲常态。
    uint32_t pump(uint32_t max_frames);
    // ★事件驱动取数★ 调用方观测到"对端有新数据了"(INT1 翻转 = PSoC 已发布新一代)时调一次, 下一次
    // pump 就发一帧把它取回来。这取代"按固定节拍不停轮询": 正常运行中链路上的每一帧都对应一次
    // 真实事件或一条真实命令, 没有一帧是白发的, 探活帧数恒为 0。
    void arm_probe() { _probe_pending = true; }
    // I3 带外 RX 空余电平：低时一个字节都不许时钟出去。
    bool tx_window_blocked() const;
    // 阻塞便利式(core1 内用): submit + pump 直到 DONE/超时。
    bool request(uint8_t cmd, const uint8_t args[LNK_ARG_BYTES], uint8_t payload[LNK_PAYLOAD_BYTES],
                 uint32_t timeout_us = 20000u);

    // 链路握手: 回显固件戳与 ABI 版本。帧长/ABI 不一致时返回 false —— 带着错帧长跑比不建链更坏。
    bool ping(uint32_t* out_fw = nullptr, uint8_t* out_abi = nullptr);
    // 兼容旧引导流程: 烧录前点亮白色指示灯。
    bool indicator_on();

    // ---- 主动状态帧发布态 ----
    uint64_t touch_mask() const { return _touch_mask; }
    uint16_t generation() const { return _generation; }
    uint8_t status() const { return _status; }
    uint32_t last_status_us() const { return _last_status_us; }
    // 掩码新鲜度: 只有主动状态帧 / STATUS 响应才带掩码, 故它与"链路是否活着"是两件事。
    bool status_fresh(uint32_t within_us) const;
    // ★链路存活判据★ 任意一帧通过 SOF+CRC 的响应都证明从机在应答 —— 每份响应都带 st, 所以
    // "活着"不需要专门等一帧状态帧。二者必须分开: 快照/算法这类密集流会长时间占满响应位,
    // 把掩码新鲜度当存活判据会让链路在正常忙碌时被误判掉线(实测 smoke 的 link_valid 抖成 false)。
    bool frame_fresh(uint32_t within_us) const;
    // 从机主循环正在执行重操作。★每份响应都带 st ⇒ 不需要为"忙不忙"单独发命令★
    bool operation_busy() const { return (_status & LNK_ST_OP_BUSY) != 0u; }

    // ---- 全通道快照(SNAP_LATCH + 流水化 SNAP_CH) ----
    // 攒满一份返回 true 并写入 *out。允许跨多次调用, 中途不需要独占总线。
    bool snapshot_pump(uint8_t max_frames, psoc::SensorSnapshot* out);
    // 单通道快路(独占精调): 一次调用取一份完整样本, 仅回填该通道。
    bool snapshot_pump_channel(uint8_t ch, psoc::SensorSnapshot* out);
    bool snapshot_pump_busy() const { return _snap.phase != 0u; }

    // ---- JIT 算法下发(流水化 ALGO_PAGE, 6 字节/页, 绝对寻址) ----
    bool begin_upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16);
    bool poll_upload_algo(bool* out_complete, bool* out_ok, bool* out_valid, uint16_t* out_len);

    // ---- 诊断(对外宽度与语义保持, 内容换成链路级计数) ----
    void algo_abort_info(uint16_t* page, uint8_t* reason, uint16_t* count) const {
        if (page) *page = _algo_abort_page;
        if (reason) *reason = _algo_abort_reason;
        if (count) *count = _algo_abort_count;
    }
    uint16_t algo_resync_count() const { return _rx_reject_count; }   // rx 非法帧数
    uint16_t algo_restart_count() const { return _txn_retry_count; }  // 事务重发次数


private:
    // ---- 调度常量 ----
    // ★重发已退化为纯兜底, 不再是主要机制★ 从机 TX 是 DMA 自己走的整帧环 ⇒ 响应几乎不会丢,
    // 迟到的响应也只是晚一两批出现。门限取大(远大于一批 8×16B ≈ 512us 的线上时间)才不会在
    // 从机还没来得及产出响应时就白发一遍, 把带宽让给真正待发的新页。
    static constexpr uint32_t RETRY_US = 5000u;
    // 重发本身是零代价且安全的(从机按 tag 去重), 所以预算给足: 兜底路径宁可多试几次也不误判永久失败。
    static constexpr uint8_t  MAX_TRIES = 16u;
    // ★探测帧由"计数器超时"驱动, 不是固定节拍★
    // 任何一帧通过 SOF+CRC 的响应都重置这两个计时器 —— 于是负载越高, 探测帧越少(极限是一帧不发,
    // 全部带宽给真实事务); 只有真闲下来或真卡住时才补一帧。固定节拍(每 N 帧一次 / 每 500us 一次)
    // 做不到这点: 它在高负载时照样白占位, 在空闲时又可能不够勤。
    // LINK_WATCHDOG_US: 纯故障判据, **不是**数据节拍。正常运行时取数完全由事件驱动
    // (arm_probe(): PSoC 每发布一份新代数就翻转 INT1, core1 见到翻转才来取) ⇒ 这个看门狗在健康
    // 链路上永远不会到期, 探活帧数恒为 0。只有"PSoC 不再发布新代数且主机也没有事务"时它才发一帧
    // 去确认对端是死是活。故可以给得很长: 150ms 远长于扫描周期(约 6ms), 又远短于人能感知的掉线。
    static constexpr uint32_t LINK_WATCHDOG_US = 150000u;
    // MASK_MAX_STALE_US: 掩码只搭在主动状态帧/STATUS 响应上, 密集命令流会正常地把它挤后。这是
    // 允许被挤后的上限, 到期投一条内部 LNK_CMD_STATUS 事务把掩码拽回来。同样是"收到带掩码的帧
    // 即重置", 故空闲时它永远不会触发(探测帧已经在刷掩码), 只在真被长时间挤占时才花掉一帧。
    // 20ms 远短于人手最短一次点击, 又足以让 410 帧的算法上传几乎独占链路。
    static constexpr uint32_t MASK_MAX_STALE_US = 20000u;
    // 并发在途上限。都留出余量不把事务表占满 —— 表满会连轮询帧都发不出去。
    static constexpr uint8_t  SNAP_MAX_INFLIGHT = 8u;
    static constexpr uint8_t  ALGO_MAX_INFLIGHT = 8u;
    // 一份快照的完成期限: 超期丢弃进度重新锁存, 免得某条事务反复失败让快照永不发布。
    static constexpr uint32_t SNAP_COMPLETE_TIMEOUT_US = 2000000u;
    // ★重投预算必须是"逐页"的, 不能是整轮累计★ 累计计数把"链路偶尔要重投一次"和"某一页真的
    // 写不进去"混成同一件事: 682 页的上传只要平均每 10 页重投一次就会撞满一个几十次的总预算,
    // 于是随机某页被判永久失败(实测 page 94 / page 164 —— 与页号、长度都无关, 正是这个特征)。
    // 绝对寻址下同一页重投结果完全相同, 所以只有"同一页连续失败这么多次"才是真故障。
    static constexpr uint8_t ALGO_PAGE_MAX_TRIES = 8u;


    struct Txn {
        uint8_t state;      // TxnState
        uint8_t tag;        // 0 = 空闲
        uint8_t cmd;
        uint8_t tries;
        uint8_t internal;   // 1 = 链路自用(掩码刷新), 不对外暴露
        uint32_t sent_us;
        uint8_t args[LNK_ARG_BYTES];
        uint8_t payload[LNK_PAYLOAD_BYTES];

        void clear() {
            state = (uint8_t)TxnState::FREE;
            tag = 0u; cmd = 0u; tries = 0u; internal = 0u; sent_us = 0u;
            for (uint32_t i = 0; i < LNK_ARG_BYTES; ++i) args[i] = 0u;
            for (uint32_t i = 0; i < LNK_PAYLOAD_BYTES; ++i) payload[i] = 0u;
        }
    };

    struct SnapState {
        uint8_t phase;      // 0=空闲 1=等锁存 2=收通道
        uint8_t latch_tag;
        uint8_t valid;
        uint16_t generation;
        uint32_t start_us;
        uint64_t pending;                            // 尚未收齐的通道位图
        uint8_t tag[LNK_CHANNEL_COUNT];              // 每通道在途事务 tag(0=未发)
        psoc::SensorSample sample[LNK_CHANNEL_COUNT];

        void clear() {
            phase = 0u; latch_tag = 0u; valid = 0u; generation = 0u; start_us = 0u; pending = 0u;
            for (uint32_t i = 0; i < LNK_CHANNEL_COUNT; ++i) { tag[i] = 0u; sample[i].clear(); }
        }
    };

    struct AlgoState {
        const uint8_t* data;
        uint16_t len;
        uint16_t crc16;
        uint16_t pages;
        uint16_t next_page;
        uint16_t retries;   // 整轮页重投总次数, ★只作诊断★ 不再当作失败判据(判据是逐页 page_try)
        uint8_t phase;      // 0=空闲 1=等 BEGIN 2=发页 3=等 END 4=等 commit
        uint8_t confirmed;
        uint8_t ctl_tag;    // 各阶段唯一在途的控制事务(BEGIN/END/INFO)
        uint32_t started_ms;
        uint32_t last_info_ms;
        uint8_t page_tag[ALGO_MAX_INFLIGHT];
        uint16_t page_no[ALGO_MAX_INFLIGHT];
        uint8_t page_try[ALGO_MAX_INFLIGHT];   // 本槽当前这一页已尝试次数

        void clear() {
            data = nullptr; len = 0u; crc16 = 0u; pages = 0u; next_page = 0u; retries = 0u;
            phase = 0u; confirmed = 0u; ctl_tag = 0u; started_ms = 0u; last_info_ms = 0u;
            // 0xFFFF = 该页槽空闲(0 是合法页号, 不能拿它当哨兵)
            for (uint32_t i = 0; i < ALGO_MAX_INFLIGHT; ++i) {
                page_tag[i] = 0u; page_no[i] = 0xFFFFu; page_try[i] = 0u;
            }
        }
    };

    // DMA 超时后恢复本机 PIO；它不参与帧边界或字节流相位处理。
    void _recover();
    bool _transfer(const uint8_t* tx, uint8_t* rx, size_t len);


    int _slot_of(uint8_t tag) const;
    int _free_slot() const;
    uint8_t _alloc_tag();
    uint8_t _submit_slot(uint8_t cmd, const uint8_t args[LNK_ARG_BYTES], bool internal);
    int _next_queued() const;
    uint32_t _inflight_count() const;
    uint32_t _free_count() const;
    void _retire_timeouts();
    // 收帧: 校验 → 发布 st → 按 tag 认领。返回 true 表示这是一帧合法帧(无论归谁)。
    bool _accept(const uint8_t* rx);
    void _publish_status_frame(const uint8_t* body);
    void _refresh_mask_if_due();

    void _snap_reset();
    void _snap_publish(psoc::SensorSnapshot* out) const;
    void _algo_note_abort(AlgoAbort reason, uint16_t page);
    void _algo_fail(AlgoAbort reason, uint16_t page, bool* out_complete);
    bool _algo_submit_page(uint32_t idx);
    // 收割已完成的页事务并补投新页。返回 false = 某页重投次数用尽(*out_fail_page 给出页号)。
    bool _algo_pump_pages(uint16_t* out_fail_page);

    uint8_t _sck_pin;
    uint8_t _mosi_pin;
    uint8_t _miso_pin;
    uint8_t _cs_pin;

    HAL_PIO* _pio;
    HAL_DMA_Duplex _dma;   // 收发两条 DMA 通道(mem ↔ 本 SM 的 TX/RX FIFO)
    uint8_t _sm;
    uint8_t _offset;
    bool _ready;

    Txn _txn[LNK_TXN_SLOTS];
    uint8_t _next_tag = LNK_TAG_MIN;
    uint8_t _status_txn_tag = 0u;      // 在途的内部掩码刷新事务(0=无)
    bool _probe_pending = false;       // 已被 arm_probe() 预约一帧取数(事件驱动)
    // I2: CS 事务只是同一响应字节流的分段，尾部不足一帧的字节必须跨事务保留。
    uint8_t _carry[LNK_FRAME_SIZE - 1u] = {};
    uint8_t _carry_len = 0u;

    uint64_t _touch_mask = 0u;
    uint16_t _generation = 0u;
    uint8_t _status = 0u;
    uint32_t _last_status_us = 0u;     // 最近一帧**主动状态帧/STATUS 响应**(掩码到手)的时刻
    uint32_t _last_frame_us = 0u;      // 最近一帧合法帧的时刻(链路存活)

    uint16_t _rx_reject_count = 0u;    // SOF/CRC 不合法的收帧数(饱和)
    uint16_t _txn_retry_count = 0u;    // 事务重发次数(饱和)
    uint16_t _stale_tag_count = 0u;    // 已退役 tag 的迟到响应数(饱和)


    SnapState _snap;
    AlgoState _algo;

    // ★上传中止现场★ 一次失败若什么痕迹都不留, 主机只能看到"PSoC 槽内 len=0", 而 BEGIN 未受理 /
    // 某页反复失败 / commit 超时 / 内容 CRC 不符 的修法完全不同。
    uint16_t _algo_abort_page = 0u;
    uint8_t  _algo_abort_reason = 0u;
    uint16_t _algo_abort_count = 0u;
};
