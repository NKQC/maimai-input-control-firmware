#include "psoc_link.h"
#include "../../config.h"
#include "../../hal/pio/hal_pio.h"
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <Arduino.h>
#include <cstring>

// ============================================================
// PIO SPI 主机程序（MODE0 / 8bit / MSB first / 全双工）—— ★延后采样 + DMA 直连版★
// 无 pioasm 集成，故手工汇编内联机器码（编码方式对照本工程已验证的 SWD 程序）。
// side-set 1 bit = SCK(GPIO26)，out pin = MOSI(GPIO28)，in pin = MISO(GPIO27)。
// ★关键1(电气)★：整条 SPI 走同一双向自适应(TXB)电平位移器，往返传播延迟大。若在 SCK 上升沿
// 当拍采样 MISO(零建立时间)，4MHz 起位错位。故采样点延后到 SCK 高相位第 4 拍，给 MISO 经位移器
// 往返留固定延迟补偿 —— 这是已验证能穿过位移器的唯一时序，绝不可改回上升沿当拍采样。
// ★关键2(不堵塞)★：autopull/autopush(阈值 8 bit) 使 DMA 能以 8 位传输直接对接内存字节流：
// CPU 不再逐字节 put/get，收发退化为纯内存操作(见 _transfer())。FIFO 空/满时 SM 自然停在
// out(side 0，SCK 保持低)，无需软件干预。
// ------------------------------------------------------------
//  0: out pins,1 [2]    side 0   (wrap target) 驱动 MOSI(OSR 高位在前=MSB)；SCK 低 3 拍(含下降沿)
//  1: nop        [3]    side 1   SCK 升高，等 4 拍让 MISO 经位移器往返稳定
//  2: in  pins,1        side 1   SCK 仍高，延后采样 MISO(升高后第 4 拍)；autopush 满 8 bit 自动推出
//     → wrap 回 0（SCK 回低，下一 bit）
// 每 bit 8 拍：SCK 高 5 拍、低 3 拍，采样点=上升后第 4 拍。★程序与采样点一个字都没动★，
// 变的只有单次 DMA 的传输长度(v1 的 7B/帧 → v2 的 K×16B 整批)——这是唯一验证过能穿过位移器的时序。
// ============================================================
static const uint16_t psoc_link_program_instructions[] = {
    0x6201, // 0: out pins,1   side 0 [2]   驱动 MOSI，SCK 低 3 拍（autopull 自动取字节）
    0xB342, // 1: nop          side 1 [3]   SCK 高，等 4 拍让 MISO 稳定
    0x5001, // 2: in  pins,1   side 1       延后采样 MISO（升高后第 4 拍，autopush 自动推出）
};

static const struct pio_program psoc_link_program = {
    .instructions = psoc_link_program_instructions,
    .length = 3,
    .origin = -1,
};

namespace {
constexpr uint8_t PROG_WRAP_TARGET = 0;
constexpr uint8_t PROG_WRAP = 2;
constexpr float CYCLES_PER_BIT = 8.0f;  // out3(低相位) + nop4 + in1 = 8；采样点=升高后第 4 拍
constexpr uint8_t SHIFT_BITS = 8;       // autopull/autopush 阈值：一次一字节，配 8 位 DMA
// CS guard time for the TXB level shifter.
constexpr uint32_t CS_SETUP_US = 2;
constexpr uint32_t CS_HOLD_US = 2;
// DMA timeout backstop; one burst is LNK_BURST_MAX x LNK_FRAME_SIZE bytes (8x16B @2MHz ~= 512us).
constexpr uint32_t XFER_TIMEOUT_US = 4000;
constexpr uint16_t PAGE_NONE = 0xFFFFu;   // 页槽空闲哨兵(0 是合法页号)
}  // namespace

PsocLink::PsocLink(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin)
    : _sck_pin(sck_pin), _mosi_pin(mosi_pin), _miso_pin(miso_pin), _cs_pin(cs_pin),
      _pio(nullptr), _sm(0), _offset(0), _ready(false) {
    for (uint32_t i = 0; i < LNK_TXN_SLOTS; ++i) _txn[i].clear();
    _snap.clear();
    _algo.clear();
}

bool PsocLink::init() {
    _pio = HAL_PIO1::getInstance();
    if (!_pio->init(_sck_pin)) {
        return false;
    }
    // 三根信号线都交给 PIO1（init 可能因已初始化而短路）
    _pio->init_pin(_sck_pin);
    _pio->init_pin(_mosi_pin);
    _pio->init_pin(_miso_pin);

    // CS 普通 GPIO 输出，空闲高（未选中）
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, true);
    gpio_put(_cs_pin, 1);

    // INT2 经自适应电平位移器中继；内部上下拉会破坏它的空余电平，保持高阻输入。
    gpio_init(PIN_SENSOR_INT2);
    gpio_set_dir(PIN_SENSOR_INT2, GPIO_IN);
    gpio_disable_pulls(PIN_SENSOR_INT2);

    if (!_pio->load_program(&psoc_link_program, &_offset)) {
        return false;
    }
    if (!_pio->claim_sm(&_sm)) {
        _pio->unload_program(&psoc_link_program, _offset);
        return false;
    }

    // 方向：SCK / MOSI 输出；MISO 保持输入（默认，勿设为输出）
    _pio->sm_set_pindirs_out(_sm, _sck_pin, 1);
    _pio->sm_set_pindirs_out(_sm, _mosi_pin, 1);

    PIOStateMachineConfig cfg;
    cfg.out_base = _mosi_pin; cfg.out_count = 1;
    cfg.in_base = _miso_pin;
    cfg.sideset_base = _sck_pin;
    cfg.sideset_bit_count = 1;
    cfg.sideset_optional = false;
    cfg.sideset_pindirs = false;
    // ★MSB first + 8 bit 自动移位★：DMA 以 8 位窄写把字节送进 TX FIFO，RP2040 窄写会把该字节
    // 复制到全部 4 个字节道，故左移的 OSR 取高位仍得到该字节；收侧左移满 8 bit 后字节落 ISR[7:0]，
    // DMA 从 RX FIFO 的第 0 字节道读出即可。阈值 8 = 一次一字节，与 DMA 的字节流严格 1:1。
    cfg.out_shift_right = false; cfg.autopull = true; cfg.pull_threshold = SHIFT_BITS;
    cfg.in_shift_right = false;  cfg.autopush = true; cfg.push_threshold = SHIFT_BITS;
    cfg.wrap_target = _offset + PROG_WRAP_TARGET;
    cfg.wrap = _offset + PROG_WRAP;
    cfg.program_offset = _offset;
    cfg.clkdiv = (float)clock_get_hz(clk_sys) / ((float)PSOC_SPI_SCK_HZ * CYCLES_PER_BIT);
    cfg.enabled = true;   // 首指令为 out(autopull)，TX FIFO 空则停在此处，SCK 保持低位

    if (!_pio->sm_configure(_sm, cfg)) {
        return false;
    }

    // 收发各一条 DMA 通道，由本 SM 的 TX/RX DREQ 节流；此后 CPU 不再触碰任何单个字节。
    DmaDuplexPorts ports;
    ports.tx_fifo = _pio->sm_fifo_byte_addr(_sm, true);
    ports.rx_fifo = _pio->sm_fifo_byte_addr(_sm, false);
    ports.tx_dreq = _pio->sm_dreq(_sm, true);
    ports.rx_dreq = _pio->sm_dreq(_sm, false);
    if (!_dma.init(ports)) {
        return false;
    }

    _ready = true;
    return true;
}

// 事务异常(DMA 完成标志未在上限内置起)后恢复主机 PIO，使下一笔硬件传输仍可启动。
// 它只处理本机硬件故障；帧边界始终由跨事务的字节流解析器判定，绝不靠这里重对齐。
void PsocLink::_recover() {
    _pio->sm_set_enabled(_sm, false);
    _pio->sm_clear_fifos(_sm);
    _pio->sm_restart(_sm);                                   // 清 ISR/OSR/移位计数/延时
    _pio->sm_exec(_sm, (uint16_t)(0x0000u | _offset));       // jmp <program start>
    _pio->sm_set_enabled(_sm, true);
}

// ★唯一线级原语★ 一次 CS 圈住整批 K 个整帧(len = K*LNK_FRAME_SIZE), 中间不拆、不等待, 一条 DMA 走完。
// I1 要求正常事务不得清 FIFO 或重置 PIO 状态；CS 仅划分事务，响应帧边界由 I2 内容解析确定。
bool PsocLink::_transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (rx != nullptr) std::memset(rx, 0, len);
    if (!_ready || len == 0) return false;

    gpio_put(_cs_pin, 0);
    busy_wait_us_32(CS_SETUP_US);
    _dma.start(tx, rx, len);
    const bool done = _dma.wait(XFER_TIMEOUT_US);
    busy_wait_us_32(CS_HOLD_US);
    gpio_put(_cs_pin, 1);
    if (!done) _recover();
    return done;
}

// ---------------------------------------------------------------- 事务表 ----
int PsocLink::_slot_of(uint8_t tag) const {
    if (tag == LNK_TAG_NONE) return -1;
    for (uint32_t i = 0; i < LNK_TXN_SLOTS; ++i) {
        if (_txn[i].tag == tag && _txn[i].state != (uint8_t)TxnState::FREE) return (int)i;
    }
    return -1;
}

int PsocLink::_free_slot() const {
    for (uint32_t i = 0; i < LNK_TXN_SLOTS; ++i) {
        if (_txn[i].state == (uint8_t)TxnState::FREE) return (int)i;
    }
    return -1;
}

uint32_t PsocLink::_free_count() const {
    uint32_t n = 0u;
    for (uint32_t i = 0; i < LNK_TXN_SLOTS; ++i) {
        if (_txn[i].state == (uint8_t)TxnState::FREE) n++;
    }
    return n;
}

uint32_t PsocLink::_inflight_count() const {
    uint32_t n = 0u;
    for (uint32_t i = 0; i < LNK_TXN_SLOTS; ++i) {
        if (_txn[i].state == (uint8_t)TxnState::INFLIGHT) n++;
    }
    return n;
}

int PsocLink::_next_queued() const {
    for (uint32_t i = 0; i < LNK_TXN_SLOTS; ++i) {
        if (_txn[i].state == (uint8_t)TxnState::QUEUED) return (int)i;
    }
    return -1;
}

// 单调循环分配 1..LNK_TAG_MAX, 跳过仍被占用的值。循环长度(127)远大于从机去重缓存深度(8),
// 故一个 tag 被复用时它的旧响应必然早已被挤出缓存 —— 不会被误判成重复请求。
uint8_t PsocLink::_alloc_tag() {
    for (uint32_t n = 0; n < LNK_TAG_MAX; ++n) {
        const uint8_t tag = _next_tag;
        _next_tag = (uint8_t)((_next_tag >= LNK_TAG_MAX) ? LNK_TAG_MIN : (_next_tag + 1u));
        if (_slot_of(tag) < 0) return tag;
    }
    return LNK_TAG_NONE;
}

uint8_t PsocLink::_submit_slot(uint8_t cmd, const uint8_t args[LNK_ARG_BYTES], bool internal) {
    const int slot = _free_slot();
    if (slot < 0) return LNK_TAG_NONE;
    const uint8_t tag = _alloc_tag();
    if (tag == LNK_TAG_NONE) return LNK_TAG_NONE;
    Txn& t = _txn[slot];
    t.clear();
    t.state = (uint8_t)TxnState::QUEUED;
    t.tag = tag;
    t.cmd = cmd;
    t.internal = internal ? 1u : 0u;
    if (args != nullptr) {
        for (uint32_t i = 0; i < LNK_ARG_BYTES; ++i) t.args[i] = args[i];
    }
    return tag;
}

uint8_t PsocLink::submit(uint8_t cmd, const uint8_t args[LNK_ARG_BYTES]) {
    if (!_ready) return LNK_TAG_NONE;
    // 永远给链路自己留一格: 表被业务占满时连掩码刷新事务都投不进去, 触控会假性冻结。
    if (_free_count() <= 1u) return LNK_TAG_NONE;
    return _submit_slot(cmd, args, false);
}

PsocLink::TxnState PsocLink::txn_state(uint8_t tag) const {
    const int slot = _slot_of(tag);
    if (slot < 0) return TxnState::FREE;
    return (TxnState)_txn[slot].state;
}

bool PsocLink::take(uint8_t tag, uint8_t payload[LNK_PAYLOAD_BYTES]) {
    const int slot = _slot_of(tag);
    if (slot < 0) return false;
    Txn& t = _txn[slot];
    if (t.state != (uint8_t)TxnState::DONE) return false;
    if (payload != nullptr) {
        for (uint32_t i = 0; i < LNK_PAYLOAD_BYTES; ++i) payload[i] = t.payload[i];
    }
    t.clear();
    return true;
}

void PsocLink::abandon(uint8_t tag) {
    const int slot = _slot_of(tag);
    if (slot < 0) return;
    // ★放弃只影响本事务★ 从机若还会把它的响应吐上来, 会因 tag 已退役而被丢弃, 不污染任何人。
    _txn[slot].clear();
}

// 在 I3 下 RX 环溢出与帧丢失结构上不会发生；此分支只保留给硬件异常，正常运行应恒不触发。
// 因而 _txn_retry_count 是链路已经出现异常的诊断指示，而非吞吐调节手段。
void PsocLink::_retire_timeouts() {
    const uint32_t now = time_us_32();
    for (uint32_t i = 0; i < LNK_TXN_SLOTS; ++i) {
        Txn& t = _txn[i];
        if (t.state != (uint8_t)TxnState::INFLIGHT) continue;
        if ((uint32_t)(now - t.sent_us) < RETRY_US) continue;
        // ★停摆期间不扣重试预算★ 本条发出之后一帧合法帧都没见过 ⇒ 从机整段没在应答(CapSense
        // 临界区压住了它的 DMA 完成中断), 这不是"我的请求丢了", 而是"谁的响应都出不来"。
        // 此时扣预算等于用别人的故障判自己死刑 —— 满容量上传就是这么被误判失败的。
        // 只重排队、刷新计时, 预算留给"从机明明在应答却偏偏不答我"这种真正可疑的情形。
        const bool slave_answered_since = (int32_t)(_last_frame_us - t.sent_us) > 0;
        t.sent_us = now;
        if (!slave_answered_since) {
            t.state = (uint8_t)TxnState::QUEUED;
            continue;
        }
        t.tries++;
        if (_txn_retry_count < 0xFFFFu) _txn_retry_count++;
        if (t.tries > MAX_TRIES) {
            t.state = (uint8_t)TxnState::FAILED;
            // 内部掩码刷新事务无人认领, 就地回收, 否则会永久占着刷新槽。
            if (t.internal != 0u) {
                if (t.tag == _status_txn_tag) _status_txn_tag = LNK_TAG_NONE;
                t.clear();
            }
        } else {
            t.state = (uint8_t)TxnState::QUEUED;
        }
    }
}

void PsocLink::_publish_status_frame(const uint8_t* body) {
    uint64_t mask = 0u;
    for (uint32_t i = 0; i < 5u; ++i) mask |= (uint64_t)body[i] << (8u * i);
    _touch_mask = mask & (((uint64_t)1u << LNK_CHANNEL_COUNT) - 1u);
    _generation = lnk_rd16(&body[5]);
    _last_status_us = time_us_32();
}

bool PsocLink::frame_fresh(uint32_t within_us) const {
    if (_last_frame_us == 0u) return false;
    return (uint32_t)(time_us_32() - _last_frame_us) <= within_us;
}

bool PsocLink::status_fresh(uint32_t within_us) const {
    if (_last_status_us == 0u) return false;
    return (uint32_t)(time_us_32() - _last_status_us) <= within_us;
}

bool PsocLink::tx_window_blocked() const {
    return !gpio_get(PIN_SENSOR_INT2);
}

// ★掩码刷新必须自己拿事务保证★ 掩码只出现在 tag=0 的主动状态帧里, 而从机一旦有排队响应就先
// 吐响应 —— 算法上传/快照这类密集流期间可能长时间见不到状态帧。到期就投一条内部 STATUS 事务,
// 它的响应带 tag、内容与状态帧完全相同, 于是触控在任何流量形态下都持续更新。
void PsocLink::_refresh_mask_if_due() {
    if (_status_txn_tag != LNK_TAG_NONE) return;
    if ((uint32_t)(time_us_32() - _last_status_us) < MASK_MAX_STALE_US) return;
    if (_free_slot() < 0) return;
    _status_txn_tag = _submit_slot(LNK_CMD_STATUS, nullptr, true);
}

// 收帧: 校验 → 无条件发布 st → 按 tag 认领。三类异常(非法帧 / 已退役 tag / 主动状态帧)各自处理,
// **任何情况都不改动其它事务的状态** ⇒ 不存在"失步"这种状态。
bool PsocLink::_accept(const uint8_t* rx) {
    if (!lnk_frame_ok(rx, LNK_SOF_RSP)) {
        if (_rx_reject_count < 0xFFFFu) _rx_reject_count++;
        return false;
    }
    _status = rx[LNK_OFF_ST];
    _last_frame_us = time_us_32();

    const uint8_t tag = rx[LNK_OFF_TAG];
    if (tag == LNK_TAG_NONE) {
        _publish_status_frame(&rx[LNK_OFF_BODY]);
        return true;
    }
    const int slot = _slot_of(tag);
    if (slot < 0) {
        // 迟到/重复响应: tag 早已退役。丢弃即可, 陈旧数据无法冒充任何一轮的结果。
        if (_stale_tag_count < 0xFFFFu) _stale_tag_count++;
        return true;
    }
    Txn& t = _txn[slot];
    if (t.state == (uint8_t)TxnState::INFLIGHT || t.state == (uint8_t)TxnState::QUEUED) {
        for (uint32_t i = 0; i < LNK_PAYLOAD_BYTES; ++i) t.payload[i] = rx[LNK_OFF_BODY + i];
        t.state = (uint8_t)TxnState::DONE;
        if (t.internal != 0u) {
            _publish_status_frame(t.payload);
            if (t.tag == _status_txn_tag) _status_txn_tag = LNK_TAG_NONE;
            t.clear();
        }
    }
    return true;
}

// ★一次 CS、一条 DMA、K 个整帧, 事后按 tag 异步对账★
// I2 不把 CS 当作响应帧边界；I3 在本批开始时按带外 INT2 电平一次性准入整批。
uint32_t PsocLink::pump(uint32_t max_frames) {
    if (!_ready) return 0u;
    // I3 reserves LNK_BURST_MAX slots above the threshold, so sample INT2 once: a high permits this
    // complete burst even if the PSoC lowers the level while it drains. Rechecking per frame would only
    // split a safe burst and waste CS setup/hold overhead.
    if (tx_window_blocked()) return 0u;
    if (max_frames == 0u) max_frames = 1u;
    if (max_frames > LNK_BURST_MAX) max_frames = LNK_BURST_MAX;

    _retire_timeouts();
    _refresh_mask_if_due();

    uint8_t txbuf[LNK_BURST_MAX * LNK_FRAME_SIZE];
    uint8_t rxbuf[LNK_BURST_MAX * LNK_FRAME_SIZE];
    int8_t batch_slot[LNK_BURST_MAX];
    const uint32_t now = time_us_32();
    uint32_t frames = 0u;

    for (; frames < max_frames; ++frames) {
        const int slot = _next_queued();
        if (slot < 0) {
            const bool drain = _inflight_count() != 0u;
            const bool watchdog = (uint32_t)(now - _last_frame_us) >= LINK_WATCHDOG_US;
            if (!drain && !_probe_pending && !watchdog) break;
            _probe_pending = false;
        }

        uint8_t* tx = &txbuf[frames * LNK_FRAME_SIZE];
        std::memset(tx, 0, LNK_FRAME_SIZE);
        tx[LNK_OFF_SOF] = LNK_SOF_REQ;
        if (slot >= 0) {
            Txn& t = _txn[slot];
            tx[LNK_OFF_TAG] = t.tag;
            tx[LNK_OFF_CMD] = t.cmd;
            for (uint32_t i = 0; i < LNK_ARG_BYTES; ++i) tx[LNK_OFF_BODY + i] = t.args[i];
            t.state = (uint8_t)TxnState::INFLIGHT;
            t.sent_us = now;
        } else {
            tx[LNK_OFF_TAG] = LNK_TAG_NONE;
            tx[LNK_OFF_CMD] = LNK_CMD_STATUS;
        }
        lnk_seal(tx);
        batch_slot[frames] = (int8_t)slot;

        if (slot < 0 && _inflight_count() == 0u) {
            frames++;
            break;
        }
    }

    if (frames == 0u) return 0u;

    if (!_transfer(txbuf, rxbuf, (size_t)frames * LNK_FRAME_SIZE)) {
        // 硬件传输失败后仅重排事务；本批未完成的事务会在后续带外电平允许的批次中安全重投。
        for (uint32_t i = 0; i < frames; ++i) {
            const int slot = batch_slot[i];
            if (slot >= 0 && _txn[slot].state == (uint8_t)TxnState::INFLIGHT) {
                _txn[slot].state = (uint8_t)TxnState::QUEUED;
            }
        }
        return 0u;
    }

    uint8_t stream[(LNK_FRAME_SIZE - 1u) + LNK_BURST_MAX * LNK_FRAME_SIZE];
    const size_t rx_len = (size_t)frames * LNK_FRAME_SIZE;
    std::memcpy(stream, _carry, _carry_len);
    std::memcpy(&stream[_carry_len], rxbuf, rx_len);
    const size_t stream_len = (size_t)_carry_len + rx_len;
    size_t offset = 0u;
    uint32_t received = 0u;
    while ((stream_len - offset) >= LNK_FRAME_SIZE) {
        const uint8_t* frame = &stream[offset];
        if (lnk_frame_ok(frame, LNK_SOF_RSP)) {
            if (_accept(frame)) received++;
            offset += LNK_FRAME_SIZE;
        } else {
            offset++;
        }
    }
    _carry_len = (uint8_t)(stream_len - offset);
    if (_carry_len != 0u) std::memcpy(_carry, &stream[offset], _carry_len);
    return received;
}

bool PsocLink::request(uint8_t cmd, const uint8_t args[LNK_ARG_BYTES],
                       uint8_t payload[LNK_PAYLOAD_BYTES], uint32_t timeout_us) {
    if (!_ready) return false;
    const uint8_t tag = submit(cmd, args);
    if (tag == LNK_TAG_NONE) return false;
    const uint32_t start = time_us_32();
    for (;;) {
        pump(4u);
        const TxnState st = txn_state(tag);
        if (st == TxnState::DONE) return take(tag, payload);
        if (st == TxnState::FAILED || st == TxnState::FREE) {
            abandon(tag);
            return false;
        }
        if ((uint32_t)(time_us_32() - start) > timeout_us) {
            abandon(tag);
            return false;
        }
    }
}

bool PsocLink::ping(uint32_t* out_fw, uint8_t* out_abi) {
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};
    if (!request(LNK_CMD_PING, nullptr, p)) return false;
    if (out_fw) *out_fw = lnk_rd32(p);
    if (out_abi) *out_abi = p[4];
    // 帧长/ABI 不一致就不建链 —— 带着错帧长跑比明确失败坏得多。
    return p[4] == LNK_ABI_VERSION && p[5] == LNK_FRAME_SIZE;
}

bool PsocLink::indicator_on() {
    return request(LNK_CMD_INDICATOR_ON, nullptr, nullptr);
}

// ---------------------------------------------------------------- 快照 ----
void PsocLink::_snap_reset() {
    for (uint32_t ch = 0; ch < LNK_CHANNEL_COUNT; ++ch) {
        if (_snap.tag[ch] != LNK_TAG_NONE) abandon(_snap.tag[ch]);
    }
    if (_snap.latch_tag != LNK_TAG_NONE) abandon(_snap.latch_tag);
    _snap.clear();
}

void PsocLink::_snap_publish(psoc::SensorSnapshot* out) const {
    if (out == nullptr) return;
    out->clear();
    out->generation = _snap.generation;
    out->valid = _snap.valid != 0u;
    for (uint32_t ch = 0; ch < LNK_CHANNEL_COUNT; ++ch) out->channels[ch] = _snap.sample[ch];
}

// 分块全通道快照。★不再有"一个分块必须在同一次调用内连续完成"这条约束★: 通道事务各带 tag,
// 乱序收齐即可, 中途插入触控轮询/命令流都不会吞掉任何一份应答。
bool PsocLink::snapshot_pump(uint8_t max_frames, psoc::SensorSnapshot* out) {
    if (!_ready) return false;
    const uint32_t now = time_us_32();
    // 陈旧的半份快照不许永久霸占: 超期即丢弃进度, 下次从新锁存的代数重来。
    if (_snap.phase != 0u && (uint32_t)(now - _snap.start_us) > SNAP_COMPLETE_TIMEOUT_US) {
        _snap_reset();
    }
    if (_snap.phase == 0u) {
        const uint8_t tag = submit(LNK_CMD_SNAP_LATCH, nullptr);
        if (tag == LNK_TAG_NONE) { (void)pump(max_frames); return false; }
        _snap.clear();
        _snap.latch_tag = tag;
        _snap.phase = 1u;
        _snap.start_us = now;
        _snap.pending = ((uint64_t)1u << LNK_CHANNEL_COUNT) - 1u;
    }

    (void)pump(max_frames);
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};

    if (_snap.phase == 1u) {
        // SNAP_LATCH 是★延迟应答★: 从机完成 252B 锁存后才回。拿到它即保证随后任意顺序的
        // SNAP_CH 都属于同一代 —— v1 那套"BEGIN 与 INFO 必须相邻 + 主循环锁存窗"随之退役。
        const TxnState st = txn_state(_snap.latch_tag);
        if (st == TxnState::DONE) {
            (void)take(_snap.latch_tag, p);
            _snap.latch_tag = LNK_TAG_NONE;
            _snap.generation = lnk_rd16(p);
            _snap.valid = (p[2] != 0u) ? 1u : 0u;
            if (_snap.valid == 0u || p[3] != LNK_CHANNEL_COUNT) {
                // 从机尚未发布过代数: 如实上报无效代数, 不为它白搬 36 帧。
                if (out != nullptr) {
                    out->clear();
                    out->generation = _snap.generation;
                    out->valid = false;
                }
                _snap_reset();
                return true;
            }
            _snap.phase = 2u;
        } else if (st == TxnState::FAILED || st == TxnState::FREE) {
            _snap_reset();
            return false;
        } else {
            return false;
        }
    }

    // 收割已完成的通道事务
    uint8_t inflight = 0u;
    for (uint32_t ch = 0; ch < LNK_CHANNEL_COUNT; ++ch) {
        if (_snap.tag[ch] == LNK_TAG_NONE) continue;
        const TxnState st = txn_state(_snap.tag[ch]);
        if (st == TxnState::DONE) {
            (void)take(_snap.tag[ch], p);
            _snap.tag[ch] = LNK_TAG_NONE;
            if (p[0] != (uint8_t)ch) continue;   // 通道自述不符: 该通道留在 pending, 重投一次
            psoc::SensorSample& s = _snap.sample[ch];
            s.raw = lnk_rd16(&p[1]);
            s.baseline = lnk_rd16(&p[3]);
            s.diff = (int16_t)lnk_rd16(&p[5]);
            s.status = p[7];
            _snap.pending &= ~((uint64_t)1u << ch);
        } else if (st == TxnState::FAILED || st == TxnState::FREE) {
            abandon(_snap.tag[ch]);
            _snap.tag[ch] = LNK_TAG_NONE;
        } else {
            inflight++;
        }
    }
    // 补投: 只保持 SNAP_MAX_INFLIGHT 条在途, 给轮询帧与其它命令留出事务表余量。
    for (uint32_t ch = 0; ch < LNK_CHANNEL_COUNT && inflight < SNAP_MAX_INFLIGHT; ++ch) {
        if (((_snap.pending >> ch) & 1u) == 0u || _snap.tag[ch] != LNK_TAG_NONE) continue;
        uint8_t args[LNK_ARG_BYTES] = {0};
        args[0] = (uint8_t)ch;
        const uint8_t tag = submit(LNK_CMD_SNAP_CH, args);
        if (tag == LNK_TAG_NONE) break;
        _snap.tag[ch] = tag;
        inflight++;
    }
    if (_snap.pending != 0u) return false;

    _snap_publish(out);
    _snap_reset();
    return true;
}

// 单通道快路(独占精调): 一次调用取一份完整样本, 仅回填该通道, 其余通道保持调用方原值。
bool PsocLink::snapshot_pump_channel(uint8_t ch, psoc::SensorSnapshot* out) {
    if (!_ready || out == nullptr || ch >= LNK_CHANNEL_COUNT) return false;
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};
    if (!request(LNK_CMD_SNAP_LATCH, nullptr, p)) return false;
    out->generation = lnk_rd16(p);
    out->valid = p[2] != 0u;
    if (!out->valid) return true;

    uint8_t args[LNK_ARG_BYTES] = {0};
    args[0] = ch;
    if (!request(LNK_CMD_SNAP_CH, args, p)) return false;
    if (p[0] != ch) return false;
    psoc::SensorSample& s = out->channels[ch];
    s.raw = lnk_rd16(&p[1]);
    s.baseline = lnk_rd16(&p[3]);
    s.diff = (int16_t)lnk_rd16(&p[5]);
    s.status = p[7];
    return true;
}

// ------------------------------------------------------------ 算法下发 ----
void PsocLink::_algo_note_abort(AlgoAbort reason, uint16_t page) {
    _algo_abort_reason = (uint8_t)reason;
    _algo_abort_page = page;
    if (_algo_abort_count < 0xFFFFu) _algo_abort_count++;
}

void PsocLink::_algo_fail(AlgoAbort reason, uint16_t page, bool* out_complete) {
    _algo_note_abort(reason, page);
    for (uint32_t i = 0; i < ALGO_MAX_INFLIGHT; ++i) {
        if (_algo.page_tag[i] != LNK_TAG_NONE) abandon(_algo.page_tag[i]);
    }
    if (_algo.ctl_tag != LNK_TAG_NONE) abandon(_algo.ctl_tag);
    _algo.clear();
    if (out_complete) *out_complete = true;
}

bool PsocLink::_algo_submit_page(uint32_t idx) {
    const uint16_t page = _algo.page_no[idx];
    uint8_t args[LNK_ARG_BYTES] = {0};
    lnk_wr16(args, page);
    const uint32_t base = (uint32_t)page * LNK_ALGO_PAGE_BYTES;
    for (uint32_t b = 0; b < LNK_ALGO_PAGE_BYTES; ++b) {
        const uint32_t off = base + b;
        // 不足一页的末页补 0；从机的 CRC16 只覆盖 [0,len)，补齐内容不参与校验。
        args[2u + b] = (off < _algo.len) ? _algo.data[off] : 0u;
    }
    const uint8_t tag = submit(LNK_CMD_ALGO_PAGE, args);
    if (tag == LNK_TAG_NONE) return false;
    _algo.page_tag[idx] = tag;
    return true;
}

// 收割已完成的页事务并补投新页。★单页失败只补发该页★(绝对寻址 offset = page*LNK_ALGO_PAGE_BYTES),
// v1 那套"逐页交替重试 + 整轮从 BEGIN 重传"在 v2 里没有存在理由。
// ★重投预算是逐页的★ 判据 = 同一页连续失败 ALGO_PAGE_MAX_TRIES 次(绝对寻址下同一页重投结果完全
// 相同, 只有连续失败才说明是真故障)。整轮累计上限已删除: 它把"链路偶尔重投一次"与"某页真的写不
// 进去"混成一件事, 几百页的上传平均每十几页重投一次就撞满总预算, 于是随机某页被判永久失败。
bool PsocLink::_algo_pump_pages(uint16_t* out_fail_page) {
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};
    for (uint32_t i = 0; i < ALGO_MAX_INFLIGHT; ++i) {
        if (_algo.page_tag[i] == LNK_TAG_NONE) continue;
        const TxnState st = txn_state(_algo.page_tag[i]);
        bool page_retry = false;
        if (st == TxnState::DONE) {
            (void)take(_algo.page_tag[i], p);
            _algo.page_tag[i] = LNK_TAG_NONE;
            // 页号回显不符 = 从机拒收(越界 / 无在途上传)。留在本槽等重投。
            if (lnk_rd16(p) == _algo.page_no[i]) _algo.page_no[i] = PAGE_NONE;
            else page_retry = true;
        } else if (st == TxnState::FAILED || st == TxnState::FREE) {
            abandon(_algo.page_tag[i]);
            _algo.page_tag[i] = LNK_TAG_NONE;
            page_retry = true;
        }
        if (!page_retry) continue;
        if (_algo.retries < 0xFFFFu) _algo.retries++;   // 仅诊断计数, 不是失败判据
        if (++_algo.page_try[i] >= ALGO_PAGE_MAX_TRIES) {
            if (out_fail_page) *out_fail_page = _algo.page_no[i];
            return false;
        }
    }
    // 补投: 先补重投页, 再取新页, 保持至多 ALGO_MAX_INFLIGHT 条在途。
    for (uint32_t i = 0; i < ALGO_MAX_INFLIGHT; ++i) {
        if (_algo.page_tag[i] != LNK_TAG_NONE) continue;
        if (_algo.page_no[i] == PAGE_NONE) {
            if (_algo.next_page >= _algo.pages) continue;
            _algo.page_no[i] = _algo.next_page++;
            _algo.page_try[i] = 0u;   // 新页领入本槽: 该页的重投预算从头计
        }
        if (!_algo_submit_page(i)) break;   // 事务表暂时满: 下一拍再投
    }
    return true;
}

bool PsocLink::begin_upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16) {
    if (!_ready || data == nullptr || len == 0u || len > LNK_ALGO_SLOT_SIZE) return false;
    if (_algo.phase != 0u) {
        // 上一轮的状态没归零就又来一次 ⇒ 状态泄漏。★必须留痕★: 否则此后每次上传都被静默拒掉,
        // 主机只看到"永远传不上去"。
        _algo_note_abort(AlgoAbort::BEGIN_BUSY, 0xFFFFu);
        return false;
    }
    uint8_t args[LNK_ARG_BYTES] = {0};
    lnk_wr16(args, len);
    const uint8_t tag = submit(LNK_CMD_ALGO_BEGIN, args);
    if (tag == LNK_TAG_NONE) {
        _algo_note_abort(AlgoAbort::BEGIN_REJECTED, 0xFFFFu);
        return false;
    }
    _algo.clear();
    _algo.data = data;
    _algo.len = len;
    _algo.crc16 = crc16;
    _algo.pages = (uint16_t)((len + LNK_ALGO_PAGE_BYTES - 1u) / LNK_ALGO_PAGE_BYTES);
    _algo.phase = 1u;
    _algo.ctl_tag = tag;
    _algo.started_ms = millis();
    _algo.last_info_ms = _algo.started_ms;
    return true;
}

bool PsocLink::poll_upload_algo(bool* out_complete, bool* out_ok, bool* out_valid, uint16_t* out_len) {
    if (out_complete) *out_complete = false;
    if (out_ok) *out_ok = false;
    if (out_valid) *out_valid = false;
    if (out_len) *out_len = 0u;
    if (_algo.phase == 0u) return true;

    // 页阶段一次推多帧(在途并发), commit 阶段只需推进那一条 INFO 事务。
    (void)pump(ALGO_MAX_INFLIGHT + 2u);
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - _algo.started_ms) >= ALGO_UPLOAD_TIMEOUT_MS) {
        const bool committing = _algo.phase >= 3u;
        _algo_fail(committing ? AlgoAbort::COMMIT_TIMEOUT : AlgoAbort::PAGE_REJECTED,
                   _algo.next_page, out_complete);
        return false;
    }
    uint8_t p[LNK_PAYLOAD_BYTES] = {0};

    if (_algo.phase == 1u) {
        const TxnState st = txn_state(_algo.ctl_tag);
        if (st == TxnState::DONE) {
            (void)take(_algo.ctl_tag, p);
            _algo.ctl_tag = LNK_TAG_NONE;
            // BEGIN 未被受理时它很可能**已经**清空了从机的暂存长度, 现象与 CRC 失败一模一样,
            // 故这条出口必须单独留痕(见 AlgoAbort 注释)。
            if (lnk_rd16(p) != _algo.len) {
                _algo_fail(AlgoAbort::BEGIN_REJECTED, 0xFFFFu, out_complete);
                return false;
            }
            _algo.phase = 2u;
        } else if (st == TxnState::FAILED || st == TxnState::FREE) {
            _algo_fail(AlgoAbort::BEGIN_REJECTED, 0xFFFFu, out_complete);
            return false;
        }
        return true;
    }

    if (_algo.phase == 2u) {
        uint16_t fail_page = 0u;
        if (!_algo_pump_pages(&fail_page)) {
            _algo_fail(AlgoAbort::PAGE_REJECTED, fail_page, out_complete);
            return false;
        }
        if (_algo.next_page < _algo.pages) return true;
        for (uint32_t i = 0; i < ALGO_MAX_INFLIGHT; ++i) {
            if (_algo.page_tag[i] != LNK_TAG_NONE || _algo.page_no[i] != PAGE_NONE) return true;
        }
        uint8_t args[LNK_ARG_BYTES] = {0};
        lnk_wr16(args, _algo.crc16);
        const uint8_t tag = submit(LNK_CMD_ALGO_END, args);
        if (tag == LNK_TAG_NONE) return true;   // 表暂时满, 下一拍再发
        _algo.ctl_tag = tag;
        _algo.phase = 3u;
        return true;
    }

    if (_algo.phase == 3u) {
        const TxnState st = txn_state(_algo.ctl_tag);
        if (st == TxnState::DONE) {
            (void)take(_algo.ctl_tag, p);
            _algo.ctl_tag = LNK_TAG_NONE;
            if (p[0] == 0u || lnk_rd16(&p[2]) != _algo.len) {
                _algo_fail(AlgoAbort::END_FAILED, _algo.pages, out_complete);
                return false;
            }
            _algo.phase = 4u;
            _algo.last_info_ms = now_ms - 100u;
        } else if (st == TxnState::FAILED || st == TxnState::FREE) {
            _algo_fail(AlgoAbort::END_FAILED, _algo.pages, out_complete);
            return false;
        }
        return true;
    }

    // phase 4: 等 commit。★终态判据 = 内容对账, 不是 valid+len★
    // "valid && len==expected" 分不出"新算法装上了"与"旧算法还在、长度恰好相同"(同一份源改一个
    // 常量再编译, 长度几乎必然不变) —— 那正是"上传成功却行为没变"的来源。uploading 还为真说明
    // 从机尚未 commit, 此刻的 valid/len 属于上一份, 一律不采信。
    if (_algo.ctl_tag != LNK_TAG_NONE) {
        const TxnState st = txn_state(_algo.ctl_tag);
        if (st == TxnState::DONE) {
            (void)take(_algo.ctl_tag, p);
            _algo.ctl_tag = LNK_TAG_NONE;
            const bool valid = p[0] != 0u;
            const bool uploading = p[1] != 0u;
            const uint16_t len = lnk_rd16(&p[2]);
            const uint16_t slot_crc = lnk_rd16(&p[4]);
            if (out_valid) *out_valid = valid;
            if (out_len) *out_len = len;
            if (valid && !uploading && len == _algo.len) {
                if (++_algo.confirmed >= 2u) {   // 连续 2 次确认(防单帧巧合)
                    const bool content_ok = slot_crc == _algo.crc16;
                    if (!content_ok) {
                        // 页全发完、END 也受理了, 但槽内内容 CRC 不符 ⇒ 中途有字节写歪。
                        _algo_note_abort(AlgoAbort::CONTENT_CRC, _algo.pages);
                    } else {
                        _algo_abort_reason = (uint8_t)AlgoAbort::NONE;   // 成功一次就清掉现场标记
                    }
                    _algo.clear();
                    if (out_complete) *out_complete = true;
                    if (out_ok) *out_ok = content_ok;
                    // 返回值是"链路应答是否正常", 与"内容是否对上"是两件事: 这里应答一切正常,
                    // 只是内容不符, 故仍返回 true 并靠 out_ok=false 让上层走"下发失败"分支。
                    return true;
                }
            } else {
                _algo.confirmed = 0u;
            }
        } else if (st == TxnState::FAILED || st == TxnState::FREE) {
            _algo.ctl_tag = LNK_TAG_NONE;   // 下轮重发 INFO
        }
        return true;
    }
    if ((uint32_t)(now_ms - _algo.last_info_ms) < 100u) return true;
    _algo.last_info_ms = now_ms;
    const uint8_t tag = submit(LNK_CMD_ALGO_INFO, nullptr);
    if (tag != LNK_TAG_NONE) _algo.ctl_tag = tag;
    return true;
}
