#include "psoc_spi.h"
#include "../../config.h"
#include "../../hal/pio/hal_pio.h"
// 只为取 PSOC_ALGO_MAX_LEN(= PSoC 的 ALGO_SLOT_SIZE)。该头只依赖 cstdint 且仅前置声明 Psoc,
// 不引入反向依赖; 槽大小是协议契约, 必须与 store 用同一个常量, 不许在此另写一份数字。
#include "../../service/psoc_algo/psoc_algo.h"
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
// ★关键2(不堵塞)★：改用 autopull/autopush(阈值 8 bit) 取代显式 pull/push + x 计数，使 DMA 能以
// 8 位传输直接对接内存字节流：CPU 不再逐字节 put/get，收发退化为纯内存操作(见 transfer())。
// FIFO 空/满时 SM 自然停在 out(side 0，SCK 保持低)，无需软件干预。
// ------------------------------------------------------------
//  0: out pins,1 [2]    side 0   (wrap target) 驱动 MOSI(OSR 高位在前=MSB)；SCK 低 3 拍(含下降沿)
//                                autopull：OSR 空则在此停等 TX FIFO，SCK 停在低位
//  1: nop        [3]    side 1   SCK 升高，等 4 拍让 MISO 经位移器往返稳定
//  2: in  pins,1        side 1   SCK 仍高，延后采样 MISO(升高后第 4 拍)；autopush 满 8 bit 自动推出
//     → wrap 回 0（SCK 回低，下一 bit）
// 每 bit 8 拍：SCK 高 5 拍、低 3 拍，采样点=上升后第 4 拍 —— 与旧手动版波形逐拍一致。
// ============================================================
static const uint16_t psoc_spi_program_instructions[] = {
    0x6201, // 0: out pins,1   side 0 [2]   驱动 MOSI，SCK 低 3 拍（autopull 自动取字节）
    0xB342, // 1: nop          side 1 [3]   SCK 高，等 4 拍让 MISO 稳定
    0x5001, // 2: in  pins,1   side 1       延后采样 MISO（升高后第 4 拍，autopush 自动推出）
};

static const struct pio_program psoc_spi_program = {
    .instructions = psoc_spi_program_instructions,
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
// DMA timeout backstop; normal seven-byte transfers complete in under 40us.
constexpr uint32_t XFER_TIMEOUT_US = 2000;
// PSoC 侧装帧窗口（本次事务结束 → 下次事务可取响应）。
// 依据(psoc_firmware/CY8C4147AZI-SensorCore/main.c: spi_isr/spi_slave_task)：RX LEVEL=6，第 7 字节
// 落入 RX FIFO 即触发 prio2 的 ISR → ReadArray(7) → ClearTxFifo → WriteArray(7)。窗口只需覆盖
// "ISR 入口延迟(可被主循环临界区推迟) + 装帧(最坏 = SNAPSHOT_BEGIN 的 36×7B 整份锁存)"。
// 150us 为长期实测稳定值，此处沿用；★它不再兼作"传输是否结束"的估时★，传输结束由 DMA 标志判定。
constexpr uint32_t PSOC_ISR_FRAME_US = 150;
// Snapshot BEGIN is acknowledged by the ISR immediately, but its 252-byte immutable copy
// intentionally happens in the PSoC main loop (spi_snapshot_latch_task).  PAGE requests
// issued before that copy return an all-zero transfer buffer while INFO still reports the
// generation as valid.  The latch task runs once per main-loop iteration and the loop spins
// tightly while waiting for the scan, so one bounded window covers it; this is paid once
// per snapshot, not per page.
constexpr uint32_t PSOC_SNAPSHOT_LATCH_GAP_US = 400;
}  // namespace

PsocSpi::PsocSpi(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin)
    : _sck_pin(sck_pin), _mosi_pin(mosi_pin), _miso_pin(miso_pin), _cs_pin(cs_pin),
      _pio(nullptr), _sm(0), _offset(0), _ready(false), _seq(0) {}

bool PsocSpi::init() {
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

    if (!_pio->load_program(&psoc_spi_program, &_offset)) {
        return false;
    }
    if (!_pio->claim_sm(&_sm)) {
        _pio->unload_program(&psoc_spi_program, _offset);
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

// 事务异常(DMA 完成标志未在上限内置起)后的复位：残留的半个字节会让之后每一帧永久错位，
// 故必须把 SM 的 ISR/OSR/移位计数与两侧 FIFO 一起清干净，并把 PC 拉回程序首指令。
void PsocSpi::_recover() {
    _pio->sm_set_enabled(_sm, false);
    _pio->sm_clear_fifos(_sm);
    _pio->sm_restart(_sm);                                   // 清 ISR/OSR/移位计数/延时
    _pio->sm_exec(_sm, (uint16_t)(0x0000u | _offset));       // jmp <program start>
    _pio->sm_set_enabled(_sm, true);
}

bool PsocSpi::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (rx != nullptr) std::memset(rx, 0, len);
    if (!_ready || len == 0) return false;

    _pio->sm_clear_fifos(_sm);
    gpio_put(_cs_pin, 0);
    busy_wait_us_32(CS_SETUP_US);
    _dma.start(tx, rx, len);
    const bool done = _dma.wait(XFER_TIMEOUT_US);
    busy_wait_us_32(CS_HOLD_US);
    gpio_put(_cs_pin, 1);
    if (!done) _recover();
    return done;
}

psoc::Frame PsocSpi::_make_request(psoc::Cmd command) {
    psoc::Frame request;
    request.clear();
    request.cmd = static_cast<uint8_t>(command);
    request.seq = _seq++;
    return request;
}

bool PsocSpi::_response_matches(const psoc::Frame& response, psoc::Cmd command, uint8_t sequence) {
    return response.magic == psoc::FRAME_MAGIC &&
           response.cmd == static_cast<uint8_t>(command) &&
           response.seq == sequence;
}

uint16_t PsocSpi::_read_u16(const uint8_t* bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8);
}

bool PsocSpi::ping() {
    if (!_ready) return false;

    psoc::Frame request = _make_request(psoc::Cmd::PING);
    psoc::Frame ignored;
    psoc::Frame fetch = _make_request(psoc::Cmd::PING);
    psoc::Frame response;

    if (!transfer(reinterpret_cast<const uint8_t*>(&request),
                  reinterpret_cast<uint8_t*>(&ignored), sizeof(request))) return false;
    busy_wait_us_32(PSOC_ISR_FRAME_US);
    if (!transfer(reinterpret_cast<const uint8_t*>(&fetch),
                  reinterpret_cast<uint8_t*>(&response), sizeof(fetch))) return false;
    return _response_matches(response, psoc::Cmd::PONG, request.seq);
}

bool PsocSpi::read_touch(uint64_t* out_mask) {
    if (!_ready || out_mask == nullptr) return false;
    // TOUCH is pipelined; retry stale or invalid frames after the PSoC ISR window.
    uint8_t tx[7];
    tx[0] = psoc::FRAME_MAGIC;
    tx[1] = static_cast<uint8_t>(psoc::Cmd::TOUCH);
    tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 0; tx[6] = 0;
    uint8_t rx[7] = {0};

    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (!transfer(tx, rx, sizeof(rx))) {
            if (attempt < 2u) busy_wait_us_32(PSOC_ISR_FRAME_US);
            continue;
        }
        if (rx[0] == psoc::FRAME_MAGIC && rx[1] == static_cast<uint8_t>(psoc::Cmd::TOUCH)) {
            uint64_t mask = 0;
            for (int i = 0; i < 5; i++) {
                mask |= (uint64_t)rx[2 + i] << (8 * i);
            }
            if ((mask >> 36) == 0ULL) {
                *out_mask = mask & 0xFFFFFFFFFULL;
                return true;
            }
        }
        if (attempt < 2u) busy_wait_us_32(PSOC_ISR_FRAME_US);
    }
    return false;
}

bool PsocSpi::_cmd_txn(uint8_t cmd, uint8_t b2, uint8_t b3, uint32_t val24, uint8_t resp[7]) {
    if (!_ready) return false;
    // txn1：发指令。PSoC RX ISR 收满 7 字节后即装响应到 TX FIFO（流水线）。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, cmd, b2, b3,
                      (uint8_t)(val24 & 0xFF), (uint8_t)((val24 >> 8) & 0xFF),
                      (uint8_t)((val24 >> 16) & 0xFF) };
    // Retry idempotent command transactions until the response command matches.
    uint8_t tx2[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::TOUCH, 0, 0, 0, 0, 0 };
    uint8_t rx1[7] = {0};
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (!transfer(tx, rx1, sizeof(rx1))) return false;
        busy_wait_us_32(PSOC_ISR_FRAME_US);
        if (!transfer(tx2, resp, 7)) return false;
        if (resp[0] != psoc::FRAME_MAGIC) return false;
        if (resp[1] == cmd) return true;
    }
    return false;
}

bool PsocSpi::set_param(uint8_t ch, uint8_t param_id, uint32_t value) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::SET_PARAM, ch, param_id, value, resp)) return false;
    // PSoC 回显 [magic, SET_PARAM, ch, param_id, val24]
    return resp[1] == (uint8_t)psoc::Cmd::SET_PARAM && resp[2] == ch && resp[3] == param_id;
}

bool PsocSpi::get_param(uint8_t ch, uint8_t param_id, uint32_t* out_value) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_PARAM, ch, param_id, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_PARAM || resp[2] != ch || resp[3] != param_id) return false;
    if (out_value) {
        *out_value = (uint32_t)resp[4] | ((uint32_t)resp[5] << 8) | ((uint32_t)resp[6] << 16);
    }
    return true;
}

bool PsocSpi::get_raw(uint8_t ch, uint16_t* out_raw) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_RAW, ch, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_RAW || resp[2] != ch) return false;
    if (out_raw) *out_raw = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::get_stats(uint32_t* out_scan_count, uint8_t* out_busy) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_STATS, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_STATS) return false;
    const uint32_t scan_count = (uint32_t)resp[3] | ((uint32_t)resp[4] << 8) |
                                ((uint32_t)resp[5] << 16) | ((uint32_t)resp[6] << 24);
    const bool scan_advanced = _stats_seen != 0u && scan_count != _last_stats_scan;
    _last_stats_scan = scan_count;
    _stats_seen = 1u;
    // A genuine heavy operation stops the PSoC main scan loop.  Some deployed
    // PSoC builds can leave the busy bit asserted after recovery despite a
    // advancing scan counter; do not let that stale bit freeze host snapshots.
    _operation_busy = (resp[2] != 0u && !scan_advanced) ? 1u : 0u;
    if (out_busy) *out_busy = resp[2];   // Raw device flag remains available to lifecycle polling.
    if (out_scan_count) *out_scan_count = scan_count;
    return true;
}

bool PsocSpi::measure_cp() {
    // BIST is a full CSD mode transition. Do not return completion until firmware has restored
    // normal sensing and dropped GET_STATS.busy; callers can then verify fresh telemetry without XRES.
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::MEASURE_CP, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::MEASURE_CP) return false;
    return _wait_op_done(10000);
}

bool PsocSpi::get_cp(uint8_t ch, uint32_t* out_cp) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_CP, ch, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_CP || resp[2] != ch) return false;
    if (out_cp) *out_cp = (uint32_t)resp[4] | ((uint32_t)resp[5] << 8) | ((uint32_t)resp[6] << 16);
    return true;
}

// Restore the default pipelined TOUCH response after command traffic.
void PsocSpi::_restore_touch_response() {
    if (!_ready) return;
    uint8_t tx[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::TOUCH, 0, 0, 0, 0, 0 };
    uint8_t discard[7] = {0};
    transfer(tx, discard, sizeof(tx));
}

bool PsocSpi::_wait_op_done(uint32_t timeout_ms, psoc::AutoTuneProgressFn on_progress,
                            void* progress_ctx, uint8_t progress_tag) {
    uint8_t resp[7];
    // 阶段1: 等 busy=1
    absolute_time_t d1 = make_timeout_time_ms(40);
    for (;;) {
        if (_cmd_txn((uint8_t)psoc::Cmd::GET_STATS, 0, 0, 0, resp) &&
            resp[1] == (uint8_t)psoc::Cmd::GET_STATS && resp[2] != 0u) {
            break;   // 已进入处理中
        }
        if (time_reached(d1)) break;   // 可能操作极快或已完成, 直接进阶段2
        sleep_ms(2);
    }
    // 阶段2: 等 busy=0
    absolute_time_t d2 = make_timeout_time_ms(timeout_ms);
    absolute_time_t next_progress = make_timeout_time_ms(PROGRESS_POLL_MS);
    for (;;) {
        if (_cmd_txn((uint8_t)psoc::Cmd::GET_STATS, 0, 0, 0, resp) &&
            resp[1] == (uint8_t)psoc::Cmd::GET_STATS && resp[2] == 0u) {
            _restore_touch_response();
            return true;   // 处理中锁定解除 = 真实完成
        }
        if (time_reached(d2)) {
            _restore_touch_response();
            return false;
        }
        // ★阶段性进度★: 长操作(自适应最坏 ~20s)期间降频(PROGRESS_POLL_MS)读一次进度回吐调用方,
        // 使上位机能持续看到"到哪一步了"; busy 判定与超时窗完全不受影响。
        if (on_progress != nullptr && time_reached(next_progress)) {
            next_progress = make_timeout_time_ms(PROGRESS_POLL_MS);
            uint8_t pr[7];
            if (_cmd_txn((uint8_t)psoc::Cmd::GET_AUTO_TUNE, 0, 0, 0, pr) &&
                pr[1] == (uint8_t)psoc::Cmd::GET_AUTO_TUNE) {
                // resp[2] = result(低 2 位) | 本轮请求标签(高 6 位, PSoC 回显)。
                const uint8_t echo_tag = (uint8_t)((pr[2] >> 2) & psoc::AUTOTUNE_TAG_MASK);
                // 标签不匹配 = 这是上一轮的进度(命令尚未被 PSoC 收下): 不回吐, 免得把旧阶段当本轮显示。
                // echo_tag==0 ⇒ 旧 PSoC 固件不带标签, 按原行为放行。
                if (progress_tag == 0u || echo_tag == 0u || echo_tag == progress_tag) {
                    psoc::AutoTuneProgress p;
                    p.state = 1;
                    p.result = (uint8_t)(pr[2] & psoc::AUTOTUNE_RESULT_MASK);
                    p.tag = echo_tag;
                    p.ch = pr[3];
                    p.cur_div = (uint16_t)((uint16_t)pr[4] | ((uint16_t)pr[5] << 8));
                    p.phase = (uint8_t)(pr[6] & 0x07u);
                    p.step = (uint8_t)((pr[6] >> 3) & 0x1Fu);
                    on_progress(progress_ctx, p);
                }
            }
        }
        sleep_ms(3);
    }
}

// 直发重操作命令并确认 PSoC 已受理。
// ★为什么必须确认(修"点了没反应 / 偶发无数据")★
// 原实现是 `transfer(cmd)` 一次、不看回显就进 _wait_op_done。而 _wait_op_done 的阶段1 只等 40ms
// busy 置起, 等不到就**默认"操作可能极快已完成"**落到阶段2, 阶段2 首轮读到 busy=0 立刻返回 true ——
// 于是"命令在 SPI 上丢了(PSoC 从未收到)"与"已完成"这两件事产生了完全相同的返回值。上位机拿到
// ACK 却什么也没发生, 而且此后 PSoC 的 TX FIFO 里停着的是别的响应(残帧), read_touch / get_stats
// 跟着错位。
// 现在: 与 _cmd_txn 同口径按 cmd 回显收割; 回显被顶掉时先看 busy —— busy 非 0 证明命令确实落了地,
// 此时**不重发**(重发 AUTO_TUNE 会让 36 通道自适应跑两遍)。收尾以 TOUCH 占位帧取响应, 顺带把
// PSoC 的默认响应换回实时触控帧, 帧边界与 _cmd_txn 完全一致。
bool PsocSpi::_send_heavy(uint8_t cmd, uint8_t b2, uint8_t b3, uint8_t b4) {
    if (!_ready) return false;
    const bool is_auto_tune = cmd == static_cast<uint8_t>(psoc::Cmd::AUTO_TUNE);
    const uint8_t wanted_tag = static_cast<uint8_t>(b4 & psoc::AUTOTUNE_TAG_MASK);
    uint8_t tx[7] = { psoc::FRAME_MAGIC, cmd, b2, b3, b4, 0, 0 };
    uint8_t tx2[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::TOUCH, 0, 0, 0, 0, 0 };
    uint8_t resp[7] = {0};

    // AUTO_TUNE 是唯一带请求身份的重操作。不能以 generic busy 证明它已受理：busy 可能属于
    // 上一条重操作，若本命令丢失则会把旧结果冒充当前请求。只发一次命令；若响应错位，随后只读
    // GET_AUTO_TUNE 查当前 channel/tag，绝不盲目重发产生第二次自适应。
    if (!transfer(tx, resp, sizeof(resp))) return false;
    busy_wait_us_32(PSOC_ISR_FRAME_US);
    if (!transfer(tx2, resp, sizeof(resp))) return false;
    if (resp[0] != psoc::FRAME_MAGIC) return false;
    if (resp[1] == cmd) {
        if (!is_auto_tune) return true;
        const uint8_t echo_tag = static_cast<uint8_t>(resp[3] & psoc::AUTOTUNE_TAG_MASK);
        // 旧固件未实现 tag 时只有「同命令 + 同通道」这一可靠受理回显，允许此唯一兼容退化。
        if (resp[2] == b2 && (echo_tag == wanted_tag || echo_tag == 0u)) return true;
    }
    if (is_auto_tune) {
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            uint8_t status[7] = {0};
            if (_cmd_txn(static_cast<uint8_t>(psoc::Cmd::GET_AUTO_TUNE), 0, 0, 0, status) &&
                status[1] == static_cast<uint8_t>(psoc::Cmd::GET_AUTO_TUNE)) {
                const uint8_t echo_tag = static_cast<uint8_t>((status[2] >> 2) & psoc::AUTOTUNE_TAG_MASK);
                if (status[3] == b2 && echo_tag == wanted_tag) return true;
            }
            sleep_ms(2);
        }
        return false;
    }

    // 其它重操作没有可回显的请求身份，保留既有「busy 表明已在执行」兼容逻辑。
    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        uint8_t busy = 0;
        if (get_stats(nullptr, &busy) && busy != 0u) return true;
        if (!transfer(tx, resp, sizeof(resp))) return false;
        busy_wait_us_32(PSOC_ISR_FRAME_US);
        if (!transfer(tx2, resp, sizeof(resp))) return false;
        if (resp[0] != psoc::FRAME_MAGIC) return false;
        if (resp[1] == cmd) return true;
    }
    return false;
}

bool PsocSpi::apply() {
    if (!begin_apply()) return false;
    return _wait_op_done(800);
}

bool PsocSpi::begin_apply() {
    if (!_ready) return false;
    // 仅确认 APPLY 已受理；完成由上层 GET_STATS 自适应观察，不能在 core1 内用短窗口阻塞。
    if (!_send_heavy((uint8_t)psoc::Cmd::APPLY, 0, 0)) return false;
    _operation_busy = 1u;
    return true;
}

bool PsocSpi::_send_runtime_param_apply(uint8_t ch, uint8_t gain, uint8_t div) {
    if (!_ready) return false;
    const uint8_t cmd = (uint8_t)psoc::Cmd::RUNTIME_PARAM_APPLY;
    uint8_t tx[7] = { psoc::FRAME_MAGIC, cmd, ch, gain, div, 0, 0 };
    uint8_t rx[7] = {0};
    if (!transfer(tx, rx, sizeof(tx))) return false;
    busy_wait_us_32(PSOC_ISR_FRAME_US);
    uint8_t fetch[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::TOUCH, 0, 0, 0, 0, 0 };
    if (!transfer(fetch, rx, sizeof(fetch))) return false;
    if (rx[0] != psoc::FRAME_MAGIC || rx[1] != cmd) return false;
    // 与 APPLY 共用相同的设备命令传输确认，但不写普通 heavy 的本地 busy 状态；
    // 完成由上层以参数回读一致和 scan_count 推进独立判定。
    return true;
}

bool PsocSpi::begin_runtime_param_apply(uint8_t ch, uint8_t gain, uint8_t div) {
    return _send_runtime_param_apply(ch, gain, div);
}

bool PsocSpi::runtime_param_apply_poll(uint32_t* out_scan_count, uint8_t* out_raw_busy) {
    uint32_t scan_count = 0u;
    uint8_t busy = 0u;
    const bool ok = get_stats(&scan_count, &busy);
    if (out_scan_count) *out_scan_count = scan_count;
    if (out_raw_busy) *out_raw_busy = busy;
    return ok;
}

bool PsocSpi::focus_scan(uint8_t ch) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::FOCUS_SCAN, ch, 0, 0, resp)) return false;
    // PSoC response: [magic, FOCUS_SCAN, applied_ch, accepted, 0, 0, 0].
    return resp[1] == (uint8_t)psoc::Cmd::FOCUS_SCAN &&
           resp[2] == ch && resp[3] == 1u;
}

bool PsocSpi::calibrate(uint8_t ch) {
    if (!_ready) return false;
    // CALIBRATE 的完成预算必须覆盖最慢分频。runtime sweep 不再逐格调用它，因此单通道 30s
    // 仅服务普通手动单通道校准；全通道逐个校准仍给 60s 覆盖高分频与
    // SPI 中断抖动。
    if (!_send_heavy((uint8_t)psoc::Cmd::CALIBRATE, ch, 0)) return false;
    return _wait_op_done((ch < 36u) ? 30000u : 60000u);
}

bool PsocSpi::baseline_reset(uint8_t ch) {
    if (!_ready) return false;
    // 基线初始化本身很快；5s 允许最慢扫描收尾，
    // 避免旧 500ms 窗口把“仍在收尾”误报成恢复失败。
    if (!_send_heavy((uint8_t)psoc::Cmd::BASELINE_RESET, ch, 0)) return false;
    return _wait_op_done(5000u);
}

bool PsocSpi::auto_tune(uint8_t ch, uint8_t pref, uint8_t tag, uint8_t* out_result, uint16_t* out_div,
                        psoc::AutoTuneProgressFn on_progress, void* progress_ctx) {
    if (!_ready) return false;
    // 触发自适应: 粗表定位 + 1 步进上探临界 + 按 pref 落档, 每次校准数百 ms → 10s 真实完成窗口。
    // 字节2 = 目标通道(0..35 单通道 / 0xFF 全通道); 字节3 = 灵敏度档位 1..7(非法值 PSoC 侧退化为 4);
    // 字节4 = 本轮请求标签(6 bit), PSoC 存下并在 GET_AUTO_TUNE 的 result 高 6 位回显。
    tag = (uint8_t)(tag & psoc::AUTOTUNE_TAG_MASK);
    if (!_send_heavy((uint8_t)psoc::Cmd::AUTO_TUNE, ch, pref, tag)) return false;
    // 单通道三步算法最坏约 24(粗定位+细搜, 二者互斥性使其不叠满) + 13(落档回退) 次单 widget 校准
    // ≈ 300-650ms。★全通道(0xFF)已改为逐通道各自校准★: 36 × 单通道 ≈ 11-23s(最坏更长) →
    // 窗口放宽到 45s, 且必须小于上位机卡死阈值(csd_diag_tick 3200 tick ≈ 51s), 否则上位机会先误报。
    if (!_wait_op_done(45000, on_progress, progress_ctx, tag)) return false;   // busy 未在窗口内清 = 超时
    // 读结果: [magic, GET_AUTO_TUNE, result|tag<<2, ch, div_lo, div_hi, progress]; 完成时 div 为最终分频。
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_AUTO_TUNE, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_AUTO_TUNE) return false;
    // ★标签校验: 陈旧结果绝不冒充本轮成功★ 本条 AUTO_TUNE 若其实没被 PSoC 收到(SPI 丢帧, 而 busy
    // 恰好因上一条重操作为 1 让 _wait_op_done 立刻返回), 这里读到的 result/div 属于上一轮。
    // 标签对不上就当"未完成"返回 false, 由上层如实上报失败(旧 PSoC 固件回显 0 ⇒ 放行, 行为不变)。
    const uint8_t echo_tag = (uint8_t)((resp[2] >> 2) & psoc::AUTOTUNE_TAG_MASK);
    if (tag != 0u && echo_tag != 0u && echo_tag != tag) return false;
    if (out_result) *out_result = (uint8_t)(resp[2] & psoc::AUTOTUNE_RESULT_MASK);
    if (out_div) *out_div = (uint16_t)((uint32_t)resp[4] | ((uint32_t)resp[5] << 8));
    return true;
}

bool PsocSpi::set_mode(uint8_t mode) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::SET_MODE, mode, 0, 0, resp)) return false;
    // PSoC 回显 [magic, SET_MODE, applied_mode, ...]
    return resp[1] == (uint8_t)psoc::Cmd::SET_MODE;
}

// ---- JIT 算法 blob 下发 ----
bool PsocSpi::algo_begin(uint16_t len) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_BEGIN, (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF), 0, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::ALGO_BEGIN;
}

bool PsocSpi::algo_page(uint16_t page, const uint8_t four[4]) {
    // 帧 [magic,ALGO_PAGE,page,d0,d1,d2,d3]: b2=page 低 8 位, b3=four[0], val24=four[1..3]
    // ★ABI v2 起 page 只是顺序校验位★: PSoC 用内部字节游标定址, 只把 (游标/4) 的低 8 位与本字节
    // 对账, 失序/越界回 page^0xFF。故这里只能发低 8 位、也只能校验低 8 位, 但调用方**必须严格
    // 顺序发页**(见 poll_upload_algo 的单向 page++)。
    const uint8_t page_lo = (uint8_t)(page & 0xFFu);
    uint8_t resp[7];
    uint32_t v = (uint32_t)four[1] | ((uint32_t)four[2] << 8) | ((uint32_t)four[3] << 16);
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_PAGE, page_lo, four[0], v, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::ALGO_PAGE && resp[2] == page_lo;
}

bool PsocSpi::algo_end(uint16_t crc16, bool* out_ok, uint16_t* out_len) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_END, (uint8_t)(crc16 & 0xFF), (uint8_t)((crc16 >> 8) & 0xFF), 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_END) return false;
    if (out_ok) *out_ok = resp[2] != 0;
    if (out_len) *out_len = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::algo_info(bool* out_valid, uint16_t* out_len, bool* out_uploading) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_INFO, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_INFO) return false;
    if (out_valid) *out_valid = resp[2] != 0;
    // b3: ABI v1 恒 0, ABI v2 = upload_active(1=正在上传)。旧固件读到 0 ⇒ 行为与改造前一致。
    if (out_uploading) *out_uploading = resp[3] != 0;
    if (out_len) *out_len = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::algo_get_crc(bool* out_valid, uint16_t* out_crc) {
    // 帧 [magic,ALGO_GET_CRC,0,...] → 响应 [..,valid,0,crc_lo,crc_hi,0]
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_CRC, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_CRC) return false;
    if (out_valid) *out_valid = resp[2] != 0;
    if (out_crc) *out_crc = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::algo_get_heap(uint16_t* out_size, uint16_t* out_used) {
    // 帧 [magic,ALGO_GET_HEAP,0,...] → 响应 [..,size_lo,size_hi,used_peak_lo,used_peak_hi,0]
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_HEAP, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_HEAP) return false;
    if (out_size) *out_size = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    if (out_used) *out_used = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::algo_get_caps(uint16_t* out_slot, uint16_t* out_heap) {
    // 帧 [magic,ALGO_GET_CAPS,0,...] → 响应 [..,heap_lo,heap_hi,slot_lo,slot_hi,0]
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_CAPS, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_CAPS) return false;
    if (out_heap) *out_heap = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    if (out_slot) *out_slot = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::begin_upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16) {
    if (!_ready || data == nullptr || len == 0u || len > PSOC_ALGO_MAX_LEN ||
        _algo_upload.phase != 0u) return false;
    if (!algo_begin(len)) return false;
    _algo_upload.data = data;
    _algo_upload.len = len;
    _algo_upload.crc16 = crc16;
    _algo_upload.page = 0u;
    _algo_upload.phase = 1u;
    _algo_upload.confirmed = 0u;
    _algo_upload.started_ms = millis();
    _algo_upload.last_info_ms = _algo_upload.started_ms - 100u;
    return true;
}

bool PsocSpi::poll_upload_algo(bool* out_complete, bool* out_ok, bool* out_valid, uint16_t* out_len) {
    if (out_complete) *out_complete = false;
    if (out_ok) *out_ok = false;
    if (out_valid) *out_valid = false;
    if (out_len) *out_len = 0u;
    if (_algo_upload.phase == 0u) return true;

    if (_algo_upload.phase == 1u) {
        uint8_t four[4] = {0, 0, 0, 0};
        const uint32_t base = (uint32_t)_algo_upload.page * 4u;
        for (uint16_t b = 0; b < 4u && base + b < _algo_upload.len; ++b) four[b] = _algo_upload.data[base + b];
        // ★不再截断成 u8★: 4KB 槽最多 1024 页, 截断后 page≥256 会回绕, 而 PSoC 只比低 8 位 ⇒
        // 回绕值恰好"校验通过", 上传报成功但字节写到了别处 = "上传成功仍跑旧算法"的根因。
        if (!algo_page(_algo_upload.page, four)) {
            _algo_upload.clear();
            if (out_complete) *out_complete = true;
            return false;
        }
        _algo_upload.page++;
        const uint16_t pages = (uint16_t)((_algo_upload.len + 3u) / 4u);
        if (_algo_upload.page < pages) return true;
        bool end_ok = false;
        uint16_t end_len = 0u;
        if (!algo_end(_algo_upload.crc16, &end_ok, &end_len) || !end_ok || end_len != _algo_upload.len) {
            _algo_upload.clear();
            if (out_complete) *out_complete = true;
            return false;
        }
        _algo_upload.phase = 2u;
        _algo_upload.last_info_ms = millis() - 100u;
        return true;
    }

    const uint32_t now_ms = millis();
    // ★上限从 120s 降到 20s★ 4KB = 1024 页, 每页一笔 SPI 事务(core1 每拍推进一笔), 正常 1~2s
    // 就走完; commit 的 CRC 校验在 PSoC 主循环里, 也只是一个扫描周期的量级。
    // 120s 的真正含义是"_algo_dl.busy 最长能把上传通道锁死两分钟" —— 那本身就是一种门禁死锁:
    // 期间用户想上传修好的算法只会拿到 DEVICE_BUSY, 而 PSoC 可能已经挂在坏算法里了。
    if ((uint32_t)(now_ms - _algo_upload.started_ms) >= ALGO_UPLOAD_TIMEOUT_MS) {
        _algo_upload.clear();
        if (out_complete) *out_complete = true;
        return false;
    }
    if ((uint32_t)(now_ms - _algo_upload.last_info_ms) < 100u) return true;
    _algo_upload.last_info_ms = now_ms;
    bool valid = false;
    bool uploading = false;
    uint16_t len = 0u;
    if (!algo_info(&valid, &len, &uploading)) return true;
    if (out_valid) *out_valid = valid;
    if (out_len) *out_len = len;
    // ★终态判据 = 内容对账, 不是 valid+len★
    // "valid && len==expected" 分不出"新算法装上了"与"旧算法还在、长度恰好相同"(同一份源改一个
    // 常量再编译, 长度几乎必然不变) —— 那正是上位机反复报"上传成功却行为没变"的来源。
    // uploading 还为真说明 PSoC 尚未 commit, 此刻的 valid/len 属于上一份, 一律不采信。
    if (valid && !uploading && len == _algo_upload.len) {
        if (++_algo_upload.confirmed < 2u) return true;   // 仍保留连续 2 次确认(防单帧巧合)
        // 再要一次槽内真实内容 CRC16: 相符才算装上。取不到就当本轮确认无效(下轮重来),
        // 不相符则直接判失败 —— 不要无限等, 否则又变成锁住通道的死等。
        bool crc_valid = false;
        uint16_t slot_crc = 0u;
        if (!algo_get_crc(&crc_valid, &slot_crc)) {
            _algo_upload.confirmed = 0u;
            return true;
        }
        const bool content_ok = crc_valid && slot_crc == _algo_upload.crc16;
        _algo_upload.clear();
        if (out_complete) *out_complete = true;
        if (out_ok) *out_ok = content_ok;
        // 返回值是"SPI 应答是否正常", 与"内容是否对上"是两件事: 这里应答一切正常, 只是内容不符,
        // 故仍返回 true 并靠 out_ok=false 让上层走"下发失败"分支(而不是混进链路失败的重试计数)。
        return true;
    }
    _algo_upload.confirmed = 0u;
    return true;
}

bool PsocSpi::upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16) {
    if (!begin_upload_algo(data, len, crc16)) return false;
    bool complete = false;
    bool ok = false;
    while (!complete) {
        if (!poll_upload_algo(&complete, &ok, nullptr, nullptr)) return false;
        sleep_ms(1);
    }
    return ok;
}

bool PsocSpi::set_algo_rom(uint8_t ch, uint16_t rom) {
    // 帧 [magic,ALGO_SET_ROM,ch,rom_lo,rom_hi,0,0]: b2=ch, b3=rom_lo, val24=rom_hi
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_SET_ROM, ch, (uint8_t)(rom & 0xFF), (uint32_t)(rom >> 8), resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::ALGO_SET_ROM && resp[2] == ch;
}

bool PsocSpi::get_algo_rom(uint8_t ch, uint16_t* out_rom) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_ROM, ch, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_ROM || resp[2] != ch) return false;
    if (out_rom) *out_rom = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::algo_get_trace(uint8_t ch, uint8_t idx, uint8_t* out_active, uint16_t* out_report) {
    // 帧 [magic,ALGO_GET_TRACE,ch,idx,0,0,0] → 响应 [..,ch,out_active,report_lo,report_hi,idx]
    // idx 回显是同命令帧的关联键；否则流水线残留的其它 report 槽会被误发布为本次读取结果。
    uint8_t resp[7];
    for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
        if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_TRACE, ch, idx, 0, resp)) return false;
        if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_TRACE || resp[2] != ch || resp[6] != idx) continue;
        if (out_active) *out_active = resp[3];
        if (out_report) *out_report = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
        return true;
    }
    return false;
}

bool PsocSpi::algo_set_cfg(uint8_t idx, uint8_t val) {
    // 帧 [magic,ALGO_SET_CFG,idx,val,0,0,0] → 响应回显 [..,idx,0,cfg[idx],0,0]
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_SET_CFG, idx, val, 0, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::ALGO_SET_CFG && resp[2] == idx;
}

bool PsocSpi::algo_get_cfg(uint8_t idx, uint8_t* out_val) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_CFG, idx, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_CFG || resp[2] != idx) return false;
    if (out_val) *out_val = (uint8_t)resp[4];
    return true;
}

bool PsocSpi::algo_set_cfg_ch(uint8_t ch, uint8_t idx, uint8_t val) {
    // 帧 [magic,SET_CFG_CH,ch,idx,val,0,0] → 响应 [..,ch,idx,实际写入值,0,0]
    // ★ch+idx 双回显都要比★: 这条命令与 GET_CFG_CH/GET_TRACE 在同一条应答流水线上, 只比 ch
    // 时另一条命令的残帧就能冒充成功(cfg 那条只有单键, 已经吃过这种苦)。
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_SET_CFG_CH, ch, idx, (uint32_t)val, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::ALGO_SET_CFG_CH && resp[2] == ch && resp[3] == idx;
}

bool PsocSpi::algo_get_cfg_ch(uint8_t ch, uint8_t idx, uint8_t* out_val) {
    // 帧 [magic,GET_CFG_CH,ch,idx,0,0,0] → 响应 [..,ch,idx,cfg_ch[ch][idx],0,0]
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_CFG_CH, ch, idx, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_CFG_CH || resp[2] != ch || resp[3] != idx) return false;
    if (out_val) *out_val = (uint8_t)resp[4];
    return true;
}

bool PsocSpi::set_global(uint8_t gparam_id, uint32_t value) {
    // 帧 [magic,SET_GLOBAL,gparam_id,0,val24]: b2=gparam_id, b3=0, val24=value
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::SET_GLOBAL, gparam_id, 0, value, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::SET_GLOBAL && resp[2] == gparam_id;
}

bool PsocSpi::get_global(uint8_t gparam_id, uint32_t* out_value) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_GLOBAL, gparam_id, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_GLOBAL || resp[2] != gparam_id) return false;
    if (out_value) *out_value = (uint32_t)resp[4] | ((uint32_t)resp[5] << 8) | ((uint32_t)resp[6] << 16);
    return true;
}

bool PsocSpi::global_commit() {
    if (!_ready) return false;
    // GLOBAL_COMMIT 只在 ISR 置 pending；必须等主循环完成 Init/Initialize 后才允许 RP2040
    // 继续排后续 PARAM_SET，避免重初始化把已排队的逐通道值覆盖。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::GLOBAL_COMMIT, 0, 0, 0, 0, 0 };
    uint8_t rx[7] = {0};
    if (!transfer(tx, rx, sizeof(tx))) return false;
    return _wait_op_done(800);
}

bool PsocSpi::indicator_on() {
    if (!_ready) return false;

    psoc::Frame request = _make_request(psoc::Cmd::INDICATOR_ON);
    psoc::Frame ignored;
    psoc::Frame fetch = _make_request(psoc::Cmd::PING);
    psoc::Frame response;

    if (!transfer(reinterpret_cast<const uint8_t*>(&request),
                  reinterpret_cast<uint8_t*>(&ignored), sizeof(request))) return false;
    busy_wait_us_32(PSOC_ISR_FRAME_US);
    if (!transfer(reinterpret_cast<const uint8_t*>(&fetch),
                  reinterpret_cast<uint8_t*>(&response), sizeof(fetch))) return false;
    return _response_matches(response, psoc::Cmd::PONG, request.seq);
}

// Latch one immutable PSoC snapshot generation and leave the transfer buffer readable.
//
// Wire contract (psoc_firmware .../main.c): SNAPSHOT_BEGIN is served in the SPI DMA ISR,
// which loads the SNAPSHOT_INFO frame straight into the single transmit slot.  That frame
// is therefore only retrievable by the *immediately following* transaction — any touch or
// command transfer squeezed in between consumes it and the whole snapshot is lost.  BEGIN
// and the INFO read must stay adjacent, so they are one indivisible step here.
//
// The 252-byte copy into transfer_snapshot is deliberately deferred by the PSoC to
// spi_snapshot_latch_task() in its main loop.  INFO is fetched with PING rather than
// PAGE(0) because a PAGE request would latch a page out of that buffer before the copy
// has run.  The bounded window afterwards covers exactly one PSoC main-loop iteration.
bool PsocSpi::_snapshot_latch(uint16_t* out_generation, bool* out_valid) {
    psoc::Frame begin = _make_request(psoc::Cmd::SNAPSHOT_BEGIN);
    psoc::Frame ignored;
    if (!transfer(reinterpret_cast<const uint8_t*>(&begin),
                  reinterpret_cast<uint8_t*>(&ignored), sizeof(begin))) return false;
    busy_wait_us_32(PSOC_ISR_FRAME_US);

    psoc::Frame fetch = _make_request(psoc::Cmd::PING);
    psoc::Frame info;
    if (!transfer(reinterpret_cast<const uint8_t*>(&fetch),
                  reinterpret_cast<uint8_t*>(&info), sizeof(fetch)) ||
        !_response_matches(info, psoc::Cmd::SNAPSHOT_INFO, begin.seq) ||
        info.payload[3] != psoc::SENSOR_CHANNEL_COUNT) {
        _restore_touch_response();
        return false;
    }

    const uint16_t generation = _read_u16(info.payload);
    const bool valid = info.payload[2] != 0u;
    if (out_generation != nullptr) *out_generation = generation;
    if (out_valid != nullptr) *out_valid = valid;
    if (valid) busy_wait_us_32(PSOC_SNAPSHOT_LATCH_GAP_US);
    return true;
}

// Read page_count consecutive snapshot pages starting at first_page into dst.
//
// PSoC page responses are pipelined one transaction behind the request, so the first
// request only primes; every later request both fetches the previous page and primes the
// next one.  A chunk must therefore be contiguous within a single call — an interleaved
// touch transfer would consume the pending SNAPSHOT_DATA frame.
bool PsocSpi::_snapshot_pages(uint16_t first_page, uint16_t page_count, uint8_t* dst) {
    if (page_count == 0u || dst == nullptr) return false;

    psoc::Frame prime = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
    prime.payload[0] = static_cast<uint8_t>(first_page);
    psoc::Frame discard;
    if (!transfer(reinterpret_cast<const uint8_t*>(&prime),
                  reinterpret_cast<uint8_t*>(&discard), sizeof(prime))) return false;

    uint8_t expected_seq = prime.seq;
    for (uint16_t i = 0u; i < page_count; ++i) {
        busy_wait_us_32(PSOC_SNAPSHOT_PAGE_DELAY_US);
        const uint16_t next_page = (uint16_t)(first_page + i + 1u);
        psoc::Frame request;
        if (next_page < psoc::SNAPSHOT_PAGE_COUNT) {
            request = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
            request.payload[0] = static_cast<uint8_t>(next_page);
        } else {
            request = _make_request(psoc::Cmd::PING);
        }
        psoc::Frame response;
        if (!transfer(reinterpret_cast<const uint8_t*>(&request),
                      reinterpret_cast<uint8_t*>(&response), sizeof(request)) ||
            !_response_matches(response, psoc::Cmd::SNAPSHOT_DATA, expected_seq)) return false;
        const size_t offset = (size_t)i * psoc::FRAME_PAYLOAD_SIZE;
        for (size_t b = 0; b < psoc::FRAME_PAYLOAD_SIZE; ++b) {
            dst[offset + b] = response.payload[b];
        }
        expected_seq = request.seq;
    }
    return true;
}

bool PsocSpi::snapshot_pump(uint8_t max_pages, psoc::SensorSnapshot* out) {
    if (!_ready || max_pages == 0) return false;

    // A stale incomplete snapshot must not monopolize paging forever; the next call
    // then restarts from a freshly published immutable PSoC generation.
    if (_snap_active && (time_us_32() - _snap_start_us) > SNAP_COMPLETE_TIMEOUT_US) {
        _snap_active = false;
    }

    if (!_snap_active) {
        if (!_snapshot_latch(&_snap_generation, &_snap_valid)) return false;
        _snap_page = 0u;
        _snap_start_us = time_us_32();
        if (!_snap_valid) {
            // The PSoC has not published a generation yet.  Report the invalid
            // generation now instead of paging 252 known-zero bytes for 16 cycles.
            _restore_touch_response();
            if (out != nullptr) {
                out->clear();
                out->generation = _snap_generation;
                out->valid = false;
            }
            return true;
        }
        _snap_active = true;
    }

    const uint16_t last_page = (uint16_t)((_snap_page + max_pages < psoc::SNAPSHOT_PAGE_COUNT)
                                          ? (_snap_page + max_pages) : psoc::SNAPSHOT_PAGE_COUNT);
    const bool chunk_ok = _snapshot_pages(_snap_page, (uint16_t)(last_page - _snap_page),
                                          &_snap_packed[(size_t)_snap_page * psoc::FRAME_PAYLOAD_SIZE]);
    _restore_touch_response();
    if (!chunk_ok) return false;
    _snap_page = last_page;
    if (_snap_page < psoc::SNAPSHOT_PAGE_COUNT) return false;

    _snap_active = false;
    if (out != nullptr) {
        out->clear();
        out->generation = _snap_generation;
        out->valid = _snap_valid;
        if (_snap_valid) {
            for (size_t ch = 0; ch < psoc::SENSOR_CHANNEL_COUNT; ++ch) {
                const size_t offset = ch * psoc::SENSOR_BYTES_PER_CHANNEL;
                auto& sample = out->channels[ch];
                sample.raw = _read_u16(&_snap_packed[offset]);
                sample.baseline = _read_u16(&_snap_packed[offset + 2u]);
                sample.diff = static_cast<int16_t>(_read_u16(&_snap_packed[offset + 4u]));
                sample.status = _snap_packed[offset + 6u];
            }
        }
    }
    return true;
}

// 单通道快照快路: 一次调用取一份完整样本(锁存 + 该通道占用的 2-3 页), 不跨 core1 周期保持
// BEGIN/INFO 相邻, 也不因分块而让页应答被触控事务吞掉。
bool PsocSpi::snapshot_pump_channel(uint8_t channel, psoc::SensorSnapshot* out) {
    if (!_ready || out == nullptr || channel >= psoc::SENSOR_CHANNEL_COUNT) return false;

    uint16_t generation = 0u;
    bool valid = false;
    if (!_snapshot_latch(&generation, &valid)) return false;
    out->generation = generation;
    out->valid = valid;
    if (!valid) {
        _restore_touch_response();
        return true;
    }

    const size_t first_byte = (size_t)channel * psoc::SENSOR_BYTES_PER_CHANNEL;
    const size_t last_byte = first_byte + psoc::SENSOR_BYTES_PER_CHANNEL - 1u;
    const uint16_t first_page = (uint16_t)(first_byte / psoc::FRAME_PAYLOAD_SIZE);
    const uint16_t last_page = (uint16_t)(last_byte / psoc::FRAME_PAYLOAD_SIZE);
    uint8_t bytes[3u * psoc::FRAME_PAYLOAD_SIZE] = {};
    const bool pages_ok = _snapshot_pages(first_page, (uint16_t)(last_page - first_page + 1u), bytes);
    _restore_touch_response();
    if (!pages_ok) return false;
    {
        const size_t offset = first_byte - (size_t)first_page * psoc::FRAME_PAYLOAD_SIZE;
        auto& sample = out->channels[channel];
        sample.raw = _read_u16(&bytes[offset]);
        sample.baseline = _read_u16(&bytes[offset + 2u]);
        sample.diff = static_cast<int16_t>(_read_u16(&bytes[offset + 4u]));
        sample.status = bytes[offset + 6u];
    }
    return true;
}

bool PsocSpi::read_snapshot(psoc::SensorSnapshot* snapshot) {
    if (!_ready || snapshot == nullptr) return false;

    snapshot->clear();
    uint16_t generation = 0u;
    bool valid = false;
    if (!_snapshot_latch(&generation, &valid)) return false;
    snapshot->generation = generation;
    snapshot->valid = valid;
    if (!valid) {
        _restore_touch_response();
        return true;
    }

    uint8_t packed[psoc::SNAPSHOT_SIZE] = {};
    const bool pages_ok = _snapshot_pages(0u, psoc::SNAPSHOT_PAGE_COUNT, packed);
    _restore_touch_response();
    if (!pages_ok) {
        snapshot->clear();
        return false;
    }

    for (size_t channel = 0; channel < psoc::SENSOR_CHANNEL_COUNT; channel++) {
        const size_t offset = channel * psoc::SENSOR_BYTES_PER_CHANNEL;
        auto& sample = snapshot->channels[channel];
        sample.raw = _read_u16(&packed[offset]);
        sample.baseline = _read_u16(&packed[offset + 2]);
        sample.diff = static_cast<int16_t>(_read_u16(&packed[offset + 4]));
        sample.status = packed[offset + 6];
    }
    return true;
}
