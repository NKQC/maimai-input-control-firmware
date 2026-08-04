#include "psoc_spi.h"
#include "../../config.h"
#include "../../hal/pio/hal_pio.h"
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
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
    if (out_busy) *out_busy = resp[2];   // PSoC 主循环重操作进行中标志
    if (out_scan_count) {
        *out_scan_count = (uint32_t)resp[3] | ((uint32_t)resp[4] << 8) |
                          ((uint32_t)resp[5] << 16) | ((uint32_t)resp[6] << 24);
    }
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
    if (!_ready) return false;
    // 发 APPLY（PSoC 主循环异步重扫/重初始化）；轮询 busy 至真实完成而非盲等固定时间。
    if (!_send_heavy((uint8_t)psoc::Cmd::APPLY, 0, 0)) return false;
    return _wait_op_done(800);   // 真实完成反馈(重初始化+首扫), 超时上限 800ms
}

bool PsocSpi::calibrate(uint8_t ch) {
    if (!_ready) return false;
    // CALIBRATE: PSoC 主循环逐通道 CalibrateWidget(重算 IDAC) + 基线复位。
    // ch=0..35 时 PSoC 只做该 widget(约全通道的 1/36) → 窗口收到 500ms; 0xFF 仍给 1500ms。
    if (!_send_heavy((uint8_t)psoc::Cmd::CALIBRATE, ch, 0)) return false;
    return _wait_op_done((ch < 36u) ? 500u : 1500u);
}

bool PsocSpi::baseline_reset(uint8_t ch) {
    if (!_ready) return false;
    // BASELINE_RESET: 单通道 = InitializeWidgetBaseline(ch), 0xFF = InitializeAllBaselines。
    if (!_send_heavy((uint8_t)psoc::Cmd::BASELINE_RESET, ch, 0)) return false;
    return _wait_op_done(500);
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

bool PsocSpi::algo_page(uint8_t page, const uint8_t four[4]) {
    // 帧 [magic,ALGO_PAGE,page,d0,d1,d2,d3]: b2=page, b3=four[0], val24=four[1..3]
    uint8_t resp[7];
    uint32_t v = (uint32_t)four[1] | ((uint32_t)four[2] << 8) | ((uint32_t)four[3] << 16);
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_PAGE, page, four[0], v, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::ALGO_PAGE && resp[2] == page;
}

bool PsocSpi::algo_end(uint16_t crc16, bool* out_ok, uint16_t* out_len) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_END, (uint8_t)(crc16 & 0xFF), (uint8_t)((crc16 >> 8) & 0xFF), 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_END) return false;
    if (out_ok) *out_ok = resp[2] != 0;
    if (out_len) *out_len = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::algo_info(bool* out_valid, uint16_t* out_len) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_INFO, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_INFO) return false;
    if (out_valid) *out_valid = resp[2] != 0;
    if (out_len) *out_len = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
}

bool PsocSpi::upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16) {
    if (!_ready || data == nullptr || len == 0 || len > 1024) return false;
    if (!algo_begin(len)) return false;

    const uint16_t pages = (uint16_t)((len + 3u) / 4u);   // 向上取整到 4 字节页
    for (uint16_t p = 0; p < pages; ++p) {
        uint8_t four[4] = {0, 0, 0, 0};
        for (uint16_t b = 0; b < 4u; ++b) {
            const uint32_t idx = (uint32_t)p * 4u + b;
            if (idx < len) four[b] = data[idx];          // 末页不足 4 字节补 0
        }
        if (!algo_page((uint8_t)p, four)) return false;
    }

    bool end_ok = false;
    uint16_t end_len = 0;
    if (!algo_end(crc16, &end_ok, &end_len)) return false;

    // PSoC commit 在其主循环执行(CRC16 校验+拷入槽); 轮询 ALGO_INFO 直到 valid 且 len 一致。
    // 采样异常时 PSoC 主循环可能慢到 ~15Hz(≈66ms/圈), commit 需跨多圈才落地; 仅轮询 40ms 会
    // 误判 "download failed"。放宽到 ~600ms(300×2ms)覆盖多个慢圈, 仍远短于用户可感知阻塞。
    for (uint16_t i = 0; i < 300u; ++i) {
        sleep_us(2000);
        bool valid = false;
        uint16_t vlen = 0;
        if (algo_info(&valid, &vlen) && valid && vlen == len) return true;
    }
    return false;
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

bool PsocSpi::snapshot_pump(uint8_t max_pages, psoc::SensorSnapshot* out) {
    if (!_ready || max_pages == 0) return false;

    // ★单次 BEGIN 锁存、跨调用续读★：PSoC 的 transfer_snapshot 锁存缓冲仅被新的 BEGIN 覆盖，
    // 触控读不会动它。故整份快照只在第一块 BEGIN 一次，后续各块无需 re-BEGIN——只要重新"引导"
    // (prime)目标页令 PSoC 把它装入 TX FIFO(其响应是触控残帧，丢弃)，随后流水线读该块页范围。
    // 好处：全程单一锁存→数据一致；每份快照仅 1 个 BEGIN→故障点极少，杜绝多块 re-BEGIN 偶发失败。
    uint8_t expected_seq;

    // ★卡死兜底(修"采样永久冻结")★：原地重试是无上限的，一旦某页因应答流水失步永远读不出来，
    // 这份快照就永不完成 → 发布态一直是旧值 → 上位机看到全通道 raw/baseline/diff 冻结。
    // 超过完成期限即丢弃已读进度，改为重新 BEGIN 一份干净快照，使遥测必然能自行恢复。
    if (_snap_active && (time_us_32() - _snap_start_us) > SNAP_COMPLETE_TIMEOUT_US) {
        _snap_active = false;
    }

    if (!_snap_active) {
        // 新快照：BEGIN 锁存 + 请求 page0 取回 INFO(generation/valid/count)。
        psoc::Frame begin = _make_request(psoc::Cmd::SNAPSHOT_BEGIN);
        psoc::Frame ignored;
        if (!transfer(reinterpret_cast<const uint8_t*>(&begin),
                      reinterpret_cast<uint8_t*>(&ignored), sizeof(begin))) return false;
        busy_wait_us_32(PSOC_SNAPSHOT_PAGE_DELAY_US);

        psoc::Frame req0 = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
        req0.payload[0] = 0;
        psoc::Frame info;
        if (!transfer(reinterpret_cast<const uint8_t*>(&req0),
                      reinterpret_cast<uint8_t*>(&info), sizeof(req0)) ||
            !_response_matches(info, psoc::Cmd::SNAPSHOT_INFO, begin.seq) ||
            info.payload[3] != psoc::SENSOR_CHANNEL_COUNT) {
            _restore_touch_response();
            return false;
        }
        _snap_generation = _read_u16(info.payload);
        _snap_valid = info.payload[2] != 0;
        _snap_page = 0;
        _snap_active = true;
        _snap_start_us = time_us_32();
        expected_seq = req0.seq;   // req0 已令 PSoC 装载 page[0]
    } else {
        // 续读：不 BEGIN。prime 请求当前页令 PSoC 装载它；prime 的响应是触控残帧，丢弃不校验。
        busy_wait_us_32(PSOC_SNAPSHOT_PAGE_DELAY_US);
        psoc::Frame prime = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
        prime.payload[0] = static_cast<uint8_t>(_snap_page);
        psoc::Frame discard;
        if (!transfer(reinterpret_cast<const uint8_t*>(&prime),
                      reinterpret_cast<uint8_t*>(&discard), sizeof(prime))) {
            _restore_touch_response();
            return false;
        }
        expected_seq = prime.seq;
    }

    // 读本块页范围：请求 PAGE[p] 取回 page[p-1] 数据（流水线）；末页用 PING 收尾。
    const uint16_t last_page = (uint16_t)((_snap_page + max_pages < psoc::SNAPSHOT_PAGE_COUNT)
                                          ? (_snap_page + max_pages) : psoc::SNAPSHOT_PAGE_COUNT);
    for (uint16_t p = (uint16_t)(_snap_page + 1); p <= last_page; p++) {
        busy_wait_us_32(PSOC_SNAPSHOT_PAGE_DELAY_US);
        psoc::Frame preq;
        if (p < psoc::SNAPSHOT_PAGE_COUNT) {
            preq = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
            preq.payload[0] = static_cast<uint8_t>(p);
        } else {
            preq = _make_request(psoc::Cmd::PING);   // 收尾取最后一页
        }
        psoc::Frame resp;
        if (!transfer(reinterpret_cast<const uint8_t*>(&preq),
                      reinterpret_cast<uint8_t*>(&resp), sizeof(preq)) ||
            !_response_matches(resp, psoc::Cmd::SNAPSHOT_DATA, expected_seq)) {
            _restore_touch_response();
            return false;
        }
        const size_t offset = (size_t)(p - 1) * psoc::FRAME_PAYLOAD_SIZE;   // 携带 page[p-1] 数据
        for (size_t b = 0; b < psoc::FRAME_PAYLOAD_SIZE; b++) {
            _snap_packed[offset + b] = resp.payload[b];
        }
        expected_seq = preq.seq;
    }

    // 收尾恢复：发一次 TOUCH 令 PSoC 把默认响应换回实时触控帧（读掉本块残留页数据）。
    // 这样下一个 update 的 read_touch 立即命中触控帧(ok=true)，消除"交替空转"——
    // 使快照读每个 update 都推进(约 2× 速度)，同时触控快路始终新鲜。
    {
        uint8_t trestore[7] = { psoc::FRAME_MAGIC, static_cast<uint8_t>(psoc::Cmd::TOUCH),
                                0, 0, 0, 0, 0 };
        uint8_t tdiscard[7];
        transfer(trestore, tdiscard, 7);
    }

    _snap_page = last_page;
    if (_snap_page < psoc::SNAPSHOT_PAGE_COUNT) {
        return false;   // 还有页未读，下次继续（锁存持久，续读无需 BEGIN）
    }

    // 全部页已读齐 → 解包输出一份来自单一锁存的一致完整快照。
    _snap_active = false;
    if (out) {
        out->clear();
        out->generation = _snap_generation;
        out->valid = _snap_valid;
        if (_snap_valid) {
            for (size_t ch = 0; ch < psoc::SENSOR_CHANNEL_COUNT; ch++) {
                const size_t o = ch * psoc::SENSOR_BYTES_PER_CHANNEL;
                auto& s = out->channels[ch];
                s.raw = _read_u16(&_snap_packed[o]);
                s.baseline = _read_u16(&_snap_packed[o + 2]);
                s.diff = static_cast<int16_t>(_read_u16(&_snap_packed[o + 4]));
                s.status = _snap_packed[o + 6];
            }
        }
    }
    return true;
}

bool PsocSpi::read_snapshot(psoc::SensorSnapshot* snapshot) {
    if (!_ready || snapshot == nullptr) return false;

    snapshot->clear();
    uint8_t packed[psoc::SNAPSHOT_SIZE] = {};
    psoc::Frame ignored;
    psoc::Frame response;
    psoc::Frame begin = _make_request(psoc::Cmd::SNAPSHOT_BEGIN);

    if (!transfer(reinterpret_cast<const uint8_t*>(&begin),
                  reinterpret_cast<uint8_t*>(&ignored), sizeof(begin))) return false;
    busy_wait_us_32(PSOC_ISR_FRAME_US);

    psoc::Frame page_request = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
    page_request.payload[0] = 0;
    if (!transfer(reinterpret_cast<const uint8_t*>(&page_request),
                  reinterpret_cast<uint8_t*>(&response), sizeof(page_request)) ||
        !_response_matches(response, psoc::Cmd::SNAPSHOT_INFO, begin.seq) ||
        response.payload[3] != psoc::SENSOR_CHANNEL_COUNT) {
        return false;
    }

    snapshot->generation = _read_u16(response.payload);
    snapshot->valid = response.payload[2] != 0;
    uint8_t expected_sequence = page_request.seq;

    for (size_t page = 1; page < psoc::SNAPSHOT_PAGE_COUNT; page++) {
        busy_wait_us_32(PSOC_ISR_FRAME_US);
        psoc::Frame next_request = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
        next_request.payload[0] = static_cast<uint8_t>(page);
        if (!transfer(reinterpret_cast<const uint8_t*>(&next_request),
                      reinterpret_cast<uint8_t*>(&response), sizeof(next_request)) ||
            !_response_matches(response, psoc::Cmd::SNAPSHOT_DATA, expected_sequence)) {
            snapshot->clear();
            return false;
        }
        const size_t offset = (page - 1) * psoc::FRAME_PAYLOAD_SIZE;
        for (size_t byte = 0; byte < psoc::FRAME_PAYLOAD_SIZE; byte++) {
            packed[offset + byte] = response.payload[byte];
        }
        expected_sequence = next_request.seq;
    }

    busy_wait_us_32(PSOC_ISR_FRAME_US);
    psoc::Frame finish = _make_request(psoc::Cmd::PING);
    if (!transfer(reinterpret_cast<const uint8_t*>(&finish),
                  reinterpret_cast<uint8_t*>(&response), sizeof(finish)) ||
        !_response_matches(response, psoc::Cmd::SNAPSHOT_DATA, expected_sequence)) {
        snapshot->clear();
        return false;
    }
    const size_t last_offset = (psoc::SNAPSHOT_PAGE_COUNT - 1) * psoc::FRAME_PAYLOAD_SIZE;
    for (size_t byte = 0; byte < psoc::FRAME_PAYLOAD_SIZE; byte++) {
        packed[last_offset + byte] = response.payload[byte];
    }

    if (!snapshot->valid) return true;

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
