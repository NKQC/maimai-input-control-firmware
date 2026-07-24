#include "psoc_spi.h"
#include "../../config.h"
#include "../../hal/pio/hal_pio.h"
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>

// ============================================================
// PIO SPI 主机程序（MODE0 / 8bit / MSB first / 全双工）—— ★延后采样版★
// 无 pioasm 集成，故手工汇编内联机器码（编码方式对照本工程已验证的 SWD 程序）。
// side-set 1 bit = SCK(GPIO26)，out pin = MOSI(GPIO28)，in pin = MISO(GPIO27)。
// ★关键★：整条 SPI 走同一双向自适应(TXB)电平位移器，往返传播延迟大。旧版在 SCK 上升沿
// 当拍采样 MISO(零建立时间)，4MHz 起位错位。本版把采样点延后到 SCK 高相位末端(上升后隔 2 拍
// 再 in)，给 MISO 经位移器往返留固定延迟补偿，从而可把 SCK 推更高仍稳定。
// ------------------------------------------------------------
//  0: pull block        side 0        (wrap target) 等数据，SCK 空闲低
//  1: set x, 7          side 0        8 bit 计数
//  2: out pins,1 [1]    side 0        驱动 MOSI(OSR 高位在前=MSB)，SCK 低(2 拍建立)
//  3: nop        [1]    side 1        SCK 升高，等 2 拍让 MISO 经位移器往返稳定
//  4: in  pins,1        side 1        SCK 仍高，延后采样 MISO(固定延迟补偿)
//  5: jmp x-- 2         side 0        SCK 回低(下降沿)，下一 bit
//  6: push block        side 0        推入收到的字节，wrap 回 0
// 每 bit 6 拍：SCK 高 3 拍(升高后隔 2 拍采样)、低 3 拍。CYCLES_PER_BIT=6。
// ============================================================
static const uint16_t psoc_spi_program_instructions[] = {
    0x80A0, // 0: pull block   side 0
    0xE027, // 1: set x, 7     side 0
    0x6101, // 2: out pins,1   side 0 [1]   驱动 MOSI，SCK 低 2 拍
    0xB342, // 3: nop          side 1 [3]   SCK 高，等 4 拍让 MISO 稳定（延后采样余量↑）
    0x5001, // 4: in  pins,1   side 1       延后采样 MISO（升高后第 4 拍）
    0x0042, // 5: jmp x-- 2    side 0       SCK 低（下降沿）
    0x8020, // 6: push block   side 0
};

static const struct pio_program psoc_spi_program = {
    .instructions = psoc_spi_program_instructions,
    .length = 7,
    .origin = -1,
};

namespace {
constexpr uint8_t PROG_WRAP_TARGET = 0;
constexpr uint8_t PROG_WRAP = 6;
constexpr float CYCLES_PER_BIT = 8.0f;  // out2 + nop4(高相位) + in1 + jmp1 = 8；采样点=升高后第4拍
// PSoC 在主循环中消费上一事务并重装 7-byte TX FIFO；留出确定性处理窗口。
constexpr uint32_t RESPONSE_DELAY_US = 150;
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
    cfg.out_shift_right = false; cfg.autopull = false; cfg.pull_threshold = 32;  // MSB first
    cfg.in_shift_right = false;  cfg.autopush = false; cfg.push_threshold = 32;   // MSB first
    cfg.wrap_target = _offset + PROG_WRAP_TARGET;
    cfg.wrap = _offset + PROG_WRAP;
    cfg.program_offset = _offset;
    cfg.clkdiv = (float)clock_get_hz(clk_sys) / ((float)PSOC_SPI_SCK_HZ * CYCLES_PER_BIT);
    cfg.enabled = true;   // 首指令为 pull block，会自动停在此处等待数据

    if (!_pio->sm_configure(_sm, cfg)) {
        return false;
    }

    _ready = true;
    return true;
}

uint8_t PsocSpi::_xfer_byte(uint8_t out) {
    // OSR 左移取高位在前：把待发字节放到 [31:24]，out pins,1 先送 MSB
    _pio->sm_put_blocking(_sm, (uint32_t)out << 24);
    // ISR 左移收满 8 bit 后落在 [7:0]，push 上来
    uint32_t rx = _pio->sm_get_blocking(_sm);
    return (uint8_t)(rx & 0xFFu);
}

void PsocSpi::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (!_ready) {
        return;
    }
    gpio_put(_cs_pin, 0);
    for (size_t i = 0; i < len; i++) {
        uint8_t r = _xfer_byte(tx ? tx[i] : 0x00);
        if (rx) rx[i] = r;
    }
    gpio_put(_cs_pin, 1);
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

    transfer(reinterpret_cast<const uint8_t*>(&request),
             reinterpret_cast<uint8_t*>(&ignored), sizeof(request));
    sleep_us(RESPONSE_DELAY_US);
    transfer(reinterpret_cast<const uint8_t*>(&fetch),
             reinterpret_cast<uint8_t*>(&response), sizeof(fetch));
    return _response_matches(response, psoc::Cmd::PONG, request.seq);
}

bool PsocSpi::read_touch(uint64_t* out_mask) {
    if (!_ready || out_mask == nullptr) return false;
    // 单次 7 字节全双工事务：MOSI 发 TOUCH 请求，MISO 读回 PSoC 常驻的触控帧（流水线）。
    uint8_t tx[7];
    tx[0] = psoc::FRAME_MAGIC;
    tx[1] = static_cast<uint8_t>(psoc::Cmd::TOUCH);
    tx[2] = 0; tx[3] = 0; tx[4] = 0; tx[5] = 0; tx[6] = 0;
    uint8_t rx[7] = {0};
    transfer(tx, rx, 7);
    if (rx[0] != psoc::FRAME_MAGIC || rx[1] != static_cast<uint8_t>(psoc::Cmd::TOUCH)) {
        return false;
    }
    uint64_t mask = 0;
    for (int i = 0; i < 5; i++) {
        mask |= (uint64_t)rx[2 + i] << (8 * i);
    }
    *out_mask = mask & 0xFFFFFFFFFULL;   // 低 36 位有效
    return true;
}

bool PsocSpi::_cmd_txn(uint8_t cmd, uint8_t b2, uint8_t b3, uint32_t val24, uint8_t resp[7]) {
    if (!_ready) return false;
    // txn1：发指令。PSoC RX ISR 收满 7 字节后即装响应到 TX FIFO（流水线）。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, cmd, b2, b3,
                      (uint8_t)(val24 & 0xFF), (uint8_t)((val24 >> 8) & 0xFF),
                      (uint8_t)((val24 >> 16) & 0xFF) };
    uint8_t rx1[7] = {0};
    transfer(tx, rx1, 7);
    sleep_us(RESPONSE_DELAY_US);   // 等 PSoC ISR 装好响应
    // txn2：发 TOUCH 占位，读回上一步装的响应。
    uint8_t tx2[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::TOUCH, 0, 0, 0, 0, 0 };
    transfer(tx2, resp, 7);
    return resp[0] == psoc::FRAME_MAGIC;
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

bool PsocSpi::get_stats(uint32_t* out_scan_count) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_STATS, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_STATS) return false;
    if (out_scan_count) {
        *out_scan_count = (uint32_t)resp[3] | ((uint32_t)resp[4] << 8) |
                          ((uint32_t)resp[5] << 16) | ((uint32_t)resp[6] << 24);
    }
    return true;
}

bool PsocSpi::measure_cp() {
    // 命令发送后等待并校验 PSoC SPI ACK；实际逐电极测量仍由 PSoC 主循环异步执行。
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::MEASURE_CP, 0, 0, 0, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::MEASURE_CP;
}

bool PsocSpi::get_cp(uint8_t ch, uint32_t* out_cp) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_CP, ch, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_CP || resp[2] != ch) return false;
    if (out_cp) *out_cp = (uint32_t)resp[4] | ((uint32_t)resp[5] << 8) | ((uint32_t)resp[6] << 16);
    return true;
}

// 轮询 GET_STATS.busy(resp[2]) 至 PSoC 主循环真正完成重操作。两阶段避免竞态:
// 阶段1 等 busy 置起(确认命令已被 PSoC ISR 接收, 最多 40ms; 若操作极快已完成则超时后进阶段2);
// 阶段2 等 busy 落下(真实完成, 最多 timeout_ms)。返回 true=真实完成, false=超时。
bool PsocSpi::_wait_op_done(uint32_t timeout_ms) {
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
    for (;;) {
        if (_cmd_txn((uint8_t)psoc::Cmd::GET_STATS, 0, 0, 0, resp) &&
            resp[1] == (uint8_t)psoc::Cmd::GET_STATS && resp[2] == 0u) {
            return true;   // 处理中锁定解除 = 真实完成
        }
        if (time_reached(d2)) return false;
        sleep_ms(3);
    }
}

bool PsocSpi::apply() {
    if (!_ready) return false;
    // 发 APPLY（PSoC 主循环异步重扫/重初始化）；轮询 busy 至真实完成而非盲等固定时间。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::APPLY, 0, 0, 0, 0, 0 };
    uint8_t rx1[7] = {0};
    transfer(tx, rx1, 7);
    return _wait_op_done(800);   // 真实完成反馈(重初始化+首扫), 超时上限 800ms
}

bool PsocSpi::calibrate() {
    if (!_ready) return false;
    // CALIBRATE: PSoC 主循环执行 CalibrateAllWidgets(重算 IDAC, 全通道耗时) + 基线复位。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::CALIBRATE, 0, 0, 0, 0, 0 };
    uint8_t rx1[7] = {0};
    transfer(tx, rx1, 7);
    return _wait_op_done(1500);  // 全通道 IDAC 校准更慢, 给足真实完成窗口
}

bool PsocSpi::baseline_reset() {
    if (!_ready) return false;
    // BASELINE_RESET: PSoC 主循环执行 InitializeAllBaselines。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::BASELINE_RESET, 0, 0, 0, 0, 0 };
    uint8_t rx1[7] = {0};
    transfer(tx, rx1, 7);
    return _wait_op_done(500);
}

bool PsocSpi::auto_tune(uint8_t* out_result, uint16_t* out_div) {
    if (!_ready) return false;
    // 触发自适应: 逐档(最多 9 档)升分频重校准, 每档校准数百 ms → 给足 10s 真实完成窗口。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::AUTO_TUNE, 0, 0, 0, 0, 0 };
    uint8_t rx1[7] = {0};
    transfer(tx, rx1, 7);
    if (!_wait_op_done(10000)) return false;   // busy 未在 10s 内清 = 超时
    // 读结果: [magic, GET_AUTO_TUNE, result, 0, div24]。
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GET_AUTO_TUNE, 0, 0, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::GET_AUTO_TUNE) return false;
    if (out_result) *out_result = resp[2];
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
    // 帧 [magic,ALGO_GET_TRACE,ch,idx,0,0,0] → 响应 [..,ch,out_active,report_lo,report_hi,0]
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::ALGO_GET_TRACE, ch, idx, 0, resp)) return false;
    if (resp[1] != (uint8_t)psoc::Cmd::ALGO_GET_TRACE || resp[2] != ch) return false;
    if (out_active) *out_active = resp[3];
    if (out_report) *out_report = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    return true;
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
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::GLOBAL_COMMIT, 0, 0, 0, resp)) return false;
    return resp[1] == (uint8_t)psoc::Cmd::GLOBAL_COMMIT;
}

bool PsocSpi::indicator_on() {
    if (!_ready) return false;

    psoc::Frame request = _make_request(psoc::Cmd::INDICATOR_ON);
    psoc::Frame ignored;
    psoc::Frame fetch = _make_request(psoc::Cmd::PING);
    psoc::Frame response;

    transfer(reinterpret_cast<const uint8_t*>(&request),
             reinterpret_cast<uint8_t*>(&ignored), sizeof(request));
    sleep_us(RESPONSE_DELAY_US);
    transfer(reinterpret_cast<const uint8_t*>(&fetch),
             reinterpret_cast<uint8_t*>(&response), sizeof(fetch));
    return _response_matches(response, psoc::Cmd::PONG, request.seq);
}

bool PsocSpi::snapshot_pump(uint8_t max_pages, psoc::SensorSnapshot* out) {
    if (!_ready || max_pages == 0) return false;

    // ★单次 BEGIN 锁存、跨调用续读★：PSoC 的 transfer_snapshot 锁存缓冲仅被新的 BEGIN 覆盖，
    // 触控读不会动它。故整份快照只在第一块 BEGIN 一次，后续各块无需 re-BEGIN——只要重新"引导"
    // (prime)目标页令 PSoC 把它装入 TX FIFO(其响应是触控残帧，丢弃)，随后流水线读该块页范围。
    // 好处：全程单一锁存→数据一致；每份快照仅 1 个 BEGIN→故障点极少，杜绝多块 re-BEGIN 偶发失败。
    uint8_t expected_seq;

    if (!_snap_active) {
        // 新快照：BEGIN 锁存 + 请求 page0 取回 INFO(generation/valid/count)。
        psoc::Frame begin = _make_request(psoc::Cmd::SNAPSHOT_BEGIN);
        psoc::Frame ignored;
        transfer(reinterpret_cast<const uint8_t*>(&begin),
                 reinterpret_cast<uint8_t*>(&ignored), sizeof(begin));
        sleep_us(PSOC_SNAPSHOT_PAGE_DELAY_US);

        psoc::Frame req0 = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
        req0.payload[0] = 0;
        psoc::Frame info;
        transfer(reinterpret_cast<const uint8_t*>(&req0),
                 reinterpret_cast<uint8_t*>(&info), sizeof(req0));
        if (!_response_matches(info, psoc::Cmd::SNAPSHOT_INFO, begin.seq) ||
            info.payload[3] != psoc::SENSOR_CHANNEL_COUNT) {
            return false;   // 起始失败，保持空闲，下次重试
        }
        _snap_generation = _read_u16(info.payload);
        _snap_valid = info.payload[2] != 0;
        _snap_page = 0;
        _snap_active = true;
        expected_seq = req0.seq;   // req0 已令 PSoC 装载 page[0]
    } else {
        // 续读：不 BEGIN。prime 请求当前页令 PSoC 装载它；prime 的响应是触控残帧，丢弃不校验。
        sleep_us(PSOC_SNAPSHOT_PAGE_DELAY_US);
        psoc::Frame prime = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
        prime.payload[0] = static_cast<uint8_t>(_snap_page);
        psoc::Frame discard;
        transfer(reinterpret_cast<const uint8_t*>(&prime),
                 reinterpret_cast<uint8_t*>(&discard), sizeof(prime));
        expected_seq = prime.seq;  // prime 令 PSoC 装载 page[_snap_page]
    }

    // 读本块页范围：请求 PAGE[p] 取回 page[p-1] 数据（流水线）；末页用 PING 收尾。
    const uint16_t last_page = (uint16_t)((_snap_page + max_pages < psoc::SNAPSHOT_PAGE_COUNT)
                                          ? (_snap_page + max_pages) : psoc::SNAPSHOT_PAGE_COUNT);
    for (uint16_t p = (uint16_t)(_snap_page + 1); p <= last_page; p++) {
        sleep_us(PSOC_SNAPSHOT_PAGE_DELAY_US);
        psoc::Frame preq;
        if (p < psoc::SNAPSHOT_PAGE_COUNT) {
            preq = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
            preq.payload[0] = static_cast<uint8_t>(p);
        } else {
            preq = _make_request(psoc::Cmd::PING);   // 收尾取最后一页
        }
        psoc::Frame resp;
        transfer(reinterpret_cast<const uint8_t*>(&preq),
                 reinterpret_cast<uint8_t*>(&resp), sizeof(preq));
        if (!_response_matches(resp, psoc::Cmd::SNAPSHOT_DATA, expected_seq)) {
            // 中途某页失败：不回退到 page0，保留已读进度，下次 prime 当前页重试本块。
            // 锁存持久，重试无副作用；避免多块累积的偶发失败导致整份永不完成。
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

    transfer(reinterpret_cast<const uint8_t*>(&begin),
             reinterpret_cast<uint8_t*>(&ignored), sizeof(begin));
    sleep_us(RESPONSE_DELAY_US);

    psoc::Frame page_request = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
    page_request.payload[0] = 0;
    transfer(reinterpret_cast<const uint8_t*>(&page_request),
             reinterpret_cast<uint8_t*>(&response), sizeof(page_request));
    if (!_response_matches(response, psoc::Cmd::SNAPSHOT_INFO, begin.seq) ||
        response.payload[3] != psoc::SENSOR_CHANNEL_COUNT) {
        return false;
    }

    snapshot->generation = _read_u16(response.payload);
    snapshot->valid = response.payload[2] != 0;
    uint8_t expected_sequence = page_request.seq;

    for (size_t page = 1; page < psoc::SNAPSHOT_PAGE_COUNT; page++) {
        sleep_us(RESPONSE_DELAY_US);
        psoc::Frame next_request = _make_request(psoc::Cmd::SNAPSHOT_PAGE);
        next_request.payload[0] = static_cast<uint8_t>(page);
        transfer(reinterpret_cast<const uint8_t*>(&next_request),
                 reinterpret_cast<uint8_t*>(&response), sizeof(next_request));
        if (!_response_matches(response, psoc::Cmd::SNAPSHOT_DATA, expected_sequence)) {
            snapshot->clear();
            return false;
        }
        const size_t offset = (page - 1) * psoc::FRAME_PAYLOAD_SIZE;
        for (size_t byte = 0; byte < psoc::FRAME_PAYLOAD_SIZE; byte++) {
            packed[offset + byte] = response.payload[byte];
        }
        expected_sequence = next_request.seq;
    }

    sleep_us(RESPONSE_DELAY_US);
    psoc::Frame finish = _make_request(psoc::Cmd::PING);
    transfer(reinterpret_cast<const uint8_t*>(&finish),
             reinterpret_cast<uint8_t*>(&response), sizeof(finish));
    if (!_response_matches(response, psoc::Cmd::SNAPSHOT_DATA, expected_sequence)) {
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
