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
    // 触发逐电极 BIST 测量(PSoC 主循环执行,耗时);此处仅确认 ack,结果稍后经 get_cp 读回。
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

bool PsocSpi::apply() {
    if (!_ready) return false;
    // 发 APPLY（PSoC 主循环异步重校准，耗时数 ms）；给足时间后确认。
    uint8_t tx[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::APPLY, 0, 0, 0, 0, 0 };
    uint8_t rx1[7] = {0};
    transfer(tx, rx1, 7);
    sleep_ms(60);   // 等主循环执行 Cy_CapSense_Enable + 重扫
    uint8_t tx2[7] = { psoc::FRAME_MAGIC, (uint8_t)psoc::Cmd::TOUCH, 0, 0, 0, 0, 0 };
    uint8_t resp[7] = {0};
    transfer(tx2, resp, 7);
    return resp[0] == psoc::FRAME_MAGIC;
}

bool PsocSpi::set_mode(uint8_t mode) {
    uint8_t resp[7];
    if (!_cmd_txn((uint8_t)psoc::Cmd::SET_MODE, mode, 0, 0, resp)) return false;
    // PSoC 回显 [magic, SET_MODE, applied_mode, ...]
    return resp[1] == (uint8_t)psoc::Cmd::SET_MODE;
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
