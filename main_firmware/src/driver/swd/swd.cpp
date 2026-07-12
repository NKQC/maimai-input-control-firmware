#include "swd.h"
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>

// ============================================================
// PIO SWD 传输程序
// 机器码依据 Raspberry Pi 官方 debugprobe 的 probe.pio 手工汇编而来
// （来源 raspberrypi/debugprobe，MIT License；本工程无 pioasm 集成，故内联机器码）。
// 命令字格式（TX FIFO）：| 13:9 Cmd(入口绝对地址) | 8 Dir(SWDIO 输出使能) | 7:0 Count(位数-1) |
// SWCLK 周期 = 4 个 PIO 执行周期（每 bit 2 条指令、各带 [1] 延时）。
// ------------------------------------------------------------
//  0: pull                          (write_cmd / turnaround_cmd)
//  1: out pins,1   [1] side 0       (write_bitloop) 主机在下降沿输出数据
//  2: jmp x-- 1    [1] side 1       目标在上升沿采样
//  3: pull         side 0           (get_next_cmd, wrap target) SWCLK 初始低
//  4: out x,8                       取位计数
//  5: out pindirs,1                 设置 SWDIO 方向
//  6: out pc,5                      跳转到命令入口
//  7: nop                           (read_bitloop) 循环分支补偿延时
//  8: in pins,1    [1] side 1       (read_cmd) 主机在上升沿采样
//  9: jmp x-- 7        side 0
// 10: push
// ============================================================
static const uint16_t swd_program_instructions[] = {
    0x80A0, // 0
    0x7101, // 1
    0x1941, // 2
    0x90A0, // 3
    0x6028, // 4
    0x6081, // 5
    0x60A5, // 6
    0xA042, // 7
    0x5901, // 8
    0x1047, // 9
    0x8020, // 10
};

static const struct pio_program swd_program = {
    .instructions = swd_program_instructions,
    .length = 11,
    .origin = -1,
};

namespace {
// PIO 程序标签（相对程序起点的偏移）
constexpr uint8_t PROG_WRITE_CMD    = 0;   // 也是 turnaround 入口
constexpr uint8_t PROG_GET_NEXT_CMD = 3;   // 命令分发入口（SM 启动点 / wrap target）
constexpr uint8_t PROG_READ_CMD     = 8;
constexpr uint8_t PROG_WRAP_TARGET  = 3;
constexpr uint8_t PROG_WRAP         = 10;

constexpr float SWCLK_HZ = 2000000.0f;    // 目标 SWCLK（>=1.5MHz 满足 acquire 窗口）

// SWD / DAP
constexpr uint32_t SWD_IDCODE_CM0P = 0x0BC11477;   // CY8C4147 / CM0+

// CPU 寄存器地址（PSoC4100S Plus，规格表 1-1）
constexpr uint32_t REG_TEST_MODE    = 0x40030014;
constexpr uint32_t REG_CPUSS_SYSREQ = 0x40100004;
constexpr uint32_t REG_CPUSS_SYSARG = 0x40100008;

// SROM 常量
constexpr uint32_t SROM_KEY1             = 0xB6;
constexpr uint32_t SROM_KEY2             = 0xD3;
constexpr uint32_t SROM_SYSREQ_BIT       = 0x80000000;
constexpr uint32_t SROM_PRIVILEGED_BIT   = 0x10000000;
constexpr uint32_t SROM_STATUS_SUCCEEDED = 0xA0000000;
constexpr uint32_t SROM_CMD_GET_SILICON_ID = 0x00;
constexpr uint32_t SROM_CMD_LOAD_LATCH   = 0x04;
constexpr uint32_t SROM_CMD_PROGRAM_ROW  = 0x06;
constexpr uint32_t SROM_CMD_ERASE_ALL    = 0x0A;
constexpr uint32_t SROM_CMD_CHECKSUM     = 0x0B;

constexpr uint32_t SRAM_PARAMS_BASE = 0x20000100;   // SROM 参数存放的 SRAM 基址

// DAP 寄存器 (apndp, addr) —— addr 为 2-bit
constexpr uint8_t DP  = 0;
constexpr uint8_t AP  = 1;
constexpr uint8_t A_IDCODE   = 0;  // DP 00b (R)
constexpr uint8_t A_CTRLSTAT = 1;  // DP 01b
constexpr uint8_t A_SELECT   = 2;  // DP 10b (W)
constexpr uint8_t A_CSW      = 0;  // AP 00b
constexpr uint8_t A_TAR      = 1;  // AP 01b
constexpr uint8_t A_DRW      = 3;  // AP 11b

constexpr int SWD_WAIT_RETRIES = 4;
}  // namespace

// ============================================================

SwdProgrammer::SwdProgrammer(uint8_t io_pin, uint8_t clk_pin, uint8_t rst_pin)
    : _io_pin(io_pin), _clk_pin(clk_pin), _rst_pin(rst_pin),
      _pio(nullptr), _sm(0), _offset(0), _ready(false), _last_idcode(0) {}

bool SwdProgrammer::init() {
    _pio = HAL_PIO0::getInstance();
    if (!_pio->init(_io_pin)) {
        return false;
    }
    // 确保 SWDIO / SWDCLK 两个引脚都交给 PIO0（init 可能因已初始化而短路）
    _pio->init_pin(_io_pin);
    _pio->init_pin(_clk_pin);

    // XRES(RST) 普通 GPIO 输出，非复位态（高）
    gpio_init(_rst_pin);
    gpio_set_dir(_rst_pin, true);
    gpio_put(_rst_pin, 1);

    // SWDIO 上拉，空闲高
    gpio_pull_up(_io_pin);

    if (!_pio->load_program(&swd_program, &_offset)) {
        return false;
    }
    if (!_pio->claim_sm(&_sm)) {
        _pio->unload_program(&swd_program, _offset);
        return false;
    }

    // SWDIO + SWDCLK（连续引脚）初始方向设为输出
    uint8_t base = (_io_pin < _clk_pin) ? _io_pin : _clk_pin;
    _pio->sm_set_pindirs_out(_sm, base, 2);

    PIOStateMachineConfig cfg;
    cfg.out_base = _io_pin; cfg.out_count = 1;
    cfg.set_base = _io_pin; cfg.set_count = 1;
    cfg.in_base = _io_pin;
    cfg.sideset_base = _clk_pin;
    cfg.sideset_bit_count = 2;      // 1 个 side bit + opt 使能位
    cfg.sideset_optional = true;
    cfg.sideset_pindirs = false;
    cfg.out_shift_right = true; cfg.autopull = false; cfg.pull_threshold = 32;  // LSB first
    cfg.in_shift_right = true;  cfg.autopush = false; cfg.push_threshold = 32;
    cfg.wrap_target = _offset + PROG_WRAP_TARGET;
    cfg.wrap = _offset + PROG_WRAP;
    cfg.program_offset = _offset;
    cfg.clkdiv = (float)clock_get_hz(clk_sys) / (SWCLK_HZ * 4.0f);
    cfg.enabled = false;   // 先不启用，跳到分发入口后再启用

    if (!_pio->sm_configure(_sm, cfg)) {
        return false;
    }

    // 关键：SM 必须从命令分发入口 get_next_cmd 开始（对齐 picoprobe），
    // 否则默认从 program_offset(=write_cmd) 起跑会把命令字当数据移位，导致全乱。
    _pio->sm_exec(_sm, pio_encode_jmp(_offset + PROG_GET_NEXT_CMD));
    _pio->sm_set_enabled(_sm, true);

    _ready = true;
    return true;
}

void SwdProgrammer::update() {
    // 保留接口，无需周期性调度
}

// ---------------- PIO 传输层原语 ----------------

void SwdProgrammer::_seq_out(uint32_t data, uint8_t nbits) {
    if (nbits == 0) return;
    uint32_t cmd = ((uint32_t)(_offset + PROG_WRITE_CMD) << 9) | (1u << 8) | ((nbits - 1) & 0xFF);
    _pio->sm_put_blocking(_sm, cmd);
    _pio->sm_put_blocking(_sm, data);
}

uint32_t SwdProgrammer::_seq_in(uint8_t nbits) {
    if (nbits == 0) return 0;
    uint32_t cmd = ((uint32_t)(_offset + PROG_READ_CMD) << 9) | (0u << 8) | ((nbits - 1) & 0xFF);
    _pio->sm_put_blocking(_sm, cmd);
    uint32_t raw = _pio->sm_get_blocking(_sm);
    return raw >> (32 - nbits);   // 右对齐（读入时右移，首 bit 落在高位）
}

void SwdProgrammer::_turnaround(uint8_t nbits) {
    if (nbits == 0) return;
    // 复用 write 入口但 Dir=0：SWDIO 高阻，仅产生时钟
    uint32_t cmd = ((uint32_t)(_offset + PROG_WRITE_CMD) << 9) | (0u << 8) | ((nbits - 1) & 0xFF);
    _pio->sm_put_blocking(_sm, cmd);
    _pio->sm_put_blocking(_sm, 0);   // dummy 数据
}

void SwdProgrammer::_line_reset() {
    _seq_out(0xFFFFFFFF, 32);
    _seq_out(0xFFFFFFFF, 32);   // >=64 clk SWDIO=1
    _seq_out(0x00000000, 8);    // idle low
}

void SwdProgrammer::_swd_connect() {
    // 标准 JTAG-to-SWD 切换 + line reset（CMSIS-DAP 惯例，对纯 SW-DP 无害）：
    // >=50 clk 高 → 16bit 0xE79E(LSB first) → >=50 clk 高 → 空闲低。
    _seq_out(0xFFFFFFFF, 32);
    _seq_out(0xFFFFFFFF, 32);
    _seq_out(0x0000E79E, 16);
    _seq_out(0xFFFFFFFF, 32);
    _seq_out(0xFFFFFFFF, 32);
    _seq_out(0x00000000, 8);
}

uint8_t SwdProgrammer::_parity32(uint32_t v) {
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 1u);
}

// ---------------- DP/AP 访问 ----------------

uint8_t SwdProgrammer::_swd_write(uint8_t apndp, uint8_t addr, uint32_t data) {
    uint8_t a2 = addr & 1u;
    uint8_t a3 = (addr >> 1) & 1u;
    uint8_t par = (apndp ^ 0u ^ a2 ^ a3) & 1u;   // RnW=0
    uint8_t hdr = (uint8_t)((1u << 0) | (apndp << 1) | (0u << 2) |
                            (a2 << 3) | (a3 << 4) | (par << 5) | (0u << 6) | (1u << 7));

    for (int attempt = 0; attempt <= SWD_WAIT_RETRIES; attempt++) {
        _seq_out(hdr, 8);
        _turnaround(1);
        uint8_t ack = (uint8_t)_seq_in(3);
        if (ack == ACK_WAIT && attempt < SWD_WAIT_RETRIES) {
            _turnaround(1);   // 归还总线后重试
            continue;
        }
        if (ack != ACK_OK) {
            _turnaround(1);
            return ack;
        }
        // 数据相：写传输在 ACK 后有一次 turnaround（目标释放，主机接管）
        _turnaround(1);
        _seq_out(data, 32);
        _seq_out(_parity32(data), 1);
        return ACK_OK;
    }
    return ACK_WAIT;
}

uint8_t SwdProgrammer::_swd_read(uint8_t apndp, uint8_t addr, uint32_t* data) {
    uint8_t a2 = addr & 1u;
    uint8_t a3 = (addr >> 1) & 1u;
    uint8_t par = (apndp ^ 1u ^ a2 ^ a3) & 1u;   // RnW=1
    uint8_t hdr = (uint8_t)((1u << 0) | (apndp << 1) | (1u << 2) |
                            (a2 << 3) | (a3 << 4) | (par << 5) | (0u << 6) | (1u << 7));

    for (int attempt = 0; attempt <= SWD_WAIT_RETRIES; attempt++) {
        _seq_out(hdr, 8);
        _turnaround(1);
        uint8_t ack = (uint8_t)_seq_in(3);
        if (ack == ACK_WAIT && attempt < SWD_WAIT_RETRIES) {
            _turnaround(1);   // 读 WAIT 无数据相，归还总线后重试
            continue;
        }
        if (ack != ACK_OK) {
            _turnaround(1);
            return ack;
        }
        uint32_t val = _seq_in(32);
        uint8_t rx_par = (uint8_t)_seq_in(1);
        _turnaround(1);   // 读毕归还总线给主机
        if (data) *data = val;
        if (_parity32(val) != rx_par) {
            return ACK_PARITY;
        }
        return ACK_OK;
    }
    return ACK_WAIT;
}

// ---------------- 经 TAR/DRW 访问任意 CPU 地址 ----------------

bool SwdProgrammer::_write_io(uint32_t addr, uint32_t data) {
    if (_swd_write(AP, A_TAR, addr) != ACK_OK) return false;
    if (_swd_write(AP, A_DRW, data) != ACK_OK) return false;
    return true;
}

bool SwdProgrammer::_read_io(uint32_t addr, uint32_t* data) {
    if (_swd_write(AP, A_TAR, addr) != ACK_OK) return false;
    uint32_t stale = 0;
    if (_swd_read(AP, A_DRW, &stale) != ACK_OK) return false;   // AP 读有一拍延迟，丢弃
    if (_swd_read(AP, A_DRW, data) != ACK_OK) return false;
    return true;
}

// ---------------- SROM ----------------

bool SwdProgrammer::_poll_srom_status() {
    absolute_time_t deadline = make_timeout_time_ms(1000);
    uint32_t st = 0;
    do {
        if (!_read_io(REG_CPUSS_SYSREQ, &st)) return false;
        if ((st & (SROM_SYSREQ_BIT | SROM_PRIVILEGED_BIT)) == 0) {
            uint32_t code = 0;
            if (!_read_io(REG_CPUSS_SYSARG, &code)) return false;
            return (code & 0xF0000000u) == SROM_STATUS_SUCCEEDED;
        }
    } while (!time_reached(deadline));
    return false;
}

// ---------------- 公共接口 ----------------

bool SwdProgrammer::acquire() {
    if (!_ready) return false;

    // 硬复位：XRES 拉低再拉高
    gpio_put(_rst_pin, 0);
    sleep_us(1500);
    gpio_put(_rst_pin, 1);

    // 复位后 ~5ms 窗口内反复尝试获取 SWD 端口
    absolute_time_t deadline = make_timeout_time_ms(5);
    uint32_t id = 0;
    bool got = false;
    do {
        _swd_connect();
        if (_swd_read(DP, A_IDCODE, &id) == ACK_OK) {
            got = true;
            break;
        }
    } while (!time_reached(deadline));
    _last_idcode = id;
    if (!got) return false;
    if (id != SWD_IDCODE_CM0P) return false;

    // 初始化调试口
    if (_swd_write(DP, A_CTRLSTAT, 0x54000000) != ACK_OK) return false;
    if (_swd_write(DP, A_SELECT, 0x00000000) != ACK_OK) return false;
    if (_swd_write(AP, A_CSW, 0x00000002) != ACK_OK) return false;

    // 进入 Test Mode
    if (!_write_io(REG_TEST_MODE, 0x80000000)) return false;
    uint32_t st = 0;
    if (!_read_io(REG_TEST_MODE, &st)) return false;
    if ((st & 0x80000000u) != 0x80000000u) return false;

    // 轮询 SROM_PRIVILEGED_BIT 清零（进入编程模式），超时 1000ms
    absolute_time_t d2 = make_timeout_time_ms(1000);
    do {
        if (!_read_io(REG_CPUSS_SYSREQ, &st)) return false;
        if ((st & SROM_PRIVILEGED_BIT) == 0) return true;
    } while (!time_reached(d2));

    return false;
}

uint32_t SwdProgrammer::read_idcode(uint8_t* ack_out) {
    uint32_t id = 0;
    _swd_connect();
    uint8_t ack = _swd_read(DP, A_IDCODE, &id);
    if (ack_out) *ack_out = ack;
    return (ack == ACK_OK) ? id : 0;
}

bool SwdProgrammer::read_silicon_id(uint32_t* out_id) {
    if (!_ready || out_id == nullptr) return false;

    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_GET_SILICON_ID) << 8);
    if (!_write_io(REG_CPUSS_SYSARG, params)) return false;
    if (!_write_io(REG_CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_GET_SILICON_ID)) return false;
    if (!_poll_srom_status()) return false;

    uint32_t p0 = 0, p1 = 0;
    if (!_read_io(REG_CPUSS_SYSARG, &p0)) return false;
    if (!_read_io(REG_CPUSS_SYSREQ, &p1)) return false;

    uint8_t hi  = (p0 >> 8) & 0xFF;
    uint8_t lo  = (p0 >> 0) & 0xFF;
    uint8_t rev = (p0 >> 16) & 0xFF;
    uint8_t fam = (p1 >> 0) & 0xFF;
    *out_id = ((uint32_t)hi << 24) | ((uint32_t)lo << 16) | ((uint32_t)rev << 8) | fam;
    return true;
}

// ---------------- 任务2b：Flash 编程流程（SROM） ----------------

bool SwdProgrammer::erase_all() {
    if (!_ready) return false;
    // 假定芯片处于 OPEN 模式（出厂/已 acquire）。参数走 SRAM。
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_ERASE_ALL) << 8);
    if (!_write_io(SRAM_PARAMS_BASE + 0x00, params)) return false;
    if (!_write_io(REG_CPUSS_SYSARG, SRAM_PARAMS_BASE)) return false;
    if (!_write_io(REG_CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_ERASE_ALL)) return false;
    return _poll_srom_status();
}

bool SwdProgrammer::checksum_all(uint32_t* out_checksum) {
    if (!_ready || out_checksum == nullptr) return false;
    // Row ID = 0x8000 表示 "Checksum All"（用户+特权行）
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_CHECKSUM) << 8) |
                      ((0x0000u & 0x00FFu) << 16) | ((0x8000u & 0xFF00u) << 16);
    if (!_write_io(REG_CPUSS_SYSARG, params)) return false;
    if (!_write_io(REG_CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_CHECKSUM)) return false;
    if (!_poll_srom_status()) return false;
    uint32_t raw = 0;
    if (!_read_io(REG_CPUSS_SYSARG, &raw)) return false;
    *out_checksum = raw & 0x0FFFFFFFu;   // 28-bit 校验和
    return true;
}

bool SwdProgrammer::_srom_load_latch(uint8_t macro_id, const uint8_t* data, uint16_t len) {
    uint32_t params1 = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_LOAD_LATCH) << 8) |
                       (0x00u << 16) | ((uint32_t)macro_id << 24);
    uint32_t params2 = (uint32_t)(len - 1);   // 装载字节数 - 1
    if (!_write_io(SRAM_PARAMS_BASE + 0x00, params1)) return false;
    if (!_write_io(SRAM_PARAMS_BASE + 0x04, params2)) return false;
    // 行数据按 32-bit 小端写入 SRAM 缓冲（偏移 0x08 起）
    for (uint16_t i = 0; i < len; i += 4) {
        uint32_t w = (uint32_t)data[i] |
                     ((uint32_t)data[i + 1] << 8) |
                     ((uint32_t)data[i + 2] << 16) |
                     ((uint32_t)data[i + 3] << 24);
        if (!_write_io(SRAM_PARAMS_BASE + 0x08 + i, w)) return false;
    }
    if (!_write_io(REG_CPUSS_SYSARG, SRAM_PARAMS_BASE)) return false;
    if (!_write_io(REG_CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_LOAD_LATCH)) return false;
    return _poll_srom_status();
}

bool SwdProgrammer::program_row(uint16_t row_id, const uint8_t* data) {
    if (!_ready || data == nullptr) return false;
    uint8_t macro_id = (uint8_t)(row_id / ROWS_PER_MACRO);   // CY8C4147 单 macro → 恒 0
    // 1. 装载 latch
    if (!_srom_load_latch(macro_id, data, ROW_SIZE)) return false;
    // 2. 编程该行
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_PROGRAM_ROW) << 8) |
                      (((uint32_t)row_id & 0x00FFu) << 16) |
                      (((uint32_t)row_id & 0xFF00u) << 16);
    if (!_write_io(SRAM_PARAMS_BASE + 0x00, params)) return false;
    if (!_write_io(REG_CPUSS_SYSARG, SRAM_PARAMS_BASE)) return false;
    if (!_write_io(REG_CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_PROGRAM_ROW)) return false;
    return _poll_srom_status();
}

bool SwdProgrammer::program_flash(const uint8_t* data, uint32_t len) {
    if (!_ready || data == nullptr || (len % ROW_SIZE) != 0) return false;
    uint16_t total_rows = (uint16_t)(len / ROW_SIZE);
    for (uint16_t row = 0; row < total_rows; row++) {
        if (!program_row(row, data + (uint32_t)row * ROW_SIZE)) return false;
    }
    return true;
}

bool SwdProgrammer::verify_flash(const uint8_t* data, uint32_t len) {
    if (!_ready || data == nullptr || (len % 4) != 0) return false;
    // flash 直接映射到 CPU 地址空间自 0x00000000，按 4 字节读回比对
    for (uint32_t addr = 0; addr < len; addr += 4) {
        uint32_t w = 0;
        if (!_read_io(addr, &w)) return false;
        uint32_t expect = (uint32_t)data[addr] |
                          ((uint32_t)data[addr + 1] << 8) |
                          ((uint32_t)data[addr + 2] << 16) |
                          ((uint32_t)data[addr + 3] << 24);
        if (w != expect) return false;
    }
    return true;
}
