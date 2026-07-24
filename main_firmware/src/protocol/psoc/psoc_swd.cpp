#include "psoc_swd.h"
#include "psoc_types.h"
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <hardware/gpio.h>

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

// SWD 电平转换器已换为 TXB 系纯推挽（hardware.txt 标注 100MHz），电气对冲/发热问题已消除
// （旧 TXS0102 自动方向+开漏取向不适配 bit-bang SWD，曾致冒烟）。故恢复到满足规格 acquire
// 窗口要求（>=1.5MHz）的 4MHz：握手远小于 400us 窗口、命中更稳；TXB 推挽 100MHz 完全支持。
constexpr float SWCLK_HZ = 4000000.0f;    // 4MHz（TXB 推挽支持；满足 acquire 窗口 >=1.5MHz）

// 单次 acquire 内整段序列的最大重试次数（有限次序列，非连续 hammer；换 TXB 后仍保留此稳健结构）。
constexpr int MAX_ACQUIRE_SEQ = 16;

// 从 psoc_types 集中定义引入（单一真源，避免魔数散落）
constexpr uint32_t SWD_IDCODE_CM0P       = psoc::SWD_IDCODE_CM0P;
constexpr uint32_t REG_TEST_MODE         = psoc::reg::TEST_MODE;
constexpr uint32_t REG_CPUSS_SYSREQ      = psoc::reg::CPUSS_SYSREQ;
constexpr uint32_t REG_CPUSS_SYSARG      = psoc::reg::CPUSS_SYSARG;
constexpr uint32_t SRAM_PARAMS_BASE      = psoc::reg::SRAM_PARAMS_BASE;
constexpr uint32_t SROM_KEY1             = psoc::srom::KEY1;
constexpr uint32_t SROM_KEY2             = psoc::srom::KEY2;
constexpr uint32_t SROM_SYSREQ_BIT       = psoc::srom::SYSREQ_BIT;
constexpr uint32_t SROM_PRIVILEGED_BIT   = psoc::srom::PRIVILEGED_BIT;
constexpr uint32_t SROM_HMASTER_BIT      = psoc::srom::HMASTER_BIT;
constexpr uint32_t SROM_REQ              = SROM_SYSREQ_BIT;
constexpr uint32_t SROM_STATUS_SUCCEEDED = psoc::srom::STATUS_SUCCEEDED;
constexpr uint32_t SROM_CMD_GET_SILICON_ID = psoc::srom::CMD_GET_SILICON_ID;
constexpr uint32_t SROM_CMD_LOAD_LATCH   = psoc::srom::CMD_LOAD_LATCH;
constexpr uint32_t SROM_CMD_PROGRAM_ROW  = psoc::srom::CMD_PROGRAM_ROW;
constexpr uint32_t SROM_CMD_ERASE_ALL    = psoc::srom::CMD_ERASE_ALL;
constexpr uint32_t SROM_CMD_CHECKSUM     = psoc::srom::CMD_CHECKSUM;
constexpr uint32_t SROM_CMD_WRITE_PROTECTION = psoc::srom::CMD_WRITE_PROTECTION;
constexpr uint32_t SROM_CMD_SET_IMO_48MHZ = psoc::srom::CMD_SET_IMO_48MHZ;

// DAP 寄存器 (apndp, addr) —— addr 为 2-bit
constexpr uint8_t DP  = 0;
constexpr uint8_t AP  = 1;
constexpr uint8_t A_IDCODE   = 0;  // DP 00b (R)
constexpr uint8_t A_CTRLSTAT = 1;  // DP 01b
constexpr uint8_t A_SELECT   = 2;  // DP 10b (W)
constexpr uint8_t A_ABORT    = 0;  // DP 00b (W) —— 写 ABORT 清 sticky error
constexpr uint8_t A_CSW      = 0;  // AP 00b
constexpr uint8_t A_TAR      = 1;  // AP 01b
constexpr uint8_t A_DRW      = 3;  // AP 11b

constexpr int SWD_WAIT_RETRIES = 4;

// AHB-AP CSW 配置（规格 Step 1A 明确值）：0x00000002 = 32-bit word 访问。
// Test Mode 编程只访问普通地址（TEST_MODE/CPUSS_SYSREQ/SYSARG/SRAM/flash），无需 PPB，故不需 HPROT。
constexpr uint32_t CSW_STD = 0x00000002;
// 特权 CSW（HPROT）：写 AIRCR(PPB) 需要。CM0+ 访问 PPB 域必须带特权位。
constexpr uint32_t CSW_PRIV = 0x03000042;

// Cortex-M0+ 系统控制：AIRCR 软复位（SYSRESETREQ）。软复位不复位 debug 域(DAP/DP)，
// 且不清 SRSS 域的 TEST_MODE 标志 → 用于 TEST_MODE workaround。
constexpr uint32_t REG_AIRCR = 0xE000ED0C;
constexpr uint32_t AIRCR_SYSRESETREQ = 0x05FA0004;   // VECTKEY(0x05FA)<<16 | SYSRESETREQ(bit2)

// Cortex-M0+ 调试寄存器（DHCSR）：debug-halt acquire 路径（对齐 OpenOCD，非 test-mode）。
constexpr uint32_t REG_DHCSR    = 0xE000EDF0;
constexpr uint32_t DHCSR_DBGKEY = 0xA05F0000;
constexpr uint32_t C_DEBUGEN    = 0x1;
constexpr uint32_t C_HALT       = 0x2;
constexpr uint32_t S_HALT       = 0x00020000;   // bit17

// bkpt-算法 SROM 执行（依据 OpenOCD psoc4.c 实证：SROM 靠 NMI，须 CPU 实际执行才被服务）。
constexpr uint32_t REG_DCRSR      = 0xE000EDF4;   // Debug Core Register Selector
constexpr uint32_t REG_DCRDR      = 0xE000EDF8;   // Debug Core Register Data
constexpr uint32_t DCRSR_REGWNR   = 0x00010000;   // bit16=写
constexpr uint32_t S_REGRDY       = 0x00010000;   // DHCSR bit16
constexpr uint32_t SROM_CODE_ADDR = 0x20000000;   // bkpt 代码地址（SRAM）
constexpr uint32_t SROM_STACK_TOP = 0x20000800;   // SROM 运行栈顶（向下；不触 params@0x100-0x188）
constexpr uint16_t THUMB_BKPT     = 0xBE00;       // bkpt #0
constexpr uint32_t XPSR_THUMB     = 0x01000000;   // T 位
// 核寄存器号（DCRSR REGSEL）：SP=13, PC=15, xPSR=16
constexpr uint8_t REG_SP   = 13;
constexpr uint8_t REG_PC   = 15;
constexpr uint8_t REG_XPSR = 16;
}  // namespace

// ============================================================

SwdProgrammer::SwdProgrammer(uint8_t io_pin, uint8_t clk_pin, uint8_t rst_pin)
    : _io_pin(io_pin), _clk_pin(clk_pin), _rst_pin(rst_pin),
      _pio(nullptr), _sm(0), _offset(0), _ready(false), _last_idcode(0),
      _last_srom_status(0), _last_fail_row(0xFFFF), _last_fail_addr(0xFFFFFFFF),
      _last_verify_read(0), _last_verify_expect(0),
      _last_chip_prot(0xFF), _last_prot_raw(0), _last_rowprot0(0), _last_rowprot1(0),
      _last_acquire_status(0), _last_acquire_sysreq(0), _last_acquire_delay(0),
      _clk_trim1(0), _clk_trim3(0), _sfl_trim_word(0), _sfl_tctrim_word(0),
      _clock_config_ok(false), _clock_select(0), _clock_imo_select(0),
      _clock_trim1(0), _clock_trim2(0), _clock_trim3(0),
      _erase_scan_complete(false), _erase_flash_sum(0), _erase_flash_or(0),
      _erase_first_nonzero_addr(0xFFFFFFFFu), _erase_first_nonzero_value(0),
      _erase_words_read(0) {}

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
    _rst_ready = true;   // 本机拥有 XRES 控制权; 即使随后 release_swd() 也保留(供运行时 REBOOT_PSOC 复位)

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
    // 纯 SW-DP line reset（规格附录C / AN84858 SwdLineReset）：>=50 clk SWDIO=HIGH → 拉低回 IDLE。
    // ★去掉 JTAG→SWD 切换序列(0xE79E)★——module-builder 精读 psoc4_progspec/an84858 结论：
    // 全文无此要求，纯 SWD 目标不需要，且多占时钟会把 acquire 握手拖出 400us 窗口。
    _seq_out(0xFFFFFFFF, 32);
    _seq_out(0xFFFFFFFF, 32);   // 64 clk 高（>=51，满足 line reset 定义）
    _seq_out(0x00000000, 8);    // 回 IDLE
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

// 写 Cortex-M0+ 核寄存器（PPB 域 DCRSR/DCRDR，需 CSW_PRIV，由 _connect_and_halt 已设好并保持）。
bool SwdProgrammer::_write_core_reg(uint8_t reg, uint32_t val) {
    if (!_write_io(REG_DCRDR, val)) return false;
    if (!_write_io(REG_DCRSR, ((uint32_t)reg & 0x7Fu) | DCRSR_REGWNR)) return false;
    absolute_time_t deadline = make_timeout_time_ms(50);
    uint32_t d = 0;
    do {
        if (!_read_io(REG_DHCSR, &d)) return false;
        if (d & S_REGRDY) return true;
    } while (!time_reached(deadline));
    return false;
}

// test-mode 下由 boot ROM 服务 SROM 请求；主机经 DAP 写 SYSREQ 后轮询完成状态。
bool SwdProgrammer::_srom_exec(uint32_t cmd) {
    if (!_write_io(REG_CPUSS_SYSREQ, SROM_REQ | cmd)) { _last_srom_status = 0xDEAD0012u; return false; }
    absolute_time_t deadline = make_timeout_time_ms(1000);
    uint32_t st = 0;
    do {
        if (!_read_io(REG_CPUSS_SYSREQ, &st)) { _last_srom_status = 0xDEAD0011u; return false; }
        if ((st & (SROM_SYSREQ_BIT | SROM_PRIVILEGED_BIT)) == 0) {
            uint32_t code = 0;
            if (!_read_io(REG_CPUSS_SYSARG, &code)) { _last_srom_status = 0xDEAD0013u; return false; }
            _last_srom_status = code;
            return (code & 0xF0000000u) == SROM_STATUS_SUCCEEDED;
        }
    } while (!time_reached(deadline));
    _last_srom_status = 0xDEAD00FFu;
    return false;
}

// ---------------- 公共接口 ----------------

// 编程模式获取：按规格在 XRES 后的启动窗口内完成 DAP 连接和 TEST_MODE 握手，
// 然后等待 SROM 释放 PRIVILEGED 位。
bool SwdProgrammer::acquire() {
    if (!_ready) return false;
    // 保护性：外层重试 20→6，配合有限次序列，避免长时间连续驱动 SWD 线（防 TXS0102 对冲发热）。
    for (uint32_t attempt = 0; attempt < 6; attempt++) {
        gpio_put(_rst_pin, 0); sleep_us(100); gpio_put(_rst_pin, 1);
        bool entered = false;
        absolute_time_t win = make_timeout_time_ms(5);
        do {
            _line_reset();
            uint32_t id = 0;
            if (_swd_read(DP, A_IDCODE, &id) != ACK_OK || id != SWD_IDCODE_CM0P) continue;
            _last_idcode = id;
            if (_swd_write(DP, A_CTRLSTAT, 0x54000000) != ACK_OK) continue;
            if (_swd_write(DP, A_SELECT,   0x00000000) != ACK_OK) continue;
            if (_swd_write(AP, A_CSW,      CSW_STD)    != ACK_OK) continue;
            if (!_write_io(REG_TEST_MODE, 0x80000000)) continue;
            uint32_t tm = 0;
            if (!_read_io(REG_TEST_MODE, &tm)) continue;
            if ((tm & 0x80000000u) == 0x80000000u) { entered = true; break; }
        } while (!time_reached(win));
        if (!entered) continue;
        absolute_time_t rdy = make_timeout_time_ms(1000);
        uint32_t sr = 0;
        do {
            if (!_read_io(REG_CPUSS_SYSREQ, &sr)) break;
            _last_acquire_sysreq = sr;
            if ((sr & SROM_PRIVILEGED_BIT) == 0) {
                _read_io(REG_CPUSS_SYSARG, &_last_acquire_status);
                _last_acquire_delay = attempt;
                return true;
            }
        } while (!time_reached(rdy));
    }
    _last_acquire_delay = 0xFFFFFFFFu;
    return false;
}

bool SwdProgrammer::set_program_indicator() {
    using namespace psoc;
    if (!_ready) return false;

    // 等价于 generated Cy_GPIO_Pin_Init(P1.6, out=1, STRONG_IN_OFF, HSIOM_GPIO)。
    // 先置输出锁存，再启用驱动，避免短暂低脉冲；所有地址/位域来自本地 PDL。
    if (!_write_io(reg::GPIO_PRT1_DR_SET, reg::STATUS_LED_MASK)) return false;

    uint32_t pc = 0;
    uint32_t pc2 = 0;
    uint32_t hsiom = 0;
    if (!_read_io(reg::GPIO_PRT1_PC, &pc) ||
        !_read_io(reg::GPIO_PRT1_PC2, &pc2) ||
        !_read_io(reg::HSIOM_PRT1_SEL, &hsiom)) return false;

    pc = (pc & ~reg::STATUS_LED_PC_MASK) | reg::STATUS_LED_PC_VALUE;
    pc2 |= reg::STATUS_LED_MASK;  // STRONG_IN_OFF 的 input-buffer-disable 位
    hsiom &= ~reg::STATUS_LED_HSIOM_MASK;  // HSIOM_SEL_GPIO = 0
    if (!_write_io(reg::GPIO_PRT1_PC, pc) ||
        !_write_io(reg::GPIO_PRT1_PC2, pc2) ||
        !_write_io(reg::HSIOM_PRT1_SEL, hsiom)) return false;

    uint32_t dr_verify = 0;
    uint32_t pc_verify = 0;
    uint32_t pc2_verify = 0;
    uint32_t hsiom_verify = 0;
    return _read_io(reg::GPIO_PRT1_DR, &dr_verify) &&
           _read_io(reg::GPIO_PRT1_PC, &pc_verify) &&
           _read_io(reg::GPIO_PRT1_PC2, &pc2_verify) &&
           _read_io(reg::HSIOM_PRT1_SEL, &hsiom_verify) &&
           (dr_verify & reg::STATUS_LED_MASK) != 0 &&
           (pc_verify & reg::STATUS_LED_PC_MASK) == reg::STATUS_LED_PC_VALUE &&
           (pc2_verify & reg::STATUS_LED_MASK) != 0 &&
           (hsiom_verify & reg::STATUS_LED_HSIOM_MASK) == 0;
}

// OpenOCD 风格 debug-halt acquire：标准 DAP 连接 → DP 上电 → 标准 CSW → 写 DHCSR 停核 →
// 轮询 S_HALT。不进 test mode，不依赖 XRES 抢窗时序。依据 psoc_swd_bringup.md 根因结论。
bool SwdProgrammer::_connect_and_halt() {
    // ★connect-under-reset + halt★（诊断确证：运行中的 CapSense 固件重配了 SWD 引脚，
    // 不复位则连 IDCODE 都读不到；必须 XRES 复位后趁 ROM boot 窗口(SWD 尚可用)立即连接
    // 并写 DHCSR 停核——核一旦 halt 就不会跳到用户固件重配引脚，SWD 保活，SROM 可执行）。
    // 1. XRES 硬复位，释放后打开 boot 窗口。
    gpio_put(_rst_pin, 0);
    sleep_us(100);
    gpio_put(_rst_pin, 1);

    // 2. 窗口内尽快连接 + 停核（少事务、命中即停）。第一拍成功即在 boot 早期停核。
    absolute_time_t deadline = make_timeout_time_ms(8);
    uint32_t dhcsr = 0;
    bool halted = false;
    do {
        _line_reset();
        uint32_t id = 0;
        if (_swd_read(DP, A_IDCODE, &id) != ACK_OK || id != SWD_IDCODE_CM0P) continue;
        _last_idcode = id;
        _swd_write(DP, A_ABORT, 0x0000001Eu);                 // 清 sticky
        if (_swd_write(DP, A_CTRLSTAT, 0x54000000) != ACK_OK) continue;   // DP 上电
        if (_swd_write(DP, A_SELECT,   0x00000000) != ACK_OK) continue;
        // ★PPB 区(DHCSR 0xE000EDF0)必须用特权 CSW(HPROT)访问；CSW_STD 访问不了 PPB★
        if (_swd_write(AP, A_CSW,      CSW_PRIV)   != ACK_OK) continue;
        if (!_write_io(REG_DHCSR, DHCSR_DBGKEY | C_HALT | C_DEBUGEN)) continue;  // 停核
        if (_read_io(REG_DHCSR, &dhcsr) && (dhcsr & S_HALT)) { halted = true; break; }
    } while (!time_reached(deadline));

    // 诊断：_last_acquire_status = DHCSR（期望含 S_HALT=0x00020000）；
    //       _last_acquire_sysreq = CPUID(0xE000ED00)（MEM-AP 健全性，CM0+≈0x410CC601）。
    _last_acquire_status = dhcsr;
    uint32_t cpuid = 0xBADAC000u;
    _read_io(0xE000ED00u, &cpuid);
    _last_acquire_sysreq = cpuid;
    return halted;
}

// TEST_MODE workaround —— 对照 OpenOCD psoc4.cfg 的 PSOC4_TEST_MODE_WORKAROUND 路径。
// 顺序：连 SWD → 配 DP（特权 CSW 以便写 PPB 的 AIRCR）→ 写 TEST_MODE=1 → SYSRESETREQ 软复位
//       → 等 boot 完成（CPU 检查 TEST_MODE 停在 ROM）→ 重连 DP → 交由外层特权探针甄别。
bool SwdProgrammer::_acquire_test_mode_workaround() {
    // 1. 连接 SWD（此刻 target 在跑上一状态，未复位）。
    _swd_connect();
    uint32_t id = 0;
    if (_swd_read(DP, A_IDCODE, &id) != ACK_OK || id != SWD_IDCODE_CM0P) return false;
    _last_idcode = id;

    // 2. 初始化 DP + 上电请求 + 特权 CSW（写 AIRCR 需访问 PPB）。
    if (_swd_write(DP, A_CTRLSTAT, 0x54000000) != ACK_OK) return false;
    if (_swd_write(DP, A_SELECT,   0x00000000) != ACK_OK) return false;
    if (_swd_write(AP, A_CSW,      CSW_PRIV)   != ACK_OK) return false;

    // 3. 复位前置位 TEST_MODE（关键：软复位后 CPU 从 ROM 启动会检查它）。
    if (!_write_io(REG_TEST_MODE, 0x80000000)) return false;

    // 4. 触发 SYSRESETREQ 软复位。复位会打断当前 SWD 事务，故忽略返回值。
    _write_io(REG_AIRCR, AIRCR_SYSRESETREQ);

    // 5. 等待内部复位(<1ms)+boot(<4ms)完成，CPU 检查 TEST_MODE 后停在 system ROM。
    sleep_ms(10);

    // 6. 软复位不复位 debug 域，但保险起见重连并复配 DP（探针访问 CPUSS 用标准 CSW 即可）。
    _swd_connect();
    if (_swd_read(DP, A_IDCODE, &id) != ACK_OK) return false;
    if (_swd_write(DP, A_CTRLSTAT, 0x54000000) != ACK_OK) return false;
    if (_swd_write(DP, A_SELECT,   0x00000000) != ACK_OK) return false;
    if (_swd_write(AP, A_CSW,      CSW_STD)     != ACK_OK) return false;

    // 诊断：记录复位后 TEST_MODE 读回与 CPUSS_SYSREQ。
    _read_io(REG_TEST_MODE, &_last_acquire_status);
    _read_io(REG_CPUSS_SYSREQ, &_last_acquire_sysreq);
    return true;   // 真伪由外层 _probe_programming_mode() 判定
}

bool SwdProgrammer::_acquire_once(uint32_t window_delay_us) {
    (void)window_delay_us;   // 保留签名；本实现用连续 hammer 覆盖窗口，不用固定延时扫描
    // 1. 硬复位：XRES 低脉冲 100us（规格最小 5us）。两线全程留 PIO0。
    gpio_put(_rst_pin, 0);
    sleep_us(100);
    gpio_put(_rst_pin, 1);   // 释放 XRES —— 内部复位(<1ms)+boot(<4ms) 后打开 400us 获取窗口

    // 2. ★保护性有限次序列重试（不再 8ms 连续 hammer）。★
    //    ⚠ 上一片冒烟根因 = TXS0102 电平转换器不适配 bit-bang 推挽 SWD，叠加"8ms 连续数千次事务"
    //      把 turnaround 期的驱动对冲放大成持续大电流 → 热损坏。故【绝不再连续 hammer】。
    //    规格 p21：boot code 监视【完整 acquire 序列】(line reset→读 ID→配 DP→写 TEST_MODE)，
    //      而非单寄存器值；任一步失败 loop 回 START。这里保留整段序列，但只做【有限次数 + 短预算】，
    //      每轮之间让 SWD 线回到空闲(line reset 末尾拉低+idle)，把连续驱动时间压到最小。
    //    命中窗口问题在硬件修正（换方向可控转换器 / 加串阻 / SWD 直连）前不可靠，本版仅保命。
    uint32_t id = 0;
    bool any_id = false;
    for (int seq = 0; seq < MAX_ACQUIRE_SEQ; seq++) {
        _line_reset();
        if (_swd_read(DP, A_IDCODE, &id) != ACK_OK || id != SWD_IDCODE_CM0P) continue;  // 未连上/ID 不符
        any_id = true;
        _last_idcode = id;
        _swd_write(DP, A_ABORT, 0x0000001Eu);                       // 清 sticky error
        if (_swd_write(DP, A_CTRLSTAT, 0x54000000) != ACK_OK) continue;
        if (_swd_write(DP, A_SELECT,   0x00000000) != ACK_OK) continue;
        if (_swd_write(AP, A_CSW,      CSW_STD)    != ACK_OK) continue;
        _write_io(REG_TEST_MODE, 0x80000000);                       // 写 TEST_MODE
    }
    if (!any_id) return false;   // 全程没连上 DAP → XRES/硬件问题

    // 3. 诊断：记录序列循环后的 TEST_MODE 读回(status)与 CPUSS_SYSREQ(sysreq)。
    //    真伪交外层特权探针甄别（bit31 假阳性不可信）。
    _read_io(REG_TEST_MODE, &_last_acquire_status);
    _read_io(REG_CPUSS_SYSREQ, &_last_acquire_sysreq);
    return true;
}

// 特权 SROM 探针：甄别是否真正进入编程模式（bit31 回读假阳性不可信）。
// 依据 module-builder 精读规格 Step 1A：SET_IMO=48MHz 是 erase/program 的前置（规格末步），
// 故先设 IMO 再用 ERASE_ALL 探针，避免"已进 test mode 但因时钟未设导致 ERASE 报 0xF0000012"被误判为失败。
// - 真正进入编程模式 → ERASE 返回 0xA0000000 成功（空片 VIRGIN 上 ERASE 无损，且正是最终所需操作）；
// - DEAD 态 → 0xF0000014 快速失败。
// SET_IMO 结果记入 _last_srom_status 前先被 ERASE 覆盖，故 SET_IMO 失败不影响判据（仅作前置尝试）。
bool SwdProgrammer::_probe_programming_mode() {
    // ★诊断版★：用最简的 GET_SILICON_ID(非特权 SROM)判定 test-mode 下 SROM 能否执行,
    // 并读回芯片保护状态,以定位 ERASE 的 0xF0000014(极可能是 protection≠OPEN)。
    // 结果映射(供 COM 诊断观察):
    //   imo(_last_srom_status) = GET_SILICON_ID 的 SROM 返回码(0xA0000000=SROM 在 test-mode 正常执行)
    //   sysarg(_last_acquire_status) = SYSARG 回读(silicon id 字节)
    //   sysreq(_last_acquire_sysreq) = SYSREQ 回读(family + prot[15:12])
    //   prot(_last_chip_prot) = 芯片保护模式(01=OPEN,02=PROTECTED,00=VIRGIN,04=KILL)
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_GET_SILICON_ID) << 8);
    if (!_write_io(REG_CPUSS_SYSARG, params)) return false;
    bool ok = _srom_exec(SROM_CMD_GET_SILICON_ID);
    uint32_t p0 = 0, p1 = 0;
    _read_io(REG_CPUSS_SYSARG, &p0);
    _read_io(REG_CPUSS_SYSREQ, &p1);
    _last_acquire_status = p0;
    _last_acquire_sysreq = p1;
    _last_chip_prot = (uint8_t)((p1 >> 12) & 0x0F);
    return ok;
}

// SET_IMO=48MHz：部分器件在 flash 擦/写前必需（规格 Table 1-1）。
// OpenOCD 官方实践：0xF0000013 表示本命令未在该器件实现，视为可安全忽略的"无需"信号；
// 其它任何非成功码都是真实错误。
bool SwdProgrammer::set_imo_48mhz() {
    if (!_ready) return false;
    // ★规格 + AN84858 权威：CY8C4147(41xxS plus)必须在 flash 擦/写前用 SROM SET_IMO_48MHz(0x15)
    //   设时钟，且该操作应属 Device Acquire 例程。之前"改走直接寄存器"是坏硬件(TXS0102)时代
    //   SROM 0x15 返回 0xF0000014 的妥协；换 TXB 推挽后 SWD 全链路已通，重试规格 SROM 方式。★
    // 规格伪码：Params=KEY1|((KEY2+0x15)<<8) 写 CPUSS_SYSARG(直接值)，SYSREQ 触发，PollSromStatus。
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_SET_IMO_48MHZ) << 8);
    if (!_write_io(REG_CPUSS_SYSARG, params)) { _last_srom_status = 0xDEAD0015u; return false; }
    bool srom_ok = _srom_exec(SROM_CMD_SET_IMO_48MHZ);   // _srom_exec 已把 SROM 返回码存入 _last_srom_status
    // 保留直接寄存器配置作后备（不覆盖 _last_srom_status，使 diagnose 的 imo 字段显示 SROM 结果码）。
    configure_flash_clock();
    return srom_ok;
}

bool SwdProgrammer::configure_flash_clock() {
    using namespace psoc;
    _clock_config_ok = false;
    _clock_select = 0;
    _clock_imo_select = 0;
    _clock_trim1 = 0;
    _clock_trim2 = 0;
    _clock_trim3 = 0;

    uint32_t w = 0;
    // ① 退回 24MHz 基准
    if (!_write_io(reg::CLK_IMO_SELECT, 0x0)) return false;
    // ② 读 SFLASH 48MHz 粗调 trim(LT24,字节车道1) → CLK_IMO_TRIM1
    if (!_read_io(reg::SFLASH_IMO_TRIM_LT24_WORD, &w)) return false;
    uint8_t off = (uint8_t)((w >> 8) & 0xFF);
    if (!_write_io(reg::CLK_IMO_TRIM1, off)) return false;
    // ③ 清细调
    if (!_write_io(reg::CLK_IMO_TRIM2, 0x0)) return false;
    // ④ 读 SFLASH 温漂 trim(TCTRIM_LT24,字节车道0) → CLK_IMO_TRIM3
    if (!_read_io(reg::SFLASH_IMO_TCTRIM_LT24_WORD, &w)) return false;
    uint8_t tct = (uint8_t)(w & 0xFF);
    if (!_write_io(reg::CLK_IMO_TRIM3, tct)) return false;
    sleep_us(20);   // ≥50 IMO 周期@24MHz
    // ⑥ 目标48M>24M: 先中间档44M 再48M
    if (!_write_io(reg::CLK_IMO_SELECT, 0x5)) return false;
    sleep_us(20);
    if (!_write_io(reg::CLK_IMO_SELECT, 0x6)) return false;
    sleep_us(20);
    // ⑦ CLK_SELECT 读-改-写: PUMP_SEL[5:4]=1(IMO), HFCLK源=IMO, HFCLK_DIV=/1
    if (!_read_io(reg::CLK_SELECT, &w)) return false;
    w = (w & ~(0x3u << 4)) | (0x1u << 4);
    w = (w & ~(0x3u << 2)) | (0x0u << 2);
    w = (w & ~(0x3u << 0)) | (0x0u << 0);
    if (!_write_io(reg::CLK_SELECT, w)) return false;

    // 保存擦除实际使用的五个时钟寄存器；任一回读失败都视为配置失败。
    if (!_read_io(reg::CLK_SELECT, &_clock_select)) return false;
    if (!_read_io(reg::CLK_IMO_SELECT, &_clock_imo_select)) return false;
    if (!_read_io(reg::CLK_IMO_TRIM1, &_clock_trim1)) return false;
    if (!_read_io(reg::CLK_IMO_TRIM2, &_clock_trim2)) return false;
    if (!_read_io(reg::CLK_IMO_TRIM3, &_clock_trim3)) return false;

    _clock_config_ok = (_clock_imo_select & 0x7u) == 0x6u &&
                       ((_clock_select >> 4) & 0x3u) == 0x1u &&
                       ((_clock_select >> 2) & 0x3u) == 0x0u &&
                       (_clock_select & 0x3u) == 0x0u;
    return _clock_config_ok;
}

bool SwdProgrammer::read_clk_trim_snapshot() {
    using namespace psoc;
    bool ok = true;
    // t1/t3 复用为"读健康 SFLASH 出厂数据"探针,判定 flash 区 AHB 读是否工作/SFLASH 是否空白:
    //   t1 = SFLASH_SILICON_ID(0x0FFFF244,出厂硅ID,健康片必非0)
    //   t3 = 0x0FFFF240(硅ID 邻近字)
    ok &= _read_io(0x0FFFF244u, &_clk_trim1);
    ok &= _read_io(0x0FFFF240u, &_clk_trim3);
    ok &= _read_io(reg::SFLASH_IMO_TRIM_LT24_WORD, &_sfl_trim_word);      // 0x0FFFF37C(trim LT23/LT24)
    ok &= _read_io(reg::SFLASH_IMO_TCTRIM_LT24_WORD, &_sfl_tctrim_word);  // 0x0FFFF364(tctrim LT24)
    return ok;
}

void SwdProgrammer::reset_target_run() {
    // ★不能用 _ready 守卫★: release_swd() 后 _ready=false 但 XRES 仍是本机 GPIO 输出,
    // 之前该守卫导致运行时 REBOOT_PSOC 静默无效(XRES 从不脉冲→PSoC 不重启→启动白灯不亮)。
    // 只要 init() 配置过 XRES(_rst_ready) 即可脉冲复位; SWD_RELEASE_TO_EXTERNAL(未 init)不驱动。
    if (!_rst_ready) return;
    gpio_set_dir(_rst_pin, true);   // 确保为输出(release_swd 后仍是 OUT, 这里冗余保险)
    // 仅脉冲 XRES 复位。复位后不发送 SWD line reset / 不进 Test Mode，
    // 目标在启动窗口内无调试握手即引导用户固件正常运行。
    gpio_put(_rst_pin, 0);
    sleep_us(2000);
    gpio_put(_rst_pin, 1);
}

void SwdProgrammer::release_swd() {
    gpio_init(_io_pin);  gpio_set_dir(_io_pin, GPIO_IN);  gpio_disable_pulls(_io_pin);
    gpio_init(_clk_pin); gpio_set_dir(_clk_pin, GPIO_IN); gpio_disable_pulls(_clk_pin);
    // XRES 保持输出高(不复位),让 PSoC 运行;若已有 reset_target_run 置高即可,这里确保为高。
    gpio_init(_rst_pin); gpio_set_dir(_rst_pin, GPIO_OUT); gpio_put(_rst_pin, 1);
    _ready = false;   // 之后不应再发起 SWD 操作
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
    if (!_srom_exec(SROM_CMD_GET_SILICON_ID)) return false;

    uint32_t p0 = 0, p1 = 0;
    if (!_read_io(REG_CPUSS_SYSARG, &p0)) return false;
    if (!_read_io(REG_CPUSS_SYSREQ, &p1)) return false;

    uint8_t hi  = (p0 >> 8) & 0xFF;
    uint8_t lo  = (p0 >> 0) & 0xFF;
    uint8_t rev = (p0 >> 16) & 0xFF;
    uint8_t fam = (p1 >> 0) & 0xFF;
    *out_id = ((uint32_t)hi << 24) | ((uint32_t)lo << 16) | ((uint32_t)rev << 8) | fam;
    // 芯片保护模式在 CPUSS_SYSREQ[15:12]（GET_SILICON_ID 副产物）
    return true;
}

// 经 SFLASH macro0 读取芯片级保护模式。
bool SwdProgrammer::read_chip_protection(uint8_t* out_prot) {
    if (out_prot == nullptr) return false;

    uint32_t addr = psoc::reg::SFLASH_MACRO0 + ROW_SIZE - 4;
    uint32_t v = 0;
    if (!_read_io(addr, &v)) return false;

    _last_prot_raw = v;
    uint8_t stored = (uint8_t)((v >> 24) & 0x0F);
    if (stored == 0x00) {
        stored = psoc::chip_prot::OPEN;
    } else if (stored == 0x01) {
        stored = psoc::chip_prot::VIRGIN;
    }
    _last_chip_prot = stored;
    *out_prot = stored;
    return true;
}

// 需先 acquire() 成功。读取 16 个 row protection word 并返回其按位 OR。
uint32_t SwdProgrammer::read_row_protection() {
    uint32_t protection = 0;
    for (uint8_t i = 0; i < 16; i++) {
        uint32_t w = 0;
        if (!_read_io(psoc::reg::SFLASH_MACRO0 + (uint32_t)(i * 4), &w)) {
            _last_rowprot0 = 0xBADBAD00u;
            return 0xFFFFFFFFu;
        }
        if (i == 0) _last_rowprot0 = w;
        if (i == 1) _last_rowprot1 = w;
        protection |= w;
    }
    return protection;
}

// ---------------- Flash 编程流程（SROM） ----------------

void SwdProgrammer::_scan_erased_flash() {
    _erase_scan_complete = false;
    _erase_flash_sum = 0;
    _erase_flash_or = 0;
    _erase_first_nonzero_addr = 0xFFFFFFFFu;
    _erase_first_nonzero_value = 0;
    _erase_words_read = 0;

    for (uint32_t addr = 0; addr < psoc::flash::SIZE; addr += sizeof(uint32_t)) {
        uint32_t value = 0;
        if (!_read_io(addr, &value)) break;
        _erase_flash_sum += (value & 0xFFu) + ((value >> 8) & 0xFFu) +
                            ((value >> 16) & 0xFFu) + ((value >> 24) & 0xFFu);
        _erase_flash_or |= value;
        if (_erase_first_nonzero_addr == 0xFFFFFFFFu && value != 0) {
            _erase_first_nonzero_addr = addr;
            _erase_first_nonzero_value = value;
        }
        ++_erase_words_read;
    }
    _erase_scan_complete = _erase_words_read == psoc::flash::SIZE / sizeof(uint32_t);
}

bool SwdProgrammer::erase_all() {
    if (!_ready) return false;
    _erase_scan_complete = false;
    _erase_flash_sum = 0;
    _erase_flash_or = 0;
    _erase_first_nonzero_addr = 0xFFFFFFFFu;
    _erase_first_nonzero_value = 0;
    _erase_words_read = 0;
    if (!configure_flash_clock()) { _last_srom_status = 0xC10C0000u; return false; }  // 时钟配置失败哨兵

    // 先读当前芯片保护模式，决定擦除路径（对照 AN84858 EraseAllFlash）。
    // 目标始终为 OPEN；PROTECTED 状态下直接 ERASE_ALL 会被拒（NA_IN_DEAD_MODE）。
    uint8_t prot = 0xFF;
    if (!read_chip_protection(&prot)) return false;

    if (prot == psoc::chip_prot::PROTECTED) {
        // PROTECTED → OPEN：WRITE_PROTECTION 写 OPEN(0x01)、macro 0，同时擦全片，
        // 之后必须重新 acquire 使芯片进入 OPEN 编程态。参数走寄存器（非 SRAM）。
        uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_WRITE_PROTECTION) << 8) |
                          ((uint32_t)psoc::chip_prot::OPEN << 16) | (0x00u << 24);
        if (!_write_io(REG_CPUSS_SYSARG, params)) return false;
        if (!_srom_exec(SROM_CMD_WRITE_PROTECTION)) return false;
        if (!acquire()) return false;
        (void)set_program_indicator();  // 指示失败不得阻断已擦除器件的恢复编程
        return true;
    }

    // OPEN / VIRGIN：直接 ERASE_ALL。参数走 SRAM。
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_ERASE_ALL) << 8);
    if (!_write_io(SRAM_PARAMS_BASE + 0x00, params)) return false;
    if (!_write_io(REG_CPUSS_SYSARG, SRAM_PARAMS_BASE)) return false;
    const bool erased = _srom_exec(SROM_CMD_ERASE_ALL);
    if (!erased && _last_srom_status == 0xF000000Au) {
        const uint32_t erase_status = _last_srom_status;
        _scan_erased_flash();
        _last_srom_status = erase_status;
    }
    return erased;
}

bool SwdProgrammer::checksum_all(uint32_t* out_checksum) {
    if (!_ready || out_checksum == nullptr) return false;
    // Row ID = 0x8000 表示 "Checksum All"（用户+特权行）
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_CHECKSUM) << 8) |
                      ((0x0000u & 0x00FFu) << 16) | ((0x8000u & 0xFF00u) << 16);
    if (!_write_io(REG_CPUSS_SYSARG, params)) return false;
    if (!_srom_exec(SROM_CMD_CHECKSUM)) return false;
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
    // 诊断：回读 params1/params2/data0 三个字,确认写入 SRAM 持久(排查 latch 空的根因)。
    {
        uint32_t c0 = 0, c1 = 0, c2 = 0;
        uint32_t expect2 = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
        _read_io(SRAM_PARAMS_BASE + 0x00, &c0);
        _read_io(SRAM_PARAMS_BASE + 0x04, &c1);
        _read_io(SRAM_PARAMS_BASE + 0x08, &c2);
        if (c0 != params1)  { _last_srom_status = 0x5A5A0100u | (c0 & 0xFFFFu); return false; }
        if (c1 != params2)  { _last_srom_status = 0x5A5A0400u | (c1 & 0xFFFFu); return false; }
        if (c2 != expect2)  { _last_srom_status = 0x5A5A0800u | (c2 & 0xFFFFu); return false; }
    }
    if (!_write_io(REG_CPUSS_SYSARG, SRAM_PARAMS_BASE)) return false;
    return _srom_exec(SROM_CMD_LOAD_LATCH);
}

bool SwdProgrammer::program_row(uint16_t row_id, const uint8_t* data) {
    if (!_ready || data == nullptr) return false;
    uint8_t macro_id = (uint8_t)(row_id / ROWS_PER_MACRO);   // 128KiB / 256B = 512 行，单 macro
    // 1. 装载 latch（256 字节整行）
    if (!_srom_load_latch(macro_id, data, ROW_SIZE)) return false;
    // 2. 擦除已完成，按编程规范 Step 5 用 PROGRAM_ROW 把 latch 写入该行。
    uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_PROGRAM_ROW) << 8) |
                      (((uint32_t)row_id & 0x00FFu) << 16) |
                      (((uint32_t)row_id & 0xFF00u) << 16);
    if (!_write_io(SRAM_PARAMS_BASE + 0x00, params)) return false;
    if (!_write_io(REG_CPUSS_SYSARG, SRAM_PARAMS_BASE)) return false;
    return _srom_exec(SROM_CMD_PROGRAM_ROW);
}

bool SwdProgrammer::program_flash(const uint8_t* data, uint32_t len) {
    if (!_ready || data == nullptr || len == 0 || len > psoc::flash::SIZE) return false;
    if (!configure_flash_clock()) { _last_srom_status = 0xC10C0000u; return false; }
    // 镜像长度可能不是 ROW_SIZE(256) 的整数倍（生成器按 128 对齐）；向上取整为整行，
    // 末行不足部分用 0 补齐（flash 擦除态即 0，语义一致）。用 256 字节暂存缓冲避免越界读镜像。
    const uint16_t total_rows =
        static_cast<uint16_t>((len + ROW_SIZE - 1) / ROW_SIZE);
    if (total_rows > psoc::flash::ROW_COUNT) return false;
    uint8_t row_buf[ROW_SIZE];
    for (uint16_t row = 0; row < total_rows; row++) {
        const uint32_t offset = static_cast<uint32_t>(row) * ROW_SIZE;
        const uint32_t avail = (len - offset >= ROW_SIZE) ? ROW_SIZE : (len - offset);
        for (uint32_t i = 0; i < ROW_SIZE; i++) {
            row_buf[i] = (i < avail) ? data[offset + i] : 0u;
        }
        if (!program_row(row, row_buf)) {
            _last_fail_row = row;
            return false;
        }
    }
    return true;
}

bool SwdProgrammer::verify_flash(const uint8_t* data, uint32_t len) {
    if (!_ready || data == nullptr || len == 0 || len > psoc::flash::SIZE ||
        (len % 4) != 0) return false;
    // flash 直接映射到 CPU 地址空间自 0x00000000，按 4 字节读回比对
    for (uint32_t addr = 0; addr < len; addr += 4) {
        uint32_t expect = (uint32_t)data[addr] |
                          ((uint32_t)data[addr + 1] << 8) |
                          ((uint32_t)data[addr + 2] << 16) |
                          ((uint32_t)data[addr + 3] << 24);
        uint32_t w = 0;
        if (!_read_io(addr, &w)) {
            _last_fail_addr = addr;
            _last_verify_expect = expect;
            _last_verify_read = 0xBADBAD00u;
            return false;
        }
        if (w != expect) {
            _last_fail_addr = addr;
            _last_verify_expect = expect;
            _last_verify_read = w;
            return false;
        }
    }
    return true;
}
