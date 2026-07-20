#pragma once

#include <stdint.h>
#include "psoc_types.h"
#include "../../hal/pio/hal_pio.h"

/**
 * SwdProgrammer - 面向 PSoC4 (CY8C4147, CM0+) 的 SWD 编程器（非单例）
 *
 * PIO 硬件驱动的 SWD 传输层 + DP/AP 读写原语 + acquire(Step1A) + 读 IDCODE/Silicon ID
 * + SROM 的 erase/program/verify flash 流程。占用 HAL_PIO0（PIO0）。
 *
 * bit-bang 通过 RP2040 PIO 硬件外设实现（SWDCLK 走 side-set，SWDIO 双向 pindirs）。
 * 命名规范：类内部成员/函数以 _ 开头，对外接口不加 _。
 */
class SwdProgrammer {
public:
    // ACK 返回码（SWD 线协议 3-bit，LSB first）
    enum Ack : uint8_t {
        ACK_OK     = 0x1,  // 001
        ACK_WAIT   = 0x2,  // 010
        ACK_FAULT  = 0x4,  // 100
        ACK_PARITY = 0x7   // 本地扩展：读数据奇偶校验错
    };

    SwdProgrammer(uint8_t io_pin, uint8_t clk_pin, uint8_t rst_pin);

    // 初始化 PIO(HAL_PIO0) + SWD 引脚(SWDIO/SWDCLK) + RST(XRES) GPIO。
    bool init();

    // 状态机轮询（当前为空实现，保留接口给上层调度器）
    void update();

    // OpenOCD psoc4.c 逐字复刻：connect-under-reset + 停核（HALT），不走 test-mode。
    // 复用 _connect_and_halt()：XRES 复位 + boot 窗口内连 DP（CSW_PRIV）+ 写 DHCSR
    // C_HALT|C_DEBUGEN 停核 + 轮询 S_HALT。core 停住后 SROM 系统调用改由 _srom_exec 的
    // bkpt-run 机制发起（NMI 需 CPU 真正执行才被服务）。
    bool acquire();

    // acquire 后、erase 前依据目标器件 PDL 寄存器定义把 P1.6 设为 GPIO 强驱动高。
    // 失败仅表示指示灯不可恢复，不得阻止可恢复的 flash 流程。
    bool set_program_indicator();

    // 直接写时钟寄存器把 IMO 设 48MHz + HFCLK源=IMO + charge pump源=IMO(绕过 SROM Configure Clock)。
    // flash 擦/写前必须成功执行。返回 true=写入且回读校验通过。需先 acquire()。
    bool configure_flash_clock();

    // 读取 IMO trim 写入后的 CLK 寄存器与对应 SFLASH 源字，供 flash 编程诊断。需在 configure_flash_clock() 后调用。
    bool read_clk_trim_snapshot();

    // 烧录完成后复位目标进入正常运行态（仅脉冲 XRES，不进 Test Mode / 不 acquire）。
    void reset_target_run();

    // 释放 SWD 总线：SWDIO/SWDCLK 交还为高阻输入(脱离 PIO,不驱动),XRES 保持高(PSoC 正常运行)。
    // 烧录结束后调用,停止一切 SWD 活动,让 PSoC 独立运行其 flash 固件。
    void release_swd();

    // SET_IMO=48MHz（部分器件 flash 操作前必需；本器件实测被拒）。独立、非致命，供诊断。
    bool set_imo_48mhz();

    // ---------- 诊断信息（供 bring-up 经 CDC 上报） ----------
    uint32_t last_srom_status() const { return _last_srom_status; }  // 最近一次 SROM 返回状态码
    uint16_t last_fail_row() const { return _last_fail_row; }        // program_flash 首个失败行
    uint32_t last_fail_addr() const { return _last_fail_addr; }      // verify_flash 首个不符地址
    uint32_t last_verify_read() const { return _last_verify_read; }  // verify_flash 首个失败的读回值
    uint32_t last_verify_expect() const { return _last_verify_expect; }  // verify_flash 首个失败的期望值
    uint32_t last_acquire_status() const { return _last_acquire_status; }  // acquire 期 TEST_MODE 后 SYSARG
    uint32_t last_acquire_sysreq() const { return _last_acquire_sysreq; }  // acquire 期 TEST_MODE 后 SYSREQ（诊断 SROM 状态）
    uint32_t last_acquire_delay() const { return _last_acquire_delay; }    // acquire 命中窗口时的 TEST_MODE 写入延时(us)；0xFFFFFFFF=全扫失败
    uint32_t clk_trim1() const { return _clk_trim1; }
    uint32_t clk_trim3() const { return _clk_trim3; }
    uint32_t sfl_trim_word() const { return _sfl_trim_word; }
    uint32_t sfl_tctrim_word() const { return _sfl_tctrim_word; }
    bool clock_config_ok() const { return _clock_config_ok; }
    uint32_t clock_select() const { return _clock_select; }
    uint32_t clock_imo_select() const { return _clock_imo_select; }
    uint32_t clock_trim1() const { return _clock_trim1; }
    uint32_t clock_trim2() const { return _clock_trim2; }
    uint32_t clock_trim3() const { return _clock_trim3; }
    bool erase_scan_complete() const { return _erase_scan_complete; }
    uint32_t erase_flash_sum() const { return _erase_flash_sum; }
    uint32_t erase_flash_or() const { return _erase_flash_or; }
    uint32_t erase_first_nonzero_addr() const { return _erase_first_nonzero_addr; }
    uint32_t erase_first_nonzero_value() const { return _erase_first_nonzero_value; }
    uint32_t erase_words_read() const { return _erase_words_read; }

    // 单独读取调试口 IDCODE（需先 line reset）。ack 可选输出。
    uint32_t read_idcode(uint8_t* ack_out = nullptr);

    // Step 2：经 SROM GET_SILICON_ID 读取 4 字节硅 ID。
    // out_id 打包为 [31:24]=Hi [23:16]=Lo [15:8]=Rev [7:0]=Family。需先 acquire() 成功。
    // 同时刷新 _last_chip_prot（保护值 = CPUSS_SYSREQ[15:12]）。
    bool read_silicon_id(uint32_t* out_id);

    // 经 AHB 读取 SFLASH macro0 的芯片级保护模式（psoc::chip_prot::*）。需先 acquire()。
    bool read_chip_protection(uint8_t* out_prot);
    // 需先 acquire() 成功。
    uint32_t read_row_protection();
    uint8_t last_chip_prot() const { return _last_chip_prot; }
    uint32_t last_prot_raw() const { return _last_prot_raw; }
    uint32_t last_rowprot0() const { return _last_rowprot0; }
    uint32_t last_rowprot1() const { return _last_rowprot1; }

    // 最近一次读到的 IDCODE（用于诊断显示）
    uint32_t last_idcode() const { return _last_idcode; }

    // ---------- Flash 编程流程（SROM） ----------
    bool erase_all();
    bool checksum_all(uint32_t* out_checksum);
    bool program_row(uint16_t row_id, const uint8_t* data);
    bool program_flash(const uint8_t* data, uint32_t len);
    bool verify_flash(const uint8_t* data, uint32_t len);

    // flash 几何（集中定义于 psoc_types.h）
    static constexpr uint16_t ROW_SIZE = psoc::flash::ROW_SIZE;
    static constexpr uint16_t ROWS_PER_MACRO = psoc::flash::ROWS_PER_MACRO;

private:
    // ---------- PIO 传输层原语 ----------
    void _seq_out(uint32_t data, uint8_t nbits);
    uint32_t _seq_in(uint8_t nbits);
    void _turnaround(uint8_t nbits);
    void _line_reset();
    void _swd_connect();

    // ---------- DP/AP 访问 ----------
    uint8_t _swd_write(uint8_t apndp, uint8_t addr, uint32_t data);
    uint8_t _swd_read(uint8_t apndp, uint8_t addr, uint32_t* data);

    // ---------- 经 TAR/DRW 访问任意 CPU 地址（AHB-AP） ----------
    bool _write_io(uint32_t addr, uint32_t data);
    bool _read_io(uint32_t addr, uint32_t* data);

    // ---------- SROM ----------
    // OpenOCD psoc4.c 逐字复刻的 halt+bkpt-run 机制：core 须已 halt（acquire() 完成）。
    // 写 bkpt 到 SRAM → 设 SP/PC/xPSR → 写 CPUSS_SYSREQ=SYSREQ_BIT|HMASTER_BIT|cmd →
    // resume（清 C_HALT）让 CPU 执行 → SROM NMI 被服务 → 撞 bkpt 停 → 读 SYSARG 状态。
    bool _srom_exec(uint32_t cmd);
    bool _srom_load_latch(uint8_t macro_id, const uint8_t* data, uint16_t len);
    void _scan_erased_flash();

    // 写 Cortex-M0+ 核寄存器（经 DCRSR/DCRDR，PPB 域，需 CSW_PRIV）。
    // reg: SP=13, PC=15, xPSR=16（DCRSR REGSEL 编码）。写 DCRDR=val → 写 DCRSR=reg|REGWnR →
    // 轮询 DHCSR.S_REGRDY(bit16)。超时/IO 失败返回 false。
    bool _write_core_reg(uint8_t reg, uint32_t val);

    // 单次"硬复位 + 延时 window_delay_us 后写 TEST_MODE + 校验 bit31"尝试。
    // 返回 true 仅表示 TEST_MODE bit31 已置位（可能假阳性），需 _probe_programming_mode 甄别。
    bool _acquire_once(uint32_t window_delay_us);

    // OpenOCD 风格 debug-halt acquire：连接 SWD → DP 上电 → 配置 CSW → 写 DHCSR 停核 →
    // 轮询 S_HALT。对齐 OpenOCD psoc4_sysreq 要求 target 处于 TARGET_HALTED 的前提，
    // 不走 test-mode 抢窗。成功后 SROM 特权命令应可正常执行。
    bool _connect_and_halt();

    // 特权 SROM 探针：真正进入编程模式才成功，否则 0xF0000014。用于甄别 acquire 真/假。
    bool _probe_programming_mode();

    // TEST_MODE workaround（对照 OpenOCD psoc4.cfg）：不做硬复位窗口命中，而是
    // 先经 SWD 写 TEST_MODE=1，再软复位(SYSRESETREQ)。CPU 从 system ROM 重启时检查
    // TEST_MODE 标志停在 ROM（编程模式）。4100S Plus 不在 psoc4.cfg 的"复位清 TEST_MODE"黑名单，
    // 该法很可能有效（4045 老片正是用它烧成的）。返回 true 仅表示流程完成，需探针甄别。
    bool _acquire_test_mode_workaround();

    // 计算 32-bit 偶校验
    static uint8_t _parity32(uint32_t v);

    // ---------- 成员 ----------
    uint8_t _io_pin;
    uint8_t _clk_pin;
    uint8_t _rst_pin;

    HAL_PIO* _pio;
    uint8_t _sm;
    uint8_t _offset;
    bool _ready;
    uint32_t _last_idcode;

    // 诊断
    uint32_t _last_srom_status;
    uint16_t _last_fail_row;
    uint32_t _last_fail_addr;
    uint32_t _last_verify_read;
    uint32_t _last_verify_expect;
    uint8_t  _last_chip_prot;      // 最近一次读到的芯片保护模式
    uint32_t _last_prot_raw;
    uint32_t _last_rowprot0;
    uint32_t _last_rowprot1;
    uint32_t _last_acquire_status; // acquire 期进入 Test Mode 后的 SYSARG
    uint32_t _last_acquire_sysreq; // acquire 期进入 Test Mode 后的 SYSREQ（诊断 SROM 忙/闲）
    uint32_t _last_acquire_delay;  // acquire 命中窗口时的 TEST_MODE 写入延时(us)
    uint32_t _clk_trim1;
    uint32_t _clk_trim3;
    uint32_t _sfl_trim_word;
    uint32_t _sfl_tctrim_word;
    bool _clock_config_ok;
    uint32_t _clock_select;
    uint32_t _clock_imo_select;
    uint32_t _clock_trim1;
    uint32_t _clock_trim2;
    uint32_t _clock_trim3;
    bool _erase_scan_complete;
    uint32_t _erase_flash_sum;
    uint32_t _erase_flash_or;
    uint32_t _erase_first_nonzero_addr;
    uint32_t _erase_first_nonzero_value;
    uint32_t _erase_words_read;
};
