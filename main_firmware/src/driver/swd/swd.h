#pragma once

#include <stdint.h>
#include "../../hal/pio/hal_pio.h"

/**
 * SwdProgrammer - 面向 PSoC4 (CY8C4147, CM0+) 的 SWD 编程器（非单例）
 *
 * 里程碑2-任务2a：PIO 硬件驱动的 SWD 传输层 + DP/AP 读写原语 + acquire(Step1A)
 *                 + 读 IDCODE + 读 Silicon ID(Step2)。
 * SROM 的 erase/program/verify 流程见任务2b（后续里程碑）。
 *
 * bit-bang 通过 RP2040 PIO 硬件外设实现（SWDCLK 走 side-set，SWDIO 双向 pindirs）。
 * 命名规范：类内部成员/函数以 _ 开头，对外接口不加 _。
 */
class SwdProgrammer {
public:
    // ACK 返回码（SWD 线协议 3-bit，LSB first）
    enum Ack : uint8_t {
        ACK_OK    = 0x1,  // 001
        ACK_WAIT  = 0x2,  // 010
        ACK_FAULT = 0x4,  // 100
        ACK_PARITY = 0x7  // 本地扩展：读数据奇偶校验错
    };

    SwdProgrammer(uint8_t io_pin, uint8_t clk_pin, uint8_t rst_pin);

    // 初始化 PIO(HAL_PIO0) + SWD 引脚(SWDIO/SWDCLK) + RST(XRES) GPIO。
    // 返回 true 表示 PIO 传输层就绪。
    bool init();

    // 状态机轮询（当前为空实现，保留接口给上层调度器）
    void update();

    // Step 1A：硬复位后获取芯片进入 Test/Programming 模式。
    // 内部完成 XRES toggle + line reset + 读 IDCODE 校验 + 进入 Test Mode + 轮询 privileged。
    bool acquire();

    // 单独读取调试口 IDCODE（需先 line reset；acquire 前的连通性检查用）。
    // 返回 32-bit IDCODE（CY8C4147/CM0+ 期望 0x0BC11477）。ack 可选输出。
    uint32_t read_idcode(uint8_t* ack_out = nullptr);

    // Step 2：经 SROM GET_SILICON_ID 读取 4 字节硅 ID。
    // out_id 打包为 [31:24]=Hi [23:16]=Lo [15:8]=Rev [7:0]=Family。需先 acquire() 成功。
    bool read_silicon_id(uint32_t* out_id);

    // 最近一次 acquire() 读到的 IDCODE（用于诊断显示）
    uint32_t last_idcode() const { return _last_idcode; }

    // ---------- 任务2b：Flash 编程流程（SROM） ----------
    // Step 3：擦除全部用户 flash 与保护位（假定芯片处于 OPEN 模式）。需先 acquire()。
    bool erase_all();

    // Step 4/9：计算全 flash（用户+特权行）28-bit 校验和。需先 acquire()。
    bool checksum_all(uint32_t* out_checksum);

    // Step 5：编程单行（data 长度须为 ROW_SIZE=128 字节）。需先 acquire() 且已 erase。
    bool program_row(uint16_t row_id, const uint8_t* data);

    // Step 5：按行编程一段数据（len 须为 ROW_SIZE 的整数倍，从 flash 0 地址起）。
    bool program_flash(const uint8_t* data, uint32_t len);

    // Step 6：回读 flash 与 data 比对（len 须为 ROW_SIZE 整数倍）。
    bool verify_flash(const uint8_t* data, uint32_t len);

    // flash 几何（CY8C4147：128B/行，512 行，单 macro，共 64KB）
    static constexpr uint16_t ROW_SIZE = 128;
    static constexpr uint16_t ROWS_PER_MACRO = 512;

private:
    // ---------- PIO 传输层原语 ----------
    // 输出 nbits（LSB first），SWDIO 由 host 驱动
    void _seq_out(uint32_t data, uint8_t nbits);
    // 读入 nbits（LSB first），SWDIO 为输入；返回右对齐结果
    uint32_t _seq_in(uint8_t nbits);
    // 空转 nbits 个时钟，SWDIO 高阻（turnaround / idle）
    void _turnaround(uint8_t nbits);
    // SWD line reset：>=50 clk SWDIO=1，随后若干 idle low
    void _line_reset();
    // JTAG-to-SWD 切换序列 + line reset（连接目标前调用）
    void _swd_connect();

    // ---------- DP/AP 访问 ----------
    // 写 DAP 寄存器，返回 ACK
    uint8_t _swd_write(uint8_t apndp, uint8_t addr, uint32_t data);
    // 读 DAP 寄存器（含 even parity 校验），返回 ACK；data 输出 32-bit
    uint8_t _swd_read(uint8_t apndp, uint8_t addr, uint32_t* data);

    // ---------- 经 TAR/DRW 访问任意 CPU 地址（AHB-AP） ----------
    bool _write_io(uint32_t addr, uint32_t data);
    bool _read_io(uint32_t addr, uint32_t* data);

    // ---------- SROM ----------
    // 等待 SROM 系统调用完成并检查状态。timeout_ms 超时或状态非成功返回 false
    bool _poll_srom_status();
    // 把一行数据装入易失 latch（LOAD_LATCH），供 program_row 使用
    bool _srom_load_latch(uint8_t macro_id, const uint8_t* data, uint16_t len);

    // 计算 32-bit 偶校验
    static uint8_t _parity32(uint32_t v);

    // ---------- 成员 ----------
    uint8_t _io_pin;
    uint8_t _clk_pin;
    uint8_t _rst_pin;

    HAL_PIO* _pio;
    uint8_t _sm;
    uint8_t _offset;   // PIO 程序加载偏移（命令字里的绝对入口地址需加上它）
    bool _ready;
    uint32_t _last_idcode;
};
