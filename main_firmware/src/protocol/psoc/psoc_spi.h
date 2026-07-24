#pragma once

#include <stdint.h>
#include <stddef.h>
#include "psoc_types.h"

class HAL_PIO;

/**
 * PsocSpi - RP2040 <-> PSoC 的 PIO SPI 主机（非单例，由门面持有）
 *
 * 为什么用 PIO 而非硬件 SPI1：本板物理连线的 MOSI/MISO 与 RP2040 硬件 SPI1
 * 固定引脚(TX=GPIO27/RX=GPIO28)相反，硬件外设无法交换 TX/RX，只能用 PIO 任意指定引脚。
 *   MOSI(数据输出)=GPIO28, MISO(数据输入)=GPIO27, SCK=GPIO26, CS=GPIO29。
 *
 * 协议：SPI MODE0(CPOL0/CPHA0)、8bit、MSB first、全双工。占用 HAL_PIO1（PIO1）。
 * CS 用普通 GPIO 手动管理，一次事务内保持拉低。
 * 命名规范：类内部成员/函数以 _ 开头，对外接口不加 _。
 */
class PsocSpi {
public:
    PsocSpi(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin);

    // 初始化 PIO1 + SPI 程序 + 引脚方向 + CS GPIO
    bool init();

    // 全双工传输 len 字节（内部拉低/拉高 CS）。tx/rx 可为 nullptr。
    void transfer(const uint8_t* tx, uint8_t* rx, size_t len);

    // 发送 PING，保留独立链路健康检查。
    bool ping();

    // ★实时触控快路★：单次 7 字节事务读取 36 区 on/off 位图（流水线，最低延迟）。
    // 返回 true 且填充 out_mask(低36位有效)当且仅当响应为合法触控帧(magic+TOUCH)。
    bool read_touch(uint64_t* out_mask);

    // ---- Phase A：CSD 运行时指令（请求-应答 2 事务；非延迟关键，仅配置/调参用）----
    bool set_param(uint8_t ch, uint8_t param_id, uint32_t value);  // 写并回显校验
    bool get_param(uint8_t ch, uint8_t param_id, uint32_t* out_value);
    bool get_raw(uint8_t ch, uint16_t* out_raw);                   // 指定通道实时 CSD 计数
    bool get_stats(uint32_t* out_scan_count);                      // 读全局扫描计数(自由递增)
    bool measure_cp();                                             // 发送命令并确认 PSoC SPI ACK；测量本身在 PSoC 主循环异步执行
    bool get_cp(uint8_t ch, uint32_t* out_cp);                     // 测量中=0，成功=fF，失败/未测量=0xFFFFFF
    bool apply();                                                   // 应用硬件参数(重扫/重校准)
    bool calibrate();
    bool baseline_reset();
    // 频率自适应下探: 触发后轮询 busy 至完成(逐档升分频重校准, 数秒), 再读结果。
    // out_result: 0=进行中/未知 1=成功 2=失败(超硬件能力); out_div: 成功时找到的统一 snsClk 分频。
    bool auto_tune(uint8_t* out_result, uint16_t* out_div);
    bool set_mode(uint8_t mode);                                   // 0=自动校准/标准完整处理，1=半自动手动

    // ---- JIT 算法 blob 下发（分页事务；仅启动/更新用，非延迟关键）----
    bool algo_begin(uint16_t len);                                 // 复位暂存 + 记录期望 len(<=1024)
    bool algo_page(uint8_t page, const uint8_t four[4]);           // 写第 page 页(4 字节)
    bool algo_end(uint16_t crc16, bool* out_ok, uint16_t* out_len);// 触发 commit；回读 ok/len 回显
    bool algo_info(bool* out_valid, uint16_t* out_len);            // 读 PSoC 端算法 valid/len
    // 完整下发: begin→逐页→end→轮询 info 直到 valid 且 len 一致(内部含 commit 等待)。
    bool upload_algo(const uint8_t* data, uint16_t len, uint16_t crc16);
    bool set_algo_rom(uint8_t ch, uint16_t rom);                   // 设每通道 16 位只读 ROM(回显校验)
    bool get_algo_rom(uint8_t ch, uint16_t* out_rom);              // 读每通道 16 位 ROM
    // 算法运行时追踪/可调变量(ABI cfg[8]/report[4]/out_active, 见 psoc_algo_abi.h)
    bool algo_get_trace(uint8_t ch, uint8_t idx, uint8_t* out_active, uint16_t* out_report);
    bool algo_set_cfg(uint8_t idx, uint8_t val);                   // 设共享 cfg[idx](回显校验)
    bool algo_get_cfg(uint8_t idx, uint8_t* out_val);
    bool set_global(uint8_t gparam_id, uint32_t value);            // 写全局 CSD 配置(仅影子, 不重初始化)
    bool get_global(uint8_t gparam_id, uint32_t* out_value);       // 读全局 CSD 配置
    bool global_commit();                                          // 全局项设完后触发一次完整重初始化



    // 兼容旧 0.4.0：复位后启动灯态确定为高，再发显式点灯命令；失败不影响后续烧录。
    bool indicator_on();

    // 流水读取一份由 PSoC BEGIN 锁存的完整不可变 CapSense 快照（阻塞，~10ms；调试/一次性用）。
    bool read_snapshot(psoc::SensorSnapshot* snapshot);

    // ★Phase C 全通道 raw 慢路（分块）★：每次调用只读 max_pages 页，与触控快路交织，避免阻塞。
    // 状态机自动 BEGIN→逐页→完成。完成时把整份快照写入 *out 并返回 true（该次为最后一块）；
    // 未完成返回 false。中途出错自动回到空闲，下次重新开始。
    bool snapshot_pump(uint8_t max_pages, psoc::SensorSnapshot* out);

    bool ready() const { return _ready; }

private:
    uint8_t _xfer_byte(uint8_t out);
    // 指令事务：发送 [magic,cmd,b2,b3,val24]，隔 RESPONSE_DELAY 读回 7 字节响应到 resp[7]。
    bool _cmd_txn(uint8_t cmd, uint8_t b2, uint8_t b3, uint32_t val24, uint8_t resp[7]);
    psoc::Frame _make_request(psoc::Cmd command);
    static bool _response_matches(const psoc::Frame& response, psoc::Cmd command, uint8_t sequence);
    static uint16_t _read_u16(const uint8_t* bytes);
    // 轮询 GET_STATS 的 busy 字节(resp[2])至 PSoC 主循环真正完成重操作(busy 1→0)或超时。
    // 用于 calibrate/apply/baseline_reset 的真实完成反馈, 替代原固定 sleep 盲等。返回 true=真实完成。
    bool _wait_op_done(uint32_t timeout_ms);

    uint8_t _sck_pin;
    uint8_t _mosi_pin;
    uint8_t _miso_pin;
    uint8_t _cs_pin;

    HAL_PIO* _pio;
    uint8_t _sm;
    uint8_t _offset;
    bool _ready;
    uint8_t _seq;

    // 分块快照读取状态机（snapshot_pump 用）
    bool _snap_active = false;                     // 是否正在读一份快照
    uint16_t _snap_page = 0;                       // 下一个待请求的页号(1..PAGE_COUNT)
    uint8_t _snap_expected_seq = 0;                // 期望的流水应答序号
    uint16_t _snap_generation = 0;
    bool _snap_valid = false;
    uint8_t _snap_packed[psoc::SNAPSHOT_SIZE];     // 累积的原始快照字节
};
