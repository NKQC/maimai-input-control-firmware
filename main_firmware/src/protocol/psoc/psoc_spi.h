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
    bool measure_cp();                                             // 触发逐电极寄生电容 BIST 测量(异步)
    bool get_cp(uint8_t ch, uint32_t* out_cp);                     // 读指定通道最近 Cp 值(fF)
    bool apply();                                                   // 应用硬件参数(重扫/重校准)
    bool set_mode(uint8_t mode);                                   // 0=全自动 SmartSense, 1=半自动/手动



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
