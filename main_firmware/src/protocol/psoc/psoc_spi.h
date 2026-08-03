#pragma once

#include <stdint.h>
#include <stddef.h>
#include "psoc_types.h"
#include "../../hal/dma/hal_dma.h"

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
 *
 * ★链路不堵塞★：PIO 配 autopull/autopush(阈值 8bit)，收发各挂一条 DMA 通道(mem↔PIO FIFO，
 * 由 SM 的 DREQ 节流)。transfer() 只做"配好地址与计数 → 启动 → 轮询 DMA 完成标志"，CPU 不触碰
 * 任何单个字节，也不用固定 sleep 估算传输时长；事务间仅保留 PSoC ISR 装帧所需的最小窗口。
 * 命名规范：类内部成员/函数以 _ 开头，对外接口不加 _。
 */
class PsocSpi {
public:
    PsocSpi(uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin);

    // 初始化 PIO1 + SPI 程序 + 引脚方向 + CS GPIO
    bool init();

    // 全双工传输 len 字节（内部拉低/拉高 CS，DMA 搬运，等 DMA 完成标志）。tx/rx 可为 nullptr。
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
    // 读全局扫描计数(自由递增)。out_busy 非空时同时回吐 PSoC 的"处理中"标志(响应 byte[2]):
    // 主循环正在跑 APPLY/CALIBRATE/GLOBAL_COMMIT/AUTO_TUNE 等重操作时为非 0。
    // ★调用方据此区分"主循环卡死"与"主循环正忙"★——重操作期间 scan_count 天然不推进, 不能判为卡死。
    bool get_stats(uint32_t* out_scan_count, uint8_t* out_busy = nullptr);
    bool measure_cp();                                             // 发送命令并等待 BIST 后固件恢复正常 CSD 扫描
    bool get_cp(uint8_t ch, uint32_t* out_cp);                     // 测量中=0，成功=fF，失败/未测量=0xFFFFFF
    bool apply();                                                   // 应用硬件参数(重扫/重校准)
    // 真正的 IDAC 重校准 + 基线复位。ch(帧字节2): 0..35=只校准该 widget 并只初始化该 widget 基线,
    // 0xFF=全 36 通道。单通道用时约为全通道的 1/36, 故超时窗按目标范围分档给。
    bool calibrate(uint8_t ch = 0xFFu);
    // 基线复位。ch(帧字节2): 0..35=只初始化该 widget 基线, 0xFF=全通道。
    bool baseline_reset(uint8_t ch = 0xFFu);
    // 频率自适应下探: 触发后轮询 busy 至完成(逐档升分频重校准, 数秒), 再读结果。
    // ch: 0..35=仅该通道下探(其余通道分频不动), 0xFF=全 36 通道统一(旧行为)。
    // pref: 灵敏度档位 1..7(帧字节3), 越高=在临界频率基础上往低频多让分频(更灵敏)。
    // out_result: 0=进行中/未知 1=成功 2=失败(超硬件能力); out_div: 最终写入的 snsClk 分频。
    // on_progress != nullptr 时, 在阻塞等待中每 ~100ms 顺带读一次 GET_AUTO_TUNE 的阶段进度并回吐
    // (不改 busy 判定与超时行为, 不提高 busy 轮询频率, 避免抢占 PSoC 的 SPI 带宽影响校准)。
    // tag: 本轮请求标签(psoc::autotune_tag_of(上位机请求 seq); 0=不带标签)。PSoC 原样回显 ⇒
    // 读到的 result/div 若带着别的标签, 那就是上一轮的残留, 一律判失败而不是冒充本轮成功。
    bool auto_tune(uint8_t ch, uint8_t pref, uint8_t tag, uint8_t* out_result, uint16_t* out_div,
                   psoc::AutoTuneProgressFn on_progress = nullptr, void* progress_ctx = nullptr);
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
    // 事务超时(DMA 未在上限内完成)后复位 SM/FIFO/移位计数，防半个字节让后续帧永久错位。
    void _recover();
    // 指令事务：发送 [magic,cmd,b2,b3,val24]，隔 PSoC ISR 装帧窗口读回 7 字节响应到 resp[7]。
    bool _cmd_txn(uint8_t cmd, uint8_t b2, uint8_t b3, uint32_t val24, uint8_t resp[7]);
    psoc::Frame _make_request(psoc::Cmd command);
    static bool _response_matches(const psoc::Frame& response, psoc::Cmd command, uint8_t sequence);
    static uint16_t _read_u16(const uint8_t* bytes);
    // 轮询 GET_STATS 的 busy 字节(resp[2])至 PSoC 主循环真正完成重操作(busy 1→0)或超时。
    // 用于 calibrate/apply/baseline_reset 的真实完成反馈, 替代原固定 sleep 盲等。返回 true=真实完成。
    // on_progress != nullptr 时额外每 PROGRESS_POLL_MS 读一次 AUTO_TUNE 进度回吐(busy 语义不变)。
    // progress_tag: 本轮请求标签(0=不校验)。回吐进度前先比对 PSoC 回显的标签, 不匹配即不回吐 ——
    // 否则上一轮的阶段进度会被当成本轮的显示出去。
    bool _wait_op_done(uint32_t timeout_ms, psoc::AutoTuneProgressFn on_progress = nullptr,
                       void* progress_ctx = nullptr, uint8_t progress_tag = 0u);
    // 把 PSoC 的默认响应换回实时触控帧(流水线收尾)，见 psoc_spi.cpp 实现处说明。
    void _restore_touch_response();
    // 直发一条"重操作"命令(APPLY/CALIBRATE/BASELINE_RESET/AUTO_TUNE)并**确认 PSoC 真的收到**。
    // 返回 false = 未受理(调用方直接失败, 不要进 _wait_op_done)。见实现处的残帧/假成功说明。
    // b4 = 帧字节4(val24 低字节): 目前只有 AUTO_TUNE 用它带请求标签, 其余重操作传 0。
    bool _send_heavy(uint8_t cmd, uint8_t b2, uint8_t b3, uint8_t b4 = 0u);
    static constexpr uint32_t PROGRESS_POLL_MS = 100;   // 进度读取降频周期(busy 轮询仍为 3ms)

    uint8_t _sck_pin;
    uint8_t _mosi_pin;
    uint8_t _miso_pin;
    uint8_t _cs_pin;

    HAL_PIO* _pio;
    HAL_DMA_Duplex _dma;   // 收发两条 DMA 通道（mem ↔ 本 SM 的 TX/RX FIFO）
    uint8_t _sm;
    uint8_t _offset;
    bool _ready;
    uint8_t _seq;

    // 一份快照的完成期限: 超期即丢弃进度重新 BEGIN。快照跨多次 pump 续读且中途失败原地重试,
    // 若某种应答流水失步让某页永远读不出来, 就会无限重试同一页 → 快照永不发布 → 上位机看到
    // raw/baseline/diff 永久冻结(误判为"采样停了", 实测只能复位设备)。30Hz 遥测下正常几十 ms 完成。
    static constexpr uint32_t SNAP_COMPLETE_TIMEOUT_US = 2000000;

    // 分块快照读取状态机（snapshot_pump 用）
    bool _snap_active = false;                     // 是否正在读一份快照
    uint32_t _snap_start_us = 0;                   // 本份快照的起始时刻(完成期限用)
    uint16_t _snap_page = 0;                       // 下一个待请求的页号(1..PAGE_COUNT)
    uint8_t _snap_expected_seq = 0;                // 期望的流水应答序号
    uint16_t _snap_generation = 0;
    bool _snap_valid = false;
    uint8_t _snap_packed[psoc::SNAPSHOT_SIZE];     // 累积的原始快照字节
};
