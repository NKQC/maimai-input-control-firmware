#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================
// PSoC 协议层公共强类型定义
// 集中原先散落在 driver/swd 与 service/sensor_link 中的魔数：
//   - SPI 传感器帧 / 命令
//   - SWD 目标器件标识
//   - CPU 寄存器地址 / SROM 系统调用常量 / opcode
//   - Flash 几何
// 用强类型让编译器理解协议，禁止在实现里手写裸魔数。
// ============================================================

namespace psoc {

// ---------------- SPI 传感器帧 ----------------
// 帧魔数，用于校验帧头合法性
static constexpr uint8_t FRAME_MAGIC = 0xA5;
static constexpr size_t FRAME_PAYLOAD_SIZE = 4;
static constexpr size_t SENSOR_CHANNEL_COUNT = 36;
static constexpr size_t SENSOR_BYTES_PER_CHANNEL = 7;
static constexpr size_t SNAPSHOT_SIZE = SENSOR_CHANNEL_COUNT * SENSOR_BYTES_PER_CHANNEL;
static constexpr size_t SNAPSHOT_PAGE_COUNT = SNAPSHOT_SIZE / FRAME_PAYLOAD_SIZE;
static constexpr uint8_t SENSOR_TOUCH_STATUS_MASK = 0x01;

enum class Cmd : uint8_t {
    PING = 0x01,
    PONG = 0x02,
    TOUCH = 0x03,   // 实时触控态：7字节帧 [magic,TOUCH,mask0..mask4]
    SNAPSHOT_BEGIN = 0x10,
    // Phase A CSD 运行时指令（帧 [magic,cmd,ch,param_id,val24]）
    SET_PARAM = 0x30,
    GET_PARAM = 0x31,
    SET_MODE  = 0x32,
    APPLY     = 0x33,
    GET_RAW   = 0x34,
    GET_STATS = 0x35,
    MEASURE_CP = 0x36,   // 发送后确认 SPI ACK；PSoC 主循环异步执行实际测量
    GET_CP     = 0x37,   // 测量中=0，成功=fF，失败/未测量=0xFFFFFF
    SET_GLOBAL = 0x38,   // 全局 CSD 配置写(RAM 影子, 不重初始化)
    GET_GLOBAL = 0x39,   // 全局 CSD 配置读：响应 [magic,GET_GLOBAL,gparam_id,0,val24]
    GLOBAL_COMMIT = 0x3A,
    CALIBRATE = 0x3B,      // 真正的 IDAC 重校准 + 基线复位(主循环执行, 耗时)
    BASELINE_RESET = 0x3C, // 仅重置全部通道基线(主循环执行)// 全部全局项设完后触发一次完整重初始化(合并, 防反复重校准漂移)
    AUTO_TUNE = 0x3D,      // 频率自适应下探(主循环逐档升 snsClk 分频重校准, 耗时数秒)
    GET_AUTO_TUNE = 0x3E,  // 读自适应结果: [magic,GET_AUTO_TUNE,result(0进行中/1成功/2失败),0,div24]
    // JIT 可加载算法引擎：分页下发 blob 到 PSoC 的 1KB 可执行槽（ABI v1，见 jit-algo-engine.md）
    ALGO_BEGIN = 0x40,   // [magic,ALGO_BEGIN,len_lo,len_hi,0,0,0] 复位暂存写指针+记录期望 len
    ALGO_PAGE  = 0x41,   // [magic,ALGO_PAGE,page,d0,d1,d2,d3] 每页 4 字节写 staging[page*4..+4]
    ALGO_END   = 0x42,   // [magic,ALGO_END,crc_lo,crc_hi,0,0,0] 触发主循环 commit(CRC16 校验+拷入槽)
    ALGO_INFO  = 0x43,   // 响应 [magic,ALGO_INFO,valid,0,len_lo,len_hi,0]
    ALGO_SET_ROM = 0x44, // [magic,ALGO_SET_ROM,ch,rom_lo,rom_hi,0,0] 设每通道 16 位只读 ROM
    ALGO_GET_ROM = 0x45, // [magic,ALGO_GET_ROM,ch,0,0,0,0] → 响应 [.. ,ch,0,rom_lo,rom_hi,0]
    ALGO_GET_TRACE = 0x46, // [magic,GET_TRACE,ch,idx,0,0,0] → 响应 [..,ch,out_active,report[idx]_lo,report[idx]_hi,0]
    ALGO_SET_CFG = 0x47,   // [magic,SET_CFG,idx,val,0,0,0] 设共享 cfg[idx] → 响应回显 [..,idx,0,cfg[idx],0,0]
    ALGO_GET_CFG = 0x48,   // [magic,GET_CFG,idx,0,0,0,0] → 响应 [..,idx,0,cfg[idx],0,0]
    SNAPSHOT_INFO = 0x11,
    SNAPSHOT_PAGE = 0x12,
    SNAPSHOT_DATA = 0x13,
    INDICATOR_ON = 0x20,
};

// SCB 从机 FIFO 安全的定长 7-byte 帧。响应在下一 SPI 事务返回。
struct Frame {
    uint8_t magic = FRAME_MAGIC;
    uint8_t cmd = 0;
    uint8_t seq = 0;
    uint8_t payload[FRAME_PAYLOAD_SIZE] = {};

    void clear() {
        magic = FRAME_MAGIC;
        cmd = 0;
        seq = 0;
        for (auto& byte : payload) byte = 0;
    }
};
static_assert(sizeof(Frame) == 3 + FRAME_PAYLOAD_SIZE, "psoc::Frame must be 7 bytes");
static_assert((SNAPSHOT_SIZE % FRAME_PAYLOAD_SIZE) == 0, "snapshot must use complete pages");

struct SensorSample {
    uint16_t raw = 0;
    uint16_t baseline = 0;
    int16_t diff = 0;
    uint8_t status = 0;

    void clear() {
        raw = 0;
        baseline = 0;
        diff = 0;
        status = 0;
    }
};

// ---------------- 频率自适应(AUTO_TUNE)阶段性进度 ----------------
// 长自适应(最坏 ~20s)期间 PSoC 逐步更新 phase/step/试探分频, RP2040 core1 在阻塞等待中降频轮询
// GET_AUTO_TUNE 取回并经 seqlock 发布, core0 再由 TxScheduler 推送给上位机(避免上位机干等)。
// 单一结构同时用于 SPI 层进度回调与门面发布态, 不再散装多个平行变量。
struct AutoTuneProgress {
    uint32_t req = 0;        // 请求代号(core0 每次启动自增, core1 回显): 用于区分本轮与上一轮的 done
    uint8_t  state = 0;      // 0=空闲 1=进行中 2=完成
    uint8_t  phase = 0;      // 0=已受理 1=粗定位 2=细搜临界 3=落档/回退 4=完成
    uint8_t  step = 0;       // 当前阶段内步序(1 起)
    uint8_t  ch = 0xFF;      // 目标通道(0..35 单通道 / 0xFF 全通道)
    uint8_t  result = 0;     // 0=进行中 1=成功 2=失败/超时
    uint16_t cur_div = 0;    // 进行中: 当前试探的 snsClk 分频
    uint16_t div = 0;        // 完成时: 最终写入的分频(失败为 0)

    void clear() {
        req = 0; state = 0; phase = 0; step = 0; ch = 0xFF; result = 0; cur_div = 0; div = 0;
    }
};

// SPI 层在阻塞等待中回吐进度用的回调(ctx 由调用方透传, 避免 SPI 层反向依赖门面类型)。
using AutoTuneProgressFn = void (*)(void* ctx, const AutoTuneProgress& progress);

struct SensorSnapshot {
    uint16_t generation = 0;
    bool valid = false;
    SensorSample channels[SENSOR_CHANNEL_COUNT] = {};

    void clear() {
        generation = 0;
        valid = false;
        for (auto& channel : channels) channel.clear();
    }

    uint64_t active_mask() const {
        uint64_t mask = 0;
        if (!valid) return mask;
        for (size_t channel = 0; channel < SENSOR_CHANNEL_COUNT; channel++) {
            if ((channels[channel].status & SENSOR_TOUCH_STATUS_MASK) != 0) {
                mask |= (uint64_t{1} << channel);
            }
        }
        return mask;
    }
};

// ---------------- SWD / 目标器件标识 ----------------
static constexpr uint32_t SWD_IDCODE_CM0P = 0x0BC11477;
static constexpr uint32_t SILICON_ID_CY8C4147AZI_S455 = 0x257011B5;
static constexpr uint32_t SILICON_ID_DEVICE_MASK = 0xFFFF00FFu;  // revision byte is intentionally ignored
static constexpr uint8_t  SILICON_FAMILY_4100S_PLUS = 0xB5;

inline bool is_cy8c4147azi_s455(uint32_t silicon_id) {
    return (silicon_id & SILICON_ID_DEVICE_MASK) ==
           (SILICON_ID_CY8C4147AZI_S455 & SILICON_ID_DEVICE_MASK);
}

// ---------------- CPU 寄存器地址（PSoC4100S Plus，规格表 1-1） ----------------
namespace reg {
    static constexpr uint32_t TEST_MODE        = 0x40030014;
    static constexpr uint32_t SFLASH_MACRO0 = 0x0FFFF000;  // SFLASH macro0 基址(芯片保护字节所在)
    static constexpr uint32_t CPUSS_SYSREQ     = 0x40100004;
    static constexpr uint32_t CPUSS_SYSARG     = 0x40100008;
    static constexpr uint32_t SRAM_PARAMS_BASE = 0x20001000;  // SROM 参数/数据 SRAM 基址(避开 SROM 自用低 SRAM 暂存区)
    // ---- flash 编程时钟配置(直接寄存器,绕过 SROM Configure Clock) ----
    static constexpr uint32_t CLK_IMO_SELECT = 0x40030F08;  // FREQ[2:0]: 0=24M..6=48M
    static constexpr uint32_t CLK_IMO_TRIM1  = 0x40030F0C;  // OFFSET[7:0] 粗调
    static constexpr uint32_t CLK_IMO_TRIM2  = 0x40030F10;  // FSOFFSET 细调(清0)
    static constexpr uint32_t CLK_IMO_TRIM3  = 0x40030F18;  // TCTRIM[6:5]+STEPSIZE[4:0]
    static constexpr uint32_t CLK_SELECT     = 0x40030028;  // PUMP_SEL[5:4], HFCLK_SEL[1:0]
    // SFLASH 出厂 trim(48MHz=LT24 档);1字节宽,按字读后取字节车道
    static constexpr uint32_t SFLASH_IMO_TRIM_LT24_WORD   = 0x0FFFF37C;  // 目标字节在车道1: (w>>8)&0xFF
    static constexpr uint32_t SFLASH_IMO_TCTRIM_LT24_WORD = 0x0FFFF364;  // 目标字节在车道0: w&0xFF
    // ---- 白色状态灯 P1.6（仅用于 acquire 后、erase 前恢复硬件指示）----
    // 权威来源：CY8C4147AZI-S455 device header + cyip_gpio_v2.h + generated cycfg_pins。
    static constexpr uint32_t GPIO_PRT1_DR       = 0x40040100;
    static constexpr uint32_t GPIO_PRT1_PC       = 0x40040108;
    static constexpr uint32_t GPIO_PRT1_PC2      = 0x40040118;
    static constexpr uint32_t GPIO_PRT1_DR_SET   = 0x40040140;
    static constexpr uint32_t HSIOM_PRT1_SEL     = 0x40020100;
    static constexpr uint32_t STATUS_LED_MASK    = 1u << 6;
    static constexpr uint32_t STATUS_LED_PC_MASK = 0x7u << 18;
    static constexpr uint32_t STATUS_LED_PC_VALUE = 0x6u << 18;  // CY_GPIO_DM_STRONG[_IN_OFF] output mode
    static constexpr uint32_t STATUS_LED_HSIOM_MASK = 0xFu << 24;
}

// ---------------- SROM 系统调用 ----------------
namespace srom {
    static constexpr uint32_t KEY1 = 0xB6;
    static constexpr uint32_t KEY2 = 0xD3;
    static constexpr uint32_t SYSREQ_BIT       = 0x80000000;
    // HMASTER 位：OpenOCD psoc4.c 逐字复刻的 halt+bkpt-run 机制必需，已恢复。
    // SROM 靠 NMI 服务，CPU 必须真正在执行（halt 态下 resume 一段 bkpt 算法）才会被服务；
    // 该机制下每次 SYSREQ 请求都须带 SYSREQ_BIT|HMASTER_BIT|cmd（不区分命令类型）。
    static constexpr uint32_t HMASTER_BIT      = 0x40000000;
    static constexpr uint32_t PRIVILEGED_BIT   = 0x10000000;
    static constexpr uint32_t STATUS_SUCCEEDED = 0xA0000000;
    // opcode
    static constexpr uint32_t CMD_GET_SILICON_ID = 0x00;
    static constexpr uint32_t CMD_LOAD_LATCH     = 0x04;
    static constexpr uint32_t CMD_WRITE_ROW      = 0x05;  // 擦+写一行(自动擦除);免单独 ERASE_ALL(后者仅 programming-mode 可用)
    static constexpr uint32_t CMD_PROGRAM_ROW    = 0x06;
    static constexpr uint32_t CMD_ERASE_ALL      = 0x0A;
    static constexpr uint32_t CMD_CHECKSUM       = 0x0B;
    static constexpr uint32_t CMD_WRITE_PROTECTION = 0x0D;  // 写 flash/chip 保护（含 PROTECTED→OPEN 转换）
    static constexpr uint32_t CMD_SET_IMO_48MHZ  = 0x15;  // 4100S Plus 需先设 IMO=48MHz 才能擦/写 flash
}

// ---------------- 芯片级保护模式（规格表：CHIP_PROT_*） ----------------
// GET_SILICON_ID 返回时，保护值位于 CPUSS_SYSREQ[15:12]。
// 非 OPEN 状态下直接 ERASE_ALL/CLK_CONFIG 会被 SROM 拒绝（0xF0000014 NA_IN_DEAD_MODE）。
namespace chip_prot {
    static constexpr uint8_t VIRGIN    = 0x00;  // 仅 Infineon；设为此值使芯片不可用
    static constexpr uint8_t OPEN      = 0x01;  // 出厂默认，flash 无保护
    static constexpr uint8_t PROTECTED = 0x02;  // 需先 WRITE_PROTECTION→OPEN（同时擦全片）再编程
    static constexpr uint8_t KILL      = 0x04;  // 不可逆，SWD 锁死
}

// ---------------- Flash 几何（CY8C4147AZI-S455：128KiB，256B programming row） ----------------
// ★行大小 = 256 字节★（规格 Table 1-1：需 SET_IMO_48MHz 的器件行大小 256；实测按 128 处理时
// row0 只填半行、后续行以 256 步长错位）。128KiB / 256 = 512 行，单 macro。
namespace flash {
    static constexpr uint32_t SIZE = 128u * 1024u;
    static constexpr uint16_t ROW_SIZE = 256;
    static constexpr uint16_t ROWS_PER_MACRO = 512;
    static constexpr uint16_t ROW_COUNT = static_cast<uint16_t>(SIZE / ROW_SIZE);
}

}  // namespace psoc
