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

// ---------------- 传感器采样几何 ----------------
// ★帧格式/命令码不在这里★ 它们的唯一真相源是 psoc_link_abi.h(两端共用)。此前 RP 侧在这里
// 另写了一份(magic/Cmd/Frame), 与 PSoC 的 #define 和上位机常量三处并存 —— 漏改任何一处都
// 表现为"上传成功却跑旧代码"。LINK v2 起本文件只留与线格式无关的采样/SWD 类型。
static constexpr size_t SENSOR_CHANNEL_COUNT = 36;
static constexpr size_t SENSOR_BYTES_PER_CHANNEL = 7;
static constexpr size_t SNAPSHOT_SIZE = SENSOR_CHANNEL_COUNT * SENSOR_BYTES_PER_CHANNEL;
static constexpr uint8_t SENSOR_TOUCH_STATUS_MASK = 0x01;

// JIT 算法 ABI 的 report 槽数(见 psoc_algo_abi.h 的 algo_io_t::report[4])。
static constexpr size_t ALGO_REPORT_SLOTS = 4;

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
    uint8_t  tag = 0;        // 本轮请求标签(6 bit, PSoC 回显; 0=未标记/旧 PSoC 固件)
    uint16_t cur_div = 0;    // 进行中: 当前试探的 snsClk 分频
    uint16_t div = 0;        // 完成时: 最终写入的分频(失败为 0)

    void clear() {
        req = 0; state = 0; phase = 0; step = 0; ch = 0xFF; result = 0; tag = 0; cur_div = 0; div = 0;
    }
};

// ---------------- AUTO_TUNE 请求标签(★仅 host 侧进度标签★) ----------------
// LINK v2 起它**不再下到线上**: 链路 tag 已经保证"这份响应属于这次请求", 陈旧结果不可能冒充
// 本轮成功(见 psoc_link_abi.h 的三条不变式)。这里保留它只是给上位机的进度推送带一个请求归属键,
// 使界面能区分本轮进度与上一轮残留。0 保留给"未标记", 故 seq 低 6 位为 0 时映射到 0x3F。
static constexpr uint8_t AUTOTUNE_TAG_MASK = 0x3Fu;
static constexpr uint8_t AUTOTUNE_RESULT_MASK = 0x03u;
static inline uint8_t autotune_tag_of(uint8_t host_seq) {
    const uint8_t tag = (uint8_t)(host_seq & AUTOTUNE_TAG_MASK);
    return (tag != 0u) ? tag : AUTOTUNE_TAG_MASK;
}

struct SensorSnapshot {
    uint16_t generation = 0;
    bool valid = false;
    SensorSample channels[SENSOR_CHANNEL_COUNT] = {};
    // ★算法运行值与采样同批发布★
    // report[0..3] 与 out_active 是 JIT 算法的运行输出, PSoC 只提供 ALGO_GET_TRACE 逐项读取,
    // 没有并入 252B 快照。放在这里由 core1 顺带取回并与快照同批经 seqlock 发布, 使上位机可以
    // 随遥测帧一次拿走 —— 取代原先"core0 阻塞读类命令 + 单响应槽"那条抢不到窗口的老路。
    // algo_channel = 这组值属于哪个通道(独占流即 Focus 通道); 0xFF = 尚无有效算法运行值。
    uint8_t algo_channel = 0xFFu;
    uint8_t algo_active = 0;
    uint16_t algo_report[ALGO_REPORT_SLOTS] = {};

    void clear() {
        generation = 0;
        valid = false;
        for (auto& channel : channels) channel.clear();
        algo_channel = 0xFFu;
        algo_active = 0;
        for (auto& value : algo_report) value = 0;
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
