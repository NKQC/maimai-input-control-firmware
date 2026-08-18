/******************************************************************************
 * File Name:   psoc_algo_abi.h
 *
 * Description: ABI v2 shared contract between the PSoC firmware (main.c) and
 *              the JIT-loadable touch algorithm blob (4KB Thumb machine code,
 *              position independent, entry at algo_slot|1). Field offsets are
 *              fixed and must never change without bumping the ABI version;
 *              both the engine (main.c) and every algorithm blob source
 *              (e.g. psoc_algo_default.c) include this single header.
 *
 *              ★v1 → v2 是纯追加★ struct 总长仍是 128 字节, 既有字段偏移一个都没动;
 *              新增的 heap/heap_size/heap_used/cfg_ch 全部从原 reserved[16] 里划出
 *              (刚好用满)。所以 v1 时代编译的 blob 二进制照旧能跑: 它不读新字段,
 *              也不写 heap_used(恒 0 ⇒ 上报"未用堆"), 行为与从前逐位一致。
 *
 * Layout (little-endian, 4-byte aligned, 128 bytes total):
 *   0x00 uint16_t baseline      in   channel baseline
 *   0x02 uint16_t diff          in   channel filtered diff count
 *   0x04 uint16_t raw           in   channel raw count
 *   0x06 uint16_t noise_th      in   noise threshold
 *   0x08 uint16_t nnoise_th     in   negative noise threshold
 *   0x0A uint16_t max_raw       in   max raw count (full-scale, 0 => 1024)
 *   0x0C uint16_t finger_th     in   finger threshold (context threshold)
 *   0x0E uint16_t base_active   in   base active flag (0/1, IsWidgetActive)
 *   0x10 uint32_t now_ms        in   millisecond timestamp, refreshed/cycle
 *   0x14 uint32_t ch            in   current channel index 0..35
 *   0x18 uint8_t  cfg[8]        in   shared algorithm config (all channels)
 *   0x20 uint8_t  state[64]     i/o  per-channel persistent state
 *   0x60 uint32_t out_active    out  blob writes final 0/1 result
 *   0x64 uint16_t rom           in   per-channel 16-bit ROM (host-downloaded,
 *                                    read-only for the blob; arbitrary use,
 *                                    e.g. per-channel fingerCap/Cp/threshold)
 *   0x66 uint16_t report[4]     out  algorithm-reported debug values
 *   0x6E uint16_t out_led       out  status LED request (0 = off); the engine
 *                                    never derives LED state from out_active
 *   0x70 uint8_t *heap         in   shared scratch heap base (ALGO_HEAP_SIZE bytes)
 *   0x74 uint16_t heap_size    in   heap capacity in bytes (= ALGO_HEAP_SIZE)
 *   0x76 uint16_t heap_used    out  bytes the algorithm claims to occupy
 *   0x78 uint8_t  cfg_ch[8]    in   per-channel algorithm config (this channel)
 ******************************************************************************/

#if !defined(PSOC_ALGO_ABI_H)
#define PSOC_ALGO_ABI_H

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct
{
    uint16_t baseline;      /* 0x00 in  */
    uint16_t diff;           /* 0x02 in  */
    uint16_t raw;             /* 0x04 in  */
    uint16_t noise_th;        /* 0x06 in  */
    uint16_t nnoise_th;       /* 0x08 in  */
    uint16_t max_raw;         /* 0x0A in  */
    uint16_t finger_th;       /* 0x0C in  */
    uint16_t base_active;     /* 0x0E in  */
    uint32_t now_ms;          /* 0x10 in  */
    uint32_t ch;              /* 0x14 in  */
    uint8_t  cfg[8];          /* 0x18 in  */
    uint8_t  state[64];       /* 0x20 i/o */
    uint32_t out_active;      /* 0x60 out */
    uint16_t rom;             /* 0x64 in  per-channel 16-bit ROM (host-downloaded) */
    uint16_t report[4];       /* 0x66 out algorithm-reported debug values (host visualizes per-channel) */
    /* 0x6E out: 白色状态 LED 请求(0=不点亮, 非 0=点亮)。从原 reserved 区首部划出, 既有字段偏移与
     * 结构总长均未变 → 不写本字段的旧算法二进制照常运行, 且因恒为 0 而不会点灯(这正是"原生不点灯")。
     * ★存在理由★: 此前固件把 out_active(触控判定)直接当作点灯信号写死, 于是任何算法只要判定触摸
     * 就必然亮灯, "点不点灯"无法被算法表达。拆出独立输出后, 点灯完全由算法决定。 */
    uint16_t out_led;         /* 0x6E out */
    /* ★共享堆(ABI v2)★ 引擎持有一块 ALGO_HEAP_SIZE 字节的持久暂存区并在这里给出**基址**。
     * ★为什么必须传指针★ blob 是位置无关裸机码, objcopy 只取 .text —— 它没有 .data/.bss,
     * 因此除了 state[64] 之外原本一个字节的持久存储都没有, 也无从知道任何绝对地址。
     * ★与 state[64] 的分工★ state 是**逐通道**私有且在"未激活→激活"边沿被引擎清零;
     * heap 是**全通道共享**且跨代持久(只在算法被替换时清零), 适合放查找表、跨通道统计、
     * 大于 64 字节的工作区。多通道共用同一块 ⇒ 算法自己负责按 io->ch 分区或做互斥语义。
     * heap_used: 算法每次进来自行声明"我占了多少字节"(通常是 sizeof(自己的堆结构))。
     * 引擎按 heap_size 钳位并记录峰值上报给上位机显示占用 —— 不写即 0, 显示"未用"。 */
    uint8_t *heap;            /* 0x70 in  */
    uint16_t heap_size;       /* 0x74 in  */
    uint16_t heap_used;       /* 0x76 out */
    /* ★逐通道配置(ABI v2)★ 与 cfg[8] 并存而不是取代它:
     *   cfg[8]    = 全通道共享, 一处改动 36 个通道同时生效(既有语义, 保留);
     *   cfg_ch[8] = 本通道专属, 每个通道各有一份(引擎按 io->ch 填入)。
     * 面板 36 个电极的 Cp 实测 22~138pF, 灵敏度天差地别, 逐通道阈值是刚需; 而"整体风格"
     * 类参数(窗口长度、模式开关)用共享的一份更省事 —— 两者是不同需求, 不该互相取代。 */
    uint8_t  cfg_ch[8];       /* 0x78 in  */
} algo_io_t;

/* Host-parsed naming metadata (expand to nothing; the upper computer greps the C source
 * for these to build the report/setting variable schema — names + defaults). Use at file scope.
 *   ALGO_REPORT(0, "delta")        -> report[0] shown/tracked in UI as "delta"
 *   ALGO_SETTING(0, "gain", 15)    -> cfg[0] adjustable in UI as "gain", default 15 */
#define ALGO_REPORT(idx, name)
#define ALGO_SETTING(idx, name, defval)
/* Extended host-only metadata declarations. They also compile to nothing and do not change the ABI. */
#define ALGO_REPORT_META(idx, name, type, minval, maxval, description, alias)
#define ALGO_SETTING_META(idx, name, type, defval, minval, maxval, description, alias)
/* 逐通道可设置变量声明(对应 cfg_ch[8])。与 ALGO_SETTING* 是**两套独立的下标空间**:
 * ALGO_SETTING(0,...) 说的是 cfg[0](全通道共享), ALGO_SETTING_CH(0,...) 说的是 cfg_ch[0]
 * (本通道)。上位机据此把两类变量分开呈现: 逐通道项进"批量设置"与单通道精调页, 共享项
 * 单独一个入口 —— 混在一起会让用户以为改一项只影响勾选的通道(实际全通道生效)。 */
#define ALGO_SETTING_CH(idx, name, defval)
#define ALGO_SETTING_CH_META(idx, name, type, defval, minval, maxval, description, alias)

/* Compile-time layout guarantees (offsetof-based; portable, no attributes). */
#define _ALGO_IO_OFFSETOF(field) ((uint32_t)(size_t)&(((const algo_io_t *)0)->field))
#if defined(__GNUC__) || defined(__clang__)
_Static_assert(_ALGO_IO_OFFSETOF(baseline)     == 0x00u, "algo_io_t.baseline offset");
_Static_assert(_ALGO_IO_OFFSETOF(diff)         == 0x02u, "algo_io_t.diff offset");
_Static_assert(_ALGO_IO_OFFSETOF(raw)          == 0x04u, "algo_io_t.raw offset");
_Static_assert(_ALGO_IO_OFFSETOF(noise_th)     == 0x06u, "algo_io_t.noise_th offset");
_Static_assert(_ALGO_IO_OFFSETOF(nnoise_th)    == 0x08u, "algo_io_t.nnoise_th offset");
_Static_assert(_ALGO_IO_OFFSETOF(max_raw)      == 0x0Au, "algo_io_t.max_raw offset");
_Static_assert(_ALGO_IO_OFFSETOF(finger_th)    == 0x0Cu, "algo_io_t.finger_th offset");
_Static_assert(_ALGO_IO_OFFSETOF(base_active)  == 0x0Eu, "algo_io_t.base_active offset");
_Static_assert(_ALGO_IO_OFFSETOF(now_ms)       == 0x10u, "algo_io_t.now_ms offset");
_Static_assert(_ALGO_IO_OFFSETOF(ch)           == 0x14u, "algo_io_t.ch offset");
_Static_assert(_ALGO_IO_OFFSETOF(cfg)          == 0x18u, "algo_io_t.cfg offset");
_Static_assert(_ALGO_IO_OFFSETOF(state)        == 0x20u, "algo_io_t.state offset");
_Static_assert(_ALGO_IO_OFFSETOF(out_active)   == 0x60u, "algo_io_t.out_active offset");
_Static_assert(_ALGO_IO_OFFSETOF(rom)          == 0x64u, "algo_io_t.rom offset");
_Static_assert(_ALGO_IO_OFFSETOF(report)       == 0x66u, "algo_io_t.report offset");
_Static_assert(_ALGO_IO_OFFSETOF(out_led)      == 0x6Eu, "algo_io_t.out_led offset");
_Static_assert(_ALGO_IO_OFFSETOF(heap)         == 0x70u, "algo_io_t.heap offset");
_Static_assert(_ALGO_IO_OFFSETOF(heap_size)    == 0x74u, "algo_io_t.heap_size offset");
_Static_assert(_ALGO_IO_OFFSETOF(heap_used)    == 0x76u, "algo_io_t.heap_used offset");
_Static_assert(_ALGO_IO_OFFSETOF(cfg_ch)       == 0x78u, "algo_io_t.cfg_ch offset");
_Static_assert(sizeof(algo_io_t)               == 0x80u, "algo_io_t total size must be 128 bytes");
#endif /* __GNUC__ || __clang__ */
#undef _ALGO_IO_OFFSETOF

/* Algorithm blob entry point type. Argument passed in r0 (AAPCS). */
typedef void (*algo_fn_t)(algo_io_t *io);

/* SPI command set for algorithm blob upload (7-byte frames, see main.c). */
#define ALGO_BEGIN   (0x40u)
#define ALGO_PAGE    (0x41u)
#define ALGO_END     (0x42u)
#define ALGO_INFO    (0x43u)
#define ALGO_SET_ROM (0x44u)  /* [magic,SET_ROM,ch,rom_lo,rom_hi,0,0] 设 per-channel ROM */
#define ALGO_GET_ROM (0x45u)  /* [magic,GET_ROM,ch,0,0,0,0] → resp [.. ,ch,0,rom_lo,rom_hi,0] */
#define ALGO_GET_TRACE (0x46u) /* [magic,GET_TRACE,ch,idx,..] → resp [..,ch,out_active,report[idx]_lo,report[idx]_hi,idx] */
#define ALGO_SET_CFG (0x47u)  /* [magic,SET_CFG,idx,val,..] 设共享 cfg[idx]=val(算法可设置变量) */
#define ALGO_GET_CFG (0x48u)  /* [magic,GET_CFG,idx,..] → resp [..,idx,0,cfg[idx],0,0] */
/* 0x49 = SENSOR_CMD_FOCUS_SCAN(见 main.c), 不属于算法域, 故算法域从 0x4A 继续。 */
#define ALGO_SET_CFG_CH (0x4Au) /* [magic,SET_CFG_CH,ch,idx,val,0,0] 设 cfg_ch[ch][idx] → resp [..,ch,idx,val,0,0] */
#define ALGO_GET_CFG_CH (0x4Bu) /* [magic,GET_CFG_CH,ch,idx,..]     → resp [..,ch,idx,val_lo,val_hi,0] */
#define ALGO_GET_HEAP   (0x4Cu) /* [magic,GET_HEAP,..] → resp [..,size_lo,size_hi,used_lo,used_hi,0] 堆容量+峰值占用 */
/* ★"上传到底有没有换代"只能由这条命令回答★ ALGO_INFO 的 valid/len 无法区分"新算法已装上"
 * 与"旧算法还在、长度恰好相同", 于是上位机会把失败的上传报成成功(实测踩到)。本命令回的是
 * **当前槽内实际内容**在 commit 时算出的 CRC16, 与上位机手里的期望值逐位对账, 冒充不了。 */
#define ALGO_GET_CRC (0x4Du) /* [magic,GET_CRC,..] → resp [..,valid,0,crc_lo,crc_hi,0] */
/* ★容量必须由设备自报, 不许上位机硬编码★ 槽大小/堆大小散落在三层(PSoC 的 ALGO_SLOT_SIZE、
 * RP2040 的 PSOC_ALGO_MAX_LEN、上位机的 proto 常量), 漏改任何一处的表现都是"上传成功但算法
 * 跑飞/仍跑旧代码", 三层都不会报错。让 PSoC 把自己**真正生效**的容量报上来, 上位机据此显示
 * 占用与做编译闸门 ⇒ 不一致当场就能看见, 而不是等到算法跑飞才去猜。 */
#define ALGO_GET_CAPS (0x4Eu) /* [magic,GET_CAPS,..] → resp [..,heap_lo,heap_hi,slot_lo,slot_hi,0] */

/* Executable RAM slot size for the algorithm blob (ABI v2: 1KB → 4KB)。
 * ★4KB 的代价是没有独立暂存区★ 16KB SRAM 装不下 4KB 槽 + 4KB 暂存(实测 .bss 之后只剩 3592B),
 * 故 ALGO_PAGE 直接写槽; 上传期间引擎退回原生 CapSense 判定(algo_valid=0), CRC 通过才重新启用。 */
#define ALGO_SLOT_SIZE (4096u)
/* 全通道共享的算法暂存堆(字节)。见 algo_io_t::heap。 */
#define ALGO_HEAP_SIZE (256u)
/* ALGO_PAGE 每帧携带的算法字节数。7 字节帧 = magic + cmd + 16 位页号 + 本常量。
 * ★为什么页号占 2 字节★ 4KB 槽的页数远超 255; 8 位页号会回绕并静默把数据写歪。
 * ★为什么必须绝对寻址(page*ALGO_PAGE_BYTES)★ RP2040 的 _cmd_txn 在回显不匹配时会重发同一条
 * 命令, 它整套原语都以"命令幂等"为前提。绝对寻址下重复写同一页结果不变, 天然幂等;
 * 换成"顺序游标"就会因一次重发而错位, 且页数越多越容易撞上(实测 1056B 必败, 616B 能过)。 */
#define ALGO_PAGE_BYTES (3u)

#if defined(__cplusplus)
}
#endif

#endif /* PSOC_ALGO_ABI_H */

/* [] END OF FILE */
