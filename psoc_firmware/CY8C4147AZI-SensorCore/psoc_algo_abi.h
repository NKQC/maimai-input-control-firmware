/******************************************************************************
 * File Name:   psoc_algo_abi.h
 *
 * Description: ABI v1 shared contract between the PSoC firmware (main.c) and
 *              the JIT-loadable touch algorithm blob (1KB Thumb machine code,
 *              position independent, entry at algo_slot|1). Field offsets are
 *              fixed and must never change without bumping the ABI version;
 *              both the engine (main.c) and every algorithm blob source
 *              (e.g. psoc_algo_default.c) include this single header.
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
 *   0x70 uint8_t  reserved[16]  -    padding to 0x80 (128 bytes)
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
    uint8_t  reserved[16];    /* 0x70 -   pad to 0x80 */
} algo_io_t;

/* Host-parsed naming metadata (expand to nothing; the upper computer greps the C source
 * for these to build the report/setting variable schema — names + defaults). Use at file scope.
 *   ALGO_REPORT(0, "delta")        -> report[0] shown/tracked in UI as "delta"
 *   ALGO_SETTING(0, "gain", 15)    -> cfg[0] adjustable in UI as "gain", default 15 */
#define ALGO_REPORT(idx, name)
#define ALGO_SETTING(idx, name, defval)

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
_Static_assert(_ALGO_IO_OFFSETOF(reserved)     == 0x70u, "algo_io_t.reserved offset");
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

/* Fixed 1KB executable RAM slot size for the algorithm blob. */
#define ALGO_SLOT_SIZE (1024u)

#if defined(__cplusplus)
}
#endif

#endif /* PSOC_ALGO_ABI_H */

/* [] END OF FILE */
