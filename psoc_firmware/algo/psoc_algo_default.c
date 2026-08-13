/******************************************************************************
 * File Name: psoc_algo_default.c
 *
 * Description: Default out-of-the-box HDR (high dynamic range) touch trigger
 *              algorithm blob (ABI v1). This is a straight single-channel
 *              port of v3.1 fast_trigger_process() (see
 *              maimai-input-control-firmware/.../fast_trigger.c) rewritten to
 *              read/write the fixed algo_io_t contract instead of the
 *              original per-channel arrays + bitmask return value.
 *
 * Hard constraints (blob executes bare from a 1KB PSoC RAM slot):
 *   - No '/' or '%' operators anywhere (division by 1000 is done with an
 *     exact 32-bit restoring binary long division helper, see _udiv1000()).
 *   - No 64-bit integer types/arithmetic anywhere.
 *   - No external symbols, no relocations, no libc/libgcc calls.
 *   - algo() must be the first symbol emitted into .text (offset 0). This is
 *     guaranteed here by forward-declaring the helpers and defining algo()
 *     first; helper bodies are placed after algo() in this translation unit
 *     so, inlined or not, nothing precedes algo() in the emitted .text.
 *
 * Build/verify exclusively via: dev.ps1 build-blob (gcc -Os -mthumb
 * -mcpu=cortex-m0plus -ffreestanding -nostdlib -c, then objcopy -O binary
 * -j .text, then nm/objdump checks). Do not hand-invoke the toolchain.
 ******************************************************************************/

/* psoc_algo_abi.h's _ALGO_IO_OFFSETOF macro (used only by its internal
 * _Static_assert checks) needs size_t. stddef.h is a compiler-provided
 * freestanding header (arm-none-eabi-gcc ships it standalone, no libc/
 * libgcc symbols), safe under -ffreestanding -nostdlib. Included here
 * rather than editing the shared ABI header, which is out of scope for
 * this file. */
#include <stddef.h>
#include "psoc_algo_abi.h"

/* ---- host UI schema declarations (macros expand to nothing; grepped by the
 * upper computer to build the report/setting variable schema, see
 * psoc_algo_abi.h ALGO_REPORT/ALGO_SETTING doc comment). Placed at file scope.
 *
 * report[0]="diff": exposes the live filtered diff count each cycle, so the
 * host "算法上报" sub-tab has something to plot out of the box.
 *
 * cfg[4]="bsln_offset": a signed per-channel baseline offset (see algo()
 * below) added to the diff used in the trigger decision. idx=4 (not 0) is
 * deliberate: cfg[0..3] already carry this file's pre-existing "rise/drop
 * permille override" feature (read once at touch-down init below); reusing
 * cfg[0] for a second, differently-timed purpose would silently couple two
 * unrelated tunables onto the same byte. cfg[4] is otherwise unused. */
/* v3.1 HDR 算法上报/可调重写: 每项设置占 cfg[] 一个字节(UI 每项一个 SpinBox), 每个上报占 report[]
 * 一个 u16。设置默认值: 0 视作"用内置默认"(仅 permille/window, 它们取 0 无意义); bsln_offset 例外
 * (0 即中性偏移, 合法)。 */
/* ★别名与简介不写在这里★ 它们是纯界面文本, 对编译与运行没有任何影响。写进 C 源就意味着中文
 * 要随源码上传进设备 flash、再回读、再被宏解析器切一遍, 这条往返链上任一处按字节处理都会产出
 * mojibake(实测踩到两处)。故约定: 固件侧只存 C 源与 ASM; 中文别名/简介由上位机按下面这些
 * **英文 name** 提供(control_software/src/algo_i18n.rs), 用户覆盖存本地 jit_metadata.json,
 * 可导入导出分享。name 即契约, 复用同名变量的自定义算法自动获得同一套中文。 */
ALGO_REPORT_META(0, "diff", "u16", 0, 65535, "", "")
ALGO_REPORT_META(1, "env_hi", "u16", 0, 65535, "", "")
ALGO_REPORT_META(2, "env_lo", "u16", 0, 65535, "", "")
ALGO_REPORT_META(3, "margin", "u16", 0, 65535, "", "")
ALGO_SETTING_META(0, "rise_permille", "u8", 15, 0, 255, "", "")
ALGO_SETTING_META(1, "drop_permille", "u8", 25, 0, 255, "", "")
ALGO_SETTING_META(2, "window_ms", "u8", 5, 0, 255, "", "")
ALGO_SETTING_META(3, "bsln_offset", "i8", 0, -128, 127, "", "")

/* ---- forward declarations (bodies defined after algo(), see file footer)
 * _udiv1000() is force-inlined via always_inline: it is called twice from
 * algo(), and at -Os GCC would otherwise materialize it as a separate,
 * non-inlined static function placed *before* algo() in .text (observed:
 * nm showed _udiv1000 at offset 0, algo at 0x30). always_inline guarantees
 * algo() remains the sole/first symbol in .text at offset 0. */
static inline uint32_t _udiv1000(uint32_t n) __attribute__((always_inline));
static inline int32_t  _median3(int32_t a, int32_t b, int32_t c);
static inline uint16_t _u16_max(uint16_t a, uint16_t b);

/* ---- HDR tuning constants (mirrors fast_trigger.h) ---- */
#define _HDR_INVALID_HIGH          ((int32_t)0)
#define _HDR_INVALID_LOW           ((int32_t)65535)
#define _HDR_WINDOW_MS              (5u)
#define _HDR_RISE_PERMILLE_DEFAULT (15u)
#define _HDR_DROP_PERMILLE_DEFAULT (25u)

/* ---- per-channel persistent state, lives in io->state[64] ----
 * Mirrors v3.1 fast_ch_t, minus the uint64 timestamps (now_ms is 32-bit ms,
 * per the ABI, which is sufficient for the 5ms envelope window) and minus
 * the redundant 'baseline' cache (io->baseline is already the live input).
 *
 * NOTE on 'initialized' polarity: the engine zero-clears io->state[] on the
 * inactive->active edge (see jit-algo-engine.md #3). A freshly-zeroed struct
 * therefore has initialized==0, which this code reads as "needs (re)init".
 * That is the opposite polarity of v3.1's 'need_reset' (true==needs reset);
 * the flip is intentional so the engine's zero-clear and this blob's own
 * deactivation-time clear both naturally converge on "needs init" == 0.
 */
typedef struct
{
    uint16_t x_delta_rise;
    uint16_t x_delta_fall;
    int32_t  z_h;
    int32_t  z_l;
    uint32_t z_ht;
    uint32_t z_lt;
    uint8_t  initialized;
    uint8_t  forced_release;
    uint8_t  hist_valid;
    uint8_t  _reserved0;
    int32_t  d0;
    int32_t  d1;
    int32_t  d2;
} hdr_state_t;

_Static_assert(sizeof(hdr_state_t) <= 64, "state overflow");

/* ---- ABI entry point. Must be the first symbol in .text (offset 0). ---- */
void algo(algo_io_t *io)
{
    hdr_state_t *st = (hdr_state_t *)io->state;

    /* Always publish the live filtered diff as report[0]="diff" (see ALGO_REPORT
     * above), regardless of active/inactive, so the host trace view shows a
     * continuous line rather than gaps while the widget is inactive. */
    io->report[0] = io->diff;

    if (io->base_active == 0u)
    {
        st->initialized     = 0u;
        st->forced_release   = 0u;
        st->hist_valid       = 0u;
        io->out_active       = 0u;
        io->report[1]        = 0u;
        io->report[2]        = 0u;
        io->report[3]        = 0u;
        return;
    }

    if (st->initialized == 0u)
    {
        /* 可调设置每字节一项(cfg[0]=rise_permille, cfg[1]=drop_permille); 0 视作用内置默认。 */
        uint16_t rise_permille = (io->cfg[0] != 0u) ? (uint16_t)io->cfg[0] : _HDR_RISE_PERMILLE_DEFAULT;
        uint16_t drop_permille = (io->cfg[1] != 0u) ? (uint16_t)io->cfg[1] : _HDR_DROP_PERMILLE_DEFAULT;

        {
            uint16_t nz    = _u16_max(io->noise_th, io->nnoise_th);
            /* baseline (u16) * permille (u16) always fits in u32 (max ~4.29e9
             * < 2^32-1), so this is safe with no overflow check needed. */
            uint32_t rise  = _udiv1000((uint32_t)io->baseline * (uint32_t)rise_permille) + nz;
            uint32_t fall  = _udiv1000((uint32_t)io->baseline * (uint32_t)drop_permille) + nz;
            st->x_delta_rise = (rise > 0xFFFFu) ? 0xFFFFu : (uint16_t)rise;
            st->x_delta_fall = (fall > 0xFFFFu) ? 0xFFFFu : (uint16_t)fall;
        }

        st->initialized     = 1u;
        st->forced_release   = 0u;
        st->hist_valid       = 0u;
        st->z_h              = _HDR_INVALID_HIGH;
        st->z_l              = _HDR_INVALID_LOW;
        st->z_ht             = io->now_ms;
        st->z_lt             = io->now_ms;
    }

    {
        uint32_t now = io->now_ms;
        uint32_t win = (io->cfg[2] != 0u) ? (uint32_t)io->cfg[2] : _HDR_WINDOW_MS;  /* window_ms(cfg[2]) */
        int32_t  d   = (int32_t)io->diff;
        int32_t  override_active;

        if (!st->hist_valid)
        {
            st->d0 = d;
            st->d1 = d;
            st->d2 = d;
            st->hist_valid = 1u;
        }
        else
        {
            st->d2 = st->d1;
            st->d1 = st->d0;
            st->d0 = d;
        }
        d = _median3(st->d0, st->d1, st->d2);

        /* Apply the "bsln_offset" adjustable setting (cfg[3], ALGO_SETTING
         * above): the raw byte is reinterpreted as signed two's complement
         * (-128..127) and added to the median-filtered diff before the
         * trigger decision. Default 0 is neutral (no change); the host UI
         * lets the user raise/lower a channel's effective sensitivity
         * without recompiling. */
        d += (int32_t)(int8_t)io->cfg[3];

        if (st->forced_release)
        {
            override_active = 0;

            if (((now - st->z_lt) > win) || (st->z_l == _HDR_INVALID_LOW))
            {
                st->z_l = d;
                st->z_lt = now;
            }
            else if (d < st->z_l)
            {
                st->z_l = d;
                st->z_lt = now;
            }

            if (((d - st->z_l) > (int32_t)st->x_delta_fall) &&
                ((uint16_t)d > io->finger_th))
            {
                st->forced_release = 0u;
                override_active    = 1;
                st->z_h  = d; st->z_l  = d;
                st->z_ht = now; st->z_lt = now;
            }
        }
        else
        {
            if ((st->z_h == _HDR_INVALID_HIGH) || (st->z_l == _HDR_INVALID_LOW))
            {
                st->z_h  = d; st->z_l  = d;
                st->z_ht = now; st->z_lt = now;
            }
            else
            {
                if ((now - st->z_ht) > win)
                {
                    st->z_h  = d;
                    st->z_ht = now;
                }
                else if (d > st->z_h)
                {
                    st->z_h  = d;
                    st->z_ht = now;
                }

                if ((now - st->z_lt) > win)
                {
                    st->z_l  = d;
                    st->z_lt = now;
                }
                else if (d < st->z_l)
                {
                    st->z_l  = d;
                    st->z_lt = now;
                }
            }

            override_active = 1;
            if ((st->z_h - d) > (int32_t)st->x_delta_rise)
            {
                st->forced_release = 1u;
                override_active    = 0;
                st->z_h  = d; st->z_l  = d;
                st->z_ht = now; st->z_lt = now;
            }
        }

        io->out_active = (uint32_t)override_active;

        /* 上报内部状态供 UI「算法上报」可视化(u16, 负值截 0, 超范围截 0xFFFF)。
         * env_hi/env_lo=动态包络峰谷; margin=距触发(未按下)或距释放(已按下)的余量。 */
        io->report[1] = (st->z_h < 0) ? 0u : ((st->z_h > 0xFFFF) ? 0xFFFFu : (uint16_t)st->z_h);
        io->report[2] = (st->z_l < 0) ? 0u : ((st->z_l > 0xFFFF) ? 0xFFFFu : (uint16_t)st->z_l);
        {
            int32_t m = st->forced_release ? (d - st->z_l) : (st->z_h - d);
            io->report[3] = (m < 0) ? 0u : ((m > 0xFFFF) ? 0xFFFFu : (uint16_t)m);
        }
    }
}

/* ---- helper implementations (placed after algo() so algo() is guaranteed
 * to be the first emitted symbol in .text regardless of whether the
 * compiler chooses to inline these or keep them as separate static
 * functions). None of these use '/', '%', or any 64-bit arithmetic. ---- */

/* Exact unsigned division by 1000 via 32-bit restoring binary long division.
 * Pure shift/add/compare on 32-bit values only: no overflow is possible
 * (running remainder never exceeds 1999), no '/'/'%'/64-bit ops, and no
 * external symbols are generated. Error vs. true n/1000 is exactly 0
 * (bit-identical to the C '/' operator's result for unsigned operands). */
static inline uint32_t _udiv1000(uint32_t n)
{
    uint32_t q   = 0u;
    uint32_t rem = 0u;
    int32_t  i;

    for (i = 31; i >= 0; --i)
    {
        rem = (rem << 1) | ((n >> i) & 1u);
        q <<= 1;
        if (rem >= 1000u)
        {
            rem -= 1000u;
            q |= 1u;
        }
    }
    return q;
}

static inline int32_t _median3(int32_t a, int32_t b, int32_t c)
{
    if (a > b) { int32_t t = a; a = b; b = t; }
    if (b > c) { int32_t t = b; b = c; c = t; }
    if (a > b) { int32_t t = a; a = b; b = t; }
    return b;
}

static inline uint16_t _u16_max(uint16_t a, uint16_t b)
{
    return (a > b) ? a : b;
}

/* [] END OF FILE */
