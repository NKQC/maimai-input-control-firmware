/******************************************************************************
 * File Name: psoc_algo_v4.c
 *
 * Description: v4 单通道 HDR 触控算法(ABI v2)。以 v3.1(psoc_algo_default.c)为基础, 两处改动:
 *
 *   ① 原先 4 个可调量改为**逐通道**(cfg_ch[0..3])。面板 36 个电极实测 Cp 22~138pF, 灵敏度
 *      天差地别, 用一套共享阈值必然是"照顾了最钝的就冤枉最灵的"。
 *
 *   ② 修掉 v3.1 的误松开: **已按下的分区被另一只手掠过时会被判成松开**。
 *      成因(v3.1 逻辑): 激活态用 5ms 窗口内的峰 z_h 作参考, 只要 z_h - d 超过 x_delta_rise
 *      就判 forced_release。掠过的手先把 d 抬高一个瞬时尖峰 ⇒ z_h 记住了这个尖峰 ⇒ 手离开后
 *      d 回落到"仍被按着"的真实水平, z_h - d 却已经超过阈值 ⇒ 明明还按着却报松开。
 *      v3.1 后来加的 stay_active_permille 是拿一个**绝对**阈值挡住 forced_release —— 那等于
 *      在 HDR 算法里插一条固定门槛, 把动态范围又还回去了, 所以本版把它整条去掉。
 *
 *      v4 的修法是三条**相对**判据同时成立才松开, 全部由信号自身导出, 不引入绝对门槛:
 *        (a) z_h - d > x_delta_rise            —— v3.1 的原判据, 原样保留;
 *        (b) d 已经跌到 press_ref 之下够多     —— press_ref 是"这次按下的稳态水平"(慢升快降跟踪),
 *            掠过造成的尖峰几乎抬不动它, 手离开后 d 回到 ≈press_ref ⇒ (b) 不成立 ⇒ 不松开;
 *            真手指抬起时 d 会明显跌破 press_ref ⇒ (b) 成立 ⇒ 正常松开;
 *        (c) 同一周期内没有"多通道同时上升"    —— 掠过的手必然同时抬高**相邻若干通道**,
 *            而真手指抬起只会让本通道下降。这条证据必须跨通道才拿得到, 所以用 ABI v2 的
 *            共享堆(io->heap)存全通道上升态 —— state[64] 是逐通道私有的, 存不了这个。
 *      另外 z_h 的上升被限速(peak_slew): 尖峰一个周期最多把参考抬高 peak_slew 个 count,
 *      从源头削掉"参考被尖峰污染"这件事本身。
 *
 *      三条判据是**互相独立**的证据, 任一条不成立就不松开; 同时保留一条绝对兜底:
 *      d 掉到 finger_th 的一半以下时无条件松开 —— 否则任何逻辑缺陷都可能变成"永久按住"。
 *
 * Hard constraints (blob executes bare from the PSoC RAM slot):
 *   - 全文禁止 '/' 与 '%'(除 1000 用 _udiv1000() 的 32 位恢复余数长除法), 禁止 64 位整数,
 *     禁止外部符号/重定位/libc/libgcc 调用。
 *   - algo() 必须是 .text 里第一个符号(偏移 0): 故 helper 全部前向声明 + 定义在 algo() 之后。
 *
 * Build/verify: 上位机算法页"编译"(内置 arm-none-eabi 工具链, 与 dev.ps1 build-blob 同参数)。
 ******************************************************************************/

#include <stddef.h>
#include "psoc_algo_abi.h"

/* ---- 上位机 schema 声明(宏展开为空, 由上位机解析源码取名/默认值/范围) ----
 * 逐通道项用 ALGO_SETTING_CH_META(对应 cfg_ch[]), 共享项用 ALGO_SETTING_META(对应 cfg[])。
 * 两者是**独立的下标空间**: CH 的 0 指 cfg_ch[0], 非 CH 的 0 指 cfg[0]。
 * 别名/简介一律留空: 中文界面文本由上位机 algo_i18n.rs 按英文 name 提供 —— 中文若随 C 源
 * 上传进设备 flash 再回读再被宏解析器切一遍, 这条往返链上任一处按字节处理都会产出乱码。 */
ALGO_REPORT_META(0, "diff", "u16", 0, 65535, "", "")
ALGO_REPORT_META(1, "env_hi", "u16", 0, 65535, "", "")
ALGO_REPORT_META(2, "press_ref", "u16", 0, 65535, "", "")
ALGO_REPORT_META(3, "margin", "u16", 0, 65535, "", "")

ALGO_SETTING_CH_META(0, "rise_permille", "u8", 15, 0, 255, "", "")
ALGO_SETTING_CH_META(1, "drop_permille", "u8", 25, 0, 255, "", "")
ALGO_SETTING_CH_META(2, "window_ms", "u8", 5, 0, 255, "", "")
ALGO_SETTING_CH_META(3, "bsln_offset", "i8", 0, -128, 127, "", "")

ALGO_SETTING_META(0, "guard_permille", "u8", 8, 0, 255, "", "")
ALGO_SETTING_META(1, "peak_slew", "u8", 24, 0, 255, "", "")
ALGO_SETTING_META(2, "press_attack_shift", "u8", 5, 1, 8, "", "")
ALGO_SETTING_META(3, "press_decay_shift", "u8", 2, 1, 8, "", "")
ALGO_SETTING_META(4, "brush_channels", "u8", 2, 0, 36, "", "")

/* ---- 前向声明: 全部 always_inline, 保证 algo() 仍是 .text 里第一个符号 ----
 * (-Os 下 GCC 会把被调用两次的 static 函数单独实体化并排在 algo() 之前, 实测踩过:
 *  nm 显示 _udiv1000 在偏移 0, algo 在 0x30 —— 上位机的"入口必须在偏移 0"校验会直接拒。) */
static inline uint32_t _udiv1000(uint32_t n) __attribute__((always_inline));
static inline int32_t  _median3(int32_t a, int32_t b, int32_t c) __attribute__((always_inline));
static inline uint16_t _u16_max(uint16_t a, uint16_t b) __attribute__((always_inline));
static inline uint16_t _clamp_u16(int32_t v) __attribute__((always_inline));

/* ---- HDR 常量(与 v3.1 一致, 只在对应 cfg 为 0 时兜底) ---- */
#define _HDR_INVALID_HIGH           ((int32_t)0)
#define _HDR_INVALID_LOW            ((int32_t)65535)
#define _HDR_WINDOW_MS              (5u)
#define _HDR_RISE_PERMILLE_DEFAULT  (15u)
#define _HDR_DROP_PERMILLE_DEFAULT  (25u)
#define _V4_GUARD_PERMILLE_DEFAULT  (8u)
#define _V4_PEAK_SLEW_DEFAULT       (24u)
#define _V4_ATTACK_SHIFT_DEFAULT    (5u)
#define _V4_DECAY_SHIFT_DEFAULT     (2u)
#define _V4_BRUSH_CHANNELS_DEFAULT  (2u)
#define _V4_CHANNELS                (36u)

/* ---- 逐通道持久状态, 位于 io->state[64] ----
 * 引擎在"未激活→激活"边沿把 state[] 清零, 故 initialized==0 天然表示"需要初始化"。 */
typedef struct
{
    uint16_t x_delta_rise;
    uint16_t x_delta_fall;
    uint16_t x_guard;        /* 松开还需要跌破 press_ref 的幅度(由 guard_permille 导出) */
    uint16_t _pad0;
    int32_t  z_h;
    int32_t  z_l;
    int32_t  press_ref;      /* 本次按下的稳态水平(慢升快降) */
    uint32_t z_ht;
    uint32_t z_lt;
    int32_t  d0;
    int32_t  d1;
    int32_t  d2;
    uint8_t  initialized;
    uint8_t  forced_release;
    uint8_t  hist_valid;
    uint8_t  _pad1;
} v4_state_t;

_Static_assert(sizeof(v4_state_t) <= 64, "state overflow");

/* ---- 全通道共享堆(io->heap), 用于"是否有手掠过"的跨通道证据 ----
 * ★为什么必须放堆而不是 state[]★ state 是逐通道私有的, 通道 A 看不到通道 B 的上升态;
 * 而"多通道同时上升"这条证据本质上是跨通道的。ABI v2 的共享堆正是为此存在。
 * 周期边界识别: 引擎每个扫描周期按通道号**升序**调用(禁用通道被跳过, 但仍升序),
 * 故一旦 ch <= last_ch 就说明新一轮开始 —— 此时把本轮累计值发布为"上一轮结论"再清零。
 * 用"上一轮结论"而不是本轮累计, 是为了让 36 个通道在同一周期内看到**同一份**证据
 * (否则通道 0 看到的计数永远是 0, 通道 35 看到的才是全量, 判据随通道号而异 = 不可复现)。 */
typedef struct
{
    uint16_t diff_prev[_V4_CHANNELS];   /* 上一周期本通道的滤波后 diff */
    uint8_t  rising[_V4_CHANNELS];      /* 本周期本通道是否在上升 */
    uint16_t last_ch;                   /* 上一次被调用的通道号(周期边界识别) */
    uint8_t  rising_count_acc;          /* 本周期累计的上升通道数 */
    uint8_t  rising_count_pub;          /* 上一周期的结论, 全通道共用 */
    uint8_t  primed;                    /* 堆已初始化标记(引擎换代时把堆清零 ⇒ 自动重初始化) */
    uint8_t  _pad[3];
} v4_heap_t;

_Static_assert(sizeof(v4_heap_t) <= 256, "heap overflow");

/* ---- ABI 入口。必须是 .text 里的第一个符号(偏移 0)。 ---- */
void algo(algo_io_t *io)
{
    v4_state_t *st = (v4_state_t *)io->state;
    v4_heap_t  *hp = (v4_heap_t *)0;
    uint16_t    noise = _u16_max(io->noise_th, io->nnoise_th);
    uint8_t     brush_now = 0u;
    /* 逐通道可调量: 0 一律表示"用内置默认"(bsln_offset 例外, 0 就是中性偏移, 合法)。 */
    uint32_t    win  = (io->cfg_ch[2] != 0u) ? (uint32_t)io->cfg_ch[2] : _HDR_WINDOW_MS;
    uint32_t    slew = (io->cfg[1] != 0u) ? (uint32_t)io->cfg[1] : _V4_PEAK_SLEW_DEFAULT;
    uint32_t    atk  = (io->cfg[2] != 0u) ? (uint32_t)io->cfg[2] : _V4_ATTACK_SHIFT_DEFAULT;
    uint32_t    dcy  = (io->cfg[3] != 0u) ? (uint32_t)io->cfg[3] : _V4_DECAY_SHIFT_DEFAULT;
    uint32_t    brush_th = (io->cfg[4] != 0u) ? (uint32_t)io->cfg[4] : _V4_BRUSH_CHANNELS_DEFAULT;
    uint32_t    now = io->now_ms;
    int32_t     d;

    if (atk > 8u) { atk = 8u; }
    if (dcy > 8u) { dcy = 8u; }

    /* ---- ① 跨通道"掠过"证据的维护(必须先做, 且对**全部**通道都做) ----
     * ★为什么不能放到 base_active 之后★ 被手掠过的通道通常还没到 base_active, 但它的 diff
     * 正在上升 —— 那正是要收集的证据。放在 early return 之后就只统计得到已激活的通道,
     * 恰好把最关键的样本漏掉。 */
    if ((io->heap != (uint8_t *)0) && (io->heap_size >= (uint16_t)sizeof(v4_heap_t)))
    {
        hp = (v4_heap_t *)io->heap;
        /* 引擎在算法换代时把整块堆清零 ⇒ primed==0 自动触发一次重初始化, 不需要额外协议。 */
        if (hp->primed == 0u)
        {
            uint32_t i;
            for (i = 0u; i < _V4_CHANNELS; i++) { hp->diff_prev[i] = 0u; hp->rising[i] = 0u; }
            hp->rising_count_acc = 0u;
            hp->rising_count_pub = 0u;
            hp->last_ch = (uint16_t)_V4_CHANNELS;   /* 逼出一次"新周期"判定 */
            hp->primed = 1u;
        }
        /* 周期边界: 通道号不再递增 ⇒ 新一轮开始, 把上一轮累计发布为全通道共用的结论。 */
        if ((uint16_t)io->ch <= hp->last_ch)
        {
            hp->rising_count_pub = hp->rising_count_acc;
            hp->rising_count_acc = 0u;
        }
        hp->last_ch = (uint16_t)io->ch;

        if (io->ch < _V4_CHANNELS)
        {
            /* 上升判据用**原始 diff** 且以噪声阈值为尺度: 自适应, 不引入任何绝对门槛。 */
            uint32_t prev = (uint32_t)hp->diff_prev[io->ch];
            uint8_t  up = ((uint32_t)io->diff > (prev + (uint32_t)noise)) ? 1u : 0u;
            hp->rising[io->ch] = up;
            if (up != 0u && hp->rising_count_acc < 255u) { hp->rising_count_acc++; }
            hp->diff_prev[io->ch] = io->diff;
        }
        /* 本通道自己在升不算"被掠过"的证据 —— 要的是**别的**通道也在升。 */
        {
            uint32_t others = (uint32_t)hp->rising_count_pub;
            if ((io->ch < _V4_CHANNELS) && (hp->rising[io->ch] != 0u) && (others > 0u)) { others--; }
            brush_now = (others >= brush_th) ? 1u : 0u;
        }
        io->heap_used = (uint16_t)sizeof(v4_heap_t);
    }
    else
    {
        /* 堆不可用(ABI v1 引擎 / 容量不足): 判据 (c) 降级为"无掠过", (a)(b) 照常工作。 */
        io->heap_used = 0u;
    }

    io->report[0] = io->diff;

    if (io->base_active == 0u)
    {
        st->initialized    = 0u;
        st->forced_release = 0u;
        st->hist_valid     = 0u;
        io->out_active     = 0u;
        io->report[1]      = 0u;
        io->report[2]      = 0u;
        io->report[3]      = 0u;
        return;
    }

    /* ---- ② 按下瞬间的一次性初始化: 由逐通道 permille 导出三个相对阈值 ---- */
    if (st->initialized == 0u)
    {
        uint16_t rise_permille = (io->cfg_ch[0] != 0u) ? (uint16_t)io->cfg_ch[0]
                                                      : _HDR_RISE_PERMILLE_DEFAULT;
        uint16_t drop_permille = (io->cfg_ch[1] != 0u) ? (uint16_t)io->cfg_ch[1]
                                                      : _HDR_DROP_PERMILLE_DEFAULT;
        uint16_t guard_permille = (io->cfg[0] != 0u) ? (uint16_t)io->cfg[0]
                                                     : _V4_GUARD_PERMILLE_DEFAULT;
        /* baseline(u16) * permille(u16) 最大约 4.29e9, 仍在 u32 内, 不会溢出。 */
        uint32_t rise  = _udiv1000((uint32_t)io->baseline * (uint32_t)rise_permille) + noise;
        uint32_t fall  = _udiv1000((uint32_t)io->baseline * (uint32_t)drop_permille) + noise;
        uint32_t guard = _udiv1000((uint32_t)io->baseline * (uint32_t)guard_permille);

        st->x_delta_rise = _clamp_u16((int32_t)rise);
        st->x_delta_fall = _clamp_u16((int32_t)fall);
        st->x_guard      = _clamp_u16((int32_t)guard);
        st->initialized    = 1u;
        st->forced_release = 0u;
        st->hist_valid     = 0u;
        st->z_h            = _HDR_INVALID_HIGH;
        st->z_l            = _HDR_INVALID_LOW;
        st->press_ref      = (int32_t)io->diff;
        st->z_ht           = now;
        st->z_lt           = now;
    }

    /* ---- ③ 三点中值滤波 + 逐通道基线偏移(cfg_ch[3], 按补码解释为 -128..127) ---- */
    d = (int32_t)io->diff;
    if (!st->hist_valid)
    {
        st->d0 = d; st->d1 = d; st->d2 = d;
        st->hist_valid = 1u;
    }
    else
    {
        st->d2 = st->d1; st->d1 = st->d0; st->d0 = d;
    }
    d = _median3(st->d0, st->d1, st->d2);
    d += (int32_t)(int8_t)io->cfg_ch[3];

    /* ---- ④ press_ref: 本次按下的稳态水平, 慢升快降 ----
     * 慢升(>>atk, 默认 1/32)是这次修复的关键: 掠过的手只造成几个周期的尖峰, 对 press_ref 的
     * 抬升可以忽略; 而真正"按得更重"是持续过程, 几十个周期后 press_ref 自然跟上。
     * 快降(>>dcy, 默认 1/4)保证手指一抬起 press_ref 就迅速跟着掉, 不会把释放判据顶死。
     * 全程只用移位与加减 —— 无除法、无 64 位, 符合 blob 的硬约束。 */
    {
        int32_t delta = d - st->press_ref;
        if (delta > 0)
        {
            int32_t step = delta >> atk;
            if (step == 0) { step = 1; }
            st->press_ref += step;
        }
        else if (delta < 0)
        {
            int32_t step = (-delta) >> dcy;
            if (step == 0) { step = 1; }
            st->press_ref -= step;
        }
    }

    if (st->forced_release)
    {
        /* ---- 已判松开: 追踪窗口低谷, 等一次足够的重新上冲才回到按下 ----
         * 这条分支与 v3.1 逐字同构(未改), 因为误松开发生在**激活→松开**方向, 不在这里。 */
        int32_t override_active = 0;

        if (((now - st->z_lt) > win) || (st->z_l == _HDR_INVALID_LOW))
        {
            st->z_l = d; st->z_lt = now;
        }
        else if (d < st->z_l)
        {
            st->z_l = d; st->z_lt = now;
        }

        if (((d - st->z_l) > (int32_t)st->x_delta_fall) && ((uint16_t)d > io->finger_th))
        {
            st->forced_release = 0u;
            override_active    = 1;
            st->z_h = d; st->z_l = d;
            st->z_ht = now; st->z_lt = now;
            st->press_ref = d;   /* 重新按下 ⇒ 稳态水平从此刻重新起算 */
        }
        io->out_active = (uint32_t)override_active;
    }
    else
    {
        int32_t override_active = 1;

        if ((st->z_h == _HDR_INVALID_HIGH) || (st->z_l == _HDR_INVALID_LOW))
        {
            st->z_h = d; st->z_l = d;
            st->z_ht = now; st->z_lt = now;
        }
        else
        {
            /* ★z_h 的上升被限速★ 窗口峰值原本是"见到多高就记多高", 于是一个瞬时尖峰能一次性
             * 把释放参考顶上去。限速后尖峰每周期最多抬高 slew 个 count, 参考被污染的幅度
             * 与尖峰高度脱钩 —— 这是从源头削掉误松开, 而不是事后再加一道门槛拦住它。
             * 窗口过期仍然直接重置为当前值(与 v3.1 一致), 所以不会积累历史包袱。 */
            if ((now - st->z_ht) > win)
            {
                st->z_h = d; st->z_ht = now;
            }
            else if (d > st->z_h)
            {
                int32_t room = d - st->z_h;
                if (room > (int32_t)slew) { room = (int32_t)slew; }
                st->z_h += room;
                st->z_ht = now;
            }

            if ((now - st->z_lt) > win)
            {
                st->z_l = d; st->z_lt = now;
            }
            else if (d < st->z_l)
            {
                st->z_l = d; st->z_lt = now;
            }
        }

        /* ---- 松开判据: (a) 相对回落够多  且  (b) 真的跌破了稳态水平  且  (c) 不是有手掠过 ----
         * 三条是相互独立的证据, 任一条不成立就继续保持按下。
         * (b) 是本版新增的核心: 掠过造成的尖峰过后 d 只是回到 ≈press_ref, 跌不破 press_ref-guard,
         *     于是 (b) 不成立 ⇒ 不松开; 真手指抬起时 d 会显著跌破 ⇒ (b) 成立 ⇒ 正常松开。
         * (c) 只在"确有其它通道同时上升"时短暂成立, 是掠过的直接物理特征; 真手指抬起时相邻
         *     通道是在下降, 不会触发, 所以它不会拖慢正常释放。 */
        if ((st->z_h - d) > (int32_t)st->x_delta_rise)
        {
            int32_t below = st->press_ref - (int32_t)st->x_guard;
            uint8_t fell_below_press = (d < below) ? 1u : 0u;

            if ((fell_below_press != 0u) && (brush_now == 0u))
            {
                st->forced_release = 1u;
                override_active    = 0;
                st->z_h = d; st->z_l = d;
                st->z_ht = now; st->z_lt = now;
            }
            /* 否则: 保持按下。这正是"被另一只手掠过时不再误松开"的落点。 */
        }

        /* ★绝对兜底★ 上面全是相对判据; 任何相对逻辑一旦出偏差都可能变成"永久按住", 那是比
         * 误松开严重得多的故障。故保留一条与动态范围无关的下限: 滤波后水平掉到 finger_th 的
         * 一半以下时无条件松开 —— 此时电极上不可能还压着手指。 */
        if (d < (int32_t)(io->finger_th >> 1u))
        {
            st->forced_release = 1u;
            override_active    = 0;
            st->z_h = d; st->z_l = d;
            st->z_ht = now; st->z_lt = now;
            st->press_ref = d;
        }

        io->out_active = (uint32_t)override_active;
    }

    /* ---- 上报: env_hi/press_ref 是两条判据各自的参考线, margin 是当前余量 ----
     * ★为什么把 v3.1 的 env_lo 换成 press_ref★ 只有 4 个上报槽, 而 press_ref 是本版新的决策
     * 变量 —— 调参时要看的就是"尖峰有没有把参考顶上去"。env_lo 只服务重新按下那条路径,
     * 且它恒等于窗口低谷, 从 diff 曲线上一眼可见, 占一个槽不值。 */
    io->report[1] = _clamp_u16(st->z_h);
    io->report[2] = _clamp_u16(st->press_ref);
    io->report[3] = _clamp_u16(st->forced_release ? (d - st->z_l) : (st->z_h - d));
}

/* ---- helper 实现(定义在 algo() 之后, 保证 algo() 是 .text 首个符号) ----
 * 全部不含 '/'、'%' 与 64 位运算, 也不产生任何外部符号。 */

/* 32 位恢复余数长除法实现的精确 /1000: 与 C 的 '/' 逐位一致(余数恒 < 2000, 不可能溢出)。 */
static inline uint32_t _udiv1000(uint32_t n)
{
    uint32_t q = 0u;
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

/* int32 → u16 的饱和转换(负值截 0, 超范围截 0xFFFF)。上报槽是 u16, 不能让它回绕。 */
static inline uint16_t _clamp_u16(int32_t v)
{
    if (v < 0) { return 0u; }
    if (v > 0xFFFF) { return 0xFFFFu; }
    return (uint16_t)v;
}

/* [] END OF FILE */
