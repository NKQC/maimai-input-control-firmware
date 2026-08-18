/* csd_autotune.c —— 频率自适应状态机, 见 csd_autotune.h。逐字自 main.c 搬入。 */
#include "csd_autotune.h"
#include "csd_params.h"
#include "diag_export.h"
#include "cybsp.h"
#include "cycfg_capsense.h"

/* AUTO_TUNE 指令置位：主循环执行频率自适应下探(逐档升 snsClk 分频重校准)。 */
volatile bool auto_tune_pending = false;
/* 自适应结果: 0=未执行/进行中, 1=成功(auto_tune_div 为找到的分频), 2=失败(超硬件上限仍压不到目标)。 */
volatile uint8_t auto_tune_result = 0u;
/* 自适应成功时找到的 snsClk 分频(已写回目标 widgetContext)。
 * ★全通道(0xFF)模式的完成态语义★: 各通道分频互不相同, 单一分频无意义, 故完成时本字段 = 成功通道数
 * (0..36); 真正的逐通道分频由 RP2040 完成后经 GET_PARAM(0x08) 逐通道回读写穿真相源。 */
volatile uint16_t auto_tune_div = 0u;
/* 本次自适应的目标通道: 0..35=单通道, LNK_CH_ALL=全通道。AUTO_TUNE_GET 回显该字节。 */
volatile uint8_t auto_tune_ch = LNK_CH_ALL;
/* 本次自适应的灵敏度偏好档位(1..7, 默认 4=居中): 落档时在临界分频上加 2*(pref-1)(往低频=更灵敏)。 */
volatile uint8_t auto_tune_pref = 4u;
/* ★阶段性进度★: 长自适应(最坏 ~20s)期间由主循环逐步更新, 经 GET_AUTO_TUNE 的 progress 字节上报,
 * 使 RP2040/上位机在过程中就能看到"到哪一步了", 而非只能干等最终结果。
 * phase: 0=空闲/已受理 1=粗定位 2=细搜临界 3=落档/回退 4=完成; step: 该阶段内步序(1 起, 上报饱和 31)。
 * SPI 是独立中断且优先级(2)高于 CapSense(3), 故校准阻塞期间 ISR 仍能应答这两个量。 */
volatile uint8_t auto_tune_phase = 0u;
volatile uint8_t auto_tune_step = 0u;
/* ★auto_tune_tag 整套机制已删除★ 它是 v1 里为"认出上一轮残留结果"而在 result 字节高 6 位手搓的
 * 私有标签。LINK v2 的链路 tag 已经在协议层做到了同一件事(且覆盖所有命令), 再留一份只会二义。 */
/* 细搜(1 步进上探临界)最大步数: 粗表最大相邻间隔为 48→64 的 16, 取 15 步即可覆盖整个区间;
 * 同时限制全通道模式下额外校准次数(单次全通道校准 ~200-270ms), 保证总耗时留在 RP2040 的 10s 窗内。 */
#define AUTO_TUNE_FINE_STEPS_MAX         (15u)
/* 频率自适应粗定位表(升序 = 逐档降频)。放在文件作用域: 逐通道串行状态机要跨轮次按下标续做。 */
static const uint16_t k_autotune_divs[] = { 8u, 12u, 16u, 20u, 24u, 32u, 40u, 48u, 64u };
#define AUTO_TUNE_DIV_COUNT              (sizeof(k_autotune_divs) / sizeof(k_autotune_divs[0]))

/* AUTO_TUNE 拆得最细: 一个通道内部本身就是 25 次左右的校准试探(5~7s), 只按通道拆仍会一步阻塞
 * 好几秒。故连通道内的三个阶段一起拆到"一轮一次试探"的粒度, 阶段码复用 AT_PH_*。 */
#define AT_PH_ENTER                      (1u)   /* 选通道 + 记录原分频 */
#define AT_PH_COARSE                     (2u)   /* 粗表逐档试探 */
#define AT_PH_ABORT                      (3u)   /* 粗定位全失败: 恢复原分频后重校准一次 */
#define AT_PH_FINE                       (4u)   /* 1 步进上探临界 */
#define AT_PH_SETTLE                     (5u)   /* 按 pref 落档 / 逐 1 回退 */
#define AT_PH_EDGE                       (6u)   /* 回退到临界档的最后一次校准 */
#define AT_PH_CHEND                      (7u)   /* 本通道收尾, 决定继续下一个还是整体结束 */
typedef struct
{
    bool     active;
    uint8_t  phase;
    uint8_t  ch;
    uint8_t  last;
    uint8_t  target;      /* 请求语义: 0..35 或 LNK_CH_ALL */
    uint8_t  pref;
    uint8_t  di;          /* 粗表下标 */
    uint8_t  step;        /* 阶段内步序(同时经 auto_tune_step 上报) */
    uint16_t saved_div;
    uint16_t div_edge;
    uint16_t div_target;
    uint16_t final_div;   /* 单通道: 最终分频; 全通道: 成功通道数(与 v2 的 div 字段语义一致) */
    bool     tuned;
} at_task_t;
static at_task_t _at_task;
/* 空闲相位码: 与 csd_ops.h 的 OP_PHASE_IDLE 同值(0), 单独给一个名字是为了不让本模块
 * 反向依赖 csd_ops.h(依赖方向只允许 csd_ops -> csd_autotune)。 */
#define AT_PH_IDLE                       (0u)

static inline void _at_task_clear(void)
{
    _at_task.active = false;
    _at_task.phase = AT_PH_IDLE;
    _at_task.ch = 0u;
    _at_task.last = 0u;
    _at_task.target = LNK_CH_ALL;
    _at_task.pref = 4u;
    _at_task.di = 0u;
    _at_task.step = 0u;
    _at_task.saved_div = 0u;
    _at_task.div_edge = 0u;
    _at_task.div_target = 0u;
    _at_task.final_div = 0u;
    _at_task.tuned = false;
}

/* _at_task 归本模块独占; 编排层只需要"它是否在跑"这一个只读事实。 */
bool at_active(void)
{
    return _at_task.active;
}


/* 自适应原子动作：把 snsClk 分频写进目标 widget 并就地校准一次, 返回该分频下 IDAC 能否把 raw
 * 校到目标容差内。粗定位/细搜/落档/回退四处复用, 避免复制粘贴。
 * ★恒为单通道判定★: 全通道模式也由外层逐通道调用本函数(不再有"写全部 widget 同一分频"的路径)。
 * ★IDAC 档口径★: 经 calibrate_widget_locked 校准, 故"该分频能否压到目标%"是在该通道自己的
 * 增益档(用户设过 PARAM_IDAC_GAIN 时)下判定的, 而不是全局起点档。 */
static inline bool auto_tune_try_div(uint8_t target_ch, uint16_t div)
{
    if (target_ch >= LNK_CHANNEL_COUNT) return false;
    cy_capsense_tuner.widgetContext[target_ch].snsClk = div;
    return calibrate_widget_locked((uint32_t)target_ch);
}

/* 自适应"打点 + 试探"：先把阶段/步序/当前试探分频发布(经 AUTO_TUNE_GET 上报, 长过程可见), 再执行
 * 一次试探校准。粗定位/细搜/落档/回退四处共用, 避免每处复制打点代码。
 * ★v3: 打点必须在试探【之前】★ 试探本身要阻塞 200~270ms, 而链路泵就在这段的前后各跑一次 ——
 * 先打点主机才能在这一步进行中就看到"到哪一步了", 而不是只能等这一步做完。 */
static inline bool auto_tune_probe(uint8_t target_ch, uint16_t div, uint8_t phase, uint32_t step)
{
    auto_tune_phase = phase;
    auto_tune_step  = (step > AUTO_TUNE_STEP_MAX_REPORT) ? (uint8_t)AUTO_TUNE_STEP_MAX_REPORT : (uint8_t)step;
    auto_tune_div   = div;   /* 进行中语义: div 字段 = 当前试探值(完成时被最终分频覆盖) */
    return auto_tune_try_div(target_ch, div);
}

/* ---- AUTO_TUNE ---- */

/* 本通道成功落档。单通道模式记最终分频, 全通道模式记成功通道数(与 v2 的 div 字段语义一致)。 */
static inline void _at_ch_ok(uint16_t div)
{
    _at_task.tuned = true;
    if (_at_task.target < LNK_CHANNEL_COUNT) { _at_task.final_div = div; }
    else { _at_task.final_div++; }
}

/* ③ 按偏好落档: 每档往低频让 2 个分频(充电更充分→过充近场效应→更灵敏)。 */
static inline void _at_enter_settle(void)
{
    uint16_t t = (uint16_t)(_at_task.div_edge + (2u * (uint16_t)(_at_task.pref - 1u)));
    if (t > 255u) { t = 255u; }   /* PSoC snsClk 合法范围 1..255 */
    _at_task.div_target = t;
    _at_task.step = 1u;
    _at_task.phase = AT_PH_SETTLE;
}

static void _at_finish(void)
{
    const uint8_t target = _at_task.target;
    if (target >= LNK_CHANNEL_COUNT) { auto_tune_ch = LNK_CH_ALL; }   /* 终态回显请求语义 */
    /* 逐档校准会改回起点档; 只恢复本次目标范围。扫描会自然装载该 widget 的新 gain,
     * 不调用全局 Initialize, 避免扰动其它 widget 的状态/基线。 */
    (void)idac_lock_restore(target);
    if (target < LNK_CHANNEL_COUNT)
    {
        if (ch_is_enabled((uint32_t)target))
        {
            Cy_CapSense_InitializeWidgetBaseline((uint32_t)target, &cy_capsense_context);
        }
    }
    else
    {
        initialize_enabled_baselines();
    }
    auto_tune_div    = _at_task.tuned ? _at_task.final_div : 0u;   /* 失败: div 无效 */
    auto_tune_phase  = AUTO_TUNE_PHASE_DONE;
    auto_tune_step   = 0u;
    auto_tune_result = _at_task.tuned ? 1u : 2u;   /* 1=成功(至少一个通道) 2=全部失败 */
    _at_task_clear();
}

/* 一次调用最多做一次 auto_tune_probe(= 一次校准, 约 200~270ms)。
 * ★为什么连通道内部也要拆★ 单通道三阶段合计约 25 次校准 = 5~7s, 只按通道拆的话一步就阻塞好几秒,
 * 主机在那期间既发不进也读不到 —— 那正是 v2 掉链路的场景。 */
void at_step(void)
{
    spi_dbg.stage = MLOOP_STAGE_AUTO_TUNE;
    switch (_at_task.phase)
    {
        case AT_PH_ENTER:
            /* 禁用通道不做频率自适应: 它要逐档连接电极重校准, 与"恒高阻"冲突。单通道模式落在禁用
             * 通道上就如实回失败(result=2), 不假装成功, 上位机据此提示"该通道已关闭"。 */
            while ((_at_task.ch < _at_task.last) && !ch_is_enabled(_at_task.ch)) { _at_task.ch++; }
            if (_at_task.ch >= _at_task.last) { _at_finish(); break; }
            /* 进度期间 auto_tune_ch 回显"当前正在处理的通道", 供上位机显示 CHn/36。 */
            auto_tune_ch = _at_task.ch;
            _at_task.saved_div = cy_capsense_tuner.widgetContext[_at_task.ch].snsClk;
            _at_task.div_edge = 0u;
            _at_task.di = 0u;
            _at_task.phase = AT_PH_COARSE;
            break;

        case AT_PH_COARSE:
            /* ① 粗定位: 粗表升序(升分频=降频)扫描, 首个校准成功的档位必然 >= 真实临界值。 */
            if (_at_task.di >= (uint8_t)AUTO_TUNE_DIV_COUNT)
            {
                /* 粗定位全失败时必须恢复原分频并尝试校准, 否则最后试探值会残留为坏状态。 */
                cy_capsense_tuner.widgetContext[_at_task.ch].snsClk = _at_task.saved_div;
                _at_task.phase = AT_PH_ABORT;
                break;
            }
            if (auto_tune_probe(_at_task.ch, k_autotune_divs[_at_task.di],
                                AUTO_TUNE_PHASE_COARSE, (uint32_t)_at_task.di + 1u))
            {
                _at_task.div_edge = k_autotune_divs[_at_task.di];
                _at_task.step = 0u;
                _at_task.phase = AT_PH_FINE;
            }
            else
            {
                _at_task.di++;
            }
            break;

        case AT_PH_ABORT:
            (void)calibrate_widget_locked(_at_task.ch);
            _at_task.phase = AT_PH_CHEND;   /* 本通道失败, 不计入成功数 */
            break;

        case AT_PH_FINE:
            /* ② 1 步进上探临界: 从 div_edge-1 起逐 1 降分频(升频)重校准, 成功即继续;
             *    首次失败即停, 最后一个成功值就是临界分频。 */
            if ((_at_task.step >= AUTO_TUNE_FINE_STEPS_MAX) || (_at_task.div_edge <= 1u))
            {
                _at_enter_settle();
                break;
            }
            if (auto_tune_probe(_at_task.ch, (uint16_t)(_at_task.div_edge - 1u),
                                AUTO_TUNE_PHASE_FINE, (uint32_t)_at_task.step + 1u))
            {
                _at_task.div_edge--;
                _at_task.step++;
            }
            else
            {
                _at_enter_settle();
            }
            break;

        case AT_PH_SETTLE:
            if (auto_tune_probe(_at_task.ch, _at_task.div_target,
                                AUTO_TUNE_PHASE_SETTLE, (uint32_t)_at_task.step))
            {
                _at_ch_ok(_at_task.div_target);
                _at_task.phase = AT_PH_CHEND;
                break;
            }
            _at_task.step++;
            /* 落档失败(该低频下 IDAC 反而压不住): 朝临界方向逐 1 回退重试;
             * 退到临界仍失败则在临界档重校准一次(已知可用)并采用它。 */
            if (_at_task.div_target <= _at_task.div_edge) { _at_task.phase = AT_PH_EDGE; }
            else { _at_task.div_target--; }
            break;

        case AT_PH_EDGE:
            (void)auto_tune_probe(_at_task.ch, _at_task.div_edge,
                                  AUTO_TUNE_PHASE_SETTLE, (uint32_t)_at_task.step);
            _at_ch_ok(_at_task.div_edge);
            _at_task.phase = AT_PH_CHEND;
            break;

        case AT_PH_CHEND:
        default:
            /* 单通道模式做完即整体结束; 全通道模式继续下一个(单通道失败不拖垮全场, 只影响该通道,
             * 其分频保持上一次的值)。 */
            if (_at_task.target < LNK_CHANNEL_COUNT) { _at_finish(); }
            else { _at_task.ch++; _at_task.phase = AT_PH_ENTER; }
            break;
    }
}

/* 主循环阶段: 受理 AUTO_TUNE(自 main() 逐字搬来, 原地的 continue 变成 return true)。 */
bool at_begin(void)
{

    /* AUTO_TUNE：频率自适应下探。高 Cp 电极在高频(小分频)下传感器来不及建立→IDAC 无论如何
     * 都压不到目标%→CalibrateWidget 返回非 SUCCESS。此处从高频起逐档升分频(降频),
     * 每档重校准, 用中间件校准返回状态判定该频率下 IDAC 能否满足目标; 首个成功的分频即为
     * "满足当前 IDAC 设置的最高频率(最快扫描)", 写回该 widgetContext 并记录; 全部档位都失败
     * = 超该通道硬件能力 → 该通道失败。运行时可调, 结果经 GET_AUTO_TUNE 上报。 */
    /* 通道 Cp 跨度极大(22pF~138pF), 单一统一分频无法兼顾 → 两种模式都按【单通道】判定:
     *   auto_tune_ch = 0..35: 只改该 widget 的 snsClk(其余通道不动), CalibrateWidget 判定;
     *   auto_tune_ch = 0xFF : 逐通道各自校准——外层遍历 36 通道, 每通道独立跑完整三步算法,
     *                         各得其自身分频; 不再要求"全通道共用同一分频且全部通过"。 */
    if (auto_tune_pending)
    {
        /* ★受理即转交状态机★ 单通道三阶段约 25 次校准(5~7s), 全通道分钟级 —— v2 就是在这
         * 段里把主循环整段占住, 主机的背靠背 pump 撞上 CS ISR 从而掉链路(见 §0 第 5 步)。
         * 拆分不改变通道顺序与每步参数: 见 at_step 逐条对照 v2 的 auto_tune_run_ch。 */
        const uint8_t target_ch = auto_tune_ch;
        auto_tune_pending = false;
        _at_task.active = true;
        _at_task.phase  = AT_PH_ENTER;
        _at_task.target = target_ch;
        _at_task.pref   = auto_tune_pref;
        _at_task.ch   = (target_ch < LNK_CHANNEL_COUNT) ? target_ch : 0u;
        _at_task.last = (target_ch < LNK_CHANNEL_COUNT)
                            ? (uint8_t)(target_ch + 1u) : (uint8_t)LNK_CHANNEL_COUNT;
        _at_task.di = 0u;
        _at_task.step = 0u;
        _at_task.saved_div = 0u;
        _at_task.div_edge = 0u;
        _at_task.div_target = 0u;
        _at_task.final_div = 0u;
        _at_task.tuned = false;
        return true;
    }
    return false;
}

/* 启动复位: 任务结构与上报量(原 main() 启动序列的对应段)。 */
void csd_autotune_reset(void)
{
    _at_task_clear();
    auto_tune_pending = false;
    auto_tune_result = 0u;
    auto_tune_div = 0u;
    auto_tune_ch = LNK_CH_ALL;
    auto_tune_phase = AUTO_TUNE_PHASE_IDLE;
    auto_tune_step = 0u;
}
