//! 主图、逻辑分析仪和延迟曲线绘图管线。
//!
//! 本模块拥有时间轴、量程、缺口虚线和颜色分配的唯一实现；
//! UI 回调只消费最终模型，不再参与坐标计算。

use super::*;

// ============================================================================
// 单通道精调主图: 唯一绘图管线
// ============================================================================
//
// ★不变量(改这块之前先读这段)★
// 「同一张图内所有系列共享唯一 x 定义域与唯一绘制面积; 任何新增系列只能加入本管线,
//   不得自带坐标计算」
//
// 旧实现把主曲线(CurvePaths)与算法叠加线(AlgoOverlay)做成两套产物, 后者自带 x 时间窗与量程,
// 而主曲线的 x 窗只由 raw/bsln/diff 求出。算法追踪点是主机轮询取回的, 时间范围与遥测缓冲并不
// 一致 ⇒ 落在主曲线时间窗之外的那一段被裁掉, 表现为"红线只画一半、左边一块空窗、线到那儿就
// 消失"。★根子不在坐标轴, 在两套产物各算一套"有效绘制面积"。★ 现在 x 定义域由【全部参与绘制
// 的系列】一次求并集算定后分发, 缺口阈值同样只在这一处给出: 某系列在某段无数据 ⇒ 那一段断线
// (gap), 而不是把整条线压缩到局部宽度。
//
// ★左右双 Y 轴是正当设计, 不要合并★: 左轴是 ADC 量纲(raw/bsln/diff 与阈值线), 右轴是算法量纲
// (计数/permille/布尔)。共用一轴会把 0/1 的布尔线压成贴底直线。统一的只有 x 定义域与绘制面积
// —— 两轴各有自己的 y 量程, 但横向对位逐像素一致。

/// ★同一张图内的唯一调色板★。容量 32 —— 当前图只用到 8 条(主曲线 3 + 算法上报 4 + 触发判定 1),
/// 留足余量是为了让"同图颜色永不重复"这条不变量在系列数增长时**不必再改代码**
/// (旧版容量恰好等于 8, 一加系列就立刻溢出到灰色降级)。
///
/// 前 8 项顺序与旧版逐项一致(蓝 raw / 琥珀 bsln / 绿 diff / 紫 / 青 / 近白 / 珊瑚 / 品红 触发判定),
/// 保持既有观感不变; 第 9 项起按黄金角(0.618)散布色相补足。
///
/// ★两条可复核的量化约束★(用脚本算过, 改表时请重算, 别凭眼睛加色):
///  - 最小两两 RGB 欧氏色距 = 55.3 —— 保证任意两条线不会被看成同色;
///  - 最低亮度(0.299R+0.587G+0.114B) = 112 —— 图底是 #1c2126(亮度约 32), 低于此值的色在深色底上看不清,
///    故所有色都设了亮度下限; 这也是为什么表里没有深蓝/深紫那类"色距够但看不见"的颜色。
const SERIES_PALETTE: [(u8, u8, u8); 32] = [
    (0x33, 0x99, 0xff), // 蓝(raw)
    (0xff, 0xaa, 0x00), // 琥珀(bsln)
    (0x33, 0xcc, 0x33), // 绿(diff)
    (0xb0, 0x8c, 0xff), // 紫
    (0x00, 0xd4, 0xc8), // 青
    (0xdf, 0xe8, 0xf0), // 近白
    (0xff, 0x7a, 0x45), // 珊瑚
    (0xff, 0x33, 0x99), // 品红(触发判定)
    (0xff, 0x84, 0x9e),
    (0xd5, 0xf2, 0x60),
    (0xc9, 0x2f, 0xd8),
    (0x84, 0xff, 0xd0),
    (0x65, 0x60, 0xf2),
    (0x19, 0xda, 0xff),
    (0x93, 0xd8, 0x2f),
    (0xff, 0x84, 0xf0),
    (0xf2, 0xae, 0x60),
    (0x8a, 0xff, 0x84),
    (0x60, 0xbd, 0xf2),
    (0xcc, 0xd8, 0x2f),
    (0x19, 0xff, 0x89),
    (0x19, 0xff, 0x3d),
    (0xf2, 0x60, 0x74),
    (0xfc, 0x19, 0xff),
    (0xff, 0xf8, 0x84),
    (0xff, 0x3b, 0x19),
    (0x6b, 0xff, 0x19),
    (0x2f, 0xd8, 0xaa),
    (0xff, 0xf9, 0x19),
    (0x60, 0xf2, 0x62),
    (0x60, 0xf2, 0xa7),
    (0xd5, 0x60, 0xf2),
];

/// 调色板耗尽后的降级色(灰): 明确表示"这条线没分到专色", 而不是静默与别人同色。
fn series_fallback_color() -> slint::Color {
    slint::Color::from_rgb_u8(0x9e, 0x9e, 0x9e)
}

/// 按"系列在图中的出现顺序"取色的游标。
/// ★递增游标而不是 `idx % len`★: 取模在系列数超过表长时会静默让两条线同色, 看图的人无法分辨
/// 谁是谁; 游标只增不减 ⇒ 同一张图内颜色永不重复, 超出容量的部分明确降级并由调用方记一条日志。
struct PaletteCursor {
    _next: usize,
    _overflow: usize,
}

impl PaletteCursor {
    fn new() -> Self {
        Self {
            _next: 0,
            _overflow: 0,
        }
    }
    /// 取下一个颜色。★分配顺序固定为图内声明顺序, 与该系列是否可见无关★ ——
    /// 否则勾掉一条线会让它后面所有线换颜色。
    fn take(&mut self) -> slint::Color {
        match SERIES_PALETTE.get(self._next) {
            Some(&(r, g, b)) => {
                self._next += 1;
                slint::Color::from_rgb_u8(r, g, b)
            }
            None => {
                self._overflow += 1;
                series_fallback_color()
            }
        }
    }
    /// 未分到专色的系列数(>0 ⇒ 调色板容量该扩了, 绝不靠重复颜色顶过去)。
    fn overflow(&self) -> usize {
        self._overflow
    }
}

/// 图内一条系列的身份: 决定它走哪根 Y 轴、按哪种采样节奏判缺口、回填到哪个 UI 属性。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SeriesId {
    Raw,
    Bsln,
    Diff,
    /// 算法上报 report[idx](idx 0..3)。
    Report(u8),
    /// 算法触发判定 out_active。
    Active,
}

impl SeriesId {
    /// true = 走右轴(算法量纲), false = 走左轴(ADC 量纲)。
    fn on_right_axis(self) -> bool {
        !matches!(self, SeriesId::Raw | SeriesId::Bsln | SeriesId::Diff)
    }
}

/// 采样间隔超过多少视为"时间缺口"(暂停/掉帧): 取标称周期的若干倍, 并给一个绝对下限,
/// 免得采样率未知(sps=0)或抖动时误判。缺口段按 0 绘制, 见 `fill_uncovered_with_zero`。
fn gap_threshold_us(sample_rate_hz: u32, period_mult: u32, floor_us: u64) -> u64 {
    let nominal = if sample_rate_hz > 0 {
        1_000_000 / sample_rate_hz as u64
    } else {
        33_333
    };
    (nominal * period_mult as u64).max(floor_us)
}

/// 整张图【唯一】的 x 定义域(展开后的设备时间 us)与【唯一】的缺口判据。
/// 由 `build_chart_frame` 在一处算定后分发给每条系列, 任何系列不得自己再算 x。
///
/// ★缺口阈值只有一档★: 算法上报值现在随遥测帧同帧到达(FIELD_ALGO), 与主曲线是同一个节奏,
/// 故所有系列共用同一个阈值。此前算法值靠 n 个 idx 轮转轮询取回, 单条线的间隔是遥测周期的 n 倍,
/// 才需要区分两档; 那条轮询路径已随随帧上报一并删除。
///
/// ★阈值的语义 = "覆盖半径"★: 与某个采样点时间距离在阈值内的时刻算被该采样覆盖(正常的帧间
/// 抖动/一个轮询周期都在此内); 超出即视为**没有数据**, 由 `fill_uncovered_with_zero` 按 0 补齐。
pub(crate) struct XWindow {
    /// 窗口左端时刻(= `t_last_us − PLOT_WINDOW_US`, 设备上电不足一窗时被 0 截住) → viewbox x = 0。
    t_first_us: u64,
    /// 最新样本时刻 → viewbox x = 1000。
    pub(crate) t_last_us: u64,
    /// 缺口阈值(每帧一点; 全部系列共用一档, 见结构体注释)。
    gap_frame_us: u64,
}

impl XWindow {
    /// ★整张图唯一的 x 映射★: 设备时刻 → viewbox x(0..1000)。所有系列必须经由它。
    /// x 值域固定 [0,1000]: 主图 PlotPath 用 fit: fill(非等比拉伸), viewbox 无论什么宽高比都被
    /// 拉满绘图区, 故不需要按绘图区宽高比预拉伸 —— 那套修正在横向缩放后必然失配并留出空白带。
    /// ★结果夹在 [0,1000]★ 与 `AxisScale::y_of` 同一处理。补齐后的系列本就恰好铺满
    /// [t_first_us, t_last_us], 夹取对它们是恒等变换; 它挡住的是退化窗口(整张图一条参与系列都
    /// 没数据 ⇒ 窗口塌成 0..0)下未参与系列被顺带投影时算出的天文数字坐标 —— 那会让下游的
    /// 虚线分段循环按 i32::MAX 跑。
    /// ★横向尺度恒为 `PLOT_WINDOW_US`★: 不用 `t_last - t_first` 当分母 —— 设备刚上电不足
    /// 30s 时 `t_first_us` 会被 0 截住, 那时按实际跨度换算又会变成"边跑边缩放"的动态尺度。
    /// 以右端为锚、固定跨度反推左端, 任何时刻 1 像素都等于同一时长。
    fn x_of(&self, t_us: u64) -> f32 {
        let span = PLOT_WINDOW_US as f32;
        let left = self.t_last_us as f32 - span;
        ((t_us as f32 - left) / span * 1000.0).clamp(0.0, 1000.0)
    }
    /// 全量时间跨度(ms) —— 固定值, 与 `x_of` 用的是同一个跨度。
    pub(crate) fn t_span_ms(&self) -> f32 {
        PLOT_WINDOW_US as f32 / 1000.0
    }
}

/// 一根 Y 轴的量程(已含两侧 5% 留白)。左轴 = ADC 量纲, 右轴 = 算法量纲。
pub(crate) struct AxisScale {
    pub(crate) min: f32,
    pub(crate) mid: f32,
    pub(crate) max: f32,
}

impl AxisScale {
    /// 无参与系列时的退化量程: 不能是 0 宽, 否则 y 映射除零。
    fn unit() -> Self {
        Self {
            min: 0.0,
            mid: 0.5,
            max: 1.0,
        }
    }
    /// 由参与系列的取值范围求量程。两侧各留 5%, 免得极值贴边看不出来。
    /// `flat_floor` = 全平序列(min==max)时的最小留白: 左轴取 1(ADC 最小有意义刻度是 1 个计数),
    /// 右轴取 0.5(算法量纲可以是 0/1 布尔, 再大就把一条平线撑到看不出高度差)。
    fn from_span(min: f32, max: f32, flat_floor: f32) -> Self {
        if !min.is_finite() || !max.is_finite() || min > max {
            return Self::unit();
        }
        let span = max - min;
        let padding = if span.abs() < f32::EPSILON {
            (min.abs() * 0.05).max(flat_floor)
        } else {
            span * 0.05
        };
        let lo = min - padding;
        let hi = max + padding;
        Self {
            min: lo,
            mid: (lo + hi) * 0.5,
            max: hi,
        }
    }
    /// 值 → viewbox y(0..1000, 向下增大)。
    fn y_of(&self, value: f32) -> f32 {
        let range = (self.max - self.min).max(f32::EPSILON);
        (1000.0 - (value - self.min) / range * 1000.0).clamp(0.0, 1000.0)
    }
}

/// 补齐后的一个绘图点。
///
/// `dashed = true` 表示该点是"此处无采样覆盖"合成出来的点: 取值沿用相邻真实采样的值(边缘保持),
/// 画成虚线。★不再合成 0 值★: 无覆盖是"不知道", 不是"设备报了 0"; 拿 0 去补会把该轴量程拉到 0,
/// 于是同一条线在有/无缺口时纵向尺度完全不同(实测: 进页面瞬间量程从 [490..542] 跳成 [0..542])。
#[derive(Clone, Copy)]
struct PlotPoint {
    t_us: u64,
    val: f32,
    dashed: bool,
}

/// 无覆盖段的"虚线"节拍(viewbox x 单位, 值域 0..1000)。
///
/// ★为什么用手工虚线而不是另开一条低透明度路径★: 另开路径要给 8 条系列各加一个 UI 元素与
/// 一份颜色, 等于把"一条系列一个 path"的模型撑成两份, 且 .slint 侧要再复制一遍描边参数。
/// 在同一条 path 里把无覆盖段拆成 on/off 小段, 颜色/线宽天然与本系列一致(仍取自 SERIES_PALETTE),
/// UI 一行都不用改, 而"虚线 = 这段没数据"与"实线 = 真实采样"一眼可分。
const DASH_ON: i32 = 9;
const DASH_OFF: i32 = 7;

/// 一条系列的最终产物。★除 path 外不带任何坐标信息★: 坐标全在 `XWindow`/`AxisScale` 里,
/// 谁也不能凭这个结构自己再投影一遍。
struct SeriesPath {
    id: SeriesId,
    path: String,
    /// 线条/图例颜色, 由本图唯一调色板顺序分配(见 `SERIES_PALETTE`)。
    color: slint::Color,
    /// 本系列的采样点数(0 ⇒ UI 可退化成基线, 或整行收起)。
    point_count: i32,
    /// 取值只有 0/1(布尔类) ⇒ UI 才为它显示"归一化幅度 N"输入。
    binary: bool,
}

/// 单通道精调主图一次回填的【唯一】产物: 一个函数一次产出全部系列。
///
/// ── 结构性不变量(由 `build_chart_frame` + `fill_uncovered_with_dashes` 联合保证) ──────────
/// 「同一张图内所有系列共享唯一 x 定义域(`ChartFrame::x`), 且每条**有采样**的系列都铺满整个
///  定义域: 无采样覆盖处沿用相邻采样的值并画成虚线, 既不补 0、不插值, 也不允许提前截断。」
///
/// 展开说明:
///  · 唯一 x: 只有 `XWindow::x_of` 一处做时间→横坐标映射, 任何系列不得自带 x 计算;
///  · 铺满定义域: 左端(早于首样本)、中间缺口、右端(晚于末样本)三种无覆盖区间全部补成虚线段。
///    ★这是"各条线能画的范围不一样"的唯一修法★ —— 遥测缓冲与算法追踪缓冲的起止时刻天然不同
///    (后者只在本页驻留时才轮询), 若各画各的, 缩放/切子标签后就会看到一条只占左半、一条只占
///    右半(实测截图); 现在每条线的 x 覆盖恒等于窗口本身, 差异只体现为实线/虚线;
///  · 虚线取边缘值而不是 0: 无覆盖是"不知道"。补 0 会把该轴量程拉到 0, 让同一条波形的纵向尺度
///    随缺口有无而变(那正是"归一化"要消掉的抖动);
///  · 垂直阶跃: 虚线段与真实数据之间用同一时刻的两个点表达跳变, 绝不画斜线 —— 斜线会伪造
///    "数值在这段时间里连续过渡"的假象;
///  · 不提前截断: 每条系列路径的 x 覆盖与 `x` 一致, 不会出现"叠加线只盖住图的左半边";
///  · 虚实可分: 无覆盖段是同色虚线(见 `DASH_ON`), 真实采样一律实线 —— 包括真值 0。
/// 空系列(该系列一个采样都没有)例外: 它不参与绘制, path 为空 —— 无数据的系列不该凭空多出一条
/// 贴底的线, 它的"没有数据"由 `point_count == 0` 表达, UI 据此收起整行。
pub(crate) struct ChartFrame {
    /// 唯一 x 定义域 + 唯一缺口判据。
    pub(crate) x: XWindow,
    /// 左轴(ADC 量纲): raw/bsln/diff 与阈值线共用。
    pub(crate) left: AxisScale,
    /// 右轴(算法量纲): 算法上报线与触发判定共用。
    pub(crate) right: AxisScale,
    /// 全部系列, 顺序 = 图内声明顺序 = 调色板分配顺序。
    series: Vec<SeriesPath>,
    /// 左轴系列的最大采样点数: UI 的"有无遥测数据"判据。
    pub(crate) point_count: i32,
    /// 当前通道窗口内 RAW 的真实样本数，不受主曲线勾选或显示平滑影响，供带宽诊断使用。
    pub(crate) channel_point_count: i32,
}

impl ChartFrame {
    fn _find(&self, id: SeriesId) -> Option<&SeriesPath> {
        self.series.iter().find(|s| s.id == id)
    }
    pub(crate) fn path_of(&self, id: SeriesId) -> &str {
        self._find(id).map_or("", |s| s.path.as_str())
    }
    pub(crate) fn color_of(&self, id: SeriesId) -> slint::Color {
        self._find(id)
            .map_or_else(series_fallback_color, |s| s.color)
    }
    pub(crate) fn count_of(&self, id: SeriesId) -> i32 {
        self._find(id).map_or(0, |s| s.point_count)
    }
    pub(crate) fn binary_of(&self, id: SeriesId) -> bool {
        self._find(id).map_or(false, |s| s.binary)
    }
    /// 左轴值 → viewbox y, 供阈值线定位。
    /// ★无遥测数据时回中线★: 此时左轴是退化的单位量程(0..1), 按它映射会把阈值线贴到顶边,
    /// 看上去像"阈值恰好等于量程上限"。UI 侧此时本就隐藏阈值线, 回中线只是不留下误导性坐标。
    pub(crate) fn left_value_to_y(&self, value: f32) -> f32 {
        if self.point_count == 0 {
            return 500.0;
        }
        self.left.y_of(value)
    }
}

/// 把一条系列铺满整个 `XWindow` 定义域: 无采样覆盖处沿用相邻采样的值并标成虚线。
///
/// ★唯一实现点★ "哪段没数据"只在这里判定, `points_to_svg_path` 只管画、不再判缺口;
/// `XWindow::gap_us` 仍是唯一的缺口判据来源(作为"覆盖半径", 由调用方按系列节奏取好传进来)。
///
/// 三种无覆盖区间的处理(全部产出 `dashed = true` 的点, 取值 = 紧邻的那个真实采样值):
///  · 左端: 首样本晚于 `t_first_us` 超出阈值 ⇒ 从 `t_first_us` 起按首样本值画虚线到首样本;
///  · 中间: 相邻样本间隔超阈值 ⇒ 在前一点处按前值平推虚线到后一点, 再垂直阶跃到后值;
///  · 右端: 末样本早于 `t_last_us` 超出阈值 ⇒ 从末样本按末值平推虚线到 `t_last_us`。
/// 差距在阈值内时不画虚线, 而是把边缘值以实线推到窗口边界: 那段时间**确实**被该采样覆盖
/// (不足一个采样周期), 画虚线会让最新的一条线每帧都闪出一小截。
///
/// ★为什么必须铺满★ 三条主曲线来自遥测环形缓冲、四条上报线与触发判定来自算法追踪缓冲, 两者的
/// 起止时刻天然不同(后者只在本页驻留时轮询)。不铺满就会出现"一条线只画左半边、另一条只画右半边"
/// (缩放或切子标签后尤其明显)。铺满后所有线的 x 覆盖恒等于窗口, 差异只体现为实线/虚线。
///
/// 空输入返回空(该系列一个采样都没有 ⇒ 不参与绘制, 不能凭空造一条线)。
fn fill_uncovered_with_dashes(points: &[(u64, f32)], x: &XWindow, gap_us: u64) -> Vec<PlotPoint> {
    let (Some(&(first_t, first_v)), Some(&(last_t, last_v))) = (points.first(), points.last())
    else {
        return Vec::new();
    };
    // 每个缺口最多插 2 点, 首尾各最多 2 点。
    let mut out: Vec<PlotPoint> = Vec::with_capacity(points.len() + 4);
    // 无覆盖段的合成点: 值沿用给定的边缘值(不是 0), 标成虚线。
    let hold_at = |t_us: u64, val: f32| PlotPoint {
        t_us,
        val,
        dashed: true,
    };

    if first_t > x.t_first_us {
        if first_t - x.t_first_us > gap_us {
            // 两个同值虚线点 ⇒ 画笔从左边界一路虚线推到首样本(见 points_to_svg_path 的判据)。
            out.push(hold_at(x.t_first_us, first_v));
            out.push(hold_at(first_t, first_v));
        } else {
            out.push(PlotPoint {
                t_us: x.t_first_us,
                val: first_v,
                dashed: false,
            });
        }
    }

    let mut prev: Option<(u64, f32)> = None;
    for &(t, v) in points {
        if let Some((pt, pv)) = prev {
            if t.saturating_sub(pt) > gap_us {
                out.push(hold_at(pt, pv));
                out.push(hold_at(t, pv));
            }
        }
        out.push(PlotPoint {
            t_us: t,
            val: v,
            dashed: false,
        });
        prev = Some((t, v));
    }

    if x.t_last_us > last_t {
        if x.t_last_us - last_t > gap_us {
            out.push(hold_at(last_t, last_v));
            out.push(hold_at(x.t_last_us, last_v));
        } else {
            out.push(PlotPoint {
                t_us: x.t_last_us,
                val: last_v,
                dashed: false,
            });
        }
    }
    out
}

/// 在同一条 path 里把一段水平的"无覆盖段"画成虚线, 并把画笔停在段末(保证后续垂直阶跃起点正确)。
fn push_dashes(path: &mut String, x_from: i32, x_to: i32, y: i32) {
    use std::fmt::Write;
    let mut cursor = x_from;
    while cursor < x_to {
        let on_end = (cursor + DASH_ON).min(x_to);
        // ★write! 而非 push_str(&format!())★: 后者每次都要先堆分配一个临时 String。
        // 一条 1024 点的曲线 × 多条系列 = 数千次分配, 实测正是主图 12ms 的主要来源。
        let _ = write!(path, " M {} {} L {} {}", cursor, y, on_end, y);
        cursor = on_end + DASH_OFF;
    }
    let _ = write!(path, " M {} {}", x_to, y);
}

/// 把补齐后的绘图点投影成 SVG path。
///
/// ★x 只走 XWindow★: 横坐标按时间而非序号 —— 掉帧/暂停时等间距序号会把时间轴画错(同样的像素
/// 距离代表不同时长); 且整张图共用同一个 XWindow, 任何系列都不得自带 x 计算。
/// ★不再有"断线"★: 缺口已由 `fill_uncovered_with_dashes` 补成显式的虚线段, 于是这里只需逐点连线;
/// 无数据不再表现为"没有线"(那与"线被裁到框外"无法区分), 而是一段明确的虚线。
/// ★无覆盖段画虚线★: 两端都是合成点且同高的水平段 ⇒ 拆成 on/off 小段(同色, 见 `DASH_ON`),
/// 与真实采样的实线区分开; 进出虚线段的垂直阶跃仍画实线, 阶跃本身必须清晰可见。
fn points_to_svg_path(points: &[PlotPoint], x: &XWindow, axis: &AxisScale) -> String {
    use std::fmt::Write;
    if points.is_empty() {
        return String::new();
    }
    // 预留容量: 每点约 12 字节 ("&nbsp;L 1000 1000")。一次分配好过边写边搬。
    let mut path = String::with_capacity(points.len() * 12 + 16);
    let mut prev: Option<(i32, i32, bool)> = None;
    for p in points {
        let px = x.x_of(p.t_us) as i32;
        let py = axis.y_of(p.val) as i32;
        match prev {
            None => {
                let _ = write!(path, "M {} {}", px, py);
            }
            Some((ppx, ppy, pz)) => {
                if pz && p.dashed && py == ppy {
                    push_dashes(&mut path, ppx, px, py);
                } else {
                    // ★同像素点直接丢弃★ viewbox 只有 1000×1000 个整数坐标, 而缓冲有 1024 点/通道:
                    // 落到同一像素的点画出来完全看不见, 却要各占一段 path 命令并让 Slint 多解析一次。
                    // 这不是抽稀采样(不丢任何可见形状), 只是不再重复画同一个点。
                    if px == ppx && py == ppy {
                        continue;
                    }
                    let _ = write!(path, " L {} {}", px, py);
                }
            }
        }
        prev = Some((px, py, p.dashed));
    }
    path
}

/// 判定二值序列: 非空且全部取值只有 0/1。0/1 在共享右轴上几乎不可见, 需要拉伸到 0..N。
/// ★只看真实采样★: 在归零补齐**之前**判定, 否则补出来的 0 会把非二值线也算成二值。
fn points_are_binary(points: &[(u64, f32)]) -> bool {
    !points.is_empty() && points.iter().all(|&(_, v)| v == 0.0 || v == 1.0)
}

/// 二值线按幅度 N 拉伸(纯显示变换, 不下发设备); 非二值线原样返回。
fn normalize_binary(points: &[(u64, f32)], amp: f32) -> Vec<(u64, f32)> {
    points.iter().map(|&(t, v)| (t, v * amp)).collect()
}

/// 求一组序列的取值范围(忽略非有限值)。空 ⇒ 返回 (+inf, -inf),
/// 由 `AxisScale::from_span` 退化成单位量程。
///
/// ★入参是"铺满后"的点, 但量程不会因缺口而变★: 虚线段取的是相邻真实采样的值, 不引入新极值,
/// 于是同一段波形无论有没有缺口, 纵向尺度都一样(这正是"归一化绘制"要保证的另一半)。
/// 反例(旧实现)是用 0 补缺口: RAW 常在 2500..4095, 一旦掺进 0 就把整条线压成贴顶的一条细带,
/// 几十个计数的变化就此看不见 —— 而且只在"恰好有缺口"时发生, 表现为纵轴莫名跳变。
fn value_span(series: &[&[PlotPoint]]) -> (f32, f32) {
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for points in series {
        for p in *points {
            if p.val.is_finite() {
                lo = lo.min(p.val);
                hi = hi.max(p.val);
            }
        }
    }
    (lo, hi)
}

/// 一张图里"哪些系列参与绘制"。
/// ★参与 = 既画线、也算进唯一 x 定义域与它那根轴的量程★: 没画出来的线不该影响别人的位置,
/// 否则勾掉一条线其余线会莫名其妙地跳。
#[derive(Debug, Clone, Copy)]
pub(crate) struct SeriesShow {
    pub(crate) raw: bool,
    pub(crate) bsln: bool,
    pub(crate) diff: bool,
    /// 4 条算法上报线各自的勾选(调用方已并入"叠加总开关 + 算法已声明该 idx"的判定)。
    pub(crate) report: [bool; 4],
    pub(crate) active: bool,
}

fn smooth_telem_points(points: &[(u64, f32)], gap_us: u64) -> Vec<(u64, f32)> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut out = Vec::with_capacity(points.len());
    for (idx, &(t, value)) in points.iter().enumerate() {
        let smoothed = match (idx.checked_sub(1), points.get(idx + 1)) {
            (Some(prev_idx), Some(&(next_t, next_value))) => {
                let (prev_t, prev_value) = points[prev_idx];
                if t.saturating_sub(prev_t) <= gap_us && next_t.saturating_sub(t) <= gap_us {
                    (prev_value + value + next_value) / 3.0
                } else {
                    value
                }
            }
            _ => value,
        };
        out.push((t, smoothed));
    }
    out
}

/// 主图一次回填: ★唯一入口★, 一次产出全部系列。
/// `amps[0..3]` 对应 report[idx] 的二值归一化幅度, `amps[4]` 对应触发判定。

pub(crate) fn build_chart_frame(
    ctrl: &AppController,
    ch: u8,
    show: SeriesShow,
    amps: &[f32; 5],
) -> ChartFrame {
    // ---- 1. 取点 ----
    // 左轴三条按勾选取(不画就不取); 右轴五条一律取: 行模型要靠"有无数据 / 是否二值"决定整行
    // 是否收起、是否显示归一化输入, 那与"画不画"是两件事(勾掉一条线不该让它的输入框消失)。
    let mut channel_raw_pts = ctrl.telem_points(ch, FIELD_RAW);
    let mut left_pts: [Vec<(u64, f32)>; 3] = [
        if show.raw {
            ctrl.telem_points(ch, FIELD_RAW)
        } else {
            Vec::new()
        },
        if show.bsln {
            ctrl.telem_points(ch, FIELD_BASELINE)
        } else {
            Vec::new()
        },
        if show.diff {
            ctrl.telem_points(ch, FIELD_DIFF)
        } else {
            Vec::new()
        },
    ];
    let mut report_pts: [Vec<(u64, f32)>; 4] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    let mut report_binary = [false; 4];
    for idx in 0..4usize {
        let raw = ctrl.algo_trace_report_points(idx as u8);
        // ★二值性优先取声明★: `ALGO_REPORT_META(idx,name,type,min,max,...)` 的 type/范围就是
        // 已知事实, 声明一到手(C 源回读完成)就该定下来。原先一律按观测数据反推, 于是声明为 u16
        // 的槽在数据到齐前恰好全是 0/1 ⇒ "归一化幅度"输入框先冒出来、真数据一到又消失。
        // 只有旧式 `ALGO_REPORT(idx,name)`(压根没带类型)才退回反推。
        // 反推必须在归一化【之前】做: 乘上 N 之后取值不再是 0/1, 输入框会自己消失。
        // ★判定与"是否参与绘制"无关★: 归一化输入框对未勾选的行也要照常出现, 故这一步不能跳。
        report_binary[idx] = ctrl
            .algo_report_declared_binary(idx as u8)
            .unwrap_or_else(|| points_are_binary(&raw));
        // 未参与绘制的系列到此为止: 归一化/归零补齐/path 生成都是纯浪费(路径不会被画),
        // 而它们正是本页每帧最重的一段 —— 8 条线里通常只有 1~2 条真的开着。
        if !show.report[idx] {
            continue;
        }
        report_pts[idx] = if report_binary[idx] {
            normalize_binary(&raw, amps[idx])
        } else {
            raw
        };
    }
    // ★触发判定改用遥测 STATUS(与 raw/bsln/diff 同帧同时间戳)★
    // 原先取 algo_trace_active_points(), 即 ALGO_GET_TRACE 独立轮询回来的 out_active。那条路
    // 有三个致命弱点(实测表现为触发判定恒不动):
    //   · 它只在算法**声明了 report 项**时才轮询(idx 轮转集合为空就整条不发);
    //   · 它与遥测不同源、频率低且会因 NAK 退避, 时间戳与主曲线错位;
    //   · 它拿的是算法中间量, 而用户要的是"设备真实确定的触发判定"。
    // 遥测 STATUS 的 bit0 由固件填 touch_mask(PSoC 最终判定, 含算法改判; 无算法时即内部基线差
    // 判定), 与主曲线同一帧到达 ⇒ 天然对齐、无需额外轮询、不受算法是否声明 report 影响。
    let active_raw = ctrl.telem_points(ch, mai2control_ui::proto::FIELD_STATUS);
    let active_binary = points_are_binary(&active_raw);
    let mut active_pts = if show.active {
        normalize_binary(&active_raw, amps[4])
    } else {
        Vec::new()
    };

    // ---- 2. ★唯一的 x 定义域★: 整张图只在这里算一次 ----
    // 左端由**主曲线(raw/bsln/diff)**定义, 叠加系列只能把右端往新的方向延伸, 不能把左端往旧的方向拉。
    //
    // ★为什么不是简单求并集★(这是实测踩到的坑)
    // 算法上报/触发判定只在"单通道精调"页驻留时才轮询, 而遥测在整个设置页都在推。去"算法"页待一会儿
    // 再回来: 上报缓冲停在离开那一刻、且其最老样本可能比遥测环的最老样本更早(遥测环持续丢弃旧样本),
    // 于是并集的左端被钉在一个陈旧时间戳上, 主曲线被挤到画面右侧 —— 用户看到的就是"只有右半边能看"。
    // 图的时间轴本质上就是遥测缓冲的时间轴("全量 x.xx s"说的是它), 比它更老的叠加样本已在窗口之外,
    // 不该有资格撑开坐标轴。
    // 反方向的老 bug(叠加线只画了一半宽)也仍然被覆盖: 叠加系列**不会**把窗口收窄, 只是不再撑左端。
    let mut t_first = u64::MAX;
    let mut t_last = 0u64;
    {
        for points in &left_pts {
            if let (Some(first), Some(last)) = (points.first(), points.last()) {
                t_first = t_first.min(first.0);
                t_last = t_last.max(last.0);
            }
        }
        let primary_has_data = t_first <= t_last;
        let mut extend_right = |points: &[(u64, f32)]| {
            if let (Some(first), Some(last)) = (points.first(), points.last()) {
                // 主曲线没有数据时(例如停流后只剩上报缓冲)才允许叠加系列定义左端, 否则整张图无从落笔。
                if !primary_has_data {
                    t_first = t_first.min(first.0);
                }
                t_last = t_last.max(last.0);
            }
        };
        for idx in 0..4usize {
            if show.report[idx] {
                extend_right(&report_pts[idx][..]);
            }
        }
        if show.active {
            extend_right(&active_pts[..]);
        }
    }
    // 一条参与系列都没有数据 ⇒ 空窗归零(不能留 u64::MAX, 那会让 x 映射全部落到 0)。
    let no_data = t_first > t_last;
    // ★横轴总跨度固定 30s★ 右端 = 最新样本时刻, 左端 = 右端 − 30s。
    // 为什么不再用"缓冲里最老那个样本"当左端: 那个值随缓冲填充、采样率变化、通道切换而漂,
    // 于是同一段波形在图上的横向尺度每帧都在变(抖动), 缩放/滚动条的比例也跟着变 ——
    // 固定窗口后横向 1 像素恒等于固定时长, 波形形状才可比。
    // 数据不足 30s 时左边就是空的(不补 0, 见 fill_uncovered_with_zero 的左端注释)。
    let t_first = t_last.saturating_sub(PLOT_WINDOW_US);
    let x = XWindow {
        t_first_us: if no_data { 0 } else { t_first },
        t_last_us: if no_data { 0 } else { t_last },
        // 遥测帧缺口: 超过 5 个标称周期(且至少 250ms)才算缺口, 容忍正常的帧间抖动。
        gap_frame_us: gap_threshold_us(ctrl.telem_samples_per_sec(), 5, 250_000),
    };

    // ---- 3. ★归零补齐★: 每条有采样的系列在 x 的整个定义域上处处有定义(见 ChartFrame 不变量)
    // 阈值按系列节奏取(两档都由 XWindow 一处算定), 补齐结果同时是后面量程与路径的唯一输入 ——
    // 量程用原始点、路径用补齐点会让归零段落在量程外, 那正是"归零段被裁到边框外"的来路。
    // 节奏仍由系列身份决定(SeriesId::cadence 是那条映射的唯一实现), 这里只是把两档各取一次。
    // ★窗口外的样本必须先丢掉(所有系列, 含主曲线)★
    // 窗口左端由固定跨度定义, 比它更老的样本仍留在各自缓冲里。若不丢:
    //  · `XWindow::x_of` 会把它们统统夹到 x=0, 在左边框上糊成一条竖线;
    //  · 它们还会参与量程, 把量程撑成陈旧数据的范围。
    // 丢弃只影响"画不进这张图的历史点", 不改变任何仍在窗口内的数据。
    {
        let cut = x.t_first_us;
        for pts in left_pts.iter_mut() {
            pts.retain(|(t, _)| *t >= cut);
        }
        channel_raw_pts.retain(|(t, _)| *t >= cut);
        for pts in report_pts.iter_mut() {
            pts.retain(|(t, _)| *t >= cut);
        }
        active_pts.retain(|(t, _)| *t >= cut);
    }

    // 全部系列同一节奏(算法值随帧到达), 故只有一档缺口阈值。
    let gap_frame = x.gap_frame_us;
    let channel_point_count = channel_raw_pts.len() as i32;
    // 主曲线只在相邻真实设备时间点连续时做三点均值；跨过停流/丢样缺口时保留原值，
    // 算法线则保持设备原始报告，尤其不能把二值触发边沿平滑成中间态。
    for points in left_pts.iter_mut() {
        *points = smooth_telem_points(points, gap_frame);
    }
    let left_fill: [Vec<PlotPoint>; 3] = [
        fill_uncovered_with_dashes(&left_pts[0][..], &x, gap_frame),
        fill_uncovered_with_dashes(&left_pts[1][..], &x, gap_frame),
        fill_uncovered_with_dashes(&left_pts[2][..], &x, gap_frame),
    ];
    let report_fill: [Vec<PlotPoint>; 4] = [
        fill_uncovered_with_dashes(&report_pts[0][..], &x, gap_frame),
        fill_uncovered_with_dashes(&report_pts[1][..], &x, gap_frame),
        fill_uncovered_with_dashes(&report_pts[2][..], &x, gap_frame),
        fill_uncovered_with_dashes(&report_pts[3][..], &x, gap_frame),
    ];
    let active_fill = fill_uncovered_with_dashes(&active_pts[..], &x, gap_frame);

    // ---- 4. 两根轴的量程: 各自只由"参与绘制且走该轴"的系列决定 ----
    let left = {
        let (lo, hi) = value_span(&[&left_fill[0][..], &left_fill[1][..], &left_fill[2][..]]);
        AxisScale::from_span(lo, hi, 1.0)
    };
    let right = {
        let mut participants: Vec<&[PlotPoint]> = Vec::with_capacity(5);
        for idx in 0..4usize {
            if show.report[idx] {
                participants.push(&report_fill[idx][..]);
            }
        }
        if show.active {
            participants.push(&active_fill[..]);
        }
        let (lo, hi) = value_span(&participants);
        AxisScale::from_span(lo, hi, 0.5)
    };

    // ---- 5. 生成路径: 同一个 x 映射 + 各自所属的轴。point_count 仍是**真实采样数**
    // (补齐点不是数据, 计进去会让"有无遥测数据"的判据永远为真)。
    let entries: [(SeriesId, &[PlotPoint], bool, usize); 8] = [
        (SeriesId::Raw, &left_fill[0][..], false, left_pts[0].len()),
        (SeriesId::Bsln, &left_fill[1][..], false, left_pts[1].len()),
        (SeriesId::Diff, &left_fill[2][..], false, left_pts[2].len()),
        (
            SeriesId::Report(0),
            &report_fill[0][..],
            report_binary[0],
            report_pts[0].len(),
        ),
        (
            SeriesId::Report(1),
            &report_fill[1][..],
            report_binary[1],
            report_pts[1].len(),
        ),
        (
            SeriesId::Report(2),
            &report_fill[2][..],
            report_binary[2],
            report_pts[2].len(),
        ),
        (
            SeriesId::Report(3),
            &report_fill[3][..],
            report_binary[3],
            report_pts[3].len(),
        ),
        (
            SeriesId::Active,
            &active_fill[..],
            active_binary,
            active_pts.len(),
        ),
    ];
    let mut palette = PaletteCursor::new();
    let series: Vec<SeriesPath> = entries
        .iter()
        .map(|&(id, points, binary, raw_count)| SeriesPath {
            id,
            path: points_to_svg_path(points, &x, if id.on_right_axis() { &right } else { &left }),
            color: palette.take(),
            point_count: raw_count as i32,
            binary,
        })
        .collect();
    if palette.overflow() > 0 {
        log::warn!(
            "主图系列数超出调色板容量({} 色): {} 条线只能用降级灰, 请扩充 SERIES_PALETTE — \
             绝不靠重复颜色顶过去(两条同色线等于看不出谁是谁)。",
            SERIES_PALETTE.len(),
            palette.overflow()
        );
    }

    ChartFrame {
        x,
        left,
        right,
        series,
        // "有无遥测数据"只看左轴三条: 算法追踪即使有点, 也不代表遥测在流。
        point_count: left_pts.iter().map(|p| p.len()).max().unwrap_or(0) as i32,
        channel_point_count,
    }
}

/// 设备端运行时刻(展开后)的人读文本: 供"最新样本的绝对时刻"这一行展示。
/// UI 侧的刻度/读数一律用"相对最新样本"的 ms 偏移(f32 精度足够), 绝对时刻只在这里出现一次。
pub(crate) fn dev_time_text(t_us: u64) -> String {
    let total_ms = t_us / 1000;
    let ms = total_ms % 1000;
    let total_s = total_ms / 1000;
    format!(
        "{:02}:{:02}:{:02}.{:03}",
        total_s / 3600,
        (total_s / 60) % 60,
        total_s % 60,
        ms
    )
}

fn series_to_svg_path(series: &[f32], min: f32, max: f32, x_scale: f32) -> String {
    if series.is_empty() {
        return String::new();
    }

    let n = series.len();
    let range = (max - min).max(f32::EPSILON);
    let mut path = String::new();

    for (i, &v) in series.iter().enumerate() {
        // x_scale = 绘图区宽高比: 把曲线横向拉伸到 [0, 1000*aspect], 配合 Slint 侧
        // viewbox 宽高比锁定, 使 contain 缩放正好铺满(1.17 Path 无 image-fit)。
        let x = (i as f32) * 1000.0 / ((n - 1).max(1) as f32) * x_scale;
        let y = (1000.0 - (v - min) / range * 1000.0).clamp(0.0, 1000.0);

        if i == 0 {
            path.push_str(&format!("M {} {}", x as i32, y as i32));
        } else {
            path.push_str(&format!(" L {} {}", x as i32, y as i32));
        }
    }

    path
}

/// 逻辑分析仪可选时间窗(微秒)。最小 500us 用于看清机械抖动的微秒级间隔,
/// 最大 5s 用于看整段按压序列。索引与 UI 下拉一一对应。
pub(crate) const LA_WINDOWS_US: [u32; 7] =
    [500, 2_000, 10_000, 50_000, 200_000, 1_000_000, 5_000_000];
/// 默认 50ms: 一次按下的抖动全景 + 3ms 级防抖窗仍清晰可辨。
pub(crate) const LA_WINDOW_DEFAULT: usize = 3;

/// 把微秒数格式化成可读时间(自动切 us/ms/s), 供逻辑分析仪的横轴刻度与触发读数使用。
pub(crate) fn fmt_time_us(us: f64) -> String {
    let a = us.abs();
    if a < 1000.0 {
        format!("{:.1}us", us)
    } else if a < 1_000_000.0 {
        format!("{:.3}ms", us / 1000.0)
    } else {
        format!("{:.3}s", us / 1_000_000.0)
    }
}

/// 逻辑分析仪一屏视图: 12 通道阶梯波形(去抖前/仅去抖后/实际输出各一条)+ 每通道触发标记 + 横轴刻度。
pub(crate) struct LogicAnalyzerView {
    pub(crate) raw_paths: Vec<slint::SharedString>,
    pub(crate) deb_paths: Vec<slint::SharedString>,
    pub(crate) out_paths: Vec<slint::SharedString>,
    pub(crate) trig_text: Vec<slint::SharedString>,
    pub(crate) trig_x: Vec<f32>,
    pub(crate) time_labels: Vec<slint::SharedString>,
    pub(crate) span_text: slint::SharedString,
}

/// 从边沿记录构建 12 通道时序图。
///
/// - 窗口右缘 = 缓冲里最新一条记录的时间戳, 向左展开 `window_us`; 全部通道共用这一条时间轴,
///   所以泳道之间的触发标记 x 可以直接横向比较(这正是"逻辑分析仪"的意义)。
/// - 设备时间戳是 `time_us_32()`, 32 位会回绕: 一律用 `wrapping_sub` 求相对偏移,
///   窗口外的旧记录其无符号差会变成巨大值, 天然被 `> window_us` 过滤掉。
/// - 窗口起点的电平取"窗口之前最后一条记录"的状态, 否则每次滚动窗口都会凭空多出一个假边沿。
pub(crate) fn build_logic_analyzer(
    edges: &std::collections::VecDeque<mai2control_ui::proto::KbdEdgeRec>,
    window_us: u32,
) -> LogicAnalyzerView {
    const KEYS: usize = 12;
    const Y_ON: i32 = 16; // 按下
    const Y_OFF: i32 = 84; // 松开
    let mut view = LogicAnalyzerView {
        raw_paths: vec![slint::SharedString::new(); KEYS],
        deb_paths: vec![slint::SharedString::new(); KEYS],
        out_paths: vec![slint::SharedString::new(); KEYS],
        trig_text: vec![slint::SharedString::new(); KEYS],
        trig_x: vec![-1.0; KEYS],
        time_labels: Vec::new(),
        span_text: slint::SharedString::new(),
    };
    // 横轴刻度先建好: 即使无数据也要有刻度, 免得空图看起来像"坏了"。
    for i in 0..5 {
        let back_us = window_us as f64 * (4 - i) as f64 / 4.0;
        view.time_labels.push(
            if i == 4 {
                "0".to_string()
            } else {
                format!("-{}", fmt_time_us(back_us))
            }
            .into(),
        );
    }
    let window_us = window_us.max(1);
    let Some(last) = edges.back() else {
        view.span_text = format!("窗口 {} · 无边沿记录", fmt_time_us(window_us as f64)).into();
        for k in 0..KEYS {
            view.trig_text[k] = "无数据".into();
        }
        return view;
    };
    let t_start = last.t_us.wrapping_sub(window_us);
    // 窗口内记录 + 窗口起点前的最后一个状态(作为初始电平)。
    let mut before: Option<(u16, u16, u16)> = None;
    let mut visible: Vec<&mai2control_ui::proto::KbdEdgeRec> = Vec::new();
    for rec in edges.iter() {
        if rec.t_us.wrapping_sub(t_start) <= window_us {
            visible.push(rec);
        } else {
            before = Some((rec.raw, rec.deb, rec.out));
        }
    }
    let x_of = |t: u32| -> f32 { t.wrapping_sub(t_start) as f32 / window_us as f32 * 1000.0 };
    for k in 0..KEYS {
        let bit = 1u16 << k;
        let (mut raw_on, mut deb_on, mut out_on) = match before {
            Some((r, d, o)) => ((r & bit) != 0, (d & bit) != 0, (o & bit) != 0),
            // 窗口前没有任何记录时, 用窗口内第一条的状态当起点(它就是该键进入窗口时的电平)。
            None => visible
                .first()
                .map(|r| ((r.raw & bit) != 0, (r.deb & bit) != 0, (r.out & bit) != 0))
                .unwrap_or((false, false, false)),
        };
        let mut raw_p = format!("M 0 {}", if raw_on { Y_ON } else { Y_OFF });
        let mut deb_p = format!("M 0 {}", if deb_on { Y_ON } else { Y_OFF });
        let mut out_p = format!("M 0 {}", if out_on { Y_ON } else { Y_OFF });
        let mut edge_count = 0u32;
        let mut trig_at: Option<u32> = None;
        for rec in &visible {
            let x = x_of(rec.t_us);
            let r = (rec.raw & bit) != 0;
            let d = (rec.deb & bit) != 0;
            let o = (rec.out & bit) != 0;
            if r != raw_on {
                // 阶梯: 先水平走到该时刻, 再垂直跳变 —— 电平信号不能画成斜线。
                raw_p.push_str(&format!(
                    " L {:.1} {} L {:.1} {}",
                    x,
                    if raw_on { Y_ON } else { Y_OFF },
                    x,
                    if r { Y_ON } else { Y_OFF }
                ));
                raw_on = r;
                edge_count += 1;
            }
            if d != deb_on {
                deb_p.push_str(&format!(
                    " L {:.1} {} L {:.1} {}",
                    x,
                    if deb_on { Y_ON } else { Y_OFF },
                    x,
                    if d { Y_ON } else { Y_OFF }
                ));
                deb_on = d;
                if d && trig_at.is_none() {
                    trig_at = Some(rec.t_us);
                }
            }
            if o != out_on {
                out_p.push_str(&format!(
                    " L {:.1} {} L {:.1} {}",
                    x,
                    if out_on { Y_ON } else { Y_OFF },
                    x,
                    if o { Y_ON } else { Y_OFF }
                ));
                out_on = o;
            }
        }
        raw_p.push_str(&format!(" L 1000 {}", if raw_on { Y_ON } else { Y_OFF }));
        deb_p.push_str(&format!(" L 1000 {}", if deb_on { Y_ON } else { Y_OFF }));
        out_p.push_str(&format!(" L 1000 {}", if out_on { Y_ON } else { Y_OFF }));
        view.raw_paths[k] = raw_p.into();
        view.deb_paths[k] = deb_p.into();
        view.out_paths[k] = out_p.into();
        view.trig_text[k] = match trig_at {
            Some(t) => {
                view.trig_x[k] = x_of(t);
                format!(
                    "+{} ·{}沿",
                    fmt_time_us(t.wrapping_sub(t_start) as f64),
                    edge_count
                )
                .into()
            }
            None if edge_count > 0 => format!("无上升沿 ·{}沿", edge_count).into(),
            None => "静默".into(),
        };
    }
    view.span_text = format!(
        "窗口 {} · 窗口内 {} 条 / 缓冲 {} 条 · 右缘 t={}us",
        fmt_time_us(window_us as f64),
        visible.len(),
        edges.len(),
        last.t_us
    )
    .into();
    view
}

/// 生成延迟历史折线 path + 自适应纵向量程 (lo, hi)。x 固定映射到 [0,1000]，由 PlotPath 的 fit: fill 拉满绘图区。
/// 不能用 aspect 修正：等比 contain 无法铺满任意矩形，宽高比变化后会重新产生空白带。
/// 纵向量程按数据 min/max 自适应(带 10% 余量), 否则接近常数的延迟会被压成一条线。
pub(crate) fn build_lat_path(series: &[f32]) -> (String, f32, f32) {
    let dmin = series.iter().cloned().fold(f32::INFINITY, f32::min);
    let dmax = series.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let (lo, hi) = if dmin.is_finite() && dmax.is_finite() {
        let pad = ((dmax - dmin) * 0.1).max(1.0);
        (dmin - pad, dmax + pad)
    } else {
        (-1.0, 1.0)
    };
    (series_to_svg_path(series, lo, hi, 1.0), lo, hi)
}
