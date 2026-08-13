//! 遥测/参数编解码 (#6g-1)
//!
//! 与固件 `main_firmware/src/service/sensor_link/sensor_link.cpp` 对齐的协议实现。
//! 所有多字节值 LE(小端)。
//!
//! 设计参照 `protocol_design.md` 修订3 T1~T5。

// ============================================================================
// 参数 ID 常量(T1)
// ============================================================================

/// 手指检测阈值
pub const PARAM_FINGER_TH: u8 = 0x01;
/// 噪声阈值
pub const PARAM_NOISE_TH: u8 = 0x02;
/// 负噪声阈值
pub const PARAM_NEG_NOISE_TH: u8 = 0x03;
/// 迟滞
pub const PARAM_HYSTERESIS: u8 = 0x04;
/// 触发去抖计数
pub const PARAM_ON_DEBOUNCE: u8 = 0x05;
/// 低基线复位
pub const PARAM_LOW_BSLN_RST: u8 = 0x06;
/// 扫描分辨率
pub const PARAM_RESOLUTION: u8 = 0x07;
/// 感应时钟分频
pub const PARAM_SNS_CLK_DIV: u8 = 0x08;
/// 调制 IDAC
pub const PARAM_IDAC_MOD: u8 = 0x09;
/// 传感时钟源
pub const PARAM_SNS_CLK_SOURCE: u8 = 0x0A;
/// IDAC 增幅档
pub const PARAM_IDAC_GAIN: u8 = 0x0B;
/// 通道启用开关(0=禁用/电极保持模拟高阻, 1=启用)。
/// ★这不是调参项而是硬件开关★: 禁用后该通道的 widget 永久退出 PSoC 的扫描序列, 电极停在高阻,
/// 因此它与 AUTO/SEMI 模式无关, 也不需要 APPLY/CALIBRATE 去"生效"。
pub const PARAM_ENABLED: u8 = 0x0C;

/// 逐通道**可批量应用**的参数清单 —— 批量面板行、勾选掩码、围栏列与应用动作的唯一真相源。
///
/// ★分辨率(0x07) 刻意不在此列★: 它决定整机扫描帧周期, 必须 36 通道完全一致, 因此语义上是
/// **全局**设置, 唯一入口是"触控全局调整"页的采样分辨率(写入时由 `set_param` 统一铺到全部通道)。
/// 批量面板若也给一个入口, 就等于允许"只给一部分通道改分辨率" —— 那正是帧周期不齐的直接来路。
/// ★通道启用开关(0x0C) 同样不在此列★: 它是硬件开关而非调参项, 由"批量启用/禁用"单独承担。
pub const BATCH_PARAM_IDS: &[u8] = &[
    PARAM_FINGER_TH,
    PARAM_NOISE_TH,
    PARAM_NEG_NOISE_TH,
    PARAM_HYSTERESIS,
    PARAM_ON_DEBOUNCE,
    PARAM_LOW_BSLN_RST,
    PARAM_SNS_CLK_DIV,
    PARAM_IDAC_MOD,
    PARAM_SNS_CLK_SOURCE,
    PARAM_IDAC_GAIN,
];

/// 所有已知 param_id 列表(对齐固件 kParamIds 顺序)
pub const KNOWN_PARAM_IDS: &[u8] = &[
    PARAM_FINGER_TH,
    PARAM_NOISE_TH,
    PARAM_NEG_NOISE_TH,
    PARAM_HYSTERESIS,
    PARAM_ON_DEBOUNCE,
    PARAM_LOW_BSLN_RST,
    PARAM_RESOLUTION,
    PARAM_SNS_CLK_DIV,
    PARAM_IDAC_MOD,
    PARAM_SNS_CLK_SOURCE,
    PARAM_IDAC_GAIN,
    PARAM_ENABLED,
];

// ============================================================================
// 单通道参数合法范围(围栏) —— 上位机侧唯一权威声明表
// ============================================================================
//
// ★本表是上位机侧唯一权威★: UI 输入围栏、下发前拒绝、回读防污染三条路径全部只读本表,
// 上位机任何其它位置(含 .slint)都不许再写第二份阈值 —— 三份围栏各自漂移正是本次要收掉的病灶。
//
// ★三处必须同源(判定逐位等价), 任何一处改动必须三处同改★:
//   1. 上位机: 本文件 `param_fence` / `param_value_legal`
//      (`control_software/src/proto/telemetry.rs`)
//   2. RP2040: `main_firmware/src/service/sensor_link/sensor_link.cpp::_handle_param_set`
//      的合法性 switch(约 :420-435, 非法 → NAK, 不下发 PSoC、不写真相源)
//   3. PSoC:   `psoc_firmware/CY8C4147AZI-SensorCore/main.c::_param_value_legal`
//      (约 :800-812, 非法 → `cmd_set_param` 直接 return false, 不写 widgetContext)
//
// 逐位等价的两个要点:
//   - `value_mask`: 固件对 SNS_CLK_SOURCE 判的是 `(value & 0x7F) <= 6`, 即高位(含 0x80 的
//     CapSense AUTO 标志)根本不参与判定。故本表用 value_mask 精确复刻固件的取位, 而不是
//     用 `!flag_mask` 反推 —— 后者会把 0x100 这类高位判成非法, 与固件行为不一致。
//   - `guarded`: 固件的 switch 只拦 0x07/0x08/0x09/0x0A/0x0B, 其余走 `default: return true`
//     完全不限制。故未被拦截项的 min/max 只是主机侧 UI 输入范围(依据 PSoC widgetContext
//     字段位宽: onDebounce 是 uint8_t, 其余阈值类是 uint16_t; 越界是静默截断而非拒绝),
//     判定必须放行, 否则上位机会拒绝固件本来接受的值。

/// 单个 param 的围栏声明。★全局项(GPARAM_*)复用同一结构★, 见本文件 `global_fence`。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParamFence {
    /// 协议名(与固件 `PARAM_*` / `GPARAM_*` 宏同名), 供告警文案指名道姓。
    pub name: &'static str,
    /// 值域下界(对取位后的"值域部分"生效)。
    pub min: u32,
    /// 值域上界(同上)。
    pub max: u32,
    /// 值域部分的取位掩码, 与固件 `value & 0x??` 逐位对齐。
    pub value_mask: u32,
    /// 标志位掩码: 不参与范围判定、且 clamp 必须原样保留的位。
    /// SNS_CLK_SOURCE 的 0x80 = CapSense AUTO 标志, 设备合法持有 128(=AUTO + 源 0)。
    pub flag_mask: u32,
    /// ★枚举型取值集合★: `bit_v == 1` 表示值 `v` 合法(仅覆盖 0..=31)。`0` = 不用集合判定,
    /// 只按 `min..=max` 连续区间。为 `GPARAM_INACTIVE_SNS` 这种非连续取值而设 ——
    /// 固件判的是 `(value==1)||(value==2)||(value==4)`, 用 min/max 表达会把 3 误判成合法。
    /// ★不另造第二种结构★: 全局项与单通道项共用本 ParamFence, 单通道项该字段一律 0。
    pub value_set: u32,
    /// 该项是否被固件的合法性 switch 真正拦截(NAK / 拒写)。false ⇒ 判定一律放行。
    pub guarded: bool,
    /// ── 以下四项 = 参数元数据(UI "?" 悬浮详解的唯一来源) ────────────────────────────
    /// ★为什么并入 ParamFence 而不另建一张表★ 名称/范围已经在这里, 单位与说明再另开一张
    /// `match param_id` 就等于第二份元数据 —— 两处必然漂移(围栏当年就是这么漂出
    /// "SNS_CLK_SOURCE 上界写死 6"的)。一个参数的一切事实只能有一个出处。
    /// 计量单位(无量纲则空串), 如 "ADC 计数" / "帧" / "档"。
    pub unit: &'static str,
    /// 作用域: 该值是逐通道各自持有, 还是 36 通道共用一份。
    pub scope: ParamScope,
    /// 这是什么 / 调它干什么(一句话)。
    pub help: &'static str,
    /// 改了会影响什么、有什么代价或风险(一句话)。
    pub impact: &'static str,
}

/// 参数作用域。★用枚举而不是 bool★: UI 要显示的是"逐通道 / 全局共用 / 只读诊断"三态,
/// 用 bool 组合表达会立刻退化成两个互相矛盾的字段。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamScope {
    /// 每个通道各自一份(PARAM_*, 经 PARAM_SET(ch, id))。
    Channel,
    /// 36 通道共用一份(GPARAM_*, 经 GLOBAL_SET + GLOBAL_COMMIT)。
    Global,
    /// 只读诊断项, 写路径固件不处理。
    ReadOnly,
}

impl ParamScope {
    pub fn text(self) -> &'static str {
        match self {
            ParamScope::Channel => "逐通道(每通道各自一份)",
            ParamScope::Global => "全局(36 通道共用)",
            ParamScope::ReadOnly => "只读诊断",
        }
    }
}

impl ParamFence {
    /// 取值域部分(丢弃标志位与固件同样不看的高位)。
    pub fn core(&self, value: u32) -> u32 {
        value & self.value_mask
    }
    /// UI 输入下界。
    pub fn ui_min(&self) -> u32 {
        self.min
    }
    /// UI 输入上界。★必须容得下"值域上界 + 全部标志位"★: SNS_CLK_SOURCE 的设备合法值 128
    /// 若被 UI 上界(6)夹掉, 用户一碰这一行就会把 AUTO 标志抹掉并下发 6。
    /// 标志位与值域不连续 ⇒ 区间内存在非法点(如 0x0A 的 7..127), 那些点由下发前的
    /// `param_value_legal` 明确拒绝并告知, 不会静默走到设备上。
    pub fn ui_max(&self) -> u32 {
        self.max | self.flag_mask
    }
    /// 单选项上限: 超过这么多可选值就不适合做单选, 回落数字输入框。
    pub const UI_CHOICE_MAX: usize = 16;

    /// UI 单选项列表 `(显示文本, 实际下发值)`。返回空 ⇒ 该项继续用数字输入框。
    ///
    /// ★为什么必须有这个★
    /// ① 带标志位的参数, 其 UI 区间内存在**非法空洞**: SNS_CLK_SOURCE 的 `ui_max()` = 6|0x80 = 134,
    ///    而 7..127 全部非法 —— 数字框必然让用户填出 26 这种值(实测用户就填出来了)。
    /// ② "128 = AUTO" 这种编码用数字表达等于让人背魔数; 凡语义上带 AUTO 的项, 界面就该有「AUTO」。
    /// ③ 离散取值集合(如 INACTIVE_SNS 的 {1,2,4})用数字框会放行 3。
    /// 于是: 有标志位、或取值离散、或可选值少于 UI_CHOICE_MAX 的项, 一律做成单选,
    /// 用户就**填不出**非法值, 也不必知道哪个数字代表 AUTO。
    pub fn ui_choices(&self) -> Vec<(String, u32)> {
        let mut out: Vec<(String, u32)> = Vec::new();
        if self.value_set != 0 {
            for v in 0u32..32 {
                if (self.value_set & (1u32 << v)) != 0 {
                    out.push((v.to_string(), v));
                }
            }
        } else {
            let span = self.max.saturating_sub(self.min).saturating_add(1) as usize;
            // 区间太宽做不成单选。★当前没有"既带标志位又区间很宽"的项★(SNS_CLK_SOURCE 是 0..6);
            // 若将来出现, 这里会回落成数字框而让 AUTO 无从选中 —— 那时必须为它单独设计控件, 而不是
            // 悄悄放行, 故此处不做静默兜底。
            if span > Self::UI_CHOICE_MAX {
                return Vec::new();
            }
            for v in self.min..=self.max {
                out.push((v.to_string(), v));
            }
        }
        if self.flag_mask != 0 {
            // AUTO = 只置标志位、值域部分取下界。设备合法持有 128(=AUTO + 源 0)。
            out.push(("AUTO".to_string(), self.flag_mask | self.min));
        }
        out
    }

    /// 设备真值在单选项里的下标; 找不到返回 -1(界面据此显示"设备值不在合法取值内", 而不是假装选中某项)。
    pub fn ui_choice_index(&self, value: u32) -> i32 {
        let choices = self.ui_choices();
        if choices.is_empty() {
            return -1;
        }
        // 标志位置起即为 AUTO, 不比较值域部分(AUTO 下值域无意义)。
        if self.flag_mask != 0 && (value & self.flag_mask) != 0 {
            return (choices.len() - 1) as i32;
        }
        let core = self.core(value);
        choices
            .iter()
            .position(|(_, v)| *v == core)
            .map(|i| i as i32)
            .unwrap_or(-1)
    }

    /// 值域部分是否落在允许集合/区间内(不含 `guarded` 短路, 由调用方判断)。
    fn _core_in_range(&self, core: u32) -> bool {
        if self.value_set != 0 {
            return core < 32 && (self.value_set & (1u32 << core)) != 0;
        }
        core >= self.min && core <= self.max
    }
    /// 把值域部分收到最近的合法取值。集合型取"绝对差最小"的成员(并列取小者), 区间型即 clamp。
    fn _core_fix(&self, core: u32) -> u32 {
        if self.value_set == 0 {
            return core.clamp(self.min, self.max);
        }
        let mut best = self.min;
        let mut best_d = u32::MAX;
        for v in 0u32..32 {
            if (self.value_set & (1u32 << v)) == 0 {
                continue;
            }
            let d = if v > core { v - core } else { core - v };
            if d < best_d {
                best_d = d;
                best = v;
            }
        }
        best
    }
    /// 补齐元数据(单位/作用域/说明/影响)。★链式而非再加三个构造函数★:
    /// `_fence_soft/_fence_hard/_fence_set` 表达的是"围栏形状", 元数据是另一维度, 混进参数表会
    /// 让每个构造函数都变成 8 个位置参数(读的人根本对不上位)。
    fn _doc(
        mut self,
        unit: &'static str,
        scope: ParamScope,
        help: &'static str,
        impact: &'static str,
    ) -> Self {
        self.unit = unit;
        self.scope = scope;
        self.help = help;
        self.impact = impact;
        self
    }

    /// UI "?" 悬浮详解的完整文本。★由本处一处拼装★: 界面只显示字符串, 不自己按 param_id 拼,
    /// 否则单位/范围又会出现第二份口径。
    pub fn help_text(&self, param_id: u8) -> String {
        let mut out = format!("{} (0x{:02X})\n", self.name, param_id);
        out.push_str(&format!("作用域: {}\n", self.scope.text()));
        if self.unit.is_empty() {
            out.push_str(&format!("取值: {}\n", self.range_text()));
        } else {
            out.push_str(&format!("取值: {} {}\n", self.range_text(), self.unit));
        }
        out.push_str(&format!(
            "越界处理: {}\n",
            if self.guarded {
                "固件拒收并回 NAK(不会静默写坏)"
            } else {
                "固件不拦, 超出字段位宽会静默截断"
            }
        ));
        if !self.help.is_empty() {
            out.push_str(&format!("说明: {}\n", self.help));
        }
        if !self.impact.is_empty() {
            out.push_str(&format!("影响: {}", self.impact));
        }
        out
    }

    /// 合法范围的中文描述(告警文案用)。
    pub fn range_text(&self) -> String {
        if self.value_set != 0 {
            let members: Vec<String> = (0u32..32)
                .filter(|v| (self.value_set & (1u32 << v)) != 0)
                .map(|v| v.to_string())
                .collect();
            return format!("仅 {{{}}}", members.join(", "));
        }
        if self.flag_mask != 0 {
            format!(
                "{}..{}, 或叠加标志位 0x{:02X}(如 {} = 标志位 + 值 0)",
                self.min,
                self.max,
                self.flag_mask,
                self.flag_mask | self.min
            )
        } else {
            format!("{}..{}", self.min, self.max)
        }
    }
}

/// 元数据默认值: 未显式 `_doc()` 的项按"逐通道 + 无单位 + 无说明"处理(UI 侧只少一段文字, 不会误导)。
const _DOC_NONE: (&str, ParamScope, &str, &str) = ("", ParamScope::Channel, "", "");

/// 固件不拦截项: min 固定 0, min/max 仅作主机侧 UI 输入范围。
const fn _fence_soft(name: &'static str, max: u32) -> ParamFence {
    ParamFence {
        name,
        min: 0,
        max,
        value_mask: 0xFFFF_FFFF,
        flag_mask: 0,
        value_set: 0,
        guarded: false,
        unit: _DOC_NONE.0,
        scope: _DOC_NONE.1,
        help: _DOC_NONE.2,
        impact: _DOC_NONE.3,
    }
}

/// 固件硬拦截项(非法即 NAK / 拒写)。
const fn _fence_hard(
    name: &'static str,
    min: u32,
    max: u32,
    value_mask: u32,
    flag_mask: u32,
) -> ParamFence {
    ParamFence {
        name,
        min,
        max,
        value_mask,
        flag_mask,
        value_set: 0,
        guarded: true,
        unit: _DOC_NONE.0,
        scope: _DOC_NONE.1,
        help: _DOC_NONE.2,
        impact: _DOC_NONE.3,
    }
}

/// 固件硬拦截 + 取值为非连续集合的项(如 `GPARAM_INACTIVE_SNS` 的 {1,2,4})。
/// `min`/`max` 仍给出, 只作 UI 输入范围(SpinBox 表达不了离散集合); 合法性判定用 `value_set`。
const fn _fence_set(name: &'static str, min: u32, max: u32, value_set: u32) -> ParamFence {
    ParamFence {
        name,
        min,
        max,
        value_mask: 0xFFFF_FFFF,
        flag_mask: 0,
        value_set,
        guarded: true,
        unit: _DOC_NONE.0,
        scope: _DOC_NONE.1,
        help: _DOC_NONE.2,
        impact: _DOC_NONE.3,
    }
}

/// 取某个 param 的围栏。未知 param_id 视为无限制(与固件 `default` 一致)。
pub fn param_fence(param_id: u8) -> ParamFence {
    use ParamScope::Channel;
    match param_id {
        // —— 阈值/迟滞/消抖类: 固件 default 放行, 上界=PSoC widgetContext 字段位宽 ——
        PARAM_FINGER_TH => _fence_soft("PARAM_FINGER_TH", 0xFFFF)._doc(
            "ADC 计数(diff)",
            Channel,
            "diff(raw 与基线之差)超过它即判定该通道被按下。",
            "调低更灵敏但更容易被噪声误触; 调高更稳但需更大接触面积。必须小于\"2^分辨率 - 基线\", 否则该通道永不触发。",
        ),
        PARAM_NOISE_TH => _fence_soft("PARAM_NOISE_TH", 0xFFFF)._doc(
            "ADC 计数(diff)",
            Channel,
            "低于它的 diff 一律当噪声丢弃, 也是基线继续跟随漂移的上限。",
            "应显著低于手指阈值。设得过高会让基线跟着手指一起漂(按住一会儿就失去触发)。",
        ),
        PARAM_NEG_NOISE_TH => _fence_soft("PARAM_NEG_NOISE_TH", 0xFFFF)._doc(
            "ADC 计数(diff)",
            Channel,
            "负方向噪声容限: diff 低于 -该值且持续超过\"低基线复位\"帧数即强制重置基线。",
            "过小会在温漂/上电瞬态时频繁重置基线; 过大则电极被负向干扰后长时间不能自愈。",
        ),
        PARAM_HYSTERESIS => _fence_soft("PARAM_HYSTERESIS", 0xFFFF)._doc(
            "ADC 计数(diff)",
            Channel,
            "触发与释放的门槛差: 按下要 > 阈值+迟滞, 释放要 < 阈值-迟滞。",
            "过小会在阈值附近抖动出连续通断; 过大则释放迟钝(手离开了还判为按住)。",
        ),
        // onDebounce 是 uint8_t → 越界静默截断, UI 上界收到 255。
        PARAM_ON_DEBOUNCE => _fence_soft("PARAM_ON_DEBOUNCE", 0xFF)._doc(
            "帧(扫描轮)",
            Channel,
            "连续多少帧都判为按下才真正上报按下。",
            "每 +1 帧就多一轮扫描周期的延迟(扫描周期见主页实测值)。过大直接表现为\"手感慢半拍\"。",
        ),
        PARAM_LOW_BSLN_RST => _fence_soft("PARAM_LOW_BSLN_RST", 0xFFFF)._doc(
            "帧(扫描轮)",
            Channel,
            "diff 持续低于负阈值多少帧后强制把基线重置到当前 raw。",
            "配合负阈值使用。过小会误重置(把真实按压当成漂移), 过大则负向干扰后恢复很慢。",
        ),
        // —— 硬件类: 非法值会让转换 railed / 时钟异常 / 校准发散, 固件拒收 ——
        PARAM_RESOLUTION => _fence_hard("PARAM_RESOLUTION", 6, 16, 0xFFFF_FFFF, 0)._doc(
            "位",
            Channel,
            "该通道单次转换的扫描分辨率, 决定 raw 满量程 = 2^N - 1。",
            "★逐通道各自持有, 但整机应保持一致★: 各通道分辨率不同会让阈值/量程口径互不可比。每 +1 位, 该通道的转换时间约翻倍(直接拉长整轮扫描周期)。",
        ),
        // 0 会除零。
        PARAM_SNS_CLK_DIV => _fence_hard("PARAM_SNS_CLK_DIV", 1, 255, 0xFFFF_FFFF, 0)._doc(
            "分频(传感时钟 = 源时钟 / 该值)",
            Channel,
            "该通道的传感时钟分频。值越大 = 频率越低 = 每次充放电时间越长。",
            "高 Cp 电极在高频(小分频)下来不及建立, IDAC 无论如何都压不到目标 ⇒ raw 满量程、diff 恒 0。\"频率自适应\"就是自动找该通道能用的最高频率。频率也决定抗干扰特性 —— 右侧\"响应噪声频谱\"可以逐点实测。",
        ),
        // idacMod 是 7 位。
        PARAM_IDAC_MOD => _fence_hard("PARAM_IDAC_MOD", 0, 127, 0xFFFF_FFFF, 0)._doc(
            "LSB(每 LSB 电流由增益档决定)",
            Channel,
            "调制 IDAC 码值: 校准就是解出它, 使空载 raw 落在\"校准目标%\"上。",
            "自动校准模式下由固件每次校准重算, 手填会被下次校准覆盖。半自动手动模式下才是持久设置; 设偏会让 raw 贴满量程或贴底, 两种情况 diff 都失效。",
        ),
        // 固件: `(value & 0x7F) <= 6`; 0x80 = AUTO 标志, 不参与判定且必须保留。
        PARAM_SNS_CLK_SOURCE => _fence_hard("PARAM_SNS_CLK_SOURCE", 0, 6, 0x7F, 0x80)._doc(
            "",
            Channel,
            "传感时钟源/扩频模式(0..6), AUTO = 由 CapSense 自行选择(设备值 128 = AUTO 标志 + 源 0)。",
            "PRS/SSC 类扩频源能压低窄带干扰, 但会让等效频率不再是单一值 —— 频谱扫描的读数应在固定源(非 AUTO)下才好解释。",
        ),
        // 增益档表只有 7 项, 索引 7 越界会让 PSoC 崩溃。
        PARAM_IDAC_GAIN => _fence_hard("PARAM_IDAC_GAIN", 0, 6, 0xFFFF_FFFF, 0)._doc(
            "档(索引 → 每 LSB 电流)",
            Channel,
            "IDAC 增益档索引。★档号不是单调的★: 每档电流(pA/LSB)依次为 37.5k / 75k / 300k / 600k / 2400k / 4800k / 1200k —— 6 档实际落在 3 与 4 之间。",
            "电流越大越能压住高 Cp 电极(否则 raw railed), 但过大会牺牲分辨力(同样的手指变化对应更少的计数)。显式设过即锁定该通道, 此后校准不再把它冲回全局起点档。",
        ),
        // 通道启用开关: 固件两处都硬判 `value <= 1`(非法即 NAK/拒写)。
        PARAM_ENABLED => _fence_hard("PARAM_ENABLED", 0, 1, 0xFFFF_FFFF, 0)._doc(
            "",
            Channel,
            "该通道是否参与扫描。0=禁用: 该通道的 widget 退出 PSoC 扫描序列, 其电极持续保持模拟高阻(不驱动、不测量); 1=启用。",
            "禁用是硬件级关闭, 不是界面隐藏: 该通道不再上报 raw/baseline/diff(一律为 0)、不会触发, 校准/基线复位/频率自适应/Cp 测量对它一律不执行(固件会明确回绝)。整轮扫描周期随之按比例缩短。重新启用时固件只为这一个通道重做校准与基线, 不扰动其它通道。",
        ),
        _ => _fence_soft("PARAM_UNKNOWN", 0xFFFF_FFFF),
    }
}

/// IDAC 增益档索引 → 每 LSB 电流(pA)。★唯一来源 = 设备生成配置★
/// `psoc_firmware/CY8C4147AZI-SensorCore/bsps/TARGET_APP_CY8CKIT-149/config/GeneratedSource/
///  cycfg_capsense.c:171` 的 `.idacGainTable`(第二个字段就是 gainValue, 单位 pA)。
/// ★为什么必须显式给出这张表★ 档号与电流**不单调**(档 6 = 1.2µA, 落在档 3 的 600nA 与档 4 的
/// 2.4µA 之间)。任何"档号越大电流越大"的假设都是错的 —— 由 Cp 推荐档位必须走电流值, 不能走档号。
pub const IDAC_GAIN_PA: [u32; 7] = [
    37_500, 75_000, 300_000, 600_000, 2_400_000, 4_800_000, 1_200_000,
];

/// 按"每 LSB 电流"升序排列的档号(由 `IDAC_GAIN_PA` 派生的固定次序): 0,1,2,3,6,4,5。
/// 供"由小到大"选档的插值算法使用, 使 UI/算法都不必再关心档号的非单调。
pub const IDAC_GAIN_BY_CURRENT: [u8; 7] = [0, 1, 2, 3, 6, 4, 5];

/// 值是否合法。★与固件判定逐位等价★(见本节头部注释)。
pub fn param_value_legal(param_id: u8, value: u32) -> bool {
    let fence = param_fence(param_id);
    if !fence.guarded {
        return true;
    }
    fence._core_in_range(fence.core(value))
}

/// 把非法值截断到最近的极值。★标志位原样保留★: 截断只作用于值域部分,
/// 因此 `param_clamp(PARAM_SNS_CLK_SOURCE, 128)` 仍是 128(AUTO 标志 + 源 0),
/// 绝不会被夹成 6 —— 那等于上位机自己把设备的 AUTO 标志抹掉。
pub fn param_clamp(param_id: u8, value: u32) -> u32 {
    let fence = param_fence(param_id);
    // ★不变量: `param_value_legal` 放行的值, clamp 必须原样返回★。固件不拦截项(guarded=false)
    // 不存在"非法值"概念(其 min/max 只是主机侧 UI 输入范围), 若在这里照样夹取, clamp 就会去改
    // 一个合法值 —— 那等于上位机自作主张。故先按 guarded 短路。
    if !fence.guarded {
        return value;
    }
    let flags = value & fence.flag_mask;
    let core = fence._core_fix(fence.core(value));
    core | flags
}

// ============================================================================
// 全局 CSD 配置(GPARAM_*)合法范围(围栏) —— 上位机侧唯一权威声明表
// ============================================================================
//
// 与上面单通道 `param_fence` 完全同构(共用 `ParamFence` 结构与 clamp/legal 语义), 只是判据表
// 对齐的是**全局项**的两处固件围栏。上位机任何其它位置(含 .slint 的 SpinBox 上下界)都不许再写
// 第二份阈值。
//
// ★三处必须同源(判定逐位等价), 任何一处改动必须三处同改★:
//   1. 上位机: 本文件 `global_fence` / `global_value_legal`
//      (`control_software/src/proto/telemetry.rs`)
//   2. RP2040: `main_firmware/src/service/sensor_link/sensor_link.cpp::_handle_global_set`
//      的 `glegal` switch(约 :735-747, 非法 → NAK, 不下发 PSoC、不写 CsdConfig 真相源)
//   3. PSoC:   `psoc_firmware/CY8C4147AZI-SensorCore/main.c::cmd_set_global`
//      (约 :509-546, 非法 → 该 case 直接不写, 保持原值; 回显走 `cmd_get_global` 的存储值)
//
// 逐位等价的几个要点:
//   - `INACTIVE_SNS` 固件判的是 `(value==1)||(value==2)||(value==4)` —— 离散集合, 3 是非法的。
//     故用 `value_set` 精确复刻, 不能写成 min=1/max=4(那会放行 3)。
//   - `IDAC_SENSE_CONFIG` / `AUTO_CALIBRATE_EN`: PSoC 侧只做 `value != 0` 判真不限范围, 但
//     **RP2040 侧硬 NAK `value > 1`**。主机的每一条 GLOBAL_SET 都必先过 RP2040, 故有效判据取
//     RP2040 的 0..1(取两者更严的一侧才不会出现"上位机放行、设备 NAK"的错配)。
//   - `MFS_DIV_F1/F2` 的 0..255 是真围栏而非位宽推测: 两处固件都已显式判 `<= 255`, 超范围 NAK。
//   - 只读诊断项(`0x09` BOOT_OVERRIDE、`0x80..0x84` DBG_*)以及未知 id: 两处固件的 set 路径都是
//     `default` 不处理/不限制 ⇒ 判定必须放行(guarded=false), 否则上位机会拒绝固件本来接受的值。

/// 取某个全局项(GPARAM)的围栏。未知 gparam_id 视为无限制(与两处固件的 `default` 一致)。
pub fn global_fence(gparam_id: u8) -> ParamFence {
    use ParamScope::{Global, ReadOnly};
    match gparam_id {
        // 未激活传感器连接: 1=GND 2=High-Z 4=Shield —— 离散集合, 3/0/其它皆非法。
        crate::proto::algo::GPARAM_INACTIVE_SNS => {
            _fence_set("GPARAM_INACTIVE_SNS", 1, 4, (1 << 1) | (1 << 2) | (1 << 4))._doc(
                "",
                Global,
                "扫描某个电极时, 其余 35 个电极接到哪里: 1=GND(接地) 2=High-Z(悬空) 4=Shield(驱动屏蔽)。",
                "★GND 必然更慢, 这是电气事实而非软件缺陷★: 相邻电极接地会把它们对被测电极的耦合全部\
计入负载电容, 转换建立时间随之拉长。本 36 段面板实测单通道从约 172.5µs 涨到约 4386µs, 叠加 MFS \
三频后整轮扫描约 833ms(≈1.2Hz) —— UI 上表现为\"实测探测周期 ≫ 期望值\"且 raw 全通道同值不抖。\
需要 GND 的抗串扰特性就必须接受这个代价; 要扫描速度请用 High-Z(面板默认), Shield 介于两者之间。",
            )
        }
        // idacGainTable 只有 7 项(CY_CAPSENSE_IDAC_GAIN_NUMBER), 索引 7 越界 → 写非法 IDAC → 挂死。
        crate::proto::algo::GPARAM_IDAC_GAIN_INIT => {
            _fence_hard("GPARAM_IDAC_GAIN_INIT", 0, 6, 0xFFFF_FFFF, 0)._doc(
                "档(索引 → 每 LSB 电流)",
                Global,
                "全局 IDAC 增益【起点】档: 每次校准都从这一档开始解 IDAC。档号非单调, 见逐通道\"IDAC 增幅档\"。",
                "改它会清空所有通道的手动增益档锁定(以全局值为准)。设得过低会让高 Cp 通道校准不收敛 → raw 满量程。",
            )
        }
        // csdIdacMin 是 7 位。
        crate::proto::algo::GPARAM_IDAC_MIN => _fence_hard("GPARAM_IDAC_MIN", 0, 127, 0xFFFF_FFFF, 0)
            ._doc(
                "LSB",
                Global,
                "自动校准允许解出的最小 IDAC 码值下限。",
                "抬高它可避免解出过小的 IDAC(信噪比差), 但过高会让低 Cp 通道无法压到校准目标。",
            ),
        // 校准目标 raw 百分比: 0 / ≥100 会让自动校准发散 → 全通道 railed。
        crate::proto::algo::GPARAM_RAW_TARGET => _fence_hard("GPARAM_RAW_TARGET", 1, 99, 0xFFFF_FFFF, 0)
            ._doc(
                "% 满量程",
                Global,
                "校准把空载 raw 拉到满量程的百分之多少(典型 85)。",
                "过高会让手指按下时 raw 撞顶(diff 被削); 过低则牺牲有效动态范围。0 或 ≥100 会让校准发散 → 全通道 railed。",
            ),
        // MFS 分频偏移落在 PSoC 的 uint8_t 字段; 两处固件都已显式判 <=255 并 NAK, 不再静默截断。
        crate::proto::algo::GPARAM_MFS_DIV_F1 => _fence_hard("GPARAM_MFS_DIV_F1", 0, 255, 0xFFFF_FFFF, 0)
            ._doc(
                "分频偏移",
                Global,
                "多频扫描(MFS)第二频点相对主频的分频偏移。",
                "MFS 用三个频点取中值来抗窄带干扰, 代价是整轮扫描时间约 ×3。偏移过小则三个频点太近, 抗干扰效果打折。",
            ),
        crate::proto::algo::GPARAM_MFS_DIV_F2 => _fence_hard("GPARAM_MFS_DIV_F2", 0, 255, 0xFFFF_FFFF, 0)
            ._doc(
                "分频偏移",
                Global,
                "多频扫描(MFS)第三频点相对主频的分频偏移。",
                "同第二频点; 三个频点应彼此拉开, 否则等于白花 ×3 的扫描时间。",
            ),
        // 0=IDAC sourcing, 1=IDAC sinking(RP2040 硬 NAK >1)。
        crate::proto::algo::GPARAM_IDAC_SENSE_CONFIG => {
            _fence_hard("GPARAM_IDAC_SENSE_CONFIG", 0, 1, 0xFFFF_FFFF, 0)._doc(
                "",
                Global,
                "IDAC 充电方向: 0=sourcing(灌流) 1=sinking(抽流)。",
                "改方向后所有通道的 IDAC 需重新校准, 未重校准前 raw 会明显偏移。",
            )
        }
        // 0=固定 IDAC, 1=Init/Apply 自动校准(RP2040 硬 NAK >1)。
        crate::proto::algo::GPARAM_AUTO_CALIBRATE_EN => {
            _fence_hard("GPARAM_AUTO_CALIBRATE_EN", 0, 1, 0xFFFF_FFFF, 0)._doc(
                "",
                Global,
                "0=固定 IDAC(用手填值) 1=初始化/应用时自动解 IDAC。",
                "由关转开会立刻触发一次真实全通道校准(固件在上升沿置校准请求)。开启后手填的 IDAC 会被校准结果覆盖。",
            )
        }
        // 只读诊断: 0x09=BOOT_OVERRIDE(启动强制改写位掩码), 0x80..0x84=SPI 链路计数。
        // 写路径两处固件都不处理 ⇒ 一律放行, 也不参与回读夹取(否则会去"修正"设备的诊断读数)。
        _ => _fence_soft("GPARAM_READONLY_OR_UNKNOWN", 0xFFFF_FFFF)._doc(
            "",
            ReadOnly,
            "只读诊断项(启动强制改写掩码 / SPI 链路计数等)。",
            "写路径固件不处理, 改它不会有任何效果。",
        ),
    }
}

/// 全局项值是否合法。★与两处固件判定逐位等价★(见本节头部注释)。
pub fn global_value_legal(gparam_id: u8, value: u32) -> bool {
    let fence = global_fence(gparam_id);
    if !fence.guarded {
        return true;
    }
    fence._core_in_range(fence.core(value))
}

/// 把非法的全局项值收到最近的合法取值。语义与 `param_clamp` 一致:
/// `global_value_legal` 放行的值必须原样返回(不拦截项不存在"非法"概念, 不许自作主张地改)。
pub fn global_clamp(gparam_id: u8, value: u32) -> u32 {
    let fence = global_fence(gparam_id);
    if !fence.guarded {
        return value;
    }
    let flags = value & fence.flag_mask;
    fence._core_fix(fence.core(value)) | flags
}

// ============================================================================
// 遥测字段位(T2)
// ============================================================================

/// 原始采样值
pub const FIELD_RAW: u8 = 0x01;
/// 基线值
pub const FIELD_BASELINE: u8 = 0x02;
/// 差值
pub const FIELD_DIFF: u8 = 0x04;
/// 触摸状态
pub const FIELD_STATUS: u8 = 0x08;
/// 采样率统计(帧级，非逐通道)：帧头 fields 字节后附 samples_per_sec(u32 LE)+scan_period_us(u32 LE)
pub const FIELD_STATS: u8 = 0x10;
/// 触控输出流水线延迟(帧级)：STATS 后附 spi/proc/usb 三个 u16 LE 滚动最大值
pub const FIELD_LATENCY: u8 = 0x20;
/// JIT 算法运行值(逐通道)：STATUS 之后附 out_active(u8) + report[0..3](u16 LE ×4) = 9B。
/// 与采样同帧同源，取代 ALGO_GET_TRACE 轮询(阻塞读类命令抢不到设备单响应槽)。
pub const FIELD_ALGO: u8 = 0x40;
/// 触控延迟线的补偿偏差(帧级)：LATENCY 之后附 dev_min(i16 LE) + dev_max(i16 LE) + flags(u8) = 5B。
/// dev = (实际发出时刻 − 该掩码的采样时刻) − `comm.touch_delay_100us`，正=发晚、负=发早。
/// ★这是"设定的触控延迟兑现了没有"的唯一直接读数★：采样零点取自延迟线**实际读出的那一片**，
/// 主机手上没有这个量；用链路耗时去减目标只是把两种不同量纲相减(那正是主页此前的缺陷)。
/// flags: bit0=本窗口真的发出过帧(有观测)，bit1=目标低于物理下限、延迟线已钳到最新采样。
pub const FIELD_DELAY_DEV: u8 = 0x80;
/// ★fields 的 8 个 bit 已全部用尽★：再要新字段必须先扩宽 fields 宽度(协议两侧同步)，
/// 不得复用任何已定义位，也不得把新数据挂到既有块尾部 —— 那会让不知情的解码方整片读偏。
pub const FIELD_ALL_ASSIGNED: u8 = 0xFF;

// ============================================================================
// 遥测编码函数(T3)
// ============================================================================

/// 编码 TELEM_START 请求载荷
/// payload = mode(u8) + rate_hz(u16 LE) + fields(u8) + ch_mask(u64 LE)
pub fn encode_telem_start(mode: u8, rate_hz: u16, fields: u8, ch_mask: u64) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(mode);
    payload.push((rate_hz & 0xFF) as u8);
    payload.push((rate_hz >> 8) as u8);
    payload.push(fields);
    payload.push((ch_mask & 0xFF) as u8);
    payload.push(((ch_mask >> 8) & 0xFF) as u8);
    payload.push(((ch_mask >> 16) & 0xFF) as u8);
    payload.push(((ch_mask >> 24) & 0xFF) as u8);
    payload.push(((ch_mask >> 32) & 0xFF) as u8);
    payload.push(((ch_mask >> 40) & 0xFF) as u8);
    payload.push(((ch_mask >> 48) & 0xFF) as u8);
    payload.push(((ch_mask >> 56) & 0xFF) as u8);
    payload
}

/// 编码 FOCUS_START 请求载荷(单通道独占流)。
/// payload = ch(u8) + fields(u8) + rate_hz(u16 LE) + lease_ms(u16 LE)
///
/// ★长度必须恰好 6 字节★ 固件 `_handle_focus_start` 判的是 `frame.len != 6` → 多一字节即 NAK。
/// rate_hz 由设备 clamp 到 1..1000 并在响应里回显实际受理值(可能不等于请求值)。
pub fn encode_focus_start(ch: u8, fields: u8, rate_hz: u16, lease_ms: u16) -> Vec<u8> {
    let mut payload = Vec::with_capacity(6);
    payload.push(ch);
    payload.push(fields);
    payload.extend_from_slice(&rate_hz.to_le_bytes());
    payload.extend_from_slice(&lease_ms.to_le_bytes());
    payload
}

/// 编码 FOCUS_STOP 请求载荷。payload = session(u16 LE)
/// 设备只接受**当前**会话号(不匹配即 NAK), 因此这里必须带上 START 响应回的 session。
pub fn encode_focus_stop(session: u16) -> Vec<u8> {
    session.to_le_bytes().to_vec()
}

/// 解码 FOCUS_START 响应载荷 → (session, accepted_rate_hz)。
/// payload = session(u16 LE) + accepted_rate_hz(u16 LE)
pub fn decode_focus_start(payload: &[u8]) -> Result<(u16, u16), String> {
    if payload.len() < 4 {
        return Err("FOCUS_START response too short (need >= 4 bytes)".to_string());
    }
    Ok((
        u16::from_le_bytes([payload[0], payload[1]]),
        u16::from_le_bytes([payload[2], payload[3]]),
    ))
}

/// 编码 SWEEP_START 请求载荷(增益/分频扫描会话)。payload = ch(u8) + settle_samples(u8) + sample_count(u8)
///
/// ★长度必须恰好 3 字节★ 固件 `_handle_sweep_start` 判的是 `frame.len != 3` → 多一字节即 NAK。
/// 两个采样参数由设备 clamp 到 0..64 / 1..64; 原 gain/div 由**设备**回读并在会话结束时写回。
pub fn encode_sweep_start(ch: u8, settle_samples: u8, sample_count: u8) -> Vec<u8> {
    vec![ch, settle_samples, sample_count]
}

/// 解码 SWEEP_START 响应载荷 → (session, total_cells)。payload = session(u16 LE) + total(u16 LE)
pub fn decode_sweep_start(payload: &[u8]) -> Result<(u16, u16), String> {
    if payload.len() < 4 {
        return Err("SWEEP_START response too short (need >= 4 bytes)".to_string());
    }
    Ok((
        u16::from_le_bytes([payload[0], payload[1]]),
        u16::from_le_bytes([payload[2], payload[3]]),
    ))
}

/// 编码 SWEEP_CTRL 取消。payload = [op=0, session u16 LE]
/// ★取消只是"请求恢复"★ 设备仍要走完写回 + 校准 + 基线才报终态, 不会立即结束会话。
pub fn encode_sweep_cancel(session: u16) -> Vec<u8> {
    let s = session.to_le_bytes();
    vec![0u8, s[0], s[1]]
}

/// 编码 SWEEP_CTRL 续租。payload = [op=2, session u16 LE]
/// 448 格会话一定长于设备 8s 租约，上位机需固定周期显式续租，不能依赖碰巧发生的其它命令。
pub fn encode_sweep_keepalive(session: u16) -> Vec<u8> {
    let s = session.to_le_bytes();
    vec![2u8, s[0], s[1]]
}

/// 编码 SWEEP_CTRL 补发。payload = [op=1, session u16 LE, first u16 LE, count u8]
/// 设备只补发**已缓存**的格(`first < produced`), 越界即 NAK; 实际区间被截到 produced 为止。
pub fn encode_sweep_resend(session: u16, first: u16, count: u8) -> Vec<u8> {
    let s = session.to_le_bytes();
    let f = first.to_le_bytes();
    vec![1u8, s[0], s[1], f[0], f[1], count]
}

/// SWEEP_DATA 结果位(低 4 位属该格采样)。
pub const SWEEP_FLAG_RAILED: u8 = 0x01;
/// 该格全部样本完全不抖动 —— 固件判为扫描停滞, 不是"噪声为零"。
pub const SWEEP_FLAG_STALLED: u8 = 0x02;
/// 该格改参数后的单通道校准失败。
pub const SWEEP_FLAG_CAL_FAIL: u8 = 0x04;
/// 设备回读的 gain/div != 该格期望值(PSoC 侧钳位/拒绝)。
pub const SWEEP_FLAG_MISMATCH: u8 = 0x08;
/// 恢复阶段异常位(高 4 位, 只出现在 RESTORING/终态帧): 写回原参数失败。
pub const SWEEP_RESTORE_FLAG_PARAM: u8 = 0x10;
/// 恢复阶段: 写回后的重新校准失败。
pub const SWEEP_RESTORE_FLAG_CAL: u8 = 0x20;
/// 恢复阶段: 基线复位失败。
pub const SWEEP_RESTORE_FLAG_BSLN: u8 = 0x40;
/// 该帧是 SWEEP_CTRL 补发的重传。
pub const SWEEP_FLAG_RETRANSMIT: u8 = 0x80;
/// 非结果帧(RESTORING / 终态)的格号占位。
pub const SWEEP_INDEX_NONE: u16 = 0xFFFF;
/// SWEEP_DATA 载荷的**兼容下限**(旧固件只发这么长)。
const SWEEP_DATA_LEN: usize = 21;

/// 设备侧扫描状态机的阶段码(固件 `SensorLink::SweepPhase` 的枚举值, 逐位同源)。
///
/// ★为什么要把它搬到上位机★ 只有 flags 时, "这一格没结果"说不出到底卡在哪一步 —— 参数写不进、
/// 回读不符、校准被 PSoC 拒、快照代次不推进, 四种处置完全不同。阶段码上报后, 任何未知故障都能
/// 直接落到状态机的具体一步。改固件枚举必须同改这里。
pub fn sweep_phase_text(phase: u8) -> &'static str {
    match phase {
        0 => "空闲",
        1 => "写入本格增益档/分频",
        2 => "等待参数写入完成",
        3 => "回读设备真值",
        4 => "应用参数",
        5 => "等待应用完成",
        6 => "等待基线/滤波稳定",
        7 => "采样",
        8 => "恢复: 写回原参数",
        9 => "恢复: 等待写回完成",
        10 => "恢复: 回读原参数",
        11 => "恢复: 发起校准",
        12 => "恢复: 等待校准完成",
        13 => "恢复: 发起基线复位",
        14 => "恢复: 等待基线复位完成",
        15 => "终态",
        16 => "恢复: 应用原参数",
        17 => "恢复: 等待应用完成",
        _ => "未知阶段",
    }
}

/// SWEEP_DATA 帧的会话状态(固件 `SweepDataState`)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepState {
    /// 一格结果(index 有效)。
    Cell,
    /// 已进入恢复链: 正在写回原 gain/div 并重新校准 + 复位基线。
    Restoring,
    /// 448 格扫完并恢复完毕。
    Done,
    /// 用户取消(或上位机丢失导致租约到期)后恢复完毕。
    Cancelled,
    /// 设备侧某阶段失败后恢复完毕。
    Failed,
}

impl SweepState {
    fn _from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(SweepState::Cell),
            2 => Some(SweepState::Restoring),
            3 => Some(SweepState::Done),
            4 => Some(SweepState::Cancelled),
            5 => Some(SweepState::Failed),
            _ => None,
        }
    }

    /// 是否为会话终态(此后设备侧会话已清, 不再接受 SWEEP_CTRL)。
    pub fn is_terminal(self) -> bool {
        matches!(
            self,
            SweepState::Done | SweepState::Cancelled | SweepState::Failed
        )
    }

    pub fn text(self) -> &'static str {
        match self {
            SweepState::Cell => "扫描中",
            SweepState::Restoring => "正在恢复原始增益档/分频",
            SweepState::Done => "已完成",
            SweepState::Cancelled => "已取消",
            SweepState::Failed => "设备侧失败",
        }
    }
}

/// SWEEP_DATA(0x38) 定长 21B 帧。
///
/// `[session u16][state u8][ch u8][index u16][total u16][produced u16][flags u8][gain u8][div u8]`
/// `[samples u16][mean u16][std_q8 u16][pp u16]`
///
/// ★gain/div 是设备回读的实际生效值★ 与 `index` 推出的期望值不一致时 flags 会带 MISMATCH,
/// 上位机据此把该格标为无效, 而不是让热图那一格标着一个设备并未生效的坐标。
#[derive(Debug, Clone, Copy)]
pub struct SweepFrame {
    pub session: u16,
    pub state: SweepState,
    pub ch: u8,
    /// 格号 = gain * 64 + (div - 1); 非结果帧为 `SWEEP_INDEX_NONE`。
    pub index: u16,
    pub total: u16,
    /// 设备已产出的格数(缺口检测的上界: 只有 `index < produced` 的格才可补发)。
    pub produced: u16,
    pub flags: u8,
    pub gain: u8,
    pub div: u8,
    pub samples: u16,
    pub mean: u16,
    /// RAW 标准差的 Q8 定点值(整数部分 << 8 + 小数)。
    pub std_q8: u16,
    pub pp: u16,
    /// 结果帧: 该格卡住的阶段; 非结果帧: 本会话首个失败阶段。0 = 无故障。
    /// `None` = 旧固件(载荷只有 21 字节)没这一项。
    pub fail_phase: Option<u8>,
    /// 发帧当刻的会话阶段(见 [`sweep_phase_text`])。
    pub phase: Option<u8>,
}

impl SweepFrame {
    /// 是否携带有效格结果。
    pub fn is_cell(&self) -> bool {
        self.state == SweepState::Cell && self.index < self.total
    }

    /// RAW 标准差真值(Q8 → f32)。
    pub fn std(&self) -> f32 {
        self.std_q8 as f32 / 256.0
    }

    /// 故障阶段的可读描述; 无故障或旧固件未上报时为 None。
    pub fn fail_phase_text(&self) -> Option<&'static str> {
        match self.fail_phase {
            Some(phase) if phase != 0 => Some(sweep_phase_text(phase)),
            _ => None,
        }
    }
}

/// 解码 SWEEP_DATA 推送载荷(定长 21B)。
pub fn decode_sweep_data(payload: &[u8]) -> Result<SweepFrame, String> {
    if payload.len() < SWEEP_DATA_LEN {
        return Err(format!(
            "SWEEP_DATA payload too short (need >= {} bytes, got {})",
            SWEEP_DATA_LEN,
            payload.len()
        ));
    }
    let state = SweepState::_from_u8(payload[2])
        .ok_or_else(|| format!("SWEEP_DATA unknown state {}", payload[2]))?;
    Ok(SweepFrame {
        session: u16::from_le_bytes([payload[0], payload[1]]),
        state,
        ch: payload[3],
        index: u16::from_le_bytes([payload[4], payload[5]]),
        total: u16::from_le_bytes([payload[6], payload[7]]),
        produced: u16::from_le_bytes([payload[8], payload[9]]),
        flags: payload[10],
        gain: payload[11],
        div: payload[12],
        samples: u16::from_le_bytes([payload[13], payload[14]]),
        mean: u16::from_le_bytes([payload[15], payload[16]]),
        std_q8: u16::from_le_bytes([payload[17], payload[18]]),
        pp: u16::from_le_bytes([payload[19], payload[20]]),
        // 尾部追加项: 旧固件没有, 缺失即 None(不当作 0 —— 0 的语义是"无故障", 与"没上报"不同)。
        fail_phase: payload.get(21).copied(),
        phase: payload.get(22).copied(),
    })
}

/// 编码 PARAM_GET 请求载荷
/// payload = channel(u8) + param_id(u8)
pub fn encode_param_get(channel: u8, param_id: u8) -> Vec<u8> {
    vec![channel, param_id]
}

/// 编码 PARAM_SET 请求载荷
/// payload = channel(u8) + param_id(u8) + value(u32 LE)
pub fn encode_param_set(channel: u8, param_id: u8, value: u32) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(channel);
    payload.push(param_id);
    payload.push((value & 0xFF) as u8);
    payload.push(((value >> 8) & 0xFF) as u8);
    payload.push(((value >> 16) & 0xFF) as u8);
    payload.push(((value >> 24) & 0xFF) as u8);
    payload
}

/// 编码 PARAM_GET_ALL 请求载荷
/// payload = channel(u8)
pub fn encode_param_get_all(channel: u8) -> Vec<u8> {
    vec![channel]
}

/// 编码 PARAM_GET_ALL 的"全通道单参数"变体请求载荷。
/// payload = 0xFF + param_id(u8) → 一帧回全 36 通道的该参数值(替代 36 条单发 PARAM_GET)。
pub fn encode_param_get_all_channels(param_id: u8) -> Vec<u8> {
    vec![PARAM_ALL_CHANNELS, param_id]
}

/// 编码 CP_MEASURE 请求载荷（空）。
pub fn encode_cp_measure() -> Vec<u8> {
    Vec::new()
}

/// 编码 CP_GET 请求载荷。
/// payload = channel(u8)
pub fn encode_cp_get(channel: u8) -> Vec<u8> {
    vec![channel]
}

/// 编码 CALIBRATE/BASELINE_RESET 的通道掩码
/// payload = ch_mask(u64 LE)
pub fn encode_ch_mask(ch_mask: u64) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push((ch_mask & 0xFF) as u8);
    payload.push(((ch_mask >> 8) & 0xFF) as u8);
    payload.push(((ch_mask >> 16) & 0xFF) as u8);
    payload.push(((ch_mask >> 24) & 0xFF) as u8);
    payload.push(((ch_mask >> 32) & 0xFF) as u8);
    payload.push(((ch_mask >> 40) & 0xFF) as u8);
    payload.push(((ch_mask >> 48) & 0xFF) as u8);
    payload.push(((ch_mask >> 56) & 0xFF) as u8);
    payload
}

// ============================================================================
// 遥测解码结构体与函数(T3)
// ============================================================================

/// 单通道采样
#[derive(Debug, Clone)]
pub struct ChannelSample {
    pub ch: u8,
    /// 采样所属帧的设备端时间戳(us, 原样搬运 TelemFrame.ts_us)。
    /// ★为什么每个样本都带★: 折线图的横轴必须是真实时间而不是等间距序号 —— 掉帧/暂停时
    /// 等间距会把时间轴画错。32 位 us 约 71 分钟回绕, 回绕展开在 app_state 侧统一处理
    /// (这里保持协议原值, 不做任何加工, 便于日志与协议排查对齐)。
    pub t_us: u32,
    pub raw: Option<u16>,
    pub bsln: Option<u16>,
    pub diff: Option<i16>,
    pub status: Option<u8>,
}

/// 遥测数据帧
#[derive(Debug, Clone)]
pub struct TelemFrame {
    pub ts_us: u32,
    pub fields: u8,
    pub samples_per_sec: u32,
    pub scan_period_us: u32,
    pub lat_spi_us: u16,
    pub lat_proc_us: u16,
    pub lat_usb_us: u16,
    /// 延迟线补偿偏差区间 (min, max)(us, 带符号)。`None` = 本窗口没有真正发出过触控帧,
    /// 无从判定偏差 —— 与"偏差恰为 0"是两件事, 不能合并成一个 0。
    pub delay_dev_us: Option<(i16, i16)>,
    /// 目标低于物理下限, 延迟线已钳到最新采样(设定延迟物理上达不到)。
    pub delay_dev_clamped: bool,
    pub samples: Vec<ChannelSample>,
}

/// 解析 DELAY_DEV 块(i16 min + i16 max + u8 flags)。返回 (偏差区间, 是否被钳制, 新读取位置)。
fn _decode_delay_dev(
    payload: &[u8],
    pos: usize,
) -> Result<(Option<(i16, i16)>, bool, usize), String> {
    if pos + 5 > payload.len() {
        return Err("truncated at DELAY_DEV block".to_string());
    }
    let lo = i16::from_le_bytes([payload[pos], payload[pos + 1]]);
    let hi = i16::from_le_bytes([payload[pos + 2], payload[pos + 3]]);
    let flags = payload[pos + 4];
    Ok((
        ((flags & 0x01) != 0).then_some((lo, hi)),
        (flags & 0x02) != 0,
        pos + 5,
    ))
}

/// 解码 TELEM_DATA 响应载荷
/// payload = ts_us(u32 LE) + ch_count(u8) + fields(u8) + [per_channel: ...]
/// 每通道按 fields 顺序: ch_index(u8) + (raw u16LE)? + (bsln u16LE)? + (diff i16LE)? + (status u8)?
pub fn decode_telem_data(payload: &[u8]) -> Result<TelemFrame, String> {
    if payload.len() < 6 {
        return Err("TELEM_DATA payload too short (need >= 6 bytes)".to_string());
    }

    let mut pos = 0;

    // ts_us(u32 LE)
    let ts_us = (payload[pos] as u32)
        | ((payload[pos + 1] as u32) << 8)
        | ((payload[pos + 2] as u32) << 16)
        | ((payload[pos + 3] as u32) << 24);
    pos += 4;

    // ch_count(u8)
    let ch_count = payload[pos] as usize;
    pos += 1;

    // fields(u8)
    let fields = payload[pos];
    pos += 1;

    // STATS(帧级，可选)：samples_per_sec(u32 LE) + scan_period_us(u32 LE)
    let (samples_per_sec, scan_period_us) = if (fields & FIELD_STATS) != 0 {
        if pos + 8 > payload.len() {
            return Err("Payload truncated while reading STATS field".to_string());
        }
        let sps = (payload[pos] as u32)
            | ((payload[pos + 1] as u32) << 8)
            | ((payload[pos + 2] as u32) << 16)
            | ((payload[pos + 3] as u32) << 24);
        let spu = (payload[pos + 4] as u32)
            | ((payload[pos + 5] as u32) << 8)
            | ((payload[pos + 6] as u32) << 16)
            | ((payload[pos + 7] as u32) << 24);
        pos += 8;
        (sps, spu)
    } else {
        (0u32, 0u32)
    };

    let (lat_spi_us, lat_proc_us, lat_usb_us) = if (fields & FIELD_LATENCY) != 0 {
        if pos + 6 > payload.len() {
            return Err("TELEM_DATA truncated at LATENCY block".to_string());
        }
        let a = u16::from_le_bytes([payload[pos], payload[pos + 1]]);
        let b = u16::from_le_bytes([payload[pos + 2], payload[pos + 3]]);
        let c = u16::from_le_bytes([payload[pos + 4], payload[pos + 5]]);
        pos += 6;
        (a, b, c)
    } else {
        (0u16, 0u16, 0u16)
    };

    let (delay_dev_us, delay_dev_clamped) = if (fields & FIELD_DELAY_DEV) != 0 {
        let (dev, clamped, next) = _decode_delay_dev(payload, pos)
            .map_err(|e| format!("TELEM_DATA {}", e))?;
        pos = next;
        (dev, clamped)
    } else {
        (None, false)
    };

    let mut samples = Vec::new();

    // 解析每个通道
    for _ in 0..ch_count {
        if pos >= payload.len() {
            return Err("Payload truncated while reading channel data".to_string());
        }

        // ch_index(u8)
        let ch = payload[pos];
        pos += 1;

        // 帧内所有通道同属一次扫描 → 共用帧级 ts_us 作为该样本的设备时间。
        let mut sample = ChannelSample {
            ch,
            t_us: ts_us,
            raw: None,
            bsln: None,
            diff: None,
            status: None,
        };

        // RAW (u16 LE)?
        if (fields & FIELD_RAW) != 0 {
            if pos + 2 > payload.len() {
                return Err("Payload truncated while reading RAW field".to_string());
            }
            let raw = (payload[pos] as u16) | ((payload[pos + 1] as u16) << 8);
            sample.raw = Some(raw);
            pos += 2;
        }

        // BASELINE (u16 LE)?
        if (fields & FIELD_BASELINE) != 0 {
            if pos + 2 > payload.len() {
                return Err("Payload truncated while reading BASELINE field".to_string());
            }
            let bsln = (payload[pos] as u16) | ((payload[pos + 1] as u16) << 8);
            sample.bsln = Some(bsln);
            pos += 2;
        }

        // DIFF (i16 LE)?
        if (fields & FIELD_DIFF) != 0 {
            if pos + 2 > payload.len() {
                return Err("Payload truncated while reading DIFF field".to_string());
            }
            let diff_u16 = (payload[pos] as u16) | ((payload[pos + 1] as u16) << 8);
            let diff = diff_u16 as i16;
            sample.diff = Some(diff);
            pos += 2;
        }

        // STATUS (u8)?
        if (fields & FIELD_STATUS) != 0 {
            if pos >= payload.len() {
                return Err("Payload truncated while reading STATUS field".to_string());
            }
            let status = payload[pos];
            sample.status = Some(status);
            pos += 1;
        }

        samples.push(sample);
    }

    Ok(TelemFrame {
        ts_us,
        fields,
        samples_per_sec,
        scan_period_us,
        lat_spi_us,
        lat_proc_us,
        lat_usb_us,
        delay_dev_us,
        delay_dev_clamped,
        samples,
    })
}

/// FOCUS_DATA(0x35) 单通道帧。
///
/// ★不引入第二套数据模型★ 采样体沿用 `TelemFrame`/`ChannelSample`(`samples` 恒为 1 项),
/// 于是上位机侧 FOCUS_DATA 与 TELEM_DATA 共用同一条入库路径与同一个绘图缓冲;
/// 会话标识(session/sample_seq/generation)是 focus 独有的那三项, 只用于归属判定与缺口统计。
#[derive(Debug, Clone)]
pub struct FocusFrame {
    /// 设备回的会话号: 与主机当前会话不一致 = 旧会话残帧, 必须立即丢弃。
    pub session: u16,
    /// 会话内单调递增的样本序号(设备背压丢帧时仍递增) → 缺口即真实丢帧数。
    pub sample_seq: u16,
    /// 该样本所属的 PSoC 快照代数(设备只在代数推进时才发帧)。
    pub generation: u16,
    /// JIT 算法运行值(FIELD_ALGO)：`(out_active, report[0..3])`；字段未请求或设备无有效算法时为 None。
    /// ★与采样同帧同源★ 取代原先的 ALGO_GET_TRACE 轮询: 那是阻塞读类命令, 在独占流推送期间
    /// 抢不到设备的单响应槽, 请求会成片超时(既无响应也无 NAK)。
    pub algo: Option<(u8, [u16; 4])>,
    pub frame: TelemFrame,
}

/// 解码 FOCUS_DATA 推送载荷。
/// payload = session(u16 LE) + sample_seq(u16 LE) + generation(u16 LE) + ch(u8) + fields(u8)
///         + t_us(u32 LE) + 按 fields 顺序: (raw u16LE)? (bsln u16LE)? (diff i16LE)? (status u8)?
///         + (STATS: samples_per_sec u32LE + scan_period_us u32LE)? + (LATENCY: spi/proc/usb u16LE)?
///
/// ★字段次序与 TELEM_DATA 不同★ 这里 STATS/LATENCY 在**单通道字段之后**(固件 `focus_tick` 的
/// 组帧顺序), 而 TELEM_DATA 是帧级前置。照 TELEM_DATA 的次序解会把统计值错位成采样值。
pub fn decode_focus_data(payload: &[u8]) -> Result<FocusFrame, String> {
    if payload.len() < 12 {
        return Err("FOCUS_DATA payload too short (need >= 12 bytes)".to_string());
    }
    let session = u16::from_le_bytes([payload[0], payload[1]]);
    let sample_seq = u16::from_le_bytes([payload[2], payload[3]]);
    let generation = u16::from_le_bytes([payload[4], payload[5]]);
    let ch = payload[6];
    let fields = payload[7];
    let t_us = u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);
    let mut pos = 12usize;

    let mut sample = ChannelSample {
        ch,
        t_us,
        raw: None,
        bsln: None,
        diff: None,
        status: None,
    };
    if (fields & FIELD_RAW) != 0 {
        if pos + 2 > payload.len() {
            return Err("FOCUS_DATA truncated at RAW field".to_string());
        }
        sample.raw = Some(u16::from_le_bytes([payload[pos], payload[pos + 1]]));
        pos += 2;
    }
    if (fields & FIELD_BASELINE) != 0 {
        if pos + 2 > payload.len() {
            return Err("FOCUS_DATA truncated at BASELINE field".to_string());
        }
        sample.bsln = Some(u16::from_le_bytes([payload[pos], payload[pos + 1]]));
        pos += 2;
    }
    if (fields & FIELD_DIFF) != 0 {
        if pos + 2 > payload.len() {
            return Err("FOCUS_DATA truncated at DIFF field".to_string());
        }
        sample.diff = Some(i16::from_le_bytes([payload[pos], payload[pos + 1]]));
        pos += 2;
    }
    if (fields & FIELD_STATUS) != 0 {
        if pos >= payload.len() {
            return Err("FOCUS_DATA truncated at STATUS field".to_string());
        }
        sample.status = Some(payload[pos]);
        pos += 1;
    }

    // 算法运行值: out_active(u8) + report[4](u16 LE)。次序与固件组帧一致(STATUS 之后、STATS 之前)。
    let algo = if (fields & FIELD_ALGO) != 0 {
        if pos + 9 > payload.len() {
            return Err("FOCUS_DATA truncated at ALGO block".to_string());
        }
        let active = payload[pos];
        let mut report = [0u16; 4];
        for (slot, value) in report.iter_mut().enumerate() {
            let at = pos + 1 + slot * 2;
            *value = u16::from_le_bytes([payload[at], payload[at + 1]]);
        }
        pos += 9;
        Some((active, report))
    } else {
        None
    };

    let (samples_per_sec, scan_period_us) = if (fields & FIELD_STATS) != 0 {
        if pos + 8 > payload.len() {
            return Err("FOCUS_DATA truncated at STATS block".to_string());
        }
        let sps = u32::from_le_bytes([
            payload[pos],
            payload[pos + 1],
            payload[pos + 2],
            payload[pos + 3],
        ]);
        let spu = u32::from_le_bytes([
            payload[pos + 4],
            payload[pos + 5],
            payload[pos + 6],
            payload[pos + 7],
        ]);
        pos += 8;
        (sps, spu)
    } else {
        (0u32, 0u32)
    };

    let (lat_spi_us, lat_proc_us, lat_usb_us) = if (fields & FIELD_LATENCY) != 0 {
        if pos + 6 > payload.len() {
            return Err("FOCUS_DATA truncated at LATENCY block".to_string());
        }
        let three = (
            u16::from_le_bytes([payload[pos], payload[pos + 1]]),
            u16::from_le_bytes([payload[pos + 2], payload[pos + 3]]),
            u16::from_le_bytes([payload[pos + 4], payload[pos + 5]]),
        );
        // ★这里必须推进 pos★: LATENCY 原先是最后一个块, 读完不推进也没人受害;
        // DELAY_DEV 接在它后面之后, 不推进就会把 LATENCY 的头两字节当成偏差再读一遍。
        pos += 6;
        three
    } else {
        (0u16, 0u16, 0u16)
    };

    let (delay_dev_us, delay_dev_clamped) = if (fields & FIELD_DELAY_DEV) != 0 {
        let (dev, clamped, _next) =
            _decode_delay_dev(payload, pos).map_err(|e| format!("FOCUS_DATA {}", e))?;
        (dev, clamped)
    } else {
        (None, false)
    };

    Ok(FocusFrame {
        session,
        sample_seq,
        generation,
        algo,
        frame: TelemFrame {
            ts_us: t_us,
            fields,
            samples_per_sec,
            scan_period_us,
            lat_spi_us,
            lat_proc_us,
            lat_usb_us,
            delay_dev_us,
            delay_dev_clamped,
            samples: vec![sample],
        },
    })
}

/// 解码 PARAM_GET 响应载荷
/// payload = channel(u8) + param_id(u8) + value(u32 LE)
/// 返回 (channel, param_id, value)
pub fn decode_param_get(payload: &[u8]) -> Result<(u8, u8, u32), String> {
    if payload.len() < 6 {
        return Err("PARAM_GET response too short (need >= 6 bytes)".to_string());
    }

    let ch = payload[0];
    let param_id = payload[1];
    let value = (payload[2] as u32)
        | ((payload[3] as u32) << 8)
        | ((payload[4] as u32) << 16)
        | ((payload[5] as u32) << 24);

    Ok((ch, param_id, value))
}

/// AUTO_TUNE_PROGRESS(0x2E) 推送帧: 设备侧频率自适应的阶段性进度/终态。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AutoTuneProgress {
    /// 0=空闲 1=进行中 2=完成
    pub state: u8,
    /// 0=已受理 1=粗定位 2=细搜临界 3=落档/回退 4=完成
    pub phase: u8,
    /// 当前阶段内步序(1 起, 设备侧上报饱和 31)
    pub step: u8,
    /// 进行中: 当前正在试探的 snsClk 分频
    pub cur_div: u16,
    /// 目标通道(0..35 单通道 / 0xFF 全通道)
    pub ch: u8,
    /// 0=进行中 1=成功 2=失败/超时
    pub result: u8,
    /// 完成时最终写入的分频(失败为 0)
    pub final_div: u16,
    /// 发起本轮自适应的**上位机请求 seq**(设备回显)。`None` = 旧固件(载荷只有 9 字节)没这一项。
    ///
    /// ★为什么必须有★ 本流是 STREAM 推送(帧头 seq 是设备自己的流序号, 与请求无关), 没有这个回显时
    /// "上一轮的终态"与"这一轮的终态"在上位机看来完全一样 —— 逐通道批量下, 迟到的旧终态会被算到
    /// 下一个通道头上(成功/失败张冠李戴)。
    pub origin_seq: Option<u8>,
}

impl AutoTuneProgress {
    /// 阶段中文名(供 UI 的"处理中"文案)。
    pub fn phase_text(&self) -> &'static str {
        match self.phase {
            1 => "粗定位",
            2 => "细搜临界",
            3 => "落档校验",
            4 => "完成",
            _ => "已受理",
        }
    }
}

/// 解码 AUTO_TUNE_PROGRESS 推送载荷。
/// payload = state(u8) + phase(u8) + step(u8) + cur_div(u16 LE) + ch(u8) + result(u8) + final_div(u16 LE)
///           [+ origin_seq(u8)]
/// ★尾部追加而非改布局★: 前 9 字节与旧固件逐字节相同, 故新旧固件/新旧上位机任意组合都能解析;
/// 只有"新固件 + 新上位机"这一组合才会用上 origin_seq 做严格归属。
pub fn decode_auto_tune_progress(payload: &[u8]) -> Result<AutoTuneProgress, String> {
    if payload.len() < 9 {
        return Err(format!(
            "AUTO_TUNE_PROGRESS payload must be >= 9 bytes, got {}",
            payload.len()
        ));
    }
    Ok(AutoTuneProgress {
        state: payload[0],
        phase: payload[1],
        step: payload[2],
        cur_div: u16::from_le_bytes([payload[3], payload[4]]),
        ch: payload[5],
        result: payload[6],
        final_div: u16::from_le_bytes([payload[7], payload[8]]),
        origin_seq: payload.get(9).copied(),
    })
}

/// PSOC_RESCUE_PROGRESS(0x09) 推送帧: 设备侧"PSoC 救砖"(强制重刷 + 重新应用)的阶段进度/终态。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PsocRescueProgress {
    /// 0=空闲 1=进行中 2=完成
    pub state: u8,
    /// 0=空闲 1=SWD 重刷中 2=重新应用(重下发算法/CSD) 3=完成 4=失败
    pub phase: u8,
    /// 0=进行中 1=成功 2=失败
    pub result: u8,
    /// bring-up 阶段码(PsocBringupStage: 8=擦除 9=写入 10=校验和 12=校验 13=运行 14=完成)
    pub stage: u8,
    /// 失败阶段码(0=无)
    pub fail_stage: u8,
}

impl PsocRescueProgress {
    /// 阶段中文名(供 UI 的"处理中"文案)。
    pub fn phase_text(&self) -> &'static str {
        match self.phase {
            1 => "SWD 重刷中",
            2 => "重新应用算法/CSD",
            3 => "完成",
            4 => "失败",
            _ => "已受理",
        }
    }
}

/// 解码 PSOC_RESCUE_PROGRESS 推送载荷。
/// payload = state(u8) + phase(u8) + result(u8) + stage(u8) + fail_stage(u8)
pub fn decode_psoc_rescue_progress(payload: &[u8]) -> Result<PsocRescueProgress, String> {
    if payload.len() < 5 {
        return Err(format!(
            "PSOC_RESCUE_PROGRESS payload must be >= 5 bytes, got {}",
            payload.len()
        ));
    }
    Ok(PsocRescueProgress {
        state: payload[0],
        phase: payload[1],
        result: payload[2],
        stage: payload[3],
        fail_stage: payload[4],
    })
}

/// 解码 CP_GET 响应载荷。
/// payload = channel(u8) + cp(u32 LE)，返回 `(channel, cp_ff)`。
pub fn decode_cp_get(payload: &[u8]) -> Result<(u8, u32), String> {
    if payload.len() != 5 {
        return Err(format!(
            "CP_GET response must be 5 bytes, got {}",
            payload.len()
        ));
    }
    Ok((
        payload[0],
        u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]),
    ))
}

/// 解码 PARAM_GET_ALL 响应载荷
/// payload = channel(u8) + count(u8) + [param_id(u8) + value(u32 LE)]×count
/// 返回 (channel, Vec<(param_id, value)>)
pub fn decode_param_get_all(payload: &[u8]) -> Result<(u8, Vec<(u8, u32)>), String> {
    if payload.len() < 2 {
        return Err("PARAM_GET_ALL response too short (need >= 2 bytes)".to_string());
    }

    let ch = payload[0];
    let count = payload[1] as usize;

    let mut params = Vec::new();
    let mut pos = 2;

    for _ in 0..count {
        if pos + 5 > payload.len() {
            return Err(format!(
                "PARAM_GET_ALL truncated: expected {} entries, got {}",
                count,
                params.len()
            ));
        }

        let param_id = payload[pos];
        pos += 1;
        let value = (payload[pos] as u32)
            | ((payload[pos + 1] as u32) << 8)
            | ((payload[pos + 2] as u32) << 16)
            | ((payload[pos + 3] as u32) << 24);
        pos += 4;

        params.push((param_id, value));
    }

    Ok((ch, params))
}

// ============================================================================
// 单元测试
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// 时钟源(SNS_CLK_SOURCE 0x0A)必须呈现为 0..6 + 「AUTO」, 且界面上不出现 128 这种跳跃值。
    /// ★这条锁住的是"用户填不出非法值"★: 该项 ui_max()=6|0x80=134, 区间里 7..127 全非法,
    /// 数字框必然让人填出 26(实测发生过) —— 单选是唯一能从根上排除它的表达方式。
    #[test]
    fn test_clk_source_choices_are_contiguous_plus_auto() {
        let fence = param_fence(0x0A);
        let choices = fence.ui_choices();
        let labels: Vec<&str> = choices.iter().map(|(t, _)| t.as_str()).collect();
        assert_eq!(
            labels,
            vec!["0", "1", "2", "3", "4", "5", "6", "AUTO"],
            "时钟源选项应为 0..6 + AUTO"
        );
        // AUTO 项的实际下发值必须带上标志位(设备合法持有 128), 而界面上只显示 "AUTO"。
        assert_eq!(choices.last().unwrap().1, 0x80, "AUTO 的下发值应为 0x80");
        assert!(
            !labels.contains(&"128"),
            "界面选项里绝不允许出现 128 这种不连续的裸值"
        );
        // 设备真值 → 下标: 128 = AUTO(最后一项); 3 = 第 4 项; 26 非法 ⇒ -1(界面如实报"值非法")。
        assert_eq!(fence.ui_choice_index(0x80), 7);
        assert_eq!(fence.ui_choice_index(3), 3);
        assert_eq!(fence.ui_choice_index(26), -1, "非法值不许假装选中某一项");
    }

    /// 可选值 ≥ UI_CHOICE_MAX 的参数仍走数字框(不能把 0..4095 摊成几千个选项)。
    #[test]
    fn test_wide_range_param_keeps_numeric_input() {
        // FINGER_TH(0x01) 是宽区间项。
        assert!(
            param_fence(0x01).ui_choices().is_empty(),
            "宽区间参数应回落数字输入框"
        );
    }

    #[test]
    fn test_encode_telem_start_byte_layout() {
        // 构造已知值: mode=0, rate_hz=1000(0x03E8), fields=RAW|BASELINE|DIFF|STATUS, ch_mask=0x0000000100000001
        let mode = 0u8;
        let rate_hz = 1000u16;
        let fields = FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS;
        let ch_mask = 0x0000000100000001u64;

        let payload = encode_telem_start(mode, rate_hz, fields, ch_mask);

        assert_eq!(payload.len(), 12, "TELEM_START payload should be 12 bytes");
        assert_eq!(payload[0], mode, "mode");
        assert_eq!(payload[1], 0xE8, "rate_hz low byte");
        assert_eq!(payload[2], 0x03, "rate_hz high byte");
        assert_eq!(payload[3], fields, "fields");
        assert_eq!(payload[4], 0x01, "ch_mask byte 0");
        assert_eq!(payload[5], 0x00, "ch_mask byte 1");
        assert_eq!(payload[6], 0x00, "ch_mask byte 2");
        assert_eq!(payload[7], 0x00, "ch_mask byte 3");
        assert_eq!(payload[8], 0x01, "ch_mask byte 4");
        assert_eq!(payload[9], 0x00, "ch_mask byte 5");
        assert_eq!(payload[10], 0x00, "ch_mask byte 6");
        assert_eq!(payload[11], 0x00, "ch_mask byte 7");
    }

    #[test]
    fn test_encode_param_set_byte_layout() {
        // 构造: ch=5, param_id=PARAM_FINGER_TH(0x01), value=100(0x00000064)
        let ch = 5u8;
        let param_id = PARAM_FINGER_TH;
        let value = 100u32;

        let payload = encode_param_set(ch, param_id, value);

        assert_eq!(payload.len(), 6, "PARAM_SET payload should be 6 bytes");
        assert_eq!(payload[0], ch);
        assert_eq!(payload[1], param_id);
        assert_eq!(payload[2], 0x64, "value byte 0");
        assert_eq!(payload[3], 0x00, "value byte 1");
        assert_eq!(payload[4], 0x00, "value byte 2");
        assert_eq!(payload[5], 0x00, "value byte 3");
    }

    #[test]
    fn test_decode_telem_data_simple() {
        // 构造: ts_us=0x12345678, ch_count=2, fields=RAW|DIFF
        // 通道0: raw=0x1234, diff=0x5678
        // 通道5: raw=0xABCD, diff=0xEF00
        let mut payload = Vec::new();

        // ts_us(u32 LE) = 0x12345678
        payload.push(0x78);
        payload.push(0x56);
        payload.push(0x34);
        payload.push(0x12);

        // ch_count=2
        payload.push(2);

        // fields = RAW | DIFF = 0x01 | 0x04 = 0x05
        payload.push(0x05);

        // Channel 0
        payload.push(0); // ch_index
        payload.push(0x34); // raw low
        payload.push(0x12); // raw high
        payload.push(0x78); // diff low
        payload.push(0x56); // diff high

        // Channel 5
        payload.push(5); // ch_index
        payload.push(0xCD); // raw low
        payload.push(0xAB); // raw high
        payload.push(0x00); // diff low
        payload.push(0xEF); // diff high

        let frame = decode_telem_data(&payload).expect("decode should succeed");

        assert_eq!(frame.ts_us, 0x12345678);
        assert_eq!(frame.fields, 0x05);
        assert_eq!(frame.samples.len(), 2);

        // Check channel 0
        assert_eq!(frame.samples[0].ch, 0);
        assert_eq!(frame.samples[0].raw, Some(0x1234));
        assert_eq!(frame.samples[0].bsln, None);
        assert_eq!(frame.samples[0].diff, Some(0x5678i16));
        assert_eq!(frame.samples[0].status, None);

        // Check channel 5
        assert_eq!(frame.samples[1].ch, 5);
        assert_eq!(frame.samples[1].raw, Some(0xABCD));
        assert_eq!(frame.samples[1].bsln, None);
        assert_eq!(frame.samples[1].diff, Some(-4352i16)); // 0xEF00 as i16
        assert_eq!(frame.samples[1].status, None);
    }

    #[test]
    fn test_decode_telem_data_all_fields() {
        // 构造: 1 通道，所有字段
        let mut payload = Vec::new();

        // ts_us=0x11223344
        payload.push(0x44);
        payload.push(0x33);
        payload.push(0x22);
        payload.push(0x11);

        // ch_count=1
        payload.push(1);

        // fields = RAW | BASELINE | DIFF | STATUS = 0x0F
        payload.push(0x0F);

        // Channel 0
        payload.push(0); // ch_index
        payload.push(0x34); // raw low
        payload.push(0x12); // raw high
        payload.push(0x78); // bsln low
        payload.push(0x56); // bsln high
        payload.push(0x12); // diff low
        payload.push(0x34); // diff high
        payload.push(0x01); // status

        let frame = decode_telem_data(&payload).expect("decode should succeed");

        assert_eq!(frame.samples.len(), 1);
        let sample = &frame.samples[0];
        assert_eq!(sample.ch, 0);
        assert_eq!(sample.raw, Some(0x1234));
        assert_eq!(sample.bsln, Some(0x5678));
        assert_eq!(sample.diff, Some(0x3412i16));
        assert_eq!(sample.status, Some(0x01));
    }

    #[test]
    fn test_decode_param_get() {
        // 响应: ch=3, param_id=PARAM_FINGER_TH(0x01), value=250(0x000000FA)
        let mut payload = Vec::new();
        payload.push(3); // channel
        payload.push(PARAM_FINGER_TH);
        payload.push(0xFA); // value low
        payload.push(0x00);
        payload.push(0x00);
        payload.push(0x00);

        let (ch, param_id, value) = decode_param_get(&payload).expect("decode should succeed");

        assert_eq!(ch, 3);
        assert_eq!(param_id, PARAM_FINGER_TH);
        assert_eq!(value, 250);
    }

    #[test]
    fn test_decode_param_get_all() {
        // 响应: ch=7, count=3, params: (PARAM_FINGER_TH, 100), (PARAM_NOISE_TH, 40), (PARAM_HYSTERESIS, 10)
        let mut payload = Vec::new();
        payload.push(7); // channel
        payload.push(3); // count

        // Entry 0: param_id=PARAM_FINGER_TH, value=100
        payload.push(PARAM_FINGER_TH);
        payload.push(100);
        payload.push(0);
        payload.push(0);
        payload.push(0);

        // Entry 1: param_id=PARAM_NOISE_TH, value=40
        payload.push(PARAM_NOISE_TH);
        payload.push(40);
        payload.push(0);
        payload.push(0);
        payload.push(0);

        // Entry 2: param_id=PARAM_HYSTERESIS, value=10
        payload.push(PARAM_HYSTERESIS);
        payload.push(10);
        payload.push(0);
        payload.push(0);
        payload.push(0);

        let (ch, params) = decode_param_get_all(&payload).expect("decode should succeed");

        assert_eq!(ch, 7);
        assert_eq!(params.len(), 3);
        assert_eq!(params[0], (PARAM_FINGER_TH, 100));
        assert_eq!(params[1], (PARAM_NOISE_TH, 40));
        assert_eq!(params[2], (PARAM_HYSTERESIS, 10));
    }

    #[test]
    fn test_known_param_ids_count() {
        // 应该有 12 个已知 param_id(0x0C = 通道启用开关)
        assert_eq!(KNOWN_PARAM_IDS.len(), 12);
        assert_eq!(KNOWN_PARAM_IDS[0], PARAM_FINGER_TH);
        assert_eq!(KNOWN_PARAM_IDS[10], PARAM_IDAC_GAIN);
        assert_eq!(KNOWN_PARAM_IDS[11], PARAM_ENABLED);
        assert_eq!(
            KNOWN_PARAM_IDS,
            &[
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C
            ]
        );
    }

    /// 通道启用开关只接受 0/1, 且与固件两处的 `value <= 1` 逐位等价。
    #[test]
    fn test_enabled_fence_is_boolean() {
        assert!(param_value_legal(PARAM_ENABLED, 0));
        assert!(param_value_legal(PARAM_ENABLED, 1));
        assert!(!param_value_legal(PARAM_ENABLED, 2));
        assert_eq!(param_clamp(PARAM_ENABLED, 9), 1);
        let fence = param_fence(PARAM_ENABLED);
        assert!(fence.guarded);
        assert_eq!((fence.ui_min(), fence.ui_max()), (0, 1));
    }

    #[test]
    fn test_encode_ch_mask_all_channels() {
        // ch_mask = 0xFFFFFFFFFFFFFFFF (所有 64 位都置 1)
        let payload = encode_ch_mask(0xFFFFFFFFFFFFFFFF);
        assert_eq!(payload.len(), 8);
        assert!(payload.iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn test_encode_ch_mask_sparse() {
        // ch_mask = 0x0000000100000001 (通道 0 和 32)
        let payload = encode_ch_mask(0x0000000100000001u64);
        assert_eq!(payload.len(), 8);
        assert_eq!(payload[0], 0x01);
        assert_eq!(payload[4], 0x01);
        assert!(payload[1..4].iter().all(|&b| b == 0));
        assert!(payload[5..8].iter().all(|&b| b == 0));
    }

    #[test]
    fn test_decode_telem_data_truncated_payload_error() {
        let payload = vec![0x01, 0x02];
        let result = decode_telem_data(&payload);
        assert!(result.is_err());
    }

    #[test]
    fn test_decode_param_get_truncated_error() {
        let payload = vec![0x05, 0x01, 0x64];
        let result = decode_param_get(&payload);
        assert!(result.is_err());
    }

    /// ★本次围栏收敛的核心回归★: SNS_CLK_SOURCE 的 0x80 是 CapSense AUTO 标志, 设备合法持有 128。
    /// 旧实现(slint 里写死 max=6)会把 128 夹成 6, 用户一碰这一行就把 AUTO 标志抹掉。
    /// 这里逐条钉死: 判定放行、clamp 恒等、UI 上界容得下。
    #[test]
    fn test_sns_clk_source_auto_flag_survives() {
        // 128 = AUTO 标志 + 源 0 ⇒ 合法(固件判的是 `(value & 0x7F) <= 6`)。
        assert!(param_value_legal(PARAM_SNS_CLK_SOURCE, 128));
        assert_eq!(param_clamp(PARAM_SNS_CLK_SOURCE, 128), 128);
        // 带标志位的其余合法源同样恒等。
        for src in 0u32..=6u32 {
            assert!(param_value_legal(PARAM_SNS_CLK_SOURCE, 0x80 | src));
            assert_eq!(param_clamp(PARAM_SNS_CLK_SOURCE, 0x80 | src), 0x80 | src);
            assert_eq!(param_clamp(PARAM_SNS_CLK_SOURCE, src), src);
        }
        // 非法值只夹值域部分, 标志位原样保留。
        assert!(!param_value_legal(PARAM_SNS_CLK_SOURCE, 0x80 | 7));
        assert_eq!(param_clamp(PARAM_SNS_CLK_SOURCE, 0x80 | 7), 0x80 | 6);
        assert_eq!(param_clamp(PARAM_SNS_CLK_SOURCE, 7), 6);
        // UI 上界必须容得下"值域上界 + 标志位", 否则 SpinBox 又会夹掉 AUTO。
        let fence = param_fence(PARAM_SNS_CLK_SOURCE);
        assert!(
            fence.ui_max() >= 128,
            "UI 上界必须 >= 128 才装得下 AUTO 标志"
        );
        assert_eq!(fence.ui_max(), 6 | 0x80);
        assert_eq!(fence.ui_min(), 0);
    }

    /// 硬拦截项与固件 `_param_value_legal` / `sensor_link.cpp` 的边界逐位对齐。
    #[test]
    fn test_guarded_param_fence_boundaries() {
        // RESOLUTION 6..16
        assert!(!param_value_legal(PARAM_RESOLUTION, 5));
        assert!(param_value_legal(PARAM_RESOLUTION, 6));
        assert!(param_value_legal(PARAM_RESOLUTION, 16));
        assert!(!param_value_legal(PARAM_RESOLUTION, 17));
        assert_eq!(param_clamp(PARAM_RESOLUTION, 0), 6);
        assert_eq!(param_clamp(PARAM_RESOLUTION, 99), 16);
        // SNS_CLK_DIV 1..255 (0 会除零)
        assert!(!param_value_legal(PARAM_SNS_CLK_DIV, 0));
        assert!(param_value_legal(PARAM_SNS_CLK_DIV, 1));
        assert!(param_value_legal(PARAM_SNS_CLK_DIV, 255));
        assert!(!param_value_legal(PARAM_SNS_CLK_DIV, 256));
        assert_eq!(param_clamp(PARAM_SNS_CLK_DIV, 0), 1);
        // IDAC_MOD 0..127
        assert!(param_value_legal(PARAM_IDAC_MOD, 127));
        assert!(!param_value_legal(PARAM_IDAC_MOD, 128));
        assert_eq!(param_clamp(PARAM_IDAC_MOD, 200), 127);
        // IDAC_GAIN 0..6 (增益表 7 项, 索引 7 会让 PSoC 崩溃)
        assert!(param_value_legal(PARAM_IDAC_GAIN, 6));
        assert!(!param_value_legal(PARAM_IDAC_GAIN, 7));
        assert_eq!(param_clamp(PARAM_IDAC_GAIN, 7), 6);
    }

    /// 固件 `default: return true` 的项一律放行 —— 上位机不许比固件更严, 否则会拒掉设备本来接受的值。
    #[test]
    fn test_unguarded_params_always_legal() {
        for &id in &[
            PARAM_FINGER_TH,
            PARAM_NOISE_TH,
            PARAM_NEG_NOISE_TH,
            PARAM_HYSTERESIS,
            PARAM_ON_DEBOUNCE,
            PARAM_LOW_BSLN_RST,
        ] {
            assert!(!param_fence(id).guarded, "0x{:02X} 固件不拦截", id);
            for v in [0u32, 1, 255, 256, 65535, 0xFFFF_FFFF] {
                assert!(param_value_legal(id, v), "0x{:02X} = {} 必须放行", id, v);
                assert_eq!(param_clamp(id, v), v, "放行项不许改值");
            }
        }
        // 未知 param_id 同样放行(复刻固件 default)。
        assert!(param_value_legal(0x7F, 0xFFFF_FFFF));
    }

    /// 表里每一项都必须有名字与自洽的区间, 且 KNOWN_PARAM_IDS 全部有声明。
    #[test]
    fn test_param_fence_table_selfconsistent() {
        for &id in KNOWN_PARAM_IDS {
            let f = param_fence(id);
            assert_ne!(f.name, "PARAM_UNKNOWN", "0x{:02X} 缺围栏声明", id);
            assert!(f.min <= f.max, "0x{:02X} 区间反了", id);
            assert_eq!(
                f.value_mask & f.flag_mask,
                0,
                "0x{:02X} 值域与标志位重叠",
                id
            );
            assert_eq!(f.max & f.value_mask, f.max, "0x{:02X} 上界超出取位掩码", id);
            // 极值本身必须合法, 且 clamp 是幂等的。
            assert_eq!(param_clamp(id, f.min), f.min);
            assert_eq!(param_clamp(id, f.max), f.max);
            let c = param_clamp(id, 0xFFFF_FFFF);
            assert_eq!(param_clamp(id, c), c, "0x{:02X} clamp 非幂等", id);
            assert!(param_value_legal(id, c), "0x{:02X} clamp 结果仍非法", id);
        }
    }
}

/// PARAM_GET_ALL 的"全通道单参数"标记(请求与响应的 payload[0])。
pub const PARAM_ALL_CHANNELS: u8 = 0xFF;

/// 解码 PARAM_GET_ALL 的"全通道单参数"响应载荷
/// payload = 0xFF + param_id(u8) + count(u8) + [channel(u8) + value(u32 LE)]×count
/// 返回 (param_id, Vec<(channel, value)>)
pub fn decode_param_get_all_channels(payload: &[u8]) -> Result<(u8, Vec<(u8, u32)>), String> {
    if payload.len() < 3 || payload[0] != PARAM_ALL_CHANNELS {
        return Err("PARAM_GET_ALL(all channels) response malformed".to_string());
    }
    let param_id = payload[1];
    let count = payload[2] as usize;
    let mut values = Vec::with_capacity(count);
    let mut pos = 3;
    for _ in 0..count {
        if pos + 5 > payload.len() {
            return Err(format!(
                "PARAM_GET_ALL(all channels) truncated: expected {} entries, got {}",
                count,
                values.len()
            ));
        }
        let ch = payload[pos];
        let value = u32::from_le_bytes([
            payload[pos + 1],
            payload[pos + 2],
            payload[pos + 3],
            payload[pos + 4],
        ]);
        pos += 5;
        values.push((ch, value));
    }
    Ok((param_id, values))
}
