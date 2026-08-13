//! JIT 算法变量的中文别名与简介（**纯 UI 侧资产**）。
//!
//! ★为什么不放在固件/C 源里★
//! 别名与简介只服务于界面可读性，对算法的编译与运行没有任何影响。把它们写进 C 源就意味着：
//!   · 中文要随 C 源上传进设备 flash，再从设备回读、再被宏解析器切一遍 ——
//!     这条往返链上任何一处按字节处理都会产出 mojibake（实测踩到两处：去注释与宏参数解析）；
//!   · 设备被迫为纯展示文本付出存储与带宽；
//!   · 同一算法换一种界面语言就得改 C 源、重编译、重上传。
//! 因此约定：**固件侧只存 C 源与 ASM**，别名/简介由本表提供，用户覆盖存本地
//! `jit_metadata.json`（可导入导出分享，见 settings_io）。
//!
//! ★索引键是变量的英文 `name`，不是算法指纹★
//! `name` 是 C 源 `ALGO_REPORT_META/ALGO_SETTING_META` 里声明的稳定标识，语义唯一；
//! 用它做键使得：改动算法实现(指纹变化)不会丢别名；任何复用同名变量的自定义算法自动获得中文。
//! 表里没有的 name ⇒ 界面回落到 C 源声明值，再回落到 name 本身；用户可自行设别名并导出分享。

/// (name, alias, description)
const REPORT_TEXT: &[(&str, &str, &str)] = &[
    (
        "diff",
        "有效差值",
        "中值滤波后的有效差值(raw-基线)，是判定的主输入；手指越靠近越大",
    ),
    (
        "env_hi",
        "高包络",
        "动态高包络 z_h：diff 近期峰值的跟踪值，用于按当前信号幅度自适应触发门限",
    ),
    (
        "env_lo",
        "低包络",
        "动态低包络 z_l：diff 近期谷值的跟踪值，代表当前静止底噪水平",
    ),
    (
        "margin",
        "判定余量",
        "当前 diff 距触发/释放门限还差多少：接近 0 表示即将翻转，可据此判断灵敏度是否合适",
    ),
    (
        "active",
        "触发状态",
        "最终触发判定(算法写出的 out_active)，与设备实际上报的触摸一致",
    ),
    (
        "led",
        "白灯状态",
        "算法请求的白灯状态：证明灯由算法驱动而非固件写死",
    ),
];

/// (name, alias, description)
const SETTING_TEXT: &[(&str, &str, &str)] = &[
    (
        "rise_permille",
        "上升触发",
        "按下门限，取包络跨度的千分比：调小更灵敏(易触发也更易误触)，调大更稳但需更实的按压；0=用算法内置默认",
    ),
    (
        "drop_permille",
        "下降释放",
        "抬起门限千分比，须大于上升值以形成回差防抖：调大更黏手(不易断触)，调小抬手更快；0=用算法内置默认",
    ),
    (
        "window_ms",
        "包络窗口",
        "包络跟踪时间窗：调大对慢速漂移更稳、响应略慢；调小跟手但易受瞬时噪声影响；0=用算法内置默认",
    ),
    (
        "bsln_offset",
        "基线偏移",
        "判定前叠加到 diff 的有符号偏移：正值整体更灵敏，负值整体更钝，用于补偿单通道底噪差异；0=中性",
    ),
];

fn _lookup(table: &[(&'static str, &'static str, &'static str)], name: &str) -> Option<(String, String)> {
    let key = name.trim();
    if key.is_empty() {
        return None;
    }
    table
        .iter()
        .find(|(n, _, _)| n.eq_ignore_ascii_case(key))
        .map(|(_, alias, description)| ((*alias).to_string(), (*description).to_string()))
}

/// 上报变量(report)的内置中文文本；无内置条目返回 None。
pub fn report_text(name: &str) -> Option<(String, String)> {
    _lookup(REPORT_TEXT, name)
}

/// 可调变量(setting/cfg)的内置中文文本；无内置条目返回 None。
pub fn setting_text(name: &str) -> Option<(String, String)> {
    _lookup(SETTING_TEXT, name)
}

/// kind: 0=上报变量, 1=可调变量。与 set_algo_metadata_override 的 kind 同一口径。
pub fn text_for(kind: u8, name: &str) -> Option<(String, String)> {
    if kind == 1 {
        setting_text(name)
    } else {
        report_text(name)
    }
}
