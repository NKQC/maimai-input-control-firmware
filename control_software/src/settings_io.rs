//! 设备设置的 JSON 导入/导出。
//!
//! 不引入 serde：设置文件的结构固定且规模有限，保留最小 JSON 实现可避免为单一功能
//! 增加依赖树；解析错误一律携带字节位置，导入文件损坏时不会 panic。

use std::collections::BTreeMap;
use std::path::PathBuf;

use anyhow::{Result, anyhow, bail};

use crate::app_state::{AppController, ConnState, HID_COORD_MAX, HID_POINT_COUNT, zone_key};
use crate::proto::{
    CfgValue, ConfigEntry, KBD_HOLD_KIND_PHYS, KBD_HOLD_KIND_ZONE, KNOWN_PARAM_IDS,
};

pub const GROUP_CONFIG: &str = "config";
pub const GROUP_CHANNEL_PARAMS: &str = "channel_params";
pub const GROUP_GLOBALS: &str = "globals";
pub const GROUP_ALGO: &str = "algo";
pub const GROUP_KEYBOARD: &str = "keyboard";
pub const GROUP_ZONES: &str = "zones";
const FORMAT: &str = "mai2control-settings";
const VERSION: u32 = 1;

/// 六个可独立导入/导出的设置组；默认全选，避免用户无意导出不完整快照。
#[derive(Clone, Copy, Debug)]
pub struct GroupSelection {
    pub config: bool,
    pub channel_params: bool,
    pub globals: bool,
    pub algo: bool,
    pub keyboard: bool,
    pub zones: bool,
}

impl Default for GroupSelection {
    fn default() -> Self {
        Self {
            config: true,
            channel_params: true,
            globals: true,
            algo: true,
            keyboard: true,
            zones: true,
        }
    }
}

impl GroupSelection {
    pub fn any(self) -> bool {
        self.config
            || self.channel_params
            || self.globals
            || self.algo
            || self.keyboard
            || self.zones
    }
}

/// 弹窗显示的实际覆盖数量。配置项数量随已读取的设备 schema 变化。
#[derive(Clone, Copy, Debug, Default)]
pub struct GroupCounts {
    pub config: i32,
    pub channel_params: i32,
    pub globals: i32,
    pub algo: i32,
    pub keyboard: i32,
    pub zones: i32,
}

pub fn group_counts(ctrl: &AppController) -> GroupCounts {
    GroupCounts {
        config: ctrl
            .config_entries()
            .into_iter()
            .filter(|e| _cfg_group_owns(&e.key))
            .count() as i32,
        channel_params: (36 * KNOWN_PARAM_IDS.len()) as i32,
        // gparam 1..8 + CSD 处理模式枚举。
        globals: 9,
        // 算法源、机器码与 cfg[0..7]。
        algo: 10,
        // 12 个物理键 + 34 个触控分区键(各含键码、修饰位与长按参数) + 现有组合映射条数。
        // ★组合映射按实际条数计★: 它是变长表, 写死常数会让勾选框上的数字与实际导出内容不符。
        keyboard: 46 + ctrl.kbd_combos().len() as i32,
        // 34 个 Serial 分区绑定 + 36 个 HID 触控点位(两套独立映射, 同组并列导出, 见 _export_zones)。
        zones: 34 + HID_POINT_COUNT as i32,
    }
}

/// 单项被拒的原因分类。导入绝不静默丢弃、绝不静默钳位：拒掉的项一律带原因计入摘要。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum SkipReason {
    /// 设备当前配置里没有这个键(旧文件、别的固件版本或手写笔误)。
    UnknownKey,
    /// 文件里的值无法无损表达为设备声明的类型。
    TypeMismatch,
    /// 值超出设备声明的 min/max，或索引越界。
    OutOfRange,
    /// 该项不经过草稿层，导入不代它下发(算法 C 源与机器码)。
    NotDraftable,
    /// 该项的 JSON 字段缺失或格式非法。
    Malformed,
}

impl SkipReason {
    pub fn label(self) -> &'static str {
        match self {
            SkipReason::UnknownKey => "设备无此配置项",
            SkipReason::TypeMismatch => "类型不符",
            SkipReason::OutOfRange => "超出允许范围",
            SkipReason::NotDraftable => "不属于草稿范围, 需在算法页手动上传",
            SkipReason::Malformed => "字段缺失或格式非法",
        }
    }
}

/// 导入完成摘要。导入只写草稿，故这里描述的是"草稿被覆盖到什么程度"，而非设备已生效范围。
#[derive(Default)]
pub struct ImportSummary {
    pub applied_groups: Vec<&'static str>,
    /// 已被文件值覆盖的草稿项数(含与设备真值相同因而不产生脏标记的项)。
    pub applied_items: usize,
    /// 文件里没给值的项数：保持草稿现值，绝不用默认值覆盖设备真值。
    pub absent_items: usize,
    /// 被拒项 `(名称, 原因)`。
    pub skipped: Vec<(String, SkipReason)>,
    /// 导入前后的未保存项数，用于告知实际产生了多少改动。
    pub dirty_before: i32,
    pub dirty_after: i32,
}

impl ImportSummary {
    fn _skip(&mut self, item: impl Into<String>, reason: SkipReason) {
        self.skipped.push((item.into(), reason));
    }

    /// 按原因归并的跳过统计，原因内部按名称保持文件出现顺序。
    pub fn skipped_by_reason(&self) -> Vec<(SkipReason, Vec<&str>)> {
        let mut grouped: BTreeMap<SkipReason, Vec<&str>> = BTreeMap::new();
        for (item, reason) in &self.skipped {
            grouped.entry(*reason).or_default().push(item.as_str());
        }
        grouped.into_iter().collect()
    }

    /// 面向用户的一句话结论：覆盖多少、跳过多少(带原因)、当前处于未保存状态。
    /// 每类原因最多列 6 个具体项名，避免上百项时把日志刷满。
    pub fn report_text(&self) -> String {
        let mut text = format!(
            "导入完成: 已覆盖 {} 项草稿({} 组: {})，当前未保存项 {} 项(导入前 {} 项)。\
             导入只改界面草稿，未向设备发送任何命令，也未写 flash；请点“保存到设备”才真正生效。",
            self.applied_items,
            self.applied_groups.len(),
            self.applied_groups.join("/"),
            self.dirty_after,
            self.dirty_before,
        );
        if self.absent_items > 0 {
            text.push_str(&format!(
                " 文件中有 {} 项未带值(导出时设备尚未回读到)，这些项保持当前值不动。",
                self.absent_items
            ));
        }
        let grouped = self.skipped_by_reason();
        if !grouped.is_empty() {
            text.push_str(&format!(" 跳过 {} 项: ", self.skipped.len()));
            let parts: Vec<String> = grouped
                .iter()
                .map(|(reason, items)| {
                    let shown: Vec<&str> = items.iter().take(6).copied().collect();
                    let more = if items.len() > shown.len() {
                        format!(" 等 {} 项", items.len())
                    } else {
                        String::new()
                    };
                    format!(
                        "{}({} 项: {}{})",
                        reason.label(),
                        items.len(),
                        shown.join(", "),
                        more
                    )
                })
                .collect();
            text.push_str(&parts.join("；"));
            text.push('。');
        }
        text
    }
}

/// 按选择范围构造可读、可回灌的 JSON 文本。
pub fn export_settings(ctrl: &AppController, selected: GroupSelection) -> Result<String> {
    if !selected.any() {
        bail!("请至少选择一个设置组");
    }
    let mut root = BTreeMap::new();
    root.insert("format".to_string(), JsonValue::String(FORMAT.to_string()));
    root.insert("version".to_string(), JsonValue::Number(VERSION as f64));
    root.insert(
        "exported_at".to_string(),
        JsonValue::String(local_timestamp()),
    );

    let mut device = BTreeMap::new();
    let (fw_version, protocol) = ctrl
        .device_info()
        .map(|info| (info.fw_version, info.protocol_version))
        .unwrap_or((0, 0));
    device.insert(
        "fw_version".to_string(),
        JsonValue::Number(fw_version as f64),
    );
    device.insert("protocol".to_string(), JsonValue::Number(protocol as f64));
    root.insert("device".to_string(), JsonValue::Object(device));

    let mut groups = BTreeMap::new();
    if selected.config {
        groups.insert(GROUP_CONFIG.to_string(), _export_config(ctrl)?);
    }
    if selected.channel_params {
        groups.insert(
            GROUP_CHANNEL_PARAMS.to_string(),
            _export_channel_params(ctrl),
        );
    }
    if selected.globals {
        groups.insert(GROUP_GLOBALS.to_string(), _export_globals(ctrl));
    }
    if selected.algo {
        groups.insert(GROUP_ALGO.to_string(), _export_algo(ctrl));
    }
    if selected.keyboard {
        groups.insert(GROUP_KEYBOARD.to_string(), _export_keyboard(ctrl));
    }
    if selected.zones {
        groups.insert(GROUP_ZONES.to_string(), _export_zones(ctrl));
    }
    root.insert("groups".to_string(), JsonValue::Object(groups));

    let mut text = String::new();
    _write_json(&JsonValue::Object(root), &mut text, 0)?;
    text.push('\n');
    Ok(text)
}

/// 解析并导入用户选择的组。
///
/// ★语义★: 导入**只写 UI 草稿**并置脏，不发送任何设备命令、不写 flash、不回读。真正下发只发生在
/// 用户点击“保存到设备”(`AppController::save_config`)时；“撤销未保存改动”会把草稿清回导入前的
/// 设备真值。为此本函数内不得出现任何 `handle.send` 路径的调用(算法 C 源/机器码的上传接口即属此类，
/// 故不再代为上传，只在摘要里说明)。
///
/// 逐项容错: 未知键 / 类型不符 / 越界一律拒绝该项并计入摘要，不静默丢弃也不静默钳位；单项失败
/// 不会中断其余项与其余组(旧实现 `bail!` 会让导入停在半途，是"覆盖不全"的直接原因)。
/// 文件里没给值的项保持草稿现值，绝不用默认值覆盖设备真值。
pub fn import_settings(
    ctrl: &mut AppController,
    text: &str,
    selected: GroupSelection,
) -> Result<ImportSummary> {
    if !selected.any() {
        bail!("请至少选择一个设置组");
    }
    if ctrl.state() != ConnState::Connected {
        bail!("设备未连接，无法导入设置；请连接设备后重试");
    }

    let root = JsonParser::new(text).parse()?;
    let root = _object(&root, "根对象")?;
    let format = _string(_required(root, "format", "根对象")?, "format")?;
    if format != FORMAT {
        bail!("不是 mai2control 设置文件(format={format:?})");
    }
    let version = _u32(_required(root, "version", "根对象")?, "version")?;
    if version != VERSION {
        bail!("不支持的设置文件版本 {version}，当前仅支持 {VERSION}");
    }
    let groups = _object(_required(root, "groups", "根对象")?, "groups")?;

    // 组缺失属于文件级问题(选了却没有), 先全部检查再落任何草稿, 避免"部分应用"。
    let mut wanted: Vec<(&'static str, &JsonValue)> = Vec::new();
    if selected.config {
        wanted.push((GROUP_CONFIG, _group(groups, GROUP_CONFIG)?));
    }
    if selected.channel_params {
        wanted.push((GROUP_CHANNEL_PARAMS, _group(groups, GROUP_CHANNEL_PARAMS)?));
    }
    if selected.globals {
        wanted.push((GROUP_GLOBALS, _group(groups, GROUP_GLOBALS)?));
    }
    if selected.algo {
        wanted.push((GROUP_ALGO, _group(groups, GROUP_ALGO)?));
    }
    if selected.keyboard {
        wanted.push((GROUP_KEYBOARD, _group(groups, GROUP_KEYBOARD)?));
    }
    if selected.zones {
        wanted.push((GROUP_ZONES, _group(groups, GROUP_ZONES)?));
    }

    let mut summary = ImportSummary {
        dirty_before: ctrl.config_dirty_count(),
        ..Default::default()
    };
    for (name, group) in wanted {
        match name {
            GROUP_CONFIG => _import_config(ctrl, group, &mut summary)?,
            GROUP_CHANNEL_PARAMS => _import_channel_params(ctrl, group, &mut summary)?,
            GROUP_GLOBALS => _import_globals(ctrl, group, &mut summary)?,
            GROUP_ALGO => _import_algo(ctrl, group, &mut summary)?,
            GROUP_KEYBOARD => _import_keyboard(ctrl, group, &mut summary)?,
            _ => _import_zones(ctrl, group, &mut summary)?,
        }
        summary.applied_groups.push(name);
    }

    // ★数据源统一★: 草稿是外部批量写入的, 没有经过任何控件, 必须显式让 16ms tick 的既有
    // `*_version` 门控判定为"有变化", 否则配置表/参数/全局/键盘各页会继续显示导入前的值。
    // 只 bump 版本号, 不改任何轮询频率, 也不发任何命令。
    ctrl.refresh_draft_views();
    summary.dirty_after = ctrl.config_dirty_count();
    Ok(summary)
}

fn _export_config(ctrl: &AppController) -> Result<JsonValue> {
    let mut entries = Vec::new();
    for entry in ctrl.config_entries() {
        // bind.mapNN / kbd.* 由 zones / keyboard 组独占，确保同一项只有一个导入出口。
        if !_cfg_group_owns(&entry.key) {
            continue;
        }
        let mut item = BTreeMap::new();
        item.insert("key".to_string(), JsonValue::String(entry.key));
        item.insert(
            "type".to_string(),
            JsonValue::String(_cfg_type_name(&entry.value).to_string()),
        );
        item.insert("value".to_string(), _cfg_to_json(&entry.value)?);
        if let Some((min, max)) = entry.range {
            let mut range = BTreeMap::new();
            range.insert("min".to_string(), _cfg_to_json(&min)?);
            range.insert("max".to_string(), _cfg_to_json(&max)?);
            item.insert("range".to_string(), JsonValue::Object(range));
        }
        entries.push(JsonValue::Object(item));
    }
    let mut group = BTreeMap::new();
    group.insert("entries".to_string(), JsonValue::Array(entries));
    Ok(JsonValue::Object(group))
}

fn _export_channel_params(ctrl: &AppController) -> JsonValue {
    let mut channels = Vec::with_capacity(36);
    for channel in 0..36u8 {
        let mut params = Vec::with_capacity(KNOWN_PARAM_IDS.len());
        for &param_id in KNOWN_PARAM_IDS {
            let mut param = BTreeMap::new();
            param.insert("param_id".to_string(), JsonValue::Number(param_id as f64));
            if let Some(value) = ctrl.param(channel, param_id) {
                param.insert("value".to_string(), JsonValue::Number(value as f64));
            }
            params.push(JsonValue::Object(param));
        }
        let mut item = BTreeMap::new();
        item.insert("channel".to_string(), JsonValue::Number(channel as f64));
        item.insert("params".to_string(), JsonValue::Array(params));
        channels.push(JsonValue::Object(item));
    }
    let mut group = BTreeMap::new();
    group.insert("channels".to_string(), JsonValue::Array(channels));
    JsonValue::Object(group)
}

fn _export_globals(ctrl: &AppController) -> JsonValue {
    let mut values = Vec::with_capacity(8);
    for id in 1..=8u8 {
        let mut item = BTreeMap::new();
        item.insert("gparam_id".to_string(), JsonValue::Number(id as f64));
        if let Some(value) = ctrl.global(id) {
            item.insert("value".to_string(), JsonValue::Number(value as f64));
        }
        values.push(JsonValue::Object(item));
    }
    let mut group = BTreeMap::new();
    group.insert("values".to_string(), JsonValue::Array(values));
    // CSD 处理模式(0=自动校准 1=半自动手动)不是 config 键值, 走 MODE_SET 独立命令, 但它与
    // gparam 1..8 同属"全局 CSD"且同在触控全局调整页, 故归入本组。旧文件没有该字段时按"文件缺键"
    // 处理(保持当前值不动)。未回读到设备真模式时不导出, 免得把未知当成 AUTO 写进文件。
    if let Some(mode) = ctrl.csd_mode_effective() {
        group.insert("csd_mode".to_string(), JsonValue::Number(mode as f64));
    }
    JsonValue::Object(group)
}

fn _export_algo(ctrl: &AppController) -> JsonValue {
    let mut group = BTreeMap::new();
    group.insert(
        "device_source".to_string(),
        JsonValue::String(ctrl.algo_device_src().to_string()),
    );
    group.insert(
        "schema_source".to_string(),
        JsonValue::String(ctrl.algo_schema_source().to_string()),
    );
    let mut metadata = Vec::new();
    for decl in ctrl.algo_report_decls() {
        let mut item = BTreeMap::new();
        item.insert("kind".to_string(), JsonValue::String("report".to_string()));
        item.insert("index".to_string(), JsonValue::Number(decl.idx as f64));
        item.insert("name".to_string(), JsonValue::String(decl.name));
        item.insert("type".to_string(), JsonValue::String(decl.value_type));
        item.insert("range".to_string(), JsonValue::String(decl.range));
        item.insert(
            "description".to_string(),
            JsonValue::String(decl.description),
        );
        item.insert("alias".to_string(), JsonValue::String(decl.alias));
        metadata.push(JsonValue::Object(item));
    }
    for decl in ctrl.algo_setting_decls() {
        let mut item = BTreeMap::new();
        item.insert("kind".to_string(), JsonValue::String("setting".to_string()));
        item.insert("index".to_string(), JsonValue::Number(decl.idx as f64));
        item.insert("name".to_string(), JsonValue::String(decl.name));
        item.insert("type".to_string(), JsonValue::String(decl.value_type));
        item.insert("range".to_string(), JsonValue::String(decl.range));
        item.insert(
            "description".to_string(),
            JsonValue::String(decl.description),
        );
        item.insert("alias".to_string(), JsonValue::String(decl.alias));
        item.insert(
            "default".to_string(),
            JsonValue::Number(decl.default as f64),
        );
        item.insert(
            "current".to_string(),
            JsonValue::Number(ctrl.algo_cfg(decl.idx) as f64),
        );
        metadata.push(JsonValue::Object(item));
    }
    group.insert("metadata".to_string(), JsonValue::Array(metadata));
    group.insert(
        "device_code_hex".to_string(),
        JsonValue::String(ctrl.algo_device_code_hex().to_string()),
    );
    let mut cfg = Vec::with_capacity(8);
    for idx in 0..8u8 {
        let mut item = BTreeMap::new();
        item.insert("index".to_string(), JsonValue::Number(idx as f64));
        item.insert(
            "value".to_string(),
            JsonValue::Number(ctrl.algo_cfg(idx) as f64),
        );
        cfg.push(JsonValue::Object(item));
    }
    group.insert("cfg".to_string(), JsonValue::Array(cfg));
    JsonValue::Object(group)
}

fn _export_keyboard(ctrl: &AppController) -> JsonValue {
    // 键码/修饰位/长按参数三者同属一个按键项: kbd.keyNN/kmNN/hdNN/mhNN 与
    // kbd.zoneNN/zmNN/zhdNN/zmhNN 全部由本组独占导出(config 组已排除), 单一出口不会两处打架。
    let mut physical = Vec::with_capacity(12);
    for index in 0..12u8 {
        let hold = ctrl.kbd_hold_phys(index);
        physical.push(_key_entry(
            index,
            ctrl.kbd_map(index),
            ctrl.kbd_keymod(index),
            hold,
        ));
    }
    let mut touch = Vec::with_capacity(34);
    for zone in 0..34u8 {
        let hold = ctrl.kbd_hold_zone(zone);
        touch.push(_key_entry(
            zone,
            ctrl.kbd_touch_keycode(zone),
            ctrl.kbd_zone_mod(zone),
            hold,
        ));
    }
    // ★组合映射(触控键盘映射页的"若干分区同时按下 → 若干键同时按下")★
    // 原先整张表既不导出也不导入: 它不是逐分区的一对一映射, 不落在 physical/touch 任何一类里,
    // 而 `_is_keyboard_key` 又没收 `kbd.cb`, 于是它被当成普通 KV 混进 config 组 —— 那是打包过的
    // 裸字节, 导入回去 UI 的组合表缓存并不会跟着更新, 表现就是"导出再导入, 组合映射没了"。
    // 现在由本组独占, 与 physical/touch 同一个出口。
    let mut combo = Vec::new();
    for item in ctrl.kbd_combos() {
        let mut obj = BTreeMap::new();
        obj.insert(
            "zone_mask".to_string(),
            JsonValue::Number(item.zone_mask as f64),
        );
        obj.insert(
            "keycodes".to_string(),
            JsonValue::Array(
                item.keycodes
                    .iter()
                    .map(|k| JsonValue::Number(*k as f64))
                    .collect(),
            ),
        );
        obj.insert(
            "modifiers".to_string(),
            JsonValue::Number(item.modifiers as f64),
        );
        obj.insert(
            "delay_ms".to_string(),
            JsonValue::Number(item.delay_ms as f64),
        );
        obj.insert(
            "max_hold_ms".to_string(),
            JsonValue::Number(item.max_hold_ms as f64),
        );
        combo.push(JsonValue::Object(obj));
    }
    let mut group = BTreeMap::new();
    group.insert("physical".to_string(), JsonValue::Array(physical));
    group.insert("touch".to_string(), JsonValue::Array(touch));
    group.insert("combo".to_string(), JsonValue::Array(combo));
    JsonValue::Object(group)
}

fn _key_entry(index: u8, keycode: u8, modifier: u8, hold: (u16, u16)) -> JsonValue {
    let mut item = BTreeMap::new();
    item.insert("index".to_string(), JsonValue::Number(index as f64));
    item.insert("keycode".to_string(), JsonValue::Number(keycode as f64));
    item.insert("modifier".to_string(), JsonValue::Number(modifier as f64));
    // 长按参数单位为毫秒(与 KBD_SET_HOLD / kbd.hdNN 一致): delay_ms=按住多久才输出,
    // max_hold_ms=输出后最长保持多久自动抬起, 0 均表示禁用。
    item.insert("delay_ms".to_string(), JsonValue::Number(hold.0 as f64));
    item.insert("max_hold_ms".to_string(), JsonValue::Number(hold.1 as f64));
    JsonValue::Object(item)
}

/// zones 组导出 = **两套互不相干的映射并列**:
///  - `bindings`(34 项): Serial 模式的 bind.mapNN, 逻辑分区 → 物理通道;
///  - `hid_points`(36 项): HID 模式的 hid.en/x/yNN, 物理通道 → 屏幕归一坐标。
///
/// ★为什么并入同一组而不新开第七组★ 新增组会改 `GroupSelection` 的字段数, 而导入/导出回调的
/// 签名是 6 个 bool(`settings_export(bool×6)`), Slint 侧勾选框与 main.rs 回调都按这个元数写死;
/// 加一个字段就要同步改 Slint 回调签名、app.slint 透传、main.rs 两处闭包与弹窗布局 —— 那是纯
/// 结构性改动, 且会让旧设置文件的"未勾选新组"语义变得含糊。
/// 二者同属"触控输入映射"、同在设置页的同一个 Tab 位置(按模式二选一), 归一组语义自洽。
/// ★互不覆盖由键前缀天然保证★: 导入 bindings 只写 bind.map*, 导入 hid_points 只写 hid.*,
/// 两段各自独立缺失容错 ⇒ 同一份文件里两套参数可同时存在, 导入任一模式的配置都不动另一套。
fn _export_zones(ctrl: &AppController) -> JsonValue {
    let mut bindings = Vec::with_capacity(34);
    for zone in 0..34usize {
        let mut item = BTreeMap::new();
        item.insert("zone".to_string(), JsonValue::Number(zone as f64));
        item.insert("key".to_string(), JsonValue::String(zone_key(zone)));
        item.insert(
            "binding".to_string(),
            JsonValue::Number(ctrl.get_binding(zone) as f64),
        );
        bindings.push(JsonValue::Object(item));
    }
    let mut hid_points = Vec::with_capacity(HID_POINT_COUNT);
    for ch in 0..HID_POINT_COUNT {
        let (x, y) = ctrl.hid_point_xy(ch);
        let mut item = BTreeMap::new();
        item.insert("channel".to_string(), JsonValue::Number(ch as f64));
        item.insert(
            "enabled".to_string(),
            JsonValue::Bool(ctrl.hid_point_enabled(ch)),
        );
        item.insert("x".to_string(), JsonValue::Number(x as f64));
        item.insert("y".to_string(), JsonValue::Number(y as f64));
        hid_points.push(JsonValue::Object(item));
    }
    let mut group = BTreeMap::new();
    group.insert("bindings".to_string(), JsonValue::Array(bindings));
    group.insert("hid_points".to_string(), JsonValue::Array(hid_points));
    JsonValue::Object(group)
}

fn _import_config(
    ctrl: &mut AppController,
    group: &JsonValue,
    sum: &mut ImportSummary,
) -> Result<()> {
    let group = _object(group, GROUP_CONFIG)?;
    let entries = _array(_required(group, "entries", GROUP_CONFIG)?, "config.entries")?;
    for item in entries {
        let item = _object(item, "config entry")?;
        let key = _string(_required(item, "key", "config entry")?, "config entry key")?.to_string();
        // 分区绑定与键盘映射由 zones / keyboard 组独占，避免同一项被两组分别写入后彼此不一致。
        if !_cfg_group_owns(&key) {
            continue;
        }
        // 设备真值条目是唯一的类型与 min/max 权威来源；缓存里没有的键说明本设备不认识它。
        let Some(truth) = ctrl.config_truth(&key).cloned() else {
            sum._skip(&key, SkipReason::UnknownKey);
            continue;
        };
        let Some(raw) = item.get("value") else {
            sum.absent_items += 1;
            continue;
        };
        let Ok(kind) = _string(
            _required(item, "type", "config entry")?,
            "config entry type",
        ) else {
            sum._skip(&key, SkipReason::Malformed);
            continue;
        };
        let Ok(parsed) = _cfg_from_json(kind, raw) else {
            sum._skip(&key, SkipReason::Malformed);
            continue;
        };
        // 一律换算到设备声明的类型；不能无损表达就拒掉该项，绝不截断/钳位后偷偷写入。
        let Some(value) = _retype_cfg(&truth.value, &parsed) else {
            sum._skip(&key, SkipReason::TypeMismatch);
            continue;
        };
        if let Some((min, max)) = &truth.range {
            if !_cfg_in_range(&value, min, max) {
                sum._skip(&key, SkipReason::OutOfRange);
                continue;
            }
        }
        match ctrl.set_config(ConfigEntry::new(key.clone(), value)) {
            Ok(()) => sum.applied_items += 1,
            Err(_) => sum._skip(&key, SkipReason::Malformed),
        }
    }
    Ok(())
}

fn _import_channel_params(
    ctrl: &mut AppController,
    group: &JsonValue,
    sum: &mut ImportSummary,
) -> Result<()> {
    let group = _object(group, GROUP_CHANNEL_PARAMS)?;
    let channels = _array(
        _required(group, "channels", GROUP_CHANNEL_PARAMS)?,
        "channel_params.channels",
    )?;
    for channel_item in channels {
        let channel_item = _object(channel_item, "channel params item")?;
        let channel = _u8(
            _required(channel_item, "channel", "channel params item")?,
            "channel",
        )?;
        if channel >= 36 {
            sum._skip(format!("channel {channel}"), SkipReason::OutOfRange);
            continue;
        }
        let params = _array(
            _required(channel_item, "params", "channel params item")?,
            "channel params",
        )?;
        for param_item in params {
            let param_item = _object(param_item, "channel param")?;
            let param_id = _u8(
                _required(param_item, "param_id", "channel param")?,
                "param_id",
            )?;
            let name = format!("ch{channel}.param 0x{param_id:02X}");
            if !KNOWN_PARAM_IDS.contains(&param_id) {
                sum._skip(name, SkipReason::UnknownKey);
                continue;
            }
            // 导出时该通道尚未被回读到的槽位没有 value：保持当前值，绝不用 0 覆盖真实设备参数。
            let Some(raw) = param_item.get("value") else {
                sum.absent_items += 1;
                continue;
            };
            let Ok(value) = _u32(raw, "param value") else {
                sum._skip(name, SkipReason::TypeMismatch);
                continue;
            };
            // 每通道参数的合法域由固件校验(上位机没有权威 min/max)，此处只做结构性校验；
            // 保存时若被固件 NAK 会在日志里如实报出，不在这里替固件猜范围。
            match ctrl.set_param(channel, param_id, value) {
                Ok(()) => sum.applied_items += 1,
                Err(_) => sum._skip(name, SkipReason::OutOfRange),
            }
        }
    }
    Ok(())
}

fn _import_globals(
    ctrl: &mut AppController,
    group: &JsonValue,
    sum: &mut ImportSummary,
) -> Result<()> {
    let group = _object(group, GROUP_GLOBALS)?;
    let values = _array(_required(group, "values", GROUP_GLOBALS)?, "globals.values")?;
    for item in values {
        let item = _object(item, "global item")?;
        let id = _u8(_required(item, "gparam_id", "global item")?, "gparam_id")?;
        let name = format!("gparam {id}");
        if !(1..=8).contains(&id) {
            sum._skip(name, SkipReason::OutOfRange);
            continue;
        }
        let Some(raw) = item.get("value") else {
            sum.absent_items += 1;
            continue;
        };
        let Ok(value) = _u32(raw, "global value") else {
            sum._skip(name, SkipReason::TypeMismatch);
            continue;
        };
        match ctrl.global_set(id, value) {
            Ok(()) => sum.applied_items += 1,
            Err(_) => sum._skip(name, SkipReason::OutOfRange),
        }
    }
    // CSD 处理模式枚举: 0=自动校准 1=半自动手动。只写草稿(MODE_SET 由保存流下发)。
    match group.get("csd_mode") {
        None => sum.absent_items += 1,
        Some(raw) => match _u8(raw, "csd_mode") {
            Ok(mode) if mode <= 1 => match ctrl.set_mode(mode) {
                Ok(()) => sum.applied_items += 1,
                Err(_) => sum._skip("csd_mode", SkipReason::OutOfRange),
            },
            Ok(_) => sum._skip("csd_mode", SkipReason::OutOfRange),
            Err(_) => sum._skip("csd_mode", SkipReason::TypeMismatch),
        },
    }
    Ok(())
}

fn _import_algo(
    ctrl: &mut AppController,
    group: &JsonValue,
    sum: &mut ImportSummary,
) -> Result<()> {
    let group = _object(group, GROUP_ALGO)?;
    // ★算法 C 源与机器码不进草稿★: 它们的唯一下发通道是 ALGO_SET_SRC / ALGO_UPLOAD 即时命令,
    // 没有草稿层也不参与 save_config。导入必须零设备命令, 故这里只统计并告知, 由用户在算法页手动上传。
    for field in ["device_source", "device_code_hex"] {
        let non_empty = group
            .get(field)
            .and_then(|v| match v {
                JsonValue::String(s) => Some(!s.trim().is_empty()),
                _ => None,
            })
            .unwrap_or(false);
        if non_empty {
            sum._skip(format!("algo.{field}"), SkipReason::NotDraftable);
        }
    }
    // schema_source/metadata 是分享时携带的只读描述；元数据真相仍来自算法源码，导入旧文件时可缺省。
    if let Some(metadata) = group.get("metadata") {
        if !matches!(metadata, JsonValue::Array(_)) {
            sum._skip("algo.metadata", SkipReason::Malformed);
        }
    }
    let cfg = _array(_required(group, "cfg", GROUP_ALGO)?, "algo.cfg")?;
    for item in cfg {
        let item = _object(item, "algo cfg item")?;
        let index = _u8(_required(item, "index", "algo cfg item")?, "algo cfg index")?;
        let name = format!("algo.cfg[{index}]");
        if index >= 8 {
            sum._skip(name, SkipReason::OutOfRange);
            continue;
        }
        let Some(raw) = item.get("value") else {
            sum.absent_items += 1;
            continue;
        };
        let Ok(value) = _u8(raw, "algo cfg value") else {
            sum._skip(name, SkipReason::TypeMismatch);
            continue;
        };
        match ctrl.set_algo_cfg(index, value) {
            Ok(()) => sum.applied_items += 1,
            Err(_) => sum._skip(name, SkipReason::OutOfRange),
        }
    }
    Ok(())
}

fn _import_keyboard(
    ctrl: &mut AppController,
    group: &JsonValue,
    sum: &mut ImportSummary,
) -> Result<()> {
    let group = _object(group, GROUP_KEYBOARD)?;
    let physical = _array(
        _required(group, "physical", GROUP_KEYBOARD)?,
        "keyboard.physical",
    )?;
    for item in physical {
        _import_key_entry(ctrl, item, KBD_HOLD_KIND_PHYS, 12, sum)?;
    }
    let touch = _array(_required(group, "touch", GROUP_KEYBOARD)?, "keyboard.touch")?;
    for item in touch {
        _import_key_entry(ctrl, item, KBD_HOLD_KIND_ZONE, 34, sum)?;
    }
    // ★combo 缺失时不报错★: 旧版本导出的文件没有这一段, 直接失败会让用户连键盘组都导不进来。
    // 缺失即"不动组合映射", 与"导入未勾选该组"同语义。
    if let Some(raw) = group.get("combo") {
        let items = _array(raw, "keyboard.combo")?;
        let mut table: Vec<crate::proto::KbdComboItem> = Vec::new();
        for item in items {
            let obj = _object(item, "keyboard.combo")?;
            let zone_mask =
                _number(_required(obj, "zone_mask", "keyboard.combo")?, "zone_mask")? as u64;
            let modifiers =
                _number(_required(obj, "modifiers", "keyboard.combo")?, "modifiers")? as u8;
            let delay_ms =
                _number(_required(obj, "delay_ms", "keyboard.combo")?, "delay_ms")? as u16;
            let max_hold_ms = _number(
                _required(obj, "max_hold_ms", "keyboard.combo")?,
                "max_hold_ms",
            )? as u16;
            let codes = _array(_required(obj, "keycodes", "keyboard.combo")?, "keycodes")?;
            let mut keycodes = [0u8; crate::proto::KBD_COMBO_KEY_COUNT];
            for (slot, raw_code) in keycodes.iter_mut().zip(codes.iter()) {
                *slot = _number(raw_code, "keycodes")? as u8;
            }
            // 空条目(无分区或无按键)一律丢弃: 下发出去只会被固件丢, 留在表里反而占用上限名额。
            if zone_mask == 0 || (keycodes.iter().all(|k| *k == 0) && modifiers == 0) {
                sum._skip("keyboard.combo(空条目)", SkipReason::OutOfRange);
                continue;
            }
            if table.len() >= crate::proto::KBD_COMBO_COUNT {
                sum._skip("keyboard.combo(超上限)", SkipReason::OutOfRange);
                continue;
            }
            table.push(crate::proto::KbdComboItem {
                zone_mask,
                keycodes,
                modifiers,
                delay_ms,
                max_hold_ms,
            });
        }
        let applied = table.len();
        ctrl.kbd_combo_replace_table(table);
        sum.applied_items += applied;
    }
    Ok(())
}

/// 物理键与触控分区的按键项结构完全一致(索引/键码/修饰位/长按), 抽一处处理避免两份重复逻辑。
fn _import_key_entry(
    ctrl: &mut AppController,
    item: &JsonValue,
    kind: u8,
    limit: u8,
    sum: &mut ImportSummary,
) -> Result<()> {
    let zone_kind = kind == KBD_HOLD_KIND_ZONE;
    let item = _object(
        item,
        if zone_kind {
            "touch key"
        } else {
            "physical key"
        },
    )?;
    let index = _u8(_required(item, "index", "key entry")?, "key entry index")?;
    let name = if zone_kind {
        format!("kbd.zone{index:02}")
    } else {
        format!("kbd.key{index:02}")
    };
    if index >= limit {
        sum._skip(name, SkipReason::OutOfRange);
        return Ok(());
    }
    // 键码与修饰位必须同时给出才能构成一次映射写入; 缺任一项按"文件缺键"保持现值。
    match (item.get("keycode"), item.get("modifier")) {
        (Some(code), Some(modifier)) => match (_u8(code, "keycode"), _u8(modifier, "modifier")) {
            (Ok(code), Ok(modifier)) => {
                let staged = if zone_kind {
                    ctrl.kbd_set_touchmap(index, code, modifier)
                } else {
                    ctrl.kbd_set_map(index, code, modifier)
                };
                match staged {
                    Ok(()) => sum.applied_items += 1,
                    Err(_) => sum._skip(&name, SkipReason::OutOfRange),
                }
            }
            _ => sum._skip(&name, SkipReason::TypeMismatch),
        },
        _ => sum.absent_items += 1,
    }
    // 长按参数(毫秒)是同一按键项的独立字段; 旧版导出文件没有这两项, 按缺键保持现值。
    match (item.get("delay_ms"), item.get("max_hold_ms")) {
        (Some(delay), Some(max_hold)) => {
            match (_u16(delay, "delay_ms"), _u16(max_hold, "max_hold_ms")) {
                (Ok(delay), Ok(max_hold)) => {
                    match ctrl.stage_kbd_hold(kind, index, delay, max_hold) {
                        Ok(()) => sum.applied_items += 1,
                        Err(_) => sum._skip(format!("{name}.hold"), SkipReason::OutOfRange),
                    }
                }
                _ => sum._skip(format!("{name}.hold"), SkipReason::TypeMismatch),
            }
        }
        _ => sum.absent_items += 1,
    }
    Ok(())
}

fn _import_zones(
    ctrl: &mut AppController,
    group: &JsonValue,
    sum: &mut ImportSummary,
) -> Result<()> {
    let group = _object(group, GROUP_ZONES)?;
    let bindings = _array(_required(group, "bindings", GROUP_ZONES)?, "zones.bindings")?;
    for item in bindings {
        let item = _object(item, "zone binding")?;
        let zone = _u32(_required(item, "zone", "zone binding")?, "zone")? as usize;
        let name = format!("bind.map{zone:02}");
        if zone >= 34 {
            sum._skip(name, SkipReason::OutOfRange);
            continue;
        }
        let Some(raw) = item.get("binding") else {
            sum.absent_items += 1;
            continue;
        };
        let Ok(value) = _u32(raw, "binding") else {
            sum._skip(name, SkipReason::TypeMismatch);
            continue;
        };
        // 语义: 0..35 = 物理通道索引, 0xFFFFFFFF = 未映射。其余值一律拒收, 不悄悄当成未映射。
        if value > 35 && value != 0xFFFF_FFFF {
            sum._skip(name, SkipReason::OutOfRange);
            continue;
        }
        match ctrl.set_binding(zone, value) {
            Ok(()) => sum.applied_items += 1,
            Err(_) => sum._skip(name, SkipReason::OutOfRange),
        }
    }
    // ★hid_points 缺失时不报错★: 旧版本导出的文件没有这一段(与 keyboard.combo 同口径),
    // 直接失败会让用户连分区绑定都导不进来。缺失即"不动 HID 点位", 与"未勾选"同语义。
    // 本段只写 hid.*, 上面那段只写 bind.map* ⇒ 导入其一绝不影响另一套。
    if let Some(raw) = group.get("hid_points") {
        let points = _array(raw, "zones.hid_points")?;
        for item in points {
            let item = _object(item, "hid point")?;
            let ch = _u32(_required(item, "channel", "hid point")?, "channel")? as usize;
            let name = format!("hid.{ch:02}");
            if ch >= HID_POINT_COUNT {
                sum._skip(name, SkipReason::OutOfRange);
                continue;
            }
            // 坐标与启用位是一个点位的两个侧面, 但允许文件只给其中之一(缺的按"保持现值")。
            match (item.get("x"), item.get("y")) {
                (Some(rx), Some(ry)) => match (_u32(rx, "hid x"), _u32(ry, "hid y")) {
                    (Ok(x), Ok(y)) => {
                        // 越界不静默钳位: 这两个域由固件描述符决定(0..32767), 超了必须让用户知道。
                        if x > HID_COORD_MAX as u32 || y > HID_COORD_MAX as u32 {
                            sum._skip(&name, SkipReason::OutOfRange);
                        } else {
                            match ctrl.set_hid_point_xy(ch, x as u16, y as u16) {
                                Ok(()) => sum.applied_items += 1,
                                Err(_) => sum._skip(&name, SkipReason::OutOfRange),
                            }
                        }
                    }
                    _ => sum._skip(&name, SkipReason::TypeMismatch),
                },
                _ => sum.absent_items += 1,
            }
            match item.get("enabled") {
                Some(JsonValue::Bool(on)) => match ctrl.set_hid_point_enabled(ch, *on) {
                    Ok(()) => sum.applied_items += 1,
                    Err(_) => sum._skip(format!("{name}.enabled"), SkipReason::OutOfRange),
                },
                Some(_) => sum._skip(format!("{name}.enabled"), SkipReason::TypeMismatch),
                None => sum.absent_items += 1,
            }
        }
    }
    Ok(())
}

fn _group<'a>(groups: &'a BTreeMap<String, JsonValue>, name: &str) -> Result<&'a JsonValue> {
    groups
        .get(name)
        .ok_or_else(|| anyhow!("JSON 不包含 {name:?} 组；请取消该选择或使用完整导出文件"))
}

/// `<prefix><两位十进制>` 形式的键，索引须小于 `limit`。
/// 必须严格限定两位数字：否则 `kbd.zm` 会连 `kbd.zmh00` 一起吃掉(修饰位与长按参数混为一谈)。
fn _indexed_key(key: &str, prefix: &str, limit: usize) -> Option<usize> {
    key.strip_prefix(prefix)
        .filter(|rest| rest.len() == 2 && rest.bytes().all(|b| b.is_ascii_digit()))
        .and_then(|rest| rest.parse::<usize>().ok())
        .filter(|index| *index < limit)
}

/// 分区绑定与 HID 触控点位键：由 zones 组独占导入/导出。
/// ★hid.* 必须收在这里★ 否则它会作为普通 KV 落进 config 组, 于是同一项有两个出口:
/// config 组按裸 KV 写一遍、zones 组按结构化 hid_points 再写一遍, 两者顺序一变结果就不同
/// (`kbd.cb*` 正是踩过这个坑才被收进 keyboard 组的)。
fn _is_zone_key(key: &str) -> bool {
    _indexed_key(key, "bind.map", 34).is_some()
        || _indexed_key(key, "hid.en", HID_POINT_COUNT).is_some()
        || _indexed_key(key, "hid.x", HID_POINT_COUNT).is_some()
        || _indexed_key(key, "hid.y", HID_POINT_COUNT).is_some()
}

/// 键盘映射与长按参数键：由 keyboard 组独占导入/导出。
/// ★`kbd.cb*`(组合映射打包 KV)也归本组★: 它原先谁都不认, 于是被当成普通 KV 落进 config 组 ——
/// 那是打包过的裸字节, 导入回去 UI 的组合表缓存不会更新, 表现就是"导出再导入组合映射没了"。
/// 现在组合映射由 keyboard 组以结构化的 `combo` 数组独占进出, 裸 KV 不再是第二个出口。
fn _is_keyboard_key(key: &str) -> bool {
    if key.starts_with("kbd.cb") {
        return true;
    }
    const PHYS: [&str; 4] = ["kbd.key", "kbd.km", "kbd.hd", "kbd.mh"];
    const ZONE: [&str; 4] = ["kbd.zone", "kbd.zm", "kbd.zhd", "kbd.zmh"];
    PHYS.iter().any(|p| _indexed_key(key, p, 12).is_some())
        || ZONE.iter().any(|p| _indexed_key(key, p, 34).is_some())
}

/// config 组是否是该键的唯一导入/导出出口。
fn _cfg_group_owns(key: &str) -> bool {
    !_is_zone_key(key) && !_is_keyboard_key(key)
}

/// 把文件里的值换算成设备声明的类型。无法无损表达则返回 `None`(调用方拒收该项)。
/// ★不做钳位★: 越界必须让用户知道，而不是写进一个他没选的值。
fn _retype_cfg(truth: &CfgValue, value: &CfgValue) -> Option<CfgValue> {
    match truth {
        CfgValue::Str(_) => {
            return match value {
                CfgValue::Str(v) => Some(CfgValue::Str(v.clone())),
                _ => None,
            };
        }
        CfgValue::Bool(_) => {
            return match value {
                CfgValue::Bool(v) => Some(CfgValue::Bool(*v)),
                _ => match _cfg_as_i64(value)? {
                    0 => Some(CfgValue::Bool(false)),
                    1 => Some(CfgValue::Bool(true)),
                    _ => None,
                },
            };
        }
        CfgValue::F32(_) => {
            return match value {
                CfgValue::F32(v) if v.is_finite() => Some(CfgValue::F32(*v)),
                CfgValue::F32(_) | CfgValue::Str(_) => None,
                _ => Some(CfgValue::F32(_cfg_as_i64(value)? as f32)),
            };
        }
        _ => {}
    }
    if matches!(value, CfgValue::Str(_)) {
        return None;
    }
    let n = _cfg_as_i64(value)?;
    Some(match truth {
        CfgValue::I8(_) => CfgValue::I8(i8::try_from(n).ok()?),
        CfgValue::U8(_) => CfgValue::U8(u8::try_from(n).ok()?),
        CfgValue::U16(_) => CfgValue::U16(u16::try_from(n).ok()?),
        CfgValue::U32(_) => CfgValue::U32(u32::try_from(n).ok()?),
        _ => return None,
    })
}

/// 整数化的配置值；小数部分非零的浮点视为不可无损转换。
fn _cfg_as_i64(value: &CfgValue) -> Option<i64> {
    Some(match value {
        CfgValue::Bool(v) => i64::from(*v),
        CfgValue::I8(v) => i64::from(*v),
        CfgValue::U8(v) => i64::from(*v),
        CfgValue::U16(v) => i64::from(*v),
        CfgValue::U32(v) => i64::from(*v),
        CfgValue::F32(v) if v.is_finite() && v.fract() == 0.0 => *v as i64,
        _ => return None,
    })
}

fn _cfg_as_f64(value: &CfgValue) -> Option<f64> {
    match value {
        CfgValue::F32(v) if v.is_finite() => Some(*v as f64),
        _ => _cfg_as_i64(value).map(|n| n as f64),
    }
}

/// 是否落在设备声明的 [min, max] 内。min/max 无法数值化(字符串项)时视为无范围约束。
fn _cfg_in_range(value: &CfgValue, min: &CfgValue, max: &CfgValue) -> bool {
    let (Some(v), Some(lo), Some(hi)) = (_cfg_as_f64(value), _cfg_as_f64(min), _cfg_as_f64(max))
    else {
        return true;
    };
    v >= lo && v <= hi
}

fn _cfg_type_name(value: &CfgValue) -> &'static str {
    match value {
        CfgValue::Bool(_) => "bool",
        CfgValue::I8(_) => "i8",
        CfgValue::U8(_) => "u8",
        CfgValue::U16(_) => "u16",
        CfgValue::U32(_) => "u32",
        CfgValue::F32(_) => "f32",
        CfgValue::Str(_) => "string",
    }
}

fn _cfg_to_json(value: &CfgValue) -> Result<JsonValue> {
    Ok(match value {
        CfgValue::Bool(v) => JsonValue::Bool(*v),
        CfgValue::I8(v) => JsonValue::Number(*v as f64),
        CfgValue::U8(v) => JsonValue::Number(*v as f64),
        CfgValue::U16(v) => JsonValue::Number(*v as f64),
        CfgValue::U32(v) => JsonValue::Number(*v as f64),
        CfgValue::F32(v) if v.is_finite() => JsonValue::Number(*v as f64),
        CfgValue::F32(_) => bail!("配置含非有限 f32，无法编码为 JSON"),
        CfgValue::Str(v) => JsonValue::String(v.clone()),
    })
}

fn _cfg_from_json(kind: &str, value: &JsonValue) -> Result<CfgValue> {
    match kind {
        "bool" => match value {
            JsonValue::Bool(v) => Ok(CfgValue::Bool(*v)),
            _ => bail!("bool 配置值必须是 JSON bool"),
        },
        "i8" => Ok(CfgValue::I8(_i8(value, "i8 config value")?)),
        "u8" => Ok(CfgValue::U8(_u8(value, "u8 config value")?)),
        "u16" => Ok(CfgValue::U16(_u16(value, "u16 config value")?)),
        "u32" => Ok(CfgValue::U32(_u32(value, "u32 config value")?)),
        "f32" => {
            let n = _number(value, "f32 config value")?;
            if !n.is_finite() || n < f32::MIN as f64 || n > f32::MAX as f64 {
                bail!("f32 配置值超出有效范围");
            }
            Ok(CfgValue::F32(n as f32))
        }
        "string" => Ok(CfgValue::Str(
            _string(value, "string config value")?.to_string(),
        )),
        _ => bail!("未知配置类型 {kind:?}"),
    }
}

fn _required<'a>(
    object: &'a BTreeMap<String, JsonValue>,
    key: &str,
    context: &str,
) -> Result<&'a JsonValue> {
    object
        .get(key)
        .ok_or_else(|| anyhow!("{context} 缺少必填字段 {key:?}"))
}

fn _object<'a>(value: &'a JsonValue, context: &str) -> Result<&'a BTreeMap<String, JsonValue>> {
    match value {
        JsonValue::Object(v) => Ok(v),
        _ => bail!("{context} 必须是 JSON object"),
    }
}
fn _array<'a>(value: &'a JsonValue, context: &str) -> Result<&'a [JsonValue]> {
    match value {
        JsonValue::Array(v) => Ok(v),
        _ => bail!("{context} 必须是 JSON array"),
    }
}
fn _string<'a>(value: &'a JsonValue, context: &str) -> Result<&'a str> {
    match value {
        JsonValue::String(v) => Ok(v),
        _ => bail!("{context} 必须是 JSON string"),
    }
}
fn _number(value: &JsonValue, context: &str) -> Result<f64> {
    match value {
        JsonValue::Number(v) if v.is_finite() => Ok(*v),
        _ => bail!("{context} 必须是有限 JSON number"),
    }
}
fn _u32(value: &JsonValue, context: &str) -> Result<u32> {
    let n = _number(value, context)?;
    if n.fract() != 0.0 || !(0.0..=u32::MAX as f64).contains(&n) {
        bail!("{context} 必须是 u32 整数")
    }
    Ok(n as u32)
}
fn _u16(value: &JsonValue, context: &str) -> Result<u16> {
    let n = _u32(value, context)?;
    u16::try_from(n).map_err(|_| anyhow!("{context} 超出 u16 范围"))
}
fn _u8(value: &JsonValue, context: &str) -> Result<u8> {
    let n = _u32(value, context)?;
    u8::try_from(n).map_err(|_| anyhow!("{context} 超出 u8 范围"))
}
fn _i8(value: &JsonValue, context: &str) -> Result<i8> {
    let n = _number(value, context)?;
    if n.fract() != 0.0 || n < i8::MIN as f64 || n > i8::MAX as f64 {
        bail!("{context} 必须是 i8 整数")
    }
    Ok(n as i8)
}

#[derive(Clone, Debug)]
enum JsonValue {
    Object(BTreeMap<String, JsonValue>),
    Array(Vec<JsonValue>),
    String(String),
    Number(f64),
    Bool(bool),
    Null,
}

fn _write_json(value: &JsonValue, out: &mut String, indent: usize) -> Result<()> {
    match value {
        JsonValue::Object(map) => {
            out.push('{');
            if !map.is_empty() {
                for (index, (key, child)) in map.iter().enumerate() {
                    out.push('\n');
                    _indent(out, indent + 1);
                    _write_string(key, out);
                    out.push_str(": ");
                    _write_json(child, out, indent + 1)?;
                    if index + 1 < map.len() {
                        out.push(',');
                    }
                }
                out.push('\n');
                _indent(out, indent);
            }
            out.push('}');
        }
        JsonValue::Array(values) => {
            out.push('[');
            if !values.is_empty() {
                for (index, child) in values.iter().enumerate() {
                    out.push('\n');
                    _indent(out, indent + 1);
                    _write_json(child, out, indent + 1)?;
                    if index + 1 < values.len() {
                        out.push(',');
                    }
                }
                out.push('\n');
                _indent(out, indent);
            }
            out.push(']');
        }
        JsonValue::String(value) => _write_string(value, out),
        JsonValue::Number(value) if value.is_finite() => out.push_str(&value.to_string()),
        JsonValue::Number(_) => bail!("JSON 不支持非有限数字"),
        JsonValue::Bool(value) => out.push_str(if *value { "true" } else { "false" }),
        JsonValue::Null => out.push_str("null"),
    }
    Ok(())
}

fn _indent(out: &mut String, level: usize) {
    for _ in 0..level {
        out.push_str("  ");
    }
}

/// 只转义 JSON 必需字符及控制字符，UTF-8 可直接保留以提高文件可读性。
fn _write_string(value: &str, out: &mut String) {
    out.push('"');
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            ch if ch <= '\u{1F}' => out.push_str(&format!("\\u{:04X}", ch as u32)),
            ch => out.push(ch),
        }
    }
    out.push('"');
}

struct JsonParser<'a> {
    text: &'a str,
    pos: usize,
}

impl<'a> JsonParser<'a> {
    fn new(text: &'a str) -> Self {
        Self { text, pos: 0 }
    }

    fn parse(mut self) -> Result<JsonValue> {
        self._whitespace();
        let value = self._value()?;
        self._whitespace();
        if self.pos != self.text.len() {
            return self._error("根值后有额外内容");
        }
        Ok(value)
    }

    fn _value(&mut self) -> Result<JsonValue> {
        self._whitespace();
        match self._peek() {
            Some(b'{') => self._object_value(),
            Some(b'[') => self._array_value(),
            Some(b'"') => Ok(JsonValue::String(self._string_value()?)),
            Some(b't') => {
                self._literal("true")?;
                Ok(JsonValue::Bool(true))
            }
            Some(b'f') => {
                self._literal("false")?;
                Ok(JsonValue::Bool(false))
            }
            Some(b'n') => {
                self._literal("null")?;
                Ok(JsonValue::Null)
            }
            Some(b'-' | b'0'..=b'9') => Ok(JsonValue::Number(self._number_value()?)),
            Some(_) => self._error("此处期待 object、array、string、number、bool 或 null"),
            None => self._error("意外到达文件末尾"),
        }
    }

    fn _object_value(&mut self) -> Result<JsonValue> {
        self._take(b'{')?;
        self._whitespace();
        let mut map = BTreeMap::new();
        if self._consume(b'}') {
            return Ok(JsonValue::Object(map));
        }
        loop {
            self._whitespace();
            if self._peek() != Some(b'"') {
                return self._error("object key 必须是 string");
            }
            let key = self._string_value()?;
            self._whitespace();
            self._take(b':')?;
            let value = self._value()?;
            if map.insert(key.clone(), value).is_some() {
                return self._error(&format!("object key {key:?} 重复"));
            }
            self._whitespace();
            if self._consume(b'}') {
                break;
            }
            self._take(b',')?;
        }
        Ok(JsonValue::Object(map))
    }

    fn _array_value(&mut self) -> Result<JsonValue> {
        self._take(b'[')?;
        self._whitespace();
        let mut values = Vec::new();
        if self._consume(b']') {
            return Ok(JsonValue::Array(values));
        }
        loop {
            values.push(self._value()?);
            self._whitespace();
            if self._consume(b']') {
                break;
            }
            self._take(b',')?;
        }
        Ok(JsonValue::Array(values))
    }

    fn _string_value(&mut self) -> Result<String> {
        self._take(b'"')?;
        let mut out = String::new();
        loop {
            let Some(byte) = self._peek() else {
                return self._error("string 未闭合");
            };
            if byte == b'"' {
                self.pos += 1;
                return Ok(out);
            }
            if byte < 0x20 {
                return self._error("string 含未转义控制字符");
            }
            if byte != b'\\' {
                let Some(ch) = self.text[self.pos..].chars().next() else {
                    return self._error("无效 UTF-8 string");
                };
                out.push(ch);
                self.pos += ch.len_utf8();
                continue;
            }
            self.pos += 1;
            let escape_pos = self.pos;
            let Some(escape) = self._peek() else {
                return self._error("string 转义未完成");
            };
            self.pos += 1;
            match escape {
                b'"' => out.push('"'),
                b'\\' => out.push('\\'),
                b'/' => out.push('/'),
                b'b' => out.push('\u{0008}'),
                b'f' => out.push('\u{000C}'),
                b'n' => out.push('\n'),
                b'r' => out.push('\r'),
                b't' => out.push('\t'),
                b'u' => {
                    let code = self._unicode_escape()?;
                    let Some(ch) = char::from_u32(code) else {
                        self.pos = escape_pos;
                        return self._error("不支持未配对的 Unicode 代理项");
                    };
                    out.push(ch);
                }
                _ => {
                    self.pos = escape_pos;
                    return self._error("未知 string 转义");
                }
            }
        }
    }

    fn _unicode_escape(&mut self) -> Result<u32> {
        if self.pos + 4 > self.text.len() {
            return self._error("Unicode 转义不足四个十六进制字符");
        }
        let mut value = 0u32;
        for _ in 0..4 {
            let byte = self.text.as_bytes()[self.pos];
            let digit = match byte {
                b'0'..=b'9' => (byte - b'0') as u32,
                b'a'..=b'f' => (byte - b'a' + 10) as u32,
                b'A'..=b'F' => (byte - b'A' + 10) as u32,
                _ => return self._error("Unicode 转义含非十六进制字符"),
            };
            value = value * 16 + digit;
            self.pos += 1;
        }
        Ok(value)
    }

    fn _number_value(&mut self) -> Result<f64> {
        let start = self.pos;
        self._consume(b'-');
        match self._peek() {
            Some(b'0') => self.pos += 1,
            Some(b'1'..=b'9') => {
                self.pos += 1;
                while matches!(self._peek(), Some(b'0'..=b'9')) {
                    self.pos += 1;
                }
            }
            _ => return self._error("number 整数部分非法"),
        }
        if self._consume(b'.') {
            let decimal_start = self.pos;
            while matches!(self._peek(), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
            if self.pos == decimal_start {
                return self._error("number 小数部分缺少数字");
            }
        }
        if matches!(self._peek(), Some(b'e' | b'E')) {
            self.pos += 1;
            if matches!(self._peek(), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            let exponent_start = self.pos;
            while matches!(self._peek(), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
            if self.pos == exponent_start {
                return self._error("number 指数部分缺少数字");
            }
        }
        let raw = &self.text[start..self.pos];
        let value = raw
            .parse::<f64>()
            .map_err(|_| anyhow!("JSON 第 {} 字节: 无法解析 number", start))?;
        if !value.is_finite() {
            return self._error("number 超出有限范围");
        }
        Ok(value)
    }

    fn _literal(&mut self, literal: &str) -> Result<()> {
        if self.text[self.pos..].starts_with(literal) {
            self.pos += literal.len();
            Ok(())
        } else {
            self._error(&format!("期待 {literal}"))
        }
    }
    fn _whitespace(&mut self) {
        while matches!(self._peek(), Some(b' ' | b'\n' | b'\r' | b'\t')) {
            self.pos += 1;
        }
    }
    fn _peek(&self) -> Option<u8> {
        self.text.as_bytes().get(self.pos).copied()
    }
    fn _consume(&mut self, expected: u8) -> bool {
        if self._peek() == Some(expected) {
            self.pos += 1;
            true
        } else {
            false
        }
    }
    fn _take(&mut self, expected: u8) -> Result<()> {
        if self._consume(expected) {
            Ok(())
        } else {
            self._error(&format!("期待字符 {:?}", expected as char))
        }
    }
    fn _error<T>(&self, message: &str) -> Result<T> {
        Err(anyhow!("JSON 第 {} 字节: {message}", self.pos))
    }
}

/// 默认导出名使用已有日志模块相同的 Win32 本地时间来源，不引入 chrono/time。
pub fn default_export_filename() -> String {
    #[cfg(windows)]
    {
        use windows::Win32::System::SystemInformation::GetLocalTime;
        // SAFETY: GetLocalTime 仅写入返回的 SYSTEMTIME 值。
        let now = unsafe { GetLocalTime() };
        return format!(
            "mai2control-settings-{:04}{:02}{:02}-{:02}{:02}{:02}.json",
            now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond
        );
    }
    #[cfg(not(windows))]
    {
        "mai2control-settings.json".to_string()
    }
}

fn local_timestamp() -> String {
    #[cfg(windows)]
    {
        use windows::Win32::System::SystemInformation::GetLocalTime;
        // SAFETY: GetLocalTime 仅写入返回的 SYSTEMTIME 值。
        let now = unsafe { GetLocalTime() };
        return format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}",
            now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond, now.wMilliseconds
        );
    }
    #[cfg(not(windows))]
    {
        "0000-00-00 00:00:00.000".to_string()
    }
}

/// 使用系统原生对话框选择导入/导出路径；取消选择返回 Ok(None)，其他对话框错误带错误码。
#[cfg(windows)]
pub fn choose_settings_path(save: bool) -> Result<Option<PathBuf>> {
    use windows::Win32::UI::Controls::Dialogs::{
        CommDlgExtendedError, GetOpenFileNameW, GetSaveFileNameW, OPENFILENAMEW,
    };
    use windows::core::{PCWSTR, PWSTR};

    let initial_dir = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|p| p.to_path_buf()))
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_else(|| PathBuf::from("."));
    let filter = _wide("JSON 设置文件 (*.json)\0*.json\0\0");
    let initial_dir = _wide(&initial_dir.to_string_lossy());
    let default_ext = _wide("json");
    let default_name = if save {
        default_export_filename()
    } else {
        String::new()
    };
    let mut file = _wide(&default_name);
    file.resize(32_768, 0);
    let mut dialog = OPENFILENAMEW {
        lStructSize: std::mem::size_of::<OPENFILENAMEW>() as u32,
        lpstrFilter: PCWSTR(filter.as_ptr()),
        lpstrFile: PWSTR(file.as_mut_ptr()),
        nMaxFile: file.len() as u32,
        lpstrInitialDir: PCWSTR(initial_dir.as_ptr()),
        lpstrDefExt: PCWSTR(default_ext.as_ptr()),
        ..Default::default()
    };
    // SAFETY: OPENFILENAMEW 和 UTF-16 缓冲在同步系统调用返回前始终有效。
    let selected = unsafe {
        if save {
            GetSaveFileNameW(&mut dialog).as_bool()
        } else {
            GetOpenFileNameW(&mut dialog).as_bool()
        }
    };
    if !selected {
        // 0 表示用户取消；非零才是需要展示的系统对话框错误。
        let error = unsafe { CommDlgExtendedError().0 };
        if error == 0 {
            return Ok(None);
        }
        bail!("系统文件对话框失败，错误码 0x{error:08X}");
    }
    let end = file.iter().position(|&ch| ch == 0).unwrap_or(file.len());
    Ok(Some(PathBuf::from(String::from_utf16_lossy(&file[..end]))))
}

#[cfg(not(windows))]
pub fn choose_settings_path(_save: bool) -> Result<Option<PathBuf>> {
    bail!("设置导入/导出文件对话框仅支持 Windows")
}

#[cfg(windows)]
fn _wide(value: &str) -> Vec<u16> {
    value.encode_utf16().chain(std::iter::once(0)).collect()
}
