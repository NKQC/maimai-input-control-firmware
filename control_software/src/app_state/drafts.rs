//! 配置草稿层: 12 份编辑草稿 + 未保存脏键集合 + 版本号的**唯一**持有者。
//!
//! ★结构性不变量★
//! 全部草稿字段一律私有(靠 Rust 模块可见性强制), 外部拿不到草稿本体的 `&mut`,
//! 因此**任何修改只能经本结构的方法**; 而每个修改方法内部必定一并完成
//! 「改草稿 + 登记/撤销脏键 + bump version」三件事 —— 调用方无从遗漏。
//!
//! 这是**结构保证, 不依赖调用方自律**。旧实现把"标记未保存"拆成调用方手工调用
//! `mark_config_dirty()`, 已因漏调用产生过真实 bug: `_stage_combo` 插了脏键却没 mark,
//! 于是"添加组合映射"后保存按钮不亮、显示 0 项未保存。旧实现另有一个可与脏键集合互相
//! 漂移的 `config_dirty: bool`(`is_config_dirty()` 读 bool 而 `config_dirty_count()` 读集合
//! 长度, 两者可矛盾), 现已删除: **脏键集合就是"未保存项的引用计数", 是唯一真相**,
//! `is_dirty()` 与 `dirty_count()` 在数学上不可能再不一致。
//!
//! 例外(显式设计, 不是遗漏): `led_region` 草稿不登记脏键 —— 灯效单元映射靠
//! `LED_SET_REGION` 即时整批下发, 不参与 SAVE_CONFIG 流程; 若登记脏键, "保存到设备"
//! 会把它算作一项未保存改动然后原地丢弃(既没下发也不再显示)。它只随 `clear_all()` /
//! `drop_led_region()` 回到设备真值, 与旧行为一致。
//!
//! "值改回设备真值即撤稿"的判定留在调用方(设备真值缓存在 `AppController` 侧):
//! 各 `set_*` 统一收 `same_as_device: bool`, 为真即移除草稿项并撤销脏键。

use std::collections::{BTreeMap, BTreeSet};

use crate::proto::{
    CfgValue, HoldParam, KBD_HOLD_KIND_PHYS, KBD_HOLD_KIND_ZONE, KbdComboItem, KbdHoldItem,
    KbdKeyCfg, LED_UNIT_COUNT, LedRegion,
};

/// 物理通道数(分辨率等"整组"参数按此数量同写)。
const PARAM_CH_COUNT: u8 = 36;
/// 整表类草稿的固定脏键(整表替换语义, 无逐条 key)。
const KEY_KBD_COMBO: &str = "kbd:combo";
const KEY_MODE: &str = "mode";

/// 配置草稿覆盖层。dirty key 命名沿用既有稳定口径:
/// `cfg:<key>` / `param:<ch>:<id>` / `param:all:<id>` / `global:<id>` / `algo:cfg:<idx>` /
/// `mode` / `kbd:phys:<idx>` / `kbd:zone:<zone>` / `kbd:hold:phys:<idx>` /
/// `kbd:hold:zone:<zone>` / `kbd:keycfg:<idx>` / `kbd:combo`。
/// key 字符串一律在本结构内部拼装, 杜绝调用方两处拼法漂移。
pub struct ConfigDrafts {
    cfg: BTreeMap<String, CfgValue>,
    param: BTreeMap<(u8, u8), u32>,
    global: BTreeMap<u8, u32>,
    algo_cfg: BTreeMap<u8, u8>,
    mode: Option<u8>,
    kbd_map: BTreeMap<u8, (u8, u8)>,
    kbd_touch: BTreeMap<u8, (u8, u8)>,
    kbd_hold_phys: BTreeMap<u8, HoldParam>,
    kbd_hold_zone: BTreeMap<u8, HoldParam>,
    kbd_keycfg: BTreeMap<u8, KbdKeyCfg>,
    kbd_combo: Option<Vec<KbdComboItem>>,
    led_region: Option<[LedRegion; LED_UNIT_COUNT]>,
    /// 未保存项集合 = 脏状态的唯一真相(空 ⇔ 不脏)。
    dirty_keys: BTreeSet<String>,
    /// 脏键集合发生任何增删即自增, 供 UI 做变更检测。
    version: u64,
}

impl Default for ConfigDrafts {
    fn default() -> Self {
        Self::new()
    }
}

impl ConfigDrafts {
    pub fn new() -> Self {
        ConfigDrafts {
            cfg: BTreeMap::new(),
            param: BTreeMap::new(),
            global: BTreeMap::new(),
            algo_cfg: BTreeMap::new(),
            mode: None,
            kbd_map: BTreeMap::new(),
            kbd_touch: BTreeMap::new(),
            kbd_hold_phys: BTreeMap::new(),
            kbd_hold_zone: BTreeMap::new(),
            kbd_keycfg: BTreeMap::new(),
            kbd_combo: None,
            led_region: None,
            dirty_keys: BTreeSet::new(),
            version: 0,
        }
    }

    // ------------------------------------------------------------------
    // 脏键与版本: 全结构唯一的两个写入点, 只被本文件的 set_* 调用。
    // ------------------------------------------------------------------

    #[inline]
    fn _mark(&mut self, key: String) {
        if self.dirty_keys.insert(key) {
            self.version = self.version.wrapping_add(1);
        }
    }

    /// 撤稿: 某项值回到设备真值时清掉它的脏标记。集合空即整体不脏(派生, 无需另置标志)。
    #[inline]
    fn _drop(&mut self, key: &str) {
        if self.dirty_keys.remove(key) {
            self.version = self.version.wrapping_add(1);
        }
    }

    // ------------------------------------------------------------------
    // 状态查询(对外契约)
    // ------------------------------------------------------------------

    /// 有未保存改动。★派生自脏键集合★, 不存在可漂移的第二份真相。
    pub fn is_dirty(&self) -> bool {
        !self.dirty_keys.is_empty()
    }
    pub fn dirty_count(&self) -> usize {
        self.dirty_keys.len()
    }
    pub fn version(&self) -> u64 {
        self.version
    }

    /// 清空全部草稿与脏键。save_config 提交后 / 撤销 / 断连 / 恢复默认共用。
    /// ★12 份草稿一个不漏★: 旧 `_clear_drafts` 漏了 `kbd_combo_draft`(只清了脏键),
    /// 于是"撤销"后组合映射页仍显示被撤销的草稿, 且下次保存会把它静默提交。
    pub fn clear_all(&mut self) {
        self.cfg.clear();
        self.param.clear();
        self.global.clear();
        self.algo_cfg.clear();
        self.mode = None;
        self.kbd_map.clear();
        self.kbd_touch.clear();
        self.kbd_hold_phys.clear();
        self.kbd_hold_zone.clear();
        self.kbd_keycfg.clear();
        self.kbd_combo = None;
        self.led_region = None;
        self.dirty_keys.clear();
        self.version = self.version.wrapping_add(1);
    }

    // ------------------------------------------------------------------
    // 通用配置项 (cfg:<key>)
    // ------------------------------------------------------------------

    pub fn set_cfg(&mut self, key: &str, value: CfgValue, same_as_device: bool) {
        let dirty_key = format!("cfg:{}", key);
        if same_as_device {
            self.cfg.remove(key);
            self._drop(&dirty_key);
            return;
        }
        self.cfg.insert(key.to_string(), value);
        self._mark(dirty_key);
    }
    pub fn cfg(&self, key: &str) -> Option<&CfgValue> {
        self.cfg.get(key)
    }
    pub fn has_cfg(&self, key: &str) -> bool {
        self.cfg.contains_key(key)
    }
    pub fn cfg_items(&self) -> Vec<(String, CfgValue)> {
        self.cfg
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }

    // ------------------------------------------------------------------
    // 每通道参数 (param:<ch>:<id> / param:all:<id>)
    // ------------------------------------------------------------------

    pub fn set_param(&mut self, ch: u8, param_id: u8, value: u32, same_as_device: bool) {
        let dirty_key = format!("param:{}:{}", ch, param_id);
        if same_as_device {
            self.param.remove(&(ch, param_id));
            self._drop(&dirty_key);
            return;
        }
        self.param.insert((ch, param_id), value);
        self._mark(dirty_key);
    }
    /// 整组语义: 36 通道同写, 只记一项脏(分辨率等必须全通道一致的参数)。
    pub fn set_param_all(&mut self, param_id: u8, value: u32, same_as_device: bool) {
        let dirty_key = format!("param:all:{}", param_id);
        if same_as_device {
            for ch in 0..PARAM_CH_COUNT {
                self.param.remove(&(ch, param_id));
            }
            self._drop(&dirty_key);
            return;
        }
        for ch in 0..PARAM_CH_COUNT {
            self.param.insert((ch, param_id), value);
        }
        self._mark(dirty_key);
    }
    pub fn param(&self, ch: u8, param_id: u8) -> Option<u32> {
        self.param.get(&(ch, param_id)).copied()
    }
    pub fn param_items(&self) -> Vec<((u8, u8), u32)> {
        self.param.iter().map(|(k, v)| (*k, *v)).collect()
    }

    // ------------------------------------------------------------------
    // 全局 CSD (global:<id>)
    // ------------------------------------------------------------------

    pub fn set_global(&mut self, gparam_id: u8, value: u32, same_as_device: bool) {
        let dirty_key = format!("global:{}", gparam_id);
        if same_as_device {
            self.global.remove(&gparam_id);
            self._drop(&dirty_key);
            return;
        }
        self.global.insert(gparam_id, value);
        self._mark(dirty_key);
    }
    pub fn global(&self, gparam_id: u8) -> Option<u32> {
        self.global.get(&gparam_id).copied()
    }
    pub fn global_items(&self) -> Vec<(u8, u32)> {
        self.global.iter().map(|(k, v)| (*k, *v)).collect()
    }

    // ------------------------------------------------------------------
    // 算法可设置变量 (algo:cfg:<idx>)
    // ------------------------------------------------------------------

    pub fn set_algo_cfg(&mut self, idx: u8, val: u8, same_as_device: bool) {
        let dirty_key = format!("algo:cfg:{}", idx);
        if same_as_device {
            self.algo_cfg.remove(&idx);
            self._drop(&dirty_key);
            return;
        }
        self.algo_cfg.insert(idx, val);
        self._mark(dirty_key);
    }
    pub fn algo_cfg(&self, idx: u8) -> Option<u8> {
        self.algo_cfg.get(&idx).copied()
    }
    pub fn algo_cfg_items(&self) -> Vec<(u8, u8)> {
        self.algo_cfg.iter().map(|(k, v)| (*k, *v)).collect()
    }
    /// 是否存在算法变量草稿: 清空/提交后需据此决定是否 bump `algo_cfg_version`。
    pub fn has_algo_cfg(&self) -> bool {
        !self.algo_cfg.is_empty()
    }

    // ------------------------------------------------------------------
    // CSD 处理模式 (mode)
    // ------------------------------------------------------------------

    pub fn set_mode(&mut self, mode: u8, same_as_device: bool) {
        if same_as_device {
            self.mode = None;
            self._drop(KEY_MODE);
            return;
        }
        self.mode = Some(mode);
        self._mark(KEY_MODE.to_string());
    }
    pub fn mode(&self) -> Option<u8> {
        self.mode
    }

    // ------------------------------------------------------------------
    // 键盘物理键映射 (kbd:phys:<idx>) / 触控键映射 (kbd:zone:<zone>)
    // ------------------------------------------------------------------

    pub fn set_kbd_map(&mut self, idx: u8, keycode: u8, modifier: u8, same_as_device: bool) {
        let dirty_key = format!("kbd:phys:{}", idx);
        if same_as_device {
            self.kbd_map.remove(&idx);
            self._drop(&dirty_key);
            return;
        }
        self.kbd_map.insert(idx, (keycode, modifier));
        self._mark(dirty_key);
    }
    pub fn kbd_map(&self, idx: u8) -> Option<(u8, u8)> {
        self.kbd_map.get(&idx).copied()
    }
    pub fn kbd_map_items(&self) -> Vec<(u8, (u8, u8))> {
        self.kbd_map.iter().map(|(k, v)| (*k, *v)).collect()
    }

    pub fn set_kbd_touch(&mut self, zone: u8, keycode: u8, modifier: u8, same_as_device: bool) {
        let dirty_key = format!("kbd:zone:{}", zone);
        if same_as_device {
            self.kbd_touch.remove(&zone);
            self._drop(&dirty_key);
            return;
        }
        self.kbd_touch.insert(zone, (keycode, modifier));
        self._mark(dirty_key);
    }
    pub fn kbd_touch(&self, zone: u8) -> Option<(u8, u8)> {
        self.kbd_touch.get(&zone).copied()
    }
    pub fn kbd_touch_items(&self) -> Vec<(u8, (u8, u8))> {
        self.kbd_touch.iter().map(|(k, v)| (*k, *v)).collect()
    }

    // ------------------------------------------------------------------
    // 长按参数 (kbd:hold:phys:<idx> / kbd:hold:zone:<zone>)
    // ------------------------------------------------------------------

    pub fn set_kbd_hold_phys(&mut self, idx: u8, hold: HoldParam, same_as_device: bool) {
        let dirty_key = format!("kbd:hold:phys:{}", idx);
        if same_as_device {
            self.kbd_hold_phys.remove(&idx);
            self._drop(&dirty_key);
            return;
        }
        self.kbd_hold_phys.insert(idx, hold);
        self._mark(dirty_key);
    }
    pub fn set_kbd_hold_zone(&mut self, zone: u8, hold: HoldParam, same_as_device: bool) {
        let dirty_key = format!("kbd:hold:zone:{}", zone);
        if same_as_device {
            self.kbd_hold_zone.remove(&zone);
            self._drop(&dirty_key);
            return;
        }
        self.kbd_hold_zone.insert(zone, hold);
        self._mark(dirty_key);
    }
    pub fn kbd_hold_phys(&self, idx: u8) -> Option<HoldParam> {
        self.kbd_hold_phys.get(&idx).copied()
    }
    pub fn kbd_hold_zone(&self, zone: u8) -> Option<HoldParam> {
        self.kbd_hold_zone.get(&zone).copied()
    }
    /// 物理键 + 分区长按草稿合并为 KBD_SET_HOLD 的项数组(整批一帧下发)。
    pub fn kbd_hold_items(&self) -> Vec<KbdHoldItem> {
        self.kbd_hold_phys
            .iter()
            .map(|(idx, hold)| KbdHoldItem {
                kind: KBD_HOLD_KIND_PHYS,
                idx: *idx,
                hold: *hold,
            })
            .chain(self.kbd_hold_zone.iter().map(|(idx, hold)| KbdHoldItem {
                kind: KBD_HOLD_KIND_ZONE,
                idx: *idx,
                hold: *hold,
            }))
            .collect()
    }

    // ------------------------------------------------------------------
    // 每键触发极性 + 独立防抖 (kbd:keycfg:<idx>)
    // ------------------------------------------------------------------

    pub fn set_kbd_keycfg(&mut self, idx: u8, cfg: KbdKeyCfg, same_as_device: bool) {
        let dirty_key = format!("kbd:keycfg:{}", idx);
        if same_as_device {
            self.kbd_keycfg.remove(&idx);
            self._drop(&dirty_key);
            return;
        }
        self.kbd_keycfg.insert(idx, cfg);
        self._mark(dirty_key);
    }
    pub fn kbd_keycfg(&self, idx: u8) -> Option<KbdKeyCfg> {
        self.kbd_keycfg.get(&idx).copied()
    }
    pub fn kbd_keycfg_items(&self) -> Vec<(u8, KbdKeyCfg)> {
        self.kbd_keycfg.iter().map(|(k, v)| (*k, *v)).collect()
    }

    // ------------------------------------------------------------------
    // 触控组合映射 (kbd:combo, 整表替换语义)
    // ------------------------------------------------------------------

    /// 整表暂存。`same_as_device` = 该表与设备缓存**完全相等** ⇒ 视为撤稿,
    /// 与 param/keycfg 同口径; 否则"改回原样"会被派生脏误报成未保存。
    pub fn set_kbd_combo(&mut self, table: Vec<KbdComboItem>, same_as_device: bool) {
        if same_as_device {
            self.kbd_combo = None;
            self._drop(KEY_KBD_COMBO);
            return;
        }
        self.kbd_combo = Some(table);
        self._mark(KEY_KBD_COMBO.to_string());
    }
    pub fn kbd_combo(&self) -> Option<&[KbdComboItem]> {
        self.kbd_combo.as_deref()
    }
    /// 回读到设备真值后丢弃草稿(与其它页"回读即真值"口径一致)。
    pub fn drop_kbd_combo(&mut self) {
        self.kbd_combo = None;
        self._drop(KEY_KBD_COMBO);
    }
    /// save_config 提交路径: 取出整表用于下发, 同时撤销其脏键。
    pub fn take_kbd_combo(&mut self) -> Option<Vec<KbdComboItem>> {
        let table = self.kbd_combo.take();
        if table.is_some() {
            self._drop(KEY_KBD_COMBO);
        }
        table
    }

    // ------------------------------------------------------------------
    // 灯效单元映射(★不登记脏键★, 见文件头例外说明)
    // ------------------------------------------------------------------

    pub fn set_led_region(&mut self, regions: [LedRegion; LED_UNIT_COUNT]) {
        self.led_region = Some(regions);
    }
    pub fn led_region(&self) -> Option<&[LedRegion; LED_UNIT_COUNT]> {
        self.led_region.as_ref()
    }
    pub fn drop_led_region(&mut self) {
        self.led_region = None;
    }
}
