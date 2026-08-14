use crate::app_state::{AppController, KBD_EDGE_KEEP, zone_label};
use crate::proto::*;
use std::collections::VecDeque;

impl AppController {
    pub fn kbd_request_state(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetState as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }
    // ------------------------------------------------------------------
    // 触控组合映射(多分区 → 多键)。草稿制: 编辑只进 `drafts` 的组合整表, 由"保存到设备"下发。
    // 固件侧是整表替换语义, 所以草稿也按整表管理, 不做逐条增删的增量协议。
    // ------------------------------------------------------------------

    pub fn kbd_combo_version(&self) -> u64 {
        self.kbd_combo_version
    }

    /// 待新建条目的分区集合(34 位)与键组合。属于纯编辑态, 但放在这里而不是 Slint:
    /// "添加"时要与已有表做去重判定, 判定逻辑在 Rust, 状态跟着一起放才不会两边不一致。
    pub fn kbd_combo_pending_zones(&self) -> Vec<bool> {
        (0..34u8)
            .map(|z| (self.kbd_combo_pending_mask & (1u64 << z)) != 0)
            .collect()
    }

    /// 34 位: 该分区是否已被某条映射用到(仅 UI 提示, 不禁止复用)。
    pub fn kbd_combo_zone_used(&self) -> Vec<bool> {
        let table = self.kbd_combos();
        let union = table.iter().fold(0u64, |acc, c| acc | c.zone_mask);
        (0..34u8).map(|z| (union & (1u64 << z)) != 0).collect()
    }

    pub fn kbd_combo_pending_keys(&self) -> ([u8; crate::proto::KBD_COMBO_KEY_COUNT], u8) {
        (self.kbd_combo_pending_keys, self.kbd_combo_pending_mods)
    }

    pub fn kbd_combo_toggle_zone(&mut self, zone: u8) {
        if zone >= 34 {
            return;
        }
        self.kbd_combo_pending_mask ^= 1u64 << zone;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    pub fn kbd_combo_clear_zones(&mut self) {
        self.kbd_combo_pending_mask = 0;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    /// 抓到一个键: 追加到待新建的键位数组(去重, 满 4 个后忽略并提示)。修饰位取并集。
    /// ★这里的累加是"一次录制会话内"的语义★(用于"同时按住 F5+F6"这类真实和弦):
    /// 跨会话的覆盖由 UI 侧完成 —— KeyCaptureBox 每次进入录制态先调 kbd_combo_clear_keys(),
    /// 所以不同时间按下的键不会攒成一条多键映射。本函数因此不需要自己判断会话边界。
    pub fn kbd_combo_capture_key(&mut self, keycode: u8, mods: u8) {
        self.kbd_combo_pending_mods |= mods;
        if keycode != 0 {
            if self.kbd_combo_pending_keys.contains(&keycode) {
                // 同一键重复抓取不算错, 静默忽略即可。
            } else if let Some(slot) = self.kbd_combo_pending_keys.iter_mut().find(|k| **k == 0) {
                *slot = keycode;
            } else {
                self.push_log_warn(format!(
                    "组合映射: 单条最多 {} 个键, 已忽略新键。",
                    crate::proto::KBD_COMBO_KEY_COUNT
                ));
            }
        }
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    pub fn kbd_combo_clear_keys(&mut self) {
        self.kbd_combo_pending_keys = [0; crate::proto::KBD_COMBO_KEY_COUNT];
        self.kbd_combo_pending_mods = 0;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    /// 把待新建条目提交成一条映射。时间参数由新建区的两个输入框给出(不再硬编码 0/0)。
    /// 成功后只清空分区与按键; ★两个时间参数留在 UI 侧且刻意不清零★(连续添加同类映射不必重填)。
    pub fn kbd_combo_commit_pending(&mut self, delay_ms: u16, max_hold_ms: u16) {
        let mask = self.kbd_combo_pending_mask;
        let keys = self.kbd_combo_pending_keys;
        let mods = self.kbd_combo_pending_mods;
        if self.kbd_combo_add(mask, keys, mods, delay_ms, max_hold_ms) {
            self.kbd_combo_pending_mask = 0;
            self.kbd_combo_pending_keys = [0; crate::proto::KBD_COMBO_KEY_COUNT];
            self.kbd_combo_pending_mods = 0;
            self.push_log("组合映射: 已加入草稿, 点左下角“保存到设备”才真正下发。".to_string());
        }
    }

    /// 当前生效的组合表(草稿优先)。未回读且无草稿时为空表。
    pub fn kbd_combos(&self) -> Vec<crate::proto::KbdComboItem> {
        match self.drafts.kbd_combo() {
            Some(d) => d.to_vec(),
            None => self.kbd_combo_cache.clone(),
        }
    }

    /// 设备是否支持组合映射。None=还没问过; Some(false)=固件回了 NAK(旧固件)。
    pub fn kbd_combo_supported(&self) -> Option<bool> {
        self.kbd_combo_supported
    }

    pub fn kbd_request_combo(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetCombo as u8, 0, seq, vec![]))?;
            self.kbd_combo_req_seq = Some(seq);
        }
        Ok(())
    }

    /// 新增一条组合。分区集合为空、或与已有条目**完全相同**时拒绝(与固件去重规则一致:
    /// 只要分区集合不完全一样就允许共存)。返回是否加入成功。
    pub fn kbd_combo_add(
        &mut self,
        zone_mask: u64,
        keycodes: [u8; crate::proto::KBD_COMBO_KEY_COUNT],
        modifiers: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> bool {
        if zone_mask == 0 {
            self.push_log_warn("组合映射: 未选择任何触控分区, 未添加。".to_string());
            return false;
        }
        if keycodes.iter().all(|k| *k == 0) && modifiers == 0 {
            self.push_log_warn("组合映射: 未指定任何按键, 未添加。".to_string());
            return false;
        }
        let mut table = self.kbd_combos();
        if table.len() >= crate::proto::KBD_COMBO_COUNT {
            self.push_log_warn(format!(
                "组合映射: 已达上限 {} 条, 未添加。",
                crate::proto::KBD_COMBO_COUNT
            ));
            return false;
        }
        if table.iter().any(|c| c.zone_mask == zone_mask) {
            self.push_log_warn(
                "组合映射: 已存在分区集合完全相同的条目, 未添加(改按键请先删除旧条目)。"
                    .to_string(),
            );
            return false;
        }
        table.push(crate::proto::KbdComboItem {
            zone_mask,
            keycodes,
            modifiers,
            delay_ms,
            max_hold_ms,
        });
        self._stage_combo(table);
        true
    }

    pub fn kbd_combo_remove(&mut self, index: usize) {
        let mut table = self.kbd_combos();
        if index >= table.len() {
            return;
        }
        table.remove(index);
        // 删除会让后续行号整体前移, 留着旧登记等于把"替换"落到另一行上, 直接撤销。
        self.kbd_combo_key_edit_row = None;
        self._stage_combo(table);
    }

    /// 改某条的时间参数(ms)。分区集合仍然只能删了重加(单一编辑路径); 按键可就地重录, 见下。
    pub fn kbd_combo_set_hold(&mut self, index: usize, delay_ms: u16, max_hold_ms: u16) {
        let mut table = self.kbd_combos();
        if index >= table.len() {
            return;
        }
        table[index].delay_ms = delay_ms;
        table[index].max_hold_ms = max_hold_ms;
        self._stage_combo(table);
    }

    /// 登记"下一次抓到的键整行替换第 index 行"。★只登记, 绝不在这里清空该行按键★:
    /// 清空会在草稿里留下一条 0 键映射, 用户此刻点"保存到设备"就会把空条目下发出去。
    /// 改成"抓到键的那一刻才替换", 于是 0 键中间态根本不存在, 也就无从被保存。
    pub fn kbd_combo_begin_edit_keys(&mut self, index: usize) {
        if index >= self.kbd_combos().len() {
            return;
        }
        self.kbd_combo_key_edit_row = Some(index);
    }

    /// 就地重录第 index 行的按键组合。语义与新建区 `kbd_combo_capture_key` 完全同口径 ——
    /// ★跨会话覆盖、会话内累加★: 本行有待替换登记(由 KeyCaptureBox 的 capture_started 边沿
    /// 打上, 一次录制会话只打一次)时本键替换整行; 否则累加(同键去重、修饰位取并集、
    /// 满 KBD_COMBO_KEY_COUNT 忽略并告警)。所以"同时按住 F5+F6"照旧能录成多键。
    pub fn kbd_combo_capture_key_at(&mut self, index: usize, keycode: u8, mods: u8) {
        let mut table = self.kbd_combos();
        if index >= table.len() {
            return;
        }
        if self.kbd_combo_key_edit_row == Some(index) {
            self.kbd_combo_key_edit_row = None;
            table[index].keycodes = [0; crate::proto::KBD_COMBO_KEY_COUNT];
            table[index].modifiers = 0;
        }
        table[index].modifiers |= mods;
        if keycode != 0 {
            if table[index].keycodes.contains(&keycode) {
                // 同一键重复抓取不算错, 静默忽略。
            } else if let Some(slot) = table[index].keycodes.iter_mut().find(|k| **k == 0) {
                *slot = keycode;
            } else {
                self.push_log_warn(format!(
                    "组合映射: 单条最多 {} 个键, 已忽略新键。",
                    crate::proto::KBD_COMBO_KEY_COUNT
                ));
            }
        }
        self._stage_combo(table);
    }

    /// 用整张表覆盖组合映射草稿(JSON 导入用)。
    /// ★为什么单独开一个入口★: 导入是"整表替换", 而 `kbd_combo_add` 带一堆交互态校验
    /// (上限提示/重复分区拒绝/空键拒绝)并逐条追加 —— 拿它做导入会把文件里合法的整表判成冲突。
    /// 合法性已在导入侧按同一口径过滤(空条目丢弃、超上限丢弃), 这里只负责落草稿。
    pub fn kbd_combo_replace_table(&mut self, table: Vec<crate::proto::KbdComboItem>) {
        self.kbd_combo_key_edit_row = None;
        self._stage_combo(table);
    }

    /// 组合映射整表入草稿。★与设备缓存完全相等即撤稿★(与 param/keycfg 同口径):
    /// 脏状态派生自脏键集合, 若不做这一步, "删了又加回原样"会被永久误报为未保存。
    /// 脏键登记与 version bump 由 `ConfigDrafts::set_kbd_combo` 内部完成, 无从遗漏
    /// —— 旧实现在这里只插脏键却漏了 `mark_config_dirty()`, 导致"添加组合映射后按钮不亮"。
    fn _stage_combo(&mut self, table: Vec<crate::proto::KbdComboItem>) {
        let same_as_device = table == self.kbd_combo_cache;
        self.drafts.set_kbd_combo(table, same_as_device);
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    /// 由 save_config 调用: 把草稿整表排入统一写队列。无草稿则什么都不做。
    pub(crate) fn _commit_combo(&mut self, queued: &mut Vec<Frame>) -> anyhow::Result<()> {
        let Some(table) = self.drafts.take_kbd_combo() else {
            return Ok(());
        };
        let payload = crate::proto::encode_kbd_set_combo(&table);
        let seq = self.next_seq();
        queued.push(Frame::new(HostCmd::KbdSetCombo as u8, 0, seq, payload));
        // 乐观写缓存: 固件对整表做了去重与空条目丢弃, 真值由随后的回读校正。
        self.kbd_combo_cache = table;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
        Ok(())
    }

    pub fn kbd_request_map(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetMap as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }
    pub fn kbd_request_touchmap(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetTouchmap as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }
    /// 暂存物理键 idx(0..11) 的 HID 键码 + 修饰位(bit0 Ctrl/1 Shift/2 Alt/3 Gui)到草稿。
    pub fn kbd_set_map(&mut self, idx: u8, keycode: u8, modifier: u8) -> anyhow::Result<()> {
        if idx >= 12 {
            return Err(anyhow::anyhow!("物理键索引非法: {}", idx));
        }
        let i = idx as usize;
        let same_as_device =
            self.kbd_map.get(i) == Some(&keycode) && self.kbd_keymod.get(i) == Some(&modifier);
        self.drafts
            .set_kbd_map(idx, keycode, modifier, same_as_device);
        // ★必须 bump★: main.rs 的键码/修饰位/显示文本回填全部由该 version 门控; 只写草稿不 bump
        // 会让"捕获组合键"在界面上毫无反应(看起来只是退出了录制态), 用户无法确认是否落地。
        // 撤稿分支同样要 bump: getter 是"草稿优先", 撤回后可见值变回设备真值, UI 必须跟着回填。
        self.kbd_map_version = self.kbd_map_version.wrapping_add(1);
        Ok(())
    }
    /// 暂存触控分区 zone(0..33) 的 HID 键码 + 修饰位到草稿。
    pub fn kbd_set_touchmap(&mut self, zone: u8, keycode: u8, modifier: u8) -> anyhow::Result<()> {
        if zone >= 34 {
            return Err(anyhow::anyhow!("分区索引非法: {}", zone));
        }
        let z = zone as usize;
        let same_as_device = self.kbd_touchmap.get(z) == Some(&keycode)
            && self.kbd_zonemod.get(z) == Some(&modifier);
        self.drafts
            .set_kbd_touch(zone, keycode, modifier, same_as_device);
        // ★必须 bump★: 见 kbd_set_map 说明 —— 显示文本/下拉索引/修饰位都由该 version 门控回填。
        self.kbd_touchmap_version = self.kbd_touchmap_version.wrapping_add(1);
        Ok(())
    }
    pub fn kbd_state(&self) -> u16 {
        self.kbd_state
    }
    pub fn kbd_state_version(&self) -> u64 {
        self.kbd_state_version
    }
    /// 触控→键盘链路诊断的最近一次实测读数。None = 该段读不到, 调用方必须显式说明"不可用"。
    pub fn kbd_link_diag(&self) -> Option<&crate::proto::KbdLinkDiag> {
        self.kbd_link_diag.as_ref()
    }
    pub fn kbd_map(&self, idx: u8) -> u8 {
        if let Some((code, _)) = self.drafts.kbd_map(idx) {
            return code;
        }
        *self.kbd_map.get(idx as usize).unwrap_or(&0)
    }
    pub fn kbd_keymod(&self, idx: u8) -> u8 {
        if let Some((_, m)) = self.drafts.kbd_map(idx) {
            return m;
        }
        *self.kbd_keymod.get(idx as usize).unwrap_or(&0)
    }
    pub fn kbd_map_version(&self) -> u64 {
        self.kbd_map_version
    }
    pub fn kbd_touch_keycode(&self, zone: u8) -> u8 {
        if let Some((code, _)) = self.drafts.kbd_touch(zone) {
            return code;
        }
        *self.kbd_touchmap.get(zone as usize).unwrap_or(&0)
    }
    pub fn kbd_zone_mod(&self, zone: u8) -> u8 {
        if let Some((_, m)) = self.drafts.kbd_touch(zone) {
            return m;
        }
        *self.kbd_zonemod.get(zone as usize).unwrap_or(&0)
    }
    pub fn kbd_touch_en(&self) -> bool {
        self.kbd_touch_en
    }
    pub fn kbd_touchmap_version(&self) -> u64 {
        self.kbd_touchmap_version
    }

    pub(crate) fn _handle_kbd_get_state_response(&mut self, frame: &Frame) {
        if frame.payload.len() >= 2 {
            self.kbd_state = (frame.payload[0] as u16) | ((frame.payload[1] as u16) << 8);
            // 尾部追加的 raw/out 两态(新固件才有)。旧固件没有就保持 0, 不用 phys_state 冒充 ——
            // 否则界面会显示"原始态与去抖态永远一致", 等于把防抖效果伪装成不存在。
            if frame.payload.len() >= 6 {
                self.kbd_state_raw = (frame.payload[2] as u16) | ((frame.payload[3] as u16) << 8);
                self.kbd_state_out = (frame.payload[4] as u16) | ((frame.payload[5] as u16) << 8);
            }
            // 第二段尾部: 触控→键盘链路诊断。读不到就写 None(界面据此显示"诊断字段不可用"),
            // ★不保留上一次的旧读数★ —— 那会让"固件不回传了"看起来像"链路还正常"。
            self.kbd_link_diag = crate::proto::decode_kbd_link_diag(&frame.payload);
            self.kbd_state_version = self.kbd_state_version.wrapping_add(1);
        }
    }
    pub(crate) fn _handle_kbd_get_map_response(&mut self, frame: &Frame) {
        if frame.payload.is_empty() {
            return;
        }
        // 每键 2 字节: [keycode, modifier]。
        let count = frame.payload[0] as usize;
        for i in 0..count.min(12) {
            if let Some(&code) = frame.payload.get(1 + i * 2) {
                self.kbd_map[i] = code;
            }
            if let Some(&m) = frame.payload.get(2 + i * 2) {
                self.kbd_keymod[i] = m;
            }
        }
        self.kbd_map_version = self.kbd_map_version.wrapping_add(1);
    }
    pub(crate) fn _handle_kbd_get_touchmap_response(&mut self, frame: &Frame) {
        if frame.payload.len() < 2 {
            return;
        }
        // [en, count, (keycode, modifier)×count]。
        self.kbd_touch_en = frame.payload[0] != 0;
        let count = frame.payload[1] as usize;
        for z in 0..count.min(34) {
            if let Some(&code) = frame.payload.get(2 + z * 2) {
                self.kbd_touchmap[z] = code;
            }
            if let Some(&m) = frame.payload.get(3 + z * 2) {
                self.kbd_zonemod[z] = m;
            }
        }
        self.kbd_touchmap_version = self.kbd_touchmap_version.wrapping_add(1);
    }

    // ------------------------------------------------------------------
    // 键盘长按参数 (KBD_GET_HOLD / KBD_SET_HOLD)、mai2 串口运行态 (MAI2_*)
    // 与掉线重连自动恢复运行态
    // ------------------------------------------------------------------

    /// 触控→键盘映射总开关的配置 key(重连恢复期望态也按此 key 下发)。
    pub(crate) const KBD_MAP_EN_KEY: &'static str = "comm.keyboard_map_en";

    /// 物理键 idx(0..11) 的长按参数：草稿优先 → 设备回读兜底。
    pub fn kbd_hold_phys(&self, idx: u8) -> (u16, u16) {
        let hold = self
            .drafts
            .kbd_hold_phys(idx)
            .or_else(|| self.kbd_hold_phys.get(idx as usize).copied())
            .unwrap_or_default();
        (hold.delay_ms, hold.max_hold_ms)
    }

    /// 触控分区 zone(0..33) 的长按参数：草稿优先 → 设备回读兜底。
    pub fn kbd_hold_zone(&self, zone: u8) -> (u16, u16) {
        let hold = self
            .drafts
            .kbd_hold_zone(zone)
            .or_else(|| self.kbd_hold_zone.get(zone as usize).copied())
            .unwrap_or_default();
        (hold.delay_ms, hold.max_hold_ms)
    }

    /// 长按参数版本号(下发/回读时自增), 供 UI 判断是否刷新。
    pub fn kbd_hold_version(&self) -> u64 {
        self.kbd_hold_version
    }

    /// 暂存长按参数到草稿(不下发)，供 JSON 导入等批量外部写入使用。
    pub fn stage_kbd_hold(
        &mut self,
        kind: u8,
        idx: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> anyhow::Result<()> {
        match kind {
            KBD_HOLD_KIND_PHYS => self.kbd_set_hold_phys(idx, delay_ms, max_hold_ms),
            KBD_HOLD_KIND_ZONE => self.kbd_set_hold_zone(idx, delay_ms, max_hold_ms),
            _ => Err(anyhow::anyhow!("长按参数类型非法: {}", kind)),
        }
    }

    /// 暂存物理键长按参数到草稿，不立即下发。
    pub fn kbd_set_hold_phys(
        &mut self,
        idx: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> anyhow::Result<()> {
        if (idx as usize) >= KBD_HOLD_PHYS_COUNT {
            return Err(anyhow::anyhow!("物理键长按索引非法: {}", idx));
        }
        let hold = HoldParam {
            delay_ms,
            max_hold_ms,
        };
        let same_as_device = self.kbd_hold_phys.get(idx as usize) == Some(&hold);
        self.drafts.set_kbd_hold_phys(idx, hold, same_as_device);
        self.kbd_hold_version = self.kbd_hold_version.wrapping_add(1);
        Ok(())
    }

    /// 暂存触控分区长按参数到草稿，不立即下发。
    pub fn kbd_set_hold_zone(
        &mut self,
        zone: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> anyhow::Result<()> {
        if (zone as usize) >= KBD_HOLD_ZONE_COUNT {
            return Err(anyhow::anyhow!("触控分区长按索引非法: {}", zone));
        }
        let hold = HoldParam {
            delay_ms,
            max_hold_ms,
        };
        let same_as_device = self.kbd_hold_zone.get(zone as usize) == Some(&hold);
        self.drafts.set_kbd_hold_zone(zone, hold, same_as_device);
        self.kbd_hold_version = self.kbd_hold_version.wrapping_add(1);
        Ok(())
    }

    /// 请求回读全部长按参数(12 物理键 + 34 分区)。
    pub fn kbd_request_hold(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetHold as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }

    // ------------------------------------------------------------------
    // 物理键每键配置(触发极性 + 独立防抖)与逻辑分析仪边沿记录
    // 草稿制与长按参数完全同一套: 编辑只进 `drafts` 的每键配置草稿, "保存到设备"才下发。
    // ------------------------------------------------------------------

    /// 物理键 idx(0..11) 的触发极性与防抖窗: 草稿优先 → 设备回读兜底。
    pub fn kbd_keycfg(&self, idx: u8) -> KbdKeyCfg {
        self.drafts
            .kbd_keycfg(idx)
            .or_else(|| self.kbd_keycfg.get(idx as usize).copied())
            .unwrap_or_default()
    }

    pub fn kbd_keycfg_version(&self) -> u64 {
        self.kbd_keycfg_version
    }

    /// 固件解析后的生效极性掩码(bit i = 1 → 该键按高电平触发判定)。
    /// AUTO 档的判定结果只能从这里读 —— `kbd_keycfg().pol` 是配置态, 值为 2 时不含判定结论。
    pub fn kbd_pol_resolved_mask(&self) -> u16 {
        self.kbd_pol_resolved_mask
    }

    /// 生效极性掩码是否有真实来源。false = 旧固件未回传 → 生效电平**未知**, UI 显示"未知"。
    pub fn kbd_pol_resolved_known(&self) -> bool {
        self.kbd_pol_resolved_known
    }

    /// 设备是否支持每键配置。None=还没问过; Some(false)=旧固件 NAK。
    pub fn kbd_keycfg_supported(&self) -> Option<bool> {
        self.kbd_keycfg_supported
    }

    /// 暂存每键触发极性 + 防抖窗到草稿(不下发)。
    /// 防抖上限与固件同源(`KBD_DEBOUNCE_US_MAX`): 越界直接报错, 不下发一个必被 NAK 的值。
    pub fn kbd_set_keycfg(&mut self, idx: u8, pol: u8, debounce_us: u16) -> anyhow::Result<()> {
        if (idx as usize) >= KBD_HOLD_PHYS_COUNT {
            return Err(anyhow::anyhow!("物理键索引非法: {}", idx));
        }
        // 极性围栏与固件 _handle_set_keycfg 同源: 越界直接拒绝, 不下发一个必被 NAK 的值。
        if pol > crate::proto::KBD_POL_AUTO {
            return Err(anyhow::anyhow!(
                "触发极性非法: {} (0=低电平 1=高电平 2=自动)",
                pol
            ));
        }
        if debounce_us > KBD_DEBOUNCE_US_MAX {
            return Err(anyhow::anyhow!(
                "防抖时间超出范围: {}us (上限 {}us)",
                debounce_us,
                KBD_DEBOUNCE_US_MAX
            ));
        }
        let cfg = KbdKeyCfg { pol, debounce_us };
        // 改回设备真值即自动撤稿(与 kbd_set_hold_phys 同口径), 免得界面一直挂着"未保存"。
        let same_as_device = self.kbd_keycfg.get(idx as usize) == Some(&cfg);
        self.drafts.set_kbd_keycfg(idx, cfg, same_as_device);
        self.kbd_keycfg_version = self.kbd_keycfg_version.wrapping_add(1);
        Ok(())
    }

    /// 请求回读 12 个物理键的极性与防抖窗。
    pub fn kbd_request_keycfg(&mut self) -> anyhow::Result<()> {
        if self.kbd_keycfg_supported == Some(false) {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetKeycfg as u8, 0, seq, vec![]))?;
            self.kbd_keycfg_req_seq = Some(seq);
        }
        Ok(())
    }

    /// 调试直发: 绕过草稿与"保存到设备", 立刻下发一帧 KBD_SET_MAP。
    ///
    /// ★为什么允许有这条旁路★ HID 键盘输出链只有"真人按键/真人触摸"一个入口, 无头环境下
    /// 无法自持验证"报文到底有没有被主机认成按键"。固件对 KBD_SET_MAP / KBD_SET_KEYCFG 只写
    /// RAM 影子(flash 落地由 SAVE_CONFIG 单独触发, 见 keyboard.cpp 各 _handle_set_* 的注释),
    /// 因此本旁路改动**不落盘、复位即消失**, 不会污染用户配置。
    /// 不进草稿是刻意的: 草稿会把这次临时改动显示成"未保存修改"并在下次保存时写进 flash。
    pub fn kbd_send_map_now(&mut self, idx: u8, keycode: u8, modifier: u8) -> anyhow::Result<()> {
        if idx >= 12 {
            return Err(anyhow::anyhow!("物理键索引非法: {}", idx));
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(
                HostCmd::KbdSetMap as u8,
                0,
                seq,
                vec![idx, keycode, modifier],
            ))?;
        }
        Ok(())
    }

    /// 调试直发: 绕过草稿, 立刻下发一帧 KBD_SET_KEYCFG。语义与落盘口径见 `kbd_send_map_now`。
    pub fn kbd_send_keycfg_now(
        &mut self,
        idx: u8,
        pol: u8,
        debounce_us: u16,
    ) -> anyhow::Result<()> {
        if (idx as usize) >= KBD_HOLD_PHYS_COUNT {
            return Err(anyhow::anyhow!("物理键索引非法: {}", idx));
        }
        if pol > crate::proto::KBD_POL_AUTO {
            return Err(anyhow::anyhow!("触发极性非法: {}", pol));
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(
                HostCmd::KbdSetKeycfg as u8,
                0,
                seq,
                crate::proto::encode_kbd_set_keycfg(&[(idx, KbdKeyCfg { pol, debounce_us })]),
            ))?;
        }
        Ok(())
    }

    /// 拉取一批边沿记录。窗口=1(有在途请求就跳过): 逻辑分析仪不能把 vendor 端点抢光,
    /// 否则会连带影响遥测与按键映射的回读。旧固件已判定不支持时直接返回。
    pub fn kbd_request_edges(&mut self) -> anyhow::Result<()> {
        if self.kbd_edges_supported == Some(false) || self.kbd_edge_req_seq.is_some() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(
                HostCmd::KbdGetEdges as u8,
                0,
                seq,
                crate::proto::encode_kbd_get_edges(0),
            ))?;
            self.kbd_edge_req_seq = Some(seq);
        }
        Ok(())
    }

    /// 主机侧留存的边沿记录(最旧在前)。
    pub fn kbd_edges(&self) -> &VecDeque<KbdEdgeRec> {
        &self.kbd_edges
    }

    pub fn kbd_edges_version(&self) -> u64 {
        self.kbd_edges_version
    }

    /// 设备是否支持边沿记录。None=还没问过; Some(false)=旧固件 NAK。
    pub fn kbd_edges_supported(&self) -> Option<bool> {
        self.kbd_edges_supported
    }

    /// 累计被设备丢弃(环满)的边沿条数。>0 时 UI 必须显示"有事件丢失", 不能假装波形连续。
    pub fn kbd_edge_lost(&self) -> u32 {
        self.kbd_edge_lost
    }

    /// 上次拉取后设备侧仍剩余的条数(>0 = 拉取跟不上设备产生速度)。
    pub fn kbd_edge_remaining(&self) -> u16 {
        self.kbd_edge_remaining
    }

    /// 清空主机侧留存窗口与丢失计数(UI "清空波形")。
    pub fn kbd_edges_clear(&mut self) {
        self.kbd_edges.clear();
        self.kbd_edge_lost = 0;
        self.kbd_edge_remaining = 0;
        self.kbd_edges_version = self.kbd_edges_version.wrapping_add(1);
    }

    /// 物理键去抖前(raw)的 12 位实时态。旧固件不提供时为 0。
    pub fn kbd_state_raw(&self) -> u16 {
        self.kbd_state_raw
    }

    /// 物理键实际输出 HID 的 12 位实时态(经去抖 + 长按状态机)。旧固件不提供时为 0。
    pub fn kbd_state_out(&self) -> u16 {
        self.kbd_state_out
    }

    pub(crate) fn _handle_kbd_get_keycfg_response(&mut self, frame: &Frame) {
        self.kbd_keycfg_req_seq = None;
        match crate::proto::decode_kbd_get_keycfg(&frame.payload) {
            Ok((list, resolved)) => {
                self.kbd_keycfg_supported = Some(true);
                for (i, cfg) in list.iter().take(KBD_HOLD_PHYS_COUNT).enumerate() {
                    self.kbd_keycfg[i] = *cfg;
                }
                // 旧固件不追加 resolved mask: 保持上次读数而不是清 0 —— 清 0 会被 UI 显示成
                // "全部判定为低电平触发", 那是凭空造出来的结论。
                if let Some(mask) = resolved {
                    self.kbd_pol_resolved_mask = mask;
                    self.kbd_pol_resolved_known = true;
                }
                self.kbd_keycfg_version = self.kbd_keycfg_version.wrapping_add(1);
            }
            Err(e) => self.push_log_warn(format!("KBD_GET_KEYCFG 解析失败: {}", e)),
        }
    }

    pub(crate) fn _handle_kbd_get_edges_response(&mut self, frame: &Frame) {
        self.kbd_edge_req_seq = None;
        let batch = match crate::proto::decode_kbd_get_edges(&frame.payload) {
            Ok(b) => b,
            Err(e) => {
                self.push_log_warn(format!("KBD_GET_EDGES 解析失败: {}", e));
                return;
            }
        };
        self.kbd_edges_supported = Some(true);
        self.kbd_edge_remaining = batch.remaining;
        // overflow 是设备累计值(只增, 设备重启才归零)。取差值累加到主机侧丢失计数;
        // 设备重启导致读数回退时按"重新计数"处理, 不产生天文数字的假丢失量。
        if batch.overflow >= self.kbd_edge_overflow_last {
            self.kbd_edge_lost = self
                .kbd_edge_lost
                .saturating_add(batch.overflow - self.kbd_edge_overflow_last);
        } else {
            self.kbd_edge_lost = self.kbd_edge_lost.saturating_add(batch.overflow);
        }
        self.kbd_edge_overflow_last = batch.overflow;
        if batch.recs.is_empty() {
            return;
        }
        for rec in batch.recs {
            if self.kbd_edges.len() >= KBD_EDGE_KEEP {
                self.kbd_edges.pop_front();
            }
            self.kbd_edges.push_back(rec);
        }
        self.kbd_edges_version = self.kbd_edges_version.wrapping_add(1);
    }

    /// mai2 串口实际有效发送态；None = 尚未回读。
    /// 用户期望只用于下发与重连恢复，绝不能覆盖设备状态机的真值。
    pub fn mai2_send_en(&self) -> Option<bool> {
        self.mai2_state.map(|s| s.send_en)
    }

    /// mai2 串口状态: 0=停 1=就绪 2=运行; None = 尚未回读。
    pub fn mai2_status(&self) -> Option<u8> {
        self.mai2_state.map(|s| s.status)
    }

    /// mai2 串口波特率; None = 尚未回读。
    pub fn mai2_baud(&self) -> Option<u32> {
        self.mai2_state.map(|s| s.baud)
    }

    /// mai2 运行态版本号(下发/回读时自增)。
    pub fn mai2_version(&self) -> u64 {
        self.mai2_version
    }

    /// 设置 mai2 串口发送使能: 立即下发 + 写期望值(掉线重连后据此自动恢复)。
    pub fn mai2_set_send_en(&mut self, en: bool) -> anyhow::Result<()> {
        self.desired.mai2_send_en = Some(en);
        self.mai2_version = self.mai2_version.wrapping_add(1);
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(
                HostCmd::Mai2SetSendEn as u8,
                0,
                seq,
                crate::proto::encode_mai2_set_send_en(en),
            ))?;
        }
        self.push_log(format!("mai2 串口发送使能 → {}", Self::_on_off(en)));
        Ok(())
    }

    /// 请求回读 mai2 串口运行态。
    pub fn mai2_request_state(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::Mai2GetState as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }

    /// 清空日志视图。
    /// 唯一的可显示数据源是 `logging::hub()` 的环形缓冲(UI 文本只读它), 所以清空必须清它,
    /// 否则点了没反应; `event_log` 只是本控制器的内部副本, 一并清掉免得留下第二份"真相"。
    /// 磁盘日志文件不受影响(继续追加), 见 logging::LogHub::clear 的说明。
    pub fn clear_log(&mut self) {
        crate::logging::hub().clear();
        self.event_log.clear();
        self.log_seq = self.log_seq.wrapping_add(1);
    }

    /// ★掉线重连自动恢复运行态★
    ///
    /// 握手完成(收到 DEVICE_INFO)时由 `handle_frame` 自动调用, 无需 UI 介入: 把用户显式设置过的
    /// 期望态(mai2 发送使能 / 触控键盘映射总开关 / 长按参数)逐条重新下发, 再拉一次真值回读对账。
    /// 也公开出来供 UI 手动重试。
    pub fn restore_after_reconnect(&mut self) -> anyhow::Result<()> {
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法恢复运行态"));
        }
        if self.desired.is_empty() {
            // 用户从未显式设置过运行态: 连接探针队列会顺序回读真实状态, 不在握手边沿直发并发请求。
            return Ok(());
        }
        // 1) 长按参数: 全部期望项合并为一帧下发。
        let mut items: Vec<KbdHoldItem> =
            Vec::with_capacity(self.desired.hold_phys.len() + self.desired.hold_zone.len());
        for (idx, hold) in self.desired.hold_phys.iter() {
            items.push(KbdHoldItem {
                kind: KBD_HOLD_KIND_PHYS,
                idx: *idx,
                hold: *hold,
            });
        }
        for (zone, hold) in self.desired.hold_zone.iter() {
            items.push(KbdHoldItem {
                kind: KBD_HOLD_KIND_ZONE,
                idx: *zone,
                hold: *hold,
            });
        }
        if !items.is_empty() {
            let payload = crate::proto::encode_kbd_set_hold(&items);
            let seq = self.next_seq();
            self._queue_tx(Frame::new(HostCmd::KbdSetHold as u8, 0, seq, payload))?;
        }
        // 2) 触控→键盘映射总开关。
        if let Some(en) = self.desired.kbd_map_en {
            let entry = ConfigEntry::new(Self::KBD_MAP_EN_KEY.to_string(), CfgValue::Bool(en));
            let payload = crate::proto::encode_entry(&entry)
                .map_err(|e| anyhow::anyhow!("encode_entry failed: {}", e))?;
            let seq = self.next_seq();
            self._queue_tx(Frame::new(HostCmd::CfgSet as u8, 0, seq, payload))?;
        }
        // 3) mai2 发送使能 —— 震动掉线后要立刻恢复触控, 这一条最关键。
        if let Some(en) = self.desired.mai2_send_en {
            let seq = self.next_seq();
            self._queue_tx(Frame::new(
                HostCmd::Mai2SetSendEn as u8,
                0,
                seq,
                crate::proto::encode_mai2_set_send_en(en),
            ))?;
        }
        self.push_log(format!(
            "重连恢复运行态: 长按 {} 项 + 触控键盘映射={} + mai2 发送使能={} 已重新下发, 连接探针将在写队列排空后回读对账",
            items.len(),
            Self::_opt_on_off(self.desired.kbd_map_en),
            Self::_opt_on_off(self.desired.mai2_send_en)));
        // 4) 回读对账由连接探针队列中的 KBD_GET_HOLD/MAI2_GET_STATE 负责，避免与恢复写入并发。
        self.restore_verify.hold = true;
        self.restore_verify.mai2 = true;
        Ok(())
    }

    fn _on_off(v: bool) -> &'static str {
        if v { "开" } else { "关" }
    }

    fn _opt_on_off(v: Option<bool>) -> &'static str {
        match v {
            Some(true) => "开",
            Some(false) => "关",
            None => "未设置",
        }
    }

    fn _mai2_status_text(status: u8) -> &'static str {
        match status {
            0 => "停",
            1 => "就绪",
            2 => "运行",
            _ => "未知",
        }
    }

    pub(crate) fn _handle_kbd_get_hold_response(&mut self, frame: &Frame) {
        let table = match crate::proto::decode_kbd_get_hold(&frame.payload) {
            Ok(t) => t,
            Err(e) => {
                self.push_log_warn(format!("KBD_GET_HOLD 解析失败: {}", e));
                self.restore_verify.hold = false;
                return;
            }
        };
        for (i, hold) in table.phys.iter().take(KBD_HOLD_PHYS_COUNT).enumerate() {
            self.kbd_hold_phys[i] = *hold;
        }
        for (i, hold) in table.zone.iter().take(KBD_HOLD_ZONE_COUNT).enumerate() {
            self.kbd_hold_zone[i] = *hold;
        }
        self.kbd_hold_version = self.kbd_hold_version.wrapping_add(1);
        if self.restore_verify.hold {
            self.restore_verify.hold = false;
            self._verify_hold_readback();
        }
    }

    /// 重连恢复后的长按参数对账: 期望值 vs 设备真值, 不一致逐条告警(设备未接受/被夹取必须可见)。
    fn _verify_hold_readback(&mut self) {
        let mut bad: Vec<String> = Vec::new();
        for (idx, want) in self.desired.hold_phys.iter() {
            let got = self
                .kbd_hold_phys
                .get(*idx as usize)
                .copied()
                .unwrap_or_default();
            if got != *want {
                bad.push(format!(
                    "物理键{}: 期望 {}/{}ms 实际 {}/{}ms",
                    *idx as u16 + 1,
                    want.delay_ms,
                    want.max_hold_ms,
                    got.delay_ms,
                    got.max_hold_ms
                ));
            }
        }
        for (zone, want) in self.desired.hold_zone.iter() {
            let got = self
                .kbd_hold_zone
                .get(*zone as usize)
                .copied()
                .unwrap_or_default();
            if got != *want {
                bad.push(format!(
                    "{}: 期望 {}/{}ms 实际 {}/{}ms",
                    zone_label(*zone as usize),
                    want.delay_ms,
                    want.max_hold_ms,
                    got.delay_ms,
                    got.max_hold_ms
                ));
            }
        }
        if bad.is_empty() {
            self.push_log("重连恢复对账: 长按参数与期望一致");
        } else {
            self.push_log_warn(format!(
                "⚠ 重连恢复对账: {} 项长按参数与期望不一致(设备未接受或已夹取): {}",
                bad.len(),
                bad.join("; ")
            ));
        }
    }

    pub(crate) fn _handle_mai2_get_state_response(&mut self, frame: &Frame) {
        let state = match crate::proto::decode_mai2_get_state(&frame.payload) {
            Ok(s) => s,
            Err(e) => {
                self.push_log_warn(format!("MAI2_GET_STATE 解析失败: {}", e));
                self.restore_verify.mai2 = false;
                return;
            }
        };
        self.mai2_state = Some(state);
        self.mai2_version = self.mai2_version.wrapping_add(1);
        if !self.restore_verify.mai2 {
            return;
        }
        self.restore_verify.mai2 = false;
        let status_text = Self::_mai2_status_text(state.status);
        match self.desired.mai2_send_en {
            Some(want) if want != state.send_en => self.push_log_warn(format!(
                "⚠ 重连恢复对账: mai2 发送使能期望 {} 但设备回读 {} (状态={} 波特率={}) → 触控可能未恢复",
                Self::_on_off(want), Self::_on_off(state.send_en), status_text, state.baud)),
            Some(want) => self.push_log(format!(
                "重连恢复对账: mai2 发送使能={} 已生效 (状态={} 波特率={})",
                Self::_on_off(want), status_text, state.baud)),
            None => {}
        }
        if state.status == 0 {
            self.push_log_warn(format!(
                "⚠ 重连恢复对账: mai2 串口状态=停(波特率={}), 游戏触控上报未运行",
                state.baud
            ));
        }
    }
    // ------------------------------------------------------------------
    // mai2light
}
