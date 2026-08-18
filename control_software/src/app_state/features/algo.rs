use crate::app_state::pipeline::RequestKind;
use crate::app_state::{
    ALGO_ABI_HEADER, AppController, CompiledAlgo, ensure_bundled_toolchain, find_toolchain_dir,
};
use crate::proto::{Frame, HostCmd};

/// 算法 C 源分片上传的在途状态。整份字节 + 已确认字节数 + 在途片的 seq 必须成组存在,
/// 否则"发到哪了/在等谁的 ACK"就成了两个可能不一致的事实。
pub(crate) struct AlgoSrcTx {
    bytes: Vec<u8>,
    /// 已被设备 ACK 的字节数(= 下一片的 offset)。
    acked: usize,
    /// None 表示等待普通写队列排空；Some(seq) 表示当前片已发出并在等 ACK/NAK。
    pub(crate) seq: Option<u8>,
    waited: u32,
}

/// 算法 C 源分片回读的聚合状态。收满 total 才替换缓存/版本 —— 半份源灌进编辑器比不灌更坏。
pub(crate) struct AlgoSrcRx {
    total: usize,
    buf: Vec<u8>,
    /// 当前请求片的序号与等待 tick，用于拒绝迟到响应并处理丢帧超时。
    seq: u8,
    waited: u32,
}

/// 算法追踪的一个采样点: 值 + 采样时刻(展开后的设备时间 us)。
///
/// ★时间是设备真值, 不再是近似★ 协议已统一给设备→主机的每一帧追加 `time_us_32()` 尾戳
/// (`proto::FLAG_TS`, 由 `Frame::device_t_us` 承载), 因此这里直接用**设备组帧那一刻**的时刻,
/// 经 `DevClock::unwrap_us` 折进与遥测同一条展开时间轴。
/// ⇒ 触发判定/上报线与 TELEM_DATA 从此是同一个时钟, 横向对位精确到设备侧组帧时刻,
/// 不再受主机轮询周期(16ms)、USB 往返抖动、GUI tick 漂移影响。
#[derive(Debug, Clone, Copy)]
pub(crate) struct TracePoint {
    pub(crate) t_us: u64,
    pub(crate) val: f32,
}

impl AppController {
    pub fn algo_get_info(&mut self) -> anyhow::Result<()> {
        let _ = self._submit_poll(
            RequestKind::AlgoGetInfo,
            crate::proto::algo::encode_algo_get_info(0),
        )?;
        Ok(())
    }
    pub fn algo_apply(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_algo_apply(seq))?;
        }
        Ok(())
    }
    pub fn algo_reset_default(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_algo_reset_default(seq))?;
        }
        // 复位命令与此查询在同一有序链路上：紧随其后取设备真值，避免调用方只能继续显示旧算法信息。
        self.algo_get_info()?;
        self.push_log("算法: 请求恢复默认(v3.1 HDR)并自动刷新设备信息");
        Ok(())
    }
    /// 上传算法二进制(≤ `algo_upload_limit()`)。内部计算 CRC16 随帧下发，并按既有写入模式等待 ACK/NAK。
    pub fn algo_upload(&mut self, data: &[u8]) -> anyhow::Result<()> {
        if data.is_empty() || data.len() > self.algo_upload_limit() {
            return Err(anyhow::anyhow!(
                "算法长度非法: {} (须 1..={} = 单帧承载上限, PSoC 槽为 {} 字节)",
                data.len(),
                self.algo_upload_limit(),
                self.algo_slot_capacity()
            ));
        }
        let seq = self.next_seq();
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接，无法上传算法"));
        }
        self._queue_tx(crate::proto::algo::encode_algo_upload(seq, data))?;
        self.algo_upload_seq = Some(seq);
        self.algo_upload_started_at = Some(std::time::Instant::now());
        // 记录本次上传的期望真值, 供 ACK 后的 ALGO_GET_INFO 回读对账(ACK 本身不是终态)。
        self.algo_upload_expect =
            Some((data.len() as u16, crate::proto::algo::crc16_ccitt(data)));
        // ★核对窗口必须覆盖满容量上传★ 每次重试间隔 50 tick(≈0.8s), 原来只给 4 次 ⇒ 总窗口
        // 只有 3.2 秒。而槽扩到 4096B 后一次满容量下发要分 1364 页、每页 2 笔 SPI 事务, 设备侧
        // 自己的上限都给到 30s —— 3.2 秒到期就宣判"终态不匹配", 实测把**正在正常进行**的大算法
        // 上传全部误判成失败(小算法侥幸能在窗口内完成, 于是表现为"小的行、大的不行", 极易误导)。
        // 40 次 × 0.8s ≈ 32s, 与设备侧 ALGO_UPLOAD_TIMEOUT_MS(30s) 对齐并留一点余量。
        self.algo_upload_verify_left = 40;
        self.algo_upload_status =
            format!("上传已发送: {} 字节，等待设备 ACK/NAK 确认…", data.len());
        self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
        self.push_log(format!(
            "算法: 上传 {} 字节 (crc16=0x{:04X}, seq={})",
            data.len(),
            crate::proto::algo::crc16_ccitt(data),
            seq
        ));
        Ok(())
    }
    pub fn algo_get_rom(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_algo_get_rom(seq))?;
        }
        Ok(())
    }
    /// 设置每通道 16 位 ROM。entries=(ch, rom) 列表(1..=36 项)。
    pub fn algo_set_rom(&mut self, entries: &[(u8, u16)]) -> anyhow::Result<()> {
        if entries.is_empty() || entries.len() > 36 {
            return Err(anyhow::anyhow!("ROM 条目数非法: {}", entries.len()));
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_algo_set_rom(seq, entries))?;
        }
        for &(ch, rom) in entries {
            if (ch as usize) < self.algo_rom.len() {
                self.algo_rom[ch as usize] = rom;
            }
        }
        self.algo_rom_version = self.algo_rom_version.wrapping_add(1);
        Ok(())
    }

    pub fn algo_info(&self) -> Option<crate::proto::algo::AlgoInfo> {
        self.algo_info
    }
    pub fn algo_version(&self) -> u64 {
        self.algo_version
    }
    /// 最近一次算法上传的设备确认结果(发送中 / ACK / NAK / 超时)，由 main.rs 回填状态行。
    pub fn algo_upload_status(&self) -> &str {
        &self.algo_upload_status
    }
    pub fn algo_upload_version(&self) -> u64 {
        self.algo_upload_version
    }
    pub fn algo_rom(&self) -> &[u16] {
        &self.algo_rom
    }
    pub fn algo_rom_version(&self) -> u64 {
        self.algo_rom_version
    }

    // ------------------------------------------------------------------
    // 算法运行时追踪(report[]/out_active) + 可调变量(cfg[8])
    // ------------------------------------------------------------------

    /// 某上报变量(idx 0..3)的**最新**值。O(1) 只看队尾。
    /// ★取最新值不要走"整条缓冲克隆成 Vec 再取末位"★: 缓冲上限 TELEM_CAP=16384 点, 而随帧
    /// 上报后这条路径每个可见 tick 都会走, 逐帧克隆几万个点纯属白烧 CPU(那正是旧写法的代价)。
    pub fn algo_trace_report_last(&self, idx: u8) -> Option<f32> {
        self.algo_trace_report
            .get(idx as usize)
            .and_then(|buf| buf.back())
            .map(|point| point.val)
    }

    /// 某上报变量(idx 0..3)的 (设备时间us, 值) 序列: 供与遥测曲线共用真实时间轴的主图。
    pub fn algo_trace_report_points(&self, idx: u8) -> Vec<(u64, f32)> {
        let report = match &self.plot_freeze {
            Some(freeze) => &freeze.algo_report,
            None => &self.algo_trace_report,
        };
        report
            .get(idx as usize)
            .map(|buf| buf.iter().map(|p| (p.t_us, p.val)).collect())
            .unwrap_or_default()
    }
    /// 触发判定(out_active)的 (设备时间us, 值) 序列, 同上。
    pub fn algo_trace_active_points(&self) -> Vec<(u64, f32)> {
        match &self.plot_freeze {
            Some(freeze) => &freeze.algo_active,
            None => &self.algo_trace_active,
        }
        .iter()
        .map(|p| (p.t_us, p.val))
        .collect()
    }
    pub fn algo_trace_version(&self) -> u64 {
        self.algo_trace_version
    }

    /// 暂存共享算法可设置变量 cfg[idx](0..7)，点击“保存到设备”后统一下发。
    pub fn set_algo_cfg(&mut self, idx: u8, val: u8) -> anyhow::Result<()> {
        if idx >= 8 {
            return Err(anyhow::anyhow!("算法可调变量索引非法: {}", idx));
        }
        let slot = idx as usize;
        let same_as_device = self.algo_cfg_valid[slot] && self.algo_cfg[slot] == val;
        self.drafts.set_algo_cfg(idx, val, same_as_device);
        // 草稿优先 getter 依赖版本号立即回显，不覆盖设备缓存以支持撤销恢复。
        self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        Ok(())
    }
    pub fn request_algo_cfg(&mut self, idx: u8) -> anyhow::Result<()> {
        if idx >= 8 {
            return Err(anyhow::anyhow!("算法可调变量索引非法: {}", idx));
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_algo_get_cfg(seq, idx))?;
        }
        Ok(())
    }
    pub fn algo_cfg(&self, idx: u8) -> u8 {
        let slot = idx as usize;
        self.drafts.algo_cfg(idx).unwrap_or_else(|| {
            if self.algo_cfg_valid.get(slot).copied().unwrap_or(false) {
                self.algo_cfg[slot]
            } else {
                self.algo_setting_decls()
                    .into_iter()
                    .find(|decl| decl.idx == idx)
                    .map_or(0, |decl| decl.default)
            }
        })
    }
    pub fn algo_cfg_version(&self) -> u64 {
        self.algo_cfg_version
    }

    // ------------------------------------------------------------------
    // 逐通道算法可设置变量(cfg_ch[8], ABI v2)
    // ------------------------------------------------------------------
    // ★与上面的 cfg[8] 完全对称★: 同一套"草稿 → 有序写队列 → ACK 归因"链路, 不新造第二条下发路。
    // 唯一差别是多一个通道维度, 以及下发时要按 (ch, idx) 聚合成一帧多条(288 项逐项发帧会堵死队列)。

    /// 某通道某槽的有效值。回退链: 草稿 → 设备缓存 → 声明默认值(逐通道声明, per_channel==true)。
    /// ★不会返回"未知"★ 与 `algo_cfg` 同口径: 声明里带 defval, 那就是设备上算法在用的默认,
    /// 显示它比显示 0 或空更接近事实。真正需要区分"未回读"的地方(批量面板)另有 `_algo_cfg_ch_known`。
    pub fn algo_cfg_ch(&self, ch: u8, idx: u8) -> u8 {
        if let Some(value) = self._algo_cfg_ch_known(ch, idx) {
            return value;
        }
        self.algo_setting_decls()
            .into_iter()
            .find(|decl| decl.per_channel && decl.idx == idx)
            .map_or(0, |decl| decl.default)
    }

    /// 只认"确定知道"的值: 草稿 → 设备缓存。都没有就返回 None(而不是拿声明默认值顶替)。
    /// 批量面板必须用它 —— 那一列写下去是要真发到 36 个通道的, 拿一个从未与设备核对过的默认值
    /// 冒充"源通道当前值"就是在批量覆盖用户没打算改的东西。
    pub(crate) fn _algo_cfg_ch_known(&self, ch: u8, idx: u8) -> Option<u8> {
        if let Some(value) = self.drafts.algo_cfg_ch(ch, idx) {
            return Some(value);
        }
        let (ch, idx) = (ch as usize, idx as usize);
        if self
            .algo_cfg_ch_valid
            .get(ch)
            .and_then(|row| row.get(idx))
            .copied()
            .unwrap_or(false)
        {
            return Some(self.algo_cfg_ch[ch][idx]);
        }
        None
    }

    /// 暂存逐通道算法配置 cfg_ch[ch][idx]，点击“保存到设备”后统一下发。
    pub fn set_algo_cfg_ch(&mut self, ch: u8, idx: u8, val: u8) -> anyhow::Result<()> {
        if (ch as usize) >= crate::proto::algo::ALGO_CHANNELS {
            return Err(anyhow::anyhow!("逐通道算法配置通道号非法: {}", ch));
        }
        if (idx as usize) >= crate::proto::algo::ALGO_CFG_CH_SLOTS {
            return Err(anyhow::anyhow!("逐通道算法配置索引非法: {}", idx));
        }
        let (c, i) = (ch as usize, idx as usize);
        let same_as_device = self.algo_cfg_ch_valid[c][i] && self.algo_cfg_ch[c][i] == val;
        self.drafts.set_algo_cfg_ch(ch, idx, val, same_as_device);
        // 草稿优先 getter 依赖版本号立即回显; 不覆盖设备缓存, 以支持撤销恢复。
        self.algo_cfg_ch_version = self.algo_cfg_ch_version.wrapping_add(1);
        Ok(())
    }

    pub fn algo_cfg_ch_version(&self) -> u64 {
        self.algo_cfg_ch_version
    }

    /// 全量回读逐通道算法配置(一帧带回 36×8)。窗口=1 由 `RequestAlgoCfgCh` 保证。
    pub fn request_algo_cfg_ch(&mut self) -> anyhow::Result<()> {
        let _ = self._submit_poll(
            RequestKind::RequestAlgoCfgCh,
            crate::proto::algo::encode_algo_get_cfg_ch(0),
        )?;
        Ok(())
    }

    /// ALGO_GET_CFG_CH 响应: 288 字节(通道主序)。截断响应只落已收到的完整通道, 其余保留旧缓存。
    pub(crate) fn _handle_algo_get_cfg_ch_response(&mut self, frame: &Frame) {
        let rows = crate::proto::algo::decode_algo_get_cfg_ch(&frame.payload);
        if rows.is_empty() {
            // 空响应的含义是"此刻取不到", 不是"设备上没有这些值" ⇒ 保留旧缓存, 只留证。
            log::warn!("ALGO_GET_CFG_CH 返回空 payload, 保留本地逐通道算法配置缓存");
            return;
        }
        let count = rows.len();
        for (ch, row) in rows.into_iter().enumerate() {
            self.algo_cfg_ch[ch] = row;
            self.algo_cfg_ch_valid[ch] = [true; crate::proto::algo::ALGO_CFG_CH_SLOTS];
        }
        self.algo_cfg_ch_version = self.algo_cfg_ch_version.wrapping_add(1);
        log::debug!("ALGO_GET_CFG_CH: 已落 {} 个通道 × 8 槽", count);
    }

    /// schema 解析用的 C 源: ★设备回读源优先★, 无设备源时退回本地最近一次编译源。
    /// 为什么不是编辑器文本: 面板与 report idx 轮询集合描述的是"设备上正在跑的算法",
    /// 编辑器里可能只是还没编译的草稿; 用草稿当 schema 会去轮询设备根本没有的 idx。
    pub fn algo_schema_source(&self) -> &str {
        if self.algo_device_src.trim().is_empty() {
            &self.algo_source
        } else {
            &self.algo_device_src
        }
    }

    pub fn algo_schema_version(&self) -> u64 {
        self.algo_schema_version
    }

    /// 当前算法源码的稳定 FNV-1a 指纹；仅用于本地 UI 元数据与配置文件匹配。
    pub fn algo_schema_fingerprint(&self) -> String {
        let mut hash = 0xcbf29ce484222325u64;
        for byte in self.algo_schema_source().as_bytes() {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
        format!("{:016x}", hash)
    }

    pub fn algo_metadata_version(&self) -> u64 {
        self.algo_metadata_version
    }

    /// 导出当前算法的有效 UI 元数据覆盖；返回值包含指纹，避免同 kind/index 跨算法串用。
    pub fn algo_metadata_overrides(&self) -> Vec<(String, u8, u8, String, String)> {
        let current = self.algo_schema_fingerprint();
        self.algo_metadata_overrides
            .iter()
            .filter(|((fingerprint, _, _), _)| fingerprint == &current)
            .map(|((fingerprint, kind, index), (alias, description))| {
                (
                    fingerprint.clone(),
                    *kind,
                    *index,
                    alias.clone(),
                    description.clone(),
                )
            })
            .collect()
    }

    /// 本地持久化使用全部算法的覆盖，切换算法后不得把其它算法的中文元数据从文件中抹掉。
    pub(crate) fn all_algo_metadata_overrides(&self) -> Vec<(String, u8, u8, String, String)> {
        self.algo_metadata_overrides
            .iter()
            .map(|((fingerprint, kind, index), (alias, description))| {
                (
                    fingerprint.clone(),
                    *kind,
                    *index,
                    alias.clone(),
                    description.clone(),
                )
            })
            .collect()
    }

    /// 读取当前算法 UI-only 覆盖；空覆盖按源码默认值处理。
    pub fn algo_metadata_override(&self, kind: u8, index: u8) -> Option<(String, String)> {
        self.algo_metadata_overrides
            .get(&(self.algo_schema_fingerprint(), kind, index))
            .cloned()
    }

    /// 从本地 jit_metadata.json 恢复覆盖；无效条目被忽略，不触发设备命令。
    pub(crate) fn load_algo_metadata_overrides(
        &mut self,
        entries: Vec<(String, u8, u8, String, String)>,
    ) {
        self.algo_metadata_overrides.clear();
        for (fingerprint, kind, index, alias, description) in entries {
            if fingerprint.len() == 16 && kind <= 1 {
                self.algo_metadata_overrides
                    .insert((fingerprint, kind, index), (alias, description));
            }
        }
        self.algo_metadata_version = self.algo_metadata_version.wrapping_add(1);
    }

    /// 写入 UI-only 覆盖，不触发任何设备命令；空 alias/description 会清除该项覆盖。
    pub fn set_algo_metadata_override(
        &mut self,
        kind: u8,
        index: u8,
        alias: String,
        description: String,
    ) -> anyhow::Result<()> {
        if kind > 1 {
            return Err(anyhow::anyhow!("算法元数据类型非法: {}", kind));
        }
        let alias = alias.trim().to_string();
        let description = description.trim().to_string();
        let key = (self.algo_schema_fingerprint(), kind, index);
        // ★"看过一眼"不能等于"设定了别名"★
        // UI 的别名/注释输入框在失焦时会无条件回调本函数并带上当前**显示**文本, 而显示文本本身
        // 就是 algo_decl_text 的结果(有 override 用 override, 否则用算法声明值)。若不比对就写入,
        // 用户只要点一下输入框, 算法声明值就被复制成一条 override 快照 —— 此后算法声明再更新
        // (或修掉了源码编码缺陷)界面也不会跟随, 永远显示那份旧快照。
        // 实测正是这条路把 mojibake 别名固化进 jit_metadata.json, 导致编码修复后仍显示乱码。
        let (decl_alias, decl_description) = self._algo_decl_declared_text(kind, index);
        if alias == decl_alias && description == decl_description {
            // 与算法声明完全一致 ⇒ 不需要 override; 顺手清掉同值的历史 override。
            self.algo_metadata_overrides.remove(&key);
        } else if alias.is_empty() && description.is_empty() {
            self.algo_metadata_overrides.remove(&key);
        } else {
            self.algo_metadata_overrides
                .insert(key, (alias, description));
        }
        self.algo_metadata_version = self.algo_metadata_version.wrapping_add(1);
        if let Err(error) = crate::settings_io::save_jit_metadata(self) {
            log::warn!("保存算法 UI 元数据失败: {}", error);
        }
        Ok(())
    }

    /// 界面展示用的 (别名, 简介)。回退链(前者缺失才用后者):
    ///   1) 用户覆盖(本地 jit_metadata.json, 可导入导出分享);
    ///   2) UI 内置本地化表(按变量英文 name 索引, 见 algo_i18n) —— 固件侧不存这些文本;
    ///   3) C 源声明里的 alias/description(自定义算法可自带, 通常为空);
    ///   4) 变量 name 本身(别名兜底, 保证界面不出现空标签)。
    pub fn algo_decl_text_named(
        &self,
        kind: u8,
        index: u8,
        name: &str,
        alias: &str,
        description: &str,
    ) -> (String, String) {
        if let Some(text) = self.algo_metadata_override(kind, index) {
            return text;
        }
        let (mut out_alias, mut out_desc) = crate::algo_i18n::text_for(kind, name)
            .unwrap_or_else(|| (String::new(), String::new()));
        if out_alias.is_empty() {
            out_alias = alias.to_string();
        }
        if out_desc.is_empty() {
            out_desc = description.to_string();
        }
        if out_alias.is_empty() {
            out_alias = name.to_string();
        }
        (out_alias, out_desc)
    }

    pub fn algo_decl_text(
        &self,
        kind: u8,
        index: u8,
        alias: &str,
        description: &str,
    ) -> (String, String) {
        self.algo_metadata_override(kind, index)
            .unwrap_or_else(|| (alias.to_string(), description.to_string()))
    }

    /// 算法**声明**里的别名/注释(不经 override)。用于判断一次提交是否只是把声明值原样回写。
    /// 该变量在**没有用户覆盖**时界面会显示的文本(内置本地化 + C 源声明的回退结果)。
    /// 用于判断一次提交是否只是把当前显示值原样回写 —— 那种情况不该产生 override。
    fn _algo_decl_declared_text(&self, kind: u8, index: u8) -> (String, String) {
        // 两种声明是不同类型(AlgoSettingDecl / AlgoReportDecl), 各自取出同一组字段。
        let (name, alias, description) = if kind == 1 {
            // ★index 里编着作用域★ 0..7 = 共享 cfg[idx], 8..15 = 逐通道 cfg_ch[idx-8]
            // (见 proto::algo::ALGO_CFG_CH_META_BASE 的说明: kind=1 只有一套 index 空间, 而设备
            // 侧有两套下标, 不错开就会两类共用同一条别名/注释)。按 idx 单独找必然找错一类。
            let base = crate::proto::algo::ALGO_CFG_CH_META_BASE;
            let (per_channel, idx) = if index >= base {
                (true, index - base)
            } else {
                (false, index)
            };
            self.algo_setting_decls()
                .into_iter()
                .find(|decl| decl.per_channel == per_channel && decl.idx == idx)
                .map(|decl| (decl.name, decl.alias, decl.description))
                .unwrap_or_default()
        } else {
            self.algo_report_decls()
                .into_iter()
                .find(|decl| decl.idx == index)
                .map(|decl| (decl.name, decl.alias, decl.description))
                .unwrap_or_default()
        };
        let (mut out_alias, mut out_desc) = crate::algo_i18n::text_for(kind, &name)
            .unwrap_or_else(|| (String::new(), String::new()));
        if out_alias.is_empty() {
            out_alias = alias;
        }
        if out_desc.is_empty() {
            out_desc = description;
        }
        if out_alias.is_empty() {
            out_alias = name;
        }
        (out_alias, out_desc)
    }

    fn _bump_algo_schema(&mut self) {
        self.algo_schema_version = self.algo_schema_version.wrapping_add(1);
        // 声明层二值性只随 schema 变，在这里一次算完缓存起来。曲线页每帧都要读它，
        // 而现算一次要把整段 C 源(~6KB)重新解析一遍。
        let mut binary = [None; crate::app_state::ALGO_REPORT_SLOTS];
        for decl in self.algo_report_decls() {
            if let Some(slot) = binary.get_mut(decl.idx as usize) {
                *slot = decl.declared_binary;
            }
        }
        self.algo_report_binary_decl = binary;
    }

    /// 某个 report 槽在**声明**里是否是二值量。`None` = 该槽没有声明，或声明是旧式
    /// `ALGO_REPORT(idx,name)` 而不带类型 —— 只有这种真未知才该退回按运行数据反推。
    ///
    /// 原实现一律用 `points_are_binary(观测数据)` 反推：算法声明了 `"u16"` 的槽在数据到齐前
    /// 恰好全是 0/1，于是"归一化幅度"输入框会先冒出来、等真实数据一到又消失（用户可见的闪跳），
    /// 且这个结论本来就该来自声明而不是等设备说话。
    pub fn algo_report_declared_binary(&self, idx: u8) -> Option<bool> {
        self.algo_report_binary_decl
            .get(idx as usize)
            .copied()
            .flatten()
    }

    /// 解析算法上报变量声明(ALGO_REPORT), 供 UI 建折线图例。源见 `algo_schema_source`。
    pub fn algo_report_decls(&self) -> Vec<crate::proto::algo::AlgoReportDecl> {
        crate::proto::algo::parse_algo_reports(self.algo_schema_source())
    }
    /// 解析算法可设置变量声明(ALGO_SETTING), 供 UI 建可调项列表。源见 `algo_schema_source`。
    pub fn algo_setting_decls(&self) -> Vec<crate::proto::algo::AlgoSettingDecl> {
        crate::proto::algo::parse_algo_settings(self.algo_schema_source())
    }

    /// ★算法运行值随帧入库★ 源为 FOCUS_DATA 的 FIELD_ALGO 块, 与 raw/bsln/diff 同帧同时间戳。
    ///
    /// 取代原先的 ALGO_GET_TRACE 轮询: 那是"core0 阻塞等 core1"的读类命令, 而设备的 vendor IN
    /// 只有单个响应槽; 独占流以上百帧/s 推送时它几乎抢不到窗口, 请求成片超时(既无响应也无 NAK),
    /// "当前值"永远空白。随帧上报后零额外往返, 且时间轴与曲线天然对齐。
    ///
    /// 设备侧每份快照只刷新一个 report 槽并轮转(为 4 槽各做一次 SPI 事务会把 core1 占满), 因此
    /// 同一帧内各槽新鲜度不同(最旧约 4 个快照周期)。这里仍按帧把 4 槽全部入库: 单槽在图上表现为
    /// 阶梯, 对"当前值"与趋势观察都够用, 且各系列节奏与遥测帧完全一致(缺口判据因此只剩一档)。
    pub(crate) fn _ingest_focus_algo(&mut self, ch: u8, t_us: u64, active: u8, report: [u16; 4]) {
        // 通道切换: 旧通道的值不能混进新通道的曲线。
        if self.algo_trace_channel != Some(ch) {
            self.algo_trace_channel = Some(ch);
            for buf in &mut self.algo_trace_report {
                buf.clear();
            }
            self.algo_trace_active.clear();
        }
        // 容量与遥测缓冲一致: 两者同帧同源、画在同一条时间轴上, 历史窗口不该不等长。
        const TRACE_CAP: usize = crate::app_state::TELEM_CAP;
        for (slot, buf) in self.algo_trace_report.iter_mut().enumerate() {
            if buf.len() >= TRACE_CAP {
                buf.pop_front();
            }
            buf.push_back(TracePoint {
                t_us,
                val: report[slot] as f32,
            });
        }
        if self.algo_trace_active.len() >= TRACE_CAP {
            self.algo_trace_active.pop_front();
        }
        self.algo_trace_active.push_back(TracePoint {
            t_us,
            val: if active != 0 { 1.0 } else { 0.0 },
        });
        self.algo_trace_version = self.algo_trace_version.wrapping_add(1);
    }

    pub(crate) fn _handle_algo_get_cfg_response(&mut self, frame: &Frame) {
        if let Some((idx, val)) = crate::proto::algo::decode_algo_get_cfg(&frame.payload) {
            if (idx as usize) < self.algo_cfg.len() {
                self.algo_cfg[idx as usize] = val;
                self.algo_cfg_valid[idx as usize] = true;
                self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
            }
        }
    }

    /// 简易 C→ASM 编译器: 把 C 源写临时文件, 用 arm-none-eabi-gcc(-mcpu=cortex-m0plus
    /// -mthumb -Os -ffreestanding -nostdlib) 编译 + objcopy 出裸 .text 二进制, 校验无外部符号/
    /// 无重定位/algo 在偏移 0/≤1024B, 成功则返回二进制供 algo_upload。方案 a: 封装现成工具链。
    /// abi_header_dir 提供 psoc_algo_abi.h 的 include 路径。
    pub fn compile_c_to_blob(&mut self, c_source: &str) -> anyhow::Result<Vec<u8>> {
        let out = Self::compile_blob(c_source, self.algo_caps_snapshot())?;
        let blob = out.blob.clone();
        self.apply_compiled(c_source, out);
        Ok(blob)
    }

    /// 纯编译(无 self): 只吃 C 源 + 一份容量快照、只吐产物, 全程不碰控制器状态。
    /// ★为什么必须是关联函数★: 编译要顺序阻塞跑 gcc/objcopy/nm/objdump 四个子进程, 首次还要
    /// 解压 18MB 内置工具链, 在 UI 线程里做会把界面冻死几秒。拆出来后可以丢进 std::thread,
    /// 而 `Rc<RefCell<AppController>>` 跨线程不安全 —— 后台只搬 String/Vec<u8> 这类纯数据,
    /// 产物回到 UI 线程再由 `apply_compiled` 写入状态。
    /// ★容量因此只能"传进来"★: 后台线程碰不到 `&self`, 又不能把闸门退回硬编码常量(那正是
    /// "固件扩容后上位机仍按旧数拒收"的来源), 所以由调用方在 spawn 之前 `algo_caps_snapshot()`。
    pub fn compile_blob(
        c_source: &str,
        caps: crate::app_state::AlgoCaps,
    ) -> anyhow::Result<CompiledAlgo> {
        use std::io::Write;
        // 0) C 源容量闸门: 设备只能存有限的"编译器有效内容"(= 滤注释后的 UTF-8 字节)。超了直接拒,
        //    绝不先编译再在上传时截断 —— 那会让设备上的映射表源与实际算法脱节且无法还原。
        let used = Self::algo_src_used(c_source);
        let cap = caps.src;
        if used > cap {
            return Err(anyhow::anyhow!(
                "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 请精简后再编译",
                used,
                cap
            ));
        }
        // 1) 定位工具链: 优先程序内置(随 exe 打包, 免各机环境差异), 解压失败再回退本机安装。
        let gcc_dir = match ensure_bundled_toolchain() {
            Ok(d) => d,
            Err(_) => find_toolchain_dir()?,
        };
        let gcc = gcc_dir.join("arm-none-eabi-gcc.exe");
        let objcopy = gcc_dir.join("arm-none-eabi-objcopy.exe");
        let nm = gcc_dir.join("arm-none-eabi-nm.exe");
        let objdump = gcc_dir.join("arm-none-eabi-objdump.exe");

        // 2) 临时目录 + 写源文件。ABI 头 include 路径指向工程 PSoC 目录。
        let tmp = std::env::temp_dir().join(format!("mai2algo_{}", std::process::id()));
        std::fs::create_dir_all(&tmp)?;
        let src = tmp.join("algo_user.c");
        let obj = tmp.join("algo_user.o");
        let bin = tmp.join("algo_user.bin");
        {
            let mut f = std::fs::File::create(&src)?;
            f.write_all(c_source.as_bytes())?;
        }
        // 内嵌 ABI 头写入临时目录, -I 指向它(不依赖本机 PSoC 工程路径)。
        std::fs::write(tmp.join("psoc_algo_abi.h"), ALGO_ABI_HEADER)?;

        // 3) 编译。
        let out = std::process::Command::new(&gcc)
            .args([
                "-mcpu=cortex-m0plus",
                "-mthumb",
                "-Os",
                "-ffreestanding",
                "-fno-jump-tables",
                "-fomit-frame-pointer",
                "-fno-common",
                "-nostdlib",
            ])
            .arg(format!("-I{}", tmp.display()))
            .arg("-c")
            .arg(&src)
            .arg("-o")
            .arg(&obj)
            .output()?;
        if !out.status.success() {
            let _ = std::fs::remove_dir_all(&tmp);
            return Err(anyhow::anyhow!(
                "编译失败:\n{}",
                String::from_utf8_lossy(&out.stderr)
            ));
        }
        // 4) objcopy 出 .text 裸二进制。
        let out2 = std::process::Command::new(&objcopy)
            .args(["-O", "binary", "-j", ".text"])
            .arg(&obj)
            .arg(&bin)
            .output()?;
        if !out2.status.success() {
            let _ = std::fs::remove_dir_all(&tmp);
            return Err(anyhow::anyhow!(
                "objcopy 失败:\n{}",
                String::from_utf8_lossy(&out2.stderr)
            ));
        }
        // 5) nm 校验无未定义(U)符号 + algo 在偏移 0(T 且地址 0)。
        let nm_out = std::process::Command::new(&nm).arg(&obj).output()?;
        let nm_txt = String::from_utf8_lossy(&nm_out.stdout);
        let mut has_undef = false;
        let mut algo_at_zero = false;
        for line in nm_txt.lines() {
            let cols: Vec<&str> = line.split_whitespace().collect();
            // 形如 "00000000 T algo" 或 "         U extsym"
            if cols.len() == 2 && cols[0] == "U" {
                has_undef = true;
            } else if cols.len() == 2 && cols[0].eq_ignore_ascii_case("U") {
                has_undef = true;
            } else if cols.len() == 3 {
                if cols[1] == "U" {
                    has_undef = true;
                }
                if cols[2] == "algo" && cols[1].eq_ignore_ascii_case("t") && cols[0] == "00000000" {
                    algo_at_zero = true;
                }
            }
        }
        // 反汇编 .o 的 .text(编译产物 ASM), 供 UI 子标签查看。清理临时目录前抓取。
        let asm = std::process::Command::new(&objdump)
            .args(["-d", "--no-show-raw-insn"])
            .arg(&obj)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).to_string())
            .unwrap_or_default();
        let data = std::fs::read(&bin).unwrap_or_default();
        let _ = std::fs::remove_dir_all(&tmp);
        if has_undef {
            return Err(anyhow::anyhow!(
                "算法含外部符号(禁止 libgcc/除法/64位): 见 nm 输出"
            ));
        }
        if !algo_at_zero {
            return Err(anyhow::anyhow!(
                "入口 algo 必须在偏移 0(检查是否首个函数/是否被内联到别处)"
            ));
        }
        // ★闸门用"单帧承载上限"而不是"槽容量"★ 槽是 4096, 但 ALGO_UPLOAD 帧要先放 len+crc16
        // 两个 u16, 于是 4093..4096 字节的产物编译得出来却传不进去(设备只回 "len invalid" NAK,
        // 现场看不出是少了 4 个字节的帧头)。这里把两个口径都写进报错文案, 免得再排一次。
        if data.is_empty() || data.len() > caps.upload_limit {
            return Err(anyhow::anyhow!(
                "产物大小非法: {} 字节(须 1..={} = 单帧承载上限; PSoC 槽本身为 {} 字节, \
                 两者相差的 4 字节是 ALGO_UPLOAD 的 len+crc16 帧内头)",
                data.len(),
                caps.upload_limit,
                caps.slot
            ));
        }
        // objdump 用制表符对齐, Slint 文本控件会把 \t 渲染成方块; 替换为空格避免乱码。
        Ok(CompiledAlgo {
            blob: data,
            asm: asm.replace('\t', " "),
        })
    }

    /// 把后台编译产物落到控制器状态(必须在 UI 线程调用): 保存 C 源与反汇编、留档本地源文件、记日志。
    pub fn apply_compiled(&mut self, c_source: &str, out: CompiledAlgo) -> usize {
        let len = out.blob.len();
        self.algo_source = c_source.to_string();
        self._bump_algo_schema();
        self.algo_asm = out.asm;
        self.algo_asm_version = self.algo_asm_version.wrapping_add(1);
        self.algo_compiled_blob = Some(out.blob);
        if let Ok(exe) = std::env::current_exe() {
            if let Some(dir) = exe.parent() {
                let _ = std::fs::write(dir.join("last_algo_source.c"), c_source);
            }
        }
        let slot = self.algo_slot_capacity();
        self.push_log(format!(
            "算法: 编译成功, ASM {} / {} 字节 ({}%){}",
            len,
            slot,
            (len * 100) / slot.max(1),
            if self.algo_caps_known() {
                ""
            } else {
                " · 容量待回读(暂用内置兜底值)"
            }
        ));
        len
    }

    pub fn algo_asm(&self) -> String {
        self.algo_asm.clone()
    }
    pub fn algo_source(&self) -> String {
        self.algo_source.clone()
    }
    pub fn algo_asm_version(&self) -> u64 {
        self.algo_asm_version
    }

    /// 编译并上传: compile_c_to_blob → algo_upload。
    pub fn compile_and_upload(&mut self, c_source: &str) -> anyhow::Result<()> {
        let blob = self.compile_c_to_blob(c_source)?;
        self.algo_upload(&blob)
    }

    // ------------------------------------------------------------------
    // 算法容量: ★一律取自设备★
    // ------------------------------------------------------------------
    // 这一组原先是**关联函数**(不带 self), 直接返回编译期常量。那等于把"这台设备的槽有多大"
    // 写死在上位机里, 而槽/单帧上限/C 源容量/堆容量四个数分散在 PSoC 与 RP 两版固件的常量里 ——
    // 上位机可以连到任意一版。硬编码的后果有两种, 都很难查:
    //   固件扩容了而上位机没跟 → 编译出的算法明明装得下, 却被上位机自己的闸门拒掉;
    //   固件缩容了而上位机没跟 → 上位机放行, 设备静默 NAK 或截断, 表现为"上传成功但跑的是旧算法"。
    // 设备现在在 ALGO_GET_INFO 里如实回报这四个容量, 故改成实例方法: 有设备值用设备值,
    // 没有(未连接/旧固件)才退回兜底常量, 且 `algo_caps_known()` 让 UI 能把两种情况区分显示。

    /// PSoC 可执行算法槽容量(字节)。**只用于占用百分比显示**, 不是编译/上传闸门。
    pub fn algo_slot_capacity(&self) -> usize {
        self.algo_caps
            .map(|caps| caps.slot)
            .unwrap_or(crate::proto::algo::ALGO_SLOT_FALLBACK)
    }

    /// 单次上传可承载的算法字节上限(设备 = 单帧 payload − 4 字节帧内头 len+crc16)。
    /// ★与 `algo_slot_capacity()` 差 4 字节, 这 4 字节必须分得清★: 编译出 4093..4096 字节的算法
    /// 在槽里放得下, 却会被设备以 "len invalid" 拒收 —— 闸门用本值, 进度条分母用槽容量。
    pub fn algo_upload_limit(&self) -> usize {
        self.algo_caps
            .map(|caps| caps.upload_limit)
            .unwrap_or(crate::proto::algo::ALGO_UPLOAD_FALLBACK)
    }

    /// 算法共享堆容量(字节)。只用于占用百分比显示。
    pub fn algo_heap_capacity(&self) -> usize {
        self.algo_caps
            .map(|caps| caps.heap)
            .unwrap_or(crate::proto::algo::ALGO_HEAP_FALLBACK)
    }

    /// 容量是否已从设备回读到。false ⇒ 上面几个方法返回的是兜底值, UI 必须显示"待回读"而非真值。
    pub fn algo_caps_known(&self) -> bool {
        self.algo_caps.is_some()
    }

    /// 设备自报"PSoC 槽容量与 RP 侧容量常量不一致"。为真时必须显著告警: 两级容量对不上时,
    /// 上传会被一侧放行、另一侧静默截断, 现象是"上传成功但算法跑飞", 两侧日志都看不出原因。
    pub fn algo_caps_mismatch(&self) -> bool {
        self.algo_caps.map(|caps| caps.mismatch).unwrap_or(false)
    }

    /// 取一份容量快照。★后台编译线程唯一的容量来源★: 那里拿不到 `&self`, 必须在 spawn 之前
    /// 由 UI 线程取好带过去(见 `compile_blob` 的签名与 main.rs 的 spawn_algo_compile)。
    /// 未回读到设备容量时返回全兜底值的快照 —— 编译不该因为"还没连上设备"而不能进行。
    pub fn algo_caps_snapshot(&self) -> crate::app_state::AlgoCaps {
        self.algo_caps
            .unwrap_or(crate::app_state::AlgoCaps {
                slot: crate::proto::algo::ALGO_SLOT_FALLBACK,
                upload_limit: crate::proto::algo::ALGO_UPLOAD_FALLBACK,
                src: crate::proto::algo::ALGO_SRC_FALLBACK,
                src_chunk: crate::proto::algo::ALGO_SRC_CHUNK,
                heap: crate::proto::algo::ALGO_HEAP_FALLBACK,
                from_psoc: false,
                mismatch: false,
            })
    }

    /// 分片粒度(字节)。设备值优先, 兜底 `ALGO_SRC_CHUNK` —— 步长必须与设备一致,
    /// 否则设备按自己的粒度校验 offset 会整片 NAK, 而报错文案只说"分片非法"。
    pub(crate) fn _algo_src_chunk(&self) -> usize {
        self.algo_caps
            .map(|caps| caps.src_chunk)
            .unwrap_or(crate::proto::algo::ALGO_SRC_CHUNK)
            .max(1)
    }

    /// 把设备回报的容量组落进缓存, 并在两级容量不一致时告警(只在结论变化时写一行, 不刷屏)。
    pub(crate) fn _absorb_algo_caps(&mut self, info: crate::proto::algo::AlgoInfo) {
        if !info.caps_known {
            return; // 旧固件不回报 ⇒ 保持 None, UI 显示"待回读", 绝不拿兜底值冒充设备真值。
        }
        let caps = crate::app_state::AlgoCaps {
            // 设备回 0 只可能是它自己也没算出来 ⇒ 退回兜底值, 免得进度条分母为 0、闸门拒一切。
            slot: if info.slot_capacity == 0 {
                crate::proto::algo::ALGO_SLOT_FALLBACK
            } else {
                info.slot_capacity as usize
            },
            upload_limit: if info.upload_limit == 0 {
                crate::proto::algo::ALGO_UPLOAD_FALLBACK
            } else {
                info.upload_limit as usize
            },
            src: if info.src_capacity == 0 {
                crate::proto::algo::ALGO_SRC_FALLBACK
            } else {
                info.src_capacity as usize
            },
            src_chunk: if info.src_chunk == 0 {
                crate::proto::algo::ALGO_SRC_CHUNK
            } else {
                info.src_chunk as usize
            },
            heap: if info.heap_size == 0 {
                crate::proto::algo::ALGO_HEAP_FALLBACK
            } else {
                info.heap_size as usize
            },
            from_psoc: info.slot_capacity_from_psoc,
            mismatch: info.capacity_mismatch,
        };
        if self.algo_caps == Some(caps) {
            return; // 每次 GET_INFO 都会走到这里, 容量却是不变量 —— 没变就别刷日志。
        }
        self.algo_caps = Some(caps);
        self.algo_version = self.algo_version.wrapping_add(1);
        if caps.mismatch {
            self.push_log_warn(format!(
                "算法: {}",
                Self::algo_caps_mismatch_text(caps.slot, caps.upload_limit)
            ));
        } else {
            self.push_log(format!(
                "算法容量(设备回报): 槽 {}B{} · 单帧上传上限 {}B · C 源 {}B(分片 {}B) · 堆 {}B",
                caps.slot,
                if caps.from_psoc {
                    "(PSoC 自报)"
                } else {
                    "(RP 兜底)"
                },
                caps.upload_limit,
                caps.src,
                caps.src_chunk,
                caps.heap
            ));
        }
    }

    /// 容量不一致的统一告警文案(日志与算法页状态行同一句, 免得两处口径漂移)。
    pub fn algo_caps_mismatch_text(slot: usize, upload_limit: usize) -> String {
        format!(
            "设备两级容量常量不一致(PSoC 槽 {} / RP 上限 {}), 上传可能静默跑飞, \
             请重新烧写匹配的固件",
            slot, upload_limit
        )
    }

    /// 仅编译(不上传, 同步版): 产出 ASM 二进制并缓存, 返回其字节数。
    /// compile_blob 已在产物 >容量 时报错, 故成功返回的长度必然 ≤ 容量(不会截断)。
    /// UI 走的是后台线程 + `apply_compiled` 那条路(见 main.rs 算法编译任务), 本函数留给无头场景。
    pub fn compile_only(&mut self, c_source: &str) -> anyhow::Result<usize> {
        let out = Self::compile_blob(c_source, self.algo_caps_snapshot())?;
        Ok(self.apply_compiled(c_source, out))
    }

    /// 上传最近一次成功编译的 ASM 产物, 并把对应 C 源(滤注释后)作为"映射表"存到设备,
    /// 供后续回读还原可编辑 C。未编译则报错(强制"先编译后上传")。
    pub fn upload_compiled(&mut self) -> anyhow::Result<()> {
        let blob = self
            .algo_compiled_blob
            .clone()
            .ok_or_else(|| anyhow::anyhow!("尚未编译: 请先点“编译”生成 ASM 再上传"))?;
        // 发送前再把 C 源容量闸门过一遍: 走到这里的产物可能来自更早一次编译, 而编辑器/模板
        // 随后被换过, 不能靠"编译时查过了"就免检。
        let src = Self::strip_c_comments(&self.algo_source);
        let cap = self.algo_src_capacity();
        if src.len() > cap {
            return Err(anyhow::anyhow!(
                "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 未上传",
                src.len(),
                cap
            ));
        }
        self.algo_upload(&blob)?;
        // 随算法上传其 C 源(滤注释)到设备映射表; 失败不阻断上传主流程。
        let _ = self.send_algo_src(&src);
        // 设备映射表刚被这次上传覆盖 → 本地那份缓存同步跟上, 否则 schema(设备源优先)会继续
        // 按上一版算法解析, 面板与 report idx 轮询集合就跟设备上真正跑的算法脱节了。
        // 只 bump schema/cfg/trace 版本: algo_device_src_version 门控的是"回读→载入编辑器",
        // 不该由一次上传去触发编辑器回填。
        self.algo_device_src = src;
        self._bump_algo_schema();
        self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        self.algo_trace_version = self.algo_trace_version.wrapping_add(1);
        Ok(())
    }

    /// 最近一次编译产物的字节数(未编译=0), 供 UI 进度条。
    pub fn algo_compiled_len(&self) -> usize {
        self.algo_compiled_blob
            .as_ref()
            .map(|b| b.len())
            .unwrap_or(0)
    }

    /// C 源容量(字节)。编译器实际看到的内容 = 滤注释后的 UTF-8 字节数, 必须 ≤ 此值。
    /// 设备值优先(见上面容量组的说明), 未回读到才用兜底常量。
    pub fn algo_src_capacity(&self) -> usize {
        self.algo_caps
            .map(|caps| caps.src)
            .unwrap_or(crate::proto::algo::ALGO_SRC_FALLBACK)
    }

    /// 某段 C 源"编译器有效内容"的字节数: 复用唯一的注释过滤实现 `strip_c_comments`,
    /// 不另造一套过滤逻辑(两套一定会漂移, 于是界面显示的占用和真正上传的长度对不上)。
    pub fn algo_src_used(src: &str) -> usize {
        Self::strip_c_comments(src).as_bytes().len()
    }

    /// 是否仍有算法 C 源分片上传或回读在途，供无头工具等待完整传输。
    pub fn algo_src_transfer_pending(&self) -> bool {
        self.algo_src_tx.is_some() || self.algo_src_rx.is_some()
    }

    /// 把算法 C 源存到设备映射表(ALGO_SET_SRC)，按唯一容量口径过滤注释后分片发送。
    /// 超上限直接报错 —— 截断会把一份编译不过的残源固化到设备上。
    pub fn send_algo_src(&mut self, src: &str) -> anyhow::Result<()> {
        let bytes = Self::strip_c_comments(src).into_bytes();
        let cap = self.algo_src_capacity();
        if bytes.len() > cap {
            return Err(anyhow::anyhow!(
                "算法 C 源 {} 字节, 超出设备存储上限 {} 字节",
                bytes.len(),
                cap
            ));
        }
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法上传算法 C 源"));
        }
        self.algo_src_tx = Some(AlgoSrcTx {
            bytes,
            acked: 0,
            seq: None,
            waited: 0,
        });
        // 已有普通写帧时只登记待传状态，等队列排空后再发首片；开始后每片仍严格 ACK 驱动。
        if self.cfg_tx_pending() == 0 {
            self._algo_src_send_next()?;
        }
        Ok(())
    }

    /// 发出"下一片"(含 total=0 的空源片)。窗口=1: 上一片没回执就不发下一片。
    fn _algo_src_send_next(&mut self) -> anyhow::Result<()> {
        let Some(tx) = self.algo_src_tx.as_ref() else {
            return Ok(());
        };
        let total = tx.bytes.len();
        let offset = tx.acked;
        // ★分片步长取设备回报值★ 设备按自己的粒度校验 offset 与片长, 步长对不上就整片 NAK,
        // 而报错只说"分片非法" —— 现场看不出是主机按 2048 发、设备按别的数收。
        let end = (offset + self._algo_src_chunk()).min(total);
        let chunk = tx.bytes[offset..end].to_vec();
        let seq = self.next_seq();
        let frame = crate::proto::algo::encode_algo_set_src_chunk(seq, offset, total, &chunk);
        let sent = self
            .io
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("设备已断开, 无法继续上传算法 C 源"))?
            .send(frame);
        match sent {
            Ok(()) => {
                if let Some(tx) = self.algo_src_tx.as_mut() {
                    tx.seq = Some(seq);
                    tx.waited = 0;
                }
                Ok(())
            }
            Err(e) => {
                self.algo_src_tx = None;
                self._record_cfg_tx_failure(
                    HostCmd::AlgoSetSrc as u8,
                    seq,
                    format!("本地发送失败: {}", e),
                );
                Err(e)
            }
        }
    }

    /// 收到 ACK/NAK 时推进/终止分片上传。NAK 即放弃并记录可查询失败状态。
    pub(crate) fn _algo_src_note_reply(&mut self, seq: u8, ok: bool) {
        let Some((expected_seq, at)) = self.algo_src_tx.as_ref().map(|tx| (tx.seq, tx.acked))
        else {
            return;
        };
        if expected_seq != Some(seq) {
            return;
        }
        if !ok {
            self.algo_src_tx = None;
            self._record_cfg_tx_failure(
                HostCmd::AlgoSetSrc as u8,
                seq,
                format!("设备拒绝分片 offset={}", at),
            );
            self.push_log_error(format!("算法 C 源上传被设备拒绝(offset={}), 已放弃", at));
            return;
        }
        let step = self._algo_src_chunk();
        let total = {
            let tx = self.algo_src_tx.as_mut().expect("刚判过 Some");
            tx.acked = (tx.acked + step).min(tx.bytes.len());
            tx.bytes.len()
        };
        if self.algo_src_tx.as_ref().map(|tx| tx.acked) == Some(total) {
            self.algo_src_tx = None; // 末片已 ACK: 设备侧才让新源生效
            return;
        }
        if let Err(e) = self._algo_src_send_next() {
            self.push_log_error(format!("算法 C 源分片发送失败: {}", e));
        }
    }

    /// 每 tick 调一次: 待普通写队列排空后启动首片，并对已发送片做超时兜底。
    pub(crate) fn _pump_algo_src_tx(&mut self) {
        const ALGO_SRC_TIMEOUT_TICKS: u32 = 125; // ~2s @16ms/tick, 与保存队列同口径
        let Some((seq, at)) = self.algo_src_tx.as_ref().map(|tx| (tx.seq, tx.acked)) else {
            return;
        };
        let Some(seq) = seq else {
            if self.cfg_tx_pending() == 0 {
                if let Err(e) = self._algo_src_send_next() {
                    self.push_log_error(format!("算法 C 源首片发送失败: {}", e));
                }
            }
            return;
        };
        let timed_out = {
            let tx = self.algo_src_tx.as_mut().expect("刚判过 Some");
            tx.waited += 1;
            tx.waited >= ALGO_SRC_TIMEOUT_TICKS
        };
        if !timed_out {
            return;
        }
        self.algo_src_tx = None;
        self._record_cfg_tx_failure(
            HostCmd::AlgoSetSrc as u8,
            seq,
            format!("offset={} 超过 2s 无 ACK/NAK", at),
        );
        self.push_log_error(format!(
            "算法 C 源上传超时(offset={} 无回执), 已放弃本次上传",
            at
        ));
    }

    /// 每 tick 调一次: 回读请求也必须等待当前片响应，丢帧时清理状态而不是永久卡住。
    pub(crate) fn _pump_algo_src_rx(&mut self) {
        const ALGO_SRC_TIMEOUT_TICKS: u32 = 125;
        let Some(rx) = self.algo_src_rx.as_mut() else {
            return;
        };
        rx.waited += 1;
        if rx.waited < ALGO_SRC_TIMEOUT_TICKS {
            return;
        }
        let offset = rx.buf.len();
        self.algo_src_rx = None;
        self.push_log_error(format!(
            "算法 C 源回读超时(offset={} 无响应), 已放弃本次回读",
            offset
        ));
    }

    /// 标记"本次回读由用户显式发起"(读取信息按钮 / 连接后首次同步): 回读到的 C 源与 ASM
    /// 都应覆盖编辑器。★两路各一个标记★: C 源与机器码是两条独立的响应, 到达时机不同;
    /// 用一个共享标记会被先到的那一路吃掉, 另一路就又退回"保护草稿"而不覆盖。
    pub fn mark_algo_explicit_readback(&mut self) {
        self.algo_explicit_readback_src = true;
        self.algo_explicit_readback_asm = true;
    }

    /// 取走 C 源的"显式回读"标记(一次性)。
    pub fn take_algo_explicit_readback_src(&mut self) -> bool {
        let armed = self.algo_explicit_readback_src;
        self.algo_explicit_readback_src = false;
        armed
    }

    /// 取走 ASM 的"显式回读"标记(一次性)。
    pub fn take_algo_explicit_readback_asm(&mut self) -> bool {
        let armed = self.algo_explicit_readback_asm;
        self.algo_explicit_readback_asm = false;
        armed
    }

    /// 请求回读设备映射表里的算法 C 源(ALGO_GET_SRC), 从第 0 片开始。
    pub fn request_algo_src(&mut self) -> anyhow::Result<()> {
        self.algo_src_rx = None;
        self._algo_src_request_at(0)
    }

    fn _algo_src_request_at(&mut self, offset: usize) -> anyhow::Result<()> {
        // ★续读前必须先注销上一片的在途登记★
        // `_submit_poll` 用 `pending_registry.contains_kind()` 实现窗口=1 去重, 而分片回读的
        // **每一片都是同一个 RequestKind::RequestAlgoSrc**。若上一片还挂在 registry 里, 本片就会
        // 被判成"重复请求"而返回 seq=0, 随后 `if seq == 0` 把 algo_src_rx 清掉 —— 整个回读被
        // 静默放弃, 没有任何报错。
        // 后果: C 源只要超过一个分片(ALGO_SRC_CHUNK=2048B)就永远读不回来。实测设备存了 6193B,
        // 于是每次连接/点"读取信息"都拿不到源, 界面报"设备未存该算法 C 源"并退回内置示例算法,
        // 进而 schema 为空 → 不轮询追踪 → 算法变量"暂无运行值"、精调页"未声明 ALGO_REPORT"。
        // 重复 confirm 是安全的(幂等), 故这里无条件注销, 不依赖 handle_frame 的先后次序。
        if offset > 0 {
            if let Some(previous_seq) = self.algo_src_rx.as_ref().map(|rx| rx.seq) {
                self.pending_registry.confirm(previous_seq);
            }
        }
        let seq = self._submit_poll(
            RequestKind::RequestAlgoSrc,
            crate::proto::algo::encode_algo_get_src(0, offset),
        )?;
        if offset == 0 {
            self.algo_src_rx = Some(AlgoSrcRx {
                total: 0,
                buf: Vec::new(),
                seq,
                waited: 0,
            });
        } else {
            let rx = self
                .algo_src_rx
                .as_mut()
                .ok_or_else(|| anyhow::anyhow!("算法 C 源回读状态丢失(offset={})", offset))?;
            if rx.buf.len() != offset {
                return Err(anyhow::anyhow!(
                    "算法 C 源续读偏移错误(期望 {}, 请求 {})",
                    rx.buf.len(),
                    offset
                ));
            }
            rx.seq = seq;
            rx.waited = 0;
        }
        if seq == 0 {
            // ★这里过去是静默 return★: `_submit_poll` 返回 0 只有两种原因(未连接 / 同 kind 仍在途),
            // 两者都会让整轮回读无声消失 —— 界面只剩"未声明 ALGO_REPORT", 日志里一行线索都没有,
            // 与"回读成功但设备真没声明"完全无法区分。必须留痕。
            self.algo_src_rx = None;
            self.push_log_warn(format!(
                "ALGO_GET_SRC 未能提交(offset={}, 同类请求仍在途或未连接), 本次回读放弃",
                offset
            ));
            return Ok(());
        }
        if let Some(rx) = self.algo_src_rx.as_mut() {
            rx.seq = seq;
        }
        Ok(())
    }

    /// ALGO_GET_SRC 响应: [total, offset, chunk]。聚合到齐且 UTF-8 有效才替换缓存/版本。
    pub(crate) fn _handle_algo_src_chunk(&mut self, frame: &Frame) {
        let Some((total, offset, chunk)) =
            crate::proto::algo::decode_algo_src_chunk(&frame.payload)
        else {
            self.algo_src_rx = None;
            self.push_log_error("ALGO_GET_SRC 响应过短, 已放弃本次回读".to_string());
            return;
        };
        if total > self.algo_src_capacity() {
            self.algo_src_rx = None;
            self.push_log_error(format!(
                "ALGO_GET_SRC 声明长度 {} 超上限, 已放弃本次回读",
                total
            ));
            return;
        }
        let Some(expected_seq) = self.algo_src_rx.as_ref().map(|rx| rx.seq) else {
            self.push_log_error("ALGO_GET_SRC 收到无在途请求的响应, 已忽略".to_string());
            return;
        };
        if expected_seq != frame.seq {
            self.push_log_error(format!(
                "ALGO_GET_SRC 响应序号错误(期望 {}, 实收 {}), 已忽略",
                expected_seq, frame.seq
            ));
            return;
        }
        let mut next = None;
        let mut complete = false;
        // 片长上限同样取设备回报的粒度(先算好: 下面的块要可变借用 algo_src_rx, 借用期内不能再碰 self)。
        let chunk_limit = self._algo_src_chunk();
        let error = {
            let rx = self.algo_src_rx.as_mut().expect("刚判过 Some");
            if offset == 0 && rx.buf.is_empty() && rx.total == 0 {
                rx.total = total;
                rx.buf = Vec::with_capacity(total);
            }
            if rx.total != total || rx.buf.len() != offset {
                Some(format!(
                    "ALGO_GET_SRC 分片错位(期望 offset={} total={}, 实收 offset={} total={})",
                    rx.buf.len(),
                    rx.total,
                    offset,
                    total
                ))
            } else if chunk.len() > chunk_limit {
                Some(format!("ALGO_GET_SRC 分片长度 {} 超过上限", chunk.len()))
            } else if chunk.len() > total.saturating_sub(rx.buf.len()) {
                Some(format!(
                    "ALGO_GET_SRC 总长度错误(已收 {} + 本片 {} > {})",
                    rx.buf.len(),
                    chunk.len(),
                    total
                ))
            } else {
                rx.buf.extend_from_slice(chunk);
                if rx.buf.len() < total {
                    if chunk.is_empty() {
                        Some(format!("ALGO_GET_SRC 在 offset={} 收到空中间片", offset))
                    } else {
                        next = Some(rx.buf.len());
                        None
                    }
                } else {
                    complete = true;
                    None
                }
            }
        };
        if let Some(error) = error {
            self.algo_src_rx = None;
            self.push_log_error(format!("{}, 已放弃本次回读", error));
            return;
        }
        if let Some(offset) = next {
            if let Err(e) = self._algo_src_request_at(offset) {
                self.algo_src_rx = None;
                self.push_log_error(format!("ALGO_GET_SRC 续请求失败: {}", e));
            }
            return;
        }
        if !complete {
            return;
        }
        let rx = self.algo_src_rx.take().expect("刚判过 Some");
        // ★total=0 不是"读到了一份空源"★
        // 设备在 NvStore 提交(擦写 ALGO_SRC 区)的窗口里会把该区长度短暂报成 0; 实测在
        // `--mai2-load --heavy` 之后连一次 UI 恰好撞上过一次(随后再连即恢复 6193B)。
        // 若照旧当成功完成, 就会用空串覆盖设备源缓存 ⇒ schema 退回编辑器文本、算法变量声明
        // 与图例整片消失, 而日志只说了句"回读完成 0 字节", 与"设备真的没存源"无法区分。
        // 空源本身也没有任何可用信息, 保留旧缓存严格更优: 下一次回读会自然纠正。
        if rx.buf.is_empty() {
            self.push_log_warn(
                "ALGO_GET_SRC 设备回报长度 0(多为 NvStore 正在提交), 已保留上一份设备源缓存"
                    .to_string(),
            );
            return;
        }
        let source = match String::from_utf8(rx.buf) {
            Ok(source) => source,
            Err(e) => {
                self.push_log_error(format!(
                    "ALGO_GET_SRC 完整内容不是有效 UTF-8: {}, 已保留旧缓存",
                    e
                ));
                return;
            }
        };
        // 回读到齐即报一行: 长度 + 解析出的声明数。"设备存了源但一条 META 都没解析出来"与
        // "源根本没读回来"在界面上是同一句提示, 只有这行日志能把两者分开。
        let report_count = crate::proto::algo::parse_algo_reports(&source).len();
        let setting_count = crate::proto::algo::parse_algo_settings(&source).len();
        self.push_log(format!(
            "ALGO_GET_SRC 回读完成: {} 字节, 解析 ALGO_REPORT×{} / ALGO_SETTING×{}",
            source.len(),
            report_count,
            setting_count
        ));
        self.algo_device_src = source;
        self.algo_device_src_version = self.algo_device_src_version.wrapping_add(1);
        // ★不再把设备源灌进 algo_source★: schema 一律走 `algo_schema_source()`(设备源优先),
        // 编辑器文本与 schema 彻底解耦 —— 连接即按设备上真正跑着的算法填面板/report idx,
        // 不必先下发一次; 同时用户手上正在改的编辑器内容永远不会被回读覆盖。
        self._bump_algo_schema();
        // 同步让 cfg 行和 report 行重建；后者复用既有 trace 版本门控以立即显示声明。
        self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        self.algo_trace_version = self.algo_trace_version.wrapping_add(1);
    }

    /// 请求回读设备算法 ASM 机器码(ALGO_GET_CODE), 用于无本地编译产物时查看真实机器码。
    pub fn request_algo_code(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_algo_get_code(seq))?;
        }
        Ok(())
    }

    /// 设备回读的算法 ASM 机器码 hex dump 文本(空表示未回读)。
    /// 设备回读的原始算法机器码，可用于同 blob 重传与 CRC 对账。
    pub fn algo_device_code(&self) -> &[u8] {
        &self.algo_device_code
    }
    pub fn algo_device_code_hex(&self) -> &str {
        &self.algo_device_code_hex
    }
    pub fn algo_device_code_version(&self) -> u64 {
        self.algo_device_code_version
    }

    /// 字节序列 → 每行 16 字节的 hex dump(offset: bytes)文本。
    pub(crate) fn _hex_dump(data: &[u8]) -> String {
        if data.is_empty() {
            return String::new();
        }
        let mut out = String::with_capacity(data.len() * 4);
        for (row, chunk) in data.chunks(16).enumerate() {
            out.push_str(&format!("{:04X}: ", row * 16));
            for b in chunk {
                out.push_str(&format!("{:02X} ", b));
            }
            out.push('\n');
        }
        out
    }

    /// 设备回读的算法 C 源(映射表), 空表示设备无存源。
    pub fn algo_device_src(&self) -> &str {
        &self.algo_device_src
    }
    pub fn algo_device_src_version(&self) -> u64 {
        self.algo_device_src_version
    }

    /// 去除 C 注释(// 行注释与 /* */ 块注释), 保留字符串字面量内容与换行结构。
    /// 用于上传时精简"映射表"源, 不改变代码语义(变量名不必与原始一致)。
    ///
    /// ★必须按字节累积再一次性解码★ 本函数原先用 `out.push(c as char)` 逐字节转 char,
    /// 那会把每个 UTF-8 多字节序列(中文别名/注释)拆成若干 U+0080..U+00FF 的独立字符,
    /// 即经典 mojibake(“别名”变成 “Ã¡Â¨Â…”)。而且损坏发生在**上传前**, 于是坏字节被写进
    /// 设备 flash, 回读解析出的元数据别名/注释也全是乱码。
    /// 逐字节扫描本身是安全的: C 的注释与字符串定界符都是 ASCII, 且 UTF-8 自同步 ——
    /// 多字节序列的每个字节都 >= 0x80, 不可能与 `/ * " '` 等定界符混淆。
    pub fn strip_c_comments(src: &str) -> String {
        let bytes = src.as_bytes();
        let mut out: Vec<u8> = Vec::with_capacity(src.len());
        let mut i = 0usize;
        // 状态: 0=普通 1=行注释 2=块注释 3=字符串"" 4=字符''
        let mut state = 0u8;
        while i < bytes.len() {
            let c = bytes[i];
            let n = if i + 1 < bytes.len() { bytes[i + 1] } else { 0 };
            match state {
                0 => {
                    if c == b'/' && n == b'/' {
                        state = 1;
                        i += 2;
                        continue;
                    }
                    if c == b'/' && n == b'*' {
                        state = 2;
                        i += 2;
                        continue;
                    }
                    if c == b'"' {
                        state = 3;
                        out.push(b'"');
                        i += 1;
                        continue;
                    }
                    if c == b'\'' {
                        state = 4;
                        out.push(b'\'');
                        i += 1;
                        continue;
                    }
                    out.push(c);
                }
                1 => {
                    if c == b'\n' {
                        state = 0;
                        out.push(b'\n');
                    }
                }
                2 => {
                    if c == b'*' && n == b'/' {
                        state = 0;
                        i += 2;
                        continue;
                    }
                    if c == b'\n' {
                        out.push(b'\n');
                    } // 保留行结构便于阅读
                }
                3 => {
                    out.push(c);
                    if c == b'\\' && n != 0 {
                        out.push(n);
                        i += 2;
                        continue;
                    }
                    if c == b'"' {
                        state = 0;
                    }
                }
                4 => {
                    out.push(c);
                    if c == b'\\' && n != 0 {
                        out.push(n);
                        i += 2;
                        continue;
                    }
                    if c == b'\'' {
                        state = 0;
                    }
                }
                _ => {}
            }
            i += 1;
        }
        // 字节流在此一次性按 UTF-8 解码: 多字节序列始终完整保留, 中文别名/注释不再被打散。
        let stripped = String::from_utf8_lossy(&out);
        // 压缩连续空行(注释删除后常留大量空行)。
        let mut cleaned = String::with_capacity(stripped.len());
        let mut blank_run = 0u32;
        for line in stripped.lines() {
            if line.trim().is_empty() {
                blank_run += 1;
                if blank_run <= 1 {
                    cleaned.push('\n');
                }
            } else {
                blank_run = 0;
                cleaned.push_str(line.trim_end());
                cleaned.push('\n');
            }
        }
        cleaned
    }

    pub(crate) fn _handle_algo_info_response(&mut self, frame: &Frame) {
        if let Some(info) = crate::proto::algo::decode_algo_info(&frame.payload) {
            self.algo_info = Some(info);
            self.algo_version = self.algo_version.wrapping_add(1);
            // 容量组先落缓存再宣判上传: 终态文案里的"槽/上传上限"要用这一拍的设备口径,
            // 否则同一条日志里会出现"按兜底值算的百分比"和"设备真值的长度"两种尺子。
            self._absorb_algo_caps(info);
            self._settle_algo_upload(info);
        }
    }

    /// 用设备真值给上一次上传下终态结论。
    ///
    /// ★为什么必须有这一步★ ACK 只表示设备"已受理并下发"; 固件的 commit 在 core1 异步执行
    /// (整段分页 + PSoC 校验最坏 ~700ms)。若在 ACK 后就不再追问, UI 会永久停在中间态,
    /// "下发到底成没成"完全不可知。
    ///
    /// ★判据必须落在 PSoC 真值上, 不能用 store 的 len/crc16★
    /// `info.len`/`info.crc16` 取自 **RP2040 存储**, 它们相符只证明"主机端把字节存下了";
    /// 而 `psoc_valid` 是"槽里有一份能跑的算法", 无法区分"新算法装上了"与"旧算法还在、
    /// 长度恰好相同"。两者组合起来仍会把一次失败的上传报成"已装上并正在运行" —— 这正是用户
    /// 报的 bug。扩展响应给了 `psoc_len`/`psoc_crc16`(commit 时按槽内实际内容算出), 那才是
    /// "装上了什么"的唯一直接证据。
    fn _settle_algo_upload(&mut self, info: crate::proto::algo::AlgoInfo) {
        let Some((expect_len, expect_crc)) = self.algo_upload_expect else {
            return;
        };
        // 隔离态优先宣判: 此刻 PSoC 跑的是原生 CapSense 判定, 无论 len/crc 怎么对都不能叫"正在运行"。
        if info.extended && info.quarantined {
            self.algo_upload_expect = None;
            self.algo_upload_verify_left = 0;
            self.algo_upload_status =
                "算法已被设备隔离(连续致命挂死), PSoC 正在跑原生判定; 重新上传或点救援即解除"
                    .to_string();
            self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
            self.push_log_error(self.algo_upload_status.clone());
            return;
        }
        // 新固件(extended): 用 PSoC 槽内实际长度/CRC 对账, 且必须已不在上传中。
        // 旧固件: 只有 store 的 len/crc 可用, 退回原判据并在文案里注明"未能按内容对账"。
        let settled = if info.extended {
            info.psoc_valid
                && !info.uploading
                && info.psoc_len == expect_len
                && info.psoc_crc16 == expect_crc
        } else {
            info.psoc_valid && info.len == expect_len && info.crc16 == expect_crc
        };
        if settled {
            self.algo_upload_expect = None;
            self.algo_upload_verify_left = 0;
            // 文案同时保留 "ACK" 与 "已装上并正在运行": 前者是既有回执判据的关键词,
            // 后者是终态判据的关键词, 两个既有校验口径都必须继续成立。
            let tail = if info.extended {
                format!(
                    "(PSoC len={}B crc16=0x{:04X})",
                    info.psoc_len, info.psoc_crc16
                )
            } else {
                format!(
                    "(store len={}B crc16=0x{:04X}; 设备固件较旧, 未能按内容对账)",
                    info.len, info.crc16
                )
            };
            self.algo_upload_status =
                format!("上传已确认(ACK) → 算法已装上并正在运行 {}", tail);
            self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
            self.push_log(format!("算法: 上传终态确认 — {}", tail));
            return;
        }
        // ★"仍在收敛"与"失败"必须分开★ uploading / psoc_cache_unavailable 为真时, PSoC 真值本来
        // 就取不到或还没落定, 此刻宣判失败纯属冤判 —— 消耗一次重试, 等下一拍再读。
        let converging = info.extended && (info.uploading || info.psoc_cache_unavailable);
        if self.algo_upload_verify_left > 0 {
            self.algo_upload_verify_left -= 1;
            self.algo_info_refresh_in = Some(50);
            let reason = if converging {
                if info.uploading {
                    "设备仍在上传/commit"
                } else {
                    "PSoC 内容缓存暂不可用"
                }
            } else {
                "等待 PSoC commit 落定"
            };
            self.algo_upload_status = format!(
                "上传已确认(ACK): {}，核对真值中… (valid={} psoc_len={}B)",
                reason, info.psoc_valid, info.psoc_len
            );
            self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
            return;
        }
        // 重试用尽仍不符 ⇒ 如实宣判, 绝不把"未确认"显示成成功。
        // ★store 与 psoc 两侧都打出来★: "RP 存下了但 PSoC 没装上"是最常见的失败形态,
        // 只打一侧的话它和"根本没存进去"在日志里长得一模一样。
        self.algo_upload_expect = None;
        self.algo_upload_status = if info.extended {
            format!(
                "上传已确认(ACK) 但终态不匹配: 期望 len={}B crc16=0x{:04X}; \
                 RP 存储 len={}B crc16=0x{:04X}; PSoC 槽内 len={}B crc16=0x{:04X}; \
                 valid={} uploading={} download_pending={} psoc_cache_unavailable={}; \
                 中止现场: {}(第 {} 页, 累计 {} 次)",
                expect_len,
                expect_crc,
                info.len,
                info.crc16,
                info.psoc_len,
                info.psoc_crc16,
                info.psoc_valid,
                info.uploading,
                info.download_pending,
                info.psoc_cache_unavailable,
                crate::proto::algo::algo_abort_reason_text(info.abort_reason),
                if info.abort_page == 0xFFFF {
                    "BEGIN".to_string()
                } else {
                    info.abort_page.to_string()
                },
                info.abort_count
            )
        } else {
            format!(
                "上传已确认(ACK) 但终态不匹配: RP 存储 len={}B crc16=0x{:04X}, \
                 期望 len={}B crc16=0x{:04X}; valid={}。设备固件较旧, 未能按 PSoC 槽内容对账",
                info.len, info.crc16, expect_len, expect_crc, info.psoc_valid
            )
        };
        self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
        self.push_log_error(self.algo_upload_status.clone());
    }
    pub(crate) fn _handle_algo_get_rom_response(&mut self, frame: &Frame) {
        let roms = crate::proto::algo::decode_algo_get_rom(&frame.payload);
        for (i, r) in roms.iter().enumerate() {
            if i < self.algo_rom.len() {
                self.algo_rom[i] = *r;
            }
        }
        self.algo_rom_version = self.algo_rom_version.wrapping_add(1);
    }
}
