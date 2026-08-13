use crate::app_state::{AppController, LedWriteOp};
use crate::proto::{Frame, HostCmd, LED_CH_UNMAPPED, LED_PREVIEW_ALL, LED_UNIT_COUNT, LedRegion};

impl AppController {
    pub fn led_version(&self) -> u64 {
        self.led_version
    }

    /// 是否已拿到设备灯效快照(未拿到时 UI 一律显示"未知", 不以 0 冒充真值)。
    pub fn led_known(&self) -> bool {
        self.led_state.is_some()
    }

    /// 灯板协议状态机: 0=停 1=就绪 2=运行; None=尚未回读。
    pub fn led_status(&self) -> Option<u8> {
        self.led_state.map(|s| s.status)
    }
    /// WS2812 指定链初始化就绪状态; None=尚未回读或索引非法。
    pub fn led_chain_ready(&self, chain: usize) -> Option<bool> {
        if chain >= 2 {
            return None;
        }
        self.led_state.map(|s| s.chain_ready[chain])
    }
    /// WS2812 初始化失败分档; None=尚未回读。
    pub fn led_init_fault(&self) -> Option<u8> {
        self.led_state.map(|s| s.init_fault)
    }
    pub fn led_resp_enabled(&self) -> Option<bool> {
        self.led_state.map(|s| s.resp_enabled)
    }
    /// 设备侧预览色是否正在覆盖协议色; None=尚未回读。false 时颜色字段即游戏协议色。
    pub fn led_preview_active(&self) -> Option<bool> {
        self.led_state.map(|s| s.preview_active)
    }
    /// 设备灯效服务是否已初始化; None=尚未回读。false = 预览色不会被刷到灯链。
    pub fn led_service_ready(&self) -> Option<bool> {
        self.led_state.map(|s| s.service_ready)
    }
    /// 设备灯效刷新是否至少执行过一次; None=尚未回读。false = 灯服务从未被主循环调用。
    pub fn led_refresh_seen(&self) -> Option<bool> {
        self.led_state.map(|s| s.refresh_seen)
    }
    /// 设备灯效刷新次数低 4 位; None=尚未回读。两次快照该值不变 = 灯服务已停摆。
    pub fn led_refresh_ticks(&self) -> Option<u8> {
        self.led_state.map(|s| s.refresh_ticks)
    }
    pub fn led_baud(&self) -> Option<u32> {
        self.led_state.map(|s| s.baud)
    }
    /// 灯板波特率是否可作为 UI 真值展示；判定集中复用协议快照的范围约束。
    pub fn led_baud_valid(&self) -> bool {
        self.led_state.map_or(false, |state| state.baud_valid())
    }
    pub fn led_rx_frames(&self) -> u32 {
        self.led_state.map_or(0, |s| s.rx_frames)
    }
    /// 收帧计数是否可无损显示，避免 u32 转 Slint int 后翻为负数。
    pub fn led_rx_frames_valid(&self) -> bool {
        self.led_state
            .map_or(false, |state| state.rx_frames_valid())
    }
    pub fn led_sum_errors(&self) -> u32 {
        self.led_state.map_or(0, |s| s.sum_errors)
    }
    /// 校验错误计数是否可无损显示，避免异常快照伪装成正常数值。
    pub fn led_sum_errors_valid(&self) -> bool {
        self.led_state
            .map_or(false, |state| state.sum_errors_valid())
    }

    /// 设备回报的灯链实际灯珠数(chain 0/1); 越界返回 0。
    pub fn led_ws_count(&self, chain: usize) -> u16 {
        if chain > 1 {
            return 0;
        }
        self.led_state.map_or(0, |s| s.ws_count[chain])
    }

    /// 单元当前采样颜色(预览生效时即预览色); 未回读时为全黑。
    pub fn led_color(&self, unit: usize) -> [u8; 3] {
        if unit >= LED_UNIT_COUNT {
            return [0, 0, 0];
        }
        self.led_state.map_or([0, 0, 0], |s| s.colors[unit])
    }

    /// 单元映射: 草稿优先 → 设备回读兜底 → 未映射。
    pub fn led_region(&self, unit: usize) -> LedRegion {
        if unit >= LED_UNIT_COUNT {
            return LedRegion::default();
        }
        if let Some(draft) = self.drafts.led_region() {
            return draft[unit];
        }
        self.led_state
            .map_or(LedRegion::default(), |s| s.regions[unit])
    }

    /// 最近一次"应用映射"的结果文本(含设备 NAK 原因); 空串=尚未操作。
    pub fn led_apply_status(&self) -> &str {
        &self.led_apply_status
    }

    /// 最近一次尚在等待设备 ACK/NAK 的灯效写操作序号(映射或预览)；None 表示已有结局或尚未下发。
    pub fn led_apply_seq(&self) -> Option<u8> {
        self.led_apply_seq.map(|(seq, _)| seq)
    }

    /// 最近一次 LED_GET 快照中的虚拟单元数量；None 表示尚未取得有效快照。
    pub fn led_unit_count(&self) -> Option<u8> {
        self.led_state.map(|s| s.unit_count)
    }

    /// 最近一次 LED_GET 快照中的设备实际生效亮度；None 表示尚未回读或旧固件未上报。
    pub fn led_applied_brightness(&self) -> Option<u8> {
        self.led_state.and_then(|state| state.applied_brightness)
    }

    /// 本地映射校验结论: Some(原因) = 明知会被设备拒收, 不该发。
    pub fn led_region_conflict(&self) -> Option<String> {
        crate::proto::validate_led_regions(&self._led_regions(), self._led_ws_counts())
    }

    /// 编辑单元映射草稿。`ch` 传 `LED_CH_UNMAPPED` 解除映射。
    pub fn led_set_region(&mut self, unit: usize, ch: u8, start: u16, count: u8) {
        if unit >= LED_UNIT_COUNT {
            return;
        }
        let mut regions = self._led_regions();
        let ch = if ch > 1 { LED_CH_UNMAPPED } else { ch };
        regions[unit] = LedRegion { ch, start, count };
        self.drafts.set_led_region(regions);
        self.led_version = self.led_version.wrapping_add(1);
    }

    /// 请求回读灯效运行态(LED_GET)。
    pub fn led_request_state(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(
                HostCmd::LedGet as u8,
                0,
                seq,
                crate::proto::encode_led_get(),
            ))?;
        }
        Ok(())
    }

    /// 整批下发 11 个单元的映射(LED_SET_REGION)。本地校验不通过则不发, 直接给出冲突原因。
    pub fn led_apply_regions(&mut self) -> anyhow::Result<()> {
        if self.io.is_none() {
            self.led_apply_status = "未连接, 无法应用映射".to_string();
            self.led_version = self.led_version.wrapping_add(1);
            return Err(anyhow::anyhow!("未连接"));
        }
        let regions = self._led_regions();
        if let Some(reason) = crate::proto::validate_led_regions(&regions, self._led_ws_counts()) {
            self.led_apply_status = format!("本地校验未通过: {}", reason);
            self.led_version = self.led_version.wrapping_add(1);
            self.push_log_warn(format!("灯效映射未下发({})", reason));
            return Ok(());
        }
        // count=0 的区段语义上就是"没占灯珠", 统一折成解除映射后下发, 免得固件把 0 长度当非法参数
        // 整批 NAK(整批原子失败对用户表现为"什么都没变", 最难排查)。
        let items: Vec<(u8, LedRegion)> = regions
            .iter()
            .enumerate()
            .map(|(unit, region)| {
                let mut region = *region;
                if region.count == 0 {
                    region = LedRegion::default();
                }
                (unit as u8, region)
            })
            .collect();
        self.led_send_regions_raw(&items)?;
        self.push_log("灯效映射: 已整批下发 LED_SET_REGION(11 单元)");
        Ok(())
    }

    /// 无头 `selftest --led` 专用的原始整批 LED_SET_REGION 下发。
    ///
    /// 设备侧必须独立验证整批原子校验；若复用 UI 的本地预校验，重叠/越界
    /// 用例会在主机被拦截而掩盖设备实现。UI 仍必须走 `led_apply_regions` 的
    /// 本地校验，此接口仅供 selftest 验证路径使用。
    pub fn led_send_regions_raw(&mut self, items: &[(u8, LedRegion)]) -> anyhow::Result<u8> {
        if self.io.is_none() {
            self.led_apply_status = "未连接, 无法下发原始映射".to_string();
            self.led_version = self.led_version.wrapping_add(1);
            return Err(anyhow::anyhow!("未连接"));
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(
                HostCmd::LedSetRegion as u8,
                0,
                seq,
                crate::proto::encode_led_set_region(items),
            ))?;
        }
        self.led_apply_seq = Some((seq, LedWriteOp::ApplyRegions));
        self.led_apply_status = "已下发, 等待设备确认...".to_string();
        self.led_version = self.led_version.wrapping_add(1);
        Ok(seq)
    }

    /// 发送预览色并返回本次下发的 seq。`unit` 传 `LED_PREVIEW_ALL` 表示全部单元;
    /// 设备约 3s 无新预览自动回协议色。
    ///
    /// 回执与"应用映射"共用 `led_apply_seq` 归因: 预览被设备拒绝(unit 越界/载荷不整)时
    /// 原因必须能显示出来, 而不是发完就当成功。返回值可忽略(UI 只关心状态文案)。
    pub fn led_preview(&mut self, unit: u8, rgb: [u8; 3]) -> anyhow::Result<u8> {
        if self.io.is_none() {
            self.led_apply_status = "未连接, 无法预览".to_string();
            self.led_version = self.led_version.wrapping_add(1);
            return Err(anyhow::anyhow!("未连接"));
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(
                HostCmd::LedPreview as u8,
                0,
                seq,
                crate::proto::encode_led_preview(&[(unit, rgb)]),
            ))?;
        }
        self.led_apply_seq = Some((seq, LedWriteOp::Preview));
        self.led_apply_status = "已下发, 等待设备确认...".to_string();
        self.led_version = self.led_version.wrapping_add(1);
        let who = if unit == LED_PREVIEW_ALL {
            "全部单元".to_string()
        } else {
            format!("单元 {}", unit)
        };
        self.push_log(format!(
            "灯效预览: {} → RGB({},{},{}), 约 3s 后自动回到协议色",
            who, rgb[0], rgb[1], rgb[2]
        ));
        Ok(seq)
    }

    /// 当前生效的 11 单元映射(草稿优先), 供校验/下发共用。
    fn _led_regions(&self) -> [LedRegion; LED_UNIT_COUNT] {
        if let Some(draft) = self.drafts.led_region() {
            return *draft;
        }
        self.led_state
            .map_or([LedRegion::default(); LED_UNIT_COUNT], |s| s.regions)
    }

    fn _led_ws_counts(&self) -> [u16; 2] {
        self.led_state.map_or([0, 0], |s| s.ws_count)
    }

    pub(crate) fn _handle_led_get_response(&mut self, frame: &Frame) {
        // 前 96B 是稳定快照；新版固件可在尾部追加运行态亮度字节，兼容旧固件。
        if frame.payload.len() < 96 {
            self.push_log_warn(format!(
                "LED_GET 响应长度错误: {} 字节 (需 >= 96)",
                frame.payload.len()
            ));
            return;
        }
        match crate::proto::decode_led_get(&frame.payload) {
            Ok(state) => {
                // 设备真值到达即丢弃草稿: 否则"应用成功"后编辑框仍显示旧草稿, 与色块/设备不同源。
                if self.drafts.led_region() == Some(&state.regions) {
                    self.drafts.drop_led_region();
                }
                self.led_state = Some(state);
                self.led_version = self.led_version.wrapping_add(1);
            }
            Err(e) => {
                self.push_log_warn(format!("LED_GET 解析失败: {}", e));
            }
        }
    }

    // ------------------------------------------------------------------
    // JIT 算法引擎 (ALGO_*)
    // ------------------------------------------------------------------
}
