{
    // 手动电容测量的一轮抓取推进: 每 tick 最多一条 CP_GET, 顺序走 0..35。
    // 单通道 CP_SWEEP_MAX_TRIES 次仍无结果即跳过(不无限重试), 整轮 CP_SWEEP_TIMEOUT 到点即停。
    {
        let mut cp_state = cp_poll_timer.borrow_mut();
        if let Some(started_at) = cp_state.started_at {
            let now = Instant::now();
            if !connected {
                cp_state._stop("未连接".to_string());
            } else if let Some(telem_baseline) = cp_state.recovery_telem_version {
                let recovered = ctrl.telem_version() != telem_baseline
                    && ctrl.telem_scan_period_valid()
                    && ctrl.telem_samples_per_sec() > 0;
                if recovered {
                    let detail = cp_state.recovery_detail.clone();
                    cp_state._stop(format!(
                        "{}；固件已恢复: 收到新遥测，扫描周期 {:.2} ms ({} Hz)",
                        detail,
                        ctrl.telem_scan_period_us() as f64 / 1000.0,
                        ctrl.telem_samples_per_sec()
                    ));
                } else if cp_state
                    .recovery_started_at
                    .is_some_and(|at| now.duration_since(at) >= CP_RECOVERY_TIMEOUT)
                {
                    let detail = cp_state.recovery_detail.clone();
                    let failure = format!(
                        "{}；固件恢复失败: {} 秒内未收到恢复后的遥测扫描进展，请检查 PSoC 固件/链路后手动恢复",
                        detail,
                        CP_RECOVERY_TIMEOUT.as_secs()
                    );
                    ctrl.push_log_error(failure.clone());
                    cp_state._stop(failure);
                }
            } else if now.duration_since(started_at) >= CP_SWEEP_TIMEOUT {
                let done = cp_state.ch.min(36);
                cp_state._await_recovery(
                    ctrl.telem_version(),
                    format!("Cp 获取超时: 仅完成 {}/36 通道", done),
                );
            } else {
                // 本通道是否已"落定": 响应到达(版本推进)且不是"测量中(0)"。
                let ch = cp_state.ch;
                let responded = cp_state.tries > 0 && ctrl.cp_channel_version(ch) > cp_state.req_version;
                if responded && ctrl.cp(ch) != Some(0) {
                    cp_state._advance();
                }
                let due = cp_state.next_at.map_or(true, |deadline| now >= deadline);
                if cp_state.ch >= 36 {
                    // 统计口径与单通道文案同源: 失败 = 回读 0x00FFFFFF; 其余取不到结果的只算"无结果"。
                    let ok = (0..36u8)
                        .filter(|&c| matches!(ctrl.cp(c), Some(v) if v != 0 && v != CP_MEASURE_FAILED))
                        .count();
                    let failed = cp_failed_list(&ctrl);
                    let detail = if failed.is_empty() {
                        format!("电容测量完成: {}/36 通道有效, 无测量失败", ok)
                    } else {
                        format!("电容测量完成: {}/36 通道有效; {} {}", ok, failed, CP_FAILURE_TEXT)
                    };
                    cp_state._await_recovery(ctrl.telem_version(), detail);
                } else if due && cp_state.tries >= CP_SWEEP_MAX_TRIES {
                    cp_state._advance();   // 该通道取不到 → 跳过, 绝不无限重试
                } else if due {
                    let ch = cp_state.ch;
                    cp_state.req_version = ctrl.cp_channel_version(ch);
                    match ctrl.request_cp(ch) {
                        Ok(()) => {
                            cp_state.tries += 1;
                            cp_state.next_at = Some(now + CP_SWEEP_RETRY);
                            cp_state.status = format!("测量中… (CH{}/36)", ch + 1);
                        }
                        Err(error) => cp_state._stop(format!("测量失败: {}", error)),
                    }
                }
            }
        }
        // 空闲时文案 = 当前通道 Cp 缓存(未测量则显示"未测量")。
        let text = if cp_state.started_at.is_some() {
            cp_state.status.clone()
        } else if !connected {
            "未连接".to_string()
        } else {
            cp_display_text(ctrl.cp(current_channel.clamp(0, 35) as u8))
        };
        ui.set_cp_text(text.into());
        // 失败通道列表送到全局调整页(原 GND 猜测警告的位置): 那里是唯一有设备级证据的警告位,
        // 空列表则整条警告隐藏。
        let failed = cp_failed_list(&ctrl);
        if failed != last_cp_failed {
            last_cp_failed = failed.clone();
            ui.set_cp_failed_channels(failed.into());
        }
    }
}
