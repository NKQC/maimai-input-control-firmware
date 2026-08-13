{
    // 采样率/探测周期：每 tick 回填(与遥测帧到达节奏一致，STATS 字段随 TELEM_DATA 更新)。
    // 探测周期统一展示为两位小数 ms(设备实测值), 由 scan_period_us 换算。
    ui.set_sample_rate_hz(ctrl.telem_samples_per_sec() as i32);
    ui.set_scan_period_ms(ctrl.telem_scan_period_us() as f32 / 1000.0);
    ui.set_scan_period_us(ctrl.telem_scan_period_us() as i32);
    // ★停流时不要把 0 当成实测值★ 主页/全通道页/全局调整页共用同一来源(telem_scan_period_us),
    // 该来源只在遥测帧带 STATS 时更新; 无值时靠这个标志让三处一致显示"未测"。
    ui.set_scan_period_valid(ctrl.telem_scan_period_valid());
    // 期望探测周期由当前分辨率解算(CH0 代表值), ★与 SnsClk 分频无关★:
    // CSDv2 子转换数 ∝ 1/snsClkDiv, 换能时长 = 2^res/ModClk, 分频相互抵消(真机已证)。
    // 改变周期的是分辨率(2^res); 分频只改传感激励频率(Cp 灵敏度/抗噪)。见 expected_scan_period_us 注释。
    ui.set_scan_period_expected_us(expected_scan_period_us(&ctrl));

    // 单通道阈值可判定区域警告: 可判定余量 = 2^分辨率(满量程) − 当前基线。
    // 若该余量 < 手指阈值, 则 diff 永远达不到阈值 → 该通道实际永不触发, 红框警示 + 建议。
    {
        let wch = current_channel.clamp(0, 35) as u8;
        let res = ctrl.param(0, PARAM_RESOLUTION).unwrap_or(0);
        let finger_th = ctrl.param(wch, PARAM_FINGER_TH).unwrap_or(0);
        let bsln = ctrl.telem_latest(wch).and_then(|s| s.bsln).map(|v| v as u32).unwrap_or(0);
        let warn = if res >= 1 && res <= 16 {
            let max_raw = 1u32 << res;
            let usable = max_raw.saturating_sub(bsln);
            if finger_th > 0 && usable < finger_th {
                format!(
                    "⚠ 可判定区域 = 2^{}({}) − 基线({}) = {} < 手指阈值({})\n\
                     该通道 diff 永远达不到阈值 → 实际永不触发! 建议: 调低手指阈值, 或提高分辨率、\
                     重新校准以降低基线, 扩大可判定余量。",
                    res, max_raw, bsln, usable, finger_th
                )
            } else {
                String::new()
            }
        } else {
            String::new()
        };
        ui.set_curve_threshold_warning(warn.into());
    ui.set_focus_bandwidth(ctrl.focus_bandwidth_text().into());
    }
    // 传感器/PSoC 健康总结(异常原因)。
    ui.set_sensor_health(ctrl.sensor_health_summary().into());
    ui.set_sensor_health_level(ctrl.sensor_health_level());
    // ★异常采样标识(全局调整页顶部状态行)★: railed 通道数 / 数据停滞 / 设备拒绝固化基线 + 动作指引。
    ui.set_sampling_advice(ctrl.sampling_advice().into());
    ui.set_sampling_advice_level(ctrl.sampling_advice_level());

    // 延迟数值始终回填(固件低成本采样,不受测量开关门控)。
    let spi = ctrl.telem_lat_spi_us() as i32;
    let proc = ctrl.telem_lat_proc_us() as i32;
    let usb = ctrl.telem_lat_usb_us() as i32;
    ui.set_latency_spi_us(spi);
    ui.set_latency_proc_us(proc);
    ui.set_latency_usb_us(usb);
    ui.set_latency_sensor_us(ctrl.telem_scan_period_us() as i32);
    ui.set_latency_total_us(spi + proc + usb);
    // ★启用补偿时要看的是"距期望延迟的偏差 ±", 不是链路耗时★
    // 偏差 = (实际发出时刻 − 该掩码的采样时刻) − comm.touch_delay_100us, 由固件算出并随帧上报
    // (FIELD_DELAY_DEV)。采样零点取自延迟线**实际读出的那一片**, 主机侧拿链路耗时是推不出来的:
    // 链路耗时只是延迟线内部被扣掉的一项, 拿它去减目标是两种量纲相减(主页此前正是这么算的,
    // 于是设 24ms 就显示"误差 −23406us")。
    // 关闭补偿时退回显示链路耗时原值 —— 那时没有"期望"这个零点, 偏差无从定义。
    let dev_target = ctrl.touch_delay_target_us();
    let corrected_text = if !ctrl.latency_correction_enabled() {
        match ctrl.telem_lat_corrected_us() {
            Some(link) => format!("链路耗时 {} us | 补偿偏差显示未启用", link),
            None => "链路耗时: 未测(等待三段非零新鲜样本)".to_string(),
        }
    } else {
        match ctrl.telem_delay_dev_us() {
            Some((lo, hi)) => {
                // 区间两端都给: 偏差里天生含延迟线 100us 时间片的截断量, 只报一端会让人以为
                // 它只往一个方向偏。端到端实测值同样按区间给, 与偏差一一对应。
                let clamp = if ctrl.telem_delay_dev_clamped() {
                    " | ⚠ 期望低于物理下限, 延迟线已钳到最新采样"
                } else {
                    ""
                };
                format!(
                    "偏差 {:+} .. {:+} us | 期望 {} us | 实测端到端 {} .. {} us{}",
                    lo,
                    hi,
                    dev_target,
                    (dev_target as i64 + lo as i64).max(0),
                    (dev_target as i64 + hi as i64).max(0),
                    clamp
                )
            }
            // 没有观测不等于偏差为 0: 延迟线只在真的发出触控帧时才被读, 而触控帧只在
            // mai2serial 链路被消费者启用后才发 ⇒ 这一支的准确含义是"链路还没启动, 等着"。
            // 原文案"未测(本窗口没有触控帧真正发出)"描述的是现象, 用户看不出该去做什么。
            None => format!("偏差: 未启动链接, 等待中 (期望 {} us)", dev_target),
        }
    };
    ui.set_latency_corrected_text(corrected_text.into());
    // ★图上右上角只有 108px 槽位★: 把上面那句完整说明塞进去必然被 elide 成一串省略号。
    // 徽标只给一个量: 偏差包络当前值 / 等待中 / 未启用 —— 完整口径看卡片里的整行说明。
    let dev_badge = if !ctrl.latency_correction_enabled() {
        "未启用".to_string()
    } else {
        match ctrl.telem_delay_dev_us() {
            Some((lo, hi)) => {
                let worst = if lo.unsigned_abs() > hi.unsigned_abs() { lo } else { hi };
                format!("偏差 {:+}us", worst)
            }
            None => "等待中".to_string(),
        }
    };
    ui.set_lat_dev_badge(dev_badge.into());
}
