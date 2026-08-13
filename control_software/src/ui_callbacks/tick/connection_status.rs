{
    // 自动重连仅在启动默认态或用户主动连接后启用；手动断开会保持停止扫描。
    reconnect_tick = reconnect_tick.wrapping_add(1);
    // ★节拍与墙钟解耦★ 所有周期性动作都按毫秒表达(`_every_ms`), 不再写死"每 N 个 tick" ——
    // tick 周期改动时那种写法会静默把所有轮询间隔一起缩放掉。
    if auto_reconnect.get()
        && ctrl.state() == ConnState::Disconnected
        && _every_ms(reconnect_tick, 2_000)
    {
        ctrl.refresh_devices();
        let labels: Vec<slint::SharedString> =
            ctrl.device_labels().into_iter().map(|s| s.into()).collect();
        ui.set_device_labels(slint::ModelRc::new(slint::VecModel::from(labels)));
        if ctrl.device_count() > 0 {
            let _ = ctrl.connect(0);
        }
    }
    // 握手重试:仍在 Connecting(已发首个 HELLO 但未收到 DEVICE_INFO)时每 ~400ms 重发 HELLO。
    // 重连场景下设备端 bulk OUT data toggle 与新句柄不同步会丢弃首个 HELLO, 重发即可握手成功。
    if ctrl.state() == ConnState::Connecting && _every_ms(reconnect_tick, 400) {
        let _ = ctrl.resend_hello();
    }
    // PSoC 链路活性探测(~1s 一次)。连接基础回读期间让 CFG/参数/状态探针独占会话，避免 HELLO 踢停流。
    if !ctrl.conn_probes_pending() && _every_ms(reconnect_tick, 1_000) {
        ctrl.psoc_link_probe();
    }

    ui.set_conn_status(ctrl.status_line().into());
    // 主页默认看摘要(人可读), 原始 debug 诊断在折叠区里备查。
    ui.set_device_summary_text(ctrl.device_summary_text().into());
    ui.set_device_info_text(ctrl.device_info_text().into());
    // 主页工作模式(只读展示): 草稿优先与配置页同口径, 未回读到即"未知"。
    ui.set_work_mode_text(work_mode_text(&ctrl).into());
    // 关于页设备版本: 复用主页那份 DEVICE_INFO, 不新增协议请求。
    ui.set_about_device_versions(
        ctrl.device_version_lines()
            .unwrap_or_else(|| "未连接 — 连接设备后显示主控 / PSoC 固件与协议版本".to_string())
            .into(),
    );

    // 阻塞类操作(校准/基线/重启/频率自适应)状态 → 驱动各页按钮锁定 + 运行中标识。
    ui.set_op_busy(ctrl.op_busy());
    ui.set_op_label(ctrl.op_label().into());
    // 频率自适应结果文案。全局页只显示全通道那次的结果; 单通道页显示所选通道自己的结果。
    let auto_tune_status = if ctrl.auto_tune_ch() == 0xFF {
        // ★逐通道模式★: 终态 div 字段是"成功通道数", 不是某个统一分频。
        match ctrl.auto_tune_result() {
            1 => {
                let ok = ctrl.auto_tune_div().min(36);
                let (lo, hi) = ctrl.sns_clk_div_range();
                if ok >= 36 {
                    format!("✔ 逐通道自适应成功: 36/36 个通道各自落档, 分频范围 ÷{} – ÷{}", lo, hi)
                } else {
                    format!("▲ 逐通道自适应完成: {}/36 成功, {} 个通道超出硬件能力(保持原分频); 分频范围 ÷{} – ÷{}",
                            ok, 36 - ok, lo, hi)
                }
            }
            2 => "✖ 逐通道自适应失败: 无任何通道能压到目标%, 请降低校准目标% 或调整 IDAC".to_string(),
            _ => String::new(),
        }
    } else {
        // ★host 侧逐通道队列跑的是"每通道一次单通道自适应"★ ⇒ 设备侧再也不会给出"全通道那一次"
        // 的统一终态, 全局页的结论必须从 36 个通道各自的终态汇总出来。
        let mut ok = 0u32;
        let mut bad = 0u32;
        for ch in 0..36u8 {
            match ctrl.auto_tune_ch_result(ch) {
                1 => ok += 1,
                2 => bad += 1,
                _ => {}
            }
        }
        if ok + bad == 0 {
            String::new()
        } else {
            let (lo, hi) = ctrl.sns_clk_div_range();
            format!(
                "逐通道自适应: {} 个通道落档成功, {} 个超出硬件能力(保持原分频); 已下探 {}/36, 分频范围 ÷{} – ÷{}",
                ok, bad, ok + bad, lo, hi
            )
        }
    };
    ui.set_auto_tune_status(auto_tune_status.into());
    // 全通道操作的 host 侧串行队列: 进度/状态/取消可用性(三种操作共用一条队列, 故只一份状态)。
    ui.set_batch_op_active(ctrl.ch_batch_active());
    ui.set_batch_op_progress(ctrl.ch_batch_progress());
    ui.set_batch_op_status(ctrl.ch_batch_status().into());
    ui.set_cp_assist(ctrl.cp_assist());
    // 绘图视窗冻结态("停止" = 只冻画面; 遥测仍在收)。
    ui.set_plot_frozen(ctrl.plot_frozen());
    // 单通道响应噪声频谱。结果带不可变扫描通道：选择已切换时清空热图，状态文本仍说明它属于哪一条 CH。
    ui.set_curve_spectrum_active(ctrl.noise_sweep_active());
    ui.set_curve_spectrum_progress(ctrl.noise_sweep_progress());
    ui.set_curve_spectrum_status(ctrl.noise_sweep_status().into());
    let sel_ch = ui.get_sel_channel().clamp(0, 35) as u8;
    if ctrl.noise_sweep_version() != last_spectrum_version || last_spectrum_channel != sel_ch as i32 {
        last_spectrum_version = ctrl.noise_sweep_version();
        last_spectrum_channel = sel_ch as i32;
        let cells = if ctrl.noise_sweep_result_channel() == Some(sel_ch) {
            build_spectrum_cells(&ctrl)
        } else {
            Vec::new()
        };
        ui.set_curve_spectrum_cells(slint::ModelRc::new(slint::VecModel::from(cells)));
    }
    let curve_auto_tune_status = match ctrl.auto_tune_ch_result(sel_ch) {
        1 => format!("✔ CH{} 自适应成功: snsClk 分频 = {}", sel_ch, ctrl.auto_tune_ch_div(sel_ch)),
        2 => format!("✖ CH{} 自适应失败: 已到硬件频率下限仍压不到目标%, 请降低校准目标% 或调整 IDAC", sel_ch),
        _ => String::new(),
    };
    ui.set_curve_auto_tune_status(curve_auto_tune_status.into());
}
