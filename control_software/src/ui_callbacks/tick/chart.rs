{
    // ★主图回填: 唯一绘图管线的唯一消费处★
    // 主曲线(左轴)、4 条算法上报、触发判定全部来自同一个 ChartFrame ⇒ x 定义域与绘制面积
    // 天然一致。这里【不许】再出现第二个产物, 也不许把 t_first/t_last 传给别的生成函数 ——
    // 那正是旧实现"叠加线只覆盖部分宽度"的来路(见 build_chart_frame 顶部的不变量)。
    // 门控里【不再】有绘图区尺寸: PlotPath 用 fit: fill 拉满, 拖分栏/改窗口只是元素几何变化,
    // path 不必重算 —— 纯 UI 交互不该把 36 通道的采样重新投影一遍。
    // ★遥测驱动的重建限速到约 30Hz(visual_refresh_tick)★: 数据仍以固件节奏全速进环形缓冲,
    // 只是"重投影成 SVG path 并推给 Slint"这一步没必要跟 16ms tick 一样快 —— 36 点以上的
    // path 重建 + 多个 VecModel::set_row_data 是本页最重的每 tick 开销, 高负载下会挤占
    // UI 线程导致明显卡顿。通道切换/勾选可见性变化仍立即生效, 不等下一个节拍。
    // ★页门控是第一道闸★ 实测(PROF)单次主图重建 ~12ms/峰值 ~17ms, 已占满 16ms tick 预算;
    // 而它此前在主页/工具箱/日志页也照样每帧重算 —— 那些页面根本没有这张图。曲线只在
    // "设置 → 单通道精调"可见, 不可见时连算都不该算。
    // 第二道闸是 visual_refresh_tick(~30Hz): 遥测实测 171Hz, 没有必要每帧重投影一次。
    let curves_visible = ui.get_current_view() == 1 && ui.get_settings_tab() == SETTINGS_PAGE_CURVES;
    // ★单通道独占流的唯一驱动点★ 只有"设置 → 单通道精调"这一页在看单个通道的波形, 也只有它
    // 值得让固件把整条链路让给一个通道(FOCUS_START 会挂起广谱流, FOCUS_STOP 自动恢复)。
    // 本调用幂等: 目标没变时一条命令都不发, 于是"进页/离页/在页内切通道"三件事各只触发一次
    // 切换, 不会每 16ms 刷一轮 START/STOP。通道取当前选中通道(与曲线/参数页同一个来源)。
    let focus_target = if curves_visible && connected && !ctrl.telem_user_paused() {
        Some(current_channel.clamp(0, 35) as u8)
    } else {
        None
    };
    ctrl.focus_set_target(focus_target);
    // ★algo_overlay_dirty 也是数据驱动的★, 必须和遥测版本走同一道限速: 它跟着算法追踪回读
    // (与遥测同量级的频率)置位, 若让它单独绕过限速, 主图就会退化成"每个 tick 重投影一次"。
    if curves_visible
        && (((current_plot_version != last_plot_version || algo_overlay_dirty)
            && visual_refresh_tick)
            || channel_changed
            || curve_visibility_changed)
    {
        let prof_chart_start = Instant::now();
        last_plot_version = current_plot_version;
        last_channel = current_channel;
        last_curve_visibility = curve_visibility;
        algo_overlay_dirty = false;
        let decls = ctrl.algo_report_decls();
        let amps = *report_norm_timer.borrow();
        // 上报线"参与绘制"的三重判定: 叠加总开关开 + 算法声明了该 idx + 用户未取消勾选。
        let overlay_on = ui.get_show_algo_overlay();
        let mut report_wanted = [false; 4];
        for (idx, slot) in report_wanted.iter_mut().enumerate() {
            *slot = overlay_on
                && decls.iter().any(|d| d.idx == idx as u8)
                && report_show_timer.borrow()[idx];
        }
        let frame = build_chart_frame(
            &ctrl,
            current_channel as u8,
            SeriesShow {
                raw: curve_visibility.0,
                bsln: curve_visibility.1,
                diff: curve_visibility.2,
                report: report_wanted,
                active: ui.get_show_active(),
            },
            &amps,
        );

        // 横轴真实时间: 跨度(ms)给 UI 换算刻度/十字线读数, 绝对时刻只做一行文本展示
        // (f32 存不住小时级的 ms 绝对值, 故 UI 侧一律用"相对最新样本"的偏移)。
        ui.set_curve_t_span_ms(frame.x.t_span_ms());
        ui.set_curve_t_end_text(dev_time_text(frame.x.t_last_us).into());

        // 左轴(ADC 量纲)三条主曲线。★颜色也由本图唯一调色板给出★: .slint 里再写一份色值
        // 就等于两套色表, 必然跨组撞色(旧实现的上报线用了 [绿,蓝,橙], 与主曲线三条全撞)。
        ui.set_raw_path(frame.path_of(SeriesId::Raw).into());
        ui.set_bsln_path(frame.path_of(SeriesId::Bsln).into());
        ui.set_diff_path(frame.path_of(SeriesId::Diff).into());
        ui.set_raw_color(frame.color_of(SeriesId::Raw));
        ui.set_bsln_color(frame.color_of(SeriesId::Bsln));
        ui.set_diff_color(frame.color_of(SeriesId::Diff));
        ui.set_curve_y_min(frame.left.min);
        ui.set_curve_y_mid(frame.left.mid);
        ui.set_curve_y_max(frame.left.max);
        ui.set_curve_point_count(frame.point_count);

        // 右轴(算法量纲)量程 + 触发判定线。左右轴各有 y 量程, 但共享上面那一套 x。
        ui.set_curve_r_min(frame.right.min);
        ui.set_curve_r_max(frame.right.max);
        ui.set_active_path(frame.path_of(SeriesId::Active).into());
        ui.set_active_point_count(frame.count_of(SeriesId::Active));
        ui.set_active_color(frame.color_of(SeriesId::Active));
        ui.set_active_norm_amp(amps[4]);

        for idx in 0usize..4 {
            let id = SeriesId::Report(idx as u8);
            let decl = decls.iter().find(|d| d.idx == idx as u8);
            let line = AlgoReportLine {
                idx: idx as i32,
                // 图例优先用别名: 别名是用户在算法页元数据面板里为该上报变量起的可读名,
                // 声明里的 name 往往是源码标识符。别名为空则退回 name(不留空图例)。
                name: decl
                    .map(|d| {
                        if d.alias.trim().is_empty() {
                            d.name.clone()
                        } else {
                            d.alias.clone()
                        }
                    })
                    .unwrap_or_default()
                    .into(),
                path: frame.path_of(id).into(),
                // 声明 + 有数据 + 用户未取消勾选 → 才画在主图上。
                visible: report_wanted[idx] && frame.count_of(id) > 0,
                is_binary: frame.binary_of(id),
                norm_amp: amps[idx],
                line_color: frame.color_of(id),
                // ★类型/范围只在真有 META 声明时给★ declared_binary.is_some() 就是"这条声明
                // 带类型参数"的唯一判据(见 proto/algo.rs: 旧式 ALGO_REPORT 的 u16/0..65535 是
                // 展示兜底)。没有声明就交空串, 让 UI 如实说"未声明类型", 不拿兜底冒充事实。
                value_type: decl
                    .filter(|d| d.declared_binary.is_some())
                    .map(|d| d.value_type.clone())
                    .unwrap_or_default()
                    .into(),
                range: decl
                    .filter(|d| d.declared_binary.is_some())
                    .map(|d| d.range.clone())
                    .unwrap_or_default()
                    .into(),
                // 当前值取该槽最近一次运行值(O(1) 访问器, 不克隆缓冲)。整数量纲直接去掉小数尾。
                current_value: decl
                    .and_then(|d| ctrl.algo_trace_report_last(d.idx))
                    .map(|v| format!("{}", v.round() as i64))
                    .unwrap_or_default()
                    .into(),
            };
            if algo_report_lines_model_timer.row_data(idx).as_ref() != Some(&line) {
                if idx < algo_report_lines_model_timer.row_count() {
                    algo_report_lines_model_timer.set_row_data(idx, line);
                } else {
                    algo_report_lines_model_timer.push(line);
                }
            }
        }

        // 阈值线走左轴的同一套映射(与曲线逐像素同坐标系, 缩放时永不错层)。
        let finger_th = ctrl.param(current_channel as u8, PARAM_FINGER_TH).unwrap_or(0) as f32;
        let noise_th = ctrl.param(current_channel as u8, PARAM_NOISE_TH).unwrap_or(0) as f32;
        ui.set_finger_th_y(frame.left_value_to_y(finger_th));
        ui.set_noise_th_y(frame.left_value_to_y(noise_th));

        // 当前值读数(raw/diff/baseline + 量程 + 阈值), 兑现"折线图数值与范围提醒"。
        let (lr, ld, lb) = match ctrl.telem_latest(current_channel as u8) {
            Some(s) => (
                s.raw.unwrap_or(0) as i32,
                s.diff.unwrap_or(0) as i32,
                s.bsln.unwrap_or(0) as i32,
            ),
            None => (0, 0, 0),
        };
        let samples_per_sec = ctrl.telem_samples_per_sec();
        let window_expected = PLOT_WINDOW_US as f64 / 1_000_000.0 * samples_per_sec as f64;
        let sample_diag = if samples_per_sec > 0 {
            format!(
                "数据诊断：30 秒窗口通道采样点 {}；设备 samples_per_sec={}；按 30 秒 × 设备速率估算理论 {:.0} 点，通道采样点覆盖率 {:.0}%",
                frame.channel_point_count,
                samples_per_sec,
                window_expected,
                frame.channel_point_count as f64 / window_expected * 100.0,
            )
        } else {
            format!(
                "数据诊断：30 秒窗口通道采样点 {}；设备 samples_per_sec=0，无法估算理论点数与通道采样点覆盖率",
                frame.channel_point_count
            )
        };
        let readout = if frame.point_count > 0 {
            format!(
                "CH{} 当前 raw={} diff={} bsln={} | 纵轴量程 [{} .. {}] | 手指阈值 {} 噪声阈值 {} | {}",
                current_channel, lr, ld, lb,
                frame.left.min.round() as i32, frame.left.max.round() as i32,
                finger_th as i32, noise_th as i32, sample_diag
            )
        } else {
            format!("CH{} 等待遥测数据 | 手指阈值 {} 噪声阈值 {} | {}",
                current_channel, finger_th as i32, noise_th as i32, sample_diag)
        };
        ui.set_curve_readout(readout.into());
        let spent = prof_chart_start.elapsed().as_micros() as u64;
        prof_chart_us += spent;
        prof_chart_hits += 1;
        prof_chart_max_us = prof_chart_max_us.max(spent);
    }
}
