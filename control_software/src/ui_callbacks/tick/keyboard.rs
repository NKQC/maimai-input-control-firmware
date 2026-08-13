{
    // 主页延迟历史折线, version 门控。绘制完整历史铺满 viewbox(1000宽), 由 UI 侧 viewbox
    // 缩放/横向滚动条查看局部, 不再固定窗口只显示一小段。
    // ★两种口径, 由补偿开关切换★ 启用补偿 → 画"距期望延迟的偏差 ±"(0 线 = 设定延迟正好兑现);
    // 关闭 → 画链路耗时原值(此时没有期望零点, 偏差无从定义)。标题跟着口径一起变, 免得看错量。
    // ★不允许出现"链路耗时 − 期望"这第三种口径★: 两者量纲不同, 相减没有物理含义。
    if ctrl.lat_version() != last_lat_version {
        last_lat_version = ctrl.lat_version();
        let dev_mode = ctrl.latency_correction_enabled();
        let series = if dev_mode {
            ctrl.lat_dev_series()
        } else {
            ctrl.lat_link_series()
        };
        let (path, lo, hi) = build_lat_path(&series);
        ui.set_lat_path(path.into());
        ui.set_lat_y_max(hi);
        ui.set_lat_y_min(lo);
        ui.set_lat_point_count(series.len() as i32);
        ui.set_lat_chart_title(
            if dev_mode {
                format!(
                    "延迟偏差历史 (实测端到端 − 期望 {} µs，µs)",
                    ctrl.touch_delay_target_us()
                )
            } else {
                "链路耗时历史 (SPI + RP处理 + USB写，µs)".to_string()
            }
            .into(),
        );
    }

    // 物理键盘实时三态(version 门控): 去抖后 / 去抖前 / 实际输出 HID。
    if ctrl.kbd_state_version() != last_kbd_state_version {
        last_kbd_state_version = ctrl.kbd_state_version();
        let st = ctrl.kbd_state();
        let raw = ctrl.kbd_state_raw();
        let out = ctrl.kbd_state_out();
        let pressed: Vec<bool> = (0..12u8).map(|i| (st >> i) & 1 != 0).collect();
        ui.set_kbd_phys_pressed(slint::ModelRc::new(slint::VecModel::from(pressed)));
        let raw_bits: Vec<bool> = (0..12u8).map(|i| (raw >> i) & 1 != 0).collect();
        ui.set_kbd_phys_raw(slint::ModelRc::new(slint::VecModel::from(raw_bits)));
        let out_bits: Vec<bool> = (0..12u8).map(|i| (out >> i) & 1 != 0).collect();
        ui.set_kbd_phys_out(slint::ModelRc::new(slint::VecModel::from(out_bits)));
        // 触控→键盘链路诊断行: 与实时三态同一份 KBD_GET_STATE 响应, 因此复用同一个 version
        // 门控与既有的 ~3Hz 轮询, 不新增任何轮询源。
        // ★读不到诊断段就明说不可用★ 不拿零值推结论 —— 那会把"旧固件不回传"说成"各环都为假"。
        ui.set_kbd_link_diag(
            match ctrl.kbd_link_diag() {
                Some(diag) => diag.summary(),
                None => "诊断字段不可用（设备固件未回传链路诊断段）".to_string(),
            }
            .into(),
        );
    }
    // 每键触发极性 + 防抖(version 门控, 草稿优先值)。
    if ctrl.kbd_keycfg_version() != last_kbd_keycfg_version {
        last_kbd_keycfg_version = ctrl.kbd_keycfg_version();
        let pol: Vec<i32> = (0..12u8).map(|i| ctrl.kbd_keycfg(i).pol as i32).collect();
        // ★生效电平只从固件回传的 resolved mask 逐位摊平★: 不用 pol_mode 推算, 否则
        // 显示的就不是设备真值而是本地二值化猜测。掩码没读到过时标记"未知"而非假装低电平。
        let resolved = ctrl.kbd_pol_resolved_mask();
        let resolved_known = ctrl.kbd_pol_resolved_known();
        let pol_hi: Vec<bool> = (0..12u8).map(|i| (resolved >> i) & 1 != 0).collect();
        let pol_known: Vec<bool> = vec![resolved_known; 12];
        let db: Vec<i32> = (0..12u8)
            .map(|i| ctrl.kbd_keycfg(i).debounce_us as i32)
            .collect();
        ui.set_kbd_phys_pol_mode(slint::ModelRc::new(slint::VecModel::from(pol)));
        ui.set_kbd_phys_pol_resolved_high(slint::ModelRc::new(slint::VecModel::from(pol_hi)));
        ui.set_kbd_phys_pol_resolved_known(slint::ModelRc::new(slint::VecModel::from(pol_known)));
        ui.set_kbd_phys_debounce(slint::ModelRc::new(slint::VecModel::from(db)));
        // 旧固件没有这两条命令: 明说"不支持"而不是显示一份看起来能改的假默认值。
        let status = match ctrl.kbd_keycfg_supported() {
            Some(false) => "设备固件不支持每键极性/防抖(需升级固件)".to_string(),
            None => "每键极性/防抖: 未回读".to_string(),
            Some(true) if !resolved_known => {
                "当前生效电平: 未知(设备固件未回传生效极性掩码)".to_string()
            }
            Some(true) => {
                // ★按生效电平统计★: 只报"高/低"而不说明有多少是自动判定的, 等于把三态
                // 折成二值; 自动档数量必须一起给出, 用户才知道这些结论是上电学来的。
                let high = (0..12u8)
                    .filter(|i| (resolved >> *i) & 1 != 0)
                    .count();
                let auto_high = (0..12u8)
                    .filter(|i| {
                        ctrl.kbd_keycfg(*i).pol == mai2control_ui::proto::KBD_POL_AUTO
                            && (resolved >> *i) & 1 != 0
                    })
                    .count();
                let auto_low = (0..12u8)
                    .filter(|i| {
                        ctrl.kbd_keycfg(*i).pol == mai2control_ui::proto::KBD_POL_AUTO
                            && (resolved >> *i) & 1 == 0
                    })
                    .count();
                format!(
                    "{} 键当前高电平触发(其中 {} 键为自动判定) · {} 键当前低电平触发(其中 {} 键为自动判定)",
                    high,
                    auto_high,
                    12 - high,
                    auto_low
                )
            }
        };
        ui.set_kbd_keycfg_status(status.into());
    }
    // 逻辑分析仪: 有新边沿或时间窗变化时重建 12 通道时序图(抽屉收起时不重建, 省 CPU)。
    {
        let win_idx = la_window_idx.get();
        let edges_changed = ctrl.kbd_edges_version() != last_la_version;
        if ui.get_phys_la_expanded() && (edges_changed || win_idx != last_la_window) {
            last_la_version = ctrl.kbd_edges_version();
            last_la_window = win_idx;
            let view = build_logic_analyzer(ctrl.kbd_edges(), LA_WINDOWS_US[win_idx]);
            ui.set_la_raw_paths(slint::ModelRc::new(slint::VecModel::from(view.raw_paths)));
            ui.set_la_deb_paths(slint::ModelRc::new(slint::VecModel::from(view.deb_paths)));
            ui.set_la_out_paths(slint::ModelRc::new(slint::VecModel::from(view.out_paths)));
            ui.set_la_trig_text(slint::ModelRc::new(slint::VecModel::from(view.trig_text)));
            ui.set_la_trig_x(slint::ModelRc::new(slint::VecModel::from(view.trig_x)));
            ui.set_la_time_labels(slint::ModelRc::new(slint::VecModel::from(view.time_labels)));
            ui.set_la_span_text(view.span_text);
            // ★丢失必须显式告知★: 环满丢最旧时波形会"缺一段", 不提示就等于给出连续的假象。
            let status = match ctrl.kbd_edges_supported() {
                Some(false) => "设备固件不支持边沿记录(需升级固件)".to_string(),
                None => String::new(),
                Some(true) => {
                    let lost = ctrl.kbd_edge_lost();
                    let remain = ctrl.kbd_edge_remaining();
                    let mut s = String::new();
                    if lost > 0 {
                        s.push_str(&format!("⚠ 设备缓冲溢出, 已丢失 {} 条边沿事件; ", lost));
                    }
                    if remain > 0 {
                        s.push_str(&format!("设备侧仍有 {} 条待取; ", remain));
                    }
                    s
                }
            };
            ui.set_la_status(status.into());
        }
    }
    // 物理键 HID 键码 → 下拉索引 + 修饰位(version 门控, 避免覆盖用户编辑)。
    if ctrl.kbd_map_version() != last_kbd_map_version {
        last_kbd_map_version = ctrl.kbd_map_version();
        let choices: Vec<i32> = (0..12u8).map(|i| kbd_code_to_choice(ctrl.kbd_map(i))).collect();
        ui.set_kbd_phys_choice(slint::ModelRc::new(slint::VecModel::from(choices)));
        let mods: Vec<i32> = (0..12u8).map(|i| ctrl.kbd_keymod(i) as i32).collect();
        ui.set_kbd_phys_mod(slint::ModelRc::new(slint::VecModel::from(mods)));
        let disp: Vec<slint::SharedString> =
            (0..12u8).map(|i| kbd_display(ctrl.kbd_map(i), ctrl.kbd_keymod(i)).into()).collect();
        ui.set_kbd_phys_display(slint::ModelRc::new(slint::VecModel::from(disp)));
    }
    // 触控分区 HID 键码 → 下拉索引 + 修饰位(version 门控)。
    if ctrl.kbd_touchmap_version() != last_kbd_touchmap_version {
        last_kbd_touchmap_version = ctrl.kbd_touchmap_version();
        let choices: Vec<i32> = (0..34u8).map(|z| kbd_code_to_choice(ctrl.kbd_touch_keycode(z))).collect();
        ui.set_kbd_zone_choice(slint::ModelRc::new(slint::VecModel::from(choices)));
        let mods: Vec<i32> = (0..34u8).map(|z| ctrl.kbd_zone_mod(z) as i32).collect();
        ui.set_kbd_zone_mod(slint::ModelRc::new(slint::VecModel::from(mods)));
        let disp: Vec<slint::SharedString> =
            (0..34u8).map(|z| kbd_display(ctrl.kbd_touch_keycode(z), ctrl.kbd_zone_mod(z)).into()).collect();
        ui.set_kbd_zone_display(slint::ModelRc::new(slint::VecModel::from(disp)));
    }
    // 长按参数(version 门控): 只在设备侧真值更新时回填, 不覆盖正在编辑的 SpinBox。
    if ctrl.kbd_hold_version() != last_kbd_hold_version {
        last_kbd_hold_version = ctrl.kbd_hold_version();
        let phys_delay: Vec<i32> = (0..12u8)
            .map(|idx| ctrl.kbd_hold_phys(idx).0 as i32)
            .collect();
        let phys_max: Vec<i32> = (0..12u8)
            .map(|idx| ctrl.kbd_hold_phys(idx).1 as i32)
            .collect();
        ui.set_kbd_phys_hold_delay(slint::ModelRc::new(slint::VecModel::from(phys_delay)));
        ui.set_kbd_phys_hold_max(slint::ModelRc::new(slint::VecModel::from(phys_max)));
        let zone_delay: Vec<i32> = (0..34u8)
            .map(|zone| ctrl.kbd_hold_zone(zone).0 as i32)
            .collect();
        let zone_max: Vec<i32> = (0..34u8)
            .map(|zone| ctrl.kbd_hold_zone(zone).1 as i32)
            .collect();
        ui.set_kbd_zone_hold_delay(slint::ModelRc::new(slint::VecModel::from(zone_delay)));
        ui.set_kbd_zone_hold_max(slint::ModelRc::new(slint::VecModel::from(zone_max)));
    }
}
