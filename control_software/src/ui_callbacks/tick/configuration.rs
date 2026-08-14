{
    // 脏标记回填: 所有编辑仅暂存为草稿, 只有点击"保存"才下发并写 flash。
    // 离开设置页不再自动保存, 未保存的草稿保持有效直到用户保存或撤销。
    ui.set_config_dirty(ctrl.is_config_dirty());
    ui.set_config_dirty_count(ctrl.config_dirty_count());
    // 串行下发进度: 保存是逐条等设备回执推进的, 没有这个数界面看着像卡住, 用户还会重复点保存。
    ui.set_save_pending(ctrl.cfg_tx_pending() as i32);
    // mode.work(Serial/HID)草稿存在时提示"需重启生效"; 保存提交后延时自动重启设备重枚举。
    ui.set_mode_change_needs_reboot(ctrl.draft_needs_reboot());
    // ★必须等串行下发队列排空后才开始倒计时★
    // 原实现在 pending_reboot 一置起就开 50 tick(≈800ms), 但那一刻整批草稿(实测 301 项)刚排进
    // 队列, 而队列是"每 tick 推进一帧、逐条等设备回执", 排空需数秒 —— 于是重启落在下发中途,
    // 设备重枚举把在途传输打断: 实测 `保存队列: cmd=0x11 seq=158 超过 2s 无 ACK/NAK 已跳过`
    // + `WinUSB write failed ... device disconnected`, 余下几十项配置永远没送到, 随后固件
    // 重新 provision 又报"算法未通过校验"。SAVE_CONFIG 是这批的最后一帧, 故 pending==0
    // 等价于"全部配置含写 flash 请求都已被设备回执", 此时再等 800ms 静默让固件的落盘窗口
    // (core1 空闲 + 200ms 无主机命令)完成, 才是安全的重启点。
    if reboot_countdown.is_none() && ctrl.pending_reboot() {
        if ctrl.cfg_tx_pending() == 0 {
            reboot_countdown = Some(50);
            reboot_wait_logged = false;
            ctrl.clear_pending_reboot();
            ctrl.push_log("保存配置帧已完成，mode.work 拓扑切换重启已排程");
        } else if !reboot_wait_logged {
            reboot_wait_logged = true;
            ctrl.push_log("mode.work 拓扑切换重启已排程");
        }
    }
    if let Some(n) = reboot_countdown {
        if n == 0 {
            let _ = ctrl.reboot();
            reboot_countdown = None;
        } else {
            reboot_countdown = Some(n - 1);
        }
    }

    let current_config_version = ctrl.config_version();
    if hid_mode != last_hid_mode {
        last_hid_mode = hid_mode;
        if hid_mode {
            ui.set_settings_tab(SETTINGS_PAGE_BINDING_OR_HID);
        }
    }
    let hid_page_visible = hid_mode
        && ui.get_current_view() == 1
        && ui.get_settings_tab() == SETTINGS_PAGE_BINDING_OR_HID;
    if hid_page_visible && !last_hid_page_visible {
        refresh_hid_screenshot(&ui);
    }
    last_hid_page_visible = hid_page_visible;
    let hid_point_key = (current_config_version, ctrl.active_ch_mask());
    if last_hid_point_key.as_ref() != Some(&hid_point_key) {
        last_hid_point_key = Some(hid_point_key);
        for (row, point) in build_hid_point_cells(&ctrl).into_iter().enumerate() {
            if hid_points_model.row_data(row).as_ref() != Some(&point) {
                hid_points_model.set_row_data(row, point);
            }
        }
    }

    let config_page_visible = ctrl.state() == ConnState::Connected
        && ui.get_current_view() == 1
        && matches!(
            ui.get_settings_tab(),
            SETTINGS_PAGE_PROTOCOL | SETTINGS_PAGE_CONFIG
        );
    if current_config_version != last_config_version
        || config_page_visible && !last_config_page_visible
    {
        last_config_version = current_config_version;
        let entries = ctrl.config_entries();
        let rows = build_config_rows(&entries);
        // ★分组直方图★: 组标题就是渲染归属, 一旦某组为 0 行, 对应页面就是一片空白, 而"空白"
        // 这个症状说不出是"没取到配置""归组名写错"还是"页面没接上数据源"。这行日志把三者分开:
        // entries=0 ⇒ 没取到; 某组=0 ⇒ 归组名对不上; 都正常却仍空白 ⇒ 页面绑定问题。
        {
            let mut hist: std::collections::BTreeMap<String, usize> =
                std::collections::BTreeMap::new();
            for r in &rows {
                *hist.entry(r.group.to_string()).or_insert(0) += 1;
            }
            log::info!(
                "配置分组直方图: entries={} rows={} {:?}",
                entries.len(),
                rows.len(),
                hist
            );
            // ★所有会被渲染的组都要出现在计数表里, 包括 0 行的★
            // 只回填"有行的组"等于让空组继续静默 —— 那正是"协议页一片空白说不出原因"的成因。
            const RENDERED_GROUPS: [&str; 8] = [
                "mai2serial 协议参数",
                "mai2light 协议参数",
                "延迟补正",
                "开机校准",
                "工作模式",
                "键盘映射",
                "状态指示灯",
                "其他",
            ];
            let counts: Vec<ConfigGroupCount> = RENDERED_GROUPS
                .iter()
                .map(|g| ConfigGroupCount {
                    group: (*g).into(),
                    count: hist.get(*g).copied().unwrap_or(0) as i32,
                })
                .collect();
            ui.set_config_group_counts(slint::ModelRc::new(slint::VecModel::from(counts)));
        }
        // 未归类行数: Slint 没有数组过滤/计数, 空组不渲染要靠这里给出行数。
        ui.set_config_other_count(rows.iter().filter(|r| r.group == "其他").count() as i32);
        ui.set_config_rows(slint::ModelRc::new(slint::VecModel::from(rows)));
        let settings_counts = mai2control_ui::settings_io::group_counts(&ctrl);
        ui.set_settings_io_config_count(settings_counts.config);
        ui.set_settings_io_channel_params_count(settings_counts.channel_params);
        ui.set_settings_io_globals_count(settings_counts.globals);
        ui.set_settings_io_algo_count(settings_counts.algo);
        ui.set_settings_io_keyboard_count(settings_counts.keyboard);
        ui.set_settings_io_zones_count(settings_counts.zones);

        if let Some(entry) = ctrl.config_get("comm.keyboard_map_en") {
            if let CfgValue::Bool(v) = entry.value {
                ui.set_keyboard_map_en(v);
            }
        }
        // comm.keyboard_map_serial_only 走通用配置组(协议页 mai2serial 块)回填,
        // 不再需要专属属性 —— 见 protocol.slint 内注释。
    }
    last_config_page_visible = config_page_visible;

    // 绑定区 34 格只在实际展示输入变化时更新：实时触摸用位掩码，配置/Cp/绑定进度用各自版本。
    // 禁止直接以 telem_version 重建，否则 16ms tick 会反复格式化 Cp/标签并替换 Slint 行模型。
    let zone_key = (
        ctrl.config_version(),
        ctrl.cp_version(),
        ctrl.active_ch_mask(),
        ctrl.bind_progress(),
        ctrl.interactive_bind_active(),
    );
    if last_zone_key.as_ref() != Some(&zone_key) {
        last_zone_key = Some(zone_key);
        let zones = build_zone_cells(&ctrl);
        for (row, zone) in zones.into_iter().enumerate() {
            if zones_model.row_data(row).as_ref() != Some(&zone) {
                zones_model.set_row_data(row, zone);
            }
        }
    }

    let bind_status = match ctrl.bind_progress() {
        Some((zone, _status)) if ctrl.interactive_bind_active() =>
            format!("交互式绑定: 请触摸 区{} ({}/34)", zone_label(zone as usize), zone + 1),
        Some((zone, status)) => format!("侦听: 区{} 状态={}", zone_label(zone as usize), status),
        None => "就绪".to_string(),
    };
    ui.set_bind_status(bind_status.into());
    ui.set_interactive_active(ctrl.interactive_bind_active());

    // 通道切换立即刷新该通道参数(单条 PARAM_GET_ALL)；参数请求使用独立边沿游标，
    // 且连接探针期间不发送，避免首批遥测/绘图尚未建立时每 tick 误判为切换。
    if current_channel != last_param_request_channel && connected && !conn_probes_pending {
        let _ = ctrl.request_params(current_channel.clamp(0, 35) as u8);
        last_param_request_channel = current_channel;
    }

    // 批量抽屉源通道的参数回读(与上面切通道走同一条 PARAM_GET_ALL)。★为什么放在 tick★
    // 选源那一刻设备可能正忙, 请求会被静默丢掉; 由本处按拍冲刷待办, 忙就下一拍再试。
    // 每个源通道最多发一条, 详见 AppController::batch_request_source_params。
    if connected && !conn_probes_pending {
        ctrl.batch_request_source_params();
    }
}
