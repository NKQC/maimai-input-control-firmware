//! 触控、绑区、曲线与工具箱端口 UI 回调。

use super::super::*;

pub(crate) struct TouchCallbackState {
    pub(crate) cp_poll: Rc<RefCell<CpSweep>>,
    pub(crate) mai2_verify_at: Rc<Cell<Option<Instant>>>,
}

pub(crate) fn register_touch_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    ui_weak: &slint::Weak<AppWindow>,
) -> TouchCallbackState {
    let cp_poll = Rc::new(RefCell::new(CpSweep {
        started_at: None,
        next_at: None,
        ch: 0,
        tries: 0,
        req_version: 0,
        recovery_telem_version: None,
        recovery_started_at: None,
        recovery_detail: String::new(),
        status: "未测量".to_string(),
    }));
    let mai2_verify_at = Rc::new(Cell::new(None::<Instant>));

    // 全局页 CSD 采样设置：统一写入全部 36 个物理通道。
    let ctrl_clone = controller.clone();
    ui.on_csd_param_set(move |param_id, value| {
        if param_id < 0 || value < 0 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        // AUTO 下这些值由 PSoC 接管；UI 已锁定，这里再守住异步旧事件。
        // 模式判定唯一来源(含"未知也不放行"): AppController::csd_mode_effective。
        if ctrl.csd_mode_effective() != Some(1) {
            return;
        }
        let _ = ctrl.set_param_all(param_id as u8, value as u32);
    });

    // ★原 on_csd_refresh(「刷新 CH0 参数」按钮)已删★: 全局页的 CH0 代表值本就有两条自动加载
    // 路径 —— 连接边沿(见下方 `connected && !was_connected` 块的 request_params(0))与每次真正
    // 进入本页(`global_tune_visible` 边沿)各拉一次, 手动按钮拿不到任何额外真值。

    // 绑区页
    let ctrl_clone = controller.clone();
    ui.on_bind_load(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.request_config_all();
    });

    let ctrl_clone = controller.clone();
    let mai2_verify_set = mai2_verify_at.clone();
    ui.on_mai2_set_send_en(move |enabled| {
        let mut ctrl = ctrl_clone.borrow_mut();
        if ctrl.mai2_set_send_en(enabled).is_ok() {
            mai2_verify_set.set(Some(Instant::now() + Duration::from_millis(250)));
        }
    });

    let ctrl_clone = controller.clone();
    ui.on_mai2_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.mai2_request_state();
    });

    super::led::register_led_callbacks(ui, controller);

    // 选中分区:回填分区名与当前绑定通道(-1=未映射),供详情面板 SpinBox 显示。
    let ctrl_clone = controller.clone();
    ui.on_hid_point_commit(move |channel, x, y| {
        if !(0..HID_POINT_COUNT as i32).contains(&channel) {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.hid_point_commit(
            channel as usize,
            x.clamp(0, HID_COORD_MAX as i32) as u16,
            y.clamp(0, HID_COORD_MAX as i32) as u16,
        );
    });

    let ctrl_clone = controller.clone();
    ui.on_hid_point_enabled_set(move |channel, enabled| {
        if !(0..HID_POINT_COUNT as i32).contains(&channel) {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_hid_point_enabled(channel as usize, enabled);
    });

    let ui_capture = ui_weak.clone();
    ui.on_hid_screenshot_refresh(move || {
        if let Some(ui) = ui_capture.upgrade() {
            refresh_hid_screenshot(&ui);
        }
    });

    // 选中分区:回填分区名与当前绑定通道(-1=未映射),供详情面板 SpinBox 显示。
    let ctrl_clone = controller.clone();
    let ui_zone = ui_weak.clone();
    ui.on_zone_selected(move |zone_idx| {
        let ui = ui_zone.upgrade().unwrap();
        let ctrl = ctrl_clone.borrow();
        let label = zone_label(zone_idx as usize);
        let ch = ctrl.binding_channel_of(zone_idx as usize);
        ui.set_selected_zone_label(label.into());
        ui.set_selected_channel(if ch == 0xFF { -1 } else { ch as i32 });
    });

    // 画布和列表的双击都进入此处：仅已绑定分区才能切到其真实物理通道精调。
    let ctrl_clone = controller.clone();
    let ui_zone_activated = ui_weak.clone();
    ui.on_zone_activated(move |zone_idx| {
        if !(0..34).contains(&zone_idx) {
            return;
        }
        let channel = ctrl_clone.borrow().binding_channel_of(zone_idx as usize);
        if channel != 0xFF {
            let ui = ui_zone_activated.upgrade().unwrap();
            ui.set_sel_channel(channel as i32);
            // Tab 顺序: 0绑区 1协议 2触控通道 3触控全局 4单通道精调 …(协议页插到索引 1 后全部后移一位)
            ui.set_settings_tab(SETTINGS_PAGE_CURVES);
        }
    });

    // 详情面板 SpinBox 编辑通道号:直接写入新语义 bind.mapNN(=物理通道索引)。
    let ctrl_clone = controller.clone();
    ui.on_zone_channel_set(move |zone_idx, channel| {
        if zone_idx < 0 || zone_idx as usize >= 34 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_binding_channel(zone_idx as usize, channel as u8);
    });

    // "侦听绑定":上位机侦听下一次触摸的物理通道(遥测上升沿)并写入绑定草稿,
    // 不发 BIND_START、不改设备运行态; 点击保存后草稿才真正下发生效。
    let ctrl_clone = controller.clone();
    ui.on_bind_touch_start(move |zone_idx| {
        if zone_idx < 0 || zone_idx as usize >= 34 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.listen_start(zone_idx as u8);
    });

    // 取消侦听。
    let ctrl_clone = controller.clone();
    ui.on_bind_listen_cancel(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.listen_cancel();
    });

    // 交互式顺序绑定(等效 v3.0): 开始 / 终止。
    let ctrl_clone = controller.clone();
    ui.on_interactive_bind(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.interactive_bind_start();
    });
    let ctrl_clone = controller.clone();
    ui.on_interactive_bind_cancel(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.listen_cancel();
    });

    // 精确命中检测: 画布把点击像素换算到 SCREEN 坐标系后调用, 返回分区 index(未命中 -1)。
    // 用 point-in-polygon 取代旧 34 个重叠 bbox, 消除 A/D/E 等相邻区误选。
    ui.on_zone_hit_test(move |x, y| {
        touch_geometry::hit_test(x, y)
            .map(|i| i as i32)
            .unwrap_or(-1)
    });

    // "清除":写回未映射(0xFF -> 0xFFFFFFFF)。
    let ctrl_clone = controller.clone();
    ui.on_zone_unbind(move |zone_idx| {
        if zone_idx < 0 || zone_idx as usize >= 34 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_binding_channel(zone_idx as usize, 0xFF);
    });

    // 曲线页: 两个回调只管理绘图快照，绝不改变遥测接收状态。
    let ctrl_clone = controller.clone();
    ui.on_plot_resume(move || {
        ctrl_clone.borrow_mut().plot_unfreeze();
    });

    let ctrl_clone = controller.clone();
    ui.on_plot_freeze(move || {
        ctrl_clone.borrow_mut().plot_freeze("用户暂停画面");
    });

    // 从全通道状态卡进入精调：选中物理通道并切换到“单通道精调”子标签(索引 4)。
    let ui_channel = ui_weak.clone();
    ui.on_channel_selected(move |channel| {
        let ui = ui_channel.upgrade().unwrap();
        ui.set_sel_channel(channel.clamp(0, 35));
        ui.set_settings_tab(SETTINGS_PAGE_CURVES);
    });

    let ctrl_clone = controller.clone();
    let ui_calib = ui_weak.clone();
    ui.on_curve_calibrate(move || {
        let ui = ui_calib.upgrade().unwrap();
        let ch = ui.get_sel_channel() as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.calibrate(1u64 << ch);
    });

    let ctrl_clone = controller.clone();
    let ui_bsln = ui_weak.clone();
    ui.on_curve_baseline_reset(move || {
        let ui = ui_bsln.upgrade().unwrap();
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.baseline_reset(1u64 << ch) {
            ctrl.push_log_warn(format!("CH{} 基线复位失败: {}", ch, e));
        }
    });

    // 单通道响应噪声频谱扫描: 开始 / 取消(都只作用于当前精调通道)。
    let ctrl_clone = controller.clone();
    let ui_sweep = ui_weak.clone();
    ui.on_curve_spectrum_start(move || {
        let ui = ui_sweep.upgrade().unwrap();
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.noise_sweep_start(ch) {
            ctrl.push_log_warn(format!("频谱扫描未能开始: {}", e));
        }
    });

    let ctrl_clone = controller.clone();
    ui.on_curve_spectrum_cancel(move || {
        ctrl_clone.borrow_mut().noise_sweep_cancel();
    });

    let ctrl_clone = controller.clone();
    let ui_th = ui_weak.clone();
    ui.on_threshold_set(move |param_id, value| {
        let ui = ui_th.upgrade().unwrap();
        let ch = ui.get_sel_channel() as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        if ctrl.csd_mode_effective() != Some(1) {
            return;
        }
        let _ = ctrl.set_param(ch, param_id as u8, value as u32);
    });

    let ctrl_clone = controller.clone();
    let ui_param = ui_weak.clone();
    ui.on_curve_param_edited(move |param_id, value| {
        let ui = ui_param.upgrade().unwrap();
        let ch = ui.get_sel_channel() as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        if ctrl.csd_mode_effective() != Some(1) {
            return;
        }
        let _ = ctrl.set_param(ch, param_id as u8, value as u32);
    });

    let ctrl_clone = controller.clone();
    let ui_measure = ui_weak.clone();
    let cp_poll_measure = cp_poll.clone();
    // ★唯一的 Cp 获取入口★: 手动"测量电容" = 触发一次全电极 BIST 测量 + 一轮 36 通道顺序抓取。
    // 除此之外任何路径(连接、切换通道、恢复默认、每 tick)都不再请求 Cp。
    ui.on_curve_measure_cp(move || {
        let ui = ui_measure.upgrade().unwrap();
        let mut ctrl = ctrl_clone.borrow_mut();
        let mut state = cp_poll_measure.borrow_mut();
        match ctrl.measure_cp() {
            Ok(()) => {
                let now = Instant::now();
                state.started_at = Some(now);
                state.next_at = Some(now + CP_SWEEP_START_DELAY);
                state.ch = 0;
                state.tries = 0;
                state.req_version = 0;
                state.recovery_telem_version = None;
                state.recovery_started_at = None;
                state.recovery_detail.clear();
                state.status = "测量中…".to_string();
                ctrl.push_log("测量电容: 已触发全电极 BIST, 将顺序回读 36 通道 Cp(仅本次)");
            }
            Err(error) => state._stop(format!("测量失败: {}", error)),
        }
        ui.set_cp_text(state.status.clone().into());
    });

    // CSD 模式切换(0=自动校准/标准完整处理, 1=半自动手动)。半自动下手动阈值/参数才持久。
    let ctrl_clone = controller.clone();
    ui.on_curve_mode_changed(move |mode| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_mode(mode as u8);
    });

    // 从设备捕获当前全部参数入 RP2040 store，作为半自动手动调参起点。
    let ctrl_clone = controller.clone();
    ui.on_curve_capture_params(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.csd_capture();
    });

    // 触控键盘映射开关 (Phase 1 占位):CFG_SET(comm.keyboard_map_en)
    let ctrl_clone = controller.clone();
    ui.on_set_keyboard_map_en(move |value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config(ConfigEntry::new(
            "comm.keyboard_map_en".to_string(),
            CfgValue::Bool(value),
        ));
    });

    // 工具箱端口设置:编辑后立即写入本地 toolbox.cfg，应用时回填状态文本。
    let ctrl_clone = controller.clone();
    ui.on_set_auto_port_enabled(move |enabled| {
        ctrl_clone.borrow_mut().set_toolbox_auto_port(enabled);
    });

    let ctrl_clone = controller.clone();
    ui.on_set_serial_com(move |port| {
        if let Ok(port) = u16::try_from(port) {
            ctrl_clone.borrow_mut().set_toolbox_serial_com(port);
        }
    });

    let ctrl_clone = controller.clone();
    ui.on_set_light_com(move |port| {
        if let Ok(port) = u16::try_from(port) {
            ctrl_clone.borrow_mut().set_toolbox_light_com(port);
        }
    });

    let ctrl_clone = controller.clone();
    let ui_ports = ui_weak.clone();
    ui.on_apply_ports(move || {
        // 用户点"立即应用" → force=true: 即使已是目标口也强制重启端口节点使其真正生效。
        let status = { ctrl_clone.borrow_mut().apply_ports(true) };
        if let Some(ui) = ui_ports.upgrade() {
            ui.set_port_status(status.into());
        }
    });

    TouchCallbackState {
        cp_poll,
        mai2_verify_at,
    }
}
