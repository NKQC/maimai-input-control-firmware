//! mai2control-ui 上位机主程序 (#6e~g 实现)
//!
//! 职责:
//! - 初始化 Slint UI 框架与应用状态控制器(AppController)
//! - 绑定 UI 回调到 AppController 方法,在事件循环中同步 UI 属性
//! - 周期性轮询 IO 事件与构建派生数据(配置行、绑区单元、曲线路径)
//! - 响应配置、绑区、遥测、参数操作及重启/进烧录模式指令

use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use log::info;
use anyhow::Result;

use mai2control_ui::app_state::{AppController, ConnState, zone_label};
use mai2control_ui::proto::{ConfigEntry, CfgValue, FIELD_RAW, FIELD_BASELINE, FIELD_DIFF, FIELD_STATUS, PARAM_FINGER_TH, PARAM_NOISE_TH};

slint::include_modules!();

const CP_MEASURE_FAILED: u32 = 0x00FF_FFFF;

struct CpPollState {
    started_at: Option<Instant>,
    next_request_at: Option<Instant>,
    measurement_start_channel_version: u64,
    requested_channel_version: u64,
    visible_channel: i32,
    visible_after_version: u64,
    waiting_for_response: bool,
    status: String,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    info!("mai2control-ui starting");

    let ui = AppWindow::new().map_err(|e| anyhow::anyhow!("Failed to create UI: {}", e))?;

    let controller = Rc::new(RefCell::new(AppController::new()));

    // ★关键★: 必须持有 Timer 直到 ui.run() 结束。slint::Timer 一旦 drop 即停止,
    // 若让它在 setup_ui_callbacks 内作为局部变量被回收,轮询循环会立刻停摆——
    // 表现为 UI 永不处理 DEVICE_INFO/遥测(连不上、无数据、功能全失效)。
    let _timer = setup_ui_callbacks(&ui, controller.clone());

    ui.run().map_err(|e| anyhow::anyhow!("UI run failed: {}", e))?;

    info!("mai2control-ui exiting");
    Ok(())
}

fn setup_ui_callbacks(ui: &AppWindow, controller: Rc<RefCell<AppController>>) -> slint::Timer {
    // 初始化设备列表
    {
        let mut ctrl = controller.borrow_mut();
        ctrl.refresh_devices();
        let labels: Vec<slint::SharedString> = ctrl
            .device_labels()
            .into_iter()
            .map(|s| s.into())
            .collect();
        ui.set_device_labels(slint::ModelRc::new(slint::VecModel::from(labels)));
        // 自动连接:检测到设备即连第一个,用户无需手动点连接。
        if ctrl.device_count() > 0 {
            let _ = ctrl.connect(0);
        }
    }

    let ui_weak = ui.as_weak();
    let cp_poll = Rc::new(RefCell::new(CpPollState {
        started_at: None,
        next_request_at: None,
        measurement_start_channel_version: 0,
        requested_channel_version: 0,
        visible_channel: -1,
        visible_after_version: 0,
        waiting_for_response: false,
        status: "读取中…".to_string(),
    }));

    // 刷新按钮
    let ctrl_clone = controller.clone();
    let ui_refresh = ui_weak.clone();
    ui.on_refresh(move || {
        let ui = ui_refresh.upgrade().unwrap();
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.refresh_devices();
        let labels: Vec<slint::SharedString> = ctrl
            .device_labels()
            .into_iter()
            .map(|s| s.into())
            .collect();
        ui.set_device_labels(slint::ModelRc::new(slint::VecModel::from(labels)));
    });

    // 连接
    let ctrl_clone = controller.clone();
    let ui_conn = ui_weak.clone();
    ui.on_connect_clicked(move || {
        let ui = ui_conn.upgrade().unwrap();
        let index = ui.get_selected_device() as usize;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.connect(index);
    });

    // 断开
    let ctrl_clone = controller.clone();
    ui.on_disconnect_clicked(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.disconnect();
    });

    // 重启设备(新增)
    let ctrl_clone = controller.clone();
    ui.on_reboot_device(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reboot();
    });

    // 进入烧录模式(新增)
    let ctrl_clone = controller.clone();
    ui.on_reboot_bootloader(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reboot_bootloader();
    });

    // 配置页
    let ctrl_clone = controller.clone();
    ui.on_cfg_load(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.request_config_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_save(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.save_config();
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_reset(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reset_defaults();
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_set_bool(move |key, value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config(ConfigEntry::new(key.to_string(), CfgValue::Bool(value)));
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_set_number(move |key, value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config_number(&key, value as f64);
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_set_enum(move |key, index| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config_enum(&key, index);
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_set_string(move |key, value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config(ConfigEntry::new(key.to_string(), CfgValue::Str(value.to_string())));
    });

    // 绑区页
    let ctrl_clone = controller.clone();
    ui.on_bind_load(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.request_config_all();
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

    // 详情面板 SpinBox 编辑通道号:直接写入新语义 bind.mapNN(=物理通道索引)。
    let ctrl_clone = controller.clone();
    ui.on_zone_channel_set(move |zone_idx, channel| {
        if zone_idx < 0 || zone_idx as usize >= 34 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_binding_channel(zone_idx as usize, channel as u8);
    });

    // "指触绑定":发起 BIND_START,固件检测下一个触摸通道并回写绑定。
    let ctrl_clone = controller.clone();
    ui.on_bind_touch_start(move |zone_idx| {
        if zone_idx < 0 || zone_idx as usize >= 34 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.bind_start(zone_idx as u8);
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

    // 曲线页
    let ctrl_clone = controller.clone();
    ui.on_telem_start(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.start_telemetry(100, FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS, 0xFFFFFFFF_FFFFFFFFu64);
    });

    let ctrl_clone = controller.clone();
    ui.on_telem_stop(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.stop_telemetry();
    });

    // 从全通道状态卡进入精调：选中物理通道并切换到“单通道精调”子标签。
    let ui_channel = ui_weak.clone();
    ui.on_channel_selected(move |channel| {
        let ui = ui_channel.upgrade().unwrap();
        ui.set_sel_channel(channel.clamp(0, 35));
        ui.set_settings_tab(2);
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
        let ch = ui.get_sel_channel() as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.baseline_reset(1u64 << ch);
    });

    let ctrl_clone = controller.clone();
    let ui_th = ui_weak.clone();
    ui.on_threshold_set(move |param_id, value| {
        let ui = ui_th.upgrade().unwrap();
        let ch = ui.get_sel_channel() as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_param(ch, param_id as u8, value as u32);
    });

    let ctrl_clone = controller.clone();
    let ui_param = ui_weak.clone();
    ui.on_curve_param_edited(move |param_id, value| {
        let ui = ui_param.upgrade().unwrap();
        let ch = ui.get_sel_channel() as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_param(ch, param_id as u8, value as u32);
    });

    let ctrl_clone = controller.clone();
    ui.on_curve_param_apply_all(move |param_id, value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_param_all(param_id as u8, value as u32);
    });

    let ctrl_clone = controller.clone();
    let ui_measure = ui_weak.clone();
    let cp_poll_measure = cp_poll.clone();
    ui.on_curve_measure_cp(move || {
        let ui = ui_measure.upgrade().unwrap();
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let mut state = cp_poll_measure.borrow_mut();
        match ctrl.measure_cp() {
            Ok(()) => {
                let now = Instant::now();
                state.started_at = Some(now);
                state.next_request_at = Some(now + Duration::from_millis(500));
                state.measurement_start_channel_version = ctrl.cp_channel_version(ch);
                state.requested_channel_version = state.measurement_start_channel_version;
                state.visible_channel = ch as i32;
                state.visible_after_version = state.measurement_start_channel_version;
                state.waiting_for_response = false;
                state.status = "测量中…".to_string();
            }
            Err(error) => {
                state.started_at = None;
                state.next_request_at = None;
                state.waiting_for_response = false;
                state.status = format!("测量失败: {}", error);
            }
        }
        ui.set_cp_text(state.status.clone().into());
    });

    // CSD 模式切换(0=自动校准, 1=半自动手动)。半自动下手动阈值/参数才持久。
    let ctrl_clone = controller.clone();
    ui.on_curve_mode_changed(move |mode| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_mode(mode as u8);
    });

    // 从设备捕获当前(自整定)全部参数入 RP2040 store，作为半自动手动调参起点。
    let ctrl_clone = controller.clone();
    ui.on_curve_capture_params(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.csd_capture();
    });

    // 触控键盘映射开关 (Phase 1 占位):CFG_SET(comm.keyboard_map_en)
    let ctrl_clone = controller.clone();
    ui.on_set_keyboard_map_en(move |value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config(ConfigEntry::new("comm.keyboard_map_en".to_string(), CfgValue::Bool(value)));
    });

    // 延迟测量开关只控制 UI 显示，固件始终低成本采样。
    let measure_on = Rc::new(std::cell::Cell::new(false));
    let measure_on_c = measure_on.clone();
    ui.on_latency_measure_toggled(move |enabled| {
        measure_on_c.set(enabled);
        info!("latency measure {}", enabled);
    });

    // 主事件循环
    let mut last_config_version = 0u64;
    let mut last_telem_version = 0u64;
    // 用非初始值确保即使尚未收到遥测，也先把完整 36 通道卡片回填到 UI。
    let mut last_telem_version_all = u64::MAX;
    let mut last_param_version = 0u64;
    let mut last_channel = -1i32;
    let mut last_curve_visibility = (false, false, false);
    let mut reconnect_tick = 0u32;
    let mut was_connected = false;
    let mut last_log_seq = u64::MAX;
    let measure_on_timer = measure_on.clone();
    let cp_poll_timer = cp_poll.clone();

    let timer = slint::Timer::default();
    timer.start(slint::TimerMode::Repeated, std::time::Duration::from_millis(16), move || {
        let ui = ui_weak.upgrade().unwrap();
        let mut ctrl = controller.borrow_mut();
        ctrl.poll();

        // 自动重连:断开且有设备时每~2s 刷新并重连第一个,用户无需手动连接。
        reconnect_tick = reconnect_tick.wrapping_add(1);
        if ctrl.state() == ConnState::Disconnected && reconnect_tick % 125 == 0 {
            ctrl.refresh_devices();
            let labels: Vec<slint::SharedString> =
                ctrl.device_labels().into_iter().map(|s| s.into()).collect();
            ui.set_device_labels(slint::ModelRc::new(slint::VecModel::from(labels)));
            if ctrl.device_count() > 0 {
                let _ = ctrl.connect(0);
            }
        }

        ui.set_conn_status(ctrl.status_line().into());
        ui.set_device_info_text(ctrl.device_info_text().into());

        // 连接/收发/错误 事件日志回填 UI 日志面板(仅在有新日志时刷新)。
        if ctrl.log_seq() != last_log_seq {
            last_log_seq = ctrl.log_seq();
            ui.set_log_text(ctrl.log_text().into());
        }

        // 连上设备后自动拉全部配置(绑定/参数)并开启全通道遥测,使 UI 立即有数据;
        // 连接期间每~1s PING 保活,维持设备侧"主机已连接"绿灯常亮。
        let connected = ctrl.state() == ConnState::Connected;
        if connected && !was_connected {
            ctrl.push_log("已连接 → 自动请求配置 + 开启全通道遥测");
            let _ = ctrl.request_config_all();
            let _ = ctrl.start_telemetry(
                100,
                FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS,
                0xFFFFFFFF_FFFFFFFFu64,
            );
        }
        if connected && reconnect_tick % 63 == 0 {
            let _ = ctrl.ping();
        }
        was_connected = connected;

        let current_config_version = ctrl.config_version();
        if current_config_version != last_config_version {
            last_config_version = current_config_version;
            let entries = ctrl.config_entries();
            let rows = build_config_rows(&entries);
            ui.set_config_rows(slint::ModelRc::new(slint::VecModel::from(rows)));

            if let Some(entry) = ctrl.config_get("comm.keyboard_map_en") {
                if let CfgValue::Bool(v) = entry.value {
                    ui.set_keyboard_map_en(v);
                }
            }
        }

        let zones = build_zone_cells(&ctrl);
        ui.set_zones(slint::ModelRc::new(slint::VecModel::from(zones)));

        let bind_status = match ctrl.bind_progress() {
            Some((zone, status)) => format!("进行中: 区{} 状态={}", zone, status),
            None => "就绪".to_string(),
        };
        ui.set_bind_status(bind_status.into());

        let current_channel = ui.get_sel_channel();
        let current_telem_version = ctrl.telem_version();
        let current_param_version = ctrl.param_version();
        let channel_changed = current_channel != last_channel;
        let curve_visibility = (ui.get_show_raw(), ui.get_show_bsln(), ui.get_show_diff());
        let curve_visibility_changed = curve_visibility != last_curve_visibility;

        // 通道切换立即刷新参数和 Cp；每通道版本门控确保不会展示前一通道/旧请求的缓存。
        {
            let now = Instant::now();
            let ch = current_channel.clamp(0, 35) as u8;
            let mut cp_state = cp_poll_timer.borrow_mut();

            if channel_changed {
                cp_state.visible_channel = current_channel;
                cp_state.visible_after_version = ctrl.cp_channel_version(ch);
                cp_state.status = "读取中…".to_string();
                if cp_state.started_at.is_some() {
                    cp_state.measurement_start_channel_version = cp_state.visible_after_version;
                    cp_state.waiting_for_response = false;
                }
                let _ = ctrl.request_params(ch);
                if let Err(error) = ctrl.request_cp(ch) {
                    cp_state.status = format!("读取失败: {}", error);
                }
            }

            let was_measuring = cp_state.started_at.is_some();
            if let Some(started_at) = cp_state.started_at {
                if now.duration_since(started_at) >= Duration::from_secs(10) {
                    cp_state.started_at = None;
                    cp_state.next_request_at = None;
                    cp_state.waiting_for_response = false;
                    cp_state.visible_after_version = ctrl.cp_channel_version(ch);
                    cp_state.status = "测量失败/超时".to_string();
                } else if cp_state.next_request_at.is_some_and(|deadline| now >= deadline) {
                    cp_state.requested_channel_version = ctrl.cp_channel_version(ch);
                    match ctrl.request_cp(ch) {
                        Ok(()) => {
                            cp_state.waiting_for_response = true;
                            cp_state.next_request_at = Some(now + Duration::from_millis(500));
                        }
                        Err(error) => {
                            cp_state.started_at = None;
                            cp_state.next_request_at = None;
                            cp_state.waiting_for_response = false;
                            cp_state.status = format!("测量失败: {}", error);
                        }
                    }
                }

                let response_version = ctrl.cp_channel_version(ch);
                if cp_state.waiting_for_response
                    && response_version > cp_state.requested_channel_version
                    && response_version > cp_state.measurement_start_channel_version
                {
                    match ctrl.cp(ch) {
                        Some(CP_MEASURE_FAILED) => {
                            cp_state.started_at = None;
                            cp_state.next_request_at = None;
                            cp_state.waiting_for_response = false;
                            cp_state.visible_after_version = response_version;
                            cp_state.status = "测量失败".to_string();
                        }
                        Some(value) if value != 0 => {
                            cp_state.started_at = None;
                            cp_state.next_request_at = None;
                            cp_state.waiting_for_response = false;
                            cp_state.visible_after_version = response_version;
                            cp_state.status = format!("Cp: {} fF（测量成功）", value);
                        }
                        _ => {
                            cp_state.status = "测量中…".to_string();
                        }
                    }
                }
            }

            if !was_measuring
                && cp_state.visible_channel == current_channel
                && ctrl.cp_channel_version(ch) > cp_state.visible_after_version
            {
                cp_state.visible_after_version = ctrl.cp_channel_version(ch);
                cp_state.status = match ctrl.cp(ch) {
                    Some(CP_MEASURE_FAILED) => "Cp: 测量失败".to_string(),
                    Some(value) => format!("Cp: {} fF", value),
                    None => "读取中…".to_string(),
                };
            }
            ui.set_cp_text(cp_state.status.clone().into());
        }

        if current_telem_version != last_telem_version || channel_changed || curve_visibility_changed {
            last_telem_version = current_telem_version;
            last_channel = current_channel;
            last_curve_visibility = curve_visibility;
            let curves = build_curve_paths(
                &ctrl,
                current_channel as u8,
                curve_visibility.0,
                curve_visibility.1,
                curve_visibility.2,
            );
            let finger_th = ctrl.param(current_channel as u8, PARAM_FINGER_TH).unwrap_or(0) as f32;
            let noise_th = ctrl.param(current_channel as u8, PARAM_NOISE_TH).unwrap_or(0) as f32;
            let finger_th_y = curves.value_to_y(finger_th);
            let noise_th_y = curves.value_to_y(noise_th);
            ui.set_raw_path(curves.raw_path.into());
            ui.set_bsln_path(curves.bsln_path.into());
            ui.set_diff_path(curves.diff_path.into());
            ui.set_curve_y_min(curves.y_min);
            ui.set_curve_y_mid(curves.y_mid);
            ui.set_curve_y_max(curves.y_max);
            ui.set_curve_point_count(curves.point_count);
            ui.set_finger_th_y(finger_th_y);
            ui.set_noise_th_y(noise_th_y);
        }

        if current_param_version != last_param_version || channel_changed {
            last_param_version = current_param_version;
            let params = ctrl.params_of(current_channel as u8);
            let param_rows = build_param_rows(&params);
            ui.set_curve_params(slint::ModelRc::new(slint::VecModel::from(param_rows)));

            if let Some(finger_th) = ctrl.param(current_channel as u8, PARAM_FINGER_TH) {
                ui.set_finger_th_val(finger_th as i32);
            }
            if let Some(noise_th) = ctrl.param(current_channel as u8, PARAM_NOISE_TH) {
                ui.set_noise_th_val(noise_th as i32);
            }
        }

        // 全通道实时状态：随遥测版本刷新(遥测流全 36 通道)。active=status bit0。
        if current_telem_version != last_telem_version_all {
            last_telem_version_all = current_telem_version;
            let all = build_channel_status(&ctrl);
            ui.set_all_channels(slint::ModelRc::new(slint::VecModel::from(all)));
        }

        // 采样率/通道刷新延迟：每 tick 回填(与遥测帧到达节奏一致，STATS 字段随 TELEM_DATA 更新)。
        ui.set_sample_rate_hz(ctrl.telem_samples_per_sec() as i32);
        ui.set_channel_delay_us(ctrl.telem_scan_period_us() as i32);

        if measure_on_timer.get() {
            let spi = ctrl.telem_lat_spi_us() as i32;
            let proc = ctrl.telem_lat_proc_us() as i32;
            let usb = ctrl.telem_lat_usb_us() as i32;
            ui.set_latency_spi_us(spi);
            ui.set_latency_proc_us(proc);
            ui.set_latency_usb_us(usb);
            ui.set_latency_sensor_us(ctrl.telem_scan_period_us() as i32);
            ui.set_latency_total_us(spi + proc + usb);
        }
    });

    // 返回 Timer,由 main() 持有到 ui.run() 结束,防止被 drop 而停止轮询。
    timer
}

fn build_config_rows(entries: &[ConfigEntry]) -> Vec<ConfigRow> {
    entries
        .iter()
        .map(|entry| {
            let (kind, type_code, bool_val, num_val, min_val, max_val, has_range, enum_index, str_val) = match &entry.value {
                CfgValue::Bool(v) => (0, 0, *v, 0.0, 0.0, 1.0, false, 0, "".to_string()),
                CfgValue::U8(v) => (1, 2, false, *v as f32, 0.0, 255.0, true, 0, "".to_string()),
                CfgValue::U16(v) => (1, 3, false, *v as f32, 0.0, 65535.0, true, 0, "".to_string()),
                CfgValue::U32(v) => (1, 4, false, *v as f32, 0.0, 4294967295.0, true, 0, "".to_string()),
                CfgValue::I8(v) => (1, 1, false, *v as f32, -128.0, 127.0, true, 0, "".to_string()),
                CfgValue::F32(v) => (1, 5, false, *v, 0.0, 1.0, false, 0, "".to_string()),
                CfgValue::Str(v) => (3, 6, false, 0.0, 0.0, 0.0, false, 0, v.clone()),
            };

            let (group, label) = parse_config_label(&entry.key);

            ConfigRow {
                key: entry.key.clone().into(),
                label: label.into(),
                group: group.into(),
                kind,
                type_code,
                bool_val,
                num_val,
                min_val,
                max_val,
                has_range,
                enum_index,
                str_val: str_val.into(),
            }
        })
        .collect()
}

fn parse_config_label(key: &str) -> (String, String) {
    let parts: Vec<&str> = key.split('.').collect();
    if parts.len() < 2 {
        return ("其他".to_string(), key.to_string());
    }

    let group = match parts[0] {
        "comm" => "通信".to_string(),
        "mode" => "模式".to_string(),
        "light" => "灯效".to_string(),
        "bind" => "绑区".to_string(),
        _ => "其他".to_string(),
    };

    let label = match parts[1] {
        "rate_limit_hz" => "速率限制(Hz)".to_string(),
        "work" => "工作模式".to_string(),
        "brightness" => "亮度".to_string(),
        k => k.replace('_', " "),
    };

    (group, label)
}

/// 按 maimai 圆形分区几何计算分区 `index`(0..33)的圆心归一化坐标
/// (画布 0..1000,中心 500,500)。
/// 环映射: A1-8=0-7(外环r430), B1-8=8-15(内环r240), C1-2=16-17(中心),
/// D1-8=18-25(外环r430), E1-8=26-33(内环r240)。
/// 角度(度,0=正右,顺时针,屏幕 y 向下): D_k = -90+(k-1)*45;
/// A_k(与 B_k 同角度) = -90+22.5+(k-1)*45(即 A 在相邻两 D 之间);
/// E_k 角度同 D_k。C1=(560,500),C2=(440,500)。
fn zone_geometry(index: usize) -> (f32, f32) {
    const CENTER: f32 = 500.0;
    let deg_to_rad = std::f32::consts::PI / 180.0;

    let (angle_deg, r): (f32, f32) = match index {
        // A1-8: index 0..7, k = index+1
        0..=7 => {
            let k = (index - 0) as f32 + 1.0;
            (-90.0 + 22.5 + (k - 1.0) * 45.0, 430.0)
        }
        // B1-8: index 8..15, k = index-8+1
        8..=15 => {
            let k = (index - 8) as f32 + 1.0;
            (-90.0 + 22.5 + (k - 1.0) * 45.0, 240.0)
        }
        // C1-2: index 16,17,直接给定坐标(不走三角函数)
        16 => return (560.0, 500.0),
        17 => return (440.0, 500.0),
        // D1-8: index 18..25, k = index-18+1
        18..=25 => {
            let k = (index - 18) as f32 + 1.0;
            (-90.0 + (k - 1.0) * 45.0, 430.0)
        }
        // E1-8: index 26..33, k = index-26+1
        26..=33 => {
            let k = (index - 26) as f32 + 1.0;
            (-90.0 + (k - 1.0) * 45.0, 240.0)
        }
        _ => (0.0, 0.0),
    };

    let angle_rad = angle_deg * deg_to_rad;
    (CENTER + r * angle_rad.cos(), CENTER + r * angle_rad.sin())
}

fn build_zone_cells(ctrl: &AppController) -> Vec<ZoneCell> {
    let mut cells = Vec::with_capacity(34);
    for i in 0..34usize {
        let label = zone_label(i);
        let ring = label.chars().next().unwrap_or('?').to_string();
        let ch = ctrl.binding_channel_of(i);
        let channel = if ch == 0xFF { -1 } else { ch as i32 };
        let touched = channel >= 0
            && ctrl
                .telem_latest(channel as u8)
                .and_then(|s| s.status)
                .map(|status| (status & 0x01) != 0)
                .unwrap_or(false);
        let (cx, cy) = zone_geometry(i);

        cells.push(ZoneCell {
            index: i as i32,
            label: label.into(),
            ring: ring.into(),
            channel,
            touched,
            cx,
            cy,
        });
    }
    cells
}

struct CurvePaths {
    raw_path: String,
    bsln_path: String,
    diff_path: String,
    y_min: f32,
    y_mid: f32,
    y_max: f32,
    point_count: i32,
}

impl CurvePaths {
    fn value_to_y(&self, value: f32) -> f32 {
        if self.point_count == 0 {
            return 500.0;
        }
        (1000.0 - (value - self.y_min) / (self.y_max - self.y_min) * 1000.0)
            .clamp(0.0, 1000.0)
    }
}

fn build_curve_paths(
    ctrl: &AppController,
    ch: u8,
    show_raw: bool,
    show_bsln: bool,
    show_diff: bool,
) -> CurvePaths {
    let raw_series = if show_raw { ctrl.telem_series(ch, FIELD_RAW) } else { vec![] };
    let bsln_series = if show_bsln { ctrl.telem_series(ch, FIELD_BASELINE) } else { vec![] };
    let diff_series = if show_diff { ctrl.telem_series(ch, FIELD_DIFF) } else { vec![] };
    let all_series = [&raw_series[..], &bsln_series[..], &diff_series[..]];

    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    let mut point_count = 0usize;
    for series in all_series {
        point_count = point_count.max(series.len());
        for &value in series {
            if value.is_finite() {
                min = min.min(value);
                max = max.max(value);
            }
        }
    }

    if !min.is_finite() || !max.is_finite() {
        return CurvePaths {
            raw_path: String::new(),
            bsln_path: String::new(),
            diff_path: String::new(),
            y_min: 0.0,
            y_mid: 0.5,
            y_max: 1.0,
            point_count: 0,
        };
    }

    let span = max - min;
    let padding = if span.abs() < f32::EPSILON {
        (min.abs() * 0.05).max(1.0)
    } else {
        span * 0.05
    };
    let y_min = min - padding;
    let y_max = max + padding;
    let y_mid = (y_min + y_max) * 0.5;

    CurvePaths {
        raw_path: series_to_svg_path(&raw_series, y_min, y_max),
        bsln_path: series_to_svg_path(&bsln_series, y_min, y_max),
        diff_path: series_to_svg_path(&diff_series, y_min, y_max),
        y_min,
        y_mid,
        y_max,
        point_count: point_count as i32,
    }
}

fn series_to_svg_path(series: &[f32], min: f32, max: f32) -> String {
    if series.is_empty() {
        return String::new();
    }

    let n = series.len();
    let range = (max - min).max(f32::EPSILON);
    let mut path = String::new();

    for (i, &v) in series.iter().enumerate() {
        let x = (i as f32) * 1000.0 / ((n - 1).max(1) as f32);
        let y = (1000.0 - (v - min) / range * 1000.0).clamp(0.0, 1000.0);

        if i == 0 {
            path.push_str(&format!("M {} {}", x as i32, y as i32));
        } else {
            path.push_str(&format!(" L {} {}", x as i32, y as i32));
        }
    }

    path
}

fn build_channel_status(ctrl: &AppController) -> Vec<ChannelStatus> {
    let mut out = Vec::with_capacity(36);
    for ch in 0u8..36 {
        let (active, raw, diff) = match ctrl.telem_latest(ch) {
            Some(s) => (
                (s.status.unwrap_or(0) & 0x01) != 0,
                s.raw.unwrap_or(0) as i32,
                s.diff.unwrap_or(0) as i32,
            ),
            None => (false, 0, 0),
        };
        let bindings: Vec<String> = (0..34usize)
            .rev()
            .filter(|&zone| ctrl.binding_channel_of(zone) == ch)
            .map(zone_label)
            .collect();
        let binding_text = if bindings.is_empty() {
            "未绑定".to_string()
        } else {
            bindings.join(" ")
        };

        out.push(ChannelStatus {
            index: ch as i32,
            label: format!("CH{}", ch).into(),
            active,
            raw,
            diff,
            grid_col: (ch % 6) as i32,
            grid_row: (ch / 6) as i32,
            binding_text: binding_text.into(),
        });
    }
    out
}

fn build_param_rows(params: &[(u8, u32)]) -> Vec<ParamRow> {
    params
        .iter()
        .map(|&(param_id, value)| {
            let label = match param_id {
                0x01 => "手指阈值(PARAM_FINGER_TH)".to_string(),
                0x02 => "噪声阈值(PARAM_NOISE_TH)".to_string(),
                0x03 => "负阈值(PARAM_NEG_NOISE_TH)".to_string(),
                0x04 => "迟滞(PARAM_HYSTERESIS)".to_string(),
                0x05 => "按键消抖(PARAM_ON_DEBOUNCE)".to_string(),
                0x06 => "低基线复位(PARAM_LOW_BSLN_RST)".to_string(),
                0x07 => "分辨率(PARAM_RESOLUTION)".to_string(),
                0x08 => "传感时钟分频(PARAM_SNS_CLK_DIV)".to_string(),
                0x09 => "模态 IDAC(PARAM_IDAC_MOD)".to_string(),
                0x0A => "时钟源(PARAM_SNS_CLK_SOURCE)".to_string(),
                0x0B => "IDAC 增幅档(PARAM_IDAC_GAIN)".to_string(),
                _ => format!("参数 0x{:02X}", param_id),
            };

            ParamRow {
                param_id: param_id as i32,
                label: label.into(),
                value: value as i32,
            }
        })
        .collect()
}
