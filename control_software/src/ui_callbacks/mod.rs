//! UI 回调注册与主循环节拍。
//!
//! 所有 Slint 回调和 5ms Timer 在此集中注册，保持原有注册顺序、
//! 捕获生命周期及 `Rc<RefCell<AppController>>` 线程边界不变。

use super::*;

mod algo;
mod batch;
mod config;
mod connection;
mod keyboard;
mod keyboard_combo;
mod led;
#[path = "log.rs"]
mod log_callbacks;
mod tick;
mod touch;
mod virtual_camera;
use algo::{register_algo_callbacks, register_algo_late_callbacks};
use batch::register_batch_callbacks;
use config::register_config_callbacks;
use connection::register_connection_callbacks;
use keyboard::register_keyboard_callbacks;
use keyboard_combo::register_combo_callbacks;
use log_callbacks::register_log_callbacks;
use touch::register_touch_callbacks;
use virtual_camera::{VirtualCameraCallbackState, register_callbacks as register_vcam_callbacks};

pub(crate) fn setup_ui_callbacks(
    ui: &AppWindow,
    controller: Rc<RefCell<AppController>>,
) -> slint::Timer {
    let settings_counts = mai2control_ui::settings_io::group_counts(&controller.borrow());
    ui.set_settings_io_config_count(settings_counts.config);
    ui.set_settings_io_channel_params_count(settings_counts.channel_params);
    ui.set_settings_io_globals_count(settings_counts.globals);
    ui.set_settings_io_algo_count(settings_counts.algo);
    ui.set_settings_io_keyboard_count(settings_counts.keyboard);
    ui.set_settings_io_zones_count(settings_counts.zones);

    // 初始化设备列表，并回填/按需应用工具箱端口设置。
    let (auto_port_enabled, serial_com, light_com, port_status) = {
        let mut ctrl = controller.borrow_mut();
        ctrl.refresh_devices();
        let labels: Vec<slint::SharedString> =
            ctrl.device_labels().into_iter().map(|s| s.into()).collect();
        ui.set_device_labels(slint::ModelRc::new(slint::VecModel::from(labels)));
        // 自动连接:检测到设备即连第一个,用户无需手动点连接。
        if ctrl.device_count() > 0 {
            let _ = ctrl.connect(0);
        }
        (
            ctrl.toolbox_auto_port(),
            ctrl.toolbox_serial_com(),
            ctrl.toolbox_light_com(),
            ctrl.maybe_auto_assign_ports(),
        )
    };
    ui.set_auto_port_enabled(auto_port_enabled);
    ui.set_serial_com(serial_com as i32);
    ui.set_light_com(light_com as i32);
    if let Some(status) = port_status {
        ui.set_port_status(status.into());
    }

    let vcam_state = VirtualCameraCallbackState::new(ui);
    // 权限状态 + 一键提权重启: Windows 不能给已运行进程提权, 只能以管理员重开自身。
    ui.set_is_elevated(mai2control_ui::elevation::is_elevated());
    ui.set_elevation_text(mai2control_ui::elevation::limitation_text().into());
    ui.on_restart_as_admin(move || {
        // 先把界面偏好与日志落盘, 否则新实例读不到本次的改动、日志缓冲也会丢。
        mai2control_ui::logging::hub().flush();
        match mai2control_ui::elevation::relaunch_as_admin() {
            Ok(()) => {
                log::info!("已请求以管理员重启, 当前实例退出(避免两个实例抢 WinUSB 句柄)");
                mai2control_ui::logging::hub().flush();
                let _ = slint::quit_event_loop();
            }
            Err(e) => log::warn!("提权重启未执行: {}", e),
        }
    });

    virtual_camera::initialize(ui, &controller, &vcam_state);

    let ui_weak = ui.as_weak();

    // 延迟图不再依赖绘图区宽高比：PlotPath 的 fit: fill 会把固定 1000×1000 数据坐标拉满.

    // 手动断开后停止后台枚举和重连；只有用户再次主动连接才恢复掉线重连。
    let auto_reconnect = Rc::new(std::cell::Cell::new(true));

    register_connection_callbacks(ui, &controller, &ui_weak, &auto_reconnect);

    register_config_callbacks(ui, &controller);

    register_combo_callbacks(ui, &controller);

    let pending_cfg = register_batch_callbacks(ui, &controller);

    let algo_state = register_algo_callbacks(ui, &controller, &ui_weak);
    register_algo_late_callbacks(ui, &controller, &ui_weak, &algo_state);
    let la_window_idx = register_keyboard_callbacks(ui, &controller, &ui_weak);

    let touch_state = register_touch_callbacks(ui, &controller, &ui_weak);
    let cp_poll = touch_state.cp_poll;
    let mai2_verify_at = touch_state.mai2_verify_at;

    // 算法编译状态由算法回调域集中持有，tick 仅取用同一实例。
    let algo_default_src = algo_state.algo_default_src;
    let algo_busy = algo_state.algo_busy.clone();
    let algo_job = algo_state.algo_job.clone();
    let report_show = algo_state.report_show.clone();
    let algo_report_lines_model = algo_state.algo_report_lines_model.clone();

    // 编译容量(PSoC 可执行槽字节数)一次性回填, 供进度条计算占用百分比。
    ui.set_algo_asm_capacity(AppController::algo_slot_capacity() as i32);

    // 分区图绘制视框 = 分区几何并集 bbox, 一次性回填, 使圆形分区图缩放铺满画布(去 16:9 留白)。
    {
        let (vx, vy, vw, vh) = touch_geometry::content_bbox();
        ui.set_zone_view_x(vx);
        ui.set_zone_view_y(vy);
        ui.set_zone_view_w(vw);
        ui.set_zone_view_h(vh);
    }

    // ---------------- 界面偏好(config.cfg) ----------------
    // 纯界面设置(日志等级/自动滚动/落盘开关/虚拟摄像头时长/延迟测量/当前页签与通道)重启即丢,
    // 每次开程序都要重新点一遍。这里在启动目录的 config.cfg 里存取, 与"会改系统状态"的
    // toolbox.cfg(固定 COM 号、扫码器设备)分开, 避免界面偏好牵连到系统级设置。
    let ui_cfg = Rc::new(RefCell::new(mai2control_ui::ui_config::UiConfig::load()));
    {
        use mai2control_ui::ui_config::keys as k;
        let cfg = ui_cfg.borrow();
        ui.set_log_filter(cfg.get_i32(k::LOG_FILTER, 2).clamp(0, 3));
        ui.set_log_auto_scroll(cfg.get_bool(k::LOG_AUTO_SCROLL, true));
        ui.set_vcam_enabled(cfg.get_bool(k::VCAM_ENABLED, false));
        ui.set_vcam_submit_secs(cfg.get_i32(k::VCAM_SUBMIT_SECS, 2).clamp(1, 10));
        ui.set_vcam_display_secs(cfg.get_i32(k::VCAM_DISPLAY_SECS, 10).clamp(1, 60));
        ui.set_vcam_mirror_x(cfg.get_bool(k::VCAM_MIRROR_X, false));
        ui.set_vcam_frame_w(
            cfg.get_i32(k::VCAM_FRAME_W, vcam::DEFAULT_FRAME_W as i32)
                .clamp(vcam::MIN_FRAME_W as i32, vcam::MAX_FRAME_W as i32),
        );
        ui.set_vcam_frame_h(
            cfg.get_i32(k::VCAM_FRAME_H, vcam::DEFAULT_FRAME_H as i32)
                .clamp(vcam::MIN_FRAME_H as i32, vcam::MAX_FRAME_H as i32),
        );
        ui.set_vcam_qr_fill_pct(
            cfg.get_i32(k::VCAM_QR_FILL_PCT, vcam::DEFAULT_QR_FILL_PCT as i32)
                .clamp(vcam::MIN_QR_FILL_PCT as i32, vcam::MAX_QR_FILL_PCT as i32),
        );
        ui.set_diag_expanded(cfg.get_bool(k::DIAG_EXPANDED, false));
        ui.set_curve_params_expanded(cfg.get_bool(k::CURVE_PARAMS_EXPANDED, true));
        ui.set_curve_algo_cfg_expanded(cfg.get_bool(k::CURVE_ALGO_CFG_EXPANDED, false));
        ui.set_curve_series_expanded(cfg.get_bool(k::CURVE_SERIES_EXPANDED, true));
        ui.set_curve_spectrum_expanded(cfg.get_bool(k::CURVE_SPECTRUM_EXPANDED, false));
        ui.set_channel_show_disabled(cfg.get_bool(k::CHANNEL_SHOW_DISABLED, false));
        ui.set_phys_live_expanded(cfg.get_bool(k::PHYS_LIVE_EXPANDED, true));
        ui.set_phys_keys_expanded(cfg.get_bool(k::PHYS_KEYS_EXPANDED, false));
        ui.set_phys_la_expanded(cfg.get_bool(k::PHYS_LA_EXPANDED, false));
        ui.set_mai2_panel_expanded(cfg.get_bool(k::MAI2_PANEL_EXPANDED, false));
        ui.set_light_panel_expanded(cfg.get_bool(k::LIGHT_PANEL_EXPANDED, false));
        ui.set_measure_latency(cfg.get_bool(k::LATENCY_MEASURE, false));
        // ★环境变量可强制初始页签★(MAI2_UI_VIEW / MAI2_UI_TAB)
        // 界面类缺陷必须能"打开就在那一页"地复现与截图, 否则每次验证都要人工点几下, 无法自动化取证
        // (本轮排"协议页参数一片空白"时就卡在这一步: 记忆值总把程序开在主页)。
        // 不设置时行为完全不变, 仍用持久化的记忆值。
        let env_i32 = |name: &str| -> Option<i32> {
            std::env::var(name).ok().and_then(|s| s.trim().parse().ok())
        };
        ui.set_current_view(
            env_i32("MAI2_UI_VIEW")
                .unwrap_or_else(|| cfg.get_i32(k::CURRENT_VIEW, 0))
                .clamp(0, 3),
        );
        // 上限 8: 稳定 page id 0..8；HID 模式只隐藏协议并在动态 TabWidget 侧转换索引。
        ui.set_settings_tab(
            env_i32("MAI2_UI_TAB")
                .unwrap_or_else(|| cfg.get_i32(k::SETTINGS_TAB, 0))
                .clamp(0, 8),
        );
        ui.set_sel_channel(cfg.get_i32(k::SEL_CHANNEL, 0).clamp(0, 35));
        // 曲线页"画哪些系列": 默认值必须与 curves.slint 的属性默认值一致(raw/bsln 关, diff 开,
        // 触发判定开, 算法叠加关), 否则首次启动会被"记忆"改掉初始观感。
        ui.set_show_raw(cfg.get_bool(k::CURVE_SHOW_RAW, false));
        ui.set_show_bsln(cfg.get_bool(k::CURVE_SHOW_BSLN, false));
        ui.set_show_diff(cfg.get_bool(k::CURVE_SHOW_DIFF, true));
        ui.set_show_active(cfg.get_bool(k::CURVE_SHOW_ACTIVE, true));
        ui.set_show_algo_overlay(cfg.get_bool(k::CURVE_ALGO_OVERLAY, false));
        {
            // 4 条上报线的勾选真相源在 Rust 侧(见 report_show 的说明), 回填进那个 Rc 即可 ——
            // 行模型下一 tick 由主图回填统一重建, 复选框自然显示恢复后的状态。
            let mut show = report_show.borrow_mut();
            for (idx, slot) in show.iter_mut().enumerate() {
                *slot = cfg.get_bool(&report_show_key(idx), true);
            }
        }
        // 恢复后的等级要同时作用于文件与控制台。
        let lv = ui.get_log_filter().clamp(0, 3) as u8;
        controller.borrow_mut().set_log_filter(lv);
        mai2control_ui::logging::hub().set_level(lv);
        // 虚拟摄像头的秒数直接进共享状态, 否则要等用户动一次 SpinBox 才生效。
        vcam_state
            .vcam
            .set_submit_timeout_ms((ui.get_vcam_submit_secs().max(1) as u32) * 1000);
        vcam_state
            .vcam
            .set_display_ms((ui.get_vcam_display_secs().max(1) as u32) * 1000);
        // 镜像同理: 只回填 UI 属性而不落到共享状态, 出画那边会一直用默认朝向, 直到用户
        // 手动切一次开关才对上 —— 那正是"设置看起来生效了其实没生效"的经典形态。
        vcam_state.vcam.set_mirror_x(ui.get_vcam_mirror_x());
        // 分辨率与占比同理: 只回填 UI 而不落到共享状态, 出画就会一直用默认值。
        // 回填夹取后的实际生效值, 免得界面显示一个从未生效过的数。
        let (w, h) = vcam_state
            .vcam
            .set_resolution(ui.get_vcam_frame_w().max(0) as u32, ui.get_vcam_frame_h().max(0) as u32);
        ui.set_vcam_frame_w(w as i32);
        ui.set_vcam_frame_h(h as i32);
        let pct = vcam_state
            .vcam
            .set_qr_fill_pct(ui.get_vcam_qr_fill_pct().max(0) as u32);
        ui.set_vcam_qr_fill_pct(pct as i32);
    }

    // 二值算法线的归一化幅度 N(索引 0..3 = report[idx], 4 = 触发判定)。
    // ★为什么需要★: 0/1 的布尔类上报与同一右轴上动辄上千的计数类上报共存时会被压成贴底的一条线,
    // 把 0/1 拉伸到 0..N 才看得见。★为什么每条线各自一个 N★: 各条二值线要去"贴"的邻居量纲不同
    // (有的邻居是几千的计数, 有的是几十的 permille), 共用一个 N 必然有一条不合适。
    // ★纯 UI 量★: 只影响本机画图, 存 config.cfg, 绝不下发设备(设备侧算法语义不能被显示偏好污染)。
    let report_norm = Rc::new(RefCell::new([1.0f32; 5]));
    {
        let cfg = ui_cfg.borrow();
        let mut norm = report_norm.borrow_mut();
        for idx in 0..5usize {
            // 千分之一整数存储: config.cfg 只有 i32 存取, 而 N 需要小于 1 的档位。
            norm[idx] = (cfg.get_i32(&algo_bin_amp_key(idx), 1000) as f32 / 1000.0)
                .clamp(ALGO_BIN_AMP_MIN, ALGO_BIN_AMP_MAX);
        }
    }
    let report_norm_cb = report_norm.clone();
    let ui_cfg_norm = ui_cfg.clone();
    let report_model_norm = algo_report_lines_model.clone();
    let ui_weak_norm = ui_weak.clone();
    ui.on_report_norm_set(move |idx, value| {
        if idx < 0 || idx as usize >= 5 {
            return;
        }
        let row = idx as usize;
        let amp = if value.is_finite() {
            value.clamp(ALGO_BIN_AMP_MIN, ALGO_BIN_AMP_MAX)
        } else {
            1.0
        };
        report_norm_cb.borrow_mut()[row] = amp;
        ui_cfg_norm
            .borrow_mut()
            .set_i32(&algo_bin_amp_key(row), (amp * 1000.0).round() as i32);
        // 立即回显钳制后的值(用户可能输了 0 或超界), 并原地改行, 不等下一次追踪刷新。
        if row < 4 {
            if let Some(mut line) = report_model_norm.row_data(row) {
                line.norm_amp = amp;
                report_model_norm.set_row_data(row, line);
            }
        } else if let Some(ui) = ui_weak_norm.upgrade() {
            ui.set_active_norm_amp(amp);
        }
    });

    let log_state = register_log_callbacks(ui, &controller, &ui_weak);
    let log_dirty = log_state.log_dirty;

    // 关于页: 上位机版本与仓库地址都是编译期常量, 一次性回填(不进 16ms tick)。
    ui.set_about_app_version(env!("CARGO_PKG_VERSION").into());
    ui.set_about_repo_url(REPO_URL.into());
    ui.on_copy_repo_url(
        move || match mai2control_ui::logging::copy_to_clipboard(REPO_URL) {
            Ok(()) => log::info!("已复制仓库地址到剪贴板: {}", REPO_URL),
            Err(e) => log::warn!("复制仓库地址失败: {}", e),
        },
    );

    register_vcam_callbacks(ui, &controller, &ui_weak, &vcam_state);

    // 延迟测量开关只控制 UI 显示，固件始终低成本采样。
    let measure_on = Rc::new(std::cell::Cell::new(false));
    let measure_on_c = measure_on.clone();
    ui.on_latency_measure_toggled(move |enabled| {
        measure_on_c.set(enabled);
        info!("latency measure {}", enabled);
    });

    tick::start(
        ui,
        tick::TickInputs {
            ui_weak,
            controller,
            auto_reconnect,
            pending_cfg,
            cp_poll,
            mai2_verify_at,
            algo_default_src,
            algo_busy,
            algo_job,
            report_show,
            algo_report_lines_model,
            report_norm,
            ui_cfg,
            log_dirty,
            la_window_idx,
            vcam_state,
        },
    )
}

/// CSD 全 36 通道预期探测周期(µs), 由代表通道 CH0 的**分辨率**解算 —— ★不含 SnsClk 分频★。
///
/// CSDv2 架构(已核 middleware cy_capsense_csd_v2.c + 真机实测): 子转换数 = 2^resolution / snsClkDiv,
/// 单子转换耗 snsClkDiv 个 ModClk → 换能总时长 = 2^resolution / ModClk, **snsClkDiv 相互抵消**。
/// 故 SnsClk 分频只改变传感激励频率(影响 Cp 灵敏度/抗噪), 不改变扫描时长; 真机 div 8→48 实测
/// 周期几乎不变(5917µs→6289µs)证实此点。改变探测周期的是分辨率(实测 res 8→10→12: 2932→3533→5917µs)。
///
/// 模型 period ≈ CHANNELS × (每通道固定开销 + 2^resolution / ModClk)。固定开销(传感切换/IMO 稳定/
/// IDAC/IsBusy 轮询/多频扫描)真机实测约 76µs/通道, 换能项 2^res/48MHz 与实测吻合。resolution 超出
/// 1..=20 返回 0 表示无法解算。
const _UI_PLOT_SECTION_MOVED: () = ();
