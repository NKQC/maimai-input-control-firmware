//! UI 主循环节拍编排。
//!
//! 保持单次 `AppController` 可变借用、硬件命令顺序和常驻 Slint 模型实例。

use super::*;

pub(super) struct TickInputs {
    pub(super) ui_weak: slint::Weak<AppWindow>,
    pub(super) controller: Rc<RefCell<AppController>>,
    pub(super) auto_reconnect: Rc<Cell<bool>>,
    pub(super) pending_cfg: Rc<RefCell<Vec<PendingCfgWrite>>>,
    pub(super) cp_poll: Rc<RefCell<CpSweep>>,
    pub(super) mai2_verify_at: Rc<Cell<Option<Instant>>>,
    pub(super) algo_default_src: &'static str,
    pub(super) algo_busy: Rc<Cell<bool>>,
    pub(super) algo_job: Rc<RefCell<Option<AlgoCompileJob>>>,
    pub(super) report_show: Rc<RefCell<[bool; 4]>>,
    pub(super) algo_report_lines_model: Rc<slint::VecModel<AlgoReportLine>>,
    pub(super) report_norm: Rc<RefCell<[f32; 5]>>,
    pub(super) ui_cfg: Rc<RefCell<mai2control_ui::ui_config::UiConfig>>,
    pub(super) log_dirty: Rc<Cell<bool>>,
    pub(super) la_window_idx: Rc<Cell<usize>>,
    pub(super) vcam_state: VirtualCameraCallbackState,
}

pub(super) fn start(ui: &AppWindow, inputs: TickInputs) -> slint::Timer {
    let TickInputs {
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
    } = inputs;

    let mut last_config_version = 0u64;
    // 主图独立版本: 冻结期间不随实时遥测推进，避免重复 SVG/path 重算；解冻时跳到最新缓冲。
    let mut last_plot_version = u64::MAX;
    // 用非初始值确保即使尚未收到遥测，也先把完整 36 通道卡片回填到 UI。
    let mut last_telem_version_all = u64::MAX;
    // Cp 缓存版本门控：无新遥测但有新 Cp 响应时也要刷新全通道卡片的 cp_text。
    let mut last_cp_version_all = u64::MAX;
    // 启用态(0x0C 草稿)与排序/筛选选择的门控: 三者任一变化都要重排/重筛全通道网格。
    let mut last_param_version_all = u64::MAX;
    let mut last_channel_telemetry_state = -1i32;
    let mut last_sort_all = -1i32;
    let mut last_show_disabled_all = false;
    let mut last_param_version = 0u64;
    // 绑定区模型仅由配置/Cp/触摸掩码/绑定进度等实际展示输入驱动，绝不由每帧遥测版本触发。
    let mut last_zone_key: Option<(u64, u64, u64, Option<(u8, u8)>, bool)> = None;
    let mut last_batch_sel_version = u64::MAX;
    // 噪声频谱格子模型的重建门控: 448 格每 tick 无条件重建纯属浪费, 只在扫描状态或当前选择通道变了才重建。
    // 结果固定归属其扫描通道；切到别的 CH 时必须清空模型，绝不把旧热图冒充新通道的数据。
    let mut last_spectrum_version = u64::MAX;
    let mut last_spectrum_channel = -1i32;
    let mut last_combo_version = u64::MAX;
    // 上次下发给 UI 的组合映射分区文本。★行身份稳定所需★: combo_zones_text 是已有映射列表
    // Repeater 的 model, 每次 set 新 ModelRc 都会重建全部 ComboRow —— 行内 KeyCaptureBox 的
    // 录制态与焦点随之丢失, 一次和弦只能录到第一个键(第二个键根本收不到)。就地重录改的是
    // keys_text, 分区文本并不变, 所以只在分区文本真的变了(增删条目/改分区)时才换 model。
    let mut last_combo_zones: Vec<slint::SharedString> = Vec::new();
    let mut last_channel = -1i32;
    // 参数回读边沿与绘图回填必须使用独立游标。绘图游标只有在 path 真正重建后更新；
    // 若复用它判断通道变化，首批遥测尚未到达时会每 tick 重发 PARAM_GET_ALL。
    let mut last_param_request_channel = -1i32;
    let mut last_curve_visibility = (false, false, false);
    // Cp 失败通道列表(形如 "CH0 CH5"); 只在内容变化时回填, 免得每 tick 都往 UI 写字符串。
    let mut last_cp_failed = String::new();
    // 算法叠加脏标记: 它会改变主图 path, 与遥测版本一起做门控(绘图区尺寸已不再参与, 见 fit: fill)。
    let mut algo_overlay_dirty = true;
    let mut reconnect_tick = 0u32;
    let mut was_connected = false;
    // 基础连接探针是否仍在收敛；下降沿只触发一次后置全通道启用态回读。
    let mut last_conn_probes_pending = false;
    // ★DEBUG_CRASH_BOOTSEL 武装已移除★: 每次连接自动武装导致任何主动重启都被误判为崩溃并进 BOOTSEL,
    // 现仅在显式需要时（selftest --arm-crash-bootsel）才手动武装，日常使用保持解除态（崩溃仅正常重启）。
    // 每个 USB 会话在基础探针收敛后同步一次"设备上真正跑着的算法"(C 源 + ASM)。
    let mut algo_sync_pending = false;
    // 每个 USB 会话在基础探针收敛后读一次"上次复位的死前遗言"并立即清除设备置位。
    // ★一次性★ 清除成功后本会话不再重复读取, 否则同一次崩溃会被反复告警。
    let mut crash_report_pending = false;
    // 冷启动首帧可能晚于基础探针数秒；在实际收到 TELEM_DATA 前禁止自愈重发，
    // 以免与固件 provisioning 的正常首帧窗口竞争。
    let mut telemetry_first_frame_seen = false;
    // mode.work 保存后自动重启倒计时(tick): 给 SAVE_CONFIG 的 flash 写留出完成窗口再重启重枚举。
    let mut reboot_countdown: Option<u32> = None;
    // "正在等待下发完成再重启"只提示一次, 避免每 tick 刷日志。
    let mut reboot_wait_logged = false;
    let mut last_log_ver = u64::MAX;
    let mut last_log_page_start = i32::MIN;
    let ui_cfg_tick = ui_cfg.clone();
    let mut last_algo_version = u64::MAX;
    let mut algo_info_filled = false;
    let mut last_algo_upload_version = 0u64;
    let mut last_algo_src_version = u64::MAX;
    let mut last_algo_code_version = u64::MAX;
    let mut last_algo_metadata_version = u64::MAX;
    // 默认算法的 C 源(已滤注释)是否已回灌到设备映射表: 设备默认算法出厂不带源,
    // 首次读到"默认+无源"时把内嵌默认源(去注释)下发设备一次, 使之后"读取信息"能真正从设备取回。
    let mut default_src_synced = false;
    let mut last_algo_asm_version = u64::MAX;
    // 算法运行值(report[]/out_active, 随帧到达)/可调变量(cfg[8]): 重建面板的版本门控。
    let mut last_algo_trace_version = u64::MAX;
    let mut last_algo_metadata_trace_version = u64::MAX;
    let mut last_algo_cfg_version = u64::MAX;
    // 逐通道算法配置(cfg_ch)的重建门控。★与 cfg 分开一个游标★: 两组值来自两条独立的回读/草稿路径,
    // 合用一个游标会让改共享项也重建逐通道行(反之亦然), 白丢一次编辑焦点。
    let mut last_algo_cfg_ch_version = u64::MAX;
    // 批量抽屉里逐通道算法行的 schema 门控: 换算法后声明项会变, 而批量块本身只看 batch/param 版本。
    let mut last_batch_algo_schema_version = u64::MAX;
    // 算法容量(设备回报)回填门控: 容量是不变量, 只在 algo_version 推进时核对一次即可。
    let mut last_algo_caps_known = false;
    // 自动载入编辑器的那份文本: 用于判断编辑器是否已被用户改过(改过就不再自动覆盖)。
    // ★初值必须等于启动时预置进编辑器的那份模板★: 否则"编辑器 == 自动载入值"恒不成立,
    // 会把开机预置的模板当成"用户的改动"而永不载入设备算法源 —— 表现为 JIT 算法从不自动同步显示。
    let mut editor_autoload_mark = algo_default_src.to_string();

    let mut last_globals_version = u64::MAX;
    let mut last_lat_version = u64::MAX;
    let mut last_global_tune_visible = false;
    // 全通道网格进页边沿: 用于批量回读通道启用开关(见 connection_protocol.rs 里的说明)。
    let mut last_all_channels_visible = false;
    let mut last_algo_page_visible = false;
    let mut last_config_page_visible = false;
    let mut last_hid_page_visible = false;
    let mut last_hid_point_key: Option<(u64, u64)> = None;
    let mut last_hid_mode = false;
    let cp_poll_timer = cp_poll.clone();

    // 持久化的 36 通道模型: 每 tick 原地 set_row_data 更新, 绝不整体替换 model。
    // 整体替换会让 Slint 销毁并重建所有 cell 元素 → 悬浮 has-hover 丢失后重获(闪烁),
    // 且重建期吞掉点击/双击事件(无法进精调)。原地更新保留元素身份, 消除闪烁与交互丢失。
    let all_channels_model: Rc<slint::VecModel<ChannelStatus>> =
        Rc::new(slint::VecModel::from(vec![ChannelStatus::default(); 36]));
    ui.set_all_channels(slint::ModelRc::from(all_channels_model.clone()));

    // 这三组可交互行模型在整个 UI 生命周期内保持同一实例；tick 只原地更新变化行。
    let zones_model: Rc<slint::VecModel<ZoneCell>> = Rc::new(slint::VecModel::from(
        build_zone_cells(&controller.borrow()),
    ));
    ui.set_zones(slint::ModelRc::from(zones_model.clone()));
    let hid_points_model: Rc<slint::VecModel<HidPointCell>> = Rc::new(slint::VecModel::from(
        build_hid_point_cells(&controller.borrow()),
    ));
    ui.set_hid_points(slint::ModelRc::from(hid_points_model.clone()));
    let curve_params_model: Rc<slint::VecModel<ParamRow>> = Rc::new(slint::VecModel::from(
        build_param_rows(&controller.borrow().params_of(0)),
    ));
    ui.set_curve_params(slint::ModelRc::from(curve_params_model.clone()));
    // cfg[8] 编辑行与参数行同样保持模型实例；版本回写只更新变化行，绝不销毁正在编辑的控件。
    let algo_setting_rows_model: Rc<slint::VecModel<AlgoSettingRow>> =
        Rc::new(slint::VecModel::from(Vec::new()));
    let algo_metadata_rows_model: Rc<slint::VecModel<AlgoMetadataRow>> =
        Rc::new(slint::VecModel::from(Vec::new()));
    // 逐通道算法配置(cfg_ch[8])的编辑行: 与共享 cfg 行**分成两个模型**而不是一个模型加个标记位。
    // 理由是它们的值口径不同 —— 共享行的值只有一份, 逐通道行的值随"当前精调通道"变。混在一个
    // 模型里, 单通道页切通道时会把共享行也一起重建(丢焦点), 而分开后只需重建逐通道那几行。
    let algo_setting_ch_rows_model: Rc<slint::VecModel<AlgoSettingRow>> =
        Rc::new(slint::VecModel::from(Vec::new()));
    // 批量抽屉里那一份逐通道算法配置行: 值取自 batch_source 且可手改(见 batch_algo_ch_values),
    // 与上面单通道精调那一份是**两组不同的值**, 因此也必须是两个模型。
    let batch_algo_ch_rows_model: Rc<slint::VecModel<AlgoSettingRow>> =
        Rc::new(slint::VecModel::from(Vec::new()));
    ui.set_algo_setting_rows(slint::ModelRc::from(algo_setting_rows_model.clone()));
    ui.set_algo_metadata_rows(slint::ModelRc::from(algo_metadata_rows_model.clone()));
    ui.set_algo_setting_ch_rows(slint::ModelRc::from(algo_setting_ch_rows_model.clone()));
    ui.set_batch_algo_ch_rows(slint::ModelRc::from(batch_algo_ch_rows_model.clone()));
    // ★这一行原先漏了★ 上报折线模型在 ui_callbacks/algo.rs 里建好、在 tick/chart.rs 里被逐行
    // 填满(每帧 4 行), 却从没绑到 Slint 的 `algo_report_lines` 属性上 —— 该属性因此永远是默认的
    // 空数组。后果有两处, 都与"算法里声明了什么"无关:
    //   · 图例区 `visible: algo_report_lines.length == 0` 恒真 ⇒ 永远显示"当前算法未声明
    //     ALGO_REPORT, 无上报变量", 哪怕 C 源里明明解析出 4 条声明;
    //   · `for line in root.algo_report_lines: PlotPath` 迭代空模型 ⇒ 右轴上报折线整组从不绘制。
    // 与上面两个模型一样, 必须绑定"同一个实例"而不是每次刷新换 ModelRc(换实例会重建行元素)。
    ui.set_algo_report_lines(slint::ModelRc::from(algo_report_lines_model.clone()));
    // 批量面板的"待写值"一列同样必须常驻同一实例。★为什么★ 原先每次刷新都 `set_batch_param_rows`
    // 换掉整个 ModelRc, Repeater 随之重建全部行元素 ⇒ 用户正在编辑的那个 SpinBox 连元素一起消失,
    // 表现为"打一个字就失焦"(与 GuardedSpinBox 的焦点守卫无关 —— 守卫救不了被销毁的元素)。
    // 常驻 + 逐行 set_row_data 后, 值没变的行连数据都不动, 编辑中的行也不会被重建。
    // 行清单唯一来源 = `proto::BATCH_PARAM_IDS`(分辨率是全局项, 故不在其中)。
    let batch_params_model: Rc<slint::VecModel<ParamRow>> = Rc::new(slint::VecModel::from(
        BATCH_PARAM_IDS
            .iter()
            .map(|id| build_param_row(*id, -1))
            .collect::<Vec<ParamRow>>(),
    ));
    ui.set_batch_param_rows(slint::ModelRc::from(batch_params_model.clone()));

    // 键盘键码下拉的共享键名表(一次性设置)。
    let kbd_choice_names: Vec<slint::SharedString> =
        kbd_key_choices().iter().map(|(n, _)| (*n).into()).collect();
    ui.set_kbd_key_choices(slint::ModelRc::new(slint::VecModel::from(kbd_choice_names)));
    // 触控分区名 A1-E8(一次性设置), 供触控键盘映射页每行标签。
    let kbd_zone_name_list: Vec<slint::SharedString> =
        (0..34u8).map(|z| zone_label(z as usize).into()).collect();
    ui.set_kbd_zone_names(slint::ModelRc::new(slint::VecModel::from(
        kbd_zone_name_list,
    )));

    let mut last_kbd_state_version = u64::MAX;
    let mut last_kbd_map_version = u64::MAX;
    let mut last_kbd_touchmap_version = u64::MAX;
    let mut last_kbd_hold_version = u64::MAX;
    let mut last_kbd_keycfg_version = u64::MAX;
    let mut last_la_version = u64::MAX;
    let mut last_la_window = usize::MAX;
    let mut last_phys_kbd_visible = false;

    // 逻辑分析仪时间窗下拉项(一次性回填, 与 LA_WINDOWS_US 同序)。
    {
        let names: Vec<slint::SharedString> = LA_WINDOWS_US
            .iter()
            .map(|us| slint::SharedString::from(fmt_time_us(*us as f64)))
            .collect();
        ui.set_la_window_names(slint::ModelRc::new(slint::VecModel::from(names)));
        ui.set_la_window_index(LA_WINDOW_DEFAULT as i32);
    }
    let mut last_mai2_version = u64::MAX;
    let mai2_verify_timer = mai2_verify_at.clone();
    let mut last_led_version = u64::MAX;
    // 灯板协议单元行同样必须常驻同一实例。★为什么★ 协议页驻留时会周期性回读 LED 状态,
    // 每收到一次 LED_GET 响应就 bump 一次 led_version(周期约 1s)。原先每次 bump 都
    // `set_light_units` 换掉整个 ModelRc, Repeater 随之重建全部行元素 ⇒ 用户正在操作的
    // 「通道」下拉/「起始」「数量」数字框连元素一起被销毁, 表现为"选中约 1 秒后自动失焦"。
    // 常驻 + 逐行 set_row_data 后, 值没变的行连数据都不动, 正在编辑的行也不会被重建。
    let light_units_model: Rc<slint::VecModel<LedUnitRow>> = Rc::new(slint::VecModel::from(
        build_led_unit_rows(&controller.borrow()),
    ));
    ui.set_light_units(slint::ModelRc::from(light_units_model.clone()));
    // 协议页驻留门控: 进页边沿请求一次, 离页停止轮询(见下方 protocol_visible)。
    let mut last_protocol_visible = false;

    let report_show_timer = report_show.clone();
    // 后台编译任务槽与 busy 闸: tick 里取回产物后由本处解锁(见"后台编译产物回收")。
    let algo_job_timer = algo_job.clone();
    let algo_busy_timer = algo_busy.clone();
    let algo_report_lines_model_timer = algo_report_lines_model.clone();
    let algo_setting_rows_model_timer = algo_setting_rows_model.clone();
    let algo_metadata_rows_model_timer = algo_metadata_rows_model.clone();
    let algo_setting_ch_rows_model_timer = algo_setting_ch_rows_model.clone();
    let batch_algo_ch_rows_model_timer = batch_algo_ch_rows_model.clone();
    let report_norm_timer = report_norm.clone();
    let vcam_timer = vcam_state.vcam.clone();
    let publisher_timer = vcam_state.frame_publisher.clone();
    let mut last_vcam_frame_version = 0u32;
    let mut last_vcam_runtime_status = String::new();
    // 上一次向共享队列发布的时刻: 定频 10fps 的节拍源(见 io_vcam.rs)。
    // None = 还没发过, 首拍立即发, 不让消费端多等一个周期。
    let mut last_vcam_publish: Option<Instant> = None;
    // 发布失败是否已记过日志: 10fps 下同一错误每秒会复现十次, 只在状态翻转时记录。
    let mut vcam_publish_failed = false;
    // ★tick 剖面(每 ~2s 一条 WARN)★: "卡顿"必须用实测归因, 不能靠猜。分别累计整帧、IO 排空、
    // 主图重建、36 卡片重建的耗时与命中次数; 峰值单独记, 因为卡顿是峰值现象而非均值现象。
    let mut prof_ticks = 0u32;
    let mut prof_tick_us = 0u64;
    let mut prof_tick_max_us = 0u64;
    let mut prof_poll_us = 0u64;
    let mut prof_chart_us = 0u64;
    let mut prof_chart_hits = 0u32;
    let mut prof_chart_max_us = 0u64;
    let mut prof_cards_us = 0u64;
    let mut prof_cards_hits = 0u32;
    // tick 间隔: 16ms 定时器的实际到达节奏。回调自身很短却间隔很大 ⇒ 时间花在事件循环/渲染上,
    // 而不是我们的数据处理里 —— 这是区分"逻辑慢"与"渲染慢"的唯一直接判据。
    let mut prof_last_tick: Option<Instant> = None;
    let mut prof_gap_us = 0u64;
    let mut prof_gap_max_us = 0u64;
    let mut prof_gaps = 0u32;
    let pending_cfg_tick = pending_cfg.clone();
    let timer = slint::Timer::default();
    timer.start(
        slint::TimerMode::Repeated,
        Duration::from_millis(UI_TICK_MS as u64),
        move || {
            let prof_tick_start = Instant::now();
            if let Some(previous) = prof_last_tick.replace(prof_tick_start) {
                let gap = prof_tick_start.duration_since(previous).as_micros() as u64;
                prof_gap_us += gap;
                prof_gap_max_us = prof_gap_max_us.max(gap);
                prof_gaps += 1;
            }
            let ui = ui_weak.upgrade().unwrap();
            let mut ctrl = controller.borrow_mut();

            // 跨功能域的本拍快照：各 include 文件只消费这些值，不复制控制器状态。
            let current_channel = ui.get_sel_channel();
            let current_telem_version = ctrl.telem_version();
            let current_plot_version = ctrl.plot_version();
            let current_param_version = ctrl.param_version();
            let channel_changed = current_channel != last_channel;
            let curve_visibility = (ui.get_show_raw(), ui.get_show_bsln(), ui.get_show_diff());
            let curve_visibility_changed = curve_visibility != last_curve_visibility;
            let visual_refresh_tick = _every_ms(reconnect_tick, VISUAL_REFRESH_MS);
            let connected = ctrl.state() == ConnState::Connected;
            let conn_probes_pending = ctrl.conn_probes_pending();
            let hid_mode = ctrl.work_mode() == Some(1);

            include!("io_vcam.rs");
            include!("connection_status.rs");
            include!("logging.rs");
            include!("algorithm.rs");
            include!("global_tune.rs");
            include!("connection_protocol.rs");
            include!("configuration.rs");
            include!("capacitance.rs");
            include!("chart.rs");
            include!("models.rs");
            include!("keyboard.rs");
            include!("protocol.rs");
            include!("health.rs");
            include!("profiler.rs");
        },
    );

    // 由 main() 持有到 ui.run() 结束，防止 Timer 被 drop。
    timer
}
