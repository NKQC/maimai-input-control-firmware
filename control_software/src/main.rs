//! mai2control-ui 上位机主程序 (#6e~g 实现)
// ★release 不带控制台窗口★: 日志已有自持可用的日志页 + logs/ 落盘, 那个类 cmd 的黑窗口
// 只会挡在界面前面且无法选择复制。debug 构建保留控制台便于开发时直接看 stderr。
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

//!
//! 职责:
//! - 初始化 Slint UI 框架与应用状态控制器(AppController)
//! - 绑定 UI 回调到 AppController 方法,在事件循环中同步 UI 属性
//! - 周期性轮询 IO 事件与构建派生数据(配置行、绑区单元、曲线路径)
//! - 响应配置、绑区、遥测、参数操作及重启/进烧录模式指令

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::{Duration, Instant};

use anyhow::Result;
use log::info;

use mai2control_ui::app_state::{
    AppController, ChBatchKind, CompiledAlgo, ConnState, SWEEP_DIVS, zone_label,
};
use mai2control_ui::proto::{
    BATCH_PARAM_IDS, CfgValue, ConfigEntry, FIELD_BASELINE, FIELD_DIFF, FIELD_LATENCY, FIELD_RAW,
    FIELD_STATS, FIELD_STATUS, PARAM_FINGER_TH, PARAM_NOISE_TH, PARAM_RESOLUTION,
    PARAM_SNS_CLK_DIV, PARAM_SNS_CLK_SOURCE,
};
use mai2control_ui::proto::{LED_CH_UNMAPPED, LED_PREVIEW_ALL, LED_UNIT_COUNT};
use mai2control_ui::touch_geometry;
use mai2control_ui::vcam::{self, FRAME_H, FRAME_W, VcamState};
use mai2control_ui::vcam::{backend as vcam_backend, share, share::FramePublisher};
use slint::Model;

slint::include_modules!();

/// 内置 v3.1 HDR 触控算法源(随程序打包), 供算法页"加载模板"按钮作参考/真实下发案例。
const ALGO_V31_TEMPLATE: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../psoc_firmware/algo/psoc_algo_default.c"
));

/// 纯"白灯演示"算法模板: 触摸即点亮白灯(out_active), 展示 JIT 算法对 PSoC 硬件的绝对可控性。
const ALGO_LED_DEMO_TEMPLATE: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../psoc_firmware/algo/psoc_algo_led_demo.c"
));

const CP_MEASURE_FAILED: u32 = 0x00FF_FFFF;

/// 项目仓库地址(关于页展示 + 一键复制到剪贴板)。
const REPO_URL: &str = "https://github.com/NKQC/project-mai2control.git";

/// 后台算法编译任务。
/// ★为什么要有这东西★: 编译一次要顺序阻塞跑 gcc/objcopy/nm/objdump 四个子进程, 首次还要解压
/// 18MB 内置工具链, 以前在 UI 线程里直接做 → 整个界面冻住数秒(点不动、不重绘)。现在只把 C 源
/// 这类纯数据搬进 std::thread, 产物经 channel 回到 UI 线程(16ms tick 取回)再写入 AppController ——
/// `Rc<RefCell<AppController>>` 不是 Send, 绝不能进后台线程。
struct AlgoCompileJob {
    rx: std::sync::mpsc::Receiver<Result<CompiledAlgo>>,
    /// 提交编译的源码: 产物回来后要连同它一起写入状态(留档/上传随附源)。
    src: String,
    /// 编译成功后是否续走上传(“编译并上传”一键流程)。
    upload_after: bool,
}

/// 起一次后台编译。同一时刻只允许一个任务(由调用处的 algo_busy 守门)。
fn spawn_algo_compile(slot: &Rc<RefCell<Option<AlgoCompileJob>>>, src: String, upload_after: bool) {
    let (tx, rx) = std::sync::mpsc::channel();
    let src_for_thread = src.clone();
    std::thread::spawn(move || {
        // 发送失败只可能是 UI 已退出, 此时无人关心结果, 忽略即可。
        let _ = tx.send(AppController::compile_blob(&src_for_thread));
    });
    *slot.borrow_mut() = Some(AlgoCompileJob {
        rx,
        src,
        upload_after,
    });
}

/// 生成与源码行数一致的行号列字符串("1\n2\n...\nN"), 供算法页行号 gutter。
fn line_numbers_for(text: &str) -> String {
    let n = text.lines().count().max(1);
    let mut s = String::with_capacity(n * 4);
    for i in 1..=n {
        if i > 1 {
            s.push('\n');
        }
        s.push_str(&i.to_string());
    }
    s
}

/// PSoC 逐电极 BIST 测量窗口: MEASURE_CP 受理后需约 1.5s 才有结果, 期间 CP_GET 恒回"测量中(0)"。
/// 单通道精调主图的**固定**横轴跨度。
///
/// ★为什么固定★ 原先左端取"缓冲里最老的样本", 于是跨度随缓冲填充、采样率变化、切通道而变:
/// 同一段波形的横向尺度每帧都在动(抖), 缩放倍数与滚动条比例也跟着动。固定跨度后横向 1 像素
/// 恒等于同一时长, 波形形状跨时间、跨通道都可比。
/// 遥测缓冲的保留时长必须 ≥ 本值(见 `app_state::TELEM_RETAIN_US`), 否则窗口左侧永远是空的。
const PLOT_WINDOW_US: u64 = 30_000_000;

/// UI 主节拍。★上限帧率就是这个数的倒数★: Slint 只在属性变脏后重绘, 而回填全在这个 tick 里,
/// 16ms 的旧值把整个界面钉死在 60fps。5ms ⇒ 200Hz 节拍, 高刷屏上才有 180fps 以上的余量。
const UI_TICK_MS: u32 = 5;
/// 视觉重投影(主图 SVG / 36 张通道卡片)的墙钟上限, 约 120Hz: 比屏幕快没有意义, 只会挤占节拍。
const VISUAL_REFRESH_MS: u32 = 8;

/// 把"每 N 毫秒做一次"表达成节拍判据。周期不足一个 tick 时退化为每 tick 都做。
fn _every_ms(tick: u32, period_ms: u32) -> bool {
    let period = (period_ms / UI_TICK_MS).max(1);
    tick % period == 0
}

const CP_SWEEP_START_DELAY: Duration = Duration::from_millis(1500);
/// 同一通道未出结果时的重试间隔。
const CP_SWEEP_RETRY: Duration = Duration::from_millis(400);
/// 同一通道最多请求次数: 到顶即跳过该通道, 绝不无限重试(NAK/无响应风暴的根因之一)。
const CP_SWEEP_MAX_TRIES: u8 = 3;
/// 整轮抓取硬超时: 到点即停, 不管还剩几个通道。
const CP_SWEEP_TIMEOUT: Duration = Duration::from_secs(20);

/// 固件恢复确认窗口：Cp 结果完成后必须看到新的遥测与扫描周期，不能用 XRES 掩盖恢复失败。
const CP_RECOVERY_TIMEOUT: Duration = Duration::from_secs(8);

/// 手动电容测量的一轮全通道抓取与固件恢复确认状态。
/// ★没有任何自动/周期性 Cp 获取★: 仅用户点击"测量电容"时置 started_at；结果收齐或抓取超时后
/// 仍保留会话，直到固件自行恢复后的新遥测证明扫描推进，绝不由 Host 静默脉冲 XRES。
struct CpSweep {
    started_at: Option<Instant>,
    next_at: Option<Instant>,
    ch: u8,
    tries: u8,
    req_version: u64,
    recovery_telem_version: Option<u64>,
    recovery_started_at: Option<Instant>,
    recovery_detail: String,
    status: String,
}

impl CpSweep {
    /// 结束本轮抓取/恢复确认并落最终文案。
    fn _stop(&mut self, status: String) {
        self.started_at = None;
        self.next_at = None;
        self.tries = 0;
        self.recovery_telem_version = None;
        self.recovery_started_at = None;
        self.recovery_detail.clear();
        self.status = status;
    }

    /// Cp 收集结束或超时后等待新遥测。基线版本保证旧快照不能冒充 BIST 恢复证据。
    fn _await_recovery(&mut self, telem_version: u64, detail: String) {
        self.next_at = None;
        self.recovery_telem_version = Some(telem_version);
        self.recovery_started_at = Some(Instant::now());
        self.recovery_detail = detail;
        self.status = "Cp 已完成，等待固件恢复后的新遥测与扫描周期…".to_string();
    }

    /// 推进到下一个通道(本通道已出结果或已放弃)。
    fn _advance(&mut self) {
        self.ch = self.ch.saturating_add(1);
        self.tries = 0;
        self.next_at = None;
    }
}

fn main() -> Result<()> {
    // 统一日志中枢: 全量记录进内存环形缓冲(供日志页) + 每次启动在 logs/ 下新建一份文件。
    // 取代 env_logger —— 它只往 stderr 打, UI 里看不到 io/nusb 层的掉线与端点错误。
    mai2control_ui::logging::init(true);
    info!("mai2control-ui starting");

    let ui = AppWindow::new().map_err(|e| anyhow::anyhow!("Failed to create UI: {}", e))?;

    // 全局项数字输入围栏: 常量, 只需回填一次(不进每 tick 的刷新循环)。
    ui.set_global_fences(build_global_fences());
    // 时钟树上的时钟源单选项同样是常量表, 一次回填。★与单通道精调共用同一份围栏派生★
    // (`ParamFence::ui_choices()`), 界面不自造标签, 也就不会漂移出第二套"128 是什么"的说法。
    {
        let choices = mai2control_ui::proto::param_fence(PARAM_SNS_CLK_SOURCE).ui_choices();
        let labels: Vec<slint::SharedString> =
            choices.iter().map(|(t, _)| t.as_str().into()).collect();
        let values: Vec<i32> = choices.iter().map(|(_, v)| *v as i32).collect();
        ui.set_csd_clk_labels(slint::ModelRc::new(slint::VecModel::from(labels)));
        ui.set_csd_clk_values(slint::ModelRc::new(slint::VecModel::from(values)));
    }

    let controller = Rc::new(RefCell::new(AppController::new()));

    // ★关键★: 必须持有 Timer 直到 ui.run() 结束。slint::Timer 一旦 drop 即停止,
    // 若让它在 setup_ui_callbacks 内作为局部变量被回收,轮询循环会立刻停摆——
    // 表现为 UI 永不处理 DEVICE_INFO/遥测(连不上、无数据、功能全失效)。
    let _timer = setup_ui_callbacks(&ui, controller.clone());

    ui.run()
        .map_err(|e| anyhow::anyhow!("UI run failed: {}", e))?;

    info!("mai2control-ui exiting");
    mai2control_ui::logging::hub().flush();
    Ok(())
}

/// 二值算法线归一化幅度 N 的上下限。下限取 0.1(允许把 0/1 压到比邻居线更矮),
/// 上限取 65535(右轴上的算法量纲可以是上万的计数, 要允许 N 跟得上)。
const ALGO_BIN_AMP_MIN: f32 = 0.1;
const ALGO_BIN_AMP_MAX: f32 = 65535.0;

/// 归一化幅度的 config.cfg 键名: 0..3 = report[idx], 4 = 触发判定。
fn algo_bin_amp_key(idx: usize) -> String {
    use mai2control_ui::ui_config::keys as k;
    if idx >= 4 {
        k::ALGO_BIN_AMP_ACTIVE.to_string()
    } else {
        format!("{}{}", k::ALGO_BIN_AMP_PREFIX, idx)
    }
}

/// 单条算法上报线勾选态的 config.cfg 键名(idx 0..3)。
fn report_show_key(idx: usize) -> String {
    format!(
        "{}{}",
        mai2control_ui::ui_config::keys::CURVE_REPORT_SHOW_PREFIX,
        idx
    )
}

/// 把当前界面偏好写回 config.cfg。由 16ms tick 调用: UiConfig::set_* 内部只在值真变了才落盘,
/// 所以这里无脑对账即可, 不必给每个开关都挂一个回调(那样每加一项设置都得改多处)。
/// `report_show` 不是 UI 属性(真相源在 Rust 侧那个 Rc, 见 on_report_toggle 的说明), 故随参传入,
/// 与其余偏好在同一处对账 —— 不给它单开一条落盘路径。
fn persist_ui_settings(
    ui: &AppWindow,
    report_show: &[bool; 4],
    cfg: &mut mai2control_ui::ui_config::UiConfig,
) {
    use mai2control_ui::ui_config::keys as k;
    cfg.set_i32(k::LOG_FILTER, ui.get_log_filter());
    cfg.set_bool(k::LOG_AUTO_SCROLL, ui.get_log_auto_scroll());
    cfg.set_bool(k::LOG_FILE_ON, ui.get_log_file_on());
    cfg.set_bool(k::VCAM_ENABLED, ui.get_vcam_enabled());
    cfg.set_i32(k::VCAM_SUBMIT_SECS, ui.get_vcam_submit_secs());
    cfg.set_i32(k::VCAM_DISPLAY_SECS, ui.get_vcam_display_secs());
    cfg.set_bool(k::LATENCY_MEASURE, ui.get_measure_latency());
    cfg.set_i32(k::CURRENT_VIEW, ui.get_current_view());
    cfg.set_i32(k::SETTINGS_TAB, ui.get_settings_tab());
    cfg.set_i32(k::SEL_CHANNEL, ui.get_sel_channel());
    // 曲线页"画哪些系列": 与上面各项同口径无脑对账(UiConfig::set_* 只在值真变了才落盘)。
    cfg.set_bool(k::CURVE_SHOW_RAW, ui.get_show_raw());
    cfg.set_bool(k::CURVE_SHOW_BSLN, ui.get_show_bsln());
    cfg.set_bool(k::CURVE_SHOW_DIFF, ui.get_show_diff());
    cfg.set_bool(k::CURVE_SHOW_ACTIVE, ui.get_show_active());
    cfg.set_bool(k::CURVE_ALGO_OVERLAY, ui.get_show_algo_overlay());
    for (idx, on) in report_show.iter().enumerate() {
        cfg.set_bool(&report_show_key(idx), *on);
    }
}

/// 重新枚举 HID 键盘并刷新设备树: 首行固定"所有键盘"(dev_index=0),
/// 之后按分类插入组头, 组内设备 dev_index = 在 `list` 中的下标 + 1。
/// `saved_path` 非空且仍在场 → 恢复该选择并生效; 否则回落"所有键盘"。返回设备个数。
fn refresh_vcam_devices(
    ui: &AppWindow,
    list: &Rc<RefCell<Vec<vcam::keyboard::KeyboardDevice>>>,
    saved_path: &str,
) -> usize {
    // list_keyboards 已按 分类→产品名→父实例→集合 排好序, 顺序扫一遍即可插两级组头。
    let devices = vcam::keyboard::list_keyboards();
    let mut rows: Vec<VcamKbdRow> = vec![VcamKbdRow {
        is_group: false,
        level: 0,
        title: "所有键盘(不限定设备)".into(),
        detail: "任何键盘输入都会进入扫码缓冲, 打字会污染数据".into(),
        dev_index: 0,
    }];
    let mut cur_cat = String::new();
    let mut cur_parent = String::new();
    for (i, dev) in devices.iter().enumerate() {
        if dev.category != cur_cat {
            cur_cat = dev.category.clone();
            cur_parent.clear();
            let n = devices.iter().filter(|d| d.category == cur_cat).count();
            rows.push(VcamKbdRow {
                is_group: true,
                level: 0,
                title: cur_cat.clone().into(),
                detail: format!("{} 项", n).into(),
                dev_index: -1,
            });
        }
        // 同一物理设备的集合个数: 1 个就折叠成一行(免去无意义的父层), 多个才展开子项。
        let siblings = devices
            .iter()
            .filter(|d| d.parent_key == dev.parent_key)
            .count();
        if dev.parent_key != cur_parent {
            cur_parent = dev.parent_key.clone();
            if siblings > 1 {
                let mut detail = dev.vendor.clone();
                if !detail.is_empty() {
                    detail.push_str("  ·  ");
                }
                detail.push_str(&format!("{} 个键盘集合", siblings));
                rows.push(VcamKbdRow {
                    is_group: true,
                    level: 1,
                    title: dev.product.clone().into(),
                    detail: detail.into(),
                    dev_index: -1,
                });
            }
        }
        let (title, detail) = if siblings > 1 {
            (dev.label.clone(), dev.detail.clone())
        } else {
            // 折叠行: 主文案用产品名, 副文案补上厂商。
            let mut d = dev.vendor.clone();
            if !d.is_empty() && !dev.detail.is_empty() {
                d.push_str("  ·  ");
            }
            d.push_str(&dev.detail);
            (dev.product.clone(), d)
        };
        rows.push(VcamKbdRow {
            is_group: false,
            level: if siblings > 1 { 2 } else { 1 },
            title: title.into(),
            detail: detail.into(),
            dev_index: i as i32 + 1,
        });
    }
    ui.set_vcam_kbd_rows(slint::ModelRc::new(slint::VecModel::from(rows)));

    let picked = devices
        .iter()
        .position(|d| d.path.eq_ignore_ascii_case(saved_path));
    ui.set_vcam_device_index(picked.map(|i| i as i32 + 1).unwrap_or(0));
    vcam::keyboard::set_target_device(picked.map(|i| devices[i].path.clone()));

    let count = devices.len();
    for d in &devices {
        log::debug!(
            "虚拟摄像头设备树: [{}] 产品={} 厂商={} 项={} ({}) parent={}",
            d.category,
            d.product,
            d.vendor,
            d.label,
            d.detail,
            d.parent_key
        );
    }
    log::info!(
        "虚拟摄像头: 枚举到 {} 个 HID 键盘设备, 当前选择={}",
        count,
        picked
            .map(|i| devices[i].label.clone())
            .unwrap_or_else(|| "所有键盘".into())
    );
    *list.borrow_mut() = devices;
    count
}

fn setup_ui_callbacks(ui: &AppWindow, controller: Rc<RefCell<AppController>>) -> slint::Timer {
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

    // 虚拟扫码摄像头共享状态 + 初始 UI 值(与状态默认对齐: 提交阈值 2s, 显示 10s)。
    let vcam = VcamState::new();
    vcam.set_submit_timeout_ms(2000);
    vcam.set_display_ms(10_000);
    ui.set_vcam_submit_secs(2);
    ui.set_vcam_display_secs(10);
    ui.set_vcam_enabled(false);
    // ★启动时**不**创建共享队列★: 队列的语义是"有且仅有一个活生产者", 生产者租约随
    // `FramePublisher` 存活。启动即创建等于本进程一直霸占租约, 第二个实例(哪怕只是想开个界面
    // 看日志)会被拒, 而摄像头明明还没启用。故只在"启用摄像头"回调里创建, 禁用/卸载时立刻 drop。
    let frame_publisher: Rc<RefCell<Option<FramePublisher>>> = Rc::new(RefCell::new(None));
    ui.set_vcam_runtime_status(
        format!(
            "未运行 · 启用后创建共享队列 {}(独占生产者租约)",
            share::MAP_NAME
        )
        .into(),
    );
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

    let install_status = vcam_backend::registration_status();
    log::info!("虚拟摄像头: 系统注册状态: {}", install_status);
    ui.set_vcam_install_status(install_status.into());
    // 可选 HID 键盘列表(下拉索引 0 = 所有键盘, 之后按此 Vec 顺序对应)。
    // 启动即按持久化的设备路径恢复选择: 设备不在场时回落到"所有键盘", 不静默失效。
    let vcam_kbd_list: Rc<RefCell<Vec<vcam::keyboard::KeyboardDevice>>> =
        Rc::new(RefCell::new(Vec::new()));
    {
        let saved = controller.borrow().vcam_kbd_device();
        refresh_vcam_devices(ui, &vcam_kbd_list, &saved);
    }

    let ui_weak = ui.as_weak();
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

    // 延迟图不再依赖绘图区宽高比：PlotPath 的 fit: fill 会把固定 1000×1000 数据坐标拉满。

    // 刷新按钮
    let ctrl_clone = controller.clone();
    let ui_refresh = ui_weak.clone();
    ui.on_refresh(move || {
        let ui = ui_refresh.upgrade().unwrap();
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.refresh_devices();
        let labels: Vec<slint::SharedString> =
            ctrl.device_labels().into_iter().map(|s| s.into()).collect();
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

    // 重启 PSoC(使需重启生效的改动生效)
    let ctrl_clone = controller.clone();
    ui.on_reboot_psoc(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reboot_psoc();
    });

    // 全通道频率自适应: ★改走 host 侧逐通道串行队列★(不再是 0xFF 让固件自己 for 36 遍)。
    // 队列会先把每通道的 IDAC 增益档写入并回读确认, 再逐通道下探; 全程可取消、有进度、
    // 失败能指名到通道。见 app_state::ch_ops。
    let ctrl_clone = controller.clone();
    ui.on_auto_tune(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.ch_batch_start(ChBatchKind::AutoTune) {
            ctrl.push_log_warn(format!("逐通道频率自适应未能开始: {}", e));
        }
    });

    // 批量队列取消(三种全通道操作共用一条队列, 故只有一个取消入口)。
    let ctrl_clone = controller.clone();
    ui.on_batch_op_cancel(move || {
        ctrl_clone.borrow_mut().ch_batch_cancel();
    });

    // "Cp 辅助"开关: 逐通道自适应按实测 Cp 推每通道的增益档/频率偏好档。
    let ctrl_clone = controller.clone();
    ui.on_set_cp_assist(move |on| {
        ctrl_clone.borrow_mut().set_cp_assist(on);
    });

    // PSoC 救砖: 经 SWD 强制重刷 PSoC 并重新下发算法/CSD(UI 已做两段式二次确认)。
    let ctrl_clone = controller.clone();
    ui.on_psoc_rescue(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.psoc_rescue();
    });

    // 校准频率偏好滑条(1..7): 只写草稿(与其他设置同规范), 但下一次自适应即读草稿生效。
    let ctrl_clone = controller.clone();
    ui.on_calib_pref_set(move |v| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config_number("calib.pref", v.clamp(1, 7) as f64);
    });

    // 全通道校准 / 全通道基线复位: 同样走 host 侧逐通道串行队列(每通道一条短命令, 可取消)。
    // ★固件那条全通道路径不删★ —— "恢复默认"等设备内部触发仍要用它, 只是 UI 不再从这里进。
    let ctrl_clone = controller.clone();
    ui.on_global_calibrate(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.ch_batch_start(ChBatchKind::Calibrate) {
            ctrl.push_log_warn(format!("逐通道校准未能开始: {}", e));
        }
    });

    let ctrl_clone = controller.clone();
    ui.on_global_baseline_reset(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.ch_batch_start(ChBatchKind::BaselineReset) {
            ctrl.push_log_warn(format!("逐通道基线复位未能开始: {}", e));
        }
    });

    // 本通道频率自适应(只下探当前选中通道, 其余通道分频不动)。
    let ctrl_clone = controller.clone();
    let ui_at = ui_weak.clone();
    ui.on_curve_auto_tune(move || {
        let ui = ui_at.upgrade().unwrap();
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.auto_tune(ch);
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

    // 触控组合映射(多分区 → 多键): 编辑全部落在 AppController(草稿), 由"保存到设备"整表下发。
    let ctrl_clone = controller.clone();
    ui.on_combo_zone_toggled(move |zone| {
        if !(0..34).contains(&zone) {
            return;
        }
        ctrl_clone.borrow_mut().kbd_combo_toggle_zone(zone as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_combo_zones_clear(move || {
        ctrl_clone.borrow_mut().kbd_combo_clear_zones();
    });

    let ctrl_clone = controller.clone();
    ui.on_combo_captured(move |text, ctrl_k, shift, alt, gui| {
        let code = char_to_hid(text.as_str());
        let mods = (ctrl_k as u8) | ((shift as u8) << 1) | ((alt as u8) << 2) | ((gui as u8) << 3);
        if code == 0 && mods == 0 {
            ctrl_clone
                .borrow_mut()
                .push_log("组合映射: 该按键无法识别为 HID 键码".to_string());
            return;
        }
        ctrl_clone.borrow_mut().kbd_combo_capture_key(code, mods);
    });

    let ctrl_clone = controller.clone();
    ui.on_combo_keys_clear(move || {
        ctrl_clone.borrow_mut().kbd_combo_clear_keys();
    });

    let ctrl_clone = controller.clone();
    ui.on_combo_add(move |delay, max_hold| {
        ctrl_clone.borrow_mut().kbd_combo_commit_pending(
            delay.clamp(0, u16::MAX as i32) as u16,
            max_hold.clamp(0, u16::MAX as i32) as u16,
        );
    });

    let ctrl_clone = controller.clone();
    ui.on_combo_removed(move |index| {
        if index < 0 {
            return;
        }
        ctrl_clone.borrow_mut().kbd_combo_remove(index as usize);
    });

    let ctrl_clone = controller.clone();
    ui.on_combo_hold_set(move |index, delay, max_hold| {
        if index < 0 {
            return;
        }
        ctrl_clone.borrow_mut().kbd_combo_set_hold(
            index as usize,
            delay.clamp(0, u16::MAX as i32) as u16,
            max_hold.clamp(0, u16::MAX as i32) as u16,
        );
    });

    // 已有映射的按键就地重录。★键码解析复用新建区那一套(char_to_hid + 同样的修饰位打包)★,
    // 不另写一份解析, 否则两条路径迟早对不上。会话边沿(capture_started) → 登记待替换行。
    let ctrl_clone = controller.clone();
    ui.on_combo_key_capture_started(move |index| {
        if index < 0 {
            return;
        }
        ctrl_clone
            .borrow_mut()
            .kbd_combo_begin_edit_keys(index as usize);
    });

    let ctrl_clone = controller.clone();
    ui.on_combo_key_captured(move |index, text, ctrl_k, shift, alt, gui| {
        if index < 0 {
            return;
        }
        let code = char_to_hid(text.as_str());
        let mods = (ctrl_k as u8) | ((shift as u8) << 1) | ((alt as u8) << 2) | ((gui as u8) << 3);
        if code == 0 && mods == 0 {
            ctrl_clone
                .borrow_mut()
                .push_log("组合映射: 该按键无法识别为 HID 键码".to_string());
            return;
        }
        ctrl_clone
            .borrow_mut()
            .kbd_combo_capture_key_at(index as usize, code, mods);
    });

    // 全通道页"批量应用": 勾选态与应用动作全部落在 AppController, UI 只转发事件。
    // 应用走 set_param(草稿), 与手工编辑同路径, 由"保存到设备"统一下发。
    let ctrl_clone = controller.clone();
    ui.on_batch_toggle_channel(move |ch| {
        if !(0..36).contains(&ch) {
            return;
        }
        ctrl_clone.borrow_mut().batch_toggle_channel(ch as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_toggle_param(move |param_id| {
        if !(0x01..=0x0B).contains(&param_id) {
            return;
        }
        ctrl_clone.borrow_mut().batch_toggle_param(param_id as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_channels_all(move || {
        ctrl_clone.borrow_mut().batch_channels_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_channels_invert(move || {
        ctrl_clone.borrow_mut().batch_channels_invert();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_params_all(move || {
        ctrl_clone.borrow_mut().batch_params_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_params_invert(move || {
        ctrl_clone.borrow_mut().batch_params_invert();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_clear(move || {
        ctrl_clone.borrow_mut().batch_clear();
    });

    // 批量启用/禁用: 复用 PARAM_ENABLED(0x0C) 的草稿路径, 与逐通道开关同一条下发链。
    let ctrl_clone = controller.clone();
    ui.on_batch_enable_channels(move || {
        let _ = ctrl_clone.borrow_mut().batch_set_enabled(true);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_disable_channels(move || {
        let _ = ctrl_clone.borrow_mut().batch_set_enabled(false);
    });

    // 抽屉收起: 真正清掉 Rust 侧的本次会话(勾选/待写值/源通道), 不是只把面板藏起来 ——
    // 勾选态活在 AppController 里, 只藏面板的话"应用到已选通道"仍会照着上一批勾选执行。
    let ctrl_clone = controller.clone();
    ui.on_batch_drawer_closed(move || {
        ctrl_clone.borrow_mut().batch_drawer_closed();
    });

    // 全通道页的采集开关(真停/开遥测流), 与曲线页的"暂停画面"分属两套语义。
    let ctrl_clone = controller.clone();
    ui.on_telem_source_start(move || {
        ctrl_clone.borrow_mut().telem_user_start();
    });

    let ctrl_clone = controller.clone();
    ui.on_telem_source_stop(move || {
        ctrl_clone.borrow_mut().telem_user_stop();
    });

    // 单通道精调页的通道启用开关(草稿路径, 由"保存到设备"下发)。
    let ctrl_clone = controller.clone();
    let ui_weak_ch_en = ui.as_weak();
    ui.on_curve_ch_enable_set(move |on| {
        let Some(ui) = ui_weak_ch_en.upgrade() else {
            return;
        };
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let _ = ctrl_clone.borrow_mut().set_ch_enabled(ch, on);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_set_source(move |ch| {
        if !(0..36).contains(&ch) {
            return;
        }
        ctrl_clone.borrow_mut().batch_set_source(ch as u8);
    });

    // 批量面板里逐项手改的"待写值"。★围栏不在这里判★: 用户把数字从一个值改到另一个值的中间态
    // 完全可能越界(退格删到只剩一位), 这里提前拒绝会让人根本改不动; 合法性统一在"应用"时由
    // set_param 的唯一围栏(proto::param_value_legal)判定并写日志告知。
    let ctrl_clone = controller.clone();
    ui.on_batch_param_value_set(move |param_id, value| {
        if !(0x01..=0x0B).contains(&param_id) || value < 0 {
            return;
        }
        ctrl_clone
            .borrow_mut()
            .batch_set_value(param_id as u8, value as u32);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_apply(move |src| {
        if !(0..36).contains(&src) {
            return;
        }
        let _ = ctrl_clone.borrow_mut().batch_apply_from(src as u8);
    });

    // JSON 导出：范围先由 Slint 弹窗确认，再使用系统原生保存对话框，失败只写日志不阻塞 UI。
    let ctrl_clone = controller.clone();
    ui.on_settings_export(
        move |config, channel_params, globals, algo, keyboard, zones| {
            let selected = mai2control_ui::settings_io::GroupSelection {
                config,
                channel_params,
                globals,
                algo,
                keyboard,
                zones,
            };
            match mai2control_ui::settings_io::choose_settings_path(true) {
                Ok(Some(path)) => match mai2control_ui::settings_io::export_settings(
                    &ctrl_clone.borrow(),
                    selected,
                ) {
                    Ok(text) => match std::fs::write(&path, text) {
                        Ok(()) => log::info!("设置 JSON 已导出: {}", path.display()),
                        Err(e) => {
                            log::warn!("设置 JSON 导出失败，无法写入 {}: {}", path.display(), e)
                        }
                    },
                    Err(e) => log::warn!("设置 JSON 导出失败: {}", e),
                },
                Ok(None) => log::info!("设置 JSON 导出已取消"),
                Err(e) => log::warn!("无法打开设置 JSON 保存对话框: {}", e),
            }
        },
    );

    // JSON 导入：只写 UI 草稿并置脏，不下发、不写 flash、不回读；结果(覆盖/跳过/未保存态)写进日志页。
    let ctrl_clone = controller.clone();
    ui.on_settings_import(
        move |config, channel_params, globals, algo, keyboard, zones| {
            let selected = mai2control_ui::settings_io::GroupSelection {
                config,
                channel_params,
                globals,
                algo,
                keyboard,
                zones,
            };
            match mai2control_ui::settings_io::choose_settings_path(false) {
                Ok(Some(path)) => match std::fs::read_to_string(&path) {
                    Ok(text) => {
                        let mut ctrl = ctrl_clone.borrow_mut();
                        match mai2control_ui::settings_io::import_settings(
                            &mut ctrl, &text, selected,
                        ) {
                            Ok(summary) => {
                                let report = summary.report_text();
                                // 跳过项必须显眼: 走 Warn 等级, 默认过滤下也能看到, 不静默丢弃。
                                if summary.skipped.is_empty() {
                                    ctrl.push_log(format!("{}（{}）", report, path.display()));
                                } else {
                                    ctrl.push_log_warn(format!("{}（{}）", report, path.display()));
                                }
                                log::info!("设置 JSON 已导入: {}", path.display());
                            }
                            Err(e) => {
                                ctrl.push_log_warn(format!(
                                    "设置 JSON 导入失败, 草稿未改动: {}",
                                    e
                                ));
                                log::warn!("设置 JSON 导入失败 {}: {}", path.display(), e);
                            }
                        }
                    }
                    Err(e) => log::warn!("设置 JSON 导入失败，无法读取 {}: {}", path.display(), e),
                },
                Ok(None) => log::info!("设置 JSON 导入已取消"),
                Err(e) => log::warn!("无法打开设置 JSON 导入对话框: {}", e),
            }
        },
    );

    // 撤销全部未保存草稿(CSD 安全操作 / 配置页): 恢复到设备当前运行态。
    let ctrl_clone = controller.clone();
    ui.on_discard_draft(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.discard_draft();
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

    // 数字配置的十六进制口径输入: LineEdit 文本(如 "0x1E") → 解析并按原类型写草稿。
    let ctrl_clone = controller.clone();
    ui.on_cfg_set_number_hex(move |key, value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config_hex(&key, &value);
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_set_enum(move |key, index| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config_enum(&key, index);
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_set_string(move |key, value| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config(ConfigEntry::new(
            key.to_string(),
            CfgValue::Str(value.to_string()),
        ));
    });

    // ---- 算法页 (JIT 触控算法 + C→ASM 编译器 + 全局设置) ----
    // 默认 C 源模板: 最小合法算法(沿用基础激活), 用户可改为自定义高动态逻辑。
    let algo_default_src = "#include <stddef.h>\n#include \"psoc_algo_abi.h\"\n\n// 入口: 每通道调用一次, 读写 io 固定字段。\n// 禁: libc / '/' '%' / 64位。辅助请 static inline。\nvoid algo(algo_io_t* io)\n{\n    // 示例: 直接沿用中间件基础激活判定。\n    // io->diff/baseline/finger_th/now_ms/rom 等可用于自定义高动态逻辑。\n    io->out_active = (io->base_active != 0u) ? 1u : 0u;\n}\n";
    ui.set_algo_c_source(algo_default_src.into());
    ui.set_algo_line_numbers(line_numbers_for(algo_default_src).into());
    // C 源容量条: 容量一次性回填, 占用随编辑器内容刷新(去注释后的字节数 = 编译器有效内容)。
    ui.set_algo_c_capacity(AppController::algo_src_capacity() as i32);
    ui.set_algo_c_bytes(AppController::algo_src_used(algo_default_src) as i32);

    let ctrl_clone = controller.clone();
    ui.on_algo_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.algo_get_info();
        let _ = ctrl.algo_get_rom();
        let _ = ctrl.request_algo_src(); // 回读设备映射表 C 源 → 还原可编辑算法
        let _ = ctrl.request_algo_code(); // 回读设备算法机器码 → 无本地编译时反汇编页看真实 ASM
        let _ = ctrl.global_get_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_algo_reset_default(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.algo_reset_default();
        // 恢复默认会清空设备侧算法 C 源 → 立即把内嵌默认源(去注释)回灌设备映射表,
        // 使"读取信息"能真正从设备取回去注释的默认 C 源(而非带注释的原始模板)。
        let _ = ctrl.send_algo_src(&AppController::strip_c_comments(ALGO_V31_TEMPLATE));
        let _ = ctrl.algo_get_rom();
        let _ = ctrl.request_algo_src();
    });

    // 载入内置 v3.1 HDR 模板到编辑器(参考 + 真实可下发案例)。
    let ui_tpl = ui_weak.clone();
    ui.on_algo_load_template(move |idx| {
        // idx: 0 = v3.1 HDR 默认(完整触发算法), 1 = 纯白灯演示(触摸点亮白灯, 展示算法可控性)。
        let tpl = if idx == 1 {
            ALGO_LED_DEMO_TEMPLATE
        } else {
            ALGO_V31_TEMPLATE
        };
        let ui = ui_tpl.upgrade().unwrap();
        ui.set_algo_c_source(tpl.into());
        ui.set_algo_line_numbers(line_numbers_for(tpl).into());
        ui.set_algo_c_bytes(AppController::algo_src_used(tpl) as i32);
    });

    // 编辑器内容变化 → 刷新行号列 + C 源占用(容量条)。
    let ui_edit = ui_weak.clone();
    ui.on_algo_source_edited(move |text| {
        let ui = ui_edit.upgrade().unwrap();
        ui.set_algo_line_numbers(line_numbers_for(text.as_str()).into());
        ui.set_algo_c_bytes(AppController::algo_src_used(text.as_str()) as i32);
    });

    // 键盘: 物理键/触控分区键码设置 + 刷新
    let ctrl_clone = controller.clone();
    ui.on_kbd_set_phys(move |idx, choice, modifier| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_map(idx as u8, kbd_choice_to_code(choice), modifier as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_kbd_set_zone(move |zone, choice, modifier| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_touchmap(zone as u8, kbd_choice_to_code(choice), modifier as u8);
    });

    // 键盘捕获输入框: 按下组合键 → 主键+修饰位；未识别键码写入可见日志提示。
    let ctrl_clone = controller.clone();
    ui.on_kbd_capture_phys(move |idx, text, c, s, a, g| {
        let code = char_to_hid(text.as_str());
        if code == 0 {
            ctrl_clone
                .borrow_mut()
                .push_log("物理键盘映射: 该按键无法识别为 HID 键码".to_string());
            return;
        }
        let m = (c as u8) | ((s as u8) << 1) | ((a as u8) << 2) | ((g as u8) << 3);
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_map(idx as u8, code, m);
    });
    let ctrl_clone = controller.clone();
    ui.on_kbd_capture_zone(move |zone, text, c, s, a, g| {
        let code = char_to_hid(text.as_str());
        if code == 0 {
            ctrl_clone
                .borrow_mut()
                .push_log("触控分区映射: 该按键无法识别为 HID 键码".to_string());
            return;
        }
        let m = (c as u8) | ((s as u8) << 1) | ((a as u8) << 2) | ((g as u8) << 3);
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_touchmap(zone as u8, code, m);
    });
    let ctrl_clone = controller.clone();
    ui.on_kbd_clear_phys(move |idx| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_map(idx as u8, 0, 0);
    });
    let ctrl_clone = controller.clone();
    ui.on_kbd_clear_zone(move |zone| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_touchmap(zone as u8, 0, 0);
    });

    let ctrl_clone = controller.clone();
    ui.on_kbd_set_hold_phys(move |idx, delay, max_hold| {
        if !(0..12).contains(&idx) {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_hold_phys(
            idx as u8,
            delay.clamp(0, u16::MAX as i32) as u16,
            max_hold.clamp(0, u16::MAX as i32) as u16,
        );
    });

    let ctrl_clone = controller.clone();
    ui.on_kbd_set_hold_zone(move |zone, delay, max_hold| {
        if !(0..34).contains(&zone) {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_hold_zone(
            zone as u8,
            delay.clamp(0, u16::MAX as i32) as u16,
            max_hold.clamp(0, u16::MAX as i32) as u16,
        );
    });

    // 每键触发极性 + 独立防抖: 写草稿(与长按参数同一条路径), 由"保存到设备"统一下发。
    // 越界(>10000us)由 AppController 直接拒绝并落日志 —— UI 围栏已同源, 走到这里说明是非 UI 路径。
    let ctrl_clone = controller.clone();
    ui.on_kbd_set_keycfg(move |idx, pol_mode, debounce_us| {
        if !(0..12).contains(&idx) {
            return;
        }
        // 极性档位围栏与 ComboBox 的 model 长度同源(0=低/1=高/2=AUTO)。
        if !(0..=2).contains(&pol_mode) {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.kbd_set_keycfg(
            idx as u8,
            pol_mode as u8,
            debounce_us.clamp(0, u16::MAX as i32) as u16,
        ) {
            ctrl.push_log_warn(format!("物理键每键配置: {}", e));
        }
    });

    // 逻辑分析仪时间窗切换/清空。时间窗是纯视图状态(不影响设备), 故只存在 UI 侧。
    let la_window_idx = Rc::new(Cell::new(LA_WINDOW_DEFAULT));
    let la_window_set = la_window_idx.clone();
    let ui_la = ui_weak.clone();
    ui.on_la_window_set(move |idx| {
        let i = (idx.max(0) as usize).min(LA_WINDOWS_US.len() - 1);
        la_window_set.set(i);
        if let Some(ui) = ui_la.upgrade() {
            ui.set_la_window_index(i as i32);
        }
    });
    let ctrl_clone = controller.clone();
    ui.on_la_clear(move || {
        ctrl_clone.borrow_mut().kbd_edges_clear();
    });

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

    // 算法编译一律丢后台线程(见 AlgoCompileJob/spawn_algo_compile): UI 线程只置 busy 并在
    // 16ms tick 里取回产物写入 AppController, 编译期间界面照常重绘与响应点击。
    let algo_busy = Rc::new(Cell::new(false));
    let algo_job: Rc<RefCell<Option<AlgoCompileJob>>> = Rc::new(RefCell::new(None));

    // 编译(不上传): 产出 ASM 并显示占用/进度; compile_blob 在超容量时报错, 保证塞得下不截断。
    let ui_algo = ui_weak.clone();
    let algo_busy_compile = algo_busy.clone();
    let algo_job_compile = algo_job.clone();
    ui.on_algo_compile(move |src| {
        let Some(ui) = ui_algo.upgrade() else {
            return;
        };
        if algo_busy_compile.get() {
            if algo_job_compile.borrow().is_some() {
                ui.set_algo_status("正在编译中，请等待完成".into());
                return;
            }
            // busy 与 job 槽必须同生同灭；仅 busy 悬挂说明上次任务已丢失，解锁后接受本次点击。
            algo_busy_compile.set(false);
            ui.set_algo_busy(false);
        }
        // C 源容量闸门: 超上限就地拒绝, 连工具链线程都不起(设备存不下, 编译出来也没法完整上传)。
        let used = AppController::algo_src_used(src.as_str());
        let cap = AppController::algo_src_capacity();
        ui.set_algo_c_bytes(used as i32);
        if used > cap {
            ui.set_algo_status(
                format!(
                    "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 已阻止编译",
                    used, cap
                )
                .into(),
            );
            return;
        }
        algo_busy_compile.set(true);
        ui.set_algo_busy(true);
        ui.set_algo_phase("编译中…".into());
        ui.set_algo_status("编译中…(后台工具链, 界面可继续操作)".into());
        spawn_algo_compile(&algo_job_compile, src.to_string(), false);
    });

    // 上传最近一次成功编译的 ASM(强制先编译后上传, 避免上传未经容量校验的产物)。
    let ctrl_clone = controller.clone();
    let ui_algo = ui_weak.clone();
    let algo_busy_upload = algo_busy.clone();
    let algo_job_upload = algo_job.clone();
    ui.on_algo_upload(move || {
        let Some(ui) = ui_algo.upgrade() else {
            return;
        };
        if algo_busy_upload.get() {
            if algo_job_upload.borrow().is_some() {
                ui.set_algo_status("正在编译中，请等待完成".into());
                return;
            }
            // 同上：busy=true 而 job 槽为空只能是遗留状态，不能永久静默吞掉用户点击。
            algo_busy_upload.set(false);
            ui.set_algo_busy(false);
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        match ctrl.upload_compiled() {
            Ok(()) => ui.set_algo_status(ctrl.algo_upload_status().into()),
            Err(e) => ui.set_algo_status(format!("上传失败: {}", e).into()),
        }
    });

    // ★一键编译并上传★: 编译同样在后台线程; 成功后由 tick 直接续走 upload_compiled
    // (上传本身走 mpsc 不阻塞), 失败即解锁并给出原因。
    let ui_algo = ui_weak.clone();
    let algo_busy_build = algo_busy.clone();
    let algo_job_build = algo_job.clone();
    ui.on_algo_build_upload(move |src| {
        let Some(ui) = ui_algo.upgrade() else {
            return;
        };
        if algo_busy_build.get() {
            if algo_job_build.borrow().is_some() {
                ui.set_algo_status("正在编译中，请等待完成".into());
                return;
            }
            // busy 与 job 槽失步时，上次任务已不可回收；先恢复不忙状态再启动新任务。
            algo_busy_build.set(false);
            ui.set_algo_busy(false);
        }
        // 同 on_algo_compile: 先过容量闸门, 超限直接拒, 不编译也不上传。
        let used = AppController::algo_src_used(src.as_str());
        let cap = AppController::algo_src_capacity();
        ui.set_algo_c_bytes(used as i32);
        if used > cap {
            ui.set_algo_status(
                format!(
                    "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 已阻止编译并上传",
                    used, cap
                )
                .into(),
            );
            return;
        }
        algo_busy_build.set(true);
        ui.set_algo_busy(true);
        ui.set_algo_phase("编译中…".into());
        ui.set_algo_status("编译中…(后台工具链, 界面可继续操作)".into());
        spawn_algo_compile(&algo_job_build, src.to_string(), true);
    });

    // 算法可调变量(cfg[8]) SpinBox 编辑 → 立即下发+持久化。
    let ctrl_clone = controller.clone();
    ui.on_algo_setting_edited(move |idx, value| {
        if idx < 0 || value < 0 || value > 255 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_algo_cfg(idx as u8, value as u8);
    });

    // 算法上报变量逐条显示开关(默认全显): 行模型常驻，勾选立即原地改对应行，
    // 后续追踪刷新仍以该真相源为准，避免复选框被模型回填弹回。
    let report_show = Rc::new(RefCell::new([true; 4usize]));
    let algo_report_lines_model: Rc<slint::VecModel<AlgoReportLine>> =
        Rc::new(slint::VecModel::from(Vec::new()));
    ui.set_algo_report_lines(slint::ModelRc::from(algo_report_lines_model.clone()));
    let rs_toggle = report_show.clone();
    let report_model_toggle = algo_report_lines_model.clone();
    ui.on_report_toggle(move |idx, on| {
        if idx >= 0 && (idx as usize) < 4 {
            let row = idx as usize;
            rs_toggle.borrow_mut()[row] = on;
            if let Some(mut line) = report_model_toggle.row_data(row) {
                line.visible = on;
                report_model_toggle.set_row_data(row, line);
            }
        }
    });

    let ctrl_clone = controller.clone();
    ui.on_algo_global_set(move |gparam_id, value| {
        if gparam_id < 0 || value < 0 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.global_set(gparam_id as u8, value as u32);
    });

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
    ui.on_mai2_set_send_en(move |enabled| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.mai2_set_send_en(enabled);
    });

    let ctrl_clone = controller.clone();
    ui.on_mai2_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.mai2_request_state();
    });

    // ---------------- 协议页: mai2light 灯板协议 ----------------
    let ctrl_clone = controller.clone();
    ui.on_light_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.led_request_state();
    });

    // 映射编辑只落 Rust 侧草稿; 下拉索引 0/1/2 ↔ 固件 ch 0xFF/0/1。
    let ctrl_clone = controller.clone();
    ui.on_light_map_set(move |unit, ch_choice, start, count| {
        if !(0..11).contains(&unit) {
            return;
        }
        let ch = if ch_choice <= 0 {
            LED_CH_UNMAPPED
        } else {
            (ch_choice - 1) as u8
        };
        ctrl_clone.borrow_mut().led_set_region(
            unit as usize,
            ch,
            start.clamp(0, u16::MAX as i32) as u16,
            count.clamp(0, 255) as u8,
        );
    });

    let ctrl_clone = controller.clone();
    ui.on_light_map_apply(move || {
        let _ = ctrl_clone.borrow_mut().led_apply_regions();
    });

    // unit < 0 → 全部单元(LED_PREVIEW_ALL)。
    let ctrl_clone = controller.clone();
    ui.on_light_preview(move |unit, r, g, b| {
        let target = if unit < 0 || unit > 10 {
            LED_PREVIEW_ALL
        } else {
            unit as u8
        };
        let rgb = [
            r.clamp(0, 255) as u8,
            g.clamp(0, 255) as u8,
            b.clamp(0, 255) as u8,
        ];
        let _ = ctrl_clone.borrow_mut().led_preview(target, rgb);
    });

    // 灯链长度/亮度是配置 KV, 复用既有草稿写入路径(随"保存到设备"落 flash), 不另造协议。
    let ctrl_clone = controller.clone();
    ui.on_light_ws_count_set(move |chain, value| {
        let key = if chain == 0 {
            "led.ws_count0"
        } else {
            "led.ws_count1"
        };
        let _ = ctrl_clone
            .borrow_mut()
            .set_config_number(key, value.clamp(1, 1000) as f64);
    });

    let ctrl_clone = controller.clone();
    ui.on_light_brightness_set(move |value| {
        let _ = ctrl_clone
            .borrow_mut()
            .set_config_number("led.ws_brightness", value.clamp(0, 255) as f64);
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
            ui.set_settings_tab(4);
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
        ui.set_settings_tab(4);
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
        // 掩码只含目标通道 —— 固件已能逐通道复位(SENSOR_CH_ALL 哨兵之外即单通道), 其余 35 个不动。
        // ★只复位, 不再顺带冻结画面★ 冻结是"暂停画面"按钮的职责: 复位本身要看的是复位之后
        // 基线怎么收敛, 冻住反而停在复位前的旧数据上; 而且冻结期间设备仍在出帧, 解冻时那段空档
        // 现在由绘图管线按"无覆盖 ⇒ 虚线"表达(见 fill_uncovered_with_dashes), 不需要靠冻结遮掩。
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

    // ★已删除单通道精调主图的 plot_area_resized/宽高比机制★:
    // 上一版靠"把曲线 x 拉伸到 [0,1000*aspect] + viewbox 宽同乘 aspect"来对抗 Path 的 contain
    // (等比缩放+居中)。该修正只在横纵缩放倍数相同(全览)时成立: 一旦横向缩放时间窗(viewbox 变窄、
    // 高度不变), viewbox 宽高比 != 元素宽高比, contain 退化为按高度缩放, 横向只占 view_w/1000 的
    // 宽度 → 数据两侧又出现空白带。现在主图 PlotPath 改用 fit: fill(非等比拉伸), 任意宽高比的
    // viewbox 都被拉满元素, 缩放/平移到任何极端位置都铺满可视区; 于是 path 的 x 固定为 [0,1000],
    // Rust 侧不必知道绘图区形状, 拖分栏/改窗口也不再触发重算全部 path。

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
        ui.set_vcam_submit_secs(cfg.get_i32(k::VCAM_SUBMIT_SECS, 2).clamp(1, 10));
        ui.set_vcam_display_secs(cfg.get_i32(k::VCAM_DISPLAY_SECS, 10).clamp(1, 60));
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
        // 上限 8: 共 9 个标签(0绑区 1协议 2触控通道 3触控全局 4精调 5触控键映 6物理键盘 7通信 8算法)。
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
        // 日志落盘: 关掉过就保持关掉(logging::init 默认开)。
        if !cfg.get_bool(k::LOG_FILE_ON, true) {
            mai2control_ui::logging::hub().set_file_enabled(false);
        }
        // 恢复后的等级要同时作用于文件与控制台。
        let lv = ui.get_log_filter().clamp(0, 3) as u8;
        controller.borrow_mut().set_log_filter(lv);
        mai2control_ui::logging::hub().set_level(lv);
        // 虚拟摄像头的秒数直接进共享状态, 否则要等用户动一次 SpinBox 才生效。
        vcam.set_submit_timeout_ms((ui.get_vcam_submit_secs().max(1) as u32) * 1000);
        vcam.set_display_ms((ui.get_vcam_display_secs().max(1) as u32) * 1000);
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

    // ---------------- 日志页 ----------------
    // 过滤等级: 写入 app_state 作真相源, 并强制下一帧重建行模型(把 last_log_ver 打脏)。
    ui.set_log_filter(controller.borrow().log_filter() as i32);
    mai2control_ui::logging::hub().set_level(controller.borrow().log_filter());
    ui.set_log_file_on(mai2control_ui::logging::hub().file_enabled());
    ui.set_log_file_path(mai2control_ui::logging::hub().file_path_text().into());
    let log_dirty = Rc::new(std::cell::Cell::new(true));

    let ctrl_logf = controller.clone();
    let log_dirty_f = log_dirty.clone();
    ui.on_set_log_filter(move |lvl| {
        let lv = lvl.clamp(0, 3) as u8;
        ctrl_logf.borrow_mut().set_log_filter(lv);
        // 落盘与控制台门槛都跟着走: 看什么等级就记什么等级, 不另设一套阈值。
        mai2control_ui::logging::hub().set_level(lv);
        log_dirty_f.set(true);
    });

    // 清空动作由状态层完成；脏标记确保下一次 tick 即回填零行视图。
    let ctrl_clear = controller.clone();
    let log_dirty_clear = log_dirty.clone();
    ui.on_clear_log(move || {
        ctrl_clear.borrow_mut().clear_log();
        log_dirty_clear.set(true);
    });

    let ctrl_copy2 = controller.clone();
    ui.on_copy_log_all(move || {
        let filter = ctrl_copy2.borrow().log_filter();
        let text =
            mai2control_ui::logging::hub().text_for_copy(filter, mai2control_ui::logging::VIEW_MAX);
        let lines = text.lines().count();
        match mai2control_ui::logging::copy_to_clipboard(&text) {
            Ok(()) => log::info!("已复制当前视图 {} 行到剪贴板", lines),
            Err(e) => log::warn!("复制失败: {}", e),
        }
    });

    ui.on_open_log_dir(move || {
        if let Err(e) = mai2control_ui::logging::open_logs_dir() {
            log::warn!("{}", e);
        }
    });

    // 关于页: 上位机版本与仓库地址都是编译期常量, 一次性回填(不进 16ms tick)。
    ui.set_about_app_version(env!("CARGO_PKG_VERSION").into());
    ui.set_about_repo_url(REPO_URL.into());
    ui.on_copy_repo_url(
        move || match mai2control_ui::logging::copy_to_clipboard(REPO_URL) {
            Ok(()) => log::info!("已复制仓库地址到剪贴板: {}", REPO_URL),
            Err(e) => log::warn!("复制仓库地址失败: {}", e),
        },
    );

    let ui_logfile = ui_weak.clone();
    ui.on_set_log_file_enabled(move |on| {
        let hub = mai2control_ui::logging::hub();
        hub.set_file_enabled(on);
        if let Some(ui) = ui_logfile.upgrade() {
            ui.set_log_file_on(hub.file_enabled());
            ui.set_log_file_path(hub.file_path_text().into());
        }
    });

    // 虚拟摄像头(与 OBS Virtual Camera 同实现: DirectShow 源过滤器)。
    // ★上位机侧没有任何 COM 对象要持有★: 消费端(游戏/Unity/OBS)在自己的进程里创建过滤器,
    // 本进程只负责把 QR 帧写进共享队列。开关只控制"采集 + 队列生产者租约"。
    let vcam_cb = vcam.clone();
    let publisher_cb = frame_publisher.clone();
    let ui_vcam = ui_weak.clone();
    ui.on_set_vcam_enabled(move |on| {
        let Some(ui) = ui_vcam.upgrade() else { return };
        if !on {
            // 停止顺序不能变：先截断采集，再释放队列生产者租约。
            vcam_cb.set_enabled(false);
            vcam::keyboard::stop();
            *publisher_cb.borrow_mut() = None;
            ui.set_vcam_runtime_status(
                "未运行 · 摄像头仍在系统设备列表中，消费端会读到黑帧".into(),
            );
            return;
        }
        // 启动顺序：注册核验 → 队列 → 键盘采集。每个失败分支都回收已完成阶段。
        match vcam_backend::is_registered() {
            Ok(true) => {}
            Ok(false) => {
                log::warn!("虚拟摄像头: DirectShow 注册未完整核验；请先点击“安装(需管理员)”");
                vcam_cb.set_enabled(false);
                ui.set_vcam_enabled(false);
                ui.set_vcam_install_status(vcam_backend::registration_status().into());
                ui.set_vcam_runtime_status("未运行 · DirectShow 注册未完整安装".into());
                return;
            }
            Err(error) => {
                log::error!("虚拟摄像头: 注册状态核验失败: {}", error);
                vcam_cb.set_enabled(false);
                ui.set_vcam_enabled(false);
                ui.set_vcam_runtime_status("未运行 · 注册状态核验失败".into());
                return;
            }
        }
        match FramePublisher::create() {
            Ok(publisher) => *publisher_cb.borrow_mut() = Some(publisher),
            Err(error) => {
                log::error!("虚拟摄像头: 无法创建共享队列: {}", error);
                vcam_cb.set_enabled(false);
                ui.set_vcam_enabled(false);
                ui.set_vcam_runtime_status(format!("未运行 · {}", error).into());
                return;
            }
        }
        vcam_cb.set_enabled(true);
        vcam::keyboard::start(vcam_cb.clone());
        ui.set_vcam_runtime_status("运行中 · 扫码即推 QR 帧到共享队列，消费端随时可打开".into());
    });
    // 安装/卸载仍在后台执行，避免 UAC 和 regsvr32 阻塞 Slint 事件循环。
    let ui_vcam_install = ui_weak.clone();
    ui.on_install_vcam(move || {
        let Some(ui) = ui_vcam_install.upgrade() else {
            return;
        };
        if ui.get_vcam_deploy_busy() {
            return;
        }
        ui.set_vcam_deploy_busy(true);
        ui.set_vcam_install_status("安装中… 请在 UAC 弹窗上确认".into());
        let ui_done = ui_vcam_install.clone();
        let spawned = std::thread::Builder::new()
            .name("vcam-install".into())
            .spawn(move || {
                let status = match vcam_backend::install() {
                    Ok(status) => {
                        log::info!("虚拟摄像头: 安装核验通过: {}", status);
                        status
                    }
                    Err(error) => {
                        let status = format!("安装未确认: {}", error);
                        log::warn!("虚拟摄像头: {}", status);
                        status
                    }
                };
                let _ = ui_done.upgrade_in_event_loop(move |ui| {
                    ui.set_vcam_install_status(status.into());
                    ui.set_vcam_deploy_busy(false);
                });
            })
            .is_ok();
        if !spawned {
            ui.set_vcam_install_status("安装未启动: 无法创建工作线程".into());
            ui.set_vcam_deploy_busy(false);
        }
    });
    let ui_vcam_uninstall = ui_weak.clone();
    let vcam_uninstall = vcam.clone();
    let publisher_uninstall = frame_publisher.clone();
    ui.on_uninstall_vcam(move || {
        let Some(ui) = ui_vcam_uninstall.upgrade() else {
            return;
        };
        if ui.get_vcam_deploy_busy() {
            return;
        }
        vcam_uninstall.set_enabled(false);
        vcam::keyboard::stop();
        *publisher_uninstall.borrow_mut() = None;
        ui.set_vcam_enabled(false);
        ui.set_vcam_runtime_status("未运行 · 采集与共享队列已释放".into());
        ui.set_vcam_deploy_busy(true);
        ui.set_vcam_install_status("卸载中… 请在 UAC 弹窗上确认".into());
        let ui_done = ui_vcam_uninstall.clone();
        let spawned = std::thread::Builder::new()
            .name("vcam-uninstall".into())
            .spawn(move || {
                let status = match vcam_backend::uninstall() {
                    Ok(status) => {
                        log::info!("虚拟摄像头: 卸载核验通过: {}", status);
                        status
                    }
                    Err(error) => {
                        let status = format!("卸载未确认: {}", error);
                        log::warn!("虚拟摄像头: {}", status);
                        status
                    }
                };
                let _ = ui_done.upgrade_in_event_loop(move |ui| {
                    ui.set_vcam_install_status(status.into());
                    ui.set_vcam_deploy_busy(false);
                });
            })
            .is_ok();
        if !spawned {
            ui.set_vcam_install_status("卸载未启动: 无法创建工作线程".into());
            ui.set_vcam_deploy_busy(false);
        }
    });
    let vcam_cb = vcam.clone();
    ui.on_set_vcam_submit_secs(move |s| {
        vcam_cb.set_submit_timeout_ms((s.max(1) as u32) * 1000);
    });
    let vcam_cb = vcam.clone();
    ui.on_set_vcam_display_secs(move |s| {
        vcam_cb.set_display_ms((s.max(1) as u32) * 1000);
    });

    // 输入源设备选择: index 0 = 所有键盘, 其余对应 vcam_kbd_list 里的设备。
    // 选中即生效(捕获线程运行中也可切换), 并持久化到 toolbox.cfg。
    let ctrl_clone = controller.clone();
    let kbd_list_sel = vcam_kbd_list.clone();
    let ui_kbd_sel = ui_weak.clone();
    ui.on_set_vcam_device(move |index| {
        let list = kbd_list_sel.borrow();
        let picked = (index > 0)
            .then(|| {
                list.get((index - 1) as usize)
                    .map(|d: &vcam::keyboard::KeyboardDevice| d.path.clone())
            })
            .flatten();
        vcam::keyboard::set_target_device(picked.clone());
        // 选中行高亮靠 dev_index 比较, 不重建整棵树。
        if let Some(ui) = ui_kbd_sel.upgrade() {
            ui.set_vcam_device_index(index);
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.set_vcam_kbd_device(picked.clone().unwrap_or_default());
        match picked {
            Some(_) => ctrl.push_log(format!(
                "虚拟摄像头: 输入源已限定为 {}",
                list.get((index - 1) as usize)
                    .map(|d| d.label.clone())
                    .unwrap_or_default()
            )),
            None => ctrl.push_log("虚拟摄像头: 输入源为所有键盘(未限定设备)".to_string()),
        }
    });

    // 刷新设备列表(扫码器热插拔后用)。
    let ui_kbd = ui_weak.clone();
    let ctrl_clone = controller.clone();
    let kbd_list_refresh = vcam_kbd_list.clone();
    ui.on_refresh_vcam_devices(move || {
        let ui = ui_kbd.upgrade().unwrap();
        let saved = ctrl_clone.borrow().vcam_kbd_device();
        let count = refresh_vcam_devices(&ui, &kbd_list_refresh, &saved);
        ctrl_clone
            .borrow_mut()
            .push_log(format!("虚拟摄像头: 已刷新键盘设备列表, 共 {} 个", count));
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
    // 主图独立版本: 冻结期间不随实时遥测推进，避免重复 SVG/path 重算；解冻时跳到最新缓冲。
    let mut last_plot_version = u64::MAX;
    // 用非初始值确保即使尚未收到遥测，也先把完整 36 通道卡片回填到 UI。
    let mut last_telem_version_all = u64::MAX;
    // Cp 缓存版本门控：无新遥测但有新 Cp 响应时也要刷新全通道卡片的 cp_text。
    let mut last_cp_version_all = u64::MAX;
    // 启用态(0x0C 草稿)与排序/筛选选择的门控: 三者任一变化都要重排/重筛全通道网格。
    let mut last_param_version_all = u64::MAX;
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
    let mut last_curve_visibility = (false, false, false);
    // Cp 失败通道列表(形如 "CH0 CH5"); 只在内容变化时回填, 免得每 tick 都往 UI 写字符串。
    let mut last_cp_failed = String::new();
    // 算法叠加脏标记: 它会改变主图 path, 与遥测版本一起做门控(绘图区尺寸已不再参与, 见 fit: fill)。
    let mut algo_overlay_dirty = true;
    let mut reconnect_tick = 0u32;
    let mut was_connected = false;
    // mode.work 保存后自动重启倒计时(tick): 给 SAVE_CONFIG 的 flash 写留出完成窗口再重启重枚举。
    let mut reboot_countdown: Option<u32> = None;
    // "正在等待下发完成再重启"只提示一次, 避免每 tick 刷日志。
    let mut reboot_wait_logged = false;
    let mut last_log_ver = u64::MAX;
    let ui_cfg_tick = ui_cfg.clone();
    let mut last_algo_version = u64::MAX;
    let mut last_algo_upload_version = 0u64;
    let mut last_algo_src_version = u64::MAX;
    let mut last_algo_code_version = u64::MAX;
    // 默认算法的 C 源(已滤注释)是否已回灌到设备映射表: 设备默认算法出厂不带源,
    // 首次读到"默认+无源"时把内嵌默认源(去注释)下发设备一次, 使之后"读取信息"能真正从设备取回。
    let mut default_src_synced = false;
    let mut last_algo_asm_version = u64::MAX;
    // 算法追踪(report[]/out_active)/可调变量(cfg[8]): 版本门控 + 轮询游标。
    let mut last_algo_trace_version = u64::MAX;
    let mut last_algo_cfg_version = u64::MAX;
    // schema(ALGO_REPORT/ALGO_SETTING)版本门控: 源取自 AppController::algo_schema_source()
    // (设备回读源优先), 与编辑器文本无关 —— 连接后自动按设备正在跑的算法填面板/轮询集合。
    let mut last_algo_schema_version = u64::MAX;
    // 自动载入编辑器的那份文本: 用于判断编辑器是否已被用户改过(改过就不再自动覆盖)。
    // ★初值必须等于启动时预置进编辑器的那份模板★: 否则"编辑器 == 自动载入值"恒不成立,
    // 会把开机预置的模板当成"用户的改动"而永不载入设备算法源 —— 表现为 JIT 算法从不自动同步显示。
    let mut editor_autoload_mark = algo_default_src.to_string();
    // 已声明的 ALGO_REPORT idx 列表(从源码 schema 解析, 轮询游标按此列表轮转; 未声明则不轮询)。
    let mut algo_report_idxs: Vec<u8> = Vec::new();
    let mut algo_trace_rr = 0usize;
    let mut last_globals_version = u64::MAX;
    let mut last_lat_version = u64::MAX;
    let mut last_global_tune_visible = false;
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
    let curve_params_model: Rc<slint::VecModel<ParamRow>> = Rc::new(slint::VecModel::from(
        build_param_rows(&controller.borrow().params_of(0)),
    ));
    ui.set_curve_params(slint::ModelRc::from(curve_params_model.clone()));
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
    let mut last_led_version = u64::MAX;
    // 协议页驻留门控: 进页边沿请求一次, 离页停止轮询(见下方 protocol_visible)。
    let mut last_protocol_visible = false;

    let report_show_timer = report_show.clone();
    // 后台编译任务槽与 busy 闸: tick 里取回产物后由本处解锁(见"后台编译产物回收")。
    let algo_job_timer = algo_job.clone();
    let algo_busy_timer = algo_busy.clone();
    let algo_report_lines_model_timer = algo_report_lines_model.clone();
    let report_norm_timer = report_norm.clone();
    let vcam_timer = vcam.clone();
    let publisher_timer = frame_publisher.clone();
    let mut last_vcam_frame_version = 0u32;
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
    let timer = slint::Timer::default();
    timer.start(slint::TimerMode::Repeated, Duration::from_millis(UI_TICK_MS as u64), move || {
        let prof_tick_start = Instant::now();
        if let Some(previous) = prof_last_tick.replace(prof_tick_start) {
            let gap = prof_tick_start.duration_since(previous).as_micros() as u64;
            prof_gap_us += gap;
            prof_gap_max_us = prof_gap_max_us.max(gap);
            prof_gaps += 1;
        }
        let ui = ui_weak.upgrade().unwrap();
        let mut ctrl = controller.borrow_mut();

        // 虚拟摄像头: 推进时序状态机(显示到期转黑)。帧变化时把 RGB24 转 NV12 提交到共享
        // 队列(序号在像素写完之后才提交, 由 FramePublisher 保证), 再刷新 UI 预览。
        // 每 tick 都刷心跳: 过滤器靠 tick_ms 区分"画面没变"与"生产者已消失(进程被杀)"。
        vcam_timer.tick();
        if let Some(publisher) = publisher_timer.borrow_mut().as_mut() {
            publisher.heartbeat();
        }
        let vframe_ver = vcam_timer.frame_version();
        if vframe_ver != last_vcam_frame_version {
            last_vcam_frame_version = vframe_ver;
            let rgb = vcam_timer.frame_copy();
            if let Some(publisher) = publisher_timer.borrow_mut().as_mut() {
                if let Err(error) = publisher.publish(&rgb) {
                    log::error!("虚拟摄像头: 帧发布失败: {}", error);
                }
            }
            let mut buf = slint::SharedPixelBuffer::<slint::Rgb8Pixel>::new(FRAME_W as u32, FRAME_H as u32);
            let dst = buf.make_mut_bytes();
            if dst.len() == rgb.len() {
                dst.copy_from_slice(&rgb);
                ui.set_vcam_preview(slint::Image::from_rgb8(buf));
            }
            ui.set_vcam_last_data(vcam_timer.last_data().into());
        }

        // CSD 调试诊断: 推进"改全局后回读设备状态"的排程与窗口(日志写入见 app_state)。
        ctrl.csd_diag_tick();

        // 恢复默认设备端回填就绪: 全量重读(配置/当前通道+CH0 参数/全局/Cp) 刷新显示。
        if ctrl.take_post_reset_refetch() {
            let ch = ui.get_sel_channel().clamp(0, 35) as u8;
            // Cp 不在此刷新: 电容获取一律由用户手动"测量电容"触发(自动测量会与重初始化抢链路)。
            ctrl.push_log("恢复默认完成: 重读设备配置 / 全 36 通道参数 / 全局 刷新显示");
            let _ = ctrl.request_config_all();
            let _ = ctrl.request_params(ch);
            let _ = ctrl.request_params(0);
            // ★必须覆盖全 36 通道★: 只重读"当前通道 + CH0"会让其余 34 通道停在恢复默认前的
            // 过期值(或空), 界面表现为"只留了一个通道的数据"。队列每 tick 发一条, 不加快轮询。
            ctrl.schedule_param_refetch_all();
            let _ = ctrl.global_get_all();
            // 重取 DEVICE_INFO: 其报告尾部 csd_flags 携带"恢复默认是否获得可信基线", 驱动异常采样警示行。
            let _ = ctrl.resend_hello();
            // 时钟树的分频范围需要全 36 通道 snsClk(恢复默认后已全变): 一条批量取回, 非 36 条单发。
            let _ = ctrl.request_param_all_channels(PARAM_SNS_CLK_DIV);
        }
        let prof_poll_start = Instant::now();
        ctrl.poll_ui();
        prof_poll_us += prof_poll_start.elapsed().as_micros() as u64;
        // 侦听绑定: 捕获下一次触摸的物理通道并写入草稿(仅在侦听态时有动作)。
        let _ = ctrl.listen_tick();

        // 自动重连:断开且有设备时每~2s 刷新并重连第一个,用户无需手动连接。
        reconnect_tick = reconnect_tick.wrapping_add(1);
        // ★节拍与墙钟解耦★ 所有周期性动作都按毫秒表达(`_every_ms`), 不再写死"每 N 个 tick" ——
        // tick 周期改动时那种写法会静默把所有轮询间隔一起缩放掉。
        // 视觉重建(SVG/36 卡片)按墙钟限速: 遥测 170Hz+, 重投影没必要比屏幕刷新还快。
        let visual_refresh_tick = _every_ms(reconnect_tick, VISUAL_REFRESH_MS);
        if ctrl.state() == ConnState::Disconnected && _every_ms(reconnect_tick, 2_000) {
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
        // PSoC 链路活性探测(~1s 一次)。★只在用户手动停流时才真的发包★(门控在 psoc_link_probe
        // 内): 遥测在跑时每帧的 samples_per_sec 就是代数推进证据, 不需要探; 而 HELLO 会顺带停流,
        // 流式期间探测等于把遥测踢停。停流后链路状态没有别的来源, 不探就只能显示握手时的旧快照。
        if _every_ms(reconnect_tick, 1_000) {
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

        // 界面偏好对账落盘(值未变则不写盘)。放在 tick 里而不是给每个开关挂回调:
        // Slint 侧有些开关(自动滚动、页签、通道)是直接改 in-out 属性的, 本来就没有回调可挂。
        persist_ui_settings(
            &ui,
            &report_show_timer.borrow(),
            &mut ui_cfg_tick.borrow_mut(),
        );

        // 日志页: 版本号门控重建行模型。只取尾部 VIEW_MAX 条, 长日志下 UI 模型不随之膨胀
        // (全量仍在 logs/ 文件里)。行号用全局序号, 过滤切换后仍能与文件对上。
        {
            let hub = mai2control_ui::logging::hub();
            let ver = hub.version();
            if ver != last_log_ver || log_dirty.get() {
                last_log_ver = ver;
                log_dirty.set(false);
                let entries = hub.snapshot(ctrl.log_filter(), mai2control_ui::logging::VIEW_MAX);
                let shown = entries.len();
                // 日志正文与算法编辑器同款 gutter 分离：正文可无干扰拖选，行号保持固定左列。
                let mut text = String::with_capacity(shown * 96);
                for e in &entries {
                    text.push_str(&e.text.replace('\n', " | "));
                    text.push('\n');
                }
                ui.set_log_line_numbers(
                    if shown == 0 { String::new() } else { line_numbers_for(&text) }.into(),
                );
                ui.set_log_text(text.into());
                let dropped = hub.dropped();
                ui.set_log_stats(
                    format!(
                        "视图 {} 行(上限 {}) · 内存保留 {} 条{}",
                        shown,
                        mai2control_ui::logging::VIEW_MAX,
                        mai2control_ui::logging::RING_MAX,
                        if dropped > 0 {
                            format!(" · 已滚出内存 {} 条(仍在日志文件中)", dropped)
                        } else {
                            String::new()
                        }
                    )
                    .into(),
                );
            }
        }

        // 后台编译产物回收: try_recv 不阻塞, 拿到结果才写状态(Rc<RefCell<AppController>> 只在本线程碰)。
        // 成功且要求上传时当帧续走 upload_compiled(其内部走 mpsc, 不阻塞 UI)。
        {
            let finished = match algo_job_timer.borrow().as_ref() {
                Some(job) => match job.rx.try_recv() {
                    Ok(result) => Some(result),
                    Err(std::sync::mpsc::TryRecvError::Empty) => None,
                    // 线程 panic 才会断连: 当作一次失败结束, 否则 busy 会永久卡住。
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        Some(Err(anyhow::anyhow!("编译线程异常退出")))
                    }
                },
                None => None,
            };
            if let Some(result) = finished {
                let job = algo_job_timer.borrow_mut().take();
                let cap = AppController::algo_slot_capacity();
                match (result, job) {
                    (Ok(out), Some(job)) => {
                        let len = ctrl.apply_compiled(&job.src, out);
                        ui.set_algo_asm_bytes(len as i32);
                        let pct = (len * 100) / cap;
                        if job.upload_after {
                            ui.set_algo_phase("上传中…".into());
                            match ctrl.upload_compiled() {
                                Ok(()) => ui.set_algo_status(format!(
                                    "编译成功: ASM {} / {} 字节 ({}%); {}",
                                    len, cap, pct, ctrl.algo_upload_status()).into()),
                                Err(error) => {
                                    ctrl.push_log_warn(format!("算法: 编译成功但上传失败: {}", error));
                                    ui.set_algo_status(format!("编译成功但上传失败: {}", error).into());
                                }
                            }
                        } else {
                            ui.set_algo_status(format!(
                                "编译成功: ASM {} / {} 字节 ({}%), 可上传", len, cap, pct).into());
                        }
                    }
                    (Err(error), job) => {
                        ui.set_algo_asm_bytes(0);
                        ctrl.push_log_error(format!("算法: 编译失败: {}", error));
                        let tail = if job.map(|j| j.upload_after).unwrap_or(false) { "(未上传)" } else { "" };
                        ui.set_algo_status(format!("编译失败{}: {}", tail, error).into());
                    }
                    // 结果与任务槽必然同生同灭, 该分支不可达; 兜底解锁避免 busy 悬挂。
                    (Ok(_), None) => {}
                }
                ui.set_algo_busy(false);
                ui.set_algo_phase("".into());
                algo_busy_timer.set(false);
            }
        }

        // 上传状态只在对应 seq 的 ACK/NAK 或无回执超时时变化，避免"已发送"被误显示为成功。
        if ctrl.algo_upload_version() != last_algo_upload_version {
            last_algo_upload_version = ctrl.algo_upload_version();
            ui.set_algo_status(ctrl.algo_upload_status().into());
        }

        // 算法信息回填(version 门控): 只更新信息文本; 编辑器由下方"设备映射表源"块统一载入。
        if ctrl.algo_version() != last_algo_version {
            last_algo_version = ctrl.algo_version();
            let txt = match ctrl.algo_info() {
                Some(i) => format!(
                    "当前算法: {} | PSoC valid={} | len={}B | crc16=0x{:04X}",
                    if i.is_default { "默认(v3.1 HDR)" } else { "自定义" },
                    i.psoc_valid, i.len, i.crc16
                ),
                None => "算法信息未读取".to_string(),
            };
            ui.set_algo_info_text(txt.into());
        }
        // 设备映射表 C 源回读(version 门控)→ 把"当前算法"的可编辑 C 载入编辑器:
        // 设备存了源(上传时随附)→ 精确还原可修改; 设备无源且为默认算法 → 载入内嵌默认模板;
        // 无源且自定义(旧固件/异常)→ 提示并保留编辑器。全程无需机器码反汇编。
        if ctrl.algo_device_src_version() != last_algo_src_version {
            last_algo_src_version = ctrl.algo_device_src_version();
            let dev_src = ctrl.algo_device_src().to_string();
            // ★不覆盖用户正在编辑的文本★: 编辑器只在"空"或"内容仍是上次自动载入的那份"时才自动填。
            // schema/面板已与编辑器解耦(走 algo_schema_source), 所以这里不填也不影响算法面板。
            let editor_now = ui.get_algo_c_source().to_string();
            let editor_untouched = editor_now.trim().is_empty() || editor_now == editor_autoload_mark;
            if !dev_src.trim().is_empty() {
                if editor_untouched {
                    ui.set_algo_c_source(dev_src.clone().into());
                    ui.set_algo_line_numbers(line_numbers_for(&dev_src).into());
                    ui.set_algo_c_bytes(AppController::algo_src_used(&dev_src) as i32);
                    editor_autoload_mark = dev_src.clone();
                    ui.set_algo_status("已从设备映射表载入当前算法 C 源(可直接修改后重新编译上传)".into());
                } else {
                    ui.set_algo_status(
                        "已回读设备算法 C 源(算法面板按设备口径刷新); 编辑器保留你的改动未被覆盖".into());
                }
            } else if ctrl.algo_info().map(|i| i.is_default).unwrap_or(false) {
                // 默认算法设备侧无源 → 用内嵌默认源(★去注释★, 与"上传即保存去注释"语义一致)载入编辑器,
                // 而非展示带注释的原始模板。同时把去注释源回灌设备映射表一次, 使之后"读取信息"真正从设备取回。
                let default_src = AppController::strip_c_comments(ALGO_V31_TEMPLATE);
                if editor_untouched {
                    ui.set_algo_c_source(default_src.clone().into());
                    ui.set_algo_line_numbers(line_numbers_for(&default_src).into());
                    ui.set_algo_c_bytes(AppController::algo_src_used(&default_src) as i32);
                    editor_autoload_mark = default_src.clone();
                }
                if !default_src_synced {
                    let _ = ctrl.send_algo_src(&default_src);
                    default_src_synced = true;
                    ui.set_algo_status("默认算法(v3.1 HDR): 已载入去注释 C 源并回灌设备(下次读取将直接取回设备保存源)".into());
                } else {
                    ui.set_algo_status("默认算法(v3.1 HDR): 已载入去注释 C 源可修改".into());
                }
            } else {
                ui.set_algo_status(
                    "设备未存该算法 C 源(可能为旧固件上传): 无法还原; 可点“加载模板”从默认源改起".into());
            }
        }
        // 反汇编页回填(version 门控): 本地编译产物优先显示 objdump 反汇编;
        // 无本地编译(算法来自设备)时, 回读到的设备机器码以 hex dump 呈现真实 ASM 字节。
        if ctrl.algo_asm_version() != last_algo_asm_version {
            last_algo_asm_version = ctrl.algo_asm_version();
            ui.set_algo_asm(ctrl.algo_asm().into());
        }
        if ctrl.algo_device_code_version() != last_algo_code_version {
            last_algo_code_version = ctrl.algo_device_code_version();
            if ctrl.algo_asm().is_empty() && !ctrl.algo_device_code_hex().is_empty() {
                ui.set_algo_asm(format!(
                    "; 设备当前算法机器码(ASM, hex dump; 无本地编译反汇编时展示)\n{}",
                    ctrl.algo_device_code_hex()
                ).into());
            }
        }
        // 算法追踪回填(version 门控)。★只管算法页自己的等间距窄带★: 单通道精调页的上报折线已
        // 并入主图并与主曲线共享真实时间轴, 必须和主曲线在同一处、用同一时间窗口生成 → 打脏标记
        // 交给下方"主图回填"统一处理, 避免两处各算一套窗口而错位。
        if ctrl.algo_trace_version() != last_algo_trace_version {
            last_algo_trace_version = ctrl.algo_trace_version();
            let active_series = ctrl.algo_trace_active_series();
            // 算法页 90px 窄带用方形 viewbox(x_scale=1.0)。
            let algo_active_path = series_to_svg_path(&active_series, -0.2, 1.2, 1.0);
            ui.set_algo_active_path(algo_active_path.into());
            ui.set_algo_trace_point_count(active_series.len() as i32);
            ui.set_algo_sel_channel(ui.get_sel_channel());
            if !ctrl.plot_frozen() {
                algo_overlay_dirty = true;
            }
        }

        // 算法可调变量(cfg[8])行回填(version 门控): schema 来自当前编辑器源码, 值来自设备缓存。
        if ctrl.algo_cfg_version() != last_algo_cfg_version {
            last_algo_cfg_version = ctrl.algo_cfg_version();
            let settings = ctrl.algo_setting_decls();
            let rows: Vec<AlgoSettingRow> = settings
                .into_iter()
                .map(|d| AlgoSettingRow {
                    idx: d.idx as i32,
                    name: d.name.into(),
                    default_val: d.default as i32,
                    value: ctrl.algo_cfg(d.idx) as i32,
                })
                .collect();
            ui.set_algo_setting_rows(slint::ModelRc::new(slint::VecModel::from(rows)));
        }

        // 全局 CSD 设置回填(version 门控, 避免覆盖用户编辑)
        if ctrl.globals_version() != last_globals_version {
            last_globals_version = ctrl.globals_version();
            if let Some(v) = ctrl.global(1) { ui.set_g_inactive_sns(v as i32); }
            if let Some(v) = ctrl.global(2) { ui.set_g_idac_gain(v as i32); }
            if let Some(v) = ctrl.global(7) { ui.set_g_idac_sense_config(v as i32); }
            if let Some(v) = ctrl.global(8) { ui.set_g_auto_calibrate(v as i32); }
            if let Some(v) = ctrl.global(3) { ui.set_g_idac_min(v as i32); }
            if let Some(v) = ctrl.global(4) { ui.set_g_raw_target(v as i32); }
            if let Some(v) = ctrl.global(5) { ui.set_g_mfs_div_f1(v as i32); }
            if let Some(v) = ctrl.global(6) { ui.set_g_mfs_div_f2(v as i32); }
        }

        // 校准频率偏好回填(草稿优先, 故滑条拖动后立即反映草稿值; 缺省 4)。
        ui.set_calib_pref(match ctrl.config_get("calib.pref").map(|e| e.value) {
            Some(CfgValue::U8(v)) => (v as i32).clamp(1, 7),
            _ => 4,
        });

        // 连接成功边沿主动拉取当前通道参数与 Cp，并开启全通道遥测；
        // 断线边沿清空 CpPollState，避免重连后展示旧设备/旧会话 Cp。
        let connected = ctrl.state() == ConnState::Connected;
        // ★CSD 模式回填: 两个 UI 属性同一个来源★ mode_known 与 scan_mode 全部由
        // csd_mode_effective()(草稿 → 设备真值 → None) 派生, 不再各走一条路。
        // None(尚未取得任何真值) ⇒ 显式 mode_known=false 且 scan_mode 归 0:
        // 旧实现在此用 `if let Some` 直接不赋值, 使 scan_mode 锁存上一次的模式 —— 掉线重连后
        // 页面仍按旧模式渲染, 与 mode_known=false 组合出"下拉说半自动、时钟树说 AUTO"的自相矛盾。
        let csd_mode = ctrl.csd_mode_effective();
        ui.set_mode_known(csd_mode.is_some());
        ui.set_scan_mode(csd_mode.map_or(0, |mode| (mode != 0) as i32));
        if connected && !was_connected {
            let ch = ui.get_sel_channel().clamp(0, 35) as u8;
            ctrl.push_log("已连接 → 自动请求配置、当前通道参数、CH0 全局采样参数 + 开启全通道遥测");
            {
                // Cp 一律不自动获取: 连接边沿只清状态并显示"未测量", 等用户点"测量电容"。
                let mut cp_state = cp_poll_timer.borrow_mut();
                cp_state._stop("未测量".to_string());
                ui.set_cp_text(cp_state.status.clone().into());
            }
            let _ = ctrl.request_config_all();
            let _ = ctrl.request_params(ch);
            // 全局页代表值固定取 CH0；即使精调当前停在其他通道也必须拉取。
            let _ = ctrl.request_params(0);
            // 其余 34 个通道同样需要真值, 否则全通道页/批量应用读到的是空缓存。
            ctrl.schedule_param_refetch_all();
            let _ = ctrl.algo_get_info();
            let _ = ctrl.algo_get_rom();
            let _ = ctrl.request_algo_src();
            let _ = ctrl.request_algo_code();
            for idx in 0u8..8u8 {
                let _ = ctrl.request_algo_cfg(idx);
            }
            let _ = ctrl.global_get_all();
            let _ = ctrl.kbd_request_map();
            let _ = ctrl.kbd_request_combo();
            let _ = ctrl.kbd_request_touchmap();
            let _ = ctrl.kbd_request_hold();
            let _ = ctrl.kbd_request_keycfg();
            let _ = ctrl.kbd_request_state();
            let _ = ctrl.mai2_request_state();
            let _ = ctrl.led_request_state();
            // 遥测降到 30Hz: 100Hz 全 36 通道(~25KB/s)会把 vendor IN 打到 stall(进精调掉线根因)。
            // 30Hz 视觉仍流畅, 大幅降低 vendor IN 负载。
            let _ = ctrl.start_telemetry(
                30,
                FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS | FIELD_STATS | FIELD_LATENCY,
                0xFFFFFFFF_FFFFFFFFu64,
            );
        } else if !connected && was_connected {
            let mut cp_state = cp_poll_timer.borrow_mut();
            cp_state._stop("未连接".to_string());
            ui.set_cp_text(cp_state.status.clone().into());
        }
        // 仅在真正进入已连接的“触控全局调整”页(索引 3)时拉取一次 CH0，避免 16ms tick 洪泛。
        // 逐通道遥测的推流范围 = **整个设置页**, 不再细分到具体 Tab。
        // ★为什么不按 Tab 细分★: 先前收成"只有触控通道调整(2)与单通道精调(4)"两页, 结果
        //  - 分区绑定页的实时触摸态(绿色分区)没数据可用;
        //  - 在设置页内来回切 Tab 会反复 TELEM_START/切档, 单通道精调的曲线与读数出现空档、更新不及时。
        // 通道数据是整个设置页的共同底座, 粒度就该是"在不在设置页"。离开设置页(主页/工具箱/日志/关于)
        // 才收回轻档(仅采样率+延迟), 主页延迟卡照常有数据。
        let want_channel_stream = connected && ui.get_current_view() == 1;
        ctrl.telem_set_scope(want_channel_stream);
        // 遥测自愈: 期望在推流却 >3s 收不到带采样统计的帧就重下发一次 TELEM_START(最快 5s 一次)。
        // ★限流是硬要求★: 本设备所有流量共用一对 bulk 端点, 连发只会打爆 vendor FIFO 并与
        // 校准/自适应抢链路(本仓已因此掉线过) —— 自愈不能反过来把固件带崩。
        ctrl.telem_heal_tick();

        let global_tune_visible = connected && ui.get_current_view() == 1 && ui.get_settings_tab() == 3;
        if global_tune_visible && !last_global_tune_visible && was_connected {
            let _ = ctrl.request_params(0);
            // 时钟树的 snsClk 范围需要全 36 通道的值: 走 PARAM_GET_ALL 的全通道单参数变体, 一帧取回。
            let _ = ctrl.request_param_all_channels(PARAM_SNS_CLK_DIV);
        }
        last_global_tune_visible = global_tune_visible;

        // 协议页(settings_tab == 1)驻留期的灯效采样刷新。
        // ★不得每 16ms 发★: 本设备所有流量共用一对 bulk 端点, 64B vendor FIFO 被高频轮询打爆会掉线
        // (遥测已因此降到 30Hz)。这里取每 12 tick ≈ 192ms(约 5Hz): 色块跟手够用, 负载可忽略。
        // 面板折叠时不轮询 —— 折叠态看不到色块, 没有理由占用链路。
        let protocol_visible = connected && ui.get_current_view() == 1 && ui.get_settings_tab() == 1;
        if protocol_visible {
            // 进页边沿总取一次(折叠态的摘要行也要有真值); 持续轮询只在展开时做。
            if !last_protocol_visible
                || (ui.get_light_panel_expanded() && _every_ms(reconnect_tick, 200))
            {
                let _ = ctrl.led_request_state();
            }
        }
        last_protocol_visible = protocol_visible;

        if connected && _every_ms(reconnect_tick, 1_000) {
            let _ = ctrl.ping();
        }
        // 物理键盘实时态 ~3Hz 轮询,降低 vendor IN 负载(键状态非高频需求)。
        if connected && _every_ms(reconnect_tick, 320) {
            let _ = ctrl.kbd_request_state();
        }
        // 物理键盘页驻留期: 实时三态提到 ~15Hz(要看得出防抖/长按的差别),
        // 边沿记录只在"逻辑分析仪"抽屉展开时才拉(~31Hz, 窗口=1)。
        // ★不放到每 tick★: 全设备共用一对 bulk 端点, 遥测已占 30Hz; 边沿是设备侧缓冲的(192 条),
        // 32ms 拉一次不会丢数据, 却能把这条新增流量压到与既有轮询同量级。
        let phys_kbd_visible =
            connected && ui.get_current_view() == 1 && ui.get_settings_tab() == 6;
        if phys_kbd_visible {
            if !last_phys_kbd_visible {
                let _ = ctrl.kbd_request_keycfg();
            }
            if _every_ms(reconnect_tick, 64) {
                let _ = ctrl.kbd_request_state();
            }
            if ui.get_phys_la_expanded() && _every_ms(reconnect_tick, 32) {
                let _ = ctrl.kbd_request_edges();
            }
        }
        last_phys_kbd_visible = phys_kbd_visible;

        // ★已删除周期性 Cp 轮询★: 原"每 5 tick 轮询一个通道"与用户是否测量无关, 实测把 vendor 端点
        // 打满并与校准/自适应抢链路(NAK 每秒 8~12 条 → endpoint stall → 掉线)。
        // Cp 现在只由手动"测量电容"触发一轮抓取(见下方 CpSweep 推进块)。

        // 算法运行时追踪(report[]/out_active): 选中通道~30Hz(每 tick, 16ms)轮询已声明的
        // report idx(轮转覆盖多个 idx); 未声明任何 ALGO_REPORT 时不轮询(省事务)。
        if connected {
            if ctrl.algo_schema_version() != last_algo_schema_version {
                last_algo_schema_version = ctrl.algo_schema_version();
                algo_report_idxs = ctrl.algo_report_decls().into_iter().map(|d| d.idx).collect();
                algo_report_idxs.sort_unstable();
                algo_report_idxs.dedup();
                algo_trace_rr = 0;
            }
            // 仅在单通道精调页(算法折线叠加所在)且声明了 report 时轮询,
            // 否则(主页/其它页/无算法声明)不发, 从根源杜绝 algo_get_trace NAK 刷屏。
            // 设备端无算法时的 NAK 退避由 app_state 内部处理(见 request_algo_trace)。
            let algo_trace_page = ui.get_current_view() == 1 && ui.get_settings_tab() == 4;
            if algo_trace_page && visual_refresh_tick && !algo_report_idxs.is_empty() {
                let ch = ui.get_sel_channel().clamp(0, 35) as u8;
                let idx = algo_report_idxs[algo_trace_rr % algo_report_idxs.len()];
                let _ = ctrl.request_algo_trace(ch, idx);
                algo_trace_rr = algo_trace_rr.wrapping_add(1);
            }
        }
        was_connected = connected;

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
                ctrl.push_log("保存: 配置已全部下发完毕, mode.work 拓扑切换将在约 0.8s 后自动重启设备生效");
            } else if !reboot_wait_logged {
                reboot_wait_logged = true;
                ctrl.push_log("保存: mode.work 需重启生效 —— 正在等待剩余配置下发完成后再重启, 以免打断在途传输");
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
        if current_config_version != last_config_version {
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
                const RENDERED_GROUPS: [&str; 6] = [
                    "mai2serial 协议参数",
                    "mai2light 协议参数",
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

        let current_channel = ui.get_sel_channel();
        let current_telem_version = ctrl.telem_version();
        let current_plot_version = ctrl.plot_version();
        let current_param_version = ctrl.param_version();
        let channel_changed = current_channel != last_channel;
        let curve_visibility = (ui.get_show_raw(), ui.get_show_bsln(), ui.get_show_diff());
        let curve_visibility_changed = curve_visibility != last_curve_visibility;

        // 通道切换立即刷新该通道参数(单条 PARAM_GET_ALL); Cp 不随通道切换请求。
        if channel_changed && connected {
            let _ = ctrl.request_params(current_channel.clamp(0, 35) as u8);
        }

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
        let curves_visible = ui.get_current_view() == 1 && ui.get_settings_tab() == 4;
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
                algo_report_idxs.len(),
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
                    name: decl.map(|d| d.name.clone()).unwrap_or_default().into(),
                    path: frame.path_of(id).into(),
                    // 声明 + 有数据 + 用户未取消勾选 → 才画在主图上。
                    visible: report_wanted[idx] && frame.count_of(id) > 0,
                    is_binary: frame.binary_of(id),
                    norm_amp: amps[idx],
                    line_color: frame.color_of(id),
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
            let readout = if frame.point_count > 0 {
                format!(
                    "CH{} 当前 raw={} diff={} bsln={} | 纵轴量程 [{} .. {}] | 手指阈值 {} 噪声阈值 {}",
                    current_channel, lr, ld, lb,
                    frame.left.min.round() as i32, frame.left.max.round() as i32,
                    finger_th as i32, noise_th as i32
                )
            } else {
                format!("CH{} 等待遥测数据 | 手指阈值 {} 噪声阈值 {}",
                    current_channel, finger_th as i32, noise_th as i32)
            };
            ui.set_curve_readout(readout.into());
            let spent = prof_chart_start.elapsed().as_micros() as u64;
            prof_chart_us += spent;
            prof_chart_hits += 1;
            prof_chart_max_us = prof_chart_max_us.max(spent);
        }

        // 触控组合映射回填(version 门控)。
        let current_combo_version = ctrl.kbd_combo_version();
        if current_combo_version != last_combo_version {
            last_combo_version = current_combo_version;
            let table = ctrl.kbd_combos();
            let zone_names: Vec<String> = (0..34usize).map(zone_label).collect();
            let zones_text: Vec<slint::SharedString> = table.iter()
                .map(|c| {
                    let names: Vec<&str> = (0..34usize)
                        .filter(|z| (c.zone_mask & (1u64 << z)) != 0)
                        .map(|z| zone_names[z].as_str())
                        .collect();
                    slint::SharedString::from(names.join(" + "))
                })
                .collect();
            let keys_text: Vec<slint::SharedString> = table.iter()
                .map(|c| {
                    let mut parts: Vec<String> = Vec::new();
                    // 修饰位单独成段, 再列具体键 —— 与 kbd_display 的口径一致。
                    for code in c.keycodes.iter().filter(|k| **k != 0) {
                        parts.push(kbd_display(*code, 0));
                    }
                    let base = if parts.is_empty() { "(仅修饰键)".to_string() } else { parts.join(" + ") };
                    if c.modifiers != 0 {
                        slint::SharedString::from(format!("{} [{}]", base, kbd_display(0, c.modifiers)))
                    } else {
                        slint::SharedString::from(base)
                    }
                })
                .collect();
            let delays: Vec<i32> = table.iter().map(|c| c.delay_ms as i32).collect();
            let maxes: Vec<i32> = table.iter().map(|c| c.max_hold_ms as i32).collect();
            ui.set_combo_count(table.len() as i32);
            // 见 last_combo_zones 的说明: 内容没变就不换 model, 以保住行内键框的录制态。
            if zones_text != last_combo_zones {
                last_combo_zones = zones_text.clone();
                ui.set_combo_zones_text(slint::ModelRc::new(slint::VecModel::from(zones_text)));
            }
            ui.set_combo_keys_text(slint::ModelRc::new(slint::VecModel::from(keys_text)));
            ui.set_combo_delay(slint::ModelRc::new(slint::VecModel::from(delays)));
            ui.set_combo_max_hold(slint::ModelRc::new(slint::VecModel::from(maxes)));
            ui.set_combo_zone_used(slint::ModelRc::new(slint::VecModel::from(
                ctrl.kbd_combo_zone_used())));
            ui.set_combo_zone_picked(slint::ModelRc::new(slint::VecModel::from(
                ctrl.kbd_combo_pending_zones())));
            let (pk, pm) = ctrl.kbd_combo_pending_keys();
            let mut pending: Vec<String> = pk.iter().filter(|k| **k != 0)
                .map(|k| kbd_display(*k, 0)).collect();
            if pm != 0 {
                pending.push(format!("[{}]", kbd_display(0, pm)));
            }
            ui.set_combo_pending_keys_text(pending.join(" + ").into());
            // 设备支持性显式区分"旧固件不支持"与"支持但一条都没配", 不让前者被误读成配置丢了。
            let status = match ctrl.kbd_combo_supported() {
                Some(false) => "设备固件不支持组合映射(请更新固件)".to_string(),
                None => "尚未回读设备组合映射".to_string(),
                Some(true) => format!("上限 {} 条 · 单条最多 {} 键",
                    mai2control_ui::proto::KBD_COMBO_COUNT,
                    mai2control_ui::proto::KBD_COMBO_KEY_COUNT),
            };
            ui.set_combo_status(status.into());
        }

        // 批量应用抽屉回填: 勾选态与"每参数待写值"共用 batch_sel_version 门控, 值在 param_version
        // 推进/换通道时补空(★只补空, 不覆盖用户手改过的项★, 见 batch_fill_missing_values)。
        // 全部并入既有 16ms tick 门控, 不新增定时器、不提高频率。
        if ctrl.batch_sel_version() != last_batch_sel_version
            || current_param_version != last_param_version
            || channel_changed
        {
            // 先补空再取版本号: 补空自身会 bump 版本, 顺序反了会多触发一次无谓的行模型重建。
            ctrl.batch_fill_missing_values();
            last_batch_sel_version = ctrl.batch_sel_version();
            ui.set_batch_source(ctrl.batch_source() as i32);
            ui.set_batch_ch_selected(slint::ModelRc::new(slint::VecModel::from(
                ctrl.batch_ch_selected())));
            ui.set_batch_param_selected(slint::ModelRc::new(slint::VecModel::from(
                ctrl.batch_param_selected())));
            // ★与单通道精调共用 build_param_row★: 围栏与单选项(时钟源 0..6+AUTO 等)只有一处派生,
            // 不会再出现"精调页是单选、批量面板还是数字框且能填非法值"这种两处不一致(实测截图)。
            // 值取的是 batch_source 的值而不是当前精调通道的 —— 抽屉里勾目标通道时不该改这一列。
            // -1 = 面板上尚无值(源通道未回读且用户未手填), .slint 侧显示"—", 区别于"值为 0"。
            let values = ctrl.batch_values();
            for (i, id) in BATCH_PARAM_IDS.iter().copied().enumerate() {
                let row = build_param_row(id, values.get(i).copied().unwrap_or(-1));
                // ★只在真的变了才写★ set_row_data 会通知 Repeater 更新该行, 无条件回写等于每
                // 16ms 把用户正在编辑的那一行推回设备值(GuardedSpinBox 有焦点守卫, 但没必要多此一举)。
                if batch_params_model.row_data(i).as_ref() != Some(&row) {
                    batch_params_model.set_row_data(i, row);
                }
            }
        }

        if current_param_version != last_param_version || channel_changed {
            last_param_version = current_param_version;
            let params = ctrl.params_of(current_channel as u8);
            let param_rows = build_param_rows(&params);
            while curve_params_model.row_count() > param_rows.len() {
                curve_params_model.remove(curve_params_model.row_count() - 1);
            }
            for (row, param) in param_rows.into_iter().enumerate() {
                if row < curve_params_model.row_count() {
                    if curve_params_model.row_data(row).as_ref() != Some(&param) {
                        curve_params_model.set_row_data(row, param);
                    }
                } else {
                    curve_params_model.push(param);
                }
            }

            if let Some(finger_th) = ctrl.param(current_channel as u8, PARAM_FINGER_TH) {
                ui.set_finger_th_val(finger_th as i32);
            }
            if let Some(noise_th) = ctrl.param(current_channel as u8, PARAM_NOISE_TH) {
                ui.set_noise_th_val(noise_th as i32);
            }

            // 全局页的三项统一采样设置固定显示 CH0 代表值，不受精调通道影响。
            if let Some(value) = ctrl.param(0, PARAM_SNS_CLK_DIV) {
                ui.set_csd_sns_clk_div(value as i32);
            }
            // ★时钟树 snsClk 改显示全 36 通道范围★: 逐通道自适应后各通道分频不同, CH0 单值不反映任何情况。
            let (div_lo, div_hi) = ctrl.sns_clk_div_range();
            ui.set_csd_sns_clk_div_min(div_lo as i32);
            ui.set_csd_sns_clk_div_max(div_hi as i32);
            if let Some(value) = ctrl.param(0, PARAM_RESOLUTION) {
                ui.set_csd_resolution(value as i32);
            }
            if let Some(value) = ctrl.param(0, PARAM_SNS_CLK_SOURCE) {
                ui.set_csd_sns_clk_source(value as i32);
                // 下标由围栏统一判定: 128 ⇒ AUTO(末项), 非法值 ⇒ -1(下拉留空, 不假装选中某项)。
                ui.set_csd_clk_index(
                    mai2control_ui::proto::param_fence(PARAM_SNS_CLK_SOURCE)
                        .ui_choice_index(value),
                );
            }
        }

        // 全通道实时状态：随遥测版本或 Cp 缓存版本刷新(遥测流全 36 通道；Cp 无新遥测时
        // 也可能因 CP_GET 响应更新，否则 cp_text 会卡在旧值不刷新)。active=status bit0。
        // 原地按行更新(仅变化的行才写)，不替换 model，避免元素重建导致的悬浮闪烁/交互丢失。
        let current_cp_version_all = ctrl.cp_version();
        // 排序/筛选选择与启用态(param_version 覆盖 0x0C 草稿)也要驱动重建: 否则改了下拉框或关掉
        // 一个通道后, 网格要等下一帧遥测才变, 看起来像"点了没反应"。
        let sort_all = ui.get_channel_sort();
        let show_disabled_all = ui.get_channel_show_disabled();
        let current_param_version_all = ctrl.param_version();
        // 同主图: 36 张卡片的格式化+逐行 diff 是另一处遥测驱动的重活, 同样限速到 visual_refresh_tick;
        // 排序/筛选/启用态变化(用户交互)仍立即生效, 不等下一个节拍。
        // 同主图: 36 张卡片只住在"设置"页, 其它页(主页/工具箱/日志/关于)不该为它付格式化与逐行 diff
        // 的代价(实测 ~0.38ms/次, 在主页也几乎每帧命中)。门控跳过时不更新 last_*, 回到该页时
        // 版本判据自然成立并立即重建一次, 不会出现"回来看到旧数据"。
        let channels_page_visible = ui.get_current_view() == 1;
        if channels_page_visible
            && ((current_telem_version != last_telem_version_all && visual_refresh_tick)
                || current_cp_version_all != last_cp_version_all
                || current_param_version_all != last_param_version_all
                || sort_all != last_sort_all
                || show_disabled_all != last_show_disabled_all)
        {
            let prof_cards_start = Instant::now();
            last_telem_version_all = current_telem_version;
            last_cp_version_all = current_cp_version_all;
            last_param_version_all = current_param_version_all;
            last_sort_all = sort_all;
            last_show_disabled_all = show_disabled_all;
            let (all, hidden) = build_channel_status(&ctrl, sort_all, show_disabled_all);
            // 行数会随筛选变化: 数量变了必须整表替换(逐行 set_row_data 只能改已有行, 多出来的
            // 旧行会留在网格里变成幽灵卡片)。数量不变时仍只写变化的行, 保持无闪烁。
            if all_channels_model.row_count() != all.len() {
                all_channels_model.set_vec(all);
            } else {
                for (i, row) in all.into_iter().enumerate() {
                    if all_channels_model.row_data(i).as_ref() != Some(&row) {
                        all_channels_model.set_row_data(i, row);
                    }
                }
            }
            ui.set_channel_hidden_count(hidden);
            prof_cards_us += prof_cards_start.elapsed().as_micros() as u64;
            prof_cards_hits += 1;
        }
        // 采集开关(真停流)与画面冻结是两件事, 这里回填的是前者。
        ui.set_telem_paused(ctrl.telem_user_paused());
        // 单通道精调页的通道启用开关(草稿优先, 与网格卡片同一真相源)。
        ui.set_curve_ch_enabled(ctrl.ch_enabled(ui.get_sel_channel().clamp(0, 35) as u8));

        // 延迟历史折线(总延迟 us),version 门控。绘制完整历史铺满 viewbox(1000宽),
        // 由 UI 侧 viewbox 缩放/横向滚动条查看局部, 不再固定窗口只显示一小段。
        if ctrl.lat_version() != last_lat_version {
            last_lat_version = ctrl.lat_version();
            let series = ctrl.lat_total_series();
            let (path, lo, hi) = build_lat_path(&series);
            ui.set_lat_path(path.into());
            ui.set_lat_y_max(hi);
            ui.set_lat_y_min(lo);
            ui.set_lat_point_count(series.len() as i32);
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
        // mai2serial 发送状态(version 门控): 未读取时显式未知，不能以 false/0 冒充设备真值。
        if ctrl.mai2_version() != last_mai2_version {
            last_mai2_version = ctrl.mai2_version();
            let send_en = ctrl.mai2_send_en();
            ui.set_mai2_send_en_known(send_en.is_some());
            ui.set_mai2_send_en(send_en.unwrap_or(false));
            let status = match ctrl.mai2_status() {
                Some(0) => "停",
                Some(1) => "就绪",
                Some(2) => "运行",
                _ => "未知",
            };
            ui.set_mai2_status(status.into());
            let baud = ctrl
                .mai2_baud()
                .map(|value| format!("{} bps", value))
                .unwrap_or_else(|| "未知".to_string());
            ui.set_mai2_baud(baud.into());
        }
        // mai2light 灯效运行态(version 门控): 回读/草稿编辑/应用结果任一变化才重建 11 行。
        if ctrl.led_version() != last_led_version {
            last_led_version = ctrl.led_version();
            ui.set_light_known(ctrl.led_known());
            ui.set_light_status(match ctrl.led_status() {
                Some(0) => "停",
                Some(1) => "就绪",
                Some(2) => "运行",
                _ => "未知",
            }.into());
            // 两链就绪位与故障分档来自同一状态字高位: 未回读时一律 false/"", UI 显示"未知"而非绿灯。
            ui.set_light_chain0_ready(ctrl.led_chain_ready(0).unwrap_or(false));
            ui.set_light_chain1_ready(ctrl.led_chain_ready(1).unwrap_or(false));
            ui.set_light_init_fault(match ctrl.led_init_fault() {
                Some(1) => "PIO1 初始化失败",
                Some(2) => "灯链0 建链失败(PIO 程序内存/状态机)",
                Some(3) => "灯链1 建链失败(PIO 程序内存/状态机)",
                _ => "",
            }.into());
            ui.set_light_resp_en(match ctrl.led_resp_enabled() {
                Some(true) => "已使能",
                Some(false) => "未使能",
                None => "未知",
            }.into());
            let baud = if ctrl.led_baud_valid() {
                format!("{} bps", ctrl.led_baud().unwrap_or_default())
            } else {
                "未知".to_string()
            };
            ui.set_light_baud(baud.into());
            let rx_frames = if ctrl.led_rx_frames_valid() {
                ctrl.led_rx_frames().to_string()
            } else {
                "未知".to_string()
            };
            ui.set_light_rx_frames(rx_frames.into());
            let sum_errors = if ctrl.led_sum_errors_valid() {
                ctrl.led_sum_errors().to_string()
            } else {
                "未知".to_string()
            };
            ui.set_light_sum_errors(sum_errors.into());
            ui.set_light_units(slint::ModelRc::new(slint::VecModel::from(build_led_unit_rows(&ctrl))));
            ui.set_light_conflict(ctrl.led_region_conflict().unwrap_or_default().into());
            ui.set_light_apply_status(ctrl.led_apply_status().into());
        }
        // 灯链长度/亮度取配置 KV(草稿优先), 与 calib_pref 同口径每帧回填。
        ui.set_light_ws_count0(cfg_u32_or(&ctrl, "led.ws_count0", 0).clamp(0, 1000) as i32);
        ui.set_light_ws_count1(cfg_u32_or(&ctrl, "led.ws_count1", 0).clamp(0, 1000) as i32);
        ui.set_light_brightness(cfg_u32_or(&ctrl, "led.ws_brightness", 0).clamp(0, 255) as i32);

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

        // ★剖面汇报★: 16ms 的 tick 预算里究竟花在哪, 只有实测能回答。每 ~2s 一条。
        let spent = prof_tick_start.elapsed().as_micros() as u64;
        prof_tick_us += spent;
        prof_tick_max_us = prof_tick_max_us.max(spent);
        prof_ticks += 1;
        if prof_ticks >= 120 {
            log::debug!(
                "[PROF] tick n={} avg={}us max={}us | gap avg={}us max={}us | poll avg={}us | chart hits={} avg={}us max={}us | cards hits={} avg={}us | telem={}Hz pts={}",
                prof_ticks,
                prof_tick_us / prof_ticks as u64,
                prof_tick_max_us,
                if prof_gaps > 0 {
                    prof_gap_us / prof_gaps as u64
                } else {
                    0
                },
                prof_gap_max_us,
                prof_poll_us / prof_ticks as u64,
                prof_chart_hits,
                if prof_chart_hits > 0 { prof_chart_us / prof_chart_hits as u64 } else { 0 },
                prof_chart_max_us,
                prof_cards_hits,
                if prof_cards_hits > 0 { prof_cards_us / prof_cards_hits as u64 } else { 0 },
                ctrl.telem_samples_per_sec(),
                ui.get_curve_point_count(),
            );
            prof_ticks = 0;
            prof_tick_us = 0;
            prof_tick_max_us = 0;
            prof_poll_us = 0;
            prof_chart_us = 0;
            prof_chart_hits = 0;
            prof_chart_max_us = 0;
            prof_cards_us = 0;
            prof_cards_hits = 0;
            prof_gap_us = 0;
            prof_gap_max_us = 0;
            prof_gaps = 0;
        }
    });

    // 返回 Timer,由 main() 持有到 ui.run() 结束,防止被 drop 而停止轮询。
    timer
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
fn expected_scan_period_us(ctrl: &AppController) -> i32 {
    const MOD_CLK_MHZ: u64 = 48;
    const CHANNELS: u64 = 36;
    // 每通道固定开销(µs): 真机实测拟合值(传感切换/IMO 稳定/IDAC/IsBusy 轮询/多频扫描等,
    // 与分辨率/分频无关)。使预期贴近实测, 从而分频/分辨率外的突变可显现为真实异常。
    const FIXED_OVERHEAD_US_PER_CH: u64 = 76;
    let res = ctrl.param(0, PARAM_RESOLUTION).unwrap_or(0) as u64;
    if res < 1 || res > 20 {
        return 0;
    }
    let conv_us = (1u64 << res) / MOD_CLK_MHZ; // 换能: 2^res / ModClk(µs), 与 snsClkDiv 无关
    (CHANNELS * (FIXED_OVERHEAD_US_PER_CH + conv_us)).min(i32::MAX as u64) as i32
}

fn build_config_rows(entries: &[ConfigEntry]) -> Vec<ConfigRow> {
    // ★显式排序★: 入参来自按 key 排序的 BTreeMap, 直接渲染就是"英文键名字典序", 与中文标签的
    // 阅读逻辑完全无关。先按组内显式次序排, 同次序再按 key 兜底(保证稳定、不随 HashMap 抖动)。
    // 组之间的顺序由各页面挂 ConfigGroupSection 的先后决定, 不在这里管。
    let mut ordered: Vec<&ConfigEntry> = entries.iter().collect();
    ordered.sort_by(|a, b| {
        config_display_order(&a.key)
            .cmp(&config_display_order(&b.key))
            .then_with(|| a.key.cmp(&b.key))
    });
    ordered
        .into_iter()
        // ★渲染归属集中在这里★ 每个 KV 只允许出现在一个页面: 有专用编辑器的 key 一律在此滤掉,
        // 其余按 parse_config_label 给出的 group 决定落在"通信系统"Tab 还是"协议"页的对应块。
        // bind.map* → 分区绑定页; kbd.* → 键盘页; calib.* → 触控全局调整页的偏好滑条;
        // led.map* → 协议页 11 单元映射可视化编辑器(裸 KV 是打包 u32, 暴露出来只会被误改);
        // led.ws_count0/1 与 led.ws_brightness → 协议页 mai2light 块已有专用 SpinBox。
        .filter(|entry| {
            let k = entry.key.as_str();
            !k.starts_with("bind.")
                && !k.starts_with("kbd.")
                && !k.starts_with("calib.")
                && !k.starts_with("led.map")
                && !matches!(k, "led.ws_count0" | "led.ws_count1" | "led.ws_brightness")
        })
        .map(|entry| {
            let (
                mut kind,
                type_code,
                bool_val,
                num_val,
                mut min_val,
                mut max_val,
                mut has_range,
                mut enum_index,
                str_val,
            ) = match &entry.value {
                CfgValue::Bool(v) => (0, 0, *v, 0.0, 0.0, 1.0, false, 0, "".to_string()),
                CfgValue::U8(v) => (1, 2, false, *v as f32, 0.0, 255.0, true, 0, "".to_string()),
                CfgValue::U16(v) => (
                    1,
                    3,
                    false,
                    *v as f32,
                    0.0,
                    65535.0,
                    true,
                    0,
                    "".to_string(),
                ),
                CfgValue::U32(v) => (
                    1,
                    4,
                    false,
                    *v as f32,
                    0.0,
                    4294967295.0,
                    true,
                    0,
                    "".to_string(),
                ),
                CfgValue::I8(v) => (
                    1,
                    1,
                    false,
                    *v as f32,
                    -128.0,
                    127.0,
                    true,
                    0,
                    "".to_string(),
                ),
                CfgValue::F32(v) => (1, 5, false, *v, 0.0, 1.0, false, 0, "".to_string()),
                CfgValue::Str(v) => (3, 6, false, 0.0, 0.0, 0.0, false, 0, v.clone()),
            };

            // ★围栏必须用设备自报的 range, 不许按类型猜★
            // 上面那张表只知道"这是个 U16", 于是给出 0..65535; 而固件 schema 对同一个 key 声明的
            // 合法区间可能窄得多(例: 触控延迟只收 0..1000)。界面围栏比固件宽 = 放行必被 NAK 的值,
            // 用户失焦夹取也夹不到正确阈值上。schema 带 range 就以它为准, 否则才回落类型上限。
            if let Some((lo, hi)) = &entry.range {
                if let (Some(lo), Some(hi)) = (lo.as_f32(), hi.as_f32()) {
                    if hi >= lo {
                        min_val = lo;
                        max_val = hi;
                        has_range = true;
                    }
                }
            }

            // schema 未携带颜色枚举选项；这四个 U8 配置使用专用 ComboBox，
            // 但数值仍原样沿用 set_config_number → CfgValue::U8 的既有链路。
            if let CfgValue::U8(value) = &entry.value {
                if matches!(
                    entry.key.as_str(),
                    "led.color_connected"
                        | "led.color_flash_error"
                        | "led.color_link_error"
                        | "led.color_healthy"
                ) {
                    kind = 4;
                    enum_index = (*value).min(7) as i32;
                }
                // mode.work 是 U8 0..1(Serial/HID)枚举, 用 kind=2 ComboBox 而非数字输入。
                if entry.key.as_str() == "mode.work" {
                    kind = 2;
                    enum_index = (*value).min(1) as i32;
                }
            }

            let (group, label, desc) = parse_config_label(&entry.key);

            // 通信系统配置一律十进制展示(hex 口径仅用于需寄存器处理的 CSD 内容, 见触控全局调整)。
            // hex_val 保留为空(不再走 hex 输入); range_hex 复用为十进制取值范围小字, 便于新人上手。
            let is_num_int = kind == 1 && matches!(type_code, 1 | 2 | 3 | 4);
            let hex_val = String::new();
            let range_hex = if is_num_int && has_range {
                let lo = (min_val as f64).round() as i64;
                let hi = (max_val as f64).round() as i64;
                format!("取值范围 {}–{}", lo, hi)
            } else {
                String::new()
            };

            ConfigRow {
                key: entry.key.clone().into(),
                label: label.into(),
                desc: desc.into(),
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
                hex_val: hex_val.into(),
                range_hex: range_hex.into(),
            }
        })
        .collect()
}

/// 造一行参数编辑数据(标签 + 围栏 + 单选项)。
/// ★单通道精调与全通道批量面板共用本函数★: 围栏与单选项只有这一处派生, 不许在 .slint 里按
/// param_id 硬编码 —— 那会成为与两处固件必然漂移的第三份定义(围栏当年就是这么漂移出
/// "SNS_CLK_SOURCE 上界写死 6 而设备合法持有 128"那个 bug 的)。
/// `value` 允许为 -1: 批量面板用它表示"面板上尚无值", 界面显示"—"。
fn build_param_row(param_id: u8, value: i32) -> ParamRow {
    let fence = mai2control_ui::proto::param_fence(param_id);
    let choices = fence.ui_choices();
    let choice_labels: Vec<slint::SharedString> =
        choices.iter().map(|(t, _)| t.as_str().into()).collect();
    let choice_values: Vec<i32> = choices.iter().map(|(_, v)| *v as i32).collect();
    ParamRow {
        param_id: param_id as i32,
        label: param_display_name(param_id).into(),
        value,
        min: fence.ui_min() as i32,
        max: fence.ui_max() as i32,
        choices: slint::ModelRc::new(slint::VecModel::from(choice_labels)),
        choice_values: slint::ModelRc::new(slint::VecModel::from(choice_values)),
        // 无值(-1)时不指向任何一项, 界面显示"—"而不是假装选中第一项。
        choice_index: if value < 0 {
            -1
        } else {
            fence.ui_choice_index(value as u32)
        },
        // 元数据与围栏同源(见 proto::telemetry::ParamFence 的 unit/scope/help/impact):
        // 界面只显示这两个字符串, 不按 param_id 自己拼说明 —— 那就是第二份元数据。
        unit: fence.unit.into(),
        help: fence.help_text(param_id).into(),
    }
}

/// 组内显示次序。★为什么必须显式给★
/// `config_entries()` 来自按 key 排序的 BTreeMap ⇒ 界面顺序 = **英文 key 的字典序**。
/// 于是协议页会排成"聚合延迟 → 额外重发 → 触控映射… → 速率上限 → 采样延迟 → 仅变化时发送 →
/// 触控串口波特率…"这种与阅读逻辑毫无关系的顺序(用户看到的是中文标签, 字典序按的却是英文键名)。
/// 这里给每个 key 一个显式次序: 先"这条链路是什么"(波特率/节点), 再"怎么发"(延迟), 最后"善后行为"。
/// 未收录的 key 一律排到末尾(999)并按 key 兜底, 新增 KV 不会插到中间打乱既有顺序。
fn config_display_order(key: &str) -> u16 {
    match key {
        // —— mai2serial: 链路 → 上报延迟 → RSET 善后 ——
        "comm.serial_baud" => 10,
        "comm.touch_delay_100us" => 20,
        "comm.keyboard_map_serial_only" => 30,
        "comm.serial_reset_baseline" => 40,
        "comm.serial_reset_calibrate" => 41,
        // —— mai2light: 链路 → 节点 → 灯珠总数 ——
        "comm.light_baud" => 10,
        "led.node_id" => 20,
        "led.count" => 30,
        // —— 键盘映射 ——
        "comm.keyboard_map_en" => 10,
        "comm.keyboard_delay_100us" => 20,
        // —— 状态指示灯: 总开关 → 亮度 → 四个颜色 ——
        "led.enable" => 10,
        "led.status_brightness" => 20,
        "led.color_connected" => 30,
        "led.color_healthy" => 31,
        "led.color_link_error" => 32,
        "led.color_flash_error" => 33,
        // —— 工作模式 ——
        "mode.work" => 10,
        _ => 999,
    }
}

/// 返回 (分组中文名, 配置项中文名, 小字说明)。未收录的 key 回退到英文 key 的可读形式。
fn parse_config_label(key: &str) -> (String, String, String) {
    let parts: Vec<&str> = key.split('.').collect();
    // ★group 就是渲染位置★ 各 Slint 端的 ConfigGroupSection 用 group_title 精确匹配取行,
    // 所以归属按语义逐 key 指定, 而不是按 key 的一级前缀 —— comm./led. 前缀下同时混着
    // "协议能力参数"(属协议页)与"键盘映射/状态指示灯"(属通信系统 Tab), 前缀分不开。
    let group = match key {
        // mai2serial: 串口本身 + 触控上报延迟线 + {E}RSET 善后行为 → 协议页 mai2serial 块
        // ★已删掉 sample_delay_ms / aggregation_delay_ms / extra_send / rate_limit_* /
        //   send_only_on_change★: 固件里没有任何消费点(见 app_config.cpp 该处注释), 已从 schema 移除。
        //   "采样延迟"与"触控延迟"两个名字都像延迟, 正是混淆来源; 现在只保留真正平移上报的那一个。
        "comm.serial_baud"
        | "comm.touch_delay_100us"
        | "comm.serial_reset_calibrate"
        | "comm.serial_reset_baseline"
        // 「触控映射仅协议启动时生效」的判定依据就是 mai2serial 的实际发送态,
        // 语义上属协议能力而非键盘映射本身, 故归到协议页 mai2serial 块。
        | "comm.keyboard_map_serial_only" => "mai2serial 协议参数",
        // mai2light: 灯板串口 + 节点号 + 灯珠总数 → 协议页 mai2light 块
        "comm.light_baud" | "led.node_id" | "led.count" => "mai2light 协议参数",
        // 触控 → 键盘映射: 虽在 comm. 前缀下, 语义与协议无关
        "comm.keyboard_map_en" | "comm.keyboard_delay_100us" => "键盘映射",
        // 板载状态指示灯(main.cpp 心跳灯消费), 与灯板协议无关
        "led.enable"
        | "led.status_brightness"
        | "led.color_connected"
        | "led.color_flash_error"
        | "led.color_link_error"
        | "led.color_healthy" => "状态指示灯",
        "mode.work" => "工作模式",
        // 未收录 key 的兜底: 按前缀落到通信系统 Tab 的"其他"组, 不会凭空消失。
        _ => match parts.first().copied().unwrap_or("") {
            "bind" => "绑区",
            _ => "其他",
        },
    }
    .to_string();

    let (label, desc): (&str, &str) = match key {
        // 上面 6 个空壳项(采样延迟/仅变化时发送/聚合延迟/额外重发/速率限制)已从固件 schema 删除,
        // 故标签也一并删除 —— 留着标签只会在将来有人误以为"设备支持只是没显示"。
        "comm.keyboard_map_en" => ("启用触摸→键盘", "把触摸分区映射为键盘按键输出"),
        "comm.keyboard_map_serial_only" => (
            "触控映射仅协议启动时生效",
            "需先开启「启用触摸→键盘」；开启后触摸→键盘映射只在 mai2serial 实际发送触控数据期间生效，停发时映射失效并释放已按下的键",
        ),
        "comm.serial_baud" => ("触控串口波特率", "游戏触控串口 (COM) 的波特率"),
        "comm.serial_reset_calibrate" => (
            "串口重启后自动 IDAC 校准",
            "收到 mai2serial 重启指令({E} RSET)后，自动执行一次全通道 IDAC 校准",
        ),
        "comm.serial_reset_baseline" => (
            "串口重启后自动基线复位",
            "收到 mai2serial 重启指令({E} RSET)后，自动执行一次全通道基线复位",
        ),
        "comm.light_baud" => ("灯板串口波特率", "灯板通信串口的波特率"),
        "comm.touch_delay_100us" => ("触控延迟 (×100µs)", "触控串口上报延迟线, 0..100ms"),
        "comm.keyboard_delay_100us" => ("键盘延迟 (×100µs)", "触摸→键盘输出的附加延迟"),
        "mode.work" => ("工作模式", "Serial=游戏串口; HID=触摸屏+键盘"),
        "led.enable" => ("启用 LED", "关闭则运行时 LED 静默(启动序列不受影响)"),
        "led.node_id" => ("灯节点 ID", "灯串协议的节点编号"),
        "led.count" => ("灯珠数量", "WS2812 灯珠个数"),
        "led.status_brightness" => ("状态亮度", "状态指示灯亮度，0=熄灭，255=最亮"),
        "led.color_connected" => ("已连接颜色", "主机连接正常时的状态灯颜色"),
        "led.color_flash_error" => ("Flash 错误颜色", "配置或存储错误时的状态灯颜色"),
        "led.color_link_error" => ("链路错误颜色", "PSoC 链路异常时的状态灯颜色"),
        "led.color_healthy" => ("健康颜色", "传感器与系统正常时的状态灯颜色"),
        _ => ("", ""),
    };

    let label = if label.is_empty() {
        parts.get(1).copied().unwrap_or(key).replace('_', " ")
    } else {
        label.to_string()
    };
    (group, label, desc.to_string())
}

/// 把 Cp(fF) 缓存值换算成两位小数 pF 文本，语义与曲线页一致：
/// None=本会话尚未测量(无任何自动获取)，Some(0)=设备尚未给出结果(测量中)，
/// Some(CP_MEASURE_FAILED)=★唯一的失败判据★(MEASURE_CP 后设备回读 0x00FFFFFF)，其余=正常测量值。
///
/// ★不从别处推断 Cp 失败★: 非激活电极接法、railed 通道数、采样率这些都不是 Cp 失败的证据,
/// 历史上按它们猜出来的警告只会把用户引到错误的排查方向。
fn cp_display_text(cp: Option<u32>) -> String {
    match cp {
        None => "未测量".to_string(),
        Some(0) => "测量中…".to_string(),
        Some(CP_MEASURE_FAILED) => CP_FAILURE_TEXT.to_string(),
        Some(value) => format!("{:.2} pF", value as f64 / 1000.0),
    }
}

/// Cp 失败的**唯一**文案。单通道显示、批量汇总、全局调整页警告三处共用同一句,
/// 免得同一个故障在三处被描述成三种不同的原因。
const CP_FAILURE_TEXT: &str = "测量失败：可能短接或电容过大";

/// 收集 Cp 失败通道(判据只有 `CP_MEASURE_FAILED`), 返回形如 `CH0 CH5` 的列表文案。
/// None/0(测量中)/GND 接法/raw 满量程/frozen/采样率一律不参与 —— 它们不是 Cp 失败的证据。
fn cp_failed_list(ctrl: &AppController) -> String {
    (0..36u8)
        .filter(|&ch| ctrl.cp(ch) == Some(CP_MEASURE_FAILED))
        .map(|ch| format!("CH{}", ch))
        .collect::<Vec<_>>()
        .join(" ")
}

/// 绑区页 34 分区静态几何 + 实时绑定/触摸/Cp 状态，几何来自 DXF 生成的
/// [`touch_geometry::ZONE_GEOMETRY`]，坐标系固定为 SCREEN_W x SCREEN_H。
fn build_zone_cells(ctrl: &AppController) -> Vec<ZoneCell> {
    let waiting_zone = ctrl
        .bind_progress()
        .and_then(|(zone, status)| (status == 0).then_some(zone));
    let mut cells = Vec::with_capacity(34);
    for i in 0..34usize {
        let label = zone_label(i);
        let ring = label.chars().next().unwrap_or('?').to_string();
        let ch = ctrl.binding_channel_of(i);
        let channel = if ch == 0xFF { -1 } else { ch as i32 };
        let touched = channel >= 0
            && ctrl
                .telem_latest(channel as u8)
                .and_then(|sample| sample.status)
                .map(|status| (status & 0x01) != 0)
                .unwrap_or(false);
        let geometry = &touch_geometry::ZONE_GEOMETRY[i];
        let cp_text = if channel >= 0 {
            cp_display_text(ctrl.cp(channel as u8))
        } else {
            "未绑定".to_string()
        };

        cells.push(ZoneCell {
            index: i as i32,
            label: label.into(),
            ring: ring.into(),
            channel,
            binding_text: if channel >= 0 {
                format!("CH{}", channel).into()
            } else {
                "未绑定".into()
            },
            cp_text: cp_text.into(),
            touched,
            binding_active: waiting_zone == Some(i as u8),
            path: geometry.path.into(),
            path_x: geometry.min_x,
            path_y: geometry.min_y,
            path_width: geometry.width,
            path_height: geometry.height,
            label_x: geometry.label_x,
            label_y: geometry.label_y,
        });
    }
    cells
}

// ============================================================================
// 单通道精调主图: 唯一绘图管线
// ============================================================================
//
// ★不变量(改这块之前先读这段)★
// 「同一张图内所有系列共享唯一 x 定义域与唯一绘制面积; 任何新增系列只能加入本管线,
//   不得自带坐标计算」
//
// 旧实现把主曲线(CurvePaths)与算法叠加线(AlgoOverlay)做成两套产物, 后者自带 x 时间窗与量程,
// 而主曲线的 x 窗只由 raw/bsln/diff 求出。算法追踪点是主机轮询取回的, 时间范围与遥测缓冲并不
// 一致 ⇒ 落在主曲线时间窗之外的那一段被裁掉, 表现为"红线只画一半、左边一块空窗、线到那儿就
// 消失"。★根子不在坐标轴, 在两套产物各算一套"有效绘制面积"。★ 现在 x 定义域由【全部参与绘制
// 的系列】一次求并集算定后分发, 缺口阈值同样只在这一处给出: 某系列在某段无数据 ⇒ 那一段断线
// (gap), 而不是把整条线压缩到局部宽度。
//
// ★左右双 Y 轴是正当设计, 不要合并★: 左轴是 ADC 量纲(raw/bsln/diff 与阈值线), 右轴是算法量纲
// (计数/permille/布尔)。共用一轴会把 0/1 的布尔线压成贴底直线。统一的只有 x 定义域与绘制面积
// —— 两轴各有自己的 y 量程, 但横向对位逐像素一致。

/// ★同一张图内的唯一调色板★。容量 32 —— 当前图只用到 8 条(主曲线 3 + 算法上报 4 + 触发判定 1),
/// 留足余量是为了让"同图颜色永不重复"这条不变量在系列数增长时**不必再改代码**
/// (旧版容量恰好等于 8, 一加系列就立刻溢出到灰色降级)。
///
/// 前 8 项顺序与旧版逐项一致(蓝 raw / 琥珀 bsln / 绿 diff / 紫 / 青 / 近白 / 珊瑚 / 品红 触发判定),
/// 保持既有观感不变; 第 9 项起按黄金角(0.618)散布色相补足。
///
/// ★两条可复核的量化约束★(用脚本算过, 改表时请重算, 别凭眼睛加色):
///  - 最小两两 RGB 欧氏色距 = 55.3 —— 保证任意两条线不会被看成同色;
///  - 最低亮度(0.299R+0.587G+0.114B) = 112 —— 图底是 #1c2126(亮度约 32), 低于此值的色在深色底上看不清,
///    故所有色都设了亮度下限; 这也是为什么表里没有深蓝/深紫那类"色距够但看不见"的颜色。
const SERIES_PALETTE: [(u8, u8, u8); 32] = [
    (0x33, 0x99, 0xff), // 蓝(raw)
    (0xff, 0xaa, 0x00), // 琥珀(bsln)
    (0x33, 0xcc, 0x33), // 绿(diff)
    (0xb0, 0x8c, 0xff), // 紫
    (0x00, 0xd4, 0xc8), // 青
    (0xdf, 0xe8, 0xf0), // 近白
    (0xff, 0x7a, 0x45), // 珊瑚
    (0xff, 0x33, 0x99), // 品红(触发判定)
    (0xff, 0x84, 0x9e),
    (0xd5, 0xf2, 0x60),
    (0xc9, 0x2f, 0xd8),
    (0x84, 0xff, 0xd0),
    (0x65, 0x60, 0xf2),
    (0x19, 0xda, 0xff),
    (0x93, 0xd8, 0x2f),
    (0xff, 0x84, 0xf0),
    (0xf2, 0xae, 0x60),
    (0x8a, 0xff, 0x84),
    (0x60, 0xbd, 0xf2),
    (0xcc, 0xd8, 0x2f),
    (0x19, 0xff, 0x89),
    (0x19, 0xff, 0x3d),
    (0xf2, 0x60, 0x74),
    (0xfc, 0x19, 0xff),
    (0xff, 0xf8, 0x84),
    (0xff, 0x3b, 0x19),
    (0x6b, 0xff, 0x19),
    (0x2f, 0xd8, 0xaa),
    (0xff, 0xf9, 0x19),
    (0x60, 0xf2, 0x62),
    (0x60, 0xf2, 0xa7),
    (0xd5, 0x60, 0xf2),
];

/// 调色板耗尽后的降级色(灰): 明确表示"这条线没分到专色", 而不是静默与别人同色。
fn series_fallback_color() -> slint::Color {
    slint::Color::from_rgb_u8(0x9e, 0x9e, 0x9e)
}

/// 按"系列在图中的出现顺序"取色的游标。
/// ★递增游标而不是 `idx % len`★: 取模在系列数超过表长时会静默让两条线同色, 看图的人无法分辨
/// 谁是谁; 游标只增不减 ⇒ 同一张图内颜色永不重复, 超出容量的部分明确降级并由调用方记一条日志。
struct PaletteCursor {
    _next: usize,
    _overflow: usize,
}

impl PaletteCursor {
    fn new() -> Self {
        Self {
            _next: 0,
            _overflow: 0,
        }
    }
    /// 取下一个颜色。★分配顺序固定为图内声明顺序, 与该系列是否可见无关★ ——
    /// 否则勾掉一条线会让它后面所有线换颜色。
    fn take(&mut self) -> slint::Color {
        match SERIES_PALETTE.get(self._next) {
            Some(&(r, g, b)) => {
                self._next += 1;
                slint::Color::from_rgb_u8(r, g, b)
            }
            None => {
                self._overflow += 1;
                series_fallback_color()
            }
        }
    }
    /// 未分到专色的系列数(>0 ⇒ 调色板容量该扩了, 绝不靠重复颜色顶过去)。
    fn overflow(&self) -> usize {
        self._overflow
    }
}

/// 图内一条系列的身份: 决定它走哪根 Y 轴、按哪种采样节奏判缺口、回填到哪个 UI 属性。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SeriesId {
    Raw,
    Bsln,
    Diff,
    /// 算法上报 report[idx](idx 0..3)。
    Report(u8),
    /// 算法触发判定 out_active。
    Active,
}

impl SeriesId {
    /// true = 走右轴(算法量纲), false = 走左轴(ADC 量纲)。
    fn on_right_axis(self) -> bool {
        !matches!(self, SeriesId::Raw | SeriesId::Bsln | SeriesId::Diff)
    }
    /// 采样节奏: 上报/触发判定是主机轮询取回的, 单条线的采样间隔是遥测周期的若干倍。
    fn cadence(self) -> SeriesCadence {
        if self.on_right_axis() {
            SeriesCadence::Poll
        } else {
            SeriesCadence::Frame
        }
    }
}

/// 采样节奏 ⇒ 缺口阈值取哪一档(两档都在 `XWindow` 一处算定, 见其注释)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SeriesCadence {
    /// 遥测直出: 每帧一点。
    Frame,
    /// 主机轮询: n 个 report idx 轮转, 单条线的间隔 = 遥测周期 × n。
    Poll,
}

/// 采样间隔超过多少视为"时间缺口"(暂停/掉帧): 取标称周期的若干倍, 并给一个绝对下限,
/// 免得采样率未知(sps=0)或抖动时误判。缺口段按 0 绘制, 见 `fill_uncovered_with_zero`。
fn gap_threshold_us(sample_rate_hz: u32, period_mult: u32, floor_us: u64) -> u64 {
    let nominal = if sample_rate_hz > 0 {
        1_000_000 / sample_rate_hz as u64
    } else {
        33_333
    };
    (nominal * period_mult as u64).max(floor_us)
}

/// 整张图【唯一】的 x 定义域(展开后的设备时间 us)与【唯一】的缺口判据。
/// 由 `build_chart_frame` 在一处算定后分发给每条系列, 任何系列不得自己再算 x。
///
/// ★为什么缺口阈值是两档而不是一个数★: 主曲线每帧一点, 算法追踪是 n 个 idx 轮转取回,
/// 单条线的采样间隔本就相差 n 倍。同一个阈值必然错一头: 取小的会把算法线逐点切碎成归零段,
/// 取大的会让主曲线上真实的停流被直线插值伪造成"这段时间数值在变化"。故按采样节奏分两档,
/// 但两档都在这里一次算定 —— 系列自己不许算。
///
/// ★阈值的语义 = "覆盖半径"★: 与某个采样点时间距离在阈值内的时刻算被该采样覆盖(正常的帧间
/// 抖动/一个轮询周期都在此内); 超出即视为**没有数据**, 由 `fill_uncovered_with_zero` 按 0 补齐。
struct XWindow {
    /// 窗口左端时刻(= `t_last_us − PLOT_WINDOW_US`, 设备上电不足一窗时被 0 截住) → viewbox x = 0。
    t_first_us: u64,
    /// 最新样本时刻 → viewbox x = 1000。
    t_last_us: u64,
    /// 遥测节奏(每帧一点)的缺口阈值。
    gap_frame_us: u64,
    /// 轮询节奏(n 个 report idx 轮转)的缺口阈值。
    gap_poll_us: u64,
}

impl XWindow {
    /// ★整张图唯一的 x 映射★: 设备时刻 → viewbox x(0..1000)。所有系列必须经由它。
    /// x 值域固定 [0,1000]: 主图 PlotPath 用 fit: fill(非等比拉伸), viewbox 无论什么宽高比都被
    /// 拉满绘图区, 故不需要按绘图区宽高比预拉伸 —— 那套修正在横向缩放后必然失配并留出空白带。
    /// ★结果夹在 [0,1000]★ 与 `AxisScale::y_of` 同一处理。补齐后的系列本就恰好铺满
    /// [t_first_us, t_last_us], 夹取对它们是恒等变换; 它挡住的是退化窗口(整张图一条参与系列都
    /// 没数据 ⇒ 窗口塌成 0..0)下未参与系列被顺带投影时算出的天文数字坐标 —— 那会让下游的
    /// 虚线分段循环按 i32::MAX 跑。
    /// ★横向尺度恒为 `PLOT_WINDOW_US`★: 不用 `t_last - t_first` 当分母 —— 设备刚上电不足
    /// 30s 时 `t_first_us` 会被 0 截住, 那时按实际跨度换算又会变成"边跑边缩放"的动态尺度。
    /// 以右端为锚、固定跨度反推左端, 任何时刻 1 像素都等于同一时长。
    fn x_of(&self, t_us: u64) -> f32 {
        let span = PLOT_WINDOW_US as f32;
        let left = self.t_last_us as f32 - span;
        ((t_us as f32 - left) / span * 1000.0).clamp(0.0, 1000.0)
    }
    fn gap_us(&self, cadence: SeriesCadence) -> u64 {
        match cadence {
            SeriesCadence::Frame => self.gap_frame_us,
            SeriesCadence::Poll => self.gap_poll_us,
        }
    }
    /// 全量时间跨度(ms) —— 固定值, 与 `x_of` 用的是同一个跨度。
    fn t_span_ms(&self) -> f32 {
        PLOT_WINDOW_US as f32 / 1000.0
    }
}

/// 一根 Y 轴的量程(已含两侧 5% 留白)。左轴 = ADC 量纲, 右轴 = 算法量纲。
struct AxisScale {
    min: f32,
    mid: f32,
    max: f32,
}

impl AxisScale {
    /// 无参与系列时的退化量程: 不能是 0 宽, 否则 y 映射除零。
    fn unit() -> Self {
        Self {
            min: 0.0,
            mid: 0.5,
            max: 1.0,
        }
    }
    /// 由参与系列的取值范围求量程。两侧各留 5%, 免得极值贴边看不出来。
    /// `flat_floor` = 全平序列(min==max)时的最小留白: 左轴取 1(ADC 最小有意义刻度是 1 个计数),
    /// 右轴取 0.5(算法量纲可以是 0/1 布尔, 再大就把一条平线撑到看不出高度差)。
    fn from_span(min: f32, max: f32, flat_floor: f32) -> Self {
        if !min.is_finite() || !max.is_finite() || min > max {
            return Self::unit();
        }
        let span = max - min;
        let padding = if span.abs() < f32::EPSILON {
            (min.abs() * 0.05).max(flat_floor)
        } else {
            span * 0.05
        };
        let lo = min - padding;
        let hi = max + padding;
        Self {
            min: lo,
            mid: (lo + hi) * 0.5,
            max: hi,
        }
    }
    /// 值 → viewbox y(0..1000, 向下增大)。
    fn y_of(&self, value: f32) -> f32 {
        let range = (self.max - self.min).max(f32::EPSILON);
        (1000.0 - (value - self.min) / range * 1000.0).clamp(0.0, 1000.0)
    }
}

/// 补齐后的一个绘图点。
///
/// `dashed = true` 表示该点是"此处无采样覆盖"合成出来的点: 取值沿用相邻真实采样的值(边缘保持),
/// 画成虚线。★不再合成 0 值★: 无覆盖是"不知道", 不是"设备报了 0"; 拿 0 去补会把该轴量程拉到 0,
/// 于是同一条线在有/无缺口时纵向尺度完全不同(实测: 进页面瞬间量程从 [490..542] 跳成 [0..542])。
#[derive(Clone, Copy)]
struct PlotPoint {
    t_us: u64,
    val: f32,
    dashed: bool,
}

/// 无覆盖段的"虚线"节拍(viewbox x 单位, 值域 0..1000)。
///
/// ★为什么用手工虚线而不是另开一条低透明度路径★: 另开路径要给 8 条系列各加一个 UI 元素与
/// 一份颜色, 等于把"一条系列一个 path"的模型撑成两份, 且 .slint 侧要再复制一遍描边参数。
/// 在同一条 path 里把无覆盖段拆成 on/off 小段, 颜色/线宽天然与本系列一致(仍取自 SERIES_PALETTE),
/// UI 一行都不用改, 而"虚线 = 这段没数据"与"实线 = 真实采样"一眼可分。
const DASH_ON: i32 = 9;
const DASH_OFF: i32 = 7;

/// 一条系列的最终产物。★除 path 外不带任何坐标信息★: 坐标全在 `XWindow`/`AxisScale` 里,
/// 谁也不能凭这个结构自己再投影一遍。
struct SeriesPath {
    id: SeriesId,
    path: String,
    /// 线条/图例颜色, 由本图唯一调色板顺序分配(见 `SERIES_PALETTE`)。
    color: slint::Color,
    /// 本系列的采样点数(0 ⇒ UI 可退化成基线, 或整行收起)。
    point_count: i32,
    /// 取值只有 0/1(布尔类) ⇒ UI 才为它显示"归一化幅度 N"输入。
    binary: bool,
}

/// 单通道精调主图一次回填的【唯一】产物: 一个函数一次产出全部系列。
///
/// ── 结构性不变量(由 `build_chart_frame` + `fill_uncovered_with_dashes` 联合保证) ──────────
/// 「同一张图内所有系列共享唯一 x 定义域(`ChartFrame::x`), 且每条**有采样**的系列都铺满整个
///  定义域: 无采样覆盖处沿用相邻采样的值并画成虚线, 既不补 0、不插值, 也不允许提前截断。」
///
/// 展开说明:
///  · 唯一 x: 只有 `XWindow::x_of` 一处做时间→横坐标映射, 任何系列不得自带 x 计算;
///  · 铺满定义域: 左端(早于首样本)、中间缺口、右端(晚于末样本)三种无覆盖区间全部补成虚线段。
///    ★这是"各条线能画的范围不一样"的唯一修法★ —— 遥测缓冲与算法追踪缓冲的起止时刻天然不同
///    (后者只在本页驻留时才轮询), 若各画各的, 缩放/切子标签后就会看到一条只占左半、一条只占
///    右半(实测截图); 现在每条线的 x 覆盖恒等于窗口本身, 差异只体现为实线/虚线;
///  · 虚线取边缘值而不是 0: 无覆盖是"不知道"。补 0 会把该轴量程拉到 0, 让同一条波形的纵向尺度
///    随缺口有无而变(那正是"归一化"要消掉的抖动);
///  · 垂直阶跃: 虚线段与真实数据之间用同一时刻的两个点表达跳变, 绝不画斜线 —— 斜线会伪造
///    "数值在这段时间里连续过渡"的假象;
///  · 不提前截断: 每条系列路径的 x 覆盖与 `x` 一致, 不会出现"叠加线只盖住图的左半边";
///  · 虚实可分: 无覆盖段是同色虚线(见 `DASH_ON`), 真实采样一律实线 —— 包括真值 0。
/// 空系列(该系列一个采样都没有)例外: 它不参与绘制, path 为空 —— 无数据的系列不该凭空多出一条
/// 贴底的线, 它的"没有数据"由 `point_count == 0` 表达, UI 据此收起整行。
struct ChartFrame {
    /// 唯一 x 定义域 + 唯一缺口判据。
    x: XWindow,
    /// 左轴(ADC 量纲): raw/bsln/diff 与阈值线共用。
    left: AxisScale,
    /// 右轴(算法量纲): 算法上报线与触发判定共用。
    right: AxisScale,
    /// 全部系列, 顺序 = 图内声明顺序 = 调色板分配顺序。
    series: Vec<SeriesPath>,
    /// 左轴系列的最大采样点数: UI 的"有无遥测数据"判据。
    point_count: i32,
}

impl ChartFrame {
    fn _find(&self, id: SeriesId) -> Option<&SeriesPath> {
        self.series.iter().find(|s| s.id == id)
    }
    fn path_of(&self, id: SeriesId) -> &str {
        self._find(id).map_or("", |s| s.path.as_str())
    }
    fn color_of(&self, id: SeriesId) -> slint::Color {
        self._find(id)
            .map_or_else(series_fallback_color, |s| s.color)
    }
    fn count_of(&self, id: SeriesId) -> i32 {
        self._find(id).map_or(0, |s| s.point_count)
    }
    fn binary_of(&self, id: SeriesId) -> bool {
        self._find(id).map_or(false, |s| s.binary)
    }
    /// 左轴值 → viewbox y, 供阈值线定位。
    /// ★无遥测数据时回中线★: 此时左轴是退化的单位量程(0..1), 按它映射会把阈值线贴到顶边,
    /// 看上去像"阈值恰好等于量程上限"。UI 侧此时本就隐藏阈值线, 回中线只是不留下误导性坐标。
    fn left_value_to_y(&self, value: f32) -> f32 {
        if self.point_count == 0 {
            return 500.0;
        }
        self.left.y_of(value)
    }
}

/// 把一条系列铺满整个 `XWindow` 定义域: 无采样覆盖处沿用相邻采样的值并标成虚线。
///
/// ★唯一实现点★ "哪段没数据"只在这里判定, `points_to_svg_path` 只管画、不再判缺口;
/// `XWindow::gap_us` 仍是唯一的缺口判据来源(作为"覆盖半径", 由调用方按系列节奏取好传进来)。
///
/// 三种无覆盖区间的处理(全部产出 `dashed = true` 的点, 取值 = 紧邻的那个真实采样值):
///  · 左端: 首样本晚于 `t_first_us` 超出阈值 ⇒ 从 `t_first_us` 起按首样本值画虚线到首样本;
///  · 中间: 相邻样本间隔超阈值 ⇒ 在前一点处按前值平推虚线到后一点, 再垂直阶跃到后值;
///  · 右端: 末样本早于 `t_last_us` 超出阈值 ⇒ 从末样本按末值平推虚线到 `t_last_us`。
/// 差距在阈值内时不画虚线, 而是把边缘值以实线推到窗口边界: 那段时间**确实**被该采样覆盖
/// (不足一个采样周期), 画虚线会让最新的一条线每帧都闪出一小截。
///
/// ★为什么必须铺满★ 三条主曲线来自遥测环形缓冲、四条上报线与触发判定来自算法追踪缓冲, 两者的
/// 起止时刻天然不同(后者只在本页驻留时轮询)。不铺满就会出现"一条线只画左半边、另一条只画右半边"
/// (缩放或切子标签后尤其明显)。铺满后所有线的 x 覆盖恒等于窗口, 差异只体现为实线/虚线。
///
/// 空输入返回空(该系列一个采样都没有 ⇒ 不参与绘制, 不能凭空造一条线)。
fn fill_uncovered_with_dashes(points: &[(u64, f32)], x: &XWindow, gap_us: u64) -> Vec<PlotPoint> {
    let (Some(&(first_t, first_v)), Some(&(last_t, last_v))) = (points.first(), points.last())
    else {
        return Vec::new();
    };
    // 每个缺口最多插 2 点, 首尾各最多 2 点。
    let mut out: Vec<PlotPoint> = Vec::with_capacity(points.len() + 4);
    // 无覆盖段的合成点: 值沿用给定的边缘值(不是 0), 标成虚线。
    let hold_at = |t_us: u64, val: f32| PlotPoint {
        t_us,
        val,
        dashed: true,
    };

    if first_t > x.t_first_us {
        if first_t - x.t_first_us > gap_us {
            // 两个同值虚线点 ⇒ 画笔从左边界一路虚线推到首样本(见 points_to_svg_path 的判据)。
            out.push(hold_at(x.t_first_us, first_v));
            out.push(hold_at(first_t, first_v));
        } else {
            out.push(PlotPoint {
                t_us: x.t_first_us,
                val: first_v,
                dashed: false,
            });
        }
    }

    let mut prev: Option<(u64, f32)> = None;
    for &(t, v) in points {
        if let Some((pt, pv)) = prev {
            if t.saturating_sub(pt) > gap_us {
                out.push(hold_at(pt, pv));
                out.push(hold_at(t, pv));
            }
        }
        out.push(PlotPoint {
            t_us: t,
            val: v,
            dashed: false,
        });
        prev = Some((t, v));
    }

    if x.t_last_us > last_t {
        if x.t_last_us - last_t > gap_us {
            out.push(hold_at(last_t, last_v));
            out.push(hold_at(x.t_last_us, last_v));
        } else {
            out.push(PlotPoint {
                t_us: x.t_last_us,
                val: last_v,
                dashed: false,
            });
        }
    }
    out
}

/// 在同一条 path 里把一段水平的"无覆盖段"画成虚线, 并把画笔停在段末(保证后续垂直阶跃起点正确)。
fn push_dashes(path: &mut String, x_from: i32, x_to: i32, y: i32) {
    use std::fmt::Write;
    let mut cursor = x_from;
    while cursor < x_to {
        let on_end = (cursor + DASH_ON).min(x_to);
        // ★write! 而非 push_str(&format!())★: 后者每次都要先堆分配一个临时 String。
        // 一条 1024 点的曲线 × 多条系列 = 数千次分配, 实测正是主图 12ms 的主要来源。
        let _ = write!(path, " M {} {} L {} {}", cursor, y, on_end, y);
        cursor = on_end + DASH_OFF;
    }
    let _ = write!(path, " M {} {}", x_to, y);
}

/// 把补齐后的绘图点投影成 SVG path。
///
/// ★x 只走 XWindow★: 横坐标按时间而非序号 —— 掉帧/暂停时等间距序号会把时间轴画错(同样的像素
/// 距离代表不同时长); 且整张图共用同一个 XWindow, 任何系列都不得自带 x 计算。
/// ★不再有"断线"★: 缺口已由 `fill_uncovered_with_dashes` 补成显式的虚线段, 于是这里只需逐点连线;
/// 无数据不再表现为"没有线"(那与"线被裁到框外"无法区分), 而是一段明确的虚线。
/// ★无覆盖段画虚线★: 两端都是合成点且同高的水平段 ⇒ 拆成 on/off 小段(同色, 见 `DASH_ON`),
/// 与真实采样的实线区分开; 进出虚线段的垂直阶跃仍画实线, 阶跃本身必须清晰可见。
fn points_to_svg_path(points: &[PlotPoint], x: &XWindow, axis: &AxisScale) -> String {
    use std::fmt::Write;
    if points.is_empty() {
        return String::new();
    }
    // 预留容量: 每点约 12 字节 ("&nbsp;L 1000 1000")。一次分配好过边写边搬。
    let mut path = String::with_capacity(points.len() * 12 + 16);
    let mut prev: Option<(i32, i32, bool)> = None;
    for p in points {
        let px = x.x_of(p.t_us) as i32;
        let py = axis.y_of(p.val) as i32;
        match prev {
            None => {
                let _ = write!(path, "M {} {}", px, py);
            }
            Some((ppx, ppy, pz)) => {
                if pz && p.dashed && py == ppy {
                    push_dashes(&mut path, ppx, px, py);
                } else {
                    // ★同像素点直接丢弃★ viewbox 只有 1000×1000 个整数坐标, 而缓冲有 1024 点/通道:
                    // 落到同一像素的点画出来完全看不见, 却要各占一段 path 命令并让 Slint 多解析一次。
                    // 这不是抽稀采样(不丢任何可见形状), 只是不再重复画同一个点。
                    if px == ppx && py == ppy {
                        continue;
                    }
                    let _ = write!(path, " L {} {}", px, py);
                }
            }
        }
        prev = Some((px, py, p.dashed));
    }
    path
}

/// 判定二值序列: 非空且全部取值只有 0/1。0/1 在共享右轴上几乎不可见, 需要拉伸到 0..N。
/// ★只看真实采样★: 在归零补齐**之前**判定, 否则补出来的 0 会把非二值线也算成二值。
fn points_are_binary(points: &[(u64, f32)]) -> bool {
    !points.is_empty() && points.iter().all(|&(_, v)| v == 0.0 || v == 1.0)
}

/// 二值线按幅度 N 拉伸(纯显示变换, 不下发设备); 非二值线原样返回。
fn normalize_binary(points: &[(u64, f32)], amp: f32) -> Vec<(u64, f32)> {
    points.iter().map(|&(t, v)| (t, v * amp)).collect()
}

/// 求一组序列的取值范围(忽略非有限值)。空 ⇒ 返回 (+inf, -inf),
/// 由 `AxisScale::from_span` 退化成单位量程。
///
/// ★入参是"铺满后"的点, 但量程不会因缺口而变★: 虚线段取的是相邻真实采样的值, 不引入新极值,
/// 于是同一段波形无论有没有缺口, 纵向尺度都一样(这正是"归一化绘制"要保证的另一半)。
/// 反例(旧实现)是用 0 补缺口: RAW 常在 2500..4095, 一旦掺进 0 就把整条线压成贴顶的一条细带,
/// 几十个计数的变化就此看不见 —— 而且只在"恰好有缺口"时发生, 表现为纵轴莫名跳变。
fn value_span(series: &[&[PlotPoint]]) -> (f32, f32) {
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for points in series {
        for p in *points {
            if p.val.is_finite() {
                lo = lo.min(p.val);
                hi = hi.max(p.val);
            }
        }
    }
    (lo, hi)
}

/// 一张图里"哪些系列参与绘制"。
/// ★参与 = 既画线、也算进唯一 x 定义域与它那根轴的量程★: 没画出来的线不该影响别人的位置,
/// 否则勾掉一条线其余线会莫名其妙地跳。
#[derive(Debug, Clone, Copy)]
struct SeriesShow {
    raw: bool,
    bsln: bool,
    diff: bool,
    /// 4 条算法上报线各自的勾选(调用方已并入"叠加总开关 + 算法已声明该 idx"的判定)。
    report: [bool; 4],
    active: bool,
}

/// 主图一次回填: ★唯一入口★, 一次产出全部系列。
/// `amps[0..3]` 对应 report[idx] 的二值归一化幅度, `amps[4]` 对应触发判定。
/// `poll_slots` = 当前算法声明的 report idx 个数(轮询轮转长度), 只用于算轮询节奏的缺口阈值。
fn build_chart_frame(
    ctrl: &AppController,
    ch: u8,
    show: SeriesShow,
    amps: &[f32; 5],
    poll_slots: usize,
) -> ChartFrame {
    // ---- 1. 取点 ----
    // 左轴三条按勾选取(不画就不取); 右轴五条一律取: 行模型要靠"有无数据 / 是否二值"决定整行
    // 是否收起、是否显示归一化输入, 那与"画不画"是两件事(勾掉一条线不该让它的输入框消失)。
    let mut left_pts: [Vec<(u64, f32)>; 3] = [
        if show.raw {
            ctrl.telem_points(ch, FIELD_RAW)
        } else {
            Vec::new()
        },
        if show.bsln {
            ctrl.telem_points(ch, FIELD_BASELINE)
        } else {
            Vec::new()
        },
        if show.diff {
            ctrl.telem_points(ch, FIELD_DIFF)
        } else {
            Vec::new()
        },
    ];
    let mut report_pts: [Vec<(u64, f32)>; 4] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    let mut report_binary = [false; 4];
    for idx in 0..4usize {
        let raw = ctrl.algo_trace_report_points(idx as u8);
        // 二值判定必须在归一化【之前】: 乘上 N 之后取值不再是 0/1, 归一化输入框会自己消失。
        // ★判定与"是否参与绘制"无关★: 归一化输入框对未勾选的行也要照常出现, 故这一步不能跳。
        report_binary[idx] = points_are_binary(&raw);
        // 未参与绘制的系列到此为止: 归一化/归零补齐/path 生成都是纯浪费(路径不会被画),
        // 而它们正是本页每帧最重的一段 —— 8 条线里通常只有 1~2 条真的开着。
        if !show.report[idx] {
            continue;
        }
        report_pts[idx] = if report_binary[idx] {
            normalize_binary(&raw, amps[idx])
        } else {
            raw
        };
    }
    let active_raw = ctrl.algo_trace_active_points();
    let active_binary = points_are_binary(&active_raw);
    let mut active_pts = if show.active {
        normalize_binary(&active_raw, amps[4])
    } else {
        Vec::new()
    };

    // ---- 2. ★唯一的 x 定义域★: 整张图只在这里算一次 ----
    // 左端由**主曲线(raw/bsln/diff)**定义, 叠加系列只能把右端往新的方向延伸, 不能把左端往旧的方向拉。
    //
    // ★为什么不是简单求并集★(这是实测踩到的坑)
    // 算法上报/触发判定只在"单通道精调"页驻留时才轮询, 而遥测在整个设置页都在推。去"算法"页待一会儿
    // 再回来: 上报缓冲停在离开那一刻、且其最老样本可能比遥测环的最老样本更早(遥测环持续丢弃旧样本),
    // 于是并集的左端被钉在一个陈旧时间戳上, 主曲线被挤到画面右侧 —— 用户看到的就是"只有右半边能看"。
    // 图的时间轴本质上就是遥测缓冲的时间轴("全量 x.xx s"说的是它), 比它更老的叠加样本已在窗口之外,
    // 不该有资格撑开坐标轴。
    // 反方向的老 bug(叠加线只画了一半宽)也仍然被覆盖: 叠加系列**不会**把窗口收窄, 只是不再撑左端。
    let mut t_first = u64::MAX;
    let mut t_last = 0u64;
    {
        for points in &left_pts {
            if let (Some(first), Some(last)) = (points.first(), points.last()) {
                t_first = t_first.min(first.0);
                t_last = t_last.max(last.0);
            }
        }
        let primary_has_data = t_first <= t_last;
        let mut extend_right = |points: &[(u64, f32)]| {
            if let (Some(first), Some(last)) = (points.first(), points.last()) {
                // 主曲线没有数据时(例如停流后只剩上报缓冲)才允许叠加系列定义左端, 否则整张图无从落笔。
                if !primary_has_data {
                    t_first = t_first.min(first.0);
                }
                t_last = t_last.max(last.0);
            }
        };
        for idx in 0..4usize {
            if show.report[idx] {
                extend_right(&report_pts[idx][..]);
            }
        }
        if show.active {
            extend_right(&active_pts[..]);
        }
    }
    // 一条参与系列都没有数据 ⇒ 空窗归零(不能留 u64::MAX, 那会让 x 映射全部落到 0)。
    let no_data = t_first > t_last;
    // ★横轴总跨度固定 30s★ 右端 = 最新样本时刻, 左端 = 右端 − 30s。
    // 为什么不再用"缓冲里最老那个样本"当左端: 那个值随缓冲填充、采样率变化、通道切换而漂,
    // 于是同一段波形在图上的横向尺度每帧都在变(抖动), 缩放/滚动条的比例也跟着变 ——
    // 固定窗口后横向 1 像素恒等于固定时长, 波形形状才可比。
    // 数据不足 30s 时左边就是空的(不补 0, 见 fill_uncovered_with_zero 的左端注释)。
    let t_first = t_last.saturating_sub(PLOT_WINDOW_US);
    let x = XWindow {
        t_first_us: if no_data { 0 } else { t_first },
        t_last_us: if no_data { 0 } else { t_last },
        // 遥测帧缺口: 超过 5 个标称周期(且至少 250ms)才算缺口, 容忍正常的帧间抖动。
        gap_frame_us: gap_threshold_us(ctrl.telem_samples_per_sec(), 5, 250_000),
        // 轮询缺口: n 个 idx 轮转 → 单条线的间隔是遥测周期的 n 倍, 阈值同比放大。
        gap_poll_us: gap_threshold_us(
            ctrl.telem_samples_per_sec(),
            (poll_slots.max(1) * 6) as u32,
            400_000,
        ),
    };

    // ---- 3. ★归零补齐★: 每条有采样的系列在 x 的整个定义域上处处有定义(见 ChartFrame 不变量)
    // 阈值按系列节奏取(两档都由 XWindow 一处算定), 补齐结果同时是后面量程与路径的唯一输入 ——
    // 量程用原始点、路径用补齐点会让归零段落在量程外, 那正是"归零段被裁到边框外"的来路。
    // 节奏仍由系列身份决定(SeriesId::cadence 是那条映射的唯一实现), 这里只是把两档各取一次。
    // ★窗口外的样本必须先丢掉(所有系列, 含主曲线)★
    // 窗口左端由固定跨度定义, 比它更老的样本仍留在各自缓冲里。若不丢:
    //  · `XWindow::x_of` 会把它们统统夹到 x=0, 在左边框上糊成一条竖线;
    //  · 它们还会参与量程, 把量程撑成陈旧数据的范围。
    // 丢弃只影响"画不进这张图的历史点", 不改变任何仍在窗口内的数据。
    {
        let cut = x.t_first_us;
        for pts in left_pts.iter_mut() {
            pts.retain(|(t, _)| *t >= cut);
        }
        for pts in report_pts.iter_mut() {
            pts.retain(|(t, _)| *t >= cut);
        }
        active_pts.retain(|(t, _)| *t >= cut);
    }

    let gap_frame = x.gap_us(SeriesId::Raw.cadence());
    let gap_poll = x.gap_us(SeriesId::Active.cadence());
    let left_fill: [Vec<PlotPoint>; 3] = [
        fill_uncovered_with_dashes(&left_pts[0][..], &x, gap_frame),
        fill_uncovered_with_dashes(&left_pts[1][..], &x, gap_frame),
        fill_uncovered_with_dashes(&left_pts[2][..], &x, gap_frame),
    ];
    let report_fill: [Vec<PlotPoint>; 4] = [
        fill_uncovered_with_dashes(&report_pts[0][..], &x, gap_poll),
        fill_uncovered_with_dashes(&report_pts[1][..], &x, gap_poll),
        fill_uncovered_with_dashes(&report_pts[2][..], &x, gap_poll),
        fill_uncovered_with_dashes(&report_pts[3][..], &x, gap_poll),
    ];
    let active_fill = fill_uncovered_with_dashes(&active_pts[..], &x, gap_poll);

    // ---- 4. 两根轴的量程: 各自只由"参与绘制且走该轴"的系列决定 ----
    let left = {
        let (lo, hi) = value_span(&[&left_fill[0][..], &left_fill[1][..], &left_fill[2][..]]);
        AxisScale::from_span(lo, hi, 1.0)
    };
    let right = {
        let mut participants: Vec<&[PlotPoint]> = Vec::with_capacity(5);
        for idx in 0..4usize {
            if show.report[idx] {
                participants.push(&report_fill[idx][..]);
            }
        }
        if show.active {
            participants.push(&active_fill[..]);
        }
        let (lo, hi) = value_span(&participants);
        AxisScale::from_span(lo, hi, 0.5)
    };

    // ---- 5. 生成路径: 同一个 x 映射 + 各自所属的轴。point_count 仍是**真实采样数**
    // (补齐点不是数据, 计进去会让"有无遥测数据"的判据永远为真)。
    let entries: [(SeriesId, &[PlotPoint], bool, usize); 8] = [
        (SeriesId::Raw, &left_fill[0][..], false, left_pts[0].len()),
        (SeriesId::Bsln, &left_fill[1][..], false, left_pts[1].len()),
        (SeriesId::Diff, &left_fill[2][..], false, left_pts[2].len()),
        (
            SeriesId::Report(0),
            &report_fill[0][..],
            report_binary[0],
            report_pts[0].len(),
        ),
        (
            SeriesId::Report(1),
            &report_fill[1][..],
            report_binary[1],
            report_pts[1].len(),
        ),
        (
            SeriesId::Report(2),
            &report_fill[2][..],
            report_binary[2],
            report_pts[2].len(),
        ),
        (
            SeriesId::Report(3),
            &report_fill[3][..],
            report_binary[3],
            report_pts[3].len(),
        ),
        (
            SeriesId::Active,
            &active_fill[..],
            active_binary,
            active_pts.len(),
        ),
    ];
    let mut palette = PaletteCursor::new();
    let series: Vec<SeriesPath> = entries
        .iter()
        .map(|&(id, points, binary, raw_count)| SeriesPath {
            id,
            path: points_to_svg_path(points, &x, if id.on_right_axis() { &right } else { &left }),
            color: palette.take(),
            point_count: raw_count as i32,
            binary,
        })
        .collect();
    if palette.overflow() > 0 {
        log::warn!(
            "主图系列数超出调色板容量({} 色): {} 条线只能用降级灰, 请扩充 SERIES_PALETTE — \
             绝不靠重复颜色顶过去(两条同色线等于看不出谁是谁)。",
            SERIES_PALETTE.len(),
            palette.overflow()
        );
    }

    ChartFrame {
        x,
        left,
        right,
        series,
        // "有无遥测数据"只看左轴三条: 算法追踪即使有点, 也不代表遥测在流。
        point_count: left_pts.iter().map(|p| p.len()).max().unwrap_or(0) as i32,
    }
}

/// 设备端运行时刻(展开后)的人读文本: 供"最新样本的绝对时刻"这一行展示。
/// UI 侧的刻度/读数一律用"相对最新样本"的 ms 偏移(f32 精度足够), 绝对时刻只在这里出现一次。
fn dev_time_text(t_us: u64) -> String {
    let total_ms = t_us / 1000;
    let ms = total_ms % 1000;
    let total_s = total_ms / 1000;
    format!(
        "{:02}:{:02}:{:02}.{:03}",
        total_s / 3600,
        (total_s / 60) % 60,
        total_s % 60,
        ms
    )
}

fn series_to_svg_path(series: &[f32], min: f32, max: f32, x_scale: f32) -> String {
    if series.is_empty() {
        return String::new();
    }

    let n = series.len();
    let range = (max - min).max(f32::EPSILON);
    let mut path = String::new();

    for (i, &v) in series.iter().enumerate() {
        // x_scale = 绘图区宽高比: 把曲线横向拉伸到 [0, 1000*aspect], 配合 Slint 侧
        // viewbox 宽高比锁定, 使 contain 缩放正好铺满(1.17 Path 无 image-fit)。
        let x = (i as f32) * 1000.0 / ((n - 1).max(1) as f32) * x_scale;
        let y = (1000.0 - (v - min) / range * 1000.0).clamp(0.0, 1000.0);

        if i == 0 {
            path.push_str(&format!("M {} {}", x as i32, y as i32));
        } else {
            path.push_str(&format!(" L {} {}", x as i32, y as i32));
        }
    }

    path
}

/// 逻辑分析仪可选时间窗(微秒)。最小 500us 用于看清机械抖动的微秒级间隔,
/// 最大 5s 用于看整段按压序列。索引与 UI 下拉一一对应。
const LA_WINDOWS_US: [u32; 7] = [500, 2_000, 10_000, 50_000, 200_000, 1_000_000, 5_000_000];
/// 默认 50ms: 一次按下的抖动全景 + 3ms 级防抖窗仍清晰可辨。
const LA_WINDOW_DEFAULT: usize = 3;

/// 把微秒数格式化成可读时间(自动切 us/ms/s), 供逻辑分析仪的横轴刻度与触发读数使用。
fn fmt_time_us(us: f64) -> String {
    let a = us.abs();
    if a < 1000.0 {
        format!("{:.1}us", us)
    } else if a < 1_000_000.0 {
        format!("{:.3}ms", us / 1000.0)
    } else {
        format!("{:.3}s", us / 1_000_000.0)
    }
}

/// 逻辑分析仪一屏视图: 12 通道阶梯波形(去抖前/仅去抖后/实际输出各一条)+ 每通道触发标记 + 横轴刻度。
struct LogicAnalyzerView {
    raw_paths: Vec<slint::SharedString>,
    deb_paths: Vec<slint::SharedString>,
    out_paths: Vec<slint::SharedString>,
    trig_text: Vec<slint::SharedString>,
    trig_x: Vec<f32>,
    time_labels: Vec<slint::SharedString>,
    span_text: slint::SharedString,
}

/// 从边沿记录构建 12 通道时序图。
///
/// - 窗口右缘 = 缓冲里最新一条记录的时间戳, 向左展开 `window_us`; 全部通道共用这一条时间轴,
///   所以泳道之间的触发标记 x 可以直接横向比较(这正是"逻辑分析仪"的意义)。
/// - 设备时间戳是 `time_us_32()`, 32 位会回绕: 一律用 `wrapping_sub` 求相对偏移,
///   窗口外的旧记录其无符号差会变成巨大值, 天然被 `> window_us` 过滤掉。
/// - 窗口起点的电平取"窗口之前最后一条记录"的状态, 否则每次滚动窗口都会凭空多出一个假边沿。
fn build_logic_analyzer(
    edges: &std::collections::VecDeque<mai2control_ui::proto::KbdEdgeRec>,
    window_us: u32,
) -> LogicAnalyzerView {
    const KEYS: usize = 12;
    const Y_ON: i32 = 16; // 按下
    const Y_OFF: i32 = 84; // 松开
    let mut view = LogicAnalyzerView {
        raw_paths: vec![slint::SharedString::new(); KEYS],
        deb_paths: vec![slint::SharedString::new(); KEYS],
        out_paths: vec![slint::SharedString::new(); KEYS],
        trig_text: vec![slint::SharedString::new(); KEYS],
        trig_x: vec![-1.0; KEYS],
        time_labels: Vec::new(),
        span_text: slint::SharedString::new(),
    };
    // 横轴刻度先建好: 即使无数据也要有刻度, 免得空图看起来像"坏了"。
    for i in 0..5 {
        let back_us = window_us as f64 * (4 - i) as f64 / 4.0;
        view.time_labels.push(
            if i == 4 {
                "0".to_string()
            } else {
                format!("-{}", fmt_time_us(back_us))
            }
            .into(),
        );
    }
    let window_us = window_us.max(1);
    let Some(last) = edges.back() else {
        view.span_text = format!("窗口 {} · 无边沿记录", fmt_time_us(window_us as f64)).into();
        for k in 0..KEYS {
            view.trig_text[k] = "无数据".into();
        }
        return view;
    };
    let t_start = last.t_us.wrapping_sub(window_us);
    // 窗口内记录 + 窗口起点前的最后一个状态(作为初始电平)。
    let mut before: Option<(u16, u16, u16)> = None;
    let mut visible: Vec<&mai2control_ui::proto::KbdEdgeRec> = Vec::new();
    for rec in edges.iter() {
        if rec.t_us.wrapping_sub(t_start) <= window_us {
            visible.push(rec);
        } else {
            before = Some((rec.raw, rec.deb, rec.out));
        }
    }
    let x_of = |t: u32| -> f32 { t.wrapping_sub(t_start) as f32 / window_us as f32 * 1000.0 };
    for k in 0..KEYS {
        let bit = 1u16 << k;
        let (mut raw_on, mut deb_on, mut out_on) = match before {
            Some((r, d, o)) => ((r & bit) != 0, (d & bit) != 0, (o & bit) != 0),
            // 窗口前没有任何记录时, 用窗口内第一条的状态当起点(它就是该键进入窗口时的电平)。
            None => visible
                .first()
                .map(|r| ((r.raw & bit) != 0, (r.deb & bit) != 0, (r.out & bit) != 0))
                .unwrap_or((false, false, false)),
        };
        let mut raw_p = format!("M 0 {}", if raw_on { Y_ON } else { Y_OFF });
        let mut deb_p = format!("M 0 {}", if deb_on { Y_ON } else { Y_OFF });
        let mut out_p = format!("M 0 {}", if out_on { Y_ON } else { Y_OFF });
        let mut edge_count = 0u32;
        let mut trig_at: Option<u32> = None;
        for rec in &visible {
            let x = x_of(rec.t_us);
            let r = (rec.raw & bit) != 0;
            let d = (rec.deb & bit) != 0;
            let o = (rec.out & bit) != 0;
            if r != raw_on {
                // 阶梯: 先水平走到该时刻, 再垂直跳变 —— 电平信号不能画成斜线。
                raw_p.push_str(&format!(
                    " L {:.1} {} L {:.1} {}",
                    x,
                    if raw_on { Y_ON } else { Y_OFF },
                    x,
                    if r { Y_ON } else { Y_OFF }
                ));
                raw_on = r;
                edge_count += 1;
            }
            if d != deb_on {
                deb_p.push_str(&format!(
                    " L {:.1} {} L {:.1} {}",
                    x,
                    if deb_on { Y_ON } else { Y_OFF },
                    x,
                    if d { Y_ON } else { Y_OFF }
                ));
                deb_on = d;
                if d && trig_at.is_none() {
                    trig_at = Some(rec.t_us);
                }
            }
            if o != out_on {
                out_p.push_str(&format!(
                    " L {:.1} {} L {:.1} {}",
                    x,
                    if out_on { Y_ON } else { Y_OFF },
                    x,
                    if o { Y_ON } else { Y_OFF }
                ));
                out_on = o;
            }
        }
        raw_p.push_str(&format!(" L 1000 {}", if raw_on { Y_ON } else { Y_OFF }));
        deb_p.push_str(&format!(" L 1000 {}", if deb_on { Y_ON } else { Y_OFF }));
        out_p.push_str(&format!(" L 1000 {}", if out_on { Y_ON } else { Y_OFF }));
        view.raw_paths[k] = raw_p.into();
        view.deb_paths[k] = deb_p.into();
        view.out_paths[k] = out_p.into();
        view.trig_text[k] = match trig_at {
            Some(t) => {
                view.trig_x[k] = x_of(t);
                format!(
                    "+{} ·{}沿",
                    fmt_time_us(t.wrapping_sub(t_start) as f64),
                    edge_count
                )
                .into()
            }
            None if edge_count > 0 => format!("无上升沿 ·{}沿", edge_count).into(),
            None => "静默".into(),
        };
    }
    view.span_text = format!(
        "窗口 {} · 窗口内 {} 条 / 缓冲 {} 条 · 右缘 t={}us",
        fmt_time_us(window_us as f64),
        visible.len(),
        edges.len(),
        last.t_us
    )
    .into();
    view
}

/// 生成延迟历史折线 path + 自适应纵向量程 (lo, hi)。x 固定映射到 [0,1000]，由 PlotPath 的 fit: fill 拉满绘图区。
/// 不能用 aspect 修正：等比 contain 无法铺满任意矩形，宽高比变化后会重新产生空白带。
/// 纵向量程按数据 min/max 自适应(带 10% 余量), 否则接近常数的延迟会被压成一条线。
fn build_lat_path(series: &[f32]) -> (String, f32, f32) {
    let dmin = series.iter().cloned().fold(f32::INFINITY, f32::min);
    let dmax = series.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let (lo, hi) = if dmin.is_finite() && dmax.is_finite() {
        let pad = ((dmax - dmin) * 0.1).max(1.0);
        ((dmin - pad).max(0.0), dmax + pad)
    } else {
        (0.0, 1.0)
    };
    (series_to_svg_path(series, lo, hi, 1.0), lo, hi)
}

/// 右锚定滚动窗口折线: 最新点固定在右缘(x=1000),越旧越靠左,超出 window 的点丢弃。
/// 未填满窗口时曲线从右向左生长, 填满后随新数据整体左移(自动滚动)。
/// (显示名, HID 键码) 表, 供物理键盘/触控键盘映射下拉。索引 0 = 不映射。
/// 键码为标准 HID Keyboard/Keypad usage, 与固件 HID_KeyCode 一致。
fn kbd_key_choices() -> Vec<(&'static str, u8)> {
    let mut v: Vec<(&'static str, u8)> = vec![("不映射", 0x00)];
    const LETTERS: [&str; 26] = [
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R",
        "S", "T", "U", "V", "W", "X", "Y", "Z",
    ];
    for (i, name) in LETTERS.iter().enumerate() {
        v.push((name, 0x04 + i as u8));
    }
    const DIGITS: [&str; 10] = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"];
    for (i, name) in DIGITS.iter().enumerate() {
        v.push((name, 0x1E + i as u8));
    }
    v.push(("Enter", 0x28));
    v.push(("Esc", 0x29));
    v.push(("Backspace", 0x2A));
    v.push(("Tab", 0x2B));
    v.push(("Space", 0x2C));
    const FKEYS: [&str; 12] = [
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
    ];
    for (i, name) in FKEYS.iter().enumerate() {
        v.push((name, 0x3A + i as u8));
    }
    v.push(("→ 右", 0x4F));
    v.push(("← 左", 0x50));
    v.push(("↓ 下", 0x51));
    v.push(("↑ 上", 0x52));
    v.push(("LCtrl", 0xE0));
    v.push(("LShift", 0xE1));
    v.push(("LAlt", 0xE2));
    v.push(("LGui", 0xE3));
    v.push(("RCtrl", 0xE4));
    v.push(("RShift", 0xE5));
    v.push(("RAlt", 0xE6));
    v.push(("RGui", 0xE7));
    v
}

/// HID 键码 → 下拉索引(未找到=0/不映射)。
fn kbd_code_to_choice(code: u8) -> i32 {
    kbd_key_choices()
        .iter()
        .position(|(_, c)| *c == code)
        .map(|p| p as i32)
        .unwrap_or(0)
}

/// 下拉索引 → HID 键码(越界=0)。
fn kbd_choice_to_code(ci: i32) -> u8 {
    if ci < 0 {
        return 0;
    }
    kbd_key_choices()
        .get(ci as usize)
        .map(|(_, c)| *c)
        .unwrap_or(0)
}

/// HID 键码 → 显示名(用于捕获输入框显示)。
fn kbd_hid_name(code: u8) -> &'static str {
    if code == 0 {
        return "";
    }
    kbd_key_choices()
        .iter()
        .find(|(_, c)| *c == code)
        .map(|(n, _)| *n)
        .unwrap_or("?")
}

/// (键码, 修饰位) → 组合键显示串, 如 "Ctrl+Shift+A"。空映射显示"未设置"。
fn kbd_display(code: u8, modifier: u8) -> String {
    if code == 0 && modifier == 0 {
        return "未设置(点击后按键)".to_string();
    }
    let mut s = String::new();
    if modifier & 1 != 0 {
        s.push_str("Ctrl+");
    }
    if modifier & 2 != 0 {
        s.push_str("Shift+");
    }
    if modifier & 4 != 0 {
        s.push_str("Alt+");
    }
    if modifier & 8 != 0 {
        s.push_str("Gui+");
    }
    if code != 0 {
        s.push_str(kbd_hid_name(code));
    } else {
        s.pop(); // 去掉尾部 '+'
    }
    s
}

/// Slint KeyEvent.text → HID 键码。纯修饰键/未识别返回 0。
/// Slint 特殊键为私有区/控制字符常量(见 i_slint_core key_codes)。
fn char_to_hid(text: &str) -> u8 {
    match text {
        "\u{000a}" | "\r" => return 0x28, // Enter
        "\u{001b}" => return 0x29,        // Escape
        "\u{0008}" => return 0x2A,        // Backspace
        "\u{0009}" => return 0x2B,        // Tab
        " " => return 0x2C,               // Space
        "\u{f700}" => return 0x52,        // Up
        "\u{f701}" => return 0x51,        // Down
        "\u{f702}" => return 0x50,        // Left
        "\u{f703}" => return 0x4F,        // Right
        _ => {}
    }
    if let Some(ch) = text.chars().next() {
        let u = ch as u32;
        if (0xf704..=0xf70f).contains(&u) {
            return 0x3A + (u - 0xf704) as u8; // F1..F12
        }
        let lc = ch.to_ascii_lowercase();
        match lc {
            'a'..='z' => return 0x04 + (lc as u8 - b'a'),
            '1'..='9' => return 0x1E + (lc as u8 - b'1'),
            '0' => return 0x27,
            _ => {}
        }
    }
    0
}

/// 全通道网格的行模型。★排序与筛选都在这里定稿★: 次序取自
/// `AppController::channel_display_order`(逻辑序的唯一来源是绑定映射), Slint 只按数组下标摆格子。
/// 返回 `(行, 被筛掉的通道数)`。
fn build_channel_status(
    ctrl: &AppController,
    sort: i32,
    show_disabled: bool,
) -> (Vec<ChannelStatus>, i32) {
    let (order, hidden) = ctrl.channel_display_order(sort, show_disabled);
    let mut out = Vec::with_capacity(order.len());
    for (pos, ch) in order.into_iter().enumerate() {
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
            // 网格坐标按【展示次序】给出(不是物理号): 排序后按物理号摆格会与实际渲染次序错位。
            grid_col: (pos % 6) as i32,
            grid_row: (pos / 6) as i32,
            binding_text: binding_text.into(),
            cp_text: cp_display_text(ctrl.cp(ch)).into(),
            frozen: ctrl.channel_frozen(ch),
            enabled: ctrl.ch_enabled(ch),
            order: pos as i32,
        });
    }
    (out, hidden as i32)
}

fn build_param_rows(params: &[(u8, u32)]) -> Vec<ParamRow> {
    params
        .iter()
        // 单通道精调展示全部由 CSD 模式接管的逐通道参数；AUTO 时 Slint 统一显示 AUTO 并锁定，
        // SEMI 时恢复数值与编辑，避免自动实时值被误当作用户的手动设置。
        .filter(|&&(param_id, _)| matches!(param_id, 0x01..=0x06 | 0x08..=0x0B))
        .map(|&(param_id, value)| {
            // 围栏随行下发: Slint 只消费数值, 不再自己按 param_id 派生阈值(见 types.slint::ParamRow)。
            build_param_row(param_id, value as i32)
        })
        .collect()
}

/// 响应噪声频谱的热图格子模型(SWEEP_GAINS × SWEEP_DIVS 格)。
///
/// ★归一化与配色必须在 Rust 侧算★ Slint 数不出"全体有效格的最小/最大值", 而热图的全部意义在于
/// 相对高低; 若把原始标准差丢给 .slint 让它自己上色, 那边只能写死一个绝对量程 —— 换个通道或换个
/// 分辨率就整片糊成一个颜色。
/// ★撞顶/贴底的格子不参与 min/max★: 那种点 RAW 贴在量程边界上, 标准差小得好看但 diff 已经失效,
/// 让它参与归一化会把整张图的基准拉歪。它们仍然画出来, 由 .slint 侧描红边区别标注。
fn build_spectrum_cells(ctrl: &AppController) -> Vec<SpectrumCell> {
    let cells = ctrl.noise_sweep_cells();
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    for cell in cells {
        if !cell.valid || cell.railed {
            continue;
        }
        lo = lo.min(cell.std);
        hi = hi.max(cell.std);
    }
    let span = (hi - lo).max(1e-6);
    cells
        .iter()
        .enumerate()
        .map(|(i, cell)| {
            let norm = if cell.valid && hi >= lo {
                ((cell.std - lo) / span).clamp(0.0, 1.0)
            } else {
                0.0
            };
            SpectrumCell {
                gain: (i / SWEEP_DIVS) as i32,
                div: (i % SWEEP_DIVS + 1) as i32,
                std: cell.std,
                pp: cell.pp,
                norm,
                valid: cell.valid,
                railed: cell.railed,
                tint: _spectrum_tint(cell.valid, cell.railed, norm),
            }
        })
        .collect()
}

/// 一格的颜色: 未测=底色, 撞顶=暗红(其"低噪声"不可信), 其余按噪声由低到高 蓝→绿→红。
/// 分两段线性插值而不是简单 R/B 对调: 中间层次(绿)才让人看出"哪一片是可用区"。
fn _spectrum_tint(valid: bool, railed: bool, norm: f32) -> slint::Color {
    if !valid {
        return slint::Color::from_rgb_u8(0x1a, 0x1f, 0x24);
    }
    if railed {
        return slint::Color::from_rgb_u8(0x4a, 0x2a, 0x2a);
    }
    let t = norm.clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.5 {
        let u = t / 0.5;
        (
            _lerp8(0x20, 0x30, u),
            _lerp8(0x60, 0xd0, u),
            _lerp8(0xd0, 0x50, u),
        )
    } else {
        let u = (t - 0.5) / 0.5;
        (
            _lerp8(0x30, 0xe0, u),
            _lerp8(0xd0, 0x40, u),
            _lerp8(0x50, 0x30, u),
        )
    };
    slint::Color::from_rgb_u8(r, g, b)
}

fn _lerp8(a: u8, b: u8, t: f32) -> u8 {
    (a as f32 + (b as f32 - a as f32) * t.clamp(0.0, 1.0)).round() as u8
}

/// 全局项(GPARAM_*)的数字输入围栏, 供触控全局调整页的 SpinBox 取上下界。
///
/// ★围栏唯一来源 = `proto::telemetry::global_fence`★(与 RP2040 `_handle_global_set`、
/// PSoC `cmd_set_global` 同源, 见该文件该节头注释)。slint 侧只消费数值, 不按 gparam_id
/// 自己派生阈值 —— 那会变成与两处固件必然漂移的第三份围栏。
/// GPARAM_INACTIVE_SNS(0x01) 取值离散({1,2,4}), 由 ComboBox 表达, 不需要数值上下界。
fn build_global_fences() -> GlobalFences {
    use mai2control_ui::proto::algo as ga;
    use mai2control_ui::proto::global_fence;
    let gain = global_fence(ga::GPARAM_IDAC_GAIN_INIT);
    let imin = global_fence(ga::GPARAM_IDAC_MIN);
    let tgt = global_fence(ga::GPARAM_RAW_TARGET);
    let f1 = global_fence(ga::GPARAM_MFS_DIV_F1);
    let f2 = global_fence(ga::GPARAM_MFS_DIV_F2);
    GlobalFences {
        idac_gain_min: gain.ui_min() as i32,
        idac_gain_max: gain.ui_max() as i32,
        idac_min_min: imin.ui_min() as i32,
        idac_min_max: imin.ui_max() as i32,
        raw_target_min: tgt.ui_min() as i32,
        raw_target_max: tgt.ui_max() as i32,
        mfs_f1_min: f1.ui_min() as i32,
        mfs_f1_max: f1.ui_max() as i32,
        mfs_f2_min: f2.ui_min() as i32,
        mfs_f2_max: f2.ui_max() as i32,
    }
}

/// per-channel 参数的中文显示名。单一真相源: 单通道精调的参数行与全通道页的批量应用面板
/// 共用本函数, 避免两处各写一份 match 而出现名称漂移。
fn param_display_name(param_id: u8) -> String {
    match param_id {
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
    }
}

/// 主页"工作模式"一行文案。草稿优先(与通信系统 Tab 的 ComboBox 同源), 未回读到 mode.work
/// 就显示"未知" —— 拿默认值 0 冒充设备真值会让人以为设备在 Serial 模式。
/// 草稿与设备值不同时追加提示: 该项要整机重启重枚举才生效, 复用 draft_needs_reboot 判定。
fn work_mode_text(ctrl: &AppController) -> String {
    let base = match ctrl.work_mode() {
        Some(0) => "Serial (mai2serial + mai2light 双 CDC)",
        Some(1) => "HID (键盘 + 触摸屏)",
        Some(_) => "未知(设备返回了未定义值)",
        None => "未知",
    };
    if ctrl.draft_needs_reboot() {
        format!("{}（未保存，保存后重启生效）", base)
    } else {
        base.to_string()
    }
}

/// 读取数值型配置 KV(草稿优先, 由 config_get 保证); 未回读到则给默认值。
/// 各数值变体统一折成 u32, 免得每个调用点再 match 一遍 CfgValue。
fn cfg_u32_or(ctrl: &AppController, key: &str, default: u32) -> u32 {
    match ctrl.config_get(key).map(|e| e.value) {
        Some(CfgValue::U8(v)) => v as u32,
        Some(CfgValue::I8(v)) => v.max(0) as u32,
        Some(CfgValue::U16(v)) => v as u32,
        Some(CfgValue::U32(v)) => v,
        Some(CfgValue::F32(v)) => v.max(0.0) as u32,
        Some(CfgValue::Bool(v)) => u32::from(v),
        _ => default,
    }
}

/// 虚拟 LED 单元语义标签: 0..7 为按键灯(经灯板协议缓冲提交), 8/9/10 为白灯直刷。
fn led_unit_label(unit: usize) -> String {
    match unit {
        0..=7 => format!("{} 按键灯{}", unit, unit + 1),
        8 => "8 Body 白灯".to_string(),
        9 => "9 Ext 白灯".to_string(),
        10 => "10 Side 白灯".to_string(),
        _ => format!("{} ?", unit),
    }
}

/// 协议页 11 个虚拟 LED 单元行: 采样色(LED_GET 回报的当前有效色)+ 映射(草稿优先)。
fn build_led_unit_rows(ctrl: &AppController) -> Vec<LedUnitRow> {
    (0..LED_UNIT_COUNT)
        .map(|unit| {
            let rgb = ctrl.led_color(unit);
            let region = ctrl.led_region(unit);
            LedUnitRow {
                unit: unit as i32,
                label: led_unit_label(unit).into(),
                sample: slint::Color::from_rgb_u8(rgb[0], rgb[1], rgb[2]),
                rgb_text: format!("R{} G{} B{}", rgb[0], rgb[1], rgb[2]).into(),
                ch_choice: if region.ch > 1 {
                    0
                } else {
                    region.ch as i32 + 1
                },
                start: region.start as i32,
                count: region.count as i32,
            }
        })
        .collect()
}
