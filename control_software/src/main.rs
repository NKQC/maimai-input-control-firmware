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

use mai2control_ui::app_state::{AppController, CompiledAlgo, ConnState, zone_label};
use mai2control_ui::proto::{
    CfgValue, ConfigEntry, FIELD_BASELINE, FIELD_DIFF, FIELD_LATENCY, FIELD_RAW, FIELD_STATS,
    FIELD_STATUS, PARAM_FINGER_TH, PARAM_NOISE_TH, PARAM_RESOLUTION, PARAM_SNS_CLK_DIV,
    PARAM_SNS_CLK_SOURCE,
};
use mai2control_ui::proto::{LED_CH_UNMAPPED, LED_PREVIEW_ALL, LED_UNIT_COUNT};
use mai2control_ui::touch_geometry;
use mai2control_ui::vcam::{self, FRAME_H, FRAME_W, VcamState};
use mai2control_ui::vcam::{backend as vcam_backend, share::FramePublisher};
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

/// 全 36 通道掩码(36 位全 1): 全通道校准/基线复位用同一动作接口(ch_mask)一次覆盖所有通道。
const ALL_CHANNELS_MASK: u64 = (1u64 << 36) - 1;

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
const CP_SWEEP_START_DELAY: Duration = Duration::from_millis(1500);
/// 同一通道未出结果时的重试间隔。
const CP_SWEEP_RETRY: Duration = Duration::from_millis(400);
/// 同一通道最多请求次数: 到顶即跳过该通道, 绝不无限重试(NAK/无响应风暴的根因之一)。
const CP_SWEEP_MAX_TRIES: u8 = 3;
/// 整轮抓取硬超时: 到点即停, 不管还剩几个通道。
const CP_SWEEP_TIMEOUT: Duration = Duration::from_secs(20);

/// 手动电容测量的一轮全通道抓取状态。
/// ★没有任何自动/周期性 Cp 获取★: 仅用户点击"测量电容"时置 started_at, 之后每 tick 最多发 1 条
/// CP_GET 顺序推进 0..35, 全部取完(或超时)即回到空闲; 单通道最多 CP_SWEEP_MAX_TRIES 次。
struct CpSweep {
    started_at: Option<Instant>,
    next_at: Option<Instant>,
    ch: u8,
    tries: u8,
    req_version: u64,
    status: String,
}

impl CpSweep {
    /// 结束本轮抓取(完成/超时/失败/断连)并落最终文案。
    fn _stop(&mut self, status: String) {
        self.started_at = None;
        self.next_at = None;
        self.tries = 0;
        self.status = status;
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

/// 把当前界面偏好写回 config.cfg。由 16ms tick 调用: UiConfig::set_* 内部只在值真变了才落盘,
/// 所以这里无脑对账即可, 不必给每个开关都挂一个回调(那样每加一项设置都得改多处)。
fn persist_ui_settings(ui: &AppWindow, cfg: &mut mai2control_ui::ui_config::UiConfig) {
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
    // 映射可在关闭虚拟相机时保留；启动即创建能让日志明确记录当前命名空间，且关闭后
    // Frame Server 重新拉起时仍有合法黑帧可读。Global 被拒绝时绝不静默退回 Local。
    let frame_publisher = Rc::new(RefCell::new(match FramePublisher::create() {
        Ok(publisher) => Some(publisher),
        Err(error) => {
            log::error!("虚拟摄像头: 共享内存创建失败: {}", error);
            None
        }
    }));
    let share_runtime_status = frame_publisher
        .borrow()
        .as_ref()
        .map(|publisher| {
            format!(
                "未运行 · 共享内存命名空间 {}",
                publisher.namespace().label()
            )
        })
        .unwrap_or_else(|| "未运行 · 共享内存创建失败".to_string());
    ui.set_vcam_runtime_status(share_runtime_status.into());
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
    // COM 指针只在本 UI 线程的 Rc<RefCell> 中保存，绝不交给 Raw Input 线程。
    let virtual_camera = Rc::new(RefCell::new(None::<vcam_backend::VirtualCamera>));
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

    // 全通道频率自适应下探(0xFF=统一分频; 阻塞类, 结果经 tick 回填 auto_tune_status)。
    let ctrl_clone = controller.clone();
    ui.on_auto_tune(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.auto_tune(0xFF);
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

    // 全通道校准 / 全通道基线复位(立即执行的动作, 不进草稿)。掩码 = 36 位全 1。
    let ctrl_clone = controller.clone();
    ui.on_global_calibrate(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.calibrate(ALL_CHANNELS_MASK);
    });

    let ctrl_clone = controller.clone();
    ui.on_global_baseline_reset(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.baseline_reset(ALL_CHANNELS_MASK);
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
    ui.on_combo_add(move || {
        ctrl_clone.borrow_mut().kbd_combo_commit_pending();
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

    let ctrl_clone = controller.clone();
    ui.on_batch_set_source(move |ch| {
        if !(0..36).contains(&ch) {
            return;
        }
        ctrl_clone.borrow_mut().batch_set_source(ch as u8);
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
    ui.on_kbd_set_keycfg(move |idx, pol_high, debounce_us| {
        if !(0..12).contains(&idx) {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.kbd_set_keycfg(
            idx as u8,
            pol_high,
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

    let ctrl_clone = controller.clone();
    ui.on_kbd_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_request_map();
        let _ = ctrl.kbd_request_touchmap();
        let _ = ctrl.kbd_request_hold();
        let _ = ctrl.kbd_request_keycfg();
        let _ = ctrl.kbd_request_state();
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
        if ctrl.mode_draft().or(ctrl.csd_mode()) != Some(1) {
            return;
        }
        let _ = ctrl.set_param_all(param_id as u8, value as u32);
    });

    // 全局页始终以 CH0 作为三项 CSD 采样参数的代表值。
    let ctrl_clone = controller.clone();
    ui.on_csd_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.request_params(0);
    });

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

    // 曲线页
    let ctrl_clone = controller.clone();
    ui.on_telem_start(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.start_telemetry(
            100,
            FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS,
            0xFFFFFFFF_FFFFFFFFu64,
        );
    });

    let ctrl_clone = controller.clone();
    ui.on_telem_stop(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.stop_telemetry();
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
        if ctrl.mode_draft().or(ctrl.csd_mode()) != Some(1) {
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
        if ctrl.mode_draft().or(ctrl.csd_mode()) != Some(1) {
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
        ui.set_current_view(cfg.get_i32(k::CURRENT_VIEW, 0).clamp(0, 3));
        // 上限 8: 共 9 个标签(0绑区 1协议 2触控通道 3触控全局 4精调 5触控键映 6物理键盘 7通信 8算法)。
        ui.set_settings_tab(cfg.get_i32(k::SETTINGS_TAB, 0).clamp(0, 8));
        ui.set_sel_channel(cfg.get_i32(k::SEL_CHANNEL, 0).clamp(0, 35));
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

    // 虚拟摄像头: 开启前先核验机器级注册；共享映射和 COM 后端都在 UI 线程创建，
    // 避免 Frame Server 与 Raw Input 线程之间错误共享 COM 指针。
    let vcam_cb = vcam.clone();
    let publisher_cb = frame_publisher.clone();
    let camera_cb = virtual_camera.clone();
    let ui_vcam = ui_weak.clone();
    ui.on_set_vcam_enabled(move |on| {
        let Some(ui) = ui_vcam.upgrade() else { return };
        let namespace = if on && publisher_cb.borrow().is_none() {
            match FramePublisher::create() {
                Ok(publisher) => {
                    let namespace = publisher.namespace();
                    *publisher_cb.borrow_mut() = Some(publisher);
                    Some(namespace)
                }
                Err(error) => {
                    log::error!("虚拟摄像头: 无法启动帧发布: {}", error);
                    vcam_cb.set_enabled(false);
                    ui.set_vcam_enabled(false);
                    ui.set_vcam_runtime_status("未运行 · 共享内存创建失败".into());
                    return;
                }
            }
        } else {
            publisher_cb
                .borrow()
                .as_ref()
                .map(|publisher| publisher.namespace())
        };
        let Some(namespace) = namespace else { return };
        if on {
            match vcam_backend::is_registered() {
                Ok(true) => {}
                Ok(false) => {
                    log::warn!("虚拟摄像头: 未注册媒体源 DLL；请先点击“安装(需管理员)”");
                    vcam_cb.set_enabled(false);
                    ui.set_vcam_enabled(false);
                    ui.set_vcam_install_status(vcam_backend::registration_status().into());
                    ui.set_vcam_runtime_status(
                        format!("未运行 · 共享内存命名空间 {} · 需先安装", namespace.label())
                            .into(),
                    );
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
            if camera_cb.borrow().is_none() {
                match vcam_backend::VirtualCamera::start() {
                    Ok(camera) => *camera_cb.borrow_mut() = Some(camera),
                    Err(error) => {
                        log::error!("虚拟摄像头: Media Foundation 启动失败: {}", error);
                        vcam_cb.set_enabled(false);
                        ui.set_vcam_enabled(false);
                        ui.set_vcam_runtime_status(
                            format!("未运行 · 共享内存命名空间 {} · 启动失败", namespace.label())
                                .into(),
                        );
                        return;
                    }
                }
            }
            let access = camera_cb
                .borrow()
                .as_ref()
                .map(|camera| camera.access_name())
                .unwrap_or("未知");
            vcam_cb.set_enabled(true);
            vcam::keyboard::start(vcam_cb.clone());
            ui.set_vcam_runtime_status(
                format!(
                    "运行中 ({}) · 共享内存命名空间 {}",
                    access,
                    namespace.label()
                )
                .into(),
            );
        } else {
            vcam_cb.set_enabled(false);
            vcam::keyboard::stop();
            if let Some(camera) = camera_cb.borrow_mut().take() {
                camera.stop();
            }
            ui.set_vcam_runtime_status(
                format!("未运行 · 共享内存命名空间 {}", namespace.label()).into(),
            );
        }
    });
    // ★安装/卸载必须离开 UI 线程★: 提权走 ShellExecute("runas"), UAC 弹窗期间调用会阻塞,
    // 再加上要等 regsvr32 跑完才能核验注册表 —— 全都压在事件循环里就是标题栏那个"(未响应)"。
    // 这里丢到工作线程, 结果用 invoke_from_event_loop 回投给 UI。
    let ui_vcam_install = ui_weak.clone();
    ui.on_install_vcam(move || {
        if let Some(ui) = ui_vcam_install.upgrade() {
            ui.set_vcam_install_status("安装中… 请在 UAC 弹窗上确认".into());
        }
        let ui_done = ui_vcam_install.clone();
        std::thread::Builder::new()
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
                });
            })
            .ok();
    });
    let ui_vcam_uninstall = ui_weak.clone();
    let camera_uninstall = virtual_camera.clone();
    let vcam_uninstall = vcam.clone();
    ui.on_uninstall_vcam(move || {
        let Some(ui) = ui_vcam_uninstall.upgrade() else {
            return;
        };
        vcam_uninstall.set_enabled(false);
        vcam::keyboard::stop();
        if let Some(camera) = camera_uninstall.borrow_mut().take() {
            camera.stop();
        }
        ui.set_vcam_install_status("卸载中… 请在 UAC 弹窗上确认".into());
        ui.set_vcam_enabled(false);
        // 同安装: 提权 + 等命令结束不能压在事件循环里。
        let ui_done = ui_vcam_uninstall.clone();
        std::thread::Builder::new()
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
                });
            })
            .ok();
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
    let mut last_telem_version = 0u64;
    // 用非初始值确保即使尚未收到遥测，也先把完整 36 通道卡片回填到 UI。
    let mut last_telem_version_all = u64::MAX;
    // Cp 缓存版本门控：无新遥测但有新 Cp 响应时也要刷新全通道卡片的 cp_text。
    let mut last_cp_version_all = u64::MAX;
    let mut last_param_version = 0u64;
    let mut last_batch_sel_version = u64::MAX;
    let mut last_combo_version = u64::MAX;
    let mut last_channel = -1i32;
    let mut last_curve_visibility = (false, false, false);
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
    let timer = slint::Timer::default();
    timer.start(slint::TimerMode::Repeated, std::time::Duration::from_millis(16), move || {
        let ui = ui_weak.upgrade().unwrap();
        let mut ctrl = controller.borrow_mut();

        // 虚拟摄像头: 推进时序状态机(显示到期转黑)，帧变化时先提交 RGB24 到命名
        // 共享内存，再刷新 UI 预览。序号提交由 FramePublisher 保证在像素写完之后发生。
        vcam_timer.tick();
        let vframe_ver = vcam_timer.frame_version();
        if vframe_ver != last_vcam_frame_version {
            last_vcam_frame_version = vframe_ver;
            let rgb = vcam_timer.frame_copy();
            if let Some(publisher) = publisher_timer.borrow_mut().as_mut() {
                if let Err(error) = publisher.publish(&rgb) {
                    log::error!("虚拟摄像头: 帧共享发布失败: {}", error);
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
        ctrl.poll();
        // 侦听绑定: 捕获下一次触摸的物理通道并写入草稿(仅在侦听态时有动作)。
        let _ = ctrl.listen_tick();

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
        // 握手重试:仍在 Connecting(已发首个 HELLO 但未收到 DEVICE_INFO)时每 ~400ms 重发 HELLO。
        // 重连场景下设备端 bulk OUT data toggle 与新句柄不同步会丢弃首个 HELLO, 重发即可握手成功。
        if ctrl.state() == ConnState::Connecting && reconnect_tick % 25 == 0 {
            let _ = ctrl.resend_hello();
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
            String::new()
        };
        ui.set_auto_tune_status(auto_tune_status.into());
        let sel_ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let curve_auto_tune_status = match ctrl.auto_tune_ch_result(sel_ch) {
            1 => format!("✔ CH{} 自适应成功: snsClk 分频 = {}", sel_ch, ctrl.auto_tune_ch_div(sel_ch)),
            2 => format!("✖ CH{} 自适应失败: 已到硬件频率下限仍压不到目标%, 请降低校准目标% 或调整 IDAC", sel_ch),
            _ => String::new(),
        };
        ui.set_curve_auto_tune_status(curve_auto_tune_status.into());

        // 界面偏好对账落盘(值未变则不写盘)。放在 tick 里而不是给每个开关挂回调:
        // Slint 侧有些开关(自动滚动、页签、通道)是直接改 in-out 属性的, 本来就没有回调可挂。
        persist_ui_settings(&ui, &mut ui_cfg_tick.borrow_mut());

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
            algo_overlay_dirty = true;
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
        // DEVICE_INFO 诊断携带的 mode 是唯一真值；未知时 UI 显示“读取中”，不再以默认 AUTO 冒充设备状态。
        ui.set_mode_known(ctrl.csd_mode().is_some());
        if let Some(mode) = ctrl.mode_draft().or(ctrl.csd_mode()) {
            ui.set_scan_mode((mode != 0) as i32);
        }
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
                || (ui.get_light_panel_expanded() && reconnect_tick % 12 == 0)
            {
                let _ = ctrl.led_request_state();
            }
        }
        last_protocol_visible = protocol_visible;

        if connected && reconnect_tick % 63 == 0 {
            let _ = ctrl.ping();
        }
        // 物理键盘实时态 ~3Hz 轮询(每 20 tick),降低 vendor IN 负载(键状态非高频需求)。
        if connected && reconnect_tick % 20 == 0 {
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
            if reconnect_tick % 4 == 0 {
                let _ = ctrl.kbd_request_state();
            }
            if ui.get_phys_la_expanded() && reconnect_tick % 2 == 0 {
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
            if algo_trace_page && !algo_report_idxs.is_empty() {
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
        }

        // 34 分区固定: 原地比较后只写变化行，保留列表内通道输入、侦听与移除按钮的焦点状态。
        let zones = build_zone_cells(&ctrl);
        for (row, zone) in zones.into_iter().enumerate() {
            if zones_model.row_data(row).as_ref() != Some(&zone) {
                zones_model.set_row_data(row, zone);
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
                } else if now.duration_since(started_at) >= CP_SWEEP_TIMEOUT {
                    let done = cp_state.ch;
                    cp_state._stop(format!("电容测量超时: 仅完成 {}/36 通道, 正在重启 PSoC 恢复扫描…", done));
                    // 半途超时同样要恢复: BIST 已经改过 CSD 硬件, 不重启会留下 raw 满量程/采样率崩塌的坏状态。
                    let _ = ctrl.reboot_psoc();
                } else {
                    // 本通道是否已"落定": 响应到达(版本推进)且不是"测量中(0)"。
                    let ch = cp_state.ch;
                    let responded = cp_state.tries > 0 && ctrl.cp_channel_version(ch) > cp_state.req_version;
                    if responded && ctrl.cp(ch) != Some(0) {
                        cp_state._advance();
                    }
                    let due = cp_state.next_at.map_or(true, |deadline| now >= deadline);
                    if cp_state.ch >= 36 {
                        let ok = (0..36u8)
                            .filter(|&c| matches!(ctrl.cp(c), Some(v) if v != 0 && v != CP_MEASURE_FAILED))
                            .count();
                        cp_state._stop(format!("电容测量完成: {}/36 通道有效, 正在重启 PSoC 恢复扫描…", ok));
                        // ★BIST 后必须重启 PSoC★: Cp 测量(BIST)会重配 CSD 硬件, 其自带的恢复路径
                        // (Cy_CapSense_Enable/Initialize)实测不可靠——测完全通道 raw 卡 4095、采样率掉到 1-7Hz,
                        // 只有 XRES 重启能复原(重启后立即回到 ~171Hz)。故测量收尾自动重启, 免得用户点一次就把
                        // 扫描搞废还以为是"扫描引擎卡死"。
                        let _ = ctrl.reboot_psoc();
                        ctrl.push_log("测量电容: BIST 会重配 CSD 硬件, 已自动重启 PSoC 恢复扫描(约 1s)");
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
        }

        // 主图回填: 主曲线(左轴) + 算法叠加线(右轴)共享同一时间窗口, 必须在同一处生成。
        // 门控里【不再】有绘图区尺寸: PlotPath 用 fit: fill 拉满, 拖分栏/改窗口只是元素几何变化,
        // path 不必重算 —— 纯 UI 交互不该把 36 通道的采样重新投影一遍。
        if current_telem_version != last_telem_version
            || channel_changed
            || curve_visibility_changed
            || algo_overlay_dirty
        {
            last_telem_version = current_telem_version;
            last_channel = current_channel;
            last_curve_visibility = curve_visibility;
            algo_overlay_dirty = false;
            let curves = build_curve_paths(
                &ctrl,
                current_channel as u8,
                curve_visibility.0,
                curve_visibility.1,
                curve_visibility.2,
            );
            // 横轴真实时间: 跨度(ms)给 UI 换算刻度/十字线读数, 绝对时刻只做一行文本展示
            // (f32 存不住小时级的 ms 绝对值, 故 UI 侧一律用"相对最新样本"的偏移)。
            ui.set_curve_t_span_ms(curves.t_span_ms());
            ui.set_curve_t_end_text(dev_time_text(curves.t_last_us).into());

            // 算法叠加线: 与主曲线同一时间窗口 + 独立右轴量程。
            let overlay_on = ui.get_show_algo_overlay();
            let show_active = ui.get_show_active();
            let decls = ctrl.algo_report_decls();
            let amps = *report_norm_timer.borrow();
            let mut wanted = [false; 4];
            for (idx, slot) in wanted.iter_mut().enumerate() {
                *slot = overlay_on
                    && decls.iter().any(|d| d.idx == idx as u8)
                    && report_show_timer.borrow()[idx];
            }
            let overlay = build_algo_overlay(
                &ctrl,
                &wanted,
                show_active,
                &amps,
                curves.t_first_us,
                curves.t_last_us,
                algo_report_idxs.len(),
            );
            ui.set_curve_r_min(overlay.r_min);
            ui.set_curve_r_max(overlay.r_max);
            ui.set_active_point_count(overlay.active_count);
            ui.set_active_path(overlay.active_path.into());
            ui.set_active_norm_amp(amps[4]);
            let colors: [slint::Color; 4] = [
                slint::Color::from_rgb_u8(0x33, 0xcc, 0x33),
                slint::Color::from_rgb_u8(0x33, 0x99, 0xff),
                slint::Color::from_rgb_u8(0xff, 0xaa, 0x00),
                slint::Color::from_rgb_u8(0xff, 0x66, 0xcc),
            ];
            for idx in 0usize..4 {
                let decl = decls.iter().find(|d| d.idx == idx as u8);
                let line = AlgoReportLine {
                    idx: idx as i32,
                    name: decl.map(|d| d.name.clone()).unwrap_or_default().into(),
                    path: overlay.report_paths[idx].clone().into(),
                    // 声明 + 有数据 + 用户未取消勾选 → 才画在主图上。
                    visible: wanted[idx] && overlay.report_has_data[idx],
                    is_binary: overlay.report_binary[idx],
                    norm_amp: amps[idx],
                    line_color: colors[idx],
                };
                if algo_report_lines_model_timer.row_data(idx).as_ref() != Some(&line) {
                    if idx < algo_report_lines_model_timer.row_count() {
                        algo_report_lines_model_timer.set_row_data(idx, line);
                    } else {
                        algo_report_lines_model_timer.push(line);
                    }
                }
            }

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

            // 当前值读数(raw/diff/baseline + 量程 + 阈值), 兑现"折线图数值与范围提醒"。
            let (lr, ld, lb) = match ctrl.telem_latest(current_channel as u8) {
                Some(s) => (
                    s.raw.unwrap_or(0) as i32,
                    s.diff.unwrap_or(0) as i32,
                    s.bsln.unwrap_or(0) as i32,
                ),
                None => (0, 0, 0),
            };
            let readout = if curves.point_count > 0 {
                format!(
                    "CH{} 当前 raw={} diff={} bsln={} | 纵轴量程 [{} .. {}] | 手指阈值 {} 噪声阈值 {}",
                    current_channel, lr, ld, lb,
                    curves.y_min.round() as i32, curves.y_max.round() as i32,
                    finger_th as i32, noise_th as i32
                )
            } else {
                format!("CH{} 等待遥测数据 | 手指阈值 {} 噪声阈值 {}",
                    current_channel, finger_th as i32, noise_th as i32)
            };
            ui.set_curve_readout(readout.into());
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
            ui.set_combo_zones_text(slint::ModelRc::new(slint::VecModel::from(zones_text)));
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

        // 批量应用抽屉回填: 勾选态走自己的 version, 源通道参数值随 param_version/换通道刷新。
        // 两者都并入既有 16ms tick 门控, 不新增定时器、不提高频率。
        let current_batch_sel_version = ctrl.batch_sel_version();
        if current_batch_sel_version != last_batch_sel_version
            || current_param_version != last_param_version
            || channel_changed
        {
            last_batch_sel_version = current_batch_sel_version;
            let batch_src = ctrl.batch_source();
            ui.set_batch_source(batch_src as i32);
            ui.set_batch_ch_selected(slint::ModelRc::new(slint::VecModel::from(
                ctrl.batch_ch_selected())));
            ui.set_batch_param_selected(slint::ModelRc::new(slint::VecModel::from(
                ctrl.batch_param_selected())));
            let names: Vec<slint::SharedString> = (0x01u8..=0x0Bu8)
                .map(|id| param_display_name(id).into())
                .collect();
            ui.set_batch_param_names(slint::ModelRc::new(slint::VecModel::from(names)));
            // 源通道该参数无真值时显示"—", 明确区别于"值为 0"。取的是 batch_source 而不是当前精调
            // 通道: 抽屉里勾目标通道时不应把"源值"一列跟着改掉。
            let values: Vec<slint::SharedString> = (0x01u8..=0x0Bu8)
                .map(|id| match ctrl.param(batch_src, id) {
                    Some(v) => slint::SharedString::from(v.to_string()),
                    None => slint::SharedString::from("—"),
                })
                .collect();
            ui.set_batch_param_values(slint::ModelRc::new(slint::VecModel::from(values)));
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
            }
        }

        // 全通道实时状态：随遥测版本或 Cp 缓存版本刷新(遥测流全 36 通道；Cp 无新遥测时
        // 也可能因 CP_GET 响应更新，否则 cp_text 会卡在旧值不刷新)。active=status bit0。
        // 原地按行更新(仅变化的行才写)，不替换 model，避免元素重建导致的悬浮闪烁/交互丢失。
        let current_cp_version_all = ctrl.cp_version();
        if current_telem_version != last_telem_version_all || current_cp_version_all != last_cp_version_all {
            last_telem_version_all = current_telem_version;
            last_cp_version_all = current_cp_version_all;
            let all = build_channel_status(&ctrl);
            for (i, row) in all.into_iter().enumerate() {
                if all_channels_model.row_data(i).as_ref() != Some(&row) {
                    all_channels_model.set_row_data(i, row);
                }
            }
        }

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
            let pol: Vec<bool> = (0..12u8).map(|i| ctrl.kbd_keycfg(i).active_high).collect();
            let db: Vec<i32> = (0..12u8)
                .map(|i| ctrl.kbd_keycfg(i).debounce_us as i32)
                .collect();
            ui.set_kbd_phys_pol_high(slint::ModelRc::new(slint::VecModel::from(pol)));
            ui.set_kbd_phys_debounce(slint::ModelRc::new(slint::VecModel::from(db)));
            // 旧固件没有这两条命令: 明说"不支持"而不是显示一份看起来能改的假默认值。
            let status = match ctrl.kbd_keycfg_supported() {
                Some(false) => "设备固件不支持每键极性/防抖(需升级固件)".to_string(),
                None => "每键极性/防抖: 未回读".to_string(),
                Some(true) => {
                    let high = (0..12u8).filter(|i| ctrl.kbd_keycfg(*i).active_high).count();
                    format!("{} 键高电平触发 · 其余低电平触发", high)
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
    entries
        .iter()
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
                min_val,
                max_val,
                has_range,
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

/// 返回 (分组中文名, 配置项中文名, 小字说明)。未收录的 key 回退到英文 key 的可读形式。
fn parse_config_label(key: &str) -> (String, String, String) {
    let parts: Vec<&str> = key.split('.').collect();
    // ★group 就是渲染位置★ 各 Slint 端的 ConfigGroupSection 用 group_title 精确匹配取行,
    // 所以归属按语义逐 key 指定, 而不是按 key 的一级前缀 —— comm./led. 前缀下同时混着
    // "协议能力参数"(属协议页)与"键盘映射/状态指示灯"(属通信系统 Tab), 前缀分不开。
    let group = match key {
        // mai2serial: 串口本身 + 触控上报节流 + {E}RSET 善后行为 → 协议页 mai2serial 块
        "comm.serial_baud"
        | "comm.touch_delay_100us"
        | "comm.sample_delay_ms"
        | "comm.send_only_on_change"
        | "comm.aggregation_delay_ms"
        | "comm.extra_send"
        | "comm.rate_limit_en"
        | "comm.rate_limit_hz"
        | "comm.serial_reset_calibrate"
        | "comm.serial_reset_baseline" => "mai2serial 协议参数",
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
        "comm.sample_delay_ms" => ("采样延迟 (ms)", "每次采样后延迟, 降低上报频率"),
        "comm.send_only_on_change" => ("仅变化时发送", "触摸状态无变化则不上报, 省带宽"),
        "comm.aggregation_delay_ms" => ("聚合延迟 (ms)", "合并一段时间的采样后再一次上报"),
        "comm.extra_send" => ("额外重发次数", "每帧额外多发几次以防丢包"),
        "comm.rate_limit_en" => ("启用速率限制", "限制遥测上报的最大帧率"),
        "comm.rate_limit_hz" => ("速率上限 (Hz)", "遥测上报帧率的上限"),
        "comm.keyboard_map_en" => ("启用触摸→键盘", "把触摸分区映射为键盘按键输出"),
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
/// None=本会话尚未测量(无任何自动获取)，Some(0)=测量中，Some(CP_MEASURE_FAILED)=未测量/测量失败，
/// 其余=正常测量值。
fn cp_display_text(cp: Option<u32>) -> String {
    match cp {
        None => "未测量".to_string(),
        Some(0) => "测量中…".to_string(),
        Some(CP_MEASURE_FAILED) => "测量失败".to_string(),
        Some(value) => format!("{:.2} pF", value as f64 / 1000.0),
    }
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

/// 单通道精调主图一次回填的产物: 左轴(ADC 量纲)三条曲线 + 全量共享时间窗口。
/// 算法叠加线(右轴)复用同一时间窗口, 见 `build_algo_overlay`。
struct CurvePaths {
    raw_path: String,
    bsln_path: String,
    diff_path: String,
    y_min: f32,
    y_mid: f32,
    y_max: f32,
    point_count: i32,
    /// 全量时间窗口(展开后的设备时间 us): 最旧样本→最新样本。UI 的横向缩放在其上取子区间。
    t_first_us: u64,
    t_last_us: u64,
}

impl CurvePaths {
    fn value_to_y(&self, value: f32) -> f32 {
        if self.point_count == 0 {
            return 500.0;
        }
        (1000.0 - (value - self.y_min) / (self.y_max - self.y_min) * 1000.0).clamp(0.0, 1000.0)
    }
    /// 全量时间跨度(ms), 供 UI 把 viewbox 横坐标换算成真实时刻。
    fn t_span_ms(&self) -> f32 {
        self.t_last_us.saturating_sub(self.t_first_us) as f32 / 1000.0
    }
}

/// 采样间隔超过多少视为"时间缺口"(暂停/掉帧): 取标称周期的若干倍, 并给一个绝对下限,
/// 免得采样率未知(sps=0)或抖动时误判。缺口两侧不连线, 见 `points_to_svg_path`。
fn gap_threshold_us(sample_rate_hz: u32, period_mult: u32, floor_us: u64) -> u64 {
    let nominal = if sample_rate_hz > 0 {
        1_000_000 / sample_rate_hz as u64
    } else {
        33_333
    };
    (nominal * period_mult as u64).max(floor_us)
}

/// 把 (设备时间us, 值) 序列按时间窗口 [t0,t1] 与数值窗口 [v_min,v_max] 投影成 SVG path。
///
/// ★横坐标按时间而非序号★: 掉帧/暂停时等间距序号会把时间轴画错(同样的像素距离代表不同时长)。
/// ★缺口断开★: 相邻两点间隔超过 gap_us 就用 `M` 重开子路径 —— 直线插值会伪造"这段时间数值在
/// 线性变化"的假象, 停流几十秒后尤其误导。
/// ★x 值域固定 [0,1000]★: 主图 PlotPath 用 fit: fill(非等比拉伸), viewbox 无论什么宽高比都被
/// 拉满绘图区, 所以不需要按绘图区宽高比预拉伸 x —— 那套修正在横向缩放后必然失配并留出空白带。
fn points_to_svg_path(
    points: &[(u64, f32)],
    t0: u64,
    t1: u64,
    v_min: f32,
    v_max: f32,
    gap_us: u64,
) -> String {
    if points.is_empty() {
        return String::new();
    }
    let t_span = t1.saturating_sub(t0).max(1) as f32;
    let v_range = (v_max - v_min).max(f32::EPSILON);
    let mut path = String::new();
    let mut prev_t: Option<u64> = None;
    for &(t, v) in points {
        let x = t.saturating_sub(t0) as f32 / t_span * 1000.0;
        let y = (1000.0 - (v - v_min) / v_range * 1000.0).clamp(0.0, 1000.0);
        let broken = prev_t.map_or(true, |p| t.saturating_sub(p) > gap_us);
        if path.is_empty() {
            path.push_str(&format!("M {} {}", x as i32, y as i32));
        } else {
            path.push_str(&format!(
                " {} {} {}",
                if broken { "M" } else { "L" },
                x as i32,
                y as i32
            ));
        }
        prev_t = Some(t);
    }
    path
}

fn build_curve_paths(
    ctrl: &AppController,
    ch: u8,
    show_raw: bool,
    show_bsln: bool,
    show_diff: bool,
) -> CurvePaths {
    let raw_pts = if show_raw {
        ctrl.telem_points(ch, FIELD_RAW)
    } else {
        vec![]
    };
    let bsln_pts = if show_bsln {
        ctrl.telem_points(ch, FIELD_BASELINE)
    } else {
        vec![]
    };
    let diff_pts = if show_diff {
        ctrl.telem_points(ch, FIELD_DIFF)
    } else {
        vec![]
    };
    let all_pts = [&raw_pts[..], &bsln_pts[..], &diff_pts[..]];

    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    let mut point_count = 0usize;
    let mut t_first = u64::MAX;
    let mut t_last = 0u64;
    for points in all_pts {
        point_count = point_count.max(points.len());
        if let (Some(first), Some(last)) = (points.first(), points.last()) {
            t_first = t_first.min(first.0);
            t_last = t_last.max(last.0);
        }
        for &(_, value) in points {
            if value.is_finite() {
                min = min.min(value);
                max = max.max(value);
            }
        }
    }

    if !min.is_finite() || !max.is_finite() || t_first > t_last {
        return CurvePaths {
            raw_path: String::new(),
            bsln_path: String::new(),
            diff_path: String::new(),
            y_min: 0.0,
            y_mid: 0.5,
            y_max: 1.0,
            point_count: 0,
            t_first_us: 0,
            t_last_us: 0,
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
    // 遥测帧缺口: 超过 5 个标称周期(且至少 250ms)才算缺口, 容忍正常的帧间抖动。
    let gap_us = gap_threshold_us(ctrl.telem_samples_per_sec(), 5, 250_000);

    CurvePaths {
        raw_path: points_to_svg_path(&raw_pts, t_first, t_last, y_min, y_max, gap_us),
        bsln_path: points_to_svg_path(&bsln_pts, t_first, t_last, y_min, y_max, gap_us),
        diff_path: points_to_svg_path(&diff_pts, t_first, t_last, y_min, y_max, gap_us),
        y_min,
        y_mid,
        y_max,
        point_count: point_count as i32,
        t_first_us: t_first,
        t_last_us: t_last,
    }
}

/// 算法叠加线(4 条上报 + 触发判定)在主图上的一次回填产物。
/// 走**独立右轴**: 算法量纲(计数/permille/布尔)与 ADC 量纲无关, 共用左轴会被压成一条平线。
struct AlgoOverlay {
    /// 右轴量程(已含二值线归一化后的幅度)。
    r_min: f32,
    r_max: f32,
    report_paths: [String; 4],
    /// 某条上报线是否为二值(取值只有 0/1) → UI 才为它显示"归一化幅度 N"输入。
    report_binary: [bool; 4],
    /// 某条上报线是否已有追踪数据(无数据的线不勾选也不算进右轴量程)。
    report_has_data: [bool; 4],
    active_path: String,
    /// 触发判定的采样点数(供 UI 判断"有无追踪数据", 免得再取一遍序列)。
    active_count: i32,
}

/// 判定二值序列: 非空且全部取值只有 0/1。0/1 在共享右轴上几乎不可见, 需要拉伸到 0..N。
fn points_are_binary(points: &[(u64, f32)]) -> bool {
    !points.is_empty() && points.iter().all(|&(_, v)| v == 0.0 || v == 1.0)
}

/// 二值线按幅度 N 拉伸(纯显示变换, 不下发设备); 非二值线原样返回。
fn normalize_binary(points: &[(u64, f32)], amp: f32) -> Vec<(u64, f32)> {
    points.iter().map(|&(t, v)| (t, v * amp)).collect()
}

/// 生成算法叠加线的 path 与右轴量程。`wanted` 决定哪些线参与右轴量程计算 ——
/// 没画出来的线不该影响量程, 否则勾掉一条线后其余线的高度会莫名其妙地变。
/// amps[0..3] 对应 report[idx], amps[4] 对应触发判定。
fn build_algo_overlay(
    ctrl: &AppController,
    wanted: &[bool; 4],
    show_active: bool,
    amps: &[f32; 5],
    t0: u64,
    t1: u64,
    poll_slots: usize,
) -> AlgoOverlay {
    // 算法追踪是主机轮询取回的: n 个 idx 轮转 → 单条线的采样间隔是遥测周期的 n 倍。
    let gap_us = gap_threshold_us(
        ctrl.telem_samples_per_sec(),
        (poll_slots.max(1) * 6) as u32,
        400_000,
    );
    let mut series: [Vec<(u64, f32)>; 4] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    let mut report_binary = [false; 4];
    let mut report_has_data = [false; 4];
    let mut r_min = f32::INFINITY;
    let mut r_max = f32::NEG_INFINITY;
    let track = |points: &[(u64, f32)]| -> (f32, f32) {
        let mut lo = f32::INFINITY;
        let mut hi = f32::NEG_INFINITY;
        for &(_, v) in points {
            if v.is_finite() {
                lo = lo.min(v);
                hi = hi.max(v);
            }
        }
        (lo, hi)
    };
    for idx in 0..4usize {
        let raw = ctrl.algo_trace_report_points(idx as u8);
        report_has_data[idx] = !raw.is_empty();
        report_binary[idx] = points_are_binary(&raw);
        let norm = if report_binary[idx] {
            normalize_binary(&raw, amps[idx])
        } else {
            raw
        };
        if wanted[idx] && report_has_data[idx] {
            let (lo, hi) = track(&norm);
            r_min = r_min.min(lo);
            r_max = r_max.max(hi);
        }
        series[idx] = norm;
    }
    let active_pts = normalize_binary(&ctrl.algo_trace_active_points(), amps[4]);
    if show_active {
        let (lo, hi) = track(&active_pts);
        r_min = r_min.min(lo);
        r_max = r_max.max(hi);
    }
    if !r_min.is_finite() || !r_max.is_finite() {
        r_min = 0.0;
        r_max = 1.0;
    }
    let span = r_max - r_min;
    let padding = if span.abs() < f32::EPSILON {
        (r_max.abs() * 0.05).max(0.5)
    } else {
        span * 0.05
    };
    let r_min = r_min - padding;
    let r_max = r_max + padding;
    let mut report_paths = [String::new(), String::new(), String::new(), String::new()];
    for idx in 0..4usize {
        report_paths[idx] = points_to_svg_path(&series[idx], t0, t1, r_min, r_max, gap_us);
    }
    AlgoOverlay {
        r_min,
        r_max,
        report_paths,
        report_binary,
        report_has_data,
        active_path: points_to_svg_path(&active_pts, t0, t1, r_min, r_max, gap_us),
        active_count: active_pts.len() as i32,
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
            cp_text: cp_display_text(ctrl.cp(ch)).into(),
            frozen: ctrl.channel_frozen(ch),
        });
    }
    out
}

fn build_param_rows(params: &[(u8, u32)]) -> Vec<ParamRow> {
    params
        .iter()
        // 单通道精调展示全部由 CSD 模式接管的逐通道参数；AUTO 时 Slint 统一显示 AUTO 并锁定，
        // SEMI 时恢复数值与编辑，避免自动实时值被误当作用户的手动设置。
        .filter(|&&(param_id, _)| matches!(param_id, 0x01..=0x06 | 0x08..=0x0B))
        .map(|&(param_id, value)| ParamRow {
            param_id: param_id as i32,
            label: param_display_name(param_id).into(),
            value: value as i32,
        })
        .collect()
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
