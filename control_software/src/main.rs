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
    AppController, ChBatchKind, CompiledAlgo, ConnState, HID_COORD_MAX, HID_POINT_COUNT,
    SWEEP_DIVS, zone_label,
};
use mai2control_ui::proto::{
    BATCH_PARAM_IDS, CfgValue, ConfigEntry, FIELD_BASELINE, FIELD_DIFF, FIELD_RAW, PARAM_ENABLED,
    PARAM_FINGER_TH, PARAM_NOISE_TH, PARAM_RESOLUTION, PARAM_SNS_CLK_DIV, PARAM_SNS_CLK_SOURCE,
};
use mai2control_ui::proto::{LED_CH_UNMAPPED, LED_PREVIEW_ALL, LED_UNIT_COUNT};
use mai2control_ui::touch_geometry;
use mai2control_ui::vcam::{self, VcamState};
use mai2control_ui::vcam::{backend as vcam_backend, share, share::FramePublisher};
use slint::Model;

slint::include_modules!();

mod ui_callbacks;
mod ui_keyboard;
mod ui_models;
mod ui_plot;
mod ui_runtime;

use ui_keyboard::{
    char_to_hid, kbd_choice_to_code, kbd_code_to_choice, kbd_display, kbd_key_choices,
    parse_hid_usage,
};

use ui_models::{
    _apply_or_queue_cfg, CP_FAILURE_TEXT, PendingCfgWrite, build_channel_status, build_config_rows,
    build_global_fences, build_hid_point_cells, build_led_unit_rows, build_param_row,
    build_param_rows, build_spectrum_cells, build_zone_cells, cfg_u32_or, channel_telemetry_state,
    cp_display_text, cp_failed_list, expected_scan_period_us, refresh_hid_screenshot,
    work_mode_text,
};

use ui_callbacks::setup_ui_callbacks;
use ui_runtime::persist_ui_settings;

use ui_plot::{
    LA_WINDOW_DEFAULT, LA_WINDOWS_US, SeriesId, SeriesShow, build_chart_frame, build_lat_path,
    build_logic_analyzer, dev_time_text, fmt_time_us,
};

// 算法 C 源模板已上移到库(mai2control_ui::algo_template): GUI 的"加载模板"/默认源回灌与
// 无头自检的同一路径复现必须用同一份字节, 不能两个二进制各 include_str! 一次。
pub(crate) use mai2control_ui::algo_template::{ALGO_LED_DEMO_TEMPLATE, ALGO_V31_TEMPLATE};

/// Cp 测量的真实失败：通道已参与测量，但结果无效。
const CP_MEASURE_FAILED: u32 = 0x00FF_FFFF;
/// Cp 未测量：通道本轮被禁用，固件刻意跳过测量。
const CP_NOT_MEASURED: u32 = 0x00FF_FFFD;

/// 项目仓库地址(关于页展示 + 一键复制到剪贴板)。
const REPO_URL: &str = "https://github.com/NKQC/maimai-input-control-firmware/tree/v4";

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

/// 设置页稳定 page id：UI 按模式隐藏标签时只转换为动态 TabWidget 索引，Rust 侧始终使用这些 id。
const SETTINGS_PAGE_BINDING_OR_HID: i32 = 0;
const SETTINGS_PAGE_PROTOCOL: i32 = 1;
const SETTINGS_PAGE_ALL_CHANNELS: i32 = 2;
const SETTINGS_PAGE_GLOBAL_TUNE: i32 = 3;
const SETTINGS_PAGE_CURVES: i32 = 4;
const SETTINGS_PAGE_PHYS_KBD: i32 = 6;
const SETTINGS_PAGE_CONFIG: i32 = 7;
const SETTINGS_PAGE_ALGO: i32 = 8;

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
    mai2control_ui::logging::init();
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
    mai2control_ui::settings_io::load_jit_metadata(&mut controller.borrow_mut());

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
