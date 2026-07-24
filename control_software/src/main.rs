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

use slint::Model;
use mai2control_ui::app_state::{AppController, ConnState, zone_label};
use mai2control_ui::proto::{ConfigEntry, CfgValue, FIELD_RAW, FIELD_BASELINE, FIELD_DIFF, FIELD_STATUS, FIELD_STATS, FIELD_LATENCY, PARAM_FINGER_TH, PARAM_NOISE_TH, PARAM_RESOLUTION, PARAM_SNS_CLK_DIV, PARAM_SNS_CLK_SOURCE};
use mai2control_ui::touch_geometry;

slint::include_modules!();

/// 内置 v3.1 HDR 触控算法源(随程序打包), 供算法页"加载模板"按钮作参考/真实下发案例。
const ALGO_V31_TEMPLATE: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../psoc_firmware/algo/psoc_algo_default.c"
));

const CP_MEASURE_FAILED: u32 = 0x00FF_FFFF;

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
    // 初始化设备列表，并回填/按需应用工具箱端口设置。
    let (auto_port_enabled, serial_com, light_com, port_status) = {
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

    // 延迟图绘图区宽高比(UI resize 回传), 供 build_lat_path 横向拉伸铺满。
    let lat_aspect = Rc::new(std::cell::Cell::new(3.0f32));
    {
        let ui_la = ui_weak.clone();
        let ctrl_la = controller.clone();
        let la = lat_aspect.clone();
        ui.on_lat_area_resized(move |a| {
            la.set(if a > 0.01 { a } else { 3.0 });
            if let Some(ui) = ui_la.upgrade() {
                let series = ctrl_la.borrow().lat_total_series();
                if !series.is_empty() {
                    let (path, lo, hi) = build_lat_path(&series, la.get());
                    ui.set_lat_path(path.into());
                    ui.set_lat_y_max(hi);
                    ui.set_lat_y_min(lo);
                    ui.set_lat_point_count(series.len() as i32);
                }
            }
        });
    }

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

    // 重启 PSoC(使需重启生效的改动生效)
    let ctrl_clone = controller.clone();
    ui.on_reboot_psoc(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reboot_psoc();
    });

    // 频率自适应下探(阻塞类, 结果经 tick 回填 auto_tune_status)。
    let ctrl_clone = controller.clone();
    ui.on_auto_tune(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.auto_tune();
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
        let _ = ctrl.set_config(ConfigEntry::new(key.to_string(), CfgValue::Str(value.to_string())));
    });

    // ---- 算法页 (JIT 触控算法 + C→ASM 编译器 + 全局设置) ----
    // 默认 C 源模板: 最小合法算法(沿用基础激活), 用户可改为自定义高动态逻辑。
    let algo_default_src = "#include <stddef.h>\n#include \"psoc_algo_abi.h\"\n\n// 入口: 每通道调用一次, 读写 io 固定字段。\n// 禁: libc / '/' '%' / 64位。辅助请 static inline。\nvoid algo(algo_io_t* io)\n{\n    // 示例: 直接沿用中间件基础激活判定。\n    // io->diff/baseline/finger_th/now_ms/rom 等可用于自定义高动态逻辑。\n    io->out_active = (io->base_active != 0u) ? 1u : 0u;\n}\n";
    ui.set_algo_c_source(algo_default_src.into());
    ui.set_algo_line_numbers(line_numbers_for(algo_default_src).into());

    let ctrl_clone = controller.clone();
    ui.on_algo_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.algo_get_info();
        let _ = ctrl.algo_get_rom();
        let _ = ctrl.request_algo_src();   // 回读设备映射表 C 源 → 还原可编辑算法
        let _ = ctrl.request_algo_code();  // 回读设备算法机器码 → 无本地编译时反汇编页看真实 ASM
        let _ = ctrl.global_get_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_algo_reset_default(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.algo_reset_default();
        // 恢复默认会清空设备侧算法 C 源 → 立即把内嵌默认源(去注释)回灌设备映射表,
        // 使"读取信息"能真正从设备取回去注释的默认 C 源(而非带注释的原始模板)。
        let _ = ctrl.send_algo_src(&AppController::strip_c_comments(ALGO_V31_TEMPLATE));
        let _ = ctrl.algo_get_info();
        let _ = ctrl.algo_get_rom();
        let _ = ctrl.request_algo_src();
    });

    // 载入内置 v3.1 HDR 模板到编辑器(参考 + 真实可下发案例)。
    let ui_tpl = ui_weak.clone();
    ui.on_algo_load_template(move || {
        let ui = ui_tpl.upgrade().unwrap();
        ui.set_algo_c_source(ALGO_V31_TEMPLATE.into());
        ui.set_algo_line_numbers(line_numbers_for(ALGO_V31_TEMPLATE).into());
    });

    // 编辑器内容变化 → 刷新行号列。
    let ui_edit = ui_weak.clone();
    ui.on_algo_source_edited(move |text| {
        let ui = ui_edit.upgrade().unwrap();
        ui.set_algo_line_numbers(line_numbers_for(text.as_str()).into());
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

    // 键盘捕获输入框: 按下组合键 → 主键+修饰位。纯修饰键(code==0)忽略, 等主键。
    let ctrl_clone = controller.clone();
    ui.on_kbd_capture_phys(move |idx, text, c, s, a, g| {
        let code = char_to_hid(text.as_str());
        if code == 0 { return; }
        let m = (c as u8) | ((s as u8) << 1) | ((a as u8) << 2) | ((g as u8) << 3);
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_set_map(idx as u8, code, m);
    });
    let ctrl_clone = controller.clone();
    ui.on_kbd_capture_zone(move |zone, text, c, s, a, g| {
        let code = char_to_hid(text.as_str());
        if code == 0 { return; }
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
    ui.on_kbd_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.kbd_request_map();
        let _ = ctrl.kbd_request_touchmap();
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

    // 编译(不上传): 产出 ASM 并显示占用/进度; compile_only 在超容量时报错, 保证塞得下不截断。
    let ctrl_clone = controller.clone();
    let ui_algo = ui_weak.clone();
    ui.on_algo_compile(move |src| {
        let ui = ui_algo.upgrade().unwrap();
        let mut ctrl = ctrl_clone.borrow_mut();
        let cap = AppController::algo_slot_capacity();
        match ctrl.compile_only(&src) {
            Ok(len) => {
                ui.set_algo_asm_bytes(len as i32);
                ui.set_algo_status(
                    format!("编译成功: ASM {} / {} 字节 ({}%), 可上传", len, cap, (len * 100) / cap).into());
            }
            Err(e) => {
                ui.set_algo_asm_bytes(0);
                ui.set_algo_status(format!("编译失败: {}", e).into());
            }
        }
    });

    // 上传最近一次成功编译的 ASM(强制先编译后上传, 避免上传未经容量校验的产物)。
    let ctrl_clone = controller.clone();
    let ui_algo = ui_weak.clone();
    ui.on_algo_upload(move || {
        let ui = ui_algo.upgrade().unwrap();
        let mut ctrl = ctrl_clone.borrow_mut();
        match ctrl.upload_compiled() {
            Ok(()) => ui.set_algo_status(format!("上传成功: {} 字节已下发设备", ctrl.algo_compiled_len()).into()),
            Err(e) => ui.set_algo_status(format!("上传失败: {}", e).into()),
        }
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
            ui.set_settings_tab(3);
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

    // 精确命中检测: 画布把点击像素换算到 SCREEN 坐标系后调用, 返回分区 index(未命中 -1)。
    // 用 point-in-polygon 取代旧 34 个重叠 bbox, 消除 A/D/E 等相邻区误选。
    ui.on_zone_hit_test(move |x, y| {
        touch_geometry::hit_test(x, y).map(|i| i as i32).unwrap_or(-1)
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

    // 从全通道状态卡进入精调：选中物理通道并切换到“单通道精调”子标签(索引 3)。
    let ui_channel = ui_weak.clone();
    ui.on_channel_selected(move |channel| {
        let ui = ui_channel.upgrade().unwrap();
        ui.set_sel_channel(channel.clamp(0, 35));
        ui.set_settings_tab(3);
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
        let _ = ctrl.set_config(ConfigEntry::new("comm.keyboard_map_en".to_string(), CfgValue::Bool(value)));
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
        let status = { ctrl_clone.borrow_mut().apply_ports() };
        if let Some(ui) = ui_ports.upgrade() {
            ui.set_port_status(status.into());
        }
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
    let mut last_channel = -1i32;
    let mut last_curve_visibility = (false, false, false);
    let mut reconnect_tick = 0u32;
    let mut was_connected = false;
    // 全通道 Cp 轮询游标(0..35): 每隔几 tick 请求一个通道, 轮流刷新全部 36 通道的 Cp。
    let mut cp_rr_channel = 0u8;
    // mode.work 保存后自动重启倒计时(tick): 给 SAVE_CONFIG 的 flash 写留出完成窗口再重启重枚举。
    let mut reboot_countdown: Option<u32> = None;
    let mut last_log_seq = u64::MAX;
    let mut last_algo_version = u64::MAX;
    let mut last_algo_src_version = u64::MAX;
    let mut last_algo_code_version = u64::MAX;
    // 默认算法的 C 源(已滤注释)是否已回灌到设备映射表: 设备默认算法出厂不带源,
    // 首次读到"默认+无源"时把内嵌默认源(去注释)下发设备一次, 使之后"读取信息"能真正从设备取回。
    let mut default_src_synced = false;
    let mut last_algo_asm_version = u64::MAX;
    // 算法追踪(report[]/out_active)/可调变量(cfg[8]): 版本门控 + 轮询游标。
    let mut last_algo_trace_version = u64::MAX;
    let mut last_algo_cfg_version = u64::MAX;
    let mut last_algo_source_for_schema = String::new();
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

    // 键盘键码下拉的共享键名表(一次性设置)。
    let kbd_choice_names: Vec<slint::SharedString> =
        kbd_key_choices().iter().map(|(n, _)| (*n).into()).collect();
    ui.set_kbd_key_choices(slint::ModelRc::new(slint::VecModel::from(kbd_choice_names)));
    // 触控分区名 A1-E8(一次性设置), 供触控键盘映射页每行标签。
    let kbd_zone_name_list: Vec<slint::SharedString> =
        (0..34u8).map(|z| zone_label(z as usize).into()).collect();
    ui.set_kbd_zone_names(slint::ModelRc::new(slint::VecModel::from(kbd_zone_name_list)));

    let mut last_kbd_state_version = u64::MAX;
    let mut last_kbd_map_version = u64::MAX;
    let mut last_kbd_touchmap_version = u64::MAX;

    let timer = slint::Timer::default();
    timer.start(slint::TimerMode::Repeated, std::time::Duration::from_millis(16), move || {
        let ui = ui_weak.upgrade().unwrap();
        let mut ctrl = controller.borrow_mut();
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
        ui.set_device_info_text(ctrl.device_info_text().into());

        // 阻塞类操作(校准/基线/重启/频率自适应)状态 → 驱动各页按钮锁定 + 运行中标识。
        ui.set_op_busy(ctrl.op_busy());
        ui.set_op_label(ctrl.op_label().into());
        // 频率自适应结果文案(成功: 统一分频; 失败: 超硬件能力)。
        let auto_tune_status = match ctrl.auto_tune_result() {
            1 => format!("✔ 自适应成功: 全通道统一 snsClk 分频 = {}", ctrl.auto_tune_div()),
            2 => "✖ 自适应失败: 已到硬件频率下限仍压不到目标%, 请降低校准目标% 或调整 IDAC".to_string(),
            _ => String::new(),
        };
        ui.set_auto_tune_status(auto_tune_status.into());

        // 连接/收发/错误 事件日志回填 UI 日志面板(仅在有新日志时刷新)。
        if ctrl.log_seq() != last_log_seq {
            last_log_seq = ctrl.log_seq();
            ui.set_log_text(ctrl.log_text().into());
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
            if !dev_src.trim().is_empty() {
                ui.set_algo_c_source(dev_src.clone().into());
                ui.set_algo_line_numbers(line_numbers_for(&dev_src).into());
                ui.set_algo_status("已从设备映射表载入当前算法 C 源(可直接修改后重新编译上传)".into());
            } else if ctrl.algo_info().map(|i| i.is_default).unwrap_or(false) {
                // 默认算法设备侧无源 → 用内嵌默认源(★去注释★, 与"上传即保存去注释"语义一致)载入编辑器,
                // 而非展示带注释的原始模板。同时把去注释源回灌设备映射表一次, 使之后"读取信息"真正从设备取回。
                let default_src = AppController::strip_c_comments(ALGO_V31_TEMPLATE);
                ui.set_algo_c_source(default_src.clone().into());
                ui.set_algo_line_numbers(line_numbers_for(&default_src).into());
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
        // 算法追踪(report[]折线 + 触发判定追踪)回填(version 门控)。
        if ctrl.algo_trace_version() != last_algo_trace_version {
            last_algo_trace_version = ctrl.algo_trace_version();
            let decls = ctrl.algo_report_decls();
            let active_series = ctrl.algo_trace_active_series();
            let point_count = active_series.len() as i32;
            let active_path = series_to_svg_path(&active_series, -0.2, 1.2, 1.0);
            ui.set_algo_active_path(active_path.clone().into());
            ui.set_algo_trace_point_count(point_count);
            ui.set_active_path(active_path.into());
            ui.set_active_point_count(point_count);
            ui.set_algo_sel_channel(ui.get_sel_channel());

            let colors: [slint::Color; 4] = [
                slint::Color::from_rgb_u8(0x33, 0xcc, 0x33),
                slint::Color::from_rgb_u8(0x33, 0x99, 0xff),
                slint::Color::from_rgb_u8(0xff, 0xaa, 0x00),
                slint::Color::from_rgb_u8(0xff, 0x66, 0xcc),
            ];
            let mut lines = Vec::with_capacity(4);
            for idx in 0u8..4u8 {
                let decl = decls.iter().find(|d| d.idx == idx);
                let series = ctrl.algo_trace_report_series(idx);
                let path = if decl.is_some() { build_report_path(&series) } else { String::new() };
                lines.push(AlgoReportLine {
                    idx: idx as i32,
                    name: decl.map(|d| d.name.clone()).unwrap_or_default().into(),
                    path: path.into(),
                    visible: decl.is_some() && !series.is_empty(),
                    line_color: colors[idx as usize],
                });
            }
            ui.set_algo_report_lines(slint::ModelRc::new(slint::VecModel::from(lines)));
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

        // 连接成功边沿主动拉取当前通道参数与 Cp，并开启全通道遥测；
        // 断线边沿清空 CpPollState，避免重连后展示旧设备/旧会话 Cp。
        let connected = ctrl.state() == ConnState::Connected;
        if connected && !was_connected {
            let ch = ui.get_sel_channel().clamp(0, 35) as u8;
            ctrl.push_log("已连接 → 自动请求配置、当前通道参数/Cp、CH0 全局采样参数 + 开启全通道遥测");
            {
                let mut cp_state = cp_poll_timer.borrow_mut();
                cp_state.started_at = None;
                cp_state.next_request_at = None;
                cp_state.measurement_start_channel_version = 0;
                cp_state.requested_channel_version = 0;
                cp_state.visible_channel = ch as i32;
                cp_state.visible_after_version = ctrl.cp_channel_version(ch);
                cp_state.waiting_for_response = false;
                cp_state.status = "读取中…".to_string();
                ui.set_cp_text(cp_state.status.clone().into());
            }
            let _ = ctrl.request_config_all();
            let _ = ctrl.request_params(ch);
            // 全局页代表值固定取 CH0；即使精调当前停在其他通道也必须拉取。
            let _ = ctrl.request_params(0);
            let _ = ctrl.request_cp(ch);
            let _ = ctrl.algo_get_info();
            let _ = ctrl.algo_get_rom();
            let _ = ctrl.request_algo_src();
            let _ = ctrl.request_algo_code();
            for idx in 0u8..8u8 {
                let _ = ctrl.request_algo_cfg(idx);
            }
            let _ = ctrl.global_get_all();
            // 连接后自动测量一次全电极 Cp: 否则设备端 Cp 缓存为 0xFFFFFF, 全通道页会一直显示
            // "测量失败"(实为从未测量)。测量为 BIST 逐电极, 完成后各页读同一 Cp 缓存显示真实电容。
            let _ = ctrl.measure_cp();
            let _ = ctrl.kbd_request_map();
            let _ = ctrl.kbd_request_touchmap();
            let _ = ctrl.kbd_request_state();
            // 遥测降到 30Hz: 100Hz 全 36 通道(~25KB/s)会把 vendor IN 打到 stall(进精调掉线根因)。
            // 30Hz 视觉仍流畅, 大幅降低 vendor IN 负载。
            let _ = ctrl.start_telemetry(
                30,
                FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS | FIELD_STATS | FIELD_LATENCY,
                0xFFFFFFFF_FFFFFFFFu64,
            );
        } else if !connected && was_connected {
            let mut cp_state = cp_poll_timer.borrow_mut();
            cp_state.started_at = None;
            cp_state.next_request_at = None;
            cp_state.measurement_start_channel_version = 0;
            cp_state.requested_channel_version = 0;
            cp_state.visible_channel = -1;
            cp_state.visible_after_version = 0;
            cp_state.waiting_for_response = false;
            cp_state.status = "未连接".to_string();
            ui.set_cp_text(cp_state.status.clone().into());
        }
        // 仅在真正进入已连接的“触控全局调整”页时拉取一次 CH0，避免 16ms tick 洪泛。
        let global_tune_visible = connected && ui.get_current_view() == 1 && ui.get_settings_tab() == 2;
        if global_tune_visible && !last_global_tune_visible && was_connected {
            let _ = ctrl.request_params(0);
        }
        last_global_tune_visible = global_tune_visible;

        if connected && reconnect_tick % 63 == 0 {
            let _ = ctrl.ping();
        }
        // 物理键盘实时态 ~3Hz 轮询(每 20 tick),降低 vendor IN 负载(键状态非高频需求)。
        if connected && reconnect_tick % 20 == 0 {
            let _ = ctrl.kbd_request_state();
        }

        // 全通道 Cp 轮询: 每 ~80ms(每 5 tick)请求一个通道, 轮流覆盖 0..35。
        // 使"触控通道调整"全通道卡片与"分区绑定"页的 Cp 不再永远停在"读取中"——
        // 只请求当前可见通道会让其余 35 个通道的 Cp 缓存永远为空。
        // 单通道手动测量进行中时暂停轮询, 避免与当前通道的版本门控互相干扰。
        if connected && reconnect_tick % 5 == 0 && cp_poll_timer.borrow().started_at.is_none() {
            let _ = ctrl.request_cp(cp_rr_channel);
            cp_rr_channel = (cp_rr_channel + 1) % 36;
        }

        // 算法运行时追踪(report[]/out_active): 选中通道~30Hz(每 tick, 16ms)轮询已声明的
        // report idx(轮转覆盖多个 idx); 未声明任何 ALGO_REPORT 时不轮询(省事务)。
        if connected {
            let src = ctrl.algo_source();
            if src != last_algo_source_for_schema {
                last_algo_source_for_schema = src.clone();
                algo_report_idxs = ctrl.algo_report_decls().into_iter().map(|d| d.idx).collect();
                algo_report_idxs.sort_unstable();
                algo_report_idxs.dedup();
                algo_trace_rr = 0;
            }
            if !algo_report_idxs.is_empty() {
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
        // mode.work(Serial/HID)草稿存在时提示"需重启生效"; 保存提交后延时自动重启设备重枚举。
        ui.set_mode_change_needs_reboot(ctrl.draft_needs_reboot());
        if reboot_countdown.is_none() && ctrl.pending_reboot() {
            // 约 800ms(50 tick)后重启, 让固件安全窗口完成 flash 写。
            reboot_countdown = Some(50);
            ctrl.clear_pending_reboot();
            ctrl.push_log("保存: mode.work 拓扑切换将在约 0.8s 后自动重启设备生效");
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
                cp_state.status = if connected { "读取中…" } else { "未连接" }.to_string();
                if cp_state.started_at.is_some() {
                    cp_state.measurement_start_channel_version = cp_state.visible_after_version;
                    cp_state.waiting_for_response = false;
                }
                if connected {
                    let _ = ctrl.request_params(ch);
                    if let Err(error) = ctrl.request_cp(ch) {
                        cp_state.status = format!("读取失败: {}", error);
                    }
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
                            cp_state.status = format!("{}（测量成功）", cp_display_text(Some(value)));
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
                cp_state.status = cp_display_text(ctrl.cp(ch));
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

            // 全局页的三项统一采样设置固定显示 CH0 代表值，不受精调通道影响。
            if let Some(value) = ctrl.param(0, PARAM_SNS_CLK_DIV) {
                ui.set_csd_sns_clk_div(value as i32);
            }
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
            let (path, lo, hi) = build_lat_path(&series, lat_aspect.get());
            ui.set_lat_path(path.into());
            ui.set_lat_y_max(hi);
            ui.set_lat_y_min(lo);
            ui.set_lat_point_count(series.len() as i32);
        }

        // 物理键盘实时按下态(version 门控)。
        if ctrl.kbd_state_version() != last_kbd_state_version {
            last_kbd_state_version = ctrl.kbd_state_version();
            let st = ctrl.kbd_state();
            let pressed: Vec<bool> = (0..12u8).map(|i| (st >> i) & 1 != 0).collect();
            ui.set_kbd_phys_pressed(slint::ModelRc::new(slint::VecModel::from(pressed)));
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
    let conv_us = (1u64 << res) / MOD_CLK_MHZ;            // 换能: 2^res / ModClk(µs), 与 snsClkDiv 无关
    (CHANNELS * (FIXED_OVERHEAD_US_PER_CH + conv_us)).min(i32::MAX as u64) as i32
}

fn build_config_rows(entries: &[ConfigEntry]) -> Vec<ConfigRow> {
    entries
        .iter()
        // 通用配置页只展示 comm./mode./led. 等; bind.map* 有"分区绑定"页, kbd.* 有键盘页,
        // 不在此重复展示(否则大量隐藏行 + 间距累积成大段空白)。
        .filter(|entry| {
            let k = entry.key.as_str();
            !k.starts_with("bind.") && !k.starts_with("kbd.")
        })
        .map(|entry| {
            let (mut kind, type_code, bool_val, num_val, min_val, max_val, has_range, mut enum_index, str_val) = match &entry.value {
                CfgValue::Bool(v) => (0, 0, *v, 0.0, 0.0, 1.0, false, 0, "".to_string()),
                CfgValue::U8(v) => (1, 2, false, *v as f32, 0.0, 255.0, true, 0, "".to_string()),
                CfgValue::U16(v) => (1, 3, false, *v as f32, 0.0, 65535.0, true, 0, "".to_string()),
                CfgValue::U32(v) => (1, 4, false, *v as f32, 0.0, 4294967295.0, true, 0, "".to_string()),
                CfgValue::I8(v) => (1, 1, false, *v as f32, -128.0, 127.0, true, 0, "".to_string()),
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
    let group = match parts.first().copied().unwrap_or("") {
        "comm" => "通信",
        "mode" => "模式",
        "light" | "led" => "灯效",
        "bind" => "绑区",
        _ => "其他",
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
/// None=读取中，Some(0)=测量中，Some(CP_MEASURE_FAILED)=测量失败，其余=正常测量值。
fn cp_display_text(cp: Option<u32>) -> String {
    match cp {
        None => "读取中…".to_string(),
        Some(0) => "测量中…".to_string(),
        Some(CP_MEASURE_FAILED) => "测量失败".to_string(),
        Some(value) => format!("{:.2} pF", value as f64 / 1000.0),
    }
}

/// 绑区页 34 分区静态几何 + 实时绑定/触摸/Cp 状态，几何来自 DXF 生成的
/// [`touch_geometry::ZONE_GEOMETRY`]，坐标系固定为 SCREEN_W x SCREEN_H。
fn build_zone_cells(ctrl: &AppController) -> Vec<ZoneCell> {
    let waiting_zone = ctrl.bind_progress().and_then(|(zone, status)| (status == 0).then_some(zone));
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
            binding_text: if channel >= 0 { format!("CH{}", channel).into() } else { "未绑定".into() },
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
        raw_path: series_to_svg_path(&raw_series, y_min, y_max, 1.0),
        bsln_path: series_to_svg_path(&bsln_series, y_min, y_max, 1.0),
        diff_path: series_to_svg_path(&diff_series, y_min, y_max, 1.0),
        y_min,
        y_mid,
        y_max,
        point_count: point_count as i32,
    }
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

/// 算法上报变量折线: 按序列自身 min/max 自适应量程(带 5% 余量), 镜像 build_curve_paths 的
/// 量程算法, 但只服务单条序列(算法上报变量语义各异, 不共享量程)。
fn build_report_path(series: &[f32]) -> String {
    if series.is_empty() {
        return String::new();
    }
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    for &v in series {
        if v.is_finite() {
            min = min.min(v);
            max = max.max(v);
        }
    }
    if !min.is_finite() || !max.is_finite() {
        return String::new();
    }
    let span = max - min;
    let padding = if span.abs() < f32::EPSILON { (min.abs() * 0.05).max(1.0) } else { span * 0.05 };
    series_to_svg_path(series, min - padding, max + padding, 1.0)
}

/// 生成延迟历史折线 path + 自适应纵向量程 (lo, hi)。x 按绘图区宽高比拉伸铺满宽度。
/// 纵向量程按数据 min/max 自适应(带 10% 余量), 否则接近常数的延迟会被压成一条线。
fn build_lat_path(series: &[f32], aspect: f32) -> (String, f32, f32) {
    let dmin = series.iter().cloned().fold(f32::INFINITY, f32::min);
    let dmax = series.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let (lo, hi) = if dmin.is_finite() && dmax.is_finite() {
        let pad = ((dmax - dmin) * 0.1).max(1.0);
        ((dmin - pad).max(0.0), dmax + pad)
    } else {
        (0.0, 1.0)
    };
    let x_scale = if aspect > 0.01 { aspect } else { 3.0 };
    (series_to_svg_path(series, lo, hi, x_scale), lo, hi)
}

/// 右锚定滚动窗口折线: 最新点固定在右缘(x=1000),越旧越靠左,超出 window 的点丢弃。
/// 未填满窗口时曲线从右向左生长, 填满后随新数据整体左移(自动滚动)。
/// (显示名, HID 键码) 表, 供物理键盘/触控键盘映射下拉。索引 0 = 不映射。
/// 键码为标准 HID Keyboard/Keypad usage, 与固件 HID_KeyCode 一致。
fn kbd_key_choices() -> Vec<(&'static str, u8)> {
    let mut v: Vec<(&'static str, u8)> = vec![("不映射", 0x00)];
    const LETTERS: [&str; 26] = [
        "A","B","C","D","E","F","G","H","I","J","K","L","M",
        "N","O","P","Q","R","S","T","U","V","W","X","Y","Z",
    ];
    for (i, name) in LETTERS.iter().enumerate() {
        v.push((name, 0x04 + i as u8));
    }
    const DIGITS: [&str; 10] = ["1","2","3","4","5","6","7","8","9","0"];
    for (i, name) in DIGITS.iter().enumerate() {
        v.push((name, 0x1E + i as u8));
    }
    v.push(("Enter", 0x28));
    v.push(("Esc", 0x29));
    v.push(("Backspace", 0x2A));
    v.push(("Tab", 0x2B));
    v.push(("Space", 0x2C));
    const FKEYS: [&str; 12] = ["F1","F2","F3","F4","F5","F6","F7","F8","F9","F10","F11","F12"];
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
    if modifier & 1 != 0 { s.push_str("Ctrl+"); }
    if modifier & 2 != 0 { s.push_str("Shift+"); }
    if modifier & 4 != 0 { s.push_str("Alt+"); }
    if modifier & 8 != 0 { s.push_str("Gui+"); }
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
        "\u{001b}" => return 0x29,               // Escape
        "\u{0008}" => return 0x2A,               // Backspace
        "\u{0009}" => return 0x2B,               // Tab
        " " => return 0x2C,                       // Space
        "\u{f700}" => return 0x52,               // Up
        "\u{f701}" => return 0x51,               // Down
        "\u{f702}" => return 0x50,               // Left
        "\u{f703}" => return 0x4F,               // Right
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
        });
    }
    out
}

fn build_param_rows(params: &[(u8, u32)]) -> Vec<ParamRow> {
    params
        .iter()
        // 单通道精调只暴露【逐通道】参数(阈值/迟滞/消抖/低基线复位/模态 IDAC 0x01-0x06,0x09)。
        // 全局/统一生效的硬件采样参数(分辨率 0x07、SnsClk 分频 0x08、时钟源 0x0A、IDAC 增幅 0x0B)
        // 会一改改全部 36 通道, 只应在“触控全局调整”页编辑 —— 从单通道页移除, 避免意外全局改动。
        .filter(|&&(param_id, _)| matches!(param_id, 0x01..=0x06 | 0x09))
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
