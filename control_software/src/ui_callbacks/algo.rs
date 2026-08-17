//! 算法页 JIT 触控算法、C→ASM 编译器与全局设置回调。

use super::super::*;

pub(crate) struct AlgoCallbackState {
    pub(crate) algo_default_src: &'static str,
    pub(crate) algo_busy: Rc<Cell<bool>>,
    pub(crate) algo_job: Rc<RefCell<Option<AlgoCompileJob>>>,
    pub(crate) report_show: Rc<RefCell<[bool; 4usize]>>,
    pub(crate) algo_report_lines_model: Rc<slint::VecModel<AlgoReportLine>>,
}

pub(crate) fn register_algo_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    ui_weak: &slint::Weak<AppWindow>,
) -> AlgoCallbackState {
    // ---- 算法页 (JIT 触控算法 + C→ASM 编译器 + 全局设置) ----
    // 默认 C 源模板: 最小合法算法(沿用基础激活), 用户可改为自定义高动态逻辑。
    let algo_default_src = "#include <stddef.h>\n#include \"psoc_algo_abi.h\"\n\n// 入口: 每通道调用一次, 读写 io 固定字段。\n// 禁: libc / '/' '%' / 64位。辅助请 static inline。\nvoid algo(algo_io_t* io)\n{\n    // 示例: 直接沿用中间件基础激活判定。\n    // io->diff/baseline/finger_th/now_ms/rom 等可用于自定义高动态逻辑。\n    io->out_active = (io->base_active != 0u) ? 1u : 0u;\n}\n";
    ui.set_algo_c_source(algo_default_src.into());
    ui.set_algo_line_numbers(line_numbers_for(algo_default_src).into());
    // C 源容量条: 这里只是**开机首帧**的兜底回填(此刻还没连上设备, 拿不到设备容量);
    // 真值由 tick 在收到 ALGO_GET_INFO 后覆盖(见 tick/algorithm.rs 的容量回填块)。
    ui.set_algo_c_capacity(controller.borrow().algo_src_capacity() as i32);
    ui.set_algo_c_bytes(AppController::algo_src_used(algo_default_src) as i32);

    let ctrl_clone = controller.clone();
    ui.on_algo_refresh(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        // ★显式回读★: 用户点"读取信息"的意图就是把设备里真正存的 C 源与 ASM 取回来看,
        // 故本轮回读必须覆盖编辑器(自动回读仍然保留草稿, 见 take_algo_explicit_readback)。
        ctrl.mark_algo_explicit_readback();
        // ★一拍只发一条只读命令★ 设备 vendor 响应单槽; 原先一次点击连发 5 条(info/rom/src/code/
        // global), 必然互相挤掉响应 —— 其中 C 源是多片顺序回读, 丢一片就整轮作废, 且失败的 seq
        // 会滞留在 pending_registry 里把后续 C 源回读永久挡住。
        // 这里只发 C 源; 机器码在 C 源回读完成后顺带发起(见 tick/algorithm.rs), info/rom/global
        // 本就由连接探针与页面轮询各自维护, 不必在此重复拉取。
        if let Err(error) = ctrl.request_algo_src() {
            ctrl.push_log_warn(format!("回读算法 C 源失败: {}", error));
        }
    });

    let ctrl_clone = controller.clone();
    ui.on_algo_reset_default(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.algo_reset_default();
        let _ = ctrl.send_algo_src(&AppController::strip_c_comments(ALGO_V31_TEMPLATE));
        let _ = ctrl.algo_get_rom();
        let _ = ctrl.request_algo_src();
    });

    let ui_tpl = ui_weak.clone();
    ui.on_algo_load_template(move |idx| {
        // ★既有两项的下标不能动★ 0/1 是用户已经习惯的"v3.1 HDR"/"LED 演示", 且 config.cfg 里没有
        // 记住这个下拉的选择 —— 但换掉顺序会让任何按截图/文档操作的人载入错误的模板。新项只能追加。
        let tpl = match idx {
            1 => ALGO_LED_DEMO_TEMPLATE,
            2 => ALGO_V4_TEMPLATE,
            _ => ALGO_V31_TEMPLATE,
        };
        let ui = ui_tpl.upgrade().unwrap();
        ui.set_algo_c_source(tpl.into());
        ui.set_algo_line_numbers(line_numbers_for(tpl).into());
        ui.set_algo_c_bytes(AppController::algo_src_used(tpl) as i32);
    });

    let ui_edit = ui_weak.clone();
    ui.on_algo_source_edited(move |text| {
        let ui = ui_edit.upgrade().unwrap();
        ui.set_algo_line_numbers(line_numbers_for(text.as_str()).into());
        ui.set_algo_c_bytes(AppController::algo_src_used(text.as_str()) as i32);
    });

    AlgoCallbackState {
        algo_default_src,
        algo_busy: Rc::new(Cell::new(false)),
        algo_job: Rc::new(RefCell::new(None)),
        report_show: Rc::new(RefCell::new([true; 4usize])),
        algo_report_lines_model: Rc::new(slint::VecModel::from(Vec::new())),
    }
}

pub(crate) fn register_algo_late_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    ui_weak: &slint::Weak<AppWindow>,
    state: &AlgoCallbackState,
) {
    let algo_busy = state.algo_busy.clone();
    let algo_job = state.algo_job.clone();

    // 算法编译一律丢后台线程(见 AlgoCompileJob/spawn_algo_compile): UI 线程只置 busy 并在
    // 16ms tick 里取回产物写入 AppController, 编译期间界面照常重绘与响应点击。
    let ui_algo = ui_weak.clone();
    let algo_busy_compile = algo_busy.clone();
    let algo_job_compile = algo_job.clone();
    let ctrl_clone = controller.clone();
    ui.on_algo_compile(move |src| {
        let Some(ui) = ui_algo.upgrade() else { return };
        if algo_busy_compile.get() {
            if algo_job_compile.borrow().is_some() {
                ui.set_algo_status("编译进行中".into());
                return;
            }
            algo_busy_compile.set(false);
            ui.set_algo_busy(false);
        }
        // ★容量快照必须在 spawn 之前取★ 后台线程碰不到 AppController(Rc<RefCell<_>> 不是 Send),
        // 而闸门又不能退回硬编码常量。快照顺带保证报错文案与实际判据用的是同一口径。
        let caps = ctrl_clone.borrow().algo_caps_snapshot();
        let used = AppController::algo_src_used(src.as_str());
        ui.set_algo_c_bytes(used as i32);
        if used > caps.src {
            ui.set_algo_status(
                format!(
                    "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 已阻止编译",
                    used, caps.src
                )
                .into(),
            );
            return;
        }
        algo_busy_compile.set(true);
        ui.set_algo_busy(true);
        ui.set_algo_phase("编译中…".into());
        ui.set_algo_status("编译中…(后台工具链, 界面可继续操作)".into());
        spawn_algo_compile(&algo_job_compile, src.to_string(), false, caps);
    });

    let ctrl_clone = controller.clone();
    let ui_algo = ui_weak.clone();
    let algo_busy_upload = algo_busy.clone();
    let algo_job_upload = algo_job.clone();
    ui.on_algo_upload(move || {
        let Some(ui) = ui_algo.upgrade() else { return };
        if algo_busy_upload.get() {
            if algo_job_upload.borrow().is_some() {
                ui.set_algo_status("编译进行中".into());
                return;
            }
            algo_busy_upload.set(false);
            ui.set_algo_busy(false);
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        match ctrl.upload_compiled() {
            Ok(()) => ui.set_algo_status(ctrl.algo_upload_status().into()),
            Err(e) => ui.set_algo_status(format!("上传失败: {}", e).into()),
        }
    });

    let ui_algo = ui_weak.clone();
    let algo_busy_build = algo_busy.clone();
    let algo_job_build = algo_job.clone();
    let ctrl_clone = controller.clone();
    ui.on_algo_build_upload(move |src| {
        let Some(ui) = ui_algo.upgrade() else { return };
        if algo_busy_build.get() {
            if algo_job_build.borrow().is_some() {
                ui.set_algo_status("编译进行中".into());
                return;
            }
            algo_busy_build.set(false);
            ui.set_algo_busy(false);
        }
        let caps = ctrl_clone.borrow().algo_caps_snapshot();
        let used = AppController::algo_src_used(src.as_str());
        ui.set_algo_c_bytes(used as i32);
        if used > caps.src {
            ui.set_algo_status(
                format!(
                    "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 已阻止编译并上传",
                    used, caps.src
                )
                .into(),
            );
            return;
        }
        algo_busy_build.set(true);
        ui.set_algo_busy(true);
        ui.set_algo_phase("编译中…".into());
        ui.set_algo_status("编译中…(后台工具链, 界面可继续操作)".into());
        spawn_algo_compile(&algo_job_build, src.to_string(), true, caps);
    });

    let ctrl_clone = controller.clone();
    ui.on_algo_setting_edited(move |idx, value| {
        if idx < 0 || value < 0 || value > 255 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_algo_cfg(idx as u8, value as u8);
    });

    // 逐通道算法配置(cfg_ch)的单通道编辑。★通道号不随参数传★: 由这里读 sel_channel(与
    // curve_ch_enable_set 同一手法), 免得界面与 Rust 各持一个"当前通道"而漂移 —— 那种漂移的
    // 表现是"在 CH7 上改的值落到了 CH0", 而两边看起来都对。
    let ctrl_clone = controller.clone();
    let ui_weak_cfg_ch = ui_weak.clone();
    ui.on_algo_setting_ch_edited(move |idx, value| {
        if idx < 0 || value < 0 || value > 255 {
            return;
        }
        let Some(ui) = ui_weak_cfg_ch.upgrade() else {
            return;
        };
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_algo_cfg_ch(ch, idx as u8, value as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_algo_metadata_edited(move |idx, kind, alias, description| {
        if idx < 0 || kind < 0 || kind > 1 {
            return;
        }
        let _ = ctrl_clone.borrow_mut().set_algo_metadata_override(
            kind as u8,
            idx as u8,
            alias.to_string(),
            description.to_string(),
        );
    });

    let ctrl_clone = controller.clone();
    ui.on_algo_global_set(move |gparam_id, value| {
        if gparam_id < 0 || value < 0 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.global_set(gparam_id as u8, value as u32);
    });
}
