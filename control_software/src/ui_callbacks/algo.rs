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
    // C 源容量条: 容量一次性回填, 占用随编辑器内容刷新(去注释后的字节数 = 编译器有效内容)。
    ui.set_algo_c_capacity(AppController::algo_src_capacity() as i32);
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

    let ctrl_clone = controller.clone();
    ui.on_algo_setting_edited(move |idx, value| {
        if idx < 0 || value < 0 || value > 255 {
            return;
        }
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_algo_cfg(idx as u8, value as u8);
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
