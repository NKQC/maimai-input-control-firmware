use super::super::*;

pub(crate) struct LogCallbackState {
    pub(crate) log_dirty: Rc<Cell<bool>>,
}

pub(crate) fn register_log_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    ui_weak: &slint::Weak<AppWindow>,
) -> LogCallbackState {
    // ---------------- 日志页 ----------------
    // 过滤等级: 写入 app_state 作真相源, 并强制下一帧重建行模型(把 last_log_ver 打脏)。
    ui.set_log_filter(controller.borrow().log_filter() as i32);
    mai2control_ui::logging::hub().set_level(controller.borrow().log_filter());
    ui.set_log_file_path(mai2control_ui::logging::hub().file_path_text().into());
    let log_dirty = Rc::new(Cell::new(true));

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
    let ui_clear = ui_weak.clone();
    ui.on_clear_log(move || {
        ctrl_clear.borrow_mut().clear_log();
        if let Some(ui) = ui_clear.upgrade() {
            ui.set_log_page_start(0);
        }
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

    LogCallbackState { log_dirty }
}
