//! 触控分区到多键组合映射的 UI 回调。

use super::super::*;

pub(crate) fn register_combo_callbacks(ui: &AppWindow, controller: &Rc<RefCell<AppController>>) {
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
    ui.on_combo_hid_submitted(move |text| match parse_hid_usage(text.as_str()) {
        Ok(code) => ctrl_clone.borrow_mut().kbd_combo_capture_key(code, 0),
        Err(error) => ctrl_clone.borrow_mut().push_log(error),
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

    let ctrl_clone = controller.clone();
    ui.on_combo_key_hid_submitted(move |index, text| {
        if index < 0 {
            return;
        }
        match parse_hid_usage(text.as_str()) {
            Ok(code) => ctrl_clone
                .borrow_mut()
                .kbd_combo_capture_key_at(index as usize, code, 0),
            Err(error) => ctrl_clone.borrow_mut().push_log(error),
        }
    });
}
