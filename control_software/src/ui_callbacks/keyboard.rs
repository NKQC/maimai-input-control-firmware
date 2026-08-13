//! 物理键盘/触控键盘映射与逻辑分析仪视图回调。

use super::super::*;

pub(crate) fn register_keyboard_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    ui_weak: &slint::Weak<AppWindow>,
) -> Rc<Cell<usize>> {
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
    ui.on_kbd_hid_phys(move |idx, text| match parse_hid_usage(text.as_str()) {
        Ok(code) => {
            let _ = ctrl_clone.borrow_mut().kbd_set_map(idx as u8, code, 0);
        }
        Err(error) => ctrl_clone.borrow_mut().push_log(error),
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

    la_window_idx
}
