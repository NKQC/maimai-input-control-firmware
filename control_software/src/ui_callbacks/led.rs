//! mai2light/LED 协议页的 UI 回调。

use super::super::*;

pub(crate) fn register_led_callbacks(ui: &AppWindow, controller: &Rc<RefCell<AppController>>) {
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
}
