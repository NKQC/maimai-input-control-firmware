{
    // 全局 CSD 设置回填(version 门控, 避免覆盖用户编辑)
    let global_tune_page_visible = ctrl.state() == ConnState::Connected
        && ui.get_current_view() == 1
        && ui.get_settings_tab() == SETTINGS_PAGE_GLOBAL_TUNE;
    if ctrl.globals_version() != last_globals_version
        || global_tune_page_visible && !last_global_tune_visible
    {
        last_globals_version = ctrl.globals_version();
        if let Some(v) = ctrl.global(1) { ui.set_g_inactive_sns(v as i32); }
        if let Some(v) = ctrl.global(2) { ui.set_g_idac_gain(v as i32); }
        if let Some(v) = ctrl.global(7) { ui.set_g_idac_sense_config(v as i32); }
        if let Some(v) = ctrl.global(8) { ui.set_g_auto_calibrate(v as i32); }
        if let Some(v) = ctrl.global(3) { ui.set_g_idac_min(v as i32); }
        if let Some(v) = ctrl.global(4) { ui.set_g_raw_target(v as i32); }
        if let Some(v) = ctrl.global(5) { ui.set_g_mfs_div_f1(v as i32); }
        if let Some(v) = ctrl.global(6) { ui.set_g_mfs_div_f2(v as i32); }
        let global_text = |id: u8| ctrl.global(id).map(|v| v.to_string()).unwrap_or_else(|| "未读取".to_string());
        ui.set_global_summary_text(format!(
            "inactive_sns={} · IDAC gain={} · MFS F1={} / F2={}",
            global_text(1), global_text(2), global_text(5), global_text(6)
        ).into());
    }

    // 校准频率偏好回填(草稿优先, 故滑条拖动后立即反映草稿值; 缺省 4)。
    ui.set_calib_pref(match ctrl.config_get("calib.pref").map(|e| e.value) {
        Some(CfgValue::U8(v)) => (v as i32).clamp(1, 7),
        _ => 4,
    });
}
