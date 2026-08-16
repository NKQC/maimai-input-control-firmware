//! UI 运行时偏好持久化。
//!
//! 本模块只负责启动偏好持久化；虚拟摄像头 HID 设备树同步归属
//! `ui_callbacks::virtual_camera`，以便设备列表、回调和状态所有权保持同域。

use super::*;

pub(crate) fn persist_ui_settings(
    ui: &AppWindow,
    report_show: &[bool; 4],
    cfg: &mut mai2control_ui::ui_config::UiConfig,
) {
    use mai2control_ui::ui_config::keys as k;
    cfg.set_i32(k::LOG_FILTER, ui.get_log_filter());
    cfg.set_bool(k::LOG_AUTO_SCROLL, ui.get_log_auto_scroll());
    cfg.set_bool(k::VCAM_ENABLED, ui.get_vcam_enabled());
    cfg.set_i32(k::VCAM_SUBMIT_SECS, ui.get_vcam_submit_secs());
    cfg.set_i32(k::VCAM_DISPLAY_SECS, ui.get_vcam_display_secs());
    cfg.set_bool(k::VCAM_MIRROR_X, ui.get_vcam_mirror_x());
    cfg.set_i32(k::VCAM_FRAME_W, ui.get_vcam_frame_w());
    cfg.set_i32(k::VCAM_FRAME_H, ui.get_vcam_frame_h());
    cfg.set_i32(k::VCAM_QR_FILL_PCT, ui.get_vcam_qr_fill_pct());
    cfg.set_bool(k::DIAG_EXPANDED, ui.get_diag_expanded());
    cfg.set_bool(k::CURVE_PARAMS_EXPANDED, ui.get_curve_params_expanded());
    cfg.set_bool(k::CURVE_ALGO_CFG_EXPANDED, ui.get_curve_algo_cfg_expanded());
    cfg.set_bool(k::CURVE_SERIES_EXPANDED, ui.get_curve_series_expanded());
    cfg.set_bool(k::CURVE_SPECTRUM_EXPANDED, ui.get_curve_spectrum_expanded());
    cfg.set_bool(k::CHANNEL_SHOW_DISABLED, ui.get_channel_show_disabled());
    cfg.set_bool(k::PHYS_LIVE_EXPANDED, ui.get_phys_live_expanded());
    cfg.set_bool(k::PHYS_KEYS_EXPANDED, ui.get_phys_keys_expanded());
    cfg.set_bool(k::PHYS_LA_EXPANDED, ui.get_phys_la_expanded());
    cfg.set_bool(k::MAI2_PANEL_EXPANDED, ui.get_mai2_panel_expanded());
    cfg.set_bool(k::LIGHT_PANEL_EXPANDED, ui.get_light_panel_expanded());
    cfg.set_bool(k::LATENCY_MEASURE, ui.get_measure_latency());
    cfg.set_i32(k::CURRENT_VIEW, ui.get_current_view());
    cfg.set_i32(k::SETTINGS_TAB, ui.get_settings_tab());
    cfg.set_i32(k::SEL_CHANNEL, ui.get_sel_channel());
    cfg.set_bool(k::CURVE_SHOW_RAW, ui.get_show_raw());
    cfg.set_bool(k::CURVE_SHOW_BSLN, ui.get_show_bsln());
    cfg.set_bool(k::CURVE_SHOW_DIFF, ui.get_show_diff());
    cfg.set_bool(k::CURVE_SHOW_ACTIVE, ui.get_show_active());
    cfg.set_bool(k::CURVE_ALGO_OVERLAY, ui.get_show_algo_overlay());
    for (idx, on) in report_show.iter().enumerate() {
        cfg.set_bool(&report_show_key(idx), *on);
    }
}
