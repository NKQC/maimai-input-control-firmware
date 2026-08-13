//! mai2control-ui 库：复用协议、I/O、应用状态模块
//!
//! 本库暴露的公共模块可被 main.rs 与 src/bin/* 自测程序复用,
//! 避免代码重复。Slint UI 相关只在 main.rs 中,不进库。

/// JIT 算法变量的中文别名/简介（纯 UI 侧资产，不进固件）。
pub mod algo_i18n;
pub mod app_state;
pub mod comport;
pub mod elevation;
pub mod io;
pub mod logging;
pub mod proto;
/// HID 触控点位页的主屏幕截图(Win32 GDI BitBlt/GetDIBits)。
pub mod screen_capture;
pub mod settings_io;
pub mod touch_geometry;
pub mod ui_config;
pub mod vcam;
