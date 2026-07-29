//! mai2control-ui 库：复用协议、I/O、应用状态模块
//!
//! 本库暴露的公共模块可被 main.rs 与 src/bin/* 自测程序复用,
//! 避免代码重复。Slint UI 相关只在 main.rs 中,不进库。

pub mod app_state;
pub mod comport;
pub mod elevation;
pub mod io;
pub mod logging;
pub mod proto;
pub mod settings_io;
pub mod touch_geometry;
pub mod ui_config;
pub mod vcam;
