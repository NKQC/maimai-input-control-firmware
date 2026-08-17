//! 全通道批量选择、参数应用和遥测源回调。

use super::super::*;

pub(crate) fn register_batch_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
) -> Rc<RefCell<Vec<PendingCfgWrite>>> {
    // 全通道页"批量应用": 勾选态与应用动作全部落在 AppController, UI 只转发事件。
    // 应用走 set_param(草稿), 与手工编辑同路径, 由"保存到设备"统一下发。
    let ctrl_clone = controller.clone();
    ui.on_batch_toggle_channel(move |ch| {
        if !(0..36).contains(&ch) {
            return;
        }
        ctrl_clone.borrow_mut().batch_toggle_channel(ch as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_toggle_param(move |param_id| {
        if !(0x01..=0x0B).contains(&param_id) {
            return;
        }
        ctrl_clone.borrow_mut().batch_toggle_param(param_id as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_channels_all(move || {
        ctrl_clone.borrow_mut().batch_channels_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_channels_invert(move || {
        ctrl_clone.borrow_mut().batch_channels_invert();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_params_all(move || {
        ctrl_clone.borrow_mut().batch_params_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_params_invert(move || {
        ctrl_clone.borrow_mut().batch_params_invert();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_clear(move || {
        ctrl_clone.borrow_mut().batch_clear();
    });

    // 批量启用/禁用: 复用 PARAM_ENABLED(0x0C) 的草稿路径, 与逐通道开关同一条下发链。
    let ctrl_clone = controller.clone();
    ui.on_batch_enable_channels(move || {
        let _ = ctrl_clone.borrow_mut().batch_set_enabled(true);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_disable_channels(move || {
        let _ = ctrl_clone.borrow_mut().batch_set_enabled(false);
    });

    // 抽屉收起: 真正清掉 Rust 侧的本次会话(勾选/待写值/源通道), 不是只把面板藏起来 ——
    // 勾选态活在 AppController 里, 只藏面板的话"应用到已选通道"仍会照着上一批勾选执行。
    let ctrl_clone = controller.clone();
    ui.on_batch_drawer_closed(move || {
        ctrl_clone.borrow_mut().batch_drawer_closed();
    });

    // 全通道页的采集开关(真停/开遥测流), 与曲线页的"暂停画面"分属两套语义。
    let ctrl_clone = controller.clone();
    ui.on_telem_source_start(move || {
        ctrl_clone.borrow_mut().telem_user_start();
    });

    let ctrl_clone = controller.clone();
    ui.on_telem_source_stop(move || {
        ctrl_clone.borrow_mut().telem_user_stop();
    });

    // 单通道精调页的通道启用开关(草稿路径, 由"保存到设备"下发)。
    let ctrl_clone = controller.clone();
    let ui_weak_ch_en = ui.as_weak();
    ui.on_curve_ch_enable_set(move |on| {
        let Some(ui) = ui_weak_ch_en.upgrade() else {
            return;
        };
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let _ = ctrl_clone.borrow_mut().set_ch_enabled(ch, on);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_set_source(move |ch| {
        if !(0..36).contains(&ch) {
            return;
        }
        ctrl_clone.borrow_mut().batch_set_source(ch as u8);
    });

    // 批量面板里逐项手改的"待写值"。★围栏不在这里判★: 用户把数字从一个值改到另一个值的中间态
    // 完全可能越界(退格删到只剩一位), 这里提前拒绝会让人根本改不动; 合法性统一在"应用"时由
    // set_param 的唯一围栏(proto::param_value_legal)判定并写日志告知。
    let ctrl_clone = controller.clone();
    ui.on_batch_param_value_set(move |param_id, value| {
        if !(0x01..=0x0B).contains(&param_id) || value < 0 {
            return;
        }
        ctrl_clone
            .borrow_mut()
            .batch_set_value(param_id as u8, value as u32);
    });

    // ---- 逐通道算法配置(cfg_ch)的批量项 ----
    // 与上面的硬件参数走**同一套**语义: 勾选/手改值都只落在 AppController 的本次批量会话里,
    // 真正写入是在 batch_apply_from 里对每个已选目标通道各写一份 set_algo_cfg_ch 草稿。
    let ctrl_clone = controller.clone();
    ui.on_batch_toggle_algo_ch(move |idx| {
        if !(0..mai2control_ui::proto::algo::ALGO_CFG_CH_SLOTS as i32).contains(&idx) {
            return;
        }
        ctrl_clone.borrow_mut().batch_toggle_algo_ch(idx as u8);
    });

    // 值域 0..255 是协议事实(设备 cfg_ch 就是 u8 数组), 不是这里猜的围栏; 越界直接忽略而不钳制 ——
    // 钳制会把用户输错的数悄悄变成 255 写下去, 而忽略至少让面板停在原值。
    let ctrl_clone = controller.clone();
    ui.on_batch_algo_ch_value_set(move |idx, value| {
        if !(0..mai2control_ui::proto::algo::ALGO_CFG_CH_SLOTS as i32).contains(&idx)
            || !(0..=255).contains(&value)
        {
            return;
        }
        ctrl_clone
            .borrow_mut()
            .batch_algo_ch_value_set(idx as u8, value as u8);
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_algo_ch_all(move || {
        ctrl_clone.borrow_mut().batch_algo_ch_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_algo_ch_invert(move || {
        ctrl_clone.borrow_mut().batch_algo_ch_invert();
    });

    let ctrl_clone = controller.clone();
    ui.on_batch_apply(move |src| {
        if !(0..36).contains(&src) {
            return;
        }
        let _ = ctrl_clone.borrow_mut().batch_apply_from(src as u8);
    });

    // JSON 导出：范围先由 Slint 弹窗确认，再使用系统原生保存对话框，失败只写日志不阻塞 UI。
    let ctrl_clone = controller.clone();
    ui.on_settings_export(
        move |config, channel_params, globals, algo, keyboard, zones| {
            let selected = mai2control_ui::settings_io::GroupSelection {
                config,
                channel_params,
                globals,
                algo,
                keyboard,
                zones,
            };
            match mai2control_ui::settings_io::choose_settings_path(true) {
                Ok(Some(path)) => match mai2control_ui::settings_io::export_settings(
                    &ctrl_clone.borrow(),
                    selected,
                ) {
                    Ok(text) => match std::fs::write(&path, text) {
                        Ok(()) => log::info!("设置 JSON 已导出: {}", path.display()),
                        Err(e) => {
                            log::warn!("设置 JSON 导出失败，无法写入 {}: {}", path.display(), e)
                        }
                    },
                    Err(e) => log::warn!("设置 JSON 导出失败: {}", e),
                },
                Ok(None) => log::info!("设置 JSON 导出已取消"),
                Err(e) => log::warn!("无法打开设置 JSON 保存对话框: {}", e),
            }
        },
    );

    // JSON 导入：只写 UI 草稿并置脏，不下发、不写 flash、不回读；结果(覆盖/跳过/未保存态)写进日志页。
    let ctrl_clone = controller.clone();
    ui.on_settings_import(
        move |config, channel_params, globals, algo, keyboard, zones| {
            let selected = mai2control_ui::settings_io::GroupSelection {
                config,
                channel_params,
                globals,
                algo,
                keyboard,
                zones,
            };
            match mai2control_ui::settings_io::choose_settings_path(false) {
                Ok(Some(path)) => match std::fs::read_to_string(&path) {
                    Ok(text) => {
                        let mut ctrl = ctrl_clone.borrow_mut();
                        match mai2control_ui::settings_io::import_settings(
                            &mut ctrl, &text, selected,
                        ) {
                            Ok(summary) => {
                                let report = summary.report_text();
                                // 跳过项必须显眼: 走 Warn 等级, 默认过滤下也能看到, 不静默丢弃。
                                if summary.skipped.is_empty() {
                                    ctrl.push_log(format!("{}（{}）", report, path.display()));
                                } else {
                                    ctrl.push_log_warn(format!("{}（{}）", report, path.display()));
                                }
                                log::info!("设置 JSON 已导入: {}", path.display());
                            }
                            Err(e) => {
                                ctrl.push_log_warn(format!(
                                    "设置 JSON 导入失败, 草稿未改动: {}",
                                    e
                                ));
                                log::warn!("设置 JSON 导入失败 {}: {}", path.display(), e);
                            }
                        }
                    }
                    Err(e) => log::warn!("设置 JSON 导入失败，无法读取 {}: {}", path.display(), e),
                },
                Ok(None) => log::info!("设置 JSON 导入已取消"),
                Err(e) => log::warn!("无法打开设置 JSON 导入对话框: {}", e),
            }
        },
    );

    // 撤销全部未保存草稿(CSD 安全操作 / 配置页): 恢复到设备当前运行态。
    let ctrl_clone = controller.clone();
    ui.on_discard_draft(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.discard_draft();
    });

    // ★配置写入必须能在"控制器已被借出"时排队★
    // 这些回调可以由 Slint 在**模型替换过程中**同步触发: 主循环持有 `controller.borrow_mut()`
    // 的同时调用 `ui.set_config_rows(...)`, 旧行控件被销毁 → 有焦点的 SpinBox 触发
    // `changed has-focus` → `committed` → 本回调 → 再 `borrow_mut()` ⇒ RefCell 双重可变借用直接
    // panic(表现就是"改完配置 UI 直接闪退", 且 panic 走 stderr 不进日志文件, 日志只停在
    // 「配置分组直方图」那一行)。
    // 处理原则: 借不到就**排队**而不是丢弃 —— 丢弃等于用户那次修改静默消失, 比崩溃更难发现。
    let pending_cfg: Rc<RefCell<Vec<PendingCfgWrite>>> = Rc::new(RefCell::new(Vec::new()));

    let ctrl_clone = controller.clone();
    let pending = pending_cfg.clone();
    ui.on_cfg_set_bool(move |key, value| {
        _apply_or_queue_cfg(
            &ctrl_clone,
            &pending,
            PendingCfgWrite::Bool(key.to_string(), value),
        );
    });

    let ctrl_clone = controller.clone();
    let pending = pending_cfg.clone();
    ui.on_cfg_set_number(move |key, value| {
        _apply_or_queue_cfg(
            &ctrl_clone,
            &pending,
            PendingCfgWrite::Number(key.to_string(), value as f64),
        );
    });

    // 数字配置的十六进制口径输入: LineEdit 文本(如 "0x1E") → 解析并按原类型写草稿。
    let ctrl_clone = controller.clone();
    let pending = pending_cfg.clone();
    ui.on_cfg_set_number_hex(move |key, value| {
        _apply_or_queue_cfg(
            &ctrl_clone,
            &pending,
            PendingCfgWrite::Hex(key.to_string(), value.to_string()),
        );
    });

    let ctrl_clone = controller.clone();
    let pending = pending_cfg.clone();
    ui.on_cfg_set_enum(move |key, index| {
        _apply_or_queue_cfg(
            &ctrl_clone,
            &pending,
            PendingCfgWrite::Enum(key.to_string(), index),
        );
    });

    let ctrl_clone = controller.clone();
    let pending = pending_cfg.clone();
    ui.on_cfg_set_string(move |key, value| {
        _apply_or_queue_cfg(
            &ctrl_clone,
            &pending,
            PendingCfgWrite::Str(key.to_string(), value.to_string()),
        );
    });

    pending_cfg
}
