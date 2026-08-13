//! 设备连接、重启、批量校准及设备列表回调。

use super::super::*;

pub(crate) fn register_connection_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    ui_weak: &slint::Weak<AppWindow>,
    auto_reconnect: &Rc<Cell<bool>>,
) {
    // 刷新按钮只执行本次手动扫描，不改变手动断开状态。
    let ctrl_clone = controller.clone();
    let ui_refresh = ui_weak.clone();
    ui.on_refresh(move || {
        let ui = ui_refresh.upgrade().unwrap();
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.refresh_devices();
        let labels: Vec<slint::SharedString> =
            ctrl.device_labels().into_iter().map(|s| s.into()).collect();
        ui.set_device_labels(slint::ModelRc::new(slint::VecModel::from(labels)));
    });

    // 连接
    let ctrl_clone = controller.clone();
    let ui_conn = ui_weak.clone();
    let reconnect_on_connect = auto_reconnect.clone();
    ui.on_connect_clicked(move || {
        let ui = ui_conn.upgrade().unwrap();
        let index = ui.get_selected_device() as usize;
        let mut ctrl = ctrl_clone.borrow_mut();
        if ctrl.connect(index).is_ok() {
            reconnect_on_connect.set(true);
        }
    });

    // 断开
    let ctrl_clone = controller.clone();
    let reconnect_on_disconnect = auto_reconnect.clone();
    ui.on_disconnect_clicked(move || {
        reconnect_on_disconnect.set(false);
        let mut ctrl = ctrl_clone.borrow_mut();
        ctrl.disconnect();
    });

    // 重启设备(新增)
    let ctrl_clone = controller.clone();
    ui.on_reboot_device(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reboot();
    });

    // 进入烧录模式(新增)
    let ctrl_clone = controller.clone();
    ui.on_reboot_bootloader(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reboot_bootloader();
    });

    // 重启 PSoC(使需重启生效的改动生效)
    let ctrl_clone = controller.clone();
    ui.on_reboot_psoc(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reboot_psoc();
    });

    // 全通道频率自适应: ★改走 host 侧逐通道串行队列★(不再是 0xFF 让固件自己 for 36 遍)。
    // 队列会先把每通道的 IDAC 增益档写入并回读确认, 再逐通道下探; 全程可取消、有进度、
    // 失败能指名到通道。见 app_state::ch_ops。
    let ctrl_clone = controller.clone();
    ui.on_auto_tune(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.ch_batch_start(ChBatchKind::AutoTune) {
            ctrl.push_log_warn(format!("逐通道频率自适应未能开始: {}", e));
        }
    });

    // 批量队列取消(三种全通道操作共用一条队列, 故只有一个取消入口)。
    let ctrl_clone = controller.clone();
    ui.on_batch_op_cancel(move || {
        ctrl_clone.borrow_mut().ch_batch_cancel();
    });

    // "Cp 辅助"开关: 逐通道自适应按实测 Cp 推每通道的增益档/频率偏好档。
    let ctrl_clone = controller.clone();
    ui.on_set_cp_assist(move |on| {
        ctrl_clone.borrow_mut().set_cp_assist(on);
    });

    // PSoC 救砖: 经 SWD 强制重刷 PSoC 并重新下发算法/CSD(UI 已做两段式二次确认)。
    let ctrl_clone = controller.clone();
    ui.on_psoc_rescue(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.psoc_rescue();
    });

    // 校准频率偏好滑条(1..7): 只写草稿(与其他设置同规范), 但下一次自适应即读草稿生效。
    let ctrl_clone = controller.clone();
    ui.on_calib_pref_set(move |v| {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.set_config_number("calib.pref", v.clamp(1, 7) as f64);
    });

    // 全通道校准 / 全通道基线复位: 同样走 host 侧逐通道串行队列(每通道一条短命令, 可取消)。
    // ★固件那条全通道路径不删★ —— "恢复默认"等设备内部触发仍要用它, 只是 UI 不再从这里进。
    let ctrl_clone = controller.clone();
    ui.on_global_calibrate(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.ch_batch_start(ChBatchKind::Calibrate) {
            ctrl.push_log_warn(format!("逐通道校准未能开始: {}", e));
        }
    });

    let ctrl_clone = controller.clone();
    ui.on_global_baseline_reset(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        if let Err(e) = ctrl.ch_batch_start(ChBatchKind::BaselineReset) {
            ctrl.push_log_warn(format!("逐通道基线复位未能开始: {}", e));
        }
    });

    // 本通道频率自适应(只下探当前选中通道, 其余通道分频不动)。
    let ctrl_clone = controller.clone();
    let ui_at = ui_weak.clone();
    ui.on_curve_auto_tune(move || {
        let ui = ui_at.upgrade().unwrap();
        let ch = ui.get_sel_channel().clamp(0, 35) as u8;
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.auto_tune(ch);
    });
}
