//! 设备配置加载、保存和恢复默认回调。

use super::super::*;

pub(crate) fn register_config_callbacks(ui: &AppWindow, controller: &Rc<RefCell<AppController>>) {
    // 配置页
    let ctrl_clone = controller.clone();
    ui.on_cfg_load(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.request_config_all();
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_save(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.save_config();
    });

    let ctrl_clone = controller.clone();
    ui.on_cfg_reset(move || {
        let mut ctrl = ctrl_clone.borrow_mut();
        let _ = ctrl.reset_defaults();
    });
}
