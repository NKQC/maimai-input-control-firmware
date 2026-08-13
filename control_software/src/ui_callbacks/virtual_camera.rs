//! 虚拟摄像头 UI 初始化、回调与设备选择。

use std::sync::Arc;

use super::*;

pub(crate) struct VirtualCameraCallbackState {
    pub(crate) vcam: Arc<VcamState>,
    pub(crate) frame_publisher: Rc<RefCell<Option<FramePublisher>>>,
    pub(crate) vcam_kbd_list: Rc<RefCell<Vec<vcam::keyboard::KeyboardDevice>>>,
}

impl VirtualCameraCallbackState {
    pub(crate) fn new(ui: &AppWindow) -> Self {
        // 虚拟扫码摄像头共享状态 + 初始 UI 值(与状态默认对齐: 提交阈值 2s, 显示 10s)。
        let vcam = VcamState::new();
        vcam.set_submit_timeout_ms(2000);
        vcam.set_display_ms(10_000);
        ui.set_vcam_submit_secs(2);
        ui.set_vcam_display_secs(10);
        ui.set_vcam_enabled(false);
        // ★启动时**不**创建共享队列★: 队列的语义是"有且仅有一个活生产者", 生产者租约随
        // `FramePublisher` 存活。启动即创建等于本进程一直霸占租约, 第二个实例(哪怕只是想开个界面
        // 看日志)会被拒, 而摄像头明明还没启用。故只在"启用摄像头"回调里创建, 禁用/卸载时立刻 drop。
        let frame_publisher: Rc<RefCell<Option<FramePublisher>>> = Rc::new(RefCell::new(None));
        ui.set_vcam_runtime_status(
            format!(
                "未运行 · 启用后创建共享队列 {}(独占生产者租约)",
                share::MAP_NAME
            )
            .into(),
        );
        let vcam_kbd_list: Rc<RefCell<Vec<vcam::keyboard::KeyboardDevice>>> =
            Rc::new(RefCell::new(Vec::new()));
        Self {
            vcam,
            frame_publisher,
            vcam_kbd_list,
        }
    }
}

pub(crate) fn initialize(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    state: &VirtualCameraCallbackState,
) {
    let install_status = vcam_backend::registration_status();
    log::info!("虚拟摄像头: 系统注册状态: {}", install_status);
    // 内核代码完整性实测结论只落日志一次: 它解释的是"为什么不自写内核过滤驱动"这个已定架构决策,
    // 对日常使用没有可行动信息, 摊在界面上纯属噪声; 但排查时它是决定性证据, 必须留痕。
    log::info!(
        "虚拟摄像头: 内核驱动可行性: {}",
        vcam::kernel_driver_feasibility()
    );
    ui.set_vcam_install_status(install_status.into());
    // 可选 HID 键盘列表(下拉索引 0 = 所有键盘, 之后按此 Vec 顺序对应)。
    // 启动即按持久化的设备路径恢复选择: 设备不在场时回落到"所有键盘", 不静默失效。
    let saved = controller.borrow().vcam_kbd_device();
    refresh_vcam_devices(ui, &state.vcam_kbd_list, &saved);
    apply_winusb_status(ui, winusb_status_texts(&saved));
}

/// WinUSB 改绑的三行状态文案(签名 / 绑定 / 内核驱动可行性)。
/// ★必须在工作线程上算★ 签名核验要起一个 PowerShell 读目录签名, 放 UI 线程会卡住事件循环。
pub(crate) fn winusb_status_texts(raw_path: &str) -> (String, String, String) {
    // ★界面给结论、日志给证据★ 完整读数(证书指纹、实例 ID、受信任存储位置、槽上限推理)一律走
    // `detail()` 落日志; 界面上只留一行结论, 否则"成没成"会被排查细节淹掉(用户明确反馈过啰嗦)。
    let signing = vcam::driver_pkg::signing_status();
    log::debug!("虚拟摄像头: WinUSB 签名状态 {}", signing.detail());
    let binding = if raw_path.trim().is_empty() {
        "未改绑（尚未选定扫码器）".to_string()
    } else {
        let status = vcam::driver_pkg::binding_status(raw_path);
        log::debug!("虚拟摄像头: WinUSB 绑定状态 {}", status.detail());
        status.short()
    };
    // 内核可行性 + 槽位压力只在**确实不利**时才上界面: 一切正常时它是纯噪声。
    let diag = match vcam::keyboard_slot_pressure() {
        Some((present, limit)) if present > limit => format!(
            "在位键盘节点 {} 个 > 键盘过滤驱动槽上限 {} 个：过滤驱动方案对本机不可靠，WinUSB 改绑不受此限",
            present, limit
        ),
        _ => String::new(),
    };
    (signing.short(), binding, diag)
}

fn apply_winusb_status(ui: &AppWindow, texts: (String, String, String)) {
    ui.set_vcam_signing_status(texts.0.into());
    ui.set_vcam_binding_status(texts.1.into());
    ui.set_vcam_kernel_diag(texts.2.into());
}

pub(crate) fn register_callbacks(
    ui: &AppWindow,
    controller: &Rc<RefCell<AppController>>,
    ui_weak: &slint::Weak<AppWindow>,
    state: &VirtualCameraCallbackState,
) {
    // 虚拟摄像头(与 OBS Virtual Camera 同实现: DirectShow 源过滤器)。
    // ★上位机侧没有任何 COM 对象要持有★: 消费端(游戏/Unity/OBS)在自己的进程里创建过滤器,
    // 本进程只负责把 QR 帧写进共享队列。开关只控制"采集 + 队列生产者租约"。
    let vcam_cb = state.vcam.clone();
    let publisher_cb = state.frame_publisher.clone();
    let ui_vcam = ui_weak.clone();
    ui.on_set_vcam_enabled(move |on| {
        let Some(ui) = ui_vcam.upgrade() else { return };
        if !on {
            // 停止顺序不能变：先截断采集，再释放队列生产者租约。
            vcam_cb.set_enabled(false);
            vcam::keyboard::stop();
            *publisher_cb.borrow_mut() = None;
            ui.set_vcam_enabled(false);
            ui.set_vcam_runtime_status(
                "未运行 · 摄像头仍在系统设备列表中，消费端会读到黑帧".into(),
            );
            return;
        }
        // 启动顺序：注册核验 → 队列 → 键盘采集。每个失败分支都回收已完成阶段。
        match vcam_backend::is_registered() {
            Ok(true) => {}
            Ok(false) => {
                log::warn!("虚拟摄像头: DirectShow 注册未完整核验；请先点击“安装(需管理员)”");
                vcam_cb.set_enabled(false);
                ui.set_vcam_enabled(false);
                ui.set_vcam_install_status(vcam_backend::registration_status().into());
                ui.set_vcam_runtime_status("未运行 · DirectShow 注册未完整安装".into());
                return;
            }
            Err(error) => {
                log::error!("虚拟摄像头: 注册状态核验失败: {}", error);
                vcam_cb.set_enabled(false);
                ui.set_vcam_enabled(false);
                ui.set_vcam_runtime_status("未运行 · 注册状态核验失败".into());
                return;
            }
        }
        match FramePublisher::create() {
            Ok(publisher) => *publisher_cb.borrow_mut() = Some(publisher),
            Err(error) => {
                log::error!("虚拟摄像头: 无法创建共享队列: {}", error);
                vcam_cb.set_enabled(false);
                ui.set_vcam_enabled(false);
                ui.set_vcam_runtime_status(format!("未运行 · {}", error).into());
                return;
            }
        }
        match vcam::keyboard::start(vcam_cb.clone()) {
            Ok(()) => {
                vcam_cb.set_enabled(true);
                ui.set_vcam_enabled(true);
                ui.set_vcam_runtime_status(vcam::keyboard::runtime_status().into());
            }
            Err(error) => {
                log::warn!("虚拟摄像头: 目标设备捕获未启动: {}", error);
                vcam_cb.set_enabled(false);
                *publisher_cb.borrow_mut() = None;
                ui.set_vcam_enabled(false);
                ui.set_vcam_runtime_status(format!("未运行 · 目标设备过滤降级: {}", error).into());
            }
        }
    });
    if ui.get_vcam_enabled() {
        ui.invoke_set_vcam_enabled(true);
    }
    // 安装/卸载仍在后台执行，避免 UAC 和 regsvr32 阻塞 Slint 事件循环。
    let ui_vcam_install = ui_weak.clone();
    ui.on_install_vcam(move || {
        let Some(ui) = ui_vcam_install.upgrade() else {
            return;
        };
        if ui.get_vcam_deploy_busy() {
            return;
        }
        ui.set_vcam_deploy_busy(true);
        ui.set_vcam_install_status("安装中… 请在 UAC 弹窗上确认".into());
        let ui_done = ui_vcam_install.clone();
        let spawned = std::thread::Builder::new()
            .name("vcam-install".into())
            .spawn(move || {
                let status = match vcam_backend::install() {
                    Ok(status) => {
                        log::info!("虚拟摄像头: 安装核验通过: {}", status);
                        status
                    }
                    Err(error) => {
                        let status = format!("安装未确认: {}", error);
                        log::warn!("虚拟摄像头: {}", status);
                        status
                    }
                };
                let _ = ui_done.upgrade_in_event_loop(move |ui| {
                    ui.set_vcam_install_status(status.into());
                    ui.set_vcam_deploy_busy(false);
                });
            })
            .is_ok();
        if !spawned {
            ui.set_vcam_install_status("安装未启动: 无法创建工作线程".into());
            ui.set_vcam_deploy_busy(false);
        }
    });
    let ui_vcam_uninstall = ui_weak.clone();
    let vcam_uninstall = state.vcam.clone();
    let publisher_uninstall = state.frame_publisher.clone();
    ui.on_uninstall_vcam(move || {
        let Some(ui) = ui_vcam_uninstall.upgrade() else {
            return;
        };
        if ui.get_vcam_deploy_busy() {
            return;
        }
        vcam_uninstall.set_enabled(false);
        vcam::keyboard::stop();
        *publisher_uninstall.borrow_mut() = None;
        ui.set_vcam_enabled(false);
        ui.set_vcam_runtime_status("未运行 · 采集与共享队列已释放".into());
        ui.set_vcam_deploy_busy(true);
        ui.set_vcam_install_status("卸载中… 请在 UAC 弹窗上确认".into());
        let ui_done = ui_vcam_uninstall.clone();
        let spawned = std::thread::Builder::new()
            .name("vcam-uninstall".into())
            .spawn(move || {
                let status = match vcam_backend::uninstall() {
                    Ok(status) => {
                        log::info!("虚拟摄像头: 卸载核验通过: {}", status);
                        status
                    }
                    Err(error) => {
                        let status = format!("卸载未确认: {}", error);
                        log::warn!("虚拟摄像头: {}", status);
                        status
                    }
                };
                let _ = ui_done.upgrade_in_event_loop(move |ui| {
                    ui.set_vcam_install_status(status.into());
                    ui.set_vcam_deploy_busy(false);
                });
            })
            .is_ok();
        if !spawned {
            ui.set_vcam_install_status("卸载未启动: 无法创建工作线程".into());
            ui.set_vcam_deploy_busy(false);
        }
    });
    // WinUSB 改绑三个动作。★签名与改绑是两个独立动作★: 签名会往受信任根/受信任发布者写证书,
    // 必须由用户在界面上二次确认后单独触发, 改绑一侧绝不隐式代做(见 driver_pkg 的前置校验)。
    let ui_sign = ui_weak.clone();
    let ctrl_sign = controller.clone();
    ui.on_sign_vcam_driver(move || {
        let Some(ui) = ui_sign.upgrade() else {
            return;
        };
        if ui.get_vcam_deploy_busy() {
            return;
        }
        let raw = ctrl_sign.borrow().vcam_kbd_device();
        if raw.trim().is_empty() {
            ui.set_vcam_signing_status("未签名（请先在设备树里选定要改绑的扫码器）".into());
            return;
        }
        ui.set_vcam_deploy_busy(true);
        ui.set_vcam_signing_status("签名中… 请在 UAC 弹窗上确认".into());
        let ui_done = ui_sign.clone();
        if !_spawn_winusb_action("vcam-winusb-sign", ui_done, raw, false, |_| {
            vcam::driver_pkg::sign_and_trust()
        }) {
            ui.set_vcam_signing_status("签名未启动: 无法创建工作线程".into());
            ui.set_vcam_deploy_busy(false);
        }
    });
    let ui_bind = ui_weak.clone();
    let ctrl_bind = controller.clone();
    ui.on_bind_vcam_winusb(move || {
        let Some(ui) = ui_bind.upgrade() else {
            return;
        };
        if ui.get_vcam_deploy_busy() {
            return;
        }
        let raw = ctrl_bind.borrow().vcam_kbd_device();
        if raw.trim().is_empty() {
            ui.set_vcam_binding_status("未改绑（请先在设备树里选定要改绑的扫码器）".into());
            return;
        }
        ui.set_vcam_deploy_busy(true);
        ui.set_vcam_binding_status("改绑中… 请在 UAC 弹窗上确认".into());
        let ui_done = ui_bind.clone();
        if !_spawn_winusb_action("vcam-winusb-bind", ui_done, raw, true, |path| {
            vcam::driver_pkg::install_and_bind(path)
        }) {
            ui.set_vcam_binding_status("改绑未启动: 无法创建工作线程".into());
            ui.set_vcam_deploy_busy(false);
        }
    });
    let ui_unbind = ui_weak.clone();
    let ctrl_unbind = controller.clone();
    ui.on_unbind_vcam_winusb(move || {
        let Some(ui) = ui_unbind.upgrade() else {
            return;
        };
        if ui.get_vcam_deploy_busy() {
            return;
        }
        let raw = ctrl_unbind.borrow().vcam_kbd_device();
        if raw.trim().is_empty() {
            ui.set_vcam_binding_status("未改绑（请先在设备树里选定扫码器）".into());
            return;
        }
        ui.set_vcam_deploy_busy(true);
        ui.set_vcam_binding_status("恢复中… 请在 UAC 弹窗上确认".into());
        let ui_done = ui_unbind.clone();
        if !_spawn_winusb_action("vcam-winusb-unbind", ui_done, raw, true, |path| {
            vcam::driver_pkg::unbind_and_uninstall(path)
        }) {
            ui.set_vcam_binding_status("恢复未启动: 无法创建工作线程".into());
            ui.set_vcam_deploy_busy(false);
        }
    });

    let vcam_cb = state.vcam.clone();
    ui.on_set_vcam_submit_secs(move |s| {
        vcam_cb.set_submit_timeout_ms((s.max(1) as u32) * 1000);
    });
    let vcam_cb = state.vcam.clone();
    ui.on_set_vcam_display_secs(move |s| {
        vcam_cb.set_display_ms((s.max(1) as u32) * 1000);
    });

    // 切换输入源只管理本进程键盘捕获会话，不触碰物理设备栈。
    let ctrl_clone = controller.clone();
    let kbd_list_sel = state.vcam_kbd_list.clone();
    let ui_kbd_sel = ui_weak.clone();
    ui.on_set_vcam_device(move |index| {
        // ★不可捕获的目标必须当场拒绝★ Raw Input 看不见的节点选中后一个字符也收不到,
        // 静默接受等于让用户以为已经选好了。
        let blocked = {
            let list = kbd_list_sel.borrow();
            (index > 0)
                .then(|| list.get((index - 1) as usize).map(|d| !d.selectable()))
                .flatten()
                .unwrap_or(false)
        };
        if blocked {
            let restored = vcam::keyboard::target_device()
                .as_deref()
                .and_then(|target| {
                    kbd_list_sel
                        .borrow()
                        .iter()
                        .position(|device| device.path.eq_ignore_ascii_case(target))
                })
                .map_or(0, |position| position as i32 + 1);
            log::warn!("虚拟摄像头: 拒绝切换输入源: {}", NOT_SELECTABLE);
            if let Some(ui) = ui_kbd_sel.upgrade() {
                ui.set_vcam_device_index(restored);
                ui.set_vcam_runtime_status(
                    format!("{} · {}", vcam::keyboard::runtime_status(), NOT_SELECTABLE).into(),
                );
            }
            return;
        }
        let picked = {
            let list = kbd_list_sel.borrow();
            (index > 0)
                .then(|| list.get((index - 1) as usize).map(|d| d.path.clone()))
                .flatten()
        };
        let old_saved = ctrl_clone.borrow().vcam_kbd_device();
        if let Err(error) = vcam::keyboard::set_target_device(picked.clone()) {
            ctrl_clone.borrow_mut().set_vcam_kbd_device(old_saved);
            let restored_index = {
                let list = kbd_list_sel.borrow();
                vcam::keyboard::target_device()
                    .as_deref()
                    .and_then(|target| {
                        list.iter()
                            .position(|device| device.path.eq_ignore_ascii_case(target))
                    })
                    .map_or(0, |position| position as i32 + 1)
            };
            if let Some(ui) = ui_kbd_sel.upgrade() {
                ui.set_vcam_device_index(restored_index);
                ui.set_vcam_runtime_status(
                    format!("{} · {}", vcam::keyboard::runtime_status(), error).into(),
                );
            }
            return;
        }
        ctrl_clone
            .borrow_mut()
            .set_vcam_kbd_device(picked.clone().unwrap_or_default());
        ctrl_clone.borrow_mut().push_log(format!(
            "虚拟摄像头: 输入源已切换为 {}",
            picked.as_deref().unwrap_or("所有键盘")
        ));
        if let Some(ui) = ui_kbd_sel.upgrade() {
            ui.set_vcam_device_index(index);
            ui.set_vcam_runtime_status(vcam::keyboard::runtime_status().into());
        }
    });

    // 刷新设备列表(扫码器热插拔后用)。
    let ui_kbd = ui_weak.clone();
    let ctrl_clone = controller.clone();
    let kbd_list_refresh = state.vcam_kbd_list.clone();
    ui.on_refresh_vcam_devices(move || {
        let ui = ui_kbd.upgrade().unwrap();
        let saved = ctrl_clone.borrow().vcam_kbd_device();
        let count = refresh_vcam_devices(&ui, &kbd_list_refresh, &saved);
        ctrl_clone
            .borrow_mut()
            .push_log(format!("虚拟摄像头: 已刷新键盘设备列表, 共 {} 个", count));
        // 绑定也可能被设备管理器等外部手段改掉，刷新时一并重新实测。
        // 放工作线程: 签名核验要起 PowerShell，压在事件循环上会明显卡顿。
        let ui_status = ui_kbd.clone();
        let _ = std::thread::Builder::new()
            .name("vcam-winusb-status".into())
            .spawn(move || {
                let texts = winusb_status_texts(&saved);
                let _ = ui_status
                    .upgrade_in_event_loop(move |ui| apply_winusb_status(&ui, texts));
            });
    });
}

/// 起一个 WinUSB 动作的工作线程: 提权与设备栈重建都是秒级阻塞操作，绝不能压在事件循环上。
/// 返回是否成功创建线程（失败时由调用方就地回填状态并解锁 busy 闸）。
fn _spawn_winusb_action(
    name: &'static str,
    ui_weak: slint::Weak<AppWindow>,
    raw_path: String,
    binding_line: bool,
    action: impl FnOnce(&str) -> anyhow::Result<String> + Send + 'static,
) -> bool {
    std::thread::Builder::new()
        .name(name.into())
        .spawn(move || {
            let mut failed = false;
            let mut report = match action(&raw_path) {
                Ok(detail) => {
                    log::info!("虚拟摄像头: {} 核验通过: {}", name, detail);
                    detail
                }
                Err(error) => {
                    failed = true;
                    let text = format!("未完成：{}", error);
                    log::warn!("虚拟摄像头: {} {}", name, text);
                    text
                }
            };
            // 绑定变了就必须重开采集: 模式选择(WinUSB 直读 / Interception / Raw Input)是在
            // start() 里按当前绑定实测决定的，不重启会一直停在旧模式上。
            if binding_line {
                match vcam::keyboard::restart_current_capture() {
                    // 成功重启不必上界面: 下面回填的运行状态行本身就是重启后的实测结果。
                    Ok(_) => {}
                    Err(error) => {
                        failed = true;
                        report.push_str(&format!("；⚠ 捕获会话重启失败：{}", error));
                    }
                }
            }
            let texts = winusb_status_texts(&raw_path);
            let runtime = vcam::keyboard::runtime_status();
            // ★成功时不再把操作回执拼到状态行上★ 状态行是刚刚重新实测出来的结论, 回执说的是同一件事,
            // 拼在一起就成了同一结论说两遍(用户反馈的啰嗦正是这个)。失败必须留在界面上 —— 那是状态行
            // 反映不出来的信息(状态只会显示"仍未改绑", 说不出为什么)。
            let note = if failed { report } else { String::new() };
            let _ = ui_weak.upgrade_in_event_loop(move |ui| {
                apply_winusb_status(&ui, texts);
                ui.set_vcam_winusb_note(note.into());
                ui.set_vcam_runtime_status(runtime.into());
                ui.set_vcam_deploy_busy(false);
            });
        })
        .is_ok()
}

/// 设备行名的状态前缀。★只在渲染侧拼，绝不烧进 `label`★ 否则持久化比较、排序与日志全被污染。
/// 改绑与"Raw Input 看不见"同时成立时只报改绑: 改绑是更强的解释, 看不见是它的必然结果。
#[inline]
fn _name_prefix(dev: &vcam::keyboard::KeyboardDevice) -> &'static str {
    match (dev.rebound, dev.rawinput_visible) {
        (true, _) => "[WinUSB 直读] ",
        (false, false) => "[无 Raw Input] ",
        (false, true) => "",
    }
}

/// 选中一个不可捕获的设备时给出的明确原因。★不静默失败★ 设备树里在位不等于 Raw Input 能收到数据。
const NOT_SELECTABLE: &str =
    "该设备不在 Raw Input 列表内，无法旁路捕获（设备树里在位，但键盘栈未把它公开给 Raw Input；\
     可改绑 WinUSB 后走直读）";

/// 重新枚举 HID 键盘并刷新设备树: 首行固定"所有键盘"(dev_index=0),
/// 之后按分类插入组头, 组内设备 dev_index = 在 `list` 中的下标 + 1。
/// `saved_path` 非空且仍在场 → 恢复该选择并生效; 否则回落"所有键盘"。返回设备个数。
pub(crate) fn refresh_vcam_devices(
    ui: &AppWindow,
    list: &Rc<RefCell<Vec<vcam::keyboard::KeyboardDevice>>>,
    saved_path: &str,
) -> usize {
    let mut devices = vcam::keyboard::list_keyboards();
    // ★已改绑 WinUSB 的目标必然从键盘枚举里消失★(Windows 不再为它建键盘栈)。若照常判定为
    // "设备已拔出"而回落到"所有键盘", 用户就再也选不回刚改绑的扫码器, 直读也随之停摆。
    // 故按所有权记录补一条合成项, 保持它可选、可恢复。
    if !saved_path.trim().is_empty()
        && !devices
            .iter()
            .any(|device| device.path.eq_ignore_ascii_case(saved_path))
        && vcam::driver_pkg::binding_status(saved_path).bound()
    {
        // 端点读数只能落在 USB 节点上: HID 节点已随改绑消失, 这里如实标注"HID 节点已消失"而非留空。
        let status = vcam::driver_pkg::binding_status(saved_path);
        let usb_node = status.instance().unwrap_or("未知").to_string();
        let service = vcam::keyboard::device_service(&usb_node).unwrap_or_else(|| "未知".into());
        // ★原名从改绑前的登记里恢复★ 占位名("已改绑 WinUSB 的扫码器")既认不出是哪台设备,
        // 也把端点读数挤掉了。名字与 VID/PID 只能来自所有权记录 —— 设备已不在键盘枚举里,
        // 现场再也查不到。老记录没登记这些字段时如实显示"未知（改绑前未登记设备名）", 不编造。
        let identity = vcam::driver_pkg::rebound_identity();
        let recorded = |value: Option<&String>| -> Option<String> {
            value.filter(|text| !text.trim().is_empty()).cloned()
        };
        let product = recorded(identity.as_ref().map(|id| &id.product))
            .unwrap_or_else(|| "未知（改绑前未登记设备名）".to_string());
        let label = recorded(identity.as_ref().map(|id| &id.label)).unwrap_or_else(|| product.clone());
        let vendor = recorded(identity.as_ref().map(|id| &id.vendor)).unwrap_or_default();
        // 端点行与普通行同口径: VID/PID + 实例 + 实测 service; 拿不到的项写"未知"。
        let vid_pid = match (
            vcam::driver_pkg::hex_field(&usb_node.to_ascii_uppercase(), "VID_"),
            vcam::driver_pkg::hex_field(&usb_node.to_ascii_uppercase(), "PID_"),
        ) {
            (Some(vid), Some(pid)) => format!("VID_{} PID_{}", vid, pid),
            _ => "VID/PID 未知".to_string(),
        };
        devices.insert(
            0,
            vcam::keyboard::KeyboardDevice {
                path: saved_path.to_string(),
                label,
                detail: format!("{}  ·  USB 实例 {}", vid_pid, usb_node),
                category: vcam::keyboard::CAT_USB.to_string(),
                parent_key: saved_path.to_string(),
                product,
                vendor,
                rawinput_visible: false,
                endpoint_text: format!(
                    "HID 节点已随改绑消失 · service={} · RawInput=不可见 · {} · USB节点={}",
                    service, vid_pid, usb_node
                ),
                service,
                rebound: true,
            },
        );
    }
    let devices = devices;
    let mut rows: Vec<VcamKbdRow> = vec![VcamKbdRow {
        is_group: false,
        level: 0,
        title: "所有键盘(不限定设备)".into(),
        detail: "任何键盘输入都会进入扫码缓冲, 打字会污染数据".into(),
        dev_index: 0,
    }];
    let mut cur_cat = String::new();
    let mut cur_parent = String::new();
    for (i, dev) in devices.iter().enumerate() {
        if dev.category != cur_cat {
            cur_cat = dev.category.clone();
            cur_parent.clear();
            let n = devices.iter().filter(|d| d.category == cur_cat).count();
            rows.push(VcamKbdRow {
                is_group: true,
                level: 0,
                title: cur_cat.clone().into(),
                detail: format!("{} 项", n).into(),
                dev_index: -1,
            });
        }
        let siblings = devices
            .iter()
            .filter(|d| d.parent_key == dev.parent_key)
            .count();
        if dev.parent_key != cur_parent {
            cur_parent = dev.parent_key.clone();
            if siblings > 1 {
                let mut detail = dev.vendor.clone();
                if !detail.is_empty() {
                    detail.push_str("  ·  ");
                }
                detail.push_str(&format!("{} 个键盘集合", siblings));
                rows.push(VcamKbdRow {
                    is_group: true,
                    level: 1,
                    title: dev.product.clone().into(),
                    detail: detail.into(),
                    dev_index: -1,
                });
            }
        }
        let (title, mut detail) = if siblings > 1 {
            (dev.label.clone(), dev.detail.clone())
        } else {
            let mut d = dev.vendor.clone();
            if !d.is_empty() && !dev.detail.is_empty() {
                d.push_str("  ·  ");
            }
            d.push_str(&dev.detail);
            (dev.product.clone(), d)
        };
        // 端点读数(含 service 与 Raw Input 可见性)一律进副文案: 这是"设备树有、Raw Input 没有"
        // 这个矛盾在界面上唯一可见的地方。
        if !detail.is_empty() {
            detail.push_str("  ·  ");
        }
        detail.push_str(&dev.endpoint_text);
        rows.push(VcamKbdRow {
            is_group: false,
            level: if siblings > 1 { 2 } else { 1 },
            title: format!("{}{}", _name_prefix(dev), title).into(),
            detail: detail.into(),
            dev_index: i as i32 + 1,
        });
    }
    ui.set_vcam_kbd_rows(slint::ModelRc::new(slint::VecModel::from(rows)));

    let found = devices
        .iter()
        .position(|d| d.path.eq_ignore_ascii_case(saved_path));
    // 持久化的目标可能已退化成"设备树在位但 Raw Input 不可见"(例如恢复原驱动后键盘栈没把它公开)。
    // 恢复到这种目标只会静默收不到数据, 故如实回落"所有键盘"并告警。
    let picked = match found {
        Some(i) if !devices[i].selectable() => {
            log::warn!(
                "虚拟摄像头: 持久化目标 {} 不可捕获（{}）；已回落到所有键盘",
                devices[i].label,
                NOT_SELECTABLE
            );
            None
        }
        other => other,
    };
    ui.set_vcam_device_index(picked.map(|i| i as i32 + 1).unwrap_or(0));
    if let Err(error) = vcam::keyboard::set_target_device(picked.map(|i| devices[i].path.clone())) {
        log::warn!("虚拟摄像头: 恢复输入源时未能重启捕获: {}", error);
    }

    let count = devices.len();
    for d in &devices {
        log::debug!(
            "虚拟摄像头设备树: [{}] 产品={} 厂商={} 项={} ({}) parent={} 端点={}",
            d.category,
            d.product,
            d.vendor,
            d.label,
            d.detail,
            d.parent_key,
            d.endpoint_text
        );
    }
    // ★设备树与 Raw Input 不一致本身就是最重要的诊断信息★ 必须在日志里点名, 否则"UI 看不到自己的
    // HID 端点"只能靠用户逐行读 debug 日志才发现。根因尚未定位, 这里只如实报告矛盾。
    let invisible: Vec<&str> = devices
        .iter()
        .filter(|d| !d.rawinput_visible && !d.rebound)
        .map(|d| d.endpoint_text.as_str())
        .collect();
    if !invisible.is_empty() {
        log::warn!(
            "虚拟摄像头: {} 个键盘节点在设备树里在位但不在 Raw Input 列表内（根因未定位，这些节点无法旁路捕获）：{}",
            invisible.len(),
            invisible.join(" ｜ ")
        );
    }
    log::info!(
        "虚拟摄像头: 枚举到 {} 个 HID 键盘设备, 当前选择={}",
        count,
        picked
            .map(|i| devices[i].label.clone())
            .unwrap_or_else(|| "所有键盘".into())
    );
    *list.borrow_mut() = devices;
    count
}
