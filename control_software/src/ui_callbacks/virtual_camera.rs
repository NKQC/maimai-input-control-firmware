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
    // 可选输入源列表(下拉索引 0 = 所有键盘, 之后按此 Vec 顺序对应)。
    // 启动即按持久化的设备路径恢复选择: 设备不在场时回落到"所有键盘", 不静默失效。
    let (_, saved) = refresh_vcam_devices(ui, &state.vcam_kbd_list, controller);
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
            log::info!("虚拟摄像头: 已停止, 共享队列生产者已释放");
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
        // 队列落在哪个命名空间决定了 Windows 设置 / 相机应用能不能取到画面(它们走 Session 0 的
        // Frame Server, 只有 Global\ 跨得过去), 因此这条结论必须上界面, 不能只躺在日志里。
        let queue_note = match FramePublisher::create() {
            Ok(publisher) => {
                let note = publisher.namespace().consequence().to_string();
                *publisher_cb.borrow_mut() = Some(publisher);
                note
            }
            Err(error) => {
                log::error!("虚拟摄像头: 无法创建共享队列: {}", error);
                vcam_cb.set_enabled(false);
                ui.set_vcam_enabled(false);
                ui.set_vcam_runtime_status(format!("未运行 · {}", error).into());
                return;
            }
        };
        match vcam::keyboard::start(vcam_cb.clone()) {
            Ok(()) => {
                vcam_cb.set_enabled(true);
                ui.set_vcam_enabled(true);
                ui.set_vcam_runtime_status(
                    format!("{} · {}", vcam::keyboard::runtime_status(), queue_note).into(),
                );
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
    // 测试覆盖模式: 只切 VcamState 的一个标志, 出画由既有的 10fps 发布节拍照常承担 ——
    // 它验证的正是那条路本身, 所以绝不能给它另开一条发布通道。
    let vcam_test_cb = state.vcam.clone();
    let ui_vcam_test = ui_weak.clone();
    ui.on_set_vcam_test_pattern(move |on| {
        let Some(ui) = ui_vcam_test.upgrade() else {
            return;
        };
        vcam_test_cb.set_test_pattern(on);
        ui.set_vcam_test_pattern(on);
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
        // ★不再按"是否在 Raw Input 列表里"拒绝选中★ 实测该列表会漏报接口健康的键盘节点
        // (用户唯一可用的扫码器就是被这条挡住的)。现在一律受理选择, 把实测障碍写进状态行,
        // 再由 keyboard::runtime_status 的收报计数如实回答"到底有没有收到数据"。
        let warning = {
            let list = kbd_list_sel.borrow();
            (index > 0)
                .then(|| {
                    list.get((index - 1) as usize)
                        .and_then(|device| device.capture_warning())
                })
                .flatten()
        };
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
        if let Some(warning) = warning.as_deref() {
            log::warn!("虚拟摄像头: 已受理输入源切换, 但实测存在障碍: {}", warning);
        }
        if let Some(ui) = ui_kbd_sel.upgrade() {
            ui.set_vcam_device_index(index);
            ui.set_vcam_runtime_status(
                match warning {
                    Some(warning) => format!("{} · {}", vcam::keyboard::runtime_status(), warning),
                    None => vcam::keyboard::runtime_status(),
                }
                .into(),
            );
        }
    });

    // 刷新设备列表(扫码器热插拔后用)。
    let ui_kbd = ui_weak.clone();
    let ctrl_clone = controller.clone();
    let kbd_list_refresh = state.vcam_kbd_list.clone();
    ui.on_refresh_vcam_devices(move || {
        let ui = ui_kbd.upgrade().unwrap();
        let (count, saved) = refresh_vcam_devices(&ui, &kbd_list_refresh, &ctrl_clone);
        ctrl_clone
            .borrow_mut()
            .push_log(format!("虚拟摄像头: 已刷新输入源列表, 共 {} 个", count));
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
/// 三档按"解释力从强到弱"排: 改绑 > 键盘接口打不开 > 仅列表滞后。前者都是后者的成因,
/// 同时成立时只报最强的那一个。
#[inline]
fn _name_prefix(dev: &vcam::keyboard::KeyboardDevice) -> &'static str {
    use vcam::keyboard::{DirectRead, KbdIface};
    if dev.rebound {
        // 已改绑设备只有直读这一条通路: 通路开不开是唯一值得摊在名字前面的事实。
        return match dev.direct_read {
            DirectRead::Failed(_) => "[已改绑 · 现在读不到] ",
            _ => "[本程序直读] ",
        };
    }
    match (dev.kbd_iface, dev.rawinput_visible) {
        (KbdIface::Failed(_), _) => "[系统收不到输入] ",
        (_, false) => "[系统未登记为输入源] ",
        (_, true) => "",
    }
}

/// 重新枚举可选输入源并刷新设备树: 首行固定"所有键盘"(dev_index=0),
/// 之后按分类插入组头, 组内设备 dev_index = 在 `list` 中的下标 + 1。
/// 持久化路径仍能认出某台设备 → 恢复该选择并生效; 否则回落"所有键盘"。
/// 返回 (设备个数, 当前生效的持久化路径)。
pub(crate) fn refresh_vcam_devices(
    ui: &AppWindow,
    list: &Rc<RefCell<Vec<vcam::keyboard::KeyboardDevice>>>,
    controller: &Rc<RefCell<AppController>>,
) -> (usize, String) {
    let mut saved_path = controller.borrow().vcam_kbd_device();
    // 已改绑设备的合成行由 `list_input_sources` 一并给出: 它挂在"本应用改绑过它"这个设备树事实上,
    // 而不是挂在这里的持久化选择上 —— 后者被清空时用户就再也选不回那台设备了。
    let devices = vcam::keyboard::list_input_sources();
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

    let mut picked = devices
        .iter()
        .position(|d| d.path.eq_ignore_ascii_case(&saved_path));
    // ★路径整串对不上时按设备身份再认一次★ 改绑设备的行用的是不含漂移字段的稳定路径, 与当初
    // 持久化的那条 HID 路径写法完全不同; 只按字符串比就会把"设备还在、只是标识换了写法"当成
    // "设备已拔出", 用户的选择随之丢失。认回后立刻把持久化改写成稳定路径, 免得每次开机重来一遍。
    if picked.is_none() && !saved_path.trim().is_empty() {
        picked = devices
            .iter()
            .position(|d| d.rebound && vcam::keyboard::same_usb_identity(&d.path, &saved_path));
        if let Some(index) = picked {
            log::info!(
                "虚拟摄像头: 持久化路径 {} 已按设备身份认回改绑设备 {}，持久化改写为稳定标识 {}",
                saved_path,
                devices[index].label,
                devices[index].path
            );
            saved_path = devices[index].path.clone();
            controller
                .borrow_mut()
                .set_vcam_kbd_device(saved_path.clone());
        }
    }
    // ★不再因"实测有障碍"就把用户选好的目标偷偷换成所有键盘★ 那样只会让用户以为自己没选中,
    // 而"所有键盘"还会把打字混进扫码缓冲。障碍照实记进日志与状态行, 选择本身保留。
    if let Some(device) = picked.map(|index| &devices[index]) {
        if let Some(warning) = device.capture_warning() {
            log::warn!(
                "虚拟摄像头: 已恢复持久化目标 {}，但实测存在障碍: {}",
                device.label,
                warning
            );
        }
    }
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
    // HID 端点"只能靠用户逐行读 debug 日志才发现。根因已定位: 缺失节点的键盘接口 CreateFileW
    // 打不开(实测 ERROR_GEN_FAILURE), win32k 因此既不列它也读不到它 ⇒ 全系统收不到该设备按键。
    // 故按"接口打不开"与"仅列表滞后"分开报: 前者要重建键盘栈, 后者无需任何处置。
    // 界面只说"系统收不到", 具体错误码留在这里: `KbdIface` 的 Debug 带 Win32 码, 是复现与归因的凭据。
    let broken: Vec<String> = devices
        .iter()
        .filter(|d| !d.rebound && matches!(d.kbd_iface, vcam::keyboard::KbdIface::Failed(_)))
        .map(|d| format!("{} · {:?}", d.endpoint_text, d.kbd_iface))
        .collect();
    if !broken.is_empty() {
        log::warn!(
            "虚拟摄像头: {} 个键盘节点的键盘接口打不开（Windows 收不到它们的任何按键，需停用再启用该节点或重启系统）：{}",
            broken.len(),
            broken.join(" ｜ ")
        );
    }
    let lagging: Vec<&str> = devices
        .iter()
        .filter(|d| {
            !d.rebound && !d.rawinput_visible && d.kbd_iface == vcam::keyboard::KbdIface::Open
        })
        .map(|d| d.endpoint_text.as_str())
        .collect();
    if !lagging.is_empty() {
        log::warn!(
            "虚拟摄像头: {} 个键盘节点键盘接口可打开但暂不在 Raw Input 列表内（属列表滞后，仍允许选中）：{}",
            lagging.len(),
            lagging.join(" ｜ ")
        );
    }
    // 改绑设备的直读实测结论必须留痕: 它是这类设备唯一的数据通路, "读不到"与"设备没在场"
    // 在界面上看着一样, 只有这条日志能分开。
    for device in devices.iter().filter(|d| d.rebound) {
        log::info!(
            "虚拟摄像头: 已改绑设备 {}（{}）直读实测 {:?}",
            device.label,
            device.endpoint_text,
            device.direct_read
        );
    }
    log::info!(
        "虚拟摄像头: 枚举到 {} 个可选输入源, 当前选择={}",
        count,
        picked
            .map(|i| devices[i].label.clone())
            .unwrap_or_else(|| "所有键盘".into())
    );
    *list.borrow_mut() = devices;
    (count, saved_path)
}
