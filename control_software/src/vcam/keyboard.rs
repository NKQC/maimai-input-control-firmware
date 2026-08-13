//! 虚拟摄像头: HID 键盘(扫码器)输入捕获 —— Raw Input 实现
//!
//! 说明:
//!   - 用 Raw Input(`RegisterRawInputDevices` + `RIDEV_INPUTSINK`)而非 `WH_KEYBOARD_LL`:
//!     低级钩子拿不到"这一击来自哪个设备", 无法区分扫码器和用户正在打字的主键盘, 于是
//!     打字内容会被混进扫码缓冲, 功能实际不可用。Raw Input 的 `RAWINPUTHEADER.hDevice`
//!     能标识来源设备, 才能只收指定扫码器。
//!   - `RIDEV_INPUTSINK` 让本程序即使不在前台也能收到输入(扫码时焦点通常在游戏上)。
//!   - 目标设备为 `None` 时接收**所有**键盘(兼容旧行为/用户不确定是哪个设备时);
//!     选定设备后只接收该设备, 主键盘打字不再污染缓冲。
//!   - 仅在虚拟摄像头启用(`state.enabled`)时累积; 每串数据提交后清空缓冲, 保证只用一次。
//!   - Raw Input 是**旁路监听**, 不吞按键, 扫码器输入照常进入前台窗口。

use super::{VcamState, interception, winusb_scanner};

/// WinUSB 直读模式的运行状态文案。★必须把副作用写在状态里★ 用户看到的"已吞键"是因为
/// 系统已不把该设备当键盘, 这同时意味着它对任何程序都不再输入。
const WINUSB_STATUS: &str =
    "运行中 · WinUSB 直读目标设备（Windows 已不再将其识别为键盘，按键不会进入任何程序）";
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::thread::JoinHandle;
use std::time::Instant;

use windows::Win32::Foundation::{HANDLE, HWND, LPARAM, LRESULT, WPARAM};
use windows::Win32::System::LibraryLoader::GetModuleHandleW;
use windows::Win32::UI::Input::KeyboardAndMouse::{
    GetKeyState, GetKeyboardState, MAPVK_VSC_TO_VK_EX, MapVirtualKeyW, ToUnicode, VK_CAPITAL,
    VK_RETURN, VK_SHIFT,
};
use windows::Win32::UI::Input::{
    GetRawInputData, GetRawInputDeviceInfoW, GetRawInputDeviceList, HRAWINPUT, RAWINPUT,
    RAWINPUTDEVICE, RAWINPUTDEVICELIST, RAWINPUTHEADER, RID_INPUT, RIDEV_INPUTSINK, RIDEV_REMOVE,
    RIDI_DEVICENAME, RIM_TYPEKEYBOARD, RegisterRawInputDevices,
};
use windows::Win32::UI::WindowsAndMessaging::{
    CreateWindowExW, DefWindowProcW, DestroyWindow, DispatchMessageW, HWND_MESSAGE, MSG, PM_REMOVE,
    PeekMessageW, RegisterClassW, TranslateMessage, WINDOW_EX_STYLE, WINDOW_STYLE, WM_INPUT,
    WM_KEYDOWN, WM_SYSKEYDOWN, WNDCLASSW,
};
use windows::core::w;

/// HID 键盘设备的 Usage Page / Usage(HID 规范: 1/6 = 通用桌面/键盘)。
const HID_USAGE_PAGE_GENERIC: u16 = 0x01;
const HID_USAGE_GENERIC_KEYBOARD: u16 = 0x06;

/// 一个可选作输入源的键盘设备。
/// `path` 是 Raw Input 设备名(唯一, 用于持久化); 其余字段供 UI 以设备树形式展示。
#[derive(Clone, Debug)]
pub struct KeyboardDevice {
    /// Raw Input 设备名 `\\?\HID#VID_xxxx&PID_xxxx#...`。
    pub path: String,
    /// 本项(HID 集合)显示名: 集合/接口标识, 单集合设备直接用产品名。
    pub label: String,
    /// 本项副文案: VID/PID + 实例后缀。
    pub detail: String,
    /// 设备树一级分类(按枚举器/父总线判定)。
    pub category: String,
    /// 二级分组键: 物理父设备实例 ID(同一扫码器的多个 HID 集合共享)。
    pub parent_key: String,
    /// 二级分组显示名: 总线上报的产品名(等价设备管理器"总线报告的设备说明")。
    pub product: String,
    /// 二级分组副文案: 厂商。
    pub vendor: String,
}

/// 一个设备节点的各路文案。SetupAPI 的 DeviceDesc 对 HID 键盘一律是
/// "HID Keyboard Device"/"USB 输入设备"这类通用名, 认不出设备; 真正有区分度的是
/// 总线上报名(`DEVPKEY_Device_BusReportedDeviceDesc`), 即设备的 USB 产品字符串。
#[derive(Default, Clone)]
struct NodeText {
    bus: String,
    friendly: String,
    desc: String,
    mfg: String,
}

struct Capture {
    buffer: String,
    last_key: Instant,
    state: Arc<VcamState>,
}

/// 捕获线程的**唯一所有者**。
///
/// ★为什么不能只用一个 `AtomicBool`★: 旧实现 `stop()` 只把标志置 false 就返回, 紧接着的
/// `start()` 又把它置 true —— 旧线程下一轮循环看到 true 于是继续跑, 新线程也起来了, 两个线程
/// 各建一个 message-only 窗口、各注册一次 Raw Input, 于是每个按键被计两次(缓冲里全是重复字符)。
/// 现在由这把互斥体持有 generation + JoinHandle: `stop()` 在**持锁**状态下 join 旧线程,
/// `start()` 取同一把锁 ⇒ 快速切换开关时不可能出现第二个线程/窗口。
struct Manager {
    /// 已分配的最大代号(单调递增, 只用于给线程一个唯一身份)。
    _generation: u64,
    /// 当前在跑的线程句柄; None = 没有线程。
    _worker: Option<JoinHandle<()>>,
}

static MANAGER: OnceLock<Mutex<Manager>> = OnceLock::new();
/// 当前**应当运行**的代号; 0 = 应当停止。线程每轮比对自己的代号, 不等即退出。
static ACTIVE_GENERATION: AtomicU64 = AtomicU64::new(0);

fn manager_lock() -> MutexGuard<'static, Manager> {
    MANAGER
        .get_or_init(|| {
            Mutex::new(Manager {
                _generation: 0,
                _worker: None,
            })
        })
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

static CAPTURE: OnceLock<Mutex<Capture>> = OnceLock::new();
/// 目标设备路径(None = 接收所有键盘)。
static TARGET: OnceLock<Mutex<Option<String>>> = OnceLock::new();
/// hDevice → 是否接受, 避免每次按键都去查设备名(每击一次注册表/驱动查询太重)。
/// 目标变更或设备重插(句柄变化)时失效即重算。
static ACCEPT_CACHE: OnceLock<Mutex<HashMap<isize, bool>>> = OnceLock::new();
/// 当前实际捕获模式。目标选择持久化不变，但运行时只能如实报告已拦截或旁路降级。
static RUNTIME_STATUS: OnceLock<Mutex<String>> = OnceLock::new();

fn target_lock() -> &'static Mutex<Option<String>> {
    TARGET.get_or_init(|| Mutex::new(None))
}
pub fn target_device() -> Option<String> {
    target_lock().lock().unwrap().clone()
}
fn accept_cache() -> &'static Mutex<HashMap<isize, bool>> {
    ACCEPT_CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}
fn runtime_status_lock() -> &'static Mutex<String> {
    RUNTIME_STATUS.get_or_init(|| Mutex::new("未运行 · 键盘捕获未启动".to_string()))
}
fn set_runtime_status(status: String) {
    *runtime_status_lock().lock().unwrap() = status;
}

/// 给现有 vcam UI 的实际运行状态，不改变目标设备持久化或任何对外协议。
pub fn runtime_status() -> String {
    runtime_status_lock().lock().unwrap().clone()
}

/// 设定输入源设备(`None`/空串 = 所有键盘)。运行中切换时必须先等待旧会话释放所有被吞按键并退出，
/// 再写入新目标、启动唯一的新会话，避免双开和卡键。
pub fn set_target_device(path: Option<String>) -> anyhow::Result<()> {
    let normalized = path.filter(|p| !p.trim().is_empty());
    let previous_target = target_device();
    let restart_state = {
        let manager = manager_lock();
        manager._worker.as_ref().and_then(|_| {
            CAPTURE
                .get()
                .map(|capture| capture.lock().unwrap().state.clone())
        })
    };
    if previous_target == normalized {
        return Ok(());
    }

    if restart_state.is_some() {
        stop();
    }
    *target_lock().lock().unwrap() = normalized.clone();
    accept_cache().lock().unwrap().clear();
    match normalized {
        Some(ref path) => log::info!("虚拟摄像头: 输入源限定为设备 {}", path),
        None => log::info!("虚拟摄像头: 输入源为所有键盘(未限定设备)"),
    }
    if let Some(state) = restart_state {
        if let Err(switch_error) = start(state.clone()) {
            *target_lock().lock().unwrap() = previous_target;
            accept_cache().lock().unwrap().clear();
            return match start(state) {
                Ok(()) => Err(anyhow::anyhow!(
                    "切换捕获会话失败，已恢复原输入源：{}",
                    switch_error
                )),
                Err(restore_error) => Err(anyhow::anyhow!(
                    "切换捕获会话失败且原会话恢复失败：{}；{}",
                    switch_error,
                    restore_error
                )),
            };
        }
    }
    Ok(())
}

pub fn restart_current_capture() -> anyhow::Result<bool> {
    let state = {
        let manager = manager_lock();
        if manager
            ._worker
            .as_ref()
            .is_none_or(|worker| worker.is_finished())
        {
            return Ok(false);
        }
        CAPTURE
            .get()
            .map(|capture| capture.lock().unwrap().state.clone())
            .ok_or_else(|| anyhow::anyhow!("键盘捕获会话状态不可用"))?
    };
    stop();
    start(state)?;
    Ok(true)
}

/// 枚举系统中所有 HID 键盘设备, 供 UI 下拉选择。
/// 过滤掉远程桌面虚拟键盘(`RDP_KBD`)——它不是物理输入源, 选它必然收不到数据。
pub fn list_keyboards() -> Vec<KeyboardDevice> {
    let mut out = Vec::new();
    // SAFETY: 两段式调用(先问数量再取数据), 缓冲按返回数量分配。
    unsafe {
        let entry_size = std::mem::size_of::<RAWINPUTDEVICELIST>() as u32;
        let mut count: u32 = 0;
        if GetRawInputDeviceList(None, &mut count, entry_size) == u32::MAX || count == 0 {
            return out;
        }
        let mut list = vec![RAWINPUTDEVICELIST::default(); count as usize];
        let got = GetRawInputDeviceList(Some(list.as_mut_ptr()), &mut count, entry_size);
        if got == u32::MAX {
            return out;
        }
        list.truncate(got as usize);
        for entry in list {
            if entry.dwType != RIM_TYPEKEYBOARD {
                continue;
            }
            let Some(path) = device_name(entry.hDevice) else {
                continue;
            };
            if path.to_ascii_uppercase().contains("RDP_KBD") {
                continue;
            }
            out.push(describe_device(path));
        }
    }
    // 同类相邻、同物理设备相邻: 分类 → 产品名 → 父设备实例 → 集合。
    out.sort_by(|a, b| {
        category_rank(&a.category)
            .cmp(&category_rank(&b.category))
            .then_with(|| a.product.cmp(&b.product))
            .then_with(|| a.parent_key.cmp(&b.parent_key))
            .then_with(|| a.label.cmp(&b.label))
            .then_with(|| a.path.cmp(&b.path))
    });
    out
}

/// 分类展示顺序: 外接 HID 在前(扫码器基本都在这一类), 内置/虚拟靠后。
pub fn category_rank(category: &str) -> u8 {
    match category {
        CAT_USB => 0,
        CAT_BT => 1,
        CAT_BUILTIN => 2,
        _ => 3,
    }
}

pub const CAT_USB: &str = "USB / HID 键盘";
pub const CAT_BT: &str = "蓝牙键盘";
pub const CAT_BUILTIN: &str = "内置键盘 (PS/2 · ACPI)";
pub const CAT_OTHER: &str = "其他键盘";

/// 用 Raw Input 路径反查设备树信息: 自身 + 父设备文案, 产品名优先取总线上报名。
fn describe_device(path: String) -> KeyboardDevice {
    let instance_id = instance_id_from_path(&path);
    let (me, parent_id) = instance_id
        .as_deref()
        .and_then(query_node)
        .unwrap_or((NodeText::default(), None));
    // 父节点通常是 USB 接口节点(`USB\VID&PID&MI_xx\...`); 同一物理键盘的多个接口会各成一节点,
    // 再向上一层的复合设备节点才代表"这一个设备", 用它做分组键与产品名来源。
    let (parent, group_id, group) = match parent_id.as_deref().and_then(query_node) {
        Some((ptext, grand_id)) => {
            let pid = parent_id.clone().unwrap_or_default();
            if pid.to_ascii_uppercase().contains("&MI_") {
                match grand_id.as_deref().and_then(query_node) {
                    Some((gtext, _)) => (ptext, grand_id.unwrap_or(pid), gtext),
                    None => (ptext.clone(), pid, ptext),
                }
            } else {
                (ptext.clone(), pid, ptext)
            }
        }
        None => (NodeText::default(), String::new(), NodeText::default()),
    };

    let upper_inst = instance_id.as_deref().unwrap_or(&path).to_ascii_uppercase();
    let upper_parent = group_id.to_ascii_uppercase();
    let category = if upper_inst.starts_with("ACPI")
        || upper_inst.contains("PS2")
        || upper_parent.starts_with("ACPI")
    {
        CAT_BUILTIN
    } else if upper_inst.contains("BTHENUM")
        || upper_inst.contains("BTHLE")
        || upper_parent.contains("BTH")
    {
        CAT_BT
    } else if upper_inst.starts_with("HID") || upper_parent.contains("USB") {
        CAT_USB
    } else {
        CAT_OTHER
    }
    .to_string();

    let vid_pid = vid_pid_of(&path);
    // 产品名: 总线上报名最准(即设备自报的 USB 产品串), 其次友好名, 最后才是通用 DeviceDesc。
    let product = [
        group.bus.as_str(),
        parent.bus.as_str(),
        me.bus.as_str(),
        group.friendly.as_str(),
        parent.friendly.as_str(),
        me.friendly.as_str(),
        group.desc.as_str(),
        parent.desc.as_str(),
        me.desc.as_str(),
    ]
    .into_iter()
    .find(|s| !s.is_empty() && !is_generic_name(s))
    .map(|s| s.to_string())
    .unwrap_or_else(|| {
        if vid_pid.is_empty() {
            "未知键盘设备".to_string()
        } else {
            format!("HID 键盘 {}", vid_pid)
        }
    });

    let vendor = [group.mfg.as_str(), parent.mfg.as_str(), me.mfg.as_str()]
        .into_iter()
        .find(|s| !s.is_empty() && !is_generic_name(s))
        .unwrap_or("")
        .to_string();

    // 集合行文案: 同一物理设备常有多个键盘集合(MI_xx/Colxx), 必须能分辨是哪一个。
    let collection = collection_label(&path);
    let label = match (
        collection.is_empty(),
        me.bus.is_empty() || is_generic_name(&me.bus),
    ) {
        (true, true) => product.clone(),
        (true, false) => me.bus.clone(),
        (false, true) => collection.clone(),
        (false, false) => format!("{} · {}", me.bus, collection),
    };

    let mut parts: Vec<String> = Vec::new();
    if !vid_pid.is_empty() {
        parts.push(vid_pid);
    }
    if let Some(tail) = instance_tail(&path) {
        parts.push(tail);
    }

    KeyboardDevice {
        path,
        label,
        detail: parts.join("  ·  "),
        category,
        parent_key: if group_id.is_empty() {
            product.clone()
        } else {
            group_id
        },
        product,
        vendor,
    }
}

/// 通用名判定: 这些名字每个 HID 键盘都一样, 认不出设备, 只能当兜底。
fn is_generic_name(s: &str) -> bool {
    const GENERIC: &[&str] = &[
        "hid keyboard device",
        "hid 键盘设备",
        "usb input device",
        "usb 输入设备",
        "keyboard",
        "键盘",
        "(标准键盘)",
        "(standard keyboards)",
        "(standard system devices)",
        "usb composite device",
        "usb 复合设备",
        "microsoft",
        "(标准系统设备)",
    ];
    let low = s.trim().to_lowercase();
    // 整体被括号包住的一律是 INF 类别名(如"(标准 USB 主控制器)"), 不是真厂商/产品名。
    if low.starts_with('(') && low.ends_with(')') {
        return true;
    }
    GENERIC.iter().any(|g| low == *g)
}

/// 从路径提取 HID 接口/集合标识(`MI_01`, `Col04`)。都没有则空串。
fn collection_label(path: &str) -> String {
    let upper = path.to_ascii_uppercase();
    let grab = |key: &str, len: usize| -> Option<String> {
        let at = upper.find(key)? + key.len();
        let rest = &upper[at..];
        let end = rest
            .char_indices()
            .take(len)
            .take_while(|(_, c)| c.is_ascii_alphanumeric())
            .count();
        (end > 0).then(|| rest[..end].to_string())
    };
    match (grab("&MI_", 2), grab("&COL", 2)) {
        (Some(mi), Some(col)) => format!("接口 MI_{} 集合 Col{}", mi, col),
        (Some(mi), None) => format!("接口 MI_{}", mi),
        (None, Some(col)) => format!("集合 Col{}", col),
        (None, None) => String::new(),
    }
}

/// 实例尾段(如 `0&0003`), 用于区分同型号多设备/多集合。
fn instance_tail(path: &str) -> Option<String> {
    let body = path.strip_prefix(r"\\?\")?;
    let cut = match body.find("#{") {
        Some(at) => &body[..at],
        None => body,
    };
    let seg = cut.rsplit('#').next()?;
    (!seg.is_empty()).then(|| format!("实例 {}", seg))
}

/// Raw Input 路径 → 设备实例 ID。
/// `\\?\HID#VID_1A2C&PID_0E24#7&abc&0&0000#{guid}` → `HID\VID_1A2C&PID_0E24\7&abc&0&0000`
fn instance_id_from_path(path: &str) -> Option<String> {
    let body = path
        .strip_prefix(r"\\?\")
        .or_else(|| path.strip_prefix(r"\\.\"))?;
    // 尾部的接口类 GUID 不属于实例 ID, 去掉。
    let body = match body.find("#{") {
        Some(at) => &body[..at],
        None => body,
    };
    let id = body.replace('#', "\\");
    (!id.is_empty()).then_some(id)
}

/// 提取 VID/PID 文案(无则空串)。
fn vid_pid_of(path: &str) -> String {
    let upper = path.to_ascii_uppercase();
    let field = |key: &str| -> Option<String> {
        let at = upper.find(key)? + key.len();
        let rest = &upper[at..];
        let end = rest
            .find(|c: char| !c.is_ascii_hexdigit())
            .unwrap_or(rest.len());
        (end > 0).then(|| rest[..end].to_string())
    };
    match (field("VID_"), field("PID_")) {
        (Some(v), Some(p)) => format!("VID_{} PID_{}", v, p),
        _ => String::new(),
    }
}

/// 按实例 ID 打开单个设备节点, 把已开好的句柄交给 `read` 取值, 出口统一 Destroy。
/// 节点查询有三处需求(文案 / 父节点 / 功能驱动名), 共用这一份开关箱, 免得各写一遍错误路径。
#[inline]
fn with_devnode<T>(
    instance_id: &str,
    read: impl FnOnce(
        windows::Win32::Devices::DeviceAndDriverInstallation::HDEVINFO,
        &windows::Win32::Devices::DeviceAndDriverInstallation::SP_DEVINFO_DATA,
    ) -> Option<T>,
) -> Option<T> {
    use windows::Win32::Devices::DeviceAndDriverInstallation::{
        SP_DEVINFO_DATA, SetupDiCreateDeviceInfoList, SetupDiDestroyDeviceInfoList,
        SetupDiOpenDeviceInfoW,
    };
    if instance_id.trim().is_empty() {
        return None;
    }
    // SAFETY: 空 devinfo 集合 + 按实例 ID 打开单个设备; 出口统一 Destroy。
    unsafe {
        let h = SetupDiCreateDeviceInfoList(None, None).ok()?;
        let mut data = SP_DEVINFO_DATA {
            cbSize: std::mem::size_of::<SP_DEVINFO_DATA>() as u32,
            ..Default::default()
        };
        let wide: Vec<u16> = instance_id
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();
        let mut result = None;
        if SetupDiOpenDeviceInfoW(
            h,
            windows::core::PCWSTR(wide.as_ptr()),
            None,
            0,
            Some(&mut data),
        )
        .is_ok()
        {
            result = read(h, &data);
        }
        let _ = SetupDiDestroyDeviceInfoList(h);
        result
    }
}

/// 查一个设备节点的各路文案 + 其父设备实例 ID。查不到返回 None(枚举照常继续)。
fn query_node(instance_id: &str) -> Option<(NodeText, Option<String>)> {
    with_devnode(instance_id, |h, data| {
        let text = NodeText {
            bus: dev_prop(h, data, &DEVPKEY_Device_BusReportedDeviceDesc).unwrap_or_default(),
            friendly: reg_prop(h, data, PROP_FRIENDLY).unwrap_or_default(),
            desc: reg_prop(h, data, PROP_DESC).unwrap_or_default(),
            mfg: reg_prop(h, data, PROP_MFG).unwrap_or_default(),
        };
        Some((text, parent_instance_id(data.DevInst)))
    })
}

/// 取某设备节点的父实例 ID。HID 键盘节点的父就是 USB 设备节点(`USB\VID_xxxx&PID_xxxx\序列号`),
/// 而 WinUSB 改绑只能作用在 USB 节点上, 故这是"HID 路径 → 可改绑目标"的唯一换算入口。
pub(crate) fn parent_of(instance_id: &str) -> Option<String> {
    with_devnode(instance_id, |_, data| parent_instance_id(data.DevInst))
}

/// 读设备节点的功能驱动服务名(`SPDRP_SERVICE`)。这是"到底绑到哪个驱动"的**唯一实测证据**:
/// 命令退出码 0 只说明命令跑完了, 不代表 PnP 真的换了绑定。
pub(crate) fn device_service(instance_id: &str) -> Option<String> {
    with_devnode(instance_id, |h, data| reg_prop(h, data, PROP_SERVICE))
}

/// 读一项 DEVPROP_TYPE_STRING 设备属性(用于总线上报名)。
fn dev_prop(
    h: windows::Win32::Devices::DeviceAndDriverInstallation::HDEVINFO,
    data: &windows::Win32::Devices::DeviceAndDriverInstallation::SP_DEVINFO_DATA,
    key: &windows::Win32::Foundation::DEVPROPKEY,
) -> Option<String> {
    use windows::Win32::Devices::DeviceAndDriverInstallation::SetupDiGetDevicePropertyW;
    use windows::Win32::Devices::Properties::DEVPROPTYPE;
    // SAFETY: 固定 512B 缓冲, 按返回长度截断解码; 属性缺失时返回 Err 落 None。
    unsafe {
        let mut buf = [0u8; 512];
        let mut ty = DEVPROPTYPE::default();
        let mut needed: u32 = 0;
        SetupDiGetDevicePropertyW(h, data, key, &mut ty, Some(&mut buf), Some(&mut needed), 0)
            .ok()?;
        Some(decode_wide(&buf, needed)).filter(|s| !s.is_empty())
    }
}

/// 把 REG_SZ / DEVPROP_TYPE_STRING 的字节缓冲解码成裁剪后的字符串。
fn decode_wide(buf: &[u8], needed: u32) -> String {
    let len = (needed as usize).min(buf.len()) / 2;
    let wide: Vec<u16> = (0..len)
        .map(|i| u16::from_le_bytes([buf[i * 2], buf[i * 2 + 1]]))
        .collect();
    let end = wide.iter().position(|&c| c == 0).unwrap_or(wide.len());
    String::from_utf16_lossy(&wide[..end]).trim().to_string()
}

use windows::Win32::Devices::DeviceAndDriverInstallation::{
    SETUP_DI_REGISTRY_PROPERTY, SPDRP_DEVICEDESC, SPDRP_FRIENDLYNAME, SPDRP_MFG, SPDRP_SERVICE,
};
use windows::Win32::Devices::Properties::DEVPKEY_Device_BusReportedDeviceDesc;
const PROP_FRIENDLY: SETUP_DI_REGISTRY_PROPERTY = SPDRP_FRIENDLYNAME;
const PROP_DESC: SETUP_DI_REGISTRY_PROPERTY = SPDRP_DEVICEDESC;
const PROP_MFG: SETUP_DI_REGISTRY_PROPERTY = SPDRP_MFG;
const PROP_SERVICE: SETUP_DI_REGISTRY_PROPERTY = SPDRP_SERVICE;

/// 读一项 REG_SZ 设备属性。
fn reg_prop(
    h: windows::Win32::Devices::DeviceAndDriverInstallation::HDEVINFO,
    data: &windows::Win32::Devices::DeviceAndDriverInstallation::SP_DEVINFO_DATA,
    prop: SETUP_DI_REGISTRY_PROPERTY,
) -> Option<String> {
    use windows::Win32::Devices::DeviceAndDriverInstallation::SetupDiGetDeviceRegistryPropertyW;
    // SAFETY: 固定 512B 缓冲, 按返回长度截断解码; 属性缺失时返回 Err 直接落 None。
    unsafe {
        let mut buf = [0u8; 512];
        let mut needed: u32 = 0;
        SetupDiGetDeviceRegistryPropertyW(h, data, prop, None, Some(&mut buf), Some(&mut needed))
            .ok()?;
        let s = decode_wide(&buf, needed);
        (!s.is_empty()).then_some(s)
    }
}

/// 取父设备实例 ID(CM API, 不需要额外的 devinfo 集合)。
fn parent_instance_id(dev_inst: u32) -> Option<String> {
    use windows::Win32::Devices::DeviceAndDriverInstallation::{
        CM_Get_Device_IDW, CM_Get_Parent, CR_SUCCESS,
    };
    // SAFETY: CM_Get_Device_IDW 按 MAX_DEVICE_ID_LEN(200) 上限给缓冲。
    unsafe {
        let mut parent: u32 = 0;
        if CM_Get_Parent(&mut parent, dev_inst, 0) != CR_SUCCESS {
            return None;
        }
        let mut buf = [0u16; 256];
        if CM_Get_Device_IDW(parent, &mut buf, 0) != CR_SUCCESS {
            return None;
        }
        let end = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
        let s = String::from_utf16_lossy(&buf[..end]);
        (!s.is_empty()).then_some(s)
    }
}

/// 读取某设备的 Raw Input 设备名(形如 `\\?\HID#VID_1234&PID_5678#...`)。
fn device_name(h_device: HANDLE) -> Option<String> {
    // SAFETY: 两段式调用; 首次取长度(字符数), 再按长度取宽字符串。
    unsafe {
        let mut chars: u32 = 0;
        if GetRawInputDeviceInfoW(Some(h_device), RIDI_DEVICENAME, None, &mut chars) == u32::MAX {
            return None;
        }
        if chars == 0 || chars > 4096 {
            return None;
        }
        let mut buf = vec![0u16; chars as usize + 1];
        let got = GetRawInputDeviceInfoW(
            Some(h_device),
            RIDI_DEVICENAME,
            Some(buf.as_mut_ptr() as *mut _),
            &mut chars,
        );
        if got == u32::MAX {
            return None;
        }
        let end = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
        Some(String::from_utf16_lossy(&buf[..end]))
    }
}

/// 判断这一击是否来自目标设备。目标为 None 时全收。
fn device_accepted(h_device: HANDLE) -> bool {
    let Some(target) = target_device() else {
        return true;
    };
    let key = h_device.0 as isize;
    if let Some(&cached) = accept_cache().lock().unwrap().get(&key) {
        return cached;
    }
    let accepted = device_name(h_device)
        .map(|n| n.eq_ignore_ascii_case(&target))
        .unwrap_or(false);
    accept_cache().lock().unwrap().insert(key, accepted);
    accepted
}

/// 启动键盘捕获线程。未指定目标时保留 Raw Input 旁路；指定目标时优先精确 Interception，
/// 任何 API/驱动/签名/系统或硬件 ID 唯一性失败都明确降级为同一目标的 Raw Input 旁路，绝不声称拦截。
pub fn start(state: Arc<VcamState>) -> anyhow::Result<()> {
    let mut manager = manager_lock();
    CAPTURE.get_or_init(|| {
        Mutex::new(Capture {
            buffer: String::new(),
            last_key: Instant::now(),
            state: state.clone(),
        })
    });
    if let Some(capture) = CAPTURE.get() {
        capture.lock().unwrap().state = state;
    }
    if manager
        ._worker
        .as_ref()
        .is_some_and(|worker| worker.is_finished())
    {
        if let Some(worker) = manager._worker.take() {
            let _ = worker.join();
        }
        ACTIVE_GENERATION.store(0, Ordering::SeqCst);
    }
    if manager._worker.is_some() {
        return Ok(());
    }

    manager._generation += 1;
    let generation = manager._generation;
    let selected_target = target_device();
    ACTIVE_GENERATION.store(generation, Ordering::SeqCst);
    match selected_target.as_deref() {
        None => {
            set_runtime_status("运行中 · Raw Input 旁路（所有键盘；不拦截按键）".to_string());
            spawn_raw_worker(&mut manager, generation)
        }
        // ★优先 WinUSB 直读★ 目标已改绑 winusb.sys 时 Windows 根本不再为它建键盘栈,
        // Raw Input 与键盘过滤驱动都收不到任何东西 —— 直读是唯一能拿到数据的路径。
        Some(path) if super::driver_pkg::binding_status(path).bound() => {
            match winusb_scanner::Capture::open(path) {
                Ok(capture) => {
                    set_runtime_status(WINUSB_STATUS.to_string());
                    let target = path.to_string();
                    spawn_worker(&mut manager, "vcam-winusb", move || {
                        pump_winusb(generation, target, capture)
                    })
                }
                Err(error) => {
                    ACTIVE_GENERATION.store(0, Ordering::SeqCst);
                    let reason = error.to_string();
                    // 这里**不能**降级 Raw Input: 设备已不是键盘, 旁路一个字符也收不到,
                    // 报"运行中"就是在骗人。如实停在未运行, 并给出可执行的下一步。
                    set_runtime_status(format!(
                        "未运行 · 目标已改绑 WinUSB 但直读未建立：{}。可点“恢复原驱动”退回普通 HID 键盘",
                        reason
                    ));
                    Err(anyhow::anyhow!("WinUSB 直读未建立：{}", reason))
                }
            }
        }
        Some(path) => match interception::Capture::open(path) {
            Ok(capture) => {
                set_runtime_status(format!("运行中 · Interception 精确拦截目标设备 {}", path));
                let target = path.to_string();
                spawn_worker(&mut manager, "vcam-interception", move || {
                    pump_interception(generation, target, capture)
                })
            }
            Err(error) => {
                let reason = error.to_string();
                log::warn!(
                    "虚拟摄像头: 目标设备 Interception 不可用，降级 Raw Input 旁路: {}",
                    reason
                );
                set_runtime_status(format!(
                    "运行中 · Raw Input 旁路（仅目标设备；未拦截按键）· 降级原因：{}",
                    reason
                ));
                spawn_raw_worker(&mut manager, generation)
            }
        },
    }
}

/// 起唯一的捕获线程并登记到 Manager。三种捕获模式共用同一份创建与失败收尾逻辑。
fn spawn_worker(
    manager: &mut Manager,
    name: &'static str,
    body: impl FnOnce() + Send + 'static,
) -> anyhow::Result<()> {
    match std::thread::Builder::new().name(name.into()).spawn(body) {
        Ok(worker) => {
            manager._worker = Some(worker);
            Ok(())
        }
        Err(error) => {
            ACTIVE_GENERATION.store(0, Ordering::SeqCst);
            set_runtime_status(format!("未运行 · 无法创建捕获线程 {}：{}", name, error));
            Err(anyhow::anyhow!("无法创建捕获线程 {}：{}", name, error))
        }
    }
}

fn spawn_raw_worker(manager: &mut Manager, generation: u64) -> anyhow::Result<()> {
    spawn_worker(manager, "vcam-rawinput", move || pump(generation))
}

/// 停止键盘捕获: **等旧线程真正注销 Raw Input、释放 Interception 已吞按键并退出**才返回。
/// 持锁 join 是关键 —— 它让紧随其后的 `start()` 不可能与旧线程并存。
pub fn stop() {
    let mut manager = manager_lock();
    ACTIVE_GENERATION.store(0, Ordering::SeqCst);
    if let Some(worker) = manager._worker.take() {
        let _ = worker.join();
    }
    set_runtime_status("未运行 · 键盘捕获已停止".to_string());
}

/// WinUSB 直读线程。
///
/// ★这条路不许降级 Raw Input★ 目标已改绑 winusb.sys ⇒ 系统里根本没有它的键盘节点,
/// 旁路一个字符也收不到。会话中断时只能如实报"未运行", 而不是换个名字继续假装在跑。
fn pump_winusb(generation: u64, target: String, mut capture: winusb_scanner::Capture) {
    let still_mine = || ACTIVE_GENERATION.load(Ordering::SeqCst) == generation;
    log::info!("虚拟摄像头: WinUSB 直读捕获已启动({})", target);
    while still_mine() {
        if target_device().as_deref() != Some(target.as_str()) {
            break;
        }
        match capture.receive(100) {
            Ok(keys) => {
                for (vk, shift) in keys {
                    push_key(vk, Some(shift));
                }
                submit_on_pause();
            }
            Err(error) => {
                let reason = error.to_string();
                log::error!("虚拟摄像头: WinUSB 直读已停止: {}", reason);
                capture.stop();
                set_runtime_status(format!("未运行 · WinUSB 直读中断：{}", reason));
                return;
            }
        }
    }
    capture.stop();
    log::info!("虚拟摄像头: WinUSB 直读捕获已停止");
}

/// Interception 捕获线程：只有已唯一匹配的目标设备进入驱动过滤；异常或目标改变时先释放已吞按键，
/// 再继续同一目标的 Raw Input 旁路，从不把旁路误报为拦截。
fn pump_interception(generation: u64, mut target: String, mut capture: interception::Capture) {
    let still_mine = || ACTIVE_GENERATION.load(Ordering::SeqCst) == generation;
    log::info!("虚拟摄像头: 目标设备 Interception 捕获已启动({})", target);
    while still_mine() {
        let current = target_device();
        if current.as_deref() != Some(target.as_str()) {
            capture.stop();
            match current {
                Some(next) => match interception::Capture::open(&next) {
                    Ok(next_capture) => {
                        target = next;
                        capture = next_capture;
                        set_runtime_status(format!(
                            "运行中 · Interception 精确拦截目标设备 {}",
                            target
                        ));
                        log::info!("虚拟摄像头: 已切换 Interception 目标设备 {}", target);
                        continue;
                    }
                    Err(error) => {
                        let reason = error.to_string();
                        set_runtime_status(format!(
                            "运行中 · Raw Input 旁路（仅目标设备；未拦截按键）· 降级原因：{}",
                            reason
                        ));
                        log::warn!("虚拟摄像头: 目标设备切换降级 Raw Input 旁路: {}", reason);
                        pump(generation);
                        return;
                    }
                },
                None => {
                    set_runtime_status(
                        "运行中 · Raw Input 旁路（所有键盘；不拦截按键）".to_string(),
                    );
                    log::info!("虚拟摄像头: 目标已清除，恢复 Raw Input 旁路捕获");
                    pump(generation);
                    return;
                }
            }
        }
        match capture.receive(100) {
            Ok(Some(stroke)) if stroke.is_key_down() => {
                let scan = u32::from(stroke.code) | if stroke.is_extended() { 0xe000 } else { 0 };
                let vk = unsafe { MapVirtualKeyW(scan, MAPVK_VSC_TO_VK_EX) } as u16;
                if vk != 0 {
                    // Interception 是键盘类过滤驱动: 按键仍然经过系统键盘状态机, OS Shift 态可信。
                    push_key(vk, None);
                }
            }
            Ok(_) => submit_on_pause(),
            Err(error) => {
                let reason = error.to_string();
                log::error!(
                    "虚拟摄像头: Interception 目标设备捕获已安全停止，降级 Raw Input 旁路: {}",
                    reason
                );
                capture.stop();
                set_runtime_status(format!(
                    "运行中 · Raw Input 旁路（仅目标设备；未拦截按键）· 降级原因：{}",
                    reason
                ));
                pump(generation);
                return;
            }
        }
    }
    capture.stop();
    log::info!("虚拟摄像头: Interception 目标设备捕获已停止");
}

/// 捕获线程主体: 建 message-only 窗口 → 注册 Raw Input → 泵消息 → 注销并销毁窗口。
/// 只要 `ACTIVE_GENERATION` 不再等于自己的代号就收尾退出。
fn pump(generation: u64) {
    let still_mine = || ACTIVE_GENERATION.load(Ordering::SeqCst) == generation;
    // SAFETY: 窗口、Raw Input 注册与消息泵全在本线程内成对完成; 出口统一注销 + DestroyWindow。
    unsafe {
        // Raw Input 必须绑定一个窗口来收 WM_INPUT; 用 message-only 窗口(HWND_MESSAGE),
        // 无界面、不进任务栏、不抢焦点。
        let Some(hwnd) = create_sink_window() else {
            return;
        };
        let devices = [RAWINPUTDEVICE {
            usUsagePage: HID_USAGE_PAGE_GENERIC,
            usUsage: HID_USAGE_GENERIC_KEYBOARD,
            dwFlags: RIDEV_INPUTSINK, // 后台也收(扫码时焦点在游戏上)
            hwndTarget: hwnd,
        }];
        if let Err(e) =
            RegisterRawInputDevices(&devices, std::mem::size_of::<RAWINPUTDEVICE>() as u32)
        {
            log::error!("虚拟摄像头: 注册 Raw Input 键盘失败: {:?}", e);
            let _ = DestroyWindow(hwnd);
            return;
        }
        log::info!(
            "虚拟摄像头: 键盘捕获已启动(Raw Input, 源={})",
            target_device().unwrap_or_else(|| "所有键盘".into())
        );

        let mut msg = MSG::default();
        while still_mine() {
            while PeekMessageW(&mut msg, None, 0, 0, PM_REMOVE).as_bool() {
                let _ = TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
            submit_on_pause();
            std::thread::sleep(std::time::Duration::from_millis(20));
        }

        // 注销: RIDEV_REMOVE 要求 hwndTarget 为空。
        let remove = [RAWINPUTDEVICE {
            usUsagePage: HID_USAGE_PAGE_GENERIC,
            usUsage: HID_USAGE_GENERIC_KEYBOARD,
            dwFlags: RIDEV_REMOVE,
            hwndTarget: HWND(std::ptr::null_mut()),
        }];
        let _ = RegisterRawInputDevices(&remove, std::mem::size_of::<RAWINPUTDEVICE>() as u32);
        let _ = DestroyWindow(hwnd);
        accept_cache().lock().unwrap().clear();
        log::info!("虚拟摄像头: 键盘捕获已停止");
    }
}

/// 创建 message-only 窗口作为 WM_INPUT 接收端。类名重复注册会失败, 直接忽略
/// (第二次 start 时类已存在, CreateWindowExW 仍可用)。
unsafe fn create_sink_window() -> Option<HWND> {
    let class_name = w!("Mai2VcamRawInputSink");
    let hinstance = unsafe { GetModuleHandleW(None) }.ok()?;
    let wc = WNDCLASSW {
        lpfnWndProc: Some(wnd_proc),
        hInstance: hinstance.into(),
        lpszClassName: class_name,
        ..Default::default()
    };
    let _ = unsafe { RegisterClassW(&wc) };
    let hwnd = unsafe {
        CreateWindowExW(
            WINDOW_EX_STYLE(0),
            class_name,
            class_name,
            WINDOW_STYLE(0),
            0,
            0,
            0,
            0,
            Some(HWND_MESSAGE),
            None,
            Some(hinstance.into()),
            None,
        )
    };
    match hwnd {
        Ok(h) => Some(h),
        Err(e) => {
            log::error!("虚拟摄像头: 创建 Raw Input 接收窗口失败: {:?}", e);
            None
        }
    }
}

unsafe extern "system" fn wnd_proc(
    hwnd: HWND,
    msg: u32,
    wparam: WPARAM,
    lparam: LPARAM,
) -> LRESULT {
    if msg == WM_INPUT {
        unsafe { handle_raw_input(HRAWINPUT(lparam.0 as *mut _)) };
    }
    unsafe { DefWindowProcW(hwnd, msg, wparam, lparam) }
}

/// 解析一条 WM_INPUT: 只取键盘按下事件, 且必须来自目标设备。
unsafe fn handle_raw_input(hri: HRAWINPUT) {
    let header_size = std::mem::size_of::<RAWINPUTHEADER>() as u32;
    let mut size: u32 = 0;
    if unsafe { GetRawInputData(hri, RID_INPUT, None, &mut size, header_size) } == u32::MAX
        || size == 0
    {
        return;
    }
    let mut buf = vec![0u8; size as usize];
    let got = unsafe {
        GetRawInputData(
            hri,
            RID_INPUT,
            Some(buf.as_mut_ptr() as *mut _),
            &mut size,
            header_size,
        )
    };
    if got == u32::MAX || (got as usize) < std::mem::size_of::<RAWINPUTHEADER>() {
        return;
    }
    // SAFETY: 缓冲按 GetRawInputData 要求的大小分配并填充, 起始即 RAWINPUT。
    let raw = unsafe { &*(buf.as_ptr() as *const RAWINPUT) };
    if raw.header.dwType != RIM_TYPEKEYBOARD.0 {
        return;
    }
    // SAFETY: dwType 已确认为键盘, 读 union 的 keyboard 成员合法。
    let kb = unsafe { &raw.data.keyboard };
    if kb.Message != WM_KEYDOWN && kb.Message != WM_SYSKEYDOWN {
        return;
    }
    if !device_accepted(raw.header.hDevice) {
        return;
    }
    push_key(kb.VKey, None);
}

/// 停顿超时提交: 缓冲非空且距上次按键超过阈值 → 提交并清空。
/// 三条捕获路径(Raw Input / Interception / WinUSB 直读)共用同一份提交判定与同一个下游,
/// 不给某一条路单独造第二套缓冲语义。
#[inline]
fn submit_on_pause() {
    let Some(m) = CAPTURE.get() else {
        return;
    };
    let mut cap = m.lock().unwrap();
    if cap.buffer.is_empty() {
        return;
    }
    if cap.last_key.elapsed().as_millis() >= cap.state.submit_timeout_ms() as u128 {
        let data = std::mem::take(&mut cap.buffer);
        cap.state.submit_data(&data);
    }
}

/// 累积一次按键: Enter 立即提交, 可打印字符入缓冲。
///
/// `shift` = None 表示"这一击确实经过了系统", Shift 态可从 OS 键盘状态读;
/// WinUSB 直读路径的按键**根本不进系统**, 必须由报告的 modifier 字节显式给出, 否则会拿到
/// 用户主键盘此刻的 Shift 态, 解出与扫码内容无关的字符。
fn push_key(vk: u16, shift: Option<bool>) {
    let Some(m) = CAPTURE.get() else {
        return;
    };
    let mut cap = m.lock().unwrap();
    if !cap.state.enabled() {
        return;
    }
    if vk == VK_RETURN.0 {
        // Enter: 立即提交当前缓冲(每串只用一次)。
        if !cap.buffer.is_empty() {
            let data = std::mem::take(&mut cap.buffer);
            cap.state.submit_data(&data);
        }
    } else if let Some(ch) = vk_to_char(vk, shift) {
        cap.buffer.push(ch);
        cap.last_key = Instant::now();
        // 防御: 单串过长(异常)截断, 避免无限增长。
        if cap.buffer.len() > 4096 {
            cap.buffer.clear();
        }
    }
}

/// VK → 可打印字符(经 ToUnicode, 尊重当前布局)。不可打印返回 None。
/// `shift` 为 Some 时**覆盖**系统 Shift 态(见 `push_key` 的说明)。
fn vk_to_char(vk: u16, shift: Option<bool>) -> Option<char> {
    unsafe {
        let mut state = [0u8; 256];
        // 取当前键盘状态(供 ToUnicode 判断 Shift 等); 失败则退化用 Shift 单键。
        if GetKeyboardState(&mut state).is_err() {
            let pressed = (GetKeyState(VK_SHIFT.0 as i32) as u16 & 0x8000) != 0;
            state[VK_SHIFT.0 as usize] = if pressed { 0x80 } else { 0 };
        }
        if let Some(pressed) = shift {
            // 直读路径: 只有报告里的 Shift 才算, 顺手清掉 CapsLock 影响(扫码器不发 CapsLock,
            // 用户主键盘的 CapsLock 不该改变扫码内容)。
            state[VK_SHIFT.0 as usize] = if pressed { 0x80 } else { 0 };
            state[VK_CAPITAL.0 as usize] = 0;
        }
        let sc = 0u32; // scan code 0: ToUnicode 会据 vk 推断
        let mut buf = [0u16; 8];
        let n = ToUnicode(vk as u32, sc, Some(&state), &mut buf, 0);
        if n == 1 {
            let c = char::from_u32(buf[0] as u32)?;
            // 只接受可打印字符(排除控制符)。
            if !c.is_control() {
                return Some(c);
            }
        }
        None
    }
}
