//! Interception v1.0.1 的最小运行时封装。
//!
//! 本模块只经官方 `interception.dll` API 与键盘过滤驱动通信；不会注册或改变
//! Raw Input。未能证明目标设备唯一、驱动可用或 API/系统兼容时，调用方必须降级，
//! 不得宣称目标设备已被拦截。

use std::ffi::c_void;
use std::os::windows::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicI32, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};
use windows::Win32::Foundation::{ERROR_SUCCESS, FreeLibrary, HMODULE};
use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryW};
use windows::Win32::System::Registry::{
    HKEY, HKEY_LOCAL_MACHINE, KEY_READ, RegCloseKey, RegOpenKeyExW, RegQueryValueExW,
};
use windows::core::{PCSTR, PCWSTR};

const API_FILE: &str = "interception.dll";
pub(crate) const INSTALLER_FILE: &str = "install-interception.exe";
pub(crate) const LICENSE_FILE: &str = "Interception-LGPL-3.0.txt";
pub(crate) const OWNER_FILE: &str = "interception-owner-v1.txt";
const OWNER_MARKER: &[u8] = b"mai2control Interception v1.0.1 driver ownership\r\n";
pub(crate) const MAX_KEYBOARD: i32 = 10;
const FILTER_KEY_ALL: u16 = 0xffff;

mod embedded {
    include!(concat!(env!("OUT_DIR"), "/interception_embedded.rs"));
}

/// 仅供 vcam backend 将已核验的官方资产部署到本应用目录。
pub(crate) fn deployment_assets() -> [(&'static str, &'static [u8]); 3] {
    [
        (API_FILE, embedded::BYTES_API_X64),
        (INSTALLER_FILE, embedded::BYTES_INSTALLER),
        (LICENSE_FILE, embedded::BYTES_LICENSE),
    ]
}

pub(crate) fn owner_marker() -> &'static [u8] {
    OWNER_MARKER
}

pub(crate) fn embedded_available() -> bool {
    cfg!(target_arch = "x86_64")
        && deployment_assets()
            .iter()
            .all(|(_, bytes)| !bytes.is_empty())
}

pub(crate) fn install_dir() -> PathBuf {
    std::env::var_os("ProgramData")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(r"C:\ProgramData"))
        .join("mai2control")
        .join("vcam")
        .join("interception")
}

pub(crate) fn owner_marker_matches() -> bool {
    std::fs::read(install_dir().join(OWNER_FILE)).is_ok_and(|bytes| bytes == OWNER_MARKER)
}

pub(crate) fn deployed_assets_match() -> bool {
    embedded_available()
        && deployment_assets().iter().all(|(name, bytes)| {
            std::fs::read(install_dir().join(name)).is_ok_and(|on_disk| on_disk == *bytes)
        })
}

#[derive(Debug)]
pub(crate) enum DriverStatus {
    Ready,
    AssetsMissing,
    ApiUnavailable(String),
    /// 驱动**注册**已在注册表落地(键盘类上层过滤 + 服务键), 但内核里还没加载 ⇒ 只差一次重启。
    /// 这不是失败: 键盘类上层过滤驱动只在设备栈重建(即开机)时挂载, 装完立刻探测必然还不可用。
    PendingReboot,
    /// 注册表实测: 键盘类 `UpperFilters` 里没有该服务、或服务键不存在 ⇒ **驱动从来没装上**。
    /// ★与"槽位不可用"必须分开★ 混成一句会让人把"没装"误判成"装了但对这个设备无效",
    /// 从而去排查根本不存在的兼容性问题(本工程实际就走过这条弯路)。
    NotInstalled,
    /// 驱动已加载, 但系统在位键盘节点数已超过 Interception 的键盘槽上限 ⇒ 排在上限之外的
    /// 设备**不会出现在任何槽里**, 且谁占到槽取决于设备栈建立顺序(开机/插拔), 不可控。
    /// 这是该驱动的结构性上限, 不是安装问题, 重装重启都不解决。
    SlotsExhausted { present: usize },
}

impl DriverStatus {
    pub(crate) fn detail(&self) -> String {
        match self {
            Self::Ready => format!(
                "Interception 驱动与 x64 API 已核验可用（在位键盘节点 {}/槽上限 {}）{}",
                present_keyboard_nodes()
                    .map(|n| n.to_string())
                    .unwrap_or_else(|| "未知".to_string()),
                MAX_KEYBOARD,
                _slot_pressure_note()
            ),
            Self::AssetsMissing => "Interception x64 API/安装器/许可证未完整部署".to_string(),
            Self::ApiUnavailable(error) => format!(
                "Interception API 无法加载（可能为签名、系统版本或位宽不兼容）：{}",
                error
            ),
            Self::PendingReboot => {
                "Interception 驱动已注册为键盘类上层过滤，需在下一次系统启动或自然设备栈重建后才可用；当前不会拦截输入".to_string()
            }
            Self::NotInstalled => format!(
                "Interception 驱动未安装（键盘类 UpperFilters 无该服务、服务键不存在）；点“安装”并重启后才可能拦截。本机在位键盘节点 {} 个，槽上限 {}{}",
                present_keyboard_nodes()
                    .map(|n| n.to_string())
                    .unwrap_or_else(|| "未知".to_string()),
                MAX_KEYBOARD,
                _slot_pressure_note()
            ),
            Self::SlotsExhausted { present } => format!(
                "Interception 驱动已加载，但本机在位键盘节点 {} 个已超过其键盘槽上限 {}；超出的设备不会出现在任何槽里，且占槽顺序随设备栈建立顺序变化，重装/重启都不解决。目标设备无法被稳定拦截",
                present, MAX_KEYBOARD
            ),
        }
    }

    /// 驱动是否**仍在内核里可枚举**。卸载核验只能用这个口径:
    /// `SlotsExhausted` 同样意味着驱动活着(只是槽位不够), 若按"是否就绪"判定就会把
    /// "槽位不足" 误读成 "已卸载干净"。
    pub(crate) fn is_loaded(&self) -> bool {
        matches!(self, Self::Ready | Self::SlotsExhausted { .. })
    }
}

pub(crate) fn driver_status() -> DriverStatus {
    if !deployed_assets_match() {
        return DriverStatus::AssetsMissing;
    }
    driver_status_from_api(&install_dir().join(API_FILE))
}

/// 判断当前 Interception API 是否已为指定 Raw Input HID 路径建立唯一目标槽位。
pub(crate) fn target_ready(raw_input_path: &str) -> Result<bool> {
    if raw_input_path.trim().is_empty() {
        return Err(anyhow!("目标 Raw Input 硬件 ID 为空"));
    }
    if !deployed_assets_match() {
        return Ok(false);
    }
    let api = Api::load(&install_dir().join(API_FILE))?;
    let context = unsafe { (api.create_context)() };
    if context.is_null() {
        return Ok(false);
    }
    let ready = resolve_target(&api, context, raw_input_path).is_ok();
    unsafe { (api.destroy_context)(context) };
    Ok(ready)
}

/// 输出 Raw Input 键盘路径与现有 Interception 槽位的只读关联信息。
/// 不设置 filter、不读取或发送按键、不改变设备栈；用于核实复合 HID 的真实路径差异。
pub(crate) fn diagnostic_report(
    target_filter: Option<&str>,
    keyboards: &[super::keyboard::KeyboardDevice],
) -> String {
    let filter = target_filter
        .map(str::trim)
        .filter(|value| !value.is_empty());
    let selected: Vec<&super::keyboard::KeyboardDevice> = keyboards
        .iter()
        .filter(|device| {
            filter.is_none_or(|value| {
                device.path.eq_ignore_ascii_case(value)
                    || device
                        .path
                        .to_ascii_uppercase()
                        .contains(&value.to_ascii_uppercase())
                    || device
                        .detail
                        .to_ascii_uppercase()
                        .contains(&value.to_ascii_uppercase())
                    || device
                        .parent_key
                        .to_ascii_uppercase()
                        .contains(&value.to_ascii_uppercase())
            })
        })
        .collect();
    let mut lines = vec![format!(
        "[INTERCEPTION] driver: {}",
        driver_status().detail()
    )];
    lines.push(format!(
        "[RAWINPUT] keyboards={} selected={} filter={}",
        keyboards.len(),
        selected.len(),
        filter.unwrap_or("<all>")
    ));
    for device in keyboards {
        let mark = if selected
            .iter()
            .any(|selected_device| selected_device.path == device.path)
        {
            " *"
        } else {
            ""
        };
        lines.push(format!(
            "[RAWINPUT]{} path={} | detail={} | parent={}",
            mark, device.path, device.detail, device.parent_key
        ));
    }
    if selected.is_empty() {
        lines.push("[INTERCEPTION] 未找到匹配筛选条件的 Raw Input 键盘".to_string());
    }
    if !deployed_assets_match() {
        lines.push("[INTERCEPTION] API 资产未部署，无法读取槽位".to_string());
        return lines.join("\n");
    }
    let api = match Api::load(&install_dir().join(API_FILE)) {
        Ok(api) => api,
        Err(error) => {
            lines.push(format!("[INTERCEPTION] API 加载失败: {}", error));
            return lines.join("\n");
        }
    };
    let context = unsafe { (api.create_context)() };
    if context.is_null() {
        lines.push("[INTERCEPTION] create_context 返回空，驱动尚未公开键盘槽位".to_string());
        return lines.join("\n");
    }
    let slots: Vec<(i32, String, Option<HardwareId>)> = (1..=MAX_KEYBOARD)
        .map(|device| {
            let raw = api.hardware_id(context, device);
            let parsed = HardwareId::parse(&raw);
            (device, raw, parsed)
        })
        .collect();
    for (slot, raw, parsed) in &slots {
        if raw.is_empty() {
            continue;
        }
        let detail = parsed.as_ref().map_or_else(
            || "<无法解析>".to_string(),
            |id| {
                format!(
                    "full={} | vid_pid={} | instance={}",
                    id.full, id.vid_pid, id.instance
                )
            },
        );
        lines.push(format!("[SLOT {}] raw={} | {}", slot, raw, detail));
    }
    for device in selected {
        let outcome = resolve_target_from_slots(&slots, &device.path)
            .map(|slot| format!("slot {}", slot))
            .unwrap_or_else(|error| format!("FAIL: {}", error));
        lines.push(format!("[MATCH] path={} => {}", device.path, outcome));
    }
    unsafe { (api.destroy_context)(context) };
    lines.join("\n")
}

/// 在设备栈重启后等待目标 HID 路径重新出现在唯一 Interception 槽位。
pub(crate) fn wait_target_ready(raw_input_path: &str, timeout: Duration) -> Result<()> {
    let started = Instant::now();
    loop {
        if target_ready(raw_input_path)? {
            return Ok(());
        }
        if started.elapsed() >= timeout {
            return Err(anyhow!(
                "等待目标设备重新出现在 Interception 槽位超时（{} 秒）",
                timeout.as_secs()
            ));
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// 键盘类 GUID: 上层过滤驱动列表挂在它的类键上。
const KEYBOARD_CLASS_GUID: &str = "{4D36E96B-E325-11CE-BFC1-08002BE10318}";
/// 官方安装器注册的驱动服务名(键盘侧)。
const DRIVER_SERVICE: &str = "keyboard";

/// 注册表层面确认"官方安装器确实把驱动注册进去了"。只读, 无副作用。
///
/// 两个条件都要满足才算注册完成: 键盘类 `UpperFilters` 里出现驱动服务名, 且该服务键存在。
/// 缺任一个就说明安装器没真正写进去(例如 UAC 被取消), 那属于失败而不是"待重启"。
pub(crate) fn registration_present() -> bool {
    upper_filters(KEYBOARD_CLASS_GUID)
        .iter()
        .any(|name| name.eq_ignore_ascii_case(DRIVER_SERVICE))
        && service_key_exists(DRIVER_SERVICE)
}

fn service_key_exists(service: &str) -> bool {
    let path = format!(r"SYSTEM\CurrentControlSet\Services\{}", service);
    let wide: Vec<u16> = path.encode_utf16().chain(Some(0)).collect();
    let mut key = HKEY::default();
    let status = unsafe {
        RegOpenKeyExW(
            HKEY_LOCAL_MACHINE,
            PCWSTR(wide.as_ptr()),
            Some(0),
            KEY_READ,
            &mut key,
        )
    };
    if status != ERROR_SUCCESS {
        return false;
    }
    unsafe { _ = RegCloseKey(key) };
    true
}

/// 读取设备类的 `UpperFilters`(REG_MULTI_SZ)。读不到就返回空表, 绝不臆测已注册。
fn upper_filters(class_guid: &str) -> Vec<String> {
    let path = format!(r"SYSTEM\CurrentControlSet\Control\Class\{}", class_guid);
    let wide: Vec<u16> = path.encode_utf16().chain(Some(0)).collect();
    let mut key = HKEY::default();
    let status = unsafe {
        RegOpenKeyExW(
            HKEY_LOCAL_MACHINE,
            PCWSTR(wide.as_ptr()),
            Some(0),
            KEY_READ,
            &mut key,
        )
    };
    if status != ERROR_SUCCESS {
        return Vec::new();
    }
    let name: Vec<u16> = "UpperFilters".encode_utf16().chain(Some(0)).collect();
    let mut bytes = 0u32;
    let status = unsafe {
        RegQueryValueExW(
            key,
            PCWSTR(name.as_ptr()),
            None,
            None,
            None,
            Some(&mut bytes),
        )
    };
    if status != ERROR_SUCCESS || bytes as usize % size_of::<u16>() != 0 {
        unsafe { _ = RegCloseKey(key) };
        return Vec::new();
    }
    let mut raw = vec![0u16; bytes as usize / size_of::<u16>()];
    let status = unsafe {
        RegQueryValueExW(
            key,
            PCWSTR(name.as_ptr()),
            None,
            None,
            Some(raw.as_mut_ptr().cast()),
            Some(&mut bytes),
        )
    };
    unsafe { _ = RegCloseKey(key) };
    if status != ERROR_SUCCESS {
        return Vec::new();
    }
    raw.truncate(bytes as usize / size_of::<u16>());
    raw.split(|unit| *unit == 0)
        .filter(|part| !part.is_empty())
        .map(String::from_utf16_lossy)
        .collect()
}

/// 用已部署或暂存的官方 API 进行无副作用探测，供卸载后确认本应用拥有的驱动已离开。
pub(crate) fn driver_status_from_api(api_path: &Path) -> DriverStatus {
    if !api_path.is_file() {
        return DriverStatus::AssetsMissing;
    }
    let api = match Api::load(api_path) {
        Ok(api) => api,
        Err(error) => return DriverStatus::ApiUnavailable(error.to_string()),
    };
    let context = unsafe { (api.create_context)() };
    if context.is_null() {
        return unloaded_status();
    }
    let found = (1..=MAX_KEYBOARD).any(|device| !api.hardware_id(context, device).is_empty());
    unsafe { (api.destroy_context)(context) };
    if !found {
        return unloaded_status();
    }
    // 驱动活着 ≠ 目标设备一定能被拦截: 在位键盘节点超过槽上限时, 谁占到槽取决于设备栈建立
    // 顺序, 排在上限之外的设备根本不会出现在任何槽里。必须单独分档, 否则 UI 会报"已可用"
    // 而用户永远等不到拦截生效。
    match present_keyboard_nodes() {
        Some(present) if present > MAX_KEYBOARD as usize => DriverStatus::SlotsExhausted { present },
        _ => DriverStatus::Ready,
    }
}

/// 驱动没响应时区分两种情形: 注册表已注册 ⇒ 只差重启; 没注册 ⇒ 压根没装上。
/// ★不要把这两种和"槽位不够"糊在一起★ 三者的用户动作完全不同(装 / 重启 / 减少键盘节点或换方案)。
fn unloaded_status() -> DriverStatus {
    if registration_present() {
        DriverStatus::PendingReboot
    } else {
        DriverStatus::NotInstalled
    }
}

/// 键盘类设备类 GUID(与 `KEYBOARD_CLASS_GUID` 同一个, 这里给二进制形式供 SetupAPI 枚举)。
const GUID_DEVCLASS_KEYBOARD: windows::core::GUID =
    windows::core::GUID::from_u128(0x4d36e96b_e325_11ce_bfc1_08002be10318);

/// 系统当前**在位**的键盘类设备节点数。Interception 只能给其中前 `MAX_KEYBOARD` 个分配槽,
/// 因此这个数字直接决定"目标设备有没有机会被拦截"。查不到返回 None(绝不用 0 冒充)。
///
/// ★为什么这个数会轻易超★ 一个复合 HID 设备的每个键盘顶层集合(`&COL0x`)都算独立键盘节点:
/// 本工程自己的 RP2040 固件就贡献 3 个, 一块带宏键的键盘再贡献 4 个, 加上真键盘与扫码器,
/// 一台普通开发机就能到 11 个 —— 这不是异常配置。
pub(crate) fn present_keyboard_nodes() -> Option<usize> {
    use windows::Win32::Devices::DeviceAndDriverInstallation::{
        DIGCF_PRESENT, SP_DEVINFO_DATA, SetupDiDestroyDeviceInfoList, SetupDiEnumDeviceInfo,
        SetupDiGetClassDevsW,
    };
    // SAFETY: 按类 GUID 取在位设备集合, 逐个枚举计数; 出口统一 Destroy。
    unsafe {
        let handle =
            SetupDiGetClassDevsW(Some(&GUID_DEVCLASS_KEYBOARD), None, None, DIGCF_PRESENT).ok()?;
        let mut count = 0usize;
        let mut data = SP_DEVINFO_DATA {
            cbSize: size_of::<SP_DEVINFO_DATA>() as u32,
            ..Default::default()
        };
        while SetupDiEnumDeviceInfo(handle, count as u32, &mut data).is_ok() {
            count += 1;
            if count > 4096 {
                break;
            }
        }
        let _ = SetupDiDestroyDeviceInfoList(handle);
        Some(count)
    }
}

fn _slot_pressure_note() -> String {
    match present_keyboard_nodes() {
        Some(present) if present > MAX_KEYBOARD as usize => {
            "；⚠ 已超上限，即使装上并重启，目标设备也不保证能占到槽位".to_string()
        }
        _ => String::new(),
    }
}

/// 运行时代码完整性状态位(`SystemCodeIntegrityInformation`)。
/// 这是"能不能自己写一个内核过滤驱动"的**决定性前提**: HVCI/KMCI 开着时, 自签名或测试签名的
/// 内核二进制加载会被拒, 老的交叉签名驱动也常因页对齐/自修改代码被拒。查不到返回 None。
pub(crate) fn code_integrity_options() -> Option<u32> {
    #[repr(C)]
    struct CodeIntegrity {
        length: u32,
        options: u32,
    }
    type NtQuery = unsafe extern "system" fn(u32, *mut c_void, u32, *mut u32) -> i32;
    // SAFETY: 只读查询, 缓冲为本地固定大小结构; 失败一律落 None。
    unsafe {
        let ntdll = LoadLibraryW(windows::core::w!("ntdll.dll")).ok()?;
        let symbol = GetProcAddress(ntdll, PCSTR(c"NtQuerySystemInformation".as_ptr().cast()));
        let query: NtQuery = symbol.map(|s| std::mem::transmute_copy(&s))?;
        let mut info = CodeIntegrity {
            length: size_of::<CodeIntegrity>() as u32,
            options: 0,
        };
        let mut returned = 0u32;
        // 103 = SystemCodeIntegrityInformation。
        let status = query(
            103,
            (&mut info as *mut CodeIntegrity).cast(),
            size_of::<CodeIntegrity>() as u32,
            &mut returned,
        );
        let _ = FreeLibrary(ntdll);
        (status >= 0).then_some(info.options)
    }
}

/// HVCI(管理程序强制代码完整性)内核侧是否**正在生效**。
pub(crate) fn hvci_enforced() -> Option<bool> {
    code_integrity_options().map(|options| options & 0x400 != 0)
}

/// 测试签名模式是否开启。HVCI 生效时它对内核驱动加载不再管用, 两者要一起看。
pub(crate) fn test_signing_enabled() -> Option<bool> {
    code_integrity_options().map(|options| options & 0x02 != 0)
}

/// 一句话说明"这台机器允不允许自写内核过滤驱动"。给 UI 与日志共用, 避免两处口径漂移。
pub(crate) fn kernel_driver_feasibility() -> String {
    match (hvci_enforced(), test_signing_enabled()) {
        (Some(true), _) => {
            "HVCI(管理程序强制代码完整性)正在生效：自签名/测试签名的内核驱动一律加载失败，老的交叉签名过滤驱动也常被拒；不关闭 HVCI 就无法自写内核拦截驱动".to_string()
        }
        (Some(false), Some(true)) => {
            "HVCI 未生效且测试签名已开启：可加载自签名内核驱动（该模式会降低系统完整性保护）".to_string()
        }
        (Some(false), Some(false)) => {
            "HVCI 未生效、测试签名未开启：内核驱动需 Microsoft 签名（WHQL/attestation）才能加载".to_string()
        }
        _ => "无法读取运行时代码完整性状态，内核驱动可行性未知".to_string(),
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(crate) struct KeyStroke {
    pub(crate) code: u16,
    pub(crate) state: u16,
    pub(crate) information: u32,
}

impl KeyStroke {
    pub(crate) fn is_key_down(self) -> bool {
        self.state & 0x0001 == 0
    }

    pub(crate) fn is_extended(self) -> bool {
        self.state & 0x0002 != 0
    }
}

/// 一次目标设备过滤会话。API 的过滤谓词由库回调，故进程内严格只允许一个活动会话。
pub(crate) struct Capture {
    api: Api,
    context: *mut c_void,
    device: i32,
    /// 被过滤且仍处于按下态的键。会话停止/切目标前必须逐一合成释放，不能留下卡键。
    held: std::collections::HashMap<(u16, u16), KeyStroke>,
    stopped: bool,
}

// 上下文在创建后立即移动到唯一的捕获线程，之后仅该线程访问；停止也由同一线程收尾。
unsafe impl Send for Capture {}

impl Capture {
    pub(crate) fn open(raw_input_path: &str) -> Result<Self> {
        if !cfg!(target_arch = "x86_64") {
            return Err(anyhow!("Interception 目标设备模式只支持 x64 上位机"));
        }
        if !deployed_assets_match() {
            return Err(anyhow!(
                "Interception 运行资产未完整部署，已降级且不会拦截输入"
            ));
        }
        let api = Api::load(&install_dir().join(API_FILE))?;
        let context = unsafe { (api.create_context)() };
        if context.is_null() {
            // 已注册但未加载是最常见的一种失败, 直接把"重启才生效"讲清楚, 而不是只报 create_context 失败。
            return Err(anyhow!(
                "{}，已降级且不会拦截输入",
                unloaded_status().detail()
            ));
        }
        let device = match resolve_target(&api, context, raw_input_path) {
            Ok(device) => device,
            Err(error) => {
                unsafe { (api.destroy_context)(context) };
                return Err(error);
            }
        };
        if ACTIVE_DEVICE
            .compare_exchange(0, device, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            unsafe { (api.destroy_context)(context) };
            return Err(anyhow!(
                "已有 Interception 目标设备会话正在运行，拒绝并发拦截"
            ));
        }
        unsafe { (api.set_filter)(context, target_predicate, FILTER_KEY_ALL) };
        Ok(Self {
            api,
            context,
            device,
            held: std::collections::HashMap::new(),
            stopped: false,
        })
    }

    /// 只会取得目标槽的按键；返回 `None` 表示本轮超时。收到的键不自动回送，调用方可按
    /// 扫码器语义消费它，从而只吞掉已精确匹配的目标设备。
    pub(crate) fn receive(&mut self, timeout_ms: u32) -> Result<Option<KeyStroke>> {
        if self.stopped {
            return Ok(None);
        }
        let device = unsafe { (self.api.wait_with_timeout)(self.context, timeout_ms) };
        if device == 0 {
            return Ok(None);
        }
        if device != self.device {
            return Err(anyhow!(
                "Interception 返回了未匹配设备槽 {}，已安全停止",
                device
            ));
        }
        let mut stroke = KeyStroke::default();
        let got = unsafe {
            (self.api.receive)(
                self.context,
                device,
                (&mut stroke as *mut KeyStroke).cast(),
                1,
            )
        };
        if got != 1 {
            return Err(anyhow!(
                "Interception 未能接收目标设备键盘数据（返回 {}）",
                got
            ));
        }
        let key = (stroke.code, stroke.state & 0x0006);
        if stroke.is_key_down() {
            self.held.insert(key, stroke);
        } else {
            self.held.remove(&key);
        }
        Ok(Some(stroke))
    }

    /// 仅允许向已验证的目标槽回送，避免跨设备注入。
    pub(crate) fn send(&mut self, stroke: KeyStroke) -> Result<()> {
        if self.stopped {
            return Err(anyhow!("Interception 会话已停止，拒绝发送"));
        }
        let sent = unsafe {
            (self.api.send)(
                self.context,
                self.device,
                (&stroke as *const KeyStroke).cast(),
                1,
            )
        };
        if sent != 1 {
            return Err(anyhow!(
                "Interception 未能向目标设备回送键盘数据（返回 {}）",
                sent
            ));
        }
        Ok(())
    }

    fn release_held(&mut self) {
        let held: Vec<KeyStroke> = self.held.drain().map(|(_, stroke)| stroke).collect();
        for down in held {
            let release = KeyStroke {
                state: down.state | 0x0001,
                ..down
            };
            if let Err(error) = self.send(release) {
                log::warn!("Interception 目标设备合成按键释放失败: {}", error);
            }
        }
    }

    pub(crate) fn stop(&mut self) {
        if self.stopped {
            return;
        }
        // 过滤仍生效时先给每个已吞掉的按下补一个目标设备释放；随后才撤销过滤/销毁上下文。
        self.release_held();
        unsafe { (self.api.set_filter)(self.context, target_predicate, 0) };
        ACTIVE_DEVICE.store(0, Ordering::SeqCst);
        unsafe { (self.api.destroy_context)(self.context) };
        self.context = std::ptr::null_mut();
        self.stopped = true;
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.stop();
    }
}

static ACTIVE_DEVICE: AtomicI32 = AtomicI32::new(0);

unsafe extern "system" fn target_predicate(device: i32) -> i32 {
    i32::from(device == ACTIVE_DEVICE.load(Ordering::SeqCst))
}

fn resolve_target(api: &Api, context: *mut c_void, raw_input_path: &str) -> Result<i32> {
    let candidates: Vec<(i32, String, Option<HardwareId>)> = (1..=MAX_KEYBOARD)
        .map(|device| {
            let raw = api.hardware_id(context, device);
            let parsed = HardwareId::parse(&raw);
            (device, raw, parsed)
        })
        .collect();
    resolve_target_from_slots(&candidates, raw_input_path)
}

fn resolve_target_from_slots(
    slots: &[(i32, String, Option<HardwareId>)],
    raw_input_path: &str,
) -> Result<i32> {
    let target = HardwareId::parse(raw_input_path)
        .ok_or_else(|| anyhow!("目标 Raw Input 硬件 ID 无效，已降级且不会拦截输入"))?;
    let candidates: Vec<(i32, &HardwareId)> = slots
        .iter()
        .filter_map(|(device, _, id)| id.as_ref().map(|id| (*device, id)))
        .collect();
    let exact: Vec<i32> = candidates
        .iter()
        .filter(|(_, id)| id.full == target.full)
        .map(|(device, _)| *device)
        .collect();
    match exact.as_slice() {
        [device] => return Ok(*device),
        [] => {}
        _ => {
            return Err(anyhow!(
                "同一完整硬件 ID 映射到多个 Interception 设备槽，已降级且不会拦截输入"
            ));
        }
    }
    let same_instance: Vec<i32> = candidates
        .iter()
        .filter(|(_, id)| id.vid_pid == target.vid_pid && id.instance == target.instance)
        .map(|(device, _)| *device)
        .collect();
    match same_instance.as_slice() {
        [device] => return Ok(*device),
        [] => {}
        _ => {
            return Err(anyhow!(
                "同一硬件实例映射到多个 Interception 设备槽，已降级且不会拦截输入"
            ));
        }
    }
    let same_model: Vec<i32> = candidates
        .iter()
        .filter(|(_, id)| id.vid_pid == target.vid_pid)
        .map(|(device, _)| *device)
        .collect();
    match same_model.as_slice() {
        [device] => Ok(*device),
        // ★"槽里没有它"最常见的成因不是不兼容, 而是槽位被占满★ 必须把在位键盘节点数与上限
        // 一起报出来, 否则用户只能看到"未找到匹配", 从而去怀疑设备本身不被支持。
        [] => Err(anyhow!(
            "所选硬件 ID 不在任何 Interception 键盘槽内（已公开槽 {}，在位键盘节点 {}，槽上限 {}）；{}已降级且不会拦截输入",
            slots.iter().filter(|(_, raw, _)| !raw.is_empty()).count(),
            present_keyboard_nodes()
                .map(|n| n.to_string())
                .unwrap_or_else(|| "未知".to_string()),
            MAX_KEYBOARD,
            match present_keyboard_nodes() {
                Some(present) if present > MAX_KEYBOARD as usize =>
                    "在位键盘节点已超过槽上限，占槽顺序随设备栈建立顺序变化，重装/重启不解决；",
                _ => "",
            }
        )),
        _ => Err(anyhow!(
            "同 VID/PID 的多个设备无法唯一匹配所选硬件，已降级且不会拦截输入"
        )),
    }
}

struct HardwareId {
    full: String,
    vid_pid: String,
    instance: String,
}

impl HardwareId {
    fn parse(source: &str) -> Option<Self> {
        let mut full = source.trim().to_ascii_uppercase();
        full = full
            .strip_prefix(r"\\?\")
            .or_else(|| full.strip_prefix(r"\\.\"))
            .unwrap_or(&full)
            .to_string();
        full = full
            .split("#{")
            .next()
            .unwrap_or(&full)
            .replace('#', "\\")
            .trim_matches('\\')
            .to_string();
        let segments: Vec<&str> = full.split('\\').filter(|part| !part.is_empty()).collect();
        let hardware = segments
            .iter()
            .find(|part| part.contains("VID_") && part.contains("PID_"))?;
        let mut fields: Vec<&str> = hardware
            .split('&')
            .filter(|field| field.starts_with("VID_") || field.starts_with("PID_"))
            .collect();
        fields.sort_unstable();
        let vid_pid = fields.join("&");
        let instance = segments.last()?.to_string();
        (!vid_pid.is_empty() && !instance.is_empty()).then_some(Self {
            full,
            vid_pid,
            instance,
        })
    }
}

type CreateContext = unsafe extern "system" fn() -> *mut c_void;
type DestroyContext = unsafe extern "system" fn(*mut c_void);
type SetFilter = unsafe extern "system" fn(*mut c_void, unsafe extern "system" fn(i32) -> i32, u16);
type WaitWithTimeout = unsafe extern "system" fn(*mut c_void, u32) -> i32;
type Receive = unsafe extern "system" fn(*mut c_void, i32, *mut c_void, u32) -> i32;
type SendStroke = unsafe extern "system" fn(*mut c_void, i32, *const c_void, u32) -> i32;
type HardwareIdFn = unsafe extern "system" fn(*mut c_void, i32, *mut c_void, u32) -> u32;

struct Api {
    module: HMODULE,
    create_context: CreateContext,
    destroy_context: DestroyContext,
    set_filter: SetFilter,
    wait_with_timeout: WaitWithTimeout,
    receive: Receive,
    send: SendStroke,
    hardware_id_fn: HardwareIdFn,
}

impl Api {
    fn load(path: &Path) -> Result<Self> {
        let wide: Vec<u16> = path.as_os_str().encode_wide().chain(Some(0)).collect();
        let module = unsafe { LoadLibraryW(PCWSTR(wide.as_ptr())) }
            .map_err(|error| anyhow!("加载 {} 失败：{}", path.display(), error))?;
        let result = (|| unsafe {
            Ok(Self {
                module,
                create_context: load_symbol(module, b"interception_create_context\0")?,
                destroy_context: load_symbol(module, b"interception_destroy_context\0")?,
                set_filter: load_symbol(module, b"interception_set_filter\0")?,
                wait_with_timeout: load_symbol(module, b"interception_wait_with_timeout\0")?,
                receive: load_symbol(module, b"interception_receive\0")?,
                send: load_symbol(module, b"interception_send\0")?,
                hardware_id_fn: load_symbol(module, b"interception_get_hardware_id\0")?,
            })
        })();
        if result.is_err() {
            unsafe { _ = FreeLibrary(module) };
        }
        result
    }

    fn hardware_id(&self, context: *mut c_void, device: i32) -> String {
        let mut raw = [0u16; 512];
        let written = unsafe {
            (self.hardware_id_fn)(
                context,
                device,
                raw.as_mut_ptr().cast(),
                (raw.len() * size_of::<u16>()) as u32,
            )
        } as usize;
        let end = raw
            .iter()
            .position(|value| *value == 0)
            .unwrap_or(written.min(raw.len()));
        String::from_utf16_lossy(&raw[..end])
    }
}

impl Drop for Api {
    fn drop(&mut self) {
        unsafe { _ = FreeLibrary(self.module) };
    }
}

unsafe fn load_symbol<T>(module: HMODULE, name: &'static [u8]) -> Result<T> {
    let symbol = unsafe { GetProcAddress(module, PCSTR(name.as_ptr())) }.ok_or_else(|| {
        anyhow!(
            "Interception API 缺少导出 {}",
            String::from_utf8_lossy(name)
        )
    })?;
    Ok(unsafe { std::mem::transmute_copy(&symbol) })
}
