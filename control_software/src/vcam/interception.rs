//! Interception v1.0.1 的最小运行时封装。
//!
//! 本模块只经官方 `interception.dll` API 与键盘过滤驱动通信；不会注册或改变
//! Raw Input。未能证明目标设备唯一、驱动可用或 API/系统兼容时，调用方必须降级，
//! 不得宣称目标设备已被拦截。

use std::ffi::c_void;
use std::os::windows::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicI32, Ordering};

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
const MAX_KEYBOARD: i32 = 10;
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
    DriverUnavailable,
}

impl DriverStatus {
    pub(crate) fn detail(&self) -> String {
        match self {
            Self::Ready => "Interception 驱动与 x64 API 已核验可用".to_string(),
            Self::AssetsMissing => "Interception x64 API/安装器/许可证未完整部署".to_string(),
            Self::ApiUnavailable(error) => format!(
                "Interception API 无法加载（可能为签名、系统版本或位宽不兼容）：{}",
                error
            ),
            Self::PendingReboot => {
                "Interception 驱动已注册为键盘类上层过滤，但需重启电脑后才会加载生效".to_string()
            }
            Self::DriverUnavailable => {
                "Interception 驱动未就绪或系统未公开任何可用键盘槽；不会拦截输入".to_string()
            }
        }
    }

    /// 是否已可真正吞键。`PendingReboot` 明确**不**算就绪, 拦截功能在重启前一律降级。
    pub(crate) fn is_ready(&self) -> bool {
        matches!(self, Self::Ready)
    }
}

pub(crate) fn driver_status() -> DriverStatus {
    if !deployed_assets_match() {
        return DriverStatus::AssetsMissing;
    }
    driver_status_from_api(&install_dir().join(API_FILE))
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
    if found {
        DriverStatus::Ready
    } else {
        unloaded_status()
    }
}

/// 驱动没响应时区分两种情形: 注册表已注册 ⇒ 只差重启; 没注册 ⇒ 真的不可用。
fn unloaded_status() -> DriverStatus {
    if registration_present() {
        DriverStatus::PendingReboot
    } else {
        DriverStatus::DriverUnavailable
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
    let target = HardwareId::parse(raw_input_path)
        .ok_or_else(|| anyhow!("目标 Raw Input 硬件 ID 无效，已降级且不会拦截输入"))?;
    let candidates: Vec<(i32, HardwareId)> = (1..=MAX_KEYBOARD)
        .filter_map(|device| {
            let text = api.hardware_id(context, device);
            HardwareId::parse(&text).map(|id| (device, id))
        })
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
        [] => Err(anyhow!(
            "未找到与所选硬件 ID 匹配的 Interception 键盘设备，已降级且不会拦截输入"
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
