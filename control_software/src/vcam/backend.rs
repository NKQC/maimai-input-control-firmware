//! Windows Media Foundation 虚拟摄像头后端与媒体源安装。
//!
//! 所有 COM 对象只由 UI 线程持有；Frame Server 自己在别的进程加载 DLL，二者只通过
//! `share` 的命名内存交换帧，避免把 COM 指针错误发送到键盘捕获线程。

use std::path::{Path, PathBuf};

use anyhow::{anyhow, Result};
use windows::core::{HSTRING, PCWSTR};
use windows::Win32::Foundation::{ERROR_FILE_NOT_FOUND, ERROR_SUCCESS};
use windows::Win32::Media::MediaFoundation::{
    MFCreateVirtualCamera, MFStartup, IMFVirtualCamera, MFVirtualCameraAccess_AllUsers,
    MFVirtualCameraAccess_CurrentUser, MFVirtualCameraLifetime_Session,
    MFVirtualCameraType_SoftwareCameraSource, MFSTARTUP_FULL, MF_VERSION,
};
use windows::Win32::System::Registry::{
    HKEY, HKEY_LOCAL_MACHINE, KEY_READ, KEY_WOW64_32KEY, KEY_WOW64_64KEY, REG_EXPAND_SZ,
    REG_SZ, REG_VALUE_TYPE, RegCloseKey, RegOpenKeyExW, RegQueryValueExW,
};


const _CLSID: &str = "{B7C5F1A2-3D64-4E8B-9A11-2F6C8D0E4A73}";
const _FRIENDLY_NAME: &str = "mai2control Virtual Camera";
const _DLL_NAME: &str = "mai2vcam_source.dll";

mod embedded {
    include!(concat!(env!("OUT_DIR"), "/vcam_embedded.rs"));
}

/// UI 生命周期内持有此对象；Drop 同样尝试 Stop/Remove，防止异常退出路径遗留会话相机。
pub struct VirtualCamera {
    _camera: IMFVirtualCamera,
    _access_name: &'static str,
    // CoInitializeEx 的成功调用必须在同一 UI 线程配对释放，避免每次开关相机累积 COM 引用计数。
    _com_initialized: bool,
}

impl VirtualCamera {
    /// 优先尝试 AllUsers，满足管理员安装场景；因权限拒绝则明确降级到 CurrentUser。
    pub fn start() -> Result<Self> {
        unsafe {
            // windows 0.62 的 CoInitializeEx 直接返回 HRESULT(不是 Result)。
            // S_FALSE 表示本线程已初始化过, 属正常情况, 只有真失败才算错。
            let hr = windows::Win32::System::Com::CoInitializeEx(
                None,
                windows::Win32::System::Com::COINIT_APARTMENTTHREADED,
            );
            if hr.is_err() {
                return Err(anyhow!("CoInitializeEx(STA) 失败: HRESULT 0x{:08X}", hr.0 as u32));
            }
        }
        unsafe { MFStartup(MF_VERSION, MFSTARTUP_FULL) }.map_err(|error| anyhow!(
            "MFStartup 失败: HRESULT 0x{:08X}",
            error.code().0 as u32
        ))?;
        // ★降级判断必须看 Start 而不是 Create★: 实测非管理员下 MFCreateVirtualCamera(AllUsers)
        // 照样返回 S_OK, 真正的拒绝发生在 IMFVirtualCamera::Start(0x80070005 E_ACCESSDENIED)。
        // 原来只在 Create 失败时才退回 CurrentUser, 于是非提权运行必然死在 Start 上。
        // 现在把"创建 + 启动"作为一次完整尝试, 任一步失败就清理掉这台再换 CurrentUser 重试。
        let mut last_error = None;
        let mut started = None;
        for (access, access_name) in [
            (MFVirtualCameraAccess_AllUsers, "AllUsers"),
            (MFVirtualCameraAccess_CurrentUser, "CurrentUser"),
        ] {
            match Self::_try_start(access) {
                Ok(camera) => {
                    started = Some((camera, access_name));
                    break;
                }
                Err(error) => {
                    log::info!("虚拟摄像头: {} 方式不可用: {}", access_name, error);
                    last_error = Some(error);
                }
            }
        }
        let (camera, access_name) = started.ok_or_else(|| {
            last_error.unwrap_or_else(|| anyhow!("MFCreateVirtualCamera 未返回可用实例"))
        })?;
        log::info!("虚拟摄像头: Media Foundation 已启动（{}）", access_name);
        Ok(Self { _camera: camera, _access_name: access_name, _com_initialized: true })
    }

    pub fn access_name(&self) -> &'static str {
        self._access_name
    }

    pub fn stop(self) {
        drop(self);
    }

    /// 一次完整尝试: 创建 + Start。失败时把已创建的实例清干净, 不给系统留半死的会话相机。
    fn _try_start(
        access: windows::Win32::Media::MediaFoundation::MFVirtualCameraAccess,
    ) -> Result<IMFVirtualCamera> {
        let camera = Self::_create(access).map_err(|error| {
            anyhow!("MFCreateVirtualCamera 失败: HRESULT 0x{:08X}", error.code().0 as u32)
        })?;
        if let Err(error) = unsafe { camera.Start(None) } {
            let hr = error.code().0 as u32;
            unsafe {
                _ = camera.Remove();
                _ = camera.Shutdown();
            }
            // 0x80070005 = E_ACCESSDENIED: AllUsers 需要管理员; 0x80040154 = 类未注册(DLL 没装好)。
            let hint = match hr {
                0x8007_0005 => "(需要管理员权限)",
                0x8004_0154 => "(媒体源 DLL 未注册, 先点安装)",
                _ => "",
            };
            return Err(anyhow!("IMFVirtualCamera::Start 失败: HRESULT 0x{:08X}{}", hr, hint));
        }
        Ok(camera)
    }

    fn _create(
        access: windows::Win32::Media::MediaFoundation::MFVirtualCameraAccess,
    ) -> windows::core::Result<IMFVirtualCamera> {
        unsafe {
            MFCreateVirtualCamera(
                MFVirtualCameraType_SoftwareCameraSource,
                MFVirtualCameraLifetime_Session,
                access,
                &HSTRING::from(_FRIENDLY_NAME),
                &HSTRING::from(_CLSID),
                None,
            )
        }
    }

    fn _stop(&self) {
        unsafe {
            if let Err(error) = self._camera.Stop() {
                log::warn!("虚拟摄像头: Stop 失败: {}", error);
            }
            if let Err(error) = self._camera.Remove() {
                log::warn!("虚拟摄像头: Remove 失败: {}", error);
            }
        }
    }
}

impl Drop for VirtualCamera {
    fn drop(&mut self) {
        self._stop();
        if self._com_initialized {
            unsafe { windows::Win32::System::Com::CoUninitialize() };
        }
    }
}

pub fn embedded_available() -> bool {
    embedded::EMBEDDED
}

pub fn registration_status() -> String {
    if !embedded_available() {
        return "本次构建未内置媒体源 DLL".to_string();
    }
    match registered_dll_path() {
        Ok(Some(path)) => format!("已安装 ({})", path.display()),
        Ok(None) => "未安装（尚未注册媒体源 DLL）".to_string(),
        Err(error) => format!("注册状态读取失败: {}", error),
    }
}

pub fn is_registered() -> Result<bool> {
    Ok(!_registered_dll_paths()?.is_empty())
}

/// 发起管理员安装，并始终独立读取 HKLM 核验；绝不把“命令已发出”当作成功。
pub fn install() -> Result<String> {
    if !embedded_available() {
        return Err(anyhow!("本次构建未内置媒体源 DLL，先执行 cargo build -p mai2vcam-source 后重建上位机"));
    }
    let (staged, target) = _stage_payload(&_install_path())?;
    let dir = target.parent().ok_or_else(|| anyhow!("DLL 安装目录无效"))?;
    let copy = if staged == target {
        String::new()
    } else {
        format!(
            "copy /Y {} {} >nul & if errorlevel 1 exit /b %errorlevel% & ",
            _cmd_quote(&staged),
            _cmd_quote(&target),
        )
    };
    let command = format!(
        "(if not exist {} mkdir {}) & icacls {} /grant *S-1-1-0:(RX) /T /C >nul & if errorlevel 1 exit /b %errorlevel% & {}regsvr32.exe /s {}",
        _cmd_quote(dir),
        _cmd_quote(dir),
        _cmd_quote(dir),
        copy,
        _cmd_quote(&target),
    );
    let exit = _run_elevated("cmd.exe", &format!("/d /s /c \"{}\"", command))?;
    if exit != 0 {
        return Err(anyhow!(
            "管理员安装命令返回错误码 {}(regsvr32/icacls 失败); DLL 目标路径 {}",
            exit,
            target.display()
        ));
    }
    let registered = _registered_dll_paths()?;
    if let Some(path) = registered.iter().find(|entry| _same_path(&entry._path, &target)) {
        return Ok(format!("已安装 ({})", path._path.display()));
    }
    if registered.is_empty() {
        return Err(anyhow!("安装命令已发出，但注册表核验未通过；请在 UAC 完成后重试安装"));
    }
    Err(anyhow!(
        "安装命令已发出，但注册表路径不匹配；期望路径: {}; 注册表实际值: {}",
        target.display(),
        _describe_registry_paths(&registered),
    ))
}

/// 发起管理员反注册，并独立核验 CLSID 键已经消失。
pub fn uninstall() -> Result<String> {
    let target = _install_path();
    let exit = _run_elevated("regsvr32.exe", &format!("/u /s {}", _cmd_quote(&target)))?;
    if exit != 0 {
        return Err(anyhow!("管理员反注册命令返回错误码 {}", exit));
    }
    let registered = _registered_dll_paths()?;
    if registered.is_empty() {
        Ok("未安装（已核验注册表项消失）".to_string())
    } else {
        Err(anyhow!(
            "卸载命令已发出，但注册表项仍存在: {}",
            _describe_registry_paths(&registered),
        ))
    }
}

/// 保留对外的单一路径接口；优先返回 64 位视图，所有调用方实际核验则读取两种视图。
pub fn registered_dll_path() -> Result<Option<PathBuf>> {
    Ok(_registered_dll_paths()?.into_iter().next().map(|entry| entry._path))
}

struct _RegisteredDllPath {
    _view: &'static str,
    _raw_value: String,
    _path: PathBuf,
}

/// 64 位 Frame Server 只看 64 位视图，而 32 位注册工具可能写入 32 位视图；两边都读取，
/// 让安装核验能接受任一实际匹配项，并在不匹配时完整报告已有的值。
fn _registered_dll_paths() -> Result<Vec<_RegisteredDllPath>> {
    let mut paths = Vec::new();
    let mut errors = Vec::new();
    for (view, view_name) in [
        (KEY_WOW64_64KEY, "64 位视图"),
        (KEY_WOW64_32KEY, "32 位视图"),
    ] {
        match _registered_dll_path_in_view(view, view_name) {
            Ok(Some(path)) => paths.push(path),
            Ok(None) => {}
            Err(error) => errors.push(error),
        }
    }
    if !paths.is_empty() || errors.is_empty() {
        return Ok(paths);
    }
    Err(anyhow!(
        "两个注册表视图均无法读取媒体源 CLSID: {}",
        errors.into_iter().map(|error| error.to_string()).collect::<Vec<_>>().join("; "),
    ))
}

fn _registered_dll_path_in_view(
    view: windows::Win32::System::Registry::REG_SAM_FLAGS,
    view_name: &'static str,
) -> Result<Option<_RegisteredDllPath>> {
    let key_name = _wide(&format!("SOFTWARE\\Classes\\CLSID\\{}\\InprocServer32", _CLSID));
    let mut key = HKEY::default();
    let status = unsafe {
        RegOpenKeyExW(
            HKEY_LOCAL_MACHINE,
            PCWSTR(key_name.as_ptr()),
            Some(0),
            KEY_READ | view,
            &mut key,
        )
    };
    if status == ERROR_FILE_NOT_FOUND {
        return Ok(None);
    }
    if status != ERROR_SUCCESS {
        return Err(anyhow!("打开 {} 媒体源注册表键失败: {}", view_name, status.0));
    }
    let result = _read_registry_dll_value(key, view_name);
    unsafe { _ = RegCloseKey(key) };
    result
}

fn _read_registry_dll_value(key: HKEY, view_name: &'static str) -> Result<Option<_RegisteredDllPath>> {
    let mut kind = REG_VALUE_TYPE(0);
    let mut bytes = 0u32;
    let status = unsafe {
        RegQueryValueExW(key, PCWSTR::null(), None, Some(&mut kind), None, Some(&mut bytes))
    };
    if status == ERROR_FILE_NOT_FOUND {
        return Ok(None);
    }
    if status != ERROR_SUCCESS {
        return Err(anyhow!("读取 {} 媒体源注册表长度失败: {}", view_name, status.0));
    }
    if bytes % std::mem::size_of::<u16>() as u32 != 0 {
        return Err(anyhow!("{} 媒体源注册表值不是有效 UTF-16 字节长度", view_name));
    }
    let mut raw = vec![0u16; bytes as usize / std::mem::size_of::<u16>()];
    let status = unsafe {
        RegQueryValueExW(
            key,
            PCWSTR::null(),
            None,
            Some(&mut kind),
            Some(raw.as_mut_ptr().cast()),
            Some(&mut bytes),
        )
    };
    if status != ERROR_SUCCESS {
        return Err(anyhow!("读取 {} 媒体源注册表值失败: {}", view_name, status.0));
    }
    let words = bytes as usize / std::mem::size_of::<u16>();
    let raw_value = String::from_utf16_lossy(&raw[..words]).trim_end_matches('\0').to_string();
    let value = match kind {
        REG_SZ => raw_value.clone(),
        REG_EXPAND_SZ => _expand_environment_strings(&raw_value).map_err(|error| {
            anyhow!("展开 {} 媒体源 REG_EXPAND_SZ 值失败: {}", view_name, error)
        })?,
        _ => return Err(anyhow!("{} 媒体源注册表值类型不受支持: {}", view_name, kind.0)),
    };
    Ok((!value.is_empty()).then(|| _RegisteredDllPath {
        _view: view_name,
        _raw_value: raw_value,
        _path: PathBuf::from(value),
    }))
}

/// REG_EXPAND_SZ 可写成 `%ProgramData%\\...`；必须让 Win32 展开后再参与比较，
/// 否则它与同一目录的绝对目标路径会被误报为不同。
fn _expand_environment_strings(value: &str) -> Result<String> {
    let source = _wide(value);
    let required = unsafe { ExpandEnvironmentStringsW(source.as_ptr(), std::ptr::null_mut(), 0) };
    if required == 0 {
        return Err(anyhow!("{}", std::io::Error::last_os_error()));
    }
    let mut expanded = vec![0u16; required as usize];
    let written = unsafe {
        ExpandEnvironmentStringsW(source.as_ptr(), expanded.as_mut_ptr(), expanded.len() as u32)
    };
    if written == 0 || written > expanded.len() as u32 {
        return Err(anyhow!("{}", std::io::Error::last_os_error()));
    }
    Ok(String::from_utf16_lossy(&expanded[..written as usize])
        .trim_end_matches('\0')
        .to_string())
}

fn _describe_registry_paths(paths: &[_RegisteredDllPath]) -> String {
    paths
        .iter()
        .map(|entry| {
            let expanded = entry._path.display().to_string();
            if entry._raw_value == expanded {
                format!("{}: {}", entry._view, entry._raw_value)
            } else {
                format!("{}: {}（展开后: {}）", entry._view, entry._raw_value, expanded)
            }
        })
        .collect::<Vec<_>>()
        .join("; ")
}

fn _install_path() -> PathBuf {
    let base = std::env::var_os("ProgramData")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(r"C:\ProgramData"));
    base.join("mai2control").join(_DLL_NAME)
}

/// 先尝试直接写 machine-wide 目录；目标 DLL 被占用时改用编号路径，避免 Frame Server
/// 持有旧 DLL 时阻断注册。其余权限不足情形保留用户可写的暂存副本，交给 runas 复制。
fn _stage_payload(primary: &Path) -> Result<(PathBuf, PathBuf)> {
    let target = if let Some(parent) = primary.parent() {
        if std::fs::create_dir_all(parent).is_ok() {
            match std::fs::write(primary, embedded::BYTES) {
                Ok(()) => return Ok((primary.to_path_buf(), primary.to_path_buf())),
                Err(error) if _is_sharing_or_permission_error(&error) => {
                    let fallback = _numbered_install_path(primary);
                    if std::fs::write(&fallback, embedded::BYTES).is_ok() {
                        return Ok((fallback.clone(), fallback));
                    }
                    log::info!(
                        "媒体源 DLL {} 被占用，改由管理员复制到 {}",
                        primary.display(),
                        fallback.display(),
                    );
                    fallback
                }
                Err(_) => primary.to_path_buf(),
            }
        } else {
            primary.to_path_buf()
        }
    } else {
        primary.to_path_buf()
    };
    let staged = std::env::temp_dir().join(_DLL_NAME);
    std::fs::write(&staged, embedded::BYTES)
        .map_err(|error| anyhow!("无法写入媒体源暂存 DLL {}: {}", staged.display(), error))?;
    Ok((staged, target))
}

fn _is_sharing_or_permission_error(error: &std::io::Error) -> bool {
    error.raw_os_error() == Some(32) || error.kind() == std::io::ErrorKind::PermissionDenied
}

fn _numbered_install_path(primary: &Path) -> PathBuf {
    let parent = primary.parent().unwrap_or_else(|| Path::new(""));
    let stem = primary
        .file_stem()
        .and_then(|value| value.to_str())
        .unwrap_or("mai2vcam_source");
    let extension = primary
        .extension()
        .and_then(|value| value.to_str())
        .unwrap_or("dll");
    for sequence in 2.. {
        let candidate = parent.join(format!("{}.{}.{}", stem, sequence, extension));
        if !candidate.exists() {
            return candidate;
        }
    }
    unreachable!("无界编号范围必须能产生候选 DLL 路径")
}

/// 以管理员执行一条命令并★等它真正跑完★, 返回退出码。
///
/// 原来用 ShellExecuteW 发出即返回, 紧接着就去读注册表核验 —— 提权进程连启动都没完成,
/// 核验必然失败(实测: 明明有管理员权限也一直报"注册表核验未通过")。这里改用 ShellExecuteExW
/// 拿到进程句柄后等待, 才能既知道命令结束、又拿到真实退出码。
fn _run_elevated(file: &str, parameters: &str) -> Result<u32> {
    use windows::Win32::Foundation::{CloseHandle, WAIT_OBJECT_0};
    use windows::Win32::System::Threading::{GetExitCodeProcess, WaitForSingleObject};
    use windows::Win32::UI::Shell::{ShellExecuteExW, SEE_MASK_NOCLOSEPROCESS, SHELLEXECUTEINFOW};

    let file = _wide(file);
    let parameters = _wide(parameters);
    let verb = _wide("runas");
    let mut info = SHELLEXECUTEINFOW {
        cbSize: std::mem::size_of::<SHELLEXECUTEINFOW>() as u32,
        fMask: SEE_MASK_NOCLOSEPROCESS,
        lpVerb: PCWSTR(verb.as_ptr()),
        lpFile: PCWSTR(file.as_ptr()),
        lpParameters: PCWSTR(parameters.as_ptr()),
        nShow: 0, // SW_HIDE: 安装是后台动作, 不必闪一个黑窗
        ..Default::default()
    };
    // SAFETY: info 各宽字符串在调用期间存活; 成功后 hProcess 由本函数负责关闭。
    unsafe { ShellExecuteExW(&mut info) }
        .map_err(|e| anyhow!("管理员命令未启动(可能取消了 UAC): {}", e))?;
    if info.hProcess.is_invalid() {
        return Err(anyhow!("管理员命令已启动但拿不到进程句柄, 无法确认结果"));
    }
    // 60s 上限: 正常 regsvr32 亚秒级完成; 卡住也不能把 UI 永久挂死。
    let waited = unsafe { WaitForSingleObject(info.hProcess, 60_000) };
    let mut code: u32 = u32::MAX;
    if waited == WAIT_OBJECT_0 {
        unsafe { _ = GetExitCodeProcess(info.hProcess, &mut code) };
    }
    unsafe { _ = CloseHandle(info.hProcess) };
    if waited != WAIT_OBJECT_0 {
        return Err(anyhow!("管理员命令超时未结束(等待结果 {:?})", waited));
    }
    Ok(code)
}

fn _wide(text: &str) -> Vec<u16> {
    text.encode_utf16().chain(Some(0)).collect()
}

fn _cmd_quote(path: &Path) -> String {
    format!("\"{}\"", path.display())
}

/// 同一个 DLL 路径可因注册工具、Win32 API 或文件系统显示形式不同而产生不同文本：
/// 引号/空白、`/`、重复或末尾 `\\`、`\\?\\` 扩展前缀、8.3 名称及规范绝对路径都不能当成未注册。
/// 每一步都只消除表示差异；无法解析的文件系统路径保留为字符串规范化结果，不会放宽匹配。
fn _same_path(left: &Path, right: &Path) -> bool {
    _normalize_path(left) == _normalize_path(right)
}

fn _normalize_path(path: &Path) -> String {
    // 注册表编辑器或命令行可能留下包裹引号及首尾空白；它们不是路径本身。
    let text = _normalize_path_text(path.to_string_lossy().as_ref());
    // GetLongPathNameW 将已有文件的 8.3 短名转换成长名；失败时保持原值以免屏蔽真实错误。
    let text = _long_path_name(&text).unwrap_or(text);
    // canonicalize 同时生成绝对路径并解析真实文件系统别名；不存在或不可访问时回退字符串比较。
    let text = std::fs::canonicalize(&text)
        .map(|canonical| canonical.to_string_lossy().into_owned())
        .unwrap_or(text);
    // canonicalize 在 Windows 常引入 `\\?\\`，所以规范绝对路径后还要再清除显示形式差异。
    _normalize_path_text(&text).to_lowercase()
}

fn _normalize_path_text(path: &str) -> String {
    let mut value = path.trim();
    while let Some(unquoted) = value.strip_prefix('"').and_then(|value| value.strip_suffix('"')) {
        value = unquoted.trim();
    }
    // Win32 接受两类分隔符；统一为反斜杠才能消除注册值与本地路径的表示差异。
    let value = value.replace('/', "\\");
    // 扩展路径前缀是 Win32 的传输细节；UNC 形式须恢复为普通 UNC 前缀，不能留下 `UNC` 目录名。
    let value = if let Some(unc) = value.strip_prefix("\\\\?\\UNC\\") {
        format!("\\\\{}", unc)
    } else {
        value.strip_prefix("\\\\?\\").unwrap_or(&value).to_string()
    };
    _trim_trailing_separator(_collapse_separators(&value))
}

fn _collapse_separators(path: &str) -> String {
    let mut chars = path.chars().peekable();
    let mut normalized = String::with_capacity(path.len());
    let mut previous_separator = false;
    // UNC 路径开头的两个反斜杠有语义，保留它们，其余重复分隔符才可以折叠。
    if path.starts_with("\\\\") {
        normalized.push_str("\\\\");
        while chars.next_if_eq(&'\\').is_some() {}
    }
    for character in chars {
        if character == '\\' {
            if previous_separator {
                continue;
            }
            previous_separator = true;
        } else {
            previous_separator = false;
        }
        normalized.push(character);
    }
    normalized
}

fn _trim_trailing_separator(mut path: String) -> String {
    // 仅保留盘符根的尾部分隔符，避免把 `C:\\` 误变成相对盘符路径 `C:`。
    while path.ends_with('\\')
        && !(path.len() == 3 && path.as_bytes().get(1) == Some(&b':'))
        && path != "\\"
    {
        path.pop();
    }
    path
}

// 直接调用 kernel32，避免为两个很小的 Win32 API 扩大 windows crate 的 feature 集合。
// (extern block 不接受 doc comment，故用普通注释)
#[link(name = "kernel32")]
unsafe extern "system" {
    fn ExpandEnvironmentStringsW(source: *const u16, destination: *mut u16, size: u32) -> u32;
    fn GetLongPathNameW(short_path: *const u16, long_path: *mut u16, size: u32) -> u32;
}

fn _long_path_name(path: &str) -> Option<String> {
    let short_path = _wide(path);
    let required = unsafe { GetLongPathNameW(short_path.as_ptr(), std::ptr::null_mut(), 0) };
    if required == 0 {
        return None;
    }
    let mut long_path = vec![0u16; required as usize];
    let written = unsafe {
        GetLongPathNameW(short_path.as_ptr(), long_path.as_mut_ptr(), long_path.len() as u32)
    };
    if written == 0 || written >= long_path.len() as u32 {
        return None;
    }
    Some(String::from_utf16_lossy(&long_path[..written as usize]))
}
