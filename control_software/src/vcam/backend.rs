//! 虚拟摄像头部署和 Media Foundation 会话生命周期。
//!
//! DirectShow 过滤器供旧式消费端枚举，Media Foundation source DLL 供 Frame Server 枚举。
//! 两者共用同一命名帧队列，但注册、部署与生命周期必须各自独立核验。

use std::path::{Path, PathBuf};
use std::sync::{Mutex, TryLockError};

use anyhow::{Result, anyhow};

use super::interception;
use windows::Win32::Foundation::{ERROR_FILE_NOT_FOUND, ERROR_NOT_FOUND, ERROR_SUCCESS};
use windows::Win32::Media::MediaFoundation::{
    IMFVirtualCamera, MF_E_NOT_FOUND, MF_E_SHUTDOWN, MFCreateVirtualCamera,
    MFVirtualCameraAccess_CurrentUser, MFVirtualCameraLifetime_System,
    MFVirtualCameraType_SoftwareCameraSource,
};
use windows::Win32::System::Registry::{
    HKEY, HKEY_LOCAL_MACHINE, KEY_READ, KEY_WOW64_32KEY, KEY_WOW64_64KEY, REG_EXPAND_SZ,
    REG_SAM_FLAGS, REG_SZ, REG_VALUE_TYPE, RegCloseKey, RegOpenKeyExW, RegQueryValueExW,
};
use windows::core::{HRESULT, PCWSTR, w};

/// DirectShow 过滤器 CLSID（与 `vcam_common.cpp` 一致）。
const DS_CLSID: &str = "{6E5A1C74-2F83-4C9B-9D1E-7A4B0F3C58E2}";
const VIDEO_INPUT_CATEGORY: &str = "{860BB310-5D01-11D0-BD3B-00A0C911CE86}";
const INSTANCE_NAME: &str = "mai2control Virtual Camera";

mod embedded {
    include!(concat!(env!("OUT_DIR"), "/vcam_embedded.rs"));
}

fn _stage_interception_assets(staging: &Path, steps: &mut Vec<String>) -> Result<()> {
    if !interception::embedded_available() {
        return Err(anyhow!(
            "本次构建未内置完整 Interception v1.0.1 x64 API、官方安装器与 LGPL-3.0 许可证，拒绝声称目标设备可拦截"
        ));
    }
    let directory = interception::install_dir();
    steps.push(format!(
        "(if not exist {dir} mkdir {dir})",
        dir = _cmd_quote(&directory)
    ));
    for (name, bytes) in interception::deployment_assets() {
        let source = staging.join(name);
        std::fs::write(&source, bytes).map_err(|error| {
            anyhow!(
                "写入 Interception 暂存资产 {} 失败：{}",
                source.display(),
                error
            )
        })?;
        steps.push(format!(
            "copy /Y {source} {target} >nul",
            source = _cmd_quote(&source),
            target = _cmd_quote(&directory.join(name)),
        ));
        steps.push("if errorlevel 1 exit /b 40".to_string());
    }
    Ok(())
}

fn _interception_assets_clean() -> bool {
    !interception::deployment_assets()
        .iter()
        .any(|(name, _)| interception::install_dir().join(name).is_file())
        && !interception::install_dir()
            .join(interception::OWNER_FILE)
            .is_file()
}

/// 安装/卸载会修改同一组文件和 HKLM 键，必须串行执行。
static _DEPLOY: Mutex<()> = Mutex::new(());

fn _deploy_guard(action: &str) -> Result<std::sync::MutexGuard<'static, ()>> {
    match _DEPLOY.try_lock() {
        Ok(guard) => Ok(guard),
        Err(TryLockError::Poisoned(poisoned)) => Ok(poisoned.into_inner()),
        Err(TryLockError::WouldBlock) => Err(anyhow!(
            "已有虚拟摄像头部署操作正在进行中，请等它结束后再{}",
            action
        )),
    }
}

struct _DeployDll {
    _label: &'static str,
    _dll: &'static str,
    _bytes: &'static [u8],
    _view: REG_SAM_FLAGS,
    _regsvr: &'static str,
    _clsid: &'static str,
    _dshow: bool,
}

/// 安装顺序固定为 DS x64 → DS x86；卸载按此顺序反向执行。
///
/// ★只有 DirectShow 一种实现★(与 OBS Virtual Camera 同路子): Media Foundation 虚拟相机
/// (`MFCreateVirtualCamera`)实测在客户端里不可用, 且它注册的是系统帧服务器设备, 残留后会变成
/// "看得见、打不开"的幽灵相机, 已整体撤除, 只在 `uninstall` 里保留一次性清理。
fn _items() -> [_DeployDll; 2] {
    [
        _DeployDll {
            _label: "DirectShow 64 位",
            _dll: "mai2vcam_dshow64.dll",
            _bytes: embedded::BYTES_DS_X64,
            _view: KEY_WOW64_64KEY,
            _regsvr: r"%windir%\System32\regsvr32.exe",
            _clsid: DS_CLSID,
            _dshow: true,
        },
        _DeployDll {
            _label: "DirectShow 32 位",
            _dll: "mai2vcam_dshow32.dll",
            _bytes: embedded::BYTES_DS_X86,
            _view: KEY_WOW64_32KEY,
            _regsvr: r"%windir%\SysWOW64\regsvr32.exe",
            _clsid: DS_CLSID,
            _dshow: true,
        },
    ]
}

/// 两个位宽缺任一个都不允许安装：少 32 位那份就表现为"32 位应用里看不到这个摄像头"。
pub fn embedded_available() -> bool {
    _items().iter().all(|item| !item._bytes.is_empty())
}

/// 所有消费端都能读取的机器级部署目录。
pub fn install_dir() -> PathBuf {
    std::env::var_os("ProgramData")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(r"C:\ProgramData"))
        .join("mai2control")
        .join("vcam")
}

struct _Registered {
    _label: &'static str,
    _expected: PathBuf,
    _clsid_key: bool,
    _inproc: Option<PathBuf>,
    _listed: bool,
    _dshow: bool,
    _dll: bool,
    _old: bool,
}

impl _Registered {
    fn _installed(&self) -> bool {
        self._clsid_key
            && (!self._dshow || self._listed)
            && self._dll
            && self
                ._inproc
                .as_ref()
                .is_some_and(|path| _same_path(path, &self._expected))
    }

    fn _absent(&self) -> bool {
        !self._clsid_key && !self._inproc.is_some() && (!self._dshow || !self._listed)
    }

    fn _files_clean(&self) -> bool {
        !self._dll && !self._old
    }
}

fn _scan() -> Vec<_Registered> {
    let directory = install_dir();
    _items()
        .into_iter()
        .map(|item| {
            let expected = directory.join(item._dll);
            _Registered {
                _label: item._label,
                _clsid_key: _key_exists(item._view, &format!("CLSID\\{}", item._clsid)),
                _inproc: _inproc_path(item._view, item._clsid).unwrap_or(None),
                _listed: item._dshow && _instance_listed(item._view),
                _dshow: item._dshow,
                _dll: expected.is_file(),
                _old: expected.with_extension("dll.old").is_file(),
                _expected: expected,
            }
        })
        .collect()
}

fn _same_path(left: &Path, right: &Path) -> bool {
    let normalize = |path: &Path| {
        _normalize_path_text(&path.display().to_string())
            .to_lowercase()
            .replace('/', "\\")
    };
    normalize(left) == normalize(right)
}

/// 给 UI 的部署状态。摄像头与目标设备吞键分别核验；驱动未确认时绝不表述为“已拦截”。
pub fn registration_status() -> String {
    let camera = if !embedded_available() {
        "本次构建未内置虚拟摄像头 DLL（需 DirectShow x64 与 x86 两份产物）".to_string()
    } else {
        let scanned = _scan();
        let missing: Vec<&str> = scanned
            .iter()
            .filter(|entry| !entry._installed())
            .map(|entry| entry._label)
            .collect();
        if missing.is_empty() {
            format!("摄像头已安装（{}）", install_dir().display())
        } else if missing.len() == scanned.len() {
            "摄像头未安装（系统中没有本摄像头）".to_string()
        } else {
            format!("摄像头部分安装（{} 未通过核验）", missing.join("、"))
        }
    };
    // ★键盘过滤驱动已退居备用★ 吞键现在由「WinUSB 改绑」承担(见 `driver_pkg`), 它没有键盘槽上限、
    // 不引入内核二进制。所以这里只留一句结论 —— 完整诊断(注册与否/待重启/槽位是否超限/内核代码
    // 完整性)一律落日志。旧版把整段诊断摊在界面上, 用户据此去排查了根本不存在的兼容性问题。
    let status = interception::driver_status();
    log::info!("虚拟摄像头: 键盘过滤驱动诊断: {}", status.detail());
    // 只有 `Ready` 算可用: `SlotsExhausted` 是"驱动活着但目标拿不到槽", 那对拦截等于不可用。
    let filter = if matches!(status, interception::DriverStatus::Ready) {
        "键盘过滤驱动可用（已被 WinUSB 改绑取代，一般无需使用）"
    } else {
        "键盘过滤驱动未就绪（改用下方 WinUSB 改绑，不需要它）"
    };
    format!("{}；{}", camera, filter)
}

/// 对选定 Raw Input HID 设备执行精确设备栈热重启，并确认 Interception 目标槽位恢复。
pub fn hot_restart_selected_device(raw_input_path: &str) -> Result<String> {
    if !embedded_available() || !interception::deployed_assets_match() {
        return Err(anyhow!(
            "Interception 驱动注册/运行资产未通过核验，拒绝热重启"
        ));
    }
    if !interception::registration_present() {
        return Err(anyhow!("Interception 驱动尚未完成注册，拒绝热重启"));
    }
    let instance_id = hid_instance_id(raw_input_path)?;
    if interception::target_ready(raw_input_path)? {
        return Ok("目标设备已在 Interception 槽位就绪，无需热重启".to_string());
    }
    let exit = run_elevated(
        "pnputil.exe",
        &format!("/restart-device {}", cmd_quote_text(&instance_id)),
    )
    .map_err(|error| anyhow!("目标设备热重启未完成：{}", error))?;
    if exit != 0 {
        return Err(anyhow!(
            "pnputil /restart-device 失败，退出码 {}（{}）",
            exit,
            _exit_reason(exit)
        ));
    }
    interception::wait_target_ready(raw_input_path, std::time::Duration::from_secs(20))?;
    Ok(format!(
        "已热重启选定设备并确认 Interception 槽位就绪：{}",
        instance_id
    ))
}

/// Raw Input HID 路径 → HID 设备实例 ID，含命令行注入字符拒绝。
/// WinUSB 改绑路径复用同一份校验，避免出现第二套宽松的路径解析。
pub(crate) fn hid_instance_id(raw_input_path: &str) -> Result<String> {
    let path = raw_input_path.trim();
    if path.is_empty() || path.len() > 4096 {
        return Err(anyhow!("拒绝空或过长的 Raw Input HID 路径"));
    }
    if path.chars().any(|ch| {
        matches!(
            ch,
            '|' | '<' | '>' | '"' | '\'' | '`' | ';' | '%' | '\r' | '\n'
        )
    }) {
        return Err(anyhow!("Raw Input HID 路径含危险命令字符，已拒绝"));
    }
    let upper = path.to_ascii_uppercase();
    let body = upper
        .strip_prefix(r"\\?\HID#")
        .or_else(|| upper.strip_prefix(r"\\.\HID#"))
        .ok_or_else(|| anyhow!("目标路径不是 HID Raw Input 设备路径"))?;
    let body = body
        .split("#{")
        .next()
        .ok_or_else(|| anyhow!("目标 HID 路径缺少设备实例"))?;
    let instance = format!("HID\\{}", body.replace('#', "\\"));
    if instance.len() <= 4 {
        return Err(anyhow!("目标 HID 路径无法转换为设备实例 ID"));
    }
    Ok(instance)
}

pub(crate) fn cmd_quote_text(value: &str) -> String {
    format!("\"{}\"", value)
}

/// 启动前的摄像头注册核验。目标设备过滤另由 `interception::Capture::open` 精确验证，
/// 因而不会把未选目标的 Raw Input 旁路功能误判为驱动拦截成功。
pub fn is_registered() -> Result<bool> {
    if !embedded_available() {
        return Ok(false);
    }
    Ok(_scan().iter().all(|entry| entry._installed()))
}

/// 部署摄像头与 Interception 运行资产；仅在 API 明确证明驱动不可用时才调用官方安装器。
/// 已存在且可用、但并非本应用安装的驱动不会被标记为本应用所有。
pub fn install() -> Result<String> {
    let _deploy = _deploy_guard("安装")?;
    if !embedded_available() {
        return Err(anyhow!(
            "本次构建未内置完整 DLL：先用 MSBuild 构建 DirectShow 的 Release|x64 与 Release|Win32，再重建上位机"
        ));
    }
    if !interception::embedded_available() {
        return Err(anyhow!(
            "本次构建未内置经核验的 Interception v1.0.1 x64 API、官方安装器与 LGPL-3.0 许可证"
        ));
    }
    let directory = install_dir();
    let staging = std::env::temp_dir().join("mai2control_vcam");
    std::fs::create_dir_all(&staging)
        .map_err(|error| anyhow!("创建暂存目录 {} 失败：{}", staging.display(), error))?;

    let mut steps = vec![format!(
        "(if not exist {dir} mkdir {dir})",
        dir = _cmd_quote(&directory)
    )];
    _stage_interception_assets(&staging, &mut steps)?;
    for (index, item) in _items().iter().enumerate() {
        let staged = staging.join(item._dll);
        std::fs::write(&staged, item._bytes)
            .map_err(|error| anyhow!("写入暂存 DLL {} 失败：{}", staged.display(), error))?;
        let target = directory.join(item._dll);
        steps.push(format!(
            "(copy /Y {staged} {target} >nul || (move /Y {target} {old} >nul 2>&1 & copy /Y {staged} {target} >nul))",
            staged = _cmd_quote(&staged),
            target = _cmd_quote(&target),
            old = _cmd_quote(&target.with_extension("dll.old")),
        ));
        steps.push(format!("if errorlevel 1 exit /b {}", 20 + index));
        steps.push(format!(
            "{regsvr} /s {target}",
            regsvr = item._regsvr,
            target = _cmd_quote(&target),
        ));
        steps.push(format!("if errorlevel 1 exit /b {}", 30 + index));
    }
    steps.push(format!(
        "icacls {dir} /grant *S-1-1-0:(RX) /T /C >nul 2>&1",
        dir = _cmd_quote(&directory)
    ));

    let exit = _run_elevated_steps(&steps)?;
    if exit != 0 {
        return Err(anyhow!(
            "部署命令以退出码 {} 结束（{}）；现状：{}",
            exit,
            _exit_reason(exit),
            _describe(&_scan()),
        ));
    }
    let verified = _scan();
    if !verified.iter().all(|entry| entry._installed()) {
        return Err(anyhow!(
            "摄像头注册命令已执行（退出码 0），但注册核验未通过：{}",
            _describe(&verified),
        ));
    }
    if !interception::deployed_assets_match() {
        return Err(anyhow!(
            "Interception 运行资产复制后未通过逐字节核验，拒绝继续安装驱动"
        ));
    }

    match interception::driver_status() {
        interception::DriverStatus::Ready => Ok(registration_status()),
        // 已注册但没加载 ⇒ 上一次安装其实成功了, 只差重启; 不再重跑安装器。
        interception::DriverStatus::PendingReboot => Ok(registration_status()),
        interception::DriverStatus::NotInstalled => {
            // 只有 API 已加载、且注册表实测确认"从来没装上"时，才执行归档内唯一的官方安装器。
            // ★SlotsExhausted 不走这里★ 槽位是该驱动的结构性上限，重装不解决，落到下面的错误分支。
            let marker = staging.join(interception::OWNER_FILE);
            std::fs::write(&marker, interception::owner_marker())
                .map_err(|error| anyhow!("写入 Interception 所有权标记失败：{}", error))?;
            let target = interception::install_dir();
            let installer = target.join(interception::INSTALLER_FILE);
            let steps = vec![
                format!("{} /install", _cmd_quote(&installer)),
                "if errorlevel 1 exit /b 50".to_string(),
                format!(
                    "copy /Y {marker} {target} >nul",
                    marker = _cmd_quote(&marker),
                    target = _cmd_quote(&target.join(interception::OWNER_FILE)),
                ),
                "if errorlevel 1 exit /b 51".to_string(),
            ];
            let exit = _run_elevated_steps(&steps)?;
            if exit != 0 {
                return Err(anyhow!(
                    "Interception 官方安装器以退出码 {} 结束（{}）；不会声称已拦截输入",
                    exit,
                    _exit_reason(exit),
                ));
            }
            match interception::driver_status() {
                // ★装完必然还没加载★ 键盘类上层过滤驱动只在开机重建设备栈时挂载, 因此首次安装
                // 后立刻探测一定拿不到设备槽。这**不是**失败: 注册表已确认注册完成, 报"待重启"并
                // 返回成功, 由 registration_status() 如实告知重启前不会拦截。
                interception::DriverStatus::Ready | interception::DriverStatus::PendingReboot => {
                    Ok(registration_status())
                }
                status => Err(anyhow!(
                    "Interception 安装器已返回成功，但驱动注册未在注册表落地：{}；不会声称已拦截输入",
                    status.detail()
                )),
            }
        }
        status => Err(anyhow!(
            "Interception 运行时核验失败：{}；不会声称已拦截输入",
            status.detail()
        )),
    }
}

fn _exit_reason(exit: u32) -> String {
    let items = _items();
    match exit {
        20..=21 => format!("复制 {} DLL 失败", items[(exit - 20) as usize]._label),
        30..=31 => format!(
            "注册 {} DLL 失败（regsvr32）",
            items[(exit - 30) as usize]._label
        ),
        40 => "复制 Interception API/安装器/许可证失败".to_string(),
        50 => "Interception 官方安装器失败".to_string(),
        51 => "写入 Interception 所有权标记失败".to_string(),
        _ => "请检查管理员命令执行结果".to_string(),
    }
}

/// 反向反注册并清理本应用资产。只有存在完整所有权标记时才调用官方驱动卸载，
/// 因而绝不移除本应用安装前就存在的 Interception 驱动。
pub fn uninstall() -> Result<String> {
    let _deploy = _deploy_guard("卸载")?;
    _remove_legacy_mf_camera();
    let directory = install_dir();
    let interception_dir = interception::install_dir();
    let owned_driver = interception::owner_marker_matches();
    if owned_driver
        && !interception_dir
            .join(interception::INSTALLER_FILE)
            .is_file()
    {
        return Err(anyhow!(
            "检测到本应用 Interception 所有权标记，但官方安装器缺失；为避免错误移除驱动，已拒绝卸载"
        ));
    }
    let staging = std::env::temp_dir().join("mai2control_vcam");
    std::fs::create_dir_all(&staging)
        .map_err(|error| anyhow!("创建卸载核验暂存目录 {} 失败：{}", staging.display(), error))?;
    let probe_api = staging.join("interception-uninstall-probe.dll");
    if owned_driver {
        std::fs::copy(interception_dir.join("interception.dll"), &probe_api)
            .map_err(|error| anyhow!("暂存 Interception API 以核验卸载结果失败：{}", error))?;
    }

    let mut steps = Vec::new();
    for item in _items().iter().rev() {
        let target = directory.join(item._dll);
        steps.push(format!(
            "{regsvr} /u /s {target}",
            regsvr = item._regsvr,
            target = _cmd_quote(&target),
        ));
    }
    if owned_driver {
        steps.push(format!(
            "{} /uninstall",
            _cmd_quote(&interception_dir.join(interception::INSTALLER_FILE))
        ));
        steps.push("if errorlevel 1 exit /b 50".to_string());
    }
    for item in _items().iter().rev() {
        let target = directory.join(item._dll);
        steps.push(format!(
            "del /q {target} >nul 2>&1",
            target = _cmd_quote(&target)
        ));
        steps.push(format!(
            "del /q {old} >nul 2>&1",
            old = _cmd_quote(&target.with_extension("dll.old")),
        ));
    }
    for (name, _) in interception::deployment_assets() {
        steps.push(format!(
            "del /q {target} >nul 2>&1",
            target = _cmd_quote(&interception_dir.join(name)),
        ));
    }
    steps.push(format!(
        "del /q {target} >nul 2>&1",
        target = _cmd_quote(&interception_dir.join(interception::OWNER_FILE)),
    ));
    steps.push(format!(
        "rmdir {dir} >nul 2>&1",
        dir = _cmd_quote(&interception_dir)
    ));

    let exit = _run_elevated_steps(&steps)?;
    if exit != 0 {
        return Err(anyhow!(
            "卸载命令以退出码 {} 结束（{}）；现状：{}",
            exit,
            _exit_reason(exit),
            _describe(&_scan())
        ));
    }
    let verified = _scan();
    let camera_clean = verified
        .iter()
        .all(|entry| entry._absent() && entry._files_clean());
    if !camera_clean || !_interception_assets_clean() {
        return Err(anyhow!(
            "卸载命令已执行（退出码 0），但仍有残留：{}；Interception 资产已清除={}",
            _describe(&verified),
            _interception_assets_clean(),
        ));
    }
    if owned_driver {
        if interception::driver_status_from_api(&probe_api).is_loaded() {
            return Err(anyhow!(
                "官方 Interception 卸载器已返回成功，但驱动仍可枚举；不会声称卸载完成"
            ));
        }
        Ok(
            "未安装（已核验摄像头注册/文件清除，且仅移除了本应用安装的 Interception 驱动）"
                .to_string(),
        )
    } else {
        Ok(
            "未安装（已核验摄像头注册/文件清除；已保留非本应用安装的 Interception 驱动）"
                .to_string(),
        )
    }
}

fn _describe(entries: &[_Registered]) -> String {
    entries
        .iter()
        .map(|entry| {
            format!(
                "{}: CLSID键={} InprocServer32={} 类别登记={} DLL={} 残留.old={}",
                entry._label,
                if entry._clsid_key { "有" } else { "无" },
                entry
                    ._inproc
                    .as_ref()
                    .map(|path| {
                        let marker = if _same_path(path, &entry._expected) {
                            ""
                        } else {
                            "(非预期路径)"
                        };
                        format!("{}{}", path.display(), marker)
                    })
                    .unwrap_or_else(|| "(无)".to_string()),
                if entry._dshow {
                    if entry._listed { "有" } else { "无" }
                } else {
                    "不适用"
                },
                if entry._dll { "在" } else { "无" },
                if entry._old { "有" } else { "无" },
            )
        })
        .collect::<Vec<_>>()
        .join("；")
}

fn _key_exists(view: REG_SAM_FLAGS, sub_path: &str) -> bool {
    match _open(view, sub_path) {
        Ok(Some(key)) => {
            unsafe { _ = RegCloseKey(key) };
            true
        }
        _ => false,
    }
}

fn _inproc_path(view: REG_SAM_FLAGS, clsid: &str) -> Result<Option<PathBuf>> {
    let Some(key) = _open(view, &format!("CLSID\\{}\\InprocServer32", clsid))? else {
        return Ok(None);
    };
    let value = _read_string(key, None);
    unsafe { _ = RegCloseKey(key) };
    Ok(value?.map(|value| PathBuf::from(_normalize_path_text(&value))))
}

fn _instance_listed(view: REG_SAM_FLAGS) -> bool {
    let path = format!(
        "CLSID\\{}\\Instance\\{}",
        VIDEO_INPUT_CATEGORY, INSTANCE_NAME
    );
    let Ok(Some(key)) = _open(view, &path) else {
        return false;
    };
    let registered_clsid = _read_string(key, Some("CLSID")).ok().flatten();
    unsafe { _ = RegCloseKey(key) };
    registered_clsid.is_some_and(|value| value.eq_ignore_ascii_case(DS_CLSID))
}

fn _open(view: REG_SAM_FLAGS, sub_path: &str) -> Result<Option<HKEY>> {
    let name = _wide(&format!("SOFTWARE\\Classes\\{}", sub_path));
    let mut key = HKEY::default();
    let status = unsafe {
        RegOpenKeyExW(
            HKEY_LOCAL_MACHINE,
            PCWSTR(name.as_ptr()),
            Some(0),
            KEY_READ | view,
            &mut key,
        )
    };
    if status == ERROR_FILE_NOT_FOUND {
        return Ok(None);
    }
    if status != ERROR_SUCCESS {
        return Err(anyhow!("打开注册表键 {} 失败：{}", sub_path, status.0));
    }
    Ok(Some(key))
}

fn _read_string(key: HKEY, name: Option<&str>) -> Result<Option<String>> {
    let name_wide = name.map(_wide);
    let name_ptr = name_wide
        .as_ref()
        .map_or(PCWSTR::null(), |value| PCWSTR(value.as_ptr()));
    let mut kind = REG_VALUE_TYPE(0);
    let mut bytes = 0u32;
    let status =
        unsafe { RegQueryValueExW(key, name_ptr, None, Some(&mut kind), None, Some(&mut bytes)) };
    if status == ERROR_FILE_NOT_FOUND {
        return Ok(None);
    }
    if status != ERROR_SUCCESS {
        return Err(anyhow!("读取注册表值长度失败：{}", status.0));
    }
    if bytes as usize % size_of::<u16>() != 0 {
        return Err(anyhow!("注册表值不是有效的 UTF-16 字节长度"));
    }
    let mut raw = vec![0u16; bytes as usize / size_of::<u16>()];
    let status = unsafe {
        RegQueryValueExW(
            key,
            name_ptr,
            None,
            Some(&mut kind),
            Some(raw.as_mut_ptr().cast()),
            Some(&mut bytes),
        )
    };
    if status != ERROR_SUCCESS {
        return Err(anyhow!("读取注册表值失败：{}", status.0));
    }
    let value = String::from_utf16_lossy(&raw[..bytes as usize / size_of::<u16>()])
        .trim_end_matches('\0')
        .to_string();
    let value = match kind {
        REG_SZ => value,
        REG_EXPAND_SZ => _expand_environment_strings(&value)?,
        _ => return Err(anyhow!("注册表值类型不受支持：{}", kind.0)),
    };
    Ok((!value.is_empty()).then_some(value))
}

fn _expand_environment_strings(value: &str) -> Result<String> {
    let source = _wide(value);
    let required = unsafe { ExpandEnvironmentStringsW(source.as_ptr(), std::ptr::null_mut(), 0) };
    if required == 0 {
        return Err(anyhow!("{}", std::io::Error::last_os_error()));
    }
    let mut expanded = vec![0u16; required as usize];
    let written = unsafe {
        ExpandEnvironmentStringsW(
            source.as_ptr(),
            expanded.as_mut_ptr(),
            expanded.len() as u32,
        )
    };
    if written == 0 || written > expanded.len() as u32 {
        return Err(anyhow!("{}", std::io::Error::last_os_error()));
    }
    Ok(String::from_utf16_lossy(&expanded[..written as usize])
        .trim_end_matches('\0')
        .to_string())
}

#[link(name = "kernel32")]
unsafe extern "system" {
    fn ExpandEnvironmentStringsW(source: *const u16, destination: *mut u16, size: u32) -> u32;
}

/// 打开旧版本注册过的 MF 虚拟相机(参数必须与当年创建时逐字一致, 否则拿不到同一个对象)。
/// 仅供卸载时的一次性清理使用 —— 当前实现不再注册任何 MF 设备。
fn _open_legacy_mf_camera() -> Result<IMFVirtualCamera> {
    // MFCreateVirtualCamera 是 COM 调用: 调用线程未初始化 COM 时直接返回 CO_E_NOTINITIALIZED。
    // 安装/卸载都可能跑在后台线程上, 故在这里就地保证一次(已初始化时返回 S_FALSE/RPC_E_CHANGED_MODE,
    // 两者都不影响后续调用, 因此不做提前返回)。
    unsafe {
        let _ = windows::Win32::System::Com::CoInitializeEx(
            None,
            windows::Win32::System::Com::COINIT_MULTITHREADED,
        );
    }
    unsafe {
        MFCreateVirtualCamera(
            MFVirtualCameraType_SoftwareCameraSource,
            MFVirtualCameraLifetime_System,
            MFVirtualCameraAccess_CurrentUser,
            w!("mai2control Virtual Camera"),
            w!("{B7C5F1A2-3D64-4E8B-9A11-2F6C8D0E4A73}"),
            None,
        )
    }
    .map_err(|error| anyhow!("创建 Media Foundation 虚拟相机失败：{}", error))
}

/// MF Frame Server 是否仍列出本相机。卸载后只用来确认幽灵设备已消失。
fn _mf_device_listed() -> Result<bool> {
    use windows::Win32::Media::MediaFoundation::{
        IMFActivate, MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID, MF_VERSION, MFCreateAttributes,
        MFEnumDeviceSources, MFSTARTUP_FULL, MFShutdown, MFStartup,
    };
    use windows::Win32::System::Com::CoTaskMemFree;

    unsafe { MFStartup(MF_VERSION, MFSTARTUP_FULL) }
        .map_err(|error| anyhow!("MFStartup 失败：{}", error))?;
    let listed = (|| -> Result<bool> {
        let mut attributes = None;
        unsafe { MFCreateAttributes(&mut attributes, 1) }
            .map_err(|error| anyhow!("MFCreateAttributes 失败：{}", error))?;
        let attributes = attributes.ok_or_else(|| anyhow!("MFCreateAttributes 未返回属性存储"))?;
        unsafe {
            attributes.SetGUID(
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID,
            )
        }
        .map_err(|error| anyhow!("设置 VIDCAP 枚举属性失败：{}", error))?;

        let mut raw: *mut Option<IMFActivate> = std::ptr::null_mut();
        let mut count = 0u32;
        unsafe { MFEnumDeviceSources(&attributes, &mut raw, &mut count) }
            .map_err(|error| anyhow!("MFEnumDeviceSources 失败：{}", error))?;
        // 返回数组由 CoTaskMemAlloc 分配：所有权先移进 Vec，保证未匹配项也按 COM 规则释放。
        let activates: Vec<Option<IMFActivate>> = unsafe {
            (0..count as usize)
                .map(|index| std::ptr::read(raw.add(index)))
                .collect()
        };
        unsafe { CoTaskMemFree(Some(raw.cast())) };
        // MF 会给 FriendlyName 追加本地化后缀，只能按稳定基名包含匹配，不能按完整值相等。
        Ok(activates.iter().flatten().any(|activate| {
            let mut name = windows::core::PWSTR::null();
            let mut length = 0u32;
            let read = unsafe {
                activate.GetAllocatedString(
                    &MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                    &mut name,
                    &mut length,
                )
            };
            if read.is_err() || name.is_null() {
                return false;
            }
            let text = unsafe { name.to_string() }.unwrap_or_default();
            unsafe { CoTaskMemFree(Some(name.as_ptr().cast())) };
            text.contains(INSTANCE_NAME)
        }))
    })();
    let _ = unsafe { MFShutdown() };
    listed
}

/// 清理 MF 帧服务器里已失效的虚拟相机登记项。
///
/// ★为什么必须有这一步★ `IMFVirtualCamera::Remove` 只注销"这一个活对象", 实测 System 生命周期
/// 下仍会残留一条登记项 —— 表现为 DLL 与 COM 键都已删掉、`MFEnumDeviceSources` 却照旧列出本相机
/// (幽灵设备: 客户端能看见、打开必失败)。`MFCleanupVirtualCameraEntries` 是系统提供的清扫入口。
/// windows 0.62 未生成该绑定, 故运行期从 `mfsensorgroup.dll` 取; 取不到就当此系统没有该入口。
fn _cleanup_virtual_camera_entries() {
    use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryW};
    unsafe {
        let Ok(module) = LoadLibraryW(w!("mfsensorgroup.dll")) else {
            return;
        };
        if let Some(entry) = GetProcAddress(
            module,
            windows::core::PCSTR(c"MFCleanupVirtualCameraEntries".as_ptr().cast()),
        ) {
            let cleanup: unsafe extern "system" fn() -> HRESULT = std::mem::transmute(entry);
            let hr = cleanup();
            if hr.is_err() {
                log::warn!("虚拟摄像头: MFCleanupVirtualCameraEntries 返回 {}", hr.0);
            }
        }
    }
}

/// 一次性清除旧 MF 实现留下的系统相机。★不返回错误★: 这是"顺手擦干净"的兼容动作,
/// 新装机上本来就没有它, 失败也不该拦住 DirectShow 的正常卸载 —— 有残留时后面的核验会报出来。
fn _remove_legacy_mf_camera() {
    let Ok(camera) = _open_legacy_mf_camera() else {
        _cleanup_virtual_camera_entries();
        return;
    };
    let mut failures = Vec::new();
    for (action, result) in [
        ("Stop", unsafe { camera.Stop() }),
        ("Remove", unsafe { camera.Remove() }),
        ("Shutdown", unsafe { camera.Shutdown() }),
    ] {
        if let Err(error) = result {
            if !_is_virtual_camera_absent(&error) {
                failures.push(format!("{}: {}", action, error));
            }
        }
    }
    // Remove 之后再清扫: 残留登记项不会被 Remove 带走(见 _cleanup_virtual_camera_entries)。
    _cleanup_virtual_camera_entries();
    if !failures.is_empty() {
        log::warn!(
            "虚拟摄像头: 清理旧 MF 系统相机未完全成功({})",
            failures.join("；")
        );
    }
}

fn _is_virtual_camera_absent(error: &windows::core::Error) -> bool {
    let code = error.code();
    code == HRESULT::from_win32(ERROR_FILE_NOT_FOUND.0)
        || code == HRESULT::from_win32(ERROR_NOT_FOUND.0)
        // MF_E_NOT_FOUND: 该相机本来就不在(重复卸载 / 已被系统清掉) ⇒ 幂等成功, 不算失败。
        || code == MF_E_NOT_FOUND
        || code == MF_E_SHUTDOWN
}

/// 将多步事务写成临时批处理后以管理员执行，避免嵌套 `cmd /c` 的引号截断。
fn _run_elevated_steps(steps: &[String]) -> Result<u32> {
    let script = std::env::temp_dir().join(format!("mai2control_vcam_{}.cmd", std::process::id()));
    let body = format!("@echo off\r\n{}\r\nexit /b 0\r\n", steps.join("\r\n"));
    std::fs::write(&script, body)
        .map_err(|error| anyhow!("创建提权脚本 {} 失败：{}", script.display(), error))?;
    let result = run_elevated("cmd.exe", &format!("/d /c {}", _cmd_quote(&script)));
    std::fs::remove_file(&script)
        .map_err(|error| anyhow!("清理提权脚本 {} 失败：{}", script.display(), error))?;
    result
}

/// 以管理员身份同步执行一条命令并取回退出码（ShellExecuteExW + "runas" + 等进程结束）。
/// 全工程只有这一份提权实现；WinUSB 驱动包也复用它，不另起一套。
pub(crate) fn run_elevated(file: &str, parameters: &str) -> Result<u32> {
    use windows::Win32::Foundation::{CloseHandle, WAIT_OBJECT_0};
    use windows::Win32::System::Threading::{GetExitCodeProcess, WaitForSingleObject};
    use windows::Win32::UI::Shell::{SEE_MASK_NOCLOSEPROCESS, SHELLEXECUTEINFOW, ShellExecuteExW};

    let file = _wide(file);
    let parameters = _wide(parameters);
    let verb = _wide("runas");
    let mut info = SHELLEXECUTEINFOW {
        cbSize: size_of::<SHELLEXECUTEINFOW>() as u32,
        fMask: SEE_MASK_NOCLOSEPROCESS,
        lpVerb: PCWSTR(verb.as_ptr()),
        lpFile: PCWSTR(file.as_ptr()),
        lpParameters: PCWSTR(parameters.as_ptr()),
        nShow: 0,
        ..Default::default()
    };
    unsafe { ShellExecuteExW(&mut info) }
        .map_err(|error| anyhow!("管理员命令未启动（可能取消了 UAC）：{}", error))?;
    let process = info.hProcess;
    if process.is_invalid() {
        return Err(anyhow!("管理员命令已启动但拿不到进程句柄，无法确认结果"));
    }
    let waited = unsafe { WaitForSingleObject(process, 60_000) };
    let mut code = u32::MAX;
    if waited == WAIT_OBJECT_0 {
        unsafe { _ = GetExitCodeProcess(process, &mut code) };
    }
    unsafe { _ = CloseHandle(process) };
    if waited != WAIT_OBJECT_0 {
        return Err(anyhow!("管理员命令超时未结束（等待结果 {:?}）", waited));
    }
    Ok(code)
}

fn _wide(text: &str) -> Vec<u16> {
    text.encode_utf16().chain(Some(0)).collect()
}

fn _cmd_quote(path: &Path) -> String {
    format!("\"{}\"", path.display())
}

fn _normalize_path_text(path: &str) -> String {
    let mut value = path.trim();
    while let Some(unquoted) = value
        .strip_prefix('"')
        .and_then(|value| value.strip_suffix('"'))
    {
        value = unquoted.trim();
    }
    let value = value.replace('/', "\\");
    let value = value.strip_prefix("\\\\?\\").unwrap_or(&value).to_string();
    let mut normalized = value.trim_end_matches('\\').to_string();
    if normalized.len() == 2 && normalized.as_bytes().get(1) == Some(&b':') {
        normalized.push('\\');
    }
    normalized
}
