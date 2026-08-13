//! 把选定 HID 键盘扫码器的 **USB 设备节点** 从 `hidusb` 改绑到 inbox 的 `winusb.sys`。
//!
//! ★为什么走这条路★ 键盘类上层过滤驱动(Interception 那套)只有 10 个键盘槽, 占槽顺序还随设备栈
//! 建立顺序变化; 而本机 HVCI 正在生效, 自写内核过滤驱动一律加载不进去。改绑到微软已签名的
//! `winusb.sys` 之后, Windows 从此不再把该设备当键盘 ⇒ 天然 100% 吞键, 与用户有多少键盘无关、
//! 无任何槽位上限, 且不引入任何新内核二进制。
//!
//! ★代价必须讲清楚★ 改绑后该扫码器在**所有**程序里都不再是键盘。这正是吞键的实现方式,
//! 但也意味着停用本程序时它同样不向别处输入; 要恢复成普通 HID 键盘必须显式执行恢复。
//!
//! 本模块只做驱动包的生成/签名/信任/安装改绑/卸载恢复/状态查询, 不读设备数据
//! (直读在 `winusb_scanner`)。所有"已签名/已改绑"的结论都必须有实测证据:
//! `Get-AuthenticodeSignature` 为 `Valid`、设备节点 `SPDRP_SERVICE` 读作 `WinUSB`;
//! 命令退出码 0 一律不算证据。

use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};

use super::{backend, keyboard};

const OWNER_FILE: &str = "winusb-owner-v1.txt";
const OWNER_MARKER: &str = "mai2control WinUSB rebind ownership v1";
const INF_FILE: &str = "mai2vcam.inf";
const CAT_FILE: &str = "mai2vcam.cat";
const CER_FILE: &str = "mai2vcam.cer";
const CERT_SUBJECT: &str = "CN=mai2control vcam WinUSB (self-signed)";
/// 与 INF 的 `DeviceInterfaceGUIDs` 同一个值: 既供 WinUSB 公开设备接口, 也是"哪一份 oemN.inf
/// 是我们发布的"这一判定的唯一稳定标记(pnputil 的输出是本地化文案, 不可作为判定依据)。
const DEVICE_INTERFACE_GUID: &str = "{B7A0F1C2-4E3D-4A5B-9C6D-8E7F00112233}";
const WINUSB_SERVICE: &str = "WinUSB";
/// 设备栈重建(restart-device / 驱动换绑)需要重新枚举, 实测秒级; 给足余量但必须有上限。
const REBIND_TIMEOUT: Duration = Duration::from_secs(25);

/// 与 `interception::install_dir()` 同级的本应用 WinUSB 驱动包目录。
fn _dir() -> PathBuf {
    std::env::var_os("ProgramData")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(r"C:\ProgramData"))
        .join("mai2control")
        .join("vcam")
        .join("winusb")
}

/// 驱动包目录: `New-FileCatalog` 按目录整体编目, 因此这里**只能**放本次要签的 INF 与 CAT。
fn _pkg_dir() -> PathBuf {
    _dir().join("pkg")
}

/// 临时 `.ps1` 与结果文件的落脚点：**只能**是当前用户私有的临时目录。
///
/// ★安全要点，勿改回 `%ProgramData%`★ `%ProgramData%` 根的默认 ACL 给 `Users` 授了
/// "创建文件夹/追加数据"，所以**任何**标准用户都能先把 `mai2control\vcam\winusb` 建出来并在里面
/// 放一个同名 `.ps1`。我们随后以管理员身份去执行那个路径 ⇒ 别人的代码在管理员上下文里跑，
/// 这是一条现成的本地提权链。`%LOCALAPPDATA%\Temp` 只对当前用户 + SYSTEM + Administrators 开放，
/// 而能触发本操作的本来就是这个用户自己，不构成跨主体提权。
fn _script_dir() -> PathBuf {
    std::env::temp_dir()
}

/// 一次改绑的目标身份。
struct _Target {
    /// INF 里的硬件 ID, 形如 `USB\VID_26F1&PID_8801`(复合设备则带 `&MI_xx`)。
    _hardware_id: String,
    /// USB 设备节点实例 ID, `pnputil /restart-device` 与绑定核验都用它。
    _usb_instance: String,
}

/// 设备 ID 只允许 PnP 实际会用到的字符集。任何越界字符一律拒绝, 而不是转义后继续 ——
/// 这些串会进命令行与 PowerShell 单引号字符串, 白名单是唯一可靠的注入防线。
fn _safe_device_text(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 512
        && value
            .chars()
            .all(|ch| ch.is_ascii_alphanumeric() || matches!(ch, '_' | '&' | '\\' | '.' | '-' | '+'))
}

/// 从 `VID_xxxx`/`PID_xxxx` 字段取 4 位十六进制。缺一个就判定这不是 USB 硬件 ID。
/// WinUSB 直读侧也用同一份解析(见 `winusb_scanner`), 不再另写第二套。
pub(crate) fn hex_field(text: &str, key: &str) -> Option<String> {
    let at = text.find(key)? + key.len();
    let value: String = text[at..]
        .chars()
        .take_while(char::is_ascii_hexdigit)
        .collect();
    (value.len() == 4).then_some(value)
}

/// Raw Input HID 路径 → 可改绑的 USB 节点身份。
///
/// ★父节点才是可改绑目标★ Raw Input 给的是 HID 集合路径, 它对应的 `HID\...` 节点由 hidusb 创建;
/// 真正能换功能驱动的是它的父节点 `USB\VID_xxxx&PID_xxxx\序列号`。
fn _resolve_target(raw_input_path: &str) -> Result<_Target> {
    let hid_instance = backend::hid_instance_id(raw_input_path)?;
    let usb_instance = keyboard::parent_of(&hid_instance)
        .ok_or_else(|| anyhow!("目标 HID 节点没有可改绑的 USB 父节点（设备可能已不在场）"))?
        .to_ascii_uppercase();
    if !_safe_device_text(&usb_instance) || !usb_instance.starts_with("USB\\") {
        return Err(anyhow!(
            "目标 HID 节点的父设备不是 USB 节点（读到 {}），拒绝改绑",
            usb_instance
        ));
    }
    let hardware_tail = usb_instance
        .split('\\')
        .nth(1)
        .ok_or_else(|| anyhow!("USB 实例 ID 缺少硬件 ID 段"))?;
    let hardware_id = format!("USB\\{}", hardware_tail);
    // VID/PID 必须都能取到 4 位十六进制，否则这条硬件 ID 不该被写进 INF 的 [Models] 段。
    for key in ["VID_", "PID_"] {
        if hex_field(&hardware_id, key).is_none() {
            return Err(anyhow!(
                "USB 硬件 ID 缺少合法 {}xxxx（读到 {}）",
                key,
                hardware_id
            ));
        }
    }
    Ok(_Target {
        _hardware_id: hardware_id,
        _usb_instance: usb_instance,
    })
}

/// 本应用改绑的所有权记录。
/// ★只卸载自己装的东西★ 没有这份记录就绝不删除任何驱动包或证书, 也不声称清理过。
struct _Owner {
    _published: String,
    _thumbprint: String,
    _instance: String,
}

fn _owner_record() -> Option<_Owner> {
    let text = std::fs::read(_dir().join(OWNER_FILE))
        .ok()
        .map(|bytes| String::from_utf8_lossy(&bytes).to_string())?;
    let mut lines = text.lines();
    if lines.next()?.trim() != OWNER_MARKER {
        return None;
    }
    let field = |key: &str| -> String {
        text.lines()
            .find_map(|line| line.trim().strip_prefix(key).map(str::to_string))
            .unwrap_or_default()
    };
    Some(_Owner {
        _published: field("published="),
        _thumbprint: field("thumbprint="),
        _instance: field("instance="),
    })
}

/// 三档签名状态。★`Set-AuthenticodeSignature` 那一步会返回 `UnknownError`, 那不是失败★:
/// 证书当时还没进受信任存储, 状态只能判为未知。唯一可信的判定是信任完成之后再读一次为 `Valid`。
pub struct SigningStatus {
    /// `Get-AuthenticodeSignature` 的 Status 文本; 目录不存在时为 `Missing`。
    pub catalog_status: String,
    pub thumbprint: String,
    pub in_root: bool,
    pub in_publisher: bool,
    pub inf_present: bool,
    /// 探测本身失败(PowerShell 起不来等)时的原因; 非空即表示"未确认", 不得当作已签名。
    pub probe_error: String,
}

impl SigningStatus {
    pub fn ready(&self) -> bool {
        self.probe_error.is_empty()
            && self.inf_present
            && self.catalog_status == "Valid"
            && self.in_root
            && self.in_publisher
    }

    pub fn detail(&self) -> String {
        if !self.probe_error.is_empty() {
            return format!("签名状态未确认（{}）", self.probe_error);
        }
        if !self.inf_present {
            return "未签名（驱动包尚未生成）".to_string();
        }
        if self.ready() {
            return format!(
                "已签名并信任（目录签名核验 Valid，证书指纹 {}，已在受信任根与受信任发布者）",
                self.thumbprint
            );
        }
        format!(
            "未完成签名信任（目录签名核验 {}；受信任根 {}；受信任发布者 {}）",
            self.catalog_status,
            if self.in_root { "在" } else { "不在" },
            if self.in_publisher { "在" } else { "不在" },
        )
    }

    /// UI 用的一行结论。★证书指纹、存储位置这些排查用的细节不进界面★:
    /// 它们已由 `detail()` 落进日志, 摊在界面上只会把"到底成没成"这一个结论淹掉。
    pub fn short(&self) -> String {
        if !self.probe_error.is_empty() {
            return "未确认".to_string();
        }
        if !self.inf_present {
            return "未签名".to_string();
        }
        if self.ready() {
            return "已签名并信任".to_string();
        }
        format!("未完成（目录签名 {}）", self.catalog_status)
    }
}

/// 绑定实测状态。判定依据只有设备节点的 `SPDRP_SERVICE`。
pub enum BindingStatus {
    /// 已改绑 WinUSB：Windows 不再把它当键盘，本程序可直读中断 IN 端点。
    Bound { instance: String },
    /// 仍绑在别的功能驱动上（HID 键盘时通常是 `hidusb`）。
    Other { instance: String, service: String },
    /// 读不到证据。绝不美化成任一侧结论。
    Unknown { reason: String },
}

impl BindingStatus {
    pub fn bound(&self) -> bool {
        matches!(self, Self::Bound { .. })
    }

    pub fn instance(&self) -> Option<&str> {
        match self {
            Self::Bound { instance } | Self::Other { instance, .. } => Some(instance),
            Self::Unknown { .. } => None,
        }
    }

    pub fn detail(&self) -> String {
        match self {
            Self::Bound { instance } => format!(
                "已改绑 WinUSB（{} 的功能驱动实测为 {}）；该设备在所有程序里都不再是键盘",
                instance, WINUSB_SERVICE
            ),
            Self::Other { instance, service } => {
                format!("未改绑（{} 的功能驱动实测为 {}）", instance, service)
            }
            Self::Unknown { reason } => format!("绑定状态未确认（{}）", reason),
        }
    }

    /// UI 用的一行结论。实例 ID 只对排查有用, 留给日志。
    pub fn short(&self) -> String {
        match self {
            Self::Bound { .. } => "已改绑 WinUSB · Windows 已不再视其为键盘".to_string(),
            Self::Other { service, .. } => format!("未改绑（当前功能驱动 {}）", service),
            Self::Unknown { .. } => "未确认".to_string(),
        }
    }
}

/// 读目标 USB 节点当前的功能驱动。
///
/// ★改绑成功后目标 HID 节点会消失★ 于是"Raw Input 路径 → HID 节点 → 父 USB 节点"这条换算也断了。
/// 因此先按现场设备树解析, 解析不到时回落到所有权记录里登记过的 USB 实例(仅当 VID/PID 与所选
/// 路径一致才采用), 两条都不成立才报未确认。
pub fn binding_status(raw_input_path: &str) -> BindingStatus {
    let instance = match _resolve_target(raw_input_path) {
        Ok(target) => target._usb_instance,
        Err(live_error) => match _recorded_instance(raw_input_path) {
            Some(instance) => instance,
            None => {
                return BindingStatus::Unknown {
                    reason: format!("{}；也没有本应用的改绑记录可核对", live_error),
                };
            }
        },
    };
    match keyboard::device_service(&instance) {
        Some(service) if service.eq_ignore_ascii_case(WINUSB_SERVICE) => {
            BindingStatus::Bound { instance }
        }
        Some(service) => BindingStatus::Other { instance, service },
        None => BindingStatus::Unknown {
            reason: format!("读不到 {} 的 SPDRP_SERVICE（设备可能已不在场）", instance),
        },
    }
}

/// 所有权记录里登记的 USB 实例，且其 VID/PID 必须与当前所选路径一致。
fn _recorded_instance(raw_input_path: &str) -> Option<String> {
    let owner = _owner_record()?;
    let instance = owner._instance.to_ascii_uppercase();
    if !_safe_device_text(&instance) || !instance.starts_with("USB\\") {
        return None;
    }
    let selected = raw_input_path.to_ascii_uppercase();
    let matches = ["VID_", "PID_"].iter().all(|key| {
        match (hex_field(&instance, key), hex_field(&selected, key)) {
            (Some(left), Some(right)) => left == right,
            _ => false,
        }
    });
    matches.then_some(instance)
}

/// PowerShell 单引号字符串字面量。单引号内不做任何展开, 只需把 `'` 自身翻倍。
fn _ps(value: &str) -> String {
    format!("'{}'", value.replace('\'', "''"))
}

fn _ps_path(path: &std::path::Path) -> String {
    _ps(&path.display().to_string())
}

/// 把脚本写盘 → 提权执行 → 读回结果文件 → 删除脚本与结果文件。
///
/// ★脚本必须带 UTF-8 BOM★ PowerShell 5.1 把**无 BOM** 的 UTF-8 脚本按 ANSI 解读, 任何非 ASCII
/// 字节都会被拆坏并报 "无法将…识别为 cmdlet"。脚本正文本身一律只写 ASCII, 但注入进去的路径
/// (未提权时会退回到用户名可能含非 ASCII 的临时目录)不受我们控制, 所以靠 BOM 兜住。
///
/// 判定一律靠脚本写出的 ASCII 标记行, 不解析(本地化的)标准输出。
fn _run_elevated_script(label: &str, build: impl FnOnce(&str) -> String) -> Result<Vec<String>> {
    let dir = _script_dir();
    let stem = format!("mai2vcam_winusb_{}_{}", label, std::process::id());
    let script = dir.join(format!("{}.ps1", stem));
    let result = dir.join(format!("{}.txt", stem));
    let _ = std::fs::remove_file(&result);
    let body = format!("\u{feff}{}", build(&result.display().to_string()));
    std::fs::write(&script, body)
        .map_err(|error| anyhow!("创建提权脚本 {} 失败：{}", script.display(), error))?;
    let exit = backend::run_elevated(
        "powershell.exe",
        &format!(
            "-NoProfile -NonInteractive -ExecutionPolicy Bypass -File {}",
            backend::cmd_quote_text(&script.display().to_string())
        ),
    );
    let lines = std::fs::read(&result).ok().map(|bytes| {
        String::from_utf8_lossy(&bytes)
            .trim_start_matches('\u{feff}')
            .lines()
            .map(str::to_string)
            .collect::<Vec<String>>()
    });
    let _ = std::fs::remove_file(&script);
    let _ = std::fs::remove_file(&result);
    let exit = exit?;
    match lines {
        Some(lines) if !lines.is_empty() => Ok(lines),
        _ => Err(anyhow!(
            "提权脚本未写出结果（PowerShell 退出码 {}）；不会声称操作已完成",
            exit
        )),
    }
}

fn _marker(lines: &[String], key: &str) -> String {
    lines
        .iter()
        .find_map(|line| line.trim().strip_prefix(key).map(str::to_string))
        .unwrap_or_default()
}

/// 无副作用的只读探测: 不提权、不弹窗、不改任何状态。
fn _run_probe(body: &str) -> Result<Vec<String>> {
    use std::os::windows::process::CommandExt;
    // CREATE_NO_WINDOW: 状态查询会在 UI 线程外反复调用, 不能每次闪一个控制台窗口。
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    let output = std::process::Command::new("powershell.exe")
        .args(["-NoProfile", "-NonInteractive", "-Command", body])
        .creation_flags(CREATE_NO_WINDOW)
        .output()
        .map_err(|error| anyhow!("启动 PowerShell 探测失败：{}", error))?;
    Ok(String::from_utf8_lossy(&output.stdout)
        .lines()
        .map(str::to_string)
        .collect())
}

/// 三档状态里的"是否已签名并信任"。全部结论都来自实测读数, 拿不到就报未确认。
pub fn signing_status() -> SigningStatus {
    let inf = _pkg_dir().join(INF_FILE);
    let catalog = _pkg_dir().join(CAT_FILE);
    let inf_present = inf.is_file();
    if !inf_present && !catalog.is_file() {
        // 包都还没生成, 没有任何可核验对象, 不必为此起一个 PowerShell。
        return SigningStatus {
            catalog_status: "Missing".to_string(),
            thumbprint: String::new(),
            in_root: false,
            in_publisher: false,
            inf_present,
            probe_error: String::new(),
        };
    }
    let body = format!(
        "$ErrorActionPreference='SilentlyContinue'\n\
         $cat = {cat}\n\
         $thumb = ''\n\
         $status = 'Missing'\n\
         if (Test-Path -LiteralPath $cat) {{ $s = Get-AuthenticodeSignature -LiteralPath $cat; \
         if ($s) {{ $status = $s.Status.ToString(); \
         if ($s.SignerCertificate) {{ $thumb = $s.SignerCertificate.Thumbprint }} }} }}\n\
         $root = '0'\n\
         $pub = '0'\n\
         if ($thumb -ne '') {{ \
         if (Test-Path -LiteralPath ('Cert:\\LocalMachine\\Root\\' + $thumb)) {{ $root = '1' }}; \
         if (Test-Path -LiteralPath ('Cert:\\LocalMachine\\TrustedPublisher\\' + $thumb)) {{ $pub = '1' }} }}\n\
         Write-Output ('CAT_STATUS=' + $status)\n\
         Write-Output ('CAT_THUMB=' + $thumb)\n\
         Write-Output ('ROOT=' + $root)\n\
         Write-Output ('PUBLISHER=' + $pub)\n",
        cat = _ps_path(&catalog)
    );
    match _run_probe(&body) {
        Ok(lines) => SigningStatus {
            catalog_status: {
                let value = _marker(&lines, "CAT_STATUS=");
                if value.is_empty() {
                    "Missing".to_string()
                } else {
                    value
                }
            },
            thumbprint: _marker(&lines, "CAT_THUMB="),
            in_root: _marker(&lines, "ROOT=") == "1",
            in_publisher: _marker(&lines, "PUBLISHER=") == "1",
            inf_present,
            probe_error: String::new(),
        },
        Err(error) => SigningStatus {
            catalog_status: "Unknown".to_string(),
            thumbprint: String::new(),
            in_root: false,
            in_publisher: false,
            inf_present,
            probe_error: error.to_string(),
        },
    }
}

/// 已实测通过 `pnputil /add-driver` 的 INF 模板。
/// ★不要"改进"它★ `Include`/`Needs` 三段分别指向 inbox `winusb.inf` 的 NT / NT.HW / NT.Services,
/// 缺任一段都会装成一个没有服务的空壳节点; `PnpLockdown=1` 是 Win10+ 驱动包的硬要求。
const INF_TEMPLATE: &str = r#"[Version]
Signature   = "$Windows NT$"
Class       = USBDevice
ClassGuid   = {88BAE032-5A81-49f0-BC3D-A4FF138216D6}
Provider    = %Provider%
CatalogFile = @@CAT@@
DriverVer   = 01/01/2026,1.0.0.0
PnpLockdown = 1

[Manufacturer]
%Provider% = Models, NTamd64

[Models.NTamd64]
%Dev.Desc% = WinUsbInst, @@HWID@@

[WinUsbInst.NT]
Include = winusb.inf
Needs   = WINUSB.NT

[WinUsbInst.NT.HW]
Include = winusb.inf
Needs   = WINUSB.NT.HW
AddReg  = Dev.AddReg

[Dev.AddReg]
HKR,,DeviceInterfaceGUIDs,0x10000,"@@GUID@@"

[WinUsbInst.NT.Services]
Include = winusb.inf
Needs   = WINUSB.NT.Services

[Strings]
Provider  = "mai2control"
Dev.Desc  = "mai2control vcam scanner (WinUSB)"
"#;

/// 把 INF 正文渲染成 PowerShell 字符串数组字面量。
///
/// ★为什么不用 here-string★ INF 里有 `$Windows NT$` 与 `%Provider%`: 双引号 here-string 会把
/// `$Windows` 当变量展开成空串, 直接毁掉 `[Version]` 段。逐行单引号数组既保住这些字面量,
/// 又让 `Set-Content -Encoding ASCII` 输出标准 CRLF 行尾。
fn _inf_ps_array(hardware_id: &str) -> String {
    let text = INF_TEMPLATE
        .replace("@@CAT@@", CAT_FILE)
        .replace("@@HWID@@", hardware_id)
        .replace("@@GUID@@", DEVICE_INTERFACE_GUID);
    let body: Vec<String> = text.lines().map(_ps).collect();
    format!("@(\n{}\n)", body.join(",\n"))
}

/// 步骤 ①：生成驱动包并自签名 + 装信任。**不** add-driver、**不**碰任何设备。
///
/// ★脚本里那道 ReparsePoint 检查不是多余的★ 驱动包落在 `%ProgramData%` 下，而该目录的默认 ACL
/// 允许任何标准用户建目录 —— 于是别人可以先把 `...\vcam\winusb` 建成指向自己可控位置的联接，
/// 让我们的管理员脚本把 INF/CAT 写进去、再由 `pnputil` 以管理员身份读回来。见到重解析点即中止。
///
/// 这是一个独立的用户动作: 它会向本机"受信任的根证书颁发机构"与"受信任的发布者"各写入一张
/// 由本应用自签发的代码签名证书 —— 这是 Windows 10 起驱动包强制签名的唯一无内核代价的满足方式。
/// 硬件 ID 取当前已选定的扫码器(只读查设备树, 不改设备栈)。
pub fn sign_and_trust() -> Result<String> {
    let selected = keyboard::target_device()
        .ok_or_else(|| anyhow!("请先在设备树里选定要改绑的扫码器，再签名驱动包"))?;
    let hardware_id = _hardware_id_for(&selected)?;
    let dir = _dir();
    let pkg = _pkg_dir();
    let owner = dir.join(OWNER_FILE);
    let lines = _run_elevated_script("sign", |result| {
        format!(
            "$ErrorActionPreference = 'Stop'\n\
             $lines = @()\n\
             try {{\n\
             $pkg = {pkg}\n\
             $owner = {owner}\n\
             $published = ''\n\
             $instance = ''\n\
             if (Test-Path -LiteralPath $owner) {{ foreach ($l in (Get-Content -LiteralPath $owner)) {{ \
             if ($l -like 'published=*') {{ $published = $l.Substring(10) }}\n\
             if ($l -like 'instance=*') {{ $instance = $l.Substring(9) }} }} }}\n\
             if (-not (Test-Path -LiteralPath $pkg)) {{ $null = New-Item -ItemType Directory -Path $pkg -Force }}\n\
             foreach ($probe in @({dir}, $pkg)) {{ $item = Get-Item -LiteralPath $probe -Force\n\
             if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {{ throw ('refuse reparse point: ' + $probe) }} }}\n\
             Get-ChildItem -LiteralPath $pkg -File | Remove-Item -Force\n\
             Set-Content -LiteralPath {inf} -Value {inf_body} -Encoding ASCII\n\
             $lines += 'INF=1'\n\
             $subject = {subject}\n\
             $cert = Get-ChildItem -Path Cert:\\CurrentUser\\My | Where-Object {{ $_.Subject -eq $subject -and $_.HasPrivateKey }} | Sort-Object NotAfter -Descending | Select-Object -First 1\n\
             if ($null -eq $cert) {{ $cert = New-SelfSignedCertificate -Type CodeSigningCert -Subject $subject -CertStoreLocation Cert:\\CurrentUser\\My -KeyUsage DigitalSignature -KeyExportPolicy Exportable }}\n\
             $lines += 'THUMB=' + $cert.Thumbprint\n\
             $null = New-FileCatalog -Path $pkg -CatalogFilePath {cat} -CatalogVersion 2\n\
             $sig = Set-AuthenticodeSignature -FilePath {cat} -Certificate $cert -HashAlgorithm SHA256\n\
             $lines += 'SIGNSTEP=' + $sig.Status.ToString()\n\
             $null = Export-Certificate -Cert $cert -FilePath {cer} -Type CERT\n\
             $null = Import-Certificate -FilePath {cer} -CertStoreLocation Cert:\\LocalMachine\\Root\n\
             $null = Import-Certificate -FilePath {cer} -CertStoreLocation Cert:\\LocalMachine\\TrustedPublisher\n\
             $lines += 'FINAL=' + (Get-AuthenticodeSignature -LiteralPath {cat}).Status.ToString()\n\
             Set-Content -LiteralPath $owner -Value @({marker}, 'published=' + $published, 'thumbprint=' + $cert.Thumbprint, 'instance=' + $instance) -Encoding ASCII\n\
             $lines += 'OK=1'\n\
             }} catch {{ $lines += 'ERROR=' + $_.Exception.Message }}\n\
             Set-Content -LiteralPath {result} -Value $lines -Encoding UTF8\n",
            pkg = _ps_path(&pkg),
            owner = _ps_path(&owner),
            inf = _ps_path(&pkg.join(INF_FILE)),
            inf_body = _inf_ps_array(&hardware_id),
            subject = _ps(CERT_SUBJECT),
            cat = _ps_path(&pkg.join(CAT_FILE)),
            cer = _ps_path(&dir.join(CER_FILE)),
            marker = _ps(OWNER_MARKER),
            dir = _ps_path(&dir),
            result = _ps(result),
        )
    })?;
    if _marker(&lines, "OK=") != "1" {
        return Err(anyhow!(
            "驱动包签名未完成：{}",
            _sign_failure_detail(&lines)
        ));
    }
    // ★唯一可信的成功判据★ 信任装完之后重新读一次目录签名, 必须为 Valid。
    // 脚本里 Set-AuthenticodeSignature 那一步返回 UnknownError 是正常的(当时证书还没受信任)。
    let status = signing_status();
    if !status.ready() {
        return Err(anyhow!(
            "签名命令已执行，但核验未通过：{}（脚本内 FINAL={}）",
            status.detail(),
            _marker(&lines, "FINAL=")
        ));
    }
    Ok(format!(
        "{}；驱动包 {}（硬件 ID {}）",
        status.detail(),
        pkg.display(),
        hardware_id
    ))
}

/// 取 INF 需要的硬件 ID。现场设备树优先；设备已改绑(HID 节点已消失)时回落到所有权记录，
/// 使"改绑后再重新签名"这一步不至于因为拿不到 HID 父节点而失败。
fn _hardware_id_for(raw_input_path: &str) -> Result<String> {
    match _resolve_target(raw_input_path) {
        Ok(target) => Ok(target._hardware_id),
        Err(live_error) => {
            let instance = _recorded_instance(raw_input_path).ok_or_else(|| {
                anyhow!("{}；也没有本应用的改绑记录可用于取硬件 ID", live_error)
            })?;
            let tail = instance
                .split('\\')
                .nth(1)
                .ok_or_else(|| anyhow!("改绑记录里的 USB 实例 ID 缺少硬件 ID 段"))?;
            Ok(format!("USB\\{}", tail))
        }
    }
}

fn _sign_failure_detail(lines: &[String]) -> String {
    let error = _marker(lines, "ERROR=");
    if error.is_empty() {
        format!(
            "脚本未报错但也未完成（INF={} FINAL={}）",
            _marker(lines, "INF="),
            _marker(lines, "FINAL=")
        )
    } else {
        error
    }
}

/// 等待设备栈重建到期望的绑定。★退出码 0 不算证据★, 只认 `SPDRP_SERVICE` 读数。
fn _await_binding(instance: &str, want_winusb: bool) -> Result<String> {
    let started = Instant::now();
    loop {
        let observed = keyboard::device_service(instance);
        if let Some(service) = &observed
            && service.eq_ignore_ascii_case(WINUSB_SERVICE) == want_winusb
        {
            return Ok(service.clone());
        }
        if started.elapsed() >= REBIND_TIMEOUT {
            return Err(anyhow!(
                "等待 {} 的功能驱动变为 {} 超时（{} 秒后仍读到 {}）",
                instance,
                if want_winusb { WINUSB_SERVICE } else { "非 WinUSB" },
                REBIND_TIMEOUT.as_secs(),
                observed.unwrap_or_else(|| "（设备节点暂不可读）".to_string())
            ));
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// 步骤 ②：发布驱动包并把所选设备改绑到 WinUSB。签名未就绪时直接拒绝，绝不隐式代做签名。
pub fn install_and_bind(raw_input_path: &str) -> Result<String> {
    let status = signing_status();
    if !status.ready() {
        return Err(anyhow!(
            "签名尚未就绪（{}）；请先执行“① 签名并信任驱动包”",
            status.detail()
        ));
    }
    let existing = binding_status(raw_input_path);
    if existing.bound() {
        return Ok(existing.detail());
    }
    let target = _resolve_target(raw_input_path)?;
    let dir = _dir();
    let owner = dir.join(OWNER_FILE);
    let lines = _run_elevated_script("bind", |result| {
        format!(
            "$ErrorActionPreference = 'Continue'\n\
             $lines = @()\n\
             $out = & pnputil.exe /add-driver {inf} /install 2>&1\n\
             $lines += 'ADD_EXIT=' + $LASTEXITCODE\n\
             $pub = ''\n\
             $m = [regex]::Matches(($out | Out-String), 'oem[0-9]+\\.inf')\n\
             if ($m.Count -gt 0) {{ $pub = $m[0].Value }}\n\
             if ($pub -eq '') {{ $hit = Get-ChildItem -LiteralPath (Join-Path $env:SystemRoot 'INF') -Filter 'oem*.inf' -ErrorAction SilentlyContinue | Where-Object {{ (Get-Content -LiteralPath $_.FullName -Raw -ErrorAction SilentlyContinue) -like {guid_like} }} | Select-Object -First 1\n\
             if ($hit) {{ $pub = $hit.Name }} }}\n\
             $lines += 'PUBLISHED=' + $pub\n\
             $null = & pnputil.exe /restart-device {instance} 2>&1\n\
             $lines += 'RESTART_EXIT=' + $LASTEXITCODE\n\
             $thumb = ''\n\
             if (Test-Path -LiteralPath {owner}) {{ foreach ($l in (Get-Content -LiteralPath {owner})) {{ \
             if ($l -like 'thumbprint=*') {{ $thumb = $l.Substring(11) }} }} }}\n\
             Set-Content -LiteralPath {owner} -Value @({marker}, 'published=' + $pub, 'thumbprint=' + $thumb, 'instance=' + {instance}) -Encoding ASCII\n\
             Set-Content -LiteralPath {result} -Value $lines -Encoding UTF8\n",
            inf = _ps_path(&_pkg_dir().join(INF_FILE)),
            guid_like = _ps(&format!("*{}*", DEVICE_INTERFACE_GUID)),
            instance = _ps(&target._usb_instance),
            owner = _ps_path(&owner),
            marker = _ps(OWNER_MARKER),
            result = _ps(result),
        )
    })?;
    let published = _marker(&lines, "PUBLISHED=");
    if published.is_empty() {
        return Err(anyhow!(
            "pnputil /add-driver 未产生可识别的发布名（退出码 {}）；没有发布名就无法安全卸载，已停止",
            _marker(&lines, "ADD_EXIT=")
        ));
    }
    let service = _await_binding(&target._usb_instance, true).map_err(|error| {
        anyhow!(
            "{}；pnputil 退出码 add={} restart={}（发布名 {}）",
            error,
            _marker(&lines, "ADD_EXIT="),
            _marker(&lines, "RESTART_EXIT="),
            published
        )
    })?;
    Ok(format!(
        "已改绑 WinUSB 并实测确认（{} 的功能驱动读作 {}，驱动包发布为 {}）；该扫码器在所有程序里都不再是键盘",
        target._usb_instance, service, published
    ))
}

/// 发布名必须形如 `oem123.inf`。这个串会进 `pnputil /delete-driver`，
/// 一旦被污染就可能删掉别人的驱动包，因此宁可拒绝也不猜。
fn _valid_published(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    lower.len() <= 32
        && lower.starts_with("oem")
        && lower.ends_with(".inf")
        && lower["oem".len()..lower.len() - ".inf".len()]
            .chars()
            .all(|ch| ch.is_ascii_digit())
        && lower.len() > "oem.inf".len()
}

/// 步骤 ③：卸载本应用的驱动包并让 Windows 自动回落原 HID 驱动，同时移除本应用装的证书。
/// 只处理本应用登记过的东西；没有所有权记录就什么都不删，也不声称清理过。
pub fn unbind_and_uninstall(raw_input_path: &str) -> Result<String> {
    let owner = _owner_record().ok_or_else(|| {
        anyhow!("没有本应用的 WinUSB 改绑记录；为避免误删他人驱动包与证书，未做任何改动")
    })?;
    if !_valid_published(&owner._published) {
        return Err(anyhow!(
            "所有权记录里的发布名不合法（读到 {:?}）；拒绝执行 pnputil /delete-driver",
            owner._published
        ));
    }
    let instance = match binding_status(raw_input_path).instance() {
        Some(instance) => instance.to_string(),
        None => return Err(anyhow!("目标 USB 节点不可读，无法核验恢复结果，已停止")),
    };
    let dir = _dir();
    let lines = _run_elevated_script("unbind", |result| {
        format!(
            "$ErrorActionPreference = 'Continue'\n\
             $lines = @()\n\
             $null = & pnputil.exe /delete-driver {published} /uninstall /force 2>&1\n\
             $lines += 'DELETE_EXIT=' + $LASTEXITCODE\n\
             $null = & pnputil.exe /restart-device {instance} 2>&1\n\
             $lines += 'RESTART_EXIT=' + $LASTEXITCODE\n\
             $thumb = {thumb}\n\
             if ($thumb -ne '') {{ foreach ($p in @('Cert:\\LocalMachine\\Root\\' + $thumb, 'Cert:\\LocalMachine\\TrustedPublisher\\' + $thumb)) {{ \
             if (Test-Path -LiteralPath $p) {{ Remove-Item -LiteralPath $p -Force -ErrorAction SilentlyContinue }}\n\
             if (Test-Path -LiteralPath $p) {{ $lines += 'CERT_LEFT=' + $p }} }}\n\
             $mine = 'Cert:\\CurrentUser\\My\\' + $thumb\n\
             if (Test-Path -LiteralPath $mine) {{ Remove-Item -LiteralPath $mine -Force -DeleteKey -ErrorAction SilentlyContinue }} }}\n\
             Remove-Item -LiteralPath {pkg} -Recurse -Force -ErrorAction SilentlyContinue\n\
             Remove-Item -LiteralPath {cer} -Force -ErrorAction SilentlyContinue\n\
             Remove-Item -LiteralPath {owner} -Force -ErrorAction SilentlyContinue\n\
             $lines += 'OK=1'\n\
             Set-Content -LiteralPath {result} -Value $lines -Encoding UTF8\n",
            published = _ps(&owner._published),
            instance = _ps(&instance),
            thumb = _ps(&owner._thumbprint),
            pkg = _ps_path(&_pkg_dir()),
            cer = _ps_path(&dir.join(CER_FILE)),
            owner = _ps_path(&dir.join(OWNER_FILE)),
            result = _ps(result),
        )
    })?;
    let service = _await_binding(&instance, false).map_err(|error| {
        anyhow!(
            "{}；pnputil 退出码 delete={} restart={}",
            error,
            _marker(&lines, "DELETE_EXIT="),
            _marker(&lines, "RESTART_EXIT=")
        )
    })?;
    // 目录/证书是否真的清掉了必须复核; 只有 pkg 目录已空时才顺手收掉空目录。
    let left: Vec<&String> = lines
        .iter()
        .filter(|line| line.starts_with("CERT_LEFT="))
        .collect();
    let _ = std::fs::remove_dir(&dir);
    let residue = signing_status();
    let mut report = format!(
        "已恢复原驱动并实测确认（{} 的功能驱动读作 {}，已删除本应用发布的 {}）",
        instance, service, owner._published
    );
    if residue.inf_present || residue.catalog_status != "Missing" {
        report.push_str(&format!(
            "；⚠ 驱动包文件仍有残留（{}）",
            residue.detail()
        ));
    }
    if left.is_empty() {
        if owner._thumbprint.is_empty() {
            report.push_str("；未登记证书指纹，故未移除任何证书");
        } else {
            report.push_str("；已移除本应用装入受信任根与受信任发布者的证书");
        }
    } else {
        report.push_str(&format!(
            "；⚠ 以下证书未能移除：{}",
            left.iter()
                .map(|line| line.trim_start_matches("CERT_LEFT="))
                .collect::<Vec<_>>()
                .join("、")
        ));
    }
    Ok(report)
}
