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
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};

use super::{backend, keyboard};

const OWNER_FILE: &str = "winusb-owner-v1.txt";
const OWNER_MARKER: &str = "mai2control WinUSB rebind ownership v1";
const INF_FILE: &str = "mai2vcam.inf";
const CAT_FILE: &str = "mai2vcam.cat";
const CER_FILE: &str = "mai2vcam.cer";
const CERT_SUBJECT: &str = "CN=mai2control vcam WinUSB (self-signed)";
/// INF `[Strings]` 段里本应用自报的两个名字。
/// ★改绑后它们会盖住设备节点的 `DeviceDesc` / `Manufacturer`★ 于是"现场读设备名"会把本应用的
/// 名字当成用户扫码器的厂商报出去。故读到这两个值一律弃用(见 `rebound_devices`), 不冒充设备信息。
const INF_PROVIDER: &str = "mai2control";
const INF_DEV_DESC: &str = "mai2control vcam scanner (WinUSB)";
/// 与 INF 的 `DeviceInterfaceGUIDs` 同一个值。三重身份, 缺一不可:
///   ① 供 WinUSB 公开设备接口(直读靠它);
///   ② "哪一份 oemN.inf 是本应用发布的"的唯一稳定标记(pnputil 输出是本地化文案, 不可作判据);
///   ③ ★"这台设备是本应用改绑的"的唯一自带标识★ —— INF 把它写进被改绑设备的硬件键, winusb.sys
///      据此注册设备接口, 于是该设备一定会出现在这个接口类的枚举里。这个标识活在**设备树上**,
///      跨重启有效, 与 `toolbox.cfg` 的用户选择、与本应用的所有权记录文件都无关, 因此它才是
///      "本应用改绑过哪些设备"的真相源(见 `rebound_devices`)。
/// ★只在这里写一次★ INF/PowerShell 要的文本形式由 `_guid_text()` 从同一常量渲染, 不许两处各写。
pub(crate) const DEVICE_INTERFACE_GUID: windows::core::GUID =
    windows::core::GUID::from_u128(0xB7A0F1C2_4E3D_4A5B_9C6D_8E7F00112233);

/// 设备接口类 GUID 的 `{XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}` 文本形式(INF 正文与 INF 扫描用)。
/// 必须与已安装 INF 里的写法逐字一致, 故固定大写加花括号。
fn _guid_text() -> String {
    let g = DEVICE_INTERFACE_GUID;
    format!(
        "{{{:08X}-{:04X}-{:04X}-{:02X}{:02X}-{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}}}",
        g.data1,
        g.data2,
        g.data3,
        g.data4[0],
        g.data4[1],
        g.data4[2],
        g.data4[3],
        g.data4[4],
        g.data4[5],
        g.data4[6],
        g.data4[7],
    )
}
pub(crate) const WINUSB_SERVICE: &str = "WinUSB";
/// 设备栈重建(restart-device / 驱动换绑)需要重新枚举, 实测秒级; 给足余量但必须有上限。
const REBIND_TIMEOUT: Duration = Duration::from_secs(25);
/// 错行格式的所有权记录只报一次(一次刷新会读它好几遍)。
static _MALFORMED_OWNER_LOGGED: AtomicBool = AtomicBool::new(false);

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
fn hex_field(text: &str, key: &str) -> Option<u16> {
    let at = text.find(key)? + key.len();
    let value: String = text[at..]
        .chars()
        .take_while(char::is_ascii_hexdigit)
        .collect();
    (value.len() == 4)
        .then(|| u16::from_str_radix(&value, 16).ok())
        .flatten()
}

/// 一台 USB 设备的**稳定身份**: VID/PID + 设备自报序列号。
///
/// ★完整实例 ID 不能当身份用★ 设备没报序列号时实例 ID 形如
/// `USB\VID_x&PID_y\6&<hash>&<port>`, 换口或重新枚举就变; 只把它记下来, 下次开机就认不回来了。
/// 本类型只保留不会漂移的部分, 所有"记录 ↔ 现场"的对认、可持久化路径的合成都走它。
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct UsbIdentity {
    pub(crate) vid: u16,
    pub(crate) pid: u16,
    /// 设备自报序列号; `None` = 设备没报(此时同型号多台无法区分, 反查命中多台一律拒绝, 不猜)。
    pub(crate) serial: Option<String>,
}

impl UsbIdentity {
    /// 从 `USB\VID_xxxx&PID_xxxx\<尾段>` 解析。尾段含 `&` 即总线生成的实例路径, 不是序列号。
    /// ★这条判据只此一份★ 直读侧(`winusb_scanner`)与改绑侧共用, 免得两套规则各自漂移。
    pub(crate) fn of_usb_instance(instance: &str) -> Option<Self> {
        let mut segments = instance.split('\\');
        let hardware = segments.nth(1)?;
        let tail = segments.next().unwrap_or_default();
        Some(Self {
            vid: hex_field(hardware, "VID_")?,
            pid: hex_field(hardware, "PID_")?,
            serial: (!tail.is_empty() && !tail.contains('&')).then(|| tail.to_string()),
        })
    }

    /// 从任意带 VID/PID 的串解析(Raw Input HID 路径、本类型合成的稳定路径)。序列号信息无从取得。
    pub(crate) fn of_text(text: &str) -> Option<Self> {
        let upper = text.to_ascii_uppercase();
        Some(Self {
            vid: hex_field(&upper, "VID_")?,
            pid: hex_field(&upper, "PID_")?,
            serial: None,
        })
    }

    /// 同一台设备(按 VID/PID; 两边都有序列号时序列号也必须相同)。
    /// 本方的序列号为 `None` 表示"不知道", 只按 VID/PID 认 —— 这也是反查命中多台时必须拒绝的原因。
    pub(crate) fn same_device(&self, other: &Self) -> bool {
        self.vid == other.vid
            && self.pid == other.pid
            && match (&self.serial, &other.serial) {
                (Some(mine), Some(theirs)) => mine.eq_ignore_ascii_case(theirs),
                _ => true,
            }
    }

    /// 界面副文案用的 VID/PID 读数。
    pub(crate) fn vid_pid_text(&self) -> String {
        format!("VID_{:04X} PID_{:04X}", self.vid, self.pid)
    }

    /// **不含漂移字段**的可持久化设备标识, 供已改绑设备当作下拉项的 `path`。
    ///
    /// ★为什么不复用原来的 HID 路径★ 那条路径里的 HID 实例尾段随重新枚举而变, 而且改绑后
    /// 那个 HID 节点根本不存在了; 拿它当持久化键, 每次重新枚举都会对不上、选择随之丢失。
    /// 尾段挂本应用的设备接口 GUID, 一眼可辨"这是本应用改绑的直读目标"。
    pub(crate) fn stable_path(&self) -> String {
        match &self.serial {
            Some(serial) => format!(
                r"\\?\USB#VID_{:04X}&PID_{:04X}#{}#{}",
                self.vid,
                self.pid,
                serial,
                _guid_text()
            ),
            None => format!(
                r"\\?\USB#VID_{:04X}&PID_{:04X}#{}",
                self.vid,
                self.pid,
                _guid_text()
            ),
        }
    }
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
    Ok(_Target {
        _hardware_id: _hardware_id_of(&usb_instance)
            .ok_or_else(|| anyhow!("USB 实例 ID 缺少合法硬件 ID 段（读到 {}）", usb_instance))?,
        _usb_instance: usb_instance,
    })
}

/// USB 实例 ID → INF `[Models]` 段要的硬件 ID。
/// ★必须原样保留 `&MI_xx`★ 复合设备可改绑的是接口节点, 砍掉它会写出一条永远匹配不上的硬件 ID;
/// 因此这里取整段硬件 ID 文本, 只用 `UsbIdentity` 校验 VID/PID 合法。
fn _hardware_id_of(instance: &str) -> Option<String> {
    let tail = instance.split('\\').nth(1)?;
    UsbIdentity::of_usb_instance(instance).map(|_| format!("USB\\{}", tail))
}

/// 本应用改绑的所有权记录。
///
/// ★这份记录只能是"更好的线索", 不能是唯一依据★ 恢复路径曾经在 `published` 为空时整体拒绝执行,
/// 于是一旦这个字段被写坏, 用户就永远回不到 hidusb。现在缺字段一律走降级(自行找回 / 跳过该步),
/// 每一层都如实报告做到了哪一步, 见 `unbind_and_uninstall`。
///
/// 后三项是改绑前登记的**原始设备文案**: 改绑后该设备从键盘枚举里整体消失, 界面上要显示原名
/// 只能从这里恢复。老记录没有这些字段时为空串, 界面显示"未知", 不编造。
struct _Owner {
    _published: String,
    _thumbprint: String,
    _instance: String,
    _product: String,
    _label: String,
    _vendor: String,
}

fn _owner_record() -> Option<_Owner> {
    let text = std::fs::read(_dir().join(OWNER_FILE))
        .ok()
        .map(|bytes| String::from_utf8_lossy(&bytes).to_string())?;
    // 记录文件现按 UTF-8 写(设备产品名可能含非 ASCII)。PowerShell 5.1 的 UTF8 一定带 BOM,
    // 不剥掉首行就永远匹配不上 marker ⇒ 整份记录被判为"不存在"。
    let text = text.trim_start_matches('\u{feff}');
    let lines: Vec<&str> = text.lines().map(str::trim).collect();
    if lines.first()?.trim() != OWNER_MARKER {
        return None;
    }
    // ★兼容一份写坏的历史记录★ 旧写盘脚本里 `@('k=' + $v, ...)` 被 PowerShell 当成
    // `@('k=', $v, ...)`(逗号优先级高于 `+`), 于是每个字段都被拆成"键行 + 值行"两行,
    // 按 `key=value` 读一律得到空串 —— 实例 ID 因此读不到, 已改绑的设备就从下拉里彻底消失。
    // 写盘侧已修(见 `install_and_bind`), 但用户机上那份还在, 只能在读侧认回:
    // 键行为空值且下一行不是键行时, 取下一行作为它的值。
    let field = |key: &str| -> String {
        lines
            .iter()
            .enumerate()
            .find_map(|(at, line)| {
                let value = line.strip_prefix(key)?;
                if !value.is_empty() {
                    return Some(value.to_string());
                }
                lines
                    .get(at + 1)
                    .filter(|next| !next.is_empty() && !next.contains('='))
                    .map(|next| next.to_string())
            })
            .unwrap_or_default()
    };
    // 认回错行格式时留一次痕: 这是"记录读不出实例 ID ⇒ 改绑设备从下拉里消失"那条故障的现场凭据,
    // 也说明这份文件出自旧写盘脚本。只报一次 —— 一次刷新会读它好几遍。
    if lines.iter().enumerate().any(|(at, line)| {
        line.ends_with('=')
            && lines
                .get(at + 1)
                .is_some_and(|next| !next.is_empty() && !next.contains('='))
    }) && !_MALFORMED_OWNER_LOGGED.swap(true, Ordering::Relaxed)
    {
        log::warn!(
            "虚拟摄像头: 改绑所有权记录是旧写盘脚本留下的错行格式（键与值分处两行），已按行对读回；下次改绑会重写为正常格式"
        );
    }
    Some(_Owner {
        _published: field("published="),
        _thumbprint: field("thumbprint="),
        _instance: field("instance="),
        _product: field("product="),
        _label: field("label="),
        _vendor: field("vendor="),
    })
}

/// 一台**已被本应用改绑到 WinUSB** 的设备。它已从键盘枚举里整体消失(Windows 不再为它建键盘栈),
/// 因此只能由本结构把它补回可选输入源列表(见 `keyboard::list_input_sources`)。
pub(crate) struct ReboundDevice {
    /// 不含漂移字段的可持久化标识(见 `UsbIdentity::stable_path`)。
    pub(crate) path: String,
    /// 现场 USB 节点实例 ID。会随换口/重新枚举漂移, 只作显示与排查, 不作身份。
    pub(crate) instance: String,
    /// 实测功能驱动名(必为 `WinUSB`, 否则这台设备不会出现在本列表里)。
    pub(crate) service: String,
    pub(crate) vid_pid: String,
    /// 用户认得的名字: 改绑前登记的产品名优先, 记录缺失时退回现场读 USB 节点的总线上报名。
    pub(crate) product: String,
    pub(crate) label: String,
    pub(crate) vendor: String,
}

/// 现场枚举"本应用改绑过、且此刻在位"的设备。
///
/// ★判据是设备自带的标识, 不是用户的持久化选择★ 旧实现把这条合成行挂在 `toolbox.cfg` 的
/// `vcam_kbd` 上: 一旦该选择被清空(选过一次"所有键盘"/换机器/换 exe 目录), 或所有权记录里的
/// 实例 ID 读不出来, 改绑过的设备就在界面上彻底不存在, 用户再也选不回来、直读随之停摆 ——
/// 这正是"改成 WinUSB 的 HIDKeyBoard 重启后从下拉里消失"的成因。
/// 现在改为按 `DEVICE_INTERFACE_GUID` 从设备树枚举: 那是 INF 在改绑时写进设备硬件键的标识,
/// 只要设备还绑在本应用的驱动包上就一定在, 与任何本地文件无关。
///
/// 所有权记录退化为**名字的来源**(设备已不在键盘枚举里, 改绑前的产品名只能从那里恢复), 外加一条
/// 兜底线索: 接口枚举没命中时, 仍按记录里的实例 / 稳定身份反查一次。
pub(crate) fn rebound_devices() -> Vec<ReboundDevice> {
    let owner = _owner_record();
    let recorded = owner
        .as_ref()
        .map(|owner| owner._instance.to_ascii_uppercase())
        .filter(|instance| _safe_device_text(instance) && instance.starts_with("USB\\"));
    let recorded_identity = recorded.as_deref().and_then(UsbIdentity::of_usb_instance);
    // ① 设备自带标识(真相源) ② 记录里登记的实例 ③ 按记录的稳定身份反查现场 WinUSB 节点
    let mut candidates = keyboard::present_interface_instances(&DEVICE_INTERFACE_GUID);
    candidates.extend(recorded);
    if let Some(identity) = &recorded_identity
        && let Ok(instance) = _scan_winusb_instance(identity)
    {
        candidates.push(instance);
    }
    let mut out: Vec<ReboundDevice> = Vec::new();
    for instance in candidates {
        let instance = instance.to_ascii_uppercase();
        // 只认现场读得到、且功能驱动实测仍是 WinUSB 的节点: 设备拔了、或已恢复成 hidusb 的
        // 陈旧接口记录一律不上界面(接口注册键在设备离场后仍留在注册表里)。
        let Some(service) = keyboard::device_service(&instance)
            .filter(|service| service.eq_ignore_ascii_case(WINUSB_SERVICE))
        else {
            continue;
        };
        let Some(identity) = UsbIdentity::of_usb_instance(&instance) else {
            continue;
        };
        let path = identity.stable_path();
        if out.iter().any(|have| have.path == path) {
            continue;
        }
        // 记录里的名字只在确认是同一台设备时才采用, 否则那是另一台设备的登记。
        let logged = owner
            .as_ref()
            .filter(|_| recorded_identity.as_ref().is_some_and(|id| *id == identity));
        let named = |value: Option<&String>| -> Option<String> {
            value.filter(|text| !text.trim().is_empty()).cloned()
        };
        // 现场读到的名字里, 凡是本应用 INF 自报的那两个串一律弃用: 那是我们盖上去的, 不是设备信息。
        let live = |value: String| {
            (!value.is_empty()
                && !value.eq_ignore_ascii_case(INF_PROVIDER)
                && !value.eq_ignore_ascii_case(INF_DEV_DESC))
            .then_some(value)
        };
        let (live_product, live_vendor) = keyboard::node_identity(&instance);
        let product = named(logged.map(|owner| &owner._product))
            .or_else(|| live(live_product))
            .unwrap_or_else(|| format!("已改绑设备 {}", identity.vid_pid_text()));
        out.push(ReboundDevice {
            label: named(logged.map(|owner| &owner._label)).unwrap_or_else(|| product.clone()),
            vendor: named(logged.map(|owner| &owner._vendor))
                .or_else(|| live(live_vendor))
                .unwrap_or_default(),
            product,
            vid_pid: identity.vid_pid_text(),
            path,
            instance,
            service,
        });
    }
    out
}

/// 写进所有权记录的一行文案。记录是行分隔的 `key=value`, 因此换行/回车必须去掉;
/// 过长的名字截断到 128 字符(设备产品串远短于此, 截断只是防御畸形描述符)。
fn _owner_text(value: &str) -> String {
    value
        .chars()
        .filter(|ch| !ch.is_control())
        .take(128)
        .collect()
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
    let mut unknown_reasons: Vec<String> = Vec::new();
    let mut candidates: Vec<String> = Vec::new();
    match _resolve_target(raw_input_path) {
        Ok(target) => candidates.push(target._usb_instance),
        Err(live_error) => unknown_reasons.push(live_error.to_string()),
    }
    candidates.extend(_recorded_instance(raw_input_path));
    for instance in candidates {
        match keyboard::device_service(&instance) {
            Some(service) if service.eq_ignore_ascii_case(WINUSB_SERVICE) => {
                return BindingStatus::Bound { instance };
            }
            Some(service) => return BindingStatus::Other { instance, service },
            None => unknown_reasons.push(format!(
                "读不到 {} 的 SPDRP_SERVICE（该实例可能已不在场）",
                instance
            )),
        }
    }
    // ★最后一级: 按稳定身份反查现场★ 无序列号的设备实例 ID 换口/重新枚举就变, 记录随之变陈旧;
    // 此时只剩"按 VID/PID(+序列号) 从 USB 枚举里找出仍绑在 WinUSB 上的那个节点"这条路。
    match _selected_identity(raw_input_path).ok_or_else(|| {
        anyhow!(
            "所选路径里没有合法的 VID/PID（读到 {}），无法反查",
            raw_input_path
        )
    }) {
        Ok(identity) => match _scan_winusb_instance(&identity) {
            Ok(instance) => BindingStatus::Bound { instance },
            Err(scan_error) => {
                unknown_reasons.push(scan_error.to_string());
                BindingStatus::Unknown {
                    reason: unknown_reasons.join("；"),
                }
            }
        },
        Err(error) => {
            unknown_reasons.push(error.to_string());
            BindingStatus::Unknown {
                reason: unknown_reasons.join("；"),
            }
        }
    }
}

/// 所有权记录里登记的 USB 实例，且其 VID/PID 必须与当前所选路径一致。
fn _recorded_instance(raw_input_path: &str) -> Option<String> {
    let want = UsbIdentity::of_text(raw_input_path)?;
    let instance = _owner_record()?._instance.to_ascii_uppercase();
    if !_safe_device_text(&instance) || !instance.starts_with("USB\\") {
        return None;
    }
    UsbIdentity::of_usb_instance(&instance)?
        .same_device(&want)
        .then_some(instance)
}

/// 所选路径对应的稳定身份。所有权记录登记的身份优先——只有它带得出序列号，
/// 而序列号是同型号多台设备唯一的区分依据；记录不是这台设备时退回路径自带的 VID/PID。
fn _selected_identity(raw_input_path: &str) -> Option<UsbIdentity> {
    let from_path = UsbIdentity::of_text(raw_input_path)?;
    let recorded = _owner_record()
        .as_ref()
        .and_then(|owner| UsbIdentity::of_usb_instance(&owner._instance))
        .filter(|recorded| recorded.same_device(&from_path));
    Some(recorded.unwrap_or(from_path))
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

/// "哪一份 `oemN.inf` 是本应用发布的"这一判定的**唯一**实现: 按 INF 正文里的设备接口 GUID 命中
/// (`pnputil` 的输出是本地化文案, 不可作为判定依据)。
///
/// ★安装与恢复必须共用同一份★ 恢复路径原先根本没有这一步, 只认所有权记录里的 `published`;
/// 该字段一旦被写空, 用户就再也删不掉驱动包、回不到 hidusb。生成的 PowerShell 片段把命中的
/// 文件名写进变量 `var`(找不到则保持原值不变, 由调用侧先置空)。
fn _ps_find_published(var: &str) -> String {
    format!(
        "$hit = Get-ChildItem -LiteralPath (Join-Path $env:SystemRoot 'INF') -Filter 'oem*.inf' -ErrorAction SilentlyContinue | \
         Where-Object {{ (Get-Content -LiteralPath $_.FullName -Raw -ErrorAction SilentlyContinue) -like {guid_like} }} | Select-Object -First 1\n\
         if ($hit) {{ {var} = $hit.Name }}\n",
        guid_like = _ps(&format!("*{}*", _guid_text())),
        var = var,
    )
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
Provider  = "@@PROVIDER@@"
Dev.Desc  = "@@DESC@@"
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
        .replace("@@GUID@@", &_guid_text())
        .replace("@@PROVIDER@@", INF_PROVIDER)
        .replace("@@DESC@@", INF_DEV_DESC);
    let body: Vec<String> = text.lines().map(_ps).collect();
    format!("@(\n{}\n)", body.join(",\n"))
}

/// 步骤 ①：生成驱动包并自签名 + 装信任。**不** add-driver、**不**碰任何设备。
///
/// ★脚本里那道 ReparsePoint 检查不是多余的★ 驱动包落在 `%ProgramData%` 下，而该目录的默认 ACL
/// 允许任何标准用户建目录 —— 于是别人可以先把 `...\vcam\winusb` 建成指向自己可控位置的联接，
/// 让我们的管理员脚本把 INF/CAT 写进去、再由 `pnputil` 以管理员身份读回来。见到重解析点即中止。
///
/// ★签名这一步绝不覆盖 `published`/`instance`★ 旧实现把整份所有权记录重写一遍, 只从旧文件里
/// "读回来保留"那两个字段 —— 于是"先改绑、后再点一次签名"在旧文件缺字段时会把发布名写成空串,
/// 而恢复路径当年只认这个字段, 用户就此被锁死在 WinUSB 上。现在只替换 `thumbprint=` 那一行,
/// 其余行逐字保留; 只有记录不存在(或首行不是本应用的 marker)时才创建完整骨架。
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
             $prev = @()\n\
             if (Test-Path -LiteralPath $owner) {{ $prev = @(Get-Content -LiteralPath $owner) }}\n\
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
             $thumbline = 'thumbprint=' + $cert.Thumbprint\n\
             if ($prev.Count -gt 0 -and $prev[0].Trim() -eq {marker}) {{ \
             $kept = @(); $done = $false\n\
             foreach ($l in $prev) {{ if ($l -like 'thumbprint=*') {{ $kept += $thumbline; $done = $true }} else {{ $kept += $l }} }}\n\
             if (-not $done) {{ $kept += $thumbline }}\n\
             Set-Content -LiteralPath $owner -Value $kept -Encoding UTF8 }}\n\
             else {{ Set-Content -LiteralPath $owner -Value @({marker}, 'published=', $thumbline, 'instance=') -Encoding UTF8 }}\n\
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
    if let Ok(target) = _resolve_target(raw_input_path) {
        return Ok(target._hardware_id);
    }
    // 已改绑(HID 节点已消失)时用同一条绑定判定链找回 USB 节点: 记录、以及按稳定身份的现场反查
    // 都在 `binding_status` 里, 这里不再写第二套降级。
    let status = binding_status(raw_input_path);
    let instance = status
        .instance()
        .ok_or_else(|| anyhow!("取不到可用于生成硬件 ID 的 USB 节点：{}", status.detail()))?;
    _hardware_id_of(instance)
        .ok_or_else(|| anyhow!("USB 实例 ID {} 缺少合法硬件 ID 段", instance))
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
///
/// ★所有权记录必须逐行 `$rec += ('key=' + $value)` 地拼★ 绝不要图省事写成
/// `@('key=' + $a, 'key2=' + $b)`: PowerShell 的逗号优先级**高于** `+`, 那种写法会被解析成
/// `@('key=', $a, 'key2=', $b)`, 于是每个字段都被写成"键行 + 值行"两行, 按 `key=value` 读一律
/// 得到空串。实测后果: 记录里的 `instance=` 读不出来 ⇒ `binding_status` 判 `Unknown` ⇒
/// 已改绑的设备从下拉里彻底消失, 用户再也选不回来。
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
    // ★原始设备文案必须在改绑**之前**登记★ 改绑一旦生效, 该设备就从键盘枚举里整体消失,
    // 此后再也查不到它的产品名/厂商 —— 界面上那条合成行只能靠这份登记显示原名。
    let identity = keyboard::list_keyboards()
        .into_iter()
        .find(|device| device.path.eq_ignore_ascii_case(raw_input_path));
    let (product, label, vendor) = match &identity {
        Some(device) => (
            _owner_text(&device.product),
            _owner_text(&device.label),
            _owner_text(&device.vendor),
        ),
        None => (String::new(), String::new(), String::new()),
    };
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
             if ($pub -eq '') {{ {find_pub} }}\n\
             $lines += 'PUBLISHED=' + $pub\n\
             $null = & pnputil.exe /restart-device {instance} 2>&1\n\
             $lines += 'RESTART_EXIT=' + $LASTEXITCODE\n\
             $thumb = ''\n\
             if (Test-Path -LiteralPath {owner}) {{ foreach ($l in (Get-Content -LiteralPath {owner})) {{ \
             if ($l -like 'thumbprint=*') {{ $thumb = $l.Substring(11) }} }} }}\n\
             $rec = @({marker})\n\
             $rec += ('published=' + $pub)\n\
             $rec += ('thumbprint=' + $thumb)\n\
             $rec += ('instance=' + {instance})\n\
             $rec += ('product=' + {product})\n\
             $rec += ('label=' + {label})\n\
             $rec += ('vendor=' + {vendor})\n\
             Set-Content -LiteralPath {owner} -Value $rec -Encoding UTF8\n\
             Set-Content -LiteralPath {result} -Value $lines -Encoding UTF8\n",
            inf = _ps_path(&_pkg_dir().join(INF_FILE)),
            find_pub = _ps_find_published("$pub"),
            instance = _ps(&target._usb_instance),
            owner = _ps_path(&owner),
            marker = _ps(OWNER_MARKER),
            product = _ps(&product),
            label = _ps(&label),
            vendor = _ps(&vendor),
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

/// 按所选路径的 VID/PID 从设备树反查"当前绑在 WinUSB 上的那个 USB 节点"。
///
/// ★恢复路径的最后一层兜底★ 改绑后 HID 节点消失, "Raw Input 路径 → HID → USB 父节点"这条换算
/// 已断; 所有权记录里的 `instance=` 又可能缺失/被写空。此时只剩这一条路。
/// 同型号有多台都在 WinUSB 上时**必须停下**: 猜错一台就会把用户另一台设备一起改回去。
fn _scan_winusb_instance(identity: &UsbIdentity) -> Result<String> {
    let mut hits: Vec<String> = Vec::new();
    for instance in keyboard::present_usb_instances() {
        let upper = instance.to_ascii_uppercase();
        if !_safe_device_text(&upper)
            || !UsbIdentity::of_usb_instance(&upper).is_some_and(|have| have.same_device(identity))
        {
            continue;
        }
        if keyboard::device_service(&upper)
            .is_some_and(|service| service.eq_ignore_ascii_case(WINUSB_SERVICE))
        {
            hits.push(upper);
        }
    }
    match hits.len() {
        1 => Ok(hits.remove(0)),
        0 => Err(anyhow!(
            "设备树里没有 {} 且功能驱动为 {} 的 USB 节点（设备可能已拔出，或早已不在 WinUSB 上）",
            identity.vid_pid_text(),
            WINUSB_SERVICE
        )),
        n => Err(anyhow!(
            "设备树里有 {} 个 {} 的节点都绑在 {} 上且无序列号可区分，无法判定是哪一台；请只保留目标设备后重试",
            n,
            identity.vid_pid_text(),
            WINUSB_SERVICE
        )),
    }
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

/// 步骤 ③：把设备恢复成普通 HID 键盘，并尽可能清掉本应用发布的驱动包与证书。
///
/// ★分层降级, 绝不单点依赖所有权记录★ 旧实现在 `published` 不合法时整体 `return Err`, 一步都不做
/// —— 而那个字段恰恰会被"先改绑、后再点签名"写空(见 `sign_and_trust`), 于是用户永远回不到 hidusb。
/// 现在的层次是:
///   ① 发布名: 记录里合法就用它; 不合法就用与安装侧同一份 GUID 扫描(`_ps_find_published`)自行找回;
///   ② 仍找不到 ⇒ **跳过** `/delete-driver`, 但照旧恢复设备绑定(restart-device, 必要时
///      remove-device + scan-devices 解除绑定让 Windows 重新匹配 inbox `hidusb`);
///   ③ USB 实例 ID 记录里也没有 ⇒ 按所选路径的 VID/PID 从设备树反查(`_scan_winusb_instance`),
///      反查不到才停止;
///   ④ 结局分三种如实报告, 不许把"包没找到但设备已恢复"说成"包已删除"。
/// 恢复成功与否一律以 `SPDRP_SERVICE` 实测判定, 退出码不作证据。
pub fn unbind_and_uninstall(raw_input_path: &str) -> Result<String> {
    let owner = _owner_record();
    let recorded_published = owner
        .as_ref()
        .map(|owner| owner._published.clone())
        .unwrap_or_default();
    let thumbprint = owner
        .as_ref()
        .map(|owner| owner._thumbprint.clone())
        .unwrap_or_default();
    // 记录里的发布名不合法(空串/被污染)时传空串进脚本, 由脚本按 GUID 自行找回。
    let published_hint = if _valid_published(&recorded_published) {
        recorded_published.clone()
    } else {
        String::new()
    };
    // 绑定判定链本身已含"按稳定身份反查现场 WinUSB 节点"这一级(见 `binding_status`),
    // 这里不再重复一套降级。
    let status = binding_status(raw_input_path);
    let instance = status
        .instance()
        .map(str::to_string)
        .ok_or_else(|| anyhow!("无法定位要恢复的 USB 节点：{}", status.detail()))?;
    let dir = _dir();
    let lines = _run_elevated_script("unbind", |result| {
        format!(
            "$ErrorActionPreference = 'Continue'\n\
             $lines = @()\n\
             $pub = {published}\n\
             if ($pub -eq '') {{ {find_pub} }}\n\
             $lines += 'PUBLISHED=' + $pub\n\
             if ($pub -ne '') {{ $null = & pnputil.exe /delete-driver $pub /uninstall /force 2>&1\n\
             $lines += 'DELETE_EXIT=' + $LASTEXITCODE }}\n\
             else {{ $lines += 'DELETE_SKIPPED=1' }}\n\
             $null = & pnputil.exe /restart-device {instance} 2>&1\n\
             $lines += 'RESTART_EXIT=' + $LASTEXITCODE\n\
             if ($pub -eq '') {{ \
             $null = & pnputil.exe /remove-device {instance} 2>&1\n\
             $lines += 'REMOVE_EXIT=' + $LASTEXITCODE\n\
             $null = & pnputil.exe /scan-devices 2>&1\n\
             $lines += 'SCAN_EXIT=' + $LASTEXITCODE }}\n\
             $thumb = {thumb}\n\
             if ($thumb -ne '') {{ foreach ($p in @('Cert:\\LocalMachine\\Root\\' + $thumb, 'Cert:\\LocalMachine\\TrustedPublisher\\' + $thumb)) {{ \
             if (Test-Path -LiteralPath $p) {{ Remove-Item -LiteralPath $p -Force -ErrorAction SilentlyContinue }}\n\
             if (Test-Path -LiteralPath $p) {{ $lines += 'CERT_LEFT=' + $p }} }}\n\
             $mine = 'Cert:\\CurrentUser\\My\\' + $thumb\n\
             if (Test-Path -LiteralPath $mine) {{ Remove-Item -LiteralPath $mine -Force -DeleteKey -ErrorAction SilentlyContinue }} }}\n\
             $left = ''\n\
             {find_left}\n\
             $lines += 'LEFT_PUBLISHED=' + $left\n\
             if ($pub -ne '' -and $left -eq '') {{ \
             Remove-Item -LiteralPath {pkg} -Recurse -Force -ErrorAction SilentlyContinue\n\
             Remove-Item -LiteralPath {cer} -Force -ErrorAction SilentlyContinue\n\
             Remove-Item -LiteralPath {owner} -Force -ErrorAction SilentlyContinue\n\
             $lines += 'CLEANED=1' }}\n\
             $lines += 'OK=1'\n\
             Set-Content -LiteralPath {result} -Value $lines -Encoding UTF8\n",
            published = _ps(&published_hint),
            find_pub = _ps_find_published("$pub"),
            find_left = _ps_find_published("$left"),
            instance = _ps(&instance),
            thumb = _ps(&thumbprint),
            pkg = _ps_path(&_pkg_dir()),
            cer = _ps_path(&dir.join(CER_FILE)),
            owner = _ps_path(&dir.join(OWNER_FILE)),
            result = _ps(result),
        )
    })?;
    let published = _marker(&lines, "PUBLISHED=");
    let left_published = _marker(&lines, "LEFT_PUBLISHED=");
    // ★唯一的成败判据★: 设备节点的功能驱动实测已不是 WinUSB。退出码只进失败时的排查文案。
    let service = _await_binding(&instance, false).map_err(|error| {
        anyhow!(
            "设备仍绑在 {} 上，恢复失败（{}）；驱动包发布名 {}，pnputil 退出码 delete={} restart={} remove={} scan={}",
            WINUSB_SERVICE,
            error,
            if published.is_empty() {
                "未找到".to_string()
            } else {
                published.clone()
            },
            _marker(&lines, "DELETE_EXIT="),
            _marker(&lines, "RESTART_EXIT="),
            _marker(&lines, "REMOVE_EXIT="),
            _marker(&lines, "SCAN_EXIT=")
        )
    })?;
    // 三种真实结局分开报告。★"包未找到但设备已恢复"绝不能说成"包已删除"★:
    // 那份包还在系统里, 下次安装会直接撞上它, 用户必须知道。
    let mut report = if published.is_empty() {
        format!(
            "驱动包未找到，但设备已恢复（{} 的功能驱动实测为 {}）；本应用发布的驱动包**未删除**\
             （%SystemRoot%\\INF 下没有带本应用设备接口 GUID 的 oem*.inf），已保留改绑记录供重试",
            instance, service
        )
    } else if left_published.is_empty() {
        format!(
            "驱动包已删除 + 设备已恢复（{} 的功能驱动实测为 {}，已删除本应用发布的 {}）",
            instance, service, published
        )
    } else {
        format!(
            "设备已恢复（{} 的功能驱动实测为 {}），但驱动包 {} 删除后仍在 %SystemRoot%\\INF 里\
             （复查命中 {}）；已保留改绑记录供重试",
            instance, service, published, left_published
        )
    };
    // 发布名到底是记录给的还是自己扫出来的, 必须写清楚: 后者说明记录已损坏, 用户下次仍会踩到。
    if !published.is_empty() && published_hint.is_empty() {
        report.push_str(&format!(
            "；发布名由 INF 内设备接口 GUID 扫描找回（所有权记录里的发布名不可用，读到 {:?}）",
            recorded_published
        ));
    }
    // 目录/证书是否真的清掉了必须复核; 只有 pkg 目录已空时才顺手收掉空目录。
    let left_certs: Vec<&String> = lines
        .iter()
        .filter(|line| line.starts_with("CERT_LEFT="))
        .collect();
    if _marker(&lines, "CLEANED=") == "1" {
        let _ = std::fs::remove_dir(&dir);
        let residue = signing_status();
        if residue.inf_present || residue.catalog_status != "Missing" {
            report.push_str(&format!("；⚠ 驱动包文件仍有残留（{}）", residue.detail()));
        }
    } else {
        report.push_str("；本地驱动包文件与改绑记录已保留（未确认驱动包已从系统删除）");
    }
    if left_certs.is_empty() {
        if thumbprint.is_empty() {
            report.push_str("；未登记证书指纹，故未移除任何证书");
        } else {
            report.push_str("；已移除本应用装入受信任根与受信任发布者的证书");
        }
    } else {
        report.push_str(&format!(
            "；⚠ 以下证书未能移除：{}",
            left_certs
                .iter()
                .map(|line| line.trim_start_matches("CERT_LEFT="))
                .collect::<Vec<_>>()
                .join("、")
        ));
    }
    Ok(report)
}
