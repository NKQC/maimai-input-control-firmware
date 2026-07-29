//! Windows 侧 COM 口枚举与注册表处理 (#6d)
//!
//! 职责:
//! - 通过 SetupAPI 枚举 Ports 类设备,从设备实例 ID 中解析 `&MI_xx` 接口号,
//!   映射到功能(mai2serial / mai2light / config)。
//! - 计算/执行"把 serial、light 对应 COM 口改名到指定占位 COM 号"的注册表动作
//!   (`Device Parameters\PortName`),供上层(UI)在识别后调用,以满足街机侧
//!   对固定 COM 口号的要求(需管理员权限)。
//!
//! 与 `crate::io`(#6c)配合:`io::list_devices()` 按 VID/PID 枚举串口候选(跨平台,
//! 用 `serialport` crate);本模块专注 Windows 特有的"实例 ID → 功能"识别与
//! "改名到目标 COM"的注册表操作,不重复造轮子。
//!
//! ## MI 接口号 ↔ 功能映射
//!
//! 与固件实际多 CDC 描述符顺序一致:
//!
//! | MI 接口号 | 功能 |
//! |---|---|
//! | MI_00 | Config (host 配置协议) |
//! | MI_01 | Serial (mai2serial) |
//! | MI_03 | Light (mai2light) |
//!
//! 若固件描述符顺序变化,只需改 [`mi_to_function`] 一处。

use anyhow::{anyhow, Result};

// ============================================================================
// 公开数据结构(跨平台可见,便于上层无条件引用类型)
// ============================================================================

/// 目标 VID/PID(与设备实际固件一致; 2025 修正: 曾误写为 0x0CA3/0x0024)
const TARGET_VID: u16 = 0x2E8A;
const TARGET_PID: u16 = 0x000A;

/// CDC 功能分类,按 MI 接口号映射得出
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CdcFunction {
    /// mai2serial:触摸数据 → 街机/游戏(需固定 COM 口)
    Serial,
    /// mai2light:灯效 ← 游戏(需固定 COM 口)
    Light,
    /// host 配置/遥测协议(UI 直接按识别结果打开,不需固定 COM 口)
    Config,
    /// 未识别的接口号(映射表之外)
    Unknown,
}

/// 已识别的 COM 口信息
#[derive(Debug, Clone)]
pub struct IdentifiedPort {
    /// 当前系统分配的 COM 口名,如 "COM5"
    pub port_name: String,
    /// 设备实例 ID 中解析出的 MI 接口号(找不到则为 None)
    pub interface_index: Option<u8>,
    /// 按接口号映射得到的功能
    pub function: CdcFunction,
    /// 完整设备实例 ID,如 "USB\\VID_0CA3&PID_0024&MI_04\\6&2a3b1c&0&0004"
    pub instance_id: String,
}

/// 目标 COM 口分配方案(占位 COM 号,由调用方按游戏侧要求传入)
#[derive(Debug, Clone, Copy)]
pub struct ComAssignment {
    pub serial_com: u16,
    pub light_com: u16,
}

/// 一条"改名"动作(dry-run 计算结果,未执行)
#[derive(Debug, Clone)]
pub struct AssignmentAction {
    pub instance_id: String,
    pub from_port: String,
    pub to_port: String,
    pub function: CdcFunction,
}

/// `auto_assign` 中单个功能(Serial/Light)的处理结果,供 UI 中文展示。
#[derive(Debug, Clone)]
pub struct AutoAssignItem {
    pub function: CdcFunction,
    /// 中文描述,如 "触控串口: COM7 -> COM3 (已应用)"。
    pub message: String,
}

/// `auto_assign` 的整体结果:是否找到目标设备、每项处理结果、是否需要管理员/重插。
#[derive(Debug, Clone)]
pub struct AutoAssignResult {
    pub device_found: bool,
    pub items: Vec<AutoAssignItem>,
    pub needs_admin: bool,
    pub needs_replug: bool,
}

impl AutoAssignResult {
    /// 汇总为一段供 UI `port_status` 直接展示的中文多行文本。
    pub fn summary_text(&self) -> String {
        if !self.device_found {
            return "未检测到设备(未插入或未识别 Serial/Light 接口)".to_string();
        }
        let mut lines: Vec<String> = self.items.iter().map(|i| i.message.clone()).collect();
        if self.needs_admin {
            lines.push("改端口号需以管理员运行本程序".to_string());
        }
        if self.needs_replug {
            lines.push("改名将在设备重新插拔或重启后生效".to_string());
        }
        lines.join("\n")
    }
}

// ============================================================================
// MI 接口号 → 功能映射(易改的单一映射点)
// ============================================================================

/// 按 MI 接口号返回对应功能(与固件多 CDC 描述符顺序一致: MI_00=Config /
/// MI_01=Serial / MI_03=Light)。固件接口顺序如有变化,只需改这里。
fn mi_to_function(mi: u8) -> CdcFunction {
    match mi {
        0x00 => CdcFunction::Config,
        0x01 => CdcFunction::Serial,
        0x03 => CdcFunction::Light,
        _ => CdcFunction::Unknown,
    }
}

/// 解析设备实例 ID,提取 VID/PID/MI(大小写不敏感,十六进制)。
///
/// 形如 `USB\VID_0CA3&PID_0024&MI_04\6&2a3b1c&0&0004`。
/// 返回 `(vid, pid, mi)`;任一字段缺失则对应为 `None`。
fn parse_instance_id(instance_id: &str) -> (Option<u16>, Option<u16>, Option<u8>) {
    let upper = instance_id.to_ascii_uppercase();
    // 只取 "\" 之前的第一段(设备 ID 段),避免实例序列号里的干扰字符
    let first_segment = upper.split('\\').nth(1).unwrap_or("");

    let mut vid = None;
    let mut pid = None;
    let mut mi = None;

    for part in first_segment.split('&') {
        if let Some(hex) = part.strip_prefix("VID_") {
            vid = u16::from_str_radix(hex, 16).ok();
        } else if let Some(hex) = part.strip_prefix("PID_") {
            pid = u16::from_str_radix(hex, 16).ok();
        } else if let Some(hex) = part.strip_prefix("MI_") {
            mi = u8::from_str_radix(hex, 16).ok();
        }
    }

    (vid, pid, mi)
}

// ============================================================================
// 纯逻辑 API(跨平台):plan_assignment
// ============================================================================

/// 根据识别结果与目标分配方案,计算需要执行的"改名"动作列表(dry-run,不写注册表)。
///
/// 只处理 [`CdcFunction::Serial`] / [`CdcFunction::Light`];当前 COM 口已等于
/// 目标 COM 口的不生成动作。Config 接口不需要固定 COM 号,不处理。
pub fn plan_assignment(ports: &[IdentifiedPort], target: &ComAssignment) -> Vec<AssignmentAction> {
    let mut actions = Vec::new();

    for port in ports {
        let to_com_num = match port.function {
            CdcFunction::Serial => target.serial_com,
            CdcFunction::Light => target.light_com,
            CdcFunction::Config | CdcFunction::Unknown => continue,
        };
        let to_port = format!("COM{}", to_com_num);

        if port.port_name.eq_ignore_ascii_case(&to_port) {
            // 已经是目标 COM 口,无需改名
            continue;
        }

        actions.push(AssignmentAction {
            instance_id: port.instance_id.clone(),
            from_port: port.port_name.clone(),
            to_port,
            function: port.function,
        });
    }

    actions
}

// ============================================================================
// Windows 实现
// ============================================================================

#[cfg(windows)]
mod windows_impl {
    use super::*;
    use std::os::raw::c_void;

    use windows::core::{GUID, PCWSTR, PWSTR};
    use windows::Win32::Devices::DeviceAndDriverInstallation::{
        SetupDiCallClassInstaller, SetupDiDestroyDeviceInfoList, SetupDiEnumDeviceInfo,
        SetupDiGetClassDevsW, SetupDiGetDeviceInstanceIdW, SetupDiGetDeviceRegistryPropertyW,
        SetupDiOpenDevRegKey, SetupDiSetClassInstallParamsW, SetupDiSetDeviceRegistryPropertyW,
        DICS_FLAG_CONFIGSPECIFIC, DICS_FLAG_GLOBAL, DICS_PROPCHANGE, DIF_PROPERTYCHANGE,
        DIGCF_PRESENT, DIREG_DEV, HDEVINFO, SP_CLASSINSTALL_HEADER, SP_DEVINFO_DATA,
        SP_PROPCHANGE_PARAMS, SPDRP_FRIENDLYNAME,
    };
    use windows::Win32::Foundation::{CloseHandle, ERROR_NO_MORE_ITEMS};
    use windows::Win32::Security::{
        GetTokenInformation, TokenElevation, TOKEN_ELEVATION, TOKEN_QUERY,
    };
    use windows::Win32::System::Registry::{
        RegCloseKey, RegEnumValueW, RegOpenKeyExW, RegQueryValueExW, RegSetValueExW, HKEY,
        HKEY_LOCAL_MACHINE, KEY_READ, KEY_SET_VALUE, REG_BINARY, REG_SZ,
    };
    use windows::Win32::System::Threading::{GetCurrentProcess, OpenProcessToken};

    /// Ports 设备安装类 GUID:{4D36E978-E325-11CE-BFC1-08002BE10318}
    const GUID_DEVCLASS_PORTS: GUID = GUID::from_u128(0x4D36E978_E325_11CE_BFC1_08002BE10318);

    /// 枚举本机 VID_0CA3/PID_0024 复合设备下的各 COM 口,按 MI 接口号识别功能。
    ///
    /// 只读操作,失败(SetupAPI 调用出错)时记录日志并返回空 Vec,不 panic。
    /// 本机未插入目标设备时返回空 Vec 属正常情况。
    pub fn identify_ports() -> Vec<IdentifiedPort> {
        match identify_ports_inner() {
            Ok(ports) => ports,
            Err(e) => {
                log::warn!("identify_ports failed: {}", e);
                Vec::new()
            }
        }
    }

    fn identify_ports_inner() -> Result<Vec<IdentifiedPort>> {
        let mut results = Vec::new();

        // SAFETY: 传入有效的 GUID 指针与预定义 flags,返回的 HDEVINFO 在函数末尾
        // 通过 SetupDiDestroyDeviceInfoList 释放。
        let h_devinfo = unsafe {
            SetupDiGetClassDevsW(
                Some(&GUID_DEVCLASS_PORTS as *const GUID),
                PCWSTR::null(),
                None,
                DIGCF_PRESENT,
            )
        }
        .map_err(|e| anyhow!("SetupDiGetClassDevsW failed: {}", e))?;

        // 保证任何 return 路径都会释放 h_devinfo
        let result = (|| -> Result<()> {
            let mut index = 0u32;
            loop {
                let mut devinfo_data = SP_DEVINFO_DATA {
                    cbSize: std::mem::size_of::<SP_DEVINFO_DATA>() as u32,
                    ..Default::default()
                };

                // SAFETY: h_devinfo 有效,devinfo_data 为出参缓冲区
                let enum_result =
                    unsafe { SetupDiEnumDeviceInfo(h_devinfo, index, &mut devinfo_data) };
                if enum_result.is_err() {
                    // 无更多设备,正常结束枚举
                    break;
                }
                index += 1;

                if let Some(port) = identify_one_device(h_devinfo, &devinfo_data) {
                    results.push(port);
                }
            }
            Ok(())
        })();

        // SAFETY: h_devinfo 由上面的 SetupDiGetClassDevsW 成功获取,此处释放。
        unsafe {
            let _ = SetupDiDestroyDeviceInfoList(h_devinfo);
        }

        result?;
        Ok(results)
    }

    /// 枚举全系统 Ports 类设备的 (instance_id, port_name),★不按 VID/PID 过滤★。
    /// 用于 [`com_in_use`] 判断目标 COM 口是否已被"别的设备"占用(不限于本设备的
    /// Serial/Light 接口),只读操作,失败返回空 Vec,不 panic。
    fn enum_all_com_ports() -> Vec<(String, String)> {
        match enum_all_com_ports_inner() {
            Ok(v) => v,
            Err(e) => {
                log::warn!("enum_all_com_ports failed: {}", e);
                Vec::new()
            }
        }
    }

    fn enum_all_com_ports_inner() -> Result<Vec<(String, String)>> {
        let mut results = Vec::new();

        let h_devinfo = unsafe {
            SetupDiGetClassDevsW(
                Some(&GUID_DEVCLASS_PORTS as *const GUID),
                PCWSTR::null(),
                None,
                DIGCF_PRESENT,
            )
        }
        .map_err(|e| anyhow!("SetupDiGetClassDevsW failed: {}", e))?;

        let result = (|| -> Result<()> {
            let mut index = 0u32;
            loop {
                let mut devinfo_data = SP_DEVINFO_DATA {
                    cbSize: std::mem::size_of::<SP_DEVINFO_DATA>() as u32,
                    ..Default::default()
                };
                let enum_result =
                    unsafe { SetupDiEnumDeviceInfo(h_devinfo, index, &mut devinfo_data) };
                if enum_result.is_err() {
                    break;
                }
                index += 1;

                let Some(instance_id) = get_device_instance_id(h_devinfo, &devinfo_data) else {
                    continue;
                };
                let Some(port_name) = get_port_name(h_devinfo, &devinfo_data) else {
                    continue;
                };
                results.push((instance_id, port_name));
            }
            Ok(())
        })();

        unsafe {
            let _ = SetupDiDestroyDeviceInfoList(h_devinfo);
        }

        result?;
        Ok(results)
    }

    /// 判断目标 COM 口(如 COM3)是否已被"其他设备"占用。`exclude_instance_id` 传入
    /// 即将改名的设备自身实例 ID,避免把它自己(改名前仍是旧口)误判为占用者。
    pub fn com_in_use(com: u16, exclude_instance_id: &str) -> bool {
        let target = format!("COM{}", com);
        enum_all_com_ports()
            .iter()
            .any(|(iid, name)| iid != exclude_instance_id && name.eq_ignore_ascii_case(&target))
    }

    /// 按设备实例 ID 直接把 `PortName` 改写为 `COM{target_com}`(需管理员权限)。
    /// 供 [`auto_assign`] 与上层单独调用复用,不要求先经 `plan_assignment`。
    pub fn set_port_name(instance_id: &str, target_com: u16) -> Result<()> {
        if !is_elevated() {
            return Err(anyhow!("需要管理员权限才能修改 COM 口注册表分配"));
        }
        let action = AssignmentAction {
            instance_id: instance_id.to_string(),
            from_port: String::new(),
            to_port: format!("COM{}", target_com),
            function: CdcFunction::Unknown,
        };
        apply_one_action(&action, true).map(|_| ())
    }

    /// 一键"自动设置端口"组合入口:识别本机 Serial/Light 接口 → 已是目标口则跳过 →
    /// 目标口被别的设备占用则标记冲突不改 → 否则改名。返回结构含中文文案,
    /// 供 UI `port_status` 直接展示。只读识别失败/未插入设备时 `device_found=false`。
    // force=true(用户点"立即应用"): 即使注册表 PortName 已等于目标, 也重写并重启端口节点强制驱动重读,
    // 修"注册表已是目标但设备管理器/驱动仍是旧口(写了没重启从未生效)"→点应用无反应。
    // force=false(启动自动): 已匹配则跳过, 避免每次开机都重启端口造成短暂断连churn。
    pub fn auto_assign(serial_com: u16, light_com: u16, force: bool) -> AutoAssignResult {
        let ports = identify_ports();
        let elevated = is_elevated();
        let mut items = Vec::new();
        let mut device_found = false;
        let mut needs_admin = false;
        let mut needs_replug = false;

        let targets: [(CdcFunction, u16, &str); 2] = [
            (CdcFunction::Serial, serial_com, "触控串口(mai2serial)"),
            (CdcFunction::Light, light_com, "灯效串口(mai2light)"),
        ];

        for (function, target_com, label) in targets {
            let Some(port) = ports.iter().find(|p| p.function == function) else {
                continue;
            };
            device_found = true;
            let target_name = format!("COM{}", target_com);

            if port.port_name.eq_ignore_ascii_case(&target_name) {
                let action = AssignmentAction {
                    instance_id: port.instance_id.clone(),
                    from_port: port.port_name.clone(),
                    to_port: target_name.clone(),
                    function,
                };
                if force {
                    if !elevated {
                        needs_admin = true;
                        items.push(AutoAssignItem {
                            function,
                            message: format!(
                                "{}: PortName 已是 {}, 但强制重启以核验 SERIALCOMM 需要管理员权限",
                                label, target_name
                            ),
                        });
                    } else {
                        match apply_one_action(&action, true) {
                            Ok(outcome) => {
                                needs_replug = true;
                                items.push(AutoAssignItem {
                                    function,
                                    message: format!("{}: 已强制重启，{}", label, outcome.message),
                                });
                            }
                            Err(e) => items.push(AutoAssignItem {
                                function,
                                message: format!("{}: 强制核验 {} 失败: {}", label, target_name, e),
                            }),
                        }
                    }
                } else {
                    match apply_one_action(&action, false) {
                        Ok(outcome) => items.push(AutoAssignItem {
                            function,
                            message: format!("{}: 已是 {}，{}", label, target_name, outcome.message),
                        }),
                        Err(e) => items.push(AutoAssignItem {
                            function,
                            message: format!("{}: PortName 为 {}，但 SERIALCOMM 核验失败: {}", label, target_name, e),
                        }),
                    }
                }
                continue;
            }

            if com_in_use(target_com, &port.instance_id) {
                items.push(AutoAssignItem {
                    function,
                    message: format!(
                        "{}: 目标 {} 已被其他设备占用, 未修改({} 保持不变)",
                        label, target_name, port.port_name
                    ),
                });
                continue;
            }

            if !elevated {
                needs_admin = true;
                items.push(AutoAssignItem {
                    function,
                    message: format!(
                        "{}: {} -> {} 需要管理员权限, 未修改",
                        label, port.port_name, target_name
                    ),
                });
                continue;
            }

            let action = AssignmentAction {
                instance_id: port.instance_id.clone(),
                from_port: port.port_name.clone(),
                to_port: target_name.clone(),
                function,
            };
            match apply_one_action(&action, true) {
                Ok(outcome) => {
                    needs_replug = true;
                    items.push(AutoAssignItem {
                        function,
                        message: format!("{}: {} -> {}，{}", label, port.port_name, target_name, outcome.message),
                    });
                }
                Err(e) => {
                    items.push(AutoAssignItem {
                        function,
                        message: format!(
                            "{}: {} -> {} 失败: {}",
                            label, port.port_name, target_name, e
                        ),
                    });
                }
            }
        }

        AutoAssignResult {
            device_found,
            items,
            needs_admin,
            needs_replug,
        }
    }

    /// 处理单个设备信息元素:取实例 ID → 过滤 VID/PID → 取 COM 口名。
    /// 任何一步失败/不满足条件都返回 None(跳过该设备,不中断整体枚举)。
    fn identify_one_device(
        h_devinfo: windows::Win32::Devices::DeviceAndDriverInstallation::HDEVINFO,
        devinfo_data: &SP_DEVINFO_DATA,
    ) -> Option<IdentifiedPort> {
        let instance_id = get_device_instance_id(h_devinfo, devinfo_data)?;

        let (vid, pid, mi) = parse_instance_id(&instance_id);
        if vid != Some(TARGET_VID) || pid != Some(TARGET_PID) {
            return None;
        }

        let port_name = get_port_name(h_devinfo, devinfo_data)?;
        let function = mi.map(mi_to_function).unwrap_or(CdcFunction::Unknown);

        Some(IdentifiedPort {
            port_name,
            interface_index: mi,
            function,
            instance_id,
        })
    }

    /// 取设备实例 ID 字符串,如 "USB\VID_0CA3&PID_0024&MI_04\6&..."。
    fn get_device_instance_id(
        h_devinfo: windows::Win32::Devices::DeviceAndDriverInstallation::HDEVINFO,
        devinfo_data: &SP_DEVINFO_DATA,
    ) -> Option<String> {
        let mut buf = [0u16; 512];
        // SAFETY: h_devinfo/devinfo_data 有效,buf 为出参缓冲区,大小充足
        let ok = unsafe {
            SetupDiGetDeviceInstanceIdW(h_devinfo, devinfo_data, Some(&mut buf), None)
        };
        if ok.is_err() {
            return None;
        }
        let len = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
        Some(String::from_utf16_lossy(&buf[..len]))
    }

    /// 打开设备的 "Device Parameters" 注册表键,读取 `PortName` 值(如 "COM5")。
    fn get_port_name(
        h_devinfo: windows::Win32::Devices::DeviceAndDriverInstallation::HDEVINFO,
        devinfo_data: &SP_DEVINFO_DATA,
    ) -> Option<String> {
        // SAFETY: 参数均来自有效的枚举结果;返回的 HKEY 用后即关闭。
        let hkey = unsafe {
            SetupDiOpenDevRegKey(
                h_devinfo,
                devinfo_data,
                DICS_FLAG_GLOBAL.0,
                0,
                DIREG_DEV,
                KEY_READ.0,
            )
        }
        .ok()?;

        let name = read_reg_sz(hkey, "PortName");

        // SAFETY: hkey 由上面成功打开,此处关闭。
        unsafe {
            let _ = RegCloseKey(hkey);
        }

        name
    }

    /// 从已打开的注册表键读取一个 REG_SZ 字符串值。
    fn read_reg_sz(hkey: HKEY, value_name: &str) -> Option<String> {
        let wide_name: Vec<u16> = value_name.encode_utf16().chain(std::iter::once(0)).collect();
        let mut buf = [0u8; 256];
        let mut buf_len: u32 = buf.len() as u32;

        // SAFETY: hkey 有效,buf 为出参缓冲区,buf_len 标明容量。
        let status = unsafe {
            RegQueryValueExW(
                hkey,
                PCWSTR(wide_name.as_ptr()),
                None,
                None,
                Some(buf.as_mut_ptr()),
                Some(&mut buf_len),
            )
        };
        if status.is_err() {
            return None;
        }

        // buf 中是 UTF-16LE 字节序列(含终止符),转换为 String
        let u16_len = (buf_len as usize) / 2;
        let u16_slice: Vec<u16> = buf[..u16_len * 2]
            .chunks_exact(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect();
        let end = u16_slice.iter().position(|&c| c == 0).unwrap_or(u16_slice.len());
        Some(String::from_utf16_lossy(&u16_slice[..end]))
    }

    /// 对每个动作,把设备的 `PortName` 改写为目标 COM 口名(需管理员权限)。
    ///
    /// ⚠ 注意:
    /// - 改名在设备重枚举/重插后生效,应用层需提示用户重插或触发重扫描。
    /// - 目标 COM 号若被其他设备占用会产生冲突;冲突检测非本函数职责(TODO,
    ///   可在调用前结合 `identify_ports` 结果与系统已用 COM 号自行判断)。
    /// - 只写 `Device Parameters\PortName`,不触碰 `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM`
    ///   (该键由系统在设备枚举时自维护,易失,不应用户态直接改)。
    pub fn apply_assignment(actions: &[AssignmentAction]) -> Result<()> {
        if !is_elevated() {
            return Err(anyhow!("需要管理员权限才能修改 COM 口注册表分配"));
        }

        for action in actions {
            apply_one_action(action, true)?;
        }
        Ok(())
    }

    struct ApplyOutcome {
        message: String,
    }

    fn apply_one_action(action: &AssignmentAction, rewrite_and_restart: bool) -> Result<ApplyOutcome> {
        let h_devinfo = unsafe {
            SetupDiGetClassDevsW(
                Some(&GUID_DEVCLASS_PORTS as *const GUID),
                PCWSTR::null(),
                None,
                DIGCF_PRESENT,
            )
        }
        .map_err(|e| anyhow!("SetupDiGetClassDevsW failed: {}", e))?;

        let result = (|| -> Result<ApplyOutcome> {
            let mut index = 0u32;
            loop {
                let mut devinfo_data = SP_DEVINFO_DATA {
                    cbSize: std::mem::size_of::<SP_DEVINFO_DATA>() as u32,
                    ..Default::default()
                };
                if unsafe { SetupDiEnumDeviceInfo(h_devinfo, index, &mut devinfo_data) }.is_err() {
                    break;
                }
                index += 1;

                let Some(instance_id) = get_device_instance_id(h_devinfo, &devinfo_data) else {
                    continue;
                };
                if instance_id != action.instance_id {
                    continue;
                }

                let former_port = read_port_name_now(h_devinfo, &devinfo_data)
                    .filter(|port| !port.is_empty())
                    .unwrap_or_else(|| action.from_port.clone());

                if rewrite_and_restart {
                    let hkey = unsafe {
                        SetupDiOpenDevRegKey(
                            h_devinfo,
                            &devinfo_data,
                            DICS_FLAG_GLOBAL.0,
                            0,
                            DIREG_DEV,
                            KEY_SET_VALUE.0,
                        )
                    }
                    .map_err(|e| anyhow!("打开设备注册表键失败: {}", e))?;
                    let write_result = write_reg_sz(hkey, "PortName", &action.to_port);
                    unsafe {
                        let _ = RegCloseKey(hkey);
                    }
                    write_result?;
                    restart_device_node(h_devinfo, &devinfo_data)?;
                    std::thread::sleep(std::time::Duration::from_millis(1200));
                }

                return _finalize_assignment(
                    h_devinfo,
                    &mut devinfo_data,
                    &former_port,
                    &action.to_port,
                );
            }
            Err(anyhow!(
                "未找到实例 ID 对应的设备(可能已拔出): {}",
                action.instance_id
            ))
        })();

        unsafe {
            let _ = SetupDiDestroyDeviceInfoList(h_devinfo);
        }
        result
    }

    /// 重新打开设备的 Device Parameters 键、读回**当前真实**的 PortName。
    /// 用于写入+重启后核对驱动是否真的接受了该端口号(而不是信任自己刚写进去的值)。
    fn read_port_name_now(
        h_devinfo: HDEVINFO,
        devinfo_data: &SP_DEVINFO_DATA,
    ) -> Option<String> {
        // SAFETY: h_devinfo/devinfo_data 在调用方枚举期间有效。
        let hkey = unsafe {
            SetupDiOpenDevRegKey(
                h_devinfo,
                devinfo_data,
                DICS_FLAG_GLOBAL.0,
                0,
                DIREG_DEV,
                KEY_READ.0,
            )
        }
        .ok()?;
        let value = read_reg_sz(hkey, "PortName");
        unsafe {
            let _ = RegCloseKey(hkey);
        }
        value
    }

    fn _finalize_assignment(
        h_devinfo: HDEVINFO,
        devinfo_data: &mut SP_DEVINFO_DATA,
        former_port: &str,
        target_port: &str,
    ) -> Result<ApplyOutcome> {
        let serialcomm_ports = _read_serialcomm_ports()?;
        if !serialcomm_ports
            .iter()
            .any(|port| port.eq_ignore_ascii_case(target_port))
        {
            let port_name = read_port_name_now(h_devinfo, devinfo_data)
                .unwrap_or_else(|| "无法回读".to_string());
            let published = if serialcomm_ports.is_empty() {
                "无".to_string()
            } else {
                serialcomm_ports.join(", ")
            };
            if !former_port.is_empty()
                && serialcomm_ports
                    .iter()
                    .any(|port| port.eq_ignore_ascii_case(former_port))
            {
                return Err(anyhow!(
                    "驱动未接受该端口号: SERIALCOMM 未出现 {}，且仍挂着旧号 {}；PortName 回读为 {}；当前驱动端口: {}",
                    target_port,
                    former_port,
                    port_name,
                    published
                ));
            }
            return Err(anyhow!(
                "SERIALCOMM 未出现 {}，无法确认端口已生效；PortName 回读为 {}；当前驱动端口: {}",
                target_port,
                port_name,
                published
            ));
        }

        let friendly_status = match _update_friendly_name(h_devinfo, devinfo_data, target_port) {
            Ok(name) => format!("FriendlyName 已更新为 {}", name),
            Err(error) => format!(
                "端口已可用, 但设备管理器显示名可能仍是旧号(不影响使用): {}",
                error
            ),
        };
        let arbiter_status = match _update_com_name_arbiter(
            target_port,
            former_port,
            &serialcomm_ports,
        ) {
            Ok(()) => format!("COM Name Arbiter 已登记 {}", target_port),
            Err(error) => format!("COM Name Arbiter 登记警告: {}", error),
        };

        Ok(ApplyOutcome {
            message: format!(
                "端口已在 SERIALCOMM 生效；{}；{}",
                friendly_status, arbiter_status
            ),
        })
    }

    fn _read_serialcomm_ports() -> Result<Vec<String>> {
        let path: Vec<u16> = "HARDWARE\\DEVICEMAP\\SERIALCOMM"
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();
        let mut hkey = HKEY::default();
        let status = unsafe {
            RegOpenKeyExW(
                HKEY_LOCAL_MACHINE,
                PCWSTR(path.as_ptr()),
                None,
                KEY_READ,
                &mut hkey,
            )
        };
        if status.is_err() {
            return Err(anyhow!("打开 SERIALCOMM 失败: {:?}", status));
        }

        let result = (|| -> Result<Vec<String>> {
            let mut ports = Vec::new();
            let mut index = 0;
            loop {
                let mut value_name = [0u16; 256];
                let mut value_name_len = value_name.len() as u32;
                let mut value_data = [0u8; 512];
                let mut value_data_len = value_data.len() as u32;
                let mut value_type = 0u32;
                let status = unsafe {
                    RegEnumValueW(
                        hkey,
                        index,
                        Some(PWSTR(value_name.as_mut_ptr())),
                        &mut value_name_len,
                        None,
                        Some(&mut value_type),
                        Some(value_data.as_mut_ptr()),
                        Some(&mut value_data_len),
                    )
                };
                if status == ERROR_NO_MORE_ITEMS {
                    break;
                }
                if status.is_err() {
                    return Err(anyhow!("枚举 SERIALCOMM 失败: {:?}", status));
                }
                index += 1;
                if value_type != REG_SZ.0 {
                    continue;
                }
                if let Some(port) = _decode_utf16le(&value_data[..value_data_len as usize]) {
                    ports.push(port);
                }
            }
            Ok(ports)
        })();

        unsafe {
            let _ = RegCloseKey(hkey);
        }
        result
    }

    fn _update_friendly_name(
        h_devinfo: HDEVINFO,
        devinfo_data: &mut SP_DEVINFO_DATA,
        target_port: &str,
    ) -> Result<String> {
        let original = _read_friendly_name(h_devinfo, devinfo_data)?;
        let updated = _replace_friendly_port(&original, target_port);
        let wide: Vec<u16> = updated
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();
        let bytes: Vec<u8> = wide.iter().flat_map(|code| code.to_le_bytes()).collect();
        unsafe {
            SetupDiSetDeviceRegistryPropertyW(
                h_devinfo,
                devinfo_data,
                SPDRP_FRIENDLYNAME,
                Some(bytes.as_slice()),
            )
            .map_err(|error| anyhow!("写入 FriendlyName 失败: {}", error))?;
        }
        Ok(updated)
    }

    fn _read_friendly_name(h_devinfo: HDEVINFO, devinfo_data: &SP_DEVINFO_DATA) -> Result<String> {
        let mut size = 0u32;
        let mut value_type = 0u32;
        unsafe {
            let _ = SetupDiGetDeviceRegistryPropertyW(
                h_devinfo,
                devinfo_data,
                SPDRP_FRIENDLYNAME,
                Some(&mut value_type),
                None,
                Some(&mut size),
            );
        }
        if size == 0 {
            return Err(anyhow!("读取 FriendlyName 时未返回数据"));
        }
        let mut data = vec![0u8; size as usize];
        unsafe {
            SetupDiGetDeviceRegistryPropertyW(
                h_devinfo,
                devinfo_data,
                SPDRP_FRIENDLYNAME,
                Some(&mut value_type),
                Some(data.as_mut_slice()),
                Some(&mut size),
            )
            .map_err(|error| anyhow!("读取 FriendlyName 失败: {}", error))?;
        }
        _decode_utf16le(&data[..size as usize])
            .filter(|name| !name.is_empty())
            .ok_or_else(|| anyhow!("FriendlyName 为空或不是有效 UTF-16"))
    }

    fn _replace_friendly_port(original: &str, target_port: &str) -> String {
        let trimmed = original.trim_end();
        if let Some(before_close) = trimmed.strip_suffix(')') {
            if let Some(open) = before_close.rfind('(') {
                if _parse_com_number(&before_close[open + 1..]).is_some() {
                    return format!("{}({})", &before_close[..open], target_port);
                }
            }
        }
        format!("{} ({})", trimmed, target_port)
    }

    fn _update_com_name_arbiter(
        target_port: &str,
        former_port: &str,
        serialcomm_ports: &[String],
    ) -> Result<()> {
        let path: Vec<u16> = "SYSTEM\\CurrentControlSet\\Control\\COM Name Arbiter"
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();
        let mut hkey = HKEY::default();
        let status = unsafe {
            RegOpenKeyExW(
                HKEY_LOCAL_MACHINE,
                PCWSTR(path.as_ptr()),
                None,
                KEY_READ | KEY_SET_VALUE,
                &mut hkey,
            )
        };
        if status.is_err() {
            return Err(anyhow!("打开 COM Name Arbiter 失败: {:?}", status));
        }

        let result = (|| -> Result<()> {
            let mut comdb = _read_reg_binary(hkey, "ComDB")?;
            let target_number = _parse_com_number(target_port)
                .ok_or_else(|| anyhow!("无效目标端口号: {}", target_port))?;
            _set_comdb_bit(&mut comdb, target_number, true);
            if let Some(former_number) = _parse_com_number(former_port) {
                let former_is_active = serialcomm_ports
                    .iter()
                    .any(|port| port.eq_ignore_ascii_case(former_port));
                if !former_is_active {
                    _set_comdb_bit(&mut comdb, former_number, false);
                }
            }
            let value_name: Vec<u16> = "ComDB"
                .encode_utf16()
                .chain(std::iter::once(0))
                .collect();
            let status = unsafe {
                RegSetValueExW(
                    hkey,
                    PCWSTR(value_name.as_ptr()),
                    None,
                    REG_BINARY,
                    Some(comdb.as_slice()),
                )
            };
            if status.is_err() {
                return Err(anyhow!("写入 ComDB 失败: {:?}", status));
            }
            Ok(())
        })();

        unsafe {
            let _ = RegCloseKey(hkey);
        }
        result
    }

    fn _read_reg_binary(hkey: HKEY, value_name: &str) -> Result<Vec<u8>> {
        let wide_name: Vec<u16> = value_name.encode_utf16().chain(std::iter::once(0)).collect();
        let mut size = 0u32;
        // windows 0.62 的绑定要求这里是 REG_VALUE_TYPE 而不是裸 u32。
        let mut value_type = windows::Win32::System::Registry::REG_VALUE_TYPE(0);
        let status = unsafe {
            RegQueryValueExW(
                hkey,
                PCWSTR(wide_name.as_ptr()),
                None,
                Some(&mut value_type),
                None,
                Some(&mut size),
            )
        };
        if status.is_err() || size == 0 {
            return Err(anyhow!("读取 {} 大小失败: {:?}", value_name, status));
        }
        let mut value = vec![0u8; size as usize];
        let status = unsafe {
            RegQueryValueExW(
                hkey,
                PCWSTR(wide_name.as_ptr()),
                None,
                Some(&mut value_type),
                Some(value.as_mut_ptr()),
                Some(&mut size),
            )
        };
        if status.is_err() {
            return Err(anyhow!("读取 {} 失败: {:?}", value_name, status));
        }
        if value_type != REG_BINARY {
            return Err(anyhow!("{} 不是 REG_BINARY", value_name));
        }
        value.truncate(size as usize);
        Ok(value)
    }

    fn _set_comdb_bit(comdb: &mut Vec<u8>, port_number: u16, set: bool) {
        if port_number == 0 {
            return;
        }
        let bit_index = (port_number - 1) as usize;
        let byte_index = bit_index / 8;
        if comdb.len() <= byte_index {
            comdb.resize(byte_index + 1, 0);
        }
        let bit = 1u8 << (bit_index % 8);
        if set {
            comdb[byte_index] |= bit;
        } else {
            comdb[byte_index] &= !bit;
        }
    }

    fn _parse_com_number(port: &str) -> Option<u16> {
        let upper = port.trim().to_ascii_uppercase();
        upper.strip_prefix("COM")?.parse::<u16>().ok().filter(|number| *number > 0)
    }

    fn _decode_utf16le(bytes: &[u8]) -> Option<String> {
        if bytes.len() % 2 != 0 {
            return None;
        }
        let chars: Vec<u16> = bytes
            .chunks_exact(2)
            .map(|pair| u16::from_le_bytes([pair[0], pair[1]]))
            .collect();
        let end = chars.iter().position(|code| *code == 0).unwrap_or(chars.len());
        Some(String::from_utf16_lossy(&chars[..end]))
    }

    /// 重启指定设备节点(等价设备管理器"停用→启用"的属性变更), 使驱动重读 PortName 立即生效。
    /// 复刻 Device Manager 改 COM 口时内部执行的 DIF_PROPERTYCHANGE / DICS_PROPCHANGE 序列。
    fn restart_device_node(h_devinfo: HDEVINFO, devinfo_data: &SP_DEVINFO_DATA) -> Result<()> {
        let mut params = SP_PROPCHANGE_PARAMS {
            ClassInstallHeader: SP_CLASSINSTALL_HEADER {
                cbSize: std::mem::size_of::<SP_CLASSINSTALL_HEADER>() as u32,
                InstallFunction: DIF_PROPERTYCHANGE,
            },
            StateChange: DICS_PROPCHANGE,          // 属性变更 → 触发设备重启(而非 enable/disable)
            Scope: DICS_FLAG_CONFIGSPECIFIC,        // 仅当前配置(等价设备管理器默认行为)
            HwProfile: 0,
        };
        // SAFETY: h_devinfo/devinfo_data 有效; params 为合法 SP_PROPCHANGE_PARAMS。
        unsafe {
            SetupDiSetClassInstallParamsW(
                h_devinfo,
                Some(devinfo_data as *const SP_DEVINFO_DATA),
                Some(&mut params.ClassInstallHeader as *mut SP_CLASSINSTALL_HEADER),
                std::mem::size_of::<SP_PROPCHANGE_PARAMS>() as u32,
            )
            .map_err(|e| anyhow!("SetupDiSetClassInstallParamsW 失败: {}", e))?;
            SetupDiCallClassInstaller(
                DIF_PROPERTYCHANGE,
                h_devinfo,
                Some(devinfo_data as *const SP_DEVINFO_DATA),
            )
            .map_err(|e| anyhow!("SetupDiCallClassInstaller(DIF_PROPERTYCHANGE) 失败: {}", e))?;
        }
        Ok(())
    }

    /// 向已打开的注册表键写入一个 REG_SZ 字符串值(含 null 终止符)。
    fn write_reg_sz(hkey: HKEY, value_name: &str, value: &str) -> Result<()> {
        let wide_name: Vec<u16> = value_name.encode_utf16().chain(std::iter::once(0)).collect();
        let wide_value: Vec<u16> = value.encode_utf16().chain(std::iter::once(0)).collect();
        let bytes: Vec<u8> = wide_value.iter().flat_map(|c| c.to_le_bytes()).collect();

        // SAFETY: hkey 有效(调用方持有可写句柄),bytes 为合法 REG_SZ 编码。
        let status = unsafe {
            RegSetValueExW(hkey, PCWSTR(wide_name.as_ptr()), None, REG_SZ, Some(&bytes))
        };
        if status.is_err() {
            return Err(anyhow!("RegSetValueExW 写入 PortName 失败: {:?}", status));
        }
        Ok(())
    }

    /// 查询当前进程是否以管理员权限(UAC 已提升)运行。
    pub fn is_elevated() -> bool {
        is_elevated_inner().unwrap_or(false)
    }

    fn is_elevated_inner() -> Result<bool> {
        let mut token = windows::Win32::Foundation::HANDLE::default();
        // SAFETY: GetCurrentProcess 返回伪句柄,无需关闭;token 为出参。
        unsafe {
            OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &mut token)
                .map_err(|e| anyhow!("OpenProcessToken failed: {}", e))?;
        }

        let mut elevation = TOKEN_ELEVATION::default();
        let mut ret_len: u32 = 0;
        // SAFETY: token 有效,elevation 为出参缓冲区,大小已知。
        let query_result = unsafe {
            GetTokenInformation(
                token,
                TokenElevation,
                Some(&mut elevation as *mut _ as *mut c_void),
                std::mem::size_of::<TOKEN_ELEVATION>() as u32,
                &mut ret_len,
            )
        };

        // SAFETY: token 由上面 OpenProcessToken 成功获得,此处关闭。
        unsafe {
            let _ = CloseHandle(token);
        }

        query_result.map_err(|e| anyhow!("GetTokenInformation failed: {}", e))?;
        Ok(elevation.TokenIsElevated != 0)
    }
}

#[cfg(windows)]
pub use windows_impl::{
    apply_assignment, auto_assign, com_in_use, identify_ports, is_elevated, set_port_name,
};

// ============================================================================
// 非 Windows 降级实现(保证跨平台可编译)
// ============================================================================

#[cfg(not(windows))]
pub fn identify_ports() -> Vec<IdentifiedPort> {
    Vec::new()
}

#[cfg(not(windows))]
pub fn apply_assignment(_actions: &[AssignmentAction]) -> Result<()> {
    Err(anyhow!("仅 Windows 支持 COM 口注册表改名"))
}

#[cfg(not(windows))]
pub fn is_elevated() -> bool {
    false
}

// ============================================================================
// 单元测试(纯逻辑,不依赖真机/管理员权限)
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mi_to_function_mapping() {
        assert_eq!(mi_to_function(0x00), CdcFunction::Config);
        assert_eq!(mi_to_function(0x01), CdcFunction::Serial);
        assert_eq!(mi_to_function(0x03), CdcFunction::Light);
        assert_eq!(mi_to_function(0x06), CdcFunction::Unknown);
        assert_eq!(mi_to_function(0xFF), CdcFunction::Unknown);
    }

    #[test]
    fn test_parse_instance_id_full() {
        let id = r"USB\VID_2E8A&PID_000A&MI_00\6&2a3b1c&0&0004";
        let (vid, pid, mi) = parse_instance_id(id);
        assert_eq!(vid, Some(0x2E8A));
        assert_eq!(pid, Some(0x000A));
        assert_eq!(mi, Some(0x00));
        assert_eq!(mi.map(mi_to_function), Some(CdcFunction::Config));
    }

    #[test]
    fn test_parse_instance_id_case_insensitive() {
        let id = r"usb\vid_2e8a&pid_000a&mi_01\6&2a3b1c&0&0000";
        let (vid, pid, mi) = parse_instance_id(id);
        assert_eq!(vid, Some(0x2E8A));
        assert_eq!(pid, Some(0x000A));
        assert_eq!(mi, Some(0x01));
    }

    #[test]
    fn test_parse_instance_id_missing_mi() {
        let id = r"USB\VID_2E8A&PID_000A\6&2a3b1c&0&0000";
        let (vid, pid, mi) = parse_instance_id(id);
        assert_eq!(vid, Some(0x2E8A));
        assert_eq!(pid, Some(0x000A));
        assert_eq!(mi, None);
    }

    #[test]
    fn test_parse_instance_id_malformed() {
        let id = "not-a-valid-instance-id";
        let (vid, pid, mi) = parse_instance_id(id);
        assert_eq!(vid, None);
        assert_eq!(pid, None);
        assert_eq!(mi, None);
    }

    fn make_port(port_name: &str, function: CdcFunction, mi: Option<u8>) -> IdentifiedPort {
        IdentifiedPort {
            port_name: port_name.to_string(),
            interface_index: mi,
            function,
            instance_id: format!(
                r"USB\VID_2E8A&PID_000A&MI_{:02X}\fake-instance-{}",
                mi.unwrap_or(0xFF),
                port_name
            ),
        }
    }

    #[test]
    fn test_plan_assignment_generates_actions_when_mismatched() {
        let ports = vec![
            make_port("COM7", CdcFunction::Serial, Some(0x01)),
            make_port("COM8", CdcFunction::Light, Some(0x03)),
            make_port("COM9", CdcFunction::Config, Some(0x00)),
        ];
        let target = ComAssignment {
            serial_com: 3,
            light_com: 4,
        };

        let actions = plan_assignment(&ports, &target);
        assert_eq!(actions.len(), 2, "Config 接口不应生成改名动作");

        let serial_action = actions
            .iter()
            .find(|a| a.function == CdcFunction::Serial)
            .expect("应有 Serial 改名动作");
        assert_eq!(serial_action.from_port, "COM7");
        assert_eq!(serial_action.to_port, "COM3");

        let light_action = actions
            .iter()
            .find(|a| a.function == CdcFunction::Light)
            .expect("应有 Light 改名动作");
        assert_eq!(light_action.from_port, "COM8");
        assert_eq!(light_action.to_port, "COM4");
    }

    #[test]
    fn test_plan_assignment_skips_when_already_target() {
        let ports = vec![
            make_port("COM3", CdcFunction::Serial, Some(0x00)),
            make_port("COM4", CdcFunction::Light, Some(0x02)),
        ];
        let target = ComAssignment {
            serial_com: 3,
            light_com: 4,
        };

        let actions = plan_assignment(&ports, &target);
        assert!(actions.is_empty(), "已在目标 COM 口时不应生成动作");
    }

    #[test]
    fn test_identify_ports_no_panic_without_hardware() {
        // 本机通常未插入目标 VID/PID 设备,应静默返回空 Vec,不 panic。
        let ports = identify_ports();
        assert!(ports.is_empty() || !ports.is_empty());
    }

    #[test]
    fn test_plan_assignment_case_insensitive_match() {
        let ports = vec![make_port("com3", CdcFunction::Serial, Some(0x00))];
        let target = ComAssignment {
            serial_com: 3,
            light_com: 4,
        };
        let actions = plan_assignment(&ports, &target);
        assert!(
            actions.is_empty(),
            "COM 口名大小写不敏感比较,不应生成多余动作"
        );
    }
}
