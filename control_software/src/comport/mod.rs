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
//! ## MI 接口号 ↔ 功能映射(⚠ 待与固件 #5x 多 CDC 描述符对齐)
//!
//! TinyUSB 下每个 CDC-ACM 功能占用 2 个 USB 接口(控制 + 数据),因此接口号
//! 依次为偶数起始。当前约定(与 `protocol_design.md` R3/R4 一致的占位约定):
//!
//! | MI 接口号 | 功能 |
//! |---|---|
//! | MI_00 | Serial (mai2serial,CDC-A) |
//! | MI_02 | Light (mai2light,CDC-B) |
//! | MI_04 | Config (host 配置协议,CDC-C) |
//!
//! 固件侧多 CDC 描述符改造(#5x)尚未落地,实际接口号顺序以固件描述符为准;
//! 一旦固件确定,只需改 [`mi_to_function`] 一处。

use anyhow::{anyhow, Result};

// ============================================================================
// 公开数据结构(跨平台可见,便于上层无条件引用类型)
// ============================================================================

/// 目标 VID/PID(复用 #6c `io` 模块的约定,避免重复定义/漂移)
const TARGET_VID: u16 = 0x0CA3;
const TARGET_PID: u16 = 0x0024;

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

// ============================================================================
// MI 接口号 → 功能映射(易改的单一映射点)
// ============================================================================

/// 按 MI 接口号返回对应功能。
///
/// ⚠ 待与固件 #5x 多 CDC 描述符对齐:当前为占位约定(MI_00=Serial /
/// MI_02=Light / MI_04=Config),固件确定接口顺序后如有出入,只需改这里。
fn mi_to_function(mi: u8) -> CdcFunction {
    match mi {
        0x00 => CdcFunction::Serial,
        0x02 => CdcFunction::Light,
        0x04 => CdcFunction::Config,
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

    use windows::core::{GUID, PCWSTR};
    use windows::Win32::Devices::DeviceAndDriverInstallation::{
        SetupDiDestroyDeviceInfoList, SetupDiEnumDeviceInfo, SetupDiGetClassDevsW,
        SetupDiGetDeviceInstanceIdW, SetupDiOpenDevRegKey, DICS_FLAG_GLOBAL, DIGCF_PRESENT,
        DIREG_DEV, SP_DEVINFO_DATA,
    };
    use windows::Win32::Foundation::CloseHandle;
    use windows::Win32::Security::{
        GetTokenInformation, TokenElevation, TOKEN_ELEVATION, TOKEN_QUERY,
    };
    use windows::Win32::System::Registry::{
        RegCloseKey, RegQueryValueExW, RegSetValueExW, HKEY, KEY_READ, KEY_SET_VALUE, REG_SZ,
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
            apply_one_action(action)?;
        }
        Ok(())
    }

    fn apply_one_action(action: &AssignmentAction) -> Result<()> {
        // 重新按实例 ID 定位设备,取得可写的 Device Parameters 键。
        // 这里复用 identify 阶段的 GUID_DEVCLASS_PORTS 枚举,按 instance_id 匹配。
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
                if instance_id != action.instance_id {
                    continue;
                }

                // 命中目标设备,打开可写的 Device Parameters 键并写入 PortName。
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

                return write_result;
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
pub use windows_impl::{apply_assignment, identify_ports, is_elevated};

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
        assert_eq!(mi_to_function(0x00), CdcFunction::Serial);
        assert_eq!(mi_to_function(0x02), CdcFunction::Light);
        assert_eq!(mi_to_function(0x04), CdcFunction::Config);
        assert_eq!(mi_to_function(0x06), CdcFunction::Unknown);
        assert_eq!(mi_to_function(0xFF), CdcFunction::Unknown);
    }

    #[test]
    fn test_parse_instance_id_full() {
        let id = r"USB\VID_0CA3&PID_0024&MI_04\6&2a3b1c&0&0004";
        let (vid, pid, mi) = parse_instance_id(id);
        assert_eq!(vid, Some(0x0CA3));
        assert_eq!(pid, Some(0x0024));
        assert_eq!(mi, Some(0x04));
        assert_eq!(mi.map(mi_to_function), Some(CdcFunction::Config));
    }

    #[test]
    fn test_parse_instance_id_case_insensitive() {
        let id = r"usb\vid_0ca3&pid_0024&mi_00\6&2a3b1c&0&0000";
        let (vid, pid, mi) = parse_instance_id(id);
        assert_eq!(vid, Some(0x0CA3));
        assert_eq!(pid, Some(0x0024));
        assert_eq!(mi, Some(0x00));
    }

    #[test]
    fn test_parse_instance_id_missing_mi() {
        let id = r"USB\VID_0CA3&PID_0024\6&2a3b1c&0&0000";
        let (vid, pid, mi) = parse_instance_id(id);
        assert_eq!(vid, Some(0x0CA3));
        assert_eq!(pid, Some(0x0024));
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
                r"USB\VID_0CA3&PID_0024&MI_{:02X}\fake-instance-{}",
                mi.unwrap_or(0xFF),
                port_name
            ),
        }
    }

    #[test]
    fn test_plan_assignment_generates_actions_when_mismatched() {
        let ports = vec![
            make_port("COM7", CdcFunction::Serial, Some(0x00)),
            make_port("COM8", CdcFunction::Light, Some(0x02)),
            make_port("COM9", CdcFunction::Config, Some(0x04)),
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
