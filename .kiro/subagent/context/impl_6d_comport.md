# impl_6d_comport — 任务完成记录

## 任务目标
在 `control_software`(Rust 上位机)实现 #6d:Windows COM 口识别(按 USB 接口号 MI_xx 映射功能)+
serial/light 自动改名到指定 COM(注册表)。

## 状态:已完成,编译与单测均通过

## 架构结论(来自 protocol_design.md R3/R4 + #6c io/mod.rs)
- VID=0x0CA3 / PID=0x0024,多 CDC 复合设备。
- MI 接口号 → 功能映射(占位约定,**待与固件 #5x 多 CDC 描述符对齐**):
  - MI_00 → Serial (mai2serial, CDC-A)
  - MI_02 → Light (mai2light, CDC-B)
  - MI_04 → Config (host 配置协议, CDC-C)
- serial/light 需改名到街机侧要求的固定 COM 号(注册表 `Device Parameters\PortName`);
  config 不需要固定 COM,UI 识别后直接打开即可。
- 不动 `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM`(系统自维护,易失)。

## 已完成步骤
1. 读取现状:`src/comport/mod.rs`(占位)、`src/io/mod.rs`(#6c 风格与 DeviceCandidate/list_devices)、
   `protocol_design.md` R3/R4、`Cargo.toml`(windows 0.62.2 default-features=false 零 feature)。
2. 查阅 windows-rs 0.62.2 官方文档确认签名:`SetupDiGetClassDevsW`/`SetupDiEnumDeviceInfo`/
   `SetupDiGetDeviceInstanceIdW`/`SetupDiOpenDevRegKey`/`SetupDiDestroyDeviceInfoList`/
   `RegQueryValueExW`/`RegSetValueExW`/`RegCloseKey`/`OpenProcessToken`/`GetTokenInformation`/
   `GetCurrentProcess`/`GUID::from_u128`/`TOKEN_ELEVATION`。
3. Cargo.toml 补充 windows feature(见下)。
4. 实现 `src/comport/mod.rs`:跨平台公开类型 + `#[cfg(windows)] mod windows_impl` 真实
   SetupAPI/注册表实现 + `#[cfg(not(windows))]` 降级空实现 + 内联单测。
5. `cargo build`(全量)一次性通过(仅 dead_code 警告,因未接入 UI,预期内);
   `cargo test`(全量 23 个测试)全部通过。

## 实际启用的 windows crate feature
```toml
windows = { version = "0.62.2", default-features = false, features = [
    "Win32_Foundation",
    "Win32_Devices_DeviceAndDriverInstallation",
    "Win32_Devices_Properties",
    "Win32_System_Registry",
    "Win32_Security",
    "Win32_System_Threading",
] }
```
(`Win32_Devices_Properties` 实际未直接用到具体符号,留作后续设备属性查询扩展点;若追求最小化可后续裁剪,当前不影响编译体积可接受)

## 公开 API(`src/comport/mod.rs`)
- `pub enum CdcFunction { Serial, Light, Config, Unknown }`
- `pub struct IdentifiedPort { port_name, interface_index: Option<u8>, function, instance_id }`
- `pub struct ComAssignment { serial_com: u16, light_com: u16 }`
- `pub struct AssignmentAction { instance_id, from_port, to_port, function }`
- `pub fn identify_ports() -> Vec<IdentifiedPort>`(只读,SetupAPI 枚举 Ports 类,过滤 VID/PID,解析 MI,读 PortName)
- `pub fn plan_assignment(ports: &[IdentifiedPort], target: &ComAssignment) -> Vec<AssignmentAction>`(纯逻辑 dry-run)
- `pub fn apply_assignment(actions: &[AssignmentAction]) -> Result<()>`(需管理员,写注册表 PortName)
- `pub fn is_elevated() -> bool`(查当前进程是否管理员令牌)
- 非 Windows:`identify_ports` 返回空 Vec,`apply_assignment` 返回 Err("仅 Windows 支持..."),`is_elevated` 返回 false。

## 验证结果
- `cargo build`:成功,exit code 0,仅既有 dead_code 警告(comport/io 未接入 main.rs 正常现象)。
- `cargo test`(全量):23 passed, 0 failed。其中 comport 模块 9 个测试:
  - MI→功能映射(mi_to_function)
  - 实例 ID 解析(完整/大小写不敏感/缺 MI/畸形)
  - plan_assignment(生成动作/已在目标跳过/大小写不敏感比较)
  - identify_ports 本机无设备不 panic(返回空 Vec)

## 只能真机+管理员验证的清单(未做,已在代码注释与此文档明确标注)
1. `identify_ports()` 在真实插入 VID_0CA3/PID_0024 多 CDC 设备时,能否正确枚举出各 COM 口
   并按 MI 解析出正确接口号/功能(**当前固件尚未做多 CDC 描述符改造 #5x,硬件侧不存在该多 CDC 设备**)。
2. `apply_assignment()` 真正写注册表 `PortName` 后,设备重插/重枚举是否生效为目标 COM 号
   (需管理员权限运行 + 真机 + 重插)。
3. 目标 COM 号被其他设备占用时的冲突表现(冲突检测本任务未做,代码注释已标 TODO)。
4. `is_elevated()` 在非管理员/管理员两种真实进程权限下的实际返回值(逻辑已实现,依赖 Windows
   UAC token,本地未做双重身份对比测试,仅信任 windows-rs 官方 API 语义)。

## 无异常
本次任务未触发"连续失败超 3 次"或"架构冲突"等异常情况,一次实现即通过编译与单测。
