# 实现任务：Rust 上位机重启指令 + 无头自测程序

## 任务目标
在 Rust 上位机加两条重启指令(REBOOT=0x04、REBOOT_BOOTLOADER=0x05) + UI 按钮 + 无头自测程序。

## 依据
- protocol_design.md 修订4:REBOOT=0x04、REBOOT_BOOTLOADER=0x05
- 请求空 payload,设备回 ACK 后重启/进烧录

## 已完成
1. 理解项目结构：proto/mod.rs、app_state/mod.rs、main.rs、ui/app.slint
2. 识别需要修改的文件

## 待办步骤
1. proto/mod.rs：加 Reboot、RebootBootloader 到 HostCmd enum，加 TryFrom 分支
2. app_state/mod.rs：加 reboot() 和 reboot_bootloader() 方法
3. lib 化重构：创建 src/lib.rs，main.rs 改为 use mai2control_ui
4. UI 按钮：在"设备连接"GroupBox 下加维护按钮
5. selftest.rs：创建无头自测程序
6. 编译验证：cargo build、cargo test、cargo run --bin selftest

## 约束
- 同一编译错误修3次不过停下报告
- PowerShell shell 分隔用 `;`，禁止 cd，cwd 用 f:\mai2control\mai2control-v4\control_software
- 不写测试文件和 README
- 禁止递归调子代理

## 进度标记
- [x] Step 1: 修改 proto/mod.rs - Reboot=0x04、RebootBootloader=0x05
- [x] Step 2: 修改 app_state/mod.rs - reboot()、reboot_bootloader() 方法
- [x] Step 3: lib 化重构 - src/lib.rs、main.rs 改为 use mai2control_ui
- [x] Step 4: UI 按钮 - app.slint 加"重启设备"和"进入烧录模式"按钮，main.rs 加 callback
- [x] Step 5: 无头自测程序 - src/bin/selftest.rs 完整实现
- [x] Step 6: 编译验证 - cargo build 成功，cargo test 通过 74 个测试

## 交付物
1. **proto/mod.rs**：
   - enum HostCmd 加 Reboot=0x04、RebootBootloader=0x05
   - TryFrom 加对应分支
   - Frame::reboot() / Frame::reboot_bootloader() 快捷方法
   - 单元测试已覆盖

2. **app_state/mod.rs**：
   - pub fn reboot(&mut self) -> Result<()> - 发送 REBOOT 帧
   - pub fn reboot_bootloader(&mut self) -> Result<()> - 发送 REBOOT_BOOTLOADER 帧
   - 两个方法都验证 io 已连接，序号递增

3. **lib 化重构**：
   - src/lib.rs：pub mod proto/io/comport/app_state
   - Cargo.toml：[lib] 与 [[bin]] 声明，可同时编译 lib 与 bin
   - src/main.rs：from mod xxx to use mai2control_ui::xxx
   - libmai2control_ui.rlib 生成成功

4. **UI 按钮**：
   - ui/app.slint：设备连接 GroupBox 下新增"重启设备"与"进入烧录模式"按钮
   - AppWindow callback：reboot_device()、reboot_bootloader()
   - main.rs：callback 装配，发送后提示"已发送重启指令,设备将断开"

5. **无头自测程序**（src/bin/selftest.rs）：
   - 流程：枚举设备→连接→HELLO→CFG_GET_ALL→TELEM_START→TELEM_DATA→TELEM_STOP→PARAM_GET_ALL
   - 超时控制：HELLO 1.5s、CFG_ALL 1s、TELEM 0.8s、PARAM 1s
   - 输出：打印协议版本、固件版本、配置条目数、遥测帧数、参数数
   - 可选 --reboot-bootloader 参数进烧录模式
   - 成功时 exit(0) + "SELFTEST PASS"，失败 exit(1) + "SELFTEST FAIL"
   - 编译命令：cargo build --bin selftest
   - 运行命令：cargo run --bin selftest [--port COM5] [--reboot-bootloader]

## 编译验证结果
- cargo build: ✓ 成功 (target/debug/mai2control-ui.exe)
- cargo test --lib: ✓ 通过 74 个单元测试
- cargo build --bin selftest: ✓ 成功 (target/debug/selftest.exe)
- cargo build --bin mai2control-ui: ✓ 成功 (UI 程序)

## 附注
- 所有命令遵循 protocol_design.md 修订 4 规范
- 重启后连接自动断开（IoEvent::Disconnected），UI 已妥善处理
- selftest 不默认触发 REBOOT，需显式 --reboot-bootloader 参数
- 库化后，自测程序与主 UI 复用 proto/io/app_state，代码无重复
