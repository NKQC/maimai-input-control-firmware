# impl_6a_rust_scaffold 进度记录

## 任务目标
从零 scaffold 一个 Rust + Slint 桌面上位机工程(`control_software/`,package `mai2control-ui`,binary):
- `cargo build` 通过
- 能起一个 Slint 空窗
- 建立模块骨架:`src/proto/mod.rs`(#6b占位)、`src/io/mod.rs`(#6c占位)、`src/comport/mod.rs`(#6d占位)、`src/app_state/mod.rs`(#6e~g占位)
- 依赖:slint, serialport, windows(暂不启用具体feature), anyhow, log+env_logger; build-dependencies: slint-build
- build.rs 用 slint_build::compile("ui/app.slint")
- .gitignore 忽略 target/

## 整体架构结论(已读 protocol_design.md + host_cmd.h)
- 链路A(host⇄RP2040): USB CDC 二进制帧, SOF 0xAA55 + cmd + flags + seq + len(u16 LE) + payload + crc16(CCITT-FALSE, poly 0x1021, init 0xFFFF, 覆盖 cmd..payload)。
- 命令码分区见 protocol_design.md 1.3 节(0x01-0x0F系统/0x10-0x1F配置KV/0x20-0x2F CapSense调参/0x30-0x3F遥测流/0x40-0x4F绑区/0x50-0x5F灯效/0x7E-0x7F应答),与固件 host_cmd.h 的 HostCmd enum 一致,#6b 需镜像该 enum。
- USB 设备多CDC复合: VID 0x0CA3 / PID 0x0024,CDC-A=mai2serial,CDC-B=mai2light,CDC-C=config(UI用),通过设备实例ID里的 `&MI_xx` 接口号映射功能(#6d)。
- Windows COM 分配需改注册表 PortName + SERIALCOMM(#6d,需管理员权限)。
- UI 框架定案 Slint(非 egui);线程模型 UI线程(Slint事件循环) + IO线程(串口) + mpsc/channel。

## 当前状态:【任务完成 - cargo build 成功】

`cargo build` 输出 `Finished \`dev\` profile [unoptimized + debuginfo] target(s) in 1m 35s`,退出码 0,无 error。工程结构、依赖、模块骨架均按计划落地。任务已交付,无待办。

### 已完成步骤
1. Rust 环境确认:rustc 1.97.0 / cargo 1.97.0,均可用。
2. `cargo init --name mai2control-ui --bin .` 于 `control_software/`,edition 自动为 2024(cargo 1.97 默认)。
3. `cargo add slint serialport anyhow log env_logger` 成功,实际拉取版本:
   - slint 1.17.1(features: accessibility/backend-default/compat-1-2/renderer-femtovg/renderer-software/std/system-tray)
   - serialport 4.9.0(feature: libudev)
   - anyhow 1.0.103
   - log 0.4.33
   - env_logger 0.11.11
4. `cargo add --no-default-features windows` 成功,拉到 windows v0.62.2(0 feature 启用,673 deactivated,避免默认 feature 编译失败风险);`cargo add --build slint-build` 成功,slint-build 1.17.1。
5. 写入文件:
   - `build.rs`(slint_build::compile("ui/app.slint"))
   - `ui/app.slint`(AppWindow,占位 Text "mai2control v4 — Host UI",900x600/min 640x480)
   - `src/main.rs`(mod 声明 + slint::include_modules!() + env_logger::init + AppWindow::new()?.run()?)
   - `src/proto/mod.rs`、`src/io/mod.rs`、`src/comport/mod.rs`、`src/app_state/mod.rs`(均为占位 + doc注释,引用架构结论)
   - `.gitignore`(/target)

### 待办步骤
1. 运行 `cargo build` 验证是否成功(error 不可接受,warning 可接受)。
2. 若 windows crate 因 default-features=false 导致某处仍报错(理论上不应,因为尚未 `use windows::...`),排查并记录。
3. 若 build 成功,汇报最终交付(结构树/依赖版本/windows feature 现状/#6b~#6d 接入点/build 结果)。
4. 若 build 失败,按"同一错误修3次不过即停"处理,记录报告。

## 关键决策(供续做时遵循,勿重新决策)
- package 名:`mai2control-ui`,binary。
- windows crate 采用 `default-features = false`,当前 0 feature 启用,待 #6d 实现时按需追加(预计 Win32_Devices_DeviceAndDriverInstallation、Win32_System_Registry 等)。
- 模块占位内容对齐"整体架构结论",不暗猜协议字段。
- 只搭骨架,不实现协议编解码/串口IO/COM识别/UI业务逻辑本体,函数体用 `// TODO #6x` 或最小占位保证编译通过。
