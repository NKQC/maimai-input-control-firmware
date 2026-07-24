# toolbox-port — 已完成（受外部接口阻塞）

- 目标：仅在 `control_software/src/app_state/mod.rs` 与 `control_software/src/main.rs` 接入工具箱 COM 口配置、持久化、自动/手动应用。
- 已完成：
  - `AppController` 新增 `toolbox_auto_port`、`toolbox_serial_com`、`toolbox_light_com`，默认 `false/3/21`，在 `new()` 中从私有 std::fs loader 初始化。
  - 增加私有 `toolbox.cfg` 路径、读取、写入辅助：优先程序目录，失败回退当前目录；读写/路径失败均 warn 且不 panic；格式为 `auto_port=0|1`、`serial_com=<u16>`、`light_com=<u16>`。
  - 增加 getter、写后保存的 setter、调用 `crate::comport::auto_assign` 并仅追加一次日志的 `apply_ports`，以及按开关调用的 `maybe_auto_assign_ports`。
  - 主程序在原设备刷新/自动连接初始化处回填三个 UI 属性、调用自动应用并以直接 `AppWindow` 引用更新状态；注册四个回调，手动应用使用 weak UI 回填状态。
  - 未修改 `app.slint`、`comport`、测试或 README。
- 验证结果（失败，已停止）：运行 `powershell -ExecutionPolicy Bypass -File dev.ps1 build-ui` 后得到实际 Rust 错误 `error[E0425]: cannot find function auto_assign in module crate::comport`，并注明 `crate::comport::windows_impl::auto_assign` 存在但 inaccessible；构建退出码 1，未产生 `Finished` 行。
- 阻塞决策：任务明确要求调用 `crate::comport::auto_assign`，但当前 `comport` 只在私有 Windows 子模块定义它且没有公开 re-export；任务明确禁止修改 `comport`，故未进行范围外修复。
- 未验证：未连接真实硬件，未验证管理员权限/注册表改名/重新插拔后的枚举生效。
