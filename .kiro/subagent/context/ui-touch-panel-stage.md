# ui-touch-panel-stage

## 任务目标
实现 34 区真实 Slint Path 逻辑分区画布、分区/列表双向选择与有效绑定双击进入单通道精调；修复全局调整页裁切和通道卡跳转，并保证绑定等待状态蓝色闪烁、触摸绿色优先。

## 已完成步骤
1. 读取并恢复上一轮状态，核对 `app.slint`、`main.rs`、`app_state/mod.rs` 以及固件 `binding_service.cpp`。
2. 移除 `GlobalTunePage` 的三处固定 `height: 34px`，避免全局调整页在窄窗口中裁切。
3. 将 `zone_activated` 从 `BindingPage` 贯通到 `SettingsPage`/`AppWindow`/Rust。画布或列表双击有效绑定区时选中其物理通道并进入单通道精调 tab 3；全通道卡的 Rust 跳转也从错误 tab 2 修正为 tab 3。
4. 以 Rust 的 `ZoneGeometry` 替换旧圆心数据：构造 34 个 SVG 路径（A/D 交错外环、B/E 交错内环、C1/C2 中心半圆）、本区局部命中框、标签坐标、绑定文本和等待状态；Slint 绘制实际路径，触摸绿色优先于绑定蓝闪。
5. 修复 Slint 限制：Path 不能包含 TouchArea，改为本区透明 Rectangle 容器下的同级 Path/TouchArea。
6. 修复绑定状态机：`bind_start_seq` 初始化并实际用于 BIND_START ACK/NAK 关联；发起后进入 `(zone,0)` 蓝闪等待，ACK 保持等待，匹配 NAK 清除；BIND_EVENT 按固件真实 `[zone,channel,status]` 解码，`status=1` 同步 `bind.mapNN` 缓存并清除等待，非零终态和断连均清除。
7. 已从工程根执行 `powershell -ExecutionPolicy Bypass -File dev.ps1 build-ui`；首次报告 Path 不能有 TouchArea，修正后第二次构建退出码 0（`Finished dev profile`）。

## 关键结论/决策
- 固件只在绑定完成时发 `BIND_EVENT(zone, channel, 1)`，其 seq 固定为 0；BIND_START/BIND_ABORT 仅返回原请求 seq 的 ACK。
- UI 仅在本地 `bind_progress=(zone,0)` 时蓝闪；触摸态先判定为绿色。成功、失败 NAK、非零事件状态、断连都不会遗留闪烁。
- BIND_EVENT 成功时立即更新 UI 本地 map，无需额外从设备加载也会看到绑定通道。

## 待办步骤
无。

## 最终验证
`powershell -ExecutionPolicy Bypass -File dev.ps1 build-ui`：通过（exit code 0）。