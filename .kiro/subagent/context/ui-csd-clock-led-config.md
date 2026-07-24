# ui-csd-clock-led-config

## 任务目标
在 control_software UI 中完成 GlobalTune global 1..6、全 36 通道统一 CSD 采样设置与时钟树展示，并将 LED 亮度/颜色配置以中文友好控件呈现；不改固件。

## 已完成步骤
1. 已恢复既有任务状态，并保留前阶段成功的 LED 颜色 kind=4、中文说明、GlobalTune 主体、路径功能和已有协议常量导入。
2. `ui/app.slint` 已在 `SettingsPage`、`AppWindow` 和二者对应实例之间透传：`g_mfs_div_f1/f2`、`csd_sns_clk_div`、`csd_resolution`、`csd_sns_clk_source`、`csd_param_set`、`csd_refresh`。
3. `main.rs` 已将 `csd_param_set` 绑定至既有 `AppController::set_param_all`，将 `csd_refresh` 绑定至 `request_params(0)`。
4. Global 回填已扩展为 1..6，新增 global 5/6 → `set_g_mfs_div_f1/f2`。
5. 参数版本变更时，已从 CH0 回填 `PARAM_SNS_CLK_DIV` (0x08)、`PARAM_RESOLUTION` (0x07)、`PARAM_SNS_CLK_SOURCE` (0x0A) 到全局页属性。
6. 连接成功时已显式额外请求 CH0 参数；真正进入已连接的“触控全局调整”页时只按可见性边沿刷新一次，避免 16ms tick 洪泛。
7. 首轮构建发现 Slint 不接受 `0x08/0x07/0x0A` 回调字面量及重复 callback 声明；已分别改为十进制 `8/7/10` 并去重。
8. 验证通过：工程根执行 `powershell -ExecutionPolicy Bypass -File dev.ps1 build-ui` 成功，Cargo dev profile 完成（18.42s）。

## 关键结论/决策
- 未创建新协议或控制器接口；全部复用既有 `GLOBAL_*`、参数常量和 `set_param_all`。
- 全局页固定显示 CH0 代表值；进入 tab 不会逐 tick 发请求。
- 仅编辑 `control_software/ui/app.slint` 与 `control_software/src/main.rs`；未改固件/PSoC、测试或 README。Path 实现未处于编辑范围，且最终 Slint/Rust UI 构建成功。

## 待办步骤
无。

## 下一步动作
如需真机验证：连接设备后进入“触控全局调整”，确认 Global 5/6、CH0 代表值刷新与“全 36 通道统一”写入效果。
