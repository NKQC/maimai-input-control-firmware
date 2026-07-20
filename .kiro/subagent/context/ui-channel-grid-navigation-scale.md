# ui-channel-grid-navigation-scale

## 任务目标
修复全通道 6x6 网格定位与通道导航，展示逻辑分区绑定；统一单通道曲线尺度并增加标尺/网格/无数据提示；修正文案；使用固定 `dev.ps1 build-ui` 验证。

## 已完成步骤
1. `main.rs`：`ChannelStatus` 回填显式 `grid_col=ch%6`、`grid_row=ch/6` 及逻辑区绑定文本；逻辑区从 33 到 0 逆序扫描，未匹配为“未绑定”。
2. `app.slint`：全通道状态卡固定 124x76，以唯一的显式网格坐标排布在 768x492 的 6x6 容器；卡片展示 CH、raw/diff、触摸状态和逻辑区，且支持 hover、选中和点击。
3. `main.rs` / `app.slint`：新增 `channel_selected`、`settings_tab` 与 TabWidget `current-index` 双向绑定。点击卡片会设置 AppWindow `sel_channel` 并切到“单通道精调”。
4. `main.rs`：曲线重构为对所有已勾选且非空的 raw/baseline/diff 序列计算共享 min/max；添加 5% 边距（常量序列至少 ±1）；回填 min/mid/max、样本数和同尺度阈值位置。
5. `app.slint`：图表有独立绘图区、纵轴 max/mid/min 标尺、5 条横网格、较早/现在/样本提示以及“等待遥测数据”。
6. 模式文案已统一为“自动校准 / 半自动手动”，包含源代码注释也不再使用“全自动 SmartSense”。
7. 构建首次发现曲线结构路径字段移动后的借用错误，已改为先计算阈值位置再移动路径字段。
8. 已两次运行指定固定脚本，最终通过。

## 验证结果
- 命令：`powershell -ExecutionPolicy Bypass -File F:\mai2control\mai2control-v4\dev.ps1 build-ui`
- 结果：`Finished dev profile [unoptimized + debuginfo]`，退出码 0。
- `git diff --check` 发现的 trailing whitespace 位于既有 `hardware.txt` 和固件构建产物，不在本任务修改的 UI/Rust 文件中；未修改这些无关文件。

## 修改文件
- `control_software/src/main.rs`
- `control_software/ui/app.slint`
- `.kiro/subagent/context/ui-channel-grid-navigation-scale.md`（可恢复上下文）

## 下一步动作
任务完成；如需人工验收，可启动 UI 后从“触控通道调整”点击任一卡检查精调标签跳转和曲线视觉效果。
