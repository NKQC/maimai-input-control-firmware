# 任务: ui-modernize (Phase 1)

## 目标
把上位机 UI 从单窗口 TabWidget 重构为现代化布局：左侧菜单(主页/设置) + 设置顶部子标签。
复用现有可用页面组件(ConfigPage/BindingPage/CurvesPage/AllChannelsPage)，勿推倒重来。
后端 AppController API 全部保留复用，不改协议。

## 布局
AppWindow root = HorizontalBox:
- 左侧导航栏(Rectangle 宽~160px, 深色 #1a1e22): 两个导航项"主页""设置"(可加简单图标字符)，
  选中高亮。属性 `current_view:int`(0=主页,1=设置)，点击切换。底部放连接状态小圆点。
- 右侧主区(stretch): current_view==0 显示 DashboardPage; ==1 显示 SettingsPage。

## DashboardPage(主页总控) 卡片式(圆角 Rectangle 卡):
1. 连接卡: device_labels 下拉 + selected_device + 刷新/连接/断开 + 重启/进烧录 按钮 + conn_status 文本。(复用现有 refresh/connect_clicked/disconnect_clicked/reboot_device/reboot_bootloader 回调)
2. 系统状态卡: device_info_text(多行) + CSD 模式切换 ComboBox["全自动","半自动"](绑 scan_mode + curve_mode_changed 回调，复用)。
3. 采样卡: sample_rate_hz + channel_delay_us(大字展示)。
4. 延迟卡: 属性 latency_total_us/latency_spi_us/latency_proc_us/latency_usb_us/latency_sensor_us(int,默认0) + measure_latency(bool in-out) + 回调 latency_measure_toggled(bool)。展示总延迟与各段组成(SPI读/RP处理/USB写/传感器年龄)。Phase2 由固件填值,现在先展示0并可切开关。

## SettingsPage(设置) = TabWidget 子标签:
- "分区绑定": 现有 BindingPage(原样挂,回调不变)
- "触控通道调整": 现有 AllChannelsPage(全通道实时+采样率;原样)
- "单通道精调": 现有 CurvesPage(曲线+参数+阈值+模式;原样)
- "触控键盘映射": 新 KeyMapPage —— 占位:展示一行提示"需固件支持(触控→键盘映射)",并把现有配置键 comm.keyboard_map_en 用一个 CheckBox 暴露(通过 cfg_set_bool)。不要伪造多组合/延迟 UI。
- "物理键盘": 新 PhysKbdPage —— 占位:提示"需固件支持(GPIO1-12 物理键盘)"。
- "通信/系统": 现有 ConfigPage(通信/模式/灯效 分组;原样)

## 约束
- 保留 app.slint 里所有 export struct(ConfigRow/ZoneCell/ParamRow/ChannelStatus)与所有现有组件及其回调签名。
- AppWindow 所有现有 in property / callback 保留(main.rs 已在用)；只新增: current_view, 延迟卡属性/回调。
- main.rs: 新增 current_view 的处理(可纯 Slint 内部状态,不必回 Rust)；延迟卡回调 latency_measure_toggled 先接一个 AppController 开关方法(Phase2 加,现在可先留空 println 或 no-op TODO)。其余 per-tick 回填逻辑全部保留。
- 现代深色主题:卡片 Rectangle border-radius 8px, 背景 #23282e, 主区背景 #15181c, 文字浅色。
- 必须 `dev.ps1 build-ui` 通过(Finished 无 error)。cargo test 若有 UI 相关不得破坏。

## 状态
- [x] app.slint 重构(左栏+主页+设置容器)
- [x] main.rs 接线(current_view + 延迟卡回调占位 + 保留原回填)
- [x] build-ui 通过

## 完成记录
- app.slint: 新增 DashboardPage(连接/系统状态/采样/延迟 4 卡,圆角8px #23282e)、
  KeyMapPage(comm.keyboard_map_en CheckBox 占位)、PhysKbdPage(纯提示占位)、
  SettingsPage(TabWidget 6 标签: 分区绑定/触控通道调整/单通道精调/触控键盘映射/物理键盘/通信系统,
  原样透传挂载 BindingPage/AllChannelsPage/CurvesPage/KeyMapPage/PhysKbdPage/ConfigPage)。
  重写 AppWindow 顶层为 HorizontalBox(左栏 160px #1a1e22 导航"主页"/"设置"+底部连接圆点;
  右侧 if current_view==0 DashboardPage else SettingsPage)。删除原顶层 GroupBox+TabWidget。
  新增 AppWindow property: current_view/latency_total_us/latency_spi_us/latency_proc_us/
  latency_usb_us/latency_sensor_us/measure_latency/keyboard_map_en；新增 callback:
  latency_measure_toggled(bool)/set_keyboard_map_en(bool)。所有既有组件/struct/property/callback
  签名均未改动。
- main.rs: 新增 on_set_keyboard_map_en(bool) → ctrl.set_config(comm.keyboard_map_en, Bool);
  新增 on_latency_measure_toggled(bool) → info! 日志 TODO Phase2 (no-op); 在 config_version
  变化块内新增 ui.set_keyboard_map_en 回填(从 config_cache 读 comm.keyboard_map_en)。其余
  set_*/on_* 绑定与 per-tick 回填逻辑全部保留原样。
- 验证: `dev.ps1 build-ui` -> `Finished \`dev\` profile [unoptimized + debuginfo] target(s) in 12.81s`,
  无 error,Exit Code 0。`cargo test --lib` -> 70 passed; 0 failed。均通过。

## 任务完成,无异常。

## 进度记录 (module-builder 接手后)
- 已完整读 app.slint(全) 与 main.rs(全) 与 app_state/mod.rs(全),确认现有 property/callback 清单与 comm.keyboard_map_en 已在固件侧存在(app_config.cpp 已注册)。
- 下一步:按方案往 app.slint 追加 DashboardPage/KeyMapPage/PhysKbdPage/SettingsPage,并重写 AppWindow 顶层可视树 + 新增 property/callback。
- 计划:main.rs 新增 on_set_keyboard_map_en / on_latency_measure_toggled(no-op log) + per-tick 在 config_version 变化块内 set_keyboard_map_en 回填。
- 尚未动手改文件。
