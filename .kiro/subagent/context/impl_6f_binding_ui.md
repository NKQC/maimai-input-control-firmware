# impl_6f_binding_ui — 完成状态

## 任务目标
在 Rust 上位机(control_software)实现 #6f 绑区可视化页(34 区映射展示 + 手动编辑),挂到主窗口 TabWidget 的"绑区" tab。

## 状态:已完成,cargo build / cargo test 均通过(61 tests ok)

## 涉及文件
- `control_software/src/app_state/mod.rs`:新增绑区辅助函数与 AppController 方法。
- `control_software/ui/app.slint`:新增 ZoneCell struct、ZoneButton/ZoneRingRow/BindingDetailPanel/BindingPage 组件,挂到 AppWindow "绑区" tab。
- `control_software/src/main.rs`:新增 build_zone_cells/push_zone_detail,回调装配,version-gated 重建 zones,bind_status 回填。

## app_state 绑区 API(均为 pub,复用 config_cache/set_config/next_seq)
- `zone_label(index) -> String`("A1".."E8")、`zone_key(index) -> String`("bind.map00".."bind.map33")(模块级自由函数)。
- `binding_channel_mask(v) -> u32`、`binding_device_mask(v) -> u8`、`make_binding(dev, ch) -> u32`(模块级自由函数)。
- `AppController::binding_map(&self) -> [u32; 34]`(缺失填 0xFFFFFFFF)。
- `AppController::get_binding(&self, zone) -> u32`。
- `AppController::set_binding(&mut self, zone, value) -> anyhow::Result<()>`(zone>=34 报错,内部走 set_config(bind.mapNN)).
- `bind_start/bind_abort/bind_confirm(&mut self) -> anyhow::Result<()>`:发 BindStart/Abort/Confirm 空 payload 帧,固件未注册这些 handler,会收 NAK → 落到已有 `_handle_nak` 记 last_error,不崩。
- `bind_progress(&self) -> Option<(u8,u8)>`:由新增 `_handle_bind_event` 在 handle_frame 分支解析 BindEvent(0x45) payload[0..2] 填充。
- 新增字段 `bind_progress: Option<(u8,u8)>`,初始化为 None。

## Slint 绑区页结构
- `ZoneCell { index, label, ring, device_mask, channel_mask, channel_count }`(ring="A".."E" 首字符,channel_count=popcount 由 Rust 侧算好传入,避免 Slint 里写位计数)。
- `BindingPage`:顶部 4 个按钮(加载/交互式绑定/确认/取消)+ bind_status Text;下方 5 个 `ZoneRingRow`(A-E)按 ring 过滤(visible+width=0 隐藏范式,同 ConfigGroupSection);选中区域下方 `BindingDetailPanel`。
- **编辑方式选择:CheckBox 网格(直观方案已实现,非退化输入框)**。detail panel = SpinBox(设备掩码 0-255) + 24 个 CheckBox(bit0..bit23,4行x6列用 for+取模筛选)。
- 回调:`zone_selected(int)`、`zone_device_changed(int,int)`、`zone_channel_toggled(int,int,bool)`、`bind_load/start/confirm/abort()`。
- AppWindow 顶层新增 `zones`/`bind_status`/`selected_zone`(in-out)/`selected_zone_label`/`selected_device_mask`/`selected_channel_bits` 属性 + 上述回调,转发给内部 BindingPage 实例;原"绑区" tab 占位 Text 已替换为 BindingPage。

## main.rs 装配
- `build_zone_cells(ctrl) -> Vec<ZoneCell>`:从 binding_map() 构造 34 个 cell。
- `push_zone_detail(window, ctrl, zone)`:回填 selected_zone/selected_zone_label/selected_device_mask/selected_channel_bits(24 bool VecModel)。
- 回调装配:bind_load→request_config_all;bind_start/confirm/abort→AppController 对应方法(失败仅 log::error,不 panic);zone_selected→push_zone_detail;zone_device_changed/zone_channel_toggled→算新 u32(make_binding)→set_binding→push_zone_detail 刷新面板。
- Timer 轮询:config_version 变化时同时重建 config_rows 与 zones(共用同一 config_cache/版本号),并在详情面板打开时用新数据刷新;每帧用 bind_progress() 回填 bind_status 文本(None 时显示"未在绑定(BIND_* 待固件支持)")。

## BIND_* 就绪状态
固件 host_cmd.cpp dispatcher 只注册了 HELLO/PING/CFG_*/SAVE_CONFIG/RESET_DEFAULTS,BindStart/Abort/Confirm/GetMap/SetMap/Event(0x40-0x45)均未注册 → 收到会走 `_handle_not_implemented` 回 NAK(NOT_IMPLEMENTED, "TODO: #5b/#5c")。UI 侧已按此设计:调用 bind_start/abort/confirm 发帧后遇 NAK 只记录 last_error,不影响 UI 稳定;bind.mapNN 本身走已实现的 CFG_GET/CFG_SET,可正常读写。

## 验证结果
- `cargo build`:成功,仅有与本次改动无关的既有 dead_code/unused warning(comport/io 模块),已确认非本次引入。
- `cargo test`:61 passed,0 failed。新增测试覆盖:zone_label/zone_key 映射、binding_channel_mask/device_mask/make_binding 往返、binding_map 默认值与读缓存、set_binding 越界报错、handle_bind_event 进度解析、build_zone_cells 数量=34 与字段(默认值/读缓存两种场景)。
- 未运行 GUI(按要求,仅 build/test 验证)。

## 给 #6g 曲线 tab 的现状
- 主窗口 TabWidget 第三个 tab "曲线" 仍是占位 Text("曲线页面(#6g 后续实现)"),挂载点未变,可直接参照 ConfigPage/BindingPage 的 struct+for+回调透传范式实现。
- PARAM_*(0x20-0x24)命令同样未在固件 dispatcher 注册,会 NAK,#6g 实现时需同样做优雅处理。
- TELEM_*(0x30-0x32)遥测流命令固件也未注册。
