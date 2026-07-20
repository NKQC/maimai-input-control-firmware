# phase3-stage2: 几何绘制圆形分区绑定 UI —— 完成

## 任务目标
重做上位机"分区绑定"页为纯 Slint 几何绘制的圆形 maimai 分区图 + 列表,
支持实时触摸高亮、选中、单分区通道编辑、指触绑定。对接 Stage1b 已就位的
后端(bind.mapNN 新语义=物理通道索引)。

## 状态: 全部完成 + 编译验证通过

## 涉及/改动文件

### control_software/ui/app.slint
- `ZoneCell` struct 改为 `{ index, label, ring, channel, touched, cx, cy }`
  (channel=-1 未映射; touched=绑定通道当前触摸中; cx/cy=画布 0..1000 归一化
  圆心坐标,由 main.rs 计算回填)。
- 删除旧 `BindingDetailPanel`(device_mask+24通道bit)、`ZoneRingRow`、
  `ZoneButton`。
- 新增 `BindingCanvas`(360x360 深色画布,内部按 cell.cx/cy 绝对定位画 34
  个小圆+分区名文字,叠加 3 个同心圆环参考线,纯 Rectangle/Text 绘制,无
  位图;TouchArea 点击→zone_selected)、`ZoneListRow`/`ZoneList`(右侧列表
  展示全部分区+通道号+触摸指示灯)、`BindingDetail`(选中分区详情:
  SpinBox 0..35 编辑通道 + "指触绑定" + "清除" 按钮)。
- `BindingPage` 重写:顶部"从设备加载"+状态文本;下方左画布+右详情/列表
  两栏布局。callback 改为 `bind_load()`、`zone_selected(int)`、
  `zone_channel_set(int,int)`、`bind_touch_start(int)`、`zone_unbind(int)`。
  删除 `bind_start()`(旧无参版)、`bind_confirm()`、`bind_abort()`、
  `zone_device_changed`、`zone_channel_toggled`。
- `SettingsPage`/`AppWindow`:同步更新 property(`selected_channel` 替代
  `selected_device_mask`/`selected_channel_bits`)与 callback 声明+透传,
  删除旧绑定相关声明,保留 `bind_load`。其它页/属性未动。

### control_software/src/main.rs
- 移除对 `binding_channel_mask`/`binding_device_mask`/`make_binding` 的
  import(不再使用,函数本体仍保留在 app_state/mod.rs 供其单测使用)。
- `on_zone_selected`:回填 `selected_zone_label` + `selected_channel`
  (用 `ctrl.binding_channel_of(zone)`,0xFF→-1)。
- 新增 `on_zone_channel_set(zone,channel)` → `ctrl.set_binding_channel`;
  `on_bind_touch_start(zone)` → `ctrl.bind_start`;
  `on_zone_unbind(zone)` → `ctrl.set_binding_channel(zone, 0xFF)`。
- 删除旧 `on_bind_start`(无参版)/`on_bind_confirm`/`on_bind_abort`/
  `on_zone_device_changed`/`on_zone_channel_toggled`。保留 `on_bind_load`。
- 新增 `zone_geometry(index) -> (f32,f32)`:按设计给定的环半径
  (A/D外环430,B/E内环240,C中心)与角度公式(D_k=-90+(k-1)*45,
  A_k/B_k=-90+22.5+(k-1)*45,E_k同D_k;C1=(560,500),C2=(440,500))
  用 Rust f32 三角函数计算画布 0..1000 坐标,不在 Slint 侧算三角函数。
- `build_zone_cells` 改签名为 `&AppController`,每次重建时用
  `binding_channel_of`/`telem_latest(ch).status bit0` 算 channel/touched,
  配合 `zone_geometry` 填 cx/cy。timer 每 tick 调用不变(`ctrl.poll()` 后
  `build_zone_cells(&ctrl)` → `set_zones`),故触摸高亮随 16ms tick 实时刷新。

## 未改动(确认不需要)
- 其它页(配置/曲线/全通道状态/仪表盘)/AppController 后端方法均不动,
  Stage1b 已就位的 `binding_channel_of`/`set_binding_channel`/`bind_start`/
  `telem_latest` 直接复用,无需新增后端接口。

## 编译/测试验证结果
- `dev.ps1 build-ui` → `Finished` dev profile,无 error(仅 PowerShell
  Select-Object 对 "Compiling" 行的无害 NativeCommandError 噪音)。
- `cargo test --lib`(control_software) → `70 passed; 0 failed`,含既有
  绑区测试(`test_zone_key_mapping`/`test_binding_map_*`/
  `test_set_binding_out_of_range_errors`/`test_handle_bind_event_sets_progress`
  /`test_binding_mask_roundtrip` 等)全部通过,未破坏任何既有测试(这些测试
  只依赖 app_state 层方法,未涉及本次改动的 UI 回调/main.rs 函数)。

## 关键决策
- 坐标计算放在 Rust 侧(f32 cos/sin)而非 Slint 表达式,任务说明里"优先在
  Rust 侧算好 cx/cy 传入(更稳)"已明确要求,避免 Slint Math.cos/sin 精度或
  语法坑。
- C1/C2 不走三角函数公式,直接给定坐标常量(560,500)/(440,500),因其角度
  定义在设计描述中是"左右半"而非环形角度序列,套通用公式反而不直观。
- BindingCanvas 内 for 循环体不接 if(Slint 既有坑,本工程其它组件已踩过),
  颜色/边框状态全部用三元表达式内联在属性绑定里,未新增辅助 if 元素。

## 无异常
未触发"连续失败超3次"或"架构冲突"情形,任务顺利完成,无需向上级报告异常。
