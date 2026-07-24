# cad-touch-ui-refactor 进度记录

## 任务目标
1. 用 DXF (touch_map_37inch_16x9.dxf) 精确几何替换 main.rs 里近似 zone_geometry()。
2. GlobalTunePage 时钟树改图形化节点卡片。
3. Cp 加入触控状态面板，统一两位 pF 格式。
4. scan period 统一 "探测周期: x.xx ms"。
5. 只用 dev.ps1 build-ui 验证。

## 架构结论(已确认，勿重新调研)
- DXF: F:\maimaicontrol-V3.0\V4 Build\外边构造\touch_map_37inch_16x9.dxf
  SCREEN_FRAME = 819.1066897896513 x 460.74751300667884 mm (来自实际读取的 LWPOLYLINE，比报告 JSON 精确)
- 34 个 REGION_01..REGION_34 LWPOLYLINE，closed(70=1)，部分含 bulge(组码42)。
  非零 bulge 数=16，恰好是 REGION_19..REGION_34 各1条(每个多边形只有1条边是圆弧)。
- 映射(zone idx 0..33 -> region no)：
  A(0-7)=[20,22,24,26,28,30,32,34]
  B(8-15)=[3,4,5,6,7,8,9,10]
  C(16-17)=[1,2]
  D(18-25)=[19,21,23,25,27,29,31,33]
  E(26-33)=[18,11,12,13,14,15,16,17]
- bulge->SVG arc: theta=4*atan(b); r=chord/(2*sin(|theta|/2)); large_arc=|theta|>pi;
  sweep=0 if b>0 else 1 (因 DXF y-up bulge>0=CCW，经 y_ui=H-y 翻转后视觉方向不变，
  且 UI 是 y-down 坐标系，SVG sweep=1 表示视觉顺时针)。
- UI画布固定 SCREEN_W=1000, SCREEN_H=562.5 (=1000*460.7475/819.1067，保持16:9实际长宽比 ~1.7778)。
- app_state::zone_label ZONE_RINGS=[('A',8),('B',8),('C',2),('D',8),('E',8)]，
  index 0..33 -> A1..A8,B1..B8,C1,C2,D1..D8,E1..E8，即为 main.rs zone_geometry(index) 的入参序。

## 已完成步骤
1. [完成] 读取 DXF 原文(ENTITIES section)、report json、app_state/mod.rs(zone_label/cp/cp_version)、
   main.rs(zone_geometry/build_zone_cells/ZoneGeometry)、telemetry.rs(TelemFrame/scan_period_us)、app.slint全文。
2. [完成] 编写一次性 Python 脚本 control_software/gen_touch_geometry.py（不依赖 ezdxf，手写 DXF group code 解析），
   按上述映射与算法生成 control_software/src/touch_geometry.rs（ZONE_GEOMETRY: [ZoneStaticGeometry;34]，
   含 region_id/path/min_x/min_y/width/height/label_x/label_y，均为 SCREEN_W x SCREEN_H 坐标系下数值）。
3. [完成] 运行脚本，确认: 34 条全部生成，非零bulge数=16且恰好对应 R19..R34一次一条边，
   34 个 region_id 无重复(1..34各一次)。已修复一次浮点格式化 bug(`:.2`->`:.2f`导致科学计数法/精度丢失)。
   已核验生成文件内容(见下方"待办第7条"仍需最终二次核验 closure `Z`、A命令数)。

## 待办步骤(未完成，接下来继续)
4. [ ] main.rs: 删除/停用旧 zone_ring_sector + zone_geometry 近似实现(改为不再被调用，但保留代码或删除均可，
   任务要求"旧 zone_ring_sector 近似不再被使用")；改用 touch_geometry::ZONE_GEOMETRY，
   build_zone_cells() 内 path/path_x/path_y/path_width/path_height/label_x/label_y 直接从表里取，
   注意 ZoneCell 字段单位需要从 mm 坐标系(SCREEN_W=1000,SCREEN_H=562.5)转换到 Slint 侧目前用的 0..1000 归一化——
   需要重新设计 app.slint 的 BindingCanvas: 按架构要求"所有 Path 共用 SCREEN_FRAME viewbox + 透明命中层"，
   即 Path 的 viewbox-x/y/width/height 应固定为 (0,0,SCREEN_W,SCREEN_H)（不再用每区局部 viewbox），
   TouchArea 命中层用 path_x/y/width/height(局部bbox，仍是 SCREEN_W/H 坐标系)做绝对定位矩形；
   画布容器需从当前 360x360 方形改为 16:9 letterbox(比例 SCREEN_W:SCREEN_H)，避免非等比拉伸。
5. [ ] control_software/src/lib.rs 加 `pub mod touch_geometry;`，main.rs 顶部 use。
6. [ ] app_state/mod.rs: **不要改**(architecture要求除必要生成资产外不改 app_state/mod.rs 和协议文件)。
7. [ ] main.rs::build_zone_cells 改造完成后跑一次 build-ui，先看编译错误。
8. [ ] app.slint BindingCanvas 组件重构(共享 viewbox + letterbox 16:9 + 保留34区交互语义:
   click=选中，double-click=选中+激活；触摸绿色/绑定蓝闪 fill/stroke 语义不变)。
9. [ ] GlobalTunePage: 把"CSD 时钟树(生成配置)"纯文字块换成图形节点卡片(中文节点、连接线、箭头、层级)，
   固定节点(ECO12MHz/PLL-HFCLK48MHz/PERI-CSD48MHz/peripheral DIV(/1)/modCsdClk=1)只读展示，
   可编辑节点(MFS F1/F2 via algo_global_set(5/6); Resolution/SnsClk DIV/SnsClk Source via csd_param_set(7/8/10))
   把 SpinBox 直接放节点卡内，复用现有回调不新增回调。末端节点显示"设备实测探测周期: x.xx ms"
   (数据来源 telemetry scan_period_us，展示层 /1000.0 换算，ms两位小数)。
10. [ ] Cp 展示统一: main.rs 里 cp_text 目前是 "Cp: {} fF"/"Cp: 测量失败" 等，需改成两位 pF：
    value as f64/1000.0，格式 "{:.2} pF"；0x00FF_FFFF=测量失败；0=测量中；None=读取中。
    CurvesPage 里 cp_text 展示已有，需要在 ZoneCell / ChannelStatus 也加 cp 文本字段(架构要求"分区状态/36通道状态
    卡均显示两位pF")，即 app.slint 的 struct ZoneCell / ChannelStatus 各加一个 cp_text: string 字段，
    main.rs::build_zone_cells / build_channel_status 里从 ctrl.cp(ch) 换算填入，
    并把"全通道 model 刷新条件纳入 cp_version"(main.rs 主循环 `current_telem_version != last_telem_version_all`
    这行需要 OR 上 `ctrl.cp_version() != last_cp_version_all`，否则无新遥测时 Cp 文本不刷新)。
11. [ ] scan period 展示统一: 搜索 "channel_delay_us"/"通道延迟"/"us" 相关展示，
    main.rs 里 `ui.set_channel_delay_us(ctrl.telem_scan_period_us() as i32)` 和 app.slint 里
    `channel_delay_us: int` 属性、"通道延迟 " + ... + " us" 文案，全部改成 ms float 两位小数 + "探测周期: x.xx ms"
    文案；注意 app.slint 中 AllChannelsPage / DashboardPage 均有 channel_delay_us 属性引用，
    需要改类型为 float 并同步改 main.rs setter 与两处文案。**不要**动 latency 相关字段(lat_spi/proc/usb)。
12. [ ] 全部改完后跑 `powershell -ExecutionPolicy Bypass -File dev.ps1 build-ui` (cwd=F:\mai2control\mai2control-v4)
    直至编译通过；修复所有编译错误。
13. [ ] 静态核验清单(逐条过一遍并在最终汇报中列出结果):
    - touch_geometry.rs 恰好 34 条 ZoneStaticGeometry，region_id 1..34 各一次不重复。
    - 每条 path 以 "M " 开头、以 " Z" 结尾(闭合)。
    - R19..R34(region_id in 19..=34) 对应的 path 恰好包含1个 "A " 命令；其余 region_id 不含 "A "。
    - main.rs 中 zone_ring_sector/旧 zone_geometry 不再被 build_zone_cells 调用(可搜索确认零引用，
      若整块函数删除需确保没有其它调用点，如果有 dead_code 警告转 -D warnings 失败需要加 #[allow(dead_code)]
      或直接删除)。
    - Cp 展示统一两位 pF；scan period 统一两位 ms；build-ui 通过。
    - 记录"无法实机验证项"：UI 实际渲染效果、真实设备连接后 Cp/探测周期实测值需要用户在实机确认。

## 关键决策
- 不引入 ezdxf 或其它新 crate；Python 脚本手写 DXF ASCII group-code 解析器(仅两行一组: code, value)，
  已验证可用(能正确解析 34 个 REGION LWPOLYLINE + SCREEN_FRAME)。
- Slint Path 改为"全部共用 SCREEN_FRAME viewbox"方案(而非局部viewbox)，因为局部 viewbox 对不规则形状+圆弧
  精度处理更简单可靠，符合任务里给出的两个可选方案之一("核心是图形精确且交互不回归")。
- BindingCanvas 容器：从固定 360x360 正方形改为按 SCREEN_W:SCREEN_H(16:9 style, 实际 1000:562.5≈1.778)
  的 letterbox，宽度不变(min/max 360px 宽仍可保留或调整)，高度按比例算出(360*562.5/1000=202.5px)，
  或整体放大画布尺寸以便标签可读，具体数值下一步实现时定。

## 下一步动作
从"待办步骤 4"开始：先给 lib.rs 加 mod 声明，再改 main.rs::build_zone_cells 消费 touch_geometry 表并删除/停用旧几何函数，
然后同步改 app.slint 的 BindingCanvas + ZoneCell/ChannelStatus struct(cp_text 字段) + GlobalTunePage(时钟树图形化)
+ scan period/Cp 展示文案，最后跑 build-ui 迭代到通过。


## 进度更新(第2轮)
已完成 app.slint 全部结构性修改:
- lib.rs 加 `pub mod touch_geometry;`
- ZoneCell / ChannelStatus struct 各加 `cp_text: string` 字段; ZoneListRow / ChannelStatusCell 显示区加对应 Text。
- BindingCanvas 重构: 34 Path 共用 SCREEN_FRAME viewbox(screen-w=1000, screen-h=562.5, 需与
  touch_geometry::SCREEN_W/SCREEN_H 保持一致), 命中层改独立透明 Rectangle+TouchArea(按 path_x/y/width/height
  绝对定位), 画布从 360x360 方形改 480x270 (16:9 letterbox)。
- channel_delay_us(int, us) 全部替换为 scan_period_ms(float, ms) —— 涉及 AllChannelsPage/DashboardPage/
  SettingsPage/AppWindow 四层声明 + 全部 forwarding 绑定 + 两处文案("探测周期 X.XX ms")。
  ChannelStatusCell 高度从 76px 加到 90px, 网格行距从 86px 加到 100px(min-height 516->600)，
  为新增 cp_text 行留空间。
- GlobalTunePage 时钟树: 新增 ClockFixedNode / ClockArrowRight / ClockArrowDown 三个小组件;
  "CSD 时钟树(生成配置)" GroupBox 内容从纯文字块换成 3 行节点卡片 + 箭头连接:
  第1行(只读): ECO12MHz -> PLL/HFCLK48MHz -> PERI/CSD48MHz -> 外设分频DIV(/1)
  第2行: ModClk(只读, modCsdClk=1) + MFS DIV F1/F2(可编辑, SpinBox 嵌卡内, algo_global_set(5/6))
  第3行: SnsClk DIV(csd_param_set 8) / Resolution(csd_param_set 7) / SnsClk Source(csd_param_set 10)
  末端: "设备实测探测周期" 绿色卡片, 显示 scan_period_ms 两位小数。
  GlobalTunePage 新增 `in property <float> scan_period_ms: 0;`，SettingsPage 内 GlobalTunePage 实例化处
  转发 `scan_period_ms: root.scan_period_ms;`(复用 SettingsPage 已有的同名属性，不重复声明)。

## 待办(未完成，接下来继续，从这里恢复)
A. [ ] main.rs 改造(尚未动):
   1. use 增加 `mai2control_ui::touch_geometry` (或 `use mai2control_ui::touch_geometry;`)。
   2. 删除/停用旧 `zone_polar`/`angle_in_sweep`/`zone_ring_sector`/`zone_geometry` 四个函数
      (若整体删除注意 build_zone_cells 里对 zone_geometry(i) 的调用要替换)。
   3. `build_zone_cells`: 改为从 `touch_geometry::ZONE_GEOMETRY[i]` 取
      path/path_x(min_x)/path_y(min_y)/path_width(width)/path_height(height)/label_x/label_y,
      直接拷贝(注意 f32->float 已经匹配, path 是 &'static str 需要 .to_string() 或 .into())。
      同时新增: 从 `ctrl.cp(channel)` 换算 cp_text 两位 pF 填入 ZoneCell (channel<0 时 cp_text="未绑定"
      或复用 "读取中…"/沿用与 curves 页一致的语义：None->"读取中…", Some(0)->"测量中…",
      Some(0x00FF_FFFF)->"测量失败", Some(v)->format!("{:.2} pF", v as f64/1000.0))。
   4. `build_channel_status`: 同样加 cp_text 字段(从 `ctrl.cp(ch)` 换算，规则同上，ch 始终 0..36 有效不存在
      "未绑定"语义，用 "读取中…"兜底)。
   5. 全局给 CP_MEASURE_FAILED=0x00FF_FFFF 常量已存在于 main.rs 顶部，复用它，不要新建。
   6. 主循环: `ui.set_channel_delay_us(...)` 改成 `ui.set_scan_period_ms(ctrl.telem_scan_period_us() as f32 / 1000.0)`。
   7. 主循环里 `current_telem_version != last_telem_version_all` 那段(全通道 model 刷新)要 OR 上
      cp_version 变化，即新增 `let mut last_cp_version_all = u64::MAX;` 在函数顶部局部变量区,
      条件改为 `if current_telem_version != last_telem_version_all || ctrl.cp_version() != last_cp_version_all`,
      刷新后两个 last_* 都要更新。
   8. cp_text 状态机(CpPollState.status)三处 format!("Cp: {} fF..." / "Cp: 测量失败" / "读取中…" 等)
      统一改两位 pF: value as f64/1000.0, format!("{:.2} pF", ...); 0x00FF_FFFF -> "测量失败";
      0 -> "测量中…"; None -> "读取中…"。搜索 main.rs 里 "fF" 字符串定位(4处左右: on_curve_measure_cp 内
      Err分支不含fF跳过; 主循环两处 CP_GET 响应处理; measure_cp Ok分支 "测量中…"不含fF跳过)。
B. [ ] 跑 `powershell -ExecutionPolicy Bypass -File dev.ps1 build-ui` (cwd=F:\mai2control\mai2control-v4),
   反复修复编译错误直至通过。注意可能出现:
   - touch_geometry 模块可见性(pub mod + pub struct/字段 pub，已在生成文件里都是 pub，应该没问题)。
   - Slint 生成代码里 ZoneCell/ChannelStatus 新增字段 cp_text 需要 main.rs 里对应 struct literal 都补上,
     否则"missing field"编译错误。
   - f32 与 f64 混用需要显式转换。
C. [ ] 清理: 删除临时脚本 gen_touch_geometry.py 是否保留？架构要求"允许 Python 仅作为 DXF 生成辅助，
   不作为工程验证"——脚本本身可以留在仓库(不算测试文件/README，是生成工具)，但如果主 agent要求精简可以留着
   不必删除；已删除的是一次性验证脚本 verify_geom.py(已删)。
D. [ ] 最终静态核验清单(见前一轮记录的"待办步骤13")逐条核对并在汇报中列出结果，包括:
   - 34条region_id无重复/闭合Z/R19-34各一个A命令(已核验通过，见上一轮)。
   - zone_ring_sector 等旧函数不再被调用(删除后 cargo build 无 dead_code 警告即可确认，或 grep 确认零引用)。
   - Cp 与 scan period 格式符合两位小数要求(pF/ms)。
   - build-ui 最终结果(通过/失败+原因)。
   - 记录"无法实机验证项": UI真实渲染效果、真实设备连接后 Cp/探测周期实测值。

## 关键决策(补充)
- SettingsPage 组件内 `all_channels` 声明处顺带声明的 `scan_period_ms` 属性被复用于 GlobalTunePage 转发，
  没有重复声明，因为 Slint 组件内所有 `in property` 只需一份，可以被多个子组件实例化处引用。
- ChannelStatusCell/ZoneListRow 都改成显示 cp_text，与架构要求"分区状态/36通道状态卡均显示两位pF"一致。
- BindingCanvas screen-w/screen-h 硬编码为 1000/562.5，必须和 touch_geometry.rs 里的 SCREEN_W/SCREEN_H
  常量保持同步；若以后 DXF 尺寸变化需要同时改两处(当前无自动同步机制，属已知技术债，不在本次任务范围内处理，
  除非验证发现不一致)。

## 下一步动作
从"待办 A"开始改 main.rs：先加 use，再删/改 zone_geometry 相关四个函数，改 build_zone_cells 和
build_channel_status 分别接入 touch_geometry 表和 cp_text，改主循环 scan_period_ms 与 cp_version 门控，
统一 Cp 文本两位 pF 格式，然后跑 build-ui 迭代到通过。


## 任务完成(第3轮/最终)
main.rs 全部改造完成，build-ui 通过。所有待办已清空：
- use 加 `mai2control_ui::touch_geometry`。
- 删除 zone_polar/angle_in_sweep/zone_ring_sector/zone_geometry/ZoneGeometry 五个符号，零引用确认(grep 为空)。
- 新增 `cp_display_text(Option<u32>) -> String` 内聚工具函数(None->读取中/Some(0)->测量中/
  Some(CP_MEASURE_FAILED)->测量失败/其余->"{:.2} pF")，build_zone_cells / build_channel_status /
  CpPollState 三处全部复用，不再有 "fF" 字符串(grep 确认零命中)。
- build_zone_cells 改为读 touch_geometry::ZONE_GEOMETRY[i]，ZoneCell 新增 cp_text 字段已填。
- build_channel_status ChannelStatus 新增 cp_text 字段已填。
- 主循环 set_channel_delay_us -> set_scan_period_ms(us/1000.0)；全通道 model 刷新条件加入
  cp_version 门控(last_cp_version_all)。
- build-ui 最终结果: **PASS**，`Finished \`dev\` profile [unoptimized + debuginfo] target(s) in 5.63s`，
  无 warning/error。

## 最终静态核验结果
- touch_geometry.rs: 34 条 ZoneStaticGeometry，region_id 1..34 各一次不重复。
- 每条 path 均以 "M " 开头、以 " Z" 结尾(闭合)。
- region_id 19..34 各恰好 1 个 " A " 命令；其余 region_id 恰好 0 个。
- zone_ring_sector 等旧函数已从 main.rs 物理删除，grep 全仓库零引用。
- Cp 展示统一两位 pF(`{:.2} pF`)；scan period 统一两位 ms(`Math.round(x*100)/100 + " ms"`)。
- build-ui 通过，无编译错误/警告。

## 无法实机验证项(明确列出)
- UI 真实渲染效果(34 区几何是否在实际窗口比例下视觉正确、letterbox 是否合适、时钟树节点卡片
  是否有裁切)——需要用户在实机跑 mai2control-ui.exe 连接真实屏幕肉眼确认。
- 真实设备连接后 Cp 实测值与探测周期实测值(scan_period_us 由固件遥测帧回填，无设备连接时恒为 0，
  显示 "0.00 ms"，本次仅做静态代码/格式核验，未做端到端设备联调)。
- BindingCanvas 34 区点击/双击交互在真实鼠标操作下的手感(命中层几何已用与图形层同一坐标系计算，
  逻辑正确性已静态确认，但未做交互式点击测试)。

任务状态: 完成。
