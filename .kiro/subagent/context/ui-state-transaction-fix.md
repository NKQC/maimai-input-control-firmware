# ui-state-transaction-fix 任务状态

## 目标（简述，详见主 agent 原始任务）
上位机(control_software) 配置型编辑改为"草稿(draft)覆盖设备快照(snapshot)"架构，只有点击"保存到设备"才批量下发+SAVE_CONFIG，ACK 全部通过才清草稿并重新 GET 验证；NAK/失败保留草稿。
分区绑定改为"侦听下一次触摸"(上位机用已开启的全通道遥测 status rising edge 捕获，不发 BIND_START)。
CSD 调整页加期望/实际周期对比、撤销/恢复安全默认(连接快照,不猜值)。
mode.work 用 ComboBox。绑定页画布响应式 16:9 + 单 TouchArea 精确命中(DXF 弧离散点 + point-in-polygon)。时钟树箭头改真图形。

## 架构结论(已读代码确认)
- HostCmd 全集见 control_software/src/proto/mod.rs；BindSetMap=0x44 payload=[zone,channel]->ACK，固件 binding_service.cpp 已实现 reload_binding()。BIND_START(0x40) 会立即让固件进 WAIT_TOUCH 并在下次触摸时真实改绑定+发 BIND_EVENT —— 新交互不得使用它作为主路径。
- CFG_SET_BATCH(0x14) payload=count(u16)+Entry×count，全成功才 ACK，否则 NAK 首个失败 key（host_cmd.cpp _handle_cfg_set_batch）。
- PARAM_SET/MODE_SET/GLOBAL_SET/KBD_SET_MAP/KBD_SET_TOUCHMAP 均单命令 ACK/NAK，无批量版本，需逐条发（sensor_link.cpp / keyboard.cpp）。
- GLOBAL_SET 收到后固件立即 psoc->global_commit() 重初始化——发多个 global 改动应逐条 ACK 后再发下一条，不并发。
- 无 MODE_GET/无法从设备回读 scan_mode；现有 UI scan_mode 本就是纯本地 in-out 属性(从未真正回读)。沿用此假设，只做本地"最近一次成功提交"的 snapshot 记忆（不算回归）。
- RESET_DEFAULTS(0x0F) 只重置 ConfigManager(普通 cfg key)，不含 CSD/globals/params。
- proto::algo 已有 encode_global_get/set/get_all, algo 相关命令。proto::mod 无 encode helper for BindSetMap/KbdSetMap/KbdSetTouchmap/ModeSet/ParamSet 等，均在 app_state 内直接 Frame::new(HostCmd::X as u8,...) 构造，风格延续。
- touch_geometry.rs 由 gen_touch_geometry.py 从 DXF 生成，含 34 区 SVG path(M/L/A/Z)+bbox+label。当前 UI 命中层用 34 个按 bbox 定位的透明 Rectangle+TouchArea（app.slint BindingCanvas），这就是"bbox 重叠误点"根因（扇形/环形区域 bbox 严重重叠）。需要：gen 脚本追加离散 hit_points(每弧>=24段)，Rust 提供 touch_geometry::hit_test(x,y)->Option<usize> (bbox 预筛+point-in-polygon)，Slint 侧只留一个全画布 TouchArea，clicked 时把本地像素坐标换算到 SCREEN_W/H 空间调用 Slint callback(返回 int)做命中。
- main.rs 当前"离开设置页自动保存"逻辑在 setup_ui_callbacks 的 timer 闭包里(`if last_view == 1 && cur_view != 1 && ctrl.is_config_dirty() { ctrl.save_config(); }`)，必须删除。
- AppController 现状：所有 set_* 方法立即发帧+乐观本地写 cache+mark_config_dirty(仅置 bool，不落 key，导致 dirty_count 永远与"N 项"文案不符——这是 dirty_count=0 的根因之一，配合"未记录 key 的 set_param/global_set/set_mode/kbd_set_*"）。

## 已完成
- 完整阅读 app_state/mod.rs, main.rs, app.slint, touch_geometry.rs, gen_touch_geometry.py, proto/mod.rs, proto/config.rs, proto/algo.rs, io/mod.rs, host_cmd.cpp/.h, sensor_link.cpp, binding_service.cpp。
- 确认协议边界与固件语义（上面"架构结论"）。

## 待办（按顺序执行，每完成一大项回写本文件）
1. [ ] touch_geometry.rs + gen_touch_geometry.py：加 hit_points 离散弧点 + hit_test(x,y)。
2. [ ] app_state/mod.rs 大改：
   - draft/snapshot 分离：config_draft, csd_mode_draft/snapshot, globals_draft, param_draft((ch,pid)->v), param_all_draft(pid->v), binding_draft(zone->ch u8 0xFF=unbound), kbd_map_draft(idx->(code,mod)), kbd_touchmap_draft(zone->(code,mod)), reset_defaults_staged。
   - dirty key 统一 BTreeSet<String>：cfg:key / mode / global:id / param:ch:id / param:all:id / bind:zone / kbd:phys:idx / kbd:zone:zone / reset_defaults。
   - effective_* getter 系列（draft 覆盖 snapshot）。
   - stage_* 公开方法只改草稿+dirty，不发帧。
   - 旧 set_config/set_param/set_param_all/set_mode/global_set/set_binding_channel/kbd_set_map/kbd_set_touchmap 的发帧逻辑拆成私有 _apply_*，仅供 commit 用；对外不再暴露立即生效版本（main.rs 里的调用点要跟着换成 stage_*）。
   - CommitQueue：VecDeque<CommitStep>，tick 驱动单 outstanding，ACK 前进，NAK 即停(保留 draft/dirty)，全部完成含 SAVE_CONFIG 后清 draft+dirty 并重新 GET(config_all/global_get_all/kbd_request_map/touchmap/request_params(0))。
   - save_config() 变成"构建并启动 commit 队列"（reset_defaults 排最前，其余按 cfg batch/mode/global/param/bind/kbd 顺序，末尾恒发 SAVE_CONFIG）；需要 commit_tick()（供 main.rs 每帧调用，驱动发下一条）。
   - listening_zone: Option<u8> + baseline 触摸掩码；listen_next_touch_start/stop；在 poll() 或新 tick 里检测遥测 rising edge -> stage_binding_channel + 停止监听。
   - CSD 安全默认：connect 成功后一次性捕获 csd_connect_snapshot{mode, globals(6项), csd_params(pid 7/8/10 的 ch0 值)}；csd_restore_safe_defaults()（无snapshot则报错文案，不猜值）；csd_discard_draft()（清 mode/global/param 相关 draft，不清 binding/kbd/cfg）。
   - 移除/废弃 bind_start/bind_abort 作为 UI 主路径（保留函数不删，仅不再被新 UI 调用）。
3. [ ] main.rs：
   - 回调改调 stage_* / listen_* / csd_discard_draft / csd_restore_safe_defaults / commit(save_config)。
   - 删除"离开设置页自动保存"整段逻辑。
   - 每 tick 调 ctrl.commit_tick()（驱动 commit 队列）。
   - build_config_rows: mode.work 特判 kind=2 enum_index=value（沿用 led.color_* 特判写法）。
   - 新增字符串属性回填：scan_period_text/expected_period_text/period_delta_text（Rust format!("{:.2} ms")，0 显示"等待设备遥测"）。
   - 新增 zone_hit_test 回调(Rust侧 touch_geometry::hit_test)。
   - ZoneCell 加 listening 字段（来自 ctrl.listening_zone()）。
   - config_dirty_count/is_config_dirty 改读新草稿 dirty 集合；commit 状态字符串回填 UI。
4. [ ] app.slint：
   - BindingCanvas: 去掉 bbox 命中层，改单 TouchArea + zone_hit_test 回调；容器改响应式 16:9 letterbox(占主要空间，非固定480x270)。
   - ZoneListRow: 三操作(通道 ComboBox/SpinBox+"暂存"、"移除"、"侦听下一次触摸/取消") + 等待态高亮。
   - ConfigPage/底栏文案改"仅保存草稿，点击保存后才下发生效"，去掉"已实时下发生效/离开保存"字样。
   - ClockArrowRight/Down 改 Rectangle 线段 + Path 三角箭头。
   - GlobalTunePage(CSD调整页)加：期望周期输入、scan_period_text/expected_period_text/period_delta_text 展示、撤销CSD改动按钮、恢复安全默认按钮、重启PSoC按钮入口。
5. [ ] `powershell -ExecutionPolicy Bypass -File dev.ps1 build-ui` 跑通，修错到过。
6. [ ] 完成自检清单逐项核对，整理最终汇报。

## 关键决策记录
- Cp 中央限速轮询逻辑（main.rs 已有 CpPollState round-robin 骨架，连接后 measure_cp 一次 + 500ms 后每 500ms 轮询当前可见通道）——现状已基本满足"连接后触发一次+限速"的要求，本任务重点是状态机/绑定/CSD/布局，不重做 Cp 调度，只需确认不冲突。
- BIND_EVENT/bind_progress 相关旧字段保留但新 UI 不再驱动，避免破坏编译单元测试。
- seq 回绕风险：commit 队列单 outstanding 设计下用精确 seq 匹配 ACK/NAK，与现有 bind_start_seq 模式一致，不做全局 seq 状态机重构（超出本任务范围，已知限制，会在最终汇报注明）。

## 下一步动作
从 touch_geometry.rs / gen_touch_geometry.py 开始动手。
