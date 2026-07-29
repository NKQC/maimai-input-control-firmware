# firmware-refactor 上下文快照

## 工程与工具链
- 工程根: `f:\mai2control\mai2control-v4`
- 上位机: `control_software`(Rust + Slint 1.17), 固件: `main_firmware`(RP2040/PlatformIO), `psoc_firmware/CY8C4147AZI-SensorCore`(PSoC CSD)
- PSoC 编译: `& 'C:\Users\asdfg\ModusToolbox\tools_3.6\modus-shell\bin\bash.exe' -lc "cd /cygdrive/f/mai2control/mai2control-v4/psoc_firmware/CY8C4147AZI-SensorCore && make build -j8 2>&1 | tail -8"`
- 一键流水线: `main_firmware\build.ps1`, 顶部 `$WorkflowMode = "Build" | "Probe" | "Flash"`(当前已复位为 `Build`)
  - Build: 编 PSoC → `psoc_hex_to_c.py` 嵌入 `src/protocol/psoc/psoc_fw_image.h` → 编 RP2040 UF2 → `cargo build --locked --bins`
  - Flash: 上述 + 自动请求 BOOTSEL + 拷 uf2 + 等拓扑稳定 + WinUSB/CDC 烟雾测试。PSoC 版本号变化时 RP2040 会经 SWD 自动重刷 PSoC
- RP2040 单独编译: `cwd=main_firmware`, `pio run`(execute_pwsh 带 cwd 参数, 勿用 cd)
- 手动烧 RP2040: `selftest.exe --reboot-bootloader-only` → 等 `G:\INFO_UF2.TXT` → 拷 `main_firmware\.pio\build\pico\firmware.uf2` 到 `G:\`
- selftest: `control_software\target\debug\selftest.exe --diagnose / --csd-reset-test / --auto-tune-test / --calib-track --div N / --soak-csd N --seed N / --reboot-test / --gain-inactive-test / --res-all-test / --semi-calib-probe / --reset-config`
- Release UI: `cargo build --release --bin mai2control-ui` → `control_software\target\release\mai2control-ui.exe`

## Shell 约束(重要)
- `execute_pwsh` 会吞掉 `$` 变量($_/$env:/自定义变量全失效) → 只写无变量的单行命令; 需要变量的逻辑改用 `Get-ChildItem|Select-String|Where-Object <Prop> -eq` 形式
- 本工程 `grep_search` 假阴性 → 一律用 `Select-String` / `Get-ChildItem -Recurse | Select-String`
- `cargo build` 若 UI 正在运行会因 exe 被占用失败(仅替换失败, 非编译错误) → 用 `cargo check` 确认编译, 或先关 UI
- app.slint 改动用唯一上下文锚点, 勿用 replace_all(曾致重复声明)

## 本次会话已完成(全部已编译, 固件已烧录验证)
1. **algo_get_trace NAK 刷屏修复**: `app_state/mod.rs` 加 `algo_trace_last_seq`/`algo_trace_backoff`; NAK 匹配 seq 设退避 180 tick(~3s), 成功响应清零; `main.rs` 轮询加页面门控(`current_view==1 && settings_tab==3`)
2. **虚拟摄像头接入**: `vcam/mod.rs`(VcamState/QR 640x480 居中75%) + `vcam/keyboard.rs`(WH_KEYBOARD_LL, 停顿/Enter 提交, 每串一次); ToolboxPage tb_tab==1 真实 UI(启用/提交阈值秒/显示秒/last_data/QR 预览 Image); main.rs 建 VcamState + `on_set_vcam_*` + timer `tick()`+帧变化推 `slint::Image`
   - **未做**: MF 系统级后端(MFCreateVirtualCamera 需独立 cdylib COM 媒体源 + regsvr32 HKLM 注册 + 共享内存 IPC, ~2000 行 unsafe, 高风险) — 已向用户报告方案, 等确认
3. **日志系统重构**: `LogLevel{Error0,Warn1,Info2,Debug3}` + `event_log: VecDeque<(LogLevel,String)>` + `log_filter`(默认 Info) + `push_log/_debug/_warn/_error`; `log_text()` 按等级过滤、带 `[E]/[W]/[I]/[D]` 前缀、最近 60 条、缓冲 400
   - 日志移入**第 4 功能区**`LogPage`(current_view==3, 导航"日志"), 移除主页内嵌日志框
   - LogPage 加**过滤等级 ComboBox**(错误/警告/信息/调试), 回调 `set_log_filter` 已透传到 AppWindow + main.rs
   - `handle_frame` 对每个收到的帧(排除高频 TelemData)打 DEBUG: `← 收到响应/NAK/帧 cmd=0x.. seq=.. len=..`
   - **CSD 诊断改 DEBUG 级**并保留: `_csd_diag_fire` + 三个响应处理器(global_get_all/param_get/cp_get)的回读日志全走 `push_log_debug`; 探针在冷却落定后触发(save 175 tick / 立即改 145 tick)
4. **CSD 串行化锁**: `csd_cooldown_ticks`/`csd_cooldown_label`; `op_busy()` 含冷却; `_reject_csd_if_locked` 守卫 `calibrate/baseline_reset/auto_tune/global_set/save_config`(忙时拒绝+日志"设备忙…已忽略", global_set 被拒时值仍存草稿); 冷却 tick: 校准120/基线90/自适应120/全局120/保存后150; `_begin_op/_end_op` 打 `▶/✔` 日志; 卡死检测每秒心跳 + 10s(625 tick)超时告警并自动解锁
5. **数据存活检测**: `telem_last_raw[36]`/`telem_freeze_count[36]`, `_handle_telem_data` 逐帧比 raw(相同+1/变化清零), `FREEZE_FRAMES=200`; `channel_frozen()`/`channel_freeze_frames()`/`data_all_frozen()`; ChannelStatus 加 `frozen` 字段 → 通道卡显示橙色"⚠停滞"; `sensor_health_summary/level` 加"✕ 数据停滞"(全通道冻结, 错误级)
6. **UI 渲染修复**: 
   - 触发判定窄带"只画一小块" = **Slint 1.17 Path 默认 contain(保持宽高比)**, 方形 1000x1000 viewbox 塞进宽扁窄带被居中 → 令 `viewbox-width = 1000*(w/h)` + 新增 `trigger_area_resized(float)` 回调上报宽高比, main.rs 用 `trigger_aspect` Cell 把 band 的 `active_path` x 拉到 `[0,1000*aspect]`(算法页窄带仍用方形 x_scale=1.0)
   - 阈值线缩放错层 = 旧实现用 plot 像素线性映射 ff, 与曲线 viewbox 变换不一致 → 改为画进 clipbox 的 Path, `commands: "M 0 \{finger_th_y} L 1000 \{finger_th_y}"`, 用与曲线**完全相同**的 viewbox 变换(移除旧 ff/nf Rectangle, 保留数值标注)
7. **单次提交(消 USB 掉线)**: 新增 `GLOBAL_COMMIT = 0x2D`(host_cmd.h + Rust `HostCmd::GlobalCommit` + `encode_global_commit`); RP2040 `_handle_global_set` **只写影子不再逐项 commit**, 新增 `_handle_global_commit` 单次触发重初始化; 上位机 save_config 发完全部 GLOBAL_SET 后**只发一次** GLOBAL_COMMIT, 即时 global_set 走 set+单次 commit
8. **恢复默认值(0 值修复)**: `reset_defaults()` 排程 `post_reset_refetch_in=160`(~2.6s) → `take_post_reset_refetch()` → main.rs 全量重读(request_config_all/request_params(ch,0)/global_get_all/measure_cp/request_cp) 刷新显示
9. **PSoC 0.4.19(已 SWD 烧录)**: `global_apply_pending` 配置更新路径**去掉 `Cy_CapSense_Enable` 自动校准**, 恒走轻量 `Init + Initialize + InitializeAllBaselines`(沿用现有 IDAC) → **配置更新不再"转一圈"自动跑校准/频率下探; 校准与频率自适应仅显式 CALIBRATE/AUTO_TUNE 手动触发**。`g_auto_calibrate` 仍被逐通道 APPLY 路径使用(非死代码)
10. **恢复默认→良好半自动基线(已烧录 RP2040)**: `CsdConfig` 加 `_recapture_pending` + `request_recapture()/has_pending_recapture()/clear_recapture()`; `_handle_reset_defaults` 里 `clear()` + `request_recapture()` + 重启 PSoC; `main.cpp` provisioning 成功后若 pending 则 `capture_from_psoc` → `note_mode(CSD_MODE_SEMI)` → `download_to_psoc` → `request_save()` → 清标志(使"正常半自动基线"成为默认, RP2040 持有, PSoC 无状态)

## 修改过的文件
- `control_software/src/app_state/mod.rs`, `control_software/src/main.rs`, `control_software/ui/app.slint`
- `control_software/src/vcam/mod.rs`, `control_software/src/vcam/keyboard.rs`, `control_software/src/lib.rs`, `Cargo.toml`(qrcode 0.14 + windows features)
- `control_software/src/proto/mod.rs`(GlobalCommit=0x2D), `control_software/src/proto/algo.rs`(encode_global_commit)
- `main_firmware/src/protocol/host_cmd/host_cmd.h`(GLOBAL_COMMIT=0x2D), `host_cmd.cpp`(_handle_reset_defaults)
- `main_firmware/src/service/sensor_link/sensor_link.h/.cpp`(_handle_global_commit + 注册, _handle_global_set 去 commit)
- `main_firmware/src/service/csd_config/csd_config.h`(_recapture_pending + 三个方法)
- `main_firmware/src/main.cpp`(provisioning 后 recapture→SEMI)
- `psoc_firmware/CY8C4147AZI-SensorCore/main.c`(FW_VERSION_PATCH 18→19, global_apply 去 Enable)

## 当前烧录状态
- RP2040: 已烧最新(含 GLOBAL_COMMIT + recapture→SEMI), `RP build=0x4D324401`
- PSoC: 已经 SWD 刷到 **0.4.19**(`embedded PSoC=0x00000413`), SWD program/verify 成功, Clock config_ok=true
- `selftest --diagnose` **DIAGNOSE PASS**, link_valid/snapshot_valid=true
- Release UI 已编译: `control_software\target\release\mai2control-ui.exe`

## ★当前阻塞的核心问题: PSoC CapSense 扫描引擎卡死★
- 症状: PSoC 白灯常亮(g_any_active 误判); raw **完全冻结**(如 CH0=3514 恒定, diff=0), 任何模式(AUTO/SEMI)都一样; `scan_period_us=142857`(~7Hz, 正常应 ~163Hz/6.13ms)
- 决定性证据: 采样 PSoC `generation` 计数器 **3s 内推进 ~3100(很快)** → PSoC 主循环在快速运行且持续发布快照, 但 **raw 值从不变化** → CapSense 转换/扫描本身卡死, 不是 SPI/链路/上位机问题
- 已排除: XRES 软复位(reboot_psoc)、RESET_DEFAULTS(clear+重启+出厂强制好全局 增益4/目标85 自动校准)、CALIBRATE(all)、BASELINE_RESET(all)、AUTO/SEMI 模式切换 — **全都救不回**
- 校准本身"看起来成功": raw≈3471~3514(≈85% 目标, 未 railed), 但数值不动
- auto_tune 必然失败: 逐档 8→64 分频 `CalibrateAllWidgets` 全失败 → result=2 / RP2040 10s 超时 NAK(`PSoC auto_tune failed/timeout`)
- 结论: 属 PSoC 侧(模拟前端锁定态 / PSoC flash 异常 / 硬件)问题, RP2040 侧配置改动无法修复

## 下一步(等用户回答)
1. **已请用户彻底断电测试**: 拔 USB 断电 ≥10s 再插回(XRES 只复位内核, 清不掉 CSD 模拟前端锁定态), 观察 raw 是否恢复抖动、触摸是否响应、采样率是否回 ~163Hz
   - 若恢复 → 锁定态已清; 本次"恢复默认→SEMI 良好基线"会维持良好基线
   - 若仍冻结 → 需**干净重刷 PSoC**(SWD 全片擦除 + 重烧 0.4.19), 或硬件损伤需单独查
2. 用户要求的"自持异常检测 + 全面过测各种设置 + 自动恢复正常 SEMI 基线为默认" — **仅在扫描引擎能正常工作后才有意义**, 待断电测试结论再做
3. 排队未做: MF 虚拟摄像头系统后端(待用户确认方案); selftest 里改全局的用例(如 `--gain-inactive-test`)需补发 `GLOBAL_COMMIT` 才生效(新固件不再自动 commit) — 已问用户是否要改
4. 提醒: UI 改动需重跑 `target\release\mai2control-ui.exe` 生效
