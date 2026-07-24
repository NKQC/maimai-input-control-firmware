# PSoC 固件修复阶段 (Phase A) — 任务契约与进度

## 背景架构(主 agent 已勘查, 事实)
- PSoC `psoc_firmware/CY8C4147AZI-SensorCore/main.c`:
  - 白 LED: P1.6, 宏 `STATUS_LED_PORT/NUM`, `STATUS_LED_ON_STATE=1/OFF_STATE=0`。当前启动点亮 300ms 后熄灭(~L805-810), 运行时不再指示。
  - `update_touch_frame()`(~L338-366): 逐通道 `Cy_CapSense_IsWidgetActive`, algo_valid 时走 `algo_engine_run_channel`; 结果置 `mask[ch>>3]` bit, 写 `touch_frame`(7B TOUCH)。out_active 只进这个 mask。
  - 主循环 `for(;;)`(~L811-950): 扫描完成→Process→`update_touch_frame()`→`publish_capsense_snapshot()`→CP→algo commit→global_apply_pending(Init+Enable)→apply_pending(SEMI: Initialize+InitializeAllBaselines / AUTO: Enable)→`scan_count++`→`ScanAllWidgets`。
  - 命令 switch(~L640-760): 已有 SET_PARAM/GET_PARAM/SET_MODE/APPLY(0x33)/GET_RAW/GET_STATS/SET_GLOBAL/GET_GLOBAL/GLOBAL_COMMIT(0x3A)/MEASURE_CP/GET_CP。**无独立 CALIBRATE/BASELINE_RESET**。
  - `apply_pending`/`global_apply_pending`/`measure_cp_pending` 是 volatile bool, ISR 置位、主循环执行。
- RP2040 `main_firmware/src/service/sensor_link/sensor_link.cpp`:
  - `_handle_calibrate`(~L323) → `Psoc::apply_params()`(发 SENSOR_CMD_APPLY)。register: `HostCmd::CALIBRATE→_handle_calibrate`, `HostCmd::BASELINE_RESET→_handle_unsupported`(空操作!)。
  - `Psoc` 类在 `main_firmware/src/protocol/psoc/psoc.{h,cpp}`, 有 `apply_params()`, SPI 命令经命令环投递(core1 独占 SPI)。
- 根因: 校准在 SEMI 模式只 re-init 不校准 IDAC; 基线复位 RP2040 空操作 → 二者"无效"。

## Phase A 任务(仅固件, 必须编译通过)
1. **白 LED 统一**(需求#8+#12): 启动点亮; 运行时"任一通道激活(algo out_active/base_active 聚合)则亮, 否则灭"。
   - 在 `update_touch_frame()` 算完 mask 后, 置全局 `static volatile bool g_any_active = (mask[0]|mask[1]|mask[2]|mask[3]|mask[4])!=0u;`
   - 主循环: 启动阶段(进入 for 前)LED 常亮; 每次 update_touch_frame 后 `Cy_GPIO_Write(STATUS_LED_PORT,STATUS_LED_NUM, g_any_active?ON:OFF)`。移除固定 300ms 后强制熄灭的逻辑(改为: 启动亮→首帧扫描后交给 g_any_active)。
   - 这样新人写的算法只要令某通道 out_active=1, 白灯即亮, 直观可视化(即"算法点亮白LED示例", 无需改 algo blob, 引擎聚合驱动)。
2. **真正的 CALIBRATE / BASELINE_RESET**(需求#9):
   - PSoC 新增命令 `SENSOR_CMD_CALIBRATE (0x3B)`, `SENSOR_CMD_BASELINE_RESET (0x3C)`; ISR 置 `calibrate_pending/baseline_reset_pending`; 主循环执行:
     - calibrate: 若 `CY_CAPSENSE_ENABLE==CY_CAPSENSE_CSD_CALIBRATION_EN` 则 `Cy_CapSense_CalibrateAllWidgets(&cy_capsense_context)` 再 `Cy_CapSense_InitializeAllBaselines`; 否则回退 `Cy_CapSense_Enable`。
     - baseline_reset: `Cy_CapSense_InitializeAllBaselines(&cy_capsense_context)`。
   - RP2040: `Psoc` 加 `calibrate()`/`baseline_reset()` 发对应新命令; `_handle_calibrate` 改为发 CALIBRATE; 新增 `_handle_baseline_reset` 并把 `HostCmd::BASELINE_RESET` 注册到它(取代 _handle_unsupported)。保留 APPLY 供参数应用。
3. **重启排查**(需求#10, 允许只诊断+加固): 阅读 `reboot_psoc`(XRES 脉冲)实现, 确认真的脉冲 XRES 并重跑 bring-up; 确认 apply_pending 不会死锁扫描循环(如需, 加简单超时/幂等)。若发现"改参卡固定值"确切根因则修, 否则在本文件记录结论与建议, 不盲改。

## 构建命令(务必执行验证)
- PSoC(dev.ps1 build-psoc 有 exit 误报, 用直接命令):
  `& 'C:\Users\asdfg\ModusToolbox\tools_3.6\modus-shell\bin\bash.exe' -lc "cd /cygdrive/f/mai2control/mai2control-v4/psoc_firmware/CY8C4147AZI-SensorCore && make build -j8"`
  成功后: `powershell -ExecutionPolicy Bypass -File dev.ps1 embed-psoc`
- RP2040: `powershell -ExecutionPolicy Bypass -File dev.ps1 build-rp`
- 不要 reflash(主 agent 负责刷写+真机验证)。

## 进度(每约5步更新)
- [ ] 未开始

## 进度更新 (主 agent 直接落地, module-builder 被中止后接手)
- [x] 白LED: main.c 加 g_any_active(update_touch_frame 聚合 mask), 启动常亮→循环按 g_any_active 驱动(移除固定300ms熄灭)。
- [x] CALIBRATE(0x3B)/BASELINE_RESET(0x3C): PSoC 命令+pending+主循环执行(CalibrateAllWidgets+InitializeAllBaselines / InitializeAllBaselines)。
- [x] RP2040: psoc_types Cmd 加 0x3B/0x3C; psoc_spi calibrate()/baseline_reset()(mirror apply, sleep 120/20ms); psoc SpiOp+_exec_cmd+方法; sensor_link _handle_calibrate 改发 calibrate, 新增 _handle_baseline_reset 并注册(取代 _handle_unsupported)。
- [x] 构建: PSoC make build OK(.text 30984), embed OK(v0x412 flash33920), build-rp SUCCESS(flash 15.8%)。
- [ ] #10 reboot_psoc/改参卡值: 尚未排查(待真机复现)。
- [ ] 真机验证 calibrate 把 railed raw 拉回、baseline_reset 生效、LED 启动亮/触发亮。
