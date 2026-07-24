# capsense-cp-tuning-completion

## 当前状态
- 已检查初始 git status/diff；工作树存在大量既有未提交改动和构建产物，本任务仅在目标文件上增量编辑。
- 已读取此前 `.kiro/subagent/context/capsense-cp-tuning-ui.md`，确认 UI 已有 36 通道 Cp 缓存、500ms/10s 轮询骨架，但连接边沿和 ParamRow 编辑值仍需修正。
- 已核对 `dev.ps1`：diagnose=只读 DEVICE_INFO，smoke=严格设备状态校验，full=完整无头协议流程；尚未执行硬件动作。
- 已核对版本漂移：PSoC `main.c` 原为 0.4.2，内嵌头为 0.4.8；已选择并修改源版本为 0.4.9，生成头尚未重建。
- 已修改 PSoC `main.c` 的 Cp 状态：新增 pending/active 双标志；启动 36 通道初始化为 0xFFFFFF；pending/active 时 CP_GET 返回 0；完成后返回成功 fF 或 0xFFFFFF；主循环与 SPI ISR 状态切换用临界区，测量期间不暴露逐通道半更新。
- 已开始修正 `main.c` SmartSense 旧文案为“自动校准/标准完整处理”和“半自动手动”。

## 下一步
1. 完成所有指定第一方源码旧文案修正。
2. 让 `Psoc::measure_cp()` 等待 core1 实际 SPI ACK，再由 Host 回复 ACK。
3. 修正 Rust 连接/断线/切通道 CpPollState 与 params+cp 主动请求，ParamRow 按钮使用 SpinBox 当前值。
4. 修正 `dev.ps1` 非零退出码传播、embed 版本解析和 `main_firmware/build.ps1` cargo `--locked --bins`。
5. 依次运行固定 build-ui、build-rp、完整 build.ps1，修复至通过；再运行安全无头 action，最后核对 diff/协议常量。

## 进度更新 2
- `Psoc::measure_cp()` 现在以结果指针提交命令，core0 等待 core1 实际执行 `_spi.measure_cp()` 并校验 PSoC SPI ACK 后才返回；SensorLink Host ACK 因此不再代表仅入队。
- 已更新 psoc.h/psoc_spi.h/psoc_types.h/sensor_link.cpp/app_state/selftest/csd_config 的相关命名与文案，统一为“自动校准/标准完整处理”与“半自动手动”。
- AppController 新连接会停止旧 handle、清空 36 通道传感器缓存；main.rs 在连接成功边沿主动请求当前通道 params+Cp，断线边沿重置 CpPollState，切通道清旧显示并同时请求 params+Cp；0 Cp 显示为测量中，轮询仍为 500ms/最多10s。
- Slint 新增 ParamEditorRow，本地保存 SpinBox 当前编辑值，应用全部通道直接发送该值，消除模型回填滞后。
- dev.ps1 新增 Invoke-NativeTail，build-rp/build-ui/build-ui-release/build-psoc/embed 可靠捕获并传播 native 非零退出码；diagnose/smoke/full 也传播退出码。embed 从 main.c 解析版本并复用 psoc_hex_to_c.py，不再默认硬编码版本。
- main_firmware/build.ps1 Rust 步骤已改为 cargo build --locked --bins。

## 进度更新 3
- 审查现有 14 文件 diff：PSoC pending/active 状态、36 通道 0xFFFFFF 初始化、RP core0 等待 core1 实际 SPI ACK、Rust 连接边沿 params+Cp 请求均已落地。
- 核对 `design.cycapsense`：`CSD_AUTOTUNE=NONE`；`BIST_EN=true`、`BIST_SNS_CAP_EN=true`，未修改 GeneratedSource/vendor。
- 发现待修：ParamEditorRow 自持 edited_value 在模型刷新/切通道后可能保持旧值，应直接读取 SpinBox 当前 value；selftest/main.rs/csd_config.h 少量“全自动/自整定”旧文案需统一。
- 已读取 `dev.ps1`：diagnose/smoke/full 均为不刷写动作；后续构建通过后将按设备状态执行 diagnose，再决定 smoke/full。

## 进度更新 4
- 修正 ParamEditorRow：按钮直接读取具名 SpinBox 的当前 `value`，不再维护可能随通道/模型刷新失同步的局部值。
- 清理 csd_config.h、main.rs、selftest、main.c 残余旧模式表述；GeneratedSource 核对为 36 widgets/sensors、全部 SmartSense 宏 0、BIST/SNS_CAP 1。
- 固定入口验证：`dev.ps1 build-ui` PASS；`dev.ps1 build-rp` PASS；`main_firmware/build.ps1` Build 模式 PASS。完整流程从 main.c 解析并嵌入版本 `0x00000409`，生成 32000 字节 PSoC 镜像头。
- `dev.ps1 diagnose`（只读、不刷写）PASS；连接设备报告其当前 RP 内嵌 PSoC 仍为 `0x00000408`，与本次构建 0.4.9 不匹配，因此不继续 smoke/full，且未切换 Flash/未刷写。

## 进度更新 5
- 最终补清 host_cmd.h、selftest provision 输出、Slint 模式注释中的旧“半自动/手动/自整定”文案；再次 `dev.ps1 build-ui` PASS。
- 已恢复仅由验证构建改动的受跟踪 `.pio` 产物并删除本轮临时 workflow 日志；保留必须生成的 `psoc_fw_image.h` 0.4.9 变更。
- 最终待办仅为一次 git diff/check、Host 命令号/5 字节响应、参数 0x01..0x0B、36 通道、500ms/10s 的结果汇总。

## 完成
- 将 PSoC 启动 Cp 初始化从字节 memset 收紧为显式循环给 36 通道赋 `0xFFFFFFu`；再次完整固定入口 Build PASS，并重新生成 0.4.9 镜像头。
- 最终完整构建仍只有 GeneratedSource 既有 Sense clock（Button3..35 超过 6MHz）及链接 RWX warning，无编译错误。
- 最终设备验证结论不变：只读 diagnose PASS，但设备运行旧内嵌 PSoC 0.4.8；未刷写，故本次 0.4.9 无硬件 smoke/full 结果。

## 硬件闭环恢复更新 1
- 已恢复上下文并复核工作树：0.4.9 UF2 与 selftest 均存在，设备当前为 app 运行态，未处于 BOOTSEL。
- 发现固定 `dev.ps1 cycle` 明显缺陷：硬编码 G:、没有 RPI-RP2 卷标/唯一卷安全检查、未校验 UF2、复制/诊断退出码不传播、固定等待 9s 而非等待设备重新枚举。
- 发现 strict smoke 仍硬编码 RP/embedded PSoC 0x00000401，与目标 0x00000409 不符；full 尚无 CP_MEASURE/CP_GET 验收。
- 下一步：最小修正 dev.ps1 安全 cycle 与 smoke 身份，向既有 selftest full 加 CP 500ms/10s 轮询；build-ui/build-rp 后仅通过固定 cycle 刷写。


## CP BIST limit 语义修复（续办）
- 已依据 `mtb_shared/capsense/release-v5.0.0/cy_capsense_selftest_v2.c` 复核：`Cy_CapSense_BistMeasureCapacitanceSensor()` 对所有非 `TIMEOUT` 状态写入 `*cpPtr`；`LOW_LIMIT` / `HIGH_LIMIT` 仅表示自动量程的原始计数越过推荐窗口，仍携带估算 Cp。
- 已确认 `main.c` 当前错误地只接受 `CY_CAPSENSE_BIST_SUCCESS_E`，导致可用 limit 估算值被改写为 `0xFFFFFF`；将改为接受 `SUCCESS` / `LOW_LIMIT` / `HIGH_LIMIT` 且非零值，24-bit 协议有效值饱和至 `0xFFFFFE`。
- PSoC 将从 0.4.9 升至 0.4.10；只允许完整 `main_firmware/build.ps1` 重建 `psoc_fw_image.h`，不手改生成头。
- 当前 selftest 已满足串行读取全部 36 通道、为 0/`0xFFFFFF` 收集失败通道但继续、最后统一失败列表，以及 ch0 500ms/10s 门控与成功路径 min/max/总耗时输出；无需为本次变更扩大它的行为范围。
- 待办：修改 main.c 后按 `main_firmware/build.ps1 → dev.ps1 cycle → diagnose → smoke → full` 验证（最多3轮），记录 embedded PSoC=0x0000040A 与 36通道结果，并仅检查任务源码 diff。

## CP 实机验收补充（2026-07-20）
- 仅修改 `control_software/src/bin/selftest.rs`：在 full 流程恢复自动校准/标准完整处理后、可选 reboot 前新增 CP 实机验收。
- 验收先发送 `measure_cp()`，记录 ch0 测量前版本，按 500ms `request_cp(0)` 与约 16ms `poll()` 等待新版本；0 继续、`0x00FFFFFF` 立即 FAIL、10s 超时 FAIL。随后串行请求 36 个通道，每个请求前记录版本，等待新版本（1s 超时），任何 0 / `0x00FFFFFF` / 断开 / `last_error` / 发送失败均 FAIL + exit(1)。成功路径输出总耗时与 36 通道 min/max。
- 固定入口 `powershell -ExecutionPolicy Bypass -File .\dev.ps1 build-ui` PASS。
- 固定入口 `powershell -ExecutionPolicy Bypass -File .\dev.ps1 full` 连续三次均在 CP 验收失败：ch0 分别于 3040ms（8556fF）、3062ms（8530fF）、3041ms（8517fF）完成；随后 ch20 恒返回失败标记 `0x00FFFFFF`。由于 36 通道未全部通过，未产生有效总耗时/min/max；未放宽失败判定或掩盖该硬件/固件异常。
- `git diff --check` 已运行但失败，原因是既有受跟踪构建产物 `main_firmware/.pio/build/pico/firmware.map` 含大量 trailing whitespace；未改动该产物。另有 CRLF 预警。

## CP BIST limit 修复与硬件闭环完成（2026-07-20）
- PSoC `main.c` 已从 `FW_VERSION_PATCH (9u)` 升至 `(10u)`，版本为 `0x0000040A` / 0.4.10；未触碰 `design.modus` 或 GeneratedSource。
- Cp 循环现在保存 `CY_CAPSENSE_BIST_SUCCESS_E`、`CY_CAPSENSE_BIST_LOW_LIMIT_E`、`CY_CAPSENSE_BIST_HIGH_LIMIT_E` 返回的非零估算 Cp；`BAD_PARAM/HW_BUSY/TIMEOUT/ERROR` 和零值继续保留 `0xFFFFFF`。有效值若会与24位失败标记碰撞则饱和至 `0xFFFFFE`；pending/active 时 GET_CP=0 的语义未改变。
- `control_software/src/bin/selftest.rs` 既有 36 通道串行读取实现复核符合要求：0 / `0xFFFFFF` 入失败通道列表但继续，最终统一 FAIL；ch0 仍按500ms请求、10s门控；成功路径输出所有值、min/max/总耗时。
- 固定闭环第1轮全部通过：`main_firmware/build.ps1` 完成 PSoC/RP/Rust 构建并从 HEX 自动生成 `psoc_fw_image.h`，生成版本 `0x0000040A`、32000字节/250行；`dev.ps1 cycle` 成功刷写并自动更新PSoC；`dev.ps1 diagnose` 与 `dev.ps1 smoke` PASS。
- 实机 `dev.ps1 full` PASS：embedded PSoC=`0x0000040A`；ch0=8634fF（+3044ms）；36通道总耗时=4020ms，min=5421fF，max=400000fF。原先失败的 ch20 现报告可用估算值400000fF，证明 LOW/HIGH_LIMIT 值已不再被错误替换为0xFFFFFF。
- 待办：仅保留本任务源码 `git diff --check` 结果归档与向主agent报告；无需第2轮。
- 36通道明细（fF）：ch0=8634, ch1=8205, ch2=9286, ch3=8895, ch4=11304, ch5=9129, ch6=9715, ch7=7319, ch8=6381, ch9=5691, ch10=5772, ch11=5421, ch12=5421, ch13=5421, ch14=5766, ch15=7514, ch16=7606, ch17=8009, ch18=6056, ch19=5899, ch20=400000, ch21=400000, ch22=5512, ch23=400000, ch24=400000, ch25=5691, ch26=5610, ch27=6603, ch28=6655, ch29=6954, ch30=6876, ch31=6642, ch32=6772, ch33=8921, ch34=8374, ch35=11812.


## CP BIST limit 语义修复（0.4.11，2026-07-20，异常中止）
- 用户更正硬件语义：面板未接时，5.4–11.9pF 的 SUCCESS 小值正常；400000fF 是 `CY_CAPSENSE_BIST_CP_MAX_VALUE` 饱和上限，对应 HIGH_LIMIT 的短接故障，不能作为有效值；LOW_LIMIT 也代表异常小 Cp。
- 已仅修改 `psoc_firmware/CY8C4147AZI-SensorCore/main.c`：`MEASURE_CP` 现在只在 `CY_CAPSENSE_BIST_SUCCESS_E && v != 0u` 时写入 Cp（仍保留与 `0xFFFFFF` 故障标记冲突时饱和至 `0xFFFFFE`）；LOW_LIMIT/HIGH_LIMIT/BAD_PARAM/HW_BUSY/TIMEOUT/ERROR/零值均保留预填的 `0xFFFFFF`。pending/active 时 GET_CP 返回 0 的语义未变。注释已明确 HIGH_LIMIT/400pF 为短接故障、LOW_LIMIT 为异常小 Cp。
- 已将 `FW_VERSION_PATCH` 从 10 更新为 11，版本为 0.4.11 / `0x0000040B`。未编辑 `design.modus` 或 GeneratedSource。
- 固定闭环第1轮：`main_firmware/build.ps1` PASS。PSoC、RP2040、Rust 全部构建成功；自动重建 `main_firmware/src/protocol/psoc/psoc_fw_image.h`，输出32000字节、250行、版本 `0x0000040B`。构建警告保持为 GeneratedSource Button3..35 Sense clock >6MHz 及链接器 RWX 段。
- `dev.ps1 cycle` 命令退出码为0，但其刷写后内置 diagnose 报告 `embedded PSoC=0x0000040A`（而非目标 `0x0000040B`），因此实机结果与预期不符。依据“出问题即停”要求，未执行后续独立 `diagnose`、`smoke` 或 `full`，也不存在本轮可接受的36通道 Cp/min/max/总耗时。需要先排查 cycle/更新器为何未将新嵌入镜像更新到PSoC。
- 待办：在明确版本未更新的根因并修复后，重新从 `main_firmware/build.ps1` 开始执行完整闭环，确认 embedded PSoC=0x0000040B，再收集全部36个 SUCCESS 小Cp值、min/max、总耗时。

## 短接语义修复 0.4.11 完成 + 刷写根因（2026-07-21）
- 用户澄清：面板未接入时所有通道应为小 Cp（SUCCESS）；`400000fF`=`CY_CAPSENSE_BIST_CP_MAX_VALUE` 饱和=HIGH_LIMIT 过量程=短接故障，不应存在。用户已物理修复 2 个短接点。
- 先以当前 0.4.10 固件跑 full：原 400pF 的 ch20/21/23/24 已恢复为 ~5.5–5.9pF，全 36 通道正常，证实 400pF 确为短接、非有效值；这也暴露 0.4.10 把 HIGH_LIMIT 当有效值会掩盖短接。
- PSoC main.c 已升 0.4.11 / `0x0000040B`：MEASURE_CP 仅接受 `CY_CAPSENSE_BIST_SUCCESS_E && v!=0`；LOW_LIMIT/HIGH_LIMIT/其它/零值一律保留 `0xFFFFFF`，恢复短接故障检测。selftest 逻辑不变（0/0xFFFFFF 记失败集合、统一 FAIL）。build.rs 从生成头注入 EXPECTED_PSOC_FW_VERSION，smoke 随重编更新为 0x40B。
- 刷写异常根因：`dev.ps1 cycle` 在设备已处于“陈旧 BOOTSEL 状态”时复制 UF2 未干净落盘，设备重启回旧 0.4.10 app（flash 未被覆盖），故连续 2 次 cycle + 内置 diagnose 都报 embedded=0x0000040A。经 clean 重编（`pio run -e pico -t clean` + build.ps1）确认 UF2 版本字节扫描不变（0x40A×1 为巧合数据、0x40B×2 为真实版本+镜像），排除构建问题。改用分步刷写：`dev.ps1 bootsel`（新鲜进 BOOTSEL，确认 G: 为 RPI-RP2 v3.0 卷）→ 复制 UF2（观测 G: 1s 内消失=设备接收并重启）→ 等 10s → diagnose，成功刷入。
- 固定验收全部 PASS（0.4.11）：diagnose embedded PSoC=`0x0000040B`；smoke PASS；full PASS。
- 实测 full：ch0=8517fF（+521ms）；CP 总耗时=1497ms；36 通道 min=5408fF、max=11812fF；全部 SUCCESS 小 Cp，无 0 / 无 0xFFFFFF / 无 400pF。
- 36 通道明细（fF）：ch0=8517,1=8205,2=9338,3=8895,4=11343,5=9181,6=9715,7=7202,8=6303,9=5698,10=5776,11=5408,12=5499,13=5421,14=5792,15=7449,16=7553,17=7983,18=6043,19=5886,20=5880,21=5860,22=5431,23=5505,24=5505,25=5769,26=5623,27=6629,28=6681,29=6993,30=6863,31=6694,32=6759,33=8882,34=8439,35=11812.
- 目标源码 git diff --check 干净（仅 LF→CRLF 提示）。未创建提交；未清理用户既有文件；本轮临时脚本 _scan_uf2_ver.ps1/_flash_observe.ps1 已删除。经验：cycle 固定 9s 等待 + 在陈旧 BOOTSEL 下复制不可靠，后续刷写应先新鲜 bootsel 再复制并确认 G: 消失。
