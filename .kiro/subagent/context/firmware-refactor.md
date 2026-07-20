# firmware-refactor — task #5 PSoC snapshot and dual CDC integration

## 任务目标与鉴权
- 鉴权通过：仅在 `f:\mai2control\mai2control-v4` 内修改源码/构建产物，并仅通过固定 `main_firmware\build.ps1` 对唯一受门控的 RP2040 设备构建、烧录与验证。
- 完成真实 36 通道 CapSense SPI 快照、SensorLink 真数据、serial/light 双 CDC 复用、Build/Flash/WinUSB/CDC smoke。
- 禁止其他命令入口、固定盘符写入、Git 提交/推送和递归 subagent。

## 已恢复与已读架构
1. 已读取旧同名记录；其 IMO trim snapshot 改动属于既有用户工作，必须保留，不回退。
2. 已完整检查固定 `main_firmware/build.ps1`：当前 `$WorkflowMode = "Flash"`；已包含 PSoC make、镜像生成、PlatformIO、cargo locked、BOOTSEL、唯一 `RPI-RP2 + INFO_UF2.TXT`、REV_0401 拓扑与 WinUSB smoke。
3. PSoC `main.c` 当前仅有 7-byte PING/PONG，CapSense 扫描完成后 Process+Scan；待根据 generated context/API 实现快照发布。
4. RP2040 `psoc_types/psoc_spi/psoc` 当前同为 7-byte ping；`SensorLink` 仍生成 sin/伪随机桩数据，PARAM/CALIBRATE/BASELINE_RESET 还会假 ACK，必须修正边界。
5. USB HAL 已有 serial/light 独立 CDC ring buffer；需新增 HAL_UART adapter，且 adapter `deinit()` 仅停用自身。
6. `Mai2Serial` 固定 6-byte `{lrscv}` 命令，STAT 为 cmd `A`，成功后持续发 9-byte `(` + 7×5-bit touch + `)`；有效 touch mask 为低 35 bit。`Mai2Light` 支持二进制 GET_PROTOCOL_VERSION(0x12) 和字符串 `/version`，内部维护 11 LED 状态。
7. `mode.work` 来自 ConfigManager，0=Serial、1=HID；默认 0。现有 LedService 只管板载 RGB 状态灯，需继续调查 NeoPixel 管理方或提供明确灯态消费缓存接口。
8. `main.cpp` 当前每秒向 config WinUSB 写裸 ASCII 联调诊断，必须删除/门控；初始化顺序为 config→Psoc/SWD flash→USB→UsbComm/SensorLink，循环为 USB task→UsbComm→SensorLink→Psoc update。

## 当前关键决策
- SPI 快照使用固定长度、无动态分配的页式请求/响应；PSoC 在 ProcessAllWidgets 后构建 staging，再在短临界区发布，SPI 只读 immutable published snapshot，避免撕裂。
- 目标尽可能传完整 raw/baseline/diff/status；先以 generated `cy_capsense_context` 的真实结构和 API 为准，不猜字段。
- SensorLink 无有效快照时只输出 0/unavailable，绝不生成数据；未实现的参数写入/校准/基线重置返回明确 NAK。
- Game I/O 仅在 WORK_SERIAL 初始化/task，直接复用 Mai2Serial/Mai2Light；serial touch 来自 PSoC active mask。

## 已知异常
- 一次只读 grep 工具参数类型写错被工具拒绝，未执行任何工作区/设备操作；随后已用合法参数完成检索。当前实际 PSoC generated 文件尚未定位，需继续通过构建清单和文件名查找。

## 待办
1. 定位并读取实际 generated CapSense 配置、36 sensor 顺序、中间件结构/API。
2. 设计并实现 PSoC/RP2040 固定页式快照协议与 SensorLink 真数据。
3. 实现 CDC HAL_UART adapter、GameIo service、灯态消费接口，接入 main 并移除裸诊断。
4. 将 build.ps1 切 Build，加入最终 CDC smoke 前先完整回读；执行固定命令并修复构建。
5. 切回 Flash，执行唯一固定命令完成真机闭环，记录证据。

## 2026-任务#5 调查里程碑
- 已确认 generated `cycfg_capsense.h` 定义 36 个独立 Button widget，WDGT_ID 严格为 Button0=0 … Button35=35，每个只有 SNS0；`cy_stc_capsense_tuner_t` 中 widgetContext[36] + sensorContext[36]。
- 已完整读取当前 PSoC `main.c`：SCB0 SPI 从机只有 7-byte PING/PONG；主循环在 ProcessAllWidgets 后立即 ScanAllWidgets，尚无快照。
- 已读取 RP PSoC/SensorLink：同为 7-byte ping；SensorLink 仍使用正弦/伪随机数据并对配置/校准假 ACK。
- 已读取 USB/UART：serial/light 独立 CDC ring buffer 已存在；需新增 HAL_UART adapter，adapter deinit 只能停自身。
- 已读取协议和主循环：Mai2Serial 有效为低35位；Mai2Light 管 11 路状态；当前无 game I/O service；main 仍每秒向 WinUSB 写裸诊断。build.ps1 当前 Flash 且已有唯一卷、拓扑和 WinUSB 门控。
- 下一步：从实际中间件路径提取 sensor context 字段；实现 PSoC/RP 分页不可变快照、SensorLink 真数据，再新增 CDC adapter/game I/O 与 smoke。

## 2026-任务#5 实现里程碑
- PSoC `main.c` 升至 0.4.1：每次 ProcessAllWidgets 后按 Button0..35 读取 `ptrWdConfig[ch].ptrSnsContext[0]`，双缓冲发布 252B raw/bsln/diff/status；BEGIN 锁存到 transfer buffer，63 个 4B 页稳定传输。
- RP `psoc_types/psoc_spi/psoc` 已实现 7B SCB 安全流水协议，严格验证 magic/cmd/seq，输出 generation/valid 和 36 通道真实快照；失败清空快照并保留 ping health。
- SensorLink 已移除 sin/伪随机和参数表，遥测只读 PSoC 快照；无效时明确全0；PARAM/SET/GET_ALL/CALIBRATE/BASELINE_RESET 全部 NAK NOT_IMPLEMENTED。
- 新增 `HAL_USB_CDC_UART`：独立 serial/light CDC ring 转发，deinit 只停自身；新增 `GameIoService` 仅 Serial 模式运行，复用 Mai2Serial/Mai2Light，触摸取真实 active mask，11 路灯态进入公开 generation 缓存接口。
- main 已移除周期裸 WinUSB ASCII，USB 枚举前预取快照，循环更新 PSoC 后再驱动 SensorLink/GameIo。DEVICE_INFO/控制端新增 generation/link_valid/snapshot_valid，WinUSB smoke 将其作为硬件通过条件。
- 下一步：修正可能的编译接口问题，给 build.ps1 加 MI_01/MI_03 自动 CDC smoke，切 Build 并执行唯一入口。

## 2026-任务#5 Build 验证里程碑
- `Read-SerialExact` 已改为 `return ,$buffer`，避免 PowerShell pipeline 将 `byte[]` 展开为 `Object[]`；固定脚本已完整回读确认。
- 仅执行固定命令 `& "f:\mai2control\mai2control-v4\main_firmware\build.ps1"`，Build 模式成功（exit 0）。
- PSoC 0.4.1 构建成功：Internal Flash utilized 25900B；嵌入镜像 25984B、203 rows、version `0x00000401`。
- RP2040 PlatformIO 成功：RAM 26788/262144B (10.2%)，Flash 466572/3141632B (14.9%)；新增 CDC adapter、game_io、PSoC/SensorLink 均完成编译链接。
- Rust `cargo build --locked --bin selftest` 成功；workflow 报告给出 `BUILD PASS`。PlatformIO 清理阶段因 transcript 位于 `.pio/build/pico` 出现一次 WinError 32/145 警告，但构建继续且完整成功；非功能性失败。
- 下一步：将脚本唯一 mode 切回 Flash，完整回读后执行同一固定命令，验证 UF2 门控、REV_0401/MI_00/01/03、WinUSB 真快照和双 CDC smoke。

## 2026-任务#5 Flash 异常停止
- 固定脚本已切为 `$WorkflowMode = "Flash"` 并完整回读，确认仍保留唯一卷、REV_0401/MI_00/01/03、exactly 2 COM、WinUSB 与 CDC 门控。
- 仅执行固定命令一次；PSoC/RP2040/Rust 再次构建成功。旧固件 DEVICE_INFO 为 protocol1/fw0x400/36ch/caps0x7，generation=0/link=false/snapshot=false；随后 WinUSB BOOTSEL 请求返回 `BOOTSEL REQUESTED`（endpoint stall 是复位时预期断链表现）。
- 硬件阻塞：20 秒内 Windows 未枚举出唯一的 `DriveType=2 + VolumeName=RPI-RP2 + INFO_UF2.TXT` 卷，脚本按门控抛错并 exit 1；未执行 UF2 Copy-Item，因此未烧录新 RP/PSoC，也未进行新固件 WinUSB snapshot 或 MI_01/MI_03 CDC smoke。
- 按“出问题即停”“不可恢复硬件阻塞立即停止”要求未重试、未绕过门控。恢复条件：人工让 RP2040 进入可被 Windows 枚举的 BOOTSEL（必要时按住 BOOTSEL 重插）；随后重新执行同一固定命令即可从 Flash 流程续做。

## 2026-白灯预烧录指示里程碑
- `hardware.txt` 唯一拓扑证据：白灯为 PSoC `P1.6 -> LED -> R -> GND`，高电平点亮；RP2040 仅直连 PSoC XRES/SWD/SPI，不直连白灯，不能用板载 RGB 冒充。
- generated `cycfg_pins.h/.c` 核对：`CYBSP_LED_SLD3 = GPIO_PRT1 pin 6`，`CY_GPIO_DM_STRONG_IN_OFF`，初始输出 1；本地 PDL 明确 POR 时 GPIO High-Z，`Cy_GPIO_Pin_Init` 顺序为 OUT→drive mode→HSIOM。
- 兼容旧 0.4.0：其 PING 每 4 次翻转导致运行态不确定；RP 在 `acquire()` 前先 XRES 正常复位、等待 75ms，让旧固件 generated 启动态确定为高，再发 `INDICATOR_ON`。旧固件将未知命令回 PONG且不翻转，随后仅一笔 PING，因此保持高；失败只记诊断，不阻止烧录。
- acquire 必须 XRES，期间 PSoC GPIO 回 High-Z，因无 RP 直连无法避免短暂熄灭。acquire 成功后、读取保护/erase/program 前，RP 按本地 `CY8C4147AZI-S455`/`cyip_gpio_v2.h`/`cy_gpio.h` 定义经 SWD 设置 P1.6：DR_SET bit6、PC DM6=strong、PC2 bit6=input-off、HSIOM GPIO，并全回读验证；失败不阻止可恢复烧录。
- 新 PSoC 0.4.1 定义确定灯态：启动高；合法 SPI 帧高；坏 magic 低；新增 `INDICATOR_ON(0x20)` 显式高；移除每4次 PING 翻转。正常 erase 不复位 GPIO，故 SWD 恢复后可跨 erase/program 保持；PROTECTED→OPEN 需二次 acquire 时再次恢复。
- 已修改：PSoC `main.c`；RP `psoc_types.h`、`psoc_spi.h/.cpp`、`psoc_swd.h/.cpp`、`psoc.h/.cpp`、`main.cpp`。下一步完整回读修改与固定脚本，切 Build 后只执行固定 `build.ps1`。

## 2026-白灯 Build 验证里程碑
- 固定 `build.ps1` 已仅把 mode 暂切 `Build` 并完整回读；仅执行唯一命令一次，exit 0。
- PSoC 0.4.1：编译/链接成功，Internal Flash 25900B；嵌入镜像 25984B/203 rows/version `0x00000401`。
- RP2040：新增旧固件预点灯、显式 SPI 命令、SWD P1.6 恢复与回读验证全部编译链接成功；RAM 26788B (10.2%)，Flash 467036B (14.9%)。
- Rust WinUSB selftest `cargo build --locked --bin selftest` 成功；最终 `BUILD PASS`。PlatformIO 启动时 transcript 占用 `workflow-report.txt` 的 WinError 32/145 为既有非致命清理告警，构建与产物成功。
- 下一步：把固定脚本切回 Flash、完整回读，然后只执行同一命令一次；由脚本门控唯一 RPI-RP2 卷、REV_0401、WinUSB snapshot 与双 CDC。

## 2026-白灯 Flash 异常停止
- 固定脚本已切回 `Flash` 并完整回读；仅执行同一固定命令一次。
- 构建再次全部成功。脚本发现唯一安全目标 `G:\`：DriveType=2、VolumeName=RPI-RP2、`INFO_UF2.TXT` 存在；UF2 已复制到该卷。
- 应用枚举后 WinUSB selftest 连接成功，但 DEVICE_INFO 仍报告旧状态：protocol=1、fw=`0x00000400`、36ch、caps=`0x7`、generation=0、link_valid=false、snapshot_valid=false。脚本据此 FAIL 并 exit 1。
- 无法确认新 RP2040 镜像实际启动，也无法确认新增白灯时序/PSoC 0.4.1 已在真机运行；按硬件阻塞/结果异常即停，未重试、未绕过，未执行 MI_01/MI_03 CDC smoke。
- 恢复前需人工确认：安全卷 `G:\` 对应的 BOOTSEL RP2040 与 WinUSB `VID_2E8A/PID_000A` 是否确为同一物理板，以及 UF2 复制后该板为何仍报告旧 fw 0x400；排除多板/旧实例歧义后再运行同一固定脚本。

## 2026-USB2 Hub/S455 用户确认与恢复点
- 用户确认：`G:\ RPI-RP2` 与当前 `VID_2E8A/PID_000A` 是同一块唯一目标板，现场无第二块目标设备；此前“卷与应用设备可能不是同板”的歧义已消除。
- 实物 PSoC 明确为 `CY8C4147AZI-S455`，硬件连接以仓库根目录 `hardware.txt` 为权威。
- 肉眼现象：白灯上电约亮 500ms 后熄灭，随后 RP2040 红灯常亮；白灯实际观察仍只能由用户确认，自动化不得编造。
- 连接拓扑：目标 RP2040 经 USB2.0 Hub；第二次自持烧录循环后常从 Hub 子设备树完全消失，Hub 自身仍正常，必须物理重插 Hub 才恢复。
- 本轮鉴权通过：范围仅工作区、唯一 `VID_2E8A/PID_000A`、其连接 PSoC 与唯一 `DriveType=2 + RPI-RP2 + INFO_UF2.TXT` 卷；不得重置/禁用 Hub，歧义或目标完全消失立即停。
- 当前阶段：先核对 S455/S475 本地 device pack/BSP、generated pin、SWD geometry 与 USB bootrom/TinyUSB 路径，再做最小修改。

## 2026-S455/USB2 Hub 调查里程碑
- 本地 `device-db release-v4.37.0` 精确支持 `CY8C4147AZI-S455`；其 `studio_5.0/view.xml` 证据：PSoC4AS3、PSoC 4100S Plus、64-TQFP、48MHz、128KiB Flash、16KiB SRAM、SiliconID `0x257011B5`。当前 S475 为 `0x257C11B5`，主要可见差异是 S475 有 CAN、S455 无 CAN；两者不是同一 silicon ID，禁止继续把 S475 当准确 target。
- 本地 PDL 2.21.0 有精确 `cy8c4147azi_s455.h`，dispatcher 也支持 `CY8C4147AZI_S455`；其内存/寄存器、P1 GPIO/HSIOM、SCB/CSD 与当前所用资源一致，因此可直接将 BSP/device-configurator target 正确切为 S455，不需兼容性豁免。
- 白灯证据：`hardware.txt` 为 P1.6→LED→R→GND（高有效）；generated pin 为 P1.6、初始 outVal=1、STRONG_IN_OFF。旧 0.4.0 每4次 PING 翻转，能解释“约500ms亮后灭”；RP 红灯来自 main loop 的 `flash_ok = acquired && program_ok && verify_ok`，任一失败即红常亮。
- Arduino-Pico `rebootToBootloader()` 只是立即调用 Pico SDK `reset_usb_boot(0,0)`；Pico SDK 再直接进入 ROM，均不先 TinyUSB detach。TinyUSB RP2040 `dcd_disconnect()` 明确只清 `USB_SIE_CTRL_PULLUP_EN`；当前 100ms ACK等待后立即 BootROM reset、应用启动仅120ms断开，USB2 Hub 可能未稳定观察到一次完整物理断连，符合 Hub 正常但子设备消失的现象。
- 修复方向：ACK drain/泵任务→明确 D+ soft-disconnect→保持400ms→直接 `reset_usb_boot`；应用初始化只 init 一次并保持350ms disconnect；脚本等待 BOOTSEL 卷稳定、copy 后先等卷消失、再要求应用拓扑连续稳定至少2秒。

## 2026-S455/Hub 修复实现里程碑
- PSoC authoritative target 已从 S475 切为 S455：`bsp.mk`、`design.modus`、generated device validation 均为 `CY8C4147AZI-S455`；本地 PDL 精确 header 将由 `CY8C4147AZI_S455` 选择。
- RP SWD 在 acquire 后、保护读取/erase/program 前调用 GET_SILICON_ID，必须等于本地 device-db/header 的 `0x257011B5`；不符则只 reset/release，绝不擦写。flash 边界补为128KiB/1024×128B，converter 上限收紧到0x20000。
- 白灯时序保持：旧应用 XRES→75ms→显式/兼容 indicator command；acquire 成功后立即 SWD 恢复 P1.6 高，再开始 protection/clock/erase/program。新 PSoC 0.4.1 启动高、合法帧高、坏 magic 低、无 PING toggle。
- USB reboot 改为 `ACK_DRAIN(200ms)`（持续 flush/task）→TinyUSB/RP2040 D+ pull-up soft-disconnect→`DETACHED(400ms)`→`reset_usb_boot(0,0)`；不再在响应处理栈内立即调用 Arduino-Pico wrapper。应用 TinyUSB 只 init 一次，启动 disconnect 窗口从120ms增为350ms。
- 固定 `build.ps1` 增加每轮 UTC+GUID RunId 与独立日志；BOOTSEL 卷需连续4×250ms稳定，copy 后必须先消失；应用 REV0401/MI00/MI01/MI03/exact2COM 需连续4×500ms稳定，目标消失/身份变化只停止，不操作 Hub。下一步完整回读，切 Build 后仅执行固定脚本。

## 2026-S455/Hub Build 验证里程碑
- 固定 `build.ps1` 已切为 `Build` 并完整回读；仅执行 `& "f:\mai2control\mai2control-v4\main_firmware\build.ps1"`，RunId `20260717T184336010Z-ec9817a3`，exit 0，最终 `BUILD PASS`。
- `.mtbqueryapi` 确认 `MTB_DEVICE=CY8C4147AZI-S455`、`MTB_MPN_LIST=CY8C4147AZI-S455`；PSoC 镜像版本 `0x00000401`、25984B、203×128B。
- RP2040 构建成功：RAM 26788/262144B (10.2%)，Flash 467324/3141632B (14.9%)；Rust `cargo build --locked --bin selftest` 成功。
- 下一步：把固定脚本切回 `Flash` 并完整回读；执行同一固定命令第一轮，PASS 且应用拓扑稳定后再执行第二轮。任何目标消失或身份歧义立即停止，不重置 Hub。

## 2026-S455/Hub 第一轮 Flash 异常停止
- 固定脚本已切回 `Flash` 并完整回读；仅执行唯一命令一次。RunId `20260717T184728887Z-b1984426`，日志 `main_firmware/.workflow-logs/workflow-20260717T184728887Z-b1984426.log`。
- S455/PSoC 0.4.1、RP2040、Rust 构建均成功：PSoC 25900B，嵌入镜像 25984B/203 rows/version `0x00000401`；RP RAM 26788B、Flash 467324B。
- 旧应用成功响应 BOOTSEL 请求；唯一稳定安全卷为 `G:\`（removable、RPI-RP2、INFO_UF2.TXT），UF2 已复制，随后该卷按门控消失，应用 REV_0401/MI_00/MI_01/MI_03/exact2COM 拓扑恢复并通过2秒稳定等待。
- 异常：恢复后的 WinUSB DEVICE_INFO 仍为旧固件 `0x00000400`、generation=0、link_valid=false、snapshot_valid=false；selftest exit 1，脚本 exit 1。新镜像 `0x00000401` 未被观察到实际运行。
- 按出问题即停要求：未重试第一轮、未执行第二轮、未操作/重置 Hub、未运行双 CDC smoke。需先调查“唯一同板 UF2 copy 后仍启动旧 0x400”的物理/BootROM/UF2落盘原因；不得在无新证据下继续刷写。


## 2026-configure_flash_clock/ERASE_ALL 深度审计里程碑（纯检索，未改代码）
- 权威依据来源已锁定：本地 PDL `mtb-pdl-cat2 release-v2.21.0` 的 `devices/include/cy8c4147azi_s455.h`（confirms `CY_SILICON_ID=0x257011B5`、`CY_FLASH_SIZE=0x20000`、`CY_SRAM_SIZE=0x4000`、`SRSSLT_BASE=0x40030000`）+ `ip/cyip_srsslt.h`（真实寄存器结构体，S8SRSSLT，非 psoc4100t_plus_trm.txt 描述的新款 M0S8SRSS）+ `ip/cyip_sflash_psoc4100sp.h`（真实 SFLASH 结构体，含 IMO_TCTRIM_LT[25]@0x34C、IMO_TRIM_LT[25]@0x365）+ `drivers/source/cy_flash.c`（官方 flash 驱动真实调用序列）。**此前调研记录里引用的 `psoc4100t_plus_trm.txt` 是 PSOC 4100T Plus（新款 SRSS/M0S8）的 TRM，寄存器布局与本芯片实际使用的 SRSSLT（PSoC4100S Plus 系列）不同源，只能作背景参考，不能作为寄存器地址权威**；真正权威是上述 PDL 头文件，本次审计以其为准。
- **configure_flash_clock() 逐寄存器核对结果（确定正确）**：
  - `CLK_SELECT`(0x40030028)、`CLK_IMO_TRIM1`(0x40030F0C)、`CLK_IMO_TRIM2`(0x40030F10)、`CLK_IMO_TRIM3`(0x40030F18)、`CLK_IMO_SELECT`(0x40030F08)、`TEST_MODE`(0x40030014) 均与 `SRSSLT_Type` 结构体逐字节偏移量精确匹配（SRSSLT_BASE=0x40030000 + 偏移）。
  - `CLK_SELECT` 位域：`HFCLK_SEL[1:0]`(Pos0,Msk0x3)、`HFCLK_DIV[3:2]`(Pos2,Msk0xC)、`PUMP_SEL[5:4]`(Pos4,Msk0x30) — 代码里 `(w&~(0x3<<4))|(0x1<<4)` 等写法与 `SRSSLT_CLK_SELECT_*_Pos/Msk` 逐位精确一致。
  - `CLK_IMO_TRIM1.OFFSET[7:0]`(全字节)、`CLK_IMO_TRIM3` 打包 `STEPSIZE[4:0]+TCTRIM[6:5]`(Pos0/Msk0x1F 与 Pos5/Msk0x60) — 与 SFLASH `IMO_TCTRIM_LT` 字节内位域布局逐位相同，因此代码"整字节搬运 SFLASH trim 字节到 CLK_IMO_TRIM3"是位对齐正确的搬运，非凑巧。
  - **trim 源地址核算（确定正确）**：SFLASH 结构体基址 0x0FFFF000；`IMO_TCTRIM_LT[25]` 位于结构体偏移 0x34C，`IMO_TRIM_LT[25]` 位于偏移 0x365。48MHz 挡对应数组下标 24（progspec Table1-1 footnote：0=24MHz...6=48MHz 分别对应 LT0/LT4/LT8/LT12/LT16/LT20/LT24，数组下标=LTxx 的 xx）。算出 `IMO_TRIM_LT[24]` 绝对地址 = 0x0FFFF000+0x365+24 = **0x0FFFF37D**；`IMO_TCTRIM_LT[24]` 绝对地址 = 0x0FFFF000+0x34C+24 = **0x0FFFF364**。代码用 `SFLASH_IMO_TRIM_LT24_WORD=0x0FFFF37C` 整字读出后取字节车道1 `(w>>8)&0xFF`（即字节地址 0x0FFFF37D）——精确命中；`SFLASH_IMO_TCTRIM_LT24_WORD=0x0FFFF364` 取车道0 `w&0xFF`（即字节地址 0x0FFFF364）——精确命中。**两个 trim 源地址与真实设备头文件完全吻合，非猜测。**
  - IMO 频率升档序列（24→24trim→中间档44(=5)→目标48(=6)，各步 sleep）与 TRM 通用 "Change IMO frequency" 流程图（Figure 9-2：先设回24M→读粗调trim→清细调→读温漂trim→等50 IMO周期→若目标>24M先切到"desired-1"中间档→再等50周期→切到目标）逐步吻合，且 20µs 延时远超"等50 IMO周期"(最坏情况在24MHz下50周期≈2.08µs)，是保守安全的超配，非 bug。
  - 时序上代码先把 IMO 完全升到48MHz并验证成功，再切 `CLK_SELECT` 让 HFCLK/PUMP 真正切到 IMO 源——这是更保守的顺序（避免 HFCLK 在 IMO 变频过程中跟着抖动），优于"先切源再变频"，架构上更稳。
  - **结论：configure_flash_clock() 的寄存器地址、位域、trim 来源、写入顺序全部审计通过，是"确定正确"，不是 ERASE_ALL 失败的根因。**
- **强证据支持该结论**：SROM 对"Invalid Flash Clock"有专用错误码 `0xF0000012`（AN84858 Table12 与 PDL `SROMCODE_INVALID_CLOCK` 完全对应），若 configure_flash_clock() 配置的时钟状态不满足 SROM 内部对 ERASE_ALL 前置条件的判定，SROM 应直接返回 0xF0000012，而不会让擦除动作真正执行到"擦后校验"阶段。当前观测到的稳定返回码是 **0xF000000A**（"Checksum Zero Failed"，AN84858 Table12："计算出的 checksum 不为零"），这是一个只有在 SROM 已经把擦除动作跑完、进入内部擦后自检阶段才会出现的错误码，说明 SROM 判定"时钟有效"这一关已经通过（并非 F0000012），also 未命中 F0000001/F0000014（保护态不对）——**因此"时钟配置是根因"的可能性被现场返回码本身证否，应定为"高概率非根因"**（无法 100% 排除 SROM 内部有寄存器级不可见的额外副作用，但现有证据强烈指向时钟不是问题）。
- **SRAM_PARAMS_BASE=0x20001000 vs 通用表 0x20000100 审计结论（确定不是根因）**：细读 progspec 4.6.1/4.8.1 等所有伪代码，SROM 通过读取 `CPUSS_SYSARG` 里host写入的指针值来定位参数块地址，即 `WriteIO(CPUSS_SYSARG, SRAM_PARAMS_BASE)`——SROM 并不硬编码某个固定物理地址，"0x20000100"只是 progspec 通用示例选用的一个惯例值。**决定性交叉证据**：官方生产级 PDL `cy_flash.c` 里 `Cy_Flash_WriteRow/StartWrite/RowChecksum` 全部用 `CPUSS_SYSARG = (uint32_t)&parameters[0]`——即用 C 局部数组（编译器/链接器决定的任意栈地址，绝不等于 0x20000100）作为参数块地址，且这是 Infineon 官方真实产品驱动，说明地址完全可自由选择，唯一约束是"合法可写的 SRAM 地址、字对齐"。本设备 SRAM 范围 0x20000000–0x20003FFF（16KiB，见 `CY_SRAM_SIZE=0x4000`），0x20001000 在范围内、4KB 对齐，且刻意避开了 bkpt 备用 acquire 路径用到的 0x20000000–0x20000800（该路径当前未被主 acquire() 流程调用，无冲突）。**结论：SRAM_PARAMS_BASE 取值合法且不是导致 ERASE_ALL "Checksum Zero Failed" 的原因。**
- **ERASE_ALL "Checksum Zero Failed" 语义定级（高概率）**：AN84858 Table12 原文仅笼统给出"计算出的 checksum 不为零"，未指明是哪个阶段的 checksum；结合本芯片架构文档明确记载"擦除后 flash 内容读回为全 0x00（不是常见的全 0xFF）"（progspec Appendix A："EraseAll() 清空整行，把每个字节复位为0"），可判定 0xF000000A 对 ERASE_ALL 而言，最合理的语义是**擦除动作执行完毕后，SROM 内部对刚擦除区域做了一次"求和应为0"的自检读回校验，校验失败**——即真实的擦除后内容非全零，而不是参数/保护/时钟类前置检查失败（那些各自有专属错误码 0xF0000001/0004/0005/0012/0014，均未出现）。**未证实但列为高概率候选的具体物理成因**：(a) 擦除脈冲电荷泵/时序仍不足以让阵列真正擦净（即便时钟寄存器配置正确，不能100%排除 SROM 内部还依赖某个我们审计不到的隐藏状态）；(b) 该芯片非"全新出厂片"，之前被 kit 示例固件写入过 User SFlash（应用专用 4 行，ERASE_ALL 明确不擦除这部分，见 TRM 23.5.6/progspec 2.2）或其他 ERASE_ALL 未覆盖区域，若 SROM 的擦后自检覆盖范围恰好包含了这些"从不被 ERASE_ALL 清空"的残留非零内容，会导致即使擦除本身完全成功也仍报 Checksum Zero Failed——这一情形只在"该颗芯片曾被编程过"时才会出现，在纯 VIRGIN 片上不会重现。
- **WRITE_ROW(0x05) vs PROGRAM_ROW(0x06) 结论（供后续参考，未改代码）**：TRM 23.5.4/23.5.5 与 progspec 4.8 明确：标准"ERASE_ALL 全片擦除 → 逐行 PROGRAM_ROW"流程里，因为 ERASE_ALL 已经把所有行擦净，Program 阶段官方推荐用 `PROGRAM_ROW`(0x06)（假定行已擦除，不再重复擦除，更快）；`WRITE_ROW`(0x05) 语义是"擦除+编程二合一"，用于单行重写场景，不依赖之前是否擦除过。当前 `program_row()` 使用 0x05（WRITE_ROW）不是错误（更稳健、对残留非零内容有自愈能力），只是比标准流程"多做一次每行擦除"更慢；按任务要求不在此阶段改动。
- **诊断建议（若要继续定位 Checksum Zero Failed 根因，最小新增寄存器/信息集合）**：
  1. `erase_all()` 成功后立即追加一次 `checksum_all()`（已有实现，SROM opcode 0x0B）调用并记录返回的 28-bit checksum 原始值——目前代码只在失败时报 0xF000000A，从未打印"到底非零到多少"；把这个数值也上报，能直接分辨是"全片小范围残留"还是"大面积未擦净"。
  2. 在 erase 失败后，用现有 `verify_flash()`/裸 `_read_io()` 直接读回 flash 起始若干行（如 row0 全 128B）和 User SFlash 4 行（`SFLASH_MACRO0` 附近的 Supervisory 区），对比是否为全 0x00，用以区分"用户主阵列没擦净"还是"SFlash 残留"两种候选原因，不需要新增寄存器地址，只需要用现有 `_read_io` 多读几处。
  3. 若怀疑电荷泵/擦除脈冲本身能量不足，可读回 `CLK_SELECT` 当前值（现有 `configure_flash_clock()` 返回前已验证过 `PUMP_SEL==1`，无需新增），结合示波器测 XRES 引脚附近 VDD 纹波（超出固件可诊断范围，仅作记录）。
- 本次为纯本地检索/审计，未修改任何源码，未运行烧录。

## 2026-07-17 ERASE 只读取证与 PROGRAM_ROW 收敛
- DEVICE_INFO v1 保持前 69B 不变，追加 10×u32（实际五个时钟寄存器 + 擦后 Flash byte-sum/word-OR/首非零 word/读取 word 数），report length=109、总 payload=124B；Rust 继续接受 `report_length>=69`，仅 `>=109` 解析 `Option<EraseFailureDiagnostics>`。
- `erase_all()` 仅在原始状态精确为 `0xF000000A` 时扫描完整 128KiB 用户 Flash；读失败保留进度，扫描后恢复原 ERASE 状态；未访问 User SFlash、未发第二次 ERASE。
- Build PASS：RunId `20260717T193909945Z-59602bc3`，PSoC 25,900B，RP RAM 26,784B / Flash 468,460B，UF2 SHA256 `3A090C532935F9897319D6FC1F1793DFA1E46C9A843CD91955AE0E85CD126A09`，size 961,536。
- 诊断 Flash RunId `20260717T194000690Z-0f2e27ba` 成功写入 RP2040。新证据：ERASE=`0xA0000000`、PROGRAM=`0xA0000000`、CHECKSUM=`0xA0000000`，但 VERIFY 在 addr0 读 `0x00000000`、期望 `0x20004000`；flags=`0x23FE`，时钟实际值 select=`0x10`、imo=`0x06`、trim1=`0xDE`、trim2=`0x00`、trim3=`0x0C`。说明 ERASE/时钟问题已消失，旧 `WRITE_ROW(0x05)` 路径报告成功但未写入主 Flash。
- 按编程规范 Step 5 将行操作恢复为 `LOAD_LATCH(0x04) -> PROGRAM_ROW(0x06)`；参数布局不变。Build PASS：RunId `20260717T194408812Z-858251f6`，UF2 SHA256 `4C5603BDAAF3E994FD14ECE88F8D474951FA95C480AAEF08A6EABD0A5519B020`，size 961,536。
- 恢复 Flash RunId `20260717T194505733Z-e418545f` 在旧 RP WinUSB 成功请求 BOOTSEL 后，25 秒内未出现唯一 `RPI-RP2 + INFO_UF2.TXT` 卷；脚本按门控停止，未 Copy UF2，因此 0x06 修复镜像尚未安装、未再次触碰 PSoC、未运行 CDC。
- `main_firmware/build.ps1` 已恢复为 `$WorkflowMode = "Build"`。恢复条件：用户人工插拔 USB hub/让该 RP2040 进入 Windows 可见的 BOOTSEL；之后再切 Flash 并执行唯一固定命令一次。


## 2026 深审里程碑：官方 PDL cy_flash.c 逐字核对 → 根因改判（纯检索，未改代码，未碰 ERASE/Flash）
任务：深审 psoc4_progspec.txt / an84858_hssp.txt(context) / S455 PDL `cy_flash.c` / device headers / `psoc_swd.cpp` / OpenOCD-HSSP 资料，回答四问。

### 1) 根因排序（决定性新证据 → 推翻此前"时钟寄存器确定正确非根因"的结论范围）
**排序不变的部分**：SRAM_PARAMS_BASE=0x20001000 合法（不是根因）；WRITE_ROW(0x05)/PROGRAM_ROW(0x06) 二者等效（不是根因，用户实测两者结果完全相同印证了这一点——因为二者共享同一个更深层的门控失效点，不是行为差异）。

**新的第一位根因（取代此前"时钟配置确定正确"结论）**：
官方 Infineon PDL `mtb-pdl-cat2 release-v2.21.0/drivers/source/cy_flash.c` 的 `Cy_Flash_WriteRow()` 逐字显示，真实产品驱动对每一次 flash 写操作的调用序列是**无条件**：
```
LOAD_LATCH(0x04, SRAM指针参数)
  → ProcessStatusCode 确认成功
  → 进临界区
  → Cy_Flash_ClockBackup()   // SROM opcode 0x16，SRAM指针参数：word0=key, word1=&backup_buf[6]
       → 成功后调用 Cy_SysClk_ImoEnable()（非SROM，普通寄存器，使能IMO）
  → Cy_Flash_ClockConfig()   // SROM opcode 0x15，直接把 key 字写进 CPUSS_SYSARG（**不是指针**，与 LOAD_LATCH/BACKUP 不同）
  → WRITE_ROW(0x05)/WRITE_SFLASH_ROW(0x18)（SRAM指针参数，复用 LOAD_LATCH 那块 SRAM）
  → ProcessStatusCode
  → Cy_Flash_ClockRestore()  // SROM opcode 0x17，SRAM指针参数：word0=key, word1=同一个&backup_buf[6]
  → 出临界区
```
这段序列**没有任何 `#ifdef` 保护**（S8FS_VER2 的 ifdef 只影响写失败后的错误码细分，不影响是否调用 ClockBackup/Config/Restore）。即：**对本芯片所属的这条官方代码路径，CLK_BACKUP(0x16)→CLK_CONFIG(0x15)→写操作→CLK_RESTORE(0x17) 是无条件必需的**，不是"某些设备可选"。

现场证据链吻合方向：
- 火线记录显示，本工程此前尝试直接调用 SROM `CLK_CONFIG`(opcode 0x15，即 progspec 的 `SET_IMO_48MHz`——**两者是同一个 opcode，只是改了名字**，firmware 里 `CMD_SET_IMO_48MHZ=0x15` 与 PDL `CY_FLASH_API_OPCODE_CLK_CONFIG=0x15` 完全一致)，**未先调用 CLK_BACKUP(0x16)**，结果返回 `0xF0000014`(NA_IN_DEAD_MODE)。官方驱动证明：CLK_CONFIG **绝不单独调用**，永远紧跟在 CLK_BACKUP 成功之后。当前固件因为"CLK_CONFIG 直接调用报未定义错误码"就整体放弃 SROM 时钟调用、改走手工寄存器复刻（`configure_flash_clock()`），这个放弃的前提本身不成立——没有先做 CLK_BACKUP 就单独测 CLK_CONFIG，不能证明"CLK_CONFIG 对本芯片不适用"，更可能是"必须先 BACKUP 才能 CONFIG"这个顺序门控被跳过了。
- 决定性交叉证据：最新实机 `checksum_all()` 的**低 28 位真实校验和数值＝0**（不是状态码，是 SROM 对全片求和的真实计算结果）。这独立证实"擦除后+多次 PROGRAM_ROW/WRITE_ROW 报告成功后，flash 真实内容仍是全零"——即写操作被静默吞掉、从未真正落地到 flash 阵列，而不是"verify 读取环节出错"的假象。这与"手工寄存器时钟配置在肉眼可见的寄存器位域层面完全正确，但没有触发 SROM 内部只有真正执行 CLK_CONFIG 系统调用才会置位的隐藏门控（可能是 SPCIF 内部某个不对外暴露的'clock ready for write pulse'标志，或者 pump/bias 稳定判定），导致 WRITE_ROW/PROGRAM_ROW 的微代码走完全部逻辑分支、返回 0xA0000000成功，却在触发真实编程脈冲前被这个隐藏门控拦下"这一假设完全吻合。**此前"configure_flash_clock() 寄存器地址/位域/trim来源/写入顺序全部审计通过=确定不是根因"的结论需要收窄范围**：那次审计只验证了"手工复刻的寄存器动作在 TRM 层面是对的"，并未也不可能验证"SROM 是否存在只能由真实系统调用置位的内部隐藏状态"——这正是当前新排名第一的根因。

**根因排序（决定性证据强度从高到低）**：
1. **[最高，新增]** 跳过了官方无条件必需的 `CLK_BACKUP(0x16)→CLK_CONFIG(0x15)→...→CLK_RESTORE(0x17)` SROM 调用序列，改用手工寄存器复刻时钟配置；后者可能无法置位 SROM 内部专属于真实系统调用路径的隐藏门控，导致 WRITE_ROW/PROGRAM_ROW 报告成功但物理编程脈冲被静默跳过。checksum_all() 真实计算值=0 独立证实"写从未真正发生"，排除了"仅 verify 读取环节出问题"的可能。
2. **[中，新增，附属风险非独立根因]** `_srom_exec()` 用 `(SYSARG top nibble)==0xA` 判成功，与官方 `ProcessStatusCode()` 判据逐位一致（非 bug），但若某次 SYSREQ 因故未被 SROM 真正服务（如被隐藏门控直接短路），SYSARG 可能保留上一次系统调用的旧 0xA0000000，造成 WRITE_ROW/PROGRAM_ROW 的布尔返回值"假成功"。**checksum_all() 的 28-bit 真实数值已经独立证实闪存内容确实未写入，所以这不是本次故障的根本原因，只是一个可能同时存在、会让人误判"WRITE_ROW 已成功"的次要放大因素**，建议后续修复中一并加固（如每次系统调用前先写一个哨兵值到 SYSARG 之外的已知位置，或对比调用前后的 attempt 序号）。
3. **[低，维持既往结论]** SRAM_PARAMS_BASE=0x20001000 合法，非根因。
4. **[低，维持既往结论]** WRITE_ROW(0x05) vs PROGRAM_ROW(0x06) 选择不是根因——用户实测二者结果完全相同，恰好印证二者共享同一个更底层的门控失效点（即根因1），而不是这两个 opcode 本身有语义差异问题。

### 2) CLK_BACKUP(0x16)/CLK_CONFIG(0x15)/WRITE_ROW/CLK_RESTORE(0x17) 是否对外部 HSSP 必需 + 精确参数布局与顺序
**结论：对本芯片这条官方代码路径必需，无条件（Cy_Flash_WriteRow 里没有任何 #ifdef 包裹这三个调用）。**

精确参数布局（源自 `cy_flash.c` 逐行读出，未做任何推测）：
- **CLK_BACKUP (opcode 0x16)**：走 SRAM 指针间接（与 LOAD_LATCH 同类）。
  - `params[0]` = `KEY1 | (KEY2+0x16)<<8`（即 `0xB6 | ((0xD3+0x16)<<8)`）
  - `params[1]` = 一个**6 字（uint32_t[6]）SRAM 备份缓冲区的地址**（PDL 里是静态结构体 `cySysFlashBackup.clockSettings[6]`；本工程若要复现，需在 `SRAM_PARAMS_BASE` 之外另开一块 6 字缓冲区，或在参数块内紧跟 2 个 key 字之后预留 6 字，两块地址均需回传给 SROM）
  - `CPUSS_SYSARG` = 指向 `params[0]` 的**地址**（SRAM 指针，2 字：key字 + 备份缓冲区指针）
  - `CPUSS_SYSREQ` = `SYSREQ_BIT | 0x16`
  - 系统调用成功返回后，PDL 额外调用 `Cy_SysClk_ImoEnable()`——这是**普通寄存器写（非 SROM 系统调用）**，用于确保 IMO 振荡器已使能；本工程 `configure_flash_clock()` 里已有等价的 IMO 使能/升频动作，这部分可以复用，不需要重新发明。
- **CLK_CONFIG (opcode 0x15)**：**直接值**，不走 SRAM 指针（与 BACKUP/RESTORE/LOAD_LATCH 不同类，容易踩坑）！
  - `CPUSS_SYSARG` = `KEY1 | (KEY2+0x15)<<8`（**这个 32-bit 值本身直接写进 SYSARG，不是地址**）
  - `CPUSS_SYSREQ` = `SYSREQ_BIT | 0x15`
  - （PDL 对 `CY_IP_M0S8SRSSHV` 变体额外包了 `Cy_SysClk_UnlockProtReg()/LockProtReg()`；S455 用的是 `SRSSLT`(S8SRSS) 而非 M0S8SRSSHV，大概率不需要这层 unlock，但若 CLK_CONFIG 仍报错，可作为下一步排查项。）
- **WRITE_ROW (opcode 0x05，Flash 分支)**：走 SRAM 指针，复用 LOAD_LATCH 用的同一块参数区。
  - `parameters[0]` = `KEY1 | (KEY2+0x05)<<8 | (rowNum<<16)`（**16位行号直接左移16位打包，不是像 progspec 4.8.1 伪码那样拆 `(rowNum&0xFF)<<16 | (rowNum&0xFF00)<<16` 两次或运算**——PDL 版本更简洁，等效结果相同，因为 `rowNum<<16` 本就同时覆盖了低字节和高字节该去的位置，跟本工程 `program_row()` 现有的两次 `((row&0xFF)<<16)|((row&0xFF00)<<16)` 写法数值上等价，不是问题）
  - `CPUSS_SYSARG` = 指向 `parameters[0]` 的地址
  - `CPUSS_SYSREQ` = `SYSREQ_BIT | 0x05`
- **CLK_RESTORE (opcode 0x17)**：走 SRAM 指针，布局与 CLK_BACKUP 完全对称。
  - `params[0]` = `KEY1 | (KEY2+0x17)<<8`
  - `params[1]` = **同一个**备份缓冲区地址（CLK_BACKUP 时保存到哪，CLK_RESTORE 就必须从同一处读回）
  - `CPUSS_SYSARG` = 指向 `params[0]` 的地址
  - `CPUSS_SYSREQ` = `SYSREQ_BIT | 0x17`

**顺序强约束**：LOAD_LATCH → (临界区内)ClockBackup → ClockConfig → WRITE_ROW/PROGRAM_ROW → ClockRestore →(出临界区)。ClockConfig 绝不能脱离 ClockBackup 单独调用（此前固件正是这样单独测了 CLK_CONFIG 才报 0xF0000014，不能就此断定该芯片不支持 CLK_CONFIG）。

### 3) `_srom_exec`/`_read_io` 是否可能读到陈旧成功码造成假成功
**结论：机制上存在这个风险（见根因排序第2条），但本次故障不是由它单独造成，checksum_all() 的真实 28-bit 数值=0 已独立证实 flash 内容确实未写入，不是"读到陈旧码而已"。**
- `_read_io()` 的"写 TAR→读 DRW 丢弃第一拍→读 DRW 取真值"模式，是 AHB-AP 标准的管线延迟处理手法，实现正确，不引入陈旧值问题（每次 `_read_io` 内部自洽）。
- `_srom_exec()` 的成功判据 `(code & 0xF0000000) == 0xA0000000` 与官方 `ProcessStatusCode()` 的 `(statuscode & SROMCODE_STATUS_MASK) == SROMCODE_SUCCESS` 逐位相同,不是本工程自造的宽松判据,是 Infineon 官方定义。
- 潜在陈旧风险点：若某次 `SYSREQ_BIT|cmd` 写入后，SROM 因某种门控直接忽略/短路该请求（例如根因1所述的隐藏门控），`SYSREQ_BIT`/`PRIVILEGED_BIT` 可能很快甚至立即回落到0（因为SROM从未真正置位它们，或极快处理并原地返回），而 `CPUSS_SYSARG` 却从未被这次调用重新写入，仍残留上一次调用（如 LOAD_LATCH 的成功码）——此时 `_srom_exec` 会误判"这次也成功"。这个假成功机制**理论成立**，但**不是本次 WRITE_ROW/PROGRAM_ROW 报告成功而 flash 未写的唯一或主要解释**，因为 checksum_all() 是一次独立的系统调用，其返回的 28-bit 数值不是"成功/失败状态码"而是 SROM 内部对全片求和的真实计算结果——如果 flash 真的被写入过（即便只是部分行），这个和值不会恰好精确等于0（除非写入的整份 25984 字节镜像巧合全部按字节求和为0，概率极低，且该镜像开头是有效的 ARM 中断向量表，含大量非零字节，见 `psoc_fw_image.h` 前几行 `0x00,0x40,0x00,0x20,0xE9,0x05,...`，明显非全零）。所以：CHECKSUM 独立确认了"写真的没发生"，而不只是"我们读到了假成功状态码"。

### 4) 最小安全下一步（禁止 ERASE/Flash，仅只读诊断）
建议按顺序做，且都不接触 flash 内容，出问题可 XRES 复位恢复：
1. **单独补做 CLK_BACKUP(0x16)** 系统调用（只读性质：只把当前5个SRSS时钟寄存器的值搬进一块新开的SRAM备份区，不改任何flash内容），记录其返回状态码。这是此前从未按官方顺序尝试过的调用。
2. **紧接着（同一次 acquire 会话内、不插入 ERASE）尝试 CLK_CONFIG(0x15)**，记录状态码。这是关键验证点：如果这次紧跟 BACKUP 之后调用 CLK_CONFIG 成功（0xA0000000），直接证实根因1（"必须先 BACKUP 才能 CONFIG"）；如果仍报 0xF0000014 或其它错误，说明还有别的前置条件缺失，需要继续排查（如 SRSSLT 变体是否需要额外 unlock 寄存器，或 acquire() 建立的 test-mode 状态本身有细节偏差）。
3. **调用 CLK_RESTORE(0x17)** 收尾（恢复时钟到备份状态，同样不碰 flash）。
4. 全程只新增/记录：CLK_BACKUP 状态码、CLK_CONFIG（紧跟 BACKUP 之后）状态码、CLK_RESTORE 状态码，以及现有的 `_clock_select/_clock_imo_select/...` 回读（可继续复用现有诊断字段，不需要新开协议字段就能先用 CDC 打印验证思路）。
5. **明确不要做**：不要在本次诊断中调用 ERASE_ALL/WRITE_ROW/PROGRAM_ROW/LOAD_LATCH，即使 CLK_CONFIG 成功也先如实上报结果，等待人工确认后才进入下一阶段"用官方顺序重做一次真实编程"的改动（那将是代码修改，超出本次纯检索任务范围）。

本次为纯本地代码/文档审计，未修改任何源码，未执行任何烧录/擦除/编程操作。
