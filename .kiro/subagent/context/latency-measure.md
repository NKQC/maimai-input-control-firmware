# latency-measure

## 任务目标
严格按规格实现 RP2040 触控输出流水线 SPI、处理、USB 耗时的滚动最大值遥测，并在控制软件解码、保存和 UI 测量开关启用时展示。

## 已完成步骤
- 已检查 `.kiro/subagent/context/latency-measure.md`：开工前不存在，未需恢复。
- 已批量核对固件遥测布局、PSoC 触控快路、游戏 I/O 快路、Rust 解码/AppController/UI 回调与属性。
- 已新建 `main_firmware/src/service/latency_stats.h`，包含规格要求的 extern 统计值与饱和滚动最大值内联函数。
- 固件：已添加 `TELEM_FIELD_LATENCY = 0x20`，并在 `sensor_link.cpp` 定义三个全局统计值；`tick()` 严格在 STATS 后、通道前按 spi/proc/usb 小端写入 6B 并清零。
- 固件：已在 `Psoc::update()` 触控 SPI 耗时记录后记录 SPI 峰值；已在 `GameIoService::task()` 精确包围 mask→区域→Mai2 帧构建及 `send_touch_data`，记录处理与 USB 峰值。
- 主机协议：已加入 `FIELD_LATENCY` 和 re-export；`TelemFrame` 增加三个 u16；解码器在 STATS 后、通道前解码 LATENCY，未设置该位时三个值均为零（STATS-only 兼容）；构造函数已带入三字段。
- AppController：已保存三个延迟字段、构造时清零、遥测启动时 OR `FIELD_STATS | FIELD_LATENCY`、收到帧时回填，并已提供三个 u16 getter。
- UI：已使用 `Rc<Cell<bool>>` 将现有测量开关改为 `measure_on` 显示门控；定时器仅在开关开启时回填 SPI/处理/USB/传感器/总计延迟。

## 改动锚点
- `main_firmware/src/service/latency_stats.h`
- `main_firmware/src/service/sensor_link/sensor_link.h`: TELEM_FIELD 常量区
- `main_firmware/src/service/sensor_link/sensor_link.cpp`: 顶部统计值定义；`SensorLink::tick()` STATS 后
- `main_firmware/src/protocol/psoc/psoc.cpp`: include 区；`Psoc::update()` `_touch_read_us` 后
- `main_firmware/src/service/game_io/game_io.cpp`: include 区；`GameIoService::task()` 触控快路
- `control_software/src/proto/telemetry.rs`: FIELD 常量、TelemFrame、decode STATS 后、TelemFrame 构造
- `control_software/src/proto/mod.rs`: telemetry re-export 区
- `control_software/src/app_state/mod.rs`: telemetry state/new/start/receive/getters
- `control_software/src/main.rs`: latency toggle、timer capture、timer UI 回填

## 验证结果
- `powershell -ExecutionPolicy Bypass -File F:\mai2control\mai2control-v4\dev.ps1 build-rp`：通过，输出 `[SUCCESS]`，退出码 0。
- `powershell -ExecutionPolicy Bypass -File F:\mai2control\mai2control-v4\dev.ps1 build-ui`：通过，Cargo 输出 `Finished dev profile`，退出码 0；PowerShell 的 `NativeCommandError` 文本来自脚本的 `2>&1 | Select-Object`，无 Cargo 编译错误。
- `cargo test --lib`（`control_software`）：通过，70 passed / 0 failed，退出码 0。

## 关键决定
- `g_lat_*` 定义为全局外部链接变量，匹配头文件 `extern` 声明；不放入匿名命名空间。
- 遥测帧扩展严格为 `ts_us,ch_count,fields,[STATS 8B],[LATENCY 6B],channels`。
- 现有 Slint 已声明延迟回调和六项属性，未修改 UI 文件。
- `TelemFrame` 没有其他字面量构造点；现有测试用解码器构造帧，不需构造处补零。

## 异常报告
- 非规格要求的最终 `git diff --check` 退出码 1，报告的是工作区既有 `hardware.txt`、`main_firmware/src/service/usb_comm/usb_comm.h` 和构建产物 `main_firmware/.pio/build/pico/firmware*.lst` 的尾随空格（另有既有 CRLF 警告）。未涉及本任务新增/修改的源代码逻辑，且未作任何猜测式修复；按“失败即停”要求不再继续操作。

## 待办
无代码待办。若需要干净的 `git diff --check`，应由相关既有文件/构建产物的维护方单独处理其尾随空格。
