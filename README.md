# mai2control v4

> 仓库：<https://github.com/NKQC/project-mai2control.git>
> design huhuzhu

## 免责声明

该软件开发目的仅供个人学习交流，使用该软件造成的任何法律责任与开发者无关。

如需商业使用或任何其他操作，详见本仓库附带的 LICENSE 条款。

仓库所有者保留修改使用条款的权利。

## 项目概述

mai2control v4 由 PSoC 传感前端、RP2040 主控和 Rust 上位机组成：

- **PSoC 4100S Plus（CY8C4147AZI）**：在 `psoc_firmware/CY8C4147AZI-SensorCore/` 运行 CapSense，完成 36 个电容触控通道扫描、CSD 参数和 IDAC 校准；作为 SPI 从机向主控提供触控状态及快照，并提供可加载的触控算法执行槽。
- **RP2040**：在 `main_firmware/` 运行，负责 PSoC 的 PIO-SPI 链路、PSoC SWD 烧录/恢复、LittleFS 配置持久化、游戏/灯光协议、WS2812 映射及 USB 设备功能。USB 配置通道为 WinUSB vendor；串口模式提供两个 CDC（mai2serial、mai2light），HID 模式提供一个复用触摸屏与键盘 Report ID 的 HID 接口。
- **Rust 上位机**：`control_software/` 中的 `mai2control-ui` 使用 Slint 提供配置、遥测和算法工具；`selftest` 是无头的 WinUSB 诊断/自测程序，并包含虚拟扫码摄像头相关组件。

链路为：PSoC 通过 SPI 与 RP2040 交换触控与控制数据；上位机经 USB WinUSB 配置接口与 RP2040 通信；RP2040 通过 SWD 对 PSoC 编程。RP2040 固件内嵌 PSoC 固件映像，启动时会比较内嵌版本与传感器版本；版本不一致时会经 SWD 自动重刷 PSoC。

## 目录结构

```text
.
├─ main_firmware/                         RP2040 固件（PlatformIO / Arduino）
│  ├─ src/                                HAL、协议和服务层
│  ├─ tools/psoc_hex_to_c.py              将 PSoC HEX 嵌入 RP2040 头文件
│  └─ build.ps1                           PSoC→嵌入→RP2040→Rust 的固定构建流程
├─ psoc_firmware/
│  ├─ CY8C4147AZI-SensorCore/             PSoC 4100S Plus / ModusToolbox 工程
│  └─ algo/                               JIT 触控算法 blob 源码
├─ control_software/                      Rust 上位机工作区
│  ├─ src/                                WinUSB、状态、协议及 selftest
│  ├─ ui/                                 Slint 入口、通用组件和页面
│  ├─ vcam_source/                        虚拟摄像头 Rust 组件
│  └─ vcam_source_cpp/                    Media Foundation 虚拟摄像头 DLL
├─ .vscode/                               编辑器配置
├─ .kiro/                                 Kiro 工作区元数据
├─ .pio/                                  PlatformIO 构建产物
├─ hardware.txt                           硬件引脚与触控通道对照
├─ dev.ps1                                固定开发、构建、烧录与诊断入口
└─ LICENSE                                许可条款
```

Slint UI 入口为 `control_software/ui/app.slint`，通用定义位于 `ui/common/{types,theme,widgets}.slint`；页面位于 `ui/pages/`：`dashboard`、`settings`、`binding`、`protocol`、`all_channels`、`global_tune`、`curves`、`keymap`、`config`、`algo`、`toolbox`、`log`。

## 构建、嵌入与烧录

### 环境依赖

- **PSoC**：ModusToolbox 3.6。固定脚本使用 `%USERPROFILE%\ModusToolbox\tools_3.6\modus-shell\bin\make.exe`；`dev.ps1 build-psoc` 的当前实现使用 `C:\Users\asdfg\ModusToolbox\tools_3.6\modus-shell\bin\bash.exe`。
- **RP2040**：PlatformIO 与 Arduino-Pico/Earle Philhower 框架。工程目标为 `pico`，主频 133 MHz。
- **上位机**：Rust/Cargo；使用仓库锁定依赖构建。
- **算法 blob**：`arm-none-eabi-gcc`、`objcopy`、`nm`、`objdump` 用于将 C 算法编译成 Cortex-M0+ Thumb 机器码并检查产物。脚本会从 ModusToolbox 或 PlatformIO 的工具链目录中查找这些工具。
- **嵌入步骤**：Python 用于执行 `main_firmware/tools/psoc_hex_to_c.py`。

在仓库根目录执行以下命令：

```powershell
# PSoC（也可在 ModusToolbox shell 内进入 PSoC 工程后执行 make build -j8）
powershell -ExecutionPolicy Bypass -File .\dev.ps1 build-psoc

# RP2040
powershell -ExecutionPolicy Bypass -File .\dev.ps1 build-rp

# 上位机与 selftest
powershell -ExecutionPolicy Bypass -File .\dev.ps1 build-ui

# 完整构建：PSoC make → HEX 嵌入 → PlatformIO UF2 → Rust UI/selftest
powershell -ExecutionPolicy Bypass -File .\dev.ps1 build-all

# 构建默认 JIT 算法 blob
powershell -ExecutionPolicy Bypass -File .\dev.ps1 build-blob
```

`main_firmware/build.ps1` 是完整固定流程的实现：先构建 PSoC，再将唯一生成的 HEX 转为 `main_firmware/src/protocol/psoc/psoc_fw_image.h`，随后构建 RP2040 UF2 以及 Rust UI/selftest。根目录 `dev.ps1` 为日常入口，除构建外还提供 BOOTSEL、`flash-rp`、`reflash`、`cycle`、`diagnose` 等工作流；涉及实际设备写入的操作仅应在确认目标设备后执行。

## 功能特性

### 触控与算法

- 36 通道 CapSense/CSD 扫描，支持通道阈值、噪声阈值、基线、IDAC、频率等参数调节与频率自适应；PSoC 侧进行校准并将快照回传。
- JIT 触控算法引擎：PSoC 从 1 KiB 可执行 SRAM 槽（Thumb 入口 `algo_slot | 1`）执行上传的算法 blob。上传协议为 `ALGO_BEGIN`、`ALGO_PAGE`、`ALGO_END`、`ALGO_INFO`，提交前校验 CRC16；运行中还可使用 `ALGO_SET_ROM`/`ALGO_GET_ROM`、`ALGO_GET_TRACE`、`ALGO_SET_CFG`/`ALGO_GET_CFG`。
- 算法 ABI v1 的 `algo_io_t` 固定为 128 字节：输入包含 `baseline`、`diff`、`raw`、噪声/触发阈值、`now_ms`、通道号和 `cfg[8]`；`state[64]` 为持久状态；输出包含 `out_active`、`report[4]` 与 `out_led`，并提供每通道只读 `rom`。
- `ALGO_REPORT(idx, name)` 与 `ALGO_SETTING(idx, name, defval)` 是算法 C 源码中的声明宏，会展开为空；上位机解析源码中的名称和默认值，以生成 `report[]` 可视化及 `cfg[]` 设置项，不是独立的 SPI 帧。
- 上位机提供遥测展示和单通道精调时间轴；为避免 WinUSB vendor IN 高吞吐导致不稳定，遥测在主机侧按 30 Hz 管理。

### 协议、灯光与输入

- 支持 mai2serial 与 mai2light CDC 协议。
- PIO 驱动 WS2812，并支持将协议中的虚拟灯光单元映射到多个 NeoPixel 地址。
- 支持触控区域到 HID 键盘的映射，以及 GPIO1–GPIO12 的物理按键到键盘映射。
- 支持虚拟扫码摄像头：Rust 组件配合 Media Foundation 媒体源 DLL 提供设备端能力。
- 配置通道支持 WinUSB 诊断、设备信息和配置交互；固件包含看门狗/BOOTSEL 恢复路径与 PSoC 自动重刷机制。

### `selftest` 无头诊断

`control_software/src/bin/selftest.rs` 构建为 `selftest`。第一个非 `--` 参数可指定串口；常用选项包括：

```text
--diagnose                 WinUSB DEVICE_INFO bring-up 诊断
--smoke                    flash/link/snapshot/silicon 严格检查
--algo                     JIT 算法上传、校验与恢复流程
--global                   全局 CSD 配置读写校验
--kbd                      物理键码和触控映射读写校验
--led                      灯光相关测试
--soak                     持续压力测试
--soak-idle N              空闲时间参数
--soak-rate Hz             压力测试频率参数
--soak-fields 0xNN         压力测试字段掩码
--soak-seconds N           压力测试时长
--csd-provision            写入 CSD 配置
--csd-verify               校验 CSD 配置
--reboot-bootloader        请求进入 RP2040 BOOTSEL
--reboot-app               请求回到应用
--list-only                仅列出设备
--reset-config             重置配置
```

另有 `--vcam-probe`、`--debug-read`、`--cfg-only`、`--telem-only`、`--idle-only`、`--param-dump`、`--algo-dump` 等诊断选项；以 `selftest --help` 和源码实际参数为准。

## 硬件与协议要点

- RP2040 与 PSoC 间采用经电平转换器的 SPI：RP2040 GPIO26 为 SCK、GPIO29 为 CS；由于板上 MOSI/MISO 连线与 RP2040 硬件 SPI1 固定 TX/RX 相反，实际由 PIO 自定义主机使用 GPIO28 输出到 PSoC P1.0、GPIO27 从 PSoC P1.1 输入，PSoC P1.2 为 SCK、P1.3 为 CS。当前 PIO SPI 时钟配置为 3 MHz。
- RP2040→PSoC SWD 使用 GPIO16（DAT）、GPIO17（CLK）和 GPIO21（RST）；PSoC 自身 SWD 为 P3.2（DAT）与 P3.3（CLK）。`SWD_RELEASE_TO_EXTERNAL` 设为真时，RP2040 会让出该链路供外部 DAP-LINK 使用。
- CapSense 共 36 个通道，分为 PA、PB、PC、PD 四组；P4.1 为 CSD SHIELD，P4.2 为 2.2 nF CMOD。完整引脚/通道对照以 `hardware.txt` 为准。
- USB 设备 VID:PID 为 `2E8A:000A`。TinyUSB 配置中 vendor、CDC、HID 实例数分别为 1、2、1；WinUSB vendor RX/TX FIFO 均固定为 64 B。该限制来自已验证的端点行为：增大 vendor TX FIFO 会导致写入数据无法送至 IN 端点，因此高吞吐通过上层队列和降频管理，而不是扩大 FIFO。

## 相关源码入口

- RP2040 配置：`main_firmware/platformio.ini`、`main_firmware/src/config.h`、`main_firmware/src/main.cpp`
- USB 配置：`main_firmware/src/hal/usb/tusb_config.h`
- PSoC 主程序与 ABI：`psoc_firmware/CY8C4147AZI-SensorCore/main.c`、`psoc_firmware/CY8C4147AZI-SensorCore/psoc_algo_abi.h`
- 上位机：`control_software/Cargo.toml`、`control_software/src/main.rs`、`control_software/src/bin/selftest.rs`
