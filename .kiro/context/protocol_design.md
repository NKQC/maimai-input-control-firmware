# mai2control-v4 通信协议设计（task #4 交付物）

> 主 agent 设计文档。基于三份调研:`recon_v4_firmware.md`(v4 固件)、`v3_ui_parity_baseline.md`(v3.0 UI parity)、`psoc_milestone3_spi.md`(PSoC SPI 现状)。
> 面向:Rust 上位机 UI ⇄ RP2040 ⇄ PSoC4 CapSense。目标——极致触控灵敏度/稳定性调参 + 高带宽实时可视化。

---

## 0. 总体拓扑与两段链路

```
[Rust 上位机 UI] --USB CDC(二进制帧)--> [RP2040] --PIO-SPI(扩展帧)--> [PSoC4 CapSense]
        <--------- 配置读写/遥测流 ----------      <----- 遥测快照/参数写 -----
```

- **链路 A(host ⇄ RP2040)**:走现有 USB **CDC** 通道(recon 确认 CDC 空闲、单向文本诊断可改造;不新增 USB interface,避免改描述符/驱动兼容问题)。
- **链路 B(RP2040 ⇄ PSoC)**:扩展现有 **PIO-SPI**(当前仅 PING/PONG 的 7 字节 `psoc::Frame`)。
- **两段解耦**:host 侧协议参数无关;RP2040 做"翻译/路由"层(`service/sensor_link/` 空目录=天然挂载点),把 host 命令翻译成 SPI 事务,把 SPI 遥测快照翻译成 host 遥测帧。host 永远不直接感知 SPI 细节。

---

## 1. 链路 A:host ⇄ RP2040(USB CDC 二进制帧)

### 1.1 帧格式(双向通用)

CDC 是字节流,需自带分帧。定长头 + 变长载荷 + CRC:

```
偏移  字段      类型      说明
0     SOF0      u8=0xAA   帧起始魔数
1     SOF1      u8=0x55
2     cmd       u8        命令/响应码(见 1.3)
3     flags     u8        bit0=1 响应(ACK/NAK); bit1=1 流数据帧; bit2=1 NAK(错误); 其余保留
4     seq       u8        请求序号,响应回填同值(流帧自增)
5     len       u16 LE    payload 字节数(0..65535,实际受重组缓冲限制)
7     payload   u8[len]
7+len crc16     u16 LE    CRC-16/CCITT-FALSE,覆盖 cmd..payload 末字节
```

- **重组**:host 与 RP2040 各维护一个接收状态机(找 SOF → 读头 → 收 len 字节 → 校验 CRC)。参考 `Mai2Serial::process_dma_received_data` 的滑窗解析范式。
- **单帧上限**:建议 payload ≤ 4096;超大遥测用"分块流"(见 1.5),不做单巨帧。
- CDC OUT/IN 端点包 64 字节,帧可跨多包,由重组层拼接。

### 1.2 版本与能力协商

首帧必须 `HELLO`。RP2040 回 `DEVICE_INFO`:协议版本、fw 版本、CapSense 通道数(=36)、能力位(是否支持遥测流/参数直调/绑区/灯效/校准)。UI 按能力位裁剪界面。

### 1.3 命令码(cmd)分区

| 范围 | 域 | 命令 |
|---|---|---|
| 0x01-0x0F | 系统 | 0x01 HELLO / 0x02 DEVICE_INFO(resp) / 0x03 PING / 0x0E SAVE_CONFIG / 0x0F RESET_DEFAULTS |
| 0x10-0x1F | 配置KV | 0x10 CFG_GET(key) / 0x11 CFG_SET(key,val) / 0x12 CFG_GET_GROUP(prefix) / 0x13 CFG_GET_ALL / 0x14 CFG_SET_BATCH |
| 0x20-0x2F | CapSense调参 | 0x20 PARAM_GET(ch,param_id) / 0x21 PARAM_SET(ch,param_id,val) / 0x22 PARAM_GET_ALL(ch) / 0x23 CALIBRATE(ch_mask) / 0x24 BASELINE_RESET(ch_mask) |
| 0x30-0x3F | 遥测流 | 0x30 TELEM_START(mode,rate_hz,ch_mask,fields) / 0x31 TELEM_STOP / 0x32 TELEM_DATA(流帧,dev→host) |
| 0x40-0x4F | 绑区 | 0x40 BIND_START / 0x41 BIND_ABORT / 0x42 BIND_CONFIRM / 0x43 BIND_GET_MAP / 0x44 BIND_SET_MAP / 0x45 BIND_EVENT(进度,dev→host) |
| 0x50-0x5F | 灯效 | 0x50 LED_GET / 0x51 LED_SET_REGION / 0x52 LED_PREVIEW |
| 0x7E/0x7F | 应答 | 0x7E ACK / 0x7F NAK(payload:err_code+可读msg) |

### 1.4 配置 KV 模型(复用 ConfigManager,新建 key 命名空间)

值编码沿用 `ConfigValue`:payload = `type(u8) + key_len(u8) + key + value(按type)`。命名空间(前缀便于 `CFG_GET_GROUP`):

- `touch.ch{NN}.<param>` — 逐通道 CapSense 参数(见链路B param_id)。
- `touch.zone.<A1..E8>.<param>` — 逻辑区(经绑区映射到通道)。
- `bind.map{NN}` — 34 区→通道映射(u32:高8位设备掩码+低24位通道bitmap,沿用 v3.0 `serial_mappings` 语义)。
- `comm.*` — 采样延迟/仅变化发送/聚合延迟/额外发送/频率限制(接替 v3.0 communication_settings 9 项;波特率项按 v4 实际是否走串口取舍)。
- `mode.work` — Serial/HID 工作模式(v4 新增运行时切换能力)。
- `ui.*` — 若保留本机屏:息屏超时/亮度(否则纯 host 侧)。
- `led.region{NN}.*` — 灯区映射+颜色(补齐 v3.0 本机 UI 都没有的可视化编辑)。

> 落盘:`CFG_SET` 只改内存 runtime_map;`SAVE_CONFIG` 才触发 `ConfigManager::save_config()`(异步 flash 写,multicore_lockout 保护)。UI 需有明确"保存"动作(对齐 v3.0"保存设置"按钮语义),避免高频写 flash。

### 1.5 遥测流(高带宽实时可视化核心)

- `TELEM_START` 参数:`mode`(连续/单次)、`rate_hz`(目标上报率)、`ch_mask`(u64,选通道)、`fields`(位:RAW|BASELINE|DIFF|STATUS)。
- RP2040 进入流模式后,按 rate 从 PSoC 拉遥测快照(链路B),打包成 `TELEM_DATA` 流帧(flags.bit1=1)推给 host。
- `TELEM_DATA` payload:`ts(u32 μs) + ch_count(u8) + [每通道: 按 fields 选择 raw(u16),bsln(u16),diff(i16),status(打包bit)]`。
- **带宽约束(recon C.4)**:36ch×(raw+bsln+diff=6B)=216B/帧。当前 1MHz 无 DMA PIO-SPI 实测吞吐不足以 1kHz 全量。策略:
  1. UI 默认只订阅"当前关注的少量通道"高速率 + 全通道低速率概览。
  2. 提升 SPI 时钟(PSoC SCB 从机上限待确认)+ PIO-SPI DMA 化(复用 `hal_uart` 的 DMA 环形缓冲范式)。
  3. RP2040 侧可做"仅变化/降采样"过滤。
- 曲线绘制在 UI 侧做环形缓冲 + 时间轴滚动;阈值线(fingerTh/noiseTh)叠加在 diff 曲线上做交互式拖拽调参(拖动即发 `PARAM_SET`,可选实时预览/确认落盘)。

---

## 2. 链路 B:RP2040 ⇄ PSoC(扩展 PIO-SPI)

### 2.1 现状与约束(psoc_milestone3_spi.md)

- SCB0 SPI 从机,MODE0/8bit/MSB/CS-low;RP2040 PIO1 主机,1MHz。
- 当前 `psoc::Frame` 7B(magic 0xA5 + cmd + seq + payload[4]),仅 PING(0x01)/PONG(0x02),FW_VERSION 0x00000400。
- **SPI 从机特性**:PSoC 预装 TX FIFO,主机 clock 时同步读回 → 请求与响应天然**流水线错位**(主机第 N 事务发请求,第 N+1 事务读到响应)。协议须按此设计。

### 2.2 帧扩展方案:定长大帧 + 命令头

废弃 4 字节 payload 定长小帧,改为**定长 64 字节事务帧**(权衡:够放遥测分块,又不至太大;PSoC FIFO/DMA 友好):

```
偏移  字段     说明
0     magic    u8=0xA5
1     cmd      u8  (见 2.3)
2     seq      u8
3     arg      u8  (通道号/param_id/分块索引等)
4     len      u8  (本帧有效数据长度 0..58)
5..62 data     u8[58]
63    xor      u8  (校验:cmd..data 异或)
```

- 主机每次全双工收发 64B。**当拍读到的是"上一拍请求"的响应**(流水线错位),`seq`/`cmd` 回显用于对齐。
- 也兼容旧 PING/PONG(cmd 复用 0x01/0x02,len=4 放 FW_VERSION)。

### 2.3 SPI 命令集(RP2040 主机 → PSoC 从机)

| cmd | 含义 | 请求 data | 响应 data(下一拍) |
|---|---|---|---|
| 0x01 PING | 心跳 | seq | 0x02 PONG + FW_VERSION |
| 0x10 GET_TELEM | 读遥测快照分块 | arg=分块索引 | 该分块的逐通道 {raw,bsln,diff,status} |
| 0x11 GET_TELEM_META | 通道数/字段布局/快照序号 | — | ch_count, field_layout, snapshot_seq |
| 0x20 GET_PARAM | 读某通道某参数 | arg=param_id, data[0]=ch | value(4B) |
| 0x21 SET_PARAM | 写某通道某参数 | arg=param_id, data[0]=ch, data[1..4]=value | ack+生效状态 |
| 0x23 CALIBRATE | 触发 SmartSense 重调/重扫 | data=ch_mask | 进度/状态 |
| 0x24 BASELINE_RESET | 重置基线 | data=ch_mask | ack |

### 2.4 PSoC 侧遥测快照机制(推荐)

- PSoC 每次 CapSense 扫描后,把 36 通道的 raw/baseline/diff/status 写进一份**双缓冲快照**(扫描线程写、SPI ISR 读,避免撕裂)。
- `GET_TELEM` 按分块索引返回(58B/帧 → 36ch×6B=216B 需约 4 分块;或按 fields 精简)。
- **参数无关**:host/RP2040 用抽象 `param_id` 命名空间寻址 CapSense 参数;PSoC 侧 `SET_PARAM/GET_PARAM` 内部把 param_id 映射到具体 CapSense 中间件字段(fingerTh/noiseTh/nNoiseTh/hysteresis/onDebounce/lowBslnRst/resolution/snsClk/idac 等)。**确切字段名/范围在 PSoC 侧实现时对齐 cycfg_capsense/中间件结构**(此处不写死,避免暗猜)。

### 2.5 param_id 抽象命名空间(host↔RP2040↔PSoC 三段一致)

| param_id | 名称 | 类型 | 说明(受 SmartSense 支配的项在自动模式下只读) |
|---|---|---|---|
| 0x01 | FINGER_TH | u16 | 手指检测阈值(diff 超过即触摸) |
| 0x02 | NOISE_TH | u16 | 噪声阈值 |
| 0x03 | NEG_NOISE_TH | u16 | 负噪声阈值 |
| 0x04 | HYSTERESIS | u16 | 迟滞 |
| 0x05 | ON_DEBOUNCE | u8 | 触发去抖计数 |
| 0x06 | LOW_BSLN_RST | u8 | 低基线复位 |
| 0x07 | RESOLUTION | u16 | 扫描分辨率 |
| 0x08 | SNS_CLK_DIV | u16 | 感应时钟分频 |
| 0x09 | IDAC_MOD | u8 | 调制 IDAC(SmartSense 自动) |
| 0x0A | IDAC_COMP | u8 | 补偿 IDAC(SmartSense 自动) |
| 0x80 | SMARTSENSE_EN | u8 | SmartSense 自动调参开关(全局/分区) |

> 以上为设计占位命名;实现时逐一对齐 PSoC CapSense 中间件真实字段与范围,不一致处以中间件为准。

---

## 3. 功能对齐(v3.0 parity)映射到本协议

| v3.0 能力 | v4 实现 |
|---|---|
| 灵敏度三层输入维度(分区批量/区域直写/通道直写) | UI 三种视图,底层统一走 `PARAM_SET`(逐通道)/绑区反查;抽象数值收敛为 CapSense 真实参数 |
| 触摸校准(AD7147 AEF) | `CALIBRATE`/`BASELINE_RESET` + SmartSense 自动调参 + 进度事件 |
| 34 区绑区(交互式) | `BIND_START/EVENT/CONFIRM`,UI 引导依次触摸,结果存 `bind.map{NN}` |
| 绑定信息只读展示 | `BIND_GET_MAP` |
| 通信设置 9 项 | `comm.*` 配置 KV(波特率按 v4 是否走街机串口取舍) |
| 通用设置(息屏/亮度) | `ui.*`(若保留本机屏) |
| 灯效区域-颜色映射 | `led.*` + UI 可视化编辑器(v3.0 本机 UI 缺失,v4 补齐) |
| 工作模式 Serial/HID | `mode.work` 运行时切换(v4 新增) |
| **触控实时可视化/曲线** | **v4 全新**:`TELEM_*` 流 + UI CSD 曲线 + 阈值交互拖拽 |

---

## 4. 待确认/实现期对齐项(不阻塞架构,标注以免暗猜)

1. **CapSense 真实参数字段名/范围/单位** + SmartSense 自动 vs 手调边界 → PSoC 实现期读 cycfg_capsense/中间件确认(原计划 recon_psoc_capsense_params,被中止;可实现时就地对齐)。
2. **36 通道 ↔ 34 分区映射**:确认物理 widget 编号与 maimai A1-E8 对应(推测映射表放 RP2040 绑区配置,PSoC 只按物理通道号)。
3. **SPI 从机时钟上限 + 是否需要 PIO-SPI DMA 化**:决定遥测最高采样率;需实测。
4. **CDC 单通道能否同时承载配置命令+高频遥测**:若抢占严重,再评估第二 CDC/vendor interface(改描述符,较重)。
5. **Rust UI 框架**:建议 `egui`/`eframe`(即时模式、绘图曲线方便、跨平台、无需 web 后端);USB CDC 走 `serialport` crate。控制反转:UI 线程 + 独立 IO 线程(串口读写)+ 通道(crossbeam/std mpsc)解耦。

---

## 5. 实现拆分建议(后续 task #5/#6,交 module-builder,每个都要够小)

- #5a RP2040:CDC 二进制帧编解码层(`service/usb_comm/` 或新 `protocol/host_cmd/`)——分帧/CRC/命令分发。
- #5b RP2040:配置 schema 注册 + `CFG_*` 命令接 ConfigManager;`SAVE/RESET`。
- #5c RP2040:`service/sensor_link/` 遥测桥 + SPI 帧扩展(`psoc_spi` 加 64B 事务 + 新命令);`TELEM_*`/`PARAM_*`/`CALIBRATE` 路由。
- #5d PSoC:SPI 从机帧扩展 + 遥测双缓冲快照 + param_id↔CapSense 字段映射 + 校准/基线命令。
- #6a Rust:工程 scaffold(eframe)+ CDC IO 线程 + 协议编解码(与 #5a 对齐)。
- #6b Rust:配置页(接替 v3.0 全部项)。
- #6c Rust:绑区可视化(34 区图形化 + 交互绑定)。
- #6d Rust:CSD 实时曲线 + 阈值交互式拖拽调参。
- #7 双侧编译 + 协议帧收发自测(host↔RP2040 先用 PSoC 未接时的桩数据联调)。
```
```


---

# 修订 1(用户定稿指令,2026-07-15)

以下 4 条为用户明确决策,**优先级高于前文默认假设**,冲突处以此为准。

## R1. V4 移除 OLED,完全由上位机负责显示与设置
- 砍掉 `ui.*`(息屏/亮度)配置与本机菜单相关一切。设备无本地 UI,所有配置/可视化在 PC 上位机。
- ConfigManager 侧不再注册 `ui.*`。

## R2. 先做 Rust 上位机;UI 框架 = Slint(非 egui)
- 顺序:**Rust UI + 协议侧优先**;PSoC 侧(#5d)与固件多 CDC 改造随后(等新片/按需)。
- 框架:**Slint**(声明式 .slint + Rust 后端;为后续复杂交互/可视化铺路)。需 Rust ≥1.92,build.rs 用 `slint_build::compile`。串口用 `serialport` crate;Windows 系统操作用 `windows` crate(SetupAPI/注册表)。
- 线程模型:UI 线程(Slint 事件循环)+ 独立 IO 线程(串口读写)+ mpsc/channel 解耦。

## R3. CDC 配置通道必须独立枚举,不与 mai2serial / mai2light 混用
- USB 改为**多 CDC 复合设备**:
  - CDC-A = **mai2serial**(触摸数据→街机/游戏)
  - CDC-B = **mai2light**(灯效←游戏)
  - CDC-C = **config**(本文件"链路A"host 配置/遥测协议,独立)
  - + HID(maimai 触控/键盘,保留)
- 三个 CDC 各带**可区分的 iInterface 字符串**(如 "mai2 serial"/"mai2 light"/"mai2 config"),便于 UI 识别。
- ★固件影响(后续任务,非现在)★:TinyUSB 需 `CFG_TUD_CDC=3`,重排接口号与端点,改 `hal_usb` 描述符。recon 已标注 arduino-pico 默认 tusb_config.h 的 CDC 数上限需先确认;可能要在 platformio.ini 加 build_flags 覆盖或提供自定义 tusb_config.h。
- 之前 #5a 的帧编解码逻辑不变(帧格式与接口绑定解耦),仅"绑到哪个 CDC 接口"在多 CDC 改造时调整——#5a 不作废。

## R4. serial/light 两个枚举通道有明确 COM 口占位要求 → UI 侧 Win API 自动识别+分配
- maimai 游戏侧(segatools/mai2io 类)期望 serial、light 在**指定 COM 口号**。Windows 对复合设备的多 CDC 分配的 COM 号不确定,需 UI 软件负责:
  1. **识别**:枚举本设备(VID `0x0CA3`/PID `0x0024`)的各 COM 口,用设备实例 ID 里的 **`&MI_xx`(接口号)**映射到功能(serial/light/config)。接口号↔功能约定与固件描述符一致(如 MI_00=serial,MI_02=light,MI_04=config;实现期对齐)。
  2. **分配**:把 serial/light 两个 COM 口**改名到指定占位 COM 号**(注册表 `HKLM\SYSTEM\CurrentControlSet\Enum\...\Device Parameters\PortName`,并同步 `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM`),使游戏在固定 COM 口找到。需管理员权限;改名后可能需重新枚举/提示重插。
  3. config CDC(CDC-C)UI 直接用(按 MI 识别其 COM 口后打开),**不需要固定 COM 号**。
- 这是 UI 软件的 Windows 专属功能模块(SetupAPI + 注册表),Linux/其它平台可降级为"仅识别不改名"。

## 修订后的任务重排(Rust 优先)
- #6a Rust 工程 scaffold(Slint 可运行空窗 + 模块骨架 + 依赖:slint/slint-build/serialport/windows/crossbeam 或 std mpsc)。
- #6b Rust 协议编解码(镜像 #5a host_cmd 帧:SOF 0xAA55/cmd/flags/seq/len/crc16 + 命令枚举),纯逻辑可单测。
- #6c Rust USB CDC IO 层(按 VID/PID 枚举、识别 config CDC、IO 线程 + channel)。
- #6d Windows COM 识别(MI_xx→功能)+ serial/light 自动改名到指定 COM(SetupAPI/注册表)。
- #6e UI:配置页(接替 v3.0 全部配置项,除已砍的 ui.*)。
- #6f UI:绑区可视化(34 区交互式绑定)。
- #6g UI:CSD 实时曲线 + 阈值交互式拖拽调参。
- 固件侧顺延:#5b 配置 schema+CFG 命令 / #5c sensor_link 遥测桥+SPI 扩展 / 多CDC描述符改造 / #5d PSoC 侧。


---

# 修订 2:CFG_* 配置命令线格式(权威定义,双侧唯一真源)

> 基于固件既有 `config_types.h`/`config_manager.h`(复用,不新造)。RP2040 与 Rust 上位机**必须逐字节按此实现**。

## C1. ConfigValueType 类型码(u8,对齐 enum class 顺序)
`BOOL=0, INT8=1, UINT8=2, UINT16=3, UINT32=4, FLOAT=5, STRING=6`

## C2. 值编码(全部小端 LE)
- BOOL/INT8/UINT8 → 1 字节
- UINT16 → 2 字节 LE
- UINT32 → 4 字节 LE
- FLOAT → 4 字节 IEEE754 LE
- STRING → `str_len(u16 LE) + bytes`

## C3. 统一条目格式(Entry,SET 与 GET 响应共用一套编解码)
```
type(u8) + has_range(u8) + key_len(u8) + key[key_len] + value(按type)
          + [ has_range==1 时: min(按type) + max(按type) ]
```
- SET 请求:客户端一般填 `has_range=0`(固件按注册的范围 clamp,不信任客户端范围)。
- GET/GET_ALL/GET_GROUP 响应:固件回填 `has_range` 与 min/max(若该键注册了范围),供 UI 渲染滑块/范围校验。

## C4. 各命令 payload(请求/响应)
响应帧 = 原 cmd 回显 + `flags.RESPONSE(0x01)`;失败一律回 `NAK`(err_code + 可读msg)。
- `CFG_GET(0x10)` 请求:payload = key 原始字节(长度由帧 len 决定)。响应:1 个 Entry。键不存在→NAK(INVALID_PARAM)。
- `CFG_SET(0x11)` 请求:payload = 1 个 Entry。响应:ACK;类型不符/越界/未注册→NAK。
- `CFG_GET_GROUP(0x12)` 请求:payload = prefix 原始字节。响应:`count(u16 LE) + Entry×count`。
- `CFG_GET_ALL(0x13)` 请求:空。响应:`count(u16 LE) + Entry×count`(总量超帧上限 4096 时分多帧:除末帧外 flags.STREAM=1,末帧 flags.RESPONSE;条目不跨帧切分)。
- `CFG_SET_BATCH(0x14)` 请求:`count(u16 LE) + Entry×count`。响应:ACK(全成功)或 NAK(附首个失败 key)。
- `SAVE_CONFIG(0x0E)` 请求:空 → 触发 `ConfigManager::save_config()` → ACK。
- `RESET_DEFAULTS(0x0F)` 请求:空 → `reset_to_defaults()` → ACK。

## C5. RP2040 侧注册的配置 schema(#5b 用 register_init_function 注册;不含已砍 ui.*)
| key | 类型 | 范围 | 默认 | 说明 |
|---|---|---|---|---|
| `comm.sample_delay_ms` | UINT8 | 0-100 | 0 | 采样延迟 |
| `comm.send_only_on_change` | BOOL | - | false | 仅变化发送 |
| `comm.aggregation_delay_ms` | UINT8 | 0-100 | 0 | 聚合延迟 |
| `comm.extra_send` | UINT8 | 0-10 | 0 | 额外发送次数 |
| `comm.rate_limit_en` | BOOL | - | false | 频率限制开关 |
| `comm.rate_limit_hz` | UINT16 | 1-1000 | 120 | 频率限制值 |
| `comm.keyboard_map_en` | BOOL | - | false | 映射键盘 |
| `comm.serial_baud` | UINT32 | - | 115200 | mai2serial 波特率 |
| `comm.light_baud` | UINT32 | - | 115200 | mai2light 波特率 |
| `mode.work` | UINT8 | 0-1 | 0 | 0=Serial,1=HID |
| `led.enable` | BOOL | - | true | 灯效总开关 |
| `led.node_id` | UINT8 | - | 1 | 灯板节点 ID |
| `led.count` | UINT16 | 1-1000 | 128 | LED 数量 |
| `bind.map00`..`bind.map33` | UINT32 | - | 0xFFFFFFFF | 34 区映射(高8位设备掩码+低24位通道bitmap);绑区页 #6f 用 |

> CapSense 逐通道参数(finger_th 等)不在 ConfigManager,走 PARAM_*(链路B)实时读写。


---

# 修订 3:遥测(TELEM_*)与参数(PARAM_*)线格式(权威定义,双侧唯一真源)

> RP2040 与 Rust 上位机必须逐字节按此实现。所有多字节 LE。

## T1. param_id 值编码(u32 承载,对齐修订前 2.5 命名)
`FINGER_TH=0x01, NOISE_TH=0x02, NEG_NOISE_TH=0x03, HYSTERESIS=0x04, ON_DEBOUNCE=0x05, LOW_BSLN_RST=0x06, RESOLUTION=0x07, SNS_CLK_DIV=0x08, IDAC_MOD=0x09, IDAC_COMP=0x0A, SMARTSENSE_EN=0x80`
所有参数值统一用 u32(LE)在协议里传输;各参数真实位宽在 PSoC 侧实现期对齐 CapSense 中间件。

## T2. 遥测字段位(fields, u8 bitmask)
`RAW=bit0(0x01), BASELINE=bit1(0x02), DIFF=bit2(0x04), STATUS=bit3(0x08)`

## T3. 命令 payload
- `TELEM_START(0x30)` 请求:`mode(u8: 0=连续,1=单次) + rate_hz(u16 LE) + fields(u8) + ch_mask(u64 LE, bit i=通道 i)`。响应:ACK。
- `TELEM_STOP(0x31)` 请求:空。响应:ACK。
- `TELEM_DATA(0x32)` dev→host,`flags.STREAM(0x02)` 置位。payload:
  `ts_us(u32 LE) + ch_count(u8) + fields(u8) + [每个包含的通道(按 index 升序): ch_index(u8) + (RAW? raw u16 LE) + (BASELINE? bsln u16 LE) + (DIFF? diff i16 LE) + (STATUS? status u8)]`
  - ch_count = 本帧包含的通道数(由 ch_mask 决定);字段顺序固定 raw→bsln→diff→status,按 fields 决定是否出现。
- `PARAM_GET(0x20)` 请求:`channel(u8) + param_id(u8)`。响应:`channel(u8) + param_id(u8) + value(u32 LE)`(cmd 回显 + RESPONSE);非法→NAK。
- `PARAM_SET(0x21)` 请求:`channel(u8) + param_id(u8) + value(u32 LE)`。响应:ACK/NAK。
- `PARAM_GET_ALL(0x22)` 请求:`channel(u8)`。响应:`channel(u8) + count(u8) + [param_id(u8) + value(u32 LE)]×count`。
- `CALIBRATE(0x23)` 请求:`ch_mask(u64 LE)`。响应:ACK。
- `BASELINE_RESET(0x24)` 请求:`ch_mask(u64 LE)`。响应:ACK。

## T4. 通道数
CapSense 36 通道(通道 index 0..35)。DEVICE_INFO 的 capsense_channels 已报 36。

## T5. 桩数据阶段(#5c-1,PSoC 未到)
RP2040 侧 `sensor_link` 先用**内部假数据源**生成 36 通道 raw/bsln/diff/status(如正弦+噪声),按 TELEM_START 的 rate/fields/ch_mask 组 TELEM_DATA 帧经 config CDC 推给上位机;PARAM_* 用 RP2040 侧参数表暂存(GET/SET 回环);CALIBRATE/BASELINE_RESET 仅置状态。待 PSoC 到位(#5d)把假数据源换成真实 SPI 双缓冲快照,param 读写路由到 PSoC,协议对上位机不变。


---

# 修订 4:系统重启指令(自持托管 / 固件更新用)

> 系统域 0x01-0x0F 内新增两条,双侧实现。用于上位机远程触发重启与进入 UF2 烧录模式(RP2040 基础移动存储枚举,复制 .uf2 烧录)。

- `REBOOT(0x04)` 请求:空 → 固件先回 `ACK`,**确保 ACK 已从 CDC 发出(flush + 短延时泵 USB)后**立即软复位(arduino-pico `rp2040.reboot()`,或 pico-sdk `watchdog_reboot(0,0,0)`)。
- `REBOOT_BOOTLOADER(0x05)` 请求:空 → 固件先回 `ACK`,flush 后进入 UF2 烧录模式(arduino-pico `rp2040.rebootToBootloader()`,或 pico-sdk `reset_usb_boot(0,0)`)。此后设备以 RPI-RP2 移动存储枚举,上位机/用户复制 .uf2 完成更新。
- 时序要点:reboot 不能在写 ACK 之前发生;实现为"处理器置 pending 标志+模式 → 主循环写完响应并泵 USB ~50-100ms 保证送达 → 执行重启 API"。
- UI 侧:维护区提供「重启设备」「进入烧录模式(更新固件)」按钮;发送后连接会断开(IoEvent::Disconnected),UI 提示相应状态。
