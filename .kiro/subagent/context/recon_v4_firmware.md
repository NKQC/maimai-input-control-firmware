# recon_v4_firmware — v4 固件通信/配置/触控架构勘查结论

任务状态：**已完成**（纯只读检索，未改任何代码）。

---

## (A) 当前 host 通信能力现状

### A.1 USB 描述符层（`hal/usb/hal_usb.cpp` / `.h` / `hal_usb_types.h` / `hal_usb_hid.h`）

TinyUSB 复合设备，`tusb_desc_device_t device_descriptor`：`bDeviceClass=TUSB_CLASS_MISC` + IAD，`idVendor=USB_VID(0x0CA3)` `idProduct=USB_PID(0x0024)`（山寨触控板厂商VID/PID，非官方）。

配置描述符 `desc_configuration[]`，`ITF_NUM_TOTAL=3`：
- `ITF_NUM_CDC`(0) + `ITF_NUM_CDC_DATA`(1)：`TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF=0x81, 8, EPNUM_CDC_OUT=0x02, EPNUM_CDC_IN=0x83, 64)`。这是**唯一的上位机通信通道**，当前只用于 CDC（串口）诊断打印（main.cpp 里 `cdc_write` 打印 SWD/SPI 联调日志），没有结构化协议。
- `ITF_NUM_HID`(2)：`TUD_HID_DESCRIPTOR(ITF_NUM_HID, 5, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_descriptor), EPNUM_HID=0x84, CFG_TUD_HID_EP_BUFSIZE, 1)`。

**没有 vendor/WebUSB 接口**，也没有第二个 CDC。`CONFIG_TOTAL_LEN = TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN`——只有 1 组 CDC + 1 个 HID。

`platformio.ini` 里 `build_flags` 未见 `CFG_TUD_CDC`/`CFG_TUD_HID` 等 tusb_config.h 覆盖项，说明 TinyUSB 配置项使用 arduino-pico 框架默认的 `tusb_config.h`（未在本仓库出现，需要另查框架侧默认值才能确认 CDC/HID 数量上限，但当前描述符只声明了 1 个 CDC + 1 个 HID interface）。

关键点：`board_build.src_filter` 排除了 `framework-arduinopico/cores/rp2040/RP2040USB.cpp`，导致框架**不会**自动调度 `tud_task()`，必须在 `main.cpp::loop()` 手动调用 `HAL_USB_Device::getInstance()->task()`（已实现，见下方）。这是新增协议时必须遵守的约束：**任何新的 host 通信通道都要保证不阻塞 `tud_task()` 的调度节奏**。

### A.2 现有诊断通道：CDC（`hal_usb.h::cdc_write/cdc_read/cdc_available/cdc_flush`）

- `HAL_USB_Device` 单例，`cdc_write(data, len)`：带 10ms 超时的阻塞写，逐段 flush。
- `cdc_read` 依赖内部环形缓冲区 `cdc_rx_buffer_[1024]`，由 `tud_cdc_rx_cb` → `handle_cdc_rx()` 填充。
- **当前实际用途**：`main.cpp` 每 1s 用 `snprintf` 格式化一条人类可读诊断字符串（SWD/SPI bring-up 状态），`cdc_write` 发出。**没有帧格式、没有命令解析、纯文本单向打印**。`protocol/usb_serial_logs/` 目录为空（此前的日志通道已裁剪）。
- 没有任何代码读取 CDC 输入做命令处理（`cdc_read` 定义了但目前无调用点）。

### A.3 HID 报表描述符（`hal_usb_hid.h`）

`hid_report_descriptor[]` 定义 4 个 Report ID：
- `REPORT_ID_TOUCHSCREEN = 0x1`：Windows Precision Touch 风格描述符，字段含 Tip Switch(1bit)/Contact ID(7bit)/Confidence(1byte，0x51 Usage)/X,Y(各16bit, LOGICAL_MAX 0x7FFF)/Scan Time(0x56,16bit)/Contact Count(0x54,8bit)/Contact Count Maximum(feature,0x55)。发送函数在 `protocol/hid/hid.cpp::report_touch()`：每次单点 9 字节 `[press,id,x_lo,x_hi,y_lo,y_hi,scantime_lo,scantime_hi,multi_flag]`，逐点发多帧。
- `REPORT_ID_KEYBOARD1/2/3 = 0x2/0x3/0x4`：3 组标准 boot-keyboard 报表(8字节: modifier, reserved, key1..6)，支持最多 `KEYBOARD_SIMUL_PRESS=18`(=3×6) 键同时按下。

这条 HID 通路是**给 maimai 游戏机用**的最终输出（键盘映射 34 触摸分区 + 部分实体按键），不适合复用为"配置/调试"通道（协议固定、report_id 已占满、且是单向 device→host）。

### A.4 UART 侧通信协议（非 USB，串口下位机协议，供参考不复用）

- `protocol/mai2serial/*`：面向"旧街机主板"的定长指令协议（`{LRcv}` 6字节命令 + `(...)` 9字节触摸帧，35位触摸位图打包成5×7bit），已有 34 分区枚举 `Mai2_TouchArea`、掩码宏；`process_dma_received_data` 是流式滑窗解析实现范例。
- `protocol/mai2light/*`：面向 LED 控制板的二进制协议，`Mai2Light_PacketReq/Ack`：`sync(0xE0) + node_id + length + command + data[32] + checksum(XOR)`，已有 EEPROM 读写命令(`SET_EEPROM/GET_EEPROM/SAVE_TO_EEPROM/LOAD_FROM_EEPROM`)、状态查询(`GET_LED_STATUS/GET_BOARD_INFO/GET_PROTOCOL_VERSION`)、系统命令(`RESET_BOARD/ENTER_BOOTLOADER`)——**这是本仓库里最接近"host<->设备配置协议"范式的现成设计**（sync+len+cmd+data+checksum、ACK带status/report字段），新协议的帧格式可以参考它的分层方式（但这是跑在 UART 而不是 USB 上，且是主机模拟"下位机"角色，方向与我们要做的相反：mai2light 是"上位机对我"，而我们要做的是"我对真正上位机"）。

### A.5 `service/usb_comm/*` 现状

`UsbComm` 单例，`init()`/`update()` 均为空实现，注释写明"3xCDC 逻辑延后到后续里程碑"。**这是目前唯一预留的、面向 host 通信服务层的挂载点，但完全未实现。**

---

## (B) 配置数据模型

`service/config_manager/*`，双 map 架构（`_default_map` 只读默认 + `_runtime_map` 可写运行时），底层用 `ConfigValue`（`config_types.h`）+ LittleFS 持久化 + CRC32 校验。

### B.1 `ConfigValue` 结构（`config_types.h`）

```cpp
enum class ConfigValueType { BOOL, INT8, UINT8, UINT16, UINT32, FLOAT, STRING };
struct ConfigValue {
  ConfigValueType type;
  union { bool bool_val; int8_t int8_val; uint8_t uint8_val;
          uint16_t uint16_val; uint32_t uint32_val; float float_val; };
  std::string string_val;
  union { int8_t int8_min; uint8_t uint8_min; uint16_t uint16_min;
          uint32_t uint32_min; float float_min; } min_val;
  union { int8_t int8_max; uint8_t uint8_max; uint16_t uint16_max;
          uint32_t uint32_max; float float_max; } max_val;
  bool has_range;
  void clamp_value();  // 按 min/max 自动夹取
};
typedef std::map<std::string, ConfigValue> config_map_t;
```

**当前没有任何具体业务配置项**——`ConfigManager::initialize_defaults()` 只遍历外部通过 `register_init_function()` 注册的初始化函数（"纯粹外部接口"设计，ConfigManager 本身不知道任何业务字段）。搜索全仓库未见任何模块调用 `register_init_function`，说明目前**没有一个服务真正注册了配置项**——触控灵敏度/阈值/绑区等配置项**目前完全不存在**，需要新建。这与 mai2serial 里的 `RATIO`/`SENS`（比例/灵敏度）指令、以及 psoc CapSense 固件的调参需求对不上——**这是本次协议设计要补的最大缺口**。

### B.2 存储与校验（`config_manager.cpp`）

- 存储介质：LittleFS（`#ifdef PICO_PLATFORM` 用 `pico/LittleFS.h`），单文件 `/config.bin`（内容实际是 JSON 文本）。
- 序列化：手写流式 JSON（`StreamingJsonSerializer`/`StreamingJsonParser`），每个 key 序列化为 `{"type":<int>,"value":"<str>","min":"<str>","max":"<str>"}`，最外层加 `"__crc32__":<uint32>` 字段。
- CRC：`ConfigCRC::calculate_crc32()`，标准 IEEE 802.3 CRC32 查找表算法，对**排序后**(按key)、**排除 CRC 字段自身**的 JSON 串计算。保存时先算 CRC 再拼接写入；读取时重新计算比对，不一致则 `config_read` 失败（`initialize()` 会转去格式化+写默认配置）。
- API 签名（`config_manager.h`，全部 `static` 类方法，无实例状态依赖）：
  - `static bool initialize(); static void deinit();`
  - `static bool has_key(const std::string&); static ConfigValue get(const std::string&); static void set(const std::string&, const ConfigValue&);`
  - 类型化便捷接口：`get_bool/get_int8/get_uint8/get_uint16/get_uint32/get_float/get_string/get_cstring` + 对应 `set_*`
  - 动态接口（未注册键也可写，仅字符串）：`set_string_dynamic/has_string_dynamic/get_string_dynamic`
  - 批量：`get_all() -> std::map<std::string,ConfigValue>`；`set_batch(...)`
  - 分组（按 key 前缀）：`get_group(prefix)`；`set_group(prefix, values)`
  - 注册机制：`register_init_function(ConfigInitFunction)`，`using ConfigInitFunction = std::function<void(config_map_t&)>`
  - 持久化触发：`save_config()`（置位 `_save_requested` 标志，异步）+ `save_config_task()`（真正执行，内部 `multicore_lockout_start/end_blocking()` + 关中断保护写 flash）
  - `reset_to_defaults()`

**这套 get/set/get_group/register_init_function 接口非常适合直接承载"host 读写配置"协议**：host 侧发一个 key（或 prefix）+ 新值，RP2040 侧调用 `ConfigManager::set()` / `get_group()`，改动量很小。唯一要补的是"业务侧注册配置项"（当前完全没有服务调用 `register_init_function`）。

---

## (C) 触控可得数据量与带宽

### C.1 RP2040 <-> PSoC 的物理链路（`protocol/psoc/psoc_spi.*`, `config.h`）

- 引脚：`PIN_PSOC_SPI_SCK=26 / MOSI=28(RP2040输出) / MISO=27(RP2040输入) / CS=29`，因物理接线与硬件 SPI1 TX/RX 相反，**用 PIO1 手写 SPI 主机**（非硬件 SPI 外设）。
- 时钟：`PSOC_SPI_SCK_HZ = 1000000`（1MHz，"首次 bringup 的保守时钟"，注释暗示后续可提速）。
- 协议：MODE0(CPOL0/CPHA0)、8bit、MSB first、全双工，CS 手动管理。

### C.2 帧格式（`psoc_types.h::psoc::Frame`）

```cpp
struct Frame {              // 7 字节，static_assert 强制紧凑
    uint8_t magic = 0xA5;   // FRAME_MAGIC
    uint8_t cmd = 0;        // psoc::Cmd: PING=0x01, PONG=0x02, DATA=0x10(预留)
    uint8_t seq = 0;
    uint8_t payload[4];     // FRAME_PAYLOAD_SIZE=4
};
```

**当前只实现了 `PING`/`PONG` 心跳**（`PsocSpi::ping()`：发 PING 帧，同事务读回校验 magic+cmd==PONG），每 20ms 一次（`Psoc::update()` 里 `LINK_UPDATE_INTERVAL_MS=20`），用于维护 `link_ok()` 布尔状态灯。

`Cmd::DATA = 0x10` **已预留但未实现**——payload 只有 4 字节，若要传 CSD 原始数据（raw count/baseline/diff 等），4 字节远不够，**协议帧格式本身需要扩展**（当前 7 字节定长帧不支持长包/分片，也没有触摸通道号、没有批量数据结构）。

### C.3 PSoC 侧固件现状

PSoC (`psoc_firmware/`) 侧目前是从 Infineon `mtb-example-psoc4-capsense-smartsense-buttons-slider` 官方 CapSense 示例移植（见打开的编辑器文件 `.hex`），RP2040 侧的 `main.cpp::psoc_flash_bringup()` 通过 SWD 每次上电烧录该固件到 PSoC（bring-up 阶段临时方案，注释写明"Phase B 会并入 psoc_updater 按版本比对烧录"）。**PSoC 侧是否已经把 CapSense raw/baseline/diff/touch 数据接到 SPI 从机侧尚未在本次检索范围内确认**（该固件源码不在 `main_firmware` 目录，需要另查 `psoc_firmware/` 才能确认 PSoC 侧 SPI 从机的数据结构，这部分我未检索，如需确认需另起一次 PSoC 侧固件检索）。

### C.4 带宽估算

- 当前 1MHz PIO-SPI，PING/PONG 帧 7 字节 × 2 (tx+rx) 每 20ms 一次 = 极低占用（约 5.6kbps 峰值瞬时，实际平均可忽略）。
- 若要流式上报触控数据：1MHz SPI 理论上限约 125KB/s（8 cycles/byte，无额外开销），实测因 PIO 逐字节 `sm_put_blocking`/`sm_get_blocking`（软件轮询，非DMA）会有可观 CPU 开销和额外延迟，**具体吞吐需实测，本次未做基准测试**。34-38 通道 raw+baseline+diff（若各 2 字节）单帧约 34×3×2=204 字节，1kHz 采样率需要 204KB/s，**已经超出当前 1MHz 无 DMA 的 PIO-SPI 实际吞吐**，若要做到手感级实时可视化，需要：(a) 提升 SPI 时钟；(b) PIO SPI 改造为 DMA 驱动（参考 `hal/uart` 里 UART0/UART1 已用的 DMA TX/RX 环形缓冲设计模式）；(c) 或降采样率。

---

## (D) 主循环/服务分层

### D.1 `main.cpp` 现状（重要：当前仍是"bring-up 联调"临时代码，非最终架构）

```
setup():
  global_irq_init()
  LedService::init()
  [SWD_RELEASE_TO_EXTERNAL 分支，调试用，正常模式为 false]
  Psoc::init()                    // SPI(PIO1)+SWD(PIO0) 双通道初始化
  psoc_flash_bringup(psoc)        // ★临时代码★ SWD烧录+复位PSoC，每次上电都做
  HAL_USB_Device::init()          // 故意放在 bringup 之后，避免枚举超时
  watchdog_enable(5000ms)

loop():
  HAL_USB_Device::task()          // 必须每轮调用，泵 tud_task()
  [SWD_RELEASE_TO_EXTERNAL 分支]
  Psoc::update()                  // 20ms节流的 SPI ping/pong
  LedService 状态灯（红/蓝/绿三态诊断）
  每1s: 通过 cdc_write 打印诊断字符串
  watchdog_update()
```

**`main.cpp` 目前完全没有调用**：`ConfigManager::initialize()`、`HID::init()`、`Mai2Serial`/`Mai2Light` 任何实例、`UsbComm::init()`、`PsocUpdater`。这些服务/协议类都已写好接口但**未被 main 接入**，说明当前 v4 固件还处于"USB+PSoC链路 bring-up"阶段，尚未组装成完整应用。

### D.2 服务分层结构（按代码组织，非运行时已验证的调用链）

```
main.cpp（调度层，目前只接了 LedService + Psoc + HAL_USB_Device）
 ├─ hal/           硬件抽象：usb(TinyUSB封装) / uart(DMA环形缓冲,UART0/1单例) / spi / i2c / pio(PIO程序装载) / global_irq
 ├─ driver/swd/    空目录（SWD底层已内聚进 protocol/psoc/psoc_swd.*，未拆分到这里）
 ├─ protocol/      协议编解码层，各自持有对应 hal 实例：
 │   ├─ hid/            HID报表封装，单例，需外部注入 HAL_USB*（未被main注入）
 │   ├─ psoc/            psoc.h门面 + psoc_swd(SWD编程) + psoc_spi(PIO-SPI链路) + psoc_types(强类型协议定义)
 │   ├─ mai2serial/      UART触摸协议（未被main接入）
 │   ├─ mai2light/       UART LED协议（未被main接入）
 │   ├─ neopixel/        （未读，本次未检索）
 │   └─ usb_serial_logs/ 空目录（日志通道已裁剪）
 └─ service/       业务服务层，单例:
     ├─ config_manager/  纯外部接口配置中心，未被任何模块注册配置项
     ├─ led_service/      GPIO三色状态灯，已接入main
     ├─ psoc_updater/     骨架空实现（未来做版本比对+按需烧录，取代main里的临时bringup）
     ├─ sensor_link/      空目录（触控数据链路的服务层尚未创建）
     └─ usb_comm/         骨架空实现（"3xCDC逻辑延后"，是host通信服务层的预留挂点）
```

---

## (E) ★关键结论★：新协议怎么设计、挂哪、缺什么

### E.1 能复用什么

1. **CDC interface（EPNUM_CDC_IN/OUT=0x83/0x02）**：物理通道已存在（`hal_usb.h::cdc_write/cdc_read/cdc_available`），当前仅用于单向文本诊断打印，**没有被任何双向协议占用**，可以直接复用做双向配置协议的传输层，不需要新增 USB interface（避免改描述符、避免驱动兼容性问题）。
2. **`ConfigManager` 的 get/set/get_group/set_group/register_init_function API**：签名成熟、支持类型化+范围夹取+CRC持久化，主机侧"读/写某个配置key"可以几乎一比一映射到这套接口，RP2040侧改动量小。
3. **`Mai2Light` 的帧格式设计范式**（sync+node_id+length+command+data[32]+checksum(XOR)，ACK带status/report）：作为新协议帧结构的参考模板，虽然它跑在 UART 上、且方向相反，但"命令层+校验+ACK状态位"的分层思路可以直接借鉴。
4. **`psoc::Frame`（7字节 magic+cmd+seq+payload[4]）+ `PsocSpi::ping()`**：已验证可用的 PIO-SPI 收发骨架（`transfer()` 全双工传输API），触控数据流式上报可以复用这条物理链路，但**帧结构必须扩展**（见下）。
5. **`Mai2Serial::process_dma_received_data` 的流式滑窗解析实现**：作为"host→device 命令流"解析器的参考实现范式（处理分包、缓冲区溢出保护）。
6. **UART DMA 环形缓冲设计**（`hal_uart.h` 的 TxBuffer/RxBuffer 静态缓冲 + DMA channel）：若触控数据流量超过 PIO-SPI 软件轮询吞吐上限，这套"DMA+环形缓冲"模式是现成的可复用范式（需要为 PsocSpi 补一份类似实现）。

### E.2 缺什么（本次协议要新建的部分）

1. **CDC 双向协议帧格式完全没有**——目前 CDC 只会往外打印诊断文本，没有任何"host→device 命令帧"的解析器（对比 mai2serial 有、mai2light 有，但 USB CDC 侧空白）。需要新建一套 USB CDC 侧命令帧协议（建议：定长/变长二进制帧，含 magic+cmd+len+payload+checksum，参考 Mai2Light_PacketReq 范式）。
2. **配置项 schema 完全没有注册**——`ConfigManager` 是空壳，没有任何 `register_init_function` 调用者。触控灵敏度/阈值/绑区相关配置项需要从零定义（建议放在一个新的 `service/touch_config/` 或直接在 `sensor_link`（空目录）里新建，通过 `ConfigManager::register_init_function` 注册默认值）。
3. **触控数据流式上报完全没有**——`psoc::Cmd::DATA` 只是预留枚举值，未实现；`Frame::payload[4]` 太小，不足以承载多通道 raw/baseline/diff。需要：
   - 扩展 `psoc::Frame` 或新增一个变长/分片帧类型（例如加 `total_len`/`chunk_index` 字段，或改为"长度前缀+可变payload"设计）。
   - PSoC 侧固件需要新增"把 CapSense raw/baseline/diff 塞进 SPI 从机可读寄存器"的逻辑（本次未检索 `psoc_firmware/`，需要另外确认 PSoC 侧现状）。
   - RP2040 侧需要新的 `service/sensor_link/`（当前空目录，是天然的挂载点）来做"从 PsocSpi 拉数据 → 组装成 host 可读的流式协议 → 经 CDC 或新起的 USB interface 转发"。
4. **绑区（分区映射）配置的存储结构没有**——`Mai2Serial` 里有 34 分区枚举和"手动触发覆盖层"（`manually_triggle_area`），但这是运行时临时状态，不是可持久化配置；触控通道号→游戏分区的映射关系目前硬编码在别处（未在本次检索范围内找到映射表来源，需要确认 PSoC 侧通道数与 `Mai2_TouchArea` 34分区的对应关系）。
5. **`UsbComm`/`PsocUpdater` 都是空壳**，服务层没有被 `main.cpp` 组装调用——新协议要接入的话，`main.cpp` 本身也需要相应改造（目前 main.cpp 处于 bring-up 阶段代码，注释已声明"Phase B 会重构"）。

### E.3 建议挂载位置

- **命令/配置协议（host↔RP2040 双向）**：挂在现有 CDC interface 上，新建一个协议解析层（可放在 `protocol/` 下新建 `protocol/host_cmd/` 或复用扩展 `service/usb_comm/`），命令解析后调用 `ConfigManager::get/set/get_group/set_group`。不建议新增 USB interface（改描述符风险高、且 CDC 通道当前完全空闲）。
- **触控数据流式上报（RP2040→host，高频）**：如果流量超过 CDC 一条通道能承载（CDC 64字节包+软件层还要跑配置协议），考虑：
  - 方案A（改动小）：复用同一 CDC，用命令协议里加"进入/退出流模式"命令，流模式下高频写 `cdc_write`（注意 `cdc_write` 当前是阻塞式10ms超时，高频调用需要评估CPU占用）。
  - 方案B（改动大，需改USB描述符）：新增第二个 CDC 或 vendor interface 专门跑流数据，避免和配置命令抢同一条队列。是否值得，取决于触控数据实际所需带宽（见C.4，需要先在 PSoC 侧固化数据结构后才能定）。
  - 两种方案都建议新建 `service/sensor_link/`（现成空目录）作为"PsocSpi数据 → USB协议帧"的服务层胶水代码，不要让 `protocol/psoc/` 直接依赖 USB（保持现有"协议层不互相耦合，服务层做胶水"的既有分层规范）。
- **绑区/阈值配置的落盘**：走 `ConfigManager`，新建配置key（如 `touch.sensor_XX.threshold` / `touch.area_map.XX`），用 `register_init_function` 注册默认值和 range，用 `get_group("touch.")` 支持前端批量拉取。

---

## 未覆盖/需要用户确认的点

1. **PSoC 侧固件（`psoc_firmware/`）未检索**——不确定 PSoC 从机现在能否通过 SPI 提供 CapSense raw/baseline/diff，或者目前仅支持 PING/PONG。若要设计流式协议的 payload 内容，需要另起一次针对 `psoc_firmware/` 的检索。
2. **34分区与 PSoC 触控通道号的映射表来源未定位**——`Mai2Serial` 侧只有分区枚举和覆盖层逻辑，没找到"物理通道→逻辑分区"的映射配置来源。
3. **`neopixel/` 协议目录未检索**（本次范围未覆盖，若涉及灯效协议可另行检索）。
4. **arduino-pico 框架默认 `tusb_config.h` 的 CFG_TUD_CDC/CFG_TUD_HID 上限值未确认**——若要新增USB interface需要先确认框架侧限制。
