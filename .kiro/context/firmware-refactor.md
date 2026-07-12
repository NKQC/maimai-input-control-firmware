# 任务：RP2040 + PSoC 固件框架重构（firmware-refactor）

## 总目标
重构 `main_firmware`（RP2040）固件：裁剪掉不再需要的驱动、按参考工程风格重组分层架构；
建立面向高带宽的 RP2040↔PSoC SPI 协议层；后续做 3xCDC USB 并行枚举、RP2040 通过 SWD 给 PSoC 烧录 + 版本自动更新。
硬件不再做屏幕交互，UI 相关工作全部移交上位机 `control_software` 并适当拓展。

## 当前状态：2 个阻塞点已解除，正在派 researcher 做 main.cpp/API 映射 → 随后派 implementer 落地 RP2040 里程碑1
（本文件由 /save 生成。恢复时用 `/load firmware-refactor`。）

### 用户最新指令（本轮）
- **MISO 修了**：PSoC MISO 路由已补。⚠ 以 hardware.txt 为准，正确路由为 **P1.0=MISO, P1.1=MOSI, P1.2=SCK, P1.3=CS**（旧 kit cycfg 是 P1.0=MOSI，MOSI/MISO 反了）。做 PSoC 侧时须核实 cycfg_routing 已按 hardware 修正。
- **led 以硬件 data 为准**：PSoC 状态灯 = hardware.txt 明确的 **P1.6 = LED W**。阻塞点2解除。
- **烧录 PSoC 的前提是 RP2040 SWD 完备**：SWD 编程器从"延后"提升为 **PSoC 实机验证的前置条件**。里程碑顺序调整为：
  RP2040 base → RP2040 SWD 编程器 → PSoC 固件 → 经 SWD 烧录 → 联调 ping/pong。

---

## 用户确认的关键决策
- 裁剪 = **直接删除文件**（git 有历史可回溯）。不再做设备端屏幕/UI。
- 两个 LED = 主 IC 的 **RGB 灯（GPIO18-20，R/G/B 三引脚）**，任意闪法、多用途测试指示。
- 另有 **PSoC 上的 LED（W）**，用于验证 PSoC 存活。
- PSoC 已在 ModusToolbox UI 里配置 SPI(SCB slave)+DMA，为后续直接获取 capsense 原始数据铺垫。
- 里程碑1 目标：设计**最大化带宽的 SPI 协议层**并跑通**双向控制**：
  RP2040 发 ping 控制 PSoC W LED 闪烁；PSoC 回 pong 控制 RP2040 RGB 其中一个色彩状态，变相验证 SPI 可用。
- 3xCDC、SWD 烧录、PSoC 自动更新 **延后**（本里程碑只搭空骨架）。

---

## ⚠ 待用户拍板的 2 个阻塞点（已在对话中提出，尚未回复）
1. **MISO(P1.1) 未路由**：PSoC SCB0 SPI 路由里只有 MOSI/SCK/CS，缺 MISO → 从机无法回传 pong。
   需用户在 Device Configurator 把 **P1.1 → SCB0_SPI_MISO** 补上并重生成（`make build` 触发），
   或授权主 agent 改 `design.modus`（更建议用户在 UI 改，避免手改生成文件被覆盖）。
2. **PSoC 状态 LED(W) 实际引脚确认**：用户说 P1.6，但那是复用 CY8CKIT-149 BSP，kit 宏 P1.6=CYBSP_LED_SLD3，
   另有 CYBSP_USER_LED=P3.4。自定义板实际连线需用户确认。

附注：`cycfg_pins.h` 残留 kit 默认 CYBSP_SPI_*=P5.0~5.3，与实际路由到 P1 不一致 → 生成文件半新半旧，改完 MISO 后跑干净再生成使引脚与路由一致。

---

## 现有 main_firmware（旧 v3 架构）
- 模式：单例 HAL（getInstance）；protocol/service 用 `new`；双核 core0/core1，用 `CoreInitBitmap` 同步结构；watchdog；分层 init（basic→hal→protocol→service）。
- `src/main.cpp` ~900 行，很乱：引脚宏堆在文件顶部；业务逻辑（按键映射、触摸映射）内联在 `init_service_layer()`；init 跨核拆分混乱；UI/Config 初始化塞在 `init_basic()`。
- SYSTEM_VERSION "3.0.2"，HARDWARE_VERSION "3.0"。
- 旧 v3 引脚（main.cpp 内，**与新硬件不符**）：SPI0→ST7735(16/18/19,cs17,dc21,rst20,blk22)，SPI1→MCP23S17(sck26,mosi27,miso28,cs29)，UART0(12/13) UART1(8/9)，I2C0(4/5) I2C1(6/7)，NeoPixel pin11。
- 模块：hal/{global_irq,i2c,pio,spi,uart,usb}，protocol/{hid,mai2light,mai2serial,mcp23s17,neopixel,st7735s,touch_sensor(ad7147/gtx312l/psoc),usb_serial_logs}，service/{config_manager,input_manager,light_manager,ui_manager}。
- hal_usb：TinyUSB（platformio.ini: USE_TINYUSB=1，board src_filter 排除核心 RP2040USB.cpp）。usb_serial_logs 是日志通道（get/set_global_instance、flush、task、info/error/infof）。

### 裁剪（直接删除）
st7735s（屏幕/2D）、ui_manager（屏幕 UI）、uart（hal_uart + 使用者）、全部 touch_sensor（ad7147/gtx312l/psoc/touch_sensor）、mcp23s17、usb_serial_logs。

### 保留复用
hal_spi、hal_pio、neopixel、mai2serial、mai2light（数据源 UART→CDC）、hal_usb（重做为 3xCDC）、config_manager（评估）、hid（评估）。

---

## 新 v4 硬件（hardware.txt，新固件权威）
- GPIO1-12 键盘（1K 上拉，gpio→二极管→out）
- GPIO13-14 WS2812 x2
- GPIO16 SWD-DAT（PSoC 编程）；GPIO17 SWD-CLK；GPIO21 SENSOR-RST
- GPIO18-20 LED B/G/R（3 颗状态灯，RGB）
- GPIO22 INT2；GPIO23 INT1
- GPIO24 NFC-SDA；GPIO25 NFC-SCL
- GPIO26-29 = SPI1 SENSOR：SCK=26 / MOSI=27 / MISO=28 / CS=29（RP2040 SPI1 硬件外设正好对应）
- PSoC 传感器 CSD 触摸，5V，经 100MHz 电平转换中继连接。

---

## PSoC 工程（psoc_firmware/CY8C4147AZI-SensorCore，ModusToolbox CAT2/PDL）
- 器件 CY8C4147AZI-S475（hw 写 S455；都是 PSoC4100S Plus，128KB flash@0x0，16KB RAM，flash row 0x100=256B）。
- APPNAME=mtb-example-psoc4-capsense-smartsense-buttons-slider（原厂 capsense 示例）。
  TARGET=APP_CY8CKIT-149，TOOLCHAIN=GCC_ARM，CONFIG=Debug，MTB_TYPE=COMBINED。
- 生成配置位置：`bsps/TARGET_APP_CY8CKIT-149/config/GeneratedSource/`（design.modus 同目录父级 config/）。
- 输出 hex：`build/APP_CY8CKIT-149/Debug/<APPNAME>.hex`（尚未构建，无产物）。
- 构建命令：在 `psoc_firmware/CY8C4147AZI-SensorCore` 下 `make build`。

### SCB0 SPI（scb_0_config，已核实）
- spiMode=CY_SCB_SPI_SLAVE（从机；RP2040=主机）
- sclkMode=CPHA0_CPOL0 → **SPI MODE 0**
- rxDataWidth=8, txDataWidth=8（8 位）；enableMsbFirst=true；ssPolarity=ACTIVE_LOW；rxFifoTriggerLevel=7
- 时钟 PCLK_SCB0_CLOCK，16-bit divider index 3
- 宏：scb_0_HW=SCB0，scb_0_IRQ=scb_0_interrupt_IRQn，config=scb_0_config

### SPI 引脚路由（cycfg_routing.h，权威）
- P1.0=SCB0_SPI_MOSI；P1.2=SCB0_SPI_CLK；P1.3=SCB0_SPI_SELECT0(CS)
- **MISO(P1.1) 未路由** ← 阻塞点1
- PSoC 自身 SWD：P3.2=SWD_DATA，P3.3=SWD_CLK

### DMA（cycfg_dmas.c，已配，高带宽基础）
- DMAC 通道0=RX、通道1=TX；SCB0 RX/TX 触发经 trigger mux 接入。
- 描述符为占位（srcAddress=0,dstAddress=0,dataCount=1,WORD,ping-pong flip=true,interrupt=true）→ 固件运行时填真实 buffer/长度。

### PSoC main.c（仍是原厂 capsense 示例）
- cybsp_init() + EZI2C tuner + CAPSENSE(CSD0) + PWM1/PWM2 + SMART_IO(PRGIO_PRT1)，主循环扫 capsense + tuner。
- **无 SPI 代码，无 FW_VERSION**。capsense 组件(cycfg_capsense.*)保留不动。
- 需新增：SPI 从机 init（Cy_SCB_SPI_Init(SCB0,&scb_0_config,&ctx)+Enable+Cy_SysInt for scb_0_IRQ）、ping/pong、点 W LED、加 FW_VERSION 宏。

---

## 参考工程风格（F:\mai2control\src）
- 分层：hal(硬件抽象,基类接口如 DisplayDriver) / module(器件驱动,依赖 hal) / service(业务,相互独立,init(Deps&) 注入) / app(组装 service)，调用方向单向向下。
- 单例：static T& instance() + 私有构造，无全局/new。
- 解耦：基类指针 + init(Deps&) 依赖注入结构体。
- 命名：PascalCase 类/方法/文件；config.h 集中 #define/#ifndef 默认值 + constexpr。
- main.cpp：分阶段 init（HAL→…→Services）+ 轻 loop 调 update()。
- **本项目规则覆盖**：类内部成员/函数以 `_` 开头，对外接口不加 `_`；多用 union/struct；循环≤3层；工具函数内联高内聚；不写测试/README，用编译验证。

---

## 拟定新目录结构（main_firmware/src）
```
src/
  config.h                 集中 v4 引脚 + 参数（constexpr 为主）
  main.cpp                 极简：分阶段 init + 轻调度 loop
  hal/  spi/(复用,SPI1->PSoC)  pio/(复用,WS2812)  usb/(重做3xCDC)
  driver/  neopixel/(复用)  swd/(新增:位操作SWD编程器 GPIO16/17+RST21)
  service/
    led_service/     BGR 状态灯(GPIO18-20)
    sensor_link/     SPI1 与 PSoC 通信通道 + 带宽协议 + ping/pong
    usb_comm/        3xCDC 管理(CDC0=mai2serial,CDC1=mai2light,CDC2=command)
    command_service/ #command# $data$ 解析(替代旧 log 通道)
    psoc_updater/    版本比对 + SWD 烧录
    config_manager/  复用(评估)
  protocol/  mai2serial/(复用,改走CDC)  mai2light/(复用,改走CDC)
```

## 里程碑1 落地拆分（两套固件）
- **RP2040**：config.h(v4引脚) + 删除裁剪 + 精简 main.cpp + 复用 hal/spi(SPI1:26/27/28/29,mode0/8bit/MSB/CS低)
  + 保留 neopixel + led_service(RGB GPIO18-20) + sensor_link(SPI 主机 + 带宽协议帧 struct/union + ping/pong:
  ping 切 PSoC W LED；收到 pong 驱动一路 RGB) + 空骨架(usb_comm/swd/psoc_updater) + pio 编译通过。
- **PSoC**：加 SPI 从机 + ping/pong + 点 W LED + FW_VERSION，capsense 保留，make build。
- **延后**：3xCDC USB、SWD 烧录器 + PSoC hex 内嵌 + 自动更新。

## 3xCDC 设计（延后实现）
TinyUSB CFG_TUD_CDC=3；CDC0=mai2serial(触摸数据)、CDC1=mai2light(灯光)、CDC2=command_service(指令,#command#+$data$,struct/union 帧,禁动态内存)。

## SWD + 自动更新设计（延后实现）
RP2040 位操作 SWD（SWCLK=GPIO17,SWDIO=GPIO16,XRES=GPIO21）走 PSoC4 flash 编程；PSoC hex 转数组内嵌 RP2040；上电读 PSoC FW_VERSION，低于内置版本则自动烧录。

## 待办进度
- [x] 调研旧 main_firmware 架构
- [x] 调研参考工程风格
- [x] 调研 PSoC 工程 SPI/DMA/LED 配置
- [x] 设计新架构、与用户对齐范围/颗粒度
- [ ] 等用户确认 2 个阻塞点（MISO 路由、W LED 引脚）
- [ ] implementer 落地 RP2040 里程碑1（裁剪+框架+RGB+SPI主机+协议骨架）
- [ ] implementer 落地 PSoC 里程碑1（SPI 从机+ping/pong+W LED+FW_VERSION）
- [ ] platformio 编译验证 RP2040；make build 验证 PSoC

## 恢复后立即要做
先看用户是否已回复 2 个阻塞点。若已确认 → 派 implementer 落地 RP2040 里程碑1（PSoC 侧待 MISO 配好紧接着做）。
若未确认 → 可先只做 RP2040 侧裁剪+框架（不依赖 PSoC 的部分）。

---

## 映射完成（本轮，module-builder 只读 + 直接读取核实）

### 已核实 API（保留模块）
- **HAL_SPI1::getInstance()**；`init(sck,mosi,miso,freq)`（不含 cs）；`set_cs_pin(cs,active_low=true)`+`cs_select/cs_deselect`；`set_format(data_bits,cpol,cpha,bit_order 0=MSB)`；`set_frequency`；同步 `write/read/transfer(tx,rx,len)`；DMA 变体。
- **NeoPixel(HAL_PIO*,num,type=RGB)**；`init/set_pixel/set_all_pixels/clear_all/show/set_brightness`。旧代码用 PIO1。
- **HAL_USB_Device::getInstance()**：当前仅 1 路 CDC（`cdc_write/read/available/flush`）+`send_hid_report`。3xCDC 延后。
- **Mai2Serial(HAL_UART*) / Mai2Light(HAL_UART*,node)** 依赖 hal/uart。HID::getInstance()->init(HAL_USB*)。ConfigManager 全静态单例 + LittleFS。

### 旧 main.cpp 结构
双核（core0=UART/I2C/PIO+neopixel/mai2serial/mai2light+服务层；core1=SPI1+mcp23s17+hid），CoreInitBitmap 位域同步，watchdog 5s，usb_serial_logs 全局日志贯穿全文，loop() 空。

### 里程碑1 落地决策（已定）
- **删除**：st7735s、ui_manager、touch_sensor/*、mcp23s17、usb_serial_logs、input_manager、light_manager，并清理所有引用（含 USB_LOG 宏）。
- **保留但暂不实例化（仍参与编译）**：hal/{global_irq,i2c,pio,spi,uart,usb}、neopixel、mai2serial、mai2light、hid、config_manager。
- **⚠ 偏离原裁剪清单**：hal/uart 暂不删（mai2serial/mai2light 依赖它，其 UART→CDC 迁移属延后项）。待 CDC 迁移里程碑再删 uart。
- **引脚修正**：WS2812 用 **GPIO13/14**（旧代码错用 pin11）；RGB 状态灯 **GPIO18=B/19=G/20=R** 普通 GPIO；SPI1=26/27/28/29；SWD=16/17/21。
- **重写 main.cpp 为极简**（去日志，去被删模块）：init USB → SPI1 主机 → led_service → sensor_link；loop 跑 ping/pong + RGB。
- **新增**：config.h(v4 引脚 constexpr)、service/led_service(RGB 普通 GPIO)、service/sensor_link(SPI1 主机 + 定长帧 struct/union + ping/pong)、空骨架 service/usb_comm + driver/swd + service/psoc_updater。neopixel 可迁 driver/（同 include 深度，安全）。
- **验证**：main_firmware 下 `pio run` 编译通过。

---

## /save 快照（Opus 4.8 会话，里程碑1 RP2040 编译收尾中）

### 重大发现：里程碑1 RP2040 代码已由之前的 module-builder 执行完成（尽管返回"aborted"）
`src/` 现状（已核实）：裁剪目标已全部删除（st7735s/ui_manager/touch_sensor*/mcp23s17/usb_serial_logs/input_manager/light_manager 均无源文件，仅剩空目录无害）。新文件已存在：
- src/config.h（constexpr v4 引脚，已核实正确：SPI1 26/27/28/29、RGB 18=B/19=G/20=R、SWD 16/17/21、WS2812 13/14、SPI1_FREQ_HZ=4000000、LED_ACTIVE_HIGH=true、SYSTEM_VERSION "4.0.0"）
- src/main.cpp（极简单核 setup+loop）
- src/service/led_service/{.h,.cpp}（RGB 普通 GPIO 单例）
- src/service/sensor_link/{.h,.cpp}（SPI1 主机 + SensorFrame 定长帧 + ping/pong）
- src/service/usb_comm、src/service/psoc_updater、src/driver/swd（空骨架，均 OK）
sonnet 写的代码质量良好，符合 _前缀/单例/struct.clear()/DI 规范。

### 子代理状态：researcher 与 module-builder 在本环境反复 "aborted"（疑似环境限制）。已改为主 agent 直接落地。

### 本会话已完成的解耦改造（依据用户新指令："config 和其他 manager 需重构，降耦合；config 做成外部调用接口；服务间不互相耦合；只允许上层以对象指针注入下层"）
1. **config_manager.cpp 顶部**：删除 `#include ../input_manager/../light_manager/../ui_manager/../../protocol/usb_serial_logs`（已完成）。
2. **initialize_defaults()**：删除硬编码的 `inputmanager/lightmanager/uimanager_register_default_configs()` 三个调用，只保留 `_init_functions`（register_init_function 注册机制）。config 不再引用任何具体服务。（已完成）
3. **config log_debug/info/error**：改为空实现 `(void)message;`（去掉 USB_SerialLogs 依赖）。（已完成）
4. **mai2light.cpp**：删除 usb_serial_logs include；log_debug/log_error 改空实现。（已完成）
5. **sensor_link 解耦 led_service**：sensor_link 只依赖 HAL_SPI（init(HAL_SPI*)），新增 `bool link_ok() const`；update() 只刷新 _link_ok，不再驱动 LED。main.cpp 作为组装层读 link_ok() 后驱动 led_service.set_rgb(!ok,ok,false)。（已完成）

### ⚠ 当前编译阻塞点（下一步必做）
`pio run` 仍失败于 config_manager.cpp **约 1250-1260 行**（应在某个 save/序列化路径里），残留对已删服务的调用，需清理/重构：
- `inputmanager_write_config_to_manager(input_config)`
- `UIManager_PrivateConfig ui_config = ui_manager_get_config_copy();` + `ui_manager_write_config_to_manager(ui_config)`
- `LightManager_PrivateConfig light_config = lightmanager_get_config_copy();` + `lightmanager_write_config_to_manager(light_config)`
处理方向（符合解耦原则）：这些"从服务抓取配置写回 manager / 把 manager 配置写回服务"的双向硬编码耦合应整段删除；改由各服务自己在需要时通过 ConfigManager 的静态 get/set 外部接口读写，或通过 register_init_function 注册默认值。删除后重新 `pio run` 直到通过。可能还有其它残留（如 config_manager.cpp 里对应的读取路径、config_types.h 引用），用编译器迭代清干净。

### 编译命令
主 agent 直接跑（子代理不可用）：在 f:\mai2control\mai2control-v4\main_firmware 下 `pio run`（PowerShell）。注意 execute_pwsh 用 `Select-Object -Last 60` 截尾看错误；不要复用旧后台终端（会显示陈旧缓冲）。框架 .o 已缓存，失败很快（约3s）。

### 里程碑顺序（用户已确认）
RP2040 base（编译通过=收尾中）→ RP2040 SWD 编程器（PSoC 烧录前置）→ PSoC 固件（SPI从机+ping/pong+W灯P1.6+FW_VERSION，需先核实 cycfg_routing 已改 P1.0=MISO/P1.1=MOSI）→ 经 SWD 烧录 → 联调。

### 待办
- [x] 清理 config_manager.cpp save_config_task 残留服务耦合（inputmanager/ui_manager/lightmanager 三段双向硬编码整段删除，改注释说明各服务自行走静态接口）→ **pio run [SUCCESS] 4.40s，RAM 7.0%/Flash 3.9%，里程碑1 RP2040 base 编译通过 ✅**
- [ ] RP2040 SWD 编程器（PSoC 烧录前置；driver/swd 目前是空骨架，需实现位操作 SWD：SWCLK=GPIO17/SWDIO=GPIO16/XRES=GPIO21，PSoC4 flash 编程时序）
- [ ] PSoC 里程碑1 固件（SPI从机+ping/pong+W灯P1.6+FW_VERSION；先核实 cycfg_routing 已改 P1.0=MISO/P1.1=MOSI）
- [ ] 联调验证 ping/pong

### /load 恢复点（RP2040 base 已收尾）
里程碑1 RP2040 base 编译通过。下一步二选一（需用户拍板方向）：
(A) 实现 RP2040 SWD 编程器（driver/swd）——PSoC 实机烧录/验证的前置；
(B) 先写 PSoC 里程碑1 固件（SPI 从机+ping/pong+W LED+FW_VERSION），但烧录仍依赖 SWD。
子代理在本环境反复 aborted，如仍不可用则主 agent 直接落地。


---

## /save 快照（Opus 4.8 会话 #2，SWD 编程器里程碑调研阶段）

### 里程碑1 RP2040 base ✅ 已完成并编译通过
- 本会话开头修复了 config_manager.cpp `save_config_task()` 里残留的服务耦合（删除 inputmanager/ui_manager/lightmanager 三段双向硬编码，改注释：各服务自行走 ConfigManager 静态 get/set）。
- `pio run` → **[SUCCESS] 4.40s，RAM 7.0% / Flash 3.9%**。RP2040 base 全部裁剪+框架+RGB+SPI主机+协议骨架编译干净。

### 用户新指令（本会话）
- **硬件已连接**：RP2040 在 **G:/**（BOOTSEL 大容量存储盘），可实际烧录测试。烧录方式：把 `.pio/build/pico/firmware.uf2` 复制到 G:/。
- **验证/上传前必须用提问工具**：需要烧录或联调时，用 ask/提问工具**等待用户**，并告知用户「进入下载模式（BOOTSEL）」和「返回设备状态」。不要擅自烧录。
- **优先用 subagent 做范围任务保护主上下文**（researcher 中小调研 / implementer 中小写码 / module-builder 大模块）。

### 子代理状态更新
- 本会话 **researcher 纯本地调研成功**（读 driver/swd 骨架+架构，见下）。
- **researcher 纯 Web 调研仍 aborted**（PSoC4 SWD 协议那次）。改为用**用户提供的官方 PDF** 替代 Web。

### 里程碑2「RP2040 SWD 编程器」进行中——已完成调研，待设计+落地

#### 本地架构调研结论（researcher，已核实）
- **driver/swd/swd.{h,cpp}**：`SwdProgrammer` 类，**非单例**，构造 `SwdProgrammer(uint8_t io,clk,rst)`，成员 `_io_pin/_clk_pin/_rst_pin`，`init()/update()` 空实现（里程碑1新建的空骨架，非 v3 遗留）。
- **config.h**：`PIN_SWD_IO=16`（SWDIO）、`PIN_SWD_CLK=17`（SWCLK）、`PIN_SWD_RST=21`（XRES），全 `constexpr uint8_t`。
- **GPIO 位操作**：工程直接用 RP2040 SDK `gpio_init/gpio_set_dir/gpio_put/gpio_get`（`pico/stdlib.h`+`hardware/gpio.h` 已可用，led_service/hal_pio 都在用）。位操作 SWD 可直接用这些，无需 sio_hw。
- **组装模式**：main.cpp setup() 里 `Service::getInstance()->init(dep)`；单例风格 `getInstance()+私有构造+static _instance`，私有成员 `_` 前缀，struct 带 `clear()`。SwdProgrammer 建议保持 driver（无上层依赖）；可保留 ctor 注入或改单例，落地时定。
- **HAL_SPI** 有同步 transfer，但 SWD 是 bit-bang GPIO，不走 SPI 外设。

#### PSoC4 编程规格（官方 PDF，已定位+已解析）
- 用户给的权威文档：`F:\mai2control\mai2control-v4\infineon-cy8c4xxx-cyblxxxx-programming-specifications-programming-specifications-en.pdf`（**Infineon 002-22325 Rev *H，CY8C4xxx/CYBLxxxx Programming Specification，53 页**，覆盖 CY8C4147 目标器件）。
- **已用 pymupdf 解析全文到 `.kiro/context/psoc4_progspec.txt`（53 页纯文本，恢复时直接读它，别再解析 PDF）**。（环境无 pdf 库，本会话 `python -m pip install pymupdf` 装好，再 `fitz` 提取。）
- **目录关键章节（页码）**：
  - §3.2 SWD interface (p11)、§3.3 Hardware access commands (p12)、§3.4 Pseudocode (p14)、§3.5 Physical layer (p15)
  - §4.1 High-level programming flow (p17)、§4.2 Subroutines (p18)
  - §4.3 **Step 1A – Acquire chip after hard reset (p20)** ← acquire 时序核心
  - §4.4 Step 1B – Acquire (alternate) (p24)、§4.5 Step 2 Check silicon ID (p27)
  - §4.6 Step 3 Erase all flash (p28)、§4.7 Step 4 Checksum (p29)
  - §4.8 **Step 5 – Program flash (p30)**、§4.9 Step 6 Verify (p33)
  - §4.10 Step 7 Program protection (p35)、…、§4.12 Step 9 Verify checksum (p41)
  - 附录 **C Serial Wire Debug (SWD) protocol (p49)**、**D Timing specs of SWD interface (p51)**
- ⚠ 尚未精读 acquire/SROM 调用/SWD 线协议的**具体寄存器地址与时序数值**——下一步必须精读 psoc4_progspec.txt 的 p11-p35 + p49-p51，提炼出：JTAG→SWD 切换序列、SWD 包格式/ACK/parity/turnaround、DPACC/APACC、Test Mode acquire 时序窗口、CPUSS SYSREQ/SYSARG SROM 调用（Load Latch/Program Row/Silicon ID/Checksum 的 opcode+key1/key2+参数）、flash row size。

### 当前 todo（里程碑2）
- [x] researcher 本地调研 driver/swd 骨架+架构+集成点
- [x] 定位并解析 PSoC4 官方编程规格 PDF → psoc4_progspec.txt
- [ ] **精读 psoc4_progspec.txt（p11-35, p49-51）提炼实现规格**（下一步；可派 researcher 纯本地读该 txt，或主 agent 直接读分段）
- [ ] 设计 SWD 编程器 API + 派 implementer/module-builder 落地位操作 SWD + PSoC4 flash 编程
- [ ] pio run 编译验证
- [ ] （需用户确认后）复制 uf2 到 G:/ 实机验证

### 里程碑顺序（用户已确认，不变）
RP2040 base ✅ → **RP2040 SWD 编程器（进行中）** → PSoC 固件（SPI从机+ping/pong+W灯P1.6+FW_VERSION，先核实 cycfg_routing 已改 P1.0=MISO/P1.1=MOSI）→ 经 SWD 烧录 → 联调 ping/pong。

### 恢复后立即要做
读 `.kiro/context/psoc4_progspec.txt` 的 p11-35 与 p49-51，提炼 SWD 位协议 + PSoC4 SROM 编程实现规格；然后设计 SwdProgrammer API 交给 subagent 落地。编译命令：main_firmware 下 `pio run`（PowerShell，`Select-Object -Last 60` 看尾部）。烧录测试须先用提问工具等用户进 BOOTSEL。


---

## /save 快照（Opus 4.8，里程碑2 完成 → 转里程碑3 PSoC 固件）

### 里程碑2 RP2040 SWD 编程器 ✅ 完成
- **2a（PIO SWD 传输层）实机验证通过**：CDC `idcode=0x0BC11477 ok=1 acquired=1 silicon=0x000012B5`（family 0xB5=PSoC4100S Plus/CY8C4147，Rev 0x12）。PIO 硬件驱动 SWD、DP/AP、acquire(Step1A)、SROM 系统调用全部工作。根因修复：SM 必须 `sm_exec(pio_encode_jmp(_offset+get_next_cmd))` 后再 enable（对照官方 probe.c）；加 JTAG→SWD 0xE79E 切换序列。代码见 `.kiro/subagent/context/milestone2_swd_pio.md`。
- **2b（SROM flash 流程）代码完成 + 编译通过**：erase_all/checksum_all/program_row/program_flash/verify_flash。**按用户决策（方案B）推迟到里程碑3 用真实 PSoC hex 验证**，不做破坏性自测。
- 子代理（researcher 本地读可用；module-builder/implementer/纯Web 在本环境 aborted）→ 代码由主 agent 直接落地。

### 本轮用户关键决策/信息
1. **方案B**：PSoC flash 烧写留到里程碑3 与真实固件一起测。PSoC 之前项目稳定、无暗病。
2. **PSoC 固件不占用 SWD 引脚**（P3.2 SWD_DATA / P3.3 SWD_CLK 保持 SWD 功能），让 SWD 始终可用、随时可重烧。我们有 SWD+RST 权限，acquire 用 Step1A（硬复位 XRES）能在 400µs 窗口内处理，无需 alternate 方式。
3. **RGB 灯实测映射**：GPIO20=蓝 / GPIO19=红 / GPIO18=绿（config.h 已改：PIN_LED_G=18/PIN_LED_R=19/PIN_LED_B=20；LED_ACTIVE_HIGH=false 共阳）。hardware.txt 的 B/G/R 标注有误。

### 已改文件（里程碑2）
- `main_firmware/src/hal/pio/hal_pio.{h,cpp}`：PIOStateMachineConfig 增移位配置 + sm_exec/sm_clear_fifos/sm_restart/init_pin/sm_set_pindirs_out。
- `main_firmware/src/driver/swd/swd.{h,cpp}`：完整 SWD 编程器（PIO 传输 + DP/AP + acquire + SROM + flash 流程）。
- `main_firmware/src/config.h`：加 stdint、LED 映射与极性修正、须在 Arduino.h 前包含。
- `main_firmware/src/main.cpp`：SWD_BRINGUP_TEST 开关（当前=true，跑 acquire + RGB 识别，用于实机验证；里程碑3 整合 psoc_updater/ping-pong 时清理）。

### 里程碑3（下一步）：PSoC 固件
目标：在 PSoC capsense 示例基础上新增 SPI 从机 + ping/pong + 点 W LED(P1.6) + FW_VERSION 宏，capsense 保留，`make build` 出 hex。
- **前置核实**：cycfg_routing 是否已按 hardware 修正（MISO P1.1 已补路由？P1.0/P1.1 的 MOSI/MISO 方向）；SWD 引脚 P3.2/P3.3 未被 SPI/其它占用（用户要求 SWD 始终可用）；W LED 实际引脚（用户先前说 P1.6）。
- SCB0 SPI slave（MODE0/8bit/MSB/CS低）+ ping/pong 协议须与 RP2040 sensor_link 的 SensorFrame 帧对齐（RP2040 端已有 sensor_link.cpp）。
- 之后：RP2040 端 psoc_updater 用 SwdProgrammer + 内嵌 PSoC hex，上电比对 FW_VERSION 低则自动烧录（program_flash+verify），再联调 SPI ping/pong。


---

## /save 当前指针（Opus 4.8）— 里程碑2 完结，里程碑3 待开工

### 精确状态
- 里程碑1 RP2040 base ✅；里程碑2 RP2040 SWD 编程器 ✅（2a 实机验证通过；2b 代码完成+编译通过，按方案B 留到里程碑3 用真实 hex 验证）。
- 最近一次改动：config.h LED 映射修正（GPIO18=绿/19=红/20=蓝，共阳 active-low），`pio run` [SUCCESS] RAM7.0%/Flash4.0%。firmware.uf2 已生成。
- main.cpp 仍带 `SWD_BRINGUP_TEST=true`（现为 acquire + RGB 逐通道识别；里程碑3 整合时清理为 psoc_updater + ping/pong）。

### 恢复后立即要做（里程碑3 第一步）：PSoC 工程 recon（上次 researcher 调用被用户中断，需重发）
用 researcher 纯本地 recon `f:\mai2control\mai2control-v4\psoc_firmware\CY8C4147AZI-SensorCore`，核实 5 点：
1. cycfg_routing 里 SCB0_SPI 的 MOSI/MISO/CLK/SELECT→P1.x 映射（**重点确认 MISO 是否已补路由、P1.0 到底 MOSI 还是 MISO**）；SPI slave 配置字段（MODE0/8bit/MSB/CS低）；有无 DMA。
2. **P3.2/P3.3 仍为 SWD 未被占用**（用户硬性要求 SWD 始终可用）。
3. 状态 LED（W）引脚（用户先前说 P1.6）是否已配 GPIO 输出。
4. main.c 初始化序列与「SPI从机 init + ping/pong + 点W LED」插入点；有无既有 SPI/FW_VERSION。
5. 构建体系（Makefile APPNAME/TARGET，`make build`，hex 产物路径）。

recon 后：设计 PSoC SPI 从机 + ping/pong（帧须与 RP2040 sensor_link.cpp 的 SensorFrame 对齐）+ 点 W LED + FW_VERSION，capsense 保留，主 agent 直接落地（module-builder 不可用），`make build` 验证。再做 RP2040 psoc_updater（SwdProgrammer+内嵌hex+版本比对自动烧录）联调。

### 子代理可用性
researcher 纯本地读=可用；module-builder / implementer / researcher纯Web = 本环境 aborted。写码由主 agent 直接落地。

### 关键规格速查
- SWD 目标 IDCODE=0x0BC11477(CM0+)；silicon family=0xB5(4100S Plus)。
- SROM：SYSREQ=0x40100004/SYSARG=0x40100008/TEST_MODE=0x40030014/SRAM_PARAMS_BASE=0x20000100/KEY1=0xB6/KEY2=0xD3/SYSREQ_BIT=0x80000000/PRIVILEGED=0x10000000/SUCCESS=0xA0000000；opcode GET_ID=0/LOAD_LATCH=4/PROGRAM_ROW=6/ERASE_ALL=0xA/CHECKSUM=0xB。flash row=128B/512行/单macro/base0。
- SWD 详细设计与进度另见 `.kiro/subagent/context/milestone2_swd_pio.md`。
