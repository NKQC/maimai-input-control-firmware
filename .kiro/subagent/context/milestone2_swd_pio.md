# 里程碑2：RP2040 PIO SWD 编程器（任务名 milestone2_swd_pio）

> 本文件是主 agent 给 module-builder 的权威架构+规格；也是分步落盘恢复点。
> 目标器件：PSoC4100S Plus **CY8C4147AZI-S475**（CM0+ 内核）。
> 工程：`f:\mai2control\mai2control-v4\main_firmware`（PlatformIO, arduino-pico / pico-sdk，`pio run` 编译）。
> 用户硬性要求：**bit-bang SWD 优先用 RP2040 PIO 硬件外设**（不要纯 gpio_put/gpio_get 软翻转）。

---

## 一、任务拆分（本里程碑分 2a / 2b 两步交付）

- **任务2a（本次交付）**：PIO SWD 传输层 + DP/AP 读写原语 + acquire(Step1A) + 读 IDCODE + 读 SiliconID(Step2)。
  - 交付判据：`pio run` 编译通过；`acquire()`+`read_idcode()` 逻辑完备，可对实机 PSoC 验证（IDCODE 应为 0x0BC11477）。
- **任务2b（下一步，2a 实机验证后再做）**：SROM 流程 erase_all / checksum / program_row / verify_flash。
  - PSoC 固件 hex 尚不存在（里程碑3产出），`program_flash` 接受外部传入 buffer/回调，先不内嵌 hex。

---

## 二、硬件引脚（config.h，权威）

- `PIN_SWD_IO   = 16`（SWDIO，双向数据）
- `PIN_SWD_CLK  = 17`（SWDCLK，host 常驱动）
- `PIN_SWD_RST  = 21`（XRES，PSoC 复位，低有效；普通 GPIO 输出即可，不走 PIO）
- 全为 `constexpr uint8_t`。

---

## 三、现有可复用架构（已核实）

### HAL_PIO（src/hal/pio/hal_pio.{h,cpp}）—— 通用 PIO 抽象
- `HAL_PIO` 抽象基类 + `HAL_PIO0` / `HAL_PIO1` 单例（`getInstance()`）。
- 现有 public 接口：
  - `bool init(uint8_t gpio_pin)` —— 只 pio_gpio_init + gpio_set_function **单个**引脚。
  - `bool load_program(const pio_program_t*, uint8_t* offset)`
  - `void unload_program(const pio_program_t*, uint8_t offset)`
  - `bool claim_sm(uint8_t* sm)` / `void unclaim_sm(uint8_t)`
  - `bool sm_configure(uint8_t sm, const PIOStateMachineConfig&)`（内部 pio_get_default_sm_config + set_out/in/set/sideset/clkdiv/wrap + pio_sm_init + 可选 enable）
  - `void sm_set_enabled(uint8_t,bool)`
  - `void sm_put_blocking / bool sm_put_nonblocking / uint32_t sm_get_blocking`
  - `bool sm_is_tx_fifo_full / bool sm_is_rx_fifo_empty`
- `PIOStateMachineConfig` 字段：out_base/out_count、in_base、set_base/set_count、sideset_base/sideset_bit_count/sideset_optional/sideset_pindirs、clkdiv、wrap_target/wrap、program_offset、enabled。
- **⚠ HAL_PIO 缺 SWD 需要的原语**（见第五节，需最小扩展）：`pio_sm_exec`、清 FIFO、restart、第二个引脚的 PIO function 初始化。
- **PIO0 当前空闲**（里程碑1 main.cpp 未实例化 neopixel）。SWD 用 **HAL_PIO0**。

### NeoPixel（src/driver/neopixel 或 protocol/neopixel）—— PIO 程序加载范式模板
- 范式：**内联 uint16_t 指令数组** + `static const struct pio_program {.instructions,.length,.origin=-1}`，无 pioasm。
  ```cpp
  static const uint16_t xxx_instructions[] = { 0x6221, 0x1123, ... };
  static const struct pio_program xxx_program = { .instructions=xxx_instructions, .length=4, .origin=-1 };
  ```
- 加载流程：`load_program(&prog,&offset)` → `claim_sm(&sm)` → `sm_configure(sm,cfg)` → `sm_set_enabled`。
- FIFO：`while(sm_is_tx_fifo_full)…; sm_put_nonblocking(sm,data);` 读用 `sm_get_blocking`。

### SwdProgrammer 骨架（src/driver/swd/swd.{h,cpp}）—— 待实现
- 现状：非单例，`SwdProgrammer(io_pin,clk_pin,rst_pin)`，成员 `_io_pin/_clk_pin/_rst_pin`，`init()/update()` 空实现。
- 本项目命名规范：类内部成员/函数 `_` 前缀，对外接口不加 `_`；多用 struct/union；循环≤3层；工具函数内联高内聚；**不写测试/README**，用 `pio run` 编译验证。

---

## 四、SWD 线协议规格（Infineon 002-22325 Rev *H，已精读）

### 4.1 SWD 包（46 clocks，3 相）
- **Header 8 bit**（host 驱动，SWDIO 在 SWDCLK 下降沿变、target 上升沿采）：
  bit0=Start(1)、bit1=APnDP、bit2=RnW、bit3=A[2]、bit4=A[3]、bit5=Parity(APnDP^RnW^A2^A3 偶校验)、bit6=Stop(0)、bit7=Park(1)。**LSB first 发送**。
- **TrN1**：header 后 0.5 cycle 转向（host 释放，target 上升沿开始驱动 ACK）。
- **ACK 3 bit**（target 驱动，LSB first）：001=OK, 010=WAIT, 100=FAULT。
- **数据 32bit + parity 1bit**（LSB first）：
  - 写：ACK 后 **TrN2=1.5 cycle**（双方都不驱动），host 再驱动 wdata[0..31]+even parity。
  - 读：target 紧接 ACK 后驱动 rdata[0..31]+parity，然后 TrN2 转回。
- **turnaround 通用**：第一个 TrN=0.5 周期，第二个 TrN=1.5 周期；空闲两包间 host 给 ≥3 个 SWDIO=0 dummy clock。
- **时序**：host 在 SWDCLK 下降沿读写 SWDIO；target 在上升沿读写。min SWDCLK 1.5MHz（仅 acquire 400µs 窗口需要），max 14MHz。本工程目标 SWDCLK ≈ 2MHz（安全满足 acquire 窗口，也可更高）。

### 4.2 LineReset / 切换
- `SWD_LineReset`：≥50 clock SWDIO=1，末尾 ≥1 clock SWDIO=0。（本器件只支持 SWD，无 JTAG，不需要 JTAG→SWD 魔术字；仅需 line reset + 读 IDCODE）
- 读 parity 校验：Read_DAP 收到 OK(001) 后，对 data32 逐位异或与 parity 比较，不符则视为失败。

### 4.3 DP/AP 寄存器（APnDP,ADDR）
- DP: IDCODE{0,00b,R}, ABORT{0,00b,W}, CTRL/STAT{0,01b,R/W}, SELECT{0,10b,W}
- AP: CSW{1,00b}, TAR{1,01b}, DRW{1,11b}
- 伪码 Register 结构：`struct { uint8_t APnDP; uint8_t Addr; }`；如 TAR={1,1}, DRW={1,3}, IDCODE={0,0}。

### 4.4 WriteIO / ReadIO（经 TAR+DRW 访问任意 32bit CPU 地址）
- `WriteIO(addr,data)` = Write_DAP(TAR,addr) + Write_DAP(DRW,data)。
- `ReadIO(addr,out data)` = Write_DAP(TAR,addr) + Read_DAP(DRW,…) **两次**（AP 读有一拍延迟，第一次丢弃，第二次取值）。返回全 ACK=OK 才算成功。

---

## 五、Acquire（Step 1A，CM0+ 版，已按目标器件修正）

```
ToggleXRES();                       // XRES 拉低≥某窗口再拉高（低有效）；见下时序
do { SWD_LineReset(); ack=Read_DAP(IDCODE,out ID); }
   while(ack!=OK && elapsed<5.0ms);
if(elapsed>=5.0ms) FAIL;
if(ID != 0x0BC11477) FAIL;          // ★CM0+ 目标 ID，非伪码里的 0x0BB11477(CM0)
Write_DAP(CTRL/STAT, 0x54000000);
Write_DAP(SELECT,    0x00000000);
Write_DAP(CSW,       0x00000002);   // 32-bit 访问（CM0+ 也可 0x03000042；先用 0x00000002 与 Step1A 一致，编译后实机验证）
WriteIO(TEST_MODE=0x40030014, 0x80000000);
ReadIO(0x40030014, out st); if((st&0x80000000)!=0x80000000) FAIL;
do { ReadIO(CPUSS_SYSREQ=0x40100004,out st); st&=0x10000000; } while(st!=0 && elapsed<1000ms);
if(elapsed>=1000ms) FAIL;
// CY8C4147 (4100S Plus) 无需 SET_IMO_48MHz（表1-1）→ 跳过
return PASS;
```
- acquire 时序窗口：XRES toggle 后，boot code<1ms + <4ms，之后 CPU 等 SWD 连接 400µs。因此必须 XRES 拉高后**尽快循环发 line reset+读 IDCODE**（5ms 内）。PIO 已上电配好、CPU 只推 FIFO，能满足。

---

## 六、SROM 机制与常量（第七节任务2b 用，先记录齐）

- `CPUSS_SYSREQ = 0x40100004`，`CPUSS_SYSARG = 0x40100008`，`TEST_MODE = 0x40030014`，`SRAM_PARAMS_BASE = 0x20000100`。
- `SROM_KEY1 = 0xB6`，`SROM_KEY2 = 0xD3`。
- `SROM_SYSREQ_BIT = 0x80000000`，`SROM_PRIVILEGED_BIT = 0x10000000`，`SROM_STATUS_SUCCEEDED = 0xA0000000`（判据 `(statusCode & 0xF0000000)==0xA0000000`）。
- SROM opcode：GET_SILICON_ID=0x00, LOAD_LATCH=0x04, PROGRAM_ROW=0x06, ERASE_ALL=0x0A, CHECKSUM=0x0B, WRITE_PROTECTION=0x0D, WRITE_SFLASH_ROW=0x18。
- **PollSROMStatus**：轮询 ReadIO(CPUSS_SYSREQ) 直到 `(SYSREQ_BIT|PRIVILEGED_BIT)` 位清零（timeout 1s），再 ReadIO(CPUSS_SYSARG) 判 `0xA0000000`。
- **Get Silicon ID（Step2）**：`Params=(KEY1)|((KEY2+0x00)<<8)`；WriteIO(SYSARG,Params)；WriteIO(SYSREQ, SYSREQ_BIT|0x00)；Poll；再 ReadIO(SYSARG,out p0)+ReadIO(SYSREQ,out p1)：
  siliconID[0]=(p0>>8)&0xFF(Hi), [1]=p0&0xFF(Lo), [2]=(p0>>16)&0xFF(Rev), [3]=p1&0xFF(Family)。
- **Flash 几何**：row=128 bytes，rows=512，macro=1，flash base=0x00000000，总 64KB。protection bytes/macro=512/8=64。chip-prot 读地址 `SFLASH_MACRO_0 + RowSize - 4`（RowSize=128）。
- **注意 SYSARG 写法差异**：ERASE_ALL/LOAD_LATCH/PROGRAM_ROW 走 SRAM_PARAMS_BASE（把 Params 写到 SRAM，再 WriteIO(SYSARG, SRAM_PARAMS_BASE)）；GET_SILICON_ID/CHECKSUM/SET_IMO 直接 WriteIO(SYSARG, Params)。严格按 §4.3~4.9 伪码区分。

---

## 七、PIO SWD 传输层设计（任务2a 核心，必须 PIO）

参考成熟设计：**picoprobe / debugprobe 的 `probe.pio`（Raspberry Pi 官方，Apache-2.0）**，是 RP2040 用 PIO 跑 SWD 的权威实现。module-builder 可 web_fetch 其 `probe.pio` 汇编产物作为 PIO 指令来源（**须在代码注释注明来源与 Apache-2.0 出处**），或据下述设计自行汇编。

### 设计要点
- 1 个 SM。**sideset(1bit)=SWDCLK**（每指令产生半个/整个时钟沿）；**SWDIO** 同时作 out/in/set 引脚（out_base=in_base=set_base=PIN_SWD_IO）。
- 两个 public 入口：`offset_write` 与 `offset_read`。CPU 通过 `sm_exec(pio_encode_jmp(offset+entry))` 切换。
- 传输原语：
  - `_seq_out(uint32_t data, uint8_t nbits)`：设 pindirs=out（`sm_exec(pio_encode_set(pio_pindirs,1))` 或程序内 SET），推 (nbits-1) 计数与 data，SM 在 SWDCLK 上驱动 SWDIO，LSB first。
  - `_seq_in(uint8_t nbits) -> uint32_t`：设 pindirs=in，推计数，SM 采样 nbits 自动 push 到 RX FIFO，`sm_get_blocking` 取。
  - `_turnaround(cycles)`：pindirs=in 空转 N 个时钟。
- clkdiv：使 SWDCLK≈2MHz（SM 频率 = SWCLK×每bit指令数）。系统时钟 125MHz → 据每 bit 指令数算 clkdiv。
- Write_DAP / Read_DAP 用上述原语拼：header(8) → TrN(0.5→按整周期近似 1) → ACK(3) → [写: TrN(2) + data32+par] / [读: data32+par + TrN(1)] → 3 dummy idle clock。**TrN 半周期用整周期近似实现即可（picoprobe 即如此），保证功能正确。**

### HAL_PIO 最小扩展（通用、可复用；在抽象基类 + HAL_PIO0 + HAL_PIO1 三处同步加）
- `void sm_exec(uint8_t sm, uint16_t instr)` → `pio_sm_exec(pioX, sm, instr)`
- `void sm_clear_fifos(uint8_t sm)` → `pio_sm_clear_fifos`
- `void sm_restart(uint8_t sm)` → `pio_sm_restart` + `pio_sm_clkdiv_restart`（如需）
- `void init_pin(uint8_t gpio)` → `pio_gpio_init(pioX,gpio)+gpio_set_function(gpio,GPIO_FUNC_PIOx)`（供 SWDCLK 第二引脚；或让 SwdProgrammer 对 IO 用 init()、对 CLK 用 init_pin()）
- 均为薄封装，保持 SWD 只依赖 HAL_PIO 抽象，不直接触碰 pico-sdk。

### SwdProgrammer 建议 API（保持 driver 层，无上层依赖）
```cpp
class SwdProgrammer {
public:
  SwdProgrammer(uint8_t io_pin, uint8_t clk_pin, uint8_t rst_pin);
  bool init();                 // 配 PIO(HAL_PIO0)+RST GPIO；加载 SWD PIO 程序、claim/config/enable SM
  void update();               // 保留（当前空或状态机推进）
  bool acquire();              // Step1A：XRES toggle + line reset + 读 IDCODE + enter test mode + poll privileged
  uint32_t read_idcode();      // 单独读 IDCODE（调试用）
  bool check_silicon_id(uint32_t expected /*或输出4字节*/); // Step2 via SROM
  // 2b: bool erase_all(); bool program_flash(const uint8_t* data,uint32_t len); bool verify_flash(...); ...
private:
  // 传输原语
  void _line_reset();
  uint8_t _swd_write(uint8_t apndp,uint8_t addr,uint32_t data);   // 返回 ACK
  uint8_t _swd_read (uint8_t apndp,uint8_t addr,uint32_t* data);  // 返回 ACK，含 parity 校验
  bool _write_io(uint32_t addr,uint32_t data);
  bool _read_io (uint32_t addr,uint32_t* data);   // 内含两次 DRW 读
  bool _poll_srom_status();
  void _seq_out(uint32_t data,uint8_t nbits);
  uint32_t _seq_in(uint8_t nbits);
  // 成员
  uint8_t _io_pin,_clk_pin,_rst_pin;
  HAL_PIO* _pio; uint8_t _sm; uint8_t _prog_offset;
  bool _ready;
};
```
- 常量集中在 swd.cpp 顶部 `constexpr`（IDCODE=0x0BC11477、TEST_MODE、CPUSS_*、KEY1/2、opcode 等），或放 config.h（若不想污染全局引脚区，放 swd.cpp 局部）。
- main.cpp 集成：本次 2a **可暂不接入 main.cpp 主流程**（只需编译通过 + 提供接口）；实机验证时再在 main setup/loop 里 acquire()+read_idcode() 并用 RGB 灯指示成功/失败（沿用 led_service）。

---

## 八、验证

- 编译：`main_firmware` 下 `pio run`（PowerShell；`Select-Object -Last 60` 看尾部错误）。框架 .o 已缓存，失败很快。
- **禁止**擅自烧录。实机验证 uf2→G:/ 前，主 agent 会用提问工具让用户进 BOOTSEL 并接好 PSoC。

---

## 九、分步落盘约定（module-builder）

每约 5 步更新本文件「## 进度」小节（在文件末尾追加/更新）：记录已建/改文件、当前编译状态、下一步。任务中断重派时先读本文件恢复。

## 进度

### 2a 已由主 agent 直接落地并编译通过（module-builder 在本环境 aborted 不可用）
已建/改文件：
- `src/hal/pio/hal_pio.h` / `hal_pio.cpp`：PIOStateMachineConfig 增加移位配置字段（out_shift_right/autopull/pull_threshold/in_shift_right/autopush/push_threshold，默认值等价 pico 默认，NeoPixel 不受影响）；sm_configure 增加 sm_config_set_out_shift/in_shift；新增通用方法 sm_exec / sm_clear_fifos / sm_restart / init_pin / sm_set_pindirs_out（PIO0+PIO1+抽象基类三处）。
- `src/driver/swd/swd.h` / `swd.cpp`：完整实现。内联 11 条 PIO 机器码（源自 raspberrypi/debugprobe probe.pio，MIT，已注明）。命令字格式 |13:9 入口|8 Dir|7:0 count-1|。原语 _seq_out/_seq_in/_turnaround/_line_reset；_swd_write/_swd_read（even parity 校验 + WAIT 重试 4 次 + 各相 turnaround）；_write_io/_read_io（AP 读两次弃首）；_poll_srom_status；acquire()（XRES toggle+5ms 内 line reset 读 IDCODE==0x0BC11477+写 CTRL/STAT 0x54000000/SELECT 0/CSW 0x00000002+进 Test Mode 0x40030014+poll PRIVILEGED 0x10000000 1000ms；CY8C4147 跳过 SET_IMO）；read_idcode()；read_silicon_id()（SROM GET_SILICON_ID）。SWCLK≈2MHz（clkdiv=sysclk/(SWCLK*4)）。用 HAL_PIO0，SWDIO=16/SWDCLK=17(连续,side-set=clk)/XRES=21。
- `src/config.h`：加 `#include <stdint.h>` + 注释（须在 Arduino.h 前包含，避免 PIN_SPI1_* 宏冲突）。
- `src/main.cpp`：新增 `SWD_BRINGUP_TEST`(=true) 开关。启动 swd_bringup()：init→read_idcode→acquire→read_silicon_id；loop 用 RGB 指示（青=acquire全成功/绿=IDCODE匹配但进TestMode失败/红闪=传输失败）+ 每秒经 USB CDC 打印诊断。config.h 移到 Arduino.h 之前。

编译：`pio run` [SUCCESS] RAM 7.0% / Flash 4.0%，firmware.uf2 已生成。

### ⚠ 待实机验证（阻塞在用户操作）
需用户把 RP2040 进 BOOTSEL（G:/ 盘）→ 复制 `.pio/build/pico/firmware.uf2` 到 G:/ → 接好 PSoC SWD(16/17/21) → 观察 RGB 灯 + USB CDC 串口输出（`SWD init= idcode=0x.. ok= acquired= silicon=0x..`）。
- 期望：idcode=0x0BC11477，acquired=1，silicon 高字节/family 属 CY8C4147(4100S Plus)。
- 若红闪/idcode 全 0：传输层问题（首查：CSW 试 0x03000042；turnaround 相位；SWDIO 上拉；接线/电平转换）。
- 潜在风险点（实机若失败优先排查）：① turnaround 用整周期近似是否导致 ACK 采样错位；② AP 读两次弃首是否够（或需读 RDBUFF）；③ acquire 的 XRES 窗口/CSW 值。

### 2a 实机验证 ✅ 通过（用户实测）
CDC: `acquired=1 silicon=0x000012B5` → family=0xB5(PSoC4100S Plus/CY8C4147)，Rev=0x12。传输层/acquire/SROM 全部工作。
根因修复：SM 必须 sm_exec(pio_encode_jmp(_offset+get_next_cmd)) 后再 enable；另加 JTAG→SWD 0xE79E 切换；LED 改共阳(active-low)；bring-up 从 acquire 内部取 last_idcode。

### 2b 已实现并编译通过（RAM7.0%/Flash4.0%），待实机验证
- erase_all()（OPEN 模式路径，参数走 SRAM_PARAMS_BASE）
- checksum_all()（Row 0x8000 全 flash，28-bit）
- program_row()（LOAD_LATCH 装 latch + PROGRAM_ROW；单 macro→macro0）
- program_flash(data,len)（按 128B 行循环）
- verify_flash(data,len)（AHB 直读 0x0 起比对）
- 常量：LOAD_LATCH=0x04/PROGRAM_ROW=0x06/ERASE_ALL=0x0A/CHECKSUM=0x0B，SRAM_PARAMS_BASE=0x20000100，ROW_SIZE=128，ROWS_PER_MACRO=512。CY8C4147 不在 ProgramRow 缺陷器件清单，跳过 workaround。

### 待用户决策：2b 实机验证方式
(A) 现在破坏性自测：erase_all → checksum(应=特权行基线) → program_flash(测试图案) → verify_flash。会擦掉 PSoC 出厂 capsense 固件（里程碑3 反正要换成我们的固件）。需重新烧 RP2040 测试版 + 用户确认可擦除。
(B) 推迟到里程碑3：用真实 PSoC 固件 hex 走 program_flash+verify，一并联调 ping/pong。

### 里程碑2 状态：2a 完成(实机验证)，2b 代码完成待实机；里程碑2 主体达成。
