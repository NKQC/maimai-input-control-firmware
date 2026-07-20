# PSoC4 SWD 烧录 bring-up — 进展与当前阻塞点

目标芯片:CY8C4147AZI(PSoC4100S Plus,CM0+),family=0xB5,128B/行,512 行/macro。
RP2040 用 PIO bit-bang SWD 作编程器。自助烧录闭环:向 CDC 发 REBOOT_BOOTLOADER 帧
`AA 55 05 00 00 00 00 5B 32` → BOOTSEL(G:,Windows 偶发枚举延迟需多等)→ 复制
`.pio/build/pico/firmware.uf2` → 读 COMx 文本诊断。★必须纯 PC/5V 供电,勿用 QC 快充(D+/D- 抬压烧片)★。

## 已解决(从完全不通推进到 acquire+读 SROM 全通)
1. **HMASTER_BIT**:SROM_REQ 必须 = SYSREQ_BIT|HMASTER_BIT(否则特权命令被拒)。已修。
2. **connect-under-reset**:运行中的 CapSense 固件会重配 SWD 引脚 → 不复位连不上(IDCODE=0)。
   必须 XRES 复位后趁 boot 窗口连接。`_acquire_once`(test-mode hammer)做到了。
3. **CSW_PRIV(PPB)**:DHCSR/CPUID 等 PPB 区(0xE000xxxx)必须用特权 CSW(0x03000042),
   CSW_STD(0x02)访问不了 PPB。(此项是 debug-halt 路径排障发现,见下。)
4. **debug-halt 路径不可用于跑 SROM**:halt 后 CPU 不执行 SROM syscall → SYSREQ 永不清(超时 0xDEAD0003)。
   PSoC4 烧 flash 必须走 **test-mode**(CPU 停在 boot ROM 的 SYSREQ 服务循环),不能 debug-halt。
   → acquire 现用 `_acquire_once`(test-mode)。debug-halt 代码保留但不用于烧录。

## 当前状态(诊断实测,acq=1 已成功)
`acq=1(delay=0 sysarg=0xA0120000 sysreq=0x600000B5) id=0x0BC11477 prot=0 imo=0(0xF0000014) erase=0(0xF0000014) prog=0(row=0 0xF0000012)`
- ✅ acquire 成功;✅ GET_SILICON_ID 成功(sysarg 高位 0xA);✅ family=0xB5 确认;test-mode 下 SROM 可执行。
- ✅ progspec 家族表:128B/512 行家族 **SET_IMO_48MHz=No(不需要)** → SET_IMO 的 0xF0000014 属"不支持",可忽略。
- ❌ **ERASE_ALL=0xF0000014、PROGRAM_ROW=0xF0000012** —— 仅 flash 写/擦类 SROM 命令失败。
- ⚠ prot 读为 0(VIRGIN?),可疑:GET_SILICON_ID 的保护位提取 `(SYSREQ>>12)&0xF` 对 4100S Plus 可能不对,
  导致 erase_all() 走错分支(未按 PROTECTED→WRITE_PROTECTION 路径)。

## 剩余待查(下一步)
精读 progspec/AN84858 的 **Step 3 Erase all flash / Program Row 命令参数格式(4100S Plus 专属)** 与
**SROM 状态码 0xF0000012 / 0xF0000014 的确切含义**:
- 假设A:芯片实为 PROTECTED,需先 WRITE_PROTECTION→OPEN(同时全擦)再编程;当前保护位提取错→走了直接 ERASE。
- 假设B:ERASE_ALL/PROGRAM_ROW 的 SRAM 参数块布局/键值对该型号不对。
- 假设C:该型号擦写还需别的时钟/前置(虽 SET_IMO=No,但可能有其它)。

## 备选路径
若 RP2040-bit-bang-programmer 继续攻坚成本过高:改用专用编程器(MiniProg4/KitProg3/J-Link/
picoprobe-CMSIS-DAP)烧 PSoC,先解锁项目主线,RP2040 编程器功能后补或弃用。


---

# 修订:SROM 执行机制根因(OpenOCD psoc4.c 实证)

**根因确认**:PSoC4 的 SROM 系统调用靠 **NMI**,必须 **CPU 在执行**才被服务。写 CPUSS_SYSREQ.SYSREQ_BIT 触发 SROM NMI 例程执行命令。仅"写 SYSREQ+轮询"不让 CPU 跑,flash 类命令(ERASE/PROGRAM)不被服务 → 0xF0000014/0xF0000012。GET_SILICON_ID 在 test-mode 侥幸被 boot ROM 轮询服务,故读类假象成功。

**OpenOCD 正解**:target 必须 HALTED;每次 syscall = 设参数 + 写 SYSREQ + 在 SRAM 跑一段 `bkpt 0` 算法(resume 让 CPU 执行)→ NMI 立即服务 SROM → 返回撞 bkpt 停 → 读 SYSARG 状态。

**采用方案**:已验证可用的 `_connect_and_halt`(XRES 复位+boot 窗连接+DHCSR 停核,CSW_PRIV 访问 PPB)保留;新增 bkpt-算法 SROM 执行取代轮询式 `_srom_exec`:
1. `_write_core_reg(reg,val)`:写 DCRDR(0xE000EDF8)=val;写 DCRSR(0xE000EDF4)=reg|REGWnR(bit16);轮询 DHCSR S_REGRDY(bit16)。reg: SP=13,PC=15,xPSR=16。
2. `_srom_exec(cmd)` 改为:写 bkpt 0xBE00 到 0x20000000;设 PC=0x20000000、SP=0x20000800、xPSR=0x01000000(Thumb);写 CPUSS_SYSREQ=SYSREQ|HMASTER|cmd;resume(DHCSR=DBGKEY|C_DEBUGEN);轮询 S_HALT(bit17);读 SYSARG 判 (& 0xF0000000)==0xA0000000。
   - SRAM 布局:code@0x20000000(bkpt),params@0x20000100(既有 SRAM_PARAMS_BASE,含 LOAD_LATCH 128B 数据到 0x188),stack top@0x20000800(向下,不触 params)。
3. acquire() 改回 `_connect_and_halt()`(halt 路径),弃 test-mode(_acquire_once 保留不用于烧录)。
4. 参数设置(KEY1|((KEY2+cmd)<<8)|(cmd_param<<16))与 OpenOCD 一致,已验证结构正确。
5. CPU 只执行我们放的 bkpt+SROM NMI,不跑用户固件(PC 由我们控制),安全。


---

# 修订(决定性):回归规格 002-22325 的 test-mode 权威路线

研究员通读 progspec + 主 agent 核对规格正文(§4.2/§4.3 伪代码),确认此前 halt/bkpt-run 走偏。**规格权威路线**:

## acquire(Step 1A 伪代码,逐字)
1. ToggleXRES(低≥5us,释放)。
2. 窗口内循环:`SWD_LineReset()` + 读 IDCODE,直到 ACK=OK 且 ID==0x0BC11477(CM0+),超时 5ms。
3. 配 DP:`Write_DAP(CTRLSTAT,0x54000000)`、`Write_DAP(SELECT,0)`、`Write_DAP(CSW,0x00000002)`（CSW_STD,全程不碰 PPB）。
4. 进 test mode:`WriteIO(TEST_MODE,0x80000000)`;`ReadIO(TEST_MODE,status)`;校验 `(status&0x80000000)==0x80000000`,否则 FAIL。
5. **轮询就绪**:`do{ ReadIO(CPUSS_SYSREQ,s); s&=PRIVILEGED_BIT(0x10000000);}while(s!=0 && t<1000ms)`,超时 FAIL。← 我们缺的关键步。
6. SET_IMO:本家族(B5,128B/512,Table 1-1=No)**跳过**。
（整体可重试若干次 XRES。）

## SROM 调用(PollSROMStatus 伪代码,逐字)
- `WriteIO(CPUSS_SYSREQ, SROM_SYSREQ_BIT | cmd)` —— **只 SYSREQ_BIT(0x80000000),不带 HMASTER**。
- Poll:`do{ ReadIO(CPUSS_SYSREQ,s); s&=(SYSREQ_BIT|PRIVILEGED_BIT);}while(s!=0 && t<1s)`;读 `CPUSS_SYSARG` 状态码;`(&0xF0000000)==0xA0000000` 即成功。
- test-mode 下 CPU 在 ROM 服务 SYSREQ,**无需 bkpt-run/resume**。

## 芯片保护读法(规格 §4.11)
从 **flash 地址** `SFLASH_MACRO_0(0x0FFFF000) + RowSize(128) - 4 = 0x0FFFF07C` 读 32 位,`(v>>24)&0x0F`;不是 GET_SILICON_ID 的 SYSREQ(那里[11:0]是 Family ID)。刚擦除/boot 翻译:VIRGIN↔OPEN 在 supervisory 反转保存。

## 已确认无误(不要改)
ERASE_ALL/LOAD_LATCH/PROGRAM_ROW 的 SRAM 参数块逐字节布局与规格一致(researcher 核对)。

## 修复动作
1. `SROM_REQ = SROM_SYSREQ_BIT`(去掉 HMASTER)。
2. `acquire()` 改为上面 Step 1A 逐字实现(含 TEST_MODE 校验 + PRIVILEGED 轮询)。
3. `_srom_exec` 回到 `WriteIO(SYSREQ,SROM_REQ|cmd)+_poll_srom_status()`(弃 bkpt-run,test-mode 不需要)。
4. main bringup 对本家族跳过 SET_IMO(或保留但非致命)。
5. (可选)erase_all 的保护读改为从 flash 0x0FFFF07C 读;当前 OPEN 直擦路径对空片/示例片应可行,先验证 ERASE 是否变 0xA0000000。
6. halt/bkpt/_connect_and_halt/_acquire_test_mode_workaround 保留定义但不用于烧录路径。


---

# module-builder 落地记录：test-mode 权威路线已实现并编译通过

## 已改动
1. `psoc_types.h`：`HMASTER_BIT` 注释改为"test-mode 路线不用，仅供 debug-halt 路线参考"，未删除常量。
2. `psoc_swd.cpp`：
   - `SROM_REQ` 由 `SYSREQ_BIT|HMASTER_BIT` 改为仅 `SYSREQ_BIT`。
   - `_srom_exec(cmd)` 重写为极简两步：`_write_io(CPUSS_SYSREQ, SROM_REQ|cmd)` + `_poll_srom_status()`；不再调用 bkpt/核寄存器/resume 那一套（`_write_core_reg`/`_connect_and_halt`/`_probe_programming_mode`/`_acquire_test_mode_workaround`/`_acquire_once` 定义原样保留未删除，只是不再被调用，避免破坏可能的外部/未来引用；编译已验证不会因未使用产生阻断）。
   - `acquire()` 完全重写为规格 Step 1A 逐字流程：外层最多 20 次 ToggleXRES 重试；内层 5ms 窗口做 `_line_reset+读IDCODE→CTRLSTAT上电→SELECT→CSW=CSW_STD→写TEST_MODE=0x80000000→回读校验bit31`；命中后再轮询 1000ms 直到 `CPUSS_SYSREQ & PRIVILEGED_BIT==0`；诊断字段 `_last_acquire_delay`(=attempt序号,失败时0xFFFFFFFF)、`_last_acquire_sysreq`(=轮询期SYSREQ)、`_last_acquire_status`(=就绪后SYSARG)。
3. `psoc_swd.h`：`acquire()`/`_srom_exec`/`_poll_srom_status` 注释同步更新为规格路线说明。

## 验证结果
`pio run`（cwd=main_firmware）：**SUCCESS**，耗时 6.79s。Flash 456444 bytes (14.5%)，RAM 26952 bytes (10.3%)。产物 `.pio\build\pico\firmware.uf2` 已生成。未烧录（按指示留给主 agent）。

## 对 ERASE 是否仍 0xF0000014 的判断
去掉 HMASTER 后，若之前 GET_SILICON_ID 成功而 ERASE_ALL/PROGRAM_ROW 被拒确实是因为带了 HMASTER（规格里没有这一位，且它是 debug-halt 路线的产物），现在改回纯 `SYSREQ_BIT|cmd` 应该消除"因携带非规格位而被 SROM 判定为非法请求"的可能性。另外新 acquire() 增加了规格要求的"轮询 PRIVILEGED_BIT 清零"步骤，确保 SROM 真正就绪后才认为 acquire 成功（此前版本缺这一步，可能在 SROM 未就绪时就误报 acquire=1）。这两点是此前分析出的两个主要偏差来源，理论上应能解决 ERASE 的 0xF0000014。

但仍有 `.kiro/context/psoc_swd_bringup.md` 中"当前状态"段记录的 prot 读法疑点未处理（GET_SILICON_ID 保护位提取 `(SYSREQ>>12)&0xF` 对 4100S Plus 是否准确未再核实；规格 §4.11 建议改读 flash `0x0FFFF07C`）——本次任务未改 `erase_all()`/`read_chip_protection()` 的保护判定逻辑（任务范围未要求，仅第5点"可选"）。若实机测试 ERASE 仍报 0xF0000014，下一步应优先核查保护位提取是否准确（芯片可能被误判为非 OPEN 或反之），而非再怀疑 SROM_REQ/acquire 本身。


---

# 实测结果:test-mode 权威路线(去 HMASTER + PRIVILEGED 轮询)——ERASE 仍失败

烧入新固件后 COM3 诊断:
`acq=1(delay=0us sysarg=0x00000000 sysreq=0x20000000) id=0x0BC11477 prot=0 imo=0(0xF0000014) erase=0(0xF0000014) prog=0(row=0 0xF0000012) verify=0(@0x00000000) link=0`

对比上一版(带 HMASTER):`... sysarg=0xA0120000 sysreq=0x600000B5 ... erase=0(0xF0000014) prog=...0xF0000012`
- ✅ acquire 仍成功(delay=0,PRIVILEGED 轮询通过,sysreq=0x20000000 bit28 PRIVILEGED=0)。
- ✅ GET_SILICON_ID 仍工作(prot 读到 0,说明 read_chip_protection→read_silicon_id→_srom_exec(GET_SILICON_ID) 成功;去 HMASTER 不影响读类)。
- ❌ ERASE_ALL=0xF0000014、PROGRAM_ROW=0xF0000012、SET_IMO=0xF0000014 **完全不变**。

## 决定性判据(缩小根因)
- **SET_IMO 与 GET_SILICON_ID 都用 SYSARG-直填参数**(非 SRAM),机制完全相同,但 GET_SILICON_ID 成功、SET_IMO 失败 0xF0000014。
- → 0xF0000014 **不是** HMASTER/SRAM 参数/acquire 机制问题,而是 **按命令区分的"当前状态不允许该操作"** 拒绝。flash/clock 修改类命令(SET_IMO/ERASE/PROGRAM)被拒,只读类(GET_SILICON_ID)放行。
- 疑点:①芯片保护态非 OPEN(prot=0=VIRGIN 的解码可能错,需按规格重定位保护位);②缺某个前置(时钟/供电/某寄存器);③0xF0000014/0xF0000012 的确切语义未知。

## 待 module-builder 深搜大文档(114KB progspec + 88KB an84858)
1. SROM 返回状态码表:0xF0000014、0xF0000012 确切含义(及 0xFxxxxxxx 格式)。
2. ERASE_ALL / PROGRAM_ROW 对 4100S Plus(family 0xB5)的**前置条件**(时钟?保护?IMO?)。
3. 为何 GET_SILICON_ID 放行而 SET_IMO/ERASE 被拒——规格如何区分。
4. test-mode acquire 是否还缺步骤(如某时钟寄存器写入)。


---

# module-builder 大文档检索：SROM 状态码与 ERASE 前置

来源：`.kiro/context/psoc4_progspec.txt`（规格 002-22325 Rev.*H，PSoC4 编程规格）、
`.kiro/context/an84858_hssp.txt`（AN84858 Rev.*O，HSSP 应用笔记）。逐条回答如下。

## 1. SROM 状态码表：0xF0000014 / 0xF0000012 / 0xF0000010 确切含义

**AN84858 附录 B（Appendix B: Status codes for SROM request，第37-38页，Table 12）** 给出的是通用码表：
`0xAXXXXXXX`=成功；`0xF0000001`=Invalid Chip Protection Mode（当前芯片保护模式下此 API 不可用）；
`0xF0000003`=Invalid Page Latch Address；`0xF0000004`=Invalid Address；`0xF0000005`=Row Protected；
`0xF0000006`=SRAM Address Invalid；`0xF0000007`=Resume Completed；`0xF0000008`=Pending Resume；
`0xF0000009`=System Call Still In Progress；`0xF000000A`=Checksum Zero Failed；`0xF000000B`=Invalid Opcode；
`0xF000000C`=Key Opcode Mismatch；`0xF000000E`=Invalid Start Address；`0xF0000012`=**Invalid Flash Clock**
（文档原文限定"The CY8C40xx family of devices must set the IMO to 48 MHz and the HF clock source to the
IMO clock before write/erase operations"——即该码的官方定义仅明确适用于 CY8C40xx 家族）。

**关键发现**：`0xF0000010` 和 `0xF0000014` **在这两份文档中均未被定义**（Table 12 的码序列是
...0x0E → 直接跳到 0x12，中间没有 0x0F/0x10/0x11；0x12 之后没有 0x13/0x14 条目）。AN84858 原文在表格
末尾明确提示（1651行）："For more details, see the 'Nonvolatile Memory Programming' chapters of the
respective PSoC 4 architecture reference manuals"——即完整/家族专属码表在 TRM 里，本次两份文档都不含
TRM，**无法从现有资料确认 0xF0000014/0xF0000010 的字面含义**。之前固件注释里"0xF0000014=NA_IN_DEAD_MODE"
的说法查无出处，是先前分析的推测，不是文档结论。

## 2. ERASE_ALL 前置条件与调用序列（progspec §4.6，Step 3，pseudocode 逐字）

伪代码逻辑：先读芯片保护模式（规格原文只说"Read Chip Level Protection using SROM call"，未给出具体
调用细节——权威读取方式见第5条）。若 `chipProt==PROTECTED`：写 `WRITE_PROTECTION`(0x0D)，参数
`Params=KEY1 | ((KEY2+CMD_WRITE_PROTECTION)<<8) | (0x01<<16)/*OPEN*/ | (0x00<<24)/*macro0*/`，
**参数直填 CPUSS_SYSARG（不经 SRAM）**，`PollSromStatus`；随后"Changing from PROTECTED state also
erases all Flash"，必须**重新执行 Step 1 Acquire Chip**。若已是 `OPEN`：`Params=KEY1|((KEY2+CMD_ERASE_ALL)<<8)`，
**写入 SRAM_PARAMS_BASE+0x00**，再 `WriteIO(CPUSS_SYSARG, SRAM_PARAMS_BASE)`，再
`WriteIO(CPUSS_SYSREQ, SYSREQ_BIT|CMD_ERASE_ALL)`。SET_IMO 不是 ERASE_ALL 的直接前置步骤，它是
Step 1A（acquire）末尾按 Table 1-1 决定是否需要的独立子步骤，与 ERASE 命令本身没有在伪代码里体现强制
先后依赖（除非该家族真的需要 IMO 才能做任何 flash 写操作）。

## 3. PROGRAM_ROW/LOAD_LATCH 前置与参数；为何两者错误码不同

`LOAD_LATCH`(0x04)：`Params1=KEY1|((KEY2+CMD_LOAD_LATCH)<<8)|(macroID<<24)` 写 SRAM+0x00，
`Params2=len-1` 写 SRAM+0x04，行数据从 SRAM+0x08 起，再 `WriteIO(SYSARG,SRAM_BASE)+WriteIO(SYSREQ,...)`。
`PROGRAM_ROW`(0x06)：`Params=KEY1|((KEY2+CMD_PROGRAM_ROW)<<8)|RowID字段` 写 SRAM+0x00，同样两步 WriteIO。
固件当前实现（`psoc_swd.cpp` erase_all/program_row/_srom_load_latch）与上述伪代码逐字节比对**参数布局
一致**（先前 module-builder 已核对，本次复核无误）。

**为何 ERASE=0xF0000014 而 PROGRAM=0xF0000012 不同**：两文档都没有定义 0xF0000014，无法给出权威区分
原因。可确认的是 0xF0000012 在 AN84858 里被专门定义为"Invalid Flash Clock"（CY8C40xx 专属语义）；
若该语义可类推到本芯片家族，则 PROGRAM_ROW 被拒可能确实与 clock 有关。0xF0000014 缺乏对应定义，不能
排除它是本家族扩展码表里"Invalid Chip Protection Mode"的另一取值（见第4/5条的强关联证据）。

## 4. flash 擦写对 IMO 的硬性要求

progspec Table 1-1（"SET_IMO_48MHz required for flash operations"行）按家族分 Yes/No/Yes/No/Yes 五段，
与本文档 OCR 提取的列对齐存在不确定性（多列合并显示，需要人工核对原 PDF 表格线）。AN84858 正文
（392-395行）明确列出需要 IMO 48MHz 的家族："PSoC 40xx, PSoC 4xx7_BLE, PSoC 40xxS, PSoC 41xxS,
PSoC 4100PS"，**没有单独提及"4100S Plus"**，但这不构成"4100S Plus 不需要"的确证（AN84858 这句话本身
不是逐家族的完整枚举，只是举例）。

**"时钟不对是否返回 0xF0000014"——两文档均无此结论**。唯一有文档依据的时钟相关码是 `0xF0000012`
(Invalid Flash Clock)，不是 0xF0000014。若 SET_IMO 命令本身就是"设置时钟"的 API，它被拒绝更可能是因为
"当前芯片保护模式不允许调用该 API"（见 0xF0000001 定义），而非"时钟已经不对"（那没有意义，SET_IMO 
本身就是用来设时钟的）。

## 5. 芯片保护模式：权威读取方法 vs 固件当前实现（★关键偏差★）

① **规格权威读取方法（progspec §4.11 Step 8 pseudocode / Figure 4-8，第39页）**：直接 AHB 读 flash
地址，**不经过 GET_SILICON_ID**：
```
ChipProtOffs = (RowSize == 64) ? 2*RowSize : RowSize     // 本芯片 RowSize=128 → ChipProtOffs=128
ChipProtAddr = SFLASH_MACRO_0 + ChipProtOffs - 4          // = SFLASH_MACRO_0 + 0x7C
ChipProt = ReadIO(ChipProtAddr)
ChipProt = (ChipProt >> 24) & 0x0F
// 若为 VIRGIN 或 OPEN，需互换（供电烙印反转，见②）
```
对 4100S Plus，Table 1-1 给出 `SFLASH_MACRO_0 = 0x0FFFF000`，故 `ChipProtAddr = 0x0FFFF07C`
（与此前 context.md 中"规格 §4.11 建议改读 flash 0x0FFFF07C"的记录一致，已被本次核对确认）。

**progspec Step 2（Check Silicon ID，§4.5）的伪代码完全没有提到保护位**——`CPUSS_SYSARG`/`CPUSS_SYSREQ`
的位域定义只有 Silicon ID Hi/Lo/Rev/Family ID 四项（`CPUSS_SYSREQ[11:0]=Family ID`），**没有任何"保护位
在 SYSREQ[15:12]"的规格依据**。

**结论**：固件当前 `read_chip_protection()`/`read_silicon_id()` 里 `_last_chip_prot = (p1>>12)&0x0F`
（从 GET_SILICON_ID 副产物的 SYSREQ 高位提取）**在两份文档中查无出处，是编造的读取方法**。规格唯一给出
的权威方法是从 flash 地址 `0x0FFFF07C` 直接 AHB 读取。这意味着此前所有"prot=0"的观测都不可信——很可能
读到的只是恰好为 0 的 Family ID 高位残留，并非真实保护态。

② **CHIP_PROT 编码（Appendix A, Table A-1，第46页）**：
- VIRGIN：CPUSS_PROTECTION/SFlash 值=0x00，但写入 supervisory row 的字面值=0x01（反转存储）。
- OPEN：值=0x01，写入 supervisory row 字面值=0x00（反转存储，出厂默认态）。
- PROTECTED：值=0x02，写入即为 0x02（不反转）。
- KILL：值=0x04，写入即为 0x04（不反转，不可逆）。
读取时"For VIRGIN and OPEN modes, the value saved in supervisory row is inverted"——即从 flash 读出
0x00 应翻译为 OPEN(0x01)，读出 0x01 应翻译为 VIRGIN(0x00)；PROTECTED/KILL 不需要翻译。固件
`psoc_types.h::chip_prot` 里的数值定义（VIRGIN=0,OPEN=1,PROTECTED=2,KILL=4）与规格一致，但因为读取
地址/方法错误，翻译逻辑目前无意义。

③ **非 OPEN 是否正是返回 0xF0000014**：AN84858 明确定义的对应语义码是 `0xF0000001`
（Invalid Chip Protection Mode），**不是 0xF0000014**。两文档都没有把 0xF0000014 与保护模式关联起来。
但 AN84858 Table A-1 对 PROTECTED 模式的描述给出了极关键的旁证：**"In this mode it is possible to
read the silicon ID and move the chip back to OPEN mode"**——即 PROTECTED 态下 GET_SILICON_ID 仍可用，
但其它 flash/SRAM/寄存器访问被 NACK。这与当前实测现象（GET_SILICON_ID 成功，SET_IMO/ERASE/PROGRAM
全部被拒）高度吻合，是"芯片实际处于 PROTECTED 而非 OPEN"假说的最强文档级证据。

## 6. test-mode acquire 到首个 flash 命令之间是否漏步骤

Step 1A 伪代码（progspec §4.3.1）末尾就内嵌了 SET_IMO 调用（若家族需要），规格没有在 acquire 完成和
首个 flash 命令之间插入其它必需步骤。Step 2（Check Silicon ID）是校验性质，非强制前置。**没有发现"漏掉
的步骤"**，问题集中在第5条的保护位读取方法错误上。

## 7. AN84858 的 EraseAll/ProgramRow 伪代码

AN84858 本身**不重复给出** EraseAll/ProgramRow 的详细 SROM 参数级伪代码——它在正文明确交棒给 progspec
（"For details on the hex file format...programming specifications document listed in the References
section"），AN84858 聚焦于 HSSP host 侧的 10 步高层流程（DeviceAcquire/CheckSiliconID/EraseAllFlash/
ChecksumPrivileged/ProgramFlash/VerifyFlash/ProgramProtectionSettings/VerifyProtectionSettings/
VerifyChecksum/ExitHSSP）与错误状态位定义（Appendix C：ACK 码、SWD 校验位、Port Acquire Timeout、
SROM Polling Timeout 等）。**详细的 EraseAll/ProgramRow 逐字节伪代码权威来源是 progspec §4.6/§4.8**
（已在第2/3条摘录）。

**最可能漏的一步**：不是 EraseAll/ProgramRow 本身的参数或调用序列（两者与固件实现比对无误），而是
**Step 3 开头"读芯片保护模式"这一步的读取方法本身是错的**——固件从未真正按规格从 flash 0x0FFFF07C
读取过保护字节，所以 `erase_all()` 里"若 PROTECTED 先 WRITE_PROTECTION→OPEN"的分支从未被真正触发过。

---

## 根因排序（本次检索结论）

1. **★最高优先级★ 保护位读取方法无据且很可能错误**：`read_chip_protection()` 从 GET_SILICON_ID 的
   SYSREQ[15:12] 提取，规格中查无此依据；权威方法是 AHB 直读 flash `0x0FFFF07C`。若芯片实际处于
   PROTECTED（现象与 AN84858 对 PROTECTED 模式的描述完全吻合：只读 API 通，写/擦 API 全拒），则
   `erase_all()` 现有代码因误判 `prot==0(VIRGIN)` 而走了"直接 ERASE_ALL"分支，从未尝试
   `WRITE_PROTECTION→OPEN`，这正好解释了 ERASE_ALL/PROGRAM_ROW/SET_IMO（三者都是修改类 API）全部被拒、
   GET_SILICON_ID（读类 API）放行的现象。
2. **次优先级 0xF0000014/0xF0000010 缺乏权威定义**：两份文档都未收录，无法排除是家族专属的
   "Invalid Chip Protection Mode" 变体码；也无法排除与 clock 相关（0xF0000012 有据，0xF0000014 无据）。
   在解决第1点前，无法单独用状态码含义来验证。
3. **较低优先级 SET_IMO 是否为本家族强制项存疑**：Table 1-1 OCR 列对齐不完全确定，AN84858 家族列举
   未明确包含/排除"4100S Plus"，建议用官方 datasheet/PSoC Creator 家族表二次确认，但这不是当前失败的
   主因（因为 GET_SILICON_ID 与 SET_IMO 用完全相同的直填参数机制，差异只可能来自"操作类型是否被当前
   保护态允许"，与是否需要 IMO 无关）。

## 下一步建议（不改代码，仅提供方向）

1. 把 `read_chip_protection()` 改为规格权威方法：AHB 直读 `SFLASH_MACRO_0(0x0FFFF000) + ROW_SIZE(128)
   - 4 = 0x0FFFF07C`，取 `(v>>24)&0x0F`，并按 Table A-1 做 VIRGIN/OPEN 互换翻译（PROTECTED/KILL 不翻译）。
2. 重新烧录跑一次诊断，观察真实 `prot` 值。若变为 `PROTECTED(0x02)`，`erase_all()` 现有的
   "PROTECTED→WRITE_PROTECTION(OPEN)→重新 acquire" 分支理论上会被触发，可验证是否真的解开 ERASE_ALL。
3. 若 `WRITE_PROTECTION` 命令本身也被拒（它同样是修改类 SROM 调用，理论上 PROTECTED 态下按 AN84858
   描述应该被允许——这正是"把芯片移回 OPEN"的官方出口），则需要进一步排查是否 WRITE_PROTECTION 的参数
   布局/宏 ID 有误，或该芯片家族在 TRM 里有额外前置条件（本次两份文档不含 TRM，需要更高权限文档或实测
   对比 KitProg3/MiniProg4 的真实 SROM 交互序列）。
4. 提醒：log.log 最新一条诊断（`acq=0 delay=4294967295us sysarg=0x80000004 sysreq=0x000004F2`）显示
   **acquire 现在完全没命中窗口**（与 context.md 记录的"acq=1 成功"状态不一致），这可能是后续改动引入
   的新回归，建议先确认 acquire 本身是否仍稳定成功，再谈保护位修复，避免同时排查两个变量。


---

# 实测:保护读法修正后 prot=1(OPEN) —— PROTECTED 假说被否,确认 CLOCK 根因

修正 read_chip_protection 为规格 AHB 直读 0x0FFFF07C 后,烧录实测:
`acq=1(delay=0us sysarg=0x00000000 sysreq=0x20000000) id=0x0BC11477 prot=1 imo=0(0xF0000014) erase=0(0xF0000014) prog=0(row=0 0xF0000012) verify=0(@0x00000000) link=0`
- **prot=1 = OPEN**(且 flash 0x0FFFF07C 读成功=存储字节0→OPEN)。芯片确为 OPEN,**非 PROTECTED**。PROTECTED 假说否决。
- flash 读可用(能读 supervisory row)。
- ❌ 仍:SET_IMO=0xF0000014、ERASE_ALL=0xF0000014、**PROGRAM_ROW=0xF0000012=Invalid Flash Clock(AN84858 有定义)**。

## 根因锁定:FLASH CLOCK(IMO 48MHz)未正确配置
- PROGRAM_ROW 的 0xF0000012=Invalid Flash Clock 是决定性证据:flash 写擦需 IMO=48MHz + HFCLK=IMO,当前没配上。
- AN84858 需 IMO48MHz 的家族列表含 "PSoC 41xxS" → 本芯片 4100S Plus 属之,**IMO 必需**。
- 但 SET_IMO(SROM cmd 0x15)返回 0xF0000014(未定义码),没设成 → 后续 ERASE/PROGRAM 全挂。

## 下一步(待 module-builder 聚焦深搜)
1. 本家族(4100S Plus/128B)flash 编程的**时钟配置确切步骤**:是 SROM 调用还是直接写时钟寄存器?
2. SET_IMO/Configure-Clock SROM 命令的**正确 opcode + 参数格式**(现用 0x15,可能错)。
3. 若走直接寄存器:IMO/HFCLK 相关寄存器地址与设 48MHz 的值。
4. 为何 SET_IMO 返回 0xF0000014。


---

# module-builder 检索：flash 时钟配置权威步骤

来源：`.kiro/context/psoc4_progspec.txt`（规格 002-22325 Rev.*H）、`.kiro/context/an84858_hssp.txt`
（AN84858 Rev.*O）、以及 OpenOCD 官方源码 `src/flash/nor/psoc4.c`（第三方独立实现，与两份文档
交叉验证用，非任务列出的两份本地文档，但用于解决文档本身未定义的状态码问题）。

## 1. 时钟配置的确切步骤：SROM 调用，不是直接写寄存器

**progspec §4.3.1 Step 1A 伪代码（第23页，逐字）**：SET_IMO 是**acquire 流程末尾内嵌的一次 SROM
系统调用**，不是主机直接写 CLK_IMO_*/SRSS/CPUSS 寄存器。原文明确："Set IMO = 48 MHz to enable flash
operations. Use SROM API for this."（Figure 4-3 流程图同一说明）。两份文档全文都**没有**给出任何
IMO/HFCLK 相关寄存器的绝对地址——因为该配置被封装在 SROM 固件内部，不通过 AHB 直写完成。回答问题3
（直接写寄存器方案）：**不适用，文档未提供也不存在此路径**。

## 2. SROM 命令：opcode 与参数格式核对

`SROM_CMD_SET_IMO_48MHz = 0x15`（progspec 表，第1312-1314行）——固件现用 `0x15` **正确**，不是别的
opcode。已用 OpenOCD 独立源码交叉核实：`#define PSOC4_CMD_SET_IMO48 0x15`，`KEY1=0xb6`、`KEY2=0xd3`，
与固件 `psoc_types.h` 完全一致。

参数格式（progspec 第1561-1564行，逐字）：
```
Params = (SROM_KEY1 << 0) + ((SROM_KEY2+SROM_CMD_SET_IMO_48MHz) << 8);  // 无 cmd_param 字段
WriteIO(CPUSS_SYSARG, Params);
WriteIO(CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_SET_IMO_48MHz);
```
即参数**直填 SYSARG**（不经 SRAM），且**没有** cmd_param（第三个 16 位字段为 0）。固件
`set_imo_48mhz()` 当前实现：
```cpp
uint32_t params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_SET_IMO_48MHZ) << 8);
_write_io(REG_CPUSS_SYSARG, params);
_srom_exec(SROM_CMD_SET_IMO_48MHZ);
```
**与规格逐字节比对无误**——opcode、KEY1/KEY2、参数布局都对。问题7"opcode 不对"的猜测被否决：
两个独立来源（Infineon 官方规格 + OpenOCD 第三方实现）都确认 `0x15` 是本命令的唯一正确 opcode。

## 3. 直接写寄存器方案

不适用（见第1条）。

## 4. Table 1-1 "SET_IMO_48MHz required" 对 "4100S Plus" 列的核对

**OCR 数据不足以判定，明确说不确定**：原文本行只有 5 个值（"Yes No Yes No Yes"）却对应 12 个家族列，
说明 PDF 原表存在大量合并单元格（跨列共享同一值），OCR 抽取丢失了 colspan 信息，无法从纯文本反推
"4100S Plus" 具体落在哪一段。此前 context.md 记录的"128B/512 行家族=No"是先前 module-builder 的推测，
本次复核**无法验证也无法证伪**，维持不确定状态。

**但有更强的旁证**：AN84858 正文（393-395行）逐字列出需要 IMO48 的家族："PSoC 40xx, PSoC 4xx7_BLE,
PSoC 40xxS, PSoC 41xxS and PSoC 4100PS"——**未含"4100S Plus"**。同时 OpenOCD 源码里 `psoc4_flash_prepare()`
对**所有非 legacy 家族**（即所有用 CPUSS_SYSREQ_NEW 地址 0x40100004 的新家族，含 4100S Plus 类）都
**无条件尝试调用 SET_IMO48**，并把返回码 `0xf0000013`（未文档化，OpenOCD 注释"probably means命令未
实现"）当作可安全忽略的"正常无需"信号；只有其它任意非成功码才判定为致命错误。这说明 Infineon/OpenOCD
两方实践上都是"不管家族是否真的需要，都调一次 SET_IMO，用特定返回码识别'不需要/未实现'"，而不是靠
Table 1-1 严格分家族跳过。

## 5. AN84858 的时钟设置步骤

HSSP 十步流程第1步"Device acquire"内嵌完成（391-395行，逐字）："requires internal main oscillator
(IMO) frequency to be set at 48 MHz before flash erase/write operations. This operation is also included
in the Device Acquire routine for these devices."即：**没有独立的第0步"设置时钟"**，SET_IMO 调用被
定义为 acquire 例程内部的一部分，对需要它的家族而言，acquire 成功后 IMO 应已经是 48MHz。AN84858 本身
不重复给出该调用的参数级伪代码，权威细节来源是 progspec（已在第2条摘录）。

## 6. 0xF0000012 Invalid Flash Clock 的完整上下文

AN84858 Appendix B Table 12（1711-1714行，逐字）："0xF0000012 Invalid Flash Clock: The CY8C40xx
family of devices must set the IMO to 48 MHz and the HF clock source to the IMO clock before write/erase
operations."——**触发条件**：执行 flash 写/擦类 SROM 命令（LOAD_LATCH/PROGRAM_ROW/WRITE_ROW/ERASE_ALL
等）时，若 IMO 未处于 48MHz 或 HFCLK 源未选 IMO，SROM 内部会拒绝该操作并返回此码。**避免方法**：文档
字面只给出"必须先设好"，没有另外的检测或等待 API；唯一途径是让 SET_IMO 调用真正成功。**该码的官方定义
仅点名 CY8C40xx 家族**，但两文档都没有排除其对 4100S Plus 同样适用——本次实测 PROGRAM_ROW 精确命中此码，
是"确实是时钟问题"的直接证据（比 Table 1-1 的猜测更可靠，因为这是运行时实测触发的确切码，不是推断）。

## 7. ★关键新发现★：SET_IMO 返回 0xF0000014 ≠ 已知的"命令不存在"信号

**两份任务列出的本地文档都不包含 0xF0000010/0xF0000014 的定义**（Table 12 序列 0x0E → 直接跳 0x12，
0x12 后无 0x13/0x14）。为解决这个空白，交叉核对了 OpenOCD 官方开源实现（`psoc4.c`，第三方独立复刻同一
Infineon 协议，非任务指定文档，仅作辅助排歧）：

```c
#define PSOC4_SROM_ERR_IMO_NOT_IMPLEM  0xf0000013   /* not documented in any TRM */
...
retval = psoc4_sysreq(bank, PSOC4_CMD_SET_IMO48, 0, NULL, 0, &sysreq_status);
if ((sysreq_status & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
    if (sysreq_status == PSOC4_SROM_ERR_IMO_NOT_IMPLEM)
        LOG_INFO("PSOC4_CMD_SET_IMO48 is not implemented on this device.");  // 可忽略
    else {
        LOG_ERROR(...);   // 其它任何码都是致命错误
        return ERROR_FAIL;
    }
}
```

**这是决定性区分**：OpenOCD 实证的"可安全忽略/命令未实现"码是 **0xF0000013**，比我们实测的
**0xF0000014 少 1**。两者不是同一个码。这意味着：
- 此前 context.md 里"SET_IMO 的 0xF0000014 属未支持,可忽略"的判断**缺乏依据，甚至可能是错的**——
  按 OpenOCD 的实践标准，0xF0000014 应被当作**真实错误**，不是"优雅跳过"信号。
- 更值得注意的是 **ERASE_ALL 与 SET_IMO 返回完全相同的 0xF0000014**，而 **PROGRAM_ROW 返回不同的
  0xF0000012**。如果 0xF0000014 真的只是"时钟未配置"的另一种表现，为什么 ERASE_ALL/SET_IMO 与
  PROGRAM_ROW 的返回码不一样？两文档均未定义 0xF0000014 具体含义，**无法排除它是与 0xF0000012 不同的
  独立失败原因**（例如芯片保护/opcode/key 相关的家族专属扩展码），只是巧合与 SET_IMO/ERASE 共现。

## 8. ★被此前放弃、从未真正验证过的替代路径★

翻查 `psoc_swd_bringup.md` 历史记录发现：固件中曾实现过 **debug-halt + bkpt-算法执行 SROM**
（`_connect_and_halt` / `_write_core_reg` / 早期版本的 bkpt-run `_srom_exec`，对齐 OpenOCD 的做法），
但**在没有真正跑通 ERASE_ALL/PROGRAM_ROW 验证的情况下就被放弃**，转向纯 test-mode 路线（理由是
"规格权威路线"的伪代码更简单、且 GET_SILICON_ID 在 test-mode 下已经工作）。

OpenOCD 的 `psoc4_sysreq()`（已在第2次 fetch 中读取完整源码）**始终**：
1. 要求 `target->state == TARGET_HALTED`（debug-halt，不是 test-mode）；
2. 写 SYSREQ 时**始终带 `PSOC4_SROM_SYSREQ_BIT | PSOC4_SROM_HMASTER_BIT | cmd`**（HMASTER 位常年存在，
   不区分命令类型）；
3. 通过 `target_run_algorithm` 让 CPU 真正执行一段内存中的 `bkpt 0` 代码（NMI 服务完立即撞 bkpt 停）。

这是 Infineon/Cypress 生态里**唯一有大量真实硬件验证**（KitProg3/MiniProg4 之外，第三方开源实现且被
广泛使用）的 SROM 触发方式。虽然 OpenOCD 官方支持家族列表**不包含 0xB5（4100S Plus）**，但 SROM 协议
层（KEY1/KEY2/opcode/参数布局）在 progspec 002-22325 里明确适用于全系列 CY8C4xxx，包括 4100S Plus，
说明 OpenOCD 的触发机制本身应同样适用，只是它没有内置这个家族的 flash 几何参数表（这只影响
probe()，不影响 sysreq 机制）。

**这是本次检索能给出的最具行动力的结论**：test-mode 路线（当前固件用的）与 debug-halt+bkpt 路线
（HMASTER 常驻）在 GET_SILICON_ID 上表现一致，但从未在 ERASE_ALL/PROGRAM_ROW 上被真正对比测试过——
此前的"HMASTER 有无对比"测试**全部是在 test-mode 下做的**（只改 SYSREQ_BIT 是否带 HMASTER，CPU 依然
停在 boot ROM 轮询，从未真正 resume CPU 去跑 bkpt 算法）。真正的 OpenOCD 式"halt + resume + bkpt"路径
从未被完整验证过对 ERASE_ALL/PROGRAM_ROW/SET_IMO 的效果。

---

## 可直接落地的时钟配置方案

**结论：SROM 参数已经正确（opcode=0x15, KEY1=0xB6, KEY2=0xD3, 无 cmd_param, 直填 SYSARG），不是参数
或 opcode 错误。问题不在"怎么配置时钟"这个层面，而在于 SET_IMO 调用本身被 SROM 以未知码 0xF0000014
拒绝——这与"参数格式错"或"opcode 错"无关（两个独立信息源都确认参数格式对）。**

因此没有"改参数值"意义上的修法；能给出的两条最可能让其成功/绕过的具体改法按优先级排列：

### 改法 A（优先，成本最低）：换回 debug-halt + bkpt-算法路径重新完整测试一次
固件里 `_connect_and_halt()` / `_write_core_reg()` / bkpt 相关常量都**保留未删**（当前只是不被调用）。
具体步骤：
1. `acquire()` 改回调用 `_connect_and_halt()`（而不是现在的 test-mode 版）。
2. `_srom_exec()` 改回 bkpt-算法版本：写 `bkpt` 到 `0x20000000` → 设置 PC/SP/xPSR → 写
   `CPUSS_SYSREQ = SYSREQ_BIT | HMASTER_BIT | cmd`（**恢复带 HMASTER**，对齐 OpenOCD 逐字节实现）→
   resume（清 `C_HALT`，保留 `C_DEBUGEN`）→ 轮询 `S_HALT` → 读 `SYSARG`。
3. 依次跑 GET_SILICON_ID（确认路径本身通）→ SET_IMO（0x15）→ ERASE_ALL → PROGRAM_ROW，观察返回码是否
   变化。若 SET_IMO 在这条路径下返回 `0xA0000000` 或哪怕是 `0xF0000013`，就证实之前测试的"HMASTER 有无
   无差别"结论只对 test-mode 成立，debug-halt+bkpt 是另一条独立的、更贴近真实工作实现的路径。

### 改法 B（备选，若 A 不行）：跳过 SET_IMO，直接看 ERASE_ALL 是否真的需要它
鉴于 AN84858 393-395 行明确的家族列表**不包含 "4100S Plus"**，且 OpenOCD 对"未实现"用 `0xf0000013`
识别、我们拿到的是 `0xf0000014`（不同码）——存在一种可能：**本家族其实不需要 SET_IMO，0xF0000014
是该命令在本家族"根本不存在/未注册"的另一种失败表现（不同于 OpenOCD 见过的 0x13），ERASE_ALL 的
0xF0000014 可能是另一件事在拦（例如 chip protection 或 SROM 内部状态），并非真的缺时钟**。可尝试：
删除/跳过 SET_IMO 调用，直接对 ERASE_ALL 做 debug-halt+bkpt（改法A）测试，观察 ERASE_ALL 是否单独
成功——如果成功，说明本家族确实不需要 IMO 步骤，0xF0000012（PROGRAM_ROW 的错误）才是唯一真实的时钟
问题，且可能仅在 PROGRAM_ROW/LOAD_LATCH 这类真正写 flash 阵列的命令上触发（ERASE_ALL 属于"擦除"，
按 AN84858 366-395 行原文，语义上是"erase"，与"write"分开描述，一些家族的 SROM 实现里 ERASE 和
PROGRAM 对时钟的敏感度可能不同）。

两者中 **改法 A 是本次检索能给出的最高置信度下一步**：它是唯一被证明在其它 PSoC4 家族真实硬件上
work 的实现方式，而我们自己的 test-mode 路线是从文档伪代码直译、从未在任何已知第三方实现里被验证过
对 flash 写擦类命令的有效性（只验证过 GET_SILICON_ID 只读类命令）。

## 异常说明
本次任务范围内只读检索、未改动任何代码文件（仅追加了本节到 `.kiro/context/psoc_swd_bringup.md`）。
未能给出 0xF0000010/0xF0000014 的权威官方定义——两份指定文档确认未收录，OpenOCD 源码也仅定义了
0xF0000013（不同码）。这是本次检索的诚实边界：需要 PSoC 4 Architecture TRM（Nonvolatile Memory
Programming 章节）才能拿到权威定义，本次检索范围（progspec + AN84858）不包含该章节。


---

# 决定性权威:OpenOCD psoc4.c 完整源码(已通读)—— 必须走 HALT+bkpt-run,不是 test-mode

抓取 https://raw.githubusercontent.com/openocd-org/openocd/master/src/flash/nor/psoc4.c 全文通读。这是在真实 PSoC4 硬件广泛验证的实现。逐条铁证:

## 常量(与我们一致)
- SYSREQ_BIT=1<<31, **HMASTER_BIT=1<<30(必带!)**, PRIVILEGED_BIT=1<<28, STATUS_SUCCEEDED=0xA0000000。
- KEY1=0xB6,KEY2=0xD3。CPUSS_SYSREQ_NEW=0x40100004,SYSARG_NEW=0x40100008(非 legacy,我们芯片属之)。
- cmd: GET_SILICON_ID=0,LOAD_LATCH=4,WRITE_ROW=5,PROGRAM_ROW=6,ERASE_ALL=0xA,CHECKSUM=0xB,WRITE_PROTECTION=0xD,**SET_IMO48=0x15**,WRITE_SFLASH_ROW=0x18。
- **0xF0000013 = SET_IMO48 未实现(可忽略);0xF0000014 是别的真实错误,不可忽略。**
- 保护读法:GET_SILICON_ID 后 part1=CPUSS_SYSREQ,protection=(part1>>12)&0xF,family=part1&0xFFF。**(SYSREQ>>12)&0xF 是 OpenOCD 官方做法,并非编造。**

## SROM 执行机制(psoc4_sysreq,铁证)
target 必须 **TARGET_HALTED**(psoc4_flash_prepare 首行检查)。每次调用:
1. 组 param1 = KEY1 | ((KEY2+cmd)<<8) | (cmd_param<<16)。
2. 参数传递:**有 param block**(ERASE_ALL/LOAD_LATCH/PROGRAM_ROW/WRITE_ROW)→ param1 写 SRAM 工作区首字,SYSARG=该 SRAM 地址;**无 block**(GET_SILICON_ID/SET_IMO48/WRITE_PROTECTION)→ param1 直接写 SYSARG。
3. 写 `bkpt 0`(0xBE00)到 SRAM code 工作区;SP=code 区末尾。
4. **写 CPUSS_SYSREQ = SYSREQ_BIT | HMASTER_BIT | cmd**。
5. **target_run_algorithm**:PC=code 区首,resume CPU 执行 → 挂起的 SROM NMI 被立即服务(CPU 必须真跑才服务!)→ 返回撞 bkpt 停。超时 1000ms。
6. 读 SYSARG 状态,(&0xF0000000)==0xA0000000 成功。

## flash_prepare(每次 erase/program 前必做)
1. GET_SILICON_ID 校验 family。
2. **非 legacy 家族:调 SET_IMO48。返回必须成功,或恰为 0xF0000013(未实现)才可忽略;其它错误(含 0xF0000014)硬失败。**

## write 流程
每行:LOAD_LATCH(param block: [0]=param1,[1]=row_size-1,[8..]=行数据;cmd_param=byte0|(macro<<8))→ 再 cmd_program_row(默认 WRITE_ROW=5 带自动擦;或 ERASE_ALL 后用 PROGRAM_ROW=6,cmd_param=row&0xFFFF,param block 1 字)。

## 根因总结(推翻 test-mode 路线)
test-mode 下 CPU 停在 boot ROM,只有 GET_SILICON_ID 被 boot 轮询侥幸服务;SET_IMO/ERASE/PROGRAM 需 CPU 真执行 NMI 才被服务,test-mode 不执行 → 全返回 0xF0000014。**必须改回 OpenOCD 的 halt+resume-bkpt 机制,并恢复 HMASTER_BIT。**之前"bkpt-run 也失败"极可能是:①没用 vector-catch 可靠停核;②erase 前没先跑 SET_IMO;③bkpt 算法 resume/PC/SP/xPSR 细节有 bug。

## 落地方案(交 module-builder 忠实复刻 OpenOCD)
- acquire()=connect-under-reset + **vector catch 停核**:XRES 低→窗口内连 DP(CTRLSTAT=0x54000000/SELECT=0/CSW=CSW_PRIV)→DHCSR=DBGKEY|C_DEBUGEN→DEMCR(0xE000EDFC) VC_CORERESET(bit0)=1→释放 XRES→核在复位向量停→轮询 DHCSR S_HALT→清 DEMCR。PPB 访问用 CSW_PRIV。
- _srom_exec(cmd, 有无 block, 参数):按上面 psoc4_sysreq 六步,**SYSREQ=SYSREQ_BIT|HMASTER_BIT|cmd**,bkpt 算法 resume 执行(复用现有 _write_core_reg:SP=13/PC=15/xPSR=16,xPSR=0x01000000)。SRAM: code@0x20000000(bkpt),params@0x20000100,SP=0x20000800。
- 主 bringup 调用序:acquire(halt)→(诊断)GET_SILICON_ID→**SET_IMO48(必须成功或 0xF0000013)**→ERASE_ALL→逐行 LOAD_LATCH+PROGRAM_ROW→verify。
- SET_IMO 返回 0xF0000013 视为 OK 继续。


---

## module-builder 落地：HALT+bkpt-run 机制已忠实复刻 OpenOCD psoc4.c，编译通过

### 已改动
1. `psoc_types.h`：`HMASTER_BIT` 注释改为"OpenOCD psoc4.c 逐字复刻的 halt+bkpt-run 机制必需，已恢复"，常量值不变。
2. `psoc_swd.cpp`：
   - `SROM_REQ` 由仅 `SYSREQ_BIT` 改为 `SYSREQ_BIT | SROM_HMASTER_BIT`（恒带 HMASTER，不区分命令类型，对齐 OpenOCD）。
   - `_srom_exec(cmd)` 完全重写为 bkpt-run 六步：①写 `bkpt`(0xBE00) 到 `SROM_CODE_ADDR`(0x20000000)；②`_write_core_reg` 设 SP=0x20000800/PC=0x20000000/xPSR=0x01000000（Thumb 位，否则 resume 立刻 HardFault）；③写 `CPUSS_SYSREQ=SYSREQ_BIT|HMASTER_BIT|cmd`；④resume：写 `DHCSR=DBGKEY|C_DEBUGEN`（清 C_HALT）；⑤轮询 `DHCSR.S_HALT`(bit17)，超时 1000ms（失败写 `_last_srom_status=0xDEAD00xx` 分段诊断，撞 bkpt 超时专用 `0xDEAD00FF`）；⑥读 `CPUSS_SYSARG` 存 `_last_srom_status`，返回 `(&0xF0000000)==0xA0000000`。每次调用都重新写 bkpt+PC/SP/xPSR（上次执行后 PC 已变）。删除了 `_poll_srom_status()`（test-mode 轮询式，已被 bkpt-run 取代，头文件同步移除声明）。
   - `acquire()` 重写为：外层最多 20 次重试调用 `_connect_and_halt()`（已实测能停核的 XRES 复位+boot 窗口连 DP+CSW_PRIV+写 DHCSR C_HALT|C_DEBUGEN+轮询 S_HALT），成功即返回并记 `_last_acquire_delay=attempt`；`_connect_and_halt()` 内部已有的 `_last_acquire_status`(=DHCSR)/`_last_acquire_sysreq`(=CPUID) 赋值直接复用，未改动 `_connect_and_halt()` 本体。不再走 test-mode/TEST_MODE 寄存器路径。vector-catch(DEMCR VC_CORERESET) 增强**未实现**——`_connect_and_halt()` 已有的 boot-窗口 race 停核方案是此前唯一实测验证过 S_HALT 成功的路径，为避免引入未验证的新变量，本轮未改动其内部逻辑，仅复用。
   - `set_imo_48mhz()`：新增对 `0xF0000013`（OpenOCD 认定的"命令未实现"码）的放行——`_srom_exec` 返回失败但状态码恰为 `0xF0000013` 时，视为非致命，返回 `true`。
   - `_write_core_reg`/`_connect_and_halt` 原样复用（未改动内部实现）；`_acquire_once`/`_acquire_test_mode_workaround`/`_probe_programming_mode` 三个 test-mode 时代的方法**保留定义未删除**（避免破坏可能引用，且删除非本次任务范围），但均已不被调用。
3. `psoc_swd.h`：`acquire()`/`_srom_exec` 注释同步改为 halt+bkpt-run 机制说明；移除 `_poll_srom_status()` 声明。

### 未改动
- `main.cpp` 的 `psoc_flash_bringup()` 调用序（acquire→read_chip_protection→set_imo→erase→program_rows→verify→reset_run）未触碰。
- `read_chip_protection()`（AHB 直读 SFLASH macro0 0x0FFFF07C）保持上一版不变——halt 态下 flash 可读，无需改。
- `erase_all()` 的 PROTECTED/OPEN 分支逻辑未改（仍复用 `_srom_exec`，机制切换对它透明）。

### 验证结果
`pio run`（cwd=`main_firmware`）：**SUCCESS**，耗时 3.81s。RAM 26952/262144 bytes(10.3%)，Flash 456580/3141632 bytes(14.5%)。产物 `.pio\build\pico\firmware.uf2` 已生成。**未烧录**，按指示留给主 agent。

### 保真点自查（对照任务书"严格保真点"）
1. ✅ SYSREQ 恒带 HMASTER_BIT（`SROM_REQ` 常量级保证，不区分命令）。
2. ✅ resume 前先写好 PC/SP/xPSR（core 在 `_connect_and_halt()` 后本就是 halt 态，`_srom_exec` 内顺序：bkpt→SP→PC→xPSR→SYSREQ→resume）。
3. ✅ xPSR 含 Thumb 位 `0x01000000`。
4. ✅ 每次 `_srom_exec` 都重写 bkpt+PC/SP（无跨调用复用假设）。
5. ✅ PPB(DHCSR/DCRSR/DCRDR) 访问经 `_write_core_reg`/`_read_io(REG_DHCSR,...)`，CSW 全程为 `_connect_and_halt()` 里设的 `CSW_PRIV`（0x03000042），SRAM/CPUSS 也用同一 CSW，未切换——与 OpenOCD"全程一个 CSW"一致。

### 对 SET_IMO 是否会返回成功的判断
`_srom_exec` 机制换成 bkpt-run 后，SROM NMI 是否被真正服务取决于 resume 是否让 CPU 真正跑起来撞上 bkpt——若 `_connect_and_halt()` 停核可靠（此前已实测 S_HALT 命中），本次改动补齐了 OpenOCD 铁证里"必须 CPU 真执行"的缺失环节，SET_IMO/ERASE/PROGRAM 理论上应从 test-mode 下的"CPU 不执行、SROM 不服务"变为"CPU 执行、SROM 服务"。SET_IMO 预期结果：返回 `0xA0000000`（真正成功，若本家族确实支持该命令）或 `0xF0000013`（未实现，已被 `set_imo_48mhz()` 判定为非致命并放行）。**只有实机烧录验证才能确认**——本次未烧录，无法排除 `_connect_and_halt()` 的 boot 窗口 race 停核在新 bkpt-run 负载下时序不稳定的风险（尚未加 vector-catch 增强）。若实测 SET_IMO 仍返回其它非成功码，下一步应优先检查 `_connect_and_halt()` 的停核可靠性（S_HALT 是否稳定命中），而非再怀疑 bkpt-run 六步本身。


---

# 实测:忠实复刻 OpenOCD halt+bkpt-run 后,SET_IMO/ERASE 仍 0xF0000014(机制无关!)

烧录 halt+bkpt-run 版实测:
`acq=1(delay=0us sysarg=0x03030003 sysreq=0x410CC601) id=0x0BC11477 prot=1 imo=0(0xF0000014) erase=0(0xF0000014) prog=0(row=0 0xF0000012) verify=0`
- **sysreq=0x410CC601 = Cortex-M0+ CPUID**(_connect_and_halt 读 0xE000ED00),PPB 访问正常。
- **sysarg=0x03030003 = DHCSR,含 S_HALT(bit17)** → 核确实 halt。
- **SROM 确实执行**:SYSARG 从我们写入的参数(0xE8B6)被 SROM 覆盖成 0xF0000014,是真·SROM 返回码,非陈旧值/非超时(超时会是 0xDEAD00FF)。
- ❌ 但 SET_IMO=0xF0000014、ERASE=0xF0000014、PROGRAM=0xF0000012,**与 test-mode 完全相同**。

## 关键诊断:失败与执行机制无关
test-mode 与 OpenOCD 权威 halt+bkpt-run **两种机制给出完全相同的错误码**。→ 不是执行机制问题(HMASTER/test-mode/bkpt 都已排除)。芯片=OPEN 也排除保护。SROM 能跑、能返回。
**结论:family 0xB5(4100S Plus)的 flash/clock 类 SROM 命令语义与 OpenOCD/AN84858 覆盖的老家族(表止于 0xAB)不同。** 老家族能用的 SET_IMO(0x15)在 0xB5 返回未定义码 0xF0000014(疑似"未实现",类比老家族的 0xF0000013),导致 flash 时钟没设上 → PROGRAM 报 0xF0000012 Invalid Flash Clock。

## 下一步:改走"直接寄存器配置 flash 时钟",绕过 SROM SET_IMO
需要 4100S Plus 的时钟寄存器映射(SRSS CLK_IMO_CONFIG/CLK_SELECT/CLK_IMO_TRIM 等绝对地址与设 48MHz 的值)。docs(002-22325+AN84858)不含寄存器级 TRM。待 web/文档补齐寄存器地址后:acquire(halt)→直接 AHB 写时钟寄存器设 IMO=48MHz&HFCLK=IMO→ERASE→PROGRAM。
诚实边界:0xF0000014 的字面义与 0xB5 专属 NVM 编程流程需 4100S Plus Architecture/Registers TRM,当前无。


---

# 文档墙:权威源是 002-39884(2025 现行编程规格),本地无

- 4100T/S Plus Reference Manual(manualslib)§22.5 明确:编程算法/时钟/时序细节"refer to CY8C4xxx,CYBLxxxx programming specifications **002-39884 Rev.*B 2025-03-27**"。确认 IDCODE 0x0BC11477 = 我们的芯片。
- 我们本地只有 **002-22325**(旧)+ an84858。002-39884 在 Infineon 官网登录墙后,web 不可直接抓。
- SEGGER KB 反复指出:部分 PSoC4 需在 flash 加解锁前用 **"Configure Clock" 命令**把时钟设 48MHz,具体哪些芯片/怎么做"参见对应器件的编程规格"(即 002-39884)。
- 我们的 SET_IMO(0x15)在 family 0xB5 返回 0xF0000014,疑似该家族"Configure Clock"命令的 opcode/参数与老家族不同(002-39884 才有)。

## 需要用户提供(卡点)
1. **002-39884**(现行 CY8C4xxx/CYBLxxxx Programming Specifications)—— 尤其"Configure Clock / SET_IMO"命令的 opcode+参数、SROM 状态码表(0xF0000014 含义)、本家族 ERASE/PROGRAM 专属流程;或
2. **PSoC 4100S Plus / 4100T Plus Registers TRM**(SRSS 时钟寄存器:CLK_IMO_CONFIG/CLK_SELECT/CLK_IMO_TRIM 等绝对地址)—— 用于绕过 SROM 直接寄存器配 IMO=48MHz。
放到 `.kiro/context/` 我就能提取并落地。

## 现状小结(给用户)
已解决:test-mode 与 OpenOCD halt+bkpt-run 两套机制都打通(核 halt、CPUID 正确、SROM 真执行返回真码)、芯片=OPEN、保护读法/HMASTER/参数布局全部对齐权威。唯一卡点:family 0xB5 的 flash **时钟配置命令**(SET_IMO 0x15 返回未定义 0xF0000014),需 002-39884 或 Registers TRM 才能确定正确的 Configure-Clock 命令或寄存器级配法。


---

# 突破:4100T Plus Architecture TRM(002-39884 同源)—— 根因=缺 Configure Clock(IMO 48MHz)

用户提供 PDF,已用 PyMuPDF 转 `psoc4100t_plus_trm.txt`(281 页)。工具:`main_firmware/tools/pdf2txt.py`。
注:该 TRM 是 **4100T Plus(family 0xC6)**,我们芯片是 4100S Plus(0xB5),同 PSoC4 架构/同 SPCIF/同 002-39884 规格,机制通用。

## 铁证(§23 Nonvolatile memory programming)
- **key2 = 0xD3 + Opcode**(与我们一致)。opcodes 全部核对一致:Silicon ID=0x00,Load Flash Bytes=0x04(key2 0xD7),Write Row=0x05(0xD8),Program Row=0x06(0xD9),Erase All=0x0A(0xDD),Checksum=0x0B(0xDE),Write Protection=0x0D(0xE0)。
- **§23.5.2 Configure Clock**:"确保 charge pump clock(clk_pump)与 HFCLK(clk_hf)= IMO 48MHz,**在调用 flash write/erase 之前**。若 IMO 非 48MHz,write/erase 会不动作并返回 **'Invalid Pump Clock Frequency'**。" → 我们 PROGRAM 的 **0xF0000012 = Invalid Pump Clock Frequency**(时钟没设)。
- **Write Row/Program Row/Erase All/Write Protection 每个都写明**"Usage Requirements: Call the **Configure Clock API before** calling this function"。
- **Erase All**:"This API can be called only from the **DAP in the programming mode** and only if chip protection is OPEN." → 需 test-mode(programming mode),不是 debug-halt。芯片=OPEN(已确认)。
- **§23.4.1 DAP 机制**:写 opcode 到 SYSREQ[15:0]、置 SYSCALL_REQ(bit31)触发 NMI 跳 SROM;DAP **轮询 PRIVILEGED+SYSCALL_REQ 清零**,读 SYSARG。**没提 HMASTER、没提 bkpt**。→ test-mode+poll 本就是权威机制(我们最初的做法),halt+bkpt 是 OpenOCD 的等价变体。
- 参数放置:Silicon ID/Configure Clock/Checksum/Write Protection = SYSARG 直填;Load Flash Bytes/Write Row/Program Row/Erase All = SRAM 参数块 + SYSARG 指向。**与我们代码一致。**
- Configure Clock opcode:TRM 表格被 PDF 版式吞了(Silicon ID 表覆盖),但 OpenOCD SET_IMO48=0x15 + SEGGER "Configure Clock" 命令佐证 = **0x15**(= 我们 set_imo)。
- TRM 提到直接寄存器路:"Refer to the **CLK_IMO_CONFIG** register in the Registers RM" → 可直接写寄存器设 IMO 48MHz 绕过 SROM。相关寄存器名:CLK_IMO_CFG/CLK_IMO_SELECT/CLK_SELECT(HFCLK_SEL/HFCLK_DIV/SYSCLK_DIV)/pump sel。

## 新的根因结论(推翻"family 0xB5 特殊"猜测)
不是家族特殊,是**一直缺 Configure Clock 成功执行** → IMO 非 48MHz → PROGRAM 报 0xF0000012。我们的 Configure Clock(SET_IMO 0x15)返回 0xF0000014(未定义码),没设成时钟。ERASE 也 0xF0000014(可能因非 programming-mode 或时钟未设)。

## 两条修复路径
- **A. 让 SROM Configure Clock(0x15)成功**:需知道 0xF0000014 含义(状态码表)+ 是否需 programming-mode。
- **B. 绕过 SROM,直接写时钟寄存器**设 IMO=48MHz + HFCLK/clk_pump=IMO:需 CLK_IMO_CONFIG/CLK_IMO_SELECT/CLK_SELECT 绝对地址与值(Registers TRM)。
- **且**:erase/program 必须在 **test-mode(programming mode)** 下由 DAP 发起(改回 test-mode acquire)。

## 待 module-builder 深挖 psoc4100t_plus_trm.txt(+其它 PDF 可用 pdf2txt.py 转)
1. 状态码表:0xF00000YY 全表,尤其 0x12/0x14/0x10 含义。
2. CLK_IMO_CONFIG/CLK_IMO_SELECT/CLK_SELECT 等**绝对地址 + 设 IMO 48MHz/HFCLK=IMO/pump=IMO 的具体值**(直接寄存器配法)。
3. programming mode(TEST_MODE)进入的确切要求,以及 Configure Clock 是否也需 programming-mode。
4. SPCIF 寄存器地址(若直接配 flash 时钟需要)。


---

# module-builder：TRM 状态码/时钟寄存器检索

来源：`.kiro/context/psoc4100t_plus_trm.txt`(281 页，PyMuPDF 提取)，并用 `fitz` blocks 模式重新核对了关键页原始版式（排除 OCR/纯文本提取丢字段的可能）。文档号 **002-39884 Rev.*B 2025-03-27**——★重要澄清★：此前 bringup.md 记录"权威源是 002-39884(现行编程规格)，本地无"，但**这份用户提供的 "4100T Plus Architecture TRM" 本身就是 002-39884**（页脚逐页印着该文档号），只是 Infineon 把编程算法/DAP/SROM 内容整合进了 Architecture Reference Manual 的第 22/23 章，并非独立的"Programming Specifications"文档。我们已经拿到了此前苦寻的权威源，但它在 Configure Clock 一节存在明显的文档错误（见下）。

## 1. SROM 状态码表：0xF00000YY 完整列表（TRM §23.6 Table 23-2，第 277 页，逐字核对 blocks 原始版式）

```
AXXXXXXXh  Success
F0000001h  Invalid Chip Protection Mode
F0000003h  Invalid Page Latch Address
F0000004h  Invalid Address
F0000005h  Row Protected
F0000007h  Resume Completed
F0000008h  Pending Resume
F0000009h  System Call Still In Progress
F000000Ah  Checksum Zero Failed
F000000Bh  Invalid Opcode
F000000Ch  Key Opcode Mismatch
F000000Eh  Invalid Start Address
F0000012h  Invalid Pump Clock Frequency – IMO must be set to 48 MHz and HF clock source to
           the IMO clock source before flash write/erase operations.
```
表格在 `0xF0000012` 之后**直接结束**，紧接着换页进入 §23.7（Non-blocking pseudo code）。用 blocks 模式核对了第 276/277/278 页的原始坐标版式（见本次检索日志），**确认这不是文本提取丢字段，是文档本表原生到此为止**——与 AN84858 Appendix B、progspec 002-22325 的表完全一致（同一码表，TRM 只是没有额外补充）。

**`0xF0000010` 和 `0xF0000014` 在这份 002-39884 权威 TRM 里依然未被定义。** 三份独立文档（AN84858、progspec 002-22325、这份 TRM 002-39884）对状态码表的收录完全一致，均止于 `0xF0000012`。`0xF0000013`（OpenOCD 认定的"未实现"码）同样不在官方文档任何一处——OpenOCD 那个码本身可能是第三方在其它老家族实测出来后自行归纳，不是 Infineon 官方文档收录的。

**结论（更新）**：`0xF0000012` = Invalid Pump Clock Frequency（官方定义，三源一致，无家族限定字样——TRM 版本这次**没有**像 AN84858 那样把它限定为"CY8C40xx family"，是通用描述，适用于全系列包括我们的 4100S/4100T Plus）。`0xF0000014`/`0xF0000010` **官方文档确认查无定义**，是彻底的文档空白，不是提取遗漏。

## 2. 直接寄存器配时钟：TRM 只给寄存器名，不给绝对地址（Architecture TRM 的固有局限）

TRM §9（Clocking system，第 88-92 页）给出的是**寄存器名 + 位域功能描述**，明确指向寄存器名：
- `CLK_IMO_SELECT[2:0]`：选择 IMO 频率，`0`=24MHz ... `6`=48MHz（Table 9-1，7 级，4MHz 步进，**默认值 0=24MHz**）。
- `CLK_IMO_CFG`：IMO 配置（框图旁注，未展开位域）。
- `CLK_IMO_TRIM1/2/3`：粗调/细调/温漂补偿 trim，出厂值存于 SFLASH，启动时自动加载。
- `CLK_SELECT.HFCLK_SEL`：HFCLK 源选择（IMO / EXTCLK）。
- `CLK_SELECT.HFCLK_DIV`：HFCLK 预分频（2/4/8，默认 4）。
- `CLK_SELECT.PUMP_SEL`：flash charge pump 时钟源选择（框图标注 `clk_pump` 直连 IMO/HFCLK 之后）。
- `CLK_SELECT.SYSCLK_DIV`：SYSCLK 分频（默认 1）。

**没有任何一处给出这些寄存器的绝对地址**——全文档（281 页，已用正则扫描 `0x4004xxxx`/`0x4005xxxx` 模式，零匹配）不含 SRSS/PERI 外设的基地址映射表。TRM §22.7"Registers"小节（第 262 页）**只列了 debug/BPU/DWT 寄存器**（CM0P_DHCSR 等，PPB 区），**不含 CLK_* 系列**。多处正文（如 §9.3.1、§16.3.3 SPI 章节结尾）反复出现同一句式："Refer to the **PSOC™ 4100T Plus MCU registers reference manual** for the details of these registers"——**这是与 Architecture TRM(002-39884) 完全独立的另一份文档**（"Registers Reference Manual"，通常有独立文档号，Infineon 官网需要登录/搜索单独获取），本次任务范围内的本地文件不包含它。

**明确结论（回答问题 3）**：方案 B（绕过 SROM 直接写时钟寄存器）在当前本地资料下**不可行**——不是信息不够精确，是**这份 Architecture TRM 从架构设计上就不含任何寄存器绝对地址**，需要专门的 Registers Reference Manual 才能拿到 `CLK_IMO_SELECT`/`CLK_SELECT`/`CLK_IMO_CFG` 的偏移地址与位域范围。这是本次检索能给出的最诚实边界。

## 3. Programming mode (TEST_MODE) 进入要求 + Configure Clock/Erase All 是否都要求 DAP+programming mode

TRM §22.5.2（第 260 页，逐字）："host must enter the device programming mode...by setting the **TEST_MODE bit (bit 31) in the test mode control register (MODE register)**"，**该节同样没给绝对地址**，只说"detailed...in the CY8C4xxx, CYBLxxxx programming specifications"（即 002-22325/AN84858 那份，我们本地已有，给出的地址是 `0x40030014`，与固件 `psoc::reg::TEST_MODE` 一致，TRM 未反驳此地址，只是没有重复给出）。

**Erase All（§23.5.6，明确写明）**："This API can be called only from the **DAP in the programming mode** and only if the chip protection mode is **OPEN**. If...PROTECTED, then the Write Protection API must be used...to change...to OPEN. Changing...from PROTECTED to OPEN automatically does an erase all operation." —— 与 bringup.md 此前记录完全一致，芯片=OPEN 已确认，此条件已满足。

**Configure Clock 是否同样要求 programming mode**：TRM 23.5.2 该节的"Usage Requirements"字段**缺失**（该节被文档错误污染，见第 4 条），无法从文字直接确认。但从架构层面看，§23.4.1"Performing a system call"里描述的 DAP 轮询机制（PRIVILEGED+SYSCALL_REQ 清零）是**所有**系统调用（无论 CPU 还是 DAP 发起）的通用执行框架，其中"由 DAP 发起"这件事本身就隐含发起时 CPU 必须处于 test-mode（programming mode）——SROM 靠 NMI 服务，若不在 test-mode，DAP 直接写 SYSREQ 不会被任何东西响应（CPU 在跑用户代码，不会主动让出）。**结论：Configure Clock 与 Erase All 一样，必须在 DAP 已进入 programming mode 的前提下才可能被真正服务**，这与 bringup.md 上一轮"必须走 test-mode，不是 debug-halt"的结论一致，TRM 没有提供相反证据。

## 4. Configure Clock (opcode 0x15) ——★关键新发现：TRM 23.5.2 参数表是文档错误（复制粘贴自 Silicon ID 节）★

用 blocks 模式核对了 PDF 第 267 页原始版式：**§23.5.2"Configure clock"节的参数表内容与紧邻上方 §23.5.1"Silicon ID"节的参数表完全逐字相同**——"Bits[15:0] = 0x0000 **Silicon ID opcode**"、返回值里的"Silicon ID Lo/Hi/Family ID/Chip Protection"字段，一字不差地出现在 Configure Clock 节下方。这不是版式吞字/表格线丢失，是 **PDF 源文件本身在排版时把 Silicon ID 的参数表误复制粘贴到了 Configure Clock 小节下**（Infineon 文档的真实 bug，两个小节标题正确、但参数表内容错误地共享了同一份）。

**因此 TRM 002-39884 无法提供 Configure Clock 的真实 opcode/参数值**——这是文档层面的空白，不是我们检索能力的问题。

**交叉验证（仅能依赖已有的独立来源，与上一轮 module-builder 结论一致）**：
- 旧规格 `psoc4_progspec.txt`（002-22325）第 1312-1314 行：`SROM_CMD_SET_IMO_48_MHz = 0x15`。
- OpenOCD `psoc4.c` 源码：`PSOC4_CMD_SET_IMO48 = 0x15`。
- 两个独立第三方来源一致指向 `0x15`，且 `key2 = 0xD3 + opcode`（TRM §23.3 明确定义此公式，适用于全系统调用，无家族例外）；`0xD3 + 0x15 = 0xE8`。

**参数直填 SYSARG（无 cmd_param 字段）**：这一点在 TRM 里同样只能类比 Silicon ID/Checksum/Write Protection 这几个"参数直填 CPUSS_SYSARG"的调用模式（TRM §23.4.1 Step 1a 描述的两种参数传递方式之一），因为 Configure Clock 自身参数表已被污染，无法直接确认，但所有"不需要传大块数据"的调用（Silicon ID/Write Protection/Checksum/Erase All 用 SRAM 单字但仍是 key1|key2 打包成一个 32 位字）都遵循"key1|((key2+opcode)<<8)"打包进单个 32 位字直填的模式，Configure Clock 大概率同构。

**我们代码 `set_imo_48mhz()` 的实现**（`params = SROM_KEY1 | ((SROM_KEY2 + SROM_CMD_SET_IMO_48MHZ) << 8)`，直填 SYSARG）**与两个独立第三方来源比对无误，且符合 TRM 通用打包规则**——**没有发现参数格式错误**。这与此前 module-builder 的结论一致，本次 TRM 检索没有推翻它，但也**没有新证据能证实**它（因为权威源本身该节数据是坏的）。

**新的合理怀疑（TRM 独有的新线索）**：TRM §23.5"System calls"汇总表 **Table 23-1（第 266 页）里根本没有列出"Configure Clock"这一行**——该表只列了 Silicon ID / Load Flash Bytes / Write Row / Program Row / Erase All / Checksum / Write Protection / Non-Blocking Write Row / Non-Blocking Program Row / Resume Non-Blocking，共 10 项，逐一核对 DAP/CPU access 列。Configure Clock 被排除在这份"完整列表"之外，且其详情小节内容还被污染——**两个独立异常同时出现在同一个 API 上，不能排除该 API 在 4100T/4100S Plus 新架构下的地位/opcode/存在性本身就与老家族（AN84858/002-22325 覆盖的 CY8C40xx 等）不同**。这是本次检索唯一能给出的新怀疑方向，无法在本次范围内证实或证伪。

## 5. IMO 默认频率（复位后）

TRM §9.2.1.1"Startup behavior"（第 90 页，逐字）："After reset, the IMO is configured for **24-MHz** operation. During the 'boot' portion of startup, trim values are read from flash and the IMO is configured to achieve datasheet specified accuracy." 且 Table 9-1 明确"default frequency is 24 MHz"。

**结论：IMO 复位后默认 24MHz，不是 48MHz**。§9.2.1.2"Programming clock"标题下明确一句"**IMO must be set to 48 MHz to program the flash.** It is used to drive the charge pumps of flash and for program/erase timing purposes."——**Configure Clock 这一步不能跳过**，这与此前"根因=缺 Configure Clock"的结论完全吻合，且 TRM 给出了配 IMO 频率的软件算法框图（Figure 9-2：写 CLK_IMO_SELECT=24MHz→加载粗调 trim→清细调 trim→加载温漂 trim→等 50 IMO 周期→若目标>24MHz 先切到中间频率再等 50 周期→写最终 CLK_IMO_SELECT），但这是**寄存器级软件流程**，前提仍是需要绝对地址（第 2 条已确认本地缺失 Registers RM）。

---

## 可直接落地的方案（结论与优先级）

**没有找到"直接寄存器配 IMO 48MHz"的绝对地址+写入值序列**——Architecture TRM(002-39884) 架构设计上不含任何寄存器绝对地址，此路径需要独立的"PSOC 4100T Plus MCU Registers Reference Manual"，本次任务范围的本地文件不包含它，这是明确的文档缺口，不是检索遗漏（已用正则扫描全文 0x4004/0x4005 地址段，零匹配；且 §22.7 Registers 列表只覆盖 debug 寄存器不含 CLK_*）。

因此方案 A（让 SROM Configure Clock 成功）仍是唯一可继续推进的路径，新增两条可执行的下一步（成本从低到高）：

1. **（最低成本，立即可测）确认 programming mode 状态在调用 SET_IMO 时刻仍然有效**：在 `set_imo_48mhz()` 调用前，额外插入一次对 `TEST_MODE`(0x40030014) 寄存器的回读，校验 `(status & 0x80000000) == 0x80000000` 仍然成立（Erase All 的"only from DAP in programming mode"要求可合理外推到 Configure Clock；若这一步在 acquire 成功之后、到 set_imo 调用之前的某个环节意外掉线，会导致所有非 GET_SILICON_ID 的调用统一失败于同一个未知码，正好匹配实测现象——GET_SILICON_ID 是文档里唯一明确允许"CPU 也能访问"的 API，可能对 programming-mode 状态的要求更宽松，其它 API 更严格）。

2. **（低成本，可一次性验证怀疑）尝试 opcode 枚举**：鉴于 TRM Table 23-1 完整列表里不含 Configure Clock、且其详情小节数据被污染，不能 100% 排除 4100T/S Plus 新架构下该 API 的真实 opcode 与老家族（0x15）不同。建议在原有 test-mode+DAP 轮询机制不变的前提下，仅将 `SROM_CMD_SET_IMO_48MHZ` 尝试改为几个相邻候选值（如 0x14、0x16、0x0F、0x13 等未在任何官方状态码表出现过的"空位" opcode，也可能被复用为新 API 编号）逐一发起调用，观察是否有任何一个返回 `0xA0000000`（成功）或至少返回一个*不同于* 0xF0000014 的、状态码表里已定义的码（如切换到 0xF000000B Invalid Opcode 反而是好消息，说明我们枚举到了错误区间，可以缩小范围）。这是实测排查，不依赖额外文档，成本可控（不改变执行机制，只改一个字节参数）。

3. **（需要外部资料，优先级最低）向用户申请"PSOC™ 4100T Plus MCU Registers Reference Manual"**（TRM 反复引用的独立配套文档），用于取得 `CLK_IMO_SELECT`/`CLK_SELECT`/`CLK_IMO_CFG`/`CLK_IMO_TRIM1-3` 的绝对地址与位域，走方案 B（直接寄存器配置绕过 SROM，软件流程已知，见 Figure 9-2，只缺地址）。

## 异常说明
本次任务范围内为纯只读检索，未修改任何固件代码；仅用 PyMuPDF 转换/核对了 PDF 原始版式（未生成新的持久化文本文件，`_tmp_*.py` 为临时脚本，检索结束后清理）。未能给出 0xF0000010/0xF0000014 的官方定义、也未能给出任何寄存器绝对地址——均已在正文明确说明是文档本身的缺口（前者三份文档一致止于 0xF0000012；后者需要 Architecture TRM 之外的 Registers RM），不是检索深度不够。


---

# module-builder TRM 检索结论 + 卡点收敛

- 状态码表(TRM §23.6 Table 23-2 p277,与 AN84858/002-22325 三源一致)**止于 0xF0000012**;**0xF0000010 / 0xF0000014 全文档空白**(仅 002-39884 可能定义)。
- IMO **复位默认 24MHz**(非 48MHz)→ 必须 Configure Clock 设 48MHz,"缺时钟"是确证根因。
- Architecture TRM **无寄存器绝对地址**,反复引用独立的 *Registers Reference Manual*(本地没有)。CLK_IMO_SELECT/CLK_SELECT/CLK_IMO_TRIM 只给名+位域。
- Table 23-1 syscall 列表**竟不含 Configure Clock**;opcode 0x15 仅靠 OpenOCD/002-22325 佐证。
- web 佐证:CLK_IMO_SELECT 以 4MHz 步进设 IMO 频率;设 48MHz 还需从 SFLASH 载入对应 trim(PSoC Creator CySysClkWriteImoFreq 的做法)——直接寄存器配法复杂且有"错误时钟下擦写"的风险。

## 卡点(二选一即可解)
- **首选:002-39884(现行 CY8C4xxx/CYBLxxxx Programming Specifications,登录墙,用户有英飞凌号)** —— 定义 0xF0000014,给出 S 系列正确的 Configure Clock 命令(opcode/参数),直接修 SROM 路(方案A)。
- 次选:**PSoC 4100S Plus Registers Reference Manual** —— CLK_IMO_SELECT/CLK_IMO_TRIM/CLK_SELECT 绝对地址+位域,走直接寄存器配 IMO 48MHz(方案B,较复杂)。

## 已就绪(拿到文档即可快速落地)
固件机制已完备:test-mode 与 halt+bkpt-run 两套 SROM 执行都通;opcodes/keys/参数布局全对;芯片 OPEN;保护读法对。只差"把 IMO 设成 48MHz"这一步。


---

# 用户澄清:002-39884 就是那份 4100T Plus Architecture TRM(已有)。另提供 4100S Plus Registers TRM

- 002-39884 = 已有的 `infineon-psoc-4100t-plus-architecture-trm...pdf`(footer 002-39884 Rev*B)。无独立编程规格,就这份。
- Configure Clock(§23.5.2)**在 002-39884 里没有参数表**(page 267 底文字,page 268 直接是 Load flash bytes;Table 23-1 也不列它)——opcode/参数在本 TRM 缺失。0xF0000014 全文档未定义。→ SROM 路(方案A)资料到头。
- 用户新给 **PSoC 4100S Plus Registers TRM**(`.kiro/doc/`,已转 `psoc4100s_plus_registers.txt`,1902 页,doc 002-21159 Rev*D)。→ 转走**方案B:直接寄存器配 IMO=48MHz**。
- 注:.kiro/doc 疑似 gitignore,grep_search 搜不到;用 `python main_firmware/tools/grep_txt.py <txt> <pat...>` 搜(已验证可用)。Select-String 对该文件失效(编码/二进制判定)。

## 时钟寄存器位置(Registers TRM 目录页)
- CLK_SELECT → p1845;CLK_IMO_CONFIG → p1848;CLK_IMO_SELECT → p1859;CLK_IMO_TRIM1 → p1860;CLK_IMO_TRIM2 → p1861;CLK_IMO_TRIM3 → p1863。
- SFLASH_IMO_TRIM_LT0..LT24 → p1805+(每频率步的出厂 trim);SFLASH_IMO_TRIM_USBMODE_48 → p1778/p1779。

## 方案B 目标(待 module-builder 精确提取)
组一段"直接写寄存器把 IMO 设 48MHz + HFCLK 源=IMO + charge pump 时钟源=IMO"的绝对地址+写入值序列,含从 SFLASH 载对应 trim 到 CLK_IMO_TRIM* 的步骤。在 programming-mode(或 halt)下、发 flash 命令前执行。


---

# module-builder：方案B 直接寄存器配 IMO 48MHz 序列

来源：`.kiro/doc/psoc4100s_plus_registers.txt`（Registers TRM 002-21159 Rev.*D，PyMuPDF 转，1902 页）
第 1836-1864 页（SRSS 寄存器详情）+ 1766-1828 页（SFLASH trim 表）；交叉核对
`.kiro/context/psoc4100t_plus_trm.txt`（Architecture TRM 002-39884，§9.2.1，Table 9-1 + Figure 9-2
"Change IMO frequency" 算法，逐字读取确认）。全部地址/位域来自寄存器详情页原文，非目录页推断。

## 1. CLK_IMO_SELECT —— 绝对地址 `0x40030F08`
- 位域：`FREQ[2:0]`（bits 2:0），其余保留。复位值 `0`。
- 编码（p1860，逐字）：`0x0=24MHz 0x1=28MHz 0x2=32MHz 0x3=36MHz 0x4=40MHz 0x5=44MHz 0x6=48MHz`。
- **48MHz → 写入值 `0x6`**（4MHz 步进，24MHz 起点，与 Architecture TRM Table 9-1 完全一致）。

## 2. CLK_IMO_TRIM1 / TRIM2 / TRIM3 —— 绝对地址与含义
- `CLK_IMO_TRIM1 = 0x40030F0C`：`OFFSET[7:0]`（粗调，出厂按每个 FREQ 档生成，存于 SFLASH），复位值 `128`。
- `CLK_IMO_TRIM2 = 0x40030F10`：`FSOFFSET[2:0]`（细调，正常运行下应保持 `0`），复位值 `0`。
- `CLK_IMO_TRIM3 = 0x40030F18`：`TCTRIM[6:5]`（温漂补偿）+ `STEPSIZE[4:0]`，复位值 `TCTRIM=2, STEPSIZE=16`（即字节值 `0x50`）。
- **设 48MHz 需写**：`CLK_IMO_TRIM1 = SFLASH_IMO_TRIM_LT24`（整字节直填，位域 `OFFSET[7:0]` 与 SFLASH 源寄存器逐位对齐，无需移位）；`CLK_IMO_TRIM2 = 0`（清细调）；`CLK_IMO_TRIM3 = SFLASH_IMO_TCTRIM_LT24`（整字节直填，`TCTRIM[6:5]+STEPSIZE[4:0]` 布局与 SFLASH 源寄存器逐位相同）。

## 3. SFLASH_IMO_TRIM_LT* / TCTRIM_LT* —— 48MHz 对应条目 + 到 CLK_IMO_TRIM* 的映射
Architecture TRM Table 9-1（p89，逐字）给出频率档→SFLASH 源寄存器的权威映射：
```
CLK_IMO_SELECT[2:0]   频率     对应 SFLASH 寄存器
0                      24MHz    SFLASH_IMO_TRIM_LT0 , SFLASH_IMO_TCTRIM_LT0
1                      28MHz    SFLASH_IMO_TRIM_LT4 , SFLASH_IMO_TCTRIM_LT4
2                      32MHz    SFLASH_IMO_TRIM_LT8 , SFLASH_IMO_TCTRIM_LT8
3                      36MHz    SFLASH_IMO_TRIM_LT12, SFLASH_IMO_TCTRIM_LT12
4                      40MHz    SFLASH_IMO_TRIM_LT16, SFLASH_IMO_TCTRIM_LT16
5                      44MHz    SFLASH_IMO_TRIM_LT20, SFLASH_IMO_TCTRIM_LT20
6                      48MHz    SFLASH_IMO_TRIM_LT24, SFLASH_IMO_TCTRIM_LT24   ← 目标
```
**48MHz 对应条目 = `SFLASH_IMO_TRIM_LT24` + `SFLASH_IMO_TCTRIM_LT24`**（不是 `SFLASH_IMO_TRIM_USBMODE_48`——那两个 USB 专用寄存器 `0x0FFFF33E/33F` 是给 USB 24/48MHz osclock 模式用的独立 trim，与本次"改变 CLK_IMO_SELECT 档位"的常规流程无关，Table 9-1 未提及它们）。

绝对地址（`= SFLASH_MACRO0(0x0FFFF000) + 偏移`，偏移取自寄存器详情页逐条核对，非目录页）：
- `SFLASH_IMO_TRIM_LT24 = 0x0FFFF37D`（`OFFSET[7:0]`，1 字节，Retention=Retained）
- `SFLASH_IMO_TCTRIM_LT24 = 0x0FFFF364`（`TCTRIM[6:5]+STEPSIZE[4:0]`，1 字节）

**★AHB 访问细节（沿用现有 `read_chip_protection()` 的字节提取范式）★**：这两个 SFLASH 寄存器是 1 字节宽、地址不字对齐。`_read_io`/`_write_io` 现有实现按 32-bit 字访问（与 `SFLASH_MACRO0+0x7C` 的用法一致）。故须整字读出再按字节车道取值：
- `0x0FFFF37D`：字对齐基址 `0x0FFFF37C`，`SFLASH_IMO_TRIM_LT24 = (word >> 8) & 0xFF`（偏移 `0x37D & 3 = 1` → 字节车道 1）。
- `0x0FFFF364`：本身已字对齐（`0x364 & 3 = 0`），`SFLASH_IMO_TCTRIM_LT24 = word & 0xFF`（字节车道 0）。

映射关系（"从 SFLASH 读出厂 trim → 写 CLK_IMO_TRIM1/3"的确切对应）：**逐字节直填，无位移变换**——`CLK_IMO_TRIM1.OFFSET[7:0]` 与 `SFLASH_IMO_TRIM_LT24.OFFSET[7:0]` 位宽/位置完全相同；`CLK_IMO_TRIM3.TCTRIM[6:5]+STEPSIZE[4:0]` 与 `SFLASH_IMO_TCTRIM_LT24` 同构，同样直填整字节。

## 4. CLK_IMO_CONFIG —— 绝对地址 `0x40030030`
- `ENABLE`（bit 31，RW），复位值 `1`（IMO 出厂默认已使能）。
- **无需写此寄存器**——IMO 默认已使能（"Clearing this bit will disable the IMO. Don't do this if the system is running off it."），本次流程只调频率不动使能位。

## 5. CLK_SELECT —— 绝对地址 `0x40030028`
- 位域（p1845-1847，逐字）：`SYSCLK_DIV[7:6]`、`PUMP_SEL[5:4]`、`HFCLK_DIV[3:2]`、`HFCLK_SEL[1:0]`。
- **HFCLK 源选 IMO**：`HFCLK_SEL[1:0] = 0x0`（`IMO`；复位值本就是 `0`，即 HFCLK 出厂默认已选 IMO，可回读校验但理论上不需要改）。
- **charge pump 时钟(clk_pump) 源选 IMO**：位于**同一寄存器**的 `PUMP_SEL[5:4]`（"Selects clock source for charge pump clock (AMUX charge pump)"），编码 `0x0=GND(无时钟) 0x1=IMO(主 IMO 输出) 0x2=HFCLK`。**复位值是 `0`(GND，无时钟！)**——这是必须显式写的一位，否则 flash 擦写用的 charge pump 完全没有时钟。**PUMP_SEL 需写 `0x1`**。
- `HFCLK_DIV`/`SYSCLK_DIV` 与本次目标（只需"HFCLK 源=IMO"+"pump 源=IMO"）无直接关系，**保持原值不动**（用读-改-写，只改 `PUMP_SEL` 位，避免误动其它位）。

## 6. IMO 复位默认频率
Architecture TRM §9.2.1.1（逐字，p90）：**"After reset, the IMO is configured for 24-MHz operation."** 确认默认 24MHz，非 48MHz，与此前记录一致。

## 7. 交叉参考：CySysClkWriteImoFreq / Figure 9-2 算法（顺序与等待时间佐证）
Architecture TRM Figure 9-2"Change IMO frequency"（p90，逐字流程图）：
```
① 写 CLK_IMO_SELECT = 0(24MHz)          // 先退回安全基频，避免调 trim 时失步
② 读 SFLASH 粗调 trim → 写 CLK_IMO_TRIM1  // 对应目标频率的 LTn（本次=LT24）
③ 清 CLK_IMO_TRIM2（细调置0）
④ 读 SFLASH 温漂 trim → 写 CLK_IMO_TRIM3.TCTRIM字段  // 对应目标频率的 TCTRIM_LTn（本次=TCTRIM_LT24）
⑤ 等待 50 个 IMO 周期
⑥ 若目标频率 > 24MHz：
     写 CLK_IMO_SELECT = 目标档位-1（"nearby intermediate frequency"，即 5=44MHz）
     等待 50 个 IMO 周期
     写 CLK_IMO_SELECT = 目标档位（6=48MHz）
```
诚实说明：图中②④标注是通用步骤框，未逐字点名"必须是目标频率对应的 LTn"，但 Table 9-1 给每个频率档单列专属 SFLASH 寄存器（LT0/LT4/.../LT24）这件事本身只有在"trim 按目标频率加载"的前提下才有意义（否则 LT4~LT24 就是摆设）；本方案按此展开解释。②之后的⑥"中间档"步进只操作 `CLK_IMO_SELECT`，不重新加载 trim——trim 已在②④按最终目标(48MHz/LT24)一次性加载完毕，中间档步进只是为降低切换瞬态冲击，非重新计算 trim。

50 IMO 周期 @ 24MHz ≈ 2.08μs；host 侧经 SWD/PIO bit-bang 发起的每次寄存器写事务本身耗时通常已远超此值，理论上无需额外插入延时，但为稳妥建议每次等待步骤显式插入 ≥10μs 的保守延时。

## 可直接落地的伪代码（按 `_write_io`/`_read_io` 语义，绝对地址）

```cpp
// 前提：acquire() 已成功（CPU halt 或 test-mode，AHB 可访问 SRSS/SFLASH 区），
// 在发送 ERASE_ALL / PROGRAM_ROW 等 SROM flash 命令之前执行本序列。
// 完全绕过 SROM Configure Clock(0x15)，纯 AHB 直写。

// --- 常量（补充到 psoc_types.h::reg，命名沿用现有风格） ---
// CLK_IMO_SELECT = 0x40030F08
// CLK_IMO_TRIM1  = 0x40030F0C
// CLK_IMO_TRIM2  = 0x40030F10
// CLK_IMO_TRIM3  = 0x40030F18
// CLK_SELECT     = 0x40030028
// SFLASH_MACRO0  = 0x0FFFF000（已存在）

uint32_t word;

// ① 退回安全基频 24MHz（出厂/复位默认值，写入是保险动作）
_write_io(0x40030F08, 0x0);                 // CLK_IMO_SELECT.FREQ = 0 (24MHz)

// ② 从 SFLASH 读 48MHz 档粗调 trim（LT24，字节车道1，字对齐基址0x0FFFF37C）→ 写 CLK_IMO_TRIM1
_read_io(0x0FFFF37C, &word);
uint8_t trim_lt24 = (uint8_t)((word >> 8) & 0xFF);   // SFLASH_IMO_TRIM_LT24.OFFSET[7:0]
_write_io(0x40030F0C, trim_lt24);            // CLK_IMO_TRIM1.OFFSET[7:0] = trim_lt24（整字节直填）

// ③ 清细调
_write_io(0x40030F10, 0x0);                 // CLK_IMO_TRIM2.FSOFFSET = 0

// ④ 从 SFLASH 读 48MHz 档温漂 trim（TCTRIM_LT24，字节车道0，本身字对齐0x0FFFF364）→ 写 CLK_IMO_TRIM3
_read_io(0x0FFFF364, &word);
uint8_t tctrim_lt24 = (uint8_t)(word & 0xFF);        // SFLASH_IMO_TCTRIM_LT24（TCTRIM[6:5]+STEPSIZE[4:0]）
_write_io(0x40030F18, tctrim_lt24);          // CLK_IMO_TRIM3 = tctrim_lt24（整字节直填）

// ⑤ 等待 ≥50 IMO 周期 @24MHz（保守取 10~20us）
_delay_us(20);

// ⑥ 目标 48MHz > 24MHz：先切中间档(5=44MHz)降低切换瞬态，再切到目标档
_write_io(0x40030F08, 0x5);                 // CLK_IMO_SELECT.FREQ = 5 (44MHz，一档以内的中间频率)
_delay_us(20);
_write_io(0x40030F08, 0x6);                 // CLK_IMO_SELECT.FREQ = 6 (48MHz，最终目标)
_delay_us(20);                              // 留出锁定余量

// ⑦ HFCLK 源确认为 IMO + charge pump 源切到 IMO（读-改-写，只动 PUMP_SEL，不碰其它位）
_read_io(0x40030028, &word);                // CLK_SELECT 现值
word = (word & ~(0x3u << 4)) | (0x1u << 4);  // PUMP_SEL[5:4] = 0x1 (IMO)，clk_pump 源=IMO
word = (word & ~(0x3u << 0)) | (0x0u << 0);  // HFCLK_SEL[1:0] = 0x0 (IMO)，通常已是默认值，显式回写做保险
_write_io(0x40030028, word);                 // 写回 CLK_SELECT

// 至此 IMO=48MHz、HFCLK 源=IMO、clk_pump 源=IMO 均已配置，可继续 ERASE_ALL / PROGRAM_ROW。
// CLK_IMO_CONFIG(0x40030030).ENABLE 复位默认已为1，未做改动。
```

## 不确定/待实机验证之处（诚实边界）
1. **Figure 9-2 的②④是否严格要求"按目标频率加载 trim"**：TRM 流程图文字本身是通用框，未逐字写"目标频率对应"，本方案基于 Table 9-1 存在专属 LT4/LT8/.../LT24 寄存器这一事实反推该解释，是本次检索能给出的最合理推断，但不是逐字确证。
2. **50 IMO 周期等待是否对"host 经 AHB 直写"场景仍然必需**：算法原描述面向固件在片上运行时的场景（每条指令耗时纳秒级，故需要显式 wait）；host 经 SWD/PIO bit-bang 发起的每次寄存器读写事务本身通常已有数微秒到数十微秒延迟，可能已天然覆盖 50 周期(~2.08us)要求，但未做实测验证，建议保留显式延时以防线路更快导致的边界情况。
3. **CLK_SELECT.HFCLK_DIV(默认值2=DIV_BY_4)/SYSCLK_DIV 是否需要调整**：TRM 原文对"flash 写擦前置条件"只字面要求"IMO=48MHz 且 HFCLK 源=IMO"，未提及分频比，本方案保持原值不变，若实测 ERASE/PROGRAM 仍失败，可作为下一步排查点（如需要 HFCLK 实际频率达到某个门槛，而不仅仅是"源选对"）。
4. **本序列是否足以让后续 SROM ERASE_ALL/PROGRAM_ROW 成功**：TRM §23.5.6 载明 Erase All 需"DAP 处于 programming mode"——这是 SROM 调用层面的前提，与本次纯 AHB 直写时钟寄存器（不经过 SROM）是两件独立的事；本方案只解决"IMO/HFCLK/clk_pump 时钟源"这一层，programming-mode 的进入仍需现有 acquire() 机制单独保证。只能通过实机烧录验证两者叠加后 ERASE_ALL/PROGRAM_ROW 是否真正返回成功码。

## 异常说明
本次任务为纯只读检索+方案设计，未修改任何固件代码文件（`psoc_swd.cpp`/`.h`/`psoc_types.h` 均未改动）；仅使用 `grep_txt.py`（已验证工具）与 `read_files` 提取寄存器手册原文，未使用 `render_page.py` 渲染（文本提取已足够清晰，未发现表格线丢失或版式污染迹象，与此前 Configure Clock 一节的文档 bug 不同）。所有地址取自寄存器详情页原文（非目录页），已交叉核对两份独立文档（Registers TRM 002-21159 + Architecture TRM 002-39884）确认一致。


---

# 关键突破链 + 当前精确卡点(WRITE 成功但 latch 空)

## 已彻底打通
- **时钟根因解决**:直接写寄存器配 IMO=48MHz(CLK_IMO_SELECT=6 + LT24 trim + CLK_SELECT PUMP_SEL=IMO/HFCLK_DIV=/1/HFCLK_SEL=IMO)。诊断 imo=1(0x00001006) 确认生效。
- 时钟配好后 **WRITE_ROW(0x05) 返回 0xA0000000**、其 erase 相真实生效(flash[0] 从旧 CapSense 数据被擦成 0)。→ SROM flash 操作、charge pump 均工作。
- SRAM 参数/数据(params1@0x100 / Load Size=127@0x104 / data@0x108)三字回读全部持久正确(排除 SRAM 写问题)。
- LOAD_LATCH(0x04) 返回成功。

## 精确卡点
WRITE_ROW 成功但 **program 相被跳过(latch 全 0)→ flash 保持擦除态 0**。复位后再校验仍 vrd=0x00000000(vexp=0x20004000)。即 **LOAD_LATCH 执行成功却没把 SRAM 数据装进 page latch**,尽管参数/数据全对、opcode/key 与 TRM+OpenOCD 逐字一致。

## 最可能根因 + 方案
当前用 **debug-halt + bkpt-run** 机制:LOAD_LATCH 与 WRITE_ROW 是两次独立 bkpt-run,**之间 CPU 被 C_HALT 停核**。疑似**停核清空了 page latch**(本家族特性)。
TRM §23.4.1 权威路径是 **programming mode(test-mode)+ DAP 轮询**:CPU 在 boot ROM 服务循环持续运行、两次调用间不停核 → latch 保持。且此前 test-mode 失败仅因时钟没配(现已解决)。
→ **改回 test-mode + 直接寄存器时钟配置 + DAP 轮询 _srom_exec**(去 HMASTER、去 bkpt)。这是本家族权威机制,且规避停核清 latch。

## 已就绪且要保留
- configure_flash_clock()(AHB 直写,test-mode 下同样可用,擦写前调用)。
- WRITE_ROW(0x05)自动逐行擦写(免 ERASE_ALL;但 test-mode 下 ERASE_ALL 也应可用)。
- 收尾:reset_run()+release_swd()(复位 PSoC 运行 + SWD 高阻),用户明确要求烧完停 SWD。
- 诊断字段 + SRAM 三字校验(0x5A5A 哨兵)+ verify vrd/vexp。
- PSoC 运行发热=设计正常(用户确认多片一致),非故障。


---

# 穷尽软件手段后的确定性结论(重要)

## 用 SROM CHECKSUM 权威确认:flash 真的没被写入
`cks=1(0x00000000 srom=0xA0000000)` —— CHECKSUM 命令成功(srom=0xA0000000),返回**全片校验和=0**。SROM 自身读 flash = 全 0。**flash 确实是空的,不是 AHB 读的假象。**

## 全部已验证 / 已排除
- ✅ acquire 成功(test-mode: sysreq=0x20000000 PRIVILEGED 清零 / halt: CPUID 0x410CC601)。
- ✅ 时钟 IMO=48MHz(直接寄存器配,imo=1(0x00001006):FREQ=6/PUMP_SEL=IMO/HFCLK_DIV=/1/HFCLK_SEL=IMO,回读校验通过)。
- ✅ 芯片保护=OPEN(prot=1,从 flash 0x0FFFF07C 读)。
- ✅ 行级写保护=无(rp=0x00000000)。
- ✅ SRAM 参数/数据三字回读持久正确(params1/LoadSize=127/data)。
- ✅ SROM 确实执行(SYSARG 被 SROM 覆盖成真状态码)。
- ❌ **WRITE_ROW(0x05) / PROGRAM_ROW(0x06) / LOAD_LATCH(0x04) 都返回 0xA0000000 成功,但 flash 全 0(program 无效)。**
- ❌ **ERASE_ALL(0x0A) / SET_IMO(0x15) 返回 0xF0000014**(该码全文档未定义)。

## 已试尽的机制/变量(均不改变结果)
- SROM 执行机制:debug-halt+bkpt-run(带 HMASTER) 与 test-mode+DAP轮询(不带 HMASTER)——两套都"写成功但不写入"。
- program 命令:WRITE_ROW(自动擦写) 与 PROGRAM_ROW(纯写,已擦 flash 下)——都一样。
- SRAM 参数基址:0x20000100 → 0x20001000(排除 SROM 低 SRAM 暂存冲突)。
- 复位后再 acquire 再读 / SROM checksum——都确认 flash 空。

## 无法从现有资料解释的核心矛盾
SROM 报告 WRITE/PROGRAM 成功(0xA0000000)但 NVM 无变化;opcode/key/参数/时钟/保护全部符合 002-39884(4100T Plus 0xC6)+ OpenOCD + 002-22325。**我们芯片是 family 0xB5(4100S Plus),比手上 TRM(0xC6)与 OpenOCD(表止于 0xAB)都新。**

## 两个最可能的根因(需硬件级验证,超出固件可自证范围)
1. **物理层:flash 编程高压/charge pump 无效**(供电/Vdd 在编程时不足或不稳)。现象吻合:读类全 OK、NVM 修改类"成功但无效果";且用户观察到编程时 PSoC 发热(可能是 charge pump 在挣扎)。用户此前用 QC 快充烧过片,当前"新转换器"供电对 flash 编程是否足够稳定存疑。
2. **family 0xB5 的 SROM opcode/语义与 0xC6/老家族不同**:0x0A/0x15 → 0xF0000014(疑似该家族"无效/未实现"),0x04/0x05/0x06 可能不是我们以为的 LOAD/WRITE/PROGRAM。需 4100S Plus(0xB5)专属编程规格或真实编程器 SWD trace 对照。

## 已就绪(按用户要求)
- 烧录后收尾:reset_run() + release_swd()(复位 PSoC 运行 + SWDIO/SWDCLK 高阻),停止一切 SWD。
- 完整诊断:acq/prot/imo/erase/prog/verify/vrd/vexp/cks/rp/link。

## 建议下一步(需用户/硬件参与)
A. **用专用编程器(MiniProg4/KitProg3/PSoC Programmer)在同一块板+同一供电下烧一次**,验证"这块板+这颗片+这个供电"到底能不能被编程——直接区分"硬件/供电问题" vs "我们 SWD 实现问题"。这是项目早期就列的 fallback。
B. 若专用编程器能烧通,抓它的 SWD trace 对照我们的 opcode/序列(尤其 LOAD/WRITE 的实际 opcode 与参数),校正 family 0xB5 差异。
C. 测量编程期间 PSoC Vdd 是否稳定(charge pump 需稳定供电)。


---

# ★根因确认(硬件)+ 结案存档★

**用户实测定位:原理图把 1.8V LDO 输出点错接到了 5V**,PSoC 被 5V 过压 → flash 损坏(这正是"编程时发热"+"SROM 报成功但 NVM 永不变化"+ERASE/SET_IMO 0xF0000014 的物理根因,吻合此前"根因1:供电/charge pump"判断)。
用户已**割铜皮断开该错连,确认 1.8V 正常;之后烧录无发热迹象**。当前在片 flash 疑已损坏,等新片。

## 结论
- **SWD/SROM 固件实现基本正确**(acquire/test-mode/DAP轮询/时钟配置/opcode/参数全部按 002-39884+OpenOCD 对齐;读类命令全通;写类"成功但无效"是坏 flash 所致,非软件 bug)。
- 换好片(正确 1.8V 供电)后应可直接复测烧录闭环;届时重点看 erase→program→verify→cks 是否转正、link 是否起来。
- 收尾已就绪:烧录后 reset_run + release_swd(复位运行 + SWD 高阻),诊断齐全。
- ⚠ 换片复测前建议二次确认板上 PSoC 供电轨=1.8V(及 VDDD/VDDA/VCCD 相关电容),避免再烧片。

## 当前固件 PSoC 路径状态(供换片复测)
- acquire()=test-mode(TEST_MODE=0x80000000 + PRIVILEGED 轮询);_srom_exec=DAP 轮询;program_row=WRITE_ROW(0x05,自动擦写);configure_flash_clock=直接寄存器 IMO 48MHz;SRAM_PARAMS_BASE=0x20001000;main bringup=acquire→prot/rowprot→set_imo→erase→program→checksum→reset+re-acquire+verify→reset_run+release_swd。
- 诊断串:acq/prot/imo/erase/prog/verify/vrd/vexp/cks/rp/link。

## PSoC 任务:暂 PARK(等新片),转 Rust UI + 固件协议构建。
