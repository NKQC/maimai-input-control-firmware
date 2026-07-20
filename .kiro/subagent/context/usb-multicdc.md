# usb-multicdc 任务上下文（最终，任务已完成）

## 任务目标（达成）
main_firmware 按 mode.work 二选一复合枚举：恒定 config CDC + (Serial模式2CDC 或 HID模式1HID)。
只做 USB 拓扑 + HAL 通路，不实现 host_cmd/mai2serial/mai2light 业务逻辑。

## 结论：`pio run -e pico` 编译链接通过（RAM 10.3% / Flash 14.6%），无新增警告。

## 涉及/改动文件
- 新建 `main_firmware/src/hal/usb/tusb_config.h`：CFG_TUD_CDC=3,HID=1,MSC=1(保持框架默认,见坑1)。
- `main_firmware/platformio.ini`：build_flags 增加
  `-DCFG_TUSB_CONFIG_FILE=\"tusb_config.h\"` + `-Isrc/hal/usb`（方案4，见坑3解法）。
- `main_firmware/src/hal/usb/hal_usb.h`：新增 `UsbWorkMode{WORK_SERIAL,WORK_HID}`、
  `UsbCdcPort{CDC_CONFIG,CDC_SERIAL,CDC_LIGHT,CDC_PORT_COUNT_}`枚举；`HAL_USB_Device`新增按端口读写重载
  `cdc_write/read/available/flush(UsbCdcPort,...)`；CDC 环形缓冲改为按端口数组
  `cdc_rx_buffer_[CDC_PORT_COUNT][CDC_BUFFER_SIZE]`。
- `main_firmware/src/hal/usb/hal_usb.cpp`：新增两套配置描述符
  `desc_configuration_serial`（3×CDC）/`desc_configuration_hid`（1×CDC+1×HID）；
  `tud_descriptor_configuration_cb`按`ConfigManager::get_uint8("mode.work")`选择；
  字符串描述符表扩到8项；`_handle_cdc_rx(itf)`按itf分流到对应端口缓冲；
  include了`service/config_manager/config_manager.h`。

## 端点/接口映射表（终稿）

### Serial 模式（mode.work=0，默认）—— 6接口/9端点
| ITF | 类型 | 说明 | 字符串idx |
|---|---|---|---|
| 0 | CDC comm | config notif | 4 "mai2 config" |
| 1 | CDC data | config data | - |
| 2 | CDC comm | serial notif | 5 "mai2 serial" |
| 3 | CDC data | serial data | - |
| 4 | CDC comm | light notif | 6 "mai2 light" |
| 5 | CDC data | light data | - |

EP: config{notif=0x81,out=0x02,in=0x83} / serial{notif=0x84,out=0x05,in=0x86} / light{notif=0x87,out=0x08,in=0x89}
IN号:{1,3,4,6,7,9} OUT号:{2,5,8}，共9个数据端点(不含EP0)，远低于RP2040上限。

### HID 模式（mode.work=1）—— 3接口/4端点，=原文件平移
| ITF | 类型 | 说明 | 字符串idx |
|---|---|---|---|
| 0 | CDC comm | config notif | 4 "mai2 config" |
| 1 | CDC data | config data | - |
| 2 | HID | 触摸+键盘(hid_report_descriptor不变) | 7 "Mai Control HID" |

EP: config同上；HID=0x84(IN)。

字符串表(两变体共用)：0=lang,1=manufacturer,2=product,3=serial,
4="mai2 config",5="mai2 serial",6="mai2 light",7="Mai Control HID"。

## mode.work 选择实现方式与时机
- `ConfigManager::get_uint8("mode.work")`：0=Serial(默认)/1=HID（`app_config.cpp`已注册，范围0-1）。
- 正常启动路径：`app_config_register_schema()→ConfigManager::initialize()→...→HAL_USB_Device::init()`，
  config 在 USB 枚举（tud_connect后主机才发描述符请求）前已就绪，安全。
- 例外分支 `SWD_RELEASE_TO_EXTERNAL`(当前常量false，未生效)跳过 ConfigManager::initialize()直接init USB：
  此时 key 未注册，`get()`fallback返回`ConfigValue(false)`(类型非UINT8)，`get_uint8`类型检查失败返回0，
  天然回退到 Serial 模式，无需特殊处理。

## tusb_config 覆盖机制（含踩坑与最终方案）
真正生效的框架默认文件是 `framework-arduinopico/tools/libpico/tusb_config.h`，CFG_TUD_CDC/HID为**裸#define
无#ifndef保护**，命令行`-D`无法覆盖，必须用TinyUSB官方`CFG_TUSB_CONFIG_FILE`重定向机制。

**踩坑与解法**（详见下方"关键决策"，供后续任务/其他固件改动参考）：
1. **CFG_TUD_MSC不能改0**：`framework-arduinopico/lib/rp2040/libpico.a`是预编译库，内含用默认配置
   (MSC=1)编译好的`usbd.c.o/cdc_device.c.o/hid_device.c.o/msc_device.c.o/dcd_rp2040.c.o`。若本工程把
   MSC改0，工程侧重新编译的Adafruit_TinyUSB_Arduino版本不再提供`tud_msc_*_cb`默认弱实现，但链接器仍会
   从libpico.a拉入需要这些符号的msc_device.c.o，导致`undefined reference`。**解法**：CFG_TUD_MSC保持
   框架默认值1不变，只改CDC(3)/HID(1)。
2. **枚举值避开裸名SERIAL/HID**：Arduino `ArduinoCore-API/api/Common.h`把`SERIAL`宏定义为`0x0`，会在
   预处理阶段污染同名枚举值导致语法错误。**解法**：改用`WORK_SERIAL/WORK_HID`、`CDC_CONFIG/CDC_SERIAL/
   CDC_LIGHT/CDC_PORT_COUNT_`。
3. **`-DCFG_TUSB_CONFIG_FILE=\"src/hal/usb/tusb_config.h\"`（带相对路径）会导致MSC链接错误重现**，
   即使tusb_config.h里MSC值本身没变。根因未来得及100%坐实（推测是该宏在不同编译单元cwd下相对路径展开
   不一致，导致工程侧文件和Adafruit库侧文件对同一份config解析出不同实际内容/或部分单元fallback到
   框架默认，造成ABI/接口不一致）。**最终解法（已验证生效）**：改用**不带路径的纯文件名**
   `-DCFG_TUSB_CONFIG_FILE=\"tusb_config.h\"` + 单独加一条 `-Isrc/hal/usb` 到build_flags，让所有编译
   单元（含框架/库）都通过统一的include search path找到同一份文件，不依赖相对路径展开的cwd语境。
   验证：clean后完整编译成功，RAM 10.3%/Flash 14.6%，无新增警告。

## 剩余的硬件验证步骤（未做，需用户在场烧录）
本任务未烧录（规则要求，涉及硬件与用户在场）。烧录后应观察：
1. 设备管理器应始终出现一个 "mai2 config" CDC 端口（不论mode.work取值）。
2. mode.work=0(Serial,默认)：额外出现 "mai2 serial" + "mai2 light" 两个CDC端口，无HID设备。
3. mode.work=1(HID)：额外出现一个HID设备(触摸屏+键盘)，无serial/light CDC端口。
4. 修改mode.work（需后续host_cmd CFG_*命令支持，本任务未实现）并重启，验证枚举拓扑随之切换。
5. 用串口终端测试"mai2 config"端口能否与Rust上位机的host_cmd协议正常握手(本任务只铺通路,未验证协议层)。

## 关键决策记录（累加，最终版）
- 不改VID/PID。
- CFG_TUD_MSC保留框架默认值1，不要改成0（坑1）。
- 枚举值避开裸名SERIAL/HID（坑2，Arduino宏污染）。
- CFG_TUSB_CONFIG_FILE用"纯文件名+独立-I搜索路径"而非"带相对路径的字符串"（坑3，已验证生效）。
- mode.work读取时机验证安全，无需改main.cpp调用顺序。
- HID模式变体的接口/端点布局=原文件平移（风险最低）。
- 本任务只铺USB拓扑+HAL读写通路，未触碰host_cmd/mai2serial/mai2light业务逻辑，留给后续任务对接
  `HAL_USB_Device::cdc_write/read/available/flush(UsbCdcPort::CDC_SERIAL/CDC_LIGHT,...)`。

## 任务状态：已完成，等待主agent/用户验收 + 后续硬件烧录验证。


---

# 追加(主 agent 手动):Code 10 根因修复 —— libpico 双 TinyUSB 栈冲突(方案B)

## 症状
上一版编译通过,但烧录后 Windows 报「该设备无法启动 (代码 10)」(默认 Serial=3×CDC 枚举失败)。

## 根因(带证据)
- 框架预编译库 `framework-arduinopico/lib/rp2040/libpico.a` **自带整套 TinyUSB 设备栈**,用框架默认 `tools/libpico/tusb_config.h` 编译:**CFG_TUD_CDC=1 / HID=2 / MSC=1**。
- 项目又编译了 Adafruit TinyUSB 3.7.1(用本工程 tusb_config.h,CFG_TUD_CDC=3)。
- 两套同名符号混链;链接顺序里 libpico(CDC=1)的 `usbd/cdc_device` 若胜出,`_cdcd_itf[CFG_TUD_CDC]` 只有 1 槽 → 打不开描述符声明的第 2/3 个 CDC 接口 → SET_CONFIG 失败 → Code 10。之前 CFG_TUD_MSC 改 0 触发的 undefined-ref 是同一根因旁证。

## 修复(方案B,不改 TinyUSB)
- 新增 `main_firmware/strip_libpico.py`(post extra_script):把 `libpico.a` 复制成 build 目录下 `libpico_nousb.a`,用 `arm-none-eabi-ar d` 删掉其中 18 个会与 Adafruit 重复的 TinyUSB 成员(dcd_rp2040/rp2040_usb/usbd/usbd_control/audio_device/cdc_device/dfu_device/dfu_rt_device/hid_device/midi_device/msc_device/ecm_rndis_device/ncm_device/usbtmc_device/vendor_device/video_device/tusb/tusb_fifo),再把 `env["LIBS"]` 里的 libpico 换成该副本。**保留 `rp2040_usb_device_enumeration.c.o`**(Pico SDK 独有,Adafruit 无,删则 undefined)。框架包原件不动。
- `platformio.ini` [env:pico] 加 `extra_scripts = post:strip_libpico.py`。
- 脚本顺带 `-Wl,-Map=$BUILD_DIR/firmware.map` 出链接表供核对。

## 静态验证(已通过,无需烧录即可确认单栈)
- `pio run -e pico` 干净链接通过(RAM 10.3%/Flash 14.6%),无 undefined ref(证明 Adafruit 提供了全部被删符号)。
- map 核对:`usbd/cdc_device/hid_device/msc_device/dcd_rp2040/tusb/tusb_fifo` 全部来自 `lib60b\Adafruit_TinyUSB_Arduino`(CDC=3);**无一来自 libpico**;链接的 libpico 为剥离版 `libpico_nousb.a`;`rp2040_usb_device_enumeration.c.o` 仍来自 `libpico_nousb.a`(正确保留)。

## 待用户硬件验证
烧录此版,设备管理器应见:恒定 "mai2 config" CDC + (默认 Serial 模式)"mai2 serial"/"mai2 light" 两个 CDC 口,无 HID。若仍 Code 10,则根因不止双栈(需查 RP2040 端点/DPRAM 或描述符),届时可先临时把 current_usb_work_mode() 硬编成 HID 模式做隔离(1CDC+HID vs 3CDC)。


---

# 追加(主 agent 手动):Code 10 深度排查(进行中)

## 现象(硬件实测,逐次烧录确认)
- 1×CDC(config)+HID:**正常枚举**(1 CDC + 触摸屏)。→ 基座(tusb_config CDC=3、剥离 libpico、bcdDevice 0x0400)无问题。
- 2×CDC(config+serial,强制 serial 模式):**正常枚举**(2 个 COM 口)。
- 3×CDC(config+serial+light):**Code 10**,与端点编号无关:
  - 原始「独立编号」(端点号到 9):Code 10。
  - 改「OUT/IN 共号」参考约定(端点号 ≤6):**仍 Code 10**。

## 已排除
- 端点号上限:USB_NUM_ENDPOINTS=16、RP2040 TUP_DCD_ENDPOINT_MAX=16、CFG_TUD_INTERFACE_MAX=16、ep2drv[16]/itf2drv[16]。9 端点/6 接口全在界内。
- DPRAM:epx_data = 4096-0x180 = 3712B,9 端点×64 = 576B,远够。
- dcd_edpt_open 对端点数/号无限制(仅 DPRAM 溢出 hard_assert,未触发)。
- 双栈:map 已证 usbd/cdc_device/hid_device/msc_device/dcd_rp2040/tusb/tusb_fifo 全来自 Adafruit(CDC=3),libpico TinyUSB 未参与。
- Windows 缓存:bcdDevice 0x0100→0x0400 + 卸载幽灵设备,无效。
- 端点编号:已对齐 tinyusb 参考(todbot 已验证 RP2040 3-CDC 在 macOS 可用的 gist),仍失败。

## 与已知可用参考(todbot gist)的差异(待查)
- 参考 tusb_config:CFG_TUD_MSC=0、CFG_TUD_HID=0(纯 3×CDC)。**我们:MSC=1、HID=1 都编译进来了**(但 2×CDC 同样带 MSC=1/HID=1 却正常,故非充分条件)。
- 参考在 macOS 验证;我们在 Windows(目标平台)。原 issue#1169 报告者的失败是旧版 tinyusb(端点号≤8 限制),更新后解决——我们已是 Adafruit 3.7.1。
- 我们独有:剥离 libpico 的构建、psoc bringup、config CDC 每秒诊断输出。

## 下一步(未定/需 ground truth)
用 USB Device Tree Viewer 看当前 3-CDC 固件到底卡在哪一步(能否取全配置描述符 / 哪个接口报错 / 是复合父设备还是某功能 Code 10),或换 Linux `dmesg`/Mac 验证是否 Windows 特有。据此再决定,不再盲烧。
候选后续实验:MSC=0(现已剥离 libpico msc,或可不再链接报错)对齐参考;或把 config CDC 的诊断输出在枚举完成前关掉。


---

# 追加:UsbTreeView ground truth(关键)

3-CDC 固件在 Windows 上的实测(USB Device Tree Viewer):
- 设备描述符**完整读出**(VID 2E8A/PID 000A/bcdDevice 0400),usbccgp.sys 已挂到复合父设备。
- **Problem 10 (CM_PROB_FAILED_START) 在复合父设备本身**。
- **Current Config Value = 0x00**(未进入 Configured 态)、**Used Endpoints = 1**、**0 pipes to data endpoints**。

结论:Windows 读全描述符、识别为复合设备,但 **SET_CONFIGURATION(1) 未完成** → 设备始终停在未配置态。故失败点是**设备侧 SET_CONFIG(打开第 3 个 CDC 的端点)阶段**,非 Windows 驱动/缓存/字符串描述符问题。2 CDC 能完成 SET_CONFIG,3 CDC 不能 —— 在本 RP2040 + Adafruit TinyUSB 3.7.1 + Windows 组合下,纯 3×CDC 是一堵墙(所有静态上限都够、且对齐了已知可用参考,仍失败)。

## 建议的方向转变(待用户定夺)
让「恒定枚举的 config 通道」不再是第 3 个 CDC,改用 **WinUSB/vendor**(二进制协议更合适,配 MS OS 2.0 描述符免驱)或 **HID**(已证 config-CDC+HID 能枚举)。于是 serial 模式 = serial CDC + light CDC(2×CDC,游戏要的 COM 口不变)+ config 走 WinUSB/HID,彻底绕开 3×CDC 墙。
影响:固件 config 接口类型改变;Rust 上位机 config 传输从 serialport 改为 nusb(WinUSB)或 hidapi;serial/light 不变。
