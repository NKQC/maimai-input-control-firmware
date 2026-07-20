# v3.0 UI引擎可配置项全集 —— v4 Rust上位机功能对齐(Parity)基线

调研来源: `F:\maimaicontrol-V3.0\maimai-input-control-firmware`(v3.0 RP2040+摇杆+OLED菜单固件)
调研范围: `src/service/ui_manager/**`(菜单引擎)、`src/service/config_manager/**`(存储)、`src/service/input_manager/**`、`src/service/light_manager/**`、`src/protocol/touch_sensor/**`(AD7147/GTX312L/PSoC三种触摸IC)、`src/protocol/mai2serial/**`。纯只读调研,未修改任何代码。

---

## 1. UI菜单树

```
main (主界面, 只读)
 ├─ 触摸轮询回报率(Hz) / 键盘回报率(Hz) / I2C总线错误提示
 └─ >> main_menu (主菜单)
     ├─ touch_settings_main (触摸设置)
     │   ├─ touch_status              只读: 各设备connected状态 + 逐通道触摸位图('-'未启用/'0'/'1')
     │   ├─ [校准区,仅当存在支持校准的IC时显示]
     │   │    ├─ 校准灵敏度目标选择器  -10~+10, 默认+2
     │   │    ├─ "校准全部传感器"按钮
     │   │    └─ zone_sensitivity      (按分区A-E批量设置"校准目标"+发起按区校准)
     │   ├─ area_sensitivity           (按逻辑区域A1-E8直写灵敏度, 仅Serial模式)
     │   │   └─ area_sensitivity_zone  (某分区下已绑定区域列表)
     │   │       └─ area_sensitivity_detail (单区域灵敏度直写, 绝对0-99 / 相对-127~127)
     │   ├─ sensitivity_main           (按物理IC设备列表)
     │   │   ├─ sensitivity_device     (通用IC:逐通道灵敏度直写)
     │   │   └─ ad7147_custom_settings (AD7147专属:寄存器级参数页)
     ├─ binding_settings   (交互式绑区: 开始/终止/确认, 仅Serial模式, 依次触摸34个Mai2区域A1-E8)
     │   └─ binding_info   (只读: 34区域绑定通道HEX地址一览)
     ├─ communication_settings (串口/协议/发送策略设置)
     ├─ general_settings   (息屏超时 / 屏幕亮度)
     └─ [按钮] 保存设置 -> 落盘

内部模板页(非菜单树节点,是可复用UI组件):
 __error__       故障提示页
 __int_setting__ 通用整数设置弹窗(上下调值+确认/取消), 被各处ADD_INT_SETTING复用
```

导航模型: 摇杆3键(下/上/确认)，支持连续导航加速(1s内150ms防抖 → 1-3s 200ms(5Hz) → 3s+ 100ms(10Hz))；行类型 `LineType`: MENU_JUMP/INT_SETTING/BUTTON_ITEM/BACK_ITEM/SELECTOR_ITEM/TEXT；弹窗式返回栈；支持左右分屏。

---

## 2. 全部可配置项清单

### 2.1 通用设置 (general_settings)
| 字段 | 类型/范围 | 默认值 | 存储Key | 立即生效/落盘时机 |
|---|---|---|---|---|
| 息屏超时 | int 30-3600 秒 | 30(页面初值),配置默认300或60000ms(代码不一致,见附注) | `UIMANAGER_SCREEN_TIMEOUT` (uint16,ms) | 改值即生效，随全局"保存设置"按钮落盘 |
| 屏幕亮度 | int 0-255 | 128 | `UIMANAGER_BRIGHTNESS` (uint8) | 改值即生效+完成时立即落盘(唯一主动触发保存的设置项) |

未在UI暴露、但config key已注册的UIManager项(纯遗留/预留,无菜单入口):
- `UIMANAGER_REFRESH_RATE` uint16 默认50ms(渲染刷新周期)
- `UIMANAGER_ENABLE_BACKLIGHT` bool 默认true
- `UIMANAGER_BACKLIGHT_TIMEOUT` uint16 默认30000ms
- `UIMANAGER_ENABLE_JOYSTICK` bool 默认true
- `UIMANAGER_JOYSTICK_SENSITIVITY` uint8 默认128(**代码里TODO未生效**,adjust_int_setting_value的step恒为1，这是摇杆导航灵敏度，非触摸灵敏度)

### 2.2 通信设置 (communication_settings)
| 字段 | 类型/范围 | 默认值 | 说明 |
|---|---|---|---|
| Serial波特率 | 枚举(HAL预设表) | 115200 | Mai2Serial UART波特率 |
| Light波特率 | 枚举(同表) | 115200 | Mai2Light灯带UART波特率 |
| Serial采样延迟 | uint8 0-100 ms | 0 | 触摸响应延迟(聚合抗抖动窗口) |
| 映射键盘开关 | bool | false | 触摸区域同时映射为键盘按键 |
| 仅改变时发送(仅Serial模式) | bool | false | 状态不变则不发送串口帧 |
| 数据聚合延迟 | uint8 0~当前采样延迟 ms | 0 | 聚合窗口内多数投票 |
| 额外发送次数 | uint8 0-10 | 0 | 状态变化后追加发送,抗丢包 |
| 频率限制开关 | bool | false | 限制发送频率 |
| 频率限制值 | 枚举{10,15,20,24,30,48,50,60,75,90,120,144,165,180,220,260,300,360,400,460,500,600,1000}Hz | 120 | 按屏幕刷新率预设步进选择 |

**无UI入口**(仅API/编译期常量,菜单没有暴露):
- `work_mode` (InputWorkMode: SERIAL_MODE/HID_MODE) — 全局grep`setWorkMode`零调用,只能编译期/初始化硬编码
- `physical_keyboard_mappings` / `touch_keyboard_mappings` / `touch_keyboard_mode` — GPIO物理键盘映射与触摸转键盘映射，均只有API无UI
- `stage_assignments` (I2C采样阶段分配) — 硬件拓扑配置，非用户可调
- HID模式下的`hid_mappings[10]`(坐标映射)与`keyboard_mappings`(按键→通道) — 代码注释"TODO:TouchHID暂未实现UI"

### 2.3 绑区设置 (binding_settings + binding_info)
- 流程: IDLE → 开始绑区 → 依次触摸A1..E8共34个区域(WAIT_TOUCH/PROCESSING,progress=index/34*100) → 完成回IDLE
- 数据模型: `AreaChannelMappingConfig.serial_mappings[34]`，每项 `channel: uint32_t`(高8位设备掩码+低24位通道bitmap)，`0xFFFFFFFF`=未绑定
- 34个区域标准maimai拓扑: A1-8(外环) / B1-8(内环) / C1-2(中心) / D1-8(外环扩展) / E1-8(内环扩展)
- binding_info: 只读展示34区域绑定通道(HEX格式)
- 还存在`startAutoSerialBinding()`/`confirmAutoSerialBinding()`(引导式自动绑区API)，当前UI未调用，是半成品/预留功能

### 2.4 触摸灵敏度与校准 — 三层输入维度 + 两种底层物理表示
v3.0的灵敏度体系较复杂，按"用户操作入口"可分三层，但最终都落到同一份 `TouchDeviceMapping.sensitivity[24]`(每物理通道一个uint8值)：

**入口①: 按分区批量"校准目标" (zone_sensitivity, 仅Serial模式)**
- 前提: A-E五个分区必须全部完成绑区
- 每分区一个SimpleSelector: 目标灵敏度 **-10 ~ +10, 默认+2**（抽象裕度参数，不是最终值）
- 调整时调用`setCalibrationTargetByBitmap`(仅设置目标，不立即写灵敏度)
- 需额外点击"发起按区校准"按钮才真正执行硬件校准(`calibrateSelectedChannels()`)

**入口②: 按逻辑区域直写 (area_sensitivity → zone → detail, 仅Serial模式)**
- 逐个区域(A1-E8)直接编辑灵敏度**绝对值**
- 范围因设备灵敏度模式而异: 绝对模式 **0-99**；相对模式 **-127~127**
- ADD_INT_SETTING弹窗，完成时调用`setSerialAreaSensitivity(area, value)`直写(不经校准流程)

**入口③: 按物理设备+通道直写 (sensitivity_main → sensitivity_device)**
- 列出所有已注册触摸IC，逐通道编辑
- 同样区分绝对(0-99)/相对(-127~127)模式，取决于`TouchSensor::isSensitivityRelativeMode()`
- 完成时调用`setDeviceChannelSensitivity(device_mask, ch, value)`

**入口④: AD7147专属寄存器级页面 (ad7147_custom_settings)** — 仅当设备类型AD7147且已连接
- 通用灵敏度接口对AD7147被禁用(`supports_general_sensitivity=false`)，必须走此专属页
| 字段 | 范围 | 含义 |
|---|---|---|
| neg_afe_offset | 0-63 | 负AFE偏移(模拟前端失调补偿) |
| neg_afe_swap | 0/1 | 负AFE交换 |
| pos_afe_offset | 0-63 | 正AFE偏移 |
| pos_afe_swap | 0/1 | 正AFE交换 |
| neg_threshold_sensitivity | 0-15 | 负阈值灵敏度(原生寄存器) |
| neg_peak_detect | 0-5 | 负峰值检测 |
| pos_threshold_sensitivity | 0-15 | 正阈值灵敏度 |
| pos_peak_detect | 0-5 | 正峰值检测 |
- 附加: 当前stage选择器、实时CDC值只读显示(uint16原始电容测量)、通道触发状态只读
- 功能按钮: 应用配置(实时生效)/重置默认/从设备读取默认/**"一键拉偏移"自动校准**(进度0-100，由MCU侧C++状态机AEF扫描实现，非IC自带算法)

### 2.5 三种触摸IC的灵敏度/校准能力矩阵
| IC | 支持通用灵敏度 | 灵敏度模式 | 私有模式 | 支持校准 | UI落点 | 底层物理量 |
|---|---|---|---|---|---|---|
| AD7147 | 否 | - | 是 | **是** | ad7147_custom_settings(寄存器级) | 位域寄存器(见2.4④)，MCU软件跑AEF扫描算法 |
| GTX312L | 是 | 绝对(0-99) | 否 | 否 | sensitivity_device | IC私有6位灵敏度寄存器(0-0x3F)，UI裁剪到0-99 |
| PSoC(v3.0外部I2C从机,**非v4本机PSoC4**) | 是 | 相对(-127~127) | 否 | 否 | sensitivity_device / area_sensitivity_detail | 直接写电容阈值寄存器(`CAPx_THRESHOLD`)的相对偏移 |

> 重要澄清：v3.0的"PSoC"指**通过I2C挂载的外部PSoC从机触摸IC**(纯阈值寄存器读写，无自带算法)。v4新固件用的PSoC4 CapSense是**本地芯片跑Infineon官方CapSense库**(SmartSense自动调参)，两者架构完全不同，见第4节。

### 2.6 灯效 (light_manager, LIGHTMANAGER_* config key, 无独立UI菜单页——仅通过communication_settings间接调波特率，区域颜色/bitmap无本地菜单页)
| 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|
| enable | bool | true | 灯光管理器启用 |
| uart_device | string | "uart1" | UART设备名 |
| baud_rate | uint32 | 115200 | 有UI(communication_settings的"Light波特率") |
| node_id | uint8 | 1 | Mai2Light协议节点ID |
| neopixel_count | uint16 | 128 | Neopixel灯珠数量 |
| neopixel_pin | uint8 | 16 | Neopixel GPIO引脚 |
| region_bitmaps[11] / region_enabled[11] / region_colors[11][3] | bitmap16/bool/RGB | 全0/false/黑 | 11个灯光区域到neopixel位置的映射+颜色，**无本地UI菜单可调，只有API**(save/load_region_mappings) |

### 2.7 存储 / 系统类只读或半只读项
- `device_count`, 各设备`is_connected`/`enabled_channels_mask` — 运行时只读状态
- 触摸轮询回报率(Hz)、键盘回报率(Hz) — 主界面只读显示

---

## 3. 配置存储机制

- **存储介质**: RP2040 LittleFS(挂载于on-chip flash)，固定文件路径 `/config.bin`
- **数据格式**: 自研JSON文本格式(非二进制)，流式分块读写(`StreamingJsonParser`/`StreamingJsonSerializer`，256字节chunk)
- **内存结构**: `config_map_t = std::map<std::string, ConfigValue>`；`ConfigValue`是带类型tag的union(BOOL/INT8/UINT8/UINT16/UINT32/FLOAT/STRING) + 可选min/max范围
- **双层Map**: `_default_map`(只读默认值，由各服务`xxx_register_default_configs`注册) + `_runtime_map`(实际生效值，首次访问时惰性从default拷贝)
- **校验**: 保存时按key排序拼接JSON(不含`__crc32__`字段)→标准CRC32(zlib表)→连同`__crc32__`写入；读取时重新计算比对，不一致则读取失败并回退到默认配置重新保存
- **无版本号/迁移机制**：没有version字段，纯靠CRC判断"文件坏了就reset成默认"
- **保存时机**: 大部分设置项只是写入内存runtime_map + 置位`_save_requested`标志，真正落盘发生在`ConfigManager::save_config_task()`(由MainMenu"保存设置"按钮触发；亮度设置完成时也会立即触发一次)；落盘期间会临时`disable_interrupts`+`multicore_lockout`

**v4建议**: 存储介质/格式应对齐v4已有的config_manager实现(如已存在同构模块，直接复用；否则可用serde+更成熟的二进制/JSON方案取代v3.0这套手写流式解析器)。"按key排序+CRC32校验+default/runtime双层"的设计思路值得保留，但不必照抄实现细节。

---

## 4. 上位机/PC通信

**结论：v3.0没有任何PC上位机配置协议。**

- 全局grep `GET_CONFIG`/`SET_CONFIG`/`CMD_GET`/`CMD_SET`等关键字，在`src/`目录下零命中(仅第三方库文档误命中)
- `mai2serial.h`定义的协议是**面向街机主板**的标准maimai通信协议：固定6字节命令包`{L R c v}`，命令集仅4类 RSET(复位)/HALT(停止)/STAT(启动)/RATIO+SENS(比率/灵敏度，这是**街机主板发给控制器**的指令，非PC配置协议) + 独立的触摸数据帧`(...)`。通信对象是maimai游戏机主板，不是配置软件。
- 所有配置(触摸灵敏度/绑区/亮度/通信参数等)**只能通过设备本机OLED菜单+摇杆操作**完成。

**v4影响**: Rust上位机UI是v3.0完全不存在的全新能力，无历史协议可复用，需要自行设计一套双向配置协议(建议方向：USB HID自定义report 或 CDC虚拟串口 + 简单二进制/JSON帧，直接映射到v4侧config_manager的键值结构做双向同步)。这是v4相对v3.0的**能力增量**，不是"接替"。

---

## 5. 触摸现状总结(用于判断v4灵敏度结构如何设计)

v3.0是"MCU(RP2040) + I2C轮询外部触摸IC"架构，三种可选IC(AD7147/GTX312L/PSoC从机)各自灵敏度语义、校准能力、支持范围都不同，导致UI层需要为每种IC做类型分支+专属页面（尤其AD7147的寄存器级页面），复杂度较高。

v4是"PSoC4本机CapSense"架构（见`f:\mai2control\mai2control-v4\psoc_firmware`），触摸检测与算法完全跑在PSoC4芯片本地，使用Infineon官方CapSense库(SmartSense自调参/CSD)，主控(RP2040)只是通过SPI/SWD对PSoC4读写参数和状态，不再需要"多IC类型判断"这层复杂度。

**给v4的初步建议**（需要另行核对v4侧psoc_firmware的CapSense Tuner具体参数名，本次未深入调研v4侧代码）：
- v3.0的"0-99绝对灵敏度" / "-127~127相对灵敏度" / "-10~+10校准目标" 这套**多套抽象数值+多入口**的设计，在v4应收敛为**CapSense标准参数**（如Finger Threshold、Noise Threshold、Hysteresis、ON/OFF Debounce、IDAC等），由PC上位机直接读写这些参数的真实物理值，而不是维持v3.0式的"IC类型分支"UI结构。
- v3.0的"按分区批量设置+按区域直写+按设备通道直写"三层输入维度这一交互思路（宏观批量调 vs 微观单点调）值得在v4 UI中保留，但底层落到的应是统一的CapSense参数结构，不必区分设备类型。
- AD7147式的"寄存器级自定义页面"在v4若不再用AD7147，则不需要保留，但"暴露底层硬件真实参数供高级用户调整"的能力，可以用"CapSense Tuner高级模式"页面对齐。

---

## 6. Parity检查表（v4 UI需要覆盖的功能点，按优先级粗分）

| 功能域 | v3.0能力 | v4建议接替方式 |
|---|---|---|
| 触摸灵敏度调整 | 三层输入维度(分区批量/区域直写/设备直写)+IC相关的0-99或-127~127数值 | 收敛为PSoC4 CapSense标准参数，PC侧统一读写 |
| 触摸校准 | AD7147专属AEF自动扫描算法(MCU软件实现) | 用PSoC4 CapSense库自带的SmartSense自动调参/基线复位替代，UI只需暴露"触发校准"+进度 |
| 绑区(34区域) | 交互式触摸绑定+只读绑定信息展示 | 保留同等交互模型(PC引导用户依次触摸)+绑定结果展示 |
| 通信设置 | 波特率/延迟/发送策略/频率限制等9项 | 若v4仍走串口，等价保留；若改用USB HID为主，部分项(波特率)可能不再适用，需按v4实际通信方案取舍 |
| 通用设置 | 息屏超时/亮度 | 若v4仍有本机OLED屏则保留；若v4完全由PC UI管理则可能不再需要本机菜单，但仍可作为PC UI的一项配置 |
| 灯效配置 | 区域-灯珠映射+颜色(仅API无本机UI) | v4 PC UI可以补齐这块v3.0本机UI都没做到的能力(可视化映射编辑器) |
| PC配置协议 | 无 | v4全新设计，建议复用v4侧config_manager键值结构 |
| 工作模式切换(Serial/HID) | 无运行时UI，编译期固定 | v4若要支持运行时切换，是新增能力 |

---

## 附注：调研留痕

- 落盘的调研过程详情(分阶段结论、待办跟踪)见 `f:\mai2control\mai2control-v4\.kiro\subagent\context\recon_v3_ui.md`
- 本次调研纯只读，未修改v3.0或v4任何代码文件
- 未深入调研的部分（如需要请另开任务）：v4侧`psoc_firmware`的CapSense Tuner具体参数结构、v4侧现有config_manager是否已与v3.0同构、v4侧现有Rust UI代码现状
