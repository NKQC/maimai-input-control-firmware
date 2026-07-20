# recon_v3_ui 任务落盘

## 任务目标
调研 v3.0 固件 (F:\maimaicontrol-V3.0\maimai-input-control-firmware) 的 UI 菜单引擎暴露的全部可配置项,
产出 v4 Rust 上位机 UI 功能对齐(parity)基线文档。纯只读,不改代码。

## 整体架构结论(已确认)
- 主固件目录: `src/` 下分 hal / protocol / service 三层。
- **UI 菜单系统**位于 `src/service/ui_manager/`:
  - `ui_manager.cpp/h`, `page_registry.cpp/h` — 顶层管理与页面注册表
  - `engine/page_construction/` — page_constructor, page_manager, page_template, page_macros (菜单引擎核心)
  - `engine/template_page/` — error_page, int_setting_page (通用页面模板,如整数设置页)
  - `engine/graphics_rendering/`, `engine/fonts/` — 渲染与字体(未细读,非配置项相关,低优先级)
  - `page/` 下实际菜单页:
    - `main_menu.cpp/h`, `main_page.cpp/h` — 主菜单/主页
    - `general_settings/` — general_settings.cpp/h (通用设置)
    - `communication_settings/` — communication_settings.cpp/h (通信设置,推测含 mai2serial/HID 切换)
    - `binding_settings/` — binding_settings.cpp/h + binding_info.cpp/h (按键/触控区域绑定)
    - `touch_settings/` — 触控设置最大分支:
      - `touch_settings_main.cpp/h`, `device_type_utils.h`
      - `sensitivity/` — sensitivity_main, area_sensitivity(+detail+zone), sensitivity_device, zone_sensitivity (灵敏度配置,分区/分设备)
      - `device_custom_settings/` — ad7147_custom_settings.cpp/h (仅 AD7147 有自定义参数页)
      - `status/` — touch_status.cpp/h (状态显示,只读)
- **配置存储**: `src/service/config_manager/` — config_manager.cpp/h, config_types.h, config_crc.cpp/h (结构体定义+CRC,存储介质待读确认 EEPROM/flash)
- **触控硬件抽象**: `src/protocol/touch_sensor/` — touch_sensor.cpp/h 统一接口,子目录:
  - `ad7147/` (ad7147.cpp/h + ad7147_ctools.cpp) — AD7147 CapSense IC
  - `gtx312l/` (gtx312l.cpp/h) — GTX312L IC
  - `psoc/` (psoc.cpp/h) — PSoC 触控(与 v4 用的 PSoC4 CapSense 类似路线)
  => v3.0 支持**多种触控芯片**,故 sensitivity 菜单需按 device_type 分支(sensitivity_device.cpp 印证)。
- **输入/灯效服务**: `src/service/input_manager/`(input_manager.cpp/h), `src/service/light_manager/`(light_manager.cpp/h)
- **显示屏驱动**: `src/protocol/st7735s/` (st7735s.cpp/h) — SPI OLED/LCD
- **其他协议**: `src/protocol/hid/`, `mai2light/`, `mai2serial/`, `mcp23s17/`(IO扩展), `neopixel/`, `usb_serial_logs/`
- 另有独立子工程 `SensorFirmware/TouchSensor/` (PSoC4 CapSense 传感器侧固件, module/capsense, module/led, module/trigger, module/i2c) — 这是**传感器侧**固件,与 main 固件通过 i2c/协议通信,不属于主 UI 菜单但影响"触控现状"结论。

## 已完成步骤
1. list_directory 摸清整体目录结构(main src 三层 + SensorFirmware 子工程)
2. 定位 ui_manager 完整页面树与 engine 结构
3. 定位 config_manager, touch_sensor(多芯片), input_manager, light_manager, st7735s

## 待办步骤(下一步动作)
1. 批量读取核心文件内容(逐项产出字段/类型/范围/默认值):
   - config_types.h, config_manager.h/cpp, config_crc.h (存储结构+CRC+介质)
   - ui_manager.h/cpp, page_registry.h/cpp (菜单树/进入切换逻辑)
   - main_menu.h/cpp, main_page.h/cpp
   - general_settings.h/cpp
   - communication_settings.h/cpp (协议切换 mai2serial/HID?)
   - binding_settings.h/cpp + binding_info.h/cpp (按键映射/绑区)
   - touch_settings_main.h/cpp, device_type_utils.h
   - sensitivity_main, area_sensitivity(+detail+zone), sensitivity_device, zone_sensitivity (灵敏度全套)
   - ad7147_custom_settings.h/cpp (自定义参数)
   - touch_status.h/cpp (只读状态)
   - touch_sensor.h/cpp + ad7147.h + gtx312l.h + psoc.h (触控现状:自带算法 vs 外部芯 IC,灵敏度物理量)
   - input_manager.h, light_manager.h (按键映射/灯效模式与颜色)
   - int_setting_page.h/cpp (通用整数设置页模板,决定"范围/步长"UI 元语义)
2. 检查 PC 端上位机通信:搜索 mai2serial/hid 协议里是否有"配置读写命令"(与本地菜单对应的远程配置协议),若有需要摘录命令集。
3. 确认配置存储介质(EEPROM/flash?调用哪个 hal?)
4. 汇总产出最终 Markdown 报告(菜单树+全部可配置项清单+存储+通信+触控现状+v4接替建议)

## 关键决策
- 判断触控灵敏度是否为"仅一个全局值"还是"分区/分设备"结构 —— 已知有 area_sensitivity_zone.cpp 和 sensitivity_device.cpp,说明**很可能是分区+分设备**结构,需读源码确认字段。
- v3.0 支持 3 种触控芯片(AD7147/GTX312L/PSoC),而 v4 用 PSoC4 CapSense(见 workspace 的 psoc_firmware),需要在"触控现状"结论里说明 v4 已收敛到单一 PSoC4 方案,菜单里的"多设备切换"项在 v4 可能不再需要,但灵敏度调节语义要对齐。

## 下一步动作
立即批量读取"待办步骤 1"中列出的全部文件,产出结构化字段清单。

## 阶段性详细结果(第一轮批量读取后)

### A. 配置存储机制(已确认)
- 存储介质: **RP2040 LittleFS**(文件系统挂载于flash),配置文件路径固定 `/config.bin`,内容是**自定义JSON文本格式**(非二进制),流式读写(StreamingJsonParser/Serializer,256字节chunk)。
- 数据结构: `config_map_t = std::map<std::string, ConfigValue>`,ConfigValue是**带类型tag的union**(BOOL/INT8/UINT8/UINT16/UINT32/FLOAT/STRING),可选min/max范围(has_range)。
- 双map架构: `_default_map`(只读默认值,由各服务`xxx_register_default_configs`注册) + `_runtime_map`(实际生效值,首次访问时从default惰性拷贝)。
- 校验: 保存时按key排序拼JSON(不含crc字段)→算CRC32(标准zlib表)→连同`__crc32__`字段一起写入;读取时重新计算比对,不一致则读取失败(返回false,上层会格式化并重存默认配置)。
- 保存时机: 各页面/服务不是"改值即存盘",而是 set 到 runtime_map(内存)+ 调用`ConfigManager::save_config()`置位`_save_requested`标志,真正落盘发生在`save_config_task()`(在MainMenu"保存设置"按钮/亮度设置完成回调等处触发),落盘会临时disable_interrupts+multicore_lockout。
- 无版本号/迁移机制字段(没看到version字段),纯靠CRC判断"文件坏了就reset成默认"。
- v4接替建议: v4是Rust+PSoC4方案,存储介质/格式应参考现有v4 config_manager(main_firmware/src/service/config_manager,已存在康康是否同构),配置项需要一个可从PC上位机读写的通道(见下方通信部分),不需要照抄v3.0的自制JSON流式解析器,可用serde等成熟库,但**"按key排序+CRC32校验+default/runtime双层"这套设计思路值得保留**。

### B. UI菜单树(已确认,树状)
```
main (主界面,只读状态显示: 触摸轮询Hz / 键盘回报Hz / I2C总线错误提示)
 └─ main_menu (主菜单,>>进入)
     ├─ touch_settings_main (触摸设置入口)
     │   ├─ touch_status        (触摸状态,只读,显示各设备触摸位状态)
     │   ├─ zone_sensitivity     (按"分区"设置灵敏度 - Mai2逻辑区域A1..E8级别)
     │   ├─ sensitivity_main     (灵敏度设置主入口,可能是入口分发页,待细读)
     │   ├─ sensitivity_device   (按"设备"设置灵敏度 - 物理IC/通道级别)
     │   ├─ area_sensitivity     (区域灵敏度,与zone_sensitivity可能是同类不同粒度,待细读)
     │   ├─ area_sensitivity_zone   (细分:某区域的zone级)
     │   ├─ area_sensitivity_detail (细分:某区域/设备的detail级,单通道数值)
     │   └─ ad7147_custom_settings  (仅AD7147芯片的自定义寄存器级参数页)
     ├─ binding_settings   (交互式绑区: 开始/终止/确认保存,仅Serial模式可用,34个Mai2区域A1-E8依次触摸绑定)
     │   └─ binding_info   (只读:显示34个区域的绑定通道ID,0xFFFFFFFF=未绑定)
     ├─ communication_settings (通信设置,见下方C)
     └─ general_settings   (通用设置: 息屏超时[30-3600s]/屏幕亮度[0-255])
     [MainMenu还有: "保存设置"按钮 -> 触发ConfigManager::save_config()落盘]
内部模板页(非菜单树,是复用组件): __error__ / __int_setting__ (通用整数设置弹窗,上下调整数值+确认/取消)
```
导航引擎要点: 摇杆3键(A=DOWN,B=UP,CONFIRM),支持连续导航加速(1s内150ms防抖,1-3s→200ms/5Hz,3s+→100ms/10Hz),行类型枚举LineType: MENU_JUMP/INT_SETTING/BUTTON_ITEM/BACK_ITEM/SELECTOR_ITEM/TEXT,弹窗式返回栈(PageNavigationManager),支持分屏(左右两栏),不可滚动模式只在可交互行间跳转/可滚动模式Z字型走全部行。

### C. 通信设置页(communication_settings) 可配置项清单
| 字段 | 类型/范围 | 默认值 | 存储位置 | 说明 |
|---|---|---|---|---|
| Serial波特率 | uint32_t,枚举表(HAL get_supported_baud_rates) | 115200 | InputManager.mai2serial_config.baud_rate | Mai2Serial UART波特率 |
| Light波特率 | uint32_t,同枚举表 | 115200 | LightManager_PrivateConfig.baud_rate | Mai2Light UART波特率(灯带协议) |
| Serial采样延迟 | uint8_t 0-100(ms) | 0 | InputManager.touch_response_delay_ms | 触摸响应延迟(用于聚合抗抖动) |
| 映射键盘开关 | bool | false | InputManager.touch_keyboard_enabled | 触摸区域是否同时映射为键盘按键 |
| 仅改变时发送 | bool(仅Serial模式显示) | false | InputManager.send_only_on_change | 状态不变则不发送串口帧,省带宽 |
| 数据聚合延迟 | uint8_t 0-current_serial_delay_(ms) | 0 | InputManager.data_aggregation_delay_ms | 聚合窗口内做多数投票(VotingAggregationState) |
| 额外发送次数 | uint8_t 0-10 | 0 | InputManager.extra_send_count | 状态变化后追加发送次数(抗丢包) |
| 频率限制开关 | bool | false | InputManager.rate_limit_enabled | 限制发送频率 |
| 频率限制值 | uint16_t,预置枚举{10,15,20,24,30,48,50,60,75,90,120,144,165,180,220,260,300,360,400,460,500,600,1000}Hz | 120 | InputManager.rate_limit_frequency | 按屏幕刷新率预设步进选择 |
| 工作模式 | enum InputWorkMode{SERIAL_MODE=0, HID_MODE=1} | SERIAL_MODE | InputManager.work_mode | mai2serial协议 vs HID模式切换,**本页没有直接UI项切换它**,由binding_settings/communication_settings里用isSerialMode()读取,需要找哪里设置切换(可能在general_settings或其它地方,待查) |

### D. 通用设置页(general_settings)
| 字段 | 类型/范围 | 默认 | 存储key | 说明 |
|---|---|---|---|---|
| 息屏超时 | int32_t 30-3600(秒) | 30 | UIMANAGER_SCREEN_TIMEOUT(uint16,存的是毫秒=秒*1000) | INT_SETTING弹窗调整,即改即生效(不立即存盘,靠MainMenu保存按钮) |
| 屏幕亮度 | int32_t 0-255 | 128 | UIMANAGER_BRIGHTNESS(uint8) | 有进度条实时预览,ADD_INT_SETTING;on_brightness_complete时立即set+save_config()落盘(唯一一个页面内主动触发保存的) |

### E. UIManager顶层配置(config key前缀UIMANAGER_*)
| Key | 类型 | 默认 | 说明 |
|---|---|---|---|
| UIMANAGER_REFRESH_RATE | uint16 | 50(ms) | 刷新率,无UI暴露入口(代码里没看到设置页) |
| UIMANAGER_BRIGHTNESS | uint8 | 128 | 见D |
| UIMANAGER_ENABLE_BACKLIGHT | bool | true | 无UI暴露入口 |
| UIMANAGER_BACKLIGHT_TIMEOUT | uint16 | 30000(ms) | 无UI暴露入口(有UIManager_PrivateConfig字段但general_settings只暴露了screen_timeout,没暴露backlight_timeout,可能是遗留/未完成功能) |
| UIMANAGER_SCREEN_TIMEOUT | uint16 | 60000(ms)注:代码里两处默认值不一致(UIManager_PrivateConfig里是300,uimanager_register_default_configs里是60000ms=60s,GeneralSettings构造时初值30) | 见D |
| UIMANAGER_ENABLE_JOYSTICK | bool | true | 无UI暴露入口 |
| UIMANAGER_JOYSTICK_SENSITIVITY | uint8 | 128(注册默认)but PrivateConfig默认给5,adjust_int_setting_value里TODO说"根据灵敏度算步长"但目前**step恒为1,灵敏度配置实际未生效**(代码注释TODO) | UI摇杆导航灵敏度,非触摸灵敏度!与触摸IC灵敏度是两个概念,注意区分 |

### F. 绑区(binding_settings + binding_info)
- 交互式绑定流程: IDLE→(按"开始绑区")→WAIT_TOUCH/PROCESSING(按顺序提示触摸A1..E8共34个区域,每触摸一个自动进入下一个,progress=index/34*100)→完成后回IDLE。
- 仅Serial模式可用(HID模式下此页整页替换为提示文字)。
- 绑定结果: `AreaChannelMappingConfig.serial_mappings[34]`,每项`channel: uint32_t`(高8位设备掩码+低24位通道bitmap),0xFFFFFFFF=未绑定。34个区域名: A1-8,B1-8,C1-2,D1-8,E1-8(标准maimai触摸区域拓扑)。
- binding_info页只读展示34项区域+HEX通道地址。
- 还有 `startAutoSerialBinding()`(引导式自动绑区)/`confirmAutoSerialBinding()`接口存在于InputManager,但当前UI(binding_settings.cpp)未见调用入口,可能是预留/半成品功能。
- 另外AreaChannelMappingConfig还有 hid_mappings[10](HID模式坐标映射,TouchAxis x/y)和keyboard_mappings(按键->通道 map<HID_KeyCode,channel>) —— **这两个在UI里完全没有暴露菜单页**(代码注释"TODO: TouchHID暂未实现UI"),只有API,无菜单可调。v4需注意是否要补齐UI。

### G. InputManager 核心配置全集(InputManager_PrivateConfig,部分在UI里没有入口,需标注"有UI"/"无UI仅API")
- work_mode: InputWorkMode枚举 — 【无UI直接切换菜单项,目前找到的地方只有读取,需继续查是否在其它页,如general_settings.cpp里没看到,可能在main.cpp初始化时硬编码或有隐藏页面待查】
- touch_device_mappings[16]: TouchDeviceMapping{device_id_mask, max_channels, sensitivity[24]每通道灵敏度uint8, enabled_channels_mask, is_connected} — 有UI(sensitivity_device/zone_sensitivity/area_sensitivity系列,细节待读)
- device_count: uint8 — 只读运行时统计
- area_channel_mappings: 见F
- physical_keyboard_mappings: vector<PhysicalKeyboardMapping>{gpio, default_key, trigger_level(ACTIVE_HIGH/LOW/AUTO)} — 【全局未见UI菜单页,只有API addPhysicalKeyboard,可能是编译期/main.cpp里硬编码,不是运行时UI可调项】
- touch_keyboard_mappings: vector<TouchKeyboardMapping>{area_mask 64bit, hold_time_ms, key, trigger_once} — 【同上未见UI菜单,仅API】
- touch_keyboard_mode: enum{KEY_ONLY,TOUCH_ONLY,BOTH} 默认BOTH — 【无UI】
- touch_keyboard_enabled: bool — 有UI(communication_settings"映射键盘"开关)
- touch_response_delay_ms: uint8 0-100 — 有UI(C表)
- send_only_on_change/data_aggregation_delay_ms/extra_send_count/rate_limit_enabled/rate_limit_frequency — 有UI(C表)
- stage_assignments: vector<StageAssignment>{i2c_bus,stage,device_id} — 【无UI,I2C总线采样阶段分配,属于硬件拓扑配置,大概率是编译期/自动探测,非用户可调项】
- mai2serial_config: Mai2Serial_Config{baud_rate,...其它字段待查mai2serial.h} — 有UI(baud_rate部分)

### H. 触摸灵敏度/校准 API(InputManager,UI细节待读sensitivity_*.cpp后补充)
- setSensitivity(device_id_mask, channel, int8_t sensitivity) / getSensitivity — 按物理设备+通道设置,uint8_t sensitivity范围未知(int8_t参数但注释#define DEFAULT_TOUCH_SENSITIVITY 45,#号注释"0-99范围") → **灵敏度是0-99的整数,每"物理通道"独立**(TouchDeviceMapping.sensitivity[24]数组,每设备最多24通道)
- setSerialAreaSensitivity(Mai2_TouchArea area, sensitivity) — 按逻辑区域(A1-E8)设置,内部会查area_channel_mappings.serial_mappings反查物理通道再设置物理灵敏度 → 说明"分区灵敏度"和"设备灵敏度"最终都落到同一份物理通道sensitivity数组,只是UI输入维度不同(逻辑区域 vs 物理设备通道)
- setHIDAreaSensitivity / setKeyboardSensitivity — 同理但对应HID/键盘映射维度,【UI未暴露,同F节遗留】
- 校准: calibrateAllSensors() / calibrateAllSensorsWithTarget(target) / calibrateSelectedChannels() / setCalibrationTargetByBitmap(bitmap,target) / getCalibrationProgress() — 存在"自动校准"整套API,UI入口位置待确认(可能在sensitivity_main.cpp,待读)

## 待办(下一步,未完成)
1. 读取 sensitivity_main.cpp/h, sensitivity_device.cpp/h, zone_sensitivity.cpp/h, area_sensitivity.cpp/h, area_sensitivity_zone.cpp/h, area_sensitivity_detail.cpp/h, touch_status.cpp/h, ad7147_custom_settings.cpp/h, touch_settings_main.cpp/h, device_type_utils.h —— 补全"灵敏度/校准/设备状态"UI维度的具体字段与范围,并确认校准功能UI入口
2. 读取 touch_sensor.h/cpp, ad7147.h, gtx312l.h, psoc.h(v3.0侧) —— 确认v3.0触控是"自带滑动窗口/阈值算法"还是"外部IC自带CapSense算法+仅读寄存器",从而回答"灵敏度到底调的是什么物理量"
3. 读取 mai2serial.h(协议帧/Mai2Serial_Config其它字段), hid.h(简单看), 判断有无"PC上位机通过USB/Serial读写配置"的协议命令(搜索关键字如"GET_CONFIG"/"SET_CONFIG"/JSON over serial等),若没有则在报告里明确"v3.0无PC上位机配置协议,纯本地菜单操作"
4. 查WorkMode(SERIAL_MODE/HID_MODE)切换的UI入口在哪(继续grep或读取剩余未读的.cpp);若确认无UI入口则记录为"编译期/初始化硬编码,无运行时切换UI"
5. int_setting_page.cpp/h, error_page.cpp/h, page_template.h/cpp, page_constructor.h, page_macros.h —— 引擎细节,用于理解"INT_SETTING/SELECTOR_ITEM"等交互语义,非必须但有助于精确描述UI交互模型给v4参考
6. 汇总产出最终Markdown报告

## 关键决策(更新)
- 已证实：v3.0"灵敏度"最终都是**每物理通道一个0-99整数**,存在TouchDeviceMapping.sensitivity[24]数组,UI提供"按逻辑区域(A1-E8等)"和"按物理设备通道"两种编辑维度,但底层数据模型是单一的每通道数值+反查映射表。v4若已用PSoC4自带CapSense(SmartSense/CapSense库),很可能改为"IDAC/finger threshold"等物理量,需要在报告里给出"v3.0的0-99抽象灵敏度 -> v4 PSoC4 CapSense实际阈值参数"的映射建议(需要读PSoC4 CapSense配置确认具体参数名,这部分依赖v4侧psoc_firmware信息,可在报告结尾建议主agent另开一个任务核对)。
- 尚未发现PC上位机协议痕迹,倾向于v3.0是"纯本地菜单化配置,无远程配置协议",但还没有确认读mai2serial.h,需要读完再下结论。

## 第二轮补充(灵敏度/校准/自定义IC参数 已读完)

### I. 触摸设置主页(touch_settings_main) 
- "查看触摸状态" -> touch_status(只读,每设备一行:设备名+bitmap字符串,'-'=通道未启用,'0'/'1'=未触摸/触摸)
- 校准区(仅当`hasCalibratableSensors()`为true时显示):
  - 校准灵敏度目标选择器: SensitivityOption枚举 **-10到+10整数,默认+2**,SimpleSelector左右调整 (注意:此为"校准目标灵敏度",不是最终灵敏度值,是校准算法的目标裕度参数)
  - "校准全部传感器"按钮 -> calibrateAllSensorsWithTarget(target)
  - "按分区校准灵敏度"菜单 -> zone_sensitivity
  - 校准中会显示进度条(0-255,255=完成)
- "按区域调整灵敏度"菜单(仅Serial模式) -> area_sensitivity
- "按模块调整灵敏度"菜单 -> sensitivity_main

### J. sensitivity_main (灵敏度调整总入口)
- 列出InputManager所有已注册触摸IC设备,每设备一个菜单项;若设备类型是AD7147且已连接则跳ad7147_custom_settings,否则跳sensitivity_device(通用IC灵敏度页)。设备连接状态用颜色区分(白=连接,红=未连接)。

### K. zone_sensitivity (按分区A-E批量设置灵敏度,仅Serial模式)
- 前提: 5个分区(A/B/C/D/E)必须**全部完成绑区**才能操作,否则提示"绑区不完整"。
- 每分区一个SimpleSelector: "X区目标灵敏度: ±N" 范围**-10到+10,默认+2**(与touch_settings_main的SensitivityOption同一套枚举/常量)
- 调整即时生效: 对该分区所有已绑定的物理通道调用`setCalibrationTargetByBitmap(bitmap, target)`——这是"设置校准目标"而不是直接写灵敏度值!需要后续走"发起按区校准"按钮(calibrateSelectedChannels())才真正执行校准写入。
- 二段式操作: ①各分区选定目标灵敏度(-10~+10) ②点击"发起按区校准"触发真正的自动校准流程。

### L. area_sensitivity 体系(按逻辑区域直接设置数值,与K的"校准目标"概念不同,是直接写灵敏度绝对值)
- area_sensitivity(顶层): 列出A-E五个zone(仅显示有绑定的),每zone显示"绑定数量"
- area_sensitivity_zone(区域组详情): 显示该zone下所有已绑定区域(如A1,A3,A5...),按区域索引排序,点击进入detail
- area_sensitivity_detail(单区域详情): 
  - 若设备为**相对模式**: 灵敏度范围 **-127 到 127**
  - 若设备为**绝对模式**: 灵敏度范围 **0 到 99**
  - 用ADD_INT_SETTING弹窗直接编辑数值,完成时调用`setSerialAreaSensitivity(area, int8_t value)`(直写,不经过校准流程)
  - 若区域未绑定/设备不支持灵敏度/设备为相对模式(此处代码逻辑有件怪异:检测支持性时调用`supportsGeneralSensitivity()`,与"相对模式"是两个不同判断维度)则显示"不支持"

### M. sensitivity_device (按物理设备+通道直接设置,与L同属直写类,细粒度到硬件通道号)
- 通过`jump_str`接收设备名称,展示该设备"每个启用的通道"一行 INT_SETTING
- 同样区分**相对模式(-127~127)** vs **绝对模式(0~99)**,取决于`TouchSensor::isSensitivityRelativeMode()`
- 完成时对每个通道调用`setDeviceChannelSensitivity(device_mask, ch, value)`

**关键结论：v3.0灵敏度体系是"三层输入维度 + 两种物理表示"**：
- 三层输入维度:分区批量校准目标(K,-10~+10抽象裕度) / 逻辑区域直写(L,0~99或-127~127) / 物理设备通道直写(M,0~99或-127~127)
- 最终落到同一份底层数据:`TouchDeviceMapping.sensitivity[24]`(每物理通道一个uint8_t,含义因设备的"绝对/相对模式"而不同)
- "绝对模式"的0-99应为**触摸阈值等效值**(数值越大越不灵敏或越灵敏,需看具体IC实现,推测数值代表"检测阈值"或"信号裕量百分比");"相对模式"的-127~127应为**相对默认基准的偏移量**(增减调整,可能是GTX312L等IC的"寄存器相对偏移"式灵敏度)

### N. AD7147自定义设置页(ad7147_custom_settings,仅AD7147且已连接时可进) — 这是v3.0里最底层/最硬核的参数页
BitfieldHelper 位域(每个stage/通道独立配置,来自AD7147寄存器PortConfig):
| 字段 | 范围 | 含义 |
|---|---|---|
| neg_afe_offset | 0-63 | 负AFE偏移(模拟前端失调补偿) |
| neg_afe_swap | 0-1(bool) | 负AFE交换 |
| pos_afe_offset | 0-63 | 正AFE偏移 |
| pos_afe_swap | 0-1(bool) | 正AFE交换 |
| neg_threshold_sensitivity | 0-15 | 负阈值灵敏度(AD7147原生寄存器字段,数值越小越灵敏,典型CapSense IC设计) |
| neg_peak_detect | 0-5 | 负峰值检测 |
| pos_threshold_sensitivity | 0-15 | 正阈值灵敏度 |
| pos_peak_detect | 0-5 | 正峰值检测 |
- 还有:当前选中stage(int32_t,可切换)、实时CDC值只读显示(uint16_t,当前通道原始电容测量值)、通道触发状态只读
- 功能按钮: 应用配置(实时生效写入硬件)/重置为默认值/从设备读取默认配置/**"一键拉偏移"自动校准**(auto_offset_active_ + 进度0-100)
- 这是**v3.0里唯一真正"暴露IC底层寄存器参数"的页面**,其它页面都是抽象的0-99/-127~127/±10。v4若用PSoC4 CapSense,这一层应对应PSoC4的CapSense Tuner参数(如Finger Threshold/Noise Threshold/Hysteresis/IDAC等),细节需要另外核对v4 psoc_firmware配置。

## 待办(剩余,收尾阶段)
1. 读取 touch_sensor.h/cpp(统一接口) + gtx312l.h + psoc.h(v3.0侧,轻量确认即可,ad7147已通过custom_settings页间接确认) —— 确认"绝对/相对模式"分别对应哪些IC,及底层算法归属(IC自带CapSense算法 vs MCU代码里跑算法)
2. 读取 mai2serial.h 关键定义(协议帧结构+Mai2Serial_Config其它字段)+ grep 是否存在"配置读写协议命令"(判断PC上位机通信协议是否存在)
3. grep WorkMode切换UI入口(搜索`setWorkMode`调用点)
4. 产出最终Markdown报告,写入正式位置(询问/默认放哪个路径?—— 按主agent要求"最终产出一份Markdown",直接作为subagent_response正文或写文件都可,决定写入 f:\mai2control\mai2control-v4\.kiro\context\v3_ui_parity_baseline.md 作为交付物,同时更新本落盘文件状态为"待汇报"）

## 第三轮补充(触控硬件归属 + 协议确认,调研完毕)

### O. 三种触摸IC的灵敏度/校准能力矩阵(TouchSensor::sensor_flag_,在各IC构造函数里设置)
| IC | supports_general_sensitivity | sensitivity_relative_mode | sensitivity_private_mode | supports_calibration | UI落点 |
|---|---|---|---|---|---|
| AD7147 | false | false | **true** | **true** | 走`ad7147_custom_settings`(底层寄存器页,N节),不走通用sensitivity_device/0-99滑条;支持"一键拉偏移"自动校准 |
| GTX312L | true | false(绝对模式) | false | false | 走`sensitivity_device`,范围**0-99**,无校准功能,setChannelSensitivity直接写`GTX312L_REG_SENSITIVITY_x`寄存器(6位有效,0-0x3F,但UI层限制在0-99再クランプ) |
| PSoC(v3.0的I2C从机方案,与v4本机PSoC4 CapSense**不是同一套**!) | true | **true(相对模式)** | false | false | 走`sensitivity_device`或`area_sensitivity_detail`,范围**-127~127**,setChannelSensitivity把-127..127映射为**阈值(threshold)寄存器**的相对写入(`PSOC_REG_CAPx_THRESHOLD`),不支持校准 |

结论:"灵敏度调的物理量"因IC而异——GTX312L是"厂商私有6位灵敏度寄存器值(0-63,UI裁到0-99)";PSoC(v3.0从机)是"直接写电容阈值寄存器的相对偏移量(-127~127)";AD7147最特殊,通用灵敏度接口被禁用(supports_general_sensitivity=false),真正生效的是`ad7147_custom_settings`页里的**位域寄存器**(neg/pos_threshold_sensitivity 0-15 + neg/pos_afe_offset 0-63等),配合独立的"校准算法"(阶段性AEF偏移扫描,由MCU侧C++代码实现，不是IC自带算法，即CalibrationTools状态机是MCU软件在跑，不是IC内置)。

**重要澄清给v4**: v3.0这里的"PSoC"是指**通过I2C连接的外部PSoC从机触摸IC**(寄存器映射见I2C_Registers_README.md,阈值式电容检测,MCU侧只读寄存器+写阈值,不做算法),而v4新固件用的PSoC4 CapSense(look at `f:\mai2control\mai2control-v4\psoc_firmware`)是**Infineon官方CapSense库跑在PSoC4芯片本地**,采用SmartSense/CSD自动调参算法，架构完全不同：
- v3.0:MCU(RP2040)通过I2C轮询IC寄存器,灵敏度/阈值/校准逻辑要么在MCU侧软件实现(AD7147的AEF扫描),要么直接写IC寄存器(GTX312L/PSoC从机)
- v4:PSoC4本地跑CapSense库自带的电容检测+SmartSense算法,阈值/灵敏度参数是CapSense Tuner的标准字段(如Finger Threshold, Noise Threshold, Hysteresis, ON/OFF debounce, IDAC等),暴露方式应该是"读写PSoC4寄存器/参数结构体",而不是v3.0这种"IC类型分支+自定义UI页"模式。v4理论上**不再需要"多IC类型判断+三种UI分支"这套复杂度**,可以用统一的CapSense参数结构。

### P. PC上位机通信协议 — 结论:**v3.0没有PC侦上位机配置协议**
- grep全局搜索GET_CONFIG/SET_CONFIG/CMD_GET/CMD_SET等关键字,仅在mtb_shared第三方库文档里命中(无关),src目录下**零匹配**。
- Mai2Serial协议(`mai2serial.h`)是**面向街机主板**的标准maimai协议:固定6字节命令包`{L R c v}`,命令集仅4类: RSET(复位)/HALT(停止)/STAT(启动状态)/RATIO+SENS(比率/灵敏度,这是**街机主板发给控制器的**灵敏度指令,不是PC上位机配置协议,是maimai游戏机与触摸控制器之间的标准通信)+触摸数据帧`(...)`。这套协议的对象是**街机主板**,不是配置PC软件。
- 结论:v3.0的所有配置(触摸灵敏度/绑区/亮度/通信参数等)**只能通过设备本地OLED菜单+摇杆操作**完成,没有任何USB/Serial配置协议供PC软件读写。v4做Rust上位机UI是**全新能力**,需要自行设计一套配置协议(建议:USB HID自定义report或CDC虚拟串口+简单JSON/二进制帧,复用v4已有的config_manager结构做双向同步),而不是"抄"v3.0(v3.0没有对应功能可抄)。

### Q. WorkMode(SERIAL_MODE/HID_MODE)切换 — 结论:**无运行时UI入口**
- grep`setWorkMode`调用点,全局零匹配(除了本落盘文件的记录)。
- 说明`InputWorkMode`只能在编译期/main.cpp初始化时设定,或通过未发现的隐藏机制设置,菜单系统里没有暴露"切换到HID模式"的选项(多处页面仅**读取**work_mode来决定是否显示某些功能,如binding_settings仅在SERIAL_MODE下可用)。v4若要支持模式切换,这是v3.0完全没有的UI能力,需要新增。

## 任务状态: DONE
所有待办已完成，最终报告已写入 f:\mai2control\mai2control-v4\.kiro\context\v3_ui_parity_baseline.md，已通过subagent_response汇报给主agent。本任务结束，无需继续。
