## ⚠️ 免责声明

该软件开发目的仅供个人学习交流，使用该软件造成的任何法律责任与开发者无关。

如需商业使用或任何其他操作，详见本仓库附带的 LICENSE 条款。

仓库所有者保留修改使用条款的权利。

## 📋 项目概述

固件基于 RP2040 构建，使用 PlatformIO 管理，Arduino 框架。

该工程的 HAL 库重写了 HID 报文接口。

建议在修改前先确保能正常编译。

编译时注意需要注释 USB 库中的冲突部分，具体在编译时会报错，注释库中报错部分，不改动工程即可。

## 🔗 配套硬件

[固件配套硬件](https://oshwhub.com/washing_controol_ap101/maimaitouchpad-controlv2-0)

## ✨ 基础特性

1. **自定义 HID 接口**
   - 基于 TinyUSB 的自定义 HID 键盘+触摸屏，可扩展

2. **TFT 屏幕菜单系统**
   - 使用 TFT 屏幕菜单简易设置大部分参数，中文界面
   - 下面提到的几乎所有内容（除键盘和触控映射外，开关不影响）都可以通过屏幕设置和保存
   - TFT 驱动使用 DMA 极低开销运行（测试时 TFT 的刷新先倒下，系统运行无可察觉影响）
   - 管理器内置协程和页面渲染器，支持元素快速拼接页面，支持元素回调
   - 自动启动页面滚动和分页处理

3. **动态配置管理**
   - 更新固件不清除配置
   - 基于 LittleFS json/Protocol 的动态配置管理
   - 允许不同版本固件使用相同的配置文件无缝切换
   - 内置 CRC 校验与异常值约束，自动异常自恢复

4. **I2C 触摸 IC 自动管理**
   - 自发现、自接管基于 I2C 的触摸 IC
   - 目前支持 GTX312L 和 AD7147，框架式拓展，极速适配新设备
   - 自动发现、注册与初始化，初始化时自动加载该设备保存的配置
   - 基于设备 ID 判断设备，同型号设备使用不同的配置
   - 设备 ID 为 8 位（7: I2C 总线地址，6-0: I2C 7 位地址）
   - 架构支持最大 24 通道设备，每个设备的通道都有各自的 ID
   - 通道 ID 为 32 位（(31-24) 设备 ID，23-0 通道 Bitmap）
   - 以上所有的 ID 和处理都是自动进行的，对用户透明
   - 配置持久化且以设备 ID 分辨，支持多次启动时增减设备且设备配置不发生改变
   - 基于 IC 型号自动启停 IC 独有功能（例如 AD7147 的 AFE 校准功能）

5. **串口协议支持**
   - 支持 mai2 serial / light 的串口协议
   - 波特率允许调节范围为 9600 - 6M，由模块层支持，可随时扩展更换
   - 串口全局 DMA 驱动，默认支持软件自适应流控，UART0 支持硬件流控
   - 触控指令支持按 ms 延迟发送触控数据，可选 0-100ms，实现为 us 级时间戳对齐
   - 触控状态与运算时间无关，全程 O(1)
   - 延迟环形缓冲区具有自训练特性，匹配时间为 ns 级
   - 交互式区域绑定，且绑区与物理设备地址绑定，不吃设备位置，只管地址上有没有对应设备
   - 可配置的触控区域映射按键，UI 可选开关：
     - 触控区域直接映射到键盘，允许独立选择映射选项
     - 可 1-多个区域映射到按键上，支持条件触发，满足纯内无按键需求

6. **WS2812 灯链支持**
   - 支持 PIO 驱动的 WS2812 灯链，调用几乎无开销
   - HAL 层 PIO ASM 可动态注册，随时替换功能并复用
   - LightManager 模块支持将 Mai2Light 协议中定义的单个灯映射到多个指定的 Neopixel 地址上

7. **HID 键盘功能**
   - HID 键盘 us 级响应，支持动态映射按键
   - 支持 MCP/MCU GPIO 状态直接映射到按键，且允许映射多个按键
   - ⚠️ **注意**：开发该部分时需要时刻注意拔模块电源，测试时回报率直接干碎搭载 Win11 24H2 系统的 9950X，导致键盘彻底失灵且看起来缓冲区溢出，按键混乱，唯有冷重启解

8. **系统架构**
   - 系统架构为 HAL -> protocol -> service，每个部分均为模块化组成
   - 如果需要替换功能，可随时更新和使用
   - 除必须使用面向对象写法的部分，其余均为静态处理
   - HAL 和服务均为单例模式

## 🎯 触控特性

### AD7147 模块专属功能

1. **全通道并发校准算法**
   - 内置全通道并发校准算法，自动检测 CDC 值计算杂散电容
   - 自动校对波动差处理干扰问题
   - 支持统一选择灵敏度目标，目前支持：
     - **低敏**（按上触发）
     - **普通**（贴上触发）
     - **高敏**（隔 1mm 触发）
     - **超敏**（1mm 以上触发）
   - ⚠️ **注意**：该处为开发者测试 34 英寸小台的情况，根据制作尺寸与选材、连接方式和环境的差异略有不同
   - 所有部分只需 UI 一键操作，且高重复精度，校准仅处理 AFE 物理电压偏置

2. **分区域校准支持**
   - 支持绑定后按绑区处理校准，不同绑区可以有不同的灵敏度目标 目前设置为 -10 ~ 10 灵敏度 默认为2 一档增减1024的目标CDC值
   - 支持 mai2 的 ABCDE 五个区域选择各自目标灵敏度
   - 有望治好手上有洞（灵敏度低吃 touch）和不能轮指（A 区灵敏度太高连点不了）两难问题

3. **高自由度参数设置**
   - 支持大部分参数屏幕上按通道设置，提供极高自由度
   - 如果自动校准不满意，可以手动调整
   - 设置时提供当前通道实时 CDC 和触发状态，避免调试凭感觉问题

4. **CDC 触控技术**
   - 该模块触控本质基于 CDC（电容-数字转换器）触发
   - CDC 为 16 位，具有极高的重复精度和可操作性
   - 模块默认启用自动校准，防止温度/空气密度变化造成的数值漂移
   - 甚至可以耦合到房间空气密度变化导致的电容变化上

5. ***mai2Serial 可选功能**
   - 支持触发式更新 / 推送式更新切换
   - 支持在启动延迟的情况下 采样聚合 在不牺牲响应延迟的前提下 允许聚合指定时间(ms)内区域的触控采样 只有指定区域的该时间段内采样均触发 才会判定该区域触发 适用于灵敏度较高和采样速率较高不稳定情况的滤波 避免出现意外区域跳转
   (示例: 当延迟设置为10 该参数为1时 发送时的时间轴内应该储存了当前直到前10ms的数据 此时应该发送10ms前的数据 但10ms-9ms时 只有某个区域全部为触发状态 当前判定的该通道状态才是触发 一旦这1ms的区间内任意一个地方不触发 就判定为非触发情况 以此类推)
   - 支持在触发式更新情况下 对抗某软件的神金串口设计 支持可选的1-10次额外数据重发 且支持新触发打断


> 后续根据 issue 和开发者自己台子使用情况，可能基于此实装更多妙妙小工具，有望彻底解决内屏人上 w6 难的问题

## ⌨️ 默认按键接口与模块绑定

接口已经足够精简，直接放注册源码。

### GPIO 定义说明

- `MCP_GPIO`：MCP23S17 的 GPIO
- `MCU_GPIO`：RP2040 自带的 GPIO

### GPIO -> Keyboard 映射

```cpp
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA0, HID_KeyCode::KEY_W);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA1, HID_KeyCode::KEY_E);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA2, HID_KeyCode::KEY_D);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA3, HID_KeyCode::KEY_C);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA4, HID_KeyCode::KEY_X);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA5, HID_KeyCode::KEY_Z);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA6, HID_KeyCode::KEY_A);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOA7, HID_KeyCode::KEY_Q);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB0, HID_KeyCode::KEY_8);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB1, HID_KeyCode::KEY_3);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB2, HID_KeyCode::KEY_ENTER);
input_manager->addPhysicalKeyboard(MCP_GPIO::GPIOB3, HID_KeyCode::KEY_SPACE);
```

### TouchArea -> Keyboard 映射

#### 接口参数说明

```
(
    区域(支持多个区域同时作为条件之一，必须全部满足),
    长按触发时间,
    触发的 Keyboard 按键,
    是否单次触发(
        true: 当达到触发条件后，对应按键只按一下，下次触发必须松开且重新到达触发条件
        false: 当到达触发条件后，对应按键一直按下，直到不满足触发条件
    )
)
```

#### 映射配置

```cpp
input_manager->addTouchKeyboardMapping(MAI2_A1_AREA, 1000, HID_KeyCode::KEY_W, true);
input_manager->addTouchKeyboardMapping(MAI2_A2_AREA, 1000, HID_KeyCode::KEY_E, true);
input_manager->addTouchKeyboardMapping(MAI2_A3_AREA, 1000, HID_KeyCode::KEY_D, true);
input_manager->addTouchKeyboardMapping(MAI2_A4_AREA, 1000, HID_KeyCode::KEY_C, true);
input_manager->addTouchKeyboardMapping(MAI2_A5_AREA, 1000, HID_KeyCode::KEY_X, true);
input_manager->addTouchKeyboardMapping(MAI2_A6_AREA, 1000, HID_KeyCode::KEY_Z, true);
input_manager->addTouchKeyboardMapping(MAI2_A7_AREA, 1000, HID_KeyCode::KEY_A, true);
input_manager->addTouchKeyboardMapping(MAI2_A8_AREA, 1000, HID_KeyCode::KEY_Q, true);
input_manager->addTouchKeyboardMapping(MAI2_B1_AREA | MAI2_B8_AREA | MAI2_E1_AREA, 1000, HID_KeyCode::KEY_SPACE, true);
input_manager->addTouchKeyboardMapping(MAI2_C1_AREA | MAI2_C2_AREA, 1000, HID_KeyCode::KEY_ENTER);
```