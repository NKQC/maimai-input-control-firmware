该软件开发目的仅供个人学习交流 使用该软件造成的任何法律责任与开发者无关
如需商业使用或任何其他操作 详见本仓库附带的LICENSE条款
仓库所有者保留修改使用条款的权利

固件基于RP2040构建 使用PlatofrmIO管理 Arduino框架
该工程的HAL库重写了HID报文接口
建议在修改前先确保能正常编译
编译时注意需要注释USB库中的冲突部分 具体在编译时会报错 注释库中报错部分 不改动工程即可

固件配套硬件: https://oshwhub.com/washing_controol_ap101/maimaitouchpad-controlv2-0

基础特性:
    1. 基于Tinyusb的自定义HID 键盘+触摸屏 可扩展
    2. 使用TFT屏幕菜单简易设置大部分参数 中文界面 (下面提到的几乎所有内容 除键盘和触控映射外(开关不影响) 都可以通过屏幕设置和保存)
        TFT驱动使用DMA极低开销运行(测试时TFT的刷新先倒下 系统运行无可察觉影响)
        管理器内置协程和页面渲染器 支持元素快速拼接页面 支持元素回调
        自动启动页面滚动和分页处理
    3. 更新固件不清除配置 基于LittleFS json/Protocol的动态配置管理 允许不同版本固件使用相同的配置文件无缝切换 内置CRC校验与异常值约束 自动异常自恢复
    4. 自发现 自接管基于I2C的触摸IC 目前支持GTX312L和AD7147 框架式拓展 极速适配新设备
        自动发现 注册与初始化 初始化时自动加载该设备保存的配置
        基于设备ID判断设备 同型号设备 使用不同的配置 设备ID为8位 (7: I2C总线地址 6-0: I2C 7位地址)
        架构支持最大24通道设备 每个设备的通道都有各自的ID 通道ID为32位 [(31-24)设备ID 23-0 通道Bitmap]
        以上所有的ID和处理都是自动进行的 对用户透明 且后续所有对设备的操作基于这些ID实现
        配置持久化且以设备ID分辨 支持多次启动时增减设备且设备配置不发生改变
        基于IC型号自动启停IC独有功能 (例如AD7147的AFE校准功能)
    5. 支持mai2 serial / light 的串口协议 波特率允许调节范围为9600 - 6M 由模块层支持 可随时扩展更换
        串口全局DMA驱动 默认支持软件自适应流控 UART0支持硬件流控
        触控指令支持按ms延迟发送触控数据 可选0-100ms 实现为us级时间戳对齐
        触控状态与运算时间无关 全程O(1)
        延迟环形缓冲区具有自训练特性 匹配时间为ns级
        交互式区域绑定 且绑区与物理设备地址绑定 不吃设备位置 只管地址上有没有对应设备
        可配置的触控区域映射按键 UI可选开关:
            触控区域直接映射到键盘 允许独立选择映射选项 可1-多个区域映射到按键上 支持条件触发 满足纯内无按键需求
    6. 支持PIO驱动的WS2812灯链 调用几乎无开销 HAL层PIO ASM可动态注册 随时替换功能并复用
        LightManager模块支持将Mai2Light协议中定义的单个灯映射到多个指定的Neopixel地址上
    7. HID键盘us级响应 支持动态映射按键
        支持MCP/MCU GPIO状态直接映射到按键 且允许映射多个按键
        需要注意 开发该部分时需要时刻注意拔模块电源 测试时回报率直接干碎搭载Win11 24H2系统的9950X
            导致键盘彻底失灵且看起来缓冲区溢出 按键混乱 唯有冷重启解
    8. 系统架构为 HAL -> protocol -> service 每个部分均为模块化组成 如果需要替换功能 可随时更新和使用
        除必须使用面向对象写法的部分 其余均为静态处理
        HAL和服务均为单例模式

触控特性:
    仅AD7147模块支持:
        1. 内置全通道并发校准算法 自动检测CDC值计算杂散电容 自动校对波动差处理干扰问题
        支持统一选择灵敏度目标 目前支持 低敏(按上触发) 普通(贴上触发) 高敏(隔1mm触发) 超敏(1mm以上触发) 注意 该处为开发者测试34英寸小台的情况 根据制作尺寸与选材 连接方式和环境的差异略有不同
        所有部分只需UI一键操作 且高重复精度 校准仅处理AFE物理电压偏置
        2. 支持绑定后按绑区处理校准 不同绑区可以有不同的灵敏度目标 支持mai2的ABCDE五个区域选择各自目标灵敏度 有望治好手上有洞(灵敏度低吃touch)和不能轮指(A区灵敏度太高连点不鸟)两难问题
        3. 支持大部分参数屏幕上按通道设置 提供极高自由度 如果自动校准不满意 可以手动调整
        设置时提供当前通道实时CDC和触发状态 避免调试凭感觉问题
        4. 该模块触控本质基于CDC(电容-数字转换器) 触发 CDC为16位 具有极高的重复精度和可操作性
        模块默认启用自动校准 防止温度/空气密度变化造成的数值漂移 (甚至可以耦合到房间空气密度变化导致的电容变化上)
    [后续根据issue和开发者自己台子使用情况 可能基于此实装更多妙妙小工具 有望彻底解决内屏人上w6难的问题]

默认按键接口与模块绑定:
    接口已经足够精简 直接放注册源码
        MCP_GPIO即 MCP23S17的GPIO
        MCU_GPIO即 RP2040自带的GPIO
    GPIO->Keyboard:
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
    TouchArea -> Keyboard:
        接口参数:
            (区域(支持多个区域同时作为条件之一 必须全部满足), 
            长按触发时间, 
            触发的Keyboard按键, 
            是否单次触发(
                true:当达到触发条件后 对应按键只按一下 下次触发必须松开且重新到达触发条件
                false: 当到达触发条件后 对应按键一直按下 直到不满足触发条件))
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