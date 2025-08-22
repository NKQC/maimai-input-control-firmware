/**
 * * 总体要求: 全局中相似功能的类应当只有一个 类中相似功能的接口只有一个 每个接口或函数只做名称标注的事 具有合理的参数和状态检查 但仅允许在实际使用该部分内容的函数去检查 其他任何位置禁止检查
 * 适当使用内联和静态函数提高效率 尽可能使用跳转表实现状态机
 * bitmap应当是uint32_t定义的bitmap32_t数据类型 如果用于处理状态或配置 应当是struct {a :1, b: 1}这种结构的bitmap
 * 该工程使用的字符串为 std::string 禁止使用Arduino的String
 * 每个文件应当仅写文件名标注的范围 不得存在超纲和混搭情况
 * 每层调用实例都不允许在模块内部初始化另一个模块 必须在app层初始化底层模块 随后初始化上层模块 以此类推
 * 每个模块都应当有适当的异常处理和错误报告方法 可以使用服务UIManager提供的错误报告接口 如果该模块已经正常启动 并通过USB Serial作为Logs
 * 
 * 文件架构要求:
 *     -src
 *      - main.cpp (应用层 直接拿loop loop1分配CPU 添加watchdog 每个服务的loop)
 *      - hal
 *        - 各类HAL模块的文件夹
 *          - HAL模块的.h文件
 *          - HAL模块的.cpp文件
 *      - module
 *        - 各类模块/协议层的文件夹
 *          - 模块的.h文件
 *          - 模块的.cpp文件
 *      - service
 *        - 服务层的文件夹
 *          - 服务的.h文件
 *          - 服务的.cpp文件
 * 
 * 架构框图:
 * ---------- HAL层(HAL层全部是单例模式 多外设的Class 需要分别为每个外设构造派生为同虚class的HAL类 如I2C Virtual ->(HAL_I2C0 / HAL_I2C1) 构造函数和析构函数需要正确处理硬件初始化 时钟等任何相关的硬件资源 析构函数需要正确释放资源 HAL不保存任何数据 没有可保存的状态) ----------
 * [HAL] I2C Class(I2C0/I2C1) (提供底层I2C接口 根据需要至多存在2个实例 I2C0和I2C1 提供底层接口交互操作 构造时传入引脚配置 读取(DMA实现 允许直接传入一块内存地址直接读取数据到该内存中) 写入接口(提供DMA内存区域注册 允许直接注册一块内存并触发DMA直接发送) 虚函数统一实现接口) ->(任何库调用时均需要传入实例 使用实例交互)
 * [HAL] UART Class(UART0/UART1) (提供底层UART接口 根据需要至多存在2个实例 UART0和UART1 提供底层接口交互操作 构造时传入引脚配置 可选启动硬件流控 读取(DMA实现的环形buffer) 写入接口(提供DMA内存区域注册 允许直接注册一块内存并触发DMA直接发送) 虚函数统一实现接口) ->(任何库调用时均需要传入实例 使用实例交互)
 * [HAL] SPI Class(SPI0/SPI1) (提供底层SPI接口 根据需要至多存在2个实例 SPI0和SPI1 提供底层接口交互操作 构造时传入引脚配置 可选启动硬件流控 读取(DMA实现的环形buffer) 写入接口(提供DMA内存区域注册 允许直接注册一块内存并触发DMA直接发送) 虚函数统一实现接口) ->(任何库调用时均需要传入实例 使用实例交互)
 * [HAL] PIO Class(PIO0/PIO1) (提供底层PIO接口 根据需要至多存在2个实例 PIO0和PIO1 提供底层接口交互操作 构造时传入引脚配置 允许外部传入ASM) ->(任何库调用时均需要传入实例 使用实例交互)
 * [HAL] USB Class  (提供底层USB接口 并实现USB HID协议 和 USB Serial协议 提供接口读取配置)
 * 
 * ---------- 协议层(协议层允许多实例 但所有硬件相关设置 如GPIO HAL必须由构造函数一次引入 并且全程不管HAL和GPIO释放 只负责协议解析和数据交互 协议层不保存任何数据 由调用的服务负责传入和保存 如果需要保存配置 应当由服务传入指针 实际数据存在服务中并包含在服务的配置中保存) ----------
 * [底层IC接口映射]Class GTX312L(每个模块一个实例) 提供 GTX312L 的初始化 读取和全部配置接口 用I2C HAL实例交互
 *      -> 实例映射的模块名称格式 {I2CNUM}_{ADDR} 返回的是Bitmap 映射0-11点的触发情况 例如 I2C0 0xB3挂载的模块 名称是 I2C0_B3 返回一个bitmap 每个bit表示一个点的触发情况 1表示触发 0表示未触发
 *      -> 提供每秒采样计数接口 
 *      -> 提供每个通道的触摸灵敏度调整(0-63) 
 *      -> 提供其他寄存器设置 具体见datasheet 完整映射 
 *      -> 提供接口 支持按通道关闭 输入一个bitmap完成开关
 *      -> 以最高速率进行采样 关闭防抖
 *      -> 实例注册到 InputManager
 * [底层IC接口映射]Class MCP23S17(每个模块一个实例) 提供 MCP23S17 的初始化 读取写入GPIO/中断配置和全部配置接口 用SPI HAL实例交互
 *      -> 应当为每个MCP23S17的GPIO提供映射enum 以协助后面的服务设置引脚 其设备名为 {SPIBUS}_{CS_NUM} 其引脚名应当为 {SPIBUS}_{CS_NUM}_{GPIO_NUM} 如 SPIBUS0_CS0_PA0 表示SPI0的CS0引脚PA0
 *      -> 响应时以Bitmap返回 只需要提供设备名 返回输入电平的Bitmap 供InputManager高效使用
 *      -> 实例注册到 InputManager
 * [底层IC接口映射] Class NeoPixel(每个模块一个实例) 提供如WS2812这种Neopixel IC的初始化 驱动 设置色彩接口 用PIO HAL实例实现 该模块需要用ASM构造接口 实现WS2812的驱动电平生成逻辑 
 *      -> 支持1-128长度的灯链 且可以任意指定某个编号或某组编号的灯亮度和色彩信息 接口使用0x000000 RGB数据传入
 *      -> 实例注册到 LightManager
 * [底层IC接口映射] Class ST7735S(每个模块一个实例) 提供 ST7735S(160x80) 的初始化 写入数据接口 用SPI HAL实例交互
 *      -> 实例注册到 UIManager
 * [协议映射] Class Mai2Serial 提供maimai touch serial串口数据包装发送参数 内置指令解析函数 使用传入的UART Class实例提供的接口交互
 *      -> 提供maimai的触摸区域逻辑映射枚举
 *      -> 实例注册到 InputManager
 * [协议模块] Class Mai2Light 提供maimai light serial串口数据包装发送参数 内置指令解析函数 使用注册回调函数方式实现功能(回调函数严格遵守 输入->指令解析参数 返回->指令返回数据 再由调用方将返回发回去) 提供Loop用于读取并触发指令解析并调用回调函数 使用传入的UART Class实例提供的接口交互
 *      -> 实例注册到 LightManager
 * [协议模块] Class USB_Serial_Logs 提供USB Serial 直接调用USB HAL
 *      -> 提供全局Logs 支持四档级别反馈 DEBUG INFO WARNING ERROR
 * [协议模块] Class HID_KEYBOARD 提供HID键盘协议 直接调用USB HAL 通过接口直接映射KEYBOARD状态 传入bitmap形式数据进行触发
 *      -> 实例注册到 InputManager
 * [协议模块] Class HID_TOUCH    提供HID触摸屏协议 直接调用USB HAL 通过接口直接映射TOUCH状态 传入点位Struct list(自有数据类型 包含触摸坐标X Y 点位的编号) 该点位是更新制度 即按下更新一次 抬起更新一次 重复设置相同状态不更新
 *      -> 实例注册到 InputManager
 * 
 * ---------- 服务层(服务层全程单例 尽可能使用静态和内联实现 构造和析构函数只作为内存管理使用 每个服务都应该具有独立的Init函数 Init函数传入构造struct 每个服务模块都应当有一个自己的struct数据类型用于初始化 该struct传入了所有需要使用的实例 如果该服务或该服务调用的实例需要加入循环 则需要提供loop函数 服务使用的其他任何底层的实例使用loop 都需要在服务层包装后再交给主循环执行) ----------
 * 服务层规则: 
 * 1.触摸区域选择的按下标准: 当用户按下需要绑定的区域时 应当有且只有一个区域在有区域按下的1秒间隔内触发 才能算作选择上 否则抛弃该选择继续等待
 * 2.所有配置的键都应当以预处理形式被定义在每个服务各自的.h中 且不直接使用任何字符串键获取/写入配置 全部使用预处理完成 预处理名称定义应当是 {服务名}_{子模块(如有)}_{键}
 * 3.所有服务的配置不应该存在Class内部 每个服务都要经过下列模式处理
 *      - 每个服务构造一个struct作为私有配置
 *      - 构造一个静态的公共公开函数[配置保管函数] 该函数保存一个静态的私有配置变量 它返回该私有地址的指针
 *      - 构造一个静态的公共公开函数[配置加载函数] 该函数调用[配置保管函数]获取指针并从ConfigManager获取到所有需要的配置 存入该指针
 *      - 构造一个静态的公共公开函数[配置读取函数] 该函数调用[配置保管函数]获取指针 并复制一个配置的副本 返回该配置副本
 *      - 构造一个静态的公共公开函数[配置写入函数] 该函数调用[配置保管函数]获取指针 并将其参数传回ConfigManager
 *   外部获取配置应当使用[配置读取函数] 写入配置直接调用[配置写入函数] 使用ConfigManager内置的数据类型 天然检查边界
 *   服务本身完全不保存配置 完全有外部公共公开函数处理
 *   该架构实现完全的单数据存储和及时响应配置更变
 * 4.服务的状态应当由内部的enum定义并使用唯一的私有变量处理 状态机的切换应当是switch维护的跳转表
 * 
 * 
 * [服务模块/输入处理] Class InputManager(单例) 管理并调度触摸底层/触摸区域绑定和映射/键盘映射/键盘->触摸映射  GTX312L应当支持最多6组 即6*12个触摸位点
 *                                    -> 物理按键注册应当由独立数据类型完成注册 并返回list 以list编号为按键ID 按键应当可以设置引脚(引脚可以是MCU本地的引脚 也可以是MCP23S17的GPIO) 触发电平
 *                                    -> 触摸的输入模式存在两种 通过设置进行切换 HID(直接模拟成HID触摸屏) / Serial(调用Mai2Serial类执行指定协议) 这两个模式互斥   
 *                                    -> [GTX312L实例管理]将每个模块的点位分别进行映射转换为bitmap 转换后称之为物理区 名称格式 {I2C通道}_{模块Addr} 映射为bitmap 每个模块一张表
 *                                    -> 设置和获取灵敏度直接调用底层接口 不要缓存
 *                                    -> [Mai2Serial] 调用该模块实例 直接传入触发映射分区bitmap 使其直接调用uart发送符合要求的报文 并注册处理函数到该模块
 *                                    -> 触摸映射具有两种模式 Serial模式就按照Mai2的分区映射 HID模式则允许绑定XY坐标
 *                                    -> 键盘输入允许映射到触摸逻辑位点 两种模式均可
 *                                    -> 定义逻辑区数组 允许将一个物理区映射到1-2个逻辑区中 -> 该映射信息通过ConfigManager存取 (触摸和键盘都适用 触摸需要映射到触摸逻辑区 键盘需要映射到键盘逻辑区)
 *                                    -> 物理区反馈为bitmap -> 映射到绑定的逻辑分区 (触摸和键盘都适用 触摸需要映射到触摸逻辑区 键盘需要映射到键盘逻辑区)
 *                                    -> 绑定分区(每个点位允许绑定2个逻辑分区)
 *                                    -> 不要在Loop中进行任何额外的操作和检查 所有异常使用惰性处理 除非抛出异常否则完全不检查 不要设置轮询周期 直接最大化速度轮询和处理 所有操作直接直通 以最简化和最高速率完成 
 *                                    -> 提供Loop0接口(运行在CPU0上的loop) (读取原始分区触发bitmap -> 映射到逻辑区 -> 模式切换([Serial模式](传递给Mai2Light -> 调用Mai2Light的Loop 处理UART消息) / [HID模式](将触摸映射Bitmap发送到FIFO跨核心传输)))
 *                                    -> 提供Loop1接口(运行在CPU1上的loop) (读取MCP23S17数据 -> 映射到键盘逻辑区 -> 映射到HID_KEYBOARD / [如果使用HID模式](接收触摸Bitmap 映射到预先设置好的点位坐标))
 *                                    -> 提供一个单通道自动触摸灵敏度调整接口 在映射完成后可用 自动校准灵敏度 (在调用前会完成:将一个导电模块悬浮在要调整的区域触摸区域上方指定高度 调用时需要完成: 逐级增大灵敏度(每次调整都需要重新校准 后面不再赘述) 直到可以感应到触摸 并记录下当前灵敏度 随后逐级缩小灵敏度 直到找到失去触摸的灵敏度 随后再逐级增大灵敏度 一旦发现只增大一级就能检测到触摸的临界点 则记录这个临界点 如果需要增大几级才能重新找到 则找到响应的中间值灵敏度 返回 不保存 这个模块只负责找临界点)
 *                                    -> 触摸映射应当根据启动时设置的模式决定调用 提供设置映射绑定接口 和一个手动触发映射的接口 允许指定一个映射区域直接触发 以进行选区设置(Serial的传入触发逻辑区域 HID的传入坐标 编号默认使用第一位)
 *                                    -> 未绑定的触摸区域应当通过GTX312L接口关闭通道 使其提高稳定性和速度 此时只在需要执行绑定操作时开启全部通道
 * 
 * [服务模块/灯光配置] Class LightManager(单例) Mai2Light串口协议专用的按键灯光映射服务 全部由配置文件映射
 *                                    -> 单个灯区域可以映射至多32个Neopixel位置 使用bitmap完成这些位置的选址
 *                                    -> 直接注册Mai2Light回调 将读取的指定灯映射到Neopixel指定路径上 包括色彩和亮度等映射
 *                                    -> 提供Loop接口 用于读取Mai2Light的回调函数 并根据配置文件映射 调用Neopixel的设置颜色接口 实现灯光映射
 *                                    -> 提供一个手动触发映射的接口 允许指定一个映射区域直接触发 以进行灯光设置(Serial的传入触发逻辑区域 HID的传入坐标 编号默认使用第一位)
 * 
 * [服务模块/用户界面] Class UIManager(单例) 使用LVGL构造彩色UI 使用现代化风格菜单 驱动ST7735S TFT屏幕 提供可视化设置 可设置其他服务模块 注意 该服务只能调用模块接口实现功能 除屏幕相关设置 自身不进行任何其他调用和设置 状态机和页面位置由内部维护
 *                                    -> 使用摇杆开关作为输入 其具有A B BUTTON三个按钮 其中A/B是拨动方向生效 其为数字输入 作为上下翻页键 在设置数值时应当在拨动一边时前指定ms只上/下调数值一次 随后加速滚动/调整数值 按钮即确认功能 LOW有效 无需软件防抖
 *                                    -> 该菜单应当具备以下模块 每个模块的设置应当具备子菜单 默认显示的是状态界面 只有按下确认键才进入设置界面
 *                                          -> 设置InputManager的工作模式 切换Serial和HID模式
 *                                          -> 设置InputManager的触摸区域映射编号 并根据其工作模式执行映射
 *                                             选择逻辑点位时/物理点位时 顺带显示它的绑定信息 如果是HID则显示绑定的响应坐标
 *                                             例如Serial模式下 选择点位 (例如是C2) -> 等待按下 -> 一旦检测到有且只有一个区域被按下 则检查该区域有没有达到绑定上限 如果没有 则绑定上 更新配置 如果超过上限了则提示
 *                                             如果是HID模式 则 选择点位 -> 等待按下 -> 一旦检测到有且只有一个区域被按下 -> 使用该物理区 -> 选择逻辑点(0 / 1) -> 每个物理点可映射多个逻辑点 物理点状态映射到逻辑点 每个逻辑点可设置一个坐标 -> 设置坐标(如果之前有坐标数据了 接着上次的设置) -> 此时使用InputManager的接口构造一个触摸点位发给HID 将当前响应坐标发过去 使其在屏幕上能正确显示触发位置 通过摇杆分别设置X Y 位置 确认无误保存
 *                                          -> 使用InputManager执行引导式绑区 该模式仅限于Serial模式 我们已知Mai2的区域设置 当执行时 执行预先设置的keyboard按键组 (由一个函数负责执行 直接按时间顺序按下按键以转到目标程序的设置中) -> 随后 按A1- F*依次设置 直接按顺序通过串口构造该区域保持按下 -> 将该物理区域绑定到刚才映射按下的逻辑区域中 随后进位下一个区域 以此类推 直到全部区域绑定完成 显示确认键 取消键或重新绑定 让用户选择 实现自动绑区
 *                                          -> 设置InputManager的按键区域绑定 允许设置每个注册的KEY按键到HID键盘的键位映射 每个HID按键只能被一个KEY按键绑定 KEY按键可以绑定多个HID按键 实现组合键 最多绑定3个 
 *                                          -> 设置InputManager的触摸映射到HID功能 该模式仅在Serial模式且绑区完成时可用 可以将A1-A8逻辑区的触发直接映射到设置的HID按键上 提供默认值 也可修改
 *                                          -> 设置InputManager的所有可用触摸传感器下的所有通道的每通道单独灵敏度调整 (0-63) 级  以模块为单位的灵敏度统一偏移
 *                                          -> 启动InputManager的自动触摸灵敏度调整 首先通过触摸按下确定区域 -> 调用接口执行调整 -> 显示目标灵敏度 询问用户是否应用 -> 应用则更新灵敏度 否则放弃
 *                                          -> 设置LightManager中每个灯区域对应的Neopixel编号 一个区域至多32个 由右侧的矩阵网格提供选择灯 一个灯只能映射一个灯区域
 *                                          -> 设置LightManager中每个灯区域的颜色 一个区域至多32个 由右侧的矩阵网格提供选择灯 一个灯只能映射一个灯区域
 *                                          -> 设置屏幕的息屏时间 无操作指定时间后息屏且跳过Loop 节省资源给其他部分
 *                                          -> InputManager状态界面(默认界面) 可以读取已检测到的触摸模组数量和总线/地址 每个模组的触摸点位开关情况 绑定的区域(Serial模式是逻辑区域名称 HID是坐标) 灵敏度 是否在触发(这些显示都具备层级结构 首先是网格直接显示每个点和是否触发(以模块分区 同模块的放一起 按顺序排列) 进入点的层级后才是名字 灵敏度和绑定区域 键盘部分也一样) 每个模块的当前轮询速率 可以读取键盘每个KEY的名称和触发状态 和该KEY的绑定状态 
 *                                          -> 设置ConfigManager 提供保存配置 重置配置
 *                                          -> 屏幕不需要屏保 只需要息屏 拨动摇杆唤醒屏幕 
 *                                          -> [应当提供全局接口]故障界面 一旦启动出现任何故障 则直接使用故障界面 显示发生异常的模块或未通过的部分 并提供重启按钮
 *                                    -> 提供Loop接口 用于读取用户输入 并根据输入调用其他服务模块的接口 实现用户界面的功能
 * 
 * [服务层/存取配置] Class ConfigManager(单例) 配置文件管理类 提供保存和加载参数的能力 直接交互flash和读取设置 该模块全局使用map进行交互 完全使用键值对对参数读写
 *                                    -> 每个模块定义存储参数规则: 每个模块必须在各自所属区域使用预处理+std::string作为键 值的类型使用ConfigManager定义的数据类型 随后写公开的初始化函数 并调用ConfigManager将函数注册 以提供配置定义和初始化
 *                                    -> ConfigManager内部自行通过struct定义常用数据类型 并在后续所有读写中直接使用自定义的数据类型 每个数字型(如任何长度的int float double等)数据类型 都允许设置最大值 最小值和默认值 和超标时进行的动作(限制到最近的合法值/拒绝修改) 每个字符串型都允许设置最大长度 长度超标时进行的动作(直接截断/驳回设置)和默认值
 *                                    -> 配置将被以json写入板载flash 并以json形式读取 json中存储了每个要存储数据的键 值和值的数据类型 和其他任何附加内容(这一点在保存设置接口中说明)
 *                                    -> 该模块由以下两个主要的私有变量执行运行时配置操作 <默认配置MAP>: 仅在实例被执行初始化时执行: 其调用所有注册的初始化函数 按顺序依次将默认函数的配置参数写入该函数    <运行时配置MAP>: 这是该管理器通过外部接口修改参数和读取参数时间接交互的变量 相当于配置文件数据的运行时映射
 *                                    -> [Init:初次初始化]默认条件: flash指定区域未找到配置文件   执行: 直接将<默认配置MAP>仅拷贝数据完整复制一份给 <运行时配置MAP>
 *                                    -> [Init:正常初始化/加载设置]默认条件: flash指定区域找到配置文件     执行: 直接读取文件 解析json 一旦发现解析失败或序列化后构造出的<运行时配置MAP>计算出的CRC和文件末尾CRC不一致 则直接执行[初次初始化] 并在logs中警告并显示两次计算的CRC 若读取成功且CRC匹配 则直接将配置依次构造放入 <运行时配置MAP>
 *                                    -> [接口:保存设置] 执行: 将<运行时配置MAP>中的数据按照json格式(键 值 值的数据类型)序列化构造json文件并写入flash 并在写完设置后 根据序列化前的<运行时配置MAP>计算CRC32 保存结果到json文件结尾
 *                                    -> [接口:修改配置] 传入: 键名, 自定义数据类型 返回: 修改是否成功 执行: 检查键名是否存在于<运行时配置MAP>存在:(检查自定义数据类型是否符合<运行时配置MAP>中数据类型中存储的极限值 如果符合则直接写入 不符合则执行限制策略) 不存在: (检查键名是否存在于<默认配置MAP> 存在: 按<运行时配置MAP>存在处理 不存在: 拒绝修改 提示键未注册)
 *                                    -> [接口:读取配置] 传入: 键名 返回: 目标数据(数据类型仍然为自定义数据类型) 执行: 检查键名是否存在于<运行时配置MAP>存在:直接返回目标数据 不存在:(检查键名是否存在于<默认配置MAP> 存在:返回<默认配置MAP>的对应数据 不存在: 返回空数据类型 提示键不存在)
 *                                    -> [接口:重置配置] 执行: 直接将<默认配置MAP>仅拷贝数据完整复制一份给 <运行时配置MAP> 并调用保存设置接口 保存配置
 * 
 * ---------- 应用层 (main.cpp) ----------
 * 统合初始化全部模块 并指派任务给CPU 除此之外不做其他事
 * 启动顺序:
 *      HAL实例 -> 协议层实例 -> 服务层(ConfigManager最优先启动 随后其他) -> 指派Loop
 * CPU分配:
 * 0: InputManager Loop0, LightManager Loop
 * 1: InputManager Loop1, UIManager Loop
 * 
 * GTX312L自动发现实现(每个总线分别处理): I2C_HAL扫描总线 -> 发现总线设备 -> 按发现的所有总线地址依次 GTX312L构造实例 传入对应总线HAL和地址 尝试初始化 -> 初始化成功 加入注册组
 *                                                                                                             -> 初始化失败 删除实例
 * 交互式Serial分区绑定: 临时启用全部触摸点 -> 按mai2触摸区域从A1开始准备绑定 -> 屏幕提示绑定区域A1 提示按下A1区域 -> mai2直接发送A1被按下的消息 -> 按下A1区域 -> 读取到是哪个区域被触发 -> A1被绑定到该区域 -> 关闭该区域的触控输入 -> 接着下一个区域 -> ...... -> 直到全部的mai2区被绑定完成 提示保存结束 -> 恢复原触摸点启用设置
 * 交互式HID绑定分区: 临时启用全部触摸点 -> 按下需要绑定的区域 -> 屏幕显示按下的点位名称 -> 设置绑定到逻辑X点位 -> 构造HID触摸点位并发送 -> 屏幕设置X Y坐标 点位实时跟随 -> 直到实际点位坐标满足需求 -> 保存结束 -> 恢复原触摸点启用设置
 */

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

// HAL层包含
#include "hal/i2c/hal_i2c.h"
#include "hal/uart/hal_uart.h"
#include "hal/spi/hal_spi.h"
#include "hal/pio/hal_pio.h"
#include "hal/usb/hal_usb.h"

// 协议层包含
#include "protocol/gtx312l/gtx312l.h"
#include "protocol/mcp23s17/mcp23s17.h"
#include "protocol/neopixel/neopixel.h"
#include "protocol/st7735s/st7735s.h"
#include "protocol/mai2serial/mai2serial.h"
#include "protocol/mai2light/mai2light.h"
#include "protocol/usb_serial_logs/usb_serial_logs.h"
#include "protocol/hid/hid.h"

// 服务层包含
#include "service/config_manager/config_manager.h"
#include "service/input_manager/input_manager.h"
#include "service/light_manager/light_manager.h"
#include "service/ui_manager/ui_manager.h"

// 系统配置
#define SYSTEM_VERSION "3.0.0"
#define HARDWARE_VERSION "1.0.0"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// 引脚定义
#define LED_BUILTIN_PIN 25
#define I2C0_SDA_PIN 4
#define I2C0_SCL_PIN 5
#define I2C1_SDA_PIN 6
#define I2C1_SCL_PIN 7
#define SPI0_MISO_PIN 16
#define SPI0_MOSI_PIN 19
#define SPI0_SCK_PIN 18
#define SPI0_CS_PIN 17
#define SPI1_MISO_PIN 12
#define SPI1_MOSI_PIN 15
#define SPI1_SCK_PIN 14
#define SPI1_CS_PIN 13
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART1_TX_PIN 8
#define UART1_RX_PIN 9
#define NEOPIXEL_PIN 2
#define ST7735S_DC_PIN 20
#define ST7735S_RST_PIN 21
#define ST7735S_CS_PIN 22

// Watchdog配置
#define WATCHDOG_TIMEOUT_MS 5000
#define WATCHDOG_FEED_INTERVAL_MS 1000

// 全局对象声明
static HAL_I2C* hal_i2c0 = nullptr;
static HAL_I2C* hal_i2c1 = nullptr;
static HAL_SPI* hal_spi0 = nullptr;
static HAL_SPI* hal_spi1 = nullptr;
static HAL_UART* hal_uart0 = nullptr;
static HAL_UART* hal_uart1 = nullptr;
static HAL_PIO* hal_pio0 = nullptr;
static HAL_USB* hal_usb = nullptr;

static GTX312L* gtx312l_devices[8] = {nullptr}; // 最多8个GTX312L设备
static uint8_t gtx312l_count = 0;
static MCP23S17* mcp23s17 = nullptr;
static NeoPixel* neopixel = nullptr;
static ST7735S* st7735s = nullptr;
static Mai2Serial* mai2_serial = nullptr;
static Mai2Light* mai2_light = nullptr;
static USB_SerialLogs* usb_logs = nullptr;
static HID* hid = nullptr;

static ConfigManager* config_manager = nullptr;
static InputManager* input_manager = nullptr;
static LightManager* light_manager = nullptr;
static UIManager* ui_manager = nullptr;

// 系统状态
static bool system_initialized = false;
static bool system_error = false;
static uint32_t system_uptime = 0;
static uint32_t last_heartbeat = 0;
static uint32_t last_watchdog_feed = 0;
static uint32_t last_status_update = 0;

// Core1任务状态
static volatile bool core1_running = false;
static volatile bool core1_error = false;

// 函数声明
bool init_hal_layer();
bool init_protocol_layer();
bool init_service_layer();
void deinit_system();
void core0_task();
void core1_task();
void core1_entry();
void heartbeat_task();
void watchdog_task();
void status_update_task();
void error_handler(const char* error_msg);
void print_system_info();
void scan_i2c_devices();
void emergency_shutdown();

/**
 * HAL层初始化
 */
bool init_hal_layer() {
    // 初始化I2C
    hal_i2c0 = HAL_I2C0::getInstance();
    if (!hal_i2c0 || !hal_i2c0->init(I2C0_SDA_PIN, I2C0_SCL_PIN, 400000)) {
        error_handler("Failed to initialize I2C0");
        return false;
    }
    
    hal_i2c1 = HAL_I2C1::getInstance();
    if (!hal_i2c1 || !hal_i2c1->init(I2C1_SDA_PIN, I2C1_SCL_PIN, 400000)) {
        error_handler("Failed to initialize I2C1");
        return false;
    }
    
    // 初始化SPI
    hal_spi0 = HAL_SPI0::getInstance();
    if (!hal_spi0 || !hal_spi0->init(SPI0_SCK_PIN, SPI0_MOSI_PIN, SPI0_MISO_PIN, 1000000)) {
        error_handler("Failed to initialize SPI0");
        return false;
    }
    
    hal_spi1 = HAL_SPI1::getInstance();
    if (!hal_spi1 || !hal_spi1->init(SPI1_SCK_PIN, SPI1_MOSI_PIN, SPI1_MISO_PIN, 1000000)) {
        error_handler("Failed to initialize SPI1");
        return false;
    }
    
    // 初始化UART
    hal_uart0 = HAL_UART0::getInstance();
    if (!hal_uart0 || !hal_uart0->init(UART0_TX_PIN, UART0_RX_PIN, 115200)) {
        error_handler("Failed to initialize UART0");
        return false;
    }
    
    hal_uart1 = HAL_UART1::getInstance();
    if (!hal_uart1 || !hal_uart1->init(UART1_TX_PIN, UART1_RX_PIN, 115200)) {
        error_handler("Failed to initialize UART1");
        return false;
    }
    
    // 初始化PIO
    hal_pio0 = HAL_PIO0::getInstance();
    if (!hal_pio0 || !hal_pio0->init()) {
        error_handler("Failed to initialize PIO0");
        return false;
    }
    
    // 初始化USB
    hal_usb = HAL_USB_Device::getInstance();
    if (!hal_usb || !hal_usb->init()) {
        error_handler("Failed to initialize USB");
        return false;
    }
    
    return true;
}

/**
 * 协议层初始化
 */
bool init_protocol_layer() {
    // 扫描I2C设备并初始化GTX312L
    scan_i2c_devices();
    
    // 初始化MCP23S17
    mcp23s17 = new MCP23S17(hal_spi0, 0);
    if (!mcp23s17 || !mcp23s17->init()) {
        error_handler("Failed to initialize MCP23S17");
        return false;
    }
    
    // 初始化NeoPixel
    neopixel = new NeoPixel(hal_pio0, NEOPIXEL_PIN, 128);
    if (!neopixel || !neopixel->init()) {
        error_handler("Failed to initialize NeoPixel");
        return false;
    }
    
    // 初始化ST7735S
    st7735s = new ST7735S(hal_spi1, ST7735S_DC_PIN, ST7735S_RST_PIN, ST7735S_CS_PIN);
    if (!st7735s || !st7735s->init()) {
        error_handler("Failed to initialize ST7735S");
        return false;
    }
    
    // 初始化Mai2Serial
    mai2_serial = new Mai2Serial(hal_uart0);
    if (!mai2_serial || !mai2_serial->init()) {
        error_handler("Failed to initialize Mai2Serial");
        return false;
    }
    
    // 初始化Mai2Light
    mai2_light = new Mai2Light(hal_uart1);
    if (!mai2_light || !mai2_light->init()) {
        error_handler("Failed to initialize Mai2Light");
        return false;
    }
    
    // 初始化USB Serial Logs
    usb_logs = new USB_SerialLogs(hal_usb);
    if (!usb_logs || !usb_logs->init()) {
        error_handler("Failed to initialize USB Serial Logs");
        return false;
    }
    
    // 初始化HID (使用单例模式)
    hid = HID::getInstance();
    if (!hid || !hid->init(hal_usb)) {
        error_handler("Failed to initialize HID");
        return false;
    }
    
    return true;
}

/**
 * 服务层初始化
 */
bool init_service_layer() {
    // 首先初始化ConfigManager
    config_manager = ConfigManager::getInstance();
    if (!config_manager || !config_manager->init()) {
        error_handler("Failed to initialize ConfigManager");
        return false;
    }
    
    // 初始化InputManager
    input_manager = InputManager::getInstance();
    
    // 准备InputManager初始化配置
    InputManager::InitConfig input_config;
    input_config.mai2_serial = mai2_serial;
    input_config.hid = hid;
    input_config.ui_manager = ui_manager;
    input_config.mcp23s17 = mcp23s17;
    
    if (!input_manager->init(input_config)) {
        error_handler("Failed to initialize InputManager");
        return false;
    }
    
    // 注册MCP23S17的GPIO 1-11作为键盘
    if (mcp23s17) {
        // 注册GPIOA1-A8 (对应MCP_GPIO::GPIOA1到GPIOA8)
        for (int i = 1; i <= 8; i++) {
            MCP_GPIO gpio = static_cast<MCP_GPIO>(0xC0 + i); // GPIOA1-A8
            input_manager->addPhysicalKeyboard(gpio, HID_KeyCode::KEY_NONE);
        }
        
        // 注册GPIOB1-B3 (对应MCP_GPIO::GPIOB1到GPIOB3)
        for (int i = 9; i <= 11; i++) {
            MCP_GPIO gpio = static_cast<MCP_GPIO>(0xC0 + i); // GPIOB1-B3
            input_manager->addPhysicalKeyboard(gpio, HID_KeyCode::KEY_NONE);
        }
        
        // 设置GPIOB8为输出模式并输出高电平以点亮LED
        mcp23s17->set_pin_direction(MCP23S17_PORT_B, 7, MCP23S17_OUTPUT); // GPIOB8是端口B的第7位(0-7)
        mcp23s17->write_pin(MCP23S17_PORT_B, 7, true); // 输出高电平
    }
    
    // 注册GTX312L设备到InputManager
    for (uint8_t i = 0; i < gtx312l_count; i++) {
        if (gtx312l_devices[i]) {
            if (!input_manager->registerGTX312L(gtx312l_devices[i])) {
                error_handler("Failed to register GTX312L device");
                return false;
            }
        }
    }
    
    // 初始化LightManager
    light_manager = LightManager::getInstance();
    
    if (!light_manager->init()) {
        error_handler("Failed to initialize LightManager");
        return false;
    }
    
    // 初始化UIManager
    ui_manager = UIManager::getInstance();
    UIManager_Config ui_config = {};
    ui_config.config_manager = config_manager;
    ui_config.input_manager = input_manager;
    ui_config.light_manager = light_manager;
    ui_config.st7735s = st7735s;
    
    if (!ui_manager->init(ui_config)) {
        error_handler("Failed to initialize UIManager");
        return false;
    }
    
    return true;
}

/**
 * 扫描I2C设备并初始化GTX312L
 */
void scan_i2c_devices() {
    gtx312l_count = 0;
    
    if (usb_logs) {
        usb_logs->info("Starting I2C device scan...");
    }
    
    // GTX312L自动发现实现: I2C_HAL扫描总线 -> 发现总线设备 -> 按发现的所有总线地址依次 GTX312L构造实例 传入对应总线HAL和地址 尝试初始化 -> 初始化成功 加入注册组
    //                                                                                                             -> 初始化失败 删除实例
    
    // 扫描I2C0总线
    if (hal_i2c0) {
        std::vector<uint8_t> i2c0_addresses = hal_i2c0->scan_devices();
        for (uint8_t addr : i2c0_addresses) {
            if (gtx312l_count >= 8) break;
            
            // 尝试创建GTX312L实例
            GTX312L* device = new GTX312L(hal_i2c0, I2C_Bus::I2C0, addr);

            if (device && device->init()) {
                // 初始化成功，加入注册组
                gtx312l_devices[gtx312l_count] = device;
                gtx312l_count++;
                
                // 注册到InputManager
                if (input_manager) {
                    input_manager->registerGTX312L(device);
                }
                
                if (usb_logs) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "Found GTX312L on I2C0: %s (0x%02X)", 
                            device->get_device_name().c_str(), addr);
                    usb_logs->info(std::string(buffer));
                }
            } else {
                // 初始化失败，删除实例
                delete device;
            }
        }
    }
    
    // 扫描I2C1总线
    if (hal_i2c1 && gtx312l_count < 8) {
        std::vector<uint8_t> i2c1_addresses = hal_i2c1->scan_devices();
        for (uint8_t addr : i2c1_addresses) {
            if (gtx312l_count >= 8) break;
            
            // 尝试创建GTX312L实例
            GTX312L* device = new GTX312L(hal_i2c1, I2C_Bus::I2C1, addr);
            if (device && device->init()) {
                // 初始化成功，加入注册组
                gtx312l_devices[gtx312l_count] = device;
                gtx312l_count++;
                
                // 注册到InputManager
                if (input_manager) {
                    input_manager->registerGTX312L(device);
                }
                
                if (usb_logs) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "Found GTX312L on I2C1: (0x%02X)", addr);
                    usb_logs->info(std::string(buffer));
                }
            } else {
                // 初始化失败，删除实例
                delete device;
            }
        }
    }
    
    if (usb_logs) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "I2C scan completed. Total GTX312L devices found: %d", gtx312l_count);
        usb_logs->info(std::string(buffer));
        
        // 输出设备详细信息
        for (uint8_t i = 0; i < gtx312l_count; i++) {
            if (gtx312l_devices[i]) {
                GTX312L_DeviceInfo info;
                if (gtx312l_devices[i]->read_device_info(info)) {
                    char buffer[256];
                    snprintf(buffer, sizeof(buffer), "Device %d: %s, I2C=0x%02X, Valid=%s", 
                            i, gtx312l_devices[i]->get_device_name().c_str(),
                            info.i2c_address, info.is_valid ? "Yes" : "No");
                    usb_logs->info(std::string(buffer));
                }
            }
        }
    }
}

/**
 * 系统反初始化
 */
void deinit_system() {
    // 停止Core1
    core1_running = false;
    
    // 反初始化服务层
    if (ui_manager) {
        ui_manager->deinit();
        ui_manager = nullptr;
    }
    
    if (light_manager) {
        light_manager->deinit();
        light_manager = nullptr;
    }
    
    if (input_manager) {
        input_manager->deinit();
        input_manager = nullptr;
    }
    
    if (config_manager) {
        config_manager->deinit();
        config_manager = nullptr;
    }
    
    // 反初始化协议层
    if (hid) {
        hid->deinit();
        delete hid;
        hid = nullptr;
    }
    
    if (usb_logs) {
        usb_logs->deinit();
        delete usb_logs;
        usb_logs = nullptr;
    }
    
    if (mai2_light) {
        mai2_light->deinit();
        delete mai2_light;
        mai2_light = nullptr;
    }
    
    if (mai2_serial) {
        mai2_serial->deinit();
        delete mai2_serial;
        mai2_serial = nullptr;
    }
    
    if (st7735s) {
        st7735s->deinit();
        delete st7735s;
        st7735s = nullptr;
    }
    
    if (neopixel) {
        neopixel->deinit();
        delete neopixel;
        neopixel = nullptr;
    }
    
    if (mcp23s17) {
        mcp23s17->deinit();
        delete mcp23s17;
        mcp23s17 = nullptr;
    }
    
    // 反初始化GTX312L设备
    for (uint8_t i = 0; i < gtx312l_count; i++) {
        if (gtx312l_devices[i]) {
            gtx312l_devices[i]->deinit();
            delete gtx312l_devices[i];
            gtx312l_devices[i] = nullptr;
        }
    }
    gtx312l_count = 0;
    
    // 反初始化HAL层
    if (hal_usb) {
        hal_usb->deinit();
        delete hal_usb;
        hal_usb = nullptr;
    }
    
    if (hal_pio0) {
        hal_pio0->deinit();
        delete hal_pio0;
        hal_pio0 = nullptr;
    }
    
    if (hal_uart1) {
        hal_uart1->deinit();
        delete hal_uart1;
        hal_uart1 = nullptr;
    }
    
    if (hal_uart0) {
        hal_uart0->deinit();
        delete hal_uart0;
        hal_uart0 = nullptr;
    }
    
    if (hal_spi1) {
        hal_spi1->deinit();
        delete hal_spi1;
        hal_spi1 = nullptr;
    }
    
    if (hal_spi0) {
        hal_spi0->deinit();
        delete hal_spi0;
        hal_spi0 = nullptr;
    }
    
    if (hal_i2c1) {
        hal_i2c1->deinit();
        delete hal_i2c1;
        hal_i2c1 = nullptr;
    }
    
    if (hal_i2c0) {
        hal_i2c0->deinit();
        delete hal_i2c0;
        hal_i2c0 = nullptr;
    }
    
    system_initialized = false;
}

/**
 * Core0任务 - InputManager Loop0, LightManager Loop
 */
void core0_task() {
    uint32_t last_input_update = 0;
    uint32_t last_light_update = 0;
    
    while (system_initialized && !system_error) {
        uint32_t current_time = millis();
        
        // InputManager Loop0 - 每1ms执行一次 (触摸数据处理和Serial模式)
        if (current_time - last_input_update >= 1) {
            if (input_manager) {
                input_manager->loop0();
            }
            last_input_update = current_time;
        }
        
        // LightManager Loop - 每10ms执行一次
        if (current_time - last_light_update >= 10) {
            if (light_manager) {
                light_manager->task();
            }
            last_light_update = current_time;
        }
        
        // 心跳和看门狗任务
        heartbeat_task();
        watchdog_task();
        status_update_task();
    }
}

/**
 * Core1任务 - InputManager Loop1, UIManager Loop
 */
void core1_task() {
    uint32_t last_input_update = 0;
    uint32_t last_ui_update = 0;
    
    core1_running = true;
    core1_error = false;
    
    while (core1_running && !system_error) {
        uint32_t current_time = millis();
        
        // InputManager Loop1 - 每5ms执行一次 (键盘数据处理和HID模式)
        if (current_time - last_input_update >= 5) {
            if (input_manager) {
                input_manager->loop1();
            }
            last_input_update = current_time;
        }
        
        // UIManager Loop - 每50ms执行一次
        if (current_time - last_ui_update >= 50) {
            if (ui_manager) {
                ui_manager->task();
            }
            last_ui_update = current_time;
        }
        
        // 移除延迟以实现最高速率处理
    }
    
    core1_running = false;
}

/**
 * Core1入口函数
 */
void core1_entry() {
    core1_task();
}

/**
 * 心跳任务
 */
void heartbeat_task() {
    uint32_t current_time = millis();
    
    if (current_time - last_heartbeat >= 1000) {
        // 切换内置LED状态
        static bool led_state = false;
        digitalWrite(LED_BUILTIN_PIN, led_state ? HIGH : LOW);
        led_state = !led_state;
        
        // 更新系统运行时间
        system_uptime = current_time / 1000;
        
        last_heartbeat = current_time;
    }
}

/**
 * 看门狗任务
 */
void watchdog_task() {
    uint32_t current_time = millis();
    
    if (current_time - last_watchdog_feed >= WATCHDOG_FEED_INTERVAL_MS) {
        watchdog_update();
        last_watchdog_feed = current_time;
    }
}

/**
 * 状态更新任务
 */
void status_update_task() {
    uint32_t current_time = millis();
    
    if (current_time - last_status_update >= 10000) {
        if (usb_logs) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "System uptime: %lu seconds", system_uptime);
            usb_logs->info(std::string(buffer));
            snprintf(buffer, sizeof(buffer), "Core1 running: %s", core1_running ? "Yes" : "No");
            usb_logs->info(std::string(buffer));
            snprintf(buffer, sizeof(buffer), "GTX312L devices: %d", gtx312l_count);
            usb_logs->info(std::string(buffer));
        }
        last_status_update = current_time;
    }
}

/**
 * 错误处理函数
 */
void error_handler(const char* error_msg) {
    system_error = true;
    
    if (usb_logs) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "SYSTEM ERROR: %s", error_msg);
        usb_logs->error(std::string(buffer));
    }
    
    // 如果UI管理器可用，显示错误界面
    if (ui_manager) {
        ui_manager->show_status_info();
    }
    
    // 紧急关机
    emergency_shutdown();
}

/**
 * 打印系统信息
 */
void print_system_info() {
    if (usb_logs) {
        usb_logs->infof("=== MaiMai Controller V%s ===", SYSTEM_VERSION);
        usb_logs->infof("Hardware Version: %s", HARDWARE_VERSION);
        usb_logs->infof("Build Date: %s %s", BUILD_DATE, BUILD_TIME);
        usb_logs->infof("CPU Frequency: %lu MHz", rp2040.f_cpu() / 1000000);
        usb_logs->info("Free Heap: Available", "Memory");
        usb_logs->info("Flash Size: Available", "Storage");
        usb_logs->info("==============================");
    }
}

/**
 * 紧急关机
 */
void emergency_shutdown() {
    // 停止所有任务
    core1_running = false;
    
    // 关闭所有LED
    if (neopixel) {
        neopixel->clear_all();
        neopixel->show();
    }
    
    // 关闭屏幕背光
    if (st7735s) {
        st7735s->set_backlight(0);
    }
    
    // 禁用看门狗
    watchdog_disable();
    
    // 等待一段时间后重启
    delay(5000);
    watchdog_reboot(0, 0, 0);
}

/**
 * Arduino setup函数
 */
void setup() {
    // 初始化内置LED
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW);
    
    // 启用看门狗
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    
    // 初始化系统
    bool init_success = true;
    
    // 初始化HAL层
    if (!init_hal_layer()) {
        init_success = false;
    }
    
    // 初始化协议层
    if (init_success && !init_protocol_layer()) {
        init_success = false;
    }
    
    // 初始化服务层
    if (init_success && !init_service_layer()) {
        init_success = false;
    }
    
    if (init_success) {
        system_initialized = true;
        print_system_info();
        
        if (usb_logs) {
            usb_logs->info("System initialization completed successfully");
        }
        
        // 启动Core1任务
        multicore_launch_core1(core1_entry);
        
        // 等待Core1启动
        while (!core1_running && !system_error) {
            delay(10);
        }
        
        if (usb_logs) {
            usb_logs->info("Core1 task started");
        }
    } else {
        if (usb_logs) {
            usb_logs->error("System initialization failed");
        }
        emergency_shutdown();
    }
}

/**
 * Arduino loop函数 - Core0主循环
 */
void loop() {
    if (system_initialized && !system_error) {
        core0_task();
    } else {
        // 系统未初始化或出现错误，进入紧急模式
        emergency_shutdown();
    }
}
