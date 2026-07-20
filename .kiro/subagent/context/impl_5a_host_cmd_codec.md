# 任务: impl_5a_host_cmd_codec - RP2040 USB CDC 二进制帧编解码 + 命令分发

## 任务完成状态：✓ 完成

## 概述
成功实现了 RP2040 固件中的 USB CDC 二进制帧编解码 + 命令分发骨架。编译通过，编译结果：
- RAM: 18424 bytes / 262144 bytes (7.0%)
- Flash: 152360 bytes / 3141632 bytes (4.8%)

## 已完成的工作

### 1. 新建帧编解码模块
**文件位置**: `src/protocol/host_cmd/host_cmd.h` & `host_cmd.cpp`

#### 功能实现:
- **帧格式完全按设计文档实现**:
  - SOF0=0xAA, SOF1=0x55, cmd(u8), flags(u8), seq(u8), len(u16 LE), payload[len], crc16(u16 LE)
  - payload 上限 4096 字节，超限丢弃保护
  
- **CRC-16/CCITT-FALSE 工具**:
  - 类: `HostCmdCrc16::crc16()` (内联)
  - 参数: poly=0x1021, init=0xFFFF
  - 覆盖范围: cmd..payload
  
- **接收状态机** (`HostCmdCodec`):
  - 增量喂字节 API: `feed_byte(uint8_t byte, HostFrame* out_frame)`
  - 状态流: FIND_SOF0 → FIND_SOF1 → READ_HEADER → READ_PAYLOAD → READ_CRC
  - CRC 校验失败时自动重新查找 SOF，鲁棒错误恢复
  - 返回解析好的 `HostFrame{cmd, flags, seq, len, payload[4096]}`
  
- **编码函数** (`HostCmdCodec::encode_frame`):
  - 输入: `const HostFrame&`, 输出缓冲
  - 自动计算 CRC 并追加
  - 支持快捷函数: `encode_ack()`, `encode_nak(err_code, msg)`
  
- **命令码 enum class** (按设计文档 1.3):
  - 系统域: HELLO(0x01), DEVICE_INFO(0x02), PING(0x03), SAVE_CONFIG(0x0E), RESET_DEFAULTS(0x0F)
  - 配置KV域: CFG_GET(0x10), CFG_SET(0x11), CFG_GET_GROUP(0x12), CFG_GET_ALL(0x13), CFG_SET_BATCH(0x14)
  - CapSense调参域: PARAM_GET(0x20), PARAM_SET(0x21), PARAM_GET_ALL(0x22), CALIBRATE(0x23), BASELINE_RESET(0x24)
  - 遥测流域: TELEM_START(0x30), TELEM_STOP(0x31), TELEM_DATA(0x32)
  - 绑区域: BIND_START(0x40), BIND_ABORT(0x41), BIND_CONFIRM(0x42), BIND_GET_MAP(0x43), BIND_SET_MAP(0x44), BIND_EVENT(0x45)
  - 灯效域: LED_GET(0x50), LED_SET_REGION(0x51), LED_PREVIEW(0x52)
  - 应答: ACK(0x7E), NAK(0x7F)
  - NAK 错误码: NOT_IMPLEMENTED(0x01), INVALID_PARAM(0x02), DEVICE_BUSY(0x03), CONFIG_ERROR(0x04), SENSOR_ERROR(0x05)

### 2. 命令分发骨架
**类**: `HostCmdDispatcher` (单例)

#### 实现:
- **HELLO** (0x01) → **DEVICE_INFO** (0x02):
  - payload: protocol_version(u16 LE=0x0001), fw_version(u32 LE=0x00000400), capsense_channels(u8=36), capability_bits(u32 LE=0x00000007)
  - capability_bits: bit0=遥测可用, bit1=配置可用, bit2=绑区可用
  
- **PING** (0x03) → **ACK**:
  - 简单心跳响应，回程 seq 相同，flags.bit0=1(响应)
  
- **其余命令** → **NAK(NOT_IMPLEMENTED)**:
  - 清晰的 TODO 注释标记后续 #5b/#5c 接入点
  - payload: err_code(0x01) + "TODO: #5b/#5c" 字符串
  
- **可扩展的处理器注册**:
  - `register_handler(HostCmd cmd, CmdHandler handler)` 供后续扩展
  - 稀疏处理器映射，不浪费 256 个无用条目

### 3. 接入 UsbComm 服务
**文件**: `src/service/usb_comm/usb_comm.h` & `.cpp`

#### 修改:
- **init()**: 初始化编解码器 (`_codec.init()`)
- **update()**: 
  - 从 CDC 读入可用字节，一次性批量读 64B 缓冲
  - 逐字节喂给编码器
  - 完整帧到达时调用 `HostCmdDispatcher::dispatch()`
  - 响应用 `cdc_write()` 发回
  - 不阻塞，单次 loop 处理完所有可用数据
  
- **编译宏**: `HOST_CMD_BINARY_MODE` 默认启用
  - 禁用时文本诊断代码保留但不执行，避免污染二进制帧流

### 4. 接入 main.cpp
**文件**: `src/main.cpp`

#### 修改(仅添加 2 行调用):
1. 头部添加 include: `#include "service/usb_comm/usb_comm.h"`
2. **setup()** 中添加: `UsbComm::getInstance()->init();`（在 HAL_USB 之后）
3. **loop()** 中添加: `UsbComm::getInstance()->update();`（在 `HAL_USB_Device::getInstance()->task()` 之后）

**重要**: 
- PSoC SWD 烧录部分、bring-up 逻辑完全保留，未做任何修改
- 文本诊断（1s 一次 cdc_write）仍然存在但被编译宏 `HOST_CMD_BINARY_MODE` 隐藏
- 不会导致二进制帧污染

## 编译验证
✓ **编译成功** ([SUCCESS] Took 1.89 seconds)

资源占用:
- **RAM**: 18424 / 262144 bytes (7.0%)
- **Flash**: 152360 / 3141632 bytes (4.8%)

## 约束遵循
✓ 类内私有成员以 `_` 开头，对外接口不加
✓ 状态机使用 private enum，状态变量用 `_` 前缀
✓ 处理器映射为稀疏 bool 数组 + handler 数组，无重复创建
✓ CRC 计算为内联静态函数
✓ 编码/解码为自包含逻辑，不依赖外部库
✓ 循环 ≤3 层（编解码器状态机单层，payload 遍历单层）
✓ 错误恢复鲁棒（CRC 失败重查 SOF，长度超限丢弃）
✓ 命令框架符合协议设计，enum 值精确对齐
✓ 未新增测试文件或 README

## 后续 #5b/#5c 接入点

### #5b - 配置 KV 支持
**接入位置**: `HostCmdDispatcher::_handle_not_implemented`
```
CFG_GET(0x10)      → 调用 ConfigManager::get() + 编码响应
CFG_SET(0x11)      → 调用 ConfigManager::set() + ACK
CFG_GET_GROUP(0x12) → 调用 ConfigManager::get_group() + 批量编码
CFG_GET_ALL(0x13)  → 调用 ConfigManager::get_all() + 批量编码
CFG_SET_BATCH(0x14) → 调用 ConfigManager::set_batch() + ACK
SAVE_CONFIG(0x0E)  → 触发 ConfigManager::save_config()
RESET_DEFAULTS(0x0F) → 触发 ConfigManager::reset_to_defaults()
```

### #5c - 遥测 / 参数 / 校准支持
**接入位置**: 同上 `_handle_not_implemented`
```
TELEM_START(0x30)    → 初始化 sensor_link 遥测流
TELEM_STOP(0x31)     → 停止遥测
TELEM_DATA(0x32)     → 从 sensor_link 获取快照，编码推送（dev→host 主动）
PARAM_GET(0x20)      → sensor_link 查询 PSoC 参数
PARAM_SET(0x21)      → sensor_link 下发参数到 PSoC
CALIBRATE(0x23)      → sensor_link 触发 PSoC 校准
BASELINE_RESET(0x24) → sensor_link 重置基线
```

这些接入点已标记为 `TODO: #5b/#5c`，配套参数结构在 protocol_design.md 中详细定义。

## 技术细节记录

### CRC-16/CCITT-FALSE 验证
- 多项式: 0x1021
- 初值: 0xFFFF
- 反射输入/输出: 否
- 终值 XOR: 0x0000
- 数据字节顺序: MSB first
- 校验范围: SOF 之后的 5 字节头 + payload（不含 SOF、CRC 本身）

### 状态机设计
接收侧采用**严格的序列验证**:
1. 双 SOF 字节校准（避免假 0xAA 或 0x55）
2. 严格长度检查（超 4096 即丢弃）
3. CRC 失败后重新查找 SOF（滑窗恢复）

### 内存效率
- 帧结构 `HostFrame` 使用固定 4096B payload 数组（栈/堆友好）
- 编解码器状态变量用 static/private，避免频繁重分配
- 单例模式（CDC 流量单向）

---

**交付日期**: [自动时间戳]
**编译命令**: `cd main_firmware && pio run`
**烧录**: 硬件端处理（不涉及本任务）
