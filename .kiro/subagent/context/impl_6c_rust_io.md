# Task #6c: Rust USB CDC IO 层实现

## 任务目标
在 `src/io/mod.rs` 实现完整的 USB CDC IO 层，包括：
1. 设备枚举 `list_devices() -> Vec<DeviceCandidate>`
2. IO 事件枚举 `pub enum IoEvent`
3. IO 句柄 `pub struct IoHandle`
4. 后台 IO 线程启动 `pub fn spawn(port_name: &str) -> anyhow::Result<IoHandle>`

## 已完成准备工作
- ✓ 读取 proto 模块完整实现（Frame、encode、Decoder、DeviceInfo）
- ✓ 理解协议规范（protocol_design.md 修订1）
- ✓ 检查 main.rs 模块声明
- ✓ 验证 Cargo.toml 依赖完备（serialport、anyhow、log）

## Proto 模块复用清单
- `crate::proto::Frame` - 帧结构
- `crate::proto::encode(&Frame) -> Vec<u8>` - 帧编码
- `crate::proto::Decoder` - 帧解码器（new/feed/feed_bytes/reset）
- `crate::proto::DeviceInfo` - 设备信息

## 实现计划（分步落盘）
### Step 1: 数据结构定义（DeviceCandidate, IoEvent, IoHandle）
### Step 2: 设备枚举函数 list_devices()
### Step 3: IO 线程主循环逻辑
### Step 4: spawn() 函数与线程启动
### Step 5: 编译验证 + 单测验证
### Step 6: 最终报告交付

## 关键约束
- VID=0x0CA3, PID=0x0024
- 波特率 115200(CDC 忽略)
- 线程安全：std::sync::mpsc + Arc<AtomicBool>
- 不新增重依赖
- 默认私有，仅公开：IoHandle/IoEvent/DeviceCandidate/list_devices/spawn
- 禁止重造帧逻辑（复用 proto）

## 状态
当前：开始 Step 1


## ✓ 完成的工作

### Step 1: 数据结构定义 ✓
- `DeviceCandidate`: VID/PID/port_name/product/serial_number
- `IoEvent`: Connected/Disconnected/Error/Frame
- `IoHandle`: cmd_tx/evt_rx/running/thread_handle

### Step 2: 设备枚举函数 ✓
- `list_devices()`: 过滤 VID=0x0CA3, PID=0x0024 的 USB 设备
- 返回空表示未找到设备（正常）

### Step 3: IO 线程主循环逻辑 ✓
- 排空出站：cmd_rx.try_recv() → encode → port.write_all()
- 读入站：port.read() → Decoder.feed_bytes() → IoEvent::Frame
- 错误处理：区分设备断开 vs 其它错误
- 智能休眠：无数据时休眠避免忙等

### Step 4: spawn() 函数 ✓
- serialport::new(port_name, 115200).timeout(10ms).open()
- 创建 mpsc channel (cmd_tx/cmd_rx, evt_tx/evt_rx)
- spawn 后台线程运行 io_thread_main()
- 返回 IoHandle (cmd_tx/evt_rx/running/thread_handle)

### Step 5: 编译验证 ✓
- cargo build: 成功（13 warnings 为设计目的 - 未被 main 调用）
- cargo test: 14 tests passed (含 4 io module 单测 + 10 proto 单测)
- cargo build --release: 成功生成 12.6 MB 二进制

### Step 6: 代码质量
- 遵循 Rust 惯用法
- 默认私有，仅公开 IoHandle/IoEvent/DeviceCandidate/list_devices/spawn
- 复用 proto 模块（encode/Decoder），不重造
- 线程安全用 std（mpsc + Arc<AtomicBool>），无新重依赖
- 错误处理完善（Result/anyhow）
- 日志完整（debug/info/warn/error）

## 公开 API 清单

### 类型
- `pub struct DeviceCandidate { port_name, vid, pid, product, serial_number }`
- `pub enum IoEvent { Connected, Disconnected, Error(String), Frame(Frame) }`
- `pub struct IoHandle { cmd_tx, evt_rx, running, _thread_handle }`

### 函数
- `pub fn list_devices() -> Vec<DeviceCandidate>`
  - 枚举所有符合 VID/PID 的设备，无设备返回空
- `pub fn spawn(port_name: &str) -> Result<IoHandle>`
  - 启动 IO 线程，返回句柄或错误

### IoHandle 方法
- `pub fn send(&self, frame: Frame) -> Result<()>` - 发送命令帧
- `pub fn try_recv(&self) -> Option<IoEvent>` - 尝试接收事件
- `pub fn recv_timeout(&self, timeout: Duration) -> Result<IoEvent>` - 阻塞接收
- `pub fn stop(self)` - 优雅停止线程

## 线程读写与断开处理策略

### 出站（UI → 设备）
1. 非阻塞 try_recv() 循环排空 cmd_rx
2. 每个 Frame 用 proto::encode() 转字节
3. port.write_all() 发送
4. 写错误 → 发 Disconnected 事件并退出

### 入站（设备 → UI）
1. port.read(&mut buf) 读字节
2. ErrorKind::TimedOut → 正常继续（无数据）
3. 字节喂 Decoder
4. 每个完整 Frame 发 IoEvent::Frame

### 断开检测
- 写错误 → 立即 Disconnected（设备移除）
- 读错误（NotFound/PermissionDenied 等）→ 判断后发 Disconnected
- 其它读错误 → 发 Error 事件但继续运行

## 给后续任务的接入点

### #6d（Windows COM 口识别与改名）
- 在 app_state 初始化时调用 `io::list_devices()`
- 根据 port_name 识别 COM 口，调用 SetupAPI 做映射识别
- 返回给用户可选择的设备列表

### #6e~g（UI 配置/绑区/曲线）
- UI 选择设备后调用 `io::spawn(port_name)`
- 获得 IoHandle，启动接收循环（非阻塞或超时接收）
- 发送命令帧：`handle.send(Frame::hello(seq))`
- 接收事件帧：`while let Some(evt) = handle.try_recv() { ... }`
- 解析 Frame.payload 得到 DEVICE_INFO/TELEM_DATA/PARAM_RESPONSE 等
- 关闭时 `handle.stop()`

## 验证结果总结
- ✓ cargo build 成功，仅 warning（预期）
- ✓ cargo test 14/14 通过
- ✓ cargo build --release 成功生成可执行文件
- ✓ 无编译错误，无运行时 panic
- ✓ 代码遵循项目规范与 Rust 惯用法

## 状态
✓ COMPLETE - 所有需求实现完毕，编译测试验证通过
