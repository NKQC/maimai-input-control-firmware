# impl_5c1_sensor_link 任务状态

## 状态：已完成，编译通过

## 目标
RP2040 固件 `service/sensor_link/` 遥测服务(桩数据)，实现 TELEM_*/PARAM_*/CALIBRATE/BASELINE_RESET 命令处理 + 周期性推 TELEM_DATA 帧。严格按 protocol_design.md 修订3 T1~T5。

## 涉及/改动文件
- 新建 `main_firmware/src/service/sensor_link/sensor_link.h`
- 新建 `main_firmware/src/service/sensor_link/sensor_link.cpp`
- 修改 `main_firmware/src/main.cpp`：include sensor_link.h；setup() 里 `SensorLink::getInstance()->init()`（在 `UsbComm::init()` 之后）；loop() 里 `SensorLink::getInstance()->tick()`（在 `UsbComm::update()` 之后，SWD bring-up 逻辑完全未动）。

## 关键实现
- 单例 `SensorLink`，成员：`_streaming/_mode/_rate_hz/_fields/_ch_mask/_last_emit_us/_stream_seq`，参数表 `_params[36][11]`（u32），发送缓冲 `_tx_buf[512]` + 复用的 `_telem_frame`（HostFrame 成员，避免栈上/重复分配）。
- `_param_index(param_id)`：0x01..0x0A → 0..9，0x80(SMARTSENSE_EN) → 10，非法 → -1。
- 桩数据源 `_gen_channel`：baseline=5000+缓慢正弦漂移；raw=baseline+正弦(相位随ch偏移)+xorshift32伪随机噪声([-5,5])；diff=raw-baseline；status=diff>finger_th。TODO 标注在函数注释处，待 #5d 换 PSoC SPI 快照。
- 命令处理器（静态方法，注册进 `HostCmdDispatcher::getInstance()->register_handler`）：
  - `TELEM_START(0x30)`：解析 mode+rate_hz+fields+ch_mask，rate_hz clamp [1,1000]，ACK。
  - `TELEM_STOP(0x31)`：置 `_streaming=false`，ACK。
  - `PARAM_GET(0x20)` / `PARAM_SET(0x21)` / `PARAM_GET_ALL(0x22)`：按 T3 线格式，非法 channel/param_id → NAK(INVALID_PARAM)。
  - `CALIBRATE(0x23)` / `BASELINE_RESET(0x24)`：桩阶段仅校验 payload 长度后回 ACK，TODO 标注待 #5d 路由到 PSoC SPI。
- `tick()`：用 `time_us_32()` 判断是否到发送间隔（`1000000/_rate_hz`），到点则遍历 ch_mask 选中通道组 TELEM_DATA payload（`ts_us+ch_count+fields+[ch_index+按fields选raw/bsln/diff/status]`），`HostCmdCodec::encode_frame` 编码（flags=HOST_CMD_FLAG_STREAM）后经 `HAL_USB_Device::getInstance()->cdc_write` 发出；mode==1(单次) 发一帧后置 `_streaming=false`。

## 验证结果
`pio run`（cwd=main_firmware）：**[SUCCESS]**，耗时 7.46s。
RAM: 10.3% (26952/262144 bytes)
Flash: 14.5% (456060/3141632 bytes)
无编译错误；仅 lwip 框架自带的既有 warning（与本次改动无关）。

## 待办（#5d 交接点，已在代码中标注 TODO）
1. `SensorLink::_gen_channel`：换成读取 PSoC SPI 双缓冲遥测快照。
2. `_handle_param_get/_handle_param_set/_handle_param_get_all`：换成路由到 PSoC SPI GET_PARAM(0x20)/SET_PARAM(0x21)（当前是 RP2040 侧参数表回环）。
3. `_handle_calibrate/_handle_baseline_reset`：换成路由到 PSoC SPI CALIBRATE(0x23)/BASELINE_RESET(0x24)。
4. 协议线格式对上位机不变，仅内部数据源替换。

## 无异常需报告
未触发编译失败重试、无架构冲突，一次实现即编译通过，任务已完成。
