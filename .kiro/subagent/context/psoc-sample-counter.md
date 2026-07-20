# 任务: psoc-sample-counter

## 目标
PSoC 内置采样计数器 → RP2040 计算每秒采样率+通道刷新延迟 → 经 telemetry 透传 → UI 显示。
需求: 即使没接触摸面板，扫描也在跑，计数也应增长(CSD 无论是否触摸都持续扫描)。

## 设计(已定，勿改语义)
- PSoC: 新增 `volatile uint32_t scan_count`，每次全 36 通道扫描完成 +1(自由递增，无需时间基准)。
  新增 SPI 命令 GET_STATS(0x35) 返回 scan_count u32。
- RP2040: 每 ~500ms 读 scan_count，用 time_us_32 算 samples_per_sec 与 scan_period_us(=1e6/rate=每通道刷新延迟)。
- telemetry: 新增帧级字段位 TELEM_FIELD_STATS(0x10)，置位时帧头 fields 字节后附 samples_per_sec(u32 LE)+scan_period_us(u32 LE)。
- Rust/UI: 解码这两个 u32；UI 全通道页显示"采样率 X Hz / 通道延迟 Y us"。

## SPI 线格式(7字节帧 [magic,cmd,b2,b3,p0,p1,p2])
- GET_STATS 请求: [A5,0x35,0,0,0,0,0]
- GET_STATS 响应: [A5,0x35,0, sc0,sc1,sc2,sc3] (scan_count u32 LE 在字节[3..6])

## 状态（全部完成）
- [x] PSoC main.c: scan_count + GET_STATS (opcode 0x35, 全局 scan_count, 主循环++, spi_load_stats, switch case)
- [x] RP2040 psoc_types.h: Cmd::GET_STATS = 0x35
- [x] RP2040 psoc_spi: get_stats() (h+cpp)
- [x] RP2040 psoc.h/.cpp: 统计计算+getter (samples_per_sec/scan_period_us, update()里500ms折算段)
- [x] RP2040 sensor_link: TELEM_FIELD_STATS(0x10) 帧头附加 (h常量 + cpp tick()里插入8字节)
- [x] Rust proto/telemetry.rs: FIELD_STATS(0x10) + TelemFrame 加 samples_per_sec/scan_period_us + decode_telem_data 解码段(帧头fields后，通道循环前，8字节，含越界检查)
- [x] Rust proto/mod.rs: re-export FIELD_STATS
- [x] Rust app_state: AppController 加 telem_samples_per_sec/telem_scan_period_us 字段+getter；_handle_telem_data 存入；start_telemetry 内部对传入 fields OR 上 FIELD_STATS(不改签名)
- [x] Rust main.rs + app.slint: 主窗口+AllChannelsPage 加 sample_rate_hz/channel_delay_us in property，main.rs 每 tick 用 ctrl.telem_samples_per_sec()/telem_scan_period_us() 回填；AllChannelsPage 工具栏加 Text 显示"采样率 X Hz 通道延迟 Y us"
- [x] selftest.rs: Step5 fields 0x0F→0x1F；soak D 段 fields 0x0F→0x1F，并在遥测数据打印后加一行采样率/通道延迟打印

## 已改动文件(全部)
- psoc_firmware/CY8C4147AZI-SensorCore/main.c
- main_firmware/src/protocol/psoc/psoc_types.h
- main_firmware/src/protocol/psoc/psoc_spi.h / .cpp
- main_firmware/src/protocol/psoc/psoc.h / .cpp
- main_firmware/src/service/sensor_link/sensor_link.h / .cpp
- control_software/src/proto/telemetry.rs
- control_software/src/proto/mod.rs
- control_software/src/app_state/mod.rs
- control_software/src/main.rs
- control_software/ui/app.slint
- control_software/src/bin/selftest.rs

## 验证结果
- `dev.ps1 build-rp`：[SUCCESS]（RAM 13.4%，Flash 15.1%），无警告无错误。
- `dev.ps1 build-ui`：Finished dev profile，无 error。
- `cargo test --lib`（control_software）：70 passed; 0 failed。既有 telemetry 解码测试(RAW/DIFF等，fields 不含 STATS 位)全部通过，说明新增的 STATS 分支不影响旧路径(向后兼容)。

## 偏差说明
无实质偏差。唯一小选择：start_telemetry 未改签名，而是在函数体内部对传入 fields `|= FIELD_STATS`，保证任何调用方(main.rs on_telem_start / selftest 里旧写法)发出的 TELEM_START 都自动带 STATS 位——规格里"更稳妥"选项已采用。main.rs 里 on_telem_start 调用处未手动加 FIELD_STATS(无需要，start_telemetry 内部已保证)；selftest.rs 里两处按规格要求手动把 0x0F 改成了 0x1F(显式)，两者不冲突，双重保证。

## PSoC 侧未编译
按要求未编译/烧录 PSoC 固件，仅完成源码改动，由主 agent 处理。

## 完成并验证(实测)
- PSoC 构建(27532B)→embed 0x00000405→build-rp→flash-rp→启动重编程 PSoC 成功(diagnose: embedded PSoC=0x00000405, DIAGNOSE PASS)。
- soak 实测: 采样率=109 Hz, 通道延迟=9174 us, 36 通道有数据, got_real=true, SOAK PASS。
- 无触摸面板时采样率仍 109 Hz(raw≈173 基线) → "未接面板也有计数" 满足。
- full PASS(含 SET_PARAM/SET_MODE/APPLY 交互配置往返) → 交互配置能力正常。
- 全部四层落地并验证。剩: 重建 release exe(含 UI 采样率显示)。
