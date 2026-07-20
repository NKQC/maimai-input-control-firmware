# impl_6g1_telem_logic - 完成报告

## 任务目标
✅ 完成：在 Rust 上位机实现遥测/参数编解码 + app_state 遥测缓冲与参数缓存(纯逻辑)。

## 交付内容

### 1. Proto 侧编解码模块 (src/proto/telemetry.rs)
- **参数 ID 常量**：FINGER_TH=0x01 ~ SMARTSENSE_EN=0x80，KNOWN_PARAM_IDS 数组
- **遥测字段位**：FIELD_RAW=0x01, FIELD_BASELINE=0x02, FIELD_DIFF=0x04, FIELD_STATUS=0x08
- **编码函数**：
  - encode_telem_start(mode, rate_hz, fields, ch_mask) → Vec<u8> (12B)
  - encode_param_get(ch, param_id) → Vec<u8> (2B)
  - encode_param_set(ch, param_id, value) → Vec<u8> (6B)
  - encode_param_get_all(ch) → Vec<u8> (1B)
  - encode_ch_mask(ch_mask) → Vec<u8> (8B LE)
- **解码函数**：
  - decode_telem_data(payload) → Result<TelemFrame> — 返回 ts_us + samples
  - decode_param_get(payload) → Result<(ch, param_id, value)>
  - decode_param_get_all(payload) → Result<(ch, Vec<(param_id, value)>)>
- **数据结构**：
  - ChannelSample { ch, raw?, bsln?, diff?, status? }
  - TelemFrame { ts_us, fields, samples }
- **字节级测试**：11 个编解码自检用例，对齐固件 sensor_link.cpp

### 2. Proto 模块更新 (src/proto/mod.rs)
- 加 `pub mod telemetry;`
- 导出关键符号到 pub use (ChannelSample, 编解码函数等)

### 3. App State 扩展 (src/app_state/mod.rs)
- **新字段**：
  - telem_buf: Vec<VecDeque<ChannelSample>> (36 通道，容量 TELEM_CAP=1024)
  - telem_active, telem_last_ts, telem_version (u64)
  - params: Vec<BTreeMap<u8, u32>> (36 通道参数映射)
  - param_version (u64)
- **新方法**：
  - start_telemetry(rate_hz, fields, ch_mask)
  - stop_telemetry()
  - request_param(ch, param_id) / request_params(ch)
  - set_param(ch, param_id, value) — 乐观本地更新+版本号自增
  - calibrate(ch_mask) / baseline_reset(ch_mask)
  - telem_latest(ch) → Option<ChannelSample>
  - telem_series(ch, field) → Vec<f32> (从缓冲提取序列)
  - telem_version() / param_version() (getter)
  - params_of(ch) / param(ch, param_id) (参数查询)
- **Poll 扩展**：
  - 处理 TELEM_DATA 帧：解码→入缓冲→版本号自增
  - 处理 PARAM_GET 响应：更新 params[ch] + 版本号自增
  - 处理 PARAM_GET_ALL 响应：清空并重填 params[ch] + 版本号自增
- **disconnect() 清理**：telem_buf、params、telem_active、telem_last_ts 全清零

### 4. 验证结果
✅ `cargo build` — 成功编译（仅警告，无错误）
✅ `cargo build --release` — 优化编译成功
✅ `cargo test` — **81 个单元测试全部通过**
  - telemetry 编解码：11 个测试
  - app_state 遥测/参数：8 个测试
  - 其它模块测试：62 个

## 字节级对齐验证
- ✅ encode_telem_start: mode(1B) + rate_hz(2B LE) + fields(1B) + ch_mask(8B LE) = 12B
- ✅ encode_param_set: ch(1B) + param_id(1B) + value(4B LE) = 6B
- ✅ decode_telem_data: 按字段顺序 RAW→BASELINE→DIFF→STATUS 解析
- ✅ decode_param_get_all: ch + count + [param_id + value LE]×count
- 所有编解码均验证 LE (小端)字节序

## 接入点 (供 #6g-2 Slint UI 使用)
- **遥测启动**: `ctrl.start_telemetry(rate_hz, fields, ch_mask)?`
- **遥测停止**: `ctrl.stop_telemetry()?`
- **读遥测数据**: `ctrl.telem_series(ch, FIELD_RAW/BASELINE/DIFF)` → Vec<f32> (绘制曲线)
- **读最新样本**: `ctrl.telem_latest(ch)` → Option<ChannelSample>
- **版本号**: `ctrl.telem_version()` (UI 判断何时重绘)
- **参数查询**: `ctrl.param(ch, param_id)` → Option<u32>
- **参数设置**: `ctrl.set_param(ch, param_id, value)?` (包括乐观更新)
- **校准/基线**: `ctrl.calibrate(ch_mask)?` / `ctrl.baseline_reset(ch_mask)?`

## 文件变更清单
- ✅ 新建 src/proto/telemetry.rs (411 行，含测试)
- ✅ 修改 src/proto/mod.rs (加 pub mod + re-export)
- ✅ 修改 src/app_state/mod.rs (加字段、方法、poll 扩展、测试)

## 约束遵循
- ✅ 不自造 Frame/HostCmd 接口，复用现有
- ✅ 私有字段默认，pub 暴露方法
- ✅ 不写 README 或独立测试文件
- ✅ 所有内联单元测试 #[cfg(test)]
- ✅ 严格 LE 字节序（对齐固件）
