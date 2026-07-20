# impl_6g2_curves_ui 任务状态(已完成)

## 目标
在 Rust 上位机 Slint UI 中实现 #6g-2 曲线页(挂 TabWidget"曲线" tab):实时波形 + 阈值交互 + 逐通道参数面板。

## 架构结论(复用,未重造)
- 遥测/参数协议已在 `src/proto/telemetry.rs` (#6g-1) 完成,`AppController`(`src/app_state/mod.rs`)已有
  `start_telemetry/stop_telemetry/telem_series/telem_latest/telem_version/param/params_of/set_param/request_params/calibrate/baseline_reset/param_version`。
- `main.rs` 既有 16ms Timer + version-gated 重建范式(config_version/bind_progress),曲线页照此扩展。
- Slint 无内置图表,采用 `Path` + SVG commands 字符串方案,viewbox 固定 1000x1000。

## 已完成实现
1. **`ui/app.slint`**:
   - 新增 `export struct ParamRow { param_id, label, value }`。
   - 新增 `component CurvesPage`:通道 SpinBox(0-35)、开始/停止/校准/基线复位按钮、RAW/BASELINE/DIFF CheckBox、
     手指阈值/噪声阈值 SpinBox(退化方案,非拖拽——见下)、绘图区(Rectangle+3条Path+2条阈值Rectangle横线+零线)、
     参数面板(GroupBox+ScrollView+for row SpinBox)。
   - `AppWindow` 顶层加了对应 in/in-out 属性(`sel_channel/show_raw/show_bsln/show_diff/diff_path/raw_path/bsln_path/
     finger_th_y/noise_th_y/finger_th_val/noise_th_val/curve_params`)和回调(`telem_start/telem_stop/curve_calibrate/
     curve_baseline_reset/threshold_set/curve_param_edited`),"曲线"tab 占位 Text 替换为 CurvesPage 实例。
   - **阈值交互选择:退化方案(SpinBox 直接编辑值 + 视觉线只读展示)**,回调 `threshold_set(param_id, value)`。
     未做拖拽手柄(原方案里的可选项),因 Slint Path/TouchArea 拖拽换算复杂度高、且 SpinBox 方案已满足"交互调节"需求。

2. **`src/main.rs`**:
   - `build_param_rows(ctrl, ch)`:遍历 `KNOWN_PARAM_IDS`,用 `param_label()` 中文名表 + `ctrl.param(ch,pid)` 取值(缺失0占位)。
   - `series_to_svg_path(series, min, max)` + `value_to_viewbox_y(value, min, max)`:序列→SVG path / 值→viewbox y,
     空序列或 min==max 返回空串/中线,避免除零。
   - `build_curve_paths(ctrl, ch, show_raw, show_bsln, show_diff)`:按 show_* 门控是否计算对应曲线 path(未勾选返回空串,
     避免无谓字符串构建);raw/baseline 域固定 [0,65535],diff 域固定 [-512,512];同时把 finger_th/noise_th 换算到
     diff 域的 viewbox y。
   - 回调装配:telem_start(200Hz, RAW|BASELINE|DIFF|STATUS, ch_mask=全通道u64::MAX)/telem_stop/calibrate/baseline_reset(全通道)/
     threshold_set→set_param(sel_channel, pid, value)/curve_param_edited→set_param(sel_channel, pid, value)。
   - Timer tick 扩展:读 `win.get_sel_channel()` 与上次比较,变化则 `request_params(ch)`;telem_version/param_version/
     channel_changed 任一变化才重建 curve paths / param_rows(version-gated,避免每 tick 无谓重算)。
   - 启动时自动 `request_params(0)`(打开曲线 tab 就有初始数据),不自动开流(需手动点"开始")。

3. **`src/proto/mod.rs`**:补充 pub use 导出 `FIELD_STATUS/KNOWN_PARAM_IDS/PARAM_*` 常量(main.rs 需要)。

4. **`src/app_state/mod.rs`**:补充两个 `#[cfg(test)]` 测试后门(与既有 `test_insert_entries` 同模式,不影响生产构建):
   - `test_push_telem_sample(ch, sample)`:直接注入遥测样本。
   - `test_set_param_cache(ch, param_id, value)`:直接注入参数缓存(因 `set_param` 仅在已连接时才落盘本地缓存,单测无真实连接)。

## 内联单测(main.rs #[cfg(test)] mod tests,新增 7 个)
- `test_build_param_rows_count_and_values`:行数=KNOWN_PARAM_IDS.len(),取值正确,未设置参数占位0。
- `test_series_to_svg_path_format_and_point_count`:以"M "开头,点数匹配(1个M+2个L)。
- `test_series_to_svg_path_empty_returns_empty`:空序列/min==max 均返回空串。
- `test_value_to_viewbox_y_roundtrip`:值↔viewbox_y 换算(含越界clamp)。
- `test_build_curve_paths_empty_data_returns_empty_strings`:无数据时三条曲线空串,阈值y在中线。
- `test_build_curve_paths_respects_show_flags`:未勾选的曲线即使有数据也不生成path。

## 验证结果
- `cargo build`:成功,仅有的 warning 均为 main_firmware/comport/io 模块既有的、与本次改动无关的 dead_code warning。
- `cargo test`:**87 passed; 0 failed**,exit code 0。未运行 GUI(遵守约束)。

## 状态:任务已完成,无待办。
