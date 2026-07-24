# Phase B(A方案): 算法上报/可调变量 + 触发追踪 + 算法页右子标签 — 任务契约

## 已锁定协议(PSoC 侧已实现+编译+embed, 勿改 PSoC)
- ABI `psoc_firmware/CY8C4147AZI-SensorCore/psoc_algo_abi.h`:
  - `algo_io_t` 新增 `uint16_t report[4] @0x66`(算法输出, 上报值), 既有 `uint8_t cfg[8] @0x18`(共享输入, 可设置变量), `out_active @0x60`(触发)。
  - 源码命名宏(上位机 grep 源码取 schema, 宏体为空): `ALGO_REPORT(idx, "name")`、`ALGO_SETTING(idx, "name", defval)`。
- PSoC SPI 7字节帧 `[magic,cmd,b2,b3,b4,b5,b6]`, 响应值在 b4-b5(u16 LE), b6=0:
  - `ALGO_GET_TRACE=0x46`: 请求`[.,.,ch,idx]` → 响应`[.,0x46,ch,out_active(b3=0/1),report[idx]_lo,report[idx]_hi,0]`。已在 main.c 实现: cmd_algo_get_trace。
  - `ALGO_SET_CFG=0x47`: 请求`[.,.,idx,val]` → 响应回显 `[.,0x47,idx,0,cfg[idx],0,0]`。已实现: cmd_algo_set_cfg。
  - `ALGO_GET_CFG=0x48`: 请求`[.,.,idx]` → 响应`[.,0x48,idx,0,cfg[idx](b4),0,0]`。已实现: cmd_algo_get_cfg。
  - main.c switch(rx[1]) 已含 case ALGO_GET_TRACE/ALGO_SET_CFG/ALGO_GET_CFG, 已验证不需改动 PSoC。
- PSoC 每次算法执行前把 `g_algo_cfg[8]` 拷入 `io->cfg`(main.c 已有 g_algo_cfg 静态数组); `g_algo_io[ch].report[]`/`out_active` 供 GET_TRACE 读。

## 架构勘察结论(镜像基准, 已读源码确认)
- RP2040 三层中继镜像基准 = set_algo_rom/get_algo_rom 三层:
  - `PsocSpi::set_algo_rom/get_algo_rom`(psoc_spi.h/.cpp): 用 `_cmd_txn((uint8_t)psoc::Cmd::X, b2, b3, val24, resp)`, 帧 `[magic,cmd,b2,b3,val24_lo,val24_mid,val24_hi]`。
  - `psoc::Cmd` 枚举在 `psoc_types.h`, 需新增 `ALGO_GET_TRACE=0x46, ALGO_SET_CFG=0x47, ALGO_GET_CFG=0x48`(与 psoc_algo_abi.h 宏值对齐)。
  - `Psoc` 门面(psoc.h/.cpp): `SpiOp` enum 加三项, `_exec_cmd` switch 加分派, `_submit()` 提交到 core1 命令环. 公开方法签名参照契约: `algo_get_trace(ch,idx,uint8_t*out_active,uint16_t*report)`, `algo_set_cfg(idx,val)`(阻塞回显校验), `algo_get_cfg(idx,uint8_t*val)`。
  - `SensorLink`(sensor_link.h/.cpp): 新 HostCmd handler, 用现成 `_handle_algo_set_rom`/`_handle_algo_get_rom` 作为编解码镜像模板。
  - `host_cmd.h`: `HostCmd::ALGO_GET_TRACE=0x69, ALGO_SET_CFG=0x6A, ALGO_GET_CFG=0x6B`(算法域 0x60-0x6F, 0x68 已被 ALGO_GET_CODE 占用, 0x69-0x6B 空闲)。
  - `PsocAlgo`(psoc_algo.h/.cpp): 加 `uint8_t _cfg[8]` 持久化(仿 `_rom[36]`/AlgoBlob 结构体, 需加 cfg 字段+crc32 覆盖范围内); `download_to_psoc` 里 for idx 0..8 调 `psoc->set_algo_cfg(idx,_cfg[idx])`(镜像现有 rom 推送 for 循环)。
- 上位机 proto 镜像基准 = `proto/algo.rs` 里 `encode_algo_set_rom/encode_algo_get_rom` + `HostCmd` 在 `proto/mod.rs` 的 enum+TryFrom 双处对齐。
- app_state(`app_state/mod.rs`)镜像基准 = `cp: Vec<Option<u32>>`+`cp_channel_versions`+`cp_version`(Cp 单值轮询模式) 与 `telem_buf: Vec<VecDeque<ChannelSample>>`(时间序列模式, TELEM_CAP=1024)。
  - 算法上报值(4个 report + out_active)需要"选中通道"的时间序列, 采用类似 telem_buf 但更小容量的 VecDeque<f32> ×4 + VecDeque<bool>(out_active), 只为当前选中通道维护(不像 telem 36 通道全存, 因为 GET_TRACE 是逐条轮询, 高频轮询 36 通道×4 idx 开销大)。
  - schema 解析: `algo_source()`(已有, 当前编辑器 C 源) 正则找 `ALGO_REPORT(idx, "name")` / `ALGO_SETTING(idx, "name", defval)` 抽取 (idx,name)/(idx,name,default) 列表。Cargo.toml 无 regex crate, 用手写字符串扫描(避免加新依赖, 契约要求不新增未批准依赖)。
- main.rs 镜像基准 = `request_cp` 轮询(cp_poll_timer 状态机) 和 `build_curve_paths`(telem_series → SVG path)。选中通道每帧(~30Hz)对已声明的 report idx 逐个 request_algo_trace(ch, idx); 响应异步入 app_state 缓冲; main.rs 每 tick 读缓冲建 path 回填 UI(仿 diff_path/raw_path/bsln_path 三线 + 新增"触发判定"线, 复用 CurvePaths 结构或新建小型等价物)。
- UI(`app.slint`)镜像基准: AlgoPage 现是单页(顶部工具栏+C源/ASM子标签的编辑面板); SettingsPage 用 `TabWidget` 承载多个 XxxPage 组件, 回调/属性经 root 转发(sample_rate_hz 单层直传, g_idac_sense_config 单层直传)。算法页右侧子标签结构参照 AlgoPage 内部已有的 "editor_tab"(0/1按钮高亮模式)自造三态子标签: 0=原生(现有C源/ASM编辑面板原样保留) 1=算法上报(新: report[] 折线+触发追踪折线) 2=可调变量(新: 按 schema 显示 SpinBox, on edited 调 set_algo_cfg)。CurvesPage(单通道精调页)加一条"触发判定"折线(out_active 0/1 阶梯线, 复用 diff_path 同类 Path 机制)。

## 待实现(RP2040 + 上位机 + UI + 默认算法示例) — 7 步序号与执行状态见"进度"
1. RP2040 中继: psoc_types.h Cmd枚举 + psoc_spi.h/.cpp + psoc.h/.cpp(SpiOp+_exec_cmd+公开方法) + sensor_link.h/.cpp(handler+register) + host_cmd.h(HostCmd枚举) + psoc_algo.h/.cpp(_cfg[8]持久化+下发)。
2. 上位机 proto: proto/mod.rs(HostCmd枚举+TryFrom两处) + proto/algo.rs(encode_algo_get_trace/set_cfg/get_cfg + decode + schema解析函数)。
3. app_state/mod.rs: algo_cfg[8]缓存+版本, 选中通道 report/out_active 时间序列缓冲, request_algo_trace/set_algo_cfg/getters, handle_frame 分派新增三个响应, schema getter(从 algo_source() 解析)。
4. main.rs: 选中通道~30Hz轮询已声明report idx的request_algo_trace; 构建report折线path+触发追踪折线path; cfg可调项回填/下发(SpinBox edited回调)。
5. app.slint: AlgoPage 右侧三子标签重构(原生/算法上报/可调变量); CurvesPage 加"触发判定"追踪折线。
6. psoc_firmware/algo/psoc_algo_default.c: 加 ALGO_REPORT(0,"diff")+ALGO_SETTING(0,"bsln_offset",0)示例; io->report[0]=io->diff; cfg[0]作基线偏移接入判定逻辑; 重编 dev.ps1 build-blob。
7. 构建验证: dev.ps1 build-rp / build-ui 均须过, 贴关键输出; 不 reflash。

## 规则
- 镜像现有模式, 不臆造接口; 不确定的函数签名先 grep 确认。同一操作失败>3次即停并报告。每约5步更新本文件"进度"。不递归调用子agent。
- 编码注意: 工程内很多注释是 GBK, str_replace 匹配含中文注释的行易失败, 优先匹配纯 ASCII 代码子串。

## 进度
- [x] 架构勘察完成(读: psoc_algo_abi.h, psoc_algo_default.c, psoc_spi.h/.cpp, psoc.h/.cpp, psoc_types.h, host_cmd.h,
      sensor_link.h/.cpp, psoc_algo.h/.cpp, proto/mod.rs, proto/algo.rs, app_state/mod.rs关键区域, main.rs关键区域,
      app.slint AlgoPage/CurvesPage/SettingsPage, main.c 确认 GET_TRACE/SET_CFG/GET_CFG 三个 case 已实现, dev.ps1)。
- [x] 步骤1 RP2040中继: 完成。改动文件: psoc_types.h(Cmd枚举+3), host_cmd.h(HostCmd枚举+3),
      psoc_spi.h/.cpp(algo_get_trace/algo_set_cfg/algo_get_cfg), psoc.h/.cpp(SpiOp+3, _exec_cmd分派+3,
      公开方法+3), psoc_algo.h/.cpp(_cfg[8]持久化+set_cfg/cfg getter+AlgoBlob.cfg字段+download_to_psoc
      推送+save()持久化), sensor_link.h/.cpp(_handle_algo_get_trace/set_cfg/get_cfg + register_handler)。
      `dev.ps1 build-rp` PASS (Flash 497204B/15.8%, RAM 35176B/13.4%, 4.56s)。
- [x] 步骤2 上位机proto: 完成。proto/mod.rs(HostCmd枚举+TryFrom两处 AlgoGetTrace/SetCfg/GetCfg=0x69/6A/6B)。
      proto/algo.rs 新增: encode/decode_algo_get_trace, encode_algo_set_cfg/get_cfg+decode_algo_get_cfg,
      AlgoReportDecl/AlgoSettingDecl + parse_algo_reports/parse_algo_settings(手写扫描, 未引入regex依赖,
      _scan_macro_calls/_parse_paren_args/_unquote 支持顶层逗号切分, 忽略字符串/嵌套括号内逗号)。
- [x] 步骤3 app_state: 完成。新增字段: algo_trace_channel/pending_idx, algo_trace_report[4]:VecDeque<f32>,
      algo_trace_active:VecDeque<f32>, algo_trace_version, algo_cfg[8], algo_cfg_version。
      新增方法: request_algo_trace(通道切换清缓冲)/algo_trace_report_series/algo_trace_active_series/
      algo_trace_version, set_algo_cfg(立即下发+本地缓存)/request_algo_cfg/algo_cfg/algo_cfg_version,
      algo_report_decls/algo_setting_decls(调用 proto::algo::parse_*)。handle_frame 新增
      AlgoGetTrace/AlgoGetCfg 响应分派 → _handle_algo_get_trace_response/_handle_algo_get_cfg_response。
      _clear_sensor_caches 新增清空 trace 缓冲。`cargo build --locked --bins` PASS(via dev.ps1 build-ui)。
- [x] 步骤5 UI(app.slint): 完成(先于步骤4做, 因需先定稿UI属性名再让main.rs回填)。新增
      AlgoReportLine/AlgoSettingRow结构体+AlgoSettingEditorRow组件; AlgoPage重构为content_tab
      三态(0原生/1算法上报: report折线图例+触发判定折线/2可调变量: for循环SpinBox编辑, 用
      algo_setting_edited(idx,v)回调); CurvesPage新增show_active复选框+触发判定追踪窄带
      (独立Rectangle, 主plot高度让出44px)。SettingsPage/AppWindow两层转发链均已补全属性与回调
      (algo_sel_channel/algo_report_lines/algo_active_path/algo_trace_point_count/
      algo_setting_rows/algo_setting_edited, CurvesPage的active_path/active_point_count/
      show_active)。`dev.ps1 build-ui` PASS(20.18s)。
- [x] 步骤4 main.rs: 完成。新增状态变量: last_algo_trace_version/last_algo_cfg_version/
      last_algo_source_for_schema/algo_report_idxs/algo_trace_rr。connect边沿新增 for idx 0..8
      request_algo_cfg。tick循环新增: 每tick(~16ms)按schema轮询已声明report idx的
      request_algo_trace(sel_channel); version门控回填 algo_report_lines/algo_active_path/
      algo_trace_point_count(AlgoPage) + active_path/active_point_count(CurvesPage同一份数据,
      两处消费同一 out_active 序列)/algo_setting_rows。新增回调 on_algo_setting_edited→
      set_algo_cfg。新增 build_report_path 辅助函数(单序列自适应量程, 镜像 build_curve_paths
      量程算法)。`cargo build --locked --bins`(via dev.ps1 build-ui) PASS。
- [x] 步骤6 默认算法示例+build-blob: 完成。psoc_algo_default.c 新增 ALGO_REPORT(0,"diff") +
      ALGO_SETTING(4,"bsln_offset",0)(idx=4 避开已被"rise/drop permille override"功能占用的
      cfg[0..3]); algo()入口无条件 io->report[0]=io->diff(含inactive态, 折线不断线); median3
      滤波后 d += (int32_t)(int8_t)io->cfg[4](有符号字节偏移, 默认0中性不改变行为)。
      `dev.ps1 build-blob` PASS: nm仅 "00000000 T algo" 无未定义符号, objdump -dr 无
      R_ARM_*重定位、algo在偏移0, blob len=468B crc16=0x8E09(<1024限制), 已写
      main_firmware/src/service/psoc_algo/psoc_algo_default.h。
- [x] 步骤7 build-rp/build-ui验证: 完成。`dev.ps1 build-rp` PASS(Flash 497236B/15.8%,
      RAM 35176B/13.4%, 4.11s)。`dev.ps1 build-ui` PASS(5.95s)。未 reflash。
