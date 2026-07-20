# capsense-cp-tuning-ui

## 任务目标
实现 CP_MEASURE/CP_GET 固件与上位机协议、36 通道 Cp 缓存与测量轮询 UI，并统一参数 ID 为 0x01..0x0B；最终运行固定 build-rp 与 build-ui。

## 已完成步骤
1. 已读取恢复上下文、工作区差异、固定 `dev.ps1`；验证入口为 `powershell -ExecutionPolicy Bypass -File dev.ps1 build-rp` 与 `build-ui`。
2. 已逐项核验固件：HostCmd 为 CP_MEASURE=0x27 / CP_GET=0x28；SensorLink 已注册严格空/单字节校验、`Psoc::measure_cp/get_cp` 调用、ACK/NAK 和 ch+u32LE 响应；`kParamIds` 为 0x01..0x0B。
3. 已核验 Rust proto：HostCmd/TryFrom、CP 编解码、导出及 `KNOWN_PARAM_IDS` 都已正确；`decode_cp_get` 规定严格 5 字节。
4. 已完成 AppController：36 通道 `Vec<Option<u32>>` Cp 缓存与每通道响应版本、初始化、显式/异步断开清空并递增版本、CP_GET 响应分发/解码、`measure_cp` / `request_cp` / `cp` / `cp_version` / `set_param_all`。
5. 已完成 Slint 局部 UI：CurvesPage 增加 Cp 状态、严格文本“测量全部电极电容”、每行“此项应用全部通道”，并完成内部 SettingsPage 透传。既有 6x6 网格、binding_text、通道卡跳转、曲线标尺未改动。
6. main.rs 已加入 Cp 轮询状态结构及标准时间依赖，待绑定回调和 Timer 状态机。

## 待办步骤
1. 完成 AppWindow 顶层 Slint 回调/属性透传。
2. 实现 main.rs 500ms/10s Cp Timer 状态机与通道切换的 params+Cp 请求。
3. 运行固定 build-rp、build-ui，依据结果修复。

## 关键结论/决策
- CP_GET 请求 payload 是 ch(u8)，响应是 ch+cp(u32 LE)。
- 成功值为非 0、非 `0x00FFFFFF`；UI 通过每通道响应版本在切换时隐藏旧缓存，并且测量仅接受实际 CP_GET 请求后的新响应。
- 固件已有 SmartSense/半自动注释与当前模式一致；此前搜索 glob 不匹配，需在收尾用无 glob 限制的纯本地检索完成必要注释核验。
- 工作树已有大量与本任务无关的未提交改动及构建产物，必须避免干扰。

## 下一步动作
添加 Slint 顶层透传，绑定 main.rs 回调和轮询后执行固定构建。
