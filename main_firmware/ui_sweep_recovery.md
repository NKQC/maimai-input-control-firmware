# UI 频谱扫描恢复处理要求

## 背景

固件将频谱扫描结果保持在设备内存中；扫描结束前会依次恢复原始 gain/div、执行单通道校准并复位基线。`SWEEP_DATA` 的终态才表示该恢复链已经结束。

本次固件修复保证：扫描或恢复期间收到 `HELLO` 不再取消扫描恢复链。设备仍会优先返回 `DEVICE_INFO`；扫描结束后不会恢复上一个主机遗留的 telemetry/focus 输出，等待当前 UI 主动开启新流。

## 必须调整的位置

### `control_software/src/app_state/ch_ops.rs`

涉及 `_handle_sweep_data()`、`_sweep_finish_session()`、断线处理以及扫描完成后的 probe/telemetry 调度。

1. 将同一 `session` 的终态 `SWEEP_DATA` 作为扫描可结束的唯一设备依据；`Restoring` 不是完成。
2. 收到 `Done` 或 `Cancelled` 且 `flags & 0x70 == 0` 时，才允许恢复常规 probe、`TELEM_START` 或 `FOCUS_START`。
3. 收到 `Failed`，或终态 `flags & 0x70 != 0` 时，必须保持当前流停止、将设备恢复状态标为未知，并禁止自动重试 `TELEM_START` / `FOCUS_START` / 频谱扫描。应要求用户明确恢复设备或等待后续健康检查成功。
4. 连接断开时，保留扫描结果供查看，但不得假定 PSoC 已恢复；重连后先完成 `HELLO`/`DEVICE_INFO`，不要用旧扫描状态直接启动流。
5. `SweepCtrl` keepalive 只在本地扫描会话仍有效且尚未收到终态时发送；终态或断链后停止发送。

### `control_software/src/proto/telemetry.rs`

已具备的协议语义需要被 UI 调度层完整使用：

- `SweepState::Failed`：设备无法确认恢复成功。
- `SWEEP_RESTORE_FLAG_PARAM = 0x10`：原 gain/div 写回或回读失败。
- `SWEEP_RESTORE_FLAG_CAL = 0x20`：恢复校准失败。
- `SWEEP_RESTORE_FLAG_BSLN = 0x40`：恢复基线复位失败。
- 终态尾部 `fail_phase`：首个故障阶段；`phase`：发帧时的状态机阶段。

`Failed` 或 `flags & 0x70 != 0` 必须视为不可自动恢复，而不是仅显示告警文本后继续发流命令。

## 建议的连接时序

1. 发送 `HELLO`，必须在 2 秒内等待 `DEVICE_INFO`。
2. 不因曾有扫描会话而立即发送 `TELEM_START` 或 `FOCUS_START`。
3. 若仍持有该扫描会话，等待其 `SWEEP_DATA` 终态；成功终态后才按当前页面需求重新启动流。
4. 对失败终态，仅提供诊断信息与显式用户操作入口；禁止形成自动重连或启动流循环。

## 兼容性

旧固件可能把恢复 flags 非零的终态仍标为 `Done` 或 `Cancelled`，因此 UI 保持 `flags & 0x70 != 0` 的独立失败判断。新固件会将此类终态统一上报为 `Failed`，同时保留 flags 与 `fail_phase` 便于诊断。

## 跳跃采样语义（固件当前实现）

扫描不再默认逐格遍历 448 个 gain/div 组合：

- `PROBE` 从索引 0 开始，每次按 10 格跳跃探测；探测点无效时继续下一个 `+10` 位置。
- 探测点有效后进入 `FILL_LEFT`，从该点左侧逐格采样；遇到第一个无效点或数组边界后停止左侧补采。
- 随后进入 `FILL_RIGHT`，从命中点右侧逐格采样；遇到第一个无效点或数组边界后停止右侧补采。
- 右侧无效边界之后继续按 10 格步长 `PROBE`，直到最后一个可探测位置；到达末端后仍会执行原 gain/div 写回、校准、基线复位恢复链。
- 结果保存在固件 RAM 的 `_sweep_results[448]`，并用 `resolved[448]` 标记已判定格。`produced` 是从索引 0 开始的连续已判定前缀长度，不是实际采样点最高水位。
- 已判定但跳过采样的空洞按真实 index 顺序发送 `CELL`，字段为 `samples=0`、`flags=0`、`mean/std_q8/pp=0`；已采样点发送真实聚合结果。发送游标只发送 `index=0..produced-1`，不会跳过空洞。
- `PROBE` 无效后，当前位置以前的未决区间才判为空并按顺序发布；有效后进入 `FILL_LEFT`，左侧首个无效边界确认时再判定更左侧未决空洞，然后补右侧。`FILL_RIGHT` 首个无效边界后继续以 `+10` 探测；最终恢复前剩余尾部统一判空，终态 `produced=448`。
- `SWEEP_CTRL` 补发允许 `0 <= index < produced` 的全部已判定格，包括空洞；仍只保存在 RAM，不触发 request_save。
