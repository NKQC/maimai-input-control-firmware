# I2C 模拟寄存器映射（TouchSensor）

本文档说明 I2C 从机寄存器的功能、读写大小、参数范围、权限以及地址组成和访问协议。文档位于 `main.c` 同级目录，供主机侧驱动/调试参考。

## I2C 从机地址映射

基于引脚P3.3和P3.2的状态，设备的I2C从机地址映射如下：

| P3.3 | P3.2 | 地址计算 | I2C地址 |
|------|------|----------|---------|
| 0    | 0    | 0x08 + 0x00 | 0x08 |
| 0    | 1    | 0x08 + 0x02 | 0x0A |
| 1    | 0    | 0x08 + 0x04 | 0x0C |
| 1    | 1    | 0x08 + 0x06 | 0x0E |

## 寄存器表
| 地址 | 名称 | 访问权限 | 大小 | 参数范围 | 功能 | 备注/默认值 |
| :---: | :--- | :---: | :---: | :---: | :--- | :--- |
| `0x00` | `SCAN_RATE` | R | 2 | `0..65535` | 当前每秒扫描次数（平均/即时） | 由固件在运行时计算与更新 |
| `0x01` | `TOUCH_STATUS` | R | 2 | 位图 | 触摸状态位图 | `bit[0..11]` 对应 `CAP0..CAPB`；位为 `1` 表示触摸，`0` 表示未触摸；高于 11 位未使用 |
| `0x02` | `CONTROL` | R/W | 2 | `bit0`=复位, `bit1`=LED, `bit2`=校准请求, `bit3`=校准完成(只读), `bit4`=绝对模式(默认0), `bit5`=LED触摸提示使能(默认1) | 复位/LED/校准/模式控制 | `bit0=1` 触发软件复位；`bit1=1` 点亮；`bit1=0` 熄灭；`bit2=1` 异步请求校准（仅在CapSense空闲时执行）；校准进行时 `bit3=0`，完成后置 `1`（包含上电初次校准）；`bit4=1` 触摸电容设置寄存器按绝对值工作；`bit4=0` 为相对（增量）模式；`bit5=1` 主循环自动按触摸状态更新LED；`bit5=0` 关闭自动提示（保留手动LED控制）；其它位保留且忽略；读返回当前状态（重启后寄存器恢复默认） |
| `0x03` | `CAP0_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP0 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x04` | `CAP1_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP1 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x05` | `CAP2_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP2 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x06` | `CAP3_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP3 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x07` | `CAP4_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP4 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x08` | `CAP5_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP5 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x09` | `CAP6_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP6 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x0A` | `CAP7_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP7 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x0B` | `CAP8_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP8 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x0C` | `CAP9_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAP9 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x0D` | `CAPA_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAPA 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x0E` | `CAPB_TOUCH_CAP_SETTING` | R/W | 2 | `MIN..MAX_STEPS` | CAPB 触摸电容设置（增量/绝对，单位 0.01 pF 步进） | 默认 `100`（1.00 pF） |
| `0x0F` | `CAP0_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP0 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x10` | `CAP1_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP1 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x11` | `CAP2_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP2 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x12` | `CAP3_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP3 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x13` | `CAP4_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP4 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x14` | `CAP5_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP5 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x15` | `CAP6_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP6 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x16` | `CAP7_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP7 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x17` | `CAP8_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP8 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x18` | `CAP9_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAP9 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x19` | `CAPA_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAPA 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |
| `0x1A` | `CAPB_TOTAL_TOUCH_CAP` | R | 2 | `0..2200` | CAPB 总触摸电容（Cp 基数 + 设置增量，单位 0.01 pF） | 只读；FULL 模式返回 Cp 基数，非 FULL 模式返回 Cp+增量；总和不超过 22.00 pF |

- 触摸灵敏度寄存器采用零点偏置编码：`TOUCH_SENSITIVITY_ZERO_BIAS=4095`。
- 原始值范围：`0..8191`（`TOUCH_SENSITIVITY_RAW_MIN..TOUCH_SENSITIVITY_RAW_MAX`）；`<4096` 为负偏移，`>4095` 为正偏移。
- 写入：主机写原始值；固件转换为有符号步进偏移，正向偏移若小于 `TOUCH_INCREMENT_MIN_STEPS` 将按宏修正；偏移幅度限制为 `±TOUCH_SENSITIVITY_MAX_STEPS`。
- 读取：返回原始值，即 `ZERO_BIAS + signed_steps`。
- 主机侧 UI 灵敏度映射：`sensitivity(0..99)` 以 `49` 为零偏；写入原始值 `raw = 4095 + (sensitivity - 49) × 10`；每 1 位对应 10 步进（0.10 pF）；降低灵敏度为负向偏移，上调为正向偏移。
- 换算关系：`steps = raw - ZERO_BIAS`；`pF = steps × 0.01`；总触摸电容为只读，且 Cp+增量不超过 `22.00 pF`（`2200` 步进）。
- 最低设置值：非 `FULL` 模式可为 `0`；`FULL` 模式最低为 `10`（0.10 pF）。

### 绝对模式（CONTROL.bit4=1）
- 写入 `CAPx_TOUCH_CAP_SETTING`：值表示总触摸电容的绝对步进（单位 0.01 pF）。固件将进行边界检查（`0..TOUCH_CAP_TOTAL_MAX_STEPS`），再以 `Cp基数` 为参考换算为增量并应用（正向增量若小于 `TOUCH_INCREMENT_MIN_STEPS` 会提升到最小步进）。
- 读取 `CAPx_TOUCH_CAP_SETTING`：返回当前总触摸电容的步进值（与只读 `CAPx_TOTAL_TOUCH_CAP` 一致），便于根据绝对参考随时调整。
- 切换参考：可随时写入新绝对值；固件会相对当前 `Cp基数` 计算增量并夹取到允许范围。

### 相对模式（CONTROL.bit4=0，默认）
- 写入 `CAPx_TOUCH_CAP_SETTING`：值为原始编码（`0..8191`），固件转换为有符号步进并应用。
- 读取 `CAPx_TOUCH_CAP_SETTING`：返回原始编码（`ZERO_BIAS + signed_steps`）。

### LED触摸提示（CONTROL.bit5）
- `bit5=1`（默认）：主循环每次处理触摸状态时会自动按 `TOUCH_STATUS` 位图更新LED显示，有触摸则点亮，无触摸则熄灭。
- `bit5=0`：关闭自动提示，LED不再由触摸状态驱动；仍可通过 `CONTROL.bit1` 手动控制LED开关。

## 参考资料
- AN85951 – PSoC 4/6 MCU CapSense 设计指南（SNR ≥ 5：信号/噪声定义与调优原则）：
  - https://www.infineon.com/AN85951 或 https://documentation.infineon.com/psoc6/docs/epf1667481159393
- Infineon 社区：CapSense Tuning and Signal-to-Noise Ratio（示例：ΔRaw=135、噪声=26、SNR=5.2；最小建议 5:1）：
  - https://community.infineon.com/t5/CAPSENSE-MagSense/CapSense-Capacitive-Sensing-Tuning-and-Signal-to-Noise-Ratio/td-p/241650
- Infineon 社区：RawCount/IDAC/Cp 与等式 3-6 讨论（Direct 时钟、IDAC 建议等）：
  - https://community.infineon.com/t5/PSoC-4/Regarding-RawCount-IDAC-and-CP-of-Capsense/td-p/235514
- AN66271 – 旧器件族 CapSense 设计指南（初始 Finger Threshold 与 SNR≥5 调优流程）：
  - https://www.infineon.com/assets/row/public/documents/cross-divisions/42/infineon-an66271-cy8c21x34-b-capsense-design-guide-applicationnotes-en.pdf?fileId=8ac78c8c7cdc391c017d0723006845ca
