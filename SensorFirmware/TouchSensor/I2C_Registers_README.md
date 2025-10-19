# I2C 模拟寄存器映射（TouchSensor）

本文档说明 I2C 从机寄存器的功能、读写大小、参数范围、权限以及地址组成和访问协议。文档位于 `main.c` 同级目录，供主机侧驱动/调试参考。

## 总览
- 从机地址：`0x08` 基地址 + 地址选择引脚 `P3.3 / P3.2` 组合。
- 地址位规则：引脚上拉，低电平表示选择位为 1。
  - `P3.3=1, P3.2=1` → 地址 `0x0B`
  - `P3.3=1, P3.2=0` → 地址 `0x0A`
  - `P3.3=0, P3.2=1` → 地址 `0x09`
  - `P3.3=0, P3.2=0` → 地址 `0x08`
- 数据大小：所有寄存器值为 16 位（2 字节）。
- 字节序：高字节在前（MSB→LSB）。
- 缓冲区：从机内部缓冲区大小 `16` 字节（不影响寄存器宽度）。

## 访问协议
- 读流程：主机先写 1 字节寄存器地址，然后执行读操作，连续读取 2 字节（高字节在前）。
- 写流程：主机一次性写入 3 字节：`[寄存器地址][高字节][低字节]`。
- 未定义寄存器读返回 `0x0000`；对只读寄存器的写入被忽略。

## 寄存器表
| 地址 | 名称 | 访问权限 | 大小 | 参数范围 | 功能 | 备注/默认值 |
| :---: | :--- | :---: | :---: | :---: | :--- | :--- |
| `0x00` | `SCAN_RATE` | R | 2 | `0..65535` | 当前每秒扫描次数（平均/即时） | 由固件在运行时计算与更新 |
| `0x01` | `TOUCH_STATUS` | R | 2 | 位图 | 触摸状态位图 | `bit[0..11]` 对应 `CAP0..CAPB`；位为 `1` 表示触摸，`0` 表示未触摸；高于 11 位未使用 |
| `0x02` | `LED_CONTROL` | R/W | 2 | `bit0={0,1}` 其余保留 | LED 控制 | `bit0=1` 点亮；`bit0=0` 熄灭；其余位保留且忽略；读可返回最后写入值 |
| `0x03` | `CAP0_THRESHOLD` | R/W | 2 | `0..65535` | CAP0 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x04` | `CAP1_THRESHOLD` | R/W | 2 | `0..65535` | CAP1 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x05` | `CAP2_THRESHOLD` | R/W | 2 | `0..65535` | CAP2 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x06` | `CAP3_THRESHOLD` | R/W | 2 | `0..65535` | CAP3 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x07` | `CAP4_THRESHOLD` | R/W | 2 | `0..65535` | CAP4 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x08` | `CAP5_THRESHOLD` | R/W | 2 | `0..65535` | CAP5 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x09` | `CAP6_THRESHOLD` | R/W | 2 | `0..65535` | CAP6 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x0A` | `CAP7_THRESHOLD` | R/W | 2 | `0..65535` | CAP7 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x0B` | `CAP8_THRESHOLD` | R/W | 2 | `0..65535` | CAP8 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x0C` | `CAP9_THRESHOLD` | R/W | 2 | `0..65535` | CAP9 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x0D` | `CAPA_THRESHOLD` | R/W | 2 | `0..65535` | CAPA 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |
| `0x0E` | `CAPB_THRESHOLD` | R/W | 2 | `0..65535` | CAPB 触发阈值 | 默认 `100`；写入后在下一次处理循环应用 |

## 说明与约束
- `TOUCH_STATUS` 位图更新在每次扫描完成后进行，读取得到的是最近一次的状态。
- 阈值更新流程：写阈值寄存器 → 置位内部标志 → 在主循环中批量应用到各控件。
- `LED_CONTROL` 仅使用低位 `bit0` 控制 LED；其它位保留，写入不会影响行为。
- 访问时序和大小由中断服务例程处理：读在地址匹配后填充 2 字节；写至少需要 3 字节，否则忽略。

---

## CapSense 阈值单位、范围与电容换算

### 阈值单位（Raw Count 差值）
- `CAPx_THRESHOLD` 的单位为原始计数差值：`ΔRaw = Raw_touch_avg − Raw_no_touch_avg`。
- 差值计数不是电容（pF），其大小受分辨率、感应时钟（Fsw）、IDAC、覆盖材料与电极结构共同影响。
- 设计指南将触摸信号（Signal）定义为平均原始计数增量，将噪声（Noise）定义为无触摸时原始计数峰峰值；推荐最小 `SNR ≥ 5:1`［AN85951；Infineon 社区 SNR 文章］。

### 推荐阈值范围（按噪声与 SNR 设定）
- 测量无触摸噪声峰峰值 `Noise_pkpk`（常见 20–50 counts）。
- 建议触发阈值：`FingerThreshold ≈ 3–5 × Noise_pkpk`，并验证 `SNR = ΔRaw/Noise_pkpk ≥ 5`。
- 经验区间（按钮场景）：
  - 低噪声/薄覆盖：`50–150 counts`
  - 常规覆盖（2–3 mm）与一般布线：`100–300 counts`
  - 厚覆盖/强干扰：`300–800 counts`（结合滤波与参数调优）
- 说明：旧版应用笔记常以 `≈96 counts` 作为初始指示阈值，实际项目以现场噪声与 SNR 优先［AN66271］。

### 与电容（pF）的近似换算方法（现场标定）
原始计数与电容并非固定线性；推荐通过现场标定得到“每 pF 的计数增量”，再据此将阈值换算为等效电容增量。

1) 已知小电容注入法（计数/每 pF 标定）
- 将感应时钟源设为 `Direct`（非 PRS/SSC），保持稳定 `Fsw`［社区答复］。
- 记录 `ΔRaw`（触摸平均 − 无触摸平均）。
- 在电极与地并联已知小电容 `Ctest`（如 0.1–0.5 pF），测得计数变化 `ΔRaw_test`。
- 得到比例：`k ≈ ΔRaw_test / Ctest（counts/pF）`，换算 `ΔC_finger ≈ ΔRaw / k`。

2) 设备自检 API 法（测 `Cp` 并配合 SNR）
- 启用自检库后，调用 `CapSense_GetSensorCapacitance()` 或新版 `Cy_CapSense_MeasureCapacitanceSensor()` 获取传感器寄生电容 `Cp`（量程视器件而定，常见约 5–255 pF）。
- 在既定参数（IDAC、Fsw、分辨率）下，确保无触摸计数不饱和且 `SNR ≥ 5`，结合注入法或历史经验建立计数—电容比例用于估算阈值对应 `ΔC`。

提示与约束：
- `PRS/SSC` 时钟会导致 `Fsw` 变量化，公式参数不适用；标定/换算请改为 `Direct`［社区答复引用 3-6 式背景］。
- 建议 IDAC 代码大于约 `18` 以获得更好灵敏度/稳定度；必要时提升 `Fsw` 或调整分辨率［社区建议］。
- SmartSense（自动调优）在较高 `Cp` 下仍可获得可靠信号，按钮的“指尖等效电容增量”经验量级约 `0.1–0.2 pF`（随覆盖/结构变化）［社区 SNR 文章对能力说明］。

### 快速现场流程
- 固定参数（`Sense Clock = Direct`，分辨率与 IDAC 模式）。
- 测噪声 `Noise_pkpk`（无触摸）。
- 测信号 `ΔRaw`（触摸），验算 `SNR ≥ 5`。
- 设阈值 `FingerThreshold ≈ 3–5 × Noise_pkpk`，必要时增加滞回避免抖动。
- 如需电容阈值：用注入法标定 `k（counts/pF）`，得到 `ΔC_finger ≈ ΔRaw/k`。

## 参考资料
- AN85951 – PSoC 4/6 MCU CapSense 设计指南（SNR ≥ 5：信号/噪声定义与调优原则）：
  - https://www.infineon.com/AN85951 或 https://documentation.infineon.com/psoc6/docs/epf1667481159393
- Infineon 社区：CapSense Tuning and Signal-to-Noise Ratio（示例：ΔRaw=135、噪声=26、SNR=5.2；最小建议 5:1）：
  - https://community.infineon.com/t5/CAPSENSE-MagSense/CapSense-Capacitive-Sensing-Tuning-and-Signal-to-Noise-Ratio/td-p/241650
- Infineon 社区：RawCount/IDAC/Cp 与等式 3-6 讨论（Direct 时钟、IDAC 建议等）：
  - https://community.infineon.com/t5/PSoC-4/Regarding-RawCount-IDAC-and-CP-of-Capsense/td-p/235514
- AN66271 – 旧器件族 CapSense 设计指南（初始 Finger Threshold 与 SNR≥5 调优流程）：
  - https://www.infineon.com/assets/row/public/documents/cross-divisions/42/infineon-an66271-cy8c21x34-b-capsense-design-guide-applicationnotes-en.pdf?fileId=8ac78c8c7cdc391c017d0723006845ca

## 代码关联
- I2C：`module/i2c/i2c_module.c/h`（寄存器处理、中断、缓冲配置）。
- CapSense：`module/capsense/capsense_module.c/h`（扫描与状态位图、阈值数组 `g_cap_thresholds[]`）。
- LED：`module/led/led_module.c/h`（`REG_LED_CONTROL` 写入时调用 `led_set_state()`）。
- 地址计算：`main.c::_get_i2c_address()`。