# rgb-status-pwm

## 任务目标
将 RP2040 GPIO18/19/20 共阳 RGB 状态灯切换为 PWM，并新增可配置亮度与状态颜色。

## 已完成步骤
1. 读取 LED 服务、应用配置和 `main.cpp` 的现有实现。
2. 核实 GPIO18=G、GPIO19=R 共用 PWM slice，GPIO20=B 使用另一 slice；LED 为 active-low。
3. 核实 ConfigManager 仅支持 BOOL/数值/字符串范围 schema，没有 enum/options 元数据，颜色注册为 U8 范围 0..7。
4. 在 `led_service` 中实现 PWM：wrap=255，GPIO18/19 只初始化并启用一次共享 slice，GPIO20 单独处理；重复 `init()` 直接返回，避免重置另一 channel。
5. 新增 `set_color(mask, brightness)` 和 8 位 `set_rgb(r,g,b)`；保留所有布尔兼容 API，并统一映射到 PWM duty。
6. 注册 `led.status_brightness` 及四个颜色配置；`main.cpp` 读取并限制配置，遵守 `led.enable`，保持主机连接优先级。
7. 运行 `powershell -ExecutionPolicy Bypass -File dev.ps1 build-rp`，构建成功（4.21s，RAM 13.4%，Flash 15.7%）。

## 关键决策
- PWM wrap=255，active-low 共阳输出 level=255-duty。
- 默认亮度=128，连接/健康绿色(2)，flash 错误红色(1)，链路错误蓝色(4)。
- 主机连接仍以 `millis() - g_last_host_cmd_ms < 2000u` 判定；非连接状态保留 flash 错误、链路错误、健康的原有优先级心跳。
- 项目现有 schema/协议没有 enum 或 options 字段，故不能向上位机附加枚举 options；按要求回退为 0..7 范围 U8，并在源码注明中英颜色映射。

## 待办步骤
无。

## 最终结果
已完成并通过 RP2040 构建验证。
