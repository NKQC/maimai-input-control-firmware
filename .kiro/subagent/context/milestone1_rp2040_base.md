# 里程碑1 RP2040 base 构造任务 - 进度记录

## 任务目标
从残留代码中构建精简单核 RP2040 里程碑1固件：config.h + led_service + sensor_link + usb_comm(骨架) + swd(骨架) + psoc_updater(骨架) + main.cpp 单核化，pio run 通过。

## 架构结论（已确认）
- config.h、led_service、sensor_link、usb_comm 已存在且内容基本符合规格（见下方"已完成"细节）。
- hal/*、protocol/{neopixel,mai2serial,hid} 本身接口无问题。
- **但 protocol/mai2light 和 service/config_manager 的 .cpp 文件里硬编码依赖了已删除的 usb_serial_logs / input_manager / light_manager / ui_manager 模块，这是本次编译报错的根因，不是"无残留"。**

## 已完成步骤
1. 确认 `src/service/ui_manager` 目录已被主 agent 彻底删除（list_directory 核实，目录已不存在）。
2. 核实 config.h 内容完全符合硬件权威定义（引脚/常量均正确），无需修改。
3. 核实 led_service、sensor_link、usb_comm 内容符合规格（单例、_write_pin 私有工具函数、SensorFrame 定长结构、20ms 节流 ping/pong、set_rgb 互斥同步），无需修改。
4. 新建 `src/driver/swd/swd.h` + `swd.cpp`：SwdProgrammer 类（非单例），构造存 io/clk/rst 三引脚，init()/update() 空实现。
5. 新建 `src/service/psoc_updater/psoc_updater.h` + `psoc_updater.cpp`：PsocUpdater 单例，init()/update() 空实现。
6. 重写 `src/main.cpp` 为单核精简版：setup() 做 global_irq_init + watchdog_enable(5000,true) + HAL_USB_Device::init + LedService::init + HAL_SPI1 传给 SensorLink::init；loop() 做 SensorLink::update + watchdog_update。已删除所有双核/CoreInitBitmap/ui_manager/input_manager/light_manager 相关代码。
7. 执行 `pio run`（cwd=main_firmware），发现两个致命编译错误：
   - `src/protocol/mai2light/mai2light.cpp:3` `#include "../../protocol/usb_serial_logs/usb_serial_logs.h"` 找不到文件（usb_serial_logs 目录已确认为空文件夹）。mai2light.cpp/h 内部 log_debug/log_error 私有方法调用 `USB_SerialLogs::get_global_instance()`，是仅用于日志的旁路依赖，不涉及协议核心解析逻辑，理论上可以摘除。
   - `src/service/config_manager/config_manager.cpp:2-4` 同时 `#include` 了 `../input_manager/input_manager.h`、`../light_manager/light_manager.h`、`../ui_manager/ui_manager.h`（三者目录均已不存在或为空）。**且这不只是 include 问题**：
     - `initialize_defaults()` 内部硬调用 `inputmanager_register_default_configs()`、`lightmanager_register_default_configs()`、`uimanager_register_default_configs()`。
     - `save_config_task()` 内部硬调用 `inputmanager_get_config_copy()`/`inputmanager_write_config_to_manager()`、`ui_manager_get_config_copy()`/`ui_manager_write_config_to_manager()`、`lightmanager_get_config_copy()`/`lightmanager_write_config_to_manager()`，类型 `InputManager_PrivateConfig`/`UIManager_PrivateConfig`/`LightManager_PrivateConfig` 均来自已删除模块。
     - 这是 config_manager 的**核心保存/加载流程**耦合了三个已删除模块的注册函数，不是简单删掉一行 include 就能解决的，需要决定是否摘除这些调用（改动 core 逻辑）还是本任务范围外。

## 待办
- 等待主 agent/用户决策：config_manager.cpp 的 initialize_defaults()/save_config_task() 中对 input_manager/light_manager/ui_manager 的调用如何处理（是否允许本次任务内摘除）。
- mai2light.cpp/h 的 usb_serial_logs 依赖（log_debug/log_error）如何处理：是否允许摘除（仅日志旁路，不动协议核心解析）。
- 决策落地后需要重新跑 `pio run` 验证。

## 关键决策/异常
**已触发"发现的偏差修正超出本任务范围"条款，已停止进一步修改 config_manager.cpp 核心逻辑，等待主 agent 决策。**

## 下一步动作
向主 agent 提交异常报告，说明 config_manager.cpp 与 mai2light.cpp 对已删除模块的深度耦合，请求明确是否授权移除这些耦合调用（不新造功能，只是摘除死引用）。
