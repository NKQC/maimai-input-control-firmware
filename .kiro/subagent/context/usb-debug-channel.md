# 任务: usb-debug-channel

## 目标
新增经 EP0 vendor 控制请求(bRequest=0x50)读取的设备调试计数器通道。
因为 flash 写会破坏 bulk vendor 端点(host->device 永久失步)，但 EP0/枚举仍存活，
故用控制传输读取设备侧真相，判定 flash 写后：主循环是否存活 / vendor OUT 是否仍 arm /
rx 回调是否触发 / 是否重新武装过。用户要求“新增 debug 指令 设置才开启”，故附带 0x51=SET_DEBUG 开关(默认关)。

## 状态
- [x] 新建 usb_debug.h
- [ ] hal_usb.cpp 埋点 + 控制请求处理 + 全局定义
- [x] main.cpp loop_count++
- [x] config_manager.cpp / csd_config.cpp flash 计数
- [x] io/mod.rs read_debug()
- [x] selftest.rs --debug-read
- [x] dev.ps1 dbg action
- [x] build-rp + build-ui 均通过 (firmware SUCCESS, cargo Finished)
- 修正了 subagent 与主 agent 重复应用导致的 hal_usb.cpp 重复 include/全局定义
- 待烧录设备验证(需手动 BOOTSEL，因当前固件 vendor 已死)

## 本轮进度（实现中）
- 已严格按规格完成非 `hal_usb.cpp` 的文件改动；`config_manager.cpp` 仅在 `save_config_task()` 的成功日志后、返回前计数，`csd_config.cpp` 仅在 PICO 分支完成写入/恢复中断后计数。
- Rust `read_debug()` 使用现有 nusb `MaybeFuture::wait()`、ControlIn vendor/device 请求，`selftest --debug-read` 在创建 AppController 前直接读取并退出。
- 待完成：`hal_usb.cpp` 的全局计数器、EP0 0x50/0x51、bulk 收发/重武装及 task 埋点；随后执行两项强制构建。

## 验证
1. dev.ps1 dbg (fresh) 读到计数器
2. dev.ps1 dbg 两次间隔，loop_count 递增 = 主循环活
3. dev.ps1 reset-config (flash 写) 后再 dev.ps1 dbg：观察 out_busy / flash_write_count / loop_count 是否推进

## 根因确认（通过 EP0 debug 通道拿到设备侧真相）
- flash_write_count=0 → reset-config 根本没写 flash，"flash 杀 vendor" 假设从头错误。
- 卡死态: out_busy=0, 但 read_flush 重武装 14 万次全失败。
- 根因: host 每次连接对 OUT 发 clear_halt → usbd_edpt_clear_stall 只清 busy 不清 claimed →
  留下 busy=0/claimed=1；tu_edpt_claim 要求 busy==0&&claimed==0 才成功 → claimed=1 时永久失败 →
  OUT 无法重新武装 → host->device 永久失步。主循环始终存活(loop_count 递增)。
- 修复: vendor_service 在 !busy 时先 usbd_edpt_release(清 claimed) 再 tud_vendor_read_flush 重武装。
- 附加: EP0 控制请求 0x52 触发 reset_usb_boot(loop 检测 g_bootsel_request)，
  dev.ps1 ctrl-bootsel / selftest --ctrl-bootsel，免物理 BOOTSEL（bulk 死时 EP0 仍活）。
- 两 build 均 SUCCESS。待烧录验证 reset-config 后 probe 是否恢复。

## 最终结论（已修复并验证）
真正根因: host io/mod.rs 每次连接对 OUT/IN 无条件 clear_halt → 固件 usbd_edpt_clear_stall 清 busy
不清 claimed → OUT 卡死(claimed=1 永久无法重新武装)。与 flash 完全无关(flash_write_count 曾为0)。
设备侧重新武装(release+read_flush)无论主循环还是控制回调上下文都会 hardfault→看门狗→BOOTSEL。
最终修复 = 移除 host 侧 clear_halt(io/mod.rs open_reader/open_writer)，消除触发源；
WinUSB 重开句柄不复位设备端点，OUT 保持已武装态。固件恢复为 vendor_service 仅监控 out_busy。

验证全绿: probe 5/5、reset-config→probe 8/8、full SELFTEST PASS(CH0 raw=442)、
soak PASS(36 通道真实数据+5s压测+25s idle 无断开)、csd-provision(flash_write_count=3)→probe 6/6、
csd-verify PASS(FINGER_TH ch0=199 持久+下发)。保留: 双核 lockout(保护 flash 写)、
debug 通道(0x50/0x51)、ctrl-bootsel(0x52)。
