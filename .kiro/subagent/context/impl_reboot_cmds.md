# 任务: RP2040 系统重启指令实现 (REBOOT/REBOOT_BOOTLOADER) ✅ 完成

## 任务目标
在 RP2040 固件中实现两条系统重启指令(修订4):
- REBOOT(0x04): 普通软复位
- REBOOT_BOOTLOADER(0x05): 进入 UF2 烧录模式

按时序规范: 回 ACK → flush USB → 短延时泵 USB → 执行重启 API

## 架构理解 ✅
- HostCmd enum 在 host_cmd.h (系统域 0x01-0x0F)
- HostCmdDispatcher 在 main loop 中分发处理
- UsbComm 在 loop() 中逐帧处理: 编解 → 分发 → 写响应
- CDC 响应写入后需 flush + 泵 tud_task 让 USB 枚举/送出
- **关键**: 不能在响应写之前重启; 必须 deadline 检查机制

## 已完成步骤 ✅
1. ✅ 读取现有代码架构（host_cmd.h、UsbComm、main.cpp）
2. ✅ 修改 host_cmd.h: 添加 REBOOT(0x04) / REBOOT_BOOTLOADER(0x05) enum
3. ✅ 修改 usb_comm.h: 添加私有成员（_reboot_pending, _reboot_mode, _reboot_deadline_ms）
4. ✅ 修改 usb_comm.cpp:
   - 构造函数初始化重启标志
   - init() 中注册两个静态 handler:
     * REBOOT: encode_ack + 设置 _reboot_pending=true, _reboot_mode=0
     * REBOOT_BOOTLOADER: encode_ack + 设置 _reboot_pending=true, _reboot_mode=1
   - update() 末尾添加 deadline 检查逻辑:
     * 首次进入 deadline=0 时: flush CDC + 设置 deadline=millis()+100ms + return
     * 非阻塞检查 millis() 是否超 deadline: 到期时调用重启 API
5. ✅ pio run 编译通过 [SUCCESS]

## 编译结果 ✅
```
RAM:   [=         ]  10.3% (used 26952 bytes from 262144 bytes)
Flash: [=         ]  14.5% (used 456460 bytes from 3141632 bytes)
[SUCCESS] Took 4.03 seconds
```

## 最终 API 确认 ✅
**使用 arduino-pico API**:
- 普通重启: `rp2040.reboot()` ✅ (编译通过, 链接成功)
- 进烧录模式: `rp2040.rebootToBootloader()` ✅ (编译通过, 链接成功)

（备用 pico-sdk API 在代码注释中保留，但不需要）

## 交付物
1. 修改文件:
   - `src/protocol/host_cmd/host_cmd.h`: REBOOT/REBOOT_BOOTLOADER enum 添加
   - `src/service/usb_comm/usb_comm.h`: 重启支持成员字段添加
   - `src/service/usb_comm/usb_comm.cpp`: init() handler 注册 + update() deadline 检查逻辑

2. Handler 落点:
   - REBOOT handler: usb_comm.cpp init() 内 dispatcher->register_handler(HostCmd::REBOOT, ...)
   - REBOOT_BOOTLOADER handler: usb_comm.cpp init() 内 dispatcher->register_handler(HostCmd::REBOOT_BOOTLOADER, ...)

3. flush-then-reboot 时序:
   - update() 第一次遇到 _reboot_pending=true: flush CDC + 设置 100ms deadline + 返回
   - 主循环继续泵 HAL_USB_Device::task() 和 UsbComm::update()，USB 枚举/CDC 数据继续发出
   - 100ms 后，deadline 到期，调用 rp2040.reboot() / rp2040.rebootToBootloader() 执行重启

4. 注意事项:
   - ✅ SWD_RELEASE_TO_EXTERNAL 隔离模式下 UsbComm::update() 仍然被调用(loop 中在 PSoC 分支前)，重启指令照常工作
   - ✅ watchdog 已 enable(5000ms)，不会在 reboot 前触发(100ms deadline < 5000ms 看门狗)
   - ✅ 非阻塞实现，每次 update() 只检查 millis()，不阻塞整个 100ms
   - ✅ 编译通过，无链接错误，RAM/Flash 占用在安全范围内
