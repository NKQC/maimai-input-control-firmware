# usb-winusb-bos-fix 任务上下文（因构建失败暂停）

## 任务目标
在不修改 PlatformIO 框架安装目录的前提下，隔离 Adafruit WebUSB 对象中与工程同名的两个弱回调，恢复正式 vendor(WinUSB)+2×CDC / vendor+HID 拓扑，并静态证明最终 ELF 使用工程的 33 字节纯 MS OS 2.0 BOS。

## 已完成步骤
1. 已读取 `usb-winusb-config.md`、`usb-multicdc.md`、现有 `strip_libpico.py`、`hal_usb.cpp` 与 `build.ps1`；本任务恢复文件此前不存在。
2. 已恢复 `hal_usb.cpp` 正式描述符：`bcdUSB=0x0210`、Serial 配置共 5 接口、总长为 config+vendor+2×CDC，并恢复 light CDC 描述符与既定端点；`current_usb_work_mode()` 未改为硬编码。
3. 已在 `strip_libpico.py` 增加 ELF 链接前动作：glob 定位构建目录 Adafruit WebUSB 对象；objcopy 重命名两个冲突回调；nm 支持增量构建幂等检查；对象缺失、状态异常或 objcopy 失败均抛错中止链接。
4. 已完整重写并人工检查 `main_firmware/build.ps1`：仅执行 `pio run -e pico`，随后以 nm/objdump/map 对象与最终 ELF 进行静态验收；无删除、烧录、上传、框架安装目录修改或网络操作。
5. 已严格通过固定 PowerShell 命令执行构建；未进入链接，静态验收未执行。

## 异常报告
`pio run -e pico` 在 SCons 调用链接前动作时失败，错误为：
```
TypeError: _rename_webusb_callbacks() got an unexpected keyword argument 'env'
```
当前回调定义是位置参数 `def _rename_webusb_callbacks(source, target, action_env)`，而 SCons 使用关键字参数 `target=..., source=..., env=...` 调用。根据“出问题即停即报”规则，未修改回调签名、未重试构建。

## 关键结论/决策
- 失败发生于工程脚本的函数签名不匹配，框架安装目录未被修改，未生成新 ELF。
- 上一次构建期间 `hal_usb.cpp` 已被重新编译；Adafruit WebUSB 对象存在于 `.pio/build/pico/lib60b/...`，但本次未成功调用 objcopy。

## 待办步骤
1. 取得继续处理的指示后，将回调函数形参改为 SCons 可接受的 `env`（或等效关键字兼容形式）。
2. 再次通过固定 `build.ps1` 命令构建；仅在成功后继续 nm/objdump/map 验收。
3. 汇总变更和静态证据；不烧录。

## 下一步动作
等待主 agent 处理本异常或明确授权继续修复该 SCons 回调签名。


---

# 最终完成记录（主 agent）

## 根因最终确认
TinyUSB `device/usbd.h` 将 `tud_descriptor_bos_cb` 与 `tud_vendor_control_xfer_cb` 声明为 `TU_ATTR_WEAK`，导致工程 `hal_usb.cpp.o` 与 `Adafruit_USBD_WebUSB.cpp.o` 中两组定义均为弱符号。原链接顺序选择了 Adafruit WebUSB 的回调，其 BOS 为57字节/2 capabilities（WebUSB + MS OS 2.0，MS vendor code=2），而工程控制处理器期望纯MS OS BOS/vendor code=1；Windows在bcdUSB=0x0210请求BOS时因此中止枚举。bcdUSB临时降至0x0200后vendor+CDC可枚举且vendor仅Code28，进一步证明vendor接口本身无误。

## 最终修复
- `strip_libpico.py` 保留原 libpico TinyUSB成员剥离逻辑，并新增 ELF 链接前 pre-action：glob定位构建目录里的 `Adafruit_USBD_WebUSB.cpp.o`，用objcopy幂等重命名：
  - `tud_descriptor_bos_cb` → `adafruit_tud_descriptor_bos_cb_unused`
  - `tud_vendor_control_xfer_cb` → `adafruit_tud_vendor_control_xfer_cb_unused`
- 对象找不到、符号处于部分状态、nm/objcopy失败均抛错中止链接；不修改PlatformIO框架安装目录。
- 修正SCons pre-action函数签名为 `(source, target, env)`。
- `hal_usb.cpp` 已恢复正式 `bcdUSB=0x0210` 与 Serial模式 vendor(config)+serial CDC+light CDC（5接口），HID模式vendor+HID；不再是vendor+1CDC诊断态。

## 构建与静态验收
- `pio run -e pico`: SUCCESS；RAM 10.2%（26816/262144），Flash 14.6%（458252/3141632）。
- nm:
  - HAL对象：原名两回调均为W。
  - Adafruit WebUSB对象：仅有 `adafruit_*_unused`，不再导出原名。
  - 最终ELF：原名回调存在，来自工程HAL唯一候选。
- 最终ELF二进制字节扫描：
  - `GOOD_BOS_OFFSETS=[8782,-1]`：33字节纯MS OS 2.0 BOS存在，头 `05 0F 21 00 01`，MS vendor code=1。
  - `OLD_WEBUSB_BOS_OFFSETS=[-1,-1]`：旧57字节WebUSB+BOS不存在。
  - `GUID_OFFSETS=[8702,-1]`：`{A4F76272-5901-410F-B895-FFACC2D9656F}`存在。
  - RESULT=PASS。
- map仍使用 `libpico_nousb.a`，TinyUSB usbd/cdc/vendor/dcd核心来自Adafruit。

## Shell固定入口
用户要求所有命令先写入 `main_firmware/build.ps1`，完整检查后再执行。当前终端不应嵌套启动`powershell.exe`（盘符解析/钩子会阻止脚本执行）；可靠固定调用为：
`& "f:\mai2control\mai2control-v4\main_firmware\build.ps1"`

## 待硬件验证
烧录 `.pio/build/pico/firmware.uf2`。默认Serial模式预期：WinUSB `mai2 config`自动绑定且无Code28/Code10，同时出现`mai2 serial`和`mai2 light`两个COM口；无HID。若异常，立即采集UsbTreeView，重点看bcdUSB=0x0210、BOS总长0x21/caps=1、Current Config Value是否为1。


## Windows BOS缓存隔离最终补充
- 因此前损坏BOS已写入 Windows `usbflags\2E8A000A0400`（UsbTreeView显示osvc=00 00），正式修复版将 `bcdDevice` 从0x0400提升到**0x0401**，强制Windows建立新缓存键 `2E8A000A0401` 并重新请求BOS/MS OS 2.0描述符。VID/PID保持0x2E8A/0x000A。
- 固定 `build.ps1` 已重新运行，日志明确重新编译 `src/hal/usb/hal_usb.cpp.o` 后链接；SUCCESS，RAM10.2%，Flash14.6%。
- 重建后的ELF再次字节验收：GOOD_BOS_OFFSETS=[8782,-1]、OLD_WEBUSB_BOS_OFFSETS=[-1,-1]、GUID_OFFSETS=[8702,-1]、RESULT=PASS。
- 最终待烧录文件：`main_firmware/.pio/build/pico/firmware.uf2`。
