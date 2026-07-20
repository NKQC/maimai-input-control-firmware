# usb-winusb-config 任务上下文（进行中）

## 任务目标
main_firmware 恒定 config 通道改为 WinUSB(vendor,免驱,BOS+MS OS 2.0),两个 CDC 让给 serial/light。
按 mode.work 二选一:Serial模式=vendor(config)+2xCDC(serial/light); HID模式=vendor(config)+HID。

## 架构结论(已定,来自 usb-multicdc.md ground truth)
- 纯 3xCDC 在本 RP2040+Adafruit TinyUSB 3.7.1+Windows 组合下 SET_CONFIG 阶段稳定失败(UsbTreeView 证实)。
- 2xCDC 稳定可用。config 改走 vendor 绕开这堵墙。
- 参考实现:Adafruit_TinyUSB_Arduino examples/WebUSB/webusb_serial(webusb_serial.ino) +
  src/arduino/webusb/Adafruit_USBD_WebUSB.cpp 里的 desc_bos/desc_ms_os_20/tud_vendor_control_xfer_cb 范式。
  本任务去掉 WebUSB(landing page/URL)分支,只保留 vendor+BOS+MS OS 2.0(WINUSB compatible ID)。
- 关键宏来源:usbd.h(TUD_BOS_DESCRIPTOR/TUD_BOS_MS_OS_20_DESCRIPTOR/TUD_VENDOR_DESCRIPTOR/TUD_VENDOR_DESC_LEN),
  tusb_types.h(MS_OS_20_* enum), vendor_device.h(tud_vendor_read/write/available/flush/tud_vendor_rx_cb)。

## 已完成步骤
1. tusb_config.h: CFG_TUD_VENDOR=1(RX/TX bufsize=64,参考webusb_serial), CFG_TUD_CDC=3→2, HID=1, MSC=1不变。
2. hal_usb.h: UsbCdcPort 去掉 CDC_CONFIG,只剩 CDC_SERIAL=0/CDC_LIGHT=1/CDC_PORT_COUNT_=2;
   HAL_USB/HAL_USB_Device 的无参 cdc_write/read/available/flush 改名为 config_write/read/available/flush;
   新增 tud_vendor_rx_cb 静态回调声明 + _handle_vendor_rx 私有方法 + config_rx_buffer_ 环形缓冲成员;
   include <class/vendor/vendor_device.h>;声明 tud_descriptor_bos_cb/tud_vendor_control_xfer_cb。
3. hal_usb.cpp 整体重写:
   - device_descriptor.bcdUSB 0x0200→0x0210(BOS要求>=0x0201)。
   - desc_configuration_serial: TUD_CONFIG+TUD_VENDOR_DESCRIPTOR(itf0)+2xTUD_CDC_DESCRIPTOR(itf1,itf3序号规则)。
   - desc_configuration_hid: TUD_CONFIG+TUD_VENDOR_DESCRIPTOR(itf0)+TUD_HID_DESCRIPTOR(itf1)。
   - current_usb_work_mode() 恢复正常读 ConfigManager,删除诊断隔离的硬编码 return。
   - 新增 desc_bos[]/desc_ms_os_20[](GUID={A4F76272-5901-410F-B895-FFACC2D9656F})+
     tud_descriptor_bos_cb/tud_vendor_control_xfer_cb(响应VENDOR_REQUEST_MICROSOFT=1,wIndex=7)。
   - config_write/read/available/flush 用 tud_vendor_* API 实现(带10ms超时阻塞写,同cdc_write风格)。
   - _handle_vendor_rx: 用 tud_vendor_read 主动 drain 内部 stream fifo 到自己的环形缓冲(仿 _handle_cdc_rx)。
4. main.cpp: HAL_USB_Device::getInstance()->cdc_write(...) 诊断打印口 → config_write(...)。
5. usb_comm.cpp: 全部 cdc_available/cdc_read/cdc_write/cdc_flush(无参,原config语义) → config_* 对应改名。

## 端点映射表(见 hal_usb.cpp 顶部注释,已写入代码)
### Serial模式 5接口/8端点(不含EP0)
| ITF | 类型 | 端点 |
|---|---|---|
| 0 | vendor(config) | OUT=0x01,IN=0x81 |
| 1 | CDC comm(serial notif) | 0x82 |
| 2 | CDC data(serial) | OUT=0x03,IN=0x83 |
| 3 | CDC comm(light notif) | 0x84 |
| 4 | CDC data(light) | OUT=0x05,IN=0x85 |

### HID模式 2接口/3端点
| ITF | 类型 | 端点 |
|---|---|---|
| 0 | vendor(config) | OUT=0x01,IN=0x81 |
| 1 | HID | IN=0x82 |

字符串表:0=lang,1=manufacturer,2=product,3=serial,4="mai2 config",5="mai2 serial",6="mai2 light",7="Mai Control HID"。

## DeviceInterfaceGUID(固定,已写入 MS OS 2.0 registry property)
{A4F76272-5901-410F-B895-FFACC2D9656F}
Rust上位机后续用 nusb/WinUSB 按此 GUID 或 VID(0x2E8A)/PID(0x000A,框架默认,firmware 实际枚举值)+接口号(itf0)打开 config 通道。
【勘误】早前笔记误写 VID 0x0CA3/PID 0x0024(来自 protocol_design.md 占位);固件描述符实际用 USB_VID/USB_PID=0x2E8A/0x000A。

## 任务状态：已完成(固件侧),等待用户硬件验证

## 编译验证结果
`pio run -e pico`:SUCCESS,RAM 10.3%(26996/262144B),Flash 14.6%(458364/3141632B),无新增警告。
额外核对 firmware.map:`vendor_device.c.o` 来自 `lib60b\Adafruit_TinyUSB_Arduino`(非 libpico),
证明 strip_libpico.py 的剥离表(含 vendor_device.c.o)对新拓扑仍然生效,单栈一致性保持。

## 补充修正(编译过程中发现,已修)
- `service/sensor_link/sensor_link.cpp::tick()` 也直接调用了旧的无参 `cdc_write()` 发送 TELEM_DATA
  流帧(与 UsbComm 同用 host_cmd 协议),一并改为 `config_write()`。sensor_link.h 两处注释同步更新。
- 这是本任务范围内被牵连的第三个调用点(main.cpp 诊断打印 + usb_comm.cpp + sensor_link.cpp)。

## 未做/明确不做
- 未烧录(用户在场验证,见交付报告的硬件验证指引)。
- 未改 Rust 上位机(control_software)侧的传输实现，仅在报告里给出 GUID 供后续任务对接。
