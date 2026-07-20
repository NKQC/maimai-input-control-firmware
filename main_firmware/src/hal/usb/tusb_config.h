#pragma once

// 工程自带的 TinyUSB 配置覆盖文件。
// 通过 platformio.ini 的 -DCFG_TUSB_CONFIG_FILE="src/hal/usb/tusb_config.h"
// 让 tusb_option.h 的 `#include CFG_TUSB_CONFIG_FILE` 指向本文件，
// 而不是 framework-arduinopico/tools/libpico/tusb_config.h（那份文件对
// CFG_TUD_CDC/CFG_TUD_HID 是裸 #define，没有 #ifndef 保护，命令行 -D 覆盖不掉它）。
//
// 内容以框架默认版本为基础，仅调整设备类数量以匹配 usb-winusb-config 的
// 「恒定 config-WinUSB(vendor) + 按 mode.work 二选一(2×CDC 或 1×HID)」拓扑
// （config 从第 3 个 CDC 改为 vendor，见 usb-multicdc.md 里 UsbTreeView 实测：
//  纯 3×CDC 在本 RP2040+Adafruit TinyUSB 3.7.1+Windows 组合下 SET_CONFIG 阶段
//  稳定失败，2×CDC 稳定可用，故改用 vendor(WinUSB) 承载 config 通道）：
//   CFG_TUD_VENDOR = 1（config 通道，免驱 WinUSB，见 hal_usb.cpp 的 BOS/MS OS 2.0）
//   CFG_TUD_CDC = 2（serial + light，仅 Serial 模式下运行时描述符声明两个接口）
//   CFG_TUD_HID = 1（触摸+键盘复用同一个 HID 接口的不同 Report ID，不需要多实例）
//   CFG_TUD_MSC 保持框架默认值 1 不变 ——
//     实测踩坑：framework-arduinopico 预编译的 lib/rp2040/libpico.a 里已内置一份
//     用默认配置(MSC=1)编译好的 TinyUSB usbd/msc_device 目标文件；一旦本文件把
//     CFG_TUD_MSC 改成 0，本工程侧重新编译的 Adafruit_TinyUSB_Arduino 版本会整段
//     排掉 tud_msc_*_cb 的默认实现，但链接时仍会从 libpico.a 拉入需要这些符号的
//     msc_device.c.o，导致 undefined reference。本任务不需要改 MSC，故不碰它，
//     避免这个脆弱的预编译库交互面。

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
 #define CFG_TUSB_MCU             OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
 #define CFG_TUSB_OS              OPT_OS_PICO
#endif

// CFG_TUSB_DEBUG is defined by compiler in DEBUG build
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG           0
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
// mai2control v4 usb-winusb-config 拓扑：1 个 vendor 实例(config) +
// 2 个 CDC 实例(serial/light，仅 Serial 模式声明) + 1 个 HID 实例(仅 HID 模式声明)。
#define CFG_TUD_HID              (1)
#define CFG_TUD_CDC              (2)
#define CFG_TUD_MSC              (1)
#define CFG_TUD_MIDI             (0)
#define CFG_TUD_VENDOR           (1)

#define CFG_TUD_CDC_RX_BUFSIZE  (256)
#define CFG_TUD_CDC_TX_BUFSIZE  (256)

#define CFG_TUD_MSC_EP_BUFSIZE  (64)

// HID buffer size Should be sufficient to hold ID (if any) + Data
#define CFG_TUD_HID_EP_BUFSIZE  (64)

// Vendor(WinUSB config 通道)缓冲区，参考 Adafruit webusb_serial 示例取值。
#define CFG_TUD_VENDOR_RX_BUFSIZE (64)
#define CFG_TUD_VENDOR_TX_BUFSIZE (64)

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
