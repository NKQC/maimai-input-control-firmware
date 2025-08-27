#pragma once
#include "hal_usb_types.h"

//@huhuzhu
// 触控输入HID设备描述
/**
 * https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/windows-precision-touchpad-sample-report-descriptors
 * https://www.usbzh.com/article/detail-562.html
 * https://www.likecs.com/show-308126193.html#sc=400
 * https://blog.csdn.net/yunwen3344/article/details/8107439
 * https://blog.csdn.net/fengli1995/article/details/120708460
 * https://learn.microsoft.com/zh-cn/windows-hardware/design/component-guidelines/sample-report-descriptor--parallel-hybrid-mode-
 * https://blog.csdn.net/qq_28738985/article/details/104026994 //多点报文
 * https://www.usbzh.com/tool/usb.html //HID分析工具
 */


// USB设备描述符配置
#ifndef USB_VID
#define USB_VID                 0x0CA3  // 参考WashingTouch
#endif
#ifndef USB_PID
#define USB_PID                 0x0024  // 参考WashingTouch
#endif
#define USB_SERIAL              "01934"
#define USB_DEVICE_NAME         "Mai Control"

// HID设置

#define TOUCH_LOCAL_NUM         40      // 最大触摸点数
#define KEYBOARD_NUM            3       // 键盘数量 要和描述符保持一致
#define KEYBOARD_SIMUL_PRESS    12      // 键盘最大同时报告数

static const uint8_t hid_report_descriptor[] = {
    0x05, 0x0D,
    0x09, 0x04, // USAGE (Touch)
    0xA1, 0x01,
    0x85, HID_ReportID::REPORT_ID_TOUCHSCREEN, //   REPORT_ID (Touch pad)
    0x09, 0x22,
    0xA1, 0x02,
    0x09, 0x42,
    0x15, 0x00, // REPORT
    0x25, 0x01, // REPORT MAX
    0x75, 0x01, // REPORT_SIZE
    0x95, 0x01,
    0x81, 0x02, // Interrupt
    0x09, 0x30,
    0x25, 0x7F,
    0x75, 0x07,
    0x95, 0x01,
    0x81, 0x02,
    0x09, 0x51,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x02,
    0x05, 0x01,
    0x09, 0x30,
    0x09, 0x31,
    0x26, 0xFF, 0x7F,
    0x65, 0x00,
    0x75, 0x10, // REPORT_SIZE
    0x95, 0x02,
    0x81, 0x02,
    0xC0,
    0x05, 0x0D,
    0x47, 0xff, 0xff, 0x00, 0x00, //  PHYSICAL_MAXIMUM X (65535)
    0x27, 0xff, 0xff, 0x00, 0x00, //  LOGICAL_MAXIMUM (65535)
    0x47, 0xff, 0xff, 0x00, 0x00, //  PHYSICAL_MAXIMUM Y (65535)
    0x75, 0x10,                   // REPORT_SIZE
    0x95, 0x01,
    0x09, 0x56,
    0x81, 0x02, // Interrupt
    0x09, 0x54,
    0x25, TOUCH_LOCAL_NUM,
    0x75, 0x08,
    0x95, 0x01, // POINT COUNT
    0x81, 0x02,
    0x05, 0x0D,
    0x09, 0x55,
    0x25, TOUCH_LOCAL_NUM,
    0x75, 0x08,
    0x95, 0x01,
    0xB1, 0x02,
    0xC0,
    /* 41 */

    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,        // USAGE (Keyboard)
    0xa1, 0x01,        // COLLECTION (Application)
    0x85, HID_ReportID::REPORT_ID_KEYBOARD1, // Report ID (2)
    0x05, 0x07,        //     USAGE_PAGE (Keyboard/Keypad)
    0x19, 0xe0,        //     USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,        //     USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x25, 0x01,        //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,        //     REPORT_COUNT (8)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x81, 0x03,        //     INPUT (Cnst,Var,Abs)
    0x95, 0x06,        //   REPORT_COUNT (6)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,        //   LOGICAL_MAXIMUM (255)
    0x05, 0x07,        //   USAGE_PAGE (Keyboard/Keypad)
    0x19, 0x00,        //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,        //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,        //     INPUT (Data,Ary,Abs)
    0x25, 0x01,        //     LOGICAL_MAXIMUM (1)
    0x95, 0x05,        //   REPORT_COUNT (5)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x05, 0x08,        //   USAGE_PAGE (LEDs)
    0x19, 0x01,        //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,        //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x75, 0x03,        //   REPORT_SIZE (3)
    0x91, 0x03,        //   OUTPUT (Cnst,Var,Abs)
    0xc0,              //   END_COLLECTION

    0x05, 0x01,            // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,            // USAGE (Keyboard)
    0xa1, 0x01,            // COLLECTION (Application)
    0x85, HID_ReportID::REPORT_ID_KEYBOARD2, // Report ID (2)
    0x05, 0x07,            //     USAGE_PAGE (Keyboard/Keypad)
    0x19, 0xe0,            //     USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,            //     USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,            //     LOGICAL_MINIMUM (0)
    0x25, 0x01,            //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,            //     REPORT_COUNT (8)
    0x75, 0x01,            //     REPORT_SIZE (1)
    0x81, 0x02,            //     INPUT (Data,Var,Abs)
    0x95, 0x01,            //     REPORT_COUNT (1)
    0x75, 0x08,            //     REPORT_SIZE (8)
    0x81, 0x03,            //     INPUT (Cnst,Var,Abs)
    0x95, 0x06,            //   REPORT_COUNT (6)
    0x75, 0x08,            //   REPORT_SIZE (8)
    0x15, 0x00,            //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,            //   LOGICAL_MAXIMUM (255)
    0x05, 0x07,            //   USAGE_PAGE (Keyboard/Keypad)
    0x19, 0x00,            //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,            //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,            //     INPUT (Data,Ary,Abs)
    0x25, 0x01,            //     LOGICAL_MAXIMUM (1)
    0x95, 0x05,            //   REPORT_COUNT (5)
    0x75, 0x01,            //   REPORT_SIZE (1)
    0x05, 0x08,            //   USAGE_PAGE (LEDs)
    0x19, 0x01,            //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,            //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,            //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,            //   REPORT_COUNT (1)
    0x75, 0x03,            //   REPORT_SIZE (3)
    0x91, 0x03,            //   OUTPUT (Cnst,Var,Abs)
    0xc0,                  // END_COLLECTION

    0x05, 0x01,            // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,            // USAGE (Keyboard)
    0xa1, 0x01,            // COLLECTION (Application)
    0x85, HID_ReportID::REPORT_ID_KEYBOARD3, // Report ID (2)
    0x05, 0x07,            //     USAGE_PAGE (Keyboard/Keypad)
    0x19, 0xe0,            //     USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,            //     USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,            //     LOGICAL_MINIMUM (0)
    0x25, 0x01,            //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,            //     REPORT_COUNT (8)
    0x75, 0x01,            //     REPORT_SIZE (1)
    0x81, 0x02,            //     INPUT (Data,Var,Abs)
    0x95, 0x01,            //     REPORT_COUNT (1)
    0x75, 0x08,            //     REPORT_SIZE (8)
    0x81, 0x03,            //     INPUT (Cnst,Var,Abs)
    0x95, 0x06,            //   REPORT_COUNT (6)
    0x75, 0x08,            //   REPORT_SIZE (8)
    0x15, 0x00,            //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,            //   LOGICAL_MAXIMUM (255)
    0x05, 0x07,            //   USAGE_PAGE (Keyboard/Keypad)
    0x19, 0x00,            //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,            //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,            //     INPUT (Data,Ary,Abs)
    0x25, 0x01,            //     LOGICAL_MAXIMUM (1)
    0x95, 0x05,            //   REPORT_COUNT (5)
    0x75, 0x01,            //   REPORT_SIZE (1)
    0x05, 0x08,            //   USAGE_PAGE (LEDs)
    0x19, 0x01,            //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,            //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,            //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,            //   REPORT_COUNT (1)
    0x75, 0x03,            //   REPORT_SIZE (3)
    0x91, 0x03,            //   OUTPUT (Cnst,Var,Abs)
    0xc0                   // END_COLLECTION
};