#include "hal_usb.h"
#include "../../config.h"   // MAI2_ENABLE_SERIAL_HID 开关
#include "../../service/config_manager/config_manager.h"
#include <pico/stdlib.h>
#include <tusb.h>
#include <device/usbd.h>
#include <device/usbd_pvt.h>
#include <class/cdc/cdc_device.h>
#include <class/hid/hid_device.h>
#include <pico/stdio.h>
#include <pico/bootrom.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <cstring>
#include "../../service/usb_debug.h"


// ★我们自己发起的 tud_* 端点调用必须与 USB IRQ 互斥★
// 依据(实测, 见 --mai2-load 二分): 只压 mai2light 那条 CDC、vendor 与 serial 全不动, 设备仍在
// 1~5s 内必挂; 死前遗言给出的定性是"主循环被拖死"且停在**我们自己**发起的 tud_cdc_n_read /
// tud_cdc_n_write 上, hardfault=0(不是跑飞)、进段时已耗时=0ms(不是前面的段拖慢), 之后主循环
// 再也没出来 → 5s 看门狗复位整机。只压 mai2serial 25s 则 740 帧/s 全程零错误 —— 差别正是
// 灯板方向每轮要发很多条小应答, 命中"端点操作窗口被 ISR 抢占"的机会高一个量级。
// 本工程 CFG_TUSB_OS=OPT_OS_PICO(tusb_config.h:41), 端点 claim 走 pico 阻塞互斥量, 该互斥量
// 不关中断; 主循环持锁期间被 USB ISR 抢占, 两边就永久等在一起。
// 本仓早有同类结论并写在案: vendor_service() 注释 "从主循环调用 dcd_edpt_xfer 会与 USB IRQ
// 竞态导致 hardfault(实测 bulk 连接即崩)"。此处是同一个坑的另一面。
// 代价: 这些调用只是搬 FIFO + 启一次传输, 微秒级; 关中断窗口极短, 不影响 USB 时序。
// ★禁止把 tud_task() / sleep 放进这个窗口★ —— 那会把中断关成毫秒级。
struct UsbIrqLock {
    uint32_t _saved;
    UsbIrqLock() : _saved(save_and_disable_interrupts()) {}
    ~UsbIrqLock() { restore_interrupts(_saved); }
    UsbIrqLock(const UsbIrqLock&) = delete;
    UsbIrqLock& operator=(const UsbIrqLock&) = delete;
};

// 静态实例指针
HAL_USB_Device* HAL_USB_Device::instance_ = nullptr;

// 字符串描述符缓冲区
static uint16_t desc_str[32];

// 调试计数器全局实例(经 vendor 控制请求 0x50 读取，见 usb_debug.h)。
volatile UsbDebugCounters g_usb_dbg = { 0xDB01u, (uint16_t)sizeof(UsbDebugCounters) };

volatile uint8_t g_bootsel_request = 0u;
volatile uint8_t g_psoc_reboot_request = 0u;
// 由 FlashWriteGuard 引用计数维护；flash XIP 临界区内调度器不得继续生成 vendor IN 推送。
volatile uint8_t g_usb_flash_busy = 0u;

volatile uint32_t g_last_host_cmd_ms = 0u;

// USB描述符
static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    // 正式 WinUSB 拓扑：bcdUSB >= 0x0201 时 Windows 请求 BOS，取得 MS OS 2.0
    // platform capability 后可将 config vendor 接口自动绑定到 WinUSB。
    .bcdUSB             = 0x0210,
    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0401,  // v4.01：隔离 Windows 对旧 0x0400 损坏 BOS 的 usbflags 缓存
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1
};

#define BOARD_TUD_RHPORT 0

// ============================================================
// USB 复合枚举拓扑（usb-winusb-config）
// 恒定：config-WinUSB(vendor)，免驱，host_cmd 上位机协议改走此通道。
// 按 ConfigManager["mode.work"] 二选一：
//   0 = Serial 模式：+ serial CDC + light CDC（不出现 HID）
//   1 = HID    模式：+ HID（触摸+键盘，不出现 serial/light CDC）
// 改动原因：纯 3×CDC 在本 RP2040+Adafruit TinyUSB 3.7.1+Windows 组合下
// SET_CONFIGURATION 阶段稳定失败（UsbTreeView 实测：Current Config Value=0x00，
// 复合父设备 Code 10），2×CDC 稳定可用，故把 config 通道从第 3 个 CDC 改为
// vendor(WinUSB)，两个 CDC 让给 serial/light。见 usb-multicdc.md。
// ============================================================

// ---- 变体A：Serial 模式 —— vendor(config) + 2×CDC(serial/light) + HID(键盘) ----
// ★键盘始终可用★：serial 模式也带 HID(触摸+键盘 report ID), 使 触控→键盘映射 与 物理键盘
// GPIO1-12 在游戏(serial)模式下同样能输出 HID 键。HID 接口排在最后, itf0 仍为 vendor,
// 不影响 MS OS 2.0/WinUSB 绑定。新增 HID IN 端点 0x86(不与既有端点冲突)。
#define EPNUM_S_CFG_OUT       0x01
#define EPNUM_S_CFG_IN        0x81
#define EPNUM_S_SERIAL_NOTIF  0x82
#define EPNUM_S_SERIAL_OUT    0x03
#define EPNUM_S_SERIAL_IN     0x83
#define EPNUM_S_LIGHT_NOTIF   0x84
#define EPNUM_S_LIGHT_OUT     0x05
#define EPNUM_S_LIGHT_IN      0x85
#define EPNUM_S_HID           0x86

#if MAI2_ENABLE_SERIAL_HID
enum {
    ITF_S_VENDOR_CFG = 0,
    ITF_S_CDC_SERIAL = 1,    // + ITF_S_CDC_SERIAL+1 = CDC数据接口
    ITF_S_CDC_LIGHT = 3,
    ITF_S_HID = 5,
    ITF_S_TOTAL = 6
};

#define CONFIG_SERIAL_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + 2 * TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t desc_configuration_serial[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_S_TOTAL, 0, CONFIG_SERIAL_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_VENDOR_DESCRIPTOR(ITF_S_VENDOR_CFG, 4, EPNUM_S_CFG_OUT, EPNUM_S_CFG_IN, CFG_TUD_VENDOR_EPSIZE),
    TUD_CDC_DESCRIPTOR(ITF_S_CDC_SERIAL, 5, EPNUM_S_SERIAL_NOTIF, 8, EPNUM_S_SERIAL_OUT, EPNUM_S_SERIAL_IN, 64),
    TUD_CDC_DESCRIPTOR(ITF_S_CDC_LIGHT, 6, EPNUM_S_LIGHT_NOTIF, 8, EPNUM_S_LIGHT_OUT, EPNUM_S_LIGHT_IN, 64),
    TUD_HID_DESCRIPTOR(ITF_S_HID, 7, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_descriptor), EPNUM_S_HID, CFG_TUD_HID_EP_BUFSIZE, 1)
};
#else
// 诊断: serial 模式回退到 vendor + 2×CDC(无 HID), 用于二分"进精调掉线"回归。
enum {
    ITF_S_VENDOR_CFG = 0,
    ITF_S_CDC_SERIAL = 1,    // + ITF_S_CDC_SERIAL+1 = CDC数据接口
    ITF_S_CDC_LIGHT = 3,
    ITF_S_TOTAL = 5
};

#define CONFIG_SERIAL_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + 2 * TUD_CDC_DESC_LEN)

static const uint8_t desc_configuration_serial[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_S_TOTAL, 0, CONFIG_SERIAL_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_VENDOR_DESCRIPTOR(ITF_S_VENDOR_CFG, 4, EPNUM_S_CFG_OUT, EPNUM_S_CFG_IN, CFG_TUD_VENDOR_EPSIZE),
    TUD_CDC_DESCRIPTOR(ITF_S_CDC_SERIAL, 5, EPNUM_S_SERIAL_NOTIF, 8, EPNUM_S_SERIAL_OUT, EPNUM_S_SERIAL_IN, 64),
    TUD_CDC_DESCRIPTOR(ITF_S_CDC_LIGHT, 6, EPNUM_S_LIGHT_NOTIF, 8, EPNUM_S_LIGHT_OUT, EPNUM_S_LIGHT_IN, 64)
};
#endif

// ---- 变体B：HID 模式 —— vendor(config) + HID ----
enum {
    ITF_H_VENDOR_CFG = 0,
    ITF_H_HID = 1,
    ITF_H_TOTAL = 2
};

#define EPNUM_H_CFG_OUT     0x01
#define EPNUM_H_CFG_IN      0x81
#define EPNUM_H_HID         0x82

#define CONFIG_HID_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t desc_configuration_hid[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_H_TOTAL, 0, CONFIG_HID_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_VENDOR_DESCRIPTOR(ITF_H_VENDOR_CFG, 4, EPNUM_H_CFG_OUT, EPNUM_H_CFG_IN, CFG_TUD_VENDOR_EPSIZE),
    TUD_HID_DESCRIPTOR(ITF_H_HID, 7, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_descriptor), EPNUM_H_HID, CFG_TUD_HID_EP_BUFSIZE, 1)
};

// 字符串描述符（两变体共用同一份，索引固定，未用到的角色索引对另一变体无副作用）
static const char* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: 支持的语言是英语 (0x0409)
    "Mai Control",                 // 1: 制造商
    "Mai Control Device",          // 2: 产品
    USB_SERIAL,                    // 3: 序列号
    "mai2 config",                 // 4: config vendor(WinUSB)接口（恒定枚举）
    "mai2 serial",                 // 5: serial CDC接口（Serial 模式）
    "mai2 light",                  // 6: light CDC接口（Serial 模式）
    "Mai Control HID",             // 7: HID接口（HID 模式）
};

// 读取当前工作模式。取自 ConfigManager["mode.work"]（0=Serial,1=HID，见 protocol_design.md）。
// ConfigManager 未初始化/键未注册时 get_uint8 返回 0（类型不匹配的安全兜底），天然回退到 Serial 模式。
static UsbWorkMode current_usb_work_mode() {
    uint8_t mode = ConfigManager::get_uint8("mode.work");
    return (mode == static_cast<uint8_t>(UsbWorkMode::WORK_HID)) ? UsbWorkMode::WORK_HID : UsbWorkMode::WORK_SERIAL;
}

// ============================================================
// BOS + MS OS 2.0 描述符集：让 config vendor 接口免驱自动绑定 WinUSB。
// 范式对齐 Adafruit TinyUSB examples/WebUSB/webusb_serial（BOS 里放
// MS OS 2.0 platform capability，用 vendor 控制请求 0x22(VENDOR_REQUEST_MICROSOFT)
// 返回 MS OS 2.0 描述符集）。本任务不需要 WebUSB 能力，故 BOS 只含 1 个
// device capability（MS OS 2.0），不含 WebUSB 那个。
//
// DeviceInterfaceGUID（固定，Rust 上位机按此 GUID 用 nusb/WinUSB 打开 config 接口）：
//   {A4F76272-5901-410F-B895-FFACC2D9656F}
// ============================================================

enum { VENDOR_REQUEST_MICROSOFT = 1 };

#define MS_OS_20_DESC_LEN 0xB2
#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT)
};

// MS OS 2.0 描述符集：Set Header + Configuration Subset + Function Subset
// (bFirstInterface 固定填 0——两套描述符变体里 vendor(config) 都恒为 itf0，
// 见 ITF_S_VENDOR_CFG/ITF_H_VENDOR_CFG，无需运行时回填) + Compatible ID
// "WINUSB" + Registry Property "DeviceInterfaceGUIDs"。字节布局与官方
// webusb_serial 示例一致，仅去掉 WebUSB 相关 vendor code 分支。
static const uint8_t desc_ms_os_20[] = {
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Configuration subset header: length, type, configuration index, reserved, configuration total length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    // Function Subset header: length, type, first interface(=0, 恒为 itf0), reserved, subset length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    0 /*itf num*/, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

    // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID "WINUSB", sub compatible ID
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W',
    'I', 'N', 'U', 'S', 'B', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, // sub-compatible

    // MS OS 2.0 Registry property descriptor: length, type
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(0x0007),
    U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength,
                           // PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00,
    'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00,
    'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00,
    0x00,
    U16_TO_U8S_LE(0x0050), // wPropertyDataLength
    // bPropertyData: "{A4F76272-5901-410F-B895-FFACC2D9656F}\0\0"
    '{', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, '7', 0x00, '6', 0x00, '2', 0x00,
    '7', 0x00, '2', 0x00, '-', 0x00, '5', 0x00, '9', 0x00, '0', 0x00, '1', 0x00,
    '-', 0x00, '4', 0x00, '1', 0x00, '0', 0x00, 'F', 0x00, '-', 0x00, 'B', 0x00,
    '8', 0x00, '9', 0x00, '5', 0x00, '-', 0x00, 'F', 0x00, 'F', 0x00, 'A', 0x00,
    'C', 0x00, 'C', 0x00, '2', 0x00, 'D', 0x00, '9', 0x00, '6', 0x00, '5', 0x00,
    '6', 0x00, 'F', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

//--------------------------------------------------------------------+
// TinyUSB 必需的回调函数 注意 这个地方的模块需要把Adafruit_USBD_HID.cpp的回调移除
//--------------------------------------------------------------------+

extern "C" {

// USB中断处理函数
void usb_irq_handler() {
    tud_int_handler(BOARD_TUD_RHPORT);
}

// 设备描述符回调
uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&device_descriptor;
}

// BOS 描述符回调（承载 MS OS 2.0 platform capability，供 Windows 免驱绑定 WinUSB）
uint8_t const* tud_descriptor_bos_cb(void) {
    return desc_bos;
}

// HID报告描述符回调 - 使用自定义描述符
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return hid_report_descriptor;
}

// 配置描述符回调：按当前 mode.work 二选一返回对应变体。
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index; // 对于单一配置
    return (current_usb_work_mode() == UsbWorkMode::WORK_HID)
        ? desc_configuration_hid
        : desc_configuration_serial;
}

// 字符串描述符回调
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    
    uint8_t chr_count;
    
    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        // 注意：数组必须是字符串字面量，否则会在栈上分配
        if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL;
        
        const char* str = string_desc_arr[index];
        
        // 将ASCII转换为UTF-16
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        
        for (uint8_t i = 0; i < chr_count; i++) {
            desc_str[1+i] = str[i];
        }
    }
    
    // 第一个字节是长度（包括头部），第二个字节是描述符类型
    desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);
    
    return desc_str;
}

// HID获取报告回调
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    
    return 0;
}

// HID设置报告回调
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}

// CDC接收回调
void tud_cdc_rx_cb(uint8_t itf) {
    HAL_USB_Device::getInstance()->tud_cdc_rx_cb(itf);
}

// vendor(config通道)接收回调。CFG_TUD_VENDOR=1，itf 恒为 0。
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
    HAL_USB_Device::getInstance()->tud_vendor_rx_cb(itf, buffer, bufsize);
}

// vendor 控制请求回调：响应 MS OS 2.0 描述符集获取请求（VENDOR_REQUEST_MICROSOFT）。
// 范式对齐 Adafruit webusb_serial 示例的 tud_vendor_control_xfer_cb，去掉 WebUSB 分支。
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    // 仅在 SETUP 阶段处理；DATA/ACK 阶段直接放行
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    // 调试读(bRequest=0x50)：经 EP0 返回诊断计数器；0x51=SET_DEBUG 开关(默认关)。
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x50) {
        g_usb_dbg.mounted = tud_mounted() ? 1u : 0u;
        g_usb_dbg.out_busy = usbd_edpt_busy(BOARD_TUD_RHPORT, 0x01) ? 1u : 0u;
        g_usb_dbg.out_stalled = usbd_edpt_stalled(BOARD_TUD_RHPORT, 0x01) ? 1u : 0u;
        uint16_t len = (uint16_t)sizeof(UsbDebugCounters);
        if (request->wLength < len) len = request->wLength;
        return tud_control_xfer(rhport, request, (void*)&g_usb_dbg, len);
    }
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x51) {
        g_usb_dbg.debug_enabled = (request->wValue != 0u) ? 1u : 0u;
        return tud_control_xfer(rhport, request, NULL, 0);
    }
    // 0x53=清零主循环阻塞剖面(loop_max_us / seg_max_us)。峰值量不可差分, 压测须能开一个干净窗口。
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x53) {
        loop_prof_clear();
        return tud_control_xfer(rhport, request, NULL, 0);
    }
    // 0x52=进 BOOTSEL：置请求位，由 loop() 在 ACK 完成后 reset_usb_boot。EP0 通道使得 bulk 死时仍可软件进烧录。
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x52) {
        g_bootsel_request = 1u;
        return tud_control_xfer(rhport, request, NULL, 0);
    }

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
        request->bRequest == VENDOR_REQUEST_MICROSOFT && request->wIndex == 7) {
        // Get Microsoft OS 2.0 compatible descriptor
        uint16_t total_len;
        memcpy(&total_len, desc_ms_os_20 + 8, 2);
        return tud_control_xfer(rhport, request, (void*)desc_ms_os_20, total_len);
    }

    // 未识别的 vendor 请求：stall
    return false;
}

// 挂载回调
void tud_mount_cb(void) {
    // 设备已连接
}

// 卸载回调
void tud_umount_cb(void) {
    // 设备已断开
}

// 挂起回调
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
}

// 恢复回调
void tud_resume_cb(void) {
    // 设备已恢复
}
} // extern "C"

// HAL_USB_Device 实现
HAL_USB_Device* HAL_USB_Device::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_USB_Device();
    }
    return instance_;
}

HAL_USB_Device::HAL_USB_Device()
    : initialized_(false), connected_(false), _command_response_active(false), config_rx_head_(0), config_rx_tail_(0) {
    for (size_t i = 0; i < CDC_PORT_COUNT; i++) {
        cdc_rx_head_[i] = 0;
        cdc_rx_tail_[i] = 0;
    }
}

HAL_USB_Device::~HAL_USB_Device() {
    deinit();
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

bool HAL_USB_Device::init() {
    if (initialized_) {
        return true;
    }
    // 初始化TinyUSB设备栈
    if(!tud_init(BOARD_TUD_RHPORT)) return false;
    // Keep the pull-up removed long enough for a USB 2.0 hub to observe a stable detach.
    tud_disconnect();
    sleep_ms(350);
    tud_connect();
    initialized_ = true;
    return true;
}

void HAL_USB_Device::deinit() {
    if (initialized_) {
        // TinyUSB没有提供deinit函数，这里只标记为未初始化
        initialized_ = false;
        connected_ = false;
    }
}

bool HAL_USB_Device::is_connected() const {
    return initialized_ && tud_mounted();
}

bool HAL_USB_Device::is_ready() const {
    return initialized_ && tud_ready();
}

bool HAL_USB_Device::config_write(const uint8_t* data, size_t length) {
    if (_command_response_active) return false;
    if (!is_ready() || !data || length == 0) {
        return length == 0;
    }

    // ★有限忙等分段发送(已验证 DIAGNOSE PASS 的机制)★: 按字节流分段写入 vendor TX FIFO(64B),
    // FIFO 满时 pump tud_task 推进 IN 泵出 + 短 sleep 让 USB IRQ 完成传输, 腾空后写余下段。
    // 超时(TIMEOUT_US)仍写不完 = 过载, 返回 false(命令响应→上位机重试; 遥测→下周期再发)。
    // 有上限、不永久阻塞; 配合上层"定时任务队列+续期降频"避免频繁触发忙等。
    static uint64_t last_avail_time = 0;
    // 10ms 上限: 大响应帧(如 CFG_GET_ALL ~3KB)在 64B FIFO 下需分约 50 段, 每段等 IN 泵出,
    // 3ms 不足以发完→超时丢帧。10ms 为已验证 PASS 值。大帧仅连接时偶发, 单次阻塞可接受;
    // 遥测等高频帧较小(分段少), 且由定时任务队列降频, 不会频繁触发满忙等。
    const uint64_t TIMEOUT_US = 10000;
    size_t total_written = 0;
    const uint64_t start_time = time_us_64();
    while (total_written < length) {
        // ★端点操作在关中断窗口内完成★(见 UsbIrqLock 注释); tud_task/sleep 绝不进这个窗口。
        uint32_t available;
        {
            UsbIrqLock lock;
            available = tud_vendor_write_available();
        }
        if (available > 0) {
            const size_t chunk = std::min((size_t)available, length - total_written);
            uint32_t written;
            {
                UsbIrqLock lock;
                written = tud_vendor_write(data + total_written, (uint32_t)chunk);
                if (written > 0) tud_vendor_write_flush();
            }
            if (written > 0) {
                total_written += written;
                last_avail_time = time_us_64();
            }
        } else {
            const uint64_t now = time_us_64();
            if (now - start_time > TIMEOUT_US ||
                (last_avail_time > 0 && now - last_avail_time > TIMEOUT_US)) {
                break;   // 过载超时: 放弃(拒绝)
            }
            if (initialized_) tud_task();       // 推进 IN 泵出腾 FIFO
            {
                UsbIrqLock lock;
                tud_vendor_write_flush();
            }
            sleep_us(50);                       // 给 USB IRQ 完成 IN 传输的时间窗口
        }
    }
    if (total_written > 0) {
        g_usb_dbg.vendor_tx_calls++;
        g_usb_dbg.vendor_tx_bytes += (uint32_t)total_written;
    }
    return total_written == length;
}

size_t HAL_USB_Device::config_write_some(const uint8_t* data, size_t length) {
    if (!is_ready() || !data || length == 0) return 0;

    UsbIrqLock lock;
    const size_t chunk = std::min((size_t)tud_vendor_write_available(), length);
    const size_t written = chunk > 0 ? tud_vendor_write(data, (uint32_t)chunk) : 0;
    tud_vendor_write_flush();
    if (written > 0) {
        g_usb_dbg.vendor_tx_calls++;
        g_usb_dbg.vendor_tx_bytes += (uint32_t)written;
    }
    return written;
}

void HAL_USB_Device::begin_command_response() {
    _command_response_active = true;
}

void HAL_USB_Device::end_command_response() {
    _command_response_active = false;
}

size_t HAL_USB_Device::config_write_available() const {
    if (!initialized_) return 0;
    return tud_vendor_write_available();
}

size_t HAL_USB_Device::config_read(uint8_t* buffer, size_t max_length) {
    if (!initialized_) return 0;

    size_t count = 0;
    while (count < max_length && config_rx_head_ != config_rx_tail_) {
        buffer[count++] = config_rx_buffer_[config_rx_tail_];
        config_rx_tail_ = (config_rx_tail_ + 1) % CONFIG_BUFFER_SIZE;
    }

    return count;
}

size_t HAL_USB_Device::config_available() const {
    if (config_rx_head_ >= config_rx_tail_) {
        return config_rx_head_ - config_rx_tail_;
    } else {
        return CONFIG_BUFFER_SIZE - config_rx_tail_ + config_rx_head_;
    }
}

void HAL_USB_Device::config_flush() {
    if (initialized_) {
        tud_vendor_write_flush();
    }
}

void HAL_USB_Device::reconnect() {
    if (!initialized_) return;
    config_flush();
    // 排空进行中的完成事件后断开物理线。
    for (uint8_t pass = 0; pass < 8; pass++) { tud_task(); sleep_ms(1); }
    tud_disconnect();
    sleep_ms(80);   // 让 host 观测到稳定拔出
    tud_connect();
    // 泵若干次 task 推进重新枚举（其余由主循环持续 task 完成）。
    for (uint8_t pass = 0; pass < 8; pass++) { tud_task(); sleep_ms(1); }
    connected_ = false;   // 由 tud_mount_cb 在重新枚举完成后置回
}

void HAL_USB_Device::soft_disconnect() {
    if (!initialized_) return;

    tud_vendor_write_flush();
    for (size_t port = 0; port < CDC_PORT_COUNT; port++) {
        tud_cdc_n_write_flush(static_cast<uint8_t>(port));
    }
    // Drain queued completion events before changing the physical line state.
    for (uint8_t pass = 0; pass < 8; pass++) {
        tud_task();
        sleep_ms(1);
    }
    tud_disconnect();
    connected_ = false;
}

void HAL_USB_Device::_handle_vendor_rx(uint8_t const* buffer, uint16_t bufsize) {
    // vendord_xfer_cb 在调用 tud_vendor_rx_cb 前已把数据写入内部 stream fifo
    // （tu_edpt_stream_read_xfer_complete），此处传入的 buffer/bufsize 只是通知用的
    // 原始只读快照；若不主动 drain 内部 fifo，OUT 传输会在 CFG_TUD_VENDOR_RX_BUFSIZE
    // 用尽后停止接收。故仿照本文件 CDC 的 _handle_cdc_rx 模式，实际用
    // tud_vendor_read() 把内部 fifo 的数据搬进自己的环形缓冲。
    (void)buffer;
    (void)bufsize;

    uint8_t tmp[64];
    uint32_t count;
    uint32_t drained = 0;
    uint32_t dropped = 0;
    while ((count = tud_vendor_read(tmp, sizeof(tmp))) > 0) {
        drained += count;
        for (uint32_t i = 0; i < count; i++) {
            size_t next_head = (config_rx_head_ + 1) % CONFIG_BUFFER_SIZE;
            if (next_head != config_rx_tail_) {
                config_rx_buffer_[config_rx_head_] = tmp[i];
                config_rx_head_ = next_head;
            } else {
                // ★溢出不再静默★: 环满意味着 core0 长时间没来取(重操作阻塞), 被丢掉的字节会让
                // 主机的命令帧残缺 —— 这正是"设备莫名不响应/掉线"排障时最需要的证据。
                dropped++;
            }
        }
    }
    g_usb_dbg.vendor_rx_cb_count++;
    g_usb_dbg.vendor_rx_bytes += drained;
    g_usb_dbg.vendor_rx_dropped += dropped;
}

// TinyUSB回调函数实现：vendor 接收数据搬进 config 环形缓冲。
void HAL_USB_Device::tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
    (void)itf; // CFG_TUD_VENDOR=1，恒为 itf0
    if (instance_) {
        instance_->_handle_vendor_rx(buffer, bufsize);
    }
}

size_t HAL_USB_Device::cdc_write_available(UsbCdcPort port) const {
    if (!initialized_) return 0;
    pm_stage(PM_STAGE_CDC_AVAIL);
    UsbIrqLock lock;
    return tud_cdc_n_write_available(static_cast<uint8_t>(port));
}

// ★非阻塞、整帧原子★
// 原实现在这里 `sleep_us(100)` 自旋最多 10ms 等 FIFO 腾空, 既不喂狗也不泵 tud_task:
//   * 违反"禁止任何形式的 delay 与空转忙等";
//   * 主机稍慢读 CDC IN, 每帧就付 10ms —— 灯板一轮 task 最多派发 ~28 帧应答 ⇒ 单轮主循环被拖到
//     百毫秒级, 而 vendor(WinUSB)只在主循环开头被泵一次 ⇒ 主机在途传输被 Windows abort、判掉线
//     (实测 --mai2-load: os error 121/22 + ConnectionAborted, 随后设备复位)。
// 现在: 空间够就整帧写, 不够就整帧不写并返回 false, 由调用方按各自语义处理(触控帧丢一帧、
// 灯板应答丢一条), 绝不半帧写入 —— 半帧会让收端永久错帧, 比丢帧严重得多。
bool HAL_USB_Device::cdc_write(UsbCdcPort port, const uint8_t* data, size_t length) {
    if (!is_ready() || !data || length == 0) {
        return length == 0;
    }
    const uint8_t itf = static_cast<uint8_t>(port);
    pm_stage(PM_STAGE_CDC_WRITE);
    UsbIrqLock lock;
    if (tud_cdc_n_write_available(itf) < length) {
        tud_cdc_n_write_flush(itf);   // 催一次让 IN 尽快腾空, 下一轮再试
        return false;
    }
    const uint32_t written = tud_cdc_n_write(itf, data, (uint32_t)length);
    tud_cdc_n_write_flush(itf);
    return written == length;
}

size_t HAL_USB_Device::cdc_read(UsbCdcPort port, uint8_t* buffer, size_t max_length) {
    if (!initialized_) return 0;

    const size_t p = static_cast<size_t>(port);
    size_t count = 0;
    while (count < max_length && cdc_rx_head_[p] != cdc_rx_tail_[p]) {
        buffer[count++] = cdc_rx_buffer_[p][cdc_rx_tail_[p]];
        cdc_rx_tail_[p] = (cdc_rx_tail_[p] + 1) % CDC_BUFFER_SIZE;
    }

    return count;
}

size_t HAL_USB_Device::cdc_available(UsbCdcPort port) const {
    const size_t p = static_cast<size_t>(port);
    if (cdc_rx_head_[p] >= cdc_rx_tail_[p]) {
        return cdc_rx_head_[p] - cdc_rx_tail_[p];
    } else {
        return CDC_BUFFER_SIZE - cdc_rx_tail_[p] + cdc_rx_head_[p];
    }
}

void HAL_USB_Device::cdc_flush(UsbCdcPort port) {
    if (initialized_) {
        UsbIrqLock lock;
        tud_cdc_n_write_flush(static_cast<uint8_t>(port));
    }
}

void HAL_USB_Device::task() {
    if (initialized_) {
        pm_stage(PM_STAGE_TUD_TASK);
        tud_task();
        pm_stage(PM_STAGE_TUD_TASK_DONE);
        g_usb_dbg.tud_task_count++;
    }
}

void HAL_USB_Device::vendor_service() {
    if (!is_ready()) return;
    // config vendor OUT 端点在两套描述符变体里恒为 0x01(EPNUM_S/H_CFG_OUT)。
    // usbd_edpt_busy==false 表示 OUT 已掉出 arm 态(正常时恒 busy 等待 host 数据)，
    // 此时用 read_flush 清 fifo 并重新 read_xfer 武装 OUT，恢复 host→device 通路。
    // 仅采样 OUT busy 供调试观测(只读，安全)。真正的重新武装在 tud_vendor_control_xfer_cb 里
    // 处理 CLEAR_FEATURE(HALT) 时进行——那是 tud_task 上下文，可安全操作 dcd；
    // 从主循环调用 dcd_edpt_xfer 会与 USB IRQ 竞态导致 hardfault(实测 bulk 连接即崩)。
    constexpr uint8_t EPNUM_CFG_OUT = 0x01;
    g_usb_dbg.out_busy = usbd_edpt_busy(BOARD_TUD_RHPORT, EPNUM_CFG_OUT) ? 1u : 0u;
}

void HAL_USB_Device::_handle_cdc_rx(uint8_t itf) {
    if (itf >= CDC_PORT_COUNT) return;
    if (!tud_cdc_n_available(itf)) return;

    uint8_t buffer[64];
    pm_stage(PM_STAGE_CDC_READ);
    uint32_t count;
    {
        // tud_cdc_n_read 内部会重新武装 OUT 端点(usbd_edpt_xfer), 同样属于端点操作。
        UsbIrqLock lock;
        count = tud_cdc_n_read(itf, buffer, sizeof(buffer));
    }
    pm_stage(PM_STAGE_CDC_READ_DONE);

    if (count > 0) {
        // 存储到该角色对应的环形缓冲区
        for (uint32_t i = 0; i < count; i++) {
            size_t next_head = (cdc_rx_head_[itf] + 1) % CDC_BUFFER_SIZE;
            if (next_head != cdc_rx_tail_[itf]) {
                cdc_rx_buffer_[itf][cdc_rx_head_[itf]] = buffer[i];
                cdc_rx_head_[itf] = next_head;
            }
        }
    }
    pm_stage(PM_STAGE_CDC_RX_DONE);
}

// TinyUSB回调函数实现：按 itf 分流到对应角色的环形缓冲。
void HAL_USB_Device::tud_cdc_rx_cb(uint8_t itf) {
    if (instance_) {
        instance_->_handle_cdc_rx(itf);
    }
}
