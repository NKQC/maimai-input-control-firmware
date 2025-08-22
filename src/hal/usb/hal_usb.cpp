#include "hal_usb.h"
#include <pico/stdlib.h>
#include <tusb.h>
#include <device/usbd.h>
#include <class/cdc/cdc_device.h>
#include <class/hid/hid_device.h>
#include <cstring>

// 静态实例指针
HAL_USB_Device* HAL_USB_Device::instance_ = nullptr;

// USB描述符
static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1
};

// HID报告描述符（参考WashingTouch实现）
static const uint8_t hid_report_descriptor[] = {
    // 触摸屏报告描述符
    0x05, 0x0D,        // Usage Page (Digitizer)
    0x09, 0x04,        // Usage (Touch Screen)
    0xA1, 0x01,        // Collection (Application)
    0x85, REPORTID_TOUCHPAD,  // Report ID
    0x09, 0x22,        // Usage (Finger)
    0xA1, 0x02,        // Collection (Logical)
    0x09, 0x42,        // Usage (Tip Switch)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x09, 0x32,        // Usage (In Range)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x09, 0x47,        // Usage (Confidence)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x95, 0x05,        // Report Count (5)
    0x81, 0x03,        // Input (Cnst,Var,Abs)
    0x75, 0x08,        // Report Size (8)
    0x09, 0x51,        // Usage (Contact Identifier)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x26, 0x80, 0x07,  // Logical Maximum (1920)
    0x75, 0x10,        // Report Size (16)
    0x55, 0x0E,        // Unit Exponent (-2)
    0x65, 0x33,        // Unit (Inch,EngLinear)
    0x09, 0x30,        // Usage (X)
    0x35, 0x00,        // Physical Minimum (0)
    0x46, 0x80, 0x07,  // Physical Maximum (1920)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x26, 0x38, 0x04,  // Logical Maximum (1080)
    0x46, 0x38, 0x04,  // Physical Maximum (1080)
    0x09, 0x31,        // Usage (Y)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0xC0,              // End Collection
    0x05, 0x0D,        // Usage Page (Digitizer)
    0x09, 0x54,        // Usage (Contact Count)
    0x25, TOUCH_LOCAL_NUM,  // Logical Maximum (10)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0xC0,              // End Collection
    
    // 键盘1报告描述符
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, REPORTID_KEYBOARD_1,  // Report ID
    0x05, 0x07,        // Usage Page (Keyboard/Keypad)
    0x19, 0xE0,        // Usage Minimum (Left Control)
    0x29, 0xE7,        // Usage Maximum (Right GUI)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x08,        // Report Count (8)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x95, 0x01,        // Report Count (1)
    0x75, 0x08,        // Report Size (8)
    0x81, 0x01,        // Input (Cnst,Ary,Abs)
    0x95, 0x06,        // Report Count (6)
    0x75, 0x08,        // Report Size (8)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x65,        // Logical Maximum (101)
    0x05, 0x07,        // Usage Page (Keyboard/Keypad)
    0x19, 0x00,        // Usage Minimum (0)
    0x29, 0x65,        // Usage Maximum (101)
    0x81, 0x00,        // Input (Data,Ary,Abs)
    0xC0,              // End Collection
    
    // 键盘2报告描述符
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, REPORTID_KEYBOARD_2,  // Report ID
    0x05, 0x07,        // Usage Page (Keyboard/Keypad)
    0x19, 0xE0,        // Usage Minimum (Left Control)
    0x29, 0xE7,        // Usage Maximum (Right GUI)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x08,        // Report Count (8)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x95, 0x01,        // Report Count (1)
    0x75, 0x08,        // Report Size (8)
    0x81, 0x01,        // Input (Cnst,Ary,Abs)
    0x95, 0x06,        // Report Count (6)
    0x75, 0x08,        // Report Size (8)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x65,        // Logical Maximum (101)
    0x05, 0x07,        // Usage Page (Keyboard/Keypad)
    0x19, 0x00,        // Usage Minimum (0)
    0x29, 0x65,        // Usage Maximum (101)
    0x81, 0x00,        // Input (Data,Ary,Abs)
    0xC0,              // End Collection
};

// HAL_USB_Device 实现
HAL_USB_Device* HAL_USB_Device::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HAL_USB_Device();
    }
    return instance_;
}

HAL_USB_Device::HAL_USB_Device() 
    : initialized_(false), connected_(false), cdc_rx_head_(0), cdc_rx_tail_(0) {
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
    
    // 初始化TinyUSB
    if (!tusb_init()) {
        return false;
    }
    
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

bool HAL_USB_Device::send_touch_report(const TouchPoint* points, uint8_t count) {
    if (!is_ready()) return false;
    
    // 构建触摸报告（简化版本）
    uint8_t report[64] = {0};
    report[0] = REPORTID_TOUCHPAD;
    
    size_t offset = 1;
    for (uint8_t i = 0; i < count && i < TOUCH_LOCAL_NUM; i++) {
        const TouchPoint& point = points[i];
        
        // 触摸状态字节
        uint8_t status = 0;
        if (point.pressed) {
            status |= 0x01;  // Tip Switch
            status |= 0x02;  // In Range
            status |= 0x04;  // Confidence
        }
        report[offset++] = status;
        
        // 触摸ID
        report[offset++] = point.id;
        
        // X坐标（16位）
        report[offset++] = point.x & 0xFF;
        report[offset++] = (point.x >> 8) & 0xFF;
        
        // Y坐标（16位）
        report[offset++] = point.y & 0xFF;
        report[offset++] = (point.y >> 8) & 0xFF;
    }
    
    // 触摸点数量
    report[offset++] = count;
    
    return tud_hid_report(0, report, offset);
}

bool HAL_USB_Device::send_keyboard_report(uint8_t report_id, const KeyboardReport& report) {
    if (!is_ready()) return false;
    
    uint8_t hid_report[9];  // 增加数组大小以容纳所有数据
    hid_report[0] = report_id;
    hid_report[1] = report.modifier;
    hid_report[2] = report.reserved;
    memcpy(&hid_report[3], report.keys, 6);
    
    return tud_hid_report(0, hid_report, 8);  // 只发送前8个字节
}

bool HAL_USB_Device::send_hid_report(uint8_t report_id, const uint8_t* data, uint8_t length) {
    if (!is_ready()) return false;
    
    return tud_hid_report(0, data, length);
}

bool HAL_USB_Device::configure_device(uint16_t vendor_id, uint16_t product_id, const std::string& manufacturer, const std::string& product, const std::string& serial) {
    // TinyUSB在运行时不支持动态配置设备描述符
    // 这些配置需要在编译时通过宏定义设置
    // 这里只是一个占位实现，返回true表示"配置成功"
    return true;
}

bool HAL_USB_Device::cdc_write(const uint8_t* data, size_t length) {
    if (!is_ready()) return false;
    
    return tud_cdc_write(data, length) == length;
}

size_t HAL_USB_Device::cdc_read(uint8_t* buffer, size_t max_length) {
    if (!initialized_) return 0;
    
    size_t count = 0;
    while (count < max_length && cdc_rx_head_ != cdc_rx_tail_) {
        buffer[count++] = cdc_rx_buffer_[cdc_rx_tail_];
        cdc_rx_tail_ = (cdc_rx_tail_ + 1) % CDC_BUFFER_SIZE;
    }
    
    return count;
}

size_t HAL_USB_Device::cdc_available() const {
    if (cdc_rx_head_ >= cdc_rx_tail_) {
        return cdc_rx_head_ - cdc_rx_tail_;
    } else {
        return CDC_BUFFER_SIZE - cdc_rx_tail_ + cdc_rx_head_;
    }
}

void HAL_USB_Device::cdc_flush() {
    if (initialized_) {
        tud_cdc_write_flush();
    }
}

void HAL_USB_Device::set_cdc_rx_callback(std::function<void(const uint8_t*, size_t)> callback) {
    cdc_rx_callback_ = callback;
}

void HAL_USB_Device::set_hid_get_report_callback(std::function<uint16_t(uint8_t, uint8_t, uint8_t*, uint16_t)> callback) {
    hid_get_report_callback_ = callback;
}

void HAL_USB_Device::set_hid_set_report_callback(std::function<void(uint8_t, uint8_t, const uint8_t*, uint16_t)> callback) {
    hid_set_report_callback_ = callback;
}

void HAL_USB_Device::task() {
    if (initialized_) {
        tud_task();
        handle_cdc_rx();
    }
}

void HAL_USB_Device::handle_cdc_rx() {
    if (!tud_cdc_available()) return;
    
    uint8_t buffer[64];
    uint32_t count = tud_cdc_read(buffer, sizeof(buffer));
    
    if (count > 0) {
        // 存储到环形缓冲区
        for (uint32_t i = 0; i < count; i++) {
            size_t next_head = (cdc_rx_head_ + 1) % CDC_BUFFER_SIZE;
            if (next_head != cdc_rx_tail_) {
                cdc_rx_buffer_[cdc_rx_head_] = buffer[i];
                cdc_rx_head_ = next_head;
            }
        }
        
        // 调用回调函数
        if (cdc_rx_callback_) {
            cdc_rx_callback_(buffer, count);
        }
    }
}

// TinyUSB回调函数实现
void HAL_USB_Device::tud_cdc_rx_cb(uint8_t itf) {
    if (instance_) {
        instance_->handle_cdc_rx();
    }
}

uint16_t HAL_USB_Device::tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    if (instance_ && instance_->hid_get_report_callback_) {
        return instance_->hid_get_report_callback_(report_id, (uint8_t)report_type, buffer, reqlen);
    }
    return 0;
}

void HAL_USB_Device::tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    if (instance_ && instance_->hid_set_report_callback_) {
        instance_->hid_set_report_callback_(report_id, (uint8_t)report_type, buffer, bufsize);
    }
}

// TinyUSB描述符回调函数
// 注意：tud_descriptor_device_cb和tud_hid_descriptor_report_cb已在Arduino框架中定义，避免重复定义
extern "C" {
    uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
        return HAL_USB_Device::tud_hid_get_report_cb(instance, report_id, report_type, buffer, reqlen);
    }
    
    void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
        HAL_USB_Device::tud_hid_set_report_cb(instance, report_id, report_type, buffer, bufsize);
    }
    
    void tud_cdc_rx_cb(uint8_t itf) {
        HAL_USB_Device::tud_cdc_rx_cb(itf);
    }
}