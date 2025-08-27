#include "hal_usb.h"
#include <pico/stdlib.h>
#include <tusb.h>
#include <device/usbd.h>
#include <class/cdc/cdc_device.h>
#include <class/hid/hid_device.h>
#include <pico/stdio.h>
#include <pico/bootrom.h>
#include <hardware/irq.h>
#include <cstring>


// 静态实例指针
HAL_USB_Device* HAL_USB_Device::instance_ = nullptr;

// 字符串描述符缓冲区
static uint16_t desc_str[32];

// USB描述符
static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
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

// 配置描述符 - CDC + HID复合设备
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN)

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x83
#define EPNUM_HID         0x84

#define BOARD_TUD_RHPORT 0

static const uint8_t desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 5, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_descriptor), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 1)
};

// 字符串描述符
static const char* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: 支持的语言是英语 (0x0409)
    "Mai Control",                 // 1: 制造商
    "Mai Control Device",          // 2: 产品
    USB_SERIAL,                    // 3: 序列号
    "Mai Control CDC",             // 4: CDC接口
    "Mai Control HID",             // 5: HID接口
};

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

// HID报告描述符回调 - 使用自定义描述符
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return hid_report_descriptor;
}

// 配置描述符回调
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index; // 对于单一配置
    return desc_configuration;
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
    if (tud_connected())    tud_disconnect();
    // 初始化TinyUSB设备栈
    if(!tud_init(BOARD_TUD_RHPORT)) return false;
    
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

bool HAL_USB_Device::cdc_write(const uint8_t* data, size_t length) {
    if (!is_ready() || !data || length == 0) {
        return length == 0;
    }
    
    static uint64_t last_avail_time = 0;
    const uint64_t TIMEOUT_US = 10000; // 10ms timeout
    
    size_t total_written = 0;
    uint64_t start_time = time_us_64();
    
    while (total_written < length) {
        uint32_t available = tud_cdc_write_available();
        
        if (available > 0) {
            size_t to_write = std::min((size_t)available, length - total_written);
            uint32_t written = tud_cdc_write(data + total_written, to_write);
            
            if (written > 0) {
                total_written += written;
                last_avail_time = time_us_64();
                tud_cdc_write_flush();
            }
        } else {
            // No space available, check timeout
            uint64_t current_time = time_us_64();
            if (current_time - start_time > TIMEOUT_US || 
                (last_avail_time > 0 && current_time - last_avail_time > TIMEOUT_US)) {
                break; // Timeout reached
            }
            
            // Brief delay and flush
            sleep_us(100);
            tud_cdc_write_flush();
        }
    }
    
    return total_written == length;
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
    }
}

// TinyUSB回调函数实现
void HAL_USB_Device::tud_cdc_rx_cb(uint8_t itf) {
    if (instance_) {
        instance_->handle_cdc_rx();
    }
}
