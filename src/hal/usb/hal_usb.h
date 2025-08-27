#pragma once

#include "hal_usb_types.h"
#include "hal_usb_hid.h"
#include <tusb.h>
#include <class/hid/hid_device.h>

// TinyUSB回调函数声明
extern "C" {
    uint8_t const* tud_descriptor_device_cb(void);
    uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance);
    uint8_t const* tud_descriptor_configuration_cb(uint8_t index);
    uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);
    uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
    void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);
    void tud_cdc_rx_cb(uint8_t itf);
    void tud_mount_cb(void);
    void tud_umount_cb(void);
    void tud_suspend_cb(bool remote_wakeup_en);
    void tud_resume_cb(void);
}


class HAL_USB {
public:
    virtual ~HAL_USB() = default;
    
    // 初始化USB接口
    virtual bool init() = 0;
    
    // 释放USB资源
    virtual void deinit() = 0;
    
    // 检查USB是否连接
    virtual bool is_connected() const = 0;
    
    // 检查USB是否就绪
    virtual bool is_ready() const = 0;
    
    // HID功能
    virtual bool send_hid_report(HID_ReportID report_id, const uint8_t* data, size_t length) = 0;

    // CDC功能
    virtual bool cdc_write(const uint8_t* data, size_t length) = 0;
    virtual size_t cdc_read(uint8_t* buffer, size_t max_length) = 0;
    virtual size_t cdc_available() const = 0;
    virtual void cdc_flush() = 0;

    // 获取实例名称
    virtual std::string get_name() const = 0;
};

// USB实现类
class HAL_USB_Device : public HAL_USB {
public:
    static HAL_USB_Device* getInstance();
    ~HAL_USB_Device();
    
    bool init() override;
    void deinit() override;
    bool is_connected() const override;
    bool is_ready() const override;
    
    bool cdc_write(const uint8_t* data, size_t length) override;
    size_t cdc_read(uint8_t* buffer, size_t max_length) override;
    size_t cdc_available() const override;
    void cdc_flush() override;
    std::string get_name() const override { return USB_DEVICE_NAME; }
    
    static void tud_cdc_rx_cb(uint8_t itf);
    
    //  HID 报告接口
    __force_inline bool send_hid_report(HID_ReportID report_id, const uint8_t* data, size_t length) override {
        return tud_hid_report(report_id, data, length);
    };

private:
    bool initialized_;
    bool connected_;
    
    // 移除回调函数成员变量
    
    // CDC缓冲区
    static const size_t CDC_BUFFER_SIZE = 1024;
    uint8_t cdc_rx_buffer_[CDC_BUFFER_SIZE];
    size_t cdc_rx_head_;
    size_t cdc_rx_tail_;
    
    // 内部方法
    void handle_cdc_rx();
    void handle_hid_requests();
    
    // 静态实例指针（用于回调）
    static HAL_USB_Device* instance_;
    
    // 私有构造函数（单例模式）
    HAL_USB_Device();
    HAL_USB_Device(const HAL_USB_Device&) = delete;
    HAL_USB_Device& operator=(const HAL_USB_Device&) = delete;
};