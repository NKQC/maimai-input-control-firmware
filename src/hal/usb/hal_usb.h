#pragma once

#include <stdint.h>
#include <string>
#include <functional>
#include <tusb.h>

/**
 * HAL层 - USB接口抽象类
 * 提供底层USB接口，支持HID和CDC功能
 * 参考WashingTouch的USB实现
 */

// USB设备描述符配置
#ifndef USB_VID
#define USB_VID                 0x0CA3  // 参考WashingTouch
#endif
#ifndef USB_PID
#define USB_PID                 0x0024  // 参考WashingTouch
#endif
#define USB_SERIAL              "123456789"
#define USB_DEVICE_NAME         "HAL_USB_Device"

// HID报告ID
#define REPORTID_TOUCHPAD       0x01
#define REPORTID_KEYBOARD_1     0x02
#define REPORTID_KEYBOARD_2     0x03

// 触摸屏配置
#define TOUCH_LOCAL_NUM         10      // 最大触摸点数
#define TOUCH_SCREEN_WIDTH      1920
#define TOUCH_SCREEN_HEIGHT     1080

// 触摸点数据结构
struct TouchPoint {
    uint8_t id;         // 触摸点ID
    uint16_t x;         // X坐标
    uint16_t y;         // Y坐标
    bool pressed;       // 是否按下
};

// 键盘数据结构
struct KeyboardReport {
    uint8_t modifier;   // 修饰键
    uint8_t reserved;   // 保留字节
    uint8_t keys[6];    // 按键码
};

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
    virtual bool send_touch_report(const TouchPoint* points, uint8_t count) = 0;
    virtual bool send_keyboard_report(uint8_t report_id, const KeyboardReport& report) = 0;
    virtual bool send_hid_report(uint8_t report_id, const uint8_t* data, uint8_t length) = 0;
    
    // 设备配置
    virtual bool configure_device(uint16_t vendor_id, uint16_t product_id, const std::string& manufacturer = "", const std::string& product = "", const std::string& serial = "") = 0;
    
    // CDC功能
    virtual bool cdc_write(const uint8_t* data, size_t length) = 0;
    virtual size_t cdc_read(uint8_t* buffer, size_t max_length) = 0;
    virtual size_t cdc_available() const = 0;
    virtual void cdc_flush() = 0;
    
    // 设置回调函数
    virtual void set_cdc_rx_callback(std::function<void(const uint8_t*, size_t)> callback) = 0;
    virtual void set_hid_get_report_callback(std::function<uint16_t(uint8_t, uint8_t, uint8_t*, uint16_t)> callback) = 0;
    virtual void set_hid_set_report_callback(std::function<void(uint8_t, uint8_t, const uint8_t*, uint16_t)> callback) = 0;
    
    // 获取实例名称
    virtual std::string get_name() const = 0;
    
    // 任务处理（需要在主循环中调用）
    virtual void task() = 0;
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
    bool send_touch_report(const TouchPoint* points, uint8_t count) override;
    bool send_keyboard_report(uint8_t report_id, const KeyboardReport& report) override;
    bool send_hid_report(uint8_t report_id, const uint8_t* data, uint8_t length) override;
    bool configure_device(uint16_t vendor_id, uint16_t product_id, const std::string& manufacturer = "", const std::string& product = "", const std::string& serial = "") override;
    bool cdc_write(const uint8_t* data, size_t length) override;
    size_t cdc_read(uint8_t* buffer, size_t max_length) override;
    size_t cdc_available() const override;
    void cdc_flush() override;
    void set_cdc_rx_callback(std::function<void(const uint8_t*, size_t)> callback) override;
    void set_hid_get_report_callback(std::function<uint16_t(uint8_t, uint8_t, uint8_t*, uint16_t)> callback) override;
    void set_hid_set_report_callback(std::function<void(uint8_t, uint8_t, const uint8_t*, uint16_t)> callback) override;
    std::string get_name() const override { return USB_DEVICE_NAME; }
    void task() override;
    
    // TinyUSB回调函数 - 需要public访问权限
    static void tud_cdc_rx_cb(uint8_t itf);
    static uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
    static void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);

private:
    bool initialized_;
    bool connected_;
    
    // 回调函数
    std::function<void(const uint8_t*, size_t)> cdc_rx_callback_;
    std::function<uint16_t(uint8_t, uint8_t, uint8_t*, uint16_t)> hid_get_report_callback_;
    std::function<void(uint8_t, uint8_t, const uint8_t*, uint16_t)> hid_set_report_callback_;
    
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