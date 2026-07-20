#pragma once

#include "hal_usb_types.h"
#include "hal_usb_hid.h"
#include <tusb.h>
#include <class/hid/hid_device.h>
#include <class/vendor/vendor_device.h>

// TinyUSB回调函数声明
extern "C" {
    uint8_t const* tud_descriptor_device_cb(void);
    uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance);
    uint8_t const* tud_descriptor_configuration_cb(uint8_t index);
    uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);
    uint8_t const* tud_descriptor_bos_cb(void);
    uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
    void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);
    void tud_cdc_rx_cb(uint8_t itf);
    void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize);
    bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request);
    void tud_mount_cb(void);
    void tud_umount_cb(void);
    void tud_suspend_cb(bool remote_wakeup_en);
    void tud_resume_cb(void);
}


// USB 工作模式（对应 config 配置项 mode.work：0=Serial,1=HID）。
// 恒定枚举的 config（vendor/WinUSB）不受此影响；仅决定 serial+light 二 CDC 与 HID 谁出现。
// 注：不能用裸名 SERIAL/HID 作枚举值——Arduino ArduinoCore-API/api/Common.h 把
// SERIAL 宏定义为 0x0，会在预处理阶段替换掉枚举名导致语法错误。
enum class UsbWorkMode : uint8_t {
    WORK_SERIAL = 0,
    WORK_HID = 1
};

// 两个 CDC 角色（config 已改为 vendor/WinUSB，不再占用 CDC 实例，见 usb-winusb-config）。
// 取值即为 TinyUSB 的 cdc itf 序号（按 TUD_CDC_DESCRIPTOR 在配置描述符里出现的先后顺序
// 分配，与 bInterfaceNumber 无关）：serial/light 仅在 Serial 模式描述符中出现，为 itf0/itf1。
// 同样避开裸名 SERIAL（Arduino 宏冲突）。
enum class UsbCdcPort : uint8_t {
    CDC_SERIAL = 0,
    CDC_LIGHT = 1,
    CDC_PORT_COUNT_ = 2
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
    virtual bool send_hid_report(HID_ReportID report_id, const uint8_t* data, size_t length) = 0;

    // config 通道（恒定枚举的 vendor/WinUSB 接口，供 host_cmd/UsbComm 复用）。
    virtual bool config_write(const uint8_t* data, size_t length) = 0;
    virtual size_t config_read(uint8_t* buffer, size_t max_length) = 0;
    virtual size_t config_available() const = 0;
    virtual void config_flush() = 0;

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
    
    bool config_write(const uint8_t* data, size_t length) override;
    size_t config_read(uint8_t* buffer, size_t max_length) override;
    size_t config_available() const override;
    void config_flush() override;
    std::string get_name() const override { return USB_DEVICE_NAME; }

    // Flush pending IN data, pump completion events, then remove the RP2040 D+ pull-up.
    // The caller owns the detached hold interval before reset/BOOTSEL entry.
    void soft_disconnect();

    // flash 写会禁中断/禁 XIP 数十 ms，host 可能在此期间超时导致 vendor 端点失步。
    // flash 写完成后调用本函数主动断开+重连 D+，强制 host 重新枚举，恢复 vendor 通信。
    void reconnect();

    // 泵 TinyUSB 设备任务。因 platformio.ini 排除了 core 的 RP2040USB.cpp，
    // 框架不再后台调度 tud_task；必须在主循环中持续调用它来完成枚举与 CDC/vendor 收发。
    void task();

    // ★vendor OUT 自愈★：已 arm 的 OUT 端点恒处于 busy(有挂起传输等待 host 数据)。
    // flash 擦写禁中断窗口可能使 RP2040 dcd 丢失该 OUT 完成中断→vendord_xfer_cb 不再触发
    // →OUT 永不重新 arm→host→device 永久失步(仅 EP0/枚举存活，正是 flash 写后 vendor 死的现象)。
    // 主循环调用本函数：若 mounted 且 OUT 非 busy(=掉出 arm 态)则重新武装，恢复接收。正常态为空操作。
    void vendor_service();

    // 按 CDC 角色（serial/light）独立读写。Serial 模式外，对应的 TinyUSB itf 未被枚举，
    // 读写在此静默失败/空转，由上层（mai2serial/mai2light 路由）在调用前自行判断当前 mode。
    bool cdc_write(UsbCdcPort port, const uint8_t* data, size_t length);
    size_t cdc_read(UsbCdcPort port, uint8_t* buffer, size_t max_length);
    size_t cdc_available(UsbCdcPort port) const;
    void cdc_flush(UsbCdcPort port);

    static void tud_cdc_rx_cb(uint8_t itf);
    static void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize);

    //  HID 报告接口
    __force_inline bool send_hid_report(HID_ReportID report_id, const uint8_t* data, size_t length) override {
        return tud_hid_report(report_id, data, length);
    };

private:
    bool initialized_;
    bool connected_;

    // config（vendor）通道的接收环形缓冲，由 tud_vendor_rx_cb 填充。
    static const size_t CONFIG_BUFFER_SIZE = 1024;
    uint8_t config_rx_buffer_[CONFIG_BUFFER_SIZE];
    size_t config_rx_head_;
    size_t config_rx_tail_;

    // CDC缓冲区：按角色(serial/light)各自一份环形缓冲，避免混流。
    static const size_t CDC_BUFFER_SIZE = 1024;
    static const size_t CDC_PORT_COUNT = static_cast<size_t>(UsbCdcPort::CDC_PORT_COUNT_);
    uint8_t cdc_rx_buffer_[CDC_PORT_COUNT][CDC_BUFFER_SIZE];
    size_t cdc_rx_head_[CDC_PORT_COUNT];
    size_t cdc_rx_tail_[CDC_PORT_COUNT];

    // 内部方法
    void _handle_cdc_rx(uint8_t itf);
    void _handle_vendor_rx(uint8_t const* buffer, uint16_t bufsize);

    // 静态实例指针（用于回调）
    static HAL_USB_Device* instance_;
    
    // 私有构造函数（单例模式）
    HAL_USB_Device();
    HAL_USB_Device(const HAL_USB_Device&) = delete;
    HAL_USB_Device& operator=(const HAL_USB_Device&) = delete;
};