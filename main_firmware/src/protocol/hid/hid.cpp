#include "hid.h"
#include "src/hal/usb/hal_usb.h"
#include <cstring>
#include <pico/time.h>

// 单例实例
HID* HID::instance_ = nullptr;

// 私有构造函数
HID::HID() 
    : initialized_(false), hal_usb_(nullptr), report_count_(0), last_report_time_(0), cached_report_rate_(0),
      kbd_send_count_(0), kbd_send_fail_(0), kbd_reports_used_(0),
      keyboard_needs_send_(false), touch_needs_send_(false), last_keyboard_send_(0), last_touch_send_(0) {
    keyboard_state.clear();
}

// 析构函数
HID::~HID() {
    deinit();
}

// 获取单例实例
HID* HID::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new HID();
    }
    return instance_;
}

// 初始化
bool HID::init(HAL_USB* hal_usb) {
    if (initialized_ || !hal_usb) {
        return false;
    }
    
    hal_usb_ = hal_usb;
    report_count_ = 0;
    last_report_time_ = to_ms_since_boot(get_absolute_time());
    cached_report_rate_ = 0;
    
    // 按键状态现在由HAL层直接管理
    
    initialized_ = true;
    return true;
}

// 反初始化
void HID::deinit() {
    if (!initialized_) {
        return;
    }
    
    initialized_ = false;
    hal_usb_ = nullptr;
    report_count_ = 0;
    last_report_time_ = 0;
    cached_report_rate_ = 0;
}

// 检查是否已初始化
bool HID::is_initialized() const {
    return initialized_;
}

// 按下按键
bool HID::press_key(HID_KeyCode key) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    bool result = keyboard_state.add(key);
    if (result && keyboard_state.has_state_changed()) {
        keyboard_needs_send_ = true;
    }
    return result;
}

// 释放按键
bool HID::release_key(HID_KeyCode key) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    bool result = keyboard_state.remove(key);
    if (result && keyboard_state.has_state_changed()) {
        keyboard_needs_send_ = true;
    }
    return result;
}

// 清空键盘状态并发送空报文
void HID::clear_keyboard_state() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    keyboard_state.clear();
    keyboard_needs_send_ = true;
    
    // 立即发送空报文确保主机知道所有按键都已释放
    force_send_keyboard_report();
}

// 强制发送键盘报文
void HID::force_send_keyboard_report() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    report_keyboard();
    keyboard_needs_send_ = false;
    keyboard_state.update_last_state();
    last_keyboard_send_ = us_to_ms(time_us_64());
}

// 获取实际回报速率
uint32_t HID::get_report_rate() const {
    return cached_report_rate_;
}

// 触摸报告发送
bool HID::send_touch_report(const HID_TouchPoint& report) {
    if (!initialized_ || !hal_usb_) {
        return false;
    }
    
    bool result = false;
    if (report.press) {
        // 按下触摸点
        result = touch_state.press(report);
    } else {
        // 松开触摸点
        result = touch_state.release(report.id);
    }
    
    if (result) {
        touch_needs_send_ = true;
    }
    
    return result;
}

// 强制发送触摸报文
void HID::force_send_touch_report() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    uint32_t _now = us_to_ms(time_us_64());
    report_touch(_now);
    touch_needs_send_ = false;
    last_touch_send_ = _now;
}

// 把当前仍按下的触点全部转成抬起事件, 由下一次 task() 统一发出。
// ★不在这里直接发★: 调用点在主循环热路径(输出抑制判定)上, 发报文要走 USB 端点;
// 交给既有的 task() 触发式发送路径, 与按下/抬起走同一个出口, 不产生第二条发送时序。
void HID::release_all_touch() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    if (touch_state.down_count() == 0) {
        return;
    }
    touch_state.release_all();
    touch_needs_send_ = true;
}

uint8_t HID::touch_down_count() const {
    return touch_state.down_count();
}

void HID::report_keyboard() {
    // HID键盘报文格式: [modifier][reserved][key1][key2][key3][key4][key5][key6]
    // 总共8字节，符合标准HID键盘报文格式
    static uint8_t keyboard_report[8];   // 8B 缓冲留静态, 避免每次进栈
    uint8_t keyboard_enum = 0;
    uint8_t keys_to_send = keyboard_state.key_count;
    uint8_t key_index = 0;
    
    // 如果没有按键按下且没有修饰键，发送空报文
    if (keyboard_state.key_count == 0 && keyboard_state.modifier_keys == 0) {
        memset(keyboard_report, 0, 8);
        // ★必须清掉上一轮用过的**每一个**集合★ NKRO 靠 3 个 report id 各摊 6 键(见 keyboard_id[]),
        // 原先只向 keyboard_id[0] 发一份空报文 —— 于是当上一轮铺到了 keyboard2/3 时, 那两个集合
        // 里按着的键再也收不到抬起报文, 在主机端永久卡住。
        const uint8_t clear_count = (kbd_reports_used_ > 0u) ? kbd_reports_used_ : 1u;
        for (uint8_t i = 0; i < clear_count && i < KEYBOARD_NUM; i++) {
            report_keyboard_send(keyboard_id[i], keyboard_report);
        }
        kbd_reports_used_ = 0u;
        return;
    }
    
    // 分批发送按键，每个报文最多6个按键
    do {
        // 清空报文缓冲区
        memset(keyboard_report, 0, 8);
        
        // 第0字节：修饰键状态
        keyboard_report[0] = keyboard_state.modifier_keys;
        // 第1字节：保留字节
        keyboard_report[1] = 0;
        
        // 第2-7字节：最多6个按键码，避免发送0x00按键
        uint8_t report_key_count = 0;
        for (uint8_t i = 0; i < 6 && key_index < keyboard_state.key_count; i++) {
            if (keyboard_state.key[key_index] != HID_KeyCode::KEY_NONE && 
                static_cast<uint8_t>(keyboard_state.key[key_index]) != 0x00) {
                keyboard_report[2 + i] = static_cast<uint8_t>(keyboard_state.key[key_index]);
                report_key_count++;
            }
            key_index++;
        }
        
        // 发送报文
        report_keyboard_send(keyboard_id[keyboard_enum], keyboard_report);
        
        keyboard_enum++;
        keys_to_send -= report_key_count;
        
    } while (keys_to_send > 0 && keyboard_enum < KEYBOARD_NUM);
    
    // ★收缩时补清★ 上一轮铺到了更多集合(例如按了 8 键用掉 2 个, 本轮只剩 3 键只用 1 个),
    // 多出来的那些集合若不显式清空, 它们的旧键会留在主机端。与上面的全松开分支同一个道理。
    for (uint8_t i = keyboard_enum; i < kbd_reports_used_ && i < KEYBOARD_NUM; i++) {
        memset(keyboard_report, 0, 8);
        report_keyboard_send(keyboard_id[i], keyboard_report);
    }
    kbd_reports_used_ = keyboard_enum;
    // 原先此处还有一个 "只有修饰键且 keyboard_enum == 0" 的兜底分支: do-while 至少执行一次,
    // keyboard_enum 退出时恒 >= 1, 该条件永假 —— modifier-only 早已由循环体正确发出。已删。
}

void HID::report_touch(uint32_t _now) {
    static uint8_t TouchData[9];

    // 摘要字段用**当前并发触点数**(down_count), 不是"本轮事件数"。
    // 边沿上报下每轮通常只有 1 个事件, 用事件数会让 36 指同按也恒报 1、抬起恒报 0。
    const uint8_t contact_count = touch_state.down_count();

    // 处理按下的触摸点
    for (uint8_t i = 0; i < touch_state.press_modifier; i++) {
        const HID_TouchPoint& report = touch_state.touch_press[i];
        
        // 构建触摸报文
        TouchData[0] = 1;                           // Press状态 (1=按下)
        TouchData[1] = report.id;                   // 触摸点ID
        TouchData[2] = report.x & 0xFF;             // X坐标低字节
        TouchData[3] = (report.x >> 8) & 0xFF;      // X坐标高字节
        TouchData[4] = report.y & 0xFF;             // Y坐标低字节
        TouchData[5] = (report.y >> 8) & 0xFF;      // Y坐标高字节
        TouchData[6] = (_now * 10) & 0xFF;          // 扫描时间低字节
        TouchData[7] = ((_now * 10) >> 8) & 0xFF;   // 扫描时间高字节
        TouchData[8] = contact_count;               // 当前并发触点数
        
        hal_usb_->send_hid_report(HID_ReportID::REPORT_ID_TOUCHSCREEN, TouchData, 9);
    }
    
    // 处理松开的触摸点
    for (uint8_t i = 0; i < touch_state.release_modifier; i++) {
        const HID_TouchPoint& report = touch_state.touch_release[i];
        
        // 构建触摸松开报文
        TouchData[0] = 0;                           // Press状态 (0=松开)
        TouchData[1] = report.id;                   // 触摸点ID
        TouchData[2] = 0;                           // X坐标清零
        TouchData[3] = 0;                           // X坐标清零
        TouchData[4] = 0;                           // Y坐标清零
        TouchData[5] = 0;                           // Y坐标清零
        TouchData[6] = 0;                           // 扫描时间清零
        TouchData[7] = 0;                           // 扫描时间清零
        TouchData[8] = contact_count;               // 剩余并发触点数(全抬起时为 0)
        
        hal_usb_->send_hid_report(HID_ReportID::REPORT_ID_TOUCHSCREEN, TouchData, 9);
    }
    
    // ★两个队列都是"本轮待发事件", 发完无条件清空★
    // 谁还按着由 touch_state.down_* 持久记录, 与这两个队列无关(见 hid.h 的语义说明);
    // 旧实现把 press_modifier 兼作"按下集合", 清空后 release() 再也找不到该 id ⇒ 抬起报文永不发出。
    touch_state.press_modifier = 0;
    touch_state.release_modifier = 0;
}

void HID::task() {
    if (!initialized_ || !hal_usb_) {
        return;
    }
    
    static uint32_t _now = 0;
    _now = us_to_ms(time_us_64());
    
    // 触发式发送键盘报文 - 只在状态变化时发送
    if ((keyboard_needs_send_ || keyboard_state.has_state_changed())) {
        report_keyboard();
        keyboard_needs_send_ = false;
        keyboard_state.update_last_state();
        last_keyboard_send_ = _now;
        report_count_++;
    }
    
    // 触发式发送触摸报文 - 只在有触摸事件时发送
    if ((touch_needs_send_ || 
         touch_state.press_modifier > 0 || 
         touch_state.release_modifier > 0)) {
        report_touch(_now);
        touch_needs_send_ = false;
        last_touch_send_ = _now;
        report_count_++;
    }
    
    // 统计报文发送速率
    if (_now - last_report_time_ >= 1000) {  // 每秒更新一次
        cached_report_rate_ = report_count_;
        report_count_ = 0;
        last_report_time_ = _now;
    }
}
