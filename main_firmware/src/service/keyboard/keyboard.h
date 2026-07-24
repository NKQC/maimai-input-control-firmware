#pragma once

#include <cstdint>
#include "../../protocol/host_cmd/host_cmd.h"

/**
 * KeyboardService - 物理键盘(GPIO1-12) + 触控→键盘映射 → HID 键盘输出
 *
 * 职责:
 *  - 物理键: 读 GPIO1-12(1K 上拉, active-low 直连键), 整字去抖, 每键经可配 HID 键码
 *    (config kbd.keyNN)驱动 HID::press_key/release_key。
 *  - 触控→键盘: comm.keyboard_map_en 开启时, 把 34 位逻辑分区触摸(经 BindingService)
 *    经可配键码(config kbd.zoneNN)映射为 HID 键。
 *  - host_cmd: KBD_GET_STATE / KBD_GET_MAP / KBD_SET_MAP / KBD_GET_TOUCHMAP / KBD_SET_TOUCHMAP。
 *
 * HID 在两种 USB 模式下均枚举(serial 也带键盘 HID), 故本服务在任意模式都能输出键。
 */
class KeyboardService {
public:
    static KeyboardService* getInstance();

    // GPIO 初始化 + 从 ConfigManager 载入键码/开关 + 注册 host_cmd handler
    void init();

    // 从 ConfigManager 重新读取 kbd.keyNN / kbd.zoneNN / comm.keyboard_map_en
    void reload_map();

    // 每轮主循环调用: 读物理键(去抖) + 触控→键盘 + 驱动并 task HID
    void task();

    // 物理键实时按下位(bit i = GPIO(1+i) 按下), 供 host 上报
    uint16_t phys_state() const { return _phys_state; }

private:
    static constexpr uint8_t KEY_COUNT = 12;   // GPIO1-12
    static constexpr uint8_t GPIO_BASE = 1;    // 第 0 键 = GPIO1
    static constexpr uint8_t ZONE_COUNT = 34;
    static constexpr uint32_t DEBOUNCE_US = 3000;

    KeyboardService();
    KeyboardService(const KeyboardService&) = delete;
    KeyboardService& operator=(const KeyboardService&) = delete;

    static KeyboardService* _instance;

    uint8_t  _keycode[KEY_COUNT];        // 物理键 HID 键码(0=不映射)
    uint8_t  _keymod[KEY_COUNT];         // 物理键修饰位(bit0 LCtrl/1 LShift/2 LAlt/3 LGui)
    uint8_t  _zone_keycode[ZONE_COUNT];  // 触控分区 HID 键码(0=不映射)
    uint8_t  _zone_mod[ZONE_COUNT];      // 触控分区修饰位
    bool     _kbd_map_en;                // 触控→键盘 总开关

    // 按 modifier 位图 press/release 修饰键(LCtrl..LGui = HID 0xE0..0xE3)。
    static void _apply_mods(uint8_t mod, bool pressed);

    uint16_t _phys_state;                // 已去抖的 12 位按下态
    uint16_t _raw_last;                  // 上次原始采样
    uint32_t _raw_stable_since_us;       // 原始采样保持不变的起始时刻
    uint64_t _touch_active;              // 上次驱动键盘的分区 mask(用于差分 press/release)
    bool     _gpio_ready;

    uint16_t _read_raw() const;                          // 读 GPIO1-12, active-low → 1=按下
    void _apply_phys(uint8_t idx, bool pressed);         // 物理键 idx → HID
    void _apply_touch(uint64_t area_mask);               // 触控分区差分 → HID

    static void _handle_get_state(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
};
