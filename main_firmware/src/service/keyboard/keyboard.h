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
 *  - 每键长按语义: delay_ms(按住够久才真正输出) + max_hold_ms(输出后最长保持即自动抬起),
 *    12 个物理键与 34 个分区各自独立(config kbd.hdNN/kbd.mhNN, kbd.zhdNN/kbd.zmhNN)。
 *  - host_cmd: KBD_GET_STATE / KBD_GET_MAP / KBD_SET_MAP / KBD_GET_TOUCHMAP / KBD_SET_TOUCHMAP /
 *    KBD_GET_HOLD / KBD_SET_HOLD。
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
    // 单键"长按延迟触发 + 最长按住自动抬起"配置(毫秒, 0=该项禁用)。物理键与分区共用此结构。
    struct HoldCfg {
        uint16_t delay_ms;      // 按下需持续该时长才真正输出 HID; 0=立即输出
        uint16_t max_hold_ms;   // 已输出后最长保持该时长即自动 release; 0=不自动抬起
        void clear() { delay_ms = 0; max_hold_ms = 0; }
    };

    // 单键时间状态机运行态。自动抬起后 expired 置位, 需物理/触摸松开才重新走一遍延迟流程。
    struct HoldState {
        uint32_t down_us;   // 本次按下(上游去抖后)的起始时刻
        uint32_t out_us;    // 真正输出 HID 的时刻(max_hold 的起算点)
        bool     down;      // 上游当前按下
        bool     out;       // 当前是否正在输出 HID
        bool     expired;   // 已因 max_hold 自动抬起, 等松开
        void clear() { down_us = 0; out_us = 0; down = false; out = false; expired = false; }
    };

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

    HoldCfg   _hold[KEY_COUNT];          // 物理键长按参数(kbd.hdNN / kbd.mhNN)
    HoldCfg   _zone_hold[ZONE_COUNT];    // 分区长按参数(kbd.zhdNN / kbd.zmhNN)
    HoldState _hold_st[KEY_COUNT];       // 物理键时间状态机
    HoldState _zone_hold_st[ZONE_COUNT]; // 分区时间状态机

    // 长按/自动抬起时间状态机(物理键与分区共用): 返回该键当前应输出的 HID 按下态。
    static inline bool _hold_eval(HoldState& st, const HoldCfg& cfg, bool raw_down, uint32_t now_us) {
        if (!raw_down) { st.clear(); return false; }
        if (!st.down) { st.down = true; st.down_us = now_us; }
        if (st.expired) return false;
        if (!st.out) {
            if (cfg.delay_ms != 0 && ((now_us - st.down_us) / 1000u) < cfg.delay_ms) return false;
            st.out = true;
            st.out_us = now_us;
        }
        if (cfg.max_hold_ms != 0 && ((now_us - st.out_us) / 1000u) >= cfg.max_hold_ms) {
            st.out = false;
            st.expired = true;   // 自动抬起后不再重按, 直到松开
            return false;
        }
        return true;
    }

    // 按 modifier 位图 press/release 修饰键(LCtrl..LGui = HID 0xE0..0xE3)。
    static void _apply_mods(uint8_t mod, bool pressed);

    uint16_t _phys_state;                // 已去抖的 12 位按下态
    uint16_t _phys_out;                  // 当前实际输出 HID 的 12 位(经长按状态机)
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
    static void _handle_get_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
};
