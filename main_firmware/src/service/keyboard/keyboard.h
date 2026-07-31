#pragma once

#include <cstdint>
#include "../../protocol/host_cmd/host_cmd.h"

/**
 * KeyboardService - 物理键盘(GPIO1-12) + 触控→键盘映射 → HID 键盘输出
 *
 * 职责:
 *  - 物理键: 读 GPIO1-12(1K 上拉直连键), 每键可选触发极性(config kbd.plNN, 默认 0=低电平触发)
 *    与独立防抖窗(config kbd.dbNN, 0..10000us, 默认 3000), 逐键去抖后经可配 HID 键码
 *    (config kbd.keyNN)驱动 HID::press_key/release_key。
 *  - 逻辑分析仪: 12 位掩码每次变化即带 time_us_32() 时间戳入定长环形缓冲(去抖前 raw + 仅去抖后 deb + 输出 out),
 *    主机经 KBD_GET_EDGES 主动拉取(无推流), 环满丢最旧并累加 overflow。
 *  - 触控→键盘: comm.keyboard_map_en 开启时, 把 34 位逻辑分区触摸(经 BindingService)
 *    经可配键码(config kbd.zoneNN)映射为 HID 键。
 *  - 每键长按语义: delay_ms(按住够久才真正输出) + max_hold_ms(输出后最长保持即自动抬起),
 *    12 个物理键与 34 个分区各自独立(config kbd.hdNN/kbd.mhNN, kbd.zhdNN/kbd.zmhNN)。
 *  - host_cmd: KBD_GET_STATE / KBD_GET_MAP / KBD_SET_MAP / KBD_GET_TOUCHMAP / KBD_SET_TOUCHMAP /
 *    KBD_GET_HOLD / KBD_SET_HOLD / KBD_GET_KEYCFG / KBD_SET_KEYCFG / KBD_GET_EDGES。
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

    // 单键去抖运行态。★逐键独立★: 原实现是"整字"判定(任一键跳变就重置全部 12 键的稳定窗),
    // 一个抖动键会连带推迟其它键的提交 —— 既是精度问题也是延迟隐患。
    struct DebounceState {
        uint32_t stable_since_us;  // 该键原始电平保持不变的起始时刻
        bool     raw_last;         // 该键上次原始采样(去抖前)
        void clear() { stable_since_us = 0; raw_last = false; }
    };

    static constexpr uint8_t KEY_COUNT = 12;   // GPIO1-12
    static constexpr uint8_t GPIO_BASE = 1;    // 第 0 键 = GPIO1
    static constexpr uint8_t ZONE_COUNT = 34;
    // 每键防抖窗上限(us)。UI 围栏与固件校验同源: 超限一律 NAK, 不静默夹取。
    static constexpr uint16_t DEBOUNCE_US_MAX = 10000;
    // 默认值 = 改造前的全局常量, 保证升级后行为不变。
    static constexpr uint16_t DEBOUNCE_US_DEFAULT = 3000;

public:
    // ★组合映射★: 一条 = "zone_mask 内的分区全部按下" → "keycode[] 全部同时输出"。
    // 取代了原来"每分区各自一个键"的模型(那只是 zone_mask 恰好一位的特例)。两套并存会让同一分区
    // 既属独立映射又属组合, 输出互相抢, 语义无法自洽, 故判定只走本表。
    static constexpr uint8_t COMBO_COUNT = 16;      // 映射条数上限(定长, 不动态分配)
    static constexpr uint8_t COMBO_KEY_COUNT = 4;   // 单条最多同时按下的键数
    // 协议单条字节数 = zone_mask(8) + keycode[COMBO_KEY_COUNT](4) + mod(1) + delay(2) + maxhold(2) = 17。
    // ★曾经写成 16★: 编解码(keyboard.cpp:_handle_get_combo/_handle_set_combo)按 17 字节逐字段读写,
    // 而长度校验用 16 ⇒ KBD_SET_COMBO 的 `frame.len != 1 + count*16` 永远不成立 → 整表写入被 NAK,
    // 组合映射从来没进过设备(实测漫灌后 kbd.cbA00 仍为 0)。常量必须与字段布局同源。
    static constexpr uint8_t COMBO_ENTRY_BYTES =
        8u + COMBO_KEY_COUNT + 1u + 2u + 2u;

private:
    struct ComboMap {
        uint64_t zone_mask;                   // 参与判定的分区集合(34 位); 0 = 该条为空
        uint8_t  keycode[COMBO_KEY_COUNT];    // 同时按下的 HID 键码(0 = 空位)
        uint8_t  mod;                         // 修饰位(bit0 LCtrl/1 LShift/2 LAlt/3 LGui)
        uint16_t delay_ms;                    // 全部分区按住需持续该时长才输出; 0=立即
        uint16_t max_hold_ms;                 // 输出后最长保持该时长即自动抬起; 0=不自动抬起
        void clear() {
            zone_mask = 0;
            for (uint8_t i = 0; i < COMBO_KEY_COUNT; i++) { keycode[i] = 0; }
            mod = 0;
            delay_ms = 0;
            max_hold_ms = 0;
        }
        bool empty() const { return zone_mask == 0; }
    };

    // ★逻辑分析仪边沿记录★ 一条 = "某一时刻 12 位掩码发生了变化"。
    // 同时留去抖前(raw)、仅去抖后(deb)与去抖+长按后(out)三态, 主机可分别对照防抖与长按的作用。
#pragma pack(push, 1)
    struct EdgeRec {
        uint32_t t_us;      // time_us_32() 原始时间戳(不做单位换算, 微秒级间隔必须可见)
        uint16_t raw_mask;  // 去抖前 12 位
        uint16_t deb_mask;  // 仅去抖后的 12 位
        uint16_t out_mask;  // 实际输出 HID 的 12 位
        void clear() { t_us = 0; raw_mask = 0; deb_mask = 0; out_mask = 0; }
    };
#pragma pack(pop)
    static_assert(sizeof(EdgeRec) == 10, "EdgeRec must remain 10 bytes");
    // 容量: 192 条 × 10B = 1920B 静态 RAM(定长, 无动态分配)。最坏覆盖时长 = 192 次掩码变化;
    // 以人手最快连击 ~20 边沿/秒计可覆盖 ~9.6s, 机械抖动集中爆发时(每次按下 3~5 个边沿)约 1~2s。
    // 主机以既有 tick(16ms)轮询拉取, 正常绝不会满; 满了也只丢最旧并累加 overflow, 让主机诚实显示丢失。
    static constexpr uint16_t EDGE_CAP = 192;
    // 单次 KBD_GET_EDGES 最多返回条数(96×10+9 = 969B, 远低于 payload 上限, 不挤占其它响应)。
    static constexpr uint8_t EDGE_READ_MAX = 96;

    // 定长环形缓冲。push/pop 全为 O(1) 且无取模除法, 不打日志、不碰 flash、不分配内存 ——
    // 主机不轮询时唯一开销就是"掩码没变则一次比较后返回", 对按键延迟的影响可忽略。
    struct EdgeRing {
        EdgeRec  rec[EDGE_CAP];
        uint16_t tail;      // 最旧记录下标
        uint16_t count;     // 当前条数
        uint32_t overflow;  // 累计因环满被丢弃的条数(只增不减, 主机取差值)
        void clear() {
            for (uint16_t i = 0; i < EDGE_CAP; i++) { rec[i].clear(); }
            tail = 0; count = 0; overflow = 0;
        }
        inline void push(uint32_t t_us, uint16_t raw_mask, uint16_t deb_mask, uint16_t out_mask) {
            if (count >= EDGE_CAP) {
                tail++;
                if (tail >= EDGE_CAP) tail = 0;
                count--;
                overflow++;
            }
            uint16_t idx = tail + count;
            if (idx >= EDGE_CAP) idx -= EDGE_CAP;
            rec[idx].t_us = t_us;
            rec[idx].raw_mask = raw_mask;
            rec[idx].deb_mask = deb_mask;
            rec[idx].out_mask = out_mask;
            count++;
        }
        inline bool pop(EdgeRec& out_rec) {
            if (count == 0) return false;
            out_rec = rec[tail];
            tail++;
            if (tail >= EDGE_CAP) tail = 0;
            count--;
            return true;
        }
    };

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
    HoldCfg   _zone_hold[ZONE_COUNT];    // 分区长按参数(kbd.zhdNN / kbd.zmhNN, 旧模型保留供迁移读取)
    HoldState _hold_st[KEY_COUNT];       // 物理键时间状态机
    HoldState _zone_hold_st[ZONE_COUNT]; // 分区时间状态机(旧模型, 判定已不使用)

    ComboMap  _combo[COMBO_COUNT];       // 组合映射表(kbd.cbA/B/K/T%02u)
    HoldState _combo_st[COMBO_COUNT];    // 每条组合各自的时间状态机
    // 上次已输出的键码集合(去重后), 用于差分 press/release —— 多条组合可能含同一键码,
    // 逐条 release 会把仍被别条按住的键误抬起, 故按"键码引用计数"聚合后再驱动 HID。
    uint8_t   _combo_out_keys[COMBO_COUNT * COMBO_KEY_COUNT];
    uint8_t   _combo_out_count;
    uint8_t   _combo_out_mod;

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
    uint64_t _touch_active;              // 上次驱动键盘的分区 mask(用于差分 press/release)
    bool     _gpio_ready;

    // 每键触发极性掩码: bit i = 1 → 该键"高电平触发"; 0 → 低电平触发(默认, 与改造前一致)。
    // 用掩码而非 bool 数组: _read_raw() 热路径里只做一次异或即可完成 12 键极性归一。
    uint16_t _pol_high_mask;
    uint16_t _debounce_us[KEY_COUNT];    // 每键防抖窗(us, 0=不去抖, 上限 DEBOUNCE_US_MAX)
    DebounceState _deb[KEY_COUNT];       // 每键去抖运行态
    EdgeRing _edges;                     // 边沿记录环(逻辑分析仪)
    uint16_t _edge_last_raw;             // 上次入队时的 raw 掩码(仅变化才入队)
    uint16_t _edge_last_deb;             // 上次入队时的仅去抖掩码
    uint16_t _edge_last_out;             // 上次入队时的 out 掩码

    uint16_t _read_raw() const;                          // 读 GPIO1-12, 按极性归一 → 1=按下
    void _apply_pulls();                                 // 按极性设置内部上/下拉
    void _apply_phys(uint8_t idx, bool pressed);         // 物理键 idx → HID
    void _apply_touch(uint64_t area_mask);               // 触控分区差分 → HID
    void _load_combo();                                  // 从 ConfigManager 读 64 个打包 KV
    void _store_combo() const;                           // 写回 ConfigManager(由 SAVE_CONFIG 落 flash)
    void _apply_combo(uint64_t area_raw, uint32_t now_us); // 组合判定 + 聚合去重 + 差分驱动 HID

    // 组合表里是否有任何有效条目。空表时 task() 回落到 per-zone 判定, 保证存量配置不因升级失效。
    inline bool _combo_active() const {
        for (uint8_t i = 0; i < COMBO_COUNT; i++) {
            if (!_combo[i].empty()) return true;
        }
        return false;
    }

    // ComboMap 的时间参数复用 HoldCfg 语义, 直接借用同一个 _hold_eval, 不再写第二份计时逻辑。
    static inline HoldCfg _combo_hold_cfg(const ComboMap& cm) {
        HoldCfg cfg;
        cfg.delay_ms = cm.delay_ms;
        cfg.max_hold_ms = cm.max_hold_ms;
        return cfg;
    }

    static void _handle_get_state(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_keycfg(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_keycfg(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_edges(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_get_combo(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
    static void _handle_set_combo(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len);
};
