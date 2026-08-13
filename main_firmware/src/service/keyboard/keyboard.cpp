#include "keyboard.h"
#include "../config_manager/config_manager.h"
#include "../game_io/game_io.h"
#include "../binding_service/binding_service.h"
#include "../sensor_link/sensor_link.h"
#include "../../protocol/hid/hid.h"
#include "../../protocol/psoc/psoc.h"
#include "../../hal/usb/hal_usb.h"
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <cstdio>

KeyboardService* KeyboardService::_instance = nullptr;

KeyboardService::KeyboardService()
    : _kbd_map_en(false), _kbd_map_serial_only(false), _phys_state(0), _phys_out(0),
      _touch_active(0), _gpio_ready(false), _pol_high_mask(0),
      _boot_level_mask(0), _boot_level_valid(false),
      _edge_last_raw(0), _edge_last_deb(0), _edge_last_out(0) {
    _diag.clear();
    _edges.clear();
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        _pol_cfg[i] = 2;   // 默认 AUTO
        _keycode[i] = 0; _keymod[i] = 0;
        _hold[i].clear(); _hold_st[i].clear();
        _debounce_us[i] = DEBOUNCE_US_DEFAULT; _deb[i].clear();
    }
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        _zone_keycode[z] = 0; _zone_mod[z] = 0;
        _zone_hold[z].clear(); _zone_hold_st[z].clear();
    }
}

void KeyboardService::_apply_mods(uint8_t mod, bool pressed) {
    HID* hid = HID::getInstance();
    for (uint8_t b = 0; b < 4; b++) {
        if ((mod >> b) & 1u) {
            const HID_KeyCode k = static_cast<HID_KeyCode>(0xE0u + b);  // LCtrl/LShift/LAlt/LGui
            if (pressed) hid->press_key(k); else hid->release_key(k);
        }
    }
}

KeyboardService* KeyboardService::getInstance() {
    if (_instance == nullptr) _instance = new KeyboardService();
    return _instance;
}

void KeyboardService::init() {
    // GPIO1-12: 输入; 内部上/下拉按每键极性设置(见 _apply_pulls)。
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        const uint8_t pin = GPIO_BASE + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }
    _gpio_ready = true;

    reload_map();

    HostCmdDispatcher* dispatcher = HostCmdDispatcher::getInstance();
    dispatcher->register_handler(HostCmd::KBD_GET_STATE, _handle_get_state);
    dispatcher->register_handler(HostCmd::KBD_GET_MAP, _handle_get_map);
    dispatcher->register_handler(HostCmd::KBD_SET_MAP, _handle_set_map);
    dispatcher->register_handler(HostCmd::KBD_GET_TOUCHMAP, _handle_get_touchmap);
    dispatcher->register_handler(HostCmd::KBD_SET_TOUCHMAP, _handle_set_touchmap);
    dispatcher->register_handler(HostCmd::KBD_GET_HOLD, _handle_get_hold);
    dispatcher->register_handler(HostCmd::KBD_SET_HOLD, _handle_set_hold);
    dispatcher->register_handler(HostCmd::KBD_GET_COMBO, _handle_get_combo);
    dispatcher->register_handler(HostCmd::KBD_SET_COMBO, _handle_set_combo);
    dispatcher->register_handler(HostCmd::KBD_GET_KEYCFG, _handle_get_keycfg);
    dispatcher->register_handler(HostCmd::KBD_SET_KEYCFG, _handle_set_keycfg);
    dispatcher->register_handler(HostCmd::KBD_GET_EDGES, _handle_get_edges);
}

void KeyboardService::reload_map() {
    char key_buf[16];
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        snprintf(key_buf, sizeof(key_buf), "kbd.key%02u", i);
        _keycode[i] = ConfigManager::get_uint8(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.km%02u", i);
        _keymod[i] = ConfigManager::get_uint8(key_buf);
        // 长按参数: ConfigManager 存 uint16, 语义上限 65535ms。
        snprintf(key_buf, sizeof(key_buf), "kbd.hd%02u", i);
        _hold[i].delay_ms = ConfigManager::get_uint16(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.mh%02u", i);
        _hold[i].max_hold_ms = ConfigManager::get_uint16(key_buf);
        // 触发极性配置态: 0=低电平触发 / 1=高电平触发 / 2=AUTO(默认)。
        // 这里只存配置, 解析成生效掩码交给 _resolve_pol()(AUTO 需要启动电平参与)。
        snprintf(key_buf, sizeof(key_buf), "kbd.pl%02u", i);
        _pol_cfg[i] = ConfigManager::get_uint8(key_buf);
        // 每键防抖窗。KV 已带 0..DEBOUNCE_US_MAX 围栏, 这里再夹一次防止旧配置文件带入越界值。
        snprintf(key_buf, sizeof(key_buf), "kbd.db%02u", i);
        const uint16_t db = ConfigManager::get_uint16(key_buf);
        _debounce_us[i] = (db > DEBOUNCE_US_MAX) ? DEBOUNCE_US_MAX : db;
    }
    // 配置态 → 生效掩码(AUTO 用开机那次采样解析)。必须在 _apply_pulls() 之前。
    _resolve_pol();
    // 极性变了要跟着换内部上/下拉, 否则空闲电平与判定口径相反。
    if (_gpio_ready) _apply_pulls();
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        snprintf(key_buf, sizeof(key_buf), "kbd.zone%02u", z);
        _zone_keycode[z] = ConfigManager::get_uint8(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.zm%02u", z);
        _zone_mod[z] = ConfigManager::get_uint8(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.zhd%02u", z);
        _zone_hold[z].delay_ms = ConfigManager::get_uint16(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.zmh%02u", z);
        _zone_hold[z].max_hold_ms = ConfigManager::get_uint16(key_buf);
    }
    _kbd_map_en = ConfigManager::get_bool("comm.keyboard_map_en");
    _kbd_map_serial_only = ConfigManager::get_bool("comm.keyboard_map_serial_only");
    _load_combo();
}

// 组合表持久化: 每条打包成 4 个 uint32 KV。逐字段一个 KV 会新增 144 项, 明显放大 JSON 与 CRC 开销;
// 打包后仅 64 项, 且省掉字符串解析。位域布局在 _load/_store 两侧必须严格镜像, 故写在一处注释里:
//   cbA = zone_mask[31:0]
//   cbB = bit0..1: zone_mask[33:32]; bit8..15: mod
//   cbK = key0 | key1<<8 | key2<<16 | key3<<24
//   cbT = delay_ms | max_hold_ms<<16
void KeyboardService::_load_combo() {
    char key_buf[16];
    for (uint8_t i = 0; i < COMBO_COUNT; i++) {
        ComboMap& cm = _combo[i];
        cm.clear();
        snprintf(key_buf, sizeof(key_buf), "kbd.cbA%02u", i);
        const uint32_t a = ConfigManager::get_uint32(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.cbB%02u", i);
        const uint32_t b = ConfigManager::get_uint32(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.cbK%02u", i);
        const uint32_t k = ConfigManager::get_uint32(key_buf);
        snprintf(key_buf, sizeof(key_buf), "kbd.cbT%02u", i);
        const uint32_t t = ConfigManager::get_uint32(key_buf);
        cm.zone_mask = (uint64_t)a | ((uint64_t)(b & 0x3u) << 32);
        cm.mod = (uint8_t)((b >> 8) & 0xFFu);
        for (uint8_t n = 0; n < COMBO_KEY_COUNT; n++) {
            cm.keycode[n] = (uint8_t)((k >> (8u * n)) & 0xFFu);
        }
        cm.delay_ms = (uint16_t)(t & 0xFFFFu);
        cm.max_hold_ms = (uint16_t)((t >> 16) & 0xFFFFu);
        // 掩码为空的条目一律整条归零, 避免"没有分区却留着键码"的半条目被后续误用。
        if (cm.zone_mask == 0) cm.clear();
    }
    for (uint8_t i = 0; i < COMBO_COUNT; i++) { _combo_st[i].clear(); }
    _combo_out_count = 0;
    _combo_out_mod = 0;
}

void KeyboardService::_store_combo() const {
    char key_buf[16];
    for (uint8_t i = 0; i < COMBO_COUNT; i++) {
        const ComboMap& cm = _combo[i];
        uint32_t k = 0;
        for (uint8_t n = 0; n < COMBO_KEY_COUNT; n++) {
            k |= (uint32_t)cm.keycode[n] << (8u * n);
        }
        snprintf(key_buf, sizeof(key_buf), "kbd.cbA%02u", i);
        ConfigManager::set_uint32(key_buf, (uint32_t)(cm.zone_mask & 0xFFFFFFFFu));
        snprintf(key_buf, sizeof(key_buf), "kbd.cbB%02u", i);
        ConfigManager::set_uint32(key_buf,
            (uint32_t)((cm.zone_mask >> 32) & 0x3u) | ((uint32_t)cm.mod << 8));
        snprintf(key_buf, sizeof(key_buf), "kbd.cbK%02u", i);
        ConfigManager::set_uint32(key_buf, k);
        snprintf(key_buf, sizeof(key_buf), "kbd.cbT%02u", i);
        ConfigManager::set_uint32(key_buf,
            (uint32_t)cm.delay_ms | ((uint32_t)cm.max_hold_ms << 16));
    }
}

// 内部上/下拉按极性设置: 低电平触发→上拉(空闲高), 高电平触发→下拉(空闲低)。
// ★注意★ 硬件板上另有 1K 外部上拉, 强度远高于内部 ~50K; 高电平触发只对"外部主动驱动"的接法有效,
// 直连按键的板子把某键设成高电平触发会读到常按。这是接线口径问题, 固件不做隐式纠正。
void KeyboardService::_apply_pulls() {
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        const uint8_t pin = GPIO_BASE + i;
        if (((_pol_high_mask >> i) & 1u) != 0) {
            gpio_pull_down(pin);
        } else {
            gpio_pull_up(pin);
        }
    }
}

// 配置态(_pol_cfg, 含 AUTO) + 启动电平 → 生效掩码 _pol_high_mask。
// AUTO 语义: 把**启动时**的电平当作该键的"抬起"电平 ⇒
//   启动电平为高 → 抬起是高 → 按下必然是低 → 低电平触发(清位);
//   启动电平为低 → 抬起是低 → 按下必然是高 → 高电平触发(置位)。
// 还没采到启动电平时回落"清位"(= 改造前的默认低电平触发), 该窗口只存在于首次 task() 之前。
void KeyboardService::_resolve_pol() {
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        const uint16_t bit = (uint16_t)(1u << i);
        bool high;
        switch (_pol_cfg[i]) {
            case 1:  high = true; break;
            case 2:  high = _boot_level_valid && (((_boot_level_mask >> i) & 1u) == 0); break;
            default: high = false; break;   // 0 与任何越界值都按低电平触发处理
        }
        if (high) _pol_high_mask |= bit; else _pol_high_mask &= (uint16_t)~bit;
    }
}

// 采样一次启动电平并重解析极性 + 重设上下拉。**只在首次 task() 调用一次, 此后永不重采**。
// 直读 GPIO 原始电平(不经 _read_raw(), 那会先套一遍极性, 拿到的就不是电平了)。
void KeyboardService::_latch_boot_level() {
    uint16_t level = 0;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (gpio_get(GPIO_BASE + i) != 0) level |= (uint16_t)(1u << i);
    }
    _boot_level_mask = level;
    _boot_level_valid = true;
    _resolve_pol();
    _apply_pulls();
}

uint16_t KeyboardService::_read_raw() const {
    uint16_t level = 0;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (gpio_get(GPIO_BASE + i) != 0) level |= (uint16_t)(1u << i);
    }
    // 按位极性归一: 高电平触发的键直接取电平, 低电平触发的键取反。
    // 一次异或完成 12 键, 热路径不引入分支。
    const uint16_t all = (uint16_t)((1u << KEY_COUNT) - 1u);
    return (uint16_t)((level ^ (uint16_t)~_pol_high_mask) & all);
}

void KeyboardService::_apply_phys(uint8_t idx, bool pressed) {
    const uint8_t code = _keycode[idx];
    const uint8_t mod = _keymod[idx];
    if (code == 0 && mod == 0) return;
    HID* hid = HID::getInstance();
    if (pressed) {
        _apply_mods(mod, true);
        if (code != 0) hid->press_key(static_cast<HID_KeyCode>(code));
    } else {
        if (code != 0) hid->release_key(static_cast<HID_KeyCode>(code));
        _apply_mods(mod, false);
    }
}

void KeyboardService::_apply_touch(uint64_t area_mask) {
    const uint64_t changed = area_mask ^ _touch_active;
    if (changed == 0) return;
    HID* hid = HID::getInstance();
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        if (((changed >> z) & 1ULL) == 0) continue;
        const uint8_t code = _zone_keycode[z];
        const uint8_t mod = _zone_mod[z];
        if (code == 0 && mod == 0) continue;
        const bool pressed = ((area_mask >> z) & 1ULL) != 0;
        if (pressed) {
            _apply_mods(mod, true);
            if (code != 0) hid->press_key(static_cast<HID_KeyCode>(code));
        } else {
            if (code != 0) hid->release_key(static_cast<HID_KeyCode>(code));
            _apply_mods(mod, false);
        }
    }
    _touch_active = area_mask;
}

void KeyboardService::_apply_combo(uint64_t area_raw, uint32_t now_us) {
    // 1) 先算出"本轮应当按下的键码集合 + 修饰位并集"。
    //    多条组合可能含同一键码, 必须先聚合去重再与上一轮差分 —— 否则逐条 release 会把
    //    仍被别条按住的键误抬起(表现为按住一个组合时另一个组合松手就整体失效)。
    uint8_t want_keys[COMBO_COUNT * COMBO_KEY_COUNT];
    uint8_t want_count = 0;
    uint8_t want_mod = 0;
    for (uint8_t i = 0; i < COMBO_COUNT; i++) {
        const ComboMap& cm = _combo[i];
        const bool raw_down = !cm.empty() && ((area_raw & cm.zone_mask) == cm.zone_mask);
        if (!_hold_eval(_combo_st[i], _combo_hold_cfg(cm), raw_down, now_us)) continue;
        want_mod |= cm.mod;
        for (uint8_t k = 0; k < COMBO_KEY_COUNT; k++) {
            const uint8_t code = cm.keycode[k];
            if (code == 0) continue;
            bool dup = false;
            for (uint8_t n = 0; n < want_count; n++) {
                if (want_keys[n] == code) { dup = true; break; }
            }
            if (!dup) { want_keys[want_count++] = code; }
        }
    }

    // 2) 与上一轮差分驱动 HID。集合规模 ≤ 64, 两层线性比对即可, 无需额外容器。
    HID* hid = HID::getInstance();
    for (uint8_t n = 0; n < _combo_out_count; n++) {
        bool still = false;
        for (uint8_t m = 0; m < want_count; m++) {
            if (want_keys[m] == _combo_out_keys[n]) { still = true; break; }
        }
        if (!still) hid->release_key(static_cast<HID_KeyCode>(_combo_out_keys[n]));
    }
    for (uint8_t m = 0; m < want_count; m++) {
        bool had = false;
        for (uint8_t n = 0; n < _combo_out_count; n++) {
            if (_combo_out_keys[n] == want_keys[m]) { had = true; break; }
        }
        if (!had) hid->press_key(static_cast<HID_KeyCode>(want_keys[m]));
    }
    if (want_mod != _combo_out_mod) {
        // 只动变化位: 松开已撤销的修饰, 按下新增的修饰。
        const uint8_t released = (uint8_t)(_combo_out_mod & ~want_mod);
        const uint8_t pressed = (uint8_t)(want_mod & ~_combo_out_mod);
        if (released != 0) _apply_mods(released, false);
        if (pressed != 0) _apply_mods(pressed, true);
        _combo_out_mod = want_mod;
    }
    for (uint8_t m = 0; m < want_count; m++) { _combo_out_keys[m] = want_keys[m]; }
    _combo_out_count = want_count;
}

void KeyboardService::task() {
    if (!_gpio_ready) return;

    // ★AUTO 只认启动时电平★ 采样点放在首次 task() 而不是 init():
    // init() 里刚刚配好上下拉, 线路未必稳定; 而首次 task() 之前已经历阻塞式 PSoC bring-up(秒级),
    // 线路早已稳定, 因此**不需要任何 delay**。采样时引脚保持 init() 设的上拉, 与板上 1K 外部上拉同向。
    // 已知取舍: 启动瞬间被按住的键会被学成"抬起", 该键极性判反 —— 这是 AUTO 的指定语义, 不做纠正。
    if (!_boot_level_valid) { _latch_boot_level(); }

    // 物理键逐键去抖 + 逐键长按判定。两件事合进同一个 12 次循环: 每键各自的稳定窗互不干扰,
    // 一个抖动键不再拖累其它键的提交时刻。debounce_us=0 时 (now - stable_since) >= 0 恒成立 → 不去抖。
    const uint32_t now = time_us_32();
    const uint16_t raw = _read_raw();
    uint16_t phys_out = 0;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        const bool bit = ((raw >> i) & 1u) != 0;
        DebounceState& db = _deb[i];
        if (bit != db.raw_last) {
            db.raw_last = bit;
            db.stable_since_us = now;
        }
        const uint16_t mask = (uint16_t)(1u << i);
        if (bit != (((_phys_state & mask) != 0))
            && (uint32_t)(now - db.stable_since_us) >= (uint32_t)_debounce_us[i]) {
            if (bit) _phys_state |= mask; else _phys_state &= (uint16_t)~mask;
        }
        // 去抖后的按下态再过一遍每键长按状态机(延迟触发 / 最长按住自动抬起)。
        if (_hold_eval(_hold_st[i], _hold[i], (_phys_state & mask) != 0, now)) {
            phys_out |= mask;
        }
    }

    // ★逻辑分析仪入队★ 仅在三路掩码任一变化时记一条(边沿触发, 不是定时采样) ⇒ 稳态零流量、零开销。
    // push 是 O(1) 纯内存写: 无 malloc / 无日志 / 无 flash / 无锁, 因此不影响按键输出延迟与 HID 上报。
    if (raw != _edge_last_raw || _phys_state != _edge_last_deb || phys_out != _edge_last_out) {
        _edge_last_raw = raw;
        _edge_last_deb = _phys_state;
        _edge_last_out = phys_out;
        _edges.push(now, raw, _phys_state, phys_out);
    }

    if (phys_out != _phys_out) {
        const uint16_t changed = phys_out ^ _phys_out;
        for (uint8_t i = 0; i < KEY_COUNT; i++) {
            if (((changed >> i) & 1u) != 0) {
                _apply_phys(i, ((phys_out >> i) & 1u) != 0);
            }
        }
        _phys_out = phys_out;
    }

    // ★总开关与协议发送限定开关必须低频重读★
    // 两项均由主机经 CFG_SET 修改，而 CFG_SET 只落 ConfigManager、不会通知本服务。
    // 这里每 200ms 查一次本地 config map(纯内存查找, 不涉及 USB/SPI, 不影响任何轮询周期)。
    static uint32_t last_en_poll_us = 0;
    if ((uint32_t)(now - last_en_poll_us) > 200000u) {
        last_en_poll_us = now;
        _kbd_map_en = ConfigManager::get_bool("comm.keyboard_map_en");
        _kbd_map_serial_only = ConfigManager::get_bool("comm.keyboard_map_serial_only");
    }

    // 映射是否生效: 总开关为纲; 若“仅协议发送时生效”开启, 再叠加 mai2serial 真实发送态。
    // 总开关关闭时本修饰开关不起作用 —— 直接不生效, 与改造前一致。
    bool map_active = _kbd_map_en;
    if (map_active && _kbd_map_serial_only) {
        map_active = GameIoService::getInstance()->mai2_touch_sending();
    }
    // 触控→键盘(组合语义): 不生效时 area_raw 保持 0，继续走同一全松开路径确保释放已按下的映射键。
    uint64_t area_raw = 0;
    // 扫描会话期间该通道的 gain/div 被逐格改写, 掩码是无意义值 → 按全松开处理(见 output_suppressed)。
    if (map_active && SensorLink::getInstance()->output_suppressed()) map_active = false;
    if (map_active) {
        Psoc* psoc = Psoc::getInstance();
        // 门禁用 touch_hold_ok()(保留窗口内仍可信), 而非瞬时 link_ok(): 后者会在错帧的那一拍
        // 把映射键判成松开, 表现为按住不动却连点。
        if (psoc->touch_hold_ok()) {
            area_raw = BindingService::getInstance()->map_to_areas(psoc->touch_mask());
        }
    }
    // ★组合表为空时必须回落到旧的 per-zone 判定★
    // 组合表是新增能力, 但存量设备的 flash 里只有 kbd.zoneNN 那套 per-zone 映射。若无条件只走
    // 组合表, 升级固件后用户已配好的触控映射会**整体失效**(表空 → 一个键都不输出), 而界面上映射
    // 明明还在, 极难自查。故: 有组合条目就用组合语义(它是 per-zone 的超集), 一条都没有才回落。
    if (_combo_active()) {
        // 逐条组合判定: 掩码内分区**全部**按下才算这条按下, 任一松开即 raw_down=false ——
        // _hold_eval 在 !raw_down 时 clear(), 于是"全部达标后才开始计时、有一个不符合就算松开"
        // 两条语义都由既有状态机天然满足, 不需要另写计时逻辑。
        _apply_combo(area_raw, now);
    } else {
        // 回落路径与改造前完全一致: 每分区独立过一遍长按状态机再差分驱动 HID。
        uint64_t area_out = 0;
        for (uint8_t z = 0; z < ZONE_COUNT; z++) {
            if (_hold_eval(_zone_hold_st[z], _zone_hold[z], ((area_raw >> z) & 1ULL) != 0, now)) {
                area_out |= (1ULL << z);
            }
        }
        _apply_touch(area_out);
    }

    // ★链路诊断快照(纯读, 不参与上面任何判定)★ 逐环取实测值而不是复用上面短路后的结果:
    // 上面的 map_active 一旦在前一环就被否掉, 后面几环根本没被求值 —— 而排查恰恰需要知道
    // "后面那些环本来是什么值"。这些访问器全是无副作用的成员读(见各自声明), 放在这里安全。
    {
        Psoc* diag_psoc = Psoc::getInstance();
        uint8_t flags = 0;
        if (_kbd_map_en) flags |= DIAG_MAP_EN;
        if (_kbd_map_serial_only) flags |= DIAG_SERIAL_ONLY;
        if (GameIoService::getInstance()->mai2_touch_sending()) flags |= DIAG_MAI2_SENDING;
        if (SensorLink::getInstance()->output_suppressed()) flags |= DIAG_SUPPRESSED;
        if (diag_psoc->touch_hold_ok()) flags |= DIAG_TOUCH_HOLD;
        if (map_active) flags |= DIAG_MAP_ACTIVE;
        if (_combo_active()) flags |= DIAG_COMBO_ACTIVE;
        if (HID::getInstance()->is_initialized()) flags |= DIAG_HID_INIT;
        _diag.flags = flags;
        _diag.touch_mask = diag_psoc->touch_mask();
        _diag.area_raw = area_raw;
        _diag.bound_zones = BindingService::getInstance()->bound_zone_count();
    }

    HID::getInstance()->task();
}

void KeyboardService::_handle_get_state(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_STATE);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // [phys_state(u16 LE), raw(u16 LE), out(u16 LE)]。
    // ★尾部追加而非新命令★: 旧上位机只读前 2 字节, 兼容不变; 新上位机据 raw/out 直接看出
    // "去抖前 / 实际输出" 两态, 不必再猜防抖与长按到底生效没有。
    r.payload[0] = (uint8_t)(self->_phys_state & 0xFF);
    r.payload[1] = (uint8_t)((self->_phys_state >> 8) & 0xFF);
    r.payload[2] = (uint8_t)(self->_edge_last_raw & 0xFF);
    r.payload[3] = (uint8_t)((self->_edge_last_raw >> 8) & 0xFF);
    r.payload[4] = (uint8_t)(self->_phys_out & 0xFF);
    r.payload[5] = (uint8_t)((self->_phys_out >> 8) & 0xFF);
    // ★第二段尾部追加: 触控→键盘链路诊断★(同 raw/out 那次追加的先例, 只读前 6 字节的
    // 旧上位机逐字节不受影响)。布局固定, 变长的键码集合放最后:
    //   [6]  diag_ver = 1(以后再追加就 +1, 上位机据此判断能读到哪些字段)
    //   [7]  flags(见 keyboard.h DIAG_*)
    //   [8]  combo_out_count       [9]  bound_zones
    //   [10..17] touch_mask(u64 LE)  [18..25] area_raw(u64 LE)
    //   [26..29] hid 键盘报文实发数(u32 LE)  [30..33] 发送失败数(u32 LE)
    //   [34..]   combo 输出键码 × combo_out_count
    uint16_t off = 6;
    r.payload[off++] = 1;
    r.payload[off++] = self->_diag.flags;
    r.payload[off++] = self->_combo_out_count;
    r.payload[off++] = self->_diag.bound_zones;
    for (uint8_t b = 0; b < 8; b++) {
        r.payload[off++] = (uint8_t)((self->_diag.touch_mask >> (8u * b)) & 0xFFu);
    }
    for (uint8_t b = 0; b < 8; b++) {
        r.payload[off++] = (uint8_t)((self->_diag.area_raw >> (8u * b)) & 0xFFu);
    }
    HID* hid = HID::getInstance();
    const uint32_t sent = hid->kbd_send_count();
    const uint32_t failed = hid->kbd_send_fail();
    for (uint8_t b = 0; b < 4; b++) {
        r.payload[off++] = (uint8_t)((sent >> (8u * b)) & 0xFFu);
    }
    for (uint8_t b = 0; b < 4; b++) {
        r.payload[off++] = (uint8_t)((failed >> (8u * b)) & 0xFFu);
    }
    for (uint8_t n = 0; n < self->_combo_out_count; n++) {
        r.payload[off++] = self->_combo_out_keys[n];
    }
    r.len = off;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

// ============================================================================
// 每键触发极性 + 独立防抖 (KBD_GET_KEYCFG / KBD_SET_KEYCFG) 与边沿记录 (KBD_GET_EDGES)
// ============================================================================

void KeyboardService::_handle_get_keycfg(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_KEYCFG);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // [count(u8)=12] + 12×(pol(u8), debounce_us(u16 LE)) + resolved_pol_high_mask(u16 LE)
    r.payload[0] = KEY_COUNT;
    uint16_t off = 1;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        // 回显**配置态**(0/1/2), 不是解析结果: AUTO 必须原样回显, 否则界面永远看不到 AUTO 档。
        r.payload[off++] = self->_pol_cfg[i];
        r.payload[off++] = (uint8_t)(self->_debounce_us[i] & 0xFF);
        r.payload[off++] = (uint8_t)((self->_debounce_us[i] >> 8) & 0xFF);
    }
    // ★尾部追加而非新命令★(与 _handle_get_state 同口径): 旧上位机只读前面的 1+12×3 字节仍正确,
    // 新上位机据此显示 AUTO 到底判成了高还是低 —— 不然用户看不出 AUTO 学到了什么。
    r.payload[off++] = (uint8_t)(self->_pol_high_mask & 0xFF);
    r.payload[off++] = (uint8_t)((self->_pol_high_mask >> 8) & 0xFF);
    r.len = off;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_set_keycfg(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // 每项 4 字节: [idx, pol(0=低/1=高/2=AUTO), debounce_us(u16 LE)]。
    if (frame.len < 4 || (frame.len % 4) != 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_keycfg payload must be [idx,pol,debounce16] quads", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    // ★先全量校验再落值★ 非法值一律 NAK 且不写入任何一项(不静默夹取, 也不写半张表):
    // 半写会让界面显示的与设备实际生效的不一致, 排障时最难发现。
    for (uint16_t off = 0; off + 3 < frame.len; off += 4) {
        const uint8_t idx = frame.payload[off];
        const uint8_t pol = frame.payload[off + 1];
        const uint16_t db = (uint16_t)(frame.payload[off + 2] | ((uint16_t)frame.payload[off + 3] << 8));
        if (idx >= KEY_COUNT) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "kbd_set_keycfg idx out of range (0..11)", resp, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        if (pol > 2) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "kbd_set_keycfg pol must be 0(low) 1(high) or 2(auto)", resp, HOST_CMD_RESP_BUF_MAX);
            return;
        }
        if (db > DEBOUNCE_US_MAX) {
            *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
                "kbd_set_keycfg debounce_us must be 0..10000", resp, HOST_CMD_RESP_BUF_MAX);
            return;
        }
    }
    char key_buf[16];
    for (uint16_t off = 0; off + 3 < frame.len; off += 4) {
        const uint8_t idx = frame.payload[off];
        snprintf(key_buf, sizeof(key_buf), "kbd.pl%02u", idx);
        ConfigManager::set_uint8(key_buf, frame.payload[off + 1]);
        snprintf(key_buf, sizeof(key_buf), "kbd.db%02u", idx);
        ConfigManager::set_uint16(key_buf,
            (uint16_t)(frame.payload[off + 2] | ((uint16_t)frame.payload[off + 3] << 8)));
    }
    // 只写 RAM 影子并即时下发生效; flash 落地统一由 SAVE_CONFIG 触发(保护 flash 寿命)。
    getInstance()->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_get_edges(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    // 请求 [max(u8)] 可选; 0 或缺省 = EDGE_READ_MAX。
    uint8_t want = EDGE_READ_MAX;
    if (frame.len >= 1 && frame.payload[0] != 0 && frame.payload[0] < EDGE_READ_MAX) {
        want = frame.payload[0];
    }
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_EDGES);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // [cap(u16 LE), overflow(u32 LE), remaining(u16 LE), count(u8)] + count×(t_us u32 LE, raw u16 LE, deb u16 LE, out u16 LE)
    // overflow 是累计值(只增), 主机取差值即可知道"这段时间丢了多少条", 不做清零以免丢失计数。
    r.payload[0] = (uint8_t)(EDGE_CAP & 0xFF);
    r.payload[1] = (uint8_t)((EDGE_CAP >> 8) & 0xFF);
    const uint32_t ovf = self->_edges.overflow;
    r.payload[2] = (uint8_t)(ovf & 0xFF);
    r.payload[3] = (uint8_t)((ovf >> 8) & 0xFF);
    r.payload[4] = (uint8_t)((ovf >> 16) & 0xFF);
    r.payload[5] = (uint8_t)((ovf >> 24) & 0xFF);
    uint16_t off = 9;   // payload[6..7]=remaining, payload[8]=count, 取完再回填
    uint8_t sent = 0;
    EdgeRec rec;
    while (sent < want && self->_edges.pop(rec)) {
        r.payload[off++] = (uint8_t)(rec.t_us & 0xFF);
        r.payload[off++] = (uint8_t)((rec.t_us >> 8) & 0xFF);
        r.payload[off++] = (uint8_t)((rec.t_us >> 16) & 0xFF);
        r.payload[off++] = (uint8_t)((rec.t_us >> 24) & 0xFF);
        r.payload[off++] = (uint8_t)(rec.raw_mask & 0xFF);
        r.payload[off++] = (uint8_t)((rec.raw_mask >> 8) & 0xFF);
        r.payload[off++] = (uint8_t)(rec.deb_mask & 0xFF);
        r.payload[off++] = (uint8_t)((rec.deb_mask >> 8) & 0xFF);
        r.payload[off++] = (uint8_t)(rec.out_mask & 0xFF);
        r.payload[off++] = (uint8_t)((rec.out_mask >> 8) & 0xFF);
        sent++;
    }
    r.payload[6] = (uint8_t)(self->_edges.count & 0xFF);
    r.payload[7] = (uint8_t)((self->_edges.count >> 8) & 0xFF);
    r.payload[8] = sent;
    r.len = off;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_get_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_MAP);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // 每键 2 字节: [keycode, modifier]。
    r.payload[0] = KEY_COUNT;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        r.payload[1 + i * 2] = self->_keycode[i];
        r.payload[2 + i * 2] = self->_keymod[i];
    }
    r.len = 1 + KEY_COUNT * 2;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_set_map(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // 每项 3 字节: [idx, keycode, modifier]。
    if (frame.len < 3 || (frame.len % 3) != 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_map payload must be [idx,keycode,mod] triples", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    char key_buf[16];
    for (uint16_t off = 0; off + 2 < frame.len; off += 3) {
        const uint8_t idx = frame.payload[off];
        const uint8_t code = frame.payload[off + 1];
        const uint8_t mod = frame.payload[off + 2];
        if (idx >= KEY_COUNT) continue;
        snprintf(key_buf, sizeof(key_buf), "kbd.key%02u", idx);
        ConfigManager::set_uint8(key_buf, code);
        snprintf(key_buf, sizeof(key_buf), "kbd.km%02u", idx);
        ConfigManager::set_uint8(key_buf, mod);
    }
    // 只写 RAM 影子并即时下发生效; flash 落地统一由 SAVE_CONFIG 触发(保护 flash 寿命)。
    getInstance()->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_get_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_HOLD);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // [phys_count, zone_count] + 12×(delay u16 LE, maxhold u16 LE) + 34×(同上)。
    r.payload[0] = KEY_COUNT;
    r.payload[1] = ZONE_COUNT;
    uint16_t off = 2;
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        r.payload[off++] = (uint8_t)(self->_hold[i].delay_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_hold[i].delay_ms >> 8) & 0xFF);
        r.payload[off++] = (uint8_t)(self->_hold[i].max_hold_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_hold[i].max_hold_ms >> 8) & 0xFF);
    }
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        r.payload[off++] = (uint8_t)(self->_zone_hold[z].delay_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_zone_hold[z].delay_ms >> 8) & 0xFF);
        r.payload[off++] = (uint8_t)(self->_zone_hold[z].max_hold_ms & 0xFF);
        r.payload[off++] = (uint8_t)((self->_zone_hold[z].max_hold_ms >> 8) & 0xFF);
    }
    r.len = off;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_set_hold(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // 每项 6 字节: [kind(0=物理键/1=分区), idx, delay_ms(u16 LE), maxhold_ms(u16 LE)]。
    if (frame.len < 6 || (frame.len % 6) != 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_hold payload must be [kind,idx,delay16,maxhold16] sextets", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    char key_buf[16];
    for (uint16_t off = 0; off + 5 < frame.len; off += 6) {
        const uint8_t kind = frame.payload[off];
        const uint8_t idx = frame.payload[off + 1];
        const uint16_t delay_ms = (uint16_t)(frame.payload[off + 2] | (frame.payload[off + 3] << 8));
        const uint16_t hold_ms = (uint16_t)(frame.payload[off + 4] | (frame.payload[off + 5] << 8));
        if (kind == 0) {
            if (idx >= KEY_COUNT) continue;
            snprintf(key_buf, sizeof(key_buf), "kbd.hd%02u", idx);
            ConfigManager::set_uint16(key_buf, delay_ms);
            snprintf(key_buf, sizeof(key_buf), "kbd.mh%02u", idx);
            ConfigManager::set_uint16(key_buf, hold_ms);
        } else {
            if (idx >= ZONE_COUNT) continue;
            snprintf(key_buf, sizeof(key_buf), "kbd.zhd%02u", idx);
            ConfigManager::set_uint16(key_buf, delay_ms);
            snprintf(key_buf, sizeof(key_buf), "kbd.zmh%02u", idx);
            ConfigManager::set_uint16(key_buf, hold_ms);
        }
    }
    // 只写 RAM 影子并即时下发生效; flash 落地统一由 SAVE_CONFIG 触发(保护 flash 寿命)。
    getInstance()->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_get_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_TOUCHMAP);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    // 每分区 2 字节: [keycode, modifier]。
    r.payload[0] = self->_kbd_map_en ? 1 : 0;
    r.payload[1] = ZONE_COUNT;
    for (uint8_t z = 0; z < ZONE_COUNT; z++) {
        r.payload[2 + z * 2] = self->_zone_keycode[z];
        r.payload[3 + z * 2] = self->_zone_mod[z];
    }
    r.len = 2 + ZONE_COUNT * 2;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_set_touchmap(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // 每项 3 字节: [zone, keycode, modifier]。
    if (frame.len < 3 || (frame.len % 3) != 0) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_touchmap payload must be [zone,keycode,mod] triples", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    char key_buf[16];
    for (uint16_t off = 0; off + 2 < frame.len; off += 3) {
        const uint8_t zone = frame.payload[off];
        const uint8_t code = frame.payload[off + 1];
        const uint8_t mod = frame.payload[off + 2];
        if (zone >= ZONE_COUNT) continue;
        snprintf(key_buf, sizeof(key_buf), "kbd.zone%02u", zone);
        ConfigManager::set_uint8(key_buf, code);
        snprintf(key_buf, sizeof(key_buf), "kbd.zm%02u", zone);
        ConfigManager::set_uint8(key_buf, mod);
    }
    // 只写 RAM 影子并即时下发生效; flash 落地统一由 SAVE_CONFIG 触发(保护 flash 寿命)。
    getInstance()->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}

// ============================================================================
// 组合映射(KBD_GET_COMBO / KBD_SET_COMBO)
// 单条 16 字节: zone_mask(u64 LE) | key0..key3 | mod | delay_ms(u16 LE) | maxhold_ms(u16 LE)
// SET 为**整表替换**: 逐条增删会产生"半新半旧"的中间态, 而组合判定依赖整表一致性
// (同一分区可能同时属于多条), 中间态会瞬时输出错误按键。
// ============================================================================

void KeyboardService::_handle_get_combo(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    KeyboardService* self = getInstance();
    // HostFrame 为 4102B；core0 栈只有 8192B 且下界就是堆顶，响应帧必须借共享静态工作帧。
    HostFrame& r = HostCmdCodec::resp_frame();
    r.clear();
    r.cmd = static_cast<uint8_t>(HostCmd::KBD_GET_COMBO);
    r.flags = HOST_CMD_FLAG_RESPONSE;
    r.seq = frame.seq;
    r.payload[0] = COMBO_COUNT;
    r.payload[1] = COMBO_KEY_COUNT;
    uint16_t off = 2;
    for (uint8_t i = 0; i < COMBO_COUNT; i++) {
        const ComboMap& cm = self->_combo[i];
        for (uint8_t b = 0; b < 8; b++) {
            r.payload[off++] = (uint8_t)((cm.zone_mask >> (8u * b)) & 0xFFu);
        }
        for (uint8_t k = 0; k < COMBO_KEY_COUNT; k++) {
            r.payload[off++] = cm.keycode[k];
        }
        r.payload[off++] = cm.mod;
        r.payload[off++] = (uint8_t)(cm.delay_ms & 0xFFu);
        r.payload[off++] = (uint8_t)((cm.delay_ms >> 8) & 0xFFu);
        r.payload[off++] = (uint8_t)(cm.max_hold_ms & 0xFFu);
        r.payload[off++] = (uint8_t)((cm.max_hold_ms >> 8) & 0xFFu);
    }
    r.len = off;
    *resp_len = HostCmdCodec::encode_frame(r, resp, HOST_CMD_RESP_BUF_MAX);
}

void KeyboardService::_handle_set_combo(const HostFrame& frame, uint8_t* resp, uint16_t* resp_len) {
    // payload = [count] + count × 16B。count 允许为 0(清空整表)。
    if (frame.len < 1) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_combo payload must start with count", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }
    const uint8_t count = frame.payload[0];
    if (count > COMBO_COUNT || frame.len != (uint16_t)(1 + count * COMBO_ENTRY_BYTES)) {
        *resp_len = HostCmdCodec::encode_nak(frame.seq, HostCmdError::INVALID_PARAM,
            "kbd_set_combo count/length mismatch", resp, HOST_CMD_RESP_BUF_MAX);
        return;
    }

    KeyboardService* self = getInstance();
    ComboMap staged[COMBO_COUNT];
    for (uint8_t i = 0; i < COMBO_COUNT; i++) { staged[i].clear(); }

    uint8_t kept = 0;
    for (uint8_t i = 0; i < count; i++) {
        const uint16_t base = (uint16_t)(1 + i * COMBO_ENTRY_BYTES);
        ComboMap cm;
        cm.clear();
        for (uint8_t b = 0; b < 8; b++) {
            cm.zone_mask |= (uint64_t)frame.payload[base + b] << (8u * b);
        }
        // 只保留合法分区位: 高于 ZONE_COUNT 的位是主机侧错误, 静默保留会造成"永远无法全部按下"
        // 的死条目(判定恒 false), 排障时极难发现, 故直接掩掉。
        cm.zone_mask &= ((uint64_t)1u << ZONE_COUNT) - 1u;
        for (uint8_t k = 0; k < COMBO_KEY_COUNT; k++) {
            cm.keycode[k] = frame.payload[base + 8 + k];
        }
        cm.mod = frame.payload[base + 8 + COMBO_KEY_COUNT];
        const uint16_t d_off = (uint16_t)(base + 9 + COMBO_KEY_COUNT);
        cm.delay_ms = (uint16_t)(frame.payload[d_off] | ((uint16_t)frame.payload[d_off + 1] << 8));
        cm.max_hold_ms = (uint16_t)(frame.payload[d_off + 2] | ((uint16_t)frame.payload[d_off + 3] << 8));
        if (cm.zone_mask == 0) continue;   // 空掩码条目直接丢弃, 不占表位

        // 去重按用户规则: **分区集合完全相同**才算重复; 只要 zone_mask 不同就允许共存。
        bool dup = false;
        for (uint8_t n = 0; n < kept; n++) {
            if (staged[n].zone_mask == cm.zone_mask) { dup = true; break; }
        }
        if (dup) continue;
        staged[kept++] = cm;
    }

    for (uint8_t i = 0; i < COMBO_COUNT; i++) { self->_combo[i] = staged[i]; }
    // 整表换掉后所有时间状态机必须复位: 沿用旧状态会让新条目继承别人的计时起点/expired 标志。
    for (uint8_t i = 0; i < COMBO_COUNT; i++) { self->_combo_st[i].clear(); }
    // 已按下的键先全部松开, 否则被删掉的条目按住的键会永久卡住(表里已无人负责释放它)。
    HID* hid = HID::getInstance();
    for (uint8_t n = 0; n < self->_combo_out_count; n++) {
        hid->release_key(static_cast<HID_KeyCode>(self->_combo_out_keys[n]));
    }
    if (self->_combo_out_mod != 0) { _apply_mods(self->_combo_out_mod, false); }
    self->_combo_out_count = 0;
    self->_combo_out_mod = 0;

    // 写 RAM 影子; flash 落地统一由 SAVE_CONFIG 触发(与 set_map/set_touchmap 同口径, 保护 flash 寿命)。
    self->_store_combo();
    // 与 set_map/set_touchmap 同口径: 立刻重载一遍, 顺带把 comm.keyboard_map_en 的最新值读进来
    // (整表已写进 KV, _load_combo 读回的内容与刚 staged 的一致, 不会丢)。
    self->reload_map();
    *resp_len = HostCmdCodec::encode_ack(frame.seq, resp, HOST_CMD_RESP_BUF_MAX);
}
