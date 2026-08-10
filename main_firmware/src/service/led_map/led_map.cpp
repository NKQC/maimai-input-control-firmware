#include "../../config.h"   // 必须在 Arduino.h(经 config_manager→LittleFS 间接引入)之前
#include "led_map.h"
#include "../config_manager/config_manager.h"
#include "../../protocol/neopixel/neopixel.h"
#include "../../hal/pio/hal_pio.h"
#include <pico/stdlib.h>
#include <cstdio>
#include <cstring>

namespace {
// 输出限速: 每 8.33ms 推一条链 → 每链 60Hz。不每 tick 推的原因: 主循环 ~kHz 级,
// 而一条 N 珠链要按字忙等 PIO FIFO(N×30us), 每 tick 全推会把 core0 大半时间耗在灯上,
// 且 WS2812 本身不需要高于人眼刷新率的更新。两链交替 → 单 tick 只付一条链的代价。
constexpr uint32_t OUT_SLOT_US = 8333u;
// 预览超时: 上位机拖动调色时会连续下发, 停手约 3s 后自动回到游戏协议色, 避免"忘了退出预览"。
constexpr uint32_t PREVIEW_TIMEOUT_MS = 3000u;
// 亮度取自 KV, 每 500ms 刷一次(map<string> 查表不该进 60Hz 输出路径)。
constexpr uint32_t BRIGHTNESS_REFRESH_MS = 500u;
constexpr uint8_t PREVIEW_ALL = 0xFFu;

inline uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

// led.mapNN 打包: bit31..28=channel(0/1, 0xF=未映射), bit27..12=start, bit11..0=count
inline uint32_t pack_entry(const LedMapEntry& e) {
    if (!e.mapped()) return 0xF0000000u;
    return ((uint32_t)e.channel << 28) | ((uint32_t)e.start << 12) | (uint32_t)e.count;
}

inline void unpack_entry(uint32_t packed, LedMapEntry* out) {
    const uint8_t ch = (uint8_t)((packed >> 28) & 0x0Fu);
    const uint16_t start = (uint16_t)((packed >> 12) & 0xFFFFu);
    const uint32_t count = packed & 0x0FFFu;
    if (ch >= LEDMAP_CHANNEL_COUNT || count == 0u || count > 255u) {
        out->clear();
        return;
    }
    out->channel = ch;
    out->start = start;
    out->count = (uint8_t)count;
}

inline void map_key(uint8_t unit, char* buf, size_t len) {
    snprintf(buf, len, "led.map%02u", (unsigned)unit);
}
}  // namespace

LedMapService* LedMapService::_instance = nullptr;

LedMapService::LedMapService()
    : _preview_mask(0u), _preview_deadline_ms(0u), _brightness(128u),
      _brightness_refresh_ms(0u), _next_out_us(0u), _out_channel(0u),
      _show_errors(0u), _refresh_count(0u), _init_fault(LEDMAP_INIT_FAULT_NONE),
      _initialized(false) {
    for (uint8_t ch = 0; ch < LEDMAP_CHANNEL_COUNT; ch++) {
        _chain[ch] = nullptr;
        _chain_ready[ch] = false;
        _ws_count[ch] = 1u;
    }
    memset(_protocol_rgb, 0, sizeof(_protocol_rgb));
    memset(_preview_rgb, 0, sizeof(_preview_rgb));
    memset(_effective, 0, sizeof(_effective));
    _load_config();
}

// 懒构造无锁: 本服务的全部入口(game_io init/task、LED_* host_cmd 处理)都只在 core0 的
// Arduino loop 内同步调用 —— UsbComm::update 直接同步派发 host_cmd, GameIoService::task 同线程,
// core1 只跑 Psoc::core1_run 且不触碰本服务, 故不存在两核同时首次 new 的竞态。
// 若将来把 host_cmd 派发或灯效输出搬到 core1/中断, 此处必须改为预创建或加锁。
LedMapService* LedMapService::getInstance() {
    if (_instance == nullptr) _instance = new LedMapService();
    return _instance;
}

void LedMapService::_load_config() {
    _ws_count[0] = ConfigManager::get_uint16("led.ws_count0");
    _ws_count[1] = ConfigManager::get_uint16("led.ws_count1");
    _brightness = ConfigManager::get_uint8("led.ws_brightness");
    char key[16];
    for (uint8_t unit = 0; unit < LEDMAP_UNIT_COUNT; unit++) {
        map_key(unit, key, sizeof(key));
        unpack_entry(ConfigManager::get_uint32(key), &_map[unit]);
    }
    // flash 里的表可能是在更长的灯链下配的; 越界/重叠一律视为未配置, 免得开机就撞段。
    if (!_validate(_map)) {
        for (uint8_t unit = 0; unit < LEDMAP_UNIT_COUNT; unit++) _map[unit].clear();
    }
}

bool LedMapService::init() {
    if (_initialized) return true;
    _load_config();

    HAL_PIO1* pio = HAL_PIO1::getInstance();
    // PIO1 已被 PSoC SPI 初始化(单例 init 会短路), 故两条灯链的引脚必须显式登记。
    // 选 PIO1 而非 PIO0: PIO0 归 SWD, 两路 WS2812 各占 1 个 sm + 4 条指令,
    // 与 SPI 的 1 sm/7 指令共存(2+1 sm, 15/32 指令), 不动 SWD 也不抢 SPI。
    _init_fault = LEDMAP_INIT_FAULT_NONE;
    if (!pio->init(PIN_WS2812_0)) {
        _init_fault = LEDMAP_INIT_FAULT_PIO;
        return false;
    }
    pio->init_pin(PIN_WS2812_0);
    pio->init_pin(PIN_WS2812_1);

    const uint8_t pins[LEDMAP_CHANNEL_COUNT] = { PIN_WS2812_0, PIN_WS2812_1 };
    for (uint8_t ch = 0; ch < LEDMAP_CHANNEL_COUNT; ch++) {
        if (_chain[ch] == nullptr) _chain[ch] = new NeoPixel(pio, pins[ch], _ws_count[ch]);
        if (!_chain[ch]->init()) {
            // 哪条链倒下就记哪条: 两链共用 PIO1, 只有分档才能区分"程序内存/状态机耗尽"
            // 发生在第一条还是第二条上。deinit 保留该分档。
            _init_fault = (ch == 0u) ? LEDMAP_INIT_FAULT_CHAIN0 : LEDMAP_INIT_FAULT_CHAIN1;
            deinit();
            return false;
        }
        _chain_ready[ch] = true;
        _chain[ch]->set_brightness(_brightness);
        _chain[ch]->clear_all();
        _chain[ch]->show();
    }

    _next_out_us = time_us_32();
    _brightness_refresh_ms = now_ms();
    _initialized = true;
    return true;
}

void LedMapService::deinit() {
    for (uint8_t ch = 0; ch < LEDMAP_CHANNEL_COUNT; ch++) {
        _chain_ready[ch] = false;
        if (_chain[ch] == nullptr) continue;
        _chain[ch]->deinit();
        delete _chain[ch];
        _chain[ch] = nullptr;
    }
    // _init_fault 不清: 失败诊断必须活过清理, 否则 LED_GET 只能看到"没就绪"而说不出为什么。
    _initialized = false;
}

uint16_t LedMapService::chain_count(uint8_t channel) const {
    if (channel >= LEDMAP_CHANNEL_COUNT) return 0u;
    return _ws_count[channel];
}

void LedMapService::reload_brightness() {
    // 立即取 KV 真值并推给已就绪的灯链。★同时把轮询窗口对齐到现在★: 否则本次刷新之后 _refresh()
    // 仍可能在同一个 500ms 窗内再读一次(读到的是同一个值, 纯浪费一次 map<string> 查表)。
    _brightness = ConfigManager::get_uint8("led.ws_brightness");
    _brightness_refresh_ms = now_ms();
    if (!_initialized) return;   // 未建链: 值已存好, init() 会用它
    for (uint8_t ch = 0; ch < LEDMAP_CHANNEL_COUNT; ch++) {
        if (_chain[ch] == nullptr || !_chain_ready[ch]) continue;
        _chain[ch]->set_brightness(_brightness);
    }
    // 不在这里 show(): 输出仍由 task() 的 8.33ms 时隙按链交替推出(每链 60Hz), 下一个时隙即带上新亮度。
    // 在命令上下文里直接推链会按字忙等 PIO FIFO(N×30us), 把 USB 响应窗口拖长。
}

void LedMapService::set_unit_colors(const uint8_t* rgb_flat) {
    if (rgb_flat == nullptr) return;
    memcpy(_protocol_rgb, rgb_flat, sizeof(_protocol_rgb));
}

void LedMapService::preview_unit(uint8_t unit, uint8_t r, uint8_t g, uint8_t b) {
    if (unit == PREVIEW_ALL) {
        for (uint8_t i = 0; i < LEDMAP_UNIT_COUNT; i++) {
            _preview_rgb[i * 3u + 0u] = r;
            _preview_rgb[i * 3u + 1u] = g;
            _preview_rgb[i * 3u + 2u] = b;
        }
        _preview_mask = (uint16_t)((1u << LEDMAP_UNIT_COUNT) - 1u);
    } else if (unit < LEDMAP_UNIT_COUNT) {
        _preview_rgb[unit * 3u + 0u] = r;
        _preview_rgb[unit * 3u + 1u] = g;
        _preview_rgb[unit * 3u + 2u] = b;
        _preview_mask |= (uint16_t)(1u << unit);
    } else {
        return;
    }
    _preview_deadline_ms = now_ms() + PREVIEW_TIMEOUT_MS;
}

bool LedMapService::_validate(const LedMapEntry* table) const {
    for (uint8_t i = 0; i < LEDMAP_UNIT_COUNT; i++) {
        const LedMapEntry& a = table[i];
        if (a.channel == LEDMAP_CH_NONE) continue;
        if (a.channel >= LEDMAP_CHANNEL_COUNT || a.count == 0u) return false;
        if ((uint32_t)a.start + (uint32_t)a.count > (uint32_t)_ws_count[a.channel]) return false;
        for (uint8_t j = (uint8_t)(i + 1u); j < LEDMAP_UNIT_COUNT; j++) {
            const LedMapEntry& b = table[j];
            if (!b.mapped() || b.channel != a.channel) continue;
            if (a.start < (uint16_t)(b.start + b.count) && b.start < (uint16_t)(a.start + a.count)) {
                return false;   // 同链两段重叠
            }
        }
    }
    return true;
}

bool LedMapService::apply_map(const LedMapEntry* table) {
    if (table == nullptr || !_validate(table)) return false;

    char key[16];
    for (uint8_t unit = 0; unit < LEDMAP_UNIT_COUNT; unit++) {
        _map[unit] = table[unit];
        map_key(unit, key, sizeof(key));
        ConfigManager::set_uint32(key, pack_entry(_map[unit]));
    }
    // 解析路径不写 flash: 只置保存信号, 由主循环安全窗口落地。
    ConfigManager::save_config();
    return true;
}

void LedMapService::_refresh(uint32_t now) {
    if (_preview_mask != 0u && (int32_t)(now - _preview_deadline_ms) >= 0) {
        _preview_mask = 0u;   // 预览超时, 回到协议色
    }
    for (uint8_t unit = 0; unit < LEDMAP_UNIT_COUNT; unit++) {
        const uint8_t* src = ((_preview_mask & (uint16_t)(1u << unit)) != 0u)
            ? &_preview_rgb[unit * 3u] : &_protocol_rgb[unit * 3u];
        uint8_t* dst = &_effective[unit * 3u];
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
    }
    if ((now - _brightness_refresh_ms) >= BRIGHTNESS_REFRESH_MS) {
        _brightness_refresh_ms = now;
        _brightness = ConfigManager::get_uint8("led.ws_brightness");
    }
    // 只在真正算过生效色后自增: 上位机据此区分"预览没生效"是发丢了还是本服务根本没被 task 到。
    _refresh_count++;
}

void LedMapService::_render(uint8_t channel) {
    NeoPixel* chain = _chain[channel];
    if (chain == nullptr || !chain->is_ready()) return;

    chain->set_brightness(_brightness);
    chain->set_all_pixels(NeoPixel_Color(0u, 0u, 0u));   // 未映射灯珠保持熄灭
    for (uint8_t unit = 0; unit < LEDMAP_UNIT_COUNT; unit++) {
        const LedMapEntry& e = _map[unit];
        if (!e.mapped() || e.channel != channel) continue;
        const uint8_t* rgb = &_effective[unit * 3u];
        chain->set_range(e.start, e.count, NeoPixel_Color(rgb[0], rgb[1], rgb[2]));
    }
    // show() 内部按字非阻塞等待 + 超时返回 false; 失败只计数, 下个时隙自然重试。
    if (!chain->show()) _show_errors++;
}

void LedMapService::task() {
    if (!_initialized) return;

    _refresh(now_ms());

    const uint32_t now_us = time_us_32();
    if ((int32_t)(now_us - _next_out_us) < 0) return;
    _next_out_us = now_us + OUT_SLOT_US;
    _render(_out_channel);
    _out_channel = (uint8_t)(_out_channel ^ 1u);
}
