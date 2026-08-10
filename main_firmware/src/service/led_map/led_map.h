#pragma once

#include <stdint.h>

/**
 * LedMapService - 虚拟 LED 单元 → WS2812 物理段映射（单例）
 *
 * 灯板协议(mai2light)只产出 11 个虚拟单元的颜色, 本服务负责把每个单元铺到某条物理灯链的
 * 一段连续灯珠上并限速刷出。刻意不包含 mai2light.h: 对外只吃 11×3 的 flat RGB, 保持解耦。
 *
 * 段方向: 从数据发起端起 0..N-1; 同一单元只占一段; 不同单元的段不得重叠。
 */

#define LEDMAP_UNIT_COUNT      11
#define LEDMAP_CHANNEL_COUNT   2
#define LEDMAP_CH_NONE         0xFFu   // 未映射

// LED_GET status 的 bit6..7 初始化故障分档；保持在 2 bit 范围内。
enum LedMapInitFault : uint8_t {
    LEDMAP_INIT_FAULT_NONE = 0u,
    LEDMAP_INIT_FAULT_PIO = 1u,
    LEDMAP_INIT_FAULT_CHAIN0 = 2u,
    LEDMAP_INIT_FAULT_CHAIN1 = 3u,
};

struct LedMapEntry {
    uint8_t channel;    // 0/1, LEDMAP_CH_NONE=未映射
    uint16_t start;     // 段起始灯珠索引
    uint8_t count;      // 段灯珠数(>=1)

    LedMapEntry() : channel(LEDMAP_CH_NONE), start(0u), count(0u) {}
    inline void clear() { channel = LEDMAP_CH_NONE; start = 0u; count = 0u; }
    inline bool mapped() const { return channel < LEDMAP_CHANNEL_COUNT && count > 0u; }
};

class NeoPixel;   // 只经指针驱动, 不把 PIO 细节泄进本头文件

class LedMapService {
public:
    static LedMapService* getInstance();

    // 按 led.ws_count0/1 建两路 NeoPixel(同一 PIO1, 各占 1 个状态机)
    bool init();
    void deinit();
    inline bool is_ready() const { return _initialized; }
    // 初始化诊断: deinit 只清链路状态, 故障分档保留供 LED_GET 上报(否则失败原因随清理一起消失)。
    inline uint8_t init_fault() const { return (uint8_t)_init_fault; }
    inline bool chain_ready(uint8_t channel) const {
        return channel < LEDMAP_CHANNEL_COUNT && _chain_ready[channel];
    }

    /// 从 ConfigManager 重新读取 led.ws_brightness 并立刻推给两条灯链。
    /// 由 CFG_SET / CFG_SET_BATCH(键前缀 led.) 与 RESET_DEFAULTS 调用 —— 与 HidTouchMapper::reload()
    /// 同一约定: 配置一写进 RAM 影子就生效, 不必等 500ms 轮询窗, 也不必等 flash 落地或重启。
    /// ★只刷亮度★ 灯链长度(led.ws_count0/1)决定 NeoPixel 实例的缓冲与 PIO 建链, 运行期改它必须重建
    /// 灯链(init/deinit), 那是与"改个亮度"完全不同量级的动作, 不在本入口内偷偷做。
    void reload_brightness();

    // 协议色输入: 11×3 (r,g,b) flat
    void set_unit_colors(const uint8_t* rgb_flat);
    // 限速输出(非阻塞); 每次只推一条链
    void task();

    /// 当前**已生效**的亮度(灯链正在用的那个值), 供 LED_GET 与上位机对账。
    /// ★与 KV 里的 led.ws_brightness 是两件事★: 后者只是配置, 前者才是硬件行为的真相源 ——
    /// 没有这个回读, "亮度生效了吗"就只能靠肉眼看灯。
    inline uint8_t applied_brightness() const { return _brightness; }

    inline const LedMapEntry* map_table() const { return _map; }
    // 整表原子校验(越界/重叠) → 生效 → 请求持久化; 任一项非法则整批不生效
    bool apply_map(const LedMapEntry* table);
    uint16_t chain_count(uint8_t channel) const;

    // 预览色覆盖协议色; unit=0xFF 表示全部单元。超时后自动回到协议色。
    void preview_unit(uint8_t unit, uint8_t r, uint8_t g, uint8_t b);
    inline const uint8_t* effective_rgb() const { return _effective; }
    inline uint32_t show_errors() const { return _show_errors; }

    // 预览链路诊断(经 LED_GET byte1 上报): "预览色发下去了却看不到灯变" 有三种成因 ——
    // 服务未 init、task 从未跑到(game_io 门控关闭)、预览已超时回落。三者在设备侧本来无从区分,
    // 故把这三个事实分别暴露: 预览掩码是否非空、服务是否就绪、_refresh 实跑次数。
    inline bool preview_active() const { return _preview_mask != 0u; }
    // _refresh 实际执行次数(仅 task 内 _initialized 通过后自增); 0 = 生效色从未被计算过。
    inline uint32_t refresh_count() const { return _refresh_count; }

private:
    LedMapService();
    LedMapService(const LedMapService&) = delete;
    LedMapService& operator=(const LedMapService&) = delete;

    void _load_config();
    bool _validate(const LedMapEntry* table) const;
    void _refresh(uint32_t now_ms);
    void _render(uint8_t channel);

    static LedMapService* _instance;

    NeoPixel* _chain[LEDMAP_CHANNEL_COUNT];
    LedMapEntry _map[LEDMAP_UNIT_COUNT];
    uint16_t _ws_count[LEDMAP_CHANNEL_COUNT];

    uint8_t _protocol_rgb[LEDMAP_UNIT_COUNT * 3];
    uint8_t _preview_rgb[LEDMAP_UNIT_COUNT * 3];
    uint8_t _effective[LEDMAP_UNIT_COUNT * 3];
    uint16_t _preview_mask;          // 每单元 1 位: 该单元有预览色
    uint32_t _preview_deadline_ms;

    // 反复更新的量一律用类级成员刷新, 不在 task 内重复构造
    uint8_t _brightness;
    uint32_t _brightness_refresh_ms;
    uint32_t _next_out_us;
    uint8_t _out_channel;
    uint32_t _show_errors;
    uint32_t _refresh_count;                      // _refresh 实跑计数, 供 LED_GET 诊断位
    uint8_t _init_fault;                          // LedMapInitFault, 跨 deinit 保留
    bool _chain_ready[LEDMAP_CHANNEL_COUNT];
    bool _initialized;
};
