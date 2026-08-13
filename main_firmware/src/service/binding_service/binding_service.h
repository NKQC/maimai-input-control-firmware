#pragma once

#include <cstdint>
#include "../../protocol/host_cmd/host_cmd.h"

/**
 * BindingService - 分区(34)↔物理通道(0..35)绑定服务
 *
 * 职责:
 *  - 注册 BIND_START/BIND_ABORT/BIND_GET_MAP/BIND_SET_MAP 四个 host_cmd handler。
 *  - 维护"指触绑定"状态机(IDLE/WAIT_TOUCH):BIND_START 进入等待,tick() 里
 *    检测首个新触发通道后写回 bind.mapNN 并回 BIND_EVENT,回 IDLE。
 *  - 缓存 34 个分区→物理通道索引(0xFF=未映射),供 game_io 消费,取代固定
 *    ZONE_CHANNEL_MAP 恒等映射。
 *
 * bind.mapNN(NN=00..33) 语义:值=物理通道索引(0..35),0xFFFFFFFF=未映射。
 */
class BindingService {
public:
    static BindingService* getInstance();

    // 注册 host_cmd handler + 首次从 ConfigManager 载入绑定表
    void init();

    // 从 ConfigManager 重新读取 bind.mapNN，刷新 _bind_ch 缓存
    void reload_binding();

    // 按当前绑定表把 36 位物理通道 mask 转成 34 位逻辑分区 mask
    uint64_t map_to_areas(uint64_t touch_mask) const;

    // 已绑定分区数(_bind_ch[z] 落在合法通道范围内的计数)。
    // ★诊断专用★ 触控→键盘链路里"摸到了但映射不出分区"这一环, 外部只能靠它与
    // map_to_areas 的结果对照才能区分"没触摸"与"绑定表是空的"; 判定路径不使用本值。
    inline uint8_t bound_zone_count() const {
        uint8_t n = 0;
        for (uint8_t z = 0; z < ZONE_COUNT; z++) {
            if (_bind_ch[z] < 36u) n++;
        }
        return n;
    }

    // 每轮主循环调用一次：WAIT_TOUCH 态下检测首个触发通道，完成绑定
    void tick(uint64_t touch_mask, bool link_ok);

private:
    enum class State : uint8_t {
        IDLE,
        WAIT_TOUCH,
    };

    static constexpr uint8_t ZONE_COUNT = 34;

    BindingService();
    BindingService(const BindingService&) = delete;
    BindingService& operator=(const BindingService&) = delete;

    static BindingService* _instance;

    State _state;
    uint8_t _active_zone;
    uint8_t _bind_ch[ZONE_COUNT];  // 0xFF = 未映射
    // BIND_EVENT 是设备主动回传，不能在命令响应占用 vendor TX 时直接丢弃。
    // 保留一个待发事件，下一轮 tick 在响应释放后重试；绑定状态机一次只产生一个事件，单槽足够。
    bool _bind_event_pending;
    uint8_t _bind_event_zone;
    uint8_t _bind_event_channel;
    uint8_t _bind_event_status;

    void _complete_bind(uint8_t zone, uint8_t channel);
    void _emit_bind_event(uint8_t zone, uint8_t channel, uint8_t status);
    void _try_emit_bind_event();

    static void _handle_bind_start(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_bind_abort(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_bind_get_map(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
    static void _handle_bind_set_map(const HostFrame& frame, uint8_t* response, uint16_t* response_length);
};
