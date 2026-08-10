#pragma once

#include <cstdint>

/**
 * TxScheduler - 大吞吐周期发送的统一"定时任务队列"(续期制)。
 *
 * 设计目标(用户架构指令):
 *  - 纯异步: 任务只在到期时生成一帧并经 HAL_USB config_write 非阻塞入队(过载自弃), 不阻塞主循环。
 *  - 续期制: 每个任务带租约; 上位机需在租约内续期(任何主机命令帧都会 renew_all), 超时自动取消。
 *  - 统一端点/结构清晰: 所有周期性大吞吐(遥测等)都注册为任务, 走同一 vendor 端点。
 *  - O(1): 固定小任务表, tick/schedule/renew/cancel 均为常数级(遍历 MAX_TASKS)。
 *
 * emit_fn 为无参函数指针, 内部自行取数据、编码、调用 config_write(非阻塞)。
 */

enum TxTaskId : uint8_t {
    TX_TASK_TELEM = 1,      // 遥测数据流
    TX_TASK_AUTOTUNE = 2,   // 频率自适应阶段性进度(完成帧后自取消)
    TX_TASK_RESCUE = 3,     // PSoC 救砖(强制重刷)阶段性进度(完成帧后自取消)
    TX_TASK_SELFHEAL = 4,   // 自持恢复事件上报(队列空后自取消)
    TX_TASK_FOCUS = 5,      // 单通道 FocusSession 数据流
    TX_TASK_SWEEP = 6,      // 频率/增益扫描会话数据流与非阻塞步骤
};

class TxScheduler {
public:
    static constexpr uint8_t MAX_TASKS = 7;
    using EmitFn = void (*)();
    using ExpireFn = void (*)();

    static TxScheduler* getInstance();

    // 注册/更新任务(按 id 复用槽)。interval_us=发送周期; lease_ms=初始租约(此刻起)。
    // 已存在同 id 且周期相同则仅续期(不打断节奏); 否则(重)建并立即排入首帧。
    void schedule(uint8_t id, uint32_t interval_us, uint32_t lease_ms, EmitFn fn,
                  ExpireFn expire_fn = nullptr);
    // 续期指定任务(不存在则忽略)。
    void renew(uint8_t id, uint32_t lease_ms);
    // 续期全部活动任务(供"收到任何主机命令帧即续租"使用)。
    void renew_all(uint32_t lease_ms);
    // 取消任务。
    void cancel(uint8_t id);
    bool active(uint8_t id) const;

    // 主循环调用: 到期且租约未过 → emit; 租约过期 → 自动 cancel。O(MAX_TASKS)。
    void tick();

private:
    struct Task {
        bool     active;
        uint8_t  id;
        uint32_t interval_us;
        uint32_t next_us;
        uint32_t lease_deadline_us;   // 0 = 永不过期
        EmitFn   fn;
        ExpireFn expire_fn;
    };

    TxScheduler();
    TxScheduler(const TxScheduler&) = delete;
    TxScheduler& operator=(const TxScheduler&) = delete;

    static TxScheduler* _instance;
    Task _tasks[MAX_TASKS];

    int8_t _find(uint8_t id) const;   // 槽索引或 -1
    int8_t _alloc(uint8_t id);        // 复用现有或取空槽; 满则 -1
};
