#include "tx_scheduler.h"
#include "../usb_debug.h"
#include <pico/stdlib.h>

TxScheduler* TxScheduler::_instance = nullptr;

TxScheduler::TxScheduler() {
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        _tasks[i] = Task{false, 0, 0, 0, 0, nullptr, nullptr};
    }
}

TxScheduler* TxScheduler::getInstance() {
    if (_instance == nullptr) _instance = new TxScheduler();
    return _instance;
}

int8_t TxScheduler::_find(uint8_t id) const {
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (_tasks[i].active && _tasks[i].id == id) return (int8_t)i;
    }
    return -1;
}

int8_t TxScheduler::_alloc(uint8_t id) {
    const int8_t existing = _find(id);
    if (existing >= 0) return existing;
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (!_tasks[i].active) return (int8_t)i;
    }
    return -1;
}

void TxScheduler::schedule(uint8_t id, uint32_t interval_us, uint32_t lease_ms, EmitFn fn,
                           ExpireFn expire_fn) {
    const int8_t slot = _alloc(id);
    if (slot < 0) return;   // 任务表满: 忽略(过载保护, 不阻塞)
    Task& t = _tasks[slot];
    const uint32_t now_us = time_us_32();
    const bool reuse = (t.active && t.id == id && t.interval_us == interval_us);
    t.active = true;
    t.id = id;
    t.interval_us = interval_us;
    t.fn = fn;
    t.expire_fn = expire_fn;
    t.lease_deadline_us = (lease_ms == 0) ? 0u : (now_us + lease_ms * 1000u);
    if (!reuse) {
        t.next_us = now_us;   // 新建/改频: 立即排首帧
    }
}

void TxScheduler::renew(uint8_t id, uint32_t lease_ms) {
    const int8_t slot = _find(id);
    if (slot < 0) return;
    _tasks[slot].lease_deadline_us = (lease_ms == 0) ? 0u : (time_us_32() + lease_ms * 1000u);
}

void TxScheduler::renew_all(uint32_t lease_ms) {
    const uint32_t deadline = (lease_ms == 0) ? 0u : (time_us_32() + lease_ms * 1000u);
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (_tasks[i].active) _tasks[i].lease_deadline_us = deadline;
    }
}

void TxScheduler::cancel(uint8_t id) {
    const int8_t slot = _find(id);
    if (slot >= 0) _tasks[slot].active = false;
}

bool TxScheduler::active(uint8_t id) const {
    return _find(id) >= 0;
}

void TxScheduler::tick() {
    // XIP 擦写时 TinyUSB 与 flash 代码都无法可靠运行；暂停所有定时 IN 推送而非只停遥测，
    // 防止 telemetry/AUTO_TUNE/RESCUE/SELF_HEAL 共同挤满同一个 64B vendor FIFO 导致 stall。
    if (g_usb_flash_busy != 0u) return;
    const uint32_t now_us = time_us_32();
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        Task& t = _tasks[i];
        if (!t.active) continue;
        // 租约到期 → 自动停(续期超时/上位机丢失)。
        if (t.lease_deadline_us != 0 && (int32_t)(now_us - t.lease_deadline_us) >= 0) {
            const ExpireFn expire_fn = t.expire_fn;
            t.active = false;
            if (expire_fn) expire_fn();
            continue;
        }
        // 未到发送时刻(有符号差处理 32 位回绕)。
        if ((int32_t)(now_us - t.next_us) < 0) continue;
        if (t.fn) t.fn();
        // 基于当前时刻排下次, 避免累积漂移; 严重滞后(> 一个周期)时直接对齐到 now。
        const uint32_t behind = now_us - t.next_us;
        t.next_us = (behind > t.interval_us) ? (now_us + t.interval_us)
                                             : (t.next_us + t.interval_us);
    }
}
