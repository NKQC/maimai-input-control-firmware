#pragma once

#include <cstdint>
#include <hardware/sync.h>   // __dmb()

/**
 * 总线容器(header-only 模板): 定长静态数组、O(1)、无 malloc、无锁。
 *
 * 同步模型(照 psoc.h:249-251 / psoc.cpp:95-101 的 volatile + __dmb() 模式,
 * 不引入 pico SDK queue/mutex/critical_section):
 *   - 生产者只推 _head, 消费者只推 _tail; 数据写入在 __dmb() 之前, 指针发布在之后。
 *   - _head/_tail 为单调递增 uint32, 取模用掩码 → 要求 N 为 2 的幂(static_assert 强制)。
 *     uint32 回绕对 (_head - _tail) 差值语义无影响。
 *
 * 两种满载语义, 按用途二选一:
 *   BusRing  : 满则覆盖最旧 —— 实时数据"可丢旧"语义。
 *              ★覆盖动作由生产者推进 _tail★, 因此 BusRing 只能单上下文或
 *              "生产者独占"使用, 严禁跨核 SPSC。跨核一律用 BusQueue。
 *   BusQueue : 满则背压返回 false —— 绝不静默丢, 可跨核 SPSC。
 */

// ── 满则背压(不丢) ───────────────────────────────────────────────────────
template <typename T, uint32_t N>
class BusQueue {
    static_assert(N > 0u && (N & (N - 1u)) == 0u, "BusQueue: N 必须是 2 的幂");

public:
    void clear() {
        _tail = 0u;
        __dmb();
        _head = 0u;
    }

    bool empty() const { return _head == _tail; }
    bool full() const { return (uint32_t)(_head - _tail) >= N; }
    uint32_t size() const { return (uint32_t)(_head - _tail); }
    static constexpr uint32_t capacity() { return N; }

    // 生产者: 满则返回 false(背压), 不覆盖、不丢。
    bool push(const T& v) {
        if (full()) {
            return false;
        }
        _slot[_head & (N - 1u)] = v;
        __dmb();
        _head = _head + 1u;
        return true;
    }

    // 消费者: 取出最旧并释放槽位。
    bool pop(T& out) {
        if (empty()) {
            return false;
        }
        out = _slot[_tail & (N - 1u)];
        __dmb();
        _tail = _tail + 1u;
        return true;
    }

    // 消费者: 查看最旧但不释放。
    T* peek() { return empty() ? nullptr : &_slot[_tail & (N - 1u)]; }

    // 消费者: 丢弃最旧(不取出)。
    bool drop() {
        if (empty()) {
            return false;
        }
        __dmb();
        _tail = _tail + 1u;
        return true;
    }

    // 消费者侧随机访问(i=0 为最旧)。★仅消费者上下文可用★:
    // 用于在队列内就地扫描/改写在飞条目(重传窗口), 避免额外的 inflight 表。
    T* at(uint32_t i) { return (i >= size()) ? nullptr : &_slot[(_tail + i) & (N - 1u)]; }

private:
    T _slot[N];
    volatile uint32_t _head = 0u;
    volatile uint32_t _tail = 0u;
};

// ── 满则覆盖最旧(可丢旧) ─────────────────────────────────────────────────
template <typename T, uint32_t N>
class BusRing {
    static_assert(N > 0u && (N & (N - 1u)) == 0u, "BusRing: N 必须是 2 的幂");

public:
    void clear() {
        _tail = 0u;
        __dmb();
        _head = 0u;
    }

    bool empty() const { return _head == _tail; }
    bool full() const { return (uint32_t)(_head - _tail) >= N; }
    uint32_t size() const { return (uint32_t)(_head - _tail); }
    static constexpr uint32_t capacity() { return N; }

    // 满则丢弃最旧再写入。返回 false 表示发生了覆盖(便于计数, 覆盖不是静默的)。
    bool push(const T& v) {
        bool overwritten = false;
        if (full()) {
            _tail = _tail + 1u;
            __dmb();
            overwritten = true;
        }
        _slot[_head & (N - 1u)] = v;
        __dmb();
        _head = _head + 1u;
        return !overwritten;
    }

    bool pop(T& out) {
        if (empty()) {
            return false;
        }
        out = _slot[_tail & (N - 1u)];
        __dmb();
        _tail = _tail + 1u;
        return true;
    }

    T* peek() { return empty() ? nullptr : &_slot[_tail & (N - 1u)]; }

    bool drop() {
        if (empty()) {
            return false;
        }
        __dmb();
        _tail = _tail + 1u;
        return true;
    }

    T* at(uint32_t i) { return (i >= size()) ? nullptr : &_slot[(_tail + i) & (N - 1u)]; }

private:
    T _slot[N];
    volatile uint32_t _head = 0u;
    volatile uint32_t _tail = 0u;
};
