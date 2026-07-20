#pragma once

#include <stdint.h>

// ======================================================================
// DelayLine - 单拷贝环形延迟线
//   以 100us 为时间片, 支持 0..MAX_UNITS*100us 的可配置输出延迟。
//   push/read 均 O(1) 摊还; 值只在环内保存一份(单拷贝), 引用返回不再拷贝。
//   用途: 把实时采样值(触控分区掩码 / 键盘报告态)按 UI 可设延迟对齐后取出,
//         触控与键盘各持一份独立实例, 互不影响。
//   线程模型: 非线程安全, 仅在单个核(core0 协议侧)内 tick。
// ======================================================================
template <typename T, uint16_t MAX_UNITS = 1000>   // 默认 1000*100us = 100ms
class DelayLine {
public:
    static constexpr uint32_t kUnitUs = 100;                     // 时间片粒度(us)
    static constexpr uint32_t kSize   = (uint32_t)MAX_UNITS + 1u; // 需容纳 [now-MAX, now]

    // 用 fill 铺满全环, 下次 tick 视首帧值重新预热。
    void reset(const T& fill) {
        for (uint32_t i = 0; i < kSize; ++i) _ring[i] = fill;
        _primed = false;
    }

    // now_us      : 当前时间(time_us_32);
    // delay_units : 延迟片数(0..MAX_UNITS, 每片 100us), 超出即钳制;
    // current     : 本次实时值。
    // 返回: 延迟 delay_units*100us 之前的值(引用, 单拷贝)。
    const T& tick(uint32_t now_us, uint16_t delay_units, const T& current) {
        if (delay_units > MAX_UNITS) delay_units = MAX_UNITS;
        const uint32_t idx = now_us / kUnitUs;

        if (!_primed) {
            // 首帧: 全环预热为现值, 避免延迟窗口内读到脏历史。
            for (uint32_t i = 0; i < kSize; ++i) _ring[i] = current;
            _last_idx = idx;
            _primed = true;
        } else {
            // 补齐上次到本次之间被跳过的时间片(调用慢于 100us 时留空片用现值填充),
            // 上限一整圈防越界/回绕(time_us_32 每 ~71.6min 回绕一次时最多重填一圈)。
            uint32_t gap = idx - _last_idx;   // 无符号回绕安全
            if (gap > kSize) gap = kSize;
            for (uint32_t k = 1; k <= gap; ++k) {
                _ring[(_last_idx + k) % kSize] = current;
            }
            _ring[idx % kSize] = current;     // 同片内多次 tick: 刷新为最新
            _last_idx = idx;
        }
        return _ring[(idx - delay_units) % kSize];
    }

private:
    T        _ring[kSize];
    uint32_t _last_idx = 0;
    bool     _primed   = false;
};
