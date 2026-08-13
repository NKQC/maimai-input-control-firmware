#pragma once

#include <stdint.h>

// ======================================================================
// DelayLine - 单拷贝环形延迟线（带链路耗时补偿）
//   以 100us 为时间片, 支持 0..MAX_UNITS*100us 的可配置输出延迟。
//   push/read 均 O(1) 摊还; 值只在环内保存一份(单拷贝), 引用返回不再拷贝。
//
//   ★补偿语义★ 本类保证的是"**实际发出时刻 − 该值的采样时刻** = 设定延迟",
//   而不是"在处理时刻之后再等设定延迟"。二者的差别就是链路耗时:
//     采样(SPI 取到掩码) → core0 映射/处理 → 写出 CDC
//   这段路径实测约数百 us。若按处理时刻入环、再叠加设定延迟, 用户设 20ms 实际会得到
//   20ms + 链路耗时, 且该耗时随负载浮动 ⇒ 延迟既偏大又不稳定。
//   因此写入用**采样时刻**索引, 读取用**预计发出时刻**索引, 链路耗时被自然吸收:
//     read_idx = emit_idx − delay_units,  write_idx = sample_idx
//   等效于自动入队一个 −(链路耗时) 的偏移。
//
//   边界: 设定延迟小于链路耗时时物理上无法达标(不能发布还没采到的值), 此时给出最新采样,
//   即退化为"尽可能快", 不会读到未写入的环位。
//
//   用途: 把实时采样值(触控分区掩码 / 键盘报告态)按 UI 可设延迟对齐后取出。
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

    // sample_us   : 本次实时值的**采样时刻**(该值在物理上何时成立);
    // emit_us     : 本次取出的值**预计实际发出的时刻**(含尚未发生的写出耗时);
    // delay_units : 期望的端到端延迟片数(0..MAX_UNITS, 每片 100us), 超出即钳制;
    // current     : 本次实时值。
    // 返回: 使 emit_us − 采样时刻 ≈ delay_units*100us 的那一份历史值(引用, 单拷贝)。
    const T& tick(uint32_t sample_us, uint32_t emit_us, uint16_t delay_units, const T& current) {
        if (delay_units > MAX_UNITS) delay_units = MAX_UNITS;
        const uint32_t write_idx = sample_us / kUnitUs;

        if (!_primed) {
            // 首帧: 全环预热为现值, 避免延迟窗口内读到脏历史。
            for (uint32_t i = 0; i < kSize; ++i) _ring[i] = current;
            _last_idx = write_idx;
            _primed = true;
        } else {
            // 补齐上次到本次之间被跳过的时间片(采样慢于 100us 时留空片用现值填充),
            // 上限一整圈防越界/回绕(time_us_32 每 ~71.6min 回绕一次时最多重填一圈)。
            // 采样时刻未推进(core1 本周期没取到新掩码)时 gap=0, 只刷新同一片。
            uint32_t gap = write_idx - _last_idx;   // 无符号回绕安全
            if (gap > kSize) gap = kSize;
            for (uint32_t k = 1; k <= gap; ++k) {
                _ring[(_last_idx + k) % kSize] = current;
            }
            _ring[write_idx % kSize] = current;     // 同片内多次 tick: 刷新为最新
            if ((int32_t)(write_idx - _last_idx) > 0) _last_idx = write_idx;
        }

        // 目标: 让这一份值在 emit_us 发出时, 距其采样时刻正好 delay_units 片。
        uint32_t read_idx = (emit_us / kUnitUs) - delay_units;
        _last_clamped = false;
        // 设定延迟 < 链路耗时: 不存在满足条件的历史片, 给最新采样(物理下限)。
        if ((int32_t)(read_idx - _last_idx) > 0) {
            read_idx = _last_idx;
            _last_clamped = true;
        }
        // 环容量下限: 不得越过一整圈之外的历史(那已是被覆盖的陈旧数据)。
        if ((uint32_t)(_last_idx - read_idx) > MAX_UNITS) {
            read_idx = _last_idx - MAX_UNITS;
            _last_clamped = true;
        }
        _last_read_idx = read_idx;
        return _ring[read_idx % kSize];
    }

    // 上一次 tick **实际读出**的那一片的片起点时刻(us, time_us_32 口径)。
    // ★补偿偏差只能这样量★ 期望是"实际发出 − 采样 = 设定值", 而"采样"就是这一片的时刻。
    // 不把它交出去, 上层只能拿自己的预测值去自证补偿正确, 那等于没量。
    uint32_t last_read_slice_us() const { return _last_read_idx * kUnitUs; }
    // 上一次 tick 是否被钳制(设定延迟小于链路耗时, 或越过了一整圈历史) —— 钳制即"物理上达不到"。
    bool last_clamped() const { return _last_clamped; }

private:
    T        _ring[kSize];
    uint32_t _last_idx = 0;
    uint32_t _last_read_idx = 0;
    bool     _last_clamped = false;
    bool     _primed   = false;
};
