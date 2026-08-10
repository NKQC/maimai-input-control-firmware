#pragma once

#include <cstdint>

class HID;

/**
 * HidTouchMapper - 物理通道(36) → HID 触摸屏固定点位 的边沿映射服务
 *
 * 职责(仅此一件):
 *  - 启动读取 hid.enNN / hid.xNN / hid.yNN(NN=00..35), 并可在配置变更后 reload() 刷新;
 *  - WORK_HID 模式下每个主循环把**可信** touch_mask 的每通道边沿翻译成一次
 *    HID_TouchPoint(id = 通道号+1, x/y = 该通道锚定的屏幕坐标), 只发边沿不发稳态;
 *  - 输出抑制(扫描会话) / 掩码不可信 / 未启用 → 把仍按下的触点全部释放。
 *
 * ★为什么是独立服务而不塞进 HID★
 *  HID 是协议层, 只该关心"怎么组触摸报文"; 一旦让它读 ConfigManager、问 Psoc 掩码、
 *  查 SensorLink 抑制态, 协议层就被拽进业务依赖里(且 serial 模式也会连带背上这些依赖)。
 *  本服务与 BindingService 同构(单例 + init + reload + tick(mask, trusted)), 经构造注入的
 *  HID* 指针输出, 不反向依赖任何调用方。
 *
 * ★与 Serial 侧 bind.mapNN 的关系: 完全无关★
 *  bind.mapNN 是"逻辑分区(34) → 物理通道", 服务 mai2 串口协议(WORK_SERIAL);
 *  hid.* 是"物理通道(36) → 屏幕坐标", 服务触摸屏上报(WORK_HID)。两套 KV 各自独立读写,
 *  本服务从不读写 bind.map*, 也不经过 BindingService 的分区映射 —— 逐通道直出, 不引入分区概念。
 *  键盘映射(kbd.zoneNN)同理与本服务无关: 它走分区→键码, 二者互不复用。
 */
class HidTouchMapper {
public:
    static HidTouchMapper* getInstance();

    /// 注册输出目标并首次载入点位表。`hid` 为空则本服务此后恒不输出(诊断关掉 HID 时的安全退化)。
    /// 同时锁存工作模式: mode.work 改动必须整机重启才换 USB 描述符(见 hal_usb.cpp 的
    /// current_usb_work_mode), 因此运行期它不会变, 无需每轮重读 KV。
    void init(HID* hid);

    /// 从 ConfigManager 重新读取 hid.enNN / hid.xNN / hid.yNN。
    /// 由 CFG_SET / CFG_SET_BATCH(键前缀 hid.) 与 RESET_DEFAULTS 调用 —— 配置改动即时生效,
    /// 不必等重启, 也不需要本服务去轮询 108 个 KV。
    void reload();

    /// 每主循环一次。`touch_mask` = 36 位物理通道掩码, `trusted` = 该掩码此刻是否可信
    /// (调用方已合成 psoc->touch_hold_ok() && !SensorLink::output_suppressed())。
    /// 非 HID 模式直接返回, 不触碰任何触摸状态。
    void tick(uint64_t touch_mask, bool trusted);

    /// 当前是否工作在 HID 模式(锁存值)。
    bool hid_mode() const { return _hid_mode; }
    /// 已启用点位数(诊断用)。
    uint8_t enabled_count() const;

private:
    static constexpr uint8_t CH_COUNT = 36;

    /// 单通道点位。三项同生同灭, 合成一个结构而不是三个平行数组: 逐通道读写只碰一处,
    /// 不会出现"坐标更新了但启用位还是旧的"这类半套状态。
    struct Point {
        uint16_t x;
        uint16_t y;
        bool enabled;
        void clear() { x = 0; y = 0; enabled = false; }
    };

    HidTouchMapper();
    HidTouchMapper(const HidTouchMapper&) = delete;
    HidTouchMapper& operator=(const HidTouchMapper&) = delete;

    /// 释放所有仍按下的触点并清零本地已应用掩码。
    inline void _release_all();

    static HidTouchMapper* _instance;

    HID* _hid;
    Point _point[CH_COUNT];
    /// 上一轮已反映到 HID 的通道掩码(仅含已启用通道)。边沿 = 本轮期望 ^ 本值。
    uint64_t _applied_mask;
    bool _hid_mode;
};
