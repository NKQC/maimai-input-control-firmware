#pragma once

#include <cstdint>

#include "../../protocol/host_cmd/host_cmd.h"

/**
 * SelfHeal - 自持恢复事件上报。
 *
 * 存在理由: 固件有若干"自己救自己"的动作(XRES 复位 PSoC、回退致命算法、清空不可信的 CSD store、
 * 重新下发配置、PSoC 启动时强制改写全局配置)。这些动作会让【设备实际状态偏离上位机以为的状态】,
 * 若不上报, 用户看到的界面就是幻觉, 会据此误判。故每次自持恢复都在此登记一条事件, 由
 * TX_TASK_SELFHEAL 经 SELF_HEAL_EVENT(0x2F) STREAM 帧推给上位机落日志 + 触发真值回读。
 *
 * 设计: 固定环形队列(不分配), note() 可在主循环任意处调用; emit() 每次弹一条, 背压时不弹(下轮重试);
 * 队列空即自取消任务。队列满时丢最旧的一条并计入 dropped, 保证最新事件必然可见。
 */

enum SelfHealCode : uint8_t {
    SH_NONE = 0,
    SH_PSOC_RESET_LINK = 1,      // SPI 链路持续丢失(PSoC 崩溃/掉线) → 已 XRES 复位
    SH_PSOC_RESET_HANG = 2,      // 主循环卡死(scan_count 不推进) → 已 XRES 复位
    SH_ALGO_FALLBACK = 3,        // 用户算法判定致命 → 已回退内嵌默认算法
    SH_STORE_CLEARED = 4,        // 采样不可信(railed/停滞) → 已清空 CSD store 回出厂默认链
    SH_REPROVISIONED = 5,        // 已向 PSoC 重新下发算法 + CSD；detail bit31=启动校准失败, low8=阶段，否则 detail=模式
    SH_PSOC_BOOT_OVERRIDE = 6,   // PSoC 启动时强制改写了生成配置(detail = 位掩码, 见 GPARAM_BOOT_OVERRIDE)
    SH_PSOC_RESCUED = 7,         // PSoC 救砖(强制重刷)完成
    SH_SERIAL_RESET_ACTIONS = 8, // mai2serial RSET 后已受理的动作(detail bit0=IDAC校准, bit1=基线复位)
    SH_BASELINE_TRUST_RESTORED = 9, // PSoC 重启或救砖后实测采样可信，已清除 baseline_untrusted 运行态标志
    // 运行期某条 mai2 CDC 掉了枚举(detail bit0=serial, bit1=light)。
    // ★如实宣判失效, 不做救援★: 主机侧已经拆掉了该接口, 设备这边任何"重新武装/重开"都改变不了
    // 主机的判断, 只会把主循环搅乱。用户明确要求: 掉枚举就直接宣判, 不做无意义救援。
    SH_CDC_LOST = 10,
    // 某个 NvStore 区在 flash 里判为无效(detail = valid_mask)。单份存储的"坏只坏在那一区"
    // 必须让用户看得见, 否则该区静默恢复默认值, 用户只会以为"设置又丢了"。
    SH_NV_REGION_INVALID = 11,
    // PSoC 自报算法槽容量与 RP 编译期上传上限不一致；detail=RP槽容量<<16 | PSoC槽容量。
    SH_ALGO_CAPACITY_MISMATCH = 12,
};

class SelfHeal {
public:
    static constexpr uint8_t QUEUE_SIZE = 8;

    static SelfHeal* getInstance();

    // 登记一条自持恢复事件。同一 code 连续重复(如反复复位)只更新计数不刷屏: 由上位机按 seq 去重。
    void note(uint8_t code, uint32_t detail);

    // 弹出最早一条待发事件。无事件返回 false。
    bool pop(uint8_t* code, uint32_t* detail, uint16_t* seq);

    // 队列非空但推送任务已自取消(上位机当时不在/租约过期)时重新拉起, 保证事件必达。
    void note_rearm();

    bool empty() const { return _count == 0u; }
    uint16_t total() const { return _total; }
    uint16_t dropped() const { return _dropped; }

    // TxScheduler 任务入口: 组帧推送一条事件; 队列空则自取消任务。
    static void emit_task();

private:
    struct Entry {
        uint8_t  code;
        uint32_t detail;
        uint16_t seq;
        // ★有限次重发★: STREAM 帧无 ACK。上位机刚连上的那一瞬间(HELLO 已刷新在线判定, 但它还没
        // 进入接收循环)发出去的帧会被丢, 实测就是这样漏掉了开机那条事件。故每条事件间隔重发
        // SEND_TIMES 次才出队; 上位机按 seq 去重, 重复无副作用。
        uint8_t  sends;
        uint32_t last_send_ms;
    };
    static constexpr uint8_t SEND_TIMES = 3u;
    static constexpr uint32_t RESEND_GAP_MS = 700u;

    SelfHeal() = default;
    SelfHeal(const SelfHeal&) = delete;
    SelfHeal& operator=(const SelfHeal&) = delete;

    void _tick();

    static SelfHeal* _instance;

    Entry    _queue[QUEUE_SIZE] = {};
    uint8_t  _head = 0u;      // 下一个弹出位置
    uint8_t  _count = 0u;
    uint16_t _seq = 0u;       // 事件序号(单调递增, 供上位机去重/发现丢失)
    uint16_t _total = 0u;     // 累计事件数
    uint16_t _dropped = 0u;   // 队列满被丢弃的最旧事件数
    uint8_t  _tx_buf[32] = {};
    // HostFrame 含 4KB payload, 放栈上会撑爆 RP2040 栈(8KB) → 作为成员复用(本单例在堆上)。
    HostFrame _frame = {};
};
