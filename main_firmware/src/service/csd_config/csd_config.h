#pragma once

#include <cstdint>

class Psoc;

// CSD 运行时参数 store：RP2040 作为"无状态 PSoC"的唯一真相源。
// 启动/重启后把 mode + (半自动)全部通道参数下发 PSoC；PSoC 自身不持久化任何数据。
// 参数 id 0x01..0x0B 连续，索引 = param_id - 0x01。
// 0x01..0x09 阈值/分辨率/snsClk/idac; 0x0A snsClkSource; 0x0B idacGainIndex(增幅)。
#define CSD_CHANNELS       36u
#define CSD_PARAM_COUNT    11u
#define CSD_PARAM_ID_MIN   0x01u
// 全局 CSD 配置(RAM 影子): INACTIVE_SNS/IDAC_GAIN_INIT/IDAC_MIN/RAW_TARGET/MFS_DIV_F1/MFS_DIV_F2, id 0x01..0x06
#define CSD_GLOBAL_COUNT   8u
#define CSD_GLOBAL_ID_MIN  0x01u
#define CSD_MODE_AUTO      0u   // 自动校准/标准完整处理
#define CSD_MODE_SEMI      1u   // 半自动手动(主用模式)

class CsdConfig {
public:
    static CsdConfig* getInstance();

    void init();                       // LittleFS 挂载后从 flash 载入(若有)
    bool valid() const { return _valid; }
    uint8_t mode() const { return _mode; }

    // 写穿：host_cmd 处理器在改 PSoC 的同时更新本 store，使其为真相源。
    void note_mode(uint8_t mode) { _mode = (mode != 0u) ? CSD_MODE_SEMI : CSD_MODE_AUTO; }
    void note_param(uint8_t ch, uint8_t param_id, uint32_t value);
    void note_global(uint8_t gparam_id, uint32_t value);   // 写穿全局配置真相源(host 改 PSoC 同时更新)

    // 清空 CSD 参数/全局 store 并标记无效, 使下次 PSoC 启动 provisioning 跳过参数下发,
    // PSoC 用其生成的出厂默认(已验证 180Hz 正常)。配合重启 PSoC = "CSD 恢复默认"。请求持久化。
    void clear();

    // "恢复默认→良好半自动基线"专用: 请求在 PSoC 重启并以出厂强制好全局(增益4/目标85%)自动校准就绪后,
    // 由主循环回读校准好的默认值 → 切 SEMI 快速模式 → 重下发生效 → 持久化。
    // 使"正常的半自动基线"成为默认(RP2040 持有, PSoC 无状态), 且避免落到慢速 AUTO / 全 0。
    void request_recapture() { _recapture_pending = true; _baseline_untrusted = false; }
    bool has_pending_recapture() const { return _recapture_pending; }
    void clear_recapture() { _recapture_pending = false; }

    // ★采样可信度抽检(恢复默认专用)★: 抽样若干通道(覆盖 Cp 两极)的 raw, 判定 PSoC 当前是否真的
    // "校准好了"。不可信 = 任一抽样通道 raw 满量程(railed, IDAC 校准发散) 或 全部抽样通道两次读取
    // 完全不抖动(扫描/测量停滞)。用于阻止把异常状态回读固化成新默认(否则"恢复默认"越点越坏)。
    bool sampling_trustworthy(Psoc* psoc);
    // 上次恢复默认是否因采样不可信而拒绝固化(经 DEVICE_INFO 上报, 供上位机提示改用 PSoC 救砖)。
    bool baseline_untrusted() const { return _baseline_untrusted; }
    void note_baseline_untrusted(bool untrusted) { _baseline_untrusted = untrusted; }

    // PSoC 同步
    void download_to_psoc(Psoc* psoc);   // 启动下发：SET_MODE + (semi 且 valid 时)全部参数 + APPLY
    // 仅半自动手动模式允许回读：AUTO 的实时值由 CapSense 自动计算，固化会毁掉用户手动参数。
    bool capture_from_psoc(Psoc* psoc);  // 从 PSoC 读当前参数入 store 并标记 valid

    bool save();                          // 持久化到 flash(/csd.bin)（实际写；由主循环在安全窗口调用）
    void request_save() { _save_pending = true; }   // 置保存信号，延迟到主循环落地
    bool has_pending_save() const { return _save_pending; }

private:
    CsdConfig();
    CsdConfig(const CsdConfig&) = delete;
    CsdConfig& operator=(const CsdConfig&) = delete;

    // 这些参数取 0 没有任何合法含义(分辨率 0 = 不扫描, 分频 0 = 除零, IDAC 0 = 无补偿电流→满量程),
    // store 里出现 0 只能理解为"该项无有效值", 下发时必须跳过, 保留 PSoC 侧校准/自适应的结果。
    static inline bool _param_zero_invalid(uint8_t param_id) {
        return param_id == 0x07u    // RESOLUTION
            || param_id == 0x08u    // SNS_CLK_DIV
            || param_id == 0x09u    // IDAC_MOD
            || param_id == 0x0Bu;   // IDAC_GAIN
    }

    static inline uint8_t _param_index(uint8_t param_id) {
        return (uint8_t)(param_id - CSD_PARAM_ID_MIN);
    }
    static inline bool _param_id_ok(uint8_t param_id) {
        return (param_id >= CSD_PARAM_ID_MIN) && (param_id < (CSD_PARAM_ID_MIN + CSD_PARAM_COUNT));
    }

    static inline bool _global_id_ok(uint8_t gparam_id) {
        return (gparam_id >= CSD_GLOBAL_ID_MIN) && (gparam_id < (CSD_GLOBAL_ID_MIN + CSD_GLOBAL_COUNT));
    }

    uint8_t  _mode;
    bool     _valid;
    bool     _save_pending = false;
    bool     _recapture_pending = false;   // 恢复默认: 重启就绪后回读校准好的默认→切SEMI
    bool     _baseline_untrusted = false;  // 上次恢复默认拒绝固化(采样异常); 运行态标志, 不持久化
    bool     _global_valid = false;   // host 设过或从 PSoC 捕获过全局配置
    uint16_t _param[CSD_CHANNELS][CSD_PARAM_COUNT];
    uint16_t _global[CSD_GLOBAL_COUNT];

    static CsdConfig* _instance;
};
