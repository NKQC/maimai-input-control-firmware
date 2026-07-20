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
#define CSD_MODE_AUTO      0u   // 一次性自校准(SmartSense 已停用)
#define CSD_MODE_SEMI      1u   // 半自动/手动(主用模式)

class CsdConfig {
public:
    static CsdConfig* getInstance();

    void init();                       // LittleFS 挂载后从 flash 载入(若有)
    bool valid() const { return _valid; }
    uint8_t mode() const { return _mode; }

    // 写穿：host_cmd 处理器在改 PSoC 的同时更新本 store，使其为真相源。
    void note_mode(uint8_t mode) { _mode = (mode != 0u) ? CSD_MODE_SEMI : CSD_MODE_AUTO; }
    void note_param(uint8_t ch, uint8_t param_id, uint32_t value);

    // PSoC 同步
    void download_to_psoc(Psoc* psoc);   // 启动下发：SET_MODE + (semi 且 valid 时)全部参数 + APPLY
    void capture_from_psoc(Psoc* psoc);  // 从 PSoC 读当前(自整定)参数入 store 并标记 valid

    bool save();                          // 持久化到 flash(/csd.bin)（实际写；由主循环在安全窗口调用）
    void request_save() { _save_pending = true; }   // 置保存信号，延迟到主循环落地
    bool has_pending_save() const { return _save_pending; }

private:
    CsdConfig();
    CsdConfig(const CsdConfig&) = delete;
    CsdConfig& operator=(const CsdConfig&) = delete;

    static inline uint8_t _param_index(uint8_t param_id) {
        return (uint8_t)(param_id - CSD_PARAM_ID_MIN);
    }
    static inline bool _param_id_ok(uint8_t param_id) {
        return (param_id >= CSD_PARAM_ID_MIN) && (param_id < (CSD_PARAM_ID_MIN + CSD_PARAM_COUNT));
    }

    uint8_t  _mode;
    bool     _valid;
    bool     _save_pending = false;
    uint16_t _param[CSD_CHANNELS][CSD_PARAM_COUNT];

    static CsdConfig* _instance;
};
