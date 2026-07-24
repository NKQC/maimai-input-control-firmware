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

    // PSoC 同步
    void download_to_psoc(Psoc* psoc);   // 启动下发：SET_MODE + (semi 且 valid 时)全部参数 + APPLY
    void capture_from_psoc(Psoc* psoc);  // 从 PSoC 读当前参数入 store 并标记 valid

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

    static inline bool _global_id_ok(uint8_t gparam_id) {
        return (gparam_id >= CSD_GLOBAL_ID_MIN) && (gparam_id < (CSD_GLOBAL_ID_MIN + CSD_GLOBAL_COUNT));
    }

    uint8_t  _mode;
    bool     _valid;
    bool     _save_pending = false;
    bool     _global_valid = false;   // host 设过或从 PSoC 捕获过全局配置
    uint16_t _param[CSD_CHANNELS][CSD_PARAM_COUNT];
    uint16_t _global[CSD_GLOBAL_COUNT];

    static CsdConfig* _instance;
};
