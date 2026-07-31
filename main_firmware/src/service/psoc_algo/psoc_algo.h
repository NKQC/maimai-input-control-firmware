#pragma once

#include <cstdint>

class Psoc;

// JIT 触控算法 blob store：RP2040 持有算法二进制(≤1024B, ABI v1)，在 PSoC 启动/复位后
// 下发到其 1KB 可执行槽。host 上传的自定义算法持久化到 /algo.bin；无存储或校验失败时回退
// 内嵌出厂默认(v3.1 HDR, 见 psoc_algo_default.h)。算法始终"上位机下发→RP 存储→PSoC 启动时下发"。
#define PSOC_ALGO_MAX_LEN 1024u
#define PSOC_ALGO_CHANNELS 36u
// 算法 C 源(已滤注释)存储上限。PSoC 只收 ASM(≤1024B); RP2040 额外持久化这份源作为"映射表",
// 供回读还原可编辑 C(变量名来自源本身)。
// ★32KB 装不进单帧★: HostFrame.payload 固定 4096(且 HostFrame 常作栈对象, 绝不能放大),
// 故 ALGO_SET_SRC/GET_SRC 走分片协议(见 host_cmd.h HOST_CMD_ALGO_SRC_CHUNK)。
#define PSOC_ALGO_SRC_MAX 32768u

class PsocAlgo {
public:
    static PsocAlgo* getInstance();

    /// ★落盘镜像★: 与 flash 中 ALGO_BIN 区的字节布局一致。本类不再自己碰 flash, 只维护镜像,
    /// 由 NvStore 单点 load/commit。C 源(_src)本身就是连续缓冲, 直接注册, 不需要额外镜像。
    #pragma pack(push, 1)
    struct Mirror {
        uint32_t magic;
        uint16_t len;
        uint16_t crc16;                     // CCITT-FALSE over data[0,len)
        uint8_t  is_default;
        uint8_t  reserved[3];
        uint8_t  data[PSOC_ALGO_MAX_LEN];
        uint16_t rom[PSOC_ALGO_CHANNELS];   // 每通道 16 位只读 ROM
        uint8_t  cfg[8];                    // 共享算法可设置变量(ABI cfg[8])
        uint32_t crc32;                     // 覆盖前面全部字节
    };
    #pragma pack(pop)

    /// 把两个区注册给 NvStore。必须在 NvStore::load() 之前调用。
    void register_storage();

    void init();                          // NvStore::load() 之后从镜像解出运行值，否则用内嵌默认

    const uint8_t* data() const { return _blob; }
    uint16_t len() const { return _len; }
    uint16_t crc16() const { return _crc16; }
    bool is_default() const { return _is_default; }

    // 每通道 16 位 ROM(算法只读常量, 如 fingerCap/Cp/阈值)。随算法一起持久化并在下发时推给 PSoC。
    uint16_t rom(uint8_t ch) const { return (ch < PSOC_ALGO_CHANNELS) ? _rom[ch] : 0u; }
    void set_rom(uint8_t ch, uint16_t val);   // 更新 RAM 表 + 请求持久化(不自动下发, 由 host 处理器决定)

    // 共享算法可设置变量(ABI cfg[8], 见 psoc_algo_abi.h ALGO_SETTING)。随算法一起持久化,
    // 并在 download_to_psoc 时随代码/ROM 一并推给 PSoC, 使复位后恢复上次设置。
    uint8_t cfg(uint8_t idx) const { return (idx < 8u) ? _cfg[idx] : 0u; }
    void set_cfg(uint8_t idx, uint8_t val);   // 更新 RAM 表 + 请求持久化(不自动下发, 由 host 处理器决定)

    // 算法 C 源(映射表): host 上传时随附, RP2040 持久化, 供回读还原可编辑 C。
    const uint8_t* src() const { return _src; }
    uint32_t src_len() const { return _src_len; }
    /// 分片写入 C 源。offset 必须从 0 开始且严格连续(乱序/跳片一律拒绝, 半份源比没有源更坏);
    /// 只有收到最后一片(offset+n == total)才更新有效长度并请求持久化。total=0 = 清空源, 合法。
    bool set_src_chunk(uint32_t offset, uint32_t total, const uint8_t* s, uint32_t n);

    // host 上传：校验 crc16(CCITT-FALSE, 覆盖 data[0,len)) 一致后存入 RAM，标记非默认并请求持久化。
    bool set_algo(const uint8_t* src, uint16_t src_len, uint16_t src_crc16);
    // 回退出厂默认(v3.1 HDR)并请求持久化(下次启动用默认)。用于坏算法致复位后的自动回退。
    void reset_default();

    // 下发当前算法到 PSoC(经 SPI ALGO_* 分页事务)。返回下发+PSoC commit 校验是否成功。
    bool download_to_psoc(Psoc* psoc);

    bool save();                           // 持久化到 flash(/algo.bin)（由主循环在安全窗口调用）
    void request_save() { _save_pending = true; }
    bool has_pending_save() const { return _save_pending; }

private:
    PsocAlgo();
    PsocAlgo(const PsocAlgo&) = delete;
    PsocAlgo& operator=(const PsocAlgo&) = delete;

    Mirror   _mirror = {};        // NvStore 注册的 ALGO_BIN 镜像(快照层)
    uint32_t _mirror_len = 0;     // 由 NvStore 读回时回填
    uint32_t _src_store_len = 0;  // ALGO_SRC 区有效长度(与 _src 同一块缓冲, 不再另存一份)

    void _load_default();
    void _sync_bin_storage();
    void _sync_src_storage();

    void _load_src();                    // 从 /algo_src.bin 载入算法 C 源(init() 调用)

    uint8_t  _blob[PSOC_ALGO_MAX_LEN];
    uint16_t _len;
    uint16_t _crc16;
    bool     _is_default;
    bool     _save_pending = false;
    uint16_t _rom[PSOC_ALGO_CHANNELS];   // 每通道 16 位只读 ROM(默认 0)
    uint8_t  _cfg[8] = {0u};             // 共享算法可设置变量(ABI cfg[8], 默认 0)
    uint8_t  _src[PSOC_ALGO_SRC_MAX];    // 算法 C 源(已滤注释), 映射表回读用
    uint32_t _src_len = 0;               // 当前生效源字节数(0=无)
    uint32_t _src_wr  = 0;               // 分片接收进度(仅传输中有意义, 收满即并入 _src_len)

    static PsocAlgo* _instance;
};
