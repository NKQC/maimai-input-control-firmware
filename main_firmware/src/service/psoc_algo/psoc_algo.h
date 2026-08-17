#pragma once

#include <cstdint>

class Psoc;

// JIT 触控算法 blob store：RP2040 持有算法二进制(≤4096B, ABI v2)，在 PSoC 启动/复位后
// 下发到其 4KB 可执行槽。host 上传的自定义算法持久化到 /algo.bin；无存储或校验失败时回退
// 内嵌出厂默认(v3.1 HDR, 见 psoc_algo_default.h)。算法始终"上位机下发→RP 存储→PSoC 启动时下发"。
// ★这个常量必须与 PSoC 的 ALGO_SLOT_SIZE 逐字节一致★(psoc_firmware 的 psoc_algo_abi.h)。
// 改一处就要同时改三处: 此宏 / PSoC 的 ALGO_SLOT_SIZE / 上位机的 proto::algo::ALGO_MAX_LEN。
// 任何一处漏改的表现都是"上传成功但算法跑飞/仍跑旧代码", 且没有任何一层会报错。
#define PSOC_ALGO_MAX_LEN 4096u
// PSoC 的算法共享堆容量(ABI v2 的 algo_io_t::heap_size)。RP2040 自己不用它, 只做上报与校对:
// 上位机据此判断算法申请的堆是否已经贴到上限(heap_used 峰值 vs 这个值)。
#define PSOC_ALGO_HEAP_SIZE 256u
#define PSOC_ALGO_CHANNELS 36u
// 算法 C 源(已滤注释)存储上限。PSoC 只收 ASM(≤PSOC_ALGO_MAX_LEN); RP2040 额外持久化这份源作为"映射表",
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
        // ★逐通道可设置变量(ABI v2 的 cfg_ch[8], 每通道各一份)★
        // 插在 cfg 之后、crc32 之前 ⇒ 布局变了, 旧 flash 内容必须被整块拒绝而不是被错读,
        // 故 .cpp 里的 ALGO_BLOB_MAGIC 同步 bump 了一代(见那里的注释)。
        uint8_t  cfg_ch[PSOC_ALGO_CHANNELS][8];
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

    // 逐通道可设置变量(ABI v2 cfg_ch[36][8])。与 cfg/rom 同等地位: 随算法一起持久化, 并在
    // download 之后由 tick() 的 runtime sync 补推给 PSoC, 使复位/重下发后恢复上次设置。
    uint8_t cfg_ch(uint8_t ch, uint8_t idx) const;
    void set_cfg_ch(uint8_t ch, uint8_t idx, uint8_t val);   // 更新 RAM + 请求持久化(不自动下发)

    // 算法 C 源(映射表): host 上传时随附, RP2040 持久化, 供回读还原可编辑 C。
    const uint8_t* src() const { return _src; }
    uint32_t src_len() const { return _src_len; }
    /// 分片写入 C 源。offset 必须从 0 开始且严格连续(乱序/跳片一律拒绝, 半份源比没有源更坏);
    /// 只有收到最后一片(offset+n == total)才更新有效长度并请求持久化。total=0 = 清空源, 合法。
    bool set_src_chunk(uint32_t offset, uint32_t total, const uint8_t* s, uint32_t n);

    // host 上传：校验 crc16(CCITT-FALSE, 覆盖 data[0,len)) 一致后存入 RAM，标记非默认并请求持久化。
    bool set_algo(const uint8_t* src, uint16_t src_len, uint16_t src_crc16);
    // 请求异步恢复出厂算法；若当前上传尚未释放 SPI 槽位，先挂起请求，待 tick() 安全收口。
    bool request_reset_default();
    bool reset_default_pending() const { return _reset_default_pending; }
    // 回退出厂默认(v3.1 HDR)并请求持久化(下次启动用默认)。用于坏算法致复位后的自动回退。
    void reset_default();

    // 下发当前算法到 PSoC(经 SPI ALGO_* 分页事务)。启动/复位调用只启动有界流程；
    // main loop 经 tick() 每轮推进一个运行时参数，完成前 provisioning_active() 保持为真。
    bool download_to_psoc(Psoc* psoc);
    void abort_provisioning();
    bool provisioning_active() const { return _params_pending || _runtime_sync_active; }
    // 只把代码下发入队即返回, ROM/cfg 交给 tick() 补推。USB 命令处理器必须走这个, 否则 ACK 被拖住。
    // ★恒返回 true(除 psoc==nullptr / 无算法)★: "链路不可用"与"被隔离"都只是**推迟或按策略不发**,
    // 不是失败 —— 拿它当 NAK 来源会让 PSoC 一挂死就永远无法上传修好的算法(那正是要修的死锁)。
    bool request_download(Psoc* psoc);
    // 主循环每轮调用: core1 写完代码后补推 ROM/cfg; 另负责重试被推迟的下发(见 _download_pending)。
    void tick(Psoc* psoc);

    // ---------------- 坏算法救援 / 隔离(RAM only, 刻意不持久化) ----------------
    // ★为什么不持久化★: RP2040 重新上电就该给用户算法一次新机会 —— 上电即隔离等于"上一次的
    // 判决永久生效", 用户除了重新上传别无办法, 而现场往往正是"想让它再跑一次看看"。
    // ★为什么绝不动 _blob/_len/_crc16/_src★: 隔离只是"这一代不下发", 算法与 C 源必须原样留在
    // flash 里可回读, 否则用户辛苦写的算法会被固件悄悄扔掉(旧的 reset_default() 自动回退就是这样)。
    static constexpr uint8_t FATAL_QUARANTINE_RUN = 3u;
    void note_fatal();                  // 记一次"算法致 PSoC 挂死"; 连续达阈值转入隔离
    bool quarantined() const { return _quarantined; }
    bool download_pending() const { return _download_pending; }
    // 解除隔离 + 清致命计数, 并**重新武装一次下发**(_download_pending)。两个入口:
    // ① host 上传新算法(set_algo); ② host 显式点救援(PSOC_RESCUE)。
    // 顺带置 pending 是刻意的: "解除隔离"这个动作若不伴随一次真实下发, 用户点了救援之后设备
    // 表面解禁、实际仍在跑原生 CapSense, 直到下次 PSoC 复位才生效 —— 那与没解除没有区别。
    void clear_quarantine();

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
    void _push_runtime_params(Psoc* psoc);   // 推送一个待同步的 ROM/cfg 项
    bool _start_runtime_sync(Psoc* psoc);

    uint8_t  _blob[PSOC_ALGO_MAX_LEN];
    uint16_t _len;
    uint16_t _crc16;
    bool     _is_default;
    bool     _save_pending = false;
    bool     _reset_default_pending = false;
    // pending 已安全切换为默认 blob 并入队；仍需等待 PSoC INFO 真正确认默认长度后才结束请求。
    bool     _reset_default_started = false;
    // true = 代码下发已入队, 等 core1 写完后还要补推 ROM/cfg(见 tick)。
    bool     _params_pending = false;
    // ★必须是 u16★: 补推项数 = 36 ROM + 8 cfg + 36×8 cfg_ch = 332 项, 早已越过 u8。
    // 顺序: [0,36)=ROM, [36,44)=cfg, [44,44+288)=cfg_ch(ch=(i-44)/8, idx=(i-44)%8)。
    uint16_t _runtime_index = 0u;
    bool     _runtime_sync_active = false;
    uint16_t _rom[PSOC_ALGO_CHANNELS];   // 每通道 16 位只读 ROM(默认 0)
    uint8_t  _cfg[8] = {0u};             // 共享算法可设置变量(ABI cfg[8], 默认 0)
    uint8_t  _cfg_ch[PSOC_ALGO_CHANNELS][8] = {};   // 逐通道可设置变量(ABI v2 cfg_ch, 默认 0)
    // 救援/隔离状态(RAM only, 见公开区注释)。
    uint8_t  _fatal_run = 0u;            // 连续被判定"算法致 PSoC 挂死"的次数
    bool     _quarantined = false;
    bool     _download_pending = false;  // 下发被推迟(链路不可用/入队失败/曾被隔离), 由 tick() 重试
    uint32_t _download_retry_ms = 0u;    // 上次重试时刻(节流, 见 DOWNLOAD_RETRY_MS)
    static constexpr uint32_t DOWNLOAD_RETRY_MS = 500u;   // 重试节流: 别每轮猛敲 core1 命令环
    uint8_t  _src[PSOC_ALGO_SRC_MAX];    // 算法 C 源(已滤注释), 映射表回读用
    uint32_t _src_len = 0;               // 当前生效源字节数(0=无)
    uint32_t _src_wr  = 0;               // 分片接收进度(仅传输中有意义, 收满即并入 _src_len)

    static PsocAlgo* _instance;
};
