#pragma once

#include <cstdint>
#include <cstddef>

/**
 * ★编译期 Region 布局表★
 *
 * 各区"每份"占用的扇区数不再统一: KV/CSD/ALGO_BIN 各 2 扇区就够, ALGO_SRC 要装下
 * Header + 32KB 算法 C 源(按 4KB 向上取整 = 9 扇区)。offset 由本表逐区累计推导, 步长不再统一,
 * 于是"改某个区的容量"只需改这一处, 不会让其它区的地址错位。
 *
 * 放在 class 外是刻意的: class 内的 static constexpr 数据成员初始化器里不能调用同 class 里
 * 尚未定义完的 constexpr 成员函数(累计求和必须用函数)。HEADER_BYTES 与 NvStore::Header 的
 * 实际尺寸由 class 内 static_assert 锁死, 不会两处漂移。
 */
namespace nv_layout {

constexpr uint32_t SECTOR_BYTES   = 4096u;            // flash 擦除粒度
constexpr uint32_t PAGE_BYTES     = 256u;             // flash_range_program 粒度
constexpr uint32_t HEADER_BYTES   = 32u;              // = sizeof(NvStore::Header), class 内 static_assert 校验
constexpr uint32_t ALGO_SRC_BYTES = 32768u;           // ALGO_SRC 负载上限(= PSOC_ALGO_SRC_MAX)
constexpr uint32_t RESERVED_BYTES = 1024u * 1024u;    // platformio.ini board_build.filesystem_size = 1m
constexpr uint32_t REGION_COUNT   = 4u;               // 与 NvStore::Region::COUNT 一致
constexpr uint32_t SLOTS          = 2u;               // 每区独立 A/B 双份

/// 装下 header+负载所需的扇区数(向上取整)。
constexpr uint32_t sectors_for(uint32_t payload_cap) {
    return (payload_cap + HEADER_BYTES + SECTOR_BYTES - 1u) / SECTOR_BYTES;
}

/// 下标与 NvStore::Region 同序: KV / CSD / ALGO_BIN / ALGO_SRC。
constexpr uint32_t SECTORS[REGION_COUNT] = { 2u, 2u, 2u, sectors_for(ALGO_SRC_BYTES) };

constexpr uint32_t region_bytes(uint32_t i) { return SECTORS[i] * SECTOR_BYTES; }
constexpr uint32_t payload_cap(uint32_t i) { return region_bytes(i) - HEADER_BYTES; }

/// A 份起点(相对保留区起始): 前序各区各占 SLOTS 份, 逐区累计。
constexpr uint32_t region_base(uint32_t i) {
    uint32_t off = 0u;
    for (uint32_t k = 0u; k < i; ++k) off += region_bytes(k) * SLOTS;
    return off;
}
constexpr uint32_t region_offset(uint32_t i, uint32_t slot) {
    return region_base(i) + slot * region_bytes(i);
}
constexpr uint32_t total_bytes() { return region_base(REGION_COUNT); }
constexpr uint32_t max_region_bytes() {
    uint32_t m = 0u;
    for (uint32_t k = 0u; k < REGION_COUNT; ++k) {
        if (region_bytes(k) > m) m = region_bytes(k);
    }
    return m;
}

constexpr bool layout_is_sector_aligned() {
    for (uint32_t i = 0u; i < REGION_COUNT; ++i) {
        if ((region_bytes(i) % SECTOR_BYTES) != 0u ||
            (region_offset(i, 0u) % SECTOR_BYTES) != 0u ||
            (region_offset(i, 1u) % SECTOR_BYTES) != 0u) return false;
    }
    return true;
}

constexpr bool payloads_are_nonempty() {
    for (uint32_t i = 0u; i < REGION_COUNT; ++i) {
        if (payload_cap(i) == 0u) return false;
    }
    return true;
}

static_assert(SECTOR_BYTES % PAGE_BYTES == 0u, "扇区必须是 256B 页整数倍(flash_range_program 要求)");
static_assert(HEADER_BYTES % 4u == 0u, "Header 必须 4 字节对齐");
static_assert(layout_is_sector_aligned(), "每个 Region 的 A/B 偏移必须按扇区对齐");
static_assert(payloads_are_nonempty(), "每个 Region 必须有有效负载容量");
static_assert(payload_cap(3) >= ALGO_SRC_BYTES, "ALGO_SRC 每份必须装得下 Header + 32KB C 源");
static_assert(region_offset(0, 0) == 0u, "首区必须从保留区起点开始");
static_assert(total_bytes() % SECTOR_BYTES == 0u, "总布局必须按扇区对齐");
static_assert(total_bytes() <= RESERVED_BYTES, "布局超出保留的 1MiB(见 board_build.filesystem_size)");

}  // namespace nv_layout

/**
 * NvStore - **唯一**的 flash 落盘实现 + 内存快照层
 *
 * ★为什么是唯一★
 * 先前落盘有三条互不知情的路径(ConfigManager 的 KV、CsdConfig 的 blob、PsocAlgo 的两个 blob),
 * 各自决定何时擦写、各自算 CRC、各自判有效性。实测结果: 写确实发生了(flash_write_count=25),
 * 但重启后只有一部分数据回得来 —— 谁覆盖谁、谁先加载, 无法排查。故收敛为单点: 除本类之外,
 * 全工程不允许再出现 flash_range_* 或任何文件 API。
 *
 * ★快照层(镜像缓存)★
 * 内存里始终保有一份与 flash 布局等价的镜像。读一律从镜像返回, 不碰 flash; 写先落镜像, 再由
 * commit_step() 增量写回。只有"首次启动 / 镜像丢失"才真正读 flash。既省 flash 寿命, 也让
 * "设备当前值"只有一个来源。
 * blob 不在本类里再复制一份: 持有者(CsdConfig / PsocAlgo)本来就在 RAM 存着, 让它注册缓冲进来,
 * 本类只负责搬运与校验 —— 不产生第二份, 就不可能两份不一致。
 *
 * ★增量落盘★
 * 写者只 mark_dirty(); commit_step() 每次最多落**一个**脏区, 于是一次保存被摊到几轮主循环,
 * 每轮之间 USB 照常服务, 不再出现数百 ms 的 USB 黑洞。
 *
 * ★地址★ 全部由链接器符号 `_FS_start` 推出(core 为 filesystem 预留区), 无任何硬编码 flash 地址。
 * MAGIC/VERSION 是 class 内 private static constexpr, 只作校验标签。
 */
class NvStore {
public:
    /// 存储区。KV 常驻本类; 其余三个是注册进来的外部缓冲。
    enum class Region : uint8_t {
        KV = 0,
        CSD = 1,
        ALGO_BIN = 2,
        ALGO_SRC = 3,
        COUNT = 4
    };

    /// 单条 KV 记录。定长 12B, 4 字节对齐。
    struct Record {
        uint32_t key_hash;   // FNV-1a 32(键名); 0 = 空槽
        uint8_t  type;       // 与 ConfigValueType 同值域
        uint8_t  reserved0;
        uint16_t reserved1;
        // 所有标量共用一个 32 位槽: bool/int8..uint32/float 都能无损放进去。
        // 用 union 而不是给每种类型开字段, 避免同功能组里散装多份。
        union Value {
            uint32_t u32;
            int32_t  i32;
            float    f32;
        } value;
        void clear() { key_hash = 0; type = 0; reserved0 = 0; reserved1 = 0; value.u32 = 0; }
    };
    static_assert(sizeof(Record) == 12, "Record 必须定长 12B(flash 布局依赖它)");

    static constexpr uint32_t SECTOR_BYTES = nv_layout::SECTOR_BYTES;

    /// 某区"每份"能装的负载字节数。注册方用它做 static_assert, 避免缓冲比区还大却只在运行期才发现。
    static constexpr uint32_t payload_capacity(Region region) {
        return nv_layout::payload_cap((uint32_t)region);
    }

    /// 定长静态字符串区。用户要求"优先定长纯静态, 动态内容能不用就不用"。
    static constexpr uint32_t STR_SLOTS = 4u;
    static constexpr uint32_t STR_LEN   = 32u;   // 含结尾 '\0'

    static NvStore* getInstance();

    // ---- 生命周期 ----

    /// 启动时调用一次: 把 flash 里各区读进镜像(KV 读入本类, blob 读入已注册的外部缓冲)。
    /// 必须在所有 register_blob() 之后调用。返回 KV 区是否有效(false = 首次启动/被写坏)。
    bool load();

    /// 由 main.cpp 在 core1 启动并完成 multicore_lockout_victim_init() 之后调用一次。
    /// 之前的写入(首次上电写默认配置)不做 lockout —— 那时 core1 还没跑, 不会碰 flash;
    /// 若此时去 lockout 会永久死锁(设备烧完枚举不出来, 实测踩过)。
    static void enable_lockout();

    /// 注册一个 blob 区的外部缓冲。len_io 既是"要写多少"也是"读回来多少"(由本类回填)。
    /// 必须在 load() 之前调用。
    void register_blob(Region region, uint8_t* buf, uint32_t* len_io, uint32_t capacity);

    // ---- 快照读写(全程不碰 flash) ----

    bool get(const char* key, Record::Value* out, uint8_t* type_out) const;
    /// 写入/覆盖一条并置 KV 脏。容量满返回 false(绝不静默丢弃)。
    bool set(const char* key, uint8_t type, Record::Value value);
    bool get_str(const char* key, char* out, uint32_t out_cap) const;
    bool set_str(const char* key, const char* value);

    /// 清空 KV 镜像(用于"恢复默认": 清空后由各服务重新注册默认值)。置脏。
    void clear_kv();

    /// 标记某区需要落盘。写者只调这个, 绝不自己擦写。
    void mark_dirty(Region region);
    bool dirty() const;
    uint8_t dirty_mask() const;
    uint32_t commit_ok_count() const { return _commit_ok; }
    uint32_t commit_fail_count() const { return _commit_fail; }
    uint32_t algo_src_len() const;

    // ---- 增量落盘 ----

    /// 落**一个**脏区。返回 true = 本次真的写了一次 flash(调用方据此知道"还要再来")。
    /// 主循环每轮调一次即可: 一次保存被摊成几轮, 中间 USB 照常服务。
    bool commit_step();

    /// KV 记录数与容量(供诊断上报)。
    uint32_t kv_count() const { return _count; }
    static constexpr uint32_t kv_capacity() { return MAX_RECORDS; }

    /// 编译期 FNV-1a 32 位哈希。键名在代码里是字面量, 多数调用点会被常量折叠。
    static constexpr uint32_t key_hash(const char* s) {
        uint32_t h = 2166136261u;
        while (*s != '\0') {
            h ^= (uint32_t)(uint8_t)(*s++);
            h *= 16777619u;
        }
        return h;
    }

private:
    /// 每份的头部。所有区共用同一格式 —— 单点实现的好处之一。
    struct Header {
        uint32_t magic;
        uint32_t version;
        uint32_t seq;      // 单调递增, 大者为新
        uint32_t len;      // 负载字节数(KV 区 = 记录区 + 字符串区)
        uint32_t crc;      // 负载 CRC32
        uint32_t reserved[3];
    };
    static_assert(sizeof(Header) == 32, "Header 必须 32B(布局依赖它)");
    // 布局表里的 HEADER_BYTES 与真实 Header 尺寸必须一致, 否则各区偏移会整体错位。
    static_assert(sizeof(Header) == nv_layout::HEADER_BYTES, "nv_layout::HEADER_BYTES 与 Header 尺寸不一致");

    /// 定长字符串槽。
    struct StrSlot {
        uint32_t key_hash;                 // 0 = 空槽
        char     text[STR_LEN];
        void clear() { key_hash = 0; text[0] = '\0'; }
    };

    /// 每个区的运行态。用 struct 聚合而不是散成几组平行数组。
    struct RegionState {
        uint8_t*  buf;        // 外部缓冲(blob 区); KV 区为 nullptr(镜像在本类内)
        uint32_t* len_io;     // 外部长度
        uint32_t  capacity;   // 外部缓冲容量
        uint32_t  seq;        // 当前生效份的 seq
        uint8_t   slot;       // 当前生效份(0=A, 1=B)
        bool      dirty;
        void clear() {
            buf = nullptr; len_io = nullptr; capacity = 0; seq = 0; slot = 0; dirty = false;
        }
    };

    static constexpr uint32_t MAGIC   = 0x5356324Du;   // 'M2VS'
    // v3 = 每区独立扇区数的布局表(ALGO_SRC 扩到 32KB), 与 v2 的统一步长布局不兼容 —— 版本号递增即
    // 让旧份自然作废(magic/version 不符 ⇒ 视为无效), 按用户要求不做兼容读。
    static constexpr uint32_t VERSION = 3u;
    // 直接取布局表(不走本 class 的 payload_capacity(): 静态数据成员初始化器里不能调用同 class
    // 内尚未定义完的 constexpr 成员函数)。
    static constexpr uint32_t KV_PAYLOAD_CAP = nv_layout::payload_cap((uint32_t)Region::KV);
    static constexpr uint32_t MAX_RECORDS =
        (KV_PAYLOAD_CAP - STR_SLOTS * (uint32_t)sizeof(StrSlot)) / (uint32_t)sizeof(Record);

    NvStore() = default;
    NvStore(const NvStore&) = delete;
    NvStore& operator=(const NvStore&) = delete;

    static NvStore* _instance;

    // KV 镜像
    Record   _rec[MAX_RECORDS];
    StrSlot  _str[STR_SLOTS] = {};
    uint32_t _count = 0;

    RegionState _rs[(uint32_t)Region::COUNT] = {};
    uint8_t _commit_next = 0u;
    uint32_t _commit_ok = 0u;
    uint32_t _commit_fail = 0u;

    /// 区在 flash 中的偏移(相对 flash 起始)。地址只由链接器符号 _FS_start + 布局表推出。
    static uint32_t _region_offset(Region region, uint8_t slot);
    /// 保留区实际大小是否够放下整张布局表(用 _FS_end 做运行期防御)。
    static bool _space_ok();
    /// 某区当前负载字节数(KV 区 = 记录区 + 字符串区; blob 区 = 注册的有效长度)。
    uint32_t _payload_len(Region region) const;
    /// 从某区负载的 off 处拷 n 字节出来。调用方保证 off+n ≤ _payload_len()。
    void _payload_copy(Region region, uint32_t off, uint8_t* dst, uint32_t n) const;
    /// 某区负载前 len 字节的 CRC32(不需要先拼成整块 —— 大区拼整块要 36KB 静态 RAM)。
    uint32_t _payload_crc(Region region, uint32_t len) const;
    /// 把读到的负载摊回镜像/外部缓冲。
    bool _unpack(Region region, const uint8_t* payload, uint32_t len);
    /// 读某份并校验; 成功则摊回。
    bool _read_slot(Region region, uint8_t slot, uint32_t* out_seq);
    /// 写某份(逐扇区擦+编程 + 回读校验)。
    bool _write_slot(Region region, uint8_t slot, uint32_t seq);
    static uint32_t _crc32_update(uint32_t crc, const uint8_t* data, uint32_t len);
    static uint32_t _crc32(const uint8_t* data, uint32_t len) {
        return ~_crc32_update(0xFFFFFFFFu, data, len);
    }
    int32_t _find(uint32_t hash) const;
};
