#include "nv_store.h"

#include <cstring>

#ifdef PICO_PLATFORM
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/platform.h>
#include "../../flash_guard.h"
#endif

// ★地址由链接器给★: earlephilhower core 为 platformio.ini 的 board_build.filesystem_size 预留了
// 一段区域并导出 _FS_start。LittleFS 已彻底弃用, 整段归 NvStore, 于是全程没有任何硬编码 flash
// 地址 —— 改 filesystem_size 或固件变大, 地址都跟着链接器走。
extern "C" uint8_t _FS_start;
// core 同时导出保留区末尾, 用它在运行期核对"布局表真的放得进这段保留区"(链接脚本被改小时不至于
// 悄悄写到别人的地盘上)。
extern "C" uint8_t _FS_end;

NvStore* NvStore::_instance = nullptr;

NvStore* NvStore::getInstance() {
    if (_instance == nullptr) {
        static NvStore inst;
        _instance = &inst;
    }
    return _instance;
}

// ★lockout 只能在 core1 已启动且已注册 victim 之后做★
// ConfigManager 初始化在 setup 很早就跑, 首次上电会写一份默认配置 —— 那时 core1 还没启动、更没
// multicore_lockout_victim_init(), 调 multicore_lockout_start_blocking() 会**永久死锁**(实测:
// 烧完设备根本枚举不出来)。core1 未就绪时它也不碰 flash, 本来就不需要 lockout。
static volatile bool _nv_lockout_ready = false;

void NvStore::enable_lockout() {
    _nv_lockout_ready = true;
}

uint32_t NvStore::_region_offset(Region region) {
#ifdef PICO_PLATFORM
    const uint32_t fs_base = (uint32_t)(&_FS_start) - XIP_BASE;
    // 排布是 [KV][CSD][ALGO_BIN][ALGO_SRC], 每区一份、步长各自不同, 由 nv_layout 的累计表给出
    // —— 绝无硬编码绝对 flash 地址。
    return fs_base + nv_layout::region_offset((uint32_t)region);
#else
    (void)region;
    return 0u;
#endif
}

bool NvStore::_space_ok() {
#ifdef PICO_PLATFORM
    const uint32_t start = (uint32_t)(&_FS_start);
    const uint32_t end   = (uint32_t)(&_FS_end);
    return (end > start) && ((end - start) >= nv_layout::total_bytes());
#else
    return false;
#endif
}

uint32_t NvStore::_crc32_update(uint32_t crc, const uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return crc;
}

void NvStore::register_blob(Region region, uint8_t* buf, uint32_t* len_io, uint32_t capacity) {
    const uint32_t i = (uint32_t)region;
    if (i >= (uint32_t)Region::COUNT || region == Region::KV) return;
    // 缓冲比该区一份还大就不收: 收下只会在落盘时静默截断, 不如当场拒绝(注册方另有 static_assert)。
    if (capacity > nv_layout::payload_cap(i)) return;
    _rs[i].buf = buf;
    _rs[i].len_io = len_io;
    _rs[i].capacity = capacity;
}

// ---------------------------------------------------------------------------
// 打包 / 摊回: KV 区的镜像在本类内, blob 区的镜像就是注册进来的那块 RAM。
// KV 负载布局 = [Record × count][StrSlot × STR_SLOTS]; count 由 Header.len 反推。
// ---------------------------------------------------------------------------

uint32_t NvStore::_payload_len(Region region) const {
    if (region == Region::KV) {
        return _count * (uint32_t)sizeof(Record) + (uint32_t)sizeof(_str);
    }
    const RegionState& rs = _rs[(uint32_t)region];
    if (rs.buf == nullptr || rs.len_io == nullptr) return 0u;
    return *rs.len_io;
}

// ★不再拼整份再写★: ALGO_SRC 一份 36KB, 拼整块要一块 36KB 的静态 RAM。改为按需从镜像取字节,
// 落盘按扇区流式写出 —— 最大写缓冲固定 1 个扇区。
void NvStore::_payload_copy(Region region, uint32_t off, uint8_t* dst, uint32_t n) const {
    if (region != Region::KV) {
        const RegionState& rs = _rs[(uint32_t)region];
        if (rs.buf == nullptr) return;
        memcpy(dst, rs.buf + off, n);
        return;
    }
    // KV 负载 = [Record × count][StrSlot × STR_SLOTS], 两段可能被扇区边界切开, 最多两轮。
    const uint32_t rec_bytes = _count * (uint32_t)sizeof(Record);
    while (n > 0u) {
        const uint8_t* src;
        uint32_t avail;
        if (off < rec_bytes) {
            src = (const uint8_t*)_rec + off;
            avail = rec_bytes - off;
        } else {
            const uint32_t so = off - rec_bytes;
            if (so >= sizeof(_str)) return;
            src = (const uint8_t*)_str + so;
            avail = (uint32_t)sizeof(_str) - so;
        }
        const uint32_t m = (n < avail) ? n : avail;
        memcpy(dst, src, m);
        dst += m; off += m; n -= m;
    }
}

uint32_t NvStore::_payload_crc(Region region, uint32_t len) const {
    uint32_t crc = 0xFFFFFFFFu;
    if (region == Region::KV) {
        const uint32_t rec_bytes = _count * (uint32_t)sizeof(Record);
        crc = _crc32_update(crc, (const uint8_t*)_rec, rec_bytes);
        crc = _crc32_update(crc, (const uint8_t*)_str, len - rec_bytes);
    } else {
        const RegionState& rs = _rs[(uint32_t)region];
        if (rs.buf != nullptr) crc = _crc32_update(crc, rs.buf, len);
    }
    return ~crc;
}

bool NvStore::_unpack(Region region, const uint8_t* payload, uint32_t len) {
    if (region == Region::KV) {
        if (len < sizeof(_str)) return false;
        const uint32_t rec_bytes = len - (uint32_t)sizeof(_str);
        if ((rec_bytes % sizeof(Record)) != 0u) return false;
        const uint32_t n = rec_bytes / (uint32_t)sizeof(Record);
        if (n > MAX_RECORDS) return false;
        memcpy(_rec, payload, rec_bytes);
        memcpy(_str, payload + rec_bytes, sizeof(_str));
        _count = n;
        return true;
    }
    RegionState& rs = _rs[(uint32_t)region];
    if (rs.buf == nullptr || rs.len_io == nullptr) return false;
    if (len > rs.capacity) return false;
    memcpy(rs.buf, payload, len);
    *rs.len_io = len;
    return true;
}

bool NvStore::_read_region(Region region, uint32_t* out_write_count) {
#ifdef PICO_PLATFORM
    if (!_space_ok()) return false;
    const uint8_t* base = (const uint8_t*)(XIP_BASE + _region_offset(region));
    Header h;
    memcpy(&h, base, sizeof(h));
    if (h.magic != MAGIC) return false;
    if (h.version < VERSION_MIN_READ || h.version > VERSION) return false;
    // ★region 自述必须对上★: 单份化让各区偏移整体前移, 新位置可能压在旧布局里别的区的数据上,
    // 那些数据的 magic/version/CRC 全都合法, 只有自述字段能拦住"把 KV 的字节摊进 CSD"。
    if (h.region != (uint32_t)region) return false;
    if (h.len > nv_layout::payload_cap((uint32_t)region)) return false;
    const uint8_t* payload = base + sizeof(Header);
    // 读走 XIP 直接 memcpy + CRC, 零擦写 ⇒ 不可能出现"只读回一半"(旧 LittleFS 路径的老毛病)。
    if (_crc32(payload, h.len) != h.crc) return false;
    if (!_unpack(region, payload, h.len)) return false;
    if (out_write_count != nullptr) *out_write_count = h.write_count;
    return true;
#else
    (void)region; (void)out_write_count;
    return false;
#endif
}

#ifdef PICO_PLATFORM
// ★必须在 RAM 里执行★: 擦写期间 XIP 停用, 任何从 flash 取指令的代码都会 hardfault。
// 关中断 + lockout core1 同理 —— core1 的 1ms 周期任务也是 flash 代码。
// 一次只处理**一个扇区**: 擦整扇区, 再按需编程 len 字节(len=0 = 只擦, 用于尾部空扇区)。
// 关中断窗口因此固定在"单扇区擦写"这个量级, 不随区变大而变长。
static void __not_in_flash_func(_nv_program)(uint32_t offset, const uint8_t* buf, uint32_t len) {
    const bool use_lockout = _nv_lockout_ready;
    const uint32_t irq = save_and_disable_interrupts();
    if (use_lockout) multicore_lockout_start_blocking();
    flash_range_erase(offset, nv_layout::SECTOR_BYTES);
    if (len > 0u && buf != nullptr) flash_range_program(offset, buf, len);
    if (use_lockout) multicore_lockout_end_blocking();
    restore_interrupts(irq);
}
#endif

bool NvStore::_begin_commit(Region region) {
    const uint32_t idx = (uint32_t)region;
    const uint32_t len = _payload_len(region);
    if (len > nv_layout::payload_cap(idx)) return false;
    // 负载为 0 是合法的(例如清空算法 C 源), 仍要写一份"空但有效"的头。
    // ★头与 CRC 一次算定、整片复用★: 若每扇区各算一次, 中途负载被改动就会写出一份
    // "头与内容各说各话"的镜像 —— 那种镜像 CRC 校验能过, 内容却是拼接的两代, 最难查。
    // 现在负载中途变化只会让最终回读校验失败(脏标记仍在, 下轮重写), 不可能留下假有效的镜像。
    memset(&_commit_hdr, 0, sizeof(_commit_hdr));
    _commit_hdr.magic = MAGIC;
    _commit_hdr.version = VERSION;
    _commit_hdr.region = idx;
    _commit_hdr.len = len;
    _commit_hdr.crc = _payload_crc(region, len);
    _commit_hdr.write_count = _rs[idx].write_count + 1u;
    _commit_region = (int8_t)idx;
    _commit_pos = 0u;
    return true;
}

bool NvStore::_write_one_sector(Region region, uint32_t pos) {
#ifdef PICO_PLATFORM
    if (!_space_ok()) return false;
    const uint32_t base_off = _region_offset(region);
    const uint32_t len = _commit_hdr.len;
    const uint32_t img_used = (uint32_t)sizeof(Header) + len;   // 本区实际有内容的字节数
    // ★最大写缓冲 = 1 个扇区(定长静态)★
    static uint8_t sect[nv_layout::SECTOR_BYTES];

    // FlashWriteGuard 暂停定时 IN 推送并置 flash 忙标记(既有约定, 复用)。
    FlashWriteGuard guard;
    if (pos >= img_used) {
        // 尾部空扇区: 只擦不编程。必须擦 —— 否则下次写更长负载时会往未擦区域编程而写坏。
        _nv_program(base_off + pos, nullptr, 0u);
        watchdog_update();
        return true;
    }
    memset(sect, 0xFF, sizeof(sect));
    uint32_t dst = 0u;
    uint32_t img = pos;
    if (img < sizeof(Header)) {
        const uint32_t n = (uint32_t)sizeof(Header) - img;
        memcpy(sect, (const uint8_t*)&_commit_hdr + img, n);
        dst = n;
        img += n;
    }
    const uint32_t pay_off = img - (uint32_t)sizeof(Header);
    if (pay_off < len) {
        uint32_t n = len - pay_off;
        if (n > nv_layout::SECTOR_BYTES - dst) n = nv_layout::SECTOR_BYTES - dst;
        _payload_copy(region, pay_off, sect + dst, n);
    }
    _nv_program(base_off + pos, sect, nv_layout::SECTOR_BYTES);
    watchdog_update();
    return true;
#else
    (void)region; (void)pos;
    return false;
#endif
}

bool NvStore::_verify_region(Region region) const {
#ifdef PICO_PLATFORM
    // 回读校验后才认为写成功 —— 否则"写坏了"会被记成 commit_ok, 直到下次启动才暴露。
    const uint8_t* base = (const uint8_t*)(XIP_BASE + _region_offset(region));
    Header rh;
    memcpy(&rh, base, sizeof(rh));
    if (rh.magic != MAGIC || rh.version != VERSION ||
        rh.region != (uint32_t)region || rh.len != _commit_hdr.len) return false;
    return _crc32(base + sizeof(Header), rh.len) == rh.crc;
#else
    (void)region;
    return false;
#endif
}

bool NvStore::load() {
    bool kv_ok = false;
    for (uint32_t i = 0; i < (uint32_t)Region::COUNT; i++) {
        const Region r = (Region)i;
        RegionState& rs = _rs[i];
        rs.write_count = 0;
        rs.valid = false;
        rs.dirty = false;
        if (r != Region::KV && rs.buf == nullptr) continue;   // 未注册的 blob 区跳过

        // ★单份: 读就是读它, 没有"选哪一份"这一步★ —— 于是也不存在跨区选出不同代、拼出半新半旧
        // 的可能。校验不过就是本区无效, 由持有者按默认值重建, 相邻区一个字节都不受影响。
        uint32_t wc = 0;
        rs.valid = _read_region(r, &wc);
        if (rs.valid) rs.write_count = wc;
        if (r == Region::KV) {
            kv_ok = rs.valid;
            if (!rs.valid) _count = 0;
        }
    }
    return kv_ok;
}

void NvStore::mark_dirty(Region region) {
    const uint32_t i = (uint32_t)region;
    if (i < (uint32_t)Region::COUNT) _rs[i].dirty = true;
}

bool NvStore::dirty() const {
    return dirty_mask() != 0u;
}

uint8_t NvStore::dirty_mask() const {
    uint8_t mask = 0u;
    for (uint32_t i = 0; i < (uint32_t)Region::COUNT; i++) {
        if (_rs[i].dirty) mask |= (uint8_t)(1u << i);
    }
    return mask;
}

uint32_t NvStore::algo_src_len() const {
    return _payload_len(Region::ALGO_SRC);
}

uint8_t NvStore::valid_mask() const {
    uint8_t mask = 0u;
    for (uint32_t i = 0; i < (uint32_t)Region::COUNT; i++) {
        if (_rs[i].valid) mask |= (uint8_t)(1u << i);
    }
    return mask;
}

bool NvStore::commit_step() {
    constexpr uint32_t COUNT = (uint32_t)Region::COUNT;

    // ① 已有分片写在进行中 ⇒ 只推进它, 绝不中途切到别的区。
    //    切区会让两个区都停在半成品状态, 一次掉电同时丢两区 —— 与"坏只坏在变更那一区"相悖。
    if (_commit_region >= 0) {
        const uint32_t i = (uint32_t)_commit_region;
        const Region r = (Region)i;
        RegionState& rs = _rs[i];
        const uint32_t region_bytes = nv_layout::region_bytes(i);

        const bool wrote = _write_one_sector(r, _commit_pos);
        _commit_pos += nv_layout::SECTOR_BYTES;
        if (!wrote) {
            // 连一个扇区都没写成(空间校验不过等) ⇒ 放弃本片, 保留脏标记, 如实标无效。
            _commit_region = -1;
            rs.valid = false;
            _commit_fail++;
            return true;
        }
        if (_commit_pos < region_bytes) return true;   // 还有扇区, 下一轮继续(中间 USB 正常服务)

        _commit_region = -1;
        if (_verify_region(r)) {
            rs.write_count = _commit_hdr.write_count;
            rs.valid = true;
            rs.dirty = false;
            _commit_ok++;
        } else {
            // 本区 flash 已被擦过又没写成 ⇒ 如实标无效并**保留脏标记**下轮重写。
            // 谎报成功或悄悄清脏, 就会让"这一区其实已经没了"永远查不出来。
            rs.valid = false;
            _commit_fail++;
        }
        return true;
    }

    // ② 没有在进行的片 ⇒ 挑下一个脏区起片。
    // 起点随每次候选区轮转：KV 持续被置脏时，后续 CSD/ALGO 区仍会在有限轮内获得一次提交机会。
    for (uint32_t pass = 0u; pass < COUNT; pass++) {
        const uint32_t i = ((uint32_t)_commit_next + pass) % COUNT;
        RegionState& rs = _rs[i];
        if (!rs.dirty) continue;

        _commit_next = (uint8_t)((i + 1u) % COUNT);
        const Region r = (Region)i;
        if (r != Region::KV && rs.buf == nullptr) {
            rs.dirty = false;
            continue;
        }
        if (!_begin_commit(r)) {
            // 负载超容量: 明确失败, 清脏避免死循环重试(容量问题重试一万次也一样)。
            rs.dirty = false;
            _commit_fail++;
            return true;
        }
        // 起片这一轮就把第一个扇区写掉, 保持"调用一次 = 至多一次 flash 写"的既有语义。
        return commit_step();
    }
    return false;
}

// ---------------------------------------------------------------------------
// KV 快照读写: 全程只碰内存镜像
// ---------------------------------------------------------------------------

int32_t NvStore::_find(uint32_t hash) const {
    for (uint32_t i = 0; i < _count; i++) {
        if (_rec[i].key_hash == hash) return (int32_t)i;
    }
    return -1;
}

bool NvStore::get(const char* key, Record::Value* out, uint8_t* type_out) const {
    if (key == nullptr || out == nullptr) return false;
    const int32_t idx = _find(key_hash(key));
    if (idx < 0) return false;
    *out = _rec[idx].value;
    if (type_out != nullptr) *type_out = _rec[idx].type;
    return true;
}

bool NvStore::set(const char* key, uint8_t type, Record::Value value) {
    if (key == nullptr) return false;
    const uint32_t h = key_hash(key);
    const int32_t idx = _find(h);
    if (idx >= 0) {
        // 值没变就不置脏 —— 避免"保存"时把整区无谓地重写一遍, 省 flash 寿命。
        if (_rec[idx].type == type && _rec[idx].value.u32 == value.u32) return true;
        _rec[idx].type = type;
        _rec[idx].value = value;
        mark_dirty(Region::KV);
        return true;
    }
    if (_count >= MAX_RECORDS) return false;   // 容量满: 明确失败, 不静默丢弃
    _rec[_count].clear();
    _rec[_count].key_hash = h;
    _rec[_count].type = type;
    _rec[_count].value = value;
    _count++;
    mark_dirty(Region::KV);
    return true;
}

bool NvStore::get_str(const char* key, char* out, uint32_t out_cap) const {
    if (key == nullptr || out == nullptr || out_cap == 0u) return false;
    const uint32_t h = key_hash(key);
    for (uint32_t i = 0; i < STR_SLOTS; i++) {
        if (_str[i].key_hash != h) continue;
        uint32_t n = 0;
        while (_str[i].text[n] != '\0' && n + 1u < out_cap && n + 1u < STR_LEN) {
            out[n] = _str[i].text[n];
            n++;
        }
        out[n] = '\0';
        return true;
    }
    return false;
}

bool NvStore::set_str(const char* key, const char* value) {
    if (key == nullptr || value == nullptr) return false;
    uint32_t len = 0;
    while (value[len] != '\0') {
        len++;
        if (len >= STR_LEN) return false;   // 超长: 明确失败, 不截断
    }
    const uint32_t h = key_hash(key);
    for (uint32_t i = 0; i < STR_SLOTS; i++) {
        if (_str[i].key_hash == h) {
            if (memcmp(_str[i].text, value, len + 1u) == 0) return true;   // 未变则不置脏
            memcpy(_str[i].text, value, len + 1u);
            mark_dirty(Region::KV);
            return true;
        }
    }
    for (uint32_t i = 0; i < STR_SLOTS; i++) {
        if (_str[i].key_hash == 0u) {
            _str[i].key_hash = h;
            memcpy(_str[i].text, value, len + 1u);
            mark_dirty(Region::KV);
            return true;
        }
    }
    return false;   // 槽位满
}

void NvStore::clear_kv() {
    for (uint32_t i = 0; i < _count; i++) _rec[i].clear();
    _count = 0;
    for (uint32_t i = 0; i < STR_SLOTS; i++) _str[i].clear();
    mark_dirty(Region::KV);
}
