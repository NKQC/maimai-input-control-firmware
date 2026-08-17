#include "psoc_algo.h"
#include "psoc_algo_default.h"
#include "../../protocol/psoc/psoc.h"
#include "../../protocol/host_cmd/host_cmd.h"   // HostCmdCrc16::crc16 (CCITT-FALSE)
#include "../config_manager/config_crc.h"
#include "../../flash_guard.h"
#include "../../hal/usb/hal_usb.h"
#include <cstring>
#include "../usb_debug.h"

#ifdef PICO_PLATFORM
#include "LittleFS.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "../nv_store/nv_store.h"   // 自管理 flash blob(替代 LittleFS 文件)
#endif

namespace {
// ★magic 随 Mirror 布局改代★ "ALG1"(ABI v1: data[1024], 无 cfg_ch) → "ALG2"(ABI v2: data[4096]
// + cfg_ch[36][8])。区头的 CRC32 只能证明"这块字节没被写坏", 证明不了"它是当前布局";
// 若沿用旧 magic, 旧 flash 内容会被按新结构错读 —— len/crc16 落在同一偏移故看着合法, 但 data
// 之后的 rom/cfg 全部错位, 表现为"升级后每通道 ROM 变成一堆随机数"。bump 一代即让旧区
// 干净作废, 由内嵌默认重建。
constexpr uint32_t ALGO_BLOB_MAGIC = 0x32474C41u;  // "ALG2"
constexpr const char* ALGO_BLOB_PATH = "/algo.bin";
constexpr uint32_t ALGO_SRC_MAGIC = 0x31435241u;   // "ARC1" (algo source / mapping table)
constexpr const char* ALGO_SRC_PATH = "/algo_src.bin";

}  // namespace

PsocAlgo* PsocAlgo::_instance = nullptr;

PsocAlgo::PsocAlgo() : _len(0), _crc16(0), _is_default(true) {
    std::memset(_rom, 0, sizeof(_rom));   // 每通道 ROM 默认 0(算法未下发校准前)
    std::memset(_cfg, 0, sizeof(_cfg));   // 共享可设置变量默认 0
    std::memset(_cfg_ch, 0, sizeof(_cfg_ch));   // 逐通道可设置变量默认 0(与 PSoC 上电值一致)
    std::memset(_src, 0, sizeof(_src));
    _src_len = 0;
    _load_default();
}

void PsocAlgo::set_rom(uint8_t ch, uint16_t val) {
    if (ch >= PSOC_ALGO_CHANNELS) return;
    _rom[ch] = val;
    _sync_bin_storage();
    request_save();
}

void PsocAlgo::set_cfg(uint8_t idx, uint8_t val) {
    if (idx >= 8u) return;
    _cfg[idx] = val;
    _sync_bin_storage();
    request_save();
}

uint8_t PsocAlgo::cfg_ch(uint8_t ch, uint8_t idx) const {
    if (ch >= PSOC_ALGO_CHANNELS || idx >= 8u) return 0u;
    return _cfg_ch[ch][idx];
}

void PsocAlgo::set_cfg_ch(uint8_t ch, uint8_t idx, uint8_t val) {
    if (ch >= PSOC_ALGO_CHANNELS || idx >= 8u) return;
    _cfg_ch[ch][idx] = val;
    _sync_bin_storage();
    request_save();
}

bool PsocAlgo::set_src_chunk(uint32_t offset, uint32_t total, const uint8_t* s, uint32_t n) {
    if (total > PSOC_ALGO_SRC_MAX) return false;
    if (offset > total || n > total - offset) return false;
    if (total != 0u && n == 0u) return false;
    if (offset == 0u) _src_wr = 0u;        // offset=0 即"新一轮传输", 允许中途重来
    if (offset != _src_wr) return false;   // 严格连续: 缺片/乱序会拼出半份源, 宁可失败
    if (n > 0u) {
        if (s == nullptr) return false;
        std::memcpy(_src + offset, s, n);
    }
    _src_wr = offset + n;
    if (_src_wr < total) return true;       // 未收完: 不动生效长度, 不置脏
    _src_len = total;                      // 最后一片才让新源生效(total=0 → 清空)
    _src_wr = 0u;                          // 已提交；下一轮传输必须重新从 offset 0 开始
    _sync_src_storage();
    request_save();                        // 保留 SAVE_CONFIG 兜底，不参与实际落盘时机
    return true;
}

PsocAlgo* PsocAlgo::getInstance() {
    if (_instance == nullptr) _instance = new PsocAlgo();
    return _instance;
}

void PsocAlgo::_load_default() {
    uint16_t n = (uint16_t)PSOC_ALGO_DEFAULT_LEN;
    if (n > PSOC_ALGO_MAX_LEN) n = PSOC_ALGO_MAX_LEN;
    std::memcpy(_blob, PSOC_ALGO_DEFAULT, n);
    _len = n;
    _crc16 = PSOC_ALGO_DEFAULT_CRC16;
    _is_default = true;
}

void PsocAlgo::init() {
#ifdef PICO_PLATFORM
    // ALGO_BIN 校验只决定是否采用自定义二进制；ALGO_SRC 必须始终独立加载，不能因默认/损坏
    // 的二进制镜像提前 return 而丢弃已校验的源码区。
    Mirror& blob = _mirror;
    if (_mirror_len == sizeof(blob) && blob.magic == ALGO_BLOB_MAGIC) {
        const uint32_t crc = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));
        if (crc == blob.crc32) {
            std::memcpy(_rom, blob.rom, sizeof(_rom));
            std::memcpy(_cfg, blob.cfg, sizeof(_cfg));
            std::memcpy(_cfg_ch, blob.cfg_ch, sizeof(_cfg_ch));
            if (blob.is_default == 0u && blob.len > 0u && blob.len <= PSOC_ALGO_MAX_LEN &&
                HostCmdCrc16::crc16(blob.data, blob.len) == blob.crc16) {
                std::memcpy(_blob, blob.data, blob.len);
                _len = blob.len;
                _crc16 = blob.crc16;
                _is_default = (blob.len == (uint16_t)PSOC_ALGO_DEFAULT_LEN &&
                               blob.crc16 == PSOC_ALGO_DEFAULT_CRC16);
            }
        }
    }
#endif
    _load_src();
}

void PsocAlgo::_load_src() {
#ifdef PICO_PLATFORM
    // ★"C 源只读回一半"的老出处★: 原实现自己拼 10 字节头 + 两次 LittleFS read, 文件系统一不一致
    // 就读出截断内容。现在 _src 由 NvStore 直接注册, 区头自带 len + CRC32, 一次 memcpy 摊回,
    // 长度不符或 CRC 不过整区丢弃 —— 不可能出现半份源码。
    if (_src_store_len > PSOC_ALGO_SRC_MAX) { _src_len = 0; _src_store_len = 0; return; }
    _src_len = _src_store_len;
#endif
    _src_wr = 0u;   // 已加载源只影响有效长度；新上传始终要求从 offset 0 开始
}

bool PsocAlgo::set_algo(const uint8_t* src, uint16_t src_len, uint16_t src_crc16) {
    if (src == nullptr || src_len == 0u || src_len > PSOC_ALGO_MAX_LEN) return false;
    // 校验 host 提供的 crc16 与内容一致(CCITT-FALSE), 与 PSoC/默认 blob 同一算法。
    if (HostCmdCrc16::crc16(src, src_len) != src_crc16) return false;
    std::memcpy(_blob, src, src_len);
    _len = src_len;
    _crc16 = src_crc16;
    // 内容等于内嵌默认(v3.1 HDR)则标记为默认: 使上位机"读取信息"能正确识别并载入默认 C 源,
    // 而非误判为无源可还原的自定义算法。
    _is_default = (src_len == (uint16_t)PSOC_ALGO_DEFAULT_LEN && src_crc16 == PSOC_ALGO_DEFAULT_CRC16);
    // ★用户显式上传 ⇒ 作废尚未完成的"恢复默认"请求★
    // reset_default 的推进分支会在 tick 里把 _blob 换回内嵌默认并重新下发。若此刻还留着那个
    // pending, 用户刚上传的算法会在随后的某一轮 tick 被默认算法静默覆盖(实测表现为上传即刻
    // ACK 且终态确认成功, 紧接着回读却变成 psoc_valid=false / 又变回默认)。
    // 上传是比"恢复默认"更晚的用户意图, 必须赢。
    _reset_default_pending = false;
    _reset_default_started = false;
    // ★"每次新上传都必然获得一次运行机会"的唯一保证★
    // 隔离是按"上一份算法连续 3 次搞死 PSoC"下的判决, 它绝不能顺延到用户刚换上的这一份 ——
    // 否则用户改好了 bug 重新上传, 设备照旧不跑它, 且界面上看不出为什么(算法明明存进去了)。
    // 同时置 pending: 即使此刻链路不可用/core1 正忙, tick() 也会自己把它推下去, 不需要用户再点一次。
    clear_quarantine();
    _download_pending = true;
    _download_retry_ms = 0u;   // 立刻允许第一次尝试(不吃 500ms 节流)
    _sync_bin_storage();
    request_save();
    return true;
}

void PsocAlgo::note_fatal() {
    if (_fatal_run < 0xFFu) _fatal_run++;
    // ★这里绝不碰 _blob/_len/_crc16/_src★: 隔离只改"这一代要不要下发", 算法与 C 源必须原样
    // 留在 flash 里可回读。旧实现在这一步直接 reset_default() 把用户算法扔了, 用户既拿不回
    // 代码也说不清是谁改的 —— 那是本次要修掉的行为。
    if (_fatal_run >= FATAL_QUARANTINE_RUN) _quarantined = true;
}

void PsocAlgo::clear_quarantine() {
    _fatal_run = 0u;
    _quarantined = false;
    // 解除隔离必须伴随一次真实下发(见头文件注释), 否则"救援"只是改了个标志位。
    if (_len != 0u) {
        _download_pending = true;
        _download_retry_ms = 0u;   // 不吃节流, 下一轮 tick 立刻试
    }
}

// ★本函数只切 blob, 不动 C 源★
// 它由 tick 延迟推进(要等 core1 交还 blob), 而"恢复默认"在上位机那边是两步:
//   ① 发 ALGO_RESET_DEFAULT;  ② 紧接着把默认源文本 send_algo_src 写进来(设备不内置源文本)。
// 若在这里清 _src_len, 清空动作会**晚于**②落地, 把上位机刚写好的源又抹掉 ——
// 实测表现为: 上传/恢复默认当时一切正常, 重开上位机却报"设备未存该算法 C 源", 编辑器退回内置示例。
// 因此 C 源的清空改由 request_reset_default() 在受理命令的那一刻同步完成(见其注释)。
void PsocAlgo::reset_default() {
    _load_default();
    // ★隔离历史属于被换掉的那份自定义算法, 不能顺延到内嵌默认★
    // 否则"恢复默认"之后设备仍拒绝下发(默认算法是出厂验证过的), PSoC 会一直跑原生 CapSense,
    // 而界面显示的是"已恢复默认算法" —— 两边说法不一致且无从解释。
    clear_quarantine();
    _sync_bin_storage();
    request_save();
}

bool PsocAlgo::request_reset_default() {
    if (_reset_default_pending) return false;
    // ★C 源在此立即清空★: 与 blob 的延迟切换不同, 清源必须发生在受理命令的这一刻, 这样上位机
    // 随后补写的默认源才是最终生效的那份(顺序: 清空 → 上位机写入 → 结束)。
    _src_len = 0;
    _src_wr = 0;
    _sync_src_storage();
    // 仅登记恢复请求；当前 _blob 可能仍被 core1 的上传状态机持有，不能在这里改写。
    _reset_default_pending = true;
    _reset_default_started = false;
    return true;
}

// ★语义: "已受理"★ 返回 false 只保留给"根本没算法可发"这种调用方写错了的情形。
// 链路不可用 / core1 正忙 / 入队失败 ⇒ 记 pending 由 tick() 重试; 被隔离 ⇒ 按策略不发但也算受理。
// 这样上层就没有任何一条路径会因为 PSoC 的状态而把上传拒掉(见 sensor_link 的 _handle_algo_upload)。
bool PsocAlgo::request_download(Psoc* psoc) {
    if (psoc == nullptr || _len == 0u) return false;
    if (_quarantined) {
        // 已隔离: 本代不下发(PSoC 跑原生 CapSense), 且不留 pending —— 否则 tick() 会一直空转重试。
        // 解除方式只有两个: 重新上传(set_algo)或用户显式救援(clear_quarantine)。
        _download_pending = false;
        return true;
    }
    // ★blob 保护必须留着★: 上一次下发未完成时 blob 被 core1 持有, 此刻重发会让它写出半新半旧的
    // 代码。推迟而不是失败, 由 tick() 在 busy 落下后自动补上。
    if (psoc->algo_download_busy()) {
        _download_pending = true;
        return true;
    }
    // 判据用去抖后的 link_alive(): 遥测流式期间瞬时 link_ok 频繁为 false, 拿它当门禁会永远下发不了。
    // 链路真不可用时也只是推迟 —— 用户在 PSoC 挂死期间上传的修复算法必须能存下并在链路恢复后自动生效。
    if (!psoc->link_alive()) {
        _download_pending = true;
        return true;
    }
    // ★不能用 provisioning_active() 当门禁★
    // 它含 _runtime_sync_active —— 那是"逐条补推 36 个 ROM + 8 个 cfg"的阶段, 共 44 轮且每轮都要
    // core1_idle()。补推阶段**并不持有 blob**: 真正的 blob 保护是 Psoc::_algo_dl.busy(在
    // upload_algo() 内, 且 ALGO_UPLOAD handler 更早也挡了一次)。
    // 用它挡上传的后果(实测): 上位机停在算法页时会持续读 cfg/追踪, core1 频繁非空 ⇒ 补推迟迟
    // 走不完 ⇒ 用户此刻点"编译并上传"必被回 DEVICE_BUSY(cmd=0x61 NAK), 且越是盯着算法页越必然。
    // 新代码上传本就要按新算法重推一遍 ROM/cfg, 打断旧补推是正确且必要的。
    if (!psoc->upload_algo(_blob, _len, _crc16)) {
        // 入队失败(core1 长时间不消费命令环)同样只是推迟, 不是拒收。
        _download_pending = true;
        return true;
    }
    // 旧补推作废, 由本次下发完成后重新走一遍(_params_pending → _start_runtime_sync)。
    _runtime_sync_active = false;
    _runtime_index = 0u;
    _params_pending = true;
    _download_pending = false;   // 真正入队成功才算发出去了
    return true;
}

bool PsocAlgo::_start_runtime_sync(Psoc* psoc) {
    if (psoc == nullptr || !psoc->core1_idle() || psoc->heavy_busy()) return false;
    _runtime_index = 0u;
    _runtime_sync_active = true;
    return true;
}

void PsocAlgo::tick(Psoc* psoc) {
    if (psoc == nullptr) {
        _params_pending = false;
        _runtime_sync_active = false;
        return;
    }
    // ★推迟的下发在这里重试, 且必须放在最前面★
    // 本函数后续每个分支(恢复默认 / 补推 / runtime sync)都带 early return, 把重试写在末尾就永远
    // 走不到 —— 那等于"链路恢复后算法再也不会被推下去", 正是 R1 要修的东西。
    // _reset_default_pending 期间不插手: 那条分支自己负责下发, 两边都发会白跑一遍旧 blob。
    if (_download_pending && !_quarantined && !_reset_default_pending) {
        const uint32_t now_ms = millis();
        // 节流: core1 忙时每轮主循环都试一次只会白白占 SPI 命令环, 500ms 一次足够(用户感知不到)。
        if (_download_retry_ms == 0u || (uint32_t)(now_ms - _download_retry_ms) >= DOWNLOAD_RETRY_MS) {
            _download_retry_ms = now_ms;
            if (psoc->link_alive() && !psoc->algo_download_busy() && psoc->core1_idle()) {
                (void)request_download(psoc);   // 成功入队即自行清 _download_pending
            }
        }
    }
    if (_reset_default_pending) {
        // 先等旧上传释放 blob；切换后 pending 持续到 PSoC 的异步 INFO cache 确认默认代码真实生效。
        if (!_reset_default_started) {
            if (psoc->algo_download_busy() || !psoc->core1_idle()) return;
            _params_pending = false;
            _runtime_sync_active = false;
            reset_default();
            // ★这里必须用严格版★: 宽松版"推迟也返回 true"会让 _reset_default_started 在没真发出去
            // 的情况下置位, 随后本函数只走"等 INFO 确认默认长度"那条分支, 而顶部的 pending 重试又
            // 被 _reset_default_pending 排除 ⇒ 恢复默认永久挂起。返回 false 就下轮重试, 与改造前一致。
            if (!download_to_psoc(psoc)) return;
            _reset_default_started = true;
            return;
        }
        if (psoc->algo_download_busy()) return;
        bool valid = false;
        uint16_t len = 0u;
        if (!psoc->get_algo_info_cached(&valid, &len) || !valid ||
            len != static_cast<uint16_t>(PSOC_ALGO_DEFAULT_LEN)) return;
        _reset_default_pending = false;
        _reset_default_started = false;
        return;
    }
    if (_params_pending) {
        if (psoc->algo_download_busy() || !psoc->core1_idle()) return;
        _params_pending = false;
        (void)_start_runtime_sync(psoc);
        return;
    }
    if (!_runtime_sync_active || !psoc->core1_idle() || psoc->heavy_busy()) return;

    if (_runtime_index < PSOC_ALGO_CHANNELS) {
        const uint8_t ch = static_cast<uint8_t>(_runtime_index++);
        if (_rom[ch] != 0u) (void)psoc->set_algo_rom(ch, _rom[ch]);
        return;
    }
    const uint16_t cfg_index = static_cast<uint16_t>(_runtime_index - PSOC_ALGO_CHANNELS);
    if (cfg_index < 8u) {
        _runtime_index++;
        (void)psoc->algo_set_cfg(static_cast<uint8_t>(cfg_index), _cfg[cfg_index]);
        return;
    }
    // 逐通道 cfg_ch: 36×8 = 288 项。★与 ROM 同款: 值为 0 就跳过不发★
    // PSoC 的 g_algo_cfg_ch 上电即全 0, 补推一个 0 等于什么也没做, 却要占 288 轮 core1_idle 窗口
    // (每轮一笔 SPI 事务)。默认状态下绝大多数是 0, 逐条硬推会让每次下发后的补推阶段拖上几百 ms,
    // 期间用户的 GET_CFG/追踪读全部排在后面(实测过这种迟滞: 上位机成片超时)。
    const uint16_t ch_index = static_cast<uint16_t>(cfg_index - 8u);
    if (ch_index < PSOC_ALGO_CHANNELS * 8u) {
        _runtime_index++;
        const uint8_t ch = static_cast<uint8_t>(ch_index / 8u);
        const uint8_t idx = static_cast<uint8_t>(ch_index % 8u);
        if (_cfg_ch[ch][idx] != 0u) (void)psoc->set_algo_cfg_ch(ch, idx, _cfg_ch[ch][idx]);
        return;
    }
    _runtime_sync_active = false;
}

void PsocAlgo::_push_runtime_params(Psoc* psoc) {
    (void)_start_runtime_sync(psoc);
}

// ★严格语义(与 request_download 的宽松语义刻意分开)★
// 启动 provisioning 用本函数判"本代是否已真正开始下发算法"(见 core0_loop 的 provisioning_started)。
// 若沿用 request_download 的"推迟也算受理", provisioning 会在算法其实一个字节都没发出去的情况下
// 直接推进到 CSD 阶段并把本代标记为完成 —— 那就再也没有人重发它了。
bool PsocAlgo::download_to_psoc(Psoc* psoc) {
    if (!request_download(psoc)) return false;
    // 被隔离时"不下发"就是本代终态: provisioning 必须照常往下走, 否则一个坏算法会把 CSD 参数
    // 下发也一起锁死(整台设备停在未配置状态), 那比不跑算法严重得多。
    return _quarantined || !_download_pending;
}

void PsocAlgo::abort_provisioning() {
    _params_pending = false;
    _runtime_sync_active = false;
    _runtime_index = 0u;
}

void PsocAlgo::_sync_bin_storage() {
#ifdef PICO_PLATFORM
    Mirror& blob = _mirror;
    std::memset(&blob, 0, sizeof(blob));
    blob.magic = ALGO_BLOB_MAGIC;
    blob.len = _len;
    blob.crc16 = _crc16;
    blob.is_default = _is_default ? 1u : 0u;
    std::memcpy(blob.data, _blob, (_len <= PSOC_ALGO_MAX_LEN) ? _len : PSOC_ALGO_MAX_LEN);
    std::memcpy(blob.rom, _rom, sizeof(blob.rom));
    std::memcpy(blob.cfg, _cfg, sizeof(blob.cfg));
    std::memcpy(blob.cfg_ch, _cfg_ch, sizeof(blob.cfg_ch));
    blob.crc32 = ConfigCRC::calculate_crc32((const uint8_t*)&blob, sizeof(blob) - sizeof(uint32_t));
    _mirror_len = sizeof(_mirror);
    NvStore::getInstance()->mark_dirty(NvStore::Region::ALGO_BIN);
#endif
}

void PsocAlgo::_sync_src_storage() {
#ifdef PICO_PLATFORM
    _src_store_len = _src_len;
    NvStore::getInstance()->mark_dirty(NvStore::Region::ALGO_SRC);
#endif
}

bool PsocAlgo::save() {
    _save_pending = false;
    _sync_bin_storage();
    _sync_src_storage();
#ifdef PICO_PLATFORM
    return true;
#else
    return false;
#endif
}

// ★落盘由 NvStore 单点负责★
// 本类不再自己碰 flash: ALGO_BIN 注册 _mirror, ALGO_SRC 直接注册 _src(它本身就是连续缓冲, 不需要
// 再复制一份镜像 —— 少一份就少一处不一致的可能)。必须在 NvStore::load() 之前调用。
void PsocAlgo::register_storage() {
#ifdef PICO_PLATFORM
    // 编译期核对两个区的容量: 区变小/结构变大都在编译时就炸, 而不是等落盘静默截断。
    static_assert(sizeof(Mirror) <= NvStore::payload_capacity(NvStore::Region::ALGO_BIN),
                  "ALGO_BIN 镜像超出该区一份的负载容量");
    static_assert(PSOC_ALGO_SRC_MAX <= NvStore::payload_capacity(NvStore::Region::ALGO_SRC),
                  "ALGO_SRC 上限超出该区一份的负载容量");
    NvStore* nv = NvStore::getInstance();
    _mirror_len = sizeof(_mirror);
    nv->register_blob(NvStore::Region::ALGO_BIN, (uint8_t*)&_mirror, &_mirror_len, sizeof(_mirror));
    _src_store_len = 0;
    nv->register_blob(NvStore::Region::ALGO_SRC, _src, &_src_store_len, PSOC_ALGO_SRC_MAX);
#endif
}
