//! 全局 CSD 配置(GLOBAL_*)与 JIT 触控算法(ALGO_*)命令编解码。
//!
//! 与固件保持一致(见 host_cmd.h / jit-algo-engine.md §4):
//! - GLOBAL_GET=0x29 / GLOBAL_SET=0x2A / GLOBAL_GET_ALL=0x2B
//! - ALGO_GET_INFO=0x60 / UPLOAD=0x61 / APPLY=0x62 / RESET_DEFAULT=0x63 / SET_ROM=0x64 / GET_ROM=0x65

use super::{Frame, HostCmd};

// ---- 全局 CSD 参数 id(镜像固件 GPARAM_*)----
pub const GPARAM_INACTIVE_SNS: u8 = 0x01; // 1=GND 2=High-Z 4=Shield
pub const GPARAM_IDAC_GAIN_INIT: u8 = 0x02;
pub const GPARAM_IDAC_MIN: u8 = 0x03;
pub const GPARAM_RAW_TARGET: u8 = 0x04;
pub const GPARAM_MFS_DIV_F1: u8 = 0x05;
pub const GPARAM_MFS_DIV_F2: u8 = 0x06;
pub const GPARAM_IDAC_SENSE_CONFIG: u8 = 0x07; // 0=IDAC sourcing, 1=IDAC sinking
pub const GPARAM_AUTO_CALIBRATE_EN: u8 = 0x08; // 0=固定IDAC(不自动校准), 1=自动校准
pub const KNOWN_GLOBAL_IDS: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

// 未激活传感器连接模式(GPARAM_INACTIVE_SNS 取值)
pub const SNS_CONN_GROUND: u32 = 1;
pub const SNS_CONN_HIGHZ: u32 = 2;
pub const SNS_CONN_SHIELD: u32 = 4;

pub const ALGO_CHANNELS: usize = 36;

// ---- 容量常量: 全部是**兜底值**, 仅在尚未从设备回读到容量前使用 ----
//
// ★为什么不能再当成事实★ 槽容量 / 单帧上限 / C 源容量 / 堆容量四个数分散在 PSoC 与 RP 两套固件的
// 常量里, 而上位机可以连到任意一版固件。把编译期常量当真值的代价是: 固件把槽扩到 8KB 之后上位机
// 仍按 4096 卡闸门(用户白等一次编译), 或固件缩到 2KB 而上位机放行(设备静默 NAK, 现场无线索)。
// ⇒ 设备已在 `ALGO_GET_INFO` 的 26 字节响应里如实回报这四个容量, 新代码必须走
//   `AppController::algo_*_capacity()` 那组实例方法; 下面这些常量只是"还没连上/旧固件"时的显示兜底。

/// PSoC 可执行算法槽容量兜底值(字节)。**只用于占用百分比显示**, 不是上传闸门。
pub const ALGO_SLOT_FALLBACK: usize = 4096;

/// 单帧可上传的算法字节上限兜底值 = HOST_CMD_PAYLOAD_MAX(4096) − 4 字节帧内头(len u16 + crc16 u16)。
///
/// ★为什么必须与槽容量分成两个数★
/// 槽是 4096, 但上传帧要先放 len/crc16, 于是 4093..4096 字节的算法**编译得出来却传不进去** ——
/// 设备只会回一个 "len invalid" 的 NAK, 现场没有任何线索指向"少了 4 个字节的帧头"。
/// 合成一个数的代价就是这种查不出根因的失败, 故两者始终并存: 进度条分母用槽, 闸门用上传上限。
pub const ALGO_UPLOAD_FALLBACK: usize = 4092;

/// 算法 C 源存储容量兜底值(字节)。
pub const ALGO_SRC_FALLBACK: usize = 32768;

/// 算法共享堆容量兜底值(字节)。上位机只用它做占用百分比显示。
pub const ALGO_HEAP_FALLBACK: usize = 256;

/// ★保留★ 单帧上传上限的旧名。既有代码(无头自检等)多处引用它, 改名会牵动与本次无关的路径。
/// 它**只是兜底值**, 语义等同 `ALGO_UPLOAD_FALLBACK`; 新代码一律走设备回报值。
pub const ALGO_MAX_LEN: usize = ALGO_UPLOAD_FALLBACK;

/// 逐通道算法配置的槽数(每通道 8 项, 与 ABI v2 的 `algo_io_t::cfg_ch[8]` 一致)。
pub const ALGO_CFG_CH_SLOTS: usize = 8;

/// `ALGO_GET_CFG_CH` 响应的固定长度: 36 通道 × 8 槽, 通道主序。
pub const ALGO_CFG_CH_TOTAL: usize = ALGO_CHANNELS * ALGO_CFG_CH_SLOTS;

/// 逐通道项在**UI 元数据 override** 里的下标偏移。
///
/// ★为什么需要这个偏移★ override 的键是 `(源码指纹, kind, index)`, 而 kind=1(可调变量)只有一套
/// index 空间; 设备侧却有两套 —— `cfg[0]` 与 `cfg_ch[0]` 是两个不同变量。不错开的话, 用户给
/// `cfg_ch[0]` 起的别名会同时显示在 `cfg[0]` 上(反之亦然), 而且后写的那次会把前一次覆盖掉。
/// 逐通道项一律按 `idx + 8` 存取; 两类各只有 8 槽, 故 0..7 归共享、8..15 归逐通道, 不会重叠。
pub const ALGO_CFG_CH_META_BASE: u8 = ALGO_CFG_CH_SLOTS as u8;

/// CRC-16/CCITT-FALSE(poly 0x1021, init 0xFFFF)——与固件/PSoC/默认 blob 三处一致。
/// 上位机编译算法后用它算 blob 校验值,随 ALGO_UPLOAD 下发。
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ---- 全局 CSD 配置 ----
pub fn encode_global_get(seq: u8, gparam_id: u8) -> Frame {
    Frame::new(HostCmd::GlobalGet as u8, 0, seq, vec![gparam_id])
}

pub fn encode_global_set(seq: u8, gparam_id: u8, value: u32) -> Frame {
    let mut p = Vec::with_capacity(5);
    p.push(gparam_id);
    p.extend_from_slice(&value.to_le_bytes());
    Frame::new(HostCmd::GlobalSet as u8, 0, seq, p)
}

pub fn encode_global_get_all(seq: u8) -> Frame {
    Frame::new(HostCmd::GlobalGetAll as u8, 0, seq, vec![])
}

/// 批量全局项下发后, 单次触发 PSoC 完整重初始化(替代逐项 commit, 防重初始化风暴)。
pub fn encode_global_commit(seq: u8) -> Frame {
    Frame::new(HostCmd::GlobalCommit as u8, 0, seq, vec![])
}

/// 响应 [gparam_id, value(u32 LE)]
pub fn decode_global_get(payload: &[u8]) -> Option<(u8, u32)> {
    if payload.len() < 5 {
        return None;
    }
    let id = payload[0];
    let v = u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
    Some((id, v))
}

/// 响应 [count(u8), (gparam_id, value u32 LE)×count]
pub fn decode_global_get_all(payload: &[u8]) -> Vec<(u8, u32)> {
    let mut out = Vec::new();
    if payload.is_empty() {
        return out;
    }
    let count = payload[0] as usize;
    let mut p = 1usize;
    for _ in 0..count {
        if p + 5 > payload.len() {
            break;
        }
        let id = payload[p];
        let v = u32::from_le_bytes([
            payload[p + 1],
            payload[p + 2],
            payload[p + 3],
            payload[p + 4],
        ]);
        out.push((id, v));
        p += 5;
    }
    out
}

// ---- JIT 触控算法 ----

// ALGO_GET_INFO 响应的扩展位(payload[14])。设备侧同名语义, 不在 UI 侧另起名字。
const ALGO_FLAG_QUARANTINED: u8 = 0x01;
const ALGO_FLAG_DOWNLOAD_PENDING: u8 = 0x02;
const ALGO_FLAG_UPLOADING: u8 = 0x04;
const ALGO_FLAG_PSOC_CACHE_UNAVAILABLE: u8 = 0x08;

// 容量组的标志位(payload[25])。同样沿用设备侧语义。
const ALGO_CAPS_SLOT_FROM_PSOC: u8 = 0x01;
const ALGO_CAPS_MISMATCH: u8 = 0x02;

/// ALGO_GET_INFO 的解析结果。
///
/// ★`len`/`crc16` 与 `psoc_len`/`psoc_crc16` 是两回事, 绝不可混用★
/// 前者是 **RP2040 存储**里那份算法的长度/校验(只证明"主机端存下了"),
/// 后者是 **PSoC 槽内实际内容**的长度/校验(commit 时算出, 才能证明"装上并在跑")。
/// 上传成功判据必须用后者: 用前者会把"RP 存下了但 PSoC 没装上"报成成功(实测过的用户 bug)。
#[derive(Debug, Clone, Copy)]
pub struct AlgoInfo {
    pub is_default: bool,
    pub psoc_valid: bool,
    /// RP2040 存储的算法长度(原字段, 旧固件也有)。
    pub len: u16,
    /// RP2040 存储的算法 CRC16(原字段, 旧固件也有)。
    pub crc16: u16,
    /// PSoC 槽内实际长度。`extended == false` 时无意义(恒 0)。
    pub psoc_len: u16,
    /// PSoC 槽内实际内容 CRC16。`extended == false` 时无意义(恒 0)。
    pub psoc_crc16: u16,
    /// 共享堆占用峰值(字节)。`extended == false` 时无意义(恒 0)。
    pub heap_used: u16,
    /// 共享堆容量(字节, = ALGO_HEAP_SIZE)。`extended == false` 时无意义(恒 0)。
    pub heap_size: u16,
    /// 算法已被设备隔离(连续致命挂死), PSoC 退回原生 CapSense 判定。
    pub quarantined: bool,
    /// 设备侧仍有待下发的算法(RP → PSoC 尚未完成)。
    pub download_pending: bool,
    /// 上传/commit 仍在进行 —— 此刻的 psoc_* 是中间态, 不能据此宣判失败。
    pub uploading: bool,
    /// PSoC 侧内容缓存暂不可用(读不到真值), 同样不能据此宣判失败。
    pub psoc_cache_unavailable: bool,

    // ---- 容量组(payload ≥ 26 才有效, 见 caps_known) ----
    // ★为什么容量要从设备取★ 见本文件顶部 *_FALLBACK 的说明: 两套固件各有一份常量, 上位机
    // 硬编码等于赌它连上的是哪一版。这一组是设备自报的事实, UI 的闸门与分母都必须用它。
    /// PSoC 可执行槽容量(字节)。PSoC 读不到时是 RP 的兜底值, 见 `slot_capacity_from_psoc`。
    pub slot_capacity: u16,
    /// 单帧可上传的算法字节上限(= 设备 HOST_CMD_PAYLOAD_MAX − 4)。编译/上传闸门用它。
    pub upload_limit: u16,
    /// 算法 C 源存储容量(字节)。
    pub src_capacity: u32,
    /// 算法 C 源分片粒度(字节)。分片上传/回读的步长必须用它, 否则设备按自己的粒度校验会 NAK。
    pub src_chunk: u16,
    /// `slot_capacity` 来自 PSoC 自报(true)还是 RP 的兜底常量(false)。
    pub slot_capacity_from_psoc: bool,
    /// PSoC 与 RP 两侧的容量常量不一致 —— 必须显式告警: 两级容量对不上时上传可能被一侧放行、
    /// 另一侧静默截断, 表现为"上传成功但算法跑飞", 而任何一侧的日志都看不出原因。
    pub capacity_mismatch: bool,

    /// 设备是否回报了扩展字段(payload ≥ 15)。false = 旧固件, 只有前 6 字节可信。
    pub extended: bool,
    /// 设备是否回报了容量组(payload ≥ 26)。false 时上面那一组恒 0/false, UI 必须显示 "—"。
    pub caps_known: bool,
}

pub fn encode_algo_get_info(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetInfo as u8, 0, seq, vec![])
}

/// 响应(前 6 字节偏移语义与旧固件完全一致, 故旧解析继续成立):
/// ```text
/// [0]      is_default        [1]      psoc_valid
/// [2..3]   store_len(u16)    [4..5]   store_crc16(u16)     ← 旧固件到此为止
/// [6..7]   psoc_len(u16)     [8..9]   psoc_crc16(u16)
/// [10..11] heap_used(u16)    [12..13] heap_size(u16)       [14] flags(u8)
/// [15..16] slot_capacity(u16)         [17..18] upload_limit(u16)
/// [19..22] src_capacity(u32)          [23..24] src_chunk(u16)      [25] caps_flags(u8)
/// ```
///
/// ★必须向后兼容★ payload ≥ 6 就解出旧字段并置 `extended=false`/`caps_known=false`
/// (新字段留 0/false); ≥ 15 才认扩展字段; ≥ 26 才认容量组。
/// 旧固件在网时若直接判长度失败, 整个算法页会退回"未读取" —— 界面上分不出"设备没这功能"与
/// "读失败了", 而这两种要走的下一步完全不同。
pub fn decode_algo_info(payload: &[u8]) -> Option<AlgoInfo> {
    if payload.len() < 6 {
        return None;
    }
    let mut info = AlgoInfo {
        is_default: payload[0] != 0,
        psoc_valid: payload[1] != 0,
        len: u16::from_le_bytes([payload[2], payload[3]]),
        crc16: u16::from_le_bytes([payload[4], payload[5]]),
        psoc_len: 0,
        psoc_crc16: 0,
        heap_used: 0,
        heap_size: 0,
        quarantined: false,
        download_pending: false,
        uploading: false,
        psoc_cache_unavailable: false,
        slot_capacity: 0,
        upload_limit: 0,
        src_capacity: 0,
        src_chunk: 0,
        slot_capacity_from_psoc: false,
        capacity_mismatch: false,
        extended: false,
        caps_known: false,
    };
    if payload.len() >= 15 {
        let flags = payload[14];
        info.psoc_len = u16::from_le_bytes([payload[6], payload[7]]);
        info.psoc_crc16 = u16::from_le_bytes([payload[8], payload[9]]);
        info.heap_used = u16::from_le_bytes([payload[10], payload[11]]);
        info.heap_size = u16::from_le_bytes([payload[12], payload[13]]);
        info.quarantined = (flags & ALGO_FLAG_QUARANTINED) != 0;
        info.download_pending = (flags & ALGO_FLAG_DOWNLOAD_PENDING) != 0;
        info.uploading = (flags & ALGO_FLAG_UPLOADING) != 0;
        info.psoc_cache_unavailable = (flags & ALGO_FLAG_PSOC_CACHE_UNAVAILABLE) != 0;
        info.extended = true;
    }
    if payload.len() >= 26 {
        let caps_flags = payload[25];
        info.slot_capacity = u16::from_le_bytes([payload[15], payload[16]]);
        info.upload_limit = u16::from_le_bytes([payload[17], payload[18]]);
        info.src_capacity =
            u32::from_le_bytes([payload[19], payload[20], payload[21], payload[22]]);
        info.src_chunk = u16::from_le_bytes([payload[23], payload[24]]);
        info.slot_capacity_from_psoc = (caps_flags & ALGO_CAPS_SLOT_FROM_PSOC) != 0;
        info.capacity_mismatch = (caps_flags & ALGO_CAPS_MISMATCH) != 0;
        info.caps_known = true;
    }
    Some(info)
}

/// payload = [len(u16 LE), crc16(u16 LE), data[len]]; crc16 内部按 CCITT-FALSE 计算。
pub fn encode_algo_upload(seq: u8, data: &[u8]) -> Frame {
    let len = data.len() as u16;
    let crc = crc16_ccitt(data);
    let mut p = Vec::with_capacity(4 + data.len());
    p.extend_from_slice(&len.to_le_bytes());
    p.extend_from_slice(&crc.to_le_bytes());
    p.extend_from_slice(data);
    Frame::new(HostCmd::AlgoUpload as u8, 0, seq, p)
}

pub fn encode_algo_apply(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoApply as u8, 0, seq, vec![])
}

pub fn encode_algo_reset_default(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoResetDefault as u8, 0, seq, vec![])
}

/// payload = [ch(u8), rom(u16 LE)]×N
pub fn encode_algo_set_rom(seq: u8, entries: &[(u8, u16)]) -> Frame {
    let mut p = Vec::with_capacity(entries.len() * 3);
    for &(ch, rom) in entries {
        p.push(ch);
        p.extend_from_slice(&rom.to_le_bytes());
    }
    Frame::new(HostCmd::AlgoSetRom as u8, 0, seq, p)
}

pub fn encode_algo_get_rom(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetRom as u8, 0, seq, vec![])
}

/// 响应 36×u16 LE(每通道 ROM 表)
pub fn decode_algo_get_rom(payload: &[u8]) -> Vec<u16> {
    let mut out = Vec::with_capacity(ALGO_CHANNELS);
    let mut p = 0usize;
    while p + 2 <= payload.len() && out.len() < ALGO_CHANNELS {
        out.push(u16::from_le_bytes([payload[p], payload[p + 1]]));
        p += 2;
    }
    out
}

/// 算法 C 源分片粒度的兜底值, 与固件 HOST_CMD_ALGO_SRC_CHUNK 一致。
/// 32KB 装不进 4096 的单帧 payload, 必须分片; 设备在 GET_INFO 里回报真值(`AlgoInfo::src_chunk`)。
pub const ALGO_SRC_CHUNK: usize = 2048;

/// 请求回读算法 C 源的某一片: payload = [offset(u16 LE)]。
///
/// 这里的 offset 钳制用兜底容量而不是设备回报值: 本函数是无状态编码器, 拿不到 `AlgoCaps`;
/// 而 offset 是 u16, 钳制的唯一作用是防止 `as u16` 静默回绕(比如传进一个 70000)。
/// 真正的容量闸门在调用方(`send_algo_src` / `_handle_algo_src_chunk`)用设备值判。
pub fn encode_algo_get_src(seq: u8, offset: usize) -> Frame {
    let off = (offset.min(ALGO_SRC_FALLBACK)) as u16;
    Frame::new(
        HostCmd::AlgoGetSrc as u8,
        0,
        seq,
        off.to_le_bytes().to_vec(),
    )
}

/// payload = [offset(u16 LE), total(u16 LE), chunk]; 存算法 C 源(已滤注释)的一片。
/// offset 必须从 0 开始严格连续, 最后一片(offset+chunk==total)才让设备侧生效。
pub fn encode_algo_set_src_chunk(seq: u8, offset: usize, total: usize, chunk: &[u8]) -> Frame {
    let mut p = Vec::with_capacity(4 + chunk.len());
    p.extend_from_slice(&(offset as u16).to_le_bytes());
    p.extend_from_slice(&(total as u16).to_le_bytes());
    p.extend_from_slice(chunk);
    Frame::new(HostCmd::AlgoSetSrc as u8, 0, seq, p)
}

/// 响应 [total(u16 LE), offset(u16 LE), chunk] → (total, offset, chunk)。
pub fn decode_algo_src_chunk(payload: &[u8]) -> Option<(usize, usize, &[u8])> {
    if payload.len() < 4 {
        return None;
    }
    let total = u16::from_le_bytes([payload[0], payload[1]]) as usize;
    let offset = u16::from_le_bytes([payload[2], payload[3]]) as usize;
    Some((total, offset, &payload[4..]))
}

/// 请求回读算法 ASM 机器码。
pub fn encode_algo_get_code(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetCode as u8, 0, seq, vec![])
}

/// 响应 [len(u16 LE), bytes] → 提取字节切片(源或机器码通用)。
pub fn decode_algo_len_prefixed(payload: &[u8]) -> Vec<u8> {
    if payload.len() < 2 {
        return Vec::new();
    }
    let len = u16::from_le_bytes([payload[0], payload[1]]) as usize;
    let end = (2 + len).min(payload.len());
    payload[2..end].to_vec()
}

// ---- 算法可调变量(cfg[8]) ----
// 运行值(report[]/out_active)不在这里: 它随遥测帧的 FIELD_ALGO 块到达, 没有对应的请求/响应对。

/// payload = [idx(u8), val(u8)]
pub fn encode_algo_set_cfg(seq: u8, idx: u8, val: u8) -> Frame {
    Frame::new(HostCmd::AlgoSetCfg as u8, 0, seq, vec![idx, val])
}

pub fn encode_algo_get_cfg(seq: u8, idx: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetCfg as u8, 0, seq, vec![idx])
}

/// 响应 [idx, cfg(u8)]
pub fn decode_algo_get_cfg(payload: &[u8]) -> Option<(u8, u8)> {
    if payload.len() < 2 {
        return None;
    }
    Some((payload[0], payload[1]))
}

// ---- 逐通道算法可调变量(cfg_ch[8], ABI v2) ----
// 与上面的 cfg[8] 是两套独立下标空间, 命令也各自独立(0x6C/0x6D)。

/// payload = [ch(u8), idx(u8), val(u8)]×N (N ≤ 288)。
///
/// ★必须一帧多条★ 36×8 = 288 项若逐项发帧, 一次"保存到设备"就是 288 个往返 + 288 次 ACK 归因,
/// 足以把有序写队列堵到用户以为程序卡死。整批 864 字节远在单帧 payload 上限之内。
pub fn encode_algo_set_cfg_ch(seq: u8, entries: &[(u8, u8, u8)]) -> Frame {
    let mut p = Vec::with_capacity(entries.len() * 3);
    for &(ch, idx, val) in entries {
        p.push(ch);
        p.push(idx);
        p.push(val);
    }
    Frame::new(HostCmd::AlgoSetCfgCh as u8, 0, seq, p)
}

/// 空请求 → 响应 288 字节(通道主序, 每通道连续 8 字节)。
pub fn encode_algo_get_cfg_ch(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetCfgCh as u8, 0, seq, vec![])
}

/// 响应 288 字节 → 每通道一组 8 值。
/// ★不足 288 字节按已收到的完整通道数返回, 不 panic★: 截断响应的正确含义是"只拿回了前几个通道",
/// 把它当致命错误会让一次链路抖动清掉整页显示; 半个通道的残组直接丢弃(宁可少一条也不给假值)。
pub fn decode_algo_get_cfg_ch(payload: &[u8]) -> Vec<[u8; ALGO_CFG_CH_SLOTS]> {
    let complete = payload.len() / ALGO_CFG_CH_SLOTS;
    let count = complete.min(ALGO_CHANNELS);
    let mut out = Vec::with_capacity(count);
    for ch in 0..count {
        let base = ch * ALGO_CFG_CH_SLOTS;
        let mut row = [0u8; ALGO_CFG_CH_SLOTS];
        row.copy_from_slice(&payload[base..base + ALGO_CFG_CH_SLOTS]);
        out.push(row);
    }
    out
}

/// 算法上报变量声明。旧格式仅提供 idx/name；META 格式补充类型、范围、介绍和别名。
#[derive(Debug, Clone)]
pub struct AlgoReportDecl {
    pub idx: u8,
    pub name: String,
    pub value_type: String,
    pub range: String,
    pub description: String,
    pub alias: String,
    /// 声明层面的"是否二值量"。`None` = 旧式 `ALGO_REPORT(idx,name)`，它压根没带类型，
    /// 这里的未知是真未知，只有这种情况才允许退回按运行数据反推。
    /// META 声明一律给出确定结论：类型/范围是已知事实，不该等数据到齐才认。
    pub declared_binary: Option<bool>,
}

/// 算法可设置变量声明。default 保持 u8 兼容设备 cfg[8]，其余字段来自 META 声明。
#[derive(Debug, Clone)]
pub struct AlgoSettingDecl {
    pub idx: u8,
    pub name: String,
    pub default: u8,
    pub value_type: String,
    pub range: String,
    pub description: String,
    pub alias: String,
    /// true = 逐通道项(`ALGO_SETTING_CH*`, 对应设备 `cfg_ch[idx]`, 每通道各一份);
    /// false = 全通道共享项(`ALGO_SETTING*`, 对应设备 `cfg[idx]`, 一份值对 36 通道生效)。
    /// ★这两类的 idx 属于**互相独立**的下标空间★, 只有 (per_channel, idx) 合起来才唯一标识一个变量。
    pub per_channel: bool,
}

fn _meta_text(args: &[String], index: usize) -> String {
    args.get(index).map(|v| _unquote(v)).unwrap_or_default()
}

fn _meta_range(args: &[String], min_index: usize) -> String {
    match (args.get(min_index), args.get(min_index + 1)) {
        (Some(min), Some(max)) => format!("{}..{}", min.trim(), max.trim()),
        _ => String::new(),
    }
}

/// META 声明的二值判定：类型名点明 bool，或声明范围恰为 `0..1`。
fn _meta_is_binary(value_type: &str, range: &str) -> bool {
    value_type.trim().eq_ignore_ascii_case("bool") || range.trim() == "0..1"
}

/// 从算法 C 源解析旧声明与稳定的 META 扩展声明：
/// `ALGO_REPORT_META(idx, "name", "type", min, max, "description", "alias")`。
pub fn parse_algo_reports(src: &str) -> Vec<AlgoReportDecl> {
    let mut out = Vec::new();
    for args in _scan_macro_calls(src, "ALGO_REPORT_META") {
        let Some(idx) = args.first().and_then(|v| v.trim().parse().ok()) else {
            continue;
        };
        let value_type = _meta_text(&args, 2);
        let range = _meta_range(&args, 3);
        out.push(AlgoReportDecl {
            idx,
            name: _meta_text(&args, 1),
            declared_binary: Some(_meta_is_binary(&value_type, &range)),
            value_type,
            range,
            description: _meta_text(&args, 5),
            alias: _meta_text(&args, 6),
        });
    }
    for args in _scan_macro_calls(src, "ALGO_REPORT") {
        let Some(idx) = args.first().and_then(|v| v.trim().parse().ok()) else {
            continue;
        };
        if out.iter().any(|decl: &AlgoReportDecl| decl.idx == idx) {
            continue;
        }
        let name = _meta_text(&args, 1);
        out.push(AlgoReportDecl {
            idx,
            alias: String::new(),
            name,
            // 旧式声明只有 idx/name; 下面两项是**展示用兜底**, 不是声明事实,
            // 所以 declared_binary 必须是 None(未知), 不能被兜底的 u16 冒充成"已知非二值"。
            value_type: "u16".to_string(),
            range: "0..65535".to_string(),
            declared_binary: None,
            description: String::new(),
        });
    }
    out
}

/// 解析四种可设置变量声明: 共享项 `ALGO_SETTING[_META]`(→ `cfg[]`)与
/// 逐通道项 `ALGO_SETTING_CH[_META]`(→ `cfg_ch[]`)。
///
/// 解析顺序仍是"META 优先、旧式补充": 同一变量若两种声明都在, META 携带类型/范围/别名, 更完整。
///
/// ★去重键必须是 `(per_channel, idx)` 而不是 `idx`★
/// 两类的下标空间互相独立 —— `ALGO_SETTING(0, "guard")` 与 `ALGO_SETTING_CH(0, "rise")` 是
/// **两个不同的变量**(分别落在 cfg[0] 与 cfg_ch[0])。只按 idx 去重会把后解析到的那一个整条吞掉,
/// 界面上表现为"算法明明声明了 9 项, 只出来 5 项", 且丢的是哪几项取决于扫描顺序。
///
/// ★`_scan_macro_calls` 的前缀陷阱★ 扫 `ALGO_SETTING` 会连 `ALGO_SETTING_CH(...)` 一起命中吗?
/// 不会: 它要求宏名之后紧跟(可含空白)`(`, 而 `ALGO_SETTING_CH` 的下一个字符是 `_`。
/// 但扫 `ALGO_SETTING_META` 与 `ALGO_SETTING_CH_META` 时反过来也成立, 故四轮互不串味。
pub fn parse_algo_settings(src: &str) -> Vec<AlgoSettingDecl> {
    let mut out: Vec<AlgoSettingDecl> = Vec::new();
    // (宏名, 是否逐通道, 是否 META 形式)。META 在前 ⇒ 旧式声明只用于补齐 META 没覆盖到的项。
    const MACROS: [(&str, bool, bool); 4] = [
        ("ALGO_SETTING_META", false, true),
        ("ALGO_SETTING_CH_META", true, true),
        ("ALGO_SETTING", false, false),
        ("ALGO_SETTING_CH", true, false),
    ];
    for (macro_name, per_channel, is_meta) in MACROS {
        for args in _scan_macro_calls(src, macro_name) {
            let Some(idx) = args.first().and_then(|v| v.trim().parse().ok()) else {
                continue;
            };
            if out
                .iter()
                .any(|decl| decl.per_channel == per_channel && decl.idx == idx)
            {
                continue;
            }
            let decl = if is_meta {
                // META 七参形式(两类同形): (idx, name, type, defval, minval, maxval, description, alias)
                AlgoSettingDecl {
                    idx,
                    name: _meta_text(&args, 1),
                    default: args.get(3).and_then(|v| v.trim().parse().ok()).unwrap_or(0),
                    value_type: _meta_text(&args, 2),
                    range: _meta_range(&args, 4),
                    description: _meta_text(&args, 6),
                    alias: _meta_text(&args, 7),
                    per_channel,
                }
            } else {
                // 旧式三参形式: (idx, name, defval)。类型/范围退回 u8/0..255 —— 那是协议事实
                // (设备 cfg/cfg_ch 都是 u8 数组), 不是猜的。
                AlgoSettingDecl {
                    idx,
                    name: _meta_text(&args, 1),
                    default: args.get(2).and_then(|v| v.trim().parse().ok()).unwrap_or(0),
                    value_type: "u8".to_string(),
                    range: "0..255".to_string(),
                    description: String::new(),
                    alias: _meta_text(&args, 1),
                    per_channel,
                }
            };
            out.push(decl);
        }
    }
    out
}

/// 去掉字符串字面量两端的双引号(若有)并 trim 空白。
fn _unquote(raw: &str) -> String {
    let t = raw.trim();
    let t = t.strip_prefix('"').unwrap_or(t);
    let t = t.strip_suffix('"').unwrap_or(t);
    t.to_string()
}

/// 逐字符扫描源码, 找形如 `macro_name(arg0, arg1, ...)` 的调用, 返回每次调用的逗号切分参数列表
/// (顶层逗号, 不切字符串内部/嵌套括号内部的逗号)。用于解析 ALGO_REPORT/ALGO_SETTING 声明,
/// 不依赖 regex crate(未在 Cargo.toml 引入)。
fn _scan_macro_calls(src: &str, macro_name: &str) -> Vec<Vec<String>> {
    let bytes = src.as_bytes();
    let mname = macro_name.as_bytes();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i + mname.len() < bytes.len() {
        if &bytes[i..i + mname.len()] == mname {
            // 前一字符若是标识符字符, 说明匹配到的是别的更长标识符的子串, 跳过。
            let prev_is_ident = i > 0 && _is_ident_byte(bytes[i - 1]);
            let mut j = i + mname.len();
            while j < bytes.len() && (bytes[j] == b' ' || bytes[j] == b'\t') {
                j += 1;
            }
            if !prev_is_ident && j < bytes.len() && bytes[j] == b'(' {
                if let Some((args, end)) = _parse_paren_args(src, j) {
                    out.push(args);
                    i = end;
                    continue;
                }
            }
        }
        i += 1;
    }
    out
}

fn _is_ident_byte(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// 从 `(` 所在字节偏移开始解析括号内参数, 按顶层逗号切分(忽略字符串内部/嵌套括号内的逗号)。
/// 返回 (参数列表, 闭括号之后的字节偏移)。
fn _parse_paren_args(src: &str, open_paren_pos: usize) -> Option<(Vec<String>, usize)> {
    let bytes = src.as_bytes();
    if bytes.get(open_paren_pos) != Some(&b'(') {
        return None;
    }
    let mut depth = 0i32;
    let mut in_str = false;
    let mut args = Vec::new();
    // ★按字节累积、成段再解码★ 原实现用 `bytes[i] as char` 逐字节推入 String, 会把任何
    // UTF-8 多字节序列拆成若干 U+0080..U+00FF 字符(mojibake)。于是即便 C 源本身编码完好,
    // 解析出的宏参数也是乱码 —— 这是"回灌干净源后界面仍乱码"的直接原因。
    // 逐字节扫描本身安全: 分隔符 ( ) , " \ 全是 ASCII, 且 UTF-8 自同步。
    let mut cur: Vec<u8> = Vec::new();
    let flush = |cur: &Vec<u8>| String::from_utf8_lossy(cur).trim().to_string();
    let mut i = open_paren_pos;
    while i < bytes.len() {
        let b = bytes[i];
        if in_str {
            cur.push(b);
            if b == b'\\' && i + 1 < bytes.len() {
                cur.push(bytes[i + 1]);
                i += 2;
                continue;
            }
            if b == b'"' {
                in_str = false;
            }
            i += 1;
            continue;
        }
        match b {
            b'"' => {
                in_str = true;
                cur.push(b);
            }
            b'(' => {
                depth += 1;
                if depth > 1 {
                    cur.push(b);
                }
            }
            b')' => {
                depth -= 1;
                if depth == 0 {
                    let text = flush(&cur);
                    if !text.is_empty() {
                        args.push(text);
                    }
                    return Some((args, i + 1));
                }
                cur.push(b);
            }
            b',' if depth == 1 => {
                args.push(flush(&cur));
                cur.clear();
            }
            _ => cur.push(b),
        }
        i += 1;
    }
    None // 未闭合括号
}
