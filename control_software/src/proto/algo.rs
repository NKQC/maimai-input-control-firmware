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
pub const ALGO_MAX_LEN: usize = 1024;

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
#[derive(Debug, Clone, Copy)]
pub struct AlgoInfo {
    pub is_default: bool,
    pub psoc_valid: bool,
    pub len: u16,
    pub crc16: u16,
}

pub fn encode_algo_get_info(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetInfo as u8, 0, seq, vec![])
}

/// 响应 [is_default(u8), psoc_valid(u8), len(u16 LE), crc16(u16 LE)]
pub fn decode_algo_info(payload: &[u8]) -> Option<AlgoInfo> {
    if payload.len() < 6 {
        return None;
    }
    Some(AlgoInfo {
        is_default: payload[0] != 0,
        psoc_valid: payload[1] != 0,
        len: u16::from_le_bytes([payload[2], payload[3]]),
        crc16: u16::from_le_bytes([payload[4], payload[5]]),
    })
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

/// 算法 C 源(映射表)存储上限, 与固件 PSOC_ALGO_SRC_MAX 一致。
pub const ALGO_SRC_MAX: usize = 32768;
/// 单片字节数, 与固件 HOST_CMD_ALGO_SRC_CHUNK 一致。32KB 装不进 4096 的单帧 payload, 必须分片。
pub const ALGO_SRC_CHUNK: usize = 2048;

/// 请求回读算法 C 源的某一片: payload = [offset(u16 LE)]。
pub fn encode_algo_get_src(seq: u8, offset: usize) -> Frame {
    let off = (offset.min(ALGO_SRC_MAX)) as u16;
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

/// 解析旧 `ALGO_SETTING(idx, "name", defval)` 与 META 扩展声明。
pub fn parse_algo_settings(src: &str) -> Vec<AlgoSettingDecl> {
    let mut out = Vec::new();
    for args in _scan_macro_calls(src, "ALGO_SETTING_META") {
        let Some(idx) = args.first().and_then(|v| v.trim().parse().ok()) else {
            continue;
        };
        let default = args.get(3).and_then(|v| v.trim().parse().ok()).unwrap_or(0);
        out.push(AlgoSettingDecl {
            idx,
            name: _meta_text(&args, 1),
            default,
            value_type: _meta_text(&args, 2),
            range: _meta_range(&args, 4),
            description: _meta_text(&args, 6),
            alias: _meta_text(&args, 7),
        });
    }
    for args in _scan_macro_calls(src, "ALGO_SETTING") {
        let Some(idx) = args.first().and_then(|v| v.trim().parse().ok()) else {
            continue;
        };
        if out.iter().any(|decl: &AlgoSettingDecl| decl.idx == idx) {
            continue;
        }
        let name = _meta_text(&args, 1);
        let default = args.get(2).and_then(|v| v.trim().parse().ok()).unwrap_or(0);
        out.push(AlgoSettingDecl {
            idx,
            name,
            default,
            value_type: "u8".to_string(),
            range: "0..255".to_string(),
            description: String::new(),
            alias: _meta_text(&args, 1),
        });
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
