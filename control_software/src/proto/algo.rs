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
        let v = u32::from_le_bytes([payload[p + 1], payload[p + 2], payload[p + 3], payload[p + 4]]);
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
pub const ALGO_SRC_MAX: usize = 3072;

/// 请求回读算法 C 源(映射表)。
pub fn encode_algo_get_src(seq: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetSrc as u8, 0, seq, vec![])
}

/// payload = [len(u16 LE), src bytes]; 存算法 C 源(已滤注释)。
pub fn encode_algo_set_src(seq: u8, src: &[u8]) -> Frame {
    let len = src.len().min(ALGO_SRC_MAX) as u16;
    let mut p = Vec::with_capacity(2 + len as usize);
    p.extend_from_slice(&len.to_le_bytes());
    p.extend_from_slice(&src[..len as usize]);
    Frame::new(HostCmd::AlgoSetSrc as u8, 0, seq, p)
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

// ---- 算法运行时追踪(report[]/out_active) + 可调变量(cfg[8]) ----

/// payload = [ch(u8), idx(u8)]
pub fn encode_algo_get_trace(seq: u8, ch: u8, idx: u8) -> Frame {
    Frame::new(HostCmd::AlgoGetTrace as u8, 0, seq, vec![ch, idx])
}

/// 响应 [ch, out_active(u8), report(u16 LE)]
pub fn decode_algo_get_trace(payload: &[u8]) -> Option<(u8, bool, u16)> {
    if payload.len() < 4 {
        return None;
    }
    let ch = payload[0];
    let active = payload[1] != 0;
    let report = u16::from_le_bytes([payload[2], payload[3]]);
    Some((ch, active, report))
}

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

/// 算法上报变量声明: `ALGO_REPORT(idx, "name")`。idx 对应 io->report[idx](0..3)。
#[derive(Debug, Clone)]
pub struct AlgoReportDecl {
    pub idx: u8,
    pub name: String,
}

/// 算法可设置变量声明: `ALGO_SETTING(idx, "name", defval)`。idx 对应 io->cfg[idx](0..7)。
#[derive(Debug, Clone)]
pub struct AlgoSettingDecl {
    pub idx: u8,
    pub name: String,
    pub default: u8,
}

/// 从算法 C 源里手写扫描 `ALGO_REPORT(idx, "name")` 声明(宏在 ABI 头里展开为空, 仅供上位机
/// grep 源码取 schema)。不引入 regex 依赖, 用简单的逐字符扫描定位调用形如 `IDENT(...)`。
pub fn parse_algo_reports(src: &str) -> Vec<AlgoReportDecl> {
    _scan_macro_calls(src, "ALGO_REPORT")
        .into_iter()
        .filter_map(|args| {
            let idx: u8 = args.first()?.trim().parse().ok()?;
            let name = _unquote(args.get(1)?);
            Some(AlgoReportDecl { idx, name })
        })
        .collect()
}

/// 从算法 C 源里扫描 `ALGO_SETTING(idx, "name", defval)` 声明。
pub fn parse_algo_settings(src: &str) -> Vec<AlgoSettingDecl> {
    _scan_macro_calls(src, "ALGO_SETTING")
        .into_iter()
        .filter_map(|args| {
            let idx: u8 = args.first()?.trim().parse().ok()?;
            let name = _unquote(args.get(1)?);
            let default: u8 = args.get(2)?.trim().parse().ok()?;
            Some(AlgoSettingDecl { idx, name, default })
        })
        .collect()
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
    let mut cur = String::new();
    let mut i = open_paren_pos;
    while i < bytes.len() {
        let c = bytes[i] as char;
        if in_str {
            cur.push(c);
            if c == '\\' && i + 1 < bytes.len() {
                cur.push(bytes[i + 1] as char);
                i += 2;
                continue;
            }
            if c == '"' {
                in_str = false;
            }
            i += 1;
            continue;
        }
        match c {
            '"' => { in_str = true; cur.push(c); }
            '(' => {
                depth += 1;
                if depth > 1 { cur.push(c); }
            }
            ')' => {
                depth -= 1;
                if depth == 0 {
                    if !cur.trim().is_empty() { args.push(cur.trim().to_string()); }
                    return Some((args, i + 1));
                }
                cur.push(c);
            }
            ',' if depth == 1 => {
                args.push(cur.trim().to_string());
                cur.clear();
            }
            _ => cur.push(c),
        }
        i += 1;
    }
    None   // 未闭合括号
}
