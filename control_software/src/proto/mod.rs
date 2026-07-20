//! host_cmd 二进制帧协议编解码 (#6b)
//!
//! 与固件 `main_firmware/src/protocol/host_cmd/host_cmd.h` 保持一致,双向通用帧格式:
//!
//! ```text
//! SOF0(0xAA) SOF1(0x55) cmd flags seq len(u16 LE) payload[len] crc16(u16 LE)
//! ```
//!
//! - CRC-16/CCITT-FALSE(poly 0x1021, init 0xFFFF),覆盖 cmd..payload。
//! - `HostCmd` 命令码分区(与固件 enum class HostCmd 对齐):
//!   - 系统域 0x01-0x0F: HELLO/DEVICE_INFO/PING/SAVE_CONFIG/RESET_DEFAULTS
//!   - 配置KV域 0x10-0x1F: CFG_GET/CFG_SET/CFG_GET_GROUP/CFG_GET_ALL/CFG_SET_BATCH
//!   - CapSense调参域 0x20-0x2F: PARAM_GET/PARAM_SET/PARAM_GET_ALL/CALIBRATE/BASELINE_RESET
//!   - 遥测流域 0x30-0x3F: TELEM_START/TELEM_STOP/TELEM_DATA
//!   - 绑区域 0x40-0x4F: BIND_START/BIND_ABORT/BIND_CONFIRM/BIND_GET_MAP/BIND_SET_MAP/BIND_EVENT
//!   - 灯效域 0x50-0x5F: LED_GET/LED_SET_REGION/LED_PREVIEW
//!   - 应答 0x7E-0x7F: ACK/NAK

#![allow(dead_code)]

pub mod config;
pub mod telemetry;

use std::convert::TryFrom;
pub use config::{CfgValue, ConfigEntry, ConfigValueType, decode_entries, decode_entry, encode_entry};
pub use telemetry::{
    ChannelSample, FIELD_RAW, FIELD_BASELINE, FIELD_DIFF, FIELD_STATUS, FIELD_STATS, FIELD_LATENCY,
    KNOWN_PARAM_IDS, PARAM_FINGER_TH, PARAM_NOISE_TH, PARAM_NEG_NOISE_TH,
    PARAM_HYSTERESIS, PARAM_ON_DEBOUNCE, PARAM_LOW_BSLN_RST, PARAM_RESOLUTION,
    PARAM_SNS_CLK_DIV, PARAM_IDAC_MOD, PARAM_SNS_CLK_SOURCE, PARAM_IDAC_GAIN,
    encode_telem_start, encode_param_get, encode_param_set, encode_cp_measure, encode_cp_get,
    encode_param_get_all, encode_ch_mask, decode_telem_data, decode_param_get, decode_param_get_all,
    decode_cp_get,
};

// ============================================================================
// Frame Constants
// ============================================================================

/// Frame start magic byte 0
const SOF0: u8 = 0xAA;
/// Frame start magic byte 1
const SOF1: u8 = 0x55;
/// Maximum payload size (bytes)
const PAYLOAD_MAX: usize = 4096;
/// CRC-16/CCITT-FALSE polynomial
const CRC_POLY: u16 = 0x1021;
/// CRC-16/CCITT-FALSE initial value
const CRC_INIT: u16 = 0xFFFF;

// Flags bits
const FLAG_RESPONSE: u8 = 0x01;  // bit0=1: response frame
const FLAG_STREAM: u8 = 0x02;    // bit1=1: stream data frame
const FLAG_NAK_ERR: u8 = 0x04;   // bit2=1: NAK (error)

// ============================================================================
// HostCmd Enumeration (mirrors firmware host_cmd.h)
// ============================================================================

/// Host Command enum - mirrors firmware HostCmd with repr(u8)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum HostCmd {
    // System domain 0x01-0x0F
    Hello = 0x01,
    DeviceInfo = 0x02,
    Ping = 0x03,
    Reboot = 0x04,
    RebootBootloader = 0x05,
    SaveConfig = 0x0E,
    ResetDefaults = 0x0F,

    // Config KV domain 0x10-0x1F
    CfgGet = 0x10,
    CfgSet = 0x11,
    CfgGetGroup = 0x12,
    CfgGetAll = 0x13,
    CfgSetBatch = 0x14,

    // CapSense param domain 0x20-0x2F
    ParamGet = 0x20,
    ParamSet = 0x21,
    ParamGetAll = 0x22,
    Calibrate = 0x23,
    BaselineReset = 0x24,
    ModeSet = 0x25,
    CsdCapture = 0x26,
    CpMeasure = 0x27,
    CpGet = 0x28,

    // Telemetry stream domain 0x30-0x3F
    TelemStart = 0x30,
    TelemStop = 0x31,
    TelemData = 0x32,

    // Binding domain 0x40-0x4F
    BindStart = 0x40,
    BindAbort = 0x41,
    BindConfirm = 0x42,
    BindGetMap = 0x43,
    BindSetMap = 0x44,
    BindEvent = 0x45,

    // LED domain 0x50-0x5F
    LedGet = 0x50,
    LedSetRegion = 0x51,
    LedPreview = 0x52,

    // Response codes 0x7E-0x7F
    Ack = 0x7E,
    Nak = 0x7F,
}

impl TryFrom<u8> for HostCmd {
    type Error = HostCmdError;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        use HostCmd::*;
        match v {
            0x01 => Ok(Hello),
            0x02 => Ok(DeviceInfo),
            0x03 => Ok(Ping),
            0x04 => Ok(Reboot),
            0x05 => Ok(RebootBootloader),
            0x0E => Ok(SaveConfig),
            0x0F => Ok(ResetDefaults),
            0x10 => Ok(CfgGet),
            0x11 => Ok(CfgSet),
            0x12 => Ok(CfgGetGroup),
            0x13 => Ok(CfgGetAll),
            0x14 => Ok(CfgSetBatch),
            0x20 => Ok(ParamGet),
            0x21 => Ok(ParamSet),
            0x22 => Ok(ParamGetAll),
            0x23 => Ok(Calibrate),
            0x24 => Ok(BaselineReset),
            0x25 => Ok(ModeSet),
            0x26 => Ok(CsdCapture),
            0x27 => Ok(CpMeasure),
            0x28 => Ok(CpGet),
            0x30 => Ok(TelemStart),
            0x31 => Ok(TelemStop),
            0x32 => Ok(TelemData),
            0x40 => Ok(BindStart),
            0x41 => Ok(BindAbort),
            0x42 => Ok(BindConfirm),
            0x43 => Ok(BindGetMap),
            0x44 => Ok(BindSetMap),
            0x45 => Ok(BindEvent),
            0x50 => Ok(LedGet),
            0x51 => Ok(LedSetRegion),
            0x52 => Ok(LedPreview),
            0x7E => Ok(Ack),
            0x7F => Ok(Nak),
            _ => Err(HostCmdError::Unknown),
        }
    }
}

impl From<HostCmd> for u8 {
    fn from(cmd: HostCmd) -> u8 {
        cmd as u8
    }
}

// ============================================================================
// HostCmdError Enumeration
// ============================================================================

/// Error codes returned in NAK responses
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HostCmdError {
    NotImplemented = 0x01,
    InvalidParam = 0x02,
    DeviceBusy = 0x03,
    ConfigError = 0x04,
    SensorError = 0x05,
    /// Unknown error code encountered during parsing
    Unknown = 0xFF,
}

impl TryFrom<u8> for HostCmdError {
    type Error = ();

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        use HostCmdError::*;
        match v {
            0x01 => Ok(NotImplemented),
            0x02 => Ok(InvalidParam),
            0x03 => Ok(DeviceBusy),
            0x04 => Ok(ConfigError),
            0x05 => Ok(SensorError),
            _ => Err(()),
        }
    }
}

impl From<HostCmdError> for u8 {
    fn from(err: HostCmdError) -> u8 {
        err as u8
    }
}

// ============================================================================
// Frame Structure
// ============================================================================

/// Parsed binary frame
#[derive(Debug, Clone)]
pub struct Frame {
    pub cmd: u8,
    pub flags: u8,
    pub seq: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    /// Create a new frame
    pub fn new(cmd: u8, flags: u8, seq: u8, payload: Vec<u8>) -> Self {
        Frame { cmd, flags, seq, payload }
    }

    /// Create a HELLO frame
    pub fn hello(seq: u8) -> Self {
        Frame::new(HostCmd::Hello as u8, 0, seq, vec![])
    }

    /// Create a PING frame
    pub fn ping(seq: u8) -> Self {
        Frame::new(HostCmd::Ping as u8, 0, seq, vec![])
    }

    /// Create a REBOOT frame (0x04)
    pub fn reboot(seq: u8) -> Self {
        Frame::new(HostCmd::Reboot as u8, 0, seq, vec![])
    }

    /// Create a REBOOT_BOOTLOADER frame (0x05)
    pub fn reboot_bootloader(seq: u8) -> Self {
        Frame::new(HostCmd::RebootBootloader as u8, 0, seq, vec![])
    }

    /// Create an ACK response frame
    pub fn ack(seq: u8) -> Self {
        Frame::new(HostCmd::Ack as u8, FLAG_RESPONSE, seq, vec![])
    }

    /// Create a NAK response frame
    pub fn nak(seq: u8, err: HostCmdError, msg: &[u8]) -> Self {
        let mut payload = vec![err as u8];
        payload.extend_from_slice(msg);
        Frame::new(HostCmd::Nak as u8, FLAG_RESPONSE | FLAG_NAK_ERR, seq, payload)
    }
}

// ============================================================================
// CRC-16/CCITT-FALSE Implementation
// ============================================================================

fn crc16(data: &[u8]) -> u16 {
    let mut crc = CRC_INIT;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ CRC_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
fn verify_crc_checksum() -> bool {
    // CRC test vector: "123456789" should yield 0x29B1
    let test_data = b"123456789";
    let result = crc16(test_data);
    result == 0x29B1
}

// ============================================================================
// Frame Encoding
// ============================================================================

/// Encode a frame to bytes including SOF, CRC, and LE multi-byte values
pub fn encode(frame: &Frame) -> Vec<u8> {
    let mut buf = Vec::new();

    // SOF magic bytes
    buf.push(SOF0);
    buf.push(SOF1);

    // Header: cmd, flags, seq
    buf.push(frame.cmd);
    buf.push(frame.flags);
    buf.push(frame.seq);

    // Length (u16 LE)
    let len = frame.payload.len() as u16;
    buf.push((len & 0xFF) as u8);
    buf.push((len >> 8) as u8);

    // Payload
    buf.extend_from_slice(&frame.payload);

    // CRC-16 over cmd..payload (skipping SOF bytes)
    let crc_data = &buf[2..];  // start from cmd
    let crc = crc16(crc_data);
    buf.push((crc & 0xFF) as u8);
    buf.push((crc >> 8) as u8);

    buf
}

// ============================================================================
// Frame Decoding State Machine
// ============================================================================

/// State machine for incremental frame decoding
#[derive(Debug)]
pub struct Decoder {
    state: DecoderState,
    header: [u8; 5],  // cmd, flags, seq, len_lo, len_hi
    header_pos: usize,
    payload_len: usize,
    payload: Vec<u8>,
    payload_pos: usize,
    crc_bytes: [u8; 2],
    crc_pos: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderState {
    FindSof0,
    FindSof1,
    ReadHeader,
    ReadPayload,
    ReadCrc,
}

impl Decoder {
    /// Create a new decoder
    pub fn new() -> Self {
        Decoder {
            state: DecoderState::FindSof0,
            header: [0; 5],
            header_pos: 0,
            payload_len: 0,
            payload: Vec::with_capacity(PAYLOAD_MAX),
            payload_pos: 0,
            crc_bytes: [0; 2],
            crc_pos: 0,
        }
    }

    /// Feed a single byte and return parsed frame if complete
    pub fn feed(&mut self, byte: u8) -> Option<Frame> {
        loop {
            match self.state {
                DecoderState::FindSof0 => {
                    if byte == SOF0 {
                        self.state = DecoderState::FindSof1;
                    }
                    return None;
                }
                DecoderState::FindSof1 => {
                    if byte == SOF1 {
                        self.state = DecoderState::ReadHeader;
                        self.header_pos = 0;
                    } else if byte == SOF0 {
                        // Stay in FindSof1, might be start of new frame
                        self.state = DecoderState::FindSof1;
                    } else {
                        self.state = DecoderState::FindSof0;
                    }
                    return None;
                }
                DecoderState::ReadHeader => {
                    self.header[self.header_pos] = byte;
                    self.header_pos += 1;
                    if self.header_pos == 5 {
                        self.payload_len =
                            (self.header[3] as usize) | ((self.header[4] as usize) << 8);
                        if self.payload_len > PAYLOAD_MAX {
                            // Payload too large, reset
                            self.state = DecoderState::FindSof0;
                            return None;
                        }
                        self.payload.clear();
                        self.payload_pos = 0;
                        if self.payload_len == 0 {
                            // No payload, go directly to CRC
                            self.state = DecoderState::ReadCrc;
                            self.crc_pos = 0;
                        } else {
                            self.state = DecoderState::ReadPayload;
                        }
                    }
                    return None;
                }
                DecoderState::ReadPayload => {
                    self.payload.push(byte);
                    self.payload_pos += 1;
                    if self.payload_pos == self.payload_len {
                        self.state = DecoderState::ReadCrc;
                        self.crc_pos = 0;
                    }
                    return None;
                }
                DecoderState::ReadCrc => {
                    self.crc_bytes[self.crc_pos] = byte;
                    self.crc_pos += 1;
                    if self.crc_pos == 2 {
                        // Frame complete, verify CRC
                        let mut check_data = Vec::new();
                        check_data.extend_from_slice(&self.header);
                        check_data.extend_from_slice(&self.payload);
                        let expected_crc = crc16(&check_data);
                        let received_crc =
                            (self.crc_bytes[0] as u16) | ((self.crc_bytes[1] as u16) << 8);

                        self.state = DecoderState::FindSof0;

                        if expected_crc == received_crc {
                            let frame = Frame {
                                cmd: self.header[0],
                                flags: self.header[1],
                                seq: self.header[2],
                                payload: self.payload.clone(),
                            };
                            return Some(frame);
                        }
                        // CRC mismatch, continue looking for SOF
                    }
                    return None;
                }
            }
        }
    }

    /// Feed multiple bytes and collect all complete frames
    pub fn feed_bytes(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();
        for &byte in bytes {
            if let Some(frame) = self.feed(byte) {
                frames.push(frame);
            }
        }
        frames
    }

    /// Reset decoder state
    pub fn reset(&mut self) {
        *self = Decoder::new();
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Decoder::new()
    }
}

// ============================================================================
// DEVICE_INFO Helper Structure
// ============================================================================

/// Bit assignments in `PsocBringupDiagnostics::flags`.
pub const BRINGUP_FLAG_INDICATOR_APP: u16 = 1 << 0;
pub const BRINGUP_FLAG_INDICATOR_SWD: u16 = 1 << 1;
pub const BRINGUP_FLAG_ACQUIRED: u16 = 1 << 2;
pub const BRINGUP_FLAG_SILICON_ID: u16 = 1 << 3;
pub const BRINGUP_FLAG_PROTECTION: u16 = 1 << 4;
pub const BRINGUP_FLAG_IMO: u16 = 1 << 5;
pub const BRINGUP_FLAG_ERASE: u16 = 1 << 6;
pub const BRINGUP_FLAG_PROGRAM: u16 = 1 << 7;
pub const BRINGUP_FLAG_CHECKSUM: u16 = 1 << 8;
pub const BRINGUP_FLAG_VERIFY_ACQUIRE: u16 = 1 << 9;
pub const BRINGUP_FLAG_VERIFY: u16 = 1 << 10;
pub const BRINGUP_FLAG_LINK: u16 = 1 << 11;
pub const BRINGUP_FLAG_SNAPSHOT: u16 = 1 << 12;
pub const BRINGUP_FLAG_CLOCK_CONFIG: u16 = 1 << 13;
pub const BRINGUP_FLAG_ERASE_SCAN_COMPLETE: u16 = 1 << 14;
pub const RP_BUILD_ID_DIAGNOSTIC_V1: u32 = 0x4D32_4401;
pub const EXPECTED_PSOC_S455_ID: u32 = 0x2570_11B5;

#[derive(Debug, Clone)]
pub struct EraseFailureDiagnostics {
    pub clock_select: u32,
    pub clock_imo_select: u32,
    pub clock_trim1: u32,
    pub clock_trim2: u32,
    pub clock_trim3: u32,
    pub flash_byte_sum: u32,
    pub flash_word_or: u32,
    pub first_nonzero_addr: u32,
    pub first_nonzero_value: u32,
    pub words_read: u32,
}

#[derive(Debug, Clone)]
pub struct PsocBringupDiagnostics {
    pub report_version: u8,
    pub report_length: u8,
    pub rp_build_id: u32,
    pub embedded_psoc_version: u32,
    pub last_stage: u8,
    pub failure_stage: u8,
    pub flags: u16,
    pub actual_silicon_id: u32,
    pub idcode: u32,
    pub chip_protection: u8,
    pub last_srom_status: u32,
    pub acquire_status: u32,
    pub acquire_sysreq: u32,
    pub acquire_delay: u32,
    pub fail_row: u16,
    pub fail_addr: u32,
    pub verify_read: u32,
    pub verify_expect: u32,
    pub erase_status: u32,
    pub program_status: u32,
    pub checksum_srom: u32,
    pub checksum_value: u32,
    pub erase_failure: Option<EraseFailureDiagnostics>,
}

impl PsocBringupDiagnostics {
    pub fn has(&self, flag: u16) -> bool {
        self.flags & flag == flag
    }

    pub fn stage_name(stage: u8) -> &'static str {
        match stage {
            0 => "none",
            1 => "swd-ready",
            2 => "indicator-app",
            3 => "acquire",
            4 => "silicon-id",
            5 => "indicator-swd",
            6 => "protection",
            7 => "imo",
            8 => "erase",
            9 => "program",
            10 => "checksum",
            11 => "verify-acquire",
            12 => "verify",
            13 => "run",
            14 => "complete",
            _ => "unknown",
        }
    }

    pub fn flash_ok(&self) -> bool {
        let required = BRINGUP_FLAG_ACQUIRED | BRINGUP_FLAG_SILICON_ID |
            BRINGUP_FLAG_ERASE | BRINGUP_FLAG_PROGRAM | BRINGUP_FLAG_VERIFY;
        self.flags & required == required
    }
}

/// Parsed DEVICE_INFO response payload. Bytes 0..14 are legacy-compatible;
/// diagnostics is present only on report-capable RP2040 firmware.
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    pub protocol_version: u16,
    pub fw_version: u32,
    pub capsense_channels: u8,
    pub capability_bits: u32,
    pub psoc_generation: u16,
    pub psoc_link_valid: bool,
    pub psoc_snapshot_valid: bool,
    pub diagnostics: Option<PsocBringupDiagnostics>,
}

fn read_u16_le(payload: &[u8], offset: usize) -> Result<u16, String> {
    let bytes = payload.get(offset..offset + 2)
        .ok_or_else(|| format!("DEVICE_INFO missing u16 at {}", offset))?;
    Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
}

fn read_u32_le(payload: &[u8], offset: usize) -> Result<u32, String> {
    let bytes = payload.get(offset..offset + 4)
        .ok_or_else(|| format!("DEVICE_INFO missing u32 at {}", offset))?;
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

impl DeviceInfo {
    pub fn from_payload(payload: &[u8]) -> Result<Self, String> {
        if payload.len() < 8 {
            return Err(format!("DEVICE_INFO payload too short: {} bytes", payload.len()));
        }

        let protocol_version = read_u16_le(payload, 0)?;
        let fw_version = read_u32_le(payload, 2)?;
        let capsense_channels = payload[6];
        let capability_bits = (payload[7] as u32)
            | ((*payload.get(8).unwrap_or(&0) as u32) << 8)
            | ((*payload.get(9).unwrap_or(&0) as u32) << 16)
            | ((*payload.get(10).unwrap_or(&0) as u32) << 24);
        let psoc_generation = (*payload.get(11).unwrap_or(&0) as u16)
            | ((*payload.get(12).unwrap_or(&0) as u16) << 8);
        let psoc_link_valid = *payload.get(13).unwrap_or(&0) != 0;
        let psoc_snapshot_valid = *payload.get(14).unwrap_or(&0) != 0;

        let diagnostics = if payload.len() >= 17 {
            let report_version = payload[15];
            let report_length = payload[16];
            if report_version != 1 {
                return Err(format!("unsupported PSoC report version {}", report_version));
            }
            if report_length < 69 || payload.len() < 15 + report_length as usize {
                return Err(format!(
                    "truncated PSoC report: declared {} bytes, payload {} bytes",
                    report_length, payload.len()
                ));
            }
            let erase_failure = if report_length >= 109 {
                Some(EraseFailureDiagnostics {
                    clock_select: read_u32_le(payload, 84)?,
                    clock_imo_select: read_u32_le(payload, 88)?,
                    clock_trim1: read_u32_le(payload, 92)?,
                    clock_trim2: read_u32_le(payload, 96)?,
                    clock_trim3: read_u32_le(payload, 100)?,
                    flash_byte_sum: read_u32_le(payload, 104)?,
                    flash_word_or: read_u32_le(payload, 108)?,
                    first_nonzero_addr: read_u32_le(payload, 112)?,
                    first_nonzero_value: read_u32_le(payload, 116)?,
                    words_read: read_u32_le(payload, 120)?,
                })
            } else {
                None
            };
            Some(PsocBringupDiagnostics {
                report_version,
                report_length,
                rp_build_id: read_u32_le(payload, 17)?,
                embedded_psoc_version: read_u32_le(payload, 21)?,
                last_stage: payload[25],
                failure_stage: payload[26],
                flags: read_u16_le(payload, 27)?,
                actual_silicon_id: read_u32_le(payload, 29)?,
                idcode: read_u32_le(payload, 33)?,
                chip_protection: payload[37],
                last_srom_status: read_u32_le(payload, 38)?,
                acquire_status: read_u32_le(payload, 42)?,
                acquire_sysreq: read_u32_le(payload, 46)?,
                acquire_delay: read_u32_le(payload, 50)?,
                fail_row: read_u16_le(payload, 54)?,
                fail_addr: read_u32_le(payload, 56)?,
                verify_read: read_u32_le(payload, 60)?,
                verify_expect: read_u32_le(payload, 64)?,
                erase_status: read_u32_le(payload, 68)?,
                program_status: read_u32_le(payload, 72)?,
                checksum_srom: read_u32_le(payload, 76)?,
                checksum_value: read_u32_le(payload, 80)?,
                erase_failure,
            })
        } else {
            None
        };

        Ok(DeviceInfo {
            protocol_version,
            fw_version,
            capsense_channels,
            capability_bits,
            psoc_generation,
            psoc_link_valid,
            psoc_snapshot_valid,
            diagnostics,
        })
    }

    pub fn to_payload(&self) -> Vec<u8> {
        let mut payload = Vec::new();
        payload.extend_from_slice(&self.protocol_version.to_le_bytes());
        payload.extend_from_slice(&self.fw_version.to_le_bytes());
        payload.push(self.capsense_channels);
        payload.extend_from_slice(&self.capability_bits.to_le_bytes());
        payload.extend_from_slice(&self.psoc_generation.to_le_bytes());
        payload.push(self.psoc_link_valid as u8);
        payload.push(self.psoc_snapshot_valid as u8);
        if let Some(report) = &self.diagnostics {
            payload.push(report.report_version);
            payload.push(report.report_length);
            payload.extend_from_slice(&report.rp_build_id.to_le_bytes());
            payload.extend_from_slice(&report.embedded_psoc_version.to_le_bytes());
            payload.push(report.last_stage);
            payload.push(report.failure_stage);
            payload.extend_from_slice(&report.flags.to_le_bytes());
            payload.extend_from_slice(&report.actual_silicon_id.to_le_bytes());
            payload.extend_from_slice(&report.idcode.to_le_bytes());
            payload.push(report.chip_protection);
            payload.extend_from_slice(&report.last_srom_status.to_le_bytes());
            payload.extend_from_slice(&report.acquire_status.to_le_bytes());
            payload.extend_from_slice(&report.acquire_sysreq.to_le_bytes());
            payload.extend_from_slice(&report.acquire_delay.to_le_bytes());
            payload.extend_from_slice(&report.fail_row.to_le_bytes());
            payload.extend_from_slice(&report.fail_addr.to_le_bytes());
            payload.extend_from_slice(&report.verify_read.to_le_bytes());
            payload.extend_from_slice(&report.verify_expect.to_le_bytes());
            payload.extend_from_slice(&report.erase_status.to_le_bytes());
            payload.extend_from_slice(&report.program_status.to_le_bytes());
            payload.extend_from_slice(&report.checksum_srom.to_le_bytes());
            payload.extend_from_slice(&report.checksum_value.to_le_bytes());
            if let Some(erase) = &report.erase_failure {
                payload.extend_from_slice(&erase.clock_select.to_le_bytes());
                payload.extend_from_slice(&erase.clock_imo_select.to_le_bytes());
                payload.extend_from_slice(&erase.clock_trim1.to_le_bytes());
                payload.extend_from_slice(&erase.clock_trim2.to_le_bytes());
                payload.extend_from_slice(&erase.clock_trim3.to_le_bytes());
                payload.extend_from_slice(&erase.flash_byte_sum.to_le_bytes());
                payload.extend_from_slice(&erase.flash_word_or.to_le_bytes());
                payload.extend_from_slice(&erase.first_nonzero_addr.to_le_bytes());
                payload.extend_from_slice(&erase.first_nonzero_value.to_le_bytes());
                payload.extend_from_slice(&erase.words_read.to_le_bytes());
            }
        }
        payload
    }
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_checksum() {
        // Test vector: "123456789" should yield 0x29B1
        assert!(verify_crc_checksum(), "CRC-16/CCITT-FALSE test vector failed");
    }

    #[test]
    fn test_frame_encode_decode() {
        // Create and encode a frame
        let original = Frame::ping(42);
        let encoded = encode(&original);

        // Verify SOF bytes
        assert_eq!(encoded[0], SOF0);
        assert_eq!(encoded[1], SOF1);

        // Decode the frame
        let mut decoder = Decoder::new();
        let decoded_frames = decoder.feed_bytes(&encoded);

        assert_eq!(decoded_frames.len(), 1, "Should decode exactly 1 frame");
        let decoded = &decoded_frames[0];
        assert_eq!(decoded.cmd, original.cmd);
        assert_eq!(decoded.flags, original.flags);
        assert_eq!(decoded.seq, original.seq);
        assert_eq!(decoded.payload, original.payload);
    }

    #[test]
    fn test_frame_with_payload() {
        let payload = b"Hello, World!".to_vec();
        let original = Frame::new(0x10, 0, 5, payload.clone());
        let encoded = encode(&original);

        let mut decoder = Decoder::new();
        let decoded_frames = decoder.feed_bytes(&encoded);

        assert_eq!(decoded_frames.len(), 1);
        let decoded = &decoded_frames[0];
        assert_eq!(decoded.payload, payload);
    }

    #[test]
    fn test_frame_feed_bytes() {
        let frame = Frame::hello(7);
        let encoded = encode(&frame);

        let mut decoder = Decoder::new();
        let mut all_frames = Vec::new();

        // Feed bytes one at a time
        for &byte in &encoded {
            if let Some(f) = decoder.feed(byte) {
                all_frames.push(f);
            }
        }

        assert_eq!(all_frames.len(), 1);
        assert_eq!(all_frames[0].seq, 7);
    }

    #[test]
    fn test_ack_nak_frames() {
        let ack = Frame::ack(10);
        assert_eq!(ack.cmd, HostCmd::Ack as u8);
        assert_eq!(ack.flags & FLAG_RESPONSE, FLAG_RESPONSE);
        assert_eq!(ack.seq, 10);

        let nak = Frame::nak(11, HostCmdError::InvalidParam, b"test error");
        assert_eq!(nak.cmd, HostCmd::Nak as u8);
        assert_eq!(nak.flags & (FLAG_RESPONSE | FLAG_NAK_ERR), FLAG_RESPONSE | FLAG_NAK_ERR);
        assert_eq!(nak.seq, 11);
        assert_eq!(nak.payload[0], HostCmdError::InvalidParam as u8);
    }

    #[test]
    fn test_bad_crc_rejected() {
        let frame = Frame::ping(99);
        let mut encoded = encode(&frame);

        // Corrupt CRC bytes
        let len = encoded.len();
        encoded[len - 2] ^= 0xFF;
        encoded[len - 1] ^= 0xFF;

        let mut decoder = Decoder::new();
        let decoded_frames = decoder.feed_bytes(&encoded);

        assert_eq!(decoded_frames.len(), 0, "Bad CRC frame should be rejected");
    }

    #[test]
    fn test_host_cmd_conversion() {
        assert_eq!(u8::from(HostCmd::Hello), 0x01);
        assert_eq!(u8::from(HostCmd::Reboot), 0x04);
        assert_eq!(u8::from(HostCmd::RebootBootloader), 0x05);
        assert_eq!(u8::from(HostCmd::Ack), 0x7E);
        assert_eq!(u8::from(HostCmd::Nak), 0x7F);

        assert_eq!(HostCmd::try_from(0x01).unwrap(), HostCmd::Hello);
        assert_eq!(HostCmd::try_from(0x04).unwrap(), HostCmd::Reboot);
        assert_eq!(HostCmd::try_from(0x05).unwrap(), HostCmd::RebootBootloader);
        assert_eq!(HostCmd::try_from(0x7E).unwrap(), HostCmd::Ack);
        assert_eq!(HostCmd::try_from(0xFF).is_err(), true);
    }

    #[test]
    fn test_device_info_serialization() {
        let info = DeviceInfo {
            protocol_version: 0x0100,
            fw_version: 0x01020304,
            capsense_channels: 36,
            capability_bits: 0x0F0F0F0F,
            psoc_generation: 0,
            psoc_link_valid: false,
            psoc_snapshot_valid: false,
            diagnostics: None,
        };

        let payload = info.to_payload();
        let parsed = DeviceInfo::from_payload(&payload).unwrap();

        assert_eq!(parsed.protocol_version, info.protocol_version);
        assert_eq!(parsed.fw_version, info.fw_version);
        assert_eq!(parsed.capsense_channels, info.capsense_channels);
        assert_eq!(parsed.capability_bits, info.capability_bits);
    }

    #[test]
    fn test_multiple_frames_in_stream() {
        let frame1 = Frame::hello(1);
        let frame2 = Frame::ping(2);
        let frame3 = Frame::ack(3);

        let mut encoded = encode(&frame1);
        encoded.extend_from_slice(&encode(&frame2));
        encoded.extend_from_slice(&encode(&frame3));

        let mut decoder = Decoder::new();
        let decoded = decoder.feed_bytes(&encoded);

        assert_eq!(decoded.len(), 3);
        assert_eq!(decoded[0].seq, 1);
        assert_eq!(decoded[1].seq, 2);
        assert_eq!(decoded[2].seq, 3);
    }

    #[test]
    fn test_oversized_payload_rejected() {
        let huge_payload = vec![0xAB; PAYLOAD_MAX + 100];
        let frame = Frame::new(0x10, 0, 0, huge_payload);
        let encoded = encode(&frame);

        let mut decoder = Decoder::new();
        let decoded = decoder.feed_bytes(&encoded);

        // Oversized frame should be rejected
        assert_eq!(decoded.len(), 0);
    }
}
