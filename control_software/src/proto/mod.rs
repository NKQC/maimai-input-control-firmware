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

pub mod algo;
pub mod config;
pub mod led;
pub mod telemetry;

pub use config::{
    CfgValue, ConfigEntry, ConfigValueType, decode_entries, decode_entry, encode_entries,
    encode_entry,
};
pub use led::{
    LED_CH_UNMAPPED, LED_PREVIEW_ALL, LED_UNIT_COUNT, LedRegion, LedState, decode_led_get,
    encode_led_get, encode_led_preview, encode_led_set_region, validate_led_regions,
};
use std::convert::TryFrom;
pub use telemetry::{
    AutoTuneProgress,
    BATCH_PARAM_IDS,
    ChannelSample,
    FIELD_ALGO,
    FIELD_BASELINE,
    FIELD_DELAY_DEV,
    FIELD_DIFF,
    FIELD_LATENCY,
    FIELD_RAW,
    FIELD_STATS,
    FIELD_STATUS,
    FocusFrame,
    // IDAC 增益档 → 每 LSB 电流(pA)真值表 + 按电流升序的档号次序(档号非单调, 见 telemetry.rs)
    IDAC_GAIN_BY_CURRENT,
    IDAC_GAIN_PA,
    KNOWN_PARAM_IDS,
    // PARAM_GET_ALL 的"全通道单参数"批量变体(替代 36 条单发)
    PARAM_ALL_CHANNELS,
    // 通道启用开关(硬件级, 0=电极保持高阻)
    PARAM_ENABLED,
    PARAM_FINGER_TH,
    PARAM_HYSTERESIS,
    PARAM_IDAC_GAIN,
    PARAM_IDAC_MOD,
    PARAM_LOW_BSLN_RST,
    PARAM_NEG_NOISE_TH,
    PARAM_NOISE_TH,
    PARAM_ON_DEBOUNCE,
    PARAM_RESOLUTION,
    PARAM_SNS_CLK_DIV,
    PARAM_SNS_CLK_SOURCE,
    ParamFence,
    ParamScope,
    PsocRescueProgress,
    SWEEP_FLAG_CAL_FAIL,
    SWEEP_FLAG_MISMATCH,
    SWEEP_FLAG_RAILED,
    SWEEP_FLAG_RETRANSMIT,
    SWEEP_FLAG_STALLED,
    SWEEP_INDEX_NONE,
    SWEEP_RESTORE_FLAG_BSLN,
    SWEEP_RESTORE_FLAG_CAL,
    SWEEP_RESTORE_FLAG_PARAM,
    SweepFrame,
    SweepState,
    TelemFrame,
    decode_auto_tune_progress,
    decode_cp_get,
    decode_focus_data,
    decode_focus_start,
    decode_param_get,
    decode_param_get_all,
    decode_param_get_all_channels,
    decode_psoc_rescue_progress,
    decode_sweep_data,
    decode_sweep_start,
    decode_telem_data,
    encode_ch_mask,
    encode_cp_get,
    encode_cp_measure,
    encode_focus_start,
    encode_focus_stop,
    encode_param_get,
    encode_param_get_all,
    encode_param_get_all_channels,
    encode_param_set,
    encode_sweep_cancel,
    encode_sweep_keepalive,
    encode_sweep_resend,
    encode_sweep_start,
    encode_telem_start,
    // 全局项(GPARAM_*)围栏, 与单通道 param_* 同构、共用 ParamFence
    global_clamp,
    global_fence,
    global_value_legal,
    param_clamp,
    param_fence,
    param_value_legal,
    sweep_phase_text,
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
const FLAG_RESPONSE: u8 = 0x01; // bit0=1: response frame
const FLAG_STREAM: u8 = 0x02; // bit1=1: stream data frame
const FLAG_NAK_ERR: u8 = 0x04; // bit2=1: NAK (error)
/// bit3=1: payload 末 4 字节是设备端 `time_us_32()` 时间戳(u32 LE)。
///
/// ★加法式协议扩展, 与固件 `HOST_CMD_FLAG_TS` 同源★ 设备侧在**唯一**出向组帧口
/// (`HostCmdCodec::encode_frame`)统一追加, 于是所有设备→主机的帧都带戳。
/// ★本侧的关键约定★: 时间戳在 `Decoder` 里就被**剥离**进 `Frame::device_t_us`,
/// `Frame::payload` 恢复为不含时间戳的原样 ⇒ 全部 `decode_*` 函数一行都不用改,
/// 也不存在"某个解析函数忘了减 4"的可能。新增命令同样自动获得 `device_t_us`。
const FLAG_TS: u8 = 0x08;
/// 尾部时间戳字节数(u32 LE)。
const TS_SIZE: usize = 4;

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
    RebootPsoc = 0x06,
    DebugCrashBootsel = 0x07,
    /// 仅在已武装且设备空闲时触发一次安全 watchdog_reboot，进入既有崩溃 BOOTSEL 路径。
    DebugTriggerCrash = 0x0A,
    /// PSoC 救砖: 空 payload → 立即 ACK("已受理"), 设备经 SWD 强制重刷 + 重新下发算法/CSD。
    PsocRescue = 0x08,
    /// 设备主动推送(flags=STREAM): 救砖阶段进度/终态, 见 `decode_psoc_rescue_progress`。
    PsocRescueProgressPush = 0x09,
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
    GlobalGet = 0x29,
    GlobalSet = 0x2A,
    GlobalGetAll = 0x2B,
    AutoTune = 0x2C,
    GlobalCommit = 0x2D,
    /// 设备主动推送(flags=STREAM): 频率自适应阶段进度/终态, 见 `decode_auto_tune_progress`。
    AutoTuneProgressPush = 0x2E,
    /// 设备主动推送(flags=STREAM): 固件自持恢复事件(复位 PSoC / 回退算法 / 清空 CSD store /
    /// 重新下发 / PSoC 启动强制改写配置)。这些动作会让设备实际状态偏离 UI 以为的状态, 必须落日志。
    /// payload = [code(u8), detail(u32 LE), seq(u16 LE), total(u16 LE)]
    SelfHealEventPush = 0x2F,

    // Telemetry stream domain 0x30-0x3F
    TelemStart = 0x30,
    TelemStop = 0x31,
    TelemData = 0x32,
    /// 单通道独占流启动: `[ch, fields, rate u16 LE, lease u16 LE]` → `[session u16, accepted_rate u16]`。
    /// 设备侧受理后会**挂起广谱 TELEM 流**(focus 期间不再有 TELEM_DATA), FOCUS_STOP 时自动恢复。
    FocusStart = 0x33,
    /// 单通道独占流停止: `[session u16 LE]`。session 不是当前会话 → NAK(拒绝误停新会话)。
    FocusStop = 0x34,
    /// 设备主动推送(flags=STREAM): 单通道采样, 见 `decode_focus_data`。
    FocusData = 0x35,
    /// 增益/分频扫描会话启动: `[ch, settle_samples, sample_count]` → `[session u16, total u16]`。
    /// 448 格由**设备**逐格切参数+校准+采样, 上位机只负责收结果与补发, 不再逐格下发 PARAM_SET。
    SweepStart = 0x36,
    /// 扫描会话控制: `[op(0=取消/1=补发), session u16, first u16, count u8]`。
    SweepCtrl = 0x37,
    /// 设备主动推送(flags=STREAM): 定长 21B 的格结果 / 恢复中 / 终态, 见 `decode_sweep_data`。
    SweepData = 0x38,

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

    // JIT algorithm domain 0x60-0x6F
    AlgoGetInfo = 0x60,
    AlgoUpload = 0x61,
    AlgoApply = 0x62,
    AlgoResetDefault = 0x63,
    AlgoSetRom = 0x64,
    AlgoGetRom = 0x65,
    AlgoGetSrc = 0x66,
    AlgoSetSrc = 0x67,
    AlgoGetCode = 0x68,
    // 0x69 已退役(原 AlgoGetTrace 逐项轮询)。算法运行值改随遥测帧的 FIELD_ALGO 上报。
    // ★不要复用该码★: 旧固件仍会把它当追踪请求处理。
    AlgoSetCfg = 0x6A,
    AlgoGetCfg = 0x6B,

    // Keyboard (physical GPIO1-12 + touch->key) domain 0x70-0x7D
    KbdGetState = 0x70,
    KbdGetMap = 0x71,
    KbdSetMap = 0x72,
    KbdGetTouchmap = 0x73,
    KbdSetTouchmap = 0x74,
    /// 长按参数回读: 空 payload → 12 物理键 + 34 分区各自的 (delay_ms, max_hold_ms)。
    KbdGetHold = 0x75,
    /// 长按参数下发: n×[kind, idx, delay_ms(u16 LE), max_hold_ms(u16 LE)]。
    KbdSetHold = 0x76,
    /// 逻辑分析仪边沿记录拉取: [max(u8) 可选] → 见 `decode_kbd_get_edges`。
    KbdGetEdges = 0x77,
    /// 每键触发极性 + 独立防抖回读: 空 → [count, 12×(pol, debounce_us u16 LE)]。
    KbdGetKeycfg = 0x7C,
    /// 每键触发极性 + 独立防抖下发: n×[idx, pol, debounce_us(u16 LE)]。
    KbdSetKeycfg = 0x7D,
    /// 触控组合映射回读: 空 payload → [combo_count, key_count, count×16B]。
    KbdGetCombo = 0x7A,
    /// 触控组合映射下发(整表替换): [count] + count×16B。
    KbdSetCombo = 0x7B,
    /// mai2 串口(游戏触控上报)运行态回读: [send_en, status, baud(u32 LE)]。
    Mai2GetState = 0x78,
    /// mai2 串口发送使能: [en]。
    Mai2SetSendEn = 0x79,

    // Response codes 0x7E-0x7F
    Ack = 0x7E,
    Nak = 0x7F,

    // Mai2Bus transport domain
    BusXfer = 0x80,
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
            0x06 => Ok(RebootPsoc),
            0x07 => Ok(DebugCrashBootsel),
            0x0A => Ok(DebugTriggerCrash),
            0x08 => Ok(PsocRescue),
            0x09 => Ok(PsocRescueProgressPush),
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
            0x29 => Ok(GlobalGet),
            0x2A => Ok(GlobalSet),
            0x2B => Ok(GlobalGetAll),
            0x2C => Ok(AutoTune),
            0x2D => Ok(GlobalCommit),
            0x2E => Ok(AutoTuneProgressPush),
            0x2F => Ok(SelfHealEventPush),
            0x30 => Ok(TelemStart),
            0x31 => Ok(TelemStop),
            0x32 => Ok(TelemData),
            0x33 => Ok(FocusStart),
            0x34 => Ok(FocusStop),
            0x35 => Ok(FocusData),
            0x36 => Ok(SweepStart),
            0x37 => Ok(SweepCtrl),
            0x38 => Ok(SweepData),
            0x40 => Ok(BindStart),
            0x41 => Ok(BindAbort),
            0x42 => Ok(BindConfirm),
            0x43 => Ok(BindGetMap),
            0x44 => Ok(BindSetMap),
            0x45 => Ok(BindEvent),
            0x50 => Ok(LedGet),
            0x51 => Ok(LedSetRegion),
            0x52 => Ok(LedPreview),
            0x60 => Ok(AlgoGetInfo),
            0x61 => Ok(AlgoUpload),
            0x62 => Ok(AlgoApply),
            0x63 => Ok(AlgoResetDefault),
            0x64 => Ok(AlgoSetRom),
            0x65 => Ok(AlgoGetRom),
            0x66 => Ok(AlgoGetSrc),
            0x67 => Ok(AlgoSetSrc),
            0x68 => Ok(AlgoGetCode),

            0x6A => Ok(AlgoSetCfg),
            0x6B => Ok(AlgoGetCfg),
            0x70 => Ok(KbdGetState),
            0x71 => Ok(KbdGetMap),
            0x72 => Ok(KbdSetMap),
            0x73 => Ok(KbdGetTouchmap),
            0x74 => Ok(KbdSetTouchmap),
            0x75 => Ok(KbdGetHold),
            0x76 => Ok(KbdSetHold),
            0x77 => Ok(KbdGetEdges),
            0x7C => Ok(KbdGetKeycfg),
            0x7D => Ok(KbdSetKeycfg),
            0x7A => Ok(KbdGetCombo),
            0x7B => Ok(KbdSetCombo),
            0x78 => Ok(Mai2GetState),
            0x79 => Ok(Mai2SetSendEn),
            0x7E => Ok(Ack),
            0x7F => Ok(Nak),
            0x80 => Ok(BusXfer),
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
    /// ★不含尾部时间戳★: 带 `FLAG_TS` 的帧在 `Decoder` 里已把末 4 字节剥进 `device_t_us`,
    /// 这里始终是命令自身定义的 payload ⇒ 所有 `decode_*` 按原偏移解析即正确。
    pub payload: Vec<u8>,
    /// 设备端 `time_us_32()`(u32 微秒, 约 71.6 分钟回绕)= 该帧在设备侧组帧那一刻的时间。
    ///
    /// `None` = 该帧不带 `FLAG_TS`(主机自己构造的请求帧, 或旧固件)。
    /// ★用途★ 把"主机轮询取回"的数据(如 ALGO_GET_TRACE)放回设备时间轴, 与 TELEM_DATA
    /// (payload 内自带 ts_us)、KBD_GET_EDGES 的 t_us 同源, 于是横向对位不再是近似值。
    pub device_t_us: Option<u32>,
}

impl Frame {
    /// Create a new frame. 主机→设备方向不带时间戳(主机时钟对设备无意义)。
    pub fn new(cmd: u8, flags: u8, seq: u8, payload: Vec<u8>) -> Self {
        Frame {
            cmd,
            flags,
            seq,
            payload,
            device_t_us: None,
        }
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
        Frame::new(
            HostCmd::Nak as u8,
            FLAG_RESPONSE | FLAG_NAK_ERR,
            seq,
            payload,
        )
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
    let crc_data = &buf[2..]; // start from cmd
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
    header: [u8; 5], // cmd, flags, seq, len_lo, len_hi
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
                            // ★时间戳剥离唯一点★: 带 FLAG_TS 就把末 4 字节取走存进
                            // device_t_us, payload 还原成命令自身的原样布局 ⇒ 下游
                            // 全部 decode_* 无需感知本扩展(见 FLAG_TS 注释)。
                            let mut payload = self.payload.clone();
                            let mut device_t_us = None;
                            if (self.header[1] & FLAG_TS) != 0 && payload.len() >= TS_SIZE {
                                let tail = payload.split_off(payload.len() - TS_SIZE);
                                device_t_us =
                                    Some(u32::from_le_bytes([tail[0], tail[1], tail[2], tail[3]]));
                            }
                            let frame = Frame {
                                cmd: self.header[0],
                                flags: self.header[1],
                                seq: self.header[2],
                                payload,
                                device_t_us,
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
pub struct SpiDebugCounters {
    pub status: u8,
    pub block_addr: u32,
    pub rx_frames: u32,
    pub scan_count: u32,
    pub ms_tick: u32,
    pub stage: u32,
    pub apply_cmd: u32,
    pub apply_last_ms: u32,
    pub apply_dirty: u32,
    pub setparam_cmd: u32,
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
    /// CSD 运行态标志(报告尾部追加, 旧固件为 0): bit0 = 上次"恢复默认"因 PSoC 采样异常
    /// (raw 满量程 railed / 数据停滞)拒绝固化基线 → 应改用"PSoC 救砖"。
    pub csd_flags: u8,
    /// 固件累计登记的自持恢复事件数(与实际收到的 SELF_HEAL_EVENT 条数核对可发现漏帧)。
    pub self_heal_total: u16,
    /// 因队列满被丢弃的事件数(>0 说明有事件永远看不到了)。
    pub self_heal_dropped: u16,
    /// 是否还有事件待发(主机不在线时会一直排队)。
    pub self_heal_pending: bool,
    /// RP2040 store 持有的真实 CSD 模式(报告尾部追加；旧固件缺失时按 AUTO=0 处理)。
    pub csd_mode: u8,
    /// PSoC 运行态 SWD 读取的 spi_dbg 计数；旧报告无此固定尾部时为 None。
    pub spi_debug: Option<SpiDebugCounters>,
}

/// `PsocBringupDiagnostics::csd_flags` 位: 恢复默认未获得可信基线。
pub const CSD_FLAG_BASELINE_UNTRUSTED: u8 = 0x01;

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
        let required = BRINGUP_FLAG_ACQUIRED
            | BRINGUP_FLAG_SILICON_ID
            | BRINGUP_FLAG_ERASE
            | BRINGUP_FLAG_PROGRAM
            | BRINGUP_FLAG_VERIFY;
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
    let bytes = payload
        .get(offset..offset + 2)
        .ok_or_else(|| format!("DEVICE_INFO missing u16 at {}", offset))?;
    Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
}

fn read_u32_le(payload: &[u8], offset: usize) -> Result<u32, String> {
    let bytes = payload
        .get(offset..offset + 4)
        .ok_or_else(|| format!("DEVICE_INFO missing u32 at {}", offset))?;
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

impl DeviceInfo {
    pub fn from_payload(payload: &[u8]) -> Result<Self, String> {
        if payload.len() < 8 {
            return Err(format!(
                "DEVICE_INFO payload too short: {} bytes",
                payload.len()
            ));
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
                return Err(format!(
                    "unsupported PSoC report version {}",
                    report_version
                ));
            }
            if report_length < 69 || payload.len() < 15 + report_length as usize {
                return Err(format!(
                    "truncated PSoC report: declared {} bytes, payload {} bytes",
                    report_length,
                    payload.len()
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
                // 报告尾部追加字段: 声明长度不足(旧固件)时按 0 处理, 保持向后兼容。
                csd_flags: if report_length >= 110 {
                    payload[124]
                } else {
                    0
                },
                // 自持恢复事件计数(再往后 5 字节): 累计 / 被丢弃 / 是否还有待发。
                // 供上位机与"自己实际收到的条数"核对, 漏帧要能被发现而不是当作没发生。
                self_heal_total: if report_length >= 115 {
                    u16::from_le_bytes([payload[125], payload[126]])
                } else {
                    0
                },
                self_heal_dropped: if report_length >= 115 {
                    u16::from_le_bytes([payload[127], payload[128]])
                } else {
                    0
                },
                self_heal_pending: report_length >= 115 && payload[129] != 0,
                // CSD 模式紧跟自持恢复计数；旧固件报告长度不足时保持 AUTO 默认值。
                csd_mode: if report_length >= 116 {
                    payload[130]
                } else {
                    0
                },
                // 固定 37B 的运行态 SWD 诊断尾部；旧报告无此尾部时不解析。
                spi_debug: if report_length >= 153 {
                    Some(SpiDebugCounters {
                        status: payload[131],
                        block_addr: read_u32_le(payload, 132)?,
                        rx_frames: read_u32_le(payload, 136)?,
                        scan_count: read_u32_le(payload, 140)?,
                        ms_tick: read_u32_le(payload, 144)?,
                        stage: read_u32_le(payload, 148)?,
                        apply_cmd: read_u32_le(payload, 152)?,
                        apply_last_ms: read_u32_le(payload, 156)?,
                        apply_dirty: read_u32_le(payload, 160)?,
                        setparam_cmd: read_u32_le(payload, 164)?,
                    })
                } else {
                    None
                },
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
            let report_length_offset = payload.len();
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
            if report.erase_failure.is_some() || report.spi_debug.is_some() {
                let empty_erase = EraseFailureDiagnostics {
                    clock_select: 0,
                    clock_imo_select: 0,
                    clock_trim1: 0,
                    clock_trim2: 0,
                    clock_trim3: 0,
                    flash_byte_sum: 0,
                    flash_word_or: 0,
                    first_nonzero_addr: 0,
                    first_nonzero_value: 0,
                    words_read: 0,
                };
                let erase = report.erase_failure.as_ref().unwrap_or(&empty_erase);
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
                payload.push(report.csd_flags); // 尾部 CSD 标志(仅在完整报告后存在)
                payload.extend_from_slice(&report.self_heal_total.to_le_bytes());
                payload.extend_from_slice(&report.self_heal_dropped.to_le_bytes());
                payload.push(u8::from(report.self_heal_pending));
                payload.push(report.csd_mode); // 诊断尾部真实 CSD 模式
                if let Some(spi_debug) = &report.spi_debug {
                    payload.push(spi_debug.status);
                    payload.extend_from_slice(&spi_debug.block_addr.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.rx_frames.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.scan_count.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.ms_tick.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.stage.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.apply_cmd.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.apply_last_ms.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.apply_dirty.to_le_bytes());
                    payload.extend_from_slice(&spi_debug.setparam_cmd.to_le_bytes());
                }
            }
            payload[report_length_offset] = (payload.len() - 15) as u8;
        }
        payload
    }
}

// ============================================================================
// 键盘长按参数 (KBD_GET_HOLD 0x75 / KBD_SET_HOLD 0x76)
// ============================================================================

/// 物理键数量(GPIO1-12)。
pub const KBD_HOLD_PHYS_COUNT: usize = 12;
/// 触控分区数量(A1..E8)。
pub const KBD_HOLD_ZONE_COUNT: usize = 34;
/// KBD_SET_HOLD 的 kind 字段: 物理键。
pub const KBD_HOLD_KIND_PHYS: u8 = 0;
/// KBD_SET_HOLD 的 kind 字段: 触控分区。
pub const KBD_HOLD_KIND_ZONE: u8 = 1;

/// 单个按键/分区的长按参数。
///
/// - `delay_ms`: 按住达到该时长才真正输出键(0 = 立即输出)。
/// - `max_hold_ms`: 输出后最长保持该时长即自动抬起(0 = 不自动抬起)。
///
/// 12 物理键与 34 分区各自独立。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct HoldParam {
    pub delay_ms: u16,
    pub max_hold_ms: u16,
}

/// KBD_SET_HOLD 的单条下发项。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KbdHoldItem {
    /// `KBD_HOLD_KIND_PHYS` 或 `KBD_HOLD_KIND_ZONE`。
    pub kind: u8,
    pub idx: u8,
    pub hold: HoldParam,
}

/// KBD_GET_HOLD 响应解出的完整长按参数表。
#[derive(Debug, Clone, Default)]
pub struct KbdHoldTable {
    pub phys: Vec<HoldParam>,
    pub zone: Vec<HoldParam>,
}

/// 触控组合映射的一条: "zone_mask 内的分区全部同时按下" → "keycodes 全部同时输出"。
/// 与固件 `KeyboardService::ComboMap` 一一对应, 单条协议编码 16 字节。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct KbdComboItem {
    /// 参与判定的分区集合(34 位); 0 = 空条目(固件会丢弃)。
    pub zone_mask: u64,
    /// 同时按下的 HID 键码, 0 = 空位。
    pub keycodes: [u8; KBD_COMBO_KEY_COUNT],
    /// 修饰位 bit0=LCtrl bit1=LShift bit2=LAlt bit3=LGui。
    pub modifiers: u8,
    /// 全部分区按住需持续该时长才输出(ms, 0=立即)。
    pub delay_ms: u16,
    /// 输出后最长保持该时长即自动抬起(ms, 0=不自动抬起)。
    pub max_hold_ms: u16,
}

/// 固件 `COMBO_KEY_COUNT`: 单条最多同时输出的键数。
pub const KBD_COMBO_KEY_COUNT: usize = 4;
/// 固件 `COMBO_COUNT`: 组合映射条数上限。
pub const KBD_COMBO_COUNT: usize = 16;
/// 单条协议字节数(固件 `COMBO_ENTRY_BYTES`) = zone_mask(8) + keycodes(4) + mod(1) + delay(2) + maxhold(2)。
/// 两端都曾写成 16, 与实际逐字段布局差 1 字节, 于是固件的整表长度校验永远失败并 NAK ——
/// 组合映射写不进设备。此处按字段和推导, 不再写字面量。
const KBD_COMBO_ENTRY_BYTES: usize = 8 + KBD_COMBO_KEY_COUNT + 1 + 2 + 2;

/// 编码 KBD_SET_COMBO 请求载荷: [count(u8)] + count×16B。**整表替换**语义。
pub fn encode_kbd_set_combo(items: &[KbdComboItem]) -> Vec<u8> {
    let n = items.len().min(KBD_COMBO_COUNT);
    let mut payload = Vec::with_capacity(1 + n * KBD_COMBO_ENTRY_BYTES);
    payload.push(n as u8);
    for item in items.iter().take(n) {
        payload.extend_from_slice(&item.zone_mask.to_le_bytes());
        payload.extend_from_slice(&item.keycodes);
        payload.push(item.modifiers);
        payload.extend_from_slice(&item.delay_ms.to_le_bytes());
        payload.extend_from_slice(&item.max_hold_ms.to_le_bytes());
    }
    payload
}

/// 解码 KBD_GET_COMBO 响应: [combo_count(u8), key_count(u8)] + combo_count×16B。
/// 只返回非空条目(zone_mask != 0) —— 空槽位对上位机没有意义, 表长由 UI 自己按上限约束。
/// key_count 与本地常量不一致时报错而不是猜: 那是固件与上位机版本不匹配, 静默截断会写出错误的键。
pub fn decode_kbd_get_combo(payload: &[u8]) -> Result<Vec<KbdComboItem>, String> {
    if payload.len() < 2 {
        return Err(format!(
            "KBD_GET_COMBO 响应过短: {} 字节 (需 >= 2)",
            payload.len()
        ));
    }
    let count = payload[0] as usize;
    let key_count = payload[1] as usize;
    if key_count != KBD_COMBO_KEY_COUNT {
        return Err(format!(
            "KBD_GET_COMBO key_count={} 与上位机常量 {} 不一致(固件/上位机版本不匹配)",
            key_count, KBD_COMBO_KEY_COUNT
        ));
    }
    let need = 2 + count * KBD_COMBO_ENTRY_BYTES;
    if payload.len() < need {
        return Err(format!(
            "KBD_GET_COMBO 响应长度 {} 不足 count={} 所需的 {}",
            payload.len(),
            count,
            need
        ));
    }
    let mut out = Vec::new();
    for i in 0..count {
        let b = 2 + i * KBD_COMBO_ENTRY_BYTES;
        let zone_mask = u64::from_le_bytes([
            payload[b],
            payload[b + 1],
            payload[b + 2],
            payload[b + 3],
            payload[b + 4],
            payload[b + 5],
            payload[b + 6],
            payload[b + 7],
        ]);
        if zone_mask == 0 {
            continue;
        }
        let mut keycodes = [0u8; KBD_COMBO_KEY_COUNT];
        keycodes.copy_from_slice(&payload[b + 8..b + 8 + KBD_COMBO_KEY_COUNT]);
        let m = b + 8 + KBD_COMBO_KEY_COUNT;
        out.push(KbdComboItem {
            zone_mask,
            keycodes,
            modifiers: payload[m],
            delay_ms: u16::from_le_bytes([payload[m + 1], payload[m + 2]]),
            max_hold_ms: u16::from_le_bytes([payload[m + 3], payload[m + 4]]),
        });
    }
    Ok(out)
}

/// 编码 KBD_SET_HOLD 请求载荷。
/// payload = n×[kind(u8), idx(u8), delay_ms(u16 LE), max_hold_ms(u16 LE)]
pub fn encode_kbd_set_hold(items: &[KbdHoldItem]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(items.len() * 6);
    for item in items {
        payload.push(item.kind);
        payload.push(item.idx);
        payload.extend_from_slice(&item.hold.delay_ms.to_le_bytes());
        payload.extend_from_slice(&item.hold.max_hold_ms.to_le_bytes());
    }
    payload
}

/// 解码 KBD_GET_HOLD 响应载荷。
/// payload = phys_count(u8) + zone_count(u8) + phys_count×4B + zone_count×4B
/// 长度不足一律返回错误(不 panic, 不用默认值糊过去)。
pub fn decode_kbd_get_hold(payload: &[u8]) -> Result<KbdHoldTable, String> {
    if payload.len() < 2 {
        return Err(format!(
            "KBD_GET_HOLD 响应过短: {} 字节 (需 >= 2)",
            payload.len()
        ));
    }
    let phys_count = payload[0] as usize;
    let zone_count = payload[1] as usize;
    let need = 2 + (phys_count + zone_count) * 4;
    if payload.len() < need {
        return Err(format!(
            "KBD_GET_HOLD 响应截断: 声明 {} 物理键 + {} 分区需 {} 字节, 实收 {} 字节",
            phys_count,
            zone_count,
            need,
            payload.len()
        ));
    }
    let read_at = |pos: usize| HoldParam {
        delay_ms: u16::from_le_bytes([payload[pos], payload[pos + 1]]),
        max_hold_ms: u16::from_le_bytes([payload[pos + 2], payload[pos + 3]]),
    };
    let mut table = KbdHoldTable {
        phys: Vec::with_capacity(phys_count),
        zone: Vec::with_capacity(zone_count),
    };
    for i in 0..phys_count {
        table.phys.push(read_at(2 + i * 4));
    }
    let zone_base = 2 + phys_count * 4;
    for i in 0..zone_count {
        table.zone.push(read_at(zone_base + i * 4));
    }
    Ok(table)
}

// ============================================================================
// 物理键每键配置 (KBD_GET_KEYCFG 0x7C / KBD_SET_KEYCFG 0x7D)
// 与逻辑分析仪边沿记录 (KBD_GET_EDGES 0x77)
// ============================================================================

/// 每键防抖窗上限(us)。与固件 `KeyboardService::DEBOUNCE_US_MAX` 同源: UI 围栏必须取同一个数,
/// 否则用户能设出一个必然被 NAK 的值。
pub const KBD_DEBOUNCE_US_MAX: u16 = 10000;
/// 每键防抖默认值(= 改造前的全局 DEBOUNCE_US)。
pub const KBD_DEBOUNCE_US_DEFAULT: u16 = 3000;

/// 触发极性: 低电平触发(按下=接地)。
pub const KBD_POL_LOW: u8 = 0;
/// 触发极性: 高电平触发(按下=被主动拉高)。
pub const KBD_POL_HIGH: u8 = 1;
/// 触发极性: AUTO(默认) —— 固件只看**启动时**的电平并把它当作该键的"抬起"电平:
/// 启动为高 → 低电平触发; 启动为低 → 高电平触发。启动后不再重采样。
pub const KBD_POL_AUTO: u8 = 2;

/// 单个物理键的触发极性与防抖窗。
///
/// - `pol`: `KBD_POL_LOW` / `KBD_POL_HIGH` / `KBD_POL_AUTO`(默认)。三态, 不能折叠成 bool。
/// - `debounce_us`: 0..=`KBD_DEBOUNCE_US_MAX`, 0 = 不去抖。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KbdKeyCfg {
    pub pol: u8,
    pub debounce_us: u16,
}

impl Default for KbdKeyCfg {
    fn default() -> Self {
        KbdKeyCfg {
            pol: KBD_POL_AUTO,
            debounce_us: KBD_DEBOUNCE_US_DEFAULT,
        }
    }
}

/// 一条边沿记录: 12 位掩码发生变化的时刻 + 去抖前/仅去抖后/实际输出三态。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct KbdEdgeRec {
    /// 设备 `time_us_32()` 原始时间戳(微秒, 会 32 位回绕)。
    pub t_us: u32,
    /// 去抖前的 12 位按下态。
    pub raw: u16,
    /// 仅经去抖后的 12 位按下态。
    pub deb: u16,
    /// 经去抖 + 长按状态机后实际输出 HID 的 12 位。
    pub out: u16,
}

/// KBD_GET_EDGES 一次拉取的结果。
#[derive(Debug, Clone, Default)]
pub struct KbdEdgeBatch {
    /// 固件环形缓冲容量(条)。
    pub cap: u16,
    /// 固件累计因环满丢弃的条数(只增)。主机取差值即可诚实显示"有事件丢失"。
    pub overflow: u32,
    /// 本次取完后固件侧还剩的条数(>0 说明该提高拉取频率)。
    pub remaining: u16,
    pub recs: Vec<KbdEdgeRec>,
}

/// 触控→键盘链路诊断: `KBD_GET_STATE` 尾部**第二段**追加字段(固件 diag_ver=1)。
///
/// ★每一项都是设备实测读数, 没有任何本地推算★ 门控链条有 6 环(总开关 → 仅协议发送 →
/// 扫描抑制 → 掩码可信 → 分区绑定 → 组合判定), 任一环断掉现象都是同一个"完全不输出";
/// 拿不到这些中间态就只能猜。响应里没有该段(旧固件/响应截断)时整体为 `None`, 界面必须显示
/// "诊断字段不可用", 不许在数据不足时给结论。
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct KbdLinkDiag {
    /// 固件诊断块版本(当前 1)。0 视为无效, 解码直接判失败。
    pub ver: u8,
    /// `comm.keyboard_map_en` 总开关。
    pub map_en: bool,
    /// `comm.keyboard_map_serial_only`: 仅 mai2serial 真在发触控数据时才生效。
    pub serial_only: bool,
    /// mai2serial 此刻是否真在发触控数据。
    pub mai2_sending: bool,
    /// 触控扫描会话正在改写通道参数, 掩码无意义 ⇒ 固件按全松开处理。
    pub output_suppressed: bool,
    /// PSoC 触摸掩码是否仍在可信窗口内。
    pub touch_hold_ok: bool,
    /// 固件本轮**实际**算出的 map_active(不是上位机按上面几位重算的)。
    pub map_active: bool,
    /// 组合映射表是否有有效条目(为假时固件走 per-zone 回落路径)。
    pub combo_active: bool,
    /// 固件 HID 子系统是否已初始化。为假时任何按键都发不出去。
    pub hid_initialized: bool,
    /// 本轮组合判定实际输出的 HID 键码集合。
    pub combo_out_keys: Vec<u8>,
    /// PSoC 实时触摸掩码(36 位物理通道)。
    pub touch_mask: u64,
    /// `map_to_areas` 结果(34 位逻辑分区)。
    pub area_raw: u64,
    /// 已绑定分区数(绑定表里落在合法通道范围内的项数)。
    pub bound_zones: u8,
    /// HID 键盘报文累计实发数。
    pub hid_sent: u32,
    /// HID 键盘报文累计发送失败数(端点忙 ⇒ 报文被静默丢弃)。
    pub hid_failed: u32,
}

/// 诊断块的固定部分字节数(变长的键码集合在其后)。
const KBD_DIAG_FIXED_BYTES: usize = 34;

impl KbdLinkDiag {
    /// 一行人话结论: **只由实测位推出**, 按门控链条自上而下取第一个成立的断点。
    /// 数据不足的情形由调用方在 `None` 分支处理, 这里不会出现"猜"的分支。
    pub fn summary(&self) -> String {
        let readings = format!(
            "触摸=0x{:X} 分区=0x{:X} 已绑定{}区 输出{}键 发送{}/失败{}",
            self.touch_mask,
            self.area_raw,
            self.bound_zones,
            self.combo_out_keys.len(),
            self.hid_sent,
            self.hid_failed
        );
        let verdict = if !self.hid_initialized {
            "设备 HID 未初始化 ⇒ 任何按键都发不出去".to_string()
        } else if !self.map_en {
            "总开关未启用 ⇒ 不输出".to_string()
        } else if self.serial_only && !self.mai2_sending {
            "已限定“仅协议发送时生效”，但 mai2serial 当前无消费者 ⇒ 不输出".to_string()
        } else if self.output_suppressed {
            "触控扫描会话进行中，输出被抑制 ⇒ 不输出".to_string()
        } else if !self.touch_hold_ok {
            "PSoC 触摸掩码不可信(近期无合法触控帧) ⇒ 不输出".to_string()
        } else if !self.map_active {
            // 上面每一环都成立却仍不生效: 只能如实报"设备实测与门控不一致", 不替设备编解释。
            "设备实测 map_active=0，但上列各环均正常 ⇒ 原因未知".to_string()
        } else if self.bound_zones == 0 {
            "分区绑定表为空(已绑定 0 个分区) ⇒ 触摸无法映射成分区".to_string()
        } else if self.touch_mask != 0 && self.area_raw == 0 {
            "触摸已检测到，但分区绑定表没把这些通道映射到分区 ⇒ 不输出".to_string()
        } else if !self.combo_active {
            "组合映射表为空，正走 per-zone 回落路径(按分区键码表判定)".to_string()
        } else if self.area_raw != 0 && self.combo_out_keys.is_empty() {
            "分区已触发，但组合判定未输出(检查触发延时是否已按满、分区集合是否全部命中)"
                .to_string()
        } else if !self.combo_out_keys.is_empty() && self.hid_failed != 0 {
            format!(
                "已输出按键但 HID 发送失败 {} 次(端点忙，报文被丢弃)",
                self.hid_failed
            )
        } else if self.touch_mask == 0 {
            "链路正常 · 当前无触摸".to_string()
        } else {
            "链路正常".to_string()
        };
        format!("{} · {}", verdict, readings)
    }
}

/// 解码 `KBD_GET_STATE` 尾部的链路诊断段。段缺失/截断/版本为 0 一律返回 `None` ——
/// 不拿零值冒充读数, 那会把"读不到"伪装成"各环都为假"。
pub fn decode_kbd_link_diag(payload: &[u8]) -> Option<KbdLinkDiag> {
    if payload.len() < KBD_DIAG_FIXED_BYTES {
        return None;
    }
    let ver = payload[6];
    if ver == 0 {
        return None;
    }
    let flags = payload[7];
    let bit = |mask: u8| (flags & mask) != 0;
    let u64_at = |at: usize| {
        let mut buf = [0u8; 8];
        buf.copy_from_slice(&payload[at..at + 8]);
        u64::from_le_bytes(buf)
    };
    let u32_at = |at: usize| {
        let mut buf = [0u8; 4];
        buf.copy_from_slice(&payload[at..at + 4]);
        u32::from_le_bytes(buf)
    };
    let out_count = payload[8] as usize;
    Some(KbdLinkDiag {
        ver,
        map_en: bit(0x01),
        serial_only: bit(0x02),
        mai2_sending: bit(0x04),
        output_suppressed: bit(0x08),
        touch_hold_ok: bit(0x10),
        map_active: bit(0x20),
        combo_active: bit(0x40),
        hid_initialized: bit(0x80),
        // 键码集合被响应截断时只取拿得到的那些: 计数字段单独保留在 combo_out_keys.len() 之外
        // 没有意义, 上位机以实际拿到的键为准。
        combo_out_keys: payload[KBD_DIAG_FIXED_BYTES..]
            .iter()
            .take(out_count)
            .copied()
            .collect(),
        touch_mask: u64_at(10),
        area_raw: u64_at(18),
        bound_zones: payload[9],
        hid_sent: u32_at(26),
        hid_failed: u32_at(30),
    })
}

/// 编码 KBD_SET_KEYCFG 请求载荷: n×[idx(u8), pol(u8), debounce_us(u16 LE)]。
/// 越界值不在这里夹取 —— 固件会 NAK, 静默夹取只会让界面与设备不一致。
pub fn encode_kbd_set_keycfg(items: &[(u8, KbdKeyCfg)]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(items.len() * 4);
    for (idx, cfg) in items {
        payload.push(*idx);
        payload.push(cfg.pol);
        payload.extend_from_slice(&cfg.debounce_us.to_le_bytes());
    }
    payload
}

/// 解码 KBD_GET_KEYCFG 响应:
/// [count(u8)] + count×(pol(u8), debounce_us(u16 LE)) + 可选 resolved_pol_high_mask(u16 LE)。
///
/// pol 是**配置态**(含 AUTO=2), 不折叠成 bool。尾部的 resolved mask 是固件解析后的生效极性
/// (bit i = 1 → 该键按高电平触发判定), 只有支持 AUTO 的固件才追加; 旧固件不带该字段 → `None`。
pub fn decode_kbd_get_keycfg(payload: &[u8]) -> Result<(Vec<KbdKeyCfg>, Option<u16>), String> {
    if payload.is_empty() {
        return Err("KBD_GET_KEYCFG 响应为空".to_string());
    }
    let count = payload[0] as usize;
    let need = 1 + count * 3;
    if payload.len() < need {
        return Err(format!(
            "KBD_GET_KEYCFG 响应截断: 声明 {} 键需 {} 字节, 实收 {}",
            count,
            need,
            payload.len()
        ));
    }
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let b = 1 + i * 3;
        out.push(KbdKeyCfg {
            pol: payload[b],
            debounce_us: u16::from_le_bytes([payload[b + 1], payload[b + 2]]),
        });
    }
    let resolved = if payload.len() >= need + 2 {
        Some(u16::from_le_bytes([payload[need], payload[need + 1]]))
    } else {
        None
    };
    Ok((out, resolved))
}

/// 编码 KBD_GET_EDGES 请求载荷: [max(u8)]; 0 = 用固件默认上限。
pub fn encode_kbd_get_edges(max: u8) -> Vec<u8> {
    vec![max]
}

/// 解码 KBD_GET_EDGES 响应:
/// [cap(u16 LE), overflow(u32 LE), remaining(u16 LE), count(u8)] + count×(t_us u32 LE, raw u16, deb u16, out u16)
pub fn decode_kbd_get_edges(payload: &[u8]) -> Result<KbdEdgeBatch, String> {
    if payload.len() < 9 {
        return Err(format!(
            "KBD_GET_EDGES 响应过短: {} 字节 (需 >= 9)",
            payload.len()
        ));
    }
    let count = payload[8] as usize;
    let need = 9 + count * 10;
    if payload.len() < need {
        return Err(format!(
            "KBD_GET_EDGES 响应截断: 声明 {} 条需 {} 字节, 实收 {}",
            count,
            need,
            payload.len()
        ));
    }
    let mut recs = Vec::with_capacity(count);
    for i in 0..count {
        let b = 9 + i * 10;
        recs.push(KbdEdgeRec {
            t_us: u32::from_le_bytes([payload[b], payload[b + 1], payload[b + 2], payload[b + 3]]),
            raw: u16::from_le_bytes([payload[b + 4], payload[b + 5]]),
            deb: u16::from_le_bytes([payload[b + 6], payload[b + 7]]),
            out: u16::from_le_bytes([payload[b + 8], payload[b + 9]]),
        });
    }
    Ok(KbdEdgeBatch {
        cap: u16::from_le_bytes([payload[0], payload[1]]),
        overflow: u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]),
        remaining: u16::from_le_bytes([payload[6], payload[7]]),
        recs,
    })
}

// ============================================================================
// mai2 串口运行态 (MAI2_GET_STATE 0x78 / MAI2_SET_SEND_EN 0x79)
// ============================================================================

/// mai2 串口(游戏触控上报)运行态。`status`: 0=停 1=就绪 2=运行。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mai2State {
    pub send_en: bool,
    pub status: u8,
    pub baud: u32,
}

/// 编码 MAI2_SET_SEND_EN 请求载荷: [en(u8)]。
pub fn encode_mai2_set_send_en(en: bool) -> Vec<u8> {
    vec![u8::from(en)]
}

/// 解码 MAI2_GET_STATE 响应载荷。
/// payload = send_en(u8) + status(u8) + baud(u32 LE)
pub fn decode_mai2_get_state(payload: &[u8]) -> Result<Mai2State, String> {
    if payload.len() < 6 {
        return Err(format!(
            "MAI2_GET_STATE 响应过短: {} 字节 (需 >= 6)",
            payload.len()
        ));
    }
    Ok(Mai2State {
        send_en: payload[0] != 0,
        status: payload[1],
        baud: u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]),
    })
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
        assert!(
            verify_crc_checksum(),
            "CRC-16/CCITT-FALSE test vector failed"
        );
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
        assert_eq!(
            nak.flags & (FLAG_RESPONSE | FLAG_NAK_ERR),
            FLAG_RESPONSE | FLAG_NAK_ERR
        );
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
