//! 遥测/参数编解码 (#6g-1)
//!
//! 与固件 `main_firmware/src/service/sensor_link/sensor_link.cpp` 对齐的协议实现。
//! 所有多字节值 LE(小端)。
//!
//! 设计参照 `protocol_design.md` 修订3 T1~T5。

// ============================================================================
// 参数 ID 常量(T1)
// ============================================================================

/// 手指检测阈值
pub const PARAM_FINGER_TH: u8 = 0x01;
/// 噪声阈值
pub const PARAM_NOISE_TH: u8 = 0x02;
/// 负噪声阈值
pub const PARAM_NEG_NOISE_TH: u8 = 0x03;
/// 迟滞
pub const PARAM_HYSTERESIS: u8 = 0x04;
/// 触发去抖计数
pub const PARAM_ON_DEBOUNCE: u8 = 0x05;
/// 低基线复位
pub const PARAM_LOW_BSLN_RST: u8 = 0x06;
/// 扫描分辨率
pub const PARAM_RESOLUTION: u8 = 0x07;
/// 感应时钟分频
pub const PARAM_SNS_CLK_DIV: u8 = 0x08;
/// 调制 IDAC
pub const PARAM_IDAC_MOD: u8 = 0x09;
/// 传感时钟源
pub const PARAM_SNS_CLK_SOURCE: u8 = 0x0A;
/// IDAC 增幅档
pub const PARAM_IDAC_GAIN: u8 = 0x0B;

/// 所有已知 param_id 列表(对齐固件 kParamIds 顺序)
pub const KNOWN_PARAM_IDS: &[u8] = &[
    PARAM_FINGER_TH,
    PARAM_NOISE_TH,
    PARAM_NEG_NOISE_TH,
    PARAM_HYSTERESIS,
    PARAM_ON_DEBOUNCE,
    PARAM_LOW_BSLN_RST,
    PARAM_RESOLUTION,
    PARAM_SNS_CLK_DIV,
    PARAM_IDAC_MOD,
    PARAM_SNS_CLK_SOURCE,
    PARAM_IDAC_GAIN,
];

// ============================================================================
// 遥测字段位(T2)
// ============================================================================

/// 原始采样值
pub const FIELD_RAW: u8 = 0x01;
/// 基线值
pub const FIELD_BASELINE: u8 = 0x02;
/// 差值
pub const FIELD_DIFF: u8 = 0x04;
/// 触摸状态
pub const FIELD_STATUS: u8 = 0x08;
/// 采样率统计(帧级，非逐通道)：帧头 fields 字节后附 samples_per_sec(u32 LE)+scan_period_us(u32 LE)
pub const FIELD_STATS: u8 = 0x10;
/// 触控输出流水线延迟(帧级)：STATS 后附 spi/proc/usb 三个 u16 LE 滚动最大值
pub const FIELD_LATENCY: u8 = 0x20;

// ============================================================================
// 遥测编码函数(T3)
// ============================================================================

/// 编码 TELEM_START 请求载荷
/// payload = mode(u8) + rate_hz(u16 LE) + fields(u8) + ch_mask(u64 LE)
pub fn encode_telem_start(mode: u8, rate_hz: u16, fields: u8, ch_mask: u64) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(mode);
    payload.push((rate_hz & 0xFF) as u8);
    payload.push((rate_hz >> 8) as u8);
    payload.push(fields);
    payload.push((ch_mask & 0xFF) as u8);
    payload.push(((ch_mask >> 8) & 0xFF) as u8);
    payload.push(((ch_mask >> 16) & 0xFF) as u8);
    payload.push(((ch_mask >> 24) & 0xFF) as u8);
    payload.push(((ch_mask >> 32) & 0xFF) as u8);
    payload.push(((ch_mask >> 40) & 0xFF) as u8);
    payload.push(((ch_mask >> 48) & 0xFF) as u8);
    payload.push(((ch_mask >> 56) & 0xFF) as u8);
    payload
}

/// 编码 PARAM_GET 请求载荷
/// payload = channel(u8) + param_id(u8)
pub fn encode_param_get(channel: u8, param_id: u8) -> Vec<u8> {
    vec![channel, param_id]
}

/// 编码 PARAM_SET 请求载荷
/// payload = channel(u8) + param_id(u8) + value(u32 LE)
pub fn encode_param_set(channel: u8, param_id: u8, value: u32) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(channel);
    payload.push(param_id);
    payload.push((value & 0xFF) as u8);
    payload.push(((value >> 8) & 0xFF) as u8);
    payload.push(((value >> 16) & 0xFF) as u8);
    payload.push(((value >> 24) & 0xFF) as u8);
    payload
}

/// 编码 PARAM_GET_ALL 请求载荷
/// payload = channel(u8)
pub fn encode_param_get_all(channel: u8) -> Vec<u8> {
    vec![channel]
}

/// 编码 CP_MEASURE 请求载荷（空）。
pub fn encode_cp_measure() -> Vec<u8> {
    Vec::new()
}

/// 编码 CP_GET 请求载荷。
/// payload = channel(u8)
pub fn encode_cp_get(channel: u8) -> Vec<u8> {
    vec![channel]
}

/// 编码 CALIBRATE/BASELINE_RESET 的通道掩码
/// payload = ch_mask(u64 LE)
pub fn encode_ch_mask(ch_mask: u64) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push((ch_mask & 0xFF) as u8);
    payload.push(((ch_mask >> 8) & 0xFF) as u8);
    payload.push(((ch_mask >> 16) & 0xFF) as u8);
    payload.push(((ch_mask >> 24) & 0xFF) as u8);
    payload.push(((ch_mask >> 32) & 0xFF) as u8);
    payload.push(((ch_mask >> 40) & 0xFF) as u8);
    payload.push(((ch_mask >> 48) & 0xFF) as u8);
    payload.push(((ch_mask >> 56) & 0xFF) as u8);
    payload
}

// ============================================================================
// 遥测解码结构体与函数(T3)
// ============================================================================

/// 单通道采样
#[derive(Debug, Clone)]
pub struct ChannelSample {
    pub ch: u8,
    pub raw: Option<u16>,
    pub bsln: Option<u16>,
    pub diff: Option<i16>,
    pub status: Option<u8>,
}

/// 遥测数据帧
#[derive(Debug, Clone)]
pub struct TelemFrame {
    pub ts_us: u32,
    pub fields: u8,
    pub samples_per_sec: u32,
    pub scan_period_us: u32,
    pub lat_spi_us: u16,
    pub lat_proc_us: u16,
    pub lat_usb_us: u16,
    pub samples: Vec<ChannelSample>,
}

/// 解码 TELEM_DATA 响应载荷
/// payload = ts_us(u32 LE) + ch_count(u8) + fields(u8) + [per_channel: ...]
/// 每通道按 fields 顺序: ch_index(u8) + (raw u16LE)? + (bsln u16LE)? + (diff i16LE)? + (status u8)?
pub fn decode_telem_data(payload: &[u8]) -> Result<TelemFrame, String> {
    if payload.len() < 6 {
        return Err("TELEM_DATA payload too short (need >= 6 bytes)".to_string());
    }

    let mut pos = 0;

    // ts_us(u32 LE)
    let ts_us = (payload[pos] as u32)
        | ((payload[pos + 1] as u32) << 8)
        | ((payload[pos + 2] as u32) << 16)
        | ((payload[pos + 3] as u32) << 24);
    pos += 4;

    // ch_count(u8)
    let ch_count = payload[pos] as usize;
    pos += 1;

    // fields(u8)
    let fields = payload[pos];
    pos += 1;

    // STATS(帧级，可选)：samples_per_sec(u32 LE) + scan_period_us(u32 LE)
    let (samples_per_sec, scan_period_us) = if (fields & FIELD_STATS) != 0 {
        if pos + 8 > payload.len() {
            return Err("Payload truncated while reading STATS field".to_string());
        }
        let sps = (payload[pos] as u32)
            | ((payload[pos + 1] as u32) << 8)
            | ((payload[pos + 2] as u32) << 16)
            | ((payload[pos + 3] as u32) << 24);
        let spu = (payload[pos + 4] as u32)
            | ((payload[pos + 5] as u32) << 8)
            | ((payload[pos + 6] as u32) << 16)
            | ((payload[pos + 7] as u32) << 24);
        pos += 8;
        (sps, spu)
    } else {
        (0u32, 0u32)
    };

    let (lat_spi_us, lat_proc_us, lat_usb_us) = if (fields & FIELD_LATENCY) != 0 {
        if pos + 6 > payload.len() {
            return Err("TELEM_DATA truncated at LATENCY block".to_string());
        }
        let a = u16::from_le_bytes([payload[pos], payload[pos + 1]]);
        let b = u16::from_le_bytes([payload[pos + 2], payload[pos + 3]]);
        let c = u16::from_le_bytes([payload[pos + 4], payload[pos + 5]]);
        pos += 6;
        (a, b, c)
    } else { (0u16, 0u16, 0u16) };

    let mut samples = Vec::new();

    // 解析每个通道
    for _ in 0..ch_count {
        if pos >= payload.len() {
            return Err("Payload truncated while reading channel data".to_string());
        }

        // ch_index(u8)
        let ch = payload[pos];
        pos += 1;

        let mut sample = ChannelSample { ch, raw: None, bsln: None, diff: None, status: None };

        // RAW (u16 LE)?
        if (fields & FIELD_RAW) != 0 {
            if pos + 2 > payload.len() {
                return Err("Payload truncated while reading RAW field".to_string());
            }
            let raw = (payload[pos] as u16) | ((payload[pos + 1] as u16) << 8);
            sample.raw = Some(raw);
            pos += 2;
        }

        // BASELINE (u16 LE)?
        if (fields & FIELD_BASELINE) != 0 {
            if pos + 2 > payload.len() {
                return Err("Payload truncated while reading BASELINE field".to_string());
            }
            let bsln = (payload[pos] as u16) | ((payload[pos + 1] as u16) << 8);
            sample.bsln = Some(bsln);
            pos += 2;
        }

        // DIFF (i16 LE)?
        if (fields & FIELD_DIFF) != 0 {
            if pos + 2 > payload.len() {
                return Err("Payload truncated while reading DIFF field".to_string());
            }
            let diff_u16 = (payload[pos] as u16) | ((payload[pos + 1] as u16) << 8);
            let diff = diff_u16 as i16;
            sample.diff = Some(diff);
            pos += 2;
        }

        // STATUS (u8)?
        if (fields & FIELD_STATUS) != 0 {
            if pos >= payload.len() {
                return Err("Payload truncated while reading STATUS field".to_string());
            }
            let status = payload[pos];
            sample.status = Some(status);
            pos += 1;
        }

        samples.push(sample);
    }

    Ok(TelemFrame {
        ts_us,
        fields,
        samples_per_sec,
        scan_period_us,
        lat_spi_us,
        lat_proc_us,
        lat_usb_us,
        samples,
    })
}

/// 解码 PARAM_GET 响应载荷
/// payload = channel(u8) + param_id(u8) + value(u32 LE)
/// 返回 (channel, param_id, value)
pub fn decode_param_get(payload: &[u8]) -> Result<(u8, u8, u32), String> {
    if payload.len() < 6 {
        return Err("PARAM_GET response too short (need >= 6 bytes)".to_string());
    }

    let ch = payload[0];
    let param_id = payload[1];
    let value = (payload[2] as u32)
        | ((payload[3] as u32) << 8)
        | ((payload[4] as u32) << 16)
        | ((payload[5] as u32) << 24);

    Ok((ch, param_id, value))
}

/// 解码 CP_GET 响应载荷。
/// payload = channel(u8) + cp(u32 LE)，返回 `(channel, cp_ff)`。
pub fn decode_cp_get(payload: &[u8]) -> Result<(u8, u32), String> {
    if payload.len() != 5 {
        return Err(format!("CP_GET response must be 5 bytes, got {}", payload.len()));
    }
    Ok((
        payload[0],
        u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]),
    ))
}

/// 解码 PARAM_GET_ALL 响应载荷
/// payload = channel(u8) + count(u8) + [param_id(u8) + value(u32 LE)]×count
/// 返回 (channel, Vec<(param_id, value)>)
pub fn decode_param_get_all(payload: &[u8]) -> Result<(u8, Vec<(u8, u32)>), String> {
    if payload.len() < 2 {
        return Err("PARAM_GET_ALL response too short (need >= 2 bytes)".to_string());
    }

    let ch = payload[0];
    let count = payload[1] as usize;

    let mut params = Vec::new();
    let mut pos = 2;

    for _ in 0..count {
        if pos + 5 > payload.len() {
            return Err(format!(
                "PARAM_GET_ALL truncated: expected {} entries, got {}",
                count,
                params.len()
            ));
        }

        let param_id = payload[pos];
        pos += 1;
        let value = (payload[pos] as u32)
            | ((payload[pos + 1] as u32) << 8)
            | ((payload[pos + 2] as u32) << 16)
            | ((payload[pos + 3] as u32) << 24);
        pos += 4;

        params.push((param_id, value));
    }

    Ok((ch, params))
}

// ============================================================================
// 单元测试
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_telem_start_byte_layout() {
        // 构造已知值: mode=0, rate_hz=1000(0x03E8), fields=RAW|BASELINE|DIFF|STATUS, ch_mask=0x0000000100000001
        let mode = 0u8;
        let rate_hz = 1000u16;
        let fields = FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS;
        let ch_mask = 0x0000000100000001u64;

        let payload = encode_telem_start(mode, rate_hz, fields, ch_mask);

        assert_eq!(payload.len(), 12, "TELEM_START payload should be 12 bytes");
        assert_eq!(payload[0], mode, "mode");
        assert_eq!(payload[1], 0xE8, "rate_hz low byte");
        assert_eq!(payload[2], 0x03, "rate_hz high byte");
        assert_eq!(payload[3], fields, "fields");
        assert_eq!(payload[4], 0x01, "ch_mask byte 0");
        assert_eq!(payload[5], 0x00, "ch_mask byte 1");
        assert_eq!(payload[6], 0x00, "ch_mask byte 2");
        assert_eq!(payload[7], 0x00, "ch_mask byte 3");
        assert_eq!(payload[8], 0x01, "ch_mask byte 4");
        assert_eq!(payload[9], 0x00, "ch_mask byte 5");
        assert_eq!(payload[10], 0x00, "ch_mask byte 6");
        assert_eq!(payload[11], 0x00, "ch_mask byte 7");
    }

    #[test]
    fn test_encode_param_set_byte_layout() {
        // 构造: ch=5, param_id=PARAM_FINGER_TH(0x01), value=100(0x00000064)
        let ch = 5u8;
        let param_id = PARAM_FINGER_TH;
        let value = 100u32;

        let payload = encode_param_set(ch, param_id, value);

        assert_eq!(payload.len(), 6, "PARAM_SET payload should be 6 bytes");
        assert_eq!(payload[0], ch);
        assert_eq!(payload[1], param_id);
        assert_eq!(payload[2], 0x64, "value byte 0");
        assert_eq!(payload[3], 0x00, "value byte 1");
        assert_eq!(payload[4], 0x00, "value byte 2");
        assert_eq!(payload[5], 0x00, "value byte 3");
    }

    #[test]
    fn test_decode_telem_data_simple() {
        // 构造: ts_us=0x12345678, ch_count=2, fields=RAW|DIFF
        // 通道0: raw=0x1234, diff=0x5678
        // 通道5: raw=0xABCD, diff=0xEF00
        let mut payload = Vec::new();

        // ts_us(u32 LE) = 0x12345678
        payload.push(0x78);
        payload.push(0x56);
        payload.push(0x34);
        payload.push(0x12);

        // ch_count=2
        payload.push(2);

        // fields = RAW | DIFF = 0x01 | 0x04 = 0x05
        payload.push(0x05);

        // Channel 0
        payload.push(0);       // ch_index
        payload.push(0x34);    // raw low
        payload.push(0x12);    // raw high
        payload.push(0x78);    // diff low
        payload.push(0x56);    // diff high

        // Channel 5
        payload.push(5);       // ch_index
        payload.push(0xCD);    // raw low
        payload.push(0xAB);    // raw high
        payload.push(0x00);    // diff low
        payload.push(0xEF);    // diff high

        let frame = decode_telem_data(&payload).expect("decode should succeed");

        assert_eq!(frame.ts_us, 0x12345678);
        assert_eq!(frame.fields, 0x05);
        assert_eq!(frame.samples.len(), 2);

        // Check channel 0
        assert_eq!(frame.samples[0].ch, 0);
        assert_eq!(frame.samples[0].raw, Some(0x1234));
        assert_eq!(frame.samples[0].bsln, None);
        assert_eq!(frame.samples[0].diff, Some(0x5678i16));
        assert_eq!(frame.samples[0].status, None);

        // Check channel 5
        assert_eq!(frame.samples[1].ch, 5);
        assert_eq!(frame.samples[1].raw, Some(0xABCD));
        assert_eq!(frame.samples[1].bsln, None);
        assert_eq!(frame.samples[1].diff, Some(-4352i16)); // 0xEF00 as i16
        assert_eq!(frame.samples[1].status, None);
    }

    #[test]
    fn test_decode_telem_data_all_fields() {
        // 构造: 1 通道，所有字段
        let mut payload = Vec::new();

        // ts_us=0x11223344
        payload.push(0x44);
        payload.push(0x33);
        payload.push(0x22);
        payload.push(0x11);

        // ch_count=1
        payload.push(1);

        // fields = RAW | BASELINE | DIFF | STATUS = 0x0F
        payload.push(0x0F);

        // Channel 0
        payload.push(0);       // ch_index
        payload.push(0x34);    // raw low
        payload.push(0x12);    // raw high
        payload.push(0x78);    // bsln low
        payload.push(0x56);    // bsln high
        payload.push(0x12);    // diff low
        payload.push(0x34);    // diff high
        payload.push(0x01);    // status

        let frame = decode_telem_data(&payload).expect("decode should succeed");

        assert_eq!(frame.samples.len(), 1);
        let sample = &frame.samples[0];
        assert_eq!(sample.ch, 0);
        assert_eq!(sample.raw, Some(0x1234));
        assert_eq!(sample.bsln, Some(0x5678));
        assert_eq!(sample.diff, Some(0x3412i16));
        assert_eq!(sample.status, Some(0x01));
    }

    #[test]
    fn test_decode_param_get() {
        // 响应: ch=3, param_id=PARAM_FINGER_TH(0x01), value=250(0x000000FA)
        let mut payload = Vec::new();
        payload.push(3);       // channel
        payload.push(PARAM_FINGER_TH);
        payload.push(0xFA);    // value low
        payload.push(0x00);
        payload.push(0x00);
        payload.push(0x00);

        let (ch, param_id, value) = decode_param_get(&payload).expect("decode should succeed");

        assert_eq!(ch, 3);
        assert_eq!(param_id, PARAM_FINGER_TH);
        assert_eq!(value, 250);
    }

    #[test]
    fn test_decode_param_get_all() {
        // 响应: ch=7, count=3, params: (PARAM_FINGER_TH, 100), (PARAM_NOISE_TH, 40), (PARAM_HYSTERESIS, 10)
        let mut payload = Vec::new();
        payload.push(7);       // channel
        payload.push(3);       // count

        // Entry 0: param_id=PARAM_FINGER_TH, value=100
        payload.push(PARAM_FINGER_TH);
        payload.push(100);
        payload.push(0);
        payload.push(0);
        payload.push(0);

        // Entry 1: param_id=PARAM_NOISE_TH, value=40
        payload.push(PARAM_NOISE_TH);
        payload.push(40);
        payload.push(0);
        payload.push(0);
        payload.push(0);

        // Entry 2: param_id=PARAM_HYSTERESIS, value=10
        payload.push(PARAM_HYSTERESIS);
        payload.push(10);
        payload.push(0);
        payload.push(0);
        payload.push(0);

        let (ch, params) = decode_param_get_all(&payload).expect("decode should succeed");

        assert_eq!(ch, 7);
        assert_eq!(params.len(), 3);
        assert_eq!(params[0], (PARAM_FINGER_TH, 100));
        assert_eq!(params[1], (PARAM_NOISE_TH, 40));
        assert_eq!(params[2], (PARAM_HYSTERESIS, 10));
    }

    #[test]
    fn test_known_param_ids_count() {
        // 应该有 11 个已知 param_id
        assert_eq!(KNOWN_PARAM_IDS.len(), 11);
        assert_eq!(KNOWN_PARAM_IDS[0], PARAM_FINGER_TH);
        assert_eq!(KNOWN_PARAM_IDS[10], PARAM_IDAC_GAIN);
        assert_eq!(KNOWN_PARAM_IDS, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B]);
    }

    #[test]
    fn test_encode_ch_mask_all_channels() {
        // ch_mask = 0xFFFFFFFFFFFFFFFF (所有 64 位都置 1)
        let payload = encode_ch_mask(0xFFFFFFFFFFFFFFFF);
        assert_eq!(payload.len(), 8);
        assert!(payload.iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn test_encode_ch_mask_sparse() {
        // ch_mask = 0x0000000100000001 (通道 0 和 32)
        let payload = encode_ch_mask(0x0000000100000001u64);
        assert_eq!(payload.len(), 8);
        assert_eq!(payload[0], 0x01);
        assert_eq!(payload[4], 0x01);
        assert!(payload[1..4].iter().all(|&b| b == 0));
        assert!(payload[5..8].iter().all(|&b| b == 0));
    }

    #[test]
    fn test_decode_telem_data_truncated_payload_error() {
        let payload = vec![0x01, 0x02];
        let result = decode_telem_data(&payload);
        assert!(result.is_err());
    }

    #[test]
    fn test_decode_param_get_truncated_error() {
        let payload = vec![0x05, 0x01, 0x64];
        let result = decode_param_get(&payload);
        assert!(result.is_err());
    }
}
