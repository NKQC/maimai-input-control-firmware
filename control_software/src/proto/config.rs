//! Configuration Entry encoding/decoding (#6e-1)
//!
//! Mirrors firmware `main_firmware/src/protocol/host_cmd/host_cmd.cpp`
//! Entry format (per protocol_design.md C3):
//! ```text
//! type(u8) + has_range(u8) + key_len(u8) + key[key_len]
//!   + value(按type) + [min+max if has_range==1]
//! ```
//!
//! All multi-byte values encoded little-endian (LE).

use std::f32;

// ============================================================================
// Type Codes (C1)
// ============================================================================

/// Configuration value type codes (mirrors firmware enum class ConfigValueType)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ConfigValueType {
    Bool = 0,
    Int8 = 1,
    Uint8 = 2,
    Uint16 = 3,
    Uint32 = 4,
    Float = 5,
    String = 6,
}

impl TryFrom<u8> for ConfigValueType {
    type Error = String;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(ConfigValueType::Bool),
            1 => Ok(ConfigValueType::Int8),
            2 => Ok(ConfigValueType::Uint8),
            3 => Ok(ConfigValueType::Uint16),
            4 => Ok(ConfigValueType::Uint32),
            5 => Ok(ConfigValueType::Float),
            6 => Ok(ConfigValueType::String),
            _ => Err(format!("Unknown type code: {}", v)),
        }
    }
}

// ============================================================================
// ConfigValue Enum
// ============================================================================

/// Configuration value wrapper (mirrors firmware ConfigValue)
#[derive(Debug, Clone)]
pub enum CfgValue {
    Bool(bool),
    I8(i8),
    U8(u8),
    U16(u16),
    U32(u32),
    F32(f32),
    Str(String),
}

impl CfgValue {
    /// Get the type code for this value
    pub fn type_code(&self) -> u8 {
        match self {
            CfgValue::Bool(_) => ConfigValueType::Bool as u8,
            CfgValue::I8(_) => ConfigValueType::Int8 as u8,
            CfgValue::U8(_) => ConfigValueType::Uint8 as u8,
            CfgValue::U16(_) => ConfigValueType::Uint16 as u8,
            CfgValue::U32(_) => ConfigValueType::Uint32 as u8,
            CfgValue::F32(_) => ConfigValueType::Float as u8,
            CfgValue::Str(_) => ConfigValueType::String as u8,
        }
    }

    /// Get the type enum for this value
    pub fn type_enum(&self) -> ConfigValueType {
        match self {
            CfgValue::Bool(_) => ConfigValueType::Bool,
            CfgValue::I8(_) => ConfigValueType::Int8,
            CfgValue::U8(_) => ConfigValueType::Uint8,
            CfgValue::U16(_) => ConfigValueType::Uint16,
            CfgValue::U32(_) => ConfigValueType::Uint32,
            CfgValue::F32(_) => ConfigValueType::Float,
            CfgValue::Str(_) => ConfigValueType::String,
        }
    }

    /// Encode value to bytes (LE), not including type/has_range/key
    fn encode_value(&self, out: &mut Vec<u8>) -> Result<(), String> {
        match self {
            CfgValue::Bool(v) => {
                out.push(if *v { 1 } else { 0 });
                Ok(())
            }
            CfgValue::I8(v) => {
                out.push(*v as u8);
                Ok(())
            }
            CfgValue::U8(v) => {
                out.push(*v);
                Ok(())
            }
            CfgValue::U16(v) => {
                out.push((v & 0xFF) as u8);
                out.push((v >> 8) as u8);
                Ok(())
            }
            CfgValue::U32(v) => {
                out.push((v & 0xFF) as u8);
                out.push(((v >> 8) & 0xFF) as u8);
                out.push(((v >> 16) & 0xFF) as u8);
                out.push(((v >> 24) & 0xFF) as u8);
                Ok(())
            }
            CfgValue::F32(v) => {
                let bits = v.to_bits();
                out.push((bits & 0xFF) as u8);
                out.push(((bits >> 8) & 0xFF) as u8);
                out.push(((bits >> 16) & 0xFF) as u8);
                out.push(((bits >> 24) & 0xFF) as u8);
                Ok(())
            }
            CfgValue::Str(v) => {
                let len = v.len() as u16;
                out.push((len & 0xFF) as u8);
                out.push((len >> 8) as u8);
                out.extend_from_slice(v.as_bytes());
                Ok(())
            }
        }
    }

    /// Decode value from bytes given type (LE), returns bytes consumed
    fn decode_value(type_enum: ConfigValueType, buf: &[u8]) -> Result<(CfgValue, usize), String> {
        if buf.is_empty() {
            return Err("Buffer empty when decoding value".to_string());
        }

        match type_enum {
            ConfigValueType::Bool => {
                let v = buf[0] != 0;
                Ok((CfgValue::Bool(v), 1))
            }
            ConfigValueType::Int8 => {
                let v = buf[0] as i8;
                Ok((CfgValue::I8(v), 1))
            }
            ConfigValueType::Uint8 => {
                let v = buf[0];
                Ok((CfgValue::U8(v), 1))
            }
            ConfigValueType::Uint16 => {
                if buf.len() < 2 {
                    return Err("Not enough bytes for U16".to_string());
                }
                let v = (buf[0] as u16) | ((buf[1] as u16) << 8);
                Ok((CfgValue::U16(v), 2))
            }
            ConfigValueType::Uint32 => {
                if buf.len() < 4 {
                    return Err("Not enough bytes for U32".to_string());
                }
                let v = (buf[0] as u32)
                    | ((buf[1] as u32) << 8)
                    | ((buf[2] as u32) << 16)
                    | ((buf[3] as u32) << 24);
                Ok((CfgValue::U32(v), 4))
            }
            ConfigValueType::Float => {
                if buf.len() < 4 {
                    return Err("Not enough bytes for F32".to_string());
                }
                let bits = (buf[0] as u32)
                    | ((buf[1] as u32) << 8)
                    | ((buf[2] as u32) << 16)
                    | ((buf[3] as u32) << 24);
                let v = f32::from_bits(bits);
                Ok((CfgValue::F32(v), 4))
            }
            ConfigValueType::String => {
                if buf.len() < 2 {
                    return Err("Not enough bytes for string length".to_string());
                }
                let len = (buf[0] as u16) | ((buf[1] as u16) << 8);
                let len = len as usize;
                if buf.len() < 2 + len {
                    return Err("Not enough bytes for string data".to_string());
                }
                let s = String::from_utf8_lossy(&buf[2..2 + len]).into_owned();
                Ok((CfgValue::Str(s), 2 + len))
            }
        }
    }
}

// ============================================================================
// ConfigEntry Structure
// ============================================================================

/// Configuration entry (key + value with optional range)
#[derive(Debug, Clone)]
pub struct ConfigEntry {
    pub key: String,
    pub value: CfgValue,
    /// If present, contains (min, max) for validation
    pub range: Option<(CfgValue, CfgValue)>,
}

impl ConfigEntry {
    /// Create a new entry without range
    pub fn new(key: String, value: CfgValue) -> Self {
        ConfigEntry { key, value, range: None }
    }

    /// Create a new entry with range
    pub fn with_range(key: String, value: CfgValue, min: CfgValue, max: CfgValue) -> Self {
        ConfigEntry { key, value, range: Some((min, max)) }
    }
}

// ============================================================================
// Entry Encoding/Decoding
// ============================================================================

/// Encode a single ConfigEntry to bytes (C3)
pub fn encode_entry(entry: &ConfigEntry) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();

    // type(u8)
    out.push(entry.value.type_code());

    // has_range(u8)
    out.push(if entry.range.is_some() { 1 } else { 0 });

    // key_len(u8) + key
    if entry.key.len() > 255 {
        return Err(format!("Key too long: {} bytes", entry.key.len()));
    }
    out.push(entry.key.len() as u8);
    out.extend_from_slice(entry.key.as_bytes());

    // value
    entry.value.encode_value(&mut out)?;

    // min/max if has_range
    if let Some((min, max)) = &entry.range {
        // Verify type consistency
        if min.type_code() != entry.value.type_code() || max.type_code() != entry.value.type_code() {
            return Err("Range min/max type mismatch with value type".to_string());
        }
        min.encode_value(&mut out)?;
        max.encode_value(&mut out)?;
    }

    Ok(out)
}

/// Decode a single ConfigEntry from bytes, returns (entry, bytes_consumed)
pub fn decode_entry(buf: &[u8]) -> Result<(ConfigEntry, usize), String> {
    if buf.len() < 3 {
        return Err("Buffer too short for entry header".to_string());
    }

    let mut pos = 0;

    // type(u8)
    let type_code = buf[pos];
    let type_enum = ConfigValueType::try_from(type_code)?;
    pos += 1;

    // has_range(u8)
    let has_range = buf[pos] != 0;
    pos += 1;

    // key_len(u8) + key
    let key_len = buf[pos] as usize;
    pos += 1;
    if pos + key_len > buf.len() {
        return Err("Buffer too short for key".to_string());
    }
    let key = String::from_utf8_lossy(&buf[pos..pos + key_len]).into_owned();
    pos += key_len;

    // value
    let (value, val_len) = CfgValue::decode_value(type_enum, &buf[pos..])?;
    pos += val_len;

    // range if has_range
    let range = if has_range {
        let (min, min_len) = CfgValue::decode_value(type_enum, &buf[pos..])?;
        pos += min_len;
        let (max, max_len) = CfgValue::decode_value(type_enum, &buf[pos..])?;
        pos += max_len;
        Some((min, max))
    } else {
        None
    };

    let entry = ConfigEntry { key, value, range };
    Ok((entry, pos))
}

/// Decode multiple entries from buffer in format: count(u16 LE) + Entry×count
pub fn decode_entries(buf: &[u8]) -> Result<Vec<ConfigEntry>, String> {
    if buf.len() < 2 {
        return Err("Buffer too short for entry count".to_string());
    }

    let count = (buf[0] as u16) | ((buf[1] as u16) << 8);
    let count = count as usize;

    let mut entries = Vec::new();
    let mut pos = 2;

    for _ in 0..count {
        if pos >= buf.len() {
            return Err(format!(
                "Buffer too short: expected {} entries, got {}",
                count,
                entries.len()
            ));
        }
        let (entry, consumed) = decode_entry(&buf[pos..])?;
        pos += consumed;
        entries.push(entry);
    }

    Ok(entries)
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_bool() {
        let entry = ConfigEntry::new("test.bool".to_string(), CfgValue::Bool(true));
        let encoded = encode_entry(&entry).unwrap();
        let (decoded, consumed) = decode_entry(&encoded).unwrap();

        assert_eq!(decoded.key, entry.key);
        assert_eq!(decoded.value.type_code(), entry.value.type_code());
        match decoded.value {
            CfgValue::Bool(v) => assert_eq!(v, true),
            _ => panic!("Wrong type"),
        }
        assert_eq!(consumed, encoded.len());
    }

    #[test]
    fn test_encode_decode_u16_with_range() {
        let min = CfgValue::U16(1);
        let max = CfgValue::U16(1000);
        let entry = ConfigEntry::with_range(
            "comm.rate_limit_hz".to_string(),
            CfgValue::U16(120),
            min,
            max,
        );

        let encoded = encode_entry(&entry).unwrap();
        let (decoded, _consumed) = decode_entry(&encoded).unwrap();

        assert_eq!(decoded.key, "comm.rate_limit_hz");
        match decoded.value {
            CfgValue::U16(v) => assert_eq!(v, 120),
            _ => panic!("Wrong type"),
        }
        assert!(decoded.range.is_some());
        let (min_dec, max_dec) = decoded.range.unwrap();
        match (min_dec, max_dec) {
            (CfgValue::U16(min_v), CfgValue::U16(max_v)) => {
                assert_eq!(min_v, 1);
                assert_eq!(max_v, 1000);
            }
            _ => panic!("Wrong range type"),
        }
    }

    #[test]
    fn test_encode_decode_string() {
        let entry = ConfigEntry::new(
            "device.name".to_string(),
            CfgValue::Str("mai2control".to_string()),
        );
        let encoded = encode_entry(&entry).unwrap();
        let (decoded, _consumed) = decode_entry(&encoded).unwrap();

        assert_eq!(decoded.key, "device.name");
        match decoded.value {
            CfgValue::Str(s) => assert_eq!(s, "mai2control"),
            _ => panic!("Wrong type"),
        }
    }

    #[test]
    fn test_encode_decode_f32() {
        let entry =
            ConfigEntry::new("touch.sensitivity".to_string(), CfgValue::F32(3.14159));
        let encoded = encode_entry(&entry).unwrap();
        let (decoded, _consumed) = decode_entry(&encoded).unwrap();

        assert_eq!(decoded.key, "touch.sensitivity");
        match decoded.value {
            CfgValue::F32(v) => assert!((v - 3.14159).abs() < 0.00001),
            _ => panic!("Wrong type"),
        }
    }

    #[test]
    fn test_encode_decode_u32() {
        let entry =
            ConfigEntry::new("led.count".to_string(), CfgValue::U32(0xFFFFFFFF));
        let encoded = encode_entry(&entry).unwrap();
        let (decoded, _consumed) = decode_entry(&encoded).unwrap();

        assert_eq!(decoded.key, "led.count");
        match decoded.value {
            CfgValue::U32(v) => assert_eq!(v, 0xFFFFFFFF),
            _ => panic!("Wrong type"),
        }
    }

    #[test]
    fn test_decode_entries_multiple() {
        // Manual construction: count=2 + 2 entries
        let mut buf = Vec::new();
        buf.push(2); // count low byte
        buf.push(0); // count high byte

        // Entry 1: UINT16 key="rate" value=100
        let e1 = ConfigEntry::new("rate".to_string(), CfgValue::U16(100));
        buf.extend_from_slice(&encode_entry(&e1).unwrap());

        // Entry 2: BOOL key="enabled" value=true
        let e2 = ConfigEntry::new("enabled".to_string(), CfgValue::Bool(true));
        buf.extend_from_slice(&encode_entry(&e2).unwrap());

        let decoded = decode_entries(&buf).unwrap();
        assert_eq!(decoded.len(), 2);
        assert_eq!(decoded[0].key, "rate");
        assert_eq!(decoded[1].key, "enabled");
    }

    #[test]
    fn test_byte_level_alignment_u16_with_range() {
        // Test byte-level alignment with firmware:
        // key="comm.rate_limit_hz" value=120 min=1 max=1000 has_range=1
        // Expected: type(3) has_range(1) key_len(18) key + value(2B LE) + min(2B LE) + max(2B LE)
        let entry = ConfigEntry::with_range(
            "comm.rate_limit_hz".to_string(),
            CfgValue::U16(120),
            CfgValue::U16(1),
            CfgValue::U16(1000),
        );

        let encoded = encode_entry(&entry).unwrap();

        // Check header
        assert_eq!(encoded[0], 3, "Type code should be 3 (UINT16)");
        assert_eq!(encoded[1], 1, "has_range should be 1");
        assert_eq!(encoded[2], 18, "key_len should be 18");

        // Check key
        let key_part = &encoded[3..21];
        assert_eq!(key_part, b"comm.rate_limit_hz");

        // Check value (120 = 0x78 0x00 LE)
        assert_eq!(encoded[21], 0x78);
        assert_eq!(encoded[22], 0x00);

        // Check min (1 = 0x01 0x00 LE)
        assert_eq!(encoded[23], 0x01);
        assert_eq!(encoded[24], 0x00);

        // Check max (1000 = 0xE8 0x03 LE)
        assert_eq!(encoded[25], 0xE8);
        assert_eq!(encoded[26], 0x03);

        // Verify round-trip
        let (decoded, consumed) = decode_entry(&encoded).unwrap();
        assert_eq!(decoded.key, "comm.rate_limit_hz");
        assert_eq!(consumed, encoded.len());
    }

    #[test]
    fn test_type_code_consistency() {
        let vals = vec![
            CfgValue::Bool(true),
            CfgValue::I8(-10),
            CfgValue::U8(255),
            CfgValue::U16(1000),
            CfgValue::U32(0x12345678),
            CfgValue::F32(1.5),
            CfgValue::Str("test".to_string()),
        ];

        let expected_codes = vec![0u8, 1, 2, 3, 4, 5, 6];

        for (val, expected) in vals.iter().zip(expected_codes.iter()) {
            assert_eq!(val.type_code(), *expected);
        }
    }
}
