const ZONE_RINGS: [(char, usize); 5] = [('A', 8), ('B', 8), ('C', 2), ('D', 8), ('E', 8)];

/// 把绑区 index(0..33)转成 maimai 分区标签。
pub fn zone_label(index: usize) -> String {
    let mut idx = index;
    for (letter, count) in ZONE_RINGS {
        if idx < count {
            return format!("{}{}", letter, idx + 1);
        }
        idx -= count;
    }
    format!("?{}", index)
}

/// 把绑区 index(0..33)转成配置 key。
pub fn zone_key(index: usize) -> String {
    format!("bind.map{:02}", index)
}

pub const HID_POINT_COUNT: usize = 36;
pub const HID_COORD_MAX: u16 = 32767;

pub fn hid_en_key(ch: usize) -> String {
    format!("hid.en{:02}", ch)
}

pub fn hid_x_key(ch: usize) -> String {
    format!("hid.x{:02}", ch)
}

pub fn hid_y_key(ch: usize) -> String {
    format!("hid.y{:02}", ch)
}

pub fn binding_channel_mask(value: u32) -> u32 {
    value & 0x00FF_FFFF
}

pub fn binding_device_mask(value: u32) -> u8 {
    (value >> 24) as u8
}

pub fn make_binding(device_mask: u8, channel_mask: u32) -> u32 {
    ((device_mask as u32) << 24) | (channel_mask & 0x00FF_FFFF)
}
