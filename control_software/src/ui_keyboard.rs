//! 键盘/HID 映射工具。
//!
//! 本模块只负责 HID usage 表、按键显示和 Slint KeyEvent 文本解析，
//! 不持有 UI 或 AppController 状态，避免键盘功能与回调注册相互耦合。

/// (显示名, HID 键码) 表, 供物理键盘/触控键盘映射下拉。索引 0 = 不映射。
/// 键码为标准 HID Keyboard/Keypad usage, 与固件 HID_KeyCode 一致。
pub(crate) fn kbd_key_choices() -> Vec<(&'static str, u8)> {
    let mut v: Vec<(&'static str, u8)> = vec![("不映射", 0x00)];
    const LETTERS: [&str; 26] = [
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R",
        "S", "T", "U", "V", "W", "X", "Y", "Z",
    ];
    for (i, name) in LETTERS.iter().enumerate() {
        v.push((name, 0x04 + i as u8));
    }
    const DIGITS: [&str; 10] = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"];
    for (i, name) in DIGITS.iter().enumerate() {
        v.push((name, 0x1E + i as u8));
    }
    v.push(("Enter", 0x28));
    v.push(("Esc", 0x29));
    v.push(("Backspace", 0x2A));
    v.push(("Tab", 0x2B));
    v.push(("Space", 0x2C));
    v.extend_from_slice(&[
        ("- / _", 0x2D),
        ("= / +", 0x2E),
        ("[ / {", 0x2F),
        ("] / }", 0x30),
        ("\\ / |", 0x31),
        ("; / :", 0x33),
        ("' / \"", 0x34),
        ("` / ~ / ·", 0x35),
        (", / <", 0x36),
        (". / >", 0x37),
        ("/ / ?", 0x38),
        ("CapsLock", 0x39),
    ]);
    const FKEYS: [&str; 12] = [
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
    ];
    for (i, name) in FKEYS.iter().enumerate() {
        v.push((name, 0x3A + i as u8));
    }
    v.extend_from_slice(&[
        ("PrintScreen", 0x46),
        ("ScrollLock", 0x47),
        ("Pause", 0x48),
        ("Insert", 0x49),
        ("Home", 0x4A),
        ("PageUp", 0x4B),
        ("Delete", 0x4C),
        ("End", 0x4D),
        ("PageDown", 0x4E),
    ]);
    v.push(("→ 右", 0x4F));
    v.push(("← 左", 0x50));
    v.push(("↓ 下", 0x51));
    v.push(("↑ 上", 0x52));
    v.extend_from_slice(&[
        ("NumLock", 0x53),
        ("Keypad /", 0x54),
        ("Keypad *", 0x55),
        ("Keypad -", 0x56),
        ("Keypad +", 0x57),
        ("Keypad Enter", 0x58),
        ("Keypad 1", 0x59),
        ("Keypad 2", 0x5A),
        ("Keypad 3", 0x5B),
        ("Keypad 4", 0x5C),
        ("Keypad 5", 0x5D),
        ("Keypad 6", 0x5E),
        ("Keypad 7", 0x5F),
        ("Keypad 8", 0x60),
        ("Keypad 9", 0x61),
        ("Keypad 0", 0x62),
        ("Keypad .", 0x63),
        ("Menu", 0x65),
        ("F13", 0x68),
        ("F14", 0x69),
        ("F15", 0x6A),
        ("F16", 0x6B),
        ("F17", 0x6C),
        ("F18", 0x6D),
        ("F19", 0x6E),
        ("F20", 0x6F),
        ("F21", 0x70),
        ("F22", 0x71),
        ("F23", 0x72),
        ("F24", 0x73),
    ]);
    v.push(("LCtrl", 0xE0));
    v.push(("LShift", 0xE1));
    v.push(("LAlt", 0xE2));
    v.push(("LGui", 0xE3));
    v.push(("RCtrl", 0xE4));
    v.push(("RShift", 0xE5));
    v.push(("RAlt", 0xE6));
    v.push(("RGui", 0xE7));
    v
}

/// HID 键码 → 下拉索引(未找到=0/不映射)。
pub(crate) fn kbd_code_to_choice(code: u8) -> i32 {
    kbd_key_choices()
        .iter()
        .position(|(_, c)| *c == code)
        .map(|p| p as i32)
        .unwrap_or(0)
}

/// 下拉索引 → HID 键码(越界=0)。
pub(crate) fn kbd_choice_to_code(ci: i32) -> u8 {
    if ci < 0 {
        return 0;
    }
    kbd_key_choices()
        .get(ci as usize)
        .map(|(_, c)| *c)
        .unwrap_or(0)
}

/// HID 键码 → 显示名(用于捕获输入框显示)。未知合法值保留十六进制编码。
pub(crate) fn kbd_hid_name(code: u8) -> String {
    if code == 0 {
        return String::new();
    }
    kbd_key_choices()
        .iter()
        .find(|(_, c)| *c == code)
        .map(|(n, _)| (*n).to_string())
        .unwrap_or_else(|| format!("HID 0x{code:02X}"))
}

/// (键码, 修饰位) → 组合键显示串, 如 "Ctrl+Shift+A"。空映射显示"未设置"。
pub(crate) fn kbd_display(code: u8, modifier: u8) -> String {
    if code == 0 && modifier == 0 {
        return "未设置(点击后按键)".to_string();
    }
    let mut s = String::new();
    if modifier & 1 != 0 {
        s.push_str("Ctrl+");
    }
    if modifier & 2 != 0 {
        s.push_str("Shift+");
    }
    if modifier & 4 != 0 {
        s.push_str("Alt+");
    }
    if modifier & 8 != 0 {
        s.push_str("Gui+");
    }
    if code != 0 {
        s.push_str(&kbd_hid_name(code));
    } else {
        s.pop(); // 去掉尾部 '+'
    }
    s
}

/// Slint KeyEvent.text → HID 键码。纯修饰键/未识别返回 0。
pub(crate) fn char_to_hid(text: &str) -> u8 {
    match text {
        "\u{000a}" | "\r" => return 0x28,
        "\u{001b}" => return 0x29,
        "\u{0008}" => return 0x2A,
        "\u{0009}" => return 0x2B,
        " " => return 0x2C,
        "-" | "_" => return 0x2D,
        "=" | "+" => return 0x2E,
        "[" | "{" => return 0x2F,
        "]" | "}" => return 0x30,
        "\\" | "|" => return 0x31,
        ";" | ":" => return 0x33,
        "'" | "\"" => return 0x34,
        "`" | "~" | "·" => return 0x35,
        "," | "<" => return 0x36,
        "." | ">" => return 0x37,
        "/" | "?" => return 0x38,
        "\u{007f}" => return 0x4C,
        "\u{f700}" => return 0x52,
        "\u{f701}" => return 0x51,
        "\u{f702}" => return 0x50,
        "\u{f703}" => return 0x4F,
        "\u{f727}" => return 0x49,
        "\u{f729}" => return 0x4A,
        "\u{f72b}" => return 0x4D,
        "\u{f72c}" => return 0x4B,
        "\u{f72d}" => return 0x4E,
        "\u{f72f}" => return 0x47,
        "\u{f730}" => return 0x48,
        "\u{f731}" => return 0x46,
        "\u{f735}" => return 0x65,
        _ => {}
    }
    if let Some(ch) = text.chars().next() {
        let u = ch as u32;
        if (0xf704..=0xf70f).contains(&u) {
            return 0x3A + (u - 0xf704) as u8;
        }
        if (0xf710..=0xf71b).contains(&u) {
            return 0x68 + (u - 0xf710) as u8;
        }
        let lc = ch.to_ascii_lowercase();
        match lc {
            'a'..='z' => return 0x04 + (lc as u8 - b'a'),
            '1'..='9' => return 0x1E + (lc as u8 - b'1'),
            '0' => return 0x27,
            _ => {}
        }
    }
    0
}

/// 解析十进制或 0x 前缀的 HID Keyboard/Keypad usage。
pub(crate) fn parse_hid_usage(text: &str) -> Result<u8, String> {
    let value = text.trim();
    if value.is_empty() {
        return Err("请输入 HID 编码".to_string());
    }
    let parsed = if let Some(hex) = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
    {
        u16::from_str_radix(hex, 16)
    } else {
        value.parse::<u16>()
    }
    .map_err(|_| format!("HID 编码无效: {value}"))?;
    if !(1..=u8::MAX as u16).contains(&parsed) {
        return Err(format!("HID 编码超出范围 1..255: {value}"));
    }
    Ok(parsed as u8)
}
