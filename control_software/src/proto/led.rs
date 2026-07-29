//! mai2light 灯板协议编解码 (灯效域 0x50-0x5F)
//!
//! 与固件 `main_firmware` 侧实测契约一致:
//! - `LED_GET(0x50)`      请求空 payload,响应固定 96 字节快照。byte0/byte1 为复合状态字,
//!   在定长不变的前提下用高位承载灯链与预览链路诊断(见 `decode_led_get` 处的位表)。
//! - `LED_SET_REGION(0x51)` payload = `[unit, ch, start(u16 LE), count] × n`,整批原子。
//! - `LED_PREVIEW(0x52)`  payload = `[unit, r, g, b] × n`,`unit=0xFF` 表示全部;
//!   预览色覆盖协议色约 3s 后自动回落。
//!
//! 虚拟 LED 单元语义: 0..7 = 按键灯(经灯板协议缓冲后提交), 8/9/10 = Body/Ext/Side 白灯(直刷)。

/// 虚拟 LED 单元数量(固件固定 11)。
pub const LED_UNIT_COUNT: usize = 11;
/// 映射通道占位值: 该单元未映射到任何灯链。
pub const LED_CH_UNMAPPED: u8 = 0xFF;
/// LED_PREVIEW 的"全部单元"占位 unit。
pub const LED_PREVIEW_ALL: u8 = 0xFF;
/// LED_GET 响应固定长度。
const LED_GET_LEN: usize = 96;
/// 固件默认值为 115200；1200..=3_000_000 覆盖常用 UART 配置，同时滤掉链路损坏产生的随机 u32。
const LED_BAUD_MIN: u32 = 1_200;
const LED_BAUD_MAX: u32 = 3_000_000;
/// Slint `int` 为有符号 i32，超过该值的 u32 裸转换会显示为负数。
const LED_DISPLAY_INT_MAX: u32 = i32::MAX as u32;

/// 单个虚拟 LED 单元在 WS 灯链上的映射区段。`ch == LED_CH_UNMAPPED` 表示未映射。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedRegion {
    pub ch: u8,
    pub start: u16,
    pub count: u8,
}

impl Default for LedRegion {
    fn default() -> Self {
        Self { ch: LED_CH_UNMAPPED, start: 0, count: 0 }
    }
}

impl LedRegion {
    /// 是否占用灯链(未映射或 count=0 都不占灯珠,校验/下发时按"空"处理)。
    pub fn active(&self) -> bool {
        self.ch != LED_CH_UNMAPPED && self.count > 0
    }
}

/// LED_GET 响应快照。`status`: 0=停 1=就绪 2=运行（来自状态字 bit0..3）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedState {
    pub status: u8,
    /// 两路 WS2812 初始化就绪位（状态字 bit4/bit5）。
    pub chain_ready: [bool; 2],
    /// 初始化故障分档（状态字 bit6..7）：0=无，1=PIO，2=chain0，3=chain1。
    pub init_fault: u8,
    /// Mai2Light 应答使能（诊断字 bit0）。
    pub resp_enabled: bool,
    /// 预览色正在覆盖协议色（诊断字 bit1）。为 false 时颜色字段就是游戏协议色。
    pub preview_active: bool,
    /// 固件 LedMapService 已初始化（诊断字 bit2）。false = 预览色不会被刷到灯链。
    pub service_ready: bool,
    /// 固件灯效刷新至少执行过一次（诊断字 bit3）。false = 灯服务从未被主循环调用。
    pub refresh_seen: bool,
    /// 固件灯效刷新次数低 4 位（诊断字 bit4..7）。两次快照该值不变 = 灯服务已停摆。
    pub refresh_ticks: u8,
    pub unit_count: u8,
    /// 当前有效颜色(预览生效时即预览色),索引 = 单元号。
    pub colors: [[u8; 3]; LED_UNIT_COUNT],
    pub regions: [LedRegion; LED_UNIT_COUNT],
    /// 灯板 UART 波特率；未初始化或链路异常时回报可能不是真实配置。调用方显示前须用 `baud_valid()` 过滤。
    pub baud: u32,
    /// 校验通过帧计数；异常 u32 裸转 Slint i32 会翻为负数，且未初始化/链路异常时不代表真实协议状态。调用方须先用 `rx_frames_valid()`。
    pub rx_frames: u32,
    /// 校验和错误计数；异常 u32 裸转 Slint i32 会翻为负数，且未初始化/链路异常时不代表真实协议状态。调用方须先用 `sum_errors_valid()`。
    pub sum_errors: u32,
    /// 两路灯链实际灯珠数(固件运行态,供本地映射越界校验)。
    pub ws_count: [u16; 2],
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            status: 0,
            chain_ready: [false; 2],
            init_fault: 0,
            resp_enabled: false,
            preview_active: false,
            service_ready: false,
            refresh_seen: false,
            refresh_ticks: 0,
            unit_count: 0,
            colors: [[0u8; 3]; LED_UNIT_COUNT],
            regions: [LedRegion::default(); LED_UNIT_COUNT],
            baud: 0,
            rx_frames: 0,
            sum_errors: 0,
            ws_count: [0u16; 2],
        }
    }
}

impl LedState {
    /// 该回报是否处于工程支持的常用 UART 波特率范围。
    pub fn baud_valid(&self) -> bool {
        (LED_BAUD_MIN..=LED_BAUD_MAX).contains(&self.baud)
    }

    /// 该计数是否可无损传入 Slint `int`。
    pub fn rx_frames_valid(&self) -> bool {
        self.rx_frames <= LED_DISPLAY_INT_MAX
    }

    /// 该计数是否可无损传入 Slint `int`。
    pub fn sum_errors_valid(&self) -> bool {
        self.sum_errors <= LED_DISPLAY_INT_MAX
    }
}

/// 编码 LED_GET 请求载荷(空)。保留函数形式以与其余 encode_* 对称,调用点无需记住"空"。
pub fn encode_led_get() -> Vec<u8> {
    Vec::new()
}

/// 解码 LED_GET 响应载荷(96 字节)。长度不足一律返回 Err,不做部分解析、不 panic。
pub fn decode_led_get(payload: &[u8]) -> Result<LedState, String> {
    if payload.len() < LED_GET_LEN {
        return Err(format!(
            "LED_GET 响应过短: {} 字节 (需 >= {})", payload.len(), LED_GET_LEN));
    }
    let u16_at = |off: usize| u16::from_le_bytes([payload[off], payload[off + 1]]);
    let u32_at = |off: usize| {
        u32::from_le_bytes([payload[off], payload[off + 1], payload[off + 2], payload[off + 3]])
    };
    // byte0 是复合状态字(96B 定长不变, 高位复用): bit0..3 灯板状态机, bit4/5 两链就绪,
    // bit6..7 初始化故障分档。旧固件这些高位恒 0, 解出来就是"未就绪且无故障", 不会误报。
    // byte1 同为复合诊断字: bit0 应答使能(旧固件只有这一位), bit1 预览生效, bit2 灯服务已初始化,
    // bit3 刷新至少跑过一次, bit4..7 刷新次数低 4 位。旧固件高位恒 0 → 诊断项全 false, 不会误报。
    let mut state = LedState {
        status: payload[0] & 0x0F,
        chain_ready: [(payload[0] & 0x10) != 0, (payload[0] & 0x20) != 0],
        init_fault: (payload[0] >> 6) & 0x03,
        resp_enabled: (payload[1] & 0x01) != 0,
        preview_active: (payload[1] & 0x02) != 0,
        service_ready: (payload[1] & 0x04) != 0,
        refresh_seen: (payload[1] & 0x08) != 0,
        refresh_ticks: (payload[1] >> 4) & 0x0F,
        unit_count: payload[2],
        baud: u32_at(80),
        rx_frames: u32_at(84),
        sum_errors: u32_at(88),
        ws_count: [u16_at(92), u16_at(94)],
        ..LedState::default()
    };
    for unit in 0..LED_UNIT_COUNT {
        let c = 3 + unit * 3;
        state.colors[unit] = [payload[c], payload[c + 1], payload[c + 2]];
        let m = 36 + unit * 4;
        state.regions[unit] = LedRegion {
            ch: payload[m],
            start: u16::from_le_bytes([payload[m + 1], payload[m + 2]]),
            count: payload[m + 3],
        };
    }
    Ok(state)
}

/// 编码 LED_SET_REGION 载荷: 每项 5 字节 `[unit, ch, start(u16 LE), count]`。
/// 整批原子由固件保证(任一项越界/重叠即 NAK 且不部分生效),此处只做打包。
pub fn encode_led_set_region(items: &[(u8, LedRegion)]) -> Vec<u8> {
    let mut out = Vec::with_capacity(items.len() * 5);
    for (unit, region) in items {
        out.push(*unit);
        out.push(region.ch);
        out.extend_from_slice(&region.start.to_le_bytes());
        out.push(region.count);
    }
    out
}

/// 编码 LED_PREVIEW 载荷: 每项 4 字节 `[unit, r, g, b]`; `unit = LED_PREVIEW_ALL` 表示全部单元。
pub fn encode_led_preview(items: &[(u8, [u8; 3])]) -> Vec<u8> {
    let mut out = Vec::with_capacity(items.len() * 4);
    for (unit, rgb) in items {
        out.push(*unit);
        out.extend_from_slice(rgb);
    }
    out
}

/// 本地映射校验: 返回首个冲突的可读描述(None = 通过)。
///
/// 与固件同口径先在上位机拦一道,避免明知会 NAK 还发(整批原子失败对用户是"什么都没变",
/// 更难定位)。校验项: 通道号合法、区段不越界(以设备回报的 ws_count 为准)、同通道区段不重叠。
pub fn validate_led_regions(regions: &[LedRegion; LED_UNIT_COUNT], ws_count: [u16; 2]) -> Option<String> {
    for (unit, region) in regions.iter().enumerate() {
        if region.ch == LED_CH_UNMAPPED {
            continue;
        }
        if region.ch > 1 {
            return Some(format!("单元 {} 通道号 {} 非法(仅 0/1)", unit, region.ch));
        }
        if region.count == 0 {
            continue;
        }
        let limit = ws_count[region.ch as usize] as u32;
        let end = region.start as u32 + region.count as u32;
        // ws_count 为 0 说明尚未回读到设备真值, 此时不做越界判断(否则会误报全部冲突)。
        if limit > 0 && end > limit {
            return Some(format!(
                "单元 {} 越界: ch{} {}..{} 超出灯链长度 {}", unit, region.ch, region.start, end, limit));
        }
    }
    for a in 0..LED_UNIT_COUNT {
        for b in (a + 1)..LED_UNIT_COUNT {
            let (x, y) = (regions[a], regions[b]);
            if !x.active() || !y.active() || x.ch != y.ch {
                continue;
            }
            let (xs, ys) = (x.start as u32, y.start as u32);
            let (xe, ye) = (xs + x.count as u32, ys + y.count as u32);
            if xs < ye && ys < xe {
                return Some(format!("单元 {} 与单元 {} 在 ch{} 上区段重叠", a, b, x.ch));
            }
        }
    }
    None
}
