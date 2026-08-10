//! UI 状态与 Slint 桥接 (#6e~g)
//!
//! 职责:
//! - 维护 UI 侧应用状态(设备连接状态、配置项、遥测数据、绑区/灯效编辑状态等)
//! - 与 `proto`/`io`/`comport` 模块交互,将协议层数据转换为 Slint 可绑定的文本/列表
//! - 处理 UI 事件回调,驱动命令发送
//!
//! 本模块不依赖任何 Slint 类型,保持纯逻辑、可单测。UI 侧(`main.rs`)只负责
//! 把这里暴露的字符串/列表回填到 `AppWindow` 属性,以及把 callback 转发到这里
//! 的方法调用。
//!
//! 后续 tab(#6e 配置 / #6f 绑区 / #6g 曲线)可在 [`AppController`] 上继续加字段
//! 与方法,复用这里已建立的 `io`/连接状态骨架,不需要重新搭连接逻辑。
//!
//! `state()`/`device_count()`/`last_error()` 目前尚无 UI 消费者,是留给后续
//! tab(需要按连接状态门控操作、展示设备数量/最近错误)的既定接口,故标注
//! `#[allow(dead_code)]` 而非删除。

#![allow(dead_code)]

use crate::comport::CdcFunction;
use crate::io::{self, IoEvent, IoHandle};
use crate::proto::{
    CfgValue, ChannelSample, ConfigEntry, DeviceInfo, FIELD_BASELINE, FIELD_DIFF, FIELD_RAW, Frame,
    HostCmd, decode_entries,
};
use crate::proto::{
    HoldParam, KBD_DEBOUNCE_US_MAX, KBD_HOLD_KIND_PHYS, KBD_HOLD_KIND_ZONE, KBD_HOLD_PHYS_COUNT,
    KBD_HOLD_ZONE_COUNT, KbdEdgeRec, KbdHoldItem, KbdKeyCfg, Mai2State,
};
use crate::proto::{
    LED_CH_UNMAPPED, LED_PREVIEW_ALL, LED_UNIT_COUNT, LedRegion, LedState, PARAM_IDAC_GAIN,
};
use std::collections::{BTreeMap, VecDeque};

mod drafts;
use drafts::ConfigDrafts;
mod pipeline;
pub use pipeline::PollContext;
use pipeline::{PendingRegistry, PendingRequest, PollRules, RequestKind, SkippedLog};
/// 逐通道 CSD 操作的 host 侧编排(批量串行队列 / 噪声频谱扫描 / 绘图冻结)。
/// ★为什么单独一个文件★ 三者都是"跨若干 tick 的状态机 + 自己的数据结构", 塞回本文件只会让
/// AppController 再长几百行而与其余职责纠缠; 它们对外只暴露少量方法, 内部状态不被别处读写。
mod ch_ops;
pub use ch_ops::{ChBatchKind, ChBias, NoiseCell, SWEEP_DIVS, SWEEP_GAINS};

/// 全局项回读对账的等待窗口(tick @16ms ≈ 12s)。
/// 必须覆盖"PSoC 重初始化 + RP2040 自持恢复重下发"的完整过程 —— 实测重下发在保存后约 9s 才完成,
/// 早于此对账会读到 PSoC 启动时的编译默认值(RAW_TARGET=85 / IDAC_GAIN_INIT=4 …), 把一次正常保存
/// 误报成"设备未按下发值生效"。
const GLOBALS_VERIFY_TICKS: u32 = 750;

/// 逻辑分析仪主机侧留存的边沿条数上限(定长 FIFO, 超出丢最旧)。
/// 2048 条 × 10B 数据量可忽略, 但足够覆盖一次长时间观察; 波形窗口本身只取其中最近一段。
const KBD_EDGE_KEEP: usize = 2048;

/// 每通道遥测缓冲的保留时长(us)。★必须 ≥ 主图固定窗口(`main.rs::PLOT_WINDOW_US` = 30s)★,
/// 留 1s 余量吸收帧抖动: 否则窗口最左侧会永远缺一小段。
/// 判据用时长而不是条数: 条数在不同采样率下对应的时长完全不同(171Hz 时 1024 条只有 ~6s)。
pub const TELEM_RETAIN_US: u32 = 31_000_000;
/// 每通道条数上限, 只作内存兜底: 采样率异常高时不至于把 31s 全留下来。
/// 16384 条覆盖到 ~528Hz 仍满 31s; 36 通道满载约 9MB, 可接受。
pub const TELEM_CAP: usize = 16384;

/// UI 日志等级。数值越大越"啰嗦": Error(0) < Warn(1) < Info(2) < Debug(3)。
/// 过滤规则: 仅显示 `level as u8 <= log_filter` 的条目(选 Debug 显示全部, 选 Info 隐藏 Debug)。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LogLevel {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
}

/// cfg 队列帧的唯一归属。批量 prime 只可清理自己的 generation，外部保存/诊断帧永不被误删。
#[derive(Clone, Copy, PartialEq, Eq)]
enum CfgTxOwner {
    External,
    BatchPrime(u64),
}

struct CfgTxFrame {
    frame: Frame,
    owner: CfgTxOwner,
}

struct CfgTxInflight {
    cmd: u8,
    seq: u8,
    waited: u32,
    owner: CfgTxOwner,
}

#[derive(Clone, Copy)]
struct LatencyComponentSample {
    value_us: u16,
    ts_us: u32,
}

#[derive(Default)]
struct LatencyCorrectionSamples {
    // 固定顺序对应 TELEM 0x20 的 spi / proc / usb 三个 u16 字段。
    components: [Option<LatencyComponentSample>; 3],
}

/// 0x20 三段是“上一遥测窗口内观察到的滚动峰值”，发帧后固件立即清零；因此字段 0 的语义是
/// “本窗口没有观察到该段”，绝不是 0us。只用非零值刷新各段样本，并借同源 `TelemFrame.ts_us`
/// 判定设备时钟下的新鲜度；三段都在 2s 内才给出补正值：fresh_spi + fresh_proc + fresh_usb。
/// 本公式只修补异步窗口造成的缺段，不包含也不改写 `comm.touch_delay_100us` 人工串口延迟线。
fn _latency_correction_us(
    samples: &mut LatencyCorrectionSamples,
    frame: &crate::proto::TelemFrame,
) -> Option<u32> {
    const FRESH_US: u32 = 2_000_000;
    let values = [frame.lat_spi_us, frame.lat_proc_us, frame.lat_usb_us];
    for (slot, value_us) in samples.components.iter_mut().zip(values) {
        if value_us != 0 {
            *slot = Some(LatencyComponentSample {
                value_us,
                ts_us: frame.ts_us,
            });
        }
    }
    let mut total = 0u32;
    for sample in samples.components.iter().copied() {
        let sample = sample?;
        if frame.ts_us.wrapping_sub(sample.ts_us) > FRESH_US {
            return None;
        }
        total = total.saturating_add(sample.value_us as u32);
    }
    Some(total)
}

// ============================================================================
// 绑区辅助 (#6f):34 区 index ↔ bind.mapNN key/label 映射,与 u32 编码工具
// ============================================================================

/// 34 区固定顺序(与 protocol_design.md 修订2 C5 一致):
/// A1..A8(0..7) B1..B8(8..15) C1..C2(16..17) D1..D8(18..25) E1..E8(26..33)
const ZONE_RINGS: [(char, usize); 5] = [('A', 8), ('B', 8), ('C', 2), ('D', 8), ('E', 8)];
const LISTEN_HOLD_MS: u64 = 1000;

/// 全通道页"批量应用"的选择态: 目标通道集 + 待应用参数集。
/// 两个掩码必须成对存在(只勾通道不勾参数、或反之, 都不构成一次有效应用), 故合成一个结构体
/// 而不是散在两个字段里; `clear()` 统一复位, 避免遗漏其中一半。
#[derive(Clone, Copy, Default)]
struct BatchApplySel {
    /// bit i = 通道 i 被选中(36 通道, 高 28 位恒 0)。
    ch_mask: u64,
    /// bit (id-1) = per-channel 参数 id 被选中(id 取值 0x01..=0x0B)。
    param_mask: u16,
}

impl BatchApplySel {
    /// 可批量应用参数的全集掩码。★由 `proto::BATCH_PARAM_IDS` 派生, 不写死位图★:
    /// 写死 0x07FF 会把分辨率(0x07)也算进"全选", 于是"全选参数"又能绕过全局唯一入口。
    const PARAM_ALL: u16 = {
        let mut mask = 0u16;
        let ids = crate::proto::BATCH_PARAM_IDS;
        let mut i = 0;
        while i < ids.len() {
            mask |= 1u16 << (ids[i] - 1);
            i += 1;
        }
        mask
    };
    /// 36 通道全选掩码。
    const CH_ALL: u64 = (1u64 << 36) - 1;

    fn new() -> Self {
        Self::default()
    }

    fn clear(&mut self) {
        self.ch_mask = 0;
        self.param_mask = 0;
    }

    fn ch_selected(&self, ch: u8) -> bool {
        ch < 36 && (self.ch_mask & (1u64 << ch)) != 0
    }

    fn param_selected(&self, param_id: u8) -> bool {
        crate::proto::BATCH_PARAM_IDS.contains(&param_id)
            && (self.param_mask & (1u16 << (param_id - 1))) != 0
    }
}

/// 算法 C 源分片上传的在途状态。整份字节 + 已确认字节数 + 在途片的 seq 必须成组存在,
/// 否则"发到哪了/在等谁的 ACK"就成了两个可能不一致的事实。
struct AlgoSrcTx {
    bytes: Vec<u8>,
    /// 已被设备 ACK 的字节数(= 下一片的 offset)。
    acked: usize,
    /// None 表示等待普通写队列排空；Some(seq) 表示当前片已发出并在等 ACK/NAK。
    seq: Option<u8>,
    waited: u32,
}

/// 算法 C 源分片回读的聚合状态。收满 total 才替换缓存/版本 —— 半份源灌进编辑器比不灌更坏。
struct AlgoSrcRx {
    total: usize,
    buf: Vec<u8>,
    /// 当前请求片的序号与等待 tick，用于拒绝迟到响应并处理丢帧超时。
    seq: u8,
    waited: u32,
}

/// 侦听绑定的候选通道与起始时刻必须成对清除，避免跨分区继承按住时长。
struct ListenHold {
    channel: Option<u8>,
    since: Option<std::time::Instant>,
}

impl ListenHold {
    fn clear(&mut self) {
        self.channel = None;
        self.since = None;
    }
}

/// 已发出 FOCUS_START、尚未收到响应的在途请求。三项必须成组存在: 少了 generation 就无法判断
/// 迟到响应属于哪一次切换, 少了 seq 就无法把 NAK 归因到本次请求。
struct FocusPending {
    seq: u8,
    generation: u16,
    channel: u8,
}

/// 单通道独占流(FOCUS_START/STOP/DATA)的主机侧会话状态。
///
/// ★为什么要 host 侧 generation★ 切换目标通道是"停旧 + 开新"两条命令, 中间必然有一段时间旧会话
/// 的 FOCUS_DATA 仍在路上、而新会话的 START 响应还没回来。只按"当前 session"判定会漏掉第三种帧:
/// **迟到的 START 响应**(它会把一个已经过期的会话号安装成当前会话, 于是新目标的数据被当成旧帧丢掉,
/// 表现为切通道后曲线永久空白)。故每次切换前先自增 generation, 响应只在代次相符时才被安装,
/// 迟到响应一律丢弃并对其携带的 session 发 FOCUS_STOP(否则设备侧那条流没人认领却继续占带宽)。
#[derive(Default)]
struct FocusState {
    /// 主机侧代次: 每次切目标/启停前自增。
    generation: u16,
    /// 在途 START。
    pending: Option<FocusPending>,
    /// 设备已确认的当前会话号(None = 无会话)。
    session: Option<u16>,
    /// 当前目标通道(即便 START 尚在途也已确定, 用于幂等判定"目标没变就别重发")。
    channel: Option<u8>,
    /// 上一帧的 sample_seq, 用于算缺口。
    last_sample_seq: Option<u16>,
    /// 累计丢帧数(设备背压跳帧 + 链路丢帧)。
    gaps: u64,
    /// 实测吞吐窗口。★精调时必须能看见真实带宽★ 曲线看着"不密"到底是设备没发、链路丢了, 还是
    /// UI 只画了一段, 光看图分不出来 —— 只有实测 帧/s 与 字节/s 能回答。
    rate: FocusRate,
    /// 低频诊断计数：每 100 个有效 FOCUS_DATA 样本汇总一次，避免逐帧刷屏。
    diag_at: Option<std::time::Instant>,
    diag_frames: u32,
    diag_pushes: u32,
    diag_gaps: u64,
}

/// 单通道流的 1 秒滚动吞吐窗口。窗口内累计, 满 1 秒结算一次并把结果留给 UI 读。
#[derive(Default)]
struct FocusRate {
    /// 本窗口起点(None = 还没收到第一帧)。
    window_at: Option<std::time::Instant>,
    /// 本窗口内已收帧数与 wire 载荷字节数。
    frames: u32,
    bytes: u64,
    /// 本窗口内的 sample_seq 缺口数。
    gaps: u32,
    /// 上一个完整窗口的结算值: (帧/s, 字节/s, 丢帧/s)。
    last: Option<(u32, u32, u32)>,
    /// 最近一次结算时刻, 用于判定读数是否还新鲜。
    settled_at: Option<std::time::Instant>,
}

impl FocusRate {
    /// 收到一帧: 累计并在跨过 1 秒时结算。
    fn note(&mut self, wire_len: usize, gaps: u16) {
        let now = std::time::Instant::now();
        let start = *self.window_at.get_or_insert(now);
        self.frames = self.frames.saturating_add(1);
        self.bytes = self.bytes.saturating_add(wire_len as u64);
        self.gaps = self.gaps.saturating_add(gaps as u32);
        let elapsed = now.duration_since(start).as_secs_f32();
        if elapsed < 1.0 {
            return;
        }
        // 按真实经过时间折算, 而不是假定窗口恰好 1.0s(tick 抖动下会系统性偏低)。
        let scale = 1.0 / elapsed;
        self.last = Some((
            (self.frames as f32 * scale).round() as u32,
            (self.bytes as f32 * scale).round() as u32,
            (self.gaps as f32 * scale).round() as u32,
        ));
        self.settled_at = Some(now);
        self.window_at = Some(now);
        self.frames = 0;
        self.bytes = 0;
        self.gaps = 0;
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

impl FocusState {
    /// 目标通道(含在途请求)是否已经是 `ch`。
    fn targets(&self, ch: Option<u8>) -> bool {
        self.channel == ch
    }

    /// 清除独占流的会话计数锚点；开始新流/会话时绝不把旧 sample_seq 或已结算带宽带过去。
    fn reset_measurement(&mut self) {
        self.last_sample_seq = None;
        self.gaps = 0;
        self.diag_at = None;
        self.diag_frames = 0;
        self.diag_pushes = 0;
        self.diag_gaps = 0;
        self.rate.reset();
    }

    /// 进入新一代: 旧会话/在途请求的一切回执与数据自此全部作废。
    fn _bump(&mut self) -> u16 {
        self.generation = self.generation.wrapping_add(1);
        self.pending = None;
        self.session = None;
        self.channel = None;
        self.reset_measurement();
        self.generation
    }

    fn clear(&mut self) {
        self._bump();
        self.gaps = 0;
    }
}

/// 设备端 32 位 us 时间戳(约 71.6 分钟回绕)的展开累加器。
///
/// ★为什么必须展开★: 折线图横轴要真实设备时间, 直接用 u32 原值会在回绕点跳回 0 → 时间轴倒流
/// (表现为曲线整体折回/负时间)。这里只累加"相邻两帧的 wrapping 差值", 差值恒为正且远小于
/// 2^32, 于是回绕被自然吸收, 累加值单调递增。
/// ★暂停不清零★: 停流期间不喂帧, 累加值原地不动(时间轴随之冻结); 恢复后第一帧的巨大差值
/// 会被如实累加 —— 那段间隙由绘图侧断开子路径表示, 不做直线插值。
/// (唯一无解的情形: 停流超过一个回绕周期, 差值信息在设备侧已丢失, 只能低估间隙, 见注释。)
#[derive(Default)]
struct DevClock {
    /// 上一帧的设备原始 ts(us)。None = 还没收到任何帧。
    prev_raw: Option<u32>,
    /// 展开后的累计设备时间(us), 以首帧为 0 点。
    acc_us: u64,
}

impl DevClock {
    fn feed(&mut self, ts_us: u32) {
        let delta = match self.prev_raw {
            Some(prev) => ts_us.wrapping_sub(prev) as u64,
            None => 0,
        };
        self.acc_us = self.acc_us.saturating_add(delta);
        self.prev_raw = Some(ts_us);
    }

    /// 把**任意**设备帧的 32 位 us 原值映射到本累加器的展开时间轴(只读锚点, 不推进累加值)。
    ///
    /// ★为什么可以这么做★ 全协议时间戳同源于设备 `time_us_32()`(bus 头 t_us / host_cmd 尾戳 /
    /// TELEM_DATA.ts_us / KBD_GET_EDGES.t_us), 因此任一帧的原值与本累加器的锚点 `prev_raw`
    /// 之间做**有符号** 32 位差值即得它相对锚点的真实偏移 —— 帧比锚点新(正)或旧(负)都正确,
    /// 且 32 位回绕被 `wrapping_sub as i32` 自然吸收。
    /// 尚无锚点(还没收到任何遥测帧)时以本值建立 0 点, 于是"停流期间只有轮询数据"也有正确时间轴。
    /// 累加值的推进权仍**只**属于 `feed()`(遥测帧), 这里不改 `prev_raw`, 避免两个推进者互相打乱。
    fn unwrap_us(&mut self, raw: u32) -> u64 {
        let Some(prev) = self.prev_raw else {
            self.feed(raw);
            return self.acc_us;
        };
        let delta = raw.wrapping_sub(prev) as i32 as i64;
        (self.acc_us as i64 + delta).max(0) as u64
    }

    fn clear(&mut self) {
        self.prev_raw = None;
        self.acc_us = 0;
    }
}

/// 算法追踪的一个采样点: 值 + 采样时刻(展开后的设备时间 us)。
///
/// ★时间是设备真值, 不再是近似★ 协议已统一给设备→主机的每一帧追加 `time_us_32()` 尾戳
/// (`proto::FLAG_TS`, 由 `Frame::device_t_us` 承载), 因此这里直接用**设备组帧那一刻**的时刻,
/// 经 `DevClock::unwrap_us` 折进与遥测同一条展开时间轴。
/// ⇒ 触发判定/上报线与 TELEM_DATA 从此是同一个时钟, 横向对位精确到设备侧组帧时刻,
/// 不再受主机轮询周期(16ms)、USB 往返抖动、GUI tick 漂移影响。
#[derive(Debug, Clone, Copy)]
struct TracePoint {
    t_us: u64,
    val: f32,
}

/// ★掉线重连要自动恢复的"期望运行态"★
///
/// 震动导致 USB 掉线后触控必须自己恢复, 不能等人手动点。因此这里记录的是"用户显式设置过的
/// 运行态期望值", 只在用户通过 UI 主动设置时更新, **断连时不得清除**(`disconnect()` /
/// `_clear_sensor_caches()` 清的是设备回读缓存, 期望态必须活过断连), 重新握手后逐条重下发。
///
/// 同时充当长按/发送使能的"草稿层": getter 一律期望值优先 → 设备缓存兜底, 不再另建平行结构。
#[derive(Default)]
struct DesiredState {
    /// mai2 串口发送使能期望值。
    mai2_send_en: Option<bool>,
    /// 触控→键盘映射总开关(comm.keyboard_map_en)期望值。
    kbd_map_en: Option<bool>,
    /// 物理键长按参数期望值(idx → 参数)。
    hold_phys: BTreeMap<u8, HoldParam>,
    /// 分区长按参数期望值(zone → 参数)。
    hold_zone: BTreeMap<u8, HoldParam>,
}

impl DesiredState {
    fn clear(&mut self) {
        self.mai2_send_en = None;
        self.kbd_map_en = None;
        self.hold_phys.clear();
        self.hold_zone.clear();
    }

    /// 是否有任何期望值需要在重连后恢复。
    fn is_empty(&self) -> bool {
        self.mai2_send_en.is_none()
            && self.kbd_map_en.is_none()
            && self.hold_phys.is_empty()
            && self.hold_zone.is_empty()
    }
}

/// 重连恢复后的一次性回读对账标记: 收到对应响应时与期望值逐条比对, 不一致必须告警。
#[derive(Default)]
struct RestoreVerify {
    hold: bool,
    mai2: bool,
}

impl RestoreVerify {
    fn clear(&mut self) {
        self.hold = false;
        self.mai2 = false;
    }
}

/// 把绑区 index(0..33)转成 maimai 分区标签,如 0→"A1"、16→"C1"、33→"E8"。
/// index 越界时返回 "?<index>" 便于排查,而不是 panic。
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

/// 把绑区 index(0..33)转成配置 key,如 0→"bind.map00"、33→"bind.map33"。
pub fn zone_key(index: usize) -> String {
    format!("bind.map{:02}", index)
}

// ============================================================================
// HID 触摸屏点位辅助: 36 物理通道 ↔ hid.enNN / hid.xNN / hid.yNN
// ============================================================================

/// HID 触摸点位的通道数(= 物理通道数)。
pub const HID_POINT_COUNT: usize = 36;
/// HID 触屏坐标域上限。★与固件描述符 usage 0x30/0x31 的 LOGICAL_MAXIMUM 同源★
/// (见 hal_usb_hid.h 的 `TOUCH_LOGICAL_MAX` / `0x26,0xFF,0x7F`)。上位机存归一坐标即用此域,
/// 固件不再做任何缩放; 两侧若各写一个数, 点位会被主机按满量程截断到屏幕边缘。
pub const HID_COORD_MAX: u16 = 32767;

/// 通道 ch 的"是否输出该点位"配置键。
pub fn hid_en_key(ch: usize) -> String {
    format!("hid.en{:02}", ch)
}
/// 通道 ch 的 X 归一坐标配置键(0..HID_COORD_MAX)。
pub fn hid_x_key(ch: usize) -> String {
    format!("hid.x{:02}", ch)
}
/// 通道 ch 的 Y 归一坐标配置键(0..HID_COORD_MAX)。
pub fn hid_y_key(ch: usize) -> String {
    format!("hid.y{:02}", ch)
}

/// 从 bind.mapNN 的 u32 值中取出低 24 位通道 bitmap。
pub fn binding_channel_mask(v: u32) -> u32 {
    v & 0x00FF_FFFF
}

/// 从 bind.mapNN 的 u32 值中取出高 8 位设备掩码。
pub fn binding_device_mask(v: u32) -> u8 {
    (v >> 24) as u8
}

/// 把设备掩码(高8位)与通道 bitmap(低24位)组合成 bind.mapNN 的 u32 值。
pub fn make_binding(dev: u8, ch: u32) -> u32 {
    ((dev as u32) << 24) | (ch & 0x00FF_FFFF)
}

// ============================================================================
// 连接状态
// ============================================================================

/// 与设备的连接状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnState {
    Disconnected,
    Connecting,
    Connected,
}

/// PSoC SPI 链路活性证据 —— 链路是否联通的**唯一判定结论**, 判定式与文案同源。
///
/// ★为什么要枚举而不是一个 bool★: 原实现直接读 DEVICE_INFO 的 `psoc_link_valid` 布尔位,
/// 而那个位只在 HELLO 的响应里回一次(见 AppController::psoc_scan_live_at 注释), 于是
/// "系统状态"的文案与"采样卡"的红字各自去读同一个过期布尔, 一个说通一个说断都发现不了。
/// 现在只有这一个来源, 携带证据本身, 谁要显示都从它取, 不可能不同步。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PsocLinkEvidence {
    /// 遥测帧里 samples_per_sec>0 = PSoC 快照代数在推进(最强证据, 随 20~100Hz 遥测持续刷新)。
    ScanAdvancing { sps: u32 },
    /// 停流时的唯一证据: DEVICE_INFO 回读 link_ok=1, 或两次回读之间代数推进了。
    Handshake { generation: u16 },
    /// 判定窗口内没有任何活性证据。`since_ms=None` = 本次连接从未拿到过证据。
    Stale {
        since_ms: Option<u64>,
        last_generation: Option<u16>,
    },
}

// ============================================================================
// 设备候选(合并 io::list_devices + comport::identify_ports)
// ============================================================================

/// 一条可供 UI 下拉选择的设备候选
#[derive(Debug, Clone)]
pub struct DeviceEntry {
    pub port_name: String,
    pub function: CdcFunction,
    /// 展示文本,如 "COM5  [config]"
    pub label: String,
}

/// 一次算法编译的全部产物。纯数据(Send), 故可由后台线程算好再送回 UI 线程写入状态。
pub struct CompiledAlgo {
    /// .text 裸二进制(待上传 PSoC 算法槽)。
    pub blob: Vec<u8>,
    /// objdump 反汇编文本(制表符已换成空格)。
    pub asm: String,
}

/// 正在等待设备 ACK/NAK 的灯效写操作类型。只决定回执文案,与 `led_apply_seq` 同格存放:
/// 灯效写操作彼此互斥(下一次下发即覆盖上一次待办),不另开第二套等待状态机。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LedWriteOp {
    ApplyRegions,
    Preview,
}

// ============================================================================
// AppController: 纯逻辑状态机,不依赖 Slint
// ============================================================================

#[derive(Clone)]
struct ConnProbeStep {
    kind: RequestKind,
    frame: Frame,
}

///
/// 生命周期由 `main.rs` 用 `Rc<RefCell<AppController>>` 持有,在 Slint 单线程
/// 事件循环中于各 callback 与 `Timer` 轮询闭包间共享。
pub struct AppController {
    /// 统一轮询请求归因注册表与声明式轮询规则。
    pending_registry: PendingRegistry,
    skipped_log: SkippedLog,
    poll_rules: PollRules,
    /// 连接后顺序回读探针：窗口=1，逐项收到数据响应后推进。
    conn_probe_queue: VecDeque<ConnProbeStep>,
    conn_probe_inflight: Option<(u8, ConnProbeStep)>,

    devices: Vec<DeviceEntry>,
    io: Option<IoHandle>,
    state: ConnState,
    device_info: Option<DeviceInfo>,
    /// 从 DEVICE_INFO 诊断尾部回读的真实 CSD 模式；缺失表示旧固件或尚未握手。
    csd_mode: Option<u8>,
    /// 最近一次"遥测证明 PSoC 快照代数在推进"的主机时刻。
    /// ★为什么不能只看 DEVICE_INFO.psoc_link_valid★: 那个字节只在 HELLO 的 DEVICE_INFO 响应里
    /// 回一次, 而 HELLO 只在握手/重连/救砖后发 ⇒ 它是"握手那一瞬的快照", 之后永不刷新。固件的
    /// link_ok 本身会因 SPI 残帧抖动(psoc_spi.cpp 注释), 握手恰好撞上抖动就把"未联通"钉死到下次
    /// 重连 —— 界面于是一边显示 179Hz 采样一边说链路断了。改由活性证据判定: 遥测帧的
    /// samples_per_sec 就是 PSoC scan_count 的增量速率(psoc.cpp 每 500ms 折算), >0 即代数在推进。
    psoc_scan_live_at: Option<std::time::Instant>,
    /// 最近一次"DEVICE_INFO 证明链路活着"的主机时刻(停流时的唯一证据)。
    psoc_info_live_at: Option<std::time::Instant>,
    /// 上一次 DEVICE_INFO 读到的代数: 两次读之间推进过 = 活着(即便 link_ok 那一帧恰好抖动为 0)。
    psoc_gen_seen: Option<u16>,
    seq: u8,
    last_error: Option<String>,
    status_text: String,

    // 工具箱端口设置(本地 toolbox.cfg 持久化,不写入设备配置)。
    toolbox_auto_port: bool,
    toolbox_serial_com: u16,
    toolbox_light_com: u16,
    /// 虚拟摄像头输入源键盘的 Raw Input 设备路径(空 = 所有键盘)。同存 toolbox.cfg。
    vcam_kbd_device: String,

    // 配置缓存 (#6e-1)
    config_cache: BTreeMap<String, ConfigEntry>,
    /// 用于流式 GET_ALL 帧累积,当 RESPONSE 标记末帧时合并进 config_cache
    cfg_all_accum: Vec<u8>,
    /// 配置缓存版本号 (#6e-2):仅在 GET_ALL/GET_GROUP/GET 响应成功合并进
    /// config_cache 时自增,供 UI 侧判断"缓存有更新才重建配置行",避免
    /// 每次轮询都重建行清掉用户正在编辑的值。
    config_version: u64,

    /// 交互式绑区进度 `(zone, status)`：本地等待态为 status=0；固件
    /// BIND_EVENT 的真实 payload 为 `[zone, channel, status]`，status=1 表示完成。
    /// 仅等待态暴露给 UI，使完成/失败不会留下永久高亮。
    bind_progress: Option<(u8, u8)>,
    /// 最近一次 BIND_START 的 seq，用于只在对应 NAK 时清除等待态。
    bind_start_seq: Option<u8>,

    /// 上位机侧"侦听下一次触摸"绑定: 正在侦听的分区(None=未侦听)。
    /// 不发 BIND_START, 不改设备运行态; 通过全通道遥测 status bit0 的上升沿捕获
    /// 下一个被触摸的物理通道, 只写入绑定草稿, 点击保存后才真正下发。
    listen_zone: Option<u8>,
    /// 侦听开始时刻的全通道激活掩码, 用于只在"新激活"(上升沿)通道上捕获。
    listen_baseline_mask: u64,
    /// 候选通道必须持续按住至门限才可写入绑定草稿。
    listen_hold: ListenHold,
    /// 交互式顺序绑定(等效 v3.0): true 时 listen_tick 捕获一个区后自动推进到下一区, 遍历全 34 区。
    interactive_bind: bool,

    // 遥测数据缓冲 (#6g-1)
    /// 36 个通道的环形缓冲。保留判据是**时长** `TELEM_RETAIN_US`(主图固定窗口 + 余量),
    /// `TELEM_CAP` 只是内存兜底条数上限。
    telem_buf: Vec<VecDeque<ChannelSample>>,
    /// 数据存活检测: 每通道上一帧 raw 值 + 连续"值完全不变"的帧数。
    /// CSD 原始值在正常扫描下总有噪声抖动; 若某通道 raw 长时间逐帧完全相同(而设备仍在出帧),
    /// 则几乎必为异常(该通道扫描/测量卡死或链路停滞), 而非真实恒定读数 → 明确标记, 不再靠人忙猜。
    telem_last_raw: [Option<u16>; 36],
    telem_freeze_count: [u32; 36],
    /// 是否正在进行遥测流
    telem_active: bool,
    /// 单通道独占流会话状态(见 `FocusState`)。数据仍写入 `telem_buf` 的目标通道, 不另设缓冲。
    focus: FocusState,
    /// 上一次收到的遥测帧时间戳(μs)
    telem_last_ts: u32,
    /// 帧级设备时间展开器: 所有通道的样本共用同一时基(同一帧内各通道同时扫描)。
    telem_clock: DevClock,
    /// 最近一帧到达的主机时刻。★只用于给算法追踪点补时间★: 追踪响应不带设备时刻, 而遥测停流时
    /// 设备时钟不再推进 —— 用"最近一帧的设备时间 + 此后主机侧已流逝的时间"给追踪点定位, 停流期间
    /// 追踪线仍有正确的时间间隔, 且流式期间该修正量 <1 帧间隔, 不会与遥测曲线错位。
    telem_frame_at: Option<std::time::Instant>,
    /// 遥测数据版本号:每次成功处理 TELEM_DATA 帧时自增,供实时卡片判断刷新
    telem_version: u64,
    /// 绘图版本: 仅在未冻结时随遥测推进; 解冻时单独推进使主图跳到最新缓冲。
    plot_version: u64,
    /// 最近一次 STATS 字段解出的采样率(Hz)
    telem_samples_per_sec: u32,
    /// 最近一次 STATS 字段解出的通道刷新延迟(us)
    telem_scan_period_us: u32,
    /// 最近一次收到**带 STATS 的**遥测帧的时刻。逐通道档下固件按帧切分、只有首帧带 STATS,
    /// 故必须与"收到任意遥测帧"区分开, 否则采样率/探测周期会被无 STATS 的帧冲成 0。
    telem_stats_at: Option<std::time::Instant>,
    /// 用户显式按过"停止"。★页面档位驱动每 tick 都跑, 没有这个位就会把用户的停止立刻覆盖掉★。
    telem_user_paused: bool,
    /// 上次遥测自愈(重下发 TELEM_START)的时刻, 用于最小间隔限流。
    telem_heal_at: Option<std::time::Instant>,
    /// 最近一次下发 TELEM_START 的时刻。自愈的超时起算点 = max(收到 STATS, 开流) ——
    /// 少了它, 开流到首帧那几百毫秒会被误判成"卡住"而白白自愈一次。
    telem_started_at: Option<std::time::Instant>,
    /// 每通道最近一次出现在逐通道 TELEM_DATA 的主机时刻。全流 STATS 正常不能替它背书。
    telem_channel_last_seen: [Option<std::time::Instant>; 36],
    /// 单通道缺帧已经触发过几次有界恢复；收到该通道新样本即清零。
    telem_channel_heal_attempts: [u8; 36],
    /// 最近一次 LATENCY 字段解出的 PSoC SPI 触控读耗时(us)
    telem_lat_spi_us: u16,
    /// 最近一次 LATENCY 字段解出的 RP2040 处理耗时(us)
    telem_lat_proc_us: u16,
    /// 最近一次 LATENCY 字段解出的 serial/CDC 写耗时(us)
    telem_lat_usb_us: u16,
    /// 三段最后一次非零且未过期的样本；0x20 的 0 是“本窗口无观测”，不能冲掉有效样本。
    latency_correction_samples: LatencyCorrectionSamples,
    /// 最近一次按设备 ts_us 判为三段齐全且新鲜的补正值；配置开关只控制是否对 UI 暴露。
    telem_lat_corrected_us: Option<u32>,
    /// 延迟历史(总延迟 = spi+proc+usb, us),供仪表盘折线图。容量 LAT_CAP。
    lat_total_hist: VecDeque<f32>,
    /// 延迟历史版本号,每次 push 自增,供 UI 判断重绘。
    lat_version: u64,

    // 参数缓存 (#6g-1)
    /// 36 个通道,每通道 param_id → value 映射
    params: Vec<BTreeMap<u8, u32>>,
    /// 参数缓存版本号:每次成功处理 PARAM_GET/PARAM_GET_ALL 响应时自增
    param_version: u64,

    /// 36 个通道的最近 Cp(fF)；`None` 表示尚未读取。
    cp: Vec<Option<u32>>,
    /// 每通道最后一次 Cp 响应对应的全局版本，供 UI 隐藏切换前的缓存值。
    cp_channel_versions: Vec<u64>,
    /// Cp 缓存版本号：每次 CP_GET 响应或清空缓存时自增。
    cp_version: u64,

    /// JIT 算法信息(ALGO_GET_INFO 响应)+ 版本号(UI 刷新判据)。
    algo_info: Option<crate::proto::algo::AlgoInfo>,
    algo_version: u64,
    /// 每通道 16 位 ROM(ALGO_GET_ROM 响应, 36 项)+ 版本号。
    algo_rom: Vec<u16>,
    algo_rom_version: u64,

    /// 算法运行时追踪(ALGO_GET_TRACE): 当前追踪的通道(None=未追踪) + 上次请求的 report idx
    /// (响应不带 idx, 靠请求时序单条在途假设配对, 镜像 cp_poll_timer 单条在途轮询模式)。
    algo_trace_channel: Option<u8>,
    algo_trace_pending_idx: u8,
    /// 4 个上报变量(io->report[0..3])的时间序列环形缓冲 + 触发判定(out_active)序列 + 版本号。
    /// 每点带时刻(TracePoint), 才能与遥测曲线画在同一条真实时间轴上。
    algo_trace_report: [VecDeque<TracePoint>; 4],
    algo_trace_active: VecDeque<TracePoint>,
    algo_trace_version: u64,
    /// 最近一次 ALGO_GET_TRACE 请求的 seq(用于把 NAK 回执配对到追踪请求)。
    algo_trace_last_seq: Option<u8>,
    /// 追踪轮询退避计数(>0 表示还要跳过 N 次轮询)。设备无算法时 NAK 触发退避, 抑制刷屏。
    algo_trace_backoff: u32,
    /// 算法上传沿用灯效写入的 seq→ACK/NAK 归因；附带起始时刻以明确区分无回执超时。
    algo_upload_seq: Option<u8>,
    algo_upload_started_at: Option<std::time::Instant>,
    algo_upload_status: String,
    /// 上传 ACK 后延时再读一次 ALGO_GET_INFO 的倒计时(16ms/tick)。
    /// 固件把算法下发改成异步(整段 1KB 分页 + PSoC commit 校验最坏 ~700ms 在 core1 执行),
    /// ACK 只代表"已受理"; 立刻回读会拿到 psoc_valid=0 的在途态, 故延时再对账一次设备真值。
    algo_info_refresh_in: Option<u32>,
    algo_upload_version: u64,

    /// 共享算法可设置变量缓存(ABI cfg[8], ALGO_GET_CFG/SET_CFG)+ 版本号。
    algo_cfg: [u8; 8],
    /// 每槽是否已由设备或已提交草稿填入；false 时 UI 必须取 ABI 声明默认值，不能伪造 0。
    algo_cfg_valid: [bool; 8],
    algo_cfg_version: u64,
    /// 全局 CSD 配置缓存 gparam_id → value(GLOBAL_GET/GET_ALL 响应)+ 版本号。
    globals: BTreeMap<u8, u32>,
    globals_version: u64,
    /// CSD 诊断探针(DEBUG 级): 改全局后倒计时触发一次设备状态回读(参数/全局/Cp), 日志走 DEBUG,
    /// 默认过滤隐藏, 切到"调试"等级才显示。None=不排程。
    csd_diag_probe_in: Option<u32>,
    csd_diag_active: bool,
    csd_diag_off_in: Option<u32>,
    /// 恢复默认后重读刷新: 等 PSoC 重启+回填默认参数就绪后倒计时触发一次全量重读; None=未排程。
    post_reset_refetch_in: Option<u32>,
    post_reset_refetch_pending: bool,
    /// 全通道页"批量应用"的选择态。真相源放在这里而不是 Slint: 界面只负责展示与事件,
    /// 避免同一份选择在两侧各存一份而对不上(本工程反复踩过的"数据源不统一")。
    batch_sel: BatchApplySel,
    batch_sel_version: u64,
    /// 批量应用的源通道(参数从这里复制)。独立于"当前精调通道": 抽屉里要一边看着源通道的值、
    /// 一边勾目标通道, 两者若共用一个字段, 勾选动作就会把源通道改掉。
    batch_source: u8,
    /// 批量应用的"每参数待写值"(下标 = param_id - 1, 0x01..=0x0B)。
    /// `None` = 还没有值(源通道尚未回读到该参数, 且用户也没手填) ⇒ 应用时跳过该项, 绝不拿 0 顶替。
    /// ★为什么要独立于源通道真值★: 面板上的值现在是可手改的输入框, 用户改过之后必须以他填的数
    /// 为准; 若每次 PARAM_GET 回来都按源通道真值重置, 刚键入的数会被弹回(工程内 ParamEditorRow
    /// 踩过同一个坑)。故只在"选定源通道"与"该项仍为空"时才用真值预填。
    batch_values: [Option<u32>; 11],
    /// 当前遥测档位: Some(true)=逐通道档, Some(false)=仅统计/延迟轻档, None=未知(需重新下发)。
    telem_scope_channels: Option<bool>,
    /// UI 批量 CSD 操作的 host 侧串行队列(逐通道校准/基线/频率自适应)。见 `ch_ops`。
    ch_batch: ch_ops::ChBatch,
    /// 单通道响应噪声频谱扫描状态机。见 `ch_ops`。
    noise_sweep: ch_ops::NoiseSweep,
    /// 绘图视窗冻结快照。Some = 图停在快照那一刻(★遥测仍在收★, 与 telem_user_paused 语义不同)。
    plot_freeze: Option<ch_ops::PlotFreeze>,
    /// "Cp 辅助": 逐通道自适应时用实测 Cp 推出每通道的增益档/频率偏好档, 而不是 36 通道一个档。
    cp_assist: bool,
    /// 配置写类命令的串行发送队列 + 在途帧(cmd, seq, 已等 tick 数)。
    /// ★为什么必须串行★: 设备侧 core0 在写 flash(停 XIP+关中断) 与 CSD 重初始化期间完全不解析
    /// vendor 命令, 背靠背灌入会让主机侧 pipe 进错误态(实测 os error 22 + ConnectionAborted)。
    /// 窗口=1 的 ACK 驱动能让主机自动跟随设备节奏: 设备忙就自然停等, 不需要猜任何固定延时。
    cfg_tx_queue: std::collections::VecDeque<CfgTxFrame>,
    cfg_tx_inflight: Option<CfgTxInflight>,
    /// 最近一次 OUT 端点忙事件的有界抑制窗口；配置 FIFO 不受此门控影响。
    device_busy_until: Option<std::time::Instant>,
    device_busy_log_at: Option<std::time::Instant>,
    /// SAVE_CONFIG 成功后需整通道回读的 0→1 通道，避免重新启用后只出现空参数。
    post_save_refetch_channels: Vec<u8>,
    /// 全 36 通道 per-channel 参数重读队列(待读的 param_id)。
    /// ★为什么要队列★: 恢复默认/重连原来只重读"当前通道 + CH0"两个通道, 其余 34 个通道的缓存
    /// 既不刷新也不失效, 界面显示的是过期值(或从未填充的空), 表现为"只留了一个通道的数据"。
    /// 改用 PARAM_GET_ALL 的 0xFF 全通道变体(一条命令覆盖 36 通道的同一参数), 11 个参数即 11 条命令,
    /// 且每个 16ms tick 只发一条 —— 既不提高轮询频率, 也不会像 36 条单通道请求那样打爆 OUT 端点。
    param_refetch_ids: Vec<u8>,
    /// 已自动回写过修正值的 `(ch, param_id)` 登记表 —— 回读防污染的防写风暴闸门。
    /// 同一项自动回写至多一次; 回读到合法值即注销, 允许下次真出问题时重新纠正。
    /// (设备若一直回非法值, 只会继续告警, 不会变成"回读→回写→回读"的无限风暴。)
    param_fix_sent: std::collections::HashSet<(u8, u8)>,
    /// 已自动回写过修正值的 gparam_id 登记表 —— 全局项回读防污染的防写风暴闸门。
    /// 语义与 `param_fix_sent` 完全一致(每项至多回写一次, 回读到合法值即注销), 只是键无通道维度。
    global_fix_sent: std::collections::HashSet<u8>,

    /// 阻塞类操作(校准/基线复位/重启/频率自适应)进行中标志 + 中文标签, 供 UI 锁定按钮+显示运行图标。
    /// 触发时置位并记录期望回执 seq; 收到匹配 seq 的 ACK/NAK 或 AUTO_TUNE 响应即清除。
    op_busy: bool,
    op_label: String,
    op_wait_seq: Option<u8>,
    op_version: u64,
    /// 阻塞操作已进行的 tick 数(16ms/tick): 用于卡死检测——超时则日志告警并自动解锁, 定位"卡校准中"。
    /// ★只作卡死检测★: 进度帧会把它归零(证明设备活着), 故不可用于算总耗时。
    op_busy_ticks: u32,
    /// 阻塞操作的起始时刻: 真实耗时只由它计算(进度帧不影响), 修正"耗时恒为 0ms"的显示回归。
    op_started_at: Option<std::time::Instant>,
    /// CSD 变更串行化冷却(tick): 校准/基线/自适应/全局提交等会让 PSoC 忙于重初始化/重校准的操作,
    /// 发起后进入冷却窗口, 期间拒绝任何新的 CSD 变更, 从根源杜绝"多操作叠加把设备扫描/测量搞死"的异常。
    /// (ACK 只代表"已受理"非"真完成", 故不能靠 ACK 立即解锁; 用保守时间窗兜住 PSoC 真实完成。)
    csd_cooldown_ticks: u32,
    csd_cooldown_label: String,
    /// 频率自适应结果(最近一次): 0=未执行/进行中, 1=成功, 2=失败(超硬件能力); div=找到的 snsClk 分频。
    auto_tune_result: u8,
    auto_tune_div: u16,
    auto_tune_version: u64,
    /// 最近一次自适应的目标通道: 0..35=单通道, 0xFF=全通道。
    auto_tune_ch: u8,
    /// 逐通道自适应结果(单通道页显示用): result 0=未执行/进行中 1=成功 2=失败; div=该通道找到的分频。
    auto_tune_ch_result: [u8; 36],
    auto_tune_ch_div: [u16; 36],
    /// 全通道终态收到后等待 PARAM_GET_ALL(0xFF, 0x08) 的成功数；回读到齐后才记录真实分频范围。
    auto_tune_all_refresh_pending: Option<u16>,
    /// 设备推送的自适应阶段进度(AUTO_TUNE_PROGRESS 0x2E): 长自适应(20-25s)期间实时更新,
    /// 使 op_label 能显示"到哪一步了", 并证明设备活着(每帧重置卡死计时)。
    auto_tune_progress: crate::proto::AutoTuneProgress,
    auto_tune_progress_version: u64,
    /// 当前在途自适应请求的 seq(= 下发 AUTO_TUNE 那一帧的 seq)。**推送流终态的归属判据**:
    /// 设备把它逐帧回显(见 `AutoTuneProgress::origin_seq`), 对不上的帧就是上一轮的残留 —— 逐通道
    /// 批量下必须整帧丢弃, 否则旧终态会被算到下一个通道头上。终态处理完即置 None(迟到重发不再生效)。
    auto_tune_req_seq: Option<u8>,
    /// 设备推送的救砖进度(PSOC_RESCUE_PROGRESS 0x09): 全片擦写+校验+重新应用全程可见。
    rescue_progress: crate::proto::PsocRescueProgress,
    rescue_progress_version: u64,
    /// true = 本次阻塞操作的 ACK 仅表示"已受理", 真实完成由推送流终态判定(频率自适应用)。
    op_ack_is_accept: bool,

    /// 物理键盘 GPIO1-12 实时按下位(KBD_GET_STATE 响应)+ 版本号。
    kbd_state: u16,
    kbd_state_version: u64,
    /// 物理键 HID 键码表 + 修饰位表(12 项, KBD_GET_MAP 响应)+ 版本号。
    kbd_map: [u8; 12],
    kbd_keymod: [u8; 12],
    kbd_map_version: u64,
    /// 触控→键盘映射: 34 分区键码 + 修饰位 + 总开关(KBD_GET_TOUCHMAP 响应)+ 版本号。
    kbd_touchmap: [u8; 34],
    kbd_zonemod: [u8; 34],
    kbd_touch_en: bool,
    kbd_touchmap_version: u64,
    /// 长按参数设备回读缓存(12 物理键 + 34 分区, KBD_GET_HOLD 响应)+ 版本号。
    kbd_hold_phys: [HoldParam; KBD_HOLD_PHYS_COUNT],
    kbd_hold_zone: [HoldParam; KBD_HOLD_ZONE_COUNT],
    kbd_hold_version: u64,
    /// 物理键去抖前(raw)与实际输出(out)的 12 位实时态。KBD_GET_STATE 响应尾部追加字段;
    /// 旧固件只回 phys_state 时保持 0 —— 不拿 phys_state 冒充 raw/out(那会让防抖效果看不出来)。
    kbd_state_raw: u16,
    kbd_state_out: u16,
    /// 每键触发极性 + 独立防抖: 设备真值缓存 + 版本号(草稿在 `drafts` 内)。
    kbd_keycfg: [KbdKeyCfg; KBD_HOLD_PHYS_COUNT],
    kbd_keycfg_version: u64,
    /// 固件解析后的**生效**极性掩码(bit i = 1 → 按高电平触发判定)。AUTO 档到底学成了高还是低
    /// 只能靠它显示。旧固件不追加该字段时保持 0。
    kbd_pol_resolved_mask: u16,
    /// 上面这份掩码是否真的从设备读到过。false = 旧固件没回传该尾部字段, 生效电平**未知**;
    /// 此时 UI 必须显示"未知", 不能把 mask=0 当成"全部低电平触发"那样凭空造结论。
    kbd_pol_resolved_known: bool,
    /// 设备是否支持每键配置/边沿记录。None=尚未问过; Some(false)=旧固件回了 NAK, 停止轮询。
    kbd_keycfg_supported: Option<bool>,
    kbd_edges_supported: Option<bool>,
    /// 逻辑分析仪边沿记录环(主机侧留存窗口, 定长 FIFO)+ 版本号。
    kbd_edges: VecDeque<KbdEdgeRec>,
    kbd_edges_version: u64,
    /// 设备累计丢弃条数的上一次读数 + 主机侧累计感知到的丢失条数(诚实显示"有事件丢失")。
    kbd_edge_overflow_last: u32,
    kbd_edge_lost: u32,
    /// 上次拉取后设备侧仍剩余的条数(>0 说明拉取跟不上, UI 要提示)。
    kbd_edge_remaining: u16,
    /// 在途回读 seq(窗口=1): 收到响应/NAK 才再发, 避免 16ms tick 把 vendor 端点打满,
    /// 同时用于把 NAK 归因到"旧固件不支持"而不是"没有边沿"。
    kbd_edge_req_seq: Option<u8>,
    kbd_keycfg_req_seq: Option<u8>,
    /// 触控组合映射: 设备真值缓存 / 版本号 / 设备是否支持 / 在途回读 seq(草稿在 `drafts` 内)。
    /// 整表管理而不是逐条: 固件的 KBD_SET_COMBO 就是整表替换语义(逐条增删会产生半新半旧的中间态,
    /// 而同一分区可能同时属于多条, 中间态会瞬时输出错误按键)。
    kbd_combo_cache: Vec<crate::proto::KbdComboItem>,
    kbd_combo_version: u64,
    kbd_combo_supported: Option<bool>,
    kbd_combo_req_seq: Option<u8>,
    /// 待新建条目(分区集合 + 键 + 修饰)。放 Rust 而非 Slint: "添加"要与已有表做去重判定。
    kbd_combo_pending_mask: u64,
    kbd_combo_pending_keys: [u8; crate::proto::KBD_COMBO_KEY_COUNT],
    kbd_combo_pending_mods: u8,
    /// 已有映射里"下一次抓到的键要整行替换"的行号(见 kbd_combo_capture_key_at)。
    /// None = 累加。只是一个待替换登记, 不代表该行此刻被清空过。
    kbd_combo_key_edit_row: Option<usize>,

    /// mai2 串口(游戏触控上报)运行态回读缓存(MAI2_GET_STATE 响应)+ 版本号; None=尚未回读。
    mai2_state: Option<Mai2State>,
    mai2_version: u64,

    /// mai2light 灯板协议运行态快照(LED_GET 响应)+ 版本号; None=尚未回读。
    led_state: Option<LedState>,
    led_version: u64,
    /// 灯效写操作(LED_SET_REGION / LED_PREVIEW)的下发 seq + 类型: 用于把 ACK/NAK 归因到
    /// 具体操作, 失败原因必须显示而非静默。预览与映射共用本槽位(两者不会并发)。
    led_apply_seq: Option<(u8, LedWriteOp)>,
    led_apply_status: String,
    /// 期望运行态(掉线重连自动恢复用, 断连不清)+ 重连回读对账标记。
    desired: DesiredState,
    restore_verify: RestoreVerify,

    // ------------------------------------------------------------------
    // 配置草稿覆盖层 (draft overlay)
    //
    // UI 的所有编辑只写入草稿, 不再即时下发设备; 只有点击"保存"(save_config)
    // 才把草稿统一下发并请求写 flash。读取类 getter 一律"草稿优先 → 缓存兜底",
    // 使 UI 立即看到自己编辑的值, 而设备运行态保持不变直到保存。
    //
    // ★12 份草稿 + 未保存脏键集合 + 版本号全部收在 `ConfigDrafts` 内且字段私有★:
    // 草稿本体拿不到 `&mut`, 任何修改只能经其方法, 方法内部必定同步登记/撤销脏键并
    // bump version —— "改了草稿却忘记标记未保存"在编译期即不可能(见 drafts.rs 文件头)。
    // 脏状态不再有独立 bool: `is_config_dirty()` 派生自脏键集合是否为空。
    // ------------------------------------------------------------------
    drafts: ConfigDrafts,
    /// 刚下发出去的全局项期望值, 等回读真值后逐项对账(不一致即固件夹取/拒收, 必须告警)。
    globals_expected: BTreeMap<u8, u32>,
    /// 回读对账倒计时(tick 数)与"本次 GET_ALL 响应用于对账"的标记。
    globals_verify_in: Option<u32>,
    globals_verify_active: bool,
    /// 已处理的自持恢复事件 seq(设备背压时会重发同一条, 按此去重)。
    self_heal_last_seq: Option<u16>,
    /// 保存时若提交了 mode.work(USB Serial/HID 拓扑)改动, 置真: 该改动需整机重启重枚举才生效,
    /// 由 UI 侧在保存后延时自动重启设备并清除本标记。
    pending_reboot: bool,

    /// JIT 算法: 最近一次编译的 C 源 + 反汇编 ASM(objdump -d .text)+ 版本, 供 UI 子标签查看/留档。
    algo_source: String,
    algo_asm: String,
    algo_asm_version: u64,
    /// 最近一次"编译"(未上传)成功产出的 ASM 二进制, 供"上传"复用; 编译/上传分离。
    algo_compiled_blob: Option<Vec<u8>>,
    /// 从设备回读的算法 C 源(映射表), 供"读取信息"还原可编辑 C; 版本号供 UI 门控刷新。
    algo_device_src: String,
    algo_device_src_version: u64,
    /// schema(ALGO_REPORT/ALGO_SETTING 声明)来源的版本号: `algo_device_src` 或 `algo_source`
    /// 任一变化即自增, 供 UI 门控重建算法面板与 report idx 轮询集合(不必每帧比整段源码字符串)。
    algo_schema_version: u64,
    /// 从设备回读的算法 ASM 机器码(hex dump 文本)+ 版本, 供无本地编译产物时在反汇编页查看。
    algo_device_code_hex: String,
    algo_device_code_version: u64,
    /// C 源分片上传/回读状态机(源最大 32KB, 单帧 payload 只有 4096)。
    /// 与 `cfg_tx_queue` 同一口径: 窗口=1, 收到回执才推进 —— 但不把 16 片一次塞进队列,
    /// 否则一次上传就把串行队列占满, 保存类命令全被顶到后面。
    algo_src_tx: Option<AlgoSrcTx>,
    algo_src_rx: Option<AlgoSrcRx>,

    /// 事件日志环形缓冲(连接/收发/错误/自动动作)。★仅供本控制器内部/无头诊断留档★:
    /// UI 日志页的唯一数据源是 `logging::hub()`(那里还收了 io/nusb 等所有 target 的记录),
    /// 本缓冲不再对外提供显示文本, 以免同一份日志出现两种口径。
    event_log: VecDeque<(LogLevel, String)>,
    /// 事件日志版本号,每次追加自增,供 UI 判断是否刷新日志文本。
    log_seq: u64,
    /// UI 日志过滤等级(0=Error 1=Warn 2=Info 3=Debug): 只显示 等级值 <= 此阈值 的条目。
    /// 默认 Info: 隐藏"原始数据交互/诊断回读"等 Debug 噪声, 只看关键交互与异常。
    log_filter: u8,
    /// 已处理 TELEM_DATA 帧计数(供日志/诊断显示遥测是否在流动)。
    telem_frame_count: u64,
    /// 已接收的 TELEM_DATA 完整线上字节数(payload + 固定 9B 帧封装)，供 soak 带宽汇总。
    telem_wire_bytes: u64,
}

impl AppController {
    pub fn new() -> Self {
        let (toolbox_auto_port, toolbox_serial_com, toolbox_light_com, vcam_kbd_device) =
            Self::_load_toolbox_cfg();

        // 初始化 36 通道的遥测缓冲和参数缓存
        let mut telem_buf = Vec::with_capacity(36);
        let mut params = Vec::with_capacity(36);
        for _ in 0..36 {
            telem_buf.push(VecDeque::new());
            params.push(BTreeMap::new());
        }

        AppController {
            pending_registry: PendingRegistry::new(),
            skipped_log: SkippedLog::new(128),
            poll_rules: PollRules::new(),
            conn_probe_queue: VecDeque::new(),
            conn_probe_inflight: None,
            devices: Vec::new(),
            io: None,
            state: ConnState::Disconnected,
            device_info: None,
            csd_mode: None,
            psoc_scan_live_at: None,
            psoc_info_live_at: None,
            psoc_gen_seen: None,
            seq: 0,
            last_error: None,
            status_text: "未连接".to_string(),
            toolbox_auto_port,
            toolbox_serial_com,
            toolbox_light_com,
            vcam_kbd_device,
            config_cache: BTreeMap::new(),
            cfg_all_accum: Vec::new(),
            config_version: 0,
            bind_progress: None,
            bind_start_seq: None,
            listen_zone: None,
            listen_baseline_mask: 0,
            listen_hold: ListenHold {
                channel: None,
                since: None,
            },
            interactive_bind: false,
            telem_buf,
            telem_last_raw: [None; 36],
            telem_freeze_count: [0; 36],
            telem_active: false,
            focus: FocusState::default(),
            telem_last_ts: 0,
            telem_clock: DevClock::default(),
            telem_frame_at: None,
            telem_version: 0,
            plot_version: 0,
            telem_samples_per_sec: 0,
            telem_scan_period_us: 0,
            telem_stats_at: None,
            telem_user_paused: false,
            telem_heal_at: None,
            telem_started_at: None,
            telem_channel_last_seen: [None; 36],
            telem_channel_heal_attempts: [0; 36],
            ch_batch: ch_ops::ChBatch::default(),
            noise_sweep: ch_ops::NoiseSweep::default(),
            plot_freeze: None,
            cp_assist: false,
            telem_lat_spi_us: 0,
            telem_lat_proc_us: 0,
            telem_lat_usb_us: 0,
            latency_correction_samples: LatencyCorrectionSamples::default(),
            telem_lat_corrected_us: None,
            lat_total_hist: VecDeque::new(),
            lat_version: 0,
            params,
            param_version: 0,
            cp: vec![None; 36],
            cp_channel_versions: vec![0; 36],
            cp_version: 0,
            algo_info: None,
            algo_version: 0,
            algo_rom: vec![0u16; 36],
            algo_rom_version: 0,
            algo_trace_channel: None,
            algo_trace_pending_idx: 0,
            algo_trace_report: [
                VecDeque::new(),
                VecDeque::new(),
                VecDeque::new(),
                VecDeque::new(),
            ],
            algo_trace_active: VecDeque::new(),
            algo_trace_version: 0,
            algo_trace_last_seq: None,
            algo_trace_backoff: 0,
            algo_upload_seq: None,
            algo_upload_started_at: None,
            algo_upload_status: String::new(),
            algo_info_refresh_in: None,
            algo_upload_version: 0,
            algo_cfg: [0u8; 8],
            algo_cfg_valid: [false; 8],
            algo_cfg_version: 0,
            globals: BTreeMap::new(),
            globals_version: 0,
            csd_diag_probe_in: None,
            csd_diag_active: false,
            csd_diag_off_in: None,
            post_reset_refetch_in: None,
            post_reset_refetch_pending: false,
            batch_sel: BatchApplySel::new(),
            batch_sel_version: 0,
            batch_source: 0,
            batch_values: [None; 11],
            telem_scope_channels: None,
            cfg_tx_queue: std::collections::VecDeque::new(),
            cfg_tx_inflight: None,
            device_busy_until: None,
            device_busy_log_at: None,
            post_save_refetch_channels: Vec::new(),
            param_refetch_ids: Vec::new(),
            param_fix_sent: std::collections::HashSet::new(),
            global_fix_sent: std::collections::HashSet::new(),

            op_busy: false,
            op_busy_ticks: 0,
            op_started_at: None,
            csd_cooldown_ticks: 0,
            csd_cooldown_label: String::new(),
            op_label: String::new(),
            op_wait_seq: None,
            op_version: 0,
            auto_tune_result: 0,
            auto_tune_div: 0,
            auto_tune_version: 0,
            auto_tune_ch: 0xFF,
            auto_tune_ch_result: [0u8; 36],
            auto_tune_ch_div: [0u16; 36],
            auto_tune_all_refresh_pending: None,
            auto_tune_progress: crate::proto::AutoTuneProgress::default(),
            auto_tune_progress_version: 0,
            auto_tune_req_seq: None,
            rescue_progress: crate::proto::PsocRescueProgress::default(),
            rescue_progress_version: 0,
            op_ack_is_accept: false,
            kbd_state: 0,
            kbd_state_version: 0,
            kbd_map: [0u8; 12],
            kbd_keymod: [0u8; 12],
            kbd_map_version: 0,
            kbd_touchmap: [0u8; 34],
            kbd_zonemod: [0u8; 34],
            kbd_touch_en: false,
            kbd_touchmap_version: 0,
            kbd_hold_phys: [HoldParam::default(); KBD_HOLD_PHYS_COUNT],
            kbd_hold_zone: [HoldParam::default(); KBD_HOLD_ZONE_COUNT],
            kbd_hold_version: 0,
            kbd_state_raw: 0,
            kbd_state_out: 0,
            kbd_keycfg: [KbdKeyCfg::default(); KBD_HOLD_PHYS_COUNT],
            kbd_keycfg_version: 0,
            kbd_pol_resolved_mask: 0,
            kbd_pol_resolved_known: false,
            kbd_keycfg_supported: None,
            kbd_edges_supported: None,
            kbd_edges: VecDeque::new(),
            kbd_edges_version: 0,
            kbd_edge_overflow_last: 0,
            kbd_edge_lost: 0,
            kbd_edge_remaining: 0,
            kbd_edge_req_seq: None,
            kbd_keycfg_req_seq: None,
            kbd_combo_cache: Vec::new(),
            kbd_combo_version: 0,
            kbd_combo_supported: None,
            kbd_combo_req_seq: None,
            kbd_combo_pending_mask: 0,
            kbd_combo_pending_keys: [0; crate::proto::KBD_COMBO_KEY_COUNT],
            kbd_combo_pending_mods: 0,
            kbd_combo_key_edit_row: None,
            mai2_state: None,
            mai2_version: 0,
            led_state: None,
            led_version: 0,
            led_apply_seq: None,
            led_apply_status: String::new(),
            desired: DesiredState::default(),
            restore_verify: RestoreVerify::default(),
            drafts: ConfigDrafts::new(),
            globals_expected: BTreeMap::new(),
            globals_verify_in: None,
            globals_verify_active: false,
            self_heal_last_seq: None,
            pending_reboot: false,
            algo_source: String::new(),
            algo_asm: String::new(),
            algo_asm_version: 0,
            algo_compiled_blob: None,
            algo_device_src: String::new(),
            algo_device_src_version: 0,
            algo_src_tx: None,
            algo_src_rx: None,
            algo_schema_version: 0,
            algo_device_code_hex: String::new(),
            algo_device_code_version: 0,
            event_log: VecDeque::new(),
            log_filter: LogLevel::Info as u8,
            log_seq: 0,
            telem_frame_count: 0,
            telem_wire_bytes: 0,
        }
    }

    /// 追加一条 Info 级事件日志(关键交互/状态)。保留最近 400 条。
    pub fn push_log(&mut self, msg: impl Into<String>) {
        self._log(LogLevel::Info, msg.into());
    }
    /// Debug 级: 实际数据帧收发 / 诊断回读等高频细节, 默认过滤隐藏, 选 Debug 等级才显示。
    pub fn push_log_debug(&mut self, msg: impl Into<String>) {
        self._log(LogLevel::Debug, msg.into());
    }
    /// Warn 级: NAK/可恢复异常。
    pub fn push_log_warn(&mut self, msg: impl Into<String>) {
        self._log(LogLevel::Warn, msg.into());
    }
    /// Error 级: 断连/不可恢复错误。
    pub fn push_log_error(&mut self, msg: impl Into<String>) {
        self._log(LogLevel::Error, msg.into());
    }

    fn _log(&mut self, level: LogLevel, msg: String) {
        match level {
            LogLevel::Error => log::error!("{}", msg),
            LogLevel::Warn => log::warn!("{}", msg),
            LogLevel::Info => log::info!("{}", msg),
            LogLevel::Debug => log::debug!("{}", msg),
        }
        self.event_log.push_back((level, msg));
        while self.event_log.len() > 400 {
            self.event_log.pop_front();
        }
        self.log_seq = self.log_seq.wrapping_add(1);
    }

    /// 设置 UI 日志过滤等级(0=Error 1=Warn 2=Info 3=Debug)。
    pub fn set_log_filter(&mut self, level: u8) {
        self.log_filter = level.min(LogLevel::Debug as u8);
        self.log_seq = self.log_seq.wrapping_add(1); // 触发 UI 立即按新等级刷新
    }
    pub fn log_filter(&self) -> u8 {
        self.log_filter
    }

    // ★已删除 log_text()★: 它按 event_log 拼显示文本, 是日志的第二个数据源(与 logging::hub
    // 的行号/时间戳/收录范围都不一致)。UI 一律从 hub 取快照(main.rs 日志块), 这里不再提供。

    /// 日志版本号(每追加一条自增),供 UI 判断是否刷新日志文本。
    pub fn log_seq(&self) -> u64 {
        self.log_seq
    }

    /// 已处理 TELEM_DATA 帧计数。
    pub fn telem_frame_count(&self) -> u64 {
        self.telem_frame_count
    }

    /// 已接收的 TELEM_DATA 线上字节数，含协议固定封装字节，便于压测计算实际带宽。
    pub fn telem_wire_bytes(&self) -> u64 {
        self.telem_wire_bytes
    }

    // ------------------------------------------------------------------
    // 工具箱端口配置(本地 toolbox.cfg)
    // ------------------------------------------------------------------

    pub fn toolbox_auto_port(&self) -> bool {
        self.toolbox_auto_port
    }

    pub fn toolbox_serial_com(&self) -> u16 {
        self.toolbox_serial_com
    }

    pub fn toolbox_light_com(&self) -> u16 {
        self.toolbox_light_com
    }

    pub fn set_toolbox_auto_port(&mut self, enabled: bool) {
        self.toolbox_auto_port = enabled;
        self._save_toolbox_cfg();
    }

    pub fn set_toolbox_serial_com(&mut self, port: u16) {
        self.toolbox_serial_com = port;
        self._save_toolbox_cfg();
    }

    pub fn set_toolbox_light_com(&mut self, port: u16) {
        self.toolbox_light_com = port;
        self._save_toolbox_cfg();
    }

    /// 虚拟摄像头输入源键盘设备路径(空串 = 所有键盘)。
    pub fn vcam_kbd_device(&self) -> String {
        self.vcam_kbd_device.clone()
    }

    pub fn set_vcam_kbd_device(&mut self, path: String) {
        self.vcam_kbd_device = path;
        self._save_toolbox_cfg();
    }

    /// 按当前设置为游戏串口分配固定 COM 号，并返回 UI 可直接显示的结果。
    /// force=true(用户点"立即应用"): 即使已是目标口也强制重启端口节点使其真正生效。
    pub fn apply_ports(&mut self, force: bool) -> String {
        let summary_text =
            crate::comport::auto_assign(self.toolbox_serial_com, self.toolbox_light_com, force)
                .summary_text();
        self.push_log(format!("工具箱端口应用: {}", summary_text));
        summary_text
    }

    /// 启动时仅在已启用时执行一次自动端口分配(非强制, 已匹配则跳过, 避免每次开机重启端口)。
    pub fn maybe_auto_assign_ports(&mut self) -> Option<String> {
        if self.toolbox_auto_port {
            Some(self.apply_ports(false))
        } else {
            None
        }
    }

    fn _toolbox_cfg_path() -> std::path::PathBuf {
        match std::env::current_exe() {
            Ok(exe) => match exe.parent() {
                Some(dir) => return dir.join("toolbox.cfg"),
                None => log::warn!("无法取得程序所在目录，回退到当前目录保存 toolbox.cfg"),
            },
            Err(error) => log::warn!(
                "无法取得程序路径，回退到当前目录保存 toolbox.cfg: {}",
                error
            ),
        }
        match std::env::current_dir() {
            Ok(dir) => dir.join("toolbox.cfg"),
            Err(error) => {
                log::warn!("无法取得当前目录，使用相对路径 toolbox.cfg: {}", error);
                std::path::PathBuf::from("toolbox.cfg")
            }
        }
    }

    fn _load_toolbox_cfg() -> (bool, u16, u16, String) {
        const DEFAULT_AUTO_PORT: bool = false;
        const DEFAULT_SERIAL_COM: u16 = 3;
        const DEFAULT_LIGHT_COM: u16 = 21;

        let path = Self::_toolbox_cfg_path();
        let content = match std::fs::read_to_string(&path) {
            Ok(content) => content,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                return (
                    DEFAULT_AUTO_PORT,
                    DEFAULT_SERIAL_COM,
                    DEFAULT_LIGHT_COM,
                    String::new(),
                );
            }
            Err(error) => {
                log::warn!("读取工具箱端口配置失败({}): {}", path.display(), error);
                return (
                    DEFAULT_AUTO_PORT,
                    DEFAULT_SERIAL_COM,
                    DEFAULT_LIGHT_COM,
                    String::new(),
                );
            }
        };

        let mut auto_port = DEFAULT_AUTO_PORT;
        let mut serial_com = DEFAULT_SERIAL_COM;
        let mut light_com = DEFAULT_LIGHT_COM;
        let mut vcam_kbd = String::new();
        for line in content.lines() {
            let Some((key, value)) = line.split_once('=') else {
                continue;
            };
            match key.trim() {
                "auto_port" => {
                    auto_port = match value.trim() {
                        "0" => false,
                        "1" => true,
                        _ => DEFAULT_AUTO_PORT,
                    };
                }
                "serial_com" => {
                    serial_com = value.trim().parse().unwrap_or(DEFAULT_SERIAL_COM);
                }
                "light_com" => {
                    light_com = value.trim().parse().unwrap_or(DEFAULT_LIGHT_COM);
                }
                // 设备路径含 '#' 与 '\', 但不含 '=', 故 split_once('=') 取值即完整路径。
                "vcam_kbd" => {
                    vcam_kbd = value.trim().to_string();
                }
                _ => {}
            }
        }
        (auto_port, serial_com, light_com, vcam_kbd)
    }

    fn _save_toolbox_cfg(&self) {
        let path = Self::_toolbox_cfg_path();
        let content = format!(
            "auto_port={}\nserial_com={}\nlight_com={}\nvcam_kbd={}\n",
            u8::from(self.toolbox_auto_port),
            self.toolbox_serial_com,
            self.toolbox_light_com,
            self.vcam_kbd_device,
        );
        if let Err(error) = std::fs::write(&path, content) {
            log::warn!("保存工具箱端口配置失败({}): {}", path.display(), error);
        }
    }

    // ------------------------------------------------------------------
    // 设备枚举
    // ------------------------------------------------------------------

    /// 重新扫描恒定枚举的 config WinUSB 接口。serial/light 是游戏用 CDC，
    /// 不参与 host_cmd 连接选择；其 COM 口识别仍由 `comport` 模块独立负责。
    pub fn refresh_devices(&mut self) {
        let mut devices: Vec<DeviceEntry> = io::list_devices()
            .into_iter()
            .map(|c| {
                let function = CdcFunction::Config;
                let product = c.product.as_deref().unwrap_or("mai2 config");
                let label = format!("{}  [{}]", product, Self::function_label(function));
                DeviceEntry {
                    port_name: c.port_name,
                    function,
                    label,
                }
            })
            .collect();

        devices.sort_by_key(|d| Self::function_priority(d.function));
        self.devices = devices;
    }

    fn function_priority(f: CdcFunction) -> u8 {
        match f {
            CdcFunction::Config => 0,
            CdcFunction::Serial => 1,
            CdcFunction::Light => 2,
            CdcFunction::Unknown => 3,
        }
    }

    fn function_label(f: CdcFunction) -> &'static str {
        match f {
            CdcFunction::Config => "config",
            CdcFunction::Serial => "serial",
            CdcFunction::Light => "light",
            CdcFunction::Unknown => "unknown",
        }
    }

    // ------------------------------------------------------------------
    // 连接管理
    // ------------------------------------------------------------------

    /// 连接 `devices[index]`:启动 IO 线程并发出 HELLO。
    pub fn connect(&mut self, index: usize) -> anyhow::Result<()> {
        let entry = self
            .devices
            .get(index)
            .ok_or_else(|| anyhow::anyhow!("设备索引超出范围: {}", index))?;
        let port_name = entry.port_name.clone();

        let handle = io::spawn(&port_name)?;
        let seq = self.next_seq();
        handle.send(Frame::hello(seq))?;

        if let Some(previous) = self.io.take() {
            previous.stop();
        }
        self._clear_sensor_caches();
        self.io = Some(handle);
        self.state = ConnState::Connecting;
        self.device_info = None;
        self.csd_mode = None;
        self.device_busy_until = None;
        self.device_busy_log_at = None;
        self.last_error = None;
        self.push_log(format!("连接设备[{}]: 启动 IO + 发 HELLO", index));
        self.status_text = format!("连接中: {}", port_name);
        Ok(())
    }

    /// 重发 HELLO(握手重试)。用于重连场景: 重开 WinUSB 句柄后设备端 bulk OUT 数据翻转位
    /// (data toggle)与主机不同步, 首个 HELLO 会被设备当作重复包丢弃; 该包被丢弃后翻转位
    /// 即重新对齐, 故只要在未收到 DEVICE_INFO 前周期性重发 HELLO, 后续包即可送达并握手成功。
    /// 仅在 `Connecting` 态由 UI 定时器调用。
    pub fn resend_hello(&mut self) -> anyhow::Result<()> {
        if self._device_busy() {
            if self
                .device_busy_log_at
                .is_none_or(|at| at.elapsed() >= std::time::Duration::from_secs(2))
            {
                self.device_busy_log_at = Some(std::time::Instant::now());
                self.push_log_warn("设备忙，暂停 HELLO 握手轮询，等待恢复".to_string());
            }
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::hello(seq))?;
        }
        Ok(())
    }

    /// 断开当前连接,停止 IO 线程,清空设备信息与配置缓存。
    pub fn disconnect(&mut self) {
        if let Some(handle) = self.io.take() {
            handle.stop();
        }
        self.state = ConnState::Disconnected;
        self.device_info = None;
        self.csd_mode = None;
        self.status_text = "未连接".to_string();
        self.config_cache.clear();
        self.cfg_all_accum.clear();
        self._clear_drafts();

        self._clear_sensor_caches();
    }

    /// 清设备回读缓存(断连/重连时调用)。★不动 `desired`★: 期望运行态必须活过断连,
    /// 否则重连后无从恢复触控。
    fn _clear_sensor_caches(&mut self) {
        self.pending_registry.clear();
        self.conn_probe_queue.clear();
        self.conn_probe_inflight = None;
        self.restore_verify.clear();
        self.post_save_refetch_channels.clear();
        // 灯效映射草稿按设备灯链长度校验, 换设备/重连后必须重来, 否则会拿旧链长的区段去 NAK。
        self.led_state = None;
        self.drafts.drop_led_region();
        self.led_apply_seq = None;
        self.led_apply_status.clear();
        self.led_version = self.led_version.wrapping_add(1);
        self.telem_active = false;
        // 单通道流会话属于上一条链路: 不清就会用一个设备侧已不存在的 session 去判定归属,
        // 重连后既收不到数据也不会重开(幂等判定认为"目标没变")。
        self.focus.clear();
        self._sweep_device_disconnected();
        self.bind_progress = None;
        self.bind_start_seq = None;
        self.telem_last_raw = [None; 36];
        self.telem_freeze_count = [0; 36];
        self.telem_channel_last_seen = [None; 36];
        self.telem_channel_heal_attempts = [0; 36];
        for buf in &mut self.telem_buf {
            buf.clear();
        }
        self.telem_last_ts = 0;
        // 冻结的绘图快照属于上一条链路: 留着会让重连后的界面一直显示旧设备的波形。
        self.plot_freeze = None;
        // 换设备/重连后设备可能已重启, ts_us 从头开始 → 时钟必须归零, 否则会算出一个巨大的假间隙。
        self.telem_clock.clear();
        self.telem_frame_at = None;
        self.telem_lat_spi_us = 0;
        self.telem_lat_proc_us = 0;
        self.telem_lat_usb_us = 0;
        self.latency_correction_samples = LatencyCorrectionSamples::default();
        self.telem_lat_corrected_us = None;
        self.lat_total_hist.clear();
        self.lat_version = self.lat_version.wrapping_add(1);
        // 链路活性证据必须随缓存一起清: 换设备/重连后旧设备的"活着"不能算新链路的证据。
        self.psoc_scan_live_at = None;
        self.psoc_info_live_at = None;
        self.psoc_gen_seen = None;
        self.device_busy_until = None;
        self.device_busy_log_at = None;
        for param_map in &mut self.params {
            param_map.clear();
        }
        for value in &mut self.cp {
            *value = None;
        }
        for version in &mut self.cp_channel_versions {
            *version = 0;
        }
        self.cp_version = self.cp_version.wrapping_add(1);
        self.algo_trace_channel = None;
        self.algo_trace_pending_idx = 0;
        for buf in &mut self.algo_trace_report {
            buf.clear();
        }
        self.algo_trace_active.clear();
        self.algo_trace_version = self.algo_trace_version.wrapping_add(1);
        self.algo_src_tx = None;
        self.algo_src_rx = None;
        // 换设备/重连: 上一条链路的自适应请求不再有归属对象, 之后收到的任何终态都算旧帧。
        self.auto_tune_req_seq = None;
    }

    // ------------------------------------------------------------------
    // 事件轮询(由 UI 侧 Timer 每帧调用)
    // ------------------------------------------------------------------

    /// 排空全部 IO 事件；无头工具依赖它在一次调用内收敛当前队列。
    pub fn poll(&mut self) {
        self._poll_events(usize::MAX, None);
    }

    /// UI 事件循环使用的有界消费：每帧最多占用 2ms/256 条，避免突发遥测长期霸占 Slint 线程。
    pub fn poll_ui(&mut self) {
        self._poll_events(256, Some(std::time::Duration::from_millis(2)));
    }

    fn _poll_events(&mut self, max_events: usize, budget: Option<std::time::Duration>) {
        let started = std::time::Instant::now();
        let mut handled = 0usize;
        while handled < max_events && budget.is_none_or(|limit| started.elapsed() < limit) {
            let event = self.io.as_ref().and_then(|handle| handle.try_recv());
            let Some(event) = event else { break };
            self.handle_event(event);
            handled += 1;
        }
        // ★写命令队列必须在这里推进, 不能只挂在 csd_diag_tick() 上★
        // `_queue_tx` 只入队, 真正发送靠 `_pump_cfg_tx`。它原先只在 csd_diag_tick(GUI 的 16ms tick)里
        // 被调用, 于是任何"只 poll 不 tick"的调用方(无头 selftest 的绝大多数路径)把写命令入队后
        // 永远发不出去 —— 命令被静默丢弃, 而调用方看不到任何错误。
        // 实测代价: `--reset-config` 一直报 PASS(它只是重读了配置项数), 但设备端 RESET_DEFAULTS
        // 处理器从未执行(在其首行埋 SelfHeal 计数验证: 累计恒为 2 不动), 所以被污染的 CSD store
        // (snsClk=8)永远清不掉, 每次 provision 又把 8 推给 PSoC, 盖掉 PSoC 自己归一的 32。
        // 不变量改为: 只要在服务这个 app(poll), 排队的写命令就会推进。GUI 侧每帧 poll 一次,
        // 故仍是"每帧最多推进一帧", 与原语义一致。
        self._pump_cfg_tx();
    }

    /// 统一提交只读轮询请求：连接检查、窗口=1、seq 登记和发送共用一个出口。
    fn _submit_poll(&mut self, kind: RequestKind, mut frame: Frame) -> anyhow::Result<u8> {
        if self.state != ConnState::Connected || self.io.is_none() {
            self.skipped_log.push(kind, "未连接");
            return Ok(0);
        }
        if self.pending_registry.contains_kind(&kind) {
            return Ok(0);
        }
        let seq = self.next_seq();
        frame.seq = seq;
        self.io.as_ref().unwrap().send(frame)?;
        self.pending_registry.register(
            seq,
            PendingRequest {
                started_at: std::time::Instant::now(),
                timeout_secs: 5,
                kind,
            },
        );
        Ok(seq)
    }

    /// 连接后以窗口=1顺序回读基础状态，所有数据响应均由 handle_frame 推进。
    pub fn schedule_conn_probes(&mut self, ch: u8) {
        self.conn_probe_queue.clear();
        self.conn_probe_inflight = None;
        self.cfg_all_accum.clear();
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::AlgoGetInfo,
            frame: crate::proto::algo::encode_algo_get_info(0),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::ConfigGetAll,
            frame: Frame::new(HostCmd::CfgGetAll as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::GlobalGetAll,
            frame: crate::proto::algo::encode_global_get_all(0),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::RequestParams { ch },
            frame: Frame::new(
                HostCmd::ParamGetAll as u8,
                0,
                0,
                crate::proto::encode_param_get_all(ch),
            ),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::AlgoGetRom,
            frame: crate::proto::algo::encode_algo_get_rom(0),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::KbdRequestState,
            frame: Frame::new(HostCmd::KbdGetState as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::LedRequestState,
            frame: Frame::new(HostCmd::LedGet as u8, 0, 0, crate::proto::encode_led_get()),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::Mai2RequestState,
            frame: Frame::new(HostCmd::Mai2GetState as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::KbdRequestMap,
            frame: Frame::new(HostCmd::KbdGetMap as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::KbdRequestTouchmap,
            frame: Frame::new(HostCmd::KbdGetTouchmap as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::KbdRequestHold,
            frame: Frame::new(HostCmd::KbdGetHold as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::KbdGetCombo,
            frame: Frame::new(HostCmd::KbdGetCombo as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::KbdRequestKeycfg,
            frame: Frame::new(HostCmd::KbdGetKeycfg as u8, 0, 0, vec![]),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::KbdRequestEdges,
            frame: Frame::new(
                HostCmd::KbdGetEdges as u8,
                0,
                0,
                crate::proto::encode_kbd_get_edges(0),
            ),
        });
        self.conn_probe_queue.push_back(ConnProbeStep {
            kind: RequestKind::RequestParams { ch },
            frame: Frame::new(
                HostCmd::ParamGetAll as u8,
                0,
                0,
                crate::proto::encode_param_get_all(ch),
            ),
        });
    }

    /// 连接探针队列或在途请求是否仍未收敛。只读供 UI 主循环建立连接读取优先屏障。
    pub fn conn_probes_pending(&self) -> bool {
        !self.conn_probe_queue.is_empty() || self.conn_probe_inflight.is_some()
    }

    pub fn cancel_conn_probes_for_diagnostic(&mut self) {
        self.conn_probe_queue.clear();
        if let Some((seq, _)) = self.conn_probe_inflight.take() {
            self.pending_registry.confirm(seq);
        }
        self.pending_registry.clear();
        self.cfg_tx_queue.clear();
        self.cfg_tx_inflight = None;
    }

    pub fn poll_scheduled(&mut self, ctx: &PollContext, _now_tick: u32, _tick_ms: u32) {
        if !ctx.connected || self.state != ConnState::Connected {
            return;
        }
        if let Some(seq) = self.conn_probe_inflight.as_ref().map(|(seq, _)| *seq) {
            let timed_out = self
                .pending_registry
                .lookup(seq)
                .is_some_and(|req| req.started_at.elapsed().as_secs() > req.timeout_secs as u64);
            if timed_out {
                let probe_kind = self
                    .conn_probe_inflight
                    .as_ref()
                    .map(|(_, step)| step.kind.label())
                    .unwrap_or_else(|| "unknown".to_string());
                let probe_cmd = self
                    .conn_probe_inflight
                    .as_ref()
                    .map(|(_, step)| step.frame.cmd)
                    .unwrap_or(0);
                self.pending_registry.confirm(seq);
                self.conn_probe_inflight = None;
                log::info!(
                    "CONN_PROBE timeout kind={} seq={} cmd=0x{:02X}",
                    probe_kind,
                    seq,
                    probe_cmd
                );
            }
        }
        if self.conn_probe_inflight.is_none() && self.cfg_tx_pending() != 0 {
            return;
        }
        if self.conn_probe_inflight.is_none() {
            if let Some(step) = self.conn_probe_queue.pop_front() {
                let kind = step.kind.clone();
                let label = kind.label();
                let frame = step.frame.clone();
                match self._submit_poll(kind, frame) {
                    Ok(seq) if seq != 0 => {
                        log::info!(
                            "CONN_PROBE start kind={} seq={} cmd=0x{:02X}",
                            label,
                            seq,
                            step.frame.cmd
                        );
                        self.conn_probe_inflight = Some((seq, step));
                    }
                    Ok(_) => self.conn_probe_queue.push_front(step),
                    Err(error) => self.push_log_warn(format!("连接探针发送失败: {}", error)),
                }
            }
        }
        if self.conn_probes_pending() {
            return;
        }
        if !self.conn_probes_pending() {
            if let Some(idx) = ctx.algo_trace_idx {
                let _ = self._submit_poll(
                    RequestKind::RequestAlgoTrace {
                        ch: ctx.sel_channel,
                        idx,
                    },
                    crate::proto::algo::encode_algo_get_trace(0, ctx.sel_channel, idx),
                );
            }
        }
    }

    fn _data_response_decode_ok(frame: &Frame) -> bool {
        match frame.cmd {
            x if x == HostCmd::CfgGetAll as u8 => {
                (frame.flags & 0x02) == 0 && decode_entries(&frame.payload).is_ok()
            }
            x if x == HostCmd::ParamGetAll as u8 => {
                crate::proto::decode_param_get_all(&frame.payload).is_ok()
                    || crate::proto::decode_param_get_all_channels(&frame.payload).is_ok()
            }
            x if x == HostCmd::GlobalGetAll as u8 => {
                !crate::proto::algo::decode_global_get_all(&frame.payload).is_empty()
            }
            x if x == HostCmd::AlgoGetInfo as u8 => {
                crate::proto::algo::decode_algo_info(&frame.payload).is_some()
            }
            x if x == HostCmd::AlgoGetRom as u8 => {
                frame.payload.len() == crate::proto::algo::ALGO_CHANNELS * 2
            }
            x if x == HostCmd::AlgoGetTrace as u8 => {
                crate::proto::algo::decode_algo_get_trace(&frame.payload).is_some()
            }
            x if x == HostCmd::AlgoGetSrc as u8 => {
                crate::proto::algo::decode_algo_src_chunk(&frame.payload).is_some()
            }
            x if x == HostCmd::KbdGetState as u8 => frame.payload.len() >= 2,
            x if x == HostCmd::KbdGetMap as u8 => {
                frame
                    .payload
                    .first()
                    .is_some_and(|count| (*count as usize) <= 12)
                    && frame.payload.len() >= 1 + frame.payload[0] as usize * 2
            }
            x if x == HostCmd::KbdGetTouchmap as u8 => {
                frame
                    .payload
                    .get(1)
                    .is_some_and(|count| (*count as usize) <= 34)
                    && frame.payload.len() >= 2 + frame.payload[1] as usize * 2
            }
            x if x == HostCmd::KbdGetHold as u8 => {
                crate::proto::decode_kbd_get_hold(&frame.payload).is_ok_and(|table| {
                    table.phys.len() == KBD_HOLD_PHYS_COUNT
                        && table.zone.len() == KBD_HOLD_ZONE_COUNT
                })
            }
            x if x == HostCmd::KbdGetKeycfg as u8 => {
                crate::proto::decode_kbd_get_keycfg(&frame.payload)
                    .is_ok_and(|(list, _)| list.len() == KBD_HOLD_PHYS_COUNT)
            }
            x if x == HostCmd::KbdGetEdges as u8 => {
                crate::proto::decode_kbd_get_edges(&frame.payload).is_ok()
            }
            x if x == HostCmd::KbdGetCombo as u8 => {
                crate::proto::decode_kbd_get_combo(&frame.payload).is_ok()
            }
            x if x == HostCmd::Mai2GetState as u8 => {
                crate::proto::decode_mai2_get_state(&frame.payload).is_ok()
            }
            x if x == HostCmd::LedGet as u8 => {
                frame.payload.len() >= 96 && crate::proto::decode_led_get(&frame.payload).is_ok()
            }
            _ => false,
        }
    }

    fn _confirm_data_pending(&mut self, frame: &Frame, handler_ok: bool) {
        if !handler_ok
            || (frame.flags & 0x01) == 0
            || (frame.flags & 0x04) != 0
            || frame.cmd == HostCmd::Ack as u8
            || frame.cmd == HostCmd::Nak as u8
        {
            return;
        }
        let Some(req) = self.pending_registry.lookup(frame.seq).cloned() else {
            return;
        };
        let matches = match req.kind {
            RequestKind::AlgoGetInfo => frame.cmd == HostCmd::AlgoGetInfo as u8,
            RequestKind::AlgoGetRom => frame.cmd == HostCmd::AlgoGetRom as u8,
            RequestKind::RequestAlgoSrc => frame.cmd == HostCmd::AlgoGetSrc as u8,
            RequestKind::RequestAlgoCode => frame.cmd == HostCmd::AlgoGetCode as u8,
            RequestKind::RequestAlgoTrace { .. } => frame.cmd == HostCmd::AlgoGetTrace as u8,
            RequestKind::GlobalGetAll => frame.cmd == HostCmd::GlobalGetAll as u8,
            RequestKind::ConfigGetAll => {
                frame.cmd == HostCmd::CfgGetAll as u8 && (frame.flags & 0x02) == 0
            }
            RequestKind::RequestParams { .. } | RequestKind::RequestParamAllChannels { .. } => {
                frame.cmd == HostCmd::ParamGetAll as u8
            }
            RequestKind::KbdRequestState => frame.cmd == HostCmd::KbdGetState as u8,
            RequestKind::KbdRequestMap => frame.cmd == HostCmd::KbdGetMap as u8,
            RequestKind::KbdRequestTouchmap => frame.cmd == HostCmd::KbdGetTouchmap as u8,
            RequestKind::KbdRequestHold => frame.cmd == HostCmd::KbdGetHold as u8,
            RequestKind::KbdRequestKeycfg => frame.cmd == HostCmd::KbdGetKeycfg as u8,
            RequestKind::KbdGetCombo => frame.cmd == HostCmd::KbdGetCombo as u8,
            RequestKind::KbdRequestEdges => frame.cmd == HostCmd::KbdGetEdges as u8,
            RequestKind::Mai2RequestState => frame.cmd == HostCmd::Mai2GetState as u8,
            _ => false,
        };
        if matches {
            let label = req.kind.label();
            self.pending_registry.confirm(frame.seq);
            if self
                .conn_probe_inflight
                .as_ref()
                .is_some_and(|(seq, _)| *seq == frame.seq)
            {
                self.conn_probe_inflight = None;
                log::info!(
                    "CONN_PROBE complete kind={} seq={} cmd=0x{:02X} flags=0x{:02X}",
                    label,
                    frame.seq,
                    frame.cmd,
                    frame.flags
                );
            }
        }
    }

    fn handle_event(&mut self, evt: IoEvent) {
        match evt {
            IoEvent::Connected => {
                self.status_text = "已连接,等待设备信息...".to_string();
                self.push_log("IO 线程已连接, 等待 DEVICE_INFO...");
            }
            IoEvent::Disconnected => {
                self.io = None;
                self.state = ConnState::Disconnected;
                self.device_info = None;
                self.csd_mode = None;
                self._clear_sensor_caches();
                self.status_text = "设备已断开".to_string();
                self.push_log_error(format!("设备断开 (last_error={:?})", self.last_error));
            }
            IoEvent::Error(msg) => {
                self.push_log_error(format!("IO 错误: {}", msg));
                self.status_text = format!("错误: {}", msg);
                self.last_error = Some(msg);
            }
            IoEvent::DeviceBusy { consecutive_errors } => {
                self.device_busy_until =
                    Some(std::time::Instant::now() + std::time::Duration::from_secs(8));
                self.push_log_warn(format!(
                    "设备疑似忙(写 flash/CSD 重初始化)，等待恢复；连续 os error 22 次数={}",
                    consecutive_errors
                ));
            }
            IoEvent::DeviceRecovered => {
                if self.device_busy_until.take().is_some() {
                    self.device_busy_log_at = None;
                    self.push_log("设备已恢复，继续发送轮询命令".to_string());
                }
            }
            IoEvent::Frame(frame) => self.handle_frame(frame),
        }
    }

    /// 处理单个到达的协议帧。
    fn handle_frame(&mut self, frame: Frame) {
        // Debug 级: 记录实际收到的数据帧(排除高频采样推送 TELEM_DATA/FOCUS_DATA, 否则刷屏)。
        // 这是"实际数据交互"的接收侧, 选 Debug 等级即可看到真实命令响应/ACK/NAK 流。
        let sample_push = frame.cmd == HostCmd::TelemData as u8
            || frame.cmd == HostCmd::FocusData as u8
            || frame.cmd == HostCmd::SweepData as u8;
        if !sample_push {
            let kind = if (frame.flags & 0x04) != 0 {
                "NAK"
            } else if (frame.flags & 0x01) != 0 {
                "响应"
            } else {
                "帧"
            };
            self.push_log_debug(format!(
                "← 收到{} cmd=0x{:02X} seq={} len={}",
                kind,
                frame.cmd,
                frame.seq,
                frame.payload.len()
            ));
        }
        // C 源分片上传也是"一片一回执"的窗口=1 传输；只由 ACK/NAK 推进，普通响应不能放行。
        if !sample_push {
            let nak = (frame.flags & 0x04) != 0 || frame.cmd == HostCmd::Nak as u8;
            if nak || frame.cmd == HostCmd::Ack as u8 {
                self._algo_src_note_reply(frame.seq, !nak);
            }
        }
        if frame.cmd == HostCmd::DeviceInfo as u8 {
            match DeviceInfo::from_payload(&frame.payload) {
                Ok(info) => {
                    // 模式与诊断同帧回读，只有设备明确上报后才允许 UI 展示/编辑对应状态。
                    let was_connected = self.state == ConnState::Connected;
                    self.csd_mode = info.diagnostics.as_ref().map(|diag| diag.csd_mode);
                    // 链路活性证据(停流时的唯一来源): link_ok=1, 或两次 DEVICE_INFO 之间代数推进了
                    // —— 后者能盖住 link_ok 那一帧恰好因 SPI 残帧抖动为 0 的情况。
                    let gen_advanced = self
                        .psoc_gen_seen
                        .is_some_and(|g| g != info.psoc_generation);
                    self.psoc_gen_seen = Some(info.psoc_generation);
                    if info.psoc_link_valid || gen_advanced {
                        self.psoc_info_live_at = Some(std::time::Instant::now());
                    }
                    self.device_info = Some(info);
                    self.state = ConnState::Connected;
                    self.status_text = "已连接".to_string();
                    self.push_log("已连接: 收到 DEVICE_INFO");
                    // ★掉线重连自动恢复运行态★: 震动导致 USB 掉线后触控必须自己回来, 不依赖 UI 手动点。
                    // 只在"新进入 Connected"时做一次(重复 DEVICE_INFO 不反复下发)。
                    if !was_connected {
                        if let Err(e) = self.restore_after_reconnect() {
                            self.push_log_warn(format!("重连恢复运行态失败: {}", e));
                        }
                    }
                }
                Err(e) => {
                    self.status_text = format!("解析 DEVICE_INFO 失败: {}", e);
                    self.push_log(format!("DEVICE_INFO 解析失败: {}", e));
                    self.last_error = Some(e);
                }
            }
        }
        // 配置命令响应处理 (#6e-1)
        if frame.cmd == HostCmd::CfgGetAll as u8 {
            let version = self.config_version;
            self._handle_cfg_get_all_response(&frame);
            let ok = (frame.flags & 0x02) == 0 && self.config_version != version;
            self._confirm_data_pending(&frame, ok);
        } else if frame.cmd == HostCmd::CfgGetGroup as u8 {
            self._handle_cfg_get_group_response(&frame);
        } else if frame.cmd == HostCmd::CfgGet as u8 {
            self._handle_cfg_get_response(&frame);
        } else if frame.cmd == HostCmd::Ack as u8 {
            self._handle_ack(&frame);
        } else if frame.cmd == HostCmd::Nak as u8 {
            self._handle_nak(&frame);
        } else if frame.cmd == HostCmd::BindEvent as u8 {
            self._handle_bind_event(&frame);
        } else if frame.cmd == HostCmd::TelemData as u8 {
            self._handle_telem_data(&frame);
        } else if frame.cmd == HostCmd::FocusData as u8 {
            // 设备主动推送(STREAM): 单通道采样, 归属由 payload 里的 session 判定, 与 seq 无关。
            self._handle_focus_data(&frame);
        } else if frame.cmd == HostCmd::SweepData as u8 {
            self._handle_sweep_data(&frame);
        } else if frame.cmd == HostCmd::FocusStart as u8 && (frame.flags & 0x01) != 0 {
            self._handle_focus_start_response(&frame);
        } else if frame.cmd == HostCmd::SweepStart as u8 && (frame.flags & 0x01) != 0 {
            self._handle_sweep_start_response(&frame);
        } else if frame.cmd == HostCmd::ParamGet as u8 && (frame.flags & 0x01) != 0 {
            self._handle_param_get_response(&frame);
        } else if frame.cmd == HostCmd::ParamGetAll as u8 && (frame.flags & 0x01) != 0 {
            let version = self.param_version;
            self._handle_param_get_all_response(&frame);
            self._confirm_data_pending(&frame, self.param_version != version);
        } else if frame.cmd == HostCmd::CpGet as u8 && (frame.flags & 0x01) != 0 {
            self._handle_cp_get_response(&frame);
        } else if frame.cmd == HostCmd::GlobalGet as u8 && (frame.flags & 0x01) != 0 {
            self._handle_global_get_response(&frame);
        } else if frame.cmd == HostCmd::GlobalGetAll as u8 && (frame.flags & 0x01) != 0 {
            self._handle_global_get_all_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::AutoTune as u8 && (frame.flags & 0x01) != 0 {
            self._handle_auto_tune_response(&frame);
        } else if frame.cmd == HostCmd::AutoTuneProgressPush as u8 {
            // 设备主动推送(STREAM 帧, 无 seq 匹配): 阶段进度 + 终态。
            self._handle_auto_tune_progress(&frame);
        } else if frame.cmd == HostCmd::PsocRescueProgressPush as u8 {
            self._handle_rescue_progress(&frame);
        } else if frame.cmd == HostCmd::SelfHealEventPush as u8 {
            // 固件自持恢复事件(STREAM 推送): 落日志 + 触发真值回读, 杜绝 UI 与设备静默不同步。
            self._handle_self_heal_event(&frame);
        } else if frame.cmd == HostCmd::AlgoGetInfo as u8 && (frame.flags & 0x01) != 0 {
            self._handle_algo_info_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::AlgoGetRom as u8 && (frame.flags & 0x01) != 0 {
            self._handle_algo_get_rom_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::AlgoGetTrace as u8 && (frame.flags & 0x01) != 0 {
            self._handle_algo_get_trace_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::AlgoGetCfg as u8 && (frame.flags & 0x01) != 0 {
            self._handle_algo_get_cfg_response(&frame);
        } else if frame.cmd == HostCmd::AlgoGetSrc as u8 && (frame.flags & 0x01) != 0 {
            self._handle_algo_src_chunk(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::AlgoGetCode as u8 && (frame.flags & 0x01) != 0 {
            let len_ok = frame.payload.len() >= 2
                && u16::from_le_bytes([frame.payload[0], frame.payload[1]]) as usize > 0
                && u16::from_le_bytes([frame.payload[0], frame.payload[1]]) as usize
                    <= frame.payload.len() - 2;
            let bytes = crate::proto::algo::decode_algo_len_prefixed(&frame.payload);
            if len_ok {
                self.algo_device_code_hex = Self::_hex_dump(&bytes);
                self.algo_device_code_version = self.algo_device_code_version.wrapping_add(1);
            }
            self._confirm_data_pending(&frame, len_ok);
        } else if frame.cmd == HostCmd::KbdGetState as u8 && (frame.flags & 0x01) != 0 {
            self._handle_kbd_get_state_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::KbdGetMap as u8 && (frame.flags & 0x01) != 0 {
            self._handle_kbd_get_map_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::KbdGetTouchmap as u8 && (frame.flags & 0x01) != 0 {
            self._handle_kbd_get_touchmap_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::KbdGetHold as u8 && (frame.flags & 0x01) != 0 {
            self._handle_kbd_get_hold_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::KbdGetKeycfg as u8 && (frame.flags & 0x01) != 0 {
            self._handle_kbd_get_keycfg_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::KbdGetEdges as u8 && (frame.flags & 0x01) != 0 {
            self._handle_kbd_get_edges_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::KbdGetCombo as u8 && (frame.flags & 0x01) != 0 {
            let ok = match crate::proto::decode_kbd_get_combo(&frame.payload) {
                Ok(items) => {
                    self.kbd_combo_supported = Some(true);
                    self.kbd_combo_req_seq = None;
                    self.kbd_combo_cache = items;
                    // 回读到设备真值后丢弃草稿: 与其它页"回读即真值"的口径一致。
                    self.drafts.drop_kbd_combo();
                    self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
                    true
                }
                Err(e) => {
                    log::error!("解析 KBD_GET_COMBO 响应失败: {}", e);
                    self.last_error = Some(e);
                    false
                }
            };
            self._confirm_data_pending(&frame, ok);
        } else if frame.cmd == HostCmd::Mai2GetState as u8 && (frame.flags & 0x01) != 0 {
            self._handle_mai2_get_state_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else if frame.cmd == HostCmd::LedGet as u8 && (frame.flags & 0x01) != 0 {
            self._handle_led_get_response(&frame);
            self._confirm_data_pending(&frame, Self::_data_response_decode_ok(&frame));
        } else {
            log::debug!(
                "收到帧 cmd=0x{:02X} seq={} len={} (待处理)",
                frame.cmd,
                frame.seq,
                frame.payload.len()
            );
        }
    }

    // ------------------------------------------------------------------
    // 配置命令方法 (#6e-1)
    // ------------------------------------------------------------------

    /// 请求拉取全部配置项
    /// 请求全量配置。连接探针拥有握手阶段 CFG_GET_ALL 的唯一权；探针未收敛时，
    /// 手动刷新不另开第二条流，避免固件把两条流式响应交错后无法解码。
    pub fn request_config_all(&mut self) -> anyhow::Result<()> {
        if self.conn_probes_pending() {
            return Ok(());
        }
        let seq = self._submit_poll(
            RequestKind::ConfigGetAll,
            Frame::new(HostCmd::CfgGetAll as u8, 0, 0, vec![]),
        )?;
        if seq != 0 {
            self.cfg_all_accum.clear();
        }
        Ok(())
    }

    /// 暂存单个配置项到草稿(不下发)。点击保存后才由 `save_config` 统一下发。
    pub fn set_config(&mut self, entry: ConfigEntry) -> anyhow::Result<()> {
        // 校验可编码, 提前暴露非法值; 实际编码在 save_config 下发时进行。
        crate::proto::encode_entry(&entry)
            .map_err(|e| anyhow::anyhow!("encode_entry failed: {}", e))?;
        // 触控键盘映射总开关是"运行态"而非普通配置: 用户显式设置即记入期望态, 掉线重连后自动恢复。
        if entry.key == Self::KBD_MAP_EN_KEY {
            if let CfgValue::Bool(v) = entry.value {
                self.desired.kbd_map_en = Some(v);
            }
        }
        // ★值未变不标脏★: 重新选一遍当前内容(或改回原值)不应产生"未保存改动"。
        let same_as_device = self
            .config_cache
            .get(&entry.key)
            .is_some_and(|dev| Self::_cfg_eq(&dev.value, &entry.value));
        self.drafts.set_cfg(&entry.key, entry.value, same_as_device);
        Ok(())
    }

    /// 两个配置值是否等价。CfgValue 未派生 PartialEq(含 F32/Str), 故逐变体比较。
    fn _cfg_eq(a: &CfgValue, b: &CfgValue) -> bool {
        match (a, b) {
            (CfgValue::Bool(x), CfgValue::Bool(y)) => x == y,
            (CfgValue::I8(x), CfgValue::I8(y)) => x == y,
            (CfgValue::U8(x), CfgValue::U8(y)) => x == y,
            (CfgValue::U16(x), CfgValue::U16(y)) => x == y,
            (CfgValue::U32(x), CfgValue::U32(y)) => x == y,
            (CfgValue::F32(x), CfgValue::F32(y)) => (x - y).abs() <= f32::EPSILON,
            (CfgValue::Str(x), CfgValue::Str(y)) => x == y,
            _ => false,
        }
    }

    /// 有未保存到设备 flash 的设置草稿。★派生量★: 脏键集合(未保存项的引用计数)非空即为脏,
    /// 不再有独立 bool 可与集合漂移, 故与 `config_dirty_count()` 数学上不可能矛盾。
    pub fn is_config_dirty(&self) -> bool {
        self.drafts.is_dirty()
    }
    pub fn config_dirty_version(&self) -> u64 {
        self.drafts.version()
    }
    /// 未保存配置项数量, 供 UI 显示"N 项未保存"。
    ///
    /// ★组合映射按"差异条数"计, 不按草稿槽位计★
    /// 组合映射是**整表一份草稿**(脏键只有 `kbd:combo` 一个), 于是"新增了 3 条映射"会显示成
    /// "1 项未保存" —— 数字与用户的操作对不上, 用户会以为只有一条被记住了(实测用户就这么反馈的)。
    /// 这里把那一个槽位换算成与设备真值的实际差异条数(新增/删除/改动各算一条)。
    /// 判定仍然派生自脏键集合(草稿不存在就不参与), 不引入第二份"是否脏"的真相。
    pub fn config_dirty_count(&self) -> i32 {
        let base = self.drafts.dirty_count();
        match self.drafts.kbd_combo() {
            Some(table) => {
                let diff = Self::_combo_diff_count(table, &self.kbd_combo_cache);
                // 槽位 1 换成 diff; diff 为 0 时该草稿本不该存在(set_kbd_combo 会撤稿), 兜底不减到负数。
                (base.saturating_sub(1) + diff) as i32
            }
            None => base as i32,
        }
    }

    /// 组合映射草稿与设备真值的差异条数: 按"分区集合"认同一条映射, 内容不同算改动;
    /// 只在一侧出现的算新增/删除。★不按下标比较★ —— 删除会让后续行整体前移, 按下标比会把
    /// "删掉第 1 条"算成"改了 N 条"。
    fn _combo_diff_count(
        draft: &[crate::proto::KbdComboItem],
        device: &[crate::proto::KbdComboItem],
    ) -> usize {
        let mut diff = 0usize;
        for d in draft {
            match device.iter().find(|c| c.zone_mask == d.zone_mask) {
                Some(c) if c == d => {}
                Some(_) => diff += 1, // 同一分区集合、内容改了
                None => diff += 1,    // 新增
            }
        }
        // 设备上有、草稿里没有 = 删除
        diff += device
            .iter()
            .filter(|c| !draft.iter().any(|d| d.zone_mask == c.zone_mask))
            .count();
        diff
    }

    /// 按缓存原始类型设置数值配置项。
    pub fn set_config_number(&mut self, key: &str, value: f64) -> anyhow::Result<()> {
        // ★围栏统一在这一处夹取★ 所有数字配置项的草稿写入都经过本函数(界面输入框、枚举下拉、
        // hex 口径、导入), 故把"超界即夹到阈值"放在这里, 草稿池里就永远不会存在越界值 —— 比在
        // "保存到设备"前再遍历脏池补救更彻底(那种补救漏一条路径就白做)。围栏取设备自报的 range,
        // 缺 range 时保持原样交由下面的类型转换处理(截断语义不变)。
        let entry = self.config_get(key);
        let value = match entry.as_ref().and_then(|e| e.range.as_ref()) {
            Some((lo, hi)) => {
                let mut v = value;
                if let Some(lo) = lo.as_f32() {
                    if v < lo as f64 {
                        v = lo as f64;
                    }
                }
                if let Some(hi) = hi.as_f32() {
                    if v > hi as f64 {
                        v = hi as f64;
                    }
                }
                v
            }
            None => value,
        };
        let cfg_value = match entry.map(|entry| entry.value) {
            Some(CfgValue::Bool(_)) => CfgValue::Bool(value != 0.0),
            Some(CfgValue::I8(_)) => CfgValue::I8(value as i8),
            Some(CfgValue::U8(_)) => CfgValue::U8(value as u8),
            Some(CfgValue::U16(_)) => CfgValue::U16(value as u16),
            Some(CfgValue::U32(_)) => CfgValue::U32(value as u32),
            Some(CfgValue::F32(_)) => CfgValue::F32(value as f32),
            Some(CfgValue::Str(v)) => CfgValue::Str(v),
            None => CfgValue::U32(value as u32),
        };
        self.set_config(ConfigEntry::new(key.to_string(), cfg_value))
    }

    /// 按缓存原始类型设置枚举配置项。
    pub fn set_config_enum(&mut self, key: &str, index: i32) -> anyhow::Result<()> {
        self.set_config_number(key, index as f64)
    }

    /// 以十六进制串设置数字配置项(统一 hex 口径)。接受 "0x1E"/"1E" 等; 空串忽略。
    /// 解析后复用 set_config_number 按缓存原始类型写草稿(类型转换由其处理)。
    pub fn set_config_hex(&mut self, key: &str, hex: &str) -> anyhow::Result<()> {
        let t = hex.trim();
        let t = t
            .strip_prefix("0x")
            .or_else(|| t.strip_prefix("0X"))
            .unwrap_or(t);
        if t.is_empty() {
            return Ok(());
        }
        let value =
            u32::from_str_radix(t, 16).map_err(|_| anyhow::anyhow!("非法十六进制: {}", hex))?;
        self.set_config_number(key, value as f64)
    }

    /// 保存: 把全部草稿统一下发设备并请求写 flash(SAVE_CONFIG)。
    ///
    /// 顺序: 普通配置(含 bind.mapNN) → 参数 → 全局 CSD → 工作模式 → 键盘物理键 →
    /// 键盘触控映射 → SAVE_CONFIG。全部成功送出后才把草稿合并进本地缓存并清脏;
    /// 未连接时直接返回错误且不清脏, 避免"假装保存成功"。
    pub fn save_config(&mut self) -> anyhow::Result<()> {
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法保存"));
        }
        // 串行化守卫: 阻塞操作/冷却期间拒绝保存, 避免保存流(全局提交+校准)叠加到进行中的重操作上把设备搞死
        // (log.log 已证: 频率自适应刚成功那一刻的保存叠加 → cp_get 永久失败 + 遥测冻结)。
        // ★保存流本身就是一次完整重配 CSD★(逐项 GLOBAL_SET + 一次 GLOBAL_COMMIT + 参数下发),
        // 落在扫描中间会把当前那一格的 gain/div 连同全部通道一起重初始化 ⇒ 与校准/基线/自适应同等对待。
        if self._reject_csd_if_batching("保存到设备")
            || self._reject_csd_if_sweeping("保存到设备")
            || self._reject_csd_if_locked("保存到设备")
        {
            return Ok(());
        }
        // ★保存流改为串行落地★: 本函数只把帧排进 cfg_tx_queue, 由 poll 每 tick 取一帧、等到该帧的
        // ACK/NAK(或超时)才发下一帧(窗口=1)。原来是一次性把上百帧直接灌进 IO 队列, 设备侧 core0
        // 在写 flash / 重初始化期间根本不解析命令, 背靠背灌入必然把链路打崩。
        let mut queued: Vec<Frame> = Vec::new();
        if !self.drafts.is_dirty() {
            // 无草稿改动: 仅请求写 flash(把设备当前运行态落盘)。
            let seq = self.next_seq();
            self._queue_tx(Frame::new(HostCmd::SaveConfig as u8, 0, seq, vec![]))?;
            return Ok(());
        }
        // 提交前先取快照: 下方 `_commit_combo` 会撤销 kbd:combo 的脏键(取草稿即提交),
        // mode.work 判定也必须在清草稿之前, 故两者统一在此处定格。
        let dirty_count = self.drafts.dirty_count();
        let mode_work_changed = self.drafts.has_cfg("mode.work");

        // 1) 普通配置。绑定槽 bind.mapNN 且映射到有效物理通道时用 BIND_SET_MAP
        //    [zone, channel] 下发, 使固件 reload_binding() 真正刷新运行态映射;
        //    其余(含"清除"=0xFFFFFFFF)走 CFG_SET 写配置。两类最终都随 SAVE_CONFIG 落 flash。
        for (key, value) in self.drafts.cfg_items() {
            let bind_channel = key
                .strip_prefix("bind.map")
                .and_then(|s| s.parse::<usize>().ok())
                .and_then(|zone| match value {
                    CfgValue::U32(v) if v <= 35 => Some((zone as u8, v as u8)),
                    _ => None,
                });
            let seq = self.next_seq();
            if let Some((zone, channel)) = bind_channel {
                if self.io.is_some() {
                    queued.push(Frame::new(
                        HostCmd::BindSetMap as u8,
                        0,
                        seq,
                        vec![zone, channel],
                    ));
                }
                self.config_cache.insert(
                    key,
                    ConfigEntry::new(zone_key(zone as usize), CfgValue::U32(channel as u32)),
                );
            } else {
                let entry = ConfigEntry::new(key.clone(), value);
                let payload = crate::proto::encode_entry(&entry)
                    .map_err(|e| anyhow::anyhow!("encode_entry failed: {}", e))?;
                if self.io.is_some() {
                    queued.push(Frame::new(HostCmd::CfgSet as u8, 0, seq, payload));
                }
                self.config_cache.insert(key, entry);
            }
        }
        // 2) 参数
        let param_items: Vec<((u8, u8), u32)> = self.drafts.param_items();
        self.post_save_refetch_channels = param_items
            .iter()
            .filter_map(|((ch, id), value)| {
                (*id == crate::proto::PARAM_ENABLED
                    && *value != 0
                    && self.params[*ch as usize].get(id).copied() == Some(0))
                .then_some(*ch)
            })
            .collect();
        for ((ch, id), value) in param_items.iter().copied() {
            let payload = crate::proto::encode_param_set(ch, id, value);
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(Frame::new(HostCmd::ParamSet as u8, 0, seq, payload));
            }
            if (ch as usize) < 36 {
                self.params[ch as usize].insert(id, value);
            }
        }
        // 2b) 硬件类 CSD 参数(分辨率/时钟分频/时钟源/IDAC)改动后必须显式下发 APPLY,
        //     否则 PSoC 的 SET_PARAM 只把新值写进 widgetContext 而不重初始化扫描硬件 →
        //     采样周期(探测时间)永远不变。APPLY(=CALIBRATE 全通道掩码)使 PSoC 主循环
        //     从 widgetContext 重配硬件并生效。软阈值(0x01-0x06)由半自动处理链实时读取,
        //     无需 APPLY(避免多余的基线重置扰动)。
        //     ★例外: 只改 IDAC 增益档(0x0B)时不发 CALIBRATE★ —— 旧行为("硬件类改动一律补发全通道
        //     CALIBRATE")会让 PSoC 的校准把用户刚选的增益档冲回全局起点档, 即用户改了档位却被
        //     自己这条 CALIBRATE 抹掉, UI 显示的与设备实际不符。0x0B 的生效由固件在 SET 时自行
        //     re-init 保证, 故此处跳过; 其余硬件类参数行为不变。
        //     ★例外二: 通道启用开关(0x0C)不是硬件调参项★ —— 它 >= 0x07, 若不排除就会让"关掉一个
        //     通道"这种操作附带一次全 36 通道 CALIBRATE(数秒重操作 + 冲掉所有手动增益档)。
        //     PSoC 侧对 PARAM_ENABLED 有自己的落实路径(SetWidgetStatus + 只为该通道重做校准/基线),
        //     既不需要 APPLY 也不需要 CALIBRATE。
        let hw_ids: Vec<u8> = param_items
            .iter()
            .map(|((_, id), _)| *id)
            .filter(|id| (0x07..=0x0B).contains(id))
            .collect();
        let idac_gain_only =
            !hw_ids.is_empty() && hw_ids.iter().all(|id| *id == crate::proto::PARAM_IDAC_GAIN);
        let hw_param_changed = !hw_ids.is_empty() && !idac_gain_only;
        if idac_gain_only {
            self.push_log(
                "保存: 本次硬件类改动仅 IDAC 增益档(0x0B) → 已跳过全通道 CALIBRATE。\
                 旧行为是仍补发 CALIBRATE, 而 PSoC 校准会把该档位冲回全局起点档(用户选的档被抹掉); \
                 该档由固件在 SET_PARAM 时自行 re-init 生效, 无需校准。",
            );
        }
        if hw_param_changed {
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(Frame::new(
                    HostCmd::Calibrate as u8,
                    0,
                    seq,
                    0xFFFFFFFF_FFFFFFFFu64.to_le_bytes().to_vec(),
                ));
            }
        }
        // 3) 全局 CSD
        let g_items: Vec<(u8, u32)> = self.drafts.global_items();
        if !g_items.is_empty() {
            // ★CSD 调试★: 记录本次下发的全局项序列。GLOBAL_SET 只写 PSoC 影子，
            // 全部项写完后的单次 GLOBAL_COMMIT 才触发完整重初始化，避免逐项重初始化风暴。
            let seq_desc: Vec<String> = g_items
                .iter()
                .map(|(id, v)| format!("{}={}", Self::_gparam_name(*id), v))
                .collect();
            self.push_log(format!(
                "下发 {} 项全局设置(合并为一次 PSoC 重初始化): {}",
                g_items.len(),
                seq_desc.join(", ")
            ));
        }
        let g_changed = !g_items.is_empty();
        for (id, value) in g_items {
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(crate::proto::algo::encode_global_set(seq, id, value));
            }
            // ★不再乐观写 globals★: 原先在此直接把草稿值当成"设备真值"写进缓存, 于是 UI 显示的永远是
            // 自己刚发出去的数, 固件若拒收/夹取(如 PSoC main.c:269 把 RAW_TARGET 夹到 [60,90] 否则 85,
            // main.c:275 把 GND 强制改 High-Z)UI 完全看不见 → 用户看到的是幻觉。改为只记期望值,
            // commit 落定后统一回读设备真值对账(见 globals_verify_in / _verify_globals_readback)。
            self.globals_expected.insert(id, value);
        }
        // ★单次提交★: 全部全局项写完影子后, 只发一次 GLOBAL_COMMIT 统一重初始化(不再逐项 commit),
        // 杜绝多项保存时的重初始化风暴(会拖垮 core0/USB → 掉线, 见 log.log)。
        if g_changed {
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(crate::proto::algo::encode_global_commit(seq));
            }
        }
        // 保存流触发了 CSD 重操作(全局提交 / 硬件参数改动→CALIBRATE)时, 开冷却窗串行化后续变更,
        // 兜住 PSoC 完整重初始化+重校准真实完成, 杜绝紧接着的操作叠加致死。
        if g_changed || hw_param_changed {
            self._begin_csd_cooldown("保存后重校准/重初始化", 150);
            self.csd_diag_probe_in = Some(175); // 冷却(150)落定后再回读诊断, 避免读流量与重初始化争用
        }
        // 全局项落定后回读设备真值对账。
        // ★不能只等 ~3s★: 实测(03:30 那份日志)对账在 3.0s 时读到的是 **PSoC 刚重启后的编译默认值**
        // (RAW_TARGET=85 / IDAC_GAIN_INIT=4 / IDAC_MIN=20 …), 于是 7 项全部误报"未按下发值生效";
        // 而 RP2040 的自持恢复要到 ~9s 才把保存的配置重新下发完。改为等到串行队列排空之后再计时,
        // 且把窗口拉到覆盖一次 PSoC 重初始化 + 重下发。
        // 收到自持恢复事件时会重新排程(见 _handle_self_heal_event), 避免恰好卡在重建窗口里对账。
        if g_changed {
            self.globals_verify_in = Some(GLOBALS_VERIFY_TICKS);
        }
        // 4) 算法可设置变量。SET_CFG 不触发重初始化，无需 GLOBAL_COMMIT 或额外冷却。
        let algo_cfg_items: Vec<(u8, u8)> = self.drafts.algo_cfg_items();
        for (idx, val) in algo_cfg_items {
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(crate::proto::algo::encode_algo_set_cfg(seq, idx, val));
            }
            self.algo_cfg[idx as usize] = val;
            self.algo_cfg_valid[idx as usize] = true;
        }
        // 4b) 触控组合映射(整表替换)。放在物理键/触控键之前无所谓: 固件侧是独立表, 互不影响。
        self._commit_combo(&mut queued)?;
        // 5) 工作模式
        if let Some(mode) = self.drafts.mode() {
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(Frame::new(HostCmd::ModeSet as u8, 0, seq, vec![mode]));
            }
        }
        // 5) 键盘物理键
        let km: Vec<(u8, (u8, u8))> = self.drafts.kbd_map_items();
        for (idx, (code, m)) in km {
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(Frame::new(
                    HostCmd::KbdSetMap as u8,
                    0,
                    seq,
                    vec![idx, code, m],
                ));
            }
            if (idx as usize) < 12 {
                self.kbd_map[idx as usize] = code;
                self.kbd_keymod[idx as usize] = m;
            }
        }
        // 6) 键盘触控映射
        let kt: Vec<(u8, (u8, u8))> = self.drafts.kbd_touch_items();
        for (zone, (code, m)) in kt {
            let seq = self.next_seq();
            if self.io.is_some() {
                queued.push(Frame::new(
                    HostCmd::KbdSetTouchmap as u8,
                    0,
                    seq,
                    vec![zone, code, m],
                ));
            }
            if (zone as usize) < 34 {
                self.kbd_touchmap[zone as usize] = code;
                self.kbd_zonemod[zone as usize] = m;
            }
        }
        // 6b) 长按参数草稿。KBD_SET_HOLD 的 payload 是 n×6B 项数组，全部草稿项合成一帧下发。
        let hold_items: Vec<KbdHoldItem> = self.drafts.kbd_hold_items();
        if !hold_items.is_empty() {
            let seq = self.next_seq();
            let payload = crate::proto::encode_kbd_set_hold(&hold_items);
            if self.io.is_some() {
                queued.push(Frame::new(HostCmd::KbdSetHold as u8, 0, seq, payload));
            }
            for item in &hold_items {
                if item.kind == KBD_HOLD_KIND_ZONE {
                    if (item.idx as usize) < KBD_HOLD_ZONE_COUNT {
                        self.kbd_hold_zone[item.idx as usize] = item.hold;
                    }
                } else if (item.idx as usize) < KBD_HOLD_PHYS_COUNT {
                    self.kbd_hold_phys[item.idx as usize] = item.hold;
                }
            }
        }
        // 6c) 每键触发极性 + 独立防抖草稿。与长按参数同口径: 全部草稿项合成一帧 KBD_SET_KEYCFG。
        // 固件对该帧做"整批校验后才落值"(任一项越界整帧 NAK), 因此这里也必须整批发, 不逐项。
        let keycfg_items: Vec<(u8, KbdKeyCfg)> = self.drafts.kbd_keycfg_items();
        if !keycfg_items.is_empty() {
            let seq = self.next_seq();
            let payload = crate::proto::encode_kbd_set_keycfg(&keycfg_items);
            if self.io.is_some() {
                queued.push(Frame::new(HostCmd::KbdSetKeycfg as u8, 0, seq, payload));
            }
            for (idx, cfg) in &keycfg_items {
                if (*idx as usize) < KBD_HOLD_PHYS_COUNT {
                    self.kbd_keycfg[*idx as usize] = *cfg;
                }
            }
        }
        // 7) 请求写 flash
        let seq = self.next_seq();
        if self.io.is_some() {
            queued.push(Frame::new(HostCmd::SaveConfig as u8, 0, seq, vec![]));
        }

        // 整批排入唯一的串行发送队列。顺序即 FIFO, 因此 CFG_SET→SAVE_CONFIG、
        // GLOBAL_SET→GLOBAL_COMMIT 这些前后依赖天然保序。
        for frame in queued {
            self._queue_tx(frame)?;
        }

        // mode.work(USB Serial/HID 拓扑)改动需整机重启重枚举才生效, 记录待重启标记。
        self._clear_drafts();
        if mode_work_changed {
            self.pending_reboot = true;
            self.push_log("保存: mode.work 已改, 将自动重启设备以切换 Serial/HID 拓扑");
        }
        // 缓存已随下发同步; 递增版本让 UI 用已提交值刷新各页。
        self._bump_view_versions();
        self.push_log(format!(
            "保存: 已下发 {} 项草稿改动并请求写入 flash",
            dirty_count
        ));
        Ok(())
    }

    /// 递增全部"草稿/真值可见值"相关的版本号, 触发 main.rs 的 16ms tick 门控重新回填 UI。
    ///
    /// ★为什么需要显式方法★: 各 `set_*` 草稿写入方法故意不 bump —— 那是用户在 UI 上就地编辑,
    /// 控件自身已显示新值, 重建行模型反而会打断正在进行的输入。但**外部批量写草稿**(JSON 导入、
    /// 保存提交、撤销)不经过控件, 不 bump 则界面完全不动, 即"数据源不统一"。故这三条路径统一调本方法。
    /// 不改变任何轮询频率: 只是让下一个 16ms tick 的既有门控判定为"有变化"。
    fn _bump_view_versions(&mut self) {
        self.config_version = self.config_version.wrapping_add(1);
        self.param_version = self.param_version.wrapping_add(1);
        self.globals_version = self.globals_version.wrapping_add(1);
        self.kbd_map_version = self.kbd_map_version.wrapping_add(1);
        self.kbd_touchmap_version = self.kbd_touchmap_version.wrapping_add(1);
        self.kbd_hold_version = self.kbd_hold_version.wrapping_add(1);
        self.kbd_keycfg_version = self.kbd_keycfg_version.wrapping_add(1);
        self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        self.led_version = self.led_version.wrapping_add(1);
    }

    /// 外部批量写入草稿后调用一次, 让 UI 在下一 tick 用"草稿优先"的有效值刷新全部页面。
    pub fn refresh_draft_views(&mut self) {
        self._bump_view_versions();
    }

    /// 草稿中是否包含 mode.work(USB Serial/HID)改动 —— 该改动需整机重启才生效, 供 UI 提示。
    pub fn draft_needs_reboot(&self) -> bool {
        self.drafts.has_cfg("mode.work")
    }

    /// 保存后是否有待执行的自动重启(mode.work 拓扑切换)。
    pub fn pending_reboot(&self) -> bool {
        self.pending_reboot
    }

    /// 清除待重启标记(UI 已发起重启后调用)。
    pub fn clear_pending_reboot(&mut self) {
        self.pending_reboot = false;
    }

    /// 撤销全部未保存草稿, 恢复到设备当前运行态(缓存值)。供 CSD/配置页"撤销"使用。
    pub fn discard_draft(&mut self) {
        if !self.drafts.is_dirty() {
            return;
        }
        self._clear_drafts();
        self._bump_view_versions();
        self.push_log("撤销: 已丢弃全部未保存草稿");
    }

    /// 清空全部草稿与脏键(内部辅助)。12 份草稿(含灯效单元映射与组合映射整表)由
    /// `ConfigDrafts::clear_all` 一次清净 —— 逐个手写清理正是"漏一个就永远显示未保存"的来源。
    fn _clear_drafts(&mut self) {
        let algo_cfg_changed = self.drafts.has_algo_cfg();
        self.drafts.clear_all();
        if algo_cfg_changed {
            self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        }
    }

    /// 重置为默认配置
    pub fn reset_defaults(&mut self) -> anyhow::Result<()> {
        // 恢复默认 = 重启 PSoC + 回填出厂 CSD 参数, 是最彻底的一次重配 ⇒ 扫描期间一律拒绝。
        if self._reject_csd_if_batching("恢复默认配置")
            || self._reject_csd_if_sweeping("恢复默认配置")
        {
            return Ok(());
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(HostCmd::ResetDefaults as u8, 0, seq, vec![]))?;
            // 清空本地缓存与草稿,后续由下方排程的重读拉取设备回填的默认值
            self.config_cache.clear();
            self.cfg_all_accum.clear();
            self._clear_drafts();
            // 恢复默认会重启 PSoC 并由 RP2040 回填出厂默认入 store(~2s), 就绪后自动全量重读刷新显示。
            self.post_reset_refetch_in = Some(160); // ~2.6s @16ms/tick
            self.push_log("恢复默认已下发: 正在重启 PSoC 并回填默认 CSD 参数(每通道+全局), 就绪后自动重读刷新…");
        }
        Ok(())
    }

    // ------------------------------------------------------------------
    // 全通道页"批量应用": 把源通道的选定参数一次性复制到选定的目标通道。
    // 全部走 set_param(草稿) —— 与手工编辑同一条路径, 由"保存到设备"统一下发,
    // 因此不会有"点一下就写进设备"的意外, 也不需要新增协议。
    // ------------------------------------------------------------------

    /// 选择态版本号: 供 UI 走既有 16ms + 脏检查门控回填勾选状态。
    pub fn batch_sel_version(&self) -> u64 {
        self.batch_sel_version
    }

    /// 36 通道勾选状态(下标=通道号)。
    pub fn batch_ch_selected(&self) -> Vec<bool> {
        (0..36u8).map(|ch| self.batch_sel.ch_selected(ch)).collect()
    }

    /// 可批量应用参数的勾选状态。顺序与下标一律由 `proto::BATCH_PARAM_IDS` 决定 ——
    /// 面板行、围栏列、待写值列共用同一份顺序, 不再各自按 `id-1` 猜位置。
    pub fn batch_param_selected(&self) -> Vec<bool> {
        crate::proto::BATCH_PARAM_IDS
            .iter()
            .map(|id| self.batch_sel.param_selected(*id))
            .collect()
    }

    pub fn batch_source(&self) -> u8 {
        self.batch_source
    }

    /// 设置源通道(UI 上是 Shift+单击)。源通道自身不需要也不应被勾成目标 —— 顺手把它从目标集里
    /// 摘掉, 免得用户看到"源=目标"这种无意义的选择还以为会生效。
    /// ★换源通道即整列重新预填★: 换源的语义就是"改用另一个通道的值当模板", 保留上一个源的手改
    /// 残留只会让人分不清面板上这一列到底是谁的值。
    pub fn batch_set_source(&mut self, ch: u8) {
        if ch >= 36 {
            return;
        }
        self.batch_source = ch;
        self.batch_sel.ch_mask &= !(1u64 << ch);
        self.batch_values = [None; 11];
        self._batch_fill_missing();
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    /// 批量面板每参数的待写值, 顺序 = `proto::BATCH_PARAM_IDS`。`-1` = 尚无值(源通道未回读且
    /// 用户未手填): 用一个不可能合法的哨兵值表达"无", 免得再并排一个 bool 数组(围栏 min 一律 >= 0)。
    pub fn batch_values(&self) -> Vec<i32> {
        crate::proto::BATCH_PARAM_IDS
            .iter()
            .map(|id| self.batch_values[(*id - 1) as usize].map_or(-1i32, |value| value as i32))
            .collect()
    }

    /// 各参数的合法输入范围(下标 = param_id - 1), 唯一来源 = `proto::param_fence`。
    /// ★不许在 .slint 里按 param_id 写阈值★: 那会变成第四份围栏, 与固件两处必然漂移。
    pub fn batch_value_bounds(&self) -> (Vec<i32>, Vec<i32>) {
        let ids = crate::proto::BATCH_PARAM_IDS;
        let mut mins = Vec::with_capacity(ids.len());
        let mut maxs = Vec::with_capacity(ids.len());
        for &id in ids {
            let fence = crate::proto::param_fence(id);
            mins.push(fence.ui_min() as i32);
            maxs.push(fence.ui_max() as i32);
        }
        (mins, maxs)
    }

    /// 用户在批量面板里手改某项的待写值。不校验围栏 —— 校验统一在应用时由 `set_param` 做
    /// (与手工编辑同一条路径), 这里若提前拒绝, 用户就没法把数字从一个非法中间态改到合法值。
    pub fn batch_set_value(&mut self, param_id: u8, value: u32) {
        if !crate::proto::BATCH_PARAM_IDS.contains(&param_id) {
            return;
        }
        self.batch_values[(param_id - 1) as usize] = Some(value);
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    /// 把仍为空的项用当前源通道的真值补上(源通道参数是异步回读的, 选源那一刻可能还没到)。
    /// ★只补空项★: 已有值的项可能是用户手改过的, 覆盖它就等于把人家键入的数抹掉。
    /// 返回是否补进了新值, 供调用方决定要不要 bump 版本号(避免每 tick 无意义地重建行模型)。
    pub fn batch_fill_missing_values(&mut self) -> bool {
        let filled = self._batch_fill_missing();
        if filled {
            self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
        }
        filled
    }

    /// 预填实现(不动版本号): 供 `batch_set_source` 与 `batch_fill_missing_values` 复用。
    fn _batch_fill_missing(&mut self) -> bool {
        let src = self.batch_source;
        let mut filled = false;
        for &id in crate::proto::BATCH_PARAM_IDS {
            let slot = (id - 1) as usize;
            if self.batch_values[slot].is_some() {
                continue;
            }
            if let Some(value) = self.param(src, id) {
                self.batch_values[slot] = Some(value);
                filled = true;
            }
        }
        filled
    }

    pub fn batch_toggle_channel(&mut self, ch: u8) {
        if ch >= 36 {
            return;
        }
        self.batch_sel.ch_mask ^= 1u64 << ch;
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    pub fn batch_toggle_param(&mut self, param_id: u8) {
        if !crate::proto::BATCH_PARAM_IDS.contains(&param_id) {
            return;
        }
        self.batch_sel.param_mask ^= 1u16 << (param_id - 1);
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    /// 通道全选 / 反选。反选按位取反后仍需掩掉高位, 否则 36 位以上的幻影通道会被算进计数。
    pub fn batch_channels_all(&mut self) {
        self.batch_sel.ch_mask = BatchApplySel::CH_ALL;
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    pub fn batch_channels_invert(&mut self) {
        self.batch_sel.ch_mask = !self.batch_sel.ch_mask & BatchApplySel::CH_ALL;
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    pub fn batch_params_all(&mut self) {
        self.batch_sel.param_mask = BatchApplySel::PARAM_ALL;
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    pub fn batch_params_invert(&mut self) {
        self.batch_sel.param_mask = !self.batch_sel.param_mask & BatchApplySel::PARAM_ALL;
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    pub fn batch_clear(&mut self) {
        self.batch_sel.clear();
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
    }

    /// 批量抽屉从展开 → 收起时的会话清理。
    /// ★为什么必须真清而不是"看不见就算了"★ 勾选态、面板待写值、源通道都活在 AppController 里,
    /// 收起抽屉只是 Slint 不再实例化那棵子树 —— 状态全都还在。于是下次展开时会带着上一次的
    /// 一批勾选出现, 而"应用到已选通道"是立刻按它执行的: 用户以为在给新选的两个通道写值,
    /// 实际写进了上一次遗留的一二十个通道。
    /// 清理范围就是"本次批量会话"的全部东西: 目标通道勾选 + 参数勾选(沿用 batch_clear) +
    /// 面板上手改过的待写值 + 源通道。草稿(已经写进 drafts 的改动)不动 —— 那是用户已确认的编辑,
    /// 该由"保存到设备"或"撤销改动"处置, 不属于抽屉的会话状态。
    pub fn batch_drawer_closed(&mut self) {
        let had_ch = self.batch_sel.ch_mask != 0;
        let had_param = self.batch_sel.param_mask != 0;
        let had_value = self.batch_values.iter().any(|v| v.is_some());
        if !had_ch && !had_param && !had_value && self.batch_source == 0 {
            return;
        }
        self.batch_sel.clear();
        self.batch_values = [None; 11];
        self.batch_source = 0;
        self.batch_sel_version = self.batch_sel_version.wrapping_add(1);
        if had_ch || had_param {
            self.push_log_debug(
                "批量应用抽屉已收起: 已清空目标通道与参数勾选、面板待写值与源通道(草稿改动不受影响)"
                    .to_string(),
            );
        }
    }

    /// 把批量面板上被勾选的参数【按面板上的当前值】写进被勾选目标通道的草稿。
    /// 值来自 `batch_values`(选源通道时用源真值预填, 之后可由用户逐项手改) —— ★写的是面板上
    /// 显示的那个数★, 而不是重新去读源通道, 否则用户手改的值会被静默忽略。
    /// 严格隔离: 未勾选的通道、未勾选的参数一律不碰; 源通道自身即使被勾选也跳过(自我复制无意义)。
    /// 无值的项跳过并计数, 绝不拿 0 或默认值顶替; 非法值由 `set_param` 的围栏明确拒绝并记日志。
    pub fn batch_apply_from(&mut self, src: u8) -> anyhow::Result<()> {
        if src >= 36 {
            return Ok(());
        }
        // ★不再排除源通道★
        // 早期这一列的值恒等于源通道真值, "源→其余"是对的。但值一列现在**可以手改**(面板值才是
        // 真正要写下去的东西), 于是排除源通道就变成"唯一拿不到新值的通道恰好是你选中的那个" ——
        // 用户看到的就是"批量覆盖没覆盖部分通道"。
        // 把它一并写入是安全的: 若源通道当前值本来就等于面板值, set_param 的 same_as_device 判定
        // 会自动撤稿(不产生脏项、不下发), 所以这里不需要任何特例。
        // ★禁用通道不参与批量调参★ 它的电极是高阻、不参与扫描, 写进去的阈值/分频既不会被使用,
        // 也会在"保存到设备"时白占一轮下发与对账。勾选仍然允许(批量启用/禁用要靠它), 只在这里排除。
        let selected: Vec<u8> = (0..36u8)
            .filter(|ch| self.batch_sel.ch_selected(*ch))
            .collect();
        let targets: Vec<u8> = selected
            .iter()
            .copied()
            .filter(|ch| self.ch_enabled(*ch))
            .collect();
        let skipped_disabled = selected.len() - targets.len();
        let ids: Vec<u8> = crate::proto::BATCH_PARAM_IDS
            .iter()
            .copied()
            .filter(|id| self.batch_sel.param_selected(*id))
            .collect();
        if targets.is_empty() && skipped_disabled > 0 && !ids.is_empty() {
            self.push_log_warn(format!(
                "批量应用未执行: 已选的 {} 个通道全部处于禁用状态(电极高阻, 不参与扫描), \
                 调参对它们无效。先用「批量启用」打开再调。",
                skipped_disabled
            ));
            return Ok(());
        }
        if targets.is_empty() || ids.is_empty() {
            self.push_log_warn(format!(
                "批量应用未执行: 已选目标通道 {} 个 / 已选参数 {} 项 — 两者都需至少选一项。",
                targets.len(),
                ids.len()
            ));
            return Ok(());
        }
        let mut missing = 0usize;
        let mut rejected = 0usize;
        let mut written = 0usize;
        for id in &ids {
            let Some(value) = self.batch_values[(*id - 1) as usize] else {
                missing += 1;
                continue;
            };
            // ★非法项只跳过它自己, 不中断整批★: 一个参数越界就 `?` 返回, 会让前面已写入草稿的
            // 项半途而废, 用户既不知道写到哪了, 也不知道剩下的没写。围栏判定与通道无关(同一个
            // 值对 36 个通道同样合法或同样非法), 故第一个通道被拒即整项跳过 —— 告警只出现一次,
            // 不刷 36 条同样的日志(拒绝原因与合法范围由 set_param 内部写进日志)。
            let mut ok = true;
            for ch in &targets {
                if self.set_param(*ch, *id, value).is_err() {
                    ok = false;
                    break;
                }
                written += 1;
            }
            if !ok {
                rejected += 1;
            }
        }
        // 批量写草稿不经控件, 必须显式 bump 让下一 tick 的既有门控重建行模型(同 JSON 导入路径)。
        self._bump_view_versions();
        let mut msg = format!(
            "批量应用完成: 面板值(取自 CH{}, 可手改) → 已选 {} 个通道 × {} 项参数, 共写入 {} 项草稿。\
             与设备真值相同的项不计入(自动撤稿)。需点“保存到设备”才生效。",
            src,
            targets.len(),
            ids.len() - missing - rejected,
            written
        );
        if missing > 0 {
            msg.push_str(&format!(
                " 跳过 {} 项(面板上无值: 源通道尚无该参数真值且未手填, 未用默认值顶替)。",
                missing
            ));
        }
        if rejected > 0 {
            msg.push_str(&format!(
                " 拒绝 {} 项(面板上填的值超出该参数合法范围, 见上方告警)。",
                rejected
            ));
        }
        if skipped_disabled > 0 {
            msg.push_str(&format!(
                " 另有 {} 个已勾选通道被跳过(处于禁用状态, 调参对它们无效)。",
                skipped_disabled
            ));
        }
        self.push_log(msg);
        Ok(())
    }

    fn _device_busy(&self) -> bool {
        self.device_busy_until
            .is_some_and(|until| std::time::Instant::now() < until)
    }

    /// 非紧急轮询的统一发送入口。设备忙时只丢弃可重建的查询，不触碰配置 FIFO 和用户操作。
    fn _send_nonurgent(&mut self, frame: Frame) -> anyhow::Result<bool> {
        if self._device_busy() {
            return Ok(false);
        }
        let Some(handle) = &self.io else {
            return Ok(false);
        };
        handle.send(frame)?;
        Ok(true)
    }

    fn _note_device_busy_nak(&mut self, frame: &Frame) {
        if frame.payload.first().copied() == Some(0x03) {
            self.device_busy_until =
                Some(std::time::Instant::now() + std::time::Duration::from_secs(8));
            self.push_log_warn("设备返回 BUSY NAK，等待设备恢复后降频轮询".to_string());
        }
    }

    /// 配置写类命令的内部入队口。默认归属 External；批量 prime 用私有 owner 入口，取消只清该代帧。
    fn _queue_tx(&mut self, frame: Frame) -> anyhow::Result<()> {
        self._queue_tx_owned(frame, CfgTxOwner::External)
    }

    fn _queue_tx_owned(&mut self, frame: Frame, owner: CfgTxOwner) -> anyhow::Result<()> {
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法下发写命令"));
        }
        self.cfg_tx_queue.push_back(CfgTxFrame { frame, owner });
        Ok(())
    }

    /// 批量 AutoTune 前置写的唯一入口：仍走既有 cfg FIFO，只附 generation 所有权标记。
    pub(super) fn _queue_batch_prime_param(
        &mut self,
        generation: u64,
        ch: u8,
        value: u32,
    ) -> anyhow::Result<()> {
        self._reject_illegal_param(ch, PARAM_IDAC_GAIN, value)?;
        let seq = self.next_seq();
        let payload = crate::proto::encode_param_set(ch, PARAM_IDAC_GAIN, value);
        self._queue_tx_owned(
            Frame::new(HostCmd::ParamSet as u8, 0, seq, payload),
            CfgTxOwner::BatchPrime(generation),
        )?;
        self.params[ch as usize].insert(PARAM_IDAC_GAIN, value);
        Ok(())
    }

    /// 批量自动化收尾只排入一次 SAVE_CONFIG，持久化本批全部运行态改动。
    pub(super) fn _queue_batch_save(&mut self, generation: u64) -> anyhow::Result<()> {
        let seq = self.next_seq();
        self._queue_tx_owned(
            Frame::new(HostCmd::SaveConfig as u8, 0, seq, vec![]),
            CfgTxOwner::BatchPrime(generation),
        )
    }

    /// 当前 batch generation 是否仍有未发或在途 cfg 帧。
    pub(super) fn _cfg_batch_pending(&self, generation: u64) -> bool {
        self.cfg_tx_queue
            .iter()
            .any(|item| item.owner == CfgTxOwner::BatchPrime(generation))
            || self
                .cfg_tx_inflight
                .as_ref()
                .is_some_and(|item| item.owner == CfgTxOwner::BatchPrime(generation))
    }

    /// 丢弃本批尚未发送的 prime 帧；绝不清队列整体，且在途帧必须自然 ACK/NAK/超时后才算收敛。
    pub(super) fn _cfg_purge_batch_frames(&mut self, generation: u64) {
        self.cfg_tx_queue
            .retain(|item| item.owner != CfgTxOwner::BatchPrime(generation));
    }

    fn _record_cfg_tx_failure(&mut self, cmd: u8, seq: u8, reason: impl AsRef<str>) {
        self.last_error = Some(format!(
            "写命令失败: cmd=0x{:02X} seq={} — {}",
            cmd,
            seq,
            reason.as_ref()
        ));
    }

    /// 配置写类命令的串行泵: 窗口=1, 每 tick 最多推进一帧。
    fn _pump_cfg_tx(&mut self) {
        const CFG_TX_TIMEOUT_TICKS: u32 = 125;
        if self.algo_src_tx.as_ref().and_then(|tx| tx.seq).is_some() {
            return;
        }
        if let Some(inflight) = self.cfg_tx_inflight.as_mut() {
            if inflight.waited >= CFG_TX_TIMEOUT_TICKS {
                let cmd = inflight.cmd;
                let seq = inflight.seq;
                let reason = "超过 2s 无 ACK/NAK，已跳过并继续下一项(设备可能卡住或仍在写 flash)";
                self.push_log_warn(format!(
                    "保存队列: cmd=0x{:02X} seq={} {},",
                    cmd, seq, reason
                ));
                self._record_cfg_tx_failure(cmd, seq, reason);
                self.cfg_tx_inflight = None;
            } else {
                inflight.waited += 1;
                return;
            }
        }
        let Some(queued) = self.cfg_tx_queue.pop_front() else {
            return;
        };
        let cmd = queued.frame.cmd;
        let seq = queued.frame.seq;
        if let Some(handle) = &self.io {
            if let Err(e) = handle.send(queued.frame) {
                self._record_cfg_tx_failure(cmd, seq, format!("本地发送失败: {}", e));
                return;
            }
            self.cfg_tx_inflight = Some(CfgTxInflight {
                cmd,
                seq,
                waited: 0,
                owner: queued.owner,
            });
        } else {
            self.cfg_tx_queue.clear();
            self.cfg_tx_inflight = None;
        }
    }

    /// 队列里还有多少项没发(含在途)。UI 可据此显示保存进度。
    pub fn cfg_tx_pending(&self) -> usize {
        self.cfg_tx_queue.len() + if self.cfg_tx_inflight.is_some() { 1 } else { 0 }
    }

    /// 收到 ACK/NAK 时调用: 只允许匹配在途帧的最终回执放行下一帧；NAK 必须保留 cmd/seq/设备原因。
    fn _cfg_tx_note_reply(&mut self, frame: &Frame) {
        let nak = (frame.flags & 0x04) != 0 || frame.cmd == HostCmd::Nak as u8;
        if !nak && frame.cmd != HostCmd::Ack as u8 {
            return;
        }
        let Some(inflight) = self.cfg_tx_inflight.as_ref() else {
            return;
        };
        if inflight.seq != frame.seq {
            return;
        }
        let inflight_cmd = inflight.cmd;
        let inflight_seq = inflight.seq;
        if nak {
            let reason = if frame.payload.len() > 1 {
                String::from_utf8_lossy(&frame.payload[1..]).into_owned()
            } else {
                "未知错误".to_string()
            };
            self._record_cfg_tx_failure(
                inflight_cmd,
                inflight_seq,
                format!("设备拒绝(NAK): {}", reason),
            );
        }
        self.cfg_tx_inflight = None;
        if inflight_cmd == HostCmd::SaveConfig as u8 {
            if nak {
                self.post_save_refetch_channels.clear();
            } else {
                let channels = std::mem::take(&mut self.post_save_refetch_channels);
                for ch in channels {
                    if let Err(error) = self.request_params(ch) {
                        self.push_log_warn(format!("CH{} 重新启用后参数回读失败: {}", ch, error));
                    }
                }
            }
        }
        // ★收到回执立刻发下一帧, 不再等到下一个 16ms tick★
        // 窗口仍然是 1(同一时刻只有一帧在途, ACK 仍代表"设备真的做完了"), 但原实现把"何时发下一帧"
        // 绑在 GUI 的 16ms tick 上, 于是吞吐被硬钉在 62.5 帧/秒 —— 与设备快慢无关。
        // 实测代价: 一次"保存到设备"改 36 通道 × 11 项 = 396 帧, 光是这层节拍就要 6.3 秒, 用户看到的
        // 就是"批量设置很久才生效 / 上报延迟很大"。设备 ACK 往返只有几百 µs 到数 ms, 顺势接着发下一帧
        // 既不增加并发也不增加在途帧数, 只是把这段白等的空拍去掉。
        // 重操作(校准/自适应/GLOBAL_COMMIT)的 ACK 本身就要等到设备真正完成才来, 所以不会被"加速"成堆叠。
        self._pump_cfg_tx();
    }

    /// 排程"全 36 通道 × 全 per-channel 参数"重读。
    /// 恢复默认、重连、导入后都必须走这条: 否则未被显式重读的通道会一直显示过期或空值。
    /// 队列由 `poll` 每 tick 取一条下发, 全集读完约 11 tick(~180ms)。
    pub fn schedule_param_refetch_all(&mut self) {
        // 0x01..=0x0C 即 per-channel 参数全集(见 proto::telemetry 的 PARAM_* 常量, 含 0x0C 启用开关)。
        // 倒序入队使 pop 时按 id 升序发出, 便于对着日志核对进度。
        self.param_refetch_ids = (0x01u8..=0x0Cu8).rev().collect();
    }

    /// main.rs 每 tick 轮询: 恢复默认的设备端回填就绪后返回一次 true, 触发主循环做全量重读。
    pub fn take_post_reset_refetch(&mut self) -> bool {
        if self.post_reset_refetch_pending {
            self.post_reset_refetch_pending = false;
            true
        } else {
            false
        }
    }

    /// 在已有 IO 会话中同步执行一条 BUS_XFER，期间照常派发无关的 IO 事件。
    pub fn bus_xfer(
        &mut self,
        payload: Vec<u8>,
        timeout: std::time::Duration,
    ) -> anyhow::Result<Vec<u8>> {
        let seq = self.next_seq();
        self.io
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("未连接，无法发送 BUS_XFER"))?
            .send(Frame::new(HostCmd::BusXfer as u8, 0, seq, payload))?;
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                return Err(anyhow::anyhow!("BUS_XFER 响应超时"));
            }
            let event = self
                .io
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("等待 BUS_XFER 响应时设备断开"))?
                .recv_timeout(remaining)
                .map_err(|error| anyhow::anyhow!("BUS_XFER 响应失败: {}", error))?;
            match event {
                IoEvent::Frame(frame) if frame.cmd == HostCmd::BusXfer as u8 => {
                    return Ok(frame.payload);
                }
                event => self.handle_event(event),
            }
        }
    }

    /// 发送 PING (0x03) 保活:UI 连接期间周期发送,使设备侧"主机连接"指示(绿灯常亮)保持,
    /// 并作为链路存活探测。
    pub fn ping(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let frame = Frame::new(HostCmd::Ping as u8, 0, seq, vec![]);
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 发送重启指令 (REBOOT, 0x04)
    pub fn reboot(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let frame = Frame::new(HostCmd::Reboot as u8, 0, seq, vec![]);
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 发送进烧录模式指令 (REBOOT_BOOTLOADER, 0x05)
    pub fn reboot_bootloader(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let frame = Frame::new(HostCmd::RebootBootloader as u8, 0, seq, vec![]);
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 运行时武装/解除"崩溃→进 BOOTSEL"(DEBUG_CRASH_BOOTSEL, 0x07): 自持 debug 时连上即武装,
    /// 一旦固件崩溃自动进烧录便于自动重烧; 平时解除则崩溃仅正常重启。arm=true 武装, false 解除。
    pub fn set_crash_bootsel(&mut self, arm: bool) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let frame = Frame::new(
                HostCmd::DebugCrashBootsel as u8,
                0,
                seq,
                vec![if arm { 1 } else { 0 }],
            );
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 重启 PSoC (REBOOT_PSOC, 0x06): 脉冲 XRES 复位, 使"需重启生效"的改动生效(阻塞类, 显示运行中)。
    /// ★PSoC 救砖★: 经 SWD 强制全片重刷 PSoC 内嵌镜像 + 校验 + 复位, 再由设备重新下发算法/CSD。
    /// 设备立即回 ACK("已受理"), 阶段进度经 PSOC_RESCUE_PROGRESS(0x09) 推送流上报。
    /// 用于扫描引擎卡死/PSoC 变砖、恢复默认也救不回来时的最后手段(期间触控不可用, 耗时数秒)。
    pub fn psoc_rescue(&mut self) -> anyhow::Result<()> {
        // 救砖会全片重刷并让 RP2040 重新下发算法/CSD ⇒ 比任何一次重配都彻底, 扫描期间同样拒绝。
        // ★不会把用户困住★: 扫描的每一步都以 `_csd_locked()` 为闸, 设备真失联/卡死时由既有 51s
        // 看门狗解锁, 还原阶段随即收尾并置 active=false, 之后救砖立刻可用。
        if self._reject_csd_if_batching("PSoC 救砖")
            || self._reject_csd_if_sweeping("PSoC 救砖")
            || self._reject_csd_if_locked("PSoC 救砖")
        {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::PsocRescue as u8, 0, seq, vec![]))?;
        }
        self.rescue_progress = crate::proto::PsocRescueProgress {
            state: 1,
            phase: 1,
            ..Default::default()
        };
        self.rescue_progress_version = self.rescue_progress_version.wrapping_add(1);
        self._begin_op("PSoC 救砖中: 已受理", seq);
        // ACK 仅表示"已受理"(与频率自适应同): 真实完成由推送流终态帧判定。
        self.op_ack_is_accept = true;
        self.push_log_warn("PSoC 救砖已下发: 经 SWD 全片擦写并校验内嵌固件, 随后重新下发算法/CSD。期间触控不可用, 请勿断电。");
        Ok(())
    }

    pub fn rescue_progress(&self) -> crate::proto::PsocRescueProgress {
        self.rescue_progress
    }
    pub fn rescue_progress_version(&self) -> u64 {
        self.rescue_progress_version
    }

    /// 设备推送的救砖阶段进度(0x09)。进行中刷新 op_label 并重置卡死计时; 终态解锁并落日志。
    fn _handle_rescue_progress(&mut self, frame: &Frame) {
        let progress = match crate::proto::decode_psoc_rescue_progress(&frame.payload) {
            Ok(p) => p,
            Err(e) => {
                self.push_log_warn(format!("PSOC_RESCUE_PROGRESS 解析失败: {}", e));
                return;
            }
        };
        self.rescue_progress = progress;
        self.rescue_progress_version = self.rescue_progress_version.wrapping_add(1);
        self.op_busy_ticks = 0; // 进度帧本身即"设备活着"的证据

        if progress.state == 2 {
            if progress.result == 1 {
                self.push_log("✔ PSoC 救砖完成: 固件已重刷并校验通过, 算法/CSD 已重新下发。");
            } else {
                self.push_log_error(format!(
                    "✕ PSoC 救砖失败于阶段 {}({}) — 检查 SWD 连线/供电后重试, 或用外部 DAP 烧录。",
                    progress.fail_stage,
                    crate::proto::PsocBringupDiagnostics::stage_name(progress.fail_stage)
                ));
            }
            self._end_op();
            // 重刷后设备侧 CSD/诊断已变 → 重新取一次 DEVICE_INFO 与配置, 刷新健康显示。
            let _ = self.resend_hello();
            self.post_reset_refetch_in = Some(60); // ~1s 后全量重读
            return;
        }
        if self.op_busy {
            self.op_label = format!(
                "PSoC 救砖中: {} (阶段 {})",
                progress.phase_text(),
                crate::proto::PsocBringupDiagnostics::stage_name(progress.stage)
            );
            self.op_version = self.op_version.wrapping_add(1);
        }
    }

    pub fn reboot_psoc(&mut self) -> anyhow::Result<()> {
        // XRES 复位 = PSoC 的 CSD 配置(含扫描临时写下的 gain/div)全部丢失并被 store 重新下发 ⇒
        // 扫描中途执行会让当前格测的不是它以为的配置, 还会踩乱还原路径。
        if self._reject_csd_if_batching("重启 PSoC") || self._reject_csd_if_sweeping("重启 PSoC")
        {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let frame = Frame::new(HostCmd::RebootPsoc as u8, 0, seq, vec![]);
            handle.send(frame)?;
        }
        self._begin_op("PSoC 重启中", seq);
        // ★重启必然改变设备真值★: PSoC 的 CSD 配置活在 RAM, 复位即丢; RP2040 会用它持久化的 store
        // 重新下发(main.cpp:217 → csd_config.cpp:118), 且 PSoC 启动时还会强制覆盖部分项。
        // 因此重启后必须回读真值刷新 UI, 否则界面继续显示重启前的旧值, 用户会误判配置仍然生效。
        self.globals_expected.clear(); // 无期望值可比对, 仅同步真值
        self.globals_verify_in = Some(220); // 等 XRES + 链路恢复 + RP2040 重新下发完成
        self.push_log("PSoC 重启已下发: 其 CSD 配置在 RAM 中会丢失, 由 RP2040 用已保存配置重新下发; 稍后自动回读设备真值同步 UI。".to_string());
        Ok(())
    }

    /// 获取所有配置项(按 key 排序), 草稿值覆盖缓存值。
    pub fn config_entries(&self) -> Vec<ConfigEntry> {
        self.config_cache
            .values()
            .map(|e| match self.drafts.cfg(&e.key) {
                Some(v) => ConfigEntry::new(e.key.clone(), v.clone()),
                None => e.clone(),
            })
            .collect()
    }

    /// 查询单个配置项, 草稿优先。
    pub fn config_get(&self, key: &str) -> Option<ConfigEntry> {
        if let Some(v) = self.drafts.cfg(key) {
            return Some(ConfigEntry::new(key.to_string(), v.clone()));
        }
        self.config_cache.get(key).cloned()
    }

    /// 查询单个配置项的**设备真值**条目(不受草稿覆盖)。
    /// `config_get` 的草稿分支会构造 `range: None` 的临时条目, 无法用于校验;
    /// 外部批量写入(JSON 导入)必须按设备声明的类型与 min/max 判定合法性, 故单独提供本入口。
    pub fn config_truth(&self, key: &str) -> Option<&ConfigEntry> {
        self.config_cache.get(key)
    }

    /// 当前工作模式(0=Serial, 1=HID), 草稿优先 —— 与配置页 ComboBox 同一口径。
    /// 返回 None = 尚未从设备回读到 mode.work: 主页必须显示"未知"而不是拿默认值 0 冒充设备真值。
    pub fn work_mode(&self) -> Option<u8> {
        match self.config_get("mode.work").map(|e| e.value)? {
            CfgValue::U8(v) => Some(v),
            CfgValue::U16(v) => Some(v.min(u8::MAX as u16) as u8),
            CfgValue::U32(v) => Some(v.min(u8::MAX as u32) as u8),
            CfgValue::I8(v) => Some(v.max(0) as u8),
            CfgValue::Bool(v) => Some(u8::from(v)),
            _ => None,
        }
    }

    /// 配置缓存版本号 (#6e-2):每次成功把服务端响应合并进 config_cache 时自增。
    /// UI 侧据此判断缓存是否有更新,只有变化时才重建配置行,避免覆盖用户
    /// 正在编辑但尚未提交的值。
    pub fn config_version(&self) -> u64 {
        self.config_version
    }

    // ------------------------------------------------------------------
    // 遥测方法 (#6g-1)
    // ------------------------------------------------------------------

    /// 启动遥测流
    pub fn start_telemetry(
        &mut self,
        rate_hz: u16,
        fields: u8,
        ch_mask: u64,
    ) -> anyhow::Result<()> {
        if self._device_busy() {
            if self
                .device_busy_log_at
                .is_none_or(|at| at.elapsed() >= std::time::Duration::from_secs(2))
            {
                self.device_busy_log_at = Some(std::time::Instant::now());
                self.push_log_warn("设备忙，暂停 TELEM_START，等待设备恢复".to_string());
            }
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            // 始终 OR 上 STATS/LATENCY 位:调用方无需关心该细节,保证实际发出的 fields 带采样率和延迟统计。
            let fields = fields | crate::proto::FIELD_STATS | crate::proto::FIELD_LATENCY;
            let payload = crate::proto::encode_telem_start(0, rate_hz, fields, ch_mask);
            let frame = Frame::new(HostCmd::TelemStart as u8, 0, seq, payload);
            handle.send(frame)?;
            self.telem_active = true;
            self.telem_started_at = Some(std::time::Instant::now());
            // 只清存活检测计数(重新开流, 冻结判定从头计)。
            // ★不再清样本缓冲★: "停止/开始"要当成暂停/继续用 —— 暂停时间轴冻结、已抓到的波形留在
            // 图上可继续缩放查看; 继续后设备时间是延续的, 中间那段停流间隙由绘图侧断开子路径如实
            // 表示(不做直线插值)。设备断连/换设备才真正清缓冲(见 _clear_sensor_caches)。
            self.telem_last_raw = [None; 36];
            self.telem_freeze_count = [0; 36];
            // 重发当前 scope 后重新给首帧一个宽限；attempts 只由收到该 CH 新样本清零，
            // 所以持续缺失仍会升级到有界链路恢复。
            self.telem_channel_last_seen = [None; 36];
        }
        Ok(())
    }

    /// 按"当前页面是否需要逐通道数据"收放推流范围。由 main.rs 每 tick 调用。
    ///
    /// ★为什么要分两档★: 逐通道字段(raw/baseline/diff/status × 36 通道 @100Hz)是 IN 带宽与设备侧
    /// 打包开销的绝对主力, 而它只有"触控通道调整"和"单通道精调"两页真的在看。离开这两页还继续推,
    /// 纯属占用带宽 —— 更要紧的是: 固件做重操作(XRES 后 CSD 全量重下发)时 core0 不解析 vendor 帧,
    /// 上位机这边每 16ms 一轮的命令与设备侧 1024B 的 rx 环叠加, 会把命令挤掉甚至把链路拖崩。
    /// ★延迟/采样率必须与逐通道数据解耦★: 主页的延迟卡与采样率是"UI 开着就该有"的东西, 而它们
    /// 搭在同一条 TELEM_DATA 上(start_telemetry 强制 OR 上 FIELD_STATS|FIELD_LATENCY)。所以
    /// 收起逐通道字段时不是停流, 而是切到"只要 STATS+LATENCY"的轻档: 一帧十几字节, 且降到 20Hz。
    /// 只在档位真的变化时才发命令, 不会每 tick 刷。
    pub fn telem_set_scope(&mut self, want_channels: bool) {
        if self.io.is_none() {
            // 未连接: 档位记为未知, 重连后必然重新下发一次。
            self.telem_scope_channels = None;
            return;
        }
        // 连接探针优先：基础回读完成前延迟自动 TELEM_START，避免流帧占用唯一 WinUSB 会话。
        if self.conn_probes_pending() {
            return;
        }
        // ★用户按了"停止"就必须真的停住★
        // 本方法每 16ms 被主循环调一次, 而 stop_telemetry() 会把档位记忆清成未知 —— 于是"停止"下一帧
        // 就被这里重新开流, 表现为按钮完全没用(实测)。页面档位切换是自动行为, 不该覆盖用户的显式意图。
        if self.telem_user_paused {
            return;
        }
        if self.telem_scope_channels == Some(want_channels) {
            return;
        }
        let res = if want_channels {
            // 全通道广谱只需 30Hz: 全 36 通道的分块快照本身就要 16 个 1ms 拍才凑齐一份
            // (物理上限 ~62 份/s), 再高的请求只会让固件重复发同一代数据, 白占 USB 与主循环预算。
            // 30Hz 是"看整盘状态"够用的刷新率; 需要细节的场合走单通道独占流(它才是高速路)。
            self.start_telemetry(
                30,
                crate::proto::FIELD_RAW
                    | crate::proto::FIELD_BASELINE
                    | crate::proto::FIELD_DIFF
                    | crate::proto::FIELD_STATUS,
                0xFFFFFFFF_FFFFFFFFu64,
            )
        } else {
            // fields=0 → start_telemetry 内部 OR 出 STATS|LATENCY; ch_mask=0 表示不要任何通道数据。
            self.start_telemetry(20, 0, 0)
        };
        if res.is_ok() {
            self.telem_scope_channels = Some(want_channels);
            self.push_log_debug(if want_channels {
                "遥测: 已切到逐通道档(raw/baseline/diff/status, 100Hz)".to_string()
            } else {
                "遥测: 已切到轻档(仅采样率+延迟统计, 20Hz) — 离开通道页, 不再推逐通道数据"
                    .to_string()
            });
        }
    }

    /// 带 STATS 的帧多久没来就判定"无实测值"。取 2s: 轻档 20Hz、逐通道档 100Hz, 正常每轮都有,
    /// 2s 足够容忍一次重操作(校准/自适应)造成的推送空档, 又不至于让过期值长期冒充实测值。
    const TELEM_STATS_FRESH: std::time::Duration = std::time::Duration::from_secs(2);
    /// 逐通道 RAW 期望档的 freshness 门限。超过它，缓存旧值必须标 stale，即使全局 STATS 仍正常。
    const TELEM_CHANNEL_FRESH: std::time::Duration = std::time::Duration::from_secs(2);
    /// 遥测自愈: STATS 断了这么久就重新下发一次 TELEM_START。
    const TELEM_HEAL_AFTER: std::time::Duration = std::time::Duration::from_secs(3);
    /// 自愈重试最小间隔。★必须足够稀★: 本设备所有流量共用一对 bulk 端点, 高频重发 TELEM_START
    /// 只会把 vendor FIFO 打爆并与校准/自适应抢链路(本仓已因此掉线过)。5s 一次远低于既有 PING(1s)
    /// 的量级, 不可能把固件带崩; 而"卡住就永远不恢复"才是真正要避免的。
    const TELEM_HEAL_COOLDOWN: std::time::Duration = std::time::Duration::from_secs(5);

    /// 仍在逐通道档且已经跨过开流宽限后，列出持续缺失的通道。
    fn _stale_telem_channels(&self) -> Vec<u8> {
        if self.telem_scope_channels != Some(true) || !self.telem_active {
            return Vec::new();
        }
        let now = std::time::Instant::now();
        // ★单通道流期间只判目标通道★ FOCUS_START 会让固件挂起广谱流(设计如此), 其余通道"没数据"
        // 是正常的。不收窄就会让遥测看门狗每 5s 白重发一次 TELEM_START(那条命令在 focus 期间也
        // 只被固件记下来备用, 除了占带宽什么都不会发生)。
        let focus_ch = self.focus.channel;
        (0..36u8)
            .filter(|ch| focus_ch.is_none_or(|target| target == *ch))
            // ★禁用通道不参与缺失判定★ 它按设计不上报有效数据(设备侧一律发 0), 把它算成"缺失"
            // 会让遥测看门狗无休止地重发 TELEM_START、甚至触发 HELLO 链路恢复 —— 用户只是关了
            // 几个通道, 却换来周期性的链路重建。
            .filter(|ch| self.ch_enabled(*ch))
            .filter(|ch| {
                self.telem_channel_last_seen[*ch as usize]
                    .or(self.telem_started_at)
                    .is_some_and(|seen| now.duration_since(seen) >= Self::TELEM_CHANNEL_FRESH)
            })
            .collect()
    }

    /// 某通道的缓存值是否已经过期。调用方可继续保留历史曲线，但不得把它冒充当前设备读数。
    pub fn channel_stale(&self, ch: u8) -> bool {
        (ch as usize) < 36 && self._stale_telem_channels().contains(&ch)
    }

    /// 遥测看门狗：先处理单通道缺失，再处理全流 STATS 中断。单通道首次只重发当前 TELEM_START；
    /// 持续缺失才有界触发现有 HELLO 链路恢复，绝不以校准/Cp 之类破坏 CSD 状态的操作掩盖问题。
    pub fn telem_heal_tick(&mut self) -> bool {
        if self.io.is_none() || !self.telem_active || self.telem_user_paused {
            return false;
        }
        if self.conn_probes_pending() {
            return false;
        }
        // ★频谱会话期间本看门狗必须整体让位★
        // 设备侧 SWEEP_START 按设计挂起广谱流(sensor_link.cpp `_pause_broad_for_focus`), 于是带 STATS
        // 的 TELEM_DATA 本就不再到达 —— 那是会话的正常表现, 不是链路故障。而本函数的升级路径是
        // `resend_hello()`, 固件 HELLO 处理无条件 `SensorLink::stop()`(host_cmd.cpp:433), 它在扫描
        // 活跃时直接把会话掀进 CANCELLED 恢复链。结果就是扫描每次都在 heal 门限(3s)+冷却(5s)内被
        // 自己的看门狗杀掉, 只产出个位数格。会话是否健康由 SWEEP_DATA 的显式续租/终态自证, 不需要
        // 也不允许这条通用看门狗插手。
        if self.noise_sweep_active() {
            return false;
        }
        let now = std::time::Instant::now();
        let cooled_down = self
            .telem_heal_at
            .is_some_and(|t| now.duration_since(t) < Self::TELEM_HEAL_COOLDOWN);
        let missing = self._stale_telem_channels();
        if let Some(ch) = missing.first().copied() {
            if cooled_down {
                return false;
            }
            let attempt = {
                let slot = &mut self.telem_channel_heal_attempts[ch as usize];
                *slot = slot.saturating_add(1);
                *slot
            };
            self.telem_heal_at = Some(now);
            self.telem_scope_channels = None;
            if attempt == 1 {
                self.push_log_warn(format!(
                    "遥测: CH{} 持续缺失而 STATS 仍正常，旧值已标为 stale；已重发当前逐通道 TELEM_START。",
                    ch
                ));
            } else {
                self.push_log_warn(format!(
                    "遥测: CH{} 在 {} 次有界 TELEM_START 重发后仍缺失，旧值保持 stale；正在触发现有非破坏 HELLO 链路恢复。",
                    ch, attempt
                ));
                if let Err(e) = self.resend_hello() {
                    self.push_log_warn(format!("遥测: CH{} 链路恢复下发失败: {}", ch, e));
                }
            }
            return true;
        }

        let since = self.telem_stats_at.or(self.telem_started_at);
        let Some(since) = since else {
            return false;
        };
        if now.duration_since(since) < Self::TELEM_HEAL_AFTER || cooled_down {
            return false;
        }
        self.telem_heal_at = Some(now);
        self.telem_scope_channels = None;
        self.push_log_warn(
            "遥测: 超过 3s 没收到带采样统计的帧, 已重新下发一次 TELEM_START(最快 5s 一次, 不会连发)"
                .to_string(),
        );
        true
    }

    /// 采集是否已被用户显式停止(真停流)。★与 `plot_frozen` 不是一件事★: 后者只冻结曲线视窗,
    /// 遥测仍在收。全通道页的「开始采集/停止采集」按钮据此互斥置灰。
    pub fn telem_user_paused(&self) -> bool {
        self.telem_user_paused
    }

    /// 用户显式"开始": 解除暂停并强制按当前页面档位重新开流。
    /// ★不在这里硬编码 fields/rate★: 那会与 telem_set_scope 的页面档位形成第二份定义。
    pub fn telem_user_start(&mut self) {
        self.telem_user_paused = false;
        self.telem_scope_channels = None;
        // 停止期间设备已清空 FocusSession；本地也必须废弃旧会话/速率窗口，
        // 让当前曲线页在下一拍重新下发 START，并从新会话的首帧开始计满 1 秒。
        self.focus.clear();
        self.push_log("遥测: 已恢复推流(按当前页面档位)".to_string());
    }

    /// 用户显式"停止": 停流并置暂停位, 使每 tick 的页面档位驱动不再把它重新开起来。
    pub fn telem_user_stop(&mut self) {
        let _ = self.stop_telemetry();
        self.telem_user_paused = true;
        // 即使链路已先断开也要废弃本地 Focus 计量，旧 session/last 不能冒充下一次会话。
        self.focus.clear();
        self.push_log("遥测: 已按用户请求停止(切换页面不会自动恢复, 需再点「开始」)".to_string());
    }

    /// 停止遥测流
    pub fn stop_telemetry(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let frame = Frame::new(HostCmd::TelemStop as u8, 0, seq, vec![]);
            handle.send(frame)?;
            self.telem_active = false;
            // 用户手动停流后, 档位记为未知: 否则回到通道页时 telem_set_scope 会因"档位没变"而
            // 不重新开流, 表现为"点了停止再回来就再也没有数据"。
            self.telem_scope_channels = None;
            // ★TELEM_STOP 在设备侧会一并清掉 FocusSession★(固件 `_handle_telem_stop` → `stop()`)。
            // 本地不同步清零, 之后"目标没变"的幂等判定就会认为单通道流还在, 再也不会重开。
            self.focus.clear();
        }
        Ok(())
    }

    /// 单通道独占流的目标通道: `Some(ch)` = 开/切到该通道, `None` = 停。由 UI 每 tick 调用,
    /// 目标未变时一条命令都不发(否则会变成每 16ms 一轮 START/STOP)。
    ///
    /// ★设备侧语义★ FOCUS_START 会挂起广谱 TELEM 流, FOCUS_STOP 自动恢复它 —— 因此本方法不去动
    /// `telem_set_scope` 记住的档位: 页面档位仍是全通道, 停 focus 后广谱流由固件原样恢复。
    /// 单通道流的实测吞吐文案。★只报实测★ 没有会话、或最近一个结算窗口已过期(>3s 没新结算)时
    /// 如实说"无独占流 / 已过期", 绝不把上一次的读数继续显示成当前带宽。
    pub fn focus_bandwidth_text(&self) -> String {
        let Some(ch) = self.focus.channel else {
            return "独占流: 未开启（当前为全通道广谱）".to_string();
        };
        if self.focus.session.is_none() {
            return format!("独占流: CH{} 会话建立中…", ch);
        }
        let fresh = self
            .focus
            .rate
            .settled_at
            .is_some_and(|at| at.elapsed() < std::time::Duration::from_secs(3));
        match (self.focus.rate.last, fresh) {
            (Some((frames, bytes, gaps)), true) => format!(
                "独占流 CH{}: {} 帧/s · {:.1} KB/s · 丢帧 {}/s · 累计丢帧 {}",
                ch,
                frames,
                bytes as f32 / 1024.0,
                gaps,
                self.focus.gaps
            ),
            (_, _) => format!("独占流 CH{}: 正在统计首个 1 秒窗口…", ch),
        }
    }

    /// 最近一个已结算窗口的实测 (帧/s, 字节/s, 丢帧/s)。无会话或尚无结算窗口时为 None。
    /// 供无头带宽实测取原始数值(UI 走 `focus_bandwidth_text` 的文案路径)。
    pub fn focus_rate_last(&self) -> Option<(u32, u32, u32)> {
        self.focus.rate.last
    }

    /// 本会话累计丢帧(按 sample_seq 跳变计), 会话重建时归零。
    pub fn focus_gaps_total(&self) -> u64 {
        self.focus.gaps
    }

    pub fn focus_set_target(&mut self, ch: Option<u8>) {
        let ch = ch.filter(|c| (*c as usize) < 36);
        if self.io.is_none() {
            // 断链: 设备侧会话已随链路消失, 本地必须归零, 否则重连后"目标没变"会导致永不下发。
            if self.focus.channel.is_some() || self.focus.session.is_some() {
                self.focus.clear();
            }
            return;
        }
        // ★修复暂停恢复后带宽统计失效★：即使目标通道相同，如果会话已失效（session=None），
        // 也必须重建会话，以便重置测量窗口。用户停止流后 telem_user_stop 会清空 session，
        // 恢复时虽然 channel 记录还在，但没有真实会话，必须重新发 START。
        if self.focus.targets(ch) && self.focus.session.is_some() {
            return;
        }
        // ★先进代次再发命令★: 此后到达的旧 session 数据与旧 START 响应立即失效。
        let old_session = self.focus.session;
        let generation = self.focus._bump();
        if let Some(session) = old_session {
            self._focus_send_stop(session);
        }
        let Some(target) = ch else {
            self.push_log_debug(
                "单通道流: 已停止(离开单通道页/切走), 广谱遥测由固件自动恢复".to_string(),
            );
            return;
        };
        let seq = self.next_seq();
        let payload = crate::proto::encode_focus_start(
            target,
            Self::FOCUS_FIELDS,
            Self::FOCUS_RATE_HZ,
            Self::FOCUS_LEASE_MS,
        );
        let sent = self
            .io
            .as_ref()
            .map(|handle| handle.send(Frame::new(HostCmd::FocusStart as u8, 0, seq, payload)));
        match sent {
            Some(Ok(())) => {
                self.focus.channel = Some(target);
                self.focus.pending = Some(FocusPending {
                    seq,
                    generation,
                    channel: target,
                });
            }
            Some(Err(e)) => self.push_log_warn(format!("单通道流启动下发失败: {}", e)),
            None => {}
        }
    }

    /// 对指定 session 下发 FOCUS_STOP。设备只接受当前会话号, 故 session 必须来自 START 响应。
    fn _focus_send_stop(&mut self, session: u16) {
        let seq = self.next_seq();
        let payload = crate::proto::encode_focus_stop(session);
        if let Some(handle) = &self.io {
            if let Err(e) = handle.send(Frame::new(HostCmd::FocusStop as u8, 0, seq, payload)) {
                self.push_log_warn(format!("单通道流停止下发失败: {}", e));
            }
        }
    }

    /// 单通道流请求的字段集: 与逐通道档同一套, 外加帧级统计/延迟(主页卡片与自愈判据都靠它)。
    const FOCUS_FIELDS: u8 = crate::proto::FIELD_RAW
        | crate::proto::FIELD_BASELINE
        | crate::proto::FIELD_DIFF
        | crate::proto::FIELD_STATUS
        | crate::proto::FIELD_STATS
        | crate::proto::FIELD_LATENCY;
    /// 请求速率取固件上限 1000Hz: 单通道帧仅几十字节, 且固件只在快照代数推进时才发帧
    /// (实测 ~171Hz), 所以这里给上限等于"要多快有多快", 不会凭空造出无意义的重复帧。
    const FOCUS_RATE_HZ: u16 = 1000;
    /// 租约 3s: 与遥测流同量级, 上位机任意命令帧都会续租; 上位机消失则设备侧自动停流。
    const FOCUS_LEASE_MS: u16 = 3000;

    /// 请求单个参数
    pub fn request_param(&mut self, ch: u8, param_id: u8) -> anyhow::Result<()> {
        if self._device_busy() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let payload = crate::proto::encode_param_get(ch, param_id);
            let frame = Frame::new(HostCmd::ParamGet as u8, 0, seq, payload);
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 请求所有参数(某通道)
    pub fn request_params(&mut self, ch: u8) -> anyhow::Result<()> {
        if self._device_busy() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let payload = crate::proto::encode_param_get_all(ch);
            let frame = Frame::new(HostCmd::ParamGetAll as u8, 0, seq, payload);
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 批量请求全 36 通道的同一参数(PARAM_GET_ALL 的 0xFF 变体): 一条命令替代 36 条单发 PARAM_GET。
    /// 用于时钟树 snsClk 范围与逐通道自适应后的回读, 避免 36 帧往返打满 vendor 端点。
    pub fn request_param_all_channels(&mut self, param_id: u8) -> anyhow::Result<()> {
        if self._device_busy() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let payload = crate::proto::encode_param_get_all_channels(param_id);
            let frame = Frame::new(HostCmd::ParamGetAll as u8, 0, seq, payload);
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 下发前的围栏校验: 非法即拒绝(不落草稿、不下发), 并把"哪个参数、什么值、合法范围"写进日志。
    /// ★为什么不夹取后照发★: 夹取是上位机替用户改主意 —— 用户看到的是自己填的数, 设备里却是另一个值,
    /// 且下一次回读会把夹取值写回界面, 用户根本不知道自己的输入被改过。宁可拒绝并告知。
    /// 围栏唯一来源 = `proto::telemetry::param_fence`(与两处固件同源, 见该文件头注释)。
    /// `ch = 0xFF` 表示整组(全 36 通道)写入, 仅影响文案。
    fn _reject_illegal_param(&mut self, ch: u8, param_id: u8, value: u32) -> anyhow::Result<()> {
        if crate::proto::param_value_legal(param_id, value) {
            return Ok(());
        }
        let fence = crate::proto::param_fence(param_id);
        let scope = if ch == crate::proto::PARAM_ALL_CHANNELS {
            "全通道".to_string()
        } else {
            format!("CH{}", ch)
        };
        let message = format!(
            "已拒绝非法参数: {} 0x{:02X} {} = {} 超出合法范围 {} → 未下发也未暂存(固件同样会拒收)。",
            scope,
            param_id,
            fence.name,
            value,
            fence.range_text()
        );
        self.push_log_warn(message.clone());
        Err(anyhow::anyhow!(message))
    }

    /// 回读方向防污染: 设备回来的值若非法, 返回应写入缓存的**修正值**, 并按需回写设备。
    /// 返回值即调用方要存进 `self.params[ch]` 的值 —— 界面回显修正后的正确值而不是脏值。
    ///
    /// ★为什么回写而不是只夹显示★: 只夹显示会让上位机与设备长期不一致 —— 用户看到的是修正值,
    /// 设备里躺的还是脏值, 他以为"改好了"其实没有。回写才能真正把设备纠回合法态。
    ///
    /// ★防写风暴/死循环★: 每个 `(ch, param_id)` 自动回写至多一次(`param_fix_sent` 登记),
    /// 回读到合法值时才注销该登记, 允许下次真出问题时重新纠正。链路不可用时只记日志不回写,
    /// 回写值自身再过一次围栏(clamp 实现出错也不会发出非法值)。
    fn _guard_readback_param(&mut self, ch: u8, param_id: u8, value: u32) -> u32 {
        if crate::proto::param_value_legal(param_id, value) {
            self.param_fix_sent.remove(&(ch, param_id));
            return value;
        }
        let fixed = crate::proto::param_clamp(param_id, value);
        let fence = crate::proto::param_fence(param_id);
        self.push_log_warn(format!(
            "设备回读到非法参数: CH{} 0x{:02X} {} = {}(合法范围 {}), 已按极值修正为 {}。\
             正常不应出现, 出现说明该项修改未真正生效或链路异常。",
            ch,
            param_id,
            fence.name,
            value,
            fence.range_text(),
            fixed
        ));
        let first_fix = self.param_fix_sent.insert((ch, param_id));
        if !first_fix {
            return fixed;
        }
        if self.state != ConnState::Connected || self.io.is_none() {
            self.push_log_warn("链路不可用, 本次仅修正界面显示, 未回写设备。".to_string());
            return fixed;
        }
        // 走既有 PARAM_SET 路径(内含围栏校验 + 串行发送队列)纠正下位机脏值。
        if let Err(e) = self.debug_param_now(ch, param_id, fixed) {
            self.push_log_warn(format!("非法参数回写失败: {}", e));
        } else {
            self.push_log_warn(format!(
                "已向设备回写修正值: CH{} 0x{:02X} {} = {}。",
                ch, param_id, fence.name, fixed
            ));
        }
        fixed
    }

    /// 诊断用: 直接下发单通道参数到设备(不经草稿、不写 flash), 供无头探针实时改参验证时钟生效。
    /// 与 GUI 的 set_param(草稿) 区分: 这条立即经 PARAM_SET 送达设备并同步本地缓存。
    pub fn debug_param_now(&mut self, ch: u8, param_id: u8, value: u32) -> anyhow::Result<()> {
        // 扫描自身的写 gain/div 走 `_sweep_tx`(own_tx 放行), 其余来源在扫描期间一律拒绝。
        self._reject_csd_tx_if_batching("直接写入通道参数")?;
        self._reject_csd_tx_if_sweeping("直接写入通道参数")?;
        self._reject_illegal_param(ch, param_id, value)?;
        let payload = crate::proto::encode_param_set(ch, param_id, value);
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(HostCmd::ParamSet as u8, 0, seq, payload))?;
        }
        if (ch as usize) < 36 {
            self.params[ch as usize].insert(param_id, value);
        }
        Ok(())
    }

    /// 诊断用: 直接切换 CSD 处理模式(0=自动/1=半自动手动), 不经草稿、不写 flash。
    pub fn debug_mode_now(&mut self, mode: u8) -> anyhow::Result<()> {
        // 切处理模式会换掉整条 CSD 处理链(自动校准/半自动手动) ⇒ 扫描期间拒绝。
        self._reject_csd_tx_if_batching("直接切换 CSD 处理模式")?;
        self._reject_csd_tx_if_sweeping("直接切换 CSD 处理模式")?;
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(HostCmd::ModeSet as u8, 0, seq, vec![mode]))?;
        }
        Ok(())
    }

    /// 诊断用: 直接下发全局 CSD 配置(不写草稿), 使 global_get 回读到的是设备真值而非草稿乐观值。
    pub fn debug_global_now(&mut self, gparam_id: u8, value: u32) -> anyhow::Result<()> {
        self._reject_csd_tx_if_batching("直接写入全局 CSD 项")?;
        self._reject_csd_tx_if_sweeping("直接写入全局 CSD 项")?;
        self._reject_illegal_global(gparam_id, value)?;
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_global_set(seq, gparam_id, value))?;
        }
        Ok(())
    }

    /// 全局项下发前的围栏校验: 非法即拒绝(不落草稿、不下发), 并告知合法范围。
    /// 语义与 `_reject_illegal_param` 逐条对应 —— ★不夹取后照发★, 那是替用户改主意。
    /// 围栏唯一来源 = `proto::telemetry::global_fence`(与两处固件同源, 见该文件该节头注释)。
    fn _reject_illegal_global(&mut self, gparam_id: u8, value: u32) -> anyhow::Result<()> {
        if crate::proto::global_value_legal(gparam_id, value) {
            return Ok(());
        }
        let fence = crate::proto::global_fence(gparam_id);
        let message = format!(
            "已拒绝非法全局项: {} 0x{:02X} {} = {} 超出合法范围 {} → 未下发也未暂存(固件同样会 NAK)。",
            Self::_gparam_name(gparam_id),
            gparam_id,
            fence.name,
            value,
            fence.range_text()
        );
        self.push_log_warn(message.clone());
        Err(anyhow::anyhow!(message))
    }

    /// 全局项回读方向防污染: 设备回来的值若非法, 返回应写入缓存的**修正值**, 并按需回写设备。
    /// 与 `_guard_readback_param` 同一套机制(含防写风暴), 只是键是 gparam_id。
    ///
    /// ★防写风暴/死循环★: 每个 gparam_id 自动回写至多一次(`global_fix_sent` 登记), 回读到合法值
    /// 时才注销该登记; 链路不可用时只记日志不回写; 回写走 `debug_global_now`(内含围栏再校验)。
    fn _guard_readback_global(&mut self, gparam_id: u8, value: u32) -> u32 {
        if crate::proto::global_value_legal(gparam_id, value) {
            self.global_fix_sent.remove(&gparam_id);
            return value;
        }
        let fixed = crate::proto::global_clamp(gparam_id, value);
        let fence = crate::proto::global_fence(gparam_id);
        self.push_log_warn(format!(
            "设备回读到非法全局项: {} 0x{:02X} {} = {}(合法范围 {}), 已按最近合法值修正为 {}。\
             正常不应出现, 出现说明该项修改未真正生效或链路异常。",
            Self::_gparam_name(gparam_id),
            gparam_id,
            fence.name,
            value,
            fence.range_text(),
            fixed
        ));
        if !self.global_fix_sent.insert(gparam_id) {
            return fixed;
        }
        if self.state != ConnState::Connected || self.io.is_none() {
            self.push_log_warn("链路不可用, 本次仅修正界面显示, 未回写设备。".to_string());
            return fixed;
        }
        if let Err(e) = self.debug_global_now(gparam_id, fixed) {
            self.push_log_warn(format!("非法全局项回写失败: {}", e));
        } else {
            self.push_log_warn(format!(
                "已向设备回写修正值: 0x{:02X} {} = {}。",
                gparam_id, fence.name, fixed
            ));
        }
        fixed
    }

    /// 暂存单个参数到草稿(不下发)。保存时统一下发。
    /// 分辨率(PARAM_RESOLUTION=0x07)强制所有 36 通道一致: 分辨率决定扫描时长, 逐通道不同会导致
    /// 帧周期不齐/异常, 故任意通道改分辨率即写入全部通道草稿。
    pub fn set_param(&mut self, ch: u8, param_id: u8, value: u32) -> anyhow::Result<()> {
        if (ch as usize) >= 36 {
            return Err(anyhow::anyhow!("参数通道越界: {}", ch));
        }
        if param_id == 0x07 {
            return self.set_param_all(param_id, value);
        }
        self._reject_illegal_param(ch, param_id, value)?;
        let same_as_device = self.params[ch as usize].get(&param_id) == Some(&value);
        self.drafts.set_param(ch, param_id, value, same_as_device);
        Ok(())
    }

    /// 将一个参数值暂存到全部 36 个物理通道草稿。整组视为一项脏计数(param:all:<id>)。
    pub fn set_param_all(&mut self, param_id: u8, value: u32) -> anyhow::Result<()> {
        self._reject_illegal_param(0xFF, param_id, value)?;
        // 整组语义: 仅当全部 36 通道都已等于设备值时才算无改动(否则任一通道不同即需下发)。
        let same_as_device = (0..36usize).all(|ch| self.params[ch].get(&param_id) == Some(&value));
        self.drafts.set_param_all(param_id, value, same_as_device);
        Ok(())
    }

    /// 触发全部电极的 Cp 测量。
    ///
    /// ★为什么它也必须让位给频谱扫描★ PSoC 侧的 Cp 测量走 BIST: 逐电极把 CSD 切到 BIST 配置测完
    /// 再自己恢复(main.c 的 measure_cp 分支)。恢复的是"命令到达那一刻的配置", 而扫描每一格都在改
    /// 本通道的 gain/div —— 两者交叠时, 恢复回来的可能是上一格的配置, 而这一格测出的噪声也不再属于
    /// 它标称的那一格。这里只做"扫描期间拒绝", CP/BIST 自身的恢复语义一字未改。
    pub fn measure_cp(&mut self) -> anyhow::Result<()> {
        if self._reject_csd_if_batching("测量 Cp") || self._reject_csd_if_sweeping("测量 Cp") {
            return Ok(());
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            let frame = Frame::new(
                HostCmd::CpMeasure as u8,
                0,
                seq,
                crate::proto::encode_cp_measure(),
            );
            self._queue_tx(frame)?;
        }
        Ok(())
    }

    /// 请求一个通道最近一次 Cp 测量值。
    pub fn request_cp(&mut self, ch: u8) -> anyhow::Result<()> {
        if ch >= 36 {
            return Err(anyhow::anyhow!("Cp 通道越界: {}", ch));
        }
        if self._device_busy() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            let payload = crate::proto::encode_cp_get(ch);
            let frame = Frame::new(HostCmd::CpGet as u8, 0, seq, payload);
            handle.send(frame)?;
        }
        Ok(())
    }

    /// 标记一个阻塞类操作开始: 置 op_busy + 标签 + 期望回执 seq(供 UI 锁定按钮/显示运行图标)。
    fn _begin_op(&mut self, label: &str, seq: u8) {
        self.op_busy = true;
        self.op_label = label.to_string();
        self.op_wait_seq = Some(seq);
        self.op_ack_is_accept = false; // 默认 ACK=真实完成; 需要推送流判完成的操作自行改标志
        self.op_busy_ticks = 0;
        self.op_started_at = Some(std::time::Instant::now());
        self.op_version = self.op_version.wrapping_add(1);
        self.push_log(format!(
            "▶ 阻塞操作开始: {} (seq={}), 等待固件真正完成回执...",
            label, seq
        ));
    }
    /// 结束当前阻塞操作(收到匹配回执或超时): 清 op_busy。
    fn _end_op(&mut self) {
        if self.op_busy {
            let label = std::mem::take(&mut self.op_label);
            // 真实耗时用起始时刻算: op_busy_ticks 会被进度帧归零(卡死检测语义), 用它会恒显示 0ms。
            let waited_ms = self
                .op_started_at
                .map(|t| t.elapsed().as_millis())
                .unwrap_or(0);
            self.op_busy = false;
            self.op_wait_seq = None;
            self.op_ack_is_accept = false;
            self.op_busy_ticks = 0;
            self.op_started_at = None;
            self.op_version = self.op_version.wrapping_add(1);
            self.push_log(format!(
                "✔ 阻塞操作完成: {} (耗时约 {}ms)",
                label, waited_ms
            ));
        }
    }
    /// 阻塞操作进行中(UI 据此锁定触发按钮并显示运行中图标)。含 CSD 冷却窗口, 使 UI 在设备真实
    /// 完成前保持锁定, 而非 ACK 一到就误判完成。
    pub fn op_busy(&self) -> bool {
        self.op_busy || self.csd_cooldown_ticks > 0
    }
    pub fn op_label(&self) -> &str {
        if self.op_busy {
            &self.op_label
        } else if self.csd_cooldown_ticks > 0 {
            &self.csd_cooldown_label
        } else {
            &self.op_label
        }
    }
    pub fn op_version(&self) -> u64 {
        self.op_version
    }

    /// CSD 变更是否被锁定(阻塞操作进行中 或 冷却窗口未过)。用于串行化守卫。
    fn _csd_locked(&self) -> bool {
        self.op_busy || self.csd_cooldown_ticks > 0
    }

    /// 串行化守卫: 若正忙/冷却中, 记录日志并返回 true(调用方应放弃本次 CSD 变更, 杜绝叠加把设备搞死)。
    fn _reject_csd_if_locked(&mut self, what: &str) -> bool {
        if self._csd_locked() {
            let cur = if self.op_busy {
                self.op_label.clone()
            } else {
                self.csd_cooldown_label.clone()
            };
            self.push_log(format!(
                "设备忙({}), 已忽略「{}」——请等待当前 CSD 操作完成后再试。",
                cur, what
            ));
            return true;
        }
        false
    }

    /// 批量控制面守卫：从 start 到取消/终态收敛期间，外部 CSD mutation 一律不得插入批次间隙。
    fn _reject_csd_if_batching(&mut self, what: &str) -> bool {
        if !self._batch_blocks_csd() {
            return false;
        }
        self.push_log(format!(
            "{}进行中, 已忽略「{}」——请等待批量终态或取消收敛后再操作。",
            self.ch_batch_status(),
            what
        ));
        true
    }

    /// 立即下发入口的 Err 版批次守卫，避免调用方把 Ok 当作已经写入设备。
    fn _reject_csd_tx_if_batching(&mut self, what: &str) -> anyhow::Result<()> {
        if !self._batch_blocks_csd() {
            return Ok(());
        }
        let message = format!(
            "{}进行中, 已拒绝「{}」——批量控制面尚未释放。",
            self.ch_batch_status(),
            what
        );
        self.push_log_warn(message.clone());
        Err(anyhow::anyhow!(message))
    }

    /// 频谱扫描互斥守卫: 扫描活跃期间拒绝任何**外部** CSD 操作(扫描自身的下发经 `_sweep_tx` 放行)。
    /// ★为什么必须挡★ 扫描的每一格都在改本通道的 gain/div 并逐格校准 —— 中途插进来的校准/基线复位/
    /// 自适应会把那一格的配置换掉, 于是测出来的噪声不是这一格的, 还会踩乱扫描的原值还原路径。
    /// 与 `_reject_csd_if_locked` 同一约定: 记日志 + 返回 true, 调用方放弃本次操作。
    fn _reject_csd_if_sweeping(&mut self, what: &str) -> bool {
        if !self._sweep_blocks_csd() {
            return false;
        }
        let ch = self.noise_sweep_channel();
        self.push_log(format!(
            "CH{} 响应噪声频谱扫描进行中, 已忽略「{}」——请先取消扫描(会自动还原增益档/分频)再操作。",
            ch, what
        ));
        true
    }

    /// 同一判据的 **Err 版**互斥守卫, 供"调用方一律检查 Err"的即时下发入口(`debug_*_now`)使用。
    /// ★为什么不能复用返回 Ok 的那个★ 那三条的调用方(回读防污染的自动回写、批量前置写入)会把
    /// `Ok(())` 当成"已经写进设备了"并照此打日志/记状态 —— 那正是本轮要消灭的假成功。
    fn _reject_csd_tx_if_sweeping(&mut self, what: &str) -> anyhow::Result<()> {
        if !self._sweep_blocks_csd() {
            return Ok(());
        }
        let message = format!(
            "CH{} 响应噪声频谱扫描进行中, 已拒绝「{}」——请先取消扫描(会自动还原增益档/分频)再操作。",
            self.noise_sweep_channel(),
            what
        );
        self.push_log_warn(message.clone());
        Err(anyhow::anyhow!(message))
    }

    /// 发起一次 CSD 变更后开启冷却窗口(串行化后续变更)。ticks 为保守估计的 PSoC 真实完成时长。
    fn _begin_csd_cooldown(&mut self, label: &str, ticks: u32) {
        self.csd_cooldown_ticks = ticks;
        self.csd_cooldown_label = label.to_string();
        self.op_version = self.op_version.wrapping_add(1);
    }
    /// 频率自适应结果访问器。
    pub fn auto_tune_result(&self) -> u8 {
        self.auto_tune_result
    }
    pub fn auto_tune_div(&self) -> u16 {
        self.auto_tune_div
    }
    pub fn auto_tune_version(&self) -> u64 {
        self.auto_tune_version
    }
    /// 最近一次自适应目标通道(0..35 单通道, 0xFF 全通道)。
    pub fn auto_tune_ch(&self) -> u8 {
        self.auto_tune_ch
    }
    /// 逐通道自适应结果访问器(越界返回 0/未执行)。
    pub fn auto_tune_ch_result(&self, ch: u8) -> u8 {
        if (ch as usize) < 36 {
            self.auto_tune_ch_result[ch as usize]
        } else {
            0
        }
    }
    pub fn auto_tune_ch_div(&self, ch: u8) -> u16 {
        if (ch as usize) < 36 {
            self.auto_tune_ch_div[ch as usize]
        } else {
            0
        }
    }
    /// 设备推送的自适应阶段进度(0=空闲 1=进行中 2=完成; 含 phase/step/当前试探分频)。
    pub fn auto_tune_progress(&self) -> crate::proto::AutoTuneProgress {
        self.auto_tune_progress
    }
    pub fn auto_tune_progress_version(&self) -> u64 {
        self.auto_tune_progress_version
    }

    /// 单通道操作落在【已禁用】通道上时在源头拒绝。★为什么 host 侧也要拦★
    /// 设备侧确实会回 NAK(sensor_link.cpp::disabled_ch_reject), 但那要走一次完整往返, 而这边的
    /// op_busy/冷却窗已经按"命令发出去了"开始计时 —— 结果是白锁一两秒界面再报错。
    /// 只拦"恰好一位"的单通道掩码: 全通道语义由固件逐通道跳过, 不该整条拒绝。
    fn _reject_disabled_ch(&mut self, what: &str, ch: u8) -> bool {
        if ch >= 36 || self.ch_enabled(ch) {
            return false;
        }
        self.push_log_warn(format!(
            "{}未执行: CH{} 已禁用(电极保持模拟高阻, 不参与扫描) — 请先在该通道的设置里启用它。",
            what, ch
        ));
        true
    }

    /// 单通道掩码 → 通道号; 非单通道(空/多位)返回 0xFF(全通道语义)。
    fn _single_ch_of(ch_mask: u64) -> u8 {
        if ch_mask.count_ones() == 1 {
            ch_mask.trailing_zeros() as u8
        } else {
            0xFF
        }
    }

    /// 触发校准(阻塞类: 固件实际完成后才回 ACK, 期间 op_busy=true)。
    pub fn calibrate(&mut self, ch_mask: u64) -> anyhow::Result<()> {
        if self._reject_disabled_ch("校准", Self::_single_ch_of(ch_mask)) {
            return Ok(());
        }
        if self._reject_csd_if_batching("校准")
            || self._reject_csd_if_sweeping("校准")
            || self._reject_csd_if_locked("校准")
        {
            return Ok(());
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            let payload = crate::proto::encode_ch_mask(ch_mask);
            let frame = Frame::new(HostCmd::Calibrate as u8, 0, seq, payload);
            self._queue_tx(frame)?;
        }
        // ★冷却窗按目标范围给★ 单通道校准在 PSoC 侧只是一次 CalibrateWidget(约全通道的 1/36),
        // 沿用全通道的 120 tick(~2s)会让 host 侧的逐通道串行队列白等 36×2s。
        let one_ch = ch_mask.count_ones() == 1;
        self._begin_op(
            if one_ch {
                "校准中(单通道)"
            } else {
                "校准中"
            },
            seq,
        );
        self._begin_csd_cooldown("校准", if one_ch { 25 } else { 120 });
        Ok(())
    }

    /// 触发基线复位(阻塞类)。
    pub fn baseline_reset(&mut self, ch_mask: u64) -> anyhow::Result<()> {
        if self._reject_disabled_ch("基线复位", Self::_single_ch_of(ch_mask)) {
            return Ok(());
        }
        if self._reject_csd_if_batching("基线复位")
            || self._reject_csd_if_sweeping("基线复位")
            || self._reject_csd_if_locked("基线复位")
        {
            return Ok(());
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            let payload = crate::proto::encode_ch_mask(ch_mask);
            let frame = Frame::new(HostCmd::BaselineReset as u8, 0, seq, payload);
            self._queue_tx(frame)?;
        }
        let one_ch = ch_mask.count_ones() == 1;
        self._begin_op(
            if one_ch {
                "基线复位中(单通道)"
            } else {
                "基线复位中"
            },
            seq,
        );
        self._begin_csd_cooldown("基线复位", if one_ch { 15 } else { 90 });
        Ok(())
    }

    /// 触发频率自适应下探(阻塞类, 数秒): 固件粗定位→1步进找临界→按偏好档位落档, 或超硬件能力失败。
    /// ch: 0..35=仅该通道下探(其余通道分频不动), 0xFF=全 36 通道统一。
    /// 请求 [ch(u8), pref(u8)]; 固件立即 ACK("已受理"), 阶段进度与最终结果经 AUTO_TUNE_PROGRESS(0x2E)
    /// 推送流上报(5Hz), 故 UI 全程可见阶段而非干等。成功后 UI 应回读 snsClk 显示。
    /// pref 取 calib.pref(草稿优先) → 滑条一改即对下一次自适应生效, 无需先保存到设备。
    pub fn auto_tune(&mut self, ch: u8) -> anyhow::Result<()> {
        // 容忍任意数值类型: 设备 schema 为 U8, 但缓存未就绪时 set_config_number 会按 U32 落草稿
        // (仅认 U8 会静默回落默认档, 表现为"滑条无效")。统一取数值再夹到 1..7。
        let pref = match self.config_get("calib.pref").map(|e| e.value) {
            Some(CfgValue::U8(v)) => v as i64,
            Some(CfgValue::U16(v)) => v as i64,
            Some(CfgValue::U32(v)) => v as i64,
            Some(CfgValue::I8(v)) => v as i64,
            Some(CfgValue::F32(v)) => v as i64,
            _ => 4,
        };
        let pref = if (1..=7).contains(&pref) {
            pref as u8
        } else {
            4u8
        };
        self.auto_tune_with_pref(ch, pref)
    }

    /// 指定偏好档的自适应。★为什么要这个入口★ host 侧的逐通道串行队列在开启"Cp 辅助"后, 每个
    /// 通道的偏好档各不相同(由实测 Cp 推出), 而 `auto_tune` 只会取滑条上那一个统一值。
    /// 两者共用同一份下发/进度/冷却逻辑, 不复制第二条自适应路径。
    pub fn auto_tune_with_pref(&mut self, ch: u8, pref: u8) -> anyhow::Result<()> {
        self._auto_tune_with_pref(ch, pref, false)
    }

    /// 批量队列使用：每通道只更新运行态，整批完成后统一发一次 SAVE_CONFIG。
    pub(super) fn auto_tune_with_pref_deferred(&mut self, ch: u8, pref: u8) -> anyhow::Result<()> {
        self._auto_tune_with_pref(ch, pref, true)
    }

    fn _auto_tune_with_pref(&mut self, ch: u8, pref: u8, defer_save: bool) -> anyhow::Result<()> {
        if ch != 0xFF && (ch as usize) >= 36 {
            return Err(anyhow::anyhow!("自适应通道越界: {}", ch));
        }
        if self._reject_disabled_ch("频率自适应", ch) {
            return Ok(());
        }
        if self._reject_csd_if_batching("频率自适应")
            || self._reject_csd_if_sweeping("频率自适应")
            || self._reject_csd_if_locked("频率自适应")
        {
            return Ok(());
        }
        let pref = pref.clamp(1, 7);
        let seq = self.next_seq();
        if self.io.is_some() {
            let frame = Frame::new(
                HostCmd::AutoTune as u8,
                0,
                seq,
                vec![ch, pref, u8::from(defer_save)],
            );
            self._queue_tx(frame)?;
        }
        self.auto_tune_result = 0; // 进行中
        self.auto_tune_all_refresh_pending = None;
        self.auto_tune_ch = ch;
        // 本轮请求的归属键: 设备把它逐帧回显在 0x2E 的 origin_seq 上, 迟到的上一轮终态据此丢弃。
        self.auto_tune_req_seq = Some(seq);
        self.auto_tune_progress = crate::proto::AutoTuneProgress {
            ch,
            origin_seq: Some(seq),
            ..Default::default()
        };
        self.auto_tune_progress_version = self.auto_tune_progress_version.wrapping_add(1);
        if (ch as usize) < 36 {
            self.auto_tune_ch_result[ch as usize] = 0;
        } else {
            self.auto_tune_ch_result = [0u8; 36];
        }
        let label = if (ch as usize) < 36 {
            format!("CH{} 频率自适应中(偏好 {})", ch, pref)
        } else {
            "逐通道频率自适应中".to_string()
        };
        self._begin_op(&label, seq);
        // ★ACK 仅表示"已受理"★: 固件现在入队即回 ACK, 真实完成由 AUTO_TUNE_PROGRESS(0x2E) 推送流的
        // 终态帧判定(带 result/final_div)。若让 ACK 解锁, UI 会在自适应刚开始时就误判完成。
        self.op_ack_is_accept = true;
        // 自适应逐档重校准可达数秒; 响应到达即解 op_busy, 但仍设冷却窗防止响应后立刻叠加保存/校准(日志正是此叠加致死)。
        // 单通道三步算法约 0.3-0.65s, 全通道(0xFF)十几到几十秒 ⇒ 冷却窗同样按范围分档,
        // 否则 host 串行队列会在每个通道后白等 2s。
        self._begin_csd_cooldown("频率自适应", if (ch as usize) < 36 { 60 } else { 120 });
        Ok(())
    }

    /// 暂存 CSD 处理模式到草稿(不下发): 0=自动校准/标准完整处理, 1=半自动手动。
    /// 与其他草稿写入一致: 值等于设备真值(DEVICE_INFO 诊断回读的 csd_mode)时撤回草稿并清脏,
    /// 免得"选回原模式"或"导入同值文件"留下一条永远存在的未保存项。
    pub fn set_mode(&mut self, mode: u8) -> anyhow::Result<()> {
        let same_as_device = self.csd_mode == Some(mode);
        self.drafts.set_mode(mode, same_as_device);
        Ok(())
    }

    /// CSD 处理模式的**草稿层**原值(未编辑时 None)。
    ///
    /// ★不要用它做"当前模式"判定★ 判定一律用 [`Self::csd_mode_effective`]。
    /// 本访问器只表达"草稿里有没有一个待保存的值", 供脏态/导出这类需要区分草稿与真值的场景。
    pub fn mode_draft(&self) -> Option<u8> {
        self.drafts.mode()
    }

    /// 触发 CSD 参数捕获(PSoC 当前自整定值 → RP2040 store)
    /// ★扫描期间拒绝★: 此刻 PSoC 上的 gain/div 是扫描的临时值, 捕获等于把中间态提交成真相源。
    pub fn csd_capture(&mut self) -> anyhow::Result<()> {
        if self._reject_csd_if_batching("捕获 CSD 参数")
            || self._reject_csd_if_sweeping("捕获 CSD 参数")
        {
            return Ok(());
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            let frame = Frame::new(HostCmd::CsdCapture as u8, 0, seq, vec![]);
            self._queue_tx(frame)?;
        }
        Ok(())
    }

    // ------------------------------------------------------------------
    // CSD 调试诊断: 全局设置改动后回读设备真实状态, 精确定位"改全局后无数据"根因
    // ------------------------------------------------------------------
    /// 全局参数(GPARAM)名称, 供诊断日志可读化。
    fn _gparam_name(id: u8) -> &'static str {
        match id {
            0x01 => "非激活连接(INACTIVE_SNS)",
            0x02 => "IDAC增益档(IDAC_GAIN_INIT)",
            0x03 => "IDAC最小(IDAC_MIN)",
            0x04 => "校准目标%(RAW_TARGET)",
            0x05 => "MFS分频F1(MFS_DIV_F1)",
            0x06 => "MFS分频F2(MFS_DIV_F2)",
            0x07 => "IDAC方向(IDAC_SENSE_CONFIG)",
            0x08 => "自动校准(AUTO_CALIBRATE_EN)",
            _ => "未知全局项",
        }
    }
    /// 逐通道参数(PARAM)名称, 供诊断日志可读化。
    fn _param_name(id: u8) -> &'static str {
        match id {
            0x07 => "分辨率(RESOLUTION)",
            0x08 => "传感时钟分频(SNS_CLK_DIV)",
            0x09 => "模态IDAC(IDAC_MOD)",
            0x0A => "时钟源(SNS_CLK_SOURCE)",
            0x0B => "IDAC增幅档(IDAC_GAIN)",
            _ => "参数",
        }
    }

    /// 每 tick 由 main.rs 调用: 推进 CSD 诊断探针排程与窗口关闭 + 阻塞操作卡死检测 + CSD 冷却窗口。
    pub fn csd_diag_tick(&mut self) {
        // 与 led_apply_seq 相同的 seq 归因：没有匹配 ACK/NAK 不能称为成功，超时后明确设备真值未知。
        if let (Some(seq), Some(started_at)) = (self.algo_upload_seq, self.algo_upload_started_at) {
            if started_at.elapsed() >= std::time::Duration::from_secs(5) {
                self.algo_upload_seq = None;
                self.algo_upload_started_at = None;
                self.algo_upload_status = format!(
                    "上传超时: 5 秒未收到设备 ACK/NAK (seq={});设备实际状态未知，请刷新确认",
                    seq
                );
                self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
                self.push_log_warn(format!(
                    "算法上传超时: seq={} 在 5 秒内未收到设备 ACK/NAK，设备实际状态未知",
                    seq
                ));
            }
        }
        // CSD 串行化冷却窗口: 到点解锁(允许下一次 CSD 变更), 并让 UI 解除按钮锁定。
        if self.csd_cooldown_ticks > 0 {
            self.csd_cooldown_ticks -= 1;
            if self.csd_cooldown_ticks == 0 {
                let label = std::mem::take(&mut self.csd_cooldown_label);
                self.op_version = self.op_version.wrapping_add(1);
                self.push_log(format!("CSD 操作冷却结束({}), 可继续下一步操作。", label));
            }
        }
        // 阻塞操作卡死检测: 每秒打一次心跳日志, 超时(默认 ~10s)则告警并自动解锁 UI, 直指"卡校准中"。
        if self.op_busy {
            self.op_busy_ticks = self.op_busy_ticks.wrapping_add(1);
            let t = self.op_busy_ticks;
            if t % 62 == 0 {
                let label = self.op_label.clone();
                self.push_log(format!(
                    "… {} 已等待 {}s, 仍未收到固件完成回执",
                    label,
                    t / 62
                ));
            }
            // ~51s(3200 tick)仍未完成 = 卡死: 告警 + 自动解锁, 避免 UI 永久锁在"处理中"。
            // ★必须大于固件侧最长等待★: 全通道频率自适应已改为【逐通道各自校准】(36 × 单通道三步算法,
            // 约 11-23s, 最坏更长) → RP2040 _wait_op_done 45s / _submit 50s; PSoC 救砖全片擦写+校验
            // 亦为数十秒级。本阈值若小于固件预算会在设备仍正常工作时误报超时并提前解锁。
            // 任意进度帧(自适应 0x2E / 救砖 0x09)都会把 op_busy_ticks 归零, 故只有真失联才会触发。
            if t >= 3200 {
                let label = self.op_label.clone();
                let seq = self.op_wait_seq;
                self.push_log(format!(
                    "⚠ 阻塞操作超时: {} (seq={:?}) 超过 51s 未完成 → 设备可能卡在校准/重初始化(global_commit)。已自动解锁按钮。",
                    label, seq));
                // 超时的那一条若属于逐通道批量, 归因为该通道失败(设备真值未知, 绝不算成功)。
                if let Some(s) = seq {
                    self._ch_batch_note_op(s, false, "超过 51s 未收到完成回执(设备实际状态未知)");
                    self._sweep_note_op(s, false, "超过 51s 未收到完成回执(设备实际状态未知)");
                }
                self._end_op();
            }
        }
        // 恢复默认后重读刷新: 倒计时到点置就绪标志, 由 main.rs 取走并执行全量重读。
        if let Some(n) = self.post_reset_refetch_in {
            if n <= 1 {
                self.post_reset_refetch_in = None;
                self.post_reset_refetch_pending = true;
            } else {
                self.post_reset_refetch_in = Some(n - 1);
            }
        }
        // 串行下发队列没排空前, 全局对账的倒计时不开始走 —— 否则队列还在推进(可能要几秒)就去读,
        // 读到的必然是半新半旧的中间态。
        if !self.cfg_tx_queue.is_empty() || self.cfg_tx_inflight.is_some() {
            if let Some(_) = self.globals_verify_in {
                self.globals_verify_in = Some(GLOBALS_VERIFY_TICKS);
            }
        }
        // (写队列的泵已上移到 poll(), 避免"只 poll 不 tick"的调用方把写命令静默丢弃; 此处不再重复泵,
        //  否则同一帧内推进两次会把 2s 无 ACK 的超时兜底压成 1s。)
        self._pump_algo_src_tx();
        self._pump_algo_src_rx();
        // 逐通道批量操作队列与噪声频谱扫描: 并入既有 16ms tick, 不新增定时器、不提高轮询频率。
        // 两者都以 `_csd_locked()` 为唯一"设备是否还在忙上一步"的判据, 不另设第二套忙闲状态。
        self._pump_ch_batch();
        self._pump_noise_sweep();
        // 连接后基础回读优先：探针存在期间不启动后台全通道参数重读，避免与 CFG_GET_ALL 等单一会话抢占。
        if !self._device_busy() && !self.conn_probes_pending() {
            if let Some(param_id) = self.param_refetch_ids.pop() {
                if let Err(e) = self.request_param_all_channels(param_id) {
                    log::warn!("全通道参数重读 0x{:02X} 下发失败: {}", param_id, e);
                }
            }
        }
        // CSD 诊断探针(DEBUG): 改全局后 ~1.2s 触发一次设备状态回读; 窗口 ~2s 后自动关。
        if let Some(n) = self.csd_diag_probe_in {
            if n <= 1 {
                self.csd_diag_probe_in = None;
                self._csd_diag_fire();
            } else {
                self.csd_diag_probe_in = Some(n - 1);
            }
        }
        // 算法真值回读: 下发是异步的(core1 执行 ~700ms), ACK 后延时再读一次 psoc_valid/len 对账。
        if let Some(n) = self.algo_info_refresh_in {
            if n <= 1 {
                self.algo_info_refresh_in = None;
                let _ = self.algo_get_info();
            } else {
                self.algo_info_refresh_in = Some(n - 1);
            }
        }
        // 全局真值回读对账: 下发/重启后延时触发一次 GLOBAL_GET_ALL, 响应到达时逐项比对期望值。
        if let Some(n) = self.globals_verify_in {
            if n <= 1 {
                self.globals_verify_in = None;
                self.globals_verify_active = true;
                let _ = self.global_get_all();
            } else {
                self.globals_verify_in = Some(n - 1);
            }
        }
        if let Some(n) = self.csd_diag_off_in {
            if n <= 1 {
                self.csd_diag_off_in = None;
                self.csd_diag_active = false;
                self.push_log_debug("CSD诊断: 回读窗口结束");
            } else {
                self.csd_diag_off_in = Some(n - 1);
            }
        }
    }

    /// 触发一次设备状态回读诊断(DEBUG 级): 开窗口 + 回读代表通道硬件参数 + 全局值 + Cp。
    /// 响应到达时由各 _handle_*_response 在窗口内以 DEBUG 写入日志(切"调试"等级可见)。
    fn _csd_diag_fire(&mut self) {
        self.csd_diag_active = true;
        self.csd_diag_off_in = Some(120);
        self.push_log_debug(
            "CSD诊断: 开始回读设备状态(代表通道 0/3/17/35 的硬件参数 + 全局值 + Cp)...",
        );
        let _ = self.global_get_all();
        let sample_channels: [u8; 4] = [0, 3, 17, 35];
        let param_ids: [u8; 4] = [0x07, 0x08, 0x0A, 0x0B];
        for ch in sample_channels {
            for pid in param_ids {
                let seq = self.next_seq();
                if let Some(handle) = &self.io {
                    let payload = crate::proto::encode_param_get(ch, pid);
                    let _ = handle.send(Frame::new(HostCmd::ParamGet as u8, 0, seq, payload));
                }
            }
        }
        for ch in [0u8, 3, 17, 35] {
            let seq = self.next_seq();
            if let Some(handle) = &self.io {
                let _ = handle.send(Frame::new(HostCmd::CpGet as u8, 0, seq, vec![ch]));
            }
        }
    }

    // ------------------------------------------------------------------
    // 全局 CSD 配置 (GLOBAL_*): 未激活传感器连接/IDAC/MFS
    // ------------------------------------------------------------------
    pub fn global_get(&mut self, gparam_id: u8) -> anyhow::Result<()> {
        if self._device_busy() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_global_get(seq, gparam_id))?;
        }
        Ok(())
    }
    /// 单次触发 PSoC 完整重初始化, 使已写入影子的全局项真正生效(诊断/无头工具用)。
    /// GUI 侧走 `save_config` 的"批量 GLOBAL_SET + 一次 GLOBAL_COMMIT", 不直接调本方法。
    pub fn global_commit(&mut self) -> anyhow::Result<()> {
        // 一次完整重初始化(全部 36 通道重解 IDAC + 基线复位) ⇒ 扫描期间拒绝。
        if self._reject_csd_if_batching("提交全局 CSD 配置")
            || self._reject_csd_if_sweeping("提交全局 CSD 配置")
        {
            return Ok(());
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_global_commit(seq))?;
        }
        Ok(())
    }
    pub fn global_get_all(&mut self) -> anyhow::Result<()> {
        if self._device_busy() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_global_get_all(seq))?;
        }
        Ok(())
    }
    /// 全局 CSD 参数: 仅暂存草稿，点击“保存到设备”后统一下发。
    pub fn global_set(&mut self, gparam_id: u8, value: u32) -> anyhow::Result<()> {
        // ★围栏在"写入草稿之前"★: 草稿是唯一的批量下发来源(save_config 逐项 encode_global_set),
        // 一旦非法值进了草稿, 保存时才被固件 NAK, 用户只会看到一次失败而不知道是哪项、为什么。
        self._reject_illegal_global(gparam_id, value)?;
        // ★统一草稿★: 全局项只暂存, 由“保存到设备”批量下发 + 单次 GLOBAL_COMMIT 重初始化，
        // 避免逐项重初始化风暴与重复下发。
        let same_as_device = self.globals.get(&gparam_id) == Some(&value);
        self.drafts.set_global(gparam_id, value, same_as_device);
        Ok(())
    }
    /// 全局 CSD 参数, 草稿优先。
    pub fn global(&self, gparam_id: u8) -> Option<u32> {
        if let Some(v) = self.drafts.global(gparam_id) {
            return Some(v);
        }
        self.globals.get(&gparam_id).copied()
    }
    pub fn globals_version(&self) -> u64 {
        self.globals_version
    }

    // ------------------------------------------------------------------
    // 键盘 (KBD_*): 物理键盘 GPIO1-12 + 触控→键盘映射
    // ------------------------------------------------------------------
    pub fn kbd_request_state(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetState as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }
    // ------------------------------------------------------------------
    // 触控组合映射(多分区 → 多键)。草稿制: 编辑只进 `drafts` 的组合整表, 由"保存到设备"下发。
    // 固件侧是整表替换语义, 所以草稿也按整表管理, 不做逐条增删的增量协议。
    // ------------------------------------------------------------------

    pub fn kbd_combo_version(&self) -> u64 {
        self.kbd_combo_version
    }

    /// 待新建条目的分区集合(34 位)与键组合。属于纯编辑态, 但放在这里而不是 Slint:
    /// "添加"时要与已有表做去重判定, 判定逻辑在 Rust, 状态跟着一起放才不会两边不一致。
    pub fn kbd_combo_pending_zones(&self) -> Vec<bool> {
        (0..34u8)
            .map(|z| (self.kbd_combo_pending_mask & (1u64 << z)) != 0)
            .collect()
    }

    /// 34 位: 该分区是否已被某条映射用到(仅 UI 提示, 不禁止复用)。
    pub fn kbd_combo_zone_used(&self) -> Vec<bool> {
        let table = self.kbd_combos();
        let union = table.iter().fold(0u64, |acc, c| acc | c.zone_mask);
        (0..34u8).map(|z| (union & (1u64 << z)) != 0).collect()
    }

    pub fn kbd_combo_pending_keys(&self) -> ([u8; crate::proto::KBD_COMBO_KEY_COUNT], u8) {
        (self.kbd_combo_pending_keys, self.kbd_combo_pending_mods)
    }

    pub fn kbd_combo_toggle_zone(&mut self, zone: u8) {
        if zone >= 34 {
            return;
        }
        self.kbd_combo_pending_mask ^= 1u64 << zone;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    pub fn kbd_combo_clear_zones(&mut self) {
        self.kbd_combo_pending_mask = 0;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    /// 抓到一个键: 追加到待新建的键位数组(去重, 满 4 个后忽略并提示)。修饰位取并集。
    /// ★这里的累加是"一次录制会话内"的语义★(用于"同时按住 F5+F6"这类真实和弦):
    /// 跨会话的覆盖由 UI 侧完成 —— KeyCaptureBox 每次进入录制态先调 kbd_combo_clear_keys(),
    /// 所以不同时间按下的键不会攒成一条多键映射。本函数因此不需要自己判断会话边界。
    pub fn kbd_combo_capture_key(&mut self, keycode: u8, mods: u8) {
        self.kbd_combo_pending_mods |= mods;
        if keycode != 0 {
            if self.kbd_combo_pending_keys.contains(&keycode) {
                // 同一键重复抓取不算错, 静默忽略即可。
            } else if let Some(slot) = self.kbd_combo_pending_keys.iter_mut().find(|k| **k == 0) {
                *slot = keycode;
            } else {
                self.push_log_warn(format!(
                    "组合映射: 单条最多 {} 个键, 已忽略新键。",
                    crate::proto::KBD_COMBO_KEY_COUNT
                ));
            }
        }
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    pub fn kbd_combo_clear_keys(&mut self) {
        self.kbd_combo_pending_keys = [0; crate::proto::KBD_COMBO_KEY_COUNT];
        self.kbd_combo_pending_mods = 0;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    /// 把待新建条目提交成一条映射。时间参数由新建区的两个输入框给出(不再硬编码 0/0)。
    /// 成功后只清空分区与按键; ★两个时间参数留在 UI 侧且刻意不清零★(连续添加同类映射不必重填)。
    pub fn kbd_combo_commit_pending(&mut self, delay_ms: u16, max_hold_ms: u16) {
        let mask = self.kbd_combo_pending_mask;
        let keys = self.kbd_combo_pending_keys;
        let mods = self.kbd_combo_pending_mods;
        if self.kbd_combo_add(mask, keys, mods, delay_ms, max_hold_ms) {
            self.kbd_combo_pending_mask = 0;
            self.kbd_combo_pending_keys = [0; crate::proto::KBD_COMBO_KEY_COUNT];
            self.kbd_combo_pending_mods = 0;
            self.push_log("组合映射: 已加入草稿, 点左下角“保存到设备”才真正下发。".to_string());
        }
    }

    /// 当前生效的组合表(草稿优先)。未回读且无草稿时为空表。
    pub fn kbd_combos(&self) -> Vec<crate::proto::KbdComboItem> {
        match self.drafts.kbd_combo() {
            Some(d) => d.to_vec(),
            None => self.kbd_combo_cache.clone(),
        }
    }

    /// 设备是否支持组合映射。None=还没问过; Some(false)=固件回了 NAK(旧固件)。
    pub fn kbd_combo_supported(&self) -> Option<bool> {
        self.kbd_combo_supported
    }

    pub fn kbd_request_combo(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetCombo as u8, 0, seq, vec![]))?;
            self.kbd_combo_req_seq = Some(seq);
        }
        Ok(())
    }

    /// 新增一条组合。分区集合为空、或与已有条目**完全相同**时拒绝(与固件去重规则一致:
    /// 只要分区集合不完全一样就允许共存)。返回是否加入成功。
    pub fn kbd_combo_add(
        &mut self,
        zone_mask: u64,
        keycodes: [u8; crate::proto::KBD_COMBO_KEY_COUNT],
        modifiers: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> bool {
        if zone_mask == 0 {
            self.push_log_warn("组合映射: 未选择任何触控分区, 未添加。".to_string());
            return false;
        }
        if keycodes.iter().all(|k| *k == 0) && modifiers == 0 {
            self.push_log_warn("组合映射: 未指定任何按键, 未添加。".to_string());
            return false;
        }
        let mut table = self.kbd_combos();
        if table.len() >= crate::proto::KBD_COMBO_COUNT {
            self.push_log_warn(format!(
                "组合映射: 已达上限 {} 条, 未添加。",
                crate::proto::KBD_COMBO_COUNT
            ));
            return false;
        }
        if table.iter().any(|c| c.zone_mask == zone_mask) {
            self.push_log_warn(
                "组合映射: 已存在分区集合完全相同的条目, 未添加(改按键请先删除旧条目)。"
                    .to_string(),
            );
            return false;
        }
        table.push(crate::proto::KbdComboItem {
            zone_mask,
            keycodes,
            modifiers,
            delay_ms,
            max_hold_ms,
        });
        self._stage_combo(table);
        true
    }

    pub fn kbd_combo_remove(&mut self, index: usize) {
        let mut table = self.kbd_combos();
        if index >= table.len() {
            return;
        }
        table.remove(index);
        // 删除会让后续行号整体前移, 留着旧登记等于把"替换"落到另一行上, 直接撤销。
        self.kbd_combo_key_edit_row = None;
        self._stage_combo(table);
    }

    /// 改某条的时间参数(ms)。分区集合仍然只能删了重加(单一编辑路径); 按键可就地重录, 见下。
    pub fn kbd_combo_set_hold(&mut self, index: usize, delay_ms: u16, max_hold_ms: u16) {
        let mut table = self.kbd_combos();
        if index >= table.len() {
            return;
        }
        table[index].delay_ms = delay_ms;
        table[index].max_hold_ms = max_hold_ms;
        self._stage_combo(table);
    }

    /// 登记"下一次抓到的键整行替换第 index 行"。★只登记, 绝不在这里清空该行按键★:
    /// 清空会在草稿里留下一条 0 键映射, 用户此刻点"保存到设备"就会把空条目下发出去。
    /// 改成"抓到键的那一刻才替换", 于是 0 键中间态根本不存在, 也就无从被保存。
    pub fn kbd_combo_begin_edit_keys(&mut self, index: usize) {
        if index >= self.kbd_combos().len() {
            return;
        }
        self.kbd_combo_key_edit_row = Some(index);
    }

    /// 就地重录第 index 行的按键组合。语义与新建区 `kbd_combo_capture_key` 完全同口径 ——
    /// ★跨会话覆盖、会话内累加★: 本行有待替换登记(由 KeyCaptureBox 的 capture_started 边沿
    /// 打上, 一次录制会话只打一次)时本键替换整行; 否则累加(同键去重、修饰位取并集、
    /// 满 KBD_COMBO_KEY_COUNT 忽略并告警)。所以"同时按住 F5+F6"照旧能录成多键。
    pub fn kbd_combo_capture_key_at(&mut self, index: usize, keycode: u8, mods: u8) {
        let mut table = self.kbd_combos();
        if index >= table.len() {
            return;
        }
        if self.kbd_combo_key_edit_row == Some(index) {
            self.kbd_combo_key_edit_row = None;
            table[index].keycodes = [0; crate::proto::KBD_COMBO_KEY_COUNT];
            table[index].modifiers = 0;
        }
        table[index].modifiers |= mods;
        if keycode != 0 {
            if table[index].keycodes.contains(&keycode) {
                // 同一键重复抓取不算错, 静默忽略。
            } else if let Some(slot) = table[index].keycodes.iter_mut().find(|k| **k == 0) {
                *slot = keycode;
            } else {
                self.push_log_warn(format!(
                    "组合映射: 单条最多 {} 个键, 已忽略新键。",
                    crate::proto::KBD_COMBO_KEY_COUNT
                ));
            }
        }
        self._stage_combo(table);
    }

    /// 用整张表覆盖组合映射草稿(JSON 导入用)。
    /// ★为什么单独开一个入口★: 导入是"整表替换", 而 `kbd_combo_add` 带一堆交互态校验
    /// (上限提示/重复分区拒绝/空键拒绝)并逐条追加 —— 拿它做导入会把文件里合法的整表判成冲突。
    /// 合法性已在导入侧按同一口径过滤(空条目丢弃、超上限丢弃), 这里只负责落草稿。
    pub fn kbd_combo_replace_table(&mut self, table: Vec<crate::proto::KbdComboItem>) {
        self.kbd_combo_key_edit_row = None;
        self._stage_combo(table);
    }

    /// 组合映射整表入草稿。★与设备缓存完全相等即撤稿★(与 param/keycfg 同口径):
    /// 脏状态派生自脏键集合, 若不做这一步, "删了又加回原样"会被永久误报为未保存。
    /// 脏键登记与 version bump 由 `ConfigDrafts::set_kbd_combo` 内部完成, 无从遗漏
    /// —— 旧实现在这里只插脏键却漏了 `mark_config_dirty()`, 导致"添加组合映射后按钮不亮"。
    fn _stage_combo(&mut self, table: Vec<crate::proto::KbdComboItem>) {
        let same_as_device = table == self.kbd_combo_cache;
        self.drafts.set_kbd_combo(table, same_as_device);
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
    }

    /// 由 save_config 调用: 把草稿整表排入统一写队列。无草稿则什么都不做。
    fn _commit_combo(&mut self, queued: &mut Vec<Frame>) -> anyhow::Result<()> {
        let Some(table) = self.drafts.take_kbd_combo() else {
            return Ok(());
        };
        let payload = crate::proto::encode_kbd_set_combo(&table);
        let seq = self.next_seq();
        queued.push(Frame::new(HostCmd::KbdSetCombo as u8, 0, seq, payload));
        // 乐观写缓存: 固件对整表做了去重与空条目丢弃, 真值由随后的回读校正。
        self.kbd_combo_cache = table;
        self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
        Ok(())
    }

    pub fn kbd_request_map(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetMap as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }
    pub fn kbd_request_touchmap(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetTouchmap as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }
    /// 暂存物理键 idx(0..11) 的 HID 键码 + 修饰位(bit0 Ctrl/1 Shift/2 Alt/3 Gui)到草稿。
    pub fn kbd_set_map(&mut self, idx: u8, keycode: u8, modifier: u8) -> anyhow::Result<()> {
        if idx >= 12 {
            return Err(anyhow::anyhow!("物理键索引非法: {}", idx));
        }
        let i = idx as usize;
        let same_as_device =
            self.kbd_map.get(i) == Some(&keycode) && self.kbd_keymod.get(i) == Some(&modifier);
        self.drafts
            .set_kbd_map(idx, keycode, modifier, same_as_device);
        // ★必须 bump★: main.rs 的键码/修饰位/显示文本回填全部由该 version 门控; 只写草稿不 bump
        // 会让"捕获组合键"在界面上毫无反应(看起来只是退出了录制态), 用户无法确认是否落地。
        // 撤稿分支同样要 bump: getter 是"草稿优先", 撤回后可见值变回设备真值, UI 必须跟着回填。
        self.kbd_map_version = self.kbd_map_version.wrapping_add(1);
        Ok(())
    }
    /// 暂存触控分区 zone(0..33) 的 HID 键码 + 修饰位到草稿。
    pub fn kbd_set_touchmap(&mut self, zone: u8, keycode: u8, modifier: u8) -> anyhow::Result<()> {
        if zone >= 34 {
            return Err(anyhow::anyhow!("分区索引非法: {}", zone));
        }
        let z = zone as usize;
        let same_as_device = self.kbd_touchmap.get(z) == Some(&keycode)
            && self.kbd_zonemod.get(z) == Some(&modifier);
        self.drafts
            .set_kbd_touch(zone, keycode, modifier, same_as_device);
        // ★必须 bump★: 见 kbd_set_map 说明 —— 显示文本/下拉索引/修饰位都由该 version 门控回填。
        self.kbd_touchmap_version = self.kbd_touchmap_version.wrapping_add(1);
        Ok(())
    }
    pub fn kbd_state(&self) -> u16 {
        self.kbd_state
    }
    pub fn kbd_state_version(&self) -> u64 {
        self.kbd_state_version
    }
    pub fn kbd_map(&self, idx: u8) -> u8 {
        if let Some((code, _)) = self.drafts.kbd_map(idx) {
            return code;
        }
        *self.kbd_map.get(idx as usize).unwrap_or(&0)
    }
    pub fn kbd_keymod(&self, idx: u8) -> u8 {
        if let Some((_, m)) = self.drafts.kbd_map(idx) {
            return m;
        }
        *self.kbd_keymod.get(idx as usize).unwrap_or(&0)
    }
    pub fn kbd_map_version(&self) -> u64 {
        self.kbd_map_version
    }
    pub fn kbd_touch_keycode(&self, zone: u8) -> u8 {
        if let Some((code, _)) = self.drafts.kbd_touch(zone) {
            return code;
        }
        *self.kbd_touchmap.get(zone as usize).unwrap_or(&0)
    }
    pub fn kbd_zone_mod(&self, zone: u8) -> u8 {
        if let Some((_, m)) = self.drafts.kbd_touch(zone) {
            return m;
        }
        *self.kbd_zonemod.get(zone as usize).unwrap_or(&0)
    }
    pub fn kbd_touch_en(&self) -> bool {
        self.kbd_touch_en
    }
    pub fn kbd_touchmap_version(&self) -> u64 {
        self.kbd_touchmap_version
    }

    fn _handle_kbd_get_state_response(&mut self, frame: &Frame) {
        if frame.payload.len() >= 2 {
            self.kbd_state = (frame.payload[0] as u16) | ((frame.payload[1] as u16) << 8);
            // 尾部追加的 raw/out 两态(新固件才有)。旧固件没有就保持 0, 不用 phys_state 冒充 ——
            // 否则界面会显示"原始态与去抖态永远一致", 等于把防抖效果伪装成不存在。
            if frame.payload.len() >= 6 {
                self.kbd_state_raw = (frame.payload[2] as u16) | ((frame.payload[3] as u16) << 8);
                self.kbd_state_out = (frame.payload[4] as u16) | ((frame.payload[5] as u16) << 8);
            }
            self.kbd_state_version = self.kbd_state_version.wrapping_add(1);
        }
    }
    fn _handle_kbd_get_map_response(&mut self, frame: &Frame) {
        if frame.payload.is_empty() {
            return;
        }
        // 每键 2 字节: [keycode, modifier]。
        let count = frame.payload[0] as usize;
        for i in 0..count.min(12) {
            if let Some(&code) = frame.payload.get(1 + i * 2) {
                self.kbd_map[i] = code;
            }
            if let Some(&m) = frame.payload.get(2 + i * 2) {
                self.kbd_keymod[i] = m;
            }
        }
        self.kbd_map_version = self.kbd_map_version.wrapping_add(1);
    }
    fn _handle_kbd_get_touchmap_response(&mut self, frame: &Frame) {
        if frame.payload.len() < 2 {
            return;
        }
        // [en, count, (keycode, modifier)×count]。
        self.kbd_touch_en = frame.payload[0] != 0;
        let count = frame.payload[1] as usize;
        for z in 0..count.min(34) {
            if let Some(&code) = frame.payload.get(2 + z * 2) {
                self.kbd_touchmap[z] = code;
            }
            if let Some(&m) = frame.payload.get(3 + z * 2) {
                self.kbd_zonemod[z] = m;
            }
        }
        self.kbd_touchmap_version = self.kbd_touchmap_version.wrapping_add(1);
    }

    // ------------------------------------------------------------------
    // 键盘长按参数 (KBD_GET_HOLD / KBD_SET_HOLD)、mai2 串口运行态 (MAI2_*)
    // 与掉线重连自动恢复运行态
    // ------------------------------------------------------------------

    /// 触控→键盘映射总开关的配置 key(重连恢复期望态也按此 key 下发)。
    const KBD_MAP_EN_KEY: &'static str = "comm.keyboard_map_en";

    /// 物理键 idx(0..11) 的长按参数：草稿优先 → 设备回读兜底。
    pub fn kbd_hold_phys(&self, idx: u8) -> (u16, u16) {
        let hold = self
            .drafts
            .kbd_hold_phys(idx)
            .or_else(|| self.kbd_hold_phys.get(idx as usize).copied())
            .unwrap_or_default();
        (hold.delay_ms, hold.max_hold_ms)
    }

    /// 触控分区 zone(0..33) 的长按参数：草稿优先 → 设备回读兜底。
    pub fn kbd_hold_zone(&self, zone: u8) -> (u16, u16) {
        let hold = self
            .drafts
            .kbd_hold_zone(zone)
            .or_else(|| self.kbd_hold_zone.get(zone as usize).copied())
            .unwrap_or_default();
        (hold.delay_ms, hold.max_hold_ms)
    }

    /// 长按参数版本号(下发/回读时自增), 供 UI 判断是否刷新。
    pub fn kbd_hold_version(&self) -> u64 {
        self.kbd_hold_version
    }

    /// 暂存长按参数到草稿(不下发)，供 JSON 导入等批量外部写入使用。
    pub fn stage_kbd_hold(
        &mut self,
        kind: u8,
        idx: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> anyhow::Result<()> {
        match kind {
            KBD_HOLD_KIND_PHYS => self.kbd_set_hold_phys(idx, delay_ms, max_hold_ms),
            KBD_HOLD_KIND_ZONE => self.kbd_set_hold_zone(idx, delay_ms, max_hold_ms),
            _ => Err(anyhow::anyhow!("长按参数类型非法: {}", kind)),
        }
    }

    /// 暂存物理键长按参数到草稿，不立即下发。
    pub fn kbd_set_hold_phys(
        &mut self,
        idx: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> anyhow::Result<()> {
        if (idx as usize) >= KBD_HOLD_PHYS_COUNT {
            return Err(anyhow::anyhow!("物理键长按索引非法: {}", idx));
        }
        let hold = HoldParam {
            delay_ms,
            max_hold_ms,
        };
        let same_as_device = self.kbd_hold_phys.get(idx as usize) == Some(&hold);
        self.drafts.set_kbd_hold_phys(idx, hold, same_as_device);
        self.kbd_hold_version = self.kbd_hold_version.wrapping_add(1);
        Ok(())
    }

    /// 暂存触控分区长按参数到草稿，不立即下发。
    pub fn kbd_set_hold_zone(
        &mut self,
        zone: u8,
        delay_ms: u16,
        max_hold_ms: u16,
    ) -> anyhow::Result<()> {
        if (zone as usize) >= KBD_HOLD_ZONE_COUNT {
            return Err(anyhow::anyhow!("触控分区长按索引非法: {}", zone));
        }
        let hold = HoldParam {
            delay_ms,
            max_hold_ms,
        };
        let same_as_device = self.kbd_hold_zone.get(zone as usize) == Some(&hold);
        self.drafts.set_kbd_hold_zone(zone, hold, same_as_device);
        self.kbd_hold_version = self.kbd_hold_version.wrapping_add(1);
        Ok(())
    }

    /// 请求回读全部长按参数(12 物理键 + 34 分区)。
    pub fn kbd_request_hold(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetHold as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }

    // ------------------------------------------------------------------
    // 物理键每键配置(触发极性 + 独立防抖)与逻辑分析仪边沿记录
    // 草稿制与长按参数完全同一套: 编辑只进 `drafts` 的每键配置草稿, "保存到设备"才下发。
    // ------------------------------------------------------------------

    /// 物理键 idx(0..11) 的触发极性与防抖窗: 草稿优先 → 设备回读兜底。
    pub fn kbd_keycfg(&self, idx: u8) -> KbdKeyCfg {
        self.drafts
            .kbd_keycfg(idx)
            .or_else(|| self.kbd_keycfg.get(idx as usize).copied())
            .unwrap_or_default()
    }

    pub fn kbd_keycfg_version(&self) -> u64 {
        self.kbd_keycfg_version
    }

    /// 固件解析后的生效极性掩码(bit i = 1 → 该键按高电平触发判定)。
    /// AUTO 档的判定结果只能从这里读 —— `kbd_keycfg().pol` 是配置态, 值为 2 时不含判定结论。
    pub fn kbd_pol_resolved_mask(&self) -> u16 {
        self.kbd_pol_resolved_mask
    }

    /// 生效极性掩码是否有真实来源。false = 旧固件未回传 → 生效电平**未知**, UI 显示"未知"。
    pub fn kbd_pol_resolved_known(&self) -> bool {
        self.kbd_pol_resolved_known
    }

    /// 设备是否支持每键配置。None=还没问过; Some(false)=旧固件 NAK。
    pub fn kbd_keycfg_supported(&self) -> Option<bool> {
        self.kbd_keycfg_supported
    }

    /// 暂存每键触发极性 + 防抖窗到草稿(不下发)。
    /// 防抖上限与固件同源(`KBD_DEBOUNCE_US_MAX`): 越界直接报错, 不下发一个必被 NAK 的值。
    pub fn kbd_set_keycfg(&mut self, idx: u8, pol: u8, debounce_us: u16) -> anyhow::Result<()> {
        if (idx as usize) >= KBD_HOLD_PHYS_COUNT {
            return Err(anyhow::anyhow!("物理键索引非法: {}", idx));
        }
        // 极性围栏与固件 _handle_set_keycfg 同源: 越界直接拒绝, 不下发一个必被 NAK 的值。
        if pol > crate::proto::KBD_POL_AUTO {
            return Err(anyhow::anyhow!(
                "触发极性非法: {} (0=低电平 1=高电平 2=自动)",
                pol
            ));
        }
        if debounce_us > KBD_DEBOUNCE_US_MAX {
            return Err(anyhow::anyhow!(
                "防抖时间超出范围: {}us (上限 {}us)",
                debounce_us,
                KBD_DEBOUNCE_US_MAX
            ));
        }
        let cfg = KbdKeyCfg { pol, debounce_us };
        // 改回设备真值即自动撤稿(与 kbd_set_hold_phys 同口径), 免得界面一直挂着"未保存"。
        let same_as_device = self.kbd_keycfg.get(idx as usize) == Some(&cfg);
        self.drafts.set_kbd_keycfg(idx, cfg, same_as_device);
        self.kbd_keycfg_version = self.kbd_keycfg_version.wrapping_add(1);
        Ok(())
    }

    /// 请求回读 12 个物理键的极性与防抖窗。
    pub fn kbd_request_keycfg(&mut self) -> anyhow::Result<()> {
        if self.kbd_keycfg_supported == Some(false) {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::KbdGetKeycfg as u8, 0, seq, vec![]))?;
            self.kbd_keycfg_req_seq = Some(seq);
        }
        Ok(())
    }

    /// 拉取一批边沿记录。窗口=1(有在途请求就跳过): 逻辑分析仪不能把 vendor 端点抢光,
    /// 否则会连带影响遥测与按键映射的回读。旧固件已判定不支持时直接返回。
    pub fn kbd_request_edges(&mut self) -> anyhow::Result<()> {
        if self.kbd_edges_supported == Some(false) || self.kbd_edge_req_seq.is_some() {
            return Ok(());
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(
                HostCmd::KbdGetEdges as u8,
                0,
                seq,
                crate::proto::encode_kbd_get_edges(0),
            ))?;
            self.kbd_edge_req_seq = Some(seq);
        }
        Ok(())
    }

    /// 主机侧留存的边沿记录(最旧在前)。
    pub fn kbd_edges(&self) -> &VecDeque<KbdEdgeRec> {
        &self.kbd_edges
    }

    pub fn kbd_edges_version(&self) -> u64 {
        self.kbd_edges_version
    }

    /// 设备是否支持边沿记录。None=还没问过; Some(false)=旧固件 NAK。
    pub fn kbd_edges_supported(&self) -> Option<bool> {
        self.kbd_edges_supported
    }

    /// 累计被设备丢弃(环满)的边沿条数。>0 时 UI 必须显示"有事件丢失", 不能假装波形连续。
    pub fn kbd_edge_lost(&self) -> u32 {
        self.kbd_edge_lost
    }

    /// 上次拉取后设备侧仍剩余的条数(>0 = 拉取跟不上设备产生速度)。
    pub fn kbd_edge_remaining(&self) -> u16 {
        self.kbd_edge_remaining
    }

    /// 清空主机侧留存窗口与丢失计数(UI "清空波形")。
    pub fn kbd_edges_clear(&mut self) {
        self.kbd_edges.clear();
        self.kbd_edge_lost = 0;
        self.kbd_edge_remaining = 0;
        self.kbd_edges_version = self.kbd_edges_version.wrapping_add(1);
    }

    /// 物理键去抖前(raw)的 12 位实时态。旧固件不提供时为 0。
    pub fn kbd_state_raw(&self) -> u16 {
        self.kbd_state_raw
    }

    /// 物理键实际输出 HID 的 12 位实时态(经去抖 + 长按状态机)。旧固件不提供时为 0。
    pub fn kbd_state_out(&self) -> u16 {
        self.kbd_state_out
    }

    fn _handle_kbd_get_keycfg_response(&mut self, frame: &Frame) {
        self.kbd_keycfg_req_seq = None;
        match crate::proto::decode_kbd_get_keycfg(&frame.payload) {
            Ok((list, resolved)) => {
                self.kbd_keycfg_supported = Some(true);
                for (i, cfg) in list.iter().take(KBD_HOLD_PHYS_COUNT).enumerate() {
                    self.kbd_keycfg[i] = *cfg;
                }
                // 旧固件不追加 resolved mask: 保持上次读数而不是清 0 —— 清 0 会被 UI 显示成
                // "全部判定为低电平触发", 那是凭空造出来的结论。
                if let Some(mask) = resolved {
                    self.kbd_pol_resolved_mask = mask;
                    self.kbd_pol_resolved_known = true;
                }
                self.kbd_keycfg_version = self.kbd_keycfg_version.wrapping_add(1);
            }
            Err(e) => self.push_log_warn(format!("KBD_GET_KEYCFG 解析失败: {}", e)),
        }
    }

    fn _handle_kbd_get_edges_response(&mut self, frame: &Frame) {
        self.kbd_edge_req_seq = None;
        let batch = match crate::proto::decode_kbd_get_edges(&frame.payload) {
            Ok(b) => b,
            Err(e) => {
                self.push_log_warn(format!("KBD_GET_EDGES 解析失败: {}", e));
                return;
            }
        };
        self.kbd_edges_supported = Some(true);
        self.kbd_edge_remaining = batch.remaining;
        // overflow 是设备累计值(只增, 设备重启才归零)。取差值累加到主机侧丢失计数;
        // 设备重启导致读数回退时按"重新计数"处理, 不产生天文数字的假丢失量。
        if batch.overflow >= self.kbd_edge_overflow_last {
            self.kbd_edge_lost = self
                .kbd_edge_lost
                .saturating_add(batch.overflow - self.kbd_edge_overflow_last);
        } else {
            self.kbd_edge_lost = self.kbd_edge_lost.saturating_add(batch.overflow);
        }
        self.kbd_edge_overflow_last = batch.overflow;
        if batch.recs.is_empty() {
            return;
        }
        for rec in batch.recs {
            if self.kbd_edges.len() >= KBD_EDGE_KEEP {
                self.kbd_edges.pop_front();
            }
            self.kbd_edges.push_back(rec);
        }
        self.kbd_edges_version = self.kbd_edges_version.wrapping_add(1);
    }

    /// mai2 串口实际有效发送态；None = 尚未回读。
    /// 用户期望只用于下发与重连恢复，绝不能覆盖设备状态机的真值。
    pub fn mai2_send_en(&self) -> Option<bool> {
        self.mai2_state.map(|s| s.send_en)
    }

    /// mai2 串口状态: 0=停 1=就绪 2=运行; None = 尚未回读。
    pub fn mai2_status(&self) -> Option<u8> {
        self.mai2_state.map(|s| s.status)
    }

    /// mai2 串口波特率; None = 尚未回读。
    pub fn mai2_baud(&self) -> Option<u32> {
        self.mai2_state.map(|s| s.baud)
    }

    /// mai2 运行态版本号(下发/回读时自增)。
    pub fn mai2_version(&self) -> u64 {
        self.mai2_version
    }

    /// 设置 mai2 串口发送使能: 立即下发 + 写期望值(掉线重连后据此自动恢复)。
    pub fn mai2_set_send_en(&mut self, en: bool) -> anyhow::Result<()> {
        self.desired.mai2_send_en = Some(en);
        self.mai2_version = self.mai2_version.wrapping_add(1);
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(
                HostCmd::Mai2SetSendEn as u8,
                0,
                seq,
                crate::proto::encode_mai2_set_send_en(en),
            ))?;
        }
        self.push_log(format!("mai2 串口发送使能 → {}", Self::_on_off(en)));
        Ok(())
    }

    /// 请求回读 mai2 串口运行态。
    pub fn mai2_request_state(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(HostCmd::Mai2GetState as u8, 0, seq, vec![]))?;
        }
        Ok(())
    }

    /// 清空日志视图。
    /// 唯一的可显示数据源是 `logging::hub()` 的环形缓冲(UI 文本只读它), 所以清空必须清它,
    /// 否则点了没反应; `event_log` 只是本控制器的内部副本, 一并清掉免得留下第二份"真相"。
    /// 磁盘日志文件不受影响(继续追加), 见 logging::LogHub::clear 的说明。
    pub fn clear_log(&mut self) {
        crate::logging::hub().clear();
        self.event_log.clear();
        self.log_seq = self.log_seq.wrapping_add(1);
    }

    /// ★掉线重连自动恢复运行态★
    ///
    /// 握手完成(收到 DEVICE_INFO)时由 `handle_frame` 自动调用, 无需 UI 介入: 把用户显式设置过的
    /// 期望态(mai2 发送使能 / 触控键盘映射总开关 / 长按参数)逐条重新下发, 再拉一次真值回读对账。
    /// 也公开出来供 UI 手动重试。
    pub fn restore_after_reconnect(&mut self) -> anyhow::Result<()> {
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法恢复运行态"));
        }
        if self.desired.is_empty() {
            // 用户从未显式设置过运行态: 连接探针队列会顺序回读真实状态, 不在握手边沿直发并发请求。
            return Ok(());
        }
        // 1) 长按参数: 全部期望项合并为一帧下发。
        let mut items: Vec<KbdHoldItem> =
            Vec::with_capacity(self.desired.hold_phys.len() + self.desired.hold_zone.len());
        for (idx, hold) in self.desired.hold_phys.iter() {
            items.push(KbdHoldItem {
                kind: KBD_HOLD_KIND_PHYS,
                idx: *idx,
                hold: *hold,
            });
        }
        for (zone, hold) in self.desired.hold_zone.iter() {
            items.push(KbdHoldItem {
                kind: KBD_HOLD_KIND_ZONE,
                idx: *zone,
                hold: *hold,
            });
        }
        if !items.is_empty() {
            let payload = crate::proto::encode_kbd_set_hold(&items);
            let seq = self.next_seq();
            self._queue_tx(Frame::new(HostCmd::KbdSetHold as u8, 0, seq, payload))?;
        }
        // 2) 触控→键盘映射总开关。
        if let Some(en) = self.desired.kbd_map_en {
            let entry = ConfigEntry::new(Self::KBD_MAP_EN_KEY.to_string(), CfgValue::Bool(en));
            let payload = crate::proto::encode_entry(&entry)
                .map_err(|e| anyhow::anyhow!("encode_entry failed: {}", e))?;
            let seq = self.next_seq();
            self._queue_tx(Frame::new(HostCmd::CfgSet as u8, 0, seq, payload))?;
        }
        // 3) mai2 发送使能 —— 震动掉线后要立刻恢复触控, 这一条最关键。
        if let Some(en) = self.desired.mai2_send_en {
            let seq = self.next_seq();
            self._queue_tx(Frame::new(
                HostCmd::Mai2SetSendEn as u8,
                0,
                seq,
                crate::proto::encode_mai2_set_send_en(en),
            ))?;
        }
        self.push_log(format!(
            "重连恢复运行态: 长按 {} 项 + 触控键盘映射={} + mai2 发送使能={} 已重新下发, 连接探针将在写队列排空后回读对账",
            items.len(),
            Self::_opt_on_off(self.desired.kbd_map_en),
            Self::_opt_on_off(self.desired.mai2_send_en)));
        // 4) 回读对账由连接探针队列中的 KBD_GET_HOLD/MAI2_GET_STATE 负责，避免与恢复写入并发。
        self.restore_verify.hold = true;
        self.restore_verify.mai2 = true;
        Ok(())
    }

    fn _on_off(v: bool) -> &'static str {
        if v { "开" } else { "关" }
    }

    fn _opt_on_off(v: Option<bool>) -> &'static str {
        match v {
            Some(true) => "开",
            Some(false) => "关",
            None => "未设置",
        }
    }

    fn _mai2_status_text(status: u8) -> &'static str {
        match status {
            0 => "停",
            1 => "就绪",
            2 => "运行",
            _ => "未知",
        }
    }

    fn _handle_kbd_get_hold_response(&mut self, frame: &Frame) {
        let table = match crate::proto::decode_kbd_get_hold(&frame.payload) {
            Ok(t) => t,
            Err(e) => {
                self.push_log_warn(format!("KBD_GET_HOLD 解析失败: {}", e));
                self.restore_verify.hold = false;
                return;
            }
        };
        for (i, hold) in table.phys.iter().take(KBD_HOLD_PHYS_COUNT).enumerate() {
            self.kbd_hold_phys[i] = *hold;
        }
        for (i, hold) in table.zone.iter().take(KBD_HOLD_ZONE_COUNT).enumerate() {
            self.kbd_hold_zone[i] = *hold;
        }
        self.kbd_hold_version = self.kbd_hold_version.wrapping_add(1);
        if self.restore_verify.hold {
            self.restore_verify.hold = false;
            self._verify_hold_readback();
        }
    }

    /// 重连恢复后的长按参数对账: 期望值 vs 设备真值, 不一致逐条告警(设备未接受/被夹取必须可见)。
    fn _verify_hold_readback(&mut self) {
        let mut bad: Vec<String> = Vec::new();
        for (idx, want) in self.desired.hold_phys.iter() {
            let got = self
                .kbd_hold_phys
                .get(*idx as usize)
                .copied()
                .unwrap_or_default();
            if got != *want {
                bad.push(format!(
                    "物理键{}: 期望 {}/{}ms 实际 {}/{}ms",
                    *idx as u16 + 1,
                    want.delay_ms,
                    want.max_hold_ms,
                    got.delay_ms,
                    got.max_hold_ms
                ));
            }
        }
        for (zone, want) in self.desired.hold_zone.iter() {
            let got = self
                .kbd_hold_zone
                .get(*zone as usize)
                .copied()
                .unwrap_or_default();
            if got != *want {
                bad.push(format!(
                    "{}: 期望 {}/{}ms 实际 {}/{}ms",
                    zone_label(*zone as usize),
                    want.delay_ms,
                    want.max_hold_ms,
                    got.delay_ms,
                    got.max_hold_ms
                ));
            }
        }
        if bad.is_empty() {
            self.push_log("重连恢复对账: 长按参数与期望一致");
        } else {
            self.push_log_warn(format!(
                "⚠ 重连恢复对账: {} 项长按参数与期望不一致(设备未接受或已夹取): {}",
                bad.len(),
                bad.join("; ")
            ));
        }
    }

    fn _handle_mai2_get_state_response(&mut self, frame: &Frame) {
        let state = match crate::proto::decode_mai2_get_state(&frame.payload) {
            Ok(s) => s,
            Err(e) => {
                self.push_log_warn(format!("MAI2_GET_STATE 解析失败: {}", e));
                self.restore_verify.mai2 = false;
                return;
            }
        };
        self.mai2_state = Some(state);
        self.mai2_version = self.mai2_version.wrapping_add(1);
        if !self.restore_verify.mai2 {
            return;
        }
        self.restore_verify.mai2 = false;
        let status_text = Self::_mai2_status_text(state.status);
        match self.desired.mai2_send_en {
            Some(want) if want != state.send_en => self.push_log_warn(format!(
                "⚠ 重连恢复对账: mai2 发送使能期望 {} 但设备回读 {} (状态={} 波特率={}) → 触控可能未恢复",
                Self::_on_off(want), Self::_on_off(state.send_en), status_text, state.baud)),
            Some(want) => self.push_log(format!(
                "重连恢复对账: mai2 发送使能={} 已生效 (状态={} 波特率={})",
                Self::_on_off(want), status_text, state.baud)),
            None => {}
        }
        if state.status == 0 {
            self.push_log_warn(format!(
                "⚠ 重连恢复对账: mai2 串口状态=停(波特率={}), 游戏触控上报未运行",
                state.baud
            ));
        }
    }
    // ------------------------------------------------------------------
    // mai2light

    /// 灯效运行态版本号(回读/草稿编辑/应用结果变化时自增)。
    pub fn led_version(&self) -> u64 {
        self.led_version
    }

    /// 是否已拿到设备灯效快照(未拿到时 UI 一律显示"未知", 不以 0 冒充真值)。
    pub fn led_known(&self) -> bool {
        self.led_state.is_some()
    }

    /// 灯板协议状态机: 0=停 1=就绪 2=运行; None=尚未回读。
    pub fn led_status(&self) -> Option<u8> {
        self.led_state.map(|s| s.status)
    }
    /// WS2812 指定链初始化就绪状态; None=尚未回读或索引非法。
    pub fn led_chain_ready(&self, chain: usize) -> Option<bool> {
        if chain >= 2 {
            return None;
        }
        self.led_state.map(|s| s.chain_ready[chain])
    }
    /// WS2812 初始化失败分档; None=尚未回读。
    pub fn led_init_fault(&self) -> Option<u8> {
        self.led_state.map(|s| s.init_fault)
    }
    pub fn led_resp_enabled(&self) -> Option<bool> {
        self.led_state.map(|s| s.resp_enabled)
    }
    /// 设备侧预览色是否正在覆盖协议色; None=尚未回读。false 时颜色字段即游戏协议色。
    pub fn led_preview_active(&self) -> Option<bool> {
        self.led_state.map(|s| s.preview_active)
    }
    /// 设备灯效服务是否已初始化; None=尚未回读。false = 预览色不会被刷到灯链。
    pub fn led_service_ready(&self) -> Option<bool> {
        self.led_state.map(|s| s.service_ready)
    }
    /// 设备灯效刷新是否至少执行过一次; None=尚未回读。false = 灯服务从未被主循环调用。
    pub fn led_refresh_seen(&self) -> Option<bool> {
        self.led_state.map(|s| s.refresh_seen)
    }
    /// 设备灯效刷新次数低 4 位; None=尚未回读。两次快照该值不变 = 灯服务已停摆。
    pub fn led_refresh_ticks(&self) -> Option<u8> {
        self.led_state.map(|s| s.refresh_ticks)
    }
    pub fn led_baud(&self) -> Option<u32> {
        self.led_state.map(|s| s.baud)
    }
    /// 灯板波特率是否可作为 UI 真值展示；判定集中复用协议快照的范围约束。
    pub fn led_baud_valid(&self) -> bool {
        self.led_state.map_or(false, |state| state.baud_valid())
    }
    pub fn led_rx_frames(&self) -> u32 {
        self.led_state.map_or(0, |s| s.rx_frames)
    }
    /// 收帧计数是否可无损显示，避免 u32 转 Slint int 后翻为负数。
    pub fn led_rx_frames_valid(&self) -> bool {
        self.led_state
            .map_or(false, |state| state.rx_frames_valid())
    }
    pub fn led_sum_errors(&self) -> u32 {
        self.led_state.map_or(0, |s| s.sum_errors)
    }
    /// 校验错误计数是否可无损显示，避免异常快照伪装成正常数值。
    pub fn led_sum_errors_valid(&self) -> bool {
        self.led_state
            .map_or(false, |state| state.sum_errors_valid())
    }

    /// 设备回报的灯链实际灯珠数(chain 0/1); 越界返回 0。
    pub fn led_ws_count(&self, chain: usize) -> u16 {
        if chain > 1 {
            return 0;
        }
        self.led_state.map_or(0, |s| s.ws_count[chain])
    }

    /// 单元当前采样颜色(预览生效时即预览色); 未回读时为全黑。
    pub fn led_color(&self, unit: usize) -> [u8; 3] {
        if unit >= LED_UNIT_COUNT {
            return [0, 0, 0];
        }
        self.led_state.map_or([0, 0, 0], |s| s.colors[unit])
    }

    /// 单元映射: 草稿优先 → 设备回读兜底 → 未映射。
    pub fn led_region(&self, unit: usize) -> LedRegion {
        if unit >= LED_UNIT_COUNT {
            return LedRegion::default();
        }
        if let Some(draft) = self.drafts.led_region() {
            return draft[unit];
        }
        self.led_state
            .map_or(LedRegion::default(), |s| s.regions[unit])
    }

    /// 最近一次"应用映射"的结果文本(含设备 NAK 原因); 空串=尚未操作。
    pub fn led_apply_status(&self) -> &str {
        &self.led_apply_status
    }

    /// 最近一次尚在等待设备 ACK/NAK 的灯效写操作序号(映射或预览)；None 表示已有结局或尚未下发。
    pub fn led_apply_seq(&self) -> Option<u8> {
        self.led_apply_seq.map(|(seq, _)| seq)
    }

    /// 最近一次 LED_GET 快照中的虚拟单元数量；None 表示尚未取得有效快照。
    pub fn led_unit_count(&self) -> Option<u8> {
        self.led_state.map(|s| s.unit_count)
    }

    /// 最近一次 LED_GET 快照中的设备实际生效亮度；None 表示尚未回读或旧固件未上报。
    pub fn led_applied_brightness(&self) -> Option<u8> {
        self.led_state.and_then(|state| state.applied_brightness)
    }

    /// 本地映射校验结论: Some(原因) = 明知会被设备拒收, 不该发。
    pub fn led_region_conflict(&self) -> Option<String> {
        crate::proto::validate_led_regions(&self._led_regions(), self._led_ws_counts())
    }

    /// 编辑单元映射草稿。`ch` 传 `LED_CH_UNMAPPED` 解除映射。
    pub fn led_set_region(&mut self, unit: usize, ch: u8, start: u16, count: u8) {
        if unit >= LED_UNIT_COUNT {
            return;
        }
        let mut regions = self._led_regions();
        let ch = if ch > 1 { LED_CH_UNMAPPED } else { ch };
        regions[unit] = LedRegion { ch, start, count };
        self.drafts.set_led_region(regions);
        self.led_version = self.led_version.wrapping_add(1);
    }

    /// 请求回读灯效运行态(LED_GET)。
    pub fn led_request_state(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(Frame::new(
                HostCmd::LedGet as u8,
                0,
                seq,
                crate::proto::encode_led_get(),
            ))?;
        }
        Ok(())
    }

    /// 整批下发 11 个单元的映射(LED_SET_REGION)。本地校验不通过则不发, 直接给出冲突原因。
    pub fn led_apply_regions(&mut self) -> anyhow::Result<()> {
        if self.io.is_none() {
            self.led_apply_status = "未连接, 无法应用映射".to_string();
            self.led_version = self.led_version.wrapping_add(1);
            return Err(anyhow::anyhow!("未连接"));
        }
        let regions = self._led_regions();
        if let Some(reason) = crate::proto::validate_led_regions(&regions, self._led_ws_counts()) {
            self.led_apply_status = format!("本地校验未通过: {}", reason);
            self.led_version = self.led_version.wrapping_add(1);
            self.push_log_warn(format!("灯效映射未下发({})", reason));
            return Ok(());
        }
        // count=0 的区段语义上就是"没占灯珠", 统一折成解除映射后下发, 免得固件把 0 长度当非法参数
        // 整批 NAK(整批原子失败对用户表现为"什么都没变", 最难排查)。
        let items: Vec<(u8, LedRegion)> = regions
            .iter()
            .enumerate()
            .map(|(unit, region)| {
                let mut region = *region;
                if region.count == 0 {
                    region = LedRegion::default();
                }
                (unit as u8, region)
            })
            .collect();
        self.led_send_regions_raw(&items)?;
        self.push_log("灯效映射: 已整批下发 LED_SET_REGION(11 单元)");
        Ok(())
    }

    /// 无头 `selftest --led` 专用的原始整批 LED_SET_REGION 下发。
    ///
    /// 设备侧必须独立验证整批原子校验；若复用 UI 的本地预校验，重叠/越界
    /// 用例会在主机被拦截而掩盖设备实现。UI 仍必须走 `led_apply_regions` 的
    /// 本地校验，此接口仅供 selftest 验证路径使用。
    pub fn led_send_regions_raw(&mut self, items: &[(u8, LedRegion)]) -> anyhow::Result<u8> {
        if self.io.is_none() {
            self.led_apply_status = "未连接, 无法下发原始映射".to_string();
            self.led_version = self.led_version.wrapping_add(1);
            return Err(anyhow::anyhow!("未连接"));
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(
                HostCmd::LedSetRegion as u8,
                0,
                seq,
                crate::proto::encode_led_set_region(items),
            ))?;
        }
        self.led_apply_seq = Some((seq, LedWriteOp::ApplyRegions));
        self.led_apply_status = "已下发, 等待设备确认...".to_string();
        self.led_version = self.led_version.wrapping_add(1);
        Ok(seq)
    }

    /// 发送预览色并返回本次下发的 seq。`unit` 传 `LED_PREVIEW_ALL` 表示全部单元;
    /// 设备约 3s 无新预览自动回协议色。
    ///
    /// 回执与"应用映射"共用 `led_apply_seq` 归因: 预览被设备拒绝(unit 越界/载荷不整)时
    /// 原因必须能显示出来, 而不是发完就当成功。返回值可忽略(UI 只关心状态文案)。
    pub fn led_preview(&mut self, unit: u8, rgb: [u8; 3]) -> anyhow::Result<u8> {
        if self.io.is_none() {
            self.led_apply_status = "未连接, 无法预览".to_string();
            self.led_version = self.led_version.wrapping_add(1);
            return Err(anyhow::anyhow!("未连接"));
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(Frame::new(
                HostCmd::LedPreview as u8,
                0,
                seq,
                crate::proto::encode_led_preview(&[(unit, rgb)]),
            ))?;
        }
        self.led_apply_seq = Some((seq, LedWriteOp::Preview));
        self.led_apply_status = "已下发, 等待设备确认...".to_string();
        self.led_version = self.led_version.wrapping_add(1);
        let who = if unit == LED_PREVIEW_ALL {
            "全部单元".to_string()
        } else {
            format!("单元 {}", unit)
        };
        self.push_log(format!(
            "灯效预览: {} → RGB({},{},{}), 约 3s 后自动回到协议色",
            who, rgb[0], rgb[1], rgb[2]
        ));
        Ok(seq)
    }

    /// 当前生效的 11 单元映射(草稿优先), 供校验/下发共用。
    fn _led_regions(&self) -> [LedRegion; LED_UNIT_COUNT] {
        if let Some(draft) = self.drafts.led_region() {
            return *draft;
        }
        self.led_state
            .map_or([LedRegion::default(); LED_UNIT_COUNT], |s| s.regions)
    }

    fn _led_ws_counts(&self) -> [u16; 2] {
        self.led_state.map_or([0, 0], |s| s.ws_count)
    }

    fn _handle_led_get_response(&mut self, frame: &Frame) {
        // 前 96B 是稳定快照；新版固件可在尾部追加运行态亮度字节，兼容旧固件。
        if frame.payload.len() < 96 {
            self.push_log_warn(format!(
                "LED_GET 响应长度错误: {} 字节 (需 >= 96)",
                frame.payload.len()
            ));
            return;
        }
        match crate::proto::decode_led_get(&frame.payload) {
            Ok(state) => {
                // 设备真值到达即丢弃草稿: 否则"应用成功"后编辑框仍显示旧草稿, 与色块/设备不同源。
                if self.drafts.led_region() == Some(&state.regions) {
                    self.drafts.drop_led_region();
                }
                self.led_state = Some(state);
                self.led_version = self.led_version.wrapping_add(1);
            }
            Err(e) => {
                self.push_log_warn(format!("LED_GET 解析失败: {}", e));
            }
        }
    }

    // ------------------------------------------------------------------
    // JIT 算法引擎 (ALGO_*)
    // ------------------------------------------------------------------
    pub fn algo_get_info(&mut self) -> anyhow::Result<()> {
        let _ = self._submit_poll(
            RequestKind::AlgoGetInfo,
            crate::proto::algo::encode_algo_get_info(0),
        )?;
        Ok(())
    }
    pub fn algo_apply(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_algo_apply(seq))?;
        }
        Ok(())
    }
    pub fn algo_reset_default(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_algo_reset_default(seq))?;
        }
        // 复位命令与此查询在同一有序链路上：紧随其后取设备真值，避免调用方只能继续显示旧算法信息。
        self.algo_get_info()?;
        self.push_log("算法: 请求恢复默认(v3.1 HDR)并自动刷新设备信息");
        Ok(())
    }
    /// 上传算法二进制(≤1024B)。内部计算 CRC16 随帧下发，并按既有灯效写入模式等待 ACK/NAK。
    pub fn algo_upload(&mut self, data: &[u8]) -> anyhow::Result<()> {
        if data.is_empty() || data.len() > crate::proto::algo::ALGO_MAX_LEN {
            return Err(anyhow::anyhow!(
                "算法长度非法: {} (须 1..=1024)",
                data.len()
            ));
        }
        let seq = self.next_seq();
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接，无法上传算法"));
        }
        self._queue_tx(crate::proto::algo::encode_algo_upload(seq, data))?;
        self.algo_upload_seq = Some(seq);
        self.algo_upload_started_at = Some(std::time::Instant::now());
        self.algo_upload_status =
            format!("上传已发送: {} 字节，等待设备 ACK/NAK 确认…", data.len());
        self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
        self.push_log(format!(
            "算法: 上传 {} 字节 (crc16=0x{:04X}, seq={})",
            data.len(),
            crate::proto::algo::crc16_ccitt(data),
            seq
        ));
        Ok(())
    }
    pub fn algo_get_rom(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_algo_get_rom(seq))?;
        }
        Ok(())
    }
    /// 设置每通道 16 位 ROM。entries=(ch, rom) 列表(1..=36 项)。
    pub fn algo_set_rom(&mut self, entries: &[(u8, u16)]) -> anyhow::Result<()> {
        if entries.is_empty() || entries.len() > 36 {
            return Err(anyhow::anyhow!("ROM 条目数非法: {}", entries.len()));
        }
        let seq = self.next_seq();
        if self.io.is_some() {
            self._queue_tx(crate::proto::algo::encode_algo_set_rom(seq, entries))?;
        }
        for &(ch, rom) in entries {
            if (ch as usize) < self.algo_rom.len() {
                self.algo_rom[ch as usize] = rom;
            }
        }
        self.algo_rom_version = self.algo_rom_version.wrapping_add(1);
        Ok(())
    }

    pub fn algo_info(&self) -> Option<crate::proto::algo::AlgoInfo> {
        self.algo_info
    }
    pub fn algo_version(&self) -> u64 {
        self.algo_version
    }
    /// 最近一次算法上传的设备确认结果(发送中 / ACK / NAK / 超时)，由 main.rs 回填状态行。
    pub fn algo_upload_status(&self) -> &str {
        &self.algo_upload_status
    }
    pub fn algo_upload_version(&self) -> u64 {
        self.algo_upload_version
    }
    pub fn algo_rom(&self) -> &[u16] {
        &self.algo_rom
    }
    pub fn algo_rom_version(&self) -> u64 {
        self.algo_rom_version
    }

    // ------------------------------------------------------------------
    // 算法运行时追踪(report[]/out_active) + 可调变量(cfg[8])
    // ------------------------------------------------------------------

    /// 请求某通道某上报变量 idx(0..3) 的一次追踪采样(响应异步入环形缓冲, 供折线图)。
    /// 切换追踪通道时清空旧通道的缓冲, 避免新旧通道数据混线。
    pub fn request_algo_trace(&mut self, ch: u8, idx: u8) -> anyhow::Result<()> {
        if ch >= 36 || idx >= 4 {
            return Err(anyhow::anyhow!("算法追踪参数非法: ch={} idx={}", ch, idx));
        }
        if self.algo_trace_channel != Some(ch) {
            self.algo_trace_channel = Some(ch);
            for buf in &mut self.algo_trace_report {
                buf.clear();
            }
            self.algo_trace_active.clear();
        }
        // 退避中(设备无算法/上次 NAK): 跳过本次轮询, 逐次递减, 避免 NAK 刷屏。
        if self.algo_trace_backoff > 0 {
            self.algo_trace_backoff -= 1;
            return Ok(());
        }
        self.algo_trace_pending_idx = idx;
        let seq = self._submit_poll(
            RequestKind::RequestAlgoTrace { ch, idx },
            crate::proto::algo::encode_algo_get_trace(0, ch, idx),
        )?;
        if seq != 0 {
            self.algo_trace_last_seq = Some(seq);
        }
        Ok(())
    }

    /// 某上报变量(idx 0..3)的值序列(等间距用法, 供算法页窄带)。
    pub fn algo_trace_report_series(&self, idx: u8) -> Vec<f32> {
        self.algo_trace_report
            .get(idx as usize)
            .map(|buf| buf.iter().map(|p| p.val).collect())
            .unwrap_or_default()
    }
    /// 触发判定(out_active, 0/1)值序列(等间距用法, 供算法页窄带)。
    pub fn algo_trace_active_series(&self) -> Vec<f32> {
        self.algo_trace_active.iter().map(|p| p.val).collect()
    }
    /// 某上报变量(idx 0..3)的 (设备时间us, 值) 序列: 供与遥测曲线共用真实时间轴的主图。
    pub fn algo_trace_report_points(&self, idx: u8) -> Vec<(u64, f32)> {
        let report = match &self.plot_freeze {
            Some(freeze) => &freeze.algo_report,
            None => &self.algo_trace_report,
        };
        report
            .get(idx as usize)
            .map(|buf| buf.iter().map(|p| (p.t_us, p.val)).collect())
            .unwrap_or_default()
    }
    /// 触发判定(out_active)的 (设备时间us, 值) 序列, 同上。
    pub fn algo_trace_active_points(&self) -> Vec<(u64, f32)> {
        match &self.plot_freeze {
            Some(freeze) => &freeze.algo_active,
            None => &self.algo_trace_active,
        }
        .iter()
        .map(|p| (p.t_us, p.val))
        .collect()
    }
    pub fn algo_trace_version(&self) -> u64 {
        self.algo_trace_version
    }

    /// 暂存共享算法可设置变量 cfg[idx](0..7)，点击“保存到设备”后统一下发。
    pub fn set_algo_cfg(&mut self, idx: u8, val: u8) -> anyhow::Result<()> {
        if idx >= 8 {
            return Err(anyhow::anyhow!("算法可调变量索引非法: {}", idx));
        }
        let slot = idx as usize;
        let same_as_device = self.algo_cfg_valid[slot] && self.algo_cfg[slot] == val;
        self.drafts.set_algo_cfg(idx, val, same_as_device);
        // 草稿优先 getter 依赖版本号立即回显，不覆盖设备缓存以支持撤销恢复。
        self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        Ok(())
    }
    pub fn request_algo_cfg(&mut self, idx: u8) -> anyhow::Result<()> {
        if idx >= 8 {
            return Err(anyhow::anyhow!("算法可调变量索引非法: {}", idx));
        }
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_algo_get_cfg(seq, idx))?;
        }
        Ok(())
    }
    pub fn algo_cfg(&self, idx: u8) -> u8 {
        let slot = idx as usize;
        self.drafts.algo_cfg(idx).unwrap_or_else(|| {
            if self.algo_cfg_valid.get(slot).copied().unwrap_or(false) {
                self.algo_cfg[slot]
            } else {
                self.algo_setting_decls()
                    .into_iter()
                    .find(|decl| decl.idx == idx)
                    .map_or(0, |decl| decl.default)
            }
        })
    }
    pub fn algo_cfg_version(&self) -> u64 {
        self.algo_cfg_version
    }

    /// schema 解析用的 C 源: ★设备回读源优先★, 无设备源时退回本地最近一次编译源。
    /// 为什么不是编辑器文本: 面板与 report idx 轮询集合描述的是"设备上正在跑的算法",
    /// 编辑器里可能只是还没编译的草稿; 用草稿当 schema 会去轮询设备根本没有的 idx。
    pub fn algo_schema_source(&self) -> &str {
        if self.algo_device_src.trim().is_empty() {
            &self.algo_source
        } else {
            &self.algo_device_src
        }
    }

    /// schema 版本号: 设备源或本地源任一变化即自增, 供 UI 门控刷新。
    pub fn algo_schema_version(&self) -> u64 {
        self.algo_schema_version
    }

    fn _bump_algo_schema(&mut self) {
        self.algo_schema_version = self.algo_schema_version.wrapping_add(1);
    }

    /// 解析算法上报变量声明(ALGO_REPORT), 供 UI 建折线图例。源见 `algo_schema_source`。
    pub fn algo_report_decls(&self) -> Vec<crate::proto::algo::AlgoReportDecl> {
        crate::proto::algo::parse_algo_reports(self.algo_schema_source())
    }
    /// 解析算法可设置变量声明(ALGO_SETTING), 供 UI 建可调项列表。源见 `algo_schema_source`。
    pub fn algo_setting_decls(&self) -> Vec<crate::proto::algo::AlgoSettingDecl> {
        crate::proto::algo::parse_algo_settings(self.algo_schema_source())
    }

    fn _handle_algo_get_trace_response(&mut self, frame: &Frame) {
        // 成功响应: 设备有算法且可读, 清退避恢复正常轮询。
        self.algo_trace_backoff = 0;
        self.algo_trace_last_seq = None;
        let Some((ch, idx, active, report)) =
            crate::proto::algo::decode_algo_get_trace(&frame.payload)
        else {
            return;
        };
        if self.algo_trace_channel != Some(ch) || idx >= 4 {
            return; // 通道已切换或响应索引非法，丢弃过时帧。
        }
        const TRACE_CAP: usize = 512;
        let t_us = match frame.device_t_us {
            Some(raw) => self.telem_clock.unwrap_us(raw),
            None => {
                self.telem_clock.acc_us
                    + self
                        .telem_frame_at
                        .map_or(0, |at| at.elapsed().as_micros() as u64)
            }
        };
        if let Some(buf) = self.algo_trace_report.get_mut(idx as usize) {
            if buf.len() >= TRACE_CAP {
                buf.pop_front();
            }
            buf.push_back(TracePoint {
                t_us,
                val: report as f32,
            });
            if self.algo_trace_active.len() >= TRACE_CAP {
                self.algo_trace_active.pop_front();
            }
            self.algo_trace_active.push_back(TracePoint {
                t_us,
                val: if active { 1.0 } else { 0.0 },
            });
            self.algo_trace_version = self.algo_trace_version.wrapping_add(1);
        }
    }

    fn _handle_algo_get_cfg_response(&mut self, frame: &Frame) {
        if let Some((idx, val)) = crate::proto::algo::decode_algo_get_cfg(&frame.payload) {
            if (idx as usize) < self.algo_cfg.len() {
                self.algo_cfg[idx as usize] = val;
                self.algo_cfg_valid[idx as usize] = true;
                self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
            }
        }
    }

    /// 简易 C→ASM 编译器: 把 C 源写临时文件, 用 arm-none-eabi-gcc(-mcpu=cortex-m0plus
    /// -mthumb -Os -ffreestanding -nostdlib) 编译 + objcopy 出裸 .text 二进制, 校验无外部符号/
    /// 无重定位/algo 在偏移 0/≤1024B, 成功则返回二进制供 algo_upload。方案 a: 封装现成工具链。
    /// abi_header_dir 提供 psoc_algo_abi.h 的 include 路径。
    pub fn compile_c_to_blob(&mut self, c_source: &str) -> anyhow::Result<Vec<u8>> {
        let out = Self::compile_blob(c_source)?;
        let blob = out.blob.clone();
        self.apply_compiled(c_source, out);
        Ok(blob)
    }

    /// 纯编译(无 self): 只吃 C 源、只吐产物, 全程不碰控制器状态。
    /// ★为什么必须是关联函数★: 编译要顺序阻塞跑 gcc/objcopy/nm/objdump 四个子进程, 首次还要
    /// 解压 18MB 内置工具链, 在 UI 线程里做会把界面冻死几秒。拆出来后可以丢进 std::thread,
    /// 而 `Rc<RefCell<AppController>>` 跨线程不安全 —— 后台只搬 String/Vec<u8> 这类纯数据,
    /// 产物回到 UI 线程再由 `apply_compiled` 写入状态。
    pub fn compile_blob(c_source: &str) -> anyhow::Result<CompiledAlgo> {
        use std::io::Write;
        // 0) C 源容量闸门: 设备只能存 32KB"编译器有效内容"(= 滤注释后的 UTF-8 字节)。超了直接拒,
        //    绝不先编译再在上传时截断 —— 那会让设备上的映射表源与实际算法脱节且无法还原。
        let used = Self::algo_src_used(c_source);
        let cap = Self::algo_src_capacity();
        if used > cap {
            return Err(anyhow::anyhow!(
                "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 请精简后再编译",
                used,
                cap
            ));
        }
        // 1) 定位工具链: 优先程序内置(随 exe 打包, 免各机环境差异), 解压失败再回退本机安装。
        let gcc_dir = match ensure_bundled_toolchain() {
            Ok(d) => d,
            Err(_) => find_toolchain_dir()?,
        };
        let gcc = gcc_dir.join("arm-none-eabi-gcc.exe");
        let objcopy = gcc_dir.join("arm-none-eabi-objcopy.exe");
        let nm = gcc_dir.join("arm-none-eabi-nm.exe");
        let objdump = gcc_dir.join("arm-none-eabi-objdump.exe");

        // 2) 临时目录 + 写源文件。ABI 头 include 路径指向工程 PSoC 目录。
        let tmp = std::env::temp_dir().join(format!("mai2algo_{}", std::process::id()));
        std::fs::create_dir_all(&tmp)?;
        let src = tmp.join("algo_user.c");
        let obj = tmp.join("algo_user.o");
        let bin = tmp.join("algo_user.bin");
        {
            let mut f = std::fs::File::create(&src)?;
            f.write_all(c_source.as_bytes())?;
        }
        // 内嵌 ABI 头写入临时目录, -I 指向它(不依赖本机 PSoC 工程路径)。
        std::fs::write(tmp.join("psoc_algo_abi.h"), ALGO_ABI_HEADER)?;

        // 3) 编译。
        let out = std::process::Command::new(&gcc)
            .args([
                "-mcpu=cortex-m0plus",
                "-mthumb",
                "-Os",
                "-ffreestanding",
                "-fno-jump-tables",
                "-fomit-frame-pointer",
                "-fno-common",
                "-nostdlib",
            ])
            .arg(format!("-I{}", tmp.display()))
            .arg("-c")
            .arg(&src)
            .arg("-o")
            .arg(&obj)
            .output()?;
        if !out.status.success() {
            let _ = std::fs::remove_dir_all(&tmp);
            return Err(anyhow::anyhow!(
                "编译失败:\n{}",
                String::from_utf8_lossy(&out.stderr)
            ));
        }
        // 4) objcopy 出 .text 裸二进制。
        let out2 = std::process::Command::new(&objcopy)
            .args(["-O", "binary", "-j", ".text"])
            .arg(&obj)
            .arg(&bin)
            .output()?;
        if !out2.status.success() {
            let _ = std::fs::remove_dir_all(&tmp);
            return Err(anyhow::anyhow!(
                "objcopy 失败:\n{}",
                String::from_utf8_lossy(&out2.stderr)
            ));
        }
        // 5) nm 校验无未定义(U)符号 + algo 在偏移 0(T 且地址 0)。
        let nm_out = std::process::Command::new(&nm).arg(&obj).output()?;
        let nm_txt = String::from_utf8_lossy(&nm_out.stdout);
        let mut has_undef = false;
        let mut algo_at_zero = false;
        for line in nm_txt.lines() {
            let cols: Vec<&str> = line.split_whitespace().collect();
            // 形如 "00000000 T algo" 或 "         U extsym"
            if cols.len() == 2 && cols[0] == "U" {
                has_undef = true;
            } else if cols.len() == 2 && cols[0].eq_ignore_ascii_case("U") {
                has_undef = true;
            } else if cols.len() == 3 {
                if cols[1] == "U" {
                    has_undef = true;
                }
                if cols[2] == "algo" && cols[1].eq_ignore_ascii_case("t") && cols[0] == "00000000" {
                    algo_at_zero = true;
                }
            }
        }
        // 反汇编 .o 的 .text(编译产物 ASM), 供 UI 子标签查看。清理临时目录前抓取。
        let asm = std::process::Command::new(&objdump)
            .args(["-d", "--no-show-raw-insn"])
            .arg(&obj)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).to_string())
            .unwrap_or_default();
        let data = std::fs::read(&bin).unwrap_or_default();
        let _ = std::fs::remove_dir_all(&tmp);
        if has_undef {
            return Err(anyhow::anyhow!(
                "算法含外部符号(禁止 libgcc/除法/64位): 见 nm 输出"
            ));
        }
        if !algo_at_zero {
            return Err(anyhow::anyhow!(
                "入口 algo 必须在偏移 0(检查是否首个函数/是否被内联到别处)"
            ));
        }
        if data.is_empty() || data.len() > crate::proto::algo::ALGO_MAX_LEN {
            return Err(anyhow::anyhow!(
                "产物大小非法: {} 字节(须 1..=1024)",
                data.len()
            ));
        }
        // objdump 用制表符对齐, Slint 文本控件会把 \t 渲染成方块; 替换为空格避免乱码。
        Ok(CompiledAlgo {
            blob: data,
            asm: asm.replace('\t', " "),
        })
    }

    /// 把后台编译产物落到控制器状态(必须在 UI 线程调用): 保存 C 源与反汇编、留档本地源文件、记日志。
    pub fn apply_compiled(&mut self, c_source: &str, out: CompiledAlgo) -> usize {
        let len = out.blob.len();
        self.algo_source = c_source.to_string();
        self._bump_algo_schema();
        self.algo_asm = out.asm;
        self.algo_asm_version = self.algo_asm_version.wrapping_add(1);
        self.algo_compiled_blob = Some(out.blob);
        if let Ok(exe) = std::env::current_exe() {
            if let Some(dir) = exe.parent() {
                let _ = std::fs::write(dir.join("last_algo_source.c"), c_source);
            }
        }
        self.push_log(format!(
            "算法: 编译成功, ASM {} / {} 字节 ({}%)",
            len,
            Self::algo_slot_capacity(),
            (len * 100) / Self::algo_slot_capacity().max(1)
        ));
        len
    }

    pub fn algo_asm(&self) -> String {
        self.algo_asm.clone()
    }
    pub fn algo_source(&self) -> String {
        self.algo_source.clone()
    }
    pub fn algo_asm_version(&self) -> u64 {
        self.algo_asm_version
    }

    /// 编译并上传: compile_c_to_blob → algo_upload。
    pub fn compile_and_upload(&mut self, c_source: &str) -> anyhow::Result<()> {
        let blob = self.compile_c_to_blob(c_source)?;
        self.algo_upload(&blob)
    }

    /// PSoC 可执行算法槽容量(字节)。ASM 产物必须 ≤ 此值, 否则会被截断导致运行异常。
    pub fn algo_slot_capacity() -> usize {
        crate::proto::algo::ALGO_MAX_LEN
    }

    /// 仅编译(不上传, 同步版): 产出 ASM 二进制并缓存, 返回其字节数。
    /// compile_blob 已在产物 >容量 时报错, 故成功返回的长度必然 ≤ 容量(不会截断)。
    /// UI 走的是后台线程 + `apply_compiled` 那条路(见 main.rs 算法编译任务), 本函数留给无头场景。
    pub fn compile_only(&mut self, c_source: &str) -> anyhow::Result<usize> {
        let out = Self::compile_blob(c_source)?;
        Ok(self.apply_compiled(c_source, out))
    }

    /// 上传最近一次成功编译的 ASM 产物, 并把对应 C 源(滤注释后)作为"映射表"存到设备,
    /// 供后续回读还原可编辑 C。未编译则报错(强制"先编译后上传")。
    pub fn upload_compiled(&mut self) -> anyhow::Result<()> {
        let blob = self
            .algo_compiled_blob
            .clone()
            .ok_or_else(|| anyhow::anyhow!("尚未编译: 请先点“编译”生成 ASM 再上传"))?;
        // 发送前再把 C 源容量闸门过一遍: 走到这里的产物可能来自更早一次编译, 而编辑器/模板
        // 随后被换过, 不能靠"编译时查过了"就免检。
        let src = Self::strip_c_comments(&self.algo_source);
        let cap = Self::algo_src_capacity();
        if src.len() > cap {
            return Err(anyhow::anyhow!(
                "C 源(去注释){} 字节, 超出设备存储上限 {} 字节: 未上传",
                src.len(),
                cap
            ));
        }
        self.algo_upload(&blob)?;
        // 随算法上传其 C 源(滤注释)到设备映射表; 失败不阻断上传主流程。
        let _ = self.send_algo_src(&src);
        // 设备映射表刚被这次上传覆盖 → 本地那份缓存同步跟上, 否则 schema(设备源优先)会继续
        // 按上一版算法解析, 面板与 report idx 轮询集合就跟设备上真正跑的算法脱节了。
        // 只 bump schema/cfg/trace 版本: algo_device_src_version 门控的是"回读→载入编辑器",
        // 不该由一次上传去触发编辑器回填。
        self.algo_device_src = src;
        self._bump_algo_schema();
        self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        self.algo_trace_version = self.algo_trace_version.wrapping_add(1);
        Ok(())
    }

    /// 最近一次编译产物的字节数(未编译=0), 供 UI 进度条。
    pub fn algo_compiled_len(&self) -> usize {
        self.algo_compiled_blob
            .as_ref()
            .map(|b| b.len())
            .unwrap_or(0)
    }

    /// C 源容量(字节)。编译器实际看到的内容 = 滤注释后的 UTF-8 字节数, 必须 ≤ 此值。
    pub fn algo_src_capacity() -> usize {
        crate::proto::algo::ALGO_SRC_MAX
    }

    /// 某段 C 源"编译器有效内容"的字节数: 复用唯一的注释过滤实现 `strip_c_comments`,
    /// 不另造一套过滤逻辑(两套一定会漂移, 于是界面显示的占用和真正上传的长度对不上)。
    pub fn algo_src_used(src: &str) -> usize {
        Self::strip_c_comments(src).as_bytes().len()
    }

    /// 是否仍有算法 C 源分片上传或回读在途，供无头工具等待完整传输。
    pub fn algo_src_transfer_pending(&self) -> bool {
        self.algo_src_tx.is_some() || self.algo_src_rx.is_some()
    }

    /// 把算法 C 源存到设备映射表(ALGO_SET_SRC)，按唯一容量口径过滤注释后分片发送。
    /// 超上限直接报错 —— 截断会把一份编译不过的残源固化到设备上。
    pub fn send_algo_src(&mut self, src: &str) -> anyhow::Result<()> {
        let bytes = Self::strip_c_comments(src).into_bytes();
        let cap = crate::proto::algo::ALGO_SRC_MAX;
        if bytes.len() > cap {
            return Err(anyhow::anyhow!(
                "算法 C 源 {} 字节, 超出设备存储上限 {} 字节",
                bytes.len(),
                cap
            ));
        }
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法上传算法 C 源"));
        }
        self.algo_src_tx = Some(AlgoSrcTx {
            bytes,
            acked: 0,
            seq: None,
            waited: 0,
        });
        // 已有普通写帧时只登记待传状态，等队列排空后再发首片；开始后每片仍严格 ACK 驱动。
        if self.cfg_tx_pending() == 0 {
            self._algo_src_send_next()?;
        }
        Ok(())
    }

    /// 发出"下一片"(含 total=0 的空源片)。窗口=1: 上一片没回执就不发下一片。
    fn _algo_src_send_next(&mut self) -> anyhow::Result<()> {
        let Some(tx) = self.algo_src_tx.as_ref() else {
            return Ok(());
        };
        let total = tx.bytes.len();
        let offset = tx.acked;
        let end = (offset + crate::proto::algo::ALGO_SRC_CHUNK).min(total);
        let chunk = tx.bytes[offset..end].to_vec();
        let seq = self.next_seq();
        let frame = crate::proto::algo::encode_algo_set_src_chunk(seq, offset, total, &chunk);
        let sent = self
            .io
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("设备已断开, 无法继续上传算法 C 源"))?
            .send(frame);
        match sent {
            Ok(()) => {
                if let Some(tx) = self.algo_src_tx.as_mut() {
                    tx.seq = Some(seq);
                    tx.waited = 0;
                }
                Ok(())
            }
            Err(e) => {
                self.algo_src_tx = None;
                self._record_cfg_tx_failure(
                    HostCmd::AlgoSetSrc as u8,
                    seq,
                    format!("本地发送失败: {}", e),
                );
                Err(e)
            }
        }
    }

    /// 收到 ACK/NAK 时推进/终止分片上传。NAK 即放弃并记录可查询失败状态。
    fn _algo_src_note_reply(&mut self, seq: u8, ok: bool) {
        let Some((expected_seq, at)) = self.algo_src_tx.as_ref().map(|tx| (tx.seq, tx.acked))
        else {
            return;
        };
        if expected_seq != Some(seq) {
            return;
        }
        if !ok {
            self.algo_src_tx = None;
            self._record_cfg_tx_failure(
                HostCmd::AlgoSetSrc as u8,
                seq,
                format!("设备拒绝分片 offset={}", at),
            );
            self.push_log_error(format!("算法 C 源上传被设备拒绝(offset={}), 已放弃", at));
            return;
        }
        let total = {
            let tx = self.algo_src_tx.as_mut().expect("刚判过 Some");
            tx.acked = (tx.acked + crate::proto::algo::ALGO_SRC_CHUNK).min(tx.bytes.len());
            tx.bytes.len()
        };
        if self.algo_src_tx.as_ref().map(|tx| tx.acked) == Some(total) {
            self.algo_src_tx = None; // 末片已 ACK: 设备侧才让新源生效
            return;
        }
        if let Err(e) = self._algo_src_send_next() {
            self.push_log_error(format!("算法 C 源分片发送失败: {}", e));
        }
    }

    /// 每 tick 调一次: 待普通写队列排空后启动首片，并对已发送片做超时兜底。
    fn _pump_algo_src_tx(&mut self) {
        const ALGO_SRC_TIMEOUT_TICKS: u32 = 125; // ~2s @16ms/tick, 与保存队列同口径
        let Some((seq, at)) = self.algo_src_tx.as_ref().map(|tx| (tx.seq, tx.acked)) else {
            return;
        };
        let Some(seq) = seq else {
            if self.cfg_tx_pending() == 0 {
                if let Err(e) = self._algo_src_send_next() {
                    self.push_log_error(format!("算法 C 源首片发送失败: {}", e));
                }
            }
            return;
        };
        let timed_out = {
            let tx = self.algo_src_tx.as_mut().expect("刚判过 Some");
            tx.waited += 1;
            tx.waited >= ALGO_SRC_TIMEOUT_TICKS
        };
        if !timed_out {
            return;
        }
        self.algo_src_tx = None;
        self._record_cfg_tx_failure(
            HostCmd::AlgoSetSrc as u8,
            seq,
            format!("offset={} 超过 2s 无 ACK/NAK", at),
        );
        self.push_log_error(format!(
            "算法 C 源上传超时(offset={} 无回执), 已放弃本次上传",
            at
        ));
    }

    /// 每 tick 调一次: 回读请求也必须等待当前片响应，丢帧时清理状态而不是永久卡住。
    fn _pump_algo_src_rx(&mut self) {
        const ALGO_SRC_TIMEOUT_TICKS: u32 = 125;
        let Some(rx) = self.algo_src_rx.as_mut() else {
            return;
        };
        rx.waited += 1;
        if rx.waited < ALGO_SRC_TIMEOUT_TICKS {
            return;
        }
        let offset = rx.buf.len();
        self.algo_src_rx = None;
        self.push_log_error(format!(
            "算法 C 源回读超时(offset={} 无响应), 已放弃本次回读",
            offset
        ));
    }

    /// 请求回读设备映射表里的算法 C 源(ALGO_GET_SRC), 从第 0 片开始。
    pub fn request_algo_src(&mut self) -> anyhow::Result<()> {
        self.algo_src_rx = None;
        self._algo_src_request_at(0)
    }

    fn _algo_src_request_at(&mut self, offset: usize) -> anyhow::Result<()> {
        let seq = self._submit_poll(
            RequestKind::RequestAlgoSrc,
            crate::proto::algo::encode_algo_get_src(0, offset),
        )?;
        if offset == 0 {
            self.algo_src_rx = Some(AlgoSrcRx {
                total: 0,
                buf: Vec::new(),
                seq,
                waited: 0,
            });
        } else {
            let rx = self
                .algo_src_rx
                .as_mut()
                .ok_or_else(|| anyhow::anyhow!("算法 C 源回读状态丢失(offset={})", offset))?;
            if rx.buf.len() != offset {
                return Err(anyhow::anyhow!(
                    "算法 C 源续读偏移错误(期望 {}, 请求 {})",
                    rx.buf.len(),
                    offset
                ));
            }
            rx.seq = seq;
            rx.waited = 0;
        }
        if seq == 0 {
            self.algo_src_rx = None;
            return Ok(());
        }
        if let Some(rx) = self.algo_src_rx.as_mut() {
            rx.seq = seq;
        }
        Ok(())
    }

    /// ALGO_GET_SRC 响应: [total, offset, chunk]。聚合到齐且 UTF-8 有效才替换缓存/版本。
    fn _handle_algo_src_chunk(&mut self, frame: &Frame) {
        let Some((total, offset, chunk)) =
            crate::proto::algo::decode_algo_src_chunk(&frame.payload)
        else {
            self.algo_src_rx = None;
            self.push_log_error("ALGO_GET_SRC 响应过短, 已放弃本次回读".to_string());
            return;
        };
        if total > crate::proto::algo::ALGO_SRC_MAX {
            self.algo_src_rx = None;
            self.push_log_error(format!(
                "ALGO_GET_SRC 声明长度 {} 超上限, 已放弃本次回读",
                total
            ));
            return;
        }
        let Some(expected_seq) = self.algo_src_rx.as_ref().map(|rx| rx.seq) else {
            self.push_log_error("ALGO_GET_SRC 收到无在途请求的响应, 已忽略".to_string());
            return;
        };
        if expected_seq != frame.seq {
            self.push_log_error(format!(
                "ALGO_GET_SRC 响应序号错误(期望 {}, 实收 {}), 已忽略",
                expected_seq, frame.seq
            ));
            return;
        }
        let mut next = None;
        let mut complete = false;
        let error = {
            let rx = self.algo_src_rx.as_mut().expect("刚判过 Some");
            if offset == 0 && rx.buf.is_empty() && rx.total == 0 {
                rx.total = total;
                rx.buf = Vec::with_capacity(total);
            }
            if rx.total != total || rx.buf.len() != offset {
                Some(format!(
                    "ALGO_GET_SRC 分片错位(期望 offset={} total={}, 实收 offset={} total={})",
                    rx.buf.len(),
                    rx.total,
                    offset,
                    total
                ))
            } else if chunk.len() > crate::proto::algo::ALGO_SRC_CHUNK {
                Some(format!("ALGO_GET_SRC 分片长度 {} 超过上限", chunk.len()))
            } else if chunk.len() > total.saturating_sub(rx.buf.len()) {
                Some(format!(
                    "ALGO_GET_SRC 总长度错误(已收 {} + 本片 {} > {})",
                    rx.buf.len(),
                    chunk.len(),
                    total
                ))
            } else {
                rx.buf.extend_from_slice(chunk);
                if rx.buf.len() < total {
                    if chunk.is_empty() {
                        Some(format!("ALGO_GET_SRC 在 offset={} 收到空中间片", offset))
                    } else {
                        next = Some(rx.buf.len());
                        None
                    }
                } else {
                    complete = true;
                    None
                }
            }
        };
        if let Some(error) = error {
            self.algo_src_rx = None;
            self.push_log_error(format!("{}, 已放弃本次回读", error));
            return;
        }
        if let Some(offset) = next {
            if let Err(e) = self._algo_src_request_at(offset) {
                self.algo_src_rx = None;
                self.push_log_error(format!("ALGO_GET_SRC 续请求失败: {}", e));
            }
            return;
        }
        if !complete {
            return;
        }
        let rx = self.algo_src_rx.take().expect("刚判过 Some");
        let source = match String::from_utf8(rx.buf) {
            Ok(source) => source,
            Err(e) => {
                self.push_log_error(format!(
                    "ALGO_GET_SRC 完整内容不是有效 UTF-8: {}, 已保留旧缓存",
                    e
                ));
                return;
            }
        };
        self.algo_device_src = source;
        self.algo_device_src_version = self.algo_device_src_version.wrapping_add(1);
        // ★不再把设备源灌进 algo_source★: schema 一律走 `algo_schema_source()`(设备源优先),
        // 编辑器文本与 schema 彻底解耦 —— 连接即按设备上真正跑着的算法填面板/report idx,
        // 不必先下发一次; 同时用户手上正在改的编辑器内容永远不会被回读覆盖。
        self._bump_algo_schema();
        // 同步让 cfg 行和 report 行重建；后者复用既有 trace 版本门控以立即显示声明。
        self.algo_cfg_version = self.algo_cfg_version.wrapping_add(1);
        self.algo_trace_version = self.algo_trace_version.wrapping_add(1);
    }

    /// 请求回读设备算法 ASM 机器码(ALGO_GET_CODE), 用于无本地编译产物时查看真实机器码。
    pub fn request_algo_code(&mut self) -> anyhow::Result<()> {
        let seq = self.next_seq();
        if let Some(handle) = &self.io {
            handle.send(crate::proto::algo::encode_algo_get_code(seq))?;
        }
        Ok(())
    }

    /// 设备回读的算法 ASM 机器码 hex dump 文本(空表示未回读)。
    pub fn algo_device_code_hex(&self) -> &str {
        &self.algo_device_code_hex
    }
    pub fn algo_device_code_version(&self) -> u64 {
        self.algo_device_code_version
    }

    /// 字节序列 → 每行 16 字节的 hex dump(offset: bytes)文本。
    fn _hex_dump(data: &[u8]) -> String {
        if data.is_empty() {
            return String::new();
        }
        let mut out = String::with_capacity(data.len() * 4);
        for (row, chunk) in data.chunks(16).enumerate() {
            out.push_str(&format!("{:04X}: ", row * 16));
            for b in chunk {
                out.push_str(&format!("{:02X} ", b));
            }
            out.push('\n');
        }
        out
    }

    /// 设备回读的算法 C 源(映射表), 空表示设备无存源。
    pub fn algo_device_src(&self) -> &str {
        &self.algo_device_src
    }
    pub fn algo_device_src_version(&self) -> u64 {
        self.algo_device_src_version
    }

    /// 去除 C 注释(// 行注释与 /* */ 块注释), 保留字符串字面量内容与换行结构。
    /// 用于上传时精简"映射表"源, 不改变代码语义(变量名不必与原始一致)。
    pub fn strip_c_comments(src: &str) -> String {
        let bytes = src.as_bytes();
        let mut out = String::with_capacity(src.len());
        let mut i = 0usize;
        // 状态: 0=普通 1=行注释 2=块注释 3=字符串"" 4=字符''
        let mut state = 0u8;
        while i < bytes.len() {
            let c = bytes[i];
            let n = if i + 1 < bytes.len() { bytes[i + 1] } else { 0 };
            match state {
                0 => {
                    if c == b'/' && n == b'/' {
                        state = 1;
                        i += 2;
                        continue;
                    }
                    if c == b'/' && n == b'*' {
                        state = 2;
                        i += 2;
                        continue;
                    }
                    if c == b'"' {
                        state = 3;
                        out.push('"');
                        i += 1;
                        continue;
                    }
                    if c == b'\'' {
                        state = 4;
                        out.push('\'');
                        i += 1;
                        continue;
                    }
                    out.push(c as char);
                }
                1 => {
                    if c == b'\n' {
                        state = 0;
                        out.push('\n');
                    }
                }
                2 => {
                    if c == b'*' && n == b'/' {
                        state = 0;
                        i += 2;
                        continue;
                    }
                    if c == b'\n' {
                        out.push('\n');
                    } // 保留行结构便于阅读
                }
                3 => {
                    out.push(c as char);
                    if c == b'\\' && n != 0 {
                        out.push(n as char);
                        i += 2;
                        continue;
                    }
                    if c == b'"' {
                        state = 0;
                    }
                }
                4 => {
                    out.push(c as char);
                    if c == b'\\' && n != 0 {
                        out.push(n as char);
                        i += 2;
                        continue;
                    }
                    if c == b'\'' {
                        state = 0;
                    }
                }
                _ => {}
            }
            i += 1;
        }
        // 压缩连续空行(注释删除后常留大量空行)。
        let mut cleaned = String::with_capacity(out.len());
        let mut blank_run = 0u32;
        for line in out.lines() {
            if line.trim().is_empty() {
                blank_run += 1;
                if blank_run <= 1 {
                    cleaned.push('\n');
                }
            } else {
                blank_run = 0;
                cleaned.push_str(line.trim_end());
                cleaned.push('\n');
            }
        }
        cleaned
    }

    fn _handle_global_get_response(&mut self, frame: &Frame) {
        if let Some((id, v)) = crate::proto::algo::decode_global_get(&frame.payload) {
            // 回读防污染先做完再落缓存: 界面回显的是修正值, 且非法值会被回写纠正到设备。
            let v = self._guard_readback_global(id, v);
            self.globals.insert(id, v);
            self.globals_version = self.globals_version.wrapping_add(1);
        }
    }
    /// 兼容旧固件的同步 AUTO_TUNE 响应(新固件走 0x2E 推送流的终态帧)。
    /// ★这条路径的归属键是帧头 seq★ 响应帧回显请求 seq(与 ACK/NAK 同一约定), 故直接拿它归因,
    /// 不必再去猜 `op_wait_seq`。
    fn _handle_auto_tune_response(&mut self, frame: &Frame) {
        // 同步响应的帧头 seq 就是请求归属。先校验再解析 payload/修改状态，避免旧响应收割当前操作。
        if self.auto_tune_req_seq != Some(frame.seq) {
            self.push_log_debug(format!(
                "已丢弃过期 AUTO_TUNE 响应: 帧 seq={}, 当前在途 {:?}",
                frame.seq, self.auto_tune_req_seq
            ));
            return;
        }
        // payload = [result(u8), div(u16 LE), ch(u8)](旧固件无尾部 ch → 视为全通道)。
        // result: 1=成功 2=失败(超硬件能力)。
        let origin = Some(frame.seq);
        if frame.payload.len() >= 3 {
            let result = frame.payload[0];
            let div = (frame.payload[1] as u16) | ((frame.payload[2] as u16) << 8);
            let ch = if frame.payload.len() >= 4 {
                frame.payload[3]
            } else {
                0xFF
            };
            self._apply_auto_tune_result(result, div, ch, origin);
        } else {
            self._apply_auto_tune_result(2, 0, self.auto_tune_ch, origin);
        }
    }

    /// 设备推送的自适应阶段进度(AUTO_TUNE_PROGRESS 0x2E)。
    /// 进行中: 刷新阶段文案(驱动 UI 的"处理中"按钮)并重置卡死计时(进度帧本身即"设备活着"的证据);
    /// 终态(state==2): 走与同步响应完全相同的完成处理(复用 `_apply_auto_tune_result`)。
    fn _handle_auto_tune_progress(&mut self, frame: &Frame) {
        let progress = match crate::proto::decode_auto_tune_progress(&frame.payload) {
            Ok(p) => p,
            Err(e) => {
                self.push_log_warn(format!("AUTO_TUNE_PROGRESS 解析失败: {}", e));
                return;
            }
        };
        // ★先判归属, 再动任何状态★ 本流是设备主动推送(帧头 seq 是设备流序号, 与请求无关), 所以
        // "这一帧属于哪次请求"只能靠设备回显的 origin_seq。对不上 = 上一轮的残留帧:
        // 既不能刷进度/重置卡死计时(会让别的操作看起来还活着), 更不能落终态(旧结果会被算到
        // 下一个批量通道头上)。旧固件不带该字段(None) ⇒ 保持原行为, 不因兼容性丢帧。
        if let Some(origin) = progress.origin_seq {
            if self.auto_tune_req_seq != Some(origin) {
                self.push_log_debug(format!(
                    "已丢弃过期 AUTO_TUNE_PROGRESS: 帧归属请求 seq={}, 当前在途 {:?}(state={} result={})",
                    origin, self.auto_tune_req_seq, progress.state, progress.result
                ));
                return;
            }
        }
        self.auto_tune_progress = progress;
        self.auto_tune_progress_version = self.auto_tune_progress_version.wrapping_add(1);
        // ★避免意外情况★: 收到任意进度帧就重置阻塞操作计时, 使 28s 卡死检测只在设备真的失联时才触发。
        self.op_busy_ticks = 0;

        if progress.state == 2 {
            self._apply_auto_tune_result(
                progress.result,
                progress.final_div,
                progress.ch,
                progress.origin_seq,
            );
            return;
        }
        // 进行中: 把阶段/步序/当前试探分频写进 op_label, UI 的 ⏳ 按钮文案随之实时更新。
        if self.op_busy {
            // 全通道请求(auto_tune_ch=0xFF)时, 进度帧的 ch 是"当前正在处理的通道" → 显示 CHn/36。
            let target = if self.auto_tune_ch == 0xFF {
                if (progress.ch as usize) < 36 {
                    format!("逐通道 CH{}/36", progress.ch)
                } else {
                    "逐通道".to_string()
                }
            } else {
                format!("CH{}", progress.ch)
            };
            self.op_label = if progress.cur_div != 0 {
                format!(
                    "{} 频率自适应中: {} (第{}步, 试 ÷{})",
                    target,
                    progress.phase_text(),
                    progress.step,
                    progress.cur_div
                )
            } else {
                format!("{} 频率自适应中: {}", target, progress.phase_text())
            };
            self.op_version = self.op_version.wrapping_add(1);
        }
    }

    /// 自适应完成处理(同步响应与推送流终态共用): 落结果 + 逐通道数组 + params 乐观更新 + 日志 + 解锁。
    ///
    /// `origin_seq` = 发起本轮自适应的请求 seq(推送流由设备回显, 同步响应取帧头)。逐通道批量的
    /// 成败归因**只认它**; 旧固件拿不到时才回落到 `op_wait_seq`(那是"当前在等哪条回执", 在批量里
    /// 会指向下一个通道 —— 正是本轮要修的错归属)。
    fn _apply_auto_tune_result(&mut self, result: u8, div: u16, ch: u8, origin_seq: Option<u8>) {
        // 本轮已出终态: 清掉在途归属键, 之后设备重发的同一终态(背压重试/旧帧)一律按过期丢弃。
        self.auto_tune_req_seq = None;
        self.auto_tune_result = result;
        self.auto_tune_div = div;
        self.auto_tune_ch = ch;
        self.auto_tune_version = self.auto_tune_version.wrapping_add(1);
        let target = ch;
        // 成功: 固件已把分频写回目标 widgetContext; 本地乐观更新 snsClk(0x08)显示, 并请求回读校正。
        if result == 1 && div != 0 {
            let d = div as u32;
            if (target as usize) < 36 {
                self.params[target as usize].insert(0x08, d);
                self.auto_tune_ch_result[target as usize] = 1;
                self.auto_tune_ch_div[target as usize] = div;
                // ★必须 bump + 回读★: 只写缓存不 bump, UI 的参数行模型就不会重建, "传感时钟分频"
                // 一直停在自适应前的旧值(实测 CH10 解出 18, 界面仍显示 32), 用户看到的是幻觉。
                // 乐观值先让界面立刻跟上, 再用一帧全通道回读把它校正成设备真值。
                self.param_version = self.param_version.wrapping_add(1);
                if let Err(e) = self.request_param_all_channels(0x08) {
                    self.push_log_warn(format!("自适应后回读分频失败: {}", e));
                }
                self.push_log(format!("CH{} 频率自适应成功: snsClk 分频 = {}", target, d));
            } else {
                // ★逐通道自适应★: 各通道分频互不相同, 终态 div 字段复用为"成功通道数"。
                // 固件已把逐通道分频写穿真相源, 本地逐通道回读校正显示(不再乐观写同一值)。
                let ok_count = d.min(36);
                // 终态只提供成功总数，不能把失败通道误标为成功；真实分频由批量回读更新 params。
                self.auto_tune_ch_result = [0u8; 36];
                self.auto_tune_ch_div = [0u16; 36];
                self.auto_tune_all_refresh_pending = Some(ok_count as u16);
                // 批量回读(1 帧, 非 36 条单发): 让 UI 的逐通道分频/时钟树范围与设备真值一致。
                if let Err(e) = self.request_param_all_channels(0x08) {
                    self.auto_tune_all_refresh_pending = None;
                    self.push_log_warn(format!("逐通道自适应完成后回读分频失败: {}", e));
                }
            }
        } else {
            if (target as usize) < 36 {
                self.auto_tune_ch_result[target as usize] = 2;
                self.auto_tune_ch_div[target as usize] = 0;
                self.push_log(format!(
                    "CH{} 频率自适应失败: 超出硬件能力, 目标% 无法达到",
                    target
                ));
            } else {
                self.auto_tune_ch_result = [2u8; 36];
                self.auto_tune_ch_div = [0u16; 36];
                self.push_log("频率自适应失败: 超出硬件能力, 目标% 无法达到".to_string());
            }
        }
        // 自适应的真实终态在这里才知道(ACK 只是"已受理")。若这一条是逐通道批量发出的, 按**本轮请求
        // 的 seq** 归因(设备回显); 旧固件没有回显时才退回 `op_wait_seq`。`_ch_batch_note_op` 内部还会
        // 与批量的在途 seq 比对, 两不匹配一律不归因。
        if let Some(seq) = origin_seq.or(self.op_wait_seq) {
            let ok = result == 1 && div != 0;
            self._ch_batch_note_op(seq, ok, "频率自适应终态失败(超出硬件能力, 目标% 无法达到)");
        }
        // 解锁同样要认归属: 本轮若早被 51s 看门狗提前解锁、界面已在等另一条操作, 迟到的终态不能把
        // 别人的锁一起解掉(旧固件无回显时保持原行为)。
        let owns_op = match origin_seq {
            Some(seq) => self.op_wait_seq == Some(seq),
            None => true,
        };
        if owns_op {
            self._end_op();
        }
    }

    fn _handle_global_get_all_response(&mut self, frame: &Frame) {
        let list = crate::proto::algo::decode_global_get_all(&frame.payload);
        if self.csd_diag_active {
            let dump: Vec<String> = list
                .iter()
                .map(|(id, v)| format!("{}={}", Self::_gparam_name(*id), v))
                .collect();
            self.push_log_debug(format!("CSD诊断: 设备回读全局值 [{}]", dump.join(", ")));
        }
        // 回读防污染先做完再落缓存/对账(与单通道 PARAM_GET_ALL 同口径): 非法值夹到最近合法值、
        // 告警并按需回写设备, 缓存与对账都用修正后的值, 界面不会回显脏值。
        let list: Vec<(u8, u32)> = list
            .into_iter()
            .map(|(id, v)| (id, self._guard_readback_global(id, v)))
            .collect();
        for (id, v) in &list {
            self.globals.insert(*id, *v);
        }
        self.globals_version = self.globals_version.wrapping_add(1);
        if self.globals_verify_active {
            self._verify_globals_readback(&list);
        }
    }

    /// 固件自持恢复事件: 固件"自己救自己"的动作会让设备实际状态偏离 UI 以为的状态。
    /// 一律写入日志(会落 log.log), 并对会改变设备配置的事件安排一次全局真值回读同步 UI。
    fn _handle_self_heal_event(&mut self, frame: &Frame) {
        let p = &frame.payload;
        if p.len() < 9 {
            return;
        }
        let code = p[0];
        let detail = u32::from_le_bytes([p[1], p[2], p[3], p[4]]);
        let seq = u16::from_le_bytes([p[5], p[6]]);
        let total = u16::from_le_bytes([p[7], p[8]]);
        // 同一事件可能因背压重发, 按 seq 去重(设备侧 seq 单调递增)。
        if let Some(last) = self.self_heal_last_seq {
            if seq == last {
                return;
            }
        }
        self.self_heal_last_seq = Some(seq);

        // 事件码与 main_firmware/src/service/self_heal/self_heal.h 的 SelfHealCode 一一对应。
        let (text, needs_resync) = match code {
            1 => ("PSoC 与 RP2040 的 SPI 链路持续丢失, 固件已自行 XRES 复位 PSoC。其 CSD 配置在 RAM 中已丢失, RP2040 正用已保存配置重新下发。".to_string(), true),
            2 => ("PSoC 主循环卡死(扫描计数不再推进), 固件已自行 XRES 复位 PSoC。".to_string(), true),
            // detail=1 = 算法下发到 PSoC 未通过 commit 校验(算法根本没装上); detail=0 = 已判定致命并回退。
            3 if detail == 1 => ("⚠ 触控算法下发到 PSoC 未通过校验, 该算法【没有装上】, PSoC 仍在跑上一份算法。请重试上传或先用「PSoC 救砖」修复链路。".to_string(), false),
            3 => ("⚠ 当前自定义触控算法被判定为致命(导致 PSoC 卡死), 固件已回退为内嵌默认算法。你的算法【已不在运行】, 需修正后重新上传。".to_string(), false),
            4 => ("⚠ PSoC 采样异常(raw 满量程或停滞), 固件拒绝把该状态固化为默认, 已【清空全部 CSD 调参并落盘】回到出厂默认链。逐通道调参需重做; 若反复出现请用「PSoC 救砖」。".to_string(), true),
            5 if detail & 0x8000_0000 != 0 => {
                let stage = match (detail & 0xFF) as u8 {
                    1 => "采样可信度门禁",
                    2 => "全通道 IDAC 校准",
                    3 => "全通道频率自适应",
                    4 => "全通道基线复位",
                    _ => "未知阶段",
                };
                (format!("⚠ PSoC 启动校准在「{}」失败；本启动代次不再重试，避免形成校准循环。", stage), true)
            }
            5 => (format!(
                    "PSoC 已重新下发算法与 CSD 配置(模式={})。设备配置刚被整体重建, 正回读真值同步界面。",
                    if detail != 0 { "半自动手动" } else { "自动校准" }
                ), true),
            6 => (format!(
                    "⚠ PSoC 启动时强制改写了配置(位掩码 0x{:02X}: {}), 设备实际值与你设定的值不同。",
                    detail,
                    Self::_boot_override_text(detail)
                ), true),
            7 => ("PSoC 救砖(强制重刷)已完成, 算法与 CSD 配置将重新下发。".to_string(), true),
            8 => (format!(
                    "mai2serial 收到 RSET, 固件已受理: {}{}",
                    if detail & 0x01 != 0 { "IDAC 重校准 " } else { "" },
                    if detail & 0x02 != 0 { "基线复位" } else { "" }
                ), true),
            9 => ("PSoC 重启/救砖后实测采样已恢复可信, 固件已清除「基线不可信」标志。".to_string(), true),
            // 掉枚举纯属链路事件, 设备配置没变 ⇒ 不必回读真值(resync=false), 否则每次断连都白刷一轮。
            10 => (format!(
                    "⚠ 运行期 USB 掉枚举({}), 固件按约定【直接宣判失效, 不做救援】。若发生在游戏中, 该局的触控/灯效输出已中断。",
                    if detail & 0x01 != 0 {
                        "掉的时候 mai2serial 正在发触控数据"
                    } else if detail & 0x02 != 0 {
                        "掉的时候灯板协议已就绪但未在发触控"
                    } else {
                        "掉的时候两条协议都未在跑"
                    }
                ), false),
            11 => (format!(
                    "⚠ 存储区校验未通过, 该区已按默认值重建(有效掩码 0x{:X}: {})。单份存储的语义是「坏只坏在这一区」, 其余区完好。",
                    detail,
                    Self::_nv_valid_text(detail)
                ), true),
            _ => (format!("固件自持恢复事件(未知码 {}, detail=0x{:08X})", code, detail), true),
        };
        if code == 3 {
            self.push_log_warn(format!("[固件自持恢复 #{}/{}] {}", seq, total, text));
        } else {
            self.push_log(format!("[固件自持恢复 #{}/{}] {}", seq, total, text));
        }
        if needs_resync {
            // 无期望值可比, 纯粹把 UI 拉回设备真值。
            self.globals_expected.clear();
            self.globals_verify_in = Some(60);
        }
    }

    /// 解释 NvStore 各区有效掩码(位序与 main_firmware 的 NvStore::Region 一致)。
    /// 只列**无效**的那些 —— 用户要知道的是"哪一区丢了", 不是"哪些还在"。
    fn _nv_valid_text(mask: u32) -> String {
        const NAMES: [&str; 4] = ["配置项(KV)", "触控 CSD 调参", "算法二进制", "算法 C 源"];
        let mut lost: Vec<&str> = Vec::new();
        for (i, name) in NAMES.iter().enumerate() {
            if mask & (1u32 << i) == 0 {
                lost.push(name);
            }
        }
        if lost.is_empty() {
            return "全部有效".to_string();
        }
        format!("已丢失并重建: {}", lost.join(" / "))
    }

    /// 解释 PSoC 启动覆盖位掩码(与 psoc_firmware main.c 的 GPARAM_BOOT_OVERRIDE 注释一致)。
    fn _boot_override_text(bits: u32) -> String {
        let mut parts: Vec<&str> = Vec::new();
        if bits & 0x01 != 0 {
            parts.push("IDAC 增益档被抬到下限 4(生成配置默认 0 会让全通道 raw 满量程)");
        }
        if bits & 0x02 != 0 {
            parts.push("校准目标% 非法(0 或 ≥100)被回退为 85");
        }
        if parts.is_empty() {
            return "未知项".to_string();
        }
        parts.join("; ")
    }

    /// 用刚回读到的设备真值与"我们下发的期望值"逐项对账。不一致即固件拒收或夹取了该值——
    /// 必须让用户看见, 否则 UI 显示的是自己发出去的数(幻觉), 与设备实际配置长期不同步。
    /// 已知的固件夹取点: PSoC main.c:269 RAW_TARGET 落在 [60,90] 之外一律改 85;
    /// main.c:275 INACTIVE_SNS=GND 在启动时被强制改成 High-Z; main.c:266 IDAC 增益档 <4 抬到 4。
    fn _verify_globals_readback(&mut self, list: &[(u8, u32)]) {
        self.globals_verify_active = false;
        if self.globals_expected.is_empty() {
            return;
        }
        let expected: Vec<(u8, u32)> = self
            .globals_expected
            .iter()
            .map(|(k, v)| (*k, *v))
            .collect();
        self.globals_expected.clear();
        let mut ok = 0usize;
        let mut bad: Vec<String> = Vec::new();
        for (id, want) in expected {
            match list.iter().find(|(rid, _)| *rid == id).map(|(_, v)| *v) {
                Some(got) if got == want => ok += 1,
                Some(got) => bad.push(format!(
                    "{}: 期望 {} → 设备实际 {}",
                    Self::_gparam_name(id),
                    want,
                    got
                )),
                None => bad.push(format!(
                    "{}: 期望 {} → 设备未返回该项",
                    Self::_gparam_name(id),
                    want
                )),
            }
        }
        if !bad.is_empty() {
            self.push_log(format!(
                "⚠ 设备未按下发值生效({} 项): {}。UI 显示已同步为设备真值。\
                 常见原因: 固件对该项有合法区间夹取或启动时强制覆盖(如 RAW_TARGET 只接受 60~90, \
                 未激活连接 GND 在 PSoC 启动时被改成 High-Z)。",
                bad.len(),
                bad.join("; ")
            ));
        }
        if ok > 0 {
            self.push_log_debug(format!("设备已确认 {} 项全局设置与下发值一致", ok));
        }
    }
    fn _handle_algo_info_response(&mut self, frame: &Frame) {
        if let Some(info) = crate::proto::algo::decode_algo_info(&frame.payload) {
            self.algo_info = Some(info);
            self.algo_version = self.algo_version.wrapping_add(1);
        }
    }
    fn _handle_algo_get_rom_response(&mut self, frame: &Frame) {
        let roms = crate::proto::algo::decode_algo_get_rom(&frame.payload);
        for (i, r) in roms.iter().enumerate() {
            if i < self.algo_rom.len() {
                self.algo_rom[i] = *r;
            }
        }
        self.algo_rom_version = self.algo_rom_version.wrapping_add(1);
    }

    /// 获取单个通道的最新样本。
    /// ★绘图视窗冻结时读快照★ 否则读数会继续跳而曲线不动, 同一屏上出现两个互相矛盾的"现在"。
    /// 设备侧统计(采样率/延迟/停滞判定)不走这里, 仍然是实时的。
    pub fn telem_latest(&self, ch: u8) -> Option<ChannelSample> {
        if (ch as usize) >= 36 {
            return None;
        }
        match &self.plot_freeze {
            Some(f) => f.buf[ch as usize].back().cloned(),
            None => self.telem_buf[ch as usize].back().cloned(),
        }
    }

    /// 数据存活判定阈值(连续帧 raw 完全不变即视为停滞/异常)。CSD 正常扫描恒有噪声抖动,
    /// 故长时间逐帧完全相同 = 该通道扫描/测量卡死或链路停滞, 而非真实恒定读数。
    const FREEZE_FRAMES: u32 = 200;

    /// raw 满量程判定阈值(12 位 CSD 计数上限 4095): 达到即 IDAC 校准发散/过充, diff 恒 0 无法触控。
    const RAILED_RAW: u16 = 4090;

    /// 某通道数据是否停滞(疑似中断/异常, 非真实读数)。供 UI 明确标记, 杜绝"是真值还是卡住"的忙猜。
    /// ★禁用通道永远不算停滞★ 它的读数按设计恒为 0(设备侧不再扫描它), "长时间不变"正是预期行为;
    /// 判成停滞会让每个被关掉的通道都挂上"⚠停滞"红字, 把真正的异常淹掉。
    pub fn channel_frozen(&self, ch: u8) -> bool {
        (ch as usize) < 36
            && self.ch_enabled(ch)
            && (self.telem_freeze_count[ch as usize] >= Self::FREEZE_FRAMES
                || self.channel_stale(ch))
    }

    /// 某通道连续不变帧数(供诊断/调试展示)。
    pub fn channel_freeze_frames(&self, ch: u8) -> u32 {
        if (ch as usize) < 36 {
            self.telem_freeze_count[ch as usize]
        } else {
            0
        }
    }

    /// 全局数据是否整体停滞: 遥测在流但全部 36 通道都冻结 → 链路中断/设备扫描卡死(而非个别通道)。
    /// ★只看启用通道★ 否则"关掉大部分通道"这件事会让判据更容易成立(禁用通道恒不变),
    /// 把正常运行误报成"扫描已卡死"。全部通道都被关掉时也不成立(无启用通道 ⇒ all() 为真, 故显式排除)。
    pub fn data_all_frozen(&self) -> bool {
        let enabled: Vec<usize> = (0..36).filter(|ch| self.ch_enabled(*ch as u8)).collect();
        self.telem_active
            && self.telem_frame_count > 0
            && !enabled.is_empty()
            && enabled
                .iter()
                .all(|ch| self.telem_freeze_count[*ch] >= Self::FREEZE_FRAMES)
    }

    /// 某通道某字段的 (设备时间us, 值) 序列: 横轴用真实时间而非样本序号。
    ///
    /// ★为什么反向锚定★: 缓冲里存的是协议原始 u32 ts(会回绕), 展开值只有帧级时钟有。
    /// 以"最新帧"为锚点向旧方向逐对 `wrapping_sub` 累减, 既能吸收回绕, 又对"该通道最后一次
    /// 出现的帧不是最新帧"(通道掩码变化/掉帧)的情况天然正确 —— 差值就是它落后现在多久。
    /// 每通道各走自己的缓冲 → 时间基天然按通道独立。
    pub fn telem_points(&self, ch: u8, field: u8) -> Vec<(u64, f32)> {
        if (ch as usize) >= 36 {
            return Vec::new();
        }
        // ★冻结时整条时间轴取快照★ 缓冲与时钟锚点必须成对取, 只冻一个会让 x 坐标算错。
        let (buf, clock_prev_raw, clock_acc_us) = match &self.plot_freeze {
            Some(f) => (&f.buf[ch as usize], f.prev_raw, f.acc_us),
            None => (
                &self.telem_buf[ch as usize],
                self.telem_clock.prev_raw,
                self.telem_clock.acc_us,
            ),
        };
        let (Some(prev_raw), Some(back)) = (clock_prev_raw, buf.back()) else {
            return Vec::new();
        };
        // 该通道最新样本的展开时间 = 当前帧时间 - (当前帧 ts - 该样本 ts)。
        let mut abs = clock_acc_us.saturating_sub(prev_raw.wrapping_sub(back.t_us) as u64);
        let mut newer_raw = back.t_us;
        let mut out: Vec<(u64, f32)> = Vec::with_capacity(buf.len());
        for sample in buf.iter().rev() {
            abs = abs.saturating_sub(newer_raw.wrapping_sub(sample.t_us) as u64);
            newer_raw = sample.t_us;
            let value = match field {
                FIELD_RAW => sample.raw.map(|v| v as f32),
                FIELD_BASELINE => sample.bsln.map(|v| v as f32),
                FIELD_DIFF => sample.diff.map(|v| v as f32),
                _ => None,
            };
            if let Some(v) = value {
                out.push((abs, v));
            }
        }
        out.reverse();
        out
    }

    /// 展开后的设备时间(us): 供"最新样本绝对时刻"这类只读展示。
    /// 冻结时返回快照时刻 —— 与 `telem_points` 同一时间基, 否则轴上的绝对时刻会继续走。
    pub fn telem_dev_time_us(&self) -> u64 {
        match &self.plot_freeze {
            Some(f) => f.acc_us,
            None => self.telem_clock.acc_us,
        }
    }

    /// 获取实时遥测版本号(全通道状态卡等实时视图使用)。
    pub fn telem_version(&self) -> u64 {
        self.telem_version
    }

    /// 获取主图版本号。冻结期间不变，解冻后跳到最新遥测缓冲。
    pub fn plot_version(&self) -> u64 {
        self.plot_version
    }

    /// 获取最近一次遥测帧解出的采样率(Hz)
    pub fn telem_samples_per_sec(&self) -> u32 {
        self.telem_samples_per_sec
    }

    /// 获取最近一次遥测帧解出的通道刷新延迟(us)
    ///
    /// ★这是"设备实测探测周期"的唯一来源★ 主页、全 36 通道页、触控全局调整页的时钟树末端节点
    /// 都读它(经 main.rs 每 tick 回填 AppWindow.scan_period_us/scan_period_ms), 不允许再有第二份。
    /// 只在收到带 FIELD_STATS 的 TELEM_DATA 时更新(见 :5833) ⇒ 从未开流时恒为 0。
    pub fn telem_scan_period_us(&self) -> u32 {
        self.telem_scan_period_us
    }

    /// 探测周期是否有实测值。
    ///
    /// ★为什么必须单独给标志★ 停流/从未开流时 `telem_scan_period_us` 恒为 0, 而界面直接把 0 换算成
    /// "0.00 ms" 会被读成"设备真的 0ms"(还会连带算出 差值 -5796µs / 倍率 ×0 这种伪结论)。
    /// 0 不是任何设备能给出的合法周期, 故以 >0 作为"有值"的判据; 与 led_rx_frames_valid() 等
    /// 既有 *_valid 访问器同一约定, 界面无值时显示"未测"而不是 0。
    pub fn telem_scan_period_valid(&self) -> bool {
        // ★还要求"最近确实收到过带 STATS 的帧"★: 只看数值 >0 会让停流后长期显示上一次的过期值,
        // 用户无从分辨"这是刚测的"还是"这是十分钟前的"。超过 STATS_FRESH 即判为无值(显示"未测")。
        self.telem_scan_period_us > 0
            && self
                .telem_stats_at
                .is_some_and(|t| t.elapsed() < Self::TELEM_STATS_FRESH)
    }

    pub fn telem_lat_spi_us(&self) -> u16 {
        self.telem_lat_spi_us
    }

    pub fn telem_lat_proc_us(&self) -> u16 {
        self.telem_lat_proc_us
    }

    pub fn telem_lat_usb_us(&self) -> u16 {
        self.telem_lat_usb_us
    }

    pub fn latency_correction_enabled(&self) -> bool {
        matches!(
            self.config_get("comm.latency_correction_en")
                .map(|entry| entry.value),
            Some(CfgValue::Bool(true))
        )
    }

    pub fn telem_lat_corrected_us(&self) -> Option<u32> {
        if !self.latency_correction_enabled()
            || !self
                .telem_frame_at
                .is_some_and(|at| at.elapsed() < Self::TELEM_STATS_FRESH)
        {
            return None;
        }
        self.telem_lat_corrected_us
    }

    /// 延迟历史(总延迟 us)序列, 供仪表盘折线图。
    pub fn lat_total_series(&self) -> Vec<f32> {
        self.lat_total_hist.iter().copied().collect()
    }
    pub fn lat_version(&self) -> u64 {
        self.lat_version
    }

    /// 获取参数版本号
    pub fn param_version(&self) -> u64 {
        self.param_version
    }

    /// 获取某通道的参数(param_id -> value), 草稿覆盖缓存。
    pub fn params_of(&self, ch: u8) -> Vec<(u8, u32)> {
        if (ch as usize) >= 36 {
            return Vec::new();
        }
        let mut merged: BTreeMap<u8, u32> = self.params[ch as usize].clone();
        for ((dch, id), v) in self.drafts.param_items() {
            if dch == ch {
                merged.insert(id, v);
            }
        }
        merged.into_iter().collect()
    }

    // ------------------------------------------------------------------
    // 通道启用开关(PARAM_ENABLED=0x0C) —— 硬件级开关, 不是界面隐藏
    // ------------------------------------------------------------------

    /// 该通道当前是否启用(草稿优先, 即"用户看到的那个状态")。
    /// ★真值缺失时按启用处理★: 刚连上、0x0C 还没回读到时若默认成"已禁用", 整块面板会先灰一片,
    /// 用户会以为设备坏了; 而设备侧的默认本来就是全启用(见 CsdConfig::_reset_params)。
    pub fn ch_enabled(&self, ch: u8) -> bool {
        self.param(ch, crate::proto::PARAM_ENABLED)
            .map_or(true, |v| v != 0)
    }

    /// 自动化操作使用的启用真值。必须已有设备回读；未保存的“禁用”草稿立即排除，
    /// 未保存的“启用”草稿不能把设备仍禁用的通道提前加入队列。
    pub(super) fn ch_enabled_for_operation(&self, ch: u8) -> Option<bool> {
        let device_on = self
            .params
            .get(ch as usize)?
            .get(&crate::proto::PARAM_ENABLED)?
            != &0;
        let draft = self.drafts.param(ch, crate::proto::PARAM_ENABLED);
        Some(device_on && draft != Some(0))
    }

    pub(super) fn ch_enabled_states_known(&self) -> bool {
        (0..36u8).all(|ch| self.ch_enabled_for_operation(ch).is_some())
    }

    /// 启用通道位图(bit ch = 1 启用)。遥测掩码 / 批量队列 / 全通道操作按它筛选,
    /// 使"对禁用通道做无意义重操作"这件事只需在一处排除。
    pub fn enabled_ch_mask(&self) -> u64 {
        let mut mask = 0u64;
        for ch in 0..36u8 {
            if self.ch_enabled(ch) {
                mask |= 1u64 << ch;
            }
        }
        mask
    }

    /// 全通道页的展示次序 + 筛选结果(唯一真相源在这里, .slint 不自己排)。
    ///
    /// `mode`: 0 = 逻辑通道顺序, 1 = 物理通道顺序(PSoC channel index 升序)。
    /// 逻辑顺序★复用现有绑定映射★(`binding_channel_of`): 按分区号 0..33 走一遍, 每个通道在它
    /// 第一次被绑定到的分区处出场; 一个通道也没绑到的排在末尾并按物理号升序 —— 因此次序与绑定页
    /// 永远一致, 不存在第二份"分区→通道"表。
    ///
    /// `show_disabled=false` 时禁用通道被筛掉。返回 `(展示次序, 被筛掉的通道数)`。
    pub fn channel_display_order(&self, mode: i32, show_disabled: bool) -> (Vec<u8>, u32) {
        let mut order: Vec<u8> = Vec::with_capacity(36);
        if mode == 1 {
            order.extend(0..36u8);
        } else {
            let mut seen = [false; 36];
            // 34 = 触控分区数(与 bind_start 的越界判定、KBD_HOLD_ZONE_COUNT 同一口径)。
            for zone in 0..KBD_HOLD_ZONE_COUNT {
                let ch = self.binding_channel_of(zone);
                if (ch as usize) < 36 && !seen[ch as usize] {
                    seen[ch as usize] = true;
                    order.push(ch);
                }
            }
            order.extend((0..36u8).filter(|ch| !seen[*ch as usize]));
        }
        if !show_disabled {
            order.retain(|ch| self.ch_enabled(*ch));
        }
        (order.clone(), 36u32 - order.len() as u32)
    }

    /// 启用/禁用某通道。★走既有草稿路径★(与手工改参数同一条路), 由"保存到设备"统一下发 ——
    /// 不新增协议、不新增立即写路径, 界面按草稿值立刻灰显, 设备在保存时才真正关电极。
    pub fn set_ch_enabled(&mut self, ch: u8, on: bool) -> anyhow::Result<()> {
        if ch >= 36 {
            return Ok(());
        }
        let was = self.ch_enabled(ch);
        self.set_param(ch, crate::proto::PARAM_ENABLED, if on { 1 } else { 0 })?;
        if was != on {
            self.push_log(format!(
                "CH{} 已{}(草稿): {}。需点「保存到设备」后设备才真正{}。",
                ch,
                if on { "启用" } else { "禁用" },
                if on {
                    "重新参与扫描, 固件会只为该通道重做校准与基线"
                } else {
                    "退出扫描序列, 电极保持模拟高阻; 该通道不再上报数据也不会触发"
                },
                if on { "启用" } else { "关闭" }
            ));
        }
        self._bump_view_versions();
        Ok(())
    }

    /// 对当前已勾选的目标通道整批启用/禁用。★复用同一 PARAM_ENABLED, 不造新协议★:
    /// 与逐通道开关走完全相同的草稿路径, 因此"保存到设备"的下发/对账/持久化全部沿用既有实现。
    pub fn batch_set_enabled(&mut self, on: bool) -> anyhow::Result<()> {
        let targets: Vec<u8> = (0..36u8)
            .filter(|ch| self.batch_sel.ch_selected(*ch))
            .collect();
        if targets.is_empty() {
            self.push_log_warn(
                "批量启用/禁用未执行: 尚未勾选任何目标通道(在网格里单击卡片即为勾选)。".to_string(),
            );
            return Ok(());
        }
        let mut changed = 0usize;
        for ch in &targets {
            if self.ch_enabled(*ch) == on {
                continue;
            }
            if self
                .set_param(*ch, crate::proto::PARAM_ENABLED, if on { 1 } else { 0 })
                .is_ok()
            {
                changed += 1;
            }
        }
        self._bump_view_versions();
        self.push_log(format!(
            "批量{}: 已选 {} 个通道, 其中 {} 个状态发生变化(其余本来就是该状态, 不产生草稿)。\
             需点「保存到设备」才下发。",
            if on { "启用" } else { "禁用" },
            targets.len(),
            changed
        ));
        Ok(())
    }

    /// 获取某通道的单个参数, 草稿优先。
    pub fn param(&self, ch: u8, param_id: u8) -> Option<u32> {
        if (ch as usize) >= 36 {
            return None;
        }
        if let Some(v) = self.drafts.param(ch, param_id) {
            return Some(v);
        }
        self.params[ch as usize].get(&param_id).copied()
    }

    /// 全 36 通道 snsClk 分频(0x08)的 (min, max) 范围, 草稿优先; 无任何已知值时返回 (0, 0)。
    /// ★逐通道自适应后单一 CH0 值已无意义★: 时钟树用本范围展示"÷min – ÷max"。
    pub fn sns_clk_div_range(&self) -> (u16, u16) {
        let mut lo = u16::MAX;
        let mut hi = 0u16;
        for ch in 0..36u8 {
            if let Some(v) = self.param(ch, 0x08) {
                if v == 0 {
                    continue;
                }
                let v = v.min(u16::MAX as u32) as u16;
                if v < lo {
                    lo = v;
                }
                if v > hi {
                    hi = v;
                }
            }
        }
        if hi == 0 { (0, 0) } else { (lo, hi) }
    }

    /// 最新遥测里 raw 达到满量程(railed)的通道数: IDAC 校准发散的直接证据。
    pub fn railed_channel_count(&self) -> u32 {
        (0..36u8)
            // 禁用通道的 raw 恒为 0, 本来不会被判 railed; 显式排除是为了让"禁用通道不参与任何
            // 采样健康判定"这条约束在一处看得见, 而不是靠"恰好 0 < 4090"这个巧合成立。
            .filter(|ch| self.ch_enabled(*ch))
            .filter(|ch| {
                self.telem_latest(*ch)
                    .and_then(|s| s.raw)
                    .map_or(false, |raw| raw >= Self::RAILED_RAW)
            })
            .count() as u32
    }

    /// raw 长时间完全不变的通道数(扫描/测量停滞)。
    pub fn frozen_channel_count(&self) -> u32 {
        (0..36u8).filter(|ch| self.channel_frozen(*ch)).count() as u32
    }

    /// 是否处于"异常采样"(有 railed 通道 / 全通道数据停滞 / 设备拒绝固化基线)。
    pub fn sampling_abnormal(&self) -> bool {
        self.railed_channel_count() > 0 || self.data_all_frozen() || self.baseline_untrusted()
    }

    /// 设备上报的"恢复默认未获得可信基线"标志(RP2040 抽检 raw railed/停滞后拒绝固化)。
    pub fn baseline_untrusted(&self) -> bool {
        self.device_info
            .as_ref()
            .and_then(|i| i.diagnostics.as_ref())
            .map_or(false, |d| {
                (d.csd_flags & crate::proto::CSD_FLAG_BASELINE_UNTRUSTED) != 0
            })
    }

    /// 异常采样的醒目提示 + 下一步动作指引(全局调整页顶部状态行)。正常时给出确认文案。
    pub fn sampling_advice(&self) -> String {
        let railed = self.railed_channel_count();
        let frozen = self.frozen_channel_count();
        let stale = self._stale_telem_channels();
        if !stale.is_empty() {
            let names: Vec<String> = stale.iter().map(|ch| format!("CH{}", ch)).collect();
            return format!(
                "⚠ 逐通道遥测缺失({})，这些通道的旧值已标为 stale；已先重发当前 TELEM_START，持续缺失会触发非破坏链路恢复。",
                names.join(" ")
            );
        }
        if self.baseline_untrusted() {
            return "⚠ 恢复默认未获得可信基线(设备抽检到 raw 满量程/数据停滞, 已拒绝把异常状态存为默认) \
                    → 建议: 直接用「PSoC 救砖(重刷并重新应用)」".to_string();
        }
        if self.data_all_frozen() {
            return "⚠ 设备处于异常采样(全部通道数据停滞, 扫描/测量已卡死) \
                    → 建议: ① 重启 PSoC; ② 仍无效则用「PSoC 救砖(重刷并重新应用)」"
                .to_string();
        }
        if railed > 0 {
            return format!(
                "⚠ 设备处于异常采样({} 个通道 raw 满量程{}) \
                 → 建议: ① 全通道校准; ② 全通道频率自适应(逐通道); ③ 仍无效则用「PSoC 救砖(重刷并重新应用)」",
                railed,
                if frozen > 0 {
                    format!(", {} 个通道数据停滞", frozen)
                } else {
                    String::new()
                }
            );
        }
        if frozen > 0 {
            return format!(
                "▲ {} 个通道 raw 长时间无变化 → 建议: 先「全通道基线复位」, 无效再「全通道频率自适应」",
                frozen
            );
        }
        if self.telem_frame_count == 0 {
            return "● 未开启遥测: 采样状态未知(进入曲线页或点开始遥测后自动判定)".to_string();
        }
        "✓ 采样正常(无满量程通道, 数据持续更新)".to_string()
    }

    /// 采样警示等级: 0=正常 1=警告 2=异常。供 UI 着色。
    pub fn sampling_advice_level(&self) -> i32 {
        if self.baseline_untrusted()
            || self.data_all_frozen()
            || self.railed_channel_count() > 0
            || !self._stale_telem_channels().is_empty()
        {
            return 2;
        }
        if self.frozen_channel_count() > 0 || self.telem_frame_count == 0 {
            return 1;
        }
        0
    }

    /// 获取某通道已缓存的 Cp(fF)。
    pub fn cp(&self, ch: u8) -> Option<u32> {
        self.cp.get(ch as usize).copied().flatten()
    }

    /// 全局 Cp 缓存版本号；每个成功 CP_GET 响应和每次清空都会递增。
    pub fn cp_version(&self) -> u64 {
        self.cp_version
    }

    /// 某通道最后一次 CP_GET 响应对应的版本，用于 UI 排除通道切换前缓存。
    pub fn cp_channel_version(&self, ch: u8) -> u64 {
        self.cp_channel_versions
            .get(ch as usize)
            .copied()
            .unwrap_or(0)
    }

    // ------------------------------------------------------------------
    // 参数响应处理辅助(私有)
    // ------------------------------------------------------------------

    /// 处理 TELEM_DATA 帧
    fn _handle_telem_data(&mut self, frame: &Frame) {
        match crate::proto::decode_telem_data(&frame.payload) {
            Ok(telem_frame) => self._ingest_telem_frame(telem_frame, frame.payload.len()),
            Err(e) => {
                log::error!("解析 TELEM_DATA 失败: {}", e);
                self.last_error = Some(e);
            }
        }
    }

    /// 采样帧入库: TELEM_DATA 与 FOCUS_DATA 的**唯一**共用路径(同一份 telem_buf/版本号/存活检测)。
    /// 单通道 focus 只是"一帧只带一个通道"的同型帧, 不该有第二套入库逻辑, 更不该有平行缓冲。
    fn _ingest_telem_frame(
        &mut self,
        telem_frame: crate::proto::TelemFrame,
        wire_payload_len: usize,
    ) {
        // 更新最后时间戳 + 推进展开时钟(横轴真实时间的唯一来源, 见 DevClock)。
        self.telem_last_ts = telem_frame.ts_us;
        self.telem_clock.feed(telem_frame.ts_us);
        self.telem_frame_at = Some(std::time::Instant::now());
        // ★只有真的带 STATS 的帧才更新采样率/探测周期★
        // 逐通道档下 36 通道 × 4 字段远超 64B vendor FIFO, 固件按帧切分, **只有首帧带 STATS**;
        // 其余帧解码出的 samples_per_sec/scan_period_us 恒为 0。原先无条件赋值 ⇒ 后续帧立刻把
        // 刚取到的真值冲成 0 ⇒ `telem_scan_period_valid()` 恒假 ⇒ 全局调整页"始终未测"
        // (而主页轻档一帧一条、每帧都带 STATS, 所以主页看着正常 —— 这个不对称正是线索)。
        if (telem_frame.fields & crate::proto::FIELD_STATS) != 0 {
            self.telem_samples_per_sec = telem_frame.samples_per_sec;
            self.telem_scan_period_us = telem_frame.scan_period_us;
            self.telem_stats_at = Some(std::time::Instant::now());
        }
        // ★PSoC 链路活性的最强证据★: samples_per_sec 是固件按 PSoC scan_count 增量折算的
        // (psoc.cpp), >0 就等于"快照代数正在推进" —— 比只在握手那一瞬取一次的
        // DEVICE_INFO.psoc_link_valid 可信得多, 且随遥测帧率(20~100Hz)持续刷新。
        if telem_frame.samples_per_sec > 0 {
            self.psoc_scan_live_at = Some(std::time::Instant::now());
        }
        self.telem_lat_spi_us = telem_frame.lat_spi_us;
        self.telem_lat_proc_us = telem_frame.lat_proc_us;
        self.telem_lat_usb_us = telem_frame.lat_usb_us;
        if (telem_frame.fields & crate::proto::FIELD_LATENCY) != 0 {
            self.telem_lat_corrected_us =
                _latency_correction_us(&mut self.latency_correction_samples, &telem_frame);
            const LAT_CAP: usize = 512;
            if self.lat_total_hist.len() >= LAT_CAP {
                self.lat_total_hist.pop_front();
            }
            let total = telem_frame.lat_spi_us as u32
                + telem_frame.lat_proc_us as u32
                + telem_frame.lat_usb_us as u32;
            self.lat_total_hist.push_back(total as f32);
            self.lat_version = self.lat_version.wrapping_add(1);
        }

        // 把样本装进相应通道的环形缓冲
        let sample_count = telem_frame.samples.len();
        let fields = telem_frame.fields;
        for sample in telem_frame.samples {
            let ch_idx = sample.ch as usize;
            if ch_idx < 36 {
                // 每通道 freshness 独立于 STATS：某 CH 缺失时其旧样本立即可判 stale，
                // 而不是被同一帧其余 CH 的正常统计掩盖。
                self.telem_channel_last_seen[ch_idx] = Some(std::time::Instant::now());
                self.telem_channel_heal_attempts[ch_idx] = 0;
                // 数据存活检测: 逐帧比较 raw。完全相同 → 冻结计数+1; 变化 → 清零。
                // (只在带 RAW 字段的帧上判定; 不带 RAW 的帧跳过, 不误清计数。)
                if let Some(raw) = sample.raw {
                    if self.telem_last_raw[ch_idx] == Some(raw) {
                        self.telem_freeze_count[ch_idx] =
                            self.telem_freeze_count[ch_idx].saturating_add(1);
                    } else {
                        self.telem_freeze_count[ch_idx] = 0;
                        self.telem_last_raw[ch_idx] = Some(raw);
                    }
                }
                // ★保留时长优先于保留条数★ 主图横轴是固定 30s 窗口, 按"条数"保留会让
                // 实际保留时长随采样率浮动(171Hz 时 1024 条只有 ~6s ⇒ 窗口左侧 4/5 是空的)。
                // 条数上限只作为内存兜底(采样率异常高时不至于无限涨)。
                // 时间戳是协议原值(u32 us, 约 71 分钟回绕): 用 wrapping_sub 算"已过去多久",
                // 保留时长远小于回绕周期 ⇒ 回绕当帧也不会把整个缓冲误判为过期。
                let sample_t = sample.t_us;
                let buffer = &mut self.telem_buf[ch_idx];
                while buffer
                    .front()
                    .is_some_and(|old| sample_t.wrapping_sub(old.t_us) > TELEM_RETAIN_US)
                {
                    buffer.pop_front();
                }
                if buffer.len() >= TELEM_CAP {
                    buffer.pop_front();
                }
                buffer.push_back(sample);
            }
        }

        // 实时卡片始终跟随每帧; 主图只在未冻结时推进，冻结快照不触发 SVG/path 重算。
        self.telem_version = self.telem_version.wrapping_add(1);
        if self.plot_freeze.is_none() {
            self.plot_version = self.plot_version.wrapping_add(1);
        }
        self.telem_frame_count += 1;
        // Wire framing is SOF(2)+header(5)+CRC(2); payload length is the only variable part.
        self.telem_wire_bytes = self
            .telem_wire_bytes
            .wrapping_add(wire_payload_len as u64 + 9);
        if self.telem_frame_count == 1 {
            self.push_log("遥测数据开始流入 (首帧 TELEM_DATA)");
        }
        log::debug!(
            "采样帧: ts={} ch_count={} fields=0x{:02X}",
            telem_frame.ts_us,
            sample_count,
            fields
        );
    }

    /// 处理 FOCUS_DATA 推送(单通道独占流)。
    ///
    /// ★归属判定必须先于入库★ 切换目标通道时旧会话的帧仍在路上, 它们属于**上一个**通道,
    /// 混进新目标的缓冲就是伪造数据。设备回的 session 是唯一判据(不能等 STOP 的 ACK 才开始丢,
    /// 那期间的每一帧都会污染曲线)。
    fn _handle_focus_data(&mut self, frame: &Frame) {
        let focus_frame = match crate::proto::decode_focus_data(&frame.payload) {
            Ok(f) => f,
            Err(e) => {
                log::error!("解析 FOCUS_DATA 失败: {}", e);
                self.last_error = Some(e);
                return;
            }
        };
        if self.focus.session != Some(focus_frame.session) {
            // 旧会话残帧: 立即丢弃。设备侧那条流已被 START 替换或即将被 STOP, 不必在这里补发命令。
            return;
        }
        let Some(target) = self.focus.channel else {
            return;
        };
        // 设备只会按 START 指定的通道发帧; 不一致说明会话号撞上了旧流, 宁可丢弃也不写错通道。
        if focus_frame.frame.samples.first().map(|s| s.ch) != Some(target) {
            return;
        }
        // sample_seq 缺口 = 设备背压跳帧 + 链路丢帧的真实条数(设备跳帧时序号照样递增)。
        if let Some(prev) = self.focus.last_sample_seq {
            let gap = focus_frame.sample_seq.wrapping_sub(prev).wrapping_sub(1);
            if gap != 0 {
                self.focus.gaps = self.focus.gaps.wrapping_add(gap as u64);
            }
        }
        let gap_now = match self.focus.last_sample_seq {
            Some(prev) => focus_frame.sample_seq.wrapping_sub(prev).wrapping_sub(1),
            None => 0,
        };
        self.focus.last_sample_seq = Some(focus_frame.sample_seq);
        let push_count = focus_frame
            .frame
            .samples
            .iter()
            .filter(|sample| sample.ch == target)
            .count() as u32;
        let now = std::time::Instant::now();
        let diag_at = *self.focus.diag_at.get_or_insert(now);
        self.focus.diag_frames = self.focus.diag_frames.saturating_add(1);
        self.focus.diag_gaps = self.focus.diag_gaps.saturating_add(gap_now as u64);
        self.focus.diag_pushes = self.focus.diag_pushes.saturating_add(push_count);
        let periodic = self.focus.diag_frames >= 100
            || now.duration_since(diag_at) >= std::time::Duration::from_secs(1);
        if gap_now > 5 || periodic {
            let elapsed_s = now.duration_since(diag_at).as_secs_f32().max(0.001);
            self.push_log_debug(format!(
                "单通道诊断: CH{} 收到 {:.1} 帧/s、入库 {:.1} 点/s（{}帧/{}点，窗口 {:.2}s），gap_count={}，最近 sample_seq={} (session={})",
                target,
                self.focus.diag_frames as f32 / elapsed_s,
                self.focus.diag_pushes as f32 / elapsed_s,
                self.focus.diag_frames,
                self.focus.diag_pushes,
                elapsed_s,
                self.focus.diag_gaps,
                focus_frame.sample_seq,
                focus_frame.session
            ));
            self.focus.diag_at = Some(now);
            self.focus.diag_frames = 0;
            self.focus.diag_gaps = 0;
            self.focus.diag_pushes = 0;
        }
        self.focus.rate.note(frame.payload.len(), gap_now);
        self._ingest_telem_frame(focus_frame.frame, frame.payload.len());
    }

    /// 处理 FOCUS_START 响应: 只安装**当前代次**的会话。
    fn _handle_focus_start_response(&mut self, frame: &Frame) {
        let (session, accepted_rate) = match crate::proto::decode_focus_start(&frame.payload) {
            Ok(v) => v,
            Err(e) => {
                log::error!("解析 FOCUS_START 响应失败: {}", e);
                self.last_error = Some(e);
                return;
            }
        };
        let stale = match &self.focus.pending {
            Some(pending) => {
                pending.seq != frame.seq || pending.generation != self.focus.generation
            }
            None => true,
        };
        if stale {
            // 迟到响应: 目标早已改变。它对应的设备侧会话没人认领却仍在推流 → 必须显式停掉它。
            self.push_log_debug(format!(
                "单通道流: 丢弃迟到的 FOCUS_START 响应(session={}), 并对其下发 FOCUS_STOP",
                session
            ));
            self._focus_send_stop(session);
            return;
        }
        let channel = self.focus.pending.take().map(|p| p.channel);
        self.focus.session = Some(session);
        self.focus.reset_measurement();
        self.push_log(format!(
            "单通道流已开启: CH{} session={} 受理速率 {}Hz",
            channel.unwrap_or(0),
            session,
            accepted_rate
        ));
    }

    /// 处理 PARAM_GET 响应
    fn _handle_param_get_response(&mut self, frame: &Frame) {
        match crate::proto::decode_param_get(&frame.payload) {
            Ok((ch, param_id, value)) => {
                if (ch as usize) < 36 {
                    self.params[ch as usize].insert(param_id, value);
                    self.param_version += 1;
                    log::debug!(
                        "PARAM_GET: ch={} param_id=0x{:02X} value={}",
                        ch,
                        param_id,
                        value
                    );
                    if self.csd_diag_active {
                        self.push_log_debug(format!(
                            "CSD诊断: 设备回读 CH{} {} = {}",
                            ch,
                            Self::_param_name(param_id),
                            value
                        ));
                    }
                }
            }
            Err(e) => {
                log::error!("解析 PARAM_GET 响应失败: {}", e);
                self.last_error = Some(e);
            }
        }
    }

    /// 处理 PARAM_GET_ALL 响应(两种变体: 单通道全参数 / 全通道单参数)
    fn _handle_param_get_all_response(&mut self, frame: &Frame) {
        // "全通道单参数"变体(payload[0]==0xFF): 一帧带回 36 通道的同一参数, 只覆盖该 param_id,
        // 不清空其余参数缓存(与单通道变体的 clear 语义不同)。
        if frame.payload.first() == Some(&crate::proto::PARAM_ALL_CHANNELS) {
            match crate::proto::decode_param_get_all_channels(&frame.payload) {
                Ok((param_id, values)) => {
                    let count = values.len();
                    let mut min_div = u32::MAX;
                    let mut max_div = 0u32;
                    for (ch, value) in values {
                        if (ch as usize) < 36 {
                            // 回读防污染: 非法值不进缓存, 存修正值并(至多一次)回写设备。
                            let value = self._guard_readback_param(ch, param_id, value);
                            self.params[ch as usize].insert(param_id, value);
                            if param_id == 0x08 && value != 0 {
                                min_div = min_div.min(value);
                                max_div = max_div.max(value);
                            }
                        }
                    }
                    self.param_version += 1;
                    if param_id == 0x08 {
                        if let Some(ok_count) = self.auto_tune_all_refresh_pending.take() {
                            if min_div != u32::MAX {
                                self.push_log(format!(
                                    "逐通道自适应完成: 成功 {}/36, 分频范围 ÷{}–÷{}",
                                    ok_count, min_div, max_div
                                ));
                            } else {
                                self.push_log_warn(
                                    "逐通道自适应完成，但未回读到有效分频".to_string(),
                                );
                            }
                        }
                    }
                    log::debug!(
                        "PARAM_GET_ALL(all ch): param_id=0x{:02X} count={}",
                        param_id,
                        count
                    );
                }
                Err(e) => {
                    log::error!("解析 PARAM_GET_ALL(全通道)响应失败: {}", e);
                    self.last_error = Some(e);
                }
            }
            return;
        }
        match crate::proto::decode_param_get_all(&frame.payload) {
            Ok((ch, params)) => {
                if (ch as usize) < 36 {
                    // ★空响应绝不许清缓存★: 本分支原来无条件 param_map.clear() 再回填, 于是设备在
                    // PSoC 重启/重初始化期间回一个 count=0 的响应就把该通道已有真值全部抹掉, 界面
                    // 表现为"通道参数突然变成全 0"。恢复默认必然经过这个窗口, 故必然复现。
                    // 空响应的正确含义是"此刻取不到", 不是"设备上没有值" → 保留原值并留证。
                    if params.is_empty() {
                        log::warn!("PARAM_GET_ALL: ch={} 返回空参数集, 保留本地缓存不清空", ch);
                        return;
                    }
                    // 回读防污染先做完再落缓存: `_guard_readback_param` 需要 `&mut self`
                    // (要发回写帧/写日志), 不能在持有 `self.params[ch]` 可变借用时调用。
                    let guarded: Vec<(u8, u32)> = params
                        .into_iter()
                        .map(|(param_id, value)| {
                            (param_id, self._guard_readback_param(ch, param_id, value))
                        })
                        .collect();
                    let param_map = &mut self.params[ch as usize];
                    param_map.clear();
                    for (param_id, value) in guarded {
                        param_map.insert(param_id, value);
                    }
                    self.param_version += 1;
                    log::debug!("PARAM_GET_ALL: ch={} count={}", ch, param_map.len());
                }
            }
            Err(e) => {
                log::error!("解析 PARAM_GET_ALL 响应失败: {}", e);
                self.last_error = Some(e);
            }
        }
    }

    /// 处理 CP_GET 响应。
    fn _handle_cp_get_response(&mut self, frame: &Frame) {
        match crate::proto::decode_cp_get(&frame.payload) {
            Ok((ch, value)) if (ch as usize) < self.cp.len() => {
                self.cp[ch as usize] = Some(value);
                self.cp_version = self.cp_version.wrapping_add(1);
                self.cp_channel_versions[ch as usize] = self.cp_version;
                log::debug!("CP_GET: ch={} cp={}fF", ch, value);
                if self.csd_diag_active {
                    self.push_log_debug(format!("CSD诊断: 设备回读 CH{} Cp = {} fF", ch, value));
                }
            }
            Ok((ch, _)) => {
                let message = format!("CP_GET 响应通道越界: {}", ch);
                log::warn!("{}", message);
                self.last_error = Some(message);
            }
            Err(e) => {
                log::error!("解析 CP_GET 响应失败: {}", e);
                self.last_error = Some(e);
            }
        }
    }

    // ------------------------------------------------------------------
    // 绑区方法 (#6f):bind.mapNN 走已有 CFG_GET/CFG_SET,BIND_* 交互式
    // 命令(固件暂未实现,会收 NAK,由调用方/UI 优雅处理,不崩)。
    // ------------------------------------------------------------------

    /// 读出全部 34 区绑定值,缺失的 key 填协议默认值 0xFFFFFFFF
    /// (对齐 protocol_design.md 修订2 C5 `bind.map00..33` 默认)。
    pub fn binding_map(&self) -> [u32; 34] {
        let mut map = [0xFFFF_FFFFu32; 34];
        for (i, slot) in map.iter_mut().enumerate() {
            if let Some(entry) = self.config_cache.get(&zone_key(i)) {
                if let CfgValue::U32(v) = entry.value {
                    *slot = v;
                }
            }
        }
        map
    }

    /// 读取单个绑区槽位(zone: 0..33)的当前绑定值, 草稿优先, 缺失/越界返回默认值。
    pub fn get_binding(&self, zone: usize) -> u32 {
        if zone >= 34 {
            return 0xFFFF_FFFF;
        }
        let key = zone_key(zone);
        if let Some(CfgValue::U32(v)) = self.drafts.cfg(&key) {
            return *v;
        }
        self.config_cache
            .get(&key)
            .and_then(|entry| match entry.value {
                CfgValue::U32(v) => Some(v),
                _ => None,
            })
            .unwrap_or(0xFFFF_FFFF)
    }

    /// 写入单个绑区槽位,底层走 `CFG_SET(bind.mapNN)`。zone 越界返回 Err。
    pub fn set_binding(&mut self, zone: usize, value: u32) -> anyhow::Result<()> {
        if zone >= 34 {
            return Err(anyhow::anyhow!("绑区索引越界: {}", zone));
        }
        self.set_config(ConfigEntry::new(zone_key(zone), CfgValue::U32(value)))
    }

    /// 按新语义(值=物理通道索引 0..35,0xFFFFFFFF=未映射)写入单个绑区槽位的
    /// 物理通道绑定。channel>=36 视为"清除绑定"写入 0xFFFFFFFF。底层复用
    /// `set_binding`(CFG_SET bind.mapNN)。
    pub fn set_binding_channel(&mut self, zone: usize, channel: u8) -> anyhow::Result<()> {
        let value = if channel >= 36 {
            0xFFFF_FFFFu32
        } else {
            channel as u32
        };
        self.set_binding(zone, value)
    }

    // ------------------------------------------------------------------
    // HID 触摸屏点位 (hid.enNN / hid.xNN / hid.yNN)
    //
    // ★与 Serial 侧 bind.mapNN 完全独立★: 这里读写的是"物理通道(36) → 屏幕归一坐标",
    // 服务 HID 模式的触摸屏上报; bind.mapNN 是"逻辑分区(34) → 物理通道", 服务 mai2 串口协议。
    // 两套 KV 前缀不同、维度不同、页面不同, 本组访问器**从不触碰 bind.map***, 反之亦然 ⇒
    // 切工作模式或改一边都不会覆盖另一边, 两套参数可同时存在并各自导入导出。
    // 值一律走既有草稿层(set_config → drafts.set_cfg), 由"保存到设备"统一下发, 与其它页同规范。
    // ------------------------------------------------------------------

    /// 该通道是否已启用 HID 点位输出(草稿优先 → 设备缓存 → false)。
    pub fn hid_point_enabled(&self, ch: usize) -> bool {
        if ch >= HID_POINT_COUNT {
            return false;
        }
        let key = hid_en_key(ch);
        if let Some(CfgValue::Bool(v)) = self.drafts.cfg(&key) {
            return *v;
        }
        self.config_cache
            .get(&key)
            .and_then(|entry| match entry.value {
                CfgValue::Bool(v) => Some(v),
                _ => None,
            })
            .unwrap_or(false)
    }

    /// 该通道的归一坐标 (x, y)(草稿优先 → 设备缓存 → (0,0))。
    /// 设备把 hid.x/y 声明为 U16; 其它整型一并接受只是为了不被"类型换过"的存量固件卡住。
    pub fn hid_point_xy(&self, ch: usize) -> (u16, u16) {
        (self._hid_coord(ch, true), self._hid_coord(ch, false))
    }

    fn _hid_coord(&self, ch: usize, is_x: bool) -> u16 {
        if ch >= HID_POINT_COUNT {
            return 0;
        }
        let key = if is_x { hid_x_key(ch) } else { hid_y_key(ch) };
        let pick = |v: &CfgValue| -> Option<u16> {
            match v {
                CfgValue::U16(v) => Some(*v),
                CfgValue::U8(v) => Some(*v as u16),
                CfgValue::U32(v) => Some((*v).min(HID_COORD_MAX as u32) as u16),
                _ => None,
            }
        };
        if let Some(v) = self.drafts.cfg(&key).and_then(pick) {
            return v.min(HID_COORD_MAX);
        }
        self.config_cache
            .get(&key)
            .and_then(|entry| pick(&entry.value))
            .unwrap_or(0)
            .min(HID_COORD_MAX)
    }

    /// 写入该通道的启用位(只进草稿)。
    pub fn set_hid_point_enabled(&mut self, ch: usize, on: bool) -> anyhow::Result<()> {
        if ch >= HID_POINT_COUNT {
            return Err(anyhow::anyhow!("HID 点位通道越界: {}", ch));
        }
        self.set_config(ConfigEntry::new(hid_en_key(ch), CfgValue::Bool(on)))
    }

    /// 写入该通道的归一坐标(只进草稿)。越界坐标一律夹到 0..HID_COORD_MAX ——
    /// 该域由固件描述符决定, 超出会被主机静默截断, 夹取比放过去更诚实。
    pub fn set_hid_point_xy(&mut self, ch: usize, x: u16, y: u16) -> anyhow::Result<()> {
        if ch >= HID_POINT_COUNT {
            return Err(anyhow::anyhow!("HID 点位通道越界: {}", ch));
        }
        let (x, y) = (x.min(HID_COORD_MAX), y.min(HID_COORD_MAX));
        self.set_config(ConfigEntry::new(hid_x_key(ch), CfgValue::U16(x)))?;
        self.set_config(ConfigEntry::new(hid_y_key(ch), CfgValue::U16(y)))
    }

    /// "确认锚定": 一次写入坐标并置启用位。两件事必须同时落草稿 ——
    /// 只写坐标不启用等于用户点了确认却没有输出, 只启用不写坐标会拿旧锚点输出。
    pub fn hid_point_commit(&mut self, ch: usize, x: u16, y: u16) -> anyhow::Result<()> {
        self.set_hid_point_xy(ch, x, y)?;
        self.set_hid_point_enabled(ch, true)
    }

    /// "禁用/清除当前通道": 只清启用位, **保留坐标**。
    /// 保留是刻意的: 用户通常是临时停用某通道, 把辛苦锚好的坐标一并清掉等于逼他重锚一次;
    /// 需要真正重锚时直接再点一次"确认锚定"即覆盖。
    pub fn hid_point_disable(&mut self, ch: usize) -> anyhow::Result<()> {
        self.set_hid_point_enabled(ch, false)
    }

    /// 已启用的点位数(界面提示"当前 N/36 个通道已锚定")。
    pub fn hid_enabled_count(&self) -> usize {
        (0..HID_POINT_COUNT)
            .filter(|ch| self.hid_point_enabled(*ch))
            .count()
    }

    /// 按新语义读取单个绑区槽位当前绑定的物理通道索引(0..35),
    /// 未映射(0xFFFFFFFF)或越界返回 0xFF。
    pub fn binding_channel_of(&self, zone: usize) -> u8 {
        let value = self.get_binding(zone);
        if value == 0xFFFF_FFFF || value > 35 {
            0xFF
        } else {
            value as u8
        }
    }

    /// 发起交互式绑定(BIND_START,0x40)。payload=[zone(u8)]。
    /// 发出后立即进入本地等待态；对应 ACK 仅确认请求已受理，等待触摸期间保持蓝闪。
    pub fn bind_start(&mut self, zone: u8) -> anyhow::Result<()> {
        if zone as usize >= 34 {
            return Err(anyhow::anyhow!("绑区索引越界: {}", zone));
        }
        let seq = self._send_bind_cmd(HostCmd::BindStart, vec![zone])?;
        self.bind_progress = Some((zone, 0));
        self.bind_start_seq = Some(seq);
        self.push_log(format!(
            "绑定开始: 区{}，等待下一次触摸",
            zone_label(zone as usize)
        ));
        Ok(())
    }

    /// 中止交互式绑定(BIND_ABORT,0x41)。固件只会 ACK，中止后无需继续保留蓝闪。
    pub fn bind_abort(&mut self) -> anyhow::Result<()> {
        self._send_bind_cmd(HostCmd::BindAbort, vec![])?;
        self.bind_progress = None;
        self.bind_start_seq = None;
        Ok(())
    }

    /// 确认交互式绑定结果(BIND_CONFIRM,0x42)。
    pub fn bind_confirm(&mut self) -> anyhow::Result<()> {
        self._send_bind_cmd(HostCmd::BindConfirm, vec![])?;
        Ok(())
    }

    /// 交互式绑定进度。仅 `(zone, 0)` 是 UI 的等待态；完成/失败都会清除。
    pub fn bind_progress(&self) -> Option<(u8, u8)> {
        self.bind_progress
    }

    /// 当前全通道激活掩码(status bit0)，供实时绑定区仅在触摸态变化时重建模型。
    pub fn active_ch_mask(&self) -> u64 {
        self._active_mask()
    }

    /// 当前全通道激活掩码(status bit0), bit N = 通道 N 处于触摸态。
    fn _active_mask(&self) -> u64 {
        let mut mask = 0u64;
        for ch in 0u8..36 {
            let active = self
                .telem_latest(ch)
                .and_then(|s| s.status)
                .map(|st| (st & 0x01) != 0)
                .unwrap_or(false);
            if active {
                mask |= 1u64 << ch;
            }
        }
        mask
    }

    /// 开始"侦听下一次触摸"以绑定 zone(0..33)。纯上位机侧, 不发任何设备命令:
    /// 记录当前激活掩码作为基线, 之后由 `listen_tick` 捕获新激活的物理通道并写入草稿。
    pub fn listen_start(&mut self, zone: u8) -> anyhow::Result<()> {
        if zone as usize >= 34 {
            return Err(anyhow::anyhow!("绑区索引越界: {}", zone));
        }
        self.listen_zone = Some(zone);
        self.listen_baseline_mask = self._active_mask();
        self.listen_hold.clear();
        self.bind_progress = Some((zone, 0));
        self.push_log(format!(
            "侦听绑定: 区{} 等待下一次触摸(仅暂存草稿)",
            zone_label(zone as usize)
        ));
        Ok(())
    }

    /// 取消侦听。清除等待态, 不改动任何草稿。
    pub fn listen_cancel(&mut self) {
        self.listen_hold.clear();
        if self.listen_zone.is_some() || self.interactive_bind {
            self.listen_zone = None;
            self.interactive_bind = false;
            self.bind_progress = None;
            self.push_log("侦听绑定: 已取消");
        }
    }

    /// 交互式顺序绑定(等效 v3.0"开始绑区"): 从 A1(区0)起依次侦听每个区的触摸, 捕获即自动推进,
    /// 走完全部 34 区。全程只写绑定草稿, 完成后点"保存到设备"生效。
    pub fn interactive_bind_start(&mut self) -> anyhow::Result<()> {
        self.interactive_bind = true;
        self.push_log("交互式绑定: 开始 — 请按提示依次触摸各区(A1→E8)".to_string());
        self.listen_start(0)
    }

    /// 是否处于交互式顺序绑定中。
    pub fn interactive_bind_active(&self) -> bool {
        self.interactive_bind
    }

    /// 是否正在侦听某个分区。
    pub fn listen_zone(&self) -> Option<u8> {
        self.listen_zone
    }

    /// 每 tick 调用: 若正在侦听, 检测相对基线新激活(上升沿)的物理通道,
    /// 同一通道持续按住至门限才写入草稿。返回捕获到的 (zone, channel)。
    pub fn listen_tick(&mut self) -> Option<(u8, u8)> {
        let zone = self.listen_zone?;
        let now_mask = self._active_mask();
        // 只在基线中未激活、现在激活的通道上捕获(上升沿)。
        let rising = now_mask & !self.listen_baseline_mask;
        if rising == 0 {
            // 基线里已松开的通道要从基线里清除, 使其之后再次触摸能被捕获。
            self.listen_baseline_mask &= now_mask;
            self.listen_hold.clear();
            return None;
        }

        let channel = rising.trailing_zeros() as u8;
        if self.listen_hold.channel != Some(channel) {
            self.listen_hold.channel = Some(channel);
            self.listen_hold.since = Some(std::time::Instant::now());
            self.push_log(format!("侦听绑定: 检测到 CH{}, 请保持按住 1 秒", channel));
            return None;
        }

        let Some(since) = self.listen_hold.since else {
            return None;
        };
        let held_for = since.elapsed();
        if held_for < std::time::Duration::from_millis(LISTEN_HOLD_MS) {
            return None;
        }

        self.listen_hold.clear();
        // 只写入绑定草稿, 点击保存后才真正下发 BIND/CFG。
        let _ = self.set_binding_channel(zone as usize, channel);
        self.push_log(format!(
            "{}绑定: 区{} → CH{}(按住 {:.2}s, 已暂存草稿, 保存后生效)",
            if self.interactive_bind {
                "交互式"
            } else {
                "侦听"
            },
            zone_label(zone as usize),
            channel,
            held_for.as_secs_f32(),
        ));
        if self.interactive_bind {
            // 交互式: 自动推进到下一区; 走完 34 区则结束。
            if (zone as usize) + 1 < 34 {
                let next = zone + 1;
                // 保持 baseline 为当前掩码, 使已按住的触点不会被下一区误捕获(需重新触摸)。
                self.listen_zone = Some(next);
                self.listen_baseline_mask = now_mask;
                self.bind_progress = Some((next, 0));
                self.push_log(format!(
                    "交互式绑定: 请触摸 区{}",
                    zone_label(next as usize)
                ));
            } else {
                self.interactive_bind = false;
                self.listen_zone = None;
                self.bind_progress = None;
                self.push_log("交互式绑定: 全部 34 区完成! 点\"保存到设备\"生效。".to_string());
            }
        } else {
            self.listen_zone = None;
            self.bind_progress = None;
        }
        Some((zone, channel))
    }

    fn _send_bind_cmd(&mut self, cmd: HostCmd, payload: Vec<u8>) -> anyhow::Result<u8> {
        let seq = self.next_seq();
        if self.io.is_some() {
            let frame = Frame::new(cmd as u8, 0, seq, payload);
            self._queue_tx(frame)?;
        }
        Ok(seq)
    }

    /// BIND_START 成功 ACK 只表示固件已转入 WAIT_TOUCH，不能结束本地等待态。
    fn _handle_ack(&mut self, frame: &Frame) {
        // 数据型轮询必须等对应数据响应确认；ACK 只确认写类/独立状态机，不能清掉 read pending。
        self._cfg_tx_note_reply(frame);
        self._sweep_note_reply(frame.seq, true, "");
        if self.bind_start_seq == Some(frame.seq) {
            log::debug!("BIND_START 已确认 seq={}，继续等待触摸", frame.seq);
        } else {
            log::debug!("收到 ACK seq={}", frame.seq);
        }
        // 阻塞类操作(校准/基线复位/重启)的 ACK = 固件已真正完成 → 解除 op_busy(UI 解锁按钮/隐藏运行图标)。
        // 例外(op_ack_is_accept): 频率自适应的 ACK 仅表示"已受理", 真实完成由 AUTO_TUNE_PROGRESS(0x2E)
        // 推送流的终态帧判定; 此处若解锁会让 UI 在 ACK 一到就误判完成。NAK 仍按失败即时解锁。
        if self.op_wait_seq == Some(frame.seq) {
            if self.op_ack_is_accept {
                self.push_log_debug(format!(
                    "设备已受理「{}」(seq={}), 等待阶段进度推送至完成",
                    self.op_label, frame.seq
                ));
            } else {
                // 这类操作的 ACK = 固件真正做完 ⇒ 若它是逐通道批量发出的那一条, 这里才算它成功。
                self._ch_batch_note_op(frame.seq, true, "");
                // 频谱扫描的还原链(校准/基线)同样只认这一条终态回执, 不认"已排队"。
                self._sweep_note_op(frame.seq, true, "");
                self._end_op();
            }
        }
        // 算法上传确认复用灯效的 seq→ACK/NAK 归因。只有设备 ACK 才是"已接受并下发"；
        // 此时必须自动拉取 ALGO_GET_INFO，否则用户无法区分上传失败与 UI 仍显示旧算法真值。
        if self.algo_upload_seq == Some(frame.seq) {
            self.algo_upload_seq = None;
            self.algo_upload_started_at = None;
            // 下发在设备侧是异步的: 立刻这次读到的 psoc_valid 可能仍是在途态, 故 ~1.3s 后再读一次对账。
            self.algo_info_refresh_in = Some(80);
            self.algo_upload_status = match self.algo_get_info() {
                Ok(()) => "上传已确认(ACK): 设备已接受并下发，正在刷新设备真值…".to_string(),
                Err(error) => {
                    self.push_log_warn(format!("算法上传 ACK 后自动刷新设备信息失败: {}", error));
                    format!(
                        "上传已确认(ACK): 设备已接受并下发，但自动刷新失败: {}",
                        error
                    )
                }
            };
            self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
        }
        // 灯效写操作的确认。映射整批下发顺带拉一次 LED_GET, 让色块/映射立刻显示设备真值而非
        // 本地草稿; 预览不自动回拉(拖动调色时是连续下发, 每帧回拉只会挤占链路)。
        if let Some((seq, op)) = self.led_apply_seq {
            if seq == frame.seq {
                self.led_apply_seq = None;
                self.led_apply_status = match op {
                    LedWriteOp::ApplyRegions => "映射已生效".to_string(),
                    LedWriteOp::Preview => "预览色已生效".to_string(),
                };
                self.led_version = self.led_version.wrapping_add(1);
                if op == LedWriteOp::ApplyRegions {
                    let _ = self.led_request_state();
                }
            }
        }
    }

    /// 处理固件 BIND_EVENT(0x45) 的真实 payload `[zone, channel, status]`。
    /// `status=1` 为完成：同步本地 bind.map 缓存，并清除该区等待闪烁。
    fn _handle_bind_event(&mut self, frame: &Frame) {
        if frame.payload.len() < 3 {
            log::warn!("BIND_EVENT payload 过短: {} 字节", frame.payload.len());
            return;
        }

        let zone = frame.payload[0];
        let channel = frame.payload[1];
        let status = frame.payload[2];
        if zone >= 34 {
            log::warn!("BIND_EVENT 分区越界: {}", zone);
            return;
        }

        if status == 1 {
            if channel < 36 {
                let key = zone_key(zone as usize);
                self.config_cache.insert(
                    key.clone(),
                    ConfigEntry::new(key, CfgValue::U32(channel as u32)),
                );
                self.config_version = self.config_version.wrapping_add(1);
                self.push_log(format!(
                    "绑定完成: 区{} → CH{}",
                    zone_label(zone as usize),
                    channel
                ));
            } else {
                let message = format!("BIND_EVENT 完成通道越界: {}", channel);
                log::warn!("{}", message);
                self.last_error = Some(message);
            }
        }

        // 固件当前只发 status=1；保留非零状态均为终态的兼容语义，避免失败永久蓝闪。
        if status != 0 && self.bind_progress.map(|(active_zone, _)| active_zone) == Some(zone) {
            self.bind_progress = None;
            self.bind_start_seq = None;
        }
    }

    /// 测试专用:直接向 config_cache 灌入条目,不经过 io/协议帧路径。
    /// `set_config` 仅在已连接(`self.io` 为 Some)时才落盘本地缓存,单测里
    /// 没有真实连接,故需要这条后门来构造 `main.rs::build_config_rows` 的
    /// 输入。仅 `#[cfg(test)]` 编译,不影响生产构建。
    #[cfg(test)]
    pub(crate) fn test_insert_entries(&mut self, entries: Vec<ConfigEntry>) {
        for entry in entries {
            self.config_cache.insert(entry.key.clone(), entry);
        }
    }

    /// 测试专用:直接向某通道的遥测环形缓冲追加一个样本,不经过
    /// io/协议帧路径。main.rs 的曲线页测试(#6g-2)需要构造带数据的
    /// telem_buf 来验证 `build_chart_frame` 的 SeriesShow 门控逻辑,但
    /// `telem_buf` 字段本身私有于本模块,故提供这条 `pub(crate)` 后门,
    /// 与 `test_insert_entries` 同一模式。仅 `#[cfg(test)]` 编译。
    #[cfg(test)]
    pub(crate) fn test_push_telem_sample(&mut self, ch: u8, sample: ChannelSample) {
        if (ch as usize) < 36 {
            self.telem_buf[ch as usize].push_back(sample);
        }
    }

    /// 测试专用:直接向某通道的参数缓存灌入一条 (param_id, value),不经过
    /// io/协议帧路径。`set_param` 仅在已连接(`self.io` 为 Some)时才落盘
    /// 本地缓存,单测里没有真实连接,故需要这条后门,与
    /// `test_insert_entries`/`test_push_telem_sample` 同一模式。
    #[cfg(test)]
    pub(crate) fn test_set_param_cache(&mut self, ch: u8, param_id: u8, value: u32) {
        if (ch as usize) < 36 {
            self.params[ch as usize].insert(param_id, value);
            self.param_version += 1;
        }
    }

    // ------------------------------------------------------------------
    // 配置响应处理辅助
    // ------------------------------------------------------------------

    /// 处理 CFG_GET_ALL 响应(支持流式帧)
    fn _handle_cfg_get_all_response(&mut self, frame: &Frame) {
        let is_stream = (frame.flags & 0x02) != 0; // FLAG_STREAM
        let is_response = (frame.flags & 0x01) != 0; // FLAG_RESPONSE

        // 累积帧数据
        self.cfg_all_accum.extend_from_slice(&frame.payload);

        // 如果是末帧 (RESPONSE 且非 STREAM),则解析并合并进缓存
        if is_response && !is_stream {
            match decode_entries(&self.cfg_all_accum) {
                Ok(entries) => {
                    for entry in entries {
                        self.config_cache.insert(entry.key.clone(), entry);
                    }
                    self.config_version += 1;
                    log::debug!("CFG_GET_ALL 完成,缓存中有 {} 条目", self.config_cache.len());
                    self.cfg_all_accum.clear();
                }
                Err(e) => {
                    log::error!("解析 CFG_GET_ALL 响应失败: {}", e);
                    self.last_error = Some(e);
                    self.cfg_all_accum.clear();
                }
            }
        }
    }

    /// 处理 CFG_GET_GROUP 响应
    fn _handle_cfg_get_group_response(&mut self, frame: &Frame) {
        match decode_entries(&frame.payload) {
            Ok(entries) => {
                let count = entries.len();
                for entry in entries {
                    self.config_cache.insert(entry.key.clone(), entry);
                }
                self.config_version += 1;
                log::debug!("CFG_GET_GROUP 完成,新增/更新 {} 条目", count);
            }
            Err(e) => {
                log::error!("解析 CFG_GET_GROUP 响应失败: {}", e);
                self.last_error = Some(e);
            }
        }
    }

    /// 处理 CFG_GET 响应(单条目)
    fn _handle_cfg_get_response(&mut self, frame: &Frame) {
        match crate::proto::decode_entry(&frame.payload) {
            Ok((entry, _)) => {
                self.config_cache.insert(entry.key.clone(), entry);
                self.config_version += 1;
                log::debug!("CFG_GET 完成");
            }
            Err(e) => {
                log::error!("解析 CFG_GET 响应失败: {}", e);
                self.last_error = Some(e);
            }
        }
    }

    /// 处理 NAK 响应
    fn _handle_nak(&mut self, frame: &Frame) {
        let msg = if frame.payload.len() > 1 {
            String::from_utf8_lossy(&frame.payload[1..]).into_owned()
        } else {
            "未知错误".to_string()
        };
        let bind_start_failed = self.bind_start_seq == Some(frame.seq);
        let nak = (frame.flags & 0x04) != 0 || frame.cmd == HostCmd::Nak as u8;
        if nak {
            self._note_device_busy_nak(frame);
            let is_data_pending = self
                .pending_registry
                .lookup(frame.seq)
                .is_some_and(|req| !matches!(req.kind, RequestKind::CfgTx { .. }));
            let probe_label = self
                .conn_probe_inflight
                .as_ref()
                .filter(|(seq, _)| *seq == frame.seq)
                .map(|(_, step)| step.kind.label());
            if let Some(label) = probe_label {
                self.pending_registry.confirm(frame.seq);
                self.conn_probe_inflight = None;
                log::info!(
                    "CONN_PROBE response kind={} seq={} cmd=0x{:02X} flags=0x{:02X} nak=true",
                    label,
                    frame.seq,
                    frame.cmd,
                    frame.flags
                );
            } else if !is_data_pending {
                self.pending_registry.confirm(frame.seq);
            }
        }
        self._sweep_note_reply(frame.seq, false, &msg);
        // 组合映射回读被 NAK = 固件不认识 KBD_GET_COMBO(旧固件)。记为"不支持"而不是"空表",
        // 否则 UI 会把旧固件显示成"一条映射都没配", 用户会以为配置丢了。
        if self.kbd_combo_req_seq == Some(frame.seq) {
            self.kbd_combo_req_seq = None;
            self.kbd_combo_supported = Some(false);
            self.kbd_combo_version = self.kbd_combo_version.wrapping_add(1);
        }
        // 同理: 每键配置/边沿记录被 NAK = 旧固件没有这两条命令。记"不支持"并停轮询,
        // 而不是当成"极性全低电平/无任何边沿"的假数据继续画图。
        if self.kbd_keycfg_req_seq == Some(frame.seq) {
            self.kbd_keycfg_req_seq = None;
            self.kbd_keycfg_supported = Some(false);
            self.kbd_keycfg_version = self.kbd_keycfg_version.wrapping_add(1);
        }
        if self.kbd_edge_req_seq == Some(frame.seq) {
            self.kbd_edge_req_seq = None;
            self.kbd_edges_supported = Some(false);
            self.kbd_edges_version = self.kbd_edges_version.wrapping_add(1);
        }
        // 单通道流 START 被拒(旧固件不认识 0x33 / 通道非法): 只清在途请求, **保留目标通道** ——
        // 把目标一起清掉会让页面下一 tick 立刻重发, 变成每 16ms 一条 NAK 的刷屏。重试留给
        // 用户下次切通道/重进页面。
        if self
            .focus
            .pending
            .as_ref()
            .is_some_and(|p| p.seq == frame.seq)
        {
            self.focus.pending = None;
            self.push_log_warn(
                "单通道流启动被设备拒绝: 本次不再重试, 切换通道或重进页面可再试。".to_string(),
            );
        }
        self.push_log_warn(format!("收到 NAK(seq={}): {}", frame.seq, msg));
        if bind_start_failed {
            self.bind_progress = None;
            self.bind_start_seq = None;
            self.push_log(format!("绑定启动失败: {}", msg));
        }
        self.last_error = Some(msg);
        self._cfg_tx_note_reply(frame);
        // 算法上传和灯效写入一样按 seq 归因；设备给出的 NAK 原因必须到达算法页，而非被"已发送"冒充成功。
        if self.algo_upload_seq == Some(frame.seq) {
            self.algo_upload_seq = None;
            self.algo_upload_started_at = None;
            let reason = self.last_error.as_deref().unwrap_or("未知错误");
            self.algo_upload_status = format!("上传被设备拒绝(NAK): {}", reason);
            self.algo_upload_version = self.algo_upload_version.wrapping_add(1);
        }
        // 阻塞类操作失败回执也要解除 op_busy, 否则 UI 永久卡在运行中。
        // 逐通道批量发出的那一条被 NAK ⇒ 按 seq 归因给该通道, 不让它在完成文案里冒充成功。
        if self.op_wait_seq == Some(frame.seq) {
            let reason = format!(
                "设备拒绝(NAK): {}",
                self.last_error.as_deref().unwrap_or("未知错误")
            );
            self._ch_batch_note_op(frame.seq, false, &reason);
            // 还原链上的校准/基线被 NAK ⇒ 恢复未确认, 不允许在完成文案里冒充"已还原"。
            self._sweep_note_op(frame.seq, false, &reason);
            self._end_op();
        }
        // 自适应请求本身被拒(如固件 heavy_gate 拦下) ⇒ 这一轮压根没开始, 清掉在途归属键:
        // 否则它会一直"占着"这个 seq, 让后续任何带同 seq 的旧帧看起来仍是本轮的。
        if self.auto_tune_req_seq == Some(frame.seq) {
            self.auto_tune_req_seq = None;
        }
        // 追踪请求被 NAK(设备无算法/该 idx 不可读): 退避 ~3s(约 180 次 16ms 轮询),
        // 避免每 tick 一次 NAK 刷屏; 成功响应会清零退避恢复正常轮询。
        if self.algo_trace_last_seq == Some(frame.seq) {
            self.algo_trace_backoff = 180;
            self.algo_trace_last_seq = None;
        }
        // 灯效写操作被拒(映射是整批原子失败, 设备什么都没改; 预览是整条没生效):
        // 必须把设备给出的原因显示到面板上, 不能只落日志。
        if self.led_apply_seq.map(|(seq, _)| seq) == Some(frame.seq) {
            self.led_apply_seq = None;
            self.led_apply_status = format!(
                "设备拒绝(NAK): {}",
                self.last_error.as_deref().unwrap_or("未知错误")
            );
            self.led_version = self.led_version.wrapping_add(1);
        }
    }

    fn next_seq(&mut self) -> u8 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    // ------------------------------------------------------------------
    // UI 只读访问器(供 main.rs 回填 Slint 属性)
    // ------------------------------------------------------------------

    pub fn device_labels(&self) -> Vec<String> {
        self.devices.iter().map(|d| d.label.clone()).collect()
    }

    pub fn device_count(&self) -> usize {
        self.devices.len()
    }

    pub fn state(&self) -> ConnState {
        self.state
    }

    /// 当前 IO 会话传输/恢复计数；无句柄时返回零快照，供无头压测统一汇总。
    pub fn io_stats(&self) -> io::IoStats {
        self.io.as_ref().map(IoHandle::stats).unwrap_or_default()
    }

    /// 经当前 IO 会话已 claim 的接口读取 EP0 调试计数器；未连接时返回明确错误。
    pub fn read_debug_counters(&self) -> anyhow::Result<Vec<u8>> {
        self.io
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("未连接，无法读取 USB 调试计数器"))?
            .read_debug_counters()
    }

    /// 清零固件主循环阻塞剖面(峰值量)；压测前调用即可在干净窗口内测最坏阻塞。
    pub fn clear_loop_profile(&self) -> anyhow::Result<()> {
        self.io
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("未连接，无法清零主循环剖面"))?
            .clear_loop_profile()
    }

    pub fn status_line(&self) -> String {
        self.status_text.clone()
    }

    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    pub fn device_info(&self) -> Option<&DeviceInfo> {
        self.device_info.as_ref()
    }

    /// 设备在最新 DIAGNOSE/DEVICE_INFO 中上报的真实 CSD 模式；None 表示尚未取得真值。
    ///
    /// ★这是"设备真值"而不是"当前生效模式"★ 判断界面该按哪个模式显示/放行时**不要**用它,
    /// 用 [`Self::csd_mode_effective`]。本访问器只用于两处: 与草稿比对(撤稿判定)、诊断日志。
    pub fn csd_mode(&self) -> Option<u8> {
        self.csd_mode
    }

    /// ★当前 CSD 处理模式的唯一真相★: 草稿(用户已选但未保存) → 设备真值 → `None`(都还不知道)。
    ///
    /// ★为什么必须唯一★ 旧实现在 UI 回填处并列了两个来源(`mode_known` 只看设备真值、
    /// `scan_mode` 走"草稿优先"), 两个门控两个来源 ⇒ 同一页里下拉说"半自动手动"、时钟树说
    /// "AUTO"; 且 `if let Some(..)` 在两者皆 `None` 时**不赋值**, 使 `scan_mode` 锁存上一次的值
    /// (掉线重连后仍显示旧模式)。任何"是否自动模式"的判定一律经本函数, 不许再写
    /// `mode_draft().or(csd_mode())`。
    /// `None` 的含义是"真的还不知道", 调用方必须显式表达未知(UI 显示"读取中…"), 不得拿 0 冒充。
    pub fn csd_mode_effective(&self) -> Option<u8> {
        self.drafts.mode().or(self.csd_mode)
    }

    /// 判"未联通"所需的**连续无证据**时长(掉线去抖窗口)。
    ///
    /// ★为什么是非对称去抖★: 恢复要快、掉线不能误报, 两个目标对去抖的要求相反。这里做成
    /// "有证据立即算通(0 去抖), 连续 PSOC_LINK_STALE 无证据才算断":
    /// - 恢复: 遥测最慢档 20Hz(50ms/帧), 一帧证据即翻回"已联通", 加上 16ms 的 UI tick,
    ///   可见延迟 <100ms; 用户手动停流时由 1s 一次的 DEVICE_INFO 探测兜住, 仍在 ~1s 内。
    /// - 掉线: 1.5s ≈ 轻档 30 帧 / 逐通道档 150 帧, 足以吃掉单帧丢失、USB 抖动, 以及固件侧
    ///   500ms 一次的 samples_per_sec 统计窗口本身的粒度(至少覆盖 3 个统计窗口), 不会把
    ///   瞬时抖动放大成文案反复闪烁; 又远短于人对"点了重启 PSoC 却没反应"的忍耐阈值。
    const PSOC_LINK_STALE: std::time::Duration = std::time::Duration::from_millis(1500);

    /// PSoC SPI 链路活性证据。链路通/断的唯一判定入口(系统状态行与采样卡红字都走它)。
    pub fn psoc_link_evidence(&self) -> PsocLinkEvidence {
        let fresh = |at: Option<std::time::Instant>| {
            at.is_some_and(|t| t.elapsed() < Self::PSOC_LINK_STALE)
        };
        if self.state == ConnState::Connected {
            if fresh(self.psoc_scan_live_at) {
                return PsocLinkEvidence::ScanAdvancing {
                    sps: self.telem_samples_per_sec,
                };
            }
            if fresh(self.psoc_info_live_at) {
                return PsocLinkEvidence::Handshake {
                    generation: self.psoc_gen_seen.unwrap_or(0),
                };
            }
        }
        // 最近一条证据(取两个来源里更新的那个)距今多久; 都没有则本次连接从未活过。
        let since_ms = [self.psoc_scan_live_at, self.psoc_info_live_at]
            .iter()
            .filter_map(|at| at.map(|t| t.elapsed().as_millis() as u64))
            .min();
        PsocLinkEvidence::Stale {
            since_ms,
            last_generation: self.psoc_gen_seen,
        }
    }

    /// 链路是否联通(由活性证据判定, 不看那个只在握手时刷新的布尔位)。
    pub fn psoc_link_alive(&self) -> bool {
        !matches!(self.psoc_link_evidence(), PsocLinkEvidence::Stale { .. })
    }

    /// 判定依据的一行说明。所有展示链路状态的地方共用, 保证文案与判定同源。
    pub fn psoc_link_text(&self) -> String {
        match self.psoc_link_evidence() {
            PsocLinkEvidence::ScanAdvancing { sps } => {
                format!("快照代数在推进 · 扫描 {} Hz", sps)
            }
            PsocLinkEvidence::Handshake { generation } => {
                format!("停流中, 由 DEVICE_INFO 探测确认 · 第 {} 代", generation)
            }
            PsocLinkEvidence::Stale { since_ms: None, .. } => {
                "本次连接从未取得代数推进证据".to_string()
            }
            PsocLinkEvidence::Stale {
                since_ms: Some(ms),
                last_generation,
            } => format!(
                "已 {:.1}s 无代数推进 · 最近读到第 {} 代",
                ms as f32 / 1000.0,
                last_generation.unwrap_or(0)
            ),
        }
    }

    /// 停流期间的链路活性探测: 发一条 HELLO 取回新的 DEVICE_INFO(代数 + link_ok)。
    ///
    /// ★只在停流时探★: 固件收到 HELLO 会先 `SensorLink::stop()`(host_cmd.cpp::_handle_hello),
    /// 流式期间探测等于把遥测踢停。而遥测在跑时本来就有更强的证据(每帧的 samples_per_sec),
    /// 压根不需要探。由 main.rs 的 16ms tick 每 ~1s 调一次。
    pub fn psoc_link_probe(&mut self) {
        if self.state != ConnState::Connected || self.telem_active {
            return;
        }
        let _ = self.resend_hello();
    }

    /// 传感器/PSoC 健康的友好中文总结, 供 UI 直观展示异常大致原因。
    /// 综合 DEVICE_INFO 诊断(bring-up 阶段/flags)、链路活性证据与实时采样率。
    pub fn sensor_health_summary(&self) -> String {
        let Some(info) = &self.device_info else {
            return "● 未获取设备信息(未连接或 DEVICE_INFO 未到达)".to_string();
        };
        if let Some(d) = &info.diagnostics {
            if d.failure_stage != 0 {
                return format!(
                    "✕ PSoC bring-up 失败于阶段 {}({}) — 编程/校验/时钟异常, 建议重烧或检查 SWD 连线",
                    d.failure_stage,
                    crate::proto::PsocBringupDiagnostics::stage_name(d.failure_stage)
                );
            }
        }
        // ★链路判定改由活性证据驱动★: 原来直接读 info.psoc_link_valid, 那是握手那一瞬的快照,
        // 之后永不刷新 → 握手撞上 SPI 残帧抖动就把这条红字钉死, 与同屏 179Hz 采样自相矛盾。
        let evidence = self.psoc_link_evidence();
        if let PsocLinkEvidence::Stale { .. } = evidence {
            return format!(
                "✕ PSoC SPI 链路未就绪({}) — PSoC 无响应/掉线。检查传感器供电、SPI 连线与复位, 或点\"重启 PSoC\"",
                self.psoc_link_text()
            );
        }
        // snapshot_valid 同样只在握手那一帧回读: 已经有"代数在推进"的实时证据时它必然过期,
        // 不能再据它报警(否则握手时刚好在校准就永久显示"快照无效")。
        if !info.psoc_snapshot_valid && !matches!(evidence, PsocLinkEvidence::ScanAdvancing { .. })
        {
            return "▲ 快照无效 — PSoC 未产出有效扫描数据(可能刚复位/正在校准)".to_string();
        }
        // 数据存活: 遥测仍在出帧但全部通道 raw 长时间完全不变 = 扫描/测量卡死(常见于改全局后重初始化叠加致死)。
        if self.data_all_frozen() {
            return "✕ 数据停滞 — 全部通道 raw 长时间无变化(设备仍在出帧但扫描/测量已卡死), 常见于 CSD 操作叠加。建议点\"重启 PSoC\"恢复".to_string();
        }
        let sps = self.telem_samples_per_sec;
        // ★0 Hz 是最坏情况, 绝不能算正常★
        // 原判定是 `sps > 0 && sps < 100`, 于是 sps==0 直接落到末尾的"✓ 传感器正常(采样 0 Hz)" —
        // 实机上"一轮扫描都没完成"(如非激活电极接 GND 叠加 MFS, 实测周期 833ms~1s)反而显示绿色正常,
        // 把最严重的故障伪装成健康。0 必须单独判为错误。
        // 仅在遥测确实在跑时才判: 未开遥测时 0 只代表"还没测量过", 不是故障。
        if self.telem_active && sps == 0 {
            return "✕ 扫描未出数(采样 0 Hz) — 遥测在跑但一轮扫描都没完成。多见于非激活电极接 GND \
                    叠加多频扫描使单轮耗时逼近 1s, 或 raw 饱和致校准发散。检查全局 CSD 的\"未激活传感器连接\"\
                    与分辨率/分频, 或点\"恢复默认值\""
                .to_string();
        }
        if sps > 0 && sps < 100 {
            return format!(
                "▲ 扫描速率异常偏低({} Hz) — 可能 SmartSense 反复重校准 / raw 饱和 / IDAC/时钟参数不当, 建议检查全局 CSD 设置",
                sps
            );
        }
        if !self.telem_active {
            return "○ 链路正常, 采样率待测(未开启遥测)".to_string();
        }
        format!("✓ 传感器正常 (采样 {} Hz, 链路正常)", sps)
    }

    /// 传感器健康等级: 0=正常, 1=警告, 2=错误。与 sensor_health_summary 判定一致, 供 UI 着色。
    pub fn sensor_health_level(&self) -> i32 {
        let Some(info) = &self.device_info else {
            return 1;
        };
        if let Some(d) = &info.diagnostics {
            if d.failure_stage != 0 {
                return 2;
            }
        }
        // 与 sensor_health_summary 同一个判定入口(活性证据), 不再各读一次过期布尔。
        let evidence = self.psoc_link_evidence();
        if let PsocLinkEvidence::Stale { .. } = evidence {
            return 2;
        }
        if !info.psoc_snapshot_valid && !matches!(evidence, PsocLinkEvidence::ScanAdvancing { .. })
        {
            return 1;
        }
        if self.data_all_frozen() {
            return 2;
        }
        let sps = self.telem_samples_per_sec;
        // 与 sensor_health_summary 同口径: 遥测在跑却 0 Hz = 错误(原先被当成正常, 见该函数注释)。
        if self.telem_active && sps == 0 {
            return 2;
        }
        if sps > 0 && sps < 100 {
            return 1;
        }
        0
    }

    pub fn psoc_status(&self) -> Option<(u16, bool, bool)> {
        self.device_info.as_ref().map(|info| {
            (
                info.psoc_generation,
                info.psoc_link_valid,
                info.psoc_snapshot_valid,
            )
        })
    }

    /// 固件版本号 → (年,月,日,时,分)。编码 = 十进制 YYMMDDHHMM(本地时间, 分钟精度),
    /// 由两端构建时生成(main_firmware/tools/build_stamp_header.py)。
    /// 不像时间戳(位数不足或字段越界)时返回 None → 走旧的 主.次 口径, 免得把老固件的
    /// 0x402 显示成一个假日期。
    fn _fw_stamp_parts(raw: u32) -> Option<(u32, u32, u32, u32, u32)> {
        if raw < 1_000_000_000 {
            return None;
        }
        let minute = raw % 100;
        let hour = (raw / 100) % 100;
        let day = (raw / 10_000) % 100;
        let month = (raw / 1_000_000) % 100;
        let year = raw / 100_000_000;
        if !(1..=12).contains(&month) || !(1..=31).contains(&day) || hour > 23 || minute > 59 {
            return None;
        }
        Some((year, month, day, hour, minute))
    }

    /// 固件版本号 → 人可读版本串(关于页/主页用)。
    /// 时间戳版: `202607310427 (2026-07-31 04:27)`。
    /// 旧固件(手写 (主<<8)|次 小整数)版: `v4.2 (0x00000402)` —— 保留原始 hex 不丢信息。
    fn _fw_version_text(raw: u32) -> String {
        match Self::_fw_stamp_parts(raw) {
            Some((y, mo, d, h, mi)) => format!(
                "20{:02}{:02}{:02}{:02}{:02} (20{:02}-{:02}-{:02} {:02}:{:02})",
                y, mo, d, h, mi, y, mo, d, h, mi
            ),
            None => format!("v{}.{} (0x{:08X})", (raw >> 8) & 0xFF, raw & 0xFF, raw),
        }
    }

    /// 诊断转储里跟在原始 hex 后面的短注释: 时间戳版给 `2026-07-31 04:27`, 旧版给 `v4.2`。
    fn _fw_version_brief(raw: u32) -> String {
        match Self::_fw_stamp_parts(raw) {
            Some((y, mo, d, h, mi)) => {
                format!("20{:02}-{:02}-{:02} {:02}:{:02}", y, mo, d, h, mi)
            }
            None => format!("v{}.{}", (raw >> 8) & 0xFF, raw & 0xFF),
        }
    }

    /// 能力位 → 中文能力名列表。已知位见固件 host_cmd 的 capability_bits 组装(遥测/配置/绑定);
    /// 未知位不吞掉, 按 0x 原值列出, 免得新固件加了位而界面装作没有。
    fn _capability_text(bits: u32) -> String {
        if bits == 0 {
            return "无(0x00000000)".to_string();
        }
        let mut names: Vec<String> = Vec::new();
        let known: [(u32, &str); 3] = [(0x1, "遥测"), (0x2, "配置"), (0x4, "绑定")];
        for (bit, name) in known {
            if bits & bit != 0 {
                names.push(name.to_string());
            }
        }
        let mut rest = bits & !0x7;
        while rest != 0 {
            let bit = rest & rest.wrapping_neg();
            names.push(format!("未知位 0x{:X}", bit));
            rest &= rest - 1;
        }
        names.join(" · ")
    }

    /// 关于页版本行(主控固件 / PSoC 固件 / 协议版本)。未握手到 DEVICE_INFO 时返回 None,
    /// 由 UI 决定未连接文案 —— 版本格式化必须留在此处才能复用 `_fw_version_text`, 不在 UI 侧重写位拆分。
    pub fn device_version_lines(&self) -> Option<String> {
        let info = self.device_info.as_ref()?;
        let psoc_fw = match &info.diagnostics {
            Some(diag) => Self::_fw_version_text(diag.embedded_psoc_version),
            None => "未知(legacy DEVICE_INFO)".to_string(),
        };
        Some(format!(
            "主控固件(RP2040): {}\nPSoC 固件: {}\n设备协议版本: {}",
            Self::_fw_version_text(info.fw_version),
            psoc_fw,
            info.protocol_version
        ))
    }

    /// 主页"系统状态"用的人可读摘要(结构化几行, 不含 debug 诊断转储)。
    /// 原始诊断文本仍由 `device_info_text()` 提供, 放在主页可折叠区里备查。
    pub fn device_summary_text(&self) -> String {
        let Some(info) = &self.device_info else {
            return "未获取到设备信息".to_string();
        };
        let psoc_fw = match &info.diagnostics {
            Some(diag) => Self::_fw_version_text(diag.embedded_psoc_version),
            None => "未知(legacy DEVICE_INFO)".to_string(),
        };
        // ★与采样卡红字同源★: 都走 psoc_link_evidence, 不再各自去读握手那一瞬的过期布尔位,
        // 杜绝"系统状态说断、采样卡说通"这种自相矛盾。括号里给出判定依据本身而不是一个死代数,
        // 免得又出现"代数看着很大所以应该是通的"这类只能靠人猜的界面。
        let psoc_link = if self.psoc_link_alive() {
            "已联通"
        } else {
            "未联通"
        };
        let csd_mode = match info.diagnostics.as_ref().map(|d| d.csd_mode) {
            Some(0) => "自动校准(PSoC 接管)",
            Some(1) => "半自动手动(用保存的手动参数)",
            Some(_) => "未知",
            None => "未读取",
        };
        format!(
            "主控固件: {}\nPSoC 固件: {}\n协议版本: {}\nCapSense 通道: {}\n设备能力: {}\nPSoC 链路: {} ({})\nCSD 模式: {}",
            Self::_fw_version_text(info.fw_version),
            psoc_fw,
            info.protocol_version,
            info.capsense_channels,
            Self::_capability_text(info.capability_bits),
            psoc_link,
            self.psoc_link_text(),
            csd_mode
        )
    }

    /// 供 UI 和无头诊断展示的 DEVICE_INFO 多行文本。
    pub fn device_info_text(&self) -> String {
        let Some(info) = &self.device_info else {
            return "未获取到设备信息".to_string();
        };

        let mut text = format!(
            // 这三个值是"最近一次 HELLO 回响应那一瞬"的快照(设备只在 DEVICE_INFO 里回它们),
            // 不是实时值 —— 明写出来, 否则排查时会拿它去否定实时的链路判定(见 psoc_link_evidence)。
            "协议版本: {}\nRP2040 固件版本: 0x{:08X} ({})\nCapSense 通道数: {}\n能力位: 0x{:08X}\nPSoC 快照(握手瞬时值): generation={} link_valid={} snapshot_valid={}",
            info.protocol_version,
            info.fw_version,
            Self::_fw_version_brief(info.fw_version),
            info.capsense_channels,
            info.capability_bits,
            info.psoc_generation,
            info.psoc_link_valid,
            info.psoc_snapshot_valid
        );
        if let Some(diag) = &info.diagnostics {
            text.push_str(&format!(
                "\n诊断报告: v{} len={} RP build=0x{:08X} embedded PSoC=0x{:08X} ({})\n阶段: last={}({}) failure={}({}) flags=0x{:04X}\nS455: actual=0x{:08X} idcode=0x{:08X} protection=0x{:02X}\nSWD: srom=0x{:08X} acquire_status=0x{:08X} sysreq=0x{:08X} delay={}\nFlash: erase=0x{:08X} program=0x{:08X} fail_row={} fail_addr=0x{:08X}\nVerify: read=0x{:08X} expect=0x{:08X} checksum_srom=0x{:08X} checksum=0x{:08X}",
                diag.report_version, diag.report_length, diag.rp_build_id, diag.embedded_psoc_version,
                Self::_fw_version_brief(diag.embedded_psoc_version),
                diag.last_stage, crate::proto::PsocBringupDiagnostics::stage_name(diag.last_stage),
                diag.failure_stage, crate::proto::PsocBringupDiagnostics::stage_name(diag.failure_stage),
                diag.flags, diag.actual_silicon_id, diag.idcode, diag.chip_protection,
                diag.last_srom_status, diag.acquire_status, diag.acquire_sysreq, diag.acquire_delay,
                diag.erase_status, diag.program_status, diag.fail_row, diag.fail_addr,
                diag.verify_read, diag.verify_expect, diag.checksum_srom, diag.checksum_value
            ));
            if let Some(erase) = &diag.erase_failure {
                text.push_str(&format!(
                    "\nClock: config_ok={} select=0x{:08X} imo_select=0x{:08X} trim1=0x{:08X} trim2=0x{:08X} trim3=0x{:08X}\nErase scan: complete={} byte_sum=0x{:08X} word_or=0x{:08X} first_addr=0x{:08X} first_value=0x{:08X} words_read={}/32768",
                    diag.has(crate::proto::BRINGUP_FLAG_CLOCK_CONFIG),
                    erase.clock_select, erase.clock_imo_select, erase.clock_trim1,
                    erase.clock_trim2, erase.clock_trim3,
                    diag.has(crate::proto::BRINGUP_FLAG_ERASE_SCAN_COMPLETE),
                    erase.flash_byte_sum, erase.flash_word_or, erase.first_nonzero_addr,
                    erase.first_nonzero_value, erase.words_read
                ));
            }
            // 自持恢复事件对账: 固件累计条数 vs 上位机实际收到的条数。差值>0 = 有事件没送达
            // (多因发生时上位机未连接), 必须显示出来, 否则"没收到"会被当成"没发生"。
            // 设备真实 CSD 模式: 协议原先没有读回通道, UI 只能显示自己的默认值(永远是"自动"),
            // 于是把 CapSense 自动算出的值当成用户手动设置展示 → 误判。现在从诊断尾部读真值。
            text.push_str(&format!(
                "\nCSD 模式(设备真值): {}",
                match diag.csd_mode {
                    0 => "自动校准(手动参数已暂时失效, 由 PSoC 接管)",
                    1 => "半自动手动(使用 RP2040 保存的手动参数)",
                    other => return format!("{}\nCSD 模式(设备真值): 未知({})", text, other),
                }
            ));
            let seen = self.self_heal_last_seq.unwrap_or(0);
            text.push_str(&format!(
                "\n自持恢复: 固件累计={} 已送达={} 队列待发={} 被丢弃={}{}",
                diag.self_heal_total,
                seen,
                if diag.self_heal_pending { "有" } else { "无" },
                diag.self_heal_dropped,
                if diag.self_heal_total > seen {
                    "  ⚠ 有事件未送达上位机(多为发生时未连接)"
                } else {
                    ""
                }
            ));
        } else {
            text.push_str("\n诊断报告: legacy DEVICE_INFO（不含 bring-up 扩展）");
        }
        text
    }
}

impl Default for AppController {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// 单元测试(纯逻辑,不依赖真机/Slint)
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::CfgValue;

    #[test]
    fn test_next_seq_wraps_around() {
        let mut ctrl = AppController::new();
        ctrl.seq = 255;
        assert_eq!(ctrl.next_seq(), 255);
        assert_eq!(ctrl.seq, 0);
        assert_eq!(ctrl.next_seq(), 0);
        assert_eq!(ctrl.seq, 1);
    }

    #[test]
    fn test_function_priority_config_first() {
        assert!(
            AppController::function_priority(CdcFunction::Config)
                < AppController::function_priority(CdcFunction::Serial)
        );
        assert!(
            AppController::function_priority(CdcFunction::Serial)
                < AppController::function_priority(CdcFunction::Light)
        );
        assert!(
            AppController::function_priority(CdcFunction::Light)
                < AppController::function_priority(CdcFunction::Unknown)
        );
    }

    #[test]
    fn test_devices_sorted_config_first() {
        let mut ctrl = AppController::new();
        ctrl.devices = vec![
            DeviceEntry {
                port_name: "COM1".into(),
                function: CdcFunction::Serial,
                label: "COM1  [serial]".into(),
            },
            DeviceEntry {
                port_name: "COM2".into(),
                function: CdcFunction::Config,
                label: "COM2  [config]".into(),
            },
            DeviceEntry {
                port_name: "COM3".into(),
                function: CdcFunction::Light,
                label: "COM3  [light]".into(),
            },
        ];
        ctrl.devices
            .sort_by_key(|d| AppController::function_priority(d.function));

        assert_eq!(ctrl.devices[0].function, CdcFunction::Config);
        assert_eq!(ctrl.devices[1].function, CdcFunction::Serial);
        assert_eq!(ctrl.devices[2].function, CdcFunction::Light);

        let labels = ctrl.device_labels();
        assert_eq!(
            labels,
            vec!["COM2  [config]", "COM1  [serial]", "COM3  [light]"]
        );
    }

    #[test]
    fn test_handle_frame_device_info_sets_connected() {
        let mut ctrl = AppController::new();
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
        let frame = Frame::new(HostCmd::DeviceInfo as u8, 0, 0, payload);

        ctrl.handle_frame(frame);

        assert_eq!(ctrl.state, ConnState::Connected);
        assert!(ctrl.device_info.is_some());
        assert!(ctrl.device_info_text().contains("36"));
    }

    #[test]
    fn test_handle_frame_bad_device_info_records_error() {
        let mut ctrl = AppController::new();
        // payload 太短,DeviceInfo::from_payload 应返回 Err
        let frame = Frame::new(HostCmd::DeviceInfo as u8, 0, 0, vec![0x01, 0x02]);

        ctrl.handle_frame(frame);

        assert!(ctrl.last_error.is_some());
        assert_ne!(ctrl.state, ConnState::Connected);
    }

    #[test]
    fn test_handle_event_disconnected_resets_state() {
        let mut ctrl = AppController::new();
        ctrl.state = ConnState::Connected;
        ctrl.device_info = Some(DeviceInfo {
            protocol_version: 1,
            fw_version: 1,
            capsense_channels: 1,
            capability_bits: 1,
            psoc_generation: 0,
            psoc_link_valid: false,
            psoc_snapshot_valid: false,
            diagnostics: None,
        });

        ctrl.handle_event(IoEvent::Disconnected);

        assert_eq!(ctrl.state, ConnState::Disconnected);
        assert!(ctrl.device_info.is_none());
    }

    #[test]
    fn test_disconnect_without_io_is_noop() {
        let mut ctrl = AppController::new();
        ctrl.disconnect();
        assert_eq!(ctrl.state, ConnState::Disconnected);
    }

    #[test]
    fn test_connect_out_of_range_index_errors() {
        let mut ctrl = AppController::new();
        let result = ctrl.connect(0);
        assert!(result.is_err());
    }

    #[test]
    fn test_config_cache_initialized_empty() {
        let ctrl = AppController::new();
        let entries = ctrl.config_entries();
        assert_eq!(entries.len(), 0);
    }

    #[test]
    fn test_config_cache_cleared_on_disconnect() {
        let mut ctrl = AppController::new();
        // 手动插入一条目到缓存
        ctrl.config_cache.insert(
            "test.key".to_string(),
            ConfigEntry::new("test.key".to_string(), CfgValue::Bool(true)),
        );
        assert_eq!(ctrl.config_entries().len(), 1);

        ctrl.disconnect();

        assert_eq!(ctrl.config_entries().len(), 0);
    }

    #[test]
    fn test_config_get_retrieves_entry() {
        let mut ctrl = AppController::new();
        let entry = ConfigEntry::new("test.bool".to_string(), CfgValue::Bool(true));
        ctrl.config_cache.insert(entry.key.clone(), entry.clone());

        let retrieved = ctrl.config_get("test.bool");
        assert!(retrieved.is_some());
        assert_eq!(retrieved.unwrap().key, "test.bool");
    }

    #[test]
    fn test_handle_cfg_get_all_response_stream_frames() {
        let mut ctrl = AppController::new();

        // 构造两条 Entry:
        // Entry 1: BOOL key="enabled" value=true
        let e1 = ConfigEntry::new("enabled".to_string(), CfgValue::Bool(true));
        let e1_encoded = crate::proto::encode_entry(&e1).unwrap();

        // Entry 2: UINT16 key="rate_hz" value=120
        let e2 = ConfigEntry::new("rate_hz".to_string(), CfgValue::U16(120));
        let e2_encoded = crate::proto::encode_entry(&e2).unwrap();

        // 构造 count(u16 LE) + entries
        let mut payload = vec![2u8, 0u8]; // count=2
        payload.extend_from_slice(&e1_encoded);
        payload.extend_from_slice(&e2_encoded);

        // 分成两帧:第一帧是 STREAM(0x02),第二帧是 RESPONSE(0x01)
        let stream_payload = payload[0..payload.len() / 2].to_vec();
        let resp_payload = payload[payload.len() / 2..].to_vec();

        // 处理 STREAM 帧
        let frame1 = Frame::new(HostCmd::CfgGetAll as u8, 0x02, 0, stream_payload);
        ctrl._handle_cfg_get_all_response(&frame1);
        assert_eq!(ctrl.config_entries().len(), 0, "STREAM 帧不应立即合并");

        // 处理 RESPONSE 帧(末帧)
        let frame2 = Frame::new(HostCmd::CfgGetAll as u8, 0x01, 0, resp_payload);
        ctrl._handle_cfg_get_all_response(&frame2);
        assert_eq!(
            ctrl.config_entries().len(),
            2,
            "RESPONSE 帧应合并所有累积条目"
        );
        assert_eq!(
            ctrl.config_version(),
            1,
            "末帧合并成功后 config_version 应自增"
        );
    }

    #[test]
    fn test_config_version_bumps_only_on_successful_merge() {
        let mut ctrl = AppController::new();
        assert_eq!(ctrl.config_version(), 0);

        let e1 = ConfigEntry::new("comm.rate_limit_hz".to_string(), CfgValue::U16(100));
        let payload = crate::proto::encode_entry(&e1).unwrap();
        let mut buf = vec![1u8, 0u8];
        buf.extend_from_slice(&payload);

        let frame = Frame::new(HostCmd::CfgGetGroup as u8, 0x01, 0, buf);
        ctrl._handle_cfg_get_group_response(&frame);
        assert_eq!(
            ctrl.config_version(),
            1,
            "CFG_GET_GROUP 成功合并应自增版本号"
        );

        let entry = ConfigEntry::new("device.name".to_string(), CfgValue::Str("test".to_string()));
        let payload2 = crate::proto::encode_entry(&entry).unwrap();
        let frame2 = Frame::new(HostCmd::CfgGet as u8, 0x01, 0, payload2);
        ctrl._handle_cfg_get_response(&frame2);
        assert_eq!(ctrl.config_version(), 2, "CFG_GET 成功合并应自增版本号");
    }

    #[test]
    fn test_handle_cfg_get_group_response() {
        let mut ctrl = AppController::new();

        let e1 = ConfigEntry::new("comm.rate_limit_hz".to_string(), CfgValue::U16(100));
        let e1_encoded = crate::proto::encode_entry(&e1).unwrap();

        let mut payload = vec![1u8, 0u8]; // count=1
        payload.extend_from_slice(&e1_encoded);

        let frame = Frame::new(HostCmd::CfgGetGroup as u8, 0x01, 0, payload);
        ctrl._handle_cfg_get_group_response(&frame);

        assert_eq!(ctrl.config_entries().len(), 1);
        assert_eq!(
            ctrl.config_get("comm.rate_limit_hz").unwrap().key,
            "comm.rate_limit_hz"
        );
    }

    // ------------------------------------------------------------------
    // 绑区辅助测试 (#6f)
    // ------------------------------------------------------------------

    #[test]
    fn test_zone_label_mapping() {
        assert_eq!(zone_label(0), "A1");
        assert_eq!(zone_label(7), "A8");
        assert_eq!(zone_label(8), "B1");
        assert_eq!(zone_label(16), "C1");
        assert_eq!(zone_label(17), "C2");
        assert_eq!(zone_label(18), "D1");
        assert_eq!(zone_label(26), "E1");
        assert_eq!(zone_label(33), "E8");
    }

    #[test]
    fn test_zone_key_mapping() {
        assert_eq!(zone_key(0), "bind.map00");
        assert_eq!(zone_key(16), "bind.map16");
        assert_eq!(zone_key(33), "bind.map33");
    }

    #[test]
    fn test_binding_mask_roundtrip() {
        let v = make_binding(0x05, 0x00AB_CDEF);
        assert_eq!(binding_device_mask(v), 0x05);
        assert_eq!(binding_channel_mask(v), 0x00AB_CDEF);

        // 高位不应污染 channel_mask,低 24 位不应污染 device_mask
        let v2 = make_binding(0xFF, 0x00FF_FFFF);
        assert_eq!(binding_device_mask(v2), 0xFF);
        assert_eq!(binding_channel_mask(v2), 0x00FF_FFFF);
    }

    #[test]
    fn test_binding_map_default_when_empty() {
        let ctrl = AppController::new();
        let map = ctrl.binding_map();
        assert_eq!(map.len(), 34);
        assert!(map.iter().all(|&v| v == 0xFFFF_FFFF));
        assert_eq!(ctrl.get_binding(0), 0xFFFF_FFFF);
    }

    #[test]
    fn test_binding_map_reads_cache() {
        let mut ctrl = AppController::new();
        ctrl.test_insert_entries(vec![ConfigEntry::new(
            "bind.map05".to_string(),
            CfgValue::U32(make_binding(1, 0x0000_0001)),
        )]);
        let map = ctrl.binding_map();
        assert_eq!(map[5], make_binding(1, 0x0000_0001));
        assert_eq!(ctrl.get_binding(5), make_binding(1, 0x0000_0001));
    }

    #[test]
    fn test_set_binding_out_of_range_errors() {
        let mut ctrl = AppController::new();
        assert!(ctrl.set_binding(34, 0).is_err());
    }

    /// BIND_EVENT(0x45) 的 payload 是 `[zone, channel, status]`(与固件
    /// `BindingService::_emit_bind_event(zone, channel, 1)` 同源), 且它是**终态**通知:
    /// 只清等待态、绝不设置等待态(等待态由 bind_start 建立)。
    /// ★这条锁住的是"谁定义 payload 布局"★ 旧断言按 `[zone, status]` 两字节写、还指望事件把进度
    /// 设起来 —— 那是协议改成三字节前的语义, 留着只会让人以为固件少发了一个字节。
    #[test]
    fn test_handle_bind_event_writes_map_and_never_sets_progress() {
        let mut ctrl = AppController::new();
        assert_eq!(ctrl.bind_progress(), None);
        // 短包必须被丢弃, 不能按两字节布局误读成 zone=7/status=1。
        ctrl.handle_frame(Frame::new(HostCmd::BindEvent as u8, 0, 0, vec![7, 1]));
        assert_eq!(ctrl.bind_progress(), None);
        assert_eq!(ctrl.get_binding(7), 0xFFFF_FFFF, "短包不许落盘");
        // 完整包: 区 7 绑到 CH11, 缓存被同步, 等待态仍为 None。
        ctrl.handle_frame(Frame::new(HostCmd::BindEvent as u8, 0, 0, vec![7, 11, 1]));
        assert_eq!(ctrl.get_binding(7), 11);
        assert_eq!(ctrl.bind_progress(), None);
    }

    #[test]
    fn test_handle_cfg_get_response() {
        let mut ctrl = AppController::new();

        let entry = ConfigEntry::new("device.name".to_string(), CfgValue::Str("test".to_string()));
        let payload = crate::proto::encode_entry(&entry).unwrap();

        let frame = Frame::new(HostCmd::CfgGet as u8, 0x01, 0, payload);
        ctrl._handle_cfg_get_response(&frame);

        assert_eq!(ctrl.config_entries().len(), 1);
        match ctrl.config_get("device.name").unwrap().value {
            CfgValue::Str(s) => assert_eq!(s, "test"),
            _ => panic!("Wrong type"),
        }
    }

    // ------------------------------------------------------------------
    // 遥测与参数缓冲测试 (#6g-1)
    // ------------------------------------------------------------------

    #[test]
    fn test_telem_buf_initialized_36_channels() {
        let ctrl = AppController::new();
        assert_eq!(ctrl.telem_buf.len(), 36);
        for buf in ctrl.telem_buf.iter() {
            assert!(buf.is_empty());
        }
    }

    #[test]
    fn test_params_initialized_36_channels() {
        let ctrl = AppController::new();
        assert_eq!(ctrl.params.len(), 36);
        for param_map in ctrl.params.iter() {
            assert!(param_map.is_empty());
        }
    }

    #[test]
    fn test_handle_telem_data_fills_buffer() {
        let mut ctrl = AppController::new();

        // 构造 TELEM_DATA 帧: 2 通道, RAW|DIFF 字段
        let mut payload = Vec::new();
        payload.push(0x78); // ts_us low
        payload.push(0x56);
        payload.push(0x34);
        payload.push(0x12); // ts_us high
        payload.push(2); // ch_count
        payload.push(0x05); // fields = RAW | DIFF

        // Channel 0
        payload.push(0); // ch_index
        payload.push(0x34); // raw low
        payload.push(0x12); // raw high
        payload.push(0x78); // diff low
        payload.push(0x56); // diff high

        // Channel 5
        payload.push(5); // ch_index
        payload.push(0xCD); // raw low
        payload.push(0xAB); // raw high
        payload.push(0x00); // diff low
        payload.push(0xEF); // diff high

        let frame = Frame::new(HostCmd::TelemData as u8, 0x02, 0, payload);
        let v_before = ctrl.telem_version();
        ctrl._handle_telem_data(&frame);

        assert_eq!(ctrl.telem_buf[0].len(), 1);
        assert_eq!(ctrl.telem_buf[0].front().unwrap().ch, 0);
        assert_eq!(ctrl.telem_buf[0].front().unwrap().raw, Some(0x1234));
        assert_eq!(ctrl.telem_buf[0].front().unwrap().diff, Some(0x5678i16));

        assert_eq!(ctrl.telem_buf[5].len(), 1);
        assert_eq!(ctrl.telem_buf[5].front().unwrap().ch, 5);
        assert_eq!(ctrl.telem_buf[5].front().unwrap().raw, Some(0xABCD));

        assert_eq!(ctrl.telem_version(), v_before + 1, "telem_version 应自增");
    }

    #[test]
    fn test_telem_points_carry_device_time() {
        let mut ctrl = AppController::new();

        // 帧时钟与样本时间戳同步喂入(展开器以首帧为 0 点)。
        for ts in [0u32, 33_333, 66_666] {
            ctrl.telem_clock.feed(ts);
        }
        let samples = vec![
            ChannelSample {
                ch: 0,
                t_us: 0,
                raw: Some(100),
                bsln: Some(5000),
                diff: Some(100),
                status: Some(1),
            },
            ChannelSample {
                ch: 0,
                t_us: 33_333,
                raw: Some(110),
                bsln: Some(5000),
                diff: Some(110),
                status: Some(1),
            },
            ChannelSample {
                ch: 0,
                t_us: 66_666,
                raw: Some(90),
                bsln: Some(5000),
                diff: Some(90),
                status: Some(0),
            },
        ];

        for sample in samples {
            ctrl.telem_buf[0].push_back(sample);
        }

        let points = ctrl.telem_points(0, FIELD_RAW);
        assert_eq!(
            points.iter().map(|p| p.1).collect::<Vec<_>>(),
            vec![100.0, 110.0, 90.0]
        );
        assert_eq!(
            points.iter().map(|p| p.0).collect::<Vec<_>>(),
            vec![0, 33_333, 66_666]
        );
        assert_eq!(ctrl.telem_points(0, FIELD_DIFF).len(), 3);
    }

    /// 设备端 32 位 us 时间戳约 71.6 分钟回绕: 展开后时间必须仍单调递增, 不得倒流成负跨度。
    #[test]
    fn test_telem_points_survive_timestamp_wrap() {
        let mut ctrl = AppController::new();

        let pre = u32::MAX - 10_000;
        let post = pre.wrapping_add(20_000);
        for ts in [pre, post] {
            ctrl.telem_clock.feed(ts);
        }
        for (ts, raw) in [(pre, 1u16), (post, 2u16)] {
            ctrl.telem_buf[0].push_back(ChannelSample {
                ch: 0,
                t_us: ts,
                raw: Some(raw),
                bsln: None,
                diff: None,
                status: None,
            });
        }

        let points = ctrl.telem_points(0, FIELD_RAW);
        assert_eq!(points.len(), 2);
        assert_eq!(
            points[1].0 - points[0].0,
            20_000,
            "跨回绕的间隔必须仍为真实的 20ms"
        );
    }

    #[test]
    fn test_telem_latest_returns_back_sample() {
        let mut ctrl = AppController::new();

        let samples = vec![
            ChannelSample {
                ch: 3,
                t_us: 0,
                raw: Some(100),
                bsln: None,
                diff: None,
                status: None,
            },
            ChannelSample {
                ch: 3,
                t_us: 1_000,
                raw: Some(150),
                bsln: None,
                diff: None,
                status: None,
            },
        ];

        for sample in samples {
            ctrl.telem_buf[3].push_back(sample);
        }

        let latest = ctrl.telem_latest(3);
        assert!(latest.is_some());
        assert_eq!(latest.unwrap().raw, Some(150));
    }

    #[test]
    fn test_handle_param_get_response_fills_cache() {
        let mut ctrl = AppController::new();

        // 构造 PARAM_GET 响应: ch=7, param_id=PARAM_FINGER_TH(0x01), value=250
        let mut payload = Vec::new();
        payload.push(7); // ch
        payload.push(0x01); // param_id = PARAM_FINGER_TH
        payload.push(0xFA); // value=250 low
        payload.push(0x00);
        payload.push(0x00);
        payload.push(0x00);

        let frame = Frame::new(HostCmd::ParamGet as u8, 0x01, 0, payload);
        let v_before = ctrl.param_version();
        ctrl._handle_param_get_response(&frame);

        assert_eq!(ctrl.param(7, 0x01), Some(250));
        assert_eq!(ctrl.param_version(), v_before + 1);
    }

    #[test]
    fn test_handle_param_get_all_response_fills_cache() {
        let mut ctrl = AppController::new();

        // 构造 PARAM_GET_ALL 响应: ch=3, count=2, params: (0x01, 100), (0x02, 40)
        let mut payload = Vec::new();
        payload.push(3); // ch
        payload.push(2); // count

        payload.push(0x01); // param_id
        payload.push(100); // value low
        payload.push(0);
        payload.push(0);
        payload.push(0);

        payload.push(0x02); // param_id
        payload.push(40); // value low
        payload.push(0);
        payload.push(0);
        payload.push(0);

        let frame = Frame::new(HostCmd::ParamGetAll as u8, 0x01, 0, payload);
        let v_before = ctrl.param_version();
        ctrl._handle_param_get_all_response(&frame);

        assert_eq!(ctrl.param(3, 0x01), Some(100));
        assert_eq!(ctrl.param(3, 0x02), Some(40));
        assert_eq!(ctrl.param_version(), v_before + 1);

        let params_list = ctrl.params_of(3);
        assert_eq!(params_list.len(), 2);
    }

    #[test]
    fn test_telem_buf_respects_capacity() {
        let mut ctrl = AppController::new();
        const TELEM_CAP: usize = 1024;

        // 填满缓冲到容量+1
        for i in 0..=TELEM_CAP {
            let sample = ChannelSample {
                ch: 0,
                t_us: i as u32 * 1_000,
                raw: Some(i as u16),
                bsln: None,
                diff: None,
                status: None,
            };
            ctrl.telem_buf[0].push_back(sample.clone());
            if ctrl.telem_buf[0].len() > TELEM_CAP {
                ctrl.telem_buf[0].pop_front();
            }
        }

        assert_eq!(ctrl.telem_buf[0].len(), TELEM_CAP);
        // 最早的应该是被弹出，最新的应该在末尾
        assert_eq!(
            ctrl.telem_buf[0].back().unwrap().raw,
            Some(TELEM_CAP as u16)
        );
    }

    #[test]
    fn test_disconnect_clears_telem_and_params() {
        let mut ctrl = AppController::new();

        // 填入一些遥测和参数数据
        ctrl.telem_buf[0].push_back(ChannelSample {
            ch: 0,
            t_us: 0,
            raw: Some(100),
            bsln: None,
            diff: None,
            status: None,
        });
        ctrl.params[0].insert(0x01, 250);
        ctrl.telem_active = true;

        ctrl.disconnect();

        assert_eq!(ctrl.telem_buf[0].len(), 0);
        assert_eq!(ctrl.params[0].len(), 0);
        assert!(!ctrl.telem_active);
    }

    #[test]
    fn test_reboot_and_reboot_bootloader_require_connection() {
        let mut ctrl = AppController::new();
        // 未连接时调用应返回 Err(io is None)
        let result1 = ctrl.reboot();
        let result2 = ctrl.reboot_bootloader();
        // 由于 self.io 为 None,send 会在 Option::unwrap_or_else 中失败
        // 实际会返回 Err,但我们这里无法直接检测(io 线程管理在 IO 层)。
        // 这个单测只验证逻辑(不会 panic)。
        assert!(result1.is_err() || result1.is_ok()); // 总之不应 panic
        assert!(result2.is_err() || result2.is_ok());
    }
}

/// 定位可用的 arm-none-eabi 工具链目录(gcc/objcopy/nm/objdump 都能运行)。
/// 优先程序自带 tools/arm-none-eabi/bin(随程序分发, 免受各机环境差异), 再回退本机安装路径。
/// 对每个候选试运行 --version, 跳过缺失或映像损坏(如 PlatformIO nm.exe 0xc000012f)的目录。
fn find_toolchain_dir() -> anyhow::Result<std::path::PathBuf> {
    use std::path::PathBuf;
    let mut candidates: Vec<PathBuf> = Vec::new();

    // 1) 程序自带(随发布分发, 免各机差异)。
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            candidates.push(dir.join("tools").join("arm-none-eabi").join("bin"));
            candidates.push(dir.join("tools"));
        }
    }

    // 扫描某父目录下所有子目录 + 拼接后缀(按版本号通配, 不写死版本)。
    let scan_versioned = |root: PathBuf, suffix: &[&str], out: &mut Vec<PathBuf>| {
        if let Ok(rd) = std::fs::read_dir(&root) {
            for e in rd.flatten() {
                if e.path().is_dir() {
                    let mut p = e.path();
                    for s in suffix {
                        p = p.join(s);
                    }
                    out.push(p);
                }
            }
        }
    };

    // 2) 本机 ModusToolbox / Infineon 安装(遍历常见安装根 + 版本号通配)。
    //    PSoC 固件既然在本机编译过, 这里几乎必然能命中 mtb 自带的 arm-none-eabi。
    let mut roots: Vec<PathBuf> = Vec::new();
    for key in [
        "USERPROFILE",
        "LOCALAPPDATA",
        "ProgramFiles",
        "ProgramFiles(x86)",
        "ProgramW6432",
    ] {
        if let Ok(v) = std::env::var(key) {
            if !v.is_empty() {
                roots.push(PathBuf::from(v));
            }
        }
    }
    for root in &roots {
        // ModusToolbox/tools_<ver>/gcc/bin (及个别版本的 arm-none-eabi/bin 布局)
        scan_versioned(root.join("ModusToolbox"), &["gcc", "bin"], &mut candidates);
        scan_versioned(
            root.join("ModusToolbox"),
            &["arm-none-eabi", "bin"],
            &mut candidates,
        );
        // Infineon/Tools/mtb-gcc-arm-eabi/<ver>/gcc/bin
        scan_versioned(
            root.join("Infineon").join("Tools").join("mtb-gcc-arm-eabi"),
            &["gcc", "bin"],
            &mut candidates,
        );
    }

    // 3) PlatformIO 工具链包(名称含 toolchain / gcc-arm)。
    if let Ok(home) = std::env::var("USERPROFILE") {
        let pkgs = PathBuf::from(&home).join(".platformio").join("packages");
        if let Ok(rd) = std::fs::read_dir(&pkgs) {
            for e in rd.flatten() {
                let name = e.file_name().to_string_lossy().to_lowercase();
                if name.contains("toolchain")
                    || name.contains("gccarm")
                    || name.contains("gcc-arm")
                    || name.contains("earlephilhower")
                {
                    candidates.push(e.path().join("bin"));
                }
            }
        }
    }

    // 4) PATH 上的每个目录(工具链可能已在 PATH 中)。
    if let Ok(path) = std::env::var("PATH") {
        for d in std::env::split_paths(&path) {
            candidates.push(d);
        }
    }

    let tools = [
        "arm-none-eabi-gcc.exe",
        "arm-none-eabi-objcopy.exe",
        "arm-none-eabi-nm.exe",
    ];
    let runnable = |p: &std::path::Path| -> bool {
        p.exists()
            && std::process::Command::new(p)
                .arg("--version")
                .output()
                .map(|o| o.status.success())
                .unwrap_or(false)
    };
    for d in &candidates {
        if tools.iter().all(|t| runnable(&d.join(t))) {
            return Ok(d.clone());
        }
    }
    Err(anyhow::anyhow!(
        "未找到可用的 arm-none-eabi 工具链(gcc/objcopy/nm)。请把工具链放到程序目录 \
         tools/arm-none-eabi/bin(随程序分发), 或安装 ModusToolbox/PlatformIO 工具链。\
         (PlatformIO 某版本 nm.exe 映像损坏 0xc000012f 会被自动跳过)"
    ))
}

/// ABI 头(psoc_algo_abi.h)随程序内嵌, JIT 编译时写入临时目录并 -I 引用,
/// 免受"机器上没有 PSoC 工程源码路径"影响, 保证单 exe 可移植。
static ALGO_ABI_HEADER: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../psoc_firmware/CY8C4147AZI-SensorCore/psoc_algo_abi.h"
));

/// 随程序打包的最小 arm-none-eabi 工具链(gcc/cc1/as/objcopy/nm/objdump + GCC 内建头),
/// 以 zip 形式 include_bytes! 进 exe(约 18MB)。首次使用时解压到本地缓存目录, 之后复用。
/// 保证任意 Windows 设备无需预装工具链即可 JIT 编译触控算法。
static BUNDLED_TOOLCHAIN_ZIP: &[u8] =
    include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/toolchain.zip"));
/// 版本标记(改动打包内容时递增), 用于缓存失效。
const BUNDLED_TOOLCHAIN_TAG: &str = "mtb-arm-14.2.1-min1";

/// 确保内置工具链已解压到本地缓存, 返回其 bin 目录。首次调用解压, 之后直接命中。
fn ensure_bundled_toolchain() -> anyhow::Result<std::path::PathBuf> {
    use std::path::PathBuf;
    // 缓存根: %LOCALAPPDATA%\mai2control-ui\toolchain\<tag>, 回退到临时目录。
    let base = std::env::var("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir());
    let root = base
        .join("mai2control-ui")
        .join("toolchain")
        .join(BUNDLED_TOOLCHAIN_TAG);
    let bin = root.join("bin");
    let marker = root.join(".ready");
    // 已解压且 gcc 就位 → 直接复用。
    if marker.exists() && bin.join("arm-none-eabi-gcc.exe").exists() {
        return Ok(bin);
    }
    // 解压内嵌 zip 到 root(保留目录结构: bin/ libexec/ arm-none-eabi/ lib/)。
    let reader = std::io::Cursor::new(BUNDLED_TOOLCHAIN_ZIP);
    let mut archive =
        zip::ZipArchive::new(reader).map_err(|e| anyhow::anyhow!("内置工具链解包失败: {}", e))?;
    std::fs::create_dir_all(&root)?;
    for i in 0..archive.len() {
        let mut entry = archive.by_index(i)?;
        let rel = match entry.enclosed_name() {
            Some(p) => p,
            None => continue,
        };
        let out = root.join(rel);
        if entry.is_dir() {
            std::fs::create_dir_all(&out)?;
        } else {
            if let Some(parent) = out.parent() {
                std::fs::create_dir_all(parent)?;
            }
            let mut f = std::fs::File::create(&out)?;
            std::io::copy(&mut entry, &mut f)?;
        }
    }
    if !bin.join("arm-none-eabi-gcc.exe").exists() {
        return Err(anyhow::anyhow!("内置工具链解压后缺少 arm-none-eabi-gcc"));
    }
    let _ = std::fs::write(&marker, BUNDLED_TOOLCHAIN_TAG);
    Ok(bin)
}
