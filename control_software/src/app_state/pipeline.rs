//! 统一请求流水线：seq 归因、发送调度、背压感知、声明式轮询规则。
//!
//! 职责：
//! - 统一 seq 归因注册表（替代散落的 12+ 个独立字段）
//! - 统一发送入口（单一出口，背压检查，连接状态检查）
//! - 声明式"页面可见性 → 周期任务"规则表（替代 main.rs 里的散落 AND 链）
//!
//! 本模块不负责"写类命令"（CfgSet/ParamSet 等）——那些仍由既有 cfg_tx_queue
//! 处理（保留其批量 owner/generation 等既有正确设计），但它们的 seq 也会登记进
//! 统一注册表供 _handle_ack/_handle_nak 统一查询。

use crate::proto::Frame;
use std::collections::HashMap;
use std::time::Instant;

/// 在途请求的唯一归因记录（seq → 请求元信息）。
#[derive(Clone)]
pub struct PendingRequest {
    /// 请求发起时刻（用于超时检测）
    pub started_at: Instant,
    /// 超时策略（秒）
    pub timeout_secs: u32,
    /// 请求类别（用于日志/调试/回调分发）
    pub kind: RequestKind,
}

/// 请求类别枚举，涵盖当前 12 种归因场景 + 新增的统一轮询类。
///
/// **迁移说明**：
/// - 已迁移到统一表：`algo_get_info`/`request_algo_trace` 以及所有当前"直发"的只读轮询
///   （ping/request_params/global_get_all/kbd_*/mai2_request_state/led_request_state 等）
/// - 尚未迁移（仍由原专属字段归因，因其与领域状态机强耦合）：
///   - cfg_tx_inflight.seq（写类命令队列，保留其 owner/generation 语义）
///   - noise_sweep.pending.seq + control_seqs（频谱扫描会话，复杂的重试/续租逻辑）
///   - focus.pending.seq（单通道流会话，generation 丢弃旧帧）
///   - ch_batch 相关 seq（批量操作队列，逐通道归因+完成统计）
///   - bind_start_seq（绑区会话，等待触摸边沿）
/// - 后续渐进式迁移：新加的请求必须用统一表；既有复杂状态机可在重构其领域逻辑时顺便迁移。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RequestKind {
    // 原"直发"类（已迁移进统一流水线）
    Ping,
    RequestParams {
        ch: u8,
    },
    RequestParamAllChannels {
        param_id: u8,
    },
    ConfigGetAll,
    GlobalGetAll,
    KbdRequestState,
    KbdRequestMap,
    KbdRequestTouchmap,
    KbdRequestHold,
    KbdRequestEdges,
    KbdRequestKeycfg,
    KbdGetCombo,
    Mai2RequestState,
    LedRequestState,
    AlgoGetInfo,
    AlgoGetRom,
    RequestAlgoSrc,
    RequestAlgoCode,
    RequestAlgoTrace {
        ch: u8,
        idx: u8,
    },
    BusXfer,

    /// 连接成功后的顺序回读探针（响应按 cmd 分发，ACK/NAK 仅用于统一归因）。
    ConnProbe {
        name: &'static str,
    },
    // 写类命令（seq 登记进统一表，但实际归因逻辑仍复用 cfg_tx_inflight）
    CfgTx {
        cmd: u8,
    },
    // 尚未迁移的复杂状态机（占位，供统一查询入口识别"不在此表"）
    // （实际不会出现在 PendingRequest 里，只是为了让 _handle_ack 有统一入口）
}

impl RequestKind {
    /// 请求类别的可读标签（用于日志）
    pub fn label(&self) -> String {
        match self {
            Self::Ping => "PING".to_string(),
            Self::RequestParams { ch } => format!("PARAM_GET(ch={})", ch),
            Self::RequestParamAllChannels { param_id } => {
                format!("PARAM_GET_ALL(0x{:02X})", param_id)
            }
            Self::ConfigGetAll => "CFG_GET_ALL".to_string(),
            Self::GlobalGetAll => "GLOBAL_GET_ALL".to_string(),
            Self::KbdRequestState => "KBD_GET_STATE".to_string(),
            Self::KbdRequestMap => "KBD_GET_MAP".to_string(),
            Self::KbdRequestTouchmap => "KBD_GET_TOUCHMAP".to_string(),
            Self::KbdRequestHold => "KBD_GET_HOLD".to_string(),
            Self::KbdRequestEdges => "KBD_GET_EDGES".to_string(),
            Self::KbdRequestKeycfg => "KBD_GET_KEYCFG".to_string(),
            Self::KbdGetCombo => "KBD_GET_COMBO".to_string(),
            Self::Mai2RequestState => "MAI2_GET_STATE".to_string(),
            Self::LedRequestState => "LED_GET".to_string(),
            Self::AlgoGetInfo => "ALGO_GET_INFO".to_string(),
            Self::AlgoGetRom => "ALGO_GET_ROM".to_string(),
            Self::RequestAlgoSrc => "ALGO_GET_SRC".to_string(),
            Self::RequestAlgoCode => "ALGO_GET_CODE".to_string(),
            Self::RequestAlgoTrace { ch, idx } => format!("ALGO_GET_TRACE(ch={}, idx={})", ch, idx),
            Self::BusXfer => "BUS_XFER".to_string(),
            Self::ConnProbe { name } => format!("连接探针({})", name),
            Self::CfgTx { cmd } => format!("CFG_TX(cmd=0x{:02X})", cmd),
        }
    }
}

/// 统一 seq 归因注册表
pub struct PendingRegistry {
    map: HashMap<u8, PendingRequest>,
}

impl PendingRegistry {
    pub fn new() -> Self {
        Self {
            map: HashMap::new(),
        }
    }

    /// 注册一个在途请求
    pub fn register(&mut self, seq: u8, req: PendingRequest) {
        self.map.insert(seq, req);
    }

    /// 查询一个 seq 是否在途（返回请求元信息）
    pub fn lookup(&self, seq: u8) -> Option<&PendingRequest> {
        self.map.get(&seq)
    }

    /// 确认一个 seq（收到 ACK/NAK 后注销）
    pub fn confirm(&mut self, seq: u8) -> Option<PendingRequest> {
        self.map.remove(&seq)
    }

    /// 是否已有同类请求在途，轮询任务用它实现窗口=1。
    pub fn contains_kind(&self, kind: &RequestKind) -> bool {
        self.map.values().any(|req| &req.kind == kind)
    }

    /// 超时检查（返回所有超时的 seq）
    pub fn check_timeouts(&self, now: Instant) -> Vec<u8> {
        self.map
            .iter()
            .filter_map(|(seq, req)| {
                if now.duration_since(req.started_at).as_secs() > req.timeout_secs as u64 {
                    Some(*seq)
                } else {
                    None
                }
            })
            .collect()
    }

    /// 清空所有在途请求（断连时调用）
    pub fn clear(&mut self) {
        self.map.clear();
    }
}

/// 轮询上下文快照（从 main.rs 每 tick 构造并传入，避免直接读 Slint Weak 造成循环依赖）
#[derive(Clone, Debug)]
pub struct PollContext {
    pub connected: bool,
    pub current_view: i32,
    pub settings_tab: i32,
    pub hid_mode: bool,
    pub sel_channel: u8,
    pub light_panel_expanded: bool,
    pub phys_la_expanded: bool,
    /// 算法追踪轮询的 report idx；None 表示当前没有声明可读的上报项。
    pub algo_trace_idx: Option<u8>,
}

/// 轮询规则：声明式"页面可见性 → 周期任务"映射
pub struct PollRule {
    /// 规则名（用于日志/调试）
    pub name: &'static str,
    /// 可见性判定（输入上下文快照，返回是否应该执行）
    pub condition: Box<dyn Fn(&PollContext) -> bool>,
    /// 周期（毫秒）；0 = 每 tick 都执行（只要条件满足）
    pub period_ms: u32,
    /// 要提交的任务构造器（根据上下文构造 RequestKind + 要发送的帧）
    pub task: Box<dyn Fn(&PollContext) -> (RequestKind, Frame)>,
    /// 上一次执行的墙钟 tick（用于周期判定）
    last_tick: u32,
}

impl PollRule {
    /// 判定本 tick 是否应该执行（条件满足 + 周期到点）
    pub fn should_run(&mut self, ctx: &PollContext, now_tick: u32, tick_ms: u32) -> bool {
        if !(self.condition)(ctx) {
            return false;
        }
        if self.period_ms == 0 {
            return true; // 每 tick 都执行
        }
        let period_ticks = if tick_ms == 0 {
            1
        } else {
            self.period_ms
                .saturating_add(tick_ms.saturating_sub(1))
                .checked_div(tick_ms)
                .unwrap_or(1)
                .max(1)
        };
        if now_tick.wrapping_sub(self.last_tick) >= period_ticks {
            self.last_tick = now_tick;
            true
        } else {
            false
        }
    }
}

/// 轮询规则表（由 AppController 持有并在每 tick 里评估）
pub struct PollRules {
    rules: Vec<PollRule>,
}

impl PollRules {
    pub fn new() -> Self {
        Self { rules: Vec::new() }
    }

    /// 添加一条规则
    pub fn add(
        &mut self,
        name: &'static str,
        condition: Box<dyn Fn(&PollContext) -> bool>,
        period_ms: u32,
        task: Box<dyn Fn(&PollContext) -> (RequestKind, Frame)>,
    ) {
        self.rules.push(PollRule {
            name,
            condition,
            period_ms,
            task,
            last_tick: 0,
        });
    }

    /// 每 tick 评估所有规则，返回需要提交的任务列表
    pub fn evaluate(
        &mut self,
        ctx: &PollContext,
        now_tick: u32,
        tick_ms: u32,
    ) -> Vec<(&'static str, RequestKind, Frame, u8)> {
        let mut tasks = Vec::new();
        for rule in &mut self.rules {
            if rule.should_run(ctx, now_tick, tick_ms) {
                let (kind, frame) = (rule.task)(ctx);
                // seq 由调用方统一分配，避免规则表持有 AppController 的可变引用。
                tasks.push((rule.name, kind, frame, 0u8));
            }
        }
        tasks
    }

    /// 清空所有规则（用于测试或重建规则表）
    pub fn clear(&mut self) {
        self.rules.clear();
    }
}

/// 未连接时被跳过的请求记录（供 UI 展示或日志）
#[derive(Clone, Debug)]
pub struct SkippedRequest {
    pub kind: RequestKind,
    pub reason: &'static str,
    pub at: Instant,
}

/// 跳过请求的历史记录（有界 FIFO，最多保留最近 N 条）
pub struct SkippedLog {
    entries: std::collections::VecDeque<SkippedRequest>,
    capacity: usize,
}

impl SkippedLog {
    pub fn new(capacity: usize) -> Self {
        Self {
            entries: std::collections::VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    /// 记录一条跳过事件
    pub fn push(&mut self, kind: RequestKind, reason: &'static str) {
        if self.entries.len() >= self.capacity {
            self.entries.pop_front();
        }
        self.entries.push_back(SkippedRequest {
            kind,
            reason,
            at: Instant::now(),
        });
    }

    /// 获取最近 N 条记录
    pub fn recent(&self, n: usize) -> Vec<SkippedRequest> {
        self.entries.iter().rev().take(n).cloned().collect()
    }

    /// 清空记录
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}
