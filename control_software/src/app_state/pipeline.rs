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

use crate::proto::HostCmd;
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
/// - 已迁移到统一表：`algo_get_info` 以及所有当前"直发"的只读轮询
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
    /// 某次算法上传 ACK 后的真值确认；会话元数据随 seq 原子注册，旧响应不能命中新会话。
    AlgoUploadVerify {
        session_id: u64,
        expected_len: u16,
        expected_crc: u16,
        attempt: u8,
    },
    AlgoGetRom,
    AlgoGetCfg {
        idx: u8,
    },
    RequestAlgoSrc,
    RequestAlgoCode,
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
            Self::AlgoUploadVerify {
                session_id,
                expected_len,
                expected_crc,
                attempt,
            } => format!(
                "ALGO_GET_INFO(upload_session={} len={} crc=0x{:04X} attempt={})",
                session_id, expected_len, expected_crc, attempt
            ),
            Self::AlgoGetRom => "ALGO_GET_ROM".to_string(),
            Self::AlgoGetCfg { idx } => format!("ALGO_GET_CFG(idx={})", idx),
            Self::RequestAlgoSrc => "ALGO_GET_SRC".to_string(),
            Self::RequestAlgoCode => "ALGO_GET_CODE".to_string(),

            Self::BusXfer => "BUS_XFER".to_string(),
            Self::ConnProbe { name } => format!("连接探针({})", name),
            Self::CfgTx { cmd } => format!("CFG_TX(cmd=0x{:02X})", cmd),
        }
    }

    /// 该请求期待的数据响应 cmd；`None` = 结局只由 ACK/NAK 或专属状态机给出。
    ///
    /// ★这里禁止 `_` 通配★
    /// 原先这份映射内联在 `_confirm_data_pending` 里并以 `_ => false` 兜底，
    /// 结果 `LedRequestState` 从未配上一臂：设备正确应答了 LED_GET、主机也解出了
    /// 96B 快照，却没人注销在途登记 ⇒ 窗口=1 的连接探针队列被占死，只能等 6s 超时
    /// 才推进（用户可见表现：每次连接必卡 6s，算法 C 源回读被推到连接后 ~8.4s，
    /// 首次进单通道精调不出数）。改为穷尽匹配后，任何新增 RequestKind 都会在
    /// 编译期被逼着表态，不会再退化成静默故障。
    pub fn expected_response_cmd(&self) -> Option<u8> {
        let cmd = match self {
            Self::RequestParams { .. } | Self::RequestParamAllChannels { .. } => {
                HostCmd::ParamGetAll
            }
            Self::ConfigGetAll => HostCmd::CfgGetAll,
            Self::GlobalGetAll => HostCmd::GlobalGetAll,
            Self::KbdRequestState => HostCmd::KbdGetState,
            Self::KbdRequestMap => HostCmd::KbdGetMap,
            Self::KbdRequestTouchmap => HostCmd::KbdGetTouchmap,
            Self::KbdRequestHold => HostCmd::KbdGetHold,
            Self::KbdRequestEdges => HostCmd::KbdGetEdges,
            Self::KbdRequestKeycfg => HostCmd::KbdGetKeycfg,
            Self::KbdGetCombo => HostCmd::KbdGetCombo,
            Self::Mai2RequestState => HostCmd::Mai2GetState,
            Self::LedRequestState => HostCmd::LedGet,
            Self::AlgoGetInfo => HostCmd::AlgoGetInfo,
            Self::AlgoGetRom => HostCmd::AlgoGetRom,
            Self::AlgoGetCfg { .. } => HostCmd::AlgoGetCfg,
            Self::RequestAlgoSrc => HostCmd::AlgoGetSrc,
            Self::RequestAlgoCode => HostCmd::AlgoGetCode,
            Self::BusXfer => HostCmd::BusXfer,
            // 以下没有"数据响应"这一步：PING/写类只有 ACK/NAK；上传确认与连接探针
            // 由各自状态机归因，不能在通用路径上被提前注销。
            Self::Ping
            | Self::AlgoUploadVerify { .. }
            | Self::ConnProbe { .. }
            | Self::CfgTx { .. } => return None,
        };
        Some(cmd as u8)
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

    /// 在途请求的可读清单(供延迟尖峰现场记录)。★按类别去重并排序★: 同类多条在途时只需知道
    /// "它在跑"; 排序使日志行可直接横向比对, 不受 HashMap 迭代顺序影响。
    pub fn inflight_labels(&self) -> Vec<String> {
        let mut v: Vec<String> = self.map.values().map(|req| req.kind.label()).collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// 是否已有同类请求在途，轮询任务用它实现窗口=1。
    pub fn contains_kind(&self, kind: &RequestKind) -> bool {
        self.map.values().any(|req| &req.kind == kind)
    }

    /// 取消指定类别的在途请求。用于写入持久化屏障前丢弃旧读请求，避免保存后的回读被旧响应占用窗口。
    pub fn cancel_kind(&mut self, kind: &RequestKind) -> Vec<u8> {
        let seqs: Vec<u8> = self
            .map
            .iter()
            .filter_map(|(seq, req)| (&req.kind == kind).then_some(*seq))
            .collect();
        for seq in &seqs {
            self.map.remove(seq);
        }
        seqs
    }

    /// 清除所有指定类别的在途请求。
    pub fn cancel_kinds(&mut self, kinds: &[RequestKind]) -> Vec<u8> {
        let mut seqs = Vec::new();
        for kind in kinds {
            seqs.extend(self.cancel_kind(kind));
        }
        seqs
    }

    /// 是否有任意 ALGO_GET_INFO 请求在途；普通刷新与上传确认共享设备端单一读窗口。
    pub fn contains_algo_info_request(&self) -> bool {
        self.map.values().any(|req| {
            matches!(
                req.kind,
                RequestKind::AlgoGetInfo | RequestKind::AlgoUploadVerify { .. }
            )
        })
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

    /// 清空记录
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}
