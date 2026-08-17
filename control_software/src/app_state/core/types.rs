use crate::comport::CdcFunction;

/// UI 日志等级。数值越大越"啰嗦": Error(0) < Warn(1) < Info(2) < Debug(3)。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LogLevel {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
}

/// 与设备的连接状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnState {
    Disconnected,
    Connecting,
    Connected,
}

/// PSoC SPI 链路活性证据。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PsocLinkEvidence {
    ScanAdvancing {
        sps: u32,
    },
    Handshake {
        generation: u16,
    },
    Stale {
        since_ms: Option<u64>,
        last_generation: Option<u16>,
    },
}

/// 一条可供 UI 下拉选择的设备候选。
#[derive(Debug, Clone)]
pub struct DeviceEntry {
    pub port_name: String,
    pub function: CdcFunction,
    pub label: String,
}

/// 一次算法编译的全部产物。
pub struct CompiledAlgo {
    pub blob: Vec<u8>,
    pub asm: String,
}

/// 设备回报的算法容量快照(来源: `ALGO_GET_INFO` 26 字节响应的容量组)。
///
/// ★为什么要"快照"这个形态★ 编译跑在后台线程(见 `AppController::compile_blob`), 那里既拿不到
/// `&AppController`(`Rc<RefCell<_>>` 不是 Send), 也不能等 UI 线程回答"容量是多少"。所以由 UI 线程
/// 在 spawn 之前 `algo_caps_snapshot()` 取一份纯数据带过去 —— 编译期间设备就算重连改了容量,
/// 这一次编译用的也仍是提交那一刻的口径, 报错文案与实际判据永远一致。
///
/// 全部字段都是 usize: 与 `Vec::len()`/切片长度同一量纲, 免得在闸门判定处到处 `as usize`。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlgoCaps {
    /// PSoC 可执行槽容量(字节)。★只作占用百分比的分母★, 不是闸门。
    pub slot: usize,
    /// 单帧可上传的算法字节上限。★编译产物与上传的唯一闸门★(比 slot 小 4 字节的帧内头)。
    pub upload_limit: usize,
    /// 算法 C 源存储容量(字节)。
    pub src: usize,
    /// 算法 C 源分片粒度(字节)。分片步长必须与设备一致, 否则设备按自己的粒度校验会 NAK。
    pub src_chunk: usize,
    /// 共享堆容量(字节), 只用于占用百分比显示。
    pub heap: usize,
    /// `slot` 是否来自 PSoC 自报(false = RP 的兜底常量)。
    pub from_psoc: bool,
    /// PSoC 与 RP 两侧容量常量不一致 —— 上传可能被一侧放行、另一侧静默截断, 必须显式告警。
    pub mismatch: bool,
}
