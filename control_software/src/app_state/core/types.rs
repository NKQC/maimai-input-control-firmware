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
