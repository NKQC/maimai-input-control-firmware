use crate::app_state::pipeline::RequestKind;
use crate::proto::Frame;

/// 连接后顺序回读探针：窗口=1，逐项收到数据响应后推进。
#[derive(Clone)]
pub(crate) struct ConnProbeStep {
    pub(crate) kind: RequestKind,
    pub(crate) frame: Frame,
}
