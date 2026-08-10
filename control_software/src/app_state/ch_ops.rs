//! 逐通道 CSD 操作的 **host 侧编排**。三件事, 都只由 `csd_diag_tick()` 的既有 16ms 节拍驱动:
//!
//! 1. [`ChBatch`] —— 批量操作(全通道校准 / 全通道基线 / 全通道频率自适应)的**可取消串行队列**。
//!    ★为什么搬到 host★ 固件内部那个 36 通道 for 循环会把 PSoC 主循环 + RP2040 的 core1 一起占住
//!    十几到几十秒: 期间不可取消、没有进度、单通道失败也说不清是哪一个, 而 SPI 上又必须靠"重操作
//!    宽限窗"硬扛链路抖动。改成 host 逐通道下发后, 每一条都是短操作(单 widget), 进度/取消/逐通道
//!    错误收敛全都成立, 固件那条全通道路径只留作兼容入口(恢复默认等内部触发仍要用)。
//!
//! 2. [`NoiseSweep`] —— 单通道**响应噪声频谱**扫描(gain 0..6 × 分频 1..64)。纯 host 编排:
//!    只用既有 `PARAM_SET(0x0B/0x08)` + 单通道 `CALIBRATE` + RAW 遥测, 不新增任何协议。
//!
//! 3. [`PlotFreeze`] —— 绘图视窗冻结。"停止"从此只冻结**看到的东西**, 设备遥测一直在收。
//!
//! ★与 mod.rs 的边界★ 本文件只放"编排状态机 + 它自己的数据结构"; 发送、参数缓存、op 忙闲判定
//! 一律复用 mod.rs 既有接口(`_queue_tx` / `debug_param_now` / `_csd_locked` / `calibrate` …),
//! 不在这里另开第二条发送路径, 也不复制一份忙闲判定。

use std::collections::VecDeque;

use super::AppController;
use crate::proto::{
    ChannelSample, IDAC_GAIN_BY_CURRENT, IDAC_GAIN_PA, SWEEP_FLAG_CAL_FAIL, SWEEP_FLAG_MISMATCH,
    SWEEP_FLAG_RAILED, SWEEP_FLAG_STALLED, SweepFrame, SweepState,
};
use crate::proto::{PARAM_IDAC_GAIN, PARAM_SNS_CLK_DIV};

/// 面板通道数(与固件 `SENSOR_CHANNEL_COUNT` 一致)。
const CH_COUNT: usize = 36;

// ============================================================================
// ① 批量操作串行队列
// ============================================================================

/// 批量操作的种类。★三种共用同一队列/同一进度/同一取消★ —— 它们的编排完全同构,
/// 分三份状态只会让"正在忙什么"出现三个可能互相矛盾的答案。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChBatchKind {
    Calibrate,
    BaselineReset,
    AutoTune,
}

impl ChBatchKind {
    fn label(self) -> &'static str {
        match self {
            ChBatchKind::Calibrate => "逐通道校准",
            ChBatchKind::BaselineReset => "逐通道基线复位",
            ChBatchKind::AutoTune => "逐通道频率自适应",
        }
    }
}

/// 一个通道在本次批量里的终态。★完成文案的唯一计数来源★ 不再另设 `failed` 之类的平行计数:
/// 两份计数必然有一份说谎, 而"下发成功"从来不等于"设备做成了"。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ChOpRes {
    /// 还没轮到(用户取消时剩下的通道就停在这里)。
    Idle,
    /// 已发出, 正等设备的终态(ACK / NAK / 自适应终态帧 / 看门狗超时)。
    Sent,
    /// 设备已确认完成。
    Ok,
    /// 没发出去(未连接 / 被串行化或互斥守卫拒绝)。
    SendFail,
    /// 发出去了, 但设备报失败或压根没回执(异步失败)。
    Failed,
}

/// 当前在途的那一条。★靠 `_begin_op` 记下的 op seq 归因★ NAK/ACK/超时路径手上只有 seq,
/// 用它匹配才不会把别的操作的失败算到本批量头上, 也天然保证同一条不被记两次。
#[derive(Debug, Clone, Copy)]
struct ChBatchOp {
    ch: u8,
    seq: u8,
}

/// 队列阶段。`Prime`/`PrimeVerify` 只有频率自适应会走 —— 开跑前必须先把每通道的 IDAC 增益档
/// 明确写进 36 个通道**并回读确认**, 否则每通道的临界频率是在一个谁也说不清的增益档上找出来的。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ChBatchStage {
    /// 等待一次全通道 ENABLED 回读，禁止以“缓存缺失=启用”构造自动化队列。
    AwaitEnabled,
    /// 把全部目标通道的 `PARAM_SET(ch, 0x0B, gain)` 一次排入共享 FIFO。
    Prime,
    /// 全部写帧结算后只发一次全通道回读确认。
    PrimeVerify,
    /// 用户取消或前置回读超时后的收敛：仅清本批未发帧，等待本批在途帧结算后才释放所有权。
    CancelPrime,
    /// 逐通道发起本次批量操作。
    Run,
    /// 取消后的在途 op/cooldown 收敛；不再发新操作。
    CancelRun,
    /// 已形成终态文案，仍保留控制面到最后一个 op/cooldown 与 cfg 帧彻底结算。
    Done,
}

/// 批量操作的进度/结果与 per-channel 偏向。
#[derive(Debug)]
pub struct ChBatch {
    kind: Option<ChBatchKind>,
    stage: ChBatchStage,
    /// 待处理通道(按下发顺序)。
    queue: Vec<u8>,
    /// 下一个待发通道在 `queue` 中的下标。
    next: usize,
    /// 前置写/回读的当前通道。与 `next` 分离，保证写一条、确认一条后才触及下一条。
    prime_next: usize,
    /// 前置当前通道的 PARAM_GET 已发出。
    prime_asked: bool,
    /// 本批拥有的 cfg 帧代号；取消只可清理同代未发帧，绝不粗暴清空他人队列。
    generation: u64,
    /// true 仅包裹批量自身的 CSD 下发，供 Host 侧控制面守卫放行。
    own_tx: bool,
    /// 逐通道终态(见 [`ChOpRes`])。
    res: [ChOpRes; CH_COUNT],
    /// 在途的那一条(通道 + op seq); None = 当前没有等回执的操作。
    inflight: Option<ChBatchOp>,
    /// 频率自适应的 per-channel 偏好档(1..7)。Cp 辅助关或无有效 Cp 时全部 = 用户偏好。
    pref: [u8; CH_COUNT],
    /// 频率自适应前置写入的 per-channel IDAC 增益档(0..6)。
    gain: [u8; CH_COUNT],
    /// 前置写入是否已被设备回读确认。
    verified: [bool; CH_COUNT],
    /// 确认阶段已等待的 tick 数(超时即中止, 绝不"没确认就开跑")。
    verify_ticks: u32,
    /// 确认阶段是否已发出回读请求(写队列排空后只发一次)。
    verify_asked: bool,
    /// 本次是否启用了 Cp 辅助(供状态文案如实说明)。
    cp_assisted: bool,
    status: String,
    version: u64,
}

impl Default for ChBatch {
    fn default() -> Self {
        Self {
            kind: None,
            stage: ChBatchStage::Done,
            queue: Vec::new(),
            next: 0,
            prime_next: 0,
            prime_asked: false,
            generation: 0,
            own_tx: false,
            res: [ChOpRes::Idle; CH_COUNT],
            inflight: None,
            pref: [4u8; CH_COUNT],
            gain: [0u8; CH_COUNT],
            verified: [false; CH_COUNT],
            verify_ticks: 0,
            verify_asked: false,
            cp_assisted: false,
            status: String::new(),
            version: 0,
        }
    }
}

impl ChBatch {
    /// 复位为空闲(保留 status/version 供 UI 显示最后一次结果)。
    fn clear(&mut self) {
        self.kind = None;
        self.stage = ChBatchStage::Done;
        self.queue.clear();
        self.next = 0;
        self.prime_next = 0;
        self.prime_asked = false;
        self.own_tx = false;
        self.res = [ChOpRes::Idle; CH_COUNT];
        self.inflight = None;
        self.verified = [false; CH_COUNT];
        self.verify_ticks = 0;
        self.verify_asked = false;
    }

    /// 逐通道终态统计: `(确认成功数, 已发出但没等到回执数, 失败通道号)`。
    fn _tally(&self) -> (usize, usize, Vec<u8>) {
        let mut ok = 0usize;
        let mut pending = 0usize;
        let mut failed: Vec<u8> = Vec::new();
        for (ch, r) in self.res.iter().enumerate() {
            match r {
                ChOpRes::Ok => ok += 1,
                ChOpRes::Sent => pending += 1,
                ChOpRes::SendFail | ChOpRes::Failed => failed.push(ch as u8),
                ChOpRes::Idle => {}
            }
        }
        (ok, pending, failed)
    }
}

/// 前置写入确认的最长等待(tick, 16ms/tick)。36 条写 + 一次全通道回读在正常链路上远快于此;
/// 到点仍对不上就必须中止并如实报告, 不能"没确认就开跑"。
const PRIME_VERIFY_TIMEOUT_TICKS: u32 = 500; // ~8s

// ============================================================================
// ② Cp → per-channel 偏向(有边界、可解释的连续映射)
// ============================================================================

/// 一个通道由 Cp 推出的偏向。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChBias {
    /// IDAC 增益档索引(0..6)。
    pub gain: u8,
    /// 频率偏好档(1..7)。
    pub pref: u8,
    /// 该通道是否真的用上了 Cp(false = 无有效 Cp, 已退回用户偏好/全局起点档)。
    pub from_cp: bool,
}

/// Cp 哨兵: 未测量 / 测量失败(与固件、`main.rs` 同一取值)。
const CP_INVALID: u32 = 0x00FF_FFFF;

/// `pref` 相对用户偏好的最大偏移(±)。★必须有界★: pref 每 +1 就在临界分频上再往低频让 2 档,
/// 放开了会把低 Cp 通道推到毫无必要的低频, 白白拖慢整轮扫描。
const PREF_SPREAD: f32 = 2.0;

/// 由一组 Cp 推出每通道偏向。
///
/// ── 为什么是这两条映射(先查了真实方向再定) ─────────────────────────────────────────
/// **增益档**: 需要的补偿电流 ∝ 被测电容(I = C·V·f)。设备的 `idacGainTable`
/// (`cycfg_capsense.c:171`)给出每档的电流(pA/LSB)是 37.5k / 75k / 300k / 600k / 2400k /
/// 4800k / **1200k** —— ★档号与电流不单调★(档 6 落在档 3 与档 4 之间)。所以映射必须在**电流**上做,
/// 而不是在档号上做线性缩放。做法: 以本面板 Cp 的**中位数**作参考点, 该点用用户在全局页选的起点档
/// (`start_gain`), 其余通道按 `I_target = I(start_gain) · Cp / Cp_median` 求目标电流, 再取
/// **电流最接近**的档(比值取对数距离, 因为相邻档是 2×/4× 的乘性关系)。于是: 全部 Cp 相同时结果
/// 精确退化为用户选的那一档; 高 Cp 通道自动往大电流档走; 表两端天然封住上下界。
/// **频率偏好**: `pref` 的语义在固件里是"落档时在临界分频上再往低频让 `2·(pref-1)` 个分频"
/// (`main.c::auto_tune_run_ch` ③)。低频=充电更充分, 正是高 Cp 电极需要的。故 Cp 在本面板 Cp
/// 值域内线性归一后, 以用户偏好为中心 ±[`PREF_SPREAD`] 档: Cp 最低的通道 -2, 最高的 +2,
/// 中间线性插值, 最后夹到 1..7。全部 Cp 相同(或无有效 Cp)时同样精确退化为用户偏好。
pub fn cp_bias_table(cp: &[Option<u32>], start_gain: u8, user_pref: u8) -> [ChBias; CH_COUNT] {
    let fallback = ChBias {
        gain: start_gain.min(6),
        pref: user_pref.clamp(1, 7),
        from_cp: false,
    };
    let mut out = [fallback; CH_COUNT];
    let mut valid: Vec<u32> = cp
        .iter()
        .take(CH_COUNT)
        .filter_map(|v| *v)
        .filter(|v| *v != 0 && *v != CP_INVALID)
        .collect();
    if valid.len() < 2 {
        return out; // 有效样本不足两个 ⇒ 谈不上"相对偏向", 全部退回用户偏好
    }
    valid.sort_unstable();
    let cp_med = valid[valid.len() / 2].max(1) as f32;
    let cp_lo = valid[0] as f32;
    let cp_hi = valid[valid.len() - 1] as f32;
    let span = (cp_hi - cp_lo).max(1.0);
    let i_ref = IDAC_GAIN_PA[start_gain.min(6) as usize] as f32;
    for (ch, slot) in out.iter_mut().enumerate() {
        let Some(v) = cp.get(ch).copied().flatten() else {
            continue;
        };
        if v == 0 || v == CP_INVALID {
            continue; // 该通道无有效 Cp: 保持用户偏好(fallback), 不猜
        }
        let cp_ff = v as f32;
        slot.gain = _gain_for_current(i_ref * cp_ff / cp_med);
        let t = ((cp_ff - cp_lo) / span).clamp(0.0, 1.0); // 0=本面板最低 Cp, 1=最高
        let delta = (t - 0.5) * 2.0 * PREF_SPREAD; // -PREF_SPREAD .. +PREF_SPREAD
        slot.pref = ((user_pref as f32 + delta).round() as i32).clamp(1, 7) as u8;
        slot.from_cp = true;
    }
    out
}

/// 取"每 LSB 电流"最接近目标值的增益档。★用对数距离★: 相邻档是 2×/4× 的乘性关系,
/// 线性距离会一律偏向大电流档(4800k 与 2400k 的线性差比 75k 与 37.5k 大两个数量级)。
fn _gain_for_current(target_pa: f32) -> u8 {
    let t = target_pa.max(1.0).ln();
    let mut best = IDAC_GAIN_BY_CURRENT[0];
    let mut best_d = f32::MAX;
    for idx in IDAC_GAIN_BY_CURRENT {
        let d = ((IDAC_GAIN_PA[idx as usize] as f32).ln() - t).abs();
        if d < best_d {
            best_d = d;
            best = idx;
        }
    }
    best
}

// ============================================================================
// ③ 响应噪声频谱扫描
// ============================================================================

/// 频谱一个格子的实测结果。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct NoiseCell {
    /// RAW 标准差(主指标: 可解释、与量纲一致)。
    pub std: f32,
    /// RAW 峰峰值(附带指标: 对偶发尖刺比标准差敏感)。
    pub pp: f32,
    /// RAW 均值(用于判断该点是否已经撞顶/贴底 —— 那种点的"低噪声"是假的)。
    pub mean: f32,
    /// 实际用于统计的样本数。
    pub samples: u16,
    /// 设备上报的本格诊断位(含 CAL_FAIL/MISMATCH/STALLED/RAILED)。
    pub flags: u8,
    /// 设备回读的实际生效 IDAC 增益档。
    pub gain: u8,
    /// 设备回读的实际生效传感时钟分频。
    pub div: u8,
    /// 设备上报的本格故障阶段；旧固件未上报时为 None。
    pub fail_phase: Option<u8>,
    /// true = 本格已测且样本足够。
    pub valid: bool,
    /// true = 该点 RAW 撞到满量程或贴底 ⇒ diff 失效, 其"噪声低"没有意义, UI 必须区别标注。
    pub railed: bool,
}

/// 扫描阶段。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SweepPhase {
    /// 写 gain/div → **按设备真值确认** → 单通道校准并等终态回执(让新配置真正生效并重解 IDAC)。
    /// 见 [`ApplyStep`]: 三步全部确认才允许采样, 任一步失败/超时则本格标无效并跳过采样。
    Configure,
    /// 等设备把这次校准做完 + 让基线/滤波稳定。
    Settle,
    /// 收集 RAW 样本。
    Sample,
    /// 恢复原始 gain/div 并重新校准 + 复位基线。
    Restore,
}

/// 「配置写入 → 按设备真值确认 → 单通道校准[→ 基线复位]」的子步骤。
///
/// ★每一步都必须拿到设备真值 / 终态回执才推进★ `debug_param_now` 只是把 PARAM_SET 排进队列并
/// **乐观**写本地缓存, 校准/基线也只有 ACK 才是真完成。把"已排队"当"已生效"在两条路上都是假成功:
/// 还原链上它把一个配置未知的通道交还给用户(且互斥已放开); 每格配置上它让热图那一格标着
/// (gain, div), 采到的 RAW 却是上一格配置产生的。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ApplyStep {
    /// 写目标 gain/div(入既有串行写队列)。
    Write,
    /// 等写队列排空 → 清本地乐观缓存 → 回读比对**设备真值**。
    Verify,
    /// 单通道校准, 等 ACK(这类操作的 ACK = 固件真做完)。
    Calibrate,
    /// 单通道基线复位, 等 ACK。★仅还原链走★ 扫描逐格改配置把基线搅过, 还原后必须复位。
    Baseline,
}

impl ApplyStep {
    fn _label(self) -> &'static str {
        match self {
            ApplyStep::Write => "配置写入",
            ApplyStep::Verify => "配置回读确认",
            ApplyStep::Calibrate => "单通道校准",
            ApplyStep::Baseline => "单通道基线复位",
        }
    }
}

/// 一次推进的目标与语境。★同一套状态机被两处复用★(每格配置 / 扫完还原): 差别只有目标值、
/// 是否收尾做基线复位、以及超时预算 —— 写成两份实现必然漂移出两种互相矛盾的"确认"语义。
#[derive(Debug, Clone, Copy)]
struct ApplyReq {
    /// 目标 IDAC 增益档; None = 本项无目标(原值缺失时不凭空写)。
    gain: Option<u32>,
    /// 目标传感时钟分频; 语义同上。
    div: Option<u32>,
    /// true = 还原链(目标是原值, 额外做基线复位, 超时预算宽松);
    /// false = 每格配置(448 格串行, 预算必须紧, 失败即弃该格)。
    restoring: bool,
}

impl ApplyReq {
    fn _what(self) -> &'static str {
        if self.restoring {
            "还原"
        } else {
            "本格配置"
        }
    }
    fn _verify_timeout(self) -> u32 {
        if self.restoring {
            RESTORE_VERIFY_TIMEOUT_TICKS
        } else {
            CELL_VERIFY_TIMEOUT_TICKS
        }
    }
    fn _op_timeout(self) -> u32 {
        if self.restoring {
            RESTORE_OP_TIMEOUT_TICKS
        } else {
            CELL_OP_TIMEOUT_TICKS
        }
    }
}

/// 一次推进的结果。★失败的后果由调用方决定★: 还原链收尾报"恢复未确认"; 每格配置把该格标无效、
/// 不采样、直接进下一格。
enum ApplyOut {
    /// 本 tick 还没收敛, 下一拍继续。
    Pending,
    /// 每一步都拿到了设备确认。
    Done,
    /// 失败或有界超时(附根因)。
    Fail(String),
}

/// 配置确认状态机。★与扫描主状态分开★ 它自己要收敛(写 → 回读确认 → 校准[→ 基线]),
/// 挤进 `phase_ticks` 那种"第几拍"计数里只能表达"发过了", 表达不了"设备确认了"。
#[derive(Debug)]
struct SweepApply {
    step: ApplyStep,
    /// 本步已等待的 tick。★有界★: 到点如实报"未确认"并收尾, 不无限锁死。
    ticks: u32,
    /// 在途 CSD 操作的 seq。★以 `op_wait_seq` 为"确实发出去了"的唯一证据★(同 `ChBatch.inflight`):
    /// `calibrate`/`baseline_reset` 被串行化守卫拒绝时返回的也是 `Ok(())`, 只看 Err 会把没发出去的当在途。
    op: Option<u8>,
    /// 在途操作的设备终态: None = 还在等, Some(true) = ACK, Some(false) = NAK/超时。
    op_res: Option<bool>,
    /// 本步的回读请求已发出(收不到就周期性重问)。
    asked: bool,
    /// 首个失败原因(根因)。Some ⇒ 本次推进不许声称成功: 还原链报"恢复未确认", 每格配置标该格无效。
    fail: Option<String>,
}

impl Default for SweepApply {
    fn default() -> Self {
        Self {
            step: ApplyStep::Write,
            ticks: 0,
            op: None,
            op_res: None,
            asked: false,
            fail: None,
        }
    }
}

/// 还原链上"回读确认"的有界等待(tick, 16ms/tick): 两条 PARAM_SET 排空 + 两条回读, 正常远快于此。
const RESTORE_VERIFY_TIMEOUT_TICKS: u32 = 375; // ~6s
/// 还原链上单步 CSD 操作(单通道校准/基线)等终态回执的有界上限。
/// 取值小于 mod.rs 的 51s 阻塞看门狗: 到点先由本状态机如实报"设备实际状态未知"并收敛,
/// 剩下的 op 忙闲仍由既有看门狗兜底解锁 —— 两者都不构成无限锁死。
const RESTORE_OP_TIMEOUT_TICKS: u32 = 1000; // ~16s
/// 每格配置的确认预算, 比还原链紧: 还原是一次性的, 而这条要走 448 格。
/// 到点即弃该格(标无效、不采样), 于是链路异常时整轮扫描仍然收敛而不是原地泡着。
const CELL_VERIFY_TIMEOUT_TICKS: u32 = 125; // ~2s
/// 每格的单通道校准等 ACK 上限。单 widget 校准在 PSoC 侧是十几 ms 级, 留足余量仍远小于 51s 看门狗。
const CELL_OP_TIMEOUT_TICKS: u32 = 375; // ~6s

/// 扫描的 gain 档数与分频档数(需求给定: gain 0..6, 分频 1..64)。
/// ★对外可见★: `noise_sweep_cells()` 是一维的, 使用方(main.rs 建热图模型)必须按同一对常量还原
/// (gain, div) —— 在那边另写一对 7/64 就是第二份定义, 改这里改不到那里。
pub const SWEEP_GAINS: usize = 7;
pub const SWEEP_DIVS: usize = 64;
const SWEEP_CELLS: usize = SWEEP_GAINS * SWEEP_DIVS;
/// 每格的稳定等待与采样窗(tick, 16ms/tick)。
const SWEEP_SETTLE_TICKS: u32 = 8; // ~128ms
const SWEEP_SAMPLE_TICKS: u32 = 20; // ~320ms(逐通道档 100Hz ⇒ 约 32 个样本)
/// 一格至少要拿到这么多样本才算有效; 拿不到就如实标为无效, 不用 1-2 个点硬算标准差。
const SWEEP_SAMPLE_MIN: usize = 8;
/// RAW 撞顶/贴底判据按本次扫描从设备确认的分辨率缩放；不允许固定按 12 位猜量程。
fn _rail_margin(full_scale: f32) -> f32 {
    (full_scale * 0.01).ceil().max(1.0)
}

/// 启动响应仍在途时的归属键。代次使切换/断连后迟到响应不能安装旧会话。
#[derive(Debug, Clone, Copy)]
struct SweepPending {
    seq: u8,
    generation: u16,
}

/// 单通道响应噪声频谱扫描状态。
///
/// 设备会话承担逐格改参、校准、采样与恢复；主机只持有会话归属、结果位图和有界补发状态。
#[derive(Debug)]
pub struct NoiseSweep {
    active: bool,
    ch: u8,
    generation: u16,
    pending: Option<SweepPending>,
    session: Option<u16>,
    total: u16,
    produced: u16,
    received: Vec<bool>,
    resend_tries: Vec<u8>,
    control_seqs: Vec<u8>,
    /// 扫描会话结束后仍可能迟到的控制 seq；仅用于静默已知过期控制 NAK。
    expired_control_seqs: Vec<u8>,
    /// 最近一次成功排入 SWEEP keepalive 的时刻；设备租约 8s，主机固定每 2s 续一次。
    keepalive_at: Option<std::time::Instant>,
    cancel: bool,
    /// 当前格字段仅保留给旧结果访问口径；设备会话不再以它们推进扫描。
    gain: u8,
    div: u16,
    resolution: u8,
    full_scale: f32,
    result_ch: Option<u8>,
    saved_gain: Option<u32>,
    saved_div: Option<u32>,
    phase: SweepPhase,
    phase_ticks: u32,
    ap: SweepApply,
    samples: Vec<u16>,
    last_t_us: Option<u32>,
    cells: Vec<NoiseCell>,
    own_tx: bool,
    status: String,
    version: u64,
}

impl Default for NoiseSweep {
    fn default() -> Self {
        Self {
            active: false,
            ch: 0,
            generation: 0,
            pending: None,
            session: None,
            total: SWEEP_CELLS as u16,
            produced: 0,
            received: vec![false; SWEEP_CELLS],
            resend_tries: vec![0; SWEEP_CELLS],
            control_seqs: Vec::new(),
            expired_control_seqs: Vec::new(),
            keepalive_at: None,
            cancel: false,
            gain: 0,
            div: 1,
            resolution: 0,
            full_scale: 0.0,
            result_ch: None,
            saved_gain: None,
            saved_div: None,
            phase: SweepPhase::Configure,
            phase_ticks: 0,
            ap: SweepApply::default(),
            samples: Vec::new(),
            last_t_us: None,
            cells: vec![NoiseCell::default(); SWEEP_CELLS],
            own_tx: false,
            status: String::new(),
            version: 0,
        }
    }
}

impl NoiseSweep {
    fn _idx(gain: u8, div: u16) -> usize {
        (gain as usize) * SWEEP_DIVS + (div as usize - 1)
    }

    fn _done(&self) -> usize {
        self.received.iter().filter(|received| **received).count()
    }

    fn _expected_total(&self) -> usize {
        usize::from(self.total).min(SWEEP_CELLS)
    }
}

// ============================================================================
// ④ 绘图视窗冻结
// ============================================================================

/// 冻结时刻的样本快照 + 时钟锚点。★为什么要整份快照★ 遥测仍在推流, 环形缓冲会持续把旧样本挤走;
/// 只记一个"截止时间"的话, 冻结的画面会随着缓冲被覆盖而慢慢空掉。快照一次(36×≤1024 样本)
/// 换来的是"图真的停住了"。
#[derive(Debug)]
pub struct PlotFreeze {
    pub(super) buf: Vec<VecDeque<ChannelSample>>,
    pub(super) acc_us: u64,
    pub(super) prev_raw: Option<u32>,
    pub(super) algo_report: [VecDeque<super::TracePoint>; 4],
    pub(super) algo_active: VecDeque<super::TracePoint>,
}

// ============================================================================
// AppController 上的编排接口
// ============================================================================

impl AppController {
    // ---------------- 批量串行队列 ----------------

    pub fn ch_batch_active(&self) -> bool {
        self.ch_batch.kind.is_some()
    }
    pub fn ch_batch_status(&self) -> &str {
        &self.ch_batch.status
    }
    pub fn ch_batch_version(&self) -> u64 {
        self.ch_batch.version
    }
    /// 完成比例 0.0..1.0(供进度条)。空闲时为 0。
    pub fn ch_batch_progress(&self) -> f32 {
        if self.ch_batch.kind.is_none() || self.ch_batch.queue.is_empty() {
            return 0.0;
        }
        self.ch_batch.next as f32 / self.ch_batch.queue.len() as f32
    }

    /// 发起一次批量操作。启用状态尚未回读完整时先自动等待一次全通道回读。
    pub fn ch_batch_start(&mut self, kind: ChBatchKind) -> anyhow::Result<()> {
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法发起{}", kind.label()));
        }
        if let Some(cur) = self.ch_batch.kind {
            self.push_log(format!(
                "{}正在进行中, 已忽略本次「{}」",
                cur.label(),
                kind.label()
            ));
            return Ok(());
        }
        if self.noise_sweep.active {
            return Err(anyhow::anyhow!("CH{} 频谱扫描进行中", self.noise_sweep.ch));
        }
        if self._csd_locked() || !self.cfg_tx_queue.is_empty() || self.cfg_tx_inflight.is_some() {
            return Err(anyhow::anyhow!(
                "设备发送队列仍忙，无法开始{}",
                kind.label()
            ));
        }
        self.ch_batch.clear();
        self.ch_batch.generation = self.ch_batch.generation.wrapping_add(1);
        self.ch_batch.kind = Some(kind);
        if !self.ch_enabled_states_known() {
            self.ch_batch.stage = ChBatchStage::AwaitEnabled;
            self.ch_batch.status = format!("{}: 正在读取通道启用状态…", kind.label());
            self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
            self.request_param_all_channels(crate::proto::PARAM_ENABLED)?;
            return Ok(());
        }
        self._ch_batch_begin_ready(kind)
    }

    fn _ch_batch_begin_ready(&mut self, kind: ChBatchKind) -> anyhow::Result<()> {
        self.ch_batch.queue = (0..CH_COUNT as u8)
            .filter(|ch| self.ch_enabled_for_operation(*ch) == Some(true))
            .collect();
        let queued = self.ch_batch.queue.len();
        if queued == 0 {
            self.ch_batch.clear();
            return Err(anyhow::anyhow!(
                "没有已启用的通道, {}无对象可执行",
                kind.label()
            ));
        }
        if kind == ChBatchKind::AutoTune {
            self.auto_tune_ch_result = [0; CH_COUNT];
            self.auto_tune_ch_div = [0; CH_COUNT];
            self._ch_batch_plan_auto_tune();
            self.ch_batch.stage = ChBatchStage::Prime;
        } else {
            self.ch_batch.stage = ChBatchStage::Run;
        }
        self.ch_batch.status = format!("{}: 已排队 {} 个启用通道", kind.label(), queued);
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
        self.push_log(format!(
            "{}: 共 {} 个启用通道，跳过 {} 个禁用通道",
            kind.label(),
            queued,
            CH_COUNT - queued
        ));
        Ok(())
    }

    /// 用户取消: 先停止新下发，再等待本批已在途帧/op 与冷却窗口收敛；控制面在整个清理期间仍归本批。
    pub fn ch_batch_cancel(&mut self) {
        let Some(kind) = self.ch_batch.kind else {
            return;
        };
        if matches!(
            self.ch_batch.stage,
            ChBatchStage::CancelPrime | ChBatchStage::CancelRun | ChBatchStage::Done
        ) {
            return;
        }
        if self.ch_batch.stage == ChBatchStage::AwaitEnabled {
            self.ch_batch.status = format!("{}: 已取消", kind.label());
            self.ch_batch.clear();
            self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
            return;
        }
        let (ok, _pending, failed) = self.ch_batch._tally();
        self.ch_batch.status = format!(
            "{}: 正在取消(已发出 {}/{}, 设备确认成功 {}{}); 正等待本批在途操作与写帧收敛…",
            kind.label(),
            self.ch_batch.next,
            self.ch_batch.queue.len(),
            ok,
            if failed.is_empty() {
                String::new()
            } else {
                format!(", 失败 {}", failed.len())
            }
        );
        let priming = matches!(
            self.ch_batch.stage,
            ChBatchStage::Prime | ChBatchStage::PrimeVerify
        );
        if priming {
            self._cfg_purge_batch_frames(self.ch_batch.generation);
            self.ch_batch.stage = ChBatchStage::CancelPrime;
        } else {
            self.ch_batch.stage = ChBatchStage::CancelRun;
        }
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
        self.push_log(self.ch_batch.status.clone());
    }

    /// 把某个 param 排进既有的"全 36 通道重读"队列(去重)。★复用 mod.rs 那条队列★:
    /// 它每 tick 只发一条 PARAM_GET_ALL, 自己另发一次会与它抢同一个单条在途的回读位。
    fn _param_refetch_push(&mut self, param_id: u8) {
        if !self.param_refetch_ids.contains(&param_id) {
            self.param_refetch_ids.push(param_id);
        }
    }

    /// 计算本次逐通道自适应的 per-channel 增益档与偏好档。
    fn _ch_batch_plan_auto_tune(&mut self) {
        let user_pref = self._calib_pref();
        let start_gain = self
            .global(crate::proto::algo::GPARAM_IDAC_GAIN_INIT)
            .unwrap_or(4)
            .min(6) as u8;
        let assist = self.cp_assist;
        let bias = if assist {
            cp_bias_table(&self.cp, start_gain, user_pref)
        } else {
            [ChBias {
                gain: start_gain,
                pref: user_pref,
                from_cp: false,
            }; CH_COUNT]
        };
        let mut from_cp = 0usize;
        for ch in 0..CH_COUNT {
            self.ch_batch.gain[ch] = bias[ch].gain;
            self.ch_batch.pref[ch] = bias[ch].pref;
            if bias[ch].from_cp {
                from_cp += 1;
            }
        }
        self.ch_batch.cp_assisted = assist && from_cp > 0;
        if assist {
            self.push_log(format!(
                "Cp 辅助: {}/36 个通道用实测 Cp 推出增益档/偏好档, 其余退回用户偏好(档 {})。",
                from_cp, user_pref
            ));
        }
    }

    /// 取用户的频率偏好档(草稿优先, 与 `auto_tune` 同一口径; 越界回落 4)。
    fn _calib_pref(&self) -> u8 {
        let v = match self.config_get("calib.pref").map(|e| e.value) {
            Some(crate::proto::config::CfgValue::U8(v)) => v as i64,
            Some(crate::proto::config::CfgValue::U16(v)) => v as i64,
            Some(crate::proto::config::CfgValue::U32(v)) => v as i64,
            Some(crate::proto::config::CfgValue::I8(v)) => v as i64,
            Some(crate::proto::config::CfgValue::F32(v)) => v as i64,
            _ => 4,
        };
        if (1..=7).contains(&v) { v as u8 } else { 4 }
    }

    /// 某通道本次实际采用的偏向(供 UI 逐通道显示"Cp 辅助把它偏到哪一档")。
    /// 批量未在跑时按当前 Cp/偏好即时算, 使界面在点按钮之前就能看到将要采用的值。
    pub fn cp_bias_of(&self, ch: u8) -> ChBias {
        let user_pref = self._calib_pref();
        let start_gain = self
            .global(crate::proto::algo::GPARAM_IDAC_GAIN_INIT)
            .unwrap_or(4)
            .min(6) as u8;
        if !self.cp_assist || (ch as usize) >= CH_COUNT {
            return ChBias {
                gain: start_gain,
                pref: user_pref,
                from_cp: false,
            };
        }
        cp_bias_table(&self.cp, start_gain, user_pref)[ch as usize]
    }

    /// "Cp 辅助"开关。关 ⇒ 所有通道用同一个用户偏好档(旧行为)。
    pub fn cp_assist(&self) -> bool {
        self.cp_assist
    }
    pub fn set_cp_assist(&mut self, on: bool) {
        if self.cp_assist == on {
            return;
        }
        self.cp_assist = on;
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
        self.push_log(if on {
            "Cp 辅助: 已开启 —— 逐通道自适应将按实测 Cp 推出每通道的增益档与频率偏好档。"
                .to_string()
        } else {
            "Cp 辅助: 已关闭 —— 全部通道统一使用滑条上的频率偏好档。".to_string()
        });
    }

    /// 批量队列的每 tick 推进。由 `csd_diag_tick()` 调用。
    pub(super) fn _pump_ch_batch(&mut self) {
        let Some(kind) = self.ch_batch.kind else {
            return;
        };
        if self.io.is_none() {
            self.ch_batch.status = format!("{}: 连接已断开, 已中止", kind.label());
            self.push_log_warn(self.ch_batch.status.clone());
            self.ch_batch.clear();
            self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
            return;
        }
        match self.ch_batch.stage {
            ChBatchStage::AwaitEnabled => {
                if self.ch_enabled_states_known() {
                    if let Err(error) = self._ch_batch_begin_ready(kind) {
                        self.push_log_warn(error.to_string());
                        self.ch_batch.clear();
                    }
                }
            }
            ChBatchStage::Prime => self._ch_batch_prime(),
            ChBatchStage::PrimeVerify => self._ch_batch_prime_verify(),
            ChBatchStage::CancelPrime => self._ch_batch_cancel_prime(),
            ChBatchStage::Run => self._ch_batch_run(kind),
            ChBatchStage::CancelRun => self._ch_batch_cancel_run(),
            ChBatchStage::Done => {
                if !self._csd_locked() && !self._cfg_batch_pending(self.ch_batch.generation) {
                    self.ch_batch.clear();
                    self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
                }
            }
        }
    }

    /// 把全部目标通道的增益档一次排入既有窗口=1 FIFO；发送仍串行，但不再逐通道写后回读。
    fn _ch_batch_prime(&mut self) {
        if self._cfg_batch_pending(self.ch_batch.generation) {
            return;
        }
        let generation = self.ch_batch.generation;
        let targets = self.ch_batch.queue.clone();
        for ch in targets {
            let gain = self.ch_batch.gain[ch as usize] as u32;
            if let Err(error) =
                self._ch_batch_tx(|s| s._queue_batch_prime_param(generation, ch, gain))
            {
                self.ch_batch.res[ch as usize] = ChOpRes::SendFail;
                self.push_log_warn(format!("前置增益档下发失败: CH{} — {}", ch, error));
                self._cfg_purge_batch_frames(generation);
                self.ch_batch.stage = ChBatchStage::CancelPrime;
                return;
            }
        }
        self.ch_batch.prime_next = self.ch_batch.queue.len();
        self.ch_batch.verify_ticks = 0;
        self.ch_batch.verify_asked = false;
        self.ch_batch.stage = ChBatchStage::PrimeVerify;
        self.ch_batch.status = format!(
            "逐通道频率自适应: 已批量排入 {} 个增益档，等待统一回读…",
            self.ch_batch.queue.len()
        );
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
    }

    /// 全部前置写结算后只发一次全通道回读，并统一校验全部目标通道。
    fn _ch_batch_prime_verify(&mut self) {
        self.ch_batch.verify_ticks = self.ch_batch.verify_ticks.wrapping_add(1);
        if self._cfg_batch_pending(self.ch_batch.generation) {
            self._ch_batch_verify_guard();
            return;
        }
        if !self.ch_batch.verify_asked {
            self.ch_batch.verify_asked = true;
            for ch in self.ch_batch.queue.iter().copied() {
                self.params[ch as usize].remove(&PARAM_IDAC_GAIN);
            }
            if let Err(error) = self.request_param_all_channels(PARAM_IDAC_GAIN) {
                self.push_log_warn(format!("前置增益档统一回读失败: {}", error));
                self.ch_batch.verify_asked = false;
            }
            return;
        }
        let mut missing = Vec::new();
        for ch in self.ch_batch.queue.iter().copied() {
            let verified = self.params[ch as usize].get(&PARAM_IDAC_GAIN).copied()
                == Some(self.ch_batch.gain[ch as usize] as u32);
            self.ch_batch.verified[ch as usize] = verified;
            if !verified {
                missing.push(ch);
            }
        }
        if missing.is_empty() {
            self.ch_batch.stage = ChBatchStage::Run;
            self.ch_batch.status = format!(
                "逐通道频率自适应: {} 个通道增益档已统一确认，开始执行…",
                self.ch_batch.queue.len()
            );
            self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
            return;
        }
        if self.ch_batch.verify_ticks % 60 == 0 {
            self.ch_batch.verify_asked = false;
        }
        self._ch_batch_verify_guard();
    }

    /// 确认阶段的超时兜底: 停止新的 prime，精确清掉尚未发出的本批帧，待在途帧结算后统一回读真值。
    fn _ch_batch_verify_guard(&mut self) {
        if self.ch_batch.verify_ticks < PRIME_VERIFY_TIMEOUT_TICKS {
            return;
        }
        self.ch_batch.status =
            "逐通道频率自适应: 增益档统一回读未确认，正在停止并收敛在途帧。".to_string();
        self.push_log_warn(self.ch_batch.status.clone());
        self._cfg_purge_batch_frames(self.ch_batch.generation);
        self.ch_batch.stage = ChBatchStage::CancelPrime;
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
    }

    fn _ch_batch_cancel_prime(&mut self) {
        self._cfg_purge_batch_frames(self.ch_batch.generation);
        if self._cfg_batch_pending(self.ch_batch.generation) {
            return;
        }
        self._param_refetch_push(PARAM_IDAC_GAIN);
        self.ch_batch
            .status
            .push_str(" 前置写已收敛，已排程全通道设备真值回读。");
        self.ch_batch.clear();
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
    }

    fn _ch_batch_cancel_run(&mut self) {
        if self.ch_batch.inflight.is_some()
            || self._csd_locked()
            || self._cfg_batch_pending(self.ch_batch.generation)
        {
            return;
        }
        self.ch_batch
            .status
            .push_str(" 已发出的操作与冷却窗口均已收敛。");
        self.ch_batch.clear();
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
    }

    /// 批量拥有控制面时仅允许它自己的调用跨过 Host 守卫。
    pub(super) fn _batch_blocks_csd(&self) -> bool {
        self.ch_batch.kind.is_some() && !self.ch_batch.own_tx
    }

    fn _ch_batch_tx<T>(&mut self, f: impl FnOnce(&mut Self) -> T) -> T {
        self.ch_batch.own_tx = true;
        let out = f(self);
        self.ch_batch.own_tx = false;
        out
    }

    /// 主阶段: 设备空闲就发下一个通道。
    fn _ch_batch_run(&mut self, kind: ChBatchKind) {
        if self.ch_batch.next >= self.ch_batch.queue.len() {
            // ★最后一条的终态没落地就不收尾★ 否则末通道的 NAK / 自适应终态失败 / 看门狗超时全都赶不上
            // 完成文案, 只能被算成"未回执"。在途期间 `_csd_locked()` 本来就挡着下一条, 这里只是把同一
            // 个条件延续到收尾这一步。
            if self.ch_batch.inflight.is_none() && !self._csd_locked() {
                self._ch_batch_finish(kind);
            }
            return;
        }
        // ★等每通道真实完成/超时再发下一个★ `_csd_locked` 同时覆盖"op 在途"与"CSD 冷却窗",
        // 卡死由 csd_diag_tick 的既有看门狗兜底(到点解锁), 故这里不需要第二套超时。
        if self._csd_locked() {
            return;
        }
        let ch = self.ch_batch.queue[self.ch_batch.next];
        let pref = self.ch_batch.pref[ch as usize];
        let res = self._ch_batch_tx(|s| match kind {
            ChBatchKind::Calibrate => s.calibrate(1u64 << ch),
            ChBatchKind::BaselineReset => s.baseline_reset(1u64 << ch),
            ChBatchKind::AutoTune => s.auto_tune_with_pref_deferred(ch, pref),
        });
        // ★"确实发出去了"的唯一证据是 `_begin_op` 记下的 op seq★ 这三个入口在被串行化/互斥守卫拒绝时
        // 走的是既有约定"记日志 + 返回 Ok", 只看 Err 会把没发出去的那一条当成已发出并等一个永不到来的
        // 回执。拿到 seq 才登记在途, 否则当场记为未下发。
        let sent_seq = self.op_wait_seq;
        match (res, sent_seq) {
            (Ok(()), Some(seq)) => {
                self.ch_batch.res[ch as usize] = ChOpRes::Sent;
                self.ch_batch.inflight = Some(ChBatchOp { ch, seq });
            }
            (Ok(()), None) => {
                self.ch_batch.res[ch as usize] = ChOpRes::SendFail;
                self.push_log_warn(format!(
                    "{}: CH{} 未下发(设备忙或未连接), 已记为失败。",
                    kind.label(),
                    ch
                ));
            }
            (Err(e), _) => {
                self.ch_batch.res[ch as usize] = ChOpRes::SendFail;
                self.push_log_warn(format!("{}: CH{} 下发失败 — {}", kind.label(), ch, e));
            }
        }
        self.ch_batch.next += 1;
        let (ok, _pending, failed) = self.ch_batch._tally();
        self.ch_batch.status = format!(
            "{}: {}/{} (CH{}) · 成功 {}{}",
            kind.label(),
            self.ch_batch.next,
            self.ch_batch.queue.len(),
            ch,
            ok,
            if failed.is_empty() {
                String::new()
            } else {
                format!(" · 失败 {}", failed.len())
            }
        );
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
    }

    /// 把一条 op 的终态归因给当前批量通道。由既有回执/超时路径调用(ACK / NAK /
    /// 自适应终态帧 / 阻塞操作看门狗), **不新增任何发送或轮询路径**。
    ///
    /// ★seq 匹配是唯一判据★ 不匹配就不是本批量发出的那一条(可能是用户在别处点的操作), 一律不归因;
    /// 归因一次即清 `inflight`, 于是"NAK 之后又超时"这类重复事件不会被记两次。
    pub(super) fn _ch_batch_note_op(&mut self, seq: u8, ok: bool, why: &str) {
        let Some(kind) = self.ch_batch.kind else {
            return;
        };
        let Some(op) = self.ch_batch.inflight else {
            return;
        };
        if op.seq != seq {
            return;
        }
        self.ch_batch.inflight = None;
        if self.ch_batch.res[op.ch as usize] != ChOpRes::Sent {
            return; // 已有终态: 不重复计数
        }
        self.ch_batch.res[op.ch as usize] = if ok { ChOpRes::Ok } else { ChOpRes::Failed };
        if !ok {
            self.push_log_warn(format!("{}: CH{} 失败 — {}", kind.label(), op.ch, why));
        }
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
    }

    fn _ch_batch_finish(&mut self, kind: ChBatchKind) {
        let total = self.ch_batch.queue.len();
        // ★成功数只数"设备确认过的"★ 下发成功不算成功: 异步 NAK、op 超时、自适应终态失败都已由
        // `_ch_batch_note_op` 归因到具体通道, 这里如实汇总。
        let (ok, pending, failed) = self.ch_batch._tally();
        let names: Vec<String> = failed.iter().map(|ch| format!("CH{}", ch)).collect();
        self.ch_batch.status = if names.is_empty() && pending == 0 {
            format!(
                "{}: 已完成 {}/{}(逐通道均已收到设备回执)",
                kind.label(),
                ok,
                total
            )
        } else {
            format!(
                "{}: 成功 {}/{}{}{}",
                kind.label(),
                ok,
                total,
                if names.is_empty() {
                    String::new()
                } else {
                    format!(", 失败 {} 个({})", names.len(), names.join(" "))
                },
                if pending == 0 {
                    String::new()
                } else {
                    format!(", 未收到回执 {} 个(设备实际状态未知)", pending)
                }
            )
        };
        self.push_log(self.ch_batch.status.clone());
        // 自适应批次内每通道均延迟持久化；收尾只排入一次 SAVE_CONFIG，再统一回读分频。
        if kind == ChBatchKind::AutoTune {
            let generation = self.ch_batch.generation;
            if let Err(error) = self._queue_batch_save(generation) {
                self.push_log_warn(format!("自适应结果统一保存失败: {}", error));
            }
            if let Err(error) = self.request_param_all_channels(PARAM_SNS_CLK_DIV) {
                self.push_log_warn(format!("自适应后回读分频失败: {}", error));
            }
        }
        // 终态文案已形成，但必须继续持有控制面到最后一个 CSD 冷却/本批 cfg 帧完全结算。
        self.ch_batch.stage = ChBatchStage::Done;
        self.ch_batch.version = self.ch_batch.version.wrapping_add(1);
    }

    // ---------------- 噪声频谱扫描 ----------------

    pub fn noise_sweep_active(&self) -> bool {
        self.noise_sweep.active
    }
    pub fn noise_sweep_status(&self) -> &str {
        &self.noise_sweep.status
    }
    pub fn noise_sweep_version(&self) -> u64 {
        self.noise_sweep.version
    }
    pub fn noise_sweep_channel(&self) -> u8 {
        self.noise_sweep.ch
    }
    /// 设备会话已产出的结果格数；与主机收到/有效格数分开，供无头验收区分设备生产与链路交付。
    pub fn noise_sweep_produced(&self) -> u16 {
        self.noise_sweep.produced
    }
    /// 热图的不可变扫描通道。空值表示从未启动过扫描，调用方不得显示任何遗留格子。
    pub fn noise_sweep_result_channel(&self) -> Option<u8> {
        self.noise_sweep.result_ch
    }
    pub fn noise_sweep_cells(&self) -> &[NoiseCell] {
        &self.noise_sweep.cells
    }
    /// 完成比例 0.0..1.0。
    pub fn noise_sweep_progress(&self) -> f32 {
        self.noise_sweep._done() as f32 / SWEEP_CELLS as f32
    }
    /// 噪声最大的格子: `(gain, div, std)`; 没有任何有效格时返回 None。
    /// ★撞顶的格子不参与★: 那种点 RAW 贴在量程边界上, 标准差小得好看但 diff 已经失效。
    pub fn noise_sweep_peak(&self) -> Option<(u8, u16, f32)> {
        let mut best: Option<(u8, u16, f32)> = None;
        for (i, cell) in self.noise_sweep.cells.iter().enumerate() {
            if !cell.valid || cell.railed {
                continue;
            }
            let better = match best {
                Some((_, _, s)) => cell.std > s,
                None => true,
            };
            if better {
                best = Some((
                    (i / SWEEP_DIVS) as u8,
                    (i % SWEEP_DIVS + 1) as u16,
                    cell.std,
                ));
            }
        }
        best
    }
    /// 噪声最小的有效格子(用户真正想落到的点)。
    pub fn noise_sweep_best(&self) -> Option<(u8, u16, f32)> {
        let mut best: Option<(u8, u16, f32)> = None;
        for (i, cell) in self.noise_sweep.cells.iter().enumerate() {
            if !cell.valid || cell.railed {
                continue;
            }
            let better = match best {
                Some((_, _, s)) => cell.std < s,
                None => true,
            };
            if better {
                best = Some((
                    (i / SWEEP_DIVS) as u8,
                    (i % SWEEP_DIVS + 1) as u16,
                    cell.std,
                ));
            }
        }
        best
    }

    /// 开始扫描当前通道的响应噪声频谱。
    ///
    /// 前置条件必须硬性检查: ① 已连接 ② 逐通道 RAW 遥测在推流(否则根本采不到样本)
    /// ③ 没有其它 CSD 批量在跑。任一不满足就明确拒绝并说清原因, 不做"开了但一直 0 样本"的哑失败。
    pub fn noise_sweep_start(&mut self, ch: u8) -> anyhow::Result<()> {
        if (ch as usize) >= CH_COUNT {
            return Err(anyhow::anyhow!("频谱扫描通道越界: {}", ch));
        }
        if self.io.is_none() {
            return Err(anyhow::anyhow!("未连接, 无法扫描频谱"));
        }
        if self.noise_sweep.active {
            return Ok(());
        }
        if !self.ch_enabled(ch) {
            return Err(anyhow::anyhow!(
                "CH{} 已禁用(电极保持高阻, 不参与扫描), 无法测响应噪声频谱; 请先启用该通道",
                ch
            ));
        }
        if self.ch_batch.kind.is_some() {
            return Err(anyhow::anyhow!(
                "正在进行{}，请先等待完成或取消后再扫描频谱",
                self.ch_batch.kind.map(|k| k.label()).unwrap_or("批量操作")
            ));
        }
        let generation = self.noise_sweep.generation.wrapping_add(1);
        let expired_control_seqs = self.noise_sweep.expired_control_seqs.clone();
        let seq = self.next_seq();
        let frame = crate::proto::Frame::new(
            crate::proto::HostCmd::SweepStart as u8,
            0,
            seq,
            crate::proto::encode_sweep_start(ch, 8, 32),
        );
        self.io
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("未连接, 无法扫描频谱"))?
            .send(frame)?;
        self.noise_sweep = NoiseSweep {
            active: true,
            ch,
            generation,
            pending: Some(SweepPending { seq, generation }),
            expired_control_seqs,
            result_ch: Some(ch),
            status: format!("CH{} 频谱扫描: 已请求设备会话，等待响应…", ch),
            version: self.noise_sweep.version.wrapping_add(1),
            ..NoiseSweep::default()
        };
        self.push_log(format!(
            "CH{} 响应噪声频谱扫描已请求设备会话(seq={}): 设备负责 448 格改参、校准、采样与恢复。",
            ch, seq
        ));
        Ok(())
    }

    /// 频谱扫描是否应挡住外部 CSD 操作。扫描自身的下发(`_sweep_tx` 内)不挡, 否则它会把自己挡死。
    pub(super) fn _sweep_blocks_csd(&self) -> bool {
        self.noise_sweep.active && !self.noise_sweep.own_tx
    }

    /// 以"扫描自身"的身份执行一段下发: 期间互斥守卫放行, 无论成败都复位标记。
    fn _sweep_tx<T>(&mut self, f: impl FnOnce(&mut Self) -> T) -> T {
        self.noise_sweep.own_tx = true;
        let out = f(self);
        self.noise_sweep.own_tx = false;
        out
    }

    /// 取消扫描: 会话已建立则请求设备恢复；启动响应尚未到达时，响应归属后立即补发取消。
    pub fn noise_sweep_cancel(&mut self) {
        if !self.noise_sweep.active || self.noise_sweep.cancel {
            return;
        }
        self.noise_sweep.cancel = true;
        self.noise_sweep.status = format!(
            "CH{} 频谱扫描: 已请求取消，等待设备恢复原始增益档/分频…",
            self.noise_sweep.ch
        );
        self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
        if let Some(session) = self.noise_sweep.session {
            self._sweep_send_cancel(session);
        }
        self.push_log(self.noise_sweep.status.clone());
    }

    /// 设备会话不再由 host 逐格推进；host 只负责显式续租与补发控制。
    pub(super) fn _pump_noise_sweep(&mut self) {
        if !self.noise_sweep.active {
            return;
        }
        if self.io.is_none() {
            self._sweep_device_disconnected();
            return;
        }
        let Some(session) = self.noise_sweep.session else {
            return;
        };
        let due = self
            .noise_sweep
            .keepalive_at
            .is_none_or(|at| at.elapsed() >= std::time::Duration::from_secs(2));
        if !due {
            return;
        }
        let seq = self.next_seq();
        let sent = self.io.as_ref().is_some_and(|handle| {
            handle
                .send(crate::proto::Frame::new(
                    crate::proto::HostCmd::SweepCtrl as u8,
                    0,
                    seq,
                    crate::proto::encode_sweep_keepalive(session),
                ))
                .is_ok()
        });
        if sent {
            self.noise_sweep.keepalive_at = Some(std::time::Instant::now());
            self.noise_sweep.control_seqs.push(seq);
        } else {
            // 不推进时间戳：下一拍立即重试；真正断链由 IO 状态统一收敛。
            self.push_log_warn(format!("CH{} 频谱扫描续租下发失败", self.noise_sweep.ch));
        }
    }

    /// SWEEP_START 的响应只接受当前 host generation 的 pending seq，迟到会话立即取消。
    pub(super) fn _handle_sweep_start_response(&mut self, frame: &crate::proto::Frame) {
        let (session, total) = match crate::proto::decode_sweep_start(&frame.payload) {
            Ok(value) => value,
            Err(error) => {
                self.push_log_warn(format!("SWEEP_START 响应解析失败: {}", error));
                return;
            }
        };
        let current = self.noise_sweep.active
            && self.noise_sweep.pending.is_some_and(|pending| {
                pending.seq == frame.seq && pending.generation == self.noise_sweep.generation
            });
        if !current {
            self._sweep_send_cancel(session);
            return;
        }
        self.noise_sweep.pending = None;
        self.noise_sweep.session = Some(session);
        self.noise_sweep.keepalive_at = Some(std::time::Instant::now());
        self.noise_sweep.total = total;
        self.noise_sweep.produced = 0;
        self.noise_sweep.received = vec![false; SWEEP_CELLS];
        self.noise_sweep.resend_tries = vec![0; SWEEP_CELLS];
        self.noise_sweep.status = format!(
            "CH{} 频谱扫描: 设备会话 {} 已建立，等待 {}/{} 格结果…",
            self.noise_sweep.ch, session, 0, total
        );
        self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
        if total as usize != SWEEP_CELLS {
            self.push_log_warn(format!(
                "CH{} 频谱扫描: 设备声明 {} 格，主机热图固定为 {} 格；正在取消该异常会话。",
                self.noise_sweep.ch, total, SWEEP_CELLS
            ));
            self.noise_sweep.cancel = true;
        }
        if self.noise_sweep.cancel {
            self._sweep_send_cancel(session);
        }
    }

    /// 接收设备推送的结果/恢复/终态。不同 session 或通道的迟到帧一律不触碰当前热图。
    pub(super) fn _handle_sweep_data(&mut self, frame: &crate::proto::Frame) {
        let sweep = match crate::proto::decode_sweep_data(&frame.payload) {
            Ok(value) => value,
            Err(error) => {
                self.push_log_warn(format!("SWEEP_DATA 解析失败: {}", error));
                return;
            }
        };
        if !self.noise_sweep.active
            || self.noise_sweep.session != Some(sweep.session)
            || self.noise_sweep.ch != sweep.ch
        {
            return;
        }
        let produced_before = self.noise_sweep.produced;
        self.noise_sweep.produced = self
            .noise_sweep
            .produced
            .max(sweep.produced.min(sweep.total));
        if sweep.is_cell() && (sweep.index as usize) < SWEEP_CELLS {
            let idx = sweep.index as usize;
            self.noise_sweep.cells[idx] = NoiseCell {
                std: sweep.std(),
                pp: sweep.pp as f32,
                mean: sweep.mean as f32,
                samples: sweep.samples,
                flags: sweep.flags,
                gain: sweep.gain,
                div: sweep.div,
                fail_phase: sweep.fail_phase,
                valid: sweep.samples != 0
                    && sweep.flags
                        & (SWEEP_FLAG_CAL_FAIL | SWEEP_FLAG_MISMATCH | SWEEP_FLAG_STALLED)
                        == 0,
                railed: sweep.flags & SWEEP_FLAG_RAILED != 0,
            };
            self.noise_sweep.received[idx] = true;
        }
        if sweep.state.is_terminal() {
            self._sweep_finish_session(sweep);
            return;
        }
        if sweep.state == SweepState::Restoring {
            self.noise_sweep.status = format!(
                "CH{} 频谱扫描: 设备正在恢复原始增益档/分频 ({}/{})…",
                self.noise_sweep.ch,
                self.noise_sweep._done(),
                self.noise_sweep.total
            );
        } else {
            // 阶段码来自设备当刻状态机: 卡住时用户能直接看到停在哪一步, 不必等终态才知道。
            let phase = match sweep.phase {
                Some(phase) => format!("，设备阶段: {}", crate::proto::sweep_phase_text(phase)),
                None => String::new(),
            };
            self.noise_sweep.status = format!(
                "CH{} 频谱扫描: 已收 {}/{} 格（设备已产出 {}）{}",
                self.noise_sweep.ch,
                self.noise_sweep._done(),
                self.noise_sweep.total,
                self.noise_sweep.produced,
                phase
            );
        }
        self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
        if self.noise_sweep.produced > produced_before {
            self._sweep_resend_missing();
        }
    }

    /// 以最多 32 格的批次补发已产出但漏收的结果；同一格最多请求三次。
    fn _sweep_resend_missing(&mut self) {
        let Some(session) = self.noise_sweep.session else {
            return;
        };
        let limit = self.noise_sweep.produced as usize;
        let mut first = 0usize;
        while first < limit {
            while first < limit
                && (self.noise_sweep.received[first] || self.noise_sweep.resend_tries[first] >= 3)
            {
                first += 1;
            }
            if first == limit {
                break;
            }
            let mut count = 0usize;
            while first + count < limit
                && count < 32
                && !self.noise_sweep.received[first + count]
                && self.noise_sweep.resend_tries[first + count] < 3
            {
                count += 1;
            }
            if count == 0 {
                first += 1;
                continue;
            }
            let seq = self.next_seq();
            let send = self.io.as_ref().map(|handle| {
                handle.send(crate::proto::Frame::new(
                    crate::proto::HostCmd::SweepCtrl as u8,
                    0,
                    seq,
                    crate::proto::encode_sweep_resend(session, first as u16, count as u8),
                ))
            });
            match send {
                Some(Ok(())) => {
                    for idx in first..first + count {
                        self.noise_sweep.resend_tries[idx] += 1;
                    }
                    self.noise_sweep.control_seqs.push(seq);
                    first += count;
                }
                Some(Err(error)) => {
                    self.push_log_warn(format!(
                        "CH{} 频谱扫描补发请求失败(index={} count={}): {}",
                        self.noise_sweep.ch, first, count, error
                    ));
                    break;
                }
                None => {
                    self._sweep_device_disconnected();
                    break;
                }
            }
        }
    }

    fn _sweep_send_cancel(&mut self, session: u16) {
        let seq = self.next_seq();
        let send = self.io.as_ref().map(|handle| {
            handle.send(crate::proto::Frame::new(
                crate::proto::HostCmd::SweepCtrl as u8,
                0,
                seq,
                crate::proto::encode_sweep_cancel(session),
            ))
        });
        match send {
            Some(Ok(())) => self.noise_sweep.control_seqs.push(seq),
            Some(Err(error)) => self.push_log_warn(format!("频谱扫描取消下发失败: {}", error)),
            None => self._sweep_device_disconnected(),
        }
    }

    pub(super) fn _sweep_is_expired_control_seq(&self, seq: u8) -> bool {
        self.noise_sweep.expired_control_seqs.contains(&seq)
    }

    pub(super) fn _sweep_forget_expired_control_seq(&mut self, seq: u8) {
        self.noise_sweep
            .expired_control_seqs
            .retain(|expired| *expired != seq);
    }

    /// ACK/NAK 复用现有帧分发的 seq 归属。启动 NAK 结束本地会话；控制 NAK 保留会话等待设备终态。
    pub(super) fn _sweep_note_reply(&mut self, seq: u8, ok: bool, why: &str) -> bool {
        if self
            .noise_sweep
            .pending
            .is_some_and(|pending| pending.seq == seq)
        {
            if !ok {
                self.noise_sweep.active = false;
                self.noise_sweep.pending = None;
                self.noise_sweep.status =
                    format!("CH{} 频谱扫描启动被设备拒绝: {}", self.noise_sweep.ch, why);
                self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
            }
            return true;
        }
        if let Some(pos) = self
            .noise_sweep
            .control_seqs
            .iter()
            .position(|item| *item == seq)
        {
            self.noise_sweep.control_seqs.swap_remove(pos);
            if !ok {
                self.push_log_warn(format!(
                    "CH{} 频谱扫描控制命令被拒绝: {}",
                    self.noise_sweep.ch, why
                ));
            }
            return true;
        }
        if self._sweep_is_expired_control_seq(seq) {
            self._sweep_forget_expired_control_seq(seq);
            return true;
        }
        false
    }

    /// 断链时设备是否已完成恢复不可知；结果仍保留供 UI 查阅，但必须放开下一次会话。
    pub(super) fn _sweep_device_disconnected(&mut self) {
        if !self.noise_sweep.active {
            return;
        }
        self.noise_sweep.generation = self.noise_sweep.generation.wrapping_add(1);
        self.noise_sweep.active = false;
        self.noise_sweep.pending = None;
        self.noise_sweep
            .expired_control_seqs
            .append(&mut self.noise_sweep.control_seqs);
        self.noise_sweep.control_seqs.clear();
        self.noise_sweep.status = format!(
            "CH{} 频谱扫描: 连接已断开，设备恢复状态未知。",
            self.noise_sweep.ch
        );
        self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
        self.push_log_warn(self.noise_sweep.status.clone());
    }

    fn _sweep_finish_session(&mut self, sweep: SweepFrame) {
        let expected = self.noise_sweep._expected_total();
        let missing = self
            .noise_sweep
            .received
            .iter()
            .take(expected)
            .filter(|cell| !**cell)
            .count();
        let restore_failed = sweep.flags & 0x70 != 0 || sweep.state == SweepState::Failed;
        let state = sweep.state.text();
        // 设备回报的首个失败阶段: 有它才说得出"卡在哪一步", 没有就不编造。
        let phase_note = match sweep.fail_phase_text() {
            Some(text) => format!("；设备侧首个失败阶段: {}", text),
            None => String::new(),
        };
        // 已收到但无效的格数(设备明确报了这一格不可用): 与"没收到"分开计, 否则用户分不清是丢包还是坏点。
        let invalid = self
            .noise_sweep
            .received
            .iter()
            .take(expected)
            .zip(self.noise_sweep.cells.iter())
            .filter(|(received, cell)| **received && !cell.valid)
            .count();
        let invalid_note = if invalid == 0 {
            String::new()
        } else {
            format!("，其中 {} 格设备侧无有效结果", invalid)
        };
        self.noise_sweep.active = false;
        self.noise_sweep.pending = None;
        self.noise_sweep
            .expired_control_seqs
            .append(&mut self.noise_sweep.control_seqs);
        self.noise_sweep.control_seqs.clear();
        self.noise_sweep.status = if restore_failed {
            format!(
                "CH{} 频谱扫描{}: 已收 {}/{} 格{}，设备恢复状态未知(恢复标志 0x{:02X}){}。",
                self.noise_sweep.ch,
                state,
                self.noise_sweep._done(),
                self.noise_sweep.total,
                invalid_note,
                sweep.flags,
                phase_note
            )
        } else if missing != 0 {
            format!(
                "CH{} 频谱扫描{}: 已收 {}/{} 格{}，仍缺 {} 格(补发已达上限){}。",
                self.noise_sweep.ch,
                state,
                self.noise_sweep._done(),
                self.noise_sweep.total,
                invalid_note,
                missing,
                phase_note
            )
        } else {
            format!(
                "CH{} 频谱扫描{}: 已收 {}/{} 格{}，设备已完成恢复{}。",
                self.noise_sweep.ch,
                state,
                self.noise_sweep._done(),
                self.noise_sweep.total,
                invalid_note,
                phase_note
            )
        };
        self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
        if restore_failed {
            self.push_log_warn(self.noise_sweep.status.clone());
        } else {
            self.push_log(self.noise_sweep.status.clone());
        }
    }

    /// 本格配置: 写 gain/div → 排空配置队列并按**设备真值**确认 → 单通道校准并等**终态回执**。
    /// 三步全部确认才进 Settle/Sample。
    ///
    /// ★为什么每一格都得确认★ 旧实现是 `debug_param_now(gain).and_then(div).and_then(calibrate)`:
    /// 前两条只是排进串行写队列(还乐观写了本地缓存), 校准的返回值也只代表"已入队"。于是 PARAM_SET
    /// 被 NAK、排队滞后、或校准还没做完时, 采到的 RAW 属于**上一格**的配置, 热图那一格却标着这一格的
    /// (gain, div) —— 整张图与标签错位, 而这正是这张图唯一的用处。
    /// 任一步失败或有界超时 ⇒ 本格标无效、**不采样**、直接进下一格(单通道校准也不用 APPLY: 那会
    /// 重初始化全部 36 通道并复位所有基线, 与"只扫这一个通道"矛盾)。
    fn _sweep_configure(&mut self) {
        let (gain, div) = (self.noise_sweep.gain, self.noise_sweep.div);
        let req = ApplyReq {
            gain: Some(gain as u32),
            div: Some(div as u32),
            restoring: false,
        };
        match self._sweep_apply(req) {
            ApplyOut::Pending => {}
            ApplyOut::Done => {
                self.noise_sweep.phase = SweepPhase::Settle;
                self.noise_sweep.phase_ticks = 0;
                self.noise_sweep.samples.clear();
                self.noise_sweep.last_t_us = None;
            }
            // 根因已由 `_sweep_note_fail` 落日志(含本格坐标与"标为无效"结论), 这里只落结果。
            ApplyOut::Fail(_) => {
                self.noise_sweep.cells[NoiseSweep::_idx(gain, div)] = NoiseCell::default();
                self._sweep_advance();
            }
        }
    }

    /// 等设备把校准做完(op/冷却解锁)+ 额外稳定窗。
    fn _sweep_settle(&mut self) {
        if self._csd_locked() {
            return;
        }
        self.noise_sweep.phase_ticks = self.noise_sweep.phase_ticks.wrapping_add(1);
        if self.noise_sweep.phase_ticks < SWEEP_SETTLE_TICKS {
            return;
        }
        // 采样起点锚在"当前最新样本": 校准期间的样本一律不计入。
        self.noise_sweep.last_t_us = self.telem_buf[self.noise_sweep.ch as usize]
            .back()
            .map(|s| s.t_us);
        self.noise_sweep.phase = SweepPhase::Sample;
        self.noise_sweep.phase_ticks = 0;
    }

    /// 采 RAW 样本; 采满窗口即结算本格。
    fn _sweep_sample(&mut self) {
        self._sweep_drain_raw();
        self.noise_sweep.phase_ticks = self.noise_sweep.phase_ticks.wrapping_add(1);
        if self.noise_sweep.phase_ticks < SWEEP_SAMPLE_TICKS {
            return;
        }
        let cell = _stats_of(&self.noise_sweep.samples, self.noise_sweep.full_scale);
        let idx = NoiseSweep::_idx(self.noise_sweep.gain, self.noise_sweep.div);
        self.noise_sweep.cells[idx] = cell;
        self._sweep_advance();
    }

    /// 把缓冲里"比上次消费点更新"的 RAW 取出来。★按设备时间戳去重★ 16ms 的 tick 与 100Hz 的
    /// 推流不同步, 直接取 `back()` 会漏样本也会重复计同一个样本。
    fn _sweep_drain_raw(&mut self) {
        let ch = self.noise_sweep.ch as usize;
        let mark = self.noise_sweep.last_t_us;
        let mut fresh: Vec<u16> = Vec::new();
        let mut newest: Option<u32> = None;
        for s in self.telem_buf[ch].iter().rev() {
            if Some(s.t_us) == mark {
                break;
            }
            if newest.is_none() {
                newest = Some(s.t_us);
            }
            if let Some(raw) = s.raw {
                fresh.push(raw);
            }
            if fresh.len() >= 256 {
                break; // 兜底: 不让一次 drain 变成整缓冲遍历
            }
        }
        if let Some(t) = newest {
            self.noise_sweep.last_t_us = Some(t);
        }
        fresh.reverse();
        self.noise_sweep.samples.extend(fresh);
    }

    /// 推进到下一格; 全部扫完(或用户已取消)则进入还原。
    fn _sweep_advance(&mut self) {
        if self.noise_sweep.cancel {
            self._sweep_enter_restore();
            return;
        }
        if (self.noise_sweep.div as usize) < SWEEP_DIVS {
            self.noise_sweep.div += 1;
        } else if (self.noise_sweep.gain as usize) + 1 < SWEEP_GAINS {
            self.noise_sweep.gain += 1;
            self.noise_sweep.div = 1;
        } else {
            self._sweep_enter_restore();
            self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
            return;
        }
        self.noise_sweep.phase = SweepPhase::Configure;
        self.noise_sweep.phase_ticks = 0;
        // 新的一格 = 新一轮确认: 上一格的步骤/计时/根因都不许带过来。
        self.noise_sweep.ap = SweepApply::default();
        let done = self.noise_sweep._done();
        self.noise_sweep.status = format!(
            "CH{} 频谱扫描: {}/{} (增益档 {} · 分频 ÷{})",
            self.noise_sweep.ch, done, SWEEP_CELLS, self.noise_sweep.gain, self.noise_sweep.div
        );
        self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
    }

    /// 进入还原阶段(取消 / 扫完 / 出错都由此进入): 确认状态机每次都从头开始。
    /// ★也丢掉本格在途的确认★ 取消时那一格已经不要了, 它的在途校准回执不再归因(`ap.op` 已清),
    /// 设备侧的忙闲仍由 `_csd_locked()` / 既有看门狗照管, 还原链的第一步本来就要等它空闲。
    fn _sweep_enter_restore(&mut self) {
        self.noise_sweep.phase = SweepPhase::Restore;
        self.noise_sweep.phase_ticks = 0;
        self.noise_sweep.ap = SweepApply::default();
    }

    /// 还原: 写回原值 → **按设备真值回读确认** → 单通道校准 → 基线复位。
    /// 完成/取消/配置出错都走这里, 且 `active` 全程保持为 true —— 外部 CSD 互斥直到本状态机收敛
    /// (每步确认成功, 或到有界超时后如实标记"恢复未确认")才放开。
    fn _sweep_restore(&mut self) {
        let req = ApplyReq {
            gain: self.noise_sweep.saved_gain,
            div: self.noise_sweep.saved_div,
            restoring: true,
        };
        match self._sweep_apply(req) {
            ApplyOut::Pending => {}
            // 成败都在这里收尾: `_sweep_finish` 按 `ap.fail` 决定是否允许声称"已还原"。
            ApplyOut::Done | ApplyOut::Fail(_) => self._sweep_finish(),
        }
    }

    /// 每格配置与扫完还原**共用**的推进: 写入 → 排空并按设备真值确认 → 单通道校准[→ 基线复位]。
    /// 只在这里实现一次, 于是两条路上的"确认"是同一个含义, 不会一处严一处松。
    fn _sweep_apply(&mut self, req: ApplyReq) -> ApplyOut {
        let out = match self.noise_sweep.ap.step {
            ApplyStep::Write => self._sweep_ap_write(req),
            ApplyStep::Verify => self._sweep_ap_verify(req),
            ApplyStep::Calibrate | ApplyStep::Baseline => self._sweep_ap_op(req),
        };
        // 写入下发报错时链条仍会往下走(设备真值才是判据), 所以收敛这一刻要把那个根因带出来:
        // 记过失败就不许声称成功。
        match out {
            ApplyOut::Done => match self.noise_sweep.ap.fail.clone() {
                Some(why) => ApplyOut::Fail(why),
                None => ApplyOut::Done,
            },
            other => other,
        }
    }

    /// 记下本轮推进的**第一个**失败原因(后续原因只落日志: 第一个才是根因)。
    /// 由 `_sweep_ap_fail` 与既有回执归因 `_sweep_note_op` 共用。
    fn _sweep_note_fail(&mut self, restoring: bool, why: String) {
        let tag = if restoring {
            "还原".to_string()
        } else {
            format!(
                "本格配置(增益档 {} · 分频 ÷{})",
                self.noise_sweep.gain, self.noise_sweep.div
            )
        };
        // 每格配置失败的唯一后果就是弃格, 写在同一行里, 免得再补第二条日志。
        let tail = if restoring {
            ""
        } else {
            " ⇒ 该点标为无效并跳过采样, 继续下一格。"
        };
        self.push_log_warn(format!(
            "CH{} 频谱扫描{}: {}{}",
            self.noise_sweep.ch, tag, why, tail
        ));
        if self.noise_sweep.ap.fail.is_none() {
            self.noise_sweep.ap.fail = Some(why);
        }
    }

    /// 记失败并把它作为本次推进的结果返回。
    fn _sweep_ap_fail(&mut self, req: ApplyReq, why: String) -> ApplyOut {
        self._sweep_note_fail(req.restoring, why.clone());
        ApplyOut::Fail(why)
    }

    /// 步骤①: 写目标值。等设备把上一条 CSD 操作与冷却窗走完再写 —— 否则 PARAM_SET 撞进 heavy-op 窗口。
    fn _sweep_ap_write(&mut self, req: ApplyReq) -> ApplyOut {
        self.noise_sweep.ap.ticks = self.noise_sweep.ap.ticks.wrapping_add(1);
        if self._csd_locked() {
            if self.noise_sweep.ap.ticks >= req._op_timeout() {
                let why = format!("设备持续忙, {}未能下发", ApplyStep::Write._label());
                return self._sweep_ap_fail(req, why);
            }
            return ApplyOut::Pending;
        }
        let ch = self.noise_sweep.ch;
        let (gain, div) = (req.gain, req.div);
        let write = self._sweep_tx(|s| {
            let mut r = Ok(());
            if let Some(g) = gain {
                r = r.and_then(|()| s.debug_param_now(ch, PARAM_IDAC_GAIN, g));
            }
            if let Some(d) = div {
                r = r.and_then(|()| s.debug_param_now(ch, PARAM_SNS_CLK_DIV, d));
            }
            r
        });
        if let Err(e) = write {
            // ★这里不收敛★ 设备真值才是判据: 两条里只成了一条时, 只有回读能说清成了哪条。
            self._sweep_note_fail(
                req.restoring,
                format!("{}下发失败 — {}", ApplyStep::Write._label(), e),
            );
        }
        self.noise_sweep.ap.step = ApplyStep::Verify;
        self.noise_sweep.ap.ticks = 0;
        self.noise_sweep.ap.asked = false;
        if req.restoring {
            self.noise_sweep.status = format!(
                "CH{} 频谱扫描: 正在写回原始增益档/分频并等待设备回读确认…",
                ch
            );
            self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
        }
        ApplyOut::Pending
    }

    /// 步骤②: 排空配置发送队列 → 清乐观缓存 → 回读比对**设备真值**。
    /// ★必须先清本地缓存★ `debug_param_now` 排队后会乐观把值写进 `self.params`, 不清就是拿自己
    /// 写的值确认自己 —— PARAM_SET 被 NAK 也照样"确认通过"。清空后这两项唯一来源就是设备响应。
    fn _sweep_ap_verify(&mut self, req: ApplyReq) -> ApplyOut {
        self.noise_sweep.ap.ticks = self.noise_sweep.ap.ticks.wrapping_add(1);
        // 写队列没排空: 回读会读到半新半旧的中间态。
        if !self.cfg_tx_queue.is_empty() || self.cfg_tx_inflight.is_some() {
            return self._sweep_ap_verify_guard(req);
        }
        let ch = self.noise_sweep.ch;
        if !self.noise_sweep.ap.asked {
            if self._device_busy() || self.io.is_none() {
                return self._sweep_ap_verify_guard(req);
            }
            self.params[ch as usize].remove(&PARAM_IDAC_GAIN);
            self.params[ch as usize].remove(&PARAM_SNS_CLK_DIV);
            let mut sent = true;
            for id in [PARAM_IDAC_GAIN, PARAM_SNS_CLK_DIV] {
                if let Err(e) = self.request_param(ch, id) {
                    sent = false;
                    self.push_log_warn(format!(
                        "频谱扫描{}回读下发失败: 0x{:02X} — {}",
                        req._what(),
                        id,
                        e
                    ));
                }
            }
            if sent {
                self.noise_sweep.ap.asked = true;
            }
            return ApplyOut::Pending;
        }
        let got_g = self.params[ch as usize].get(&PARAM_IDAC_GAIN).copied();
        let got_d = self.params[ch as usize].get(&PARAM_SNS_CLK_DIV).copied();
        let hit_g = match req.gain {
            Some(w) => got_g == Some(w),
            None => true, // 无目标 ⇒ 本项没什么可确认的(原值缺失时不凭空写也就不必回读)
        };
        let hit_d = match req.div {
            Some(w) => got_d == Some(w),
            None => true,
        };
        if hit_g && hit_d {
            self.noise_sweep.ap.step = ApplyStep::Calibrate;
            self.noise_sweep.ap.ticks = 0;
            self.noise_sweep.ap.asked = false;
            if req.restoring {
                self.noise_sweep.status =
                    format!("CH{} 频谱扫描: 原值已按设备真值确认, 正在重新校准…", ch);
                self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
            }
            return ApplyOut::Pending;
        }
        // 每 ~1s 再问一次: 单条回读可能被别的响应挤掉, 要能自愈。
        if self.noise_sweep.ap.ticks % 60 == 0 {
            self.noise_sweep.ap.asked = false;
        }
        self._sweep_ap_verify_guard(req)
    }

    /// 回读确认的有界兜底: 到点如实报"设备真值与目标不一致 / 未回读到"。
    /// ★不接着校准★ 配置本身都没确认下去, 再校准一次只会让"这一格到底是什么配置"更说不清 ——
    /// 还原链上是"到底还原了吗", 每格配置上是"这一格的噪声到底属于谁"。
    fn _sweep_ap_verify_guard(&mut self, req: ApplyReq) -> ApplyOut {
        if self.noise_sweep.ap.ticks < req._verify_timeout() {
            return ApplyOut::Pending;
        }
        let ch = self.noise_sweep.ch as usize;
        let why = format!(
            "{}未能按设备真值确认: 期望(增益档 {:?}, 分频 {:?}), 设备回读(增益档 {:?}, 分频 {:?}) \
             —— PARAM_SET 可能被 NAK 或无回执{}",
            ApplyStep::Verify._label(),
            req.gain,
            req.div,
            self.params[ch].get(&PARAM_IDAC_GAIN).copied(),
            self.params[ch].get(&PARAM_SNS_CLK_DIV).copied(),
            if req.restoring {
                ", 该通道仍可能停在扫描配置上。"
            } else {
                "。"
            }
        );
        self._sweep_ap_fail(req, why)
    }

    /// 步骤③/④: 单通道校准 →(仅还原链)基线复位。两步同构: 等设备空闲 → 下发 → 等**终态回执**。
    fn _sweep_ap_op(&mut self, req: ApplyReq) -> ApplyOut {
        self.noise_sweep.ap.ticks = self.noise_sweep.ap.ticks.wrapping_add(1);
        let step = self.noise_sweep.ap.step;
        // 设备终态已由 `_sweep_note_op` 从既有 ACK/NAK/看门狗路径归因过来。
        if let Some(ok) = self.noise_sweep.ap.op_res.take() {
            if !ok {
                // 根因(设备 NAK 原因 / 看门狗超时)已在 `_sweep_note_op` 里记下。
                let why = self
                    .noise_sweep
                    .ap
                    .fail
                    .clone()
                    .unwrap_or_else(|| format!("{}未成功", step._label()));
                return ApplyOut::Fail(why);
            }
            if step == ApplyStep::Calibrate && req.restoring {
                self.noise_sweep.ap.step = ApplyStep::Baseline;
                self.noise_sweep.ap.ticks = 0;
                return ApplyOut::Pending;
            }
            return ApplyOut::Done;
        }
        if self.noise_sweep.ap.op.is_some() {
            if self.noise_sweep.ap.ticks >= req._op_timeout() {
                self.noise_sweep.ap.op = None;
                let why = format!(
                    "{}未在 {}s 内收到完成回执(设备实际状态未知)",
                    step._label(),
                    req._op_timeout() / 62
                );
                return self._sweep_ap_fail(req, why);
            }
            return ApplyOut::Pending;
        }
        // 还没下发: 等上一步的 op / 冷却窗结束(有界)。
        if self._csd_locked() {
            if self.noise_sweep.ap.ticks >= req._op_timeout() {
                let why = format!("设备持续忙, {}未能下发", step._label());
                return self._sweep_ap_fail(req, why);
            }
            return ApplyOut::Pending;
        }
        let mask = 1u64 << self.noise_sweep.ch;
        let sent = self._sweep_tx(|s| match step {
            ApplyStep::Baseline => s.baseline_reset(mask),
            _ => s.calibrate(mask),
        });
        // ★以 `op_wait_seq` 为"确实发出去了"的唯一证据★: 被串行化守卫拒绝时上面返回的也是 Ok(())。
        match (sent, self.op_wait_seq) {
            (Ok(()), Some(seq)) => {
                self.noise_sweep.ap.op = Some(seq);
                self.noise_sweep.ap.ticks = 0;
                ApplyOut::Pending
            }
            (r, _) => {
                let why = match r {
                    Err(e) => format!("{}下发失败 — {}", step._label(), e),
                    Ok(()) => format!("{}未能下发(已被串行化守卫拒绝)", step._label()),
                };
                self._sweep_ap_fail(req, why)
            }
        }
    }

    /// 把一条 CSD op 的终态归因给确认链上在途的那一步(每格配置的校准 / 还原链的校准与基线)。
    /// 由既有 ACK / NAK / 阻塞操作看门狗路径调用, **不新增任何发送或轮询路径**。
    /// seq 不匹配 ⇒ 不是本状态机发出的那一条, 一律不归因; 归因一次即清 `op`, 于是"NAK 之后又超时"
    /// 不会被记两次。取消时 `ap` 被重置(`op` 清空), 那一格在途的回执自然不再归因给还原链。
    pub(super) fn _sweep_note_op(&mut self, seq: u8, ok: bool, why: &str) {
        if !self.noise_sweep.active || self.noise_sweep.ap.op != Some(seq) {
            return;
        }
        self.noise_sweep.ap.op = None;
        self.noise_sweep.ap.op_res = Some(ok);
        if !ok {
            let restoring = self.noise_sweep.phase == SweepPhase::Restore;
            let why = format!("{}未成功 — {}", self.noise_sweep.ap.step._label(), why);
            self._sweep_note_fail(restoring, why);
        }
    }

    /// 收尾: 标记扫描结束(放开外部 CSD 互斥)并写最终状态。
    /// ★只有还原链每步都拿到设备确认(`ap.fail` 为 None)才允许声称已还原★ 否则明确报"恢复未确认"。
    fn _sweep_finish(&mut self) {
        self.noise_sweep.active = false;
        self.noise_sweep.ap.op = None;
        self.noise_sweep.ap.op_res = None;
        let ch = self.noise_sweep.ch;
        let done = self.noise_sweep._done();
        let head = if self.noise_sweep.cancel {
            "已取消"
        } else {
            "已完成"
        };
        let failed = self.noise_sweep.ap.fail.clone();
        let mut text = match &failed {
            None => format!(
                "CH{} 频谱扫描{}: 已测 {}/{} 点; 原值(增益档 {:?}, 分频 {:?})已按设备真值确认写回, \
                 单通道校准与基线复位均已收到设备完成回执。",
                ch,
                head,
                done,
                SWEEP_CELLS,
                self.noise_sweep.saved_gain,
                self.noise_sweep.saved_div
            ),
            Some(why) => format!(
                "CH{} 频谱扫描{}, 但★恢复未确认★: {} 请以设备真值为准核对该通道的增益档/分频, \
                 并手动点\"校准\"/\"基线\"确认恢复。已测 {}/{} 点。",
                ch, head, why, done, SWEEP_CELLS
            ),
        };
        if let Some((g, d, s)) = self.noise_sweep_peak() {
            text.push_str(&format!(
                " 噪声最大: 增益档 {} · 分频 ÷{} (RAW 标准差 {:.1})",
                g, d, s
            ));
        }
        if let Some((g, d, s)) = self.noise_sweep_best() {
            text.push_str(&format!(
                "; 噪声最小: 增益档 {} · 分频 ÷{} (RAW 标准差 {:.1})",
                g, d, s
            ));
        }
        self.noise_sweep.status = text;
        self.noise_sweep.version = self.noise_sweep.version.wrapping_add(1);
        let msg = self.noise_sweep.status.clone();
        if failed.is_some() {
            self.push_log_warn(msg);
        } else {
            self.push_log(msg);
        }
    }

    // ---------------- 绘图视窗冻结 ----------------

    /// 绘图视窗是否被冻结(遥测仍在收)。
    pub fn plot_frozen(&self) -> bool {
        self.plot_freeze.is_some()
    }

    /// 冻结绘图视窗: 快照当下的样本与时钟锚点, 时间轴自此不再走。
    /// ★不停数据源★ 与 `telem_user_stop()` 是两件不同的事: 那条真的停流(设备不再推), 这条只冻结
    /// "看到的东西" —— 需求要的是后者(停止后仍要继续收, 只是图不动)。
    pub fn plot_freeze(&mut self, why: &str) {
        if self.plot_freeze.is_some() {
            return;
        }
        self.plot_freeze = Some(PlotFreeze {
            buf: self.telem_buf.clone(),
            acc_us: self.telem_clock.acc_us,
            prev_raw: self.telem_clock.prev_raw,
            algo_report: self.algo_trace_report.clone(),
            algo_active: self.algo_trace_active.clone(),
        });
        self.plot_version = self.plot_version.wrapping_add(1);
        self.push_log(format!(
            "绘图视窗已冻结({}) —— 设备遥测继续接收, 只是时间轴与曲线停在此刻; 点「继续」恢复。",
            why
        ));
    }

    /// 解冻: 回到实时视窗。
    pub fn plot_unfreeze(&mut self) {
        if self.plot_freeze.take().is_none() {
            return;
        }
        self.plot_version = self.plot_version.wrapping_add(1);
        self.push_log("绘图视窗已解冻, 恢复实时时间轴。".to_string());
    }
}

/// 一组 RAW 样本的噪声统计。样本不足时返回 `valid=false`(不用两三个点硬算标准差冒充结果)。
fn _stats_of(samples: &[u16], full_scale: f32) -> NoiseCell {
    let n = samples.len();
    if n < SWEEP_SAMPLE_MIN {
        return NoiseCell {
            samples: n as u16,
            ..Default::default()
        };
    }
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    let mut sum = 0.0f32;
    for v in samples {
        let x = *v as f32;
        sum += x;
        if x < lo {
            lo = x;
        }
        if x > hi {
            hi = x;
        }
    }
    let mean = sum / n as f32;
    let mut var = 0.0f32;
    for v in samples {
        let d = *v as f32 - mean;
        var += d * d;
    }
    // 样本标准差(n-1): 这里是有限样本估计, 用 n 会系统性低估噪声。
    let std = (var / (n as f32 - 1.0)).sqrt();
    NoiseCell {
        std,
        pp: hi - lo,
        mean,
        samples: n as u16,
        flags: 0,
        gain: 0,
        div: 0,
        fail_phase: None,
        valid: true,
        // 按冻结的设备分辨率计算 rail：量程两端各留约 1%（至少 1 count）余量。
        railed: hi >= full_scale - _rail_margin(full_scale) || lo <= _rail_margin(full_scale),
    }
}
