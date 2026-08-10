//! 虚拟摄像头 (QR 登录桥接)
//!
//! 目的: 把 HID 键盘扫码器输入的字符串转换成游戏可用摄像头扫描的 QR 码画面, 实现扫码登录。
//!
//! 流程:
//!   1. 全局键盘钩子捕获扫码器(作为 HID 键盘)输入的字符, 累积到缓冲。
//!   2. 输入停顿超过 `submit_timeout_ms` 或按下 Enter → 提交该串数据(每串只用一次)。
//!   3. 用提交的数据生成 QR 帧(居中占帧 75%, 其余黑底), 推入虚拟摄像头显示 `display_ms`(默认 10s)。
//!   4. 显示期满 → 转黑屏, 等待下一次提交。
//!
//! 本模块的"核心"(QR 生成 + 键盘捕获 + 时序状态机 + 帧缓冲)与平台无关且可独立验证。
//! 出画链路分两块: `share` 是共享帧队列的**生产者**(RGB24 → NV12, 唯一生产者就是本进程);
//! `backend` 只管把自研 DirectShow 源过滤器 DLL(见 `vcam_source_cpp/`)部署注册到系统 ——
//! 过滤器本身活在**消费端进程**(游戏/播放器)里, 从队列取帧, 与本进程无 COM 耦合。

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, AtomicU32, Ordering},
};
use std::time::{Duration, Instant};

pub mod backend;
mod interception;
pub mod keyboard;
pub mod share;

/// 输出帧尺寸(RGB24)。多数摄像头消费端支持 640x480@RGB。
pub const FRAME_W: usize = 640;
pub const FRAME_H: usize = 480;
/// QR 占帧比例(min(W,H) 的 75%)。
const QR_FILL: f32 = 0.75;

/// 一帧 RGB24 数据(FRAME_W*FRAME_H*3), 供预览与虚拟摄像头后端共享。
pub type Rgb24 = Vec<u8>;

/// 虚拟摄像头运行时状态(线程安全共享)。
pub struct VcamState {
    /// 当前对外输出帧(RGB24)。默认全黑。
    frame: Mutex<Rgb24>,
    /// 帧版本号, 每次帧更新 +1, 供预览/后端判断是否变化。
    frame_version: AtomicU32,
    /// 是否启用(启用后键盘钩子生效并产生 QR 帧)。
    enabled: AtomicBool,
    /// 当前 QR 显示截止时刻(Some=显示中, None=黑屏)。
    show_until: Mutex<Option<Instant>>,
    /// 最近一次提交的数据(用于 UI 展示; 已消费)。
    last_data: Mutex<String>,
    /// 显示时长(ms)。
    display_ms: AtomicU32,
    /// 提交停顿阈值(ms): 键盘输入停顿超过此值自动提交。
    submit_timeout_ms: AtomicU32,
}

impl VcamState {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            frame: Mutex::new(black_frame()),
            frame_version: AtomicU32::new(1),
            enabled: AtomicBool::new(false),
            show_until: Mutex::new(None),
            last_data: Mutex::new(String::new()),
            display_ms: AtomicU32::new(10_000),
            submit_timeout_ms: AtomicU32::new(1500),
        })
    }

    pub fn set_enabled(&self, on: bool) {
        self.enabled.store(on, Ordering::SeqCst);
        if !on {
            // 关闭即转黑屏并清显示态。
            *self.show_until.lock().unwrap() = None;
            self.publish(black_frame());
        }
    }
    pub fn enabled(&self) -> bool {
        self.enabled.load(Ordering::SeqCst)
    }
    pub fn set_display_ms(&self, ms: u32) {
        self.display_ms.store(ms.max(500), Ordering::SeqCst);
    }
    pub fn set_submit_timeout_ms(&self, ms: u32) {
        self.submit_timeout_ms.store(ms.max(200), Ordering::SeqCst);
    }
    pub fn submit_timeout_ms(&self) -> u32 {
        self.submit_timeout_ms.load(Ordering::SeqCst)
    }
    pub fn frame_version(&self) -> u32 {
        self.frame_version.load(Ordering::SeqCst)
    }
    pub fn last_data(&self) -> String {
        self.last_data.lock().unwrap().clone()
    }
    /// 拷贝当前帧(供后端/预览)。
    pub fn frame_copy(&self) -> Rgb24 {
        self.frame.lock().unwrap().clone()
    }

    fn publish(&self, f: Rgb24) {
        *self.frame.lock().unwrap() = f;
        self.frame_version.fetch_add(1, Ordering::SeqCst);
    }

    /// 提交一串扫码数据: 生成 QR 帧, 设显示截止, 记录 last_data(每串只用一次: 由调用方保证提交后清缓冲)。
    pub fn submit_data(&self, data: &str) {
        if !self.enabled() || data.is_empty() {
            return;
        }
        let frame = match render_qr_frame(data) {
            Ok(f) => f,
            Err(_) => black_frame(),
        };
        *self.last_data.lock().unwrap() = data.to_string();
        let dur = Duration::from_millis(self.display_ms.load(Ordering::SeqCst) as u64);
        *self.show_until.lock().unwrap() = Some(Instant::now() + dur);
        self.publish(frame);
        log::info!(
            "虚拟摄像头: 已提交数据(长度 {}) → 显示 QR {} 秒",
            data.len(),
            self.display_ms.load(Ordering::SeqCst) / 1000
        );
    }

    /// 周期调用(定时器): 显示期满则转黑屏。返回帧是否发生变化。
    pub fn tick(&self) {
        let expired = {
            let mut g = self.show_until.lock().unwrap();
            match *g {
                Some(t) if Instant::now() >= t => {
                    *g = None;
                    true
                }
                _ => false,
            }
        };
        if expired {
            self.publish(black_frame());
            log::info!("虚拟摄像头: QR 显示到期 → 黑屏");
        }
    }
}

/// 全黑 RGB24 帧。
fn black_frame() -> Rgb24 {
    vec![0u8; FRAME_W * FRAME_H * 3]
}

/// 用数据生成 QR 帧: 黑底, 中央白色 QR(含静区)方块, 占 min(W,H) 的 75%, 居中。
pub fn render_qr_frame(data: &str) -> anyhow::Result<Rgb24> {
    use qrcode::{EcLevel, QrCode};
    let code = QrCode::with_error_correction_level(data.as_bytes(), EcLevel::M)
        .map_err(|e| anyhow::anyhow!("QR 生成失败: {:?}", e))?;
    let modules = code.width(); // QR 模块边长(不含静区)
    let colors = code.to_colors(); // 行优先, Dark/Light

    let mut frame = black_frame();

    // QR 方块像素边长 = 75% * min(W,H)。
    let side_px = ((FRAME_W.min(FRAME_H) as f32) * QR_FILL) as usize;
    // 含 4 模块静区(标准最小静区), 总模块数 = modules + 8。
    let quiet = 4usize;
    let total_modules = modules + quiet * 2;
    // 每模块像素(至少 1)。
    let mpx = (side_px / total_modules).max(1);
    let qr_px = mpx * total_modules; // 实际方块像素(整数模块对齐)
    let ox = (FRAME_W - qr_px) / 2;
    let oy = (FRAME_H - qr_px) / 2;

    for my in 0..total_modules {
        for mx in 0..total_modules {
            // 静区(白)与数据模块。
            let dark = if mx < quiet || my < quiet || mx >= quiet + modules || my >= quiet + modules
            {
                false // 静区为白
            } else {
                let cmx = mx - quiet;
                let cmy = my - quiet;
                matches!(colors[cmy * modules + cmx], qrcode::Color::Dark)
            };
            let (r, g, b) = if dark {
                (0u8, 0u8, 0u8)
            } else {
                (255u8, 255u8, 255u8)
            };
            // 填充该模块的 mpx*mpx 像素块。
            for py in 0..mpx {
                for px in 0..mpx {
                    let x = ox + mx * mpx + px;
                    let y = oy + my * mpx + py;
                    let idx = (y * FRAME_W + x) * 3;
                    frame[idx] = r;
                    frame[idx + 1] = g;
                    frame[idx + 2] = b;
                }
            }
        }
    }
    Ok(frame)
}
