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
pub mod driver_pkg;
mod interception;
pub mod keyboard;
pub mod share;
mod winusb_scanner;

/// 一句话说明本机能不能走"自写内核过滤驱动"这条路。
/// UI 的诊断行与日志共用这一个出口，避免两处口径漂移。
pub fn kernel_driver_feasibility() -> String {
    interception::kernel_driver_feasibility()
}

/// 在位键盘类设备节点数 / Interception 键盘槽上限。查不到返回 None（绝不用 0 冒充）。
pub fn keyboard_slot_pressure() -> Option<(usize, usize)> {
    interception::present_keyboard_nodes()
        .map(|present| (present, interception::MAX_KEYBOARD as usize))
}

/// 输出当前 Raw Input 键盘与 Interception 槽位的只读关联诊断。
/// 该入口不会安装、卸载、重启设备，也不会设置拦截过滤器。
pub fn interception_diagnostic(target_filter: Option<&str>) -> String {
    interception::diagnostic_report(target_filter, &keyboard::list_keyboards())
}

/// 默认输出帧尺寸(RGB24)。多数摄像头消费端支持 640x480@RGB, 故作为出厂值与一切回落取值。
pub const DEFAULT_FRAME_W: usize = 640;
pub const DEFAULT_FRAME_H: usize = 480;

/// 分辨率上下限。★上限必须与 C++ 侧 `MAI2VCAM_MAX_WIDTH/HEIGHT` 一致★ 共享映射是按上限
/// 一次性开好的(见 `share::MAP_BYTES`), 超过上限就会写到映射之外。
pub const MIN_FRAME_W: usize = 160;
pub const MIN_FRAME_H: usize = 120;
pub const MAX_FRAME_W: usize = 1920;
pub const MAX_FRAME_H: usize = 1080;

/// QR 占短边的比例(百分比)及其上下限。下限再小就扫不出来, 上限 100% = 顶满短边。
pub const MIN_QR_FILL_PCT: u32 = 10;
pub const MAX_QR_FILL_PCT: u32 = 100;
pub const DEFAULT_QR_FILL_PCT: u32 = 75;

/// 把任意输入夹成合法分辨率: 落进上下限, 且宽高都取偶数。
///
/// ★偶数是硬要求, 不是美观问题★ 输出格式是 NV12(YUV 4:2:0), 色度平面按 2x2 像素块取样;
/// 奇数宽高会让最后一行/列没有对应的色度样本, 转换与消费端的行距计算都会错位。
pub fn clamp_resolution(width: u32, height: u32) -> (usize, usize) {
    (
        (width as usize).clamp(MIN_FRAME_W, MAX_FRAME_W) & !1,
        (height as usize).clamp(MIN_FRAME_H, MAX_FRAME_H) & !1,
    )
}

/// 一帧 RGB24 像素数据。长度恒为 width*height*3, 尺寸由 `Frame` 携带。
pub type Rgb24 = Vec<u8>;

/// 一帧输出画面: 像素与它的尺寸绑在一起。
///
/// ★为什么帧必须自带尺寸★ 分辨率成了运行期可改的量之后, "取一帧"与"读当前分辨率"就是两次
/// 独立的读取 —— 中间只要发生一次分辨率变更, 拿到的像素和拿到的尺寸就对不上, 后果是按错误
/// 行距解释像素(花屏)或按错误长度拷贝(越界)。把两者装进同一个结构体后, 这类错配在类型层面
/// 就不成立, 不必依赖每个调用方自觉去配对。
#[derive(Clone)]
pub struct Frame {
    pub width: usize,
    pub height: usize,
    pub rgb: Rgb24,
}

impl Frame {
    /// 该尺寸下 RGB24 应有的字节数。
    pub fn expected_len(&self) -> usize {
        self.width * self.height * 3
    }
    /// 尺寸与像素长度是否自洽。交给下游(转 NV12 / 上预览)之前一律先过这一关。
    pub fn is_consistent(&self) -> bool {
        self.rgb.len() == self.expected_len()
    }
}

/// 虚拟摄像头运行时状态(线程安全共享)。
pub struct VcamState {
    /// 当前对外输出帧(RGB24, 自带尺寸)。默认全黑。
    frame: Mutex<Frame>,
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
    /// 测试覆盖模式: 恒定输出带外框的 TEST 图案, 用来把"出画链路通不通"与"扫码数据对不对"
    /// 拆成两个可独立验证的问题。开着时 QR 仍可临时占用显示期, 到期回落到 TEST 而不是黑屏。
    test_pattern: AtomicBool,
    /// X 镜像(左右翻转)输出。
    ///
    /// ★为什么是输出侧变换, 而不是在渲染时就画反★ QR 的生成、测试图案的绘制、以及"当前该
    /// 显示什么"的时序状态机都与朝向无关; 把翻转塞进渲染就得让每个渲染路径各记一遍这件事,
    /// 多一条路就多一处会忘。存起来的 `frame` 始终是**未翻转**的规范帧, 翻转只在帧离开本状态
    /// 机时施加(见 `frame_copy`), 于是出画与预览必然一致, 且开关可随时切换而不必重新渲染。
    mirror_x: AtomicBool,
    /// 输出分辨率。改动只影响**之后**产生的帧: 已在共享队列里的那一帧仍按它自己的尺寸被消费。
    width: AtomicU32,
    height: AtomicU32,
    /// QR 占短边的百分比。
    qr_fill_pct: AtomicU32,
    /// 测试图案的动画序号: 每次定频重绘 +1, 驱动一个游标。
    /// ★静态图案证明不了帧在更新★ 消费端画面卡住与生产者停发在静态图上完全同形,
    /// 只有会动的元素能把两者区分开 —— 这正是本次要验证的东西。
    test_seq: AtomicU32,
}

impl VcamState {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            frame: Mutex::new(black_frame(DEFAULT_FRAME_W, DEFAULT_FRAME_H)),
            frame_version: AtomicU32::new(1),
            enabled: AtomicBool::new(false),
            show_until: Mutex::new(None),
            last_data: Mutex::new(String::new()),
            display_ms: AtomicU32::new(10_000),
            submit_timeout_ms: AtomicU32::new(1500),
            test_pattern: AtomicBool::new(false),
            mirror_x: AtomicBool::new(false),
            width: AtomicU32::new(DEFAULT_FRAME_W as u32),
            height: AtomicU32::new(DEFAULT_FRAME_H as u32),
            qr_fill_pct: AtomicU32::new(DEFAULT_QR_FILL_PCT),
            test_seq: AtomicU32::new(0),
        })
    }

    pub fn set_enabled(&self, on: bool) {
        self.enabled.store(on, Ordering::SeqCst);
        if !on {
            // 关闭即转黑屏并清显示态。
            *self.show_until.lock().unwrap() = None;
            let (w, h) = self.resolution();
            self.publish(black_frame(w, h));
        }
    }
    /// 当前输出分辨率。
    pub fn resolution(&self) -> (usize, usize) {
        (
            self.width.load(Ordering::SeqCst) as usize,
            self.height.load(Ordering::SeqCst) as usize,
        )
    }
    /// 设置输出分辨率(自动夹到合法范围并取偶数)。返回实际生效值。
    ///
    /// ★立刻按新尺寸重画当前画面★ 否则队列头里的 width/height 已经变了、槽里的像素还是旧尺寸,
    /// 消费端那一瞬间读到的就是尺寸与内容不符的一帧。改完即重画, 让两者始终同步。
    pub fn set_resolution(&self, width: u32, height: u32) -> (usize, usize) {
        let (w, h) = clamp_resolution(width, height);
        let changed = self.width.swap(w as u32, Ordering::SeqCst) != w as u32
            || self.height.swap(h as u32, Ordering::SeqCst) != h as u32;
        if changed {
            self._redraw_current();
            log::info!(
                "虚拟摄像头: 输出分辨率 → {}x{}（消费端需重新打开摄像头才会按新尺寸协商）",
                w,
                h
            );
        }
        (w, h)
    }
    /// 当前 QR 占短边的百分比。
    pub fn qr_fill_pct(&self) -> u32 {
        self.qr_fill_pct.load(Ordering::SeqCst)
    }
    /// 设置 QR 占比(百分比, 自动夹到上下限)。返回实际生效值。
    pub fn set_qr_fill_pct(&self, pct: u32) -> u32 {
        let pct = pct.clamp(MIN_QR_FILL_PCT, MAX_QR_FILL_PCT);
        if self.qr_fill_pct.swap(pct, Ordering::SeqCst) != pct {
            self._redraw_current();
            log::info!("虚拟摄像头: QR 占比 → {}%", pct);
        }
        pct
    }
    /// 按当前分辨率/占比重画"此刻本该显示的东西"。
    ///
    /// 三种状态各自的答案在这里集中一次, 避免每个改设置的入口各写一遍(那样必然漂移):
    /// 显示期内 → 重绘 QR; 测试模式 → 重绘测试图; 其余 → 黑屏。
    fn _redraw_current(&self) {
        let (w, h) = self.resolution();
        let showing_qr = self.show_until.lock().unwrap().is_some();
        if showing_qr {
            let data = self.last_data();
            if !data.is_empty() {
                match render_qr_frame_sized(&data, w, h, self.qr_fill_pct()) {
                    Ok(frame) => {
                        self.publish(frame);
                        return;
                    }
                    // 新尺寸放不下这串数据(比如分辨率调得极小): 不静默留着旧尺寸的帧,
                    // 转黑并说明原因, 否则用户会看到一张尺寸与队列头不符的画面。
                    Err(error) => {
                        log::warn!("虚拟摄像头: 按新设置重绘 QR 失败({}) → 黑屏", error);
                        self.publish(black_frame(w, h));
                        return;
                    }
                }
            }
        }
        if self.test_pattern() {
            self.publish(render_test_frame_sized(
                self.test_seq.load(Ordering::SeqCst),
                w,
                h,
            ));
        } else {
            self.publish(black_frame(w, h));
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
    pub fn test_pattern(&self) -> bool {
        self.test_pattern.load(Ordering::SeqCst)
    }
    /// 开/关测试覆盖模式。开启即立刻出图并清掉 QR 显示期(测试图是"当前该显示什么"的答案,
    /// 不能让上一次扫码的残余显示期继续压着它)。关闭则回落黑屏, 等下一次扫码。
    pub fn set_test_pattern(&self, on: bool) {
        self.test_pattern.store(on, Ordering::SeqCst);
        *self.show_until.lock().unwrap() = None;
        let (w, h) = self.resolution();
        if on {
            self.test_seq.store(0, Ordering::SeqCst);
            self.publish(render_test_frame_sized(0, w, h));
        } else {
            self.publish(black_frame(w, h));
        }
        log::info!(
            "虚拟摄像头: 测试覆盖模式{}",
            if on {
                "已开启 → 恒定输出 TEST 图案"
            } else {
                "已关闭 → 回落黑屏"
            }
        );
    }
    /// 定频重绘测试图案(由发布侧的 10fps 节拍驱动)。仅在测试模式开启且当前没有 QR 占着
    /// 显示期时动作 —— 扫码结果的可读性优先于测试图。
    pub fn advance_test_frame(&self) {
        if !self.test_pattern() || !self.enabled() {
            return;
        }
        if self.show_until.lock().unwrap().is_some() {
            return;
        }
        let seq = self.test_seq.fetch_add(1, Ordering::SeqCst).wrapping_add(1);
        let (w, h) = self.resolution();
        self.publish(render_test_frame_sized(seq, w, h));
    }
    pub fn last_data(&self) -> String {
        self.last_data.lock().unwrap().clone()
    }
    /// 拷贝当前帧(供后端/预览), 按需施加 X 镜像。
    ///
    /// ★镜像只在这一个出口施加★ 出画与预览是本状态机唯一的两个消费者, 都走这里, 因此
    /// "看到的"与"发出去的"不可能不一致。这里本来就要 clone, 翻转做在克隆体上是原地交换,
    /// 不额外分配。
    pub fn frame_copy(&self) -> Frame {
        let mut frame = self.frame.lock().unwrap().clone();
        if self.mirror_x() {
            mirror_x_in_place(&mut frame);
        }
        frame
    }
    pub fn mirror_x(&self) -> bool {
        self.mirror_x.load(Ordering::SeqCst)
    }
    /// 开/关 X 镜像。只翻一个标志并让帧版本前进一格 —— 版本推进是让 UI 预览立刻重建的依据;
    /// 出画那边本来就在按 10fps 定频取帧, 下一拍自然带上新朝向, 不需要重新渲染任何内容。
    pub fn set_mirror_x(&self, on: bool) {
        if self.mirror_x.swap(on, Ordering::SeqCst) == on {
            return;
        }
        self.frame_version.fetch_add(1, Ordering::SeqCst);
        log::info!(
            "虚拟摄像头: X 镜像{}",
            if on { "已开启" } else { "已关闭" }
        );
    }

    fn publish(&self, f: Frame) {
        debug_assert!(f.is_consistent(), "帧尺寸与像素长度不符");
        *self.frame.lock().unwrap() = f;
        self.frame_version.fetch_add(1, Ordering::SeqCst);
    }

    /// 提交一串扫码数据: 生成 QR 帧, 设显示截止, 记录 last_data(每串只用一次: 由调用方保证提交后清缓冲)。
    pub fn submit_data(&self, data: &str) {
        if !self.enabled() || data.is_empty() {
            return;
        }
        let (w, h) = self.resolution();
        let frame = match render_qr_frame_sized(data, w, h, self.qr_fill_pct()) {
            Ok(f) => f,
            Err(error) => {
                log::warn!("虚拟摄像头: QR 渲染失败({}) → 黑屏", error);
                black_frame(w, h)
            }
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
            // 测试模式开着时到期回落到测试图, 而不是黑屏: 否则一开测试模式扫一次码,
            // 十秒后画面就变黑, 用户会以为链路又断了。
            let (w, h) = self.resolution();
            if self.test_pattern() {
                let seq = self.test_seq.load(Ordering::SeqCst);
                self.publish(render_test_frame_sized(seq, w, h));
                log::info!("虚拟摄像头: QR 显示到期 → 回落测试图案");
            } else {
                self.publish(black_frame(w, h));
                log::info!("虚拟摄像头: QR 显示到期 → 黑屏");
            }
        }
    }
}

/// 指定尺寸的全黑 RGB24 帧。
fn black_frame(width: usize, height: usize) -> Frame {
    Frame {
        width,
        height,
        rgb: vec![0u8; width * height * 3],
    }
}

/// RGB24 帧原地左右翻转(逐行对称交换像素)。
///
/// 按 3 字节像素整体交换, 不逐分量搬: 分量顺序在翻转中不变, 拆开只会多两轮下标计算。
/// 只交换到行中点, 正中那一列(奇数宽度时)本来就不动。
/// 尺寸取自 `Frame` 自身, 因此不存在"拿错宽度翻转"这种可能。
pub fn mirror_x_in_place(frame: &mut Frame) {
    if !frame.is_consistent() {
        return;
    }
    let (w, h) = (frame.width, frame.height);
    for y in 0..h {
        let row = y * w * 3;
        for x in 0..w / 2 {
            let left = row + x * 3;
            let right = row + (w - 1 - x) * 3;
            for component in 0..3 {
                frame.rgb.swap(left + component, right + component);
            }
        }
    }
}

/// QR 纠错级别: **M**。
///
/// ★与游戏侧实际在用的图样一一对齐★ 依据是仓库根的 `sample.jpg`: 把它解回模块矩阵后实测为
/// Version 4(33x33) / 纠错 M / 掩码 0 / 字母数字模式, 而本函数按同一套规则对同一串数据生成的
/// 矩阵与它逐模块相同(0 个差异)。改动本常量或下面任何一个参数都会让图样与样本分家。
const QR_EC_LEVEL: qrcodegen::QrCodeEcc = qrcodegen::QrCodeEcc::Medium;

/// 生成 QR 模块矩阵(行优先, true = 深色), 返回 (边长模块数, 矩阵)。不含静区。
///
/// 规则全部固定, 与 `sample.jpg` 对齐:
///   - **模式**: 由 `QrSegment::make_segments` 按内容自选(纯数字→数字、全为字母数字集→字母数字、
///     其余→字节)。样本内容是大写字母加数字, 因此走字母数字模式 —— 这也是 84 个字符能塞进
///     Version 4 的前提(字节模式在 v4/M 下只有 62 字节, 根本放不下)。
///   - **版本**: 在 1..=40 里取放得下的最小值, 不写死。样本那串 84 字符因此落在 Version 4。
///   - **纠错**: 固定 `QR_EC_LEVEL`, 且**关闭**自动提升。
///     ★boost 必须关★ 打开后, 只要同一版本还塞得下更高一级纠错, 级别就会被悄悄上调 ——
///     于是纠错级别变成"随数据长度而变"的隐式行为, 短数据和长数据生成规则不一致, 无法与
///     样本这种固定级别的图样长期对齐。
///   - **掩码**: 交给按 ISO/IEC 18004 罚分规则的自动选择(样本那串数据的结果是掩码 0)。
///     不写死掩码: 罚分规则本身就是标准的一部分, 固定成某个数字反而会在别的数据上偏离标准。
pub fn qr_matrix(data: &str) -> anyhow::Result<(usize, Vec<bool>)> {
    use qrcodegen::{QrCode, QrSegment, Version};
    let segments = QrSegment::make_segments(data);
    let code = QrCode::encode_segments_advanced(
        &segments,
        QR_EC_LEVEL,
        Version::new(1),
        Version::new(40),
        None,
        false,
    )
    .map_err(|error| anyhow::anyhow!("QR 生成失败: {}", error))?;
    let size = code.size() as usize;
    let mut matrix = Vec::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            matrix.push(code.get_module(x as i32, y as i32));
        }
    }
    Ok((size, matrix))
}

/// 按默认分辨率与默认占比生成 QR 帧。仅供不关心尺寸的场合(自检等)使用。
pub fn render_qr_frame(data: &str) -> anyhow::Result<Frame> {
    render_qr_frame_sized(data, DEFAULT_FRAME_W, DEFAULT_FRAME_H, DEFAULT_QR_FILL_PCT)
}

/// 用数据生成 QR 帧: 黑底, 中央白色 QR(含静区)方块, 占 min(W,H) 的 `fill_pct`%, 居中。
pub fn render_qr_frame_sized(
    data: &str,
    width: usize,
    height: usize,
    fill_pct: u32,
) -> anyhow::Result<Frame> {
    let (modules, matrix) = qr_matrix(data)?;
    let mut frame = black_frame(width, height);

    // QR 方块像素边长 = fill_pct% * min(W,H)。
    let fill_pct = fill_pct.clamp(MIN_QR_FILL_PCT, MAX_QR_FILL_PCT) as usize;
    let side_px = width.min(height) * fill_pct / 100;
    // 含 4 模块静区(标准最小静区), 总模块数 = modules + 8。
    let quiet = 4usize;
    let total_modules = modules + quiet * 2;
    // 每模块像素(至少 1)。
    let mpx = (side_px / total_modules).max(1);
    let qr_px = mpx * total_modules; // 实际方块像素(整数模块对齐)
    // ★放不下就明确失败, 不要画一张缺角的 QR★ 分辨率被调得很小、或数据长导致版本很高时,
    // 即使每模块只占 1 像素也可能超出画面。此时给出可行动的错误, 由调用方转黑并提示,
    // 而不是让 ox/oy 下溢 panic 或裁掉一部分模块(缺角的 QR 是扫不出来的废图)。
    if qr_px > width || qr_px > height {
        return Err(anyhow::anyhow!(
            "QR 放不下: 需要 {}x{} 像素(共 {} 模块), 当前画面 {}x{}；请提高分辨率或降低占比",
            qr_px,
            qr_px,
            total_modules,
            width,
            height
        ));
    }
    let ox = (width - qr_px) / 2;
    let oy = (height - qr_px) / 2;

    for my in 0..total_modules {
        for mx in 0..total_modules {
            // 静区(白)与数据模块。
            let dark = if mx < quiet || my < quiet || mx >= quiet + modules || my >= quiet + modules
            {
                false // 静区为白
            } else {
                let cmx = mx - quiet;
                let cmy = my - quiet;
                matrix[cmy * modules + cmx]
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
                    let idx = (y * width + x) * 3;
                    frame.rgb[idx] = r;
                    frame.rgb[idx + 1] = g;
                    frame.rgb[idx + 2] = b;
                }
            }
        }
    }
    Ok(frame)
}

// ── 测试覆盖图案 ────────────────────────────────────────────────────────────────
//
// 用途: 把"帧到底有没有出到消费端"从"扫码数据对不对"里剥离出来独立验证。
// 之所以自己画而不引字体库: 只需要 T/E/S 三个字形, 引一个字体渲染栈(以及它的字体文件依赖)
// 换来的是同样的四个字母, 却多出一份要随二进制分发的资产与一套失败路径。

/// 5x7 点阵字形, 每行低 5 位有效(bit4 = 最左列)。
const GLYPH_W: usize = 5;
const GLYPH_H: usize = 7;

/// "TEST" 四个字形。顺序即绘制顺序。
const TEST_GLYPHS: [[u8; GLYPH_H]; 4] = [
    // T
    [
        0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100,
    ],
    // E
    [
        0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111,
    ],
    // S
    [
        0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110,
    ],
    // T
    [
        0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100,
    ],
];

/// 外框内缩、线宽、字形放大倍数、游标边长 —— 全部按短边比例算, 好让图案在任何分辨率下
/// 都保持同一副观感(在 160x120 上不会糊成一团, 在 1920x1080 上也不会缩成一个点)。
/// 基准是 480 短边下的原始取值(内缩 16 / 线宽 4 / 字形 12 倍 / 游标 18)。
fn _test_metrics(width: usize, height: usize) -> (usize, usize, usize, usize) {
    let short = width.min(height).max(16);
    let scale_of = |base: usize, min: usize| (base * short / 480).max(min);
    (
        scale_of(16, 2), // 内缩
        scale_of(4, 1),  // 线宽
        scale_of(12, 1), // 字形放大倍数
        scale_of(18, 2), // 游标边长
    )
}

/// 在帧上填一个实心矩形(按帧自身边界裁剪, 越界不 panic)。尺寸取自 `Frame`, 不会用错宽度。
#[inline]
fn fill_rect(frame: &mut Frame, x0: usize, y0: usize, w: usize, h: usize, rgb: (u8, u8, u8)) {
    let (fw, fh) = (frame.width, frame.height);
    let x_end = (x0 + w).min(fw);
    let y_end = (y0 + h).min(fh);
    for y in y0.min(fh)..y_end {
        let row = y * fw;
        for x in x0.min(fw)..x_end {
            let idx = (row + x) * 3;
            frame.rgb[idx] = rgb.0;
            frame.rgb[idx + 1] = rgb.1;
            frame.rgb[idx + 2] = rgb.2;
        }
    }
}

/// 生成测试覆盖帧: 黑底 + 白色外框 + 居中放大的 "TEST" + 沿框内侧走的动画游标。
///
/// `seq` 只驱动游标位置。游标是判"帧在更新"的唯一现场证据: 画面静止与生产者停发在静态图上
/// 完全同形, 有它才分得开。
pub fn render_test_frame(seq: u32) -> Frame {
    render_test_frame_sized(seq, DEFAULT_FRAME_W, DEFAULT_FRAME_H)
}

/// 按指定分辨率生成测试覆盖帧。所有尺寸按短边等比缩放, 观感与分辨率无关。
pub fn render_test_frame_sized(seq: u32, width: usize, height: usize) -> Frame {
    const WHITE: (u8, u8, u8) = (255, 255, 255);
    let (inset, thick, glyph_scale, cursor) = _test_metrics(width, height);
    let mut frame = black_frame(width, height);

    // 外框: 四条边各画一个实心矩形。极小分辨率下 inset*2 可能吃掉整幅画面, 故先兜住。
    let inner_w = width.saturating_sub(inset * 2);
    let inner_h = height.saturating_sub(inset * 2);
    if inner_w == 0 || inner_h == 0 {
        return frame;
    }
    fill_rect(&mut frame, inset, inset, inner_w, thick, WHITE);
    fill_rect(
        &mut frame,
        inset,
        height - inset - thick,
        inner_w,
        thick,
        WHITE,
    );
    fill_rect(&mut frame, inset, inset, thick, inner_h, WHITE);
    fill_rect(
        &mut frame,
        width - inset - thick,
        inset,
        thick,
        inner_h,
        WHITE,
    );

    // 居中的 "TEST"。放不下就整体跳过文字, 只留外框与游标 —— 画面仍能证明帧在更新。
    let gap = glyph_scale;
    let text_w = TEST_GLYPHS.len() * GLYPH_W * glyph_scale + (TEST_GLYPHS.len() - 1) * gap;
    let text_h = GLYPH_H * glyph_scale;
    if text_w <= width && text_h <= height {
        let ox = (width - text_w) / 2;
        let oy = (height - text_h) / 2;
        for (index, glyph) in TEST_GLYPHS.iter().enumerate() {
            let gx = ox + index * (GLYPH_W * glyph_scale + gap);
            for (row, bits) in glyph.iter().enumerate() {
                for col in 0..GLYPH_W {
                    // bit4 是最左列。
                    if bits & (1 << (GLYPH_W - 1 - col)) == 0 {
                        continue;
                    }
                    fill_rect(
                        &mut frame,
                        gx + col * glyph_scale,
                        oy + row * glyph_scale,
                        glyph_scale,
                        glyph_scale,
                        WHITE,
                    );
                }
            }
        }
    }

    // 动画游标: 沿外框内侧一圈匀速走。周长按四段等分, 每帧走一步。
    let track_w = inner_w.saturating_sub(cursor);
    let track_h = inner_h.saturating_sub(cursor);
    let perimeter = (track_w + track_h) * 2;
    if perimeter > 0 {
        let position = (seq as usize) % perimeter;
        let base = inset + thick;
        let (cx, cy) = if position < track_w {
            (base + position, base)
        } else if position < track_w + track_h {
            (base + track_w, base + (position - track_w))
        } else if position < track_w * 2 + track_h {
            (
                base + track_w - (position - track_w - track_h),
                base + track_h,
            )
        } else {
            (base, base + track_h - (position - track_w * 2 - track_h))
        };
        fill_rect(&mut frame, cx, cy, cursor, cursor, WHITE);
    }
    frame
}
