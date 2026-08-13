//! 统一日志中枢: 一份记录同时供 UI 查看与文件持久化。
//!
//! 为什么不用 env_logger:
//!   - 原来只有 env_logger 打到 stderr, UI 里那个日志页显示的是 `AppController::event_log`
//!     ——只有上位机自己 push 的业务事件, io/nusb 层的错误(掉线、端点错误)根本进不去,
//!     出问题时 UI 上什么都看不到, 只能去看控制台窗口, 等于日志页不可用。
//!   - 本模块作为全局 `log::Log` 实现, 把**所有** target 的记录一并收进环形缓冲 + 落盘文件,
//!     UI 直接读这份缓冲, 与控制台内容完全一致, 不再需要那个类 cmd 窗口。
//!
//! 内存占用: 环形缓冲硬上限 `RING_MAX` 条, 超出丢最旧(计入 dropped 供 UI 提示);
//! UI 每次只取尾部 `VIEW_MAX` 条构建模型, 长日志下视图不随日志无限增长。

use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};

/// 内存环形缓冲上限(条)。约 20000 × ~120B ≈ 2.4MB 上界。
pub const RING_MAX: usize = 20_000;
/// UI 模型的单次日志页不再以固定 2000 行截断；只保留一个安全上限，数据量超过时由 Slint ScrollView 自适应滚动。
pub const VIEW_MAX: usize = 20_000;
/// logs 目录里保留的历史文件个数, 超出删最旧, 避免长期运行把目录堆满。
const KEEP_FILES: usize = 20;
/// 单个日志文件字节上限, 超出即换下一份(便于用编辑器打开)。
const FILE_MAX_BYTES: u64 = 32 * 1024 * 1024;

/// 一条日志。`level` 用 0=错误 1=警告 2=信息 3=调试, 与 UI 过滤等级同口径。
#[derive(Clone)]
pub struct LogEntry {
    pub no: u64,
    pub level: u8,
    pub text: String,
}

pub struct LogHub {
    ring: Mutex<VecDeque<LogEntry>>,
    /// 每次写入 +1, UI 据此判断是否需要重建模型(避免每帧重建)。
    version: AtomicU64,
    next_no: AtomicU64,
    /// 显示边界：`no < view_start_no` 的记录仍保留在内存环和日志文件中，但不再属于当前视图。
    /// 只单调推进，不重置 `next_no`，避免清空与并发写入之间出现重号或丢行。
    view_start_no: AtomicU64,
    /// 内存环溢出丢弃的历史记录总数。
    dropped: AtomicU64,
    /// 当前显示边界对应的内存环丢弃数基线；`dropped() - dropped_at_view_start` 即清空后的统计。
    dropped_at_view_start: AtomicU64,
    file: Mutex<Option<BufWriter<File>>>,
    file_path: Mutex<Option<PathBuf>>,
    file_bytes: AtomicU64,
    /// 写入文件的最高等级(0错误..3调试), 跟随日志页选择的过滤等级。默认 2=信息。
    file_level: std::sync::atomic::AtomicU8,
}

static HUB: OnceLock<LogHub> = OnceLock::new();

/// 全局日志中枢。
pub fn hub() -> &'static LogHub {
    HUB.get_or_init(|| LogHub {
        ring: Mutex::new(VecDeque::with_capacity(1024)),
        version: AtomicU64::new(0),
        next_no: AtomicU64::new(1),
        view_start_no: AtomicU64::new(1),
        dropped: AtomicU64::new(0),
        dropped_at_view_start: AtomicU64::new(0),
        file: Mutex::new(None),
        file_path: Mutex::new(None),
        file_bytes: AtomicU64::new(0),
        file_level: std::sync::atomic::AtomicU8::new(2),
    })
}

/// 安装为全局 logger，并在启动时无条件初始化日志文件。
/// 等级仍尊重 `RUST_LOG`(缺省 debug 全收: 过滤交给 UI, 这样切"调试"等级不必重启)。
pub fn init() {
    let max = match std::env::var("RUST_LOG").ok().as_deref() {
        Some("error") => log::LevelFilter::Error,
        Some("warn") => log::LevelFilter::Warn,
        Some("info") => log::LevelFilter::Info,
        Some("trace") => log::LevelFilter::Trace,
        _ => log::LevelFilter::Debug,
    };
    // 已装过(例如测试进程重复调用)就别再装, set_logger 只允许一次。
    let _ = log::set_logger(&HubLogger);
    log::set_max_level(max);
    // 文件落盘没有关闭入口；创建失败时 logger 仍完整收集内存日志并记录 warning。
    hub().ensure_file();
}

struct HubLogger;

impl log::Log for HubLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        let level = match metadata.level() {
            log::Level::Error => 0u8,
            log::Level::Warn => 1,
            log::Level::Info => 2,
            _ => 3,
        };
        // One production threshold governs the in-memory ring, file and debug
        // console. Filtering only in snapshot() lets hidden Debug flood the ring
        // and evict visible Info records before the user can inspect them.
        level <= hub().level()
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let level = match record.level() {
            log::Level::Error => 0u8,
            log::Level::Warn => 1,
            log::Level::Info => 2,
            _ => 3,
        };
        let (date, time) = now_strings();
        let lvl = level_tag(level);
        let target = record.target();
        let msg = record.args().to_string();
        // UI 行: 不带日期(同一次运行不必每行重复), 带毫秒 + target 便于定位来源。
        let view_line = format!("{} {} {}: {}", time, lvl, short_target(target), msg);
        // 文件行: 带完整日期, 便于跨次运行比对。
        let file_line = format!("{} {} {} {}: {}", date, time, lvl, target, msg);
        hub().push(level, view_line, &file_line);

        // 调试构建保留控制台输出便于开发; release 不带控制台窗口, 一切看 UI 与文件。
        // ★控制台同样只受日志等级约束★: 否则选了"信息"控制台仍在按 30Hz 刷每帧十六进制转储,
        // 刷屏到没法用。与文件、UI 视图共用同一个等级(日志页选的那个)。
        #[cfg(debug_assertions)]
        if level <= hub().level() {
            eprintln!("{}", file_line);
        }
    }

    fn flush(&self) {
        hub().flush();
    }
}

impl LogHub {
    fn push(&self, level: u8, view_line: String, file_line: &str) {
        let no = self.next_no.fetch_add(1, Ordering::Relaxed);
        {
            let mut ring = self.ring.lock().unwrap();
            ring.push_back(LogEntry {
                no,
                level,
                text: view_line,
            });
            while ring.len() > RING_MAX {
                ring.pop_front();
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
        self.version.fetch_add(1, Ordering::Relaxed);

        // File writes use the same threshold as the in-memory production gate.
        // Keep the explicit guard so direct callers remain safe if this logger is
        // ever reused outside the log facade.
        if level <= self.level() {
            let mut rotate = false;
            if let Ok(mut guard) = self.file.lock() {
                if let Some(w) = guard.as_mut() {
                    let _ = writeln!(w, "{}", file_line);
                    // ★每条都 flush★: 日志的用处就在崩溃/掉线/被强杀之后还能看。留在 BufWriter
                    // 里的内容在进程非正常结束时会全部丢失(实测被 kill 后文件里只剩表头)。
                    let _ = w.flush();
                    let n = self
                        .file_bytes
                        .fetch_add(file_line.len() as u64 + 2, Ordering::Relaxed);
                    rotate = n > FILE_MAX_BYTES;
                }
            }
            // 单文件超上限即换下一份, 免得跑一整天堆出一个几 GB 的文件没法打开。
            if rotate {
                self.rotate_file();
            }
        }
    }

    pub fn version(&self) -> u64 {
        self.version.load(Ordering::Relaxed)
    }

    /// 清空当前显示视图：不修改内存环、文件内容或全局序号，只推进显示边界。
    /// `next_no` 的当前值是下一条将分配的序号，因此边界设置为该值后，清空前已分配的记录
    /// 都被隐藏；并发写入若在边界之后分配序号，则自然保留在新视图中，不会重号或漏行。
    pub fn clear(&self) {
        let boundary = self.next_no.load(Ordering::Acquire);
        self.view_start_no.store(boundary, Ordering::Release);
        self.dropped_at_view_start
            .store(self.dropped.load(Ordering::Acquire), Ordering::Release);
        self.version.fetch_add(1, Ordering::Relaxed);
    }

    /// 设置当前日志等级(0错误..3调试)。由日志页的过滤等级驱动, 同时约束落盘与控制台输出:
    /// 看什么等级就记什么等级, 不另设第二套阈值。
    pub fn set_level(&self, level: u8) {
        let lv = level.min(3);
        if self.file_level.swap(lv, Ordering::Relaxed) != lv {
            log::info!(
                "日志等级 → {}(同时作用于文件与控制台)",
                level_tag(lv).trim()
            );
        }
    }

    pub fn level(&self) -> u8 {
        self.file_level.load(Ordering::Relaxed)
    }

    pub fn dropped(&self) -> u64 {
        self.dropped
            .load(Ordering::Acquire)
            .saturating_sub(self.dropped_at_view_start.load(Ordering::Acquire))
    }

    /// 取尾部最多 `max_lines` 条、等级不高于 `min_level`(数值越大越啰嗦)的记录。
    pub fn snapshot(&self, level_filter: u8, max_lines: usize) -> Vec<LogEntry> {
        let ring = self.ring.lock().unwrap();
        let mut out: Vec<LogEntry> = Vec::with_capacity(max_lines.min(ring.len()));
        let view_start_no = self.view_start_no.load(Ordering::Acquire);
        // 从尾往前收集, 收满即停(长日志下不遍历全量)。清空只推进边界，环内历史仍保留。
        for e in ring.iter().rev() {
            if e.no < view_start_no || e.level > level_filter {
                continue;
            }
            out.push(e.clone());
            if out.len() >= max_lines {
                break;
            }
        }
        out.reverse();
        out
    }

    /// 把当前视图范围的日志拼成纯文本(供复制)。
    pub fn text_for_copy(&self, level_filter: u8, max_lines: usize) -> String {
        let lines = self.snapshot(level_filter, max_lines);
        let mut s = String::with_capacity(lines.len() * 96);
        for e in lines {
            s.push_str(&e.text);
            s.push_str("\r\n");
        }
        s
    }

    pub fn flush(&self) {
        if let Ok(mut guard) = self.file.lock() {
            if let Some(w) = guard.as_mut() {
                let _ = w.flush();
            }
        }
    }

    /// 当前强制日志文件路径；文件不可用时为空，内存日志仍继续工作。
    pub fn file_path_text(&self) -> String {
        match self.file_path.lock().unwrap().as_ref() {
            Some(p) => p.display().to_string(),
            None => String::new(),
        }
    }

    /// 换下一份日志文件(达到单文件上限时调用)。不经过关闭开关，直接创建下一文件。
    fn rotate_file(&self) {
        self.flush();
        *self.file.lock().unwrap() = None;
        self.file_bytes.store(0, Ordering::Relaxed);
        self.ensure_file();
    }

    /// 确保强制落盘文件存在；失败时仅保留内存日志并由 warning 留下诊断。
    fn ensure_file(&self) {
        if self.file.lock().unwrap().is_some() {
            return;
        }
        let dir = logs_dir();
        if let Err(e) = std::fs::create_dir_all(&dir) {
            log::warn!("无法创建日志目录 {}: {}", dir.display(), e);
            return;
        }
        prune_old_files(&dir);
        let (date, time) = now_strings();
        let stamp = format!(
            "{}-{}",
            date.replace('-', ""),
            time.replace(':', "").replace('.', "-")
        );
        let path = dir.join(format!("mai2control-{}.log", stamp));
        match File::create(&path) {
            Ok(f) => {
                let mut w = BufWriter::new(f);
                let _ = writeln!(w, "# mai2control-ui 日志 {} {}", date, time);
                let _ = w.flush();
                *self.file.lock().unwrap() = Some(w);
                *self.file_path.lock().unwrap() = Some(path.clone());
                self.file_bytes.store(0, Ordering::Relaxed);
                log::info!("日志始终写入文件: {}", path.display());
            }
            Err(e) => {
                log::warn!("无法创建日志文件 {}: {}", path.display(), e);
            }
        }
    }
}

/// logs 目录: 程序所在目录下的 logs/(取不到程序路径则退回当前目录)。
pub fn logs_dir() -> PathBuf {
    let base = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.to_path_buf()))
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("logs")
}

/// 只保留最新 KEEP_FILES 个日志文件, 其余删除(按文件名排序即时间序)。
fn prune_old_files(dir: &PathBuf) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    let mut files: Vec<PathBuf> = rd
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            p.is_file()
                && p.file_name()
                    .and_then(|n| n.to_str())
                    .map(|n| n.starts_with("mai2control-") && n.ends_with(".log"))
                    .unwrap_or(false)
        })
        .collect();
    if files.len() < KEEP_FILES {
        return;
    }
    files.sort();
    let remove_count = files.len() + 1 - KEEP_FILES;
    for p in files.into_iter().take(remove_count) {
        let _ = std::fs::remove_file(p);
    }
}

fn level_tag(level: u8) -> &'static str {
    match level {
        0 => "ERROR",
        1 => "WARN ",
        2 => "INFO ",
        _ => "DEBUG",
    }
}

/// target 取最后一段, 避免每行都被 `mai2control_ui::app_state` 这种长前缀占满。
fn short_target(target: &str) -> &str {
    target.rsplit("::").next().unwrap_or(target)
}

/// 本地时间 → ("YYYY-MM-DD", "HH:MM:SS.mmm")。
/// 用 Win32 GetLocalTime 而不是引入 chrono/time: 只为格式化时间戳不值得多一个依赖树。
#[cfg(windows)]
fn now_strings() -> (String, String) {
    use windows::Win32::System::SystemInformation::GetLocalTime;
    // SAFETY: GetLocalTime 只写出参结构体。
    let st = unsafe { GetLocalTime() };
    (
        format!("{:04}-{:02}-{:02}", st.wYear, st.wMonth, st.wDay),
        format!(
            "{:02}:{:02}:{:02}.{:03}",
            st.wHour, st.wMinute, st.wSecond, st.wMilliseconds
        ),
    )
}

#[cfg(not(windows))]
fn now_strings() -> (String, String) {
    use std::time::{SystemTime, UNIX_EPOCH};
    let d = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let secs = d.as_secs();
    (
        "0000-00-00".to_string(),
        format!(
            "{:02}:{:02}:{:02}.{:03}",
            (secs / 3600) % 24,
            (secs / 60) % 60,
            secs % 60,
            d.subsec_millis()
        ),
    )
}

/// 把文本放进系统剪贴板(UI 的"复制"用)。Slint 没有通用剪贴板 API, 故走 Win32。
#[cfg(windows)]
pub fn copy_to_clipboard(text: &str) -> anyhow::Result<()> {
    use windows::Win32::Foundation::HANDLE;
    use windows::Win32::System::DataExchange::{
        CloseClipboard, EmptyClipboard, OpenClipboard, SetClipboardData,
    };
    use windows::Win32::System::Memory::{GMEM_MOVEABLE, GlobalAlloc, GlobalLock, GlobalUnlock};

    const CF_UNICODETEXT: u32 = 13;
    let wide: Vec<u16> = text.encode_utf16().chain(std::iter::once(0)).collect();
    let bytes = wide.len() * 2;
    // SAFETY: 标准剪贴板序列; 成功 SetClipboardData 后所有权移交系统, 不再 free。
    unsafe {
        OpenClipboard(None).map_err(|e| anyhow::anyhow!("打开剪贴板失败: {}", e))?;
        let result = (|| -> anyhow::Result<()> {
            EmptyClipboard().map_err(|e| anyhow::anyhow!("清空剪贴板失败: {}", e))?;
            let h = GlobalAlloc(GMEM_MOVEABLE, bytes)
                .map_err(|e| anyhow::anyhow!("分配剪贴板内存失败: {}", e))?;
            let dst = GlobalLock(h) as *mut u16;
            if dst.is_null() {
                return Err(anyhow::anyhow!("锁定剪贴板内存失败"));
            }
            std::ptr::copy_nonoverlapping(wide.as_ptr(), dst, wide.len());
            let _ = GlobalUnlock(h);
            SetClipboardData(CF_UNICODETEXT, Some(HANDLE(h.0)))
                .map_err(|e| anyhow::anyhow!("写入剪贴板失败: {}", e))?;
            Ok(())
        })();
        let _ = CloseClipboard();
        result
    }
}

#[cfg(not(windows))]
pub fn copy_to_clipboard(_text: &str) -> anyhow::Result<()> {
    Err(anyhow::anyhow!("仅 Windows 支持剪贴板复制"))
}

/// 在文件管理器里打开 logs 目录。
pub fn open_logs_dir() -> anyhow::Result<()> {
    let dir = logs_dir();
    std::fs::create_dir_all(&dir).ok();
    #[cfg(windows)]
    {
        std::process::Command::new("explorer")
            .arg(dir.as_os_str())
            .spawn()
            .map_err(|e| anyhow::anyhow!("打开日志目录失败: {}", e))?;
    }
    Ok(())
}
