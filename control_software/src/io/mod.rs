//! WinUSB bulk transport for the always-present `mai2 config` interface.
//!
//! The firmware exposes vendor interface 0 with bulk OUT 0x01 and bulk IN 0x81.
//! On Windows, the MS OS 2.0 descriptor binds that interface to WinUSB. `nusb`
//! uses WinUSB directly, so the game-facing serial/light CDC ports remain free.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow};
use log::{debug, error, info, warn};
use nusb::MaybeFuture;
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient};

use crate::proto::{Decoder, Frame, encode};

const TARGET_VID: u16 = 0x2E8A;
const TARGET_PID: u16 = 0x000A;
const TARGET_BCD_DEVICE: u16 = 0x0401;
const CONFIG_INTERFACE: u8 = 0;
const CONFIG_EP_OUT: u8 = 0x01;
const CONFIG_EP_IN: u8 = 0x81;
const TRANSFER_SIZE: usize = 4096;
const IO_TIMEOUT: Duration = Duration::from_millis(20);
const IDLE_SLEEP: Duration = Duration::from_millis(1);
/// 命令发送必须有上限：UI 连续轮询或设备暂忙时不能无限吃内存；可靠命令会在满时反压调用方，绝不丢弃。
const COMMAND_QUEUE_CAPACITY: usize = 256;
/// 端点错误后每轮只做有限次递增退避，避免重开句柄风暴同时给固件重新 arm 端点的时间。
const ENDPOINT_REOPEN_ATTEMPTS: u32 = 4;
const ENDPOINT_REOPEN_BASE_BACKOFF: Duration = Duration::from_millis(15);
const ENDPOINT_REOPEN_CYCLE_DELAY: Duration = Duration::from_millis(200);
/// 写路径对"设备忙(写超时)"的容忍预算。设备执行 CALIBRATE(~1.5s)/auto_tune(~10s) 等重操作期间
/// core0 暂不排空 OUT，主机写会连续超时——这是良性的(设备仍在)，与读路径"超时不判死"一致：
/// 在该预算内持续重试同一句柄、不拆端点；仅当忙到超过预算(疑似真掉线)才判定断开。
const WRITE_BUSY_BUDGET: Duration = Duration::from_secs(12);
/// 写路径遇到"真正端点错误"(非超时，如 stall/句柄失效)时连续重建端点的上限，达到即判定断开。
const WRITE_MAX_REBUILDS: u32 = 6;
/// 端点重建之间的递增退避：首次立即重试，其后线性递增。设备重新枚举需约 2s，
/// 若不退避则 6 次预算会在 ~80ms 内打光(实测)，真实可恢复的抖动也会被判成拔出。
/// 累计 0+120+240+480+960 = 1.8s > 一次重新枚举窗口。
const WRITE_REBUILD_BACKOFF_STEP: Duration = Duration::from_millis(120);
const WRITE_REBUILD_BACKOFF_CAP: Duration = Duration::from_millis(960);
/// 单轮 IO 循环最多连续写出的帧数。一次 UI 保存会把数百帧一次性入队(36 通道参数 + 键位 + 全局项)，
/// 全部背靠背写出会把设备 64B vendor OUT FIFO 打满并饿死读路径；分批写让读/写在同一轮交替推进。
/// 这只限制单轮突发量，不改变任何轮询周期。
const WRITE_BURST_PER_PASS: usize = 8;

#[derive(Debug, Clone)]
pub struct DeviceCandidate {
    /// Stable for the current enumeration. The value is the opaque nusb ID's
    /// debug representation and is only used to select the same device again.
    pub port_name: String,
    pub vid: u16,
    pub pid: u16,
    pub product: Option<String>,
    pub serial_number: Option<String>,
}

#[derive(Debug, Clone)]
pub enum IoEvent {
    Connected,
    Disconnected,
    Error(String),
    Frame(Frame),
}

/// 当前 WinUSB 会话的单调计数快照。用于断链日志与 soak 汇总，不参与控制流。
#[derive(Debug, Clone, Copy, Default)]
pub struct IoStats {
    pub bytes_read: u64,
    pub bytes_written: u64,
    pub stall_recoveries: u32,
    pub queue_dropped: u64,
}

#[derive(Default)]
struct IoStatsShared {
    bytes_read: AtomicU64,
    bytes_written: AtomicU64,
    stall_recoveries: AtomicU32,
    queue_dropped: AtomicU64,
}

impl IoStatsShared {
    fn snapshot(&self) -> IoStats {
        IoStats {
            bytes_read: self.bytes_read.load(Ordering::Relaxed),
            bytes_written: self.bytes_written.load(Ordering::Relaxed),
            stall_recoveries: self.stall_recoveries.load(Ordering::Relaxed),
            queue_dropped: self.queue_dropped.load(Ordering::Relaxed),
        }
    }
}

pub struct IoHandle {
    cmd_tx: mpsc::SyncSender<Frame>,
    evt_rx: mpsc::Receiver<IoEvent>,
    running: Arc<AtomicBool>,
    stats: Arc<IoStatsShared>,
    /// 与 IO 线程共用的已 claim WinUSB 接口；EP0 控制传输不需另开句柄或重 claim。
    debug_interface: Arc<nusb::Interface>,
    _thread_handle: Option<thread::JoinHandle<()>>,
}

impl IoHandle {
    pub fn send(&self, frame: Frame) -> Result<()> {
        // 满队列时阻塞而非静默丢命令；IO 线程会把可替代的轮询帧合并，真正写/配置请求仍完整保留。
        self.cmd_tx
            .send(frame)
            .map_err(|e| anyhow!("Failed to queue WinUSB frame: {e}"))
    }

    pub fn try_recv(&self) -> Option<IoEvent> {
        self.evt_rx.try_recv().ok()
    }

    pub fn recv_timeout(&self, timeout: Duration) -> Result<IoEvent> {
        self.evt_rx
            .recv_timeout(timeout)
            .map_err(|e| anyhow!("No WinUSB event received: {e}"))
    }

    pub fn stats(&self) -> IoStats {
        self.stats.snapshot()
    }

    /// 通过 IO 会话已经 claim 的接口读取 EP0 调试计数器，避免 WinUSB 重 claim 失败。
    pub fn read_debug_counters(&self) -> Result<Vec<u8>> {
        read_debug_from_interface(self.debug_interface.as_ref())
    }

    pub fn stop(mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(handle) = self._thread_handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for IoHandle {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(handle) = self._thread_handle.take() {
            let _ = handle.join();
        }
    }
}

fn is_target(info: &nusb::DeviceInfo) -> bool {
    info.vendor_id() == TARGET_VID
        && info.product_id() == TARGET_PID
        && info.device_version() == TARGET_BCD_DEVICE
        && info
            .interfaces()
            .any(|itf| itf.interface_number() == CONFIG_INTERFACE && itf.class() == 0xFF)
}

fn selector(info: &nusb::DeviceInfo) -> String {
    format!("{:?}", info.id())
}

pub fn list_devices() -> Vec<DeviceCandidate> {
    let devices = match nusb::list_devices().wait() {
        Ok(devices) => devices,
        Err(e) => {
            warn!("Failed to enumerate USB devices: {e}");
            return Vec::new();
        }
    };

    let candidates: Vec<_> = devices
        .filter(is_target)
        .map(|info| DeviceCandidate {
            port_name: selector(&info),
            vid: info.vendor_id(),
            pid: info.product_id(),
            product: info.product_string().map(str::to_owned),
            serial_number: info.serial_number().map(str::to_owned),
        })
        .collect();

    info!(
        "Enumerated {} mai2 config WinUSB device(s)",
        candidates.len()
    );
    candidates
}

/// 经 EP0 vendor 控制请求(bRequest=0x50)读取设备调试计数器。
/// bulk vendor 被 flash 写破坏后 EP0 仍存活，故本读取可诊断 bulk 死后的设备侧真相。
pub fn read_debug(device_selector: &str) -> Result<Vec<u8>> {
    let device_info = nusb::list_devices()
        .wait()
        .context("enumerate for debug read")?
        .find(|info| is_target(info) && selector(info) == device_selector)
        .ok_or_else(|| anyhow!("device disappeared before debug read"))?;
    let device = device_info.open().wait().context("open for debug read")?;
    let interface = device
        .claim_interface(CONFIG_INTERFACE)
        .wait()
        .context("claim interface for debug read")?;
    read_debug_from_interface(&interface)
}

fn read_debug_from_interface(interface: &nusb::Interface) -> Result<Vec<u8>> {
    interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: 0x50,
                value: 0,
                index: 0,
                // 必须 >= 固件 UsbDebugCounters 的实际大小(现 67B, 且只会在末尾追加字段)。
                // 原来固定 64 会把新追加的 NvStore 落盘诊断字段整段截掉, 读出来看不到但也不报错。
                length: 192,
            },
            Duration::from_millis(500),
        )
        .wait()
        .context("control_in 0x50 debug")
}

/// 经 EP0 vendor 控制请求(bRequest=0x52)令设备进 BOOTSEL。
/// bulk vendor 死后 EP0 仍存活，故本路径可在 vendor 卡死时软件触发进烧录，免物理 BOOTSEL。
pub fn ctrl_bootsel(device_selector: &str) -> Result<()> {
    let device_info = nusb::list_devices()
        .wait()
        .context("enumerate for ctrl bootsel")?
        .find(|info| is_target(info) && selector(info) == device_selector)
        .ok_or_else(|| anyhow!("device disappeared before ctrl bootsel"))?;
    let device = device_info.open().wait().context("open for ctrl bootsel")?;
    let interface = device
        .claim_interface(CONFIG_INTERFACE)
        .wait()
        .context("claim interface for ctrl bootsel")?;
    // 设备收到后会重启进 BOOTSEL 并断开，控制传输可能因断开而报错，忽略即可。
    let _ = interface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: 0x52,
                value: 0,
                index: 0,
                data: &[],
            },
            Duration::from_millis(500),
        )
        .wait();
    Ok(())
}

/// 打开 IN 端点并构建 reader。不能 clear_halt：该请求会让固件 OUT claimed 状态失步并永久卡死。
fn open_reader(interface: &nusb::Interface) -> Result<nusb::io::EndpointRead<Bulk>> {
    let ep = interface
        .endpoint::<Bulk, In>(CONFIG_EP_IN)
        .context("Missing config bulk IN endpoint 0x81")?;
    let mut reader = ep.reader(TRANSFER_SIZE);
    reader.set_read_timeout(IO_TIMEOUT);
    // 连接后先保持两个 IN 传输，防止遗留诊断包占满 64B vendor TX FIFO。
    reader.set_num_transfers(2);
    Ok(reader)
}

/// 打开 OUT 端点并构建 writer。同样不 clear_halt，避免固件 OUT claimed 卡死。
fn open_writer(interface: &nusb::Interface) -> Result<nusb::io::EndpointWrite<Bulk>> {
    let ep = interface
        .endpoint::<Bulk, Out>(CONFIG_EP_OUT)
        .context("Missing config bulk OUT endpoint 0x01")?;
    let mut writer = ep.writer(TRANSFER_SIZE);
    writer.set_write_timeout(IO_TIMEOUT);
    Ok(writer)
}

/// 只有这些请求是"最新值覆盖旧值"的纯轮询；任何配置/写入/操作请求均不在此集合，保证可靠送达。
fn is_replaceable_poll(cmd: u8) -> bool {
    matches!(cmd, 0x03 | 0x69 | 0x70) // PING / ALGO_GET_TRACE / KBD_GET_STATE
}

fn enqueue_frame(
    pending: &mut VecDeque<Frame>,
    frame: Frame,
    stats: &IoStatsShared,
    dropped_since_log: &mut u64,
) {
    if is_replaceable_poll(frame.cmd) {
        let old_len = pending.len();
        pending.retain(|queued| queued.cmd != frame.cmd);
        let replaced = old_len - pending.len();
        if replaced != 0 {
            stats
                .queue_dropped
                .fetch_add(replaced as u64, Ordering::Relaxed);
            *dropped_since_log += replaced as u64;
        }
    }
    pending.push_back(frame);
}

fn flush_drop_log(last_drop_log: &mut Instant, dropped_since_log: &mut u64) {
    if *dropped_since_log != 0 && last_drop_log.elapsed() >= Duration::from_secs(2) {
        warn!(
            "WinUSB send queue replaced {} stale periodic poll(s) in the last interval",
            *dropped_since_log
        );
        *dropped_since_log = 0;
        *last_drop_log = Instant::now();
    }
}

fn selected_device_present(device_selector: &str) -> Result<bool> {
    Ok(nusb::list_devices()
        .wait()
        .context("enumerate after endpoint reopen failure")?
        .any(|info| is_target(&info) && selector(&info) == device_selector))
}

fn reopen_with_backoff<T, F>(label: &str, mut open: F) -> Result<T>
where
    F: FnMut() -> Result<T>,
{
    let mut last_error = None;
    for attempt in 1..=ENDPOINT_REOPEN_ATTEMPTS {
        let multiplier = 1u64 << (attempt - 1);
        thread::sleep(ENDPOINT_REOPEN_BASE_BACKOFF.saturating_mul(multiplier as u32));
        match open() {
            Ok(endpoint) => return Ok(endpoint),
            Err(error) => {
                // 重试过程走 debug: 真正的失败由调用方在判定断开时报一次 ERROR。
                debug!(
                    "{} reopen attempt {}/{} failed after backoff: {}",
                    label, attempt, ENDPOINT_REOPEN_ATTEMPTS, error
                );
                last_error = Some(error);
            }
        }
    }
    Err(last_error.expect("endpoint reopen has at least one attempt"))
}

/// 端点短暂不可用时只要同一 selector 仍被系统枚举，就继续有限退避轮次而不误报拔出。
/// 每轮退避总时长有上限；只有重新枚举确认设备消失才允许转为 Disconnected。
fn recover_endpoint<T, F>(label: &str, device_selector: &str, mut open: F) -> Result<T>
where
    F: FnMut() -> Result<T>,
{
    loop {
        match reopen_with_backoff(label, &mut open) {
            Ok(endpoint) => return Ok(endpoint),
            Err(error) => match selected_device_present(device_selector) {
                Ok(false) => {
                    return Err(error.context("device disappeared after endpoint recovery retries"));
                }
                Ok(true) => {
                    debug!(
                        "{} remains enumerated but endpoint is temporarily unavailable; keeping session alive for another bounded retry cycle",
                        label
                    );
                    thread::sleep(ENDPOINT_REOPEN_CYCLE_DELAY);
                }
                Err(enumerate_error) => {
                    // 枚举失败本身不能证明拔出，继续下一轮避免系统短暂重枚举时的假断线。
                    debug!(
                        "{} reopen verification could not enumerate device: {}; retrying",
                        label, enumerate_error
                    );
                    thread::sleep(ENDPOINT_REOPEN_CYCLE_DELAY);
                }
            },
        }
    }
}

fn read_u32(bytes: &[u8], offset: usize) -> Option<u32> {
    let data = bytes.get(offset..offset + 4)?;
    Some(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
}

/// 断链判定点用已经 claim 的 interface 读一次 EP0 诊断计数器，全程 debug 级。
/// ★不在恢复热路径调用★：设备真掉线时这条同步控制传输必然失败，若在每次端点重建前后各做一次，
/// 只会刷两条 WARN 把真实错误埋掉，并给恢复流程额外插入一次同步 EP0 传输拖慢重试。
/// 失败即静默(仅 debug)：设备已消失时"读不到诊断"本身不是新信息。
fn log_link_diagnostics(interface: &nusb::Interface, phase: &str, stats: &IoStatsShared) {
    if !log::log_enabled!(log::Level::Debug) {
        return;
    }
    let session = stats.snapshot();
    match read_debug_from_interface(interface) {
        Ok(raw) if raw.len() >= 44 => debug!(
            "WinUSB link diagnostic ({phase}): out_stalled={} rearm_count={} vendor_tx_bytes={} flash_write_count={} session_rx={} session_tx={} stall_recoveries={} queue_replaced={}",
            raw[41],
            read_u32(&raw, 36).unwrap_or(0),
            read_u32(&raw, 24).unwrap_or(0),
            read_u32(&raw, 28).unwrap_or(0),
            session.bytes_read,
            session.bytes_written,
            session.stall_recoveries,
            session.queue_dropped,
        ),
        Ok(raw) => debug!(
            "WinUSB link diagnostic ({phase}) returned short {} byte response",
            raw.len()
        ),
        Err(error) => debug!("WinUSB link diagnostic ({phase}) unavailable: {error}"),
    }
}

pub fn spawn(device_selector: &str) -> Result<IoHandle> {
    let device_info = nusb::list_devices()
        .wait()
        .context("Failed to enumerate USB devices")?
        .find(|info| is_target(info) && selector(info) == device_selector)
        .ok_or_else(|| anyhow!("mai2 config WinUSB device disappeared before open"))?;

    let device = device_info
        .open()
        .wait()
        .context("Failed to open mai2 config WinUSB device")?;
    let interface = device
        .claim_interface(CONFIG_INTERFACE)
        .wait()
        .context("Failed to claim mai2 config interface 0")?;
    // nusb::Interface 内部以 Arc 共享，可与 IO 线程并行提交 EP0 控制传输而不重 claim。
    let debug_interface = Arc::new(interface.clone());

    let writer = Some(open_writer(&interface)?);
    let reader = Some(open_reader(&interface)?);

    // 同步 ingress + IO 线程本地 VecDeque 双重有界：既阻止生产者无限堆积，也便于替换旧轮询。
    let (cmd_tx, cmd_rx) = mpsc::sync_channel::<Frame>(COMMAND_QUEUE_CAPACITY);
    let (evt_tx, evt_rx) = mpsc::channel::<IoEvent>();
    let running = Arc::new(AtomicBool::new(true));
    let thread_running = Arc::clone(&running);
    let stats = Arc::new(IoStatsShared::default());
    let thread_stats = Arc::clone(&stats);
    let thread_selector = device_selector.to_owned();

    let thread_handle = thread::spawn(move || {
        info!("WinUSB IO thread started");
        if evt_tx.send(IoEvent::Connected).is_err() {
            return;
        }

        let mut writer = writer;
        let mut reader = reader;
        let mut pending = VecDeque::with_capacity(COMMAND_QUEUE_CAPACITY);
        let mut decoder = Decoder::new();
        let mut read_buf = [0u8; 512];
        let mut last_drop_log = Instant::now();
        let mut dropped_since_log = 0u64;
        // stall 自恢复限流：短窗口内过多次错误才真正进入端点恢复流程。
        const MAX_STALL_RECOVERIES: u32 = 8;
        let mut stall_recoveries: u32 = 0;
        let mut window_start = Instant::now();

        while thread_running.load(Ordering::Acquire) {
            while pending.len() < COMMAND_QUEUE_CAPACITY {
                match cmd_rx.try_recv() {
                    Ok(frame) => {
                        enqueue_frame(&mut pending, frame, &thread_stats, &mut dropped_since_log)
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => return,
                }
            }
            flush_drop_log(&mut last_drop_log, &mut dropped_since_log);

            let mut burst = 0usize;
            while let Some(frame) = pending.pop_front() {
                // 突发限速: 一次 UI 保存会连发数百帧, 而设备侧 host→device 环仅 1024B 且只在主循环
                // 排空(其间每条命令还可能自旋等 core1)。背靠背灌满会让设备静默丢字节→帧流损坏。
                // 每 WRITE_BURST_PER_PASS 帧让出 2ms 供设备排空; 不改变任何轮询周期。
                burst += 1;
                if burst > WRITE_BURST_PER_PASS {
                    burst = 0;
                    thread::sleep(Duration::from_millis(2));
                }
                let bytes = encode(&frame);
                let mut sent = false;
                let mut rebuild_streak: u32 = 0;
                let write_start = Instant::now();
                while thread_running.load(Ordering::Acquire) {
                    let result = writer
                        .as_mut()
                        .expect("writer present while sending")
                        .write_all(&bytes)
                        .and_then(|_| {
                            writer
                                .as_mut()
                                .expect("writer present while flushing")
                                .flush()
                        });
                    match result {
                        Ok(()) => {
                            thread_stats
                                .bytes_written
                                .fetch_add(bytes.len() as u64, Ordering::Relaxed);
                            sent = true;
                            break;
                        }
                        Err(error)
                            if matches!(
                                error.kind(),
                                std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                            ) =>
                        {
                            if write_start.elapsed() > WRITE_BUSY_BUDGET {
                                error!(
                                    "WinUSB write remained busy beyond {:?}; checking link",
                                    WRITE_BUSY_BUDGET
                                );
                                break;
                            }
                            thread::sleep(Duration::from_millis(4));
                        }
                        Err(error) => {
                            rebuild_streak += 1;
                            // 边界: 达到上限即结束，绝不打印越界的 #7/6(重试次数永远 <= 上限)。
                            if rebuild_streak >= WRITE_MAX_REBUILDS {
                                error!(
                                    "WinUSB write failed after {} endpoint errors (recovery budget exhausted): {error} (kind={:?})",
                                    rebuild_streak,
                                    error.kind()
                                );
                                break;
                            }
                            // 同一次故障只在最终失败时报一次 ERROR；中间重试全部走 debug，不刷屏。
                            debug!(
                                "WinUSB write error: {error} (kind={:?}), endpoint recovery {}/{}",
                                error.kind(),
                                rebuild_streak,
                                WRITE_MAX_REBUILDS
                            );
                            // 递增退避: 首次立即重试(瞬时 stall 常能立刻恢复), 其后 120/240/480/960ms
                            // 递增, 使"设备重新枚举约 2s"这种真实窗口有机会在不断开会话的前提下恢复。
                            if rebuild_streak > 1 {
                                let backoff = WRITE_REBUILD_BACKOFF_STEP
                                    .saturating_mul(1u32 << (rebuild_streak - 2))
                                    .min(WRITE_REBUILD_BACKOFF_CAP);
                                thread::sleep(backoff);
                            }
                            drop(writer.take());
                            match recover_endpoint("OUT endpoint", &thread_selector, || {
                                open_writer(&interface)
                            }) {
                                Ok(reopened) => {
                                    writer = Some(reopened);
                                }
                                Err(reopen_error) => {
                                    error!(
                                        "OUT endpoint recovery confirmed device removal: {reopen_error}"
                                    );
                                    log_link_diagnostics(
                                        &interface,
                                        "before disconnect after OUT recovery",
                                        &thread_stats,
                                    );
                                    let _ = evt_tx.send(IoEvent::Error(reopen_error.to_string()));
                                    let _ = evt_tx.send(IoEvent::Disconnected);
                                    return;
                                }
                            }
                        }
                    }
                }
                if !sent {
                    log_link_diagnostics(
                        &interface,
                        "before disconnect after write failure",
                        &thread_stats,
                    );
                    let _ = evt_tx.send(IoEvent::Error("write failed".into()));
                    let _ = evt_tx.send(IoEvent::Disconnected);
                    return;
                }
                debug!(
                    "Sent WinUSB frame cmd=0x{:02X} seq={} len={}",
                    frame.cmd,
                    frame.seq,
                    bytes.len()
                );
            }

            let read_result = reader
                .as_mut()
                .expect("reader present while reading")
                .read(&mut read_buf);
            match read_result {
                Ok(0) => thread::sleep(IDLE_SLEEP),
                Ok(count) => {
                    thread_stats
                        .bytes_read
                        .fetch_add(count as u64, Ordering::Relaxed);
                    debug!(
                        "Received {} WinUSB byte(s): {:02X?}",
                        count,
                        &read_buf[..count.min(64)]
                    );
                    for frame in decoder.feed_bytes(&read_buf[..count]) {
                        debug!(
                            "Decoded frame cmd=0x{:02X} flags=0x{:02X} seq={} len={}",
                            frame.cmd,
                            frame.flags,
                            frame.seq,
                            frame.payload.len()
                        );
                        if evt_tx.send(IoEvent::Frame(frame)).is_err() {
                            return;
                        }
                    }
                }
                Err(error)
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                    ) =>
                {
                    thread::sleep(IDLE_SLEEP)
                }
                Err(error) => {
                    if window_start.elapsed() > Duration::from_secs(5) {
                        window_start = Instant::now();
                        stall_recoveries = 0;
                    }
                    stall_recoveries += 1;
                    thread_stats
                        .stall_recoveries
                        .fetch_add(1, Ordering::Relaxed);
                    // 重试过程走 debug: 同一次故障只在最终判定断开时报一次 ERROR。
                    debug!(
                        "WinUSB read error: {error} (kind={:?}), recovery {}/{}",
                        error.kind(),
                        stall_recoveries,
                        MAX_STALL_RECOVERIES
                    );
                    if stall_recoveries >= MAX_STALL_RECOVERIES {
                        error!(
                            "WinUSB read failed after {} short-window recoveries: {error}",
                            stall_recoveries
                        );
                        log_link_diagnostics(
                            &interface,
                            "before disconnect after read recovery limit",
                            &thread_stats,
                        );
                        let _ = evt_tx.send(IoEvent::Error(error.to_string()));
                        let _ = evt_tx.send(IoEvent::Disconnected);
                        return;
                    }
                    drop(reader.take());
                    match recover_endpoint("IN endpoint", &thread_selector, || {
                        open_reader(&interface)
                    }) {
                        Ok(reopened) => {
                            reader = Some(reopened);
                            decoder = Decoder::new();
                            debug!("IN endpoint recovered; keeping WinUSB session connected");
                        }
                        Err(reopen_error) => {
                            error!("IN endpoint recovery confirmed device removal: {reopen_error}");
                            log_link_diagnostics(
                                &interface,
                                "before disconnect after IN recovery",
                                &thread_stats,
                            );
                            let _ = evt_tx.send(IoEvent::Error(reopen_error.to_string()));
                            let _ = evt_tx.send(IoEvent::Disconnected);
                            return;
                        }
                    }
                    thread::sleep(IDLE_SLEEP);
                }
            }
        }

        debug!("WinUSB IO thread stopped");
    });

    Ok(IoHandle {
        cmd_tx,
        evt_rx,
        running,
        stats,
        debug_interface,
        _thread_handle: Some(thread_handle),
    })
}
