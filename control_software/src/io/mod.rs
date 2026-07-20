//! WinUSB bulk transport for the always-present `mai2 config` interface.
//!
//! The firmware exposes vendor interface 0 with bulk OUT 0x01 and bulk IN 0x81.
//! On Windows, the MS OS 2.0 descriptor binds that interface to WinUSB. `nusb`
//! uses WinUSB directly, so the game-facing serial/light CDC ports remain free.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use log::{debug, error, info, warn};
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient};
use nusb::MaybeFuture;

use crate::proto::{encode, Decoder, Frame};

const TARGET_VID: u16 = 0x2E8A;
const TARGET_PID: u16 = 0x000A;
const TARGET_BCD_DEVICE: u16 = 0x0401;
const CONFIG_INTERFACE: u8 = 0;
const CONFIG_EP_OUT: u8 = 0x01;
const CONFIG_EP_IN: u8 = 0x81;
const TRANSFER_SIZE: usize = 4096;
const IO_TIMEOUT: Duration = Duration::from_millis(20);
const IDLE_SLEEP: Duration = Duration::from_millis(1);

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

pub struct IoHandle {
    cmd_tx: mpsc::Sender<Frame>,
    evt_rx: mpsc::Receiver<IoEvent>,
    running: Arc<AtomicBool>,
    _thread_handle: Option<thread::JoinHandle<()>>,
}

impl IoHandle {
    pub fn send(&self, frame: Frame) -> Result<()> {
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
        && info.interfaces().any(|itf| {
            itf.interface_number() == CONFIG_INTERFACE && itf.class() == 0xFF
        })
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

    info!("Enumerated {} mai2 config WinUSB device(s)", candidates.len());
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
    let data = interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: 0x50,
                value: 0,
                index: 0,
                length: 64,
            },
            Duration::from_millis(500),
        )
        .wait()
        .context("control_in 0x50 debug")?;
    Ok(data)
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

/// 打开 IN 端点并清除可能残留的 halt（上一进程断开可能留下 stall 状态），再构建 reader。
fn open_reader(interface: &nusb::Interface) -> Result<nusb::io::EndpointRead<Bulk>> {
    let ep = interface
        .endpoint::<Bulk, In>(CONFIG_EP_IN)
        .context("Missing config bulk IN endpoint 0x81")?;
    // 注意：不做 clear_halt。CLEAR_FEATURE(HALT) 会令固件 usbd_edpt_clear_stall 清 busy 但不清
    // claimed，留下 OUT 端点卡死(claimed=1 永久无法重新武装)。WinUSB 重开句柄不会复位设备端点，
    // 故设备 OUT 保持已武装态，无需清 halt。(实测 clear_halt 是 vendor 失步的根因触发。)
    let mut reader = ep.reader(TRANSFER_SIZE);
    reader.set_read_timeout(IO_TIMEOUT);
    // Keep one IN transfer pending before the first OUT command. The firmware may
    // already have a diagnostic packet queued; draining it prevents the 64-byte
    // vendor TX FIFO from making the immediate DEVICE_INFO response time out.
    reader.set_num_transfers(2);
    Ok(reader)
}

/// 打开 OUT 端点(清 halt)并构建 writer。
fn open_writer(interface: &nusb::Interface) -> Result<nusb::io::EndpointWrite<Bulk>> {
    let ep = interface
        .endpoint::<Bulk, Out>(CONFIG_EP_OUT)
        .context("Missing config bulk OUT endpoint 0x01")?;
    // 同 open_reader：不做 clear_halt，避免触发固件 OUT 端点 claimed 卡死。
    let mut writer = ep.writer(TRANSFER_SIZE);
    writer.set_write_timeout(IO_TIMEOUT);
    Ok(writer)
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

    // 连接时先主动清两端点 halt 并建 reader/writer（消除上一会话残留的 stall）。
    let mut writer = open_writer(&interface)?;
    let mut reader = open_reader(&interface)?;

    let (cmd_tx, cmd_rx) = mpsc::channel::<Frame>();
    let (evt_tx, evt_rx) = mpsc::channel::<IoEvent>();
    let running = Arc::new(AtomicBool::new(true));
    let thread_running = Arc::clone(&running);

    let thread_handle = thread::spawn(move || {
        info!("WinUSB IO thread started");
        if evt_tx.send(IoEvent::Connected).is_err() {
            return;
        }

        let mut decoder = Decoder::new();
        let mut read_buf = [0u8; 512];
        // stall 自恢复限流：短窗口内过多次恢复失败才真正断开。
        const MAX_STALL_RECOVERIES: u32 = 8;
        let mut stall_recoveries: u32 = 0;
        let mut window_start = std::time::Instant::now();

        while thread_running.load(Ordering::Acquire) {
            loop {
                match cmd_rx.try_recv() {
                    Ok(frame) => {
                        let bytes = encode(&frame);
                        if let Err(e) = writer.write_all(&bytes).and_then(|_| writer.flush()) {
                            warn!(
                                "WinUSB write failed: {e} (kind={:?})，尝试清 halt 重建 OUT 端点并重发",
                                e.kind()
                            );
                            match open_writer(&interface) {
                                Ok(w) => {
                                    writer = w;
                                    if let Err(e2) =
                                        writer.write_all(&bytes).and_then(|_| writer.flush())
                                    {
                                        error!("WinUSB 重发仍失败: {e2}");
                                        let _ = evt_tx.send(IoEvent::Error(e2.to_string()));
                                        let _ = evt_tx.send(IoEvent::Disconnected);
                                        return;
                                    }
                                    info!("OUT 端点恢复成功，命令已重发");
                                }
                                Err(e2) => {
                                    error!("OUT 端点恢复失败: {e2}");
                                    let _ = evt_tx.send(IoEvent::Error(e2.to_string()));
                                    let _ = evt_tx.send(IoEvent::Disconnected);
                                    return;
                                }
                            }
                        } else {
                            debug!(
                                "Sent WinUSB frame cmd=0x{:02X} seq={} len={}",
                                frame.cmd,
                                frame.seq,
                                bytes.len()
                            );
                        }
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => return,
                }
            }

            match reader.read(&mut read_buf) {
                Ok(0) => thread::sleep(IDLE_SLEEP),
                Ok(count) => {
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
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                    ) =>
                {
                    thread::sleep(IDLE_SLEEP)
                }
                Err(e) => {
                    // 端点 stall/错误：不立即断开，清 halt 重建 IN 端点后继续（与手动重连等效）。
                    if window_start.elapsed() > Duration::from_secs(5) {
                        window_start = std::time::Instant::now();
                        stall_recoveries = 0;
                    }
                    stall_recoveries += 1;
                    warn!(
                        "WinUSB read error: {e} (kind={:?})，自恢复尝试 #{}/{}",
                        e.kind(),
                        stall_recoveries,
                        MAX_STALL_RECOVERIES
                    );
                    if stall_recoveries > MAX_STALL_RECOVERIES {
                        error!("WinUSB read 短窗口内连续 {} 次错误，判定断开", stall_recoveries);
                        let _ = evt_tx.send(IoEvent::Error(e.to_string()));
                        let _ = evt_tx.send(IoEvent::Disconnected);
                        return;
                    }
                    // 旧 reader 在下面赋值时被丢弃(其 pending 传输随之取消)，再由 open_reader 清 halt。
                    match open_reader(&interface) {
                        Ok(r) => {
                            reader = r;
                            decoder = Decoder::new();
                            info!("IN 端点已清 halt 重建，连接继续");
                        }
                        Err(e2) => {
                            error!("IN 端点恢复失败: {e2}");
                            let _ = evt_tx.send(IoEvent::Error(e2.to_string()));
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
        _thread_handle: Some(thread_handle),
    })
}
