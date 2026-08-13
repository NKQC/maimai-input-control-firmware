//! 已改绑 `winusb.sys` 的扫码器直读。
//!
//! 设备改绑后 Windows 不再为它建 HID/键盘栈, 因此 Raw Input 与键盘过滤驱动都收不到任何东西 ——
//! 唯一的数据来源就是本模块: 用 `nusb`(项目既有的 WinUSB 传输层)claim 接口 0, 从中断 IN 端点
//! 直接读原始 HID 报告, 自己解出按键。
//!
//! ★解码只支持 8 字节 boot protocol★(byte0=modifier, byte1=保留, byte2..7=6 个 usage ID)。
//! 报告长度/布局不符时**如实报错并把原始字节打进日志**, 绝不换个布局硬解 —— 用错布局解出来的
//! 是看起来像数据的乱码, 那比明确失败危险得多。

use std::time::Duration;

use anyhow::{Result, anyhow};
use nusb::MaybeFuture;
use nusb::descriptors::TransferType;
use nusb::transfer::{ControlIn, ControlType, Direction, In, Interrupt, Recipient, TransferError};

use super::driver_pkg;

/// HID 类的 report descriptor 类型号(HID 1.11 §7.1.1)。
const HID_REPORT_DESCRIPTOR: u8 = 0x22;
const STANDARD_GET_DESCRIPTOR: u8 = 0x06;
const TARGET_INTERFACE: u8 = 0;
const BOOT_REPORT_LEN: usize = 8;
/// 同时在飞的传输数。★不能只留 1★ 读完再提交会留下一个未武装窗口, 扫码器的毫秒级连击正好落进去。
const QUEUE_DEPTH: usize = 4;

/// 一次 WinUSB 直读会话。生命周期由 `keyboard::Manager` 统一管理, 本类型只负责端点收发与解码。
pub(crate) struct Capture {
    /// 接口句柄必须与端点同寿: 它一 drop, WinUSB claim 就释放, 端点随之失效。
    _interface: nusb::Interface,
    _endpoint: nusb::Endpoint<Interrupt, In>,
    _packet: usize,
    /// 上一份报告里按下的 6 个 usage。boot protocol 每次上报**全量**按下集合,
    /// 不做差分就会把一次长按当成连续输入, 缓冲里全是重复字符。
    _held: [u8; 6],
    _stopped: bool,
}

impl Capture {
    /// 按所选 Raw Input 路径找到已改绑的 USB 设备并打开中断 IN 端点。
    /// 设备尚未改绑时直接失败 —— 这条路径**只**服务于已改绑设备, 不做任何隐式回退。
    pub(crate) fn open(raw_input_path: &str) -> Result<Self> {
        let status = driver_pkg::binding_status(raw_input_path);
        if !status.bound() {
            return Err(anyhow!("{}，WinUSB 直读不可用", status.detail()));
        }
        let instance = status
            .instance()
            .ok_or_else(|| anyhow!("已改绑但读不到 USB 实例 ID"))?
            .to_string();
        let (vid, pid, serial) = _identity(&instance)?;
        let mut matched: Vec<nusb::DeviceInfo> = nusb::list_devices()
            .wait()
            .map_err(|error| anyhow!("枚举 USB 设备失败：{}", error))?
            .filter(|info| info.vendor_id() == vid && info.product_id() == pid)
            .filter(|info| match serial.as_deref() {
                // 序列号是同型号多台设备的唯一区分依据; 设备没有序列号时才允许按 VID/PID 唯一匹配。
                Some(want) => info
                    .serial_number()
                    .is_some_and(|have| have.eq_ignore_ascii_case(want)),
                None => true,
            })
            .collect();
        let info = match matched.len() {
            1 => matched.remove(0),
            0 => {
                return Err(anyhow!(
                    "WinUSB 枚举里找不到 {}（VID_{:04X}/PID_{:04X}）",
                    instance,
                    vid,
                    pid
                ));
            }
            count => {
                return Err(anyhow!(
                    "同 VID_{:04X}/PID_{:04X} 匹配到 {} 台设备且无序列号可区分，拒绝任选一台",
                    vid,
                    pid,
                    count
                ));
            }
        };
        let device = info
            .open()
            .wait()
            .map_err(|error| anyhow!("打开 WinUSB 设备失败：{}", error))?;
        let interface = device
            .claim_interface(TARGET_INTERFACE)
            .wait()
            .map_err(|error| anyhow!("claim WinUSB 接口 {} 失败：{}", TARGET_INTERFACE, error))?;
        _log_report_descriptor(&interface);
        let (address, packet) = _interrupt_in(&interface)?;
        let endpoint = interface
            .endpoint::<Interrupt, In>(address)
            .map_err(|error| anyhow!("打开中断 IN 端点 0x{:02X} 失败：{}", address, error))?;
        log::info!(
            "虚拟摄像头: WinUSB 直读已就绪({} 中断 IN 0x{:02X}, 最大包 {} 字节)",
            instance,
            address,
            packet
        );
        Ok(Self {
            _interface: interface,
            _endpoint: endpoint,
            _packet: packet,
            _held: [0u8; 6],
            _stopped: false,
        })
    }
}

impl Capture {
    /// 读一批新按下的键。返回空表示本轮无新按键(含超时), `Err` 一律代表会话不可继续。
    pub(crate) fn receive(&mut self, timeout_ms: u32) -> Result<Vec<(u16, bool)>> {
        if self._stopped {
            return Ok(Vec::new());
        }
        while self._endpoint.pending() < QUEUE_DEPTH {
            let buffer = self._endpoint.allocate(self._packet);
            self._endpoint.submit(buffer);
        }
        let Some(completion) = self
            ._endpoint
            .wait_next_complete(Duration::from_millis(u64::from(timeout_ms)))
        else {
            return Ok(Vec::new());
        };
        let status = completion.status;
        // 先把这一份报告拷出来再把缓冲还给端点: 端点必须尽快重新武装, 8 字节的拷贝代价可忽略。
        let report: Vec<u8> = completion.buffer[..].to_vec();
        self._endpoint.submit(completion.buffer);
        match status {
            Ok(()) => self._decode(&report),
            // Cancelled 只是这一笔没赶上(超时取消/停止收尾), 端点仍然健康。
            Err(TransferError::Cancelled) => Ok(Vec::new()),
            Err(TransferError::Stall) => {
                log::warn!("虚拟摄像头: WinUSB 中断 IN 端点 stall，尝试 clear_halt");
                self._endpoint
                    .clear_halt()
                    .wait()
                    .map_err(|error| anyhow!("清除中断 IN 端点 stall 失败：{}", error))?;
                Ok(Vec::new())
            }
            Err(error) => Err(anyhow!("WinUSB 中断 IN 读取失败：{}", error)),
        }
    }

    /// boot protocol 解码。长度不符即失败, 并把原始字节留在日志里供排查。
    fn _decode(&mut self, report: &[u8]) -> Result<Vec<(u16, bool)>> {
        if report.len() != BOOT_REPORT_LEN {
            log::error!(
                "虚拟摄像头: WinUSB 报告长度 {} 字节，不是 8 字节 boot protocol；原始字节 {:02X?}",
                report.len(),
                report
            );
            return Err(anyhow!(
                "无法解码该设备的报告布局（收到 {} 字节，本实现只支持 8 字节 boot protocol）",
                report.len()
            ));
        }
        // modifier: bit1 = 左 Shift, bit5 = 右 Shift。
        let shift = report[0] & 0x22 != 0;
        let pressed = &report[2..BOOT_REPORT_LEN];
        let mut keys = Vec::new();
        for usage in pressed {
            // 0x00 = 空位; 0x01..0x03 = ErrorRollOver/POSTFail/ErrorUndefined, 都不是按键。
            if *usage <= 0x03 || self._held.contains(usage) {
                continue;
            }
            match _usage_to_vk(*usage) {
                Some(vk) => keys.push((vk, shift)),
                None => log::debug!("虚拟摄像头: WinUSB 报告含未映射 usage 0x{:02X}", usage),
            }
        }
        self._held.copy_from_slice(pressed);
        Ok(keys)
    }

    pub(crate) fn stop(&mut self) {
        if self._stopped {
            return;
        }
        self._endpoint.cancel_all();
        self._held = [0u8; 6];
        self._stopped = true;
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.stop();
    }
}

/// `USB\VID_xxxx&PID_xxxx\<尾段>` → (vid, pid, 序列号)。
/// 尾段含 `&` 时是总线生成的实例路径(设备没报序列号), 此时没有序列号可用于区分同型号设备。
fn _identity(instance: &str) -> Result<(u16, u16, Option<String>)> {
    let mut segments = instance.split('\\');
    let (_, hardware, tail) = (
        segments.next(),
        segments
            .next()
            .ok_or_else(|| anyhow!("USB 实例 ID 缺少硬件 ID 段：{}", instance))?,
        segments.next().unwrap_or_default(),
    );
    // VID/PID 的取法与 driver_pkg 共用同一份(它已保证是 4 位十六进制), 不另写第二套解析。
    let field = |key: &str| -> Result<u16> {
        let text = driver_pkg::hex_field(hardware, key)
            .ok_or_else(|| anyhow!("USB 硬件 ID 缺少合法 {}xxxx：{}", key, hardware))?;
        u16::from_str_radix(&text, 16)
            .map_err(|error| anyhow!("USB 硬件 ID 的 {} 不是十六进制：{}", key, error))
    };
    let serial = (!tail.is_empty() && !tail.contains('&')).then(|| tail.to_string());
    Ok((field("VID_")?, field("PID_")?, serial))
}

/// 找接口上的中断 IN 端点。HID 键盘接口固定有且只有一个。
fn _interrupt_in(interface: &nusb::Interface) -> Result<(u8, usize)> {
    let descriptor = interface
        .descriptor()
        .ok_or_else(|| anyhow!("读不到 WinUSB 接口描述符"))?;
    descriptor
        .endpoints()
        .find(|endpoint| {
            endpoint.direction() == Direction::In
                && endpoint.transfer_type() == TransferType::Interrupt
        })
        .map(|endpoint| (endpoint.address(), endpoint.max_packet_size()))
        .ok_or_else(|| anyhow!("该接口没有中断 IN 端点，无法直读扫码数据"))
}

/// 把 report descriptor 落日志备查。★取不到不算失败★: 解码走的是 boot protocol 固定布局,
/// 这一步只为将来排查"这台扫码器到底怎么排报告"留证据。
fn _log_report_descriptor(interface: &nusb::Interface) {
    let request = ControlIn {
        control_type: ControlType::Standard,
        recipient: Recipient::Interface,
        request: STANDARD_GET_DESCRIPTOR,
        value: u16::from(HID_REPORT_DESCRIPTOR) << 8,
        // WinUSB 限制: recipient 为 Interface 时 index 低字节必须等于已 claim 的接口号。
        index: u16::from(TARGET_INTERFACE),
        length: 512,
    };
    match interface.control_in(request, Duration::from_millis(500)).wait() {
        Ok(bytes) => log::info!(
            "虚拟摄像头: WinUSB 扫码器 report descriptor({} 字节) {:02X?}",
            bytes.len(),
            bytes
        ),
        Err(error) => log::warn!(
            "虚拟摄像头: 读取 report descriptor 失败（{}），仅影响排查，不影响 boot protocol 解码",
            error
        ),
    }
}

/// HID Usage Page 0x07(键盘/小键盘)的 usage ID → Win32 虚拟键码。
///
/// 只覆盖扫码器可能发出的键: 字母、数字、Enter、常见符号与小键盘。
/// ★到 VK 就停★ VK→字符仍交给 `keyboard::vk_to_char`(`ToUnicode` + 报告里的 Shift 态),
/// 这样直读路径与 Raw Input 路径共用同一份布局解释, 不会出现两套字符表。
fn _usage_to_vk(usage: u8) -> Option<u16> {
    let vk = match usage {
        // a..z → VK_A..VK_Z(0x41..0x5A)
        0x04..=0x1D => 0x41 + u16::from(usage - 0x04),
        // 1..9 → VK_1..VK_9
        0x1E..=0x26 => 0x31 + u16::from(usage - 0x1E),
        0x27 => 0x30, // 0
        0x28 => 0x0D, // Enter
        0x29 => 0x1B, // Esc
        0x2A => 0x08, // Backspace
        0x2B => 0x09, // Tab
        0x2C => 0x20, // Space
        0x2D => 0xBD, // VK_OEM_MINUS
        0x2E => 0xBB, // VK_OEM_PLUS
        0x2F => 0xDB, // VK_OEM_4  [
        0x30 => 0xDD, // VK_OEM_6  ]
        0x31 | 0x32 => 0xDC, // VK_OEM_5  \ (0x32 为非美式键盘的同位键)
        0x33 => 0xBA, // VK_OEM_1  ;
        0x34 => 0xDE, // VK_OEM_7  '
        0x35 => 0xC0, // VK_OEM_3  `
        0x36 => 0xBC, // VK_OEM_COMMA
        0x37 => 0xBE, // VK_OEM_PERIOD
        0x38 => 0xBF, // VK_OEM_2  /
        0x54 => 0x6F, // VK_DIVIDE
        0x55 => 0x6A, // VK_MULTIPLY
        0x56 => 0x6D, // VK_SUBTRACT
        0x57 => 0x6B, // VK_ADD
        0x58 => 0x0D, // 小键盘 Enter
        // 小键盘 1..9 → VK_NUMPAD1..VK_NUMPAD9
        0x59..=0x61 => 0x61 + u16::from(usage - 0x59),
        0x62 => 0x60, // VK_NUMPAD0
        0x63 => 0x6E, // VK_DECIMAL
        _ => return None,
    };
    Some(vk)
}
