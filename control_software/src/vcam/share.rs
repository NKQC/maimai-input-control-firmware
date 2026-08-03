//! 虚拟摄像头帧共享队列 —— **生产者侧**(上位机进程唯一生产者)。
//!
//! 对端是 DirectShow 源过滤器(`vcam_source_cpp/`), 它被载入**消费端进程**(游戏/播放器)后
//! 只读打开同一块命名映射取帧。两侧唯一的耦合就是这段布局, 因此:
//!   ★改动任何常量/偏移都必须同步 `vcam_source_cpp/vcam_common.h` 与 `vcam_queue.cpp`★。
//!
//! 布局(头 64 字节, 全部 u32 little-endian; 之后紧跟 3 个 NV12 槽):
//! ```text
//! 0x00 magic 'M2VC' | 0x04 version | 0x08 header_bytes | 0x0C state
//! 0x10 width        | 0x14 height  | 0x18 fourcc'NV12' | 0x1C slot_count
//! 0x20 slot_bytes   | 0x24 sequence| 0x28 producer_pid | 0x2C tick_ms(心跳)
//! 0x30 frame_interval_100ns        | 0x34..0x3C 保留
//! ```
//! 无锁三缓冲: 生产者写 `slot = (sequence+1) % 3`, 再以 **release 原子写**提交 `sequence`;
//! 消费者 **acquire 原子读**前后各取一次 `sequence`, 差值 >= 2 判撕裂重试。生产者永不被消费者
//! 阻塞 —— 过滤器跑在别人的进程里, 任何可阻塞的同步原语都是事故源。
//!
//! ★动态头字段(state/sequence/tick)一律走 4 字节对齐的原子读写, 绝不用 volatile 当跨进程原子★:
//! volatile 只约束编译器, 不是原子性/可见性契约。这里用 `AtomicU32::from_ptr`(标准库为共享内存
//! 明确支持的入口), 在 x86/x64 上与 Win32 `InterlockedExchange`/对齐 `mov` 生成同样的指令、
//! 同样的 ABI, 与 C++ 侧 `std::atomic_ref<unsigned int>` 互操作。
//! (windows crate 0.62 不提供任何用户态 `Interlocked*` 绑定, x64 kernel32 也不导出它们 ——
//!  它们在 SDK 里是编译器内建, 无法从 Rust 直接调用。)
//!
//! 命名空间固定 `Local\`(会话内): 消费端与上位机同会话, 不依赖 SeCreateGlobalPrivilege,
//! 因此**发布帧本身不需要管理员**; 需要管理员的只有过滤器 DLL 的注册(见 `backend`)。

use std::mem::size_of;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU32, Ordering};

use anyhow::{Result, anyhow};
use windows::Win32::Foundation::{CloseHandle, ERROR_ALREADY_EXISTS, GetLastError, HANDLE, HLOCAL};
use windows::Win32::Security::Authorization::{
    ConvertStringSecurityDescriptorToSecurityDescriptorW, SDDL_REVISION_1,
};
use windows::Win32::Security::{PSECURITY_DESCRIPTOR, SECURITY_ATTRIBUTES};
use windows::Win32::System::Memory::{
    CreateFileMappingW, FILE_MAP_ALL_ACCESS, FILE_MAP_READ, MEMORY_MAPPED_VIEW_ADDRESS,
    MapViewOfFile, OpenFileMappingW, PAGE_READWRITE, UnmapViewOfFile,
};
use windows::Win32::System::SystemInformation::GetTickCount;
use windows::Win32::System::Threading::{CreateMutexW, GetCurrentProcessId, ReleaseMutex};
use windows::core::w;

use super::{FRAME_H, FRAME_W};

/// 队列名(与 C++ 侧 `MAI2VCAM_MAP_NAME` 一致)。
pub const MAP_NAME: &str = r"Local\Mai2ControlVirtualCamVideoV1";
/// 生产者租约名: 与映射同 `Local\` 会话, 与 publisher 同生命周期。
/// 队列只允许**一个**活生产者, 这把互斥体就是唯一凭证。
pub const LEASE_NAME: &str = r"Local\Mai2ControlVirtualCamProducerV1";

const MAGIC: u32 = 0x4D32_5643; // 'M2VC'
const LAYOUT_VERSION: u32 = 1;
const HEADER_BYTES: usize = 64;
const SLOT_COUNT: u32 = 3;
const FOURCC_NV12: u32 = u32::from_le_bytes(*b"NV12");
/// 默认 30fps(100ns 单位), 与过滤器协商下限一致。
const FRAME_INTERVAL_100NS: u32 = 333_333;

/// 一帧 NV12 字节数: Y 平面 W*H + 交错 UV 平面 W*H/2。
pub const NV12_BYTES: usize = FRAME_W * FRAME_H * 3 / 2;
const RGB24_BYTES: usize = FRAME_W * FRAME_H * 3;
const MAP_BYTES: usize = HEADER_BYTES + SLOT_COUNT as usize * NV12_BYTES;

/// 头部字段的 u32 下标(= 字节偏移 / 4)。用显式下标而非结构体, 保证不受打包策略影响。
#[derive(Clone, Copy)]
#[repr(u32)]
enum Field {
    Magic = 0,
    Version = 1,
    HeaderBytes = 2,
    State = 3,
    Width = 4,
    Height = 5,
    Fourcc = 6,
    SlotCount = 7,
    SlotBytes = 8,
    Sequence = 9,
    ProducerPid = 10,
    TickMs = 11,
    Interval = 12,
}

/// 队列状态。消费者只在 `Ready` 且心跳新鲜时取真实帧, 否则输出占位帧。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueueState {
    Invalid = 0,
    Starting = 1,
    Ready = 2,
    Stopping = 3,
}

impl QueueState {
    fn from_raw(raw: u32) -> Self {
        match raw {
            1 => Self::Starting,
            2 => Self::Ready,
            3 => Self::Stopping,
            _ => Self::Invalid,
        }
    }
}

/// 把 RGB24(FRAME_W*FRAME_H*3) 转成 NV12(BT.601 **full range**)。
///
/// full range 而不是 studio range: 本路画面只有 QR 黑白两色, 压到 16..235 会牺牲对比度,
/// 而扫码识别对比度最敏感。黑 = Y 0 / UV 128, 与过滤器的占位帧编码一致。
pub fn rgb24_to_nv12(rgb: &[u8], nv12: &mut [u8]) -> Result<()> {
    if rgb.len() != RGB24_BYTES || nv12.len() != NV12_BYTES {
        return Err(anyhow!(
            "帧尺寸错误: RGB24 {}(期望 {}) / NV12 {}(期望 {})",
            rgb.len(),
            RGB24_BYTES,
            nv12.len(),
            NV12_BYTES
        ));
    }
    let (luma, chroma) = nv12.split_at_mut(FRAME_W * FRAME_H);
    for y in 0..FRAME_H {
        for x in 0..FRAME_W {
            let p = (y * FRAME_W + x) * 3;
            let (r, g, b) = (rgb[p] as i32, rgb[p + 1] as i32, rgb[p + 2] as i32);
            luma[y * FRAME_W + x] = (((77 * r + 150 * g + 29 * b + 128) >> 8).clamp(0, 255)) as u8;
        }
    }
    // 色度 4:2:0: 每 2x2 像素块取一组 UV。四个像素直接按下标取, 不再嵌第三层循环。
    for by in 0..FRAME_H / 2 {
        for bx in 0..FRAME_W / 2 {
            let top = ((by * 2) * FRAME_W + bx * 2) * 3;
            let bottom = top + FRAME_W * 3;
            let r =
                rgb[top] as i32 + rgb[top + 3] as i32 + rgb[bottom] as i32 + rgb[bottom + 3] as i32;
            let g = rgb[top + 1] as i32
                + rgb[top + 4] as i32
                + rgb[bottom + 1] as i32
                + rgb[bottom + 4] as i32;
            let b = rgb[top + 2] as i32
                + rgb[top + 5] as i32
                + rgb[bottom + 2] as i32
                + rgb[bottom + 5] as i32;
            // >>10 = 4 像素平均(>>2) 与 8 位定点系数(>>8) 合并。
            let u = (((-43 * r - 85 * g + 128 * b + 512) >> 10) + 128).clamp(0, 255);
            let v = (((128 * r - 107 * g - 21 * b + 512) >> 10) + 128).clamp(0, 255);
            let uv = (by * FRAME_W / 2 + bx) * 2;
            chroma[uv] = u as u8;
            chroma[uv + 1] = v as u8;
        }
    }
    Ok(())
}

/// 纯黑 NV12 帧(Y=0, UV=128)。与过滤器占位帧逐字节相同, 消费端在"生产者不在"与
/// "生产者输出黑屏"两种情况下看到的画面完全一致。
pub fn black_nv12() -> Vec<u8> {
    let mut frame = vec![0u8; NV12_BYTES];
    frame[FRAME_W * FRAME_H..].fill(128);
    frame
}

/// 帧生产者。句柄刻意只在 UI 线程持有(与 `VcamState` 的 tick 同线程),
/// 不跨线程传递 Windows HANDLE。
pub struct FramePublisher {
    /// 生产者租约(命名互斥体), 与本对象同生死: 它存在即代表"本进程是唯一生产者"。
    _lease: HANDLE,
    _map: HANDLE,
    _view: NonNull<u8>,
    _sequence: u32,
}

impl FramePublisher {
    /// 取得生产者租约并创建命名映射, 最后置 READY。
    ///
    /// ★不接管活着的 owner★: 先 `CreateMutexW` 抢租约, 拿到 `ERROR_ALREADY_EXISTS` 就直接报错
    /// 退出。历史实现是"已存在同名映射就重写头接管", 那等于两个实例同时往一块内存写像素,
    /// 消费端拿到的帧必然撕裂, 且先来的实例的 Drop 会把队列置 STOPPING 把后来者一起打死。
    pub fn create() -> Result<Self> {
        let (mut attributes, descriptor) = Self::_security_attributes()?;
        // SAFETY: attributes 内的安全描述符在本次调用期间存活; 句柄由本函数负责关闭。
        let lease = unsafe {
            CreateMutexW(
                Some(&mut attributes),
                true,
                w!("Local\\Mai2ControlVirtualCamProducerV1"),
            )
        };
        let existed = unsafe { GetLastError() } == ERROR_ALREADY_EXISTS;
        let lease = match lease {
            Ok(handle) if !existed => handle,
            Ok(handle) => {
                unsafe { _ = CloseHandle(handle) };
                unsafe { Self::_free_descriptor(descriptor) };
                return Err(anyhow!(
                    "另一个 mai2control 实例已在发布虚拟摄像头帧(租约 {} 已被占用); 请先关闭它",
                    LEASE_NAME
                ));
            }
            Err(error) => {
                unsafe { Self::_free_descriptor(descriptor) };
                return Err(anyhow!("创建生产者租约 {} 失败: {}", LEASE_NAME, error));
            }
        };
        let mapping = unsafe {
            CreateFileMappingW(
                HANDLE((-1isize) as *mut _),
                Some(&mut attributes),
                PAGE_READWRITE,
                0,
                MAP_BYTES as u32,
                w!("Local\\Mai2ControlVirtualCamVideoV1"),
            )
        };
        unsafe { Self::_free_descriptor(descriptor) };
        let mapping = match mapping {
            Ok(mapping) => mapping,
            Err(error) => {
                unsafe { Self::_drop_lease(lease) };
                return Err(error.into());
            }
        };
        let view = unsafe { MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, MAP_BYTES) };
        let Some(view) = NonNull::new(view.Value.cast::<u8>()) else {
            let error = windows::core::Error::from_thread();
            unsafe {
                _ = CloseHandle(mapping);
                Self::_drop_lease(lease);
            }
            return Err(error.into());
        };
        let mut publisher = Self {
            _lease: lease,
            _map: mapping,
            _view: view,
            _sequence: 0,
        };
        publisher._init_header();
        log::info!(
            "虚拟摄像头: 共享队列已就绪 {} (NV12 640x480 x3 槽, 生产者租约 {})",
            MAP_NAME,
            LEASE_NAME
        );
        Ok(publisher)
    }

    /// SAFETY: descriptor 必须来自 `_security_attributes` 且尚未释放。
    unsafe fn _free_descriptor(descriptor: PSECURITY_DESCRIPTOR) {
        unsafe { windows::Win32::Foundation::LocalFree(Some(HLOCAL(descriptor.0 as *mut _))) };
    }

    /// SAFETY: lease 必须是本进程通过 `CreateMutexW` 取得且持有的租约句柄。
    unsafe fn _drop_lease(lease: HANDLE) {
        unsafe {
            _ = ReleaseMutex(lease);
            _ = CloseHandle(lease);
        }
    }

    /// 发布一帧 RGB24: 先把像素转成 NV12 写进下一个槽, release 栅栏后才提交 sequence,
    /// 消费者把 sequence 当作"该槽像素已写完"的提交标志。
    pub fn publish(&mut self, rgb: &[u8]) -> Result<()> {
        let next = self._sequence.wrapping_add(1);
        let slot = (next % SLOT_COUNT) as usize;
        // SAFETY: 槽区间在映射内(HEADER_BYTES + slot*NV12_BYTES + NV12_BYTES <= MAP_BYTES),
        // 且只有本生产者写该槽。
        let destination = unsafe {
            std::slice::from_raw_parts_mut(
                self._view.as_ptr().add(HEADER_BYTES + slot * NV12_BYTES),
                NV12_BYTES,
            )
        };
        rgb24_to_nv12(rgb, destination)?;
        // release 原子写 = "该槽像素已写完"的提交点: 它同时是屏障与可见性发布, 不再依赖
        // 单独的 fence + volatile 写(后者不构成跨进程原子)。
        self._sequence = next;
        self._store(Field::Sequence, next, Ordering::Release);
        self._store(Field::TickMs, unsafe { GetTickCount() }, Ordering::Release);
        Ok(())
    }

    /// 心跳: 不换帧, 只刷新 tick_ms。消费者靠它区分"生产者在但画面没变"与"生产者已消失"
    /// (进程被杀不会有机会把 state 改成 STOPPING)。
    pub fn heartbeat(&mut self) {
        self._store(Field::TickMs, unsafe { GetTickCount() }, Ordering::Release);
    }

    /// SDDL: Everyone 读/执行 + 低完整性可读(`S:(ML;;NW;;;LW)`)。
    /// 消费端可能是低/中完整性进程, 而上位机可能以管理员(高完整性)运行 —— 不降标签的话
    /// 强制完整性控制会直接拦掉低完整性进程的读取, 表现为"摄像头一片黑"。
    fn _security_attributes() -> Result<(SECURITY_ATTRIBUTES, PSECURITY_DESCRIPTOR)> {
        let mut descriptor = PSECURITY_DESCRIPTOR::default();
        unsafe {
            ConvertStringSecurityDescriptorToSecurityDescriptorW(
                w!("D:(A;;GRGX;;;WD)(A;;GA;;;BA)(A;;GA;;;SY)S:(ML;;NW;;;LW)"),
                SDDL_REVISION_1 as u32,
                &mut descriptor,
                None,
            )?;
        }
        Ok((
            SECURITY_ATTRIBUTES {
                nLength: size_of::<SECURITY_ATTRIBUTES>() as u32,
                lpSecurityDescriptor: descriptor.0 as *mut _,
                bInheritHandle: false.into(),
            },
            descriptor,
        ))
    }

    /// 头部初始化顺序: 先 STARTING → 写全部字段与黑帧 → 最后才 READY。
    /// 消费者只认 READY, 因此不会看到半初始化的队列。
    ///
    /// 静态字段(magic/版本/尺寸/槽布局/pid/interval)在 READY 之前只有本进程会读写, 用普通写即可;
    /// 只有 state/sequence/tick 这三个动态字段需要跨进程原子。
    fn _init_header(&mut self) {
        self._store(Field::State, QueueState::Starting as u32, Ordering::Release);
        self._write(Field::Magic, MAGIC);
        self._write(Field::Version, LAYOUT_VERSION);
        self._write(Field::HeaderBytes, HEADER_BYTES as u32);
        self._write(Field::Width, FRAME_W as u32);
        self._write(Field::Height, FRAME_H as u32);
        self._write(Field::Fourcc, FOURCC_NV12);
        self._write(Field::SlotCount, SLOT_COUNT);
        self._write(Field::SlotBytes, NV12_BYTES as u32);
        self._store(Field::Sequence, 0, Ordering::Relaxed);
        self._write(Field::ProducerPid, unsafe { GetCurrentProcessId() });
        self._write(Field::Interval, FRAME_INTERVAL_100NS);
        let black = black_nv12();
        for slot in 0..SLOT_COUNT as usize {
            // SAFETY: 同 publish, 槽区间在映射内。
            unsafe {
                std::ptr::copy_nonoverlapping(
                    black.as_ptr(),
                    self._view.as_ptr().add(HEADER_BYTES + slot * NV12_BYTES),
                    NV12_BYTES,
                );
            }
        }
        self._store(Field::TickMs, unsafe { GetTickCount() }, Ordering::Release);
        // Release 存 READY: 上面所有静态字段与黑帧像素对消费者的可见性由这一步发布。
        self._store(Field::State, QueueState::Ready as u32, Ordering::Release);
    }

    /// 静态头字段的普通写(仅 READY 之前使用)。
    fn _write(&mut self, field: Field, value: u32) {
        // SAFETY: field 取值受限于头部 64 字节内, view 至少 MAP_BYTES。
        unsafe {
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(field as usize), value);
        }
    }

    /// 动态头字段的跨进程原子写。
    fn _store(&mut self, field: Field, value: u32, order: Ordering) {
        _cell(self._view.as_ptr(), field).store(value, order);
    }
}

impl Drop for FramePublisher {
    fn drop(&mut self) {
        // 正常退出先原子发布 STOPPING: 消费者立刻转占位帧, 不必等心跳超时。
        // 顺序固定为 STOPPING → 解除映射 → 释放租约: 租约必须最后放, 否则下一个实例可能在
        // 本实例还持有映射视图时就开始重写头。
        self._store(Field::State, QueueState::Stopping as u32, Ordering::Release);
        unsafe {
            _ = UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self._view.as_ptr().cast(),
            });
            _ = CloseHandle(self._map);
            Self::_drop_lease(self._lease);
        }
    }
}

/// 把头部字段当作跨进程原子单元访问。
///
/// SAFETY 前提(由调用方保证): `base` 是本布局的映射视图起点(页对齐 ⇒ 每个 4 字节字段天然对齐),
/// 字段偏移在头部 64 字节内。`AtomicU32::from_ptr` 是标准库为"另一进程也会访问同一内存"这类
/// 场景提供的入口, 只要访问全部经由原子操作即可。
fn _cell(base: *mut u8, field: Field) -> &'static AtomicU32 {
    unsafe { AtomicU32::from_ptr(base.cast::<u32>().add(field as usize)) }
}

/// 队列头快照(只读校验用)。
#[derive(Clone, Copy, Debug)]
pub struct QueueHeader {
    pub state: QueueState,
    pub width: u32,
    pub height: u32,
    pub slot_count: u32,
    pub slot_bytes: u32,
    pub sequence: u32,
    pub producer_pid: u32,
    pub tick_ms: u32,
    pub interval_100ns: u32,
}

/// 队列**消费端**的 Rust 实现: 与 `vcam_queue.cpp` 同一套判撕裂/心跳规则。
/// 生产链路的本机自检(`selftest --vcam-probe`)靠它验证, 免得只能靠装摄像头看画面。
pub struct QueueReader {
    _map: HANDLE,
    _view: NonNull<u8>,
}

impl QueueReader {
    pub fn open() -> Result<Self> {
        let map = unsafe {
            OpenFileMappingW(
                FILE_MAP_READ.0,
                false,
                w!("Local\\Mai2ControlVirtualCamVideoV1"),
            )
        }
        .map_err(|error| anyhow!("打开共享队列 {} 失败: {}", MAP_NAME, error))?;
        let view = unsafe { MapViewOfFile(map, FILE_MAP_READ, 0, 0, MAP_BYTES) };
        let Some(view) = NonNull::new(view.Value.cast::<u8>()) else {
            let error = windows::core::Error::from_thread();
            unsafe { _ = CloseHandle(map) };
            return Err(anyhow!("映射共享队列失败: {}", error));
        };
        Ok(Self {
            _map: map,
            _view: view,
        })
    }

    pub fn header(&self) -> Result<QueueHeader> {
        let magic = self._read(Field::Magic);
        if magic != MAGIC {
            return Err(anyhow!(
                "队列 magic 不符: 0x{:08X}(期望 0x{:08X})",
                magic,
                MAGIC
            ));
        }
        let version = self._read(Field::Version);
        if version != LAYOUT_VERSION {
            return Err(anyhow!(
                "队列布局版本不符: {}(期望 {})",
                version,
                LAYOUT_VERSION
            ));
        }
        let header_bytes = self._read(Field::HeaderBytes);
        if header_bytes != HEADER_BYTES as u32 {
            return Err(anyhow!(
                "队列头长度不符: {}(期望 {})",
                header_bytes,
                HEADER_BYTES
            ));
        }
        let fourcc = self._read(Field::Fourcc);
        if fourcc != FOURCC_NV12 {
            return Err(anyhow!("队列像素格式不符: 0x{:08X}(期望 NV12)", fourcc));
        }
        Ok(QueueHeader {
            state: QueueState::from_raw(self._read(Field::State)),
            width: self._read(Field::Width),
            height: self._read(Field::Height),
            slot_count: self._read(Field::SlotCount),
            slot_bytes: self._read(Field::SlotBytes),
            sequence: self._read(Field::Sequence),
            producer_pid: self._read(Field::ProducerPid),
            tick_ms: self._read(Field::TickMs),
            interval_100ns: self._read(Field::Interval),
        })
    }

    /// 取一帧到 destination(长度必须为 NV12_BYTES)。返回该帧的 sequence。
    /// 与 C++ 侧同规则: 读前后各取 sequence, 差值 >= 2 判撕裂, 最多重试 3 次。
    pub fn read(&self, destination: &mut [u8]) -> Result<u32> {
        if destination.len() != NV12_BYTES {
            return Err(anyhow!(
                "接收缓冲长度 {}(期望 {})",
                destination.len(),
                NV12_BYTES
            ));
        }
        let header = self.header()?;
        if header.state != QueueState::Ready {
            return Err(anyhow!("队列状态 {:?}, 非 Ready", header.state));
        }
        for _ in 0..3 {
            // acquire 原子读: 与生产者的 release 提交配对, 保证读到 sequence 之后看到的
            // 就是该槽写完后的像素。
            let first = self._read(Field::Sequence);
            let slot = (first % SLOT_COUNT) as usize;
            // SAFETY: 槽区间在映射内; 只读拷贝。
            unsafe {
                std::ptr::copy_nonoverlapping(
                    self._view.as_ptr().add(HEADER_BYTES + slot * NV12_BYTES),
                    destination.as_mut_ptr(),
                    NV12_BYTES,
                );
            }
            let second = self._read(Field::Sequence);
            if second.wrapping_sub(first) < 2 {
                return Ok(second);
            }
        }
        Err(anyhow!("连续 3 次读到撕裂帧: 生产者刷新速度远高于消费者"))
    }

    /// 头部字段的跨进程原子读(acquire)。
    ///
    /// ★只读映射不能用读改写指令做"原子读"★: 这块视图以 `FILE_MAP_READ` 打开, 任何
    /// `lock cmpxchg` 一类的实现都会对只读页写入而触发访问违例。对齐的 32 位原子 load
    /// 在 x86/x64 上就是一条 `mov`, 不写内存, 与 C++ 侧 `std::atomic_ref::load` 同语义。
    fn _read(&self, field: Field) -> u32 {
        _cell(self._view.as_ptr(), field).load(Ordering::Acquire)
    }
}

impl Drop for QueueReader {
    fn drop(&mut self) {
        unsafe {
            _ = UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self._view.as_ptr().cast(),
            });
            _ = CloseHandle(self._map);
        }
    }
}
