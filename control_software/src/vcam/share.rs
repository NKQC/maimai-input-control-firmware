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
//! 命名空间按 `Global\` → `Local\` 顺序各建一份(见 `Namespace`): 同会话消费端两者都能用,
//! 跨会话/以服务运行的消费端只有 `Global\` 那份可用, 而建 `Global\` 需要管理员。

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

use super::{Frame, MAX_FRAME_H, MAX_FRAME_W, clamp_resolution};

/// 共享对象所在的内核命名空间。
///
/// `Local\` 是**会话相对**的: 它在每个登录会话里解析到各自的 `\Sessions\<N>\BaseNamedObjects`。
/// 只要消费端进程不在上位机所在的登录会话里(以服务运行、或跑在 Session 0), 它查的
/// `Local\...` 与我们建的就是两个毫无关系的对象目录, `OpenFileMapping` 必然失败。
///
/// `Global\` 跨会话都指向同一个对象, 是这类消费端唯一取得到帧的途径。代价是创建它需要
/// `SeCreateGlobalPrivilege`(管理员/服务持有; UAC 过滤后的普通令牌没有), 所以生产者按
/// Global → Local 顺序退化, 并把实际落到哪一层如实报给界面, 不假装成功。
///
/// ★不要再把这件事和"Windows 设置里恒黑"挂钩★ 那条路根本不经过命名空间:
/// Media Foundation 不枚举 DirectShow 采集过滤器(`selftest --vcam-mf-list` 已实测),
/// 所以 Windows 设置 / 相机应用 / Teams 这类 MF 消费端从来就看不到本摄像头, 提权与否无关。
/// 用户此前在设置里看到的那一条是旧 Media Foundation 实现残留的幽灵相机(见
/// `backend::LEGACY_MF_CLSID`)。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Namespace {
    /// 跨会话可见: Windows 设置 / 相机应用(Session 0 的 Frame Server)也能取到帧。需管理员。
    Global,
    /// 仅本会话可见: 同会话消费端(OBS / 游戏)可用, Windows 自带相机预览取不到。
    Local,
}

impl Namespace {
    /// 映射名的宽字符串常量。用 `w!()` 编译期字面量而不是运行时拼串: 避免为每次调用分配,
    /// 也避免忘记结尾 NUL。
    fn _map(self) -> windows::core::PCWSTR {
        match self {
            Self::Global => w!("Global\\Mai2ControlVirtualCamVideoV1"),
            Self::Local => w!("Local\\Mai2ControlVirtualCamVideoV1"),
        }
    }

    /// 生产者租约名(命名互斥体), 与映射同命名空间。
    fn _lease(self) -> windows::core::PCWSTR {
        match self {
            Self::Global => w!("Global\\Mai2ControlVirtualCamProducerV1"),
            Self::Local => w!("Local\\Mai2ControlVirtualCamProducerV1"),
        }
    }

    pub fn map_name(self) -> &'static str {
        match self {
            Self::Global => r"Global\Mai2ControlVirtualCamVideoV1",
            Self::Local => r"Local\Mai2ControlVirtualCamVideoV1",
        }
    }

    pub fn lease_name(self) -> &'static str {
        match self {
            Self::Global => r"Global\Mai2ControlVirtualCamProducerV1",
            Self::Local => r"Local\Mai2ControlVirtualCamProducerV1",
        }
    }

    /// 界面用的一句话后果说明。只说用户能据此行动的事, 不解释内核命名空间。
    ///
    /// ★这里不再提"Windows 设置 / 相机应用"★ 实测(`selftest --vcam-mf-list`)确认: Media
    /// Foundation 根本不枚举 DirectShow 采集过滤器, 所以 Windows 设置、相机应用、Teams 这类
    /// MF 消费端**无论命名空间给到哪一层都看不到本摄像头**, 与提权无关。旧文案把这件事挂在
    /// 提权上, 把用户引到了完全错误的排查方向。
    pub fn consequence(self) -> &'static str {
        match self {
            Self::Global => "全局可见（跨会话、以服务运行的 DirectShow 消费端也能取到画面）",
            Self::Local => "仅本会话可见（同会话的 OBS / 游戏可用；跨会话或以服务运行的消费端取不到画面，需以管理员重启本程序）",
        }
    }

    /// 所有候选命名空间, 按优先级从高到低。
    const ORDER: [Self; 2] = [Self::Global, Self::Local];
}

/// 默认(优先)命名空间下的队列名, 供界面在尚未创建生产者时展示。
pub const MAP_NAME: &str = r"Global\Mai2ControlVirtualCamVideoV1";
/// 默认(优先)命名空间下的生产者租约名。队列只允许**一个**活生产者, 这把互斥体就是唯一凭证。
pub const LEASE_NAME: &str = r"Global\Mai2ControlVirtualCamProducerV1";

/// 一次建队尝试的失败原因。
///
/// ★必须把"没权限"与"已被占用"分开★ 前者应当退化到 `Local\` 继续可用; 后者说明**另一个实例
/// 正在发布**, 此时退化到 `Local\` 会凭空造出第二个生产者往另一块内存写, 消费端随机连到哪一块
/// 全看运气 —— 那正是租约机制要杜绝的事。故占用一律直接上报, 绝不退化。
enum _CreateFailure {
    /// 租约已被他人持有 ⇒ 立即上报, 不退化。
    LeaseTaken(anyhow::Error),
    /// 权限/系统原因失败 ⇒ 允许退化到下一个命名空间。
    Rejected(anyhow::Error),
}

impl _CreateFailure {
    fn into_error(self) -> anyhow::Error {
        match self {
            Self::LeaseTaken(error) | Self::Rejected(error) => error,
        }
    }
}

const MAGIC: u32 = 0x4D32_5643; // 'M2VC'
/// ★布局版本 2★ 与 1 相比有两处**不兼容**改动, 因此必须换号(与 C++ 侧
/// `MAI2VCAM_LAYOUT_VERSION` 同步):
///   · `slot_bytes` 的含义从"当前帧字节数"变成"槽间距(上限帧长, 恒定)";
///   · 映射总大小随之按上限分辨率计算, 从约 1.4MB 变成约 8.9MB。
/// 不换号的后果是: 装着旧 DLL 的消费端会在头部校验里静默失败(它拿 `slot_bytes` 与自己编译期的
/// 帧长比), 只表现为"一直黑屏", 没有任何线索指向"DLL 该更新了"。
const LAYOUT_VERSION: u32 = 2;
const HEADER_BYTES: usize = 64;
const SLOT_COUNT: u32 = 3;
const FOURCC_NV12: u32 = u32::from_le_bytes(*b"NV12");
/// ★固定 10fps(100ns 单位)★ 与 C++ 侧 `MAI2VCAM_DEFAULT_INTERVAL`/`MAI2VCAM_MAX_INTERVAL`
/// 取同一个值 ⇒ 协商结果恒为 10fps。本路画面是"静态 QR 显示若干秒"的准静态源, 更高帧率只是把
/// 同一帧重复推更多次; 生产者侧也按 100ms 定频发布(见 `PUBLISH_INTERVAL_MS`), 两端一一对应。
const FRAME_INTERVAL_100NS: u32 = 1_000_000;

/// 生产者定频发布周期(ms), 与 `FRAME_INTERVAL_100NS` 对应的 10fps。
///
/// ★为什么必须定频重发, 而不是"帧变了才发"★ 消费端(过滤器)靠 `tick_ms` 心跳判活, 超过
/// `MAI2VCAM_HEARTBEAT_TIMEOUT_MS` 就转占位帧; 而 QR 一旦显示就是长达十秒的静止画面, 期间
/// 帧内容根本不变。只在帧变化时发布, 等于把"画面静止"与"生产者死了"压成同一种观测结果,
/// 只能靠另一路 heartbeat 补救。定频发布让 sequence 与 tick_ms 一起单调推进, 心跳语义由
/// 发布本身承担, 不再需要第二条易漏的旁路。
pub const PUBLISH_INTERVAL_MS: u64 = 100;

/// 指定尺寸下一帧 NV12 的字节数: Y 平面 W*H + 交错 UV 平面 W*H/2。
pub fn nv12_bytes(width: usize, height: usize) -> usize {
    width * height * 3 / 2
}

/// 槽间距 = 上限分辨率下的一帧字节数。**恒定**, 与当前分辨率无关。
///
/// ★为什么映射按上限开而不按当前分辨率开★ 映射的名字是固定的, 消费端(可能在别的进程甚至别的
/// 会话)映射时必须先知道映射多大。若映射大小随分辨率变, 每次改分辨率都得重建映射并让所有消费端
/// 重新打开 —— 而消费端何时打开不由我们决定。按上限一次开好后, 槽偏移恒定, 改分辨率只动头部两个
/// 字段, 布局完全不变。代价是虚拟地址空间按上限预留(1920x1080 三槽约 8.9MB), 没写到的页不会真正
/// 占用物理内存。★这两个上限必须与 C++ 侧 `MAI2VCAM_MAX_WIDTH/HEIGHT` 一致★。
pub const SLOT_STRIDE: usize = MAX_FRAME_W * MAX_FRAME_H * 3 / 2;
const MAP_BYTES: usize = HEADER_BYTES + SLOT_COUNT as usize * SLOT_STRIDE;

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
pub fn rgb24_to_nv12(frame: &Frame, nv12: &mut [u8]) -> Result<()> {
    let (frame_w, frame_h) = (frame.width, frame.height);
    let rgb = &frame.rgb[..];
    if !frame.is_consistent() || nv12.len() != nv12_bytes(frame_w, frame_h) {
        return Err(anyhow!(
            "帧尺寸错误: {}x{} 的 RGB24 应为 {} 字节(实为 {}), NV12 应为 {} 字节(实为 {})",
            frame_w,
            frame_h,
            frame.expected_len(),
            rgb.len(),
            nv12_bytes(frame_w, frame_h),
            nv12.len()
        ));
    }
    let (luma, chroma) = nv12.split_at_mut(frame_w * frame_h);
    for y in 0..frame_h {
        for x in 0..frame_w {
            let p = (y * frame_w + x) * 3;
            let (r, g, b) = (rgb[p] as i32, rgb[p + 1] as i32, rgb[p + 2] as i32);
            luma[y * frame_w + x] = (((77 * r + 150 * g + 29 * b + 128) >> 8).clamp(0, 255)) as u8;
        }
    }
    // 色度 4:2:0: 每 2x2 像素块取一组 UV。四个像素直接按下标取, 不再嵌第三层循环。
    for by in 0..frame_h / 2 {
        for bx in 0..frame_w / 2 {
            let top = ((by * 2) * frame_w + bx * 2) * 3;
            let bottom = top + frame_w * 3;
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
            let uv = (by * frame_w / 2 + bx) * 2;
            chroma[uv] = u as u8;
            chroma[uv + 1] = v as u8;
        }
    }
    Ok(())
}

/// 纯黑 NV12 帧(Y=0, UV=128)。与过滤器占位帧逐字节相同, 消费端在"生产者不在"与
/// "生产者输出黑屏"两种情况下看到的画面完全一致。
pub fn black_nv12(width: usize, height: usize) -> Vec<u8> {
    let mut frame = vec![0u8; nv12_bytes(width, height)];
    frame[width * height..].fill(128);
    frame
}

/// 当前进程用户的 SID 字符串。拿不到就返回 None(此时安全描述符退化到不含用户 ACE 的那份)。
fn _current_user_sid() -> Option<String> {
    use windows::Win32::Foundation::HANDLE as WinHandle;
    use windows::Win32::Security::Authorization::ConvertSidToStringSidW;
    use windows::Win32::Security::{GetTokenInformation, TOKEN_QUERY, TOKEN_USER, TokenUser};
    use windows::Win32::System::Threading::{GetCurrentProcess, OpenProcessToken};

    let mut token = WinHandle::default();
    // SAFETY: GetCurrentProcess 返回伪句柄无需关闭; token 是出参。
    unsafe { OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &mut token) }.ok()?;
    // TOKEN_USER 后面紧跟着可变长的 SID, 所以要按运行期长度取一块缓冲, 不能只给结构体大小。
    let mut needed = 0u32;
    let _ = unsafe { GetTokenInformation(token, TokenUser, None, 0, &mut needed) };
    let mut buffer = vec![0u8; needed as usize];
    let queried = unsafe {
        GetTokenInformation(
            token,
            TokenUser,
            Some(buffer.as_mut_ptr().cast()),
            needed,
            &mut needed,
        )
    };
    let text = queried.ok().and_then(|()| {
        let user = unsafe { &*(buffer.as_ptr() as *const TOKEN_USER) };
        let mut raw = windows::core::PWSTR::null();
        unsafe { ConvertSidToStringSidW(user.User.Sid, &mut raw) }.ok()?;
        let text = unsafe { raw.to_string() }.ok();
        unsafe { _ = windows::Win32::Foundation::LocalFree(Some(HLOCAL(raw.as_ptr() as *mut _))) };
        text
    });
    unsafe { _ = CloseHandle(token) };
    text
}

/// 共享对象(映射与租约)的安全描述符 SDDL, 进程内只算一次。
///
/// ★必须显式给"本进程用户"完全控制★ 一块命名映射只要还有任何进程持着句柄就不会消失, 而消费端
/// 里的过滤器正是这样的持有者。于是"停用再启用摄像头"时 `CreateFileMappingW` 走的是**按名字
/// 打开已存在对象**这条路 —— 那要过一次访问检查。只给 Everyone 读、只给 Administrators 写的
/// 描述符会让**未提权**的上位机在这里拿到 ERROR_ACCESS_DENIED, 表现就是"第一次能开, 关掉再开
/// 就说拒绝访问", 而代码里那段"接管孤儿映射"的逻辑根本走不到。写权限只给当前用户自己, 不放给
/// Users/Authenticated Users: 别的本地账户没有理由能往这条摄像头里塞像素。
fn _sddl() -> &'static [u16] {
    static SDDL: std::sync::OnceLock<Vec<u16>> = std::sync::OnceLock::new();
    SDDL.get_or_init(|| {
        let owner = _current_user_sid()
            .map(|sid| format!("(A;;GA;;;{})", sid))
            .unwrap_or_default();
        format!(
            "D:{owner}(A;;GRGX;;;WD)(A;;GRGX;;;AC)(A;;GRGX;;;S-1-15-2-2)(A;;GA;;;BA)(A;;GA;;;SY)S:(ML;;NW;;;LW)"
        )
        .encode_utf16()
        .chain(Some(0))
        .collect()
    })
}

/// 一个命名空间下的一份队列(租约 + 映射 + 视图)。
///
/// ★为什么要允许"同时存在多份"★ 一块命名映射只能有一个名字, 而消费端分两类:
/// 已部署的旧过滤器只查 `Local\`, 新过滤器与跨会话的 Frame Server 要 `Global\`。若只建
/// 优先的那一个, 提权运行就会把同会话的旧消费端(OBS/游戏)一起打黑 —— 修一个洞开另一个洞。
/// 故两个名字各建一份, 由 `publish` 把同一帧写进各自的槽; NV12 转换只做一次, 多出的成本
/// 只是一次 460KB memcpy(10fps 下约 4.6MB/s), 换来的是两类消费端同时可用。
struct _Target {
    /// 生产者租约(命名互斥体), 与本对象同生死: 它存在即代表"本进程是该命名空间的唯一生产者"。
    _lease: HANDLE,
    _map: HANDLE,
    _view: NonNull<u8>,
    _namespace: Namespace,
}

/// 帧生产者。句柄刻意只在 UI 线程持有(与 `VcamState` 的 tick 同线程),
/// 不跨线程传递 Windows HANDLE。
pub struct FramePublisher {
    /// 已建成的队列, 按 `Namespace::ORDER` 的优先级顺序。非空由 `create` 保证。
    _targets: Vec<_Target>,
    _sequence: u32,
    /// NV12 暂存: 一帧只转一次, 再分别拷进各目标的槽。
    /// 复用同一块缓冲而不是每帧新分配 —— 10fps 下每次 460KB 的分配纯属浪费。
    _staging: Vec<u8>,
}

impl FramePublisher {
    /// 在所有可用命名空间上取租约并建映射, 最后各自置 READY。
    ///
    /// `width`/`height` 是**建队那一刻**的输出分辨率, 会直接写进队列头与初始黑帧。
    ///
    /// ★为什么建队就必须带上真实分辨率, 不能等第一次 publish 补★ 消费端里的过滤器是按队列头
    /// 报格式的(见 `vcam_source_cpp` 的 `_RefreshSize`), 而它可能在建队后的任意时刻探测 ——
    /// 包括第一次 publish 之前那不到 100ms 的窗口。头里若先写一个固定的 640x480, 那个窗口里
    /// 打开摄像头的消费端就会协商到 640x480, 之后与真实尺寸永久错开(不缩放 ⇒ 黑帧)。
    /// 初始黑帧同理: 映射页初值全 0, 按小尺寸铺黑会把大尺寸下多出来的部分留成 UV=0(绿屏)。
    ///
    /// ★不接管活着的 owner★: 先 `CreateMutexW` 抢租约, 拿到 `ERROR_ALREADY_EXISTS` 就直接报错
    /// 退出。历史实现是"已存在同名映射就重写头接管", 那等于两个实例同时往一块内存写像素,
    /// 消费端拿到的帧必然撕裂, 且先来的实例的 Drop 会把队列置 STOPPING 把后来者一起打死。
    pub fn create(width: usize, height: usize) -> Result<Self> {
        let (width, height) = clamp_resolution(width as u32, height as u32);
        let mut targets: Vec<_Target> = Vec::new();
        let mut rejected: Vec<String> = Vec::new();
        for namespace in Namespace::ORDER {
            match _Target::_create(namespace, width, height) {
                Ok(target) => targets.push(target),
                // 租约被占用 ⇒ 另一个实例正在发布。此时**整体**放弃: 继续下去就会出现两个
                // 生产者各写一块内存, 消费端连到哪一块全看运气。已建成的 target 由 Drop 收回。
                Err(_CreateFailure::LeaseTaken(error)) => return Err(error),
                Err(failure @ _CreateFailure::Rejected(_)) => {
                    let error = failure.into_error();
                    if namespace == Namespace::Global {
                        // 未提权拿不到 SeCreateGlobalPrivilege 是最常见的一种, 属预期降级路径。
                        log::warn!(
                            "虚拟摄像头: 无法在全局命名空间建队({}); 仅本会话可见, \
                             Windows 设置的相机预览将取不到画面(以管理员重启本程序可解决)",
                            error
                        );
                    }
                    rejected.push(error.to_string());
                }
            }
        }
        if targets.is_empty() {
            return Err(anyhow!(
                "没有可用的共享队列命名空间: {}",
                if rejected.is_empty() {
                    "未知原因".to_string()
                } else {
                    rejected.join("; ")
                }
            ));
        }
        let reached: Vec<&str> = targets.iter().map(|t| t._namespace.map_name()).collect();
        log::info!(
            // 槽布局按上限预留, 与当前分辨率无关; 头里的尺寸就是消费端会协商到的那个(硬透传)。
            "虚拟摄像头: 共享队列已就绪 (NV12 {}x{}, 槽 {}x{} 上限 {}x{}, 10fps) 覆盖 {} · {}",
            width,
            height,
            SLOT_COUNT,
            SLOT_STRIDE,
            MAX_FRAME_W,
            MAX_FRAME_H,
            reached.join(" + "),
            targets[0]._namespace.consequence()
        );
        Ok(Self {
            _targets: targets,
            _sequence: 0,
            // 暂存按建队尺寸起步; publish 遇到更大的尺寸会一次性扩容后复用。
            _staging: black_nv12(width, height),
        })
    }

    /// 可达性最好的命名空间(界面据此说明 Windows 自带相机预览能否取到画面)。
    /// `_targets` 按优先级顺序建成, 首个即最佳。
    pub fn namespace(&self) -> Namespace {
        self._targets[0]._namespace
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

    /// 发布一帧 RGB24: 像素转 NV12 一次, 写进每份队列的下一个槽, release 原子写提交 sequence,
    /// 消费者把 sequence 当作"该槽像素已写完"的提交标志。
    pub fn publish(&mut self, frame: &Frame) -> Result<()> {
        let (w, h) = clamp_resolution(frame.width as u32, frame.height as u32);
        if w != frame.width || h != frame.height {
            return Err(anyhow!(
                "帧分辨率 {}x{} 不在支持范围内(夹取后为 {}x{})",
                frame.width,
                frame.height,
                w,
                h
            ));
        }
        let bytes = nv12_bytes(w, h);
        // 暂存按当前尺寸取用; 只在变大时重新分配, 常态下不分配。
        if self._staging.len() < bytes {
            self._staging.resize(bytes, 0);
        }
        // 转换先做且只做一次: 它是本函数里唯一的重活(逐像素 YUV), 每个目标各转一遍纯属重复。
        rgb24_to_nv12(frame, &mut self._staging[..bytes])?;
        let next = self._sequence.wrapping_add(1);
        let slot = (next % SLOT_COUNT) as usize;
        let tick = unsafe { GetTickCount() };
        for target in &self._targets {
            // ★尺寸先写、序号后提交★ 消费端把 sequence 当作"这一槽已就绪"的提交标志, 并在读之前
            // 校验头部 width/height 与自己已协商的尺寸是否相等。因此尺寸必须在提交序号之前落地,
            // 否则会出现"序号已推进、尺寸还是旧值"的窗口, 消费端据此按旧尺寸解释新像素。
            target._store(Field::Width, w as u32, Ordering::Release);
            target._store(Field::Height, h as u32, Ordering::Release);
            // SAFETY: 槽区间在映射内(槽间距恒为 SLOT_STRIDE, 而 bytes <= SLOT_STRIDE 由
            // clamp_resolution 的上限保证), 且只有本生产者写该槽。
            unsafe {
                std::ptr::copy_nonoverlapping(
                    self._staging.as_ptr(),
                    target._view.as_ptr().add(HEADER_BYTES + slot * SLOT_STRIDE),
                    bytes,
                );
            }
            // release 原子写 = "该槽像素已写完"的提交点: 它同时是屏障与可见性发布, 不再依赖
            // 单独的 fence + volatile 写(后者不构成跨进程原子)。
            target._store(Field::Sequence, next, Ordering::Release);
            target._store(Field::TickMs, tick, Ordering::Release);
        }
        self._sequence = next;
        Ok(())
    }

    /// 心跳: 不换帧, 只刷新 tick_ms。消费者靠它区分"生产者在但画面没变"与"生产者已消失"
    /// (进程被杀不会有机会把 state 改成 STOPPING)。
    ///
    /// ★主链路已不依赖它★ 发布改成定频 10fps 后, tick_ms 由 `publish` 自己带着推进
    /// (见 `PUBLISH_INTERVAL_MS` 的说明)。保留本入口是给不发帧只判活的场景(自检)。
    pub fn heartbeat(&mut self) {
        let tick = unsafe { GetTickCount() };
        for target in &self._targets {
            target._store(Field::TickMs, tick, Ordering::Release);
        }
    }

    /// SDDL: Everyone + **应用容器** 读/执行 + 低完整性可读(`S:(ML;;NW;;;LW)`)。
    ///
    /// 三个 ACE 各自不可少:
    /// - `WD`(Everyone): 普通桌面消费端(OBS / 游戏)与服务(帧服务器以 LOCAL SERVICE 运行)。
    /// - `AC`(ALL APPLICATION PACKAGES) 与 `S-1-15-2-2`(所有受限制的应用程序包):
    ///   ★这两个是"Windows 设置 / 相机应用里恒黑"的直接原因★ UWP / AppContainer 进程的访问
    ///   检查比普通进程多一道: 除了用户与组 SID, 安全描述符里还必须有一条 ACE 授权给该应用包
    ///   SID、某个能力 SID, 或这两个"所有应用程序包"通配 SID。只给 `Everyone` 时 AppContainer
    ///   一律 ERROR_ACCESS_DENIED —— 这也是为什么 `C:\Program Files` 默认就带着这两条 ACE,
    ///   而 `C:\ProgramData` 不带。Windows 的相机取帧管道跑在(受限)应用容器里, 少了它们就
    ///   连共享段都打不开, 表现正是"设备列得出来、画面恒黑、过滤器日志里一条记录都没有"。
    /// - `S:(ML;;NW;;;LW)`: 上位机可能以管理员(高完整性)运行, 不把强制完整性标签降到低,
    ///   低完整性的消费端连读都过不去。
    fn _security_attributes() -> Result<(SECURITY_ATTRIBUTES, PSECURITY_DESCRIPTOR)> {
        let sddl = _sddl();
        let mut descriptor = PSECURITY_DESCRIPTOR::default();
        unsafe {
            ConvertStringSecurityDescriptorToSecurityDescriptorW(
                windows::core::PCWSTR(sddl.as_ptr()),
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
}

impl _Target {
    /// 在指定命名空间里取租约并建映射。失败原因区分"被占用"与"被拒绝", 供上层决定是否降级。
    fn _create(
        namespace: Namespace,
        width: usize,
        height: usize,
    ) -> std::result::Result<Self, _CreateFailure> {
        let (mut attributes, descriptor) =
            FramePublisher::_security_attributes().map_err(_CreateFailure::Rejected)?;
        // SAFETY: attributes 内的安全描述符在本次调用期间存活; 句柄由本函数负责关闭。
        let lease = unsafe { CreateMutexW(Some(&mut attributes), true, namespace._lease()) };
        let existed = unsafe { GetLastError() } == ERROR_ALREADY_EXISTS;
        let lease = match lease {
            Ok(handle) if !existed => handle,
            Ok(handle) => {
                unsafe { _ = CloseHandle(handle) };
                unsafe { FramePublisher::_free_descriptor(descriptor) };
                return Err(_CreateFailure::LeaseTaken(anyhow!(
                    "另一个 mai2control 实例已在发布虚拟摄像头帧(租约 {} 已被占用); 请先关闭它",
                    namespace.lease_name()
                )));
            }
            Err(error) => {
                unsafe { FramePublisher::_free_descriptor(descriptor) };
                return Err(_CreateFailure::Rejected(anyhow!(
                    "创建生产者租约 {} 失败: {}",
                    namespace.lease_name(),
                    error
                )));
            }
        };
        let mapping = unsafe {
            CreateFileMappingW(
                HANDLE((-1isize) as *mut _),
                Some(&mut attributes),
                PAGE_READWRITE,
                0,
                MAP_BYTES as u32,
                namespace._map(),
            )
        };
        // ★"映射已存在"必须留痕★ 同名映射只要还有**任何**进程持着句柄就不会消失 —— 消费端里的
        // 过滤器正是这样一个持有者。于是常见情形是: 上一轮的生产者早已退出, 而某个消费端把那份
        // 队列钉在内存里; 此时 CreateFileMapping 不会新建, 而是**接管**这份孤儿(返回
        // ERROR_ALREADY_EXISTS)。租约已经保证了"只有一个活生产者", 所以接管本身是安全且正确的
        // (随后 _init_header 会把它整个重写成 READY)。但它同时说明"还有旧消费端连着上一份队列",
        // 那是排查黑屏时的关键线索, 不能一声不响。
        let adopted = unsafe { GetLastError() } == ERROR_ALREADY_EXISTS;
        unsafe { FramePublisher::_free_descriptor(descriptor) };
        let mapping = match mapping {
            Ok(mapping) => mapping,
            Err(error) => {
                unsafe { FramePublisher::_drop_lease(lease) };
                return Err(_CreateFailure::Rejected(anyhow!(
                    "创建共享映射 {} 失败: {}",
                    namespace.map_name(),
                    error
                )));
            }
        };
        let view = unsafe { MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, MAP_BYTES) };
        let Some(view) = NonNull::new(view.Value.cast::<u8>()) else {
            let error = windows::core::Error::from_thread();
            unsafe {
                _ = CloseHandle(mapping);
                FramePublisher::_drop_lease(lease);
            }
            return Err(_CreateFailure::Rejected(anyhow!(
                "映射 {} 失败: {}",
                namespace.map_name(),
                error
            )));
        };
        if adopted {
            log::warn!(
                "虚拟摄像头: {} 已存在, 本次为接管(说明仍有消费端持着上一份队列; \
                 若它此前协商的分辨率与当前不同, 需要在那个消费端里重新打开摄像头)",
                namespace.map_name()
            );
        }
        let mut target = Self {
            _lease: lease,
            _map: mapping,
            _view: view,
            _namespace: namespace,
        };
        target._init_header(width, height);
        Ok(target)
    }

    /// 头部初始化顺序: 先 STARTING → 写全部字段与黑帧 → 最后才 READY。
    /// 消费者只认 READY, 因此不会看到半初始化的队列。
    ///
    /// 静态字段(magic/版本/尺寸/槽布局/pid/interval)在 READY 之前只有本进程会读写, 用普通写即可;
    /// 只有 state/sequence/tick 这三个动态字段需要跨进程原子。
    fn _init_header(&mut self, width: usize, height: usize) {
        self._store(Field::State, QueueState::Starting as u32, Ordering::Release);
        self._write(Field::Magic, MAGIC);
        self._write(Field::Version, LAYOUT_VERSION);
        self._write(Field::HeaderBytes, HEADER_BYTES as u32);
        // ★这里就得是真实分辨率, 不能填一个固定初值★ 消费端可能在第一次 publish 之前就来探测
        // 并据此协商(见 `create` 的说明), 协商完就再也改不了了。
        self._write(Field::Width, width as u32);
        self._write(Field::Height, height as u32);
        self._write(Field::Fourcc, FOURCC_NV12);
        self._write(Field::SlotCount, SLOT_COUNT);
        // 报的是**槽间距**(上限帧长), 不是当前帧长: 消费端靠它算槽偏移, 那个值必须恒定。
        self._write(Field::SlotBytes, SLOT_STRIDE as u32);
        self._store(Field::Sequence, 0, Ordering::Relaxed);
        self._write(Field::ProducerPid, unsafe { GetCurrentProcessId() });
        self._write(Field::Interval, FRAME_INTERVAL_100NS);
        // 按**建队尺寸**铺黑: 映射页初值全 0(Y=0/UV=0 是绿, 不是黑), 少铺一块就是绿边。
        let black = black_nv12(width, height);
        for slot in 0..SLOT_COUNT as usize {
            // SAFETY: 同 publish, 槽区间在映射内(槽间距恒为 SLOT_STRIDE, 黑帧只占其前一段)。
            unsafe {
                std::ptr::copy_nonoverlapping(
                    black.as_ptr(),
                    self._view.as_ptr().add(HEADER_BYTES + slot * SLOT_STRIDE),
                    black.len(),
                );
            }
        }
        self._store(Field::TickMs, unsafe { GetTickCount() }, Ordering::Release);
        // Release 存 READY: 上面所有静态字段与黑帧像素对消费者的可见性由这一步发布。
        self._store(Field::State, QueueState::Ready as u32, Ordering::Release);
    }

    /// 静态头字段的普通写(仅 READY 之前使用)。
    fn _write(&self, field: Field, value: u32) {
        // SAFETY: field 取值受限于头部 64 字节内, view 至少 MAP_BYTES。
        unsafe {
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(field as usize), value);
        }
    }

    /// 动态头字段的跨进程原子写。
    ///
    /// 取 `&self` 而非 `&mut self`: 写的是共享内存里的原子单元, 不改本结构体自身的任何字段,
    /// 这样 `publish` 才能在遍历 `&self._targets` 的同时提交各自的 sequence。
    fn _store(&self, field: Field, value: u32, order: Ordering) {
        _cell(self._view.as_ptr(), field).store(value, order);
    }
}

impl Drop for _Target {
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
            FramePublisher::_drop_lease(self._lease);
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
    /// 按 Global → Local 顺序挑一份**可用**的队列打开, 与 C++ 侧 `Mai2VcamQueueReader::_Open`
    /// 同一顺序、同一判据。
    ///
    /// ★"能打开"不等于"可用", 必须按可用性选★
    /// 同名映射只要还有任何进程持着句柄就不会消失, 而消费端里的过滤器正是这样的持有者。于是
    /// 会出现这种局面: 上位机曾以管理员在 `Global\` 建过队列, 某个消费端把它钉在内存里; 之后
    /// 上位机以普通权限重启, 只能在 `Local\` 建队。此时两个名字都打得开, 但 `Global\` 那份是
    /// 布局版本过时、状态为 Stopping 的孤儿。只按"能否打开"来选就会一直读那份死队列, 明明有
    /// 活着的 `Local\` 队列却一帧都取不到(实测正是如此)。
    ///
    /// 故分两轮: 先找**头部合法且状态为 Ready** 的; 都没有再退而接受"头部合法"的(便于诊断
    /// Starting/Stopping 这类中间态); 仍没有才报错。
    pub fn open() -> Result<Self> {
        let mut fallback: Option<Self> = None;
        let mut last: Option<anyhow::Error> = None;
        for namespace in Namespace::ORDER {
            let candidate = match Self::_open_in(namespace) {
                Ok(reader) => reader,
                Err(error) => {
                    last = Some(error);
                    continue;
                }
            };
            match candidate.header() {
                Ok(header) if header.state == QueueState::Ready => return Ok(candidate),
                Ok(header) => {
                    log::debug!(
                        "虚拟摄像头: {} 状态 {:?}, 暂不作首选",
                        namespace.map_name(),
                        header.state
                    );
                    // 头部合法但非 Ready: 留作备选, 继续看下一个命名空间有没有活的。
                    fallback = fallback.or(Some(candidate));
                }
                Err(error) => {
                    // 头部非法(版本过时的孤儿等) ⇒ 直接弃用这一份。
                    last = Some(anyhow!("{} 头部不可用: {}", namespace.map_name(), error));
                }
            }
        }
        fallback
            .ok_or_else(|| last.unwrap_or_else(|| anyhow!("没有可用的共享队列命名空间")))
    }

    /// 只做"打开并映射"这一步, 不判可用性。
    fn _open_in(namespace: Namespace) -> Result<Self> {
        let map = unsafe { OpenFileMappingW(FILE_MAP_READ.0, false, namespace._map()) }
            .map_err(|error| anyhow!("打开共享队列 {} 失败: {}", namespace.map_name(), error))?;
        let view = unsafe { MapViewOfFile(map, FILE_MAP_READ, 0, 0, MAP_BYTES) };
        let Some(view) = NonNull::new(view.Value.cast::<u8>()) else {
            let error = windows::core::Error::from_thread();
            unsafe { _ = CloseHandle(map) };
            return Err(anyhow!("映射 {} 失败: {}", namespace.map_name(), error));
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
        let header = self.header()?;
        if header.state != QueueState::Ready {
            return Err(anyhow!("队列状态 {:?}, 非 Ready", header.state));
        }
        // 期望长度按**头部当前报的尺寸**算, 与 C++ 侧同一规则: 队列的尺寸是运行期量。
        let expected = nv12_bytes(header.width as usize, header.height as usize);
        if destination.len() != expected {
            return Err(anyhow!(
                "接收缓冲长度 {}(队列当前 {}x{} 期望 {})",
                destination.len(),
                header.width,
                header.height,
                expected
            ));
        }
        for _ in 0..3 {
            // acquire 原子读: 与生产者的 release 提交配对, 保证读到 sequence 之后看到的
            // 就是该槽写完后的像素。
            let first = self._read(Field::Sequence);
            let slot = (first % SLOT_COUNT) as usize;
            // SAFETY: 槽区间在映射内(槽间距恒为 SLOT_STRIDE, expected <= SLOT_STRIDE); 只读拷贝。
            unsafe {
                std::ptr::copy_nonoverlapping(
                    self._view.as_ptr().add(HEADER_BYTES + slot * SLOT_STRIDE),
                    destination.as_mut_ptr(),
                    expected,
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
