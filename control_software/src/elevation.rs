//! 运行中途申请管理员权限(自提权重启)。
//!
//! 为什么需要: 有两类功能非管理员做不了 ——
//!   1. 固定游戏串口 COM 号: 写 HKLM 下设备的 `Device Parameters\PortName` 并重启端口节点。
//!   2. 虚拟摄像头: 媒体源 DLL 只能注册到 HKLM(会被多个进程加载), 且帧共享内存要建在 `Global\`
//!      命名空间(需要 SeCreateGlobalPrivilege)帧服务器才读得到。
//! 但让用户"每次都记得右键以管理员运行"是糟糕的体验, 平时用不到这两项时也没必要提权。
//! 所以做成"用到时再申请": 保持普通权限启动, 需要时一键以管理员重开自身(UAC 由系统弹),
//! 参数原样带过去, 旧进程退出。等价于 Clash 开 TUN 时那种"提权重启"交互。
//!
//! 注意: Windows 不允许给已运行的进程提权, 只能新起一个提升的进程 —— 因此必须重启, 无法原地提权。

use anyhow::{anyhow, Result};

/// 当前进程是否已是管理员(UAC 已提升)。复用 comport 里已有的令牌查询, 不重复实现。
pub fn is_elevated() -> bool {
    crate::comport::is_elevated()
}

/// 以管理员身份重新启动自身。成功返回 Ok(()) —— ★调用方随后必须退出当前进程★,
/// 否则会出现两个实例同时抢 WinUSB 句柄。
#[cfg(windows)]
pub fn relaunch_as_admin() -> Result<()> {
    use windows::core::{w, HSTRING, PCWSTR};
    use windows::Win32::UI::Shell::ShellExecuteW;
    use windows::Win32::UI::WindowsAndMessaging::SW_SHOWNORMAL;

    if is_elevated() {
        return Err(anyhow!("当前已是管理员权限, 无需重启"));
    }
    let exe = std::env::current_exe().map_err(|e| anyhow!("取不到自身路径: {}", e))?;
    // 原样把命令行参数带到提升后的实例(跳过 argv[0]), 免得提权后丢掉用户的启动选项。
    let args: Vec<String> = std::env::args().skip(1).collect();
    let params = args
        .iter()
        .map(|a| if a.contains(' ') { format!("\"{}\"", a) } else { a.clone() })
        .collect::<Vec<_>>()
        .join(" ");
    let exe_w = HSTRING::from(exe.as_os_str());
    let params_w = HSTRING::from(params.as_str());
    let dir = exe.parent().map(|p| HSTRING::from(p.as_os_str()));

    // SAFETY: 传入的宽字符串在调用期间存活; "runas" 触发 UAC, 由系统决定是否放行。
    let result = unsafe {
        ShellExecuteW(
            None,
            w!("runas"),
            PCWSTR(exe_w.as_ptr()),
            if params.is_empty() {
                PCWSTR::null()
            } else {
                PCWSTR(params_w.as_ptr())
            },
            match &dir {
                Some(d) => PCWSTR(d.as_ptr()),
                None => PCWSTR::null(),
            },
            SW_SHOWNORMAL,
        )
    };
    // ShellExecuteW 的返回值 <=32 表示失败; 用户在 UAC 上点"否"会得到 ERROR_CANCELLED(1223)。
    let code = result.0 as isize;
    if code > 32 {
        return Ok(());
    }
    if code == 1223 {
        return Err(anyhow!("已取消提权(UAC 被拒绝)"));
    }
    Err(anyhow!("提权重启失败(ShellExecuteW 返回 {})", code))
}

#[cfg(not(windows))]
pub fn relaunch_as_admin() -> Result<()> {
    Err(anyhow!("仅 Windows 支持提权重启"))
}

/// 给 UI 用的一句话说明: 当前权限下哪些功能受限。
pub fn limitation_text() -> String {
    if is_elevated() {
        "当前以管理员运行: 固定 COM 口、虚拟摄像头注册与 Global 帧共享均可用".to_string()
    } else {
        "当前为普通权限: 固定游戏 COM 口、虚拟摄像头安装/卸载, 以及帧服务器可见的 Global 共享内存都需要管理员"
            .to_string()
    }
}
