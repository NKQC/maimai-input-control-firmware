//! DLL 进程内调用轨迹。
//!
//! 帧服务器可能以受限服务账户加载 DLL，因此日志完全是尽力而为：任何格式化或 I/O
//! 异常都被吞掉，绝不改变 COM 方法的返回值或让异常跨越 COM 边界。

use std::fmt::Write as _;
use std::fs::OpenOptions;
use std::io::Write as _;
use std::panic::{AssertUnwindSafe, catch_unwind};

use windows::Win32::System::SystemInformation::GetLocalTime;
use windows::core::{GUID, HRESULT, Result};

const _LOG_PATH: &str = r"C:\ProgramData\mai2control\vcam_dll.log";

/// 输出一行 DLL 调用日志；目录不存在或服务账户无权写入时一律忽略。
pub(crate) fn _trace(msg: &str) {
    _trace_with(|| msg.to_owned());
}

/// 仅在日志构造成功时写入，避免调试字符串本身影响 COM 调用。
pub(crate) fn _trace_result<T, F>(message: F, result: Result<T>) -> Result<T>
where
    F: FnOnce() -> String,
{
    let hr = match &result {
        Ok(_) => 0,
        Err(error) => error.code().0 as u32,
    };
    _trace_with(|| {
        let mut text = message();
        let _ = write!(text, " hr=0x{hr:08X}");
        text
    });
    result
}

/// 为 extern "system" 导出函数记录其原始 HRESULT。
pub(crate) fn _trace_hresult<F>(message: F, result: HRESULT) -> HRESULT
where
    F: FnOnce() -> String,
{
    let hr = result.0 as u32;
    _trace_with(|| {
        let mut text = message();
        let _ = write!(text, " hr=0x{hr:08X}");
        text
    });
    result
}

/// 只在日志构造闭包内调用，避免 NULL GUID 解引用。
pub(crate) fn _trace_guid(guid: *const GUID) -> String {
    if guid.is_null() {
        "NULL".to_owned()
    } else {
        format!("{:?}", unsafe { *guid })
    }
}

fn _trace_with<F>(message: F)
where
    F: FnOnce() -> String,
{
    let _ = catch_unwind(AssertUnwindSafe(|| _trace_write(&message())));
}

fn _trace_write(message: &str) {
    // SAFETY: GetLocalTime 仅返回当前线程可读取的 SYSTEMTIME 值。
    let now = unsafe { GetLocalTime() };
    let process_name = match std::env::current_exe() {
        Ok(path) => match path.file_name() {
            Some(name) => name.to_string_lossy().into_owned(),
            None => "<unknown>".to_owned(),
        },
        Err(_) => "<unknown>".to_owned(),
    };
    let line = format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03} pid={} process={} {}",
        now.wYear,
        now.wMonth,
        now.wDay,
        now.wHour,
        now.wMinute,
        now.wSecond,
        now.wMilliseconds,
        std::process::id(),
        process_name,
        message,
    );
    if let Ok(mut file) = OpenOptions::new().create(true).append(true).open(_LOG_PATH) {
        let _ = writeln!(file, "{line}");
    }
}
