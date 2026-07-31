//! 自注册: HKLM\SOFTWARE\Classes\CLSID\{CLSID} 进程内服务器登记。
//!
//! 写 HKLM 而不是 HKCU: 帧服务器/游戏可能以别的用户身份运行, 只有机器级注册才对所有会话可见。

use windows::Win32::Foundation::{ERROR_SUCCESS, HMODULE, MAX_PATH};
use windows::Win32::System::LibraryLoader::{
    GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
    GetModuleFileNameW, GetModuleHandleExW,
};
use windows::Win32::System::Registry::{
    HKEY, HKEY_LOCAL_MACHINE, KEY_WRITE, REG_OPTION_NON_VOLATILE, REG_SZ, RegCloseKey,
    RegCreateKeyExW, RegDeleteTreeW, RegSetValueExW,
};
use windows::core::{PCWSTR, Result};

use crate::{_hresult_from_win32, CLSID_TEXT, FRIENDLY_NAME};

/// CLSID 键相对 HKLM 的路径。
fn _clsid_key() -> Vec<u16> {
    _wide(&format!("SOFTWARE\\Classes\\CLSID\\{CLSID_TEXT}"))
}

/// UTF-16 + 结尾 NUL。
fn _wide(text: &str) -> Vec<u16> {
    text.encode_utf16().chain(std::iter::once(0)).collect()
}

/// REG_SZ 的字节表示(含结尾 NUL)。
fn _sz_bytes(text: &[u16]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(text.len() * 2);
    for unit in text {
        bytes.extend_from_slice(&unit.to_le_bytes());
    }
    bytes
}

/// 本 DLL 自身的完整路径: 注册表必须写实际落盘位置, 不能靠调用方传。
fn _module_path() -> Result<Vec<u16>> {
    let mut module = HMODULE::default();
    unsafe {
        GetModuleHandleExW(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            PCWSTR(_module_path as *const u16),
            &mut module,
        )?
    };
    let mut buffer = [0u16; MAX_PATH as usize];
    let len = unsafe { GetModuleFileNameW(Some(module), &mut buffer) } as usize;
    if len == 0 || len >= buffer.len() {
        let code = unsafe { windows::Win32::Foundation::GetLastError() };
        return Err(_hresult_from_win32(code.0).into());
    }
    let mut path: Vec<u16> = buffer[..len].to_vec();
    path.push(0);
    Ok(path)
}

/// 创建子键并写默认值(可选再写一个命名值)。
fn _write_key(
    subkey: &[u16],
    default_value: &[u16],
    named: Option<(&[u16], &[u16])>,
) -> Result<()> {
    let mut key = HKEY::default();
    let status = unsafe {
        RegCreateKeyExW(
            HKEY_LOCAL_MACHINE,
            PCWSTR(subkey.as_ptr()),
            None,
            PCWSTR::null(),
            REG_OPTION_NON_VOLATILE,
            KEY_WRITE,
            None,
            &mut key,
            None,
        )
    };
    if status != ERROR_SUCCESS {
        return Err(_hresult_from_win32(status.0).into());
    }
    let result = (|| -> Result<()> {
        let data = _sz_bytes(default_value);
        let status =
            unsafe { RegSetValueExW(key, PCWSTR::null(), None, REG_SZ, Some(data.as_slice())) };
        if status != ERROR_SUCCESS {
            return Err(_hresult_from_win32(status.0).into());
        }
        if let Some((name, value)) = named {
            let data = _sz_bytes(value);
            let status = unsafe {
                RegSetValueExW(
                    key,
                    PCWSTR(name.as_ptr()),
                    None,
                    REG_SZ,
                    Some(data.as_slice()),
                )
            };
            if status != ERROR_SUCCESS {
                return Err(_hresult_from_win32(status.0).into());
            }
        }
        Ok(())
    })();
    unsafe { _ = RegCloseKey(key) };
    result
}

pub(crate) fn register() -> Result<()> {
    let clsid_key = _clsid_key();
    _write_key(&clsid_key, &_wide(FRIENDLY_NAME), None)?;

    let mut inproc = clsid_key.clone();
    inproc.pop(); // 去掉 NUL 再拼子路径
    inproc.extend(_wide("\\InprocServer32"));
    // ThreadingModel=Both: 源对象是 agile 的, 任意套间都能直接用, 省掉编组开销。
    _write_key(
        &inproc,
        &_module_path()?,
        Some((&_wide("ThreadingModel"), &_wide("Both"))),
    )
}

pub(crate) fn unregister() -> Result<()> {
    let status = unsafe { RegDeleteTreeW(HKEY_LOCAL_MACHINE, PCWSTR(_clsid_key().as_ptr())) };
    if status != ERROR_SUCCESS {
        return Err(_hresult_from_win32(status.0).into());
    }
    Ok(())
}
