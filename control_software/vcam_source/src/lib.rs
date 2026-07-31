//! mai2control 虚拟摄像头媒体源(Windows Media Foundation 进程内 COM 服务器)。
//!
//! 为什么独立成 cdylib: 媒体源必须由消费端(帧服务器/游戏)进程加载, 无法编进上位机 exe;
//! 画面通过命名共享内存从上位机单向传入, 因此本 DLL 与上位机之间没有任何编译期耦合。

// COM 方法名必须与接口签名一致, 不能改成 snake_case。
#![allow(non_snake_case)]

// #[implement] 生成的代码写死 ::windows_core 绝对路径。依赖只允许 windows 0.62.2, 因此把本 crate
// 自身别名成 windows_core, 再原样转出 windows::core 的全部条目(含隐藏的 imp), 供宏生成代码解析。
extern crate self as windows_core;
pub use windows::core::*;

mod activate;
mod frame;
mod reg;
mod source;
mod stream;
mod trace;

use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use windows::Win32::Foundation::{
    CLASS_E_CLASSNOTAVAILABLE, CLASS_E_NOAGGREGATION, E_POINTER, ERROR_SET_NOT_FOUND, S_FALSE, S_OK,
};
use windows::Win32::Media::MediaFoundation::{MF_VERSION, MFSTARTUP_FULL, MFStartup};
use windows::Win32::System::Com::{IClassFactory, IClassFactory_Impl};
// BOOL / Error / GUID / HRESULT / IUnknown / Interface / Ref / Result / implement 由上面
// `pub use windows::core::*` 引入, 不再重复 import(否则会遮蔽公开的 glob 转出)。

pub(crate) use trace::{_trace_guid, _trace_hresult, _trace_result};

/// 媒体源 CLSID: {B7C5F1A2-3D64-4E8B-9A11-2F6C8D0E4A73}
pub const CLSID_VCAM_SOURCE: GUID = GUID::from_u128(0xB7C5F1A2_3D64_4E8B_9A11_2F6C8D0E4A73);
/// 注册表键名用的字符串形式(必须与 CLSID 一致)。
pub const CLSID_TEXT: &str = "{B7C5F1A2-3D64-4E8B-9A11-2F6C8D0E4A73}";
/// 注册表默认值: 消费端界面上显示的友好名。
pub const FRIENDLY_NAME: &str = "mai2control Virtual Camera";

/// 存活对象计数: DllCanUnloadNow 只有归零才允许卸载。
static _OBJECTS: AtomicI32 = AtomicI32::new(0);
/// MF 平台只启动一次, 且不再 MFShutdown: 宿主可能仍持有 MF 对象, 抢先关闭会踩到别人。
static _MF_STARTED: AtomicBool = AtomicBool::new(false);

pub(crate) fn _object_created() {
    _OBJECTS.fetch_add(1, Ordering::AcqRel);
}

pub(crate) fn _object_released() {
    _OBJECTS.fetch_sub(1, Ordering::AcqRel);
}

/// IKsControl 一律回报"属性集不存在", 表明本源不支持内核流控制。
pub(crate) fn _ks_not_found() -> Error {
    Error::from_hresult(_hresult_from_win32(ERROR_SET_NOT_FOUND.0))
}

pub(crate) const fn _hresult_from_win32(code: u32) -> HRESULT {
    HRESULT((0x8007_0000u32 | (code & 0xFFFF)) as i32)
}

fn _ensure_mf() -> Result<()> {
    if _MF_STARTED.load(Ordering::Acquire) {
        return Ok(());
    }
    unsafe { MFStartup(MF_VERSION, MFSTARTUP_FULL)? };
    _MF_STARTED.store(true, Ordering::Release);
    Ok(())
}

#[implement(IClassFactory)]
struct _Factory;

impl IClassFactory_Impl for _Factory_Impl {
    fn CreateInstance(
        &self,
        punkouter: Ref<IUnknown>,
        riid: *const GUID,
        ppvobject: *mut *mut c_void,
    ) -> Result<()> {
        let result = (|| {
            if riid.is_null() || ppvobject.is_null() {
                return Err(E_POINTER.into());
            }
            unsafe { *ppvobject = std::ptr::null_mut() };
            if !punkouter.is_null() {
                return Err(CLASS_E_NOAGGREGATION.into());
            }
            _ensure_mf()?;
            // 注册 CLSID 必须返回 IMFActivate；帧服务器随后由 ActivateObject 取得实际媒体源。
            let activate = activate::VcamActivate::create()?;
            let unknown = activate.to_interface::<IUnknown>();
            unsafe { unknown.query(riid, ppvobject).ok() }
        })();
        _trace_result(
            || {
                format!(
                    "IClassFactory::CreateInstance riid={} ppv_null={}",
                    _trace_guid(riid),
                    ppvobject.is_null()
                )
            },
            result,
        )
    }

    fn LockServer(&self, flock: BOOL) -> Result<()> {
        if flock.as_bool() {
            _object_created();
        } else {
            _object_released();
        }
        Ok(())
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "system" fn DllGetClassObject(
    rclsid: *const GUID,
    riid: *const GUID,
    ppv: *mut *mut c_void,
) -> HRESULT {
    let result = (|| {
        if rclsid.is_null() || riid.is_null() || ppv.is_null() {
            return E_POINTER;
        }
        unsafe { *ppv = std::ptr::null_mut() };
        if unsafe { *rclsid } != CLSID_VCAM_SOURCE {
            return CLASS_E_CLASSNOTAVAILABLE;
        }
        let factory: IClassFactory = _Factory.into();
        unsafe { factory.query(riid, ppv) }
    })();
    _trace_hresult(
        || {
            format!(
                "DllGetClassObject rclsid={} riid={} ppv_null={}",
                _trace_guid(rclsid),
                _trace_guid(riid),
                ppv.is_null(),
            )
        },
        result,
    )
}

#[unsafe(no_mangle)]
pub extern "system" fn DllCanUnloadNow() -> HRESULT {
    if _OBJECTS.load(Ordering::Acquire) > 0 {
        S_FALSE
    } else {
        S_OK
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn DllRegisterServer() -> HRESULT {
    _trace_hresult(
        || "DllRegisterServer".to_owned(),
        match reg::register() {
            Ok(()) => S_OK,
            Err(e) => e.code(),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "system" fn DllUnregisterServer() -> HRESULT {
    _trace_hresult(
        || "DllUnregisterServer".to_owned(),
        match reg::unregister() {
            Ok(()) => S_OK,
            Err(e) => e.code(),
        },
    )
}
