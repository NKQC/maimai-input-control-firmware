//! 激活对象: 帧服务器先由 CLSID 创建它，再经 ActivateObject 取得实际媒体源。
//!
//! 为什么属性全部交给 MF 自己存储: IMFActivate 继承 IMFAttributes；帧服务器会在创建后
//! 先查询属性接口。只转发给 MFCreateAttributes 创建的真实存储，避免自定义属性语义偏差。

use std::ffi::c_void;
use std::sync::{Mutex, MutexGuard};

use windows::Win32::Foundation::E_POINTER;
use windows::Win32::Media::MediaFoundation::{
    IMFActivate, IMFActivate_Impl, IMFAttributes, IMFAttributes_Impl, IMFMediaSource,
    IMFMediaSourceEx, MF_ATTRIBUTE_TYPE, MF_ATTRIBUTES_MATCH_TYPE,
    MF_VIRTUALCAMERA_PROVIDE_ASSOCIATED_CAMERA_SOURCES, MFCreateAttributes,
    MFT_TRANSFORM_CLSID_Attribute,
};
use windows::Win32::System::Com::StructuredStorage::PROPVARIANT;
use windows::core::{
    BOOL, ComObject, GUID, IUnknown, Interface, PCWSTR, PWSTR, Ref, Result, implement,
};

use crate::source::VcamSource;
use crate::{_object_created, _object_released, _trace_guid, _trace_result};

macro_rules! _trace_attr {
    ($method:literal, $key:expr, $details:expr, $result:expr) => {{
        _trace_result(
            || {
                format!(
                    "IMFAttributes::{} key={} {}",
                    $method,
                    _trace_guid($key),
                    $details
                )
            },
            $result,
        )
    }};
}

// ★必须声明 IAgileObject★: C++/WinRT 的 implements 默认让对象 agile(可跨套间直接使用), 官方可用
// 实现即如此; windows-rs 的 #[implement] 默认不 agile, 帧服务器跨套间使用本对象时会退化到编组,
// 而我们没有代理/存根 → 使用侧拿不到对象。
#[implement(IMFActivate, windows::Win32::System::Com::IAgileObject)]
pub(crate) struct VcamActivate {
    _attrs: IMFAttributes,
    _source: Mutex<Option<IMFMediaSource>>,
}

impl VcamActivate {
    pub(crate) fn create() -> Result<ComObject<Self>> {
        let mut attrs = None;
        unsafe { MFCreateAttributes(&mut attrs, 4)? };
        let attrs = attrs.expect("MFCreateAttributes 成功时必有对象");
        // 与官方参考实现一致。
        unsafe {
            attrs.SetUINT32(&MF_VIRTUALCAMERA_PROVIDE_ASSOCIATED_CAMERA_SOURCES, 1)?;
            attrs.SetGUID(&MFT_TRANSFORM_CLSID_Attribute, &crate::CLSID_VCAM_SOURCE)?;
        }
        let source = VcamSource::create(attrs.clone())?;
        let media_source: IMFMediaSource = source.to_interface::<IMFMediaSourceEx>().cast()?;
        let activate = ComObject::new(Self {
            _attrs: attrs,
            _source: Mutex::new(Some(media_source)),
        });
        _object_created();
        Ok(activate)
    }

    /// 中毒锁不传播 panic，避免跨 COM 边界 unwind。
    fn _lock(&self) -> MutexGuard<'_, Option<IMFMediaSource>> {
        self._source.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl Drop for VcamActivate {
    fn drop(&mut self) {
        _object_released();
    }
}

impl IMFAttributes_Impl for VcamActivate_Impl {
    fn GetItem(&self, guidkey: *const GUID, pvalue: *mut PROPVARIANT) -> Result<()> {
        _trace_attr!(
            "GetItem",
            guidkey,
            format!("pvalue_null={}", pvalue.is_null()),
            {
                // ★pvalue 允许为 NULL★: 调用方(包括 MF 平台自身的 MFGetAttribute* 辅助函数)常用
                // GetItem(key, NULL) 仅探测某属性是否存在。无条件当成 Some(ptr) 转发会让内层返回
                // E_POINTER, 帧服务器就是在这里失败的(Start 返回 0x80004003)。
                let value = if pvalue.is_null() { None } else { Some(pvalue) };
                unsafe { self._attrs.GetItem(guidkey, value) }
            }
        )
    }

    fn GetItemType(&self, guidkey: *const GUID) -> Result<MF_ATTRIBUTE_TYPE> {
        _trace_attr!("GetItemType", guidkey, "", unsafe {
            self._attrs.GetItemType(guidkey)
        })
    }

    fn CompareItem(&self, guidkey: *const GUID, value: *const PROPVARIANT) -> Result<BOOL> {
        _trace_attr!(
            "CompareItem",
            guidkey,
            format!("value_null={}", value.is_null()),
            unsafe { self._attrs.CompareItem(guidkey, value) }
        )
    }

    fn Compare(
        &self,
        pattributes: Ref<IMFAttributes>,
        matchtype: MF_ATTRIBUTES_MATCH_TYPE,
    ) -> Result<BOOL> {
        _trace_attr!(
            "Compare",
            std::ptr::null::<GUID>(),
            format!(
                "attributes_null={} match_type={}",
                pattributes.is_null(),
                matchtype.0
            ),
            unsafe { self._attrs.Compare(pattributes.ok()?, matchtype) }
        )
    }

    fn GetUINT32(&self, guidkey: *const GUID) -> Result<u32> {
        _trace_attr!("GetUINT32", guidkey, "", unsafe {
            self._attrs.GetUINT32(guidkey)
        })
    }

    fn GetUINT64(&self, guidkey: *const GUID) -> Result<u64> {
        _trace_attr!("GetUINT64", guidkey, "", unsafe {
            self._attrs.GetUINT64(guidkey)
        })
    }

    fn GetDouble(&self, guidkey: *const GUID) -> Result<f64> {
        _trace_attr!("GetDouble", guidkey, "", unsafe {
            self._attrs.GetDouble(guidkey)
        })
    }

    fn GetGUID(&self, guidkey: *const GUID) -> Result<GUID> {
        _trace_attr!("GetGUID", guidkey, "", unsafe {
            self._attrs.GetGUID(guidkey)
        })
    }

    fn GetStringLength(&self, guidkey: *const GUID) -> Result<u32> {
        _trace_attr!("GetStringLength", guidkey, "", unsafe {
            self._attrs.GetStringLength(guidkey)
        })
    }

    fn GetString(
        &self,
        guidkey: *const GUID,
        pwszvalue: PWSTR,
        cchbufsize: u32,
        pcchlength: *mut u32,
    ) -> Result<()> {
        _trace_attr!(
            "GetString",
            guidkey,
            format!(
                "value_null={} buffer_size={} length_null={}",
                pwszvalue.0.is_null(),
                cchbufsize,
                pcchlength.is_null()
            ),
            {
                let value = if cchbufsize == 0 {
                    &mut []
                } else {
                    unsafe { std::slice::from_raw_parts_mut(pwszvalue.0, cchbufsize as usize) }
                };
                // pcchlength 是可选出参, NULL 合法。
                let length = if pcchlength.is_null() {
                    None
                } else {
                    Some(pcchlength)
                };
                unsafe { self._attrs.GetString(guidkey, value, length) }
            }
        )
    }

    fn GetAllocatedString(
        &self,
        guidkey: *const GUID,
        ppwszvalue: *mut PWSTR,
        pcchlength: *mut u32,
    ) -> Result<()> {
        _trace_attr!(
            "GetAllocatedString",
            guidkey,
            format!(
                "value_out_null={} length_out_null={}",
                ppwszvalue.is_null(),
                pcchlength.is_null()
            ),
            unsafe {
                self._attrs
                    .GetAllocatedString(guidkey, ppwszvalue, pcchlength)
            }
        )
    }

    fn GetBlobSize(&self, guidkey: *const GUID) -> Result<u32> {
        _trace_attr!("GetBlobSize", guidkey, "", unsafe {
            self._attrs.GetBlobSize(guidkey)
        })
    }

    fn GetBlob(
        &self,
        guidkey: *const GUID,
        pbuf: *mut u8,
        cbbufsize: u32,
        pcbblobsize: *mut u32,
    ) -> Result<()> {
        _trace_attr!(
            "GetBlob",
            guidkey,
            format!(
                "buffer_null={} buffer_size={} size_out_null={}",
                pbuf.is_null(),
                cbbufsize,
                pcbblobsize.is_null()
            ),
            {
                let value = if cbbufsize == 0 {
                    &mut []
                } else {
                    unsafe { std::slice::from_raw_parts_mut(pbuf, cbbufsize as usize) }
                };
                // pcbblobsize 同为可选出参。
                let size = if pcbblobsize.is_null() {
                    None
                } else {
                    Some(pcbblobsize)
                };
                unsafe { self._attrs.GetBlob(guidkey, value, size) }
            }
        )
    }

    fn GetAllocatedBlob(
        &self,
        guidkey: *const GUID,
        ppbuf: *mut *mut u8,
        pcbsize: *mut u32,
    ) -> Result<()> {
        _trace_attr!(
            "GetAllocatedBlob",
            guidkey,
            format!(
                "buffer_out_null={} size_out_null={}",
                ppbuf.is_null(),
                pcbsize.is_null()
            ),
            unsafe { self._attrs.GetAllocatedBlob(guidkey, ppbuf, pcbsize) }
        )
    }

    fn GetUnknown(
        &self,
        guidkey: *const GUID,
        riid: *const GUID,
        ppv: *mut *mut c_void,
    ) -> Result<()> {
        let ppv_null = ppv.is_null();
        _trace_attr!(
            "GetUnknown",
            guidkey,
            format!("riid={} value_out_null={ppv_null}", _trace_guid(riid)),
            if riid.is_null() || ppv_null {
                Err(E_POINTER.into())
            } else {
                unsafe { *ppv = std::ptr::null_mut() };
                // windows-rs 的 GetUnknown 是泛型接口封装；先取 IUnknown 后按调用者 riid 再查询。
                let unknown = unsafe { self._attrs.GetUnknown::<IUnknown>(guidkey)? };
                unsafe { unknown.query(riid, ppv).ok() }
            }
        )
    }

    fn SetItem(&self, guidkey: *const GUID, value: *const PROPVARIANT) -> Result<()> {
        _trace_attr!(
            "SetItem",
            guidkey,
            format!("value_null={}", value.is_null()),
            unsafe { self._attrs.SetItem(guidkey, value) }
        )
    }

    fn DeleteItem(&self, guidkey: *const GUID) -> Result<()> {
        _trace_attr!("DeleteItem", guidkey, "", unsafe {
            self._attrs.DeleteItem(guidkey)
        })
    }

    fn DeleteAllItems(&self) -> Result<()> {
        _trace_attr!("DeleteAllItems", std::ptr::null::<GUID>(), "", unsafe {
            self._attrs.DeleteAllItems()
        })
    }

    fn SetUINT32(&self, guidkey: *const GUID, unvalue: u32) -> Result<()> {
        _trace_attr!("SetUINT32", guidkey, format!("value={unvalue}"), unsafe {
            self._attrs.SetUINT32(guidkey, unvalue)
        })
    }

    fn SetUINT64(&self, guidkey: *const GUID, unvalue: u64) -> Result<()> {
        _trace_attr!("SetUINT64", guidkey, format!("value={unvalue}"), unsafe {
            self._attrs.SetUINT64(guidkey, unvalue)
        })
    }

    fn SetDouble(&self, guidkey: *const GUID, fvalue: f64) -> Result<()> {
        _trace_attr!("SetDouble", guidkey, format!("value={fvalue}"), unsafe {
            self._attrs.SetDouble(guidkey, fvalue)
        })
    }

    fn SetGUID(&self, guidkey: *const GUID, guidvalue: *const GUID) -> Result<()> {
        _trace_attr!(
            "SetGUID",
            guidkey,
            format!("value={}", _trace_guid(guidvalue)),
            unsafe { self._attrs.SetGUID(guidkey, guidvalue) }
        )
    }

    fn SetString(&self, guidkey: *const GUID, wszvalue: &PCWSTR) -> Result<()> {
        _trace_attr!(
            "SetString",
            guidkey,
            format!("value_null={}", wszvalue.0.is_null()),
            unsafe { self._attrs.SetString(guidkey, *wszvalue) }
        )
    }

    fn SetBlob(&self, guidkey: *const GUID, pbuf: *const u8, cbsize: u32) -> Result<()> {
        _trace_attr!(
            "SetBlob",
            guidkey,
            format!("buffer_null={} buffer_size={}", pbuf.is_null(), cbsize),
            {
                let value = if cbsize == 0 {
                    &[]
                } else {
                    unsafe { std::slice::from_raw_parts(pbuf, cbsize as usize) }
                };
                unsafe { self._attrs.SetBlob(guidkey, value) }
            }
        )
    }

    fn SetUnknown(&self, guidkey: *const GUID, punkunknown: Ref<IUnknown>) -> Result<()> {
        _trace_attr!(
            "SetUnknown",
            guidkey,
            format!("unknown_null={}", punkunknown.is_null()),
            unsafe { self._attrs.SetUnknown(guidkey, punkunknown.ok()?) }
        )
    }

    fn LockStore(&self) -> Result<()> {
        _trace_attr!("LockStore", std::ptr::null::<GUID>(), "", unsafe {
            self._attrs.LockStore()
        })
    }

    fn UnlockStore(&self) -> Result<()> {
        _trace_attr!("UnlockStore", std::ptr::null::<GUID>(), "", unsafe {
            self._attrs.UnlockStore()
        })
    }

    fn GetCount(&self) -> Result<u32> {
        _trace_attr!("GetCount", std::ptr::null::<GUID>(), "", unsafe {
            self._attrs.GetCount()
        })
    }

    fn GetItemByIndex(
        &self,
        unindex: u32,
        pguidkey: *mut GUID,
        pvalue: *mut PROPVARIANT,
    ) -> Result<()> {
        _trace_attr!(
            "GetItemByIndex",
            std::ptr::null::<GUID>(),
            format!(
                "index={} key_out_null={} value_out_null={}",
                unindex,
                pguidkey.is_null(),
                pvalue.is_null()
            ),
            {
                // 同 GetItem: pvalue 可为 NULL(只取键名时)。
                let value = if pvalue.is_null() { None } else { Some(pvalue) };
                unsafe { self._attrs.GetItemByIndex(unindex, pguidkey, value) }
            }
        )
    }

    fn CopyAllItems(&self, pdest: Ref<IMFAttributes>) -> Result<()> {
        _trace_attr!(
            "CopyAllItems",
            std::ptr::null::<GUID>(),
            format!("destination_null={}", pdest.is_null()),
            unsafe { self._attrs.CopyAllItems(pdest.ok()?) }
        )
    }
}

/// 纯标记接口, 无方法; 声明它即表示本对象 agile。
impl windows::Win32::System::Com::IAgileObject_Impl for VcamActivate_Impl {}

impl IMFActivate_Impl for VcamActivate_Impl {
    fn ActivateObject(&self, riid: *const GUID, ppv: *mut *mut c_void) -> Result<()> {
        let cache_hit = !riid.is_null() && !ppv.is_null() && self._lock().is_some();
        let result = (|| {
            if riid.is_null() || ppv.is_null() {
                return Err(E_POINTER.into());
            }
            unsafe { *ppv = std::ptr::null_mut() };

            // 源已在 create 时与 activate 属性存储一起建立；这里仅返回缓存对象的所请求接口。
            let source = self
                ._lock()
                .as_ref()
                .expect("VcamActivate::create 已创建媒体源")
                .clone();
            unsafe { source.query(riid, ppv).ok() }
        })();
        _trace_result(
            || {
                format!(
                    "IMFActivate::ActivateObject riid={} ppv_null={} cache_hit={}",
                    _trace_guid(riid),
                    ppv.is_null(),
                    cache_hit
                )
            },
            result,
        )
    }

    fn DetachObject(&self) -> Result<()> {
        let result = {
            // Detach 只解除 activate 与源的关联；按 IMFActivate 语义不得关闭仍可能被使用的源。
            _ = self._lock().take();
            Ok(())
        };
        _trace_result(|| "IMFActivate::DetachObject".to_owned(), result)
    }

    fn ShutdownObject(&self) -> Result<()> {
        let result = (|| {
            let source = self._lock().take();
            if let Some(source) = source {
                // 先取出缓存再关闭，重复 ShutdownObject 不会再次触及已关闭的源。
                unsafe { source.Shutdown()? };
            }
            Ok(())
        })();
        _trace_result(|| "IMFActivate::ShutdownObject".to_owned(), result)
    }
}
