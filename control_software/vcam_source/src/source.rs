//! 媒体源: 单流实时视频源(MFMEDIASOURCE_IS_LIVE)。
//!
//! 生命周期严格按 IMFMediaSource 规范: Start 先在源队列发 MENewStream/MEUpdatedStream,
//! 再由流发 MEStreamStarted, 最后源发 MESourceStarted; Shutdown 之后一切方法返回 MF_E_SHUTDOWN。

use std::ffi::c_void;
use std::sync::{Mutex, MutexGuard};

use windows::Win32::Foundation::{E_NOTIMPL, E_POINTER, S_OK};
// KSCAMERAPROFILE_* 在 KernelStreaming 而不是 MediaFoundation 下。
use windows::Win32::Media::KernelStreaming::{
    IKsControl, IKsControl_Impl, KSCAMERAPROFILE_HighFrameRate, KSCAMERAPROFILE_Legacy,
    KSIDENTIFIER,
};
use windows::Win32::Media::MediaFoundation::{
    IMFAsyncCallback, IMFAsyncResult, IMFAttributes, IMFAttributes_Impl, IMFGetService,
    IMFGetService_Impl, IMFMediaEvent, IMFMediaEventGenerator_Impl, IMFMediaEventQueue,
    IMFMediaSource, IMFMediaSource_Impl, IMFMediaSourceEx, IMFMediaSourceEx_Impl,
    IMFPresentationDescriptor, IMFSampleAllocatorControl, IMFSampleAllocatorControl_Impl,
    MEDIA_EVENT_GENERATOR_GET_EVENT_FLAGS, MENewStream, MESourcePaused, MESourceStarted,
    MESourceStopped, MEUpdatedStream, MF_ATTRIBUTE_TYPE, MF_ATTRIBUTES_MATCH_TYPE,
    MF_DEVICEMFT_SENSORPROFILE_COLLECTION, MF_E_INVALIDREQUEST, MF_E_SHUTDOWN,
    MF_E_UNSUPPORTED_SERVICE, MF_E_UNSUPPORTED_TIME_FORMAT, MF_STREAM_STATE,
    MF_STREAM_STATE_PAUSED, MF_STREAM_STATE_RUNNING, MF_STREAM_STATE_STOPPED, MFCreateAttributes,
    MFCreateEventQueue, MFCreatePresentationDescriptor, MFCreateSensorProfile,
    MFCreateSensorProfileCollection, MFCreateStreamDescriptor, MFMEDIASOURCE_IS_LIVE,
    MFSampleAllocatorUsage, MFSampleAllocatorUsage_DoesNotAllocate,
};
use windows::Win32::System::Com::StructuredStorage::PROPVARIANT;
use windows::core::w;
use windows::core::{
    BOOL, ComObject, GUID, IUnknown, IUnknownImpl, Interface, PCWSTR, PWSTR, Ref, Result, implement,
};

use crate::stream::{VcamStream, create_media_types};
use crate::{_ks_not_found, _object_created, _object_released, _trace_guid, _trace_result};

macro_rules! _trace_source {
    ($method:literal, $details:expr, $result:expr) => {{ _trace_result(|| format!("{} {}", $method, $details), $result) }};
}

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

struct _Inner {
    _state: MF_STREAM_STATE,
    _shutdown: bool,
    /// 是否已经公布过流: 首次 Start 用 MENewStream, 之后用 MEUpdatedStream。
    _announced: bool,
}

// IAgileObject: 同 activate, 与官方 C++/WinRT 实现的 agile 语义对齐。
#[implement(
    IMFMediaSourceEx,
    IMFAttributes,
    IMFGetService,
    IKsControl,
    IMFSampleAllocatorControl,
    windows::Win32::System::Com::IAgileObject
)]
pub struct VcamSource {
    _queue: IMFMediaEventQueue,
    _attrs: IMFAttributes,
    _pd: IMFPresentationDescriptor,
    _stream: ComObject<VcamStream>,
    _inner: Mutex<_Inner>,
}

impl VcamSource {
    pub(crate) fn create(activate_attrs: IMFAttributes) -> Result<ComObject<Self>> {
        // Keep descriptor order aligned with the Microsoft sample: RGB32 first, NV12 second.
        let types = create_media_types()?;
        let sd = unsafe {
            MFCreateStreamDescriptor(0, &[Some(types[0].clone()), Some(types[1].clone())])?
        };
        unsafe { sd.GetMediaTypeHandler()?.SetCurrentMediaType(&types[0])? };
        let pd = unsafe { MFCreatePresentationDescriptor(Some(&[Some(sd.clone())]))? };
        unsafe { pd.SelectStream(0)? };

        let mut attrs = None;
        unsafe { MFCreateAttributes(&mut attrs, 8)? };
        let attrs = attrs.expect("MFCreateAttributes 成功时必有对象");
        unsafe {
            activate_attrs.CopyAllItems(&attrs)?;
            // ★MF_DEVICESTREAM_ATTRIBUTE_FRAMESOURCE_TYPES 只设在流属性上, 源属性上绝不设★:
            // 官方可用实现(VCamSampleSource)源属性里只有 CLSID/关联源标志/传感器配置集合三项;
            // 源上多出 MF_DEVICESTREAM_* 会让帧服务器按另一套(设备 MFT)语义枚举流。

            // ★必须提供传感器配置集合★: 缺了它帧服务器认为本源"0 个流", FsProxy 初始化直接以
            // E_POINTER 失败(实测事件日志里我们是"流: 0", 可用参考实现是"流: 1")。
            // 同时声明常规与高帧率档，让帧服务器按其目标帧率选择匹配的配置。
            let collection = MFCreateSensorProfileCollection()?;
            let legacy = MFCreateSensorProfile(&KSCAMERAPROFILE_Legacy, 0, PCWSTR::null())?;
            legacy.AddProfileFilter(0, w!("((RES==;FRT<=30,1;SUT==))"))?;
            collection.AddProfile(&legacy)?;
            let high_frame_rate =
                MFCreateSensorProfile(&KSCAMERAPROFILE_HighFrameRate, 0, PCWSTR::null())?;
            high_frame_rate.AddProfileFilter(0, w!("((RES==;FRT>=60,1;SUT==))"))?;
            collection.AddProfile(&high_frame_rate)?;
            attrs.SetUnknown(&MF_DEVICEMFT_SENSORPROFILE_COLLECTION, &collection)?;
        }

        let stream = VcamStream::create(unsafe { MFCreateEventQueue()? }, sd, types)?;
        let source = ComObject::new(Self {
            _queue: unsafe { MFCreateEventQueue()? },
            _attrs: attrs,
            _pd: pd,
            _stream: stream,
            _inner: Mutex::new(_Inner {
                _state: MF_STREAM_STATE_STOPPED,
                _shutdown: false,
                _announced: false,
            }),
        });
        let interface: IMFMediaSource = source.to_interface::<IMFMediaSourceEx>().cast()?;
        source._stream.get()._set_source(interface.downgrade()?);
        _object_created();
        Ok(source)
    }

    /// 中毒锁不再传播 panic: COM 边界不能 unwind。
    fn _lock(&self) -> MutexGuard<'_, _Inner> {
        self._inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// 所有对外 COM 方法的第一道闸: Shutdown 之后一律 MF_E_SHUTDOWN。
    /// 返回前锁已释放, 后续对外部接口的调用都发生在锁外。
    #[inline]
    fn _alive(&self) -> Result<()> {
        if self._lock()._shutdown {
            return Err(MF_E_SHUTDOWN.into());
        }
        Ok(())
    }

    /// 校验时间格式: 只支持默认(100ns)时间轴。
    fn _check_time_format(format: *const GUID) -> Result<()> {
        if format.is_null() {
            return Ok(());
        }
        if unsafe { *format } == GUID::zeroed() {
            return Ok(());
        }
        Err(MF_E_UNSUPPORTED_TIME_FORMAT.into())
    }
}

impl Drop for VcamSource {
    fn drop(&mut self) {
        _object_released();
    }
}

impl windows::Win32::System::Com::IAgileObject_Impl for VcamSource_Impl {}

impl IMFAttributes_Impl for VcamSource_Impl {
    fn GetItem(&self, guidkey: *const GUID, pvalue: *mut PROPVARIANT) -> Result<()> {
        _trace_attr!(
            "GetItem",
            guidkey,
            format!("pvalue_null={}", pvalue.is_null()),
            {
                // pvalue 允许为 NULL，帧服务器可仅探测属性是否存在。
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
                // pcchlength 是可选出参，NULL 合法。
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
                // pvalue 可为 NULL，只取键名时必须原样转发。
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

impl IMFMediaEventGenerator_Impl for VcamSource_Impl {
    fn GetEvent(&self, dwflags: MEDIA_EVENT_GENERATOR_GET_EVENT_FLAGS) -> Result<IMFMediaEvent> {
        self._alive()?;
        unsafe { self._queue.GetEvent(dwflags.0) }
    }
    fn BeginGetEvent(
        &self,
        pcallback: Ref<IMFAsyncCallback>,
        punkstate: Ref<IUnknown>,
    ) -> Result<()> {
        self._alive()?;
        unsafe {
            self._queue
                .BeginGetEvent(pcallback.ok()?, punkstate.as_ref())
        }
    }
    fn EndGetEvent(&self, presult: Ref<IMFAsyncResult>) -> Result<IMFMediaEvent> {
        self._alive()?;
        unsafe { self._queue.EndGetEvent(presult.ok()?) }
    }
    fn QueueEvent(
        &self,
        met: u32,
        guidextendedtype: *const GUID,
        hrstatus: windows::core::HRESULT,
        pvvalue: *const PROPVARIANT,
    ) -> Result<()> {
        self._alive()?;
        unsafe {
            self._queue
                .QueueEventParamVar(met, guidextendedtype, hrstatus, pvvalue)
        }
    }
}

impl IMFMediaSource_Impl for VcamSource_Impl {
    fn GetCharacteristics(&self) -> Result<u32> {
        _trace_source!(
            "IMFMediaSource::GetCharacteristics",
            "",
            (|| {
                self._alive()?;
                Ok(MFMEDIASOURCE_IS_LIVE.0 as u32)
            })()
        )
    }

    fn CreatePresentationDescriptor(&self) -> Result<IMFPresentationDescriptor> {
        _trace_source!(
            "IMFMediaSource::CreatePresentationDescriptor",
            "",
            (|| {
                self._alive()?;
                unsafe { self._pd.Clone() }
            })()
        )
    }

    fn Start(
        &self,
        ppresentationdescriptor: Ref<IMFPresentationDescriptor>,
        pguidtimeformat: *const GUID,
        pvarstartposition: *const PROPVARIANT,
    ) -> Result<()> {
        _trace_source!(
            "IMFMediaSource::Start",
            format!(
                "presentation_descriptor_null={} time_format={} start_position_null={}",
                ppresentationdescriptor.is_null(),
                _trace_guid(pguidtimeformat),
                pvarstartposition.is_null()
            ),
            (|| {
                self._alive()?;
                VcamSource::_check_time_format(pguidtimeformat)?;
                let pd = ppresentationdescriptor.ok()?;
                // 消费端可能只传来未选中流的描述符, 这里确保唯一一路流被选中。
                unsafe { _ = pd.SelectStream(0) };

                let announced = {
                    let mut g = self._lock();
                    if g._shutdown {
                        return Err(MF_E_SHUTDOWN.into());
                    }
                    let announced = g._announced;
                    g._announced = true;
                    g._state = MF_STREAM_STATE_RUNNING;
                    announced
                };

                // 锁外入队, 避免在持锁时回调外部接口。
                let event = if announced {
                    MEUpdatedStream
                } else {
                    MENewStream
                };
                let stream = self._stream.to_interface::<IUnknown>();
                unsafe {
                    self._queue.QueueEventParamUnk(
                        event.0 as u32,
                        &GUID::zeroed(),
                        S_OK,
                        &stream,
                    )?
                };
                self._stream.get()._start(pvarstartposition)?;

                let fallback = PROPVARIANT::default();
                let position = if pvarstartposition.is_null() {
                    &fallback as *const PROPVARIANT
                } else {
                    pvarstartposition
                };
                unsafe {
                    self._queue.QueueEventParamVar(
                        MESourceStarted.0 as u32,
                        &GUID::zeroed(),
                        S_OK,
                        position,
                    )
                }
            })()
        )
    }

    fn Stop(&self) -> Result<()> {
        _trace_source!(
            "IMFMediaSource::Stop",
            "",
            (|| {
                {
                    let mut g = self._lock();
                    if g._shutdown {
                        return Err(MF_E_SHUTDOWN.into());
                    }
                    g._state = MF_STREAM_STATE_STOPPED;
                }
                self._stream.get()._stop()?;
                let empty = PROPVARIANT::default();
                unsafe {
                    self._queue.QueueEventParamVar(
                        MESourceStopped.0 as u32,
                        &GUID::zeroed(),
                        S_OK,
                        &empty,
                    )
                }
            })()
        )
    }

    fn Pause(&self) -> Result<()> {
        _trace_source!(
            "IMFMediaSource::Pause",
            "",
            (|| {
                {
                    let mut g = self._lock();
                    if g._shutdown {
                        return Err(MF_E_SHUTDOWN.into());
                    }
                    if g._state != MF_STREAM_STATE_RUNNING {
                        return Err(MF_E_INVALIDREQUEST.into());
                    }
                    g._state = MF_STREAM_STATE_PAUSED;
                }
                self._stream.get()._pause()?;
                let empty = PROPVARIANT::default();
                unsafe {
                    self._queue.QueueEventParamVar(
                        MESourcePaused.0 as u32,
                        &GUID::zeroed(),
                        S_OK,
                        &empty,
                    )
                }
            })()
        )
    }

    fn Shutdown(&self) -> Result<()> {
        _trace_source!(
            "IMFMediaSource::Shutdown",
            "",
            (|| {
                {
                    let mut g = self._lock();
                    if g._shutdown {
                        return Err(MF_E_SHUTDOWN.into());
                    }
                    g._shutdown = true;
                    g._state = MF_STREAM_STATE_STOPPED;
                }
                self._stream.get()._shutdown();
                unsafe { self._queue.Shutdown() }
            })()
        )
    }
}

impl IMFMediaSourceEx_Impl for VcamSource_Impl {
    fn GetSourceAttributes(&self) -> Result<IMFAttributes> {
        _trace_source!(
            "IMFMediaSourceEx::GetSourceAttributes",
            "return=self",
            (|| {
                self._alive()?;
                // 帧服务器会在返回对象上继续 QI IMFMediaSourceEx/IMFGetService/IKsControl。
                Ok(self.to_object().to_interface::<IMFAttributes>())
            })()
        )
    }

    fn GetStreamAttributes(&self, dwstreamidentifier: u32) -> Result<IMFAttributes> {
        _trace_source!(
            "IMFMediaSourceEx::GetStreamAttributes",
            format!("stream_id={dwstreamidentifier}"),
            (|| {
                self._alive()?;
                if dwstreamidentifier != 0 {
                    return Err(windows::Win32::Foundation::E_INVALIDARG.into());
                }
                // 平台需由属性对象继续 QI 流接口，故返回流自身而非底层属性存储。
                Ok(self._stream.to_interface::<IMFAttributes>())
            })()
        )
    }

    /// 纯 CPU 输出, 不使用 D3D; 但必须返回 S_OK, 否则帧服务器会认为源不可用。
    fn SetD3DManager(&self, pmanager: Ref<IUnknown>) -> Result<()> {
        _trace_source!(
            "IMFMediaSourceEx::SetD3DManager",
            format!("manager_null={}", pmanager.is_null()),
            (|| {
                self._alive()?;
                Ok(())
            })()
        )
    }
}

impl IMFSampleAllocatorControl_Impl for VcamSource_Impl {
    fn SetDefaultAllocator(
        &self,
        dwoutputstreamid: u32,
        // windows 0.62 的绑定里这个参数是裸 Ref<IUnknown>(不是 IMFSampleAllocator)。
        pallocator: Ref<IUnknown>,
    ) -> Result<()> {
        _trace_source!(
            "IMFSampleAllocatorControl::SetDefaultAllocator",
            format!(
                "output_stream_id={} allocator_null={}",
                dwoutputstreamid,
                pallocator.is_null()
            ),
            (|| {
                self._alive()?;
                // 帧数据来自共享内存，并由 MFCreateMemoryBuffer 创建系统内存缓冲；不接受帧服务器的 D3D 分配器。
                Err(E_NOTIMPL.into())
            })()
        )
    }

    fn GetAllocatorUsage(
        &self,
        dwoutputstreamid: u32,
        pdwinputstreamid: *mut u32,
        peusage: *mut MFSampleAllocatorUsage,
    ) -> Result<()> {
        _trace_source!(
            "IMFSampleAllocatorControl::GetAllocatorUsage",
            format!(
                "output_stream_id={} input_stream_out_null={} usage_out_null={}",
                dwoutputstreamid,
                pdwinputstreamid.is_null(),
                peusage.is_null()
            ),
            (|| {
                self._alive()?;
                if pdwinputstreamid.is_null() || peusage.is_null() {
                    return Err(windows::Win32::Foundation::E_POINTER.into());
                }
                unsafe {
                    *pdwinputstreamid = dwoutputstreamid;
                    *peusage = MFSampleAllocatorUsage_DoesNotAllocate;
                }
                Ok(())
            })()
        )
    }
}

impl IMFGetService_Impl for VcamSource_Impl {
    fn GetService(
        &self,
        guidservice: *const GUID,
        riid: *const GUID,
        ppvobject: *mut *mut core::ffi::c_void,
    ) -> Result<()> {
        _trace_source!(
            "IMFGetService::GetService",
            format!(
                "service={} riid={} value_out_null={}",
                _trace_guid(guidservice),
                _trace_guid(riid),
                ppvobject.is_null()
            ),
            (|| {
                self._alive()?;
                Err(MF_E_UNSUPPORTED_SERVICE.into())
            })()
        )
    }
}

impl IKsControl_Impl for VcamSource_Impl {
    fn KsProperty(
        &self,
        property: *const KSIDENTIFIER,
        propertylength: u32,
        propertydata: *mut core::ffi::c_void,
        datalength: u32,
        bytesreturned: *mut u32,
    ) -> Result<()> {
        _trace_source!(
            "IKsControl::KsProperty",
            format!(
                "property_null={} property_length={} data_null={} data_length={} bytes_out_null={}",
                property.is_null(),
                propertylength,
                propertydata.is_null(),
                datalength,
                bytesreturned.is_null()
            ),
            (|| {
                self._alive()?;
                Err(_ks_not_found())
            })()
        )
    }
    fn KsMethod(
        &self,
        method: *const KSIDENTIFIER,
        methodlength: u32,
        methoddata: *mut core::ffi::c_void,
        datalength: u32,
        bytesreturned: *mut u32,
    ) -> Result<()> {
        _trace_source!(
            "IKsControl::KsMethod",
            format!(
                "method_null={} method_length={} data_null={} data_length={} bytes_out_null={}",
                method.is_null(),
                methodlength,
                methoddata.is_null(),
                datalength,
                bytesreturned.is_null()
            ),
            (|| {
                self._alive()?;
                Err(_ks_not_found())
            })()
        )
    }
    fn KsEvent(
        &self,
        event: *const KSIDENTIFIER,
        eventlength: u32,
        eventdata: *mut core::ffi::c_void,
        datalength: u32,
        bytesreturned: *mut u32,
    ) -> Result<()> {
        _trace_source!(
            "IKsControl::KsEvent",
            format!(
                "event_null={} event_length={} data_null={} data_length={} bytes_out_null={}",
                event.is_null(),
                eventlength,
                eventdata.is_null(),
                datalength,
                bytesreturned.is_null()
            ),
            (|| {
                self._alive()?;
                Err(_ks_not_found())
            })()
        )
    }
}
