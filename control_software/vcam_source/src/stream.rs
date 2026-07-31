//! 媒体流: 唯一一路 RGB32 640x480@30 视频流。
//!
//! 只做 CPU 内存样本, 不接 D3D: 消费端拿到的永远是系统内存缓冲, 省掉设备丢失/共享句柄一整类问题。

use std::ffi::c_void;
use std::sync::{Mutex, MutexGuard};

use windows::Win32::Foundation::{E_POINTER, S_OK};
use windows::Win32::Media::KernelStreaming::PINNAME_VIDEO_CAPTURE;
use windows::Win32::Media::KernelStreaming::{IKsControl, IKsControl_Impl, KSIDENTIFIER};
use windows::Win32::Media::MediaFoundation::{
    IMFAsyncCallback, IMFAsyncResult, IMFAttributes, IMFAttributes_Impl, IMFMediaEvent,
    IMFMediaEventGenerator_Impl, IMFMediaEventQueue, IMFMediaSource, IMFMediaStream_Impl,
    IMFMediaStream2, IMFMediaStream2_Impl, IMFMediaType, IMFMediaTypeHandler_Impl, IMFSample,
    IMFStreamDescriptor, MEDIA_EVENT_GENERATOR_GET_EVENT_FLAGS, MEMediaSample, MEStreamPaused,
    MEStreamStarted, MEStreamStopped, MF_ATTRIBUTE_TYPE, MF_ATTRIBUTES_MATCH_TYPE,
    MF_DEVICESTREAM_ATTRIBUTE_FRAMESOURCE_TYPES, MF_DEVICESTREAM_FRAMESERVER_SHARED,
    MF_DEVICESTREAM_STREAM_CATEGORY, MF_DEVICESTREAM_STREAM_ID, MF_E_INVALIDMEDIATYPE,
    MF_E_INVALIDREQUEST, MF_E_SHUTDOWN, MF_MT_ALL_SAMPLES_INDEPENDENT, MF_MT_AVG_BITRATE,
    MF_MT_DEFAULT_STRIDE, MF_MT_FIXED_SIZE_SAMPLES, MF_MT_FRAME_RATE, MF_MT_FRAME_SIZE,
    MF_MT_INTERLACE_MODE, MF_MT_MAJOR_TYPE, MF_MT_PIXEL_ASPECT_RATIO, MF_MT_SAMPLE_SIZE,
    MF_MT_SUBTYPE, MF_STREAM_STATE, MF_STREAM_STATE_PAUSED, MF_STREAM_STATE_RUNNING,
    MF_STREAM_STATE_STOPPED, MFCreateAttributes, MFCreateMediaType, MFCreateMemoryBuffer,
    MFCreateSample, MFFrameSourceTypes_Color, MFMediaType_Video, MFSampleExtension_Token,
    MFVideoFormat_NV12, MFVideoFormat_RGB32, MFVideoInterlace_Progressive,
};
use windows::Win32::System::Com::StructuredStorage::PROPVARIANT;
use windows::core::{
    BOOL, ComObject, GUID, IUnknown, Interface, PCWSTR, PWSTR, Ref, Result, Weak, implement,
};

use crate::frame::{FrameReader, HEIGHT, RGB32_BYTES, WIDTH};
use crate::{_ks_not_found, _trace_guid, _trace_result};

macro_rules! _trace_stream {
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

/// 30fps: 每帧 100ns 单位时长。
const _FRAME_DURATION: i64 = 333_333;
const _RGB32_STRIDE: i32 = (WIDTH as i32) * 4;
const _NV12_STRIDE: i32 = WIDTH as i32;
const _NV12_BYTES: usize = (WIDTH as usize) * (HEIGHT as usize) * 3 / 2;

/// 构造对外媒体类型；顺序必须保持 RGB32、NV12，与参考实现一致。
pub(crate) fn create_media_types() -> Result<[IMFMediaType; 2]> {
    Ok([
        _create_media_type(MFVideoFormat_RGB32, _RGB32_STRIDE, RGB32_BYTES)?,
        _create_media_type(MFVideoFormat_NV12, _NV12_STRIDE, _NV12_BYTES)?,
    ])
}

fn _create_media_type(subtype: GUID, stride: i32, sample_size: usize) -> Result<IMFMediaType> {
    let mt = unsafe { MFCreateMediaType()? };
    unsafe {
        mt.SetGUID(&MF_MT_MAJOR_TYPE, &MFMediaType_Video)?;
        mt.SetGUID(&MF_MT_SUBTYPE, &subtype)?;
        mt.SetUINT64(&MF_MT_FRAME_SIZE, pack_ratio(WIDTH, HEIGHT))?;
        mt.SetUINT64(&MF_MT_FRAME_RATE, pack_ratio(30, 1))?;
        mt.SetUINT64(&MF_MT_PIXEL_ASPECT_RATIO, pack_ratio(1, 1))?;
        mt.SetUINT32(&MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive.0 as u32)?;
        mt.SetUINT32(&MF_MT_ALL_SAMPLES_INDEPENDENT, 1)?;
        // ★不设 MF_MT_FIXED_SIZE_SAMPLES / MF_MT_SAMPLE_SIZE★: 官方可用实现的媒体类型里没有这两项,
        // 帧服务器在 SHARED 模式下自行按 FRAME_SIZE + 子类型推算样本尺寸。
        mt.SetUINT32(&MF_MT_AVG_BITRATE, (sample_size * 8 * 30) as u32)?;
        mt.SetUINT32(&MF_MT_DEFAULT_STRIDE, stride as u32)?;
    }
    Ok(mt)
}

/// MF 的比值/尺寸类属性统一是高 32 位 + 低 32 位打包。
pub(crate) fn pack_ratio(high: u32, low: u32) -> u64 {
    ((high as u64) << 32) | (low as u64)
}

/// BT.601 full-range RGB→YUV。输入为 MF RGB32 的 BGRA 内存序。
#[inline]
fn _rgb32_to_yuv(px: &[u8]) -> (u8, u8, u8) {
    let (b, g, r) = (px[0] as i32, px[1] as i32, px[2] as i32);
    let y = (77 * r + 150 * g + 29 * b + 128) >> 8;
    let u = ((-43 * r - 85 * g + 128 * b + 128) >> 8) + 128;
    let v = ((128 * r - 107 * g - 21 * b + 128) >> 8) + 128;
    (
        y.clamp(0, 255) as u8,
        u.clamp(0, 255) as u8,
        v.clamp(0, 255) as u8,
    )
}

/// 将一帧紧凑 BGRA 转为紧凑 NV12；色度按 2x2 区块平均。
#[inline]
fn _rgb32_to_nv12(src: &[u8], dst: &mut [u8]) {
    let pixels = (WIDTH * HEIGHT) as usize;
    let (y_plane, uv_plane) = dst.split_at_mut(pixels);
    for (index, px) in src.chunks_exact(4).enumerate() {
        y_plane[index] = _rgb32_to_yuv(px).0;
    }
    for y in (0..HEIGHT as usize).step_by(2) {
        for x in (0..WIDTH as usize).step_by(2) {
            let i = y * WIDTH as usize + x;
            let (_, u0, v0) = _rgb32_to_yuv(&src[i * 4..]);
            let (_, u1, v1) = _rgb32_to_yuv(&src[(i + 1) * 4..]);
            let (_, u2, v2) = _rgb32_to_yuv(&src[(i + WIDTH as usize) * 4..]);
            let (_, u3, v3) = _rgb32_to_yuv(&src[(i + WIDTH as usize + 1) * 4..]);
            let uv = (y / 2) * WIDTH as usize + x;
            uv_plane[uv] = ((u0 as u16 + u1 as u16 + u2 as u16 + u3 as u16 + 2) / 4) as u8;
            uv_plane[uv + 1] = ((v0 as u16 + v1 as u16 + v2 as u16 + v3 as u16 + 2) / 4) as u8;
        }
    }
}

struct _Inner {
    /// 回指媒体源用弱引用: 源强持有流, 反向再强持有会形成循环引用永不释放。
    _source: Weak<IMFMediaSource>,
    _state: MF_STREAM_STATE,
    _shutdown: bool,
    /// 样本时间戳(每帧累加)。
    _time: i64,
    _reader: FrameReader,
    _rgb32: Vec<u8>,
    _current: IMFMediaType,
}

// ★流对象不实现 IMFMediaTypeHandler★: 官方可用实现只实现 IMFMediaStream2/IKsControl/IMFAttributes,
// 媒体类型协商一律走 MFCreateStreamDescriptor 自带的 handler。我们自己顶上一个 handler 会让帧服务器
// 拿到我方实现去枚举类型, 与其预期语义不一致。
#[implement(
    IMFMediaStream2,
    IKsControl,
    IMFAttributes,
    windows::Win32::System::Com::IAgileObject
)]
pub struct VcamStream {
    _queue: IMFMediaEventQueue,
    _attrs: IMFAttributes,
    _desc: IMFStreamDescriptor,
    _types: [IMFMediaType; 2],
    _inner: Mutex<_Inner>,
}

impl VcamStream {
    pub(crate) fn create(
        queue: IMFMediaEventQueue,
        desc: IMFStreamDescriptor,
        types: [IMFMediaType; 2],
    ) -> Result<ComObject<Self>> {
        let mut attrs = None;
        unsafe { MFCreateAttributes(&mut attrs, 2)? };
        let attrs = attrs.expect("MFCreateAttributes 成功时必有对象");
        unsafe {
            attrs.SetUINT32(&MF_DEVICESTREAM_STREAM_ID, 0)?;
            attrs.SetGUID(&MF_DEVICESTREAM_STREAM_CATEGORY, &PINNAME_VIDEO_CAPTURE)?;
            // ★FRAMESOURCE_TYPES 是"流"属性, 不是源属性★: 原来只设在源上, 帧服务器按流去读时读不到,
            // FsProxy 初始化流 0 就以 E_POINTER 失败(见 MF-FrameServer 事件日志)。
            attrs.SetUINT32(
                &MF_DEVICESTREAM_ATTRIBUTE_FRAMESOURCE_TYPES,
                MFFrameSourceTypes_Color.0 as u32,
            )?;
            // 帧服务器以 SHARED 方式初始化本流(日志 StreamType: SHARED), 显式声明可共享。
            attrs.SetUINT32(&MF_DEVICESTREAM_FRAMESERVER_SHARED, 1)?;
        }
        Ok(ComObject::new(Self {
            _queue: queue,
            _attrs: attrs,
            _desc: desc,
            _types: types.clone(),
            _inner: Mutex::new(_Inner {
                _source: Weak::new(),
                _state: MF_STREAM_STATE_STOPPED,
                _shutdown: false,
                _time: 0,
                _reader: FrameReader::new(),
                _rgb32: vec![0; RGB32_BYTES],
                _current: types[0].clone(),
            }),
        }))
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

    pub(crate) fn _set_source(&self, source: Weak<IMFMediaSource>) {
        self._lock()._source = source;
    }

    /// 进入运行态并发 MEStreamStarted(在锁外入队)。
    pub(crate) fn _start(&self, position: *const PROPVARIANT) -> Result<()> {
        {
            let mut g = self._lock();
            if g._shutdown {
                return Err(MF_E_SHUTDOWN.into());
            }
            g._state = MF_STREAM_STATE_RUNNING;
        }
        let fallback = PROPVARIANT::default();
        let pv = if position.is_null() {
            &fallback as *const PROPVARIANT
        } else {
            position
        };
        unsafe {
            self._queue
                .QueueEventParamVar(MEStreamStarted.0 as u32, &GUID::zeroed(), S_OK, pv)
        }
    }

    pub(crate) fn _pause(&self) -> Result<()> {
        {
            let mut g = self._lock();
            if g._shutdown {
                return Err(MF_E_SHUTDOWN.into());
            }
            g._state = MF_STREAM_STATE_PAUSED;
        }
        let empty = PROPVARIANT::default();
        unsafe {
            self._queue
                .QueueEventParamVar(MEStreamPaused.0 as u32, &GUID::zeroed(), S_OK, &empty)
        }
    }

    /// 停止: 时间轴归零, 下次 Start 重新从 0 开始。
    pub(crate) fn _stop(&self) -> Result<()> {
        {
            let mut g = self._lock();
            if g._shutdown {
                return Err(MF_E_SHUTDOWN.into());
            }
            g._state = MF_STREAM_STATE_STOPPED;
            g._time = 0;
        }
        let empty = PROPVARIANT::default();
        unsafe {
            self._queue
                .QueueEventParamVar(MEStreamStopped.0 as u32, &GUID::zeroed(), S_OK, &empty)
        }
    }

    pub(crate) fn _shutdown(&self) {
        let mut g = self._lock();
        if g._shutdown {
            return;
        }
        g._shutdown = true;
        g._state = MF_STREAM_STATE_STOPPED;
        g._source = Weak::new();
        drop(g);
        unsafe { _ = self._queue.Shutdown() };
    }

    /// 仅支持描述符公开的 RGB32/NV12 640x480 类型。
    fn _check_media_type(mt: &IMFMediaType) -> Result<()> {
        let subtype = unsafe { mt.GetGUID(&MF_MT_SUBTYPE)? };
        let size = unsafe { mt.GetUINT64(&MF_MT_FRAME_SIZE)? };
        if (subtype != MFVideoFormat_RGB32 && subtype != MFVideoFormat_NV12)
            || size != pack_ratio(WIDTH, HEIGHT)
        {
            return Err(MF_E_INVALIDMEDIATYPE.into());
        }
        Ok(())
    }

    /// 从 RGB24 共享内存取一帧，按当前媒体类型填充 RGB32 或 NV12 样本。
    fn _make_sample(inner: &mut _Inner) -> Result<IMFSample> {
        let subtype = unsafe { inner._current.GetGUID(&MF_MT_SUBTYPE)? };
        let byte_count = if subtype == MFVideoFormat_NV12 {
            _NV12_BYTES
        } else {
            RGB32_BYTES
        };
        let buffer = unsafe { MFCreateMemoryBuffer(byte_count as u32)? };
        let mut data: *mut u8 = std::ptr::null_mut();
        unsafe { buffer.Lock(&mut data, None, None)? };
        let dst = unsafe { std::slice::from_raw_parts_mut(data, byte_count) };
        if subtype == MFVideoFormat_NV12 {
            inner._reader.fill(&mut inner._rgb32);
            _rgb32_to_nv12(&inner._rgb32, dst);
        } else {
            inner._reader.fill(dst);
        }
        unsafe {
            buffer.Unlock()?;
            buffer.SetCurrentLength(byte_count as u32)?;
        }
        let sample = unsafe { MFCreateSample()? };
        unsafe {
            sample.AddBuffer(&buffer)?;
            sample.SetSampleTime(inner._time)?;
            sample.SetSampleDuration(_FRAME_DURATION)?;
        }
        inner._time += _FRAME_DURATION;
        Ok(sample)
    }
}

impl windows::Win32::System::Com::IAgileObject_Impl for VcamStream_Impl {}

impl IMFAttributes_Impl for VcamStream_Impl {
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

impl IMFMediaEventGenerator_Impl for VcamStream_Impl {
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

impl IMFMediaStream_Impl for VcamStream_Impl {
    fn GetMediaSource(&self) -> Result<IMFMediaSource> {
        _trace_stream!(
            "IMFMediaStream::GetMediaSource",
            "",
            (|| {
                let g = self._lock();
                if g._shutdown {
                    return Err(MF_E_SHUTDOWN.into());
                }
                g._source.upgrade().ok_or_else(|| MF_E_SHUTDOWN.into())
            })()
        )
    }

    fn GetStreamDescriptor(&self) -> Result<IMFStreamDescriptor> {
        _trace_stream!(
            "IMFMediaStream::GetStreamDescriptor",
            "",
            (|| {
                self._alive()?;
                Ok(self._desc.clone())
            })()
        )
    }

    fn RequestSample(&self, ptoken: Ref<IUnknown>) -> Result<()> {
        _trace_stream!(
            "IMFMediaStream::RequestSample",
            format!("token_null={}", ptoken.is_null()),
            (|| {
                let sample = {
                    let mut g = self._lock();
                    if g._shutdown {
                        return Err(MF_E_SHUTDOWN.into());
                    }
                    if g._state != MF_STREAM_STATE_RUNNING {
                        return Err(MF_E_INVALIDREQUEST.into());
                    }
                    VcamStream::_make_sample(&mut g)?
                };
                // 令牌必须原样回传, 消费端靠它配对请求与样本。
                if let Some(token) = ptoken.as_ref() {
                    unsafe { sample.SetUnknown(&MFSampleExtension_Token, token)? };
                }
                unsafe {
                    self._queue.QueueEventParamUnk(
                        MEMediaSample.0 as u32,
                        &GUID::zeroed(),
                        S_OK,
                        &sample,
                    )
                }
            })()
        )
    }
}

impl IMFMediaStream2_Impl for VcamStream_Impl {
    fn SetStreamState(&self, value: MF_STREAM_STATE) -> Result<()> {
        _trace_stream!(
            "IMFMediaStream2::SetStreamState",
            format!("state={value:?}"),
            (|| {
                self._alive()?;
                if value == MF_STREAM_STATE_RUNNING {
                    return self._start(std::ptr::null());
                }
                if value == MF_STREAM_STATE_PAUSED {
                    return self._pause();
                }
                self._stop()
            })()
        )
    }

    fn GetStreamState(&self) -> Result<MF_STREAM_STATE> {
        _trace_stream!(
            "IMFMediaStream2::GetStreamState",
            "",
            (|| {
                let g = self._lock();
                if g._shutdown {
                    return Err(MF_E_SHUTDOWN.into());
                }
                Ok(g._state)
            })()
        )
    }
}

impl IMFMediaTypeHandler_Impl for VcamStream_Impl {
    fn IsMediaTypeSupported(
        &self,
        pmediatype: Ref<IMFMediaType>,
        ppmediatype: windows::core::OutRef<IMFMediaType>,
    ) -> Result<()> {
        let media_type_null = pmediatype.is_null();
        let closest_match_out_null = ppmediatype.is_null();
        _trace_stream!(
            "IMFMediaTypeHandler::IsMediaTypeSupported",
            format!(
                "media_type_null={media_type_null} closest_match_out_null={closest_match_out_null}"
            ),
            (|| {
                self._alive()?;
                if !ppmediatype.is_null() {
                    ppmediatype.write(None)?;
                }
                VcamStream::_check_media_type(pmediatype.ok()?)
            })()
        )
    }

    fn GetMediaTypeCount(&self) -> Result<u32> {
        _trace_stream!(
            "IMFMediaTypeHandler::GetMediaTypeCount",
            "",
            (|| {
                self._alive()?;
                Ok(self._types.len() as u32)
            })()
        )
    }

    fn GetMediaTypeByIndex(&self, dwindex: u32) -> Result<IMFMediaType> {
        _trace_stream!(
            "IMFMediaTypeHandler::GetMediaTypeByIndex",
            format!("index={dwindex}"),
            (|| {
                self._alive()?;
                self._types
                    .get(dwindex as usize)
                    .cloned()
                    .ok_or_else(|| windows::Win32::Foundation::E_INVALIDARG.into())
            })()
        )
    }

    fn SetCurrentMediaType(&self, pmediatype: Ref<IMFMediaType>) -> Result<()> {
        _trace_stream!(
            "IMFMediaTypeHandler::SetCurrentMediaType",
            format!("media_type_null={}", pmediatype.is_null()),
            (|| {
                self._alive()?;
                let mt = pmediatype.ok()?;
                VcamStream::_check_media_type(mt)?;
                let mut g = self._lock();
                if g._shutdown {
                    return Err(MF_E_SHUTDOWN.into());
                }
                g._current = mt.clone();
                Ok(())
            })()
        )
    }

    fn GetCurrentMediaType(&self) -> Result<IMFMediaType> {
        _trace_stream!(
            "IMFMediaTypeHandler::GetCurrentMediaType",
            "",
            (|| {
                let g = self._lock();
                if g._shutdown {
                    return Err(MF_E_SHUTDOWN.into());
                }
                Ok(g._current.clone())
            })()
        )
    }

    fn GetMajorType(&self) -> Result<GUID> {
        _trace_stream!(
            "IMFMediaTypeHandler::GetMajorType",
            "",
            (|| {
                self._alive()?;
                Ok(MFMediaType_Video)
            })()
        )
    }
}

impl IKsControl_Impl for VcamStream_Impl {
    fn KsProperty(
        &self,
        _property: *const KSIDENTIFIER,
        _propertylength: u32,
        _propertydata: *mut core::ffi::c_void,
        _datalength: u32,
        _bytesreturned: *mut u32,
    ) -> Result<()> {
        self._alive()?;
        Err(_ks_not_found())
    }
    fn KsMethod(
        &self,
        _method: *const KSIDENTIFIER,
        _methodlength: u32,
        _methoddata: *mut core::ffi::c_void,
        _datalength: u32,
        _bytesreturned: *mut u32,
    ) -> Result<()> {
        self._alive()?;
        Err(_ks_not_found())
    }
    fn KsEvent(
        &self,
        _event: *const KSIDENTIFIER,
        _eventlength: u32,
        _eventdata: *mut core::ffi::c_void,
        _datalength: u32,
        _bytesreturned: *mut u32,
    ) -> Result<()> {
        self._alive()?;
        Err(_ks_not_found())
    }
}
