//! 共享内存帧读取。
//!
//! 上位机是生产者(写), 本媒体源只读; 双方靠命名映射解耦, 因此媒体源不依赖上位机是否在运行:
//! 打开失败或帧头非法时输出黑帧, 绝不返回失败, 避免消费端(游戏/相机应用)整条管线报错。

use std::ptr::{null, null_mut};

use windows::Win32::Foundation::{CloseHandle, HANDLE};
use windows::Win32::System::Memory::{
    FILE_MAP_READ, MEMORY_MAPPED_VIEW_ADDRESS, MapViewOfFile, OpenFileMappingW, UnmapViewOfFile,
};
use windows::core::w;

/// 对外输出画面尺寸(固定单一媒体类型)。
pub const WIDTH: u32 = 640;
pub const HEIGHT: u32 = 480;
/// RGB32 一帧字节数(消费端缓冲大小)。
pub const RGB32_BYTES: usize = (WIDTH as usize) * (HEIGHT as usize) * 4;

const _MAGIC: u32 = 0x4D32_4356;
const _VERSION: u32 = 1;
const _FORMAT_RGB24: u32 = 1;
const _HEADER_BYTES: usize = 32;
const _RGB24_BYTES: usize = (WIDTH as usize) * (HEIGHT as usize) * 3;
const _TOTAL_BYTES: usize = _HEADER_BYTES + _RGB24_BYTES;

/// 只读帧头(小端): magic/version/width/height/format/序号/保留。
#[repr(C)]
struct _Header {
    _magic: u32,
    _version: u32,
    _width: u32,
    _height: u32,
    _format: u32,
    _seq: u32,
    _reserved: [u32; 2],
}

pub struct FrameReader {
    _map: HANDLE,
    _view: *const u8,
    /// 复用的暂存缓冲: 先整帧拷出共享内存再转换, 避免在生产者可能改写的内存上做多次读取。
    _stage: Vec<u8>,
}

impl FrameReader {
    pub fn new() -> Self {
        Self {
            _map: HANDLE(null_mut()),
            _view: null(),
            _stage: Vec::new(),
        }
    }

    /// 把当前帧转成 RGB32(BGRA 内存序, alpha=255)写入 `dst`; 任何异常都写黑帧。
    pub fn fill(&mut self, dst: &mut [u8]) {
        if dst.len() < RGB32_BYTES {
            return;
        }
        if !self._open() || !self._header_ok() {
            dst[..RGB32_BYTES].fill(0);
            return;
        }
        if self._stage.len() != _RGB24_BYTES {
            self._stage.resize(_RGB24_BYTES, 0);
        }
        unsafe {
            std::ptr::copy_nonoverlapping(
                self._view.add(_HEADER_BYTES),
                self._stage.as_mut_ptr(),
                _RGB24_BYTES,
            );
        }
        // 生产者是 R,G,B 顺序; MFVideoFormat_RGB32 内存序为 B,G,R,A。
        for (i, px) in self._stage.chunks_exact(3).enumerate() {
            let o = i * 4;
            dst[o] = px[2];
            dst[o + 1] = px[1];
            dst[o + 2] = px[0];
            dst[o + 3] = 0xFF;
        }
    }

    /// 惰性打开映射(生产者可能后启动), 已打开则直接复用。
    fn _open(&mut self) -> bool {
        if !self._view.is_null() {
            return true;
        }
        let map = match unsafe {
            OpenFileMappingW(FILE_MAP_READ.0, false, w!("Global\\mai2control_vcam_frame"))
        } {
            Ok(h) => h,
            // Global 因非管理员发布退回 Local 时，仅同会话宿主才可能读到；仍尝试以
            // 便于诊断，Frame Server(Local Service)读不到时上位机会明确提示管理员运行。
            Err(_) => match unsafe {
                OpenFileMappingW(FILE_MAP_READ.0, false, w!("Local\\mai2control_vcam_frame"))
            } {
                Ok(h) => h,
                Err(_) => return false,
            },
        };
        let view: MEMORY_MAPPED_VIEW_ADDRESS =
            unsafe { MapViewOfFile(map, FILE_MAP_READ, 0, 0, _TOTAL_BYTES) };
        if view.Value.is_null() {
            unsafe { _ = CloseHandle(map) };
            return false;
        }
        self._map = map;
        self._view = view.Value as *const u8;
        true
    }

    fn _header_ok(&self) -> bool {
        let h = unsafe { std::ptr::read_volatile(self._view as *const _Header) };
        h._magic == _MAGIC
            && h._version == _VERSION
            && h._width == WIDTH
            && h._height == HEIGHT
            && h._format == _FORMAT_RGB24
    }

    fn _close(&mut self) {
        if !self._view.is_null() {
            unsafe {
                _ = UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                    Value: self._view as *mut _,
                });
            }
            self._view = null();
        }
        if !self._map.0.is_null() {
            unsafe { _ = CloseHandle(self._map) };
            self._map = HANDLE(null_mut());
        }
    }
}

impl Drop for FrameReader {
    fn drop(&mut self) {
        self._close();
    }
}
