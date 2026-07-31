//! 虚拟摄像头帧共享内存生产者。
//!
//! 媒体源在 Frame Server 进程中读取这块命名映射；上位机只在 UI 线程持有它，
//! 这样既不需要跨线程传递 Windows 句柄，也不会让媒体源依赖上位机进程内 ABI。

use std::mem::size_of;
use std::ptr::NonNull;
use std::sync::atomic::{Ordering, fence};

use anyhow::{Result, anyhow};
use windows::Win32::Foundation::{CloseHandle, ERROR_ACCESS_DENIED, HANDLE, HLOCAL};
use windows::Win32::Security::Authorization::{
    ConvertStringSecurityDescriptorToSecurityDescriptorW, SDDL_REVISION_1,
};
use windows::Win32::Security::{PSECURITY_DESCRIPTOR, SECURITY_ATTRIBUTES};
use windows::Win32::System::Memory::{
    CreateFileMappingW, FILE_MAP_ALL_ACCESS, MEMORY_MAPPED_VIEW_ADDRESS, MapViewOfFile,
    PAGE_READWRITE, UnmapViewOfFile,
};
use windows::core::{HRESULT, w};

use super::{FRAME_H, FRAME_W};

const _MAGIC: u32 = 0x4D32_4356;
const _VERSION: u32 = 1;
const _FORMAT_RGB24: u32 = 1;
const _HEADER_BYTES: usize = 32;
const _FRAME_BYTES: usize = FRAME_W * FRAME_H * 3;
const _TOTAL_BYTES: usize = _HEADER_BYTES + _FRAME_BYTES;

/// 映射实际创建的会话命名空间。Local 仅是权限受限时的明确降级，不能保证
/// Local Service 所在的 Frame Server 能读取，因此 UI 必须提示管理员运行。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ShareNamespace {
    Global,
    Local,
}

impl ShareNamespace {
    pub fn label(self) -> &'static str {
        match self {
            Self::Global => "Global",
            Self::Local => "Local",
        }
    }
}

/// RGB24 帧生产者。字段刻意只在 UI 线程使用，避免 HANDLE 被无约束地跨线程传递。
pub struct FramePublisher {
    _map: HANDLE,
    _view: NonNull<u8>,
    _namespace: ShareNamespace,
    _sequence: u32,
}

impl FramePublisher {
    /// 先创建 Global 映射；普通用户缺少 SeCreateGlobalPrivilege 时仅在
    /// ERROR_ACCESS_DENIED 下退回 Local，其他错误不能伪装成权限问题。
    pub fn create() -> Result<Self> {
        match Self::_create(ShareNamespace::Global) {
            Ok(publisher) => {
                log::info!("虚拟摄像头: 共享内存已创建 Global\\mai2control_vcam_frame");
                Ok(publisher)
            }
            Err(error)
                if error
                    .downcast_ref::<windows::core::Error>()
                    .is_some_and(|win| {
                        win.code() == HRESULT::from_win32(ERROR_ACCESS_DENIED.0)
                    }) =>
            {
                log::warn!(
                    "虚拟摄像头: 非管理员运行，帧共享退回会话内命名空间 Local；Frame Server(Local Service)可能读不到 → 需要以管理员运行上位机"
                );
                let publisher = Self::_create(ShareNamespace::Local)?;
                log::info!("虚拟摄像头: 共享内存已创建 Local\\mai2control_vcam_frame");
                Ok(publisher)
            }
            Err(error) => Err(error),
        }
    }

    pub fn namespace(&self) -> ShareNamespace {
        self._namespace
    }

    /// 先顺序写完整像素，再以 release 栅栏和 volatile 写发布帧序号；读取端把序号
    /// 视为该次像素拷贝已完成的提交标志，避免先看到新序号而后半帧仍是旧数据。
    pub fn publish(&mut self, rgb: &[u8]) -> Result<()> {
        if rgb.len() != _FRAME_BYTES {
            return Err(anyhow!(
                "RGB24 帧大小错误: {}，期望 {}",
                rgb.len(),
                _FRAME_BYTES
            ));
        }
        unsafe {
            std::ptr::copy_nonoverlapping(
                rgb.as_ptr(),
                self._view.as_ptr().add(_HEADER_BYTES),
                _FRAME_BYTES,
            );
            fence(Ordering::Release);
            self._sequence = self._sequence.wrapping_add(1);
            std::ptr::write_volatile(self._view.as_ptr().add(20).cast::<u32>(), self._sequence);
        }
        Ok(())
    }

    fn _create(namespace: ShareNamespace) -> Result<Self> {
        let (mut attributes, descriptor) = Self::_security_attributes()?;
        let name = match namespace {
            ShareNamespace::Global => w!("Global\\mai2control_vcam_frame"),
            ShareNamespace::Local => w!("Local\\mai2control_vcam_frame"),
        };
        let mapping = unsafe {
            CreateFileMappingW(
                HANDLE((-1isize) as *mut _),
                Some(&mut attributes),
                PAGE_READWRITE,
                0,
                _TOTAL_BYTES as u32,
                name,
            )
        };
        unsafe { windows::Win32::Foundation::LocalFree(Some(HLOCAL(descriptor.0 as *mut _))) };
        let mapping = mapping?;
        let view = unsafe { MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, _TOTAL_BYTES) };
        let Some(view) = NonNull::new(view.Value.cast::<u8>()) else {
            unsafe { _ = CloseHandle(mapping) };
            return Err(windows::core::Error::from_thread().into());
        };

        let mut publisher = Self {
            _map: mapping,
            _view: view,
            _namespace: namespace,
            _sequence: 0,
        };
        publisher._write_header();
        Ok(publisher)
    }

    /// SDDL 显式给 Everyone 读/执行；Frame Server 以 Local Service 运行，继承默认
    /// DACL 时通常无权打开用户创建的映射。管理员和 Local System 保留完全控制。
    fn _security_attributes() -> Result<(SECURITY_ATTRIBUTES, PSECURITY_DESCRIPTOR)> {
        let mut descriptor = PSECURITY_DESCRIPTOR::default();
        unsafe {
            ConvertStringSecurityDescriptorToSecurityDescriptorW(
                w!("D:(A;;GRGX;;;WD)(A;;GA;;;BA)(A;;GA;;;SY)"),
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

    fn _write_header(&mut self) {
        unsafe {
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(0), _MAGIC);
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(1), _VERSION);
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(2), FRAME_W as u32);
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(3), FRAME_H as u32);
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(4), _FORMAT_RGB24);
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(5), 0);
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(6), 0);
            std::ptr::write_volatile(self._view.as_ptr().cast::<u32>().add(7), 0);
            std::ptr::write_bytes(self._view.as_ptr().add(_HEADER_BYTES), 0, _FRAME_BYTES);
        }
    }
}

impl Drop for FramePublisher {
    fn drop(&mut self) {
        unsafe {
            _ = UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self._view.as_ptr().cast(),
            });
            _ = CloseHandle(self._map);
        }
    }
}
