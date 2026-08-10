//! 主屏幕截图(Win32 GDI)。
//!
//! 只服务一个用途: HID 触控点位页要在"真实屏幕画面"上锚定 36 个通道的坐标, 所以需要一张
//! 与目标屏幕**原始分辨率**一致的位图作底图。
//!
//! ★为什么走 GDI BitBlt 而不是 DXGI Desktop Duplication★
//! 这里要的是"进页面时抓一张静态图", 不是持续视频流。BitBlt 一次调用即得完整帧、无需初始化
//! D3D 设备/交换链, 也不受"独占全屏应用占用复制接口"影响; 每次抓图约几十毫秒, 用户手动触发,
//! 完全不在热路径上。Desktop Duplication 的优势(高帧率、增量脏区)在这个场景没有任何价值。
//!
//! 已知取舍(如实记录, 不掩盖):
//!  - 抓的是**主显示器**(SM_CXSCREEN/SM_CYSCREEN 描述的那块), 多屏系统不抓副屏。触控屏在副屏时
//!    需要用户把它设为主显示器 —— 这一点由界面文案告知, 而不是悄悄抓错一块屏。
//!  - 进程未声明 DPI 感知时, GetSystemMetrics 返回的是**缩放后**的逻辑分辨率, 抓到的图也是那个
//!    尺寸。由于点位坐标全程按"图像尺寸归一"处理(见 HID_COORD_MAX 域), 缩放不会让锚点偏移:
//!    归一坐标与物理像素无关。
//!  - 分层/受保护内容(部分播放器、DRM 窗口)在 BitBlt 下可能为黑, 这是系统行为, 不做规避。

use anyhow::{Result, anyhow};

use windows::Win32::Graphics::Gdi::{
    BI_RGB, BITMAPINFO, BITMAPINFOHEADER, BitBlt, CreateCompatibleBitmap, CreateCompatibleDC,
    DIB_RGB_COLORS, DeleteDC, DeleteObject, GetDC, GetDIBits, HGDIOBJ, ReleaseDC, SRCCOPY,
    SelectObject,
};
use windows::Win32::UI::WindowsAndMessaging::{GetSystemMetrics, SM_CXSCREEN, SM_CYSCREEN};

/// 一张截图: 原始像素尺寸 + RGB8 紧密排列的像素(长度恒为 w*h*3)。
/// 尺寸与像素必须同生同灭, 故合成一个结构 —— 分开传两个值必然出现"尺寸对不上缓冲"的越界。
pub struct Screenshot {
    pub width: u32,
    pub height: u32,
    /// RGB8, 行优先, 无行填充。
    pub rgb: Vec<u8>,
}

/// GDI 资源的 RAII 收纳。★不靠"每条错误分支各自记得 Delete"★:
/// 抓图路径上有 6 处可失败点, 手写清理必然漏掉某一条(泄漏 DC/位图是会累积到整个进程的)。
struct GdiScope {
    screen_dc: windows::Win32::Graphics::Gdi::HDC,
    mem_dc: windows::Win32::Graphics::Gdi::HDC,
    bitmap: windows::Win32::Graphics::Gdi::HBITMAP,
    prev: HGDIOBJ,
}

impl Drop for GdiScope {
    fn drop(&mut self) {
        // SAFETY: 每个句柄都在本结构存活期内有效; 未创建成功的句柄为 0/null, 相应 API 对其安全。
        unsafe {
            if !self.mem_dc.is_invalid() {
                if !self.prev.is_invalid() {
                    let _ = SelectObject(self.mem_dc, self.prev);
                }
                let _ = DeleteDC(self.mem_dc);
            }
            if !self.bitmap.is_invalid() {
                let _ = DeleteObject(self.bitmap.into());
            }
            if !self.screen_dc.is_invalid() {
                // 屏幕 DC 由 GetDC(None) 取得, 必须 ReleaseDC(不是 DeleteDC)。
                ReleaseDC(None, self.screen_dc);
            }
        }
    }
}

/// 抓取主屏幕当前画面。失败一律返回带原因的 Err, 由调用方写日志并在界面上如实提示 ——
/// 绝不返回一张黑图冒充成功(那会让用户在纯黑底图上锚出一堆错坐标)。
pub fn capture_primary_screen() -> Result<Screenshot> {
    // SAFETY: 以下 GDI 调用的句柄生命周期由 GdiScope 统一管理; 位图尺寸与 GetDIBits 的
    // BITMAPINFO 完全一致, 输出缓冲按 w*h*4 预留, 不存在越界写。
    unsafe {
        let width = GetSystemMetrics(SM_CXSCREEN);
        let height = GetSystemMetrics(SM_CYSCREEN);
        if width <= 0 || height <= 0 {
            return Err(anyhow!(
                "读取主屏幕尺寸失败(GetSystemMetrics 返回 {width}x{height})"
            ));
        }

        let screen_dc = GetDC(None);
        if screen_dc.is_invalid() {
            return Err(anyhow!("GetDC(主屏幕) 失败"));
        }
        let mem_dc = CreateCompatibleDC(Some(screen_dc));
        let bitmap = CreateCompatibleBitmap(screen_dc, width, height);
        let mut scope = GdiScope {
            screen_dc,
            mem_dc,
            bitmap,
            prev: HGDIOBJ::default(),
        };
        if mem_dc.is_invalid() {
            return Err(anyhow!("CreateCompatibleDC 失败"));
        }
        if bitmap.is_invalid() {
            return Err(anyhow!("CreateCompatibleBitmap({width}x{height}) 失败"));
        }
        scope.prev = SelectObject(mem_dc, bitmap.into());

        BitBlt(mem_dc, 0, 0, width, height, Some(screen_dc), 0, 0, SRCCOPY)
            .map_err(|e| anyhow!("BitBlt 抓屏失败: {e}"))?;

        // 取 32 位顶向下 DIB: biHeight 取负 = 行序自顶向下, 省掉一次整图翻转。
        let mut info = BITMAPINFO::default();
        info.bmiHeader.biSize = std::mem::size_of::<BITMAPINFOHEADER>() as u32;
        info.bmiHeader.biWidth = width;
        info.bmiHeader.biHeight = -height;
        info.bmiHeader.biPlanes = 1;
        info.bmiHeader.biBitCount = 32;
        info.bmiHeader.biCompression = BI_RGB.0;

        let pixel_count = (width as usize) * (height as usize);
        let mut bgra = vec![0u8; pixel_count * 4];
        let copied = GetDIBits(
            mem_dc,
            bitmap,
            0,
            height as u32,
            Some(bgra.as_mut_ptr().cast()),
            &mut info,
            DIB_RGB_COLORS,
        );
        if copied == 0 {
            return Err(anyhow!("GetDIBits 未复制任何扫描行"));
        }
        if copied != height {
            return Err(anyhow!(
                "GetDIBits 只复制了 {copied}/{height} 行, 截图不完整"
            ));
        }

        // BGRA(GDI 的 32 位 DIB 恒为此序) → RGB8(Slint Image::from_rgb8 所需), 丢弃 alpha:
        // 屏幕内容不透明, GDI 也不保证该字节有意义。
        let mut rgb = Vec::with_capacity(pixel_count * 3);
        for px in bgra.chunks_exact(4) {
            rgb.push(px[2]);
            rgb.push(px[1]);
            rgb.push(px[0]);
        }

        Ok(Screenshot {
            width: width as u32,
            height: height as u32,
            rgb,
        })
    }
}
