//! mai2control 无头自测程序
//!
//! 行为流程:
//! 1. 初始化日志
//! 2. 枚举设备,取第一个或读命令行参数覆盖
//! 3. 连接,发 HELLO,等 DEVICE_INFO
//! 4. 发 CFG_GET_ALL,统计配置条目数
//! 5. 发 TELEM_START,收 TELEM_DATA 帧并统计
//! 6. 发 TELEM_STOP
//! 7. 发 PARAM_GET_ALL(ch0),统计参数数
//! 8. 打印 SELFTEST PASS/FAIL
//! 9. exit(0) 或 exit(1)
//!
//! 若 argv 含 `--reboot-bootloader` 则末尾发送进烧录指令(可选,默认不发)

use mai2control_ui::app_state::{AppController, ConnState};
use mai2control_ui::io;
use mai2control_ui::proto::{
    BRINGUP_FLAG_CHECKSUM, BRINGUP_FLAG_LINK, BRINGUP_FLAG_SNAPSHOT, CfgValue,
    EXPECTED_PSOC_S455_ID, LED_PREVIEW_ALL, LED_UNIT_COUNT, LedRegion, RP_BUILD_ID_DIAGNOSTIC_V1,
};
use std::thread;
use std::time::Duration;

const HELLO_TIMEOUT_MS: u64 = 1500;
const CFG_GET_ALL_TIMEOUT_MS: u64 = 1000;
const TELEM_TIMEOUT_MS: u64 = 800;
const PARAM_GET_ALL_TIMEOUT_MS: u64 = 1000;
const CP_MEASURE_TIMEOUT_MS: u64 = 10_000;
const CP_GET_TIMEOUT_MS: u64 = 1_000;
const CP_REQUEST_INTERVAL_MS: u64 = 500;
const CP_CHANNEL_COUNT: u8 = 36;
const CP_FAILURE_VALUE: u32 = 0x00FF_FFFF;
const LED_STATE_TIMEOUT_MS: u64 = 1_000;
const LED_APPLY_TIMEOUT_MS: u64 = 1_000;
const LED_PREVIEW_FALLBACK_MS: u64 = 3_300;

fn _print_vcam_probe_hr<T>(step: &str, result: &windows::core::Result<T>) -> bool {
    match result {
        Ok(_) => {
            println!("[VCAM] {}: 0x00000000", step);
            true
        }
        Err(error) => {
            println!("[VCAM] {}: 0x{:08X}", step, error.code().0 as u32);
            false
        }
    }
}

/// 无论 Start 是否成功都回收会话相机，避免探测留下系统可见但不可用的残留项。
fn _cleanup_vcam_probe_camera(
    access_name: &str,
    camera: &windows::Win32::Media::MediaFoundation::IMFVirtualCamera,
) {
    _print_vcam_probe_hr(
        &format!("IMFVirtualCamera::Stop({})", access_name),
        &unsafe { camera.Stop() },
    );
    _print_vcam_probe_hr(
        &format!("IMFVirtualCamera::Remove({})", access_name),
        &unsafe { camera.Remove() },
    );
    _print_vcam_probe_hr(
        &format!("IMFVirtualCamera::Shutdown({})", access_name),
        &unsafe { camera.Shutdown() },
    );
}

fn _run_vcam_probe() {
    use mai2control_ui::vcam::{
        FRAME_H, FRAME_W,
        share::{FramePublisher, ShareNamespace},
    };
    use windows::Win32::Media::KernelStreaming::IKsControl;
    use windows::Win32::Media::MediaFoundation::{
        IMFActivate, IMFAttributes, IMFGetService, IMFMediaEventGenerator, IMFMediaSource,
        IMFMediaSourceEx, IMFSampleAllocatorControl, IMFSourceReader,
        MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID, MF_E_SHUTDOWN, MF_MT_MAJOR_TYPE,
        MF_MT_SUBTYPE, MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING,
        MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_VERSION, MFCreateAttributes, MFCreateMediaType,
        MFCreateSourceReaderFromMediaSource, MFCreateVirtualCamera, MFEnumDeviceSources,
        MFMediaType_Video, MFSTARTUP_FULL, MFShutdown, MFStartup, MFVideoFormat_NV12,
        MFVideoFormat_RGB32, MFVirtualCameraAccess_AllUsers, MFVirtualCameraAccess_CurrentUser,
        MFVirtualCameraLifetime_Session, MFVirtualCameraType_SoftwareCameraSource,
    };
    use windows::Win32::System::Com::{
        CLSCTX_INPROC_SERVER, COINIT_APARTMENTTHREADED, COINIT_MULTITHREADED, CoCreateInstance,
        CoInitializeEx, CoTaskMemFree, CoUninitialize,
    };
    use windows::core::{GUID, IUnknown, Interface};

    fn _measure_vcam_frame(bytes: &[u8], subtype: GUID) -> Option<([f32; 3], f32)> {
        const SAMPLE_STEP: usize = 8;
        let x_start = FRAME_W / 2 - 80;
        let x_end = FRAME_W / 2 + 80;
        let y_start = FRAME_H / 2 - 60;
        let y_end = FRAME_H / 2 + 60;
        let (mut sums, mut non_black, mut samples) = ([0u64; 3], 0usize, 0usize);

        if subtype == MFVideoFormat_RGB32 {
            if bytes.len() < FRAME_W * FRAME_H * 4 {
                return None;
            }
            for y in (y_start..y_end).step_by(SAMPLE_STEP) {
                for x in (x_start..x_end).step_by(SAMPLE_STEP) {
                    let offset = (y * FRAME_W + x) * 4;
                    let pixel = [bytes[offset + 2], bytes[offset + 1], bytes[offset]];
                    for channel in 0..3 {
                        sums[channel] += pixel[channel] as u64;
                    }
                    non_black += usize::from(pixel.iter().any(|&value| value > 12));
                    samples += 1;
                }
            }
        } else if subtype == MFVideoFormat_NV12 {
            if bytes.len() < FRAME_W * FRAME_H {
                return None;
            }
            for y in (y_start..y_end).step_by(SAMPLE_STEP) {
                for x in (x_start..x_end).step_by(SAMPLE_STEP) {
                    let luma = bytes[y * FRAME_W + x];
                    sums[0] += luma as u64;
                    non_black += usize::from(luma > 24);
                    samples += 1;
                }
            }
        } else {
            return None;
        }

        if samples == 0 {
            return None;
        }
        Some((
            [
                sums[0] as f32 / samples as f32,
                sums[1] as f32 / samples as f32,
                sums[2] as f32 / samples as f32,
            ],
            non_black as f32 * 100.0 / samples as f32,
        ))
    }

    fn _vcam_frame_matches(
        mean: [f32; 3],
        non_black_percent: f32,
        subtype: GUID,
        color: [u8; 3],
    ) -> bool {
        const COLOR_TOLERANCE: f32 = 20.0;
        const MIN_NON_BLACK_PERCENT: f32 = 80.0;
        if non_black_percent < MIN_NON_BLACK_PERCENT {
            return false;
        }
        if subtype == MFVideoFormat_NV12 {
            let expected_luma =
                16.0 + 0.257 * color[0] as f32 + 0.504 * color[1] as f32 + 0.098 * color[2] as f32;
            return (mean[0] - expected_luma).abs() <= COLOR_TOLERANCE;
        }
        subtype == MFVideoFormat_RGB32
            && mean
                .iter()
                .zip(color)
                .all(|(actual, expected)| (*actual - expected as f32).abs() <= COLOR_TOLERANCE)
    }

    fn _publish_and_verify_vcam_frame(
        reader: &IMFSourceReader,
        publisher: &mut FramePublisher,
        subtype: GUID,
        round_name: &str,
        color: [u8; 3],
    ) -> Result<(), String> {
        let mut frame = vec![0u8; FRAME_W * FRAME_H * 3];
        for pixel in frame.chunks_exact_mut(3) {
            pixel.copy_from_slice(&color);
        }
        if let Err(error) = publisher.publish(&frame) {
            println!("[VCAM] FramePublisher::publish({}): {}", round_name, error);
            return Err(format!("{} 发布帧失败", round_name));
        }

        let start = std::time::Instant::now();
        let mut frames = 0u32;
        while start.elapsed() < Duration::from_secs(3) && frames < 60 {
            let mut actual_stream = 0u32;
            let mut stream_flags = 0u32;
            let mut sample = None;
            let read_result = unsafe {
                reader.ReadSample(
                    MF_SOURCE_READER_FIRST_VIDEO_STREAM.0 as u32,
                    0,
                    Some(&mut actual_stream),
                    Some(&mut stream_flags),
                    None,
                    Some(&mut sample),
                )
            };
            if let Err(error) = read_result {
                println!(
                    "[VCAM] IMFSourceReader::ReadSample({}): 0x{:08X}",
                    round_name,
                    error.code().0 as u32
                );
                return Err(format!("{} 读帧失败", round_name));
            }
            let Some(sample) = sample else {
                thread::sleep(Duration::from_millis(10));
                continue;
            };
            frames += 1;

            let buffer_result = unsafe { sample.ConvertToContiguousBuffer() };
            let buffer = match buffer_result {
                Ok(buffer) => buffer,
                Err(error) => {
                    println!(
                        "[VCAM] IMFSample::ConvertToContiguousBuffer({}): 0x{:08X}",
                        round_name,
                        error.code().0 as u32
                    );
                    return Err(format!("{} 连续缓冲区失败", round_name));
                }
            };
            let mut buffer_ptr = std::ptr::null_mut();
            let mut buffer_len = 0u32;
            let lock_result = unsafe { buffer.Lock(&mut buffer_ptr, None, Some(&mut buffer_len)) };
            if let Err(error) = lock_result {
                println!(
                    "[VCAM] IMFMediaBuffer::Lock({}): 0x{:08X}",
                    round_name,
                    error.code().0 as u32
                );
                return Err(format!("{} 锁定缓冲区失败", round_name));
            }
            let measurement = if buffer_ptr.is_null() {
                None
            } else {
                let bytes = unsafe { std::slice::from_raw_parts(buffer_ptr, buffer_len as usize) };
                _measure_vcam_frame(bytes, subtype)
            };
            let unlock_result = unsafe { buffer.Unlock() };
            if let Err(error) = unlock_result {
                println!(
                    "[VCAM] IMFMediaBuffer::Unlock({}): 0x{:08X}",
                    round_name,
                    error.code().0 as u32
                );
                return Err(format!("{} 解锁缓冲区失败", round_name));
            }
            let Some((mean, non_black_percent)) = measurement else {
                return Err(format!("{} 输出格式或帧大小不支持", round_name));
            };
            if _vcam_frame_matches(mean, non_black_percent, subtype, color) {
                if subtype == MFVideoFormat_NV12 {
                    println!(
                        "[VCAM] e2e {} hit frame={} mean=Y:{:.1} nonblack={:.1}%",
                        round_name, frames, mean[0], non_black_percent
                    );
                } else {
                    println!(
                        "[VCAM] e2e {} hit frame={} mean=R:{:.1} G:{:.1} B:{:.1} nonblack={:.1}%",
                        round_name, frames, mean[0], mean[1], mean[2], non_black_percent
                    );
                }
                return Ok(());
            }
        }
        Err(format!("{} 在 3 秒/60 帧内未命中目标颜色", round_name))
    }

    fn _run_current_user_vcam_e2e(vcam_name: &str) -> Result<(), String> {
        let mut publisher = match FramePublisher::create() {
            Ok(publisher) => publisher,
            Err(error) => {
                println!("[VCAM] FramePublisher::create: {}", error);
                return Err("无法创建帧共享内存".to_string());
            }
        };
        println!(
            "[VCAM] FramePublisher namespace: {}",
            publisher.namespace().label()
        );
        if publisher.namespace() == ShareNamespace::Local {
            println!(
                "[VCAM] WARNING: Local 映射无法被 session 0 Frame Server 读取，端到端判定不可用"
            );
            return Err("FramePublisher 退回 Local 命名空间".to_string());
        }

        let mut enum_attributes: Option<IMFAttributes> = None;
        let enum_attributes_result = unsafe { MFCreateAttributes(&mut enum_attributes, 1) };
        if !_print_vcam_probe_hr("MFCreateAttributes(device enum)", &enum_attributes_result) {
            return Err("无法创建设备枚举属性".to_string());
        }
        let Some(enum_attributes) = enum_attributes else {
            return Err("设备枚举属性为空".to_string());
        };
        let source_type_result = unsafe {
            enum_attributes.SetGUID(
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID,
            )
        };
        if !_print_vcam_probe_hr("IMFAttributes::SetGUID(VIDCAP)", &source_type_result) {
            return Err("无法设置视频捕获枚举条件".to_string());
        }

        let mut device_activates = std::ptr::null_mut();
        let mut device_count = 0u32;
        let enum_result = unsafe {
            MFEnumDeviceSources(&enum_attributes, &mut device_activates, &mut device_count)
        };
        if !_print_vcam_probe_hr("MFEnumDeviceSources(VIDCAP)", &enum_result) {
            return Err("枚举视频捕获设备失败".to_string());
        }
        // MFCreateVirtualCamera 将传入名称包装为 Windows Shell 可见的友好名；按枚举出的完整值精确匹配，
        // 仍以本次唯一 MAI2_VCAM_NAME 为前缀，避免误取同 CLSID 的旧会话相机。
        let expected_friendly_name = format!("{} (Windows 虚拟摄像头)", vcam_name);
        let mut matching_activate = None;
        if !device_activates.is_null() {
            for index in 0..device_count as usize {
                let candidate = unsafe { std::ptr::read(device_activates.add(index)) };
                let Some(candidate) = candidate else {
                    continue;
                };
                let name_len_result =
                    unsafe { candidate.GetStringLength(&MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME) };
                let Ok(name_len) = name_len_result else {
                    _print_vcam_probe_hr(
                        "IMFActivate::GetStringLength(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME)",
                        &name_len_result,
                    );
                    continue;
                };
                let mut name_utf16 = vec![0u16; name_len as usize + 1];
                let name_result = unsafe {
                    candidate.GetString(
                        &MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                        &mut name_utf16,
                        None,
                    )
                };
                if !_print_vcam_probe_hr(
                    "IMFActivate::GetString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME)",
                    &name_result,
                ) {
                    continue;
                }
                let name = String::from_utf16_lossy(&name_utf16[..name_len as usize]);
                println!(
                    "[VCAM] MFEnumDeviceSources candidate[{}]: '{}'",
                    index, name
                );
                if name == expected_friendly_name {
                    matching_activate = Some(candidate);
                    break;
                }
            }
            unsafe { CoTaskMemFree(Some(device_activates.cast())) };
        }
        println!(
            "[VCAM] MFEnumDeviceSources: total={} {}",
            device_count,
            if matching_activate.is_some() {
                "matched"
            } else {
                "not matched"
            }
        );
        let Some(matching_activate) = matching_activate else {
            return Err("未按友好名找到本次虚拟摄像头".to_string());
        };

        let source_result: windows::core::Result<IMFMediaSource> =
            unsafe { matching_activate.ActivateObject() };
        if !_print_vcam_probe_hr(
            "IMFActivate::ActivateObject(IMFMediaSource, enumerated camera)",
            &source_result,
        ) {
            return Err("无法激活枚举到的虚拟摄像头".to_string());
        }
        let media_source = match source_result {
            Ok(media_source) => media_source,
            Err(_) => return Err("枚举相机媒体源为空".to_string()),
        };

        let mut reader_attributes: Option<IMFAttributes> = None;
        let reader_attributes_result = unsafe { MFCreateAttributes(&mut reader_attributes, 1) };
        let mut e2e_result = if !_print_vcam_probe_hr(
            "MFCreateAttributes(source reader)",
            &reader_attributes_result,
        ) {
            Err("无法创建 SourceReader 属性".to_string())
        } else if let Some(reader_attributes) = reader_attributes {
            let enable_processing_result = unsafe {
                reader_attributes.SetUINT32(&MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, 1)
            };
            let video_processing = _print_vcam_probe_hr(
                "IMFAttributes::SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING)",
                &enable_processing_result,
            );
            let reader_result =
                unsafe { MFCreateSourceReaderFromMediaSource(&media_source, &reader_attributes) };
            if !_print_vcam_probe_hr("MFCreateSourceReaderFromMediaSource", &reader_result) {
                Err("无法创建 SourceReader".to_string())
            } else {
                match reader_result {
                    Ok(reader) => {
                        if video_processing {
                            let rgb_type_result = unsafe { MFCreateMediaType() };
                            if _print_vcam_probe_hr("MFCreateMediaType(RGB32)", &rgb_type_result) {
                                if let Ok(rgb_type) = rgb_type_result {
                                    let major_type_result = unsafe {
                                        rgb_type.SetGUID(&MF_MT_MAJOR_TYPE, &MFMediaType_Video)
                                    };
                                    let subtype_result = unsafe {
                                        rgb_type.SetGUID(&MF_MT_SUBTYPE, &MFVideoFormat_RGB32)
                                    };
                                    if _print_vcam_probe_hr(
                                        "IMFMediaType::SetGUID(MF_MT_MAJOR_TYPE=Video)",
                                        &major_type_result,
                                    ) && _print_vcam_probe_hr(
                                        "IMFMediaType::SetGUID(MF_MT_SUBTYPE=RGB32)",
                                        &subtype_result,
                                    ) {
                                        _print_vcam_probe_hr(
                                            "IMFSourceReader::SetCurrentMediaType(RGB32)",
                                            &unsafe {
                                                reader.SetCurrentMediaType(
                                                    MF_SOURCE_READER_FIRST_VIDEO_STREAM.0 as u32,
                                                    None,
                                                    &rgb_type,
                                                )
                                            },
                                        );
                                    }
                                }
                            }
                        }
                        let current_type_result = unsafe {
                            reader.GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM.0 as u32)
                        };
                        if !_print_vcam_probe_hr(
                            "IMFSourceReader::GetCurrentMediaType",
                            &current_type_result,
                        ) {
                            Err("无法取得 SourceReader 输出媒体类型".to_string())
                        } else if let Ok(current_type) = current_type_result {
                            let subtype_result = unsafe { current_type.GetGUID(&MF_MT_SUBTYPE) };
                            if !_print_vcam_probe_hr(
                                "IMFMediaType::GetGUID(MF_MT_SUBTYPE)",
                                &subtype_result,
                            ) {
                                Err("无法取得 SourceReader 输出子类型".to_string())
                            } else if let Ok(subtype) = subtype_result {
                                println!("[VCAM] SourceReader output subtype: {:?}", subtype);
                                if subtype != MFVideoFormat_RGB32 && subtype != MFVideoFormat_NV12 {
                                    Err(format!("不支持的 SourceReader 输出格式 {:?}", subtype))
                                } else if let Err(reason) = _publish_and_verify_vcam_frame(
                                    &reader,
                                    &mut publisher,
                                    subtype,
                                    "A",
                                    [200, 40, 60],
                                ) {
                                    Err(reason)
                                } else {
                                    _publish_and_verify_vcam_frame(
                                        &reader,
                                        &mut publisher,
                                        subtype,
                                        "B",
                                        [30, 180, 220],
                                    )
                                }
                            } else {
                                Err("SourceReader 输出子类型为空".to_string())
                            }
                        } else {
                            Err("SourceReader 输出媒体类型为空".to_string())
                        }
                    }
                    Err(_) => Err("SourceReader 为空".to_string()),
                }
            }
        } else {
            Err("SourceReader 属性为空".to_string())
        };

        let source_shutdown_result = unsafe { media_source.Shutdown() };
        // SourceReader 释放时会连带关闭它所拥有的媒体源, 所以这里的 MF_E_SHUTDOWN 是预期结果,
        // 只有其它错误码才说明媒体源真的没能正常收尾。
        let already_shutdown = source_shutdown_result
            .as_ref()
            .err()
            .is_some_and(|error| error.code() == MF_E_SHUTDOWN);
        if !_print_vcam_probe_hr(
            "IMFMediaSource::Shutdown(enumerated camera)",
            &source_shutdown_result,
        ) && !already_shutdown
            && e2e_result.is_ok()
        {
            e2e_result = Err("枚举相机媒体源 Shutdown 失败".to_string());
        }
        drop(media_source);
        drop(matching_activate);
        drop(publisher);
        e2e_result
    }

    const VCAM_CLSID: GUID = GUID::from_u128(0xB7C5F1A2_3D64_4E8B_9A11_2F6C8D0E4A73);
    // ★探针允许用唯一友好名★: MFCreateVirtualCamera 以入参为键复用已注册的虚拟相机, 沿用同名会
    // 复用上一次(可能是失败态)的注册记录。设 MAI2_VCAM_NAME 环境变量即可用全新键跑一次干净验证。
    let vcam_name_owned = std::env::var("MAI2_VCAM_NAME")
        .unwrap_or_else(|_| "mai2control Virtual Camera".to_string());
    let vcam_name: &str = vcam_name_owned.as_str();
    const VCAM_CLSID_TEXT: &str = "{B7C5F1A2-3D64-4E8B-9A11-2F6C8D0E4A73}";

    println!("[VCAM] probe begin");
    // windows 0.62 的 CoInitializeEx 返回裸 HRESULT, 直接打印即可(S_FALSE=已初始化过, 非错误)。
    let mut _com_init_count = 0u8;
    let apartment = unsafe { CoInitializeEx(None, COINIT_APARTMENTTHREADED) };
    println!(
        "[VCAM] CoInitializeEx(COINIT_APARTMENTTHREADED): 0x{:08X}",
        apartment.0 as u32
    );
    if apartment.is_ok() {
        _com_init_count += 1;
    }
    let multithreaded = unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) };
    println!(
        "[VCAM] CoInitializeEx(COINIT_MULTITHREADED): 0x{:08X}",
        multithreaded.0 as u32
    );
    if multithreaded.is_ok() {
        _com_init_count += 1;
    }

    let mf_startup = unsafe { MFStartup(MF_VERSION, MFSTARTUP_FULL) };
    let mf_started = _print_vcam_probe_hr("MFStartup(MF_VERSION, MFSTARTUP_FULL)", &mf_startup);

    let unknown_result: windows::core::Result<IUnknown> =
        unsafe { CoCreateInstance(&VCAM_CLSID, None, CLSCTX_INPROC_SERVER) };
    _print_vcam_probe_hr(
        "CoCreateInstance(CLSID_mai2vcam_source, CLSCTX_INPROC_SERVER, IID_IUnknown)",
        &unknown_result,
    );
    let unknown = unknown_result.ok();

    if let Some(unknown) = unknown.as_ref() {
        let activate_result = unknown.cast::<IMFActivate>();
        _print_vcam_probe_hr("QueryInterface(IMFActivate)", &activate_result);
        if let Some(activate) = activate_result.ok() {
            let media_source_result: windows::core::Result<IMFMediaSource> =
                unsafe { activate.ActivateObject() };
            _print_vcam_probe_hr(
                "IMFActivate::ActivateObject(IMFMediaSource)",
                &media_source_result,
            );

            if let Ok(media_source) = media_source_result.as_ref() {
                _print_vcam_probe_hr(
                    "QueryInterface(IMFMediaSource)",
                    &media_source.cast::<IMFMediaSource>(),
                );
                _print_vcam_probe_hr(
                    "QueryInterface(IMFMediaSourceEx)",
                    &media_source.cast::<IMFMediaSourceEx>(),
                );
                _print_vcam_probe_hr(
                    "QueryInterface(IMFMediaEventGenerator)",
                    &media_source.cast::<IMFMediaEventGenerator>(),
                );
                _print_vcam_probe_hr(
                    "QueryInterface(IMFGetService)",
                    &media_source.cast::<IMFGetService>(),
                );
                _print_vcam_probe_hr(
                    "QueryInterface(IKsControl)",
                    &media_source.cast::<IKsControl>(),
                );
                _print_vcam_probe_hr(
                    "QueryInterface(IMFAttributes)",
                    &media_source.cast::<IMFAttributes>(),
                );
                _print_vcam_probe_hr(
                    "QueryInterface(IMFSampleAllocatorControl)",
                    &media_source.cast::<IMFSampleAllocatorControl>(),
                );
                _print_vcam_probe_hr("IMFMediaSource::CreatePresentationDescriptor", &unsafe {
                    media_source.CreatePresentationDescriptor()
                });
                _print_vcam_probe_hr("IMFActivate::ShutdownObject", &unsafe {
                    activate.ShutdownObject()
                });
                _print_vcam_probe_hr("IMFActivate::DetachObject", &unsafe {
                    activate.DetachObject()
                });
            } else {
                for interface_name in [
                    "IMFMediaSource",
                    "IMFMediaSourceEx",
                    "IMFMediaEventGenerator",
                    "IMFGetService",
                    "IKsControl",
                    "IMFAttributes",
                    "IMFSampleAllocatorControl",
                ] {
                    println!(
                        "[VCAM] QueryInterface({}): skipped (ActivateObject failed)",
                        interface_name
                    );
                }
                println!(
                    "[VCAM] IMFMediaSource::CreatePresentationDescriptor: skipped (ActivateObject failed)"
                );
                println!("[VCAM] IMFActivate::ShutdownObject: skipped (ActivateObject failed)");
                println!("[VCAM] IMFActivate::DetachObject: skipped (ActivateObject failed)");
            }
        } else {
            println!(
                "[VCAM] IMFActivate::ActivateObject(IMFMediaSource): skipped (IMFActivate unavailable)"
            );
            for interface_name in [
                "IMFMediaSource",
                "IMFMediaSourceEx",
                "IMFMediaEventGenerator",
                "IMFGetService",
                "IKsControl",
                "IMFAttributes",
                "IMFSampleAllocatorControl",
            ] {
                println!(
                    "[VCAM] QueryInterface({}): skipped (IMFActivate unavailable)",
                    interface_name
                );
            }
            println!(
                "[VCAM] IMFMediaSource::CreatePresentationDescriptor: skipped (IMFActivate unavailable)"
            );
            println!("[VCAM] IMFActivate::ShutdownObject: skipped (IMFActivate unavailable)");
            println!("[VCAM] IMFActivate::DetachObject: skipped (IMFActivate unavailable)");
        }
    } else {
        println!("[VCAM] QueryInterface(IMFActivate): skipped (CoCreateInstance failed)");
        println!(
            "[VCAM] IMFActivate::ActivateObject(IMFMediaSource): skipped (CoCreateInstance failed)"
        );
        for interface_name in [
            "IMFMediaSource",
            "IMFMediaSourceEx",
            "IMFMediaEventGenerator",
            "IMFGetService",
            "IKsControl",
            "IMFAttributes",
            "IMFSampleAllocatorControl",
        ] {
            println!(
                "[VCAM] QueryInterface({}): skipped (CoCreateInstance failed)",
                interface_name
            );
        }
        println!(
            "[VCAM] IMFMediaSource::CreatePresentationDescriptor: skipped (CoCreateInstance failed)"
        );
        println!("[VCAM] IMFActivate::ShutdownObject: skipped (CoCreateInstance failed)");
        println!("[VCAM] IMFActivate::DetachObject: skipped (CoCreateInstance failed)");
    }
    // COM 引用必须在 MFShutdown/CoUninitialize 前释放，否则 DLL 的析构会访问已卸载的套间。
    drop(unknown);

    // ★MAI2_VCAM_ONLY_CURRENT=1 时只测 CurrentUser★: 同一 sourceId 同时存在两台会互相干扰 ——
    // AllUsers 那台 Start 失败后被 Remove(), 可能连带把该 CLSID 的虚拟相机注册一起删掉,
    // 导致随后 CurrentUser 那台 Start 时帧服务器只看到"0 个流"。
    let only_current = std::env::var("MAI2_VCAM_ONLY_CURRENT").is_ok();
    let all_users_result = if only_current {
        Err(windows::core::Error::from(
            windows::Win32::Foundation::E_ABORT,
        ))
    } else {
        unsafe {
            MFCreateVirtualCamera(
                MFVirtualCameraType_SoftwareCameraSource,
                MFVirtualCameraLifetime_Session,
                MFVirtualCameraAccess_AllUsers,
                &windows::core::HSTRING::from(vcam_name),
                &windows::core::HSTRING::from(VCAM_CLSID_TEXT),
                None,
            )
        }
    };
    if only_current {
        println!("[VCAM] MFCreateVirtualCamera(AllUsers): skipped (MAI2_VCAM_ONLY_CURRENT)");
    } else {
        _print_vcam_probe_hr("MFCreateVirtualCamera(AllUsers)", &all_users_result);
    }
    let current_user_result = unsafe {
        MFCreateVirtualCamera(
            MFVirtualCameraType_SoftwareCameraSource,
            MFVirtualCameraLifetime_Session,
            MFVirtualCameraAccess_CurrentUser,
            &windows::core::HSTRING::from(vcam_name),
            &windows::core::HSTRING::from(VCAM_CLSID_TEXT),
            None,
        )
    };
    _print_vcam_probe_hr("MFCreateVirtualCamera(CurrentUser)", &current_user_result);

    // ★两种 access 都要各自 Start 一次★: CurrentUser Start 成功后才可用枚举路径验证真实画面。
    let mut any_created = false;
    let mut e2e_result = None;
    for (access_name, camera) in [
        ("AllUsers", all_users_result.ok()),
        ("CurrentUser", current_user_result.ok()),
    ] {
        if let Some(camera) = camera {
            any_created = true;
            let start_result = unsafe { camera.Start(None) };
            let started = _print_vcam_probe_hr(
                &format!("IMFVirtualCamera::Start({})", access_name),
                &start_result,
            );
            if access_name == "CurrentUser" {
                e2e_result = Some(if started {
                    _run_current_user_vcam_e2e(vcam_name)
                } else {
                    Err("CurrentUser 虚拟摄像头 Start 失败".to_string())
                });
            }
            _cleanup_vcam_probe_camera(access_name, &camera);
        } else {
            println!(
                "[VCAM] IMFVirtualCamera::Start({}): skipped (creation failed)",
                access_name
            );
        }
    }
    if !any_created {
        println!("[VCAM] IMFVirtualCamera::Start: skipped (both creation calls failed)");
    }
    match e2e_result {
        Some(Ok(())) => println!("[VCAM] e2e: PASS"),
        Some(Err(reason)) => println!("[VCAM] e2e: FAIL {}", reason),
        None => println!("[VCAM] e2e: FAIL CurrentUser 虚拟摄像头未创建"),
    }

    if mf_started {
        _print_vcam_probe_hr("MFShutdown", &unsafe { MFShutdown() });
    } else {
        println!("[VCAM] MFShutdown: skipped (MFStartup failed)");
    }
    while _com_init_count > 0 {
        unsafe { CoUninitialize() };
        _com_init_count -= 1;
    }
    println!("[VCAM] probe end");
}

#[derive(Clone, Copy, Default)]
struct SoakDebugCounters {
    vendor_tx_bytes: u32,
    flash_write_count: u32,
    rearm_count: u32,
    out_stalled: u8,
    /// NvStore 落盘真值(固件 67B 诊断结构新增段)。dirty_mask==0 且 commit_fail 不增长
    /// 才是"设备真的写完了", 比按 flash_write_count 猜静止可靠。
    nv_dirty_mask: u8,
    nv_commit_ok: u32,
    nv_commit_fail: u32,
    nv_algo_src_len: u32,
}

fn read_soak_debug(ctrl: &AppController) -> Option<SoakDebugCounters> {
    let bytes = match ctrl.read_debug_counters() {
        Ok(bytes) if bytes.len() >= 44 => bytes,
        Ok(bytes) => {
            println!("[SOAK] DBG short response len={}", bytes.len());
            return None;
        }
        Err(error) => {
            println!("[SOAK] DBG read failed: {}", error);
            return None;
        }
    };
    let u32_at = |offset: usize| {
        u32::from_le_bytes([
            bytes[offset],
            bytes[offset + 1],
            bytes[offset + 2],
            bytes[offset + 3],
        ])
    };
    // 偏移必须与固件 UsbDebugCounters(pack(1)) 一致, 与 --debug-read 分支同源:
    // tx_calls@24, tx_bytes@28, flash_write@32, loop_at_last_flash@36, rearm@40,
    // out_busy@44, out_stalled@45。此前这里整体错了一档(把 tx_bytes 当 flash_write_count),
    // 于是"落盘次数"实际读的是 USB 发送字节数, 任何基于它的判据都失真。
    let has_nv = bytes.len() >= 67;
    Some(SoakDebugCounters {
        vendor_tx_bytes: u32_at(28),
        flash_write_count: u32_at(32),
        rearm_count: u32_at(40),
        out_stalled: bytes[45],
        nv_dirty_mask: if has_nv { bytes[54] } else { 0 },
        nv_commit_ok: if has_nv { u32_at(55) } else { 0 },
        nv_commit_fail: if has_nv { u32_at(59) } else { 0 },
        nv_algo_src_len: if has_nv { u32_at(63) } else { 0 },
    })
}

fn led_read_state(ctrl: &mut AppController, step: &str) -> bool {
    let version = ctrl.led_version();
    if let Err(error) = ctrl.led_request_state() {
        println!("[LED] {} FAIL LED_GET send: {}", step, error);
        return false;
    }
    let start = std::time::Instant::now();
    while start.elapsed() < Duration::from_millis(LED_STATE_TIMEOUT_MS) {
        ctrl.poll();
        if ctrl.led_version() > version {
            return true;
        }
        thread::sleep(Duration::from_millis(16));
    }
    println!("[LED] {} FAIL LED_GET 96B snapshot timeout", step);
    false
}

fn led_wait_receipt(ctrl: &mut AppController, step: &str, seq: u8, expect_accept: bool) -> bool {
    let start = std::time::Instant::now();
    while start.elapsed() < Duration::from_millis(LED_APPLY_TIMEOUT_MS) {
        ctrl.poll();
        if ctrl.led_apply_seq() != Some(seq) {
            let status = ctrl.led_apply_status();
            // 映射与预览共用回执槽位, 成功文案分别是"映射已生效"/"预览色已生效";
            // 失败文案是"设备拒绝(NAK): ..." 或"未连接...", 故以"已生效"收尾判定受理。
            let accepted = status.ends_with("已生效");
            println!(
                "[LED] {} device receipt: seq={} status='{}' error={:?}",
                step,
                seq,
                status,
                ctrl.last_error()
            );
            if accepted == expect_accept {
                return true;
            }
            println!(
                "[LED] {} FAIL expected device {} but got '{}'",
                step,
                if expect_accept { "ACK" } else { "NAK" },
                status
            );
            return false;
        }
        thread::sleep(Duration::from_millis(16));
    }
    println!("[LED] {} FAIL receipt timeout for seq={}", step, seq);
    false
}

fn led_regions_match(
    ctrl: &AppController,
    expected: &[LedRegion; LED_UNIT_COUNT],
    step: &str,
) -> bool {
    let mut matches = true;
    for (unit, expected_region) in expected.iter().enumerate() {
        let actual = ctrl.led_region(unit);
        if actual != *expected_region {
            println!(
                "[LED] {} FAIL unit={} expected=({},{},{}) actual=({},{},{})",
                step,
                unit,
                expected_region.ch,
                expected_region.start,
                expected_region.count,
                actual.ch,
                actual.start,
                actual.count
            );
            matches = false;
        }
    }
    matches
}

/// 单个预览用例: 下发 → 等设备回执(ACK) → LED_GET 回读校验生效色。
/// `unit = LED_PREVIEW_ALL` 时校验全部单元, 否则只校验该单元。
/// 失败一律打印设备侧诊断(回执文案 + byte1 诊断位), 让"没变色"能定位到具体环节。
fn led_preview_case(ctrl: &mut AppController, step: &str, unit: u8, rgb: [u8; 3]) -> bool {
    let seq = match ctrl.led_preview(unit, rgb) {
        Ok(seq) => seq,
        Err(error) => {
            println!("[LED] {} FAIL preview send: {}", step, error);
            return false;
        }
    };
    if !led_wait_receipt(ctrl, step, seq, true) {
        println!(
            "[LED] {} FAIL preview receipt: apply_status='{}' device_reason={:?}",
            step,
            ctrl.led_apply_status(),
            ctrl.last_error()
        );
        return false;
    }
    if !led_read_state(ctrl, step) {
        return false;
    }
    let targets: Vec<usize> = if unit == LED_PREVIEW_ALL {
        (0..LED_UNIT_COUNT).collect()
    } else {
        vec![unit as usize]
    };
    let mismatch: Vec<usize> = targets
        .iter()
        .copied()
        .filter(|target| ctrl.led_color(*target) != rgb)
        .collect();
    if mismatch.is_empty() {
        println!(
            "[LED] {} PASS preview RGB={:02X?} units={:?} preview_active={:?} refresh_ticks={:?}",
            step,
            rgb,
            targets,
            ctrl.led_preview_active(),
            ctrl.led_refresh_ticks()
        );
        return true;
    }
    println!(
        "[LED] {} FAIL preview RGB={:02X?} not effective on units={:?} (preview_active={:?} service_ready={:?} refresh_seen={:?} refresh_ticks={:?})",
        step,
        rgb,
        mismatch,
        ctrl.led_preview_active(),
        ctrl.led_service_ready(),
        ctrl.led_refresh_seen(),
        ctrl.led_refresh_ticks()
    );
    false
}

fn run_led_test(ctrl: &mut AppController) -> bool {
    println!("[LED] step 1/6: LED_GET and 96B snapshot contract...");
    if !led_read_state(ctrl, "step 1") {
        return false;
    }
    let unit_count = ctrl.led_unit_count();
    let ws_counts = [ctrl.led_ws_count(0), ctrl.led_ws_count(1)];
    if !ctrl.led_known() || unit_count != Some(LED_UNIT_COUNT as u8) {
        println!(
            "[LED] step 1 FAIL 96B contract: known={} unit_count={:?} expected={}",
            ctrl.led_known(),
            unit_count,
            LED_UNIT_COUNT
        );
        return false;
    }
    let chain_ready = [
        ctrl.led_chain_ready(0).unwrap_or(false),
        ctrl.led_chain_ready(1).unwrap_or(false),
    ];
    let init_fault = ctrl.led_init_fault().unwrap_or(0);
    println!(
        "[LED] step 1 PASS 96B snapshot: status={:?} chain_ready={:?} init_fault={} units={} ws_count={:?}",
        ctrl.led_status(),
        chain_ready,
        init_fault,
        LED_UNIT_COUNT,
        ws_counts
    );
    // 预览链路诊断(byte1): 预览没效果时据此区分"发丢了"/"服务没初始化"/"刷新没跑"。
    println!(
        "[LED] step 1 diag: resp_enabled={:?} preview_active={:?} service_ready={:?} refresh_seen={:?} refresh_ticks={:?}",
        ctrl.led_resp_enabled(),
        ctrl.led_preview_active(),
        ctrl.led_service_ready(),
        ctrl.led_refresh_seen(),
        ctrl.led_refresh_ticks()
    );
    if ctrl.led_service_ready() == Some(false) {
        println!(
            "[LED] step 1 WARN device LED map service not initialized (init_fault={})",
            init_fault
        );
    }
    if ctrl.led_refresh_seen() == Some(false) {
        println!(
            "[LED] step 1 WARN device LED refresh never ran: preview colors cannot reach the chains"
        );
    }
    if !chain_ready[0] && !chain_ready[1] {
        println!(
            "[LED] step 1 WARN both WS2812 chains are not ready (init_fault={})",
            init_fault
        );
    }

    let original: [LedRegion; LED_UNIT_COUNT] = std::array::from_fn(|unit| ctrl.led_region(unit));
    println!(
        "[LED] step 2/6: captured original {}-unit mapping",
        LED_UNIT_COUNT
    );
    let original_items: Vec<(u8, LedRegion)> = original
        .iter()
        .copied()
        .enumerate()
        .map(|(unit, region)| (unit as u8, region))
        .collect();
    let mut passed = true;

    println!("[LED] step 3/6: valid mapping round-trip...");
    match ctrl.led_send_regions_raw(&original_items) {
        Ok(seq) => passed &= led_wait_receipt(ctrl, "step 3 valid mapping", seq, true),
        Err(error) => {
            println!("[LED] step 3 FAIL LED_SET_REGION send: {}", error);
            passed = false;
        }
    }
    passed &= led_read_state(ctrl, "step 3 readback");
    if led_regions_match(ctrl, &original, "step 3 readback") {
        println!("[LED] step 3 PASS field-by-field readback");
    } else {
        passed = false;
    }

    println!("[LED] step 4/6: device-side atomic rejection of invalid mappings...");
    if let Some((chain, chain_len)) = ws_counts
        .iter()
        .copied()
        .enumerate()
        .find(|(_, count)| *count > 0)
    {
        let mut overlap = original_items.clone();
        overlap[0].1 = LedRegion {
            ch: chain as u8,
            start: 0,
            count: 1,
        };
        overlap[1].1 = LedRegion {
            ch: chain as u8,
            start: 0,
            count: 1,
        };
        match ctrl.led_send_regions_raw(&overlap) {
            Ok(seq) => passed &= led_wait_receipt(ctrl, "step 4 overlap", seq, false),
            Err(error) => {
                println!("[LED] step 4 overlap FAIL send: {}", error);
                passed = false;
            }
        }

        let mut out_of_bounds = original_items.clone();
        out_of_bounds[0].1 = LedRegion {
            ch: chain as u8,
            start: chain_len,
            count: 1,
        };
        match ctrl.led_send_regions_raw(&out_of_bounds) {
            Ok(seq) => passed &= led_wait_receipt(ctrl, "step 4 out-of-bounds", seq, false),
            Err(error) => {
                println!("[LED] step 4 out-of-bounds FAIL send: {}", error);
                passed = false;
            }
        }
    } else {
        println!("[LED] step 4 FAIL no non-empty WS chain in LED_GET snapshot");
        passed = false;
    }

    println!("[LED] step 5/6: preview receipt, effective color and automatic fallback...");
    let colors_before: [[u8; 3]; LED_UNIT_COUNT] = std::array::from_fn(|unit| ctrl.led_color(unit));
    // 先全体(0xFF)再单点(unit=0): 全体用例证明预览通路整体可用, 单点用例证明 unit 寻址
    // 没串到别的单元。单点用例只校验被点名的单元 —— 其余单元此刻仍持有上一次全体预览色
    // (预览超时未到), 拿协议色去比会得到假失败。
    let all_ok = led_preview_case(
        ctrl,
        "step 5 all-units",
        LED_PREVIEW_ALL,
        [0x17, 0xA5, 0x3C],
    );
    let unit0_ok = led_preview_case(ctrl, "step 5 unit0", 0, [0x3C, 0x17, 0xA5]);
    println!(
        "[LED] step 5 preview conclusion: all-units={} unit0={}",
        if all_ok { "PASS" } else { "FAIL" },
        if unit0_ok { "PASS" } else { "FAIL" }
    );
    passed &= all_ok && unit0_ok;
    let fallback_start = std::time::Instant::now();
    while fallback_start.elapsed() < Duration::from_millis(LED_PREVIEW_FALLBACK_MS) {
        ctrl.poll();
        thread::sleep(Duration::from_millis(16));
    }
    if !led_read_state(ctrl, "step 5 fallback readback") {
        passed = false;
    } else if (0..LED_UNIT_COUNT).all(|unit| ctrl.led_color(unit) == colors_before[unit]) {
        println!(
            "[LED] step 5 fallback PASS after ~3s (preview_active={:?})",
            ctrl.led_preview_active()
        );
    } else {
        println!(
            "[LED] step 5 FAIL preview did not return to protocol colors (preview_active={:?})",
            ctrl.led_preview_active()
        );
        passed = false;
    }

    println!("[LED] step 6/6: restore original mapping and confirm readback...");
    match ctrl.led_send_regions_raw(&original_items) {
        Ok(seq) => passed &= led_wait_receipt(ctrl, "step 6 restore", seq, true),
        Err(error) => {
            println!("[LED] step 6 FAIL restore send: {}", error);
            passed = false;
        }
    }
    passed &= led_read_state(ctrl, "step 6 restore readback");
    if led_regions_match(ctrl, &original, "step 6 restore readback") {
        println!("[LED] step 6 PASS original mapping restored");
    } else {
        passed = false;
    }
    passed
}

fn main() {
    env_logger::init();

    let args: Vec<String> = std::env::args().collect();
    // 虚拟摄像头探测不依赖 WinUSB 固件，必须在设备枚举之前独立退出。
    if args.iter().any(|a| a == "--vcam-probe") {
        _run_vcam_probe();
        return;
    }
    let reboot_bootloader = args.iter().any(|a| a == "--reboot-bootloader");
    let reboot_bootloader_only = args.iter().any(|a| a == "--reboot-bootloader-only");
    let smoke_only = args.iter().any(|a| a == "--smoke");
    let diagnose_only = args.iter().any(|a| a == "--diagnose");
    let csd_provision = args.iter().any(|a| a == "--csd-provision");
    let csd_verify = args.iter().any(|a| a == "--csd-verify");
    let algo_test = args.iter().any(|a| a == "--algo");
    let global_test = args.iter().any(|a| a == "--global");
    let kbd_test = args.iter().any(|a| a == "--kbd");
    let led_test = args.iter().any(|a| a == "--led");
    let soak = args.iter().any(|a| a == "--soak");
    let list_only = args.iter().any(|a| a == "--list-only");
    let debug_read = args.iter().any(|a| a == "--debug-read");
    let ctrl_bootsel = args.iter().any(|a| a == "--ctrl-bootsel");
    // 只请求配置并观测：每 200ms 打印 config_entries 数，持续 ~2.5s，看是否/何时到达及项数。
    let cfg_only = args.iter().any(|a| a == "--cfg-only");
    // 只读遥测: 握手后直接 TELEM_START, 打印全 36 通道 raw/bsln/diff/status, 排查"计数打满"。
    let telem_only = args.iter().any(|a| a == "--telem-only");
    // 纯空闲复现：连接+DEVICE_INFO 后立即 idle(不驱动任何功能)，模拟 GUI "连上就放着看"。
    let idle_only = args.iter().any(|a| a == "--idle-only");
    // 恢复设备配置：发 RESET_DEFAULTS 令固件 _runtime_map = _default_map(完整 schema) 并保存。
    let reset_config = args.iter().any(|a| a == "--reset-config");
    // 落盘全量漫灌回归: 走 GUI 同一套 AppController 通路写满可写项 → 保存 → 重启 → 全量回读比对。
    let nv_soak = args.iter().any(|a| a == "--nv-soak");
    // PSoC 救砖(PSOC_RESCUE 0x08): 扫描引擎卡死/恢复默认也救不回来时的最后手段。
    // 走设备既有救砖流程(SWD 全片重刷内嵌镜像 + 校验 + 复位 + 重新下发算法/CSD), 进度经推送流上报。
    let psoc_rescue = args.iter().any(|a| a == "--psoc-rescue");
    // --soak 空闲时长(秒)，默认 30；命令行可 `--soak-idle 60`
    let soak_idle_s: u64 = args
        .iter()
        .position(|a| a == "--soak-idle")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(30);
    // --soak-rate/--soak-fields/--soak-seconds 仅影响 soak 压测；缺省保持各入口既有负载。
    let soak_rate_hz: Option<u16> = args
        .iter()
        .position(|a| a == "--soak-rate")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok());
    let soak_fields: Option<u8> = args
        .iter()
        .position(|a| a == "--soak-fields")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| {
            let value = s
                .strip_prefix("0x")
                .or_else(|| s.strip_prefix("0X"))
                .unwrap_or(s);
            u8::from_str_radix(
                value,
                if s.starts_with("0x") || s.starts_with("0X") {
                    16
                } else {
                    10
                },
            )
            .ok()
        });
    let soak_seconds: u64 = args
        .iter()
        .position(|a| a == "--soak-seconds")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);

    println!("[SELFTEST] mai2control WinUSB 无头自测程序启动");

    // Step 1: 枚举设备
    let candidates = io::list_devices();
    if list_only {
        if candidates.is_empty() {
            println!("NONE");
        } else {
            println!("FOUND({})", candidates.len());
        }
        std::process::exit(0);
    }
    if candidates.is_empty() {
        println!("[SELFTEST] 未发现 VID 2E8A:000A / MI_00 WinUSB 设备");
        std::process::exit(2);
    }

    let selected_index = if args.len() > 1 && !args[1].starts_with("--") {
        candidates
            .iter()
            .position(|candidate| candidate.port_name == args[1])
            .unwrap_or(0)
    } else {
        0
    };
    let port_name = candidates[selected_index].port_name.clone();

    println!("[SELFTEST] 使用设备: {}", port_name);

    if ctrl_bootsel {
        match io::ctrl_bootsel(&port_name) {
            Ok(_) => {
                println!("[SELFTEST] CTRL BOOTSEL 已发送(设备将断开进烧录)");
                std::process::exit(0);
            }
            Err(e) => {
                println!("[SELFTEST] FAIL ctrl_bootsel: {}", e);
                std::process::exit(1);
            }
        }
    }

    if debug_read {
        match io::read_debug(&port_name) {
            Ok(b) if b.len() >= 44 => {
                let le16 = |o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
                let le32 = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
                println!("[DBG] magic=0x{:04X} len={}", le16(0), le16(2));
                println!("[DBG] loop_count={}", le32(4));
                println!("[DBG] tud_task_count={}", le32(8));
                println!("[DBG] vendor_rx_cb_count={}", le32(12));
                println!("[DBG] vendor_rx_bytes={}", le32(16));
                // 新增字段插在 rx_bytes 之后, 其后各偏移整体 +4。
                println!("[DBG] vendor_rx_dropped={}", le32(20));
                println!("[DBG] vendor_tx_calls={}", le32(24));
                println!("[DBG] vendor_tx_bytes={}", le32(28));
                println!("[DBG] flash_write_count={}", le32(32));
                println!("[DBG] loop_at_last_flash={}", le32(36));
                println!("[DBG] rearm_count={}", le32(40));
                println!(
                    "[DBG] out_busy={} out_stalled={} mounted={} debug_enabled={}",
                    b[44], b[45], b[46], b[47]
                );
                // 死前遗言: 上次复位前 core0 停在哪个阶段 + 上次是否为看门狗复位。
                if b.len() > 49 {
                    let stage = match b[48] {
                        0 => "none/unknown",
                        1 => "CFG_FLASH(ConfigManager::save_config_task)",
                        2 => "CSD_FLASH(CsdConfig::save)",
                        3 => "ALGO_FLASH(PsocAlgo::save)",
                        4 => "PSOC_SUBMIT(等 core1 结果)",
                        5 => "PSOC_ENQUEUE(等命令环空位)",
                        6 => "USB_UPDATE(解析主机命令)",
                        7 => "CORE1_CMD(core1 执行 SPI)",
                        _ => "?",
                    };
                    println!(
                        "[DBG] last_crash_stage={} ({}) last_boot_was_wd={}",
                        b[48], stage, b[49]
                    );
                }
                // flash 子系统真相: lfs 是否挂载 + 保存流程走到第几步(死在哪半段一目了然)。
                if b.len() > 52 {
                    let ss = b[52] as i8;
                    let step = match ss {
                        0 => "未进入过保存",
                        2 => "JSON已生成",
                        3 => "文件已打开",
                        4 => "写循环已跑完",
                        5 => "close(含sync)已返回",
                        -3 => "打开文件失败",
                        _ => "?",
                    };
                    println!(
                        "[DBG] lfs_ready={} save_entry_count={} last_save_stage={} ({})",
                        b[50], b[51], ss, step
                    );
                }
                if b.len() >= 67 {
                    println!(
                        "[DBG] nv_dirty_mask={} nv_commit_ok={} nv_commit_fail={} nv_algo_src_len={}",
                        b[54],
                        le32(55),
                        le32(59),
                        le32(63)
                    );
                }
                std::process::exit(0);
            }
            Ok(b) => {
                println!("[DBG] FAIL short response len={}", b.len());
                std::process::exit(1);
            }
            Err(e) => {
                println!("[DBG] FAIL {}", e);
                std::process::exit(1);
            }
        }
    }

    // Step 2: 创建控制器并连接同一枚举序号的 WinUSB config 接口。
    let mut ctrl = AppController::new();
    ctrl.refresh_devices();
    let index = selected_index;

    // 手动连接(不通过 on_connect_clicked,直接调 connect)
    match ctrl.connect(index) {
        Ok(_) => println!("[SELFTEST] 已连接"),
        Err(e) => {
            println!("[SELFTEST] FAIL 连接失败: {}", e);
            std::process::exit(1);
        }
    }

    // 小延迟让连接建立
    thread::sleep(Duration::from_millis(100));

    // Step 3: 轮询等待 HELLO 并收 DEVICE_INFO。重连场景下设备端 bulk OUT data toggle 与新句柄
    // 不同步会丢弃首个 HELLO, 故每 ~300ms 重发 HELLO(丢一包后 toggle 自动重同步), 与 UI 侧一致。
    println!("[SELFTEST] 轮询 DEVICE_INFO...");
    let start = std::time::Instant::now();
    let timeout = Duration::from_millis(HELLO_TIMEOUT_MS);
    let mut last_hello = std::time::Instant::now();
    loop {
        ctrl.poll();
        if ctrl.device_info_text() != "未获取到设备信息" {
            println!("[SELFTEST] 成功获取 DEVICE_INFO:");
            println!("{}", ctrl.device_info_text());
            if let Some(spi_debug) = ctrl
                .device_info()
                .and_then(|info| info.diagnostics.as_ref())
                .and_then(|diag| diag.spi_debug.as_ref())
            {
                println!(
                    // 槽位含义随 PSoC 0.4.44 换代: 槽0/4/5/6 改为"snsClk 被谁改回 8"的四件套。
                    // clk_boot=启动归一+Enable 后生效值; clk_now=当前值; clk_set=经 SET_PARAM 写入次数;
                    // clk_last=最后写入值(低16位=值, 高16位=通道)。stage 见 MLOOP_STAGE_*。
                    "PSoC SWD 调试: status={} block=0x{:08X} clk_boot={} scan={} ms={} stage={} clk_set={} clk_last=0x{:X} clk_now={} setparam_cmd={}",
                    spi_debug.status,
                    spi_debug.block_addr,
                    spi_debug.rx_frames,
                    spi_debug.scan_count,
                    spi_debug.ms_tick,
                    spi_debug.stage,
                    spi_debug.apply_cmd,
                    spi_debug.apply_last_ms,
                    spi_debug.apply_dirty,
                    spi_debug.setparam_cmd
                );
            }
            break;
        }
        if last_hello.elapsed() > Duration::from_millis(300) {
            let _ = ctrl.resend_hello();
            last_hello = std::time::Instant::now();
        }
        if start.elapsed() > timeout {
            println!(
                "[SELFTEST] FAIL 未在 {}ms 内收到 DEVICE_INFO",
                HELLO_TIMEOUT_MS
            );
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(50));
    }

    // 重启 RP2040 到应用(不进 BOOTSEL): 用于验证下次启动的 PSoC 烧录跳过(版本/内容一致则不擦写)。
    if args.iter().any(|a| a == "--reboot-app") {
        println!("[SELFTEST] 发送 REBOOT(RP2040 重启到应用)...");
        let _ = ctrl.reboot();
        thread::sleep(Duration::from_millis(300));
        println!("[SELFTEST] REBOOT sent");
        std::process::exit(0);
    }

    // 仅用于自持烧录：握手成功后立即请求 BOOTSEL，不执行耗时遥测测试。
    if reboot_bootloader_only {
        println!("[SELFTEST] WinUSB 握手成功，发送进 BOOTSEL 指令...");
        if let Err(e) = ctrl.reboot_bootloader() {
            println!("[SELFTEST] FAIL reboot_bootloader: {}", e);
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(350));
        println!("[SELFTEST] BOOTSEL REQUESTED");
        std::process::exit(0);
    }

    if led_test {
        let passed = run_led_test(&mut ctrl);
        println!("[LED] {}", if passed { "PASS" } else { "FAIL" });
        std::process::exit(if passed { 0 } else { 1 });
    }

    // Diagnose is intentionally read-only and accepts legacy short DEVICE_INFO payloads.
    if diagnose_only {
        println!("[SELFTEST] DIAGNOSE PASS");
        std::process::exit(0);
    }

    // 只读遥测: 直接开流并打印全通道 raw/bsln/diff/status + 全通道 Cp, 排查"计数打满"根因。
    if telem_only {
        use mai2control_ui::proto::{FIELD_BASELINE, FIELD_DIFF, FIELD_RAW, FIELD_STATUS};
        let _ = ctrl.start_telemetry(
            30,
            FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS,
            u64::MAX,
        );
        let start = std::time::Instant::now();
        while start.elapsed() < Duration::from_millis(1500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        // 同时请求各通道 Cp(fF), 便于判断是否传感器/短路导致饱和。
        for ch in 0..36u8 {
            let _ = ctrl.request_cp(ch);
        }
        let cp_start = std::time::Instant::now();
        while cp_start.elapsed() < Duration::from_millis(800) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        // 读回 CH0 参数与全局 CSD 配置, 定位导致全通道饱和的非法值。
        let _ = ctrl.request_params(0);
        let _ = ctrl.request_params(3);
        let _ = ctrl.global_get_all();
        let g_start = std::time::Instant::now();
        while g_start.elapsed() < Duration::from_millis(600) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let pname = |id: u8| match id {
            0x01 => "FINGER_TH",
            0x02 => "NOISE_TH",
            0x03 => "NEG_NOISE_TH",
            0x04 => "HYSTERESIS",
            0x05 => "ON_DEBOUNCE",
            0x06 => "LOW_BSLN_RST",
            0x07 => "RESOLUTION",
            0x08 => "SNS_CLK_DIV",
            0x09 => "IDAC_MOD",
            0x0A => "SNS_CLK_SOURCE",
            0x0B => "IDAC_GAIN",
            _ => "?",
        };
        for ch in [0u8, 3u8] {
            println!("[PARAM] --- CH{} ---", ch);
            for (id, v) in ctrl.params_of(ch) {
                println!("[PARAM] CH{} {:>14}(0x{:02X}) = {}", ch, pname(id), id, v);
            }
        }
        let gname = |id: u8| match id {
            1 => "INACTIVE_SNS",
            2 => "IDAC_GAIN_INIT",
            3 => "IDAC_MIN",
            4 => "RAW_TARGET",
            5 => "MFS_DIV_F1",
            6 => "MFS_DIV_F2",
            _ => "?",
        };
        for id in 1u8..=6 {
            println!(
                "[GLOBAL] {:>14}(0x{:02X}) = {:?}",
                gname(id),
                id,
                ctrl.global(id)
            );
        }
        // PSoC SPI 链路只读诊断计数(GPARAM_DBG_*, 复用 GET_GLOBAL 通道)。用于区分故障域:
        // rx_frames=0 → 帧根本没进 PSoC; bad_magic 随 rx_frames 同增 → 帧边界错位;
        // cs_resync 持续增长 → CS 边界与 DMA 字节流不同步。
        let dname = |id: u8| match id {
            0x80 => "DBG_RX_FRAMES",
            0x81 => "DBG_BAD_MAGIC",
            0x82 => "DBG_CS_RESYNC",
            0x83 => "DBG_TX_ARM",
            0x84 => "DBG_RX_LEFTOVER",
            _ => "?",
        };
        for id in 0x80u8..=0x84 {
            println!(
                "[SPIDBG] {:>15}(0x{:02X}) = {:?}",
                dname(id),
                id,
                ctrl.global(id)
            );
        }
        println!(
            "[TELEM] scan_period_us={} samples_per_sec={}",
            ctrl.telem_scan_period_us(),
            ctrl.telem_samples_per_sec()
        );
        for ch in 0..36u8 {
            let s = ctrl.telem_latest(ch);
            let (raw, bsln, diff, status) = match s {
                Some(ref x) => (x.raw, x.bsln, x.diff, x.status),
                None => (None, None, None, None),
            };
            println!(
                "[TELEM] CH{:02} raw={:?} bsln={:?} diff={:?} status={:?} cp_fF={:?}",
                ch,
                raw,
                bsln,
                diff,
                status,
                ctrl.cp(ch)
            );
        }
        let _ = ctrl.stop_telemetry();
        println!("[SELFTEST] TELEM-ONLY DONE");
        std::process::exit(0);
    }

    // Cp 测量实验: 触发 MEASURE_CP(BIST 逐电极电容) → 等 → 读回各通道 Cp, 定位"启动测量失败"。
    if args.iter().any(|a| a == "--cp-measure-test") {
        println!("[CP] 触发 MEASURE_CP...");
        match ctrl.measure_cp() {
            Ok(()) => println!("[CP] measure_cp 命令已发"),
            Err(e) => println!("[CP] measure_cp 失败: {}", e),
        }
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(4000) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(30));
        }
        for ch in 0..36u8 {
            let _ = ctrl.request_cp(ch);
        }
        let s2 = std::time::Instant::now();
        while s2.elapsed() < Duration::from_millis(1000) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        for ch in [0u8, 1, 2, 3, 10, 20, 35] {
            println!("[CP] CH{:02} cp_fF={:?}", ch, ctrl.cp(ch));
        }
        std::process::exit(0);
    }

    // CSD 恢复默认实验: RESET_DEFAULTS(清 CSD store + 重启 PSoC 用出厂默认) → 等 → 读 raw/频率,
    // 验证"清掉被保存的坏参数后 PSoC 回到 180Hz 正常" 的根因假设。
    if args.iter().any(|a| a == "--csd-reset-test") {
        use mai2control_ui::proto::{FIELD_BASELINE, FIELD_DIFF, FIELD_RAW, FIELD_STATUS};
        let _ = ctrl.start_telemetry(
            30,
            FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS,
            u64::MAX,
        );
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(1000) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let dump = |ctrl: &AppController, tag: &str| {
            let f = |ch: u8| {
                ctrl.telem_latest(ch)
                    .map(|x| (x.raw, x.diff))
                    .unwrap_or((None, None))
            };
            println!(
                "[CSDRST-{}] scan_period_us={} CH0={:?} CH3={:?} CH20={:?}",
                tag,
                ctrl.telem_scan_period_us(),
                f(0),
                f(3),
                f(20)
            );
        };
        dump(&ctrl, "BEFORE");
        println!("[CSDRST] 发送 RESET_DEFAULTS(清 CSD store + 重启 PSoC)...");
        let _ = ctrl.reset_defaults();
        let w = std::time::Instant::now();
        while w.elapsed() < Duration::from_millis(9000) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        dump(&ctrl, "AFTER-RESET");
        // 再显式校准 + 基线复位, 看 raw 是否从满量程回落(判断 railing 是否只是缺校准)。
        println!("[CSDRST] 发送 CALIBRATE(all) + BASELINE_RESET(all)...");
        let _ = ctrl.calibrate(u64::MAX);
        thread::sleep(Duration::from_millis(1500));
        let _ = ctrl.baseline_reset(u64::MAX);
        let w2 = std::time::Instant::now();
        while w2.elapsed() < Duration::from_millis(2500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        dump(&ctrl, "AFTER-CALIB");
        let _ = ctrl.stop_telemetry();
        std::process::exit(0);
    }

    // 参数防护验证: 读回 CH0 分辨率原值 → 尝试写非法值(99) → 回读应仍为原值(被固件拒绝)。
    if args.iter().any(|a| a == "--guard-test") {
        let _ = ctrl.request_params(0);
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let before = ctrl.param(0, 0x07);
        println!("[GUARD] CH0 RESOLUTION before = {:?}", before);
        let _ = ctrl.set_param(0, 0x07, 99); // 非法(合法 6..16), 仅暂存草稿
        let _ = ctrl.save_config(); // 提交草稿 → 实际发 PARAM_SET(99) 到设备
        thread::sleep(Duration::from_millis(150));
        // 从设备回读真实值(PARAM_GET_ALL 响应覆盖乐观缓存): 防护生效则仍为原值。
        let _ = ctrl.request_params(0);
        let s2 = std::time::Instant::now();
        while s2.elapsed() < Duration::from_millis(700) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let after = ctrl.param(0, 0x07);
        println!("[GUARD] CH0 RESOLUTION after commit 99 = {:?}", after);
        println!(
            "[GUARD] {}",
            if after == before && after != Some(99) {
                "PASS 非法值被设备拒绝"
            } else {
                "FAIL 非法值被接受"
            }
        );
        std::process::exit(0);
    }

    // 分辨率全通道下发诊断: 分别测【路径A UI草稿+save_config】与【路径B 直接debug_param_now】,
    // 各自对全 36 通道设同一分辨率并逐通道回读, 统计真正生效的通道数, 定位"只改1-2通道"丢在哪一层。
    if args.iter().any(|a| a == "--res-all-test") {
        let read_all = |ctrl: &mut AppController| -> Vec<Option<u32>> {
            for ch in 0..36u8 {
                let _ = ctrl.request_params(ch);
            }
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(1200) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(12));
            }
            (0..36u8).map(|ch| ctrl.param(ch, 0x07)).collect()
        };
        println!("[RESALL] 切半自动手动模式(SET_MODE=1)...");
        let _ = ctrl.debug_mode_now(1);
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let base = read_all(&mut ctrl);
        println!("[RESALL] 基线分辨率: {:?}", base);

        // 路径A: 完全复现 UI 全局改分辨率(set_param 0x07 内部重定向 set_param_all → 写36草稿) + save_config(背靠背发36条PARAM_SET+CALIBRATE)。
        let ta = 14u32;
        let _ = ctrl.set_param(0, 0x07, ta);
        let _ = ctrl.save_config();
        thread::sleep(Duration::from_millis(500));
        let a = read_all(&mut ctrl);
        let a_ok = a.iter().filter(|v| **v == Some(ta)).count();
        println!("[RESALL] 路径A(UI草稿+save) 目标={} 生效={}/36", ta, a_ok);
        println!("[RESALL]   回读A: {:?}", a);

        // 路径B: 直接逐通道 debug_param_now + calibrate(与 clk-probe 同法)。
        let tb = 12u32;
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, 0x07, tb);
        }
        let _ = ctrl.calibrate(u64::MAX);
        thread::sleep(Duration::from_millis(500));
        let b = read_all(&mut ctrl);
        let b_ok = b.iter().filter(|v| **v == Some(tb)).count();
        println!(
            "[RESALL] 路径B(直接debug_param_now) 目标={} 生效={}/36",
            tb, b_ok
        );
        println!("[RESALL]   回读B: {:?}", b);

        println!(
            "[RESALL] 判读: A<<36且B==36 → 丢在UI save路径(草稿/背靠背发送); A和B都<36 → 丢在RP2040/PSoC中继; 都==36 → 分辨率下发正常(问题在别处)"
        );
        let _ = ctrl.debug_mode_now(0);
        std::process::exit(0);
    }

    // SEMI 模式校准效力诊断: 扫 IDAC增益 × snsClkDiv, 每档 calibrate 后读全36通道 raw,
    // 判断 100pF 面板能否在合法参数内把 raw 拉离满量程(4095)。
    if args.iter().any(|a| a == "--semi-calib-probe") {
        use mai2control_ui::proto::{FIELD_BASELINE, FIELD_DIFF, FIELD_RAW};
        let stat = |ctrl: &mut AppController| -> (u32, u32, u32, usize) {
            let mut mn = u32::MAX;
            let mut mx = 0u32;
            let mut sum = 0u64;
            let mut railed = 0usize;
            for ch in 0..36u8 {
                if let Some(s) = ctrl.telem_latest(ch) {
                    let r = s.raw.unwrap_or(0) as u32;
                    if r < mn {
                        mn = r;
                    }
                    if r > mx {
                        mx = r;
                    }
                    sum += r as u64;
                    if r >= 4090 {
                        railed += 1;
                    }
                }
            }
            (mn, mx, (sum / 36) as u32, railed)
        };
        let settle = |ctrl: &mut AppController, ms: u64| {
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(15));
            }
        };
        println!("[CALIB] 切 SEMI(半自动手动)...");
        let _ = ctrl.debug_mode_now(1);
        settle(&mut ctrl, 400);
        let _ = ctrl.start_telemetry(30, FIELD_RAW | FIELD_BASELINE | FIELD_DIFF, u64::MAX);
        settle(&mut ctrl, 600);
        let (mn, mx, av, rl) = stat(&mut ctrl);
        println!(
            "[CALIB] 初始 raw: min={} max={} avg={} railed(>=4090)={}/36",
            mn, mx, av, rl
        );

        for &div in &[8u32, 20u32, 40u32, 80u32] {
            for &gain in &[4u32, 5u32, 6u32] {
                for ch in 0..36u8 {
                    let _ = ctrl.debug_param_now(ch, 0x08, div);
                } // SNS_CLK_DIV
                let _ = ctrl.debug_global_now(0x02, gain); // IDAC_GAIN_INIT
                let _ = ctrl.calibrate(u64::MAX);
                settle(&mut ctrl, 700);
                let (mn, mx, av, rl) = stat(&mut ctrl);
                println!(
                    "[CALIB] div={:>3} gain={} → raw min={} max={} avg={} railed={}/36",
                    div, gain, mn, mx, av, rl
                );
            }
        }
        // 基线复位效力: 复位后 diff 应≈0, 随后不动应保持稳定小噪声。
        let _ = ctrl.baseline_reset(u64::MAX);
        settle(&mut ctrl, 500);
        let mut diff_nonzero = 0usize;
        for ch in 0..36u8 {
            if let Some(s) = ctrl.telem_latest(ch) {
                if (s.diff.unwrap_or(0) as i32).abs() > 3 {
                    diff_nonzero += 1;
                }
            }
        }
        println!(
            "[CALIB] baseline_reset 后 |diff|>3 的通道数={}/36 (应≈0)",
            diff_nonzero
        );

        let _ = ctrl.stop_telemetry();
        // 复位到安全默认。
        let _ = ctrl.debug_global_now(0x02, 4);
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, 0x08, 8);
        }
        let _ = ctrl.debug_mode_now(0);
        println!(
            "[CALIB] 判读: 若某 div/gain 组合 railed 显著下降=校准可用(需设该默认); 若全组合都 railed=36=CSD扫描/时钟根本问题"
        );
        std::process::exit(0);
    }

    // 频率自适应(AUTO_TUNE)端到端验证: 触发自适应, 等结果, 校验成功后全通道离轨且有抖动。
    if args.iter().any(|a| a == "--auto-tune-test") {
        use mai2control_ui::proto::FIELD_RAW;
        let settle = |ctrl: &mut AppController, ms: u64| {
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(10));
            }
        };
        let target: u32 = args
            .iter()
            .position(|a| a == "--target")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse().ok())
            .unwrap_or(50);
        println!("[AUTOTUNE] SEMI + 目标{}% + 触发频率自适应下探...", target);
        let _ = ctrl.debug_mode_now(1);
        settle(&mut ctrl, 300);
        let _ = ctrl.debug_global_now(0x04, target);
        settle(&mut ctrl, 500);
        // --ch N: 只对该通道下探(逐通道分频); 缺省 0xFF = 全通道统一分频(旧行为)。
        let tune_ch: u8 = args
            .iter()
            .position(|a| a == "--ch")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse::<u8>().ok())
            .filter(|c| *c < 36)
            .unwrap_or(0xFF);
        // --pref N(1..7): 校准频率偏好档位。只写草稿即生效(auto_tune 组帧时草稿优先读取),
        // 缺省不设 → 沿用设备当前 calib.pref。档位越高=在临界频率上多让 2×(N-1) 个分频(越灵敏)。
        if let Some(pref) = args
            .iter()
            .position(|a| a == "--pref")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse::<u8>().ok())
            .filter(|p| (1..=7).contains(p))
        {
            let _ = ctrl.set_config_number("calib.pref", pref as f64);
            println!(
                "[AUTOTUNE] 校准频率偏好档位 = {} (临界分频 + {})",
                pref,
                2 * (pref - 1)
            );
        }
        let _ = ctrl.auto_tune(tune_ch);
        let t0 = std::time::Instant::now();
        // 轮询窗须大于固件预算(RP2040 _wait_op_done 45s / _submit 50s): 全通道已改为【逐通道各自校准】
        // (36 × 单通道三步算法 ≈ 11-23s, 最坏更长) → 窗口取 60s。
        // 固件以 5Hz 推送 AUTO_TUNE_PROGRESS(0x2E) 阶段进度(含当前通道号), 顺带打印以确认"过程可见"。
        let mut last_progress_ver = ctrl.auto_tune_progress_version();
        while ctrl.auto_tune_result() == 0 && t0.elapsed() < Duration::from_millis(60000) {
            ctrl.poll();
            if ctrl.auto_tune_progress_version() != last_progress_ver {
                last_progress_ver = ctrl.auto_tune_progress_version();
                let p = ctrl.auto_tune_progress();
                println!(
                    "[AUTOTUNE] 进度 state={} phase={}({}) step={} ch={} 试探div={} @{}ms",
                    p.state,
                    p.phase,
                    p.phase_text(),
                    p.step,
                    p.ch,
                    p.cur_div,
                    t0.elapsed().as_millis()
                );
            }
            thread::sleep(Duration::from_millis(50));
        }
        let res = ctrl.auto_tune_result();
        let div = ctrl.auto_tune_div();
        // 全通道(0xFF)模式下 div 字段语义 = 成功通道数; 单通道模式下 = 该通道最终分频。
        if tune_ch == 0xFF {
            let (lo, hi) = ctrl.sns_clk_div_range();
            println!(
                "[AUTOTUNE] 结果 result={} (1=至少一个通道成功 2=全失败) 成功通道数={}/36 分频范围=÷{}..÷{} 耗时~{}ms",
                res,
                div.min(36),
                lo,
                hi,
                t0.elapsed().as_millis()
            );
        } else {
            println!(
                "[AUTOTUNE] 结果 result={} (1=成功 2=失败) 找到分频div={} 耗时~{}ms",
                res,
                div,
                t0.elapsed().as_millis()
            );
        }
        // 校验: 成功则全通道应离轨且有抖动。
        let _ = ctrl.start_telemetry(30, FIELD_RAW, u64::MAX);
        settle(&mut ctrl, 700);
        let mut railed = 0usize;
        let mut seen: Vec<std::collections::BTreeSet<u16>> =
            vec![std::collections::BTreeSet::new(); 36];
        for _ in 0..12 {
            for ch in 0..36u8 {
                if let Some(s) = ctrl.telem_latest(ch) {
                    let r = s.raw.unwrap_or(0);
                    seen[ch as usize].insert(r);
                }
            }
            settle(&mut ctrl, 60);
        }
        for ch in 0..36 {
            if let Some(&mx) = seen[ch].iter().max() {
                if mx >= 4090 {
                    railed += 1;
                }
            }
        }
        let frozen: Vec<usize> = (0..36).filter(|&c| seen[c].len() <= 1).collect();
        println!(
            "[AUTOTUNE] 自适应后: railed={}/36 frozen={:?}",
            railed, frozen
        );
        let _ = ctrl.stop_telemetry();
        let _ = ctrl.debug_mode_now(0);
        let pass = res == 1 && railed == 0 && frozen.is_empty();
        println!(
            "[AUTOTUNE] {}",
            if pass {
                "PASS 自适应成功且全通道离轨/有抖动"
            } else {
                "CHECK 见上(result/railed/frozen)"
            }
        );
        std::process::exit(0);
    }

    // 校准跟踪 + 抖动 诊断: 随机切换校准目标%并校准, 验证全通道 raw 准确跟到 target%*maxRaw;
    // 并采样多帧检测抖动(某通道多帧取值恒定=frozen=故障)。
    if args.iter().any(|a| a == "--calib-track") {
        use mai2control_ui::proto::FIELD_RAW;
        let settle = |ctrl: &mut AppController, ms: u64| {
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(10));
            }
        };
        let div: u32 = args
            .iter()
            .position(|a| a == "--div")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse().ok())
            .unwrap_or(24);
        println!(
            "[TRACK] SEMI + 统一 res=12/snsClk={} + gain_init=4(auto-gain 开)...",
            div
        );
        let _ = ctrl.debug_mode_now(1);
        settle(&mut ctrl, 300);
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, 0x07, 12);
            let _ = ctrl.debug_param_now(ch, 0x08, div);
        }
        let _ = ctrl.debug_global_now(0x02, 4);
        let _ = ctrl.start_telemetry(30, FIELD_RAW, u64::MAX);
        settle(&mut ctrl, 500);
        let max_raw = 4095.0f32; // res=12
        for &target in &[25u32, 50, 40, 70, 30, 60, 85] {
            let _ = ctrl.debug_global_now(0x04, target); // 设目标%(RP2040 自动 commit → Init+Enable 自动校准)
            let _ = ctrl.calibrate(u64::MAX); // 再显式校准一次确保收敛
            settle(&mut ctrl, 900);
            let expected = target as f32 / 100.0 * max_raw;
            let mut sum = 0f32;
            let mut within = 0usize;
            let mut railed = 0usize;
            let mut n = 0usize;
            for ch in 0..36u8 {
                if let Some(s) = ctrl.telem_latest(ch) {
                    let r = s.raw.unwrap_or(0) as f32;
                    sum += r;
                    n += 1;
                    if r >= 4090.0 {
                        railed += 1;
                    }
                    if (r - expected).abs() <= expected * 0.25 + 60.0 {
                        within += 1;
                    }
                }
            }
            let avg = if n > 0 { sum / n as f32 } else { 0.0 };
            println!(
                "[TRACK] target={:>2}% 期望raw≈{:>4.0} 实测avg={:>6.0} 命中(±25%)={:>2}/36 railed={:>2}/36",
                target, expected, avg, within, railed
            );
        }
        // 抖动检测: 固定校准后采样 15 帧, 统计每通道不同取值数; 恒定(仅1种值)=frozen。
        let _ = ctrl.debug_global_now(0x04, 50);
        let _ = ctrl.calibrate(u64::MAX);
        settle(&mut ctrl, 600);
        let mut seen: Vec<std::collections::BTreeSet<u16>> =
            vec![std::collections::BTreeSet::new(); 36];
        for _ in 0..15 {
            for ch in 0..36u8 {
                if let Some(s) = ctrl.telem_latest(ch) {
                    seen[ch as usize].insert(s.raw.unwrap_or(0));
                }
            }
            settle(&mut ctrl, 60);
        }
        let frozen: Vec<usize> = (0..36).filter(|&c| seen[c].len() <= 1).collect();
        println!(
            "[TRACK] 抖动检测(15帧): frozen(恒定不变)通道 = {:?}",
            frozen
        );
        let _ = ctrl.stop_telemetry();
        let _ = ctrl.debug_mode_now(0);
        println!(
            "[TRACK] {}",
            if frozen.is_empty() {
                "PASS 无 frozen 通道(均有抖动)"
            } else {
                "FAIL 存在 frozen 通道(无抖动=未扫描/卡死)"
            }
        );
        std::process::exit(0);
    }

    // 综合压力测试: 随机排列组合 半自动/全自动 + IDAC全形态(增益/min/目标/sense/autocal) + 时钟分频
    // + 分辨率 + inactive + 校准 + 基线复位 + 重启, 每步后验证设备存活+链路恢复。任一步链路无法恢复=FAIL。
    // 用法: selftest.exe --soak-csd [次数] [--seed N]
    if args.iter().any(|a| a == "--soak-csd") {
        use mai2control_ui::proto::FIELD_RAW;
        let iters: u32 = args
            .iter()
            .position(|a| a == "--soak-csd")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse().ok())
            .unwrap_or(40);
        let mut seed: u32 = args
            .iter()
            .position(|a| a == "--seed")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse().ok())
            .unwrap_or(12345);
        let mut rng = move || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed >> 8) & 0x7FFF
        };
        let settle = |ctrl: &mut AppController, ms: u64| {
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(10));
            }
        };
        // 验证设备存活: resend_hello + 轮询 link_valid 恢复(reboot/重初始化给更久预算)。
        let check_alive = |ctrl: &mut AppController, budget_ms: u64| -> Option<u128> {
            let t0 = std::time::Instant::now();
            while t0.elapsed() < Duration::from_millis(budget_ms) {
                let _ = ctrl.resend_hello();
                let s = std::time::Instant::now();
                while s.elapsed() < Duration::from_millis(200) {
                    ctrl.poll();
                    thread::sleep(Duration::from_millis(10));
                }
                if ctrl
                    .device_info()
                    .map(|d| d.psoc_link_valid)
                    .unwrap_or(false)
                {
                    return Some(t0.elapsed().as_millis());
                }
            }
            None
        };
        println!("[SOAK] 开始综合压测: {} 次随机操作 (seed 起始)", iters);
        let _ = ctrl.start_telemetry(
            soak_rate_hz.unwrap_or(30),
            soak_fields.unwrap_or(FIELD_RAW),
            u64::MAX,
        );
        settle(&mut ctrl, 300);
        let mut fails = 0u32;
        for i in 0..iters {
            let op = rng() % 14;
            let desc: String;
            let mut budget = 2000u64;
            match op {
                0 => {
                    let _ = ctrl.debug_mode_now(1);
                    desc = "mode=SEMI".into();
                    budget = 1500;
                }
                1 => {
                    let _ = ctrl.debug_mode_now(0);
                    desc = "mode=AUTO".into();
                    budget = 1500;
                }
                2 => {
                    let g = rng() % 7;
                    let _ = ctrl.debug_global_now(0x02, g);
                    desc = format!("gain_init={}", g);
                    budget = 2800;
                }
                3 => {
                    let v = rng() % 128;
                    let _ = ctrl.debug_global_now(0x03, v);
                    desc = format!("idac_min={}", v);
                    budget = 2800;
                }
                4 => {
                    let v = 1 + rng() % 99;
                    let _ = ctrl.debug_global_now(0x04, v);
                    desc = format!("raw_target={}", v);
                    budget = 2800;
                }
                5 => {
                    let opts = [1u32, 2, 4];
                    let v = opts[(rng() % 3) as usize];
                    let _ = ctrl.debug_global_now(0x01, v);
                    desc = format!("inactive={}", v);
                    budget = 2800;
                }
                6 => {
                    let v = rng() % 2;
                    let _ = ctrl.debug_global_now(0x07, v);
                    desc = format!("sense_cfg={}", v);
                    budget = 2800;
                }
                7 => {
                    let v = rng() % 2;
                    let _ = ctrl.debug_global_now(0x08, v);
                    desc = format!("autocal={}", v);
                    budget = 2800;
                }
                8 => {
                    let d = 4 + rng() % 29;
                    for ch in 0..36u8 {
                        let _ = ctrl.debug_param_now(ch, 0x08, d);
                    }
                    desc = format!("snsClkDiv_all={}", d);
                }
                9 => {
                    let r = 8 + rng() % 7;
                    for ch in 0..36u8 {
                        let _ = ctrl.debug_param_now(ch, 0x07, r);
                    }
                    desc = format!("resolution_all={}", r);
                }
                10 => {
                    let g = rng() % 7;
                    for ch in 0..36u8 {
                        let _ = ctrl.debug_param_now(ch, 0x0B, g);
                    }
                    desc = format!("idac_gain_all={}", g);
                }
                11 => {
                    let _ = ctrl.calibrate(u64::MAX);
                    desc = "CALIBRATE".into();
                    budget = 2800;
                }
                12 => {
                    let _ = ctrl.baseline_reset(u64::MAX);
                    desc = "BASELINE_RESET".into();
                    budget = 1500;
                }
                _ => {
                    let _ = ctrl.reboot_psoc();
                    desc = "REBOOT_PSOC".into();
                    budget = 6000;
                }
            }
            settle(&mut ctrl, 120);
            match check_alive(&mut ctrl, budget) {
                Some(ms) => println!("[SOAK] #{:02} {:22} -> OK (存活, 恢复~{}ms)", i, desc, ms),
                None => {
                    fails += 1;
                    println!(
                        "[SOAK] #{:02} {:22} -> FAIL 链路 {}ms 内未恢复!",
                        i, desc, budget
                    );
                    // 尝试用重启自救, 便于后续步骤继续观察
                    let _ = ctrl.reboot_psoc();
                    let _ = check_alive(&mut ctrl, 6000);
                }
            }
        }
        // 收尾: 半自动 + 统一分辨率 + 中等增益 + 校准, 确认全通道能离轨。
        let _ = ctrl.debug_mode_now(1);
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, 0x07, 12);
            let _ = ctrl.debug_param_now(ch, 0x08, 8);
        }
        let _ = ctrl.debug_global_now(0x02, 5);
        let _ = ctrl.calibrate(u64::MAX);
        settle(&mut ctrl, 800);
        let mut railed = 0usize;
        for ch in 0..36u8 {
            if let Some(s) = ctrl.telem_latest(ch) {
                if s.raw.unwrap_or(0) >= 4090 {
                    railed += 1;
                }
            }
        }
        let _ = ctrl.stop_telemetry();
        let _ = ctrl.debug_mode_now(0);
        println!(
            "[SOAK] ===== 结果: {} 步, 失败 {} 步; 收尾校准后 railed={}/36 =====",
            iters, fails, railed
        );
        println!(
            "[SOAK] {}",
            if fails == 0 {
                "PASS 全部操作后设备均存活/链路可恢复"
            } else {
                "FAIL 存在使设备失联且无法自恢复的操作组合"
            }
        );
        std::process::exit(if fails == 0 { 0 } else { 1 });
    }

    // PSoC 重启(XRES)生效诊断: 物理信号法。SEMI 下校准使 raw 离轨(IDAC 校准值存于 PSoC RAM);
    // XRES 重启会丢失该 RAM 校准, 重新 provision(SEMI 走 APPLY 不重校准)→ raw 重新 railed。
    // 故"校准后离轨 → 重启后重新 railed"即证明 PSoC 真正复位。
    if args.iter().any(|a| a == "--reboot-test") {
        let settle = |ctrl: &mut AppController, ms: u64| {
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(15));
            }
        };
        let read_res0 = |ctrl: &mut AppController| -> Option<u32> {
            for ch in 0..36u8 {
                let _ = ctrl.request_params(ch);
            }
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(1000) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(15));
            }
            ctrl.param(0, 0x07)
        };
        println!("[REBOOT] 切 SEMI...");
        let _ = ctrl.debug_mode_now(1);
        settle(&mut ctrl, 300);
        // 在 CH0 widgetContext 写入异常分辨率 16(生成默认为 10/12)。PSoC XRES 重启会把 widgetContext
        // 重置为生成默认; XRES(~2ms)+boot(~5ms) << 400ms 去抖, RP2040 不会重 provision → 16 应消失。
        println!("[REBOOT] 在 CH0 设分辨率=16(标记值)...");
        let _ = ctrl.debug_param_now(0, 0x07, 16);
        settle(&mut ctrl, 200);
        let before = read_res0(&mut ctrl);
        println!("[REBOOT] 重启前 CH0 分辨率={:?} (应=16)", before);
        println!("[REBOOT] 发送 REBOOT_PSOC(脉冲 XRES)...");
        let _ = ctrl.reboot_psoc();
        // 轮询链路恢复(宽限修复关键验证): 最多 8s, 报告恢复耗时。
        let t0 = std::time::Instant::now();
        let mut recovered_ms: Option<u128> = None;
        while t0.elapsed() < Duration::from_millis(8000) {
            let _ = ctrl.resend_hello();
            settle(&mut ctrl, 250);
            if ctrl
                .device_info()
                .map(|d| d.psoc_link_valid)
                .unwrap_or(false)
            {
                recovered_ms = Some(t0.elapsed().as_millis());
                break;
            }
        }
        match recovered_ms {
            Some(ms) => println!(
                "[REBOOT] 链路在重启后 ~{}ms 恢复 link_valid=true (宽限修复生效)",
                ms
            ),
            None => println!("[REBOOT] FAIL 链路 8s 内未恢复(仍需修复)"),
        }
        settle(&mut ctrl, 500);
        let after = read_res0(&mut ctrl);
        println!(
            "[REBOOT] 重启后 CH0 分辨率={:?} (若≠16=widgetContext 已重置=真正重启, 白灯应闪亮)",
            after
        );
        let _ = ctrl.debug_param_now(0, 0x07, 12);
        println!(
            "[REBOOT] {}",
            if after != Some(16) && before == Some(16) {
                "PASS PSoC 已真正 XRES 重启(标记分辨率被重置)"
            } else {
                "FAIL 标记分辨率仍在, XRES 复位未生效"
            }
        );
        std::process::exit(0);
    }

    // IDAC增益夹紧(防越界崩溃) + inactive屏障 生效性诊断。
    if args.iter().any(|a| a == "--gain-inactive-test") {
        let get_g = |ctrl: &mut AppController, id: u8| -> Option<u32> {
            let _ = ctrl.global_get(id);
            let s = std::time::Instant::now();
            while s.elapsed() < Duration::from_millis(400) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(12));
            }
            ctrl.global(id)
        };
        println!(
            "[GI] 基线: IDAC_GAIN_INIT(0x02)={:?} INACTIVE_SNS(0x01)={:?}",
            get_g(&mut ctrl, 0x02),
            get_g(&mut ctrl, 0x01)
        );

        // 增益=7(越界值): 应被夹紧拒绝且设备不崩溃。用 debug_global_now(不写草稿)使回读=设备真值。
        let _ = ctrl.debug_global_now(0x02, 7);
        thread::sleep(Duration::from_millis(300));
        let g7 = get_g(&mut ctrl, 0x02);
        println!(
            "[GI] 设增益=7后回读(设备真值)={:?} → {}",
            g7,
            if g7 != Some(7) {
                "PASS 被拒(未越界)"
            } else {
                "FAIL 接受了7(会越界崩溃)"
            }
        );
        // 增益=6(合法上限): 应被接受。
        let _ = ctrl.debug_global_now(0x02, 6);
        thread::sleep(Duration::from_millis(300));
        let g6 = get_g(&mut ctrl, 0x02);
        println!(
            "[GI] 设增益=6后回读(设备真值)={:?} → {}",
            g6,
            if g6 == Some(6) {
                "PASS 接受"
            } else {
                "FAIL 未接受合法值6"
            }
        );

        // per-channel IDAC_GAIN(0x0B) ch0: 7 拒 / 6 收。
        let _ = ctrl.debug_param_now(0, 0x0B, 7);
        thread::sleep(Duration::from_millis(200));
        let _ = ctrl.request_params(0);
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(12));
        }
        let p7 = ctrl.param(0, 0x0B);
        println!(
            "[GI] CH0 param增益=7后回读={:?} → {}",
            p7,
            if p7 != Some(7) {
                "PASS 被拒"
            } else {
                "FAIL 接受7"
            }
        );

        // inactive: 2(High-Z)/4(Shield) 应生效(RP2040 自动 global_commit → PSoC Init 重算)。设备真值回读。
        let _ = ctrl.debug_global_now(0x01, 2);
        thread::sleep(Duration::from_millis(400));
        let iz = get_g(&mut ctrl, 0x01);
        println!(
            "[GI] 设inactive=2(High-Z)后回读(设备真值)={:?} → {}",
            iz,
            if iz == Some(2) {
                "PASS 生效"
            } else {
                "FAIL 未生效"
            }
        );
        let _ = ctrl.debug_global_now(0x01, 4);
        thread::sleep(Duration::from_millis(400));
        let ish = get_g(&mut ctrl, 0x01);
        println!(
            "[GI] 设inactive=4(Shield)后回读(设备真值)={:?} → {}",
            ish,
            if ish == Some(4) {
                "PASS 生效"
            } else {
                "FAIL 未生效"
            }
        );

        // 复位到安全默认(GND, 增益4)。
        let _ = ctrl.debug_global_now(0x01, 1);
        let _ = ctrl.debug_global_now(0x02, 4);
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, 0x0B, 4);
        }
        thread::sleep(Duration::from_millis(300));
        println!("[GI] 已复位 inactive=GND 增益=4; 全程设备存活(能回读)=未崩溃");
        std::process::exit(0);
    }

    // 建立"半自动默认基线": 先让 PSoC 在 AUTO 下把阈值/snsClk/IDAC 自动算好, 显式捕获成手动基线,
    // 切回 SEMI 并持久化, 最后重启核验 store 真的能把这套值下发回去。
    // 用途: 手动参数被污染成 0 之后重建一套可用且安全的起点(0 阈值在 SEMI 下等于一直判定触摸)。
    if args.iter().any(|a| a == "--semi-baseline") {
        let settle = |ctrl: &mut AppController, ms: u64| {
            let w = std::time::Instant::now();
            while w.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(10));
            }
        };
        // 阈值取 PSoC 生成配置的出厂默认(cycfg_capsense.c: fingerTh=60 noiseTh=30 nNoiseTh=30
        // hysteresis=7 onDebounce=3 lowBslnRst=30)。不从设备回读: AUTO 模式并不会重算这些阈值
        // (生成配置里它们不是自动项), 阈值一旦被写成 0 就一直是 0, 回读只会把 0 再收一遍。
        // 分辨率/snsClk/IDAC 保持设备现值不动 —— 那些是校准与自适应的结果, 不该被基线覆盖。
        // ★阈值按实测噪声地板抬高★: 生成配置的 fingerTh=60 是给示例板的, 本面板基线复位后静止
        // 残余 diff 仍有 50~99(通道间差异大), 60 的阈值会持续误判触摸 —— 而触控板同时是 HID 键盘,
        // 误判会直接往前台窗口发按键(实测把本程序界面上的按钮点了)。故 fingerTh 取 150、噪声阈值取 60,
        // 先保证"静止不误触发"这个安全底线; 真实手感留给用户在单通道精调里按 diff 峰值微调。
        const BASE: [(u8, u32); 6] = [
            (0x01, 150), // FINGER_TH
            (0x02, 60),  // NOISE_TH
            (0x03, 60),  // NEG_NOISE_TH
            (0x04, 7),   // HYSTERESIS
            (0x05, 3),   // ON_DEBOUNCE
            (0x06, 30),  // LOW_BSLN_RST
        ];
        println!("[BASE] 1) 切 SEMI(手动参数生效)...");
        let _ = ctrl.debug_mode_now(1);
        settle(&mut ctrl, 1200);
        // ★可选 --div N: 顺带把 SNS_CLK_DIV 写进全 36 通道★
        // 默认仍不动分频(它是校准/自适应的结果, 不该被基线覆盖)。但 store 里一旦存了对本面板不可用的
        // 旧分频(实测 8: 高频下传感器建立不足 → raw 逼近满量程、整轮扫描被拖到近 1s), 每次 provision
        // 都会把它重新推给 PSoC, 盖掉 PSoC 自己归一的 32 —— 带外 SWD 已实证: clk_boot=32、clk_now=8、
        // clk_set=36(逐通道 SET_PARAM 写入)。SET_PARAM 是写穿路径, 显式写一次即可同时纠正 store 并持久化。
        let div_override: Option<u32> = args
            .iter()
            .position(|a| a == "--div")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse().ok());
        if let Some(d) = div_override {
            println!("[BASE] 2) 把出厂默认阈值 + SNS_CLK_DIV={} 写进全 36 通道...", d);
        } else {
            println!("[BASE] 2) 把出厂默认阈值写进全 36 通道(分频保持设备现值; 需要纠正时加 --div 32)...");
        }
        for ch in 0u8..36u8 {
            for (pid, v) in BASE {
                let _ = ctrl.debug_param_now(ch, pid, v);
            }
            if let Some(d) = div_override {
                let _ = ctrl.debug_param_now(ch, 0x08, d); // 0x08 = PARAM_SNS_CLK_DIV
            }
            settle(&mut ctrl, 40);
        }
        settle(&mut ctrl, 1500);
        println!("[BASE] 3) 持久化到 flash...");
        if let Err(e) = ctrl.save_config() {
            println!("[BASE] FAIL save_config: {}", e);
            std::process::exit(1);
        }
        settle(&mut ctrl, 3000);
        println!("[BASE] 完成。请重启设备后用 --param-dump 核验 store 下发结果。");
        std::process::exit(0);
    }

    // 全通道基线复位: 把 baseline 拉回当前 raw, 消掉漂移导致的常触发(diff 长期高于阈值)。
    if args.iter().any(|a| a == "--baseline-reset") {
        let _ = ctrl.baseline_reset(0xFFFFFFFF_FFFFFFFFu64);
        let w = std::time::Instant::now();
        while w.elapsed() < Duration::from_millis(3000) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(10));
        }
        println!("[BSLN] 已请求全通道基线复位");
        std::process::exit(0);
    }

    // 只切 CSD 模式(0=自动校准 1=半自动手动), 不动任何参数。用于验证"AUTO 只让手动设置失效、
    // 切回 SEMI 后 RP2040 store 里的手动参数应当原样恢复"。
    if let Some(i) = args.iter().position(|a| a == "--set-mode") {
        let mode: u8 = args.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or(1);
        let _ = ctrl.debug_mode_now(mode);
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(1500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(10));
        }
        println!(
            "[MODE] 已下发 CSD 模式 = {} ({})",
            mode,
            if mode == 0 {
                "自动校准"
            } else {
                "半自动手动"
            }
        );
        std::process::exit(0);
    }

    // 通道参数真值转储: 直接从设备回读指定通道(缺省全 36 通道)的全部已知参数, 用来判定
    // "重启 UI 后参数显示 0" 到底是上位机没读到, 还是设备侧真的被清成了 0。
    if args.iter().any(|a| a == "--param-dump") {
        use mai2control_ui::proto::KNOWN_PARAM_IDS;
        let only_ch: Option<u8> = args
            .iter()
            .position(|a| a == "--ch")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse().ok());
        let settle = |ctrl: &mut AppController, ms: u64| {
            let w = std::time::Instant::now();
            while w.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(10));
            }
        };
        // 用批量变体(一帧回 36 通道)逐参数拉, 比 36×N 条单发省一个数量级的往返。
        for &pid in KNOWN_PARAM_IDS.iter() {
            let _ = ctrl.request_param_all_channels(pid);
            settle(&mut ctrl, 250);
        }
        settle(&mut ctrl, 800);
        let header: Vec<String> = KNOWN_PARAM_IDS
            .iter()
            .map(|p| format!("0x{:02X}", p))
            .collect();
        println!("[PDUMP] ch  {}", header.join("     "));
        for ch in 0u8..36u8 {
            if let Some(only) = only_ch {
                if ch != only {
                    continue;
                }
            }
            let vals: Vec<String> = KNOWN_PARAM_IDS
                .iter()
                .map(|&pid| match ctrl.param(ch, pid) {
                    Some(v) => format!("{:>6}", v),
                    None => "     -".to_string(),
                })
                .collect();
            println!("[PDUMP] {:>2}  {}", ch, vals.join(" "));
        }
        std::process::exit(0);
    }

    // GND 降速机理判据: 对 inactive=GND/High-Z 各测 分辨率 8 与 12 的实测扫描周期。
    // 转换时长 ∝ 2^res(subConv=2^res/div, 每子转换 div 个 ModClk, 乘积与 div 无关);
    // 故若 GND 的多出开销随 res 一起缩小 → 慢在硬件转换等待(Cp 拖长建立);
    // 若 res 8→12 周期几乎不变(开销恒定) → 慢在与转换无关的固定开销(引脚状态切换/setup)。
    if args.iter().any(|a| a == "--gnd-scale-probe") {
        use mai2control_ui::proto::FIELD_STATS;
        const CLK_DIV: u8 = 0x08;
        const CLK_RES: u8 = 0x07;
        const G_INACTIVE: u8 = 0x01;
        let settle = |ctrl: &mut AppController, ms: u64| {
            let w = std::time::Instant::now();
            while w.elapsed() < Duration::from_millis(ms) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(10));
            }
        };
        println!("[GNDP] 切半自动手动模式(SET_MODE=1)...");
        let _ = ctrl.debug_mode_now(1);
        settle(&mut ctrl, 400);
        // ★不用设备上报的 scan_period_us 单次值★: 它是 1e6/整数sps(psoc.cpp:130), 5~7Hz 时量化
        // 误差 ±25%。改为 15s 内多次独立窗口取均值(见下), 把量化误差摊平。
        // (device_info().psoc_generation 不可用: 它只在连接时取一次, 运行中不刷新。)
        // 矩阵: 控制点 + GND 下变分辨率(改子转换数与转换时长) + GND 下变分频(只改子转换数, 转换时长不变)。
        let points: [(u32, u32, u32); 7] = [
            (2, 12, 32), // High-Z 基准
            (2, 8, 32),
            (1, 8, 32), // GND
            (1, 10, 32),
            (1, 12, 32),
            (1, 12, 8),  // 同 res 变 div: subConv ×4, 转换时长不变
            (1, 12, 64), // subConv ÷2
        ];
        let mut last_inactive = 0u32;
        for &(inactive, res, div) in points.iter() {
            if inactive != last_inactive {
                let _ = ctrl.debug_global_now(G_INACTIVE, inactive);
                settle(&mut ctrl, 300);
                let _ = ctrl.global_commit(); // 必须 commit, 否则只写影子不生效
                settle(&mut ctrl, 3000);
                last_inactive = inactive;
            }
            for ch in 0..36u8 {
                let _ = ctrl.debug_param_now(ch, CLK_DIV, div);
                let _ = ctrl.debug_param_now(ch, CLK_RES, res);
            }
            settle(&mut ctrl, 800);
            let _ = ctrl.start_telemetry(30, FIELD_STATS, u64::MAX);
            settle(&mut ctrl, 1500);
            // 设备每 500ms 用 scan_count 增量算一次 sps(整数量化)。取 15s 内多次独立窗口求均值,
            // 把低速档 ±25% 的量化误差摊平到 <1%。
            let mut samples: Vec<u32> = Vec::new();
            let mut last = u32::MAX;
            let w = std::time::Instant::now();
            while w.elapsed() < Duration::from_millis(15000) {
                ctrl.poll();
                let s = ctrl.telem_samples_per_sec();
                if s != last {
                    last = s;
                    if s > 0 {
                        samples.push(s);
                    }
                }
                thread::sleep(Duration::from_millis(20));
            }
            let tag = if inactive == 1 { "GND" } else { "High-Z" };
            let sub_conv = (1u32 << res) / div.max(1);
            if samples.is_empty() {
                println!(
                    "[GNDP] inactive={:<6} res={:>2} div={:>2} → 无有效 sps 采样",
                    tag, res, div
                );
            } else {
                let mean: f64 =
                    samples.iter().map(|&v| v as f64).sum::<f64>() / samples.len() as f64;
                let cycle = 1e6 / mean;
                println!(
                    "[GNDP] inactive={:<6} res={:>2} div={:>2} subConv={:>4} → sps均值={:.2} 每轮={:.0}us 每通道={:.1}us (n={} 样本 min={} max={})",
                    tag,
                    res,
                    div,
                    sub_conv,
                    mean,
                    cycle,
                    cycle / 36.0,
                    samples.len(),
                    samples.iter().min().unwrap(),
                    samples.iter().max().unwrap()
                );
            }
            let _ = ctrl.stop_telemetry();
            settle(&mut ctrl, 300);
        }
        // 复位到常态: High-Z + res12 + div32 + 自动模式。
        let _ = ctrl.debug_global_now(G_INACTIVE, 2);
        settle(&mut ctrl, 300);
        let _ = ctrl.global_commit();
        settle(&mut ctrl, 2500);
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, CLK_RES, 12);
            let _ = ctrl.debug_param_now(ch, CLK_DIV, 32);
        }
        settle(&mut ctrl, 500);
        let _ = ctrl.calibrate(u64::MAX);
        let _ = ctrl.debug_mode_now(0);
        println!("[GNDP] 已复位 High-Z/res12/div32/自动模式");
        println!(
            "[GNDP] 判读: GND 多出的开销若随 res 8→12 一起变大=硬件转换等待; 若恒定=固定 setup 开销"
        );
        std::process::exit(0);
    }

    // 时钟生效实验: 半自动模式下, 直接把全 36 通道 SNS_CLK_DIV 设成不同值 + APPLY,
    // 测每档的设备实测扫描周期(scan_period_us)。周期应随 div 近似线性变化; 若恒定=时钟未真正生效。
    if args.iter().any(|a| a == "--clk-probe") {
        use mai2control_ui::proto::FIELD_STATS;
        const CLK_DIV: u8 = 0x08;
        println!("[CLK] 切半自动手动模式(SET_MODE=1)...");
        let _ = ctrl.debug_mode_now(1);
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(400) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        const CLK_RES: u8 = 0x07; // PARAM_RESOLUTION
        // (A) 固定分辨率, 变 div 8→48: 验证 div 是否影响周期(CSDv2 预期: 不影响, 因 conversionsNum∝1/div 抵消)。
        for &div in &[8u32, 48u32] {
            for ch in 0..36u8 {
                let _ = ctrl.debug_param_now(ch, CLK_DIV, div);
            }
            let _ = ctrl.calibrate(u64::MAX);
            let _ = ctrl.start_telemetry(30, FIELD_STATS, u64::MAX);
            let w = std::time::Instant::now();
            while w.elapsed() < Duration::from_millis(2000) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(20));
            }
            println!(
                "[CLK] (A) SNS_CLK_DIV={:>2} res=固定 → scan_period_us={} sps={}",
                div,
                ctrl.telem_scan_period_us(),
                ctrl.telem_samples_per_sec()
            );
            let _ = ctrl.stop_telemetry();
            thread::sleep(Duration::from_millis(150));
        }
        // (B) 固定 div=8, 变分辨率 8→10→12: 验证分辨率是否驱动周期(CSDv2 预期: 周期∝2^res)。
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, CLK_DIV, 8);
        }
        for &res in &[8u32, 10u32, 12u32] {
            for ch in 0..36u8 {
                let _ = ctrl.debug_param_now(ch, CLK_RES, res);
            }
            let _ = ctrl.calibrate(u64::MAX);
            let _ = ctrl.start_telemetry(30, FIELD_STATS, u64::MAX);
            let w = std::time::Instant::now();
            while w.elapsed() < Duration::from_millis(2000) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(20));
            }
            println!(
                "[CLK] (B) RESOLUTION={:>2} div=8 → scan_period_us={} sps={}",
                res,
                ctrl.telem_scan_period_us(),
                ctrl.telem_samples_per_sec()
            );
            let _ = ctrl.stop_telemetry();
            thread::sleep(Duration::from_millis(150));
        }
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, CLK_RES, 10);
        } // 复位分辨率
        let _ = ctrl.calibrate(u64::MAX);
        let _ = ctrl.debug_mode_now(0); // 复位回自动模式
        println!("[CLK] 若周期随 div 近似线性(8→48 约 6×)则时钟生效; 恒定则未真正实时控制");
        std::process::exit(0);
    }

    // 算法回读实验: 读设备信息 + C 源(映射表) + ASM 机器码, 打印长度与内容首部,
    // 确认"读取信息"能否真正取回设备保存的(已滤注释)C 源与机器码。
    if args.iter().any(|a| a == "--algo-dump") {
        let _ = ctrl.algo_get_info();
        let _ = ctrl.request_algo_src();
        let _ = ctrl.request_algo_code();
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(1200) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        match ctrl.algo_info() {
            Some(i) => println!(
                "[ALGO] info: is_default={} psoc_valid={} len={} crc16=0x{:04X}",
                i.is_default, i.psoc_valid, i.len, i.crc16
            ),
            None => println!("[ALGO] info: <未取到>"),
        }
        let src = ctrl.algo_device_src();
        println!("[ALGO] device C src: {} 字节", src.len());
        for (n, line) in src.lines().take(6).enumerate() {
            println!("[ALGO]   src[{}]: {}", n, line);
        }
        let code = ctrl.algo_device_code_hex();
        println!("[ALGO] device ASM code hex: {} 字符", code.len());
        std::process::exit(0);
    }

    // 自持 debug: 武装/解除"崩溃→进 BOOTSEL"。武装后固件一旦看门狗复位即进烧录, 便于自动重烧恢复。
    if args.iter().any(|a| a == "--arm-crash-bootsel") {
        match ctrl.set_crash_bootsel(true) {
            Ok(()) => println!("[DBG] 已武装: 运行中崩溃将自动进 BOOTSEL(自持 debug)"),
            Err(e) => println!("[DBG] 武装失败: {}", e),
        }
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(300) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        std::process::exit(0);
    }
    if args.iter().any(|a| a == "--disarm-crash-bootsel") {
        match ctrl.set_crash_bootsel(false) {
            Ok(()) => println!("[DBG] 已解除: 运行中崩溃仅正常重启(默认/生产)"),
            Err(e) => println!("[DBG] 解除失败: {}", e),
        }
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(300) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        std::process::exit(0);
    }

    // Strict smoke requires the unambiguous diagnostic RP image and a complete S455 path.
    if smoke_only {
        let Some(info) = ctrl.device_info() else {
            println!("[SELFTEST] FAIL DEVICE_INFO unavailable after handshake");
            std::process::exit(1);
        };
        let Some(diag) = &info.diagnostics else {
            println!("[SELFTEST] FAIL legacy DEVICE_INFO lacks bring-up diagnostics");
            std::process::exit(1);
        };
        let silicon_device_mask = 0xFFFF_00FFu32; // programming spec: ignore revision byte
        // ★版本判定 = "不低于最低要求", 不是"完全相等"★
        // 版本号已改为编译时间戳(YYMMDDHHMM), 每次编译都变。若继续要求相等, 就等于强制"主机与固件
        // 必须来自同一次构建" —— 只要先编固件后编主机的顺序错一次, 或设备上跑着比本次源码更新的固件,
        // 就会误判成身份不符(实测连撞两次 identity_ok=false, 而设备其实完全正常)。
        // 上位机真正需要的只是"固件够不够新以支撑本版协议", 故取 >=:
        // 下限沿用 build.rs 从固件头解析出的时间戳(= 本仓当前固件), 设备更新则放行、更旧才拦。
        let min_rp_stamp = env!("EXPECTED_RP_FW_VERSION")
            .parse::<u32>()
            .expect("valid generated RP2040 version");
        let min_psoc_stamp = env!("EXPECTED_PSOC_FW_VERSION")
            .parse::<u32>()
            .expect("valid generated PSoC version");
        let identity_ok = info.fw_version >= min_rp_stamp
            && diag.rp_build_id == RP_BUILD_ID_DIAGNOSTIC_V1
            && diag.embedded_psoc_version >= min_psoc_stamp
            && (diag.actual_silicon_id & silicon_device_mask)
                == (EXPECTED_PSOC_S455_ID & silicon_device_mask);
        let runtime_ok = diag.flash_ok()
            && diag.has(BRINGUP_FLAG_CHECKSUM | BRINGUP_FLAG_LINK | BRINGUP_FLAG_SNAPSHOT)
            && info.psoc_generation > 0
            && info.psoc_link_valid
            && info.psoc_snapshot_valid
            && diag.failure_stage == 0;
        if !identity_ok || !runtime_ok {
            println!(
                "[SELFTEST] FAIL diagnostic identity/runtime: identity_ok={} runtime_ok={} last={} failure={} flags=0x{:04X} (最低要求 RP>={} PSoC>={}, 实际 RP={} PSoC={})",
                identity_ok, runtime_ok, diag.last_stage, diag.failure_stage, diag.flags,
                min_rp_stamp, min_psoc_stamp, info.fw_version, diag.embedded_psoc_version
            );
            std::process::exit(1);
        }
        println!(
            "[SELFTEST] PSOC SNAPSHOT PASS generation={} silicon=0x{:08X} flags=0x{:04X}",
            info.psoc_generation, diag.actual_silicon_id, diag.flags
        );
        println!("[SELFTEST] WINUSB SMOKE PASS");
        std::process::exit(0);
    }

    // Phase B 配置阶段:把 CSD 配置写入 RP2040 真相源并持久化(供重启/重刷后下发)。
    if csd_provision {
        const FT: u8 = 0x01; // FINGER_TH
        println!("[SELFTEST] CSD provision: 捕获 PSoC 当前参数入 store...");
        if let Err(e) = ctrl.csd_capture() {
            println!("[SELFTEST] FAIL csd_capture: {}", e);
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(300)); // 等 324 次 GET_PARAM 完成
        println!("[SELFTEST] 切换半自动 + 设 FINGER_TH ch0=199 + 保存...");
        if let Err(e) = ctrl.set_mode(1) {
            println!("[SELFTEST] FAIL set_mode: {}", e);
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(50));
        if let Err(e) = ctrl.set_param(0, FT, 199) {
            println!("[SELFTEST] FAIL set_param: {}", e);
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(50));
        if let Err(e) = ctrl.save_config() {
            println!("[SELFTEST] FAIL save_config: {}", e);
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(300)); // 等 flash 写入
        println!("[SELFTEST] CSD PROVISION DONE (semi + FINGER_TH ch0=199 已持久化)");
        std::process::exit(0);
    }

    // Phase B 验证阶段:重启/重刷后读回,证明 RP2040 已从持久化 store 下发到(被 wiped 的)PSoC。
    if csd_verify {
        if let Err(e) = ctrl.request_params(0) {
            println!("[SELFTEST] FAIL request_params: {}", e);
            std::process::exit(1);
        }
        let start = std::time::Instant::now();
        let to = Duration::from_millis(PARAM_GET_ALL_TIMEOUT_MS);
        loop {
            ctrl.poll();
            if ctrl.param_version() > 0 && !ctrl.params_of(0).is_empty() {
                break;
            }
            if start.elapsed() > to {
                break;
            }
            thread::sleep(Duration::from_millis(50));
        }
        let ft = ctrl.param(0, 0x01);
        match ft {
            Some(199) => {
                println!(
                    "[SELFTEST] CSD VERIFY PASS: FINGER_TH ch0=199 (启动下发成功,PSoC 无状态验证通过)"
                );
                std::process::exit(0);
            }
            other => {
                println!(
                    "[SELFTEST] CSD VERIFY FAIL: FINGER_TH ch0 期望 199 实得 {:?} (启动下发未生效)",
                    other
                );
                std::process::exit(1);
            }
        }
    }

    // JIT 算法引擎闭环: 读信息 → 编译并上传测试算法 → 校验 psoc_valid+非默认 → 恢复默认 → 校验默认。
    if algo_test {
        let pump = |ctrl: &mut AppController, n: u32| {
            for _ in 0..n {
                ctrl.poll();
                thread::sleep(Duration::from_millis(20));
            }
        };
        println!("[SELFTEST] ALGO: 读取当前算法信息...");
        let _ = ctrl.algo_get_info();
        pump(&mut ctrl, 30);
        match ctrl.algo_info() {
            Some(i) => println!(
                "[SELFTEST]   is_default={} psoc_valid={} len={} crc16=0x{:04X}",
                i.is_default, i.psoc_valid, i.len, i.crc16
            ),
            None => {
                println!("[SELFTEST] FAIL 未收到 ALGO_INFO");
                std::process::exit(1);
            }
        }
        let src = "#include <stddef.h>\n#include \"psoc_algo_abi.h\"\nvoid algo(algo_io_t* io){ io->out_active = (io->base_active!=0u)?1u:0u; }\n";
        println!("[SELFTEST] ALGO: 编译并上传测试算法(base_active 透传)...");
        if let Err(e) = ctrl.compile_and_upload(src) {
            println!("[SELFTEST] FAIL 编译/上传: {}", e);
            std::process::exit(1);
        }
        pump(&mut ctrl, 40);
        let _ = ctrl.algo_get_info();
        pump(&mut ctrl, 40);
        match ctrl.algo_info() {
            Some(i) if i.psoc_valid && !i.is_default && i.len > 0 => println!(
                "[SELFTEST]   上传后 is_default={} psoc_valid={} len={}",
                i.is_default, i.psoc_valid, i.len
            ),
            other => {
                println!(
                    "[SELFTEST] ALGO UPLOAD FAIL: {:?}",
                    other.map(|i| (i.is_default, i.psoc_valid, i.len))
                );
                std::process::exit(1);
            }
        }
        println!("[SELFTEST] ALGO: 恢复默认(v3.1 HDR)...");
        let _ = ctrl.algo_reset_default();
        pump(&mut ctrl, 50);
        let _ = ctrl.algo_get_info();
        pump(&mut ctrl, 40);
        match ctrl.algo_info() {
            Some(i) if i.is_default && i.psoc_valid => {
                println!("[SELFTEST] ALGO PASS: 上传+恢复默认均生效");
                std::process::exit(0);
            }
            other => {
                println!(
                    "[SELFTEST] ALGO RESET FAIL: {:?}",
                    other.map(|i| (i.is_default, i.psoc_valid, i.len))
                );
                std::process::exit(1);
            }
        }
    }

    // 全局 CSD 配置闭环: 读全部 → 设未激活传感器=High-Z(2) → 设备回读校验 → 恢复 GND(1)。
    if global_test {
        let pump = |ctrl: &mut AppController, n: u32| {
            for _ in 0..n {
                ctrl.poll();
                thread::sleep(Duration::from_millis(20));
            }
        };
        println!("[SELFTEST] GLOBAL: 读取全部全局配置...");
        let _ = ctrl.global_get_all();
        pump(&mut ctrl, 30);
        for id in 1u8..=6 {
            if let Some(v) = ctrl.global(id) {
                println!("[SELFTEST]   gparam {} = {}", id, v);
            }
        }
        println!("[SELFTEST] GLOBAL: 设未激活传感器连接=High-Z(2) + APPLY...");
        let _ = ctrl.global_set(1, 2);
        pump(&mut ctrl, 40);
        let _ = ctrl.global_get(1); // 设备回读覆盖本地乐观值,真正校验设备
        pump(&mut ctrl, 30);
        let got = ctrl.global(1);
        let _ = ctrl.global_set(1, 1); // 恢复 GND
        pump(&mut ctrl, 40);
        match got {
            Some(2) => {
                println!("[SELFTEST] GLOBAL PASS: inactive_sns 设备回读=2(High-Z)");
                std::process::exit(0);
            }
            other => {
                println!("[SELFTEST] GLOBAL FAIL: 期望 2 实得 {:?}", other);
                std::process::exit(1);
            }
        }
    }

    // 键盘闭环: 读物理键码表(12) + 触控映射表(34) + 物理实时态; SET_MAP round-trip 校验设备回读。
    if kbd_test {
        let pump = |ctrl: &mut AppController, n: u32| {
            for _ in 0..n {
                ctrl.poll();
                thread::sleep(Duration::from_millis(20));
            }
        };
        println!("[SELFTEST] KBD: 读取物理键码表 / 触控映射表 / 物理实时态...");
        let _ = ctrl.kbd_request_map();
        let _ = ctrl.kbd_request_touchmap();
        let _ = ctrl.kbd_request_state();
        pump(&mut ctrl, 30);
        let phys: Vec<u8> = (0..12u8).map(|i| ctrl.kbd_map(i)).collect();
        println!("[SELFTEST]   物理键码(12) = {:02X?}", phys);
        println!(
            "[SELFTEST]   物理实时态 = 0x{:03X} (需按键才非0)",
            ctrl.kbd_state()
        );
        let zones: Vec<u8> = (0..34u8).map(|z| ctrl.kbd_touch_keycode(z)).collect();
        println!("[SELFTEST]   触控映射(34, 0=不映射) = {:02X?}", zones);
        // round-trip: 物理键0 改成 KEY_A(0x04) → 设备回读校验 → 恢复原值。
        let orig = ctrl.kbd_map(0);
        let test_code: u8 = if orig == 0x04 { 0x05 } else { 0x04 };
        println!(
            "[SELFTEST] KBD: SET_MAP 键0 {:#04X} -> {:#04X} (写穿+save)...",
            orig, test_code
        );
        let _ = ctrl.kbd_set_map(0, test_code, 0);
        pump(&mut ctrl, 30);
        let _ = ctrl.kbd_request_map(); // 设备回读覆盖本地乐观值
        pump(&mut ctrl, 30);
        let got = ctrl.kbd_map(0);
        let _ = ctrl.kbd_set_map(0, orig, 0); // 恢复
        pump(&mut ctrl, 30);
        if got == test_code {
            println!("[SELFTEST] KBD PASS: SET_MAP 设备回读={:#04X}", got);
            std::process::exit(0);
        }
        println!(
            "[SELFTEST] KBD FAIL: 期望 {:#04X} 实得 {:#04X}",
            test_code, got
        );
        std::process::exit(1);
    }

    // 无头 soak：完全模拟 UI 的持久连接 + 16ms 轮询，逐个驱动所有功能，最后长时 idle。
    // 用于复现 GUI 的 "endpoint stalled / 几乎不可用"，并回验所有特性稳定联通。
    if cfg_only {
        println!("[SELFTEST] CFG_GET_ALL 观测...");
        if let Err(e) = ctrl.request_config_all() {
            println!("[SELFTEST] FAIL request_config_all: {}", e);
            std::process::exit(1);
        }
        let start = std::time::Instant::now();
        loop {
            ctrl.poll();
            if start.elapsed().as_millis() % 200 < 30 {
                // 采样打印(粗略)
            }
            if ctrl.state() == ConnState::Disconnected {
                println!("[SELFTEST] 断开! err={:?}", ctrl.last_error());
                std::process::exit(1);
            }
            if start.elapsed() > Duration::from_millis(2500) {
                break;
            }
            thread::sleep(Duration::from_millis(50));
        }
        println!(
            "[SELFTEST] 2.5s 后 config_entries = {}",
            ctrl.config_entries().len()
        );
        std::process::exit(0);
    }

    // --global-set ID VAL: 直接下发单个全局 CSD 项(诊断/救砖用, 绕过草稿)。可重复多组。
    // 例: --global-set 8 1 (AUTO_CALIBRATE_EN=1) --global-set 2 4 (IDAC_GAIN_INIT=4)
    if args.iter().any(|a| a == "--global-set") {
        let mut i = 0usize;
        while i < args.len() {
            if args[i] == "--global-set" {
                let id = args.get(i + 1).and_then(|s| s.parse::<u8>().ok());
                let val = args.get(i + 2).and_then(|s| s.parse::<u32>().ok());
                if let (Some(id), Some(val)) = (id, val) {
                    let _ = ctrl.debug_global_now(id, val);
                    println!("[GSET] 下发全局项 0x{:02X} = {}", id, val);
                    let s = std::time::Instant::now();
                    while s.elapsed() < Duration::from_millis(900) {
                        ctrl.poll();
                        thread::sleep(Duration::from_millis(10));
                    }
                }
                i += 3;
            } else {
                i += 1;
            }
        }
        // ★必须 commit★: GLOBAL_SET 只写 PSoC RAM 影子, 不重初始化; 少了这一步"设了等于没设"
        // (实测 inactive_sns 改 GND 后采样率不变, 就是漏了 commit 造成的假阴性)。
        let _ = ctrl.global_commit();
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(2500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(10));
        }
        // 回读全部 8 项(含 0x07 IDAC_SENSE_CONFIG / 0x08 AUTO_CALIBRATE_EN, 旧 dump 只到 0x06)。
        let _ = ctrl.global_get_all();
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(800) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(10));
        }
        // 0x09 = GPARAM_BOOT_OVERRIDE(只读, PSoC 启动强制改写位掩码)。旧 PSoC 固件不认该 id 返回 0,
        // 故它同时充当"PSoC 是否已跑上新镜像"的判据。
        for id in 1u8..=8 {
            println!("[GSET] 回读 0x{:02X} = {:?}", id, ctrl.global(id));
        }
        // 0x09 = GPARAM_BOOT_OVERRIDE(只读, PSoC 启动强制改写位掩码)。GET_ALL 只回 8 项, 必须单项读。
        // 旧 PSoC 固件不认该 id(返回 0/NAK), 故它同时充当"PSoC 是否已跑上新镜像"的判据。
        let _ = ctrl.global_get(9);
        let s = std::time::Instant::now();
        while s.elapsed() < Duration::from_millis(1200) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(10));
        }
        println!("[GSET] 单项回读 0x09(BOOT_OVERRIDE) = {:?}", ctrl.global(9));
        std::process::exit(0);
    }

    if reset_config {
        println!("[SELFTEST] RESET_DEFAULTS 恢复配置...");
        if let Err(e) = ctrl.reset_defaults() {
            println!("[SELFTEST] FAIL reset_defaults: {}", e);
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(600));
        let _ = ctrl.request_config_all();
        let start = std::time::Instant::now();
        loop {
            ctrl.poll();
            if ctrl.config_entries().len() > 0 {
                break;
            }
            if start.elapsed() > Duration::from_millis(1500) {
                break;
            }
            thread::sleep(Duration::from_millis(30));
        }
        let n = ctrl.config_entries().len();
        println!("[SELFTEST] RESET 后配置项 = {}", n);
        if n >= 40 {
            println!("[SELFTEST] CONFIG RECOVER PASS");
            std::process::exit(0);
        }
        println!("[SELFTEST] CONFIG RECOVER FAIL (仍 <40 项)");
        std::process::exit(1);
    }

    if idle_only {
        println!(
            "[SOAK] 纯空闲复现: DEVICE_INFO 后立即 idle {}s (不驱动功能)",
            soak_idle_s
        );
        let idle_start = std::time::Instant::now();
        let mut last_report = std::time::Instant::now();
        loop {
            ctrl.poll();
            if ctrl.state() == ConnState::Disconnected {
                println!(
                    "[SOAK] REPRO! 纯空闲断开 @ +{}ms: status='{}' err={:?}",
                    idle_start.elapsed().as_millis(),
                    ctrl.status_line(),
                    ctrl.last_error()
                );
                std::process::exit(1);
            }
            if last_report.elapsed() >= Duration::from_secs(1) {
                last_report = std::time::Instant::now();
                println!("[SOAK]   pure-idle +{}s ok", idle_start.elapsed().as_secs());
            }
            if idle_start.elapsed() >= Duration::from_secs(soak_idle_s) {
                break;
            }
            thread::sleep(Duration::from_millis(16));
        }
        println!("[SOAK] 纯空闲 {}s 无断开", soak_idle_s);
        std::process::exit(0);
    }

    if psoc_rescue {
        println!("[RESCUE] 发送 PSOC_RESCUE(SWD 全片重刷 + 校验 + 复位 + 重新下发)…");
        if let Err(error) = ctrl.psoc_rescue() {
            println!("[RESCUE] FAIL psoc_rescue: {}", error);
            std::process::exit(1);
        }
        // 终态由设备推送流给出(state=2); 全片擦写+校验+重新应用耗时数秒, 留 90s 预算。
        let deadline = std::time::Instant::now() + Duration::from_secs(90);
        let mut last_version = ctrl.rescue_progress_version();
        while std::time::Instant::now() < deadline {
            ctrl.poll();
            if ctrl.rescue_progress_version() != last_version {
                last_version = ctrl.rescue_progress_version();
                let p = ctrl.rescue_progress();
                println!(
                    "[RESCUE] state={} phase={} result={} stage={} fail_stage={}",
                    p.state, p.phase, p.result, p.stage, p.fail_stage
                );
            }
            if ctrl.rescue_progress().state == 2 {
                break;
            }
            thread::sleep(Duration::from_millis(50));
        }
        let p = ctrl.rescue_progress();
        println!(
            "[RESCUE] {} (state={} result={} fail_stage={})",
            if p.state == 2 && p.result == 1 {
                "PASS"
            } else {
                "FAIL"
            },
            p.state,
            p.result,
            p.fail_stage
        );
        std::process::exit(if p.state == 2 && p.result == 1 { 0 } else { 1 });
    }

    if nv_soak {
        run_nv_soak(&mut ctrl);
    }

    if soak {
        run_soak(
            &mut ctrl,
            soak_idle_s,
            soak_rate_hz.unwrap_or(100),
            soak_fields.unwrap_or(0x1F),
            soak_seconds,
        );
        // run_soak 内部在失败时已 exit(1)；到此即全通过。
        println!("[SELFTEST] ================ SOAK PASS ================");
        std::process::exit(0);
    }

    // Step 4: 请求配置全部 & 统计条目数
    println!("[SELFTEST] 请求全部配置...");
    if let Err(e) = ctrl.request_config_all() {
        println!("[SELFTEST] FAIL request_config_all 失败: {}", e);
        std::process::exit(1);
    }

    let start = std::time::Instant::now();
    let timeout = Duration::from_millis(CFG_GET_ALL_TIMEOUT_MS);
    let mut config_count = 0;
    loop {
        ctrl.poll();
        let current_config_version = ctrl.config_version();
        // 简单判断:version > 0 说明收到过响应
        if current_config_version > 0 {
            config_count = ctrl.config_entries().len();
            if config_count >= 40 {
                println!("[SELFTEST] 成功获取 {} 条配置项(期望 ≥40)", config_count);
                break;
            }
        }
        if start.elapsed() > timeout {
            println!(
                "[SELFTEST] FAIL 未在 {}ms 内收到足够配置项(当前 {} 项)",
                CFG_GET_ALL_TIMEOUT_MS, config_count
            );
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(50));
    }

    // 打印部分配置示例
    for entry in ctrl.config_entries().iter().take(3) {
        println!("  - {}: {:?}", entry.key, entry.value);
    }

    // Step 5: 启动遥测,收 TELEM_DATA
    println!("[SELFTEST] 启动遥测(RAW|BASELINE|DIFF|STATUS|STATS)...");
    let fields = 0x1F; // RAW|BASELINE|DIFF|STATUS|STATS
    if let Err(e) = ctrl.start_telemetry(200, fields, u64::MAX) {
        println!("[SELFTEST] FAIL start_telemetry 失败: {}", e);
        std::process::exit(1);
    }

    let start = std::time::Instant::now();
    let timeout = Duration::from_millis(TELEM_TIMEOUT_MS);

    // Phase C：等待 DMA/慢路全通道快照真正填充（raw!=0），而非仅收到首帧。
    // 慢路分块读需若干个 update 才完成首份快照，故须等到真实数据到达再判定。
    let mut got_real = false;
    loop {
        ctrl.poll();
        if let Some(latest) = ctrl.telem_latest(0) {
            if latest.raw.unwrap_or(0) != 0 || latest.bsln.unwrap_or(0) != 0 {
                got_real = true;
                break;
            }
        }
        if start.elapsed() > timeout {
            break;
        }
        thread::sleep(Duration::from_millis(20));
    }
    let frames = ctrl.telem_version() as usize;
    if frames == 0 {
        println!("[SELFTEST] FAIL 未在 {}ms 内收到遥测数据", TELEM_TIMEOUT_MS);
        std::process::exit(1);
    }
    println!("[SELFTEST] 成功收到 {} 个遥测帧", frames);
    if let Some(latest) = ctrl.telem_latest(0) {
        println!(
            "  - CH0 最新: raw={:?} bsln={:?} diff={:?}",
            latest.raw, latest.bsln, latest.diff
        );
    }
    if !got_real {
        println!("[SELFTEST] FAIL 全通道 raw 慢路无真实数据(raw/bsln 恒为0) — snapshot 未填充");
        std::process::exit(1);
    }
    println!("[SELFTEST] 全通道 raw 慢路 OK: CH0 收到真实 CSD 计数");

    // Step 6: 停止遥测
    println!("[SELFTEST] 停止遥测...");
    if let Err(e) = ctrl.stop_telemetry() {
        println!("[SELFTEST] FAIL stop_telemetry 失败: {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(100));

    // Step 7: 请求参数全部(通道0)
    println!("[SELFTEST] 请求通道0全部参数...");
    if let Err(e) = ctrl.request_params(0) {
        println!("[SELFTEST] FAIL request_params 失败: {}", e);
        std::process::exit(1);
    }

    let start = std::time::Instant::now();
    let timeout = Duration::from_millis(PARAM_GET_ALL_TIMEOUT_MS);
    let mut param_count = 0;

    loop {
        ctrl.poll();
        let current_param_version = ctrl.param_version();
        if current_param_version > 0 {
            param_count = ctrl.params_of(0).len();
            if param_count > 0 {
                println!("[SELFTEST] 成功获取通道0 {} 个参数", param_count);
                for (pid, val) in ctrl.params_of(0).iter().take(3) {
                    println!("  - param_id=0x{:02X} value={}", pid, val);
                }
                break;
            }
        }
        if start.elapsed() > timeout {
            println!(
                "[SELFTEST] FAIL 未在 {}ms 内收到参数数据(当前 {} 项)",
                PARAM_GET_ALL_TIMEOUT_MS, param_count
            );
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(50));
    }

    // Step 7b: SET_PARAM round-trip 验证(证明写入真正落地 PSoC widgetContext,非仅回显)
    // 用 onDebounce(0x05)：标准完整处理链不会重写该字段，可干净验证写入机制。
    const TEST_PARAM: u8 = 0x05; // ON_DEBOUNCE
    let orig_val = match ctrl.param(0, TEST_PARAM) {
        Some(v) => v,
        None => {
            println!("[SELFTEST] FAIL round-trip: 未找到 ON_DEBOUNCE");
            std::process::exit(1);
        }
    };
    let test_val = if orig_val == 5 { 7 } else { 5 };
    println!(
        "[SELFTEST] SET_PARAM round-trip: ON_DEBOUNCE {} -> {}",
        orig_val, test_val
    );
    if let Err(e) = ctrl.set_param(0, TEST_PARAM, test_val) {
        println!("[SELFTEST] FAIL set_param: {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(50));
    let base_ver = ctrl.param_version();
    if let Err(e) = ctrl.request_params(0) {
        println!("[SELFTEST] FAIL request_params(回读): {}", e);
        std::process::exit(1);
    }
    let start = std::time::Instant::now();
    let mut readback: Option<u32> = None;
    loop {
        ctrl.poll();
        if ctrl.param_version() > base_ver {
            readback = ctrl.param(0, TEST_PARAM);
            break;
        }
        if start.elapsed() > timeout {
            break;
        }
        thread::sleep(Duration::from_millis(50));
    }
    match readback {
        Some(v) if v == test_val => println!("[SELFTEST] round-trip OK: 回读={}", v),
        other => {
            println!(
                "[SELFTEST] FAIL round-trip: 期望 {} 实得 {:?}",
                test_val, other
            );
            let _ = ctrl.set_param(0, TEST_PARAM, orig_val);
            std::process::exit(1);
        }
    }
    // 恢复原值(PSoC 无状态,重启即恢复;此处仍主动还原保持一致)
    let _ = ctrl.set_param(0, TEST_PARAM, orig_val);
    thread::sleep(Duration::from_millis(50));

    // Step 7c: 半自动手动模式 + 阈值持久验证
    // 证明 SET_MODE 生效：半自动手动模式跳过阈值处理，FINGER_TH 跨多个标准处理周期保持手动值。
    const TH_PARAM: u8 = 0x01; // FINGER_TH（自动校准模式运行标准完整处理）
    println!("[SELFTEST] 切换半自动手动模式(SET_MODE=1)...");
    if let Err(e) = ctrl.set_mode(1) {
        println!("[SELFTEST] FAIL set_mode(semi): {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(200));
    let th_test: u32 = 199; // 明显区别于自动整定值(≈44)
    if let Err(e) = ctrl.set_param(0, TH_PARAM, th_test) {
        println!("[SELFTEST] FAIL set_param(FINGER_TH): {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(200)); // 跨多个处理周期，验证手动阈值保持不变
    let base_ver = ctrl.param_version();
    if let Err(e) = ctrl.request_params(0) {
        println!("[SELFTEST] FAIL request_params(semi回读): {}", e);
        std::process::exit(1);
    }
    let start = std::time::Instant::now();
    let mut th_read: Option<u32> = None;
    loop {
        ctrl.poll();
        if ctrl.param_version() > base_ver {
            th_read = ctrl.param(0, TH_PARAM);
            break;
        }
        if start.elapsed() > timeout {
            break;
        }
        thread::sleep(Duration::from_millis(50));
    }
    match th_read {
        Some(v) if v == th_test => {
            println!("[SELFTEST] semi-mode 阈值持久 OK: FINGER_TH 回读={}", v)
        }
        other => {
            println!(
                "[SELFTEST] FAIL semi-mode 阈值未持久: 期望 {} 实得 {:?} (半自动手动处理未保持阈值?)",
                th_test, other
            );
            let _ = ctrl.set_mode(0);
            std::process::exit(1);
        }
    }
    // Step 7d: 半自动手动模式硬件参数 APPLY 重初始化验证
    // 证明模式修改重初始化生效：改 SNS_CLK_DIV + CALIBRATE(APPLY) 后，
    // 硬件参数持久且手动 FINGER_TH(199) 不被重初始化覆盖。
    const CLK_PARAM: u8 = 0x08; // SNS_CLK_DIV
    let clk_orig = ctrl.param(0, CLK_PARAM).unwrap_or(16);
    let clk_test = if clk_orig >= 8 && clk_orig < 250 {
        clk_orig + 2
    } else {
        16
    };
    println!(
        "[SELFTEST] semi-mode 硬件参数 APPLY: SNS_CLK_DIV {} -> {}",
        clk_orig, clk_test
    );
    if let Err(e) = ctrl.set_param(0, CLK_PARAM, clk_test) {
        println!("[SELFTEST] FAIL set_param(SNS_CLK): {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(50));
    if let Err(e) = ctrl.calibrate(0xFFFF_FFFF_FFFF_FFFF) {
        println!("[SELFTEST] FAIL calibrate(APPLY): {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(400)); // 等主循环重初始化 + 重置基线
    let base_ver = ctrl.param_version();
    if let Err(e) = ctrl.request_params(0) {
        println!("[SELFTEST] FAIL request_params(APPLY回读): {}", e);
        std::process::exit(1);
    }
    let start = std::time::Instant::now();
    loop {
        ctrl.poll();
        if ctrl.param_version() > base_ver {
            break;
        }
        if start.elapsed() > timeout {
            break;
        }
        thread::sleep(Duration::from_millis(50));
    }
    let clk_after = ctrl.param(0, CLK_PARAM);
    let th_after = ctrl.param(0, TH_PARAM);
    match (clk_after, th_after) {
        (Some(c), Some(t)) if c == clk_test && t == th_test => {
            println!(
                "[SELFTEST] APPLY 重初始化 OK: SNS_CLK={} 保持, FINGER_TH={} 未被覆盖",
                c, t
            )
        }
        (c, t) => {
            println!(
                "[SELFTEST] FAIL APPLY 重初始化: SNS_CLK 期望 {} 实得 {:?}, FINGER_TH 期望 {} 实得 {:?}",
                clk_test, c, th_test, t
            );
            let _ = ctrl.set_mode(0);
            std::process::exit(1);
        }
    }

    // 切回自动校准/标准完整处理，并在可选重启前验收一次完整的实机 Cp 测量。
    if let Err(e) = ctrl.set_mode(0) {
        println!("[SELFTEST] FAIL set_mode(auto): {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(50));

    println!("[SELFTEST] CP 实机验收: 启动全通道测量...");
    let cp_acceptance_start = std::time::Instant::now();
    let ch0_version_before_measure = ctrl.cp_channel_version(0);
    if let Err(e) = ctrl.measure_cp() {
        println!("[SELFTEST] FAIL measure_cp: {}", e);
        std::process::exit(1);
    }

    let mut ch0_version = ch0_version_before_measure;
    let mut next_ch0_request = std::time::Instant::now();
    let ch0_measurement = loop {
        if std::time::Instant::now() >= next_ch0_request {
            if let Err(e) = ctrl.request_cp(0) {
                println!("[SELFTEST] FAIL request_cp(ch0): {}", e);
                std::process::exit(1);
            }
            next_ch0_request =
                std::time::Instant::now() + Duration::from_millis(CP_REQUEST_INTERVAL_MS);
        }

        cp_poll_or_fail(&mut ctrl, "等待 ch0 测量完成");
        let latest_version = ctrl.cp_channel_version(0);
        if latest_version > ch0_version {
            ch0_version = latest_version;
            match ctrl.cp(0) {
                Some(CP_FAILURE_VALUE) => break CP_FAILURE_VALUE,
                Some(0) => {}
                Some(value) => break value,
                None => {
                    println!("[SELFTEST] FAIL CP ch0 收到新版本但无测量值");
                    std::process::exit(1);
                }
            }
        }
        if cp_acceptance_start.elapsed() >= Duration::from_millis(CP_MEASURE_TIMEOUT_MS) {
            println!(
                "[SELFTEST] FAIL CP ch0 测量未在 {}ms 内完成",
                CP_MEASURE_TIMEOUT_MS
            );
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(16));
    };
    if ch0_measurement == CP_FAILURE_VALUE {
        println!("[SELFTEST] CP ch0 测量返回失败标记 0x{CP_FAILURE_VALUE:08X}; 继续收集全部通道");
    } else {
        println!(
            "[SELFTEST] CP ch0 测量完成: {}fF (+{}ms)",
            ch0_measurement,
            cp_acceptance_start.elapsed().as_millis()
        );
    }

    let mut cp_values = Vec::with_capacity(CP_CHANNEL_COUNT as usize);
    let mut cp_results = Vec::with_capacity(CP_CHANNEL_COUNT as usize);
    let mut cp_failed_channels = Vec::new();
    for ch in 0..CP_CHANNEL_COUNT {
        let version_before_request = ctrl.cp_channel_version(ch);
        if let Err(e) = ctrl.request_cp(ch) {
            println!("[SELFTEST] FAIL request_cp(ch{}): {}", ch, e);
            std::process::exit(1);
        }

        let request_start = std::time::Instant::now();
        loop {
            cp_poll_or_fail(&mut ctrl, &format!("等待 CP ch{}", ch));
            if ctrl.cp_channel_version(ch) > version_before_request {
                match ctrl.cp(ch) {
                    Some(CP_FAILURE_VALUE) | Some(0) => {
                        let value = ctrl.cp(ch).expect("CP value present");
                        cp_failed_channels.push(ch);
                        cp_results.push((ch, value));
                        break;
                    }
                    Some(value) => {
                        cp_values.push(value);
                        cp_results.push((ch, value));
                        break;
                    }
                    None => {
                        println!("[SELFTEST] FAIL CP ch{} 收到新版本但无测量值", ch);
                        std::process::exit(1);
                    }
                }
            }
            if request_start.elapsed() >= Duration::from_millis(CP_GET_TIMEOUT_MS) {
                println!(
                    "[SELFTEST] FAIL CP ch{} 未在 {}ms 内收到新响应",
                    ch, CP_GET_TIMEOUT_MS
                );
                std::process::exit(1);
            }
            thread::sleep(Duration::from_millis(16));
        }
    }

    let cp_distribution = cp_results
        .iter()
        .map(|(ch, value)| format!("ch{}={}fF", ch, value))
        .collect::<Vec<_>>()
        .join(", ");
    println!("[SELFTEST] CP values: [{}]", cp_distribution);
    if !cp_failed_channels.is_empty() {
        println!("[SELFTEST] FAIL CP channels={:?}", cp_failed_channels);
        std::process::exit(1);
    }

    let cp_min = cp_values.iter().copied().min().expect("36 Cp values");
    let cp_max = cp_values.iter().copied().max().expect("36 Cp values");
    println!(
        "[SELFTEST] CP PASS: 总测量耗时={}ms, 36通道 min={}fF max={}fF",
        cp_acceptance_start.elapsed().as_millis(),
        cp_min,
        cp_max
    );

    // Step 8: 可选进烧录模式
    if reboot_bootloader {
        println!("[SELFTEST] 发送进烧录指令(--reboot-bootloader)...");
        if let Err(e) = ctrl.reboot_bootloader() {
            println!("[SELFTEST] WARN reboot_bootloader 失败: {} (继续)", e);
        } else {
            println!("[SELFTEST] 已发送进烧录指令,设备将断开");
        }
    }

    // Step 9: 输出结果
    println!("[SELFTEST] ========================================");
    println!("[SELFTEST] SELFTEST PASS");
    println!("[SELFTEST] ========================================");
    std::process::exit(0);
}

/// CP 验收的 16ms 轮询：任何连接/协议错误均立即使完整自测失败。
fn cp_poll_or_fail(ctrl: &mut AppController, label: &str) {
    ctrl.poll();
    if ctrl.state() == ConnState::Disconnected {
        println!(
            "[SELFTEST] FAIL CP {}期间连接断开: status='{}' err={:?}",
            label,
            ctrl.status_line(),
            ctrl.last_error()
        );
        std::process::exit(1);
    }
    if let Some(error) = ctrl.last_error() {
        println!("[SELFTEST] FAIL CP {}期间通信错误: {}", label, error);
        std::process::exit(1);
    }
}

/// soak 轮询泵：模拟 UI 16ms 定时器 poll，期间检测断开；断开即打印上下文并 exit(1)。
fn soak_pump(ctrl: &mut AppController, ms: u64, label: &str) {
    let start = std::time::Instant::now();
    loop {
        ctrl.poll();
        if ctrl.state() == ConnState::Disconnected {
            println!(
                "[SOAK] FAIL 连接断开 during '{}' @ +{}ms: status='{}' err={:?}",
                label,
                start.elapsed().as_millis(),
                ctrl.status_line(),
                ctrl.last_error()
            );
            std::process::exit(1);
        }
        if start.elapsed() >= Duration::from_millis(ms) {
            return;
        }
        thread::sleep(Duration::from_millis(16));
    }
}

struct SoakPhaseStart {
    telem_frames: u64,
    telem_bytes: u64,
    io: io::IoStats,
    debug: Option<SoakDebugCounters>,
    // 设备侧 SelfHeal 计数在 DEVICE_INFO 诊断里就是 u16，这里保持同宽度，避免无意义的类型放大。
    self_heal: Option<(u16, u16)>,
}

fn read_self_heal(ctrl: &AppController) -> Option<(u16, u16)> {
    ctrl.device_info()
        .and_then(|info| info.diagnostics.as_ref())
        .map(|diag| (diag.self_heal_total, diag.self_heal_dropped))
}

fn print_soak_summary(
    ctrl: &AppController,
    start: &SoakPhaseStart,
    rate_hz: u16,
    elapsed: Duration,
    disconnects: u32,
    end_debug: Option<SoakDebugCounters>,
    end_self_heal: Option<(u16, u16)>,
) {
    let frames = ctrl.telem_frame_count().saturating_sub(start.telem_frames);
    let wire_bytes = ctrl.telem_wire_bytes().saturating_sub(start.telem_bytes);
    let end_io = ctrl.io_stats();
    let elapsed_secs = elapsed.as_secs_f64();
    let actual_rate = if elapsed_secs > 0.0 {
        frames as f64 / elapsed_secs
    } else {
        0.0
    };
    println!(
        "[SOAK] ===== 压测汇总: elapsed={:.3}s frames={} wire_bytes={} actual_rate={:.2}Hz expected_rate={}Hz delta={:+.2}Hz stall_recoveries={} disconnects={} queue_replaced={} =====",
        elapsed_secs,
        frames,
        wire_bytes,
        actual_rate,
        rate_hz,
        actual_rate - rate_hz as f64,
        end_io
            .stall_recoveries
            .saturating_sub(start.io.stall_recoveries),
        disconnects,
        end_io.queue_dropped.saturating_sub(start.io.queue_dropped),
    );
    match (start.debug, end_debug) {
        (Some(before), Some(after)) => println!(
            "[SOAK] device delta: out_stalled={} rearm_count={} vendor_tx_bytes={} flash_write_count={}",
            after.out_stalled.wrapping_sub(before.out_stalled),
            after.rearm_count.wrapping_sub(before.rearm_count),
            after.vendor_tx_bytes.wrapping_sub(before.vendor_tx_bytes),
            after
                .flash_write_count
                .wrapping_sub(before.flash_write_count),
        ),
        _ => println!("[SOAK] device delta: unavailable (EP0 debug read unavailable)"),
    }
    match (start.self_heal, end_self_heal) {
        (Some(before), Some(after)) => println!(
            "[SOAK] SelfHeal delta: total={} dropped={}",
            after.0.wrapping_sub(before.0),
            after.1.wrapping_sub(before.1)
        ),
        _ => println!("[SOAK] SelfHeal delta: unavailable (DEVICE_INFO diagnostics absent)"),
    }
}

/// 无头 exerciser：逐个驱动 UI 全部功能后长时 idle，复现并回验稳定性。
fn run_soak(
    ctrl: &mut AppController,
    idle_s: u64,
    rate_hz: u16,
    fields: u8,
    telemetry_seconds: u64,
) {
    println!("[SOAK] 开始无头 exerciser (模拟 UI 持久连接 + 16ms 轮询)");

    // A. 拉全部配置
    println!("[SOAK] A: request_config_all");
    let _ = ctrl.request_config_all();
    soak_pump(ctrl, 800, "config_all");
    println!("[SOAK]   配置项 = {}", ctrl.config_entries().len());

    // B. 改一个数值配置(设为当前值，无副作用地走 CFG_SET 路径)
    if let Some(entry) = ctrl.config_entries().into_iter().find(|e| {
        matches!(
            e.value,
            CfgValue::U8(_) | CfgValue::U16(_) | CfgValue::U32(_)
        )
    }) {
        let key = entry.key.clone();
        let val = match entry.value {
            CfgValue::U8(v) => v as f64,
            CfgValue::U16(v) => v as f64,
            CfgValue::U32(v) => v as f64,
            _ => 0.0,
        };
        println!("[SOAK] B: cfg_set {} = {}", key, val);
        let _ = ctrl.set_config_number(&key, val);
        soak_pump(ctrl, 300, "cfg_set");
    }

    // C. 读绑区 + 改一个绑区(设为当前值)
    println!("[SOAK] C: binding get/set");
    let b0 = ctrl.get_binding(0);
    let _ = ctrl.set_binding(0, b0);
    soak_pump(ctrl, 300, "set_binding");

    // D. 遥测(全通道) + 等真实数据 + 全通道抽样
    println!(
        "[SOAK] D: start_telemetry(全通道 {}Hz fields=0x{:02X})",
        rate_hz, fields
    );
    let _ = ctrl.start_telemetry(rate_hz, fields, 0xFFFF_FFFF_FFFF_FFFFu64);
    let start = std::time::Instant::now();
    let mut got = false;
    while start.elapsed() < Duration::from_millis(2500) {
        ctrl.poll();
        if ctrl.state() == ConnState::Disconnected {
            println!(
                "[SOAK] FAIL 断开 during telem 等真实数据: {:?}",
                ctrl.last_error()
            );
            std::process::exit(1);
        }
        if let Some(s) = ctrl.telem_latest(0) {
            if s.raw.unwrap_or(0) != 0 || s.bsln.unwrap_or(0) != 0 {
                got = true;
                break;
            }
        }
        thread::sleep(Duration::from_millis(16));
    }
    let ch_with_data = (0..36u8)
        .filter(|&c| {
            ctrl.telem_latest(c)
                .map(|s| s.raw.unwrap_or(0) != 0)
                .unwrap_or(false)
        })
        .count();
    println!(
        "[SOAK]   telem got_real={} 有数据通道={}  CH0={:?}",
        got,
        ch_with_data,
        ctrl.telem_latest(0).map(|s| (s.raw, s.diff, s.status))
    );
    println!(
        "[SOAK]   采样率={} Hz  通道延迟={} us",
        ctrl.telem_samples_per_sec(),
        ctrl.telem_scan_period_us()
    );
    println!(
        "[SOAK]   延迟组成: SPI={}us RP处理={}us USB={}us 传感器={}us",
        ctrl.telem_lat_spi_us(),
        ctrl.telem_lat_proc_us(),
        ctrl.telem_lat_usb_us(),
        ctrl.telem_scan_period_us()
    );
    if !got {
        println!("[SOAK] FAIL 遥测无真实数据(慢路未填充)");
        std::process::exit(1);
    }

    // D2 单独重新开流并从此处取基线，确保 --soak-seconds 是完整、唯一的计量窗口。
    let _ = ctrl.stop_telemetry();
    soak_pump(ctrl, 100, "telem_prime_stop");
    println!(
        "[SOAK] D2: 持续遥测 {}s + 周期 USB debug",
        telemetry_seconds
    );
    let _ = ctrl.start_telemetry(rate_hz, fields, 0xFFFF_FFFF_FFFF_FFFFu64);
    let phase_start = std::time::Instant::now();
    let phase = SoakPhaseStart {
        telem_frames: ctrl.telem_frame_count(),
        telem_bytes: ctrl.telem_wire_bytes(),
        io: ctrl.io_stats(),
        debug: read_soak_debug(ctrl),
        self_heal: read_self_heal(ctrl),
    };
    let mut last_debug_at = phase_start - Duration::from_secs(1);
    let mut last_ping_at = phase_start;
    let mut last_debug = phase.debug;
    let mut last_self_heal = phase.self_heal;
    while phase_start.elapsed() < Duration::from_secs(telemetry_seconds) {
        ctrl.poll();
        if let Some(current) = read_self_heal(ctrl) {
            last_self_heal = Some(current);
        }
        if ctrl.state() == ConnState::Disconnected {
            let elapsed = phase_start.elapsed();
            println!(
                "[SOAK] FAIL 连接断开 during telemetry_run: {:?}",
                ctrl.last_error()
            );
            println!("[SOAK] 压测中断于第 {} 秒", elapsed.as_secs());
            print_soak_summary(
                ctrl,
                &phase,
                rate_hz,
                elapsed,
                1,
                last_debug,
                last_self_heal,
            );
            std::process::exit(1);
        }
        if last_debug_at.elapsed() >= Duration::from_secs(1) {
            last_debug_at = std::time::Instant::now();
            // ★压测期间绝不能发 HELLO★: 固件 HELLO 处理的第一步是 SensorLink::stop()(停遥测流),
            // 每秒刷新一次 DEVICE_INFO 等于每秒把自己要压测的遥测流关掉(实测 frames≈0)。
            // SelfHeal 累计值改为压测结束后单独读一次, 期间只用 EP0 计数(不经 bulk、不影响流)。
            if let Some(debug) = read_soak_debug(ctrl) {
                last_debug = Some(debug);
                println!(
                    "[SOAK] DBG +{}s out_stalled={} rearm={} vendor_tx={} flash_writes={}",
                    phase_start.elapsed().as_secs(),
                    debug.out_stalled,
                    debug.rearm_count,
                    debug.vendor_tx_bytes,
                    debug.flash_write_count
                );
            }
        }
        // ★必须周期续租★: 固件 TxScheduler 的遥测任务是租约制(约 3s 到期自停), 靠下行命令续期。
        // PING 是唯一既能续租又不会停流的轻量命令(HELLO 会 SensorLink::stop())。
        if last_ping_at.elapsed() >= Duration::from_millis(500) {
            last_ping_at = std::time::Instant::now();
            let _ = ctrl.ping();
        }
        thread::sleep(Duration::from_millis(16));
    }
    let elapsed = phase_start.elapsed();
    let end_debug = read_soak_debug(ctrl).or(last_debug);
    // 压测窗口结束后才刷新 DEVICE_INFO: HELLO 会停流, 只能放在计量之外。
    let _ = ctrl.resend_hello();
    soak_pump(ctrl, 200, "self_heal_refresh");
    let end_self_heal = read_self_heal(ctrl).or(last_self_heal);
    print_soak_summary(ctrl, &phase, rate_hz, elapsed, 0, end_debug, end_self_heal);
    let _ = ctrl.stop_telemetry();
    soak_pump(ctrl, 300, "stop_telem");

    // E. 半自动手动 + 捕获 + 调参 + 校准 + 回自动校准/标准完整处理
    println!("[SOAK] E: 半自动手动/捕获/调参/校准/自动校准标准完整处理");
    let _ = ctrl.set_mode(1);
    soak_pump(ctrl, 200, "set_mode_semi");
    let _ = ctrl.csd_capture();
    soak_pump(ctrl, 400, "csd_capture");
    let _ = ctrl.set_param(0, 0x01, 180);
    soak_pump(ctrl, 200, "set_param");
    let _ = ctrl.calibrate(0xFFFF_FFFF_FFFF_FFFFu64);
    soak_pump(ctrl, 600, "calibrate");
    let _ = ctrl.set_mode(0);
    soak_pump(ctrl, 200, "set_mode_auto");

    // G. 长时 idle(复现 UI 空闲 stall)
    println!("[SOAK] G: idle {}s (复现空闲 stall)", idle_s);
    let idle_start = std::time::Instant::now();
    let mut last_report = std::time::Instant::now();
    loop {
        ctrl.poll();
        if ctrl.state() == ConnState::Disconnected {
            println!(
                "[SOAK] FAIL idle 期间断开 @ +{}ms: status='{}' err={:?}",
                idle_start.elapsed().as_millis(),
                ctrl.status_line(),
                ctrl.last_error()
            );
            std::process::exit(1);
        }
        if last_report.elapsed() >= Duration::from_secs(2) {
            last_report = std::time::Instant::now();
            println!("[SOAK]   idle +{}s ok", idle_start.elapsed().as_secs());
        }
        if idle_start.elapsed() >= Duration::from_secs(idle_s) {
            break;
        }
        thread::sleep(Duration::from_millis(16));
    }
    println!("[SOAK] 全部功能驱动完毕 + idle {}s 无断开", idle_s);
}

// ============================================================================
// --nv-soak: 落盘全量漫灌回归
// ----------------------------------------------------------------------------
// 走的是 GUI 用的**同一套** AppController 通路(set_config / set_binding / kbd_set_* /
// kbd_combo_add / set_param → save_config), 因此它验证的就是用户点界面时真正发生的事,
// 而不是另开一条测试专用捷径。
//
// 流程: 全量回读基线 → 逐项写入"可辨识且合法"的新值 → save_config → 等串行队列排空 →
//       重启设备 → 重连 → 全量回读 → 逐项比对 → 打印差异清单。
//
// 安全边界(故意不碰的项, 碰了会让设备当场不可用, 与"落盘是否可靠"无关):
//   - mode.work: 改它会切 Serial/HID 拓扑并重启重枚举
//   - per-channel PARAM 的硬件类 0x07..0x0B(分辨率/分频/IDAC): 写坏会 raw 满量程且需重新校准
//   - 全局 CSD GPARAM: 同上, 且 PSoC 启动钳位会合法地改写它(另有专门的对账通道)
// ============================================================================

/// 按当前值与设备上报 range 生成一个不同的合法值；字符串以短标记值覆盖以验证持久化。
fn soak_next_value(
    entry: &mai2control_ui::proto::ConfigEntry,
) -> Option<mai2control_ui::proto::CfgValue> {
    use mai2control_ui::proto::CfgValue as V;
    match (&entry.value, entry.range.as_ref()) {
        (V::Bool(value), _) => Some(V::Bool(!value)),
        (V::Str(value), _) => {
            let candidate = format!("nv{}", value.len());
            Some(V::Str(if candidate == *value {
                "nvx".to_string()
            } else {
                candidate
            }))
        }
        (V::I8(current), Some((V::I8(lo), V::I8(hi)))) if lo < hi => {
            let candidate = if *current != *lo { *lo } else { *hi };
            Some(V::I8(candidate))
        }
        (V::U8(current), Some((V::U8(lo), V::U8(hi)))) if lo < hi => {
            let candidate = if *current != *lo { *lo } else { *hi };
            Some(V::U8(candidate))
        }
        (V::U16(current), Some((V::U16(lo), V::U16(hi)))) if lo < hi => {
            let candidate = if *current != *lo { *lo } else { *hi };
            Some(V::U16(candidate))
        }
        (V::U32(current), Some((V::U32(lo), V::U32(hi)))) if lo < hi => {
            let candidate = if *current != *lo { *lo } else { *hi };
            Some(V::U32(candidate))
        }
        (V::F32(current), Some((V::F32(lo), V::F32(hi)))) if lo < hi => {
            let candidate = if *current != *lo { *lo } else { *hi };
            Some(V::F32(candidate))
        }
        (V::I8(current), _) => Some(V::I8(*current ^ 1)),
        (V::U8(current), _) => Some(V::U8(*current ^ 1)),
        (V::U16(current), _) => Some(V::U16(*current ^ 1)),
        (V::U32(current), _) => Some(V::U32(*current ^ 1)),
        (V::F32(current), _) => Some(V::F32(*current + 1.0)),
    }
}

fn soak_val_text(v: &mai2control_ui::proto::CfgValue) -> String {
    use mai2control_ui::proto::CfgValue as V;
    match v {
        V::Bool(b) => format!("{}", b),
        V::I8(x) => format!("{}", x),
        V::U8(x) => format!("{}", x),
        V::U16(x) => format!("{}", x),
        V::U32(x) => format!("{}", x),
        V::F32(x) => format!("{}", x),
        V::Str(s) => s.clone(),
    }
}

fn run_nv_soak(ctrl: &mut AppController) -> ! {
    use mai2control_ui::proto::{
        KBD_COMBO_COUNT, KBD_COMBO_KEY_COUNT, KNOWN_PARAM_IDS, KbdComboItem,
    };

    macro_rules! fail {
        ($($arg:tt)*) => {{
            println!("[SOAK] FAIL {}", format!($($arg)*));
            std::process::exit(1);
        }};
    }
    macro_rules! require {
        ($expr:expr, $label:expr) => {{
            if let Err(error) = $expr {
                fail!("{}: {}", $label, error);
            }
        }};
    }

    fn param_value(id: u8, current: u32) -> Option<u32> {
        // 这些候选值来自现有控制器的半自动基线、guard/probe 与 IDAC 夹紧路径；
        // 每项都选与设备刚回读值不同的最小安全候选，不把硬件参数当作任意 u32 写入。
        let values: &[u32] = match id {
            0x01 => &[150, 151], // FINGER_TH
            0x02 => &[60, 61],   // NOISE_TH
            0x03 => &[60, 61],   // NEG_NOISE_TH
            0x04 => &[7, 8],     // HYSTERESIS
            0x05 => &[3, 4],     // ON_DEBOUNCE
            0x06 => &[30, 31],   // LOW_BSLN_RST
            // 0x07/0x08/0x09/0x0B 取 0 在固件里等于"该项无有效值"(CsdConfig::_param_zero_invalid),
            // 下发时被跳过以保留 PSoC 自校准结果 ⇒ 写 0 不可能持久化, 只能用非 0 合法值。
            0x07 => &[12, 16], // RESOLUTION: 12/16 位, 6 会被 PSoC 侧拒绝并回读为 12
            0x08 => &[8, 20, 32], // SNS_CLK_DIV: existing calibration probes
            0x09 => &[64, 80], // IDAC_MOD: 手动模式下的合法非零补偿电流档
            0x0A => &[0, 1],   // SNS_CLK_SOURCE: generated configuration's discrete legal choices
            0x0B => &[4, 6],   // IDAC_GAIN: existing guard path documents [0, 6], 0 不可持久化
            _ => return None,
        };
        values.iter().copied().find(|value| *value != current)
    }

    let pump = |ctrl: &mut AppController, ms: u64| -> bool {
        let deadline = std::time::Instant::now() + Duration::from_millis(ms);
        while std::time::Instant::now() < deadline {
            ctrl.poll();
            ctrl.csd_diag_tick();
            if ctrl.state() == ConnState::Disconnected {
                return false;
            }
            thread::sleep(Duration::from_millis(16));
        }
        true
    };
    let wait_psoc_ready = |ctrl: &mut AppController, phase: &str, timeout: Duration| {
        let deadline = std::time::Instant::now() + timeout;
        let mut last_hello = std::time::Instant::now() - Duration::from_secs(1);
        while std::time::Instant::now() < deadline {
            ctrl.poll();
            ctrl.csd_diag_tick();
            if ctrl.state() == ConnState::Disconnected {
                fail!(
                    "{} PSoC 就绪等待期间设备断开: {:?}",
                    phase,
                    ctrl.last_error()
                );
            }
            if ctrl
                .device_info()
                .is_some_and(|info| info.psoc_link_valid && info.psoc_snapshot_valid)
            {
                return;
            }
            if last_hello.elapsed() >= Duration::from_millis(300) {
                require!(
                    ctrl.resend_hello(),
                    format!("{} HELLO 刷新 PSoC 状态", phase)
                );
                last_hello = std::time::Instant::now();
            }
            thread::sleep(Duration::from_millis(16));
        }
        let (link_valid, snapshot_valid) = ctrl
            .device_info()
            .map(|info| (info.psoc_link_valid, info.psoc_snapshot_valid))
            .unwrap_or((false, false));
        fail!(
            "{} PSoC 就绪超时: link_valid={} snapshot_valid={}",
            phase,
            link_valid,
            snapshot_valid
        );
    };
    let missing_param_channels = |ctrl: &AppController, id: u8| -> Vec<u8> {
        (0..36u8)
            .filter(|channel| ctrl.param(*channel, id).is_none())
            .collect()
    };
    let wait_config = |ctrl: &mut AppController, before: u64, phase: &str, timeout: Duration| {
        let deadline = std::time::Instant::now() + timeout;
        while std::time::Instant::now() < deadline && ctrl.config_version() <= before {
            if !pump(ctrl, 16) {
                fail!(
                    "{} CFG_GET_ALL 回读期间设备断开: {:?}",
                    phase,
                    ctrl.last_error()
                );
            }
        }
        if ctrl.config_version() <= before {
            fail!(
                "{} CFG_GET_ALL 回读超时: cfg_version expected>{} actual={}",
                phase,
                before,
                ctrl.config_version()
            );
        }
    };
    let wait_param = |ctrl: &mut AppController,
                      id: u8,
                      before: u64,
                      phase: &str,
                      timeout: Duration| {
        let deadline = std::time::Instant::now() + timeout;
        let mut last_request = std::time::Instant::now();
        while std::time::Instant::now() < deadline {
            let missing = missing_param_channels(ctrl, id);
            if ctrl.param_version() > before && missing.is_empty() {
                break;
            }
            if !pump(ctrl, 16) {
                fail!(
                    "{} PARAM_GET_ALL(0x{:02X}) 回读期间设备断开: {:?}",
                    phase,
                    id,
                    ctrl.last_error()
                );
            }
            if last_request.elapsed() >= Duration::from_millis(300) {
                require!(
                    ctrl.request_param_all_channels(id),
                    format!("{} PARAM_GET_ALL(0x{:02X}) 重试", phase, id)
                );
                last_request = std::time::Instant::now();
            }
        }
        let missing = missing_param_channels(ctrl, id);
        if ctrl.param_version() <= before || !missing.is_empty() {
            let channels = missing
                .iter()
                .map(|channel| format!("CH{}", channel))
                .collect::<Vec<_>>()
                .join(",");
            fail!(
                "{} PARAM_GET_ALL 回读超时: param_id=0x{:02X} param_version expected>{} actual={} missing_channels=[{}]",
                phase,
                id,
                before,
                ctrl.param_version(),
                channels
            );
        }
    };
    let wait_version = |ctrl: &mut AppController,
                        before: u64,
                        phase: &str,
                        response: &str,
                        version: fn(&AppController) -> u64,
                        timeout: Duration| {
        let deadline = std::time::Instant::now() + timeout;
        while std::time::Instant::now() < deadline && version(ctrl) <= before {
            if !pump(ctrl, 16) {
                fail!(
                    "{} {} 回读期间设备断开: {:?}",
                    phase,
                    response,
                    ctrl.last_error()
                );
            }
        }
        if version(ctrl) <= before {
            fail!(
                "{} {} 回读超时: version expected>{} actual={}",
                phase,
                response,
                before,
                version(ctrl)
            );
        }
    };
    let request_all_serial = |ctrl: &mut AppController, phase: &str, timeout: Duration| {
        let before = ctrl.config_version();
        require!(
            ctrl.request_config_all(),
            format!("{} CFG_GET_ALL 请求", phase)
        );
        wait_config(ctrl, before, phase, timeout);

        for &id in KNOWN_PARAM_IDS {
            let before = ctrl.param_version();
            require!(
                ctrl.request_param_all_channels(id),
                format!("{} PARAM_GET_ALL(0x{:02X}) 请求", phase, id)
            );
            wait_param(ctrl, id, before, phase, timeout);
        }

        let before = ctrl.kbd_map_version();
        require!(
            ctrl.kbd_request_map(),
            format!("{} KBD_GET_MAP 请求", phase)
        );
        wait_version(
            ctrl,
            before,
            phase,
            "KBD_GET_MAP",
            AppController::kbd_map_version,
            timeout,
        );
        let before = ctrl.kbd_touchmap_version();
        require!(
            ctrl.kbd_request_touchmap(),
            format!("{} KBD_GET_TOUCHMAP 请求", phase)
        );
        wait_version(
            ctrl,
            before,
            phase,
            "KBD_GET_TOUCHMAP",
            AppController::kbd_touchmap_version,
            timeout,
        );
        let before = ctrl.kbd_hold_version();
        require!(
            ctrl.kbd_request_hold(),
            format!("{} KBD_GET_HOLD 请求", phase)
        );
        wait_version(
            ctrl,
            before,
            phase,
            "KBD_GET_HOLD",
            AppController::kbd_hold_version,
            timeout,
        );
        let before = ctrl.kbd_combo_version();
        require!(
            ctrl.kbd_request_combo(),
            format!("{} KBD_GET_COMBO 请求", phase)
        );
        wait_version(
            ctrl,
            before,
            phase,
            "KBD_GET_COMBO",
            AppController::kbd_combo_version,
            timeout,
        );
    };

    println!("[SOAK] 等待基线 PSoC 链路与快照就绪…");
    wait_psoc_ready(ctrl, "基线", Duration::from_secs(20));
    // ★漫灌 PARAM 前必须处于半自动手动(SEMI)★: AUTO 模式下 CapSense 自己算阈值/IDAC, 固件不下发
    // store 里的手动值, PARAM_GET_ALL 读回的永远是 PSoC 自算结果 —— 于是"写入 → 重启回读"必然全项
    // 不一致(实测 RESET_DEFAULTS 回到 AUTO 后 353/576 项对不上, 实际值全是出厂默认)。
    // 用 debug_mode_now 直接下发, 不进草稿, 使随后的回读就是设备真值。
    println!("[SOAK] 切换 CSD 半自动手动模式(SEMI)…");
    require!(ctrl.debug_mode_now(1), "set_mode SEMI");
    if !pump(ctrl, 1500) {
        fail!("切 SEMI 期间设备断开: {:?}", ctrl.last_error());
    }

    println!("[SOAK] 全量回读基线(串行 CFG → 11×PARAM → 键盘)…");
    request_all_serial(ctrl, "基线", Duration::from_secs(12));

    // 这些 CFG 键与 BIND_SET_MAP / KBD_SET_MAP / TOUCHMAP / HOLD / COMBO 写的是同一份底层数据,
    // 后面的专用命令会覆盖它们 → 在 CFG 层再断言就会用专用命令写入的值判 CFG 期望失败(实测
    // kbd.hd00 期望 101 实际 300)。这些项的持久化由 BIND_GET_MAP / KBD_GET_* 回读单独校验,
    // 此处只去掉 CFG 侧的重复断言, 不减少覆盖面。
    const CFG_SKIP_PREFIXES: &[&str] = &[
        "bind.map", "kbd.key", "kbd.km", "kbd.zone", "kbd.zm", "kbd.hd", "kbd.mh", "kbd.zhd",
        "kbd.zmh", "kbd.cb",
    ];
    let mut want_cfg = Vec::new();
    for entry in ctrl.config_entries() {
        if CFG_SKIP_PREFIXES
            .iter()
            .any(|prefix| entry.key.starts_with(prefix))
        {
            continue;
        }
        let Some(value) = soak_next_value(&entry) else {
            fail!("CFG {} 无法按现有类型/range 生成不同合法值", entry.key);
        };
        want_cfg.push((entry.key, value));
    }
    if want_cfg.is_empty() {
        fail!("没有可写 CFG 项");
    }
    for (key, value) in &want_cfg {
        require!(
            ctrl.set_config(mai2control_ui::proto::ConfigEntry::new(
                key.clone(),
                value.clone()
            )),
            format!("set_config {}", key)
        );
    }

    let mut want_bind = [0u8; 34];
    for zone in 0..34usize {
        let current = ctrl.binding_channel_of(zone);
        let value = if current < 36 {
            (current + 1) % 36
        } else {
            zone as u8 % 36
        };
        want_bind[zone] = value;
        require!(
            ctrl.set_binding_channel(zone, value),
            format!("set_binding_channel z{}", zone)
        );
    }

    let mut want_phys = [(0u8, 0u8); 12];
    for index in 0..12u8 {
        let candidate = 0x04 + index;
        let code = if ctrl.kbd_map(index) == candidate {
            0x24 + index
        } else {
            candidate
        };
        let modifier = if ctrl.kbd_keymod(index) == index % 8 {
            (index + 1) % 8
        } else {
            index % 8
        };
        want_phys[index as usize] = (code, modifier);
        require!(
            ctrl.kbd_set_map(index, code, modifier),
            format!("kbd_set_map {}", index)
        );
    }
    let mut want_touch = [(0u8, 0u8); 34];
    for zone in 0..34u8 {
        let candidate = 0x04 + zone % 20;
        let code = if ctrl.kbd_touch_keycode(zone) == candidate {
            0x24 + zone % 10
        } else {
            candidate
        };
        let modifier = if ctrl.kbd_zone_mod(zone) == zone % 8 {
            (zone + 1) % 8
        } else {
            zone % 8
        };
        want_touch[zone as usize] = (code, modifier);
        require!(
            ctrl.kbd_set_touchmap(zone, code, modifier),
            format!("kbd_set_touchmap {}", zone)
        );
    }
    let mut want_hold_phys = [(0u16, 0u16); 12];
    for index in 0..12u8 {
        let current = ctrl.kbd_hold_phys(index);
        let value = if current == (100 + index as u16, 200 + index as u16) {
            (300 + index as u16, 400 + index as u16)
        } else {
            (100 + index as u16, 200 + index as u16)
        };
        want_hold_phys[index as usize] = value;
        require!(
            ctrl.stage_kbd_hold(0, index, value.0, value.1),
            format!("stage_kbd_hold phys {}", index)
        );
    }
    let mut want_hold_zone = [(0u16, 0u16); 34];
    for zone in 0..34u8 {
        let current = ctrl.kbd_hold_zone(zone);
        let value = if current == (500 + zone as u16, 600 + zone as u16) {
            (700 + zone as u16, 800 + zone as u16)
        } else {
            (500 + zone as u16, 600 + zone as u16)
        };
        want_hold_zone[zone as usize] = value;
        require!(
            ctrl.stage_kbd_hold(1, zone, value.0, value.1),
            format!("stage_kbd_hold zone {}", zone)
        );
    }

    while !ctrl.kbd_combos().is_empty() {
        ctrl.kbd_combo_remove(0);
    }
    let mut want_combos = Vec::new();
    for index in 0..KBD_COMBO_COUNT {
        let mut keycodes = [0u8; KBD_COMBO_KEY_COUNT];
        keycodes[0] = 0x1E + index as u8;
        let item = KbdComboItem {
            zone_mask: 0b11u64 << (index * 2),
            keycodes,
            modifiers: index as u8 % 8,
            delay_ms: 900 + index as u16,
            max_hold_ms: 1000 + index as u16,
        };
        if !ctrl.kbd_combo_add(
            item.zone_mask,
            item.keycodes,
            item.modifiers,
            item.delay_ms,
            item.max_hold_ms,
        ) {
            fail!("kbd_combo_add {} 被拒绝", index);
        }
        want_combos.push(item);
    }

    let mut want_param = Vec::with_capacity(36 * KNOWN_PARAM_IDS.len());
    for channel in 0..36u8 {
        for &id in KNOWN_PARAM_IDS {
            let Some(current) = ctrl.param(channel, id) else {
                fail!("PARAM CH{} 0x{:02X} 基线缺失", channel, id);
            };
            let Some(value) = param_value(id, current) else {
                fail!("PARAM CH{} 0x{:02X} 无安全不同值", channel, id);
            };
            want_param.push((channel, id, value));
            require!(
                ctrl.set_param(channel, id, value),
                format!("set_param CH{} 0x{:02X}", channel, id)
            );
        }
    }

    let prefix = "#include <stddef.h>\n#include \"psoc_algo_abi.h\"\nvoid algo(algo_io_t* io){ io->out_active=(io->base_active!=0u)?1u:0u; }\nstatic const char nv_soak_pad[] = \"";
    let suffix = "\";\n";
    let capacity = AppController::algo_src_capacity();
    let fill = capacity
        .checked_sub(prefix.len() + suffix.len())
        .unwrap_or_else(|| fail!("C 源模板超过 {} 字节", capacity));
    let source = format!("{}{}{}", prefix, "x".repeat(fill), suffix);
    if AppController::algo_src_used(&source) != capacity {
        fail!(
            "C 源有效长度 {} 不等于 {}",
            AppController::algo_src_used(&source),
            capacity
        );
    }
    let expected_source = AppController::strip_c_comments(&source);
    require!(ctrl.send_algo_src(&source), "send_algo_src 32768B");

    println!("[SOAK] 写入完毕，提交保存…");
    require!(ctrl.save_config(), "save_config");
    let deadline = std::time::Instant::now() + Duration::from_secs(90);
    while std::time::Instant::now() < deadline
        && (ctrl.cfg_tx_pending() != 0 || ctrl.algo_src_transfer_pending())
    {
        if !pump(ctrl, 16) {
            fail!("保存队列/源传输期间设备断开: {:?}", ctrl.last_error());
        }
    }
    if ctrl.cfg_tx_pending() != 0 || ctrl.algo_src_transfer_pending() {
        fail!(
            "保存队列或算法 C 源传输超时: cfg_pending={} src_pending={}",
            ctrl.cfg_tx_pending(),
            ctrl.algo_src_transfer_pending()
        );
    }
    // ★等落盘真正静止, 不用固定时长★: 固件每轮主循环最多落一个脏区(KV/CSD/ALGO_BIN/ALGO_SRC),
    // 且 core1 忙(PSoC 重初始化/校准)时整轮跳过落盘。固定等 8s 会在排在最后、占 9 个扇区的
    // ALGO_SRC 尚未写入时就 REBOOT → 32KB C 源整份丢失(实测回读 0 字节)。以设备侧
    // flash_write_count 停止增长作为"全部脏区已写完"的真实信号。
    // ★以设备上报的 NvStore 脏位判定"真的落盘了"★: 固件每轮主循环最多落一个脏区
    // (KV/CSD/ALGO_BIN/ALGO_SRC), 且命令未静默 / core1 忙时整轮不擦 flash。按固定时长或按
    // flash_write_count 静止都只是猜, 会在排最后、占 9 扇区的 ALGO_SRC 还没写时就 REBOOT
    // (实测 32KB C 源整份丢失)。dirty_mask==0 才是设备侧"已全部写完"的真值;
    // commit_fail 增长则是"写失败在重试", 必须报错而不是当成写完。
    println!("[SOAK] 等待设备落盘完成(dirty_mask=0)…");
    let flush_deadline = std::time::Instant::now() + Duration::from_secs(90);
    let mut last = read_soak_debug(ctrl);
    loop {
        if !pump(ctrl, 250) {
            fail!("等待设备落盘期间设备断开: {:?}", ctrl.last_error());
        }
        let Some(now) = read_soak_debug(ctrl) else {
            fail!("落盘等待期间无法读取设备 NvStore 状态(EP0 诊断不可用)");
        };
        if let Some(before) = last {
            if now.nv_commit_fail > before.nv_commit_fail {
                fail!(
                    "设备落盘失败并在重试: nv_commit_fail {} → {} dirty_mask=0x{:X}",
                    before.nv_commit_fail,
                    now.nv_commit_fail,
                    now.nv_dirty_mask
                );
            }
        }
        last = Some(now);
        if now.nv_dirty_mask == 0 {
            println!(
                "[SOAK] 落盘完成 dirty_mask=0 commit_ok={} commit_fail={} algo_src_len={}",
                now.nv_commit_ok, now.nv_commit_fail, now.nv_algo_src_len
            );
            break;
        }
        if std::time::Instant::now() >= flush_deadline {
            fail!(
                "落盘未在 90 秒内完成: dirty_mask=0x{:X} commit_ok={} commit_fail={}",
                now.nv_dirty_mask,
                now.nv_commit_ok,
                now.nv_commit_fail
            );
        }
    }

    // ★PARAM 的重启期望取"写入并落盘后的设备真值"★: 0x07(RESOLUTION)/0x09(IDAC_MOD) 等量的最终
    // 生效值由 PSoC 侧 clamp 与自动校准决定(实测写 16 生效 12, 写 64 被重算为 120)。拿写入值断言
    // 等于在验证硬件语义, 不是验证持久化。仍然写满并逐项校验全部 36×11 项, 只是期望换成设备真值。
    println!("[SOAK] 落盘后回读设备真值作为 PARAM 重启期望…");
    request_all_serial(ctrl, "落盘后", Duration::from_secs(20));
    // 重启前先看一眼组合表与它的 KV 承载键: 若此处已是 0 条/0 值, 问题在"写入未生效";
    // 若此处正常而重启后为 0, 问题在"落盘/加载"。省掉为定位再跑一整轮漫灌。
    {
        let staged = ctrl.kbd_combos().len();
        let raw: Vec<String> = ["A", "B", "K", "T"]
            .iter()
            .map(|part| {
                let key = format!("kbd.cb{}00", part);
                match ctrl.config_get(&key) {
                    Some(entry) => format!("{}={}", key, soak_val_text(&entry.value)),
                    None => format!("{}=缺失", key),
                }
            })
            .collect();
        println!("[SOAK] 落盘后 combo={} 条, {}", staged, raw.join(" "));
    }
    let mut param_clamped = 0usize;
    for (channel, id, value) in want_param.iter_mut() {
        let Some(actual) = ctrl.param(*channel, *id) else {
            fail!("落盘后 PARAM CH{} 0x{:02X} 回读缺失", channel, id);
        };
        if actual != *value {
            param_clamped += 1;
            *value = actual;
        }
    }
    if param_clamped != 0 {
        println!(
            "[SOAK] {} 项 PARAM 被 PSoC clamp/自校准改写, 已改用设备真值作为重启期望",
            param_clamped
        );
    }

    // 两次重启共用同一枚举、重连和 PSoC 就绪路径；二次验证不会再次写入任何配置。
    macro_rules! reboot_and_reconnect {
        ($phase:expr) => {{
            println!("[SOAK] {}：重启设备并等待重新枚举…", $phase);
            require!(ctrl.reboot(), format!("{} reboot", $phase));
            let reconnect_deadline = std::time::Instant::now() + Duration::from_secs(45);
            let mut saw_disconnect = false;
            let mut last_refresh = std::time::Instant::now() - Duration::from_secs(1);
            let mut last_hello = std::time::Instant::now();
            while std::time::Instant::now() < reconnect_deadline {
                ctrl.poll();
                ctrl.csd_diag_tick();
                if ctrl.state() == ConnState::Disconnected {
                    saw_disconnect = true;
                    if last_refresh.elapsed() >= Duration::from_millis(300) {
                        last_refresh = std::time::Instant::now();
                        ctrl.refresh_devices();
                        let _ = ctrl.connect(0);
                        last_hello = std::time::Instant::now();
                    }
                } else if saw_disconnect && ctrl.state() == ConnState::Connected {
                    break;
                } else if ctrl.state() == ConnState::Connecting
                    && last_hello.elapsed() >= Duration::from_millis(300)
                {
                    let _ = ctrl.resend_hello();
                    last_hello = std::time::Instant::now();
                }
                thread::sleep(Duration::from_millis(50));
            }
            if !saw_disconnect || ctrl.state() != ConnState::Connected {
                fail!(
                    "{} 未在 45 秒内重新枚举并连接 (seen_disconnect={} state={:?})",
                    $phase,
                    saw_disconnect,
                    ctrl.state()
                );
            }
            println!("[SOAK] 等待{} PSoC 链路与快照就绪…", $phase);
            wait_psoc_ready(ctrl, $phase, Duration::from_secs(20));
        }};
    }

    reboot_and_reconnect!("重启后");

    println!("[SOAK] 重启后全量回读(串行 CFG → 11×PARAM → 键盘)…");
    request_all_serial(ctrl, "重启后", Duration::from_secs(20));
    let post_source = ctrl.algo_device_src_version();
    require!(ctrl.request_algo_src(), "重启后 ALGO_GET_SRC 请求");
    let deadline = std::time::Instant::now() + Duration::from_secs(20);
    while std::time::Instant::now() < deadline
        && (ctrl.algo_device_src_version() <= post_source || ctrl.algo_src_transfer_pending())
    {
        if !pump(ctrl, 16) {
            fail!(
                "重启后 ALGO_GET_SRC 回读期间设备断开: {:?}",
                ctrl.last_error()
            );
        }
    }
    if ctrl.algo_device_src_version() <= post_source || ctrl.algo_src_transfer_pending() {
        fail!(
            "重启后 ALGO_GET_SRC 回读超时: source_version expected>{} actual={} transfer_pending={}",
            post_source,
            ctrl.algo_device_src_version(),
            ctrl.algo_src_transfer_pending()
        );
    }

    let mut bad = Vec::new();
    for (key, want) in &want_cfg {
        match ctrl.config_get(key) {
            Some(got) if soak_val_text(&got.value) == soak_val_text(want) => {}
            Some(got) => bad.push(format!(
                "CFG {}: 期望 {} 实际 {}",
                key,
                soak_val_text(want),
                soak_val_text(&got.value)
            )),
            None => bad.push(format!("CFG {}: 回读缺失", key)),
        }
    }
    for zone in 0..34usize {
        let got = ctrl.binding_channel_of(zone);
        if got != want_bind[zone] {
            bad.push(format!(
                "binding zone{}: 期望 CH{} 实际 CH{}",
                zone, want_bind[zone], got
            ));
        }
    }
    for index in 0..12u8 {
        let got = (ctrl.kbd_map(index), ctrl.kbd_keymod(index));
        if got != want_phys[index as usize] {
            bad.push(format!(
                "kbd phys{}: 期望 {:?} 实际 {:?}",
                index, want_phys[index as usize], got
            ));
        }
    }
    for zone in 0..34u8 {
        let got = (ctrl.kbd_touch_keycode(zone), ctrl.kbd_zone_mod(zone));
        if got != want_touch[zone as usize] {
            bad.push(format!(
                "kbd touch{}: 期望 {:?} 实际 {:?}",
                zone, want_touch[zone as usize], got
            ));
        }
    }
    for index in 0..12u8 {
        let got = ctrl.kbd_hold_phys(index);
        if got != want_hold_phys[index as usize] {
            bad.push(format!(
                "hold phys{}: 期望 {:?} 实际 {:?}",
                index, want_hold_phys[index as usize], got
            ));
        }
    }
    for zone in 0..34u8 {
        let got = ctrl.kbd_hold_zone(zone);
        if got != want_hold_zone[zone as usize] {
            bad.push(format!(
                "hold touch{}: 期望 {:?} 实际 {:?}",
                zone, want_hold_zone[zone as usize], got
            ));
        }
    }
    let got_combos = ctrl.kbd_combos();
    if got_combos.len() != want_combos.len() {
        bad.push(format!(
            "combo: 期望 {} 条 实际 {} 条",
            want_combos.len(),
            got_combos.len()
        ));
        // 组合表存在 KV 的 kbd.cbA/B/K/T 四组键里(固件 keyboard.cpp:98 _load_combo)。同时打印这些键
        // 的回读值即可区分"KV 没落盘"与"KV 有值但重启加载没恢复", 免去再跑一轮漫灌。
        for index in 0..2usize {
            let raw: Vec<String> = ["A", "B", "K", "T"]
                .iter()
                .map(|part| {
                    let key = format!("kbd.cb{}{:02}", part, index);
                    match ctrl.config_get(&key) {
                        Some(entry) => format!("{}={}", key, soak_val_text(&entry.value)),
                        None => format!("{}=缺失", key),
                    }
                })
                .collect();
            bad.push(format!("combo KV 诊断: {}", raw.join(" ")));
        }
    }
    for (index, (want, got)) in want_combos.iter().zip(got_combos.iter()).enumerate() {
        if want.zone_mask != got.zone_mask
            || want.keycodes != got.keycodes
            || want.modifiers != got.modifiers
            || want.delay_ms != got.delay_ms
            || want.max_hold_ms != got.max_hold_ms
        {
            bad.push(format!("combo {}: 期望 mask=0x{:X} keys={:02X?} mod={} hold={}/{}，实际 mask=0x{:X} keys={:02X?} mod={} hold={}/{}",
                index, want.zone_mask, want.keycodes, want.modifiers, want.delay_ms, want.max_hold_ms,
                got.zone_mask, got.keycodes, got.modifiers, got.delay_ms, got.max_hold_ms));
        }
    }
    // 0x09 与 0x0B 都会在 PSoC CalibrateWidget 中按全局初始档覆盖 idacGainIndex；0x04 会在
    // 自动阈值处理链重写 hysteresis。这些重启后值不应用落盘前真值作严格相等断言，而应验证
    // 重复 provision 后结果稳定：R1 == R2。其余 PARAM 仍严格比较落盘前设备真值。
    let mut recalculated_r1 = Vec::new();
    for (channel, id, want) in &want_param {
        let Some(got) = ctrl.param(*channel, *id) else {
            bad.push(format!("PARAM CH{} 0x{:02X}: 回读缺失", channel, id));
            continue;
        };
        if matches!(*id, 0x04 | 0x09 | 0x0B) {
            println!(
                "[SOAK] PSoC 重算 PARAM CH{} 0x{:02X}: 落盘前真值 {} → R1 {}",
                channel, id, want, got
            );
            recalculated_r1.push((*channel, *id, got));
        } else if got != *want {
            bad.push(format!(
                "PARAM CH{} 0x{:02X}: 期望 {} 实际 {}",
                channel, id, want, got
            ));
        }
    }

    println!("[SOAK] 二次重启，核验 PSoC 重算 PARAM 的 R1 == R2…");
    reboot_and_reconnect!("二次重启后");
    request_all_serial(ctrl, "二次重启后", Duration::from_secs(20));
    for (channel, id, r1) in recalculated_r1 {
        match ctrl.param(channel, id) {
            Some(r2) if r2 == r1 => println!(
                "[SOAK] PSoC 重算 PARAM CH{} 0x{:02X}: R1 {} == R2 {} [PASS]",
                channel, id, r1, r2
            ),
            Some(r2) => bad.push(format!(
                "PSoC 重算 PARAM CH{} 0x{:02X}: R1 {} != R2 {}",
                channel, id, r1, r2
            )),
            None => bad.push(format!(
                "PSoC 重算 PARAM CH{} 0x{:02X}: R2 回读缺失",
                channel, id
            )),
        }
    }
    if ctrl.algo_device_src() != expected_source {
        bad.push(format!(
            "ALGO_SRC: 期望 {} 字节，实际 {} 字节",
            expected_source.len(),
            ctrl.algo_device_src().len()
        ));
    }

    let total = want_cfg.len() + 34 + 12 + 34 + 12 + 34 + want_combos.len() + want_param.len() + 1;
    if bad.is_empty() {
        println!("[SOAK] PASS 全部 {} 项在重启后一致", total);
        std::process::exit(0);
    }
    println!("[SOAK] FAIL {} / {} 项不一致:", bad.len(), total);
    for line in bad {
        println!("  {}", line);
    }
    std::process::exit(1);
}
