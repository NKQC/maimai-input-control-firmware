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
    BRINGUP_FLAG_CHECKSUM, BRINGUP_FLAG_LINK, BRINGUP_FLAG_SNAPSHOT, CfgValue, ConfigEntry,
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
const ACCEPTANCE_READY_TIMEOUT_MS: u64 = 2_000;
const ACCEPTANCE_REQUEST_TIMEOUT_MS: u64 = 4_000;
const ACCEPTANCE_ROUNDS: usize = 60;
const ACCEPTANCE_PERIOD_MS: u64 = 90;
const REVIEW_CONFIG_TIMEOUT_MS: u64 = 15_000;
const REVIEW_SAVE_TIMEOUT_MS: u64 = 15_000;
const REVIEW_READBACK_TIMEOUT_MS: u64 = 12_000;
const REVIEW_ALGO_TIMEOUT_MS: u64 = 12_000;

fn _cfg_u32(value: &CfgValue) -> Option<u32> {
    match value {
        CfgValue::U8(value) => Some(u32::from(*value)),
        CfgValue::U16(value) => Some(u32::from(*value)),
        CfgValue::U32(value) => Some(*value),
        _ => None,
    }
}

fn _review_pump<F>(ctrl: &mut AppController, timeout_ms: u64, mut done: F) -> Result<(), String>
where
    F: FnMut(&AppController) -> bool,
{
    // ★必须驱动 poll_scheduled★ 连接探针队列(schedule_conn_probes 排好的那串)只由它提交,
    // `poll()` 只负责排空 IO 事件。此前本泵只调 poll(), 于是 `conn_probes_pending()` 永远为真 ——
    // `--review-closure` 第一步"等待配置真值"必然走满 15s 超时, 整条验收路径实际是断的。
    // 上下文取值与 `_run_acceptance` 同形(无头场景无 UI 页面态)。
    let context = mai2control_ui::app_state::PollContext {
        connected: true,
        current_view: 1,
        settings_tab: 8,
        hid_mode: false,
        sel_channel: 0,
        light_panel_expanded: false,
        phys_la_expanded: false,
    };
    let deadline = std::time::Instant::now() + Duration::from_millis(timeout_ms);
    let mut tick = 0u32;
    while std::time::Instant::now() < deadline {
        ctrl.poll();
        tick = tick.wrapping_add(1);
        ctrl.poll_scheduled(&context, tick, 16);
        ctrl.csd_diag_tick();
        if ctrl.state() == ConnState::Disconnected {
            return Err("设备断开".to_string());
        }
        if done(ctrl) {
            return Ok(());
        }
        thread::sleep(Duration::from_millis(16));
    }
    Err(format!("{}ms 内未收敛", timeout_ms))
}

fn _review_truth_u32(ctrl: &AppController, key: &str) -> Option<u32> {
    ctrl.config_truth(key)
        .and_then(|entry| _cfg_u32(&entry.value))
}

fn _review_save_value(
    ctrl: &mut AppController,
    key: &str,
    entry: ConfigEntry,
    expected: u32,
) -> Result<(), String> {
    let before_version = ctrl.config_version();
    ctrl.set_config(entry)
        .map_err(|error| format!("暂存 {} 失败: {}", key, error))?;
    ctrl.save_config()
        .map_err(|error| format!("保存 {} 失败: {}", key, error))?;
    _review_pump(ctrl, REVIEW_SAVE_TIMEOUT_MS, |ctrl| {
        ctrl.cfg_tx_pending() == 0 && !ctrl.is_config_dirty()
    })
    .map_err(|error| format!("{} 保存终态: {}", key, error))?;
    _review_pump(ctrl, REVIEW_READBACK_TIMEOUT_MS, |ctrl| {
        ctrl.config_version() > before_version && _review_truth_u32(ctrl, key) == Some(expected)
    })
    .map_err(|error| {
        format!(
            "{} 保存后真值回读: {} (期望 {}, 实得 {:?})",
            key,
            error,
            expected,
            _review_truth_u32(ctrl, key)
        )
    })
}

fn _run_review_closure(ctrl: &mut AppController) -> Result<(), String> {
    const KEY: &str = "comm.touch_delay_100us";
    println!("[REVIEW] 等待配置真值...");
    ctrl.schedule_conn_probes(0);
    _review_pump(ctrl, REVIEW_CONFIG_TIMEOUT_MS, |ctrl| {
        !ctrl.conn_probes_pending() && ctrl.config_truth(KEY).is_some()
    })?;

    let original_entry = ctrl
        .config_truth(KEY)
        .cloned()
        .ok_or_else(|| format!("设备未返回 {}", KEY))?;
    let original =
        _cfg_u32(&original_entry.value).ok_or_else(|| format!("{} 类型不是无符号整数", KEY))?;
    let (minimum, maximum) = original_entry
        .range
        .as_ref()
        .and_then(|(minimum, maximum)| Some((_cfg_u32(minimum)?, _cfg_u32(maximum)?)))
        .unwrap_or((0, u32::MAX));
    let preferred = 20u32.clamp(minimum, maximum);
    let test_value = if preferred != original {
        preferred
    } else if original < maximum {
        original + 1
    } else if original > minimum {
        original - 1
    } else {
        return Err(format!(
            "{} 的合法范围只有当前值 {}，无法往返",
            KEY, original
        ));
    };
    println!(
        "[REVIEW] SAVE 测试: {} 原值={} 临时值={}",
        KEY, original, test_value
    );

    let test_entry = ConfigEntry {
        key: KEY.to_string(),
        value: match &original_entry.value {
            CfgValue::U8(_) => CfgValue::U8(test_value as u8),
            CfgValue::U16(_) => CfgValue::U16(test_value as u16),
            CfgValue::U32(_) => CfgValue::U32(test_value),
            _ => return Err(format!("{} 类型无法安全往返", KEY)),
        },
        range: original_entry.range.clone(),
    };

    let closure = (|| -> Result<(), String> {
        _review_save_value(ctrl, KEY, test_entry, test_value)?;
        println!("[REVIEW] SAVE 临时值设备回读={}", test_value);

        ctrl.start_telemetry(20, 0, 0)
            .map_err(|error| format!("启动延迟遥测失败: {}", error))?;
        let lat_before = ctrl.lat_version();
        let telem_deadline = std::time::Instant::now() + Duration::from_secs(10);
        let mut next_start = std::time::Instant::now();
        while std::time::Instant::now() < telem_deadline
            && !(ctrl.lat_version() > lat_before && ctrl.telem_lat_corrected_us().is_some())
        {
            if std::time::Instant::now() >= next_start {
                ctrl.start_telemetry(20, 0, 0)
                    .map_err(|error| format!("启动延迟遥测失败: {}", error))?;
                next_start = std::time::Instant::now() + Duration::from_millis(500);
            }
            ctrl.poll();
            ctrl.csd_diag_tick();
            if ctrl.state() == ConnState::Disconnected {
                return Err("延迟遥测期间设备断开".to_string());
            }
            thread::sleep(Duration::from_millis(16));
        }
        if ctrl.lat_version() <= lat_before || ctrl.telem_lat_corrected_us().is_none() {
            return Err("延迟遥测未形成补正值: 10000ms 内未收敛".to_string());
        }
        // ★口径核验按"主页实际画的那一条"来★ 此前核验的是"画的 = 实测链路耗时 − 延迟线目标",
        // 而那个式子本身就是缺陷(把端到端目标当成链路耗时的期望值去相减), 于是它反过来把缺陷
        // 锁成了验收标准。现在主页两种口径二选一: 启用补偿画偏差, 关闭画链路耗时原值。
        let link = ctrl
            .telem_lat_corrected_us()
            .ok_or_else(|| "链路耗时为空".to_string())?;
        let floor = ctrl
            .latency_compensable_us()
            .ok_or_else(|| "可补偿段为空".to_string())?;
        let target = ctrl.touch_delay_target_us();
        if floor > link {
            return Err(format!(
                "可补偿段 {}us 不该大于链路耗时 {}us(SPI 段必须被排除在外)",
                floor, link
            ));
        }
        let plotted_link = ctrl
            .lat_link_series()
            .last()
            .copied()
            .ok_or_else(|| "链路耗时序列为空".to_string())?;
        if (plotted_link - link as f32).abs() > 0.5 {
            return Err(format!(
                "链路耗时口径不一致: link={} plotted={}",
                link, plotted_link
            ));
        }
        let plotted_dev = ctrl
            .lat_dev_series()
            .last()
            .copied()
            .ok_or_else(|| "偏差序列为空".to_string())?;
        let dev = ctrl.telem_delay_dev_us();
        // 图上画的是区间里"偏得更狠"的那一端(见 LatObs::dev_worst_us)。
        let worst = dev
            .map(|(lo, hi)| if hi.unsigned_abs() >= lo.unsigned_abs() { hi } else { lo })
            .unwrap_or(0);
        if (plotted_dev - worst as f32).abs() > 0.5 {
            return Err(format!(
                "偏差口径不一致: dev={:?} worst={} plotted={}",
                dev, worst, plotted_dev
            ));
        }
        if let Some((lo, hi)) = dev {
            if lo > hi {
                return Err(format!("偏差区间倒挂: min={} max={}", lo, hi));
            }
            // 偏差的物理上界: 它等于 (实际写出耗时 − 预测耗时) + 延迟线 100us 时间片截断量, 而两个
            // 耗时都不超过遥测窗口内测到的链路耗时峰值 ⇒ |偏差| 必然被 link + 一片有余量地框住。
            // 超出即说明解码错位、符号弄反, 或采样零点取错(那才是真缺陷, 不是抖动)。
            let bound = link as i64 + 200;
            if (lo as i64).abs() > bound || (hi as i64).abs() > bound {
                return Err(format!(
                    "偏差区间 {}..{}us 超出物理上界(链路耗时 {}us + 一个 100us 时间片): 口径或解码有误",
                    lo, hi, link
                ));
            }
        }
        println!(
            "[REVIEW] LATENCY 链路耗时={}us 可补偿段={}us 延迟线目标={}us 目标可达={} | 偏差={:?}us plotted_dev={:.0} plotted_link={:.0}",
            link,
            floor,
            target,
            target == 0 || target >= floor,
            dev,
            plotted_dev,
            plotted_link
        );
        let _ = ctrl.stop_telemetry();

        let baseline = ctrl
            .algo_info()
            .ok_or_else(|| "连接探针未取得算法信息".to_string())?;
        if baseline.is_default || !baseline.psoc_valid || baseline.len == 0 {
            return Err(format!("当前用户算法状态不适合原样闭环: {:?}", baseline));
        }
        let code_before = ctrl.algo_device_code_version();
        let code_deadline = std::time::Instant::now() + Duration::from_secs(10);
        let mut next_code_request = std::time::Instant::now();
        while std::time::Instant::now() < code_deadline
            && !(ctrl.algo_device_code_version() > code_before
                && !ctrl.algo_device_code().is_empty())
        {
            if std::time::Instant::now() >= next_code_request {
                ctrl.request_algo_code()
                    .map_err(|error| format!("回读算法机器码失败: {}", error))?;
                next_code_request = std::time::Instant::now() + Duration::from_millis(750);
            }
            ctrl.poll();
            ctrl.csd_diag_tick();
            if ctrl.state() == ConnState::Disconnected {
                return Err("算法机器码回读期间设备断开".to_string());
            }
            thread::sleep(Duration::from_millis(16));
        }
        if ctrl.algo_device_code_version() <= code_before || ctrl.algo_device_code().is_empty() {
            return Err("算法机器码回读 10000ms 内未收敛".to_string());
        }
        let code = ctrl.algo_device_code().to_vec();
        let code_crc = mai2control_ui::proto::algo::crc16_ccitt(&code);
        if code.len() != usize::from(baseline.len) || code_crc != baseline.crc16 {
            return Err(format!(
                "算法回读 blob 与设备信息不符: info len={} crc=0x{:04X}, blob len={} crc=0x{:04X}",
                baseline.len,
                baseline.crc16,
                code.len(),
                code_crc
            ));
        }
        println!(
            "[REVIEW] ALGO baseline is_default={} valid={} len={} crc16=0x{:04X}",
            baseline.is_default, baseline.psoc_valid, baseline.len, baseline.crc16
        );
        ctrl.algo_upload(&code)
            .map_err(|error| format!("原样重传算法失败: {}", error))?;
        _review_pump(ctrl, REVIEW_ALGO_TIMEOUT_MS, |ctrl| {
            ctrl.algo_upload_status().contains("已装上并正在运行")
                || ctrl.algo_upload_status().contains("终态不匹配")
                || ctrl.algo_upload_status().contains("上传被设备拒绝(NAK)")
                || ctrl.algo_upload_status().contains("上传超时")
        })?;
        let upload_status = ctrl.algo_upload_status().to_string();
        if !upload_status.contains("已装上并正在运行") {
            return Err(format!("算法上传未获运行终态: {}", upload_status));
        }
        let final_info = ctrl
            .algo_info()
            .ok_or_else(|| "算法上传后未收到设备真值".to_string())?;
        if !final_info.psoc_valid
            || final_info.len != baseline.len
            || final_info.crc16 != baseline.crc16
        {
            return Err(format!(
                "算法重传后真值不一致: valid={} len={} crc=0x{:04X}",
                final_info.psoc_valid, final_info.len, final_info.crc16
            ));
        }
        println!("[REVIEW] ALGO status={}", upload_status);
        println!(
            "[REVIEW] ALGO final valid={} len={} crc16=0x{:04X}",
            final_info.psoc_valid, final_info.len, final_info.crc16
        );

        let bad_messages: Vec<String> = ctrl
            .diagnostic_event_messages()
            .into_iter()
            .filter(|message| message.contains('�') || message.contains("没有装上"))
            .collect();
        if !bad_messages.is_empty() {
            return Err(format!(
                "日志含乱码或误导文案: {}",
                bad_messages.join(" | ")
            ));
        }
        println!("[REVIEW] TEXT PASS replacement-char=0 misleading-fallback=0");
        Ok(())
    })();

    println!("[REVIEW] 恢复 {}={}...", KEY, original);
    let restore_result = _review_save_value(ctrl, KEY, original_entry, original);
    match (closure, restore_result) {
        (Ok(()), Ok(())) => {
            println!("[REVIEW] RESTORE PASS device_truth={}", original);
            Ok(())
        }
        (Err(error), Ok(())) => Err(format!("{}；原配置已恢复", error)),
        (Ok(()), Err(restore)) => Err(format!("闭环通过，但恢复原配置失败: {}", restore)),
        (Err(error), Err(restore)) => Err(format!("{}；且恢复原配置失败: {}", error, restore)),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// --vcam-probe 的过滤器侧覆盖: MSBuild 产物的 PE 校验 + x64 DLL 的真实 COM 构造路径。
//
// 存在理由: 共享队列那半边(Rust 生产者 ↔ Rust 读端)本来就能自验, 但"过滤器 DLL 到底能不能被
// 当成 COM 服务器加载起来"此前完全没有覆盖 —— 只打印了一句注册状态。于是 DllGetClassObject /
// 类工厂 / IBaseFilter 状态机这条路只能靠"装上摄像头, 打开某个游戏看画面"来验, 一旦回归就只有
// 用户能发现。这里直接 LoadLibrary 构建产物并走完整 COM 路径, **不写 HKLM、不需要管理员**。
// ─────────────────────────────────────────────────────────────────────────────

/// MSBuild 产物的相对路径(与 build.rs 内嵌来源同源, 改一处必须同步另一处)。
const VCAM_DLL_X64: &str = "vcam_source_cpp/x64/Release/mai2vcam_dshow.dll";
const VCAM_DLL_X86: &str = "vcam_source_cpp/Win32/Release/mai2vcam_dshow.dll";
/// 与 mai2vcam_dshow.def 的 EXPORTS 一一对应。
const VCAM_EXPORTS: [&str; 4] = [
    "DllGetClassObject",
    "DllCanUnloadNow",
    "DllRegisterServer",
    "DllUnregisterServer",
];
const PE_MACHINE_AMD64: u16 = 0x8664;
const PE_MACHINE_I386: u16 = 0x014C;
/// 过滤器 CLSID(与 vcam_common.cpp 的 DEFINE_GUID 及 vcam/backend.rs 的常量一致)。
const VCAM_CLSID: windows::core::GUID =
    windows::core::GUID::from_u128(0x6E5A1C74_2F83_4C9B_9D1E_7A4B0F3C58E2);
/// quartz.dll 内置 FilterGraph 与 NullRenderer；windows 0.62 的生成绑定未暴露这两个 CLSID 常量。
const VCAM_FILTER_GRAPH_CLSID: windows::core::GUID =
    windows::core::GUID::from_u128(0xE436EBB3_524F_11CE_9F53_0020AF0BA770);
const VCAM_NULL_RENDERER_CLSID: windows::core::GUID =
    windows::core::GUID::from_u128(0xC1F400A4_3F08_11D3_9F0B_006008039E37);
/// 与 `vcam_common.h::MAI2VCAM_FRIENDLY_NAME` / `vcam/backend.rs::INSTANCE_NAME` 一致。
const MAI2VCAM_FRIENDLY_NAME_STR: &str = "mai2control Virtual Camera";

/// 在"当前目录"与"selftest.exe 的各级父目录"下找构建产物。
/// selftest.exe 通常在 `control_software/target/debug/`, 因此 `control_software/` 就在其祖先里。
fn _locate_build_output(relative: &str) -> Option<std::path::PathBuf> {
    let mut roots: Vec<std::path::PathBuf> = Vec::new();
    if let Ok(dir) = std::env::current_dir() {
        roots.push(dir);
    }
    if let Ok(exe) = std::env::current_exe() {
        roots.extend(exe.ancestors().map(|path| path.to_path_buf()));
    }
    roots
        .into_iter()
        .map(|root| root.join(relative))
        .find(|candidate| candidate.is_file())
}

/// 自解析 PE 导出表, 返回 (machine, 导出名列表)。
///
/// 为什么自己解析而不调 dumpbin: 自测不能依赖机器上装了 VS 工具链, 而"这份 DLL 是不是目标位宽、
/// 有没有那四个导出"恰恰是 x86 产物唯一能在 x64 进程里验的东西(x86 DLL 不可能被 x64 进程加载)。
fn _pe_exports(path: &std::path::Path) -> Result<(u16, Vec<String>), String> {
    let bytes =
        std::fs::read(path).map_err(|error| format!("读取 {} 失败: {}", path.display(), error))?;
    let u16_at = |offset: usize| -> Result<u16, String> {
        bytes
            .get(offset..offset + 2)
            .map(|slice| u16::from_le_bytes([slice[0], slice[1]]))
            .ok_or_else(|| format!("偏移 0x{:X} 越界(文件 {}B)", offset, bytes.len()))
    };
    let u32_at = |offset: usize| -> Result<u32, String> {
        bytes
            .get(offset..offset + 4)
            .map(|slice| u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
            .ok_or_else(|| format!("偏移 0x{:X} 越界(文件 {}B)", offset, bytes.len()))
    };
    if u16_at(0)? != 0x5A4D {
        return Err("不是 PE 文件(缺少 MZ)".to_string());
    }
    let pe = u32_at(0x3C)? as usize;
    if u32_at(pe)? != 0x0000_4550 {
        return Err("不是 PE 文件(缺少 PE\\0\\0 签名)".to_string());
    }
    let machine = u16_at(pe + 4)?;
    let sections = u16_at(pe + 6)? as usize;
    let optional_size = u16_at(pe + 20)? as usize;
    let optional = pe + 24;
    // PE32 的 DataDirectory 在可选头 +96, PE32+ 在 +112。
    let directory = match u16_at(optional)? {
        0x010B => optional + 96,
        0x020B => optional + 112,
        magic => return Err(format!("未知可选头 magic 0x{:04X}", magic)),
    };
    let export_rva = u32_at(directory)?;
    if export_rva == 0 {
        return Ok((machine, Vec::new()));
    }
    // 节表: 每项 40 字节, 用它把 RVA 换算成文件偏移。
    let section_table = optional + optional_size;
    let to_offset = |rva: u32| -> Result<usize, String> {
        for index in 0..sections {
            let entry = section_table + index * 40;
            let virtual_size = u32_at(entry + 8)?;
            let virtual_address = u32_at(entry + 12)?;
            let raw_size = u32_at(entry + 16)?;
            let raw_pointer = u32_at(entry + 20)?;
            let span = virtual_size.max(raw_size);
            if rva >= virtual_address && rva < virtual_address + span {
                return Ok((raw_pointer + (rva - virtual_address)) as usize);
            }
        }
        Err(format!("RVA 0x{:X} 不落在任何节内", rva))
    };
    let export = to_offset(export_rva)?;
    let name_count = u32_at(export + 24)? as usize;
    let names_rva = u32_at(export + 32)?;
    let names = to_offset(names_rva)?;
    let mut exported = Vec::with_capacity(name_count);
    for index in 0..name_count {
        let rva = u32_at(names + index * 4)?;
        let start = to_offset(rva)?;
        let end = bytes[start..]
            .iter()
            .position(|byte| *byte == 0)
            .map(|length| start + length)
            .ok_or_else(|| "导出名未以 NUL 结尾".to_string())?;
        exported.push(String::from_utf8_lossy(&bytes[start..end]).to_string());
    }
    Ok((machine, exported))
}

/// PE 层面核对一个产物: 位宽 + 四个导出齐全。失败项累加到 `failures`。
fn _check_vcam_pe(
    label: &str,
    relative: &str,
    expected_machine: u16,
    failures: &mut Vec<String>,
) -> Option<std::path::PathBuf> {
    let inspected = match _locate_build_output(relative) {
        Some(path) => match _pe_exports(&path) {
            Ok((machine, exports)) => {
                let missing: Vec<&str> = VCAM_EXPORTS
                    .iter()
                    .copied()
                    .filter(|name| !exports.iter().any(|export| export == name))
                    .collect();
                println!(
                    "[VCAM] {} 产物: {} machine=0x{:04X} 导出={}",
                    label,
                    path.display(),
                    machine,
                    exports.join(",")
                );
                if machine != expected_machine {
                    failures.push(format!(
                        "{} 产物 machine=0x{:04X}, 期望 0x{:04X}",
                        label, machine, expected_machine
                    ));
                }
                if !missing.is_empty() {
                    failures.push(format!(
                        "{} 产物缺少导出: {}(def 文件是否漏了?)",
                        label,
                        missing.join(",")
                    ));
                }
                Some(path)
            }
            Err(error) => {
                failures.push(format!("{} 产物 PE 解析失败: {}", label, error));
                None
            }
        },
        None => {
            failures.push(format!(
                "{} 构建产物未找到: {}(先用 MSBuild 构建 vcam_source_cpp 的 Release|x64 与 Release|Win32)",
                label, relative
            ));
            None
        }
    };
    inspected
}

/// 走真实 COM 路径构造并释放过滤器: `LoadLibrary` → `DllGetClassObject` → `IClassFactory` →
/// `CreateInstance(IBaseFilter)` → 身份/针脚/格式核对 → 接入系统 Null Renderer 的完整 filter graph →
/// graph Pause/Run/Stop → 全部释放 → `DllCanUnloadNow` 应回 S_OK → `FreeLibrary`。
/// 直接加载构建产物，不写注册表；同一探针可分别编译成 x64/x86，验证两个 in-proc 位宽。
/// `during_run`: graph 进入 Running 之后立刻回调一次, 供调用方在**推流真的在跑的时候**改动
/// 生产者状态。目前唯一的用途是把发布分辨率换成与已协商的那一种不同的值, 以此覆盖"尺寸错开 →
/// 只给占位帧、绝不缩放"这条分支 —— 它在别处根本没法触发: 针脚一旦连上尺寸就冻结, 同一次
/// 连接里不主动改生产者就永远相等。
fn _probe_vcam_com(
    dll: &std::path::Path,
    probe_registered: bool,
    during_run: &mut dyn FnMut(),
    failures: &mut Vec<String>,
) {
    use std::os::windows::ffi::OsStrExt;
    use windows::Win32::Foundation::FreeLibrary;
    use windows::Win32::Media::DirectShow::{IBaseFilter, IEnumPins};
    use windows::Win32::System::Com::{
        COINIT_MULTITHREADED, CoInitializeEx, CoUninitialize, IClassFactory,
    };
    use windows::Win32::System::LibraryLoader::{
        GetProcAddress, LOAD_WITH_ALTERED_SEARCH_PATH, LoadLibraryExW,
    };
    use windows::core::{GUID, HRESULT, Interface, PCSTR, PCWSTR};

    type GetClassObject =
        unsafe extern "system" fn(*const GUID, *const GUID, *mut *mut core::ffi::c_void) -> HRESULT;
    type CanUnloadNow = unsafe extern "system" fn() -> HRESULT;

    let wide: Vec<u16> = dll
        .as_os_str()
        .encode_wide()
        .chain(std::iter::once(0))
        .collect();
    // SAFETY: 全程手工管理 COM 引用与模块句柄; 所有 COM 指针在 FreeLibrary 之前释放。
    unsafe {
        let _ = CoInitializeEx(None, COINIT_MULTITHREADED);
        let module =
            match LoadLibraryExW(PCWSTR(wide.as_ptr()), None, LOAD_WITH_ALTERED_SEARCH_PATH) {
                Ok(module) => module,
                Err(error) => {
                    failures.push(format!("LoadLibraryEx {} 失败: {}", dll.display(), error));
                    CoUninitialize();
                    return;
                }
            };
        let entry = GetProcAddress(module, PCSTR(c"DllGetClassObject".as_ptr().cast()));
        let unload = GetProcAddress(module, PCSTR(c"DllCanUnloadNow".as_ptr().cast()));
        match (entry, unload) {
            (Some(entry), Some(unload)) => {
                let get_class_object: GetClassObject = std::mem::transmute(entry);
                let can_unload: CanUnloadNow = std::mem::transmute(unload);
                _probe_vcam_class_object(get_class_object, can_unload, during_run, failures);
                if probe_registered {
                    _probe_registered_vcam(during_run, failures);
                } else {
                    println!("[VCAM] 系统尚未完整部署，跳过注册表 CLSID graph 探针");
                }
            }
            _ => failures.push(
                "GetProcAddress 取不到 DllGetClassObject / DllCanUnloadNow(def 文件是否漏了导出?)"
                    .to_string(),
            ),
        }
        if let Err(error) = FreeLibrary(module) {
            failures.push(format!("FreeLibrary 失败: {}", error));
        }
        CoUninitialize();
    }

    /// COM 对象的构造/使用/释放全在这个函数里, 保证在返回时所有引用都已 drop ——
    /// 任何一个 COM 引用活过 FreeLibrary 都是立刻崩溃。
    unsafe fn _probe_vcam_class_object(
        get_class_object: GetClassObject,
        can_unload: CanUnloadNow,
        during_run: &mut dyn FnMut(),
        failures: &mut Vec<String>,
    ) {
        let mut raw: *mut core::ffi::c_void = std::ptr::null_mut();
        let hr = unsafe { get_class_object(&VCAM_CLSID, &IClassFactory::IID, &mut raw) };
        if hr.is_err() || raw.is_null() {
            failures.push(format!(
                "DllGetClassObject(IClassFactory) hr=0x{:08X}",
                hr.0
            ));
            return;
        }
        // SAFETY: raw 是刚由 DllGetClassObject 返回的 IClassFactory, 所有权转交给这个包装。
        let factory: IClassFactory = unsafe { IClassFactory::from_raw(raw) };
        // 未注册 CLSID 也应可拒绝聚合并直接构造 —— 这才是消费端枚举后真正走的路。
        let filter: IBaseFilter = match unsafe { factory.CreateInstance(None) } {
            Ok(filter) => filter,
            Err(error) => {
                failures.push(format!(
                    "IClassFactory::CreateInstance(IBaseFilter): {}",
                    error
                ));
                return;
            }
        };
        println!("[VCAM] COM: DllGetClassObject → IClassFactory → IBaseFilter 构造成功");
        match unsafe { filter.GetClassID() } {
            Ok(id) if id == VCAM_CLSID => println!("[VCAM] COM: GetClassID 与预期 CLSID 一致"),
            Ok(id) => failures.push(format!("GetClassID 返回 {:?}, 与预期 CLSID 不符", id)),
            Err(error) => failures.push(format!("GetClassID: {}", error)),
        }
        // 针脚: 必须恰好一个输出针脚。
        match unsafe { filter.EnumPins() } {
            Ok(pins) => _probe_vcam_pins(&pins, failures),
            Err(error) => failures.push(format!("EnumPins: {}", error)),
        }
        // 完整图：源针脚必须真的完成 allocator 协商、推流并让 Null Renderer 进入 Paused，
        // 这比未连接状态下直调过滤器状态机更接近 OBS/游戏的实际消费路径。
        _probe_vcam_graph(&filter, during_run, failures);
        drop(filter);
        // ★第二轮: "改完分辨率, 在消费端重新打开摄像头"这一步必须真的走一遍★
        // 上一轮的 during_run 已经把生产者分辨率换掉了, 此刻新建的过滤器实例就是用户重开摄像头
        // 时拿到的那个东西 —— 它必须按**新**尺寸协商并取到真实帧。这一面不能只靠注册表那一轮
        // 覆盖: 本机没装摄像头时那一轮整个会被跳过, 而"改分辨率要卸载重建"恰恰是最需要钉住的
        // 硬要求, 不能让它的覆盖取决于本机装没装。
        match unsafe { factory.CreateInstance(None) } {
            Ok(reopened) => {
                println!("[VCAM] COM: 生产者分辨率已变更 → 重新打开摄像头(新建第二个过滤器实例)");
                _probe_vcam_graph(&reopened, during_run, failures);
                drop(reopened);
            }
            Err(error) => failures.push(format!(
                "分辨率变更后重新 CreateInstance(IBaseFilter): {}",
                error
            )),
        }
        drop(factory);
        // 全部引用已释放 ⇒ 模块计数必须归零, 否则 DLL 永远卸不掉(类工厂/枚举器漏了计数)。
        let hr = unsafe { can_unload() };
        if hr == HRESULT(0) {
            println!("[VCAM] COM: 释放全部引用后 DllCanUnloadNow=S_OK(模块计数已归零)");
        } else {
            failures.push(format!(
                "释放全部引用后 DllCanUnloadNow=0x{:08X}(期望 S_OK; 有对象漏了模块计数)",
                hr.0
            ));
        }
    }

    unsafe fn _probe_registered_vcam(during_run: &mut dyn FnMut(), failures: &mut Vec<String>) {
        use windows::Win32::System::Com::{CLSCTX_INPROC_SERVER, CoCreateInstance};
        let filter: IBaseFilter =
            match unsafe { CoCreateInstance(&VCAM_CLSID, None, CLSCTX_INPROC_SERVER) } {
                Ok(filter) => filter,
                Err(error) => {
                    failures.push(format!(
                        "注册表 CoCreateInstance(mai2control vcam): {}",
                        error
                    ));
                    return;
                }
            };
        println!("[VCAM] 注册表 CLSID：原生位宽 CoCreateInstance 成功");
        _probe_vcam_graph(&filter, during_run, failures);
        // ★直接 CoCreateInstance(CLSID) 只证明"能造实例", 不证明"消费端枚举得到它"★
        // OBS/游戏这类消费端从不硬编码 CLSID, 而是走 ICreateDevEnum::CreateClassEnumerator(
        // CLSID_VideoInputDeviceCategory) 拿 IEnumMoniker, 再 BindToStorage 到 IPropertyBag 读
        // FriendlyName —— 这正是 IFilterMapper2::RegisterFilter 写的 Instance 子键那条路径的
        // 运行期镜像。之前的探针跳过了这一步, "已安装但摄像头列表里看不到"这类问题因此漏检。
        unsafe { _probe_system_enum(failures) };
    }

    /// 走消费端真实路径: `ICreateDevEnum` → `IEnumMoniker`(VideoInputDeviceCategory) →
    /// 逐个 moniker `BindToStorage(IPropertyBag)` 读 FriendlyName, 核对能枚举到本过滤器。
    unsafe fn _probe_system_enum(failures: &mut Vec<String>) {
        use windows::Win32::Media::DirectShow::ICreateDevEnum;
        use windows::Win32::Media::MediaFoundation::CLSID_VideoInputDeviceCategory;
        use windows::Win32::System::Com::StructuredStorage::IPropertyBag;
        use windows::Win32::System::Com::{CLSCTX_INPROC_SERVER, CoCreateInstance, IBindCtx};
        use windows::core::w;

        let dev_enum: ICreateDevEnum = match unsafe {
            CoCreateInstance(
                &windows::Win32::Media::MediaFoundation::CLSID_SystemDeviceEnum,
                None,
                CLSCTX_INPROC_SERVER,
            )
        } {
            Ok(dev_enum) => dev_enum,
            Err(error) => {
                failures.push(format!("CoCreateInstance(SystemDeviceEnum): {}", error));
                return;
            }
        };
        let mut moniker_enum = None;
        if let Err(error) = unsafe {
            dev_enum.CreateClassEnumerator(&CLSID_VideoInputDeviceCategory, &mut moniker_enum, 0)
        } {
            failures.push(format!(
                "CreateClassEnumerator(VideoInputDeviceCategory): {}",
                error
            ));
            return;
        }
        let Some(moniker_enum) = moniker_enum else {
            failures.push(
                "CreateClassEnumerator(VideoInputDeviceCategory) 返回空枚举器: 系统里没有任何视频输入设备"
                    .to_string(),
            );
            return;
        };
        let bind_ctx: IBindCtx = match unsafe { windows::Win32::System::Com::CreateBindCtx(0) } {
            Ok(ctx) => ctx,
            Err(error) => {
                failures.push(format!("CreateBindCtx: {}", error));
                return;
            }
        };
        let mut found = false;
        let mut names: Vec<String> = Vec::new();
        loop {
            let mut slot = [None];
            let mut fetched = 0u32;
            let hr = unsafe { moniker_enum.Next(&mut slot, Some(&mut fetched)) };
            let Some(moniker) = slot[0].take() else {
                break;
            };
            let bag: Result<IPropertyBag, _> = unsafe { moniker.BindToStorage(&bind_ctx, None) };
            match bag {
                Ok(bag) => {
                    let mut var = windows::Win32::System::Variant::VARIANT::default();
                    let read = unsafe { bag.Read(w!("FriendlyName"), &mut var, None) };
                    if read.is_ok() {
                        let name = unsafe { var.Anonymous.Anonymous.Anonymous.bstrVal.to_string() };
                        if name == MAI2VCAM_FRIENDLY_NAME_STR {
                            found = true;
                        }
                        names.push(name);
                    }
                }
                Err(error) => failures.push(format!(
                    "枚举视频输入设备: BindToStorage(IPropertyBag) 失败: {}",
                    error
                )),
            }
            if hr.is_err() {
                break;
            }
        }
        if found {
            println!(
                "[VCAM] 系统枚举: ICreateDevEnum→VideoInputDeviceCategory 已列出「{}」(共 {} 个视频输入设备)",
                MAI2VCAM_FRIENDLY_NAME_STR,
                names.len()
            );
        } else {
            failures.push(format!(
                "系统枚举: VideoInputDeviceCategory 未列出「{}」; 实际列出: [{}]",
                MAI2VCAM_FRIENDLY_NAME_STR,
                names.join(", ")
            ));
        }
    }

    fn _probe_vcam_pins(pins: &IEnumPins, failures: &mut Vec<String>) {
        use windows::Win32::Media::DirectShow::{IAMStreamConfig, PINDIR_OUTPUT};
        let mut slot = [None];
        let mut fetched = 0u32;
        let hr = unsafe { pins.Next(&mut slot, Some(&mut fetched)) };
        let Some(pin) = slot[0].take() else {
            failures.push(format!("EnumPins::Next 未返回针脚(hr=0x{:08X})", hr.0));
            return;
        };
        match unsafe { pin.QueryDirection() } {
            Ok(direction) if direction == PINDIR_OUTPUT => {}
            Ok(direction) => failures.push(format!("针脚方向 {:?}, 期望输出", direction)),
            Err(error) => failures.push(format!("QueryDirection: {}", error)),
        }
        match pin.cast::<IAMStreamConfig>() {
            Ok(config) => {
                let mut count = 0i32;
                let mut size = 0i32;
                match unsafe { config.GetNumberOfCapabilities(&mut count, &mut size) } {
                    Ok(()) if count == 1 => {
                        println!("[VCAM] COM: 输出针脚 1 个, IAMStreamConfig 报 1 种能力")
                    }
                    Ok(()) => failures.push(format!(
                        "GetNumberOfCapabilities 报 {} 种能力, 期望 1",
                        count
                    )),
                    Err(error) => failures.push(format!("GetNumberOfCapabilities: {}", error)),
                }
            }
            Err(error) => failures.push(format!("针脚 QueryInterface(IAMStreamConfig): {}", error)),
        }
        // 第二次 Next 必须报没有更多针脚。
        let mut extra = [None];
        let hr = unsafe { pins.Next(&mut extra, Some(&mut fetched)) };
        if extra[0].is_some() || fetched != 0 {
            failures.push(format!("EnumPins 报了第 2 个针脚(hr=0x{:08X})", hr.0));
        }
    }

    fn _probe_vcam_graph(
        source: &IBaseFilter,
        during_run: &mut dyn FnMut(),
        failures: &mut Vec<String>,
    ) {
        use windows::Win32::Media::DirectShow::{
            FILTER_STATE, IFilterGraph, IGraphBuilder, IMediaFilter, PIN_DIRECTION, PINDIR_INPUT,
            PINDIR_OUTPUT, State_Paused, State_Running, State_Stopped,
        };
        use windows::Win32::System::Com::{CLSCTX_INPROC_SERVER, CoCreateInstance};
        use windows::core::{Interface, w};

        fn first_pin(
            filter: &IBaseFilter,
            direction: PIN_DIRECTION,
        ) -> Result<windows::Win32::Media::DirectShow::IPin, String> {
            let pins = unsafe { filter.EnumPins() }.map_err(|error| error.to_string())?;
            loop {
                let mut slot = [None];
                let mut fetched = 0u32;
                let hr = unsafe { pins.Next(&mut slot, Some(&mut fetched)) };
                let Some(pin) = slot[0].take() else {
                    return Err(format!(
                        "未找到方向 {:?} 的针脚(hr=0x{:08X})",
                        direction, hr.0
                    ));
                };
                if unsafe { pin.QueryDirection() }.ok() == Some(direction) {
                    return Ok(pin);
                }
            }
        }

        let expect = |media: &IMediaFilter,
                      want: FILTER_STATE,
                      label: &str,
                      failures: &mut Vec<String>| {
            match unsafe { media.GetState(2_000) } {
                Ok(state) if state == want => println!("[VCAM] graph: {} → GetState 一致", label),
                Ok(state) => failures.push(format!(
                    "graph {} 后 GetState={:?}, 期望 {:?}",
                    label, state, want
                )),
                Err(error) => failures.push(format!("graph {} 后 GetState: {}", label, error)),
            }
        };

        unsafe {
            let graph: IGraphBuilder =
                match CoCreateInstance(&VCAM_FILTER_GRAPH_CLSID, None, CLSCTX_INPROC_SERVER) {
                    Ok(graph) => graph,
                    Err(error) => {
                        failures.push(format!("CoCreateInstance(FilterGraph): {}", error));
                        return;
                    }
                };
            let sink: IBaseFilter =
                match CoCreateInstance(&VCAM_NULL_RENDERER_CLSID, None, CLSCTX_INPROC_SERVER) {
                    Ok(sink) => sink,
                    Err(error) => {
                        failures.push(format!("CoCreateInstance(NullRenderer): {}", error));
                        return;
                    }
                };
            if let Err(error) = graph.AddFilter(source, w!("mai2control source")) {
                failures.push(format!("graph AddFilter(source): {}", error));
                return;
            }
            if let Err(error) = graph.AddFilter(&sink, w!("Null Renderer")) {
                failures.push(format!("graph AddFilter(null renderer): {}", error));
                return;
            }
            let output = match first_pin(source, PINDIR_OUTPUT) {
                Ok(pin) => pin,
                Err(error) => {
                    failures.push(format!("graph source pin: {}", error));
                    return;
                }
            };
            let input = match first_pin(&sink, PINDIR_INPUT) {
                Ok(pin) => pin,
                Err(error) => {
                    failures.push(format!("graph sink pin: {}", error));
                    return;
                }
            };
            let filter_graph: IFilterGraph = match graph.cast() {
                Ok(filter_graph) => filter_graph,
                Err(error) => {
                    failures.push(format!("graph QueryInterface(IFilterGraph): {}", error));
                    return;
                }
            };
            if let Err(error) = filter_graph.ConnectDirect(&output, &input, None) {
                failures.push(format!(
                    "graph ConnectDirect(NV12 → NullRenderer): {}",
                    error
                ));
                return;
            }
            println!("[VCAM] graph: NV12 输出针脚已连接到系统 Null Renderer");
            let media: IMediaFilter = match graph.cast() {
                Ok(media) => media,
                Err(error) => {
                    failures.push(format!("graph QueryInterface(IMediaFilter): {}", error));
                    return;
                }
            };
            expect(&media, State_Stopped, "初始 Stopped", failures);
            match media.Pause() {
                Ok(()) => expect(&media, State_Paused, "Pause(已收到预卷帧)", failures),
                Err(error) => failures.push(format!("graph Pause: {}", error)),
            }
            match media.Run(0) {
                Ok(()) => {
                    // 推流线程此刻确实在跑, 调用方可以在这里改生产者状态(见 during_run 说明)。
                    during_run();
                    std::thread::sleep(std::time::Duration::from_millis(180));
                    expect(&media, State_Running, "Run", failures);
                }
                Err(error) => failures.push(format!("graph Run: {}", error)),
            }
            match media.Stop() {
                Ok(()) => expect(&media, State_Stopped, "Stop", failures),
                Err(error) => failures.push(format!("graph Stop: {}", error)),
            }
        }
    }
}

/// `--vcam-probe`: 虚拟摄像头链路的**本机自验**(不依赖固件, 也不需要先装摄像头)。
///
/// 验的是生产侧全链路: 共享队列布局 → RGB24→NV12 转换 → 三缓冲发布/读取协议 → 心跳 →
/// 退出时的 STOPPING 迁移。消费侧(DirectShow 过滤器)活在别的进程里, 与本侧唯一的耦合就是
/// 这套布局, 因此这里用与 C++ `Mai2VcamQueueReader` **同规则**的 Rust 读端逐字节核对;
/// 过滤器自身的注册状态只做信息打印, 不参与判定(装不装是用户的选择)。
fn _run_vcam_probe() -> bool {
    use mai2control_ui::vcam::share::{
        FramePublisher, QueueReader, QueueState, SLOT_STRIDE, black_nv12, nv12_bytes,
        rgb24_to_nv12,
    };
    use mai2control_ui::vcam::{
        DEFAULT_FRAME_H as FRAME_H, DEFAULT_FRAME_W as FRAME_W, Frame, backend as vcam_backend,
        render_qr_frame,
    };
    // 本探针一律在默认分辨率下验证协议本身: 分辨率可变这件事由下面 2.6 节单独覆盖。
    let nv12_len = nv12_bytes(FRAME_W, FRAME_H);

    println!("[VCAM] probe begin");
    println!(
        "[VCAM] 过滤器 DLL 内嵌: {}",
        if vcam_backend::embedded_available() {
            "是(x64 + x86)"
        } else {
            "否(本次构建未携带)"
        }
    );
    println!(
        "[VCAM] 系统注册状态: {}",
        vcam_backend::registration_status()
    );

    let mut publisher = match FramePublisher::create(FRAME_W, FRAME_H) {
        Ok(publisher) => publisher,
        Err(error) => {
            println!("[VCAM] FramePublisher::create: FAIL {}", error);
            println!("[VCAM] probe: FAIL");
            println!("[VCAM] probe end");
            return false;
        }
    };
    let reader = match QueueReader::open() {
        Ok(reader) => reader,
        Err(error) => {
            println!("[VCAM] QueueReader::open: FAIL {}", error);
            println!("[VCAM] probe: FAIL");
            println!("[VCAM] probe end");
            return false;
        }
    };

    let mut failures: Vec<String> = Vec::new();

    // 1) 头部字段: 消费端就是按这些字段决定"要不要相信这块内存", 任一不符都会被判占位帧。
    match reader.header() {
        Ok(header) => {
            println!(
                "[VCAM] header state={:?} {}x{} slots={} slot_bytes={} interval={}x100ns pid={} seq={}",
                header.state,
                header.width,
                header.height,
                header.slot_count,
                header.slot_bytes,
                header.interval_100ns,
                header.producer_pid,
                header.sequence,
            );
            if header.state != QueueState::Ready {
                failures.push(format!("初始状态 {:?} 不是 Ready", header.state));
            }
            if header.width != FRAME_W as u32 || header.height != FRAME_H as u32 {
                failures.push(format!(
                    "尺寸 {}x{} 与 {}x{} 不符",
                    header.width, header.height, FRAME_W, FRAME_H
                ));
            }
            // 槽间距报的是**上限帧长**(恒定), 不是当前帧长 —— 消费端靠它算槽偏移。
            if header.slot_count != 3 || header.slot_bytes != SLOT_STRIDE as u32 {
                failures.push(format!(
                    "槽布局 {}x{}B 与 3x{}B 不符",
                    header.slot_count, header.slot_bytes, SLOT_STRIDE
                ));
            }
            if header.producer_pid != std::process::id() {
                failures.push(format!(
                    "producer_pid {} 不是本进程 {}",
                    header.producer_pid,
                    std::process::id()
                ));
            }
        }
        Err(error) => failures.push(format!("头部校验: {}", error)),
    }

    // 2) 黑帧编码必须与过滤器的占位帧逐字节相同, 否则"生产者不在"与"输出黑屏"会有可见跳变。
    let black_rgb = Frame {
        width: FRAME_W,
        height: FRAME_H,
        rgb: vec![0u8; FRAME_W * FRAME_H * 3],
    };
    let mut converted = vec![0u8; nv12_len];
    match rgb24_to_nv12(&black_rgb, &mut converted) {
        Ok(()) if converted == black_nv12(FRAME_W, FRAME_H) => {
            println!("[VCAM] 黑帧编码: 与占位帧一致(Y=0 UV=128)")
        }
        Ok(()) => failures.push("黑帧 NV12 编码与占位帧不一致".to_string()),
        Err(error) => failures.push(format!("黑帧转换: {}", error)),
    }

    // 2.3) QR 生成规则必须与游戏侧实际在用的图样(仓库根 sample.jpg)一一对齐。
    //
    // 判据来自把 sample.jpg 解回模块矩阵的实测: Version 4(33x33) / 纠错 M / 掩码 0 /
    // 字母数字模式。这里用样本里那串原文再生成一遍, 核对边长与整张矩阵的 FNV-1a 校验值 ——
    // 校验值是从"与样本逐模块相同(0 个差异)"的那份矩阵上算出来的, 因此它同时锁住了模式选择、
    // 版本挑选、纠错级别与掩码这四件事; 任何一处漂移都会让本项立刻失败。
    {
        const SAMPLE_DATA: &str =
            "SGWCMAID260817033841D664FF353C56D87225240F0DFCDE1F415D2BF66CF91CDD766881215EB17A5B65";
        const SAMPLE_MODULES: usize = 33;
        const SAMPLE_FNV1A64: u64 = 0x6F6E_0981_8214_633D;
        match mai2control_ui::vcam::qr_matrix(SAMPLE_DATA) {
            Ok((size, matrix)) => {
                let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
                for dark in &matrix {
                    hash ^= u64::from(*dark);
                    hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
                }
                if size != SAMPLE_MODULES {
                    failures.push(format!(
                        "QR 边长 {} 模块(样本 {}; 版本对不上 ⇒ 模式或纠错级别已偏离)",
                        size, SAMPLE_MODULES
                    ));
                } else if hash != SAMPLE_FNV1A64 {
                    failures.push(format!(
                        "QR 模块矩阵校验 0x{:016X} 与样本 0x{:016X} 不符(掩码/纠错/模式已偏离)",
                        hash, SAMPLE_FNV1A64
                    ));
                } else {
                    println!(
                        "[VCAM] QR 规则对齐 sample.jpg: {}x{} 模块(Version 4), 纠错 M, 掩码 0, 字母数字模式, 矩阵逐格一致",
                        size, size
                    );
                }
            }
            Err(error) => failures.push(format!("QR 矩阵生成: {}", error)),
        }
    }

    // 2.4) X 镜像必须是真正的逐行对称翻转, 且翻两次回到原样(对合)。
    // 用 QR 帧而不是纯色/对称图案来验: 只有左右不对称的内容才分得出"翻了"与"没翻"。
    {
        let source = match render_qr_frame("MAI2CONTROL-VCAM-MIRROR") {
            Ok(frame) => Some(frame),
            Err(error) => {
                failures.push(format!("镜像用 QR 帧生成: {}", error));
                None
            }
        };
        if let Some(source) = source {
            let (w, h) = (source.width, source.height);
            let mut mirrored = source.clone();
            mai2control_ui::vcam::mirror_x_in_place(&mut mirrored);
            // 逐行核对: 目标第 x 列必须等于源第 (W-1-x) 列, 三个分量都对上。
            let mut mismatch = 0usize;
            for y in 0..h {
                let row = y * w * 3;
                for x in 0..w {
                    let dst = row + x * 3;
                    let src = row + (w - 1 - x) * 3;
                    if mirrored.rgb[dst..dst + 3] != source.rgb[src..src + 3] {
                        mismatch += 1;
                    }
                }
            }
            let mut twice = mirrored.clone();
            mai2control_ui::vcam::mirror_x_in_place(&mut twice);
            if mismatch != 0 {
                failures.push(format!("X 镜像有 {} 个像素不满足左右对称映射", mismatch));
            } else if twice.rgb != source.rgb {
                failures.push("X 镜像翻两次未回到原帧(非对合)".to_string());
            } else if mirrored.rgb == source.rgb {
                failures.push("X 镜像后与原帧完全相同(未生效)".to_string());
            } else {
                println!("[VCAM] X 镜像: 逐像素对称映射一致, 翻两次复原");
            }
        }
    }

    // 2.5) 测试覆盖图案: 它是"出画链路通不通"的独立判据, 自身必须先被判定为有效画面。
    // 验三件事 —— 几何不越界(渲染不 panic 且长度正确)、内容足够醒目(亮像素占比落在合理区间,
    // 既不是全黑也不是全白), 以及**会动**(不同序号必须给出不同像素: 静止图证明不了帧在更新,
    // 而"证明帧在更新"正是这个模式存在的唯一理由)。
    {
        let expected_len = FRAME_W * FRAME_H * 3;
        let first = mai2control_ui::vcam::render_test_frame(0);
        let later = mai2control_ui::vcam::render_test_frame(64);
        if first.rgb.len() != expected_len || later.rgb.len() != expected_len {
            failures.push(format!(
                "测试图案长度 {}/{}(期望 {})",
                first.rgb.len(),
                later.rgb.len(),
                expected_len
            ));
        } else {
            let bright = first.rgb.chunks_exact(3).filter(|p| p[0] > 128).count();
            let ratio = bright as f32 / (FRAME_W * FRAME_H) as f32 * 100.0;
            if !(1.0..=60.0).contains(&ratio) {
                failures.push(format!("测试图案亮像素占比 {:.1}% 不在 1%..60%", ratio));
            } else if first.rgb == later.rgb {
                failures.push("测试图案不随序号变化(游标未动, 无法证明帧在更新)".to_string());
            } else {
                println!("[VCAM] 测试图案: 亮像素 {:.1}%, 游标随序号推进", ratio);
            }
        }
    }

    // 2.6) 自定义分辨率与 QR 占比: 三件事必须成立 ——
    //   ① 非法输入被夹进范围且宽高取偶数(NV12 的色度按 2x2 取样, 奇数尺寸必然错位);
    //   ② 渲染出的帧自带尺寸且与像素长度自洽(帧自带尺寸就是为了消灭"尺寸与像素对不上");
    //   ③ 占比确实改变 QR 的像素占地面积, 且不会溢出画面。
    {
        use mai2control_ui::vcam::{
            MAX_FRAME_H, MAX_FRAME_W, MIN_FRAME_H, MIN_FRAME_W, clamp_resolution,
            render_qr_frame_sized,
        };
        // ① 夹取与偶数化。
        let cases = [
            ((1u32, 1u32), (MIN_FRAME_W, MIN_FRAME_H)),
            ((99999, 99999), (MAX_FRAME_W, MAX_FRAME_H)),
            ((641, 481), (640, 480)),
        ];
        for ((in_w, in_h), want) in cases {
            let got = clamp_resolution(in_w, in_h);
            if got != want {
                failures.push(format!(
                    "分辨率夹取 {}x{} → {:?}(期望 {:?})",
                    in_w, in_h, got, want
                ));
            }
        }
        // ② 多种分辨率下渲染都要自洽。
        for (w, h) in [(320usize, 240usize), (640, 480), (1280, 720)] {
            match render_qr_frame_sized("MAI2CONTROL-VCAM-RES", w, h, 75) {
                Ok(frame) => {
                    if frame.width != w || frame.height != h {
                        failures.push(format!(
                            "{}x{} 渲染回报尺寸 {}x{}",
                            w, h, frame.width, frame.height
                        ));
                    } else if !frame.is_consistent() {
                        failures.push(format!(
                            "{}x{} 渲染像素 {} 字节与尺寸不符(应为 {})",
                            w,
                            h,
                            frame.rgb.len(),
                            frame.expected_len()
                        ));
                    }
                }
                Err(error) => failures.push(format!("{}x{} 渲染失败: {}", w, h, error)),
            }
        }
        // ③ 占比越大, QR 的白色模块覆盖面积越大; 且始终不越出画面(长度自洽已含此意)。
        let area_of = |pct: u32| -> Option<usize> {
            render_qr_frame_sized("MAI2CONTROL-VCAM-FILL", 640, 480, pct)
                .ok()
                .map(|f| f.rgb.chunks_exact(3).filter(|p| p[0] > 128).count())
        };
        match (area_of(30), area_of(90)) {
            (Some(small), Some(large)) if large > small && small > 0 => {
                println!(
                    "[VCAM] 自定义分辨率/占比: 夹取与偶数化正确, 320x240~1280x720 渲染自洽, 占比 30%→90% 白模块 {}→{} 像素",
                    small, large
                );
            }
            (Some(small), Some(large)) => failures.push(format!(
                "QR 占比未随设置增大(30%={} 像素, 90%={} 像素)",
                small, large
            )),
            _ => failures.push("按占比渲染 QR 失败".to_string()),
        }
    }

    // 3) 发布/读取往返: 逐字节核对, 并检查 sequence 单调 +1(顺带覆盖 3 个槽的轮转)。
    let qr: Frame = match render_qr_frame("MAI2CONTROL-VCAM-PROBE") {
        Ok(frame) => frame,
        Err(error) => {
            failures.push(format!("QR 帧生成: {}", error));
            black_rgb.clone()
        }
    };
    let mut expected_sequence = 0u32;
    for round in 0..4u32 {
        // 交替黑/QR: 4 轮既覆盖 slot 0/1/2 轮转, 也覆盖"内容变化"与"内容重复"两种情况。
        let (name, rgb) = if round % 2 == 0 {
            ("QR", &qr)
        } else {
            ("黑", &black_rgb)
        };
        if let Err(error) = publisher.publish(rgb) {
            failures.push(format!("第 {} 轮 publish({}): {}", round + 1, name, error));
            break;
        }
        expected_sequence += 1;
        let mut got = vec![0u8; nv12_len];
        let sequence = match reader.read(&mut got) {
            Ok(sequence) => sequence,
            Err(error) => {
                failures.push(format!("第 {} 轮 read({}): {}", round + 1, name, error));
                break;
            }
        };
        if sequence != expected_sequence {
            failures.push(format!(
                "第 {} 轮 sequence={} 期望 {}",
                round + 1,
                sequence,
                expected_sequence
            ));
        }
        let mut expected = vec![0u8; nv12_len];
        if let Err(error) = rgb24_to_nv12(rgb, &mut expected) {
            failures.push(format!("第 {} 轮转换: {}", round + 1, error));
            break;
        }
        match got.iter().zip(expected.iter()).position(|(a, b)| a != b) {
            Some(index) => failures.push(format!(
                "第 {} 轮({})读回内容不一致: 首个差异在字节 {}",
                round + 1,
                name,
                index
            )),
            None => {
                // 亮度统计只作为"画面确实有内容"的旁证: QR 应有大量非零亮度, 黑帧必须全零。
                let luma = &got[..FRAME_W * FRAME_H];
                let bright = luma.iter().filter(|value| **value > 32).count();
                let percent = bright as f32 * 100.0 / luma.len() as f32;
                println!(
                    "[VCAM] 第 {} 轮({}) seq={} slot={} 亮像素={:.1}%",
                    round + 1,
                    name,
                    sequence,
                    sequence % 3,
                    percent
                );
                if name == "QR" && percent < 20.0 {
                    failures.push(format!("QR 帧亮像素仅 {:.1}%, 疑似未真正绘制", percent));
                }
                if name == "黑" && percent > 0.0 {
                    failures.push(format!("黑帧仍有 {:.1}% 亮像素", percent));
                }
            }
        }
    }

    // 4) 心跳: 过滤器靠 tick_ms 区分"画面没变"与"生产者进程已被杀"。
    let before = reader.header().map(|header| header.tick_ms).unwrap_or(0);
    thread::sleep(Duration::from_millis(80));
    publisher.heartbeat();
    match reader.header() {
        Ok(header) if header.tick_ms.wrapping_sub(before) > 0 => {
            println!("[VCAM] 心跳: tick {} → {}", before, header.tick_ms)
        }
        Ok(header) => failures.push(format!("心跳未推进(tick 仍为 {})", header.tick_ms)),
        Err(error) => failures.push(format!("心跳读取: {}", error)),
    }

    // 5) 正常退出必须落到 STOPPING, 消费端立刻转占位帧而不用等心跳超时。
    drop(publisher);
    match reader.header() {
        Ok(header) if header.state == QueueState::Stopping => {
            println!("[VCAM] 生产者退出: state=Stopping")
        }
        Ok(header) => failures.push(format!(
            "生产者退出后状态为 {:?}, 期望 Stopping",
            header.state
        )),
        Err(error) => failures.push(format!("退出后头部读取: {}", error)),
    }
    if reader.read(&mut vec![0u8; nv12_len]).is_ok() {
        failures.push("生产者退出后仍能取到帧(消费端应转占位帧)".to_string());
    }

    // 6) 过滤器侧：两个产物都做 PE/导出核对；当前进程位宽对应的 DLL 走真实 COM + 完整 graph。
    //    同一 selftest 分别构建为 x64 与 i686 后，两边都会覆盖自己的 in-proc 消费路径。
    let x64 = _check_vcam_pe("x64", VCAM_DLL_X64, PE_MACHINE_AMD64, &mut failures);
    let x86 = _check_vcam_pe("x86", VCAM_DLL_X86, PE_MACHINE_I386, &mut failures);
    let native = if cfg!(target_pointer_width = "64") {
        ("x64", x64)
    } else {
        ("x86", x86)
    };
    let probe_registered = match vcam_backend::is_registered() {
        Ok(installed) => installed,
        Err(error) => {
            failures.push(format!("读取系统注册状态失败: {}", error));
            false
        }
    };
    // ★建图期间必须有活生产者, 且它必须先跑在一个**非默认**分辨率上, 建图中途再改一次★
    //
    // 这一节要钉住的是"分辨率硬透传, 全链路不缩放"这条硬要求, 它有两个必须分别覆盖到的面:
    //   ① 新建针脚报出去的尺寸 = 队列头此刻的尺寸。起手就用 800x600(不是回落值 640x480),
    //      否则针脚即使根本没读队列、直接用回落值, 也照样"看起来对"。
    //   ② 已协商的连接遇到队列尺寸变化时**不缩放**, 只给占位帧。所以中途必须真的把两者错开。
    // 判据取过滤器自己写在日志里的那几行(见下面第 7 步): 它跑在消费端进程里, 是唯一的现场证据。
    const ALT_W: usize = 800;
    const ALT_H: usize = 600;
    let mut flip_publisher = match FramePublisher::create(ALT_W, ALT_H) {
        Ok(mut publisher) => {
            // 先按 800x600 发一帧: 第一轮针脚新建时读到的就是它。
            let _ = publisher.publish(&mai2control_ui::vcam::render_test_frame_sized(
                1, ALT_W, ALT_H,
            ));
            Some(publisher)
        }
        Err(error) => {
            failures.push(format!("透传覆盖: 重建生产者失败: {}", error));
            None
        }
    };
    // 交替尺寸: 第一轮(类工厂建图)针脚定在 800x600 → 换成 640x480;
    // 第二轮(注册表 CLSID 建图)针脚新建时队列已是 640x480, 于是它必须报 640x480(= 覆盖面 ①
    // 里"改过分辨率之后重开也能拿到新尺寸"这一半), 再换回 800x600 错开一次。
    let mut alternate = false;
    let mut seq = 1u32;
    let mut flip_resolution = || {
        let Some(publisher) = flip_publisher.as_mut() else {
            return;
        };
        alternate = !alternate;
        let (w, h) = if alternate {
            (FRAME_W, FRAME_H)
        } else {
            (ALT_W, ALT_H)
        };
        // 连发几帧: 过滤器按 10fps 取帧, 一帧不够保证它一定读到新尺寸。
        for _ in 0..5 {
            seq += 1;
            if let Err(error) = publisher.publish(&mai2control_ui::vcam::render_test_frame_sized(
                seq, w, h,
            )) {
                println!("[VCAM] 透传覆盖: publish {}x{} 失败: {}", w, h, error);
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(
                mai2control_ui::vcam::share::PUBLISH_INTERVAL_MS,
            ));
        }
    };
    match native {
        (label, Some(path)) => {
            println!(
                "[VCAM] {} 原生消费探针：加载并运行完整 DirectShow graph",
                label
            );
            _probe_vcam_com(&path, probe_registered, &mut flip_resolution, &mut failures);
        }
        (label, None) => failures.push(format!(
            "{} 原生构建产物缺失，无法执行 COM graph 探针",
            label
        )),
    }
    drop(flip_resolution);
    drop(flip_publisher);

    // 7) 过滤器自己的判决: 它跑在消费端进程里, 是"分辨率到底有没有原样透传"的唯一现场证据。
    //    C++ 侧的取帧代码不可能从 Rust 直接调用, 在 Rust 里另写一份对照实现只会造出第二套真相;
    //    所以判据取过滤器自己写下的那几行。
    //
    //    "尺寸透传 WxH" 只在**队列尺寸与已协商尺寸相等**时才会写出(见 vcam_queue.cpp 的 Read),
    //    因此它同时证明了两件事: 那一轮取到的是真实帧, 且协商尺寸 = 生产者尺寸。
    {
        let log = std::path::Path::new(r"C:\ProgramData\mai2control\mai2vcam_dshow.log");
        match std::fs::read_to_string(log) {
            Ok(text) => {
                let tail: Vec<&str> = text.lines().rev().take(80).collect();
                for line in tail.iter().rev() {
                    println!("[VCAM]   {}", line);
                }
                let has = |needle: &str| tail.iter().any(|line| line.contains(needle));
                // ① 起手的 800x600: 针脚若没读队列(直接用回落 640x480), 这行就不会出现。
                let first = format!("尺寸透传 {}x{}", ALT_W, ALT_H);
                if !has(&first) {
                    failures.push(format!(
                        "过滤器日志里没有「{}」: 针脚没把队列头的分辨率原样报给下游",
                        first
                    ));
                }
                // ② 改过分辨率之后新建的那一轮针脚必须拿到新尺寸 —— 这正是"卸载重建摄像头"能
                //    解决黑屏的前提。
                let second = format!("尺寸透传 {}x{}", FRAME_W, FRAME_H);
                if !has(&second) {
                    failures.push(format!(
                        "过滤器日志里没有「{}」: 生产者改过分辨率后, 新建针脚仍没有按新尺寸协商",
                        second
                    ));
                }
                // ③ 已协商的连接遇到尺寸变化时只许给占位帧, 不许缩放。
                if !has("本源不缩放") {
                    failures.push(
                        "过滤器日志里没有「本源不缩放」: 队列与协商尺寸错开时本应给占位帧, \
                         而不是缩放或按旧尺寸解释像素"
                            .to_string(),
                    );
                }
            }
            Err(error) => failures.push(format!("读取过滤器日志失败: {}", error)),
        }
    }

    if failures.is_empty() {
        println!("[VCAM] probe: PASS");
    } else {
        for reason in &failures {
            println!("[VCAM] FAIL {}", reason);
        }
        println!("[VCAM] probe: FAIL ({} 项)", failures.len());
    }
    println!("[VCAM] probe end");
    failures.is_empty()
}

/// `--vcam-consume`: **纯消费侧**排查, 不创建任何生产者。
///
/// ★为什么必须单独有这一条★ `--vcam-probe` 总是自己造一个生产者再自己消费, 所以它证明的是
/// "协议自洽", 证明不了真实场景 —— 真实场景是生产者在上位机进程、过滤器在**别的**进程(游戏 /
/// OBS / Windows 帧服务器)里。两者的差别恰好是最容易出问题的地方: 命名空间跨不跨会话、队列是
/// 活的还是被消费端钉住的孤儿、协商的分辨率与当前分辨率是否一致。
///
/// 用法: 先让上位机跑起来并**启用摄像头**, 再执行本命令。它做三件事:
///   ① 只读打开队列, 隔 1 秒采两次头部 —— 序号是否推进直接回答"生产者到底在不在发帧";
///   ② 用**注册表里那份**已部署 DLL 走真实 COM 建图并跑起来(与消费端同一条路径);
///   ③ 把过滤器自己的日志尾巴打出来 —— 它会写明"已连上生产者(seq=N)"还是"队列不可用(...)",
///      那是判定黑屏原因的唯一直接证据。
fn _run_vcam_consume() -> bool {
    use mai2control_ui::vcam::share::QueueReader;
    use mai2control_ui::vcam::backend as vcam_backend;

    println!("[VCAM] consume begin");
    let mut failures: Vec<String> = Vec::new();

    println!("[VCAM] 系统注册状态: {}", vcam_backend::registration_status());

    // ① 队列活性: 序号推进是"正在发帧"的硬判据, 单看 state 不够(孤儿队列的 state 也可能是 Ready)。
    let reader = match QueueReader::open() {
        Ok(reader) => Some(reader),
        Err(error) => {
            failures.push(format!(
                "打开共享队列失败: {}；请确认上位机已运行且已勾选「启用虚拟扫码摄像头」",
                error
            ));
            None
        }
    };
    if let Some(reader) = reader.as_ref() {
        let first = reader.header();
        std::thread::sleep(Duration::from_millis(1000));
        let second = reader.header();
        match (first, second) {
            (Ok(a), Ok(b)) => {
                println!(
                    "[VCAM] 队列头: state={:?} {}x{} slot_bytes={} pid={} seq {} → {}",
                    b.state, b.width, b.height, b.slot_bytes, b.producer_pid, a.sequence, b.sequence
                );
                if b.state != mai2control_ui::vcam::share::QueueState::Ready {
                    failures.push(format!(
                        "队列状态 {:?}(非 Ready): 生产者未在发布(上位机未启用摄像头, 或刚退出留下孤儿队列)",
                        b.state
                    ));
                } else if b.sequence == a.sequence {
                    failures.push(format!(
                        "队列序号 1 秒内未推进(恒为 {}): 生产者没有在发帧",
                        b.sequence
                    ));
                } else {
                    println!(
                        "[VCAM] 生产者在发帧: 1 秒内推进 {} 帧(定频 10fps)",
                        b.sequence.wrapping_sub(a.sequence)
                    );
                }
            }
            (Err(error), _) | (_, Err(error)) => {
                failures.push(format!("读取队列头失败: {}", error))
            }
        }
    }

    // ② 走消费端真实路径: 用注册表里那份已部署 DLL 建图并跑起来。
    // 本进程不持有生产者, 因此这一步等价于"别的进程来取帧"。
    let installed = vcam_backend::is_registered().unwrap_or(false);
    if !installed {
        failures.push(
            "系统未完整注册本摄像头: 先在虚拟摄像头页点「安装(需管理员)」".to_string(),
        );
    } else {
        let native = if cfg!(target_pointer_width = "64") {
            ("x64", VCAM_DLL_X64)
        } else {
            ("x86", VCAM_DLL_X86)
        };
        match _check_vcam_pe(native.0, native.1, if cfg!(target_pointer_width = "64") { PE_MACHINE_AMD64 } else { PE_MACHINE_I386 }, &mut failures) {
            Some(path) => {
                println!("[VCAM] {} 消费探针: 走注册表 CLSID 建图取帧", native.0);
                // 纯消费侧: 生产者在别的进程里, 这里不碰它的状态。
                _probe_vcam_com(&path, true, &mut || {}, &mut failures);
            }
            None => failures.push(format!("{} 构建产物缺失, 无法建图", native.0)),
        }
    }

    // ③ 过滤器自己的判决: 这一步的输出比上面任何断言都直接。
    let log = std::path::Path::new(r"C:\ProgramData\mai2control\mai2vcam_dshow.log");
    println!("[VCAM] ── 过滤器日志尾部 ──");
    match std::fs::read_to_string(log) {
        Ok(text) => {
            let lines: Vec<&str> = text.lines().collect();
            for line in lines.iter().rev().take(12).rev() {
                println!("[VCAM]   {}", line);
            }
            // 只看本次建图之后新增的那几行里有没有"已连上生产者"。
            let connected = lines
                .iter()
                .rev()
                .take(12)
                .any(|line| line.contains("已连上生产者"));
            let unusable = lines
                .iter()
                .rev()
                .take(12)
                .any(|line| line.contains("队列不可用") || line.contains("生产者不在"));
            if connected {
                println!("[VCAM] 过滤器判决: 已取到真实帧(非占位)");
            } else if unusable {
                failures.push(
                    "过滤器判决为占位帧: 见上面日志里的 state/心跳滞后/协商尺寸 —— \
                     尺寸不符就在消费端重开摄像头, state=3 就是生产者没在发"
                        .to_string(),
                );
            } else {
                println!("[VCAM] 过滤器日志中未见本次判决(可能被节流未重复记录)");
            }
        }
        Err(error) => println!("[VCAM]   读取失败: {}", error),
    }

    if failures.is_empty() {
        println!("[VCAM] consume: PASS");
    } else {
        for reason in &failures {
            println!("[VCAM] FAIL {}", reason);
        }
        println!("[VCAM] consume: FAIL ({} 项)", failures.len());
    }
    println!("[VCAM] consume end");
    failures.is_empty()
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

#[derive(Debug)]
struct BusEnvelope {
    msg_id: u8,
    seq: u16,
    total_len: u16,
    frag_off: u16,
    flags: u8,
    /// 设备端组帧时刻 time_us_32()(线上头 off 10..13), 与 host_cmd 尾戳/TELEM_DATA 同源。
    t_us: u32,
    payload: Vec<u8>,
}

const BUS_MAGIC: u8 = 0xB5;
/// 与固件 bus_types.h 的 BUS_LEN_UNKNOWN 一致: 流式/推送的总长未知哨兵值。
const BUS_LEN_UNKNOWN: u16 = 0xFFFF;
const BUS_MSG_LED_SET: u8 = 0x01;
const BUS_MSG_LED_STATE: u8 = 0x02;
const BUS_FLAG_FIRST: u8 = 0x01;
const BUS_FLAG_LAST: u8 = 0x02;
const BUS_FLAG_STREAM: u8 = 0x04;
const BUS_FLAG_NAK: u8 = 0x20;
const BUS_LED_STATE_FLAG_ERROR: u8 = 0x01;
const BUS_POLL_INTERVAL_MS: u64 = 50;
const BUS_POLL_ATTEMPTS: usize = 20;
/// 与固件 bus_core.h 同源的容量常量: 重组暂存上限 / 单片 MTU / 推送环容量 / 子环池槽数。
const BUS_ASM_MAX: u16 = 1024;
const BUS_INLINE_MAX: usize = 64;
const BUS_PUSH_RING_CAP: usize = 8;
/// 与固件 bus_core.h 同源: 头加 t_us 后 BusEnv 96B, 为守住 12KB 硬上限子环池由 4 降为 3。
const BUS_SUBRING_SLOTS: usize = 3;
/// 线上信封头字节数(与固件 BUS_HDR_SIZE 同源): 12 → 16(插入 t_us(u32 LE) @ off 10..13)。
const BUS_HDR_SIZE: usize = 16;
/// 头内被 CRC 覆盖的字节数(off 1..13 = msg_id..t_us), 与固件 BUS_CRC_SPAN 同源。
const BUS_CRC_SPAN: usize = 13;
/// 未被任何服务订阅的自测 msg_id: 0x10 走分片重组, 0x11 由固件自测钩子当流式通道用。
const BUS_MSG_PROBE_ASM: u8 = 0x10;
const BUS_MSG_PROBE_STREAM: u8 = 0x11;
/// 自测钩子请求码(BUS_XFER 请求 payload 恰好 1 字节时生效, 见 bus_usb_link.cpp 文件头注释)。
const BUS_PROBE_STAT: u8 = 0x01;
const BUS_PROBE_STREAM: u8 = 0x02;
const BUS_PROBE_SUBRING: u8 = 0x03;
const BUS_STAT_FIELDS: usize = 9;
/// 一次排空动作的轮询轮数(每轮一次空 BUS_XFER + 50ms 间隔)。
const BUS_DRAIN_ROUNDS: usize = 6;

/// 固件 BusStat 的主机侧镜像。字段序与 bus_usb_link.cpp:_probe(BUS_XFER_PROBE_STAT) 写死一致。
#[derive(Debug, Clone, Copy, Default)]
struct BusStatSnapshot {
    tx_frag: u32,
    rx_frag: u32,
    rx_drop_crc: u32,
    rx_drop_full: u32,
    retrans: u32,
    timeout: u32,
    deliver: u32,
    svc_overwrite: u32,
    push_overwrite: u32,
}

/// CRC 覆盖 = 头 off[1..1+BUS_CRC_SPAN)(msg_id..t_us) + payload; 不含 magic 与 crc 自身。
fn _bus_crc(frame: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for byte in frame[1..1 + BUS_CRC_SPAN]
        .iter()
        .chain(frame[BUS_HDR_SIZE..].iter())
    {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = if (crc & 0x8000) != 0 {
                crc.wrapping_shl(1) ^ 0x1021
            } else {
                crc.wrapping_shl(1)
            };
        }
    }
    crc
}

/// 通用线上信封构帧(16B 头 + payload, CRC 现算)。LED_SET 与后续分片/超长/坏 CRC 子项
/// 全部复用这一条构帧路径, 不另开第二套。
/// 主机→设备方向的 t_us 填 0: 主机时钟对设备无意义, 设备侧也不解释入向 t_us(只是原样搬运)。
fn _bus_make_frame(
    msg_id: u8,
    seq: u16,
    total_len: u16,
    frag_off: u16,
    flags: u8,
    payload: &[u8],
) -> Vec<u8> {
    let mut frame = Vec::with_capacity(BUS_HDR_SIZE + payload.len());
    frame.push(BUS_MAGIC);
    frame.push(msg_id);
    frame.extend_from_slice(&seq.to_le_bytes());
    frame.extend_from_slice(&total_len.to_le_bytes());
    frame.extend_from_slice(&frag_off.to_le_bytes());
    frame.push(payload.len() as u8);
    frame.push(flags);
    frame.extend_from_slice(&0u32.to_le_bytes()); // t_us(off 10..13)
    frame.extend_from_slice(&[0u8; 2]); // crc 占位(off 14..15)
    frame.extend_from_slice(payload);
    let crc = _bus_crc(&frame);
    frame[BUS_HDR_SIZE - 2..BUS_HDR_SIZE].copy_from_slice(&crc.to_le_bytes());
    frame
}

fn _bus_make_led_set(seq: u16, rgb: &[u8]) -> Vec<u8> {
    _bus_make_frame(
        BUS_MSG_LED_SET,
        seq,
        rgb.len() as u16,
        0,
        BUS_FLAG_FIRST | BUS_FLAG_LAST,
        rgb,
    )
}

/// 原有严格口径: 只接受"整条消息就一片"的帧(LED 回读专用)。
fn _bus_parse_xfer(response: &[u8]) -> Result<(u8, u8, Vec<BusEnvelope>), String> {
    _bus_parse_xfer_ex(response, true)
}

/// `strict_single=false` 时放宽单分片约束: 多片流式帧(frag_off 递增)与 NAK 信封都要能收。
/// 头部 4B 解析与逐帧 CRC 校验两种口径完全共用, 不重写第二套。
fn _bus_parse_xfer_ex(
    response: &[u8],
    strict_single: bool,
) -> Result<(u8, u8, Vec<BusEnvelope>), String> {
    if response.len() < 4 {
        return Err(format!("BUS_XFER 响应过短: {}B", response.len()));
    }
    let (accepted, rejected, frame_count) = (response[0], response[1], response[2]);
    let mut offset = 4usize;
    let mut frames = Vec::with_capacity(frame_count as usize);
    while offset < response.len() {
        if response.len() - offset < BUS_HDR_SIZE {
            return Err(format!(
                "线上帧头截断: offset={} len={}",
                offset,
                response.len()
            ));
        }
        if response[offset] != BUS_MAGIC {
            return Err(format!("线上 magic 错误: 0x{:02X}", response[offset]));
        }
        let frag_len = response[offset + 8] as usize;
        let frame_len = BUS_HDR_SIZE + frag_len;
        if response.len() - offset < frame_len {
            return Err(format!(
                "线上帧载荷截断: offset={} frag_len={}",
                offset, frag_len
            ));
        }
        let frame = &response[offset..offset + frame_len];
        let total_len = u16::from_le_bytes([frame[4], frame[5]]);
        let frag_off = u16::from_le_bytes([frame[6], frame[7]]);
        // 单分片校验: 偏移必须为 0。总长有两种合法形态 ——
        //   非流式: total_len == frag_len(整条消息就这一片);
        //   流式/推送: total_len == BUS_LEN_UNKNOWN(0xFFFF), 长度本就未知, 不能拿 frag_len 去比。
        let stream = (frame[9] & BUS_FLAG_STREAM) != 0;
        let total_ok = if stream {
            total_len == BUS_LEN_UNKNOWN || total_len == frag_len as u16
        } else {
            total_len == frag_len as u16
        };
        if strict_single && (frag_off != 0 || !total_ok) {
            return Err(format!(
                "非完整单分片: stream={} total={} off={} frag={}",
                stream, total_len, frag_off, frag_len
            ));
        }
        let actual_crc = u16::from_le_bytes([frame[14], frame[15]]);
        let expected_crc = _bus_crc(frame);
        if actual_crc != expected_crc {
            return Err(format!(
                "线上 CRC 错误: expected=0x{:04X} actual=0x{:04X}",
                expected_crc, actual_crc
            ));
        }
        let envelope = BusEnvelope {
            msg_id: frame[1],
            seq: u16::from_le_bytes([frame[2], frame[3]]),
            total_len,
            frag_off,
            flags: frame[9],
            t_us: u32::from_le_bytes([frame[10], frame[11], frame[12], frame[13]]),
            payload: frame[BUS_HDR_SIZE..].to_vec(),
        };
        println!(
            "[BUS] 收到信封: msg_id=0x{:02X} seq={} flags=0x{:02X} frag={}B t_us={} (设备端 time_us_32)",
            envelope.msg_id, envelope.seq, envelope.flags, frag_len, envelope.t_us
        );
        frames.push(envelope);
        offset += frame_len;
    }
    if frames.len() != frame_count as usize {
        return Err(format!(
            "frames_out 不匹配: header={} actual={}",
            frame_count,
            frames.len()
        ));
    }
    Ok((accepted, rejected, frames))
}

fn _bus_send_led_set(ctrl: &mut AppController, seq: u16, rgb: &[u8]) -> Result<(), String> {
    let response = ctrl
        .bus_xfer(
            _bus_make_led_set(seq, rgb),
            Duration::from_millis(BUS_POLL_INTERVAL_MS),
        )
        .map_err(|error| format!("LED_SET BUS_XFER 失败: {}", error))?;
    let (accepted, rejected, _) = _bus_parse_xfer(&response)?;
    if accepted != 1 || rejected != 0 {
        return Err(format!(
            "LED_SET 未受理: accepted={} rejected={}",
            accepted, rejected
        ));
    }
    Ok(())
}

fn _bus_poll_led_state(ctrl: &mut AppController) -> Result<BusEnvelope, String> {
    for _ in 0..BUS_POLL_ATTEMPTS {
        thread::sleep(Duration::from_millis(BUS_POLL_INTERVAL_MS));
        let response = ctrl
            .bus_xfer(Vec::new(), Duration::from_millis(BUS_POLL_INTERVAL_MS))
            .map_err(|error| format!("空 BUS_XFER 轮询失败: {}", error))?;
        let (_, rejected, frames) = _bus_parse_xfer(&response)?;
        if rejected != 0 {
            return Err(format!("空 BUS_XFER 被拒绝: rejected={}", rejected));
        }
        if let Some(frame) = frames
            .into_iter()
            .find(|frame| frame.msg_id == BUS_MSG_LED_STATE)
        {
            return Ok(frame);
        }
    }
    Err(format!(
        "{} 次空 BUS_XFER 轮询未收到 LED_STATE",
        BUS_POLL_ATTEMPTS
    ))
}

fn _bus_validate_led_state(
    state: &BusEnvelope,
    rgb: [u8; 3],
    expect_error: bool,
) -> Result<(), String> {
    if state.msg_id != BUS_MSG_LED_STATE {
        return Err(format!("msg_id 错误: 0x{:02X}", state.msg_id));
    }
    if (state.flags & BUS_FLAG_STREAM) == 0 {
        return Err(format!("LED_STATE 非 STREAM: flags=0x{:02X}", state.flags));
    }
    if state.payload.len() != 4 {
        return Err(format!(
            "LED_STATE payload 长度错误: {}",
            state.payload.len()
        ));
    }
    if state.payload[0..3] != rgb {
        return Err(format!(
            "LED_STATE RGB 错误: expected={:02X?} actual={:02X?}",
            rgb,
            &state.payload[0..3]
        ));
    }
    let has_error = (state.payload[3] & BUS_LED_STATE_FLAG_ERROR) != 0;
    if has_error != expect_error {
        return Err(format!(
            "LED_STATE 错误标志错误: expected={} actual=0x{:02X}",
            expect_error, state.payload[3]
        ));
    }
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// --bus 扩展子项的公共通道: 一次 BUS_XFER 喂入任意信封流 / 排空出向 / 读 BusStat /
// 触发固件自测钩子。全部复用 _bus_make_frame + _bus_parse_xfer_ex, 不另开构帧或解析路径。
// ─────────────────────────────────────────────────────────────────────────────

/// 喂入一段(可含多帧的)信封流, 返回 (accepted, rejected, 顺带排空到的出向帧)。
fn _bus_send_raw(
    ctrl: &mut AppController,
    payload: Vec<u8>,
) -> Result<(u8, u8, Vec<BusEnvelope>), String> {
    let response = ctrl
        .bus_xfer(payload, Duration::from_millis(BUS_POLL_INTERVAL_MS * 4))
        .map_err(|error| format!("BUS_XFER 失败: {}", error))?;
    _bus_parse_xfer_ex(&response, false)
}

/// 固定轮数排空出向帧(空 payload = 纯轮询)。返回本次收到的全部帧。
fn _bus_drain(ctrl: &mut AppController, rounds: usize) -> Result<Vec<BusEnvelope>, String> {
    let mut collected = Vec::new();
    for _ in 0..rounds {
        thread::sleep(Duration::from_millis(BUS_POLL_INTERVAL_MS));
        let response = ctrl
            .bus_xfer(Vec::new(), Duration::from_millis(BUS_POLL_INTERVAL_MS * 4))
            .map_err(|error| format!("空 BUS_XFER 轮询失败: {}", error))?;
        let (_, rejected, frames) = _bus_parse_xfer_ex(&response, false)?;
        if rejected != 0 {
            return Err(format!("空 BUS_XFER 被拒绝: rejected={}", rejected));
        }
        collected.extend(frames);
    }
    Ok(collected)
}

/// 触发一个自测钩子(payload 恰好 1 字节)并取回响应体。
/// 头部 3 字节必须为 0 —— 钩子不喂入信封、不排空出向队列, 正常数据路径不受影响。
fn _bus_probe(ctrl: &mut AppController, code: u8) -> Result<Vec<u8>, String> {
    let response = ctrl
        .bus_xfer(vec![code], Duration::from_millis(BUS_POLL_INTERVAL_MS * 8))
        .map_err(|error| format!("自测钩子 0x{:02X} BUS_XFER 失败: {}", code, error))?;
    if response.len() < 4 {
        return Err(format!(
            "自测钩子 0x{:02X} 响应过短: {}B",
            code,
            response.len()
        ));
    }
    if response[0] != 0 || response[1] != 0 || response[2] != 0 {
        return Err(format!(
            "自测钩子 0x{:02X} 头部应为 accepted/rejected/frames_out 全 0, 实际 {:02X?}",
            code,
            &response[0..3]
        ));
    }
    if response.len() == 4 {
        return Err(format!(
            "自测钩子 0x{:02X} 响应体为空: 设备固件不支持该钩子",
            code
        ));
    }
    Ok(response[4..].to_vec())
}

fn _bus_stat(ctrl: &mut AppController) -> Result<BusStatSnapshot, String> {
    let body = _bus_probe(ctrl, BUS_PROBE_STAT)?;
    if body.len() != BUS_STAT_FIELDS * 4 {
        return Err(format!(
            "BusStat 响应体长度错误: 期望 {}B 实际 {}B",
            BUS_STAT_FIELDS * 4,
            body.len()
        ));
    }
    let field = |i: usize| -> u32 {
        u32::from_le_bytes([
            body[i * 4],
            body[i * 4 + 1],
            body[i * 4 + 2],
            body[i * 4 + 3],
        ])
    };
    Ok(BusStatSnapshot {
        tx_frag: field(0),
        rx_frag: field(1),
        rx_drop_crc: field(2),
        rx_drop_full: field(3),
        retrans: field(4),
        timeout: field(5),
        deliver: field(6),
        svc_overwrite: field(7),
        push_overwrite: field(8),
    })
}

/// 子项 10: BusStat 可读性。两次读数必须逐字段单调不减(计数器只增), 否则后面所有增量断言都不可信。
fn _bus_step_stat(ctrl: &mut AppController) -> Result<BusStatSnapshot, String> {
    let first = _bus_stat(ctrl)?;
    let second = _bus_stat(ctrl)?;
    let pairs: [(&str, u32, u32); BUS_STAT_FIELDS] = [
        ("tx_frag", first.tx_frag, second.tx_frag),
        ("rx_frag", first.rx_frag, second.rx_frag),
        ("rx_drop_crc", first.rx_drop_crc, second.rx_drop_crc),
        ("rx_drop_full", first.rx_drop_full, second.rx_drop_full),
        ("retrans", first.retrans, second.retrans),
        ("timeout", first.timeout, second.timeout),
        ("deliver", first.deliver, second.deliver),
        ("svc_overwrite", first.svc_overwrite, second.svc_overwrite),
        (
            "push_overwrite",
            first.push_overwrite,
            second.push_overwrite,
        ),
    ];
    for (name, before, after) in pairs {
        if after < before {
            return Err(format!("BusStat.{} 倒退: {} → {}", name, before, after));
        }
    }
    println!(
        "[BUS] stat PASS: tx={} rx={} drop_crc={} drop_full={} retrans={} timeout={} deliver={} svc_ovr={} push_ovr={}",
        second.tx_frag,
        second.rx_frag,
        second.rx_drop_crc,
        second.rx_drop_full,
        second.retrans,
        second.timeout,
        second.deliver,
        second.svc_overwrite,
        second.push_overwrite
    );
    Ok(second)
}

/// 子项 5 前半: 一条 200B 消息拆 64/64/64/8 四片依序喂入, 校验全部受理且无丢帧计数增长。
fn _bus_step_fragment(ctrl: &mut AppController, before: &BusStatSnapshot) -> Result<(), String> {
    const TOTAL: u16 = 200;
    let seq = 0x2000u16;
    let body: Vec<u8> = (0..TOTAL as usize).map(|i| (i & 0xFF) as u8).collect();
    let lens = [BUS_INLINE_MAX, BUS_INLINE_MAX, BUS_INLINE_MAX, 8usize];
    let mut stream = Vec::new();
    let mut off = 0usize;
    for (index, len) in lens.iter().enumerate() {
        let mut flags = 0u8;
        if index == 0 {
            flags |= BUS_FLAG_FIRST;
        }
        if index + 1 == lens.len() {
            flags |= BUS_FLAG_LAST;
        }
        stream.extend(_bus_make_frame(
            BUS_MSG_PROBE_ASM,
            seq,
            TOTAL,
            off as u16,
            flags,
            &body[off..off + len],
        ));
        off += len;
    }
    if off != TOTAL as usize {
        return Err(format!("构造分片总长错误: {}", off));
    }

    let (accepted, rejected, _) = _bus_send_raw(ctrl, stream)?;
    if accepted != lens.len() as u8 || rejected != 0 {
        return Err(format!(
            "分片未全部受理: accepted={} rejected={} (期望 accepted={})",
            accepted,
            rejected,
            lens.len()
        ));
    }
    thread::sleep(Duration::from_millis(BUS_POLL_INTERVAL_MS * 2));
    let after = _bus_stat(ctrl)?;
    if after.rx_frag < before.rx_frag + lens.len() as u32 {
        return Err(format!(
            "rx_frag 未按分片数增长: {} → {} (期望 +{})",
            before.rx_frag,
            after.rx_frag,
            lens.len()
        ));
    }
    if after.rx_drop_crc != before.rx_drop_crc || after.rx_drop_full != before.rx_drop_full {
        return Err(format!(
            "顺序分片竟被记为丢帧: drop_crc {} → {} drop_full {} → {}",
            before.rx_drop_crc, after.rx_drop_crc, before.rx_drop_full, after.rx_drop_full
        ));
    }
    println!(
        "[BUS] frag-reassembly PASS: 4 片(64/64/64/8) 全受理 rx_frag {} → {} 丢帧计数未增长",
        before.rx_frag, after.rx_frag
    );
    Ok(())
}

/// 子项 5 后半: frag_off 不接续必须回 NAK 信封(flags bit5), 不能静默吞掉。
fn _bus_step_fragment_gap(ctrl: &mut AppController) -> Result<(), String> {
    let seq = 0x2001u16;
    let chunk = [0x5Au8; BUS_INLINE_MAX];
    let mut stream = _bus_make_frame(BUS_MSG_PROBE_ASM, seq, 200, 0, BUS_FLAG_FIRST, &chunk);
    // 期望偏移是 64, 故意写 128 → 固件必须弃槽 + NAK。
    stream.extend(_bus_make_frame(BUS_MSG_PROBE_ASM, seq, 200, 128, 0, &chunk));
    let (accepted, rejected, mut frames) = _bus_send_raw(ctrl, stream)?;
    if accepted != 2 || rejected != 0 {
        return Err(format!(
            "失序分片的线上受理异常: accepted={} rejected={} (两帧都应通过 CRC 校验)",
            accepted, rejected
        ));
    }
    frames.extend(_bus_drain(ctrl, BUS_DRAIN_ROUNDS)?);
    let nak = frames.iter().find(|frame| {
        frame.msg_id == BUS_MSG_PROBE_ASM && (frame.flags & BUS_FLAG_NAK) != 0 && frame.seq == seq
    });
    match nak {
        Some(frame) => {
            println!(
                "[BUS] frag-gap-nak PASS: frag_off 不接续 → NAK 信封 msg_id=0x{:02X} seq={} flags=0x{:02X}",
                frame.msg_id, frame.seq, frame.flags
            );
            Ok(())
        }
        None => Err(format!(
            "frag_off 不接续未收到 NAK: 收到 {} 帧 {:?}",
            frames.len(),
            frames
                .iter()
                .map(|f| (f.msg_id, f.seq, f.flags))
                .collect::<Vec<_>>()
        )),
    }
}

/// 子项 6: total_len 超 BUS_ASM_MAX 必须显式拒绝(NAK), 不允许静默截断后照收。
fn _bus_step_oversize(ctrl: &mut AppController, before: &BusStatSnapshot) -> Result<(), String> {
    let seq = 0x2002u16;
    let chunk = [0x33u8; BUS_INLINE_MAX];
    let frame = _bus_make_frame(BUS_MSG_PROBE_ASM, seq, 2000, 0, BUS_FLAG_FIRST, &chunk);
    let (_, _, mut frames) = _bus_send_raw(ctrl, frame)?;
    frames.extend(_bus_drain(ctrl, BUS_DRAIN_ROUNDS)?);
    let nak = frames
        .iter()
        .any(|f| f.msg_id == BUS_MSG_PROBE_ASM && (f.flags & BUS_FLAG_NAK) != 0 && f.seq == seq);
    if !nak {
        return Err(format!(
            "声明 total_len=2000(>{}) 未被显式拒绝: 收到 {:?}",
            BUS_ASM_MAX,
            frames
                .iter()
                .map(|f| (f.msg_id, f.seq, f.flags))
                .collect::<Vec<_>>()
        ));
    }
    let after = _bus_stat(ctrl)?;
    // 静默截断的表征 = 它被当成一条正常消息派发出去了。deliver 不许增长。
    if after.deliver != before.deliver {
        return Err(format!(
            "超长消息竟被派发(疑似静默截断): deliver {} → {}",
            before.deliver, after.deliver
        ));
    }
    println!(
        "[BUS] oversize PASS: total_len=2000 > BUS_ASM_MAX={} 回 NAK, deliver 未增长({})",
        BUS_ASM_MAX, after.deliver
    );
    Ok(())
}

/// 子项 7: 坏 CRC 必须被线上层拒绝(rejected+1, rx_drop_crc+1)且不产生任何派发。
fn _bus_step_bad_crc(ctrl: &mut AppController, before: &BusStatSnapshot) -> Result<(), String> {
    let mut frame = _bus_make_led_set(0x30, &[7, 7, 7]);
    frame[BUS_HDR_SIZE - 2] ^= 0xFF; // 只翻 CRC 低字节(off 14), 其余字段保持完全合法
    let (accepted, rejected, _) = _bus_send_raw(ctrl, frame)?;
    if accepted != 0 || rejected != 1 {
        return Err(format!(
            "坏 CRC 帧未被拒绝: accepted={} rejected={}",
            accepted, rejected
        ));
    }
    thread::sleep(Duration::from_millis(BUS_POLL_INTERVAL_MS * 2));
    let after = _bus_stat(ctrl)?;
    if after.rx_drop_crc != before.rx_drop_crc + 1 {
        return Err(format!(
            "rx_drop_crc 未按坏帧数增长: {} → {} (期望 +1)",
            before.rx_drop_crc, after.rx_drop_crc
        ));
    }
    if after.deliver != before.deliver {
        return Err(format!(
            "坏 CRC 帧竟产生派发: deliver {} → {}",
            before.deliver, after.deliver
        ));
    }
    let _ = _bus_drain(ctrl, 2)?; // 顺手排掉这条坏帧引出的 NAK 信封
    println!(
        "[BUS] bad-crc PASS: rejected=1 rx_drop_crc {} → {} deliver 未增长({})",
        before.rx_drop_crc, after.rx_drop_crc, after.deliver
    );
    Ok(())
}

/// 子项 8: 推送环溢出 → 计数跳号可见。★"丢包可见而非静默丢"的核心判据★
fn _bus_step_push_overflow(ctrl: &mut AppController) -> Result<(), String> {
    let _ = _bus_drain(ctrl, BUS_DRAIN_ROUNDS)?;
    // 先拿一个已知的基准 seq: 没有它, 环丢掉最旧几条后剩下的序列本身是连续的, 跳号无从对比。
    _bus_send_led_set(ctrl, 0x40, &[2, 4, 6])?;
    let baseline = _bus_drain(ctrl, BUS_DRAIN_ROUNDS)?
        .into_iter()
        .filter(|f| f.msg_id == BUS_MSG_LED_STATE)
        .next_back()
        .ok_or_else(|| "基准 LED_STATE 未收到".to_string())?;

    let before = _bus_stat(ctrl)?;
    // 12 条合法 LED_SET 打进同一次 BUS_XFER: 响应在 task() 之前就返回, 期间没有任何排空机会,
    // 因此 12 条 LED_STATE 全部在同一轮 task() 里推入容量 8 的 _push_out → 必然覆盖 4 条。
    let burst = 12usize;
    let mut stream = Vec::new();
    for i in 0..burst {
        stream.extend(_bus_make_led_set(0x50 + i as u16, &[i as u8, 0x11, 0x22]));
    }
    let (accepted, rejected, _) = _bus_send_raw(ctrl, stream)?;
    if accepted != burst as u8 || rejected != 0 {
        return Err(format!(
            "突发 LED_SET 未全部受理: accepted={} rejected={} (期望 {})",
            accepted, rejected, burst
        ));
    }

    let states: Vec<BusEnvelope> = _bus_drain(ctrl, BUS_DRAIN_ROUNDS)?
        .into_iter()
        .filter(|f| f.msg_id == BUS_MSG_LED_STATE)
        .collect();
    if states.is_empty() {
        return Err("突发后未收到任何 LED_STATE".to_string());
    }
    if states.len() > BUS_PUSH_RING_CAP {
        return Err(format!(
            "推送环容量应为 {}, 却收到 {} 帧",
            BUS_PUSH_RING_CAP,
            states.len()
        ));
    }
    let after = _bus_stat(ctrl)?;
    if after.push_overwrite <= before.push_overwrite {
        return Err(format!(
            "推送环未记录覆盖: push_overwrite {} → {}",
            before.push_overwrite, after.push_overwrite
        ));
    }
    // 跳号判定一律用 wrapping_sub 增量, 对 u16 回绕天然成立。
    let mut previous = baseline.seq;
    let mut gaps = Vec::new();
    for state in &states {
        let delta = state.seq.wrapping_sub(previous);
        if delta > 1 {
            gaps.push((previous, state.seq, delta));
        }
        previous = state.seq;
    }
    if gaps.is_empty() {
        return Err(format!(
            "推送环溢出后 seq 竟连续(丢包不可见): baseline={} seqs={:?}",
            baseline.seq,
            states.iter().map(|f| f.seq).collect::<Vec<_>>()
        ));
    }
    println!(
        "[BUS] push-overflow PASS: 发 {} 收 {} 帧, push_overwrite {} → {}, 跳号 {:?}",
        burst,
        states.len(),
        before.push_overwrite,
        after.push_overwrite,
        gaps
    );
    Ok(())
}

/// 子项 9: 计数回绕。65536 次真跑无意义, 只需断言 wrapping_sub 的增量语义跨回绕点成立。
fn _bus_step_wrap() -> Result<(), String> {
    let series: [u16; 4] = [0xFFFE, 0xFFFF, 0x0000, 0x0001];
    for pair in series.windows(2) {
        let delta = pair[1].wrapping_sub(pair[0]);
        if delta != 1 {
            return Err(format!(
                "回绕点相邻计数增量应为 1: {} → {} delta={}",
                pair[0], pair[1], delta
            ));
        }
    }
    // 跨回绕的跳号同样必须表现为 delta > 1(否则丢包在回绕点会被误判成连续)。
    let skipped = series[3].wrapping_sub(series[1]);
    if skipped != 2 {
        return Err(format!("跨回绕跳号增量应为 2, 实际 {}", skipped));
    }
    // 回退(乱序/重放)在回绕点必须表现为一个巨大的增量, 而不是负数溢出成 1。
    let backwards = series[0].wrapping_sub(series[2]);
    if backwards != 0xFFFE {
        return Err(format!(
            "回绕点回退增量应为 0xFFFE, 实际 0x{:04X}",
            backwards
        ));
    }
    println!("[BUS] wrap PASS: 0xFFFE→0xFFFF→0x0000→0x0001 增量恒为 1, 跳号=2, 回退=0xFFFE");
    Ok(())
}

/// 子项 11: 流式通道。固件自测钩子自己 open/write×3/close, 主机排空校验帧形态。
fn _bus_step_stream(ctrl: &mut AppController) -> Result<(), String> {
    let _ = _bus_drain(ctrl, 2)?;
    let body = _bus_probe(ctrl, BUS_PROBE_STREAM)?;
    if body.len() != 5 {
        return Err(format!("流式钩子响应体长度错误: {}B (期望 5)", body.len()));
    }
    let names = [
        "stream_open",
        "write#0",
        "write#1",
        "write#2",
        "stream_close",
    ];
    for (index, name) in names.iter().enumerate() {
        if body[index] != 1 {
            return Err(format!("固件侧 {} 失败", name));
        }
    }

    let frames: Vec<BusEnvelope> = _bus_drain(ctrl, BUS_DRAIN_ROUNDS)?
        .into_iter()
        .filter(|f| f.msg_id == BUS_MSG_PROBE_STREAM)
        .collect();
    let data: Vec<&BusEnvelope> = frames.iter().filter(|f| !f.payload.is_empty()).collect();
    let closes: Vec<&BusEnvelope> = frames
        .iter()
        .filter(|f| f.payload.is_empty() && (f.flags & BUS_FLAG_LAST) != 0)
        .collect();
    if data.len() != 3 {
        return Err(format!(
            "流式数据帧数错误: 期望 3 实际 {} (全部帧 {:?})",
            data.len(),
            frames
                .iter()
                .map(|f| (f.seq, f.frag_off, f.payload.len(), f.flags))
                .collect::<Vec<_>>()
        ));
    }
    if closes.len() != 1 {
        return Err(format!("流式收尾帧数错误: 期望 1 实际 {}", closes.len()));
    }
    let mut expect_off = 0u16;
    for (index, frame) in data.iter().enumerate() {
        if (frame.flags & BUS_FLAG_STREAM) == 0 {
            return Err(format!(
                "流式帧 #{} 未置 STREAM: flags=0x{:02X}",
                index, frame.flags
            ));
        }
        if frame.total_len != BUS_LEN_UNKNOWN {
            return Err(format!(
                "流式帧 #{} total_len 应为 0x{:04X}, 实际 0x{:04X}",
                index, BUS_LEN_UNKNOWN, frame.total_len
            ));
        }
        if frame.frag_off != expect_off {
            return Err(format!(
                "流式帧 #{} frag_off 应为 {}, 实际 {}",
                index, expect_off, frame.frag_off
            ));
        }
        // 同一条流的全部分片共享 open 时取的那一个 seq(bus_core.cpp:248-262),
        // 因此这里必须断言"seq 恒等 + frag_off 递增", 而不是 seq 递增。
        if frame.seq != data[0].seq {
            return Err(format!(
                "同一条流的 seq 应恒等: #{} seq={} 首片 seq={}",
                index, frame.seq, data[0].seq
            ));
        }
        expect_off = expect_off.wrapping_add(frame.payload.len() as u16);
    }
    if (data[0].flags & BUS_FLAG_FIRST) == 0 {
        return Err(format!("流式首片未置 FIRST: flags=0x{:02X}", data[0].flags));
    }
    if closes[0].seq != data[0].seq || closes[0].frag_off != expect_off {
        return Err(format!(
            "收尾帧不接续: seq={} off={} (期望 seq={} off={})",
            closes[0].seq, closes[0].frag_off, data[0].seq, expect_off
        ));
    }
    println!(
        "[BUS] stream PASS: msg_id=0x{:02X} seq={} 3 片 frag_off=0/8/16 total_len=0x{:04X} + LAST 收尾帧",
        BUS_MSG_PROBE_STREAM, data[0].seq, BUS_LEN_UNKNOWN
    );
    Ok(())
}

/// 子项 12: 子环池。★必须验证第 5 次 acquire 失败(容量硬上限)★
fn _bus_step_subring(ctrl: &mut AppController) -> Result<(), String> {
    let body = _bus_probe(ctrl, BUS_PROBE_SUBRING)?;
    if body.len() != BUS_SUBRING_SLOTS + 3 {
        return Err(format!(
            "子环钩子响应体长度错误: {}B (期望 {})",
            body.len(),
            BUS_SUBRING_SLOTS + 3
        ));
    }
    for slot in 0..BUS_SUBRING_SLOTS {
        if body[slot] != 1 {
            return Err(format!(
                "第 {} 次 acquire 失败(应能借满 {} 个)",
                slot + 1,
                BUS_SUBRING_SLOTS
            ));
        }
    }
    if body[BUS_SUBRING_SLOTS] != 1 {
        return Err(format!(
            "第 {} 次 acquire 竟成功: 子环池容量硬上限被突破",
            BUS_SUBRING_SLOTS + 1
        ));
    }
    if body[BUS_SUBRING_SLOTS + 1] != 1 {
        return Err("release 未能全部归还".to_string());
    }
    if body[BUS_SUBRING_SLOTS + 2] != 1 {
        return Err("全部归还后再 acquire 仍失败(槽位未真正释放)".to_string());
    }
    println!(
        "[BUS] subring PASS: 借满 {} 个, 第 {} 次被拒, 全还后可再借",
        BUS_SUBRING_SLOTS,
        BUS_SUBRING_SLOTS + 1
    );
    Ok(())
}

fn run_bus_test(ctrl: &mut AppController) -> Result<(), String> {
    _bus_send_led_set(ctrl, 0, &[10, 20, 30])?;
    println!("[BUS] step 1 PASS: accepted=1 rejected=0");

    let first = _bus_poll_led_state(ctrl)?;
    _bus_validate_led_state(&first, [10, 20, 30], false)?;
    println!("[BUS] step 2 PASS: LED_STATE seq={}", first.seq);

    let mut previous_seq = first.seq;
    let mut sequences = Vec::with_capacity(3);
    for (index, rgb) in [[1, 2, 3], [4, 5, 6], [7, 8, 9]].iter().enumerate() {
        _bus_send_led_set(ctrl, (index + 1) as u16, rgb)?;
        let state = _bus_poll_led_state(ctrl)?;
        _bus_validate_led_state(&state, *rgb, false)?;
        let delta = state.seq.wrapping_sub(previous_seq);
        if delta == 0 || delta > 16 {
            return Err(format!(
                "LED_STATE seq 未严格小步递增: previous={} current={} delta={}",
                previous_seq, state.seq, delta
            ));
        }
        previous_seq = state.seq;
        sequences.push(state.seq);
    }
    println!("[BUS] step 3 PASS: LED_STATE seq={:?}", sequences);

    // 步骤 4: 非法长度必须被显式拒绝, 而不是夹取/截断后照用。
    // 注意不能断言"RGB 仍等于上一步的值": main.cpp:442 的主循环心跳每 150ms 翻转 LED,
    // 会正常改写 LedService 内部存储值, 任何等值断言都是竞态。
    // 真正要证的是"非法载荷没有被施加": 心跳只会写 0 或 255, 所以只要 RGB 前两字节
    // 不是非法载荷 [1,2], 就说明设备拒绝了它。同时错误标志必须置位。
    _bus_send_led_set(ctrl, 4, &[1, 2])?;
    let invalid = _bus_poll_led_state(ctrl)?;
    if invalid.msg_id != BUS_MSG_LED_STATE {
        return Err(format!("msg_id 错误: 0x{:02X}", invalid.msg_id));
    }
    if invalid.payload.len() != 4 {
        return Err(format!(
            "LED_STATE payload 长度错误: {}",
            invalid.payload.len()
        ));
    }
    if (invalid.payload[3] & BUS_LED_STATE_FLAG_ERROR) == 0 {
        return Err(format!(
            "非法长度未置错误标志: payload[3]=0x{:02X}",
            invalid.payload[3]
        ));
    }
    if invalid.payload[0] == 1 && invalid.payload[1] == 2 {
        return Err(format!(
            "非法载荷竟被施加(应拒绝而非截断): rgb={:02X?}",
            &invalid.payload[0..3]
        ));
    }
    println!(
        "[BUS] step 4 PASS: 非法长度被拒(错误标志置位, 未施加), 回显设备真值 rgb={:02X?} seq={}",
        &invalid.payload[0..3],
        invalid.seq
    );

    // ── 扩展子项: 分片/重组、超长拒绝、坏帧、推送环溢出、回绕、统计、流式、子环池 ──
    // 每项独立打印 PASS/FAIL; 任一项失败即整体非零退出(由 main 的分支负责)。
    let _ = _bus_drain(ctrl, 2)?;
    let base = _bus_step_stat(ctrl)?;
    _bus_step_fragment(ctrl, &base)?;
    _bus_step_fragment_gap(ctrl)?;
    let before_oversize = _bus_stat(ctrl)?;
    _bus_step_oversize(ctrl, &before_oversize)?;
    let before_crc = _bus_stat(ctrl)?;
    _bus_step_bad_crc(ctrl, &before_crc)?;
    _bus_step_push_overflow(ctrl)?;
    _bus_step_wrap()?;
    _bus_step_stream(ctrl)?;
    _bus_step_subring(ctrl)?;
    Ok(())
}

// ============================================================================
// --cfg-set <键> <值>: 无头改单个配置项并落盘。
//
// ★类型必须取自设备回读的 schema 真值★ 同名键在不同版本可能是 Bool/U8/U16/U32, 按字面量猜
// 类型会下发一个必被固件 NAK(或被静默截断)的值。因此先 CFG_GET_ALL 拿到该键当前值的类型,
// 再按该类型解析命令行给的字面量; 键不存在直接失败, 不新建键。
// 走的是界面上改同一项的**同一条**路径(set_config → save_config), 因此它验证/修改的就是
// 用户点界面时真正发生的事。
// ============================================================================
fn run_cfg_set(ctrl: &mut AppController, key: &str, raw: &str) -> ! {
    let _ = ctrl.request_config_all();
    let deadline = std::time::Instant::now() + Duration::from_millis(4000);
    while std::time::Instant::now() < deadline && ctrl.config_entries().is_empty() {
        ctrl.poll();
        thread::sleep(Duration::from_millis(20));
    }
    let Some(current) = ctrl.config_get(key) else {
        println!("[CFGSET] FAIL: 设备配置里没有键 '{}'", key);
        std::process::exit(1);
    };
    let parse_bool = |s: &str| -> Option<bool> {
        match s.to_ascii_lowercase().as_str() {
            "1" | "true" | "on" | "yes" => Some(true),
            "0" | "false" | "off" | "no" => Some(false),
            _ => None,
        }
    };
    let value = match &current.value {
        CfgValue::Bool(_) => parse_bool(raw).map(CfgValue::Bool),
        CfgValue::U8(_) => raw.parse::<u8>().ok().map(CfgValue::U8),
        CfgValue::U16(_) => raw.parse::<u16>().ok().map(CfgValue::U16),
        CfgValue::U32(_) => raw.parse::<u32>().ok().map(CfgValue::U32),
        CfgValue::I8(_) => raw.parse::<i8>().ok().map(CfgValue::I8),
        CfgValue::F32(_) => raw.parse::<f32>().ok().map(CfgValue::F32),
        CfgValue::Str(_) => Some(CfgValue::Str(raw.to_string())),
    };
    let Some(value) = value else {
        println!(
            "[CFGSET] FAIL: '{}' 的 schema 类型是 {:?}, 无法解析给定值 '{}'",
            key, current.value, raw
        );
        std::process::exit(1);
    };
    println!(
        "[CFGSET] {} : {:?} -> {:?} (写穿 + 落盘)",
        key, current.value, value
    );
    if let Err(e) = ctrl.set_config(ConfigEntry::new(key.to_string(), value)) {
        println!("[CFGSET] FAIL set_config: {}", e);
        std::process::exit(1);
    }
    if let Err(e) = ctrl.save_config() {
        println!("[CFGSET] FAIL save_config: {}", e);
        std::process::exit(1);
    }
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    while std::time::Instant::now() < deadline && ctrl.cfg_tx_pending() != 0 {
        ctrl.poll();
        thread::sleep(Duration::from_millis(16));
    }
    if ctrl.cfg_tx_pending() != 0 {
        println!("[CFGSET] FAIL: 保存队列超时 pending={}", ctrl.cfg_tx_pending());
        std::process::exit(1);
    }
    // 回读校验: 退出码不作证据, 只认设备真值。
    let _ = ctrl.request_config_all();
    let deadline = std::time::Instant::now() + Duration::from_millis(4000);
    let mut got = None;
    while std::time::Instant::now() < deadline {
        ctrl.poll();
        thread::sleep(Duration::from_millis(20));
        if let Some(e) = ctrl.config_get(key) {
            got = Some(e.value.clone());
        }
    }
    println!("[CFGSET] 回读: {} = {:?}", key, got);
    std::process::exit(0);
}

// ============================================================================
// --kbd-hidout: HID 键盘输出闭环自检(无人值守)
//
// ★为什么必须有这条旁路★ "按键映射显示发送成功、系统却没有真实输入"横跨固件 HID 报文、USB
// 端点、Windows 键盘栈三层, 而这条链原本唯一的触发入口是"真人按物理键 / 真人摸触控板" ——
// 无头环境下不可复现, 只能靠猜。本开关把触发端也变成可编程的:
//   1) 把物理键 idx 的键码临时改成"只有 LeftCtrl 修饰位、无普通键码": 主机即便真收到也只是
//      按下一个 Ctrl, 不会把字符打进当前焦点窗口(沿用用户原键码会真往编辑器里敲字)。
//   2) 把该键极性临时改成高电平触发: 板上 1K 外部上拉使引脚恒高 ⇒ 固件判为"一直按住",
//      走的是与真人按键**完全同一条** _apply_phys → HID::press_key → 报文出口。
//   3) 按住期间用 GetAsyncKeyState 观测主机侧 Ctrl 是否真的处于按下态。
// 两项改动只写设备 RAM 影子(固件 KBD_SET_MAP / KBD_SET_KEYCFG 不碰 flash), 收尾回填原值,
// 复位亦自然恢复, 不污染用户配置。
//
// 判据三分, 不合并 —— 合并就回到"只知道不工作、不知道断在哪":
//   固件计数不增        ⇒ 断在固件内部(极性/长按/HID 未初始化)
//   计数增 + 主机没看到 ⇒ 报文发出去了但 Windows 没认成按键(描述符/报文格式/键盘栈)
//   计数增 + 主机看到   ⇒ HID 键盘链路本身是通的, 故障在更上层(消费方/焦点)
// ============================================================================
const KBD_HIDOUT_KEY_INDEX: u8 = 0;
const KBD_HIDOUT_MOD_LCTRL: u8 = 0x01;
const KBD_HIDOUT_VK_LCONTROL: i32 = 0xA2;
const KBD_HIDOUT_VK_CONTROL: i32 = 0x11;
const KBD_HIDOUT_KEYCODE_ESC: u8 = 0x29;
const KBD_HIDOUT_VK_ESCAPE: i32 = 0x1B;

fn _kbd_hidout_key_down(vk: i32) -> bool {
    // 只看 0x8000(此刻是否按下)。低位是"上次调用以来是否按过", 会把早已抬起的历史算成按下。
    (unsafe { windows::Win32::UI::Input::KeyboardAndMouse::GetAsyncKeyState(vk) } as u16 & 0x8000)
        != 0
}

fn run_kbd_hidout(ctrl: &mut AppController, hold_ms: u64) -> ! {
    let pump = |ctrl: &mut AppController, n: u32| {
        for _ in 0..n {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
    };
    let read_state = |ctrl: &mut AppController| {
        let _ = ctrl.kbd_request_state();
        pump(ctrl, 15);
    };

    println!("[HIDOUT] 读取物理键码表 / 每键配置 / 实时态…");
    let _ = ctrl.kbd_request_map();
    let _ = ctrl.kbd_request_keycfg();
    let _ = ctrl.kbd_request_state();
    pump(ctrl, 40);

    let idx = KBD_HIDOUT_KEY_INDEX;
    let orig_code = ctrl.kbd_map(idx);
    let orig_mod = ctrl.kbd_keymod(idx);
    let orig_cfg = ctrl.kbd_keycfg(idx);
    println!(
        "[HIDOUT] 键{} 原值: keycode={:#04X} mod={:#04X} pol={} debounce={}us",
        idx + 1,
        orig_code,
        orig_mod,
        orig_cfg.pol,
        orig_cfg.debounce_us
    );

    let diag0 = ctrl.kbd_link_diag().cloned();
    let sent0 = diag0.as_ref().map(|d| d.hid_sent);
    let fail0 = diag0.as_ref().map(|d| d.hid_failed);
    let hid_init = diag0.as_ref().map(|d| d.hid_initialized);
    println!(
        "[HIDOUT] 基线: hid_initialized={:?} hid_sent={:?} hid_failed={:?}",
        hid_init, sent0, fail0
    );
    if _kbd_hidout_key_down(KBD_HIDOUT_VK_LCONTROL) || _kbd_hidout_key_down(KBD_HIDOUT_VK_CONTROL) {
        println!("[HIDOUT] FAIL: 测试开始前主机侧 Ctrl 已处于按下态, 观测口径不可信");
        std::process::exit(1);
    }

    // 一轮 = 临时把该键改成给定 [keycode, modifier] + 高电平触发(恒读按下), 观测主机是否真收到。
    // ★修饰位与普通键码必须分成两轮★ 两者在报文里是**不同字节**([0] vs [2..7]), 也走中间件里
    // 不同的分支; 只测一种就无法区分"整条链断了"和"只有键码数组那一段没被主机认"。
    let phase = |ctrl: &mut AppController,
                     label: &str,
                     keycode: u8,
                     modifier: u8,
                     vks: &[i32],
                     hold: u64|
     -> (Option<u32>, bool, u16) {
        let base = ctrl.kbd_link_diag().map(|d| d.hid_sent);
        if let Err(e) = ctrl.kbd_send_map_now(idx, keycode, modifier) {
            println!("[HIDOUT] FAIL: KBD_SET_MAP 下发失败: {}", e);
            std::process::exit(1);
        }
        if let Err(e) = ctrl.kbd_send_keycfg_now(idx, 1, 0) {
            println!("[HIDOUT] FAIL: KBD_SET_KEYCFG 下发失败: {}", e);
            std::process::exit(1);
        }
        println!(
            "[HIDOUT] {}: 键{} → keycode={:#04X} mod={:#04X} + 高电平触发, 按住 {}ms…",
            label,
            idx + 1,
            keycode,
            modifier,
            hold
        );
        let mut saw = false;
        let mut out_seen: u16 = 0;
        let deadline = std::time::Instant::now() + Duration::from_millis(hold);
        let mut last_req = std::time::Instant::now() - Duration::from_millis(1000);
        while std::time::Instant::now() < deadline {
            ctrl.poll();
            if vks.iter().any(|vk| _kbd_hidout_key_down(*vk)) {
                saw = true;
            }
            if last_req.elapsed() >= Duration::from_millis(150) {
                last_req = std::time::Instant::now();
                let _ = ctrl.kbd_request_state();
            }
            out_seen |= ctrl.kbd_state_out();
            thread::sleep(Duration::from_millis(10));
        }
        read_state(ctrl);
        let after = ctrl.kbd_link_diag().map(|d| d.hid_sent);
        // 松手: 极性回 AUTO 即视为抬起, 报文由固件的收缩补清路径发出。
        let _ = ctrl.kbd_send_keycfg_now(idx, orig_cfg.pol, orig_cfg.debounce_us);
        pump(ctrl, 25);
        let delta = match (base, after) {
            (Some(a), Some(b)) => Some(b.wrapping_sub(a)),
            _ => None,
        };
        println!(
            "[HIDOUT] {}: out=0x{:03X} hid_sent {:?}->{:?} 主机侧按下={}",
            label, out_seen, base, after, saw
        );
        (delta, saw, out_seen)
    };

    // A) 仅修饰位(LeftCtrl): 报文第 0 字节。
    let (mod_delta, mod_saw, _) = phase(
        ctrl,
        "阶段A 仅修饰位LeftCtrl",
        0,
        KBD_HIDOUT_MOD_LCTRL,
        &[KBD_HIDOUT_VK_LCONTROL, KBD_HIDOUT_VK_CONTROL],
        hold_ms,
    );
    // B) 仅普通键码(Escape): 报文第 2..7 字节的键码数组。
    // ★选 Escape★ 无副作用(不往焦点窗口写入字符), 且 HID usage 0x29 与 VK 0x1B 一一对应。
    let (key_delta, key_saw, _) = phase(
        ctrl,
        "阶段B 仅键码Escape",
        KBD_HIDOUT_KEYCODE_ESC,
        0,
        &[KBD_HIDOUT_VK_ESCAPE],
        hold_ms.min(600),
    );

    // 收尾: 无论上面结论如何都必须回填键码与极性, 否则该键会一直被判成按住。
    let _ = ctrl.kbd_send_keycfg_now(idx, orig_cfg.pol, orig_cfg.debounce_us);
    let _ = ctrl.kbd_send_map_now(idx, orig_code, orig_mod);
    pump(ctrl, 30);
    let _ = ctrl.kbd_request_keycfg();
    let _ = ctrl.kbd_request_map();
    read_state(ctrl);
    println!(
        "[HIDOUT] 已回填: keycode={:#04X} mod={:#04X} pol={} debounce={}us (未落盘)",
        ctrl.kbd_map(idx),
        ctrl.kbd_keymod(idx),
        ctrl.kbd_keycfg(idx).pol,
        ctrl.kbd_keycfg(idx).debounce_us
    );

    let verdict = |label: &str, delta: Option<u32>, saw: bool| -> bool {
        match (delta, saw) {
            (None, _) => {
                println!("[HIDOUT] {} FAIL: 设备未回传链路诊断段, 无法判定", label);
                false
            }
            (Some(0), _) => {
                println!(
                    "[HIDOUT] {} FAIL(断点=固件内部): HID 报文实发数没有增加 ⇒ 按键没走到 HID 出口",
                    label
                );
                false
            }
            (Some(n), false) => {
                println!(
                    "[HIDOUT] {} FAIL(断点=主机侧): 固件发出 {} 份报文, Windows 从未认成按下 ⇒ 报文格式/描述符/键盘栈",
                    label, n
                );
                false
            }
            (Some(n), true) => {
                println!(
                    "[HIDOUT] {} PASS: 固件发出 {} 份报文, 主机侧确实按下",
                    label, n
                );
                true
            }
        }
    };
    let ok_mod = verdict("阶段A 修饰位", mod_delta, mod_saw);
    let ok_key = verdict("阶段B 键码", key_delta, key_saw);
    if ok_mod && ok_key {
        println!("[HIDOUT] PASS: 修饰位与普通键码两条报文字段主机都能收到");
        std::process::exit(0);
    }
    std::process::exit(1);
}

// ============================================================================
// --kbd-repair: 把 --nv-soak 写坏的**这几项**定向救回来, 不做整体 RESET_DEFAULTS。
//
// 只改三处:
//   1) kbd.pl00..kbd.pl11 → AUTO(2)。soak 把它们从 0 改成 1(12 键全高电平触发),
//      上拉工装上恒读按下 0xFFF → 边沿环零记录 → 键盘卡死。
//   2) led.enable → true(schema 默认)
//   3) led.status_brightness → 128(schema 默认)
// 防抖 kbd.dbNN 保留当前设备值(不趁机改写)。
// ★其余被 soak 改过的键(kbd.key*/kbd.zone*/kbd.hd*/kbd.mh*/kbd.cb*/bind.*/PARAM/算法源)
//   一律不动★ —— 那些不属于本次故障, 是否恢复由用户自己决定。
// ============================================================================
fn run_kbd_repair(ctrl: &mut AppController) -> ! {
    use mai2control_ui::proto::{CfgValue, ConfigEntry, KBD_POL_AUTO};

    const REPAIR_KEY_COUNT: u8 = 12;
    const LED_ENABLE_KEY: &str = "led.enable";
    const LED_BRIGHTNESS_KEY: &str = "led.status_brightness";
    // 与固件 app_config.cpp:132/135 的 schema 默认值同源。
    const LED_ENABLE_DEFAULT: bool = true;
    const LED_BRIGHTNESS_DEFAULT: u8 = 128;

    macro_rules! fail {
        ($($arg:tt)*) => {{
            println!("[REPAIR] FAIL: {}", format!($($arg)*));
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

    let pump = |ctrl: &mut AppController, ms: u64| -> bool {
        let deadline = std::time::Instant::now() + Duration::from_millis(ms);
        while std::time::Instant::now() < deadline {
            ctrl.poll();
            if ctrl.state() == ConnState::Disconnected {
                return false;
            }
            thread::sleep(Duration::from_millis(16));
        }
        true
    };
    let wait_version = |ctrl: &mut AppController,
                        before: u64,
                        label: &str,
                        version: fn(&AppController) -> u64,
                        timeout: Duration| {
        let deadline = std::time::Instant::now() + timeout;
        while std::time::Instant::now() < deadline && version(ctrl) <= before {
            if !pump(ctrl, 16) {
                fail!("{} 回读期间设备断开: {:?}", label, ctrl.last_error());
            }
        }
        if version(ctrl) <= before {
            fail!("{} 回读超时(version 仍为 {})", label, version(ctrl));
        }
    };
    let read_keycfg = |ctrl: &mut AppController, label: &str| {
        let before = ctrl.kbd_keycfg_version();
        require!(
            ctrl.kbd_request_keycfg(),
            format!("{} KBD_GET_KEYCFG 请求", label)
        );
        wait_version(
            ctrl,
            before,
            &format!("{} KBD_GET_KEYCFG", label),
            AppController::kbd_keycfg_version,
            Duration::from_secs(8),
        );
    };
    let read_state = |ctrl: &mut AppController, label: &str| {
        let before = ctrl.kbd_state_version();
        require!(
            ctrl.kbd_request_state(),
            format!("{} KBD_GET_STATE 请求", label)
        );
        wait_version(
            ctrl,
            before,
            &format!("{} KBD_GET_STATE", label),
            AppController::kbd_state_version,
            Duration::from_secs(8),
        );
    };
    let read_config = |ctrl: &mut AppController, label: &str| {
        let before = ctrl.config_version();
        require!(
            ctrl.request_config_all(),
            format!("{} CFG_GET_ALL 请求", label)
        );
        let deadline = std::time::Instant::now() + Duration::from_secs(12);
        while std::time::Instant::now() < deadline && ctrl.config_version() <= before {
            if !pump(ctrl, 16) {
                fail!(
                    "{} CFG_GET_ALL 期间设备断开: {:?}",
                    label,
                    ctrl.last_error()
                );
            }
        }
        if ctrl.config_version() <= before {
            fail!("{} CFG_GET_ALL 回读超时", label);
        }
    };

    read_keycfg(ctrl, "修复前");
    read_state(ctrl, "修复前");
    println!(
        "[REPAIR] 修复前: pol_cfg={:?} resolved=0x{:03X} phys_state=0x{:03X}",
        (0..REPAIR_KEY_COUNT)
            .map(|i| ctrl.kbd_keycfg(i).pol)
            .collect::<Vec<_>>(),
        ctrl.kbd_pol_resolved_mask(),
        ctrl.kbd_state()
    );

    // 1) 12 个物理键的极性一律设为 AUTO, 防抖沿用设备当前值。
    for index in 0..REPAIR_KEY_COUNT {
        let debounce = ctrl.kbd_keycfg(index).debounce_us;
        require!(
            ctrl.kbd_set_keycfg(index, KBD_POL_AUTO, debounce),
            format!("kbd_set_keycfg {} → AUTO", index)
        );
    }
    // 2) 状态灯两项回 schema 默认(走通用 CFG_SET, 与界面上改这两项完全同一条路)。
    require!(
        ctrl.set_config(ConfigEntry::new(
            LED_ENABLE_KEY.to_string(),
            CfgValue::Bool(LED_ENABLE_DEFAULT)
        )),
        format!("set_config {}", LED_ENABLE_KEY)
    );
    require!(
        ctrl.set_config(ConfigEntry::new(
            LED_BRIGHTNESS_KEY.to_string(),
            CfgValue::U8(LED_BRIGHTNESS_DEFAULT)
        )),
        format!("set_config {}", LED_BRIGHTNESS_KEY)
    );

    // 3) 走既有"保存到设备"路径落盘, 等串行队列排空 + 设备侧脏位归零, 再重启。
    println!("[REPAIR] 提交保存(仅上述 3 项)…");
    require!(ctrl.save_config(), "save_config");
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    while std::time::Instant::now() < deadline && ctrl.cfg_tx_pending() != 0 {
        if !pump(ctrl, 16) {
            fail!("保存队列期间设备断开: {:?}", ctrl.last_error());
        }
    }
    if ctrl.cfg_tx_pending() != 0 {
        fail!("保存队列超时: cfg_pending={}", ctrl.cfg_tx_pending());
    }
    let flush_deadline = std::time::Instant::now() + Duration::from_secs(60);
    loop {
        if !pump(ctrl, 250) {
            fail!("等待落盘期间设备断开: {:?}", ctrl.last_error());
        }
        let Some(now) = read_soak_debug(ctrl) else {
            fail!("无法读取设备 NvStore 状态(EP0 诊断不可用)");
        };
        if now.nv_dirty_mask == 0 {
            println!(
                "[REPAIR] 落盘完成 dirty_mask=0 commit_ok={} commit_fail={}",
                now.nv_commit_ok, now.nv_commit_fail
            );
            break;
        }
        if std::time::Instant::now() >= flush_deadline {
            fail!("落盘未在 60 秒内完成: dirty_mask=0x{:X}", now.nv_dirty_mask);
        }
    }

    println!("[REPAIR] 重启设备并等待重新枚举…");
    require!(ctrl.reboot(), "reboot");
    let reconnect_deadline = std::time::Instant::now() + Duration::from_secs(45);
    let mut saw_disconnect = false;
    let mut last_refresh = std::time::Instant::now() - Duration::from_secs(1);
    let mut last_hello = std::time::Instant::now();
    while std::time::Instant::now() < reconnect_deadline {
        ctrl.poll();
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
            "未在 45 秒内重新枚举并连接 (seen_disconnect={} state={:?})",
            saw_disconnect,
            ctrl.state()
        );
    }
    let _ = pump(ctrl, 1200);

    // 4) 重启后回读校验。
    read_keycfg(ctrl, "重启后");
    read_state(ctrl, "重启后");
    read_config(ctrl, "重启后");

    let mut reasons: Vec<String> = Vec::new();

    let pol: Vec<u8> = (0..REPAIR_KEY_COUNT)
        .map(|i| ctrl.kbd_keycfg(i).pol)
        .collect();
    let wrong: Vec<String> = pol
        .iter()
        .enumerate()
        .filter(|(_, value)| **value != KBD_POL_AUTO)
        .map(|(index, value)| format!("键{}={}", index + 1, value))
        .collect();
    if !wrong.is_empty() {
        reasons.push(format!("pol_cfg 未全为 AUTO(2): {}", wrong.join(",")));
    }

    let resolved = ctrl.kbd_pol_resolved_mask();
    if !ctrl.kbd_pol_resolved_known() {
        reasons.push("设备未回传生效极性掩码(固件过旧)".to_string());
    }
    let levels: Vec<String> = (0..REPAIR_KEY_COUNT)
        .map(|i| {
            format!(
                "键{}={}",
                i + 1,
                if (resolved >> i) & 1 != 0 {
                    "高"
                } else {
                    "低"
                }
            )
        })
        .collect();
    println!(
        "[REPAIR] pol_cfg={:?} resolved=0x{:03X} 生效电平: {}",
        pol,
        resolved,
        levels.join(" ")
    );

    // ★AUTO 功能的真机判据★: 修复前 0xFFF(恒读按下), AUTO 把上电电平学成"抬起"后应为 0x000。
    let phys_state = ctrl.kbd_state();
    if phys_state != 0 {
        reasons.push(format!(
            "phys_state 应为 0x000, 实际 0x{:03X}(仍有键被判成按下)",
            phys_state
        ));
    }
    println!("[REPAIR] phys_state=0x{:03X}", phys_state);

    let led_enable = ctrl.config_get(LED_ENABLE_KEY);
    match &led_enable {
        Some(entry) => {
            println!(
                "[REPAIR] {} = {}",
                LED_ENABLE_KEY,
                soak_val_text(&entry.value)
            );
            if soak_val_text(&entry.value) != LED_ENABLE_DEFAULT.to_string() {
                reasons.push(format!(
                    "{} 期望 {} 实际 {}",
                    LED_ENABLE_KEY,
                    LED_ENABLE_DEFAULT,
                    soak_val_text(&entry.value)
                ));
            }
        }
        None => reasons.push(format!("{} 回读缺失", LED_ENABLE_KEY)),
    }
    let led_brightness = ctrl.config_get(LED_BRIGHTNESS_KEY);
    match &led_brightness {
        Some(entry) => {
            println!(
                "[REPAIR] {} = {}",
                LED_BRIGHTNESS_KEY,
                soak_val_text(&entry.value)
            );
            if soak_val_text(&entry.value) != LED_BRIGHTNESS_DEFAULT.to_string() {
                reasons.push(format!(
                    "{} 期望 {} 实际 {}",
                    LED_BRIGHTNESS_KEY,
                    LED_BRIGHTNESS_DEFAULT,
                    soak_val_text(&entry.value)
                ));
            }
        }
        None => reasons.push(format!("{} 回读缺失", LED_BRIGHTNESS_KEY)),
    }

    println!(
        "[REPAIR] 本次只改了 kbd.pl00..pl11 → AUTO 与 {} / {}; \
         其余 --nv-soak 改动(kbd.key*/kbd.zone*/kbd.hd*/kbd.mh*/kbd.cb*/bind.*/PARAM/算法源)未动。",
        LED_ENABLE_KEY, LED_BRIGHTNESS_KEY
    );
    if reasons.is_empty() {
        println!("[REPAIR] PASS");
        std::process::exit(0);
    }
    println!("[REPAIR] FAIL: {}", reasons.join(" | "));
    std::process::exit(1);
}

// ============================================================================
// --mai2-load: 三路同时顶流压测(mai2serial CDC + mai2light CDC + vendor)
// ----------------------------------------------------------------------------
// 存在理由: 本仓此前**没有任何**驱动两条 mai2 协议的测试设施(comport 只做 COM 口识别/改名),
// 所谓"压流量"只压了 vendor 一路 —— 于是"设备自发重启"这类 bug 只能等实机暴露。
// 核心指标不是"有没有报错", 而是 ★主循环最长阻塞时长 与 距 5s 看门狗的余量★:
// 那才是决定会不会被看门狗咬死的量, 其余都是旁证。
// 剖面数据来自固件 UsbDebugCounters 尾部(见 main_firmware/src/service/usb_debug.h),
// 经 EP0 0x50 读、0x53 清零(峰值量不可差分, 必须能开干净窗口)。
// ============================================================================

/// 与固件 `UsbDebugCounters`(pack(1)) 尾部字段同源的偏移。改固件结构必须同步这里。
/// nv 段止于 @66, 故剖面段从 @67 起; 总长 108。
const DBG_OFF_LOOP_MAX_US: usize = 67;
const DBG_OFF_SEG_MAX_US: usize = 71;
const DBG_OFF_HEAVY_REJECTS: usize = 103;
const DBG_OFF_HEAVY_BUSY: usize = 107;
const DBG_LEN_WITH_PROFILE: usize = 108;
/// 跨复位死前遗言(增强段)。改固件结构必须同步这里。
const DBG_OFF_LAST_LOOP_MAX_US: usize = 108;
const DBG_OFF_LAST_WAS_FAULT: usize = 112;
const DBG_OFF_LAST_RESET_REASON: usize = 113;
const DBG_LEN_WITH_POSTMORTEM: usize = 114;
/// game_io 子段最长耗时数组。
const DBG_OFF_SEG2_MAX_US: usize = 114;
const DBG_LEN_WITH_SEG2: usize = 146;
/// NvStore 各区有效掩码(单份存储: 坏只坏在那一区, 必须看得见)。
const DBG_OFF_NV_VALID_MASK: usize = 154;
const DBG_LEN_WITH_NV_VALID: usize = 155;
/// 上次 hardfault 发生在哪个核 / 当前 core1 阶段码(固件 last_boot_fault_core / core1_stage)。
const DBG_OFF_LAST_FAULT_CORE: usize = 155;
const DBG_OFF_CORE1_STAGE: usize = 156;
/// 响应编码失败取证(固件 resp_encode_fail / resp_fail_cmd / resp_fail_req_len)。
/// dispatch 返回 resp_len==0 ⇒ encode_* 因 max_len 装不下而拒绝组帧, 主机永远等不到该命令的响应。
const DBG_OFF_RESP_ENCODE_FAIL: usize = 157;
const DBG_OFF_RESP_FAIL_CMD: usize = 161;
const DBG_OFF_RESP_FAIL_REQ_LEN: usize = 163;
/// sizeof(UsbDebugCounters)。★GPIO 尾部必须以它为基址★: 先前这里直接用了
/// DBG_LEN_WITH_NV_VALID(155), 但 nv_valid_mask 之后固件还有 last_boot_fault_core/core1_stage,
/// 于是尾部整体读偏 2 字节(实测 read_ok/status/PC 全是错位垃圾值)。改固件结构必须同步这里。
/// sizeof(UsbDebugCounters) 的旧前缀长度，旧固件仍可按此前缀解析。
const DBG_LEN_LEGACY_COUNTERS: usize = 165;
const DBG_LEN_COUNTERS: usize = 191;
/// 新增 HostCmd 分发观测字段(紧随 UsbDebugCounters 原有 165B 前缀)。
const DBG_OFF_HOST_DISPATCH_COUNT: usize = 165;
const DBG_OFF_HOST_ALGO_INFO_DISPATCH_COUNT: usize = 169;
const DBG_OFF_HOST_LAST_DISPATCH_CMD: usize = 173;
const DBG_OFF_HOST_LAST_DISPATCH_SEQ: usize = 174;
const DBG_OFF_HOST_LAST_DISPATCH_RESP_LEN: usize = 175;
const DBG_LEN_WITH_HOST_DISPATCH: usize = 177;

/// core1 新代数通知线(PSoC P1.4 → GPIO23)实证。armed=1 = core1 已停止固定间隔空转轮询,
/// 改为等 PSoC 的"新代数已发布"翻转; edges 应约等于扫描速率 × 观测时长。
/// ★这三个字段是该改造唯一的外部证据★: 通知驱动与轮询在帧率/丢帧/延迟上完全同形。
const DBG_OFF_CORE1_INT1_EDGES: usize = 177;
const DBG_OFF_CORE1_INT1_TIMEOUTS: usize = 181;
const DBG_OFF_CORE1_INT1_ARMED: usize = 185;
/// 触控延迟线的补偿量(GameIoService::_emit_cost_us, us)。★这是"设定延迟是否被削短"的唯一外部
/// 读数★: 补偿量被高估多少, `comm.touch_delay_100us` 就被削短多少。
const DBG_OFF_GIO_EMIT_COST: usize = 186;
const DBG_LEN_WITH_EMIT_COST: usize = 190;
/// 开机校准流水线的五判据诊断字节(见固件 usb_debug.h::boot_cal_diag)。
const DBG_OFF_BOOT_CAL_DIAG: usize = 190;
/// 开机校准各档结局位图(见固件 usb_debug.h::boot_cal_fail_mask)。
const DBG_OFF_BOOT_CAL_FAIL: usize = 191;
const DBG_LEN_WITH_BOOT_CAL: usize = 192;
const DBG_LEN_WITH_INT1: usize = 186;
/// EP0 DEBUG_READ 的固定 GPIO/HSIOM 尾部(见 UsbDebugGpioTail)。
const DBG_OFF_GPIO_PC: usize = DBG_LEN_COUNTERS;
const DBG_OFF_HSIOM_PORT_SEL: usize = DBG_OFF_GPIO_PC + 8 * 4;
const DBG_OFF_GPIO_SWD_STATUS: usize = DBG_OFF_HSIOM_PORT_SEL + 8 * 4;
const DBG_OFF_GPIO_READ_OK: usize = DBG_OFF_GPIO_SWD_STATUS + 4;
const DBG_LEN_WITH_GPIO: usize = DBG_OFF_GPIO_READ_OK + 4;
/// 上次复位时"最后一次进段时本轮已耗时"(ms) —— 定位时间到底花在哪一段。
const DBG_OFF_LAST_STAGE_AT_MS: usize = 150;
const DBG_LEN_WITH_STAGE_AT: usize = 154;
/// 固件 CRASH_STAGE_LOOP_BASE / CRASH_STAGE_GAMEIO_BASE(usb_debug.h)。
const CRASH_STAGE_LOOP_BASE: u8 = 0x10;
const CRASH_STAGE_GAMEIO_BASE: u8 = 0x20;
/// 段名顺序必须与固件 `enum GameIoSeg` 一一对应。
const GAMEIO_SEG_NAMES: [&str; 8] = [
    "binding",
    "serial_rx",
    "ser_reset",
    "light",
    "touch_map",
    "send_touch",
    "light_state",
    "ledmap",
];
/// 段名顺序必须与固件 `enum LoopSeg` 一一对应。
const LOOP_SEG_NAMES: [&str; 8] = [
    "usb_task",
    "psoc",
    "host_cmd",
    "tx_sched",
    "game_io",
    "keyboard",
    "nv_commit",
    "led",
];
/// 固件 WATCHDOG_TIMEOUT_MS = 5000(main.cpp) ⇒ 余量以此为基准。
const WATCHDOG_BUDGET_US: u32 = 5_000_000;

#[derive(Clone, Copy, Default)]
struct LoopProfile {
    loop_count: u32,
    rx_dropped: u32,
    loop_max_us: u32,
    seg_max_us: [u32; 8],
    heavy_rejects: u32,
    heavy_busy: u8,
    nv_commit_fail: u32,
    /// 触控延迟线补偿量(us)。`None` = 固件尚无该字段(旧固件), 与"补偿量为 0"必须分开表达。
    emit_cost_us: Option<u32>,
    /// flash 落地次数 / NvStore 成功提交次数。★这两项是延迟尖峰的头号嫌疑★:
    /// XIP 擦写期间 USB 与主循环都停摆, 一次提交就能造出一簇尖峰。差分即知窗口内是否发生过。
    flash_writes: u32,
    nv_commit_ok: u32,
    seg2_max_us: [u32; 8],
    /// core1 新代数通知线实证。`None` = 固件尚无该字段(旧固件), 与"armed=0"必须分开表达。
    int1: Option<Int1Witness>,
}

/// core1 是按 PSoC 的通知走, 还是退回了固定间隔自由跑。
#[derive(Clone, Copy, Default)]
struct Int1Witness {
    edges: u32,
    timeouts: u32,
    armed: bool,
}

/// 上次复位的死前遗言。定性三分: fault=1 → 跑飞; fault=0 且 last_loop_max 接近 5s → 主循环真被拖死;
/// fault=0 且 last_loop_max 很小 → 既没跑飞也没拖慢, 那就是外部原因(掉电/XRES/主动重启)。
fn print_post_mortem(tag: &str, b: &[u8]) {
    if b.len() < DBG_LEN_WITH_POSTMORTEM {
        println!(
            "{} 诊断结构无死前遗言增强段(len={}), 固件需更新",
            tag,
            b.len()
        );
        return;
    }
    let stage = b[48];
    let stage_text = if stage == 0xFF {
        "不可信(上次非运行中复位)".to_string()
    } else if let Some(name) = match stage {
        // Mai2Light/CDC 语句级阶段码, 见固件 usb_debug.h 的 PM_STAGE_*。
        0x30 => Some("light/cdc_read(搬 CDC 环)"),
        0x31 => Some("light/_feed 逐字节(含 dispatch/ack)"),
        0x32 => Some("light/ack 查 TX 剩余空间"),
        0x33 => Some("light/ack 写 TX FIFO"),
        0x34 => Some("light/_fade_step"),
        0x36 => Some("cdc/tud_cdc_n_write_available"),
        0x37 => Some("cdc/tud_cdc_n_write"),
        0x38 => Some("cdc/tud_cdc_n_write_flush"),
        0x39 => Some("cdc/tud_cdc_n_read(rx 回调)"),
        0x3A => Some("cdc/rx 回调: read 已返回, 搬环中"),
        0x3B => Some("cdc/rx 回调已结束 → 卡在 tud_task 内部别处"),
        0x3C => Some("usb/tud_task 内部(尚未进 rx 回调)"),
        0x3D => Some("usb/tud_task 已返回"),
        _ => None,
    } {
        name.to_string()
    } else if stage >= CRASH_STAGE_GAMEIO_BASE
        && (stage - CRASH_STAGE_GAMEIO_BASE) < GAMEIO_SEG_NAMES.len() as u8
    {
        format!(
            "game_io 子段 {}",
            GAMEIO_SEG_NAMES[(stage - CRASH_STAGE_GAMEIO_BASE) as usize]
        )
    } else if stage >= CRASH_STAGE_LOOP_BASE
        && (stage - CRASH_STAGE_LOOP_BASE) < LOOP_SEG_NAMES.len() as u8
    {
        format!(
            "主循环段 {}",
            LOOP_SEG_NAMES[(stage - CRASH_STAGE_LOOP_BASE) as usize]
        )
    } else {
        format!("CrashStage {}", stage)
    };
    let last_loop_max = u32::from_le_bytes([
        b[DBG_OFF_LAST_LOOP_MAX_US],
        b[DBG_OFF_LAST_LOOP_MAX_US + 1],
        b[DBG_OFF_LAST_LOOP_MAX_US + 2],
        b[DBG_OFF_LAST_LOOP_MAX_US + 3],
    ]);
    let fault = b[DBG_OFF_LAST_WAS_FAULT];
    let reason = b[DBG_OFF_LAST_RESET_REASON];
    // ★关键判据★: 进入最后那一段时"本轮已耗时"。接近 5s ⇒ 时间花在**它前面**的段, stage 指向的是
    // 无辜的短段; 接近 0 ⇒ 时间确实花在 stage 指的那一段里。
    let stage_at_ms: Option<u16> = if b.len() >= DBG_LEN_WITH_STAGE_AT {
        Some(u16::from_le_bytes([
            b[DBG_OFF_LAST_STAGE_AT_MS],
            b[DBG_OFF_LAST_STAGE_AT_MS + 1],
        ]))
    } else {
        None
    };
    let verdict = if fault != 0 {
        "跑飞(hardfault)"
    } else if b[49] == 0 {
        "上电复位/主动重启(scratch 已清)"
    } else if matches!(stage_at_ms, Some(ms) if (ms as u32) * 1000 >= WATCHDOG_BUDGET_US / 2) {
        "主循环被拖死: 时间花在 stage 之前的段(见 进段时已耗时)"
    } else if stage_at_ms.is_some() {
        "主循环被拖死: 时间花在 stage 指的那一段里面"
    } else if last_loop_max >= WATCHDOG_BUDGET_US / 2 {
        "主循环被拖死(看门狗超时)"
    } else {
        "运行中复位, 但主循环并未变慢 —— 查外部原因(供电/XRES/软复位)"
    };
    println!(
        "{} 死前遗言: 定性={} | 运行中复位={} hardfault={} 停在={} 进段时已耗时={} 上次整轮峰值={}us wd_reason=0x{:X}",
        tag,
        verdict,
        b[49],
        fault,
        stage_text,
        stage_at_ms
            .map(|ms| format!("{}ms", ms))
            .unwrap_or_else(|| "n/a".to_string()),
        last_loop_max,
        reason
    );
}

/// 延迟剖面的一格窗口。窗口内先清零、再累积, 于是段峰值是**该窗口内**的最坏值。
/// ★峰值必须分窗★: 单调最大值只答得出"史上最坏是多少", 答不出"多久来一次" —— 而
/// "每隔数秒一簇尖峰"要找的恰恰是周期。
#[derive(Clone)]
struct LatWindow {
    /// 窗口结束时刻(自探测开始, ms)。
    t_ms: u64,
    /// 本窗口内收到的逐帧延迟观测(三段 + 延迟线补偿偏差)。
    lat: Vec<mai2control_ui::app_state::LatObs>,
    prof: LoopProfile,
    /// 本窗口内 core0 主循环轮数(loop_count 差分)。
    loops: u32,
}

/// 升序数组的百分位(线性下标, 不插值 —— 这里只需要量级判断)。
fn _pct(sorted: &[u32], p: f64) -> u32 {
    if sorted.is_empty() {
        return 0;
    }
    let idx = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn read_loop_profile(ctrl: &AppController) -> Option<LoopProfile> {
    let bytes = ctrl.read_debug_counters().ok()?;
    if bytes.len() < DBG_LEN_WITH_PROFILE {
        return None;
    }
    let u32_at =
        |o: usize| u32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
    let mut seg = [0u32; 8];
    for (i, slot) in seg.iter_mut().enumerate() {
        *slot = u32_at(DBG_OFF_SEG_MAX_US + i * 4);
    }
    let mut seg2 = [0u32; 8];
    if bytes.len() >= DBG_LEN_WITH_SEG2 {
        for (i, slot) in seg2.iter_mut().enumerate() {
            *slot = u32_at(DBG_OFF_SEG2_MAX_US + i * 4);
        }
    }
    let int1 = if bytes.len() >= DBG_LEN_WITH_INT1 {
        Some(Int1Witness {
            edges: u32_at(DBG_OFF_CORE1_INT1_EDGES),
            timeouts: u32_at(DBG_OFF_CORE1_INT1_TIMEOUTS),
            armed: bytes[DBG_OFF_CORE1_INT1_ARMED] != 0,
        })
    } else {
        None
    };
    Some(LoopProfile {
        seg2_max_us: seg2,
        loop_count: u32_at(4),
        rx_dropped: u32_at(20),
        loop_max_us: u32_at(DBG_OFF_LOOP_MAX_US),
        seg_max_us: seg,
        heavy_rejects: u32_at(DBG_OFF_HEAVY_REJECTS),
        heavy_busy: bytes[DBG_OFF_HEAVY_BUSY],
        nv_commit_fail: u32_at(59),
        flash_writes: u32_at(32),
        nv_commit_ok: u32_at(55),
        emit_cost_us: (bytes.len() >= DBG_LEN_WITH_EMIT_COST)
            .then(|| u32_at(DBG_OFF_GIO_EMIT_COST)),
        int1,
    })
}

/// 剖面结论。★结论必须自带判据★: 只贴一堆数字等于把排查原样推回给人。
/// 判据链: ① 三段各自的分位数 → 基线在哪、抖的是哪一段; ② 超阈值样本按时间聚簇 → 周期;
/// ③ 尖峰窗口与平静窗口的段峰值中位数对比 → 那一刻最坏的代码段是谁。
fn _report_lat_probe(windows: &[LatWindow], lost: u64, base: &LoopProfile) -> i32 {
    let mut seq: Vec<(u64, u32, u32, u32)> = Vec::new();
    // 延迟线补偿偏差单独收: 它是带符号量, 也不参与"总延迟"的求和。
    let mut devs: Vec<i32> = Vec::new();
    for w in windows {
        for s in &w.lat {
            seq.push((w.t_ms, s.spi_us as u32, s.proc_us as u32, s.usb_us as u32));
            if let Some((lo, hi)) = s.dev_us {
                devs.push(lo as i32);
                devs.push(hi as i32);
            }
        }
    }
    if seq.len() < 20 {
        println!("[LAT] FAIL 有效样本仅 {} 个, 不足以判周期", seq.len());
        return 1;
    }
    let col = |k: usize| -> Vec<u32> {
        let mut v: Vec<u32> = seq
            .iter()
            .map(|s| match k {
                0 => s.1,
                1 => s.2,
                2 => s.3,
                _ => s.1 + s.2 + s.3,
            })
            .collect();
        v.sort_unstable();
        v
    };
    println!(
        "[LAT] 样本 {} 个 / 窗口 {} 个 / 环溢出丢样 {}",
        seq.len(),
        windows.len(),
        lost
    );
    println!("[LAT] 段              p50      p90      p99      max");
    let mut p50 = [0u32; 4];
    let mut p90 = [0u32; 4];
    for (k, name) in ["SPI(core1)", "RP处理", "USB写", "总延迟"].iter().enumerate() {
        let v = col(k);
        p50[k] = _pct(&v, 0.50);
        p90[k] = _pct(&v, 0.90);
        println!(
            "[LAT]   {:<12} {:>6}us {:>6}us {:>6}us {:>6}us",
            name,
            p50[k],
            p90[k],
            _pct(&v, 0.99),
            v[v.len() - 1]
        );
    }
    // 阈值: 中位数 + 3 倍"p90 与中位数之差", 下限 150us。★不用固定绝对阈★: 基线随负载变,
    // 固定阈在轻载时把正常波动判成尖峰、在重载时又一个都抓不到。
    let spread = p90[3].saturating_sub(p50[3]);
    let thresh = p50[3] + (3 * spread).max(150);
    let hot: Vec<&(u64, u32, u32, u32)> = seq
        .iter()
        .filter(|s| s.1 + s.2 + s.3 > thresh)
        .collect();
    println!(
        "[LAT] 尖峰阈值 {}us (中位 {} + max(3×{}, 150)); 超阈样本 {} 个 ({:.1}%)",
        thresh,
        p50[3],
        spread,
        hot.len(),
        hot.len() as f64 * 100.0 / seq.len() as f64
    );
    // 聚簇: 相邻超阈样本间隔 ≤300ms 归一簇。周期性事件的表现是"一簇", 逐个样本列出会淹没周期。
    let mut clusters: Vec<(u64, u64, u32, [u32; 3], usize)> = Vec::new();
    for s in &hot {
        let total = s.1 + s.2 + s.3;
        match clusters.last_mut() {
            Some(c) if s.0.saturating_sub(c.1) <= 300 => {
                c.1 = s.0;
                c.4 += 1;
                if total > c.2 {
                    c.2 = total;
                    c.3 = [s.1, s.2, s.3];
                }
            }
            _ => clusters.push((s.0, s.0, total, [s.1, s.2, s.3], 1)),
        }
    }
    println!("[LAT] 尖峰簇 {} 个:", clusters.len());
    for (i, c) in clusters.iter().enumerate() {
        println!(
            "[LAT]   #{:<2} t={:>6}..{:<6}ms 样本{} 峰值总计{}us (SPI {} / RP {} / USB {})",
            i, c.0, c.1, c.4, c.2, c.3[0], c.3[1], c.3[2]
        );
    }
    if clusters.len() >= 2 {
        let mut gaps: Vec<u32> = clusters
            .windows(2)
            .map(|w| (w[1].0 - w[0].0) as u32)
            .collect();
        gaps.sort_unstable();
        println!(
            "[LAT] 簇间隔: 中位 {}ms 最小 {}ms 最大 {}ms ⇒ {}",
            _pct(&gaps, 0.50),
            gaps[0],
            gaps[gaps.len() - 1],
            if gaps[gaps.len() - 1] as f64 <= _pct(&gaps, 0.50) as f64 * 1.6 {
                "间隔集中 = 周期性事件"
            } else {
                "间隔分散 = 事件驱动而非固定周期"
            }
        );
    }
    // 分窗段峰值分布 + 各段自己的"超阈窗"周期。★这张表才是点名用的★:
    // 逐帧延迟只看得到 game_io 里那两段, 而周期性最坏路径可能在任何一段; 段峰值按窗分布后,
    // "某段每隔 N 个窗就冒一次" 直接可读, 不必再靠与尖峰窗求交集去猜。
    let mut picks: Vec<(String, Vec<u32>)> = Vec::new();
    picks.push((
        "loop 整轮".to_string(),
        windows.iter().map(|w| w.prof.loop_max_us).collect(),
    ));
    for i in 0..8usize {
        picks.push((
            format!("seg {}", LOOP_SEG_NAMES[i]),
            windows.iter().map(|w| w.prof.seg_max_us[i]).collect(),
        ));
        picks.push((
            format!("gio {}", GAMEIO_SEG_NAMES[i]),
            windows.iter().map(|w| w.prof.seg2_max_us[i]).collect(),
        ));
    }
    println!("[LAT] 分窗段峰值(窗宽 100ms, 每窗清零后重测) + 该段自身的超阈窗周期:");
    println!("[LAT]   段                    p50      p90      p99      max   超阈窗数  间隔中位");
    for (name, raw) in &picks {
        let mut v = raw.clone();
        v.sort_unstable();
        let (a, b) = (_pct(&v, 0.50), _pct(&v, 0.90));
        let th = a + (3 * b.saturating_sub(a)).max(150);
        let idx: Vec<usize> = raw
            .iter()
            .enumerate()
            .filter(|(_, x)| **x > th)
            .map(|(i, _)| i)
            .collect();
        let gap = if idx.len() >= 2 {
            let mut g: Vec<u32> = idx.windows(2).map(|w| (w[1] - w[0]) as u32 * 100).collect();
            g.sort_unstable();
            format!("{}ms", _pct(&g, 0.50))
        } else {
            "-".to_string()
        };
        println!(
            "[LAT]   {:<16} {:>7} {:>8} {:>8} {:>8} {:>9} {:>9}",
            name,
            a,
            b,
            _pct(&v, 0.99),
            v[v.len() - 1],
            idx.len(),
            gap
        );
    }
    // 最大的几个尖峰簇: 逐条打出覆盖窗(含前一窗)的整套段峰值 —— 单条现场比任何统计都直接。
    // ★为什么要带前一窗★: g_lat_* 是上一个遥测窗口(50ms)内的滚动峰值, 发帧才清零, 造成该峰值的
    // 那次执行可能落在前一个 100ms 剖面窗里。
    let mut top: Vec<&(u64, u64, u32, [u32; 3], usize)> = clusters.iter().collect();
    top.sort_by_key(|c| std::cmp::Reverse(c.2));
    for c in top.iter().take(3) {
        println!(
            "[LAT] 现场 t={}ms 峰值{}us (SPI {} / RP {} / USB {}):",
            c.0, c.2, c.3[0], c.3[1], c.3[2]
        );
        for w in windows.iter().filter(|w| w.t_ms + 100 >= c.0 && w.t_ms <= c.1 + 100) {
            let mut hot: Vec<String> = Vec::new();
            for i in 0..8usize {
                if w.prof.seg_max_us[i] >= 300 {
                    hot.push(format!("{}={}us", LOOP_SEG_NAMES[i], w.prof.seg_max_us[i]));
                }
                if w.prof.seg2_max_us[i] >= 300 {
                    hot.push(format!(
                        "gio/{}={}us",
                        GAMEIO_SEG_NAMES[i], w.prof.seg2_max_us[i]
                    ));
                }
            }
            println!(
                "[LAT]     窗 t={:>6}ms loop_max={}us 轮数={} | {}",
                w.t_ms,
                w.prof.loop_max_us,
                w.loops,
                hot.join(" ")
            );
        }
    }
    // flash / NvStore 是"整机停摆"级嫌疑: 一次 XIP 擦写就能造出一簇尖峰。有没有发生过必须看得见。
    let last = windows.last().map(|w| w.prof).unwrap_or_default();
    println!(
        "[LAT] 全程 flash 落地 {} 次 / NvStore 提交 {} 次(失败 {}) — {}",
        last.flash_writes.wrapping_sub(base.flash_writes),
        last.nv_commit_ok.wrapping_sub(base.nv_commit_ok),
        last.nv_commit_fail.wrapping_sub(base.nv_commit_fail),
        if last.flash_writes == base.flash_writes && last.nv_commit_ok == base.nv_commit_ok {
            "全程无 flash 活动 ⇒ 尖峰与落盘无关"
        } else {
            "★存在 flash 活动, 需按上表核对是否与尖峰窗重合"
        }
    );
    let mut loops: Vec<u32> = windows.iter().map(|w| w.loops).collect();
    loops.sort_unstable();
    println!(
        "[LAT] core0 主循环轮数/窗: 中位 {} 最少 {} 最多 {}(窗宽 100ms)",
        _pct(&loops, 0.50),
        loops[0],
        loops[loops.len() - 1]
    );
    if let (Some(b), Some(a)) = (base.int1, last.int1) {
        println!(
            "[LAT] core1 通知线: armed={} 翻转 {} 次 等待超时 {} 次",
            a.armed,
            a.edges.wrapping_sub(b.edges),
            a.timeouts.wrapping_sub(b.timeouts)
        );
    }
    // 延迟线补偿量与"可补偿段"的实测值对照: 补偿量应贴近 RP处理+USB写 的典型值。
    // 明显大于它 = 过补偿 ⇒ 用户设的 comm.touch_delay_100us 被削短同样多。
    // 延迟线补偿量的合理性判据。★注意两个量的统计口径不同★: 遥测里的 RP处理+USB写 是**遥测窗
    // (50ms)内的滚动峰值**, 即单拍真实耗时的上界; 补偿量要预测的是"这一拍"的耗时, 因此它**应当
    // 低于**那个上界。补偿量一旦超过上界, 只可能是它 latch 在了历史最坏值上(即本轮修掉的缺陷),
    // 而补偿量高出多少, 用户设的 comm.touch_delay_100us 就被削短多少。
    match last.emit_cost_us {
        Some(cost) => {
            let mut pu: Vec<u32> = seq.iter().map(|s| s.2 + s.3).collect();
            pu.sort_unstable();
            let bound = _pct(&pu, 0.50);
            println!(
                "[LAT] 延迟线补偿量 {}us / 可补偿段峰值上界(遥测口径中位) {}us ⇒ {}",
                cost,
                bound,
                if cost <= bound {
                    "未超上界: 设定触控延迟不被削短"
                } else {
                    "★过补偿: 设定触控延迟被削短约两者之差"
                }
            );
        }
        None => println!("[LAT] 延迟线补偿量: 固件无该诊断字段(旧固件)"),
    }
    // 补偿偏差本身: 这是"设定的触控延迟兑现了没有"的直接读数, 0 = 正好兑现。
    // ★没有观测的窗口不计入★ 那些窗口里没有触控帧真正发出(串口无消费者/被限速), 谈不上偏差。
    if devs.is_empty() {
        println!("[LAT] 延迟线补偿偏差: 全程无观测(没有触控帧真正发出, 串口无消费者)");
    } else {
        let mut abs: Vec<u32> = devs.iter().map(|d| d.unsigned_abs()).collect();
        abs.sort_unstable();
        let mut signed = devs.clone();
        signed.sort_unstable();
        println!(
            "[LAT] 延迟线补偿偏差: 观测 {} 个区间端点, 带符号 {}..{}us 中位 {}us | |偏差| p50 {}us p99 {}us max {}us",
            devs.len(),
            signed[0],
            signed[signed.len() - 1],
            signed[signed.len() / 2],
            _pct(&abs, 0.50),
            _pct(&abs, 0.99),
            abs[abs.len() - 1]
        );
    }
    0
}

/// 总延迟抖动剖面(`--lat-probe`)。逐帧三段延迟与分窗主循环剖面落在同一时间轴上:
/// 前者答"抖了多少、多久来一次", 后者答"那一刻是哪一段最坏"。缺任何一半都只能靠猜。
/// ★遥测档位必须与仪表盘一致(20Hz / fields=0 / ch_mask=0)★: 主页那张"总延迟历史"就是这条流
/// 画出来的, 换档位量出来的抖动代表不了用户看到的那张图。
fn run_lat_probe(
    ctrl: &mut AppController,
    secs: u64,
    per_window_clear: bool,
    read_each_window: bool,
) -> i32 {
    const WIN_MS: u64 = 100;
    // ★只在首尾读剖面的模式是"测量自身是否就是扰动源"的对照★: EP0 控制传输本身要经 tud_task
    // 处理, 每 100ms 两条(读+清)相当于给设备加了一份周期性负载。若关掉它之后逐帧延迟的尖峰
    // 随之消失, 那这些尖峰就是脚手架自造的, 与用户在 GUI 上看到的不是同一件事。
    if !read_each_window {
        println!("[LAT] 剖面只在首尾各读一次(全程无周期性 EP0 传输), 作为测量扰动的对照");
    }
    // ★不清零模式是交叉验证用的★: 段峰值不清零即为全程单调上界, 它必然 ≥ 任何 50ms 遥测窗内
    // 测到的同一段耗时。若 g_lat_* 报出的尖峰在单调上界里根本不存在, 那两个读数就不是在量同一
    // 件事, 必须先解决口径矛盾再谈周期。
    if !per_window_clear {
        println!("[LAT] 段峰值不分窗(全程单调上界), 用于与逐帧延迟交叉验证口径");
    }
    println!(
        "[LAT] 档位=仪表盘同形(20Hz / fields=0 → 仅 STATS|LATENCY / ch_mask=0), 时长 {}s, 剖面窗 {}ms",
        secs, WIN_MS
    );
    if let Err(e) = ctrl.start_telemetry(20, 0, 0) {
        println!("[LAT] FAIL start_telemetry: {}", e);
        return 1;
    }
    let warm = std::time::Instant::now();
    while ctrl.lat_version() == 0 && warm.elapsed() < Duration::from_secs(5) {
        ctrl.poll();
        thread::sleep(Duration::from_millis(5));
    }
    if ctrl.lat_version() == 0 {
        println!("[LAT] FAIL 5s 内没有收到任何带 LATENCY 的遥测帧");
        return 1;
    }
    let Some(base) = read_loop_profile(ctrl) else {
        println!("[LAT] FAIL 固件无主循环剖面段(需带剖面的新固件)");
        return 1;
    };
    let _ = ctrl.clear_loop_profile();
    let started = std::time::Instant::now();
    let mut last_ver = ctrl.lat_version();
    let mut last_loops = base.loop_count;
    let mut windows: Vec<LatWindow> = Vec::new();
    let mut next_win = started + Duration::from_millis(WIN_MS);
    let mut lost = 0u64;
    while started.elapsed() < Duration::from_secs(secs) {
        ctrl.poll();
        if std::time::Instant::now() < next_win {
            thread::sleep(Duration::from_millis(2));
            continue;
        }
        // ★续租★ 遥测租约 3s(TELEM_LEASE_MS), 靠"任意主机命令帧"续期。EP0 的剖面读走的是
        // vendor 控制请求, **不经** UsbComm::update ⇒ 不续租, 实测流在第 3s 静默停掉(样本只有
        // 头 60 个)。GUI 天然每帧都有命令流量, 本脚手架必须自己补。每窗一条最轻的读命令即可,
        // 且 100ms 的固定节奏会均匀落在所有窗口里, 不会伪造出"数秒一簇"的周期性扰动。
        let _ = ctrl.request_param(0, 0x0B);
        // 逐帧延迟: 版本差 = 本窗口新到的帧数, 取环尾同样多个即为本窗口样本。
        // 环容量 512 而 20Hz × 100ms ≈ 2 帧/窗 ⇒ 正常不会溢出; 真溢出则如实计入 lost。
        let ver = ctrl.lat_version();
        let fresh = ver.saturating_sub(last_ver) as usize;
        let all = ctrl.lat_obs_series();
        lost += fresh.saturating_sub(all.len()) as u64;
        let take = fresh.min(all.len());
        let lat = all[all.len() - take..].to_vec();
        last_ver = ver;
        let prof = if read_each_window {
            read_loop_profile(ctrl).unwrap_or_default()
        } else {
            LoopProfile::default()
        };
        let loops = prof.loop_count.wrapping_sub(last_loops);
        last_loops = prof.loop_count;
        if read_each_window && per_window_clear {
            let _ = ctrl.clear_loop_profile();
        }
        windows.push(LatWindow {
            t_ms: started.elapsed().as_millis() as u64,
            lat,
            prof,
            loops,
        });
        next_win += Duration::from_millis(WIN_MS);
    }
    if !read_each_window {
        // 首尾对照模式: 结束时补读一次, 使全程单调段峰值仍然可见(它是判"哪一段被拖过"的唯一凭据)。
        if let (Some(end), Some(w)) = (read_loop_profile(ctrl), windows.last_mut()) {
            w.prof = end;
        }
    }
    let _ = ctrl.stop_telemetry();
    _report_lat_probe(&windows, lost, &base)
}

/// core1 新代数通知线的一句话结论。供 `--soak` 等压测在结束时打印, 使"通知驱动真的生效"
/// 成为可复核的实测证据而不是推断。
fn int1_witness_text(before: &LoopProfile, after: &LoopProfile, elapsed_s: f32) -> String {
    let (Some(b), Some(a)) = (before.int1, after.int1) else {
        return "core1 通知线: 固件无该诊断字段(旧固件)".to_string();
    };
    let edges = a.edges.wrapping_sub(b.edges);
    let timeouts = a.timeouts.wrapping_sub(b.timeouts);
    let hz = if elapsed_s > 0.0 {
        edges as f32 / elapsed_s
    } else {
        0.0
    };
    format!(
        "core1 通知线: armed={} 新代数通知 {} 次({:.1}/s) 等待超时 {} 次",
        if a.armed { "是(已停止空转轮询)" } else { "否(退回自由跑兜底)" },
        edges,
        hz,
        timeouts
    )
}

#[derive(Default)]
struct Mai2LoadStats {
    ser_rx_bytes: std::sync::atomic::AtomicU64,
    ser_touch_frames: std::sync::atomic::AtomicU64,
    ser_cmd_resp: std::sync::atomic::AtomicU64,
    ser_halts: std::sync::atomic::AtomicU64,
    ser_restarts: std::sync::atomic::AtomicU64,
    ser_errors: std::sync::atomic::AtomicU64,
    light_tx_frames: std::sync::atomic::AtomicU64,
    light_tx_bytes: std::sync::atomic::AtomicU64,
    light_ack_ok: std::sync::atomic::AtomicU64,
    light_ack_sum_err: std::sync::atomic::AtomicU64,
    light_bad_sum: std::sync::atomic::AtomicU64,
    light_errors: std::sync::atomic::AtomicU64,
}

impl Mai2LoadStats {
    fn bump(counter: &std::sync::atomic::AtomicU64, by: u64) {
        counter.fetch_add(by, std::sync::atomic::Ordering::Relaxed);
    }
    fn get(counter: &std::sync::atomic::AtomicU64) -> u64 {
        counter.load(std::sync::atomic::Ordering::Relaxed)
    }
}

/// 组一帧 mai2light 请求。body = dst,src,len,cmd,payload...; len = command 起到 sum 之前的字节数;
/// sum = **转义前** dst 起逐字节相加取低 8 位; 先算 sum 再对 body 做 0xD0 转义(sum 本身不转义)。
///
/// ★sum 落在 0xE0/0xD0 上必须回避★: sum 不转义, 收端遇 0xE0 会当成新 sync、遇 0xD0 会当成转义前缀,
/// 于是整帧被吃掉。官方板与本固件都有这个协议瑕疵。压测器要量的是"设备在满负载下的行为",
/// 不是这个已知瑕疵, 故此处微调最后一个 payload 字节把 sum 挪开(payload 为空的命令 sum 恒定, 已核过安全)。
fn mai2_light_frame(dst: u8, src: u8, cmd: u8, payload: &[u8]) -> Vec<u8> {
    let mut body = Vec::with_capacity(4 + payload.len());
    body.push(dst);
    body.push(src);
    body.push((1 + payload.len()) as u8);
    body.push(cmd);
    body.extend_from_slice(payload);
    let mut sum = body.iter().fold(0u8, |a, b| a.wrapping_add(*b));
    if (sum == 0xE0 || sum == 0xD0) && !payload.is_empty() {
        let last = body.len() - 1;
        body[last] = body[last].wrapping_add(1);
        sum = sum.wrapping_add(1);
    }
    let mut out = Vec::with_capacity(2 * body.len() + 2);
    out.push(0xE0);
    for b in &body {
        if *b == 0xE0 || *b == 0xD0 {
            out.push(0xD0);
            out.push(b.wrapping_sub(1));
        } else {
            out.push(*b);
        }
    }
    out.push(sum);
    out
}

/// mai2light 应答解析。★与固件 `Mai2Light::_feed` 同一状态机★(sync 重同步 / 0xD0 转义 /
/// len 自描述 / 末字节为 sum)。不能只数 0xE0 —— sum 不转义且可能等于 0xE0, 会把一帧数成两帧。
struct LightAckParser {
    body: Vec<u8>,
    sum: u8,
    active: bool,
    escape: bool,
}

impl LightAckParser {
    fn new() -> Self {
        Self {
            body: Vec::with_capacity(32),
            sum: 0,
            active: false,
            escape: false,
        }
    }
    fn feed(&mut self, byte: u8, stats: &Mai2LoadStats) {
        if byte == 0xE0 {
            self.body.clear();
            self.sum = 0;
            self.active = true;
            self.escape = false;
            return;
        }
        if !self.active {
            return;
        }
        let mut b = byte;
        if b == 0xD0 {
            self.escape = true;
            return;
        }
        if self.escape {
            b = b.wrapping_add(1);
            self.escape = false;
        }
        if self.body.len() >= 4 && self.body.len() == self.body[2] as usize + 3 {
            self.active = false;
            if self.sum == b {
                // body = dst,src,len,status,cmd,report,payload...; status 0x02 = 设备判我们 sum 错。
                if self.body.get(3).copied() == Some(0x02) {
                    Mai2LoadStats::bump(&stats.light_ack_sum_err, 1);
                } else {
                    Mai2LoadStats::bump(&stats.light_ack_ok, 1);
                }
            } else {
                Mai2LoadStats::bump(&stats.light_bad_sum, 1);
            }
            return;
        }
        if self.body.len() >= 48 {
            self.active = false; // 超长必为错位, 丢弃等重同步(与固件一致)
            return;
        }
        self.body.push(b);
        self.sum = self.sum.wrapping_add(b);
    }
}

/// mai2serial 设备→主机帧解析: 触控帧 `(`+7B+`)`, 命令回执 `(`+4B+`)`。
/// 触控载荷每字节只用低 5 位(0..31), 不可能撞上 '('(0x28)/')'(0x29) ⇒ 按定界符切帧不会错帧。
struct TouchFrameParser {
    active: bool,
    len: usize,
}

impl TouchFrameParser {
    fn new() -> Self {
        Self {
            active: false,
            len: 0,
        }
    }
    fn feed(&mut self, b: u8, stats: &Mai2LoadStats) {
        if b == b'(' {
            self.active = true;
            self.len = 0;
            return;
        }
        if !self.active {
            return;
        }
        if b == b')' {
            self.active = false;
            if self.len == 7 {
                Mai2LoadStats::bump(&stats.ser_touch_frames, 1);
            } else {
                Mai2LoadStats::bump(&stats.ser_cmd_resp, 1);
            }
            return;
        }
        self.len += 1;
        if self.len > 16 {
            self.active = false;
        }
    }
}

/// 线程A: mai2serial。起流 + 持续读触控帧 + 周期性启停/快速重启扰动。
/// 收尾必须 `{HALT}` —— 否则设备被留在 RUNNING, 后续测试的"未启用发送"基线就不对了。
fn mai2_load_serial_thread(
    port_name: String,
    baud: u32,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    stats: std::sync::Arc<Mai2LoadStats>,
) {
    use std::io::{Read, Write};
    let mut port = match serialport::new(&port_name, baud)
        .timeout(Duration::from_millis(20))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            println!("[LOAD] 线程A 打开 {} 失败: {}", port_name, e);
            Mai2LoadStats::bump(&stats.ser_errors, 1);
            return;
        }
    };
    let _ = port.write_all(b"{STAT}");
    let mut parser = TouchFrameParser::new();
    let mut buf = [0u8; 4096];
    let started = std::time::Instant::now();
    let mut next_halt = Duration::from_millis(3000);
    let mut next_reset = Duration::from_millis(11000);
    // ★错误必须退避并封顶★: 设备一掉线, 句柄上的每次 read/write 都立刻返回错误, 不退避就会在
    // 几秒里刷出几百万次计数(首轮实测 543 万), 既污染统计又把 CPU 占满、干扰同机的 vendor 一路。
    let mut consecutive_errors = 0u32;
    const ERROR_GIVE_UP: u32 = 40;
    while !stop.load(std::sync::atomic::Ordering::Relaxed) {
        match port.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                consecutive_errors = 0;
                Mai2LoadStats::bump(&stats.ser_rx_bytes, n as u64);
                for b in &buf[..n] {
                    parser.feed(*b, &stats);
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                consecutive_errors = 0;
            }
            Err(e) => {
                Mai2LoadStats::bump(&stats.ser_errors, 1);
                consecutive_errors += 1;
                if consecutive_errors <= 3 {
                    println!("[LOAD] 线程A 读错误: {}", e);
                }
                if consecutive_errors >= ERROR_GIVE_UP {
                    println!(
                        "[LOAD] 线程A 连续 {} 次错误, 判端口已失效, 退出。",
                        ERROR_GIVE_UP
                    );
                    return;
                }
                thread::sleep(Duration::from_millis(50));
            }
        }
        let now = started.elapsed();
        // 启停扰动: {HALT} 停发 → 200ms 后 {STAT} 续发。命令字符在索引 3, 直接用真实 ASCII 命令。
        if now >= next_halt {
            next_halt = now + Duration::from_millis(3000);
            if port.write_all(b"{HALT}").is_ok() {
                Mai2LoadStats::bump(&stats.ser_halts, 1);
            }
            thread::sleep(Duration::from_millis(200));
            let _ = port.write_all(b"{STAT}");
        }
        // 快速重启扰动: {RSET} 令协议层回 READY 且停发, 立刻再 {STAT} 起流。
        if now >= next_reset {
            next_reset = now + Duration::from_millis(11000);
            if port.write_all(b"{RSET}").is_ok() {
                Mai2LoadStats::bump(&stats.ser_restarts, 1);
            }
            thread::sleep(Duration::from_millis(120));
            let _ = port.write_all(b"{STAT}");
        }
    }
    let _ = port.write_all(b"{HALT}");
    let _ = port.flush();
}

/// 线程B: mai2light。满速灌请求帧并读应答, 覆盖单灯/多灯/提交/查询四类命令。
fn mai2_load_light_thread(
    port_name: String,
    baud: u32,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    stats: std::sync::Arc<Mai2LoadStats>,
) {
    use std::io::{Read, Write};
    let mut port = match serialport::new(&port_name, baud)
        .timeout(Duration::from_millis(5))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            println!("[LOAD] 线程B 打开 {} 失败: {}", port_name, e);
            Mai2LoadStats::bump(&stats.light_errors, 1);
            return;
        }
    };
    let mut parser = LightAckParser::new();
    let mut buf = [0u8; 4096];
    let mut tick: u32 = 0;
    // 同线程A: 错误退避 + 封顶, 否则掉线后会空转刷千万级计数。
    let mut consecutive_errors = 0u32;
    const ERROR_GIVE_UP: u32 = 40;
    while !stop.load(std::sync::atomic::Ordering::Relaxed) {
        // 每轮一组: 单灯 → 多灯 → 提交 → (每 32 轮插一次板状态查询, 走带 payload 应答的分支)。
        let phase = (tick % 8) as u8;
        let color = [
            (tick & 0xFF) as u8,
            ((tick >> 3) & 0xFF) as u8,
            ((tick >> 5) & 0xFF) as u8,
        ];
        let mut frames: Vec<Vec<u8>> = Vec::with_capacity(4);
        frames.push(mai2_light_frame(
            0,
            0,
            0x31,
            &[phase % 8, color[0], color[1], color[2]],
        ));
        frames.push(mai2_light_frame(
            0,
            0,
            0x32,
            &[0, 0x20, 0, color[2], color[0], color[1]],
        ));
        frames.push(mai2_light_frame(0, 0, 0x3C, &[]));
        if tick % 32 == 0 {
            frames.push(mai2_light_frame(0, 0, 0xF1, &[]));
        }
        let mut errored = false;
        for frame in &frames {
            match port.write_all(frame) {
                Ok(()) => {
                    Mai2LoadStats::bump(&stats.light_tx_frames, 1);
                    Mai2LoadStats::bump(&stats.light_tx_bytes, frame.len() as u64);
                }
                Err(e) => {
                    Mai2LoadStats::bump(&stats.light_errors, 1);
                    errored = true;
                    if consecutive_errors < 3 {
                        println!("[LOAD] 线程B 写错误: {}", e);
                    }
                }
            }
        }
        match port.read(&mut buf) {
            Ok(n) => {
                for b in &buf[..n] {
                    parser.feed(*b, &stats);
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                Mai2LoadStats::bump(&stats.light_errors, 1);
                errored = true;
                if consecutive_errors < 3 {
                    println!("[LOAD] 线程B 读错误: {}", e);
                }
            }
        }
        if errored {
            consecutive_errors += 1;
            if consecutive_errors >= ERROR_GIVE_UP {
                println!(
                    "[LOAD] 线程B 连续 {} 次错误, 判端口已失效, 退出。",
                    ERROR_GIVE_UP
                );
                return;
            }
            thread::sleep(Duration::from_millis(50));
        } else {
            consecutive_errors = 0;
        }
        tick = tick.wrapping_add(1);
    }
    // 收尾: 全灯灭 + 提交, 别把灯留在压测色上。
    let _ = port.write_all(&mai2_light_frame(0, 0, 0x32, &[0, 0x20, 0, 0, 0, 0]));
    let _ = port.write_all(&mai2_light_frame(0, 0, 0x3C, &[]));
    let _ = port.flush();
}

/// 压测通道开关。三路同压能复现故障, 但复现不等于定位 —— 必须能逐路关掉做二分。
#[derive(Clone, Copy)]
struct Mai2LoadLanes {
    serial: bool,
    light: bool,
    vendor: bool,
    heavy: bool,
}

fn run_mai2_load(
    ctrl: &mut AppController,
    port_name: &str,
    seconds: u64,
    lanes: Mai2LoadLanes,
) -> ! {
    use mai2control_ui::comport::{self, CdcFunction};
    use mai2control_ui::proto::{FIELD_BASELINE, FIELD_DIFF, FIELD_RAW, FIELD_STATUS};

    println!(
        "[LOAD] 压测 {}s | serial={} light={} vendor={}{}",
        seconds,
        lanes.serial,
        lanes.light,
        lanes.vendor,
        if lanes.heavy {
            " + 长周期 PSoC 指令堆叠(--heavy)"
        } else {
            ""
        }
    );

    let ports = comport::identify_ports();
    let pick = |f: CdcFunction| -> Option<String> {
        ports
            .iter()
            .find(|p| p.function == f)
            .map(|p| p.port_name.clone())
    };
    let serial_port = if lanes.serial {
        pick(CdcFunction::Serial)
    } else {
        None
    };
    let light_port = if lanes.light {
        pick(CdcFunction::Light)
    } else {
        None
    };
    println!(
        "[LOAD] 端口: mai2serial={} mai2light={}",
        serial_port.as_deref().unwrap_or("未识别"),
        light_port.as_deref().unwrap_or("未识别")
    );
    if serial_port.is_none() && light_port.is_none() && !lanes.vendor {
        println!("[LOAD] FAIL 三路全关, 无事可压");
        std::process::exit(1);
    }
    if lanes.serial && lanes.light && serial_port.is_none() && light_port.is_none() {
        println!("[LOAD] FAIL 两条 CDC 都没识别到(设备需处于 serial 工作模式)");
        std::process::exit(1);
    }

    // 波特率取设备真值(KV), 与固件实际配置同源; 缺省 115200。
    let baud_of = |key: &str| -> u32 {
        match ctrl.config_get(key).map(|e| e.value) {
            Some(CfgValue::U32(v)) => v,
            Some(CfgValue::U16(v)) => v as u32,
            _ => 115_200,
        }
    };
    let serial_baud = baud_of("comm.serial_baud");
    let light_baud = baud_of("comm.light_baud");
    println!("[LOAD] 波特率: serial={} light={}", serial_baud, light_baud);

    // 干净窗口: 峰值量不可差分, 先清零再压。
    if let Err(e) = ctrl.clear_loop_profile() {
        println!(
            "[LOAD] FAIL 清零主循环剖面失败(固件是否为带剖面的新版本?): {}",
            e
        );
        std::process::exit(1);
    }
    let base = match read_loop_profile(ctrl) {
        Some(p) if p.loop_max_us < 200_000 => p,
        Some(p) => {
            println!(
                "[LOAD] 注意: 清零后 loop_max_us 仍为 {}us(清零到读取之间已跑过长轮), 继续。",
                p.loop_max_us
            );
            p
        }
        None => {
            println!("[LOAD] FAIL 诊断结构过短, 固件不含主循环剖面字段 —— 请先烧录本轮固件。");
            std::process::exit(1);
        }
    };
    println!(
        "[LOAD] 基线: loop_count={} rx_dropped={} heavy_rejects={}",
        base.loop_count, base.rx_dropped, base.heavy_rejects
    );

    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let stats = std::sync::Arc::new(Mai2LoadStats::default());
    let mut workers = Vec::new();
    if let Some(name) = serial_port {
        let (s, st) = (stop.clone(), stats.clone());
        workers.push(thread::spawn(move || {
            mai2_load_serial_thread(name, serial_baud, s, st)
        }));
    }
    if let Some(name) = light_port {
        let (s, st) = (stop.clone(), stats.clone());
        workers.push(thread::spawn(move || {
            mai2_load_light_thread(name, light_baud, s, st)
        }));
    }

    // 主线程(vendor): 遥测拉满 + 轮询参数 + 周期采剖面。★AppController 不跨线程★ —— 它持有的
    // IoHandle 与 nusb 会话按单线程使用设计, 故 vendor 一路固定留在主线程, 只有两条 CDC 开子线程。
    if lanes.vendor {
        let _ = ctrl.start_telemetry(
            250,
            FIELD_RAW | FIELD_BASELINE | FIELD_DIFF | FIELD_STATUS,
            u64::MAX,
        );
    }
    let t0 = std::time::Instant::now();
    let total = Duration::from_secs(seconds);
    let mut last_sample = std::time::Instant::now();
    let mut last_param = std::time::Instant::now();
    let mut last_heavy = std::time::Instant::now();
    let mut last_loop_count = base.loop_count;
    let mut reboots = 0u32;
    let mut disconnects = 0u32;
    let mut worst = base;
    let mut heavy_sent = 0u32;
    let mut heavy_tune_fired = false;
    let mut param_polls = 0u32;
    let param_ids: [u8; 4] = [0x01, 0x08, 0x0B, 0x07];

    while t0.elapsed() < total {
        ctrl.poll();
        // ★掉线即收尾★: 继续跑只是在死链路上空转, 而死前遗言(scratch)会被下一次复位覆盖,
        // 越早去读越可信。判据已经成立, 没有再压下去的价值。
        if ctrl.state() == ConnState::Disconnected {
            disconnects += 1;
            println!(
                "[LOAD] ★掉线★ @{}s —— 已达判据, 提前收尾去读死前遗言",
                t0.elapsed().as_secs()
            );
            break;
        }
        if lanes.vendor && last_param.elapsed() >= Duration::from_millis(100) {
            last_param = std::time::Instant::now();
            let id = param_ids[(param_polls as usize) % param_ids.len()];
            let _ = ctrl.request_param_all_channels(id);
            param_polls += 1;
        }
        // --heavy: 故意堆叠长周期指令。用 CP_MEASURE 而不是 AUTO_TUNE —— 两者走同一条反堆叠闸门,
        // 但 CP 测量不改写任何持久参数, 而自适应会重写 SNS_CLK_DIV(上一轮 nv-soak 写坏配置的教训)。
        // ★真正制造堆叠★: CP 测量是"读类"提交(core0 等到完成才返回), 天然自我串行, 拒不了自己 ——
        // 只用它压, 闸门永远是 0 次拒绝, 等于没测。频率自适应才是异步长周期(20s+)的那一类:
        // 开一条在途, 后续每条 CP 测量都必须被回 DEVICE_BUSY。这也正是用户崩溃日志里的场景。
        if lanes.heavy && !heavy_tune_fired && t0.elapsed() >= Duration::from_secs(2) {
            heavy_tune_fired = true;
            match ctrl.auto_tune(0) {
                Ok(()) => {
                    println!("[LOAD] 已发起 ch0 频率自适应(长周期在途), 后续长周期指令应被闸门拒绝")
                }
                Err(e) => println!("[LOAD] 频率自适应发起失败: {}", e),
            }
        }
        if lanes.heavy && last_heavy.elapsed() >= Duration::from_millis(400) {
            last_heavy = std::time::Instant::now();
            let _ = ctrl.measure_cp();
            heavy_sent += 1;
        }
        if last_sample.elapsed() >= Duration::from_millis(500) {
            last_sample = std::time::Instant::now();
            if let Some(p) = read_loop_profile(ctrl) {
                // loop_count 回退 = 设备重启(计数器从 0 重来)。ms 类时基会被 PSoC 复位干扰, 这个不会。
                if p.loop_count < last_loop_count {
                    reboots += 1;
                    println!(
                        "[LOAD] ★设备重启★ @{}s (loop_count {} → {})",
                        t0.elapsed().as_secs(),
                        last_loop_count,
                        p.loop_count
                    );
                    worst = p; // 重启已清零剖面, 峰值从新窗口重新累计
                } else {
                    if p.loop_max_us > worst.loop_max_us {
                        worst.loop_max_us = p.loop_max_us;
                    }
                    for i in 0..8 {
                        if p.seg_max_us[i] > worst.seg_max_us[i] {
                            worst.seg_max_us[i] = p.seg_max_us[i];
                        }
                        if p.seg2_max_us[i] > worst.seg2_max_us[i] {
                            worst.seg2_max_us[i] = p.seg2_max_us[i];
                        }
                    }
                }
                worst.rx_dropped = p.rx_dropped;
                worst.heavy_rejects = p.heavy_rejects;
                worst.heavy_busy = p.heavy_busy;
                worst.nv_commit_fail = p.nv_commit_fail;
                last_loop_count = p.loop_count;
            }
        }
        thread::sleep(Duration::from_millis(4));
    }

    stop.store(true, std::sync::atomic::Ordering::Relaxed);
    for w in workers {
        let _ = w.join();
    }
    let _ = ctrl.stop_telemetry();
    ctrl.poll();

    let elapsed = t0.elapsed().as_secs_f64().max(0.001);
    let io = ctrl.io_stats();
    println!("[LOAD] ---- mai2serial ----");
    println!(
        "[LOAD] rx={}B 触控帧={} ({:.0} 帧/s) 命令回执={} 启停={} 快速重启={} 错误={}",
        Mai2LoadStats::get(&stats.ser_rx_bytes),
        Mai2LoadStats::get(&stats.ser_touch_frames),
        Mai2LoadStats::get(&stats.ser_touch_frames) as f64 / elapsed,
        Mai2LoadStats::get(&stats.ser_cmd_resp),
        Mai2LoadStats::get(&stats.ser_halts),
        Mai2LoadStats::get(&stats.ser_restarts),
        Mai2LoadStats::get(&stats.ser_errors)
    );
    println!("[LOAD] ---- mai2light ----");
    println!(
        "[LOAD] tx={}帧/{}B ({:.0} 帧/s) 应答ok={} 设备判我sum错={} 我判设备sum错={} 错误={}",
        Mai2LoadStats::get(&stats.light_tx_frames),
        Mai2LoadStats::get(&stats.light_tx_bytes),
        Mai2LoadStats::get(&stats.light_tx_frames) as f64 / elapsed,
        Mai2LoadStats::get(&stats.light_ack_ok),
        Mai2LoadStats::get(&stats.light_ack_sum_err),
        Mai2LoadStats::get(&stats.light_bad_sum),
        Mai2LoadStats::get(&stats.light_errors)
    );
    println!("[LOAD] ---- vendor ----");
    println!(
        "[LOAD] rx={}B tx={}B stall恢复={} 队列丢弃={} 遥测帧={} 参数轮询={}",
        io.bytes_read,
        io.bytes_written,
        io.stall_recoveries,
        io.queue_dropped,
        ctrl.telem_frame_count(),
        param_polls
    );
    println!("[LOAD] ---- 主循环阻塞剖面(核心指标) ----");
    let margin = WATCHDOG_BUDGET_US.saturating_sub(worst.loop_max_us);
    println!(
        "[LOAD] 整轮最长 = {}us ({:.1}ms) → 距 5s 看门狗余量 {}us ({:.1}ms, 占用 {:.2}%)",
        worst.loop_max_us,
        worst.loop_max_us as f64 / 1000.0,
        margin,
        margin as f64 / 1000.0,
        worst.loop_max_us as f64 * 100.0 / WATCHDOG_BUDGET_US as f64
    );
    let mut ranked: Vec<(usize, u32)> = worst.seg_max_us.iter().copied().enumerate().collect();
    ranked.sort_by(|a, b| b.1.cmp(&a.1));
    for (idx, us) in &ranked {
        println!(
            "[LOAD]   {:<9} 最长 {:>8}us ({:.1}ms)",
            LOOP_SEG_NAMES[*idx],
            us,
            *us as f64 / 1000.0
        );
    }
    let mut ranked2: Vec<(usize, u32)> = worst.seg2_max_us.iter().copied().enumerate().collect();
    ranked2.sort_by(|a, b| b.1.cmp(&a.1));
    for (idx, us) in ranked2.iter().take(4) {
        println!(
            "[LOAD]   └ game_io/{:<11} 最长 {:>8}us ({:.1}ms)",
            GAMEIO_SEG_NAMES[*idx],
            us,
            *us as f64 / 1000.0
        );
    }
    println!(
        "[LOAD] 重启={} 掉线={} vendor_rx_dropped={}(基线 {}) nv_commit_fail={} 最后成功采样 loop_count={}",
        reboots,
        disconnects,
        worst.rx_dropped,
        base.rx_dropped,
        worst.nv_commit_fail,
        last_loop_count
    );
    println!(
        "[LOAD] 反堆叠闸门: 拒绝累计={}(基线 {}, 本轮 +{}) 当前在途={} 主动发起长周期指令={}",
        worst.heavy_rejects,
        base.heavy_rejects,
        worst.heavy_rejects.saturating_sub(base.heavy_rejects),
        worst.heavy_busy,
        heavy_sent
    );
    // 死前遗言必须**独立开句柄**读: 掉线后 AppController 的会话已废, 而 EP0 在 bulk 死后仍存活。
    // 掉线场景下这是唯一还能问到"设备当时怎么死的"的通道。
    // ★掉线 ≠ 设备重启★: 掉线只说明 vendor 会话没了; 设备是否真的重启, 唯一可信判据是 loop_count
    // 相对基线是否回退(计数器只增, 复位归零)。此前把两者混为一谈, 白追了一轮。
    // 重新枚举需要时间, 故重试几轮而不是一次失败就放弃。
    // 会话还活着就用它读: 另开句柄会与本进程已 claim 的接口冲突而失败(不是设备的问题)。
    let mut fresh: Option<Vec<u8>> = if ctrl.state() != ConnState::Disconnected {
        ctrl.read_debug_counters().ok()
    } else {
        None
    };
    if fresh.is_none() {
        for _ in 0..12 {
            match io::read_debug(port_name) {
                Ok(bytes) => {
                    fresh = Some(bytes);
                    break;
                }
                Err(_) => thread::sleep(Duration::from_millis(500)),
            }
        }
    }
    match fresh {
        Some(bytes) => {
            if bytes.len() >= 8 {
                let now_loop = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
                println!(
                    "[LOAD] 设备是否重启: loop_count 基线={} 现在={} → {}",
                    base.loop_count,
                    now_loop,
                    if now_loop < base.loop_count {
                        "★已重启★"
                    } else {
                        "未重启(计数器连续, 掉线只是 vendor 会话断开)"
                    }
                );
            }
            print_post_mortem("[LOAD]", &bytes);
        }
        None => println!("[LOAD] 设备迟迟未重新枚举, 死前遗言无法读取"),
    }

    // 判据: 重启/掉线是硬失败; rx_dropped 增长 = core0 曾长时间没来取命令(命令被截断), 同样是失败。
    let mut fails: Vec<String> = Vec::new();
    if reboots > 0 {
        fails.push(format!("设备重启 {} 次", reboots));
    }
    if disconnects > 0 {
        fails.push(format!("USB 掉线 {} 次", disconnects));
    }
    if worst.rx_dropped > base.rx_dropped {
        fails.push(format!(
            "vendor_rx_dropped 增长 {}(core0 曾长时间未取命令, 主机命令被截断)",
            worst.rx_dropped - base.rx_dropped
        ));
    }
    if worst.loop_max_us >= WATCHDOG_BUDGET_US / 2 {
        fails.push(format!(
            "整轮最长 {}us 已过看门狗预算一半(余量不足)",
            worst.loop_max_us
        ));
    }
    if fails.is_empty() {
        println!("[LOAD] PASS");
        std::process::exit(0);
    }
    println!("[LOAD] FAIL: {}", fails.join(" | "));
    std::process::exit(1);
}

/// 验收模式的统一轮询等待器：只把控制器实际回读版本作为完成判据，不读取乐观草稿。
fn _acceptance_wait<F>(ctrl: &mut AppController, timeout_ms: u64, mut done: F) -> Result<(), String>
where
    F: FnMut(&mut AppController) -> bool,
{
    let deadline = std::time::Instant::now() + Duration::from_millis(timeout_ms);
    while std::time::Instant::now() < deadline {
        ctrl.poll();
        ctrl.csd_diag_tick();
        if ctrl.state() == ConnState::Disconnected {
            return Err("设备断开".to_string());
        }
        if let Some(error) = ctrl.last_error() {
            return Err(format!("控制器错误: {}", error));
        }
        if done(ctrl) {
            return Ok(());
        }
        thread::sleep(Duration::from_millis(5));
    }
    Err(format!("{}ms 内未收到设备回读", timeout_ms))
}

fn _acceptance_request<F, R>(
    ctrl: &mut AppController,
    label: &str,
    request: R,
    mut done: F,
) -> Result<(), String>
where
    F: FnMut(&mut AppController) -> bool,
    R: FnOnce(&mut AppController) -> anyhow::Result<()>,
{
    request(ctrl).map_err(|error| format!("{} 请求失败: {}", label, error))?;
    _acceptance_wait(ctrl, ACCEPTANCE_REQUEST_TIMEOUT_MS, |ctrl| done(ctrl))
        .map_err(|error| format!("{}: {}", label, error))
}
fn _run_acceptance(ctrl: &mut AppController, connect_started: std::time::Instant) -> bool {
    let started = connect_started;
    let context = mai2control_ui::app_state::PollContext {
        connected: true,
        current_view: 1,
        settings_tab: 8,
        hid_mode: false,
        sel_channel: 0,
        light_panel_expanded: false,
        phys_la_expanded: false,
    };
    ctrl.schedule_conn_probes(0);
    let mut tick = 0u32;
    let ready = _acceptance_wait(ctrl, ACCEPTANCE_READY_TIMEOUT_MS, |ctrl| {
        tick = tick.wrapping_add(1);
        ctrl.poll_scheduled(&context, tick, 5);
        !ctrl.conn_probes_pending()
            && ctrl.config_version() > 0
            && ctrl.algo_version() > 0
            && ctrl.globals_version() > 0
    });
    let ready_ms = started.elapsed().as_millis();
    if let Err(error) = ready {
        println!("[ACCEPTANCE] READY FAIL elapsed_ms={} {}", ready_ms, error);
        return false;
    }
    println!("[ACCEPTANCE] READY PASS elapsed_ms={}", ready_ms);

    let param_id = mai2control_ui::proto::PARAM_ON_DEBOUNCE;
    let baseline_version = ctrl.param_version();
    if let Err(error) = ctrl.request_param(0, param_id) {
        println!("[ACCEPTANCE] PARAM baseline FAIL: {}", error);
        return false;
    }
    if let Err(error) = _acceptance_wait(ctrl, ACCEPTANCE_REQUEST_TIMEOUT_MS, |ctrl| {
        ctrl.param_version() > baseline_version && ctrl.param(0, param_id).is_some()
    }) {
        println!("[ACCEPTANCE] PARAM baseline FAIL: {}", error);
        return false;
    }
    let Some(original) = ctrl.param(0, param_id) else {
        println!("[ACCEPTANCE] PARAM baseline FAIL: 无设备回读值");
        return false;
    };
    let alternate = if original < 0xFF {
        original + 1
    } else {
        original - 1
    };
    let io_before = ctrl.io_stats();
    let roundtrip_start = std::time::Instant::now();
    let mut rounds = 0usize;
    let mut max_cycle_ms = 0u128;
    let mut failures = Vec::new();
    for round in 0..ACCEPTANCE_ROUNDS {
        let expected = if round % 2 == 0 { alternate } else { original };
        let cycle_start = std::time::Instant::now();
        if let Err(error) = ctrl.debug_param_now(0, param_id, expected) {
            failures.push(format!("第{}轮 SET: {}", round + 1, error));
            break;
        }
        if let Err(error) = _acceptance_wait(ctrl, 1_000, |ctrl| ctrl.cfg_tx_pending() == 0) {
            failures.push(format!("第{}轮 ACK: {}", round + 1, error));
            break;
        }
        let before_get = ctrl.param_version();
        if let Err(error) = ctrl.request_param(0, param_id) {
            failures.push(format!("第{}轮 GET: {}", round + 1, error));
            break;
        }
        if let Err(error) = _acceptance_wait(ctrl, 1_000, |ctrl| {
            ctrl.param_version() > before_get && ctrl.param(0, param_id) == Some(expected)
        }) {
            failures.push(format!(
                "第{}轮回读: {} 实际={:?}",
                round + 1,
                error,
                ctrl.param(0, param_id)
            ));
            break;
        }
        rounds += 1;
        max_cycle_ms = max_cycle_ms.max(cycle_start.elapsed().as_millis());
        let elapsed = cycle_start.elapsed();
        if elapsed < Duration::from_millis(ACCEPTANCE_PERIOD_MS) {
            thread::sleep(Duration::from_millis(ACCEPTANCE_PERIOD_MS) - elapsed);
        }
    }
    let roundtrip_ms = roundtrip_start.elapsed().as_millis();
    let io_after = ctrl.io_stats();
    let hz = if roundtrip_ms == 0 {
        0.0
    } else {
        rounds as f64 * 1000.0 / roundtrip_ms as f64
    };
    let io_clean = io_after.queue_dropped == io_before.queue_dropped
        && io_after.stall_recoveries == io_before.stall_recoveries;
    println!(
        "[ACCEPTANCE] PARAM rounds={} duration_ms={} hz={:.2} max_cycle_ms={} queue_dropped_delta={} stall_recoveries_delta={}",
        rounds,
        roundtrip_ms,
        hz,
        max_cycle_ms,
        io_after
            .queue_dropped
            .saturating_sub(io_before.queue_dropped),
        io_after
            .stall_recoveries
            .saturating_sub(io_before.stall_recoveries)
    );
    if let Err(error) = ctrl.debug_param_now(0, param_id, original) {
        failures.push(format!("恢复原值 SET: {}", error));
    } else if let Err(error) = _acceptance_wait(ctrl, 1_000, |ctrl| ctrl.cfg_tx_pending() == 0) {
        failures.push(format!("恢复原值 ACK: {}", error));
    } else {
        let before_restore = ctrl.param_version();
        let _ = ctrl.request_param(0, param_id);
        if let Err(error) = _acceptance_wait(ctrl, 1_000, |ctrl| {
            ctrl.param_version() > before_restore && ctrl.param(0, param_id) == Some(original)
        }) {
            failures.push(format!("恢复原值回读: {}", error));
        }
    }
    if rounds != ACCEPTANCE_ROUNDS || hz < 10.0 || !io_clean {
        failures.push("10Hz 零错误门限未满足".to_string());
    }

    let mut matrix = Vec::new();
    let before = ctrl.config_version();
    if let Err(error) = _acceptance_request(
        ctrl,
        "CFG_GET_ALL",
        |ctrl| ctrl.request_config_all(),
        |ctrl| ctrl.config_version() > before && !ctrl.config_entries().is_empty(),
    ) {
        matrix.push(error);
    }
    for (label, request, version) in [
        ("KBD_GET_STATE", 0u8, 0u8),
        ("KBD_GET_MAP", 1, 0),
        ("KBD_GET_TOUCHMAP", 2, 0),
        ("KBD_GET_HOLD", 3, 0),
        ("KBD_GET_KEYCFG", 4, 0),
        ("KBD_GET_COMBO", 5, 0),
    ] {
        let before_version = match version {
            0 => ctrl.kbd_state_version(),
            _ => 0,
        };
        let result = match request {
            0 => _acceptance_request(
                ctrl,
                label,
                |ctrl| ctrl.kbd_request_state(),
                |ctrl| ctrl.kbd_state_version() > before_version,
            ),
            1 => {
                let v = ctrl.kbd_map_version();
                _acceptance_request(
                    ctrl,
                    label,
                    |ctrl| ctrl.kbd_request_map(),
                    |ctrl| ctrl.kbd_map_version() > v,
                )
            }
            2 => {
                let v = ctrl.kbd_touchmap_version();
                _acceptance_request(
                    ctrl,
                    label,
                    |ctrl| ctrl.kbd_request_touchmap(),
                    |ctrl| ctrl.kbd_touchmap_version() > v,
                )
            }
            3 => {
                let v = ctrl.kbd_hold_version();
                _acceptance_request(
                    ctrl,
                    label,
                    |ctrl| ctrl.kbd_request_hold(),
                    |ctrl| ctrl.kbd_hold_version() > v,
                )
            }
            4 => {
                let v = ctrl.kbd_keycfg_version();
                _acceptance_request(
                    ctrl,
                    label,
                    |ctrl| ctrl.kbd_request_keycfg(),
                    |ctrl| ctrl.kbd_keycfg_version() > v,
                )
            }
            _ => {
                let v = ctrl.kbd_combo_version();
                _acceptance_request(
                    ctrl,
                    label,
                    |ctrl| ctrl.kbd_request_combo(),
                    |ctrl| ctrl.kbd_combo_version() > v,
                )
            }
        };
        if let Err(error) = result {
            matrix.push(error);
        }
    }
    for id in mai2control_ui::proto::KNOWN_PARAM_IDS {
        let before = ctrl.param_version();
        if let Err(error) = _acceptance_request(
            ctrl,
            &format!("PARAM_GET_ALL 0x{:02X}", id),
            |ctrl| ctrl.request_param_all_channels(*id),
            |ctrl| ctrl.param_version() > before,
        ) {
            matrix.push(error);
        }
    }
    let before = ctrl.globals_version();
    if let Err(error) = _acceptance_request(
        ctrl,
        "GLOBAL_GET_ALL",
        |ctrl| ctrl.global_get_all(),
        |ctrl| ctrl.globals_version() > before,
    ) {
        matrix.push(error);
    }
    let before = ctrl.algo_version();
    if let Err(error) = _acceptance_request(
        ctrl,
        "ALGO_GET_INFO",
        |ctrl| ctrl.algo_get_info(),
        |ctrl| ctrl.algo_version() > before,
    ) {
        matrix.push(error);
    }
    let before = ctrl.algo_rom_version();
    if let Err(error) = _acceptance_request(
        ctrl,
        "ALGO_GET_ROM",
        |ctrl| ctrl.algo_get_rom(),
        |ctrl| ctrl.algo_rom_version() > before && !ctrl.algo_rom().is_empty(),
    ) {
        matrix.push(error);
    }
    for idx in 0..8u8 {
        let before = ctrl.algo_cfg_version();
        if let Err(error) = _acceptance_request(
            ctrl,
            &format!("ALGO_GET_CFG {}", idx),
            |ctrl| ctrl.request_algo_cfg(idx),
            |ctrl| ctrl.algo_cfg_version() > before,
        ) {
            matrix.push(error);
        }
    }
    let before = ctrl.algo_device_src_version();
    if let Err(error) = _acceptance_request(
        ctrl,
        "ALGO_GET_SRC",
        |ctrl| ctrl.request_algo_src(),
        |ctrl| ctrl.algo_device_src_version() > before && !ctrl.algo_src_transfer_pending(),
    ) {
        matrix.push(error);
    }
    let before = ctrl.algo_device_code_version();
    if let Err(error) = _acceptance_request(
        ctrl,
        "ALGO_GET_CODE",
        |ctrl| ctrl.request_algo_code(),
        |ctrl| ctrl.algo_device_code_version() > before && !ctrl.algo_device_code_hex().is_empty(),
    ) {
        matrix.push(error);
    }
    println!(
        "[ACCEPTANCE] MATRIX {}",
        if matrix.is_empty() { "PASS" } else { "FAIL" }
    );
    for error in &matrix {
        println!("[ACCEPTANCE] MATRIX-ERROR {}", error);
    }
    failures.extend(matrix);
    if failures.is_empty() {
        println!("[ACCEPTANCE] PASS");
        true
    } else {
        println!("[ACCEPTANCE] FAIL {}", failures.join(" | "));
        false
    }
}

fn main() {
    env_logger::init();

    let args: Vec<String> = std::env::args().collect();
    // 只读枚举 Raw Input 与 Interception 槽位：不连接下位机、不改驱动、不启用过滤器。
    if let Some(probe_index) = args.iter().position(|arg| arg == "--interception-probe") {
        let filter = args
            .get(probe_index + 1)
            .filter(|value| !value.starts_with("--"))
            .map(String::as_str);
        println!("{}", mai2control_ui::vcam::interception_diagnostic(filter));
        std::process::exit(0);
    }
    // 纯消费侧排查: 不造生产者, 专门验证"上位机在发帧 → 别的进程取得到帧"这条真实路径。
    if args.iter().any(|a| a == "--vcam-consume") {
        std::process::exit(if _run_vcam_consume() { 0 } else { 1 });
    }
    // 帧服务器眼里到底有哪些摄像头。★这是区分"我方 DirectShow 摄像头"与"旧实现残留的 MF 幽灵
    // 相机"的唯一手段★ 两者在 Windows 设置里显示成同一个名字, 只能靠这份枚举结果对上号。
    if args.iter().any(|a| a == "--vcam-mf-list") {
        match mai2control_ui::vcam::backend::mf_video_sources() {
            Ok(names) => {
                println!("[VCAM] 帧服务器列出 {} 个视频采集源:", names.len());
                for name in &names {
                    println!("  · {}", name);
                }
                std::process::exit(0);
            }
            Err(error) => {
                println!("[VCAM] mf-list FAIL: {}", error);
                std::process::exit(1);
            }
        }
    }
    // 虚拟摄像头探测不依赖 WinUSB 固件，必须在设备枚举之前独立退出。
    if args.iter().any(|a| a == "--vcam-probe") {
        std::process::exit(if _run_vcam_probe() { 0 } else { 1 });
    }
    // 复用 UI 的真实部署事务，供无头验收双位宽注册/卸载；两条路径都会独立核验注册表与文件。
    if args.iter().any(|a| a == "--vcam-install") {
        match mai2control_ui::vcam::backend::install() {
            Ok(status) => {
                println!("[VCAM] install PASS: {}", status);
                std::process::exit(0);
            }
            Err(error) => {
                println!("[VCAM] install FAIL: {}", error);
                std::process::exit(1);
            }
        }
    }
    if args.iter().any(|a| a == "--vcam-uninstall") {
        match mai2control_ui::vcam::backend::uninstall() {
            Ok(status) => {
                println!("[VCAM] uninstall PASS: {}", status);
                std::process::exit(0);
            }
            Err(error) => {
                println!("[VCAM] uninstall FAIL: {}", error);
                std::process::exit(1);
            }
        }
    }
    let acceptance = args.iter().any(|a| a == "--acceptance");
    let review_closure = args.iter().any(|a| a == "--review-closure");
    let reboot_bootloader = args.iter().any(|a| a == "--reboot-bootloader");
    let reboot_bootloader_only = args.iter().any(|a| a == "--reboot-bootloader-only");
    let smoke_only = args.iter().any(|a| a == "--smoke");
    let diagnose_only = args.iter().any(|a| a == "--diagnose");
    let csd_provision = args.iter().any(|a| a == "--csd-provision");
    let csd_verify = args.iter().any(|a| a == "--csd-verify");
    let algo_test = args.iter().any(|a| a == "--algo");
    let global_test = args.iter().any(|a| a == "--global");
    let kbd_test = args.iter().any(|a| a == "--kbd");
    let bus_test = args.iter().any(|a| a == "--bus");
    // --mai2-load [秒]: 两条 mai2 CDC + vendor 三路同时顶流, 量主循环最长阻塞与看门狗余量。
    // 追加 --heavy 则同时故意堆叠长周期 PSoC 指令, 检验反堆叠闸门。
    let mai2_load = args.iter().any(|a| a == "--mai2-load");
    let mai2_load_secs: u64 = args
        .iter()
        .position(|a| a == "--mai2-load")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(60);
    // 逐路开关: 复现之后必须能二分定位, 否则三路同压只能证明"有问题"不能证明"问题在哪"。
    let mai2_load_lanes = Mai2LoadLanes {
        serial: !args.iter().any(|a| a == "--only-light"),
        light: !args.iter().any(|a| a == "--only-serial"),
        vendor: !args.iter().any(|a| a == "--no-vendor"),
        heavy: args.iter().any(|a| a == "--heavy"),
    };
    // 定向修复: 只把 --nv-soak 写坏的极性与状态灯这几项救回来, 不做整体 RESET_DEFAULTS。
    let kbd_repair = args.iter().any(|a| a == "--kbd-repair");
    // --cfg-set <键> <值>: 无头改单个配置项并落盘。类型按设备回读的 schema 真值推导, 不猜。
    let cfg_set: Option<(String, String)> = args
        .iter()
        .position(|a| a == "--cfg-set")
        .and_then(|i| match (args.get(i + 1), args.get(i + 2)) {
            (Some(k), Some(v)) => Some((k.clone(), v.clone())),
            _ => None,
        });
    // --kbd-hidout [按住毫秒]: HID 键盘输出闭环自检(无人值守)。见 run_kbd_hidout 的说明。
    let kbd_hidout = args.iter().any(|a| a == "--kbd-hidout");
    let kbd_hidout_ms: u64 = args
        .iter()
        .position(|a| a == "--kbd-hidout")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(2500);
    let led_test = args.iter().any(|a| a == "--led");
    let set_verify_brightness: Option<u8> = args
        .iter()
        .position(|a| a == "--set-verify-brightness")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<u16>().ok())
        .and_then(|value| u8::try_from(value).ok());
    let set_verify_brightness_requested = args.iter().any(|a| a == "--set-verify-brightness");
    let soak = args.iter().any(|a| a == "--soak");
    let list_only = args.iter().any(|a| a == "--list-only");
    let debug_read = args.iter().any(|a| a == "--debug-read");
    let algo_info_only = args.iter().any(|a| a == "--algo-info-only");
    let ctrl_bootsel = args.iter().any(|a| a == "--ctrl-bootsel");
    let trigger_crash_bootsel = args.iter().any(|a| a == "--trigger-crash-bootsel");
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
                let report_len = usize::from(le16(2)).min(b.len());
                println!("[DBG] magic=0x{:04X} len={}", le16(0), report_len);
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
                if report_len > 49 {
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
                if report_len > 52 {
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
                if report_len >= 67 {
                    println!(
                        "[DBG] nv_dirty_mask={} nv_commit_ok={} nv_commit_fail={} nv_algo_src_len={}",
                        b[54],
                        le32(55),
                        le32(59),
                        le32(63)
                    );
                }
                // 主循环阻塞剖面(仅新固件有): 整轮最长 + 各段最长 + 反堆叠闸门实证。
                if b.len() >= DBG_LEN_WITH_PROFILE {
                    let loop_max = le32(DBG_OFF_LOOP_MAX_US);
                    println!(
                        "[DBG] loop_max_us={} (距 5s 看门狗余量 {}us) heavy_rejects={} heavy_busy={}",
                        loop_max,
                        WATCHDOG_BUDGET_US.saturating_sub(loop_max),
                        le32(DBG_OFF_HEAVY_REJECTS),
                        b[DBG_OFF_HEAVY_BUSY]
                    );
                    for (i, name) in LOOP_SEG_NAMES.iter().enumerate() {
                        println!(
                            "[DBG]   seg {:<9} max={}us",
                            name,
                            le32(DBG_OFF_SEG_MAX_US + i * 4)
                        );
                    }
                }
                if report_len >= DBG_LEN_WITH_NV_VALID {
                    const NV_REGION_NAMES: [&str; 4] = ["KV", "CSD", "ALGO_BIN", "ALGO_SRC"];
                    let m = b[DBG_OFF_NV_VALID_MASK];
                    let list: Vec<String> = NV_REGION_NAMES
                        .iter()
                        .enumerate()
                        .map(|(i, n)| {
                            format!(
                                "{}={}",
                                n,
                                if (m >> i) & 1 == 1 {
                                    "有效"
                                } else {
                                    "无效"
                                }
                            )
                        })
                        .collect();
                    println!("[DBG] nv_valid_mask=0x{:X} {}", m, list.join(" "));
                }
                if report_len >= DBG_LEN_LEGACY_COUNTERS {
                    // 响应编码失败取证: >0 即确证某条命令的响应被 encode_* 拒绝组帧(主机永远收不到它)。
                    println!(
                        "[DBG] last_boot_fault_core={} core1_stage=0x{:02X} resp_encode_fail={} last_fail_cmd=0x{:02X} last_fail_req_len={}",
                        b[DBG_OFF_LAST_FAULT_CORE],
                        b[DBG_OFF_CORE1_STAGE],
                        le32(DBG_OFF_RESP_ENCODE_FAIL),
                        b[DBG_OFF_RESP_FAIL_CMD],
                        u16::from_le_bytes([
                            b[DBG_OFF_RESP_FAIL_REQ_LEN],
                            b[DBG_OFF_RESP_FAIL_REQ_LEN + 1],
                        ])
                    );
                }
                if report_len >= DBG_LEN_WITH_EMIT_COST {
                    println!(
                        "[DBG] 延迟线补偿量 gio_emit_cost_us={}us (应贴近 RP处理+USB写 的典型值; 偏大即过补偿, 设定触控延迟会被削短)",
                        le32(DBG_OFF_GIO_EMIT_COST)
                    );
                }
                // ★门槛必须是"这组字段自己的末端长度"★ 原先写的是 DBG_LEN_COUNTERS, 而它每次
                // 追加新字段都会变大 —— 追加一格就把这组早就存在的字段对旧固件整体判成不可解析。
                if report_len >= DBG_LEN_WITH_BOOT_CAL {
                    let d = b[DBG_OFF_BOOT_CAL_DIAG];
                    let stage = match d & 0x0F {
                        0 => "WAIT_TRUST",
                        1 => "IDAC_START",
                        2 => "IDAC_WAIT",
                        3 => "CHANNEL_START",
                        4 => "CHANNEL_WAIT",
                        5 => "BASELINE_START",
                        6 => "BASELINE_WAIT",
                        7 => "VERIFY_WAIT",
                        8 => "DONE",
                        // ★9 排在 DONE 之后不是笔误★ 固件把新增的统一延迟档追加在枚举末尾,
                        // 以免把既有 0..8 平移一格、改变历史诊断读数的含义(见 boot_calibration.h)。
                        9 => "DELAY_WAIT(等 calib.boot_delay_ms)",
                        other => {
                            println!("[DBG] 开机校准: 未知 stage 编码 {}", other);
                            "?"
                        }
                    };
                    println!(
                        "[DBG] 开机校准: stage={} provisioned={} 输出被抑制={} heavy_busy={} link_alive={}",
                        stage,
                        (d & 0x10) != 0,
                        (d & 0x20) != 0,
                        (d & 0x40) != 0,
                        (d & 0x80) != 0
                    );
                    // 结局位图: stage=DONE 只说明流水线走完, 说不出"到底做了没有"。
                    let m = b[DBG_OFF_BOOT_CAL_FAIL];
                    let mut notes: Vec<&str> = Vec::new();
                    if m & 0x01 != 0 { notes.push("IDAC=跳过(开关关)"); }
                    if m & 0x02 != 0 { notes.push("IDAC=启动失败"); }
                    if m & 0x04 != 0 { notes.push("频率自适应=跳过(开关关)"); }
                    if m & 0x08 != 0 { notes.push("频率自适应=启动失败"); }
                    if m & 0x10 != 0 { notes.push("基线复位=跳过(开关关)"); }
                    if m & 0x20 != 0 { notes.push("基线复位=启动失败"); }
                    if m & 0x40 != 0 { notes.push("末尾验收=失败"); }
                    if m & 0x80 != 0 { notes.push("至少一档已真正执行"); }
                    // mask 只有 RAN_SOMETHING 一位 = 三档都下达成功、没有任何一档被跳过或失败,
                    // 且末尾验收通过。这是"开机校准完全成功"的判据, 与"什么都没做"必须区分开。
                    println!(
                        "[DBG] 开机校准结局: mask=0x{:02X} {}",
                        m,
                        match m {
                            0x80 => "三档全部真正执行、验收通过".to_string(),
                            0 => "流水线未推进(尚未开始或全部开关关闭)".to_string(),
                            _ => notes.join(" | "),
                        }
                    );
                }
                if report_len >= DBG_LEN_WITH_HOST_DISPATCH {
                    println!(
                        "[DBG] dispatch_count={} algo_dispatch_count={} last_cmd=0x{:02X} last_seq={} last_resp_len={}",
                        le32(DBG_OFF_HOST_DISPATCH_COUNT),
                        le32(DBG_OFF_HOST_ALGO_INFO_DISPATCH_COUNT),
                        b[DBG_OFF_HOST_LAST_DISPATCH_CMD],
                        b[DBG_OFF_HOST_LAST_DISPATCH_SEQ],
                        u16::from_le_bytes([
                            b[DBG_OFF_HOST_LAST_DISPATCH_RESP_LEN],
                            b[DBG_OFF_HOST_LAST_DISPATCH_RESP_LEN + 1],
                        ])
                    );
                }
                if report_len >= DBG_LEN_WITH_GPIO {
                    let pc = le32(DBG_OFF_GPIO_PC + 4);
                    let hsiom = le32(DBG_OFF_HSIOM_PORT_SEL + 4);
                    let dm = (pc >> (7 * 3)) & 0x7;
                    let hsiom_sel = (hsiom >> (7 * 4)) & 0xF;
                    println!(
                        "[DBG] GPIO SWD: read_ok={} status={} P1.PC=0x{:08X} P1.HSIOM=0x{:08X} P1.7 DM={} HSIOM={}",
                        b[DBG_OFF_GPIO_READ_OK],
                        le32(DBG_OFF_GPIO_SWD_STATUS),
                        pc,
                        hsiom,
                        dm,
                        hsiom_sel
                    );
                } else {
                    println!(
                        "[DBG] GPIO SWD tail unavailable (report_len={})",
                        report_len
                    );
                }
                if report_len >= DBG_LEN_WITH_SEG2 {
                    for (i, name) in GAMEIO_SEG_NAMES.iter().enumerate() {
                        println!(
                            "[DBG]   gio {:<11} max={}us",
                            name,
                            le32(DBG_OFF_SEG2_MAX_US + i * 4)
                        );
                    }
                }
                print_post_mortem("[DBG]", &b);
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

    let connect_started = std::time::Instant::now();
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

    if trigger_crash_bootsel {
        println!("[DBG] 请求已由 UI 自动武装的 DEBUG_TRIGGER_CRASH...");
        if let Err(error) = ctrl.debug_trigger_crash() {
            println!("[DBG] trigger-crash-bootsel 发送失败: {}", error);
            std::process::exit(1);
        }
        let deadline = std::time::Instant::now() + Duration::from_secs(3);
        while std::time::Instant::now() < deadline && !ctrl.debug_trigger_crash_acknowledged() {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let acknowledged = ctrl.debug_trigger_crash_acknowledged();
        println!(
            "[DBG] DEBUG_TRIGGER_CRASH ACK={}{}",
            acknowledged,
            if acknowledged {
                "，设备将进入 BOOTSEL"
            } else {
                "，未确认 ACK"
            }
        );
        std::process::exit(if acknowledged { 0 } else { 1 });
    }

    if acceptance {
        let pass = _run_acceptance(&mut ctrl, connect_started);
        std::process::exit(if pass { 0 } else { 1 });
    }

    if review_closure {
        match _run_review_closure(&mut ctrl) {
            Ok(()) => {
                println!("[REVIEW] PASS");
                std::process::exit(0);
            }
            Err(error) => {
                println!("[REVIEW] FAIL {}", error);
                std::process::exit(1);
            }
        }
    }

    if algo_info_only {
        ctrl.cancel_conn_probes_for_diagnostic();
        let before = ctrl.algo_version();
        let sent = ctrl.algo_get_info().is_ok();
        println!(
            "[ALGO-INFO] ALGO_GET_INFO 已发送={}，进入 7 秒只读观测",
            sent
        );
        let started = std::time::Instant::now();
        while started.elapsed() < Duration::from_secs(7) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let received = ctrl.algo_version() > before;
        match ctrl.algo_info() {
            Some(info) => println!(
                "[ALGO-INFO] {} is_default={} psoc_valid={} len={} crc16=0x{:04X}",
                if received { "收到" } else { "未收到" },
                info.is_default,
                info.psoc_valid,
                info.len,
                info.crc16
            ),
            None => println!(
                "[ALGO-INFO] {} algo_info=<未取到>",
                if received { "收到" } else { "未收到" }
            ),
        }
        match ctrl.read_debug_counters() {
            Ok(bytes) if bytes.len() >= DBG_LEN_COUNTERS => {
                let le16 = |offset: usize| u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
                let le32 = |offset: usize| {
                    u32::from_le_bytes([
                        bytes[offset],
                        bytes[offset + 1],
                        bytes[offset + 2],
                        bytes[offset + 3],
                    ])
                };
                println!(
                    "[ALGO-INFO] debug dispatch_count={} algo_dispatch_count={} last_cmd=0x{:02X} last_seq={} last_resp_len={}",
                    le32(DBG_OFF_HOST_DISPATCH_COUNT),
                    le32(DBG_OFF_HOST_ALGO_INFO_DISPATCH_COUNT),
                    bytes[DBG_OFF_HOST_LAST_DISPATCH_CMD],
                    bytes[DBG_OFF_HOST_LAST_DISPATCH_SEQ],
                    le16(DBG_OFF_HOST_LAST_DISPATCH_RESP_LEN)
                );
            }
            Ok(bytes) => println!(
                "[ALGO-INFO] debug counters short len={} (need {} for new dispatch fields)",
                bytes.len(),
                DBG_LEN_COUNTERS
            ),
            Err(error) => println!("[ALGO-INFO] debug-read failed: {}", error),
        }
        // ★schema 解析结果必须能无头看到★ 界面上"未声明 ALGO_SETTING"这句话有两个完全不同的
        // 来源: 设备源码里真的没声明, 或者声明解析出来了但没透传到那个页面。只看界面分不开,
        // 这里直接打设备回读源的解析计数与每一项, 作为界面结论的对照真值。
        // 本开关前面 cancel_conn_probes_for_diagnostic() 把连接探针(含 ALGO_GET_SRC)取消了,
        // 所以必须自己再要一次源, 否则读到的恒是空串 —— 那只说明没请求, 不说明设备没源。
        if let Err(error) = ctrl.request_algo_src() {
            println!("[ALGO-INFO] ALGO_GET_SRC 请求失败: {}", error);
        }
        let started = std::time::Instant::now();
        while started.elapsed() < Duration::from_secs(6) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        // 设备侧无存源(出厂默认算法只有内嵌 blob, 映射表里没有 C 源)时, GUI 的做法是把内嵌默认源
        // 去注释后回灌一次, 使此后"读取信息"能真从设备取回。这里走**同一条**路径复现它, 否则
        // 无头侧永远只能读到空串, 也就无法证明整条 schema 链是通的。
        if ctrl.algo_device_src().trim().is_empty()
            && ctrl.algo_info().map(|i| i.is_default).unwrap_or(false)
        {
            let default_src =
                AppController::strip_c_comments(mai2control_ui::algo_template::ALGO_V31_TEMPLATE);
            println!(
                "[ALGO-INFO] 设备映射表无 C 源(默认算法) → 按 GUI 同一路径回灌内嵌默认源 {} 字节",
                default_src.len()
            );
            if let Err(error) = ctrl.send_algo_src(&default_src) {
                println!("[ALGO-INFO] ALGO_SET_SRC 回灌失败: {}", error);
            }
            let started = std::time::Instant::now();
            while started.elapsed() < Duration::from_secs(8) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(20));
            }
            let _ = ctrl.request_algo_src();
            let started = std::time::Instant::now();
            while started.elapsed() < Duration::from_secs(6) {
                ctrl.poll();
                thread::sleep(Duration::from_millis(20));
            }
        }
        let src_len = ctrl.algo_device_src().len();
        let settings = ctrl.algo_setting_decls();
        let reports = ctrl.algo_report_decls();
        println!(
            "[ALGO-INFO] 设备源回读 {} 字节 → 解析 ALGO_REPORT×{} / ALGO_SETTING×{}",
            src_len,
            reports.len(),
            settings.len()
        );
        for d in &settings {
            println!(
                "[ALGO-INFO]   cfg[{}] {} type={} range={} default={} 设备当前值={}",
                d.idx,
                d.name,
                d.value_type,
                d.range,
                d.default,
                ctrl.algo_cfg(d.idx)
            );
        }
        std::process::exit(if sent && received { 0 } else { 1 });
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

    if set_verify_brightness_requested {
        let Some(brightness) = set_verify_brightness else {
            println!("[LED-BRIGHTNESS] FAIL 参数必须是 0..255 的整数");
            std::process::exit(1);
        };
        let key = "led.ws_brightness";
        let expected = brightness as u32;
        let config_deadline = std::time::Instant::now() + Duration::from_secs(5);
        while std::time::Instant::now() < config_deadline && ctrl.config_get(key).is_none() {
            let _ = ctrl.request_config_all();
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        if ctrl.config_get(key).is_none() {
            println!("[LED-BRIGHTNESS] FAIL CFG_GET_ALL 超时: key={}", key);
            std::process::exit(1);
        }
        if let Err(error) = ctrl.set_config_number(key, brightness as f64) {
            println!("[LED-BRIGHTNESS] FAIL 设置草稿: {}", error);
            std::process::exit(1);
        }
        if let Err(error) = ctrl.save_config() {
            println!("[LED-BRIGHTNESS] FAIL save_config: {}", error);
            std::process::exit(1);
        }
        let save_deadline = std::time::Instant::now() + Duration::from_secs(30);
        while std::time::Instant::now() < save_deadline && ctrl.cfg_tx_pending() != 0 {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        if ctrl.cfg_tx_pending() != 0 {
            println!(
                "[LED-BRIGHTNESS] FAIL 保存超时: cfg_tx_pending={}",
                ctrl.cfg_tx_pending()
            );
            std::process::exit(1);
        }
        let before_led = ctrl.led_version();
        if let Err(error) = ctrl.led_request_state() {
            println!("[LED-BRIGHTNESS] FAIL LED_GET 请求: {}", error);
            std::process::exit(1);
        }
        let led_deadline = std::time::Instant::now() + Duration::from_secs(5);
        while std::time::Instant::now() < led_deadline && ctrl.led_version() <= before_led {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let config_value = ctrl.config_get(key).and_then(|entry| match entry.value {
            CfgValue::U8(value) => Some(value as u32),
            CfgValue::U16(value) => Some(value as u32),
            CfgValue::U32(value) => Some(value),
            _ => None,
        });
        let applied = ctrl.led_applied_brightness();
        let pass = config_value == Some(expected) && applied == Some(brightness);
        println!(
            "[LED-BRIGHTNESS] expected_config={} device_config={:?} applied_brightness={:?} {}",
            expected,
            config_value,
            applied,
            if pass { "PASS" } else { "FAIL" }
        );
        if !pass {
            println!("[LED-BRIGHTNESS] 软件回读未完成一致性对账；物理亮度/光强未由本验收仪表化");
        } else {
            println!("[LED-BRIGHTNESS] 软件证据通过；物理亮度/光强仍需人工或仪器验证");
        }
        std::process::exit(if pass { 0 } else { 1 });
    }

    if bus_test {
        match run_bus_test(&mut ctrl) {
            Ok(()) => {
                println!("[BUS] ALL PASS");
                std::process::exit(0);
            }
            Err(reason) => {
                println!("[BUS] FAIL: {}", reason);
                std::process::exit(1);
            }
        }
    }

    if kbd_repair {
        run_kbd_repair(&mut ctrl);
    }

    // 压测前先把配置取到手: 波特率要用设备真值而不是默认值。
    if mai2_load {
        let _ = ctrl.request_config_all();
        let deadline = std::time::Instant::now() + Duration::from_millis(1200);
        while std::time::Instant::now() < deadline && ctrl.config_entries().is_empty() {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        run_mai2_load(&mut ctrl, &port_name, mai2_load_secs, mai2_load_lanes);
    }

    if led_test {
        let passed = run_led_test(&mut ctrl);
        println!("[LED] {}", if passed { "PASS" } else { "FAIL" });
        std::process::exit(if passed { 0 } else { 1 });
    }

    // 无头真实 SweepSession 生命周期：设备负责 448 格、恢复与丢包补发；这里仅轮询并验收终态。
    // 复用生产 AppController，不另写测试协议栈，保证与 UI 实际路径完全一致。
    // --focus-band [ch] [秒]: 实测单通道独占流的端到端交付率。
    // ★为什么按"设备代数"判定而不是只看帧率★ 固件只在快照代数推进时才发 FOCUS_DATA, 所以
    // 帧/s 的上限就是设备扫描速率; 只报帧率会把"设备只扫这么快"误读成"链路带宽不够"。
    // 这里同时取 samples_per_sec(设备自报扫描速率), 用交付率/扫描率的比值判断链路是否漏帧。
    // --lat-probe [秒]: 排查"总延迟每隔数秒一簇尖峰"。见 run_lat_probe 的判据链说明。
    if args.iter().any(|a| a == "--lat-probe") {
        let secs = args
            .iter()
            .position(|a| a == "--lat-probe")
            .and_then(|i| args.get(i + 1))
            .and_then(|v| v.parse::<u64>().ok())
            .unwrap_or(60);
        let per_window_clear = !args.iter().any(|a| a == "--no-prof-clear");
        let read_each_window = !args.iter().any(|a| a == "--prof-endonly");
        std::process::exit(run_lat_probe(
            &mut ctrl,
            secs,
            per_window_clear,
            read_each_window,
        ));
    }

    if args.iter().any(|a| a == "--focus-band") {
        let position = args.iter().position(|a| a == "--focus-band");
        let channel = position
            .and_then(|i| args.get(i + 1))
            .and_then(|v| v.parse::<u8>().ok())
            .unwrap_or(0);
        let secs = position
            .and_then(|i| args.get(i + 2))
            .and_then(|v| v.parse::<u64>().ok())
            .unwrap_or(10);
        println!("[FOCUS] 启动 CH{} 单通道独占流, 实测 {}s...", channel, secs);
        ctrl.focus_set_target(Some(channel));
        let started = std::time::Instant::now();
        let mut last_report = std::time::Instant::now();
        let mut best_frames = 0u32;
        let mut worst_frames = u32::MAX;
        let mut last_renew = std::time::Instant::now();
        while started.elapsed() < Duration::from_secs(secs) {
            ctrl.poll();
            ctrl.csd_diag_tick();
            // ★续租★ 独占流租约 3s, 靠"任意主机帧"续期。GUI 天然每帧都有命令流量, 而本无头
            // 脚手架只读不写 ⇒ 不补一条命令的话设备会在 ~3s 后自动停流(实测第 7s 起读数归零),
            // 那是脚手架的缺陷, 不是设备带宽问题。每 1s 发一条最轻的读命令即可。
            if last_renew.elapsed() >= Duration::from_secs(1) {
                let _ = ctrl.request_param(channel, 0x0B);
                last_renew = std::time::Instant::now();
            }
            if last_report.elapsed() >= Duration::from_secs(1) {
                if let Some((frames, _bytes, _gaps)) = ctrl.focus_rate_last() {
                    if frames > best_frames {
                        best_frames = frames;
                    }
                    // 首个窗口常是半窗(会话刚建立), 不计入最差值。
                    if started.elapsed() >= Duration::from_secs(2) && frames < worst_frames {
                        worst_frames = frames;
                    }
                }
                println!(
                    "[FOCUS] {} | 设备扫描 {}/s",
                    ctrl.focus_bandwidth_text(),
                    ctrl.telem_samples_per_sec()
                );
                last_report = std::time::Instant::now();
            }
            thread::sleep(Duration::from_millis(2));
        }
        let scan_rate = ctrl.telem_samples_per_sec();
        let gaps = ctrl.focus_gaps_total();
        println!(
            "[FOCUS] END 峰值={}帧/s 最差={}帧/s 设备扫描={}/s 累计丢帧={}",
            best_frames,
            if worst_frames == u32::MAX {
                0
            } else {
                worst_frames
            },
            scan_rate,
            gaps
        );
        // 过测判据分两层: ① 链路必须把设备扫出来的每一轮都送到(交付率 >= 扫描率的 95%,
        // 留 5% 给窗口错位); ② 设备扫描率本身是否达到 300QPS 目标 —— 两者分别如实报告,
        // 不把"设备只扫 170" 混report成"链路不够"。
        let delivered_ok = scan_rate == 0 || best_frames as f32 >= scan_rate as f32 * 0.95;
        let target_ok = best_frames >= 300;
        println!(
            "[FOCUS] 链路交付={} (峰值/扫描率={:.0}%) 300QPS目标={}",
            if delivered_ok { "PASS" } else { "FAIL" },
            if scan_rate > 0 {
                best_frames as f32 / scan_rate as f32 * 100.0
            } else {
                0.0
            },
            if target_ok { "PASS" } else { "FAIL" }
        );
        ctrl.focus_set_target(None);
        for _ in 0..20 {
            ctrl.poll();
            thread::sleep(Duration::from_millis(10));
        }
        std::process::exit(if target_ok { 0 } else { 1 });
    }

    if args.iter().any(|a| a == "--sweep-session") {
        let channel = args
            .iter()
            .position(|a| a == "--sweep-session")
            .and_then(|i| args.get(i + 1))
            .and_then(|v| v.parse::<u8>().ok())
            .unwrap_or(0);
        println!("[SWEEP] 启动 CH{} 设备侧 448 格会话...", channel);
        if let Err(error) = ctrl.noise_sweep_start(channel) {
            println!("[SWEEP] FAIL start: {}", error);
            std::process::exit(1);
        }
        let started = std::time::Instant::now();
        let mut last_report = std::time::Instant::now();
        while ctrl.noise_sweep_active() && started.elapsed() < Duration::from_secs(1200) {
            ctrl.poll();
            // UI 每 16ms 调用同一生产 tick；无头模式也必须推进它，否则 Sweep keepalive 永远不会下发。
            ctrl.csd_diag_tick();
            if last_report.elapsed() >= Duration::from_secs(5) {
                println!(
                    "[SWEEP] {:5.1}% {}",
                    ctrl.noise_sweep_progress() * 100.0,
                    ctrl.noise_sweep_status()
                );
                last_report = std::time::Instant::now();
            }
            thread::sleep(Duration::from_millis(10));
        }
        // 再泵一拍，收终态后紧随的恢复/补发帧。
        for _ in 0..20 {
            ctrl.poll();
            thread::sleep(Duration::from_millis(10));
        }
        let cells = ctrl.noise_sweep_cells();
        let valid = cells.iter().filter(|cell| cell.valid).count();
        let flag_count = |flag| cells.iter().filter(|cell| cell.flags & flag != 0).count();
        let progress = ctrl.noise_sweep_progress();
        let produced = ctrl.noise_sweep_produced();
        let stalled = flag_count(mai2control_ui::proto::SWEEP_FLAG_STALLED);
        println!(
            "[SWEEP] END produced={} progress={:.1}% valid={}/448 status={}",
            produced,
            progress * 100.0,
            valid,
            ctrl.noise_sweep_status()
        );
        println!(
            "[SWEEP] FLAGS CAL_FAIL={} MISMATCH={} STALLED={} RAILED={}",
            flag_count(mai2control_ui::proto::SWEEP_FLAG_CAL_FAIL),
            flag_count(mai2control_ui::proto::SWEEP_FLAG_MISMATCH),
            flag_count(mai2control_ui::proto::SWEEP_FLAG_STALLED),
            flag_count(mai2control_ui::proto::SWEEP_FLAG_RAILED),
        );
        for (index, cell) in cells
            .iter()
            .enumerate()
            .filter(|(_, cell)| !cell.valid)
            .take(12)
        {
            println!(
                "[SWEEP] INVALID index={} gain={} div={} samples={} flags=0x{:02X} fail_phase={:?}",
                index, cell.gain, cell.div, cell.samples, cell.flags, cell.fail_phase
            );
        }
        let passed = !ctrl.noise_sweep_active() && produced == 448 && valid == 448 && stalled == 0;
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
        // ★逐通道档下"实测探测周期"必须仍有值★
        // 36 通道 × 4 字段远超 64B vendor FIFO, 固件按帧切分且**只有首帧带 STATS**。
        // 上位机若无条件用每帧的 stats 覆盖, 后续无 STATS 的帧会立刻把真值冲成 0 ——
        // 症状是"全局调整页始终未测"而主页(轻档、每帧带 STATS)看着正常。这里把它钉住。
        println!(
            "[SELFTEST] TELEM stats: scan_period_us={} samples_per_sec={} valid={}",
            ctrl.telem_scan_period_us(),
            ctrl.telem_samples_per_sec(),
            ctrl.telem_scan_period_valid()
        );
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

    // IDAC 增益围栏 + inactive 下禁用电极的真实 GPIO High-Z 诊断。
    if args.iter().any(|a| a == "--gain-inactive-test") {
        const G_INACTIVE: u8 = 0x01;
        const G_IDAC_GAIN: u8 = 0x02;
        const PARAM_IDAC_GAIN: u8 = 0x0B;
        const PARAM_ENABLED: u8 = 0x0C;
        const TARGET_CH: u8 = 35;
        let settle = |ctrl: &mut AppController, ms: u64| {
            let deadline = std::time::Instant::now() + Duration::from_millis(ms);
            while std::time::Instant::now() < deadline {
                ctrl.poll();
                thread::sleep(Duration::from_millis(15));
            }
        };
        let get_g = |ctrl: &mut AppController, id: u8| -> Option<u32> {
            let _ = ctrl.global_get(id);
            settle(ctrl, 500);
            ctrl.global(id)
        };
        let get_param = |ctrl: &mut AppController, ch: u8, id: u8| -> Option<u32> {
            let before = ctrl.param_version();
            if ctrl.request_params(ch).is_err() {
                return None;
            }
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            while std::time::Instant::now() < deadline {
                ctrl.poll();
                if ctrl.param_version() > before {
                    return ctrl.param(ch, id);
                }
                thread::sleep(Duration::from_millis(15));
            }
            None
        };
        let save_and_settle = |ctrl: &mut AppController, stage: &str| -> Result<(), String> {
            ctrl.save_config()
                .map_err(|error| format!("{}: save_config: {}", stage, error))?;
            let deadline = std::time::Instant::now() + Duration::from_secs(35);
            while std::time::Instant::now() < deadline {
                ctrl.poll();
                if ctrl.cfg_tx_pending() == 0 && ctrl.config_dirty_count() == 0 {
                    return Ok(());
                }
                thread::sleep(Duration::from_millis(20));
            }
            Err(format!(
                "{}: 保存超时(pending={} dirty={})",
                stage,
                ctrl.cfg_tx_pending(),
                ctrl.config_dirty_count()
            ))
        };
        let verify_ch35_gpio = |ctrl: &AppController,
                                mode: &str,
                                stage: &str|
         -> Result<(), String> {
            let bytes = ctrl
                .read_debug_counters()
                .map_err(|error| format!("mode={} stage={}: DEBUG_READ: {}", mode, stage, error))?;
            if bytes.len() < 4 {
                return Err(format!(
                    "mode={} stage={}: DEBUG_READ 太短({})",
                    mode,
                    stage,
                    bytes.len()
                ));
            }
            let le16 = |offset: usize| u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
            let le32 = |offset: usize| {
                u32::from_le_bytes([
                    bytes[offset],
                    bytes[offset + 1],
                    bytes[offset + 2],
                    bytes[offset + 3],
                ])
            };
            let report_len = usize::from(le16(2)).min(bytes.len());
            if report_len < DBG_LEN_WITH_GPIO {
                return Err(format!(
                    "mode={} stage={}: GPIO 尾部缺失(report_len={}, need={})",
                    mode, stage, report_len, DBG_LEN_WITH_GPIO
                ));
            }
            let pc = le32(DBG_OFF_GPIO_PC + 4);
            let hsiom = le32(DBG_OFF_HSIOM_PORT_SEL + 4);
            let dm = (pc >> (7 * 3)) & 0x7;
            let hsiom_sel = (hsiom >> (7 * 4)) & 0xF;
            let read_ok = bytes[DBG_OFF_GPIO_READ_OK];
            let swd_status = le32(DBG_OFF_GPIO_SWD_STATUS);
            println!(
                "[GI] mode={} stage={} raw PC(P1)=0x{:08X} HSIOM(P1)=0x{:08X} DM(P1.7)={} HSIOM(P1.7)={} read_ok={} swd_status={}",
                mode, stage, pc, hsiom, dm, hsiom_sel, read_ok, swd_status
            );
            if read_ok != 1 {
                return Err(format!(
                    "mode={} stage={}: GPIO SWD read_ok={} status={}",
                    mode, stage, read_ok, swd_status
                ));
            }
            if dm != 1 || hsiom_sel != 0 {
                return Err(format!(
                    "mode={} stage={}: P1.7 expected DM=1 HSIOM=0, got DM={} HSIOM={}",
                    mode, stage, dm, hsiom_sel
                ));
            }
            Ok(())
        };
        let run_cp_bist = |ctrl: &mut AppController, mode: &str| -> Result<(), String> {
            ctrl.measure_cp()
                .map_err(|error| format!("mode={}: Cp BIST submit: {}", mode, error))?;
            settle(ctrl, 4_000);
            Ok(())
        };

        let original_inactive = get_g(&mut ctrl, G_INACTIVE).unwrap_or(1);
        let original_enabled = get_param(&mut ctrl, TARGET_CH, PARAM_ENABLED).unwrap_or(1) != 0;
        let mut failures: Vec<String> = Vec::new();
        println!(
            "[GI] 基线: IDAC_GAIN_INIT(0x02)={:?} INACTIVE_SNS(0x01)={} CH35 enabled={}",
            get_g(&mut ctrl, G_IDAC_GAIN),
            original_inactive,
            original_enabled
        );

        match ctrl.debug_global_now(G_IDAC_GAIN, 7) {
            Err(error) => println!("[GI] 增益=7 被主机围栏拒绝: {}", error),
            Ok(()) => settle(&mut ctrl, 300),
        }
        let g7 = get_g(&mut ctrl, G_IDAC_GAIN);
        println!("[GI] 增益=7 回读={:?}", g7);
        if g7 == Some(7) {
            failures.push("IDAC_GAIN_INIT 非法值 7 被接受".to_string());
        }
        if let Err(error) = ctrl.debug_global_now(G_IDAC_GAIN, 6) {
            failures.push(format!("IDAC_GAIN_INIT=6 下发失败: {}", error));
        }
        settle(&mut ctrl, 300);
        let g6 = get_g(&mut ctrl, G_IDAC_GAIN);
        println!("[GI] 增益=6 回读={:?}", g6);
        if g6 != Some(6) {
            failures.push(format!("IDAC_GAIN_INIT 合法值 6 未生效: {:?}", g6));
        }
        match ctrl.debug_param_now(0, PARAM_IDAC_GAIN, 7) {
            Err(error) => println!("[GI] CH0 增益=7 被主机围栏拒绝: {}", error),
            Ok(()) => settle(&mut ctrl, 300),
        }
        let p7 = get_param(&mut ctrl, 0, PARAM_IDAC_GAIN);
        println!("[GI] CH0 param 增益=7 回读={:?}", p7);
        if p7 == Some(7) {
            failures.push("CH0 IDAC_GAIN 非法值 7 被接受".to_string());
        }

        if let Err(error) = ctrl.set_ch_enabled(TARGET_CH, false) {
            failures.push(format!("禁用 CH35 草稿失败: {}", error));
        }
        if let Err(error) = save_and_settle(&mut ctrl, "禁用 CH35") {
            failures.push(error);
        }
        let disabled = get_param(&mut ctrl, TARGET_CH, PARAM_ENABLED);
        println!("[GI] CH35 disabled readback={:?}", disabled);
        if disabled != Some(0) {
            failures.push(format!("禁用 CH35 回读期望 0, 实得 {:?}", disabled));
        }

        for (mode, inactive) in [("GND", 1u32), ("Shield", 4u32)] {
            if let Err(error) = ctrl.debug_global_now(G_INACTIVE, inactive) {
                failures.push(format!("mode={}: INACTIVE_SNS 下发失败: {}", mode, error));
                continue;
            }
            if let Err(error) = ctrl.global_commit() {
                failures.push(format!("mode={}: GLOBAL_COMMIT 下发失败: {}", mode, error));
                continue;
            }
            settle(&mut ctrl, 3_000);
            let actual = get_g(&mut ctrl, G_INACTIVE);
            println!("[GI] mode={} INACTIVE_SNS readback={:?}", mode, actual);
            if actual != Some(inactive) {
                failures.push(format!(
                    "mode={}: INACTIVE_SNS 期望 {}, 实得 {:?}",
                    mode, inactive, actual
                ));
            }
            if let Err(error) = verify_ch35_gpio(&ctrl, mode, "after-apply") {
                failures.push(error);
            }
            if let Err(error) = run_cp_bist(&mut ctrl, mode) {
                failures.push(error);
            }
            if let Err(error) = verify_ch35_gpio(&ctrl, mode, "after-cp-bist") {
                failures.push(error);
            }
        }

        if let Err(error) = ctrl.debug_global_now(G_INACTIVE, original_inactive) {
            failures.push(format!("恢复 INACTIVE_SNS 下发失败: {}", error));
        }
        if let Err(error) = ctrl.global_commit() {
            failures.push(format!("恢复 INACTIVE_SNS GLOBAL_COMMIT 失败: {}", error));
        }
        settle(&mut ctrl, 3_000);
        let inactive_after = get_g(&mut ctrl, G_INACTIVE);
        if inactive_after != Some(original_inactive) {
            failures.push(format!(
                "恢复 INACTIVE_SNS 期望 {}, 实得 {:?}",
                original_inactive, inactive_after
            ));
        }
        if let Err(error) = ctrl.set_ch_enabled(TARGET_CH, original_enabled) {
            failures.push(format!("恢复 CH35 启用态草稿失败: {}", error));
        }
        if let Err(error) = save_and_settle(&mut ctrl, "恢复 CH35 启用态") {
            failures.push(error);
        }
        let enabled_after = get_param(&mut ctrl, TARGET_CH, PARAM_ENABLED);
        let expected_enabled = if original_enabled { 1 } else { 0 };
        println!(
            "[GI] restore INACTIVE_SNS={:?} CH35 enabled={:?}",
            inactive_after, enabled_after
        );
        if enabled_after != Some(expected_enabled) {
            failures.push(format!(
                "恢复 CH35 启用态期望 {}, 实得 {:?}",
                expected_enabled, enabled_after
            ));
        }

        let _ = ctrl.debug_global_now(G_IDAC_GAIN, 4);
        for ch in 0..36u8 {
            let _ = ctrl.debug_param_now(ch, PARAM_IDAC_GAIN, 4);
        }
        settle(&mut ctrl, 300);
        if failures.is_empty() {
            println!("[GI] PASS gain fence + CH35 P1.7 inactive GPIO evidence");
            std::process::exit(0);
        }
        for failure in failures {
            println!("[GI] FAIL {}", failure);
        }
        std::process::exit(1);
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
            println!(
                "[BASE] 2) 把出厂默认阈值 + SNS_CLK_DIV={} 写进全 36 通道...",
                d
            );
        } else {
            println!(
                "[BASE] 2) 把出厂默认阈值写进全 36 通道(分频保持设备现值; 需要纠正时加 --div 32)..."
            );
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
    // --algo-default: 单独把触控算法恢复为内嵌默认并落盘。
    // ★为什么需要它★: --algo 是"上传测试算法 → 校验 → 恢复默认"的闭环, 而上传是**先落盘再校验**;
    // 校验失败时测试提前退出, 那份未通过校验的测试算法就留在 flash 里, 每次开机都被下发给 PSoC,
    // 可能把 PSoC 打死(实测: scan=0 ms=0 link_valid=false)。此时需要一个不依赖 PSoC 存活的
    // 恢复入口 —— 本命令只改 RP2040 侧的算法 store, 不需要 PSoC 配合。
    if args.iter().any(|a| a == "--algo-default") {
        println!("[ALGODEF] 恢复内嵌默认算法并落盘...");
        if let Err(e) = ctrl.algo_reset_default() {
            println!("[ALGODEF] FAIL 发送失败: {}", e);
            std::process::exit(1);
        }
        let start = std::time::Instant::now();
        while start.elapsed() < Duration::from_millis(2500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        let _ = ctrl.algo_get_info();
        let start = std::time::Instant::now();
        while start.elapsed() < Duration::from_millis(1500) {
            ctrl.poll();
            thread::sleep(Duration::from_millis(20));
        }
        println!(
            "[ALGODEF] 现在: is_default/psoc_valid/len = {:?}",
            ctrl.algo_info()
                .map(|i| (i.is_default, i.psoc_valid, i.len))
        );
        println!("[ALGODEF] DONE (请重启设备使 PSoC 重新加载默认算法)");
        std::process::exit(0);
    }

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
                identity_ok,
                runtime_ok,
                diag.last_stage,
                diag.failure_stage,
                diag.flags,
                min_rp_stamp,
                min_psoc_stamp,
                info.fw_version,
                diag.embedded_psoc_version
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
        // ★上传必须在"遥测流式进行中"做★ 这正是 UI 的真实场景, 也是本项修复的回归判据:
        // 流式期间 core1 要分页读快照, SPI 流水线里随时横着别的响应, 于是瞬时 link_ok 频繁为 false。
        // 旧实现拿瞬时值当上传门禁 ⇒ 开着遥测必回 NAK("PSoC 链路暂时不可用"), 而不开流时一次就过 ——
        // 所以不开流的测试根本抓不到这个 bug。现在固件改看去抖后的 link_alive(), 此处必须开流复现。
        {
            use mai2control_ui::proto::{FIELD_BASELINE, FIELD_DIFF, FIELD_RAW};
            let _ = ctrl.start_telemetry(30, FIELD_RAW | FIELD_BASELINE | FIELD_DIFF, u64::MAX);
        }
        pump(&mut ctrl, 40);
        println!(
            "[SELFTEST] ALGO: 遥测流式中 samples_per_sec={} (上传将在流式期间进行)",
            ctrl.telem_samples_per_sec()
        );
        if ctrl.telem_samples_per_sec() == 0 {
            println!("[SELFTEST] FAIL 遥测未真正流起来, 无法复现 UI 场景");
            std::process::exit(1);
        }
        let src = "#include <stddef.h>\n#include \"psoc_algo_abi.h\"\nvoid algo(algo_io_t* io){ io->out_active = (io->base_active!=0u)?1u:0u; }\n";
        println!("[SELFTEST] ALGO: 编译并上传测试算法(base_active 透传)...");
        if let Err(e) = ctrl.compile_and_upload(src) {
            println!("[SELFTEST] FAIL 编译/上传: {}", e);
            std::process::exit(1);
        }
        pump(&mut ctrl, 40);
        // 回执判据独立于"算法是否生效": ACK 迟到(处理器被入队自旋卡住)时设备照样会装上算法,
        // 只看 algo_info 抓不到"发出去没有回执"这个症状。
        let ack = ctrl.algo_upload_status().to_string();
        println!("[SELFTEST] ALGO: 上传回执 = {}", ack);
        if !ack.contains("ACK") {
            println!("[SELFTEST] FAIL 未在超时窗口内收到 ACK(回执缺失或被拒)");
            std::process::exit(1);
        }
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

    if let Some((key, raw)) = cfg_set.clone() {
        run_cfg_set(&mut ctrl, &key, &raw);
    }

    if kbd_hidout {
        run_kbd_hidout(&mut ctrl, kbd_hidout_ms);
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
        // ★组合表与链路诊断必须一起打★ per-zone 表为空时判定全走组合表, 只看前者会得出
        // "一个映射都没有"的错误结论; 而 delay_ms/max_hold_ms 正是"点一下没反应"的常见原因,
        // 不打出来就只能靠猜。
        let _ = ctrl.kbd_request_combo();
        let _ = ctrl.kbd_request_state();
        pump(&mut ctrl, 30);
        let combos = ctrl.kbd_combos();
        println!(
            "[SELFTEST]   组合表 {} 条 (supported={:?})",
            combos.len(),
            ctrl.kbd_combo_supported()
        );
        for (i, c) in combos.iter().enumerate() {
            println!(
                "[SELFTEST]     #{:02} zone_mask=0x{:09X} keys={:02X?} mod={:#04X} delay={}ms max_hold={}ms",
                i, c.zone_mask, c.keycodes, c.modifiers, c.delay_ms, c.max_hold_ms
            );
        }
        match ctrl.kbd_link_diag() {
            Some(d) => println!("[SELFTEST]   链路诊断: {}", d.summary()),
            None => println!("[SELFTEST]   链路诊断: 不可用(设备未回传)"),
        }
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
        // 可选按前缀 dump 键值: `--cfg-only <前缀>`。用于核对某个 KV 的**设备真值**(而不是界面上
        // 的草稿态) —— 例如开机校准三档到底是 true 还是 false, 那决定固件流水线要不要动手。
        if let Some(prefix) = args.iter().skip(1).find(|a| !a.starts_with("--")) {
            let mut hit = 0usize;
            for entry in ctrl.config_entries() {
                if entry.key.starts_with(prefix.as_str()) {
                    println!("[CFG] {} = {:?}", entry.key, entry.value);
                    hit += 1;
                }
            }
            println!("[CFG] 前缀 '{}' 命中 {} 项", prefix, hit);
        }
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

    // 保存完整自测会临时修改的 CH0 参数；后续无论 Cp 验收成败均不能遗留测试值。
    const TH_PARAM: u8 = 0x01; // FINGER_TH
    const CLK_PARAM: u8 = 0x08; // SNS_CLK_DIV
    let th_orig = match ctrl.param(0, TH_PARAM) {
        Some(value) => value,
        None => {
            println!("[SELFTEST] FAIL 未找到 CH0 FINGER_TH 原值");
            std::process::exit(1);
        }
    };
    let clk_orig = match ctrl.param(0, CLK_PARAM) {
        Some(value) => value,
        None => {
            println!("[SELFTEST] FAIL 未找到 CH0 SNS_CLK_DIV 原值");
            std::process::exit(1);
        }
    };

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

    // ── Step 7b-2: 通道启用开关(PARAM_ENABLED=0x0C)真机往返 ────────────────────────
    // ★这一项必须走"保存到设备"才算数★: set_ch_enabled 只写草稿, 而 param() 是草稿优先 ——
    // 不落盘就回读只会读到自己刚写的草稿, 证明不了任何事。故序列是
    //   草稿 → save_config(排队串行下发) → 等 dirty 清零 → GET_ALL 回读设备真值 → 遥测取证。
    // 遥测取证是关键: 禁用后该通道 raw 必须恒 0(电极高阻/不参与扫描), 而相邻参照通道照旧有读数,
    // 这才排除了"只是不上报"的伪关闭; 重新启用后 raw 必须回来(固件只为该通道重做校准+基线)。
    const EN_PARAM: u8 = 0x0C;
    const EN_CH: u8 = 35; // 目标通道(Cp 正常)
    const EN_REF_CH: u8 = 34; // 参照通道: 证明只关掉了目标, 没有连坐
    let save_and_settle = |ctrl: &mut AppController, what: &str| -> bool {
        if let Err(e) = ctrl.save_config() {
            println!("[SELFTEST] FAIL 保存({}) : {}", what, e);
            return false;
        }
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        loop {
            ctrl.poll();
            if ctrl.cfg_tx_pending() == 0 && ctrl.config_dirty_count() == 0 {
                return true;
            }
            if std::time::Instant::now() > deadline {
                println!(
                    "[SELFTEST] FAIL 保存({})超时: 待发 {} 帧, 未清脏 {} 项",
                    what,
                    ctrl.cfg_tx_pending(),
                    ctrl.config_dirty_count()
                );
                return false;
            }
            thread::sleep(Duration::from_millis(20));
        }
    };
    let device_param = |ctrl: &mut AppController, ch: u8, id: u8| -> Option<u32> {
        let base = ctrl.param_version();
        if let Err(e) = ctrl.request_params(ch) {
            println!("[SELFTEST] FAIL request_params(CH{}): {}", ch, e);
            return None;
        }
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            ctrl.poll();
            if ctrl.param_version() > base || std::time::Instant::now() > deadline {
                return ctrl.param(ch, id);
            }
            thread::sleep(Duration::from_millis(20));
        }
    };
    // ★必须等"新帧"再读★: telem_latest 是缓存, 切换前那一帧会一直留在里面 —— 直接读它
    // (或在窗口里取峰值)只会读到禁用之前的旧读数, 从而把"真关闭"误判成"没关掉"。
    // 故先把帧计数推进若干帧, 再取当前值; 推不动就是遥测本身出了问题, 如实报出。
    let pump_frames = |ctrl: &mut AppController, frames: u64, ms: u64| -> bool {
        let base = ctrl.telem_version();
        let deadline = std::time::Instant::now() + Duration::from_millis(ms);
        loop {
            ctrl.poll();
            if ctrl.telem_version() >= base + frames {
                return true;
            }
            if std::time::Instant::now() > deadline {
                return false;
            }
            thread::sleep(Duration::from_millis(5));
        }
    };
    let latest_raw = |ctrl: &AppController, ch: u8| {
        ctrl.telem_latest(ch).and_then(|s| s.raw).unwrap_or(0) as u32
    };
    let restore_ch0_params = |ctrl: &mut AppController, stage: &str| -> bool {
        println!(
            "[SELFTEST] 恢复 CH0 FINGER_TH/SNS_CLK_DIV 原值 ({})...",
            stage
        );
        let mut restored = true;
        for (id, value, name) in [
            (TH_PARAM, th_orig, "FINGER_TH"),
            (CLK_PARAM, clk_orig, "SNS_CLK_DIV"),
        ] {
            if let Err(e) = ctrl.set_param(0, id, value) {
                println!("[SELFTEST] WARN 恢复 CH0 {}={} 失败: {}", name, value, e);
                restored = false;
            }
        }
        if let Err(e) = ctrl.calibrate(u64::MAX) {
            println!("[SELFTEST] WARN 恢复 CH0 参数 APPLY 失败: {}", e);
            restored = false;
        }
        if !save_and_settle(ctrl, "恢复 CH0 FINGER_TH/SNS_CLK_DIV") {
            restored = false;
        }
        let th_readback = device_param(ctrl, 0, TH_PARAM);
        let clk_readback = device_param(ctrl, 0, CLK_PARAM);
        let readback_ok = th_readback == Some(th_orig) && clk_readback == Some(clk_orig);
        if readback_ok {
            println!(
                "[SELFTEST] CH0 临时参数已恢复并回读确认: FINGER_TH={} SNS_CLK_DIV={}",
                th_orig, clk_orig
            );
        } else {
            println!(
                "[SELFTEST] WARN CH0 临时参数恢复回读不匹配: FINGER_TH 期望 {} 实得 {:?}, SNS_CLK_DIV 期望 {} 实得 {:?}",
                th_orig, th_readback, clk_orig, clk_readback
            );
        }
        restored && readback_ok
    };
    let restore_then_auto = |ctrl: &mut AppController, stage: &str| {
        if !restore_ch0_params(ctrl, stage) {
            println!("[SELFTEST] WARN CH0 临时参数恢复未完全确认");
        }
        if let Err(e) = ctrl.set_mode(0) {
            println!("[SELFTEST] WARN 恢复自动模式失败: {}", e);
        }
    };
    println!(
        "[SELFTEST] ENABLED(0x0C) 往返: 目标 CH{} / 参照 CH{}",
        EN_CH, EN_REF_CH
    );
    let en_orig = device_param(&mut ctrl, EN_CH, EN_PARAM);
    println!("[SELFTEST]   初始设备值 CH{} enabled={:?}", EN_CH, en_orig);
    let mut en_fail: Option<String> = None;
    if let Err(e) = ctrl.set_ch_enabled(EN_CH, false) {
        en_fail = Some(format!("set_ch_enabled(false) 失败: {}", e));
    }
    if en_fail.is_none() && !save_and_settle(&mut ctrl, "禁用通道") {
        en_fail = Some("禁用后保存未完成".to_string());
    }
    if en_fail.is_none() {
        match device_param(&mut ctrl, EN_CH, EN_PARAM) {
            Some(0) => println!("[SELFTEST]   禁用回读 OK: 设备侧 enabled=0"),
            other => en_fail = Some(format!("禁用回读期望 0 实得 {:?}", other)),
        }
    }
    if en_fail.is_none() {
        if let Err(e) = ctrl.start_telemetry(200, 0x1F, u64::MAX) {
            en_fail = Some(format!("start_telemetry(禁用后): {}", e));
        } else if !pump_frames(&mut ctrl, 40, 5000) {
            en_fail = Some("禁用后 5s 内收不到 40 帧遥测, 无法取证".to_string());
        } else {
            let off_target = latest_raw(&ctrl, EN_CH);
            let off_ref = latest_raw(&ctrl, EN_REF_CH);
            println!(
                "[SELFTEST]   禁用后新帧 raw: CH{}={} CH{}={}",
                EN_CH, off_target, EN_REF_CH, off_ref
            );
            if off_target != 0 {
                en_fail = Some(format!(
                    "禁用后 CH{} 仍在出数(raw 峰值 {}) — 不是真关闭",
                    EN_CH, off_target
                ));
            } else if off_ref == 0 {
                en_fail = Some(format!(
                    "参照 CH{} 也停了(raw 峰值 0) — 禁用连坐到了其它通道",
                    EN_REF_CH
                ));
            }
        }
    }
    // 无论成败都要把通道恢复回启用(禁用态会落 flash, 留给用户就是"一个通道莫名不工作")。
    let restore_on = en_orig.unwrap_or(1) != 0;
    if let Err(e) = ctrl.set_ch_enabled(EN_CH, restore_on) {
        println!("[SELFTEST] WARN 恢复 CH{} 启用态失败: {}", EN_CH, e);
    }
    let restored = save_and_settle(&mut ctrl, "恢复通道启用态");
    if en_fail.is_none() && restore_on {
        match device_param(&mut ctrl, EN_CH, EN_PARAM) {
            Some(1) => println!("[SELFTEST]   重新启用回读 OK: 设备侧 enabled=1"),
            other => en_fail = Some(format!("重新启用回读期望 1 实得 {:?}", other)),
        }
        // 重新启用要给固件留出"该通道重校准 + 重建基线"的时间, 故多推几帧再取。
        if !pump_frames(&mut ctrl, 80, 8000) {
            println!("[SELFTEST] WARN 重新启用后遥测帧推进不足, 读数可能偏旧");
        }
        let on_target = latest_raw(&ctrl, EN_CH);
        println!("[SELFTEST]   重新启用后新帧 raw: CH{}={}", EN_CH, on_target);
        if en_fail.is_none() && on_target == 0 {
            en_fail = Some(format!(
                "重新启用后 CH{} 仍无读数(raw 峰值 0) — 恢复扫描/校准未生效",
                EN_CH
            ));
        }
    }
    let _ = ctrl.stop_telemetry();
    if !restored {
        println!(
            "[SELFTEST] WARN CH{} 启用态恢复保存未确认, 请复查设备",
            EN_CH
        );
    }
    match en_fail {
        None => println!("[SELFTEST] ENABLED round-trip PASS"),
        Some(reason) => {
            println!("[SELFTEST] FAIL ENABLED round-trip: {}", reason);
            std::process::exit(1);
        }
    }

    // Step 7c: 半自动手动模式 + 阈值持久验证
    // 证明 SET_MODE 生效：半自动手动模式跳过阈值处理，FINGER_TH 跨多个标准处理周期保持手动值。
    println!("[SELFTEST] 切换半自动手动模式(SET_MODE=1)...");
    if let Err(e) = ctrl.set_mode(1) {
        println!("[SELFTEST] FAIL set_mode(semi): {}", e);
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(200));
    let th_test: u32 = 199; // 明显区别于自动整定值(≈44)
    if let Err(e) = ctrl.set_param(0, TH_PARAM, th_test) {
        println!("[SELFTEST] FAIL set_param(FINGER_TH): {}", e);
        restore_then_auto(&mut ctrl, "FINGER_TH 测试写入失败");
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(200)); // 跨多个处理周期，验证手动阈值保持不变
    let base_ver = ctrl.param_version();
    if let Err(e) = ctrl.request_params(0) {
        println!("[SELFTEST] FAIL request_params(semi回读): {}", e);
        restore_then_auto(&mut ctrl, "FINGER_TH 回读请求失败");
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
            restore_then_auto(&mut ctrl, "FINGER_TH 持久回读失败");
            std::process::exit(1);
        }
    }
    // Step 7d: 半自动手动模式硬件参数 APPLY 重初始化验证
    // 证明模式修改重初始化生效：改 SNS_CLK_DIV + CALIBRATE(APPLY) 后，
    // 硬件参数持久且手动 FINGER_TH(199) 不被重初始化覆盖。
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
        restore_then_auto(&mut ctrl, "SNS_CLK_DIV 测试写入失败");
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(50));
    if let Err(e) = ctrl.calibrate(0xFFFF_FFFF_FFFF_FFFF) {
        println!("[SELFTEST] FAIL calibrate(APPLY): {}", e);
        restore_then_auto(&mut ctrl, "SNS_CLK_DIV 测试 APPLY 失败");
        std::process::exit(1);
    }
    thread::sleep(Duration::from_millis(400)); // 等主循环重初始化 + 重置基线
    let base_ver = ctrl.param_version();
    if let Err(e) = ctrl.request_params(0) {
        println!("[SELFTEST] FAIL request_params(APPLY回读): {}", e);
        restore_then_auto(&mut ctrl, "SNS_CLK_DIV APPLY 回读请求失败");
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
            restore_then_auto(&mut ctrl, "SNS_CLK_DIV APPLY 回读失败");
            std::process::exit(1);
        }
    }

    // 切回自动校准/标准完整处理前，先恢复完整自测借用的 CH0 参数并确认其已落盘。
    if !restore_ch0_params(&mut ctrl, "进入 CP 验收前") {
        let _ = ctrl.set_mode(0);
        println!("[SELFTEST] FAIL CP 前 CH0 临时参数恢复未确认");
        std::process::exit(1);
    }
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
    let mut cp_empty_channels = Vec::new();
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
                    // ★失败与"无结果"必须分开报★: 0x00FFFFFF 是设备明确的测量失败标记(唯一判据),
                    // 0 只是"响应回来了但没给出值"。两者都不算通过, 但混成一句会误导排查方向。
                    Some(CP_FAILURE_VALUE) => {
                        cp_failed_channels.push(ch);
                        cp_results.push((ch, CP_FAILURE_VALUE));
                        break;
                    }
                    Some(0) => {
                        cp_empty_channels.push(ch);
                        cp_results.push((ch, 0));
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
    if !cp_failed_channels.is_empty() || !cp_empty_channels.is_empty() {
        if !cp_failed_channels.is_empty() {
            println!(
                "[SELFTEST] FAIL CP 测量失败(设备回读 0x{CP_FAILURE_VALUE:08X}) channels={:?}",
                cp_failed_channels
            );
        }
        if !cp_empty_channels.is_empty() {
            println!(
                "[SELFTEST] FAIL CP 无结果(响应到达但值为 0) channels={:?}",
                cp_empty_channels
            );
        }
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
    // core1 新代数通知线的起点读数(见 int1_witness_text)。压测窗口本身就是最好的观测窗口。
    profile: Option<LoopProfile>,
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
    match (start.profile, read_loop_profile(ctrl)) {
        (Some(before), Some(after)) => println!(
            "[SOAK] {}",
            int1_witness_text(&before, &after, elapsed_secs as f32)
        ),
        _ => println!("[SOAK] core1 通知线: unavailable (EP0 剖面读取失败)"),
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
        profile: read_loop_profile(ctrl),
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
    // 容量改成实例方法(设备回报值优先): 这条软压测跑在真机上, 用设备真值填满才是"填到上限"。
    let capacity = ctrl.algo_src_capacity();
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
