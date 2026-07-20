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
    BRINGUP_FLAG_CHECKSUM, BRINGUP_FLAG_LINK, BRINGUP_FLAG_SNAPSHOT,
    EXPECTED_PSOC_S455_ID, RP_BUILD_ID_DIAGNOSTIC_V1, CfgValue,
};
use std::thread;
use std::time::Duration;

const HELLO_TIMEOUT_MS: u64 = 1500;
const CFG_GET_ALL_TIMEOUT_MS: u64 = 1000;
const TELEM_TIMEOUT_MS: u64 = 800;
const PARAM_GET_ALL_TIMEOUT_MS: u64 = 1000;

fn main() {
    env_logger::init();

    let args: Vec<String> = std::env::args().collect();
    let reboot_bootloader = args.iter().any(|a| a == "--reboot-bootloader");
    let reboot_bootloader_only = args.iter().any(|a| a == "--reboot-bootloader-only");
    let smoke_only = args.iter().any(|a| a == "--smoke");
    let diagnose_only = args.iter().any(|a| a == "--diagnose");
    let csd_provision = args.iter().any(|a| a == "--csd-provision");
    let csd_verify = args.iter().any(|a| a == "--csd-verify");
    let soak = args.iter().any(|a| a == "--soak");
    let list_only = args.iter().any(|a| a == "--list-only");
    let debug_read = args.iter().any(|a| a == "--debug-read");
    let ctrl_bootsel = args.iter().any(|a| a == "--ctrl-bootsel");
    // 只请求配置并观测：每 200ms 打印 config_entries 数，持续 ~2.5s，看是否/何时到达及项数。
    let cfg_only = args.iter().any(|a| a == "--cfg-only");
    // 纯空闲复现：连接+DEVICE_INFO 后立即 idle(不驱动任何功能)，模拟 GUI "连上就放着看"。
    let idle_only = args.iter().any(|a| a == "--idle-only");
    // 恢复设备配置：发 RESET_DEFAULTS 令固件 _runtime_map = _default_map(完整 schema) 并保存。
    let reset_config = args.iter().any(|a| a == "--reset-config");
    // --soak 空闲时长(秒)，默认 30；命令行可 `--soak-idle 60`
    let soak_idle_s: u64 = args
        .iter()
        .position(|a| a == "--soak-idle")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(30);

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
                println!("[DBG] vendor_tx_calls={}", le32(20));
                println!("[DBG] vendor_tx_bytes={}", le32(24));
                println!("[DBG] flash_write_count={}", le32(28));
                println!("[DBG] loop_at_last_flash={}", le32(32));
                println!("[DBG] rearm_count={}", le32(36));
                println!(
                    "[DBG] out_busy={} out_stalled={} mounted={} debug_enabled={}",
                    b[40], b[41], b[42], b[43]
                );
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

    // Step 3: 轮询等待 HELLO 并收 DEVICE_INFO
    println!("[SELFTEST] 轮询 DEVICE_INFO...");
    let start = std::time::Instant::now();
    let timeout = Duration::from_millis(HELLO_TIMEOUT_MS);
    loop {
        ctrl.poll();
        if ctrl.device_info_text() != "未获取到设备信息" {
            println!("[SELFTEST] 成功获取 DEVICE_INFO:");
            println!("{}", ctrl.device_info_text());
            break;
        }
        if start.elapsed() > timeout {
            println!("[SELFTEST] FAIL 未在 {}ms 内收到 DEVICE_INFO", HELLO_TIMEOUT_MS);
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(50));
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

    // Diagnose is intentionally read-only and accepts legacy short DEVICE_INFO payloads.
    if diagnose_only {
        println!("[SELFTEST] DIAGNOSE PASS");
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
        let identity_ok = info.fw_version == 0x0000_0401
            && diag.rp_build_id == RP_BUILD_ID_DIAGNOSTIC_V1
            && diag.embedded_psoc_version == 0x0000_0401
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
                "[SELFTEST] FAIL diagnostic identity/runtime: identity_ok={} runtime_ok={} last={} failure={} flags=0x{:04X}",
                identity_ok, runtime_ok, diag.last_stage, diag.failure_stage, diag.flags
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
        println!("[SELFTEST] CSD provision: 捕获 PSoC 自整定参数入 store...");
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
                println!("[SELFTEST] CSD VERIFY PASS: FINGER_TH ch0=199 (启动下发成功,PSoC 无状态验证通过)");
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
        println!("[SELFTEST] 2.5s 后 config_entries = {}", ctrl.config_entries().len());
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
        println!("[SOAK] 纯空闲复现: DEVICE_INFO 后立即 idle {}s (不驱动功能)", soak_idle_s);
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

    if soak {
        run_soak(&mut ctrl, soak_idle_s);
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
    // 用 onDebounce(0x05):SmartSense 不自动管理,不会被每周期处理覆盖,可干净验证写入机制。
    const TEST_PARAM: u8 = 0x05; // ON_DEBOUNCE
    let orig_val = match ctrl.param(0, TEST_PARAM) {
        Some(v) => v,
        None => {
            println!("[SELFTEST] FAIL round-trip: 未找到 ON_DEBOUNCE");
            std::process::exit(1);
        }
    };
    let test_val = if orig_val == 5 { 7 } else { 5 };
    println!("[SELFTEST] SET_PARAM round-trip: ON_DEBOUNCE {} -> {}", orig_val, test_val);
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
            println!("[SELFTEST] FAIL round-trip: 期望 {} 实得 {:?}", test_val, other);
            let _ = ctrl.set_param(0, TEST_PARAM, orig_val);
            std::process::exit(1);
        }
    }
    // 恢复原值(PSoC 无状态,重启即恢复;此处仍主动还原保持一致)
    let _ = ctrl.set_param(0, TEST_PARAM, orig_val);
    thread::sleep(Duration::from_millis(50));

    // Step 7c: 半自动模式 + 阈值持久验证
    // 证明 SET_MODE 生效:semi 模式下 SmartSense 阈值自整定被跳过,手动 FINGER_TH 跨多个处理周期不被覆盖。
    const TH_PARAM: u8 = 0x01; // FINGER_TH (全自动模式下会被 SmartSense 每周期重算)
    println!("[SELFTEST] 切换半自动模式(SET_MODE=1)...");
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
    thread::sleep(Duration::from_millis(200)); // 跨多个处理周期,验证不被 SmartSense 覆盖
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
                "[SELFTEST] FAIL semi-mode 阈值未持久: 期望 {} 实得 {:?} (SmartSense 未被跳过?)",
                th_test, other
            );
            let _ = ctrl.set_mode(0);
            std::process::exit(1);
        }
    }
    // Step 7d: 半自动模式硬件参数 APPLY 重初始化验证
    // 证明"模式修改重初始化"生效:semi 模式改 SNS_CLK_DIV + CALIBRATE(APPLY) 后,
    // 硬件参数持久且手动 FINGER_TH(199) 不被重初始化覆盖(Initialize 不重跑 SmartSense)。
    const CLK_PARAM: u8 = 0x08; // SNS_CLK_DIV
    let clk_orig = ctrl.param(0, CLK_PARAM).unwrap_or(16);
    let clk_test = if clk_orig >= 8 && clk_orig < 250 { clk_orig + 2 } else { 16 };
    println!("[SELFTEST] semi-mode 硬件参数 APPLY: SNS_CLK_DIV {} -> {}", clk_orig, clk_test);
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
            println!("[SELFTEST] APPLY 重初始化 OK: SNS_CLK={} 保持, FINGER_TH={} 未被覆盖", c, t)
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

    // 切回全自动(PSoC 无状态,重启亦恢复)
    let _ = ctrl.set_mode(0);
    thread::sleep(Duration::from_millis(50));

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

/// 无头 exerciser：逐个驱动 UI 全部功能后长时 idle，复现并回验稳定性。
fn run_soak(ctrl: &mut AppController, idle_s: u64) {
    println!("[SOAK] 开始无头 exerciser (模拟 UI 持久连接 + 16ms 轮询)");

    // A. 拉全部配置
    println!("[SOAK] A: request_config_all");
    let _ = ctrl.request_config_all();
    soak_pump(ctrl, 800, "config_all");
    println!("[SOAK]   配置项 = {}", ctrl.config_entries().len());

    // B. 改一个数值配置(设为当前值，无副作用地走 CFG_SET 路径)
    if let Some(entry) = ctrl
        .config_entries()
        .into_iter()
        .find(|e| matches!(e.value, CfgValue::U8(_) | CfgValue::U16(_) | CfgValue::U32(_)))
    {
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
    println!("[SOAK] D: start_telemetry(全通道 100Hz)");
    let _ = ctrl.start_telemetry(100, 0x1F, 0xFFFF_FFFF_FFFF_FFFFu64);
    let start = std::time::Instant::now();
    let mut got = false;
    while start.elapsed() < Duration::from_millis(2500) {
        ctrl.poll();
        if ctrl.state() == ConnState::Disconnected {
            println!("[SOAK] FAIL 断开 during telem 等真实数据: {:?}", ctrl.last_error());
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
        .filter(|&c| ctrl.telem_latest(c).map(|s| s.raw.unwrap_or(0) != 0).unwrap_or(false))
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

    // D2. 持续遥测 5s(采样时快慢路并发压测：观察 link 稳定 / 是否 stall)
    println!("[SOAK] D2: 持续遥测 5s(并发压测)");
    soak_pump(ctrl, 5000, "telemetry_run");

    // E. 半自动 + 捕获 + 调参 + 校准 + 回全自动
    println!("[SOAK] E: set_mode(semi)/capture/set_param/calibrate/set_mode(auto)");
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

    // F. 停遥测
    println!("[SOAK] F: stop_telemetry");
    let _ = ctrl.stop_telemetry();
    soak_pump(ctrl, 300, "stop_telem");

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
