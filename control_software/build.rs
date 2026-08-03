use std::fs;

fn main() {
    embed_windows_icon();

    slint_build::compile("ui/app.slint").unwrap();

    const PSOC_HEADER: &str = "../main_firmware/src/protocol/psoc/psoc_fw_image.h";
    println!("cargo:rerun-if-changed={PSOC_HEADER}");
    let psoc_source = fs::read_to_string(PSOC_HEADER).expect("read generated PSoC image header");
    let psoc_version = psoc_source
        .lines()
        .find(|line| line.contains("PSOC_FW_VERSION = 0x"))
        .and_then(|line| line.split("0x").nth(1))
        .and_then(|value| value.split('u').next())
        .and_then(|value| u32::from_str_radix(value, 16).ok())
        .expect("parse PSOC_FW_VERSION from generated image header");
    println!("cargo:rustc-env=EXPECTED_PSOC_FW_VERSION={psoc_version}");

    // 该头由 main_firmware 的 pre:gen_build_stamp.py 每次构建生成(版本=编译时间戳 YYMMDDHHMM),
    // 十六进制字面量形式专为下面这段正则保留。
    const RP_HEADER: &str = "../main_firmware/src/service/psoc_updater/rp_build_stamp.h";
    println!("cargo:rerun-if-changed={RP_HEADER}");
    let rp_source = fs::read_to_string(RP_HEADER).expect("read RP2040 firmware version header");
    let rp_version = rp_source
        .lines()
        .find(|line| line.contains("RP_FIRMWARE_VERSION = 0x"))
        .and_then(|line| line.split("0x").nth(1))
        .and_then(|value| value.split('u').next())
        .and_then(|value| u32::from_str_radix(value, 16).ok())
        .expect("parse RP_FIRMWARE_VERSION from firmware version header");
    println!("cargo:rustc-env=EXPECTED_RP_FW_VERSION={rp_version}");

    embed_vcam_filters();
}

/// 把 DirectShow 虚拟摄像头过滤器的 **两个位宽** DLL 内嵌进 exe。
///
/// 为什么内嵌: 安装是"用户点一下 → 提权 → regsvr32"的一次性动作, 独立分发两个 DLL 会
/// 变成"文件丢了就装不上"的支持负担。两个位宽都带: 32 位应用只看 WOW6432Node 视图,
/// 少一个就表现为"32 位游戏里看不到这个摄像头"。
///
/// DLL 由 MSBuild 构建(`vcam_source_cpp/mai2vcam_dshow.vcxproj`, Release|x64 与 Release|Win32),
/// 这里只负责搬运。**缺文件不让 cargo 构建失败**: 上位机的其余功能与摄像头无关,
/// 缺失时内嵌为空字节, 由 `vcam::backend::embedded_available()` 在 UI 上明确报"本次构建未内置"。
fn embed_vcam_filters() {
    const SOURCES: [(&str, &str); 2] = [
        (
            "BYTES_DS_X64",
            "vcam_source_cpp/x64/Release/mai2vcam_dshow.dll",
        ),
        (
            "BYTES_DS_X86",
            "vcam_source_cpp/Win32/Release/mai2vcam_dshow.dll",
        ),
    ];
    let out_dir = std::env::var("OUT_DIR").expect("cargo 未提供 OUT_DIR");
    let mut generated = String::from(
        "// 由 build.rs 生成: DirectShow 过滤器 DLL 的内嵌字节(空 = 本次构建未携带)。\n",
    );
    for (symbol, relative) in SOURCES {
        println!("cargo:rerun-if-changed={relative}");
        let source = std::path::Path::new(relative);
        if source.is_file() {
            // include_bytes! 走绝对路径, 避免受 OUT_DIR 相对位置影响。
            let absolute = std::fs::canonicalize(source)
                .unwrap_or_else(|error| panic!("解析 {relative} 绝对路径失败: {error}"));
            let absolute = absolute
                .to_str()
                .expect("DLL 路径含非 UTF-8 字符")
                .trim_start_matches(r"\\?\")
                .replace('\\', r"\\");
            generated.push_str(&format!(
                "pub const {symbol}: &[u8] = include_bytes!(\"{absolute}\");\n"
            ));
        } else {
            println!(
                "cargo:warning=未找到 {relative}，虚拟摄像头安装将不可用(先用 MSBuild 构建 vcam_source_cpp)"
            );
            generated.push_str(&format!("pub const {symbol}: &[u8] = &[];\n"));
        }
    }
    fs::write(
        std::path::Path::new(&out_dir).join("vcam_embedded.rs"),
        generated,
    )
    .expect("写入 vcam_embedded.rs 失败");
}

/// 把 `ui/assets/icon.ico` 作为 Win32 ICON 资源嵌进可执行文件，
/// 使 exe 在资源管理器 / 任务栏 / Alt-Tab 里显示图标（而非默认空白图标）。
///
/// 要点:
///  - 目标判定用 `CARGO_CFG_TARGET_OS` 而不是 `cfg!(windows)`：build.rs 里的 `cfg` 描述的是
///    **宿主**平台，交叉编译时会判错；`CARGO_CFG_TARGET_OS` 才是真正的目标平台。
///  - MSVC ABI 下 winresource 需要 Windows SDK 的 `rc.exe`。找不到时**不让整个构建失败**：
///    图标只是外观，缺它不影响功能，故降级为 warning 继续编译。
///  - ★该资源会附加到本 crate 的**所有** bin（`mai2control-ui.exe` 与 `selftest.exe`）★——
///    winresource 无法按 bin 目标分别设置资源。selftest 跟着有图标无害，故不额外绕。
///  - `.ico` 是从同目录 `icon.png`(1024×1024) 派生的多尺寸容器(16/24/32/48/64/128/256, PNG 编码)。
///    换图标时请重新生成 `.ico`，否则两者会不一致。
fn embed_windows_icon() {
    const ICON: &str = "ui/assets/icon.ico";
    println!("cargo:rerun-if-changed={ICON}");

    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("windows") {
        return;
    }
    if !std::path::Path::new(ICON).is_file() {
        println!("cargo:warning=未找到 {ICON}，exe 将使用默认图标");
        return;
    }

    let mut res = winresource::WindowsResource::new();
    res.set_icon(ICON);
    if let Err(error) = res.compile() {
        println!(
            "cargo:warning=嵌入 exe 图标失败({error})，继续构建；如需图标请确认 Windows SDK 的 rc.exe 可用"
        );
    }
}
