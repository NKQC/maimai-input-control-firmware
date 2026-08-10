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
    embed_interception_assets();
}

/// 把 Interception v1.0.1 的**库/安装器/许可证**从固定资产包里解出并内嵌进 exe。
///
/// 为什么要它: 虚拟摄像头的"目标设备"模式必须**吞掉**扫码器按键(否则扫码内容同时打进游戏
/// 输入框)。用户态拿不到"按设备吞键"的能力 —— `WH_KEYBOARD_LL` 分不出来源设备, Raw Input
/// 只能旁路监听。Interception 是键盘类的**上层过滤驱动**, 可按设备过滤并决定是否放行, 是唯一
/// 能同时满足"只吞这一个设备"与"真的不进系统"的路子。
///
/// 授权边界: 官方双授权。非商业用途按 LGPL, 且**明文授予**驱动与安装器这些二进制资产的分发权,
/// 前提是"与驱动的通信只经由该库及其 API" —— 本实现只调 `interception.dll` 的导出函数,
/// 不自己去 DeviceIoControl 驱动, 符合该前提。商业用途须另购授权, 故资产默认不入库(见 .gitignore)。
///
/// 缺资产**不让 cargo 构建失败**(与 `embed_vcam_filters` 同一策略): 内嵌空字节,
/// 由 `vcam::interception::embedded_available()` 在 UI 上如实报"本次构建未内置", 目标模式降级旁路。
fn embed_interception_assets() {
    /// 官方发布包(与 `vcam::interception` 的文案同源, 改一处必须同步另一处)。
    const ARCHIVE: &str = "vendor/interception/Interception.zip";
    /// v1.0.1 发布资产的 SHA-256 与字节数。★对不上就当没有★: 内嵌一个来源不明的内核驱动安装器
    /// 比缺功能危险得多, 所以这里只认这一个确定的版本, 不做"尽力而为"的兜底。
    const SHA256: &str = "ad038963d6413055765128b0b931f6e765147c9916dba79e65d872b261f9af10";
    const SIZE: u64 = 389_119;
    /// (生成的符号, zip 内路径)。目标进程固定 x64，只内嵌运行所需的 x64 API。
    const ENTRIES: [(&str, &str); 3] = [
        ("BYTES_API_X64", "Interception/library/x64/interception.dll"),
        (
            "BYTES_INSTALLER",
            "Interception/command line installer/install-interception.exe",
        ),
        (
            "BYTES_LICENSE",
            "Interception/licenses/non-commercial-usage/LGPL 3.0.txt",
        ),
    ];

    println!("cargo:rerun-if-changed={ARCHIVE}");
    let out_dir = std::env::var("OUT_DIR").expect("cargo 未提供 OUT_DIR");
    let staging = std::path::Path::new(&out_dir).join("interception");
    let mut generated = String::from(
        "// 由 build.rs 生成: Interception 库/安装器/许可证的内嵌字节(空 = 本次构建未携带)。\n",
    );

    let blobs = read_interception_archive(ARCHIVE, SHA256, SIZE, &ENTRIES);
    for (symbol, relative) in ENTRIES {
        match blobs.as_ref().and_then(|map| {
            map.iter()
                .find(|(name, _)| name == relative)
                .map(|(_, bytes)| bytes)
        }) {
            Some(bytes) => {
                // 解出的文件落到 OUT_DIR 再 include_bytes!: 与 vcam DLL 一致, 走绝对路径。
                fs::create_dir_all(&staging).expect("创建 Interception 暂存目录失败");
                let file = staging.join(symbol.to_ascii_lowercase());
                fs::write(&file, bytes).expect("写入 Interception 暂存文件失败");
                let absolute = file
                    .to_str()
                    .expect("暂存路径含非 UTF-8 字符")
                    .replace('\\', r"\\");
                generated.push_str(&format!(
                    "pub const {symbol}: &[u8] = include_bytes!(\"{absolute}\");\n"
                ));
            }
            None => generated.push_str(&format!("pub const {symbol}: &[u8] = &[];\n")),
        }
    }
    fs::write(
        std::path::Path::new(&out_dir).join("interception_embedded.rs"),
        generated,
    )
    .expect("写入 interception_embedded.rs 失败");
}

/// 校验固定资产包并取出所需条目。任一环节不成立就返回 None 并给出 cargo 警告
/// (说明具体是"没这个文件"还是"内容与 v1.0.1 不一致"), 不静默降级。
fn read_interception_archive(
    archive: &str,
    sha256: &str,
    size: u64,
    entries: &[(&str, &str)],
) -> Option<Vec<(String, Vec<u8>)>> {
    use sha2::{Digest, Sha256};

    let path = std::path::Path::new(archive);
    if !path.is_file() {
        println!(
            "cargo:warning=未找到 {archive}，虚拟摄像头目标设备模式将无法吞键(只能旁路监听)；\
             非商业用途可从 Interception v1.0.1 发布页取该包放到此路径"
        );
        return None;
    }
    let bytes = fs::read(path).unwrap_or_else(|error| panic!("读取 {archive} 失败: {error}"));
    let digest = format!("{:x}", Sha256::digest(&bytes));
    if bytes.len() as u64 != size || digest != sha256 {
        println!(
            "cargo:warning={archive} 不是 Interception v1.0.1 的官方资产\
             (期望 {size} 字节/SHA256 {sha256}，实得 {} 字节/{digest})，已按未携带处理",
            bytes.len()
        );
        return None;
    }
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(bytes))
        .unwrap_or_else(|error| panic!("{archive} 不是有效的 zip: {error}"));
    let mut out = Vec::with_capacity(entries.len());
    for (_, relative) in entries {
        use std::io::Read;
        let mut file = zip
            .by_name(relative)
            .unwrap_or_else(|error| panic!("{archive} 内缺少 {relative}: {error}"));
        let mut blob = Vec::with_capacity(file.size() as usize);
        file.read_to_end(&mut blob)
            .unwrap_or_else(|error| panic!("解出 {relative} 失败: {error}"));
        out.push(((*relative).to_string(), blob));
    }
    Some(out)
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
