use std::fs;

fn main() {
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

    // 媒体源是进程内 COM DLL，由 Frame Server 自己加载，主包不把它作为 Rust 依赖链接；
    // 构建脚本只把已产出的 DLL 字节嵌进上位机，DLL 缺失时生成空数组，让首次单独构建也能通过。
    //
    // 候选顺序(先命中者胜)：
    //   1. MAI2_VCAM_DLL 环境变量指定的任意路径(打包/CI 可覆盖，避免写死本机路径)
    //   2. C++ 子项目 vcam_source_cpp/x64/<Debug|Release>/mai2vcam_source.dll —— 正式实现
    // 都不存在则维持"本次构建未内置媒体源 DLL"的明确降级: 刻意不回退到 target/<profile> 下的
    // 同名 Rust cdylib —— 那个 COM 实现未跑通, 静默嵌进去只会让安装看似成功、实际起不来。
    let manifest = std::path::PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    // MSBuild 的配置名首字母大写，与 cargo 的 profile 名不同。
    let msbuild_config = if std::env::var("PROFILE").unwrap() == "release" {
        "Release"
    } else {
        "Debug"
    };
    let mut candidates: Vec<std::path::PathBuf> = Vec::new();
    println!("cargo:rerun-if-env-changed=MAI2_VCAM_DLL");
    if let Some(path) = std::env::var_os("MAI2_VCAM_DLL") {
        candidates.push(std::path::PathBuf::from(path));
    }
    candidates.push(
        manifest
            .join("vcam_source_cpp")
            .join("x64")
            .join(msbuild_config)
            .join("mai2vcam_source.dll"),
    );
    let generated =
        std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("vcam_embedded.rs");
    let mut found: Option<&std::path::PathBuf> = None;
    for dll in &candidates {
        println!("cargo:rerun-if-changed={}", dll.display());
        if found.is_none() && dll.is_file() {
            found = Some(dll);
        }
    }
    let embedded = match found {
        Some(dll) => format!(
            "pub const EMBEDDED: bool = true;\npub static BYTES: &[u8] = include_bytes!(r#\"{}\"#);\n",
            dll.display()
        ),
        None => "pub const EMBEDDED: bool = false;\npub static BYTES: &[u8] = &[];\n".to_string(),
    };
    fs::write(generated, embedded).expect("write embedded virtual camera source");
}
