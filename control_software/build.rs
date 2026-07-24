use std::fs;

fn main() {
    slint_build::compile("ui/app.slint").unwrap();

    const PSOC_HEADER: &str = "../main_firmware/src/protocol/psoc/psoc_fw_image.h";
    println!("cargo:rerun-if-changed={PSOC_HEADER}");
    let source = fs::read_to_string(PSOC_HEADER).expect("read generated PSoC image header");
    let version = source.lines()
        .find(|line| line.contains("PSOC_FW_VERSION = 0x"))
        .and_then(|line| line.split("0x").nth(1))
        .and_then(|value| value.split('u').next())
        .and_then(|value| u32::from_str_radix(value, 16).ok())
        .expect("parse PSOC_FW_VERSION from generated image header");
    println!("cargo:rustc-env=EXPECTED_PSOC_FW_VERSION={version}");
}
