//! 随程序打包的 JIT 算法 C 源模板。
//!
//! ★为什么放在库里而不是 GUI 二进制里★ 这两份源既是算法页"加载模板"的内容, 也是"设备映射表
//! 无存源(出厂默认算法)"时回灌设备的那一份 —— 后者是设备真值的来源之一, 无头自检要复现同一
//! 行为就必须拿到**同一份**字节。放在 bin 里会逼着第二个二进制再写一次 include_str!, 两处各自
//! 演化后"回灌的源"与"界面显示的源"就会悄悄不一致。

/// 内置 v3.1 HDR 触控算法源, 供算法页"加载模板"与默认算法的源回灌使用。
pub const ALGO_V31_TEMPLATE: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../psoc_firmware/algo/psoc_algo_default.c"
));

/// 纯"白灯演示"算法模板: 触摸即点亮白灯(out_active), 展示 JIT 算法对 PSoC 硬件的绝对可控性。
pub const ALGO_LED_DEMO_TEMPLATE: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../psoc_firmware/algo/psoc_algo_led_demo.c"
));
