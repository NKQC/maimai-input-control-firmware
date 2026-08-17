#pragma once

// 本文件由 main_firmware/tools/build_stamp_header.py 在每次构建时生成，请勿手改。
// 固件版本 = 十进制 YYMMDDHHMM（本地时间，分钟精度）；上位机补 "20" 前缀还原完整时间。
// 已知上限：YY <= 42 才放得进 uint32（4294967295），2043 年起需改口径。

#define RP_BUILD_STAMP (2608180218u)

#include <stdint.h>

// 十六进制字面量形式供 control_software/build.rs 正则提取；与上面的十进制同值。
static constexpr uint32_t RP_FIRMWARE_VERSION = 0x9B75ABFAu;
static_assert(RP_FIRMWARE_VERSION == RP_BUILD_STAMP, "build stamp hex/dec mismatch");
