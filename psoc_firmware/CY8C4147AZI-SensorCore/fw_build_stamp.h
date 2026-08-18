#pragma once

// 本文件由 main_firmware/tools/build_stamp_header.py 在每次构建时生成，请勿手改。
// 固件版本 = 十进制 YYMMDDHHMM（本地时间，分钟精度）；上位机补 "20" 前缀还原完整时间。
// 已知上限：YY <= 42 才放得进 uint32（4294967295），2043 年起需改口径。

#define FW_BUILD_STAMP (2608190146u)
