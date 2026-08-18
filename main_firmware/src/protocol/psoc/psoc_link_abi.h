#pragma once

// ★唯一真相源转发★ 帧格式/命令码/状态位/容量常量/CRC8 全部只在 psoc_firmware 那一份里定义。
// PlatformIO 按"包含文件所在目录"解析相对路径, 故这里能直接跨到 PSoC 工程去取, 不需要额外 -I。
// 之所以留这个一行转发头而不让每个 .cpp 各写一遍长路径: 路径只在这一处出现, PSoC 工程改名时
// 只需改这一行, 且 RP 侧任何地方都不可能"忘了 include 而自己另写一份数字"。
// 契约头的注释里写着 `protocol/psoc/*` 这样的路径, GCC 的 -Wcomment 会把 `/*` 当成嵌套注释开头。
// 那份头是两端共用的真相源, 不为一条无害的排版告警去改它; 在这里就地静音, 范围只有这一行 include。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcomment"
#include "../../../../psoc_firmware/CY8C4147AZI-SensorCore/psoc_link_abi.h"
#pragma GCC diagnostic pop
