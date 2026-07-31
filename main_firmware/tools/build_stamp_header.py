#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_stamp_header.py

生成"编译时间戳版本"头文件，两端固件(PSoC / RP2040)共用同一份编码实现，避免各处自己算。

编码约定：版本号 = 十进制 YYMMDDHHMM（本地时间，分钟精度）。
    2026-07-31 04:24 -> 2607310424
该值必须放得进 uint32（上限 4294967295）；已知上限 YY <= 42，即 2043 年起会溢出，
届时需要改口径（C 侧有编译期断言兜底，溢出会直接编译失败而不是静默截断）。
上位机按十进制解，前缀补 "20" 还原 YYYYMMDDHHMM。

用法:
    build_stamp_header.py <out.h> <MACRO_NAME> [--constexpr <CppConstName>]

--constexpr 额外产出 `static constexpr uint32_t <Name> = 0x........u;`：
control_software/build.rs 沿用现有正则从十六进制字面量取值，故该形式必须保留。

内容未变化时不重写文件（同一分钟内的增量构建不会被无谓地全量重编）。
"""

import os
import sys
import time

STAMP_MAX = 0xFFFFFFFF


def build_stamp(now=None):
    """本地时间 -> 十进制 YYMMDDHHMM。

    modus-shell(bash) 会带进 TZ=Asia/Shanghai 这种 Olson 名, Windows CRT 解不了会算出
    错误偏移(实测差 7 小时)。构建时间戳必须与用户的墙上时钟一致, 所以先把 TZ 从环境里摘掉,
    让 localtime 直接用操作系统时区。必须在第一次 localtime 之前摘, CRT 才会重新取。
    """
    os.environ.pop("TZ", None)
    if hasattr(time, "tzset"):
        time.tzset()
    t = time.localtime(now)
    return (((t.tm_year % 100) * 100 + t.tm_mon) * 100 + t.tm_mday) * 10000 \
        + t.tm_hour * 100 + t.tm_min


def render(macro, stamp, cpp_const=None):
    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("// 本文件由 main_firmware/tools/build_stamp_header.py 在每次构建时生成，请勿手改。")
    lines.append("// 固件版本 = 十进制 YYMMDDHHMM（本地时间，分钟精度）；上位机补 \"20\" 前缀还原完整时间。")
    lines.append("// 已知上限：YY <= 42 才放得进 uint32（4294967295），2043 年起需改口径。")
    lines.append("")
    lines.append("#define %s (%uu)" % (macro, stamp))
    if cpp_const:
        lines.append("")
        lines.append("#include <stdint.h>")
        lines.append("")
        lines.append("// 十六进制字面量形式供 control_software/build.rs 正则提取；与上面的十进制同值。")
        lines.append("static constexpr uint32_t %s = 0x%08Xu;" % (cpp_const, stamp))
        lines.append("static_assert(%s == %s, \"build stamp hex/dec mismatch\");" % (cpp_const, macro))
    lines.append("")
    return "\n".join(lines)


def write_if_changed(path, text):
    old = None
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as f:
            old = f.read()
    if old == text:
        return False
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return True


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 1
    out_path = argv[1]
    macro = argv[2]
    cpp_const = None
    rest = argv[3:]
    while rest:
        if rest[0] == "--constexpr" and len(rest) >= 2:
            cpp_const = rest[1]
            rest = rest[2:]
        else:
            print("unknown argument: %s" % rest[0])
            return 1

    stamp = build_stamp()
    if stamp > STAMP_MAX:
        print("build stamp %u exceeds uint32; encoding must be revised" % stamp)
        return 1
    text = render(macro, stamp, cpp_const)
    changed = write_if_changed(out_path, text)
    print("[build-stamp] %s = %u (%s) %s" % (
        macro, stamp, time.strftime("%Y-%m-%d %H:%M"),
        "written" if changed else "unchanged(same minute)"))
    print("[build-stamp] %s" % os.path.abspath(out_path))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
