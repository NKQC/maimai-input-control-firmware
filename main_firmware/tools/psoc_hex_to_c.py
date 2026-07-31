#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
psoc_hex_to_c.py

将 PSoC4 (CY8C4147) 的 Intel HEX 产物转换为可内嵌进 RP2040 固件的 C 头文件。
仅提取 application flash 区（地址 0x00000000 起），跳过 Cypress 的高地址 metadata/
checksum/protection 段（0x90xxxxxx）。按 128 字节 flash 行对齐，空洞用 0x00 填充。

用法：
    python psoc_hex_to_c.py <input.hex> <output.h> [fw_version_hex]

fw_version_hex 由调用方(dev.ps1 / build.ps1)从 PSoC 的 fw_build_stamp.h 读出的编译时间戳
（十进制 YYMMDDHHMM）换算而来；缺省值仅作占位。
"""

import sys

ROW_SIZE = 128
APP_FLASH_LIMIT = 0x00020000  # CY8C4147AZI-S455: 128 KiB application flash
FILL_BYTE = 0x00


def parse_intel_hex(path):
    """返回 {absolute_address: byte} 字典，仅保留 application flash 区。"""
    mem = {}
    base = 0  # 扩展线性地址高 16 位
    with open(path, "r") as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line[0] != ":":
                continue
            data = bytes.fromhex(line[1:])
            count = data[0]
            addr = (data[1] << 8) | data[2]
            rectype = data[3]
            payload = data[4:4 + count]
            # 校验和
            if (sum(data) & 0xFF) != 0:
                raise ValueError(f"第 {lineno} 行校验和错误")
            if rectype == 0x00:  # data
                full = base + addr
                for i, b in enumerate(payload):
                    a = full + i
                    if a < APP_FLASH_LIMIT:
                        mem[a] = b
            elif rectype == 0x04:  # extended linear address
                base = ((payload[0] << 8) | payload[1]) << 16
            elif rectype == 0x02:  # extended segment address
                base = ((payload[0] << 8) | payload[1]) << 4
            elif rectype == 0x01:  # EOF
                break
            # 0x03 / 0x05 起始地址记录：忽略
    return mem


def build_image(mem):
    if not mem:
        raise ValueError("未解析到任何 application flash 数据")
    max_addr = max(mem.keys())
    length = ((max_addr + 1 + ROW_SIZE - 1) // ROW_SIZE) * ROW_SIZE
    img = bytearray([FILL_BYTE]) * length
    for a, b in mem.items():
        img[a] = b
    return bytes(img)


def emit_header(img, out_path, version):
    rows = len(img) // ROW_SIZE
    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("// 本文件由 tools/psoc_hex_to_c.py 自动生成，请勿手改。")
    lines.append("// 内嵌 PSoC4 application flash 镜像，供 RP2040 经 SWD 烧录。")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"// PSoC 固件版本 = 编译时间戳 十进制 {version} (YYMMDDHHMM), 见 fw_build_stamp.h")
    lines.append(f"static const uint32_t PSOC_FW_VERSION = 0x{version:08X}u;")
    lines.append("")
    lines.append(f"// 镜像总长度 {len(img)} 字节 = {rows} 行 x {ROW_SIZE}B")
    lines.append(f"static const uint32_t PSOC_FW_IMAGE_LEN = {len(img)}u;")
    lines.append("")
    lines.append("static const uint8_t PSOC_FW_IMAGE[] = {")
    for i in range(0, len(img), 16):
        chunk = img[i:i + 16]
        body = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"    {body},")
    lines.append("};")
    lines.append("")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return len(img), rows


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    in_path = sys.argv[1]
    out_path = sys.argv[2]
    version = int(sys.argv[3], 16) if len(sys.argv) > 3 else 0x00000400

    mem = parse_intel_hex(in_path)
    img = build_image(mem)
    total, rows = emit_header(img, out_path, version)
    print(f"OK: {in_path}")
    print(f"  application flash 字节数(含填充) = {total}  行数 = {rows}")
    print(f"  版本 = 0x{version:08X}")
    print(f"  输出 = {out_path}")


if __name__ == "__main__":
    main()
