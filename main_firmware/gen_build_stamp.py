# PlatformIO pre 脚本: 编译前重新生成 RP2040 的编译时间戳版本头。
#
# 编码实现与 PSoC 侧共用 tools/build_stamp_header.py(十进制 YYMMDDHHMM, 本地时间),
# 两端不各算一套。生成物 src/service/psoc_updater/rp_build_stamp.h 同时给出:
#   #define RP_BUILD_STAMP (..u)                        —— 固件用
#   static constexpr uint32_t RP_FIRMWARE_VERSION = 0x..u; —— control_software/build.rs 正则取值
# 内容未变化(同一分钟内)时脚本不重写文件, 避免无谓的全量重编。

Import("env")
import os
import subprocess
import sys

_PROJECT_DIR = env.subst("$PROJECT_DIR")
_GENERATOR = os.path.join(_PROJECT_DIR, "tools", "build_stamp_header.py")
_OUT_HEADER = os.path.join(_PROJECT_DIR, "src", "service", "psoc_updater",
                           "rp_build_stamp.h")

if not os.path.isfile(_GENERATOR):
    raise RuntimeError("[build-stamp] generator missing: %s" % _GENERATOR)

_python = env.subst("$PYTHONEXE") or sys.executable
subprocess.check_call([
    _python, _GENERATOR, _OUT_HEADER, "RP_BUILD_STAMP",
    "--constexpr", "RP_FIRMWARE_VERSION",
])
