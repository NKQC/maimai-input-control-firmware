# Path-B fix for the TinyUSB dual-stack link conflict (task: usb-multicdc).
#
# 背景:框架预编译库 libpico.a 自带一整套 TinyUSB 设备栈,是用框架默认
# tusb_config(CFG_TUD_CDC=1 / CFG_TUD_HID=2)编译的。本工程又用自带的
# src/hal/usb/tusb_config.h(CFG_TUD_CDC=3)编译了 Adafruit TinyUSB。两套栈
# 定义相同符号;一旦 libpico 的 CDC=1 版本在链接中胜出,TinyUSB 的每实例数组
# _cdcd_itf[CFG_TUD_CDC] 只有 1 槽,打不开描述符里第 2/3 个 CDC 接口,导致
# Windows 枚举「设备无法启动 (代码 10)」。
#
# 本脚本在 build 目录里生成 libpico.a 的一个副本,用 ar 删掉其中所有会与
# Adafruit 重复的 TinyUSB 成员,再把链接指向该副本 —— 于是整套 TinyUSB 只由
# Adafruit(CDC=3)提供,单栈一致。框架包原件保持不动。
#
# 保留 rp2040_usb_device_enumeration.c.o:它是 Pico SDK 独有的 RP2040 枚举
# 勘误修复,Adafruit 未提供,删掉会产生未定义引用。

Import("env")
import glob
import os
import shutil
import subprocess

# libpico.a 中会被 Adafruit TinyUSB 重新提供的成员(经 `ar t` 核实)。
_TINYUSB_MEMBERS = [
    "dcd_rp2040.c.o", "rp2040_usb.c.o", "usbd.c.o", "usbd_control.c.o",
    "audio_device.c.o", "cdc_device.c.o", "dfu_device.c.o", "dfu_rt_device.c.o",
    "hid_device.c.o", "midi_device.c.o", "msc_device.c.o", "ecm_rndis_device.c.o",
    "ncm_device.c.o", "usbtmc_device.c.o", "vendor_device.c.o", "video_device.c.o",
    "tusb.c.o", "tusb_fifo.c.o",
]


def _basename(node):
    return os.path.basename(str(node))


_ar = env.subst("$AR") or "arm-none-eabi-ar"
_build_dir = env.subst("$BUILD_DIR")
os.makedirs(_build_dir, exist_ok=True)

_new_libs = []
_patched = False
for _lib in list(env["LIBS"]):
    if _basename(_lib) == "libpico.a":
        _src = env.File(_lib).get_abspath()
        _dst = os.path.join(_build_dir, "libpico_nousb.a")
        shutil.copyfile(_src, _dst)
        subprocess.check_call([_ar, "d", _dst] + _TINYUSB_MEMBERS)
        _new_libs.append(env.File(_dst))
        _patched = True
        print("[strip_libpico] stripped %d TinyUSB members -> %s"
              % (len(_TINYUSB_MEMBERS), _dst))
    else:
        _new_libs.append(_lib)

if _patched:
    env.Replace(LIBS=_new_libs)
else:
    print("[strip_libpico] WARNING: libpico.a not found in LIBS; no change made")

# 输出链接 map,便于静态核对 TinyUSB 目标全部来自 Adafruit、无一来自 libpico。
env.Append(LINKFLAGS=["-Wl,-Map=%s" % os.path.join(_build_dir, "firmware.map")])

# Adafruit WebUSB 示例也以弱符号提供这两个应用层回调。工程自己的同名回调
# 同样是 TinyUSB 声明的弱符号；若二者同时参与链接，库的链接顺序会使
# Adafruit 的 WebUSB BOS(57 bytes, vendor code 2)胜出，覆盖工程的纯 MS OS
# 2.0 BOS(33 bytes, vendor code 1)。链接前只重命名该对象内的两项回调，框架
# 安装目录和其余 Adafruit TinyUSB 核心对象均不修改。
_WEBUSB_OBJECT_GLOB = os.path.join(
    _build_dir, "lib*", "Adafruit_TinyUSB_Arduino", "arduino", "webusb",
    "Adafruit_USBD_WebUSB.cpp.o")
_WEBUSB_CALLBACKS = {
    "tud_descriptor_bos_cb": "adafruit_tud_descriptor_bos_cb_unused",
    "tud_vendor_control_xfer_cb": "adafruit_tud_vendor_control_xfer_cb_unused",
}


def _nm_symbols(path):
    nm = env.subst("$NM") or "arm-none-eabi-nm"
    try:
        output = subprocess.check_output([nm, "-g", path], text=True,
                                         stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError("[strip_libpico] nm failed for %s:\n%s" %
                           (path, exc.output))
    return {line.split()[-1] for line in output.splitlines() if line.split()}


def _rename_webusb_callbacks(source, target, env):
    matches = glob.glob(_WEBUSB_OBJECT_GLOB)
    if len(matches) != 1:
        raise RuntimeError("[strip_libpico] expected exactly one Adafruit WebUSB object "
                           "matching %s, found %d: %s" %
                           (_WEBUSB_OBJECT_GLOB, len(matches), matches))

    webusb_obj = matches[0]
    symbols = _nm_symbols(webusb_obj)
    original = set(_WEBUSB_CALLBACKS)
    renamed = set(_WEBUSB_CALLBACKS.values())

    if original.issubset(symbols):
        objcopy = env.subst("$OBJCOPY") or "arm-none-eabi-objcopy"
        command = [objcopy]
        for old, new in _WEBUSB_CALLBACKS.items():
            command.extend(["--redefine-sym", "%s=%s" % (old, new)])
        command.append(webusb_obj)
        try:
            subprocess.check_call(command)
        except subprocess.CalledProcessError as exc:
            raise RuntimeError("[strip_libpico] objcopy failed for %s with exit code %d" %
                               (webusb_obj, exc.returncode))

        symbols = _nm_symbols(webusb_obj)
        if not renamed.issubset(symbols) or original.intersection(symbols):
            raise RuntimeError("[strip_libpico] callback rename verification failed for %s" %
                               webusb_obj)
        print("[strip_libpico] renamed conflicting Adafruit WebUSB callbacks -> %s" %
              webusb_obj)
    elif renamed.issubset(symbols) and not original.intersection(symbols):
        print("[strip_libpico] Adafruit WebUSB callbacks already renamed; keeping incremental build object")
    else:
        raise RuntimeError("[strip_libpico] Adafruit WebUSB callback symbols are in an "
                           "unexpected partial state: %s" % webusb_obj)


# Execute after all objects have been built but before the ELF link action. This action
# is intentionally fatal: linking a firmware whose BOS callback was not isolated is unsafe.
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", _rename_webusb_callbacks)
