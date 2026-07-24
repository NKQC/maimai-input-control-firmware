# JIT 触控算法引擎 / 单 exe 资源打包

## 现状(已完成, build-ui-release 通过, exe 38MB)
JIT 触控算法编译链所需的一切资源已内嵌进单个 mai2control-ui.exe, 任意 Windows 设备无需预装工具链即可编译。

### 打包内容
- `control_software/assets/toolchain.zip`(18.1MB): 最小 arm-none-eabi 工具链
  - `bin/`: arm-none-eabi-gcc.exe / objcopy.exe / nm.exe / objdump.exe
  - `libexec/gcc/arm-none-eabi/14.2.1/cc1.exe`(C 编译器)
  - `arm-none-eabi/bin/as.exe`(汇编器)
  - `lib/gcc/arm-none-eabi/14.2.1/include/`(GCC 内建头: stdint.h/stddef.h 等)
  - 来源: `C:\Users\asdfg\Infineon\Tools\mtb-gcc-arm-eabi\14.2.1\gcc`(静态链接, 无 DLL 依赖)
  - zip 条目用 forward-slash(.NET System.IO.Compression 手工生成, 非 Compress-Archive 的反斜杠)
- ABI 头 `psoc_algo_abi.h`: include_str! 内嵌(源: psoc_firmware/CY8C4147AZI-SensorCore)

### 代码(control_software/src/app_state/mod.rs)
- `BUNDLED_TOOLCHAIN_ZIP = include_bytes!(assets/toolchain.zip)`
- `ALGO_ABI_HEADER = include_str!(../psoc_firmware/.../psoc_algo_abi.h)`
- `BUNDLED_TOOLCHAIN_TAG = "mtb-arm-14.2.1-min1"`(改打包内容时递增使缓存失效)
- `ensure_bundled_toolchain()`: 首次解压 zip 到 `%LOCALAPPDATA%\mai2control-ui\toolchain\<tag>`, 写 `.ready` marker, 返回 bin 目录; 之后直接命中缓存
- `compile_c_to_blob()`: gcc_dir = ensure_bundled_toolchain() 失败回退 find_toolchain_dir(); ABI 头写入 tmp 并 `-I tmp`(已删硬编码 F:\ 路径)
- `find_toolchain_dir()`: 保留为回退, 动态扫描 ModusToolbox tools_*/gcc/bin + Infineon mtb-gcc-arm-eabi/*/gcc/bin + PlatformIO packages + PATH(不写死版本/用户名)
- Cargo.toml: `zip = { version = "2", default-features = false, features = ["deflate"] }`

### 编译参数(不变)
`-mcpu=cortex-m0plus -mthumb -Os -ffreestanding -fno-jump-tables -fomit-frame-pointer -fno-common -nostdlib -c` → objcopy -O binary -j .text → nm 校验(无 U 符号, algo@0) → objdump -d 反汇编存 algo_asm 供 UI 子标签

### 验证
从 zip 解压后完整跑通 gcc(exit0)/objcopy(exit0)/nm(`00000000 T algo`)/objdump(反汇编正常), 产物 10 字节 blob。

## 重新打包工具链(若需换版本)
1. staged: 拷贝上述最小文件集到临时目录, 保留目录结构
2. 用 .NET System.IO.Compression 逐条 CreateEntry, 条目名 `.Replace('\','/')`
3. 覆盖 assets/toolchain.zip, 递增 BUNDLED_TOOLCHAIN_TAG
