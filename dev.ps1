# mai2control-v4 固定开发/烧录/诊断脚本
# 用法: powershell -ExecutionPolicy Bypass -File dev.ps1 <action>
# 目的: 所有 RP2040/PSoC/上位机 操作统一走此固定文件, 避免每次不同命令触发鉴权。
#
# actions:
#   build-rp    编译 RP2040 固件 (main_firmware, pio run)
#   build-ui    编译上位机 (control_software, cargo build --bins)
#   diagnose    连 WinUSB 打印完整 DEVICE_INFO bring-up 诊断 (只读)
#   smoke       连 WinUSB 严格校验 flash_ok+link+snapshot+silicon (PASS/FAIL)
#   full        selftest 完整流程 (DEVICE_INFO/config/telem/param)
#   bootsel     令当前运行的 RP2040 进入 BOOTSEL (G:), 等待就绪
#   flash-rp    复制 firmware.uf2 到 G: (需已在 BOOTSEL)
#   cycle       一键: 进BOOTSEL -> 烧RP2040 -> 等重启+自动烧PSoC -> 打印诊断
#   reflash     可靠刷写: 新鲜BOOTSEL -> 复制 -> 确认离开BOOTSEL -> 等bringup -> 诊断
#   build-all   全量构建: PSoC make + embed + RP2040 UF2 + Rust UI/selftest (调 build.ps1)
#   build-blob  编译默认HDR算法blob: gcc->objcopy->nm/objdump校验->生成C数组头+CRC16
#   status      打印 G: 状态 + 关键产物是否存在

param(
    [Parameter(Position=0)]
    [string]$Action = 'status',
    [Parameter(Position=1)]
    [string]$Version = ''
)

$ErrorActionPreference = 'Continue'
$root     = $PSScriptRoot
$fw       = Join-Path $root 'main_firmware'
$cs       = Join-Path $root 'control_software'
$psoc     = Join-Path $root 'psoc_firmware\CY8C4147AZI-SensorCore'
$psocMain = Join-Path $psoc 'main.c'
$psocStampHdr = Join-Path $psoc 'fw_build_stamp.h'
$psocHexDir = Join-Path $psoc 'build\last_config'
$psocConverter = Join-Path $fw 'tools\psoc_hex_to_c.py'
$psocImage = Join-Path $fw 'src\protocol\psoc\psoc_fw_image.h'
$uf2      = Join-Path $fw '.pio\build\pico\firmware.uf2'
$selftest = Join-Path $cs 'target\debug\selftest.exe'
$buildAll = Join-Path $fw 'build.ps1'
$blobDir  = Join-Path $root 'psoc_firmware\algo'
$blobSrc  = Join-Path $blobDir 'psoc_algo_default.c'
$blobObj  = Join-Path $blobDir 'psoc_algo_default.o'
$blobBin  = Join-Path $blobDir 'psoc_algo_default.bin'
$blobHdr  = Join-Path $fw 'src\service\psoc_algo\psoc_algo_default.h'

function Get-Crc16Ccitt([byte[]]$data) {
    $crc = 0xFFFF
    foreach ($x in $data) {
        $crc = $crc -bxor ([int]$x -shl 8)
        for ($i = 0; $i -lt 8; $i++) {
            if ($crc -band 0x8000) { $crc = (($crc -shl 1) -bxor 0x1021) -band 0xFFFF }
            else { $crc = ($crc -shl 1) -band 0xFFFF }
        }
    }
    return $crc
}

function Invoke-NativeTail(
    [string]$Name,
    [string]$WorkingDirectory,
    [string]$FilePath,
    [string[]]$Arguments,
    [int]$TailLines
) {
    Push-Location -LiteralPath $WorkingDirectory
    $output = @()
    $exitCode = 1
    try {
        $output = @(& $FilePath @Arguments 2>&1)
        $exitCode = $LASTEXITCODE
        if ($null -eq $exitCode) { $exitCode = 1 }
    }
    catch {
        $output += $_
        $exitCode = 1
    }
    finally {
        Pop-Location
    }
    $output | Select-Object -Last $TailLines
    if ($exitCode -ne 0) {
        Write-Error "$Name failed with exit code $exitCode"
        exit $exitCode
    }
}

# PSoC 版本 = 编译时间戳(十进制 YYMMDDHHMM), 由 make build 的 PREBUILD 生成到 fw_build_stamp.h。
# 这里读生成头而不是 main.c: 它就是上一次 make build 真正编进 HEX 的那个值。
function Get-PsocStamp {
    if (-not (Test-Path -LiteralPath $psocStampHdr)) {
        Write-Error "PSoC build stamp header missing: $psocStampHdr (run 'build-psoc' first)"
        exit 1
    }
    $source = Get-Content -LiteralPath $psocStampHdr -Raw
    $m = [regex]::Match($source, '#define\s+FW_BUILD_STAMP\s+\((\d+)u\)')
    if (-not $m.Success) {
        Write-Error "Unable to parse FW_BUILD_STAMP from $psocStampHdr"
        exit 1
    }
    return [uint32]$m.Groups[1].Value
}

# 2607310427 -> "2026-07-31 04:27"
function Format-BuildStamp([uint32]$stamp) {
    $s = '{0:D10}' -f $stamp
    return '20{0}-{1}-{2} {3}:{4}' -f $s.Substring(0,2), $s.Substring(2,2), $s.Substring(4,2),
                                      $s.Substring(6,2), $s.Substring(8,2)
}

function Wait-Bootsel([int]$timeoutSec = 10) {
    for ($i = 0; $i -lt $timeoutSec; $i++) {
        if (Test-Path G:\) { return $true }
        Start-Sleep -Seconds 1
    }
    return (Test-Path G:\)
}

function Ensure-Selftest() {
    if (-not (Test-Path $selftest)) {
        Write-Output "selftest.exe 不存在, 先 build-ui"
        Invoke-NativeTail 'build-ui' $cs 'cargo' @('build', '--locked', '--bins') 20
    }
}

switch ($Action) {
    'build-rp' {
        Invoke-NativeTail 'build-rp' $fw 'pio' @('run') 14
    }
    'build-ui' {
        Invoke-NativeTail 'build-ui' $cs 'cargo' @('build', '--locked', '--bins') 20
    }
    'build-ui-release' {
        Invoke-NativeTail 'build-ui-release' $cs 'cargo' @('build', '--locked', '--release', '--bin', 'mai2control-ui') 20
    }
    'diagnose' {
        Ensure-Selftest
        Invoke-NativeTail 'diagnose' $root $selftest @('--diagnose') 200
    }
    'smoke' {
        Ensure-Selftest
        Invoke-NativeTail 'smoke' $root $selftest @('--smoke') 200
    }
    'full' {
        Ensure-Selftest
        Invoke-NativeTail 'full' $root $selftest @() 300
    }
    'algo' {
        # JIT 算法引擎闭环: 读信息→编译上传测试算法→校验→恢复默认→校验
        Ensure-Selftest
        Invoke-NativeTail 'algo' $root $selftest @('--algo') 60
    }
    'global' {
        # 全局 CSD 配置闭环: 读全部→设 High-Z→设备回读校验→恢复 GND
        Ensure-Selftest
        Invoke-NativeTail 'global' $root $selftest @('--global') 60
    }
    'kbd' {
        # 键盘闭环: 读物理键码/触控映射/物理实时态 + SET_MAP round-trip 校验
        Ensure-Selftest
        Invoke-NativeTail 'kbd' $root $selftest @('--kbd') 60
    }
    'enum' {
        # monitor device enumeration presence ~12s: intermittent=reboot loop, stable-but-nocomm=main loop hang
        Ensure-Selftest
        for ($i = 0; $i -lt 12; $i++) {
            $present = & $selftest --list-only 2>$null | Select-String 'FOUND|NONE'
            Write-Output "t=${i}s: $present"
            Start-Sleep -Seconds 1
        }
    }
    'connlog' {
        # single connect with debug log: shows clear_halt (enumerated) + whether HELLO gets a reply
        Ensure-Selftest
        $env:RUST_LOG = 'debug'
        & $selftest --idle-only --soak-idle 1 2>&1 |
            Select-String 'clear_halt|IO thread|Sent|Decoded|Received|read error|DEVICE_INFO|open|claim|disappeared|FAIL'
        $env:RUST_LOG = ''
    }
    'cfglog' {
        # connect + request CFG_GET_ALL with debug: shows whether the config response frame arrives + item count
        Ensure-Selftest
        $env:RUST_LOG = 'debug'
        & $selftest --cfg-only 2>&1 |
            Select-String 'Sent|Decoded|Received|read error|CFG|config_entries|断开|FAIL'
        $env:RUST_LOG = ''
    }
    'reset-config' {
        Ensure-Selftest
        & $selftest --reset-config 2>$null
    }
    'soak' {
        Ensure-Selftest
        & $selftest --soak --soak-idle $Version 2>$null
    }
    'probe' {
        # connection reliability probe: repeat connect + read DEVICE_INFO, count ok/fail. Version arg = tries (default 8)
        Ensure-Selftest
        $tries = 8
        if ($Version -match '^[0-9]+$') { $tries = [int]$Version }
        $ok = 0; $fail = 0
        for ($i = 1; $i -le $tries; $i++) {
            & $selftest --idle-only --soak-idle 1 > $null 2>&1
            if ($LASTEXITCODE -eq 0) { $ok++; Write-Output "attempt ${i}: OK" }
            else { $fail++; Write-Output "attempt ${i}: FAIL(exit=$LASTEXITCODE)" }
            Start-Sleep -Milliseconds 800
        }
        Write-Output "=== PROBE OK=$ok FAIL=$fail ==="
    }
    'csd-provision' {
        Ensure-Selftest
        & $selftest --csd-provision 2>$null
    }
    'csd-verify' {
        Ensure-Selftest
        & $selftest --csd-verify 2>$null
    }
    'bootsel' {
        Ensure-Selftest
        & $selftest --reboot-bootloader-only 2>$null
        if (Wait-Bootsel 10) { "G_EXISTS (BOOTSEL ready)" } else { "G_ABSENT (bootsel failed)" }
    }
    'flash-rp' {
        if (Test-Path G:\) {
            Copy-Item $uf2 'G:\' -Force
            "FLASHED firmware.uf2 -> G:"
        } else {
            "G_ABSENT: device not in BOOTSEL; run 'bootsel' first"
        }
    }
    'cycle' {
        Ensure-Selftest
        if (-not (Test-Path G:\)) {
            & $selftest --reboot-bootloader-only 2>$null
            Wait-Bootsel 10 | Out-Null
        }
        if (-not (Test-Path G:\)) { "CYCLE FAIL: no BOOTSEL"; break }
        Copy-Item $uf2 'G:\' -Force
        "FLASHED; waiting for reboot + PSoC bringup..."
        Start-Sleep -Seconds 9
        & $selftest --diagnose 2>$null
    }
    'build-psoc' {
        # bash -lc 登录 shell 会切到 $HOME, 必须在命令内显式 cd 回工程目录(cygdrive 路径), 否则
        # make 在 $HOME 找不到 Makefile 目标 → "no rule to make target build"。
        $bash = 'C:\Users\asdfg\ModusToolbox\tools_3.6\modus-shell\bin\bash.exe'
        $drive = $psoc.Substring(0,1).ToLower()
        $cyg = "/cygdrive/$drive" + ($psoc.Substring(2) -replace '\\','/')
        Invoke-NativeTail 'build-psoc' $psoc $bash @('-lc', "cd '$cyg' && make build -j8") 30
    }
    'embed-psoc' {
        $hexFiles = @(Get-ChildItem -LiteralPath $psocHexDir -Filter '*.hex' -File)
        if ($hexFiles.Count -ne 1) {
            Write-Error "Expected exactly one PSoC HEX in $psocHexDir, found $($hexFiles.Count)"
            exit 1
        }
        $stamp = Get-PsocStamp
        $sourceVersion = '0x{0:X8}' -f $stamp
        Write-Output ("PSoC build stamp: {0} ({1}) -> {2}" -f
            $stamp, (Format-BuildStamp $stamp), $sourceVersion)
        Invoke-NativeTail 'embed-psoc' $fw 'python' @(
            $psocConverter, $hexFiles[0].FullName, $psocImage, $sourceVersion
        ) 20
    }
    'imgsum' {
        $img = 'F:\mai2control\mai2control-v4\main_firmware\src\protocol\psoc\psoc_fw_image.h'
        $raw = Get-Content $img -Raw
        $m = [regex]::Matches($raw, '0x([0-9A-Fa-f]{2})\s*,')
        $b = New-Object System.Collections.Generic.List[byte]
        $sum = 0
        foreach ($x in $m) { $val = [Convert]::ToInt32($x.Groups[1].Value, 16); $sum += $val; $b.Add([byte]$val) }
        "image bytes={0} sum=0x{1:X8} sum28=0x{2:X8}" -f $m.Count, $sum, ($sum -band 0x0FFFFFFF)
        foreach ($off in @(0,4,0x80,0x84,0x100,0x6000,0x657C)) {
            if ($off + 3 -lt $b.Count) {
                $v = $b[$off] -bor ($b[$off+1] -shl 8) -bor ($b[$off+2] -shl 16) -bor ($b[$off+3] -shl 24)
                "img[0x{0:X}]=0x{1:X8}" -f $off, $v
            }
        }
    }
    'capinfo' {
        # dump CapSense runtime-tunable param accessor macros (RAM widget context) for feasibility recon
        $h = 'F:\mai2control\mai2control-v4\psoc_firmware\CY8C4147AZI-SensorCore\bsps\TARGET_APP_CY8CKIT-149\config\GeneratedSource\cycfg_capsense.h'
        Get-Content $h | Select-String 'BUTTON0_FINGER_TH_VALUE|BUTTON0_NOISE_TH_VALUE|BUTTON0_NEGATIVE_NOISE_TH_VALUE|BUTTON0_HYSTERESIS_VALUE|BUTTON0_ON_DEBOUNCE_VALUE|BUTTON0_RESOLUTION_VALUE|BUTTON0_SNS_CLK_VALUE|BUTTON0_IDAC_MOD0_VALUE|BUTTON0_SNS0_IDAC_COMP0_VALUE|ptrWdContext|ptrSnsContext' | Select-Object -First 24
    }
    'capstruct' {
        # dump widget context struct fields (runtime-tunable params) + tuner extern declaration
        $s = 'F:\mai2control\mai2control-v4\psoc_firmware\mtb_shared\capsense\release-v5.0.0\cy_capsense_structure.h'
        Get-Content $s | Select-String '\bfingerTh;|\bnoiseTh;|\bnNoiseTh;|\bhysteresis;|\bonDebounce;|\bresolution;|\bsnsClk;|\bidacMod\[|\blowBslnRst;|\bsigPFC;|\bmaxRawCount;' | Select-Object -First 16
        Write-Output '--- tuner extern in cycfg ---'
        $h = 'F:\mai2control\mai2control-v4\psoc_firmware\CY8C4147AZI-SensorCore\bsps\TARGET_APP_CY8CKIT-149\config\GeneratedSource\cycfg_capsense.h'
        Get-Content $h | Select-String 'cy_capsense_tuner;|extern.*cy_capsense_tuner|cy_stc_capsense_tuner_t' | Select-Object -First 4
    }
    'dbg' {
        Ensure-Selftest
        & $selftest --debug-read 2>$null
    }
    'ctrl-bootsel' {
        Ensure-Selftest
        & $selftest --ctrl-bootsel 2>$null
        if (Wait-Bootsel 10) { "G_EXISTS (BOOTSEL ready)" } else { "G_ABSENT (ctrl-bootsel failed)" }
    }
    'build-all' {
        # 全量构建: PSoC make + embed + RP2040 UF2 + Rust UI/selftest (调用固定的 build.ps1)
        & $buildAll
        if ($LASTEXITCODE -ne 0) { Write-Error "build-all failed (exit=$LASTEXITCODE)"; exit $LASTEXITCODE }
    }
    'reflash' {
        # 可靠刷写: 新鲜进 BOOTSEL -> 复制 UF2 -> 轮询确认离开 BOOTSEL(设备已接收并重启) -> 等 PSoC bringup -> 诊断
        Ensure-Selftest
        if (-not (Test-Path G:\)) {
            & $selftest --reboot-bootloader-only 2>$null
            Wait-Bootsel 10 | Out-Null
        }
        if (-not (Test-Path G:\)) { "REFLASH FAIL: no BOOTSEL"; break }
        Copy-Item $uf2 'G:\' -Force
        "COPIED firmware.uf2 -> G:; waiting for device to leave BOOTSEL..."
        $left = $false
        for ($i = 0; $i -lt 15; $i++) {
            Start-Sleep -Seconds 1
            if (-not (Test-Path G:\)) { $left = $true; "left BOOTSEL at t=$($i + 1)s (flash accepted)"; break }
        }
        if (-not $left) { "REFLASH WARN: still in BOOTSEL after 15s" }
        Start-Sleep -Seconds 10
        & $selftest --diagnose 2>$null
    }
    'build-blob' {
        # 固定编译默认 HDR 算法 blob: gcc -> objcopy binary -> nm/objdump 校验 -> 生成 C 数组头 + CRC16
        $gccDirs = @(
            'C:\Users\asdfg\ModusToolbox\tools_3.6\gcc\bin',
            'C:\Users\asdfg\.platformio\packages\toolchain-rp2040-earlephilhower\bin'
        )
        $gccBin = $null
        foreach ($d in $gccDirs) { if (Test-Path (Join-Path $d 'arm-none-eabi-gcc.exe')) { $gccBin = $d; break } }
        if (-not $gccBin) { Write-Error 'arm-none-eabi-gcc not found in known toolchain dirs'; exit 1 }
        if (-not (Test-Path $blobSrc)) { Write-Error "blob source missing: $blobSrc"; exit 1 }
        $gcc     = Join-Path $gccBin 'arm-none-eabi-gcc.exe'
        $objcopy = Join-Path $gccBin 'arm-none-eabi-objcopy.exe'
        $nm      = Join-Path $gccBin 'arm-none-eabi-nm.exe'
        $objdump = Join-Path $gccBin 'arm-none-eabi-objdump.exe'
        Write-Output "gcc: $gcc"
        & $gcc '-mcpu=cortex-m0plus' '-mthumb' '-Os' '-ffreestanding' '-fno-jump-tables' '-fomit-frame-pointer' '-fno-common' '-nostdlib' "-I$psoc" '-c' $blobSrc '-o' $blobObj
        if ($LASTEXITCODE -ne 0) { Write-Error 'blob compile failed'; exit 1 }
        & $objcopy '-O' 'binary' '-j' '.text' $blobObj $blobBin
        if ($LASTEXITCODE -ne 0) { Write-Error 'objcopy failed'; exit 1 }
        Write-Output '--- nm (must show NO undefined "U" symbol except algo defined "T") ---'
        & $nm $blobObj
        Write-Output '--- objdump -dr (must show NO R_ARM_* reloc in .text; algo at offset 0) ---'
        & $objdump '-dr' $blobObj
        $bytes = [System.IO.File]::ReadAllBytes($blobBin)
        $crc = Get-Crc16Ccitt $bytes
        Write-Output ("--- blob len={0} bytes crc16=0x{1:X4} (limit 1024) ---" -f $bytes.Length, $crc)
        if ($bytes.Length -gt 1024) { Write-Error "blob too large: $($bytes.Length) > 1024"; exit 1 }
        $sb = New-Object System.Text.StringBuilder
        [void]$sb.AppendLine('/* AUTO-GENERATED by dev.ps1 build-blob. Do not edit by hand. */')
        [void]$sb.AppendLine('#ifndef PSOC_ALGO_DEFAULT_H')
        [void]$sb.AppendLine('#define PSOC_ALGO_DEFAULT_H')
        [void]$sb.AppendLine('')
        [void]$sb.AppendLine('static const unsigned char PSOC_ALGO_DEFAULT[] = {')
        for ($i = 0; $i -lt $bytes.Length; $i += 12) {
            $line = '    '
            $end = [Math]::Min($i + 12, $bytes.Length)
            for ($j = $i; $j -lt $end; $j++) { $line += ('0x{0:X2}, ' -f $bytes[$j]) }
            [void]$sb.AppendLine($line.TrimEnd())
        }
        [void]$sb.AppendLine('};')
        [void]$sb.AppendLine(('static const unsigned int   PSOC_ALGO_DEFAULT_LEN   = {0}u;' -f $bytes.Length))
        [void]$sb.AppendLine(('static const unsigned short PSOC_ALGO_DEFAULT_CRC16 = 0x{0:X4}u;' -f $crc))
        [void]$sb.AppendLine('')
        [void]$sb.AppendLine('#endif /* PSOC_ALGO_DEFAULT_H */')
        $hdrDir = Split-Path $blobHdr
        if (-not (Test-Path $hdrDir)) { New-Item -ItemType Directory -Path $hdrDir -Force | Out-Null }
        Set-Content -LiteralPath $blobHdr -Value $sb.ToString() -Encoding ASCII
        "WROTE $blobHdr"
    }
    'status' {
        if (Test-Path G:\)      { "G: EXISTS (BOOTSEL)" } else { "G: ABSENT (app running or disconnected)" }
        if (Test-Path $uf2)     { "uf2 OK: $uf2" }        else { "uf2 MISSING" }
        if (Test-Path $selftest){ "selftest OK" }         else { "selftest MISSING (run build-ui)" }
    }
    default {
        "Unknown action: $Action"
        "Actions: build-rp build-ui build-all build-psoc embed-psoc build-blob diagnose smoke full bootsel flash-rp cycle reflash status"
    }
}
