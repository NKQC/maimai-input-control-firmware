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
#   status      打印 G: 状态 + 关键产物是否存在

param(
    [Parameter(Position=0)]
    [string]$Action = 'status',
    [Parameter(Position=1)]
    [string]$Version = '0x00000401'
)

$ErrorActionPreference = 'Continue'
$root     = $PSScriptRoot
$fw       = Join-Path $root 'main_firmware'
$cs       = Join-Path $root 'control_software'
$uf2      = Join-Path $fw '.pio\build\pico\firmware.uf2'
$selftest = Join-Path $cs 'target\debug\selftest.exe'

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
        Push-Location $cs; cargo build --bins 2>&1 | Select-Object -Last 6; Pop-Location
    }
}

switch ($Action) {
    'build-rp' {
        Push-Location $fw
        pio run 2>&1 | Select-Object -Last 14
        Pop-Location
    }
    'build-ui' {
        Push-Location $cs
        cargo build --bins 2>&1 | Select-Object -Last 20
        Pop-Location
    }
    'build-ui-release' {
        Push-Location $cs
        cargo build --release --bin mai2control-ui 2>&1 | Select-Object -Last 20
        Pop-Location
    }
    'diagnose' {
        Ensure-Selftest
        & $selftest --diagnose 2>$null
    }
    'smoke' {
        Ensure-Selftest
        & $selftest --smoke 2>$null
    }
    'full' {
        Ensure-Selftest
        & $selftest 2>$null
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
        $bash = 'C:\Users\asdfg\ModusToolbox\tools_3.6\modus-shell\bin\bash.exe'
        & $bash -lc "cd /cygdrive/f/mai2control/mai2control-v4/psoc_firmware/CY8C4147AZI-SensorCore && make build -j8 2>&1 | tail -n 25"
    }
    'embed-psoc' {
        $hex = 'F:\mai2control\mai2control-v4\psoc_firmware\CY8C4147AZI-SensorCore\build\APP_CY8CKIT-149\Debug\mtb-example-psoc4-capsense-smartsense-buttons-slider.hex'
        $out = 'F:\mai2control\mai2control-v4\main_firmware\src\protocol\psoc\psoc_fw_image.h'
        python 'F:\mai2control\mai2control-v4\main_firmware\tools\psoc_hex_to_c.py' $hex $out $Version
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
    'status' {
        if (Test-Path G:\)      { "G: EXISTS (BOOTSEL)" } else { "G: ABSENT (app running or disconnected)" }
        if (Test-Path $uf2)     { "uf2 OK: $uf2" }        else { "uf2 MISSING" }
        if (Test-Path $selftest){ "selftest OK" }         else { "selftest MISSING (run build-ui)" }
    }
    default {
        "Unknown action: $Action"
        "Actions: build-rp build-ui diagnose smoke full bootsel flash-rp cycle status"
    }
}
