# mai2control fixed autonomous build/flash/validation entry.
# Change only this constant between "Build" and "Flash"; invocation stays fixed.
$WorkflowMode = "Build"

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
Set-StrictMode -Version Latest

$FirmwareDir = $PSScriptRoot
$RootDir = Split-Path -Parent $FirmwareDir
$PsocDir = Join-Path $RootDir "psoc_firmware\CY8C4147AZI-SensorCore"
$ControlDir = Join-Path $RootDir "control_software"
$BuildDir = Join-Path $FirmwareDir ".pio\build\pico"
$RunId = "{0}-{1}" -f ([DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffZ")), ([Guid]::NewGuid().ToString("N").Substring(0, 8))
$LogDir = Join-Path $FirmwareDir ".workflow-logs"
$ReportPath = Join-Path $LogDir "workflow-$RunId.log"
$PnpReportPath = Join-Path $LogDir "pnp-$RunId.txt"
$PsocHexDir = Join-Path $PsocDir "build\last_config"
$PsocImageHeader = Join-Path $FirmwareDir "src\protocol\psoc\psoc_fw_image.h"
$Uf2Path = Join-Path $BuildDir "firmware.uf2"
$SelftestPath = Join-Path $ControlDir "target\debug\selftest.exe"
$MakeExe = Join-Path $env:USERPROFILE "ModusToolbox\tools_3.6\modus-shell\bin\make.exe"
$PlatformioExe = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
$PythonExe = (Get-Command python.exe -ErrorAction Stop).Source
$CargoExe = (Get-Command cargo.exe -ErrorAction Stop).Source
$PsocConverter = Join-Path $FirmwareDir "tools\psoc_hex_to_c.py"
$PsocMain = Join-Path $PsocDir "main.c"
$PsocStampHeader = Join-Path $PsocDir "fw_build_stamp.h"

function Assert-Path([string]$Path, [string]$Description) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Description not found: $Path"
    }
}

function Invoke-NativeStep(
    [string]$Name,
    [string]$FilePath,
    [string[]]$Arguments,
    [string]$WorkingDirectory
) {
    Write-Host "`n=== $Name ===" -ForegroundColor Cyan
    Write-Host "cwd: $WorkingDirectory"
    Write-Host "exe: $FilePath"
    Write-Host "args: $($Arguments -join ' ')"
    Push-Location -LiteralPath $WorkingDirectory
    try {
        & $FilePath @Arguments
        if ($LASTEXITCODE -ne 0) {
            throw "$Name failed with exit code $LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }
}

# PSoC 版本 = 编译时间戳(十进制 YYMMDDHHMM), 由 make build 的 PREBUILD 写入 fw_build_stamp.h;
# 读生成头而不是 main.c, 因为它就是刚编进 HEX 的那个值。
function Get-PsocStamp {
    Assert-Path $PsocStampHeader "PSoC build stamp header"
    $source = Get-Content -LiteralPath $PsocStampHeader -Raw
    $match = [regex]::Match($source, '#define\s+FW_BUILD_STAMP\s+\((\d+)u\)')
    if (-not $match.Success) {
        throw "Unable to parse FW_BUILD_STAMP from $PsocStampHeader"
    }
    return [uint32]$match.Groups[1].Value
}

# 2607310427 -> "2026-07-31 04:27"
function Format-BuildStamp([uint32]$Stamp) {
    $text = "{0:D10}" -f $Stamp
    return "20{0}-{1}-{2} {3}:{4}" -f $text.Substring(0, 2), $text.Substring(2, 2),
                                      $text.Substring(4, 2), $text.Substring(6, 2),
                                      $text.Substring(8, 2)
}

function Get-RpiBootVolume {
    $volumes = @(Get-CimInstance Win32_LogicalDisk | Where-Object {
        $_.DriveType -eq 2 -and $_.VolumeName -eq "RPI-RP2" -and $_.DeviceID
    })
    $safe = @($volumes | Where-Object {
        Test-Path -LiteralPath (Join-Path ($_.DeviceID + "\") "INFO_UF2.TXT")
    })
    if ($safe.Count -gt 1) {
        throw "Multiple RPI-RP2 UF2 volumes detected; refusing ambiguous write."
    }
    if ($safe.Count -eq 1) { return $safe[0] }
    return $null
}

function Wait-RpiBootVolumeStable([int]$TimeoutSeconds) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $stableDeviceId = $null
    $stableSamples = 0
    do {
        $volume = Get-RpiBootVolume
        if ($null -ne $volume) {
            $deviceId = [string]$volume.DeviceID
            if ($deviceId -eq $stableDeviceId) {
                $stableSamples++
            }
            else {
                $stableDeviceId = $deviceId
                $stableSamples = 1
            }
            if ($stableSamples -ge 4) { return $volume }
        }
        else {
            $stableDeviceId = $null
            $stableSamples = 0
        }
        Start-Sleep -Milliseconds 250
    } while ([DateTime]::UtcNow -lt $deadline)
    throw "Timed out waiting for a stable unique removable RPI-RP2 volume with INFO_UF2.TXT. Hold BOOTSEL while reconnecting, then rerun."
}

function Wait-RpiBootVolumeGone([string]$DeviceId, [int]$TimeoutSeconds) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    do {
        $volume = Get-RpiBootVolume
        if ($null -eq $volume) { return }
        if ([string]$volume.DeviceID -ne $DeviceId) {
            throw "RPI-RP2 identity changed from $DeviceId to $($volume.DeviceID); refusing ambiguous continuation."
        }
        Start-Sleep -Milliseconds 250
    } while ([DateTime]::UtcNow -lt $deadline)
    throw "RPI-RP2 volume $DeviceId did not disappear after UF2 copy; target did not leave BOOTSEL safely."
}

function Get-Mai2Topology {
    $devices = @(Get-CimInstance Win32_PnPEntity | Where-Object {
        $_.DeviceID -like "USB\VID_2E8A&PID_000A*"
    })
    $parent = @($devices | Where-Object {
        $_.DeviceID -notlike "*&MI_*" -and
        $_.Service -eq "usbccgp" -and
        $_.ConfigManagerErrorCode -eq 0 -and
        (@($_.HardwareID) -match 'REV_0401').Count -gt 0
    })
    $config = @($devices | Where-Object {
        $_.DeviceID -like "USB\VID_2E8A&PID_000A&MI_00*" -and
        $_.Service -eq "WINUSB" -and $_.ConfigManagerErrorCode -eq 0
    })
    $serial = @($devices | Where-Object {
        $_.DeviceID -like "USB\VID_2E8A&PID_000A&MI_01*" -and
        $_.Service -eq "usbser" -and $_.ConfigManagerErrorCode -eq 0
    })
    $light = @($devices | Where-Object {
        $_.DeviceID -like "USB\VID_2E8A&PID_000A&MI_03*" -and
        $_.Service -eq "usbser" -and $_.ConfigManagerErrorCode -eq 0
    })
    $ports = @(Get-CimInstance Win32_SerialPort | Where-Object {
        $_.PNPDeviceID -like "USB\VID_2E8A&PID_000A*" -and $_.Status -eq "OK"
    })
    [pscustomobject]@{
        Ready = ($parent.Count -eq 1 -and $config.Count -eq 1 -and
                 $serial.Count -eq 1 -and $light.Count -eq 1 -and $ports.Count -eq 2)
        Devices = $devices
        Ports = $ports
    }
}

function Wait-Mai2TopologyStable([int]$TimeoutSeconds) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $stableSamples = 0
    $last = $null
    do {
        $last = Get-Mai2Topology
        if ($last.Ready) {
            $stableSamples++
            if ($stableSamples -ge 4) { return $last }
        }
        else {
            $stableSamples = 0
        }
        Start-Sleep -Milliseconds 500
    } while ([DateTime]::UtcNow -lt $deadline)
    if ($null -eq $last) { $last = Get-Mai2Topology }
    $last.Devices | Format-List Name, Service, Status, ConfigManagerErrorCode, DeviceID, HardwareID | Out-String | Write-Host
    throw "Timed out waiting for REV_0401 topology stable for 2 seconds: usbccgp + MI_00 WINUSB + MI_01/MI_03 usbser + exactly two COM ports."
}

function Write-PnpReport($Topology) {
    "=== PNP DEVICES ===" | Set-Content -LiteralPath $PnpReportPath -Encoding UTF8
    $Topology.Devices |
        Select-Object Name, PNPClass, Service, Status, ConfigManagerErrorCode, DeviceID, HardwareID |
        Format-List | Out-String | Add-Content -LiteralPath $PnpReportPath -Encoding UTF8
    "=== SERIAL PORTS ===" | Add-Content -LiteralPath $PnpReportPath -Encoding UTF8
    $Topology.Ports |
        Select-Object Name, DeviceID, PNPDeviceID, Status |
        Format-List | Out-String | Add-Content -LiteralPath $PnpReportPath -Encoding UTF8
}

function Get-CdcPortName($Topology, [string]$InterfaceMarker) {
    $matches = @($Topology.Ports | Where-Object {
        $_.PNPDeviceID -like "*$InterfaceMarker*"
    })
    if ($matches.Count -ne 1) {
        throw "Expected exactly one CDC port for $InterfaceMarker, found $($matches.Count)."
    }
    return [string]$matches[0].DeviceID
}

function Read-SerialExact(
    [System.IO.Ports.SerialPort]$Port,
    [int]$Length,
    [int]$TimeoutMilliseconds
) {
    $buffer = [byte[]]::new($Length)
    $offset = 0
    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMilliseconds)
    while ($offset -lt $Length -and [DateTime]::UtcNow -lt $deadline) {
        try {
            $read = $Port.Read($buffer, $offset, $Length - $offset)
            if ($read -gt 0) { $offset += $read }
        }
        catch [System.TimeoutException] {
            # Continue until the overall deadline; ReadTimeout is intentionally short.
        }
    }
    if ($offset -ne $Length) {
        throw "Timed out reading $Length bytes from $($Port.PortName); received $offset."
    }
    return ,$buffer
}

function Invoke-CdcSmoke($Topology) {
    $serialName = Get-CdcPortName $Topology "&MI_01"
    $lightName = Get-CdcPortName $Topology "&MI_03"
    Write-Host "CDC mapping: serial=$serialName (MI_01), light=$lightName (MI_03)"

    $serialPort = [System.IO.Ports.SerialPort]::new($serialName, 115200, "None", 8, "One")
    $serialPort.ReadTimeout = 200
    $serialPort.WriteTimeout = 1000
    $serialPort.DtrEnable = $true
    try {
        $serialPort.Open()
        $serialPort.DiscardInBuffer()
        [byte[]]$stat = 0x7B, 0x52, 0x30, 0x41, 0x30, 0x7D  # {R0A0}
        $serialPort.Write($stat, 0, $stat.Length)
        $touch = Read-SerialExact $serialPort 9 2500
        if ($touch[0] -ne 0x28 -or $touch[8] -ne 0x29) {
            throw "Mai2Serial STAT reply stream has invalid framing: $([BitConverter]::ToString($touch))"
        }
        for ($index = 1; $index -le 7; $index++) {
            if ($touch[$index] -gt 0x1F) {
                throw "Mai2Serial touch payload is not 5-bit packed: $([BitConverter]::ToString($touch))"
            }
        }
        Write-Host "MAI2SERIAL CDC PASS: STAT started legal 9-byte touch stream $([BitConverter]::ToString($touch))"
    }
    finally {
        if ($serialPort.IsOpen) { $serialPort.Close() }
        $serialPort.Dispose()
    }

    $lightPort = [System.IO.Ports.SerialPort]::new($lightName, 115200, "None", 8, "One")
    $lightPort.ReadTimeout = 200
    $lightPort.WriteTimeout = 1000
    $lightPort.DtrEnable = $true
    try {
        $lightPort.Open()
        $lightPort.DiscardInBuffer()
        [byte[]]$versionRequest = 0xE0, 0x00, 0x00, 0x12, 0xF2
        $lightPort.Write($versionRequest, 0, $versionRequest.Length)
        $reply = Read-SerialExact $lightPort 8 2500
        $checksum = [byte]0
        for ($index = 0; $index -lt 7; $index++) { $checksum = $checksum -bxor $reply[$index] }
        if ($reply[0] -ne 0xE0 -or $reply[2] -ne 3 -or $reply[3] -ne 0x12 -or
            $reply[4] -ne 0 -or $reply[5] -ne 0 -or $reply[6] -ne 0x10 -or
            $reply[7] -ne $checksum) {
            throw "Mai2Light GET_PROTOCOL_VERSION reply invalid: $([BitConverter]::ToString($reply))"
        }
        Write-Host "MAI2LIGHT CDC PASS: protocol version 0x$('{0:X2}' -f $reply[6]), reply $([BitConverter]::ToString($reply))"
    }
    finally {
        if ($lightPort.IsOpen) { $lightPort.Close() }
        $lightPort.Dispose()
    }
}

Assert-Path $MakeExe "ModusToolbox make"
Assert-Path $PlatformioExe "PlatformIO"
Assert-Path $PsocDir "PSoC project"
Assert-Path $PsocConverter "PSoC HEX converter"
Assert-Path $PsocMain "PSoC main source"
Assert-Path $ControlDir "Rust control software"
New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

Start-Transcript -LiteralPath $ReportPath -Force | Out-Null
try {
    Write-Host "Workflow round: $RunId" -ForegroundColor Green
    Write-Host "Workflow log: $ReportPath"
    Write-Host "Workflow mode: $WorkflowMode" -ForegroundColor Green
    Write-Host "Safety: UF2 writes require one removable RPI-RP2 volume and INFO_UF2.TXT; no fixed drive letter is used."

    $oldPath = $env:PATH
    $env:PATH = (Split-Path -Parent $MakeExe) + ";" + $env:PATH
    try {
        Invoke-NativeStep "Build PSoC firmware" $MakeExe @("build", "-j8") $PsocDir
    }
    finally {
        $env:PATH = $oldPath
    }

    $hexFiles = @(Get-ChildItem -LiteralPath $PsocHexDir -Filter "*.hex" -File)
    if ($hexFiles.Count -ne 1) {
        throw "Expected exactly one PSoC HEX in $PsocHexDir, found $($hexFiles.Count)."
    }
    $psocStamp = Get-PsocStamp
    $psocVersion = "0x{0:X8}" -f $psocStamp
    Write-Host ("PSoC build stamp: {0} ({1}) -> {2}" -f
        $psocStamp, (Format-BuildStamp $psocStamp), $psocVersion)
    Invoke-NativeStep "Embed PSoC HEX ($psocVersion)" $PythonExe @(
        $PsocConverter, $hexFiles[0].FullName, $PsocImageHeader, $psocVersion
    ) $FirmwareDir

    Invoke-NativeStep "Build RP2040 firmware" $PlatformioExe @("run", "-e", "pico") $FirmwareDir
    Assert-Path $Uf2Path "RP2040 UF2"
    $uf2Item = Get-Item -LiteralPath $Uf2Path
    $uf2Hash = (Get-FileHash -LiteralPath $Uf2Path -Algorithm SHA256).Hash
    Write-Host ("RP2040 UF2 identity: SHA256={0} size={1} mtime_utc={2:o}" -f
        $uf2Hash, $uf2Item.Length, $uf2Item.LastWriteTimeUtc)

    Invoke-NativeStep "Build Rust UI and selftest" $CargoExe @(
        "build", "--locked", "--bins"
    ) $ControlDir
    Assert-Path $SelftestPath "WinUSB selftest executable"

    if ($WorkflowMode -eq "Probe") {
        $oldRustLog = $env:RUST_LOG
        $env:RUST_LOG = "mai2control_ui=debug,nusb=debug"
        try {
            & $SelftestPath --diagnose
            if ($LASTEXITCODE -ne 0) {
                throw "WinUSB diagnose failed with exit code $LASTEXITCODE"
            }
        }
        finally {
            $env:RUST_LOG = $oldRustLog
        }
        Write-Host "PROBE PASS: WinUSB HELLO/DEVICE_INFO round trip succeeded." -ForegroundColor Green
        return
    }
    if ($WorkflowMode -eq "Build") {
        Write-Host "BUILD PASS: PSoC HEX embedded, RP2040 UF2 built, WinUSB selftest compiled." -ForegroundColor Green
        return
    }
    if ($WorkflowMode -notin @("Build", "Flash", "Probe")) {
        throw "WorkflowMode must be Build, Probe, or Flash, got: $WorkflowMode"
    }

    $bootVolume = Get-RpiBootVolume
    if ($null -eq $bootVolume) {
        Write-Host "[$RunId] Requesting RP2040 BOOTSEL through mai2 config WinUSB..."
        & $SelftestPath --reboot-bootloader-only
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "Automatic BOOTSEL request failed; waiting for manual BOOTSEL recovery."
        }
    }
    $bootVolume = Wait-RpiBootVolumeStable 25

    $bootRoot = $bootVolume.DeviceID + "\"
    Write-Host "[$RunId] Verified stable UF2 target: $bootRoot (label RPI-RP2, removable, INFO_UF2.TXT present)"
    Copy-Item -LiteralPath $Uf2Path -Destination (Join-Path $bootRoot "firmware.uf2") -Force
    Write-Host "[$RunId] UF2 copied; waiting for RPI-RP2 to disappear before accepting application topology..."
    Wait-RpiBootVolumeGone ([string]$bootVolume.DeviceID) 25

    $topology = Wait-Mai2TopologyStable 45
    Write-PnpReport $topology

    Write-Host "Running WinUSB HELLO/DEVICE_INFO snapshot smoke test..."
    & $SelftestPath --smoke
    if ($LASTEXITCODE -ne 0) {
        throw "WinUSB smoke test failed with exit code $LASTEXITCODE"
    }

    Write-Host "Running MI_01/MI_03 CDC protocol smoke tests..."
    Invoke-CdcSmoke $topology

    Write-Host "FLASH PASS: UF2 programmed, REV_0401 topology, PSoC snapshot, WinUSB, Mai2Serial and Mai2Light smoke tests passed." -ForegroundColor Green
}
finally {
    Stop-Transcript | Out-Null
}
