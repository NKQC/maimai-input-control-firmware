param(
    [string]$Com = 'COM3',
    [string]$Uf2 = 'F:\mai2control\mai2control-v4\main_firmware\.pio\build\pico\firmware.uf2',
    [int]$ReadSeconds = 25
)

$ErrorActionPreference = 'Continue'

Write-Host "[1] Sending REBOOT_BOOTLOADER frame to $Com ..."
try {
    $p = New-Object System.IO.Ports.SerialPort($Com, 115200)
    $p.DtrEnable = $true
    $p.Open()
    $b = [byte[]](0xAA,0x55,0x05,0x00,0x00,0x00,0x00,0x5B,0x32)
    $p.Write($b, 0, $b.Length)
    Start-Sleep -Milliseconds 200
    $p.Close()
    Write-Host "    frame sent."
} catch {
    Write-Host "    reboot-frame send failed: $($_.Exception.Message)"
}

Write-Host "[2] Waiting for RPI-RP2 mass-storage drive ..."
$drive = $null
for ($i = 0; $i -lt 90; $i++) {
    Start-Sleep -Seconds 1
    $vols = Get-Volume -ErrorAction SilentlyContinue | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
    if ($vols) {
        $drive = ($vols | Select-Object -First 1).DriveLetter
        if ($drive) { break }
    }
    # fallback: scan for INFO_UF2.TXT on removable letters
    foreach ($d in @('D','E','F','G','H','I','J','K')) {
        if (Test-Path "${d}:\INFO_UF2.TXT") { $drive = $d; break }
    }
    if ($drive) { break }
}

if (-not $drive) {
    Write-Host "    RPI-RP2 drive not found within 40s. Aborting."
    exit 2
}
Write-Host "    found bootloader drive: ${drive}:"

Write-Host "[3] Copying UF2 ..."
if (-not (Test-Path $Uf2)) {
    Write-Host "    UF2 not found: $Uf2"
    exit 3
}
try {
    Copy-Item $Uf2 "${drive}:\firmware.uf2" -Force
    Write-Host "    copied. Device will reboot into new firmware."
} catch {
    Write-Host "    copy reported: $($_.Exception.Message) (RP2040 often resets mid-copy, usually OK)"
}

Write-Host "[4] Waiting for device to re-enumerate ..."
Start-Sleep -Seconds 8

Write-Host "[5] Reading diagnostics from $Com for $ReadSeconds s ..."
$deadline = (Get-Date).AddSeconds($ReadSeconds)
$opened = $false
for ($t = 0; $t -lt 20 -and -not $opened; $t++) {
    try {
        $sp = New-Object System.IO.Ports.SerialPort($Com, 115200)
        $sp.ReadTimeout = 1500
        $sp.Open()
        $opened = $true
    } catch {
        Start-Sleep -Milliseconds 500
    }
}
if (-not $opened) {
    Write-Host "    could not open $Com to read diagnostics."
    exit 4
}
while ((Get-Date) -lt $deadline) {
    try {
        $line = $sp.ReadLine()
        if ($line) { Write-Host $line }
    } catch {
        # read timeout, keep trying until deadline
    }
}
$sp.Close()
Write-Host "[done]"
