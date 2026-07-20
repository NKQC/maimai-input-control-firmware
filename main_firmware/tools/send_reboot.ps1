param(
    [string]$Com = 'COM3',
    [byte]$Cmd = 0x05,      # REBOOT_BOOTLOADER
    [int]$ListenSeconds = 3
)

function Get-Crc16Ccitt([byte[]]$data) {
    [uint16]$crc = 0xFFFF
    foreach ($b in $data) {
        $crc = $crc -bxor ([uint16]($b) -shl 8)
        for ($i = 0; $i -lt 8; $i++) {
            if ($crc -band 0x8000) {
                $crc = (($crc -shl 1) -bxor 0x1021) -band 0xFFFF
            } else {
                $crc = ($crc -shl 1) -band 0xFFFF
            }
        }
    }
    return [uint16]$crc
}

# body = cmd flags seq len_lo len_hi (no payload)
$body = [byte[]]($Cmd, 0x00, 0x00, 0x00, 0x00)
$crc = Get-Crc16Ccitt $body
$crcLo = [byte]($crc -band 0xFF)
$crcHi = [byte](($crc -shr 8) -band 0xFF)
$frame = [byte[]](0xAA, 0x55) + $body + [byte[]]($crcLo, $crcHi)

$hex = ($frame | ForEach-Object { $_.ToString('X2') }) -join ' '
Write-Host "frame bytes: $hex   (crc=0x$($crc.ToString('X4')))"

try {
    $p = New-Object System.IO.Ports.SerialPort($Com, 115200)
    $p.DtrEnable = $true
    $p.ReadTimeout = 500
    $p.Open()
    $p.Write($frame, 0, $frame.Length)
    Write-Host "sent. listening ${ListenSeconds}s for response/diag ..."
    $deadline = (Get-Date).AddSeconds($ListenSeconds)
    while ((Get-Date) -lt $deadline) {
        try {
            $n = $p.BytesToRead
            if ($n -gt 0) {
                $buf = New-Object byte[] $n
                $r = $p.Read($buf, 0, $n)
                $rx = ($buf[0..($r-1)] | ForEach-Object { $_.ToString('X2') }) -join ' '
                Write-Host "RX: $rx"
            } else {
                Start-Sleep -Milliseconds 100
            }
        } catch {
            Start-Sleep -Milliseconds 100
        }
    }
    $p.Close()
} catch {
    Write-Host "error: $($_.Exception.Message)"
}
Write-Host "[done]"
