for ($i = 0; $i -lt 60; $i++) {
    $v = Get-Volume -ErrorAction SilentlyContinue | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
    if ($v) {
        $d = ($v | Select-Object -First 1).DriveLetter
        if ($d) { Write-Host "FOUND ${d}: after $i s"; exit 0 }
    }
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($i % 5 -eq 0) { Write-Host "t=${i}s ports=$($ports -join ',')" }
    Start-Sleep -Seconds 1
}
Write-Host "drive NOT found in 60s"
exit 1
