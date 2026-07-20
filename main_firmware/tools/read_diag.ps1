param(
    [string]$Com = 'COM3',
    [int]$WaitSeconds = 8,
    [int]$ReadSeconds = 12
)

Write-Host "waiting ${WaitSeconds}s for device to re-enumerate ..."
Start-Sleep -Seconds $WaitSeconds

$opened = $false
$sp = $null
for ($t = 0; $t -lt 30 -and -not $opened; $t++) {
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
    Write-Host "could not open $Com"
    exit 4
}

Write-Host "reading ${ReadSeconds}s from $Com ..."
$deadline = (Get-Date).AddSeconds($ReadSeconds)
while ((Get-Date) -lt $deadline) {
    try {
        $line = $sp.ReadLine()
        if ($line) { Write-Host $line }
    } catch {
        # timeout; keep polling
    }
}
$sp.Close()
Write-Host "[done]"
