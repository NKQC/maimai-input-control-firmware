Write-Host 'PORTS:'
[System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object -Unique | ForEach-Object { Write-Host "  $_" }

Write-Host 'RPI-RP2 VOLUME:'
$v = Get-Volume -ErrorAction SilentlyContinue | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
if ($v) {
    $v | ForEach-Object { Write-Host "  drive $($_.DriveLetter): label=$($_.FileSystemLabel)" }
} else {
    Write-Host '  (none)'
}

Write-Host 'INFO_UF2.TXT scan:'
foreach ($d in @('D','E','F','G','H','I','J','K')) {
    if (Test-Path "${d}:\INFO_UF2.TXT") { Write-Host "  ${d}: has INFO_UF2.TXT" }
}
Write-Host '[done]'
