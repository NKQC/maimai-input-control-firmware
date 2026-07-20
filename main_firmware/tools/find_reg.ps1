param([string]$Doc, [string[]]$Pat)
Select-String -Path $Doc -Pattern $Pat -SimpleMatch |
    Select-Object -First 60 |
    ForEach-Object { Write-Host ($_.LineNumber.ToString() + ': ' + $_.Line) }
