$root = "C:\Users\asdfg\.cargo\registry\src\index.crates.io-1949cf8c6b5b557f"
$hits = Get-ChildItem -Path $root -Recurse -Filter "*.rs" -ErrorAction SilentlyContinue |
    Select-String -Pattern "enum PointerEventKind|enum InputType"
foreach ($h in $hits) { Write-Output ($h.Path + " : " + $h.LineNumber) }
$f = Join-Path $root "i-slint-common-1.17.1\src\enums.rs"
if (Test-Path $f) {
    $txt = Get-Content $f
    for ($i = 0; $i -lt $txt.Count; $i++) {
        if ($txt[$i] -match "PointerEventKind|InputType") {
            Write-Output ("L" + ($i + 1) + ": " + $txt[$i])
            for ($j = 1; $j -le 8; $j++) { Write-Output ("    " + $txt[$i + $j]) }
        }
    }
}
