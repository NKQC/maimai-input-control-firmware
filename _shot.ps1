param([int]$Tab = 3, [string]$Out = "shot.png", [int]$Wait = 9, [int]$Wheel = 0, [int]$WheelX = 900, [int]$WheelY = 700)
Add-Type -AssemblyName System.Windows.Forms, System.Drawing
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class W {
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out R r);
  [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
  [DllImport("user32.dll")] public static extern void mouse_event(uint f, int dx, int dy, int d, IntPtr e);
  public struct R { public int L, T, Rt, B; }
  public static void Wheel(int n) { for (int i = 0; i < Math.Abs(n); i++) { mouse_event(0x0800, 0, 0, n > 0 ? -120 : 120, IntPtr.Zero); System.Threading.Thread.Sleep(60); } }
}
"@
Get-Process explorer -ErrorAction SilentlyContinue | ForEach-Object {
  if ($_.MainWindowHandle -ne 0) { [void][W]::ShowWindow($_.MainWindowHandle, 6) }
}
Stop-Process -Name mai2control-ui -Force -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 500
$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = (Resolve-Path ".\control_software\target\release\mai2control-ui.exe").Path
$psi.WorkingDirectory = (Resolve-Path ".\control_software\target\release").Path
$psi.UseShellExecute = $false
$psi.EnvironmentVariables["MAI2_UI_VIEW"] = "1"
$psi.EnvironmentVariables["MAI2_UI_TAB"] = "$Tab"
$psi.EnvironmentVariables["RUST_LOG"] = "off"
$p = [System.Diagnostics.Process]::Start($psi)
Start-Sleep -Seconds $Wait
$p.Refresh()
$h = $p.MainWindowHandle
[void][W]::ShowWindow($h, 3)
Start-Sleep -Milliseconds 700
[void][W]::SetForegroundWindow($h)
Start-Sleep -Milliseconds 1200
if ($Wheel -ne 0) {
  [void][W]::SetCursorPos($WheelX, $WheelY)
  Start-Sleep -Milliseconds 300
  [W]::Wheel($Wheel)
  Start-Sleep -Milliseconds 900
}
$r = New-Object 'W+R'
[void][W]::GetWindowRect($h, [ref]$r)
$w = $r.Rt - $r.L; $ht = $r.B - $r.T
if ($w -le 0 -or $ht -le 0) { $b = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds; $r.L = 0; $r.T = 0; $w = $b.Width; $ht = $b.Height }
$bmp = New-Object System.Drawing.Bitmap $w, $ht
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen((New-Object System.Drawing.Point($r.L, $r.T)), [System.Drawing.Point]::Empty, (New-Object System.Drawing.Size($w, $ht)))
$bmp.Save((Join-Path (Get-Location) $Out), [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue
Write-Output ("saved " + $Out + " " + $w + "x" + $ht)
