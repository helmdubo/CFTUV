param([string]$Case = "building", [string]$Tag = "r1", [string]$Script = "dragprof_runner.py", [string]$Extra = "", [int]$Workers = 12)
# One headless Blender run of a profile script on a frozen scene; scene SHA checked before/after; only the PID started here is recorded.
$blender = "C:/Program Files/Blender Foundation/Blender 4.5/blender.exe"
$scenes = @{ "building" = @("C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend", "750BAEFC");
             "cover008" = @("C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend", "A044B5AA") }
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$sp = "D:/dragprof-scratch"
$scene = $scenes[$Case][0]
$h0 = (Get-FileHash -Algorithm SHA256 $scene).Hash
if (-not $h0.StartsWith($scenes[$Case][1])) { "SCENE SHA MISMATCH $h0"; exit 3 }
foreach ($d in 'D','E') { if ((Get-PSDrive $d).Free / 1GB -lt 1.5) { "$d LOW"; exit 3 } }
$name = "${Case}_${Tag}"
Get-ChildItem env: | Where-Object { $_.Name -like 'CFTUV_*' } | ForEach-Object { Remove-Item "env:$($_.Name)" }
$env:PYTHONSAFEPATH = "1"; $env:PYTHONDONTWRITEBYTECODE = "1"; $env:TMP = "$sp/tmp"; $env:TEMP = "$sp/tmp"
New-Item -ItemType Directory -Force "$sp/out", "$sp/tmp" | Out-Null
$pyargs = @("-b", "`"$scene`"", "--python-exit-code", "1", "--python", "$here/$Script", "--", "--root", "D:/CFTUV-dragprof", "--native-path", "$sp/site", "--case", $Case, "--workers", $Workers, "--out", "$sp/out/$name.json")
if ($Extra) { $pyargs += $Extra.Split(' ') }
$load0 = (Get-CimInstance Win32_Processor | Measure-Object LoadPercentage -Average).Average
$sw = [Diagnostics.Stopwatch]::StartNew()
$proc = Start-Process -FilePath $blender -ArgumentList $pyargs -PassThru -RedirectStandardOutput "$sp/out/$name.log" -RedirectStandardError "$sp/out/$name.err" -WindowStyle Hidden
Add-Content "$sp/pids.log" ("{0} {1} pid={2}" -f (Get-Date -Format s), $name, $proc.Id)
$null = $proc.Handle
$proc.WaitForExit()
$h1 = (Get-FileHash -Algorithm SHA256 $scene).Hash
"{0} exit={1} elapsed={2:N1} sceneSame={3} load_before={4}" -f $name, $proc.ExitCode, $sw.Elapsed.TotalSeconds, ($h0 -eq $h1), $load0
