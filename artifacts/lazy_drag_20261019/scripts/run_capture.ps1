param([string]$Case = "cover008", [string]$Extra = "--all")
$blender = "C:/Program Files/Blender Foundation/Blender 4.5/blender.exe"
$scenes = @{ "building" = @("C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend", "750BAEFC");
             "cover008" = @("C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend", "A044B5AA") }
$sp = "D:/lazydrag-scratch"
$scene = $scenes[$Case][0]
$h0 = (Get-FileHash -Algorithm SHA256 $scene).Hash
if (-not $h0.StartsWith($scenes[$Case][1])) { "SCENE SHA MISMATCH $h0"; exit 3 }
foreach ($d in 'C','D','E') { if ((Get-PSDrive $d).Free / 1GB -lt 1.5) { "$d LOW"; exit 3 } }
Get-ChildItem env: | Where-Object { $_.Name -like 'CFTUV_*' } | ForEach-Object { Remove-Item "env:$($_.Name)" }
$env:PYTHONSAFEPATH = "1"; $env:PYTHONDONTWRITEBYTECODE = "1"; $env:TMP = "$sp/tmp"; $env:TEMP = "$sp/tmp"
New-Item -ItemType Directory -Force "$sp/out", "$sp/tmp" | Out-Null
$pyargs = @("-b", "`"$scene`"", "--python-exit-code", "1", "--python", "$sp/capture_focus.py", "--", "--root", "D:/CFTUV-lazydrag", "--native-path", "$sp/site", "--case", $Case, "--out", "$sp/cap_$Case") + $Extra.Split(' ')
$sw = [Diagnostics.Stopwatch]::StartNew()
$proc = Start-Process -FilePath $blender -ArgumentList $pyargs -PassThru -RedirectStandardOutput "$sp/out/cap_$Case.log" -RedirectStandardError "$sp/out/cap_$Case.err" -WindowStyle Hidden
Add-Content "$sp/pids.log" ("{0} cap_{1} pid={2}" -f (Get-Date -Format s), $Case, $proc.Id)
$null = $proc.Handle
$proc.WaitForExit()
$h1 = (Get-FileHash -Algorithm SHA256 $scene).Hash
"{0} exit={1} elapsed={2:N1} sceneSame={3}" -f $Case, $proc.ExitCode, $sw.Elapsed.TotalSeconds, ($h0 -eq $h1)
