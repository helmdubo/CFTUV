param([string]$Case = "building", [int]$Workers = 0, [string]$Tag = "r1", [string]$Tree = "D:/CFTUV-r3plus", [string]$Site = "D:/r3plus-scratch/base-site",
      [string]$Build = "a14d5084e0363e9a3f8693e95e5b1ac48c98045f77d242bae25ff86b52db60d7", [int]$Pstats = 1, [int]$Laps = 1, [string]$Hook = "")
$blender = "C:/Program Files/Blender Foundation/Blender 4.5/blender.exe"
$scenes = @{ "building" = @("C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend", "750BAEFC");
             "cover008" = @("C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend", "A044B5AA") }
$sp = "D:/r3plus-scratch/prof"
$scene = $scenes[$Case][0]
$h0 = (Get-FileHash -Algorithm SHA256 $scene).Hash
if (-not $h0.StartsWith($scenes[$Case][1])) { "SCENE SHA MISMATCH $h0"; exit 3 }
if ((Get-PSDrive E).Free / 1GB -lt 1.5) { "E LOW"; exit 3 }
$name = "${Case}_w${Workers}_${Tag}"
Get-ChildItem env: | Where-Object { $_.Name -like 'CFTUV_*' } | ForEach-Object { Remove-Item "env:$($_.Name)" }
$env:PYTHONSAFEPATH = "1"; $env:PYTHONDONTWRITEBYTECODE = "1"; $env:TMP = "D:/r3plus-scratch/tmp"; $env:TEMP = "D:/r3plus-scratch/tmp"
$pyargs = @("-b", "`"$scene`"", "--python-exit-code", "1")
if ($Hook) { $pyargs += @("--python", $Hook) }
$pyargs += @("--python", "$sp/prof_runner.py", "--", "--tree", $Tree, "--native-site", $Site, "--build-id", $Build, "--case", $Case, "--workers", $Workers, "--laps", $Laps, "--output", "$sp/out/$name.json")
if ($Pstats) { $pyargs += @("--pstats", "$sp/out/$name.pstats") }
New-Item -ItemType Directory -Force "$sp/out" | Out-Null
$sw = [Diagnostics.Stopwatch]::StartNew()
$proc = Start-Process -FilePath $blender -ArgumentList $pyargs -PassThru -RedirectStandardOutput "$sp/out/$name.log" -RedirectStandardError "$sp/out/$name.err" -WindowStyle Hidden
Add-Content "$sp/pids.log" ("{0} {1} pid={2}" -f (Get-Date -Format s), $name, $proc.Id)
$null = $proc.Handle
$proc.WaitForExit()
$h1 = (Get-FileHash -Algorithm SHA256 $scene).Hash
"{0} exit={1} elapsed={2:N1} sceneSame={3}" -f $name, $proc.ExitCode, $sw.Elapsed.TotalSeconds, ($h0 -eq $h1)
