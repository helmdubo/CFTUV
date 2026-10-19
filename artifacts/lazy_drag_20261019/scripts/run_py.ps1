param([string]$Script, [string]$Name, [string]$ArgsLine, [string]$Case = "cover008")
# Headless Blender on a frozen scene (hash checked before/after), no press, never saves; records only its own PID.
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
$pyargs = @("-b", "`"$scene`"", "--python-exit-code", "1", "--python", "$sp/$Script", "--", "--root", "D:/CFTUV-lazydrag", "--native-path", "$sp/site") + $ArgsLine.Split(' ')
$proc = Start-Process -FilePath $blender -ArgumentList $pyargs -PassThru -RedirectStandardOutput "$sp/out/$Name.log" -RedirectStandardError "$sp/out/$Name.err" -WindowStyle Hidden
Add-Content "$sp/pids.log" ("{0} {1} pid={2}" -f (Get-Date -Format s), $Name, $proc.Id)
$null = $proc.Handle
$proc.WaitForExit()
$h1 = (Get-FileHash -Algorithm SHA256 $scene).Hash
"{0} exit={1} sceneSame={2}" -f $Name, $proc.ExitCode, ($h0 -eq $h1)
Select-String -Path "$sp/out/$Name.log" -Pattern "PROBE|Traceback|Error" | Select-Object -Last 15 | ForEach-Object { $_.Line }
