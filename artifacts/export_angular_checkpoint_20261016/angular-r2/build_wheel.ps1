param([string]$Out)
$ErrorActionPreference = 'Stop'
$A = 'D:/CFTUV-export-20261015-evidence/angular'
$BP = 'C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe'
$TC = 'C:/Users/helmd/.rustup/toolchains/stable-x86_64-pc-windows-msvc/bin'
New-Item -ItemType Directory -Path "$Out/wheels","$Out/tmp" | Out-Null
Get-ChildItem env: | Where-Object { $_.Name -like 'CFTUV_*' } | ForEach-Object { Remove-Item "env:$($_.Name)" }
$env:CARGO_TARGET_DIR = 'C:/Users/helmd/AppData/Local/Temp/cftuv-r3-density-target'
$env:TEMP = "$Out/tmp"; $env:TMP = "$Out/tmp"; $env:PYTHONDONTWRITEBYTECODE = '1'; $env:PYO3_PYTHON = $BP
$env:RUSTC = "$TC/rustc.exe"; $env:CARGO = "$TC/cargo.exe"; $env:CARGO_BUILD_JOBS = '4'; $env:PATH = "$TC;$env:PATH"
$env:PIP_DISABLE_PIP_VERSION_CHECK = '1'; $env:PIP_NO_INDEX = '1'
Push-Location D:/CFTUV-export-angular-20261016/native/cftuv-python
& "$A/tool-bin/maturin.exe" build --release --locked --offline --interpreter $BP --out "$Out/wheels"
if ($LASTEXITCODE -ne 0) { throw "maturin $LASTEXITCODE" }
Pop-Location
$whl = Get-ChildItem "$Out/wheels/*.whl"
$env:PYTHONPATH = 'C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages'
& $BP -B -m pip install --no-deps --no-index --no-cache-dir --target "$Out/site" $whl.FullName
if ($LASTEXITCODE -ne 0) { throw "pip $LASTEXITCODE" }
$env:PYTHONPATH = "$Out/site;D:/CFTUV-export-angular-20261016/kernel/src;D:/CFTUV-export-angular-20261016/tools;C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages"
Push-Location D:/CFTUV-export-angular-20261016
& $BP -B -c "import cftuv_native as n; print(n.__file__); print(n.native_build_id()); print(n.native_status())"
& $BP -B tools/native_build_id.py --json --require
Pop-Location
(Get-FileHash $whl.FullName -Algorithm SHA256).Hash
