param([string]$Native, [string]$Out, [string]$Target, [switch]$Fast)
# Builds the cftuv_native wheel from $Native (a native/ directory) into $Out/site. All scratch on D:.
$ErrorActionPreference = 'Continue'
$BP = 'C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe'
$TC = 'C:/Users/helmd/.rustup/toolchains/stable-x86_64-pc-windows-msvc/bin'
$MATURIN = 'D:/CFTUV-export-20261015-evidence/angular/tool-bin/maturin.exe'
New-Item -ItemType Directory -Force -Path "$Out/wheels","$Out/tmp" | Out-Null
Get-ChildItem env: | Where-Object { $_.Name -like 'CFTUV_*' } | ForEach-Object { Remove-Item "env:$($_.Name)" }
$env:CARGO_TARGET_DIR = $Target
$env:TEMP = "$Out/tmp"; $env:TMP = "$Out/tmp"; $env:PYTHONDONTWRITEBYTECODE = '1'; $env:PYO3_PYTHON = $BP
$env:RUSTC = "$TC/rustc.exe"; $env:CARGO = "$TC/cargo.exe"; $env:CARGO_BUILD_JOBS = '6'; $env:PATH = "$TC;$env:PATH"
$env:PIP_DISABLE_PIP_VERSION_CHECK = '1'; $env:PIP_NO_INDEX = '1'
if ($Fast) {
  # Same sources (the build id is a digest of sources, not of the binary); only the optimiser settings differ for mutant turnaround.
  $env:CARGO_PROFILE_RELEASE_LTO = 'false'; $env:CARGO_PROFILE_RELEASE_CODEGEN_UNITS = '16'; $env:CARGO_PROFILE_RELEASE_INCREMENTAL = 'true'
}
Push-Location "$Native/cftuv-python"
& $MATURIN build --release --locked --offline --interpreter $BP --out "$Out/wheels"
if ($LASTEXITCODE -ne 0) { Pop-Location; throw "maturin $LASTEXITCODE" }
Pop-Location
$whl = Get-ChildItem "$Out/wheels/*.whl" | Select-Object -First 1
$env:PYTHONPATH = 'C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages'
& $BP -B -m pip install --no-deps --no-index --no-cache-dir --target "$Out/site" $whl.FullName | Out-Null
if ($LASTEXITCODE -ne 0) { throw "pip $LASTEXITCODE" }
$env:PYTHONPATH = "$Out/site"
& $BP -B -c "import cftuv_native as n; print(n.__file__); print(n.native_build_id())"
(Get-FileHash $whl.FullName -Algorithm SHA256).Hash
