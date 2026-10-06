# Установить колесо нативного ядра `cftuv_native` в Blender и ДОКАЗАТЬ, что оно импортируется.
#
# Колесо ставится туда же, где лежат ядро `cftuv_envelope`, `sympy` и `mpmath`: в пользовательский `scripts\modules` версии
# Blender (без администратора). Это и есть «интерпретатор воркеров»:
#   * воркеры на встроенном Python Blender (умолчание) наследуют `sys.path` родителя, то есть видят этот каталог сами;
#   * внешний Python (настройка «Worker Python») стартует с `-I -S`, и каталоги пакетов ему передаёт РОДИТЕЛЬ
#     (`envelope_domain_pool.OPTIONAL_HOST_PACKAGES`), поэтому ставить колесо в его site-packages бессмысленно.
# Параметр -WorkerPython лишь ПРОВЕРЯЕТ, что именно такой воркер (-I -S, каталог от родителя) колесо импортирует.
#
# Идемпотентно: то же колесо повторно — «уже установлено» и проверка, ничего не меняется. Занятые файлы (Blender с подгруженным
# `_core.pyd`, воркеры пула) НАЗЫВАЮТСЯ до любой правки, и установка отказывается, ничего не тронув. Установка — подмена каталога
# (`install_common.ps1`): сбой или непроходящая проверка возвращают прежнюю версию.
#
# Запуск:
#   powershell -ExecutionPolicy Bypass -File tools\install_native_to_blender.ps1 -Wheel C:\path\cftuv_native-0.1.0-cp311-abi3-win_amd64.whl
#   ... -BlenderVersion 4.5          # если версий несколько
#   ... -WorkerPython C:\Python313\python.exe   # проверить и внешний Python воркеров
#   ... -WhatIf                      # показать, что будет сделано, и выйти (ничего не меняется)
#   ... -ModulesDir D:\x\modules -PythonExe D:\py\python.exe   # другой каталог и другой Python (проверка самого установщика)

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Wheel,
    [string]$BlenderVersion = "",
    [string]$WorkerPython = "",
    [string]$ModulesDir = "",
    [string]$PythonExe = "",
    [switch]$WhatIf
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "install_common.ps1")
Add-Type -AssemblyName System.IO.Compression.FileSystem

function Fail($message) {
    Write-Host ""
    Write-Host "  NOT INSTALLED: $message" -ForegroundColor Red
    exit 1
}

function Invoke-Python([string]$Exe, [string[]]$Arguments) {
    # Запуск Python с раздельным чтением stdout и stderr; сбой запуска - тоже результат, а не исключение.
    $info = New-Object System.Diagnostics.ProcessStartInfo
    $info.FileName = $Exe
    $info.Arguments = ($Arguments | ForEach-Object { '"' + $_ + '"' }) -join ' '
    $info.UseShellExecute = $false
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    $info.CreateNoWindow = $true
    try {
        $process = [System.Diagnostics.Process]::Start($info)
    } catch {
        return @{ ExitCode = -1; Out = ""; Err = $_.Exception.Message }
    }
    $out = $process.StandardOutput.ReadToEnd()
    $err = $process.StandardError.ReadToEnd()
    $process.WaitForExit()
    return @{ ExitCode = $process.ExitCode; Out = $out.Trim(); Err = $err.Trim() }
}

function Read-WheelText($archive, [string]$Name) {
    $entry = $archive.GetEntry($Name)
    if ($entry -eq $null) { return "" }
    $reader = New-Object System.IO.StreamReader($entry.Open())
    try { return $reader.ReadToEnd() } finally { $reader.Close() }
}

# --- колесо ------------------------------------------------------------------
if (-not (Test-Path -LiteralPath $Wheel -PathType Leaf)) { Fail "the wheel is not a file: $Wheel" }
if ([System.IO.Path]::GetExtension($Wheel).ToLower() -ne ".whl") { Fail "not a wheel (.whl expected): $Wheel" }
$Wheel = (Resolve-Path -LiteralPath $Wheel).Path
try {
    $archive = [System.IO.Compression.ZipFile]::OpenRead($Wheel)
} catch {
    Fail "the wheel is not a readable zip archive: $($_.Exception.Message)"
}
try {
    $names = @($archive.Entries | ForEach-Object { $_.FullName })
    $metaName = $names | Where-Object { $_ -match '^cftuv_native-[^/]+\.dist-info/METADATA$' } | Select-Object -First 1
    if (-not $metaName) { Fail "the wheel has no cftuv_native-*.dist-info/METADATA (is it a cftuv_native wheel)" }
    $distInfo = $metaName.Substring(0, $metaName.IndexOf('/'))
    foreach ($name in $names) {
        if (-not ($name.StartsWith("cftuv_native/") -or $name.StartsWith($distInfo + "/")) -or $name.Contains("..")) {
            Fail "unexpected path in the wheel (only cftuv_native/ and $distInfo/ are installed): $name"
        }
    }
    if (-not ($names -contains "cftuv_native/__init__.py")) { Fail "the wheel has no cftuv_native/__init__.py" }
    if (-not ($names | Where-Object { $_ -match '^cftuv_native/_core[^/]*\.pyd$' })) { Fail "the wheel has no compiled cftuv_native/_core*.pyd" }
    $meta = Read-WheelText $archive $metaName
    $wheelInfo = Read-WheelText $archive ($distInfo + "/WHEEL")
} finally {
    $archive.Dispose()
}
$wheelVersion = if ($meta -match '(?m)^Version:\s*(\S+)') { $Matches[1] } else { "unknown" }
$requires = if ($meta -match '(?m)^Requires-Python:\s*>=\s*(\d+)\.(\d+)') { @([int]$Matches[1], [int]$Matches[2]) } else { $null }
$tags = @([regex]::Matches($wheelInfo, '(?m)^Tag:\s*(\S+)') | ForEach-Object { $_.Groups[1].Value })

Write-Host ""
Write-Host "=== wheel ===" -ForegroundColor Cyan
Write-Host "  $Wheel"
Write-Host "  cftuv_native $wheelVersion, tags: $($tags -join ', ')"

# --- Blender -----------------------------------------------------------------
$configName = $BlenderVersion
if ($ModulesDir) {
    $modules = $ModulesDir
} else {
    $configRoot = Join-Path $env:APPDATA "Blender Foundation\Blender"
    if (-not (Test-Path $configRoot)) { Fail "$configRoot not found: has Blender ever been started" }
    $versions = Get-ChildItem $configRoot -Directory |
        Where-Object { $_.Name -match '^\d+\.\d+$' } |
        Sort-Object { [version]$_.Name } -Descending
    if (-not $versions) { Fail "no version folders in $configRoot" }
    if ($BlenderVersion) {
        $chosen = $versions | Where-Object { $_.Name -eq $BlenderVersion }
        if (-not $chosen) { Fail "version $BlenderVersion not found; there are: $($versions.Name -join ', ')" }
    } else {
        $chosen = $versions[0]
        if ($versions.Count -gt 1) {
            Write-Host "  several versions ($($versions.Name -join ', ')), the newest is taken" -ForegroundColor Yellow
        }
    }
    $configName = $chosen.Name
    $modules = Join-Path $chosen.FullName "scripts\modules"
}
$modules = $modules.TrimEnd('\')
$scriptsDir = Split-Path -Parent $modules
Write-Host ""
Write-Host "=== target ===" -ForegroundColor Cyan
Write-Host "  modules: $modules"

# --- Python Blender: совместим ли с колесом ----------------------------------
$py = $PythonExe
if (-not $py) {
    if (-not $configName) { Fail "give -PythonExe or -BlenderVersion: the Blender Python cannot be located" }
    $py = "C:\Program Files\Blender Foundation\Blender $configName\$configName\python\bin\python.exe"
}
if (-not (Test-Path -LiteralPath $py -PathType Leaf)) { Fail "Python for the compatibility check not found: $py" }
$probe = Join-Path ([System.IO.Path]::GetTempPath()) ("cftuv_native_probe_" + [guid]::NewGuid().ToString('N').Substring(0, 8) + ".py")
$probeLines = @(
    "import json, struct, sys",
    "print(json.dumps({'version': [sys.version_info[0], sys.version_info[1], sys.version_info[2]], 'bits': struct.calcsize('P') * 8, 'platform': sys.platform}))"
)
Set-Content -Path $probe -Value $probeLines -Encoding ascii
try {
    $shape = Invoke-Python $py @($probe)
} finally {
    Remove-Item -LiteralPath $probe -Force -ErrorAction SilentlyContinue
}
if ($shape.ExitCode -ne 0) { Fail "cannot run $py : $($shape.Err)" }
$interpreter = $shape.Out | ConvertFrom-Json
$major = [int]$interpreter.version[0]
$minor = [int]$interpreter.version[1]
Write-Host "  Blender Python: $py ($major.$minor.$([int]$interpreter.version[2]), $($interpreter.bits)-bit, $($interpreter.platform))"
$compatible = $false
$problems = @()
foreach ($tag in $tags) {
    $parsed = [regex]::Match($tag, '^(?<py>[a-z]+)(?<ver>\d+)-(?<abi>[^-]+)-(?<plat>.+)$')
    if (-not $parsed.Success) { $problems += "unreadable tag $tag"; continue }
    $tagPython = $parsed.Groups['py'].Value
    $tagAbi = $parsed.Groups['abi'].Value
    $platformOk = switch ($parsed.Groups['plat'].Value) {
        "any"       { $true }
        "win_amd64" { ($interpreter.platform -eq "win32") -and ($interpreter.bits -eq 64) }
        default     { $false }
    }
    $pythonOk = $false
    $version = [regex]::Match($parsed.Groups['ver'].Value, '^(\d)(\d+)$')
    if ($tagPython -eq "cp" -and $version.Success) {
        $tagMajor = [int]$version.Groups[1].Value
        $tagMinor = [int]$version.Groups[2].Value
        if ($tagAbi -eq "abi3") { $pythonOk = ($major -eq $tagMajor) -and ($minor -ge $tagMinor) }
        else { $pythonOk = ($major -eq $tagMajor) -and ($minor -eq $tagMinor) }
    } elseif ($tagPython -eq "py") {
        $pythonOk = $true
    }
    if ($platformOk -and $pythonOk) { $compatible = $true } else { $problems += "tag $tag does not fit Python $major.$minor on $($interpreter.platform)/$($interpreter.bits)" }
}
if ($compatible -and $requires -ne $null -and (($major -lt $requires[0]) -or (($major -eq $requires[0]) -and ($minor -lt $requires[1])))) {
    $compatible = $false
    $problems += "the wheel requires Python >= $($requires[0]).$($requires[1])"
}
if (-not $compatible) { Fail ("the wheel does not fit the Blender Python: " + ($problems -join "; ")) }
Write-Host "  the wheel fits this Python" -ForegroundColor Green

# --- что уже установлено и занято ---------------------------------------------
$packageTarget = Join-Path $modules "cftuv_native"
$distTarget = Join-Path $modules $distInfo
$obsolete = @()
if (Test-Path -LiteralPath $modules) {
    $obsolete = @(Get-ChildItem -LiteralPath $modules -Directory -Filter "cftuv_native-*.dist-info" | Where-Object { $_.Name -ne $distInfo } | ForEach-Object { $_.FullName })
}
$installedVersion = ""
if (Test-Path -LiteralPath $packageTarget) {
    $existing = @(Get-ChildItem -LiteralPath $modules -Directory -Filter "cftuv_native-*.dist-info" | ForEach-Object { $_.Name -replace '^cftuv_native-(.+)\.dist-info$', '$1' })
    $installedVersion = if ($existing.Count -gt 0) { $existing -join ", " } else { "unknown" }
}
Write-Host ""
Write-Host "=== installed now ===" -ForegroundColor Cyan
if ($installedVersion) { Write-Host "  cftuv_native $installedVersion" } else { Write-Host "  cftuv_native is not installed" }
$locked = New-Object System.Collections.ArrayList
foreach ($path in @($packageTarget) + @($distTarget) + $obsolete) {
    foreach ($file in (Get-LockedFiles $path)) { [void]$locked.Add($file) }
}
if ($locked.Count -gt 0) { Write-Host "  held by another process: $($locked.Count) file(s)" -ForegroundColor Yellow } else { Write-Host "  no file is held" }

if ($WhatIf) {
    Write-Host ""
    Write-Host "  -WhatIf: nothing was changed." -ForegroundColor Yellow
    if ($locked.Count -gt 0) { Write-Host "  An install now would be refused: close Blender first." -ForegroundColor Yellow }
    exit 0
}

# --- сторона: распаковка и сверка ---------------------------------------------
$stagingRoot = Join-Path $scriptsDir (".cftuv_staging\" + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Force -Path $stagingRoot | Out-Null
try {
    $archive = [System.IO.Compression.ZipFile]::OpenRead($Wheel)
    try {
        foreach ($entry in $archive.Entries) {
            if ($entry.FullName.EndsWith("/")) { continue }
            $destination = Join-Path $stagingRoot ($entry.FullName.Replace('/', '\'))
            New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
            [System.IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $destination, $true)
        }
    } finally {
        $archive.Dispose()
    }
} catch {
    Remove-Staging $stagingRoot | Out-Null
    Fail "cannot unpack the wheel: $($_.Exception.Message)"
}
$stagedPackage = Join-Path $stagingRoot "cftuv_native"
$stagedDist = Join-Path $stagingRoot $distInfo
$newPrint = Get-TreeFingerprint $stagedPackage
$swap = $null
$unchanged = $false
if ((Test-Path -LiteralPath $packageTarget) -and (Test-Path -LiteralPath $distTarget) -and ($obsolete.Count -eq 0)) {
    if ((Get-TreeFingerprint $packageTarget) -eq $newPrint) {
        # установлено то же самое: остаётся только проверка (ниже)
        Remove-Staging $stagingRoot | Out-Null
        Write-Host ""
        Write-Host "  already installed: cftuv_native $wheelVersion (fingerprint $newPrint), nothing changed" -ForegroundColor Green
        $unchanged = $true
    }
}

if (-not $unchanged) {
    if ($locked.Count -gt 0) {
        Remove-Staging $stagingRoot | Out-Null
        Fail (Get-LockMessage "the installed cftuv_native" $locked)
    }
    Write-Host ""
    Write-Host "=== swap ===" -ForegroundColor Cyan
    New-Item -ItemType Directory -Force -Path $modules | Out-Null
    $pairs = @(
        @{ Staged = $stagedPackage; Target = $packageTarget },
        @{ Staged = $stagedDist;    Target = $distTarget }
    )
    foreach ($old in $obsolete) { $pairs += @{ Staged = $null; Target = $old } }
    try {
        $swap = Install-Directories $pairs (Join-Path $stagingRoot "old")
    } catch {
        $reason = $_.Exception.Message
        Remove-Staging $stagingRoot | Out-Null
        Fail "the swap was refused, the previous installation is untouched: $reason"
    }
    Write-Host "  installed: cftuv_native $wheelVersion -> $packageTarget"
    $installedPrint = Get-TreeFingerprint $packageTarget
    if ($installedPrint -ne $newPrint) {
        Undo-Install $swap
        Remove-Staging $stagingRoot | Out-Null
        Fail "the installed copy differs from the wheel (fingerprint $installedPrint, wheel $newPrint); the previous installation is restored"
    }
}

# --- доказательство: импортируется ли ------------------------------------------
$checker = Join-Path ([System.IO.Path]::GetTempPath()) ("cftuv_native_check_" + [guid]::NewGuid().ToString('N').Substring(0, 8) + ".py")
$checkLines = @(
    "import json, sys",
    "sys.path.append(sys.argv[1])",
    "try:",
    "    import cftuv_native as native",
    "except Exception as exc:",
    "    print(json.dumps({'ok': False, 'error': '%s: %s' % (type(exc).__name__, exc)}))",
    "    sys.exit(0)",
    "report = {'ok': True, 'python': '%d.%d.%d' % sys.version_info[:3], 'path': native.__file__}",
    "version = getattr(native, 'native_version', None)",
    "report['version'] = version() if version else ''",
    "status = getattr(native, 'native_status', None)",
    "if status is None:",
    "    report['status'] = 'no native_status in this wheel'",
    "else:",
    "    try:",
    "        report['status'] = status()",
    "    except Exception as exc:",
    "        report['status'] = 'error: %s: %s' % (type(exc).__name__, exc)",
    "print(json.dumps(report))"
)
Set-Content -Path $checker -Value $checkLines -Encoding ascii

function Test-Import([string]$Exe, [bool]$Isolated) {
    $arguments = @()
    if ($Isolated) { $arguments += @("-I", "-S") }
    $arguments += @($checker, $modules)
    $result = Invoke-Python $Exe $arguments
    if ($result.ExitCode -ne 0) { return @{ ok = $false; error = "exit code $($result.ExitCode): $($result.Err)" } }
    try { return ($result.Out | ConvertFrom-Json) } catch { return @{ ok = $false; error = "unreadable answer: $($result.Out)" } }
}

Write-Host ""
Write-Host "=== import check ===" -ForegroundColor Cyan
try {
    $report = Test-Import $py $false
    if (-not $report.ok) {
        if ($swap -ne $null) {
            Undo-Install $swap
            Remove-Staging $stagingRoot | Out-Null
            Fail "the Blender Python cannot import cftuv_native ($($report.error)); the previous installation is restored"
        }
        Fail "the Blender Python cannot import the installed cftuv_native: $($report.error)"
    }
    Write-Host "  Blender Python $($report.python): cftuv_native $($report.version) from $($report.path)"
    Write-Host ("  native_status: " + ($report.status | ConvertTo-Json -Compress))
    $workerFailure = ""
    if ($WorkerPython) {
        if (-not (Test-Path -LiteralPath $WorkerPython -PathType Leaf)) {
            $workerFailure = "Worker Python is not a file: $WorkerPython"
        } else {
            $worker = Test-Import $WorkerPython $true
            if ($worker.ok) {
                Write-Host "  Worker Python $($worker.python) (-I -S, folder from the parent): cftuv_native $($worker.version)"
            } else {
                $workerFailure = "the worker Python cannot import cftuv_native: $($worker.error)"
            }
        }
    }
} finally {
    Remove-Item -LiteralPath $checker -Force -ErrorAction SilentlyContinue
}
if ($swap -ne $null) { Commit-Install $swap $stagingRoot }
Write-Host ""
Write-Host "  Installed: cftuv_native $wheelVersion in $modules" -ForegroundColor Green
if ($workerFailure) {
    Write-Host "  WARNING: $workerFailure" -ForegroundColor Yellow
    Write-Host "  Domains computed by that worker are named NATIVE_UNAVAILABLE and computed in Python." -ForegroundColor Yellow
    exit 3
}
