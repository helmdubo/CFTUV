# Скопировать аддон и ядро в Blender и ДОКАЗАТЬ, что скопировалось.
#
# Проверка отпечатков здесь не украшение. Однажды правка углового сертификата
# считалась установленной, не будучи ею: копию взяли из другой ветки, полевой
# прогон показал прежнее поведение, и день ушёл на поиск несуществующего бага.
# Поэтому скрипт заканчивается не словом «готово», а сравнением хешей.
#
# Запуск (двойной клик по install_to_blender.bat либо):
#   powershell -ExecutionPolicy Bypass -File tools\install_to_blender.ps1
#   ... -BlenderVersion 4.3        # если версий несколько
#   ... -WhatIf                    # показать, что будет сделано, и выйти

[CmdletBinding()]
param(
    [string]$BlenderVersion = "",
    [switch]$WhatIf
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

function Fail($message) {
    Write-Host ""
    Write-Host "  НЕ ГОТОВО: $message" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=== репозиторий ===" -ForegroundColor Cyan
Write-Host "  $repo"
foreach ($needed in @("cftuv", "kernel\src\cftuv_envelope")) {
    if (-not (Test-Path (Join-Path $repo $needed))) {
        Fail "в репозитории нет $needed — скрипт запущен не оттуда"
    }
}

# --- найти Blender -----------------------------------------------------------
# Конфиг пользователя, а не Program Files: `scripts\modules` не требует
# администратора и ПЕРЕКРЫВАЕТ устаревшую копию в site-packages, если она там
# осталась с прежних установок.
$configRoot = Join-Path $env:APPDATA "Blender Foundation\Blender"
if (-not (Test-Path $configRoot)) { Fail "не найден $configRoot — Blender ни разу не запускался?" }

$versions = Get-ChildItem $configRoot -Directory |
    Where-Object { $_.Name -match '^\d+\.\d+$' } |
    Sort-Object { [version]$_.Name } -Descending
if (-not $versions) { Fail "в $configRoot нет каталогов версий" }

if ($BlenderVersion) {
    $chosen = $versions | Where-Object { $_.Name -eq $BlenderVersion }
    if (-not $chosen) {
        Fail "версии $BlenderVersion нет; есть: $($versions.Name -join ', ')"
    }
} else {
    $chosen = $versions[0]
    if ($versions.Count -gt 1) {
        Write-Host "  версий несколько ($($versions.Name -join ', ')), взята новейшая:" -ForegroundColor Yellow
    }
}

$addons  = Join-Path $chosen.FullName "scripts\addons"
$modules = Join-Path $chosen.FullName "scripts\modules"
Write-Host ""
Write-Host "=== Blender $($chosen.Name) ===" -ForegroundColor Cyan
Write-Host "  аддоны : $addons"
Write-Host "  модули : $modules"

if ($WhatIf) {
    Write-Host ""
    Write-Host "  --WhatIf: ничего не скопировано." -ForegroundColor Yellow
    exit 0
}

# --- копирование -------------------------------------------------------------
# Установка - ПОДМЕНА каталогов, а не «стереть и скопировать» (см. install_common.ps1). Прежняя версия стирала старую копию
# целиком и копировала новую: пока Blender держал .pyc ядра (воркеры пула), стирание обрывалось на середине, штамп установки
# исчезал, а аддон оставался наполовину стёртым. Теперь новая копия собирается и сверяется В СТОРОНЕ, занятые файлы называются
# ДО подмены, старый каталог уходит одним переименованием, а сбой возвращает прежнюю установку вместе со штампом.
. (Join-Path $PSScriptRoot "install_common.ps1")

$scriptsDir   = Join-Path $chosen.FullName "scripts"
$stagingRoot  = Join-Path $scriptsDir (".cftuv_staging\" + [guid]::NewGuid().ToString('N').Substring(0, 8))
$addonTarget  = Join-Path $addons "cftuv"
$kernelTarget = Join-Path $modules "cftuv_envelope"

Write-Host ""
Write-Host "=== занятые файлы ===" -ForegroundColor Cyan
foreach ($held in @(
    @{ label = "the installed cftuv add-on";      path = $addonTarget },
    @{ label = "the installed cftuv_envelope kernel"; path = $kernelTarget }
)) {
    $locked = Get-LockedFiles $held.path
    if ($locked.Count -gt 0) { Fail (Get-LockMessage $held.label $locked) }
}
Write-Host "  занятых файлов нет"

# --- проверка отпечатков -----------------------------------------------------
# Считается Python'ом самого Blender: он гарантированно есть и им же будет
# исполняться установленный код.
$blenderPython = Get-ChildItem "C:\Program Files\Blender Foundation\Blender $($chosen.Name)\$($chosen.Name)\python\bin\python.exe" -ErrorAction SilentlyContinue
if (-not $blenderPython) {
    $blenderPython = Get-Command python -ErrorAction SilentlyContinue
}
if (-not $blenderPython) { Fail "не найден python для проверки отпечатков" }
# Оператор `??` появился только в PowerShell 7, а в Windows штатно стоит 5.1, и
# там это ошибка РАЗБОРА: скрипт падает целиком, не выполнив ни строки и не
# напечатав ни одной своей диагностики. Так у владельца установка и не пошла.
# `Get-ChildItem` возвращает FileInfo (у него `FullName`), `Get-Command` —
# ApplicationInfo (у него `Source`), поэтому берётся тот, который есть.
$py = if ($blenderPython.PSObject.Properties['Source'] -and $blenderPython.Source) {
    $blenderPython.Source
} else {
    $blenderPython.FullName
}
$checker = Join-Path $repo "tools\blender_check_install.py"

function Fingerprint($path) {
    (& $py $checker --fingerprint-path $path).Trim()
}

function Stage($sourcePath, $label) {
    # Копия во временный каталог рядом с аддонами; `__pycache__` от прежней версии не переживает копирование исходников и
    # выглядит как «поведение застряло».
    New-Item -ItemType Directory -Force -Path $stagingRoot | Out-Null
    Copy-Item -Recurse -Force $sourcePath $stagingRoot
    $staged = Join-Path $stagingRoot (Split-Path -Leaf $sourcePath)
    Get-ChildItem $staged -Recurse -Directory -Filter "__pycache__" -ErrorAction SilentlyContinue |
        Remove-Item -Recurse -Force
    Write-Host "  собрано в стороне: $label -> $staged"
    return $staged
}

Write-Host ""
Write-Host "=== сборка в стороне ===" -ForegroundColor Cyan
$sources = @(
    @{ label = "cftuv";          source = Join-Path $repo "cftuv";                     target = $addonTarget },
    @{ label = "cftuv_envelope"; source = Join-Path $repo "kernel\src\cftuv_envelope"; target = $kernelTarget }
)
foreach ($item in $sources) {
    $item.staged = Stage $item.source $item.label
}

Write-Host ""
Write-Host "=== отпечатки (до подмены) ===" -ForegroundColor Cyan
$failed = $false
foreach ($item in $sources) {
    $want = Fingerprint $item.source
    $got  = Fingerprint $item.staged
    if ($want -eq $got) {
        Write-Host "  ОК  $($item.label): $got"
    } else {
        Write-Host "  НЕТ $($item.label): репозиторий $want, собрано $got" -ForegroundColor Red
        $failed = $true
    }
}
if ($failed) {
    Remove-Staging $stagingRoot | Out-Null
    Fail "отпечатки разошлись — новая копия НЕ совпадает с репозиторием; установленное не тронуто"
}

# --- штамп версии -------------------------------------------------------------
# Пишется в новую копию ДО подмены: установленный аддон либо старый целиком (со своим штампом), либо новый целиком (со своим).
# Причина существования: владелец ставил сборку из чекаута, отставшего на день, и
# «каждый раз запускал старый» — без штампа это неотличимо от поломки.
$commit = (git -C $repo rev-parse --short HEAD 2>$null)
$branch = (git -C $repo rev-parse --abbrev-ref HEAD 2>$null)
$stamp = "commit $commit ($branch), установлено $(Get-Date -Format 'yyyy-MM-dd HH:mm') из $repo"
foreach ($item in $sources) {
    Set-Content -Path (Join-Path $item.staged "_install_stamp.txt") -Value $stamp -Encoding utf8
}

# --- подмена -----------------------------------------------------------------
Write-Host ""
Write-Host "=== подмена ===" -ForegroundColor Cyan
try {
    $swap = Install-Directories @(
        @{ Staged = $sources[0].staged; Target = $sources[0].target },
        @{ Staged = $sources[1].staged; Target = $sources[1].target }
    ) (Join-Path $stagingRoot "old")
} catch {
    $reason = $_.Exception.Message
    Remove-Staging $stagingRoot | Out-Null
    Fail "the swap was refused, the previous installation is untouched: $reason"
}
foreach ($item in $sources) {
    Write-Host "  установлено: $($item.label) -> $($item.target)"
}

# --- доказательство ----------------------------------------------------------
Write-Host ""
Write-Host "=== отпечатки (после подмены) ===" -ForegroundColor Cyan
$failed = $false
foreach ($item in $sources) {
    $want = Fingerprint $item.source
    $got  = Fingerprint $item.target
    if ($want -eq $got) {
        Write-Host "  ОК  $($item.label): $got"
    } else {
        Write-Host "  НЕТ $($item.label): репозиторий $want, установлено $got" -ForegroundColor Red
        $failed = $true
    }
}
if ($failed) {
    Undo-Install $swap
    Remove-Staging $stagingRoot | Out-Null
    Fail "отпечатки разошлись после подмены — прежняя установка возвращена"
}
Commit-Install $swap $stagingRoot

Write-Host ""
Write-Host "  Штамп: $stamp" -ForegroundColor Cyan
Write-Host "  Установлено и сверено." -ForegroundColor Green
Write-Host "  Дальше: Blender -> Edit -> Preferences -> Add-ons -> включить CFTUV,"
Write-Host "  затем Scripting -> открыть tools\blender_check_install.py -> Run Script."
