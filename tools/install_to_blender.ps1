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
function Deploy($sourcePath, $targetRoot, $label) {
    New-Item -ItemType Directory -Force -Path $targetRoot | Out-Null
    $target = Join-Path $targetRoot (Split-Path -Leaf $sourcePath)
    # Старая копия удаляется целиком: частичное наложение оставляет файлы,
    # удалённые в репозитории, и они продолжают работать.
    if (Test-Path $target) { Remove-Item -Recurse -Force $target }
    Copy-Item -Recurse -Force $sourcePath $targetRoot
    # `__pycache__` от прежней версии переживает копирование исходников и
    # выглядит как «поведение застряло».
    Get-ChildItem $target -Recurse -Directory -Filter "__pycache__" -ErrorAction SilentlyContinue |
        Remove-Item -Recurse -Force
    Write-Host "  скопировано: $label -> $target"
    return $target
}

Write-Host ""
Write-Host "=== копирование ===" -ForegroundColor Cyan
$addonTarget  = Deploy (Join-Path $repo "cftuv") $addons "аддон cftuv"
$kernelTarget = Deploy (Join-Path $repo "kernel\src\cftuv_envelope") $modules "ядро cftuv_envelope"

# --- доказательство ----------------------------------------------------------
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

Write-Host ""
Write-Host "=== отпечатки ===" -ForegroundColor Cyan
$failed = $false
foreach ($pair in @(
    @{ label = "cftuv";           source = Join-Path $repo "cftuv";                    target = $addonTarget },
    @{ label = "cftuv_envelope";  source = Join-Path $repo "kernel\src\cftuv_envelope"; target = $kernelTarget }
)) {
    $want = Fingerprint $pair.source
    $got  = Fingerprint $pair.target
    if ($want -eq $got) {
        Write-Host "  ОК  $($pair.label): $got"
    } else {
        Write-Host "  НЕТ $($pair.label): репозиторий $want, установлено $got" -ForegroundColor Red
        $failed = $true
    }
}

Write-Host ""
if ($failed) {
    Fail "отпечатки разошлись — установленная копия НЕ совпадает с репозиторием"
}

# --- штамп версии -------------------------------------------------------------
# Пишется ПОСЛЕ сверки отпечатков (Deploy стирает цель целиком, так что штамп
# прошлой установки не переживает копирование и не портит сверку). Причина
# существования: владелец ставил сборку из чекаута, отставшего на день, и
# «каждый раз запускал старый» — без штампа это неотличимо от поломки.
$commit = (git -C $repo rev-parse --short HEAD 2>$null)
$branch = (git -C $repo rev-parse --abbrev-ref HEAD 2>$null)
$stamp = "commit $commit ($branch), установлено $(Get-Date -Format 'yyyy-MM-dd HH:mm') из $repo"
foreach ($target in @($addonTarget, $kernelTarget)) {
    Set-Content -Path (Join-Path $target "_install_stamp.txt") -Value $stamp -Encoding utf8
}
Write-Host "  Штамп: $stamp" -ForegroundColor Cyan
Write-Host "  Установлено и сверено." -ForegroundColor Green
Write-Host "  Дальше: Blender -> Edit -> Preferences -> Add-ons -> включить CFTUV,"
Write-Host "  затем Scripting -> открыть tools\blender_check_install.py -> Run Script."
