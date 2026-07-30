# Полевой цикл на building.002: замер на настоящем меше, а не на синтетике.
#
# Что делает, по шагам:
#   1. (по желанию, -Install) кладёт аддон и ядро в Blender и сверяет отпечатки
#      — это install_to_blender.ps1, ничего нового;
#   2. гоняет полевые ворота run_envelope_mr1_building_gate.py на building.002:
#      по умолчанию — ФОНОВЫМ Blender'ом по файлу сцены (ничего не трогает в
#      сеансе владельца, воспроизводимо, свежий интерпретатор — кэш модулей не
#      врёт); ворота исполняют УСТАНОВЛЕННЫЕ пакеты и сверяют их штампы и
#      отпечатки с HEAD; с ключом -Live — в ЖИВОМ сеансе через remote control,
#      если проверяется именно то, что владелец видит на экране сейчас;
#   3. печатает сводку: домены, Resolved N/M, секунды.
#
# Density-ворота не подменяют импорт checkout'ом: принимается только установленный
# продукт с двумя совпавшими штампами/отпечатками. `-Install` нужен после каждого
# изменения; без него допустим лишь уже установленный точно тот же HEAD.
#
# Запуск:
#   tools\field_cycle.bat -Density 1 -Install
#   tools\field_cycle.bat -Density 0 -Scopes all_seam_chains_l0
#   tools\field_cycle.bat -Density legacy       — только контроль, не приёмка
#
# Выход: artifacts\envelope_runtime_r1\building_002_field_run.json (не коммитится).

[CmdletBinding()]
param(
    [switch]$Live,
    [switch]$Install,
    [string]$Scopes = "",
    [string]$Scene = "E:\testscene.blend",
    [string]$BlenderExe = "",
    [string]$Output = "",
    [int]$TimeoutSeconds = 1800,
    # Ручка обязана быть измерена явно. legacy — именованный контроль старого
    # 60° закона; product-acceptance разрешена только для 0/1/4.
    [string]$Density = "",
    [switch]$DryRun,
    # Имя меша в сцене. Ворота по умолчанию заточены на building.002; для
    # другого меша осмыслен только scope all_seam_chains_l0.
    [string]$ObjectName = "building.002",
    # "raw,queue" либо "queue": RAW на сотнях цепочек стоит часы, пропуск
    # обязан быть явным выбором.
    [string]$Engines = "raw,queue"
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

function Fail($message) {
    Write-Host ""
    Write-Host "  НЕ ГОТОВО: $message" -ForegroundColor Red
    exit 1
}

$validDensity = @("0", "1", "4", "legacy")
if (-not $validDensity.Contains($Density)) {
    Fail "UNMEASURED_REQUEST_POLICY_KNOB: укажите -Density 0, 1, 4 или legacy"
}

if (-not (Test-Path (Join-Path $repo "tools\run_envelope_mr1_building_gate.py"))) {
    Fail "скрипт запущен не из репозитория"
}

if ($Install) {
    $canonicalInstallRoot = "E:\GITHUB\CFTUV"
    if (
        -not (Test-Path -LiteralPath $canonicalInstallRoot) -or
        (Resolve-Path -LiteralPath $repo).Path -ne
            (Resolve-Path -LiteralPath $canonicalInstallRoot).Path
    ) {
        Fail "INSTALL_SOURCE_NOT_CANONICAL: установка разрешена только из E:\GITHUB\CFTUV"
    }
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repo "tools\install_to_blender.ps1")
    if ($LASTEXITCODE -ne 0) { Fail "установка не прошла — замер по ней не имеет смысла" }
}

if (-not $Output) {
    $slug = ($ObjectName -replace '[^0-9A-Za-z]+', '_').ToLower()
    $Output = Join-Path $repo "artifacts\envelope_runtime_r1\${slug}_density_${Density}_field_run.json"
}
$gate = Join-Path $repo "tools\run_envelope_mr1_building_gate.py"
# Позиционные аргументы: выход, scope'ы, объект, движки, density. Литерал "all"
# сохраняет прежнее умолчание scope, но измеряемая ручка умолчания не имеет.
$scopeArg = if ($Scopes) { $Scopes } else { "all" }
$tailArgs = @($Output, $scopeArg, $ObjectName, $Engines, $Density)
$env:PYTHONSAFEPATH = "1"

if ($DryRun) {
    [ordered]@{
        schema = "cftuv.envelope.runtime_metric_building_gate.v2"
        measurement_rule = "REQUEST_POLICY_KNOBS_MEASURED_V1"
        density = $Density
        acceptance_eligible = $Density -ne "legacy"
        python_safe_path = $env:PYTHONSAFEPATH
        gate_arguments = $tailArgs
    } | ConvertTo-Json -Compress
    exit 0
}

if ($Live) {
    # Живой сеанс: интерфейс владельца будет занят на всё время счёта.
    Write-Host "=== полевой прогон в ЖИВОМ сеансе (порт 9876) ===" -ForegroundColor Cyan
    $scriptArgs = @("--") + $tailArgs
    & python (Join-Path $repo "tools\blender_remote.py") run `
        --file $gate --timeout $TimeoutSeconds --script-args @scriptArgs
    if ($LASTEXITCODE -ne 0) { Fail "живой сеанс вернул ошибку" }
} else {
    if (-not (Test-Path $Scene)) { Fail "нет файла сцены $Scene" }
    if (-not $BlenderExe) {
        $found = Get-ChildItem "C:\Program Files\Blender Foundation\Blender *\blender.exe" -ErrorAction SilentlyContinue |
            Sort-Object FullName -Descending | Select-Object -First 1
        if (-not $found) { Fail "не найден blender.exe — укажите -BlenderExe" }
        $BlenderExe = $found.FullName
    }
    Write-Host "=== полевой прогон фоновым Blender ===" -ForegroundColor Cyan
    Write-Host "  $BlenderExe"
    Write-Host "  сцена: $Scene, меш: $ObjectName, движки: $Engines"
    # Прошлый выход стирается ДО прогона: упавшие ворота не оставят файла, и
    # сводка не сможет молча пересказать устаревший JSON. --python-exit-code
    # обязателен: без него Blender выходит нулём даже при падении скрипта.
    if (Test-Path $Output) { Remove-Item -Force $Output }
    $gateArgs = @("-b", $Scene, "--python-exit-code", "1", "--python", $gate, "--") + $tailArgs
    & $BlenderExe @gateArgs
    if ($LASTEXITCODE -ne 0) { Fail "фоновый Blender вернул код $LASTEXITCODE" }
}

if (-not (Test-Path $Output)) { Fail "ворота не оставили файла $Output" }
Write-Host ""
Write-Host "=== сводка ===" -ForegroundColor Cyan
& python (Join-Path $repo "tools\field_cycle_summary.py") $Output
exit $LASTEXITCODE
