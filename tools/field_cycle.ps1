# Полевой цикл на building.002: замер на настоящем меше, а не на синтетике.
#
# Что делает, по шагам:
#   1. (по желанию, -Install) кладёт аддон и ядро в Blender и сверяет отпечатки
#      — это install_to_blender.ps1, ничего нового;
#   2. гоняет полевые ворота run_envelope_mr1_building_gate.py на building.002:
#      по умолчанию — ФОНОВЫМ Blender'ом по файлу сцены (ничего не трогает в
#      сеансе владельца, воспроизводимо, свежий интерпретатор — кэш модулей не
#      врёт); с ключом -Live — в ЖИВОМ сеансе через remote control (порт 9876),
#      если проверяется именно то, что владелец видит на экране сейчас;
#   3. печатает сводку: домены, Resolved N/M, секунды.
#
# Ворота сами ставят код РЕПОЗИТОРИЯ в начало sys.path и вычищают кэш модулей
# cftuv*/cftuv_envelope*, поэтому шаг -Install для замера НЕ обязателен: правка
# в рабочей копии видна следующему же прогону. -Install нужен, когда проверяется
# именно установленная копия (то, что видит аддон при обычной работе владельца).
#
# Запуск:
#   tools\field_cycle.bat                       — фоновый прогон всех scope
#   tools\field_cycle.bat -Scopes all_seam_chains_l0
#   tools\field_cycle.bat -Live -TimeoutSeconds 1800
#   tools\field_cycle.bat -Install
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

if (-not (Test-Path (Join-Path $repo "tools\run_envelope_mr1_building_gate.py"))) {
    Fail "скрипт запущен не из репозитория"
}

if ($Install) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repo "tools\install_to_blender.ps1")
    if ($LASTEXITCODE -ne 0) { Fail "установка не прошла — замер по ней не имеет смысла" }
}

if (-not $Output) {
    $slug = ($ObjectName -replace '[^0-9A-Za-z]+', '_').ToLower()
    $Output = Join-Path $repo "artifacts\envelope_runtime_r1\${slug}_field_run.json"
}
$gate = Join-Path $repo "tools\run_envelope_mr1_building_gate.py"
# Позиционные аргументы ворот: выход, scope'ы, объект, движки. Литерал "all"
# сохраняет прежнее умолчание «все scope», позволяя передать объект позиционно.
$scopeArg = if ($Scopes) { $Scopes } else { "all" }
$tailArgs = @($Output, $scopeArg, $ObjectName, $Engines)

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
