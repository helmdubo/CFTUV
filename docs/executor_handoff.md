# Executor handoff

Этот файл содержит только локальные операционные детали рабочей машины. План,
геометрические контракты и критерии срезов находятся в канонических документах.

## Checkout и установленный аддон

- Рабочий checkout: `C:\Users\helmd\Documents\CFTUV_decal_runtime`.
- Ветка: `claude/blender-decal-corner-preview-yq4lir`.
- Установленный пакет Blender 4.3:
  `C:\Users\helmd\AppData\Roaming\Blender Foundation\Blender\4.3\scripts\addons\cftuv`.
- Blender: `C:\Program Files\Blender Foundation\Blender 4.3\blender.exe`.

Не выполнять decal-runtime работу в соседнем
`C:\Users\helmd\Documents\CFTUV`: это другой worktree.

## Hot-sync после implementation commit

Обычный итерационный sync затрагивает пять файлов:

1. `cftuv\decal_voronoi.py`
2. `cftuv\decals.py`
3. `cftuv\model.py`
4. `cftuv\operators.py`
5. `cftuv\version_info.py`

Из корня checkout:

```powershell
$addon = 'C:\Users\helmd\AppData\Roaming\Blender Foundation\Blender\4.3\scripts\addons\cftuv'
$files = @('decal_voronoi.py', 'decals.py', 'model.py', 'operators.py', 'version_info.py')

foreach ($file in $files) {
    Copy-Item -Force -LiteralPath (Join-Path (Get-Location) "cftuv\$file") `
        -Destination (Join-Path $addon $file)
}
```

После копирования обязательна SHA-256-проверка обеих сторон, а не сравнение
timestamp/размера:

```powershell
foreach ($file in $files) {
    $repo = Join-Path (Get-Location) "cftuv\$file"
    $installed = Join-Path $addon $file
    $repoHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $repo).Hash
    $installedHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $installed).Hash
    if ($repoHash -ne $installedHash) {
        throw "Addon sync mismatch: $file"
    }
}
```

Если implementation меняет другой модуль пакета, его тоже нужно скопировать и
включить в hash-проверку: список выше — hot-sync текущего decal/rail runtime, а
не разрешение оставлять зависимость старой версии.

Копирования недостаточно: Blender держит Python-модули в `sys.modules`. После
sync пакет перезагружается через MCP `execute_blender_code` (или тем же кодом в
Blender Python Console):

```python
import addon_utils
import importlib
import sys

addon_utils.disable("cftuv", default_set=False)
for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]
importlib.invalidate_caches()
addon_utils.enable("cftuv", default_set=False, persistent=False)
```

`version_info.py` копируется обязательно: установленная папка не содержит
`.git`, поэтому панель Hotspot UV использует embedded `Code commit`. Это SHA
implementation-коммита; последующий metadata-only HEAD закономерно новее.

## Blender и MCP

Для live-проверки запускается обычный Blender GUI. В правой `N`-панели viewport
нужно открыть вкладку `BlenderMCP` и нажать `Connect to MCP server`; локальный
порт — `9876`. Сервер не стартует автоматически вместе с Blender.

Наличие процессов `blender-mcp.exe` не является health-check. В этой среде
одновременно могут оставаться клиенты от Claude и Codex. Источник истины:

```powershell
Get-NetTCPConnection -LocalPort 9876 -ErrorAction SilentlyContinue
```

Если listener отсутствует, `get_scene_info` и `execute_blender_code` отвечают
`Could not connect to Blender`, даже когда `blender.exe` и несколько
`blender-mcp.exe` видны в списке процессов. Нужно запустить bridge кнопкой в
Blender, а не перезапускать test suite.

Минимальная последовательность live-проверки:

1. `get_scene_info` — связь и актуальная сцена.
2. `execute_blender_code` — reload пакета, выбор target edges, вызов оператора и
   структурные измерения; длинные сценарии разбиваются на короткие вызовы.
3. `get_viewport_screenshot` — визуальный receipt после генерации и width drag.

Headless Blender-проверки запускаются из корня checkout полным путём к binary,
например:

```powershell
& 'C:\Program Files\Blender Foundation\Blender 4.3\blender.exe' `
    -b 'artifacts\cftuv_decal_verification.blend' `
    --python 'artifacts\verify_decal_runtime.py' -- `
    --output 'artifacts\decal_runtime_benchmark.json'
```

## Artifacts

Все воспроизводимые receipts лежат внутри checkout в
`C:\Users\helmd\Documents\CFTUV_decal_runtime\artifacts`:

- `*.json` — счётчики, topology/provenance и benchmark receipts;
- `*.png` — viewport evidence;
- `*.blend` — сохранённые Blender fixtures;
- `verify_*.py` и `dump_*.py` — headless/MCP repro scripts.

Временные картинки из `AppData\Local\Temp` и несохранённое состояние открытой
сцены не являются артефактами: нужный результат переносится в `artifacts` до
handoff.
