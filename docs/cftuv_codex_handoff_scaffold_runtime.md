# CFTUV Codex Handoff - Scaffold Runtime

## Что это

Этот документ нужен как точка входа для нового AI-агента в Codex.
Он описывает текущее состояние рефакторинга CFTUV, текущий рабочий контекст по `PatchGraph -> ScaffoldMap -> UV runtime`, а также главный блокер, который сейчас мешает получить корректный UV layout.

Читать вместе с:
- `AGENTS.md`
- `docs/cftuv_architecture_v1.4.md`

## Коротко о проекте

CFTUV — Blender addon для semiautomatic UV unwrap hard-surface environment geometry.
Пользователь размечает mesh через `Seam` edges и выделение faces.
Система:
- строит `PatchGraph`
- определяет topology/loops/chains/corners
- планирует solve через `Flow Debug`
- пытается собрать 2D scaffold map
- потом переносит scaffold в UV и запускает `Conformal` для остальных вершин

Центральный IR:
- `PatchGraph` — topology map
- `ScaffoldMap` — solve/runtime 2D frame map

## Главные инварианты

Смотри `AGENTS.md`, но важно повторить:
- `model.py` не импортирует `bpy` и `bmesh`
- `analysis.py` строит `PatchGraph`, не делает runtime UV write
- `solve.py` не делает topology analysis
- `debug.py` не читает BMesh напрямую
- `Sharp` не участвует в patch split, только `Seam`

## Структура модулей

- `cftuv/model.py`
- `cftuv/analysis.py`
- `cftuv/solve.py`
- `cftuv/debug.py`
- `cftuv/Hotspot_UV_v2_5_26.py`

## Что уже сделано

### 1. PatchGraph как реальный backend

`analysis.py` уже строит `PatchGraph`, включая:
- `PatchNode`
- `BoundaryLoop`
- `BoundaryChain`
- `BoundaryCorner`
- `SeamEdge`
- `WorldFacing`

Есть debug:
- `Analyze` / console report по `PatchGraph`
- `Flow Debug` для planner/scoring
- `Scaffold Debug` для `ScaffoldMap`

### 2. Flow planner уже работает

В `solve.py` уже есть:
- `SolverGraph`
- `PatchCertainty`
- `AttachmentCandidate`
- `SolvePlan`
- `QuiltPlan`

`Flow Debug` уже показывает:
- root selection
- frontier candidates
- deferred/rejected links
- quilts как отдельные solve components

Текущий flow-принцип:
- идём по strongest frontier
- если связь ниже порога, останавливаем quilt
- остаток patch-ей идёт в новый root/new quilt cycle

### 3. ScaffoldMap уже существует

В `solve.py` есть runtime solve-level IR:
- `ScaffoldPointKey`
- `ScaffoldChainPlacement`
- `ScaffoldPatchPlacement`
- `ScaffoldQuiltPlacement`
- `ScaffoldMap`

`Scaffold Debug` уже печатает:
- quilts
- patch placements
- chain uv sample points
- corner positions
- bbox patch-а

### 4. Mixed loops уже частично поддерживаются

Раньше `ScaffoldMap` работал только на loops целиком из `H/V`.
Сейчас mixed loops (`H/V + FREE`) уже не падают автоматически в `unsupported`.
`FREE` chains уже строятся не как 2 endpoint, а как polyline samples.

## Что именно работает сейчас

На уровне анализа:
- `PatchGraph` стабилен достаточно для старта solve
- `Flow Debug` даёт осмысленные quilts
- `Scaffold Debug` показывает, что `ScaffoldMap` действительно строится и больше не является фикцией

На уровне runtime:
- `Solve Phase 1 Preview` уже не просто запускает обычный `Conformal`
- scaffold действительно пишется в UV
- quilts уже не сваливаются в один глобальный solve block автоматически

## Что НЕ работает

Главная проблема:

**Финальный UV layout не соответствует тому scaffold, который показывается в `Scaffold Debug`.**

Симптомы:
- UV islands визуально отрываются от тех patch/chain связей, которые уже видны в debug
- child patches могут быть логически приклеены в `ScaffoldMap`, но в UV выглядят висящими в воздухе
- orientation и placement ещё расходятся с тем, как quilt должен расти по flow

Проще:
- `Flow Debug` и `Scaffold Debug` уже говорят одно
- а итоговый UV runtime пока говорит другое

## Последний подтверждённый диагноз

На последней итерации было подтверждено:
- проблема уже не в scorer
- проблема уже не в `PatchGraph`
- проблема в `solve.py`, в слое:
  - `ScaffoldMap -> UV transfer`
  - и в политике final unwrap

Ранее был найден явный баг:
- активный `execute_phase1_preview()` делал один общий `Conformal` по всем patch сразу
- это рвало уже построенные quilts

Этот кусок уже был изменён:
- unwrap теперь идёт по quilt, а не глобально по всем patch сразу

Но этого оказалось недостаточно.
То есть текущий структурный баг глубже:
- либо `_apply_patch_scaffold_to_uv()` пишет не те UV loop corners
- либо child patch placement ещё не соответствует реальному anchor relation
- либо final relax всё ещё получает слишком много свободы внутри конкретного quilt

## Где конкретно смотреть в коде

Основной файл:
- `cftuv/solve.py`

Ключевые active-runtime области:
- `_apply_patch_scaffold_to_uv(...)`
- `_resolve_scaffold_loop_side(...)`
- `_build_patch_scaffold_walk(...)`
- `_build_child_patch_scaffold(...)`
- `build_root_scaffold_map(...)`
- `format_root_scaffold_report(...)`
- `execute_phase1_preview(...)`

Также важно:
- в `solve.py` есть дубли старых функций
- Python использует последние определения в файле
- это надо вычистить, иначе новый агент легко будет чинить не тот runtime path

## Ключевая концептуальная модель

Новый агент не должен пытаться чинить legacy unwrap.
Legacy нужен только как host/UI layer.

Правильная модель solve:

1. `analysis.py` строит `PatchGraph`
2. `Flow Debug` строит `SolvePlan`
3. `ScaffoldMap` строит виртуальную 2D карту:
   - quilts
   - patches
   - chains
   - corners
4. UV должны рождаться от этой карты, а не от локальной 3D-проекции patch-а
5. `Conformal` должен только дорешивать незапиненные вершины

Ключевой принцип:

**UV должны рождаться от уже размещённых chain/corner, а не от локальной проекции patch-а.**

## Ментальная модель solve

Используется модель "потока" / "электричества":
- есть strongest frontier
- следующий шаг берётся из общего пула доступных шагов
- каждый новый patch/chain раскрывает новые варианты
- идём по strongest available step
- если поток обрывается, запускается новый root / новый quilt

Это applies не только к patch-level, но и к chain-level continuity.

## Чего НЕ надо делать

- Не возвращаться к legacy logic как к источнику solve
- Не чинить только косметику debug
- Не подменять проблему runtime solve новым глобальным unwrap
- Не смешивать `Flow Debug` и `Scaffold Debug` с “может быть потом UV исправится”

Если итоговый UV не повторяет scaffold, это runtime bug, а не acceptable approximation.

## Текущий главный blocker

Нужно добиться следующего:

1. `Scaffold Debug` показывает quilt и patch placement
2. `Solve Phase 1 Preview` даёт UV, который повторяет этот scaffold
3. Только после этого имеет смысл двигаться дальше в:
   - child attachment polish
   - better FREE solve
   - HOLE two-pass runtime
   - graph-level docking

Сейчас пункт 2 не выполнен.

## Что должен делать следующий агент

### Приоритет 1

Разобраться, почему `ScaffoldMap -> UV` расходится с реальным UV.

Проверить:
- правильно ли `_apply_patch_scaffold_to_uv()` адресует side-aware UV loops
- не теряются ли split sides на seam/self-seam
- действительно ли child patch пишет UV в тот же quilt frame, который виден в scaffold debug
- не остаются ли внутри quilt непреднамеренно незапиненные loop corners, которые потом уводит `Conformal`

### Приоритет 2

Вычистить дубли функций в `solve.py`, чтобы остался один активный runtime path.

Без этого очень легко чинить не ту реализацию.

### Приоритет 3

Если `ScaffoldMap -> UV` начнёт совпадать визуально с debug:
- только тогда продолжать развитие solve
- а не раньше

## Минимальный тестовый набор

Использовать три типа мешей:

1. Цилиндр с caps и vertical seam на tube
- wall patch должен становиться прямоугольным
- caps не должны случайно “липнуть”, если flow их не взял в quilt

2. Cylinder.007
- важный mixed-loop кейс
- есть несколько quilts
- по нему уже видно, что `ScaffoldMap` строится, но UV всё ещё расходится

3. Простой cube
- отдельно проверять:
  - cube с seam на всех рёбрах
  - cube без seam на всех рёбрах

## Полезные кнопки/операторы для ручной проверки

- `Analyze`
- `Flow Debug`
- `Scaffold Debug`
- `Solve Phase 1 Preview`

## Короткая формулировка задачи для нового агента

Если нужно дать агенту очень короткий prompt, можно использовать это:

> Прочитай `AGENTS.md`, затем `docs/cftuv_architecture_v1.4.md`, затем `docs/cftuv_codex_handoff_scaffold_runtime.md`. Не опирайся на legacy solve как на источник логики. Текущий blocker: `Scaffold Debug` и `Flow Debug` уже согласованы, но итоговый UV runtime из `Solve Phase 1 Preview` не соответствует scaffold. Найди и исправь расхождение в `cftuv/solve.py` между `ScaffoldMap -> UV transfer` и финальным quilt-local unwrap.
