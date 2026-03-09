# CLAUDE.md — Project Context for AI Agents

## What is this project

CFTUV (Constraint-First Trim UV) — Blender addon для полупроцедурной UV развёртки
архитектурных ассетов (hard-surface environment geometry). Пользователь размечает
mesh в 3D Viewer (seams, face selection; sharp edges только для shading), система строит граф 3D patches
и автоматически генерирует UV layout по правилам стыковки.

Target: AA-AAA game production, trim sheet / tile workflow.
Blender: 3.0+, Python 3.10+.
Single developer, in-house studio tool.

## Architecture

Прочитай `docs/cftuv_architecture_v1.1.md` перед любой работой. Это главный документ.

Система разделена на 6 модулей:

| Модуль | Роль | Читает | Пишет |
|--------|------|--------|-------|
| model.py | Структуры данных | — | — |
| analysis.py | BMesh → PatchGraph | BMesh | PatchGraph |
| solve.py | UV операции | PatchGraph + BMesh UV | UV layer |
| debug.py | Визуализация | PatchGraph | Grease Pencil |
| operators.py | Blender UI обёртки | всё | — |
| constants.py | Конфигурация | — | — |

**Центральный IR — PatchGraph.** Все данные между модулями передаются через него.

## Invariants — нарушение любого = баг

1. `model.py` НЕ импортирует `bpy`, `bmesh` (только `mathutils`)
2. `analysis.py` НЕ пишет UV (кроме `_classify_loops_outer_hole` — помечен явно)
3. `solve.py` НЕ делает flood fill и НЕ классифицирует патчи
4. `debug.py` НЕ читает BMesh напрямую
5. Операторы НЕ содержат геометрической логики (max 5 строк math)
6. Нет глобальных мутабельных переменных — UVSettings передаётся параметром
7. PatchGraph хранит индексы (int), НЕ ссылки на BMFace/BMEdge

## Domain terminology

- **Patch** — набор faces по flood fill, ограниченный seam/boundary edges
- **PatchType** — dispatch key для UV стратегии: WALL, FLOOR, SLOPE
  (НЕ semantic description, а именно "какой алгоритм unwrap применять")
- **Boundary Loop** — замкнутый контур boundary edges одного patch (OUTER или HOLE)
- **Boundary Chain** — сегмент boundary loop с одним соседом (split point = смена соседа)
- **FrameRole** — роль сегмента boundary: H_FRAME, V_FRAME, FREE
- **SeamEdge** — связь между двумя соседними patches через общий шов
- **Basis** — локальная система координат patch (basis_u = tangent, basis_v = bitangent)
- **Core faces** — faces выделенные пользователем
- **Full faces** — core + expanded до seam boundary (весь patch)
- **Two-Pass Unwrap** — unwrap cores → pin → unwrap full → dock → align
- **Sharp Edge** — источник shading only; не участвует в patch split и не создаёт seam links

## Code conventions

- Язык комментариев: русский (это внутренний инструмент студии)
- Docstrings: русский или английский, без разницы
- Naming: snake_case для функций, PascalCase для классов, UPPER_CASE для констант
- Enum values: UPPER_CASE строки (`PatchType.WALL`, `FrameRole.H_FRAME`)
- Private functions: prefix `_` (не `__`)
- Типы: dataclass для данных, обычные классы для операторов
- Нет third-party зависимостей кроме Blender built-in (bpy, bmesh, mathutils)

## What NOT to do

- НЕ добавлять multi-axis semantic profiles (confidence scores, role_class, etc.)
- НЕ создавать отдельный Boundary Graph — chains это подструктура PatchNode
- НЕ создавать constraint classes — правила стыковки это код в solve.py
- НЕ разбивать solve.py на подмодули (пока < 600 строк)
- НЕ делать debug geometry optional/lazy — debug всегда включён
- НЕ использовать globals для настроек — передавать UVSettings параметром
- НЕ хранить BMFace/BMEdge ссылки в model — только индексы

## Testing approach

Формальных тестов нет. Верификация через:
1. Debug visualization (Grease Pencil) — включается кнопкой Analyze в панели
2. Console output (patch stats, chain info, frame roles)
3. Ручная проверка UV в UV Editor на production meshes

При рефакторинге: debug visualization должна работать на КАЖДОЙ фазе.
Если debug сломался — рефакторинг неправильный.

## Refactoring state

Текущая фаза: см. `docs/cftuv_architecture_v1.1.md`, раздел "План рефакторинга".
Каждая фаза = отдельный коммит. Не смешивать фазы.
