# Phase 2 — Инструкция по патчу solve.py

## Две правки. Ничего больше.

---

## Правка 1: Вставить функцию

Открой `cftuv/solve.py`.

Найди строку (примерно в конце файла, можно через Ctrl+F):

```python
def execute_phase1_preview(context, obj, bm, patch_graph: PatchGraph, settings, solve_plan: Optional[SolvePlan] = None) -> dict[str, int]:
```

ПЕРЕД этой строкой вставь ВЕСЬ код из файла `phase2_validation_patch.py`,
начиная со строки `def validate_scaffold_uv_transfer(` и до конца файла.

Между вставленным кодом и `def execute_phase1_preview` должна быть пустая строка.

ДО:
```python
    ...какой-то код выше...


def execute_phase1_preview(context, obj, bm, patch_graph...
```

ПОСЛЕ:
```python
    ...какой-то код выше...


def validate_scaffold_uv_transfer(bm, graph, uv_layer, patch_placement, uv_offset, epsilon=1e-4):
    """Phase 2: Diagnostic — проверяет что scaffold points корректно записаны в UV."""
    ...весь код функции...


def execute_phase1_preview(context, obj, bm, patch_graph...
```

---

## Правка 2: Добавить один вызов

Внутри функции `execute_phase1_preview`, найди эти две строки:

```python
            patch_stats = _apply_patch_scaffold_to_uv(bm, patch_graph, uv_layer, patch_placement, uv_offset)
            _print_phase1_preview_patch_report(quilt_scaffold.quilt_index, patch_id, patch_stats)
```

МЕЖДУ ними вставь одну строку:

```python
            validate_scaffold_uv_transfer(bm, patch_graph, uv_layer, patch_placement, uv_offset)
```

РЕЗУЛЬТАТ:
```python
            patch_stats = _apply_patch_scaffold_to_uv(bm, patch_graph, uv_layer, patch_placement, uv_offset)
            validate_scaffold_uv_transfer(bm, patch_graph, uv_layer, patch_placement, uv_offset)
            _print_phase1_preview_patch_report(quilt_scaffold.quilt_index, patch_id, patch_stats)
```

---

## Проверка

1. Сохрани solve.py
2. Перезапусти Blender
3. Выдели faces на mesh → нажми "Solve Phase 1 Preview"
4. Открой System Console (Window → Toggle System Console)
5. Ищи строки вида:
   - `[CFTUV][Validate] Patch 0: OK (42/42 points verified)` — всё хорошо
   - `[CFTUV][Validate] Patch 0: 3 mismatches` — есть проблемы (будут конкретные строки ниже)

---

## Что означают ошибки

- `MISMATCH` — scaffold записал UV, но в BMesh оказалось другое значение.
  Показывает expected vs actual координаты и расстояние.

- `UNRESOLVED` — scaffold point не смог найти UV loop для записи.
  Проблема в _resolve_scaffold_uv_targets().

- `VERT_NOT_IN_FACE` — vert_index не найден среди loops face.
  Проблема в side_face_indices.

- `MISSING_FACE` — face_index за пределами mesh.
  Проблема в boundary loop serialization.

- `collapsed SEAM_SELF verts` — вершина на SEAM_SELF получила одинаковый UV
  на обеих сторонах шва. Должна была получить разные.
