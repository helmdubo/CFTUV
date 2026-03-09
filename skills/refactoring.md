# SKILL: Refactoring — Moving Code Between Modules

## When to use
When executing any phase from `docs/cftuv_architecture_v1.1.md`.

## Golden rule

**Одна фаза = один коммит. Не смешивать фазы.**

После каждой фазы: debug visualization должна работать.
Если debug сломался — откатить и разобраться.

## How to move a function

1. **Скопировать** функцию в целевой модуль (НЕ переписывать).
2. **Адаптировать сигнатуру** — заменить BMFace/BMEdge параметры на индексы,
   добавить PatchGraph/UVSettings где нужно.
3. **Добавить import** в целевом модуле.
4. **В старом месте** — заменить тело на вызов нового модуля (временный bridge).
5. **Проверить** что debug visualization работает.
6. **Удалить** bridge из старого кода.

Не пытаться "улучшить" функцию при переносе. Перенос и улучшение — разные коммиты.

## Module boundary checks

При каждом переносе проверяй:

```
analysis.py импортирует:
  ✅ bpy, bmesh, mathutils
  ✅ model (PatchGraph, PatchNode, enums)
  ✅ constants
  ❌ solve
  ❌ debug
  ❌ operators

solve.py импортирует:
  ✅ bpy, bmesh, mathutils
  ✅ model
  ✅ constants
  ✅ analysis.get_expanded_islands (единственная функция из analysis)
  ❌ debug
  ❌ operators

debug.py импортирует:
  ✅ bpy, mathutils
  ✅ model
  ✅ constants
  ❌ bmesh
  ❌ analysis
  ❌ solve
  ❌ operators

operators.py импортирует:
  ✅ bpy, bmesh
  ✅ model
  ✅ analysis
  ✅ solve
  ✅ debug
  ✅ constants
```

## Signature adaptation patterns

### Было (BMFace ссылки):
```python
def align_connected_islands(islands_list, links, uv_layer):
    for isl in islands_list:
        for f in isl.faces:          # BMFace — инвалидируется
            for l in f.loops:
                l[uv_layer].uv = ...
```

### Стало (индексы + bm параметр):
```python
def _align_connected(islands_data, links, uv_layer, bm):
    for isl in islands_data:
        for fi in isl['face_indices']:       # int — стабилен
            for l in bm.faces[fi].loops:     # BMFace через индекс
                l[uv_layer].uv = ...
```

### Правило: BMesh передаётся параметром, не берётся из globals

```python
# ❌ Плохо
def do_stuff():
    bm = bmesh.from_edit_mesh(bpy.context.edit_object.data)

# ✅ Хорошо
def do_stuff(bm, uv_layer, ...):
    ...
```

## Handling the BMesh rebuild problem

Если функция вызывает `bpy.ops.uv.unwrap()`, BMesh инвалидируется.
Правило: функция, которая инвалидирует BMesh, **возвращает новый BMesh**.

```python
def _unwrap_and_rebuild(bm, face_indices, context):
    """Unwrap + rebuild. Возвращает НОВЫЙ bm."""
    for f in bm.faces:
        f.select = (f.index in face_indices_set)
    bmesh.update_edit_mesh(context.edit_object.data)
    bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)

    bm = bmesh.from_edit_mesh(context.edit_object.data)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    return bm
```

Вызывающий код обязан использовать возвращённый bm:
```python
bm = _unwrap_and_rebuild(bm, indices, context)
uv_layer = bm.loops.layers.uv.verify()  # заново!
```

## Typical refactoring mistakes

- Перенести функцию но забыть обновить вызовы в других местах
- Улучшать логику одновременно с переносом (два изменения в одном коммите)
- Забыть что BMesh rebuild инвалидирует uv_layer (нужно `.verify()` заново)
- Оставить `from model import *` вместо явных импортов
- Не проверить debug visualization после переноса
