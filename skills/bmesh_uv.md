# SKILL: Blender BMesh & UV Operations

## When to use
Any task involving reading mesh geometry, UV layers, or writing UV coordinates.
Most work in analysis.py and solve.py touches BMesh.

## BMesh lifecycle (CRITICAL)

```python
# READING in Edit Mode
bm = bmesh.from_edit_mesh(obj.data)
bm.faces.ensure_lookup_table()   # ОБЯЗАТЕЛЬНО перед bm.faces[index]
bm.verts.ensure_lookup_table()
bm.edges.ensure_lookup_table()

# После ЛЮБОГО bpy.ops.uv.unwrap() или bmesh.update_edit_mesh():
# ВСЕ ссылки на BMFace/BMEdge/BMVert ИНВАЛИДИРОВАНЫ.
# Нужно заново:
bm = bmesh.from_edit_mesh(obj.data)
bm.faces.ensure_lookup_table()
# ... и заново получать объекты через bm.faces[index]
```

**Почему PatchGraph хранит индексы, а не ссылки:**
BMFace/BMEdge объекты умирают при bmesh.update_edit_mesh() + bmesh.from_edit_mesh().
`bm.faces[42]` после rebuild — новый объект. Индекс 42 — стабилен.

## UV layer access

```python
uv_layer = bm.loops.layers.uv.verify()  # создаёт если нет
# или
uv_layer = bm.loops.layers.uv.active

# Чтение/запись UV
for face in bm.faces:
    for loop in face.loops:
        uv = loop[uv_layer].uv          # Vector((u, v))
        loop[uv_layer].uv = new_uv      # запись
        loop[uv_layer].pin_uv = True     # pin для unwrap
```

## Conformal unwrap через оператор

```python
# ОБЯЗАТЕЛЬНО: выделить нужные faces перед вызовом
for f in bm.faces:
    f.select = (f.index in target_indices)
bmesh.update_edit_mesh(obj.data)         # синхронизировать с Blender

bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)

# ПОСЛЕ: BMesh инвалидирован, rebuild обязателен
bm = bmesh.from_edit_mesh(obj.data)
bm.faces.ensure_lookup_table()
uv_layer = bm.loops.layers.uv.verify()
```

## Seam/Sharp edges

```python
edge.seam       # bool — UV seam
edge.smooth     # bool — True = smooth, False = sharp (инвертированная логика!)
# sharp edge: not edge.smooth
# В проекте: seam и sharp edges оба являются границами patches
```

## Типичные ошибки

- Забыть `ensure_lookup_table()` → crash при доступе по индексу
- Использовать BMFace ссылку после `update_edit_mesh()` → crash или мусорные данные
- Забыть `bmesh.update_edit_mesh()` перед `bpy.ops` → оператор работает со старыми данными
- Проверить `edge.smooth` вместо `not edge.smooth` для sharp (инвертированная семантика)
