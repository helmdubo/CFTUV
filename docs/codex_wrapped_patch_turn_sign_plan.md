# CFTUV: strict local corner wedge orientation plan

## Цель

Убрать зависимость `corner turn sign` от глобального `basis_u / basis_v` и от `mesh_tris` fan triangulation.
После изменений turn sign должен корректно работать:

- на **плоских concave / П-образных патчах**;
- на **wrapped / U-shaped патчах** с wrap `> 180°`;
- без угадывания ориентации через global patch basis.

Итоговый источник истины должен быть **локальным для конкретного угла**:
`BoundaryCorner.wedge_normal`.

---

## Проблема

Сейчас `_compute_corner_turn_sign()` в `cftuv/frontier_place.py` опирается на 2D-проекцию tangent'ов в `node.basis_u / node.basis_v`.
Это ломается на wrapped патчах:

1. tangent может почти исчезать при проекции в global basis;
2. знак поворота начинает зависеть от global patch frame, а не от локальной геометрии угла;
3. на wrap `> 180°` это даёт неверный выбор направления chain placement.

Старый fallback через `mesh_tris` тоже не подходит:

- на flat concave ngons он зависит от fan triangulation;
- это нестрогий источник локальной ориентации.

**Правильный уровень решения** — topology layer. Там уже есть доступ к `bm`, к owner faces и к финальным chains/corners.

---

## Новый invariant

Для каждого `BoundaryCorner` типа `JUNCTION` должен быть предвычислен и сериализован в IR:

- `wedge_face_indices: tuple[int, ...]`
- `wedge_normal: Vector`
- `wedge_normal_valid: bool`

`wedge_normal` — это **локальная нормаль углового сектора owner patch**, вычисленная из реальных faces patch'а, которые прилегают к corner через `prev_chain` и `next_chain`.

Runtime слой (`frontier_place.py`) должен только читать это поле и считать:

```python
turn_sign = sign((incoming.cross(outgoing)).dot(corner.wedge_normal))
```

Никаких вычислений через:

- `basis_u / basis_v`
- `mesh_tris`
- global patch normal

в runtime больше быть не должно.

---

## Обязательные ограничения

### Делать

- хранить local wedge orientation в `BoundaryCorner`;
- вычислять wedge normal в topology layer, пока есть доступ к `bm`;
- использовать только реальные owner faces, а не триангуляцию для debug;
- репортить invalid wedge как invariant / diagnostic.

### Не делать

- не возвращать `mesh_tris` как fallback;
- не добавлять новый basis-based hybrid в `frontier_place.py`;
- не чинить проблему через global parity / global patch normal;
- не менять frame-role classification в этой задаче;
- не менять loop classification в этой задаче.

---

## Файлы, которые нужно изменить

### 1. `cftuv/model.py`
Добавить поля в `BoundaryCorner`.

### 2. `cftuv/analysis_boundary_loops.py`
Добавить предвычисление `corner.wedge_normal` в момент сборки topology.

### 3. `cftuv/frontier_place.py`
Переписать `_compute_corner_turn_sign()` так, чтобы он использовал только `BoundaryCorner.wedge_normal`.

### 4. Опционально: `cftuv/analysis_records.py`
Если для новых helper contracts потребуется typed payload, можно добавить private dataclass, но это не обязательно.

---

## Точный план изменений

# Шаг 1. Расширить `BoundaryCorner` в `cftuv/model.py`

Найти dataclass `BoundaryCorner` и добавить поля:

```python
wedge_face_indices: tuple[int, ...] = ()
wedge_normal: Vector = field(default_factory=lambda: Vector((0.0, 0.0, 0.0)))
wedge_normal_valid: bool = False
```

### Требования

- не ломать существующие вызовы конструктора;
- оставить старые поля без изменения;
- не вводить сюда никакой runtime-логики.

---

# Шаг 2. Добавить helper для endpoint-face в `cftuv/analysis_boundary_loops.py`

Добавить функцию:

```python
def _chain_endpoint_side_face_index(chain: BoundaryChain, at_start: bool) -> int:
    if not chain.side_face_indices:
        return -1
    return int(chain.side_face_indices[0] if at_start else chain.side_face_indices[-1])
```

### Зачем

Для `prev_chain -> corner -> next_chain` нам нужны owner-side faces именно на концах chains:

- `prev_chain` на **end**,
- `next_chain` на **start**.

---

# Шаг 3. Добавить вычисление local wedge normal в `cftuv/analysis_boundary_loops.py`

## 3.1. Создать helper `_compute_corner_wedge_data(...)`

Добавить функцию примерно такого вида:

```python
def _compute_corner_wedge_data(boundary_loop, corner, patch_face_indices, bm):
    ...
```

### Входные данные

- `boundary_loop`: финальный `BoundaryLoop`
- `corner`: конкретный `BoundaryCorner`
- `patch_face_indices`: `set[int]` owner patch faces
- `bm`: BMesh

### Алгоритм primary path

1. Проверить, что `corner.corner_kind == CornerKind.JUNCTION`.
2. Взять:
   - `prev_chain = boundary_loop.chains[corner.prev_chain_index]`
   - `next_chain = boundary_loop.chains[corner.next_chain_index]`
3. Получить owner-side face ids:
   - `prev_face = _chain_endpoint_side_face_index(prev_chain, at_start=False)`
   - `next_face = _chain_endpoint_side_face_index(next_chain, at_start=True)`
4. Собрать уникальные валидные `face_ids`, которые входят в `patch_face_indices`.
5. Если face_ids не пусты:
   - просуммировать `bm.faces[fid].normal`
   - если длина суммы > epsilon: нормализовать и вернуть как `wedge_normal`

### Важное требование

Не усреднять random tri normals. Только реальные `bm.faces[fid].normal`.

---

## 3.2. Fallback внутри topology layer

Если primary path выродился:

1. взять `corner.vert_index`;
2. собрать все `bm.verts[corner.vert_index].link_faces`, которые входят в `patch_face_indices`;
3. просуммировать их normals;
4. если длина суммы > epsilon — использовать её;
5. иначе вернуть invalid wedge.

### Почему это допустимо

Это всё ещё owner-patch local geometry.
Это не basis fallback и не triangulation fallback.

---

## 3.3. Возвращаемые данные helper'а

Функция должна вернуть три вещи:

```python
(face_indices_tuple, wedge_normal, wedge_normal_valid)
```

Где:

- `face_indices_tuple` — отсортированный tuple уникальных owner face ids;
- `wedge_normal` — `Vector`, нулевой если invalid;
- `wedge_normal_valid` — bool.

---

# Шаг 4. Аннотировать все final corners в `cftuv/analysis_boundary_loops.py`

Добавить функцию вроде:

```python
def _annotate_boundary_loop_corner_wedges(boundary_loop, patch_face_indices, bm):
    ...
```

### Что делает

Для каждого `corner` в `boundary_loop.corners`:

- если это `JUNCTION`, вычисляет wedge data;
- записывает в corner:
  - `corner.wedge_face_indices`
  - `corner.wedge_normal`
  - `corner.wedge_normal_valid`
- если это не `JUNCTION`, можно оставить invalid по умолчанию.

### Где вызывать

В `_finalize_boundary_loop_build(...)` после того, как `boundary_loop.corners` уже построены, и до финальной валидации.

Рекомендуемый порядок:

1. `boundary_loop.chains = ...`
2. `boundary_loop.corners = ...`
3. `_annotate_boundary_loop_corner_wedges(...)`
4. `_assign_loop_chain_endpoint_topology(boundary_loop)`
5. `_validate_boundary_loop_topology(...)`

Допустимо вызвать аннотацию сразу после `_assign_loop_chain_endpoint_topology`, если corner refs уже стабильны.

---

# Шаг 5. Добавить invariant checks для wedge data

В `cftuv/analysis_boundary_loops.py` внутри `_validate_boundary_loop_topology(...)` добавить новые проверки.

Для каждого `corner` типа `JUNCTION`:

- если `corner.wedge_normal_valid == False` → репортить invariant, например `R8`;
- если `corner.wedge_normal.length_squared < 1e-12` → тоже `R8`;
- если `corner.wedge_face_indices` пусты, но wedge valid — это подозрительно, тоже логировать.

### Пример сообщения

```python
_report_boundary_loop_invariant_violation(
    patch_id,
    loop_index,
    "R8",
    f"corner={corner_index} invalid_wedge_normal prev={corner.prev_chain_index} next={corner.next_chain_index}"
)
```

---

# Шаг 6. Упростить runtime в `cftuv/frontier_place.py`

## 6.1. Добавить helper `_cf_resolve_anchor_corner(...)`

Нужна функция, которая по `src_chain`, `anchor`, `node`, `loop_index` возвращает соответствующий `BoundaryCorner`.

### Логика

Если `anchor.source_point_index == 0`:
- взять `src_chain.start_corner_index`

Иначе:
- взять `src_chain.end_corner_index`

Потом:
- взять `boundary_loop = node.boundary_loops[anchor.source_ref[1]]`
- вернуть `boundary_loop.corners[corner_index]`, если индекс валиден

### Важно

Не угадывать corner через координаты. Только через topology links.

---

## 6.2. Полностью переписать `_compute_corner_turn_sign()`

Текущая basis-based версия должна быть удалена.

### Новый алгоритм

1. Получить `src_tangent` и `chain_tangent` ровно как сейчас.
2. Сформировать:

```python
incoming = -src_tangent
outgoing = chain_tangent
```

3. Получить `corner = _cf_resolve_anchor_corner(...)`.
4. Если `corner is None` → вернуть `0`.
5. Если `not corner.wedge_normal_valid` → вернуть `0`.
6. Если `corner.wedge_normal.length_squared < 1e-12` → вернуть `0`.
7. Посчитать:

```python
cross_vec = incoming.cross(outgoing)
if cross_vec.length_squared < 1e-12:
    return 0

dot_val = cross_vec.dot(corner.wedge_normal)
if abs(dot_val) < 1e-8:
    return 0
return 1 if dot_val > 0.0 else -1
```

### Строгий запрет

В новой реализации `_compute_corner_turn_sign()` не должно быть:

- `basis_u`
- `basis_v`
- `cross_2d`
- `mesh_tris`
- `_cf_compute_local_normal(...)` на базе tri fan

Если старый helper для tri normal останется в файле ради отладки — он не должен больше использоваться этим кодом.

---

# Шаг 7. Обновить docstrings и комментарии

## В `frontier_place.py`

Обновить docstring `_compute_corner_turn_sign()`.

Новая формулировка должна говорить:

- turn sign — это local wedge-space решение;
- источник истины — `BoundaryCorner.wedge_normal`;
- normal вычисляется upstream в topology layer.

### Убрать старые утверждения

Нельзя оставлять комментарии про:

- `2D cross in patch basis`
- `works for wrapped patches through basis projection`

Это больше не соответствует реальной логике.

---

## Псевдокод финальной архитектуры

### Topology layer

```python
for each boundary_loop:
    build chains
    build corners
    for each junction corner:
        prev_face = prev_chain.side_face_indices[-1]
        next_face = next_chain.side_face_indices[0]
        wedge_normal = normalize(sum(owner endpoint face normals))
        if degenerate:
            wedge_normal = normalize(sum(owner one-ring face normals around corner vertex))
        if still degenerate:
            corner.wedge_normal_valid = False
        else:
            corner.wedge_normal_valid = True
            corner.wedge_normal = wedge_normal
```

### Runtime layer

```python
corner = resolve_anchor_corner(...)
if no valid wedge_normal:
    return 0
return sign((incoming × outgoing) · corner.wedge_normal)
```

---

## Acceptance criteria

### Кодовые

- `BoundaryCorner` хранит локальную wedge normal.
- `analysis_boundary_loops.py` предвычисляет её при сборке topology.
- `frontier_place._compute_corner_turn_sign()` использует только `corner.wedge_normal`.
- В runtime больше нет basis-based turn sign logic.

### Поведенческие

- flat concave / П-shaped patches не ломаются;
- wrapped patches `> 180°` не зависят от global basis orientation;
- исчезает basis projection collapse как причина неверного turn sign;
- `Main_wall.009` должен перестать давать row scatter, вызванный неправильным `corner turn inheritance`.

### Диагностические

- invalid wedge normal репортится как topology invariant, а не скрывается fallback'ом;
- в логах можно увидеть, у каких corner'ов wedge invalid.

---

## Чеклист выполнения

- [ ] Добавить поля `wedge_face_indices`, `wedge_normal`, `wedge_normal_valid` в `BoundaryCorner`
- [ ] Добавить `_chain_endpoint_side_face_index()`
- [ ] Добавить `_compute_corner_wedge_data()`
- [ ] Добавить `_annotate_boundary_loop_corner_wedges()`
- [ ] Вызывать аннотацию wedge data во время final boundary loop build
- [ ] Добавить validation / invariant `R8` для invalid wedge normals
- [ ] Добавить `_cf_resolve_anchor_corner()` в `frontier_place.py`
- [ ] Полностью переписать `_compute_corner_turn_sign()` на `corner.wedge_normal`
- [ ] Удалить basis-based turn sign path из runtime
- [ ] Обновить docstrings и комментарии

---

## Первое сообщение для Codex

Скопируй и отправь агенту это сообщение первым:

```text
Нужно реализовать strict local corner wedge orientation для CFTUV.

Задача: убрать зависимость corner turn sign от global patch basis и от mesh_tris. Источник истины должен стать локальным для каждого BoundaryCorner.

Что нужно сделать:
1. В cftuv/model.py расширить BoundaryCorner полями:
   - wedge_face_indices: tuple[int, ...] = ()
   - wedge_normal: Vector = field(default_factory=lambda: Vector((0.0, 0.0, 0.0)))
   - wedge_normal_valid: bool = False

2. В cftuv/analysis_boundary_loops.py:
   - добавить helper _chain_endpoint_side_face_index(chain, at_start)
   - добавить helper _compute_corner_wedge_data(boundary_loop, corner, patch_face_indices, bm)
   - primary path: wedge normal = normalized sum of owner endpoint face normals:
       prev_chain.side_face_indices[-1] + next_chain.side_face_indices[0]
   - fallback: owner-patch one-ring face normals around corner vertex
   - добавить _annotate_boundary_loop_corner_wedges(boundary_loop, patch_face_indices, bm)
   - вызывать это во время final loop build, когда финальные corners уже созданы
   - добавить invariant/diagnostic R8 для invalid wedge normals

3. В cftuv/frontier_place.py:
   - добавить helper _cf_resolve_anchor_corner(src_chain, anchor, node)
   - полностью переписать _compute_corner_turn_sign()
   - новый алгоритм:
       incoming = -src_tangent
       outgoing = chain_tangent
       corner = resolved anchor corner
       if no valid corner.wedge_normal -> return 0
       return sign((incoming.cross(outgoing)).dot(corner.wedge_normal))
   - удалить basis_u / basis_v logic from turn sign
   - не использовать mesh_tris fallback

Ограничения:
- не менять frame-role classification
- не менять loop classification
- не делать новый hybrid через basis
- не использовать fan triangulation normals для final runtime logic

Сначала изучи текущие файлы:
- cftuv/model.py
- cftuv/analysis_boundary_loops.py
- cftuv/frontier_place.py

Потом внеси изменения минимально инвазивно и покажи diff по этим файлам.
```

---

## Примечание

Если по ходу реализации окажется, что `BoundaryCorner` нужен ещё один служебный флаг или diagnostic field — это допустимо, но только если он напрямую служит local wedge orientation и не раздувает runtime layer.
