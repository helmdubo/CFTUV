# Agent Prompt — Corner Turn Sign Fix

---

Исправь функцию `_compute_corner_turn_sign()` в файле `cftuv/frontier_place.py`.

## Проблема

Текущая реализация определяет знак поворота (CW/CCW) на corner через 3D cross product, спроецированный на local normal грани. Local normal берётся из `_cf_compute_local_normal()`, которая ищет triangle из `node.mesh_tris` (fan-triangulation). Для concave ngon fan-tri может дать перевёрнутую нормаль. Замена на face normals ломает wrapped (U-shaped) patch, потому что две chain на corner могут лежать на гранях с разными нормалями.

## Решение

Заменить 3D подход на **2D pseudo cross product** в координатах `(node.basis_u, node.basis_v)`. Это единый базис patch, не зависящий ни от fan-triangulation, ни от нормалей отдельных граней.

## Что сделать

### 1. Перепиши `_compute_corner_turn_sign()` (строки ~582–618 в `cftuv/frontier_place.py`)

Сигнатура остаётся **та же самая**:
```python
def _compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node):
```

Новое тело:
```python
def _compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node):
    """Определяет знак поворота на corner из 3D tangent'ов.

    Возвращает +1 (CCW), -1 (CW), или 0 (неопределённо).
    """
    src_cos = src_chain.vert_cos
    if not src_cos or len(src_cos) < 2:
        return 0

    # Tangent of src_chain pointing AWAY from the corner
    if anchor.source_point_index == 0:
        src_tangent = src_cos[1] - src_cos[0]
    else:
        src_tangent = src_cos[-2] - src_cos[-1]

    chain_cos = chain.vert_cos
    if not chain_cos or len(chain_cos) < 2:
        return 0

    # Tangent of the new chain pointing AWAY from the corner
    if is_start_anchor:
        chain_tangent = chain_cos[1] - chain_cos[0]
    else:
        chain_tangent = chain_cos[-2] - chain_cos[-1]

    # Incoming = into corner, outgoing = out of corner
    incoming = -src_tangent
    outgoing = chain_tangent

    # Project into patch 2D basis
    in_u = incoming.dot(node.basis_u)
    in_v = incoming.dot(node.basis_v)
    out_u = outgoing.dot(node.basis_u)
    out_v = outgoing.dot(node.basis_v)

    # 2D pseudo cross product
    cross_2d = in_u * out_v - in_v * out_u

    if abs(cross_2d) < 1e-8:
        return 0
    return 1 if cross_2d > 0 else -1
```

### 2. Удали `_cf_compute_local_normal()` (строки ~562–579)

Эта функция больше не вызывается. Сначала проверь grep'ом по всему `cftuv/`, что нет других вызовов `_cf_compute_local_normal`. Если вызовов нет — удаляй. Если есть — оставь функцию, но убери вызов из `_compute_corner_turn_sign`.

### 3. Не трогай ничего другого

- НЕ меняй сигнатуру `_compute_corner_turn_sign`
- НЕ меняй `_try_inherit_direction`, `_perpendicular_direction_for_role`, `_cf_determine_direction`
- НЕ добавляй новые импорты, параметры или классы
- НЕ трогай другие файлы

## Проверка

После изменения убедись: в файле нет ни одной оставшейся ссылки на `_cf_compute_local_normal` (если удалил) и нет `mesh_tris` / `mesh_verts` в `_compute_corner_turn_sign`.
