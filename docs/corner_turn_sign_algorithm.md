# Corner Turn Sign: Universal Algorithm

## Суть проблемы

`_compute_corner_turn_sign()` определяет, куда повернуть (CW / CCW) при переходе от одной chain к следующей на corner. Текущий метод:

1. Берёт 3D tangent двух chains (входящий и исходящий) в точке corner.
2. Вычисляет cross product этих tangent'ов.
3. Проецирует cross на **local normal** грани в точке corner.
4. Знак dot → знак поворота.

Проблема в шаге 3: **откуда берётся local normal**.

### Почему `mesh_tris` ломается на flat concave patches

`_serialize_patch_geometry()` делает fan-triangulation от `tri[0]` каждого face. Для concave ngon (или любого ngon > 4 вершин) получается треугольник, один из рёбер которого — внутренняя диагональ, не имеющая геометрического отношения к corner. Нормаль такого tri может быть перевёрнута (для concave) или направлена по-другому (для non-planar).

**Конкретно для П-образного patch** (или patch с holes): corner vertex попадает в fan-tri, нормаль которого computed из произвольных двух рёбер fan'а, и знак поворота инвертируется.

### Почему "face normals около chains" ломается на U-wrapped patches

Если вместо fan-tri брать **реальные face normals тех faces, которые касаются source/target chain в corner** — это работает для flat patches, потому что все faces плоского patch имеют одну и ту же нормаль.

Но для **U-shaped / wrapped** patch (например, стена, оборачивающая угол здания):
- Две chain, входящие в corner, могут лежать на гранях с **разными нормалями** (90° или больше).
- Нормаль грани, прилегающей к source chain, может не совпадать с нормалью грани, прилегающей к target chain.
- Взяв нормаль от лица source chain, вы корректно определяете поворот в плоскости source chain, но target chain уже лежит в другой плоскости — и cross product даёт неверный знак, потому что проецируется на "чужую" нормаль.

---

## Ключевое наблюдение

Поворот (CW / CCW) — это **2D-вопрос в UV-плоскости**, не 3D-вопрос. Метод через `cross.dot(normal)` пытается решить 2D-задачу через 3D, и любая нережимная нормаль ломает знак.

**Но patch уже имеет basis_u и basis_v** — локальный 2D базис, спроецированный из 3D. Этот базис:
- единый для всего patch (даже wrapped),
- не зависит от fan-triangulation,
- не зависит от того, какая грань прилегает к corner.

---

## Предлагаемый алгоритм

> **Вместо `cross(tangent_a, tangent_b).dot(local_normal)` — делать 2D cross product в пространстве `(basis_u, basis_v)` patch'а.**

### Шаги

```
1.  Получить 3D tangent source chain, направленный AWAY от corner:
      src_tangent = src_cos[1] - src_cos[0]         (если corner = start)
      src_tangent = src_cos[-2] - src_cos[-1]        (если corner = end)

2.  Получить 3D tangent target chain, направленный AWAY от corner:
      tgt_tangent = chain_cos[1] - chain_cos[0]      (если anchor = start)
      tgt_tangent = chain_cos[-2] - chain_cos[-1]     (если anchor = end)

3.  Инвертировать src_tangent → incoming = -src_tangent
      (теперь incoming указывает В corner, outgoing указывает ИЗ corner)

4.  Спроецировать оба вектора в 2D базис patch:
      in_u  = incoming.dot(node.basis_u)
      in_v  = incoming.dot(node.basis_v)
      out_u = outgoing.dot(node.basis_u)
      out_v = outgoing.dot(node.basis_v)

5.  2D pseudo cross product (z-компонента):
      cross_2d = in_u * out_v - in_v * out_u

6.  Знак:
      if abs(cross_2d) < epsilon:  return 0
      return +1 if cross_2d > 0 else -1
```

### Почему это работает для обоих кейсов

| Кейс | Почему ОК |
|------|-----------|
| **Flat patch (concave, П-образный, с holes)** | `basis_u / basis_v` = плоскость patch. Все tangent'ы лежат в этой плоскости. 2D cross даёт точный угол без зависимости от нормали конкретной грани. |
| **U-wrapped patch** | `basis_u / basis_v` = осреднённый базис patch (который строится из area-weighted нормали всех faces). Tangent'ы chain'ов проецируются в единое 2D пространство, и поворот вычисляется в этом пространстве. Даже если chains лежат на гранях с разными нормалями, проекция в общий базис даёт согласованный знак. |

### Граничный случай: wrapped patch с >90° изгибом

Если patch обёрнут на 180° (U-образный изгиб), tangent одной chain может проецироваться в `(basis_u, basis_v)` с потерей длины — но **знак cross product сохраняется**, пока проекция не вырождается (оба вектора не коллапсируют в ноль).

Вырождение возможно только если tangent chain перпендикулярен плоскости `(basis_u, basis_v)` — т.е. chain идёт вдоль нормали patch. На практике это означает, что chain идёт "внутрь стены", что невозможно для boundary edge.

### Что удаляется

- `_cf_compute_local_normal()` — больше не нужна.
- Зависимость от `node.mesh_tris` / `node.mesh_verts` из `_compute_corner_turn_sign` — удаляется.

### Что сохраняется

- Интерфейс `_compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node)` — без изменений.
- Вся обработка tangent'ов в шагах 1-2 — та же логика, что и сейчас.
- `node` всё ещё нужен для `node.basis_u` / `node.basis_v`.

---

## Итого

| Свойство | Старый (mesh_tris) | Промежуточный (face normals) | **Новый (basis 2D cross)** |
|---|---|---|---|
| Flat convex | ✅ | ✅ | ✅ |
| Flat concave / П | ❌ (fan-tri перевёрнут) | ✅ | ✅ |
| U-wrapped | ✅ | ❌ (разные face normals) | ✅ |
| Зависимость от mesh_tris | да | нет | **нет** |
| Зависимость от face lookup | нет | да | **нет** |
| Единственный источник правды | — | — | **basis_u / basis_v** |

> [!IMPORTANT]
> `basis_u / basis_v` — уже существующий, проверенный, единый базис patch, который используется повсеместно в `_cf_determine_direction`, `_segment_source_step_directions`, и guided free chain placement. Turn sign через этот же базис автоматически согласован со всей остальной placement-логикой.
