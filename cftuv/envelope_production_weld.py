"""Сварка вершин соседних доменов продуктового меша: по семантической ссылке, митра смещения.

АДАПТЕР ОТОБРАЖАЕТ КОНТРАКТ (AGENTS.md): позиции, UV, владельцы и цепи приходят из батчей, а
здесь решаются ровно две вещи политики отображения, обе — над готовыми числами ядра, и ни одна не
чинит геометрию.

СВАРКА. Батч называет вершину исходника ссылкой `semantic_location_ref = location:src:<id>`: она
ГЛОБАЛЬНА (одна на вершину во всех доменах), а `location:node:<k>` — доменная и не сваривается
никогда. Вершины разных доменов с одной ссылкой становятся ОДНОЙ вершиной меша — но только если их
позиции в батчах ПОБИТОВО равны (закон ядра `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` кладёт
вершину в одну и ту же позицию хоста). Это ПРОВЕРКА, а не ремонт: позиции не сходятся — вершины
остаются раздельными, ссылка считается в `ADAPTER_WELD_POSITION_MISMATCH` (число групп и наибольшее
расхождение), и ничто не подтягивается друг к другу. По расстоянию вершины не сливаются нигде.

МИТРА. Смещение декали над поверхностью (z-fighting) — политика хоста. У вершины одного домена оно
идёт вдоль её нормали; у общей вершины k доменов — в точку ПЕРЕСЕЧЕНИЯ k сдвинутых плоскостей, чтобы
каждая сваренная грань осталась на сдвинутой плоскости СВОЕГО домена (плоская грань остаётся
плоской): `n_i . o = d` для всех i. k = 1 — обычное смещение `d n`; k = 2 — `d (n1 + n2) /
(1 + n1 . n2)`; k = 3 — единственное решение; больше — решение, если система совместна. Вершины
домена-развёртки приносят СВОИ нормали смещения (закон ядра `SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`).
Равные нормали (компланарные соседи) смещаются ровно как одиночная вершина: побитово прежний ответ.

ОТКАЗ ОТ СВАРКИ ИМЕНОВАН. Нормали почти противоположны (митра улетает: длина `d / cos(угла / 2)`
выше предела), вырождены (три плоскости без общей точки), либо система несовместна — вершины
остаются раздельными, каждая со смещением по своей нормали, и это `ADAPTER_WELD_MITER_FALLBACK`
(число групп, причина и худший множитель митры). Предел митры и допуск вырожденности — решения
хоста, объявленные ниже и записанные в исходе; они меняют ответ (сваривать или нет), поэтому ни
одно срабатывание не молчит.

ШВЫ UV. UV остаётся по петле: у вершины, сваренной из двух доменов, UV граней разные. Ребро, у
которого две грани РАЗНЫХ доменов и UV хоть на одном конце различаются, — разрыв UV на складке, и
оно помечается швом (`ADAPTER_WELD_SEAMS_MARKED`): раньше эту роль играла открытая граница.

ПЛОСКОСТЬ ПОСЛЕ СМЕЩЕНИЯ. Грань от четырёх вершин, плоская в батче (закон ядра
`SOURCE_TRIANGLES_CLIPPED_V1`: кусок лежит в одном треугольнике источника), смещается вдоль нормалей
СВОИХ вершин, и у домена развёртки они разные: смещённая грань перестаёт быть плоской. Хост этого не
чинит и не режет молча: отклонение вершин от плоскости их грани ПОСЛЕ смещения измеряется
(`off_plane_after_offset`) и пишется в квитанцию (`ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES`).
Порога у числа нет — решает владелец глазами (прецедент: `DEVELOPABLE_OFFSET_MIN_GAP_COSINE` ядра).
Закон по граням (`SOURCE_FACES_CLIPPED_V1`, политика кнопки) к этому числу добавляет глубину хорды: кусок
поперёк диагонали непланарной грани источника уже в батче не плоский, на величину до допуска ядра
(четверть смещения, 5 мм; `CLIP_DIAGONAL_CHORD_BUDGET`), и это отклонение входит в то же число.

ШОВ БЕЗ T-СТЫКОВ. Соседние домены делят цепь источника, и сваривает их только `location:src:`: вершины
между двумя `src:` одной цепи (`node:`, `clip:`) в разные домены не сливаются и T-стыков хост не считает
(`half_edge_conflicts` видит лишь повторные полурёбра). Поэтому шов проверяется ПО ЦЕПЯМ БАТЧЕЙ, точно:
у каждой пары соседних `src:` на цепи `boundary:SOURCE` берётся число вершин между ними в каждом домене, и
пара, у которой оно в двух доменах различно, — T-стык (`seam_report`; запись, а не ремонт: вершины не
подтягиваются). Туда же — пара соседних `src:` одного домена, между которыми у другого домена той же цепи стоит ещё `src:`: так
выглядит точка, растворённая лишь с одной стороны (закон ядра `SILHOUETTE_SOURCE_DOTS_V1` решает место сразу во всех доменах,
и эта проверка его страхует). Вершина `clip:` на цепи источника или стены — свой счёт: закон ядра их там не допускает.

ПОРЯДОК. Вершина меша получает номер первого вхождения при обходе доменов по номеру патча и вершин
по ключу, поэтому нумерация не зависит ни от воркера, ни от порядка множеств батча.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: Только ссылки вершин ИСХОДНИКА сваривают: ссылка узла (`location:node:<k>`) принадлежит домену.
WELD_LOCATION_PREFIX = "location:src:"

OUTCOME_WELD_POSITION_MISMATCH = "ADAPTER_WELD_POSITION_MISMATCH"
OUTCOME_WELD_MITER_FALLBACK = "ADAPTER_WELD_MITER_FALLBACK"
OUTCOME_WELD_HALF_EDGE_CONFLICT = "ADAPTER_WELD_HALF_EDGE_CONFLICT"
COUNTER_WELD_GROUPS = "ADAPTER_WELD_GROUPS"
COUNTER_WELD_VERTICES_MERGED = "ADAPTER_WELD_VERTICES_MERGED"
COUNTER_WELD_SEAMS_MARKED = "ADAPTER_WELD_SEAMS_MARKED"
#: Числа плоскости граней после смещения: наибольшее отклонение вершины от плоскости своей грани (нм) и
#: число граней от четырёх вершин с отклонением больше нуля.
COUNTER_MAX_OFF_PLANE_AFTER_OFFSET = "ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES"
COUNTER_FACES_OFF_PLANE_AFTER_OFFSET = "ADAPTER_FACES_OFF_PLANE_AFTER_OFFSET"
NANOMETRES_PER_METRE = 10**9
#: Шов по цепям батчей (см. `seam_report`): пары соседних `src:` цепи источника с РАЗНЫМ числом вершин между
#: ними в двух доменах (T-стык шва) и вершины `clip:` на цепях источника и стены.
COUNTER_SEAM_T_JUNCTIONS = "ADAPTER_SEAM_T_JUNCTIONS"
COUNTER_SEAM_CLIP_VERTICES = "ADAPTER_SEAM_CLIP_VERTICES"
OUTCOME_SEAM_T_JUNCTIONS = "ADAPTER_SEAM_T_JUNCTIONS"

#: Предел митры: длина смещения общей вершины не больше `MITER_LIMIT` смещений одиночной (для двух
#: доменов это `1 / cos(угла между нормалями / 2) <= 4`, угол до ~151 градуса). Острее — вершины
#: остаются раздельными (`ADAPTER_WELD_MITER_FALLBACK`): митра ножевого ребра улетела бы за декаль.
MITER_LIMIT = 4.0
#: Допуск вырожденности системы митры: ведущий элемент Гаусса не больше `MITER_PIVOT_EPSILON` —
#: нормали линейно зависимы; несовместность и невязка — не больше `MITER_RESIDUAL_EPSILON`.
MITER_PIVOT_EPSILON = 1e-9
MITER_RESIDUAL_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class DomainVerticesV1:
    """Вершины одного домена в порядке ключей: позиция батча, нормаль смещения, ссылка (либо `None`)."""

    patch_id: int
    positions: tuple
    normals: tuple
    refs: tuple


@dataclass(frozen=True, slots=True)
class WeldV1:
    """Итог сварки: позиции вершин меша со смещением, номера вершин каждого домена и числа."""

    positions: tuple
    #: По домену: номер вершины меша для каждой его локальной вершины.
    index: tuple
    counters: tuple
    #: `(None, исход, деталь)`: находки про весь меш (в предупреждения квитанции).
    warnings: tuple


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _eliminate(rows, size):
    """Метод Гаусса — Жордана с выбором ведущего: `(номера ведущих столбцов, строки)`."""

    pivots = []
    row = 0
    for column in range(size):
        if row == size:
            break
        best = max(range(row, size), key=lambda index: abs(rows[index][column]))
        if abs(rows[best][column]) <= MITER_PIVOT_EPSILON:
            continue
        rows[row], rows[best] = rows[best], rows[row]
        scale = rows[row][column]
        rows[row] = [value / scale for value in rows[row]]
        for other in range(size):
            if other != row and rows[other][column]:
                factor = rows[other][column]
                rows[other] = [a - factor * b for a, b in zip(rows[other], rows[row])]
        pivots.append(column)
        row += 1
    return pivots, rows


def miter_offset(normals):
    """`(вектор смещения при d = 1, множитель митры, причина)`: `n_i . o = 1` для всех нормалей.

    `o` ищется в линейной оболочке нормалей (`o = sum a_i n_i`, `G a = 1`, `G` — матрица Грама), и
    она единственна, когда система совместна. Равные побитово нормали дают сразу `n`. Отказ —
    `(None, множитель | None, причина)`: система несовместна (почти противоположные либо
    несходящиеся плоскости) либо множитель митры выше `MITER_LIMIT`.
    """

    first = normals[0]
    if all(item == first for item in normals):
        return first, 1.0, ""
    size = len(normals)
    rows = [[_dot(a, b) for b in normals] + [1.0] for a in normals]
    pivots, rows = _eliminate(rows, size)
    for row in range(len(pivots), size):
        if abs(rows[row][size]) > MITER_RESIDUAL_EPSILON:
            return None, None, "the offset planes do not meet in one point"
    weights = [0.0] * size
    for row, column in enumerate(pivots):
        weights[column] = rows[row][size]
    offset = tuple(
        sum(weight * normal[axis] for weight, normal in zip(weights, normals))
        for axis in range(3)
    )
    if any(abs(_dot(normal, offset) - 1.0) > MITER_RESIDUAL_EPSILON for normal in normals):
        return None, None, "the offset planes do not meet in one point"
    factor = math.sqrt(_dot(offset, offset))
    if factor > MITER_LIMIT:
        return None, factor, "the miter exceeds the limit"
    return offset, factor, ""


def _hex(point) -> tuple:
    return tuple(float(axis).hex() for axis in point)


def _groups_by_reference(domains):
    found: dict = {}
    for ordinal, domain in enumerate(domains):
        for local, ref in enumerate(domain.refs):
            if ref is not None and ref.startswith(WELD_LOCATION_PREFIX):
                found.setdefault(ref, []).append((ordinal, local))
    return {ref: members for ref, members in found.items() if len(members) > 1}


def _plan(domains):
    """`(класс сварки по (домен, вершина), множитель смещения класса, числа, находки)`."""

    classes: dict = {}
    miters: dict = {}
    mismatches = fallbacks = 0
    worst_gap = 0.0
    worst_factor = 0.0
    reasons: set = set()
    for members in _groups_by_reference(domains).values():
        by_position: dict = {}
        for ordinal, local in members:
            point = domains[ordinal].positions[local]
            by_position.setdefault(_hex(point), []).append((ordinal, local))
        if len(by_position) > 1:
            mismatches += 1
            points = [domains[o].positions[l] for o, l in (cls[0] for cls in by_position.values())]
            worst_gap = max(
                worst_gap, max(math.dist(a, b) for a in points for b in points)
            )
        for cls in by_position.values():
            if len(cls) < 2:
                continue
            offset, factor, reason = miter_offset(
                [domains[o].normals[l] for o, l in cls]
            )
            if offset is None:
                fallbacks += 1
                reasons.add(reason)
                worst_factor = max(worst_factor, factor or 0.0)
                continue
            for member in cls:
                classes[member] = cls[0]
            miters[cls[0]] = offset
    return classes, miters, (mismatches, worst_gap, fallbacks, worst_factor, reasons)


def weld_vertices(domains, offset: float) -> WeldV1:
    """Вершины меша из вершин доменов: сварка по ссылке при равных позициях, митра смещения."""

    classes, miters, (mismatches, gap, fallbacks, factor, reasons) = _plan(domains)
    positions: list = []
    placed: dict = {}
    index = []
    for ordinal, domain in enumerate(domains):
        mapping = []
        for local, point in enumerate(domain.positions):
            head = classes.get((ordinal, local))
            if head is not None and head in placed:
                mapping.append(placed[head])
                continue
            shift = miters[head] if head is not None else domain.normals[local]
            positions.append(
                (
                    point[0] + offset * shift[0],
                    point[1] + offset * shift[1],
                    point[2] + offset * shift[2],
                )
            )
            mapping.append(len(positions) - 1)
            if head is not None:
                placed[head] = len(positions) - 1
        index.append(tuple(mapping))
    merged = len(classes) - len(miters)
    warnings = []
    if mismatches:
        warnings.append(
            (
                None,
                OUTCOME_WELD_POSITION_MISMATCH,
                f"{mismatches} shared source vertices have positions that are not bitwise "
                f"equal across domains and stay separate (largest gap {gap:.6g} m)",
            )
        )
    if fallbacks:
        warnings.append(
            (
                None,
                OUTCOME_WELD_MITER_FALLBACK,
                f"{fallbacks} shared source vertices stay separate with their own offsets: "
                f"{'; '.join(sorted(reasons))} (worst miter factor {factor:.6g}, "
                f"limit {MITER_LIMIT:g})",
            )
        )
    return WeldV1(
        positions=tuple(positions),
        index=tuple(index),
        counters=(
            (COUNTER_WELD_GROUPS, len(miters)),
            (COUNTER_WELD_VERTICES_MERGED, merged),
            (OUTCOME_WELD_POSITION_MISMATCH, mismatches),
            (OUTCOME_WELD_MITER_FALLBACK, fallbacks),
        ),
        warnings=tuple(warnings),
    )


def _anchors_between(anchored) -> int:
    """Пары соседних `src:` одного домена, между которыми у другого домена в той же цепи стоит ещё `src:` (вершина растворена лишь с одной стороны).

    Счёт по числу вершин `node:`/`clip:` между соседними `src:` такого шва не видит: пара, соседняя у одного домена, у другого
    не пара вовсе. Каждая такая пара — T-стык: вершина другого домена лежит на ребре этого.
    """

    where: dict = {}
    consecutive: dict = {}
    for number, chain_id, anchors in anchored:
        consecutive[(number, chain_id)] = {frozenset(pair) for pair in zip(anchors, anchors[1:])}
        for key in anchors:
            where.setdefault((number, key), chain_id)
    numbers = {number for number, _chain_id, _anchors in anchored}
    open_pairs = set()
    for (number, _chain_id), pairs in consecutive.items():
        for pair in pairs:
            if len(pair) != 2:
                continue
            first, second = sorted(pair)
            for other in numbers - {number}:
                chain = where.get((other, first))
                if chain is not None and chain == where.get((other, second)) and pair not in consecutive[(other, chain)]:
                    open_pairs.add(pair)
    return len(open_pairs)


def seam_report(batches) -> tuple:
    """`((имя, число), ...)`: T-стыки шва между доменами и вершины `clip:` на шовных цепях, по цепям батчей.

    Шовные цепи — `boundary:SOURCE:*` и `boundary:WALL:*` (граница домена вдоль контура патча). Батч без
    `boundary_chains` ничего не даёт. Точно, без допусков: ключи вершин, а не координаты.
    """

    by_pair: dict = {}
    clip_vertices = 0
    anchored: list = []
    for number, batch in enumerate(batches):
        for chain in getattr(batch, "boundary_chains", ()) or ():
            kind = chain.semantic_boundary_id.value.split(":")[1]
            if kind not in ("SOURCE", "WALL"):
                continue
            keys = [item.value for item in chain.ordered_vert_keys]
            anchors = [index for index, key in enumerate(keys) if key.startswith("src:")]
            clip_vertices += sum(1 for key in keys if key.startswith("clip:"))
            if kind != "SOURCE":
                continue
            anchored.append((number, chain.semantic_boundary_id.value, [keys[index] for index in anchors]))
            for first, second in zip(anchors, anchors[1:]):
                by_pair.setdefault(frozenset((keys[first], keys[second])), []).append(second - first - 1)
    junctions = sum(1 for counts in by_pair.values() if len(counts) > 1 and len(set(counts)) > 1)
    junctions += _anchors_between(anchored)
    return (
        (COUNTER_SEAM_T_JUNCTIONS, junctions),
        (COUNTER_SEAM_CLIP_VERTICES, clip_vertices),
    )


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def off_plane_after_offset(positions, faces) -> tuple:
    """`((имя, число), ...)`: плоскость граней от четырёх вершин ПОСЛЕ смещения над поверхностью.

    Плоскость грани — через её первую вершину с нормалью вектора площади (веер из первой вершины);
    мера — наибольшее расстояние вершины грани от неё, метры. Грань нулевой площади ничего не меряет.
    Возвращает наибольшее отклонение в нанометрах и число граней, у которых оно не меньше нанометра.
    """

    worst, off = 0.0, 0
    for loop in faces:
        if len(loop) < 4:
            continue
        points = [positions[index] for index in loop]
        origin = points[0]
        normal = (0.0, 0.0, 0.0)
        for index in range(1, len(points) - 1):
            part = _cross(
                tuple(b - a for a, b in zip(origin, points[index])),
                tuple(b - a for a, b in zip(origin, points[index + 1])),
            )
            normal = tuple(a + b for a, b in zip(normal, part))
        length = math.sqrt(_dot(normal, normal))
        if not length:
            continue
        deviation = max(
            abs(_dot(tuple(b - a for a, b in zip(origin, point)), normal)) / length
            for point in points[1:]
        )
        worst = max(worst, deviation)
        # Разрешение записи — нанометр: шум округления binary64 (1e-16 м) гранью «вне плоскости» не числится.
        off += int(round(deviation * NANOMETRES_PER_METRE) > 0)
    return (
        (COUNTER_MAX_OFF_PLANE_AFTER_OFFSET, round(worst * NANOMETRES_PER_METRE)),
        (COUNTER_FACES_OFF_PLANE_AFTER_OFFSET, off),
    )


def _edge_uvs(faces, uvs, face_domain):
    """`{ребро: [(грань, домен, uv у меньшей вершины, uv у большей)]}` по петлям меша."""

    edges: dict = {}
    cursor = 0
    for number, loop in enumerate(faces):
        size = len(loop)
        for position in range(size):
            first, second = loop[position], loop[(position + 1) % size]
            uv_first = uvs[cursor + position]
            uv_second = uvs[cursor + (position + 1) % size]
            if first > second:
                first, second, uv_first, uv_second = second, first, uv_second, uv_first
            edges.setdefault((first, second), []).append(
                (number, face_domain[number], uv_first, uv_second)
            )
        cursor += size
    return edges


def cross_domain_seams(faces, uvs, face_domain) -> tuple:
    """Рёбра меша между гранями РАЗНЫХ доменов, где UV разрывен: шов на сваренной складке."""

    return tuple(
        sorted(
            edge
            for edge, sides in _edge_uvs(faces, uvs, face_domain).items()
            if len(sides) == 2
            and sides[0][1] != sides[1][1]
            and (sides[0][2] != sides[1][2] or sides[0][3] != sides[1][3])
        )
    )


def half_edge_conflicts(faces) -> int:
    """Направленные рёбра, лежащие в двух гранях: так сваренные соседи с разным обходом."""

    seen: dict = {}
    for loop in faces:
        size = len(loop)
        for position in range(size):
            half = (loop[position], loop[(position + 1) % size])
            seen[half] = seen.get(half, 0) + 1
    return sum(1 for count in seen.values() if count > 1)


__all__ = (
    "COUNTER_WELD_GROUPS",
    "COUNTER_WELD_SEAMS_MARKED",
    "COUNTER_WELD_VERTICES_MERGED",
    "DomainVerticesV1",
    "MITER_LIMIT",
    "OUTCOME_WELD_HALF_EDGE_CONFLICT",
    "OUTCOME_WELD_MITER_FALLBACK",
    "OUTCOME_WELD_POSITION_MISMATCH",
    "WELD_LOCATION_PREFIX",
    "WeldV1",
    "cross_domain_seams",
    "half_edge_conflicts",
    "miter_offset",
    "weld_vertices",
)
