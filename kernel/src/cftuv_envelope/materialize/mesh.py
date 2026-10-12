"""Точная сетка слитых граней: вершины, факты `(s, r)`, тесселяция (стадии STATIONS_UV и TESSELLATE материализатора).

Это ЭТАЛОН нативной операции `materialize_w1` (`cftuv_native.pin.OPERATION_FILES`): файл закреплён целиком, и каждое слово, которое
здесь меняется, меняет и порт. Два входа операции:

* `exact_mesh` - `intern_vertices` + `station_values` + `tessellate_faces` слитых граней домена одним вызовом: шаги идут в порядке
  стадий и под одним бюджетом точной работы (`stage="MATERIALIZE"`), одной памятью канонизации и одним счётчиком знаков;
* `canonical_triangles` - вторая, каноническая тесселяция (`TRIANGLES_V1`) тех же слитых граней для положения вершин `src:`; она платит
  заново в тот момент, когда её зовут (после резки и подъёма), и результат первого прохода не переиспользует.

Здесь нет ничего про Blender, батч и цепи: сборка `GeometryBatchV1` живёт в `assemble.py`, а положение вершин и силуэт - в `source_lift.py`
и `silhouette.py`. Раскладка регионов (`assemble.Layout`) приходит параметром, поэтому файл не импортирует `assemble`.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from decimal import (
    ROUND_HALF_EVEN,
    Context,
    Decimal,
    DivisionByZero,
    InvalidOperation,
    Overflow,
    localcontext,
)
from fractions import Fraction
from itertools import combinations

from ..contracts.geometry_batch import DecalTopologyLawV1
from ..exact_sqrt_sum import SqrtSumV1
from ..wavefront.faces import doubled_shoelace, orientation, shoelace_sign
from .admit import MaterializationOutcome
from .coalesce import lattice_node, point_key
from .convex_partition import (
    DIAGONALS as PARTITION_DIAGONALS,
    FACES_EMITTED as PARTITION_FACES_EMITTED,
    PIECES_EXACT,
    PIECES_GREEDY,
    PIECES_PARTITIONED,
    PIECES_REFUSED as PARTITION_PIECES_REFUSED,
    REASON_AREA as PARTITION_REASON_AREA,
    REASON_NO_GAIN as PARTITION_REASON_NO_GAIN,
    REASON_UV_FOLD as PARTITION_REASON_UV_FOLD,
    REFUSED_PREFIX as PARTITION_REFUSED_PREFIX,
    PartitionRefusalV1,
    plan_partition,
    verify_partition,
)
from .frames import MaterializationRefusal
from .lift import ENCLOSURE_BITS, enclosure_midpoint
from .stations import ChainStationTableV1, station_of, transverse_of, transverse_root
from .tessellate import (
    affine_frame,
    contour_is_simple,
    convex_polygon_ring,
    convex_quad_ring,
    counter_clockwise_ring,
    has_right_turn,
    triangulate_exact,
    triangulate_from_apex,
    uv_affine_defect_milli,
    uv_is_affine_in_chart,
    uv_vertex_on_affine_map,
)

#: 28 значащих цифр — точность десятичного контекста по умолчанию, на которой
#: кодек читает `Decimal`: больше цифр кодек молча обрезал бы на круговом проходе.
DECIMAL_DIGITS = 28

#: Десятичный контекст ЗАДАН целиком, а не унаследован от окружения: до этого
#: ставилась одна `prec`, а округление, границы порядка и ловушки брались у
#: процесса, то есть ответ материализатора зависел от того, кто вызвал его до
#: нас (хост, тест, чужой аддон в том же интерпретаторе). Значения — штатные
#: значения Python; важно, что они теперь написаны здесь.
DECIMAL_CONTEXT = Context(
    prec=DECIMAL_DIGITS,
    rounding=ROUND_HALF_EVEN,
    Emin=-999999,
    Emax=999999,
    capitals=1,
    clamp=0,
    flags=[],
    traps=[InvalidOperation, DivisionByZero, Overflow],
)


def decimal_of(value: SqrtSumV1, divisor: int) -> Decimal:
    """`value / divisor` как `Decimal` из середины строгой оболочки.

    Детерминированно: ЗАДАННЫЙ контекст `DECIMAL_CONTEXT` (копия на время
    деления), середина оболочки на `ENCLOSURE_BITS` разрядах. Рациональное с
    конечной десятичной записью проходит без шума. `Decimal(int)` точен при
    любом контексте, поэтому от контекста зависит только само деление.
    """

    numerator, denominator = enclosure_midpoint(value, ENCLOSURE_BITS, Fraction(1, divisor))
    with localcontext(DECIMAL_CONTEXT):
        return Decimal(numerator) / Decimal(denominator)


def _cycle(keys_and_points):
    """Контур без подряд идущих повторов вершины (нулевые отрезки)."""

    out: list = []
    for item in keys_and_points:
        if out and out[-1][0] == item[0]:
            continue
        out.append(item)
    if len(out) > 1 and out[0][0] == out[-1][0]:
        out.pop()
    return out


#: Имя вершины исходника, которое сварка точек отбросила: точка уже получила
#: другой ключ, а второй регион называет её другой исходной вершиной.
SOURCE_VERTEX_NAME_DROPPED = "SOURCE_VERTEX_NAME_DROPPED"


def intern_vertices(
    items,
    table: ChainStationTableV1,
    notes: list | None = None,
    chord_names: dict | None = None,
):
    """Ключи вершин по точкам контуров: `([цикл граней], {ключ: точка})`.

    `items` — `[(region_id, FrameFaceV1)]`. Порядок ключей `node:` — порядок
    первого появления точки при обходе граней и их вершин.

    Имя вершины даёт узел решётки (`table.node_vertex_ids`); точку, которая уже не узел (станция
    на хорде прямой цепи, `chord_station`), называет `chord_names[(регион, point_key)]`.

    ИМЯ `src:` берётся у ПЕРВОГО региона, который точку назвал, и этот выбор
    теперь не молчит. Две вещи, на которые он способен, названы:

    * та же точка, а другой регион называет её ДРУГОЙ исходной вершиной (либо
      первый назвать не смог и точка получила `node:`) — имя второй вершины
      сваркой отброшено; в `notes` ложится строка `SOURCE_VERTEX_NAME_DROPPED`
      (по одной на пару «точка, имя»), и число этих строк — счётчик
      материализатора. Меш при этом остаётся одним: сварка точек — решение, а
      не ошибка, ошибкой было бы его не заметить;
    * ДВЕ РАЗНЫЕ точки под одним именем `src:` — это уже не потеря имени, а
      слипшаяся геометрия (прежний код молча перезаписывал точку ключа), и это
      именованный отказ `VERTEX_KEY_COLLISION`.
    """

    interned: dict = {}
    points: dict[str, tuple] = {}
    counter = 0
    cycles = []
    reported: set = set()
    for region_id, frame_face in items:
        pairs = []
        for point in frame_face.face.points:
            pk = point_key(point)
            node = lattice_node(point)
            if node is not None:
                vertex_id = table.node_vertex_ids.get((region_id, node))
            else:
                vertex_id = None if chord_names is None else chord_names.get((region_id, pk))
            where = "its chord station" if node is None else f"node {node}"
            key = interned.get(pk)
            if key is None:
                if vertex_id is not None:
                    key = f"src:{vertex_id}"
                    if key in points:
                        raise MaterializationRefusal(
                            MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
                            f"VERTEX_KEY_COLLISION: {key} names two different "
                            f"points (region {region_id}, {where})",
                        )
                else:
                    key = f"node:{counter}"
                    counter += 1
                interned[pk] = key
                points[key] = point
            elif (
                notes is not None
                and vertex_id is not None
                and key != f"src:{vertex_id}"
                and (pk, vertex_id) not in reported
            ):
                reported.add((pk, vertex_id))
                notes.append(
                    f"{SOURCE_VERTEX_NAME_DROPPED}: {vertex_id} at {where} of "
                    f"region {region_id} is welded into {key}"
                )
            pairs.append((key, point))
        cycles.append(_cycle(pairs))
    return cycles, points


#: Число закона `RUNG_STATION_FROM_CHAIN_VERTEX_V1`: вершин на перекладине JOIN,
#: получивших станцию вершины цепи вместо двух ответов двух пробегов.
RUNG_STATIONS_FROM_CHAIN_VERTEX = "MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX"
#: Число закона `RUNG_CHORD_STATION_V1`: новых вершин резки на ОБЩЕМ ребре двух граней потока, получивших станцию
#: интерполяцией фактов концов ребра вдоль него (точно), потому что аффинные карты двух пробегов в ней не сошлись.
RUNG_CHORD_STATIONS = "MATERIALIZE_RUNG_CHORD_STATIONS"


def _rung_station(table, answers: dict, run_id: str, value):
    """`(s, r)` вершины на биссектрисе угла JOIN либо `None` (закон `RUNG_STATION_FROM_CHAIN_VERTEX_V1`).

    Два пробега одного потока, стыкующиеся углом JOIN, дают точке биссектрисы
    два `s`: `s_v + d sin(δ/2)` и `s_v - d sin(δ/2)` — и один `r`. Их среднее
    есть станция вершины цепи `s_v` ТОЧНО, и ровно это проверяется (нуль
    разности канонических `SqrtSumV1`): точка не на биссектрисе ответа не
    получает, и конфликт остаётся конфликтом.
    """

    if len(answers) != 1 or run_id in answers:
        return None
    (other_run, (other_s, other_r)), = answers.items()
    station = table.join_station(other_run, run_id)
    if station is None or not (other_r - value[1]).is_zero:
        return None
    if not (other_s + value[0] - station - station).is_zero:
        return None
    return station, value[1]


def _chord_station(chord, point, region, anchors, budget):
    """`(s, r)` новой вершины на ребре `(u, v)` контура: интерполяция фактов концов ребра вдоль него, либо `None`.

    Закон `RUNG_CHORD_STATION_V1`. Перекладина JOIN (общее ребро двух граней потока, биссектриса угла) несёт
    станцию вершины цепи, а не аффинную станцию пробега (`_rung_station`): `s` вдоль неё постоянна, `r` общая.
    Это выполняется ТОЧНО, пока перекладина лежит на биссектрисе. Привязка вершины `src:` к углу карты перед
    резкой (`clip_snap`, допуск `SOURCE_VERTEX_CORNER_SNAP_CELLS`) сдвигает конец перекладины на доли ячеек, и
    новая вершина резки на ней стоит рядом с биссектрисой, а не на ней: два пробега дают ей разные `(s, r)`, и
    `_rung_station` честно отказывает. Единственный ответ для вершины на ОБЩЕМ ребре — линейный вдоль ребра между
    фактами его концов (`share = (p - u) / (v - u)`, `(s, r) = u + (v - u) * share`): на биссектрисе он равен
    `_rung_station` точно (оба конца уже подчиняются перекладине), а при сдвинутом конце расходится с аффинной
    картой пробега на тот же сдвиг, что назван счётчиком привязки. Новых допусков нет; вершина, лежащая вне ребра
    (точная проверка второй координаты) или вне отрезка, ответа не получает, и конфликт остаётся конфликтом.
    """

    u_key, v_key, u_point, v_point = chord
    u_fact = anchors.get((region, u_key))
    v_fact = anchors.get((region, v_key))
    if u_fact is None or v_fact is None:
        return None
    axis = 0 if not (v_point[0] - u_point[0]).is_zero else 1
    span = v_point[axis] - u_point[axis]
    if span.is_zero:
        return None
    share = (point[axis] - u_point[axis]).divided_by(span, budget)
    other = 1 - axis
    if not ((point[other] - u_point[other]) - (v_point[other] - u_point[other]) * share).is_zero:
        return None
    if share.sign(budget=budget) < 0 or (SqrtSumV1.rational(1) - share).sign(budget=budget) < 0:
        return None
    return tuple(start + (end - start) * share for start, end in zip(u_fact, v_fact))


def _unify_across_frames(facts, answers, table, tally, rungs) -> None:
    """Закон `RUNG_STATION_FROM_CHAIN_VERTEX_V1` ЧЕРЕЗ границу кадров разомкнутого кольца.

    Угол JOIN между открывателем кольца и следующим вхождением лежит в двух регионах
    (`ChainStationTableV1.frame_of_run`), и в каждом из них вершина перекладины получает
    только свой ответ — конфликта нет, но UV на стыке рвалась бы на `2 d sin(δ/2)`. Здесь
    ответы двух пробегов ОДНОГО стыка в разных регионах сводятся к станции вершины цепи ТОЧНО
    по тому же условию, что и внутри региона (`r` равны, `s_a + s_b = 2 s_v`): после этого стык
    непрерывен, и `chains_of` не называет его швом.
    """

    by_key: dict = {}
    for (region, key), given in answers.items():
        for run_id, value in given.items():
            by_key.setdefault(key, []).append((region, run_id, value))
    for key, entries in by_key.items():
        for first in range(len(entries)):
            for second in range(first + 1, len(entries)):
                (region_a, run_a, value_a), (region_b, run_b, value_b) = entries[first], entries[second]
                station = table.cut_join_station(run_a, run_b)
                if region_a == region_b or station is None:
                    continue
                if not (value_a[1] - value_b[1]).is_zero:
                    continue
                if not (value_a[0] + value_b[0] - station - station).is_zero:
                    continue
                for region, value in ((region_a, value_a), (region_b, value_b)):
                    slot = (region, key)
                    if facts[slot] != (station, value[1]):
                        facts[slot] = (station, value[1])
                        rungs.add(slot)
                        if tally is not None:
                            tally[RUNG_STATIONS_FROM_CHAIN_VERTEX] += 1


def station_values(
    frame_faces, cycles, layout, table, lattice_alpha, budget, tally=None, rungs=None, chords=None, anchors=None, plain=None
):
    """`{(регион, ключ): (s, r)}` в единицах решётки, точные. Конфликт — отказ.

    Второй ответ на вершину региона допустим в двух случаях. Первый — перекладина угла JOIN
    (`_rung_station`); он считается в `tally`. Станцию вершины
    цепи на перекладине получает и стык двух регионов разомкнутого кольца
    (`_unify_across_frames`). `rungs` — множество `(регион, ключ)` вершин, получивших её:
    только четырёхгранье, в котором такая вершина есть, может нести билинейную UV.
    Второй — новая вершина резки на ОБЩЕМ ребре граней (`RUNG_CHORD_STATION_V1`, `_chord_station`): `chords[i][ключ]` —
    ребро `(u, v, точка u, точка v)` контура `i`-й грани, на котором она лежит, `anchors` — факты вершин контура
    (результат прежнего вызова). Закон действует, только если ВСЕ ответившие грани называют одно и то же ребро.

    `plain` - множество (изменяемое) номеров слитых граней, у которых факты КАЖДОЙ вершины контура в итоге равны собственным значениям
    грани (`station_of`, `transverse_of` её пробега и прямой): `(s, r)` такой грани - точно аффинная функция положения на карте по
    построению, и проверять это заново (`uv_is_affine_in_chart`) незачем (`_plane_ring`, `affine_known`). Конфликт, перекладина,
    станция по хорде и разомкнутое кольцо выводят грань из множества.
    """

    rungs = set() if rungs is None else rungs
    own: dict = {}

    facts: dict = {}
    roots: dict = {}
    answers: dict = {}
    edges_of: dict = {}
    for index, (frame_face, cycle) in enumerate(zip(frame_faces, cycles)):
        region = layout.region_of(frame_face)
        face_chords = None if chords is None else chords[index]
        line = frame_face.line
        root = roots.get(line.q)
        if root is None:
            root = roots[line.q] = transverse_root(line, budget)
        run_id = frame_face.run.run_id
        for key, point in cycle:
            station = (
                frame_face.fan_station
                if frame_face.is_fan
                else station_of(frame_face.run, point)
            )
            value = (station, transverse_of(line, point, root))
            own[(index, key)] = value
            slot = (region, key)
            chord = None if face_chords is None else face_chords.get(key)
            edges_of.setdefault(slot, set()).add(None if chord is None else frozenset(chord[:2]))
            known = facts.setdefault(slot, value)
            given = answers.setdefault(slot, {})
            if known == value or given.get(run_id) == value:
                given.setdefault(run_id, value)
                continue
            rung = None if frame_face.is_fan else _rung_station(table, given, run_id, value)
            on_one_edge = chord is not None and anchors is not None and len(edges_of[slot]) == 1
            if rung is None and on_one_edge and not frame_face.is_fan:
                interpolated = _chord_station(chord, point, region, anchors, budget)
                if interpolated is not None:
                    given[run_id] = value
                    if facts[slot] != interpolated:
                        facts[slot] = interpolated
                        if tally is not None:
                            tally[RUNG_CHORD_STATIONS] += 1
                    continue
            if rung is None:
                answered = ", ".join(
                    f"{name}=(s {decimal_of(given_value[0], table.scale):.6f}, r {decimal_of(given_value[1], table.scale):.6f})"
                    for name, given_value in (*given.items(), (run_id, value))
                )
                raise MaterializationRefusal(
                    MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
                    f"STATION_VALUE_CONFLICT: vertex {key} in region {region} "
                    f"has two (s, r) answers from owner {frame_face.face.owner}: {answered}",
                    station_conflict=(run_id, *given),
                )
            given[run_id] = value
            facts[slot] = rung
            rungs.add(slot)
            if tally is not None:
                tally[RUNG_STATIONS_FROM_CHAIN_VERTEX] += 1
    if table.frame_of_run:
        _unify_across_frames(facts, answers, table, tally, rungs)
    if plain is not None:
        for index, (frame_face, cycle) in enumerate(zip(frame_faces, cycles)):
            region = layout.region_of(frame_face)
            if all(facts[(region, key)] == own[(index, key)] for key, _point in cycle):
                plain.add(index)
    return facts


def _closes_the_area(total, expected, owner, parts: str) -> None:
    """Сумма удвоенных площадей частей разбиения равна площади контура: точно, иначе отказ."""

    if not (total - expected).is_zero:
        raise MaterializationRefusal(
            MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
            f"owner {owner}: {parts} differ from the face",
        )


def _ring_polygon(keys, ring, reverse: bool):
    """Многоугольник по кольцу против часовой; при `reverse` — обход, сохраняющий первую вершину."""

    ordered = tuple(keys[index] for index in ring)
    return (ordered[0], *reversed(ordered[1:])) if reverse else ordered


def _triangle_polygons(owner, expected, points, keys, budget, reverse: bool):
    """Треугольники контура по ключам: отсечение ушей, сумма площадей — точное равенство."""

    triangles = triangulate_exact(points, budget) if len(points) >= 3 else None
    if triangles is None:
        raise MaterializationRefusal(
            MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
            f"owner {owner}: {len(points)} contour points",
        )
    return _emitted_triangles(owner, expected, points, keys, triangles, reverse)


def _emitted_triangles(owner, expected, points, keys, triangles, reverse: bool):
    """Треугольники по индексам контура -> по ключам; сумма удвоенных площадей равна площади контура точно."""

    total = SqrtSumV1.zero()
    for first, second, third in triangles:
        total = total + doubled_shoelace(
            (points[first], points[second], points[third])
        )
    _closes_the_area(total, expected, owner, "triangle areas")
    return tuple(
        (keys[a], keys[c], keys[b]) if reverse else (keys[a], keys[b], keys[c])
        for a, b, c in triangles
    )


def tessellate_faces(
    frame_faces,
    cycles,
    budget,
    reverse: bool,
    law: DecalTopologyLawV1 = DecalTopologyLawV1.TRIANGLES_V1,
    exact_plane: bool = True,
    tally: Counter | None = None,
    uv_values=None,
    lattice_alpha=None,
    is_rung=None,
    partition=False,
    plain=frozenset(),
):
    """Грани каждой слитой грани по КЛЮЧАМ вершин: `[(грань, ...), ...]`. Не сложилось — отказ.

    Под `TRIANGLES_V1` — только треугольники. Под `QUAD_STRIPS_V1` строго
    выпуклый четырёхугольник ЛЕНТЫ (не веер) остаётся одной четырёхгранью, и
    площадь проверяется по контуру целиком; всё остальное — треугольники, как
    раньше. Подъём вершин здесь не виден: четырёхгранья, чьи вершины лягут на
    разные треугольники источника, режутся позже (`settle_topology`).

    Под `PLANAR_POLYGONS_V1` — `_polygon_law_faces`: `exact_plane` говорит, что
    укладка домена — точная плоскость (многоугольник плоский по построению), `tally`
    собирает названные исходы закона для `settle_topology`, а `uv_values(грань, ключ)`
    отдаёт точные `(s, r)` вершины в регионе грани: по ним проверяется аффинность UV
    (`PLANAR_AFFINE_UV_POLYGON_V1`). Без `uv_values` закон не может ничего доказать и
    отказывает `ValueError`, а не молча берёт грань целой. `lattice_alpha` — единица,
    в тысячных которой записывается излом UV билинейного четырёхгранья
    (`MATERIALIZE_QUADS_UV_BILINEAR_MAX_MILLI_ALPHA`); без неё — в единицах решётки.
    `is_rung(грань, ключ)` — вершина получила станцию вершины цепи на перекладине JOIN
    (`station_values`, `rungs`): билинейным четырёхгранье может быть ТОЛЬКО с такой
    вершиной; без `is_rung` закон закрыт и неаффинное четырёхгранье режется (`UV_NOT_AFFINE`).
    `partition` включает закон разбиения диагоналями (`CONVEX_PARTITION_BY_DIAGONALS_V1`) для кусков с неаффинной UV,
    которые прежде шли в уши; без него закона нет и ответ прежний побитово.
    """

    if law is DecalTopologyLawV1.PLANAR_POLYGONS_V1:
        if uv_values is None:
            raise ValueError("PLANAR_POLYGONS_V1 needs the exact (s, r) of every vertex")
        return _polygon_law_faces(
            frame_faces,
            cycles,
            budget,
            reverse,
            exact_plane,
            Counter() if tally is None else tally,
            uv_values,
            1 if lattice_alpha is None else lattice_alpha,
            is_rung,
            partition,
            plain,
        )
    result = []
    for frame_face, cycle in zip(frame_faces, cycles):
        points = tuple(point for _key, point in cycle)
        keys = tuple(key for key, _point in cycle)
        owner, area = frame_face.face.owner, frame_face.face.doubled_area
        ring = (
            convex_quad_ring(points, budget)
            if law is DecalTopologyLawV1.QUAD_STRIPS_V1 and not frame_face.is_fan
            else None
        )
        if ring is None:
            result.append(_triangle_polygons(owner, area, points, keys, budget, reverse))
            continue
        _closes_the_area(
            doubled_shoelace(tuple(points[index] for index in ring)),
            area,
            owner,
            "quad areas",
        )
        result.append((_ring_polygon(keys, ring, reverse),))
    return result


def canonical_triangles(frame_faces, cycles, budget, reverse: bool):
    """Треугольники закона `TRIANGLES_V1` СЛИТЫХ граней: `[(треугольник, ...), ...]` по ключам.

    Они одни и те же при любом законе топологии (уши точных контуров слитых граней), и по
    ним закон положения вершин `src:` (`source_lift`) сверяет ориентацию: позиции не вправе
    зависеть от закона.
    """

    return tessellate_faces(
        frame_faces, cycles, budget, reverse, DecalTopologyLawV1.TRIANGLES_V1
    )


#: Имена чисел закона `PLANAR_POLYGONS_V1` (они же ключи счётчиков материализатора).
POLYGON_FACES_EMITTED = "MATERIALIZE_POLYGON_FACES_EMITTED"
POLYGON_FACES_CONCAVE_EMITTED = "MATERIALIZE_POLYGON_FACES_CONCAVE_EMITTED"
POLYGON_FACES_TRIANGULATED_NOT_SIMPLE = "MATERIALIZE_POLYGON_FACES_TRIANGULATED_NOT_SIMPLE"
POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE = (
    "MATERIALIZE_POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE"
)
CURVED_STRIP_FACES_TRIANGULATED = "MATERIALIZE_CURVED_STRIP_FACES_TRIANGULATED"
MERGED_RUNS_SPLIT_AT_RUNGS = "MATERIALIZE_MERGED_RUNS_SPLIT_AT_RUNGS"
MERGED_RUNS_KEPT_WHOLE = "MATERIALIZE_MERGED_RUNS_KEPT_WHOLE"
#: Числа закона `FAN_FACE_TRIANGULATED_FROM_APEX_V1` (веера под `PLANAR_POLYGONS_V1`).
#: `CUT_BY_NEIGHBOUR` — веерных граней, чей контур ДЛИННЕЕ ТРЕУГОЛЬНИКА: обычно это клетка
#: среза локусом соседа, но вершина фронта на прямой (T-стык) тоже даёт контур длиннее трёх,
#: и срез от такой вершины число не отличает — оно верхняя оценка срезов. Оно всегда равно
#: сумме остальных трёх: каждая такая грань выпущена одним многоугольником, разрезана от
#: вершины веера либо — название беды — по ушам.
FAN_FACES_CUT_BY_NEIGHBOUR = "MATERIALIZE_FAN_FACES_CUT_BY_NEIGHBOUR"
FAN_POLYGON_FACES_EMITTED = "MATERIALIZE_FAN_POLYGON_FACES_EMITTED"
FAN_POLYGON_FACES_CONCAVE_EMITTED = "MATERIALIZE_FAN_POLYGON_FACES_CONCAVE_EMITTED"
FAN_FACES_TRIANGULATED_FROM_APEX = "MATERIALIZE_FAN_FACES_TRIANGULATED_FROM_APEX"
FAN_FACES_NOT_STAR_FROM_APEX = "MATERIALIZE_FAN_FACES_NOT_STAR_FROM_APEX"
#: Числа закона `QUAD_UV_BILINEAR_V1`: строго выпуклое четырёхгранье ленты, чья UV
#: НЕ аффинна по положению (перекладина угла JOIN несёт станцию вершины цепи), выпущено
#: одной гранью; второе число — наибольший излом UV на диагонали показа, в тысячных alpha.
QUADS_UV_BILINEAR = "MATERIALIZE_QUADS_UV_BILINEAR"
QUADS_UV_BILINEAR_MAX_MILLI_ALPHA = "MATERIALIZE_QUADS_UV_BILINEAR_MAX_MILLI_ALPHA"
#: То же для контура от пяти вершин (`QUAD_UV_BILINEAR_V1`, поправка 2026-10-03): прямые и почти прямые вершины
#: фронта и Т-стыки соседей не делают грань потока неаффинной иначе, чем четырёхгранье из ее углов.
POLYGONS_UV_BILINEAR = "MATERIALIZE_POLYGONS_UV_BILINEAR"
POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA = "MATERIALIZE_POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA"


def _rung_pieces(frame_face, key_of):
    """Части слитого пробега с их контурами по ключам: `[(часть, цикл), ...]` либо `None`.

    `None` — пробег не сливался либо у какой-то части есть точка, которой нет в
    слитом контуре (вершина внутри перекладины: сливаясь, перекладина её
    теряет). Новых вершин здесь не заводят: сетка вершин, их ключи и номера
    `node:` те же, что у остальных законов; такой пробег остаётся целым и
    называется (`MERGED_RUNS_KEPT_WHOLE`).
    """

    parts = getattr(frame_face.face, "parts", ())
    if len(parts) < 2:
        return None
    pieces = []
    for part in parts:
        pairs = []
        for point in part.points:
            key = key_of.get(point_key(point))
            if key is None:
                return None
            pairs.append((key, point))
        cycle = _cycle(pairs)
        if len(cycle) < 3:
            return None
        pieces.append((part, cycle))
    return pieces


#: Сверка пропуска `uv_is_affine_in_chart` у простых граней (`station_values`, `plain`): с флагом проверка идёт и ОБЯЗАНА сойтись (тесты).
VERIFY_PLAIN_AFFINE = False


def _plane_ring(points, keys, budget, uv_of, affine_known=False):
    """`(кольцо, имя)` грани на точной плоскости (закон `PLANAR_AFFINE_UV_POLYGON_V1`).

    Грань берётся целой, если контур ПРОСТ и UV — аффинная функция положения на
    карте по всему контуру (оба условия точные). Выпуклость не нужна: любая
    триангуляция показа даёт ту же поверхность и ту же UV-интерполяцию.

    Простота проверяется у КАЖДОГО контура, а не только с правым поворотом: пятиконечная
    звезда сплошь из левых поворотов, но сама себя пересекает. Контур прост, если вершины
    попарно различны по ключам (повтор — перетяжка, касание в вершине, которого
    трансверсальные пересечения не видят) и `contour_is_simple` (пересечений нет, допустимая
    триангуляция есть).

    Кольцо `None` — грань уходит в треугольники. Имя — счётчик, который вызывающий
    прибавит, ТОЛЬКО когда грань действительно выпущена: `NOT_SIMPLE` и `UV_NOT_AFFINE`
    означают «разрезана на треугольники», `CONCAVE_EMITTED` — «выпущена целой с правым
    поворотом». Нулевая площадь не названа ничем: ни многоугольника, ни треугольников
    у такого контура нет, и `_triangle_polygons` откажет `TESSELLATION_DID_NOT_CLOSE`.

    `affine_known` - контур целиком из вершин простой грани: её `(s, r)` - точно аффинная функция положения по построению (формулы
    `station_of` и `transverse_of` линейны по точке), и точная проверка тождества - то же `True`, которое она стоила бы тысячи
    произведений корней; её пропускают (`VERIFY_PLAIN_AFFINE` возвращает проверку и требует `True`).
    """

    ring = counter_clockwise_ring(points, budget)
    if ring is None:
        return None, None
    if len(set(keys)) != len(keys) or not contour_is_simple(points, budget):
        return None, POLYGON_FACES_TRIANGULATED_NOT_SIMPLE
    if not affine_known or VERIFY_PLAIN_AFFINE:
        affine = uv_is_affine_in_chart(points, [uv_of(key) for key in keys], budget)
        if affine_known and not affine:
            raise AssertionError("PLAIN_FACE_UV_NOT_AFFINE: a plain face must have an affine UV by construction")
        if not affine:
            return None, POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE
    concave = has_right_turn(points, ring, budget)
    return ring, POLYGON_FACES_CONCAVE_EMITTED if concave else None


def _fan_apex(owner, points):
    """Индекс вершины веера в контуре либо `None`: точка узла `owner[:2]` (решёточные координаты)."""

    for index, point in enumerate(points):
        if point[0].as_rational() == owner[0] and point[1].as_rational() == owner[1]:
            return index
    return None


def _fan_faces(frame_face, cycle, budget, reverse, exact_plane, tally, uv_of, affine_known=False):
    """Грани ОДНОГО веера под `PLANAR_POLYGONS_V1` (закон `FAN_FACE_TRIANGULATED_FROM_APEX_V1`).

    Треугольник — он сам. Обрезанная соседом клетка (контур длиннее трёх) на точной плоскости
    остаётся ОДНОЙ гранью, если она проста и её UV аффинна (`_plane_ring`, точно): в веере
    `s` постоянна, а `r` линейна по положению, поэтому аффинность выполнена по построению, но
    доказывается всё равно. Выпуклость не нужна и не обещана: полевая клетка бывает невыпуклой
    (срез локусом соседа даёт правый поворот), и любая триангуляция показа даёт ту же
    поверхность и ту же UV-интерполяцию; невыпуклые названы (`FAN_POLYGON_FACES_CONCAVE_EMITTED`).
    Иначе (кривая укладка, непростая клетка, неаффинная UV — последние две названы прежними
    `POLYGON_FACES_TRIANGULATED_*`) клетка режется ОТ вершины веера (`triangulate_from_apex`);
    если и так не складывается (вершины веера нет в контуре либо треугольник вырожден) — по
    ушам и называется (`FAN_FACES_NOT_STAR_FROM_APEX`). Секторы одного угла между собой не
    сливаются: у каждого своя опора и свой `r`, UV кусочно-линейна.
    """

    points = tuple(point for _key, point in cycle)
    keys = tuple(key for key, _point in cycle)
    owner, area = frame_face.face.owner, frame_face.face.doubled_area
    if len(points) <= 3:
        return _triangle_polygons(owner, area, points, keys, budget, reverse)
    tally[FAN_FACES_CUT_BY_NEIGHBOUR] += 1
    named = None
    if exact_plane:
        ring, named = _plane_ring(points, keys, budget, uv_of, affine_known)
        if ring is not None:
            _closes_the_area(
                doubled_shoelace(tuple(points[index] for index in ring)),
                area,
                owner,
                "fan polygon areas",
            )
            tally[FAN_POLYGON_FACES_EMITTED] += 1
            tally[FAN_POLYGON_FACES_CONCAVE_EMITTED] += int(named is not None)
            return (_ring_polygon(keys, ring, reverse),)
    apex = _fan_apex(owner, points)
    triangles = None if apex is None else triangulate_from_apex(points, apex, budget)
    if triangles is None:
        polygons = _triangle_polygons(owner, area, points, keys, budget, reverse)
        tally[FAN_FACES_NOT_STAR_FROM_APEX] += 1
    else:
        polygons = _emitted_triangles(owner, area, points, keys, triangles, reverse)
        tally[FAN_FACES_TRIANGULATED_FROM_APEX] += 1
    # Причина, по которой клетка не стала одной гранью (непростая, неаффинная UV), названа на
    # обоих путях и только когда грань увидел меш: отказ разреза её не оставляет.
    if named is not None:
        tally[named] += 1
    return polygons


def _strictly_convex_ring(points, budget):
    """Кольцо выпуклого контура (вершины на прямой допустимы) не меньше чем с четырьмя строго левыми поворотами, либо `None`.

    Геометрическая половина закона `QUAD_UV_BILINEAR_V1` (`_bilinear_ring`): дешёвые знаки без UV. Такой контур допустим
    ВСЕГДА — аффинной UV либо билинейной, — поэтому разбиение диагоналями (`_convex_faces`) решает «допустима ли часть»
    по нему, не считая UV (точная проверка аффинности на иррациональных `(s, r)` — самая дорогая операция куска).
    """

    ring = convex_quad_ring(points, budget) if len(points) == 4 else convex_polygon_ring(points, budget)
    if ring is None:
        return None
    size = len(ring)
    corners = sum(
        1
        for position in range(size)
        if orientation(
            points[ring[position - 1]], points[ring[position]], points[ring[(position + 1) % size]], budget
        )
        > 0
    )
    return ring if corners >= 4 else None


def _bilinear_ring(points, keys, budget, uv_of, unit, tally):
    """Кольцо выпуклой грани потока с билинейной UV (закон `QUAD_UV_BILINEAR_V1`) либо `None`.

    Четырёхгранье ленты с неаффинной UV рождается ровно одним законом — станцией
    вершины цепи на перекладине угла JOIN (`RUNG_STATION_FROM_CHAIN_VERTEX_V1`):
    в UV оно прямоугольник, на карте трапеция. Любая триангуляция показа даёт ту
    же поверхность; UV на диагонали показа изламывается не больше записанного
    числа (`MATERIALIZE_QUADS_UV_BILINEAR_MAX_MILLI_ALPHA`, тысячные alpha).
    Резать его по диагонали (`UV_NOT_AFFINE`) значило бы вернуть на ленту
    ребро, которого в топологии источника нет.

    То же для контура от пяти вершин (`MATERIALIZE_POLYGONS_UV_BILINEAR`): его лишние вершины — вершины фронта
    на хорде полосы (повороты в доли градуса) и Т-стыки соседей, а углов, строго поворачивающих влево, не меньше
    четырёх. Выпуклость точная (`convex_polygon_ring`: ни одного правого поворота; простота контура доказана до
    вызова, `_plane_ring`), вершины на прямой остаются в кольце грани. Контур с тремя углами (треугольник с вершинами
    на сторонах) и невыпуклый ушами режутся по-прежнему. Число — запись, а не суд: излом UV грани считается тем же
    `uv_affine_defect_milli`, в тысячных alpha.
    """

    ring = _strictly_convex_ring(points, budget)
    if ring is None:
        return None
    size = len(ring)
    defect = uv_affine_defect_milli(points, [uv_of(key) for key in keys], unit, budget)
    name = QUADS_UV_BILINEAR_MAX_MILLI_ALPHA if size == 4 else POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA
    tally[name] = max(tally[name], int(defect))
    return ring


def _merge_scratch(tally, scratch) -> None:
    """Числа, накопленные вхолостую на куске, — в общий счёт: излом UV берёт наибольший, остальное складывается."""

    for name, value in scratch.items():
        if name.endswith("_MAX_MILLI_ALPHA"):
            tally[name] = max(tally[name], value)
        else:
            tally[name] += value


def _admitted_polygons(faces, coords, node_keys, look, budget, reverse, in_flow, unit, scratch):
    """Части разбиения (узлы против часовой на карте) допускаются ПРЕЖНИМ законом куска: `(многоугольники, площадь, имя)`.

    Треугольник — как есть; многоугольник — аффинная UV (`_plane_ring`) либо, у полосы потока, строго выпуклая грань с
    билинейной UV (`_bilinear_ring`). Числа допуска копятся в `scratch`, а не в счёте домена: они войдут в него, только
    если разбиение примут целиком. Часть, которую закон не берёт, — `(None, None, имя причины)`.
    """

    emitted: list = []
    total = SqrtSumV1.zero()
    for nodes in faces:
        face_points = tuple(coords[node] for node in nodes)
        face_keys = tuple(node_keys[node] for node in nodes)
        ring = tuple(range(len(nodes)))
        if len(nodes) > 3:
            ring, named = _plane_ring(face_points, face_keys, budget, look.__getitem__)
            if ring is None and in_flow and named == POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE:
                ring = _bilinear_ring(face_points, face_keys, budget, look.__getitem__, unit, scratch)
                if ring is not None:
                    scratch[QUADS_UV_BILINEAR if len(nodes) == 4 else POLYGONS_UV_BILINEAR] += 1
            elif ring is not None and named is not None:
                scratch[named] += 1
            if ring is None:
                return None, None, named
            scratch[POLYGON_FACES_EMITTED] += int(len(ring) > 4)
        total = total + doubled_shoelace(tuple(face_points[index] for index in ring))
        emitted.append(_ring_polygon(face_keys, ring, reverse))
    return tuple(emitted), total, None


def _refused_partition(tally, refusal: PartitionRefusalV1) -> None:
    """Отказ закона разбиения диагоналями записан по имени причины."""

    tally[PARTITION_PIECES_REFUSED] += 1
    tally[PARTITION_REFUSED_PREFIX + refusal.reason] += 1


def _convex_faces(piece, cycle, budget, reverse, tally, uv_of, unit, in_flow):
    """Грани куска по закону `CONVEX_PARTITION_BY_DIAGONALS_V1` либо `None` (кусок остаётся на ушах, отказ назван).

    Диагонали только между вершинами куска: новых вершин нет, шва и T-стыков закон не касается. Часть допустима прежним
    законом куска (`_admitted_polygons`: треугольник, аффинная UV либо строго выпуклый многоугольник потока); число
    частей наименьшее у малого куска и жадное слияние у длинного (`plan_partition`). Доказательство планарного
    разбиения (`verify_partition`), та же ориентация образа каждой части в UV, что у образа куска (складки нет) и
    точная сумма площадей — иначе уши, отказ назван.
    """

    points = tuple(point for _key, point in cycle)
    keys = tuple(key for key, _point in cycle)
    look = {key: tuple(uv_of(key)) for key in keys}
    values = [look[key] for key in keys]
    frames: dict = {}
    on_map: dict = {}

    def affine(verts) -> bool:
        """UV аффинна на вершинах части: по канонической основе (три наименьших неколлинеарных индекса), ответы запомнены."""

        ordered = sorted(verts)
        base = next(
            (
                (first, second, third)
                for first, second, third in combinations(ordered, 3)
                if orientation(points[first], points[second], points[third], budget)
            ),
            None,
        )
        if base is None:
            return False
        frame = frames.get(base)
        if frame is None:
            frame = frames[base] = affine_frame(points, base)
        for index in ordered:
            if index in base:
                continue
            found = on_map.get((base, index))
            if found is None:
                found = on_map[(base, index)] = uv_vertex_on_affine_map(points, values, base, frame, index)
            if not found:
                return False
        return True

    def admissible(verts) -> bool:
        """Часть допустима прежним законом: строго выпуклая у потока (UV не нужна) либо с аффинной UV.

        Части режутся допустимыми диагоналями, поэтому каждая — простой многоугольник по построению; закон куска
        (`_admitted_polygons`) всё равно проверяет выпущенные части заново.
        """

        if in_flow and _strictly_convex_ring(tuple(points[index] for index in verts), budget) is not None:
            return True
        return affine(verts)

    plan = plan_partition(points, budget, admissible)
    if isinstance(plan, PartitionRefusalV1):
        _refused_partition(tally, plan)
        return None
    if len(plan.pieces) >= len(points) - 2:
        _refused_partition(tally, PartitionRefusalV1(PARTITION_REASON_NO_GAIN))
        return None
    bad = verify_partition(points, plan, budget)
    if bad is not None:
        _refused_partition(tally, bad)
        return None
    piece_sign = shoelace_sign(tuple(look[keys[index]] for index in plan.order), budget)
    if not piece_sign or any(
        shoelace_sign(tuple(look[keys[index]] for index in part), budget) != piece_sign for part in plan.pieces
    ):
        _refused_partition(tally, PartitionRefusalV1(PARTITION_REASON_UV_FOLD))
        return None
    scratch: Counter = Counter()
    emitted, total, _named = _admitted_polygons(plan.pieces, points, keys, look, budget, reverse, in_flow, unit, scratch)
    if emitted is None or not (total - piece.doubled_area).is_zero:
        _refused_partition(tally, PartitionRefusalV1(PARTITION_REASON_AREA))
        return None
    _merge_scratch(tally, scratch)
    tally[PIECES_PARTITIONED] += 1
    tally[PIECES_EXACT if plan.exact else PIECES_GREEDY] += 1
    tally[PARTITION_FACES_EMITTED] += len(plan.pieces)
    tally[PARTITION_DIAGONALS] += len(plan.pieces) - 1
    return emitted


def _contour_polygons(
    piece, cycle, budget, reverse, exact_plane, tally, uv_of, unit=1, in_flow=False, partition=False, affine_known=False
):
    """Грани ОДНОГО контура по закону `PLANAR_POLYGONS_V1`: многоугольник либо треугольники под именем.

    `piece` — то, чьи владелец и площадь контур обязан замкнуть. На точной
    плоскости контур от четырёх вершин — одна грань любой длины, если он прост и его
    UV аффинен (`_plane_ring`); строго выпуклое четырёхгранье с билинейной UV — тоже
    одна грань, под своим именем (`_bilinear_ring`), но ТОЛЬКО у полосы потока И с
    вершиной перекладины JOIN (`in_flow`: `FrameFaceV1.flow_key` и `is_rung` хотя бы у одной
    вершины контура): только такую UV рождает закон перекладины. Любое другое неаффинное
    многоугольник сперва идёт в разбиение диагоналями между своими вершинами (`CONVEX_PARTITION_BY_DIAGONALS_V1`,
    `partition`: без него закона нет; новых вершин нет) и только при его названном отказе режется по-прежнему
    (`UV_NOT_AFFINE`, уши). На укладке на треугольники источника целым остаётся только строго выпуклое
    четырёхгранье (его плоскостность решает `settle_topology`), контур длиннее — треугольники и
    `CURVED_STRIP_FACES_TRIANGULATED`.
    """

    points = tuple(point for _key, point in cycle)
    keys = tuple(key for key, _point in cycle)
    owner, area = piece.owner, piece.doubled_area
    named = None
    if len(points) > 3:
        if exact_plane:
            ring, named = _plane_ring(points, keys, budget, uv_of, affine_known)
            if in_flow and ring is None and named == POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE:
                ring = _bilinear_ring(points, keys, budget, uv_of, unit, tally)
                if ring is not None:
                    named = QUADS_UV_BILINEAR if len(points) == 4 else POLYGONS_UV_BILINEAR
            if partition and ring is None and named == POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE:
                parts = _convex_faces(piece, cycle, budget, reverse, tally, uv_of, unit, in_flow)
                if parts is not None:
                    return parts
        else:
            ring = convex_quad_ring(points, budget)
            if ring is None:
                named = (
                    QUADS_REFUSED_NOT_CONVEX
                    if len(points) == 4
                    else CURVED_STRIP_FACES_TRIANGULATED
                )
        if ring is not None:
            _closes_the_area(
                doubled_shoelace(tuple(points[index] for index in ring)),
                area,
                owner,
                "polygon areas",
            )
            tally[POLYGON_FACES_EMITTED] += int(len(ring) > 4)
            if named is not None:
                tally[named] += 1
            return (_ring_polygon(keys, ring, reverse),)
    polygons = _triangle_polygons(owner, area, points, keys, budget, reverse)
    # Имя — про грань, которую увидел меш: отказ `_triangle_polygons` его не оставляет.
    if named is not None:
        tally[named] += 1
    return polygons


def _polygon_law_faces(
    frame_faces, cycles, budget, reverse, exact_plane, tally, uv_values, unit=1, is_rung=None, partition=False, plain=frozenset()
):
    """`tessellate_faces` под `PLANAR_POLYGONS_V1`: веера — `_fan_faces`, ленты — многоугольники.

    Слитый пробег режется по перекладинам на грани своих рёбер-источников
    (`_rung_pieces`): площадь каждой части замыкается точно, и сумма частей
    равна площади слитой грани. `unit` — единица записи излома билинейной UV. `partition` включает разбиение
    диагоналями для кусков, чья UV неаффинна и которым не хватило билинейного закона (`_convex_faces`).
    """

    key_of = {point_key(point): key for cycle in cycles for key, point in cycle}
    result = []
    for index, (frame_face, cycle) in enumerate(zip(frame_faces, cycles)):
        face = frame_face.face
        simple = index in plain
        if frame_face.is_fan:
            result.append(
                _fan_faces(
                    frame_face,
                    cycle,
                    budget,
                    reverse,
                    exact_plane,
                    tally,
                    lambda key, frame_face=frame_face: uv_values(frame_face, key),
                    simple,
                )
            )
            continue
        pieces = _rung_pieces(frame_face, key_of)
        if len(getattr(face, "parts", ())) >= 2:
            tally[MERGED_RUNS_SPLIT_AT_RUNGS if pieces else MERGED_RUNS_KEPT_WHOLE] += 1
        if pieces is None:
            pieces = [(face, cycle)]
        else:
            total = SqrtSumV1.zero()
            for part, _cycle_of_part in pieces:
                total = total + part.doubled_area
            _closes_the_area(total, face.doubled_area, face.owner, "run part areas")
        polygons = []
        cycle_keys = frozenset(key for key, _point in cycle) if simple else frozenset()
        for part, part_cycle in pieces:
            polygons.extend(
                _contour_polygons(
                    part,
                    part_cycle,
                    budget,
                    reverse,
                    exact_plane,
                    tally,
                    lambda key, frame_face=frame_face: uv_values(frame_face, key),
                    unit,
                    getattr(frame_face, "flow_key", None) is not None
                    and is_rung is not None
                    and any(is_rung(frame_face, key) for key, _point in part_cycle),
                    partition,
                    simple and all(key in cycle_keys for key, _point in part_cycle),
                )
            )
        result.append(tuple(polygons))
    return result


#: Имена чисел закона топологии (они же ключи счётчиков материализатора).
QUADS_REFUSED_NOT_CONVEX = "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX"


@dataclass(frozen=True, slots=True)
class ExactMeshV1:
    """Ответ `exact_mesh`: контуры граней, точки, факты `(s, r)`, грани по ключам, числа закона, вершины перекладин.

    `cycles` - `[[(ключ, точка)]]` по слитым граням (точка - объект самой грани), `points` - `{ключ: первая точка ключа}`, `facts` -
    `{(регион, ключ): (s, r)}` в порядке первого появления, `polygons` - `[(грань, ...)]` по слитым граням, `tally` - числа закона (`Counter`),
    `rungs` - `{(регион, ключ)}` вершин, получивших станцию вершины цепи (`station_values`).
    """

    cycles: list
    points: dict
    facts: dict
    polygons: list
    tally: Counter
    rungs: set


def exact_mesh(
    frame_faces,
    region_ids,
    layout,
    table: ChainStationTableV1,
    notes: list,
    chord_names: dict | None,
    lattice_alpha,
    budget,
    chart_cw: bool,
    law: DecalTopologyLawV1,
    exact_plane: bool,
    lap=None,
) -> ExactMeshV1:
    """Вершины, факты `(s, r)` и грани слитых граней домена: стадии STATIONS_UV и TESSELLATE материализатора, одним вызовом.

    `region_ids[i]` - имя региона покрытия грани `frame_faces[i]`, `layout` - раскладка регионов домена (`assemble.Layout`), `notes` - список,
    в который кладутся строки `SOURCE_VERTEX_NAME_DROPPED` (на месте), `chord_names` - имена вершин станций на хордах прямых цепей,
    `chart_cw` - карта обходится по часовой, `law` - закон ТЕССЕЛЯЦИИ (`domain._tessellation_law`), `exact_plane` - укладка на точную плоскость
    (или резка). `lap(имя)` вызывается после станций и после тесселяции (`_Clock.lap`: метки времени стадий); `None` - без меток.
    """

    cycles, points = intern_vertices(list(zip(region_ids, frame_faces)), table, notes, chord_names)
    tally, rungs = Counter(), set()
    plain: set = set()
    facts = station_values(frame_faces, cycles, layout, table, lattice_alpha, budget, tally, rungs, plain=plain)
    if lap is not None:
        lap("STATIONS_UV")
    polygons = tessellate_faces(
        frame_faces,
        cycles,
        budget,
        reverse=chart_cw,
        law=law,
        exact_plane=exact_plane,
        tally=tally,
        uv_values=lambda frame_face, key: facts[(layout.region_of(frame_face), key)],
        lattice_alpha=lattice_alpha,
        is_rung=lambda frame_face, key: (layout.region_of(frame_face), key) in rungs,
        partition=True,
        plain=plain,
    )
    if lap is not None:
        lap("TESSELLATE")
    return ExactMeshV1(cycles, points, facts, polygons, tally, rungs)
