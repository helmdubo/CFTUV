"""Сборка `GeometryBatchV1` из слитых граней с кадрами: вершины, факты, цепи.

Вход — `FrameFaceV1` слитых граней (точные контуры, кадр станции, имя
огибающей) и таблица станций. Выход — батч, который проходит
`validate_geometry_batch`. Правила, которые здесь держатся:

* ВЕРШИНА ОДНА НА ТОЧКУ. Ключ `src:<SourceVertexId>` — у узла, который есть
  образ исходной вершины на решётке; ключ `node:<k>` — у остальных точек, по
  порядку первого появления в контурах. `node:` — тождество ИНТЕРНИРОВАННОЙ
  точной точки, а не округление координат: запрещённые валидатором префиксы
  (`coord:`, `xyz:`, `rounded:`) и их смысл здесь не нарушены.
* РЕГИОН = (огибающая, кадр станции). Валидатор требует ОДИН `(s, r)` и ОДИН
  UV на вершину внутри региона; на изломе цепи и на стыке разных цепей у
  вершины законно два набора, и они живут в разных регионах. Вершина при этом
  остаётся ОДНОЙ (сварной): шов UV — это разные факты на одной вершине, а не
  дубликат вершины.
* ЦЕПИ. Граничные — полурёбра без встречной пары (источник `r = 0`, фронт
  `r = alpha`, остальное — стена); интерфейсные — общие полурёбра граней
  РАЗНЫХ регионов (шов UV либо граница владения). Цепи собираются по
  вершинам СЛИТЫХ контуров, то есть не зависят от триангуляции.
"""

from __future__ import annotations

from collections import Counter
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

from ..contracts.geometry_batch import (
    GEOMETRY_BATCH_SCHEMA_V1,
    DecalTopologyLawV1,
    GeometryBatchV1,
    GeometryBoundaryChainV1,
    GeometryFaceV1,
    GeometryInterfaceChainV1,
    GeometryProvenanceV1,
    GeometrySemanticRegionV1,
    GeometryStationFactV1,
    GeometryUvFactV1,
    GeometryVertexV1,
)
from ..ids import (
    ChainUseId,
    ContractVersionId,
    GeometryFaceId,
    GeometryStationFactId,
    LineageId,
    MaterialId,
    OwnershipClaimId,
    PhysicalEdgeId,
    SemanticBoundaryId,
    SemanticDigestValue,
    SemanticInterfaceId,
    SemanticLocationId,
    SemanticRegionId,
    SourceFaceId,
    VertexKey,
)
from ..numeric import LocalCoordinateV1
from ..exact_sqrt_sum import SqrtSumV1
from .admit import MaterializationOutcome
from .coalesce import lattice_node, point_key
from .frames import FrameFaceV1, MaterializationRefusal
from .lift import ENCLOSURE_BITS
from .stations import ChainStationTableV1, station_of, transverse_of, transverse_root
from .tessellate import (
    contour_is_simple,
    convex_polygon_ring,
    convex_quad_ring,
    counter_clockwise_ring,
    fan_out,
    has_right_turn,
    triangulate_exact,
    triangulate_from_apex,
    uv_affine_defect_milli,
    uv_is_affine_in_chart,
)
from .uv_law import uv_direct_strip_v1
from ..wavefront.faces import doubled_shoelace, orientation

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

    low, high = value.scaled(Fraction(1, divisor)).enclosure(ENCLOSURE_BITS)
    middle = (low + high) / 2
    with localcontext(DECIMAL_CONTEXT):
        return Decimal(middle.numerator) / Decimal(middle.denominator)


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


class Layout:
    """Порядковые имена регионов и огибающих: отсортированные имена -> номера."""

    def __init__(self, frame_faces):
        claims = sorted({item.claim_key for item in frame_faces})
        frames = sorted({(item.claim_key, item.frame_key) for item in frame_faces})
        self.claim = {name: index for index, name in enumerate(claims)}
        self.region = {name: index for index, name in enumerate(frames)}

    def region_of(self, frame_face) -> int:
        return self.region[(frame_face.claim_key, frame_face.frame_key)]

    def cut_pairs(self, frame_faces) -> frozenset:
        """Пары регионов одного РАЗОМКНУТОГО кольца потока (`FLOW_CYCLE_OPENED`): `{frozenset({a, b}), ...}`.

        Поток — один регион; два региона у одного потока бывают ровно у кольца, разомкнутого
        в одном месте, и граница между ними — не шов целиком: шов там, где UV рвётся (вершина
        разреза), а стык `a -> b` у открывателя непрерывен (`chains_of`).
        """

        regions: dict = {}
        for item in frame_faces:
            if getattr(item, "flow_key", None) is not None:
                regions.setdefault(item.flow_key, set()).add(self.region_of(item))
        return frozenset(frozenset(found) for found in regions.values() if len(found) == 2)

    def claim_id(self, name: str) -> OwnershipClaimId:
        return OwnershipClaimId(f"claim:{self.claim[name]}")

    @staticmethod
    def region_id(index: int) -> SemanticRegionId:
        return SemanticRegionId(f"region:{index}")


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
    frame_faces, cycles, layout, table, lattice_alpha, budget, tally=None, rungs=None, chords=None, anchors=None
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
    """

    rungs = set() if rungs is None else rungs

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


def _plane_ring(points, keys, budget, uv_of):
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
    """

    ring = counter_clockwise_ring(points, budget)
    if ring is None:
        return None, None
    if len(set(keys)) != len(keys) or not contour_is_simple(points, budget):
        return None, POLYGON_FACES_TRIANGULATED_NOT_SIMPLE
    if not uv_is_affine_in_chart(points, [uv_of(key) for key in keys], budget):
        return None, POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE
    concave = has_right_turn(points, ring, budget)
    return ring, POLYGON_FACES_CONCAVE_EMITTED if concave else None


def _fan_apex(owner, points):
    """Индекс вершины веера в контуре либо `None`: точка узла `owner[:2]` (решёточные координаты)."""

    for index, point in enumerate(points):
        if point[0].as_rational() == owner[0] and point[1].as_rational() == owner[1]:
            return index
    return None


def _fan_faces(frame_face, cycle, budget, reverse, exact_plane, tally, uv_of):
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
        ring, named = _plane_ring(points, keys, budget, uv_of)
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
    if corners < 4:
        return None
    defect = uv_affine_defect_milli(points, [uv_of(key) for key in keys], unit, budget)
    name = QUADS_UV_BILINEAR_MAX_MILLI_ALPHA if size == 4 else POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA
    tally[name] = max(tally[name], int(defect))
    return ring


def _contour_polygons(piece, cycle, budget, reverse, exact_plane, tally, uv_of, unit=1, in_flow=False):
    """Грани ОДНОГО контура по закону `PLANAR_POLYGONS_V1`: многоугольник либо треугольники под именем.

    `piece` — то, чьи владелец и площадь контур обязан замкнуть. На точной
    плоскости контур от четырёх вершин — одна грань любой длины, если он прост и его
    UV аффинен (`_plane_ring`); строго выпуклое четырёхгранье с билинейной UV — тоже
    одна грань, под своим именем (`_bilinear_ring`), но ТОЛЬКО у полосы потока И с
    вершиной перекладины JOIN (`in_flow`: `FrameFaceV1.flow_key` и `is_rung` хотя бы у одной
    вершины контура): только такую UV рождает закон перекладины, а любое другое неаффинное
    многоугольник режется по-прежнему (`UV_NOT_AFFINE`). На укладке на треугольники
    источника целым остаётся только строго выпуклое четырёхгранье (его плоскостность
    решает `settle_topology`), контур длиннее — треугольники и
    `CURVED_STRIP_FACES_TRIANGULATED`.
    """

    points = tuple(point for _key, point in cycle)
    keys = tuple(key for key, _point in cycle)
    owner, area = piece.owner, piece.doubled_area
    named = None
    if len(points) > 3:
        if exact_plane:
            ring, named = _plane_ring(points, keys, budget, uv_of)
            if in_flow and ring is None and named == POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE:
                ring = _bilinear_ring(points, keys, budget, uv_of, unit, tally)
                if ring is not None:
                    named = QUADS_UV_BILINEAR if len(points) == 4 else POLYGONS_UV_BILINEAR
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
    frame_faces, cycles, budget, reverse, exact_plane, tally, uv_values, unit=1, is_rung=None
):
    """`tessellate_faces` под `PLANAR_POLYGONS_V1`: веера — `_fan_faces`, ленты — многоугольники.

    Слитый пробег режется по перекладинам на грани своих рёбер-источников
    (`_rung_pieces`): площадь каждой части замыкается точно, и сумма частей
    равна площади слитой грани. `unit` — единица записи излома билинейной UV.
    """

    key_of = {point_key(point): key for cycle in cycles for key, point in cycle}
    result = []
    for frame_face, cycle in zip(frame_faces, cycles):
        face = frame_face.face
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
                )
            )
        result.append(tuple(polygons))
    return result


#: Имена чисел закона топологии (они же ключи счётчиков материализатора).
QUADS_REFUSED_NOT_CONVEX = "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX"
QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES = "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"
QUADS_SPLIT_OFFSET_NORMALS_DIFFER = "MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER"
MERGED_RUN_FACES_TRIANGULATED = "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED"


def _split_reason(polygon, sources):
    """Почему четырёхгранье не плоское в 3D: `"triangles"`, `"normals"` либо `None` (годится)."""

    tags = [sources[key] for key in polygon]
    if len({triangle for triangle, _normal in tags}) != 1:
        return "triangles"
    if len({normal for _triangle, normal in tags}) != 1:
        return "normals"
    return None


def _settle_polygon_law(polygons, sources, tally):
    """`settle_topology` под `PLANAR_POLYGONS_V1`: четырёхгранья по тому же закону, остальное — как собрано.

    Многоугольник длиннее четырёх (и невыпуклое четырёхгранье) `_polygon_law_faces`
    излучает только на точной плоскости, где `sources` — `(None, None)` на каждой
    вершине; здесь это проверяется, а не предполагается: непланарный многоугольник —
    отказ, а не молчаливый разрез (у него нет канонического `fan_out`).
    """

    split = {"triangles": 0, "normals": 0}
    settled = []
    for face_polygons in polygons:
        kept = []
        for polygon in face_polygons:
            reason = _split_reason(polygon, sources) if len(polygon) >= 4 else None
            if reason is None:
                kept.append(polygon)
            elif len(polygon) == 4:
                kept.extend(fan_out(polygon))
                split[reason] += 1
            else:
                raise MaterializationRefusal(
                    MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
                    f"POLYGON_NOT_PLANAR: {len(polygon)} vertices ({reason})",
                )
        settled.append(tuple(kept))
    return settled, (
        (QUADS_REFUSED_NOT_CONVEX, tally[QUADS_REFUSED_NOT_CONVEX]),
        (QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES, split["triangles"]),
        (QUADS_SPLIT_OFFSET_NORMALS_DIFFER, split["normals"]),
        (POLYGON_FACES_EMITTED, tally[POLYGON_FACES_EMITTED]),
        (POLYGON_FACES_CONCAVE_EMITTED, tally[POLYGON_FACES_CONCAVE_EMITTED]),
        (
            POLYGON_FACES_TRIANGULATED_NOT_SIMPLE,
            tally[POLYGON_FACES_TRIANGULATED_NOT_SIMPLE],
        ),
        (
            POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE,
            tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE],
        ),
        (CURVED_STRIP_FACES_TRIANGULATED, tally[CURVED_STRIP_FACES_TRIANGULATED]),
        (MERGED_RUNS_SPLIT_AT_RUNGS, tally[MERGED_RUNS_SPLIT_AT_RUNGS]),
        (MERGED_RUNS_KEPT_WHOLE, tally[MERGED_RUNS_KEPT_WHOLE]),
        (FAN_FACES_CUT_BY_NEIGHBOUR, tally[FAN_FACES_CUT_BY_NEIGHBOUR]),
        (FAN_POLYGON_FACES_EMITTED, tally[FAN_POLYGON_FACES_EMITTED]),
        (FAN_POLYGON_FACES_CONCAVE_EMITTED, tally[FAN_POLYGON_FACES_CONCAVE_EMITTED]),
        (FAN_FACES_TRIANGULATED_FROM_APEX, tally[FAN_FACES_TRIANGULATED_FROM_APEX]),
        (FAN_FACES_NOT_STAR_FROM_APEX, tally[FAN_FACES_NOT_STAR_FROM_APEX]),
        (QUADS_UV_BILINEAR, tally[QUADS_UV_BILINEAR]),
        (QUADS_UV_BILINEAR_MAX_MILLI_ALPHA, tally[QUADS_UV_BILINEAR_MAX_MILLI_ALPHA]),
        (POLYGONS_UV_BILINEAR, tally[POLYGONS_UV_BILINEAR]),
        (POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA, tally[POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA]),
    )


def settle_topology(
    frame_faces,
    cycles,
    polygons,
    sources,
    law: DecalTopologyLawV1,
    tally: Counter | None = None,
):
    """`(грани, числа закона)`: закон `QUAD_IN_ONE_SOURCE_TRIANGLE_V1` и счёт того, что закон не взял.

    `sources` — `{ключ: (имя исходного треугольника, нормаль смещения)}`, как их
    записал подъём (`lift_vertices`; у плоской укладки везде `(None, None)`).
    Подъём барицентрический В НАЙДЕННОМ треугольнике, поэтому четырёхгранье, все
    четыре вершины которого лежат в ОДНОМ треугольнике источника, плоское в 3D
    (точно; в binary64 — до одного округления на координату);
    иначе оно могло бы изломаться по складке, а Blender разрезал бы его сам, по
    float, и это было бы безымянное разбиение. Грань развёртки хост смещает вдоль
    нормали КАЖДОЙ вершины (закон ядра), и смещённая по разным нормалям грань
    уже не плоская: четырёхгранье требует ещё и ПОБИТОВО равных нормалей смещения.
    Не выполнено любое из двух — грань режется каноническим `fan_out` (ровно
    прежняя тесселяция) и называется своим счётчиком. Точка на ребре источника
    получает канонический (первый по имени) треугольник — разрез консервативен,
    ошибкой он не бывает.

    Остальное, что закон `QUAD_STRIPS_V1` оставил треугольниками, тоже названо:
    строго невыпуклые (и с плоским углом) четырёхугольники ленты и слитые
    пробеги — контуры больше четырёх вершин (счётчик носит имя «слитый пробег»
    по традиции закона: считает он любой контур длиннее четырёх, и поэтому у
    `PLANAR_POLYGONS_V1` его нет — там причины названы поимённо, см.
    `_polygon_law_faces`). Под `PLANAR_POLYGONS_V1` счёт приходит в `tally`.
    """

    if law is DecalTopologyLawV1.PLANAR_POLYGONS_V1:
        return _settle_polygon_law(
            polygons, sources, Counter() if tally is None else tally
        )
    refused = merged = 0
    split = {"triangles": 0, "normals": 0}
    settled = []
    for frame_face, cycle, face_polygons in zip(frame_faces, cycles, polygons):
        if law is DecalTopologyLawV1.QUAD_STRIPS_V1 and not frame_face.is_fan:
            refused += int(len(cycle) == 4 and len(face_polygons) != 1)
            merged += int(len(cycle) > 4)
        kept = []
        for polygon in face_polygons:
            reason = _split_reason(polygon, sources) if len(polygon) == 4 else None
            if reason is None:
                kept.append(polygon)
            else:
                kept.extend(fan_out(polygon))
                split[reason] += 1
        settled.append(tuple(kept))
    return settled, (
        (QUADS_REFUSED_NOT_CONVEX, refused),
        (QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES, split["triangles"]),
        (QUADS_SPLIT_OFFSET_NORMALS_DIFFER, split["normals"]),
        (MERGED_RUN_FACES_TRIANGULATED, merged),
    )


def lift_vertices(points, plane):
    """`({ключ: позиция}, {ключ: (треугольник, нормаль смещения)})`: подъём ровно ОДИН на вершину.

    Счётчики подъёма (`LOCATIONS`, `PREDICATES`) считают точки, а не обращения:
    нахождение (`locate`) делается один раз на вершину и отдаёт и позицию, и
    запись об источнике (имя найденного треугольника, нормаль смещения), поэтому
    закон топологии не прибавляет к ним ничего (записи нужны только
    `settle_topology`). Запись диагностик берётся ПОСЛЕ этого вызова. У плоской
    укладки треугольников источника и нормалей вершин нет: `(None, None)`.
    """

    lifted = {key: plane.lift_named(point) for key, point in points.items()}
    return (
        {key: position for key, (position, _source) in lifted.items()},
        {key: source for key, (_position, source) in lifted.items()},
    )


def _paths(edges):
    """Максимальные неветвящиеся пути по полурёбрам: `[(a, ..., z), ...]`.

    Путь рвётся в вершине, где входов или выходов не один; цикл без разрыва
    замыкается (первый ключ повторён в конце). Порядок детерминирован: старты и
    выходы идут по отсортированным ключам.
    """

    successors: dict[str, list[str]] = {}
    in_degree: Counter = Counter()
    for a, b in sorted(edges):
        successors.setdefault(a, []).append(b)
        in_degree[b] += 1

    def interior(node) -> bool:
        return in_degree[node] == 1 and len(successors.get(node, ())) == 1

    seen: set = set()
    paths = []

    def walk(start, first):
        path = [start, first]
        seen.add((start, first))
        node = first
        while interior(node) and node != start:
            following = successors[node][0]
            seen.add((node, following))
            path.append(following)
            node = following
        return tuple(path)

    for start in sorted(successors):
        if interior(start):
            continue
        for first in successors[start]:
            paths.append(walk(start, first))
    for a, b in sorted(edges):
        if (a, b) not in seen:
            paths.append(walk(a, b))
    return tuple(paths)


def edge_kind(facts, region, a, b, lattice_alpha) -> str:
    """Граничное полуребро: источник (`r = 0`), фронт (`r = alpha`), иначе стена."""

    ra, rb = facts[(region, a)][1], facts[(region, b)][1]
    if ra.is_zero and rb.is_zero:
        return "SOURCE"
    front = SqrtSumV1.rational(lattice_alpha)
    if (ra - front).is_zero and (rb - front).is_zero:
        return "RIM"
    return "WALL"


def chains_of(frame_faces, cycles, layout, facts, lattice_alpha):
    """Граничные и интерфейсные цепи по ключам вершин СЛИТЫХ контуров."""

    owner_of: dict = {}
    for index, cycle in enumerate(cycles):
        size = len(cycle)
        for position in range(size):
            half = (cycle[position][0], cycle[(position + 1) % size][0])
            if half in owner_of:
                raise MaterializationRefusal(
                    MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
                    f"HALF_EDGE_SHARED_IN_ONE_DIRECTION: {half}",
                )
            owner_of[half] = index
    boundary: dict = {}
    interface: dict = {}
    cut_pairs = layout.cut_pairs(frame_faces)
    for (a, b), index in owner_of.items():
        region = layout.region_of(frame_faces[index])
        other = owner_of.get((b, a))
        if other is None:
            kind = edge_kind(facts, region, a, b, lattice_alpha)
            boundary.setdefault((kind, region), []).append((a, b))
            continue
        other_region = layout.region_of(frame_faces[other])
        if other_region != region and region < other_region:
            if frozenset((region, other_region)) in cut_pairs and all(
                facts[(region, key)] == facts[(other_region, key)] for key in (a, b)
            ):
                continue  # технический край разомкнутого кольца: UV на нём не рвётся, это не шов
            interface.setdefault((region, other_region), []).append((a, b))
    boundary_chains = []
    for (kind, region), edges in sorted(boundary.items()):
        for number, path in enumerate(_paths(edges)):
            boundary_chains.append(
                GeometryBoundaryChainV1(
                    SemanticBoundaryId(f"boundary:{kind}:{region}:{number}"),
                    tuple(VertexKey(key) for key in path),
                )
            )
    interface_chains = []
    for (left, right), edges in sorted(interface.items()):
        for number, path in enumerate(_paths(edges)):
            interface_chains.append(
                GeometryInterfaceChainV1(
                    SemanticInterfaceId(f"interface:{left}:{right}:{number}"),
                    tuple(VertexKey(key) for key in path),
                )
            )
    return frozenset(boundary_chains), frozenset(interface_chains)


def _claim_lineage(frame_face) -> tuple:
    """`claim:` записи грани: имя огибающей региона и, у потока, имя экземпляра (спеки, цепи), которое поток заместил."""

    names = [frame_face.claim_key]
    instance = getattr(frame_face, "instance_claim", None)
    if instance is not None and instance != frame_face.claim_key:
        names.append(instance)
    return tuple(f"claim:{name}" for name in names)


def _provenance(frame_face, edge_faces, extra_lineage=()) -> GeometryProvenanceV1:
    source_faces = {
        name
        for edge in frame_face.physical_edge_ids
        for name in edge_faces.get(edge, ())
    }
    return GeometryProvenanceV1(
        source_face_ids=frozenset(SourceFaceId(item) for item in source_faces),
        physical_edge_ids=frozenset(
            PhysicalEdgeId(item) for item in frame_face.physical_edge_ids
        ),
        chain_use_ids=frozenset(ChainUseId(item) for item in frame_face.chain_use_ids),
        lineage_ids=frozenset(
            LineageId(item)
            for item in (*frame_face.run.lineage_ids, *extra_lineage)
        ),
    )


def _merge_provenance(items) -> GeometryProvenanceV1:
    items = tuple(items)
    return GeometryProvenanceV1(
        source_face_ids=frozenset().union(*(i.source_face_ids for i in items)),
        physical_edge_ids=frozenset().union(*(i.physical_edge_ids for i in items)),
        chain_use_ids=frozenset().union(*(i.chain_use_ids for i in items)),
        lineage_ids=frozenset().union(*(i.lineage_ids for i in items)),
    )


def _vertex_records(positions, cycles, face_prov):
    """Вершины батча: позиция уже поднята, происхождение — слияние граней, где она есть."""

    vertex_prov: dict[str, list] = {}
    for prov, cycle in zip(face_prov, cycles):
        for key, _point in cycle:
            vertex_prov.setdefault(key, []).append(prov)
    return frozenset(
        GeometryVertexV1(
            vert_key=VertexKey(key),
            position=position,
            semantic_location_ref=SemanticLocationId(f"location:{key}"),
            provenance=_merge_provenance(vertex_prov[key]),
        )
        for key, position in positions.items()
    )


def _face_records(frame_faces, face_prov, polygons, layout, facts, lattice_alpha, material):
    """Грани батча: по записи на каждый многоугольник тесселяции, UV — на вершину региона."""

    uv_cache: dict = {}

    def uv_of(region, key):
        slot = uv_cache.get((region, key))
        if slot is None:
            s, r = facts[(region, key)]
            slot = uv_cache[(region, key)] = uv_direct_strip_v1(s, r, lattice_alpha)
        return slot

    faces = []
    for item, prov, polygon_list in zip(frame_faces, face_prov, polygons):
        region = layout.region_of(item)
        for polygon in polygon_list:
            faces.append(
                GeometryFaceV1(
                    face_id=GeometryFaceId(f"face:{len(faces)}"),
                    ordered_vert_keys=tuple(VertexKey(k) for k in polygon),
                    uv_facts=tuple(
                        GeometryUvFactV1(VertexKey(k), uv_of(region, k)) for k in polygon
                    ),
                    semantic_region_id=Layout.region_id(region),
                    ownership_claim_id=layout.claim_id(item.claim_key),
                    provenance=prov,
                    material_id=material,
                )
            )
    return tuple(faces)


def assemble_batch(
    *,
    frame_faces,
    cycles,
    positions,
    polygons,
    facts,
    layout: Layout,
    scale: int,
    lattice_alpha: Fraction,
    edge_faces,
    request,
    source_revision,
    patch_domain_id,
    contract_versions,
    diagnostics,
    vertex_cycles=None,
):
    """Все записи батча, без дайджеста: `GeometryBatchV1` с `pending`.

    `positions` — `{ключ: позиция}` ПОДНЯТЫХ вершин (`lift_vertices`), `polygons` —
    грани каждой слитой грани по ключам (`tessellate_faces`): треугольники либо,
    под `QUAD_STRIPS_V1`, четырёхгранья. `diagnostics` — функция без аргументов,
    её зовут ПОСЛЕ подъёма всех точек: счётчики подъёма копятся в подъёме. `vertex_cycles` —
    все вершины каждой грани, если они шире контура (закон `SOURCE_TRIANGLES_CLIPPED_V1`:
    вершина внутри грани — вершина её кусков, но не цепи); без него вершины граней — контуры.
    """

    material = MaterialId(request.material_policy_id.value)
    face_prov = [
        _provenance(item, edge_faces, _claim_lineage(item))
        for item in frame_faces
    ]
    vertices = _vertex_records(
        positions, cycles if vertex_cycles is None else vertex_cycles, face_prov
    )
    region_prov: dict[int, list] = {}
    region_claim: dict[int, str] = {}
    for item, prov in zip(frame_faces, face_prov):
        index = layout.region_of(item)
        region_prov.setdefault(index, []).append(prov)
        region_claim[index] = item.claim_key
    regions = frozenset(
        GeometrySemanticRegionV1(
            Layout.region_id(index),
            layout.claim_id(region_claim[index]),
            material,
            _merge_provenance(region_prov[index]),
        )
        for index in region_prov
    )
    faces = _face_records(
        frame_faces, face_prov, polygons, layout, facts, lattice_alpha, material
    )
    model_of = {
        layout.region_of(item): item.station_model for item in frame_faces
    }
    lineage_of = {
        layout.region_of(item): frozenset(LineageId(x) for x in item.run.lineage_ids)
        for item in frame_faces
    }
    station_facts = frozenset(
        GeometryStationFactV1(
            station_fact_id=GeometryStationFactId(f"station:{region}:{key}"),
            vert_key=VertexKey(key),
            semantic_region_id=Layout.region_id(region),
            ownership_claim_id=layout.claim_id(region_claim[region]),
            source_s=LocalCoordinateV1(decimal_of(s, scale)),
            source_r=LocalCoordinateV1(decimal_of(r, scale)),
            station_model_id=model_of[region],
            lineage_ids=lineage_of[region],
        )
        for (region, key), (s, r) in facts.items()
    )
    boundary_chains, interface_chains = chains_of(
        frame_faces, cycles, layout, facts, lattice_alpha
    )
    return GeometryBatchV1(
        schema_version=GEOMETRY_BATCH_SCHEMA_V1,
        source_revision=source_revision,
        decal_request_id=request.decal_request_id,
        patch_domain_id=patch_domain_id,
        vertices=vertices,
        faces=faces,
        station_facts=station_facts,
        semantic_regions=regions,
        boundary_chains=boundary_chains,
        interface_chains=interface_chains,
        diagnostics=frozenset(diagnostics()),
        contract_versions=frozenset(ContractVersionId(x) for x in contract_versions),
        semantic_digest=SemanticDigestValue("pending"),
    )
