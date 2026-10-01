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
from .coalesce import point_key
from .frames import FrameFaceV1, MaterializationRefusal
from .lift import ENCLOSURE_BITS
from .stations import ChainStationTableV1, station_of, transverse_of, transverse_root
from .tessellate import convex_quad_ring, fan_out, triangulate_exact
from .uv_law import uv_direct_strip_v1
from ..wavefront.faces import doubled_shoelace

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


def _lattice_node(point):
    x, y = point[0].as_rational(), point[1].as_rational()
    if x is None or y is None or x.denominator != 1 or y.denominator != 1:
        return None
    return (int(x), int(y))


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


def intern_vertices(items, table: ChainStationTableV1, notes: list | None = None):
    """Ключи вершин по точкам контуров: `([цикл граней], {ключ: точка})`.

    `items` — `[(region_id, FrameFaceV1)]`. Порядок ключей `node:` — порядок
    первого появления точки при обходе граней и их вершин.

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
            node = _lattice_node(point)
            vertex_id = (
                None if node is None else table.node_vertex_ids.get((region_id, node))
            )
            key = interned.get(pk)
            if key is None:
                if vertex_id is not None:
                    key = f"src:{vertex_id}"
                    if key in points:
                        raise MaterializationRefusal(
                            MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
                            f"VERTEX_KEY_COLLISION: {key} names two different "
                            f"points (region {region_id}, node {node})",
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
                    f"{SOURCE_VERTEX_NAME_DROPPED}: {vertex_id} at node {node} of "
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

    def claim_id(self, name: str) -> OwnershipClaimId:
        return OwnershipClaimId(f"claim:{self.claim[name]}")

    @staticmethod
    def region_id(index: int) -> SemanticRegionId:
        return SemanticRegionId(f"region:{index}")


def station_values(frame_faces, cycles, layout, table, lattice_alpha, budget):
    """`{(регион, ключ): (s, r)}` в единицах решётки, точные. Конфликт — отказ."""

    facts: dict = {}
    roots: dict = {}
    for frame_face, cycle in zip(frame_faces, cycles):
        region = layout.region_of(frame_face)
        line = frame_face.line
        root = roots.get(line.q)
        if root is None:
            root = roots[line.q] = transverse_root(line, budget)
        for key, point in cycle:
            station = (
                frame_face.fan_station
                if frame_face.is_fan
                else station_of(frame_face.run, point)
            )
            value = (station, transverse_of(line, point, root))
            known = facts.setdefault((region, key), value)
            if known != value:
                raise MaterializationRefusal(
                    MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
                    f"STATION_VALUE_CONFLICT: vertex {key} in region {region} "
                    f"has two (s, r) answers from owner {frame_face.face.owner}",
                )
    return facts


def _closes_the_face(total, frame_face, parts: str) -> None:
    """Сумма удвоенных площадей частей разбиения равна площади грани: точно, иначе отказ."""

    if not (total - frame_face.face.doubled_area).is_zero:
        raise MaterializationRefusal(
            MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
            f"owner {frame_face.face.owner}: {parts} differ from the face",
        )


def _quad_polygon(keys, ring, reverse: bool):
    """Четырёхгранья по кольцу против часовой; при `reverse` — обход, сохраняющий первую вершину."""

    first, second, third, fourth = (keys[index] for index in ring)
    return (first, fourth, third, second) if reverse else (first, second, third, fourth)


def _triangle_polygons(frame_face, points, keys, budget, reverse: bool):
    """Треугольники грани по ключам: отсечение ушей, сумма площадей — точное равенство."""

    triangles = triangulate_exact(points, budget) if len(points) >= 3 else None
    if triangles is None:
        raise MaterializationRefusal(
            MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
            f"owner {frame_face.face.owner}: {len(points)} contour points",
        )
    total = SqrtSumV1.zero()
    for first, second, third in triangles:
        total = total + doubled_shoelace(
            (points[first], points[second], points[third])
        )
    _closes_the_face(total, frame_face, "triangle areas")
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
):
    """Грани каждой слитой грани по КЛЮЧАМ вершин: `[(грань, ...), ...]`. Не сложилось — отказ.

    Под `TRIANGLES_V1` — только треугольники. Под `QUAD_STRIPS_V1` строго
    выпуклый четырёхугольник ЛЕНТЫ (не веер) остаётся одной четырёхгранью, и
    площадь проверяется по контуру целиком; всё остальное — треугольники, как
    раньше. Подъём вершин здесь не виден: четырёхгранья, чьи вершины лягут на
    разные треугольники источника, режутся позже (`settle_topology`).
    """

    result = []
    for frame_face, cycle in zip(frame_faces, cycles):
        points = tuple(point for _key, point in cycle)
        keys = tuple(key for key, _point in cycle)
        ring = (
            convex_quad_ring(points, budget)
            if law is DecalTopologyLawV1.QUAD_STRIPS_V1 and not frame_face.is_fan
            else None
        )
        if ring is None:
            result.append(_triangle_polygons(frame_face, points, keys, budget, reverse))
            continue
        _closes_the_face(
            doubled_shoelace(tuple(points[index] for index in ring)),
            frame_face,
            "quad areas",
        )
        result.append((_quad_polygon(keys, ring, reverse),))
    return result


#: Имена чисел закона топологии (они же ключи счётчиков материализатора).
QUADS_REFUSED_NOT_CONVEX = "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX"
QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES = "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"
MERGED_RUN_FACES_TRIANGULATED = "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED"


def settle_topology(frame_faces, cycles, polygons, names, law: DecalTopologyLawV1):
    """`(грани, числа закона)`: закон `QUAD_IN_ONE_SOURCE_TRIANGLE_V1` и счёт того, что закон не взял.

    `names` — `{ключ: имя исходного треугольника, в котором вершина найдена}`
    (`lift_vertices`; у плоской укладки везде `None`). Подъём барицентрический
    В НАЙДЕННОМ треугольнике, поэтому четырёхгранья, все четыре вершины которого
    лежат в ОДНОМ треугольнике источника, — плоское в 3D; иначе оно могло бы
    изломаться по складке, а Blender разрезал бы его сам, по float, и это было
    бы безымянное разбиение. Такое четырёхгранье режется каноническим
    `fan_out` (ровно прежняя тесселяция) и называется счётчиком. Точка на ребре
    источника получает канонический (первый по имени) треугольник — разрез
    консервативен, ошибкой он не бывает.

    Остальное, что закон `QUAD_STRIPS_V1` оставил треугольниками, тоже названо:
    строго невыпуклые (и с плоским углом) четырёхугольники ленты и слитые
    пробеги — контуры больше четырёх вершин (их разбиение — отдельный срез).
    """

    refused = merged = split = 0
    settled = []
    for frame_face, cycle, face_polygons in zip(frame_faces, cycles, polygons):
        if law is DecalTopologyLawV1.QUAD_STRIPS_V1 and not frame_face.is_fan:
            refused += int(len(cycle) == 4 and len(face_polygons) != 1)
            merged += int(len(cycle) > 4)
        kept = []
        for polygon in face_polygons:
            if len(polygon) == 4 and len({names[key] for key in polygon}) != 1:
                kept.extend(fan_out(polygon))
                split += 1
            else:
                kept.append(polygon)
        settled.append(tuple(kept))
    return settled, (
        (QUADS_REFUSED_NOT_CONVEX, refused),
        (QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES, split),
        (MERGED_RUN_FACES_TRIANGULATED, merged),
    )


def lift_vertices(points, plane):
    """`({ключ: позиция}, {ключ: имя исходного треугольника})`: подъём ровно ОДИН на вершину.

    Счётчики подъёма (`LOCATIONS`, `PREDICATES`) считают точки, а не обращения:
    нахождение (`locate`) делается один раз на вершину и отдаёт и позицию, и имя
    найденного треугольника, поэтому закон топологии не прибавляет к ним ничего
    (имена нужны только `settle_topology`). Запись диагностик берётся ПОСЛЕ
    этого вызова. У плоской укладки треугольников источника нет, имя — `None`.
    """

    lifted = {key: plane.lift_named(point) for key, point in points.items()}
    return (
        {key: position for key, (position, _name) in lifted.items()},
        {key: name for key, (_position, name) in lifted.items()},
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


def _edge_kind(facts, region, a, b, lattice_alpha) -> str:
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
    for (a, b), index in owner_of.items():
        region = layout.region_of(frame_faces[index])
        other = owner_of.get((b, a))
        if other is None:
            kind = _edge_kind(facts, region, a, b, lattice_alpha)
            boundary.setdefault((kind, region), []).append((a, b))
            continue
        other_region = layout.region_of(frame_faces[other])
        if other_region != region and region < other_region:
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
):
    """Все записи батча, без дайджеста: `GeometryBatchV1` с `pending`.

    `positions` — `{ключ: позиция}` ПОДНЯТЫХ вершин (`lift_vertices`), `polygons` —
    грани каждой слитой грани по ключам (`tessellate_faces`): треугольники либо,
    под `QUAD_STRIPS_V1`, четырёхгранья. `diagnostics` — функция без аргументов,
    её зовут ПОСЛЕ подъёма всех точек: счётчики подъёма копятся в подъёме.
    """

    material = MaterialId(request.material_policy_id.value)
    face_prov = [
        _provenance(item, edge_faces, (f"claim:{item.claim_key}",))
        for item in frame_faces
    ]
    vertices = _vertex_records(positions, cycles, face_prov)
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
