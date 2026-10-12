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
from fractions import Fraction

from ..canonical import PENDING_SEMANTIC_DIGEST
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
from .audit import location_key as _location_key
from .frames import MaterializationRefusal
from .mesh import (
    CURVED_STRIP_FACES_TRIANGULATED,
    FAN_FACES_CUT_BY_NEIGHBOUR,
    FAN_FACES_NOT_STAR_FROM_APEX,
    FAN_FACES_TRIANGULATED_FROM_APEX,
    FAN_POLYGON_FACES_CONCAVE_EMITTED,
    FAN_POLYGON_FACES_EMITTED,
    MERGED_RUNS_KEPT_WHOLE,
    MERGED_RUNS_SPLIT_AT_RUNGS,
    POLYGON_FACES_CONCAVE_EMITTED,
    POLYGON_FACES_EMITTED,
    POLYGON_FACES_TRIANGULATED_NOT_SIMPLE,
    POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE,
    POLYGONS_UV_BILINEAR,
    POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA,
    QUADS_REFUSED_NOT_CONVEX,
    QUADS_UV_BILINEAR,
    QUADS_UV_BILINEAR_MAX_MILLI_ALPHA,
    decimal_of,
    # a re-export: `clip.py` (a pinned file of the clip port) imports it from here, and a pinned file is not edited for a move
    station_values as station_values,
)
from .convex_partition import partition_counters
from .tessellate import fan_out
from .uv_law import uv_direct_strip_v1


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
        *partition_counters(tally),
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


def paths_of(edges):
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


def _own_half_edge(owner_of: dict, half: tuple, entry: tuple) -> None:
    if half in owner_of:
        raise MaterializationRefusal(
            MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
            f"HALF_EDGE_SHARED_IN_ONE_DIRECTION: {half}",
        )
    owner_of[half] = entry


def chains_of(frame_faces, cycles, layout, facts, lattice_alpha):
    """Граничные и интерфейсные цепи по ключам вершин СЛИТЫХ контуров.

    Полуребро знает место своих концов (`_location_key`): две копии вершины разреза кольца - одно место, поэтому ребро
    разреза с двух сторон - ОДНО ребро между гранями (интерфейс, где станции `(s, r)` разные), а не две стены границы.
    Ключи цепи остаются ключами вершин батча (первой из двух сторон).
    """

    owner_of: dict = {}
    for index, cycle in enumerate(cycles):
        size = len(cycle)
        for position in range(size):
            first, second = cycle[position][0], cycle[(position + 1) % size][0]
            _own_half_edge(owner_of, (_location_key(first), _location_key(second)), (index, first, second))
    return _chains_from(owner_of, frame_faces, layout, facts, lattice_alpha)


def chains_of_faces(frame_faces, polygons, layout, facts, lattice_alpha, continuous: Counter, reverse: bool = False):
    """Граничные и интерфейсные цепи по ИТОГОВЫМ граням (кольцам многоугольников каждой слитой грани), закон `SILHOUETTE_TOPOLOGY_V1`.

    Закон растворяет рёбра между гранями одного региона и вершины между двумя рёбрами, поэтому контур слитой грани (он описывает
    ГРАНИ ДО растворения) уже не говорит, где в итоговой сетке кончается ребро: узел контуров нескольких слитых граней одного
    региона после растворения рёбер между ними — вершина степени два, и цепи строятся по кольцам итога, а не по контурам.
    Полуребро грани номер `index` принадлежит слитой грани `index` (тем же порядком, что у `chains_of`). Кольца граней обходятся
    против контуров на CW-карте (`reverse`: так их кладёт тесселяция, чтобы нормаль смотрела наружу), и цепи идут в направлении
    контуров, как у `chains_of`: направление цепи входит в дайджест батча.

    ШОВ — ТАМ, ГДЕ UV РВЁТСЯ. Общее ребро граней двух РАЗНЫХ регионов, у которого факты `(s, r)` на обоих концах ТОЧНО равны
    (UV на нём одна с обеих сторон: граница владения между секторами одного веера, технический край разомкнутого кольца), не
    интерфейс: шва там нет, а хост метит швом ровно интерфейсные цепи. Сколько таких рёбер не вошло в интерфейсы,
    записывается в `continuous` (`CONTINUOUS_INTERFACE_EDGES`): каждый такой край назван числом, а не потерян.
    """

    owner_of: dict = {}
    for index, face_polygons in enumerate(polygons):
        for ring in face_polygons:
            size = len(ring)
            for position in range(size):
                first, second = ring[position], ring[(position + 1) % size]
                if reverse:
                    first, second = second, first
                _own_half_edge(owner_of, (_location_key(first), _location_key(second)), (index, first, second))
    return _chains_from(owner_of, frame_faces, layout, facts, lattice_alpha, continuous)


#: Число закона `SILHOUETTE_TOPOLOGY_V1`: рёбра между регионами с ТОЧНО непрерывной UV, не ставшие интерфейсом (шва нет).
CONTINUOUS_INTERFACE_EDGES = "MATERIALIZE_SILHOUETTE_CONTINUOUS_INTERFACE_EDGES"


def _chains_from(owner_of, frame_faces, layout, facts, lattice_alpha, continuous=None):
    boundary: dict = {}
    interface: dict = {}
    cut_pairs = layout.cut_pairs(frame_faces)
    for (place_a, place_b), (index, a, b) in owner_of.items():
        region = layout.region_of(frame_faces[index])
        opposite = owner_of.get((place_b, place_a))
        if opposite is None:
            kind = edge_kind(facts, region, a, b, lattice_alpha)
            boundary.setdefault((kind, region), []).append((a, b))
            continue
        other, other_start, other_end = opposite
        other_region = layout.region_of(frame_faces[other])
        if (region, index) < (other_region, other) and (other_region != region or (a, b) != (other_end, other_start)):
            if other_region != region and (continuous is not None or frozenset((region, other_region)) in cut_pairs) and all(
                facts[(region, key)] == facts[(other_region, twin)]
                for key, twin in ((a, other_end), (b, other_start))
            ):
                # UV на ребре не рвётся, это не шов: технический край разомкнутого кольца потока, а под законом силуэта —
                # любое ребро между регионами с равными фактами (границы владения секторов одного веера)
                if continuous is not None and frozenset((region, other_region)) not in cut_pairs:
                    continuous[CONTINUOUS_INTERFACE_EDGES] += 1
                continue
            interface.setdefault((region, other_region), []).append((a, b))
    boundary_chains = []
    for (kind, region), edges in sorted(boundary.items()):
        for number, path in enumerate(paths_of(edges)):
            boundary_chains.append(
                GeometryBoundaryChainV1(
                    SemanticBoundaryId(f"boundary:{kind}:{region}:{number}"),
                    tuple(VertexKey(key) for key in path),
                )
            )
    interface_chains = []
    for (left, right), edges in sorted(interface.items()):
        for number, path in enumerate(paths_of(edges)):
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
            semantic_location_ref=SemanticLocationId(f"location:{_location_key(key)}"),
            provenance=_merge_provenance(vertex_prov[key]),
        )
        for key, position in positions.items()
    )


def _face_records(frame_faces, face_prov, polygons, layout, facts, lattice_alpha, material, merged=None):
    """Грани батча: по записи на каждый многоугольник тесселяции, UV — на вершину региона.

    `merged` — `{(номер слитой грани, многоугольник): (номера других слитых граней)}`: грань, в которую закон
    `SILHOUETTE_TOPOLOGY_V1` растворил ребро между слитыми гранями одного региона, несёт происхождение всех.
    """

    uv_cache: dict = {}

    def uv_of(region, key):
        slot = uv_cache.get((region, key))
        if slot is None:
            s, r = facts[(region, key)]
            slot = uv_cache[(region, key)] = uv_direct_strip_v1(s, r, lattice_alpha)
        return slot

    faces = []
    for index, (item, prov, polygon_list) in enumerate(zip(frame_faces, face_prov, polygons)):
        region = layout.region_of(item)
        for polygon in polygon_list:
            others = () if merged is None else merged.get((index, tuple(polygon)), ())
            faces.append(
                GeometryFaceV1(
                    face_id=GeometryFaceId(f"face:{len(faces)}"),
                    ordered_vert_keys=tuple(VertexKey(k) for k in polygon),
                    uv_facts=tuple(
                        GeometryUvFactV1(VertexKey(k), uv_of(region, k)) for k in polygon
                    ),
                    semantic_region_id=Layout.region_id(region),
                    ownership_claim_id=layout.claim_id(item.claim_key),
                    provenance=_merge_provenance((prov, *(face_prov[other] for other in others))) if others else prov,
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
    merged_frames=None,
    chains=None,
):
    """Все записи батча, без дайджеста: `GeometryBatchV1` с `pending`.

    `positions` — `{ключ: позиция}` ПОДНЯТЫХ вершин (`lift_vertices`), `polygons` —
    грани каждой слитой грани по ключам (`tessellate_faces`): треугольники либо,
    под `QUAD_STRIPS_V1`, четырёхгранья. `diagnostics` — функция без аргументов,
    её зовут ПОСЛЕ подъёма всех точек: счётчики подъёма копятся в подъёме. `vertex_cycles` —
    все вершины каждой грани, если они шире контура (закон `SOURCE_TRIANGLES_CLIPPED_V1`:
    вершина внутри грани — вершина её кусков, но не цепи); без него вершины граней — контуры. `merged_frames` — происхождение
    граней, слитых законом `SILHOUETTE_TOPOLOGY_V1` из нескольких слитых граней (`_face_records`). `chains` — `(граничные,
    интерфейсные)`, построенные законом `SILHOUETTE_TOPOLOGY_V1` по кольцам итоговых граней (`chains_of_faces`); без них цепи
    строятся по контурам (`chains_of`).
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
        frame_faces, face_prov, polygons, layout, facts, lattice_alpha, material, merged_frames
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
    boundary_chains, interface_chains = (
        chains_of(frame_faces, cycles, layout, facts, lattice_alpha) if chains is None else chains
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
        semantic_digest=SemanticDigestValue(PENDING_SEMANTIC_DIGEST),
    )
