"""Immutable surface contracts shared by analysis and decal runtime.

The module is deliberately Blender-free. Analysis is the only writer;
rail/chart compilers consume emitted polygon cycles and loop triangles
without reconstructing source topology from derived triangulation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .model import PatchGraph


Vec3 = tuple[float, float, float]


class AnalysisSchemaError(ValueError):
    """Analysis output cannot be consumed by the current decal runtime."""

    def __init__(self, reason: str, details: str = ""):
        self.reason = str(reason)
        self.details = str(details)
        message = self.reason
        if self.details:
            message += f": {self.details}"
        super().__init__(message)


class AnalysisCrossIrError(ValueError):
    """PatchGraph and PatchSurfaceIR disagree about one source revision."""

    def __init__(self, invariant: str, details: str):
        self.invariant = str(invariant)
        self.reason = "DECAL_ANALYSIS_CROSS_IR_INVALID"
        self.details = str(details)
        super().__init__(f"{self.reason}:{self.invariant}: {self.details}")


@dataclass(frozen=True)
class SourceRevision:
    """Stable identity of the source topology/coordinates used by analysis."""

    source_name: str
    digest: str


@dataclass(frozen=True)
class AnalysisCapabilities:
    """Versioned schemas advertised by one analysis result."""

    patch_graph_schema: int = 1
    patch_surface_schema: int = 1
    geometry_batch_schema: int = 1

    def require_supported(self) -> None:
        expected = (1, 1, 1)
        actual = (
            int(self.patch_graph_schema),
            int(self.patch_surface_schema),
            int(self.geometry_batch_schema),
        )
        if actual != expected:
            raise AnalysisSchemaError(
                "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED",
                f"expected={expected} actual={actual}",
            )


@dataclass(frozen=True, order=True)
class SourceVertex:
    vertex_id: int
    position: Vec3


@dataclass(frozen=True, order=True)
class SourceEdge:
    edge_id: int
    vertex_ids: tuple[int, int]
    source_face_ids: tuple[int, ...]


@dataclass(frozen=True)
class SourceFace:
    face_id: int
    patch_id: int
    vertex_cycle: tuple[int, ...]
    edge_cycle: tuple[int, ...]
    polygon_normal: Vec3
    triangle_ids: tuple[int, ...]


@dataclass(frozen=True)
class SurfaceTriangle:
    triangle_id: int
    source_face_id: int
    vertex_ids: tuple[int, int, int]
    # Indexed by the opposite triangle vertex. Blender tessellation
    # diagonals are None: they are not physical source edges.
    physical_edge_ids: tuple[int | None, int | None, int | None]
    triangle_normal: Vec3


@dataclass(frozen=True)
class PatchSurfaceIR:
    """Authoritative source surface: polygon cycles plus Blender triangles."""

    source_revision: SourceRevision
    vertices: tuple[SourceVertex, ...]
    edges: tuple[SourceEdge, ...]
    faces: tuple[SourceFace, ...]
    triangles: tuple[SurfaceTriangle, ...]

    @property
    def vertex_by_id(self) -> dict[int, SourceVertex]:
        return {vertex.vertex_id: vertex for vertex in self.vertices}

    @property
    def edge_by_id(self) -> dict[int, SourceEdge]:
        return {edge.edge_id: edge for edge in self.edges}

    @property
    def face_by_id(self) -> dict[int, SourceFace]:
        return {face.face_id: face for face in self.faces}

    @property
    def triangle_by_id(self) -> dict[int, SurfaceTriangle]:
        return {triangle.triangle_id: triangle for triangle in self.triangles}

    def patch_faces(self, patch_id: int) -> tuple[SourceFace, ...]:
        return tuple(face for face in self.faces if face.patch_id == int(patch_id))

    def patch_triangles(self, patch_id: int) -> tuple[SurfaceTriangle, ...]:
        face_ids = {face.face_id for face in self.patch_faces(patch_id)}
        return tuple(
            triangle
            for triangle in self.triangles
            if triangle.source_face_id in face_ids
        )


@dataclass(frozen=True)
class AnalysisBundle:
    """Atomic analysis output: topology and surface from one revision."""

    source_revision: SourceRevision
    patch_graph: PatchGraph
    patch_surface: PatchSurfaceIR
    capabilities: AnalysisCapabilities = AnalysisCapabilities()

    def __post_init__(self) -> None:
        self.capabilities.require_supported()
        graph_revision = getattr(self.patch_graph, "source_revision", None)
        if graph_revision is not self.source_revision:
            raise AnalysisCrossIrError(
                "REVISION_IDENTITY",
                "PatchGraph does not carry the bundle SourceRevision instance",
            )
        if self.patch_surface.source_revision is not self.source_revision:
            raise AnalysisCrossIrError(
                "REVISION_IDENTITY",
                "PatchSurfaceIR does not carry the bundle SourceRevision instance",
            )

    def __getattr__(self, name):
        """Topology-only compatibility view for existing solve consumers."""

        return getattr(self.patch_graph, name)


class HostPlanarityPolicy(str, Enum):
    """Какую плоскость хост объявляет ядру для патча.

    Объявляется явно, а не выбирается ядром при отказе: координаты Blender —
    binary64 без гарантии компланарности, поэтому строгая приёмка отвергала
    почти любой отредактированный меш.
    """

    EXACT_SOURCE_PLANE_V1 = "EXACT_SOURCE_PLANE_V1"
    NEAR_PLANAR_PROJECTION_V1 = "NEAR_PLANAR_PROJECTION_V1"


HOST_PLANARITY_POLICY = HostPlanarityPolicy.NEAR_PLANAR_PROJECTION_V1


class HostGridPolicy(str, Enum):
    """Привязывает ли хост вершины источника к целочисленной решётке.

    Объявляется явно по тем же причинам, что и планарность: ядро не выбирает
    политику за хост, и переключение видно в сертификате метрики, а значит и в
    дайджесте.
    """

    UNSNAPPED_EXACT_V1 = "UNSNAPPED_EXACT_V1"
    # Привязывается только источник; конструкции остаются точными.
    SOURCE_ONLY_GRID_SNAP_V1 = "SOURCE_ONLY_GRID_SNAP_V1"
    INTEGER_GRID_SNAP_V1 = "INTEGER_GRID_SNAP_V1"


# Хост запрашивает привязку ИСТОЧНИКА и не запрашивает привязку конструкций.
# Разрез не выбран, а измерен на `building.002` — единственном полевом меше:
#
#   UNSNAPPED_EXACT_V1        EXACT, восстановлено 0 из 3 задуманно прямых
#   SOURCE_ONLY_GRID_SNAP_V1  EXACT, восстановлено 3 из 3, топология та же
#                             (3 петли, 3 региона, 2 точечных контакта)
#   INTEGER_GRID_SNAP_V1      REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN
#
# То есть выигрыш даёт привязка источника, а отказ приносит привязка
# конструкций — `offset_support_g` и `segment_intersections`, то самое слияние
# вычисленных точек, которое карточка R1b сама числит лотереей, а не
# механизмом: на полевом меше оно сливает три вычисленные точки и оставляет
# две висячие полурёбра вместо замкнутой границы.
#
# `INTEGER_GRID_SNAP_V1` не удалён: он нужен, когда привязку конструкций
# починят topology-preserving snap rounding'ом. До тех пор его отказ сторожит
# `kernel/tests/test_grid_wiring.py::test_the_field_mesh_still_refuses_downstream_of_the_restored_corners`,
# а выигрыш нового закона —
# `...::test_the_field_mesh_keeps_its_topology_when_only_the_source_is_snapped`.
HOST_GRID_POLICY = HostGridPolicy.SOURCE_ONLY_GRID_SNAP_V1


class HostNearPlanarLiftPolicy(str, Enum):
    """На какую поверхность хост просит класть меш near-planar домена.

    Объявляется явно, как планарность и решётка: ядро не выбирает политику за
    хост, и она видна и в сертификате метрики (`lift_law`), и в исходе
    материализации (`NEAR_PLANAR_LIFT_ONTO_SOURCE_TRIANGLES`).
    """

    CERTIFIED_PLANE_V1 = "CERTIFIED_PLANE_V1"
    SOURCE_TRIANGLES_V1 = "SOURCE_TRIANGLES_V1"
    SOURCE_TRIANGLES_CLIPPED_V1 = "SOURCE_TRIANGLES_CLIPPED_V1"
    SOURCE_FACES_CLIPPED_V1 = "SOURCE_FACES_CLIPPED_V1"


# Хост просит укладку на ТРЕУГОЛЬНИКИ ИСТОЧНИКА С РЕЗКОЙ граней (NEAR_PLANAR V2, решение
# владельца 2026-10-02; резка — 2026-10-03, «от триангуляции всё ещё есть на декалях
# (на криволинейных поверхностях точно)»): меш лежит на поверхности, абсолютная невязка
# плоскости 1.25 см — записанная диагностика, а судят искажение ширины (2 % относительно)
# и вложение проекции. Грань, пересекающая ребро источника, режется вершиной на нём, и
# каждый кусок лежит в ОДНОМ треугольнике источника: хорда через сгиб больше не срезает
# стену, а кусок плоский точно и остаётся четырёхгранью. Суд и метрика те же, что у
# `SOURCE_TRIANGLES_V1` (сертификат пишет его), поэтому закон возвращается одной строкой.
# 2026-10-03, «лишние рёбра на кривых»: диагональ четырёхгранья — ребро триангуляции хоста, а не
# меша, и резать по ней нельзя. `SOURCE_FACES_CLIPPED_V1` режет только по рёбрам, общим у граней
# источника: кусок лежит в одной ЗАМКНУТОЙ грани, а грань, непланарная глубже четверти смещения
# (5 мм, умолчание в ожидании владельца), режется по своим треугольникам под счётчиком.
# Прежние законы остаются членами перечисления: они нужны отладочным сценам и красным контролям.
HOST_NEAR_PLANAR_LIFT_POLICY = HostNearPlanarLiftPolicy.SOURCE_FACES_CLIPPED_V1


class HostNearPlanarFramePolicy(str, Enum):
    """Каким репером хост просит описывать near-planar карту.

    Объявляется явно, как остальные политики: ядро не выбирает закон репера за
    хост, и применённый закон виден в метрике (`frame_selection_law`).
    """

    CANONICAL_ONLY_V1 = "CANONICAL_ONLY_V1"
    REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1 = "REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1"


# Приведённый целочисленный базис плоскости (NEAR_PLANAR V2, коммит 4). Репер от
# разностей спроецированных вершин наследует знаменатели проекции (деление на
# `n·n`), матрица Грама возводит их в квадрат, и радиканды `SqrtSumV1` растут:
# Ро-Поллард `building.004` patch 4 не возвращается за кап. Приведённый базис
# оставляет вершины на месте (они точно те же) и сжимает Грам. Замер на `building`
# 106/109/120/121: радиканды 248/227/264/248 -> 145/138/143/142 бит (DECISIONS
# 2026-10-03). У точной плоскости закон не применяется, и её байты прежние.
HOST_NEAR_PLANAR_FRAME_POLICY = HostNearPlanarFramePolicy.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1


class HostCurvatureLadderPolicy(str, Enum):
    """Что хост просит ядро пробовать ПОСЛЕ именованного отказа near-planar.

    Объявляется явно, как остальные политики: ядро не выбирает политику за хост, и
    применённый закон виден в сертификате метрики. `NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1`
    — лестница EXACT -> NEAR_PLANAR -> DEVELOPABLE: развёртка пробуется только после
    отказа near-planar по ширине, перевороту треугольника или вложению проекции, поэтому
    домен, принятый сегодня, не перемаршрутизируется и его байты прежние.
    """

    NEAR_PLANAR_ONLY_V1 = "NEAR_PLANAR_ONLY_V1"
    NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1 = "NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1"


# Хост просит лестницу кривизны с развёрткой (S1 DEVELOPABLE_UNFOLDED_V1, решение владельца
# 2026-10-03): домен, отвергнутый near-planar по ширине (патч 89 `building`: ступенька с
# перпендикулярным треугольником), получает ещё одну попытку - привязанную к решётке
# шарнирную развёртку, которую судит точное растяжение (1/50 в обе стороны). Смещение
# декали над развёрнутым доменом идёт по нормали ВЕРШИНЫ (углы-веса), не плоскости.
HOST_CURVATURE_LADDER_POLICY = HostCurvatureLadderPolicy.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1


class HostDecalTopologyPolicy(str, Enum):
    """Из каких граней хост просит собрать сетку декали.

    Объявляется явно, как остальные политики: ядро не выбирает закон топологии за
    хост, и применённый закон виден в исходе материализации
    (`MaterializationV1.decal_topology_law`), в строке JSON и в квитанции писателя.
    """

    TRIANGLES_V1 = "TRIANGLES_V1"
    QUAD_STRIPS_V1 = "QUAD_STRIPS_V1"
    PLANAR_POLYGONS_V1 = "PLANAR_POLYGONS_V1"


# Кнопка «Build Decal Mesh» просит ПЛОСКИЕ МНОГОУГОЛЬНИКИ лент (решение владельца 2026-10-03,
# «от триангуляции всё ещё есть на декалях»): слитый пробег режется по перекладинам в грани
# рёбер-источников, а лента на точной плоскости остаётся одним многоугольником любой длины,
# выпуклым или нет, если контур прост и UV в нём аффинен по положению на карте (допуск
# `PLANAR_AFFINE_UV_POLYGON_V1`); веера и кривые домены остаются треугольниками под названными
# счётчиками. Сетка вершин, UV, швы и семантический дайджест те же, что у `TRIANGLES_V1` и
# `QUAD_STRIPS_V1` (ворота ядра), поэтому закон возвращается одной строкой, а прежние остаются
# членами перечисления для отладочных сцен и сверок.
HOST_DECAL_TOPOLOGY_POLICY = HostDecalTopologyPolicy.PLANAR_POLYGONS_V1


__all__ = (
    "AnalysisBundle",
    "AnalysisCapabilities",
    "AnalysisCrossIrError",
    "AnalysisSchemaError",
    "HOST_CURVATURE_LADDER_POLICY",
    "HOST_DECAL_TOPOLOGY_POLICY",
    "HOST_GRID_POLICY",
    "HOST_NEAR_PLANAR_FRAME_POLICY",
    "HOST_NEAR_PLANAR_LIFT_POLICY",
    "HOST_PLANARITY_POLICY",
    "HostCurvatureLadderPolicy",
    "HostDecalTopologyPolicy",
    "HostGridPolicy",
    "HostNearPlanarFramePolicy",
    "HostNearPlanarLiftPolicy",
    "HostPlanarityPolicy",
    "PatchSurfaceIR",
    "SourceEdge",
    "SourceFace",
    "SourceRevision",
    "SourceVertex",
    "SurfaceTriangle",
    "Vec3",
)
