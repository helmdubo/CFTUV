"""Patch-bounded segment-Voronoi backend для Decal Seams.

Диаграмма строится один раз на owner patch по выбранным boundary segments.
Во время modal drag segment- и endpoint-cells обрезаются динамическими
extrusion polygons текущей ширины. Ячейки заранее пересечены с boundary-доменом
patch, поэтому крылья не выходят за mesh boundary, не пересекаются внутри patch
и меняют топологию в тот же момент, когда фронты встречаются. Внутренняя
триангуляция домена используется только для clipping и сваривается обратно:
topology исходного mesh не должна отпечатываться на итоговой декали.

``pyvoronoi`` изолирован в этом модуле. Если wheel недоступен либо owner
surface не поддержана, compile возвращает структурированный отказ; production
SEAMS runtime больше не имеет legacy backend для маскировки такого отказа.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass, field, replace
from enum import Enum
from fractions import Fraction
from heapq import heappop, heappush
from math import atan2, cos, gcd, isfinite, pi, sqrt, tau

from mathutils import Vector

try:
    from mathutils.geometry import tessellate_polygon as _tessellate_polygon
except ImportError:  # Unit tests используют минимальный mathutils stub.
    _tessellate_polygon = None

from .constants import DECAL_COPLANAR_DOT, DECAL_WELD_DISTANCE
from .decal_diagram import (
    DIAGRAM_INT_LIMIT as _DIAGRAM_INT_LIMIT,
    DiagramTransform,
    DiagramTransformError,
    build_diagram_transform,
)
from .decal_chart_admission import (
    CHART_DISTORTION_BUDGET,
    admit_intrinsic_strip_runtime,
)
from .decal_atlas import IntrinsicStripAtlas
from .decal_charts import (
    ChartBuildFailure,
    ChartCut,
    ChartSiteSeed,
    build_intrinsic_strip_charts,
    validate_periodic_chart_fields,
)
from .decal_geometry import (
    DecalGeometryFace,
    DomainLocation,
    lift_offset_position,
    polygon_area2,
    segment_point_distance2,
)
from .decal_corner_model import (
    BandSide,
    CornerModel,
    CornerPointProvenance,
    CornerSeed,
    CornerStationRef,
    CornerStripVertex,
    CornerVertexRef,
    LocalClippedCornerStrip,
)
from .model import CornerJoinMode
from .surface_ir import (
    AnalysisBundle,
    AnalysisSchemaError,
    CapacityPolicy,
    DecalBackendKind,
)
from .decal_rails import RailCompetitionKind


# Compatibility aliases для internal tests/старых scripts. Источник shared
# contracts больше не legacy decal_network.
_NetworkFace = DecalGeometryFace
_lift_position = lift_offset_position
_polygon_area2 = polygon_area2
_segment_point_distance2 = segment_point_distance2


@dataclass(frozen=True)
class _PatchSurfaceNodeView:
    """Read-only algorithm view derived from one PatchSurfaceIR patch."""

    node: object
    mesh_verts: tuple
    mesh_vert_indices: tuple[int, ...]
    mesh_tris: tuple[tuple[int, int, int], ...]
    mesh_tri_face_indices: tuple[int, ...]
    mesh_tri_face_normals: tuple
    mesh_tri_edge_indices: tuple[tuple[int, int, int], ...]
    source_face_normals: tuple[tuple[int, tuple[float, float, float]], ...]

    def __getattr__(self, name):
        return getattr(self.node, name)


def _patch_surface_node_view(node, patch_surface):
    triangles = patch_surface.patch_triangles(node.patch_id)
    source_vertex_ids = tuple(
        sorted(
            {
                vertex_id
                for triangle in triangles
                for vertex_id in triangle.vertex_ids
            }
        )
    )
    local_by_source = {
        vertex_id: local_index
        for local_index, vertex_id in enumerate(source_vertex_ids)
    }
    vertex_by_id = patch_surface.vertex_by_id
    face_by_id = patch_surface.face_by_id
    return _PatchSurfaceNodeView(
        node=node,
        mesh_verts=tuple(
            Vector(vertex_by_id[vertex_id].position)
            for vertex_id in source_vertex_ids
        ),
        mesh_vert_indices=source_vertex_ids,
        mesh_tris=tuple(
            tuple(local_by_source[vertex_id] for vertex_id in triangle.vertex_ids)
            for triangle in triangles
        ),
        mesh_tri_face_indices=tuple(
            int(triangle.source_face_id) for triangle in triangles
        ),
        mesh_tri_face_normals=tuple(
            Vector(triangle.triangle_normal) for triangle in triangles
        ),
        mesh_tri_edge_indices=tuple(
            tuple(-1 if edge_id is None else int(edge_id) for edge_id in triangle.physical_edge_ids)
            for triangle in triangles
        ),
        source_face_normals=tuple(
            (face.face_id, face.polygon_normal)
            for face in patch_surface.patch_faces(node.patch_id)
        ),
    )

try:
    import pyvoronoi
except ImportError:  # Blender может открыть старый файл без установленного wheel.
    pyvoronoi = None


_GEOMETRY_EPS = 1e-9
_FRAGMENT_TOPOLOGY_TOLERANCE = max(1e-8, DECAL_WELD_DISTANCE * 0.01)
_ANGLE_CLASSIFICATION_EPS = 1e-5
_MITER_ANGLE = 2.0 * pi / 3.0
_KITE_ANGLE = pi / 2.0
_SPLIT_ANGLE = pi / 3.0
_HAIRPIN_ANGLE = pi / 6.0
_SMOOTH_TURN_ANGLE = pi / 18.0
_MITER_LIMIT = 8.0

DECAL_CORNER_JOIN_RECOMPILE_REQUIRED = (
    "DECAL_CORNER_JOIN_RECOMPILE_REQUIRED"
)


def require_decal_corner_join_available(settings):
    """S-CM.b compatibility gate: валидирует join без коэрции."""

    join_mode_value = getattr(
        settings,
        "corner_join_mode",
        getattr(settings, "join_mode", "MITER"),
    )
    join_mode = str(getattr(join_mode_value, "value", join_mode_value)).upper()
    return CornerJoinMode(join_mode)


class _CornerPolicy(str, Enum):
    """Intrinsic corner policy; не зависит от способа lift на owner mesh."""

    CAP = "CAP"
    SMOOTH = "SMOOTH"
    MITER = "MITER"
    KITE = "KITE"
    FAN = "FAN"
    ACUTE_SPLIT = "ACUTE_SPLIT"
    HAIRPIN = "HAIRPIN"
    JUNCTION = "JUNCTION"


@dataclass(frozen=True)
class _PatchVoronoiSite:
    patch_id: int
    edge_index: int
    vert_a: int
    vert_b: int
    source_a: Vector
    source_b: Vector
    point_a: tuple[float, float]
    point_b: tuple[float, float]
    arc_start: float
    segment_length: float
    uv_sign: float
    inward_normal: tuple[float, float]
    arc_sign: float = 1.0
    two_sided: bool = False
    uv_length: float = 0.0
    owner_face_index: int = -1


@dataclass(frozen=True)
class _DiagramVertex:
    X: float
    Y: float


@dataclass(frozen=True)
class CornerSpec:
    """Статические геометрические факты endpoint одного planar patch.

    Corner policy намеренно не хранится в compiled plan: она зависит от
    runtime-настроек и переоценивается без повторного PyVoronoi solve.
    """

    vert_index: int
    point: tuple[float, float]
    incident_sites: tuple[int, ...]
    ordered_sites: tuple[int, ...]
    turn_sign: float
    interior_angle: float
    extrusion_angle: float
    is_convex: bool
    miter_ratio: float
    split_chord: tuple[tuple[float, float], tuple[float, float]] = ()
    static_wedge: tuple[tuple[float, float], ...] = ()
    site_u_offsets: tuple[tuple[int, float], ...] = ()


@dataclass(frozen=True, init=False)
class CornerRuntimeSettings:
    """Дешёвые corner-настройки, допустимые к изменению во время drag."""

    miter_angle: float
    kite_angle: float
    split_angle: float
    hairpin_angle: float
    apex_limit: float
    dynamic_corner_bands: bool
    join_mode: str

    def __init__(
        self,
        acute_split_angle=None,
        apex_limit=_MITER_LIMIT,
        *,
        miter_limit=None,
        miter_angle=_MITER_ANGLE,
        kite_angle=_KITE_ANGLE,
        split_angle=None,
        hairpin_angle=_HAIRPIN_ANGLE,
        dynamic_corner_bands=False,
        join_mode="MITER",
    ):
        # ``miter_limit`` остаётся constructor adapter для старых scripts.
        if miter_limit is not None:
            apex_limit = miter_limit
        if split_angle is None:
            split_angle = (
                _SPLIT_ANGLE
                if acute_split_angle is None
                else acute_split_angle
            )
        object.__setattr__(self, "miter_angle", float(miter_angle))
        object.__setattr__(self, "kite_angle", float(kite_angle))
        object.__setattr__(self, "split_angle", float(split_angle))
        object.__setattr__(self, "hairpin_angle", float(hairpin_angle))
        object.__setattr__(self, "apex_limit", float(apex_limit))
        object.__setattr__(
            self, "dynamic_corner_bands", bool(dynamic_corner_bands)
        )
        join_mode = str(getattr(join_mode, "value", join_mode)).upper()
        if join_mode not in {"MITER", "BEVEL"}:
            raise ValueError(
                f"DECAL_CORNER_JOIN_MODE_UNSUPPORTED:{join_mode}"
            )
        object.__setattr__(self, "join_mode", join_mode)

    @property
    def acute_split_angle(self):
        """Compatibility alias для A10 и старых scripts."""

        return self.split_angle

    @property
    def miter_limit(self):
        """Compatibility alias; runtime geometry использует apex_limit."""

        return self.apex_limit


def _normalized_corner_runtime_settings(settings):
    settings = settings or CornerRuntimeSettings()
    apex_limit = getattr(
        settings,
        "apex_limit",
        getattr(settings, "miter_limit", _MITER_LIMIT),
    )
    miter_angle = max(
        0.0,
        min(pi, float(getattr(settings, "miter_angle", _MITER_ANGLE))),
    )
    kite_angle = max(
        0.0,
        min(
            miter_angle,
            float(getattr(settings, "kite_angle", _KITE_ANGLE)),
        ),
    )
    split_angle = max(
        0.0,
        min(
            kite_angle,
            float(
                getattr(
                    settings,
                    "split_angle",
                    getattr(settings, "acute_split_angle", _SPLIT_ANGLE),
                )
            ),
        ),
    )
    hairpin_angle = max(
        0.0,
        min(
            split_angle,
            float(getattr(settings, "hairpin_angle", _HAIRPIN_ANGLE)),
        ),
    )
    return CornerRuntimeSettings(
        miter_angle=miter_angle,
        kite_angle=kite_angle,
        split_angle=split_angle,
        hairpin_angle=hairpin_angle,
        apex_limit=max(1.0, float(apex_limit)),
        dynamic_corner_bands=bool(
            getattr(settings, "dynamic_corner_bands", False)
        ),
        join_mode=getattr(settings, "join_mode", "MITER"),
    )


def corner_runtime_settings_from_decal_settings(settings):
    """Единственный adapter immutable DecalSettings → runtime policy."""

    return CornerRuntimeSettings(
        miter_angle=float(
            getattr(settings, "corner_miter_angle", _MITER_ANGLE)
        ),
        kite_angle=float(
            getattr(settings, "corner_kite_angle", _KITE_ANGLE)
        ),
        split_angle=float(settings.corner_acute_split_angle),
        hairpin_angle=float(
            getattr(settings, "corner_hairpin_angle", _HAIRPIN_ANGLE)
        ),
        apex_limit=float(
            getattr(
                settings,
                "corner_apex_limit",
                getattr(settings, "corner_miter_limit", _MITER_LIMIT),
            )
        ),
        dynamic_corner_bands=bool(
            getattr(settings, "dynamic_corner_bands", False)
        ),
        join_mode=getattr(settings, "corner_join_mode", "MITER"),
    )


@dataclass(frozen=True)
class _PatchVoronoiAtom:
    site_index: int
    fragments: tuple[tuple[tuple[float, float], ...], ...]
    cell_kind: str
    fragment_triangle_ids: tuple[int, ...] = ()
    corner_index: int = -1
    source_category: int = 0
    periodic_shift: int = 0

    def __post_init__(self):
        if self.fragment_triangle_ids and (
            len(self.fragment_triangle_ids) != len(self.fragments)
        ):
            raise ValueError(
                "Patch Voronoi atom fragment provenance must be aligned"
            )


@dataclass(frozen=True)
class _CornerReleaseAtom:
    """Второй owner point-cell после удаления материи угла RC5a.

    Primary point-cell остаётся единственным источником формы угла. Эта
    запись хранит только width-independent обычную конкуренцию в той части
    cell, которую выбранный join может освободить.
    """

    corner_index: int
    point_site_index: int
    point_periodic_shift: int
    owner_site_index: int
    owner_periodic_shift: int
    fragments: tuple[tuple[tuple[float, float], ...], ...]
    fragment_triangle_ids: tuple[int, ...] = ()

    def __post_init__(self):
        if self.fragment_triangle_ids and (
            len(self.fragment_triangle_ids) != len(self.fragments)
        ):
            raise ValueError(
                "Corner release fragment provenance must be aligned"
            )


@dataclass(frozen=True)
class _IntrinsicDomainTriangle:
    """Один triangle intrinsic chart и его lift-данные на owner mesh."""

    chart_points: tuple[tuple[float, float], ...]
    positions: tuple[Vector, ...]
    normals: tuple[Vector, ...]
    face_normal: Vector | None = None
    source_triangle_id: object | None = None
    source_face_id: object | None = None
    # Edge id имеет индекс противоположной локальной вершины triangle.
    source_edge_ids: tuple[object | None, ...] = (None, None, None)
    source_vertex_ids: tuple[object | None, ...] = (None, None, None)
    edge_transition_keys: tuple[object | None, ...] = (None, None, None)
    vertex_transition_keys: tuple[object | None, ...] = (None, None, None)

    def __post_init__(self):
        fields = (
            self.chart_points,
            self.positions,
            self.normals,
            self.source_edge_ids,
            self.source_vertex_ids,
            self.edge_transition_keys,
            self.vertex_transition_keys,
        )
        if any(len(values) != 3 for values in fields):
            raise ValueError("Intrinsic domain triangle fields must have 3 items")


@dataclass(frozen=True)
class _TriangleAabbGrid:
    """Неизменяемый AABB-index chart triangles.

    ``cell_keys`` отсортированы и ищутся binary search. В отличие от dict в
    frozen dataclass индекс нельзя случайно изменить между preview frames.
    Triangle ids внутри каждой cell сохраняют исходный порядок full scan.
    """

    cell_size: float
    cell_keys: tuple[tuple[int, int], ...]
    triangle_ids: tuple[tuple[int, ...], ...]
    all_triangle_ids: tuple[int, ...]

    def _cell_triangle_ids(self, key):
        index = bisect_left(self.cell_keys, key)
        if index >= len(self.cell_keys) or self.cell_keys[index] != key:
            return ()
        return self.triangle_ids[index]

    def query_aabb(self, min_x, min_y, max_x, max_y, tolerance=1e-10):
        if self.cell_size <= 0.0:
            return self.all_triangle_ids
        min_grid_x = int((min_x - tolerance) // self.cell_size)
        max_grid_x = int((max_x + tolerance) // self.cell_size)
        min_grid_y = int((min_y - tolerance) // self.cell_size)
        max_grid_y = int((max_y + tolerance) // self.cell_size)
        candidates = set()
        for grid_x in range(min_grid_x, max_grid_x + 1):
            for grid_y in range(min_grid_y, max_grid_y + 1):
                candidates.update(
                    self._cell_triangle_ids((grid_x, grid_y))
                )
        return tuple(sorted(candidates))

    def query_point(self, point, tolerance=1e-10):
        return self.query_aabb(
            point[0], point[1], point[0], point[1], tolerance
        )


def _build_triangle_aabb_grid(triangles):
    """Компилирует детерминированный immutable grid из chart triangles."""

    triangles = tuple(tuple(triangle) for triangle in triangles)
    triangle_ids = tuple(range(len(triangles)))
    if not triangles:
        return _TriangleAabbGrid(0.0, (), (), ())
    all_points = tuple(point for triangle in triangles for point in triangle)
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    diagonal = sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)
    cell_size = max(
        diagonal / max(2.0, sqrt(len(triangles)) * 2.0),
        DECAL_WELD_DISTANCE * 4.0,
    )
    cells = {}
    for triangle_id, triangle in enumerate(triangles):
        triangle_min_x = min(point[0] for point in triangle)
        triangle_max_x = max(point[0] for point in triangle)
        triangle_min_y = min(point[1] for point in triangle)
        triangle_max_y = max(point[1] for point in triangle)
        min_grid_x = int(triangle_min_x // cell_size)
        max_grid_x = int(triangle_max_x // cell_size)
        min_grid_y = int(triangle_min_y // cell_size)
        max_grid_y = int(triangle_max_y // cell_size)
        for grid_x in range(min_grid_x, max_grid_x + 1):
            for grid_y in range(min_grid_y, max_grid_y + 1):
                cells.setdefault((grid_x, grid_y), []).append(triangle_id)
    ordered = tuple(
        (key, tuple(cells[key])) for key in sorted(cells)
    )
    return _TriangleAabbGrid(
        cell_size=cell_size,
        cell_keys=tuple(key for key, _ids in ordered),
        triangle_ids=tuple(ids for _key, ids in ordered),
        all_triangle_ids=triangle_ids,
    )


@dataclass(frozen=True)
class DecalSurfaceDomain:
    """Surface adapter между intrinsic 2D solver и исходным mesh.

    PLANAR использует одну ортонормальную basis. INTRINSIC хранит atlas
    triangles; Voronoi/corner/crop ничего не знают о способе обратного lift.
    """

    patch_id: int
    kind: str
    origin: Vector
    reference_normal: Vector
    basis_u: Vector
    basis_v: Vector
    boundary_triangles: tuple[tuple[tuple[float, float], ...], ...]
    boundary_triangle_source_face_ids: tuple[object, ...] = ()
    intrinsic_triangles: tuple[_IntrinsicDomainTriangle, ...] = ()
    periodic_axis: str = ""
    period: float = 0.0
    period_quantum: float = 0.0
    wrap_origin: float = 0.0
    periodic_cut: ChartCut | None = None
    transition_equivalences: tuple[tuple[object, tuple[object, ...]], ...] = ()
    chart_id: int = 0
    alpha_budget: float = float("inf")
    budget_source: str = "FULL_CONNECTED_COMPONENT"
    normal_mode: str = "PIECEWISE_PLANAR_HARD"
    triangle_grid: _TriangleAabbGrid | None = None
    source_edge_features: tuple[tuple[object, tuple[int, ...]], ...] = ()
    source_vertex_features: tuple[tuple[object, tuple[int, ...]], ...] = ()
    planar_source_edges: tuple[
        tuple[
            int,
            int,
            int,
            tuple[float, float],
            tuple[float, float],
        ],
        ...,
    ] = ()
    planar_source_vertices: tuple[
        tuple[int, tuple[float, float]], ...
    ] = ()
    planar_source_edge_positions: tuple[
        tuple[int, tuple[float, float, float], tuple[float, float, float]],
        ...,
    ] = ()
    triangle_merge_groups: tuple[int, ...] = ()
    transition_metadata: tuple[tuple[object, str, object], ...] = ()
    admission_tier: str = "EXACT"
    normalize_fragment_t_junctions: bool = False
    location_tolerance: float = 1e-7
    reference_full_scan: bool = False

    def __post_init__(self):
        if self.kind not in {"PLANAR", "INTRINSIC"}:
            raise ValueError(f"Unsupported decal domain kind: {self.kind}")
        if self.admission_tier not in {"EXACT", "APPROXIMATE"}:
            raise ValueError("Decal domain admission tier is invalid")
        if self.location_tolerance <= 0.0:
            raise ValueError("Decal domain location tolerance must be positive")
        if self.normal_mode not in {
            "PIECEWISE_PLANAR_HARD",
            "SMOOTH_INTERPOLATED",
        }:
            raise ValueError(
                f"Unsupported intrinsic normal mode: {self.normal_mode}"
            )
        if not self.alpha_budget > 0.0:
            raise ValueError("Decal domain alpha budget must be positive")
        if not self.budget_source:
            raise ValueError("Decal domain budget source must be explicit")
        triangle_count = (
            len(self.intrinsic_triangles)
            if self.intrinsic_triangles
            else len(self.boundary_triangles)
        )
        if self.boundary_triangle_source_face_ids and (
            len(self.boundary_triangle_source_face_ids)
            != len(self.boundary_triangles)
        ):
            raise ValueError(
                "Decal planar source faces must be triangle-aligned"
            )
        if (
            self.kind == "PLANAR"
            and self.boundary_triangles
            and not self.boundary_triangle_source_face_ids
        ):
            object.__setattr__(
                self,
                "boundary_triangle_source_face_ids",
                (self.patch_id,) * len(self.boundary_triangles),
            )
        if self.triangle_merge_groups and (
            len(self.triangle_merge_groups) != triangle_count
        ):
            raise ValueError(
                "Decal domain triangle merge groups must be aligned"
            )
        validate_periodic_chart_fields(
            self.periodic_axis,
            self.period,
            self.period_quantum,
            self.wrap_origin,
            self.periodic_cut,
            self.transition_equivalences,
        )
        if self.periodic_axis and self.kind != "INTRINSIC":
            raise ValueError("Periodic domain must be intrinsic")
        if self.kind == "INTRINSIC" and self.intrinsic_triangles:
            edge_features = {}
            vertex_features = {}
            transitions = set()
            for triangle_id, triangle in enumerate(self.intrinsic_triangles):
                for source_id, transition_key in zip(
                    triangle.source_edge_ids,
                    triangle.edge_transition_keys,
                ):
                    edge_features.setdefault(source_id, set()).add(triangle_id)
                    if transition_key is not None:
                        transitions.add((transition_key, "EDGE", source_id))
                for source_id, transition_key in zip(
                    triangle.source_vertex_ids,
                    triangle.vertex_transition_keys,
                ):
                    vertex_features.setdefault(source_id, set()).add(triangle_id)
                    if transition_key is not None:
                        transitions.add((transition_key, "VERTEX", source_id))
            if not self.source_edge_features:
                object.__setattr__(
                    self,
                    "source_edge_features",
                    tuple(
                        (key, tuple(sorted(values)))
                        for key, values in sorted(
                            edge_features.items(), key=lambda item: repr(item[0])
                        )
                    ),
                )
            if not self.source_vertex_features:
                object.__setattr__(
                    self,
                    "source_vertex_features",
                    tuple(
                        (key, tuple(sorted(values)))
                        for key, values in sorted(
                            vertex_features.items(), key=lambda item: repr(item[0])
                        )
                    ),
                )
            if not self.transition_metadata:
                object.__setattr__(
                    self,
                    "transition_metadata",
                    tuple(sorted(transitions, key=repr)),
                )
        if self.triangle_grid is not None:
            return
        chart_triangles = (
            tuple(triangle.chart_points for triangle in self.intrinsic_triangles)
            if self.intrinsic_triangles
            else self.boundary_triangles
        )
        object.__setattr__(
            self, "triangle_grid", _build_triangle_aabb_grid(chart_triangles)
        )

    def project(self, position):
        if self.kind != "PLANAR":
            raise ValueError(
                "Intrinsic domain projection requires triangle provenance"
            )
        delta = position - self.origin
        return (delta.dot(self.basis_u), delta.dot(self.basis_v))

    def canonical_transition_key(self, transition_key):
        """Сворачивает только явно объявленные DP3-equivalence keys."""

        if transition_key is None:
            return None
        for canonical_key, image_keys in self.transition_equivalences:
            if transition_key == canonical_key or any(
                transition_key == image_key for image_key in image_keys
            ):
                return canonical_key
        return transition_key

    def locate(self, point):
        """Разрешает chart point в source feature только у lift boundary."""

        uv = (float(point[0]), float(point[1]))
        if self.kind == "PLANAR":
            tolerance = max(
                self.location_tolerance,
                DECAL_WELD_DISTANCE * 0.25,
            )
            triangle_id = -1
            triangle_weights = (1.0, 0.0, 0.0)
            best_margin = -float("inf")
            triangle_ids = self.triangle_grid.query_point(uv, tolerance)
            for candidate_id in triangle_ids:
                weights = _triangle_weights2(
                    uv, self.boundary_triangles[candidate_id]
                )
                if weights is None:
                    continue
                margin = min(weights)
                if margin >= -tolerance and margin > best_margin:
                    triangle_id = int(candidate_id)
                    triangle_weights = tuple(
                        float(weight) for weight in weights
                    )
                    best_margin = margin
            vertex_candidates = []
            for vertex_id, vertex_uv in self.planar_source_vertices:
                distance = _dist2(uv, vertex_uv)
                if distance <= tolerance:
                    vertex_candidates.append(
                        (distance, int(vertex_id), vertex_uv)
                    )
            if vertex_candidates:
                _distance, vertex_id, vertex_uv = min(vertex_candidates)
                return DomainLocation(
                    chart_id=self.chart_id,
                    triangle_id=triangle_id,
                    uv=vertex_uv,
                    barycentric=triangle_weights,
                    source_feature="VERTEX",
                    source_feature_id=vertex_id,
                )
            edge_candidates = []
            for edge_record in self.planar_source_edges:
                edge_id, _vert_a, _vert_b, point_a, point_b = edge_record
                distance, parameter = _segment_point_distance2(
                    point_a, point_b, uv
                )
                if distance > tolerance:
                    continue
                projection = (
                    point_a[0] + (point_b[0] - point_a[0]) * parameter,
                    point_a[1] + (point_b[1] - point_a[1]) * parameter,
                )
                edge_candidates.append(
                    (distance, int(edge_id), parameter, projection)
                )
            if edge_candidates:
                _distance, edge_id, _parameter, projection = min(
                    edge_candidates
                )
                return DomainLocation(
                    chart_id=self.chart_id,
                    triangle_id=triangle_id,
                    uv=projection,
                    barycentric=triangle_weights,
                    source_feature="EDGE",
                    source_feature_id=edge_id,
                )
            return DomainLocation(
                chart_id=self.chart_id,
                triangle_id=triangle_id,
                uv=uv,
                barycentric=triangle_weights,
                source_feature="TRIANGLE",
                source_feature_id=("PLANAR", self.patch_id),
            )
        best_triangle_id = -1
        best_weights = None
        best_margin = -float("inf")
        if self.reference_full_scan:
            triangle_ids = range(len(self.intrinsic_triangles))
        else:
            triangle_ids = self.triangle_grid.query_point(
                point, self.location_tolerance
            )
        candidate_triangle_ids = tuple(triangle_ids)
        for triangle_id in candidate_triangle_ids:
            triangle = self.intrinsic_triangles[triangle_id]
            weights = _triangle_weights2(point, triangle.chart_points)
            if weights is None:
                continue
            margin = min(weights)
            if margin >= -self.location_tolerance and margin > best_margin:
                best_triangle_id = triangle_id
                best_weights = tuple(float(weight) for weight in weights)
                best_margin = margin
        if best_weights is None and self.admission_tier == "APPROXIMATE":
            best_projection = None
            for triangle_id in candidate_triangle_ids:
                triangle = self.intrinsic_triangles[triangle_id]
                for edge_index, first in enumerate(triangle.chart_points):
                    second = triangle.chart_points[(edge_index + 1) % 3]
                    distance, factor = _segment_point_distance2(
                        first, second, point
                    )
                    if distance > self.location_tolerance * 2.0:
                        continue
                    projection = (
                        first[0] + (second[0] - first[0]) * factor,
                        first[1] + (second[1] - first[1]) * factor,
                    )
                    rank = (distance, triangle_id, edge_index)
                    if best_projection is None or rank < best_projection[0]:
                        best_projection = (rank, projection, triangle_id)
            if best_projection is not None:
                _rank, uv, best_triangle_id = best_projection
                weights = _triangle_weights2(
                    uv,
                    self.intrinsic_triangles[best_triangle_id].chart_points,
                )
                if weights is not None:
                    best_weights = tuple(float(value) for value in weights)
        if best_weights is None:
            return None

        triangle = self.intrinsic_triangles[best_triangle_id]
        feature_tolerance = self.location_tolerance
        boundary_weights = tuple(
            index
            for index, weight in enumerate(best_weights)
            if abs(weight) <= feature_tolerance
        )
        transition_key = None
        if len(boundary_weights) >= 2:
            feature_index = max(
                range(3), key=lambda index: best_weights[index]
            )
            source_feature = "VERTEX"
            source_feature_id = triangle.source_vertex_ids[feature_index]
            transition_key = triangle.vertex_transition_keys[feature_index]
        elif len(boundary_weights) == 1:
            feature_index = boundary_weights[0]
            source_feature = "EDGE"
            source_feature_id = triangle.source_edge_ids[feature_index]
            transition_key = triangle.edge_transition_keys[feature_index]
        else:
            feature_index = -1
            source_feature = "TRIANGLE"
            source_feature_id = triangle.source_triangle_id
        if source_feature_id is None:
            source_feature_id = (
                self.chart_id,
                best_triangle_id,
                source_feature,
                feature_index,
            )
        return DomainLocation(
            chart_id=self.chart_id,
            triangle_id=best_triangle_id,
            uv=uv,
            barycentric=best_weights,
            source_feature=source_feature,
            source_feature_id=source_feature_id,
            transition_key=self.canonical_transition_key(transition_key),
        )

    def planar_edge_parameter(self, edge_id, point):
        """Канонический параметр физического PLANAR source edge."""

        source_point = (
            self.origin
            + self.basis_u * float(point[0])
            + self.basis_v * float(point[1])
        )
        for candidate_id, source_a, source_b in (
            self.planar_source_edge_positions
        ):
            if int(candidate_id) != int(edge_id):
                continue
            point_a = Vector(source_a)
            delta = Vector(source_b) - point_a
            denominator = delta.length_squared
            if denominator <= _GEOMETRY_EPS:
                return 0.0
            return max(
                0.0,
                min(1.0, (source_point - point_a).dot(delta) / denominator),
            )
        for record in self.planar_source_edges:
            candidate_id, _vert_a, _vert_b, point_a, point_b = record
            if int(candidate_id) != int(edge_id):
                continue
            _distance, parameter = _segment_point_distance2(
                point_a, point_b, point
            )
            return float(parameter)
        return None

    def _intrinsic_location(self, point):
        """Compatibility view старого private adapter."""

        location = self.locate(point)
        if location is None:
            return None
        return (
            self.intrinsic_triangles[location.triangle_id],
            location.barycentric,
        )

    def _triangle_face_normal(self, triangle):
        normal = triangle.face_normal
        if normal is not None and normal.length_squared > 1e-12:
            return normal.normalized()
        blended = sum(
            triangle.normals, Vector((0.0, 0.0, 0.0))
        )
        if blended.length_squared <= 1e-12:
            return self.reference_normal.normalized()
        return blended.normalized()

    def _location_normals(self, location):
        triangle = self.intrinsic_triangles[location.triangle_id]
        if location.source_feature == "TRIANGLE":
            return (self._triangle_face_normal(triangle),)
        normals = []
        for candidate in self.intrinsic_triangles:
            feature_ids = (
                candidate.source_edge_ids
                if location.source_feature == "EDGE"
                else candidate.source_vertex_ids
            )
            if location.source_feature_id not in feature_ids:
                continue
            normals.append(self._triangle_face_normal(candidate))
        if not normals:
            normals.append(self._triangle_face_normal(triangle))
        return tuple(normals)

    def _normal_at_location(self, location):
        triangle = self.intrinsic_triangles[location.triangle_id]
        if self.normal_mode == "SMOOTH_INTERPOLATED":
            normal = sum(
                (
                    triangle.normals[index]
                    * location.barycentric[index]
                    for index in range(3)
                ),
                Vector((0.0, 0.0, 0.0)),
            )
        else:
            normal = sum(
                self._location_normals(location),
                Vector((0.0, 0.0, 0.0)),
            )
        if normal.length_squared <= 1e-12:
            return self.reference_normal.copy()
        return normal.normalized()

    def source_position(self, location):
        """Barycentric source position без decal offset."""

        if self.kind == "PLANAR":
            return (
                self.origin
                + self.basis_u * location.uv[0]
                + self.basis_v * location.uv[1]
            )
        triangle = self.intrinsic_triangles[location.triangle_id]
        return sum(
            (
                triangle.positions[index] * location.barycentric[index]
                for index in range(3)
            ),
            Vector((0.0, 0.0, 0.0)),
        )

    def normal_at(self, point):
        if self.kind == "PLANAR":
            return self.reference_normal.copy()
        location = self.locate(point)
        if location is None:
            raise ValueError("Point lies outside intrinsic decal domain")
        return self._normal_at_location(location)

    def lift(self, point, offset, location=None):
        if self.kind == "PLANAR":
            return (
                self.origin
                + self.basis_u * point[0]
                + self.basis_v * point[1]
                + self.reference_normal * offset
            )
        location = location or self.locate(point)
        if location is None:
            raise ValueError("Point lies outside intrinsic decal domain")
        position = self.source_position(location)
        if self.normal_mode == "SMOOTH_INTERPOLATED":
            return position + self._normal_at_location(location) * offset
        return _lift_position(
            position, self._location_normals(location), offset
        )


@dataclass(frozen=True)
class _CompiledSitePort:
    """Width-independent endpoint relation для junction port matching."""

    site_index: int
    edge_index: int
    vert_index: int
    point: tuple[float, float]
    source_category: int


@dataclass(frozen=True)
class _PatchVoronoiSurface:
    patch_id: int
    domain: DecalSurfaceDomain
    sites: tuple[_PatchVoronoiSite, ...]
    corners: tuple[CornerSpec, ...]
    atoms: tuple[_PatchVoronoiAtom, ...]
    diagram_transform: DiagramTransform
    site_grid_size: float
    site_grid: dict[tuple[int, int], tuple[int, ...]]
    atoms_by_site: dict[int, tuple[int, ...]]
    owner_atoms_by_corner: dict[int, tuple[int, ...]]
    corners_by_site: dict[int, tuple[int, ...]]
    sites_by_vertex: dict[int, tuple[int, ...]]
    ports_by_vertex: dict[int, tuple["_CompiledSitePort", ...]]
    ports_by_site: dict[int, tuple["_CompiledSitePort", ...]]
    corner_release_atoms: dict[int, tuple[_CornerReleaseAtom, ...]]
    semantic_owner_chart_by_vertex: tuple[tuple[int, int], ...] = ()
    native_site_edge_indices: tuple[int, ...] = ()
    corner_seeds: tuple[CornerSeed, ...] = ()

    @property
    def origin(self):
        return self.domain.origin

    @property
    def normal(self):
        return self.domain.reference_normal

    @property
    def basis_u(self):
        return self.domain.basis_u

    @property
    def basis_v(self):
        return self.domain.basis_v


@dataclass(frozen=True)
class _PlanarOwnerSurface:
    """Локальная coplanar часть patch для decal backend.

    Один topology patch может огибать фаску или extrusion. Для Voronoi это
    не причина отклонять весь selection: его owner faces делятся на
    независимые planar surfaces, а junction layer соединяет их rails.
    """

    patch_id: int
    centroid: Vector
    normal: Vector
    basis_u: Vector
    basis_v: Vector
    boundary_loops: tuple
    mesh_verts: tuple
    mesh_tris: tuple
    mesh_tri_face_indices: tuple[int, ...]


@dataclass(frozen=True)
class PatchVoronoiPlan:
    """Width-independent segment-Voronoi diagrams одного modal invoke."""

    offset: float
    surfaces: tuple[_PatchVoronoiSurface, ...]
    lifted_vertices: dict[int, Vector]
    max_lateral_lift_ratio: float
    alpha_budget: float = float("inf")
    support_triangle_ids: tuple[tuple[int, ...], ...] = ()
    budget_source: str = "FULL_CONNECTED_COMPONENT"
    requested_alpha_budget: float = float("inf")
    approximate_admit_count: int = 0
    capacity_policy: CapacityPolicy = CapacityPolicy.SATURATE_PROVEN
    corner_join_mode: CornerJoinMode = CornerJoinMode.MITER

    def __post_init__(self):
        object.__setattr__(
            self,
            "capacity_policy",
            CapacityPolicy(self.capacity_policy),
        )
        object.__setattr__(
            self,
            "corner_join_mode",
            CornerJoinMode(self.corner_join_mode),
        )
        if not self.alpha_budget > 0.0:
            raise ValueError("Patch Voronoi alpha_budget must be positive")
        if not self.requested_alpha_budget > 0.0:
            raise ValueError(
                "Patch Voronoi requested_alpha_budget must be positive"
            )
        if self.approximate_admit_count < 0:
            raise ValueError("Approximate admit count must be non-negative")
        support_triangle_ids = self.support_triangle_ids
        if not support_triangle_ids:
            support_triangle_ids = tuple(
                tuple(
                    range(
                        len(surface.domain.intrinsic_triangles)
                        if surface.domain.intrinsic_triangles
                        else len(surface.domain.boundary_triangles)
                    )
                )
                for surface in self.surfaces
            )
            object.__setattr__(
                self, "support_triangle_ids", support_triangle_ids
            )
        if len(support_triangle_ids) != len(self.surfaces):
            raise ValueError(
                "Patch Voronoi support_triangle_ids must match surfaces"
            )
        for surface, triangle_ids in zip(
            self.surfaces, support_triangle_ids
        ):
            triangle_count = (
                len(surface.domain.intrinsic_triangles)
                if surface.domain.intrinsic_triangles
                else len(surface.domain.boundary_triangles)
            )
            if tuple(sorted(set(triangle_ids))) != tuple(triangle_ids):
                raise ValueError(
                    "Patch Voronoi support triangle ids must be unique/sorted"
                )
            if any(
                triangle_id < 0 or triangle_id >= triangle_count
                for triangle_id in triangle_ids
            ):
                raise ValueError(
                    "Patch Voronoi support triangle id lies outside domain"
                )
        if not self.budget_source:
            raise ValueError("Patch Voronoi budget_source must be explicit")

    def active_triangle_ids(self, alpha):
        """Возвращает compiled support либо явно отклоняет excess width."""

        alpha = float(alpha)
        if not isfinite(alpha) or alpha < 0.0:
            raise ValueError("Runtime alpha must be finite and non-negative")
        if (
            alpha > self.alpha_budget + _GEOMETRY_EPS
            and self.capacity_policy is CapacityPolicy.SATURATE_PROVEN
        ):
            return self.support_triangle_ids
        if (
            alpha > self.alpha_budget + _GEOMETRY_EPS
            and self.capacity_policy is CapacityPolicy.REJECT_UNPROVEN
        ):
            raise DomainCapacityUnproven(
                alpha,
                self.alpha_budget,
                self.budget_source,
            )
        if alpha > self.alpha_budget + _GEOMETRY_EPS:
            raise DomainBudgetExceeded(
                alpha,
                self.alpha_budget,
                self.budget_source,
                self.capacity_policy,
            )
        return self.support_triangle_ids

    @property
    def backend_kind(self):
        kinds = {surface.domain.kind for surface in self.surfaces}
        if kinds == {"PLANAR"}:
            return "PLANAR"
        if kinds == {"INTRINSIC"}:
            return "INTRINSIC_DEVELOPABLE"
        return "PLANAR+INTRINSIC_DEVELOPABLE"


class DomainBudgetExceeded(ValueError):
    """Runtime width вышла за доказанную область compiled strip chart."""

    code = "DOMAIN_BUDGET_EXCEEDED"

    def __init__(
        self,
        requested_alpha,
        alpha_budget,
        budget_source,
        capacity_policy=CapacityPolicy.CONTROLLED_RECOMPILE,
    ):
        self.requested_alpha = float(requested_alpha)
        self.alpha_budget = float(alpha_budget)
        self.budget_source = str(budget_source)
        self.capacity_policy = CapacityPolicy(capacity_policy)
        super().__init__(
            f"{self.code}: alpha={self.requested_alpha:.6g} exceeds "
            f"budget={self.alpha_budget:.6g} "
            f"(source={self.budget_source}, "
            f"policy={self.capacity_policy.value})"
        )


class DomainCapacityUnproven(DomainBudgetExceeded):
    """I8: максимальное покрытие не доказано и не может быть клампнуто."""

    code = "DOMAIN_CAPACITY_UNPROVEN"

    def __init__(self, requested_alpha, alpha_budget, budget_source):
        super().__init__(
            requested_alpha,
            alpha_budget,
            budget_source,
            CapacityPolicy.REJECT_UNPROVEN,
        )


@dataclass(frozen=True)
class PatchVoronoiCompileFailure:
    """Локальный отказ compile, не обязанный отменять весь selection."""

    patch_id: int
    reason: str
    edge_indices: tuple[int, ...]
    details: str = ""


@dataclass(frozen=True)
class PatchVoronoiCompileAttempt:
    """Результат диагностического compile с rejected physical edges."""

    plan: PatchVoronoiPlan | None
    rejected_edge_indices: tuple[int, ...] = ()
    failures: tuple[PatchVoronoiCompileFailure, ...] = ()


@dataclass
class PatchVoronoiDiagnostics:
    """Явные compile/runtime counters без module-global состояния."""

    construct_calls: int = 0
    cell_disorder_fallbacks: int = 0
    tessellation_mesh_tri_fallbacks: int = 0
    convex_fragment_decomposition_fallbacks: int = 0
    clamped_miter_count: int = 0
    clamped_kite_count: int = 0
    clamped_acute_count: int = 0
    apex_limit_saturated_count: int = 0
    periodic_copy_count: int = 0
    periodic_weld_count: int = 0
    atlas_chart_count: int = 0
    atlas_site_image_count: int = 0
    interior_transition_count: int = 0
    interior_weld_count: int = 0
    atlas_unresolved_overlap_count: int = 0
    approximate_admit_count: int = 0
    margin_relief_cut_count: int = 0
    max_width_error_sampled: float = 0.0
    max_station_normal_variation: float = 0.0
    foldover_count: int = 0
    atlas_sliver_owner_count: int = 0
    atlas_max_sliver_owner_distance: float = 0.0
    atlas_no_owner_drop_count: int = 0
    atlas_declared_owner_count: int = 0
    atlas_declared_owner_missing_count: int = 0
    atlas_degenerate_interval_merge_count: int = 0
    atlas_undeclared_materialized_interval_count: int = 0
    atlas_clamped_segment_face_count: int = 0
    atlas_v_continuation_count: int = 0
    atlas_touch_no_token_drop_count: int = 0
    atlas_single_side_drop_count: int = 0
    atlas_semantic_import_count: int = 0
    atlas_semantic_transition_count: int = 0
    atlas_arrangement_integrity_failure_count: int = 0
    terminal_route_saturation_count: int = 0
    terminal_route_station_clamp_count: int = 0
    terminal_route_revisit_guard_count: int = 0
    terminal_contact_meeting_count: int = 0
    resolved_corner_view_count: int = 0
    resolved_corner_boundary_vertex_count: int = 0
    resolved_corner_competition_vertex_count: int = 0
    # Тестовое отключение транспорта для обязательного отрицательного T8.
    semantic_transport_disabled_owner_ids: frozenset = field(
        default_factory=frozenset,
        repr=False,
    )
    runtime_policy_counts: dict[str, int] = field(default_factory=dict)
    cap_keep_counts: dict[str, int] = field(default_factory=dict)
    reference_full_scan: bool = False

    def record_runtime_policy(self, policy):
        key = str(getattr(policy, "value", policy))
        self.runtime_policy_counts[key] = (
            self.runtime_policy_counts.get(key, 0) + 1
        )

    def record_cap_keep(self, reason):
        key = str(reason)
        if not key.startswith("CAP_KEEP_"):
            key = f"CAP_KEEP_{key}"
        self.cap_keep_counts[key] = self.cap_keep_counts.get(key, 0) + 1

    def as_dict(self):
        result = {
            "construct_calls": int(self.construct_calls),
            "cell_disorder_fallbacks": int(self.cell_disorder_fallbacks),
            "tessellation_mesh_tri_fallbacks": int(
                self.tessellation_mesh_tri_fallbacks
            ),
            "convex_fragment_decomposition_fallbacks": int(
                self.convex_fragment_decomposition_fallbacks
            ),
            "clamped_miter_count": int(self.clamped_miter_count),
            "clamped_kite_count": int(self.clamped_kite_count),
            "clamped_acute_count": int(self.clamped_acute_count),
            "apex_limit_saturated_count": int(
                self.apex_limit_saturated_count
            ),
            "periodic_copy_count": int(self.periodic_copy_count),
            "periodic_weld_count": int(self.periodic_weld_count),
            "atlas_chart_count": int(self.atlas_chart_count),
            "atlas_site_image_count": int(self.atlas_site_image_count),
            "interior_transition_count": int(
                self.interior_transition_count
            ),
            "interior_weld_count": int(self.interior_weld_count),
            "atlas_unresolved_overlap_count": int(
                self.atlas_unresolved_overlap_count
            ),
            "approximate_admit_count": int(self.approximate_admit_count),
            "margin_relief_cut_count": int(self.margin_relief_cut_count),
            "max_width_error_sampled": float(
                self.max_width_error_sampled
            ),
            "max_station_normal_variation": float(
                self.max_station_normal_variation
            ),
            "foldover_count": int(self.foldover_count),
            "atlas_sliver_owner_count": int(
                self.atlas_sliver_owner_count
            ),
            "atlas_no_owner_drop_count": int(
                self.atlas_no_owner_drop_count
            ),
            "atlas_touch_no_token_drop_count": int(
                self.atlas_touch_no_token_drop_count
            ),
            "atlas_single_side_drop_count": int(
                self.atlas_single_side_drop_count
            ),
            "atlas_semantic_import_count": int(
                self.atlas_semantic_import_count
            ),
            "atlas_semantic_transition_count": int(
                self.atlas_semantic_transition_count
            ),
            "terminal_route_saturation_count": int(
                self.terminal_route_saturation_count
            ),
            "terminal_route_station_clamp_count": int(
                self.terminal_route_station_clamp_count
            ),
            "terminal_route_revisit_guard_count": int(
                self.terminal_route_revisit_guard_count
            ),
            "terminal_contact_meeting_count": int(
                self.terminal_contact_meeting_count
            ),
            "resolved_corner_view_count": int(
                self.resolved_corner_view_count
            ),
            "resolved_corner_boundary_vertex_count": int(
                self.resolved_corner_boundary_vertex_count
            ),
            "resolved_corner_competition_vertex_count": int(
                self.resolved_corner_competition_vertex_count
            ),
            "runtime_policy_counts": dict(
                sorted(self.runtime_policy_counts.items())
            ),
        }
        # Условные E-only поля сохраняют EP1 benchmark JSON exact-пути.
        if self.atlas_declared_owner_count:
            result["atlas_declared_owner_count"] = int(
                self.atlas_declared_owner_count
            )
        if self.atlas_declared_owner_missing_count:
            result["atlas_declared_owner_missing_count"] = int(
                self.atlas_declared_owner_missing_count
            )
        if self.atlas_degenerate_interval_merge_count:
            result["atlas_degenerate_interval_merge_count"] = int(
                self.atlas_degenerate_interval_merge_count
            )
        if self.atlas_undeclared_materialized_interval_count:
            result["atlas_undeclared_materialized_interval_count"] = int(
                self.atlas_undeclared_materialized_interval_count
            )
        if self.atlas_clamped_segment_face_count:
            result["atlas_clamped_segment_face_count"] = int(
                self.atlas_clamped_segment_face_count
            )
        if self.atlas_v_continuation_count:
            result["atlas_v_continuation_count"] = int(
                self.atlas_v_continuation_count
            )
        if self.atlas_arrangement_integrity_failure_count:
            result["atlas_arrangement_integrity_failure_count"] = int(
                self.atlas_arrangement_integrity_failure_count
            )
        if self.cap_keep_counts:
            result["cap_keep_counts"] = dict(
                sorted(self.cap_keep_counts.items())
            )
        if self.atlas_max_sliver_owner_distance:
            result["atlas_max_sliver_owner_distance"] = float(
                self.atlas_max_sliver_owner_distance
            )
        return result


class _PatchVoronoiSurfaceCompileError(RuntimeError):
    """Ожидаемый локальный отказ geometry/backend compile одной surface."""

    def __init__(self, reason, edge_indices, details=""):
        self.reason = str(reason)
        self.edge_indices = tuple(sorted({int(index) for index in edge_indices}))
        self.details = str(details)
        message = self.reason
        if self.details:
            message += f": {self.details}"
        super().__init__(message)


def _stable_serialized_value(value, digits):
    if isinstance(value, (tuple, list)):
        return tuple(
            _stable_serialized_value(item, digits) for item in value
        )
    if isinstance(value, float):
        return round(value, digits)
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    try:
        return tuple(round(float(component), digits) for component in value)
    except TypeError:
        return repr(value)


def serialize_network_faces(network_faces, digits=8):
    """Детерминированный JSON-ready snapshot evaluator output."""

    digits = max(0, int(digits))
    records = []
    for face in network_faces:
        loop_items = [
            (
                _stable_serialized_value(key, digits),
                tuple(round(float(value), digits) for value in position),
                (
                    round(float(u_frac), digits),
                    round(float(v_length), digits),
                ),
            )
            for key, position, u_frac, v_length in zip(
                face.vert_keys,
                face.positions,
                face.u_fracs,
                face.v_lengths,
            )
        ]
        if loop_items:
            start = min(
                range(len(loop_items)),
                key=lambda index: tuple(
                    repr(item[0])
                    for item in loop_items[index:] + loop_items[:index]
                ),
            )
            loop_items = loop_items[start:] + loop_items[:start]
        records.append(
            {
                "surface_id": int(face.surface_id),
                "component_kind": str(face.component_kind),
                "component_side": str(face.component_side),
                "vert_keys": tuple(item[0] for item in loop_items),
                "positions": tuple(item[1] for item in loop_items),
                "uv": tuple(item[2] for item in loop_items),
            }
        )
    records.sort(
        key=lambda record: (
            record["surface_id"],
            record["component_kind"],
            record["component_side"],
            repr(record["vert_keys"]),
        )
    )
    ordered_keys = sorted(
        {
            key
            for record in records
            for key in record["vert_keys"]
        },
        key=repr,
    )
    index_by_key = {key: index for index, key in enumerate(ordered_keys)}
    face_loops = tuple(
        tuple(index_by_key[key] for key in record["vert_keys"])
        for record in records
    )
    edges = {
        tuple(sorted((loop[index], loop[(index + 1) % len(loop)])))
        for loop in face_loops
        for index in range(len(loop))
    }
    return {
        "round_digits": digits,
        "topology_signature": {
            "vertex_count": len(ordered_keys),
            "edge_count": len(edges),
            "face_count": len(face_loops),
            "face_loops": face_loops,
        },
        "faces": tuple(records),
    }


@dataclass(frozen=True)
class _DecalArrangementFace:
    """Cell face после общей surface-level edge conformity."""

    surface: _PatchVoronoiSurface
    site: _PatchVoronoiSite
    points: tuple[tuple[float, float], ...]
    crop: _CropComponent
    point_keys: tuple[object, ...] = ()
    uv_site: _PatchVoronoiSite | None = None
    uv_point_transform: tuple[float, float, float, float] | None = None
    uv_frame_transform: tuple[float, ...] | None = None
    uv_frame_owner_chart_id: int | None = None
    uv_frame_is_imported: bool = False
    declared_owner_token: object | None = None
    declared_semantic_class: object | None = None
    absorbed_corner_vertices: tuple[int, ...] = ()
    terminal_cut_vertices: tuple[int, ...] = ()
    # T7-P3.10: corner predicate-curves, реально входящие в boundary face.
    boundary_corner_vertices: tuple[int, ...] = ()
    boundary_corner_edges: tuple[tuple[object, object, tuple[int, ...]], ...] = ()
    boundary_crop_edges: tuple[tuple[object, object, tuple[object, ...]], ...] = ()
    boundary_semantic_edges: tuple[
        tuple[object, object, tuple[object, ...]], ...
    ] = ()
    # -1/+1 разрешает unbounded V только через соответствующий endpoint.
    uv_v_continuation_endpoint: int = 0
    transition_uv: tuple[tuple[object, float, float], ...] = ()


@dataclass(frozen=True)
class DecalArrangement:
    """Математическая decal-сеть до preview/final BMesh adapters."""

    faces: tuple[_DecalArrangementFace, ...]
    inserted_stations: int
    resolved_corner_views: tuple = ()


@dataclass(frozen=True)
class _M1TransitionSide:
    """Локальная копия канонического 1D atlas-transition (T1--T6)."""

    transition_key: object
    chart_id: int
    segment: tuple[tuple[float, float], tuple[float, float]]
    station_extent: int
    stations: tuple[int, ...]
    interior_sign: int
    interval_owners: tuple[tuple[int, int, object | None], ...]
    interval_semantics: tuple[tuple[int, int, object | None], ...] = ()
    single_declared_side: bool = False


@dataclass(frozen=True)
class _M1ImportedSemantic:
    """Одна открытая T7-P curve и owner-side semantic delegate."""

    transition_key: object
    owner_chart_id: int
    target_chart_id: int
    points: tuple[tuple[float, float], ...]
    owner_crop: _CropComponent
    owner_site: _PatchVoronoiSite
    owner_edge_indices: tuple[int, ...]
    owner_region_points: tuple[tuple[float, float], ...]
    local_to_owner: tuple[float, float, float, float]
    uv_local_to_owner: tuple[float, ...]
    target_region_points: tuple[tuple[float, float], ...] = ()
    anchor_stations: tuple[tuple[int, object, int], ...] = ()
    is_predicate_separator: bool = False
    is_endpoint_separator: bool = False


@dataclass(frozen=True)
class _M1UVFrameDelegate:
    """R5: site-frame владельца, перенесённый в consumer chart."""

    owner_chart_id: int
    target_chart_id: int
    edge_index: int
    owner_site: _PatchVoronoiSite
    local_to_owner: tuple[float, ...]
    absorbed_corner_vertices: tuple[int, ...] = ()
    terminal_cut_vertices: tuple[int, ...] = ()


@dataclass(frozen=True)
class _CropComponent:
    """Один convex runtime crop с семантикой, живущей до NetworkFace."""

    kind: str
    side: str
    points: tuple[tuple[float, float], ...]
    uv_anchors: tuple[tuple[float, float], ...] = ()
    v_origin: float = 0.0
    owner_site_indices: tuple[int, ...] = ()
    semantic_owner_id: object | None = None


@dataclass(frozen=True)
class _PendingArrangementFace:
    surface: _PatchVoronoiSurface
    site: _PatchVoronoiSite
    points: tuple[tuple[float, float], ...]
    crop: _CropComponent
    absorbed_corner_vertices: tuple[int, ...] = ()
    terminal_cut_vertices: tuple[int, ...] = ()


@dataclass(frozen=True)
class _TerminalBridgeStationPrefix:
    """Конструктивный station-prefix одного terminal contour route."""

    extent: float
    point: tuple[float, float]
    contour_points: tuple[tuple[float, float], ...]
    source_vertex_ids: tuple[int, ...] = ()


@dataclass(frozen=True)
class _TerminalBridgeGuide:
    """RM10: station contact вместе с пройденной contour-полилинией."""

    point: tuple[float, float]
    contour_points: tuple[tuple[float, float], ...]
    source_vertex_ids: tuple[int, ...] = ()
    route_id: int | None = None
    route_edge_ids: tuple[int, ...] = ()
    extent: float = 0.0
    route_reach: float = 0.0
    station_prefixes: tuple[_TerminalBridgeStationPrefix, ...] = ()
    saturated: bool = False
    revisit_guarded: bool = False
    station_clamped: bool = False
    contact_met: bool = False
    frozen: bool = False
    freeze_locus: object | None = None
    capacity_policy: CapacityPolicy = CapacityPolicy.SATURATE_PROVEN

    def __post_init__(self):
        object.__setattr__(
            self,
            "capacity_policy",
            CapacityPolicy(self.capacity_policy),
        )


@dataclass(frozen=True)
class _ResolvedArrangementPoint:
    """Affine lift endpoints и provenance одной materialized station."""

    position_zero: Vector
    position_full: Vector
    vert_key: object
    location: DomainLocation


@dataclass(frozen=True)
class PatchVoronoiVertexProvenance:
    """I5 source/edge/route/station facts одного output-loop."""

    source_face_id: object
    source_edge_id: int
    route_id: object
    station_key: object
    domain_location: DomainLocation | None


@dataclass(frozen=True)
class PatchVoronoiFaceProvenance:
    """Тотальная immutable provenance одной PatchVoronoi face."""

    source_face_ids: tuple[object, ...]
    source_edge_ids: tuple[int, ...]
    route_ids: tuple[object, ...]
    station_keys: tuple[object, ...]
    vertices: tuple[PatchVoronoiVertexProvenance, ...]
    semantic_owner_id: object | None = None

    def __post_init__(self):
        if not self.source_face_ids:
            raise ValueError("PATCH_VORONOI_SOURCE_FACE_PROVENANCE_MISSING")
        if not self.source_edge_ids:
            raise ValueError("PATCH_VORONOI_SOURCE_EDGE_PROVENANCE_MISSING")
        if not self.route_ids:
            raise ValueError("PATCH_VORONOI_ROUTE_PROVENANCE_MISSING")
        if not self.station_keys:
            raise ValueError("PATCH_VORONOI_STATION_PROVENANCE_MISSING")
        if len(self.vertices) != len(self.station_keys):
            raise ValueError("PATCH_VORONOI_VERTEX_PROVENANCE_DESYNC")


@dataclass(frozen=True)
class _JunctionPort:
    """Открытый core-to-rail луч математической decal-сети."""

    surface_id: int
    edge_index: int
    surface_normal: Vector
    core_position: Vector
    outer_key: tuple
    outer_position: Vector
    outer_u: float
    outer_v: float
    core_v: float
    core_provenance: PatchVoronoiVertexProvenance
    outer_provenance: PatchVoronoiVertexProvenance


def patch_voronoi_available():
    return pyvoronoi is not None


def _dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return sqrt(dx * dx + dy * dy)


def _sub2(point_a, point_b):
    return (point_a[0] - point_b[0], point_a[1] - point_b[1])


def _dot2(vector_a, vector_b):
    return vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1]


def _cross2(vector_a, vector_b):
    return vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]


def _norm2(vector):
    length = sqrt(_dot2(vector, vector))
    if length <= _GEOMETRY_EPS:
        return None
    return (vector[0] / length, vector[1] / length)


def _triangle_weights2(point, triangle):
    if len(triangle) != 3:
        return None
    point_a, point_b, point_c = triangle
    denominator = _cross2(
        _sub2(point_b, point_a), _sub2(point_c, point_a)
    )
    if abs(denominator) <= 1e-12:
        return None
    weight_b = _cross2(
        _sub2(point, point_a), _sub2(point_c, point_a)
    ) / denominator
    weight_c = _cross2(
        _sub2(point_b, point_a), _sub2(point, point_a)
    ) / denominator
    return 1.0 - weight_b - weight_c, weight_b, weight_c


def _dedupe_polygon(points, tolerance=1e-8):
    result = []
    for point in points:
        point = (float(point[0]), float(point[1]))
        if result and _dist2(result[-1], point) <= tolerance:
            continue
        result.append(point)
    if len(result) > 1 and _dist2(result[0], result[-1]) <= tolerance:
        result.pop()
    if len(result) < 3:
        return []
    return result


def _line_clip_intersection(point_a, point_b, clip_a, clip_b):
    """Пересечение subject edge с бесконечной прямой clip edge."""

    sx = point_b[0] - point_a[0]
    sy = point_b[1] - point_a[1]
    cx = clip_b[0] - clip_a[0]
    cy = clip_b[1] - clip_a[1]
    denom = sx * cy - sy * cx
    if abs(denom) < 1e-15:
        return point_a
    qx = clip_a[0] - point_a[0]
    qy = clip_a[1] - point_a[1]
    t = (qx * cy - qy * cx) / denom
    return (point_a[0] + sx * t, point_a[1] + sy * t)


def _clip_to_convex(points, clip_polygon):
    """Sutherland-Hodgman: произвольный polygon ∩ convex polygon."""

    clip = list(clip_polygon)
    if _polygon_area2(clip) < 0.0:
        clip.reverse()
    output = list(points)
    for index in range(len(clip)):
        if not output:
            break
        clip_a = clip[index]
        clip_b = clip[(index + 1) % len(clip)]

        def inside(point):
            return (
                (clip_b[0] - clip_a[0]) * (point[1] - clip_a[1])
                - (clip_b[1] - clip_a[1]) * (point[0] - clip_a[0])
            ) >= -1e-10

        source = output
        output = []
        previous = source[-1]
        previous_inside = inside(previous)
        for current in source:
            current_inside = inside(current)
            if current_inside != previous_inside:
                output.append(
                    _line_clip_intersection(
                        previous, current, clip_a, clip_b
                    )
                )
            if current_inside:
                output.append(current)
            previous = current
            previous_inside = current_inside
    return _dedupe_polygon(output)


def _clip_to_halfplane(points, clip_a, clip_b, keep_inside=True):
    """Обрезает polygon одной ориентированной half-plane."""

    source = list(points)
    if not source:
        return []

    def signed_distance(point):
        return (
            (clip_b[0] - clip_a[0]) * (point[1] - clip_a[1])
            - (clip_b[1] - clip_a[1]) * (point[0] - clip_a[0])
        )

    def inside(point):
        distance = signed_distance(point)
        return distance >= -1e-10 if keep_inside else distance <= 1e-10

    output = []
    previous = source[-1]
    previous_inside = inside(previous)
    for current in source:
        current_inside = inside(current)
        if current_inside != previous_inside:
            output.append(
                _line_clip_intersection(previous, current, clip_a, clip_b)
            )
        if current_inside:
            output.append(current)
        previous = current
        previous_inside = current_inside
    return _dedupe_polygon(output)


def _subtract_convex_polygon(points, clip_polygon):
    """Возвращает convex pieces ``points \\ clip_polygon``.

    На каждом ребре clip уже найденная внешняя часть окончательна, а
    оставшаяся внутренняя часть идёт к следующей half-plane. Поэтому pieces
    не перекрываются и не требуют общего 2D boolean backend.
    """

    clip = list(clip_polygon)
    if _polygon_area2(clip) < 0.0:
        clip.reverse()
    inside_parts = [list(points)]
    outside_parts = []
    for index, clip_a in enumerate(clip):
        clip_b = clip[(index + 1) % len(clip)]
        next_inside = []
        for polygon in inside_parts:
            outside = _clip_to_halfplane(
                polygon, clip_a, clip_b, keep_inside=False
            )
            if outside and abs(_polygon_area2(outside)) > 1e-12:
                outside_parts.append(outside)
            inside = _clip_to_halfplane(
                polygon, clip_a, clip_b, keep_inside=True
            )
            if inside and abs(_polygon_area2(inside)) > 1e-12:
                next_inside.append(inside)
        inside_parts = next_inside
        if not inside_parts:
            break
    return outside_parts


def _clip_to_triangle(points, triangle):
    return _clip_to_convex(points, triangle)


def _simplify_collinear_polygon(points, tolerance):
    """Убирает вычислительные stations на прямой, не меняя silhouette."""

    result = _dedupe_polygon(points, tolerance=tolerance)
    changed = True
    while changed and len(result) > 3:
        changed = False
        simplified = []
        count = len(result)
        for index, point in enumerate(result):
            previous = result[(index - 1) % count]
            following = result[(index + 1) % count]
            dx = following[0] - previous[0]
            dy = following[1] - previous[1]
            length = sqrt(dx * dx + dy * dy)
            if length <= tolerance:
                changed = True
                continue
            deviation = abs(
                dx * (point[1] - previous[1])
                - dy * (point[0] - previous[0])
            ) / length
            between = (
                (point[0] - previous[0]) * (point[0] - following[0])
                + (point[1] - previous[1]) * (point[1] - following[1])
            ) <= tolerance * tolerance
            if deviation <= tolerance and between:
                changed = True
                continue
            simplified.append(point)
        if len(simplified) < 3:
            break
        result = simplified
    return result


def _polygon_is_simple(points, tolerance=1e-8):
    """Отсекает contours, которые BMesh превратил бы в пересекающиеся tris."""

    count = len(points)
    if count < 3:
        return False
    quantum = max(tolerance, 1e-12)
    keys = [
        (round(point[0] / quantum), round(point[1] / quantum))
        for point in points
    ]
    if len(set(keys)) != count:
        return False

    def orientation(point_a, point_b, point_c):
        return (
            (point_b[0] - point_a[0]) * (point_c[1] - point_a[1])
            - (point_b[1] - point_a[1]) * (point_c[0] - point_a[0])
        )

    def on_segment(point_a, point_b, point):
        return (
            min(point_a[0], point_b[0]) - tolerance
            <= point[0]
            <= max(point_a[0], point_b[0]) + tolerance
            and min(point_a[1], point_b[1]) - tolerance
            <= point[1]
            <= max(point_a[1], point_b[1]) + tolerance
        )

    def intersects(point_a, point_b, point_c, point_d):
        turns = (
            orientation(point_a, point_b, point_c),
            orientation(point_a, point_b, point_d),
            orientation(point_c, point_d, point_a),
            orientation(point_c, point_d, point_b),
        )
        if (
            turns[0] * turns[1] < -tolerance * tolerance
            and turns[2] * turns[3] < -tolerance * tolerance
        ):
            return True
        return (
            (abs(turns[0]) <= tolerance and on_segment(point_a, point_b, point_c))
            or (abs(turns[1]) <= tolerance and on_segment(point_a, point_b, point_d))
            or (abs(turns[2]) <= tolerance and on_segment(point_c, point_d, point_a))
            or (abs(turns[3]) <= tolerance and on_segment(point_c, point_d, point_b))
        )

    # Sweep broad phase: прежний полный O(n²) перебор занимал почти весь
    # compile на sampled parabolic Voronoi cells. Exact narrow phase выше не
    # меняется; sweep только исключает пары с непересекающимися AABB.
    edges = []
    for edge_index, point_a in enumerate(points):
        point_b = points[(edge_index + 1) % count]
        edges.append(
            (
                min(point_a[0], point_b[0]) - tolerance,
                max(point_a[0], point_b[0]) + tolerance,
                min(point_a[1], point_b[1]) - tolerance,
                max(point_a[1], point_b[1]) + tolerance,
                edge_index,
                point_a,
                point_b,
            )
        )
    edges.sort(key=lambda edge: (edge[0], edge[2], edge[4]))
    active = []
    for edge in edges:
        min_x, max_x, min_y, max_y, edge_index, point_a, point_b = edge
        active = [candidate for candidate in active if candidate[1] >= min_x]
        for candidate in active:
            (
                _other_min_x,
                _other_max_x,
                other_min_y,
                other_max_y,
                other_index,
                point_c,
                point_d,
            ) = candidate
            if (
                other_index == edge_index
                or (other_index + 1) % count == edge_index
                or (edge_index + 1) % count == other_index
            ):
                continue
            if other_max_y < min_y or max_y < other_min_y:
                continue
            if intersects(point_a, point_b, point_c, point_d):
                return False
        active.append(edge)
    return True


def _polygon_is_convex(points, tolerance=1e-9):
    sign = 0
    for index, point in enumerate(points):
        point_a = points[(index - 1) % len(points)]
        point_b = points[(index + 1) % len(points)]
        turn = (
            (point[0] - point_a[0]) * (point_b[1] - point[1])
            - (point[1] - point_a[1]) * (point_b[0] - point[0])
        )
        if abs(turn) <= tolerance:
            continue
        current_sign = 1 if turn > 0.0 else -1
        if sign and current_sign != sign:
            return False
        sign = current_sign
    return sign != 0


def _convex_fragment_decomposition(polygons, tolerance):
    """Укрупняет triangle clips только когда их union остаётся convex."""

    result = [list(polygon) for polygon in polygons]
    quantum = max(tolerance, 1e-10)

    def point_key(point):
        return (round(point[0] / quantum), round(point[1] / quantum))

    while True:
        edge_owner = {}
        merge_pair = None
        for polygon_index, polygon in enumerate(result):
            for index, point in enumerate(polygon):
                other = polygon[(index + 1) % len(polygon)]
                undirected = tuple(sorted((point_key(point), point_key(other))))
                previous = edge_owner.get(undirected)
                if previous is not None and previous != polygon_index:
                    pair = tuple(sorted((previous, polygon_index)))
                    first = result[pair[0]]
                    second = result[pair[1]]
                    hull = _convex_hull(first + second)
                    source_area = abs(_polygon_area2(first)) + abs(
                        _polygon_area2(second)
                    )
                    area_tolerance = max(
                        tolerance * tolerance * 4.0,
                        source_area * 1e-7,
                    )
                    if (
                        len(hull) >= 3
                        and abs(abs(_polygon_area2(hull)) - source_area)
                        <= area_tolerance
                    ):
                        merge_pair = pair, hull
                        break
                else:
                    edge_owner[undirected] = polygon_index
            if merge_pair is not None:
                break
        if merge_pair is None:
            return result
        (first_index, second_index), hull = merge_pair
        result[first_index] = hull
        del result[second_index]


def _merge_polygon_fragments(
    fragments,
    tolerance=1e-7,
    diagnostics=None,
    normalize_t_junctions=False,
    merge_groups=None,
):
    """Собирает triangle-clips одной cell обратно в цельные contours.

    Internal edges совпадают попарно в противоположных направлениях и
    удаляются. Компоненты, касающиеся только вершиной, намеренно не слипаются.
    Если union содержит hole, возвращаем исходные fragments этой компоненты:
    Blender face не может хранить внутренний контур без разбиения.
    """

    quantum = max(tolerance, 1e-10)

    def point_key(point):
        return (
            round(point[0] / quantum),
            round(point[1] / quantum),
        )

    normalized = []
    normalized_merge_groups = []
    source_merge_groups = (
        tuple(merge_groups) if merge_groups is not None else ()
    )
    if len(source_merge_groups) != len(fragments):
        source_merge_groups = (0,) * len(fragments)
    for fragment_index, fragment in enumerate(fragments):
        polygon = _dedupe_polygon(fragment, tolerance=tolerance)
        if len(polygon) < 3:
            continue
        area = _polygon_area2(polygon)
        if abs(area) <= tolerance * tolerance:
            continue
        if area < 0.0:
            polygon.reverse()
        normalized.append(polygon)
        normalized_merge_groups.append(source_merge_groups[fragment_index])
    if len(normalized) <= 1:
        return normalized

    if normalize_t_junctions:
        # Разные Boost wheels могут разбить periodic cell boundary
        # неодинаково: одна clip-грань содержит T-station, соседняя остаётся
        # цельной. Без общей станции half-edge union видит разные компоненты;
        # при следующей ширине станция исчезает и получается topology-pop.
        # Нормализуем разбиение до построения edge ownership, а не надеемся
        # на поздний arrangement pass, где semantic faces уже разделены.
        normalized, _inserted = _insert_surface_edge_stations(
            normalized, tolerance
        )
    normalized_keys = [
        tuple(point_key(point) for point in polygon)
        for polygon in normalized
    ]

    parents = list(range(len(normalized)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(first, second):
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parents[second_root] = first_root

    edge_owners = {}
    for fragment_index, keys in enumerate(normalized_keys):
        for index, key_a in enumerate(keys):
            key_b = keys[(index + 1) % len(keys)]
            undirected = tuple(sorted((key_a, key_b)))
            previous_owners = edge_owners.setdefault(undirected, [])
            for previous_owner in previous_owners:
                if (
                    normalized_merge_groups[fragment_index]
                    == normalized_merge_groups[previous_owner]
                ):
                    union(fragment_index, previous_owner)
            previous_owners.append(fragment_index)

    groups = {}
    for fragment_index in range(len(normalized)):
        groups.setdefault(find(fragment_index), []).append(fragment_index)

    merged = []
    for group_indices in groups.values():
        boundary_edges = {}
        representatives = {}
        for fragment_index in group_indices:
            polygon = normalized[fragment_index]
            keys = normalized_keys[fragment_index]
            for index, point in enumerate(polygon):
                key_a = keys[index]
                key_b = keys[(index + 1) % len(keys)]
                other = polygon[(index + 1) % len(polygon)]
                representatives.setdefault(key_a, point)
                representatives.setdefault(key_b, other)
                reverse = (key_b, key_a)
                if reverse in boundary_edges:
                    del boundary_edges[reverse]
                else:
                    boundary_edges[(key_a, key_b)] = True

        outgoing = {}
        for key_a, key_b in boundary_edges:
            outgoing.setdefault(key_a, []).append(key_b)
        unused = set(boundary_edges)
        loops = []
        while unused:
            start_edge = next(iter(unused))
            start_key, current_key = start_edge
            unused.remove(start_edge)
            loop_keys = [start_key]
            previous_key = start_key
            guard = len(boundary_edges) + 1
            while current_key != start_key and guard > 0:
                loop_keys.append(current_key)
                candidates = [
                    next_key
                    for next_key in outgoing.get(current_key, ())
                    if (current_key, next_key) in unused
                ]
                if not candidates:
                    break
                # В vertex-contact могут сходиться несколько contours одной
                # edge-connected группы. Продолжаем по ближайшему clockwise
                # half-edge от обратного входящего направления: interior
                # каждого CCW contour остаётся слева, а касающиеся loops не
                # сшиваются случайной лексикографической хордой.
                current_point = representatives[current_key]
                previous_point = representatives[previous_key]
                reverse = (
                    previous_point[0] - current_point[0],
                    previous_point[1] - current_point[1],
                )

                def clockwise_turn(next_key):
                    next_point = representatives[next_key]
                    outgoing = (
                        next_point[0] - current_point[0],
                        next_point[1] - current_point[1],
                    )
                    cross = reverse[0] * outgoing[1] - reverse[1] * outgoing[0]
                    dot = reverse[0] * outgoing[0] + reverse[1] * outgoing[1]
                    return (-atan2(cross, dot)) % tau, next_key

                next_key = min(candidates, key=clockwise_turn)
                unused.remove((current_key, next_key))
                previous_key = current_key
                current_key = next_key
                guard -= 1
            if current_key != start_key or len(loop_keys) < 3:
                loops = []
                break
            polygon = [representatives[key] for key in loop_keys]
            polygon = _simplify_collinear_polygon(polygon, tolerance)
            if len(polygon) >= 3 and abs(_polygon_area2(polygon)) > tolerance * tolerance:
                loops.append(polygon)

        # Negative loop is a real hole. Только он требует decomposition:
        # простой concave contour Blender хранит одним корректным ngon.
        # Прежняя convex-only проверка отпечатывала triangulation domain в
        # митре ещё до реального столкновения соседних фронтов.
        source_area = sum(
            abs(_polygon_area2(normalized[index]))
            for index in group_indices
        )
        traced_area = sum(abs(_polygon_area2(loop)) for loop in loops)
        area_tolerance = max(
            tolerance * tolerance * max(8, len(group_indices) * 4),
            source_area * 1e-6,
        )
        if (
            not loops
            or any(_polygon_area2(loop) < 0.0 for loop in loops)
            or any(not _polygon_is_simple(loop, tolerance) for loop in loops)
            # Half-edge graph с T-junction stations может замкнуть формально
            # простой, но неполный contour. Geometry source fragments уже
            # непересекающиеся, поэтому их суммарная площадь — обязательный
            # инвариант union; при расхождении сохраняем её decomposition.
            or abs(traced_area - source_area) > area_tolerance
        ):
            if diagnostics is not None:
                diagnostics.convex_fragment_decomposition_fallbacks += 1
            merged.extend(
                _convex_fragment_decomposition(
                    [normalized[index] for index in group_indices],
                    tolerance,
                )
            )
            continue
        merged.extend(loops)
    return merged


def _patch_domain_triangles(
    node,
    origin,
    basis_u,
    basis_v,
    diagnostics=None,
    quantize_point=None,
):
    """Триангулирует только boundary loops, не topology owner faces."""

    def project_point(point):
        projected = _project(point, origin, basis_u, basis_v)
        return (
            quantize_point(projected)
            if quantize_point is not None
            else projected
        )

    domain_loops = []
    if _tessellate_polygon is not None:
        ordered_loops = sorted(
            (loop for loop in node.boundary_loops if len(loop.vert_cos) >= 3),
            key=lambda loop: (
                int(getattr(loop, "depth", 0)),
                str(getattr(loop, "kind", "")),
            ),
        )
        for boundary_loop in ordered_loops:
            points = _dedupe_polygon(
                [project_point(point) for point in boundary_loop.vert_cos]
            )
            if len(points) < 3:
                continue
            is_hole = int(getattr(boundary_loop, "depth", 0)) % 2 == 1
            area = _polygon_area2(points)
            if (not is_hole and area < 0.0) or (is_hole and area > 0.0):
                points.reverse()
            domain_loops.append(points)

    if domain_loops:
        flat_points = [point for loop in domain_loops for point in loop]
        vector_loops = [
            [Vector((point[0], point[1], 0.0)) for point in loop]
            for loop in domain_loops
        ]
        try:
            triangle_indices = _tessellate_polygon(vector_loops)
        except (RuntimeError, ValueError):
            triangle_indices = []
        triangles = []
        for indices in triangle_indices:
            if len(indices) != 3:
                continue
            triangle = [flat_points[int(index)] for index in indices]
            if abs(_polygon_area2(triangle)) <= 1e-12:
                continue
            if _polygon_area2(triangle) < 0.0:
                triangle.reverse()
            triangles.append(triangle)
        if triangles:
            return triangles

    # Compatibility fallback для старых serialized graphs/unit tests.
    if diagnostics is not None:
        diagnostics.tessellation_mesh_tri_fallbacks += 1
    triangles = []
    for tri in node.mesh_tris:
        points = [project_point(node.mesh_verts[index]) for index in tri]
        if abs(_polygon_area2(points)) > 1e-12:
            if _polygon_area2(points) < 0.0:
                points.reverse()
            triangles.append(points)
    return triangles


def _point_in_triangle(point, triangle, tolerance=1e-10):
    """Проверяет принадлежность point треугольнику в patch space."""

    signs = []
    for index in range(3):
        point_a = triangle[index]
        point_b = triangle[(index + 1) % 3]
        signs.append(
            (point_b[0] - point_a[0]) * (point[1] - point_a[1])
            - (point_b[1] - point_a[1]) * (point[0] - point_a[0])
        )
    return min(signs) >= -tolerance or max(signs) <= tolerance


def _point_in_domain(point, triangles, triangle_grid=None, reference=False):
    if reference or triangle_grid is None:
        triangle_ids = range(len(triangles))
    else:
        triangle_ids = triangle_grid.query_point(point)
    return any(
        _point_in_triangle(point, triangles[triangle_id])
        for triangle_id in triangle_ids
    )


def _inward_site_normal(
    point_a,
    point_b,
    triangles,
    probe_distance,
    triangle_grid=None,
    reference=False,
):
    """Выбирает нормаль segment в сторону owner patch, а не по winding chain."""

    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    length = sqrt(dx * dx + dy * dy)
    if length <= _GEOMETRY_EPS:
        return (0.0, 0.0)
    left = (-dy / length, dx / length)
    midpoint = ((point_a[0] + point_b[0]) * 0.5, (point_a[1] + point_b[1]) * 0.5)
    left_probe = (
        midpoint[0] + left[0] * probe_distance,
        midpoint[1] + left[1] * probe_distance,
    )
    right_probe = (
        midpoint[0] - left[0] * probe_distance,
        midpoint[1] - left[1] * probe_distance,
    )
    left_inside = _point_in_domain(
        left_probe, triangles, triangle_grid, reference
    )
    right_inside = _point_in_domain(
        right_probe, triangles, triangle_grid, reference
    )
    if left_inside != right_inside:
        return left if left_inside else (-left[0], -left[1])

    # На очень коротком edge probe может пересечь соседний угол. Ближайшая
    # owner triangle даёт устойчивый fallback без зависимости от winding.
    best_centroid = None
    best_distance = float("inf")
    if reference or triangle_grid is None:
        triangle_ids = range(len(triangles))
    else:
        triangle_ids = triangle_grid.query_point(
            midpoint, max(probe_distance, DECAL_WELD_DISTANCE)
        )
    for triangle_id in triangle_ids:
        triangle = triangles[triangle_id]
        centroid = (
            sum(point[0] for point in triangle) / 3.0,
            sum(point[1] for point in triangle) / 3.0,
        )
        distance = _dist2(midpoint, centroid)
        if distance < best_distance:
            best_distance = distance
            best_centroid = centroid
    if best_centroid is not None:
        towards_owner = (
            best_centroid[0] - midpoint[0],
            best_centroid[1] - midpoint[1],
        )
        if towards_owner[0] * left[0] + towards_owner[1] * left[1] < 0.0:
            return (-left[0], -left[1])
    return left


def _line_intersection(point_a, direction_a, point_b, direction_b):
    denominator = (
        direction_a[0] * direction_b[1]
        - direction_a[1] * direction_b[0]
    )
    if abs(denominator) <= 1e-12:
        return None
    delta = (point_b[0] - point_a[0], point_b[1] - point_a[1])
    factor = (
        delta[0] * direction_b[1] - delta[1] * direction_b[0]
    ) / denominator
    return (
        point_a[0] + direction_a[0] * factor,
        point_a[1] + direction_a[1] * factor,
    )


def _convex_hull(points):
    """Минимальный deterministic hull для runtime extrusion crop."""

    unique = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(unique) < 3:
        return []

    def cross(origin, point_a, point_b):
        return (
            (point_a[0] - origin[0]) * (point_b[1] - origin[1])
            - (point_a[1] - origin[1]) * (point_b[0] - origin[0])
        )

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-12:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-12:
            upper.pop()
        upper.append(point)
    return _dedupe_polygon(lower[:-1] + upper[:-1])


def _segment_crop_polygon(site, alpha):
    """Extrusion polygon segment-cell: прямой strip без endpoint fan."""

    dx = site.point_b[0] - site.point_a[0]
    dy = site.point_b[1] - site.point_a[1]
    length = max(site.segment_length, _GEOMETRY_EPS)
    normal = (-dy / length * alpha, dx / length * alpha)
    return [
        (site.point_a[0] + normal[0], site.point_a[1] + normal[1]),
        (site.point_b[0] + normal[0], site.point_b[1] + normal[1]),
        (site.point_b[0] - normal[0], site.point_b[1] - normal[1]),
        (site.point_a[0] - normal[0], site.point_a[1] - normal[1]),
    ]


def _terminal_segment_crop_components(
    site,
    alpha,
    start_guide=None,
    end_guide=None,
):
    """RM9-fix: rail-срез и прежнее тело как один SEGMENT owner."""

    if start_guide is None and end_guide is None:
        return (
            (
                _CropComponent(
                    kind="SEGMENT",
                    side="",
                    points=tuple(_segment_crop_polygon(site, alpha)),
                ),
                (),
            ),
        )
    if site.two_sided:
        raise RuntimeError(
            "TERMINAL_BRIDGE_TWO_SIDED_SITE_UNSUPPORTED: "
            f"patch={site.patch_id} edge={site.edge_index}"
        )
    tangent = _norm2(_sub2(site.point_b, site.point_a))
    inward = _norm2(site.inward_normal)
    if tangent is None or inward is None:
        raise RuntimeError(
            "TERMINAL_BRIDGE_SITE_FRAME_INVALID: "
            f"patch={site.patch_id} edge={site.edge_index}"
        )
    offset = (inward[0] * alpha, inward[1] * alpha)
    base_depth = min(alpha, site.segment_length * 0.5)

    def shifted(point, direction, distance):
        return (
            point[0] + direction[0] * distance,
            point[1] + direction[1] * distance,
        )

    def terminal_depth(corner, guide, direction):
        if guide is None:
            return 0.0
        guide_point = (
            guide.point if isinstance(guide, _TerminalBridgeGuide) else guide
        )
        # ``inward_normal`` — локальный frame исходного site, а не
        # глобальное полупространство developable owner-chart. На повороте
        # поверхности легальный pChain может пересечь продолжение site и
        # получить отрицательный dot. Принадлежность guide owner-домену уже
        # доказана структурно при чтении station route.
        return max(
            0.0,
            _dot2(_sub2(guide_point, corner), direction),
        ) + base_depth

    def terminal_spec(
        side, corner, guide, inner, outer, vertex_id, absorbed=False
    ):
        if guide is None:
            return None
        guide_point = (
            guide.point if isinstance(guide, _TerminalBridgeGuide) else guide
        )
        contour_points = (
            guide.contour_points
            if isinstance(guide, _TerminalBridgeGuide)
            else (corner, guide_point)
        )
        if not contour_points or contour_points[0] != corner:
            raise RuntimeError(
                "TERMINAL_BRIDGE_CONTOUR_START_DESYNC: "
                f"patch={site.patch_id} edge={site.edge_index} "
                f"vertex={vertex_id}"
            )
        if contour_points[-1] != guide_point:
            raise RuntimeError(
                "TERMINAL_BRIDGE_CONTOUR_END_DESYNC: "
                f"patch={site.patch_id} edge={site.edge_index} "
                f"vertex={vertex_id}"
            )
        # Полигон замыкается от contact к apex строго по route vertices.
        # Старое (corner, inner, outer, guide) проводило здесь хорду.
        polygon = (
            corner,
            inner,
            outer,
            guide_point,
            *reversed(contour_points[1:-1]),
        )
        if absorbed and len(contour_points) <= 2:
            polygon = tuple(_convex_hull(polygon))
        return {
            "side": side,
            "vertex_id": vertex_id,
            "polygon": polygon,
            "source_vertex_ids": tuple(
                sorted(
                    {
                        int(vertex_id),
                        *(
                            int(source_vertex_id)
                            for source_vertex_id in guide.source_vertex_ids
                        ),
                    }
                )
            )
            if isinstance(guide, _TerminalBridgeGuide)
            else (int(vertex_id),),
        }

    def terminal_components(spec):
        if spec is None:
            return ()
        polygon = spec["polygon"]
        triangles = _triangulate_cell_polygon(polygon)
        if not triangles:
            raise RuntimeError(
                "TERMINAL_BRIDGE_CUT_INVALID: "
                f"patch={site.patch_id} edge={site.edge_index} "
                f"vertex={spec['vertex_id']}"
            )
        return tuple(
            (
                _CropComponent(
                    kind="SEGMENT",
                    side=f"TERMINAL_{spec['side']}_{index}",
                    points=tuple(points),
                ),
                spec["source_vertex_ids"],
            )
            for index, points in enumerate(triangles)
        )

    start_depth = terminal_depth(
        site.point_a, start_guide, tangent
    )
    end_direction = (-tangent[0], -tangent[1])
    end_depth = terminal_depth(
        site.point_b, end_guide, end_direction
    )
    start_absorbed = False
    end_absorbed = False
    if start_depth + end_depth > site.segment_length:
        if start_guide is not None and end_guide is None:
            # Один терминальный срез может структурно поглотить весь
            # короткий site: BODY пуст, но второго владельца не возникает.
            start_depth = site.segment_length
            start_absorbed = True
        elif end_guide is not None and start_guide is None:
            end_depth = site.segment_length
            end_absorbed = True
        else:
            start_locus = getattr(start_guide, "freeze_locus", None)
            end_locus = getattr(end_guide, "freeze_locus", None)
            if (
                not getattr(start_guide, "frozen", False)
                or not getattr(end_guide, "frozen", False)
                or start_locus is None
                or start_locus is not end_locus
            ):
                raise RuntimeError(
                    "RAIL_COMPETITION_METRIC_UNRESOLVED: "
                    f"patch={site.patch_id} edge={site.edge_index} "
                    f"start_depth={start_depth!r} end_depth={end_depth!r} "
                    f"site_length={site.segment_length!r}"
                )
            arrivals = tuple(
                float(value) for value in start_locus.arrival_distances
            )
            arrival_total = sum(arrivals)
            if len(arrivals) != 2 or not arrival_total > 0.0:
                raise RuntimeError(
                    "RAIL_COMPETITION_METRIC_UNRESOLVED: "
                    f"patch={site.patch_id} edge={site.edge_index} "
                    f"arrivals={arrivals!r}"
                )
            # RC1: полное покрытие site делится только immutable станционной
            # метрикой freeze-locus. Геометрический overlap не голосует.
            start_route_id = getattr(start_guide, "route_id", None)
            arrival_by_route = dict(
                zip(start_locus.route_ids, arrivals)
            )
            start_arrival = arrival_by_route.get(
                start_route_id, arrivals[0]
            )
            start_depth = site.segment_length * (
                start_arrival / arrival_total
            )
            end_depth = site.segment_length - start_depth
    body_start = shifted(site.point_a, tangent, start_depth)
    body_end = shifted(site.point_b, tangent, -end_depth)
    start_spec = terminal_spec(
        "START",
        site.point_a,
        start_guide,
        body_start,
        (body_start[0] + offset[0], body_start[1] + offset[1]),
        site.vert_a,
        absorbed=start_absorbed,
    )
    end_spec = terminal_spec(
        "END",
        site.point_b,
        end_guide,
        body_end,
        (body_end[0] + offset[0], body_end[1] + offset[1]),
        site.vert_b,
        absorbed=end_absorbed,
    )
    body = _convex_hull(
        (
            body_start,
            (body_start[0] + offset[0], body_start[1] + offset[1]),
            (body_end[0] + offset[0], body_end[1] + offset[1]),
            body_end,
        )
    )
    result = []
    result.extend(terminal_components(start_spec))
    if body:
        result.append(
            (
                _CropComponent(
                    kind="SEGMENT", side="BODY", points=tuple(body)
                ),
                (),
            )
        )
    result.extend(terminal_components(end_spec))
    return tuple(result)


def _site_lateral_u(site, point, alpha):
    """Signed U для same-chart two-sided site, compatibility UV для остальных."""

    if site.two_sided:
        direction = _norm2(_sub2(site.point_b, site.point_a))
        if direction is not None:
            signed_lateral = _cross2(
                direction, _sub2(point, site.point_a)
            )
            return max(-1.0, min(1.0, signed_lateral / alpha))
    distance, _t = _segment_point_distance2(
        site.point_a, site.point_b, point
    )
    return site.uv_sign * max(0.0, min(1.0, distance / alpha))


def _site_v_length(site, parameter):
    uv_length = site.uv_length if site.uv_length > 0.0 else site.segment_length
    return site.arc_start + site.arc_sign * parameter * uv_length


def _site_unbounded_parameter(site, point):
    """Линейный parameter site без endpoint clamp для T7-P3.8."""

    direction = _sub2(site.point_b, site.point_a)
    length2 = _dot2(direction, direction)
    if length2 <= 1e-20:
        return 0.0
    return _dot2(_sub2(point, site.point_a), direction) / length2


def _corner_offset_lines(surface, corner, alpha):
    """Две ordered offset-линии intrinsic corner."""

    offset_lines = []
    for site_index in corner.ordered_sites:
        site = _corner_site_view(surface, corner, site_index)
        if site.segment_length <= _GEOMETRY_EPS:
            return []
        if site.vert_a == corner.vert_index:
            direction = (
                (site.point_b[0] - corner.point[0]) / site.segment_length,
                (site.point_b[1] - corner.point[1]) / site.segment_length,
            )
        else:
            direction = (
                (site.point_a[0] - corner.point[0]) / site.segment_length,
                (site.point_a[1] - corner.point[1]) / site.segment_length,
            )
        offset_point = (
            corner.point[0] + site.inward_normal[0] * alpha,
            corner.point[1] + site.inward_normal[1] * alpha,
        )
        offset_lines.append((offset_point, direction))
    return offset_lines


def _corner_offset_edge_relation(surface, corner, alpha=1.0):
    """RF24: различает щель и перекрытие по двум offset-рёбрам.

    У каждой incident strip-квады offset-ребро начинается в P и идёт
    вдоль source-segment от V. Если оба луча пересекаются впереди, квады
    перекрываются. Если пересечение лежит позади обоих лучей, между ними
    щель. В предикате нет normal/winding sign и нет числового порога.
    """

    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return "NONE"
    point_a, direction_a = offset_lines[0]
    point_b, direction_b = offset_lines[1]
    denominator = _cross2(direction_a, direction_b)
    if denominator == 0.0:
        return "NONE"
    delta = _sub2(point_b, point_a)
    factor_a = _cross2(delta, direction_b) / denominator
    factor_b = _cross2(delta, direction_a) / denominator
    if factor_a > 0.0 and factor_b > 0.0:
        return "OVERLAP"
    if factor_a < 0.0 and factor_b < 0.0:
        return "GAP"
    if factor_a == 0.0 and factor_b == 0.0:
        return "NONE"
    raise RuntimeError(
        "BEVEL_JOIN_SIDE_AMBIGUOUS: "
        f"patch={surface.patch_id} vertex={corner.vert_index} "
        f"edge_factors=({factor_a!r},{factor_b!r})"
    )


def _corner_static_wedge_polygon(surface, corner):
    """Compiled-domain wedge между endpoint perpendicular rays.

    FAN/SPLIT-компоненты растут внутри этого клина. Его боковые границы
    проходят через endpoint и не зависят от ``alpha``; поэтому incident
    strips должны уступать клин целиком, а не вычитаться движущимся
    фронтиром corner crop.
    """

    return list(corner.static_wedge)


def _kite_crop_polygon(
    surface, corner, alpha, settings=None, diagnostics=None
):
    """Один realtime kite из двух endpoint triangles convex corner."""

    settings = _normalized_corner_runtime_settings(settings)
    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return []
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    if intersection is None or (
        _dist2(corner.point, intersection) > alpha * settings.apex_limit
    ):
        if diagnostics is not None:
            diagnostics.clamped_kite_count += 1
        return _convex_hull(
            [corner.point, offset_lines[0][0], offset_lines[1][0]]
        )
    return _convex_hull(
        [
            corner.point,
            offset_lines[0][0],
            intersection,
            offset_lines[1][0],
        ]
    )


def _corner_arc_origin(surface, corner):
    values = []
    for site_index in corner.ordered_sites:
        site = _corner_site_view(surface, corner, site_index)
        values.append(
            _site_v_length(
                site, 0.0 if site.vert_a == corner.vert_index else 1.0
            )
        )
    return sum(values) / len(values) if values else 0.0


def _corner_frame_axes(surface, corner, alpha):
    """Канонический UV frame: биссектриса +V, turn parity задаёт +U."""

    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return None
    cap_a = offset_lines[0][0]
    cap_b = offset_lines[1][0]
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    target = intersection
    if target is None:
        target = (
            (cap_a[0] + cap_b[0]) * 0.5,
            (cap_a[1] + cap_b[1]) * 0.5,
        )
    forward = _norm2(_sub2(target, corner.point))
    if forward is None:
        return None
    lateral = (-forward[1], forward[0])
    if corner.turn_sign < 0.0:
        lateral = (-lateral[0], -lateral[1])
    return lateral, forward


def _corner_component_from_polygon(
    surface,
    corner,
    alpha,
    kind,
    side,
    polygon,
    *,
    owner_site_indices=(),
):
    """Создаёт semantic crop с deterministic owner-independent UV."""

    frame = _corner_frame_axes(surface, corner, alpha)
    if frame is None:
        return None
    lateral, forward = frame
    scale = max(alpha, _GEOMETRY_EPS)
    anchors = []
    for point in polygon:
        delta = _sub2(point, corner.point)
        anchors.append(
            (
                point,
                (
                    _dot2(delta, lateral) / scale,
                    _dot2(delta, forward),
                ),
            )
        )
    component = _crop_component_from_anchors(
        kind,
        side,
        tuple(anchors),
        _corner_arc_origin(surface, corner),
    )
    if component is None:
        return None
    return replace(
        component,
        owner_site_indices=tuple(owner_site_indices),
    )


def _corner_components_from_semantic_polygon(
    surface,
    corner,
    alpha,
    kind,
    side,
    polygon,
    *,
    owner_site_indices,
):
    """Сохраняет вогнутый semantic contour без подмены convex hull.

    ``_CropComponent`` намеренно convex: clipping и subtraction полагаются
    на этот контракт. Поэтому вогнутый CornerModel раскладывается на
    детерминированные ears, а не расширяется до hull. Все pieces сохраняют
    одного semantic owner; post-competition view по-прежнему читает одну
    модель и те же authoritative anchors.
    """

    convex_parts = _triangulate_cell_polygon(polygon)
    return tuple(
        component
        for component in (
            _corner_component_from_polygon(
                surface,
                corner,
                alpha,
                kind,
                side,
                part,
                owner_site_indices=owner_site_indices,
            )
            for part in convex_parts
        )
        if component is not None
    )


def _cap_crop_components(surface, corner, alpha):
    """FLAT CAP: tangent-aligned terminal half-quad, без axis square."""

    if len(corner.incident_sites) != 1:
        return ()
    site_index = corner.incident_sites[0]
    site = _corner_site_view(surface, corner, site_index)
    other = site.point_b if site.vert_a == corner.vert_index else site.point_a
    tangent = _norm2(_sub2(other, corner.point))
    if tangent is None:
        return ()
    lateral = (-tangent[1], tangent[0])
    depth = min(alpha, site.segment_length * 0.5)
    cap_a = (
        corner.point[0] - lateral[0] * alpha,
        corner.point[1] - lateral[1] * alpha,
    )
    cap_b = (
        corner.point[0] + lateral[0] * alpha,
        corner.point[1] + lateral[1] * alpha,
    )
    inner_b = (cap_b[0] + tangent[0] * depth, cap_b[1] + tangent[1] * depth)
    inner_a = (cap_a[0] + tangent[0] * depth, cap_a[1] + tangent[1] * depth)
    v_origin = _site_v_length(
        site, 0.0 if site.vert_a == corner.vert_index else 1.0
    )
    component = _crop_component_from_anchors(
        _CornerPolicy.CAP.value,
        "START" if site.vert_a == corner.vert_index else "END",
        (
            (cap_a, (-1.0, 0.0)),
            (cap_b, (1.0, 0.0)),
            (inner_b, (1.0, depth)),
            (inner_a, (-1.0, depth)),
        ),
        v_origin,
    )
    if component is None:
        return ()
    return (replace(component, owner_site_indices=(site_index,)),)


def _smooth_crop_components(surface, corner, alpha, settings=None):
    """Angle-only pass-through: две SEGMENT-половины по биссектрисе."""

    if len(corner.ordered_sites) != 2:
        return ()
    polygon = _corner_crop_polygon(
        surface,
        corner,
        _CornerPolicy.MITER,
        alpha,
        settings,
    )
    if len(polygon) < 3:
        return ()
    rays = []
    for site_index in corner.ordered_sites:
        site = _corner_site_view(surface, corner, site_index)
        other = (
            site.point_b
            if site.vert_a == corner.vert_index
            else site.point_a
        )
        ray = _norm2(_sub2(other, corner.point))
        if ray is None:
            return ()
        rays.append(ray)
    bisector = _norm2(
        (rays[0][0] + rays[1][0], rays[0][1] + rays[1][1])
    )
    if bisector is None:
        return ()
    bisector_end = (
        corner.point[0] + bisector[0],
        corner.point[1] + bisector[1],
    )
    sites = tuple(
        _corner_site_view(surface, corner, site_index)
        for site_index in corner.ordered_sites
    )

    def site_uv(site, point):
        parameter = _site_unbounded_parameter(site, point)
        return (
            _site_lateral_u(site, point, alpha),
            _site_v_length(site, parameter),
        )

    split_tolerance = max(DECAL_WELD_DISTANCE * 0.1, 1e-9)
    components = []
    for site_index, site, ray in zip(
        corner.ordered_sites,
        sites,
        rays,
    ):
        ray_side = _cross2(bisector, ray)
        half = _clip_to_halfplane(
            polygon,
            corner.point,
            bisector_end,
            keep_inside=ray_side >= 0.0,
        )
        if len(half) < 3:
            continue
        anchors = []
        for point in half:
            on_bisector = abs(
                _cross2(bisector, _sub2(point, corner.point))
            ) <= split_tolerance
            if on_bisector:
                uv_values = tuple(
                    site_uv(candidate, point) for candidate in sites
                )
                uv = (
                    sum(value[0] for value in uv_values) * 0.5,
                    sum(value[1] for value in uv_values) * 0.5,
                )
            else:
                uv = site_uv(site, point)
            anchors.append((point, uv))
        component = _crop_component_from_anchors(
            "SEGMENT",
            "",
            tuple(anchors),
        )
        if component is not None:
            components.append(
                replace(
                    component,
                    owner_site_indices=(site_index,),
                )
            )
    return tuple(components)


def _limited_corner_apex(
    corner, intersection, alpha, settings, diagnostics=None
):
    distance = _dist2(corner.point, intersection)
    limit = alpha * settings.apex_limit
    if distance <= limit or distance <= _GEOMETRY_EPS:
        return intersection
    if diagnostics is not None:
        diagnostics.clamped_kite_count += 1
    direction = _norm2(_sub2(intersection, corner.point))
    return (
        corner.point[0] + direction[0] * limit,
        corner.point[1] + direction[1] * limit,
    )


def _fan_crop_components(
    surface, corner, alpha, settings=None, diagnostics=None
):
    """Два независимых triangle-компонента по биссектрисе kite."""

    settings = _normalized_corner_runtime_settings(settings)
    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return ()
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    if intersection is None:
        return ()
    apex = _limited_corner_apex(
        corner, intersection, alpha, settings, diagnostics
    )
    owner_sites = tuple(corner.incident_sites)
    components = []
    for side, cap in (("A", offset_lines[0][0]), ("B", offset_lines[1][0])):
        component = _corner_component_from_polygon(
            surface,
            corner,
            alpha,
            _CornerPolicy.FAN.value,
            side,
            (corner.point, cap, apex),
            owner_site_indices=owner_sites,
        )
        if component is not None:
            components.append(component)
    return tuple(components)


def _crop_component_from_anchors(kind, side, anchors, v_origin=0.0):
    """Сохраняет UV anchors после deterministic hull ordering."""

    polygon = _convex_hull([point for point, _uv in anchors])
    if len(polygon) < 3:
        return None
    quantum = max(DECAL_WELD_DISTANCE * 0.1, 1e-9)

    def key(point):
        return (
            round(point[0] / quantum),
            round(point[1] / quantum),
        )

    uv_by_point = {key(point): uv for point, uv in anchors}
    return _CropComponent(
        kind=kind,
        side=side,
        points=tuple(polygon),
        uv_anchors=tuple(uv_by_point[key(point)] for point in polygon),
        v_origin=v_origin,
    )


def _clamped_acute_apex(
    corner_point,
    cap_a,
    cap_b,
    intersection,
    alpha,
    settings,
    diagnostics,
):
    """Ограничивает outer apex, не перенося его внутрь cap chord."""

    ray = _sub2(intersection, corner_point)
    apex_distance = sqrt(_dot2(ray, ray))
    if apex_distance <= _GEOMETRY_EPS:
        return intersection
    ray_direction = (ray[0] / apex_distance, ray[1] / apex_distance)
    chord_direction = _sub2(cap_b, cap_a)
    crossing = _line_intersection(
        corner_point,
        ray_direction,
        cap_a,
        chord_direction,
    )
    epsilon = max(_GEOMETRY_EPS * 10.0, alpha * 1e-6)
    minimum_distance = 0.0
    if crossing is not None:
        crossing_ray = _sub2(crossing, corner_point)
        crossing_distance = _dot2(crossing_ray, ray_direction)
        if crossing_distance > 0.0:
            minimum_distance = crossing_distance + epsilon
    requested_distance = alpha * settings.apex_limit
    effective_limit = requested_distance
    if requested_distance < minimum_distance:
        effective_limit = minimum_distance
        if diagnostics is not None:
            diagnostics.apex_limit_saturated_count += 1
    target_distance = min(apex_distance, effective_limit)
    if target_distance < apex_distance - epsilon:
        if diagnostics is not None:
            diagnostics.clamped_acute_count += 1
        return (
            corner_point[0] + ray_direction[0] * target_distance,
            corner_point[1] + ray_direction[1] * target_distance,
        )
    return intersection


def _acute_crop_components(
    surface, corner, alpha, settings=None, diagnostics=None
):
    """Разделяет острый kite статичной compiled split-хордой S1."""

    settings = _normalized_corner_runtime_settings(settings)
    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return ()
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    if intersection is None:
        fallback = _crop_component_from_anchors(
            _CornerPolicy.ACUTE_SPLIT.value,
            "INNER",
            (
                (corner.point, (0.0, 0.0)),
                (offset_lines[0][0], (-1.0, alpha)),
                (offset_lines[1][0], (1.0, alpha)),
            ),
            _corner_arc_origin(surface, corner),
        )
        return (fallback,) if fallback is not None else ()

    cap_a = offset_lines[0][0]
    cap_b = offset_lines[1][0]
    outer_apex = _clamped_acute_apex(
        corner.point,
        cap_a,
        cap_b,
        intersection,
        alpha,
        settings,
        diagnostics,
    )
    owner_sites = tuple(corner.incident_sites)
    split_chord = corner.split_chord
    if len(split_chord) != 2:
        # Compile fallback локализован: без статичной хорды не создаём
        # alpha-зависимый внутренний шов, но сохраняем непрерывный crop.
        inner = _corner_component_from_polygon(
            surface,
            corner,
            alpha,
            _CornerPolicy.ACUTE_SPLIT.value,
            "INNER",
            (corner.point, cap_a, outer_apex, cap_b),
            owner_site_indices=owner_sites,
        )
        return (inner,) if inner is not None else ()

    split_a, split_b = split_chord
    reach_alpha = max(
        _dist2(corner.point, split_a),
        _dist2(corner.point, split_b),
    )
    if alpha + _GEOMETRY_EPS < reach_alpha:
        # До collision со split-хордой весь клин — один INNER crop;
        # движущиеся offset-рельсы остаются только внешним фронтиром.
        inner = _corner_component_from_polygon(
            surface,
            corner,
            alpha,
            _CornerPolicy.ACUTE_SPLIT.value,
            "INNER",
            (corner.point, cap_a, outer_apex, cap_b),
            owner_site_indices=owner_sites,
        )
        return (inner,) if inner is not None else ()

    inner = _corner_component_from_polygon(
        surface,
        corner,
        alpha,
        _CornerPolicy.ACUTE_SPLIT.value,
        "INNER",
        (corner.point, split_a, split_b),
        owner_site_indices=owner_sites,
    )

    frame = _corner_frame_axes(surface, corner, alpha)
    outer = None
    if frame is not None:
        lateral, forward = frame
        split_midpoint = (
            (split_a[0] + split_b[0]) * 0.5,
            (split_a[1] + split_b[1]) * 0.5,
        )
        scale = max(alpha, _GEOMETRY_EPS)
        anchors = []
        for point in (split_a, cap_a, outer_apex, cap_b, split_b):
            anchors.append(
                (
                    point,
                    (
                        _dot2(_sub2(point, corner.point), lateral) / scale,
                        _dot2(_sub2(point, split_midpoint), forward),
                    ),
                )
            )
        outer = _crop_component_from_anchors(
            _CornerPolicy.ACUTE_SPLIT.value,
            "OUTER",
            tuple(anchors),
            _corner_arc_origin(surface, corner),
        )
        if outer is not None:
            outer = replace(outer, owner_site_indices=owner_sites)
    return tuple(component for component in (inner, outer) if component)


def _stable_acute_crop_components(
    surface, corner, alpha, settings=None, diagnostics=None
):
    """A10 acute split: движется только внешний фронтир crop.

    Stable path намеренно не вводит отдельную compiled-хорду. Semantic crop
    целиком пересекается с неизменной Voronoi arrangement, поэтому уже
    разрешённые cell boundaries не пересчитываются при росте ширины.
    """

    settings = _normalized_corner_runtime_settings(settings)
    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return ()
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    if intersection is None:
        fallback = _crop_component_from_anchors(
            _CornerPolicy.ACUTE_SPLIT.value,
            "INNER",
            (
                (corner.point, (0.0, 0.0)),
                (offset_lines[0][0], (-1.0, alpha)),
                (offset_lines[1][0], (1.0, alpha)),
            ),
            _corner_arc_origin(surface, corner),
        )
        return (fallback,) if fallback is not None else ()

    cap_a = offset_lines[0][0]
    cap_b = offset_lines[1][0]
    chord_midpoint = (
        (cap_a[0] + cap_b[0]) * 0.5,
        (cap_a[1] + cap_b[1]) * 0.5,
    )
    outer_apex = _clamped_acute_apex(
        corner.point,
        cap_a,
        cap_b,
        intersection,
        alpha,
        settings,
        diagnostics,
    )
    inner_height = _dist2(corner.point, chord_midpoint)
    outer_height = _dist2(outer_apex, chord_midpoint)
    orientation = 1.0 if corner.turn_sign >= 0.0 else -1.0
    v_origin = _corner_arc_origin(surface, corner)
    inner = _crop_component_from_anchors(
        _CornerPolicy.ACUTE_SPLIT.value,
        "INNER",
        (
            (corner.point, (0.0, 0.0)),
            (cap_a, (-orientation, inner_height)),
            (cap_b, (orientation, inner_height)),
        ),
        v_origin,
    )
    outer = _crop_component_from_anchors(
        _CornerPolicy.ACUTE_SPLIT.value,
        "OUTER",
        (
            (cap_a, (-orientation, 0.0)),
            (outer_apex, (0.0, outer_height)),
            (cap_b, (orientation, 0.0)),
        ),
        v_origin,
    )
    return tuple(component for component in (inner, outer) if component)


def _hairpin_crop_components(
    surface, corner, alpha, settings=None, diagnostics=None
):
    """S1 hairpin: статичный inner + растущий тупой торец без spike."""

    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return ()
    cap_a = offset_lines[0][0]
    cap_b = offset_lines[1][0]
    owner_sites = tuple(corner.incident_sites)
    split_chord = corner.split_chord
    if diagnostics is not None:
        diagnostics.apex_limit_saturated_count += 1
    if len(split_chord) != 2:
        blunt = _corner_component_from_polygon(
            surface,
            corner,
            alpha,
            _CornerPolicy.HAIRPIN.value,
            "BLUNT",
            (corner.point, cap_a, cap_b),
            owner_site_indices=owner_sites,
        )
        return (blunt,) if blunt is not None else ()

    split_a, split_b = split_chord
    reach_alpha = max(
        _dist2(corner.point, split_a),
        _dist2(corner.point, split_b),
    )
    if alpha + _GEOMETRY_EPS < reach_alpha:
        blunt = _corner_component_from_polygon(
            surface,
            corner,
            alpha,
            _CornerPolicy.HAIRPIN.value,
            "BLUNT",
            (corner.point, cap_a, cap_b),
            owner_site_indices=owner_sites,
        )
        return (blunt,) if blunt is not None else ()

    inner = _corner_component_from_polygon(
        surface,
        corner,
        alpha,
        _CornerPolicy.HAIRPIN.value,
        "INNER",
        (corner.point, split_a, split_b),
        owner_site_indices=owner_sites,
    )
    blunt = _corner_component_from_polygon(
        surface,
        corner,
        alpha,
        _CornerPolicy.HAIRPIN.value,
        "BLUNT",
        (split_a, cap_a, cap_b, split_b),
        owner_site_indices=owner_sites,
    )
    return tuple(component for component in (inner, blunt) if component)


def _junction_sector_specs(surface, corner):
    """Occupied non-reflex angular sectors valence-N junction."""

    rays = {}
    for site_index in corner.incident_sites:
        site = _corner_site_view(surface, corner, site_index)
        other = site.point_b if site.vert_a == corner.vert_index else site.point_a
        ray = _norm2(_sub2(other, corner.point))
        if ray is not None:
            rays[site_index] = ray
    ordered = tuple(
        sorted(
            rays,
            key=lambda site_index: (
                atan2(rays[site_index][1], rays[site_index][0]),
                site_index,
            ),
        )
    )
    if len(ordered) < 3:
        return ()
    specs = []
    for sector_index, first_index in enumerate(ordered):
        second_index = ordered[(sector_index + 1) % len(ordered)]
        first_angle = atan2(rays[first_index][1], rays[first_index][0])
        second_angle = atan2(rays[second_index][1], rays[second_index][0])
        sector_angle = (second_angle - first_angle) % tau
        # Reflex sector не занят patch-domain; straight sector pass-through.
        if sector_angle >= pi - 1e-7 or sector_angle <= 1e-7:
            continue
        specs.append(
            (
                sector_index,
                replace(
                    corner,
                    incident_sites=(first_index, second_index),
                    ordered_sites=(first_index, second_index),
                    turn_sign=1.0,
                    interior_angle=sector_angle,
                    extrusion_angle=sector_angle,
                    is_convex=True,
                ),
            )
        )
    return tuple(specs)


def _junction_crop_components(
    surface, corner, alpha, settings, diagnostics=None
):
    """JUNCTION = deterministic fan независимых A11 corner sectors."""

    components = []
    for sector_index, sector in _junction_sector_specs(surface, corner):
        policy = _classify_extrusion_angle(
            sector.extrusion_angle, settings
        )
        sector_components = _corner_crop_components(
            surface,
            sector,
            policy,
            alpha,
            settings,
            diagnostics,
        )
        for component in sector_components:
            side = f"SECTOR_{sector_index}"
            if component.side:
                side += f"_{component.side}"
            components.append(
                replace(
                    component,
                    side=side,
                    owner_site_indices=sector.incident_sites,
                )
            )
    return tuple(components)


def _corner_runtime_policy_entries(surface, corner, policy, settings):
    """Evaluator-owned semantic counts; JUNCTION разворачивается в sectors."""

    if policy != _CornerPolicy.JUNCTION:
        return (policy,)
    return tuple(
        _classify_extrusion_angle(sector.extrusion_angle, settings)
        for _sector_index, sector in _junction_sector_specs(surface, corner)
    )


def _corner_crop_components(
    surface,
    corner,
    policy,
    alpha,
    settings,
    diagnostics=None,
    terminal_guide=None,
    corner_model=None,
    corner_derived=None,
):
    settings = _normalized_corner_runtime_settings(settings)
    if corner_model is not None:
        derived = corner_derived or corner_model.derive(
            apex_limit=alpha * settings.apex_limit
        )
        if derived.releases_competitor or policy in {
            _CornerPolicy.MITER,
            _CornerPolicy.KITE,
        }:
            if derived.miter_clamped and diagnostics is not None:
                diagnostics.clamped_miter_count += 1
            owner_site = _corner_site_view(
                surface, corner, corner.ordered_sites[0]
            )
            side_sign = -1 if owner_site.uv_sign < 0.0 else 1
            components = _corner_components_from_semantic_polygon(
                surface,
                corner,
                alpha,
                derived.component_kind,
                (
                    f"S_CM_A_v{corner.vert_index}_e{owner_site.edge_index}_"
                    f"s{side_sign:+d}"
                ),
                tuple(
                    vertex.chart_point
                    for vertex in derived.collision_boundary
                ),
                owner_site_indices=corner.incident_sites,
            )
            return tuple(
                replace(
                    component,
                    semantic_owner_id=(
                        "corner-model",
                        int(corner.vert_index),
                        corner_model.seed.sector_id,
                    ),
                )
                for component in components
            )
    # FLAT CAP — базовая endpoint-семантика, а не dynamic band. Stable path
    # обязан использовать tangent-aligned half-quad; axis-aligned square
    # остаётся явный compile failure для неподдержанного valence-N junction.
    if policy == _CornerPolicy.CAP:
        # RM9-fix: guide не создаёт отдельный CAP-owner. Он режет общий
        # SEGMENT crop ниже; point-cell становится частью той же ленты.
        if terminal_guide is not None:
            return ()
        return _cap_crop_components(surface, corner, alpha)
    if policy == _CornerPolicy.SMOOTH:
        return _smooth_crop_components(surface, corner, alpha, settings)
    if not settings.dynamic_corner_bands:
        if policy == _CornerPolicy.ACUTE_SPLIT:
            return _stable_acute_crop_components(
                surface, corner, alpha, settings, diagnostics
            )
        polygon = _corner_crop_polygon(
            surface,
            corner,
            policy,
            alpha,
            settings,
            diagnostics,
        )
        if len(polygon) < 3:
            return ()
        return (
            _CropComponent(
                kind=policy.value,
                side="",
                points=tuple(polygon),
            ),
        )
    if policy == _CornerPolicy.JUNCTION:
        return _junction_crop_components(
            surface, corner, alpha, settings, diagnostics
        )
    if policy == _CornerPolicy.FAN:
        return _fan_crop_components(
            surface, corner, alpha, settings, diagnostics
        )
    if policy == _CornerPolicy.ACUTE_SPLIT:
        return _acute_crop_components(
            surface, corner, alpha, settings, diagnostics
        )
    if policy == _CornerPolicy.HAIRPIN:
        return _hairpin_crop_components(
            surface, corner, alpha, settings, diagnostics
        )
    polygon = _corner_crop_polygon(
        surface,
        corner,
        policy,
        alpha,
        settings,
        diagnostics,
    )
    if len(polygon) < 3:
        return ()
    component = _corner_component_from_polygon(
        surface,
        corner,
        alpha,
        policy.value,
        "",
        polygon,
        owner_site_indices=corner.incident_sites,
    )
    return (component,) if component is not None else ()


def _corner_endpoint_ownership_crop(surface, corner, crop):
    """Ограничивает crop детерминированной половиной incident sites.

    Endpoint ``point_a`` владеет ``t <= 0.5``, endpoint ``point_b`` —
    ``t >= 0.5``. Divider определяется физическим segment, поэтому результат
    не зависит от corner index, site index и направления site endpoints.
    """

    points = list(crop.points)

    def site_key(site_index):
        site = _corner_site_view(surface, corner, site_index)
        return (
            site.edge_index,
            min(site.vert_a, site.vert_b),
            max(site.vert_a, site.vert_b),
            min(site.point_a, site.point_b),
            max(site.point_a, site.point_b),
        )

    owner_site_indices = (
        crop.owner_site_indices or corner.incident_sites
    )
    for site_index in sorted(set(owner_site_indices), key=site_key):
        site = _corner_site_view(surface, corner, site_index)
        if corner.vert_index == site.vert_a:
            keep_point_a = True
        elif corner.vert_index == site.vert_b:
            keep_point_a = False
        else:
            continue
        midpoint = (
            (site.point_a[0] + site.point_b[0]) * 0.5,
            (site.point_a[1] + site.point_b[1]) * 0.5,
        )
        direction = _sub2(site.point_b, site.point_a)
        if _dot2(direction, direction) <= _GEOMETRY_EPS * _GEOMETRY_EPS:
            continue
        divider_end = (
            midpoint[0] - direction[1],
            midpoint[1] + direction[0],
        )
        points = _clip_to_halfplane(
            points,
            midpoint,
            divider_end,
            keep_inside=keep_point_a,
        )
        if not points:
            return None

    points = tuple(points)
    if points == crop.points:
        return crop
    uv_anchors = ()
    if len(crop.uv_anchors) == len(crop.points):
        mapped_anchors = []
        for point in points:
            uv = _crop_component_uv(crop, point)
            if uv is None:
                mapped_anchors = []
                break
            mapped_anchors.append((uv[0], uv[1] - crop.v_origin))
        uv_anchors = tuple(mapped_anchors)
    return replace(crop, points=points, uv_anchors=uv_anchors)


def _miter_requires_explicit_crop(surface, corner, alpha, settings):
    """Implicit Voronoi join достаточен, пока apex не требуется усекать."""

    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return False
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    return (
        intersection is not None
        and _dist2(corner.point, intersection) > alpha * settings.apex_limit
    )


def _crop_component_uv(crop, point):
    """Affine UV component; None оставляет site UV."""

    if len(crop.points) < 3 or len(crop.uv_anchors) != len(crop.points):
        return None
    anchor_indices = None
    for second_index in range(1, len(crop.points) - 1):
        for third_index in range(second_index + 1, len(crop.points)):
            candidate = (0, second_index, third_index)
            candidate_points = tuple(crop.points[index] for index in candidate)
            denominator = _cross2(
                _sub2(candidate_points[1], candidate_points[0]),
                _sub2(candidate_points[2], candidate_points[0]),
            )
            if abs(denominator) > 1e-12:
                anchor_indices = candidate
                break
        if anchor_indices is not None:
            break
    if anchor_indices is None:
        return None
    point_a, point_b, point_c = (
        crop.points[index] for index in anchor_indices
    )
    denominator = _cross2(
        _sub2(point_b, point_a), _sub2(point_c, point_a)
    )
    weight_b = _cross2(
        _sub2(point, point_a), _sub2(point_c, point_a)
    ) / denominator
    weight_c = _cross2(
        _sub2(point_b, point_a), _sub2(point, point_a)
    ) / denominator
    weight_a = 1.0 - weight_b - weight_c
    anchors = tuple(crop.uv_anchors[index] for index in anchor_indices)
    u_value = sum(
        weight * uv[0]
        for weight, uv in zip(
            (weight_a, weight_b, weight_c), anchors
        )
    )
    v_value = crop.v_origin + sum(
        weight * uv[1]
        for weight, uv in zip(
            (weight_a, weight_b, weight_c), anchors
        )
    )
    return u_value, v_value


def _corner_crop_polygon(
    surface, corner, policy, alpha, settings, diagnostics=None
):
    """Runtime endpoint extrusion polygon выбранной corner policy."""

    point = corner.point
    if len(corner.incident_sites) != 2:
        settings = _normalized_corner_runtime_settings(settings)
        if settings.dynamic_corner_bands:
            return []
        return [
            (point[0] - alpha, point[1] - alpha),
            (point[0] + alpha, point[1] - alpha),
            (point[0] + alpha, point[1] + alpha),
            (point[0] - alpha, point[1] + alpha),
        ]
    if policy == _CornerPolicy.KITE:
        return _kite_crop_polygon(
            surface, corner, alpha, settings, diagnostics
        )

    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return []
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    points = [point, offset_lines[0][0], offset_lines[1][0]]
    if (
        intersection is not None
        and _dist2(point, intersection) <= alpha * settings.apex_limit
    ):
        points.append(intersection)
    elif intersection is not None and diagnostics is not None:
        diagnostics.clamped_miter_count += 1
    return _convex_hull(points)


def _edge_points(
    diagram,
    edges,
    vertices,
    edge_index,
    curve_step,
    diagram_transform=None,
):
    edge = edges[edge_index]
    if edge.start < 0 or edge.end < 0:
        return None
    start = (vertices[edge.start].X, vertices[edge.start].Y)
    end = (vertices[edge.end].X, vertices[edge.end].Y)
    if edge.is_linear:
        return [start, end]
    try:
        points = [
            (
                diagram_transform.from_diagram(point)
                if diagram_transform is not None
                else (float(point[0]), float(point[1]))
            )
            for point in diagram.DiscretizeCurvedEdge(
                edge_index, curve_step, 0.0001
            )
        ]
    except (
        pyvoronoi.FocusOnDirectixException,
        pyvoronoi.UnsolvableParabolaEquation,
    ):
        # Boost иногда помечает edge как параболический, хотя один из его
        # концов не лежит на параболе point/segment sites. Увеличение tolerance
        # в таком случае строит выдуманную дугу с разрывом у endpoint. Общая
        # хорда twin edges сохраняет замкнутую partition и локализует дефект
        # внешней библиотеки одним низкополигональным ребром.
        return [start, end]
    if not points:
        return [start, end]
    if _dist2(points[0], start) > _dist2(points[-1], start):
        points.reverse()
    points[0] = start
    points[-1] = end
    return points


def _cell_polygon(
    diagram,
    edges,
    vertices,
    cell,
    curve_step,
    diagnostics=None,
    diagram_transform=None,
):
    """Восстанавливает ordered boundary конечной pyvoronoi cell."""

    polygon = []
    for edge_index in cell.edges:
        edge_points = _edge_points(
            diagram,
            edges,
            vertices,
            edge_index,
            curve_step,
            diagram_transform,
        )
        if edge_points is None:
            return None
        if not polygon:
            polygon.extend(edge_points)
            continue
        if _dist2(polygon[-1], edge_points[0]) <= 1e-7:
            polygon.extend(edge_points[1:])
        elif _dist2(polygon[-1], edge_points[-1]) <= 1e-7:
            polygon.extend(reversed(edge_points[:-1]))
        else:
            # Cell.edges обычно уже циклически упорядочены. Этот fallback
            # сохраняет контур при небольшой ошибке округления backend.
            if diagnostics is not None:
                diagnostics.cell_disorder_fallbacks += 1
            polygon.extend(edge_points)
    polygon = _dedupe_polygon(polygon)
    if len(polygon) < 3 or abs(_polygon_area2(polygon)) <= 1e-12:
        return None
    if _polygon_area2(polygon) < 0.0:
        polygon.reverse()
    return polygon


def _triangulate_cell_polygon(points, diagram_quantum=None):
    """Разбивает Voronoi-cell до clipping, сохраняя convex atoms."""

    polygon = _dedupe_polygon(points)
    if len(polygon) < 3 or not _polygon_is_simple(polygon):
        return []
    if _polygon_area2(polygon) < 0.0:
        polygon.reverse()
    if _polygon_is_convex(polygon):
        return [polygon]

    remaining = list(range(len(polygon)))
    triangles = []
    guard = len(polygon) * len(polygon)
    while len(remaining) > 3 and guard > 0:
        ear_found = False
        for position, current_index in enumerate(remaining):
            previous_index = remaining[position - 1]
            next_index = remaining[(position + 1) % len(remaining)]
            triangle = [
                polygon[previous_index],
                polygon[current_index],
                polygon[next_index],
            ]
            if _polygon_area2(triangle) <= 1e-12:
                continue
            has_inner_point = False
            for candidate_index in remaining:
                if candidate_index in (
                    previous_index,
                    current_index,
                    next_index,
                ):
                    continue
                candidate = polygon[candidate_index]
                turns = []
                for edge_index in range(3):
                    point_a = triangle[edge_index]
                    point_b = triangle[(edge_index + 1) % 3]
                    turns.append(
                        (point_b[0] - point_a[0])
                        * (candidate[1] - point_a[1])
                        - (point_b[1] - point_a[1])
                        * (candidate[0] - point_a[0])
                    )
                if min(turns) > 1e-10:
                    has_inner_point = True
                    break
            if has_inner_point:
                continue
            triangles.append(triangle)
            del remaining[position]
            ear_found = True
            break
        if not ear_found:
            remaining_polygon = [polygon[index] for index in remaining]
            quantum = (
                DECAL_WELD_DISTANCE * 0.1
                if diagram_quantum is None
                else diagram_quantum
            )
            if abs(_polygon_area2(remaining_polygon)) <= max(
                1e-10,
                quantum * quantum,
            ):
                # Все ненулевые ears уже покрыли исходную cell; Boost может
                # оставить после них только коллинеарную zero-area цепочку.
                # Это успешная triangulation, а не повод выбрасывать ранее
                # собранные triangles и отклонять весь component.
                return triangles
            return []
        guard -= 1
    if len(remaining) == 3:
        triangle = [polygon[index] for index in remaining]
        if _polygon_area2(triangle) > 1e-12:
            triangles.append(triangle)
    return triangles


def _project(point, origin, basis_u, basis_v):
    delta = point - origin
    return (delta.dot(basis_u), delta.dot(basis_v))


def _build_diagram_transform(points, edge_indices=()):
    """Подбирает precision, не выводя centred input из Boost int range."""

    try:
        return build_diagram_transform(points)
    except DiagramTransformError as exc:
        raise _PatchVoronoiSurfaceCompileError(
            "DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED",
            edge_indices,
            exc.details,
        ) from exc


def _patch_is_planar(node):
    if not node.mesh_verts or not node.mesh_tris:
        return False
    normal = node.normal.normalized()
    origin = node.centroid
    projected = [_project(point, origin, node.basis_u, node.basis_v) for point in node.mesh_verts]
    if not projected:
        return False
    min_x = min(point[0] for point in projected)
    max_x = max(point[0] for point in projected)
    min_y = min(point[1] for point in projected)
    max_y = max(point[1] for point in projected)
    extent = max(max_x - min_x, max_y - min_y, 1.0)
    tolerance = max(DECAL_WELD_DISTANCE * 0.25, extent * 1e-5)
    return all(abs((point - origin).dot(normal)) <= tolerance for point in node.mesh_verts)


def _canonical_plane_key(normal, point):
    """Tolerance-stable identity одной geometric plane."""

    plane_normal = normal.normalized()
    dominant_axis = max(
        range(3), key=lambda axis: (abs(plane_normal[axis]), -axis)
    )
    if plane_normal[dominant_axis] > 0.0:
        plane_normal = plane_normal * -1.0
    normal_quantum = 1e-5
    distance_quantum = max(DECAL_WELD_DISTANCE * 0.25, 1e-6)
    return (
        round(plane_normal.x / normal_quantum),
        round(plane_normal.y / normal_quantum),
        round(plane_normal.z / normal_quantum),
        round(plane_normal.dot(point) / distance_quantum),
    )


def _planar_owner_surfaces(node, raw_sites):
    """Делит непланарный patch по реальным owner-face planes."""

    tri_face_indices = getattr(node, "mesh_tri_face_indices", ())
    tri_face_normals = getattr(node, "mesh_tri_face_normals", ())
    if (
        len(tri_face_indices) == len(node.mesh_tris)
        and len(tri_face_normals) == len(node.mesh_tris)
    ):
        return _provenance_owner_surfaces(node, raw_sites)

    triangles_by_plane = {}
    for triangle in node.mesh_tris:
        point_a, point_b, point_c = (
            node.mesh_verts[index] for index in triangle
        )
        triangle_normal = (point_b - point_a).cross(point_c - point_a)
        if triangle_normal.length_squared <= _GEOMETRY_EPS:
            continue
        triangle_normal.normalize()
        key = _canonical_plane_key(triangle_normal, point_a)
        triangles_by_plane.setdefault(key, []).append(tuple(triangle))

    sites_by_plane = {}
    for raw in raw_sites:
        side_normal = raw.get("side_normal", node.normal)
        if side_normal.length_squared <= _GEOMETRY_EPS:
            side_normal = node.normal
        key = _canonical_plane_key(side_normal, raw["source_a"])
        sites_by_plane.setdefault(key, []).append(raw)

    surfaces = []
    for group_index, key in enumerate(sorted(sites_by_plane)):
        triangles = triangles_by_plane.get(key, ())
        if not triangles:
            return ()
        group_sites = sites_by_plane[key]
        fallback_face_id = int(
            group_sites[0].get("owner_face_index", node.patch_id)
        )
        normal = group_sites[0].get("side_normal", node.normal).normalized()
        used_indices = sorted({index for tri in triangles for index in tri})
        centroid = sum(
            (node.mesh_verts[index] for index in used_indices),
            Vector((0.0, 0.0, 0.0)),
        ) / max(len(used_indices), 1)
        surface_id = -(int(node.patch_id) * 1000 + group_index + 1)
        surfaces.append(
            (
                _PlanarOwnerSurface(
                    patch_id=surface_id,
                    centroid=centroid,
                    normal=normal,
                    basis_u=node.basis_u,
                    basis_v=node.basis_v,
                    boundary_loops=(),
                    mesh_verts=tuple(node.mesh_verts),
                    mesh_tris=tuple(triangles),
                    mesh_tri_face_indices=(fallback_face_id,) * len(triangles),
                ),
                group_sites,
            )
        )
    return tuple(surfaces)


def _provenance_owner_surfaces(node, raw_sites):
    """Строит owner surfaces по source-face, а не по triangle fan normal.

    Blender хранит нормаль polygon отдельно от неявной tessellation. У слегка
    непланарного quad/ngon fan-треугольники имеют другие плоскости, поэтому
    точное сопоставление по их normals теряло весь patch. Сначала восстанавливаем
    исходные face-группы, затем объединяем только действительно coplanar faces.
    """

    triangles_by_face = {}
    normals_by_face = {
        int(face_id): Vector(normal)
        for face_id, normal in node.source_face_normals
    }
    for triangle, face_index in zip(
        node.mesh_tris,
        node.mesh_tri_face_indices,
    ):
        face_index = int(face_index)
        triangles_by_face.setdefault(face_index, []).append(tuple(triangle))

    group_by_face = {}
    triangles_by_group = {}
    triangle_faces_by_group = {}
    normals_by_group = {}
    for face_index in sorted(triangles_by_face):
        triangles = triangles_by_face[face_index]
        normal = normals_by_face.get(face_index)
        if normal is None:
            continue
        used_indices = sorted({index for tri in triangles for index in tri})
        if not used_indices:
            continue
        centroid = sum(
            (node.mesh_verts[index] for index in used_indices),
            Vector((0.0, 0.0, 0.0)),
        ) / len(used_indices)
        extent = max(
            ((node.mesh_verts[index] - centroid).length for index in used_indices),
            default=0.0,
        )
        planarity_tolerance = max(
            DECAL_WELD_DISTANCE * 0.25,
            extent * 1e-5,
            1e-6,
        )
        max_residual = max(
            abs((node.mesh_verts[index] - centroid).dot(normal))
            for index in used_indices
        )
        # Непланарный polygon остаётся отдельным tangent owner surface.
        # Планарные соседние faces можно безопасно объединить в общий chart.
        group_key = (
            ("FACE", face_index)
            if max_residual > planarity_tolerance
            else ("PLANE",) + _canonical_plane_key(normal, centroid)
        )
        group_by_face[face_index] = group_key
        triangles_by_group.setdefault(group_key, []).extend(triangles)
        triangle_faces_by_group.setdefault(group_key, []).extend(
            (face_index,) * len(triangles)
        )
        normals_by_group.setdefault(group_key, normal)

    sites_by_group = {}
    for raw in raw_sites:
        group_key = group_by_face.get(int(raw.get("owner_face_index", -1)))
        if group_key is None:
            return ()
        sites_by_group.setdefault(group_key, []).append(raw)

    surfaces = []
    for group_index, group_key in enumerate(sorted(sites_by_group, key=repr)):
        triangles = triangles_by_group.get(group_key, ())
        if not triangles:
            return ()
        used_indices = sorted({index for tri in triangles for index in tri})
        centroid = sum(
            (node.mesh_verts[index] for index in used_indices),
            Vector((0.0, 0.0, 0.0)),
        ) / max(len(used_indices), 1)
        surface_id = -(int(node.patch_id) * 1000 + group_index + 1)
        surfaces.append(
            (
                _PlanarOwnerSurface(
                    patch_id=surface_id,
                    centroid=centroid,
                    normal=normals_by_group[group_key].copy(),
                    basis_u=node.basis_u,
                    basis_v=node.basis_v,
                    boundary_loops=(),
                    mesh_verts=tuple(node.mesh_verts),
                    mesh_tris=tuple(triangles),
                    mesh_tri_face_indices=tuple(
                        triangle_faces_by_group[group_key]
                    ),
                ),
                sites_by_group[group_key],
            )
        )
    return tuple(surfaces)


def _assign_network_v_phases(raw_by_patch):
    """Даёт connected selected network одну накопленную deterministic V-фазу."""

    physical_edges = {}
    for patch_sites in raw_by_patch.values():
        for raw in patch_sites:
            physical_edges.setdefault(raw["edge_index"], raw)
    adjacency = {}
    for raw in physical_edges.values():
        vert_a = raw["vert_a"]
        vert_b = raw["vert_b"]
        length = float(raw["segment_length"])
        adjacency.setdefault(vert_a, []).append((vert_b, length))
        adjacency.setdefault(vert_b, []).append((vert_a, length))

    distances = {}
    remaining = set(adjacency)
    while remaining:
        root = min(remaining)
        distances[root] = 0.0
        queue = [(0.0, root)]
        while queue:
            distance, vert_index = heappop(queue)
            if distance > distances.get(vert_index, float("inf")) + 1e-12:
                continue
            remaining.discard(vert_index)
            for neighbour, edge_length in adjacency.get(vert_index, ()):
                candidate = distance + edge_length
                if candidate + 1e-12 < distances.get(
                    neighbour, float("inf")
                ):
                    distances[neighbour] = candidate
                    heappush(queue, (candidate, neighbour))

    for patch_sites in raw_by_patch.values():
        for raw in patch_sites:
            phase_a = distances.get(raw["vert_a"], 0.0)
            phase_b = distances.get(raw["vert_b"], phase_a)
            raw["arc_start"] = phase_a
            raw["arc_sign"] = 1.0 if phase_b >= phase_a else -1.0


def _collect_patch_sites(graph, selected_edges):
    """Сохраняет patch/chain identity вместо сведения сети к голым runs."""

    raw_by_patch = {}
    uses_by_edge = {}
    normals_by_vert = {}
    positions_by_vert = {}
    chart_use_counts = {}
    for patch_id in sorted(graph.nodes):
        node = graph.nodes[patch_id]
        seen_edges = set()
        patch_sites = []
        for boundary_loop in node.boundary_loops:
            for chain in boundary_loop.chains:
                if len(chain.vert_cos) < 2:
                    continue
                arc = 0.0
                segment_count = len(chain.edge_indices)
                for segment_index in range(segment_count):
                    next_index = segment_index + 1
                    if next_index >= len(chain.vert_cos):
                        if not chain.is_closed:
                            continue
                        next_index = 0
                    source_a = chain.vert_cos[segment_index].copy()
                    source_b = chain.vert_cos[next_index].copy()
                    length = (source_b - source_a).length
                    edge_index = int(chain.edge_indices[segment_index])
                    if edge_index in selected_edges:
                        key = (patch_id, edge_index)
                        chart_use_counts[key] = chart_use_counts.get(key, 0) + 1
                    if (
                        edge_index in selected_edges
                        and edge_index not in seen_edges
                        and length > _GEOMETRY_EPS
                    ):
                        vert_a = int(chain.vert_indices[segment_index])
                        vert_b = int(chain.vert_indices[next_index])
                        raw = {
                            "patch_id": patch_id,
                            "edge_index": edge_index,
                            "vert_a": vert_a,
                            "vert_b": vert_b,
                            "source_a": source_a,
                            "source_b": source_b,
                            "arc_start": arc,
                            "segment_length": length,
                            "side_normal": (
                                chain.side_face_normals[segment_index].copy()
                                if segment_index < len(chain.side_face_normals)
                                else node.normal.copy()
                            ),
                            "owner_face_index": (
                                int(chain.side_face_indices[segment_index])
                                if segment_index < len(chain.side_face_indices)
                                else -1
                            ),
                        }
                        patch_sites.append(raw)
                        uses_by_edge.setdefault(edge_index, []).append(raw)
                        for vert_index, position in (
                            (vert_a, source_a),
                            (vert_b, source_b),
                        ):
                            positions_by_vert.setdefault(vert_index, position.copy())
                            normals_by_vert.setdefault(vert_index, []).append(
                                raw["side_normal"].copy()
                            )
                        seen_edges.add(edge_index)
                    arc += length
        if patch_sites:
            raw_by_patch[patch_id] = patch_sites

    for edge_index, uses in uses_by_edge.items():
        for use_index, raw in enumerate(
            sorted(uses, key=lambda item: (item["patch_id"], item["vert_a"], item["vert_b"]))
        ):
            raw["uv_sign"] = -1.0 if use_index % 2 == 0 else 1.0
            raw["two_sided"] = (
                chart_use_counts.get((raw["patch_id"], edge_index), 0) > 1
            )
    _assign_network_v_phases(raw_by_patch)
    return raw_by_patch, normals_by_vert, positions_by_vert


def _single_use_internal_seam_failures(graph, selected_edges):
    """Локализует внутренний edge без второй owner-side до compile."""

    uses_by_edge = {int(edge_index): [] for edge_index in selected_edges}
    for patch_id in sorted(graph.nodes):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                for segment_index, edge_index in enumerate(chain.edge_indices):
                    edge_index = int(edge_index)
                    if edge_index not in uses_by_edge:
                        continue
                    uses_by_edge[edge_index].append(
                        (
                            int(patch_id),
                            int(loop_index),
                            int(chain_index),
                            int(segment_index),
                            int(chain.neighbor_patch_id),
                        )
                    )

    failures = []
    for edge_index in sorted(uses_by_edge):
        uses = tuple(sorted(uses_by_edge[edge_index]))
        if len(uses) != 1 or uses[0][4] == -1:
            continue
        failures.append(
            PatchVoronoiCompileFailure(
                patch_id=uses[0][0],
                reason="SINGLE_USE_INTERNAL_SEAM",
                edge_indices=(edge_index,),
                details=f"use_ref={uses[0]!r}",
            )
        )
    return tuple(failures)


def _corner_site_view_from_sites(sites, corner, site_index):
    """Локальная periodic image incident site для геометрии corner."""

    site = sites[site_index]
    offset = dict(getattr(corner, "site_u_offsets", ())).get(
        site_index, 0.0
    )
    if abs(offset) <= _GEOMETRY_EPS:
        return site
    return replace(
        site,
        point_a=(site.point_a[0] + offset, site.point_a[1]),
        point_b=(site.point_b[0] + offset, site.point_b[1]),
    )


def _corner_site_view(surface, corner, site_index):
    return _corner_site_view_from_sites(surface.sites, corner, site_index)


def _compile_corners(sites, periodic_axis="", period=0.0):
    incidents = {}
    points = {}
    for site_index, site in enumerate(sites):
        for vert_index, point in (
            (site.vert_a, site.point_a),
            (site.vert_b, site.point_b),
        ):
            incidents.setdefault(vert_index, []).append(site_index)
            points.setdefault(vert_index, point)

    corners = []
    for vert_index in sorted(incidents):
        incident_sites = tuple(sorted(incidents[vert_index]))
        site_u_offsets = ()
        point = points[vert_index]
        if periodic_axis == "U" and period > 0.0:
            endpoint_u = {}
            for site_index in incident_sites:
                site = sites[site_index]
                endpoint = (
                    site.point_a
                    if site.vert_a == vert_index
                    else site.point_b
                )
                endpoint_u[site_index] = endpoint[0]
            anchor_u = min(endpoint_u.values())
            site_u_offsets = tuple(
                (
                    site_index,
                    round((anchor_u - endpoint_u[site_index]) / period)
                    * period,
                )
                for site_index in incident_sites
            )
            first_site = sites[incident_sites[0]]
            first_endpoint = (
                first_site.point_a
                if first_site.vert_a == vert_index
                else first_site.point_b
            )
            point = (
                first_endpoint[0] + dict(site_u_offsets)[incident_sites[0]],
                first_endpoint[1],
            )
        corner_view = CornerSpec(
            vert_index=vert_index,
            point=point,
            incident_sites=incident_sites,
            ordered_sites=incident_sites,
            turn_sign=0.0,
            interior_angle=0.0,
            extrusion_angle=0.0,
            is_convex=False,
            miter_ratio=float("inf"),
            site_u_offsets=site_u_offsets,
        )
        ordered_sites = incident_sites
        turn_sign = 0.0
        interior_angle = 0.0
        extrusion_angle = 0.0
        is_convex = False
        miter_ratio = float("inf")
        if len(incident_sites) == 2:
            rays = {}
            incoming = []
            outgoing = []
            for site_index in incident_sites:
                site = _corner_site_view_from_sites(
                    sites, corner_view, site_index
                )
                if site.vert_a == vert_index:
                    other = site.point_b
                    outgoing.append(site_index)
                else:
                    other = site.point_a
                    incoming.append(site_index)
                rays[site_index] = _norm2(_sub2(other, point))
            if (
                len(incoming) == 1
                and len(outgoing) == 1
                and incoming[0] != outgoing[0]
            ):
                ordered_sites = (incoming[0], outgoing[0])
                incoming_ray = rays[incoming[0]]
                outgoing_ray = rays[outgoing[0]]
                if incoming_ray is not None and outgoing_ray is not None:
                    travel_in = (-incoming_ray[0], -incoming_ray[1])
                    signed_turn = _cross2(travel_in, outgoing_ray)
                    if abs(signed_turn) > 1e-9:
                        turn_sign = 1.0 if signed_turn > 0.0 else -1.0
            else:
                ordered_sites = tuple(
                    sorted(
                        incident_sites,
                        key=lambda site_index: atan2(
                            rays[site_index][1], rays[site_index][0]
                        )
                        if rays[site_index] is not None
                        else 0.0,
                    )
                )
                first_ray = rays[ordered_sites[0]]
                second_ray = rays[ordered_sites[1]]
                if first_ray is not None and second_ray is not None:
                    signed_turn = _cross2(first_ray, second_ray)
                    if abs(signed_turn) > 1e-9:
                        turn_sign = 1.0 if signed_turn > 0.0 else -1.0

            first_ray = rays[incident_sites[0]]
            second_ray = rays[incident_sites[1]]
            if first_ray is not None and second_ray is not None:
                minor_angle = atan2(
                    abs(_cross2(first_ray, second_ray)),
                    max(-1.0, min(1.0, _dot2(first_ray, second_ray))),
                )
                bisector = _norm2(
                    (
                        first_ray[0] + second_ray[0],
                        first_ray[1] + second_ray[1],
                    )
                )
                if bisector is None:
                    interior_angle = pi
                    is_convex = False
                else:
                    inside_small_wedge = all(
                        _dot2(
                            bisector,
                            _corner_site_view_from_sites(
                                sites, corner_view, site_index
                            ).inward_normal,
                        )
                        >= -1e-7
                        for site_index in incident_sites
                    )
                    is_convex = inside_small_wedge and minor_angle < pi - 1e-7
                    interior_angle = (
                        minor_angle if inside_small_wedge else tau - minor_angle
                    )
                    extrusion_angle = (
                        tau - interior_angle
                        if interior_angle > pi
                        else interior_angle
                    )

            offset_lines = []
            for site_index in incident_sites:
                site = _corner_site_view_from_sites(
                    sites, corner_view, site_index
                )
                other = site.point_b if site.vert_a == vert_index else site.point_a
                direction = (
                    (other[0] - point[0]) / site.segment_length,
                    (other[1] - point[1]) / site.segment_length,
                )
                offset_lines.append(
                    (
                        (
                            point[0] + site.inward_normal[0],
                            point[1] + site.inward_normal[1],
                        ),
                        direction,
                    )
                )
            intersection = _line_intersection(
                offset_lines[0][0],
                offset_lines[0][1],
                offset_lines[1][0],
                offset_lines[1][1],
            )
            if intersection is not None:
                miter_ratio = _dist2(point, intersection)
        elif len(incident_sites) > 2:
            rays = {}
            for site_index in incident_sites:
                site = _corner_site_view_from_sites(
                    sites, corner_view, site_index
                )
                other = (
                    site.point_b
                    if site.vert_a == vert_index
                    else site.point_a
                )
                rays[site_index] = _norm2(_sub2(other, point))
            ordered_sites = tuple(
                sorted(
                    incident_sites,
                    key=lambda site_index: (
                        atan2(
                            rays[site_index][1], rays[site_index][0]
                        )
                        if rays[site_index] is not None
                        else 0.0,
                        site_index,
                    ),
                )
            )
        corners.append(
            CornerSpec(
                vert_index=vert_index,
                point=point,
                incident_sites=incident_sites,
                ordered_sites=ordered_sites,
                turn_sign=turn_sign,
                interior_angle=interior_angle,
                extrusion_angle=extrusion_angle,
                is_convex=is_convex,
                miter_ratio=miter_ratio,
                site_u_offsets=site_u_offsets,
            )
        )
    return tuple(corners)


def _corner_wedge_coordinates(sites, corner, point):
    """Коэффициенты point в cone двух endpoint perpendicular rays."""

    if len(corner.ordered_sites) != 2:
        return None
    normal_a = sites[corner.ordered_sites[0]].inward_normal
    normal_b = sites[corner.ordered_sites[1]].inward_normal
    determinant = _cross2(normal_a, normal_b)
    if abs(determinant) <= _GEOMETRY_EPS:
        return None
    delta = _sub2(point, corner.point)
    return (
        _cross2(delta, normal_b) / determinant,
        _cross2(normal_a, delta) / determinant,
    )


def _compile_corner_split_chord(
    sites,
    corner,
    diagram_vertices,
    triangles,
    triangle_grid=None,
    reference=False,
):
    """Статичная B3-хорда, якорённая к первой Voronoi-вершине клина."""

    if len(corner.ordered_sites) != 2:
        return ()
    normal_a = sites[corner.ordered_sites[0]].inward_normal
    normal_b = sites[corner.ordered_sites[1]].inward_normal
    bisector = _norm2(
        (normal_a[0] + normal_b[0], normal_a[1] + normal_b[1])
    )
    if bisector is None:
        return ()
    tolerance = max(DECAL_WELD_DISTANCE * 0.25, 1e-8)
    candidates = []
    for vertex in diagram_vertices:
        point = (float(vertex.X), float(vertex.Y))
        if not all(isfinite(value) for value in point):
            # R1 image-only endpoint cells могут граничить с бесконечным
            # Voronoi-ребром; такая служебная вершина не является B3-якорем.
            continue
        coordinates = _corner_wedge_coordinates(sites, corner, point)
        if coordinates is None or min(coordinates) < -tolerance:
            continue
        distance = _dist2(corner.point, point)
        if distance <= tolerance or not _point_in_domain(
            point, triangles, triangle_grid, reference
        ):
            continue
        candidates.append((distance, point))

    if candidates:
        _distance, anchor = min(candidates)
    else:
        # Domain-boundary fallback oracle B3. Берём самую дальнюю domain
        # station вдоль bisector, чтобы fallback не вводил произвольную
        # runtime-дистанцию и гарантированно оставался compiled-фактом.
        projected = [
            (_dot2(_sub2(point, corner.point), bisector), point)
            for triangle in triangles
            for point in triangle
        ]
        projected = [entry for entry in projected if entry[0] > tolerance]
        if not projected:
            return ()
        _distance, anchor = max(projected)

    chord_direction = (-bisector[1], bisector[0])
    chord_a = _line_intersection(
        corner.point, normal_a, anchor, chord_direction
    )
    chord_b = _line_intersection(
        corner.point, normal_b, anchor, chord_direction
    )
    if chord_a is None or chord_b is None:
        return ()
    coordinates_a = _corner_wedge_coordinates(sites, corner, chord_a)
    coordinates_b = _corner_wedge_coordinates(sites, corner, chord_b)
    if (
        coordinates_a is None
        or coordinates_b is None
        or min(coordinates_a) < -tolerance
        or min(coordinates_b) < -tolerance
    ):
        return ()
    return (chord_a, chord_b)


def _compile_corner_static_wedge(sites, corner, triangles):
    """Конечное представление compiled endpoint cone вне patch domain."""

    if len(corner.ordered_sites) != 2:
        return ()
    extent = max(
        (
            _dist2(corner.point, point)
            for triangle in triangles
            for point in triangle
        ),
        default=0.0,
    )
    extent = max(extent * 2.0, DECAL_WELD_DISTANCE * 4.0)
    points = [corner.point]
    for site_index in corner.ordered_sites:
        normal = sites[site_index].inward_normal
        points.append(
            (
                corner.point[0] + normal[0] * extent,
                corner.point[1] + normal[1] * extent,
            )
        )
    return tuple(_convex_hull(points))


def _classify_extrusion_angle(extrusion_angle, settings):
    """Ordered A11 band classifier; exact threshold chooses softer band."""

    theta = max(0.0, min(pi, float(extrusion_angle)))
    if theta + _ANGLE_CLASSIFICATION_EPS >= settings.miter_angle:
        return _CornerPolicy.MITER
    if theta + _ANGLE_CLASSIFICATION_EPS >= settings.kite_angle:
        return _CornerPolicy.KITE
    if theta + _ANGLE_CLASSIFICATION_EPS >= settings.split_angle:
        return _CornerPolicy.FAN
    if theta + _ANGLE_CLASSIFICATION_EPS >= settings.hairpin_angle:
        return _CornerPolicy.ACUTE_SPLIT
    return _CornerPolicy.HAIRPIN


def classify_corner_runtime(corner, settings=None):
    """Выбирает базовую corner policy из compiled angle facts.

    Функция не читает PyVoronoi и не меняет plan. Поэтому thresholds и
    apex limit можно менять между preview frames без перекомпиляции sites.
    Join-style здесь намеренно не применяется: GAP/OVERLAP требует двух
    incident strip-квадов и принадлежит surface-aware selector ниже.
    """

    settings = _normalized_corner_runtime_settings(settings)
    incident_count = len(corner.incident_sites)
    if incident_count == 1:
        return _CornerPolicy.CAP
    if incident_count != 2:
        return _CornerPolicy.JUNCTION

    if abs(corner.interior_angle - pi) <= 1e-7:
        policy = _CornerPolicy.MITER
    elif (
        not settings.dynamic_corner_bands
        and pi - corner.extrusion_angle
        <= _SMOOTH_TURN_ANGLE + _ANGLE_CLASSIFICATION_EPS
    ):
        policy = _CornerPolicy.SMOOTH
    elif not settings.dynamic_corner_bands:
        if corner.extrusion_angle < settings.split_angle:
            policy = _CornerPolicy.ACUTE_SPLIT
        elif corner.is_convex:
            policy = _CornerPolicy.MITER
        elif corner.interior_angle > pi:
            policy = _CornerPolicy.KITE
        else:
            policy = _CornerPolicy.MITER
    else:
        policy = _classify_extrusion_angle(
            corner.extrusion_angle, settings
        )
    return policy


def _classify_surface_corner_runtime(surface, corner, settings=None):
    """Геометрическая band-policy без join-семантики CornerModel."""

    settings = _normalized_corner_runtime_settings(settings)
    policy = classify_corner_runtime(corner, settings)
    if (
        policy == _CornerPolicy.SMOOTH
        and surface.domain.admission_tier == "APPROXIMATE"
    ):
        policy = _CornerPolicy.MITER
    return policy


def _canonical_planar_basis(normal):
    """Детерминированный chart, одинаковый для ``normal`` и ``-normal``.

    Знак выбирается так, чтобы сохранить историческую ориентацию основных
    architectural planes (для X-plane это ``-X/-Y/+Z``). Это важно для
    PyVoronoi: одинаковый контур standalone и внутри extruded mesh должен
    получать те же integer sites и topology events.
    """

    chart_normal = normal.normalized()
    dominant_axis = max(
        range(3), key=lambda axis: (abs(chart_normal[axis]), -axis)
    )
    if chart_normal[dominant_axis] > 0.0:
        chart_normal = chart_normal * -1.0
    world_axes = (
        Vector((1.0, 0.0, 0.0)),
        Vector((0.0, 1.0, 0.0)),
        Vector((0.0, 0.0, 1.0)),
    )
    anchor = world_axes[(dominant_axis + 1) % 3] * -1.0
    basis_u = anchor - chart_normal * anchor.dot(chart_normal)
    basis_u.normalize()
    basis_v = chart_normal.cross(basis_u).normalized()
    return basis_u, basis_v


def _compile_surface_relations(sites, corners, atoms):
    """Строит width-independent relation indices одного surface."""

    atoms_by_site = {}
    for atom_index, atom in enumerate(atoms):
        atoms_by_site.setdefault(atom.site_index, []).append(atom_index)

    corners_by_site = {}
    for corner_index, corner in enumerate(corners):
        for site_index in corner.incident_sites:
            corners_by_site.setdefault(site_index, []).append(corner_index)

    sites_by_vertex = {}
    ports_by_vertex = {}
    ports_by_site = {}
    for site_index, site in enumerate(sites):
        sites_by_vertex.setdefault(site.vert_a, []).append(site_index)
        sites_by_vertex.setdefault(site.vert_b, []).append(site_index)
        for vert_index, point, source_category in (
            (site.vert_a, site.point_a, 1),
            (site.vert_b, site.point_b, 2),
        ):
            port = _CompiledSitePort(
                site_index=site_index,
                edge_index=site.edge_index,
                vert_index=vert_index,
                point=point,
                source_category=source_category,
            )
            ports_by_vertex.setdefault(vert_index, []).append(port)
            ports_by_site.setdefault(site_index, []).append(port)

    owner_atoms_by_corner = {}
    for corner_index, corner in enumerate(corners):
        incident_sites = set(corner.incident_sites)
        owner_atoms_by_corner[corner_index] = tuple(
            atom_index
            for atom_index, atom in enumerate(atoms)
            if (
                atom.cell_kind == "SEGMENT"
                and atom.site_index in incident_sites
            )
            or (
                atom.cell_kind == "POINT"
                and atom.corner_index == corner_index
            )
        )

    def frozen(mapping):
        return {
            key: tuple(values)
            for key, values in sorted(mapping.items())
        }

    return (
        frozen(atoms_by_site),
        frozen(owner_atoms_by_corner),
        frozen(corners_by_site),
        frozen(sites_by_vertex),
        frozen(ports_by_vertex),
        frozen(ports_by_site),
    )


def _corner_route_id(surface, site):
    return (
        "patch-voronoi-route",
        int(surface.patch_id),
        int(site.edge_index),
    )


def _corner_station_key(surface, site, point):
    quantum = max(float(surface.diagram_transform.quantum), 1e-10)
    parameter = _site_unbounded_parameter(site, point)
    tangent = _norm2(_sub2(site.point_b, site.point_a))
    lateral = (
        0.0
        if tangent is None
        else _cross2(tangent, _sub2(point, site.point_a))
    )
    return (
        "patch-voronoi-station",
        int(surface.patch_id),
        int(site.edge_index),
        round(_site_v_length(site, parameter) / quantum),
        round(lateral / quantum),
    )


def _corner_point_source_face_id(surface, location, fallback):
    if (
        surface.domain.kind == "PLANAR"
        and location is not None
        and 0 <= int(location.triangle_id)
        < len(surface.domain.boundary_triangle_source_face_ids)
    ):
        return surface.domain.boundary_triangle_source_face_ids[
            int(location.triangle_id)
        ]
    if (
        surface.domain.kind == "INTRINSIC"
        and location is not None
        and 0 <= int(location.triangle_id) < len(surface.domain.intrinsic_triangles)
    ):
        source_face_id = surface.domain.intrinsic_triangles[
            int(location.triangle_id)
        ].source_face_id
        if source_face_id is not None:
            return source_face_id
    return int(fallback)


def _corner_seed_records(surface, corner):
    if len(corner.ordered_sites) == 2:
        return ((None, corner),)
    if len(corner.incident_sites) > 2:
        return tuple(_junction_sector_specs(surface, corner))
    return ()


def _compile_corner_seeds(
    surface,
    lifted_vertices,
    requested_join=CornerJoinMode.MITER,
):
    """Фиксирует V/order/owner/join до width-dependent model build."""

    requested_join = CornerJoinMode(requested_join)
    seeds = []
    for corner in surface.corners:
        for sector_id, sector in _corner_seed_records(surface, corner):
            site_indices = tuple(int(value) for value in sector.ordered_sites)
            if len(site_indices) != 2:
                continue
            sites = tuple(surface.sites[index] for index in site_indices)
            route_ids = tuple(
                _corner_route_id(surface, site) for site in sites
            )
            station_keys = tuple(
                _corner_station_key(surface, site, sector.point)
                for site in sites
            )
            source_edge_ids = tuple(
                int(site.edge_index) for site in sites
            )
            owner_face_ids = tuple(
                int(site.owner_face_index)
                for site in sites
                if int(site.owner_face_index) >= 0
            )
            owner_face_id = (
                min(owner_face_ids)
                if owner_face_ids
                else int(surface.patch_id)
            )
            location = surface.domain.locate(sector.point)
            provenance = CornerPointProvenance(
                source_face_id=_corner_point_source_face_id(
                    surface, location, owner_face_id
                ),
                source_edge_ids=source_edge_ids,
                route_ids=route_ids,
                station_keys=station_keys,
                domain_location=location,
            )
            position = lifted_vertices.get(sector.vert_index)
            if position is None:
                position = surface.domain.lift(
                    sector.point, 0.0, location=location
                )
            side = (
                BandSide.NEGATIVE
                if sites[0].uv_sign < 0.0
                else BandSide.POSITIVE
            )
            join = (
                requested_join
                if _corner_offset_edge_relation(surface, sector) == "GAP"
                else CornerJoinMode.MITER
            )
            seeds.append(
                CornerSeed(
                    apex_ref=CornerVertexRef(
                        key=("pv-sv", int(sector.vert_index)),
                        chart_point=tuple(float(value) for value in sector.point),
                        position=tuple(float(value) for value in position),
                        provenance=provenance,
                    ),
                    corner_vertex_id=int(sector.vert_index),
                    incident_site_ids=site_indices,
                    side=side,
                    sector_id=(None if sector_id is None else int(sector_id)),
                    owner_surface_id=int(surface.patch_id),
                    join=join,
                )
            )
    return tuple(seeds)


def _local_corner_strip(surface, corner, site_index, alpha):
    """Клиппит own-strip owner-доменом без Voronoi-конкурентов."""

    site = _corner_site_view(surface, corner, site_index)
    strip_polygon = _segment_crop_polygon(site, alpha)
    period = float(getattr(surface.domain, "period", 0.0))
    site_offset = dict(getattr(corner, "site_u_offsets", ())).get(
        site_index, 0.0
    )
    if period > _GEOMETRY_EPS:
        image_shift = int(round(float(site_offset) / period))
        domain_triangles = tuple(
            tuple(
                (point[0] + shift * period, point[1])
                for point in triangle
            )
            for shift in (image_shift - 1, image_shift, image_shift + 1)
            for triangle in surface.domain.boundary_triangles
        )
    else:
        domain_triangles = surface.domain.boundary_triangles
    fragments = []
    for triangle in domain_triangles:
        clipped = _clip_to_triangle(strip_polygon, triangle)
        if len(clipped) >= 3 and abs(_polygon_area2(clipped)) > 1e-10:
            fragments.append(clipped)
    components = _merge_polygon_fragments(
        fragments,
        tolerance=_FRAGMENT_TOPOLOGY_TOLERANCE,
        normalize_t_junctions=bool(
            surface.domain.normalize_fragment_t_junctions
        ),
    )
    if not components:
        raise RuntimeError(
            "CORNER_LOCAL_STRIP_CLIP_EMPTY: "
            f"surface={surface.patch_id} vertex={corner.vert_index} "
            f"site={site_index}"
        )

    if site.vert_a == corner.vert_index:
        tangent_away = _norm2(_sub2(site.point_b, corner.point))
    else:
        tangent_away = _norm2(_sub2(site.point_a, corner.point))
    if tangent_away is None:
        raise RuntimeError(
            "CORNER_LOCAL_STRIP_TANGENT_INVALID: "
            f"surface={surface.patch_id} vertex={corner.vert_index}"
        )
    target = (
        corner.point[0] + site.inward_normal[0] * alpha,
        corner.point[1] + site.inward_normal[1] * alpha,
    )
    route_id = _corner_route_id(surface, site)
    fallback_face_id = (
        site.owner_face_index
        if site.owner_face_index >= 0
        else surface.patch_id
    )
    vertices_by_key = {}
    target_candidates = []
    cap_candidates = []
    for point in (point for component in components for point in component):
        location = surface.domain.locate(point)
        if location is None and period > _GEOMETRY_EPS:
            lower = float(surface.domain.wrap_origin)
            shift = int((point[0] - lower) // period)
            canonical = (point[0] - shift * period, point[1])
            candidates = (canonical, (canonical[0] - period, canonical[1]))
            location = next(
                (
                    candidate_location
                    for candidate in candidates
                    if (
                        candidate_location := surface.domain.locate(candidate)
                    )
                    is not None
                ),
                None,
            )
        if location is None:
            continue
        parameter = _site_unbounded_parameter(site, point)
        direction = _norm2(_sub2(site.point_b, site.point_a))
        lateral = (
            0.0
            if direction is None
            else _cross2(direction, _sub2(point, site.point_a))
        )
        station_key = _corner_station_key(surface, site, point)
        provenance = CornerPointProvenance(
            source_face_id=_corner_point_source_face_id(
                surface, location, fallback_face_id
            ),
            source_edge_ids=(int(site.edge_index),),
            route_ids=(route_id,),
            station_keys=(station_key,),
            domain_location=location,
        )
        key = _domain_location_key(surface, location)
        vertex = CornerStripVertex(
            key=key,
            chart_point=tuple(float(value) for value in point),
            provenance=provenance,
            station_ref=CornerStationRef(
                route_id=route_id,
                site_id=int(site_index),
                source_edge_id=int(site.edge_index),
                station_key=station_key,
                s=float(_site_v_length(site, parameter)),
                r=float(lateral),
                tangent_away=tuple(float(value) for value in tangent_away),
                source_s_per_chart_unit=(
                    float(site.arc_sign)
                    * (1.0 if site.vert_a == corner.vert_index else -1.0)
                    * (
                        float(site.uv_length)
                        if site.uv_length > 0.0
                        else float(site.segment_length)
                    )
                    / max(float(site.segment_length), _GEOMETRY_EPS)
                ),
            ),
        )
        vertices_by_key.setdefault(key, vertex)
        target_candidates.append((_dist2(point, target), repr(key), key))
        # P — внешний угол именно endpoint-cap собственного strip, а не
        # ближайшая вершина его продольного rail. После насыщения domain
        # cap клиппится на той же прямой через V; выбор rail/domain-угла
        # заставлял P сходить с опорной прямой при width drag.
        cap_line_error = abs(
            _cross2(site.inward_normal, _sub2(point, corner.point))
        )
        if cap_line_error <= _FRAGMENT_TOPOLOGY_TOLERANCE:
            cap_candidates.append((_dist2(point, target), repr(key), key))
    if not target_candidates:
        raise RuntimeError(
            "CORNER_LOCAL_STRIP_PROVENANCE_UNRESOLVED: "
            f"surface={surface.patch_id} vertex={corner.vert_index} "
            f"site={site_index}"
        )
    if not cap_candidates:
        raise RuntimeError(
            "CORNER_LOCAL_STRIP_CAP_VERTEX_UNRESOLVED: "
            f"surface={surface.patch_id} vertex={corner.vert_index} "
            f"site={site_index}"
        )
    _target_distance, _target_repr, outer_key = min(cap_candidates)
    return LocalClippedCornerStrip(
        site_id=int(site_index),
        vertices=tuple(
            vertices_by_key[key]
            for key in sorted(vertices_by_key, key=repr)
        ),
        outer_corner_key=outer_key,
    )


def _build_local_corner_models(
    surface,
    alpha,
    *,
    test_join_override=None,
    test_join_vertex_ids=(),
):
    """S-CM.a: seed -> local own-strip clips -> model, без competition."""

    corners_by_vertex = {
        int(corner.vert_index): corner for corner in surface.corners
    }
    models = []
    override_vertices = frozenset(
        int(value) for value in test_join_vertex_ids
    )
    for compiled_seed in getattr(surface, "corner_seeds", ()):
        seed = (
            compiled_seed
            if test_join_override is None
            or override_vertices
            and compiled_seed.corner_vertex_id not in override_vertices
            else replace(compiled_seed, join=CornerJoinMode(test_join_override))
        )
        corner = corners_by_vertex[seed.corner_vertex_id]
        if seed.sector_id is not None:
            sectors = dict(_junction_sector_specs(surface, corner))
            corner = sectors[seed.sector_id]
        strips = tuple(
            _local_corner_strip(surface, corner, site_index, alpha)
            for site_index in seed.incident_site_ids
        )
        try:
            model = CornerModel.from_local_strips(seed, *strips)
        except ValueError as exc:
            if str(exc) != "CORNER_MODEL_ANCHORS_DEGENERATE":
                raise
            # Локальный owner-domain может схлопнуть оба outer anchors в
            # один fold endpoint. Такой cross-surface угол принадлежит
            # S-CM.b/R3 и не получает ложную планарную форму в S-CM.a.
            continue
        models.append(model)
    return tuple(models)


def _compile_corner_release_atoms(
    *,
    diagram_transform,
    diagram_site_records,
    sites,
    corners,
    diagram_cells,
    diagram_edges,
    guard,
    curve_step,
    point_cell_records,
    diagnostics=None,
):
    """Компилирует конкуренцию за освобождаемую часть point-cell RC5a.

    BEVEL не меняет incident segments собственной ленты. Поэтому из
    конкуренции за клин исключаются оба incident site, а кандидаты берутся
    структурно из границы их Voronoi-региона. Partition не зависит от width
    или join и во время drag только клиппится фактической формой угла.
    """

    if not point_cell_records:
        return {}

    def neighbor_cells(cell_index):
        result = set()
        cell = diagram_cells[cell_index]
        for edge_index in cell.edges:
            if edge_index < 0 or edge_index >= len(diagram_edges):
                continue
            twin_index = diagram_edges[edge_index].twin
            if twin_index < 0 or twin_index >= len(diagram_edges):
                continue
            neighbor_index = diagram_edges[twin_index].cell
            if 0 <= neighbor_index < len(diagram_cells):
                result.add(int(neighbor_index))
        return result

    records_by_corner = {}
    diagram_scale = int(diagram_transform.scale)
    for record in point_cell_records:
        (
            primary_cell_index,
            corner_index,
            point_site_index,
            point_periodic_shift,
            primary_fragments,
            primary_triangle_ids,
        ) = record
        corner = corners[corner_index]
        incident_sites = set(corner.incident_sites)

        queue = [int(primary_cell_index)]
        visited = set()
        candidate_record_indices = set()
        while queue:
            cell_index = queue.pop()
            if cell_index in visited:
                continue
            visited.add(cell_index)
            for neighbor_index in neighbor_cells(cell_index):
                neighbor = diagram_cells[neighbor_index]
                if not (0 <= neighbor.site < len(diagram_site_records)):
                    continue
                owner_site_index = int(
                    diagram_site_records[neighbor.site][0]
                )
                if owner_site_index in incident_sites:
                    queue.append(neighbor_index)
                else:
                    candidate_record_indices.add(int(neighbor.site))

        if not candidate_record_indices:
            continue
        candidate_record_indices = tuple(sorted(candidate_record_indices))

        if len(candidate_record_indices) == 1:
            candidate_index = candidate_record_indices[0]
            owner_site_index, owner_shift = diagram_site_records[
                candidate_index
            ][:2]
            records_by_corner.setdefault(corner_index, []).append(
                _CornerReleaseAtom(
                    corner_index=corner_index,
                    point_site_index=int(point_site_index),
                    point_periodic_shift=int(point_periodic_shift),
                    owner_site_index=int(owner_site_index),
                    owner_periodic_shift=int(owner_shift),
                    fragments=tuple(primary_fragments),
                    fragment_triangle_ids=tuple(primary_triangle_ids),
                )
            )
            continue

        release_diagram = pyvoronoi.Pyvoronoi(diagram_scale)
        candidate_records = tuple(
            diagram_site_records[index]
            for index in candidate_record_indices
        )
        for _site_index, _shift, point_a, point_b in candidate_records:
            release_diagram.AddSegment(
                [
                    diagram_transform.to_diagram(point_a),
                    diagram_transform.to_diagram(point_b),
                ]
            )
        for guard_index in range(4):
            release_diagram.AddSegment(
                [
                    diagram_transform.to_diagram(guard[guard_index]),
                    diagram_transform.to_diagram(
                        guard[(guard_index + 1) % 4]
                    ),
                ]
            )
        try:
            release_diagram.Construct()
            if diagnostics is not None:
                diagnostics.construct_calls += 1
        except Exception as exc:
            raise _PatchVoronoiSurfaceCompileError(
                "BEVEL_RELEASE_COMPETITION_COMPILE_FAILED",
                (sites[index].edge_index for index in sorted(incident_sites)),
                f"{type(exc).__name__}: {exc}",
            ) from exc

        release_edges = release_diagram.GetEdges()
        release_vertices = tuple(
            _DiagramVertex(
                *diagram_transform.from_diagram((vertex.X, vertex.Y))
            )
            for vertex in release_diagram.GetVertices()
        )
        fragments_by_candidate = {}
        triangles_by_candidate = {}
        for cell in release_diagram.GetCells():
            if (
                cell.site < 0
                or cell.site >= len(candidate_records)
                or cell.is_open
                or cell.is_degenerate
            ):
                continue
            polygon = _cell_polygon(
                release_diagram,
                release_edges,
                release_vertices,
                cell,
                curve_step,
                diagnostics,
                diagram_transform,
            )
            if polygon is None:
                continue
            for cell_triangle in _triangulate_cell_polygon(
                polygon, diagram_transform.quantum
            ):
                for primary_fragment, triangle_id in zip(
                    primary_fragments, primary_triangle_ids
                ):
                    clipped = _clip_to_convex(
                        cell_triangle, primary_fragment
                    )
                    if (
                        len(clipped) < 3
                        or abs(_polygon_area2(clipped)) <= 1e-10
                    ):
                        continue
                    if _polygon_area2(clipped) < 0.0:
                        clipped.reverse()
                    fragments_by_candidate.setdefault(
                        int(cell.site), []
                    ).append(tuple(clipped))
                    triangles_by_candidate.setdefault(
                        int(cell.site), []
                    ).append(int(triangle_id))

        for local_index in sorted(fragments_by_candidate):
            owner_site_index, owner_shift = candidate_records[local_index][
                :2
            ]
            records_by_corner.setdefault(corner_index, []).append(
                _CornerReleaseAtom(
                    corner_index=corner_index,
                    point_site_index=int(point_site_index),
                    point_periodic_shift=int(point_periodic_shift),
                    owner_site_index=int(owner_site_index),
                    owner_periodic_shift=int(owner_shift),
                    fragments=tuple(fragments_by_candidate[local_index]),
                    fragment_triangle_ids=tuple(
                        triangles_by_candidate[local_index]
                    ),
                )
            )

    return {
        corner_index: tuple(records)
        for corner_index, records in sorted(records_by_corner.items())
    }


def _normalized_intrinsic_triangles(intrinsic_triangles, quantize_point):
    result = []
    for triangle in intrinsic_triangles:
        points = tuple(
            quantize_point(point) for point in triangle.chart_points
        )
        if abs(_polygon_area2(points)) <= 1e-12:
            raise _PatchVoronoiSurfaceCompileError(
                "QUANTIZED_DEGENERATE_DOMAIN_TRIANGLE",
                (),
                f"source_triangle={triangle.source_triangle_id!r}",
            )
        if _polygon_area2(points) < 0.0:
            triangle = replace(
                triangle,
                chart_points=tuple(reversed(points)),
                positions=tuple(reversed(triangle.positions)),
                normals=tuple(reversed(triangle.normals)),
                source_edge_ids=tuple(reversed(triangle.source_edge_ids)),
                source_vertex_ids=tuple(
                    reversed(triangle.source_vertex_ids)
                ),
                edge_transition_keys=tuple(
                    reversed(triangle.edge_transition_keys)
                ),
                vertex_transition_keys=tuple(
                    reversed(triangle.vertex_transition_keys)
                ),
            )
        else:
            triangle = replace(triangle, chart_points=points)
        result.append(triangle)
    return tuple(result)


def _intrinsic_triangle_merge_groups(triangles):
    """G1: объединяет только source-face coplanar adjacency."""

    if not triangles:
        return ()
    parents = list(range(len(triangles)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(first, second):
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parents[second_root] = first_root

    owners_by_edge = {}
    for triangle_id, triangle in enumerate(triangles):
        for source_edge in triangle.source_edge_ids:
            if source_edge is None:
                continue
            owners_by_edge.setdefault(source_edge, []).append(triangle_id)
    for owners in owners_by_edge.values():
        for first_position, first_id in enumerate(owners):
            for second_id in owners[first_position + 1 :]:
                first = triangles[first_id]
                second = triangles[second_id]
                same_face = (
                    first.source_face_id is not None
                    and first.source_face_id == second.source_face_id
                )
                normal_a = first.face_normal
                normal_b = second.face_normal
                coplanar = (
                    normal_a is not None
                    and normal_b is not None
                    and normal_a.length_squared > _GEOMETRY_EPS
                    and normal_b.length_squared > _GEOMETRY_EPS
                    and normal_a.normalized().dot(normal_b.normalized())
                    >= DECAL_COPLANAR_DOT
                )
                if same_face or coplanar:
                    union(first_id, second_id)

    group_by_root = {}
    return tuple(
        group_by_root.setdefault(find(triangle_id), len(group_by_root))
        for triangle_id in range(len(triangles))
    )


def _planar_domain_source_features(
    node,
    raw_sites,
    origin,
    basis_u,
    basis_v,
    quantize_point,
):
    """Компилирует physical boundary provenance PLANAR-domain."""

    edges = {}
    edge_positions = {}
    vertices = {}

    def add_edge(edge_id, vert_a, vert_b, source_a, source_b):
        edge_id = int(edge_id)
        vert_a = int(vert_a)
        vert_b = int(vert_b)
        if vert_b < vert_a:
            vert_a, vert_b = vert_b, vert_a
            source_a, source_b = source_b, source_a
        point_a = quantize_point(
            _project(source_a, origin, basis_u, basis_v)
        )
        point_b = quantize_point(
            _project(source_b, origin, basis_u, basis_v)
        )
        vertices.setdefault(vert_a, point_a)
        vertices.setdefault(vert_b, point_b)
        record = (edge_id, vert_a, vert_b, point_a, point_b)
        position_record = (
            edge_id,
            tuple(float(value) for value in source_a),
            tuple(float(value) for value in source_b),
        )
        previous = edges.get(edge_id)
        if previous is None or repr(record) < repr(previous):
            edges[edge_id] = record
            edge_positions[edge_id] = position_record

    for raw in raw_sites:
        add_edge(
            raw["edge_index"],
            raw["vert_a"],
            raw["vert_b"],
            raw["source_a"],
            raw["source_b"],
        )

    def add_polyline(vert_indices, vert_cos, edge_indices, is_closed):
        if len(vert_indices) != len(vert_cos) or len(vert_cos) < 2:
            return
        for segment_index, edge_id in enumerate(edge_indices):
            next_index = segment_index + 1
            if next_index >= len(vert_cos):
                if not is_closed:
                    continue
                next_index = 0
            add_edge(
                edge_id,
                vert_indices[segment_index],
                vert_indices[next_index],
                vert_cos[segment_index],
                vert_cos[next_index],
            )

    for boundary_loop in node.boundary_loops:
        add_polyline(
            boundary_loop.vert_indices,
            boundary_loop.vert_cos,
            boundary_loop.edge_indices,
            True,
        )
        for chain in boundary_loop.chains:
            add_polyline(
                chain.vert_indices,
                chain.vert_cos,
                chain.edge_indices,
                chain.is_closed,
            )

    return (
        tuple(edges[edge_id] for edge_id in sorted(edges)),
        tuple(
            (vertex_id, vertices[vertex_id])
            for vertex_id in sorted(vertices)
        ),
        tuple(
            edge_positions[edge_id] for edge_id in sorted(edge_positions)
        ),
    )


def _compile_surface(
    node,
    raw_sites,
    diagnostics=None,
    *,
    intrinsic_triangles=(),
    intrinsic_site_points=None,
    chart_id=0,
    intrinsic_alpha_budget=float("inf"),
    intrinsic_chart=None,
    required_site_edge_indices=None,
    corner_site_edge_indices=None,
    semantic_owner_chart_by_vertex=(),
):
    origin = node.centroid.copy()
    normal = node.normal.normalized()
    basis_u, basis_v = _canonical_planar_basis(normal)
    if intrinsic_triangles:
        raw_projected_sites = tuple(
            intrinsic_site_points[int(raw["edge_index"])]
            for raw in raw_sites
        )
        transform_points = [
            point
            for triangle in intrinsic_triangles
            for point in triangle.chart_points
        ]
    else:
        raw_projected_sites = tuple(
            (
                _project(raw["source_a"], origin, basis_u, basis_v),
                _project(raw["source_b"], origin, basis_u, basis_v),
            )
            for raw in raw_sites
        )
        transform_points = [
            _project(point, origin, basis_u, basis_v)
            for point in node.mesh_verts
        ]
        transform_points.extend(
            _project(point, origin, basis_u, basis_v)
            for loop in node.boundary_loops
            for point in loop.vert_cos
        )
    transform_points.extend(
        point
        for endpoints in raw_projected_sites
        for point in endpoints
    )
    diagram_transform = _build_diagram_transform(
        transform_points,
        (raw["edge_index"] for raw in raw_sites),
    )
    compiled_intrinsic_triangles = ()
    if intrinsic_triangles:
        compiled_intrinsic_triangles = _normalized_intrinsic_triangles(
            intrinsic_triangles, diagram_transform.quantize
        )
        triangles = [
            triangle.chart_points
            for triangle in compiled_intrinsic_triangles
        ]
    else:
        triangles = _patch_domain_triangles(
            node,
            origin,
            basis_u,
            basis_v,
            diagnostics,
            diagram_transform.quantize,
        )
    if not triangles:
        return None
    triangle_grid = _build_triangle_aabb_grid(triangles)
    reference_full_scan = bool(
        diagnostics is not None and diagnostics.reference_full_scan
    )

    all_points = [point for triangle in triangles for point in triangle]
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    diagonal = sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)
    probe_distance = max(
        DECAL_WELD_DISTANCE * 2.0,
        min(diagonal * 1e-4, max(diagonal, 1.0) * 1e-3),
    )
    projected_sites = []
    diagram_quantum = diagram_transform.quantum
    for raw, (raw_point_a, raw_point_b) in zip(
        raw_sites, raw_projected_sites
    ):
        # Один и тот же planar patch может прийти как отдельная плоскость
        # или как face объёмного mesh с микроскопически иными float32
        # координатами. Квантуем рабочие sites, а не только вход pyvoronoi:
        # иначе Voronoi cells совпадают, но realtime crop проходит разные
        # topology events на визуально идентичной геометрии.
        point_a = diagram_transform.quantize(raw_point_a)
        point_b = diagram_transform.quantize(raw_point_b)
        quantized_length = _dist2(point_a, point_b)
        if point_a == point_b or quantized_length <= diagram_quantum:
            raise _PatchVoronoiSurfaceCompileError(
                "QUANTIZED_DEGENERATE_SITE",
                (raw["edge_index"],),
                (
                    f"edge={int(raw['edge_index'])} "
                    f"raw_length={float(raw['segment_length']):.12g} "
                    f"quantized_length={quantized_length:.12g} "
                    f"quantum={diagram_quantum:.12g}"
                ),
            )
        endpoint_key = tuple(
            sorted(
                diagram_transform.quantize(point)
                for point in (point_a, point_b)
            )
        )
        projected_sites.append(
            (
                endpoint_key,
                raw,
                point_a,
                point_b,
                raw_point_a,
                raw_point_b,
            )
        )

    sites = []
    classification_sites = []
    for (
        _endpoint_key,
        raw,
        point_a,
        point_b,
        raw_point_a,
        raw_point_b,
    ) in sorted(
        projected_sites, key=lambda item: item[0]
    ):
        quantized_length = _dist2(point_a, point_b)
        inward_normal = _inward_site_normal(
            point_a,
            point_b,
            triangles,
            probe_distance,
            triangle_grid,
            reference_full_scan,
        )
        if (
            not all(isfinite(value) for value in inward_normal)
            or _norm2(inward_normal) is None
        ):
            raise _PatchVoronoiSurfaceCompileError(
                "INVALID_INWARD_NORMAL",
                (raw["edge_index"],),
                f"edge={int(raw['edge_index'])} normal={inward_normal!r}",
            )
        site = _PatchVoronoiSite(
            patch_id=node.patch_id,
            edge_index=raw["edge_index"],
            vert_a=raw["vert_a"],
            vert_b=raw["vert_b"],
            source_a=raw["source_a"],
            source_b=raw["source_b"],
            point_a=point_a,
            point_b=point_b,
            arc_start=raw["arc_start"],
            segment_length=quantized_length,
            uv_sign=raw["uv_sign"],
            inward_normal=inward_normal,
            arc_sign=raw.get("arc_sign", 1.0),
            two_sided=raw.get("two_sided", False),
            uv_length=raw["segment_length"],
            owner_face_index=int(raw.get("owner_face_index", -1)),
        )
        sites.append(site)
        # Угловая policy остаётся геометрическим фактом исходного контура.
        # Рабочая квантизация не должна менять точный concave/convex angle.
        classification_sites.append(
            replace(
                site,
                point_a=raw_point_a,
                point_b=raw_point_b,
                segment_length=raw["segment_length"],
                inward_normal=_inward_site_normal(
                    raw_point_a,
                    raw_point_b,
                    triangles,
                    probe_distance,
                    triangle_grid,
                    reference_full_scan,
                ),
            )
        )
    corner_points = {}
    for site in sites:
        corner_points.setdefault(site.vert_a, site.point_a)
        corner_points.setdefault(site.vert_b, site.point_b)
    periodic_axis = intrinsic_chart.periodic_axis if intrinsic_chart else ""
    periodic_period = intrinsic_chart.period if intrinsic_chart else 0.0
    compiled_corners = _compile_corners(
        classification_sites,
        periodic_axis=periodic_axis,
        period=periodic_period,
    )
    required_edges = (
        {site.edge_index for site in sites}
        if required_site_edge_indices is None
        else {int(edge_index) for edge_index in required_site_edge_indices}
    )
    corner_edges = (
        required_edges
        if corner_site_edge_indices is None
        else {int(edge_index) for edge_index in corner_site_edge_indices}
    )
    if required_site_edge_indices is not None:
        compiled_corners = tuple(
            corner
            for corner in compiled_corners
            if any(
                classification_sites[site_index].edge_index in corner_edges
                for site_index in corner.incident_sites
            )
        )
    corners = tuple(
        replace(
            corner,
            point=(
                diagram_transform.quantize(corner.point)
                if periodic_axis
                else corner_points[corner.vert_index]
            ),
        )
        for corner in compiled_corners
    )
    corner_by_vertex = {
        corner.vert_index: index for index, corner in enumerate(corners)
    }
    margin = max(diagonal, diagram_quantum * 32.0)
    guard = tuple(
        diagram_transform.quantize(point)
        for point in (
            (min_x - margin, min_y - margin),
            (max_x + margin, min_y - margin),
            (max_x + margin, max_y + margin),
            (min_x - margin, max_y + margin),
        )
    )
    actual_chart_points = [
        endpoint
        for site in sites
        for endpoint in (site.point_a, site.point_b)
    ]
    actual_chart_points.extend(guard)
    actual_diagram_points = tuple(
        diagram_transform.to_diagram(point)
        for point in actual_chart_points
    )
    maximum_integer_coordinate = max(
        abs(value) * diagram_transform.scale
        for point in actual_diagram_points
        for value in point
    )
    safe_integer_coordinate = (
        _DIAGRAM_INT_LIMIT * diagram_transform.int_safety_margin
    )
    if maximum_integer_coordinate > safe_integer_coordinate:
        raise _PatchVoronoiSurfaceCompileError(
            "DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED",
            (site.edge_index for site in sites),
            (
                f"actual_integer_coordinate={maximum_integer_coordinate:.12g} "
                f"safe_integer_coordinate={safe_integer_coordinate:.12g}"
            ),
        )

    diagram_scale = int(diagram_transform.scale)
    diagram = pyvoronoi.Pyvoronoi(diagram_scale)
    diagram_segments = []
    diagram_endpoint_vertices = []
    diagram_site_records = [
        (site_index, 0, site.point_a, site.point_b)
        for site_index, site in enumerate(sites)
    ]
    if intrinsic_chart is not None and intrinsic_chart.periodic_axis:
        period = float(intrinsic_chart.period)
        quantum = diagram_transform.quantum
        if (
            intrinsic_chart.periodic_axis != "U"
            or abs(period / quantum - round(period / quantum)) > 1e-7
        ):
            raise _PatchVoronoiSurfaceCompileError(
                "PERIODIC_QUANTIZATION_MISMATCH",
                (site.edge_index for site in sites),
                (
                    f"axis={intrinsic_chart.periodic_axis!r} "
                    f"period={period:.12g} quantum={quantum:.12g}"
                ),
            )
        lower = float(intrinsic_chart.wrap_origin)
        upper = lower + period
        budget = min(float(intrinsic_chart.alpha_budget), period * 0.5)
        tolerance = quantum * 1e-7
        images = []
        for site_index, site in enumerate(sites):
            endpoints = (site.point_a, site.point_b)
            if min(abs(point[0] - lower) for point in endpoints) <= (
                budget + tolerance
            ):
                images.append((site_index, 1))
            if min(abs(point[0] - upper) for point in endpoints) <= (
                budget + tolerance
            ):
                images.append((site_index, -1))
        for site_index, shift in sorted(images):
            site = sites[site_index]
            shifted = tuple(
                diagram_transform.quantize(
                    (point[0] + shift * period, point[1])
                )
                for point in (site.point_a, site.point_b)
            )
            diagram_site_records.append(
                (site_index, shift, shifted[0], shifted[1])
            )
        if diagnostics is not None:
            diagnostics.periodic_copy_count += len(images)

    for owner_site_index, _shift, record_a, record_b in diagram_site_records:
        site = sites[owner_site_index]
        point_by_vertex = {
            site.vert_a: record_a,
            site.vert_b: record_b,
        }
        ordered_vertices = tuple(sorted(point_by_vertex))
        ordered_points = tuple(point_by_vertex[index] for index in ordered_vertices)
        if ordered_points[0] <= ordered_points[1]:
            diagram_points = ordered_points
            endpoint_vertices = ordered_vertices
        else:
            diagram_points = tuple(reversed(ordered_points))
            endpoint_vertices = tuple(reversed(ordered_vertices))
        diagram_segment = [
            diagram_transform.to_diagram(point) for point in diagram_points
        ]
        diagram.AddSegment(diagram_segment)
        diagram_segments.append(diagram_segment)
        diagram_endpoint_vertices.append(endpoint_vertices)
    for index in range(4):
        diagram_segment = [
            diagram_transform.to_diagram(guard[index]),
            diagram_transform.to_diagram(guard[(index + 1) % 4]),
        ]
        diagram.AddSegment(diagram_segment)
        diagram_segments.append(diagram_segment)

    # Некоторые wheels меняют внутреннее состояние validation-методами даже
    # до Construct. Поэтому проверки выполняются на отдельном input diagram.
    validation_diagram = pyvoronoi.Pyvoronoi(diagram_scale)
    for segment in diagram_segments:
        validation_diagram.AddSegment(segment)
    for method_name, reason in (
        ("GetDegenerateSegments", "PYVORONOI_DEGENERATE_SEGMENT"),
        ("GetPointsOnSegments", "PYVORONOI_POINT_ON_SEGMENT"),
    ):
        method = getattr(validation_diagram, method_name, None)
        if method is None:
            continue
        invalid_indices = tuple(int(index) for index in method())
        invalid_edges = tuple(
            sites[diagram_site_records[index][0]].edge_index
            for index in invalid_indices
            if 0 <= index < len(diagram_site_records)
        )
        if invalid_edges:
            raise _PatchVoronoiSurfaceCompileError(
                reason,
                invalid_edges,
                f"site_indices={invalid_indices!r}",
            )
    try:
        diagram.Construct()
        if diagnostics is not None:
            diagnostics.construct_calls += 1
    except Exception as exc:
        raise _PatchVoronoiSurfaceCompileError(
            "PYVORONOI_CONSTRUCT_FAILED",
            (site.edge_index for site in sites),
            f"{type(exc).__name__}: {exc}",
        ) from exc

    # Curved segment-Voronoi bisectors are parabolas. Слишком длинная хорда
    # может ложно отдать кусок endpoint-cell соседнему segment-cell: его strip
    # туда ещё не дошёл, поэтому при width drag возникает мигрирующая дырка.
    # diagonal / 40 сохраняет low-poly contour, но удерживает chord error ниже
    # размера видимого фронтирного зазора на широких production patches.
    sorted_site_lengths = sorted(site.segment_length for site in sites)
    median_site_length = sorted_site_lengths[len(sorted_site_lengths) // 2]
    curve_step = max(
        min(diagonal / 40.0, median_site_length * 0.25),
        DECAL_WELD_DISTANCE * 2.0,
    )
    diagram_edges = diagram.GetEdges()
    diagram_cells = tuple(diagram.GetCells())
    diagram_vertices = tuple(
        _DiagramVertex(*diagram_transform.from_diagram((vertex.X, vertex.Y)))
        for vertex in diagram.GetVertices()
    )
    corners = tuple(
        replace(
            corner,
            split_chord=_compile_corner_split_chord(
                sites,
                corner,
                diagram_vertices,
                triangles,
                triangle_grid,
                reference_full_scan,
            ),
            static_wedge=_compile_corner_static_wedge(
                sites, corner, triangles
            ),
        )
        for corner in corners
    )
    atoms = []
    point_cell_records = []
    for cell_index, cell in enumerate(diagram_cells):
        if (
            cell.site < 0
            or cell.site >= len(diagram_site_records)
            or cell.is_open
            or cell.is_degenerate
        ):
            continue
        polygon = _cell_polygon(
            diagram,
            diagram_edges,
            diagram_vertices,
            cell,
            curve_step,
            diagnostics,
            diagram_transform,
        )
        if polygon is None:
            continue
        cell_triangles = _triangulate_cell_polygon(
            polygon, diagram_transform.quantum
        )
        if not cell_triangles:
            continue
        owner_site_index, periodic_shift = diagram_site_records[cell.site][:2]
        site = sites[owner_site_index]
        fragments = []
        fragment_triangle_ids = []
        for cell_triangle in cell_triangles:
            if reference_full_scan:
                domain_triangle_ids = range(len(triangles))
            else:
                domain_triangle_ids = triangle_grid.query_aabb(
                    min(point[0] for point in cell_triangle),
                    min(point[1] for point in cell_triangle),
                    max(point[0] for point in cell_triangle),
                    max(point[1] for point in cell_triangle),
                )
            for domain_triangle_id in domain_triangle_ids:
                domain_triangle = triangles[domain_triangle_id]
                clipped = _clip_to_triangle(cell_triangle, domain_triangle)
                if (
                    len(clipped) < 3
                    or abs(_polygon_area2(clipped)) <= 1e-10
                ):
                    continue
                if _polygon_area2(clipped) < 0.0:
                    clipped.reverse()
                fragments.append(tuple(clipped))
                fragment_triangle_ids.append(int(domain_triangle_id))
        if fragments:
            source_category = int(cell.source_category)
            corner_index = -1
            cell_kind = "SEGMENT"
            if cell.contains_point:
                cell_kind = "POINT"
                if source_category == 1:
                    source_vertex = diagram_endpoint_vertices[cell.site][0]
                elif source_category == 2:
                    source_vertex = diagram_endpoint_vertices[cell.site][1]
                else:
                    source_vertex = -1
                corner_index = corner_by_vertex.get(source_vertex, -1)
            atoms.append(
                _PatchVoronoiAtom(
                    site_index=int(owner_site_index),
                    fragments=tuple(fragments),
                    cell_kind=cell_kind,
                    fragment_triangle_ids=tuple(fragment_triangle_ids),
                    corner_index=corner_index,
                    source_category=source_category,
                    periodic_shift=int(periodic_shift),
                )
            )
            if cell_kind == "POINT" and corner_index >= 0:
                point_cell_records.append(
                    (
                        int(cell_index),
                        int(corner_index),
                        int(owner_site_index),
                        int(periodic_shift),
                        tuple(fragments),
                        tuple(fragment_triangle_ids),
                    )
                )
    corner_release_atoms = _compile_corner_release_atoms(
        diagram_transform=diagram_transform,
        diagram_site_records=tuple(diagram_site_records),
        sites=tuple(sites),
        corners=corners,
        diagram_cells=diagram_cells,
        diagram_edges=diagram_edges,
        guard=guard,
        curve_step=curve_step,
        point_cell_records=tuple(point_cell_records),
        diagnostics=diagnostics,
    )
    segment_site_indices = {
        atom.site_index for atom in atoms if atom.cell_kind == "SEGMENT"
    }
    missing_segment_sites = tuple(
        index
        for index, site in enumerate(sites)
        if site.edge_index in required_edges and index not in segment_site_indices
    )
    if missing_segment_sites:
        raise _PatchVoronoiSurfaceCompileError(
            "MISSING_SEGMENT_ATOM",
            (sites[index].edge_index for index in missing_segment_sites),
            f"site_indices={missing_segment_sites!r}",
        )
    site_grid_size = max(
        median_site_length,
        diagonal / max(8.0, sqrt(len(sites)) * 2.0),
        DECAL_WELD_DISTANCE * 4.0,
    )
    site_grid = {}
    for site_index, site in enumerate(sites):
        margin = max(DECAL_WELD_DISTANCE, site.segment_length * 1e-7)
        min_grid_x = int(
            (min(site.point_a[0], site.point_b[0]) - margin)
            // site_grid_size
        )
        max_grid_x = int(
            (max(site.point_a[0], site.point_b[0]) + margin)
            // site_grid_size
        )
        min_grid_y = int(
            (min(site.point_a[1], site.point_b[1]) - margin)
            // site_grid_size
        )
        max_grid_y = int(
            (max(site.point_a[1], site.point_b[1]) + margin)
            // site_grid_size
        )
        for grid_x in range(min_grid_x, max_grid_x + 1):
            for grid_y in range(min_grid_y, max_grid_y + 1):
                site_grid.setdefault((grid_x, grid_y), []).append(site_index)
    (
        atoms_by_site,
        owner_atoms_by_corner,
        corners_by_site,
        sites_by_vertex,
        ports_by_vertex,
        ports_by_site,
    ) = _compile_surface_relations(sites, corners, atoms)
    if compiled_intrinsic_triangles:
        planar_source_edges = ()
        planar_source_vertices = ()
        planar_source_edge_positions = ()
        planar_triangle_source_face_ids = ()
        triangle_merge_groups = _intrinsic_triangle_merge_groups(
            compiled_intrinsic_triangles
        )
    else:
        (
            planar_source_edges,
            planar_source_vertices,
            planar_source_edge_positions,
        ) = _planar_domain_source_features(
            node,
            raw_sites,
            origin,
            basis_u,
            basis_v,
            diagram_transform.quantize,
        )
        mesh_tri_face_indices = tuple(
            getattr(node, "mesh_tri_face_indices", ())
        )
        if len(mesh_tri_face_indices) == len(triangles):
            planar_triangle_source_face_ids = mesh_tri_face_indices
        else:
            fallback_face_id = next(
                (
                    int(raw["owner_face_index"])
                    for raw in raw_sites
                    if int(raw.get("owner_face_index", -1)) >= 0
                ),
                int(node.patch_id),
            )
            planar_triangle_source_face_ids = (
                fallback_face_id,
            ) * len(triangles)
        triangle_merge_groups = (0,) * len(triangles)
    domain = DecalSurfaceDomain(
        patch_id=node.patch_id,
        kind="INTRINSIC" if compiled_intrinsic_triangles else "PLANAR",
        origin=origin,
        reference_normal=normal,
        basis_u=basis_u,
        basis_v=basis_v,
        boundary_triangles=tuple(tuple(triangle) for triangle in triangles),
        boundary_triangle_source_face_ids=tuple(
            planar_triangle_source_face_ids
        ),
        intrinsic_triangles=compiled_intrinsic_triangles,
        planar_source_edges=planar_source_edges,
        planar_source_vertices=planar_source_vertices,
        planar_source_edge_positions=planar_source_edge_positions,
        triangle_merge_groups=triangle_merge_groups,
        periodic_axis=(
            intrinsic_chart.periodic_axis if intrinsic_chart else ""
        ),
        period=(intrinsic_chart.period if intrinsic_chart else 0.0),
        period_quantum=(
            intrinsic_chart.period_quantum if intrinsic_chart else 0.0
        ),
        wrap_origin=(
            intrinsic_chart.wrap_origin if intrinsic_chart else 0.0
        ),
        periodic_cut=(
            intrinsic_chart.periodic_cut if intrinsic_chart else None
        ),
        transition_equivalences=(
            intrinsic_chart.transition_equivalences
            if intrinsic_chart
            else ()
        ),
        chart_id=int(chart_id),
        alpha_budget=float(intrinsic_alpha_budget),
        budget_source=(
            intrinsic_chart.budget_source
            if intrinsic_chart is not None
            else "FULL_CONNECTED_COMPONENT"
        ),
        normal_mode=(
            "SMOOTH_INTERPOLATED"
            if intrinsic_chart is not None
            and intrinsic_chart.admission_tier == "APPROXIMATE"
            else "PIECEWISE_PLANAR_HARD"
        ),
        triangle_grid=triangle_grid,
        admission_tier=(
            intrinsic_chart.admission_tier
            if intrinsic_chart is not None
            else "EXACT"
        ),
        normalize_fragment_t_junctions=bool(
            intrinsic_chart is not None
            and intrinsic_chart.admission_tier == "APPROXIMATE"
        ),
        location_tolerance=(
            max(1e-7, diagram_transform.quantum * 2.0)
            if intrinsic_chart is not None
            and intrinsic_chart.admission_tier == "APPROXIMATE"
            else 1e-7
        ),
        reference_full_scan=reference_full_scan,
    )
    return _PatchVoronoiSurface(
        patch_id=node.patch_id,
        domain=domain,
        sites=tuple(sites),
        corners=corners,
        atoms=tuple(atoms),
        diagram_transform=diagram_transform,
        site_grid_size=site_grid_size,
        site_grid={
            key: tuple(indices) for key, indices in site_grid.items()
        },
        atoms_by_site=atoms_by_site,
        owner_atoms_by_corner=owner_atoms_by_corner,
        corners_by_site=corners_by_site,
        sites_by_vertex=sites_by_vertex,
        ports_by_vertex=ports_by_vertex,
        ports_by_site=ports_by_site,
        corner_release_atoms=corner_release_atoms,
        semantic_owner_chart_by_vertex=tuple(
            sorted(
                (int(vertex_id), int(owner_chart_id))
                for vertex_id, owner_chart_id
                in semantic_owner_chart_by_vertex
            )
        ),
        native_site_edge_indices=tuple(
            sorted(int(edge_index) for edge_index in (required_site_edge_indices or ()))
        ),
    )


def _intrinsic_domain_triangles(chart):
    cuts_by_edge = {
        source_edge: cut
        for cut in chart.cuts
        for source_edge in cut.source_edges
    }
    result = []
    image_keys_by_transition = dict(chart.transition_equivalences)
    for triangle in chart.triangles:
        face_normal = Vector(triangle.face_normal)
        source_edge_ids = []
        edge_transition_keys = []
        for source_edge, edge_index in zip(
            triangle.source_edge_ids, triangle.source_edge_indices
        ):
            cut = cuts_by_edge.get(source_edge)
            source_edge_ids.append(
                int(edge_index)
                if edge_index >= 0
                else ("TRIANGULATION_EDGE", source_edge)
            )
            if cut is not None:
                transition_key = cut.transition_key
                image_keys = tuple(
                    image_keys_by_transition.get(transition_key, ())
                )
                if chart.periodic_axis and len(image_keys) == 2:
                    points = dict(
                        zip(
                            triangle.source_vertex_ids,
                            triangle.chart_points,
                        )
                    )
                    midpoint_u = (
                        points[source_edge[0]][0]
                        + points[source_edge[1]][0]
                    ) * 0.5
                    side_index = (
                        0
                        if midpoint_u
                        < chart.wrap_origin + chart.period * 0.5
                        else 1
                    )
                    transition_key = tuple(
                        sorted(image_keys, key=repr)
                    )[side_index]
                edge_transition_keys.append(transition_key)
            elif edge_index >= 0:
                edge_transition_keys.append(("SOURCE_EDGE", int(edge_index)))
            else:
                edge_transition_keys.append(None)
        result.append(
            _IntrinsicDomainTriangle(
                chart_points=tuple(triangle.chart_points),
                positions=tuple(Vector(point) for point in triangle.positions),
                normals=(face_normal.copy(),) * 3,
                face_normal=face_normal,
                source_triangle_id=triangle.triangle_id,
                source_face_id=triangle.source_face_id,
                source_edge_ids=tuple(source_edge_ids),
                source_vertex_ids=tuple(triangle.source_vertex_ids),
                edge_transition_keys=tuple(edge_transition_keys),
                vertex_transition_keys=tuple(
                    ("SOURCE_VERTEX", int(vertex_id))
                    for vertex_id in triangle.source_vertex_ids
                ),
            )
        )
    return tuple(result)


def build_intrinsic_surface_domain(node, chart):
    """C4 adapter admitted chart → barycentric intrinsic domain."""

    intrinsic_triangles = _intrinsic_domain_triangles(chart)
    return DecalSurfaceDomain(
        patch_id=int(node.patch_id),
        kind="INTRINSIC",
        origin=node.centroid.copy(),
        reference_normal=node.normal.normalized(),
        basis_u=node.basis_u.copy(),
        basis_v=node.basis_v.copy(),
        boundary_triangles=tuple(
            triangle.chart_points for triangle in intrinsic_triangles
        ),
        intrinsic_triangles=intrinsic_triangles,
        triangle_merge_groups=_intrinsic_triangle_merge_groups(
            intrinsic_triangles
        ),
        periodic_axis=chart.periodic_axis,
        period=chart.period,
        period_quantum=chart.period_quantum,
        wrap_origin=chart.wrap_origin,
        periodic_cut=chart.periodic_cut,
        transition_equivalences=chart.transition_equivalences,
        chart_id=int(chart.chart_id),
        alpha_budget=float(chart.alpha_budget),
        budget_source=chart.budget_source,
        normal_mode=(
            "SMOOTH_INTERPOLATED"
            if chart.admission_tier == "APPROXIMATE"
            else "PIECEWISE_PLANAR_HARD"
        ),
        admission_tier=chart.admission_tier,
        normalize_fragment_t_junctions=(
            chart.admission_tier == "APPROXIMATE"
        ),
    )


def _intrinsic_site_points(chart, raw_sites):
    result = {}
    for raw in raw_sites:
        source_edge = tuple(
            sorted((int(raw["vert_a"]), int(raw["vert_b"])))
        )
        candidates = tuple(
            triangle
            for triangle in chart.triangles
            if source_edge in triangle.source_edge_ids
        )
        owner_candidates = tuple(
            triangle
            for triangle in candidates
            if triangle.source_face_id == int(raw["owner_face_index"])
        )
        candidates = owner_candidates or candidates
        if not candidates:
            raise _PatchVoronoiSurfaceCompileError(
                "SITE_OUTSIDE_INTRINSIC_CHART",
                (raw["edge_index"],),
                f"source_edge={source_edge!r}",
            )
        owner = min(
            candidates,
            key=lambda triangle: (
                triangle.source_face_id,
                triangle.triangle_id,
            ),
        )
        points = dict(zip(owner.source_vertex_ids, owner.chart_points))
        result[int(raw["edge_index"])] = (
            points[int(raw["vert_a"])],
            points[int(raw["vert_b"])],
        )
    return result


def _periodic_transport_raw_sites(raw_sites):
    """Разворачивает замкнутые selected-компоненты в монотонную V-фазу.

    Изменение локально для periodic chart: non-periodic Dijkstra transport
    остаётся бит-в-бит прежним. Направление цикла задают минимальная source
    vertex и минимальный incident edge id, поэтому winding owner mesh на него
    не влияет.
    """

    result = [dict(raw) for raw in raw_sites]
    unique_edges = {}
    for raw in result:
        unique_edges.setdefault(int(raw["edge_index"]), raw)
    adjacency = {}
    for edge_index, raw in unique_edges.items():
        first = int(raw["vert_a"])
        second = int(raw["vert_b"])
        length = float(raw["segment_length"])
        adjacency.setdefault(first, []).append((edge_index, second, length))
        adjacency.setdefault(second, []).append((edge_index, first, length))

    remaining = set(adjacency)
    transport = {}
    while remaining:
        root = min(remaining)
        component_vertices = {root}
        frontier = [root]
        while frontier:
            current = frontier.pop()
            for _edge_index, neighbour, _length in adjacency[current]:
                if neighbour not in component_vertices:
                    component_vertices.add(neighbour)
                    frontier.append(neighbour)
        remaining.difference_update(component_vertices)
        if any(len(adjacency[vertex]) != 2 for vertex in component_vertices):
            continue

        current = root
        previous_edge = None
        distance = 0.0
        visited_edges = set()
        while True:
            candidates = sorted(
                entry
                for entry in adjacency[current]
                if entry[0] != previous_edge
            )
            if not candidates:
                break
            edge_index, neighbour, length = candidates[0]
            if edge_index in visited_edges:
                break
            visited_edges.add(edge_index)
            transport[edge_index] = (current, neighbour, distance, length)
            distance += length
            previous_edge = edge_index
            current = neighbour
            if current == root:
                break
        component_edges = {
            edge_index
            for vertex in component_vertices
            for edge_index, _neighbour, _length in adjacency[vertex]
        }
        if current != root or visited_edges != component_edges:
            for edge_index in visited_edges:
                transport.pop(edge_index, None)

    for raw in result:
        entry = transport.get(int(raw["edge_index"]))
        if entry is None:
            continue
        start, end, distance, length = entry
        if int(raw["vert_a"]) == start and int(raw["vert_b"]) == end:
            raw["arc_start"] = distance
            raw["arc_sign"] = 1.0
        else:
            raw["arc_start"] = distance + length
            raw["arc_sign"] = -1.0
    return tuple(result)


def _compile_intrinsic_surface(node, chart, raw_sites, diagnostics=None):
    if diagnostics is not None:
        if chart.admission_tier == "APPROXIMATE":
            diagnostics.approximate_admit_count += 1
        diagnostics.max_width_error_sampled = max(
            diagnostics.max_width_error_sampled,
            chart.metrics.max_width_error_sampled,
        )
        diagnostics.max_station_normal_variation = max(
            diagnostics.max_station_normal_variation,
            chart.metrics.max_station_normal_variation,
        )
        diagnostics.foldover_count += chart.metrics.foldover_count
    if chart.periodic_axis:
        raw_sites = _periodic_transport_raw_sites(raw_sites)
    intrinsic_triangles = _intrinsic_domain_triangles(chart)
    return _compile_surface(
        node,
        raw_sites,
        diagnostics,
        intrinsic_triangles=intrinsic_triangles,
        intrinsic_site_points=_intrinsic_site_points(chart, raw_sites),
        chart_id=chart.chart_id,
        intrinsic_alpha_budget=chart.alpha_budget,
        intrinsic_chart=chart,
    )


def _affine_inverse(transform):
    cosine, sine, tx, ty = transform
    return (
        cosine,
        -sine,
        -(cosine * tx + sine * ty),
        sine * tx - cosine * ty,
    )


def _affine_compose(second, first):
    """Возвращает transform second(first(point))."""

    ac, ass, atx, aty = first
    bc, bs, btx, bty = second
    return (
        bc * ac - bs * ass,
        bs * ac + bc * ass,
        bc * atx - bs * aty + btx,
        bs * atx + bc * aty + bty,
    )


def _affine_point(transform, point):
    cosine, sine, tx, ty = transform
    return (
        cosine * point[0] - sine * point[1] + tx,
        sine * point[0] + cosine * point[1] + ty,
    )


def _frame_affine_inverse(transform):
    """Обратный общий affine для R5 UV-frame transport."""

    m00, m01, m10, m11, tx, ty = transform
    determinant = m00 * m11 - m01 * m10
    if abs(determinant) <= 1e-20:
        raise ValueError("ATLAS_TRANSITION_DESYNC: UV frame is singular")
    inverse = 1.0 / determinant
    i00 = m11 * inverse
    i01 = -m01 * inverse
    i10 = -m10 * inverse
    i11 = m00 * inverse
    return (
        i00,
        i01,
        i10,
        i11,
        -(i00 * tx + i01 * ty),
        -(i10 * tx + i11 * ty),
    )


def _frame_affine_compose(second, first):
    """Возвращает общий affine second(first(point))."""

    a00, a01, a10, a11, atx, aty = first
    b00, b01, b10, b11, btx, bty = second
    return (
        b00 * a00 + b01 * a10,
        b00 * a01 + b01 * a11,
        b10 * a00 + b11 * a10,
        b10 * a01 + b11 * a11,
        b00 * atx + b01 * aty + btx,
        b10 * atx + b11 * aty + bty,
    )


def _frame_affine_point(transform, point):
    m00, m01, m10, m11, tx, ty = transform
    return (
        m00 * point[0] + m01 * point[1] + tx,
        m10 * point[0] + m11 * point[1] + ty,
    )


def _atlas_transforms_from(atlas, owner_chart_id):
    """Каноническое spanning-tree transport без повторного holonomy solve."""

    graph = {chart.chart_id: [] for chart in atlas.charts}
    for transition in atlas.transitions:
        forward = (
            transition.rotation_cos,
            transition.rotation_sin,
            transition.translation[0],
            transition.translation[1],
        )
        graph[transition.owner_chart_id].append(
            (transition.neighbor_chart_id, transition.transition_key, forward)
        )
        graph[transition.neighbor_chart_id].append(
            (
                transition.owner_chart_id,
                transition.transition_key,
                _affine_inverse(forward),
            )
        )
    identity = (1.0, 0.0, 0.0, 0.0)
    result = {owner_chart_id: identity}
    queue = [owner_chart_id]
    while queue:
        current = queue.pop(0)
        for neighbor, key, transform in sorted(
            graph[current], key=lambda item: (item[0], repr(item[1]))
        ):
            if neighbor in result:
                continue
            result[neighbor] = _affine_compose(transform, result[current])
            queue.append(neighbor)
    if len(result) != len(graph):
        raise _PatchVoronoiSurfaceCompileError(
            "DISCONNECTED_ATLAS", (), f"owner_chart={owner_chart_id}"
        )
    return result


def _atlas_vertex_owner(atlas, vertex_id):
    candidates = []
    for chart in atlas.charts:
        for triangle in chart.triangles:
            if vertex_id not in triangle.source_vertex_ids:
                continue
            points = dict(zip(triangle.source_vertex_ids, triangle.chart_points))
            candidates.append(
                (
                    chart.chart_id,
                    triangle.triangle_id,
                    points[vertex_id],
                )
            )
    if not candidates:
        raise _PatchVoronoiSurfaceCompileError(
            "SITE_OUTSIDE_INTRINSIC_ATLAS",
            (),
            f"source_vertex={vertex_id!r}",
        )
    return min(candidates, key=lambda item: (item[0], item[1]))


def _compile_intrinsic_atlas_surfaces(
    node, atlas, raw_sites, diagnostics=None
):
    """Компилирует atlas charts и R1 site images в их локальных frames."""

    if diagnostics is not None:
        diagnostics.atlas_chart_count += atlas.atlas_chart_count
        diagnostics.interior_transition_count += (
            atlas.interior_transition_count
        )
        diagnostics.atlas_unresolved_overlap_count += (
            atlas.unresolved_overlap_count
        )
        diagnostics.margin_relief_cut_count += atlas.margin_relief_cut_count
        diagnostics.max_width_error_sampled = max(
            diagnostics.max_width_error_sampled,
            atlas.metrics.max_width_error_sampled,
        )
        diagnostics.max_station_normal_variation = max(
            diagnostics.max_station_normal_variation,
            atlas.metrics.max_station_normal_variation,
        )
        diagnostics.foldover_count += atlas.metrics.foldover_count

    vertex_ids = {
        int(raw[key]) for raw in raw_sites for key in ("vert_a", "vert_b")
    }
    owner_records = {
        vertex_id: _atlas_vertex_owner(atlas, vertex_id)
        for vertex_id in vertex_ids
    }
    transforms_by_owner = {
        owner_chart_id: _atlas_transforms_from(atlas, owner_chart_id)
        for owner_chart_id, _triangle_id, _point in owner_records.values()
    }
    surfaces = []
    for source_chart in atlas.charts:
        chart = replace(
            source_chart, admission_tier=atlas.admission_tier
        )
        site_points = {}
        native_edges = {seed.edge_index for seed in chart.site_seeds}
        chart_points = tuple(
            point for triangle in chart.triangles for point in triangle.chart_points
        )
        budget = float(chart.alpha_budget)
        chart_bounds = (
            min(point[0] for point in chart_points) - budget,
            min(point[1] for point in chart_points) - budget,
            max(point[0] for point in chart_points) + budget,
            max(point[1] for point in chart_points) + budget,
        )
        chart_raw_sites = []
        for raw in raw_sites:
            edge_index = int(raw["edge_index"])
            endpoint_records = tuple(
                owner_records[int(raw[key])] for key in ("vert_a", "vert_b")
            )
            points = tuple(
                _affine_point(
                    transforms_by_owner[owner_chart_id][chart.chart_id],
                    point,
                )
                for owner_chart_id, _triangle_id, point in endpoint_records
            )
            site_min_x = min(point[0] for point in points)
            site_max_x = max(point[0] for point in points)
            site_min_y = min(point[1] for point in points)
            site_max_y = max(point[1] for point in points)
            if (
                edge_index not in native_edges
                and (
                    site_max_x < chart_bounds[0]
                    or site_min_x > chart_bounds[2]
                    or site_max_y < chart_bounds[1]
                    or site_min_y > chart_bounds[3]
                )
            ):
                continue
            site_points[edge_index] = points
            chart_raw_sites.append(raw)
            if diagnostics is not None and any(
                owner_chart_id != chart.chart_id
                for owner_chart_id, _triangle_id, _point in endpoint_records
            ):
                diagnostics.atlas_site_image_count += 1
        surfaces.append(
            _compile_surface(
                node,
                tuple(chart_raw_sites),
                diagnostics,
                intrinsic_triangles=_intrinsic_domain_triangles(chart),
                intrinsic_site_points=site_points,
                chart_id=chart.chart_id,
                intrinsic_alpha_budget=chart.alpha_budget,
                intrinsic_chart=chart,
                required_site_edge_indices=native_edges,
                corner_site_edge_indices=tuple(
                    int(raw["edge_index"]) for raw in chart_raw_sites
                ),
                semantic_owner_chart_by_vertex=tuple(
                    (
                        vertex_id,
                        owner_record[0],
                    )
                    for vertex_id, owner_record in owner_records.items()
                ),
            )
        )
    return tuple(surfaces)


def _fallback_chart_alpha_budget(node):
    """Конечный compile budget для diagnostic вызовов без runtime width."""

    if not node.mesh_verts:
        return 1.0
    minimum = Vector(
        tuple(min(point[axis] for point in node.mesh_verts) for axis in range(3))
    )
    maximum = Vector(
        tuple(max(point[axis] for point in node.mesh_verts) for axis in range(3))
    )
    return max((maximum - minimum).length, 1e-6)


def _intrinsic_chart_site_seeds(node, patch_sites):
    """Строго сериализует sites; отсутствие provenance означает fallback."""

    result = []
    for raw in sorted(
        patch_sites,
        key=lambda item: (
            int(item["edge_index"]),
            int(item["owner_face_index"]),
        ),
    ):
        edge_index = int(raw["edge_index"])
        owner_face_index = int(raw["owner_face_index"])
        if owner_face_index < 0:
            raise ChartBuildFailure(
                "MISSING_SITE_FACE_PROVENANCE",
                int(node.patch_id),
                edge_ids=(edge_index,),
            )
        try:
            result.append(
                ChartSiteSeed(
                    edge_index=edge_index,
                    source_vertex_ids=tuple(
                        sorted((int(raw["vert_a"]), int(raw["vert_b"])))
                    ),
                    source_face_id=owner_face_index,
                    chain_ref=None,
                )
            )
        except (TypeError, ValueError) as exc:
            raise ChartBuildFailure(
                "INVALID_SITE_PROVENANCE",
                int(node.patch_id),
                edge_ids=(edge_index,),
                details=str(exc),
            ) from exc
    return tuple(result)


def compile_patch_voronoi_attempt(
    analysis_bundle,
    selected_edge_indices,
    offset,
    *,
    allow_partial=False,
    diagnostics=None,
    alpha_budget=None,
    distortion_budget=CHART_DISTORTION_BUDGET,
    corner_join_mode=CornerJoinMode.MITER,
):
    """Компилирует plan и локализует unsupported patches до physical edges.

    ``allow_partial=False`` сохраняет прежний all-or-nothing контракт.
    Диагностический partial-режим нужен hybrid router: его неполный plan не
    материализуется напрямую, потому что rejected topology component может
    потребовать исключить дополнительные соседние edges и повторный compile.
    """

    if not isinstance(analysis_bundle, AnalysisBundle):
        raise AnalysisSchemaError(
            "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED",
            "Patch Voronoi compiler requires AnalysisBundle",
        )
    analysis_bundle.capabilities.require_supported()
    corner_join_mode = CornerJoinMode(corner_join_mode)
    graph = analysis_bundle.patch_graph
    patch_surface = analysis_bundle.patch_surface
    requested_alpha_budget = (
        float("inf") if alpha_budget is None else float(alpha_budget)
    )
    if not requested_alpha_budget > 0.0:
        raise ValueError("Patch Voronoi compile alpha_budget must be positive")
    selected_edges = {
        int(edge_index) for edge_index in selected_edge_indices or ()
    }
    if not selected_edges:
        return PatchVoronoiCompileAttempt(plan=None)
    if pyvoronoi is None:
        return PatchVoronoiCompileAttempt(
            plan=None,
            rejected_edge_indices=tuple(sorted(selected_edges)),
            failures=(
                PatchVoronoiCompileFailure(
                    patch_id=-1,
                    reason="PYVORONOI_UNAVAILABLE",
                    edge_indices=tuple(sorted(selected_edges)),
                ),
            ),
        )
    internal_failures = _single_use_internal_seam_failures(
        graph, selected_edges
    )
    rejected_internal_edges = {
        edge_index
        for failure in internal_failures
        for edge_index in failure.edge_indices
    }
    if internal_failures and not allow_partial:
        return PatchVoronoiCompileAttempt(
            plan=None,
            rejected_edge_indices=tuple(sorted(rejected_internal_edges)),
            failures=internal_failures,
        )
    compilable_edges = selected_edges - rejected_internal_edges
    if not compilable_edges:
        return PatchVoronoiCompileAttempt(
            plan=None,
            rejected_edge_indices=tuple(sorted(rejected_internal_edges)),
            failures=internal_failures,
        )
    raw_by_patch, normals_by_vert, positions_by_vert = _collect_patch_sites(
        graph, compilable_edges
    )
    if not raw_by_patch:
        return PatchVoronoiCompileAttempt(
            plan=None,
            rejected_edge_indices=tuple(sorted(selected_edges)),
            failures=internal_failures
            + (
                PatchVoronoiCompileFailure(
                    patch_id=-1,
                    reason="NO_PATCH_SITES",
                    edge_indices=tuple(sorted(compilable_edges)),
                ),
            ),
        )
    surfaces = []
    rejected_edges = set(rejected_internal_edges)
    failures = list(internal_failures)
    for patch_id in sorted(raw_by_patch):
        node = _patch_surface_node_view(
            graph.nodes[patch_id],
            patch_surface,
        )
        patch_sites = raw_by_patch[patch_id]
        if _patch_is_planar(node):
            owner_surfaces = ((node, patch_sites, None),)
        else:
            chart_budget = requested_alpha_budget
            if not isfinite(chart_budget):
                chart_budget = _fallback_chart_alpha_budget(node)
            try:
                chart_seeds = _intrinsic_chart_site_seeds(node, patch_sites)
                charts = build_intrinsic_strip_charts(
                    node,
                    chart_seeds,
                    alpha_budget=chart_budget,
                    patch_surface=patch_surface,
                )
                charts = tuple(
                    admit_intrinsic_strip_runtime(
                        chart,
                        initial_alpha=chart_budget,
                        distortion_budget=distortion_budget,
                    )
                    for chart in charts
                )
            except ChartBuildFailure as exc:
                # Chart builder принимает patch-wide seed set. До того как
                # charts успешно разделены, локализовать отказ уже одного
                # seed нельзя: strict router обязан отклонить весь этот patch
                # component, иначе появится частично покрытый результат.
                edge_indices = tuple(
                    sorted(
                        {int(site["edge_index"]) for site in patch_sites}
                    )
                )
                rejected_edges.update(edge_indices)
                failures.append(
                    PatchVoronoiCompileFailure(
                        patch_id=int(patch_id),
                        reason=exc.code,
                        edge_indices=edge_indices,
                        details=exc.details,
                    )
                )
                if not allow_partial:
                    return PatchVoronoiCompileAttempt(
                        plan=None,
                        rejected_edge_indices=tuple(sorted(rejected_edges)),
                        failures=tuple(failures),
                    )
                continue
            owner_surfaces = tuple(
                (
                    node,
                    (
                        tuple(patch_sites)
                        if isinstance(chart, IntrinsicStripAtlas)
                        else tuple(
                            raw
                            for raw in patch_sites
                            if int(raw["edge_index"])
                            in {seed.edge_index for seed in chart.site_seeds}
                        )
                    ),
                    chart,
                )
                for chart in charts
            )
        for owner_surface, owner_sites, intrinsic_chart in owner_surfaces:
            try:
                if intrinsic_chart is None:
                    compiled_surfaces = (
                        _compile_surface(owner_surface, owner_sites, diagnostics),
                    )
                elif isinstance(intrinsic_chart, IntrinsicStripAtlas):
                    compiled_surfaces = _compile_intrinsic_atlas_surfaces(
                        owner_surface,
                        intrinsic_chart,
                        owner_sites,
                        diagnostics,
                    )
                else:
                    compiled_surfaces = (
                        _compile_intrinsic_surface(
                            owner_surface,
                            intrinsic_chart,
                            owner_sites,
                            diagnostics,
                        ),
                    )
            except _PatchVoronoiSurfaceCompileError as exc:
                edge_indices = exc.edge_indices or tuple(
                    sorted(
                        {int(site["edge_index"]) for site in owner_sites}
                    )
                )
                rejected_edges.update(edge_indices)
                failures.append(
                    PatchVoronoiCompileFailure(
                        patch_id=int(patch_id),
                        reason=exc.reason,
                        edge_indices=edge_indices,
                        details=exc.details,
                    )
                )
                if not allow_partial:
                    return PatchVoronoiCompileAttempt(
                        plan=None,
                        rejected_edge_indices=tuple(sorted(rejected_edges)),
                        failures=tuple(failures),
                    )
                continue
            except Exception as exc:
                edge_indices = tuple(
                    sorted(
                        {int(site["edge_index"]) for site in owner_sites}
                    )
                )
                rejected_edges.update(edge_indices)
                failures.append(
                    PatchVoronoiCompileFailure(
                        patch_id=int(patch_id),
                        reason="SURFACE_COMPILE_EXCEPTION",
                        edge_indices=edge_indices,
                        details=f"{type(exc).__name__}: {exc}",
                    )
                )
                if not allow_partial:
                    return PatchVoronoiCompileAttempt(
                        plan=None,
                        rejected_edge_indices=tuple(sorted(rejected_edges)),
                        failures=tuple(failures),
                    )
                continue
            if not compiled_surfaces or any(
                surface is None for surface in compiled_surfaces
            ):
                edge_indices = tuple(
                    sorted(
                        {int(site["edge_index"]) for site in owner_sites}
                    )
                )
                rejected_edges.update(edge_indices)
                failures.append(
                    PatchVoronoiCompileFailure(
                        patch_id=int(patch_id),
                        reason="SURFACE_COMPILE_FAILED",
                        edge_indices=edge_indices,
                    )
                )
                if not allow_partial:
                    return PatchVoronoiCompileAttempt(
                        plan=None,
                        rejected_edge_indices=tuple(sorted(rejected_edges)),
                        failures=tuple(failures),
                    )
                continue
            surfaces.extend(compiled_surfaces)
    lifted_vertices = {
        vert_index: _lift_position(
            positions_by_vert[vert_index], normals, float(offset)
        )
        for vert_index, normals in normals_by_vert.items()
    }
    surfaces = [
        replace(
            surface,
            corner_seeds=_compile_corner_seeds(
                surface,
                lifted_vertices,
                corner_join_mode,
            ),
        )
        for surface in surfaces
    ]
    max_lateral_lift_ratio = 0.0
    if abs(float(offset)) > _GEOMETRY_EPS:
        for surface in surfaces:
            for site in surface.sites:
                for vert_index, source in (
                    (site.vert_a, site.source_a),
                    (site.vert_b, site.source_b),
                ):
                    delta = lifted_vertices[vert_index] - source
                    lateral = delta - surface.normal * float(offset)
                    max_lateral_lift_ratio = max(
                        max_lateral_lift_ratio,
                        lateral.length / abs(float(offset)),
                    )
    plan = None
    if surfaces:
        intrinsic_budgets = tuple(
            (
                surface.domain.alpha_budget,
                surface.domain.budget_source,
            )
            for surface in surfaces
            if surface.domain.kind == "INTRINSIC"
        )
        if intrinsic_budgets:
            domain_budget, domain_budget_source = min(intrinsic_budgets)
            actual_alpha_budget = min(
                requested_alpha_budget, domain_budget
            )
            actual_budget_source = (
                domain_budget_source
                if domain_budget <= requested_alpha_budget
                else "STRIP_BUDGET"
            )
        else:
            actual_alpha_budget = float("inf")
            actual_budget_source = "FULL_CONNECTED_COMPONENT"
        if any(
            surface.domain.period > 0.0
            or surface.domain.admission_tier == "APPROXIMATE"
            for surface in surfaces
        ):
            capacity_policy = CapacityPolicy.REJECT_UNPROVEN
        elif isfinite(actual_alpha_budget):
            capacity_policy = CapacityPolicy.CONTROLLED_RECOMPILE
        else:
            capacity_policy = CapacityPolicy.SATURATE_PROVEN
        plan = PatchVoronoiPlan(
            offset=float(offset),
            surfaces=tuple(surfaces),
            lifted_vertices=lifted_vertices,
            max_lateral_lift_ratio=max_lateral_lift_ratio,
            alpha_budget=actual_alpha_budget,
            support_triangle_ids=tuple(
                tuple(
                    range(
                        len(surface.domain.intrinsic_triangles)
                        if surface.domain.intrinsic_triangles
                        else len(surface.domain.boundary_triangles)
                    )
                )
                for surface in surfaces
            ),
            budget_source=actual_budget_source,
            requested_alpha_budget=requested_alpha_budget,
            approximate_admit_count=len(
                {
                    surface.patch_id
                    for surface in surfaces
                    if surface.domain.admission_tier == "APPROXIMATE"
                }
            ),
            capacity_policy=capacity_policy,
            corner_join_mode=corner_join_mode,
        )
    return PatchVoronoiCompileAttempt(
        plan=plan,
        rejected_edge_indices=tuple(sorted(rejected_edges)),
        failures=tuple(failures),
    )


def compile_patch_voronoi_plan(
    analysis_bundle,
    selected_edge_indices,
    offset,
    *,
    diagnostics=None,
    alpha_budget=None,
    distortion_budget=CHART_DISTORTION_BUDGET,
    corner_join_mode=CornerJoinMode.MITER,
):
    """Строго компилирует все touched surfaces без смены backend."""

    return compile_patch_voronoi_attempt(
        analysis_bundle,
        selected_edge_indices,
        offset,
        allow_partial=False,
        diagnostics=diagnostics,
        alpha_budget=alpha_budget,
        distortion_budget=distortion_budget,
        corner_join_mode=corner_join_mode,
    ).plan


def _insert_surface_edge_stations(polygons, tolerance):
    """Делит edge каждой cell всеми point-on-edge вершинами surface.

    До этого соседние cells могли геометрически совпадать, но оставлять
    топологический T-контакт: вершина одной face лежала в середине цельного
    edge другой. BMesh/remove_doubles такой edge не разрезает.
    """

    quantum = max(tolerance * 0.5, 1e-10)

    def point_key(point):
        return (
            round(point[0] / quantum),
            round(point[1] / quantum),
        )

    representatives = {}
    keyed_polygons = []
    for polygon in polygons:
        keys = []
        for point in polygon:
            key = point_key(point)
            representatives.setdefault(key, (float(point[0]), float(point[1])))
            if not keys or keys[-1] != key:
                keys.append(key)
        if len(keys) > 1 and keys[0] == keys[-1]:
            keys.pop()
        keyed_polygons.append(keys)
    if not representatives:
        return [list(polygon) for polygon in polygons], 0

    all_points = list(representatives.values())
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    diagonal = sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)
    grid_size = max(
        tolerance * 4.0,
        diagonal / max(8.0, sqrt(len(representatives)) * 2.0),
    )

    def grid_key(point):
        return (
            int(point[0] // grid_size),
            int(point[1] // grid_size),
        )

    point_grid = {}
    for key, point in representatives.items():
        point_grid.setdefault(grid_key(point), []).append(key)

    rebuilt = []
    inserted_stations = 0
    for polygon_keys in keyed_polygons:
        result_keys = []
        for index, key_a in enumerate(polygon_keys):
            key_b = polygon_keys[(index + 1) % len(polygon_keys)]
            point_a = representatives[key_a]
            point_b = representatives[key_b]
            edge_length = _dist2(point_a, point_b)
            if edge_length <= _GEOMETRY_EPS:
                continue
            result_keys.append(key_a)
            min_grid_x = int((min(point_a[0], point_b[0]) - tolerance) // grid_size)
            max_grid_x = int((max(point_a[0], point_b[0]) + tolerance) // grid_size)
            min_grid_y = int((min(point_a[1], point_b[1]) - tolerance) // grid_size)
            max_grid_y = int((max(point_a[1], point_b[1]) + tolerance) // grid_size)
            candidates = set()
            for grid_x in range(min_grid_x, max_grid_x + 1):
                for grid_y in range(min_grid_y, max_grid_y + 1):
                    candidates.update(point_grid.get((grid_x, grid_y), ()))
            stations = []
            endpoint_fraction = min(0.1, tolerance / edge_length)
            for candidate_key in candidates:
                if candidate_key in (key_a, key_b):
                    continue
                candidate = representatives[candidate_key]
                distance, factor = _segment_point_distance2(
                    point_a, point_b, candidate
                )
                if (
                    distance <= tolerance
                    and endpoint_fraction < factor < 1.0 - endpoint_fraction
                ):
                    stations.append((factor, candidate_key))
            for _factor, candidate_key in sorted(stations):
                if result_keys[-1] == candidate_key:
                    continue
                result_keys.append(candidate_key)
                inserted_stations += 1
        result = [representatives[key] for key in result_keys]
        result = _dedupe_polygon(result, tolerance=quantum)
        rebuilt.append(result)
    return rebuilt, inserted_stations


def _intrinsic_boundary_segments(domain):
    quantum = max(DECAL_WELD_DISTANCE * 0.01, 1e-10)

    def point_key(point):
        return (
            round(float(point[0]) / quantum),
            round(float(point[1]) / quantum),
        )

    segments = {}
    for triangle in domain.boundary_triangles:
        for index, first in enumerate(triangle):
            second = triangle[(index + 1) % 3]
            key = tuple(sorted((point_key(first), point_key(second))))
            segments.setdefault(
                key,
                (
                    (float(first[0]), float(first[1])),
                    (float(second[0]), float(second[1])),
                ),
            )
    return tuple(segments[key] for key in sorted(segments))


def _segment_boundary_factors(first, second, boundary_a, boundary_b, tolerance):
    edge = _sub2(second, first)
    boundary = _sub2(boundary_b, boundary_a)
    denominator = _cross2(edge, boundary)
    relative = _sub2(boundary_a, first)
    factors = []
    edge_length = _dist2(first, second)
    if edge_length <= _GEOMETRY_EPS:
        return ()
    endpoint_fraction = min(0.1, tolerance / edge_length)
    if abs(denominator) > 1e-14:
        factor = _cross2(relative, boundary) / denominator
        boundary_factor = _cross2(relative, edge) / denominator
        if (
            endpoint_fraction < factor < 1.0 - endpoint_fraction
            and -1e-9 <= boundary_factor <= 1.0 + 1e-9
        ):
            factors.append(float(factor))
        return tuple(factors)

    if abs(_cross2(relative, edge)) > tolerance * edge_length:
        return ()
    for candidate in (boundary_a, boundary_b):
        distance, factor = _segment_point_distance2(first, second, candidate)
        if (
            distance <= tolerance
            and endpoint_fraction < factor < 1.0 - endpoint_fraction
        ):
            factors.append(float(factor))
    return tuple(sorted(set(factors)))


def _insert_intrinsic_triangle_stations(polygons, domain, tolerance):
    """Вставляет exact stations на crossings source triangle boundaries."""

    segments = _intrinsic_boundary_segments(domain)
    rebuilt = []
    inserted = 0
    for polygon in polygons:
        result = []
        for index, first in enumerate(polygon):
            second = polygon[(index + 1) % len(polygon)]
            result.append((float(first[0]), float(first[1])))
            factors = set()
            for boundary_a, boundary_b in segments:
                factors.update(
                    _segment_boundary_factors(
                        first,
                        second,
                        boundary_a,
                        boundary_b,
                        tolerance,
                    )
                )
            for factor in sorted(factors):
                point = (
                    first[0] + (second[0] - first[0]) * factor,
                    first[1] + (second[1] - first[1]) * factor,
                )
                if _dist2(result[-1], point) <= tolerance * 0.5:
                    continue
                result.append(point)
                inserted += 1
        rebuilt.append(_dedupe_polygon(result, tolerance=tolerance * 0.5))
    return rebuilt, inserted


def _m1_point_in_polygon(point, polygon, tolerance=1e-10):
    inside = False
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        distance, _factor = _segment_point_distance2(first, second, point)
        if distance <= tolerance:
            return True
        if (first[1] > point[1]) == (second[1] > point[1]):
            continue
        crossing_x = first[0] + (
            (second[0] - first[0])
            * (point[1] - first[1])
            / (second[1] - first[1])
        )
        if crossing_x > point[0]:
            inside = not inside
    return inside


def _m1_representative_point(polygon):
    area2 = _polygon_area2(polygon)
    if abs(area2) > 1e-18:
        x_value = 0.0
        y_value = 0.0
        for index, first in enumerate(polygon):
            second = polygon[(index + 1) % len(polygon)]
            cross = first[0] * second[1] - second[0] * first[1]
            x_value += (first[0] + second[0]) * cross
            y_value += (first[1] + second[1]) * cross
        centroid = (x_value / (3.0 * area2), y_value / (3.0 * area2))
        if _m1_point_in_polygon(centroid, polygon):
            return centroid
    anchor = polygon[0]
    for index in range(1, len(polygon) - 1):
        candidate = (
            (anchor[0] + polygon[index][0] + polygon[index + 1][0]) / 3.0,
            (anchor[1] + polygon[index][1] + polygon[index + 1][1]) / 3.0,
        )
        if _m1_point_in_polygon(candidate, polygon):
            return candidate
    return polygon[0]


def _m1_surface_curve_segments(entries):
    """Возвращает все 2D-кривые, которые обязаны объявить T-пересечения."""

    surface = entries[0][1].surface
    polygons = [
        tuple((float(point[0]), float(point[1])) for point in face.crop.points)
        for _index, face in entries
    ]
    polygons.extend(
        tuple((float(point[0]), float(point[1])) for point in fragment)
        for atom in surface.atoms
        for fragment in atom.fragments
    )
    polygons.extend(
        tuple((float(point[0]), float(point[1])) for point in triangle)
        for triangle in (
            tuple(
                item.chart_points
                for item in surface.domain.intrinsic_triangles
            )
            if surface.domain.intrinsic_triangles
            else surface.domain.boundary_triangles
        )
    )
    return tuple(
        (polygon[index], polygon[(index + 1) % len(polygon)])
        for polygon in polygons
        if len(polygon) >= 2
        for index in range(len(polygon))
        if polygon[index] != polygon[(index + 1) % len(polygon)]
    )


def _m1_exact_segment_intersection(first, second, edge_a, edge_b):
    """Exact intersection: (segment factor, edge factor, point) или marker."""

    direction = (second[0] - first[0], second[1] - first[1])
    edge_direction = (edge_b[0] - edge_a[0], edge_b[1] - edge_a[1])

    def cross(first_vector, second_vector):
        return (
            first_vector[0] * second_vector[1]
            - first_vector[1] * second_vector[0]
        )

    denominator = cross(direction, edge_direction)
    relative = (edge_a[0] - first[0], edge_a[1] - first[1])
    if denominator == 0:
        if cross(relative, direction) == 0:
            return "COLLINEAR"
        return None
    segment_factor = cross(relative, edge_direction) / denominator
    edge_factor = cross(relative, direction) / denominator
    if not 0 <= segment_factor <= 1 or not 0 <= edge_factor <= 1:
        return None
    point = (
        first[0] + direction[0] * segment_factor,
        first[1] + direction[1] * segment_factor,
    )
    return segment_factor, edge_factor, point


def _m1_collinear_overlap(first, second, edge_a, edge_b):
    """Exact extent overlap двух уже известных collinear segments.

    Возвращает факторы границ overlap одновременно на локальной кривой и
    каноническом transition. Совпадение бесконечных прямых без общего extent
    возвращает ``None`` и не даёт права удалять локальную кривую.
    """

    edge_direction = (edge_b[0] - edge_a[0], edge_b[1] - edge_a[1])
    if edge_direction == (0, 0) or first == second:
        return None
    axis = 0 if abs(edge_direction[0]) >= abs(edge_direction[1]) else 1
    edge_delta = edge_direction[axis]
    first_edge_factor = (first[axis] - edge_a[axis]) / edge_delta
    second_edge_factor = (second[axis] - edge_a[axis]) / edge_delta
    edge_start = max(Fraction(0), min(first_edge_factor, second_edge_factor))
    edge_end = min(Fraction(1), max(first_edge_factor, second_edge_factor))
    if edge_start > edge_end:
        return None

    segment_delta = second[axis] - first[axis]
    if segment_delta == 0:
        return None

    def segment_factor(edge_factor):
        coordinate = edge_a[axis] + edge_delta * edge_factor
        return (coordinate - first[axis]) / segment_delta

    segment_start = segment_factor(edge_start)
    segment_end = segment_factor(edge_end)
    return (
        min(segment_start, segment_end),
        max(segment_start, segment_end),
        edge_start,
        edge_end,
    )


def _m1_crop_semantic_class(crop):
    if crop.kind == "SEGMENT":
        return ("SEGMENT",)
    return ("CORNER", crop.kind, crop.semantic_owner_id)


def _m1_canonicalize_transition_intervals(intervals, diagnostics=None):
    """T7-P3.7: поглощает T2-интервалы, неразличимые на B0-решётке."""

    canonical = [list(interval) for interval in intervals]
    while True:
        degenerate_index = next(
            (
                index
                for index, interval in enumerate(canonical)
                if interval[2] is not None
                and interval[1] - interval[0] <= 1
            ),
            None,
        )
        if degenerate_index is None:
            break
        if len(canonical) == 1:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: transition is shorter than "
                "the canonical quantum"
            )

        current = canonical[degenerate_index]
        previous = (
            canonical[degenerate_index - 1]
            if degenerate_index > 0
            else None
        )
        following = (
            canonical[degenerate_index + 1]
            if degenerate_index + 1 < len(canonical)
            else None
        )
        if previous is None:
            merge_into_previous = False
        elif following is None:
            merge_into_previous = True
        else:
            previous_same_class = previous[3] == current[3]
            following_same_class = following[3] == current[3]
            neighbors_same_class = previous[3] == following[3]
            if neighbors_same_class:
                # Поглощённый класс между эквивалентными соседями не
                # создаёт две semantic смены; левый interval каноничен.
                merge_into_previous = True
            elif not previous_same_class and not following_same_class:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: two semantic class changes "
                    "fall inside one transition quantum"
                )
            else:
                merge_into_previous = previous_same_class

        if merge_into_previous:
            previous[1] = current[1]
        else:
            following[0] = current[0]
        del canonical[degenerate_index]
        if diagnostics is not None:
            diagnostics.atlas_degenerate_interval_merge_count += 1

    for first, second, owner, _semantic in canonical:
        if owner is not None and second - first <= 1:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: degenerate interval escaped "
                "T2 canonicalization"
            )
    return tuple(tuple(interval) for interval in canonical)


def _m1_anchor_coverage_boundaries(intervals, frontier_stations_by_token):
    """T2-C.7: coverage election и frontier geometry читают один T-key."""

    anchored = [list(interval) for interval in intervals]
    for index in range(len(anchored) - 1):
        previous = anchored[index]
        following = anchored[index + 1]
        if previous[1] != following[0]:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: non-contiguous T2 intervals"
            )
        previous_covered = previous[2] is not None
        following_covered = following[2] is not None
        if previous_covered == following_covered:
            continue
        covered_owner = previous[2] if previous_covered else following[2]
        declared_station = int(previous[1])
        candidates = tuple(
            sorted(frontier_stations_by_token.get(int(covered_owner), ()))
        )
        if not candidates:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: coverage frontier has no "
                "canonical station"
            )
        canonical_station = min(
            candidates,
            key=lambda station: (
                abs(int(station) - declared_station),
                int(station),
            ),
        )
        if abs(int(canonical_station) - declared_station) > 1:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: coverage declaration misses "
                "its frontier station "
                f"declared={declared_station} "
                f"canonical={canonical_station} owner={covered_owner!r}"
            )
        previous[1] = int(canonical_station)
        following[0] = int(canonical_station)
        if previous[0] >= previous[1] or following[0] >= following[1]:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: coverage anchor collapses interval"
            )
    return tuple(tuple(interval) for interval in anchored)


def _m1_build_transition_contract(
    groups,
    semantic_imports=None,
    diagnostics=None,
):
    """T1/T2: single-source 1D station set до локальных arrangements."""

    semantic_imports = semantic_imports or {}

    def published_anchor_stations(surface, transition_key):
        transition_id = _hashable_provenance(transition_key)
        return {
            int(station)
            for semantic in semantic_imports.get(id(surface.domain), ())
            for _endpoint_index, anchor_key, station
            in semantic.anchor_stations
            if _hashable_provenance(anchor_key) == transition_id
        }

    def curve_segments(surface, entries):
        yield from _m1_surface_curve_segments(entries)
        for semantic in semantic_imports.get(id(surface.domain), ()):
            if len(semantic.points) < 2:
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: imported semantic curve is not open"
                )
            yield from zip(semantic.points, semantic.points[1:])

    def imported_candidates(
        surface, midpoint, segment=None, factor_fraction=None
    ):
        """T2-C: R1/T7 images участвуют в coverage до arrangement."""

        for import_index, semantic in enumerate(
            semantic_imports.get(id(surface.domain), ())
        ):
            owner_point = _affine_point(semantic.local_to_owner, midpoint)
            if segment is not None and factor_fraction is not None:
                quantum = max(
                    float(surface.diagram_transform.quantum), 1e-10
                )
                edge_a = tuple(
                    Fraction(round(value / quantum))
                    for value in segment[0]
                )
                edge_b = tuple(
                    Fraction(round(value / quantum))
                    for value in segment[1]
                )
                target_point = (
                    edge_a[0]
                    + (edge_b[0] - edge_a[0]) * factor_fraction,
                    edge_a[1]
                    + (edge_b[1] - edge_a[1]) * factor_fraction,
                )
                target_region_points = semantic.target_region_points
                if not target_region_points:
                    owner_to_target = _affine_inverse(
                        semantic.local_to_owner
                    )
                    target_region_points = tuple(
                        _affine_point(owner_to_target, vertex)
                        for vertex in semantic.owner_region_points
                    )
                target_region = tuple(
                    (
                        Fraction(round(vertex[0] / quantum)),
                        Fraction(round(vertex[1] / quantum)),
                    )
                    for vertex in target_region_points
                )
                inside = _m1_point_in_polygon(
                    target_point, target_region, tolerance=0.0
                )
            else:
                inside = (
                    _m1_point_in_polygon(
                        midpoint, semantic.target_region_points
                    )
                    if semantic.target_region_points
                    else _m1_point_in_polygon(
                        owner_point, semantic.owner_region_points
                    )
                )
            priority = (
                3
                if semantic.owner_crop.kind == "JUNCTION"
                else 1
                if semantic.owner_crop.kind == "SEGMENT"
                else 2
            )
            distance = _segment_point_distance2(
                semantic.owner_site.point_a,
                semantic.owner_site.point_b,
                owner_point,
            )[0]
            yield (
                inside,
                priority,
                distance,
                import_index,
                semantic,
            )

    def local_crop_contains(surface, segment, crop_points, factor):
        """T2-C.7: election читает ту же B0-геометрию, что arrangement."""

        quantum = max(float(surface.diagram_transform.quantum), 1e-10)
        edge_a = tuple(
            Fraction(round(value / quantum)) for value in segment[0]
        )
        edge_b = tuple(
            Fraction(round(value / quantum)) for value in segment[1]
        )
        point = (
            edge_a[0] + (edge_b[0] - edge_a[0]) * factor,
            edge_a[1] + (edge_b[1] - edge_a[1]) * factor,
        )
        polygon = tuple(
            (
                Fraction(round(vertex[0] / quantum)),
                Fraction(round(vertex[1] / quantum)),
            )
            for vertex in crop_points
        )
        return _m1_point_in_polygon(point, polygon, tolerance=0.0)

    declarations = {}
    for entries in groups:
        surface = entries[0][1].surface
        if surface.domain.admission_tier != "APPROXIMATE":
            continue
        for local_key, segment in _m1_atlas_transition_segments(
            surface.domain
        ).items():
            transition_key = local_key[0]
            declarations.setdefault(transition_key, []).append(
                (surface, entries, segment)
            )

    sides_by_domain = {}
    for transition_key in sorted(declarations, key=repr):
        declared_sides = sorted(
            declarations[transition_key],
            key=lambda item: item[0].domain.chart_id,
        )
        if len(declared_sides) == 1:
            # У одиночной materialized стороны нет законной второй half-curve.
            # Вставляем T как boundary, но помечаем все интервалы непокрытыми:
            # это отсекает margin/site-image артефакт вместо отдельного острова.
            surface, entries, segment = declared_sides[0]
            quantum = max(
                float(surface.diagram_transform.quantum), 1e-10
            )
            station_extent = max(
                1,
                round(_dist2(segment[0], segment[1]) / quantum),
            )
            stations = {
                0,
                station_extent,
                *published_anchor_stations(surface, transition_key),
            }
            edge_a = tuple(
                Fraction(round(value / quantum)) for value in segment[0]
            )
            edge_b = tuple(
                Fraction(round(value / quantum)) for value in segment[1]
            )
            for first, second in curve_segments(surface, entries):
                lattice_first = tuple(
                    Fraction(round(value / quantum)) for value in first
                )
                lattice_second = tuple(
                    Fraction(round(value / quantum)) for value in second
                )
                hit = _m1_exact_segment_intersection(
                    lattice_first, lattice_second, edge_a, edge_b
                )
                if hit is None:
                    continue
                if hit == "COLLINEAR":
                    overlap = _m1_collinear_overlap(
                        lattice_first, lattice_second, edge_a, edge_b
                    )
                    if overlap is not None:
                        for edge_factor in overlap[2:]:
                            station = round(
                                float(edge_factor) * station_extent
                            )
                            stations.add(
                                max(0, min(station_extent, station))
                            )
                    continue
                station = round(float(hit[1]) * station_extent)
                stations.add(max(0, min(station_extent, station)))
            ordered_stations = tuple(sorted(stations))
            canonical_intervals = _m1_canonicalize_transition_intervals(
                tuple(
                    (first, second, None, None)
                    for first, second in zip(
                        ordered_stations, ordered_stations[1:]
                    )
                ),
                diagnostics,
            )
            interior_vertices = [
                triangle.chart_points[local_edge]
                for triangle in surface.domain.intrinsic_triangles
                for local_edge, key in enumerate(
                    triangle.edge_transition_keys
                )
                if key == transition_key
            ]
            if len(interior_vertices) != 1:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: transition interior is ambiguous"
                )
            direction = (
                segment[1][0] - segment[0][0],
                segment[1][1] - segment[0][1],
            )
            relative = (
                interior_vertices[0][0] - segment[0][0],
                interior_vertices[0][1] - segment[0][1],
            )
            interior_cross = (
                direction[0] * relative[1]
                - direction[1] * relative[0]
            )
            sides_by_domain.setdefault(id(surface.domain), []).append(
                _M1TransitionSide(
                    transition_key=transition_key,
                    chart_id=surface.domain.chart_id,
                    segment=tuple(segment),
                    station_extent=station_extent,
                    stations=ordered_stations,
                    interior_sign=1 if interior_cross > 0.0 else -1,
                    interval_owners=tuple(
                        (first, second, None)
                        for first, second, _owner, _semantic
                        in canonical_intervals
                    ),
                    interval_semantics=tuple(
                        (first, second, None)
                        for first, second, _owner, _semantic
                        in canonical_intervals
                    ),
                    single_declared_side=True,
                )
            )
            continue
        if len(declared_sides) != 2:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: transition must have two sides"
            )
        owner_chart_id = int(transition_key[3])
        owner_matches = tuple(
            item
            for item in declared_sides
            if item[0].domain.chart_id == owner_chart_id
        )
        if len(owner_matches) != 1:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: canonical owner is missing"
            )
        owner_surface, _owner_entries, owner_segment = owner_matches[0]
        owner_quantum = max(
            float(owner_surface.diagram_transform.quantum), 1e-10
        )
        station_extent = max(
            1,
            round(
                _dist2(owner_segment[0], owner_segment[1])
                / owner_quantum
            ),
        )
        stations = {0, station_extent}
        for surface, _entries, _segment in declared_sides:
            stations.update(
                published_anchor_stations(surface, transition_key)
            )
        frontier_stations_by_token = {}
        for surface, entries, segment in declared_sides:
            quantum = max(float(surface.diagram_transform.quantum), 1e-10)
            edge_a = (
                Fraction(round(segment[0][0] / quantum)),
                Fraction(round(segment[0][1] / quantum)),
            )
            edge_b = (
                Fraction(round(segment[1][0] / quantum)),
                Fraction(round(segment[1][1] / quantum)),
            )
            for _pending_index, pending_face in entries:
                crop_points = tuple(pending_face.crop.points)
                for first, second in zip(
                    crop_points, crop_points[1:] + crop_points[:1]
                ):
                    lattice_first = (
                        Fraction(round(first[0] / quantum)),
                        Fraction(round(first[1] / quantum)),
                    )
                    lattice_second = (
                        Fraction(round(second[0] / quantum)),
                        Fraction(round(second[1] / quantum)),
                    )
                    hit = _m1_exact_segment_intersection(
                        lattice_first, lattice_second, edge_a, edge_b
                    )
                    if hit is None:
                        continue
                    edge_factors = ()
                    if hit == "COLLINEAR":
                        overlap = _m1_collinear_overlap(
                            lattice_first,
                            lattice_second,
                            edge_a,
                            edge_b,
                        )
                        if overlap is not None:
                            edge_factors = overlap[2:]
                    else:
                        edge_factors = (hit[1],)
                    for edge_factor in edge_factors:
                        frontier_station = max(
                            0,
                            min(
                                station_extent,
                                round(float(edge_factor) * station_extent),
                            ),
                        )
                        frontier_stations_by_token.setdefault(
                            int(pending_face.site.edge_index), set()
                        ).add(frontier_station)
            for first, second in curve_segments(surface, entries):
                lattice_first = (
                    Fraction(round(first[0] / quantum)),
                    Fraction(round(first[1] / quantum)),
                )
                lattice_second = (
                    Fraction(round(second[0] / quantum)),
                    Fraction(round(second[1] / quantum)),
                )
                hit = _m1_exact_segment_intersection(
                    lattice_first, lattice_second, edge_a, edge_b
                )
                if hit is None:
                    continue
                if hit == "COLLINEAR":
                    overlap = _m1_collinear_overlap(
                        lattice_first, lattice_second, edge_a, edge_b
                    )
                    if overlap is not None:
                        for edge_factor in overlap[2:]:
                            station = round(
                                float(edge_factor) * station_extent
                            )
                            stations.add(
                                max(0, min(station_extent, station))
                            )
                    continue
                _segment_factor, edge_factor, _point = hit
                station = round(float(edge_factor) * station_extent)
                stations.add(max(0, min(station_extent, station)))
        ordered_stations = tuple(sorted(stations))
        canonical_coverage = []
        canonical_interval_owners = []
        canonical_interval_semantics = []
        for first_station, second_station in zip(
            ordered_stations, ordered_stations[1:]
        ):
            factor_fraction = Fraction(
                first_station + second_station,
                2 * station_extent,
            )
            factor = float(factor_fraction)
            side_coverage = []
            # T3: обе half-curves объявляются owner'у. Покрытие интервала
            # существует, если его объявила хотя бы одна локальная сторона;
            # решение всё равно принимается здесь один раз и раздаётся обеим.
            for _surface, candidate_entries, candidate_segment in declared_sides:
                midpoint = (
                    candidate_segment[0][0]
                    + (candidate_segment[1][0] - candidate_segment[0][0])
                    * factor,
                    candidate_segment[0][1]
                    + (candidate_segment[1][1] - candidate_segment[0][1])
                    * factor,
                )
                local_coverage = any(
                    local_crop_contains(
                        _surface,
                        candidate_segment,
                        pending_face.crop.points,
                        factor_fraction,
                    )
                    for _pending_index, pending_face in candidate_entries
                )
                image_coverage = any(
                    inside
                    for inside, _priority, _distance, _index, _semantic
                    in imported_candidates(
                        _surface,
                        midpoint,
                        candidate_segment,
                        factor_fraction,
                    )
                )
                side_coverage.append(local_coverage or image_coverage)
            canonical_coverage.append(any(side_coverage))
            canonical_candidates = []
            if canonical_coverage[-1]:
                for (
                    candidate_surface,
                    candidate_entries,
                    candidate_segment,
                ) in declared_sides:
                    midpoint = (
                        candidate_segment[0][0]
                        + (
                            candidate_segment[1][0]
                            - candidate_segment[0][0]
                        )
                        * factor,
                        candidate_segment[0][1]
                        + (
                            candidate_segment[1][1]
                            - candidate_segment[0][1]
                        )
                        * factor,
                    )
                    for pending_index, pending_face in candidate_entries:
                        semantic_owner_id = (
                            pending_face.crop.semantic_owner_id
                        )
                        if (
                            pending_face.crop.kind != "SEGMENT"
                            and isinstance(semantic_owner_id, tuple)
                            and semantic_owner_id[:1] == ("corner",)
                        ):
                            declared_owner_chart = dict(
                                candidate_surface.semantic_owner_chart_by_vertex
                            ).get(int(semantic_owner_id[1]))
                            if (
                                declared_owner_chart is not None
                                and int(declared_owner_chart)
                                != int(candidate_surface.domain.chart_id)
                            ):
                                # T7: сосед не пере-выводит corner-класс из
                                # своей локальной копии crop; он потребляет
                                # только owner image ниже.
                                continue
                        inside = local_crop_contains(
                            candidate_surface,
                            candidate_segment,
                            pending_face.crop.points,
                            factor_fraction,
                        )
                        priority = (
                            3
                            if pending_face.crop.kind == "JUNCTION"
                            else 1
                            if pending_face.crop.kind == "SEGMENT"
                            else 2
                        )
                        semantic_class = _m1_crop_semantic_class(
                            pending_face.crop
                        )
                        distance = _segment_point_distance2(
                            pending_face.site.point_a,
                            pending_face.site.point_b,
                            midpoint,
                        )[0]
                        canonical_candidates.append(
                            (
                                0 if inside else 1,
                                -priority,
                                round(distance, 12),
                                pending_face.crop.kind,
                                pending_face.crop.side,
                                int(pending_face.site.edge_index),
                                candidate_surface.domain.chart_id,
                                pending_index,
                                semantic_class,
                            )
                        )
                    for (
                        inside,
                        priority,
                        distance,
                        import_index,
                        semantic,
                    ) in imported_candidates(
                        candidate_surface,
                        midpoint,
                        candidate_segment,
                        factor_fraction,
                    ):
                        canonical_candidates.append(
                            (
                                0 if inside else 1,
                                -priority,
                                round(distance, 12),
                                semantic.owner_crop.kind,
                                semantic.owner_crop.side,
                                int(semantic.owner_site.edge_index),
                                int(semantic.owner_chart_id),
                                len(candidate_entries) + import_index,
                                _m1_crop_semantic_class(
                                    semantic.owner_crop
                                ),
                            )
                        )
            canonical_candidate = (
                min(canonical_candidates)
                if canonical_candidates
                else None
            )
            canonical_interval_owners.append(
                int(canonical_candidate[5])
                if canonical_candidate is not None
                else None
            )
            canonical_interval_semantics.append(
                canonical_candidate[8]
                if canonical_candidate is not None
                else None
            )
        anchored_intervals = _m1_anchor_coverage_boundaries(
            tuple(
                (
                    first,
                    second,
                    owner,
                    semantic_class,
                )
                for (first, second), owner, semantic_class in zip(
                    zip(ordered_stations, ordered_stations[1:]),
                    canonical_interval_owners,
                    canonical_interval_semantics,
                )
            ),
            frontier_stations_by_token,
        )
        canonical_intervals = _m1_canonicalize_transition_intervals(
            anchored_intervals,
            diagnostics,
        )
        for surface, local_entries, segment in declared_sides:
            interval_owners = []
            for (
                first_station,
                second_station,
                local_owner,
                _semantic_class,
            ) in canonical_intervals:
                interval_owners.append(
                    (first_station, second_station, local_owner)
                )
            interior_vertices = []
            for triangle in surface.domain.intrinsic_triangles:
                for local_edge, key in enumerate(
                    triangle.edge_transition_keys
                ):
                    if key == transition_key:
                        interior_vertices.append(
                            triangle.chart_points[local_edge]
                        )
            if len(interior_vertices) != 1:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: transition interior is ambiguous"
                )
            direction = (
                segment[1][0] - segment[0][0],
                segment[1][1] - segment[0][1],
            )
            relative = (
                interior_vertices[0][0] - segment[0][0],
                interior_vertices[0][1] - segment[0][1],
            )
            interior_cross = (
                direction[0] * relative[1]
                - direction[1] * relative[0]
            )
            if abs(interior_cross) <= 1e-12:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: degenerate transition interior"
                )
            sides_by_domain.setdefault(id(surface.domain), []).append(
                _M1TransitionSide(
                    transition_key=transition_key,
                    chart_id=surface.domain.chart_id,
                    segment=tuple(segment),
                    station_extent=station_extent,
                    stations=ordered_stations,
                    interior_sign=1 if interior_cross > 0.0 else -1,
                    interval_owners=tuple(interval_owners),
                    interval_semantics=tuple(
                        (first, second, semantic_class)
                        for first, second, _owner, semantic_class
                        in canonical_intervals
                    ),
                )
            )
    return {
        domain_id: tuple(sorted(sides, key=lambda side: repr(side.transition_key)))
        for domain_id, sides in sides_by_domain.items()
    }


def _m1_crop_structure_absorbed(points, quantum):
    """True, если B0 crop не содержит ни одной interior lattice point."""

    keys = tuple(
        (
            round(float(point[0]) / quantum),
            round(float(point[1]) / quantum),
        )
        for point in points
    )
    unique_keys = tuple(dict.fromkeys(keys))
    area2 = sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(keys, keys[1:] + keys[:1])
    )
    boundary_points = sum(
        gcd(
            abs(second[0] - first[0]),
            abs(second[1] - first[1]),
        )
        for first, second in zip(keys, keys[1:] + keys[:1])
    )
    # Pick: 2*I = |2A| - B + 2. I==0 не может дать face graph.
    return (
        len(unique_keys) < 3
        or area2 == 0
        or abs(area2) - boundary_points + 2 <= 0
    )


def _m1_prepare_absorbed_corner_structures(entries):
    """T7-P3.8: оставляет absorbed boundary без corner ownership."""

    surface = entries[0][1].surface
    quantum = max(float(surface.diagram_transform.quantum), 1e-10)
    absorbed_vertices_by_edge = {}
    kept = []
    for pending_index, pending_face in entries:
        owner_id = pending_face.crop.semantic_owner_id
        is_corner = (
            pending_face.crop.kind != "SEGMENT"
            and isinstance(owner_id, tuple)
            and owner_id[:1] == ("corner",)
        )
        if not is_corner:
            kept.append((pending_index, pending_face))
            continue
        absorbed = _m1_crop_structure_absorbed(
            pending_face.crop.points, quantum
        )
        if not absorbed:
            kept.append((pending_index, pending_face))
            continue
        corner_vertex = int(owner_id[1])
        corner = next(
            (
                candidate
                for candidate in surface.corners
                if candidate.vert_index == corner_vertex
            ),
            None,
        )
        if corner is None:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: absorbed corner is missing"
            )
        owner_site_indices = (
            pending_face.crop.owner_site_indices or corner.incident_sites
        )
        for site_index in owner_site_indices:
            edge_index = int(surface.sites[site_index].edge_index)
            absorbed_vertices_by_edge.setdefault(edge_index, set()).add(
                corner_vertex
            )
        # Boundary остаётся инертной conformity-кривой graph; T2 и
        # materialization видят chain-continuous SEGMENT, не corner matter.
        kept.append(
            (
                pending_index,
                replace(
                    pending_face,
                    crop=replace(
                        pending_face.crop,
                        kind="SEGMENT",
                        side="",
                        semantic_owner_id=None,
                    ),
                ),
            )
        )

    result = []
    for pending_index, pending_face in kept:
        if pending_face.crop.kind == "SEGMENT":
            pending_face = replace(
                pending_face,
                absorbed_corner_vertices=tuple(
                    sorted(
                        set(pending_face.absorbed_corner_vertices)
                        | absorbed_vertices_by_edge.get(
                            int(pending_face.site.edge_index), set()
                        )
                    )
                ),
            )
        result.append((pending_index, pending_face))
    return result


def _m1_surface_arrangement(
    entries,
    transition_sides=(),
    diagnostics=None,
    semantic_imports=(),
    uv_frame_delegates=None,
):
    """E4: exact rational half-edge arrangement на B0 integer lattice."""

    surface = entries[0][1].surface
    uv_frame_delegates = uv_frame_delegates or {}
    quantum = max(float(surface.diagram_transform.quantum), 1e-10)
    representatives = {}

    def lattice_point(point, prefer=False):
        key = (
            round(float(point[0]) / quantum),
            round(float(point[1]) / quantum),
        )
        candidate = (Fraction(key[0]), Fraction(key[1]))
        if prefer or key not in representatives:
            representatives[key] = (float(point[0]), float(point[1]))
        return candidate

    predicates = {}
    raw_segments = []
    semantic_segment_keys = set()
    raw_corner_boundary_vertices = {}
    raw_crop_boundary_ids = {}
    raw_semantic_boundary_ids = {}
    expected_crop_boundary_ids = set()

    def polygon_lattice_keys(points, prefer=False):
        return tuple(
            lattice_point(point, prefer=prefer) for point in points
        )

    def append_polygon_segments(keys, corner_vertices=(), crop_ids=()):
        for index, first in enumerate(keys):
            second = keys[(index + 1) % len(keys)]
            if first != second:
                raw_segments.append((first, second))
                if corner_vertices:
                    raw_corner_boundary_vertices.setdefault(
                        tuple(sorted((first, second))), set()
                    ).update(corner_vertices)
                if crop_ids:
                    raw_crop_boundary_ids.setdefault(
                        tuple(sorted((first, second))), set()
                    ).update(crop_ids)

    def add_polygon(points, prefer=False):
        keys = polygon_lattice_keys(points, prefer=prefer)
        append_polygon_segments(keys)
        return keys

    for pending_index, pending_face in entries:
        crop_points = tuple(
            (float(point[0]), float(point[1]))
            for point in pending_face.crop.points
        )
        crop_keys = polygon_lattice_keys(crop_points)
        semantic_owner_id = pending_face.crop.semantic_owner_id
        corner_vertices = (
            (int(semantic_owner_id[1]),)
            if (
                pending_face.crop.kind != "SEGMENT"
                and isinstance(semantic_owner_id, tuple)
                and semantic_owner_id[:1] == ("corner",)
            )
            else ()
        )
        crop_id = (
            int(pending_index),
            pending_face.crop.kind,
            pending_face.crop.side,
            int(pending_face.site.edge_index),
            _hashable_provenance(pending_face.crop.semantic_owner_id),
        )
        crop_area2 = sum(
            first[0] * second[1] - second[0] * first[1]
            for first, second in zip(
                crop_keys, crop_keys[1:] + crop_keys[:1]
            )
        )
        absorbed_structure = (
            bool(pending_face.absorbed_corner_vertices)
            and _m1_crop_structure_absorbed(crop_points, quantum)
        )
        if (
            len(set(crop_keys)) >= 3
            and crop_area2 != 0
            and not absorbed_structure
        ):
            expected_crop_boundary_ids.add(crop_id)
        append_polygon_segments(
            crop_keys, corner_vertices, (crop_id,)
        )
        predicate_key = (
            pending_face.crop.kind,
            pending_face.crop.side,
            int(pending_face.site.edge_index),
            crop_keys,
        )
        predicates.setdefault(
            predicate_key, (pending_index, pending_face, crop_keys)
        )

    # Compile Voronoi boundaries are part of the same graph, but no longer
    # carry ownership themselves.
    for atom in surface.atoms:
        for fragment in atom.fragments:
            add_polygon(fragment)

    domain_triangles = (
        tuple(
            triangle.chart_points
            for triangle in surface.domain.intrinsic_triangles
        )
        if surface.domain.intrinsic_triangles
        else surface.domain.boundary_triangles
    )
    for triangle in domain_triangles:
        add_polygon(triangle, prefer=True)

    imported_curves = []
    for semantic in semantic_imports:
        if len(semantic.points) < 2:
            raise ValueError(
                "ATLAS_CLASS_DESYNC: imported semantic curve is not open"
            )
        curve = list(lattice_point(point) for point in semantic.points)
        for endpoint_index, transition_key, station in semantic.anchor_stations:
            side = next(
                (
                    candidate
                    for candidate in transition_sides
                    if candidate.transition_key == transition_key
                ),
                None,
            )
            if side is None or not 0 <= endpoint_index < len(curve):
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: imported semantic anchor is missing"
                )
            edge_a = tuple(
                Fraction(round(value / quantum))
                for value in side.segment[0]
            )
            edge_b = tuple(
                Fraction(round(value / quantum))
                for value in side.segment[1]
            )
            factor = Fraction(station, side.station_extent)
            canonical_point = (
                edge_a[0] + (edge_b[0] - edge_a[0]) * factor,
                edge_a[1] + (edge_b[1] - edge_a[1]) * factor,
            )
            if (
                semantic.is_endpoint_separator
                and endpoint_index in (0, len(curve) - 1)
            ):
                # T7-P3.11e: endpoint уже обрезанной predicate-кривой
                # переносится в общий source-vertex key субквантово;
                # остальная compile-статичная кривая не сдвигается.
                curve[endpoint_index] = canonical_point
            elif (
                semantic.is_predicate_separator
                and endpoint_index in (0, len(curve) - 1)
            ):
                # T3 переносит supporting curve, не вращает её вокруг
                # дальнего конца: иначе compile-static predicate-line
                # перестаёт быть своей опорной прямой и рождает sliver.
                delta = (
                    canonical_point[0] - curve[endpoint_index][0],
                    canonical_point[1] - curve[endpoint_index][1],
                )
                curve = [
                    (point[0] + delta[0], point[1] + delta[1])
                    for point in curve
                ]
            else:
                curve[endpoint_index] = canonical_point
        for first, second in zip(curve, curve[1:]):
            if first == second:
                continue
            segment = (first, second)
            raw_segments.append(segment)
            imported_curves.append((semantic, segment))
            semantic_id = (
                int(semantic.owner_chart_id),
                int(semantic.target_chart_id),
                tuple(int(value) for value in semantic.owner_edge_indices),
                semantic.owner_crop.kind,
                semantic.owner_crop.side,
                _hashable_provenance(
                    semantic.owner_crop.semantic_owner_id
                ),
                bool(semantic.is_predicate_separator),
                bool(semantic.is_endpoint_separator),
                _hashable_provenance(semantic.transition_key),
                tuple(
                    (
                        int(endpoint_index),
                        _hashable_provenance(anchor_key),
                        int(station),
                    )
                    for endpoint_index, anchor_key, station
                    in semantic.anchor_stations
                ),
            )
            raw_semantic_boundary_ids.setdefault(
                tuple(sorted(segment)), set()
            ).add(semantic_id)
            semantic_owner_id = semantic.owner_crop.semantic_owner_id
            if (
                semantic.owner_crop.kind != "SEGMENT"
                and isinstance(semantic_owner_id, tuple)
                and semantic_owner_id[:1] == ("corner",)
            ):
                raw_corner_boundary_vertices.setdefault(
                    tuple(sorted(segment)), set()
                ).add(int(semantic_owner_id[1]))
    semantic_segment_keys.update(
        tuple(sorted(curve)) for _semantic, curve in imported_curves
    )

    # T2/T3: все локальные кривые сначала режутся на едином 1D station set.
    # После этого локальный half-edge tracer уже не имеет права изобретать
    # дополнительные точки на transition.
    transition_point_keys = {}
    transition_records = []
    for side in transition_sides:
        edge_a = (
            Fraction(round(side.segment[0][0] / quantum)),
            Fraction(round(side.segment[0][1] / quantum)),
        )
        edge_b = (
            Fraction(round(side.segment[1][0] / quantum)),
            Fraction(round(side.segment[1][1] / quantum)),
        )
        station_points = {}
        source_edge = tuple(side.transition_key[2])
        for station in side.stations:
            factor = Fraction(station, side.station_extent)
            point = (
                edge_a[0] + (edge_b[0] - edge_a[0]) * factor,
                edge_a[1] + (edge_b[1] - edge_a[1]) * factor,
            )
            station_points[station] = point
            if station == 0:
                point_key = ("m1-source-vertex", source_edge[0])
            elif station == side.station_extent:
                point_key = ("m1-source-vertex", source_edge[1])
            else:
                point_key = (
                    "m1-transition",
                    _hashable_provenance(side.transition_key),
                    int(station),
                )
            previous_key = transition_point_keys.setdefault(point, point_key)
            if previous_key != point_key:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: conflicting transition vertex"
                )
        transition_records.append((side, edge_a, edge_b, station_points))

    def conformed_path(first, second, snap_near_transition=True):
        replacements = {Fraction(0): first, Fraction(1): second}
        covered_intervals = []
        covered_transition_edges = []
        for side, edge_a, edge_b, station_points in transition_records:
            edge_direction = (
                edge_b[0] - edge_a[0], edge_b[1] - edge_a[1]
            )
            edge_length2 = (
                edge_direction[0] * edge_direction[0]
                + edge_direction[1] * edge_direction[1]
            )
            if edge_length2 and snap_near_transition:
                for endpoint_factor, endpoint in (
                    (Fraction(0), first),
                    (Fraction(1), second),
                ):
                    relative = (
                        endpoint[0] - edge_a[0],
                        endpoint[1] - edge_a[1],
                    )
                    projection = (
                        relative[0] * edge_direction[0]
                        + relative[1] * edge_direction[1]
                    ) / edge_length2
                    cross_distance = (
                        relative[0] * edge_direction[1]
                        - relative[1] * edge_direction[0]
                    )
                    if (
                        0 <= projection <= 1
                        and cross_distance * cross_distance <= edge_length2
                    ):
                        declared_station = round(
                            float(projection) * side.station_extent
                        )
                        station = min(
                            side.stations,
                            key=lambda value: (
                                abs(value - declared_station), value
                            ),
                        )
                        if abs(station - declared_station) <= 1:
                            replacements[endpoint_factor] = (
                                station_points[station]
                            )
            hit = _m1_exact_segment_intersection(
                first, second, edge_a, edge_b
            )
            if hit is None:
                continue
            if hit == "COLLINEAR":
                overlap = _m1_collinear_overlap(
                    first, second, edge_a, edge_b
                )
                if overlap is None:
                    continue
                (
                    segment_start,
                    segment_end,
                    edge_start,
                    edge_end,
                ) = overlap
                direction = (
                    second[0] - first[0], second[1] - first[1]
                )
                boundary_stations = []
                for edge_factor in (edge_start, edge_end):
                    declared_station = round(
                        float(edge_factor) * side.station_extent
                    )
                    station = min(
                        side.stations,
                        key=lambda value: (
                            abs(value - declared_station), value
                        ),
                    )
                    if abs(station - declared_station) > 1:
                        raise ValueError(
                            "ATLAS_TRANSITION_DESYNC: undeclared "
                            "collinear endpoint"
                        )
                    boundary_stations.append(station)
                    canonical_point = station_points[station]
                    factor = (
                        (canonical_point[0] - first[0]) / direction[0]
                        if abs(direction[0]) >= abs(direction[1])
                        else (canonical_point[1] - first[1]) / direction[1]
                    )
                    if 0 <= factor <= 1:
                        replacements[factor] = canonical_point
                for station, station_point in station_points.items():
                    station_factor = Fraction(
                        station, side.station_extent
                    )
                    if not edge_start <= station_factor <= edge_end:
                        continue
                    factor = (
                        (station_point[0] - first[0]) / direction[0]
                        if abs(direction[0]) >= abs(direction[1])
                        else (station_point[1] - first[1]) / direction[1]
                    )
                    if 0 <= factor <= 1:
                        replacements[factor] = station_point
                if segment_start < segment_end:
                    covered_intervals.append(
                        (segment_start, segment_end)
                    )
                    overlap_stations = tuple(
                        sorted(
                            {
                                *boundary_stations,
                                *(
                                    station
                                    for station in side.stations
                                    if edge_start
                                    <= Fraction(
                                        station, side.station_extent
                                    )
                                    <= edge_end
                                ),
                            }
                        )
                    )
                    covered_transition_edges.extend(
                        (
                            station_points[first_station],
                            station_points[second_station],
                        )
                        for first_station, second_station in zip(
                            overlap_stations, overlap_stations[1:]
                        )
                        if station_points[first_station]
                        != station_points[second_station]
                    )
                continue
            segment_factor, edge_factor, _point = hit
            declared_station = round(
                float(edge_factor) * side.station_extent
            )
            station = min(
                side.stations,
                key=lambda value: (abs(value - declared_station), value),
            )
            if abs(station - declared_station) > 1:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: undeclared local intersection "
                    f"transition={side.transition_key!r} "
                    f"declared={declared_station!r} nearest={station!r} "
                    f"stations={side.stations!r} "
                    f"curve={(first, second)!r} "
                    f"semantic={tuple(sorted((first, second))) in semantic_segment_keys}"
                )
            canonical_point = station_points[station]
            previous = replacements.get(segment_factor)
            if previous is not None and previous != canonical_point:
                if (
                    abs(float(previous[0] - canonical_point[0])) > 1.0
                    or abs(float(previous[1] - canonical_point[1])) > 1.0
                ):
                    raise ValueError(
                        "ATLAS_TRANSITION_DESYNC: endpoint shift exceeds quantum"
                    )
            replacements[segment_factor] = canonical_point
        return (
            tuple(sorted(replacements.items())),
            tuple(covered_intervals),
            tuple(covered_transition_edges),
        )

    conformed_segments = []
    conformed_corner_boundary_vertices = {}
    conformed_crop_boundary_ids = {}
    conformed_semantic_boundary_ids = {}

    def toggle_crop_boundary_ids(edge, crop_ids):
        """Граница crop — Z2-cycle: совпавшие после B0 рёбра сокращаются."""

        edge_key = tuple(sorted(edge))
        bucket = conformed_crop_boundary_ids.setdefault(edge_key, set())
        for crop_id in crop_ids:
            if crop_id in bucket:
                bucket.remove(crop_id)
            else:
                bucket.add(crop_id)
        if not bucket:
            conformed_crop_boundary_ids.pop(edge_key, None)

    # Один и тот же raw edge приходит из crop, diagram и domain polygons.
    # Геометрический граф читает его один раз; provenance уже объединён в
    # raw_* maps. Иначе Z2-toggle ошибочно считает producer-дубликаты.
    unique_raw_segments = tuple(
        sorted(
            {
                tuple(sorted((first, second)))
                for first, second in raw_segments
                if first != second
            }
        )
    )
    for first, second in unique_raw_segments:
        source_corner_vertices = raw_corner_boundary_vertices.get(
            tuple(sorted((first, second))), ()
        )
        source_crop_ids = raw_crop_boundary_ids.get(
            tuple(sorted((first, second))), ()
        )
        source_semantic_ids = raw_semantic_boundary_ids.get(
            tuple(sorted((first, second))), ()
        )
        (
            ordered_path,
            covered_intervals,
            covered_transition_edges,
        ) = conformed_path(
            first,
            second,
            snap_near_transition=(
                tuple(sorted((first, second)))
                not in semantic_segment_keys
            ),
        )
        for transition_edge in covered_transition_edges:
            transition_edge_key = tuple(sorted(transition_edge))
            if source_crop_ids:
                toggle_crop_boundary_ids(
                    transition_edge_key, source_crop_ids
                )
            if source_corner_vertices:
                conformed_corner_boundary_vertices.setdefault(
                    transition_edge_key, set()
                ).update(source_corner_vertices)
            if source_semantic_ids:
                conformed_semantic_boundary_ids.setdefault(
                    transition_edge_key, set()
                ).update(source_semantic_ids)
        for (first_factor, first_point), (
            second_factor,
            second_point,
        ) in zip(ordered_path, ordered_path[1:]):
            midpoint_factor = (first_factor + second_factor) / 2
            if any(
                start <= midpoint_factor <= end
                for start, end in covered_intervals
            ):
                continue
            if first_point != second_point:
                conformed_segment = (first_point, second_point)
                conformed_segments.append(conformed_segment)
                if source_corner_vertices:
                    conformed_corner_boundary_vertices.setdefault(
                        tuple(sorted(conformed_segment)), set()
                    ).update(source_corner_vertices)
                if source_crop_ids:
                    toggle_crop_boundary_ids(
                        conformed_segment, source_crop_ids
                    )
                if source_semantic_ids:
                    conformed_semantic_boundary_ids.setdefault(
                        tuple(sorted(conformed_segment)), set()
                    ).update(source_semantic_ids)

    original_predicates = dict(predicates)
    conformed_predicates = {}
    for key, (pending_index, pending_face, crop_keys) in predicates.items():
        polygon = []
        for index, first in enumerate(crop_keys):
            second = crop_keys[(index + 1) % len(crop_keys)]
            (
                ordered_path,
                _covered_intervals,
                _covered_transition_edges,
            ) = conformed_path(first, second)
            path = tuple(point for _factor, point in ordered_path)
            if not polygon:
                polygon.extend(path)
            elif polygon[-1] == path[0]:
                polygon.extend(path[1:])
            else:
                polygon.extend(path)
        if len(polygon) > 1 and polygon[0] == polygon[-1]:
            polygon.pop()
        compact = []
        for point in polygon:
            if not compact or compact[-1] != point:
                compact.append(point)
        conformed_predicates[key] = (
            pending_index,
            pending_face,
            tuple(compact),
        )
    predicates = conformed_predicates

    for side, _edge_a, _edge_b, station_points in transition_records:
        conformed_segments.extend(
            (station_points[first], station_points[second])
            for first, second in zip(side.stations, side.stations[1:])
            if station_points[first] != station_points[second]
        )
    raw_segments = conformed_segments

    # Duplicate input curves collapse before pairwise intersection.
    segments = tuple(
        (first, second)
        for first, second in sorted(
            {
                tuple(sorted((first, second)))
                for first, second in raw_segments
            }
        )
    )
    split_points = [set(segment) for segment in segments]

    def cross(first, second, third):
        return (
            (second[0] - first[0]) * (third[1] - first[1])
            - (second[1] - first[1]) * (third[0] - first[0])
        )

    def on_segment(point, first, second):
        return (
            cross(first, second, point) == 0
            and min(first[0], second[0]) <= point[0] <= max(first[0], second[0])
            and min(first[1], second[1]) <= point[1] <= max(first[1], second[1])
        )

    for first_index, (point_a, point_b) in enumerate(segments):
        min_ax = min(point_a[0], point_b[0])
        max_ax = max(point_a[0], point_b[0])
        min_ay = min(point_a[1], point_b[1])
        max_ay = max(point_a[1], point_b[1])
        direction_a = (point_b[0] - point_a[0], point_b[1] - point_a[1])
        for second_index in range(first_index + 1, len(segments)):
            point_c, point_d = segments[second_index]
            if (
                max(point_c[0], point_d[0]) < min_ax
                or min(point_c[0], point_d[0]) > max_ax
                or max(point_c[1], point_d[1]) < min_ay
                or min(point_c[1], point_d[1]) > max_ay
            ):
                continue
            direction_b = (
                point_d[0] - point_c[0],
                point_d[1] - point_c[1],
            )
            denominator = (
                direction_a[0] * direction_b[1]
                - direction_a[1] * direction_b[0]
            )
            relative = (point_c[0] - point_a[0], point_c[1] - point_a[1])
            if denominator == 0:
                if relative[0] * direction_a[1] != relative[1] * direction_a[0]:
                    continue
                for candidate in (point_a, point_b, point_c, point_d):
                    if on_segment(candidate, point_a, point_b) and on_segment(
                        candidate, point_c, point_d
                    ):
                        split_points[first_index].add(candidate)
                        split_points[second_index].add(candidate)
                continue
            factor_a = Fraction(
                relative[0] * direction_b[1]
                - relative[1] * direction_b[0],
                denominator,
            )
            factor_b = Fraction(
                relative[0] * direction_a[1]
                - relative[1] * direction_a[0],
                denominator,
            )
            if not (0 <= factor_a <= 1 and 0 <= factor_b <= 1):
                continue
            intersection = (
                point_a[0] + direction_a[0] * factor_a,
                point_a[1] + direction_a[1] * factor_a,
            )
            split_points[first_index].add(intersection)
            split_points[second_index].add(intersection)

    graph = {}
    graph_corner_boundary_vertices = {}
    graph_crop_boundary_ids = {}
    graph_semantic_boundary_ids = {}
    for (first, second), stations in zip(segments, split_points):
        segment_corner_vertices = conformed_corner_boundary_vertices.get(
            tuple(sorted((first, second))), ()
        )
        segment_crop_ids = conformed_crop_boundary_ids.get(
            tuple(sorted((first, second))), ()
        )
        segment_semantic_ids = conformed_semantic_boundary_ids.get(
            tuple(sorted((first, second))), ()
        )
        direction = (second[0] - first[0], second[1] - first[1])
        ordered = sorted(
            stations,
            key=lambda point: (
                (point[0] - first[0]) * direction[0]
                + (point[1] - first[1]) * direction[1],
                point,
            ),
        )
        for point_a, point_b in zip(ordered, ordered[1:]):
            if point_a == point_b:
                continue
            graph.setdefault(point_a, set()).add(point_b)
            graph.setdefault(point_b, set()).add(point_a)
            if segment_corner_vertices:
                graph_corner_boundary_vertices.setdefault(
                    tuple(sorted((point_a, point_b))), set()
                ).update(segment_corner_vertices)
            if segment_crop_ids:
                graph_crop_boundary_ids.setdefault(
                    tuple(sorted((point_a, point_b))), set()
                ).update(segment_crop_ids)
            if segment_semantic_ids:
                graph_semantic_boundary_ids.setdefault(
                    tuple(sorted((point_a, point_b))), set()
                ).update(segment_semantic_ids)

    # A-INT: representative-классификация корректна только при целой
    # замкнутой границе каждого predicate crop. Проверяем это до tracing.
    crop_edges = {crop_id: set() for crop_id in expected_crop_boundary_ids}
    for edge, crop_ids in graph_crop_boundary_ids.items():
        for crop_id in crop_ids:
            crop_edges.setdefault(crop_id, set()).add(edge)
    for crop_id in sorted(expected_crop_boundary_ids, key=repr):
        edges = crop_edges.get(crop_id, set())
        adjacency = {}
        for first, second in edges:
            adjacency.setdefault(first, set()).add(second)
            adjacency.setdefault(second, set()).add(first)
        broken_vertices = tuple(
            sorted(
                (vertex, len(neighbors))
                for vertex, neighbors in adjacency.items()
                if len(neighbors) != 2
            )
        )
        connected = set()
        if adjacency:
            queue = [min(adjacency)]
            while queue:
                vertex = queue.pop()
                if vertex in connected:
                    continue
                connected.add(vertex)
                queue.extend(adjacency[vertex] - connected)
        if (
            not edges
            or broken_vertices
            or len(connected) != len(adjacency)
        ):
            if diagnostics is not None:
                diagnostics.atlas_arrangement_integrity_failure_count += 1
            raise ValueError(
                "ATLAS_ARRANGEMENT_INTEGRITY: conformed crop boundary "
                "is not one closed cycle "
                f"crop={crop_id!r} edges={len(edges)} "
                f"vertices={len(adjacency)} connected={len(connected)} "
                f"broken={broken_vertices!r} "
                f"raw={tuple(edge for edge, ids in raw_crop_boundary_ids.items() if crop_id in ids)!r} "
                f"conformed={tuple(edge for edge, ids in conformed_crop_boundary_ids.items() if crop_id in ids)!r}"
            )

    ordered_neighbors = {
        vertex: tuple(
            sorted(
                neighbors,
                key=lambda other: atan2(
                    float(other[1] - vertex[1]),
                    float(other[0] - vertex[0]),
                ),
            )
        )
        for vertex, neighbors in graph.items()
    }
    visited = set()
    cycles = []
    half_edge_count = sum(len(values) for values in graph.values())

    def simple_cycles(walk):
        first_index_by_vertex = {}
        for index, vertex in enumerate(walk):
            previous = first_index_by_vertex.get(vertex)
            if previous is None:
                first_index_by_vertex[vertex] = index
                continue
            inner = walk[previous:index]
            outer = walk[:previous] + walk[index:]
            result = []
            if len(inner) >= 3:
                result.extend(simple_cycles(inner))
            if len(outer) >= 3:
                result.extend(simple_cycles(outer))
            return result
        return [walk]

    for start in sorted(
        (first, second)
        for first, neighbors in graph.items()
        for second in neighbors
    ):
        if start in visited:
            continue
        cycle = []
        current = start
        for _step in range(half_edge_count + 1):
            if current in visited:
                break
            visited.add(current)
            first, second = current
            cycle.append(first)
            neighbors = ordered_neighbors[second]
            reverse_index = neighbors.index(first)
            third = neighbors[(reverse_index - 1) % len(neighbors)]
            current = (second, third)
            if current == start:
                break
        if current != start or len(cycle) < 3:
            continue
        for simple_cycle in simple_cycles(cycle):
            area2 = sum(
                first[0] * second[1] - second[0] * first[1]
                for first, second in zip(
                    simple_cycle, simple_cycle[1:] + simple_cycle[:1]
                )
            )
            if area2 > 0:
                cycles.append(tuple(simple_cycle))

    ordered_predicates = tuple(
        (
            predicates[key][0],
            predicates[key][1],
            predicates[key][2],
            original_predicates[key][2],
        )
        for key in sorted(predicates)
    )
    semantic_delegates = {}
    for semantic, _curve in imported_curves:
        delegate_key = (
            semantic.owner_chart_id,
            semantic.target_chart_id,
            semantic.owner_crop.kind,
            semantic.owner_crop.side,
            semantic.owner_crop.semantic_owner_id,
            int(semantic.owner_site.edge_index),
            semantic.owner_edge_indices,
            semantic.owner_region_points,
            semantic.target_region_points,
            tuple(round(value, 12) for value in semantic.local_to_owner),
        )
        semantic_delegates.setdefault(delegate_key, semantic)

    def exact_cycle_centroid(cycle):
        area2 = sum(
            first[0] * second[1] - second[0] * first[1]
            for first, second in zip(cycle, cycle[1:] + cycle[:1])
        )
        if area2 == 0:
            raise ValueError(
                "ATLAS_ARRANGEMENT_INTEGRITY: zero-area traced cycle"
            )
        x_numerator = sum(
            (first[0] + second[0])
            * (first[0] * second[1] - second[0] * first[1])
            for first, second in zip(cycle, cycle[1:] + cycle[:1])
        )
        y_numerator = sum(
            (first[1] + second[1])
            * (first[0] * second[1] - second[0] * first[1])
            for first, second in zip(cycle, cycle[1:] + cycle[:1])
        )
        return (
            x_numerator / (3 * area2),
            y_numerator / (3 * area2),
        )

    cycle_area2 = {
        cycle: abs(
            sum(
                first[0] * second[1] - second[0] * first[1]
                for first, second in zip(
                    cycle, cycle[1:] + cycle[:1]
                )
            )
        )
        for cycle in cycles
    }
    cycle_centroids = {
        cycle: exact_cycle_centroid(cycle) for cycle in cycles
    }
    cycle_parent = {}
    for cycle in cycles:
        parent_candidates = tuple(
            candidate
            for candidate in cycles
            if candidate is not cycle
            and cycle_area2[candidate] > cycle_area2[cycle]
            and _m1_point_in_polygon(
                cycle_centroids[cycle], candidate, tolerance=0.0
            )
        )
        cycle_parent[cycle] = (
            min(
                parent_candidates,
                key=lambda candidate: (
                    cycle_area2[candidate], candidate
                ),
            )
            if parent_candidates
            else None
        )
    cycle_children = {cycle: [] for cycle in cycles}
    for cycle, parent in cycle_parent.items():
        if parent is not None:
            cycle_children[parent].append(cycle)

    def region_representative(cycle):
        children = tuple(cycle_children[cycle])

        def in_region(point):
            return _m1_point_in_polygon(
                point, cycle, tolerance=0.0
            ) and not any(
                _m1_point_in_polygon(point, child, tolerance=0.0)
                for child in children
            )

        centroid = cycle_centroids[cycle]
        if in_region(centroid):
            return centroid
        # Сэмпл у собственной границы остаётся внутри outer cycle, но
        # гарантированно выходит из вложенного immediate child.
        for first, second in zip(cycle, cycle[1:] + cycle[:1]):
            midpoint = (
                (first[0] + second[0]) / 2,
                (first[1] + second[1]) / 2,
            )
            for denominator in (16, 256, 4096):
                candidate = (
                    (
                        midpoint[0] * (denominator - 1)
                        + centroid[0]
                    )
                    / denominator,
                    (
                        midpoint[1] * (denominator - 1)
                        + centroid[1]
                    )
                    / denominator,
                )
                if in_region(candidate):
                    return candidate
        if diagnostics is not None:
            diagnostics.atlas_arrangement_integrity_failure_count += 1
        raise ValueError(
            "ATLAS_ARRANGEMENT_INTEGRITY: traced region has no "
            f"representative cycle={cycle!r} children={children!r}"
        )

    def output_point(point):
        integer_key = None
        if point[0].denominator == 1 and point[1].denominator == 1:
            integer_key = (int(point[0]), int(point[1]))
        if integer_key in representatives:
            return representatives[integer_key]
        return (float(point[0]) * quantum, float(point[1]) * quantum)

    arranged = []
    for cycle in cycles:
        exact_representative = region_representative(cycle)
        polygon = tuple(output_point(point) for point in cycle)
        representative = (
            float(exact_representative[0]) * quantum,
            float(exact_representative[1]) * quantum,
        )

        def has_source_owner_at_representative():
            return any(
                _m1_point_in_polygon(
                    exact_representative, original_crop
                )
                for _index, _face, _crop, original_crop
                in ordered_predicates
            )

        wrong_transition_side = False
        boundary_owner_tokens = set()
        boundary_semantic_classes = set()
        boundary_owner_declarations = []
        touches_canonical_transition = False
        touched_transition_sides = []
        for side, edge_a, edge_b, _station_points in transition_records:
            transition_orientations = []
            point_stations = {
                point: station
                for station, point in _station_points.items()
            }
            interval_owner_by_key = {
                (first, second): owner
                for first, second, owner in side.interval_owners
            }
            interval_semantic_by_key = {
                (first, second): semantic_class
                for first, second, semantic_class
                in side.interval_semantics
            }
            edge_direction = (
                edge_b[0] - edge_a[0], edge_b[1] - edge_a[1]
            )
            for first, second in zip(cycle, cycle[1:] + cycle[:1]):
                if not (
                    on_segment(first, edge_a, edge_b)
                    and on_segment(second, edge_a, edge_b)
                ):
                    continue
                cycle_direction = (
                    second[0] - first[0], second[1] - first[1]
                )
                alignment = (
                    cycle_direction[0] * edge_direction[0]
                    + cycle_direction[1] * edge_direction[1]
                )
                if alignment:
                    transition_orientations.append(
                        1 if alignment > 0 else -1
                    )
                first_station = point_stations.get(first)
                second_station = point_stations.get(second)
                if (
                    first_station is not None
                    and second_station is not None
                    and first_station != second_station
                ):
                    interval = tuple(
                        sorted((first_station, second_station))
                    )
                    for (start, end), owner in interval_owner_by_key.items():
                        overlap_start = max(interval[0], start)
                        overlap_end = min(interval[1], end)
                        if overlap_start >= overlap_end:
                            continue
                        semantic_class = interval_semantic_by_key.get(
                            (start, end)
                        )
                        boundary_owner_tokens.add(owner)
                        if semantic_class is not None:
                            boundary_semantic_classes.add(semantic_class)
                        boundary_owner_declarations.append(
                            (
                                side,
                                start,
                                end,
                                overlap_start,
                                overlap_end,
                                owner,
                                semantic_class,
                            )
                        )
            if not transition_orientations:
                continue
            touches_canonical_transition = True
            touched_transition_sides.append(side)
            if any(
                orientation != side.interior_sign
                for orientation in transition_orientations
            ):
                wrong_transition_side = True
                break
        if wrong_transition_side:
            continue
        boundary_owner_tokens.discard(None)
        if len(boundary_semantic_classes) > 1:
            boundary_corner_curves = tuple(
                sorted(
                    {
                        corner_vertex
                        for first, second in zip(
                            cycle, cycle[1:] + cycle[:1]
                        )
                        for corner_vertex in graph_corner_boundary_vertices.get(
                            tuple(sorted((first, second))), ()
                        )
                    }
                )
            )
            raise ValueError(
                "ATLAS_CLASS_DESYNC: one face spans different declared "
                "semantic classes "
                f"chart={surface.domain.chart_id} "
                f"classes={tuple(sorted(boundary_semantic_classes, key=repr))!r} "
                f"declarations={tuple((item[0].transition_key,) + item[1:] for item in boundary_owner_declarations)!r} "
                f"boundary_corners={boundary_corner_curves!r} "
                f"polygon={cycle!r}"
            )
        expected_semantic_class = (
            next(iter(boundary_semantic_classes))
            if len(boundary_semantic_classes) == 1
            else None
        )
        if touches_canonical_transition and not boundary_owner_tokens:
            if not has_source_owner_at_representative():
                # Margin/site-image fragment без локальной материи.
                if diagnostics is not None:
                    if any(
                        side.single_declared_side
                        for side in touched_transition_sides
                    ):
                        diagnostics.atlas_single_side_drop_count += 1
                    else:
                        diagnostics.atlas_touch_no_token_drop_count += 1
                continue

        def resolve_declared_owner_token():
            """T7-P3: выбирает owner только по T2 station intervals."""

            if not boundary_owner_tokens:
                return None
            if len(boundary_owner_tokens) == 1:
                return next(iter(boundary_owner_tokens))

            transition_candidates = []
            seen_sides = set()
            for (
                side,
                _start,
                _end,
                _touch_start,
                _touch_end,
                _owner,
                _semantic,
            ) in sorted(
                boundary_owner_declarations,
                key=lambda item: (
                    repr(item[0].transition_key),
                    item[1],
                    item[2],
                    item[3],
                    item[4],
                    repr(item[5]),
                ),
            ):
                side_key = (
                    _hashable_provenance(side.transition_key),
                    side.chart_id,
                )
                if side_key in seen_sides:
                    continue
                seen_sides.add(side_key)
                direction = _sub2(side.segment[1], side.segment[0])
                length2 = _dot2(direction, direction)
                if length2 <= 1e-20:
                    raise ValueError(
                        "ATLAS_TRANSITION_DESYNC: degenerate declared "
                        "transition"
                    )
                factor = _dot2(
                    _sub2(representative, side.segment[0]), direction
                ) / length2
                station = factor * side.station_extent
                touched_spans = sorted(
                    {
                        (touch_start, touch_end)
                        for (
                            declared_side,
                            _interval_start,
                            _interval_end,
                            touch_start,
                            touch_end,
                            _declared_owner,
                            _semantic_class,
                        ) in boundary_owner_declarations
                        if declared_side is side
                    }
                )
                merged_spans = []
                for touch_start, touch_end in touched_spans:
                    if merged_spans and touch_start <= merged_spans[-1][1]:
                        merged_spans[-1][1] = max(
                            merged_spans[-1][1], touch_end
                        )
                    else:
                        merged_spans.append([touch_start, touch_end])
                if len(merged_spans) != 1:
                    raise ValueError(
                        "ATLAS_TRANSITION_DESYNC: one face has disjoint "
                        "contact spans on a transition"
                    )
                touch_start, touch_end = merged_spans[0]
                if station < touch_start:
                    clamped_station = touch_start
                elif station >= touch_end:
                    # Station-span описывает покрытые интервалы, поэтому
                    # правый конец принадлежит левому half-open интервалу.
                    clamped_station = max(touch_start, touch_end - 1)
                else:
                    clamped_station = station
                interval_semantics = {
                    (start, end): semantic_class
                    for start, end, semantic_class
                    in side.interval_semantics
                }
                ordered_intervals = sorted(
                    (
                        start,
                        end,
                        owner,
                        interval_semantics.get((start, end)),
                    )
                    for start, end, owner in side.interval_owners
                )
                side_matches = []
                for interval_index, (
                    start,
                    end,
                    owner,
                    semantic_class,
                ) in enumerate(ordered_intervals):
                    # T7-P3.3: внутреннюю станцию забирает правый
                    # half-open интервал; только последний замкнут справа.
                    contains_station = start <= clamped_station < end
                    if interval_index == len(ordered_intervals) - 1:
                        contains_station = start <= clamped_station <= end
                    if (
                        owner is not None
                        and contains_station
                        and (
                            expected_semantic_class is None
                            or semantic_class == expected_semantic_class
                        )
                    ):
                        side_matches.append((start, owner))
                side_matches = sorted(set(side_matches))
                if len(side_matches) != 1:
                    raise ValueError(
                        "ATLAS_TRANSITION_DESYNC: clamped station does not "
                        "resolve one declared owner "
                        f"transition={side.transition_key!r} "
                        f"station={station!r} clamped={clamped_station!r} "
                        f"touched={tuple(map(tuple, merged_spans))!r} "
                        f"expected={expected_semantic_class!r} "
                        f"intervals={tuple(ordered_intervals)!r} "
                        f"declarations={tuple((item[0].transition_key,) + item[1:] for item in boundary_owner_declarations)!r} "
                        f"polygon={cycle!r}"
                    )
                interval_start, owner = side_matches[0]
                transition_candidates.append(
                    (
                        -(touch_end - touch_start),
                        repr(_hashable_provenance(side.transition_key)),
                        interval_start,
                        owner,
                    )
                )
            if not transition_candidates:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: no declared transition "
                    "candidate"
                )
            # T7-P3.3b: максимальный station-contact, затем только
            # канонические integer/provenance keys — без spatial distance.
            return min(transition_candidates)[-1]

        declared_owner_token = resolve_declared_owner_token()
        has_boundary_owner = declared_owner_token is not None
        locations = tuple(
            surface.domain.locate(point) for point in polygon
        )
        strict_triangles = (
            tuple(
                triangle.chart_points
                for triangle in surface.domain.intrinsic_triangles
            )
            if surface.domain.intrinsic_triangles and transition_sides
            else surface.domain.boundary_triangles
        )
        domain_tolerance = (
            1e-10
            if transition_sides
            else surface.domain.location_tolerance
        )
        if not has_boundary_owner and (
            not any(
                _point_in_triangle(
                    representative,
                    triangle,
                    tolerance=domain_tolerance,
                )
                for triangle in strict_triangles
            )
            or any(location is None for location in locations)
        ):
            continue
        owners = []
        for pending_index, pending_face, crop, original_crop in ordered_predicates:
            pending_token = int(pending_face.site.edge_index)
            if has_boundary_owner and pending_token != declared_owner_token:
                continue
            if not (
                _m1_point_in_polygon(exact_representative, crop)
                or _m1_point_in_polygon(
                    exact_representative, original_crop
                )
            ):
                continue
            priority = (
                3
                if pending_face.crop.kind == "JUNCTION"
                else 1
                if pending_face.crop.kind == "SEGMENT"
                else 2
            )
            distance = _segment_point_distance2(
                pending_face.site.point_a,
                pending_face.site.point_b,
                representative,
            )[0]
            owners.append(
                (
                    -priority,
                    round(distance, 12),
                    pending_face.crop.kind,
                    pending_face.crop.side,
                    pending_face.site.edge_index,
                    pending_index,
                    pending_face,
                )
            )
        if not owners and has_boundary_owner:
            # T2-декларация уже решила семантику. Локальный face нужен только
            # как носитель site-атрибутов; расстояние до crop не участвует.
            carriers = []
            for pending_index, pending_face, _crop, _original_crop in (
                ordered_predicates
            ):
                if int(pending_face.site.edge_index) != declared_owner_token:
                    continue
                semantic_class = _m1_crop_semantic_class(pending_face.crop)
                carriers.append(
                    (
                        0
                        if semantic_class == expected_semantic_class
                        else 1,
                        pending_face.crop.kind,
                        pending_face.crop.side,
                        pending_index,
                        pending_face,
                    )
                )
            if not carriers:
                if diagnostics is not None:
                    diagnostics.atlas_declared_owner_missing_count += 1
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: declared owner carrier is "
                    "missing "
                    f"chart={surface.domain.chart_id} "
                    f"owner={declared_owner_token!r}"
                )
            carrier = min(carriers)
            owners.append(
                (
                    0,
                    0.0,
                    carrier[1],
                    carrier[2],
                    declared_owner_token,
                    carrier[3],
                    carrier[4],
                )
            )
        if not owners:
            # T3/T6: endpoint snap может оставить локальную ячейку уже одного
            # B0-кванта между двумя исходными crop-предикатами. Это не новая
            # материя и не spatial weld: arrangement уже построен, а семантика
            # узкой ячейки наследуется от ближайшей исходной границы.
            sliver_owners = []
            for (
                pending_index,
                pending_face,
                _crop,
                original_crop,
            ) in ordered_predicates:
                boundary_distance = min(
                    _segment_point_distance2(
                        first,
                        second,
                        exact_representative,
                    )[0]
                    for first, second in zip(
                        original_crop,
                        original_crop[1:] + original_crop[:1],
                    )
                )
                # §8a: только недекларированный B0-sliver в пределах кванта.
                if boundary_distance > 1.0 + 1e-9:
                    continue
                priority = (
                    3
                    if pending_face.crop.kind == "JUNCTION"
                    else 1
                    if pending_face.crop.kind == "SEGMENT"
                    else 2
                )
                sliver_owners.append(
                    (
                        round(float(boundary_distance), 12),
                        -priority,
                        pending_face.crop.kind,
                        pending_face.crop.side,
                        pending_face.site.edge_index,
                        pending_index,
                        pending_face,
                    )
                )
            if sliver_owners:
                sliver_owner = min(sliver_owners)
                if diagnostics is not None:
                    diagnostics.atlas_sliver_owner_count += 1
                    diagnostics.atlas_max_sliver_owner_distance = max(
                        diagnostics.atlas_max_sliver_owner_distance,
                        float(sliver_owner[0]),
                    )
                owners.append(
                    (
                        sliver_owner[1],
                        sliver_owner[0],
                        *sliver_owner[2:],
                    )
                )
        if not owners:
            if (
                diagnostics is not None
                and has_source_owner_at_representative()
            ):
                diagnostics.atlas_no_owner_drop_count += 1
            continue
        owner = min(owners)[-1]
        if has_boundary_owner and diagnostics is not None:
            diagnostics.atlas_declared_owner_count += 1
        semantic_owner_id = owner.crop.semantic_owner_id
        if (
            owner.crop.kind != "SEGMENT"
            and isinstance(semantic_owner_id, tuple)
            and semantic_owner_id[:1] == ("corner",)
            and dict(surface.semantic_owner_chart_by_vertex).get(
                int(semantic_owner_id[1]), surface.domain.chart_id
            )
            != surface.domain.chart_id
            and expected_semantic_class
            != _m1_crop_semantic_class(owner.crop)
        ):
            # Невладеющий chart хранит corner crop только как инертную
            # кривую arrangement. Без T7-импорта ярлык остаётся SEGMENT:
            # сосед не имеет права повторно вывести corner policy сам.
            segment_images = [
                pending_face
                for _index, pending_face, _crop, _original_crop
                in ordered_predicates
                if pending_face.crop.kind == "SEGMENT"
                and pending_face.site.edge_index == owner.site.edge_index
            ]
            if segment_images:
                owner = min(
                    segment_images,
                    key=lambda face: (
                        round(
                            _segment_point_distance2(
                                face.site.point_a,
                                face.site.point_b,
                                representative,
                            )[0],
                            12,
                        ),
                        face.site.edge_index,
                    ),
                )
        if (
            expected_semantic_class == ("SEGMENT",)
            and owner.crop.kind != "SEGMENT"
        ):
            segment_images = [
                pending_face
                for _index, pending_face, _crop, _original_crop
                in ordered_predicates
                if pending_face.crop.kind == "SEGMENT"
                and (
                    not has_boundary_owner
                    or int(pending_face.site.edge_index)
                    == declared_owner_token
                )
            ]
            if segment_images:
                owner = min(
                    segment_images,
                    key=lambda face: (
                        round(
                            _segment_point_distance2(
                                face.site.point_a,
                                face.site.point_b,
                                representative,
                            )[0],
                            12,
                        ),
                        face.site.edge_index,
                    ),
                )
        imported_candidates = []
        for semantic in semantic_delegates.values():
            semantic_class = _m1_crop_semantic_class(
                semantic.owner_crop
            )
            if (
                expected_semantic_class is not None
                and semantic_class != expected_semantic_class
            ):
                continue
            if (
                has_boundary_owner
                and not (
                    declared_owner_token in semantic.owner_edge_indices
                )
            ):
                continue
            owner_point = _affine_point(
                semantic.local_to_owner, representative
            )
            if (
                expected_semantic_class is None
                and not (
                    _m1_point_in_polygon(
                        representative,
                        semantic.target_region_points,
                    )
                    if semantic.target_region_points
                    else _m1_point_in_polygon(
                        owner_point,
                        semantic.owner_region_points,
                    )
                )
            ):
                continue
            priority = (
                3
                if semantic.owner_crop.kind == "JUNCTION"
                else 2
            )
            distance = _segment_point_distance2(
                semantic.owner_site.point_a,
                semantic.owner_site.point_b,
                owner_point,
            )[0]
            imported_candidates.append(
                (
                    -priority,
                    round(distance, 12),
                    semantic.owner_crop.kind,
                    semantic.owner_crop.side,
                    repr(semantic.owner_crop.semantic_owner_id),
                    semantic.owner_site.edge_index,
                        tuple(
                            round(value, 12)
                            for value in semantic.local_to_owner
                        ),
                        semantic.owner_region_points,
                        semantic.points,
                        semantic,
                )
            )
        imported_semantic = (
            min(
                imported_candidates,
                key=lambda item: (item[:-1], repr(item[-1])),
            )[-1]
            if imported_candidates
            else None
        )
        if (
            imported_semantic is None
            and expected_semantic_class is not None
            and _m1_crop_semantic_class(owner.crop)
            != expected_semantic_class
        ):
            # T2-C: R1 image/crop уже присутствует локально, даже если её
            # class-separator не пересекал этот T и не попал в T7 curves.
            # Контракт разрешает использовать её только как semantic carrier.
            image_carriers = tuple(
                pending_face
                for _index, pending_face, _crop, _original_crop
                in ordered_predicates
                if _m1_crop_semantic_class(pending_face.crop)
                == expected_semantic_class
            )
            if image_carriers:
                owner = min(
                    image_carriers,
                    key=lambda face: (
                        round(
                            _segment_point_distance2(
                                face.site.point_a,
                                face.site.point_b,
                                representative,
                            )[0],
                            12,
                        ),
                        face.site.edge_index,
                    ),
                )
        resolved_semantic_class = _m1_crop_semantic_class(
            imported_semantic.owner_crop
            if imported_semantic is not None
            else owner.crop
        )
        if (
            expected_semantic_class is not None
            and resolved_semantic_class != expected_semantic_class
        ):
            raise ValueError(
                "ATLAS_CLASS_DESYNC: canonical interval label is missing "
                f"chart={surface.domain.chart_id} "
                f"expected={expected_semantic_class!r} "
                f"actual={resolved_semantic_class!r}"
            )
        resolved_crop = (
            imported_semantic.owner_crop
            if imported_semantic is not None
            else owner.crop
        )
        uv_frame_delegate = (
            uv_frame_delegates.get(
                (int(owner.site.edge_index), resolved_semantic_class)
            )
            if imported_semantic is None
            else None
        )
        semantic_owner_id = resolved_crop.semantic_owner_id
        local_uv_frame_owner = surface.domain.chart_id
        if (
            isinstance(semantic_owner_id, tuple)
            and semantic_owner_id[:1] == ("corner",)
        ):
            local_uv_frame_owner = dict(
                surface.semantic_owner_chart_by_vertex
            ).get(int(semantic_owner_id[1]), local_uv_frame_owner)
        boundary_corner_vertices = tuple(
            sorted(
                {
                    corner_vertex
                    for first, second in zip(
                        cycle, cycle[1:] + cycle[:1]
                    )
                    for corner_vertex in graph_corner_boundary_vertices.get(
                        tuple(sorted((first, second))), ()
                    )
                }
            )
        )
        point_keys = tuple(
            transition_point_keys.get(
                point,
                (
                    "m1",
                    surface.patch_id,
                    surface.domain.chart_id,
                    (point[0].numerator, point[0].denominator),
                    (point[1].numerator, point[1].denominator),
                ),
            )
            for point in cycle
        )
        boundary_corner_edges = tuple(
            (
                point_keys[index],
                point_keys[(index + 1) % len(point_keys)],
                tuple(
                    sorted(
                        graph_corner_boundary_vertices.get(
                            tuple(
                                sorted(
                                    (
                                        cycle[index],
                                        cycle[(index + 1) % len(cycle)],
                                    )
                                )
                            ),
                            (),
                        )
                    )
                ),
            )
            for index in range(len(cycle))
            if graph_corner_boundary_vertices.get(
                tuple(
                    sorted(
                        (
                            cycle[index],
                            cycle[(index + 1) % len(cycle)],
                        )
                    )
                ),
                (),
            )
        )
        boundary_crop_edges = tuple(
            (
                point_keys[index],
                point_keys[(index + 1) % len(point_keys)],
                tuple(
                    sorted(
                        graph_crop_boundary_ids.get(
                            tuple(
                                sorted(
                                    (
                                        cycle[index],
                                        cycle[(index + 1) % len(cycle)],
                                    )
                                )
                            ),
                            (),
                        ),
                        key=repr,
                    )
                ),
            )
            for index in range(len(cycle))
            if graph_crop_boundary_ids.get(
                tuple(
                    sorted(
                        (
                            cycle[index],
                            cycle[(index + 1) % len(cycle)],
                        )
                    )
                ),
                (),
            )
        )
        boundary_semantic_edges = tuple(
            (
                point_keys[index],
                point_keys[(index + 1) % len(point_keys)],
                tuple(
                    sorted(
                        graph_semantic_boundary_ids.get(
                            tuple(
                                sorted(
                                    (
                                        cycle[index],
                                        cycle[(index + 1) % len(cycle)],
                                    )
                                )
                            ),
                            (),
                        ),
                        key=repr,
                    )
                ),
            )
            for index in range(len(cycle))
            if graph_semantic_boundary_ids.get(
                tuple(
                    sorted(
                        (
                            cycle[index],
                            cycle[(index + 1) % len(cycle)],
                        )
                    )
                ),
                (),
            )
        )
        arranged.append(
            _DecalArrangementFace(
                surface=surface,
                site=owner.site,
                points=polygon,
                crop=resolved_crop,
                point_keys=point_keys,
                uv_site=(
                    imported_semantic.owner_site
                    if imported_semantic is not None
                    else uv_frame_delegate.owner_site
                    if uv_frame_delegate is not None
                    else None
                ),
                uv_point_transform=(
                    imported_semantic.local_to_owner
                    if imported_semantic is not None
                    else None
                ),
                uv_frame_transform=(
                    imported_semantic.uv_local_to_owner
                    if imported_semantic is not None
                    else uv_frame_delegate.local_to_owner
                    if uv_frame_delegate is not None
                    else None
                ),
                uv_frame_owner_chart_id=(
                    imported_semantic.owner_chart_id
                    if imported_semantic is not None
                    else uv_frame_delegate.owner_chart_id
                    if uv_frame_delegate is not None
                    else int(local_uv_frame_owner)
                ),
                uv_frame_is_imported=(
                    imported_semantic is not None
                    or uv_frame_delegate is not None
                ),
                declared_owner_token=declared_owner_token,
                declared_semantic_class=expected_semantic_class,
                absorbed_corner_vertices=(
                    tuple(
                        sorted(
                            set(owner.absorbed_corner_vertices)
                            | set(
                                uv_frame_delegate.absorbed_corner_vertices
                                if uv_frame_delegate is not None
                                else ()
                            )
                        )
                    )
                ),
                terminal_cut_vertices=tuple(
                    sorted(
                        set(owner.terminal_cut_vertices)
                        | set(
                            uv_frame_delegate.terminal_cut_vertices
                            if uv_frame_delegate is not None
                            else ()
                        )
                    )
                ),
                boundary_corner_vertices=boundary_corner_vertices,
                boundary_corner_edges=boundary_corner_edges,
                boundary_crop_edges=boundary_crop_edges,
                boundary_semantic_edges=boundary_semantic_edges,
            )
        )
    arranged.sort(
        key=lambda face: (
            face.crop.kind,
            face.crop.side,
            face.site.edge_index,
            face.points,
        )
    )
    return tuple(arranged), max(0, len(graph) - len(representatives))


def _m1_atlas_transition_segments(domain):
    coordinate_edge_uses = {}
    for triangle in domain.intrinsic_triangles:
        for local_edge in range(3):
            edge_vertices = tuple(
                index for index in range(3) if index != local_edge
            )
            signature = tuple(
                sorted(
                    (
                        round(triangle.chart_points[index][0], 12),
                        round(triangle.chart_points[index][1], 12),
                    )
                    for index in edge_vertices
                )
            )
            coordinate_edge_uses[signature] = (
                coordinate_edge_uses.get(signature, 0) + 1
            )
    result = {}
    for triangle in domain.intrinsic_triangles:
        for local_edge, transition_key in enumerate(
            triangle.edge_transition_keys
        ):
            if (
                not isinstance(transition_key, tuple)
                or transition_key[:1] != ("atlas-transition",)
            ):
                continue
            source_edge = tuple(transition_key[2])
            edge_vertices = tuple(
                index for index in range(3) if index != local_edge
            )
            point_by_vertex = {
                triangle.source_vertex_ids[index]: triangle.chart_points[index]
                for index in edge_vertices
            }
            if set(point_by_vertex) == set(source_edge):
                segment = tuple(
                    point_by_vertex[vertex_id] for vertex_id in source_edge
                )
                signature = tuple(
                    sorted(
                        (round(point[0], 12), round(point[1], 12))
                        for point in segment
                    )
                )
                if coordinate_edge_uses.get(signature, 0) == 1:
                    result[(transition_key, signature)] = segment
    return result


def _m1_segment_isometry(source_segment, target_segment):
    """Изометрия source chart -> target chart по oriented transition."""

    source_direction = _sub2(source_segment[1], source_segment[0])
    target_direction = _sub2(target_segment[1], target_segment[0])
    source_length = sqrt(_dot2(source_direction, source_direction))
    target_length = sqrt(_dot2(target_direction, target_direction))
    denominator = max(source_length * target_length, 1e-20)
    cosine = _dot2(source_direction, target_direction) / denominator
    sine = _cross2(source_direction, target_direction) / denominator
    return (
        cosine,
        sine,
        target_segment[0][0]
        - cosine * source_segment[0][0]
        + sine * source_segment[0][1],
        target_segment[0][1]
        - sine * source_segment[0][0]
        - cosine * source_segment[0][1],
    )


def _m1_segment_frame_transform(source_segment, target_segment):
    """R5: endpoint-exact UV-frame transport без изменения geometry isometry."""

    source_direction = _sub2(source_segment[1], source_segment[0])
    target_direction = _sub2(target_segment[1], target_segment[0])
    source_length = sqrt(_dot2(source_direction, source_direction))
    target_length = sqrt(_dot2(target_direction, target_direction))
    if source_length <= 1e-20 or target_length <= 1e-20:
        raise ValueError("ATLAS_TRANSITION_DESYNC: UV frame segment is degenerate")
    source_tangent = (
        source_direction[0] / source_length,
        source_direction[1] / source_length,
    )
    target_tangent = (
        target_direction[0] / target_length,
        target_direction[1] / target_length,
    )
    source_normal = (-source_tangent[1], source_tangent[0])
    target_normal = (-target_tangent[1], target_tangent[0])
    tangent_scale = target_length / source_length
    # Тангенциальный масштаб ровно совмещает обе station-endpoint; нормаль
    # сохраняет физический масштаб и не меняет геометрию arrangement.
    m00 = (
        tangent_scale * target_tangent[0] * source_tangent[0]
        + target_normal[0] * source_normal[0]
    )
    m01 = (
        tangent_scale * target_tangent[0] * source_tangent[1]
        + target_normal[0] * source_normal[1]
    )
    m10 = (
        tangent_scale * target_tangent[1] * source_tangent[0]
        + target_normal[1] * source_normal[0]
    )
    m11 = (
        tangent_scale * target_tangent[1] * source_tangent[1]
        + target_normal[1] * source_normal[1]
    )
    tx = target_segment[0][0] - (
        m00 * source_segment[0][0] + m01 * source_segment[0][1]
    )
    ty = target_segment[0][1] - (
        m10 * source_segment[0][0] + m11 * source_segment[0][1]
    )
    return m00, m01, m10, m11, tx, ty


def _m1_transition_interior_sign(surface, transition_key, segment):
    interior_vertices = [
        triangle.chart_points[local_edge]
        for triangle in surface.domain.intrinsic_triangles
        for local_edge, key in enumerate(triangle.edge_transition_keys)
        if key == transition_key
    ]
    if len(interior_vertices) != 1:
        raise ValueError(
            "ATLAS_TRANSITION_DESYNC: semantic interior is ambiguous"
        )
    direction = _sub2(segment[1], segment[0])
    relative = _sub2(interior_vertices[0], segment[0])
    cross = _cross2(direction, relative)
    if abs(cross) <= 1e-12:
        raise ValueError(
            "ATLAS_TRANSITION_DESYNC: semantic interior is degenerate"
        )
    return 1 if cross > 0.0 else -1


def _m1_clip_open_curve(first, second, line_a, line_b, keep_sign):
    """Обрезает открытую curve указанной стороной oriented line."""

    direction = _sub2(line_b, line_a)

    def signed(point):
        return _cross2(direction, _sub2(point, line_a)) * keep_sign

    first_distance = signed(first)
    second_distance = signed(second)
    first_inside = first_distance >= -1e-10
    second_inside = second_distance >= -1e-10
    if not first_inside and not second_inside:
        return None
    if first_inside and second_inside:
        return (first, second) if first != second else None
    denominator = first_distance - second_distance
    if abs(denominator) <= 1e-20:
        return None
    factor = first_distance / denominator
    intersection = (
        first[0] + (second[0] - first[0]) * factor,
        first[1] + (second[1] - first[1]) * factor,
    )
    clipped = (
        (first, intersection) if first_inside else (intersection, second)
    )
    return clipped if clipped[0] != clipped[1] else None


def _m1_open_curves_beyond_transition(
    curves,
    segment,
    interior_sign,
    quantum,
):
    """T7-P: возвращает связанную с T открытую часть owner boundaries."""

    direction = _sub2(segment[1], segment[0])
    length2 = _dot2(direction, direction)
    length = sqrt(max(length2, 0.0))
    if length <= 1e-12:
        return ()

    def transition_parameter(point):
        relative = _sub2(point, segment[0])
        factor = _dot2(relative, direction) / length2
        tolerance = quantum / length * 1.5
        if factor < -tolerance or factor > 1.0 + tolerance:
            return None
        cross_distance = abs(_cross2(direction, relative)) / length
        endpoint_anchor = (
            abs(factor) <= tolerance
            or abs(factor - 1.0) <= tolerance
        )
        if cross_distance > max(1e-10, length * 1e-10) and not (
            endpoint_anchor and cross_distance <= quantum * 1.5
        ):
            return None
        return max(0.0, min(1.0, factor))

    outside_sign = -int(interior_sign)
    kept = []
    anchor_stations = set()
    line_crossings = 0
    for first, second in curves:
        first_cross = _cross2(direction, _sub2(first, segment[0]))
        second_cross = _cross2(direction, _sub2(second, segment[0]))
        if first_cross * second_cross < -1e-20:
            line_crossings += 1
        clipped = _m1_clip_open_curve(
            first,
            second,
            segment[0],
            segment[1],
            outside_sign,
        )
        if clipped is None:
            continue
        snapped = []
        for point in clipped:
            factor = transition_parameter(point)
            if factor is not None:
                anchor_stations.add(round(factor * length / quantum))
                point = (
                    segment[0][0] + direction[0] * factor,
                    segment[0][1] + direction[1] * factor,
                )
            snapped.append(point)
        clipped = tuple(snapped)
        if clipped[0] == clipped[1]:
            continue
        lattice_line_a = tuple(round(value / quantum) for value in segment[0])
        lattice_line_b = tuple(round(value / quantum) for value in segment[1])
        lattice_first = tuple(round(value / quantum) for value in clipped[0])
        lattice_second = tuple(round(value / quantum) for value in clipped[1])
        lattice_direction = _sub2(lattice_line_b, lattice_line_a)
        if (
            _cross2(
                lattice_direction, _sub2(lattice_first, lattice_line_a)
            )
            == 0
            and _cross2(
                lattice_direction, _sub2(lattice_second, lattice_line_a)
            )
            == 0
        ):
            # T7-P.5: коллинеарная граница уже представлена самой T.
            continue
        kept.append(clipped)
    if not kept:
        return ()
    if len(anchor_stations) < 2:
        polygon = tuple(curve[0] for curve in curves)
        continues_through_endpoint = (
            len(anchor_stations) == 1
            and line_crossings >= 2
            and any(
                _m1_point_in_polygon(endpoint, polygon)
                for endpoint in segment
            )
        )
        if not continues_through_endpoint:
            # Касание без связи с соседним T не создаёт semantic interval.
            return ()
    return tuple(kept)


def _m1_build_semantic_imports(groups, diagnostics=None):
    """T7-P: owner экспортирует только открытые class separators."""

    chart_records = {}
    target_corner_regions = {}
    sides_by_transition = {}
    for entries in groups:
        surface = entries[0][1].surface
        if surface.domain.admission_tier != "APPROXIMATE":
            continue
        chart_records[surface.domain.chart_id] = (
            id(surface.domain),
            entries,
            surface,
        )
        for _pending_index, pending_face in entries:
            crop = pending_face.crop
            if crop.kind == "SEGMENT" or crop.semantic_owner_id is None:
                continue
            target_corner_regions.setdefault(
                (
                    int(surface.domain.chart_id),
                    crop.kind,
                    crop.side,
                    _hashable_provenance(crop.semantic_owner_id),
                    int(pending_face.site.edge_index),
                ),
                tuple(crop.points),
            )
        for local_key, segment in _m1_atlas_transition_segments(
            surface.domain
        ).items():
            transition_key = local_key[0]
            sides_by_transition.setdefault(transition_key, {})[
                surface.domain.chart_id
            ] = (
                id(surface.domain),
                entries,
                surface,
                segment,
                _m1_transition_interior_sign(
                    surface, transition_key, segment
                ),
            )

    graph = {chart_id: [] for chart_id in chart_records}
    for transition_key in sorted(sides_by_transition, key=repr):
        sides = sides_by_transition[transition_key]
        if len(sides) != 2:
            continue
        ordered = sorted(sides.items())
        first_id, first_record = ordered[0]
        second_id, second_record = ordered[1]
        forward = _m1_segment_isometry(
            first_record[3], second_record[3]
        )
        forward_uv_frame = _m1_segment_frame_transform(
            first_record[3], second_record[3]
        )
        graph[first_id].append(
            (
                second_id,
                transition_key,
                first_record[3],
                first_record[4],
                forward,
                forward_uv_frame,
            )
        )
        graph[second_id].append(
            (
                first_id,
                transition_key,
                second_record[3],
                second_record[4],
                _affine_inverse(forward),
                _frame_affine_inverse(forward_uv_frame),
            )
        )

    semantic_sources = {}
    for chart_id, (_domain_id, entries, surface) in chart_records.items():
        owner_by_vertex = dict(surface.semantic_owner_chart_by_vertex)
        for _pending_index, pending_face in entries:
            crop = pending_face.crop
            if crop.semantic_owner_id is None or crop.kind == "SEGMENT":
                continue
            if (
                diagnostics is not None
                and crop.semantic_owner_id
                in diagnostics.semantic_transport_disabled_owner_ids
            ):
                continue
            if (
                isinstance(crop.semantic_owner_id, tuple)
                and crop.semantic_owner_id[:1] == ("corner",)
            ):
                owner_chart_id = owner_by_vertex.get(
                    int(crop.semantic_owner_id[1]), chart_id
                )
            else:
                owner_chart_id = chart_id
            if chart_id != owner_chart_id:
                continue
            identity = (
                owner_chart_id,
                crop.kind,
                crop.side,
                crop.semantic_owner_id,
                int(pending_face.site.edge_index),
                crop.points,
            )
            semantic_sources.setdefault(identity, (surface, pending_face))

    imports_by_domain = {}
    for identity in sorted(semantic_sources, key=repr):
        owner_chart_id = int(identity[0])
        owner_surface, pending_face = semantic_sources[identity]
        owner_edge_indices = tuple(
            sorted(
                {
                    int(pending_face.site.edge_index),
                    *(
                        int(owner_surface.sites[site_index].edge_index)
                        for site_index in pending_face.crop.owner_site_indices
                        if 0 <= site_index < len(owner_surface.sites)
                    ),
                }
            )
        )
        owner_curves = tuple(
            zip(
                pending_face.crop.points,
                pending_face.crop.points[1:]
                + pending_face.crop.points[:1],
            )
        )
        identity_transform = (1.0, 0.0, 0.0, 0.0)
        identity_uv_frame = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        queue = [
            (
                owner_chart_id,
                owner_curves,
                tuple(pending_face.crop.points),
                identity_transform,
                identity_uv_frame,
                None,
            )
        ]
        visited_charts = {owner_chart_id}
        while queue:
            (
                current_chart_id,
                current_curves,
                current_region,
                owner_to_current,
                owner_frame_to_current,
                incoming_transition,
            ) = queue.pop(0)
            current_surface = chart_records[current_chart_id][2]
            current_quantum = max(
                float(current_surface.diagram_transform.quantum), 1e-10
            )
            for (
                target_chart_id,
                transition_key,
                current_segment,
                current_interior_sign,
                current_to_target,
                current_frame_to_target,
            ) in sorted(
                graph[current_chart_id],
                key=lambda item: (item[0], repr(item[1])),
            ):
                if transition_key == incoming_transition:
                    continue
                if target_chart_id in visited_charts:
                    continue
                exported = _m1_open_curves_beyond_transition(
                    current_curves,
                    current_segment,
                    current_interior_sign,
                    current_quantum,
                )
                if not exported:
                    continue
                target_region = _clip_to_halfplane(
                    current_region,
                    current_segment[0],
                    current_segment[1],
                    keep_inside=current_interior_sign < 0,
                )
                if len(target_region) < 3:
                    raise ValueError(
                        "ATLAS_CLASS_DESYNC: transported corner region "
                        "collapsed before election"
                    )
                owner_to_target = _affine_compose(
                    current_to_target, owner_to_current
                )
                owner_frame_to_target = _frame_affine_compose(
                    current_frame_to_target, owner_frame_to_current
                )
                mapped_curves = tuple(
                    tuple(
                        _affine_point(current_to_target, point)
                        for point in curve
                    )
                    for curve in exported
                )
                mapped_region = tuple(
                    _affine_point(current_to_target, point)
                    for point in target_region
                )
                canonical_owner_chart_id = int(transition_key[3])
                canonical_owner_record = sides_by_transition[
                    transition_key
                ].get(canonical_owner_chart_id)
                if canonical_owner_record is None:
                    raise ValueError(
                        "ATLAS_TRANSITION_DESYNC: corner image owner side "
                        "is missing"
                    )
                canonical_owner_surface = canonical_owner_record[2]
                canonical_quantum = max(
                    float(
                        canonical_owner_surface.diagram_transform.quantum
                    ),
                    1e-10,
                )
                station_extent = max(
                    1,
                    round(
                        _dist2(
                            canonical_owner_record[3][0],
                            canonical_owner_record[3][1],
                        )
                        / canonical_quantum
                    ),
                )
                current_direction = _sub2(
                    current_segment[1], current_segment[0]
                )
                current_length2 = _dot2(
                    current_direction, current_direction
                )
                target_segment = sides_by_transition[transition_key][
                    target_chart_id
                ][3]
                anchored_region = []
                for current_point, mapped_point in zip(
                    target_region, mapped_region
                ):
                    distance, factor = _segment_point_distance2(
                        current_segment[0],
                        current_segment[1],
                        current_point,
                    )
                    if (
                        current_length2 > 1e-20
                        # A-INT.5: region и её boundary-curves потребляют
                        # один и тот же допустимый station snap.
                        and distance <= current_quantum * 1.5
                        and -1e-9 <= factor <= 1.0 + 1e-9
                    ):
                        station = max(
                            0,
                            min(
                                station_extent,
                                round(float(factor) * station_extent),
                            ),
                        )
                        canonical_factor = station / station_extent
                        mapped_point = (
                            target_segment[0][0]
                            + (
                                target_segment[1][0]
                                - target_segment[0][0]
                            )
                            * canonical_factor,
                            target_segment[0][1]
                            + (
                                target_segment[1][1]
                                - target_segment[0][1]
                            )
                            * canonical_factor,
                        )
                    anchored_region.append(mapped_point)
                mapped_region = tuple(anchored_region)
                local_region_key = (
                    int(target_chart_id),
                    pending_face.crop.kind,
                    pending_face.crop.side,
                    _hashable_provenance(
                        pending_face.crop.semantic_owner_id
                    ),
                    int(pending_face.site.edge_index),
                )
                local_region = target_corner_regions.get(local_region_key)
                if local_region is not None:
                    # T7-P3.15: target crop — только канонический B0
                    # geometry carrier. Класс, owner и UV-frame по-прежнему
                    # приходят из единственного semantic owner.
                    mapped_region = tuple(local_region)
                local_to_owner = _affine_inverse(owner_to_target)
                uv_local_to_owner = _frame_affine_inverse(
                    owner_frame_to_target
                )
                target_domain_id = chart_records[target_chart_id][0]
                target_imports = imports_by_domain.setdefault(
                    target_domain_id, {}
                )
                for mapped in mapped_curves:
                    import_key = (
                        owner_chart_id,
                        target_chart_id,
                        pending_face.crop.kind,
                        pending_face.crop.side,
                        pending_face.crop.semantic_owner_id,
                        int(pending_face.site.edge_index),
                        tuple(
                            sorted(
                                (
                                    round(point[0] / current_quantum),
                                    round(point[1] / current_quantum),
                                )
                                for point in mapped
                            )
                        ),
                    )
                    if import_key in target_imports:
                        continue
                    target_imports[import_key] = _M1ImportedSemantic(
                        transition_key=transition_key,
                        owner_chart_id=owner_chart_id,
                        target_chart_id=target_chart_id,
                        points=mapped,
                        owner_crop=pending_face.crop,
                        owner_site=pending_face.site,
                        owner_edge_indices=owner_edge_indices,
                        owner_region_points=pending_face.crop.points,
                        local_to_owner=local_to_owner,
                        uv_local_to_owner=uv_local_to_owner,
                        target_region_points=mapped_region,
                    )
                    if diagnostics is not None:
                        diagnostics.atlas_semantic_import_count += 1
                visited_charts.add(target_chart_id)
                queue.append(
                    (
                        target_chart_id,
                        mapped_curves,
                        mapped_region,
                        owner_to_target,
                        owner_frame_to_target,
                        transition_key,
                    )
                )

    def publish_anchor_stations(semantic):
        """A-INT.4-6: owner один раз публикует все T-стыки chain."""

        target_record = chart_records.get(int(semantic.target_chart_id))
        if target_record is None:
            raise ValueError(
                "ATLAS_ARRANGEMENT_INTEGRITY: import target chart is missing"
            )
        target_surface = target_record[2]
        target_quantum = max(
            float(target_surface.diagram_transform.quantum), 1e-10
        )
        points = tuple(semantic.points)
        rebuilt = []
        anchors = []
        for segment_index, (first, second) in enumerate(
            zip(points, points[1:])
        ):
            lattice_first = tuple(
                Fraction(round(value / target_quantum)) for value in first
            )
            lattice_second = tuple(
                Fraction(round(value / target_quantum)) for value in second
            )
            intersections = {
                Fraction(0): (
                    float(first[0]),
                    float(first[1]),
                    [],
                ),
                Fraction(1): (
                    float(second[0]),
                    float(second[1]),
                    [],
                ),
            }
            for transition_key, sides in sorted(
                sides_by_transition.items(), key=lambda item: repr(item[0])
            ):
                target_side = sides.get(int(semantic.target_chart_id))
                if target_side is None:
                    continue
                target_segment = target_side[3]
                edge_a = tuple(
                    Fraction(round(value / target_quantum))
                    for value in target_segment[0]
                )
                edge_b = tuple(
                    Fraction(round(value / target_quantum))
                    for value in target_segment[1]
                )
                hit = _m1_exact_segment_intersection(
                    lattice_first, lattice_second, edge_a, edge_b
                )
                if hit is None or hit == "COLLINEAR":
                    continue
                segment_factor, edge_factor, _point = hit
                owner_chart_id = int(transition_key[3])
                owner_side = sides.get(owner_chart_id)
                if owner_side is None:
                    raise ValueError(
                        "ATLAS_TRANSITION_DESYNC: anchor owner side is missing"
                    )
                owner_surface = owner_side[2]
                owner_quantum = max(
                    float(owner_surface.diagram_transform.quantum), 1e-10
                )
                station_extent = max(
                    1,
                    round(
                        _dist2(owner_side[3][0], owner_side[3][1])
                        / owner_quantum
                    ),
                )
                station = max(
                    0,
                    min(
                        station_extent,
                        round(float(edge_factor) * station_extent),
                    ),
                )
                canonical_factor = station / station_extent
                canonical_point = (
                    target_segment[0][0]
                    + (
                        target_segment[1][0] - target_segment[0][0]
                    )
                    * canonical_factor,
                    target_segment[0][1]
                    + (
                        target_segment[1][1] - target_segment[0][1]
                    )
                    * canonical_factor,
                )
                previous = intersections.get(segment_factor)
                if previous is not None:
                    if (
                        round(previous[0] / target_quantum)
                        != round(canonical_point[0] / target_quantum)
                        or round(previous[1] / target_quantum)
                        != round(canonical_point[1] / target_quantum)
                    ):
                        raise ValueError(
                            "ATLAS_TRANSITION_DESYNC: import anchors disagree"
                        )
                    anchor_records = previous[2]
                    anchor_records.append((transition_key, station))
                    intersections[segment_factor] = (
                        canonical_point[0],
                        canonical_point[1],
                        anchor_records,
                    )
                else:
                    intersections[segment_factor] = (
                        canonical_point[0],
                        canonical_point[1],
                        [(transition_key, station)],
                    )
            for factor, (x_value, y_value, records) in sorted(
                intersections.items()
            ):
                if segment_index and factor == 0:
                    point_index = len(rebuilt) - 1
                else:
                    point_index = len(rebuilt)
                    rebuilt.append((x_value, y_value))
                anchors.extend(
                    (point_index, transition_key, station)
                    for transition_key, station in records
                )
        return replace(
            semantic,
            points=tuple(rebuilt),
            anchor_stations=tuple(
                sorted(set(anchors), key=repr)
            ),
        )

    return {
        domain_id: tuple(
            sorted(
                (
                    publish_anchor_stations(semantic)
                    for semantic in imports.values()
                ),
                key=lambda item: (
                    repr(item.transition_key),
                    item.owner_crop.kind,
                    item.owner_crop.side,
                    repr(item.owner_crop.semantic_owner_id),
                    item.points,
                ),
            )
        )
        for domain_id, imports in imports_by_domain.items()
    }


def _m1_build_uv_frame_delegates(groups):
    """R5: переносит owner site-frame независимо от локальной геометрии crop."""

    chart_records = {}
    sides_by_transition = {}
    for entries in groups:
        surface = entries[0][1].surface
        if surface.domain.admission_tier != "APPROXIMATE":
            continue
        chart_records[surface.domain.chart_id] = (surface, entries)
        for local_key, segment in _m1_atlas_transition_segments(
            surface.domain
        ).items():
            sides_by_transition.setdefault(local_key[0], {})[
                surface.domain.chart_id
            ] = segment

    graph = {chart_id: [] for chart_id in chart_records}
    for transition_key in sorted(sides_by_transition, key=repr):
        sides = sides_by_transition[transition_key]
        if len(sides) != 2:
            continue
        (first_id, first_segment), (second_id, second_segment) = sorted(
            sides.items()
        )
        forward = _m1_segment_frame_transform(
            first_segment, second_segment
        )
        graph[first_id].append((second_id, transition_key, forward))
        graph[second_id].append(
            (first_id, transition_key, _frame_affine_inverse(forward))
        )

    semantic_sources = {}
    semantic_keys_by_chart = {}
    absorbed_by_edge_global = {}
    terminal_cuts_by_edge_global = {}
    for _chart_id, (_surface, entries) in chart_records.items():
        for _pending_index, pending_face in entries:
            if pending_face.crop.kind != "SEGMENT":
                continue
            absorbed_by_edge_global.setdefault(
                int(pending_face.site.edge_index), set()
            ).update(pending_face.absorbed_corner_vertices)
            terminal_cuts_by_edge_global.setdefault(
                int(pending_face.site.edge_index), set()
            ).update(pending_face.terminal_cut_vertices)
    for chart_id, (surface, entries) in chart_records.items():
        chart_sites = {
            int(site.edge_index): site
            for site in sorted(
                surface.sites, key=lambda candidate: candidate.edge_index
            )
        }
        for edge_index in surface.native_site_edge_indices:
            site = chart_sites.get(int(edge_index))
            if site is not None:
                semantic_sources.setdefault(
                    (int(edge_index), ("SEGMENT",)), []
                ).append(
                    (
                        chart_id,
                        site,
                        tuple(
                            sorted(
                                absorbed_by_edge_global.get(
                                    int(edge_index), ()
                                )
                            )
                        ),
                        tuple(
                            sorted(
                                terminal_cuts_by_edge_global.get(
                                    int(edge_index), ()
                                )
                            )
                        ),
                    )
                )
        owner_by_vertex = dict(surface.semantic_owner_chart_by_vertex)
        for _pending_index, pending_face in entries:
            semantic_class = _m1_crop_semantic_class(pending_face.crop)
            semantic_key = (
                int(pending_face.site.edge_index), semantic_class
            )
            semantic_keys_by_chart.setdefault(chart_id, set()).add(
                semantic_key
            )
            owner_id = pending_face.crop.semantic_owner_id
            if (
                pending_face.crop.kind != "SEGMENT"
                and isinstance(owner_id, tuple)
                and owner_id[:1] == ("corner",)
                and owner_by_vertex.get(int(owner_id[1]), chart_id)
                == chart_id
            ):
                semantic_sources.setdefault(semantic_key, []).append(
                    (chart_id, pending_face.site, (), ())
                )

    delegates_by_domain = {}
    identity = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    for semantic_key, candidates in sorted(
        semantic_sources.items(), key=lambda item: repr(item[0])
    ):
        edge_index, _semantic_class = semantic_key
        (
            owner_chart_id,
            owner_site,
            owner_absorbed_corners,
            owner_terminal_cuts,
        ) = min(candidates, key=lambda item: item[0])
        owner_to_chart = {owner_chart_id: identity}
        queue = [owner_chart_id]
        while queue:
            current_chart_id = queue.pop(0)
            for target_chart_id, transition_key, current_to_target in sorted(
                graph[current_chart_id],
                key=lambda item: (item[0], repr(item[1])),
            ):
                if target_chart_id in owner_to_chart:
                    continue
                owner_to_chart[target_chart_id] = _frame_affine_compose(
                    current_to_target, owner_to_chart[current_chart_id]
                )
                queue.append(target_chart_id)
        for target_chart_id, target_keys in semantic_keys_by_chart.items():
            if (
                target_chart_id == owner_chart_id
                or semantic_key not in target_keys
            ):
                continue
            owner_to_target = owner_to_chart.get(target_chart_id)
            if owner_to_target is None:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: UV frame consumer is disconnected"
                )
            target_surface = chart_records[target_chart_id][0]
            delegates_by_domain.setdefault(id(target_surface.domain), {})[
                semantic_key
            ] = _M1UVFrameDelegate(
                owner_chart_id=owner_chart_id,
                target_chart_id=target_chart_id,
                edge_index=edge_index,
                owner_site=owner_site,
                local_to_owner=_frame_affine_inverse(owner_to_target),
                absorbed_corner_vertices=owner_absorbed_corners,
                terminal_cut_vertices=owner_terminal_cuts,
            )
    return delegates_by_domain


def _m1_build_predicate_boundary_imports(
    groups,
    transition_contract,
    semantic_imports=None,
    diagnostics=None,
):
    """T7-P2: продолжает owner predicate-boundary, а не её half-edge след."""

    semantic_imports = semantic_imports or {}
    chart_records = {}
    sides_by_transition = {}
    for entries in groups:
        surface = entries[0][1].surface
        if surface.domain.admission_tier != "APPROXIMATE":
            continue
        chart_records[surface.domain.chart_id] = (surface, entries)
        for side in transition_contract.get(id(surface.domain), ()):
            transition_id = _hashable_provenance(side.transition_key)
            sides_by_transition.setdefault(transition_id, {})[
                surface.domain.chart_id
            ] = side

    def interval_records(side):
        owners = {
            (first, second): owner
            for first, second, owner in side.interval_owners
        }
        return tuple(
            (
                first,
                second,
                owners.get((first, second)),
                semantic_class,
            )
            for first, second, semantic_class in side.interval_semantics
        )

    def supporting_candidate(
        owner_side,
        station,
        corner_id,
        corner_face,
        segment_face,
    ):
        segment_site = segment_face.site
        if corner_id not in (segment_site.vert_a, segment_site.vert_b):
            return None
        anchor = (
            segment_site.point_a
            if segment_site.vert_a == corner_id
            else segment_site.point_b
        )
        tangent = _norm2(_sub2(segment_site.point_b, segment_site.point_a))
        if tangent is None:
            return None
        transition_direction = _sub2(
            owner_side.segment[1], owner_side.segment[0]
        )
        transition_length2 = _dot2(
            transition_direction, transition_direction
        )
        if transition_length2 <= 1e-20:
            return None
        canonical_factor = station / owner_side.station_extent
        canonical_point = (
            owner_side.segment[0][0]
            + transition_direction[0] * canonical_factor,
            owner_side.segment[0][1]
            + transition_direction[1] * canonical_factor,
        )
        # Crop-boundary является первичным источником T7-P3.9. Для
        # diagram-boundary сохраняем аналитический перпендикуляр через site
        # endpoint: оба кандидата сравниваются в owner-space одной метрикой.
        supporting_lines = [
            (anchor, (-tangent[1], tangent[0])),
        ]
        crop_points = tuple(corner_face.crop.points)
        for first, second in zip(
            crop_points, crop_points[1:] + crop_points[:1]
        ):
            crop_direction = _norm2(_sub2(second, first))
            if crop_direction is not None:
                supporting_lines.append((first, crop_direction))
        candidates = []
        for line_anchor, direction in supporting_lines:
            intersection = _line_intersection(
                line_anchor,
                direction,
                owner_side.segment[0],
                transition_direction,
            )
            if intersection is None:
                continue
            candidates.append(
                (
                    _dist2(intersection, canonical_point),
                    round(_dist2(intersection, owner_side.segment[0]), 12),
                    intersection,
                    direction,
                    corner_face,
                    segment_face,
                )
            )
        return (
            min(
                candidates,
                key=lambda item: (
                    item[:4],
                    int(item[4].site.edge_index),
                    int(item[5].site.edge_index),
                ),
            )
            if candidates
            else None
        )

    def endpoint_supporting_candidate(
        owner_side,
        station,
        corner_id,
        corner_face,
        segment_face,
    ):
        """T7-P3.11e: меряет узел до crop-кривой, не до её пересечения с T."""

        transition_direction = _sub2(
            owner_side.segment[1], owner_side.segment[0]
        )
        if _dot2(transition_direction, transition_direction) <= 1e-20:
            return None
        canonical_factor = station / owner_side.station_extent
        canonical_point = (
            owner_side.segment[0][0]
            + transition_direction[0] * canonical_factor,
            owner_side.segment[0][1]
            + transition_direction[1] * canonical_factor,
        )
        corner_site = corner_face.site
        if corner_id == corner_site.vert_a:
            corner_origin = corner_site.point_a
        elif corner_id == corner_site.vert_b:
            corner_origin = corner_site.point_b
        else:
            return None
        candidates = []
        crop_points = tuple(corner_face.crop.points)
        for first, second in zip(
            crop_points, crop_points[1:] + crop_points[:1]
        ):
            direction = _norm2(_sub2(second, first))
            if direction is None:
                continue
            curve_distance, factor = _segment_point_distance2(
                first, second, canonical_point
            )
            continuation_index = max(
                range(2),
                key=lambda index: (
                    _dist2((first, second)[index], corner_origin), index
                ),
            )
            candidates.append(
                (
                    curve_distance,
                    factor,
                    continuation_index,
                    first,
                    second,
                    canonical_point,
                    corner_face,
                    segment_face,
                )
            )
        return (
            min(
                candidates,
                key=lambda item: (
                    item[:5],
                    int(item[6].site.edge_index),
                    int(item[7].site.edge_index),
                ),
            )
            if candidates
            else None
        )

    imports_by_domain = {}
    processed = set()
    for transition_id, sides in sorted(
        sides_by_transition.items(), key=lambda item: repr(item[0])
    ):
        if len(sides) != 2:
            continue
        exemplar_side = sides[min(sides)]
        intervals = interval_records(exemplar_side)
        for previous, current in zip(intervals, intervals[1:]):
            station = previous[1]
            if (
                station != current[0]
                or previous[3] is None
                or current[3] is None
                or previous[3] == current[3]
            ):
                continue
            classes = (previous[3], current[3])
            corner_classes = tuple(
                item
                for item in classes
                if isinstance(item, tuple) and item[:1] == ("CORNER",)
            )
            if len(corner_classes) != 1 or ("SEGMENT",) not in classes:
                if previous[2] != current[2]:
                    raise ValueError(
                        "ATLAS_CLASS_DESYNC: unsupported predicate pair"
                    )
                continue
            corner_class = corner_classes[0]
            owner_id = corner_class[2]
            if (
                diagnostics is not None
                and owner_id
                in diagnostics.semantic_transport_disabled_owner_ids
            ):
                continue
            if not (
                isinstance(owner_id, tuple)
                and owner_id[:1] == ("corner",)
            ):
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: corner predicate owner is missing"
                )
            corner_id = int(owner_id[1])
            corner_interval = (
                previous if previous[3] == corner_class else current
            )
            segment_interval = (
                previous if previous[3] == ("SEGMENT",) else current
            )
            owner_chart_id = None
            for chart_id, (surface, _entries) in chart_records.items():
                candidate = dict(
                    surface.semantic_owner_chart_by_vertex
                ).get(corner_id)
                if candidate is not None:
                    owner_chart_id = int(candidate)
                    break
            if owner_chart_id is None:
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: predicate owner chart is missing "
                    f"transition={transition_id!r} station={station!r} "
                    f"corner={corner_id!r} owner_chart={owner_chart_id!r} "
                    f"side_charts={tuple(sorted(sides))!r} "
                    f"classes={classes!r}"
                )
            if owner_chart_id not in sides:
                transition_key = exemplar_side.transition_key
                transition_identity = _hashable_provenance(
                    transition_key
                )
                anchored_charts = {
                    int(chart_id)
                    for chart_id in sides
                    if any(
                        int(semantic.owner_chart_id) == owner_chart_id
                        and _m1_crop_semantic_class(
                            semantic.owner_crop
                        )
                        == corner_class
                        and any(
                            _hashable_provenance(anchor_key)
                            == transition_identity
                            and int(anchor_station) == int(station)
                            for _endpoint, anchor_key, anchor_station
                            in semantic.anchor_stations
                        )
                        for semantic in semantic_imports.get(
                            id(chart_records[int(chart_id)][0].domain), ()
                        )
                    )
                }
                if anchored_charts == set(map(int, sides)):
                    # T7-P2.2(a)/T7-P3.15: multi-hop crop image уже
                    # принесла boundary владельца в оба локальных графа.
                    # Повторный analytic separator здесь не строится.
                    continue
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: multi-hop predicate boundary is "
                    "missing "
                    f"transition={transition_id!r} station={station!r} "
                    f"corner={corner_id!r} owner_chart={owner_chart_id!r} "
                    f"side_charts={tuple(sorted(sides))!r} "
                    f"anchored={tuple(sorted(anchored_charts))!r}"
                )
            owner_surface, owner_entries = chart_records[owner_chart_id]
            owner_side = sides[owner_chart_id]
            target_chart_id = next(
                chart_id for chart_id in sorted(sides)
                if chart_id != owner_chart_id
            )
            target_surface = chart_records[target_chart_id][0]
            target_side = sides[target_chart_id]
            corner_faces = tuple(
                pending_face
                for _pending_index, pending_face in owner_entries
                if _m1_crop_semantic_class(pending_face.crop)
                == corner_class
                and int(pending_face.site.edge_index)
                == int(corner_interval[2])
            )
            segment_faces = tuple(
                pending_face
                for _pending_index, pending_face in owner_entries
                if pending_face.crop.kind == "SEGMENT"
                and int(pending_face.site.edge_index)
                == int(segment_interval[2])
            )
            candidates = tuple(
                candidate
                for corner_face in corner_faces
                for segment_face in segment_faces
                for candidate in (
                    supporting_candidate(
                        owner_side,
                        station,
                        corner_id,
                        corner_face,
                        segment_face,
                    ),
                )
                if candidate is not None
            )
            if not candidates:
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: predicate supporting curve is missing"
                )
            candidate = min(
                candidates,
                key=lambda item: (
                    item[:4],
                    int(item[4].site.edge_index),
                    int(item[5].site.edge_index),
                ),
            )
            anchor_tolerance = max(
                float(owner_surface.diagram_transform.quantum) * 1.5,
                float(owner_surface.domain.alpha_budget) * 0.02,
            )
            if candidate[0] > anchor_tolerance:
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: predicate curve misses T station "
                    f"transition={owner_side.transition_key!r} "
                    f"station={station} error={candidate[0]!r} "
                    f"tolerance={anchor_tolerance!r} "
                    f"corner={corner_id} "
                    f"tokens={(corner_interval[2], segment_interval[2])!r}"
                )
            _error, _distance, owner_anchor, direction, corner_face, _ = candidate
            owner_to_target = _m1_segment_isometry(
                owner_side.segment, target_side.segment
            )
            owner_frame_to_target = _m1_segment_frame_transform(
                owner_side.segment, target_side.segment
            )
            mapped_anchor = _affine_point(owner_to_target, owner_anchor)
            reach = max(
                float(owner_surface.diagram_transform.quantum),
                float(owner_surface.domain.alpha_budget),
            )
            owner_endpoints = tuple(
                (
                    owner_anchor[0] + sign * direction[0] * reach,
                    owner_anchor[1] + sign * direction[1] * reach,
                )
                for sign in (1.0, -1.0)
            )
            owner_transition_direction = _sub2(
                owner_side.segment[1], owner_side.segment[0]
            )
            owner_candidates = tuple(
                (
                    _cross2(
                        owner_transition_direction,
                        _sub2(endpoint, owner_anchor),
                    )
                    * owner_side.interior_sign,
                    endpoint,
                )
                for endpoint in owner_endpoints
            )
            owner_interior_score, owner_endpoint = max(owner_candidates)
            if owner_interior_score <= 0.0:
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: predicate continuation leaves owner"
                )
            mapped_endpoints = tuple(
                _affine_point(owner_to_target, point)
                for point in owner_endpoints
            )
            target_direction = _sub2(
                target_side.segment[1], target_side.segment[0]
            )
            target_candidates = tuple(
                (
                    _cross2(
                        target_direction,
                        _sub2(endpoint, mapped_anchor),
                    )
                    * target_side.interior_sign,
                    endpoint,
                )
                for endpoint in mapped_endpoints
            )
            interior_score, mapped_endpoint = max(target_candidates)
            if interior_score <= 0.0:
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: predicate continuation leaves target"
                )
            owner_edge_indices = tuple(
                sorted(
                    {
                        int(corner_interval[2]),
                        int(segment_interval[2]),
                    }
                )
            )
            import_key = (
                transition_id,
                station,
                owner_chart_id,
                target_chart_id,
                corner_class,
                int(corner_interval[2]),
                int(segment_interval[2]),
            )
            if import_key in processed:
                continue
            processed.add(import_key)
            identity_transform = (1.0, 0.0, 0.0, 0.0)
            identity_frame = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
            imports_by_domain.setdefault(
                id(owner_surface.domain), {}
            )[import_key + ("OWNER",)] = _M1ImportedSemantic(
                transition_key=owner_side.transition_key,
                owner_chart_id=owner_chart_id,
                target_chart_id=owner_chart_id,
                points=(owner_anchor, owner_endpoint),
                owner_crop=corner_face.crop,
                owner_site=corner_face.site,
                owner_edge_indices=owner_edge_indices,
                owner_region_points=corner_face.crop.points,
                local_to_owner=identity_transform,
                uv_local_to_owner=identity_frame,
                anchor_stations=(
                    (0, owner_side.transition_key, station),
                ),
                is_predicate_separator=True,
            )
            imports_by_domain.setdefault(
                id(target_surface.domain), {}
            )[import_key + ("TARGET",)] = _M1ImportedSemantic(
                transition_key=owner_side.transition_key,
                owner_chart_id=owner_chart_id,
                target_chart_id=target_chart_id,
                points=(mapped_anchor, mapped_endpoint),
                owner_crop=corner_face.crop,
                owner_site=corner_face.site,
                owner_edge_indices=owner_edge_indices,
                owner_region_points=corner_face.crop.points,
                local_to_owner=_affine_inverse(owner_to_target),
                uv_local_to_owner=_frame_affine_inverse(
                    owner_frame_to_target
                ),
                anchor_stations=(
                    (0, owner_side.transition_key, station),
                ),
                is_predicate_separator=True,
            )
    # T7-P3.11: обязательный separator вокруг общего source endpoint.
    endpoint_groups = {}
    for domain_id, sides in transition_contract.items():
        for side in sides:
            source_edge = tuple(side.transition_key[2])
            owners = {
                (first, second): owner
                for first, second, owner in side.interval_owners
            }
            semantics = tuple(side.interval_semantics)
            if not semantics:
                continue
            for source_vertex, interval in (
                (source_edge[0], semantics[0]),
                (source_edge[1], semantics[-1]),
            ):
                first, second, semantic_class = interval
                owner = owners.get((first, second))
                if owner is None or semantic_class is None:
                    continue
                endpoint_groups.setdefault(
                    (int(domain_id), int(side.chart_id), int(source_vertex)),
                    [],
                ).append((side, int(owner), semantic_class))

    for (domain_id, consumer_chart_id, source_vertex), records in sorted(
        endpoint_groups.items()
    ):
        classes = tuple(sorted({item[2] for item in records}, key=repr))
        if len(classes) <= 1:
            continue
        corner_classes = tuple(
            item
            for item in classes
            if isinstance(item, tuple) and item[:1] == ("CORNER",)
        )
        if len(corner_classes) != 1 or ("SEGMENT",) not in classes:
            raise ValueError(
                "ATLAS_CLASS_DESYNC: unsupported endpoint predicate pair"
            )
        corner_class = corner_classes[0]
        corner_owner = corner_class[2]
        if (
            not isinstance(corner_owner, tuple)
            or len(corner_owner) != 2
            or corner_owner[0] != "corner"
        ):
            raise ValueError(
                "ATLAS_CLASS_DESYNC: endpoint semantic owner is inconsistent "
                f"domain={domain_id} chart={consumer_chart_id} "
                f"source_vertex={source_vertex} classes={classes!r} "
                f"records={records!r}"
            )
        corner_owner_vertex = int(corner_owner[1])
        semantic_owner_chart_id = next(
            (
                int(declared)
                for surface, _entries in chart_records.values()
                for declared in (
                    dict(surface.semantic_owner_chart_by_vertex).get(
                        corner_owner_vertex
                    ),
                )
                if declared is not None
            ),
            None,
        )
        if semantic_owner_chart_id is None:
            raise ValueError(
                "ATLAS_CLASS_DESYNC: endpoint semantic owner is missing"
            )
        owner_surface, owner_entries = chart_records[
            semantic_owner_chart_id
        ]
        consumer_surface, _consumer_entries = chart_records[
            consumer_chart_id
        ]
        bridge = next(
            (
                (
                    sides_by_transition[
                        _hashable_provenance(consumer_side.transition_key)
                    ][semantic_owner_chart_id],
                    consumer_side,
                )
                for consumer_side, _owner, _class in records
                if semantic_owner_chart_id
                in sides_by_transition.get(
                    _hashable_provenance(consumer_side.transition_key), {}
                )
            ),
            None,
        )
        if bridge is None:
            raise ValueError(
                "ATLAS_CLASS_DESYNC: endpoint owner path is missing"
            )
        owner_side, consumer_side = bridge
        owner_station = (
            0
            if source_vertex == tuple(owner_side.transition_key[2])[0]
            else owner_side.station_extent
        )
        corner_tokens = {
            owner
            for _side, owner, semantic_class in records
            if semantic_class == corner_class
        }
        segment_tokens = {
            owner
            for _side, owner, semantic_class in records
            if semantic_class == ("SEGMENT",)
        }
        corner_faces = tuple(
            pending_face
            for _index, pending_face in owner_entries
            if _m1_crop_semantic_class(pending_face.crop) == corner_class
            and int(pending_face.site.edge_index) in corner_tokens
        )
        segment_faces = tuple(
            pending_face
            for _index, pending_face in owner_entries
            if pending_face.crop.kind == "SEGMENT"
            and int(pending_face.site.edge_index) in segment_tokens
        )
        candidates = tuple(
            candidate
            for corner_face in corner_faces
            for segment_face in segment_faces
            for candidate in (
                endpoint_supporting_candidate(
                    owner_side,
                    owner_station,
                    corner_owner_vertex,
                    corner_face,
                    segment_face,
                ),
            )
            if candidate is not None
        )
        if not candidates:
            raise ValueError(
                "ATLAS_CLASS_DESYNC: endpoint supporting curve is missing "
                f"domain={domain_id} chart={consumer_chart_id} "
                f"source_vertex={source_vertex} "
                f"corner_owner_vertex={corner_owner_vertex} "
                f"owner_chart={semantic_owner_chart_id} "
                f"corner_tokens={sorted(corner_tokens)!r} "
                f"segment_tokens={sorted(segment_tokens)!r} "
                f"corner_faces={len(corner_faces)} "
                f"segment_faces={len(segment_faces)} records={records!r}"
            )
        candidate = min(
            candidates,
            key=lambda item: (
                item[:5],
                int(item[6].site.edge_index),
                int(item[7].site.edge_index),
            ),
        )
        owner_quantum = max(
            float(owner_surface.diagram_transform.quantum), 1e-10
        )
        incidence_epsilon = max(1e-12, owner_quantum * 1e-9)
        if candidate[0] > owner_quantum + incidence_epsilon:
            raise ValueError(
                "ATLAS_CLASS_DESYNC: endpoint predicate misses source vertex "
                f"domain={domain_id} chart={consumer_chart_id} "
                f"source_vertex={source_vertex} "
                f"corner_owner_vertex={corner_owner_vertex} "
                f"owner_chart={semantic_owner_chart_id} "
                f"error={candidate[0]!r} quantum={owner_quantum!r} "
                f"factor={candidate[1]!r} curve={candidate[3:5]!r} "
                f"transition={owner_side.transition_key!r} "
                f"station={owner_station} records={records!r}"
            )
        (
            _curve_distance,
            curve_factor,
            continuation_index,
            first,
            second,
            owner_anchor,
            corner_face,
            _face,
        ) = candidate
        # T3 endpoint-move: только субквантовый конец переходит в общий
        # source-vertex key; кривая обрезается в ближайшей к узлу точке,
        # а её продолжение от semantic owner остаётся неизменным.
        owner_endpoint = (first, second)[continuation_index]
        owner_to_consumer = _m1_segment_isometry(
            owner_side.segment, consumer_side.segment
        )
        frame_to_consumer = _m1_segment_frame_transform(
            owner_side.segment, consumer_side.segment
        )
        consumer_anchor = _affine_point(owner_to_consumer, owner_anchor)
        consumer_endpoint = _affine_point(owner_to_consumer, owner_endpoint)
        owner_edge_indices = tuple(sorted(corner_tokens | segment_tokens))
        endpoint_key = (
            "ENDPOINT",
            consumer_chart_id,
            source_vertex,
            semantic_owner_chart_id,
            corner_class,
        )
        identity_transform = (1.0, 0.0, 0.0, 0.0)
        identity_frame = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        imports_by_domain.setdefault(
            id(owner_surface.domain), {}
        )[endpoint_key + ("OWNER",)] = _M1ImportedSemantic(
            transition_key=owner_side.transition_key,
            owner_chart_id=semantic_owner_chart_id,
            target_chart_id=semantic_owner_chart_id,
            points=(owner_anchor, owner_endpoint),
            owner_crop=corner_face.crop,
            owner_site=corner_face.site,
            owner_edge_indices=owner_edge_indices,
            owner_region_points=corner_face.crop.points,
            local_to_owner=identity_transform,
            uv_local_to_owner=identity_frame,
            anchor_stations=((0, owner_side.transition_key, owner_station),),
            is_predicate_separator=True,
            is_endpoint_separator=True,
        )
        imports_by_domain.setdefault(
            id(consumer_surface.domain), {}
        )[endpoint_key + ("CONSUMER",)] = _M1ImportedSemantic(
            transition_key=consumer_side.transition_key,
            owner_chart_id=semantic_owner_chart_id,
            target_chart_id=consumer_chart_id,
            points=(consumer_anchor, consumer_endpoint),
            owner_crop=corner_face.crop,
            owner_site=corner_face.site,
            owner_edge_indices=owner_edge_indices,
            owner_region_points=corner_face.crop.points,
            local_to_owner=_affine_inverse(owner_to_consumer),
            uv_local_to_owner=_frame_affine_inverse(frame_to_consumer),
            anchor_stations=((0, consumer_side.transition_key, owner_station),),
            is_predicate_separator=True,
            is_endpoint_separator=True,
        )

    def anchor_reached_transitions(semantic):
        target_surface = chart_records[int(semantic.target_chart_id)][0]
        quantum = max(
            float(target_surface.diagram_transform.quantum), 1e-10
        )
        points = tuple(semantic.points)
        if len(points) != 2:
            raise ValueError(
                "ATLAS_ARRANGEMENT_INTEGRITY: predicate chain is not linear"
            )
        first, second = points
        lattice_first = tuple(
            Fraction(round(value / quantum)) for value in first
        )
        lattice_second = tuple(
            Fraction(round(value / quantum)) for value in second
        )
        intersections = {
            Fraction(0): [float(first[0]), float(first[1]), []],
            Fraction(1): [float(second[0]), float(second[1]), []],
        }
        for endpoint_index, transition_key, station in semantic.anchor_stations:
            factor = Fraction(0 if endpoint_index == 0 else 1)
            intersections[factor][2].append((transition_key, int(station)))
        anchored_transition_keys = {
            _hashable_provenance(transition_key)
            for _endpoint_index, transition_key, _station
            in semantic.anchor_stations
        }
        for side in transition_contract.get(id(target_surface.domain), ()):
            if (
                _hashable_provenance(side.transition_key)
                in anchored_transition_keys
            ):
                continue
            edge_a = tuple(
                Fraction(round(value / quantum))
                for value in side.segment[0]
            )
            edge_b = tuple(
                Fraction(round(value / quantum))
                for value in side.segment[1]
            )
            hit = _m1_exact_segment_intersection(
                lattice_first, lattice_second, edge_a, edge_b
            )
            if hit is None or hit == "COLLINEAR":
                continue
            segment_factor, edge_factor, _point = hit
            declared_station = round(
                float(edge_factor) * side.station_extent
            )
            station = min(
                side.stations,
                key=lambda value: (
                    abs(int(value) - declared_station), int(value)
                ),
            )
            if abs(int(station) - declared_station) > 1:
                # Первая fixed-point итерация сама публикует новый T-key;
                # следующий contract обязан прочитать уже его.
                station = max(
                    0, min(side.station_extent, declared_station)
                )
            canonical_factor = int(station) / side.station_extent
            canonical_point = (
                side.segment[0][0]
                + (side.segment[1][0] - side.segment[0][0])
                * canonical_factor,
                side.segment[0][1]
                + (side.segment[1][1] - side.segment[0][1])
                * canonical_factor,
            )
            record = intersections.setdefault(
                segment_factor,
                [canonical_point[0], canonical_point[1], []],
            )
            record[0] = canonical_point[0]
            record[1] = canonical_point[1]
            record[2].append((side.transition_key, int(station)))
        rebuilt = []
        anchors = []
        for point_index, (_factor, record) in enumerate(
            sorted(intersections.items())
        ):
            rebuilt.append((record[0], record[1]))
            anchors.extend(
                (point_index, transition_key, station)
                for transition_key, station in record[2]
            )
        return replace(
            semantic,
            points=tuple(rebuilt),
            anchor_stations=tuple(sorted(set(anchors), key=repr)),
        )

    result = {
        domain_id: tuple(
            sorted(
                (
                    anchor_reached_transitions(semantic)
                    for semantic in imports.values()
                ),
                key=lambda item: (
                    repr(item.transition_key),
                    repr(item.owner_crop.semantic_owner_id),
                    item.points,
                ),
            )
        )
        for domain_id, imports in imports_by_domain.items()
    }
    return result


def _m1_validate_separator_requirements(
    transition_contract, predicate_imports
):
    """T7-P3.9: каждая разрешимая class-change имеет anchor в обоих graphs."""

    requirements = {}
    for domain_id, sides in transition_contract.items():
        for side in sides:
            owners = {
                (first, second): owner
                for first, second, owner in side.interval_owners
            }
            intervals = tuple(
                (
                    first,
                    second,
                    owners.get((first, second)),
                    semantic_class,
                )
                for first, second, semantic_class in side.interval_semantics
            )
            for previous, current in zip(intervals, intervals[1:]):
                if (
                    previous[1] != current[0]
                    or previous[3] is None
                    or current[3] is None
                    or previous[3] == current[3]
                ):
                    continue
                classes = (previous[3], current[3])
                corner_count = sum(
                    isinstance(item, tuple) and item[:1] == ("CORNER",)
                    for item in classes
                )
                if corner_count != 1 or ("SEGMENT",) not in classes:
                    continue
                requirement_key = (
                    _hashable_provenance(side.transition_key),
                    int(previous[1]),
                    tuple(sorted(classes, key=repr)),
                )
                requirements.setdefault(requirement_key, set()).add(
                    int(domain_id)
                )

    for (transition_id, station, classes), domain_ids in sorted(
        requirements.items(), key=lambda item: repr(item[0])
    ):
        for domain_id in sorted(domain_ids):
            anchored = any(
                _hashable_provenance(transition_key) == transition_id
                and int(anchor_station) == station
                for semantic in predicate_imports.get(domain_id, ())
                for _endpoint, transition_key, anchor_station
                in semantic.anchor_stations
            )
            if not anchored:
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: class separator requirement is "
                    "missing before arrangement "
                    f"transition={transition_id!r} station={station} "
                    f"classes={classes!r} domain={domain_id}"
                )


def _m1_semantic_class(face):
    """T8 equivalence class без повторного вывода corner-семантики."""

    if face.crop.kind == "SEGMENT":
        return ("SEGMENT",)
    return (
        "CORNER",
        face.crop.kind,
        face.crop.semantic_owner_id,
    )


def _m1_transition_semantic_class(record):
    """T7-P.5: T-collinear semantic boundary принадлежит strip-классу."""

    face = record[0]
    semantic_class = _m1_semantic_class(face)
    if semantic_class == ("SEGMENT",):
        return semantic_class
    semantic_points = face.crop.points
    if len(semantic_points) < 2:
        return semantic_class
    first = record[3]
    second = record[4]
    if face.uv_point_transform is not None:
        first = _affine_point(face.uv_point_transform, first)
        second = _affine_point(face.uv_point_transform, second)
    quantum = max(float(face.surface.diagram_transform.quantum), 1e-10)
    midpoint = (
        (first[0] + second[0]) * 0.5,
        (first[1] + second[1]) * 0.5,
    )
    endpoint_edge = any(
        isinstance(point_key, tuple)
        and point_key[:1] == ("m1-source-vertex",)
        for point_key in record[1:3]
    )
    short_boundary_edge = (
        endpoint_edge
        and _dist2(first, second) <= quantum * 4.0
    )
    for boundary_a, boundary_b in zip(
        semantic_points,
        semantic_points[1:] + semantic_points[:1],
    ):
        if (
            _segment_point_distance2(boundary_a, boundary_b, first)[0]
            <= quantum * 1.5
            and _segment_point_distance2(boundary_a, boundary_b, second)[0]
            <= quantum * 1.5
        ) or (
            short_boundary_edge
            and _segment_point_distance2(
                boundary_a, boundary_b, midpoint
            )[0]
            <= quantum * 4.0
        ):
            return ("SEGMENT",)
    return semantic_class


def _m1_raw_arrangement_uv(face, point, alpha):
    """Нормализованный U и network V в authoring-frame face."""

    uv_site = face.uv_site or face.site
    uv_point = (
        _frame_affine_point(face.uv_frame_transform, point)
        if face.uv_frame_transform is not None
        else _affine_point(face.uv_point_transform, point)
        if face.uv_point_transform is not None
        else point
    )
    raw_parameter = _site_unbounded_parameter(uv_site, uv_point)
    continuation = face.uv_v_continuation_endpoint
    terminal_continuation = (
        (
            raw_parameter < 0.0
            and uv_site.vert_a in face.terminal_cut_vertices
        )
        or (
            raw_parameter > 1.0
            and uv_site.vert_b in face.terminal_cut_vertices
        )
    )
    if (
        terminal_continuation
        or continuation < 0
        and raw_parameter < 0.0
        or continuation > 0
        and raw_parameter > 1.0
    ):
        parameter = raw_parameter
    else:
        _distance, parameter = _segment_point_distance2(
            uv_site.point_a, uv_site.point_b, uv_point
        )
    component_uv = _crop_component_uv(face.crop, uv_point)
    if component_uv is not None:
        return component_uv
    return (
        _site_lateral_u(uv_site, uv_point, alpha),
        _site_v_length(uv_site, parameter),
    )


def _m1_arrangement_uv(face, point, point_key, alpha):
    """Физический U и network V с каноническим T override."""

    override = next(
        (
            (u_fraction, v_length)
            for key, u_fraction, v_length in face.transition_uv
            if key == point_key
        ),
        None,
    )
    u_fraction, v_length = (
        override
        if override is not None
        else _m1_raw_arrangement_uv(face, point, alpha)
    )
    return u_fraction * alpha, v_length


def _m1_raw_physical_arrangement_uv(face, point, alpha):
    """Сырой физический UV до R2-канонизации transition-станции."""

    u_fraction, v_length = _m1_raw_arrangement_uv(face, point, alpha)
    return u_fraction * alpha, v_length


def _m1_validate_semantic_transitions(
    transition_edge_records,
    station_by_point,
    alpha,
    diagnostics=None,
):
    """T8 O1--O3: semantic/UV согласие канонического T-графа."""

    paired_by_transition = {}
    touches_by_face = {}
    for edge_key, records in sorted(transition_edge_records.items()):
        if len(records) != 2:
            continue
        first, second = records
        first_face = first[0]
        second_face = second[0]
        if first_face.surface.domain.chart_id == second_face.surface.domain.chart_id:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: T8 O1 duplicate chart owner "
                f"edge={edge_key!r}"
            )
        first_class = _m1_semantic_class(first_face)
        second_class = _m1_semantic_class(second_face)
        raw_first_class = first_class
        raw_second_class = second_class
        collinear_boundary = False
        if first_class != second_class:
            first_class = _m1_transition_semantic_class(first)
            second_class = _m1_transition_semantic_class(second)
            collinear_boundary = first_class == second_class
            if first_class != second_class:
                face_diagnostics = tuple(
                    (
                        face.surface.domain.chart_id,
                        face.site.edge_index,
                        face.crop.kind,
                        face.crop.side,
                        face.transition_uv,
                    )
                    for face in (first_face, second_face)
                )
                raise ValueError(
                    "ATLAS_CLASS_DESYNC: T8 O1 semantic mismatch "
                    f"edge={edge_key!r} classes="
                    f"{(first_class, second_class)!r} "
                    f"faces={face_diagnostics!r}"
                )
        first_uv = {
            first[1]: _m1_arrangement_uv(
                first_face, first[3], first[1], alpha
            ),
            first[2]: _m1_arrangement_uv(
                first_face, first[4], first[2], alpha
            ),
        }
        second_uv = {
            second[1]: _m1_arrangement_uv(
                second_face, second[3], second[1], alpha
            ),
            second[2]: _m1_arrangement_uv(
                second_face, second[4], second[2], alpha
            ),
        }
        quantum = max(
            float(first_face.surface.diagram_transform.quantum),
            float(second_face.surface.diagram_transform.quantum),
            1e-10,
        )
        raw_first_uv = {
            first[1]: _m1_raw_physical_arrangement_uv(
                first_face, first[3], alpha
            ),
            first[2]: _m1_raw_physical_arrangement_uv(
                first_face, first[4], alpha
            ),
        }
        raw_second_uv = {
            second[1]: _m1_raw_physical_arrangement_uv(
                second_face, second[3], alpha
            ),
            second[2]: _m1_raw_physical_arrangement_uv(
                second_face, second[4], alpha
            ),
        }
        # R2 вправе публиковать одну owner-станцию, но не вправе скрывать
        # holonomy/изометрическую ошибку больше принятого E2-бюджета.
        raw_uv_tolerance = max(quantum * 1.5, abs(float(alpha)) * 0.02)
        for point_key in first_uv.keys() & second_uv.keys():
            canonical_uv = first_uv[point_key]
            for side_name, raw_uv in (
                ("first", raw_first_uv[point_key]),
                ("second", raw_second_uv[point_key]),
            ):
                if any(
                    abs(raw_value - canonical_value) > raw_uv_tolerance
                    for raw_value, canonical_value in zip(
                        raw_uv, canonical_uv
                    )
                ):
                    raise ValueError(
                        "ATLAS_TRANSITION_DESYNC: T8 O1 raw UV mismatch "
                        f"edge={edge_key!r} point={point_key!r} "
                            f"side={side_name} raw={raw_uv!r} "
                            f"canonical={canonical_uv!r} "
                            f"tolerance={raw_uv_tolerance!r} "
                            f"raw_sides={(raw_first_uv, raw_second_uv)!r} "
                            f"frames={tuple((face.surface.domain.chart_id, face.site.edge_index, face.crop.kind, face.crop.semantic_owner_id, face.uv_site.edge_index if face.uv_site is not None else None, face.uv_frame_owner_chart_id, face.uv_frame_is_imported, face.uv_frame_transform) for face in (first_face, second_face))!r}"
                        )
            if any(
                abs(first_value - second_value) > quantum * 1.5
                for first_value, second_value in zip(
                    first_uv[point_key], second_uv[point_key]
                )
            ):
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: T8 O1 UV mismatch "
                    f"edge={edge_key!r} point={point_key!r} "
                    f"uv={(first_uv[point_key], second_uv[point_key])!r}"
                )
        for record, other, record_class, other_class in (
            (
                first,
                second,
                raw_first_class if collinear_boundary else first_class,
                raw_second_class if collinear_boundary else second_class,
            ),
            (
                second,
                first,
                raw_second_class if collinear_boundary else second_class,
                raw_first_class if collinear_boundary else first_class,
            ),
        ):
            touches_by_face.setdefault(id(record[0]), []).append(
                (
                    record[0],
                    other[0],
                    edge_key,
                    record_class,
                    other_class,
                )
            )
        transition_id = first[5]
        start = station_by_point.get((repr(transition_id), first[1]))
        end = station_by_point.get((repr(transition_id), first[2]))
        if (
            not collinear_boundary
            and start is not None
            and end is not None
            and start != end
        ):
            paired_by_transition.setdefault(repr(transition_id), []).append(
                (
                    min(start, end),
                    max(start, end),
                    first_class,
                    {
                        first_face.surface.domain.chart_id: first_face,
                        second_face.surface.domain.chart_id: second_face,
                    },
                )
            )
        if diagnostics is not None:
            diagnostics.atlas_semantic_transition_count += 1

    # O2: одна грань не может одновременно потреблять разные corner-классы.
    for face, touches in (
        (items[0][0], items) for items in touches_by_face.values()
    ):
        classes = {item[3] for item in touches}
        if len(classes) > 1:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: T8 O2 multi-owner face "
                f"chart={face.surface.domain.chart_id} "
                f"classes={tuple(sorted(map(repr, classes)))} "
                f"touches={tuple((item[2], item[3], item[4]) for item in touches)!r} "
                f"points={face.points!r}"
            )

    # O3: при смене класса обе chart-local face обязаны смениться в общей
    # станции; это и есть инцидентный separator в каждом локальном графе.
    for transition_id, intervals in paired_by_transition.items():
        ordered = sorted(intervals, key=lambda item: (item[0], item[1]))
        for previous, current in zip(ordered, ordered[1:]):
            if previous[1] != current[0] or previous[2] == current[2]:
                continue
            shared_charts = previous[3].keys() & current[3].keys()
            if not shared_charts or any(
                previous[3][chart_id] is current[3][chart_id]
                for chart_id in shared_charts
            ):
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: T8 O3 missing separator "
                    f"transition={transition_id!r} "
                    f"station={current[0]}"
                )


def _m1_validate_materialized_transition_coverage(
    transition_edge_records,
    station_by_point,
    transition_contract,
    diagnostics=None,
):
    """T2-C: парный materialized subedge не может жить над owner=None."""

    sides_by_transition = {}
    for sides in transition_contract.values():
        for side in sides:
            transition_id = _hashable_provenance(side.transition_key)
            sides_by_transition.setdefault(transition_id, []).append(side)
    for edge_key, records in sorted(transition_edge_records.items()):
        if len(records) != 2:
            continue
        transition_id = records[0][5]
        stations = tuple(
            station_by_point.get((repr(transition_id), point_key))
            for point_key in records[0][1:3]
        )
        if None in stations or stations[0] == stations[1]:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: materialized T2 interval "
                "has no canonical station span"
            )
        span_start, span_end = sorted(int(station) for station in stations)
        side_owner_sequences = []
        for side in sorted(
            sides_by_transition.get(transition_id, ()),
            key=lambda candidate: candidate.chart_id,
        ):
            overlapping = tuple(
                (start, end, owner)
                for start, end, owner in side.interval_owners
                if max(span_start, start) < min(span_end, end)
            )
            if not overlapping:
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: materialized subedge is "
                    "outside the published T2 contract"
                )
            if any(owner is None for _start, _end, owner in overlapping):
                if diagnostics is not None:
                    diagnostics.atlas_undeclared_materialized_interval_count += 1
                raise ValueError(
                    "ATLAS_TRANSITION_DESYNC: owner=None under materialized "
                    f"transition subedge edge={edge_key!r} "
                    f"span={(span_start, span_end)!r}"
                )
            side_owner_sequences.append(overlapping)
        if len(side_owner_sequences) != 2:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: materialized transition lacks "
                "two published T2 sides"
            )
        if side_owner_sequences[0] != side_owner_sequences[1]:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: T2 owner declaration differs "
                "between materialized sides"
            )


def _m1_absorbed_v_continuation_endpoint(
    face,
    endpoint_sign,
    alpha,
    quantum,
    region_corner_vertices=(),
):
    """T7-P3.8: доказывает same-chain SMOOTH endpoint ниже B0-resolution."""

    if face.declared_semantic_class not in (None, ("SEGMENT",)):
        return False
    uv_site = face.uv_site or face.site
    endpoint_vertex = (
        uv_site.vert_a if endpoint_sign < 0 else uv_site.vert_b
    )
    if (
        endpoint_vertex not in face.absorbed_corner_vertices
        and endpoint_vertex not in face.boundary_corner_vertices
        and endpoint_vertex not in region_corner_vertices
    ):
        return False
    incident_indices = face.surface.sites_by_vertex.get(endpoint_vertex, ())
    if len(incident_indices) != 2:
        return False
    incident_sites = tuple(
        face.surface.sites[index] for index in incident_indices
    )
    if int(uv_site.edge_index) not in {
        int(site.edge_index) for site in incident_sites
    }:
        return False
    corner = next(
        (
            candidate
            for candidate in face.surface.corners
            if candidate.vert_index == endpoint_vertex
            and len(candidate.incident_sites) == 2
        ),
        None,
    )
    if corner is None:
        return False

    # Same-chain V-фаза должна совпадать в общей вершине до разрешения
    # континуации. На junction или разорванной phase это не SMOOTH-проход.
    endpoint_values = []
    for site in incident_sites:
        parameter = 0.0 if site.vert_a == endpoint_vertex else 1.0
        endpoint_values.append(_site_v_length(site, parameter))
    if abs(endpoint_values[0] - endpoint_values[1]) > max(
        1e-7, quantum * 1.5
    ):
        return False

    # T7-P3.8/10: авторитет — либо absorbed исход B0, либо реально
    # материализованная corner-кривая в boundary этой face. Числовой
    # tau-порог и endpoint-перпендикуляр здесь не являются предикатами.
    return True


def _m1_region_corner_authorities(faces, point_transition_ids):
    """T7-P3.12: переносит separator-authority по одному semantic region."""

    faces = tuple(faces)
    semantic_keys = tuple(
        (
            _hashable_provenance(face.declared_owner_token),
            _hashable_provenance(face.declared_semantic_class),
        )
        for face in faces
    )
    edge_records = {}

    def edge_key(face, first, second):
        return (
            int(face.surface.domain.chart_id),
            *sorted((repr(first), repr(second))),
        )

    corner_ids_by_face_edge = {}
    for face_index, face in enumerate(faces):
        for first, second, corner_ids in face.boundary_corner_edges:
            corner_ids_by_face_edge[
                (face_index, edge_key(face, first, second))
            ] = tuple(int(value) for value in corner_ids)
        for index, first in enumerate(face.point_keys):
            second = face.point_keys[(index + 1) % len(face.point_keys)]
            key = edge_key(face, first, second)
            edge_records.setdefault(key, []).append(
                (face_index, first, second)
            )

    adjacency = {index: set() for index in range(len(faces))}
    for records in edge_records.values():
        unique_indices = tuple(
            sorted({record[0] for record in records})
        )
        if len(unique_indices) != 2:
            continue
        first_index, second_index = unique_indices
        if semantic_keys[first_index] != semantic_keys[second_index]:
            continue
        representative = records[0]
        if point_transition_ids(
            representative[1]
        ) & point_transition_ids(representative[2]):
            # T4: region-authority не переходит через chart boundary.
            continue
        adjacency[first_index].add(second_index)
        adjacency[second_index].add(first_index)

    component_by_index = {}
    components = []
    for start in range(len(faces)):
        if start in component_by_index:
            continue
        component_index = len(components)
        component = set()
        queue = [start]
        while queue:
            face_index = queue.pop()
            if face_index in component:
                continue
            component.add(face_index)
            component_by_index[face_index] = component_index
            queue.extend(adjacency[face_index] - component)
        components.append(component)

    authorities = {}
    for component in components:
        exemplar = min(component)
        if semantic_keys[exemplar][1] != ("SEGMENT",):
            continue
        corner_vertices = set()
        for face_index in component:
            face = faces[face_index]
            for edge_index, first in enumerate(face.point_keys):
                second = face.point_keys[
                    (edge_index + 1) % len(face.point_keys)
                ]
                key = edge_key(face, first, second)
                neighbor_indices = {
                    record[0]
                    for record in edge_records.get(key, ())
                    if record[0] != face_index
                }
                if neighbor_indices & component:
                    continue
                corner_vertices.update(
                    corner_ids_by_face_edge.get((face_index, key), ())
                )
        for face_index in component:
            authorities[id(faces[face_index])] = tuple(
                sorted(corner_vertices)
            )
    return authorities


def _m1_validate_segment_face_parameters(
    faces,
    alpha,
    diagnostics=None,
    region_corner_authorities=None,
):
    """T2-C/T7-P3.8: запрещает clamp, тотализируя SMOOTH V-проход."""

    result = []
    region_corner_authorities = region_corner_authorities or {}
    for face in faces:
        if face.crop.kind != "SEGMENT" or not face.points:
            result.append(face)
            continue
        uv_site = face.uv_site or face.site
        representative = _m1_representative_point(face.points)
        uv_representative = (
            _frame_affine_point(
                face.uv_frame_transform, representative
            )
            if face.uv_frame_transform is not None
            else _affine_point(
                face.uv_point_transform, representative
            )
            if face.uv_point_transform is not None
            else representative
        )
        length = max(float(uv_site.segment_length), 1e-20)
        quantum = max(float(face.surface.diagram_transform.quantum), 1e-10)
        parameter = _site_unbounded_parameter(uv_site, uv_representative)
        overrun = (
            -parameter * length
            if parameter < 0.0
            else (parameter - 1.0) * length
            if parameter > 1.0
            else 0.0
        )
        if overrun <= quantum:
            result.append(face)
            continue

        continuation_endpoint = -1 if parameter < 0.0 else 1
        endpoint_vertex = (
            uv_site.vert_a
            if continuation_endpoint < 0
            else uv_site.vert_b
        )
        if endpoint_vertex in face.terminal_cut_vertices:
            result.append(
                replace(
                    face,
                    uv_v_continuation_endpoint=continuation_endpoint,
                )
            )
            continue
        if continuation_endpoint and _m1_absorbed_v_continuation_endpoint(
            face,
            continuation_endpoint,
            alpha,
            quantum,
            region_corner_authorities.get(id(face), ()),
        ):
            if diagnostics is not None:
                diagnostics.atlas_v_continuation_count += 1
            result.append(
                replace(
                    face,
                    uv_v_continuation_endpoint=continuation_endpoint,
                )
            )
            continue

        if diagnostics is not None:
            diagnostics.atlas_clamped_segment_face_count += 1
        endpoint_corner = next(
            (
                corner
                for corner in face.surface.corners
                if corner.vert_index == endpoint_vertex
            ),
            None,
        )
        raise ValueError(
            "ATLAS_TRANSITION_DESYNC: materialized SEGMENT face is "
            f"clamped beyond one quantum chart="
            f"{face.surface.domain.chart_id} site={face.site.edge_index} "
            f"overrun={overrun!r} "
            f"quantum={quantum!r} "
            f"declared_owner={face.declared_owner_token!r} "
            f"declared_class={face.declared_semantic_class!r} "
            f"uv_owner={face.uv_frame_owner_chart_id!r} "
            f"uv_site={uv_site.edge_index!r} endpoint={endpoint_vertex!r} "
            f"absorbed={face.absorbed_corner_vertices!r} "
            f"boundary_corners={face.boundary_corner_vertices!r} "
            f"incident={face.surface.sites_by_vertex.get(endpoint_vertex, ())!r} "
            f"corner_angle={getattr(endpoint_corner, 'interior_angle', None)!r} "
            f"point_keys={face.point_keys!r}"
        )
    return tuple(result)


def _resolved_corner_owner_id(model):
    return (
        "corner-model",
        int(model.seed.corner_vertex_id),
        model.seed.sector_id,
    )


def _resolve_arrangement_corner_views(faces, corner_resolution_sources):
    """S-CM.b: post-competition view читает тот же Model/DerivedGeometry."""

    sources = {
        (int(patch_id), owner_id, int(chart_id)): (model, derived)
        for patch_id, owner_id, model, derived, chart_id
        in corner_resolution_sources
    }
    points_by_source = {source_key: {} for source_key in sources}
    for face in faces:
        owner_id = face.crop.semantic_owner_id
        source_key = (
            int(face.surface.patch_id),
            owner_id,
            int(face.surface.domain.chart_id),
        )
        if source_key not in sources:
            continue
        for point_index, point in enumerate(face.points):
            if len(face.point_keys) == len(face.points):
                key = face.point_keys[point_index]
            else:
                location = face.surface.domain.locate(point)
                if location is None:
                    raise ValueError(
                        "RESOLVED_CORNER_DOMAIN_LOCATION_MISSING: "
                        f"owner={owner_id!r} point={point!r}"
                    )
                key = _domain_location_key(face.surface, location)
            canonical_point = tuple(float(value) for value in point)
            previous = points_by_source[source_key].setdefault(
                key, canonical_point
            )
            if previous != canonical_point:
                raise ValueError(
                    "RESOLVED_CORNER_KEY_POSITION_DESYNC: "
                    f"owner={owner_id!r} chart={source_key[2]!r} key={key!r}"
                )
    return tuple(
        sources[source_key][0].resolve_after_competition(
            tuple(
                sorted(
                    points_by_source[source_key].items(),
                    key=lambda item: repr(item[0]),
                )
            ),
            derived=sources[source_key][1],
            tolerance=_FRAGMENT_TOPOLOGY_TOLERANCE,
        )
        for source_key in sorted(sources, key=repr)
    )


def _build_decal_arrangement(
    pending,
    tolerance,
    diagnostics=None,
    alpha=None,
    corner_resolution_sources=(),
):
    """Создаёт conforming subdivision отдельно на каждом owner surface."""

    grouped = {}
    for pending_index, pending_face in enumerate(pending):
        domain = pending_face.surface.domain
        group_key = (
            pending_face.surface.patch_id,
            domain.kind,
            domain.chart_id,
        )
        grouped.setdefault(group_key, []).append(
            (pending_index, pending_face)
        )

    for group_key, entries in tuple(grouped.items()):
        if entries[0][1].surface.domain.admission_tier == "APPROXIMATE":
            grouped[group_key] = _m1_prepare_absorbed_corner_structures(
                entries
            )

    arranged_by_index = {}
    m1_faces = []
    inserted_stations = 0
    crop_semantic_imports = _m1_build_semantic_imports(
        grouped.values(), diagnostics
    )
    # R1 images сначала дополняют coverage. Каждый новый separator может
    # сдвинуть station election на квант, поэтому T7-P3.9 замыкается до
    # fixed point с границей R3, а не одним постфактум проходом.
    predicate_import_sets = {}
    transition_contract = _m1_build_transition_contract(
        grouped.values(), crop_semantic_imports, diagnostics
    )
    semantic_imports = dict(crop_semantic_imports)

    def predicate_import_key(semantic):
        return (
            _hashable_provenance(semantic.transition_key),
            int(semantic.owner_chart_id),
            int(semantic.target_chart_id),
            semantic.owner_crop.kind,
            semantic.owner_crop.side,
            _hashable_provenance(
                semantic.owner_crop.semantic_owner_id
            ),
            tuple(int(value) for value in semantic.owner_edge_indices),
            bool(semantic.is_predicate_separator),
            bool(semantic.is_endpoint_separator),
        )

    for _requirement_iteration in range(8):
        produced = _m1_build_predicate_boundary_imports(
            grouped.values(),
            transition_contract,
            semantic_imports,
            diagnostics,
        )
        for domain_id, imports in produced.items():
            collected = predicate_import_sets.setdefault(domain_id, {})
            for semantic in imports:
                # Fixed point заменяет предыдущую station-версию той же
                # owner-кривой; накопление полного repr оставляло stale
                # separators с уже не опубликованными T-ключами.
                collected[predicate_import_key(semantic)] = semantic
        predicate_semantic_imports = {
            domain_id: tuple(
                imports[key] for key in sorted(imports, key=repr)
            )
            for domain_id, imports in predicate_import_sets.items()
        }
        semantic_imports = {
            domain_id: tuple(crop_semantic_imports.get(domain_id, ()))
            + tuple(predicate_semantic_imports.get(domain_id, ()))
            for domain_id in (
                crop_semantic_imports.keys()
                | predicate_semantic_imports.keys()
            )
        }
        _m1_validate_separator_requirements(
            transition_contract, semantic_imports
        )
        next_contract = _m1_build_transition_contract(
            grouped.values(), semantic_imports, diagnostics
        )
        try:
            _m1_validate_separator_requirements(
                next_contract, semantic_imports
            )
        except ValueError as exc:
            if "class separator requirement is missing" not in str(exc):
                raise
            transition_contract = next_contract
            continue
        transition_contract = next_contract
        break
    else:
        raise ValueError(
            "ATLAS_CLASS_DESYNC: separator requirement iteration limit"
        )
    if diagnostics is not None:
        diagnostics.atlas_semantic_import_count += sum(
            len(imports) for imports in predicate_import_sets.values()
        )
    uv_frame_delegates = _m1_build_uv_frame_delegates(grouped.values())
    for entries in grouped.values():
        domain = entries[0][1].surface.domain
        if domain.admission_tier == "APPROXIMATE":
            faces, inserted = _m1_surface_arrangement(
                entries,
                transition_contract.get(id(domain), ()),
                diagnostics,
                semantic_imports.get(id(domain), ()),
                uv_frame_delegates.get(id(domain), {}),
            )
            m1_faces.extend(faces)
            inserted_stations += inserted
            continue
        polygons, inserted = _insert_surface_edge_stations(
            [entry[1].points for entry in entries], tolerance
        )
        inserted_stations += inserted
        if domain.kind == "INTRINSIC":
            polygons, inserted = _insert_intrinsic_triangle_stations(
                polygons, domain, tolerance
            )
            inserted_stations += inserted
        for entry, polygon in zip(entries, polygons):
            if len(polygon) < 3 or abs(_polygon_area2(polygon)) <= 1e-10:
                continue
            if _polygon_area2(polygon) < 0.0:
                polygon.reverse()
            pending_face = entry[1]
            arranged_by_index[entry[0]] = _DecalArrangementFace(
                surface=pending_face.surface,
                site=pending_face.site,
                points=tuple(polygon),
                crop=pending_face.crop,
                absorbed_corner_vertices=(
                    pending_face.absorbed_corner_vertices
                ),
                terminal_cut_vertices=pending_face.terminal_cut_vertices,
            )
    transition_ids_by_source_vertex = {}
    for sides in transition_contract.values():
        for side in sides:
            transition_id = _hashable_provenance(side.transition_key)
            for source_vertex in side.transition_key[2]:
                transition_ids_by_source_vertex.setdefault(
                    int(source_vertex), set()
                ).add(transition_id)

    def point_transition_ids(point_key):
        if (
            isinstance(point_key, tuple)
            and point_key[:1] == ("m1-transition",)
        ):
            return {point_key[1]}
        if (
            isinstance(point_key, tuple)
            and point_key[:1] == ("m1-source-vertex",)
        ):
            return transition_ids_by_source_vertex.get(
                int(point_key[1]), set()
            )
        return set()

    faces = tuple(m1_faces) + tuple(
        arranged_by_index[index] for index in sorted(arranged_by_index)
    )
    if m1_faces:
        if alpha is None:
            raise ValueError(
                "ATLAS_TRANSITION_DESYNC: M1 V validation needs alpha"
            )
        m1_faces = list(
            _m1_validate_segment_face_parameters(
                m1_faces,
                float(alpha),
                diagnostics,
                _m1_region_corner_authorities(
                    m1_faces, point_transition_ids
                ),
            )
        )
        faces = tuple(m1_faces) + tuple(
            arranged_by_index[index] for index in sorted(arranged_by_index)
        )

    transition_edge_owners = {}
    transition_edge_records = {}
    station_by_point = {}
    for sides in transition_contract.values():
        for side in sides:
            transition_id = _hashable_provenance(side.transition_key)
            source_edge = tuple(side.transition_key[2])
            station_by_point[
                (repr(transition_id), ("m1-source-vertex", source_edge[0]))
            ] = 0
            station_by_point[
                (repr(transition_id), ("m1-source-vertex", source_edge[1]))
            ] = side.station_extent
            for station in side.stations:
                station_by_point[
                    (
                        repr(transition_id),
                        ("m1-transition", transition_id, int(station)),
                    )
                ] = int(station)
    for face in m1_faces:
        if len(face.point_keys) != len(face.points):
            continue
        for index, first in enumerate(face.point_keys):
            second = face.point_keys[(index + 1) % len(face.point_keys)]
            shared_transition_ids = (
                point_transition_ids(first)
                & point_transition_ids(second)
            )
            if not shared_transition_ids:
                continue
            for transition_id in shared_transition_ids:
                edge = (
                    repr(transition_id),
                    *sorted((repr(first), repr(second))),
                )
                transition_edge_owners.setdefault(edge, []).append(
                    (
                        face.surface.domain.chart_id,
                        face.crop.kind,
                        face.site.edge_index,
                    )
                )
                transition_edge_records.setdefault(edge, []).append(
                    (
                        face,
                        first,
                        second,
                        face.points[index],
                        face.points[(index + 1) % len(face.points)],
                        transition_id,
                    )
                )
    overfull = {
        edge: owners
        for edge, owners in transition_edge_owners.items()
        if len(owners) > 2
    }
    if overfull:
        first_edge, owners = min(overfull.items(), key=lambda item: item[0])
        raise ValueError(
            "ATLAS_TRANSITION_DESYNC: overfull transition subedge "
            f"edge={first_edge!r} owners={tuple(owners)!r}"
        )
    _m1_validate_materialized_transition_coverage(
        transition_edge_records,
        station_by_point,
        transition_contract,
        diagnostics,
    )
    if alpha is not None:
        # T7/R2: владелец transition публикует UV-станцию один раз;
        # сосед потребляет её вместе с каноническим ключом точки.
        overrides_by_face = {}
        candidates_by_point = {}
        for records in transition_edge_records.values():
            if len(records) != 2:
                continue
            transition_id = records[0][5]
            owner_chart_id = (
                int(transition_id[3])
                if isinstance(transition_id, tuple)
                and transition_id[:1] == ("atlas-transition",)
                else min(
                    record[0].surface.domain.chart_id
                    for record in records
                )
            )
            for record in records:
                face = record[0]
                owner_id = face.crop.semantic_owner_id
                is_corner = (
                    isinstance(owner_id, tuple)
                    and owner_id[:1] == ("corner",)
                )
                if is_corner:
                    semantic_owner_chart_id = int(
                        dict(
                            face.surface.semantic_owner_chart_by_vertex
                        ).get(
                            int(owner_id[1]), face.surface.domain.chart_id
                        )
                    )
                    if (
                        face.uv_frame_owner_chart_id
                        != semantic_owner_chart_id
                    ):
                        raise ValueError(
                            "ATLAS_TRANSITION_DESYNC: UV frame provenance "
                            f"owner={semantic_owner_chart_id} "
                            f"actual={face.uv_frame_owner_chart_id}"
                        )
                    if (
                        face.surface.domain.chart_id
                        == semantic_owner_chart_id
                        and not face.uv_frame_is_imported
                    ):
                        preference = 0
                    elif face.uv_frame_is_imported:
                        preference = 1
                    else:
                        raise ValueError(
                            "ATLAS_TRANSITION_DESYNC: non-owner corner UV "
                            "frame was not transported"
                        )
                else:
                    # R5: transition owner публикует station key, но не UV.
                    # Для segment/site frame локальный materialization owner
                    # так же строго выше его transported-копии.
                    if (
                        face.uv_frame_is_imported
                        and face.uv_frame_owner_chart_id
                        == face.surface.domain.chart_id
                    ):
                        raise ValueError(
                            "ATLAS_TRANSITION_DESYNC: imported UV frame "
                            "points back to consumer chart"
                        )
                    preference = 1 if face.uv_frame_is_imported else 0
                for point_key, point in (
                    (record[1], record[3]),
                    (record[2], record[4]),
                ):
                    candidates_by_point.setdefault(point_key, []).append(
                        (
                            preference,
                            face.surface.domain.chart_id,
                            face.site.edge_index,
                            _m1_raw_arrangement_uv(
                                face, point, float(alpha)
                            ),
                        )
                    )
        canonical_uv_by_point = {
            point_key: min(candidates)[-1]
            for point_key, candidates in candidates_by_point.items()
        }
        for records in transition_edge_records.values():
            for record in records:
                for point_key in (record[1], record[2]):
                    canonical_uv = canonical_uv_by_point.get(point_key)
                    if canonical_uv is not None:
                        overrides_by_face.setdefault(id(record[0]), {})[
                            point_key
                        ] = canonical_uv
        face_replacements = {}
        for face in m1_faces:
            overrides = overrides_by_face.get(id(face), {})
            replacement = (
                replace(
                    face,
                    transition_uv=tuple(
                        (key, values[0], values[1])
                        for key, values in sorted(
                            overrides.items(), key=lambda item: repr(item[0])
                        )
                    ),
                )
                if overrides
                else face
            )
            face_replacements[id(face)] = replacement
        m1_faces = [face_replacements[id(face)] for face in m1_faces]
        transition_edge_records = {
            edge: [
                (face_replacements[id(record[0])], *record[1:])
                for record in records
            ]
            for edge, records in transition_edge_records.items()
        }
        _m1_validate_semantic_transitions(
            transition_edge_records,
            station_by_point,
            float(alpha),
            diagnostics,
        )
        faces = tuple(m1_faces) + tuple(
            arranged_by_index[index]
            for index in sorted(arranged_by_index)
        )
    resolved_corner_views = _resolve_arrangement_corner_views(
        faces,
        corner_resolution_sources,
    )
    return DecalArrangement(
        faces=faces,
        inserted_stations=inserted_stations,
        resolved_corner_views=resolved_corner_views,
    )


def _source_station_location(surface, point, source_feature, feature_id):
    """Provenance для selected source spine, известная без chart lookup."""

    location = surface.domain.locate(point)
    if location is None:
        raise ValueError("Source station lies outside decal surface domain")
    return replace(
        location,
        source_feature=source_feature,
        source_feature_id=feature_id,
    )


def _hashable_provenance(value):
    try:
        hash(value)
    except TypeError:
        return repr(value)
    return value


def _ordered_unique_provenance(values):
    by_key = {}
    for value in values:
        by_key.setdefault(repr(value), value)
    return tuple(by_key[key] for key in sorted(by_key))


def _patch_face_provenance(vertices, semantic_owner_id=None):
    vertices = tuple(vertices)
    if not vertices:
        raise ValueError("PATCH_VORONOI_VERTEX_PROVENANCE_MISSING")
    return PatchVoronoiFaceProvenance(
        source_face_ids=_ordered_unique_provenance(
            vertex.source_face_id for vertex in vertices
        ),
        source_edge_ids=tuple(
            sorted({int(vertex.source_edge_id) for vertex in vertices})
        ),
        route_ids=_ordered_unique_provenance(
            vertex.route_id for vertex in vertices
        ),
        station_keys=tuple(vertex.station_key for vertex in vertices),
        vertices=vertices,
        semantic_owner_id=semantic_owner_id,
    )


def _domain_location_key(surface, location):
    """Shared identity source feature, включая chart-cut copies."""

    quantum = max(DECAL_WELD_DISTANCE * 0.25, 1e-7)
    if surface.domain.kind == "PLANAR":
        if location.source_feature == "VERTEX":
            return ("pv-sv", int(location.source_feature_id))
        if location.source_feature == "EDGE":
            parameter = surface.domain.planar_edge_parameter(
                location.source_feature_id, location.uv
            )
            if parameter is not None:
                return (
                    "pv-se",
                    int(location.source_feature_id),
                    round(parameter, 7),
                )
        # Interior PLANAR identity остаётся patch-local.
        return (
            "pv",
            surface.patch_id,
            round(location.uv[0] / quantum),
            round(location.uv[1] / quantum),
        )
    if (
        surface.domain.admission_tier == "APPROXIMATE"
        and location.transition_key is None
    ):
        quantum = max(float(surface.diagram_transform.quantum), 1e-10)
        return (
            "m1",
            surface.patch_id,
            location.chart_id,
            round(location.uv[0] / quantum),
            round(location.uv[1] / quantum),
        )
    if location.transition_key is not None:
        identity = (
            "TRANSITION",
            _hashable_provenance(location.transition_key),
        )
    else:
        identity = (
            location.source_feature,
            _hashable_provenance(location.source_feature_id),
        )
    if location.source_feature == "VERTEX":
        return ("pv-feature",) + identity
    if location.source_feature == "EDGE" or location.transition_key is not None:
        source_position = surface.domain.source_position(location)
        return (
            "pv-feature",
            *identity,
            *(round(float(value) / quantum) for value in source_position),
        )
    return (
        "pv",
        surface.patch_id,
        location.chart_id,
        round(location.uv[0] / quantum),
        round(location.uv[1] / quantum),
    )


def _position_and_key(
    plan,
    surface,
    site,
    point,
    lift_scale,
    effective_offset,
    projection=None,
):
    if projection is None:
        distance, t = _segment_point_distance2(
            site.point_a, site.point_b, point
        )
    else:
        distance, t = projection
    spine_eps = max(DECAL_WELD_DISTANCE * 0.25, site.segment_length * 1e-7)
    if (
        distance <= spine_eps
        and surface.domain.admission_tier != "APPROXIMATE"
    ):
        endpoint_eps = max(
            site.segment_length * 1e-6,
            min(DECAL_WELD_DISTANCE, site.segment_length * 0.01),
        )
        if _dist2(point, site.point_a) <= endpoint_eps:
            position = site.source_a.lerp(
                plan.lifted_vertices[site.vert_a], lift_scale
            )
            location = _source_station_location(
                surface, point, "VERTEX", site.vert_a
            )
            return position, ("pv-sv", site.vert_a), location
        if _dist2(point, site.point_b) <= endpoint_eps:
            position = site.source_b.lerp(
                plan.lifted_vertices[site.vert_b], lift_scale
            )
            location = _source_station_location(
                surface, point, "VERTEX", site.vert_b
            )
            return position, ("pv-sv", site.vert_b), location
        start = site.source_a.lerp(
            plan.lifted_vertices[site.vert_a], lift_scale
        )
        end = site.source_b.lerp(
            plan.lifted_vertices[site.vert_b], lift_scale
        )
        location = _source_station_location(
            surface, point, "EDGE", site.edge_index
        )
        # Один physical source edge может быть скомпилирован с разных сторон
        # в противоположных направлениях. Identity станции всегда ориентирован
        # по vertex-id, как planar domain provenance, а не по направлению site.
        canonical_t = t if site.vert_a < site.vert_b else 1.0 - t
        return (
            start.lerp(end, t),
            ("pv-se", site.edge_index, round(canonical_t, 7)),
            location,
        )
    location = surface.domain.locate(point)
    if location is None:
        raise ValueError("Point lies outside decal surface domain")
    position = surface.domain.lift(
        point, effective_offset, location=location
    )
    return position, _domain_location_key(surface, location), location


def _resolve_arrangement_point(
    plan,
    surface,
    owning_site,
    point,
    desired_scale,
    cache,
):
    """Кэширует scale-independent owner и affine endpoints station."""

    cache_key = (id(surface), owning_site.edge_index, point)
    cached = cache.get(cache_key)
    if cached is not None:
        return cached
    grid_size = surface.site_grid_size
    grid_x = int(point[0] // grid_size)
    grid_y = int(point[1] // grid_size)
    candidate_indices = set()
    for offset_x in (-1, 0, 1):
        for offset_y in (-1, 0, 1):
            candidate_indices.update(
                surface.site_grid.get(
                    (grid_x + offset_x, grid_y + offset_y), ()
                )
            )
    best_site = owning_site
    best_distance, best_factor = _segment_point_distance2(
        owning_site.point_a, owning_site.point_b, point
    )
    for candidate_index in candidate_indices:
        candidate = surface.sites[candidate_index]
        distance, _factor = _segment_point_distance2(
            candidate.point_a, candidate.point_b, point
        )
        if (
            distance < best_distance - 1e-12
            or (
                abs(distance - best_distance) <= 1e-12
                and candidate.edge_index < best_site.edge_index
            )
        ):
            best_distance = distance
            best_factor = _factor
            best_site = candidate
    projection = (best_distance, best_factor)
    position_zero, key_zero, location_zero = _position_and_key(
        plan,
        surface,
        best_site,
        point,
        0.0,
        0.0,
        projection=projection,
    )
    position_full, key_full, location_full = _position_and_key(
        plan,
        surface,
        best_site,
        point,
        desired_scale,
        plan.offset * desired_scale,
        projection=projection,
    )
    if key_zero != key_full:
        raise RuntimeError("Arrangement identity depends on lift scale")
    if location_zero != location_full:
        raise RuntimeError("Arrangement provenance depends on lift scale")
    resolved = _ResolvedArrangementPoint(
        position_zero=position_zero,
        position_full=position_full,
        vert_key=key_full,
        location=location_full,
    )
    cache[cache_key] = resolved
    return resolved


def _count_periodic_transition_welds(plan, resolved_points):
    """Считает реально сведённые seam stations по DP3 keys, не по modulo."""

    surfaces_by_identity = {id(surface): surface for surface in plan.surfaces}
    stations = {}
    for cache_key, resolved in resolved_points.items():
        surface = surfaces_by_identity.get(cache_key[0])
        if surface is None or not surface.domain.periodic_axis:
            continue
        transition_key = resolved.location.transition_key
        if transition_key is None:
            continue
        canonical_key = surface.domain.canonical_transition_key(
            transition_key
        )
        stations.setdefault(
            (
                id(surface),
                _hashable_provenance(canonical_key),
                resolved.vert_key,
            ),
            set(),
        ).add(float(resolved.location.uv[0]))
    count = 0
    for (surface_identity, _transition_key, _vert_key), values in stations.items():
        surface = surfaces_by_identity[surface_identity]
        if len(values) < 2:
            continue
        if max(values) - min(values) >= surface.domain.period * (1.0 - 1e-7):
            count += 1
    return count


def _count_atlas_transition_welds(plan, resolved_points):
    """Считает equivalence stations, реально разделённые atlas charts."""

    surfaces = {id(surface): surface for surface in plan.surfaces}
    stations = {}
    for cache_key, resolved in resolved_points.items():
        transition_key = resolved.location.transition_key
        if (
            not isinstance(transition_key, tuple)
            or transition_key[:1] != ("atlas-transition",)
        ):
            continue
        stations.setdefault(
            (_hashable_provenance(transition_key), resolved.vert_key), set()
        ).add(cache_key[0])
    return sum(
        1
        for surface_ids in stations.values()
        if len({surfaces[value].domain.chart_id for value in surface_ids}) >= 2
    )


def _component_area_coefficients(
    plan,
    surface,
    site,
    component,
    desired_scale,
    resolved_points,
):
    """Коэффициенты signed area для affine lift: A(t)=qa*t2+qb*t+qc."""

    endpoints = []
    used_keys = set()
    for point in component:
        resolved = _resolve_arrangement_point(
            plan,
            surface,
            site,
            point,
            desired_scale,
            resolved_points,
        )
        if resolved.vert_key in used_keys:
            continue
        used_keys.add(resolved.vert_key)
        endpoints.append(
            (
                resolved.position_zero,
                resolved.position_full - resolved.position_zero,
            )
        )
    if len(endpoints) < 3:
        return None
    vector_qc = Vector((0.0, 0.0, 0.0))
    vector_qb = Vector((0.0, 0.0, 0.0))
    vector_qa = Vector((0.0, 0.0, 0.0))
    origin_zero, origin_delta = endpoints[0]
    for index in range(1, len(endpoints) - 1):
        point_zero, point_delta = endpoints[index]
        next_zero, next_delta = endpoints[index + 1]
        edge_zero = point_zero - origin_zero
        edge_delta = point_delta - origin_delta
        next_edge_zero = next_zero - origin_zero
        next_edge_delta = next_delta - origin_delta
        vector_qc += edge_zero.cross(next_edge_zero)
        vector_qb += edge_zero.cross(next_edge_delta)
        vector_qb += edge_delta.cross(next_edge_zero)
        vector_qa += edge_delta.cross(next_edge_delta)
    centroid = (
        sum(point[0] for point in component) / len(component),
        sum(point[1] for point in component) / len(component),
    )
    normal = _polygon_domain_normal(surface.domain, component, centroid)
    return (
        vector_qa.dot(normal) * 0.5,
        vector_qb.dot(normal) * 0.5,
        vector_qc.dot(normal) * 0.5,
    )


def _orientation_safe_lift_scale(plan, pending, desired_scale, resolved_points):
    """Аналитический общий offset-scale без перевёрнутых wing faces.

    Каждая 3D-позиция affine по scale, следовательно signed area face —
    квадратный полином. Три измерения дают его точно и заменяют дорогой
    бинарный rebuild всей сети на один линейный проход.
    """

    safe_fraction = 1.0
    for pending_face in pending:
        surface = pending_face.surface
        site = pending_face.site
        component = pending_face.points
        coefficients = _component_area_coefficients(
            plan,
            surface,
            site,
            component,
            desired_scale,
            resolved_points,
        )
        if coefficients is None:
            continue
        qa, qb, qc = coefficients
        roots = []
        if abs(qa) <= 1e-18:
            if abs(qb) > 1e-18:
                roots.append(-qc / qb)
        else:
            discriminant = qb * qb - 4.0 * qa * qc
            if discriminant >= 0.0:
                root_delta = sqrt(discriminant)
                roots.extend(
                    (
                        (-qb - root_delta) / (2.0 * qa),
                        (-qb + root_delta) / (2.0 * qa),
                    )
                )
        for root in sorted(value for value in roots if 0.0 < value <= 1.0):
            probe = min(1.0, root + max(1e-5, (1.0 - root) * 1e-3))
            after_root = qa * probe * probe + qb * probe + qc
            if after_root < 0.0:
                safe_fraction = min(safe_fraction, root)
                break
    if safe_fraction >= 1.0:
        return desired_scale
    # Запас не даёт float32/weld вернуть face точно на нулевой Jacobian.
    return desired_scale * safe_fraction * 0.99


def _synchronize_cross_surface_spine_stations(plan, faces):
    """Объединяет pv-se stations общего source edge соседних surfaces.

    Arrangement conformal только внутри одной owner surface. Corner crop
    может разбить source edge каждой поверхности в разных станциях. Без этой
    синхронизации materialization получает геометрический T-контакт и щель.
    Уже разбитые runs сливаются теми же canonical keys; новые faces не
    создаются.
    """

    endpoints_by_edge = {}
    edge_by_vertices = {}
    for surface in plan.surfaces:
        for site in surface.sites:
            pair = frozenset((site.vert_a, site.vert_b))
            endpoints_by_edge.setdefault(
                site.edge_index, (site.vert_a, site.vert_b)
            )
            edge_by_vertices.setdefault(pair, site.edge_index)

    provenance_by_key = {}
    for face in faces:
        provenance = getattr(face, "provenance", None)
        if not isinstance(provenance, PatchVoronoiFaceProvenance):
            raise RuntimeError("PATCH_VORONOI_FACE_PROVENANCE_MISSING")
        if len(provenance.vertices) != len(face.vert_keys):
            raise RuntimeError("PATCH_VORONOI_VERTEX_PROVENANCE_DESYNC")
        for key, vertex_provenance in zip(
            face.vert_keys, provenance.vertices
        ):
            provenance_by_key.setdefault(key, []).append(vertex_provenance)

    stations_by_edge = {}
    for face in faces:
        for key, position in zip(face.vert_keys, face.positions):
            if not isinstance(key, tuple) or key[:1] != ("pv-se",):
                continue
            stations_by_edge.setdefault(key[1], {}).setdefault(
                key, position.copy()
            )
    if not stations_by_edge:
        return

    for face in faces:
        count = len(face.vert_keys)
        if count < 3:
            continue
        new_keys = []
        new_positions = []
        new_u_fracs = []
        new_v_lengths = []
        new_vertex_provenance = []
        face_provenance = face.provenance
        original_provenance = dict(
            zip(face.vert_keys, face_provenance.vertices)
        )
        for index in range(count):
            key_a = face.vert_keys[index]
            key_b = face.vert_keys[(index + 1) % count]
            position_a = face.positions[index]
            position_b = face.positions[(index + 1) % count]
            u_a = face.u_fracs[index]
            u_b = face.u_fracs[(index + 1) % count]
            v_a = face.v_lengths[index]
            v_b = face.v_lengths[(index + 1) % count]

            new_keys.append(key_a)
            new_positions.append(position_a)
            new_u_fracs.append(u_a)
            new_v_lengths.append(v_a)
            new_vertex_provenance.append(original_provenance[key_a])

            if not isinstance(key_a, tuple) or not isinstance(key_b, tuple):
                continue
            edge_ids = {
                int(key[1])
                for key in (key_a, key_b)
                if key[:1] == ("pv-se",)
            }
            if len(edge_ids) > 1:
                continue
            if edge_ids:
                edge_index = next(iter(edge_ids))
                # Станция source edge может делить только ребро face-cycle,
                # оба конца которого имеют provenance того же source edge.
                # Сегмент INTERIOR -> pv-se является crop/frontier, а не
                # продолжением source edge; spatial proximity здесь рождает
                # pinched cycle с несмежным повтором ключа (C8.6).
                source_key_kinds = {("pv-se",), ("pv-sv",)}
                if any(
                    key[:1] not in source_key_kinds
                    for key in (key_a, key_b)
                ):
                    continue
                endpoints = endpoints_by_edge.get(edge_index, ())
                if any(
                    key[:1] == ("pv-sv",) and key[1] not in endpoints
                    for key in (key_a, key_b)
                ):
                    continue
            elif all(key[:1] == ("pv-sv",) for key in (key_a, key_b)):
                edge_index = edge_by_vertices.get(
                    frozenset((key_a[1], key_b[1]))
                )
            else:
                edge_index = None
            station_records = stations_by_edge.get(edge_index, ())
            if not station_records:
                continue
            edge_vector = position_b - position_a
            edge_length2 = edge_vector.length_squared
            if edge_length2 <= _GEOMETRY_EPS:
                continue
            edge_length = sqrt(edge_length2)
            tolerance = max(DECAL_WELD_DISTANCE, edge_length * 1e-6)
            insertions = []
            for station_key, station_position in station_records.items():
                factor = (
                    (station_position - position_a).dot(edge_vector)
                    / edge_length2
                )
                if factor <= 1e-7 or factor >= 1.0 - 1e-7:
                    continue
                closest = position_a.lerp(position_b, factor)
                if (closest - station_position).length > tolerance:
                    continue
                insertions.append((factor, repr(station_key), station_key, station_position))
            for factor, _key_order, station_key, station_position in sorted(
                insertions
            ):
                new_keys.append(station_key)
                new_positions.append(station_position.copy())
                new_u_fracs.append(u_a + (u_b - u_a) * factor)
                new_v_lengths.append(v_a + (v_b - v_a) * factor)
                candidates = provenance_by_key.get(station_key, ())
                if not candidates:
                    raise RuntimeError(
                        "PATCH_VORONOI_INSERTED_STATION_PROVENANCE_MISSING: "
                        f"key={station_key!r}"
                    )
                new_vertex_provenance.append(
                    min(candidates, key=repr)
                )

        face.vert_keys = new_keys
        face.positions = new_positions
        face.u_fracs = new_u_fracs
        face.v_lengths = new_v_lengths
        face.provenance = _patch_face_provenance(
            new_vertex_provenance,
            semantic_owner_id=face_provenance.semantic_owner_id,
        )


def _junction_connector_faces(
    plan,
    faces,
    alpha,
    corner_settings=None,
    diagnostics=None,
):
    """Закрывает парные cross-patch sectors до BMesh materialization.

    Внутри planar surface endpoint Voronoi-cell уже соединяет соседние
    strips. После fold остаются парные открытые core-to-rail rays: это
    математическая граница отсутствующего junction sector, а не mesh-hole
    для последующего fill. Соединяем только такие rays и только локально у
    одной source-вершины.
    """

    surfaces_by_id = {
        surface.patch_id: surface
        for surface in plan.surfaces
        if getattr(
            getattr(surface, "domain", None),
            "admission_tier",
            "EXACT",
        )
        != "APPROXIMATE"
    }
    incident_edges_by_vertex = {}
    for surface in plan.surfaces:
        if getattr(
            getattr(surface, "domain", None),
            "admission_tier",
            "EXACT",
        ) == "APPROXIMATE":
            continue
        if (
            diagnostics is not None
            and diagnostics.reference_full_scan
        ) or not hasattr(surface, "ports_by_vertex"):
            for site in surface.sites:
                incident_edges_by_vertex.setdefault(site.vert_a, set()).add(
                    site.edge_index
                )
                incident_edges_by_vertex.setdefault(site.vert_b, set()).add(
                    site.edge_index
                )
        else:
            for vert_index, ports in surface.ports_by_vertex.items():
                incident_edges_by_vertex.setdefault(vert_index, set()).update(
                    port.edge_index for port in ports
                )

    edge_uses = {}
    for face_index, face in enumerate(faces):
        for index, key_a in enumerate(face.vert_keys):
            key_b = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge_key = tuple(sorted((key_a, key_b), key=repr))
            edge_uses.setdefault(edge_key, []).append((face_index, index))

    ports_by_vertex = {}
    for edge_key, uses in edge_uses.items():
        if len(uses) != 1:
            continue
        core_key = next(
            (key for key in edge_key if key[:1] == ("pv-sv",)), None
        )
        if core_key is None:
            continue
        outer_key = edge_key[0] if edge_key[1] == core_key else edge_key[1]
        if outer_key[:1] == ("pv-sv",):
            continue
        face_index, _edge_index = uses[0]
        face = faces[face_index]
        face_provenance = getattr(face, "provenance", None)
        if not isinstance(face_provenance, PatchVoronoiFaceProvenance):
            raise RuntimeError("PATCH_VORONOI_JUNCTION_PROVENANCE_MISSING")
        provenance_by_key = dict(
            zip(face.vert_keys, face_provenance.vertices)
        )
        surface = surfaces_by_id.get(face.surface_id)
        if surface is None:
            continue
        core_index = face.vert_keys.index(core_key)
        outer_index = face.vert_keys.index(outer_key)
        outer_position = face.positions[outer_index]
        core_position = face.positions[core_index]
        domain = getattr(surface, "domain", None)
        domain_kind = getattr(domain, "kind", "PLANAR")
        outer_point = None
        if domain_kind == "PLANAR":
            outer_point = (
                (outer_position - surface.origin).dot(surface.basis_u),
                (outer_position - surface.origin).dot(surface.basis_v),
            )
        matching_sites = []
        if (
            diagnostics is not None
            and diagnostics.reference_full_scan
        ) or not hasattr(surface, "ports_by_vertex"):
            compiled_ports = tuple(
                _CompiledSitePort(
                    site_index=site_index,
                    edge_index=site.edge_index,
                    vert_index=core_key[1],
                    point=(
                        site.point_a
                        if site.vert_a == core_key[1]
                        else site.point_b
                    ),
                    source_category=(1 if site.vert_a == core_key[1] else 2),
                )
                for site_index, site in enumerate(surface.sites)
                if core_key[1] in (site.vert_a, site.vert_b)
            )
        else:
            compiled_ports = surface.ports_by_vertex.get(core_key[1], ())
        for compiled_port in compiled_ports:
            site = surface.sites[compiled_port.site_index]
            station = compiled_port.point
            if domain_kind == "PLANAR":
                expected_outer = (
                    station[0] + site.inward_normal[0] * alpha,
                    station[1] + site.inward_normal[1] * alpha,
                )
                matching_sites.append(
                    (_dist2(outer_point, expected_outer), site.edge_index)
                )
                continue

            # Intrinsic rail может быть обрезан первым source-transition
            # раньше alpha-frontier. Поэтому сравнение с полной alpha-точкой
            # неверно: распознаём тот же compile-static inward луч через
            # локальную изометрию chart -> source surface.
            station_location = domain.locate(station)
            if station_location is None:
                continue
            station_position = domain.source_position(station_location)
            probe = max(
                float(surface.diagram_transform.quantum),
                float(domain.location_tolerance) * 4.0,
            )
            expected_direction = None
            for probe_scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
                probe_point = (
                    station[0]
                    + site.inward_normal[0] * probe * probe_scale,
                    station[1]
                    + site.inward_normal[1] * probe * probe_scale,
                )
                probe_location = domain.locate(probe_point)
                if probe_location is None:
                    continue
                direction = (
                    domain.source_position(probe_location)
                    - station_position
                )
                if direction.length_squared > 1e-18:
                    expected_direction = direction.normalized()
                    break
            if expected_direction is None:
                continue
            actual_direction = outer_position - core_position
            actual_length = actual_direction.length
            if actual_length <= _GEOMETRY_EPS:
                continue
            alignment = actual_direction.dot(expected_direction) / actual_length
            # Source fold и offset-normal могут слегка повернуть lifted chord,
            # но longitudinal spine edge остаётся почти ортогонален inward.
            if alignment < cos(pi / 6.0):
                continue
            if actual_length > max(
                DECAL_WELD_DISTANCE * 2.0,
                alpha * 1.05 + abs(float(plan.offset)) * 2.0,
            ):
                continue
            matching_sites.append(
                ((1.0 - alignment) * max(alpha, 1e-12), site.edge_index)
            )
        if not matching_sites:
            continue
        match_distance, matched_edge_index = min(matching_sites)
        # Boundary edge большой Voronoi-cell тоже может начинаться в core,
        # но junction port обязан лежать у локального alpha-offset cap.
        if domain_kind == "PLANAR" and match_distance > max(
            DECAL_WELD_DISTANCE, alpha * 0.05
        ):
            continue
        ports_by_vertex.setdefault(core_key[1], []).append(
            _JunctionPort(
                surface_id=face.surface_id,
                edge_index=matched_edge_index,
                surface_normal=face.surface_normal.copy(),
                core_position=face.positions[core_index].copy(),
                outer_key=outer_key,
                outer_position=face.positions[outer_index].copy(),
                outer_u=face.u_fracs[outer_index],
                outer_v=face.v_lengths[outer_index],
                core_v=face.v_lengths[core_index],
                core_provenance=provenance_by_key[core_key],
                outer_provenance=provenance_by_key[outer_key],
            )
        )

    connectors = []
    max_span = max(alpha * 2.25, DECAL_WELD_DISTANCE * 4.0)
    for vert_index, ports in sorted(ports_by_vertex.items()):
        # Cross-patch connector описывает поворот одной ветви. T/X junction
        # уже принадлежит surface Voronoi arrangement и не должен получать
        # произвольный greedy chord между несколькими ветвями.
        incident_count = len(incident_edges_by_vertex.get(vert_index, ()))
        if incident_count < 2:
            continue
        if incident_count > 2:
            core_position = sum(
                (port.core_position for port in ports),
                Vector((0.0, 0.0, 0.0)),
            ) / len(ports)
            blended = sum(
                (port.surface_normal for port in ports),
                Vector((0.0, 0.0, 0.0)),
            )
            if blended.length_squared <= 1e-12:
                continue
            normal = blended.normalized()
            projected = []
            for port in ports:
                direction = port.outer_position - core_position
                direction -= normal * direction.dot(normal)
                if direction.length_squared <= 1e-12:
                    continue
                projected.append((port, direction.normalized()))
            if len(projected) < 3:
                continue
            basis_u = projected[0][1]
            basis_v = normal.cross(basis_u).normalized()
            ordered = sorted(
                projected,
                key=lambda item: (
                    atan2(item[1].dot(basis_v), item[1].dot(basis_u)),
                    item[0].edge_index,
                    repr(item[0].outer_key),
                ),
            )
            for sector_index, (first, first_direction) in enumerate(ordered):
                second, second_direction = ordered[
                    (sector_index + 1) % len(ordered)
                ]
                if (
                    first.edge_index == second.edge_index
                    or first.outer_key == second.outer_key
                ):
                    continue
                signed_angle = atan2(
                    normal.dot(first_direction.cross(second_direction)),
                    max(-1.0, min(1.0, first_direction.dot(second_direction))),
                )
                sector_angle = signed_angle % tau
                if sector_angle >= pi - 1e-7 or sector_angle <= 1e-7:
                    continue
                policy = _classify_extrusion_angle(
                    sector_angle,
                    _normalized_corner_runtime_settings(corner_settings),
                )
                entries = [first, second]
                winding = (first.outer_position - core_position).cross(
                    second.outer_position - core_position
                )
                if winding.dot(normal) < 0.0:
                    entries.reverse()
                connectors.append(
                    _NetworkFace(
                        surface_id=-1,
                        surface_normal=normal,
                        vert_keys=[("pv-sv", vert_index)]
                        + [entry.outer_key for entry in entries],
                        positions=[core_position]
                        + [entry.outer_position for entry in entries],
                        u_fracs=[0.0]
                        + [entry.outer_u for entry in entries],
                        v_lengths=[
                            sum(entry.core_v for entry in entries) / 2.0
                        ]
                        + [entry.outer_v for entry in entries],
                        component_kind=policy.value,
                        component_side=f"JUNCTION_SECTOR_{sector_index}",
                        provenance=_patch_face_provenance(
                            (entries[0].core_provenance,)
                            + tuple(
                                entry.outer_provenance for entry in entries
                            ),
                            semantic_owner_id=(
                                "junction-sector",
                                int(vert_index),
                                int(sector_index),
                            ),
                        ),
                    )
                )
                if diagnostics is not None:
                    diagnostics.record_runtime_policy(policy)
            continue
        # Одна ray — штатный открытый cap. Несколько rays образуют
        # независимые парные sectors; greedy matching детерминирован и не
        # делает fan, где один boundary edge получил бы три owner faces.
        candidates = []
        for index, first in enumerate(ports):
            for other_index in range(index + 1, len(ports)):
                second = ports[other_index]
                if first.outer_key == second.outer_key:
                    continue
                if first.surface_id == second.surface_id:
                    continue
                if first.edge_index == second.edge_index:
                    continue
                distance = (first.outer_position - second.outer_position).length
                candidates.append(
                    (
                        distance,
                        first.surface_id,
                        repr(first.outer_key),
                        second.surface_id,
                        repr(second.outer_key),
                        index,
                        other_index,
                    )
                )
        used_ports = set()
        for distance, _sa, _ka, _sb, _kb, index, other_index in sorted(
            candidates
        ):
            if distance > max_span:
                break
            if index in used_ports or other_index in used_ports:
                continue
            first = ports[index]
            second = ports[other_index]
            core_position = first.core_position.lerp(
                second.core_position, 0.5
            )
            blended = first.surface_normal + second.surface_normal
            if blended.length_squared <= 1e-12:
                continue
            normal = blended.normalized()
            entries = [first, second]
            winding = (first.outer_position - core_position).cross(
                second.outer_position - core_position
            )
            if winding.dot(normal) < 0.0:
                entries.reverse()
            connectors.append(
                _NetworkFace(
                    surface_id=-1,
                    surface_normal=normal,
                    vert_keys=[("pv-sv", vert_index)]
                    + [entry.outer_key for entry in entries],
                    positions=[core_position]
                    + [entry.outer_position for entry in entries],
                    u_fracs=[0.0] + [entry.outer_u for entry in entries],
                    v_lengths=[
                        (first.core_v + second.core_v) * 0.5
                    ]
                    + [entry.outer_v for entry in entries],
                    provenance=_patch_face_provenance(
                        (entries[0].core_provenance,)
                        + tuple(
                            entry.outer_provenance for entry in entries
                        ),
                        semantic_owner_id=(
                            "junction-pair",
                            int(vert_index),
                        ),
                    ),
                )
            )
            used_ports.update((index, other_index))
    return connectors


def _surface_is_approximate(surface):
    """Совместимый с unit fixtures admission probe без domain-заглушки."""

    return (
        getattr(getattr(surface, "domain", None), "admission_tier", None)
        == "APPROXIMATE"
    )


def _atom_fragment_records(surface, atom):
    """Возвращает fragment вместе с G1 merge-group source triangle."""

    triangle_ids = atom.fragment_triangle_ids
    domain = getattr(surface, "domain", None)
    merge_groups = getattr(domain, "triangle_merge_groups", ())
    for fragment_index, fragment in enumerate(atom.fragments):
        triangle_id = (
            triangle_ids[fragment_index]
            if fragment_index < len(triangle_ids)
            else -1
        )
        merge_group = (
            merge_groups[triangle_id]
            if 0 <= triangle_id < len(merge_groups)
            else 0
        )
        yield fragment, merge_group


def _append_pending_fragments(
    pending,
    surface,
    site,
    crop,
    fragments,
    diagnostics=None,
    absorbed_corner_vertices=(),
    terminal_cut_vertices=(),
    fragment_merge_groups=(),
):
    """Сваривает fragments одного semantic owner до materialization."""

    if _surface_is_approximate(surface):
        # M1 сам строит exact arrangement из compile atoms, domain edges и
        # прежние clip/subtract fragments он не читает, поэтому
        # их вычисление и merge здесь только дублировали дорогую работу.
        components = [list(crop.points)]
    else:
        # Final BMesh weld намеренно крупнее, но применять его здесь нельзя:
        # близкие curve/domain stations несут разную topology. Их преждевременное
        # схлопывание создаёт ложный T-junction, после которого area-safe fallback
        # сохраняет дырки закрытыми, но отпечатывает source triangulation.
        components = _merge_polygon_fragments(
            fragments,
            tolerance=_FRAGMENT_TOPOLOGY_TOLERANCE,
            diagnostics=diagnostics,
            normalize_t_junctions=bool(
                getattr(getattr(surface, "domain", None), "periodic_axis", "")
            ),
            merge_groups=fragment_merge_groups,
        )
    for component in components:
        # _merge_polygon_fragments уже возвращает deduped валидные contours;
        # повторный distance/area pass на каждом runtime crop был лишним.
        if len(component) < 3:
            continue
        area = _polygon_area2(component)
        if abs(area) <= 1e-10:
            continue
        if area < 0.0:
            component.reverse()
        pending.append(
            _PendingArrangementFace(
                surface=surface,
                site=site,
                points=tuple(component),
                crop=crop,
                absorbed_corner_vertices=tuple(
                    sorted({int(vertex) for vertex in absorbed_corner_vertices})
                ),
                terminal_cut_vertices=tuple(
                    sorted({int(vertex) for vertex in terminal_cut_vertices})
                ),
            )
        )


def _periodic_site_image(surface, site, shift):
    """Возвращает runtime-view site image без размножения compile IR."""

    shift = int(shift)
    if not shift:
        return site
    return _translated_site_u(site, shift * float(surface.domain.period))


def _translated_site_u(site, offset):
    if abs(offset) <= _GEOMETRY_EPS:
        return site
    return replace(
        site,
        point_a=(site.point_a[0] + offset, site.point_a[1]),
        point_b=(site.point_b[0] + offset, site.point_b[1]),
    )


def _periodic_crop_image(surface, crop, shift):
    """Переносит geometry crop; UV anchors остаются в transport frame."""

    shift = int(shift)
    if not shift:
        return crop
    return _translated_crop_u(crop, shift * float(surface.domain.period))


def _translated_crop_u(crop, offset):
    if abs(offset) <= _GEOMETRY_EPS:
        return crop
    return replace(
        crop,
        points=tuple((point[0] + offset, point[1]) for point in crop.points),
    )


def _corner_atom_image_offset(surface, corner, atom):
    """Единый transport corner crop для emission и subtraction."""

    period = float(
        getattr(getattr(surface, "domain", None), "period", 0.0)
    )
    corner_offsets = dict(getattr(corner, "site_u_offsets", ()))
    return (
        atom.periodic_shift * period
        - corner_offsets.get(atom.site_index, 0.0)
    )


def _polygon_domain_normal(domain, component, centroid=None):
    """Берёт normal из точки внутри domain даже для concave atlas contour."""

    if centroid is None:
        centroid = (
            sum(point[0] for point in component) / len(component),
            sum(point[1] for point in component) / len(component),
        )
    candidates = [centroid]
    candidates.extend(component)
    candidates.extend(
        (
            (first[0] + second[0]) * 0.5,
            (first[1] + second[1]) * 0.5,
        )
        for first, second in zip(component, component[1:] + component[:1])
    )
    for point in candidates:
        location = domain.locate(point)
        if location is not None:
            return domain.normal_at(point)
    return domain.reference_normal.copy()


def _terminal_source_vertex_points(domain, vertex_id):
    """Chart-images source vertex для read-only RM9 bridge."""

    if domain.kind == "PLANAR":
        points = tuple(
            point
            for source_vertex_id, point in domain.planar_source_vertices
            if source_vertex_id == vertex_id
        )
    else:
        points = tuple(
            triangle.chart_points[local_index]
            for triangle in domain.intrinsic_triangles
            for local_index, source_vertex_id in enumerate(
                triangle.source_vertex_ids
            )
            if source_vertex_id == vertex_id
        )
    return tuple(
        dict.fromkeys((float(point[0]), float(point[1])) for point in points)
    )


def _terminal_source_edge_segments(domain, source_edge):
    """Source edge в координатах одного chart, ориентированный rail IR."""

    edge_id = source_edge.edge_id
    vertex_ids = tuple(source_edge.vertex_ids)
    segments = []
    if domain.kind == "PLANAR":
        for (
            source_edge_id,
            vert_a,
            vert_b,
            point_a,
            point_b,
        ) in domain.planar_source_edges:
            if source_edge_id != edge_id:
                continue
            point_by_vertex = {vert_a: point_a, vert_b: point_b}
            if set(point_by_vertex) != set(vertex_ids):
                continue
            segments.append(
                tuple(point_by_vertex[vertex_id] for vertex_id in vertex_ids)
            )
    else:
        for triangle in domain.intrinsic_triangles:
            for local_edge, source_edge_id in enumerate(
                triangle.source_edge_ids
            ):
                if source_edge_id != edge_id:
                    continue
                edge_vertices = tuple(
                    index for index in range(3) if index != local_edge
                )
                point_by_vertex = {
                    triangle.source_vertex_ids[index]: (
                        triangle.chart_points[index]
                    )
                    for index in edge_vertices
                }
                if set(point_by_vertex) != set(vertex_ids):
                    continue
                segments.append(
                    tuple(
                        point_by_vertex[vertex_id]
                        for vertex_id in vertex_ids
                    )
                )
    return tuple(
        dict.fromkeys(
            tuple((float(point[0]), float(point[1])) for point in segment)
            for segment in segments
        )
    )


def _terminal_station_chart_points(domain, station, edge_by_id):
    if station.kind.value == "VERTEX":
        return _terminal_source_vertex_points(
            domain, station.source_vertex_id
        )
    source_edge = edge_by_id[station.source_edge_id]
    parameter = float(station.edge_parameter)
    return tuple(
        (
            point_a[0] + (point_b[0] - point_a[0]) * parameter,
            point_a[1] + (point_b[1] - point_a[1]) * parameter,
        )
        for point_a, point_b in _terminal_source_edge_segments(
            domain, source_edge
        )
    )


def _terminal_route_chart_guide(
    surface, terminal, rail_plan, alpha, corner_point
):
    """RM10: читает contact и все пройденные route vertices owner-chart."""

    route = next(
        (
            candidate
            for candidate in rail_plan.routes
            if candidate.route_id == terminal.route_id
        ),
        None,
    )
    if route is None or not route.segments or len(route.stations) < 2:
        raise RuntimeError(
            "TERMINAL_BRIDGE_ROUTE_MISSING: "
            f"patch={surface.patch_id} vertex={terminal.spine_vertex_id} "
            f"edge={terminal.spine_edge_id} route={terminal.route_id}"
        )
    station_by_index = {
        station.station_index: station for station in route.stations
    }
    # RR8d: compile уже ставит DAM перед повторным обходом boundary, но
    # materializer защищает тот же инвариант самостоятельно. Второй проход
    # физического edge никогда не становится новой станцией контакта.
    route_segments = []
    route_segment_edge_ids = []
    visited_edge_ids = set()
    revisit_guarded = False
    terminal_edge_ids = tuple(getattr(terminal, "edge_ids", ()) or ())
    for segment_index, segment in enumerate(route.segments):
        segment_edge_id = getattr(segment, "edge_id", None)
        if segment_edge_id is None:
            segment_edge_id = (
                terminal_edge_ids[segment_index]
                if segment_index < len(terminal_edge_ids)
                else -(segment_index + 1)
            )
        if segment_edge_id in visited_edge_ids:
            revisit_guarded = True
            break
        visited_edge_ids.add(segment_edge_id)
        route_segments.append(segment)
        route_segment_edge_ids.append(int(segment_edge_id))
    if not route_segments:
        raise RuntimeError(
            "TERMINAL_BRIDGE_ROUTE_MISSING: "
            f"patch={surface.patch_id} vertex={terminal.spine_vertex_id} "
            f"edge={terminal.spine_edge_id} route={terminal.route_id}"
        )
    route_reach = float(
        station_by_index[route_segments[-1].to_station_index].distance
    )
    requested_extent = float(alpha)
    extent = min(requested_extent, route_reach)
    interval = next(
        (
            (
                station_by_index[segment.from_station_index],
                station_by_index[segment.to_station_index],
            )
            for segment in route_segments
            if (
                station_by_index[segment.from_station_index].distance
                <= extent
                <= station_by_index[segment.to_station_index].distance
            )
        ),
        None,
    )
    if interval is None:
        raise RuntimeError(
            "TERMINAL_BRIDGE_STATION_INTERVAL_MISSING: "
            f"route={terminal.route_id} extent={extent:.9g}"
        )
    station_a, station_b = interval
    edge_by_id = {edge.edge_id: edge for edge in rail_plan.edges}
    points_a = _terminal_station_chart_points(
        surface.domain, station_a, edge_by_id
    )
    points_b = _terminal_station_chart_points(
        surface.domain, station_b, edge_by_id
    )
    if not points_a or not points_b:
        raise RuntimeError(
            "TERMINAL_BRIDGE_ROUTE_OUTSIDE_CAP_CHART: "
            f"patch={surface.patch_id} chart={surface.domain.chart_id} "
            f"vertex={terminal.spine_vertex_id} "
            f"guide_edges={terminal.edge_ids}"
        )
    interval_length = float(station_b.distance - station_a.distance)
    factor = (
        0.0
        if interval_length <= 0.0
        else (extent - float(station_a.distance)) / interval_length
    )
    expected_length = interval_length
    supported_pairs = []
    for point_a in points_a:
        for point_b in points_b:
            guide = (
                point_a[0] + (point_b[0] - point_a[0]) * factor,
                point_a[1] + (point_b[1] - point_a[1]) * factor,
            )
            if surface.domain.kind == "PLANAR":
                guide_is_owned = _point_in_domain(
                    guide,
                    surface.domain.boundary_triangles,
                    surface.domain.triangle_grid,
                    surface.domain.reference_full_scan,
                )
            else:
                guide_is_owned = surface.domain.locate(guide) is not None
            if not guide_is_owned:
                continue
            supported_pairs.append(
                (
                    abs(sqrt(_dist2(point_a, point_b)) - expected_length),
                    (
                        _dist2(point_a, corner_point)
                        if station_a.distance == 0.0
                        else 0.0
                    ),
                    point_a,
                    point_b,
                    guide,
                )
            )
    if not supported_pairs:
        raise RuntimeError(
            "TERMINAL_BRIDGE_ROUTE_OUTSIDE_OWNER_DOMAIN: "
            f"patch={surface.patch_id} chart={surface.domain.chart_id} "
            f"vertex={terminal.spine_vertex_id} "
            f"guide_edges={terminal.edge_ids}"
        )
    pair = min(
        supported_pairs,
        key=lambda item: (item[0], item[1], item[2], item[3]),
    )
    selected_points = {station_a.station_index: pair[2]}
    valid_station_indices = {
        route_segments[0].from_station_index,
        *(segment.to_station_index for segment in route_segments),
    }
    ordered_stations = tuple(
        sorted(
            (
                station
                for station in route.stations
                if station.station_index in valid_station_indices
            ),
            key=lambda station: (station.distance, station.station_index),
        )
    )
    station_position = {
        station.station_index: index
        for index, station in enumerate(ordered_stations)
    }
    current_position = station_position[station_a.station_index]
    next_point = pair[2]
    for previous_position in range(current_position - 1, -1, -1):
        previous_station = ordered_stations[previous_position]
        next_station = ordered_stations[previous_position + 1]
        candidates = _terminal_station_chart_points(
            surface.domain, previous_station, edge_by_id
        )
        if not candidates:
            raise RuntimeError(
                "TERMINAL_BRIDGE_ROUTE_OUTSIDE_CAP_CHART: "
                f"patch={surface.patch_id} chart={surface.domain.chart_id} "
                f"vertex={terminal.spine_vertex_id} "
                f"station={previous_station.station_index}"
            )
        expected = float(next_station.distance - previous_station.distance)
        previous_point = min(
            candidates,
            key=lambda candidate: (
                abs(sqrt(_dist2(candidate, next_point)) - expected),
                (
                    _dist2(candidate, corner_point)
                    if previous_position == 0
                    else 0.0
                ),
                candidate,
            ),
        )
        selected_points[previous_station.station_index] = previous_point
        next_point = previous_point

    contour_points = [corner_point]
    source_vertex_ids = {int(terminal.spine_vertex_id)}
    station_prefixes = []
    for station in ordered_stations:
        if station.distance <= 0.0 or station.distance > extent:
            continue
        point = selected_points.get(station.station_index)
        if point is None:
            if station.station_index == station_b.station_index and factor == 1.0:
                point = pair[3]
            else:
                continue
        if point != contour_points[-1]:
            contour_points.append(point)
        if station.source_vertex_id is not None:
            source_vertex_ids.add(int(station.source_vertex_id))
        station_prefixes.append(
            _TerminalBridgeStationPrefix(
                extent=float(station.distance),
                point=contour_points[-1],
                contour_points=tuple(contour_points),
                source_vertex_ids=tuple(sorted(source_vertex_ids)),
            )
        )
    # На vertex-station берём сам station point, а не алгебраически
    # интерполированный эквивалент: два почти совпавших endpoint'а создавали
    # ложное микроребро в насыщенном контуре.
    guide_point = pair[3] if factor == 1.0 else pair[4]
    if guide_point != contour_points[-1]:
        contour_points.append(guide_point)
    if factor == 1.0 and station_b.source_vertex_id is not None:
        source_vertex_ids.add(int(station_b.source_vertex_id))
    return _TerminalBridgeGuide(
        point=guide_point,
        contour_points=tuple(contour_points),
        source_vertex_ids=tuple(sorted(source_vertex_ids)),
        route_id=int(route.route_id),
        route_edge_ids=tuple(route_segment_edge_ids),
        extent=float(extent),
        route_reach=float(route_reach),
        station_prefixes=tuple(station_prefixes),
        saturated=bool(requested_extent >= route_reach),
        revisit_guarded=bool(revisit_guarded),
        capacity_policy=getattr(
            terminal,
            "capacity_policy",
            CapacityPolicy.SATURATE_PROVEN,
        ),
    )


def _terminal_route_chart_point(
    surface, terminal, rail_plan, alpha, corner_point
):
    """Compatibility view: только конечная station contact."""

    return _terminal_route_chart_guide(
        surface, terminal, rail_plan, alpha, corner_point
    ).point


def _terminal_station_prefix_guide(guide, prefix):
    """RR8d: immutable view контакта на более ранней route-station."""

    return replace(
        guide,
        point=prefix.point,
        contour_points=prefix.contour_points,
        source_vertex_ids=prefix.source_vertex_ids,
        extent=float(prefix.extent),
        station_prefixes=tuple(
            candidate
            for candidate in guide.station_prefixes
            if candidate.extent <= prefix.extent
        ),
        saturated=True,
        station_clamped=True,
    )


def _resolve_terminal_site_saturation(
    site,
    alpha,
    start_guide,
    end_guide,
    diagnostics=None,
):
    """Клампит одиночный route на последней constructible station.

    Геометрическая ёмкость terminal bridge может закончиться раньше route:
    следующая station уже лежит в owner-domain, но polygon между station и
    исходным site не triangulates. Поэтому ``route.saturated`` не является
    валидатором ёмкости моста. Один и тот же pre-materialization validator
    сначала пробует текущий guide, затем — station prefixes согласно
    CapacityPolicy. Два встречных контакта разрешаются отдельно станционной
    конкуренцией; независимые стороны торца здесь не связываются.
    """

    active = tuple(
        guide for guide in (start_guide, end_guide) if guide is not None
    )
    if len(active) != 1:
        return start_guide, end_guide
    guide = active[0]
    capacity_policy = CapacityPolicy(guide.capacity_policy)
    if guide.saturated and capacity_policy is CapacityPolicy.REJECT_UNPROVEN:
        raise RuntimeError(
            "TERMINAL_ROUTE_CAPACITY_UNPROVEN: "
            f"patch={site.patch_id} edge={site.edge_index}"
        )
    if (
        guide.saturated
        and capacity_policy is CapacityPolicy.CONTROLLED_RECOMPILE
    ):
        raise RuntimeError(
            "TERMINAL_ROUTE_RECOMPILE_REQUIRED: "
            f"patch={site.patch_id} edge={site.edge_index}"
        )
    try:
        _terminal_segment_crop_components(
            site,
            alpha,
            start_guide=start_guide,
            end_guide=end_guide,
        )
        return start_guide, end_guide
    except RuntimeError as exc:
        if not str(exc).startswith("TERMINAL_BRIDGE_CUT_INVALID:"):
            raise

    if capacity_policy is CapacityPolicy.REJECT_UNPROVEN:
        raise RuntimeError(
            "TERMINAL_ROUTE_CAPACITY_UNPROVEN: "
            f"patch={site.patch_id} edge={site.edge_index}"
        )
    if capacity_policy is CapacityPolicy.CONTROLLED_RECOMPILE:
        raise RuntimeError(
            "TERMINAL_ROUTE_RECOMPILE_REQUIRED: "
            f"patch={site.patch_id} edge={site.edge_index}"
        )
    for prefix in reversed(guide.station_prefixes):
        if prefix.extent >= guide.extent:
            continue
        candidate = _terminal_station_prefix_guide(guide, prefix)
        candidate_start = candidate if start_guide is not None else None
        candidate_end = candidate if end_guide is not None else None
        try:
            _terminal_segment_crop_components(
                site,
                alpha,
                start_guide=candidate_start,
                end_guide=candidate_end,
            )
        except RuntimeError as exc:
            if str(exc).startswith("TERMINAL_BRIDGE_CUT_INVALID:"):
                continue
            raise
        if diagnostics is not None:
            diagnostics.terminal_route_station_clamp_count += 1
            diagnostics.record_runtime_policy(
                "TERMINAL_ROUTE_STATION_CLAMPED"
            )
        return candidate_start, candidate_end

    raise RuntimeError(
        "TERMINAL_BRIDGE_CUT_INVALID: "
        f"patch={site.patch_id} edge={site.edge_index} "
        f"vertex={site.vert_a if start_guide is not None else site.vert_b}"
    )


def _surface_terminal_bridge_points(
    surface,
    terminal_routing,
    rail_plan,
    alpha,
    consumed_terminal_ids=None,
    diagnostics=None,
):
    """RM9 guides одного Patch surface; PERP не меняет старый CAP."""

    if rail_plan is None:
        return {}
    result = {}
    terminal_records = {}
    route_by_id = {
        route.route_id: route for route in rail_plan.routes
    }
    terminal_freeze_by_routes = {
        tuple(sorted(locus.route_ids)): locus
        for locus in getattr(rail_plan, "freeze_loci", ())
        if getattr(locus, "competition_kind", None)
        is RailCompetitionKind.TERMINAL_ROUTE_PAIR
    }
    for terminal in terminal_routing or ():
        if (
            terminal.backend != DecalBackendKind.PATCH_VORONOI
            or terminal.patch_id != surface.patch_id
            or terminal.choice == "PERP"
        ):
            continue
        key = (terminal.spine_vertex_id, terminal.spine_edge_id)
        corner = next(
            (
                candidate
                for candidate in surface.corners
                if candidate.vert_index == terminal.spine_vertex_id
                and len(candidate.incident_sites) == 1
                and surface.sites[candidate.incident_sites[0]].edge_index
                == terminal.spine_edge_id
            ),
            None,
        )
        if corner is None:
            continue
        point = _terminal_route_chart_guide(
            surface, terminal, rail_plan, alpha, corner.point
        )
        if diagnostics is not None and point.saturated:
            diagnostics.terminal_route_saturation_count += 1
            diagnostics.record_runtime_policy("TERMINAL_ROUTE_SATURATED")
        if diagnostics is not None and point.revisit_guarded:
            diagnostics.terminal_route_revisit_guard_count += 1
            diagnostics.record_runtime_policy(
                "TERMINAL_ROUTE_REVISIT_GUARDED"
            )
        previous = result.get(key)
        if previous is not None and previous != point:
            raise RuntimeError(
                "TERMINAL_BRIDGE_MULTIPLE_SIDE_GUIDES_UNSUPPORTED: "
                f"patch={surface.patch_id} vertex={key[0]} edge={key[1]}"
            )
        result[key] = point
        terminal_records[key] = (terminal, corner)
        if consumed_terminal_ids is not None:
            consumed_terminal_ids.add(
                (
                    terminal.patch_id,
                    terminal.spine_vertex_id,
                    terminal.spine_edge_id,
                    terminal.route_id,
                )
            )
    meeting_candidates = []
    terminal_keys = tuple(sorted(terminal_records))
    for index, key_a in enumerate(terminal_keys):
        terminal_a, _corner_a = terminal_records[key_a]
        route_a = route_by_id.get(terminal_a.route_id)
        if route_a is None:
            continue
        for key_b in terminal_keys[index + 1 :]:
            terminal_b, _corner_b = terminal_records[key_b]
            if terminal_a.spine_vertex_id == terminal_b.spine_vertex_id:
                continue
            route_b = route_by_id.get(terminal_b.route_id)
            if route_b is None:
                continue
            locus = terminal_freeze_by_routes.get(
                tuple(sorted((route_a.route_id, route_b.route_id)))
            )
            if locus is None:
                continue
            arrival_by_route = dict(
                zip(locus.route_ids, locus.arrival_distances)
            )
            distance_a = arrival_by_route[route_a.route_id]
            distance_b = arrival_by_route[route_b.route_id]
            if alpha < distance_a or alpha < distance_b:
                continue
            meeting_candidates.append(
                (
                    max(distance_a, distance_b),
                    locus.owner_chain_ref,
                    locus.route_ids,
                    key_a,
                    key_b,
                    distance_a,
                    distance_b,
                    locus,
                )
            )
    met_keys = set()
    for (
        _distance,
        _owner_chain_ref,
        _route_ids,
        key_a,
        key_b,
        distance_a,
        distance_b,
        locus,
    ) in sorted(meeting_candidates):
        if key_a in met_keys or key_b in met_keys:
            continue
        terminal_a, corner_a = terminal_records[key_a]
        terminal_b, corner_b = terminal_records[key_b]
        result[key_a] = replace(
            _terminal_route_chart_guide(
                surface,
                terminal_a,
                rail_plan,
                distance_a,
                corner_a.point,
            ),
            contact_met=True,
            frozen=True,
            freeze_locus=locus,
        )
        result[key_b] = replace(
            _terminal_route_chart_guide(
                surface,
                terminal_b,
                rail_plan,
                distance_b,
                corner_b.point,
            ),
            contact_met=True,
            frozen=True,
            freeze_locus=locus,
        )
        met_keys.update((key_a, key_b))
        if diagnostics is not None:
            diagnostics.terminal_contact_meeting_count += 1
            diagnostics.record_runtime_policy("TERMINAL_CONTACT_MEETING")
    for site in surface.sites:
        start_key = (site.vert_a, site.edge_index)
        end_key = (site.vert_b, site.edge_index)
        start_guide = result.get(start_key)
        end_guide = result.get(end_key)
        if start_guide is None and end_guide is None:
            continue
        start_guide, end_guide = _resolve_terminal_site_saturation(
            site,
            alpha,
            start_guide,
            end_guide,
            diagnostics,
        )
        if start_guide is not None:
            result[start_key] = start_guide
        if end_guide is not None:
            result[end_key] = end_guide
    return result


def _arrangement_face_vertex_key(pending_face, point_index, resolved):
    """RM10 identity: contour source-vertex общий с rail geometry."""

    if resolved.location.source_feature == "VERTEX":
        source_vertex_id = int(resolved.location.source_feature_id)
        if source_vertex_id in pending_face.terminal_cut_vertices:
            return ("rail-source-vertex", source_vertex_id)
    if len(pending_face.point_keys) == len(pending_face.points):
        return pending_face.point_keys[point_index]
    return resolved.vert_key


def _evaluate_surface_crops(
    surface,
    alpha,
    pending,
    corner_settings,
    diagnostics=None,
    terminal_bridge_points=None,
    test_join_override=None,
    test_join_vertex_ids=(),
):
    """Строит cell ownership без внутренних endpoint boundaries.

    pyvoronoi отдельно хранит point-cell и две incident segment-cells. Для
    decal corner это одна semantic область. Если материализовать три cells
    независимо, sampled parabola point-cell становится преждевременным fan
    ещё до встречи с другой ветвью. Здесь полный corner crop пересекается с
    объединением этих трёх ownership regions, а incident strips вычитают crop.
    Параболические stations остаются только на реальной границе с
    неincident competitor, то есть появляются непосредственно при collision.
    """

    terminal_bridge_points = terminal_bridge_points or {}
    local_corner_models = _build_local_corner_models(
        surface,
        alpha,
        test_join_override=test_join_override,
        test_join_vertex_ids=test_join_vertex_ids,
    )
    corner_models_by_key = {
        (model.seed.corner_vertex_id, model.seed.sector_id): model
        for model in local_corner_models
    }
    corner_model_geometry_by_key = {
        key: model.derive(
            apex_limit=alpha * corner_settings.apex_limit
        )
        for key, model in corner_models_by_key.items()
    }
    corner_models_by_vertex = {
        vertex_id: model
        for (vertex_id, sector_id), model in corner_models_by_key.items()
        if sector_id is None
    }
    corner_model_geometry = {
        vertex_id: corner_model_geometry_by_key[(vertex_id, None)]
        for vertex_id in corner_models_by_vertex
    }

    def terminal_guide(corner):
        if len(corner.incident_sites) != 1:
            return None
        site = surface.sites[corner.incident_sites[0]]
        return terminal_bridge_points.get(
            (corner.vert_index, site.edge_index)
        )

    def site_terminal_guides(site, periodic_shift=0):
        guides = (
            terminal_bridge_points.get((site.vert_a, site.edge_index)),
            terminal_bridge_points.get((site.vert_b, site.edge_index)),
        )
        if not periodic_shift or not getattr(surface.domain, "period", 0.0):
            return guides
        offset = int(periodic_shift) * float(surface.domain.period)
        return tuple(
            None
            if guide is None
            else replace(
                guide,
                point=(guide.point[0] + offset, guide.point[1]),
                contour_points=tuple(
                    (point[0] + offset, point[1])
                    for point in guide.contour_points
                ),
            )
            if isinstance(guide, _TerminalBridgeGuide)
            else (guide[0] + offset, guide[1])
            for guide in guides
        )

    reference_full_scan = bool(
        diagnostics is not None and diagnostics.reference_full_scan
    )
    point_atoms_by_corner = {}
    if reference_full_scan:
        atom_indices_by_corner = {
            corner_index: range(len(surface.atoms))
            for corner_index in range(len(surface.corners))
        }
    elif hasattr(surface, "owner_atoms_by_corner"):
        atom_indices_by_corner = surface.owner_atoms_by_corner
    else:
        atom_indices_by_corner = {
            corner_index: range(len(surface.atoms))
            for corner_index in range(len(surface.corners))
        }
    for corner_index, atom_indices in atom_indices_by_corner.items():
        point_atoms = tuple(
            surface.atoms[atom_index]
            for atom_index in atom_indices
            if surface.atoms[atom_index].cell_kind == "POINT"
            and surface.atoms[atom_index].corner_index == corner_index
        )
        if point_atoms:
            point_atoms_by_corner[corner_index] = point_atoms

    corner_crops = {}
    # Обычный endpoint corner существует только там, где pyvoronoi дал
    # отдельную point-cell. Острый convex miter такой cell не имеет: две
    # segment-cells сходятся непосредственно по биссектрисе. Но его всё
    # равно нужно сначала собрать в один semantic corner, а затем разделить
    # на INNER/OUTER. Иначе длинный miter режется конкурентами как два
    # независимых крыла и визуально распадается при большой ширине.
    runtime_policies = tuple(
        _classify_surface_corner_runtime(
            surface, corner, corner_settings
        )
        for corner in surface.corners
    )
    structurally_absorbed_corner_indices = set()
    if _surface_is_approximate(surface):
        quantum = max(float(surface.diagram_transform.quantum), 1e-10)
        for corner_index, (corner, policy) in enumerate(
            zip(surface.corners, runtime_policies)
        ):
            if len(corner.incident_sites) != 2:
                continue
            raw_probe_crops = _corner_crop_components(
                surface,
                corner,
                policy,
                alpha,
                corner_settings,
                diagnostics=None,
                terminal_guide=terminal_guide(corner),
                corner_model=corner_models_by_vertex.get(
                    int(corner.vert_index)
                ),
                corner_derived=corner_model_geometry.get(
                    int(corner.vert_index)
                ),
            )
            owned_probe_crops = tuple(
                owned_crop
                for crop in raw_probe_crops
                for owned_crop in (
                    _corner_endpoint_ownership_crop(
                        surface, corner, crop
                    ),
                )
                if owned_crop is not None
            )
            if not owned_probe_crops or all(
                _m1_crop_structure_absorbed(crop.points, quantum)
                for crop in owned_probe_crops
            ):
                structurally_absorbed_corner_indices.add(corner_index)
    if diagnostics is not None:
        for corner, policy in zip(surface.corners, runtime_policies):
            for entry in _corner_runtime_policy_entries(
                surface, corner, policy, corner_settings
            ):
                diagnostics.record_runtime_policy(entry)
    if corner_settings.dynamic_corner_bands:
        explicit_corner_indices = {
            corner_index
            for corner_index, (corner, policy) in enumerate(
                zip(surface.corners, runtime_policies)
            )
            if len(corner.incident_sites) == 2
            and abs(corner.interior_angle - pi) > 1e-7
        }
    else:
        explicit_corner_indices = {
            corner_index
            for corner_index, (corner, policy) in enumerate(
                zip(surface.corners, runtime_policies)
            )
            if len(corner.incident_sites) == 2
            and (
                policy in {
                    _CornerPolicy.SMOOTH,
                    _CornerPolicy.ACUTE_SPLIT,
                }
                or corner_model_geometry.get(
                    int(corner.vert_index), None
                ) is not None
                and corner_model_geometry[
                    int(corner.vert_index)
                ].releases_competitor
                or (
                    policy == _CornerPolicy.MITER
                    and _miter_requires_explicit_crop(
                        surface, corner, alpha, corner_settings
                    )
                )
            )
        }
    corner_indices = sorted(
        set(point_atoms_by_corner) | explicit_corner_indices
    )
    corner_indices_by_site = {}
    for corner_index in sorted(
        set(corner_indices) | structurally_absorbed_corner_indices
    ):
        for site_index in surface.corners[corner_index].incident_sites:
            corner_indices_by_site.setdefault(site_index, []).append(
                corner_index
            )
    absorbed_corner_indices = set(structurally_absorbed_corner_indices)
    crops_by_site = {}
    for corner_index in corner_indices:
        point_atoms = point_atoms_by_corner.get(corner_index, ())
        corner = surface.corners[corner_index]
        policy = runtime_policies[corner_index]
        raw_crops = _corner_crop_components(
            surface,
            corner,
            policy,
            alpha,
            corner_settings,
            diagnostics,
            terminal_guide=terminal_guide(corner),
            corner_model=corner_models_by_vertex.get(
                int(corner.vert_index)
            ),
            corner_derived=corner_model_geometry.get(
                int(corner.vert_index)
            ),
        )
        crops = tuple(
            replace(
                owned_crop,
                semantic_owner_id=(
                    owned_crop.semantic_owner_id
                    if owned_crop.semantic_owner_id is not None
                    else ("corner", int(corner.vert_index))
                ),
            )
            for crop in raw_crops
            for owned_crop in (
                _corner_endpoint_ownership_crop(surface, corner, crop),
            )
            if owned_crop is not None
        )
        if not crops:
            absorbed_corner_indices.add(corner_index)
            continue
        corner_crops[corner_index] = crops
        corner_emitted = False
        split_dynamic_atoms = (
            corner_settings.dynamic_corner_bands
            and policy
            in {
                _CornerPolicy.FAN,
                _CornerPolicy.ACUTE_SPLIT,
                _CornerPolicy.HAIRPIN,
            }
        )
        for crop in crops:
            owner_site_indices = (
                crop.owner_site_indices or corner.incident_sites
            )
            for site_index in owner_site_indices:
                crops_by_site.setdefault(site_index, []).append(
                    (corner, crop)
                )
            if reference_full_scan:
                owner_atom_indices = range(len(surface.atoms))
            elif hasattr(surface, "owner_atoms_by_corner"):
                owner_atom_indices = surface.owner_atoms_by_corner.get(
                    corner_index, ()
                )
            else:
                owner_atom_indices = range(len(surface.atoms))
            owner_atoms = [
                surface.atoms[atom_index]
                for atom_index in owner_atom_indices
                if (
                    surface.atoms[atom_index].site_index
                    in owner_site_indices
                    and surface.atoms[atom_index].cell_kind == "SEGMENT"
                )
                or (
                    surface.atoms[atom_index].cell_kind == "POINT"
                    and surface.atoms[atom_index].corner_index == corner_index
                )
            ]
            owner_site_index = min(
                (
                    atom.site_index
                    for atom in point_atoms
                    if atom.site_index in owner_site_indices
                ),
                default=min(owner_site_indices),
            )
            if split_dynamic_atoms:
                atom_groups = tuple(
                    (
                        _corner_atom_image_offset(surface, corner, atom),
                        (atom,),
                    )
                    for atom in owner_atoms
                )
            else:
                atoms_by_offset = {}
                for atom in owner_atoms:
                    atoms_by_offset.setdefault(
                        _corner_atom_image_offset(surface, corner, atom), []
                    ).append(atom)
                atom_groups = tuple(
                    (offset, tuple(atoms_by_offset[offset]))
                    for offset in sorted(atoms_by_offset)
                )
            for image_offset, atoms in atom_groups:
                if not atoms:
                    continue
                image_crop = _translated_crop_u(crop, image_offset)
                if _surface_is_approximate(surface):
                    fragments = (image_crop.points,)
                    fragment_merge_groups = ()
                else:
                    fragments = []
                    fragment_merge_groups = []
                    for atom in atoms:
                        for fragment, merge_group in _atom_fragment_records(
                            surface, atom
                        ):
                            clipped = _clip_to_convex(
                                fragment, image_crop.points
                            )
                            if clipped:
                                fragments.append(clipped)
                                fragment_merge_groups.append(merge_group)
                image_site = _translated_site_u(
                    _corner_site_view(
                        surface, corner, owner_site_index
                    ),
                    image_offset,
                )
                pending_count = len(pending)
                _append_pending_fragments(
                    pending,
                    surface,
                    image_site,
                    image_crop,
                    fragments,
                    diagnostics,
                    fragment_merge_groups=fragment_merge_groups,
                )
                corner_emitted = corner_emitted or len(pending) > pending_count
        if not corner_emitted:
            absorbed_corner_indices.add(corner_index)

    # RC5a: BEVEL освобождает часть primary point-cell. Она не исчезает и не
    # остаётся невидимой стеной: compile-static second-owner partition отдаёт
    # её дотекающим чужим SEGMENT-потокам. Собственные incident sites в этой
    # partition отсутствуют, поэтому их faces/limits остаются MITER-identical.
    release_atoms_by_corner = getattr(surface, "corner_release_atoms", {})
    if release_atoms_by_corner and not _surface_is_approximate(surface):
        period = float(
            getattr(getattr(surface, "domain", None), "period", 0.0)
        )
        for corner_index, crops in sorted(corner_crops.items()):
            corner = surface.corners[corner_index]
            derived = corner_model_geometry.get(int(corner.vert_index))
            if derived is None or not derived.releases_competitor:
                continue
            corner_offsets = dict(corner.site_u_offsets)
            for release_atom in release_atoms_by_corner.get(corner_index, ()):
                join_offset = (
                    release_atom.point_periodic_shift * period
                    - corner_offsets.get(
                        release_atom.point_site_index, 0.0
                    )
                )
                image_join_crops = tuple(
                    _translated_crop_u(crop, join_offset) for crop in crops
                )
                source_site = surface.sites[
                    release_atom.owner_site_index
                ]
                site = _periodic_site_image(
                    surface,
                    source_site,
                    release_atom.owner_periodic_shift,
                )
                start_guide, end_guide = site_terminal_guides(
                    source_site, release_atom.owner_periodic_shift
                )
                for segment_crop, terminal_cut_vertices in (
                    _terminal_segment_crop_components(
                        site,
                        alpha,
                        start_guide=start_guide,
                        end_guide=end_guide,
                    )
                ):
                    fragments = []
                    fragment_merge_groups = []
                    for fragment, merge_group in _atom_fragment_records(
                        surface, release_atom
                    ):
                        pieces = [fragment]
                        for join_crop in image_join_crops:
                            pieces = [
                                outside
                                for piece in pieces
                                for outside in _subtract_convex_polygon(
                                    piece, join_crop.points
                                )
                            ]
                            if not pieces:
                                break
                        for piece in pieces:
                            clipped = _clip_to_convex(
                                piece, segment_crop.points
                            )
                            if (
                                len(clipped) < 3
                                or abs(_polygon_area2(clipped)) <= 1e-10
                            ):
                                continue
                            fragments.append(clipped)
                            fragment_merge_groups.append(merge_group)
                    if fragments:
                        _append_pending_fragments(
                            pending,
                            surface,
                            site,
                            segment_crop,
                            fragments,
                            diagnostics,
                            terminal_cut_vertices=terminal_cut_vertices,
                            fragment_merge_groups=fragment_merge_groups,
                        )

    for atom in surface.atoms:
        if atom.cell_kind == "POINT" and atom.corner_index in corner_crops:
            continue
        source_site = surface.sites[atom.site_index]
        site = _periodic_site_image(
            surface, source_site, atom.periodic_shift
        )
        start_guide, end_guide = site_terminal_guides(
            source_site, atom.periodic_shift
        )
        subtraction_crops = tuple(
            _translated_crop_u(
                corner_crop,
                _corner_atom_image_offset(surface, corner, atom),
            )
            for corner, corner_crop in crops_by_site.get(
                atom.site_index, ()
            )
        )
        for crop, terminal_cut_vertices in (
            _terminal_segment_crop_components(
                site,
                alpha,
                start_guide=start_guide,
                end_guide=end_guide,
            )
        ):
            fragments = []
            fragment_merge_groups = []
            if _surface_is_approximate(surface):
                fragments = (crop.points,)
                fragment_merge_groups = ()
            else:
                for fragment, merge_group in _atom_fragment_records(
                    surface, atom
                ):
                    clipped = _clip_to_convex(fragment, crop.points)
                    if not clipped:
                        continue
                    pieces = [clipped]
                    for corner_crop in subtraction_crops:
                        pieces = [
                            outside
                            for piece in pieces
                            for outside in _subtract_convex_polygon(
                                piece, corner_crop.points
                            )
                        ]
                        if not pieces:
                            break
                    fragments.extend(pieces)
                    fragment_merge_groups.extend(
                        merge_group for _piece in pieces
                    )
            _append_pending_fragments(
                pending,
                surface,
                site,
                crop,
                fragments,
                diagnostics,
                absorbed_corner_vertices=tuple(
                    surface.corners[corner_index].vert_index
                    for corner_index in corner_indices_by_site.get(
                        atom.site_index, ()
                    )
                    if corner_index in absorbed_corner_indices
                ),
                terminal_cut_vertices=terminal_cut_vertices,
                fragment_merge_groups=fragment_merge_groups,
            )

    return tuple(
        (
            int(surface.patch_id),
            (
                "corner-model",
                model.seed.corner_vertex_id,
                model.seed.sector_id,
            ),
            model,
            derived,
            int(surface.domain.chart_id),
        )
        for key, model in corner_models_by_key.items()
        for derived in (corner_model_geometry_by_key[key],)
    )


def _terminal_partition_loop_fact(face, loop_index):
    """Точный geometry/UV факт loop для lossless terminal merge."""

    return (
        tuple(float(value) for value in face.positions[loop_index]),
        float(face.u_fracs[loop_index]),
        float(face.v_lengths[loop_index]),
    )


def _terminal_partition_side(side):
    """Участвует ли SEGMENT face во внутренней RM9-partition."""

    value = str(side or "")
    return (
        value in {"BODY", "CAP_ALIGNED"}
        or value.startswith("TERMINAL_")
    )


def _terminal_partition_edge_is_lossless(faces, first, second):
    """Можно ли убрать общий edge без смены surface/UV семантики."""

    first_face_index, first_loop, first_a, first_b = first
    second_face_index, second_loop, second_a, second_b = second
    first_face = faces[first_face_index]
    second_face = faces[second_face_index]
    first_side = str(first_face.component_side or "")
    second_side = str(second_face.component_side or "")
    cap_aligned_pair = (
        first_side == "CAP_ALIGNED" or second_side == "CAP_ALIGNED"
    )
    terminal_partition_pair = (
        _terminal_partition_side(first_side)
        and _terminal_partition_side(second_side)
        and (
            first_side.startswith("TERMINAL_")
            or second_side.startswith("TERMINAL_")
        )
    )
    if first_face_index == second_face_index:
        return False
    if first_a != second_b or first_b != second_a:
        return False
    if (
        first_face.component_kind != "SEGMENT"
        or second_face.component_kind != "SEGMENT"
        or not (cap_aligned_pair or terminal_partition_pair)
        or first_face.surface_id != second_face.surface_id
        or first_face.surface_normal.dot(second_face.surface_normal)
        < DECAL_COPLANAR_DOT
    ):
        return False
    first_count = len(first_face.vert_keys)
    second_count = len(second_face.vert_keys)
    return (
        _terminal_partition_loop_fact(first_face, first_loop)
        == _terminal_partition_loop_fact(
            second_face, (second_loop + 1) % second_count
        )
        and _terminal_partition_loop_fact(
            first_face, (first_loop + 1) % first_count
        )
        == _terminal_partition_loop_fact(second_face, second_loop)
    )


def _merged_terminal_partition_side(component_faces):
    sides = {
        "START"
        if str(face.component_side).startswith("TERMINAL_START_")
        else "END"
        if str(face.component_side).startswith("TERMINAL_END_")
        else "OTHER"
        for face in component_faces
        if str(face.component_side).startswith("TERMINAL_")
    }
    if sides == {"START"}:
        return "TERMINAL_START_MERGED"
    if sides == {"END"}:
        return "TERMINAL_END_MERGED"
    if not sides:
        return "CAP_ALIGNED_MERGED"
    return "TERMINAL_BOTH_MERGED"


def _aligned_strip_cap_side(strip_side):
    value = str(strip_side or "")
    if value.startswith("TERMINAL_START_"):
        return "TERMINAL_START_CAP_ALIGNED"
    if value.startswith("TERMINAL_END_"):
        return "TERMINAL_END_CAP_ALIGNED"
    return "CAP_ALIGNED"


def _align_strip_cap_face(cap_face, cap_loop, strip_face, strip_loop):
    """Переводит CAP в station-UV и возвращает факт affine-transform."""

    cap_count = len(cap_face.vert_keys)
    strip_count = len(strip_face.vert_keys)
    cap_a = cap_face.vert_keys[cap_loop]
    cap_b = cap_face.vert_keys[(cap_loop + 1) % cap_count]
    strip_a = strip_face.vert_keys[strip_loop]
    strip_b = strip_face.vert_keys[
        (strip_loop + 1) % strip_count
    ]
    if cap_a != strip_b or cap_b != strip_a:
        return None
    cap_a_fact = _terminal_partition_loop_fact(cap_face, cap_loop)
    cap_b_fact = _terminal_partition_loop_fact(
        cap_face, (cap_loop + 1) % cap_count
    )
    strip_a_fact = _terminal_partition_loop_fact(
        strip_face, strip_loop
    )
    strip_b_fact = _terminal_partition_loop_fact(
        strip_face, (strip_loop + 1) % strip_count
    )
    if cap_a_fact[0] != strip_b_fact[0] or cap_b_fact[0] != strip_a_fact[0]:
        return None

    cap_u_a = cap_a_fact[1]
    cap_u_b = cap_b_fact[1]
    strip_u_a = strip_b_fact[1]
    strip_u_b = strip_a_fact[1]
    cap_u_span = cap_u_b - cap_u_a
    if cap_u_span == 0.0:
        return None
    u_scale = (strip_u_b - strip_u_a) / cap_u_span
    u_offset = strip_u_a - cap_u_a * u_scale

    cap_v = cap_a_fact[2]
    strip_v = strip_b_fact[2]
    v_offset = strip_v - cap_v
    u_fracs = [
        strip_u_a + (float(value) - cap_u_a) * u_scale
        for value in cap_face.u_fracs
    ]
    v_lengths = [
        float(value) + v_offset for value in cap_face.v_lengths
    ]
    # Shared keys — источник истины. Остаточный float drift CAP-frame не
    # должен оставлять UV seam после структурного выравнивания.
    u_fracs[cap_loop] = strip_u_a
    v_lengths[cap_loop] = strip_v
    cap_next = (cap_loop + 1) % cap_count
    u_fracs[cap_next] = strip_u_b
    v_lengths[cap_next] = strip_a_fact[2]
    return (
        _NetworkFace(
            surface_id=cap_face.surface_id,
            surface_normal=cap_face.surface_normal.copy(),
            vert_keys=list(cap_face.vert_keys),
            positions=list(cap_face.positions),
            u_fracs=u_fracs,
            v_lengths=v_lengths,
            component_kind="SEGMENT",
            component_side=_aligned_strip_cap_side(
                strip_face.component_side
            ),
            provenance=cap_face.provenance,
        ),
        (u_scale, u_offset, v_offset),
    )


def _align_strip_cap_neighbor(cap_face, strip_face, shared_edges):
    """Один affine seed + shared-key canonicalization всей adjacency."""

    adjacency = {}
    cap_indices_by_key = {}
    strip_facts_by_key = {}
    seeds = []
    for cap_use, strip_use in shared_edges:
        cap_loop = cap_use[1]
        strip_loop = strip_use[1]
        cap_count = len(cap_face.vert_keys)
        strip_count = len(strip_face.vert_keys)
        cap_keys = (
            cap_face.vert_keys[cap_loop],
            cap_face.vert_keys[(cap_loop + 1) % cap_count],
        )
        strip_keys = (
            strip_face.vert_keys[(strip_loop + 1) % strip_count],
            strip_face.vert_keys[strip_loop],
        )
        if cap_keys != strip_keys:
            return None, "CAP_KEEP_SHARED_GEOMETRY_MISMATCH"
        adjacency.setdefault(cap_keys[0], set()).add(cap_keys[1])
        adjacency.setdefault(cap_keys[1], set()).add(cap_keys[0])
        for cap_index, strip_index, key in (
            (cap_loop, (strip_loop + 1) % strip_count, cap_keys[0]),
            ((cap_loop + 1) % cap_count, strip_loop, cap_keys[1]),
        ):
            cap_indices_by_key.setdefault(key, set()).add(cap_index)
            fact = (
                float(strip_face.u_fracs[strip_index]),
                float(strip_face.v_lengths[strip_index]),
            )
            previous = strip_facts_by_key.setdefault(key, fact)
            if previous != fact:
                return None, "CAP_KEEP_STATION_KEY_DESYNC"
        alignment = _align_strip_cap_face(
            cap_face, cap_loop, strip_face, strip_loop
        )
        if alignment is not None:
            cap_a = float(cap_face.u_fracs[cap_loop])
            cap_b = float(
                cap_face.u_fracs[(cap_loop + 1) % cap_count]
            )
            seeds.append(
                (
                    abs(cap_b - cap_a),
                    repr(cap_keys),
                    alignment[0],
                )
            )
    if not adjacency or any(len(neighbors) > 2 for neighbors in adjacency.values()):
        return None, "CAP_KEEP_SHARED_EDGE_DISCONNECTED"
    pending = [min(adjacency, key=repr)]
    visited = set()
    while pending:
        key = pending.pop()
        if key in visited:
            continue
        visited.add(key)
        pending.extend(adjacency[key])
    if visited != set(adjacency):
        return None, "CAP_KEEP_SHARED_EDGE_DISCONNECTED"
    if not seeds:
        return None, "CAP_KEEP_STATION_UV_DISCONTINUOUS"

    # Самый широкий общий span — детерминированный affine seed. Все общие
    # keys после этого читают UV из SEGMENT как единственного источника;
    # так roundoff разных clipping pieces не превращается в epsilon-policy.
    _span, _key, aligned = max(seeds, key=lambda item: (item[0], item[1]))
    for key, cap_indices in cap_indices_by_key.items():
        u_frac, v_length = strip_facts_by_key[key]
        for cap_index in cap_indices:
            aligned.u_fracs[cap_index] = u_frac
            aligned.v_lengths[cap_index] = v_length
    return aligned, None


def _align_strip_cap_faces(faces, diagnostics=None):
    """Поглощает CAP по однозначной station-UV смежности со strip'ом.

    CAP остаётся геометрически нужным закрытием owner surface, но перестаёт
    быть отдельным UV-piece: его поперечная координата аффинно привязывается
    к общему ребру, а station V переносится одним сдвигом. Геометрическая
    копланарность и surface_id не участвуют: fold не является UV seam.
    Каждый непоглощённый CAP получает именованную counted-причину.
    """

    faces = tuple(faces)
    edge_uses = {}
    for face_index, face in enumerate(faces):
        count = len(face.vert_keys)
        for loop_index, key in enumerate(face.vert_keys):
            following = face.vert_keys[(loop_index + 1) % count]
            edge_uses.setdefault(frozenset((key, following)), []).append(
                (face_index, loop_index, key, following)
            )

    shared_by_cap = {}
    for uses in edge_uses.values():
        if len(uses) != 2:
            continue
        first, second = uses
        first_face = faces[first[0]]
        second_face = faces[second[0]]
        if first_face.component_kind == "CAP":
            cap_use, strip_use = first, second
        elif second_face.component_kind == "CAP":
            cap_use, strip_use = second, first
        else:
            continue
        cap_face = faces[cap_use[0]]
        strip_face = faces[strip_use[0]]
        if (
            strip_face.component_kind != "SEGMENT"
            or strip_face.surface_id != cap_face.surface_id
        ):
            continue
        shared_by_cap.setdefault(cap_use[0], {}).setdefault(
            strip_use[0], []
        ).append((cap_use, strip_use))

    replacements = {}
    for cap_index, cap_face in enumerate(faces):
        if cap_face.component_kind != "CAP":
            continue
        neighbors = shared_by_cap.get(cap_index, {})
        if not neighbors:
            reason = "CAP_KEEP_NO_SEGMENT_NEIGHBOR"
        elif len(neighbors) != 1:
            reason = "CAP_KEEP_MULTIPLE_SEGMENT_NEIGHBORS"
        else:
            strip_index, shared_edges = next(iter(neighbors.items()))
            aligned, reason = _align_strip_cap_neighbor(
                cap_face,
                faces[strip_index],
                shared_edges,
            )
            if aligned is not None:
                replacements[cap_index] = aligned
                continue
        if diagnostics is not None:
            diagnostics.record_cap_keep(reason)

    return [replacements.get(index, face) for index, face in enumerate(faces)]


def _merge_terminal_partition_component(faces, component):
    """Собирает один простой boundary-cycle из RM9-partition faces."""

    component = tuple(sorted(component))
    component_faces = tuple(faces[index] for index in component)
    reference_normal = component_faces[0].surface_normal
    if any(
        reference_normal.dot(face.surface_normal) < DECAL_COPLANAR_DOT
        for face in component_faces[1:]
    ):
        return None

    facts_by_key = {}
    raw_by_key = {}
    provenance_by_key = {}
    edge_uses = {}
    for face_index in component:
        face = faces[face_index]
        face_provenance = getattr(face, "provenance", None)
        if not isinstance(face_provenance, PatchVoronoiFaceProvenance):
            raise RuntimeError("PATCH_VORONOI_MERGE_PROVENANCE_MISSING")
        count = len(face.vert_keys)
        for loop_index, key in enumerate(face.vert_keys):
            fact = _terminal_partition_loop_fact(face, loop_index)
            previous = facts_by_key.setdefault(key, fact)
            if previous != fact:
                return None
            raw_by_key.setdefault(
                key,
                (
                    face.positions[loop_index],
                    face.u_fracs[loop_index],
                    face.v_lengths[loop_index],
                ),
            )
            provenance_by_key.setdefault(key, []).append(
                face_provenance.vertices[loop_index]
            )
            following = face.vert_keys[(loop_index + 1) % count]
            edge_uses.setdefault(frozenset((key, following)), []).append(
                (face_index, loop_index, key, following)
            )

    boundary = []
    for uses in edge_uses.values():
        if len(uses) == 1:
            boundary.append(uses[0])
            continue
        if len(uses) != 2 or not _terminal_partition_edge_is_lossless(
            faces, uses[0], uses[1]
        ):
            return None
    if len(boundary) < 3:
        return None

    outgoing = {}
    incoming = {}
    for use in boundary:
        _face_index, _loop_index, start, end = use
        if start in outgoing or end in incoming:
            return None
        outgoing[start] = end
        incoming[end] = start
    if set(outgoing) != set(incoming):
        return None

    start = min(outgoing, key=repr)
    cycle = []
    visited_edges = set()
    current = start
    for _step in range(len(boundary) + 1):
        if current == start and cycle:
            break
        if current in cycle or current not in outgoing:
            return None
        cycle.append(current)
        following = outgoing[current]
        visited_edges.add((current, following))
        current = following
    if current != start or len(visited_edges) != len(boundary):
        return None
    if len(cycle) < 3 or len(set(cycle)) != len(cycle):
        return None

    normal = Vector((0.0, 0.0, 0.0))
    for face in component_faces:
        normal = normal + face.surface_normal
    if normal.length_squared <= _GEOMETRY_EPS * _GEOMETRY_EPS:
        normal = reference_normal.copy()
    else:
        normal = normal.normalized()
    return _NetworkFace(
        surface_id=component_faces[0].surface_id,
        surface_normal=normal,
        vert_keys=list(cycle),
        positions=[raw_by_key[key][0] for key in cycle],
        u_fracs=[raw_by_key[key][1] for key in cycle],
        v_lengths=[raw_by_key[key][2] for key in cycle],
        component_kind="SEGMENT",
        component_side=_merged_terminal_partition_side(component_faces),
        provenance=_patch_face_provenance(
            tuple(
                min(provenance_by_key[key], key=repr) for key in cycle
            ),
            semantic_owner_id=(
                "terminal-merge",
                _ordered_unique_provenance(
                    face.provenance.semantic_owner_id
                    for face in component_faces
                    if face.provenance.semantic_owner_id is not None
                ),
            ),
        ),
    )


def _merge_terminal_partition_faces(faces, diagnostics=None):
    """RD-1: убирает только lossless внутренние швы RM9 terminal cut.

    Геометрия terminal cut сначала остаётся разложенной на convex pieces для
    надёжного clipping/arrangement. После lift соседние SEGMENT pieces одной
    surface с точными общими position/UV facts собираются обратно в один
    boundary-cycle. Fold, UV seam, CAP и любой не-terminal edge не затрагиваются.
    """

    faces = tuple(_align_strip_cap_faces(faces, diagnostics))
    edge_uses = {}
    for face_index, face in enumerate(faces):
        count = len(face.vert_keys)
        for loop_index, key in enumerate(face.vert_keys):
            following = face.vert_keys[(loop_index + 1) % count]
            edge_uses.setdefault(frozenset((key, following)), []).append(
                (face_index, loop_index, key, following)
            )

    adjacency = {}
    for uses in edge_uses.values():
        if len(uses) != 2 or not _terminal_partition_edge_is_lossless(
            faces, uses[0], uses[1]
        ):
            continue
        first = uses[0][0]
        second = uses[1][0]
        adjacency.setdefault(first, set()).add(second)
        adjacency.setdefault(second, set()).add(first)

    replacements = {}
    removed = set()
    remaining = set(adjacency)
    while remaining:
        seed = min(remaining)
        stack = [seed]
        component = set()
        while stack:
            current = stack.pop()
            if current in component:
                continue
            component.add(current)
            stack.extend(adjacency.get(current, ()))
        remaining.difference_update(component)
        merged = _merge_terminal_partition_component(faces, component)
        if merged is None:
            continue
        first = min(component)
        replacements[first] = merged
        removed.update(component.difference({first}))

    return [
        replacements.get(index, face)
        for index, face in enumerate(faces)
        if index not in removed
    ]


def evaluate_patch_voronoi_plan(
    plan,
    width,
    preview=False,
    corner_settings=None,
    diagnostics=None,
    terminal_routing=(),
    rail_plan=None,
):
    """Перестраивает extrusion polygons внутри статических Voronoi cells."""

    corner_settings = _normalized_corner_runtime_settings(corner_settings)
    runtime_join = CornerJoinMode(corner_settings.join_mode)
    compiled_join = CornerJoinMode(
        getattr(plan, "corner_join_mode", CornerJoinMode.MITER)
    )
    if runtime_join is not compiled_join:
        raise RuntimeError(
            f"{DECAL_CORNER_JOIN_RECOMPILE_REQUIRED}: "
            f"compiled={compiled_join.value} "
            f"runtime={runtime_join.value}"
        )
    if diagnostics is not None:
        diagnostics.runtime_policy_counts.clear()
        diagnostics.cap_keep_counts.clear()
        diagnostics.periodic_weld_count = 0
        diagnostics.interior_weld_count = 0
        diagnostics.atlas_sliver_owner_count = 0
        diagnostics.atlas_max_sliver_owner_distance = 0.0
        diagnostics.atlas_no_owner_drop_count = 0
        diagnostics.atlas_declared_owner_count = 0
        diagnostics.atlas_declared_owner_missing_count = 0
        diagnostics.atlas_degenerate_interval_merge_count = 0
        diagnostics.atlas_undeclared_materialized_interval_count = 0
        diagnostics.atlas_clamped_segment_face_count = 0
        diagnostics.atlas_v_continuation_count = 0
        diagnostics.atlas_touch_no_token_drop_count = 0
        diagnostics.atlas_single_side_drop_count = 0
        diagnostics.atlas_semantic_import_count = 0
        diagnostics.atlas_semantic_transition_count = 0
        diagnostics.atlas_arrangement_integrity_failure_count = 0
        diagnostics.terminal_route_saturation_count = 0
        diagnostics.terminal_route_station_clamp_count = 0
        diagnostics.terminal_route_revisit_guard_count = 0
        diagnostics.terminal_contact_meeting_count = 0
        diagnostics.resolved_corner_view_count = 0
        diagnostics.resolved_corner_boundary_vertex_count = 0
        diagnostics.resolved_corner_competition_vertex_count = 0
    alpha = max(1e-6, float(width) * 0.5)
    # Проверка выполняется до crop/arrangement: excess frame не имеет
    # geometry side effects и modal может оставить последний valid preview.
    plan.active_triangle_ids(alpha)
    lateral_at_full_offset = (
        abs(plan.offset) * plan.max_lateral_lift_ratio
    )
    if lateral_at_full_offset > _GEOMETRY_EPS:
        # Общий offset-rail не должен пересекать внешний фронт узкого крыла.
        # Масштаб един для всей selection: shared vertices остаются общими,
        # а все owner planes сохраняют одинаковый фактический offset.
        lift_scale = min(1.0, alpha * 0.75 / lateral_at_full_offset)
    else:
        lift_scale = 1.0
    pending = []
    expected_terminal_ids = {
        (
            terminal.patch_id,
            terminal.spine_vertex_id,
            terminal.spine_edge_id,
            terminal.route_id,
        )
        for terminal in terminal_routing or ()
        if terminal.backend == DecalBackendKind.PATCH_VORONOI
        and terminal.choice != "PERP"
    }
    consumed_terminal_ids = set()
    corner_resolution_sources = []
    for surface in plan.surfaces:
        terminal_bridge_points = _surface_terminal_bridge_points(
            surface,
            terminal_routing,
            rail_plan,
            alpha,
            consumed_terminal_ids=consumed_terminal_ids,
            diagnostics=diagnostics,
        )
        corner_resolution_sources.extend(
            _evaluate_surface_crops(
                surface,
                alpha,
                pending,
                corner_settings,
                diagnostics,
                terminal_bridge_points=terminal_bridge_points,
            )
        )
    missing_terminal_ids = expected_terminal_ids.difference(
        consumed_terminal_ids
    )
    if missing_terminal_ids:
        raise RuntimeError(
            "TERMINAL_BRIDGE_GUIDE_UNCONSUMED: "
            + ",".join(repr(key) for key in sorted(missing_terminal_ids))
        )

    arrangement = _build_decal_arrangement(
        pending,
        tolerance=max(1e-8, DECAL_WELD_DISTANCE * 0.5),
        diagnostics=diagnostics,
        alpha=alpha,
        corner_resolution_sources=tuple(corner_resolution_sources),
    )
    if diagnostics is not None:
        diagnostics.resolved_corner_view_count = len(
            arrangement.resolved_corner_views
        )
        diagnostics.resolved_corner_boundary_vertex_count = sum(
            len(view.materialized_vertices)
            for view in arrangement.resolved_corner_views
        )
        diagnostics.resolved_corner_competition_vertex_count = sum(
            len(view.competition_vertices)
            for view in arrangement.resolved_corner_views
        )
    pending = arrangement.faces
    desired_lift_scale = lift_scale
    resolved_points = {}
    lift_scale = _orientation_safe_lift_scale(
        plan,
        pending,
        desired_lift_scale,
        resolved_points,
    )
    lift_fraction = (
        lift_scale / desired_lift_scale
        if desired_lift_scale > _GEOMETRY_EPS
        else 0.0
    )
    faces = []
    emitted_faces = set()
    for pending_face in pending:
        surface = pending_face.surface
        site = pending_face.site
        uv_site = pending_face.uv_site or site
        component = pending_face.points
        crop = pending_face.crop
        vert_keys = []
        positions = []
        u_fracs = []
        v_lengths = []
        vertex_provenance = []
        used_keys = set()
        transition_uv = {
            key: (u_fraction, v_length)
            for key, u_fraction, v_length
            in pending_face.transition_uv
        }
        for point_index, point in enumerate(component):
            resolved = _resolve_arrangement_point(
                plan,
                surface,
                site,
                point,
                desired_lift_scale,
                resolved_points,
            )
            position = resolved.position_zero.lerp(
                resolved.position_full, lift_fraction
            )
            vert_key = _arrangement_face_vertex_key(
                pending_face,
                point_index,
                resolved,
            )
            # Triangle boundaries и pyvoronoi endpoint-cells могут
            # дать две почти одинаковые 2D точки, которые после
            # conformal lift закономерно становятся одной вершиной.
            if vert_key in used_keys:
                continue
            used_keys.add(vert_key)
            _distance, t = _segment_point_distance2(
                site.point_a, site.point_b, point
            )
            uv_point = (
                _frame_affine_point(
                    pending_face.uv_frame_transform, point
                )
                if pending_face.uv_frame_transform is not None
                else _affine_point(
                    pending_face.uv_point_transform, point
                )
                if pending_face.uv_point_transform is not None
                else point
            )
            raw_uv_t = _site_unbounded_parameter(uv_site, uv_point)
            continuation = pending_face.uv_v_continuation_endpoint
            terminal_continuation = (
                (
                    raw_uv_t < 0.0
                    and uv_site.vert_a
                    in pending_face.terminal_cut_vertices
                )
                or (
                    raw_uv_t > 1.0
                    and uv_site.vert_b
                    in pending_face.terminal_cut_vertices
                )
            )
            if (
                terminal_continuation
                or continuation < 0
                and raw_uv_t < 0.0
                or continuation > 0
                and raw_uv_t > 1.0
            ):
                uv_t = raw_uv_t
            else:
                _uv_distance, uv_t = _segment_point_distance2(
                    uv_site.point_a, uv_site.point_b, uv_point
                )
            vert_keys.append(vert_key)
            positions.append(position)
            fallback_face_id = (
                uv_site.owner_face_index
                if uv_site.owner_face_index >= 0
                else surface.patch_id
            )
            vertex_provenance.append(
                PatchVoronoiVertexProvenance(
                    source_face_id=_corner_point_source_face_id(
                        surface, resolved.location, fallback_face_id
                    ),
                    source_edge_id=int(uv_site.edge_index),
                    route_id=_corner_route_id(surface, uv_site),
                    station_key=_corner_station_key(
                        surface, uv_site, uv_point
                    ),
                    domain_location=resolved.location,
                )
            )
            canonical_uv = transition_uv.get(vert_key)
            if canonical_uv is not None:
                u_fracs.append(canonical_uv[0])
                v_lengths.append(canonical_uv[1])
                continue
            component_uv = _crop_component_uv(crop, uv_point)
            if component_uv is None:
                u_fracs.append(
                    _site_lateral_u(uv_site, uv_point, alpha)
                )
                v_lengths.append(_site_v_length(uv_site, uv_t))
            else:
                u_fracs.append(component_uv[0])
                v_lengths.append(component_uv[1])
        if len(vert_keys) < 3:
            continue
        face_identity = frozenset(vert_keys)
        if face_identity in emitted_faces:
            continue
        emitted_faces.add(face_identity)
        surface_normal = _polygon_domain_normal(
            surface.domain, component
        )
        winding_normal = sum(
            (
                positions[index].cross(positions[(index + 1) % len(positions)])
                for index in range(len(positions))
            ),
            Vector((0.0, 0.0, 0.0)),
        )
        if winding_normal.dot(surface_normal) < 0.0:
            vert_keys.reverse()
            positions.reverse()
            u_fracs.reverse()
            v_lengths.reverse()
            vertex_provenance.reverse()
        faces.append(
            _NetworkFace(
                surface_id=surface.patch_id,
                surface_normal=surface_normal,
                vert_keys=vert_keys,
                positions=positions,
                u_fracs=u_fracs,
                v_lengths=v_lengths,
                component_kind=crop.kind,
                component_side=crop.side,
                provenance=_patch_face_provenance(
                    vertex_provenance,
                    semantic_owner_id=crop.semantic_owner_id,
                ),
            )
        )
    _synchronize_cross_surface_spine_stations(plan, faces)
    if diagnostics is not None:
        diagnostics.periodic_weld_count = _count_periodic_transition_welds(
            plan, resolved_points
        )
        diagnostics.interior_weld_count = _count_atlas_transition_welds(
            plan, resolved_points
        )
    faces.extend(
        _junction_connector_faces(
            plan,
            faces,
            alpha,
            corner_settings,
            diagnostics,
        )
    )
    return _merge_terminal_partition_faces(faces, diagnostics)
