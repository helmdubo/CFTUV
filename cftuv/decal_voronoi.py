"""Patch-bounded segment-Voronoi backend для Decal Seams.

Диаграмма строится один раз на owner patch по выбранным boundary segments.
Во время modal drag segment- и endpoint-cells обрезаются динамическими
extrusion polygons текущей ширины. Ячейки заранее пересечены с boundary-доменом
patch, поэтому крылья не выходят за mesh boundary, не пересекаются внутри patch
и меняют топологию в тот же момент, когда фронты встречаются. Внутренняя
триангуляция домена используется только для clipping и сваривается обратно:
topology исходного mesh не должна отпечатываться на итоговой декали.

``pyvoronoi`` изолирован в этом модуле. Если wheel недоступен или patch не
планарен, вызывающий код явно возвращается к legacy seam-network backend.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from heapq import heappop, heappush
from math import atan2, ceil, isfinite, pi, sqrt, tau

from mathutils import Vector

try:
    from mathutils.geometry import tessellate_polygon as _tessellate_polygon
except ImportError:  # Unit tests используют минимальный mathutils stub.
    _tessellate_polygon = None

from .constants import DECAL_WELD_DISTANCE
from .decal_network import (
    _NetworkFace,
    _lift_position,
    _polygon_area2,
    _segment_point_distance2,
)

try:
    import pyvoronoi
except ImportError:  # Blender может открыть старый файл без установленного wheel.
    pyvoronoi = None


_GEOMETRY_EPS = 1e-9
_DIAGRAM_INT_LIMIT = (1 << 31) - 1
_DIAGRAM_INT_SAFETY_MARGIN = 0.8
_DIAGRAM_RELATIVE_QUANTUM = 1e-4
_ANGLE_CLASSIFICATION_EPS = 1e-5
_MITER_ANGLE = 2.0 * pi / 3.0
_KITE_ANGLE = pi / 2.0
_SPLIT_ANGLE = pi / 3.0
_HAIRPIN_ANGLE = pi / 6.0
_MITER_LIMIT = 8.0


class _CornerPolicy(str, Enum):
    """Intrinsic corner policy; не зависит от способа lift на owner mesh."""

    CAP = "CAP"
    MITER = "MITER"
    KITE = "KITE"
    FAN = "FAN"
    ACUTE_SPLIT = "ACUTE_SPLIT"
    HAIRPIN = "HAIRPIN"
    BEVEL = "BEVEL"
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


@dataclass(frozen=True)
class DiagramTransform:
    """Детерминированная граница chart-space -> Boost integer space."""

    center: tuple[float, float]
    scale: float
    inv_scale: float
    quantum: float
    max_abs_input: float
    int_safety_margin: float

    def quantize(self, point):
        return tuple(
            self.center[axis]
            + round(
                (float(point[axis]) - self.center[axis]) / self.quantum
            )
            * self.quantum
            for axis in range(2)
        )

    def to_diagram(self, point):
        quantized = self.quantize(point)
        return (
            quantized[0] - self.center[0],
            quantized[1] - self.center[1],
        )

    def from_diagram(self, point):
        return (
            float(point[0]) + self.center[0],
            float(point[1]) + self.center[1],
        )


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


@dataclass(frozen=True, init=False)
class CornerRuntimeSettings:
    """Дешёвые corner-настройки, допустимые к изменению во время drag."""

    miter_angle: float
    kite_angle: float
    split_angle: float
    hairpin_angle: float
    apex_limit: float
    dynamic_corner_bands: bool

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
        dynamic_corner_bands=True,
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
            getattr(settings, "dynamic_corner_bands", True)
        ),
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
    )


@dataclass(frozen=True)
class _PatchVoronoiAtom:
    site_index: int
    fragments: tuple[tuple[tuple[float, float], ...], ...]
    cell_kind: str
    corner_index: int = -1
    source_category: int = 0


@dataclass(frozen=True)
class _IntrinsicDomainTriangle:
    """Один triangle intrinsic chart и его lift-данные на owner mesh."""

    chart_points: tuple[tuple[float, float], ...]
    positions: tuple[Vector, ...]
    normals: tuple[Vector, ...]


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
    intrinsic_triangles: tuple[_IntrinsicDomainTriangle, ...] = ()
    periodic_axis: str = ""

    def project(self, position):
        if self.kind != "PLANAR":
            raise ValueError(
                "Intrinsic domain projection requires triangle provenance"
            )
        delta = position - self.origin
        return (delta.dot(self.basis_u), delta.dot(self.basis_v))

    def _intrinsic_location(self, point):
        best = None
        best_margin = -float("inf")
        for triangle in self.intrinsic_triangles:
            weights = _triangle_weights2(point, triangle.chart_points)
            if weights is None:
                continue
            margin = min(weights)
            if margin >= -1e-7 and margin > best_margin:
                best = (triangle, weights)
                best_margin = margin
        return best

    def normal_at(self, point):
        if self.kind == "PLANAR":
            return self.reference_normal.copy()
        location = self._intrinsic_location(point)
        if location is None:
            raise ValueError("Point lies outside intrinsic decal domain")
        triangle, weights = location
        normal = sum(
            (
                triangle.normals[index] * weights[index]
                for index in range(3)
            ),
            Vector((0.0, 0.0, 0.0)),
        )
        if normal.length_squared <= 1e-12:
            return self.reference_normal.copy()
        return normal.normalized()

    def lift(self, point, offset):
        if self.kind == "PLANAR":
            return (
                self.origin
                + self.basis_u * point[0]
                + self.basis_v * point[1]
                + self.reference_normal * offset
            )
        location = self._intrinsic_location(point)
        if location is None:
            raise ValueError("Point lies outside intrinsic decal domain")
        triangle, weights = location
        position = sum(
            (
                triangle.positions[index] * weights[index]
                for index in range(3)
            ),
            Vector((0.0, 0.0, 0.0)),
        )
        return position + self.normal_at(point) * offset


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
    не причина откатывать весь selection в legacy: его owner faces делятся
    на независимые planar surfaces, а junction layer соединяет их rails.
    """

    patch_id: int
    centroid: Vector
    normal: Vector
    basis_u: Vector
    basis_v: Vector
    boundary_loops: tuple
    mesh_verts: tuple
    mesh_tris: tuple


@dataclass(frozen=True)
class PatchVoronoiPlan:
    """Width-independent segment-Voronoi diagrams одного modal invoke."""

    offset: float
    surfaces: tuple[_PatchVoronoiSurface, ...]
    lifted_vertices: dict[int, Vector]
    max_lateral_lift_ratio: float


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
    runtime_policy_counts: dict[str, int] = field(default_factory=dict)

    def record_runtime_policy(self, policy):
        key = str(getattr(policy, "value", policy))
        self.runtime_policy_counts[key] = (
            self.runtime_policy_counts.get(key, 0) + 1
        )

    def as_dict(self):
        return {
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
            "runtime_policy_counts": dict(
                sorted(self.runtime_policy_counts.items())
            ),
        }


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


@dataclass(frozen=True)
class DecalArrangement:
    """Математическая decal-сеть до preview/final BMesh adapters."""

    faces: tuple[_DecalArrangementFace, ...]
    inserted_stations: int


@dataclass(frozen=True)
class _CropComponent:
    """Один convex runtime crop с семантикой, живущей до NetworkFace."""

    kind: str
    side: str
    points: tuple[tuple[float, float], ...]
    uv_anchors: tuple[tuple[float, float], ...] = ()
    v_origin: float = 0.0
    owner_site_indices: tuple[int, ...] = ()


@dataclass(frozen=True)
class _PendingArrangementFace:
    surface: _PatchVoronoiSurface
    site: _PatchVoronoiSite
    points: tuple[tuple[float, float], ...]
    crop: _CropComponent


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


def _merge_polygon_fragments(fragments, tolerance=1e-7, diagnostics=None):
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
    normalized_keys = []
    for fragment in fragments:
        polygon = _dedupe_polygon(fragment, tolerance=tolerance)
        if len(polygon) < 3:
            continue
        area = _polygon_area2(polygon)
        if abs(area) <= tolerance * tolerance:
            continue
        if area < 0.0:
            polygon.reverse()
        keys = tuple(point_key(point) for point in polygon)
        normalized.append(polygon)
        normalized_keys.append(keys)
    if len(normalized) <= 1:
        return normalized

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
            previous_owner = edge_owners.get(undirected)
            if previous_owner is None:
                edge_owners[undirected] = fragment_index
            else:
                union(fragment_index, previous_owner)

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
        if (
            not loops
            or any(_polygon_area2(loop) < 0.0 for loop in loops)
            or any(not _polygon_is_simple(loop, tolerance) for loop in loops)
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


def _point_in_domain(point, triangles):
    return any(_point_in_triangle(point, triangle) for triangle in triangles)


def _inward_site_normal(point_a, point_b, triangles, probe_distance):
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
    left_inside = _point_in_domain(left_probe, triangles)
    right_inside = _point_in_domain(right_probe, triangles)
    if left_inside != right_inside:
        return left if left_inside else (-left[0], -left[1])

    # На очень коротком edge probe может пересечь соседний угол. Ближайшая
    # owner triangle даёт устойчивый fallback без зависимости от winding.
    best_centroid = None
    best_distance = float("inf")
    for triangle in triangles:
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


def _site_lateral_u(site, point, alpha):
    """Signed U для same-chart two-sided site, legacy parity для остальных."""

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


def _corner_offset_lines(surface, corner, alpha):
    """Две ordered offset-линии intrinsic corner."""

    offset_lines = []
    for site_index in corner.ordered_sites:
        site = surface.sites[site_index]
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
        site = surface.sites[site_index]
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


def _cap_crop_components(surface, corner, alpha):
    """FLAT CAP: tangent-aligned terminal half-quad, без axis square."""

    if len(corner.incident_sites) != 1:
        return ()
    site_index = corner.incident_sites[0]
    site = surface.sites[site_index]
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
        site = surface.sites[site_index]
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
    surface, corner, policy, alpha, settings, diagnostics=None
):
    settings = _normalized_corner_runtime_settings(settings)
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
    if policy == _CornerPolicy.CAP:
        return _cap_crop_components(surface, corner, alpha)
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
        site = surface.sites[site_index]
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
        site = surface.sites[site_index]
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
                # собранные triangles и включать Legacy для всего component.
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

    points = tuple(
        (float(point[0]), float(point[1])) for point in points
    )
    if not points or any(
        not all(isfinite(value) for value in point) for point in points
    ):
        raise _PatchVoronoiSurfaceCompileError(
            "DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED",
            edge_indices,
            "diagram input is empty or non-finite",
        )
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    extent_x = max_x - min_x
    extent_y = max_y - min_y
    diagonal = sqrt(extent_x * extent_x + extent_y * extent_y)
    if not isfinite(diagonal) or diagonal <= 0.0:
        raise _PatchVoronoiSurfaceCompileError(
            "DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED",
            edge_indices,
            f"degenerate chart extent={diagonal!r}",
        )
    center = ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)
    desired_quantum = min(
        DECAL_WELD_DISTANCE * 0.1,
        diagonal * _DIAGRAM_RELATIVE_QUANTUM,
    )
    if not isfinite(desired_quantum) or desired_quantum <= 0.0:
        raise _PatchVoronoiSurfaceCompileError(
            "DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED",
            edge_indices,
            f"invalid desired quantum={desired_quantum!r}",
        )

    # Guard frame на один chart diagonal шире domain. Этого достаточно,
    # чтобы закрыть все cells, которые затем всё равно clip'ятся domain'ом.
    guard_margin = max(diagonal, desired_quantum * 32.0)
    max_abs_input = max(
        abs(min_x - guard_margin - center[0]),
        abs(max_x + guard_margin - center[0]),
        abs(min_y - guard_margin - center[1]),
        abs(max_y + guard_margin - center[1]),
    )
    safe_integer = _DIAGRAM_INT_LIMIT * _DIAGRAM_INT_SAFETY_MARGIN
    maximum_scale = min(safe_integer / max_abs_input, safe_integer)
    required_scale = float(ceil(1.0 / desired_quantum))
    preferred_scale = float(ceil(10.0 / desired_quantum))
    if (
        not isfinite(maximum_scale)
        or not isfinite(required_scale)
        or required_scale < 1.0
        or required_scale > maximum_scale
    ):
        raise _PatchVoronoiSurfaceCompileError(
            "DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED",
            edge_indices,
            (
                f"extent={diagonal:.12g} desired_quantum="
                f"{desired_quantum:.12g} required_scale="
                f"{required_scale:.12g} maximum_scale="
                f"{maximum_scale:.12g} max_abs_input="
                f"{max_abs_input:.12g}"
            ),
        )
    scale = float(int(min(preferred_scale, maximum_scale)))
    inv_scale = 1.0 / scale
    quantum_steps = max(1, int(desired_quantum * scale))
    quantum = quantum_steps * inv_scale
    return DiagramTransform(
        center=center,
        scale=scale,
        inv_scale=inv_scale,
        quantum=quantum,
        max_abs_input=max_abs_input,
        int_safety_margin=_DIAGRAM_INT_SAFETY_MARGIN,
    )


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
    normals_by_face = {}
    for triangle, face_index, face_normal in zip(
        node.mesh_tris,
        node.mesh_tri_face_indices,
        node.mesh_tri_face_normals,
    ):
        face_index = int(face_index)
        triangles_by_face.setdefault(face_index, []).append(tuple(triangle))
        if face_normal.length_squared > _GEOMETRY_EPS:
            normals_by_face.setdefault(face_index, face_normal.normalized())

    group_by_face = {}
    triangles_by_group = {}
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


def _compile_corners(sites):
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
        point = points[vert_index]
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
                site = sites[site_index]
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
                        _dot2(bisector, sites[site_index].inward_normal)
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
                site = sites[site_index]
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
                site = sites[site_index]
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
    sites, corner, diagram_vertices, triangles
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
        coordinates = _corner_wedge_coordinates(sites, corner, point)
        if coordinates is None or min(coordinates) < -tolerance:
            continue
        distance = _dist2(corner.point, point)
        if distance <= tolerance or not _point_in_domain(point, triangles):
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
    """Выбирает corner policy из compiled facts и текущих настроек.

    Функция не читает PyVoronoi и не меняет plan. Поэтому thresholds и
    apex limit можно менять между preview frames без перекомпиляции sites.
    """

    settings = _normalized_corner_runtime_settings(settings)
    incident_count = len(corner.incident_sites)
    if incident_count == 1:
        return _CornerPolicy.CAP
    if incident_count != 2:
        return _CornerPolicy.JUNCTION

    if abs(corner.interior_angle - pi) <= 1e-7:
        return _CornerPolicy.MITER
    if not settings.dynamic_corner_bands:
        if corner.extrusion_angle < settings.split_angle:
            return _CornerPolicy.ACUTE_SPLIT
        if corner.is_convex:
            return _CornerPolicy.MITER
        if corner.interior_angle > pi:
            return _CornerPolicy.KITE
        return _CornerPolicy.MITER
    return _classify_extrusion_angle(corner.extrusion_angle, settings)


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


def _compile_surface(node, raw_sites, diagnostics=None):
    origin = node.centroid.copy()
    normal = node.normal.normalized()
    basis_u, basis_v = _canonical_planar_basis(normal)
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
            point_a, point_b, triangles, probe_distance
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
                ),
            )
        )
    corner_points = {}
    for site in sites:
        corner_points.setdefault(site.vert_a, site.point_a)
        corner_points.setdefault(site.vert_b, site.point_b)
    corners = tuple(
        replace(corner, point=corner_points[corner.vert_index])
        for corner in _compile_corners(classification_sites)
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
    for site in sites:
        if site.point_a <= site.point_b:
            diagram_points = (site.point_a, site.point_b)
            endpoint_vertices = (site.vert_a, site.vert_b)
        else:
            diagram_points = (site.point_b, site.point_a)
            endpoint_vertices = (site.vert_b, site.vert_a)
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
            sites[index].edge_index
            for index in invalid_indices
            if 0 <= index < len(sites)
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

    # Curved segment-Voronoi bisectors are parabolas. Их прежняя частота
    # (diagonal / 48) давала визуальный fan из десятков почти избыточных
    # stations в широких collisions. Ограничиваем шаг и размером patch, и
    # характерной длиной site: контур остаётся стабильным, но low-poly.
    sorted_site_lengths = sorted(site.segment_length for site in sites)
    median_site_length = sorted_site_lengths[len(sorted_site_lengths) // 2]
    curve_step = max(
        min(diagonal / 20.0, median_site_length * 0.5),
        DECAL_WELD_DISTANCE * 2.0,
    )
    diagram_edges = diagram.GetEdges()
    diagram_vertices = tuple(
        _DiagramVertex(*diagram_transform.from_diagram((vertex.X, vertex.Y)))
        for vertex in diagram.GetVertices()
    )
    corners = tuple(
        replace(
            corner,
            split_chord=_compile_corner_split_chord(
                sites, corner, diagram_vertices, triangles
            ),
            static_wedge=_compile_corner_static_wedge(
                sites, corner, triangles
            ),
        )
        for corner in corners
    )
    atoms = []
    for cell in diagram.GetCells():
        if (
            cell.site < 0
            or cell.site >= len(sites)
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
        site = sites[cell.site]
        fragments = []
        for cell_triangle in cell_triangles:
            for domain_triangle in triangles:
                clipped = _clip_to_triangle(cell_triangle, domain_triangle)
                if (
                    len(clipped) < 3
                    or abs(_polygon_area2(clipped)) <= 1e-10
                ):
                    continue
                if _polygon_area2(clipped) < 0.0:
                    clipped.reverse()
                fragments.append(tuple(clipped))
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
                    site_index=int(cell.site),
                    fragments=tuple(fragments),
                    cell_kind=cell_kind,
                    corner_index=corner_index,
                    source_category=source_category,
                )
            )
    segment_site_indices = {
        atom.site_index for atom in atoms if atom.cell_kind == "SEGMENT"
    }
    missing_segment_sites = tuple(
        index for index in range(len(sites)) if index not in segment_site_indices
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
    return _PatchVoronoiSurface(
        patch_id=node.patch_id,
        domain=DecalSurfaceDomain(
            patch_id=node.patch_id,
            kind="PLANAR",
            origin=origin,
            reference_normal=normal,
            basis_u=basis_u,
            basis_v=basis_v,
            boundary_triangles=tuple(tuple(triangle) for triangle in triangles),
        ),
        sites=tuple(sites),
        corners=corners,
        atoms=tuple(atoms),
        diagram_transform=diagram_transform,
        site_grid_size=site_grid_size,
        site_grid={
            key: tuple(indices) for key, indices in site_grid.items()
        },
    )


def compile_patch_voronoi_attempt(
    graph,
    selected_edge_indices,
    offset,
    *,
    allow_partial=False,
    diagnostics=None,
):
    """Компилирует plan и локализует unsupported patches до physical edges.

    ``allow_partial=False`` сохраняет прежний all-or-nothing контракт.
    Диагностический partial-режим нужен hybrid router: его неполный plan не
    материализуется напрямую, потому что rejected topology component может
    потребовать исключить дополнительные соседние edges и повторный compile.
    """

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
    raw_by_patch, normals_by_vert, positions_by_vert = _collect_patch_sites(
        graph, selected_edges
    )
    if not raw_by_patch:
        return PatchVoronoiCompileAttempt(
            plan=None,
            rejected_edge_indices=tuple(sorted(selected_edges)),
            failures=(
                PatchVoronoiCompileFailure(
                    patch_id=-1,
                    reason="NO_PATCH_SITES",
                    edge_indices=tuple(sorted(selected_edges)),
                ),
            ),
        )
    surfaces = []
    rejected_edges = set()
    failures = []
    for patch_id in sorted(raw_by_patch):
        node = graph.nodes[patch_id]
        patch_sites = raw_by_patch[patch_id]
        if _patch_is_planar(node):
            owner_surfaces = ((node, patch_sites),)
        else:
            owner_surfaces = _planar_owner_surfaces(node, patch_sites)
            if not owner_surfaces:
                edge_indices = tuple(
                    sorted({int(site["edge_index"]) for site in patch_sites})
                )
                rejected_edges.update(edge_indices)
                failures.append(
                    PatchVoronoiCompileFailure(
                        patch_id=int(patch_id),
                        reason="NO_OWNER_SURFACES",
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
        for owner_surface, owner_sites in owner_surfaces:
            try:
                surface = _compile_surface(
                    owner_surface, owner_sites, diagnostics
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
            if surface is None:
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
            surfaces.append(surface)
    lifted_vertices = {
        vert_index: _lift_position(
            positions_by_vert[vert_index], normals, float(offset)
        )
        for vert_index, normals in normals_by_vert.items()
    }
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
        plan = PatchVoronoiPlan(
            offset=float(offset),
            surfaces=tuple(surfaces),
            lifted_vertices=lifted_vertices,
            max_lateral_lift_ratio=max_lateral_lift_ratio,
        )
    return PatchVoronoiCompileAttempt(
        plan=plan,
        rejected_edge_indices=tuple(sorted(rejected_edges)),
        failures=tuple(failures),
    )


def compile_patch_voronoi_plan(
    graph, selected_edge_indices, offset, *, diagnostics=None
):
    """Компилирует все touched surfaces или сохраняет legacy fallback."""

    return compile_patch_voronoi_attempt(
        graph,
        selected_edge_indices,
        offset,
        allow_partial=False,
        diagnostics=diagnostics,
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


def _build_decal_arrangement(pending, tolerance):
    """Создаёт conforming subdivision отдельно на каждом owner surface."""

    grouped = {}
    for pending_index, pending_face in enumerate(pending):
        grouped.setdefault(pending_face.surface.patch_id, []).append(
            (pending_index, pending_face)
        )

    arranged_by_index = {}
    inserted_stations = 0
    for entries in grouped.values():
        polygons, inserted = _insert_surface_edge_stations(
            [entry[1].points for entry in entries], tolerance
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
            )
    return DecalArrangement(
        faces=tuple(
            arranged_by_index[index] for index in sorted(arranged_by_index)
        ),
        inserted_stations=inserted_stations,
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
    if distance <= spine_eps:
        endpoint_eps = max(
            site.segment_length * 1e-6,
            min(DECAL_WELD_DISTANCE, site.segment_length * 0.01),
        )
        if _dist2(point, site.point_a) <= endpoint_eps:
            position = site.source_a.lerp(
                plan.lifted_vertices[site.vert_a], lift_scale
            )
            return position, ("pv-sv", site.vert_a)
        if _dist2(point, site.point_b) <= endpoint_eps:
            position = site.source_b.lerp(
                plan.lifted_vertices[site.vert_b], lift_scale
            )
            return position, ("pv-sv", site.vert_b)
        start = site.source_a.lerp(
            plan.lifted_vertices[site.vert_a], lift_scale
        )
        end = site.source_b.lerp(
            plan.lifted_vertices[site.vert_b], lift_scale
        )
        return start.lerp(end, t), ("pv-se", site.edge_index, round(t, 7))
    position = surface.domain.lift(point, effective_offset)
    quantum = max(DECAL_WELD_DISTANCE * 0.25, 1e-7)
    return position, (
        "pv",
        surface.patch_id,
        round(point[0] / quantum),
        round(point[1] / quantum),
    )


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
    position_zero, key_zero = _position_and_key(
        plan,
        surface,
        best_site,
        point,
        0.0,
        0.0,
        projection=projection,
    )
    position_full, key_full = _position_and_key(
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
    resolved = (position_zero, position_full, key_full)
    cache[cache_key] = resolved
    return resolved


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
        position_zero, position_full, key = _resolve_arrangement_point(
            plan,
            surface,
            site,
            point,
            desired_scale,
            resolved_points,
        )
        if key in used_keys:
            continue
        used_keys.add(key)
        endpoints.append(
            (position_zero, position_full - position_zero)
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
    normal = surface.domain.normal_at(centroid)
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
    """Зеркалит pv-se stations на общий source edge соседних surfaces.

    Arrangement conformal только внутри одной owner surface. Corner crop
    может добавить точку на spine edge одной поверхности, пока соседняя
    поверхность всё ещё содержит цельный pv-sv -> pv-sv edge. Без этой
    синхронизации materialization получает геометрический T-контакт и
    визуальную щель. Станция уже имеет общую 3D rail-position; здесь мы
    лишь вставляем тот же key в соседний polygon loop.
    """

    edge_by_vertices = {}
    for surface in plan.surfaces:
        for site in surface.sites:
            pair = frozenset((site.vert_a, site.vert_b))
            edge_by_vertices.setdefault(pair, site.edge_index)

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

            if (
                not isinstance(key_a, tuple)
                or not isinstance(key_b, tuple)
                or key_a[:1] != ("pv-sv",)
                or key_b[:1] != ("pv-sv",)
            ):
                continue
            edge_index = edge_by_vertices.get(
                frozenset((key_a[1], key_b[1]))
            )
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

        face.vert_keys = new_keys
        face.positions = new_positions
        face.u_fracs = new_u_fracs
        face.v_lengths = new_v_lengths


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
        surface.patch_id: surface for surface in plan.surfaces
    }
    incident_edges_by_vertex = {}
    for surface in plan.surfaces:
        for site in surface.sites:
            incident_edges_by_vertex.setdefault(site.vert_a, set()).add(
                site.edge_index
            )
            incident_edges_by_vertex.setdefault(site.vert_b, set()).add(
                site.edge_index
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
        surface = surfaces_by_id.get(face.surface_id)
        if surface is None:
            continue
        core_index = face.vert_keys.index(core_key)
        outer_index = face.vert_keys.index(outer_key)
        outer_position = face.positions[outer_index]
        outer_point = (
            (outer_position - surface.origin).dot(surface.basis_u),
            (outer_position - surface.origin).dot(surface.basis_v),
        )
        matching_sites = []
        for site in surface.sites:
            if site.vert_a == core_key[1]:
                station = site.point_a
            elif site.vert_b == core_key[1]:
                station = site.point_b
            else:
                continue
            expected_outer = (
                station[0] + site.inward_normal[0] * alpha,
                station[1] + site.inward_normal[1] * alpha,
            )
            matching_sites.append(
                (_dist2(outer_point, expected_outer), site.edge_index)
            )
        if not matching_sites:
            continue
        match_distance, matched_edge_index = min(matching_sites)
        # Boundary edge большой Voronoi-cell тоже может начинаться в core,
        # но junction port обязан лежать у локального alpha-offset cap.
        if match_distance > max(DECAL_WELD_DISTANCE, alpha * 0.05):
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
                )
            )
            used_ports.update((index, other_index))
    return connectors


def _append_pending_fragments(
    pending, surface, site, crop, fragments, diagnostics=None
):
    """Сваривает fragments одного semantic owner до materialization."""

    components = _merge_polygon_fragments(
        fragments,
        tolerance=max(1e-8, DECAL_WELD_DISTANCE * 0.25),
        diagnostics=diagnostics,
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
            )
        )


def _evaluate_surface_crops(
    surface, alpha, pending, corner_settings, diagnostics=None
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

    point_atoms_by_corner = {}
    for atom in surface.atoms:
        if atom.cell_kind == "POINT" and atom.corner_index >= 0:
            point_atoms_by_corner.setdefault(atom.corner_index, []).append(atom)

    corner_crops = {}
    # Обычный endpoint corner существует только там, где pyvoronoi дал
    # отдельную point-cell. Острый convex miter такой cell не имеет: две
    # segment-cells сходятся непосредственно по биссектрисе. Но его всё
    # равно нужно сначала собрать в один semantic corner, а затем разделить
    # на INNER/OUTER. Иначе длинный miter режется конкурентами как два
    # независимых крыла и визуально распадается при большой ширине.
    runtime_policies = tuple(
        classify_corner_runtime(corner, corner_settings)
        for corner in surface.corners
    )
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
                policy == _CornerPolicy.ACUTE_SPLIT
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
        )
        crops = tuple(
            owned_crop
            for crop in raw_crops
            for owned_crop in (
                _corner_endpoint_ownership_crop(surface, corner, crop),
            )
            if owned_crop is not None
        )
        if not crops:
            continue
        corner_crops[corner_index] = crops
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
                crops_by_site.setdefault(site_index, []).append(crop)
            owner_atoms = [
                atom
                for atom in surface.atoms
                if (
                    atom.site_index in owner_site_indices
                    and atom.cell_kind == "SEGMENT"
                )
                or (
                    atom.cell_kind == "POINT"
                    and atom.corner_index == corner_index
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
            owner_site = surface.sites[owner_site_index]
            atom_groups = (
                ((atom,) for atom in owner_atoms)
                if split_dynamic_atoms
                else (owner_atoms,)
            )
            for atoms in atom_groups:
                fragments = []
                for atom in atoms:
                    for fragment in atom.fragments:
                        clipped = _clip_to_convex(fragment, crop.points)
                        if clipped:
                            fragments.append(clipped)
                _append_pending_fragments(
                    pending,
                    surface,
                    owner_site,
                    crop,
                    fragments,
                    diagnostics,
                )

    for atom in surface.atoms:
        if atom.cell_kind == "POINT" and atom.corner_index in corner_crops:
            continue
        site = surface.sites[atom.site_index]
        crop = _CropComponent(
            kind="SEGMENT",
            side="",
            points=tuple(_segment_crop_polygon(site, alpha)),
        )
        fragments = []
        subtraction_crops = crops_by_site.get(atom.site_index, ())
        for fragment in atom.fragments:
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
        _append_pending_fragments(
            pending, surface, site, crop, fragments, diagnostics
        )


def evaluate_patch_voronoi_plan(
    plan,
    width,
    preview=False,
    corner_settings=None,
    diagnostics=None,
):
    """Перестраивает extrusion polygons внутри статических Voronoi cells."""

    corner_settings = _normalized_corner_runtime_settings(corner_settings)
    if diagnostics is not None:
        diagnostics.runtime_policy_counts.clear()
    alpha = max(1e-6, float(width) * 0.5)
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
    for surface in plan.surfaces:
        _evaluate_surface_crops(
            surface,
            alpha,
            pending,
            corner_settings,
            diagnostics,
        )

    arrangement = _build_decal_arrangement(
        pending,
        tolerance=max(1e-8, DECAL_WELD_DISTANCE * 0.5),
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
        component = pending_face.points
        crop = pending_face.crop
        vert_keys = []
        positions = []
        u_fracs = []
        v_lengths = []
        used_keys = set()
        for point in component:
            position_zero, position_full, key = _resolve_arrangement_point(
                plan,
                surface,
                site,
                point,
                desired_lift_scale,
                resolved_points,
            )
            position = position_zero.lerp(position_full, lift_fraction)
            # Triangle boundaries и pyvoronoi endpoint-cells могут
            # дать две почти одинаковые 2D точки, которые после
            # conformal lift закономерно становятся одной вершиной.
            if key in used_keys:
                continue
            used_keys.add(key)
            _distance, t = _segment_point_distance2(
                site.point_a, site.point_b, point
            )
            vert_keys.append(key)
            positions.append(position)
            component_uv = _crop_component_uv(crop, point)
            if component_uv is None:
                u_fracs.append(_site_lateral_u(site, point, alpha))
                v_lengths.append(_site_v_length(site, t))
            else:
                u_fracs.append(component_uv[0])
                v_lengths.append(component_uv[1])
        if len(vert_keys) < 3:
            continue
        face_identity = frozenset(vert_keys)
        if face_identity in emitted_faces:
            continue
        emitted_faces.add(face_identity)
        surface_normal = surface.domain.normal_at(
            (
                sum(point[0] for point in component) / len(component),
                sum(point[1] for point in component) / len(component),
            )
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
            )
        )
    _synchronize_cross_surface_spine_stations(plan, faces)
    faces.extend(
        _junction_connector_faces(
            plan,
            faces,
            alpha,
            corner_settings,
            diagnostics,
        )
    )
    return faces
