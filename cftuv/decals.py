"""Strict compiled SEAMS producer и общие materialization adapters.

Потребитель PatchGraph не читает исходный BMesh. Архивированные
TOP/BOTTOM/CORNERS остаются распознаваемыми capability names, но их direct
builders физически отсутствуют и runtime всегда завершается именованным
отказом до analysis/materialization.
"""

from dataclasses import dataclass
from enum import Enum
from time import perf_counter

import bpy
import bmesh
from mathutils import Vector

from .constants import (
    DECAL_COLLECTION_NAME,
    DECAL_UV_RECT_SEAM,
    DECAL_WELD_DISTANCE,
    WORLD_UP,
)
from .decal_geometry import GeometryBatch, geometry_batch_from_faces
from .decal_rails import (
    DecalRailPlan,
    RailTerminalKind,
    RailTermination,
    compile_decal_rail_attempt,
)
from .decal_rail_geometry import (
    PlanarRailGeometryPlan,
    RailGeometryFailure,
    compile_planar_rail_geometry_attempt,
    evaluate_planar_rail_geometry_plan,
)
from .decal_voronoi import (
    PatchVoronoiDiagnostics,
    PatchVoronoiPlan,
    compile_patch_voronoi_attempt,
    compile_patch_voronoi_plan,
    corner_runtime_settings_from_decal_settings,
    evaluate_patch_voronoi_plan,
    require_decal_corner_join_available,
)
from .decal_transform import (
    DecalSourceTransformError,
    local_decal_settings_for_source,
)
from .model import ChainRef, DecalSettings, PatchGraph
from .surface_ir import (
    AnalysisBundle,
    AnalysisSchemaError,
    CapacityPolicy,
    DecalBackendKind,
    PreviewFailurePolicy,
)

DECAL_MODES = ("TOP", "BOTTOM", "CORNERS", "SEAMS")
SUPPORTED_DECAL_MODES = ("SEAMS",)
ARCHIVED_DECAL_MODE_REASONS = {
    "TOP": "DECAL_TOP_ARCHIVED_UNTIL_ENGINE_PLAN",
    "BOTTOM": "DECAL_BOTTOM_ARCHIVED_UNTIL_ENGINE_PLAN",
    "CORNERS": "DECAL_CORNERS_ARCHIVED_UNTIL_ENGINE_PLAN",
}


class DecalModeArchivedError(RuntimeError):
    """Запрошен producer, архивированный до mode-specific engine plan."""

    def __init__(self, mode):
        self.mode = str(mode).upper()
        self.reason = ARCHIVED_DECAL_MODE_REASONS[self.mode]
        super().__init__(self.reason)


def decal_mode_archived_reason(mode):
    """Возвращает стабильную capability reason без изменения mode."""

    return ARCHIVED_DECAL_MODE_REASONS.get(str(mode).upper(), "")


def require_decal_mode_available(mode):
    """Fail-fast для архивированных TOP/BOTTOM/CORNERS producers."""

    normalized = str(mode).upper()
    if normalized in ARCHIVED_DECAL_MODE_REASONS:
        raise DecalModeArchivedError(normalized)

_DECAL_PREVIEW_OBJECT_PREFIX = ".CFTUV_Preview"
_DECAL_PREVIEW_MARKER = "cftuv_decal_preview"
_DECAL_PREVIEW_SOURCE = "cftuv_decal_source"
_DECAL_COMPILE_ALPHA_HEADROOM = 4.0


def decal_compile_alpha_budget(settings):
    """Начальный modal width получает явный запас intrinsic support."""

    initial_alpha = max(1e-6, float(settings.width_seam) * 0.5)
    return max(
        initial_alpha * _DECAL_COMPILE_ALPHA_HEADROOM,
        initial_alpha + DECAL_WELD_DISTANCE * 8.0,
    )

_MODE_OBJECT_SUFFIX = {
    "TOP": "Top",
    "BOTTOM": "Bottom",
    "CORNERS": "Corners",
    "SEAMS": "Seams",
}


@dataclass
class DecalPreviewState:
    """Modal-lifetime topology cache; не хранится глобально или в PatchGraph."""

    topology_signature: tuple = ()
    canonical_mesh_indices: tuple[int, ...] = ()
    object_name: str = ""
    object_pointer: int = 0
    mesh_pointer: int = 0
    fast_updates: int = 0
    topology_rebuilds: int = 0


class PreviewStatus(str, Enum):
    UPDATED = "UPDATED"
    # Compatibility value for old callers. Active adapters implement the
    # sole PreviewFailurePolicy.CLEAR and never publish this status.
    RETAINED_LAST_VALID = "RETAINED_LAST_VALID"
    EMPTY = "EMPTY"
    ERROR = "ERROR"


@dataclass(frozen=True)
class DecalGenerationResult:
    """Structured internal result generation/preview transaction."""

    status: PreviewStatus
    object_name: str | None
    reason: str = ""
    topology_changed: bool = False
    backend_summary: str = ""
    policy_counts: tuple[tuple[str, int], ...] = ()
    evaluation_ms: float = 0.0


@dataclass
class _OrientedCornerRun:
    """Непрерывный двухсторонний seam-run с фреймами на каждый сегмент."""

    vert_indices: list[int]
    points: list[Vector]
    segment_normals_a: list[Vector]
    segment_normals_b: list[Vector]
    segment_convexities: list[float]
    segment_edge_indices: list[int]

    @property
    def start_vert_index(self) -> int:
        return self.vert_indices[0]

    @property
    def end_vert_index(self) -> int:
        return self.vert_indices[-1]

    @property
    def is_closed(self) -> bool:
        return (
            len(self.vert_indices) > 2
            and self.start_vert_index == self.end_vert_index
        )

    def reversed_copy(self):
        # dihedral convexity — геометрический инвариант фолда: не зависит ни
        # от направления обхода, ни от порядка owner-sides. Негация при
        # reverse/swap выворачивает стороны после stitch-разворота.
        return _OrientedCornerRun(
            vert_indices=list(reversed(self.vert_indices)),
            points=list(reversed(self.points)),
            segment_normals_a=list(reversed(self.segment_normals_a)),
            segment_normals_b=list(reversed(self.segment_normals_b)),
            segment_convexities=list(reversed(self.segment_convexities)),
            segment_edge_indices=list(reversed(self.segment_edge_indices)),
        )

    def swapped_copy(self):
        return _OrientedCornerRun(
            vert_indices=list(self.vert_indices),
            points=list(self.points),
            segment_normals_a=list(self.segment_normals_b),
            segment_normals_b=list(self.segment_normals_a),
            segment_convexities=list(self.segment_convexities),
            segment_edge_indices=list(self.segment_edge_indices),
        )


@dataclass(frozen=True)
class _ManualSeamBackendPartition:
    """Независимая routing-группа строгого современного backend."""

    backend: DecalBackendKind
    edge_indices: tuple[int, ...]
    topology_component_count: int
    corner_runs: tuple
    boundary_runs: tuple
    compiled_plan: PlanarRailGeometryPlan | PatchVoronoiPlan | None = None

    def __post_init__(self):
        try:
            backend = DecalBackendKind(self.backend)
        except ValueError as exc:
            raise ValueError(
                "Legacy/unknown SEAMS backend is disabled: "
                f"{self.backend}"
            ) from exc
        object.__setattr__(self, "backend", backend)
        if self.compiled_plan is None:
            raise ValueError(f"{backend.value} partition requires compiled plan")


@dataclass(frozen=True)
class _ManualBackendEvaluation:
    """Faces и evaluator-owned diagnostics одной routing partition."""

    geometry_batch: GeometryBatch
    evaluation_ms: float
    policy_counts: tuple[tuple[str, int], ...] = ()

    @property
    def faces(self):
        return self.geometry_batch.faces


@dataclass(frozen=True, order=True)
class ManualSeamTerminalRouting:
    """RM9-диагностика одного торца без повторной деривации выбора."""

    component_index: int
    patch_id: int
    spine_vertex_id: int
    spine_edge_id: int
    source_face_ids: tuple[int, ...]
    backend: DecalBackendKind
    backend_kind: str
    choice: str
    edge_ids: tuple[int, ...] = ()
    plan_kind: str = ""
    route_id: int | None = None
    capacity_policy: CapacityPolicy = CapacityPolicy.SATURATE_PROVEN

    def __post_init__(self):
        object.__setattr__(self, "backend", DecalBackendKind(self.backend))
        object.__setattr__(
            self,
            "capacity_policy",
            CapacityPolicy(self.capacity_policy),
        )

    @property
    def report_line(self):
        edge_suffix = (
            " " + ",".join(str(edge_id) for edge_id in self.edge_ids)
            if self.edge_ids
            else ""
        )
        backend_value = self.backend.value
        backend = (
            backend_value
            if self.backend_kind == backend_value
            else f"{backend_value}/{self.backend_kind}"
        )
        return (
            f"component={self.component_index} "
            f"patch={self.patch_id} "
            f"terminal=v{self.spine_vertex_id}/e{self.spine_edge_id} "
            f"faces={','.join(str(face_id) for face_id in self.source_face_ids)} "
            f"choice={self.choice}{edge_suffix} backend={backend} "
            f"plan={self.plan_kind} capacity={self.capacity_policy.value}"
        )


@dataclass(frozen=True)
class ManualSeamEdgeRejection:
    """Selected physical edge, который не имеет допустимого decal use."""

    edge_index: int
    reason: str
    use_count: int = 0
    use_refs: tuple = ()


@dataclass(frozen=True)
class _ManualEdgeDecalCollection:
    """Runs и полный accounting исходного selected-edge scope."""

    corner_runs: tuple
    boundary_runs: tuple
    accepted_edge_indices: tuple[int, ...]
    rejected_edges: tuple[ManualSeamEdgeRejection, ...]

    def __iter__(self):
        # Сохраняет старый private unpacking contract для geometry callers.
        yield list(self.corner_runs)
        yield list(self.boundary_runs)


@dataclass(frozen=True)
class ManualSeamDecalPlan:
    """Width-independent manual SEAMS scope для одного modal drag."""

    corner_runs: tuple
    boundary_runs: tuple
    network_plan: object = None
    patch_voronoi_plan: PatchVoronoiPlan | None = None
    backend_partitions: tuple[_ManualSeamBackendPartition, ...] = ()
    compile_failures: tuple = ()
    selected_edge_indices: tuple[int, ...] = ()
    rejected_edges: tuple[ManualSeamEdgeRejection, ...] = ()
    direct_legacy_edge_indices: tuple[int, ...] = ()
    # Канонический rail IR общий для R0-overlay и принятого R1 PLANAR backend.
    rail_plan: DecalRailPlan | None = None
    rail_compile_failures: tuple = ()
    rail_geometry_failures: tuple = ()
    terminal_routing: tuple[ManualSeamTerminalRouting, ...] = ()

    def __post_init__(self):
        if self.network_plan is not None or self.direct_legacy_edge_indices:
            raise ValueError(
                "Legacy SEAMS plans are disabled; compile a strict rail/chart plan"
            )
        invalid_backends = tuple(
            str(getattr(partition, "backend", "UNKNOWN"))
            for partition in self.backend_partitions
            if getattr(partition, "backend", None)
            not in {DecalBackendKind.RAIL_PLANAR, DecalBackendKind.PATCH_VORONOI}
        )
        if invalid_backends:
            raise ValueError(
                "Legacy/unknown SEAMS partitions are disabled: "
                + ",".join(invalid_backends)
            )

    @property
    def rejected_edge_indices(self):
        return tuple(rejection.edge_index for rejection in self.rejected_edges)

    @property
    def accepted_patch_voronoi_edge_indices(self):
        return tuple(
            sorted(
                edge_index
                for partition in self.backend_partitions
                if partition.backend == DecalBackendKind.PATCH_VORONOI
                for edge_index in partition.edge_indices
            )
        )

    @property
    def accepted_rail_planar_edge_indices(self):
        return tuple(
            sorted(
                edge_index
                for partition in self.backend_partitions
                if partition.backend == DecalBackendKind.RAIL_PLANAR
                for edge_index in partition.edge_indices
            )
        )

    @property
    def accepted_legacy_edge_indices(self):
        """Compatibility view: production routing больше не принимает legacy."""

        return ()

    @property
    def accounting_is_exact(self):
        selected = set(self.selected_edge_indices)
        rail = set(self.accepted_rail_planar_edge_indices)
        patch = set(self.accepted_patch_voronoi_edge_indices)
        rejected = set(self.rejected_edge_indices)
        return (
            not rail.intersection(patch)
            and not rail.intersection(rejected)
            and not patch.intersection(rejected)
            and selected == rail.union(patch, rejected)
        )

    @property
    def supports_live_corner_controls(self):
        """A/M применимы только ко всему чистому Patch Voronoi scope."""

        selected = set(self.selected_edge_indices)
        accepted = set(self.accepted_patch_voronoi_edge_indices)
        return bool(selected) and selected == accepted and not self.rejected_edges

    @property
    def backend_summary(self):
        if (
            not self.backend_partitions
            and not self.rejected_edges
            and not self.compile_failures
            and not self.rail_geometry_failures
        ):
            return ""
        counts = {}
        for partition in self.backend_partitions:
            if partition.backend == DecalBackendKind.RAIL_PLANAR:
                backend_label = "RAIL_PLANAR"
            elif partition.backend == DecalBackendKind.PATCH_VORONOI:
                backend_label = getattr(
                    partition.compiled_plan, "backend_kind", "PLANAR"
                )
            else:
                raise AssertionError(
                    f"Unsupported strict SEAMS backend: {partition.backend}"
                )
            bucket = counts.setdefault(backend_label, [0, 0])
            bucket[0] += int(partition.topology_component_count)
            bucket[1] += len(partition.edge_indices)
        order = (
            "RAIL_PLANAR",
            "PLANAR",
            "INTRINSIC_DEVELOPABLE",
            "PLANAR+INTRINSIC_DEVELOPABLE",
        )
        parts = [
            f"{label}:{counts[label][0]}c/{counts[label][1]}e"
            for label in order
            if label in counts
        ]
        failure_counts = {}
        for failure in self.compile_failures:
            reason = str(getattr(failure, "reason", "UNKNOWN"))
            failure_counts[reason] = failure_counts.get(reason, 0) + 1
        if failure_counts:
            parts.append(
                "Unsupported["
                + ",".join(
                    f"{reason}:x{failure_counts[reason]}"
                    for reason in sorted(failure_counts)
                )
                + "]"
            )
        rail_failure_counts = {}
        for failure in self.rail_geometry_failures:
            reason = str(getattr(failure, "reason", "UNKNOWN"))
            rail_failure_counts[reason] = rail_failure_counts.get(reason, 0) + 1
        if rail_failure_counts:
            parts.append(
                "RailUnsupported["
                + ",".join(
                    f"{reason}:x{rail_failure_counts[reason]}"
                    for reason in sorted(rail_failure_counts)
                )
                + "]"
            )
        if self.rejected_edges:
            rejected_reason_counts = {}
            for rejection in self.rejected_edges:
                reason = str(rejection.reason or "UNKNOWN")
                rejected_reason_counts[reason] = (
                    rejected_reason_counts.get(reason, 0) + 1
                )
            parts.append(
                f"Failed:{len(self.rejected_edges)}e["
                + ",".join(
                    f"{reason}:x{rejected_reason_counts[reason]}"
                    for reason in sorted(rejected_reason_counts)
                )
                + "]"
            )
        approximate_count = sum(
            int(
                getattr(
                    partition.compiled_plan,
                    "approximate_admit_count",
                    0,
                )
            )
            for partition in self.backend_partitions
            if partition.backend == DecalBackendKind.PATCH_VORONOI
        )
        if approximate_count:
            parts.append(f"Approx:{approximate_count}c")
        return " | ".join(parts)

    @property
    def terminal_routing_report(self):
        return tuple(record.report_line for record in self.terminal_routing)


class PatchVoronoiRuntimeError(RuntimeError):
    """Compiled Patch Voronoi plan не имеет права молча сменить backend."""

    def __init__(self, edge_indices, width, preview, reason):
        self.edge_indices = tuple(int(index) for index in edge_indices)
        self.width = float(width)
        self.preview = bool(preview)
        self.reason = str(reason)
        super().__init__(
            "Patch Voronoi runtime failed for "
            f"{len(self.edge_indices)} edge(s) at width={self.width:.6g} "
            f"preview={self.preview}: {self.reason}"
        )


class RailPlanarRuntimeError(RuntimeError):
    """Compiled rail-plan обязан материализоваться без runtime fallback."""

    def __init__(self, edge_indices, width, preview, reason):
        self.edge_indices = tuple(int(index) for index in edge_indices)
        self.width = float(width)
        self.preview = bool(preview)
        self.reason = str(reason)
        super().__init__(
            "PLANAR rail runtime failed for "
            f"{len(self.edge_indices)} edge(s) at width={self.width:.6g} "
            f"preview={self.preview}: {self.reason}"
        )


class StrictSeamRuntimeError(RuntimeError):
    """Manual/automatic SEAMS не имеет права материализовать legacy."""

    def __init__(self, edge_indices, reason):
        self.edge_indices = tuple(
            sorted({int(index) for index in edge_indices or ()})
        )
        self.reason = str(reason)
        shown_edges = ",".join(str(index) for index in self.edge_indices[:12])
        if len(self.edge_indices) > 12:
            shown_edges += ",..."
        super().__init__(
            "Strict SEAMS runtime failed for "
            f"{len(self.edge_indices)} edge(s)"
            + (f" [{shown_edges}]" if shown_edges else "")
            + f": {self.reason}"
        )


@dataclass(frozen=True)
class DecalMaterializationResult:
    """Полный успешный результат одной network-face partition."""

    backend: str
    edge_indices: tuple[int, ...]
    source_face_count: int
    created_face_count: int
    created_vertex_count: int
    policy_counts: tuple[tuple[str, int], ...] = ()
    evaluation_ms: float = 0.0


class DecalMaterializationError(RuntimeError):
    """Fail-fast отказ network face до публикации temporary BMesh."""

    def __init__(
        self,
        *,
        backend,
        edge_indices,
        face_index,
        reason,
        vertex_count,
        repeated_keys=(),
        cycle_keys=(),
        cycle_positions=(),
        repeated_occurrences=(),
        component_kind="",
        component_side="",
    ):
        self.backend = str(backend or "UNKNOWN")
        self.edge_indices = tuple(
            sorted({int(index) for index in edge_indices or ()})
        )
        self.face_index = int(face_index)
        self.reason = str(reason)
        self.vertex_count = int(vertex_count)
        self.repeated_keys = tuple(repeated_keys)
        self.cycle_keys = tuple(cycle_keys)
        self.cycle_positions = tuple(cycle_positions)
        self.repeated_occurrences = tuple(repeated_occurrences)
        self.component_kind = str(component_kind or "")
        self.component_side = str(component_side or "")
        details = (
            f"backend={self.backend} edges={self.edge_indices} "
            f"face={self.face_index} verts={self.vertex_count} "
            f"component={self.component_kind}/{self.component_side or '-'} "
            f"reason={self.reason}"
        )
        if self.repeated_keys:
            details += f" repeated_keys={self.repeated_keys!r}"
        if self.repeated_occurrences:
            details += (
                f" repeated_occurrences={self.repeated_occurrences!r}"
                f" cycle={tuple(zip(self.cycle_keys, self.cycle_positions))!r}"
            )
        super().__init__(f"Decal materialization failed: {details}")


def _join_corner_runs(first, second):
    """Соединяет corner runs и сохраняет непрерывную идентичность сторон."""

    if first.end_vert_index != second.start_vert_index:
        raise ValueError("Corner runs do not share an oriented endpoint")
    same_score = (
        first.segment_normals_a[-1].dot(second.segment_normals_a[0])
        + first.segment_normals_b[-1].dot(second.segment_normals_b[0])
    )
    swapped_score = (
        first.segment_normals_a[-1].dot(second.segment_normals_b[0])
        + first.segment_normals_b[-1].dot(second.segment_normals_a[0])
    )
    if swapped_score > same_score:
        second = second.swapped_copy()
    return _OrientedCornerRun(
        vert_indices=first.vert_indices + second.vert_indices[1:],
        points=first.points + second.points[1:],
        segment_normals_a=first.segment_normals_a + second.segment_normals_a,
        segment_normals_b=first.segment_normals_b + second.segment_normals_b,
        segment_convexities=(
            first.segment_convexities + second.segment_convexities
        ),
        segment_edge_indices=(
            first.segment_edge_indices + second.segment_edge_indices
        ),
    )


# ============================================================
# Сбор кандидатов из PatchGraph (чистая геометрия, без bpy)
# ============================================================


def _chain_segment_surface_frame(node, chain, segment_index):
    """Local owner normal/up; semantic surface data has no fallback."""

    if segment_index >= len(chain.side_face_normals):
        raise ValueError(
            "DECAL_SEGMENT_SURFACE_NORMAL_MISSING: "
            f"edge={chain.edge_indices[segment_index]}"
        )
    local_normal = chain.side_face_normals[segment_index]
    if local_normal.length_squared <= 0.0:
        raise ValueError(
            "DECAL_SEGMENT_SURFACE_NORMAL_INVALID: "
            f"edge={chain.edge_indices[segment_index]}"
        )
    normal = local_normal.normalized()

    up = WORLD_UP - normal * WORLD_UP.dot(normal)
    if up.length_squared > 1e-12:
        up = up.normalized()
    else:
        up = node.basis_v.copy()
    return normal, up


def chain_refs_for_edge_indices(
    graph: PatchGraph, edge_indices
) -> set[ChainRef]:
    """Возвращает полные PatchGraph chains, содержащие выбранные mesh edges.

    Одно физическое seam-ребро обычно представлено двумя chains — по одной со
    стороны каждого соседнего patch. Обе ссылки намеренно сохраняются: producer
    сам дедуплицирует WALL-WALL пару по owner patch id.
    """

    selected_edge_indices = {int(edge_index) for edge_index in edge_indices}
    if not selected_edge_indices:
        return set()

    chain_refs = set()
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                if selected_edge_indices.intersection(chain.edge_indices):
                    chain_refs.add((patch_id, loop_index, chain_index))
    return chain_refs


def _stitch_oriented_runs(runs, join_runs):
    """Склеивает oriented runs, не проходя неоднозначный point-contact.

    Вершина с числом инцидентных run не равным двум считается границей пути.
    Это исключает прежний жадный выбор произвольного продолжения в развилке.
    """

    if not runs:
        return []
    endpoint_map = {}
    for run_index, run in enumerate(runs):
        endpoint_map.setdefault(run.start_vert_index, []).append(run_index)
        endpoint_map.setdefault(run.end_vert_index, []).append(run_index)

    unused = set(range(len(runs)))
    stitched = []
    while unused:
        seed_index = min(unused)
        unused.remove(seed_index)
        current = runs[seed_index]

        while not current.is_closed:
            tail = current.end_vert_index
            candidates = [
                index for index in endpoint_map.get(tail, ()) if index in unused
            ]
            if len(endpoint_map.get(tail, ())) != 2 or len(candidates) != 1:
                break
            next_index = candidates[0]
            unused.remove(next_index)
            next_run = runs[next_index]
            if next_run.start_vert_index != tail:
                next_run = next_run.reversed_copy()
            current = join_runs(current, next_run)

        while not current.is_closed:
            head = current.start_vert_index
            candidates = [
                index for index in endpoint_map.get(head, ()) if index in unused
            ]
            if len(endpoint_map.get(head, ())) != 2 or len(candidates) != 1:
                break
            previous_index = candidates[0]
            unused.remove(previous_index)
            previous_run = runs[previous_index]
            if previous_run.end_vert_index != head:
                previous_run = previous_run.reversed_copy()
            current = join_runs(previous_run, current)

        stitched.append(current)
    return stitched


def _stitch_corner_runs(runs):
    return _stitch_oriented_runs(runs, _join_corner_runs)


def _manual_edge_pair_convexity(points, normal_a, normal_b):
    if len(points) < 2:
        return 0.0
    direction = points[1] - points[0]
    if direction.length_squared <= 0.0:
        return 0.0
    cross = direction.normalized().cross(normal_a)
    if cross.length_squared <= 0.0:
        return 0.0
    value = cross.normalized().dot(normal_b)
    return max(-1.0, min(1.0, value))


def _chain_segment_dihedral_pair(chain, segment_index):
    return next(
        (
            pair
            for pair in chain.dihedral_segment_pairs
            if pair.segment_index == segment_index
        ),
        None,
    )


def _collect_manual_edge_decals(graph: PatchGraph, edge_indices):
    """Selected physical edges as runs с полным per-edge accounting."""

    selected_edges = {int(edge_index) for edge_index in edge_indices or ()}
    uses_by_edge = {edge_index: [] for edge_index in selected_edges}
    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                for segment_index, edge_index in enumerate(chain.edge_indices):
                    edge_index = int(edge_index)
                    if (
                        edge_index not in selected_edges
                        or len(chain.vert_cos) < 2
                        or len(chain.vert_indices) != len(chain.vert_cos)
                    ):
                        continue
                    next_index = segment_index + 1
                    if next_index >= len(chain.vert_cos):
                        if not chain.is_closed:
                            continue
                        next_index = 0
                    normal, _up = _chain_segment_surface_frame(
                        node, chain, segment_index
                    )
                    uses_by_edge[edge_index].append(
                        (
                            (patch_id, loop_index, chain_index, segment_index),
                            [
                                int(chain.vert_indices[segment_index]),
                                int(chain.vert_indices[next_index]),
                            ],
                            [
                                chain.vert_cos[segment_index].copy(),
                                chain.vert_cos[next_index].copy(),
                            ],
                            normal,
                            _chain_segment_dihedral_pair(chain, segment_index),
                        )
                    )

    paired_segments = []
    boundary_uses = {}
    accepted_edges = set()
    rejected_edges = []
    for edge_index in sorted(selected_edges):
        uses = sorted(uses_by_edge.get(edge_index, ()), key=lambda use: use[0])
        if len(uses) == 2:
            _ref_a, vert_indices, points, normal_a, pair_a = uses[0]
            _ref_b, _other_verts, _other_points, normal_b, pair_b = uses[1]
            convexity = (
                pair_a.convexity
                if pair_a is not None and pair_b is not None
                else _manual_edge_pair_convexity(points, normal_a, normal_b)
            )
            paired_segments.append(
                _OrientedCornerRun(
                    vert_indices=vert_indices,
                    points=points,
                    segment_normals_a=[normal_a],
                    segment_normals_b=[normal_b],
                    segment_convexities=[convexity],
                    segment_edge_indices=[edge_index],
                )
            )
            accepted_edges.add(edge_index)
        elif len(uses) == 1:
            ref, vert_indices, points, normal, _pair = uses[0]
            boundary_uses.setdefault(ref[:3], []).append(
                (ref[3], edge_index, vert_indices, points, normal)
            )
            accepted_edges.add(edge_index)
        else:
            reason = (
                "NO_BOUNDARY_CHAIN_USE"
                if not uses
                else "NON_MANIFOLD_EDGE_USE"
            )
            rejected_edges.append(
                ManualSeamEdgeRejection(
                    edge_index=edge_index,
                    reason=reason,
                    use_count=len(uses),
                    use_refs=tuple(use[0] for use in uses),
                )
            )
    return _ManualEdgeDecalCollection(
        corner_runs=tuple(_stitch_corner_runs(paired_segments)),
        boundary_runs=tuple(_group_boundary_runs(boundary_uses)),
        accepted_edge_indices=tuple(sorted(accepted_edges)),
        rejected_edges=tuple(
            sorted(
                {
                    int(rejection.edge_index): rejection
                    for rejection in rejected_edges
                }.values(),
                key=lambda rejection: (
                    int(rejection.edge_index),
                    str(rejection.reason),
                ),
            )
        ),
    )


def _manual_seam_edge_components(runs, selected_edge_indices):
    """Physical selected-edge components по общим source vertices."""

    selected_edges = {
        int(edge_index) for edge_index in selected_edge_indices or ()
    }
    endpoints_by_edge = {}
    for run in runs:
        for segment_index, edge_index in enumerate(
            run.segment_edge_indices
        ):
            if segment_index + 1 >= len(run.vert_indices):
                continue
            edge_index = int(edge_index)
            if edge_index not in selected_edges:
                continue
            endpoints_by_edge.setdefault(
                edge_index,
                (
                    int(run.vert_indices[segment_index]),
                    int(run.vert_indices[segment_index + 1]),
                ),
            )

    edges_by_vertex = {}
    for edge_index, endpoints in endpoints_by_edge.items():
        for vert_index in endpoints:
            edges_by_vertex.setdefault(vert_index, []).append(edge_index)

    unseen = set(selected_edges)
    components = []
    while unseen:
        root = min(unseen)
        unseen.remove(root)
        component = {root}
        stack = [root]
        while stack:
            edge_index = stack.pop()
            for vert_index in endpoints_by_edge.get(edge_index, ()):
                for neighbour in edges_by_vertex.get(vert_index, ()):
                    if neighbour not in unseen:
                        continue
                    unseen.remove(neighbour)
                    component.add(neighbour)
                    stack.append(neighbour)
        components.append(tuple(sorted(component)))
    components.sort(key=lambda component: (component[0], len(component)))
    return tuple(components)


def _manual_terminal_routing(graph, rail_plan, backend_partitions):
    """RM9: публикует per-component terminal choice из rail IR."""

    if rail_plan is None or not all(
        hasattr(rail_plan, name)
        for name in ("edges", "routes", "terminal_uses")
    ):
        return ()
    edge_by_id = {edge.edge_id: edge for edge in rail_plan.edges}
    route_by_id = {route.route_id: route for route in rail_plan.routes}
    component_records = []
    for partition in backend_partitions:
        components = _manual_seam_edge_components(
            partition.corner_runs + partition.boundary_runs,
            partition.edge_indices,
        )
        backend_kind = (
            "RAIL_PLANAR"
            if partition.backend == DecalBackendKind.RAIL_PLANAR
            else str(
                getattr(partition.compiled_plan, "backend_kind", "PLANAR")
            )
        )
        for component in components:
            component_records.append(
                (
                    component[0],
                    component,
                    partition.backend,
                    backend_kind,
                    partition.compiled_plan,
                )
            )
    component_records.sort(key=lambda item: (item[0], item[1]))

    result = []
    for component_index, (
        _root_edge,
        component,
        backend,
        backend_kind,
        compiled_plan,
    ) in enumerate(component_records):
        component_edges = set(component)
        for use in rail_plan.terminal_uses:
            if use.spine_edge_id not in component_edges:
                continue
            choice = "PERP"
            edge_ids = ()
            patch_ids = tuple(
                sorted(
                    {
                        int(graph.face_to_patch[face_id])
                        for face_id in use.source_face_ids
                        if face_id in getattr(graph, "face_to_patch", {})
                    }
                )
            )
            patch_id = patch_ids[0] if len(patch_ids) == 1 else -1
            if (
                use.kind == RailTerminalKind.ROUTE
                and use.route_edge_id is not None
            ):
                guide = edge_by_id[use.route_edge_id]
                if guide.is_pchain:
                    choice = "PCHAIN"
                    route = route_by_id.get(use.route_id)
                    route_edges = (
                        tuple(segment.edge_id for segment in route.segments)
                        if route is not None
                        else ()
                    )
                    edge_ids = tuple(
                        dict.fromkeys((use.route_edge_id,) + route_edges)
                    )
                else:
                    choice = "FOLD"
                    edge_ids = (use.route_edge_id,)
            backend_capacity_policy = CapacityPolicy(
                getattr(
                    compiled_plan,
                    "capacity_policy",
                    CapacityPolicy.SATURATE_PROVEN,
                )
            )
            route = route_by_id.get(use.route_id)
            route_termination = getattr(
                route,
                "termination",
                RailTermination.PCHAIN,
            )
            if (
                route is not None
                and route_termination is RailTermination.ALPHA
                and backend_capacity_policy
                is not CapacityPolicy.REJECT_UNPROVEN
            ):
                capacity_policy = CapacityPolicy.CONTROLLED_RECOMPILE
            elif (
                route is not None
                and route_termination is not RailTermination.ALPHA
                and backend_capacity_policy
                is not CapacityPolicy.REJECT_UNPROVEN
            ):
                capacity_policy = CapacityPolicy.SATURATE_PROVEN
            else:
                capacity_policy = backend_capacity_policy
            result.append(
                ManualSeamTerminalRouting(
                    component_index=component_index,
                    patch_id=patch_id,
                    spine_vertex_id=int(use.spine_vertex_id),
                    spine_edge_id=int(use.spine_edge_id),
                    source_face_ids=tuple(use.source_face_ids),
                    backend=(backend.value if isinstance(backend, DecalBackendKind) else str(backend)),
                    backend_kind=backend_kind,
                    choice=choice,
                    edge_ids=edge_ids,
                    plan_kind=use.kind.value,
                    route_id=use.route_id,
                    capacity_policy=capacity_policy,
                )
            )
    return tuple(sorted(result))


def _group_boundary_runs(boundary_uses):
    """Односторонние boundary-runs по последовательным сегментам одной chain.

    Wing задаётся chain boundary-ориентацией (материал слева, n × t),
    поэтому runs строятся строго в порядке обхода chain и не реверсируются
    generic-ститчером. Сторона A пустая (нулевые нормали) — признак
    односторонней ветви для compiled decal backend.
    """

    runs = []
    for _chain_key, entries in sorted(boundary_uses.items()):
        entries.sort()
        current = None
        previous_segment = None
        for segment_index, edge_index, vert_indices, points, normal in entries:
            joins = (
                current is not None
                and previous_segment == segment_index - 1
                and current.vert_indices[-1] == vert_indices[0]
            )
            if joins:
                current.vert_indices.append(vert_indices[1])
                current.points.append(points[1])
                current.segment_normals_a.append(Vector((0.0, 0.0, 0.0)))
                current.segment_normals_b.append(normal)
                current.segment_convexities.append(0.0)
                current.segment_edge_indices.append(edge_index)
            else:
                current = _OrientedCornerRun(
                    vert_indices=list(vert_indices),
                    points=list(points),
                    segment_normals_a=[Vector((0.0, 0.0, 0.0))],
                    segment_normals_b=[normal],
                    segment_convexities=[0.0],
                    segment_edge_indices=[edge_index],
                )
                runs.append(current)
            previous_segment = segment_index
    return runs


def _materialization_error(
    network_face,
    face_index,
    backend,
    edge_indices,
    reason,
    repeated_keys=(),
):
    try:
        vertex_count = len(getattr(network_face, "vert_keys", ()))
    except TypeError:
        vertex_count = -1
    raw_cycle_keys = getattr(network_face, "vert_keys", ())
    try:
        cycle_keys = tuple(raw_cycle_keys)
    except TypeError:
        cycle_keys = (repr(raw_cycle_keys),)
    raw_cycle_positions = getattr(network_face, "positions", ())
    try:
        raw_cycle_positions = tuple(raw_cycle_positions)
    except TypeError:
        cycle_positions = (repr(raw_cycle_positions),)
    else:
        cycle_positions = []
        for position in raw_cycle_positions:
            try:
                cycle_positions.append(
                    tuple(float(value) for value in position)
                )
            except (TypeError, ValueError):
                # Диагностика не должна скрывать исходный hard fail, даже
                # если повреждённая face несёт несерилизуемую позицию.
                cycle_positions.append(repr(position))
        cycle_positions = tuple(cycle_positions)
    repeated_occurrences = []
    cycle_count = len(cycle_keys)
    for key in repeated_keys:
        indices = tuple(
            index
            for index, candidate in enumerate(cycle_keys)
            if candidate == key
        )
        adjacent_pairs = tuple(
            (first, second)
            for offset, first in enumerate(indices)
            for second in indices[offset + 1 :]
            if (second - first) % cycle_count in {1, cycle_count - 1}
        )
        repeated_occurrences.append((key, indices, adjacent_pairs))
    return DecalMaterializationError(
        backend=backend,
        edge_indices=edge_indices,
        face_index=face_index,
        reason=reason,
        vertex_count=vertex_count,
        repeated_keys=repeated_keys,
        cycle_keys=cycle_keys,
        cycle_positions=cycle_positions,
        repeated_occurrences=repeated_occurrences,
        component_kind=getattr(network_face, "component_kind", ""),
        component_side=getattr(network_face, "component_side", ""),
    )


def _validate_network_faces_for_materialization(
    network_faces, backend, edge_indices
):
    """Проверяет всю partition до первой записи в temporary BMesh."""

    for face_index, network_face in enumerate(network_faces):
        try:
            lengths = tuple(
                len(getattr(network_face, field, ()))
                for field in (
                    "vert_keys",
                    "positions",
                    "u_fracs",
                    "v_lengths",
                )
            )
        except TypeError as exc:
            raise _materialization_error(
                network_face,
                face_index,
                backend,
                edge_indices,
                f"invalid loop arrays: {exc}",
            ) from exc
        if len(set(lengths)) != 1:
            raise _materialization_error(
                network_face,
                face_index,
                backend,
                edge_indices,
                f"loop data length mismatch {lengths}",
            )
        seen_keys = set()
        repeated_keys = []
        try:
            for key in network_face.vert_keys:
                if key in seen_keys and key not in repeated_keys:
                    repeated_keys.append(key)
                seen_keys.add(key)
        except TypeError as exc:
            raise _materialization_error(
                network_face,
                face_index,
                backend,
                edge_indices,
                f"unhashable vertex key: {exc}",
            ) from exc
        if repeated_keys:
            raise _materialization_error(
                network_face,
                face_index,
                backend,
                edge_indices,
                "repeated vertex keys",
                repeated_keys,
            )
        if lengths[0] < 3:
            raise _materialization_error(
                network_face,
                face_index,
                backend,
                edge_indices,
                "fewer than three vertices",
            )


def _materialize_network_faces(
    bm,
    geometry_batch,
    settings,
    uv_rect,
    *,
    backend="UNKNOWN",
    edge_indices=(),
    evaluator_policy_counts=None,
    evaluation_ms=0.0,
):
    """Материализует faces decal-сети в bmesh с shared вершинами по ключам.

    Ключи ('sv', vert) сшивают spine-станции между крыльями/поверхностями и
    junction cores; прочие вершины дедуплицируются по квантованной позиции
    внутри поверхности, остаточные совпадения сваривает финальный weld.
    """

    try:
        backend_kind = DecalBackendKind(backend)
    except ValueError as exc:
        raise StrictSeamRuntimeError(
            edge_indices,
            (
                "LEGACY_NETWORK_MATERIALIZATION_DISABLED"
                if backend == "LEGACY_NETWORK"
                else f"UNSUPPORTED_MATERIALIZATION_BACKEND:{backend}"
            ),
        ) from exc
    if not isinstance(geometry_batch, GeometryBatch):
        raise TypeError("BMeshMaterializer requires immutable GeometryBatch")
    network_faces = geometry_batch.faces
    _validate_network_faces_for_materialization(
        network_faces, backend_kind.value, edge_indices
    )
    uv_layer = bm.loops.layers.uv.verify()
    u_min, _v_min, u_max, _v_max = uv_rect
    u_mid = (u_min + u_max) / 2.0
    u_radius = (u_max - u_min) / 2.0
    scale = settings.uv_length_scale
    verts_by_key = {}
    created = 0
    materialized_policy_counts = (
        {} if evaluator_policy_counts is None else None
    )

    for face_index, network_face in enumerate(network_faces):
        loop_data = []
        for key, position, u_frac, v_length in zip(
            network_face.vert_keys,
            network_face.positions,
            network_face.u_fracs,
            network_face.v_lengths,
        ):
            vert = verts_by_key.get(key)
            if vert is None:
                vert = bm.verts.new(position)
                verts_by_key[key] = vert
            loop_data.append((vert, u_frac, v_length))
        try:
            face = bm.faces.new(tuple(item[0] for item in loop_data))
            for loop, (_vert, u_frac, v_length) in zip(
                face.loops, loop_data
            ):
                loop[uv_layer].uv = (
                    u_mid + u_frac * u_radius,
                    v_length * scale,
                )
        except Exception as exc:
            raise _materialization_error(
                network_face,
                face_index,
                backend,
                edge_indices,
                f"{type(exc).__name__}: {exc}",
            ) from exc
        created += 1
        component_kind = str(
            getattr(network_face, "component_kind", "") or "SURFACE"
        )
        if materialized_policy_counts is not None:
            materialized_policy_counts[component_kind] = (
                materialized_policy_counts.get(component_kind, 0) + 1
            )
    policy_counts = (
        tuple(sorted(materialized_policy_counts.items()))
        if materialized_policy_counts is not None
        else tuple(
            sorted(
                (str(kind), int(count))
                for kind, count in evaluator_policy_counts
            )
        )
    )
    return DecalMaterializationResult(
        backend=backend_kind.value,
        edge_indices=tuple(
            sorted({int(index) for index in edge_indices or ()})
        ),
        source_face_count=len(network_faces),
        created_face_count=created,
        created_vertex_count=len(verts_by_key),
        policy_counts=policy_counts,
        evaluation_ms=float(evaluation_ms),
    )


def _collection_in_tree(root, target):
    stack = [root]
    while stack:
        current = stack.pop()
        for child in current.children:
            if child is target:
                return True
            stack.append(child)
    return False


def _decal_collection(scene):
    col = bpy.data.collections.get(DECAL_COLLECTION_NAME)
    if col is None:
        col = bpy.data.collections.new(DECAL_COLLECTION_NAME)
        scene.collection.children.link(col)
    elif not _collection_in_tree(scene.collection, col):
        # Коллекция существует, но не привязана к текущей сцене —
        # без привязки декали были бы невидимы.
        scene.collection.children.link(col)
    return col


def _decal_object_name(mode: str, source_obj) -> str:
    return f"Decal_{_MODE_OBJECT_SUFFIX[mode]}_{source_obj.name}"


def _decal_preview_object_name(mode: str, source_obj) -> str:
    return (
        f"{_DECAL_PREVIEW_OBJECT_PREFIX}_"
        f"{_MODE_OBJECT_SUFFIX[mode]}_{source_obj.name}"
    )


def _set_decal_preview_metadata(obj, source_obj):
    """Помечает временный object для cleanup/export filters."""

    try:
        obj[_DECAL_PREVIEW_MARKER] = True
        obj[_DECAL_PREVIEW_SOURCE] = source_obj.name
    except TypeError:
        # Lightweight unit-test doubles не реализуют Blender ID mapping.
        setattr(obj, _DECAL_PREVIEW_MARKER, True)
        setattr(obj, _DECAL_PREVIEW_SOURCE, source_obj.name)
    obj.hide_render = True


def _is_decal_preview_object(obj) -> bool:
    getter = getattr(obj, "get", None)
    if getter and getter(_DECAL_PREVIEW_MARKER, False):
        return True
    return bool(getattr(obj, _DECAL_PREVIEW_MARKER, False))


def _copy_decal_mesh_metadata(source_mesh, target_mesh):
    """Явно переносит material slots; object metadata сохраняет identity."""

    source_materials = getattr(source_mesh, "materials", ())
    target_materials = getattr(target_mesh, "materials", None)
    if target_materials is not None:
        for material in source_materials:
            target_materials.append(material)


def _bmesh_topology_payload(bm):
    """Каноническая face/loop signature, независимая от BMesh indices."""

    bm.verts.index_update()
    bm.faces.index_update()
    ordered_faces = sorted(bm.faces, key=lambda face: face.index)
    canonical_index_by_vert = {}
    canonical_verts = []
    face_signatures = []
    for face in ordered_faces:
        canonical_loop = []
        for loop in face.loops:
            canonical_index = canonical_index_by_vert.get(loop.vert)
            if canonical_index is None:
                canonical_index = len(canonical_verts)
                canonical_index_by_vert[loop.vert] = canonical_index
                canonical_verts.append(loop.vert)
            canonical_loop.append(canonical_index)
        face_signatures.append(
            (
                face.material_index,
                bool(face.smooth),
                tuple(canonical_loop),
            )
        )
    signature = (
        len(bm.verts),
        len(bm.edges),
        tuple(face_signatures),
    )
    canonical_verts = tuple(canonical_verts)
    return (
        signature,
        canonical_verts,
        tuple(vert.index for vert in canonical_verts),
    )


def _update_preview_mesh_data(
    bm,
    mesh,
    canonical_verts,
    canonical_mesh_indices,
):
    """Обновляет только coordinates/UV при уже доказанной topology parity."""

    ordered_faces = sorted(bm.faces, key=lambda face: face.index)
    loop_count = sum(len(face.loops) for face in ordered_faces)
    if (
        len(mesh.vertices) != len(canonical_verts)
        or len(mesh.polygons) != len(ordered_faces)
        or len(mesh.loops) != loop_count
        or len(canonical_mesh_indices) != len(canonical_verts)
        or set(canonical_mesh_indices) != set(range(len(mesh.vertices)))
    ):
        return False

    bm_uv_layer = bm.loops.layers.uv.active
    mesh_uv_layer = mesh.uv_layers.active
    if (bm_uv_layer is None) != (mesh_uv_layer is None):
        return False
    if mesh_uv_layer is not None and len(mesh_uv_layer.data) != loop_count:
        return False

    coordinates = [0.0] * (len(mesh.vertices) * 3)
    for vert, mesh_index in zip(canonical_verts, canonical_mesh_indices):
        coordinates[mesh_index * 3 : mesh_index * 3 + 3] = tuple(vert.co)
    mesh.vertices.foreach_set("co", coordinates)
    if bm_uv_layer is not None:
        uv_values = [
            component
            for face in ordered_faces
            for loop in face.loops
            for component in loop[bm_uv_layer].uv
        ]
        mesh_uv_layer.data.foreach_set("uv", uv_values)
    mesh.update()
    return True


def _record_preview_state(
    preview_state,
    obj,
    topology_signature,
    canonical_mesh_indices,
    rebuilt,
):
    if preview_state is None:
        return
    preview_state.topology_signature = topology_signature
    preview_state.canonical_mesh_indices = canonical_mesh_indices
    preview_state.object_name = obj.name
    preview_state.object_pointer = obj.as_pointer()
    preview_state.mesh_pointer = obj.data.as_pointer()
    if rebuilt:
        preview_state.topology_rebuilds += 1


def _prepare_decal_bmesh(bm):
    """Выполняет destructive cleanup только внутри temporary BMesh."""

    bmesh.ops.remove_doubles(
        bm, verts=list(bm.verts), dist=DECAL_WELD_DISTANCE
    )
    loose_verts = [vert for vert in bm.verts if not vert.link_faces]
    if loose_verts:
        bmesh.ops.delete(bm, geom=loose_verts, context="VERTS")
    return bool(bm.faces)


def _finalize_decal_object(
    bm,
    name: str,
    source_obj,
    scene,
    reuse_existing=False,
    preview_state=None,
    prepared=False,
):
    """Материализует точный production или persistent preview mesh.

    Production object никогда не удаляется: новый mesh сначала полностью
    строится отдельно и только затем подменяет data-block существующего
    object. Поэтому identity и object-level custom properties переживают
    confirm, а старый mesh удаляется лишь после успешного swap.
    """

    old_obj = bpy.data.objects.get(name)
    if reuse_existing and old_obj is not None and not _is_decal_preview_object(
        old_obj
    ):
        # Не присваиваем пользовательский object, случайно занявший internal
        # name: Blender выдаст preview безопасный суффикс.
        old_obj = None
    try:
        has_faces = bool(bm.faces) if prepared else _prepare_decal_bmesh(bm)
        if not has_faces:
            bm.free()
            if (
                reuse_existing
                and old_obj is not None
                and old_obj.mode != "EDIT"
                and old_obj.type == "MESH"
            ):
                # Невалидный промежуточный кадр не уничтожает последний
                # корректный preview. Confirm по-прежнему требует faces.
                return old_obj
            return None

        if preview_state is not None:
            (
                topology_signature,
                canonical_verts,
                canonical_mesh_indices,
            ) = _bmesh_topology_payload(bm)
        else:
            topology_signature = ()
            canonical_verts = ()
            canonical_mesh_indices = ()

        if (
            reuse_existing
            and old_obj is not None
            and old_obj.mode != "EDIT"
            and old_obj.type == "MESH"
            and old_obj.data is not None
        ):
            old_mesh = old_obj.data
            if (
                preview_state is not None
                and preview_state.topology_signature == topology_signature
                and preview_state.object_pointer == old_obj.as_pointer()
                and preview_state.mesh_pointer == old_mesh.as_pointer()
                and _update_preview_mesh_data(
                    bm,
                    old_mesh,
                    canonical_verts,
                    preview_state.canonical_mesh_indices,
                )
            ):
                bm.free()
                old_obj.matrix_world = source_obj.matrix_world.copy()
                preview_state.fast_updates += 1
                return old_obj
            if old_mesh.users > 1:
                old_mesh = old_mesh.copy()
                old_obj.data = old_mesh
            bm.to_mesh(old_mesh)
            old_mesh.update()
            bm.free()
            old_obj.matrix_world = source_obj.matrix_world.copy()
            _record_preview_state(
                preview_state,
                old_obj,
                topology_signature,
                canonical_mesh_indices,
                rebuilt=True,
            )
            return old_obj

        if old_obj is not None and old_obj.mode == "EDIT":
            raise RuntimeError(
                f"Cannot update decal '{name}' while it is in Edit Mode"
            )

        mesh = bpy.data.meshes.new(name + "_Geo")
        try:
            bm.to_mesh(mesh)
            mesh.update()
        except Exception:
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
            raise
    except Exception:
        bm.free()
        raise
    bm.free()

    if old_obj is not None:
        old_mesh = old_obj.data
        try:
            _copy_decal_mesh_metadata(old_mesh, mesh)
        except Exception:
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
            raise
        old_obj.data = mesh
        old_obj.matrix_world = source_obj.matrix_world.copy()
        if old_mesh is not None and old_mesh.users == 0:
            bpy.data.meshes.remove(old_mesh)
        obj = old_obj
    else:
        obj = None
        try:
            obj = bpy.data.objects.new(name, mesh)
            _decal_collection(scene).objects.link(obj)
            obj.matrix_world = source_obj.matrix_world.copy()
        except Exception:
            if obj is not None:
                bpy.data.objects.remove(obj, do_unlink=True)
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
            raise
    _record_preview_state(
        preview_state,
        obj,
        topology_signature,
        canonical_mesh_indices,
        rebuilt=True,
    )
    return obj


def _rail_geometry_owner_face_ids(plan):
    """Возвращает полный PLANAR footprint immutable rail-plan на max budget."""

    owner_face_ids = {
        int(cell.owner_face_id)
        for channel in plan.channels
        for cell in channel.cells
    }
    owner_face_ids.update(
        int(partition.owner_face_id)
        for partition in plan.corner_partitions
        if partition.owner_face_id is not None
    )
    owner_face_ids.update(
        int(cell.owner_face_id)
        for cell in getattr(plan, "corner_cells", ())
    )
    return frozenset(owner_face_ids)


def _rail_geometry_scope_has_face_conflicts(compiled_components):
    """R1 не смешивает независимые rail-конкуренты на одной source-грани."""

    footprints = [
        _rail_geometry_owner_face_ids(compiled_plan)
        for _component, compiled_plan in compiled_components
    ]
    for index, footprint in enumerate(footprints):
        for other_footprint in footprints[index + 1 :]:
            if footprint.intersection(other_footprint):
                return True
    return False


def _compile_rail_geometry_components(
    rail_plan,
    topology_components,
    settings,
):
    """Компилирует R1 plans без частичного routing side effect."""

    compiled_components = []
    failures = []
    for component in topology_components:
        geometry_attempt = compile_planar_rail_geometry_attempt(
            rail_plan,
            edge_indices=component,
            apex_limit=settings.corner_apex_limit,
            split_angle=settings.corner_acute_split_angle,
            dynamic_corner_bands=settings.dynamic_corner_bands,
            join_mode=settings.corner_join_mode,
        )
        if geometry_attempt.plan is None:
            failures.extend(geometry_attempt.failures)
            continue
        compiled_components.append((component, geometry_attempt.plan))
    return compiled_components, failures


def _rail_geometry_required_trace_scale(compiled_components):
    """Максимальный stable-A10 scale для сохранения modal headroom."""

    scales = [1.0]
    scales.extend(
        float(scale)
        for _component, plan in compiled_components
        for _path_id, scale in plan.path_reach_scales
    )
    return max(scales)


def _strict_backend_component_rejections(
    components,
    failures,
    *,
    default_reason="PATCH_VORONOI_STRICT_COMPILE_FAILED",
):
    """Переводит неподдержанный topology-component в явные failed edges."""

    result = []
    for component in components:
        component_edges = {int(edge_index) for edge_index in component}
        reasons = sorted(
            {
                str(getattr(failure, "reason", "UNKNOWN"))
                for failure in failures
                if component_edges.intersection(
                    int(edge_index)
                    for edge_index in (
                        getattr(failure, "edge_indices", ()) or ()
                    )
                )
            }
        )
        reason = "+".join(reasons) if reasons else str(default_reason)
        result.extend(
            ManualSeamEdgeRejection(
                edge_index=edge_index,
                reason=reason,
            )
            for edge_index in sorted(component_edges)
        )
    return tuple(result)


def compile_manual_seam_decal_plan(
    analysis_bundle: AnalysisBundle,
    settings: DecalSettings,
    selected_edge_indices,
    *,
    alpha_budget=None,
    rail_mark_edge_indices=(),
):
    """Собирает selected edges/runs и статическую сеть один раз на invoke."""

    require_decal_corner_join_available(settings)
    if not isinstance(analysis_bundle, AnalysisBundle):
        raise AnalysisSchemaError(
            "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED",
            "SEAMS compiler requires AnalysisBundle",
        )
    analysis_bundle.capabilities.require_supported()
    graph = analysis_bundle.patch_graph
    if alpha_budget is None:
        alpha_budget = decal_compile_alpha_budget(settings)
    alpha_budget = float(alpha_budget)
    if not alpha_budget > 0.0:
        raise ValueError("Decal compile alpha_budget must be positive")
    selected_edges = tuple(
        sorted({int(edge_index) for edge_index in selected_edge_indices or ()})
    )
    collection = _collect_manual_edge_decals(
        graph, selected_edge_indices
    )
    corner_runs, boundary_runs = collection
    accepted_scope_edges = collection.accepted_edge_indices
    rejected_edges = list(collection.rejected_edges)
    patch_voronoi_plan = None
    backend_partitions = []
    compile_failures = ()
    rail_plan = None
    rail_compile_failures = ()
    rail_geometry_failures = []
    if accepted_scope_edges:
        rail_attempt = compile_decal_rail_attempt(
            analysis_bundle,
            accepted_scope_edges,
            alpha_budget=alpha_budget,
            rail_mark_edge_indices=rail_mark_edge_indices,
        )
        rail_plan = rail_attempt.plan
        rail_compile_failures = rail_attempt.failures
    if corner_runs or boundary_runs:
        topology_components = _manual_seam_edge_components(
            corner_runs + boundary_runs, accepted_scope_edges
        )
        fallback_components = []
        if rail_plan is None:
            fallback_components = list(topology_components)
        else:
            compiled_components, geometry_failures = (
                _compile_rail_geometry_components(
                    rail_plan,
                    topology_components,
                    settings,
                )
            )
            rail_geometry_failures.extend(geometry_failures)

            # До R2 Patch и rail не могут независимо доказывать отсутствие
            # конкуренции. Поэтому R1 принимает весь scope только целиком:
            # все компоненты PLANAR и их max-budget source-face footprints
            # попарно не пересекаются. Иначе старый joint Patch backend
            # сохраняет глобальную конкуренцию и single-cover.
            all_components_compiled = (
                len(compiled_components) == len(topology_components)
            )
            face_conflict = (
                all_components_compiled
                and _rail_geometry_scope_has_face_conflicts(compiled_components)
            )
            rail_scope_ready = all_components_compiled and not face_conflict

            if rail_scope_ready:
                # Stable A10 miter может требовать rail trace длиннее
                # нейтральной ширины. Сохраняем исходный B4 headroom в
                # эффективном (s, r)-домене одним compile-only retry;
                # Patch budget при этом не меняется.
                trace_scale = _rail_geometry_required_trace_scale(
                    compiled_components
                )
                expanded_budget = (
                    alpha_budget * trace_scale
                    + DECAL_WELD_DISTANCE * 8.0
                    if trace_scale > 1.0
                    else float(rail_plan.alpha_budget)
                )
                if expanded_budget > float(rail_plan.alpha_budget):
                    expanded_attempt = compile_decal_rail_attempt(
                        analysis_bundle,
                        accepted_scope_edges,
                        alpha_budget=expanded_budget,
                        rail_mark_edge_indices=rail_mark_edge_indices,
                    )
                    if expanded_attempt.plan is None:
                        rail_compile_failures = tuple(
                            rail_compile_failures
                        ) + tuple(expanded_attempt.failures)
                        rail_geometry_failures.append(
                            RailGeometryFailure(
                                reason=(
                                    "RAIL_GEOMETRY_SUPPORT_RECOMPILE_FAILED"
                                ),
                                edge_indices=tuple(accepted_scope_edges),
                                details=(("trace_scale", trace_scale),),
                            )
                        )
                        rail_scope_ready = False
                    else:
                        expanded_components, expanded_failures = (
                            _compile_rail_geometry_components(
                                expanded_attempt.plan,
                                topology_components,
                                settings,
                            )
                        )
                        rail_geometry_failures.extend(expanded_failures)
                        expanded_complete = (
                            len(expanded_components)
                            == len(topology_components)
                        )
                        expanded_conflict = (
                            expanded_complete
                            and _rail_geometry_scope_has_face_conflicts(
                                expanded_components
                            )
                        )
                        effective_budget_ok = (
                            expanded_complete
                            and all(
                                float(plan.alpha_budget) >= alpha_budget
                                for _component, plan in expanded_components
                            )
                        )
                        if (
                            expanded_complete
                            and not expanded_conflict
                            and effective_budget_ok
                        ):
                            rail_plan = expanded_attempt.plan
                            rail_compile_failures = tuple(
                                expanded_attempt.failures
                            )
                            compiled_components = expanded_components
                        else:
                            rail_scope_ready = False
                            if expanded_conflict:
                                face_conflict = True
                            elif expanded_complete and not effective_budget_ok:
                                rail_geometry_failures.append(
                                    RailGeometryFailure(
                                        reason=(
                                            "RAIL_GEOMETRY_DOMAIN_BUDGET_"
                                            "UNAVAILABLE"
                                        ),
                                        edge_indices=tuple(
                                            accepted_scope_edges
                                        ),
                                        details=(
                                            (
                                                "required_alpha_budget",
                                                alpha_budget,
                                            ),
                                        ),
                                    )
                                )

            if not rail_scope_ready:
                fallback_components = list(topology_components)
                if face_conflict:
                    rail_geometry_failures.append(
                        RailGeometryFailure(
                            reason="RAIL_GEOMETRY_COMPETITION_PENDING",
                            edge_indices=tuple(
                                edge_index
                                for component in topology_components
                                for edge_index in component
                            ),
                            details=(("component_count", len(topology_components)),),
                        )
                    )
            else:
                for component, compiled_plan in compiled_components:
                    component_corner_runs, component_boundary_runs = (
                        _collect_manual_edge_decals(graph, component)
                    )
                    backend_partitions.append(
                        _ManualSeamBackendPartition(
                            backend=DecalBackendKind.RAIL_PLANAR,
                            edge_indices=tuple(component),
                            topology_component_count=1,
                            corner_runs=tuple(component_corner_runs),
                            boundary_runs=tuple(component_boundary_runs),
                            compiled_plan=compiled_plan,
                        )
                    )

        fallback_scope_edges = tuple(
            sorted(
                edge_index
                for component in fallback_components
                for edge_index in component
            )
        )
        failed_components = []
        accepted_components = []
        accepted_edges = ()
        if fallback_scope_edges:
            attempt = compile_patch_voronoi_attempt(
                analysis_bundle,
                fallback_scope_edges,
                settings.offset,
                allow_partial=True,
                alpha_budget=alpha_budget,
                distortion_budget=settings.chart_distortion_budget,
                corner_join_mode=settings.corner_join_mode,
            )
            compile_failures = attempt.failures
            rejected_seed = set(attempt.rejected_edge_indices)
            if attempt.plan is None and not rejected_seed:
                rejected_seed.update(fallback_scope_edges)

            failed_components = [
                component
                for component in fallback_components
                if rejected_seed.intersection(component)
            ]
            accepted_components = [
                component
                for component in fallback_components
                if not rejected_seed.intersection(component)
            ]
            accepted_edges = tuple(
                sorted(
                    edge_index
                    for component in accepted_components
                    for edge_index in component
                )
            )

            if not failed_components and attempt.plan is not None:
                patch_voronoi_plan = attempt.plan
            elif accepted_edges:
                # Partial probe мог скомпилировать surface, содержащую sites
                # из rejected topology component. Strict compile строит
                # competition domain только для оставшихся компонентов.
                patch_voronoi_plan = compile_patch_voronoi_plan(
                    analysis_bundle,
                    accepted_edges,
                    settings.offset,
                    alpha_budget=alpha_budget,
                    distortion_budget=settings.chart_distortion_budget,
                    corner_join_mode=settings.corner_join_mode,
                )
                if patch_voronoi_plan is None:
                    # Компонентная атомарность важнее частичного результата:
                    # без strict plan весь современный scope явно провален.
                    failed_components = list(fallback_components)
                    accepted_components = []
                    accepted_edges = ()

            if patch_voronoi_plan is not None and accepted_edges:
                accepted_corner_runs, accepted_boundary_runs = (
                    _collect_manual_edge_decals(graph, accepted_edges)
                )
                backend_partitions.append(
                    _ManualSeamBackendPartition(
                        backend=DecalBackendKind.PATCH_VORONOI,
                        edge_indices=accepted_edges,
                        topology_component_count=len(accepted_components),
                        corner_runs=tuple(accepted_corner_runs),
                        boundary_runs=tuple(accepted_boundary_runs),
                        compiled_plan=patch_voronoi_plan,
                    )
                )

        rejected_edges.extend(
            _strict_backend_component_rejections(
                failed_components,
                compile_failures,
            )
        )
    terminal_routing = _manual_terminal_routing(
        graph, rail_plan, tuple(backend_partitions)
    )
    plan = ManualSeamDecalPlan(
        corner_runs=tuple(corner_runs),
        boundary_runs=tuple(boundary_runs),
        patch_voronoi_plan=patch_voronoi_plan,
        backend_partitions=tuple(backend_partitions),
        compile_failures=tuple(compile_failures),
        selected_edge_indices=selected_edges,
        rejected_edges=tuple(rejected_edges),
        rail_plan=rail_plan,
        rail_compile_failures=tuple(rail_compile_failures),
        rail_geometry_failures=tuple(rail_geometry_failures),
        terminal_routing=terminal_routing,
    )
    if not plan.accounting_is_exact:
        raise AssertionError(
            "Manual seam edge accounting invariant violated: "
            "selected != rail_planar + patch_voronoi + failed"
        )
    if plan.accepted_rail_planar_edge_indices and (
        plan.accepted_patch_voronoi_edge_indices
    ):
        raise AssertionError(
            "R1 routing invariant violated: RAIL_PLANAR cannot mix with "
            "Patch before rail competition is implemented"
        )
    if plan.backend_summary:
        print(f"[CFTUV][Decals] backend routing: {plan.backend_summary}")
    for line in plan.terminal_routing_report:
        print(f"[CFTUV][Decals] terminal routing: {line}")
    if plan.compile_failures:
        details = ", ".join(
            f"patch {failure.patch_id}:{failure.reason}"
            for failure in plan.compile_failures
        )
        print(f"[CFTUV][Decals] unsupported surface reasons: {details}")
    if plan.rail_compile_failures:
        details = ", ".join(
            str(failure.reason) for failure in plan.rail_compile_failures
        )
        print(f"[CFTUV][Decals] rail materialization unavailable: {details}")
    if plan.rejected_edges:
        details = ", ".join(
            f"{rejection.edge_index}:{rejection.reason}"
            f"(uses={rejection.use_count})"
            for rejection in plan.rejected_edges
        )
        print(f"[CFTUV][Decals] failed selected edges: {details}")
    return plan


def _evaluate_manual_backend_partition(
    partition,
    settings,
    width,
    preview,
    *,
    rail_plan=None,
    terminal_routing=(),
):
    """Вычисляет одну routing-группу без BMesh side effects."""

    started = perf_counter()
    if partition.backend == DecalBackendKind.RAIL_PLANAR:
        try:
            faces = evaluate_planar_rail_geometry_plan(
                partition.compiled_plan,
                width,
                offset=settings.offset,
                preview=preview,
            )
        except Exception as exc:
            raise RailPlanarRuntimeError(
                partition.edge_indices,
                width,
                preview,
                repr(exc),
            ) from exc
        if not faces:
            raise RailPlanarRuntimeError(
                partition.edge_indices,
                width,
                preview,
                "evaluation produced no faces",
            )
        return _ManualBackendEvaluation(
            geometry_batch=geometry_batch_from_faces(faces),
            evaluation_ms=(perf_counter() - started) * 1000.0,
        )

    if partition.backend == DecalBackendKind.PATCH_VORONOI:
        diagnostics = PatchVoronoiDiagnostics()
        try:
            faces = evaluate_patch_voronoi_plan(
                partition.compiled_plan,
                width,
                preview=preview,
                corner_settings=corner_runtime_settings_from_decal_settings(
                    settings
                ),
                diagnostics=diagnostics,
                terminal_routing=terminal_routing,
                rail_plan=rail_plan,
            )
        except Exception as exc:
            raise PatchVoronoiRuntimeError(
                partition.edge_indices,
                width,
                preview,
                repr(exc),
            ) from exc
        if not faces:
            raise PatchVoronoiRuntimeError(
                partition.edge_indices,
                width,
                preview,
                "evaluation produced no faces",
            )
        return _ManualBackendEvaluation(
            geometry_batch=geometry_batch_from_faces(
                faces,
                diagnostics=tuple(
                    sorted(
                        {
                            **diagnostics.runtime_policy_counts,
                            **diagnostics.cap_keep_counts,
                        }.items()
                    )
                ),
            ),
            evaluation_ms=(perf_counter() - started) * 1000.0,
            policy_counts=tuple(
                sorted(
                    {
                        **diagnostics.runtime_policy_counts,
                        **diagnostics.cap_keep_counts,
                    }.items()
                )
            ),
        )

    raise StrictSeamRuntimeError(
        partition.edge_indices,
        f"UNSUPPORTED_BACKEND:{partition.backend}",
    )


@dataclass(frozen=True, init=False)
class ManualSeamFacesResult:
    """Immutable evaluator result shared by display adapters."""

    geometry_batch: GeometryBatch
    policy_counts: tuple = ()
    evaluation_ms: float = 0.0

    def __init__(
        self,
        geometry_batch=None,
        *,
        faces=(),
        policy_counts=(),
        evaluation_ms=0.0,
    ):
        if geometry_batch is None:
            geometry_batch = geometry_batch_from_faces(faces)
        object.__setattr__(self, "geometry_batch", geometry_batch)
        object.__setattr__(self, "policy_counts", tuple(policy_counts))
        object.__setattr__(self, "evaluation_ms", float(evaluation_ms))

    @property
    def faces(self):
        return self.geometry_batch.faces


def evaluate_manual_seam_faces(
    source_obj, settings, decal_plan, preview=True
):
    """Display-adapter путь: evaluated faces БЕЗ BMesh side effects.

    Использует тот же transaction-порядок и те же evaluator'ы, что
    mesh-путь (`_fill_manual_chain_decals`): parity by construction.
    Исключения evaluator'а (включая DOMAIN_BUDGET_EXCEEDED) пробрасываются
    вызывающему; оба preview-adapter'а применяют единственную политику
    PreviewFailurePolicy.CLEAR.
    """

    require_decal_corner_join_available(settings)
    started = perf_counter()
    local_settings = local_decal_settings_for_source(settings, source_obj)
    width = local_settings.width_seam
    faces = []
    policy_totals = {}
    for partition in decal_plan.backend_partitions:
        evaluation = _evaluate_manual_backend_partition(
            partition,
            local_settings,
            width,
            preview,
            rail_plan=decal_plan.rail_plan,
            terminal_routing=decal_plan.terminal_routing,
        )
        faces.extend(evaluation.faces)
        if partition.backend == DecalBackendKind.PATCH_VORONOI:
            for kind, count in evaluation.policy_counts:
                policy_totals[kind] = policy_totals.get(kind, 0) + count
    return ManualSeamFacesResult(
        geometry_batch=GeometryBatch(tuple(faces)),
        policy_counts=tuple(sorted(policy_totals.items())),
        evaluation_ms=(perf_counter() - started) * 1000.0,
    )


def _fill_manual_chain_decals(
    bm,
    graph,
    settings,
    chain_refs,
    mode="SEAMS",
    selected_edge_indices=None,
    preview=False,
    decal_plan=None,
):
    """Материализует только strict compiled SEAMS по exact physical scope."""

    require_decal_mode_available(mode)
    if mode != "SEAMS":
        raise StrictSeamRuntimeError(
            selected_edge_indices or (),
            f"SEAMS_MODE_REQUIRED:{mode}",
        )
    is_seam_mode = mode == "SEAMS"
    if is_seam_mode and selected_edge_indices is None:
        raise StrictSeamRuntimeError((), "SEAMS_REQUIRE_SELECTED_EDGE_PLAN")
    width = settings.width_seam
    uv_rect = DECAL_UV_RECT_SEAM

    if selected_edge_indices is not None:
        if decal_plan is None:
            corner_runs, boundary_runs = _collect_manual_edge_decals(
                graph, selected_edge_indices
            )
        else:
            corner_runs = decal_plan.corner_runs
            boundary_runs = decal_plan.boundary_runs
        if is_seam_mode:
            if decal_plan is None:
                raise StrictSeamRuntimeError(
                    selected_edge_indices,
                    "SEAMS_REQUIRE_COMPILED_PLAN",
                )
            if type(decal_plan) is not ManualSeamDecalPlan:
                raise StrictSeamRuntimeError(
                    selected_edge_indices,
                    "SEAMS_PLAN_TYPE_INVALID",
                )
            runtime_scope = tuple(
                sorted({int(index) for index in selected_edge_indices})
            )
            raw_selected = tuple(
                int(index) for index in decal_plan.selected_edge_indices
            )
            raw_accepted = tuple(
                int(edge_index)
                for partition in decal_plan.backend_partitions
                for edge_index in partition.edge_indices
            )
            raw_rejected = tuple(
                int(rejection.edge_index)
                for rejection in decal_plan.rejected_edges
            )
            compiled_scope = tuple(sorted(set(raw_selected)))
            accepted_scope = set(raw_accepted)
            rejected_scope = set(raw_rejected)
            structurally_exact = (
                decal_plan.network_plan is None
                and not decal_plan.direct_legacy_edge_indices
                and all(
                    type(partition) is _ManualSeamBackendPartition
                    and partition.backend
                    in {
                        DecalBackendKind.RAIL_PLANAR,
                        DecalBackendKind.PATCH_VORONOI,
                    }
                    for partition in decal_plan.backend_partitions
                )
                and len(raw_selected) == len(compiled_scope)
                and len(raw_accepted) == len(accepted_scope)
                and len(raw_rejected) == len(rejected_scope)
                and not accepted_scope.intersection(rejected_scope)
                and set(compiled_scope)
                == accepted_scope.union(rejected_scope)
            )
            if not structurally_exact:
                raise StrictSeamRuntimeError(
                    compiled_scope,
                    "SEAMS_PLAN_ACCOUNTING_MISMATCH",
                )
            if runtime_scope != compiled_scope:
                scope_delta = tuple(
                    sorted(set(runtime_scope).symmetric_difference(compiled_scope))
                )
                raise StrictSeamRuntimeError(
                    scope_delta,
                    "SEAMS_PLAN_SCOPE_MISMATCH:"
                    f"runtime={runtime_scope},compiled={compiled_scope}",
                )
            if decal_plan.rejected_edges:
                # SEAMS-транзакция атомарна: частично поддержанный scope не
                # материализуется. Иначе щель выглядела бы как успешный
                # результат, а неподдержанный компонент оставался бы скрыт.
                raise StrictSeamRuntimeError(
                    decal_plan.rejected_edge_indices,
                    decal_plan.backend_summary
                    or "UNSUPPORTED_SEAMS_COMPONENT",
                )
            if not decal_plan.backend_partitions:
                raise StrictSeamRuntimeError(
                    selected_edge_indices,
                    decal_plan.backend_summary
                    or "NO_SUPPORTED_SEAMS_BACKEND",
                )
            # Все современные partitions вычисляются до первой BMesh-записи.
            # Ни compile-, ни runtime-отказ не может перейти в старую сеть.
            face_batches = [
                (
                    partition,
                    _evaluate_manual_backend_partition(
                        partition,
                        settings,
                        width,
                        preview,
                        rail_plan=decal_plan.rail_plan,
                        terminal_routing=decal_plan.terminal_routing,
                    ),
                )
                for partition in decal_plan.backend_partitions
            ]
            materialization_results = []
            for partition, evaluation in face_batches:
                materialization_results.append(
                    _materialize_network_faces(
                        bm,
                        evaluation.geometry_batch,
                        settings,
                        uv_rect,
                        backend=partition.backend,
                        edge_indices=partition.edge_indices,
                        evaluator_policy_counts=(
                            evaluation.policy_counts
                            if partition.backend == DecalBackendKind.PATCH_VORONOI
                            else None
                        ),
                        evaluation_ms=evaluation.evaluation_ms,
                    )
                )
            return tuple(materialization_results)


def _fill_decal_bmesh(
    bm,
    graph: PatchGraph,
    settings: DecalSettings,
    mode: str,
    chain_refs=None,
    selected_edge_indices=None,
    preview=False,
    decal_plan=None,
):
    require_decal_mode_available(mode)
    if mode != "SEAMS":
        raise StrictSeamRuntimeError(
            selected_edge_indices or (),
            f"SEAMS_MODE_REQUIRED:{mode}",
        )
    if chain_refs is None or selected_edge_indices is None:
        raise StrictSeamRuntimeError(
            selected_edge_indices or (),
            "SEAMS_REQUIRE_SELECTED_EDGE_PLAN",
        )
    return _fill_manual_chain_decals(
        bm,
        graph,
        settings,
        chain_refs,
        mode=mode,
        selected_edge_indices=selected_edge_indices,
        preview=preview,
        decal_plan=decal_plan,
    ) or ()


@dataclass(frozen=True)
class _DecalTransactionResult:
    obj: object | None
    topology_changed: bool
    policy_counts: tuple[tuple[str, int], ...]
    evaluation_ms: float = 0.0


def _aggregate_policy_counts(materialization_results):
    counts = {}
    for result in materialization_results or ():
        for kind, count in result.policy_counts:
            counts[kind] = counts.get(kind, 0) + int(count)
    return tuple(sorted(counts.items()))


def _aggregate_evaluation_ms(materialization_results):
    return sum(
        float(getattr(result, "evaluation_ms", 0.0))
        for result in materialization_results or ()
    )


def _existing_decal_object(name):
    data = getattr(bpy, "data", None)
    objects = getattr(data, "objects", None)
    getter = getattr(objects, "get", None)
    if getter is None:
        return None
    obj = getter(name)
    if (
        obj is None
        or getattr(obj, "mode", "OBJECT") == "EDIT"
        or getattr(obj, "type", "MESH") != "MESH"
        or getattr(obj, "data", None) is None
    ):
        return None
    return obj


def _existing_decal_preview_object(name):
    obj = _existing_decal_object(name)
    return obj if obj is not None and _is_decal_preview_object(obj) else None


def _reset_decal_preview_state(preview_state):
    if preview_state is None:
        return
    preview_state.topology_signature = ()
    preview_state.canonical_mesh_indices = ()
    preview_state.object_name = ""
    preview_state.object_pointer = 0
    preview_state.mesh_pointer = 0


def remove_decal_preview_object(
    mode: str,
    source_obj,
    preview_state=None,
) -> bool:
    """Удаляет только internal preview object и его orphan mesh."""

    if mode not in DECAL_MODES:
        return False
    name = (
        preview_state.object_name
        if preview_state is not None and preview_state.object_name
        else _decal_preview_object_name(mode, source_obj)
    )
    obj = _existing_decal_object(name)
    if obj is None or not _is_decal_preview_object(obj):
        _reset_decal_preview_state(preview_state)
        return False
    mesh = obj.data
    bpy.data.objects.remove(obj, do_unlink=True)
    if mesh is not None and mesh.users == 0:
        bpy.data.meshes.remove(mesh)
    _reset_decal_preview_state(preview_state)
    return True


def _generate_decal_transaction(
    graph,
    source_obj,
    settings,
    mode,
    scene,
    chain_refs,
    selected_edge_indices,
    preview,
    decal_plan,
    preview_state,
):
    """Выполняет raising generation path с backend-local settings."""

    require_decal_mode_available(mode)
    require_decal_corner_join_available(settings)
    topology_rebuilds_before = (
        preview_state.topology_rebuilds if preview_state is not None else 0
    )
    bm = bmesh.new()
    try:
        materialization_results = _fill_decal_bmesh(
            bm,
            graph,
            settings,
            mode,
            chain_refs=chain_refs,
            selected_edge_indices=selected_edge_indices,
            preview=preview,
            decal_plan=decal_plan,
        )
        has_faces = _prepare_decal_bmesh(bm)
    except Exception:
        bm.free()
        raise

    if not has_faces:
        bm.free()
        return _DecalTransactionResult(None, False, ())

    object_name = (
        (
            preview_state.object_name
            if preview_state is not None and preview_state.object_name
            else _decal_preview_object_name(mode, source_obj)
        )
        if preview
        else _decal_object_name(mode, source_obj)
    )
    obj = _finalize_decal_object(
        bm,
        object_name,
        source_obj,
        scene,
        reuse_existing=preview,
        # Production materialization не имеет права перепривязывать state
        # временного объекта. Иначе confirm записывает сюда имя Decal_*, а
        # terminal cleanup теряет .CFTUV_Preview_* и оставляет оба объекта.
        preview_state=preview_state if preview else None,
        prepared=True,
    )
    if preview and obj is not None:
        _set_decal_preview_metadata(obj, source_obj)
    topology_changed = obj is not None
    if preview_state is not None:
        topology_changed = (
            preview_state.topology_rebuilds > topology_rebuilds_before
        )
    return _DecalTransactionResult(
        obj=obj,
        topology_changed=topology_changed,
        policy_counts=_aggregate_policy_counts(materialization_results),
        evaluation_ms=_aggregate_evaluation_ms(materialization_results),
    )


def generate_decal_result(
    graph: PatchGraph,
    source_obj,
    settings: DecalSettings,
    mode: str,
    scene=None,
    chain_refs=None,
    selected_edge_indices=None,
    preview=False,
    decal_plan=None,
    preview_state=None,
) -> DecalGenerationResult:
    """Генерирует decal и возвращает status без исключений geometry runtime."""

    failure_policy = PreviewFailurePolicy.CLEAR
    backend_summary = getattr(decal_plan, "backend_summary", "")
    if mode not in DECAL_MODES:
        return DecalGenerationResult(
            PreviewStatus.ERROR,
            None,
            reason=f"Unknown decal mode: {mode}",
            backend_summary=backend_summary,
        )
    try:
        require_decal_mode_available(mode)
    except DecalModeArchivedError as exc:
        if preview:
            remove_decal_preview_object(
                mode,
                source_obj,
                preview_state,
            )
        return DecalGenerationResult(
            PreviewStatus.ERROR,
            None,
            reason=exc.reason,
            backend_summary=backend_summary,
        )
    require_decal_corner_join_available(settings)
    try:
        local_settings = local_decal_settings_for_source(settings, source_obj)
    except DecalSourceTransformError as exc:
        if mode == "SEAMS" and preview:
            remove_decal_preview_object(
                mode,
                source_obj,
                preview_state,
            )
        return DecalGenerationResult(
            PreviewStatus.ERROR,
            None,
            reason=str(exc),
            backend_summary=backend_summary,
        )
    if scene is None:
        scene = bpy.context.scene
    try:
        transaction = _generate_decal_transaction(
            graph,
            source_obj,
            local_settings,
            mode,
            scene,
            chain_refs,
            selected_edge_indices,
            preview,
            decal_plan,
            preview_state,
        )
    except Exception as exc:
        if preview and failure_policy == PreviewFailurePolicy.CLEAR:
            remove_decal_preview_object(
                mode,
                source_obj,
                preview_state,
            )
        return DecalGenerationResult(
            PreviewStatus.ERROR,
            None,
            reason=str(exc),
            backend_summary=backend_summary,
        )

    if transaction.obj is None:
        if preview and failure_policy == PreviewFailurePolicy.CLEAR:
            remove_decal_preview_object(
                mode,
                source_obj,
                preview_state,
            )
        return DecalGenerationResult(
            PreviewStatus.EMPTY,
            None,
            reason="generation produced no faces",
            backend_summary=backend_summary,
        )
    return DecalGenerationResult(
        PreviewStatus.UPDATED,
        transaction.obj.name,
        topology_changed=transaction.topology_changed,
        backend_summary=backend_summary,
        policy_counts=transaction.policy_counts,
        evaluation_ms=transaction.evaluation_ms,
    )


def generate_decal_objects(
    graph: PatchGraph,
    source_obj,
    settings: DecalSettings,
    mode: str,
    scene=None,
    chain_refs=None,
    selected_edge_indices=None,
    preview=False,
    decal_plan=None,
    preview_state=None,
) -> list[str]:
    """Compatibility wrapper: генерирует decal и возвращает имена объектов.

    preview=True — быстрый режим для интерактивного modal drag (SEAMS
    network): криволинейные границы не уточняются хордами. Финальная
    генерация вызывается с preview=False.
    """

    if mode not in DECAL_MODES:
        raise ValueError(f"Unknown decal mode: {mode}")
    require_decal_mode_available(mode)
    require_decal_corner_join_available(settings)
    try:
        local_settings = local_decal_settings_for_source(settings, source_obj)
        if scene is None:
            scene = bpy.context.scene
        transaction = _generate_decal_transaction(
            graph,
            source_obj,
            local_settings,
            mode,
            scene,
            chain_refs,
            selected_edge_indices,
            preview,
            decal_plan,
            preview_state,
        )
    except Exception:
        if mode == "SEAMS" and preview:
            remove_decal_preview_object(
                mode,
                source_obj,
                preview_state,
            )
        raise
    if transaction.obj is not None:
        return [transaction.obj.name]
    if mode == "SEAMS":
        if preview:
            remove_decal_preview_object(
                mode,
                source_obj,
                preview_state,
            )
        raise StrictSeamRuntimeError(
            selected_edge_indices or (),
            "SEAMS_GENERATION_PRODUCED_NO_FACES",
        )
    retained = (
        _existing_decal_preview_object(
            (
                preview_state.object_name
                if preview_state is not None and preview_state.object_name
                else _decal_preview_object_name(mode, source_obj)
            )
        )
        if preview
        else None
    )
    return [retained.name] if retained is not None else []
