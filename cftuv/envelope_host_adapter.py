"""Pure host mapping for the static exact-planar Envelope debug pipeline.

The module imports the standalone kernel lazily so the Blender add-on remains
loadable when the wheel or its pinned SymPy dependency is unavailable.
"""

from __future__ import annotations

import hashlib
import importlib
import json
import math
import time
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from enum import Enum
from typing import TYPE_CHECKING

from .model import ChainNeighborKind, LoopKind, PatchType

if TYPE_CHECKING:
    import cftuv_envelope as envelope_kernel

    from .surface_ir import AnalysisBundle


class EnvelopeDebugHostOutcome(str, Enum):
    EXACT = "EXACT"
    ENVELOPE_DEBUG_KERNEL_UNAVAILABLE = "ENVELOPE_DEBUG_KERNEL_UNAVAILABLE"
    ENVELOPE_DEBUG_SYMPY_VERSION_UNSUPPORTED = (
        "ENVELOPE_DEBUG_SYMPY_VERSION_UNSUPPORTED"
    )
    ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID = (
        "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID"
    )
    ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE = (
        "ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE"
    )
    ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE = (
        "ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE"
    )
    ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED = (
        "ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED"
    )
    ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID = (
        "ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID"
    )
    ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE = (
        "ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE"
    )
    ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE = (
        "ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE"
    )
    ENVELOPE_DEBUG_EMPTY_SELECTION = "ENVELOPE_DEBUG_EMPTY_SELECTION"
    ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN = (
        "ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN"
    )
    ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED = (
        "ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED"
    )
    ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED = (
        "ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED"
    )


class EnvelopeDebugHostSeverity(str, Enum):
    INFO = "INFO"
    UNSUPPORTED = "UNSUPPORTED"
    ERROR = "ERROR"


@dataclass(frozen=True, slots=True)
class EnvelopeDebugHostDiagnosticV1:
    outcome: EnvelopeDebugHostOutcome | str
    severity: EnvelopeDebugHostSeverity
    message: str
    patch_domain_id: str | None = None


@dataclass(frozen=True, slots=True)
class EnvelopeDebugTimingV1:
    stage: str
    elapsed_seconds: float


@dataclass(frozen=True, slots=True)
class EnvelopeDebugEvaluationV1:
    snapshot: envelope_kernel.AnalysisSnapshotV1 | None
    request: envelope_kernel.DecalRequestV1 | None
    compilations: tuple[envelope_kernel.ReferenceEnvelopeCompilationV1, ...]
    raw_results: tuple[envelope_kernel.RawCoverageResultV1, ...]
    interaction_results: tuple[envelope_kernel.InteractionResolutionResultV1, ...]
    debug_scene: envelope_kernel.EnvelopeDebugSceneV1 | None
    diagnostics: tuple[EnvelopeDebugHostDiagnosticV1, ...]
    timings: tuple[EnvelopeDebugTimingV1, ...]


class EnvelopeHostAdapterError(RuntimeError):
    def __init__(
        self,
        outcome: EnvelopeDebugHostOutcome,
        message: str,
        *,
        patch_domain_id: str | None = None,
    ):
        self.outcome = outcome
        self.patch_domain_id = patch_domain_id
        super().__init__(message)

    def diagnostic(self) -> EnvelopeDebugHostDiagnosticV1:
        return EnvelopeDebugHostDiagnosticV1(
            self.outcome,
            EnvelopeDebugHostSeverity.UNSUPPORTED,
            str(self),
            self.patch_domain_id,
        )


@dataclass(frozen=True, slots=True)
class _HostChainRecord:
    patch_id: int
    loop_index: int
    chain_index: int
    chain: object
    canonical_vertex_ids: tuple[int, ...]
    canonical_edge_ids: tuple[int, ...]
    reversed_from_canonical: bool


def _load_kernel():
    try:
        kernel = importlib.import_module("cftuv_envelope")
    except ModuleNotFoundError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_KERNEL_UNAVAILABLE,
            f"cftuv-envelope-core is unavailable: {exc}",
        ) from exc
    try:
        sympy = importlib.import_module("sympy")
    except ModuleNotFoundError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_KERNEL_UNAVAILABLE,
            f"SymPy is unavailable: {exc}",
        ) from exc
    if sympy.__version__ != "1.14.0":
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SYMPY_VERSION_UNSUPPORTED,
            f"exact Envelope debug requires sympy==1.14.0, found {sympy.__version__}",
        )
    return kernel, sympy


def _stable_token(kind: str, revision: str, *parts: object) -> str:
    payload = json.dumps(
        (kind, revision, parts),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:24]


def _typed_value(kind: str, revision: str, *parts: object) -> str:
    return f"host-v0:{kind}:{_stable_token(kind, revision, *parts)}"


def _revision_value(source_revision) -> str:
    source_name = str(source_revision.source_name).strip()
    digest = str(source_revision.digest).strip()
    if not source_name or not digest:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "AnalysisBundle SourceRevision requires non-empty source_name and digest",
        )
    return f"host-source:{digest}:{source_name}"


def _vector3(value) -> tuple[float, float, float]:
    result = tuple(float(item) for item in value)
    if len(result) != 3 or not all(math.isfinite(item) for item in result):
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "host vector must contain three finite components",
        )
    return result


def _canonical_chain(chain) -> tuple[tuple[int, ...], tuple[int, ...], bool]:
    vertices = tuple(int(item) for item in chain.vert_indices)
    edges = tuple(int(item) for item in chain.edge_indices)
    is_closed = bool(chain.is_closed)
    if is_closed and len(vertices) > 1 and vertices[0] == vertices[-1]:
        vertices = vertices[:-1]
    expected_edges = len(vertices) if is_closed else len(vertices) - 1
    if len(vertices) < 2 or len(edges) != expected_edges:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
            "host BoundaryChain does not satisfy open E=V-1 or closed E=V",
        )
    if is_closed:
        candidates = []
        for start in range(len(vertices)):
            forward_vertices = vertices[start:] + vertices[:start]
            forward_edges = edges[start:] + edges[:start]
            candidates.append(
                ((forward_edges, forward_vertices), forward_vertices, forward_edges, False)
            )
            reverse_vertices = tuple(
                vertices[(start - offset) % len(vertices)]
                for offset in range(len(vertices))
            )
            reverse_edges = tuple(
                edges[(start - offset - 1) % len(edges)]
                for offset in range(len(edges))
            )
            candidates.append(
                ((reverse_edges, reverse_vertices), reverse_vertices, reverse_edges, True)
            )
        _, canonical_vertices, canonical_edges, reversed_from_canonical = min(
            candidates, key=lambda item: item[0]
        )
        return canonical_vertices, canonical_edges, reversed_from_canonical
    forward = (edges, vertices)
    reverse = (tuple(reversed(edges)), tuple(reversed(vertices)))
    if reverse < forward:
        return reverse[1], reverse[0], True
    return vertices, edges, False


def _collect_host_chains(analysis_bundle: AnalysisBundle) -> tuple[_HostChainRecord, ...]:
    records = []
    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        for loop_index, loop in enumerate(patch.boundary_loops):
            for chain_index, chain in enumerate(loop.chains):
                vertices, edges, reversed_from_canonical = _canonical_chain(chain)
                records.append(
                    _HostChainRecord(
                        int(patch_id),
                        loop_index,
                        chain_index,
                        chain,
                        vertices,
                        edges,
                        reversed_from_canonical,
                    )
                )
    if not records:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "AnalysisBundle contains no BoundaryChain records",
        )
    return tuple(records)


def _shape_class(kernel, patch) -> object:
    value = getattr(patch, "shape_class", None)
    if value is None:
        # PatchNode v1 has no shape-class field. MIX is the declared host
        # contract for that schema, not a geometry-repair fallback.
        return kernel.PatchShapeClass.MIX
    text = value.value if hasattr(value, "value") else str(value)
    try:
        return kernel.PatchShapeClass(text)
    except ValueError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            f"unsupported host Patch shape_class: {text}",
        ) from exc


def _context_tags(kernel, patch) -> frozenset:
    mapping = {
        PatchType.WALL: kernel.PatchContextTag.WALL,
        PatchType.FLOOR: kernel.PatchContextTag.FLOOR,
        PatchType.SLOPE: kernel.PatchContextTag.SLOPE,
    }
    patch_type = patch.patch_type
    if not isinstance(patch_type, PatchType):
        try:
            patch_type = PatchType(str(getattr(patch_type, "value", patch_type)))
        except ValueError as exc:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
                f"unsupported host PatchType: {patch_type}",
            ) from exc
    return frozenset({mapping[patch_type]})


def _source_ids(kernel, revision: str, analysis_bundle: AnalysisBundle):
    vertex_ids = {
        int(item.vertex_id): kernel.SourceVertexId(
            f"host-vertex:{revision}:{int(item.vertex_id)}"
        )
        for item in analysis_bundle.patch_surface.vertices
    }
    edge_ids = {
        int(item.edge_id): kernel.PhysicalEdgeId(
            f"host-edge:{revision}:{int(item.edge_id)}"
        )
        for item in analysis_bundle.patch_surface.edges
    }
    face_ids = {
        int(item.face_id): kernel.SourceFaceId(
            f"host-face:{revision}:{int(item.face_id)}"
        )
        for item in analysis_bundle.patch_surface.faces
    }
    triangle_ids = {
        int(item.triangle_id): kernel.SurfaceTriangleId(
            f"host-triangle:{revision}:{int(item.triangle_id)}"
        )
        for item in analysis_bundle.patch_surface.triangles
    }
    patch_ids = {
        int(patch_id): kernel.PatchId(f"host-patch:{revision}:{int(patch_id)}")
        for patch_id in analysis_bundle.patch_graph.nodes
    }
    return vertex_ids, edge_ids, face_ids, triangle_ids, patch_ids


def _exact_frame(
    kernel,
    sympy,
    *,
    revision: str,
    patch_id: int,
    patch,
    patch_domain_id,
    patch_vertex_ids: tuple[int, ...],
    host_vertex_by_id: dict[int, object],
    kernel_vertex_ids: dict[int, object],
):
    scalar = lambda value: sympy.Rational(str(float(value)))
    if not patch_vertex_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            "PatchDomain has no source vertices",
            patch_domain_id=patch_domain_id.value,
        )
    origin_values = _vector3(host_vertex_by_id[min(patch_vertex_ids)].position)
    axis_u_values = _vector3(patch.basis_u)
    axis_v_values = _vector3(patch.basis_v)
    normal_values = _vector3(patch.normal)
    origin = tuple(scalar(value) for value in origin_values)
    axis_u = tuple(scalar(value) for value in axis_u_values)
    axis_v = tuple(scalar(value) for value in axis_v_values)
    normal = tuple(scalar(value) for value in normal_values)

    def dot(left, right):
        return sympy.factor(sum(a * b for a, b in zip(left, right, strict=True)))

    def cross(left, right):
        return (
            sympy.factor(left[1] * right[2] - left[2] * right[1]),
            sympy.factor(left[2] * right[0] - left[0] * right[2]),
            sympy.factor(left[0] * right[1] - left[1] * right[0]),
        )

    basis_checks = (
        dot(axis_u, axis_u) - 1,
        dot(axis_v, axis_v) - 1,
        dot(normal, normal) - 1,
        dot(axis_u, axis_v),
        dot(axis_u, normal),
        dot(axis_v, normal),
    )
    if any(item != 0 for item in basis_checks):
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            "host planar basis is not exactly orthonormal",
            patch_domain_id=patch_domain_id.value,
        )
    cross_uv = cross(axis_u, axis_v)
    if cross_uv == normal:
        handedness = kernel.PatchFrameHandedness.RIGHT_HANDED_U_V_NORMAL
    elif cross_uv == tuple(-item for item in normal):
        handedness = kernel.PatchFrameHandedness.LEFT_HANDED_U_V_NORMAL
    else:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            "host planar basis handedness is not exactly certified",
            patch_domain_id=patch_domain_id.value,
        )

    coordinates = []
    for vertex_id in patch_vertex_ids:
        position_values = _vector3(host_vertex_by_id[vertex_id].position)
        position = tuple(scalar(value) for value in position_values)
        relative = tuple(
            value - base for value, base in zip(position, origin, strict=True)
        )
        if dot(relative, normal) != 0:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                f"source vertex {vertex_id} is not exactly coplanar",
                patch_domain_id=patch_domain_id.value,
            )
        exact_x = sympy.factor(dot(relative, axis_u))
        exact_y = sympy.factor(dot(relative, axis_v))
        float_x = float(exact_x)
        float_y = float(exact_y)
        if (
            not math.isfinite(float_x)
            or not math.isfinite(float_y)
            or scalar(float_x) != exact_x
            or scalar(float_y) != exact_y
        ):
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                f"source vertex {vertex_id} planar coordinate has no exact float round-trip",
                patch_domain_id=patch_domain_id.value,
            )
        coordinates.append(
            kernel.PlanarSourceVertexCoordinateV1(
                kernel_vertex_ids[vertex_id],
                kernel.LocalPoint2V1(float_x, float_y),
            )
        )
    certificate = kernel.PlanarityCertificateV1(
        kernel.PlanarityCertificateId(
            _typed_value("planarity", revision, patch_id)
        ),
        patch_domain_id,
        kernel.PlanarityLaw.ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1,
        True,
        kernel.LocalLengthV1(Decimal(0)),
        kernel.SurfaceMetricAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
    )
    return kernel.PlanarPatchFrameV1(
        patch_domain_id,
        kernel.LocalPoint3V1(*origin_values),
        kernel.LocalVector3V1(*axis_u_values),
        kernel.LocalVector3V1(*axis_v_values),
        kernel.LocalVector3V1(*normal_values),
        handedness,
        certificate,
        kernel.PlanarProjectionLaw.DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1,
        frozenset(coordinates),
    )


def _decimal_interval_from_rational(
    kernel,
    sympy,
    value,
    *,
    patch_domain_id: str,
):
    value = sympy.factor(value)
    if value.is_Rational is not True:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
            f"angular ratio is not an exact rational multiple of pi: {value}",
            patch_domain_id=patch_domain_id,
        )
    numerator = int(value.p)
    denominator = int(value.q)
    scale = 10**28
    lower_units = numerator * scale // denominator
    upper_units = -((-numerator * scale) // denominator)
    lower = Decimal(lower_units).scaleb(-28)
    upper = Decimal(upper_units).scaleb(-28)
    interval_kind = kernel.IntervalEndpointKind.CLOSED
    return kernel.CertifiedDecimalIntervalV1(
        lower,
        upper,
        interval_kind,
        interval_kind,
        upper - lower,
    )


def _build_angular_relations(
    kernel,
    sympy,
    *,
    analysis_bundle,
    revision: str,
    patch_ids: dict[int, object],
    patch_domains: dict[int, object],
    frames: dict[int, object],
    use_id_by_ref: dict[tuple[int, int, int], object],
    sector_id_by_ref: dict[tuple[int, int, int], object],
    launch_id_by_ref: dict[tuple[int, int, int], object],
    record_by_ref: dict[tuple[int, int, int], _HostChainRecord],
    vertex_ids: dict[int, object],
):
    scalar = lambda value: sympy.Rational(str(float(value)))

    def point_map(frame):
        return {
            item.source_vertex_id: (
                scalar(item.domain_coordinate.x),
                scalar(item.domain_coordinate.y),
            )
            for item in frame.source_vertex_coordinates
        }

    def cross(left, right):
        return sympy.factor(left[0] * right[1] - left[1] * right[0])

    def dot(left, right):
        return sympy.factor(left[0] * right[0] + left[1] * right[1])

    angular_sectors = []
    angle_certificates = []
    corner_relations = []
    used_sector_ids = set()
    host_face_by_patch = {
        int(patch_id): analysis_bundle.patch_surface.patch_faces(int(patch_id))
        for patch_id in analysis_bundle.patch_graph.nodes
    }

    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        patch_id = int(patch_id)
        domain_id = patch_domains[patch_id]
        frame = frames[patch_id]
        coordinates = point_map(frame)
        kernel_vertex_for_host = vertex_ids

        def owner_normal(record: _HostChainRecord, anchor_vertex_id: int):
            host_vertices = tuple(int(item) for item in record.chain.vert_indices)
            if anchor_vertex_id == host_vertices[0]:
                start_vertex_id, end_vertex_id = host_vertices[0], host_vertices[1]
                physical_edge_id = int(record.chain.edge_indices[0])
            elif anchor_vertex_id == host_vertices[-1]:
                start_vertex_id, end_vertex_id = host_vertices[-2], host_vertices[-1]
                physical_edge_id = int(record.chain.edge_indices[-1])
            else:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner anchor is not a physical ChainUse endpoint",
                    patch_domain_id=domain_id.value,
                )
            start = coordinates[kernel_vertex_for_host[start_vertex_id]]
            end = coordinates[kernel_vertex_for_host[end_vertex_id]]
            tangent = (end[0] - start[0], end[1] - start[1])
            if dot(tangent, tangent) == 0:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner incident support is degenerate",
                    patch_domain_id=domain_id.value,
                )
            candidates = []
            for face in host_face_by_patch[patch_id]:
                if physical_edge_id not in face.edge_cycle:
                    continue
                ordinal = face.edge_cycle.index(physical_edge_id)
                cycle = tuple(
                    coordinates[kernel_vertex_for_host[int(item)]]
                    for item in face.vertex_cycle
                )
                twice_area = sum(
                    cycle[index][0] * cycle[(index + 1) % len(cycle)][1]
                    - cycle[index][1] * cycle[(index + 1) % len(cycle)][0]
                    for index in range(len(cycle))
                )
                if twice_area == 0:
                    continue
                face_start = cycle[ordinal]
                face_end = cycle[(ordinal + 1) % len(cycle)]
                face_direction = (
                    face_end[0] - face_start[0],
                    face_end[1] - face_start[1],
                )
                same_direction = dot(tangent, face_direction) > 0
                interior_is_left = (twice_area > 0) == same_direction
                candidates.append(
                    (-tangent[1], tangent[0])
                    if interior_is_left
                    else (tangent[1], -tangent[0])
                )
            if not candidates:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner support has no exact owner-face direction",
                    patch_domain_id=domain_id.value,
                )
            first = candidates[0]
            if any(cross(first, item) != 0 or dot(first, item) <= 0 for item in candidates[1:]):
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner owner-face directions disagree",
                    patch_domain_id=domain_id.value,
                )
            return first

        for loop_index, loop in enumerate(patch.boundary_loops):
            for corner_index, corner in enumerate(loop.corners):
                try:
                    host_phi = sympy.Rational(str(float(corner.turn_angle_deg))) / 180
                except (TypeError, ValueError):
                    continue
                if host_phi <= 1:
                    continue
                if host_phi >= 2:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "exact 2*pi corner must be Terminal/Junction, not Angular",
                        patch_domain_id=domain_id.value,
                    )
                prev_ref = (
                    patch_id,
                    loop_index,
                    int(corner.prev_chain_index),
                )
                next_ref = (
                    patch_id,
                    loop_index,
                    int(corner.next_chain_index),
                )
                if prev_ref not in record_by_ref or next_ref not in record_by_ref:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "BoundaryCorner incident ChainUse references are incomplete",
                        patch_domain_id=domain_id.value,
                    )
                anchor_vertex_id = int(corner.vert_index)
                if anchor_vertex_id not in vertex_ids:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "BoundaryCorner source vertex is absent from PatchSurfaceIR",
                        patch_domain_id=domain_id.value,
                    )
                incoming_normal = owner_normal(
                    record_by_ref[prev_ref], anchor_vertex_id
                )
                outgoing_normal = owner_normal(
                    record_by_ref[next_ref], anchor_vertex_id
                )
                turn_cross = cross(incoming_normal, outgoing_normal)
                if turn_cross == 0:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "BoundaryCorner incident supports are parallel",
                        patch_domain_id=domain_id.value,
                    )
                turn_dot = dot(incoming_normal, outgoing_normal)
                delta = sympy.factor(
                    sympy.atan2(abs(turn_cross), turn_dot) / sympy.pi
                )
                if sympy.factor(host_phi - (1 + delta)) != 0:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "BoundaryCorner turn_angle_deg disagrees with exact support directions",
                        patch_domain_id=domain_id.value,
                    )
                interval_phi = _decimal_interval_from_rational(
                    kernel,
                    sympy,
                    host_phi,
                    patch_domain_id=domain_id.value,
                )
                interval_delta = kernel.CertifiedDecimalIntervalV1(
                    interval_phi.lower - Decimal(1),
                    interval_phi.upper - Decimal(1),
                    interval_phi.lower_kind,
                    interval_phi.upper_kind,
                    interval_phi.absolute_error_bound,
                )
                exact_delta_interval = _decimal_interval_from_rational(
                    kernel,
                    sympy,
                    delta,
                    patch_domain_id=domain_id.value,
                )
                if (
                    interval_delta.lower != exact_delta_interval.lower
                    or interval_delta.upper != exact_delta_interval.upper
                ):
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "phi and reflex-excess rational intervals disagree",
                        patch_domain_id=domain_id.value,
                    )
                orientation = (
                    kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
                    if turn_cross > 0
                    else kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
                )
                owner_sector_id = sector_id_by_ref[prev_ref]
                if owner_sector_id in used_sector_ids:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED,
                        "V0 cannot encode two Angular relations through one owner-sector record",
                        patch_domain_id=domain_id.value,
                    )
                used_sector_ids.add(owner_sector_id)

                def exact_float(value):
                    result = float(value)
                    if scalar(result) != value:
                        raise EnvelopeHostAdapterError(
                            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                            "support direction has no exact float round-trip",
                            patch_domain_id=domain_id.value,
                        )
                    return result

                incoming_use_id = use_id_by_ref[prev_ref]
                outgoing_use_id = use_id_by_ref[next_ref]
                angular_sectors.append(
                    kernel.OrientedOwnerSectorV1(
                        owner_sector_id,
                        patch_ids[patch_id],
                        domain_id,
                        True,
                        (incoming_use_id, outgoing_use_id),
                        kernel.SourceSupportRefV1(
                            incoming_use_id,
                            launch_id_by_ref[prev_ref],
                            kernel.CertifiedPlanarSupportDirectionV1(
                                kernel.LocalVector2V1(
                                    exact_float(incoming_normal[0]),
                                    exact_float(incoming_normal[1]),
                                ),
                                kernel.SupportDirectionAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
                            ),
                        ),
                        kernel.SourceSupportRefV1(
                            outgoing_use_id,
                            launch_id_by_ref[next_ref],
                            kernel.CertifiedPlanarSupportDirectionV1(
                                kernel.LocalVector2V1(
                                    exact_float(outgoing_normal[0]),
                                    exact_float(outgoing_normal[1]),
                                ),
                                kernel.SupportDirectionAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
                            ),
                        ),
                        orientation,
                        kernel.InteriorSelectionLaw.OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS,
                    )
                )
                angle_id = kernel.AngleCertificateId(
                    _typed_value(
                        "angle-certificate",
                        revision,
                        patch_id,
                        loop_index,
                        corner_index,
                    )
                )
                angle_certificates.append(
                    kernel.ReflexAngleCertificateV1(
                        angle_id,
                        owner_sector_id,
                        kernel.AngleMeasureLaw.ORIENTED_OWNER_SECTOR_ANGLE,
                        kernel.AngleMeasureSource.HOST_ANALYSIS_EXACT_OR_CERTIFIED,
                        kernel.StrictAngleRangeCertificate.STRICT_PI_LT_PHI_LT_2PI,
                        kernel.ReflexExcessLaw.DELTA_EQUALS_PHI_MINUS_PI,
                        kernel.CertifiedReflexAngleMeasureV1(
                            interval_phi,
                            interval_delta,
                            orientation,
                            kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
                        ),
                        False,
                    )
                )
                corner_relations.append(
                    kernel.CornerRelationV1(
                        kernel.CornerRelationId(
                            _typed_value(
                                "corner-relation",
                                revision,
                                patch_id,
                                loop_index,
                                corner_index,
                            )
                        ),
                        vertex_ids[anchor_vertex_id],
                        owner_sector_id,
                        angle_id,
                        False,
                    )
                )
    return (
        frozenset(angular_sectors),
        frozenset(angle_certificates),
        frozenset(corner_relations),
    )


def build_envelope_analysis_snapshot(
    analysis_bundle: AnalysisBundle,
) -> envelope_kernel.AnalysisSnapshotV1:
    """Map one real host AnalysisBundle without geometry repair or fallback."""

    kernel, sympy = _load_kernel()
    analysis_bundle.capabilities.require_supported()
    revision = _revision_value(analysis_bundle.source_revision)
    source_revision = kernel.SourceRevision(revision)
    (
        vertex_ids,
        edge_ids,
        face_ids,
        triangle_ids,
        patch_ids,
    ) = _source_ids(kernel, revision, analysis_bundle)
    host_vertex_by_id = analysis_bundle.patch_surface.vertex_by_id

    source_vertices = frozenset(
        kernel.SourceVertexV1(
            vertex_ids[int(item.vertex_id)],
            kernel.LocalPoint3V1(*_vector3(item.position)),
        )
        for item in analysis_bundle.patch_surface.vertices
    )
    source_edges = frozenset(
        kernel.SourceEdgeV1(
            edge_ids[int(item.edge_id)],
            vertex_ids[int(item.vertex_ids[0])],
            vertex_ids[int(item.vertex_ids[1])],
        )
        for item in analysis_bundle.patch_surface.edges
    )
    source_faces = frozenset(
        kernel.SourceFaceV1(
            face_ids[int(item.face_id)],
            patch_ids[int(item.patch_id)],
            tuple(vertex_ids[int(value)] for value in item.vertex_cycle),
            tuple(edge_ids[int(value)] for value in item.edge_cycle),
            kernel.LocalVector3V1(*_vector3(item.polygon_normal)),
            tuple(triangle_ids[int(value)] for value in item.triangle_ids),
        )
        for item in analysis_bundle.patch_surface.faces
    )
    surface_triangles = frozenset(
        kernel.SurfaceTriangleV1(
            triangle_ids[int(item.triangle_id)],
            face_ids[int(item.source_face_id)],
            tuple(vertex_ids[int(value)] for value in item.vertex_ids),
            tuple(
                edge_ids[int(value)] if value is not None else None
                for value in (
                    item.physical_edge_ids[2],
                    item.physical_edge_ids[0],
                    item.physical_edge_ids[1],
                )
            ),
            kernel.LocalVector3V1(*_vector3(item.triangle_normal)),
        )
        for item in analysis_bundle.patch_surface.triangles
    )
    surface_ir = kernel.PatchSurfaceIRV1(
        "cftuv.envelope.patch_surface_ir.v1",
        source_revision,
        kernel.SurfacePayloadMode.FULL_HOST_SURFACE,
        frozenset(vertex_ids.values()),
        source_edges,
        source_faces,
        surface_triangles,
    )

    host_chains = _collect_host_chains(analysis_bundle)
    chain_groups: dict[tuple[bool, tuple[int, ...], tuple[int, ...]], list] = {}
    for record in host_chains:
        key = (
            bool(record.chain.is_closed),
            record.canonical_edge_ids,
            record.canonical_vertex_ids,
        )
        chain_groups.setdefault(key, []).append(record)

    physical_chains = []
    chain_id_by_key = {}
    use_id_by_ref = {}
    for key, records in sorted(chain_groups.items(), key=lambda item: item[0]):
        is_closed, canonical_edges, canonical_vertices = key
        neighbor_kinds = {record.chain.neighbor_kind for record in records}
        patch_group = {record.patch_id for record in records}
        if len(neighbor_kinds) != 1:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
                "one canonical PhysicalChain has conflicting host neighbor kinds",
            )
        neighbor_kind = next(iter(neighbor_kinds))
        if neighbor_kind is ChainNeighborKind.SEAM_SELF:
            if len(records) != 2 or len(patch_group) != 1:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE,
                    "SEAM_SELF requires exactly two host ChainUses in one PatchDomain",
                )
            chain_kind = kernel.PhysicalChainKind.SEAM_SELF
        elif neighbor_kind is ChainNeighborKind.PATCH:
            if len(records) != 2 or len(patch_group) != 2:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE,
                    "physical seam requires two exact whole-chain patch-side uses",
                )
            chain_kind = kernel.PhysicalChainKind.PHYSICAL_SEAM
        else:
            if len(records) != 1:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
                    "ordinary host boundary source requires exactly one ChainUse",
                )
            chain_kind = kernel.PhysicalChainKind.PHYSICAL_DECAL_SOURCE
        chain_id = kernel.PhysicalChainId(
            _typed_value(
                "physical-chain",
                revision,
                is_closed,
                canonical_edges,
                canonical_vertices,
            )
        )
        chain_id_by_key[key] = chain_id
        lineage_id = kernel.LineageId(
            _typed_value("physical-lineage", revision, canonical_edges)
        )
        physical_chains.append(
            kernel.PhysicalChainV1(
                chain_id,
                chain_kind,
                is_closed,
                tuple(vertex_ids[item] for item in canonical_vertices),
                tuple(edge_ids[item] for item in canonical_edges),
                frozenset({lineage_id}),
                frozenset(
                    kernel.LineageId(
                        _typed_value(
                            "chain-record",
                            revision,
                            item.patch_id,
                            item.loop_index,
                            item.chain_index,
                        )
                    )
                    for item in records
                ),
            )
        )
        for record in records:
            use_id_by_ref[
                (record.patch_id, record.loop_index, record.chain_index)
            ] = kernel.ChainUseId(
                _typed_value(
                    "chain-use",
                    revision,
                    record.patch_id,
                    record.loop_index,
                    record.chain_index,
                    canonical_edges,
                )
            )

    patch_domains = {}
    loop_ids = {}
    boundary_loops = []
    boundary_constraints = []
    chain_uses = []
    sector_id_by_ref = {}
    launch_id_by_ref = {}
    sectors_by_patch: dict[int, list] = {
        int(patch_id): [] for patch_id in analysis_bundle.patch_graph.nodes
    }
    terminal_relations = []
    domain_constraint_ids: dict[int, set] = {
        int(patch_id): set() for patch_id in analysis_bundle.patch_graph.nodes
    }

    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        patch_id = int(patch_id)
        domain_id = kernel.PatchDomainId(
            _typed_value("patch-domain", revision, patch_id)
        )
        patch_domains[patch_id] = domain_id
        for loop_index, loop in enumerate(patch.boundary_loops):
            loop_id = kernel.BoundaryLoopId(
                _typed_value("boundary-loop", revision, patch_id, loop_index)
            )
            loop_ids[(patch_id, loop_index)] = loop_id
            ordered_uses = tuple(
                use_id_by_ref[(patch_id, loop_index, chain_index)]
                for chain_index in range(len(loop.chains))
            )
            boundary_loops.append(
                kernel.BoundaryLoopV1(
                    loop_id,
                    domain_id,
                    kernel.BoundaryLoopKind(loop.kind.value),
                    ordered_uses,
                )
            )
            topology_constraint_id = kernel.BoundaryConstraintId(
                _typed_value(
                    "topological-boundary",
                    revision,
                    patch_id,
                    loop_index,
                )
            )
            boundary_constraints.append(
                kernel.BoundaryConstraintV1(
                    topology_constraint_id,
                    domain_id,
                    kernel.BoundaryConstraintKind.TOPOLOGICAL_BOUNDARY_USE,
                    kernel.BoundaryLoopConstraintTargetV1(loop_id),
                    None,
                    False,
                    False,
                )
            )
            domain_constraint_ids[patch_id].add(topology_constraint_id)

    record_by_ref = {
        (item.patch_id, item.loop_index, item.chain_index): item
        for item in host_chains
    }
    for ref, use_id in sorted(use_id_by_ref.items()):
        patch_id, loop_index, chain_index = ref
        record = record_by_ref[ref]
        key = (
            bool(record.chain.is_closed),
            record.canonical_edge_ids,
            record.canonical_vertex_ids,
        )
        physical_chain_id = chain_id_by_key[key]
        domain_id = patch_domains[patch_id]
        launch_id = kernel.BoundaryConstraintId(
            _typed_value("source-launch", revision, *ref)
        )
        launch_id_by_ref[ref] = launch_id
        launch = kernel.LaunchLocusV1(
            launch_id,
            kernel.OwnerInteriorDirection.OWNER_INTERIOR,
            True,
        )
        roles = {
            kernel.ChainUseRole.TOPOLOGICAL_BOUNDARY_USE,
            kernel.ChainUseRole.DOMAIN_BOUNDARY,
        }
        chain_uses.append(
            kernel.ChainUseV1(
                use_id,
                physical_chain_id,
                patch_ids[patch_id],
                domain_id,
                loop_ids[(patch_id, loop_index)],
                kernel.ChainUseOrientation.B_START_TO_END
                if record.reversed_from_canonical
                else kernel.ChainUseOrientation.A_START_TO_END,
                frozenset(roles),
                launch,
            )
        )
        boundary_constraints.append(
            kernel.BoundaryConstraintV1(
                launch_id,
                domain_id,
                kernel.BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
                kernel.ChainUseConstraintTargetV1(use_id),
                use_id,
                False,
                True,
            )
        )
        domain_constraint_ids[patch_id].add(launch_id)
        sector_id = kernel.OwnerSectorId(
            _typed_value("owner-sector", revision, *ref)
        )
        sector_id_by_ref[ref] = sector_id
        sectors_by_patch[patch_id].append(
            kernel.PatchSectorV1(
                sector_id,
                use_id,
                True,
                kernel.LawId("HOST_BOUNDARY_CHAIN_OWNER_INTERIOR_V1"),
            )
        )
        if not record.chain.is_closed:
            lineage = next(
                item.source_lineage
                for item in physical_chains
                if item.physical_chain_id == physical_chain_id
            )
            for role, vertex_id in (
                (
                    kernel.TerminalEndpointRole.START,
                    record.canonical_vertex_ids[0],
                ),
                (
                    kernel.TerminalEndpointRole.END,
                    record.canonical_vertex_ids[-1],
                ),
            ):
                terminal_relations.append(
                    kernel.TerminalRelationV1(
                        kernel.TerminalRelationId(
                            _typed_value(
                                "terminal",
                                revision,
                                *ref,
                                role.value,
                            )
                        ),
                        vertex_ids[vertex_id],
                        use_id,
                        patch_ids[patch_id],
                        domain_id,
                        kernel.TerminalRelationKind.PHYSICAL_CHAIN_ENDPOINT,
                        role,
                        sector_id,
                        lineage,
                    )
                )

    patch_descriptors = []
    metric_descriptors = []
    frames = {}
    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        patch_id = int(patch_id)
        if patch_id not in patch_ids:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
                f"PatchGraph patch {patch_id} has no PatchSurfaceIR identity",
            )
        patch_descriptors.append(
            kernel.PatchDescriptorV1(
                patch_ids[patch_id],
                kernel.SurfaceRegime.PLANAR,
                _shape_class(kernel, patch),
                _context_tags(kernel, patch),
            )
        )
        patch_face_records = analysis_bundle.patch_surface.patch_faces(patch_id)
        patch_vertex_ids = tuple(
            sorted(
                {
                    int(vertex_id)
                    for face in patch_face_records
                    for vertex_id in face.vertex_cycle
                }
            )
        )
        frame = _exact_frame(
            kernel,
            sympy,
            revision=revision,
            patch_id=patch_id,
            patch=patch,
            patch_domain_id=patch_domains[patch_id],
            patch_vertex_ids=patch_vertex_ids,
            host_vertex_by_id=host_vertex_by_id,
            kernel_vertex_ids=vertex_ids,
        )
        frames[patch_id] = frame
        metric_descriptors.append(frame)

    (
        angular_owner_sectors,
        reflex_angle_certificates,
        corner_relations,
    ) = _build_angular_relations(
        kernel,
        sympy,
        analysis_bundle=analysis_bundle,
        revision=revision,
        patch_ids=patch_ids,
        patch_domains=patch_domains,
        frames=frames,
        use_id_by_ref=use_id_by_ref,
        sector_id_by_ref=sector_id_by_ref,
        launch_id_by_ref=launch_id_by_ref,
        record_by_ref=record_by_ref,
        vertex_ids=vertex_ids,
    )

    domain_records = frozenset(
        kernel.PatchDomainV1(
            patch_domains[patch_id],
            patch_ids[patch_id],
            kernel.SurfaceRegime.PLANAR,
            frozenset(domain_constraint_ids[patch_id]),
            frozenset(sectors_by_patch[patch_id]),
        )
        for patch_id in sorted(patch_domains)
    )
    snapshot = kernel.AnalysisSnapshotV1(
        kernel.ANALYSIS_SNAPSHOT_SCHEMA_V1,
        source_revision,
        frozenset(
            {
                kernel.AnalysisCapability.PATCH_DOMAIN_V1,
                kernel.AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
                kernel.AnalysisCapability.PATCH_SURFACE_IR_V1,
                kernel.AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
                kernel.AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
                kernel.AnalysisCapability.RELATION_LINEAGE_V1,
                kernel.AnalysisCapability.AUTHORITATIVE_SURFACE_METRIC_V1,
                kernel.AnalysisCapability.PHYSICAL_EDGE_TABLE_V1,
                kernel.AnalysisCapability.BOUNDARY_CONSTRAINT_TARGET_V1,
                kernel.AnalysisCapability.TYPED_JUNCTION_ROUTE_TOPOLOGY_V1,
                kernel.AnalysisCapability.TERMINAL_RELATION_V1,
            }
        ),
        frozenset(patch_descriptors),
        domain_records,
        frozenset(metric_descriptors),
        surface_ir,
        source_vertices,
        frozenset(physical_chains),
        frozenset(chain_uses),
        frozenset(boundary_loops),
        frozenset(boundary_constraints),
        angular_owner_sectors,
        reflex_angle_certificates,
        corner_relations,
        frozenset(),
        frozenset(terminal_relations),
    )
    issues = kernel.validate_analysis_snapshot(snapshot)
    if issues:
        message = "; ".join(
            f"{item.code.value}:{'.'.join(item.path)}:{item.message}"
            for item in issues
        )
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            message,
        )
    validation = importlib.import_module(
        "cftuv_envelope.reference.validation"
    )
    for domain_id in sorted(
        (item.patch_domain_id for item in snapshot.patch_domains),
        key=lambda item: item.value,
    ):
        _, diagnostics = validation.validate_reference_geometry_payload(
            snapshot, domain_id
        )
        if diagnostics:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                "; ".join(
                    f"{item.outcome.value}:{item.message}" for item in diagnostics
                ),
                patch_domain_id=domain_id.value,
            )
    return snapshot


def _host_edge_number(edge_id) -> int:
    try:
        return int(edge_id.value.rsplit(":", 1)[1])
    except (AttributeError, ValueError) as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            f"PhysicalEdgeId is not a V0 host identity: {edge_id}",
        ) from exc


def build_envelope_decal_request(
    snapshot: envelope_kernel.AnalysisSnapshotV1,
    selected_physical_edge_ids: frozenset[int],
    alpha: float,
) -> envelope_kernel.DecalRequestV1:
    """Compile whole-chain selection into one immutable debug request."""

    kernel, _ = _load_kernel()
    if not selected_physical_edge_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "Envelope debug requires at least one selected physical edge",
        )
    selected_edges = frozenset(int(item) for item in selected_physical_edge_ids)
    known_edges = {
        _host_edge_number(item.edge_id) for item in snapshot.surface_ir.source_edges
    }
    unknown = selected_edges - known_edges
    if unknown:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN,
            f"selected physical edges are absent from AnalysisBundle: {sorted(unknown)}",
        )
    selected_chain_ids = set()
    for chain in snapshot.physical_chains:
        chain_edges = frozenset(
            _host_edge_number(item) for item in chain.ordered_physical_edge_ids
        )
        overlap = selected_edges & chain_edges
        if not overlap:
            continue
        if overlap != chain_edges:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED,
                "V0 accepts only complete PhysicalChain selection; "
                f"selected={sorted(overlap)} chain={sorted(chain_edges)}",
            )
        selected_chain_ids.add(chain.physical_chain_id)
    covered = {
        _host_edge_number(edge_id)
        for chain in snapshot.physical_chains
        if chain.physical_chain_id in selected_chain_ids
        for edge_id in chain.ordered_physical_edge_ids
    }
    if covered != selected_edges:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED,
            "selected edges do not resolve to an exact set of whole PhysicalChains",
        )
    selected_use_ids = frozenset(
        item.chain_use_id
        for item in snapshot.chain_uses
        if item.physical_chain_id in selected_chain_ids
    )
    if not selected_use_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "whole-chain selection resolved to no ChainUse",
        )
    try:
        alpha_decimal = Decimal(str(float(alpha)))
    except (InvalidOperation, ValueError) as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
            f"requested alpha is invalid: {alpha}",
        ) from exc
    if not alpha_decimal.is_finite() or alpha_decimal < 0:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
            f"requested alpha must be finite and non-negative: {alpha}",
        )
    request_id = kernel.DecalRequestId(
        _typed_value(
            "decal-request",
            snapshot.source_revision.value,
            tuple(sorted(item.value for item in selected_chain_ids)),
        )
    )
    request = kernel.DecalRequestV1(
        schema_version=kernel.DECAL_REQUEST_SCHEMA_V1,
        decal_request_id=request_id,
        selected_chain_use_ids=selected_use_ids,
        requested_alpha=kernel.LocalLengthV1(alpha_decimal),
        metric_space=kernel.MetricSpace.SOURCE_LOCAL_INTRINSIC,
        angular_profile_family_id=kernel.AngularProfileFamilyId.LINEAR_REFLEX_EQUAL_V1,
        angular_profile_selection_policy_id=kernel.AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1,
        max_subturn_parameter_id=kernel.MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1,
        max_subturn_value_id=kernel.MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1,
        max_subturn_exact_value=kernel.ExactAngleV1(
            kernel.ExactAngleSymbol.PI_OVER_3
        ),
        cap_policy_id=kernel.CapPolicyId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1,
        boundary_policy_id=kernel.BoundaryPolicyId.BOUNDARY_LIMITED_PROPAGATION,
        interaction_policy_id=kernel.InteractionPolicyId.INTRAPATCH_POLICY_B_V1,
        ownership_policy_id=kernel.OwnershipPolicyId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1,
        material_policy_id=kernel.PolicyId("ENVELOPE_DEBUG_NO_MATERIAL_V1"),
        uv_policy_id=kernel.PolicyId("ENVELOPE_DEBUG_NO_UV_V1"),
    )
    issues = kernel.validate_decal_request(request)
    issues += kernel.validate_snapshot_request_references(snapshot, request)
    if issues:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "; ".join(
                f"{item.code.value}:{'.'.join(item.path)}:{item.message}"
                for item in issues
            ),
        )
    return request


def _host_diagnostic_from_stage(
    outcome: object,
    message: str,
    patch_domain_id: object | None,
) -> EnvelopeDebugHostDiagnosticV1:
    value = outcome.value if hasattr(outcome, "value") else str(outcome)
    domain_value = (
        patch_domain_id.value
        if patch_domain_id is not None and hasattr(patch_domain_id, "value")
        else None
    )
    return EnvelopeDebugHostDiagnosticV1(
        value,
        EnvelopeDebugHostSeverity.UNSUPPORTED,
        message,
        domain_value,
    )


def _scene_diagnostics(kernel, diagnostics):
    result = []
    for ordinal, diagnostic in enumerate(diagnostics):
        domain_id = (
            kernel.PatchDomainId(diagnostic.patch_domain_id)
            if diagnostic.patch_domain_id is not None
            else None
        )
        outcome = (
            diagnostic.outcome.value
            if hasattr(diagnostic.outcome, "value")
            else str(diagnostic.outcome)
        )
        token = _stable_token(
            "host-diagnostic",
            outcome,
            ordinal,
            diagnostic.patch_domain_id,
            diagnostic.message,
        )
        result.append(
            kernel.DebugDiagnosticV1(
                kernel.DebugDiagnosticId(f"host-debug-diagnostic:{token}"),
                domain_id,
                kernel.EnvelopeDebugStage.DIAGNOSTIC,
                kernel.DebugPrimitiveKind.DIAGNOSTIC,
                outcome,
                kernel.DebugDiagnosticSeverity.UNSUPPORTED,
                diagnostic.message,
                None,
                frozenset(),
                "ENV_80_DIAGNOSTICS",
                outcome,
            )
        )
    return tuple(result)


def evaluate_envelope_debug(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    alpha: float,
) -> EnvelopeDebugEvaluationV1:
    """Run V0-A full recompute and publish only one complete DebugScene."""

    timings = []
    diagnostics: list[EnvelopeDebugHostDiagnosticV1] = []
    started = time.perf_counter()
    try:
        kernel, _ = _load_kernel()
        snapshot = build_envelope_analysis_snapshot(analysis_bundle)
        request = build_envelope_decal_request(
            snapshot, selected_physical_edge_ids, alpha
        )
    except EnvelopeHostAdapterError as exc:
        timings.append(
            EnvelopeDebugTimingV1(
                "HOST_ADAPTER", time.perf_counter() - started
            )
        )
        return EnvelopeDebugEvaluationV1(
            None,
            None,
            (),
            (),
            (),
            None,
            (exc.diagnostic(),),
            tuple(timings),
        )
    except (KeyError, TypeError, ValueError) as exc:
        timings.append(
            EnvelopeDebugTimingV1(
                "HOST_ADAPTER", time.perf_counter() - started
            )
        )
        diagnostic = EnvelopeDebugHostDiagnosticV1(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            EnvelopeDebugHostSeverity.ERROR,
            f"AnalysisBundle cannot satisfy the V0 host contract: {exc}",
        )
        return EnvelopeDebugEvaluationV1(
            None,
            None,
            (),
            (),
            (),
            None,
            (diagnostic,),
            tuple(timings),
        )
    timings.append(
        EnvelopeDebugTimingV1("HOST_ADAPTER", time.perf_counter() - started)
    )

    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    domain_ids = tuple(
        sorted(
            {
                use_by_id[use_id].patch_domain_id
                for use_id in request.selected_chain_use_ids
            },
            key=lambda item: item.value,
        )
    )
    compilations = []
    raw_results = []
    interaction_results = []

    started = time.perf_counter()
    for domain_id in domain_ids:
        compile_result = kernel.compile_reference_envelopes(
            snapshot, request, domain_id
        )
        if compile_result.compilation is None:
            diagnostics.extend(
                _host_diagnostic_from_stage(
                    item.outcome, item.message, domain_id
                )
                for item in compile_result.diagnostics
            )
            continue
        compilations.append(compile_result.compilation)
    timings.append(
        EnvelopeDebugTimingV1("COMPILE", time.perf_counter() - started)
    )

    started = time.perf_counter()
    for compilation in compilations:
        result = kernel.evaluate_reference_raw_coverage(
            compilation, request.requested_alpha
        )
        if result.raw_coverage is None:
            diagnostics.extend(
                _host_diagnostic_from_stage(
                    item.outcome,
                    item.message,
                    compilation.plan_key.patch_domain_id,
                )
                for item in result.diagnostics
            )
            continue
        raw_results.append(result.raw_coverage)
    timings.append(
        EnvelopeDebugTimingV1("RAW_COVERAGE", time.perf_counter() - started)
    )

    compilation_by_domain = {
        item.plan_key.patch_domain_id: item for item in compilations
    }
    started = time.perf_counter()
    for raw in raw_results:
        result = kernel.resolve_coverage_interactions(
            compilation_by_domain[raw.plan_key.patch_domain_id],
            raw.boundary_resolved_envelopes,
            raw,
        )
        interaction_results.append(result)
        if result.resolved_coverage is None:
            diagnostics.extend(
                _host_diagnostic_from_stage(
                    item.outcome,
                    item.message,
                    raw.plan_key.patch_domain_id,
                )
                for item in result.diagnostics
            )
    timings.append(
        EnvelopeDebugTimingV1("INTERACTION", time.perf_counter() - started)
    )

    started = time.perf_counter()
    try:
        scene = kernel.build_envelope_debug_scene(
            snapshot,
            request,
            tuple(compilations),
            tuple(raw_results),
            tuple(interaction_results),
            _scene_diagnostics(kernel, diagnostics),
        )
    except Exception as exc:
        diagnostics.append(
            EnvelopeDebugHostDiagnosticV1(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
                EnvelopeDebugHostSeverity.ERROR,
                f"DebugScene projection failed without fallback: {exc}",
            )
        )
        scene = None
    timings.append(
        EnvelopeDebugTimingV1("DEBUG_SCENE", time.perf_counter() - started)
    )
    return EnvelopeDebugEvaluationV1(
        snapshot,
        request,
        tuple(compilations),
        tuple(raw_results),
        tuple(interaction_results),
        scene,
        tuple(diagnostics),
        tuple(timings),
    )


__all__ = (
    "EnvelopeDebugEvaluationV1",
    "EnvelopeDebugHostDiagnosticV1",
    "EnvelopeDebugHostOutcome",
    "EnvelopeDebugHostSeverity",
    "EnvelopeDebugTimingV1",
    "EnvelopeHostAdapterError",
    "build_envelope_analysis_snapshot",
    "build_envelope_decal_request",
    "evaluate_envelope_debug",
)
